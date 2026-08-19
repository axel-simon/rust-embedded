//! A register-level [`GpioTrait`](crate::api::gpio::GpioTrait) driver for
//! a real STM32G4 chip, backed by `stm32-metapac`.
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs`: it depends on `stm32-metapac`,
//! which is only pulled in as a dependency for that target (see
//! Cargo.toml), so host-side `cargo test` (see peripherals/README.md)
//! never compiles it.

use crate::api::gpio::{GpioMode, GpioPin, GpioPort, GpioPull, GpioSpeed, GpioTrait};
use stm32_metapac::gpio::vals;

/// Register-level [`GpioTrait`] driver for a real STM32G4 chip, backed by
/// `stm32-metapac`. Deliberately not `Clone`/`Copy`, even though it's
/// zero-sized: [`GpioTrait::configure`] needs `&mut self`, and cloning
/// this type would let two independent owners each obtain their own
/// exclusive `&mut` and race on the same MMIO registers if `configure()`
/// were ever called through both — see `mod app`'s own `Shared::gpio`
/// (in `firmware/benchtest/motor_control`) for how more than one RTIC
/// task safely shares the single instance a board constructs instead
/// (via `Mutex::lock`, not by duplicating the value).
pub struct Gpio;

impl Gpio {
    /// This driver does no bookkeeping of its own — it stays a
    /// zero-sized type; register access is stateless port/pin math, see
    /// `gpio_block`. Enables every GPIO port's clock (see
    /// [`enable_gpio_clocks`]) so `configure`/`set`/`get`'s register
    /// writes aren't silently ineffective.
    pub fn new() -> Self {
        enable_gpio_clocks();
        Gpio
    }

    /// Claims ownership of a pin-resource handle — typically a real
    /// `embassy_stm32::Peri<'static, embassy_stm32::peripherals::PAx>`.
    /// Purely a Rust move: `pin` is dropped immediately and this driver
    /// does nothing else with it (unlike
    /// [`Gpio::claim_pin`](crate::fake::gpio::Gpio::claim_pin), which uses
    /// [`PinToken`](crate::fake::gpio::PinToken) to actually register the
    /// pin — real `embassy_stm32` pin types can't implement that trait
    /// without violating Rust's orphan rule, so this side can't do the
    /// same bookkeeping). Its only effect is preventing `pin` from being
    /// independently claimed and used elsewhere (e.g. through
    /// `embassy_stm32`'s own pin API) — register access via
    /// `configure`/`set`/`get` is driven entirely by the `GpioPin` config
    /// value, unrelated to whatever claimed the underlying physical pin.
    /// [`crate::claim_pins!`] calls this once per pin for a whole list at
    /// once.
    pub fn claim_pin<T>(&mut self, _pin: T) {}
}

impl Default for Gpio {
    fn default() -> Self {
        Self::new()
    }
}

impl GpioTrait for Gpio {
    fn configure(&mut self, pin: GpioPin) {
        let n = pin.pin_number() as usize;
        let r = gpio_block(pin.port());

        let moder = match pin.mode() {
            GpioMode::Input | GpioMode::InvertedInput => vals::Moder::INPUT,
            GpioMode::Output | GpioMode::InvertedOutput => vals::Moder::OUTPUT,
            GpioMode::AlternateMode => vals::Moder::ALTERNATE,
            GpioMode::Analog => vals::Moder::ANALOG,
        };
        let ospeedr = match pin.speed() {
            GpioSpeed::Low => vals::Ospeedr::LOW_SPEED,
            GpioSpeed::Medium => vals::Ospeedr::MEDIUM_SPEED,
            GpioSpeed::High => vals::Ospeedr::HIGH_SPEED,
            GpioSpeed::VeryHigh => vals::Ospeedr::VERY_HIGH_SPEED,
        };
        let pupdr = match pin.pull() {
            GpioPull::None => vals::Pupdr::FLOATING,
            GpioPull::Up => vals::Pupdr::PULL_UP,
            GpioPull::Down => vals::Pupdr::PULL_DOWN,
        };

        // Drive the pull-implied physical level via BSRR before flipping
        // MODER to output, so an output pin never glitches through the
        // opposite level on its way to its configured idle state.
        if matches!(pin.mode(), GpioMode::Output | GpioMode::InvertedOutput) {
            if pin.pull() == GpioPull::Up {
                r.bsrr().write(|w| w.set_bs(n, true));
            } else {
                r.bsrr().write(|w| w.set_br(n, true));
            }
        }

        // Route the alternate function before flipping MODER to Alternate,
        // for the same glitch-avoidance reason as the BSRR write above.
        if pin.mode() == GpioMode::AlternateMode {
            r.afr(n / 8)
                .modify(|w| w.set_afr(n % 8, pin.alternate_function()));
        }

        r.otyper().modify(|w| w.set_ot(n, vals::Ot::PUSH_PULL));
        r.ospeedr().modify(|w| w.set_ospeedr(n, ospeedr));
        r.pupdr().modify(|w| w.set_pupdr(n, pupdr));
        r.moder().modify(|w| w.set_moder(n, moder));
    }

    fn set(&self, pin: GpioPin, logical: bool) {
        let physical = match pin.mode() {
            GpioMode::Output => logical,
            GpioMode::InvertedOutput => !logical,
            _ => return,
        };
        let n = pin.pin_number() as usize;
        let r = gpio_block(pin.port());
        // BSRR: a single volatile write that atomically sets or clears
        // exactly this pin's bit. Every other bit in the write is 0, which
        // BSRR defines as a no-op, so this can never race with (or
        // read-modify-write clobber) another pin on the same port.
        if physical {
            r.bsrr().write(|w| w.set_bs(n, true));
        } else {
            r.bsrr().write(|w| w.set_br(n, true));
        }
    }

    fn get(&self, pin: GpioPin) -> bool {
        let n = pin.pin_number() as usize;
        let r = gpio_block(pin.port());
        // IDR: a single volatile read of this pin's live input level.
        let physical_high = r.idr().read().idr(n) == vals::Idr::HIGH;
        match pin.mode() {
            GpioMode::InvertedInput | GpioMode::InvertedOutput => !physical_high,
            _ => physical_high,
        }
    }
}

/// Enables the AHB2 clock for every GPIO port that exists on the STM32G431
/// (A..G). `Gpio::configure`/`set`/`get` poke GPIOx registers directly,
/// bypassing whatever peripheral-clock bookkeeping a higher-level HAL or
/// `embassy_stm32::init()` might otherwise do on your behalf (they only
/// enable a port's clock when *their own* pin API claims that port), so
/// without this, register writes to an unclocked GPIO port are silently
/// ineffective. Idempotent; safe to call more than once, and safe to call
/// even for ports this board doesn't use. Called from [`Gpio::new`], not
/// meant to be called independently of constructing a [`Gpio`].
fn enable_gpio_clocks() {
    stm32_metapac::RCC.ahb2enr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_gpioden(true);
        w.set_gpioeen(true);
        w.set_gpiofen(true);
        w.set_gpiogen(true);
    });
}

/// Maps a [`GpioPort`] to its `stm32-metapac` register block.
///
/// STM32G431 exposes GPIOA..GPIOG; `GpioPort::PH`/`PI`/`PJ` exist for
/// forward compatibility with larger G4 parts that do have those ports,
/// but no board using this crate ever constructs a `GpioPin` on one (this
/// chip has no such physical pins), so `pin.port()` can never actually be
/// `PH`/`PI`/`PJ` at runtime.
///
/// `pub(crate)` so [`crate::stm32g4::quadrature`] can read a plain digital
/// pin's live level (its `zero_input`) the same stateless way
/// [`Gpio::get`] does, without needing to hold (or be passed) a `&Gpio`.
pub(crate) fn gpio_block(port: GpioPort) -> stm32_metapac::gpio::Gpio {
    match port {
        GpioPort::PA => stm32_metapac::GPIOA,
        GpioPort::PB => stm32_metapac::GPIOB,
        GpioPort::PC => stm32_metapac::GPIOC,
        GpioPort::PD => stm32_metapac::GPIOD,
        GpioPort::PE => stm32_metapac::GPIOE,
        GpioPort::PF => stm32_metapac::GPIOF,
        GpioPort::PG => stm32_metapac::GPIOG,
        GpioPort::PH | GpioPort::PI | GpioPort::PJ => {
            unreachable!("STM32G431 has no GPIO{:?} port", port)
        }
    }
}
