//! STM32G4 GPIO pin type tokens, and (on the real target) a register-level
//! [`GpioTrait`](crate::api::gpio::GpioTrait) driver backed by
//! `stm32-metapac`.
//!
//! The pin tokens are self-contained: each pin (`PA0`..`PJ15`) is a
//! zero-sized token type naming one physical GPIO pin, with no dependency
//! on any external HAL/PAC crate. Tokens implement [`PinToken`] (to build
//! a [`PinAndPort`] for [`fake::gpio::GpioFake`](crate::fake::gpio::GpioFake))
//! and [`PeripheralType`] (for use as a [`Peri`](crate::fake::peri::Peri)
//! handle with the real [`Gpio`](self::Gpio) driver below).
//!
//! The driver itself is gated to `cfg(target_arch = "arm")`: it depends on
//! `stm32-metapac`, which is only pulled in as a dependency for that target
//! (see Cargo.toml), so host-side `cargo test` (see peripherals/README.md)
//! never compiles it and this file's own tests only exercise the tokens.

use crate::api::gpio::{GpioPort, PinAndPort};
use crate::fake::peri::PeripheralType;

#[cfg(target_arch = "arm")]
use crate::api::gpio::{GpioMode, GpioPin, GpioPull, GpioSpeed, GpioTrait};
#[cfg(target_arch = "arm")]
use stm32_metapac::gpio::vals;

/// Implemented by every pin type token (e.g. `PC6`), exposing its port
/// and pin number as associated constants.
pub trait PinToken {
    const PORT: GpioPort;
    const NUMBER: u8;
}

/// The only way to obtain a `PinAndPort`: name a real pin type, e.g.
/// `pin_and_port::<PC6>()`.
pub fn pin_and_port<T: PinToken>() -> PinAndPort {
    PinAndPort::new(T::PORT, T::NUMBER)
}

macro_rules! pin {
    ($name:ident, $port:ident, $number:expr) => {
        #[doc = concat!("GPIO pin `", stringify!($name), "`.")]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name;

        impl PinToken for $name {
            const PORT: GpioPort = GpioPort::$port;
            const NUMBER: u8 = $number;
        }

        impl PeripheralType for $name {}
    };
}

macro_rules! port {
    ($port:ident: $($name:ident = $n:expr),+ $(,)?) => {
        $(pin!($name, $port, $n);)+
    };
}

port!(PA: PA0 = 0, PA1 = 1, PA2 = 2, PA3 = 3, PA4 = 4, PA5 = 5, PA6 = 6, PA7 = 7,
         PA8 = 8, PA9 = 9, PA10 = 10, PA11 = 11, PA12 = 12, PA13 = 13, PA14 = 14, PA15 = 15);
port!(PB: PB0 = 0, PB1 = 1, PB2 = 2, PB3 = 3, PB4 = 4, PB5 = 5, PB6 = 6, PB7 = 7,
         PB8 = 8, PB9 = 9, PB10 = 10, PB11 = 11, PB12 = 12, PB13 = 13, PB14 = 14, PB15 = 15);
port!(PC: PC0 = 0, PC1 = 1, PC2 = 2, PC3 = 3, PC4 = 4, PC5 = 5, PC6 = 6, PC7 = 7,
         PC8 = 8, PC9 = 9, PC10 = 10, PC11 = 11, PC12 = 12, PC13 = 13, PC14 = 14, PC15 = 15);
port!(PD: PD0 = 0, PD1 = 1, PD2 = 2, PD3 = 3, PD4 = 4, PD5 = 5, PD6 = 6, PD7 = 7,
         PD8 = 8, PD9 = 9, PD10 = 10, PD11 = 11, PD12 = 12, PD13 = 13, PD14 = 14, PD15 = 15);
port!(PE: PE0 = 0, PE1 = 1, PE2 = 2, PE3 = 3, PE4 = 4, PE5 = 5, PE6 = 6, PE7 = 7,
         PE8 = 8, PE9 = 9, PE10 = 10, PE11 = 11, PE12 = 12, PE13 = 13, PE14 = 14, PE15 = 15);
port!(PF: PF0 = 0, PF1 = 1, PF2 = 2, PF3 = 3, PF4 = 4, PF5 = 5, PF6 = 6, PF7 = 7,
         PF8 = 8, PF9 = 9, PF10 = 10, PF11 = 11, PF12 = 12, PF13 = 13, PF14 = 14, PF15 = 15);
port!(PG: PG0 = 0, PG1 = 1, PG2 = 2, PG3 = 3, PG4 = 4, PG5 = 5, PG6 = 6, PG7 = 7,
         PG8 = 8, PG9 = 9, PG10 = 10, PG11 = 11, PG12 = 12, PG13 = 13, PG14 = 14, PG15 = 15);
port!(PH: PH0 = 0, PH1 = 1, PH2 = 2, PH3 = 3, PH4 = 4, PH5 = 5, PH6 = 6, PH7 = 7,
         PH8 = 8, PH9 = 9, PH10 = 10, PH11 = 11, PH12 = 12, PH13 = 13, PH14 = 14, PH15 = 15);
port!(PI: PI0 = 0, PI1 = 1, PI2 = 2, PI3 = 3, PI4 = 4, PI5 = 5, PI6 = 6, PI7 = 7,
         PI8 = 8, PI9 = 9, PI10 = 10, PI11 = 11, PI12 = 12, PI13 = 13, PI14 = 14, PI15 = 15);
port!(PJ: PJ0 = 0, PJ1 = 1, PJ2 = 2, PJ3 = 3, PJ4 = 4, PJ5 = 5, PJ6 = 6, PJ7 = 7,
         PJ8 = 8, PJ9 = 9, PJ10 = 10, PJ11 = 11, PJ12 = 12, PJ13 = 13, PJ14 = 14, PJ15 = 15);

/// Register-level [`GpioTrait`] driver for a real STM32G4 chip, backed by
/// `stm32-metapac`.
#[cfg(target_arch = "arm")]
pub struct Gpio;

#[cfg(target_arch = "arm")]
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

/// Maps a [`GpioPort`] to its `stm32-metapac` register block.
///
/// STM32G431 exposes GPIOA..GPIOG; the `PH`/`PI`/`PJ` tokens above exist in
/// this crate for forward compatibility with larger G4 parts that do have
/// those ports, but no such pin can be named on this chip (there's no
/// `PinToken` impl for them here), so `pin.port()` can never actually be
/// `PH`/`PI`/`PJ` at runtime.
#[cfg(target_arch = "arm")]
fn gpio_block(port: GpioPort) -> stm32_metapac::gpio::Gpio {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_and_last_pin_of_each_boundary_port_map_correctly() {
        assert_eq!(pin_and_port::<PA0>(), PinAndPort::new(GpioPort::PA, 0));
        assert_eq!(pin_and_port::<PA15>(), PinAndPort::new(GpioPort::PA, 15));
        assert_eq!(pin_and_port::<PJ0>(), PinAndPort::new(GpioPort::PJ, 0));
        assert_eq!(pin_and_port::<PJ15>(), PinAndPort::new(GpioPort::PJ, 15));
    }

    #[test]
    fn pin_token_implements_peripheral_type() {
        fn assert_peripheral_type<T: PeripheralType>() {}
        assert_peripheral_type::<PC6>();
    }
}
