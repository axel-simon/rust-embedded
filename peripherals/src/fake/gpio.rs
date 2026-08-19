//! A fake [`GpioTrait`] implementation for host-side testing, with no
//! real hardware involved.
//!
//! [`Gpio::new`] hands back two handles onto one shared, simulated chip:
//! [`Gpio`] itself, for firmware — its [`GpioTrait::set`] mirrors real
//! hardware (see [`crate::stm32g4::gpio::Gpio::set`]), only writing
//! `Output`/`InvertedOutput` pins — and [`FakeGpio`], for a test to
//! observe what firmware did, or to force a pin's state regardless of
//! mode (simulating external hardware, e.g. a button press on an input
//! pin).

use core::cell::Cell;
use std::rc::Rc;

use crate::api::gpio::{GpioMode, GpioPin, GpioPort, GpioPull, GpioTrait};

const NUM_PORTS: usize = 10; // PA..=PJ
const PINS_PER_PORT: usize = 256; // pin_number is a u8.

/// Implemented by any type that knows its own physical GPIO identity —
/// e.g. a test-local pin-identity type, or (via a blanket impl over its
/// inner type) a `Peri`-style ownership wrapper around one, like
/// `boards/resources`'s fake `Peri<'d, T>`. [`Gpio::claim_pin`] uses this
/// to register a claimed pin.
///
/// The real driver ([`crate::stm32g4::gpio::Gpio::claim_pin`]) can't do
/// the same: it's called with real values from whichever embassy-X
/// backend a board uses (e.g. `embassy_stm32::Peri<'static,
/// embassy_stm32::peripherals::PAx>`), and no crate in this workspace is
/// allowed to implement this trait for those — `PinToken` is foreign to
/// whichever crate would want to write that `impl` (it's defined here, in
/// `peripherals`), and so are the backend's own pin types (e.g. defined in
/// `embassy_stm32`), which is exactly what Rust's orphan rule forbids.
/// Ownership (a plain move of `pin`) is the only bookkeeping available
/// there instead.
pub trait PinToken {
    const PORT: GpioPort;
    const NUMBER: u8;
}

#[derive(Clone, Copy)]
struct PinState {
    mode: GpioMode,
    physical_high: bool,
}

impl Default for PinState {
    fn default() -> Self {
        PinState {
            mode: GpioMode::Analog,
            physical_high: false,
        }
    }
}

struct PortState {
    pins: [Cell<PinState>; PINS_PER_PORT],
    /// Bitmap of pins passed to [`Gpio::claim_pin`]; bit `n` set means pin
    /// `n` of this port was registered.
    registered: Cell<u32>,
}

impl PortState {
    fn new() -> Self {
        PortState {
            pins: core::array::from_fn(|_| Cell::new(PinState::default())),
            registered: Cell::new(0),
        }
    }
}

/// The simulated chip state shared between a [`Gpio`] and its
/// [`FakeGpio`] counterpart (see [`Gpio::new`]).
struct GpioState {
    ports: [PortState; NUM_PORTS],
}

impl GpioState {
    fn is_registered(&self, pin: GpioPin) -> bool {
        let port = &self.ports[pin.port() as usize];
        port.registered.get() & (1 << pin.pin_number() as u32) != 0
    }

    fn warn(&self, args: core::fmt::Arguments) {
        emit_warning(args);
    }

    /// Shared by [`GpioTrait::get`] and [`FakeGpio::get`] — reading a
    /// pin's state never depends on which side is asking.
    fn get(&self, pin: GpioPin) -> bool {
        let port = &self.ports[pin.port() as usize];
        let state = port.pins[pin.pin_number() as usize].get();
        match state.mode {
            GpioMode::InvertedInput | GpioMode::InvertedOutput => !state.physical_high,
            _ => state.physical_high,
        }
    }
}

/// A fake GPIO driver that simulates the physical state of every pin of
/// every port, without touching any real hardware. Only pins claimed via
/// [`Gpio::claim_pin`] are considered "wired up"; calling
/// [`GpioTrait::configure`] on any other pin still works, but warns, since
/// that's almost always a test-setup mistake. Deliberately not `Clone`,
/// matching the real driver's own (see `crate::stm32g4::gpio::Gpio`'s
/// doc comment) — see `mod app`'s own `Shared::gpio` (in
/// `firmware/benchtest/motor_control`) for how more than one RTIC task
/// safely shares the single instance a board constructs instead.
pub struct Gpio(Rc<GpioState>);

/// A test's handle onto the same simulated chip a [`Gpio`] drives — see
/// [`Gpio::new`]. Cheap to [`Clone`] (all clones share the same
/// underlying state).
#[derive(Clone)]
pub struct FakeGpio(Rc<GpioState>);

impl Gpio {
    /// Creates a fake chip with no pins claimed yet, and a [`FakeGpio`]
    /// handle onto the same simulated state. Every pin starts in `Analog`
    /// mode with a low physical level, matching real GPIO reset state.
    pub fn new() -> (Self, FakeGpio) {
        let state = Rc::new(GpioState {
            ports: core::array::from_fn(|_| PortState::new()),
        });
        (Gpio(state.clone()), FakeGpio(state))
    }

    /// Claims ownership of a pin-resource handle and registers it as
    /// "wired up" — e.g. a fake `resources::Peri<'static,
    /// resources::peripherals::PAx>` stand-in for a real embassy-X
    /// backend's `Peri` (e.g. `embassy_stm32::Peri`), or a bare test-local
    /// pin-identity type. `T: PinToken` is how the identity to register is
    /// recovered (the real backend's own pin types, e.g. `embassy_stm32`'s,
    /// can't implement it without violating Rust's orphan rule, which is
    /// why [`Gpio::claim_pin`](crate::stm32g4::gpio::Gpio::claim_pin) can't
    /// do the same). [`crate::claim_pins!`] calls this once per pin for a
    /// whole list at once.
    pub fn claim_pin<T: PinToken>(&mut self, _pin: T) {
        let port = &self.0.ports[T::PORT as usize];
        let mask = 1u32 << (T::NUMBER as u32);
        port.registered.set(port.registered.get() | mask);
    }
}

impl GpioTrait for Gpio {
    fn configure(&mut self, pin: GpioPin) {
        if !self.0.is_registered(pin) {
            self.0.warn(format_args!(
                "configure() called on pin {:?}{} that was never claimed via Gpio::claim_pin()",
                pin.port(),
                pin.pin_number()
            ));
        }

        let port = &self.0.ports[pin.port() as usize];
        let cell = &port.pins[pin.pin_number() as usize];
        let mut state = cell.get();
        state.mode = pin.mode();
        match pin.pull() {
            // A pull-up idles the pin high; a pull-down idles it low.
            // `None` leaves whatever physical level was already there.
            GpioPull::Up => state.physical_high = true,
            GpioPull::Down => state.physical_high = false,
            GpioPull::None => {}
        }
        cell.set(state);
    }

    /// Mirrors [`crate::stm32g4::gpio::Gpio::set`]'s real-hardware
    /// behavior: only `Output`/`InvertedOutput` pins are writable — every
    /// other mode is a no-op (and, since the fake can log where real
    /// hardware can't, warns). To simulate external hardware driving a
    /// pin regardless of mode (e.g. a button press on an input pin), use
    /// [`FakeGpio::set`] instead.
    fn set(&self, pin: GpioPin, value: bool) {
        let port = &self.0.ports[pin.port() as usize];
        let cell = &port.pins[pin.pin_number() as usize];
        let mut state = cell.get();

        let physical = match state.mode {
            GpioMode::Output => value,
            GpioMode::InvertedOutput => !value,
            _ => {
                self.0.warn(format_args!(
                    "set() called on pin {:?}{} while it is in {:?} mode (not Output/InvertedOutput)",
                    pin.port(),
                    pin.pin_number(),
                    state.mode
                ));
                return;
            }
        };
        state.physical_high = physical;
        cell.set(state);
    }

    fn get(&self, pin: GpioPin) -> bool {
        self.0.get(pin)
    }
}

impl FakeGpio {
    /// Forces the pin's simulated physical state directly, regardless of
    /// its configured mode — simulating external hardware driving the pin
    /// (e.g. a button press on an input pin). Unlike [`Gpio::set`]
    /// (`GpioTrait::set`), which only works on `Output`/`InvertedOutput`
    /// pins, this only refuses `Analog`/`AlternateMode` pins (which have
    /// no digital state to force either way).
    pub fn set(&self, pin: GpioPin, value: bool) {
        let port = &self.0.ports[pin.port() as usize];
        let cell = &port.pins[pin.pin_number() as usize];
        let mut state = cell.get();

        match state.mode {
            GpioMode::AlternateMode | GpioMode::Analog => {
                self.0.warn(format_args!(
                    "FakeGpio::set() called on pin {:?}{} while it is in {:?} mode",
                    pin.port(),
                    pin.pin_number(),
                    state.mode
                ));
                return;
            }
            GpioMode::InvertedInput | GpioMode::InvertedOutput => {
                state.physical_high = !value;
            }
            GpioMode::Input | GpioMode::Output => {
                state.physical_high = value;
            }
        }
        cell.set(state);
    }

    /// Reads the pin's current logical state, regardless of mode.
    pub fn get(&self, pin: GpioPin) -> bool {
        self.0.get(pin)
    }

    /// Whether `pin` was claimed via [`Gpio::claim_pin`] — the same check
    /// `GpioTrait::configure` makes internally to decide whether to warn.
    /// Lets a test verify claiming happened for exactly the pins it
    /// expects, without going through `configure`.
    pub fn pin_registered(&self, pin: GpioPin) -> bool {
        self.0.is_registered(pin)
    }
}

// The embedded target this crate normally builds for has no logging
// backend wired up yet (that'll come with the real stm32g4 driver), so
// warnings are only surfaced on the host, where `cfg(test)` builds have
// `std` available. See peripherals/README.md.
#[cfg(test)]
fn emit_warning(args: core::fmt::Arguments) {
    eprintln!("gpio fake warning: {args}");
}

#[cfg(not(test))]
fn emit_warning(_args: core::fmt::Arguments) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::gpio::GpioSpeed;

    fn pin(port: GpioPort, pin_number: u8, mode: GpioMode, pull: GpioPull) -> GpioPin {
        GpioPin::new(port, pin_number, mode, pull, GpioSpeed::Low)
    }

    /// An arbitrary pin identity for tests to claim — its port/number
    /// don't matter, only that it's consistent across a test.
    #[derive(Clone, Copy)]
    struct TestPin;

    impl PinToken for TestPin {
        const PORT: GpioPort = GpioPort::PC;
        const NUMBER: u8 = 6;
    }

    #[test]
    fn starts_up_analog_and_low() {
        let (_gpio, fake) = Gpio::new();
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::None);
        assert!(!fake.get(p));
    }

    #[test]
    fn pin_starts_unregistered() {
        let (_gpio, fake) = Gpio::new();
        let p = pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None);
        assert!(!fake.pin_registered(p));
    }

    #[test]
    fn claim_pin_registers_it() {
        let (mut gpio, fake) = Gpio::new();
        let p = pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None);
        gpio.claim_pin(TestPin);
        assert!(fake.pin_registered(p));
    }

    #[test]
    fn pull_up_forces_physical_high() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Up);
        gpio.configure(p);
        assert!(fake.get(p));
    }

    #[test]
    fn pull_down_forces_physical_low() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        gpio.configure(pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Up));
        // Reconfiguring with a pull-down should now force it back low.
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Down);
        gpio.configure(p);
        assert!(!fake.get(p));
    }

    #[test]
    fn set_and_get_round_trip() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None);
        gpio.configure(p);
        gpio.set(p, true);
        assert!(fake.get(p));
        gpio.set(p, false);
        assert!(!fake.get(p));
    }

    #[test]
    fn inverted_output_flips_logical_value() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::InvertedOutput, GpioPull::None);
        gpio.configure(p);
        gpio.set(p, true);
        assert!(fake.get(p));
    }

    #[test]
    fn set_on_analog_pin_does_not_write() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::Analog, GpioPull::None);
        gpio.configure(p);
        gpio.set(p, true);
        assert!(!fake.get(p));
    }

    #[test]
    fn set_on_alternate_mode_pin_does_not_write() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::AlternateMode, GpioPull::None);
        gpio.configure(p);
        gpio.set(p, true);
        assert!(!fake.get(p));
    }

    #[test]
    fn set_on_input_pin_does_not_write() {
        // Mirrors real hardware (`stm32g4::gpio::Gpio::set` returns early
        // for any non-Output/InvertedOutput mode) — `GpioTrait::set` isn't
        // meant to be usable on an input pin at all; use `FakeGpio::set`
        // to simulate external hardware driving one instead.
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::None);
        gpio.configure(p);
        gpio.set(p, true);
        assert!(!fake.get(p));
    }

    #[test]
    fn claim_pin_registers_only_the_pin_token_given() {
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        assert!(fake.pin_registered(pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None)));
        // PB0 was never claimed.
        assert!(!fake.pin_registered(pin(GpioPort::PB, 0, GpioMode::Output, GpioPull::None)));
    }

    #[test]
    fn fake_gpio_set_forces_state_regardless_of_mode() {
        // Simulates external hardware driving an input pin (e.g. a
        // button press) — something `GpioTrait::set` deliberately can't
        // do (see `set_on_input_pin_does_not_write`).
        let (mut gpio, fake) = Gpio::new();
        gpio.claim_pin(TestPin);
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::None);
        gpio.configure(p);
        fake.set(p, true);
        assert!(fake.get(p));
    }
}
