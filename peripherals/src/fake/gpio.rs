//! A fake [`GpioTrait`] implementation for host-side testing, with no
//! real hardware involved.

use core::cell::Cell;

use crate::api::gpio::{GpioMode, GpioPin, GpioPull, GpioTrait, PinToken};

const NUM_PORTS: usize = 10; // PA..=PJ
const PINS_PER_PORT: usize = 32; // pin_number is a 5-bit value (0..=31)

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
    /// Bitmap of pins passed to `GpioFake::new()`; bit `n` set means pin
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

/// A fake GPIO driver that simulates the physical state of every pin of
/// every port, without touching any real hardware. Only pins claimed via
/// [`GpioFake::claim_pin`] are considered "wired up"; calling
/// [`GpioTrait::configure`] on any other pin still works, but warns, since
/// that's almost always a test-setup mistake.
pub struct GpioFake {
    ports: [PortState; NUM_PORTS],
    warning_count: Cell<u32>,
}

impl GpioFake {
    /// Creates a fake chip with no pins claimed yet. Every pin starts in
    /// `Analog` mode with a low physical level, matching real GPIO reset
    /// state.
    pub fn new() -> Self {
        GpioFake {
            ports: core::array::from_fn(|_| PortState::new()),
            warning_count: Cell::new(0),
        }
    }

    /// Number of warnings emitted so far (misuse detected by `configure`
    /// or `set`); mainly useful for tests to assert a warning happened.
    pub fn warning_count(&self) -> u32 {
        self.warning_count.get()
    }

    /// Claims ownership of a pin-resource handle and registers it as
    /// "wired up" — e.g. a fake `resources::Peri<'static,
    /// resources::peripherals::PAx>` stand-in for a real
    /// `embassy_stm32::Peri`, or a bare pin-token type like
    /// `stm32g4::gpio::PC6`. `T: PinToken` is how the identity to
    /// register is recovered (real `embassy_stm32` pin types can't
    /// implement it without violating Rust's orphan rule, which is why
    /// [`Gpio::claim_pin`](crate::stm32g4::gpio::Gpio::claim_pin) can't do
    /// the same). [`crate::claim_pins!`] calls this once per pin for a
    /// whole list at once.
    pub fn claim_pin<T: PinToken>(&mut self, _pin: T) {
        let port = &self.ports[T::PORT as usize];
        let mask = 1u32 << (T::NUMBER as u32);
        port.registered.set(port.registered.get() | mask);
    }

    fn is_registered(&self, pin: GpioPin) -> bool {
        let port = &self.ports[pin.port() as usize];
        port.registered.get() & (1 << pin.pin_number() as u32) != 0
    }

    fn warn(&self, args: core::fmt::Arguments) {
        self.warning_count.set(self.warning_count.get() + 1);
        emit_warning(args);
    }
}

impl GpioTrait for GpioFake {
    fn configure(&mut self, pin: GpioPin) {
        if !self.is_registered(pin) {
            self.warn(format_args!(
                "configure() called on pin {:?}{} that was never passed to GpioFake::new()",
                pin.port(),
                pin.pin_number()
            ));
        }

        let port = &self.ports[pin.port() as usize];
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

    fn set(&self, pin: GpioPin, value: bool) {
        let port = &self.ports[pin.port() as usize];
        let cell = &port.pins[pin.pin_number() as usize];
        let mut state = cell.get();

        match state.mode {
            GpioMode::AlternateMode | GpioMode::Analog => {
                self.warn(format_args!(
                    "set() called on pin {:?}{} while it is in {:?} mode",
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

    fn get(&self, pin: GpioPin) -> bool {
        let port = &self.ports[pin.port() as usize];
        let state = port.pins[pin.pin_number() as usize].get();
        match state.mode {
            GpioMode::InvertedInput | GpioMode::InvertedOutput => !state.physical_high,
            _ => state.physical_high,
        }
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
    use crate::api::gpio::{GpioPort, GpioSpeed};
    use crate::stm32g4::gpio::PC6;

    fn pin(port: GpioPort, pin_number: u8, mode: GpioMode, pull: GpioPull) -> GpioPin {
        GpioPin::new(port, pin_number, mode, pull, GpioSpeed::Low)
    }

    #[test]
    fn starts_up_analog_and_low() {
        let fake = GpioFake::new();
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::None);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn configure_unregistered_pin_warns() {
        let mut fake = GpioFake::new();
        fake.configure(pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None));
        assert_eq!(fake.warning_count(), 1);
    }

    #[test]
    fn configure_registered_pin_does_not_warn() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        fake.configure(pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None));
        assert_eq!(fake.warning_count(), 0);
    }

    #[test]
    fn pull_up_forces_physical_high() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Up);
        fake.configure(p);
        assert_eq!(fake.get(p), true);
    }

    #[test]
    fn pull_down_forces_physical_low() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        fake.configure(pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Up));
        // Reconfiguring with a pull-down should now force it back low.
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Down);
        fake.configure(p);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn set_and_get_round_trip() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        let p = pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.get(p), true);
        fake.set(p, false);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn inverted_output_flips_logical_value() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        let p = pin(GpioPort::PC, 6, GpioMode::InvertedOutput, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.get(p), true);
    }

    #[test]
    fn set_on_analog_pin_warns_and_does_not_write() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        let p = pin(GpioPort::PC, 6, GpioMode::Analog, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.warning_count(), 1);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn set_on_alternate_mode_pin_warns() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        let p = pin(GpioPort::PC, 6, GpioMode::AlternateMode, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.warning_count(), 1);
    }

    #[test]
    fn claim_pin_registers_the_pin_token_gave() {
        let mut fake = GpioFake::new();
        fake.claim_pin(PC6);
        // PB0 was never claimed, so it should still warn.
        fake.configure(pin(GpioPort::PB, 0, GpioMode::Output, GpioPull::None));
        assert_eq!(fake.warning_count(), 1);
    }
}
