//! A fake [`GpioTrait`] implementation for host-side testing, with no
//! real hardware involved.

use core::cell::Cell;

use crate::api::gpio::{GpioMode, GpioPin, GpioPull, GpioTrait, PinAndPort};

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
/// every port, without touching any real hardware. Only pins named at
/// construction (via [`GpioFake::new`]) are considered "wired up"; calling
/// [`GpioTrait::configure`] on any other pin still works, but warns, since
/// that's almost always a test-setup mistake.
pub struct GpioFake {
    ports: [PortState; NUM_PORTS],
    warning_count: Cell<u32>,
}

impl GpioFake {
    /// Creates a fake chip where only the given pins are considered
    /// wired up. Every pin (registered or not) starts in `Analog` mode
    /// with a low physical level, matching real GPIO reset state.
    pub fn new(pins: &[PinAndPort]) -> Self {
        let fake = GpioFake {
            ports: core::array::from_fn(|_| PortState::new()),
            warning_count: Cell::new(0),
        };
        for p in pins {
            let port = &fake.ports[p.port() as usize];
            let mask = 1u32 << (p.pin_number() as u32);
            port.registered.set(port.registered.get() | mask);
        }
        fake
    }

    /// Number of warnings emitted so far (misuse detected by `configure`
    /// or `set`); mainly useful for tests to assert a warning happened.
    pub fn warning_count(&self) -> u32 {
        self.warning_count.get()
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

    // `PinAndPort::new` is `pub(crate)`, so these tests (being part of
    // this crate) can build one directly without going through a real
    // pin-type token.
    fn pc6() -> PinAndPort {
        PinAndPort::new(GpioPort::PC, 6)
    }

    fn pin(port: GpioPort, pin_number: u8, mode: GpioMode, pull: GpioPull) -> GpioPin {
        GpioPin::new(port, pin_number, mode, pull, GpioSpeed::Low)
    }

    #[test]
    fn starts_up_analog_and_low() {
        let fake = GpioFake::new(&[]);
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::None);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn configure_unregistered_pin_warns() {
        let mut fake = GpioFake::new(&[]);
        fake.configure(pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None));
        assert_eq!(fake.warning_count(), 1);
    }

    #[test]
    fn configure_registered_pin_does_not_warn() {
        let mut fake = GpioFake::new(&[pc6()]);
        fake.configure(pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None));
        assert_eq!(fake.warning_count(), 0);
    }

    #[test]
    fn pull_up_forces_physical_high() {
        let mut fake = GpioFake::new(&[pc6()]);
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Up);
        fake.configure(p);
        assert_eq!(fake.get(p), true);
    }

    #[test]
    fn pull_down_forces_physical_low() {
        let mut fake = GpioFake::new(&[pc6()]);
        fake.configure(pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Up));
        // Reconfiguring with a pull-down should now force it back low.
        let p = pin(GpioPort::PC, 6, GpioMode::Input, GpioPull::Down);
        fake.configure(p);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn set_and_get_round_trip() {
        let mut fake = GpioFake::new(&[pc6()]);
        let p = pin(GpioPort::PC, 6, GpioMode::Output, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.get(p), true);
        fake.set(p, false);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn inverted_output_flips_logical_value() {
        let mut fake = GpioFake::new(&[pc6()]);
        let p = pin(GpioPort::PC, 6, GpioMode::InvertedOutput, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.get(p), true);
    }

    #[test]
    fn set_on_analog_pin_warns_and_does_not_write() {
        let mut fake = GpioFake::new(&[pc6()]);
        let p = pin(GpioPort::PC, 6, GpioMode::Analog, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.warning_count(), 1);
        assert_eq!(fake.get(p), false);
    }

    #[test]
    fn set_on_alternate_mode_pin_warns() {
        let mut fake = GpioFake::new(&[pc6()]);
        let p = pin(GpioPort::PC, 6, GpioMode::AlternateMode, GpioPull::None);
        fake.configure(p);
        fake.set(p, true);
        assert_eq!(fake.warning_count(), 1);
    }

    #[test]
    fn pin_and_port_from_real_pin_token_matches() {
        use crate::stm32g4::gpio::{pin_and_port, PC6};

        assert_eq!(pin_and_port::<PC6>(), pc6());
    }
}
