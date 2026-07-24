//! Hardware-agnostic GPIO API. Concrete drivers (real or fake) implement
//! [`GpioTrait`]; nothing in this module depends on any particular chip.

/// Output drive strength / slew rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioSpeed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

/// GPIO port identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioPort {
    PA = 0,
    PB = 1,
    PC = 2,
    PD = 3,
    PE = 4,
    PF = 5,
    PG = 6,
    PH = 7,
    PI = 8,
    PJ = 9,
}

/// Pull-up / pull-down resistor configuration. The Up / Down direction relates
/// to the physical pin level rather than the logical value, i.e. a
/// disconnected pin configured with Pull::Up will read as low (false) if the
/// pin mode is InvertedInput or InvertedOutput.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioPull {
    None = 0,
    Up = 1,
    Down = 2,
}

/// Pin direction / function. The `Inverted*` variants flip the logical
/// value seen through [`GpioTrait::set`] / [`GpioTrait::get`] relative to
/// the physical pin level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioMode {
    Input = 0,
    InvertedInput = 1,
    Output = 2,
    InvertedOutput = 3,
    AlternateMode = 4,
    Analog = 5,
}

const PIN_NUMBER_BITS: u32 = 8;
const PORT_BITS: u32 = 4;
const MODE_BITS: u32 = 3;
const PULL_BITS: u32 = 2;
const SPEED_BITS: u32 = 2;
const AF_BITS: u32 = 4;

const PIN_NUMBER_SHIFT: u32 = 0;
const PORT_SHIFT: u32 = PIN_NUMBER_SHIFT + PIN_NUMBER_BITS;
const MODE_SHIFT: u32 = PORT_SHIFT + PORT_BITS;
const PULL_SHIFT: u32 = MODE_SHIFT + MODE_BITS;
const SPEED_SHIFT: u32 = PULL_SHIFT + PULL_BITS;
const AF_SHIFT: u32 = SPEED_SHIFT + SPEED_BITS;

const PIN_NUMBER_MASK: u32 = (1 << PIN_NUMBER_BITS) - 1;
const PORT_MASK: u32 = (1 << PORT_BITS) - 1;
const MODE_MASK: u32 = (1 << MODE_BITS) - 1;
const PULL_MASK: u32 = (1 << PULL_BITS) - 1;
const SPEED_MASK: u32 = (1 << SPEED_BITS) - 1;
const AF_MASK: u32 = (1 << AF_BITS) - 1;

/// A fully-specified GPIO pin configuration, bit-packed so that the struct
/// fits into a single MCU register.
///
/// When `GpioPin` contains not only a pin's identity, but also the
/// configuration of a GPIO. In particular, when a GPIO is used as a plain
/// input/output, it tracks if the physical level is inverted relative to the
/// logical interpretation. An wire called nRST should be called RST and
/// created through an inverted_output(). Calling `gpio.set(RST, true)` will
/// then output a 0V voltage.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct GpioPin(u32);

const _: () = assert!(core::mem::size_of::<GpioPin>() == 4);

impl GpioPin {
    /// General-purpose constructor. Prefer
    /// [`GpioPin::output`],[`GpioPin::inverted_output`],
    /// [`GpioPin::input`], [`GpioPin::inverted_input`],
    /// [`GpioPin::alternate`], or [`GpioPin::analog`] where they fit.
    pub const fn new(
        port: GpioPort,
        pin_number: u8,
        mode: GpioMode,
        pull: GpioPull,
        speed: GpioSpeed,
    ) -> Self {
        Self::packed(port, pin_number, mode, pull, speed, 0)
    }

    /// A push-pull digital output pin, with no pull resistor and low
    /// speed. Chain [`with_pull_up`](Self::with_pull_up),
    /// [`with_pull_down`](Self::with_pull_down), or one of the
    /// `with_*_speed` builder methods to override either. Use
    /// [`GpioPin::new`] directly for `InvertedOutput`.
    pub const fn output(port: GpioPort, pin_number: u8) -> Self {
        Self::new(
            port,
            pin_number,
            GpioMode::Output,
            GpioPull::None,
            GpioSpeed::Low,
        )
    }
    // A push-pull digital output pin whose physical level is inverted relative to
    // the logical value seen through [`GpioTrait::set`] / [`GpioTrait::get`].
    // Chain [`with_pull_up`](Self::with_pull_up) or
    // [`with_pull_down`](Self::with_pull_down) to enable a pull-up or pull-down
    // resistor on the *physical* pin.
    pub const fn inverted_output(port: GpioPort, pin_number: u8) -> Self {
        Self::new(
            port,
            pin_number,
            GpioMode::InvertedOutput,
            GpioPull::None,
            GpioSpeed::Low,
        )
    }

    /// An analog pin: an ADC input, DAC output, comparator/OPAMP terminal,
    /// or an unused/not-connected pin parked in the lowest-power
    /// reset-safe state. Defaults to no pull resistor.
    pub const fn analog(port: GpioPort, pin_number: u8) -> Self {
        Self::new(
            port,
            pin_number,
            GpioMode::Analog,
            GpioPull::None,
            GpioSpeed::Low,
        )
    }

    /// A digital input pin, with no pull resistor. Chain
    /// [`with_pull_up`](Self::with_pull_up) or
    /// [`with_pull_down`](Self::with_pull_down) to override.
    pub const fn input(port: GpioPort, pin_number: u8) -> Self {
        Self::new(
            port,
            pin_number,
            GpioMode::Input,
            GpioPull::None,
            GpioSpeed::Low,
        )
    }
    // A digital input pin whose physical level is inverted relative to the logical
    // value seen through [`GpioTrait::set`] / [`GpioTrait::get`]. Chain
    // [`with_pull_up`](Self::with_pull_up) or
    // [`with_pull_down`](Self::with_pull_down) to enable a pull-up or pull-down
    // resistor on the *physical* pin.
    pub const fn inverted_input(port: GpioPort, pin_number: u8) -> Self {
        Self::new(
            port,
            pin_number,
            GpioMode::InvertedInput,
            GpioPull::None,
            GpioSpeed::Low,
        )
    }

    /// A pin routed through one of the chip's alternate functions (a timer
    /// channel, USART, CAN controller, etc), with no pull resistor and low
    /// speed. `af` is the alternate-function index (0..=15) to write into
    /// this pin's AFRL/AFRH nibble, taken from the chip's alternate-function
    /// table. Chain [`with_pull_up`](Self::with_pull_up),
    /// [`with_pull_down`](Self::with_pull_down), or one of the
    /// `with_*_speed` builder methods to override either.
    ///
    /// # Panics
    /// Panics if `af` doesn't fit in 4 bits (i.e. is greater than 15).
    pub const fn alternate(port: GpioPort, pin_number: u8, af: u8) -> Self {
        assert!((af as u32) <= AF_MASK, "af must fit in 4 bits");
        Self::packed(
            port,
            pin_number,
            GpioMode::AlternateMode,
            GpioPull::None,
            GpioSpeed::Low,
            af,
        )
    }

    /// Overrides this pin's pull resistor to pull-up.
    pub const fn with_pull_up(self) -> Self {
        self.with_pull(GpioPull::Up)
    }

    /// Overrides this pin's pull resistor to pull-down.
    pub const fn with_pull_down(self) -> Self {
        self.with_pull(GpioPull::Down)
    }

    /// Overrides this pin's output speed / slew rate to low (the default
    /// for [`output`](Self::output)/[`input`](Self::input)/
    /// [`alternate`](Self::alternate)).
    pub const fn with_low_speed(self) -> Self {
        self.with_speed(GpioSpeed::Low)
    }

    /// Overrides this pin's output speed / slew rate to medium.
    pub const fn with_medium_speed(self) -> Self {
        self.with_speed(GpioSpeed::Medium)
    }

    /// Overrides this pin's output speed / slew rate to high.
    pub const fn with_high_speed(self) -> Self {
        self.with_speed(GpioSpeed::High)
    }

    /// Overrides this pin's output speed / slew rate to very high.
    pub const fn with_very_high_speed(self) -> Self {
        self.with_speed(GpioSpeed::VeryHigh)
    }

    const fn with_pull(self, pull: GpioPull) -> Self {
        GpioPin((self.0 & !(PULL_MASK << PULL_SHIFT)) | ((pull as u32) << PULL_SHIFT))
    }

    const fn with_speed(self, speed: GpioSpeed) -> Self {
        GpioPin((self.0 & !(SPEED_MASK << SPEED_SHIFT)) | ((speed as u32) << SPEED_SHIFT))
    }

    const fn packed(
        port: GpioPort,
        pin_number: u8,
        mode: GpioMode,
        pull: GpioPull,
        speed: GpioSpeed,
        af: u8,
    ) -> Self {
        let bits = ((af as u32) << AF_SHIFT)
            | ((speed as u32) << SPEED_SHIFT)
            | ((pull as u32) << PULL_SHIFT)
            | ((mode as u32) << MODE_SHIFT)
            | ((port as u32) << PORT_SHIFT)
            | ((pin_number as u32) << PIN_NUMBER_SHIFT);
        GpioPin(bits)
    }

    pub fn port(&self) -> GpioPort {
        GpioPort::from_bits(((self.0 >> PORT_SHIFT) & PORT_MASK) as u8)
    }

    pub fn pin_number(&self) -> u8 {
        ((self.0 >> PIN_NUMBER_SHIFT) & PIN_NUMBER_MASK) as u8
    }

    pub fn mode(&self) -> GpioMode {
        GpioMode::from_bits(((self.0 >> MODE_SHIFT) & MODE_MASK) as u8)
    }

    pub fn pull(&self) -> GpioPull {
        GpioPull::from_bits(((self.0 >> PULL_SHIFT) & PULL_MASK) as u8)
    }

    pub fn speed(&self) -> GpioSpeed {
        GpioSpeed::from_bits(((self.0 >> SPEED_SHIFT) & SPEED_MASK) as u8)
    }

    /// The alternate-function index this pin was configured with via
    /// [`GpioPin::alternate`]. Meaningless (always 0) unless
    /// `mode() == GpioMode::AlternateMode`.
    pub fn alternate_function(&self) -> u8 {
        ((self.0 >> AF_SHIFT) & AF_MASK) as u8
    }
}

impl core::fmt::Debug for GpioPin {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("GpioPin")
            .field("port", &self.port())
            .field("pin_number", &self.pin_number())
            .field("mode", &self.mode())
            .field("pull", &self.pull())
            .field("speed", &self.speed())
            .finish()
    }
}

impl GpioPort {
    fn from_bits(v: u8) -> Self {
        match v {
            0 => GpioPort::PA,
            1 => GpioPort::PB,
            2 => GpioPort::PC,
            3 => GpioPort::PD,
            4 => GpioPort::PE,
            5 => GpioPort::PF,
            6 => GpioPort::PG,
            7 => GpioPort::PH,
            8 => GpioPort::PI,
            9 => GpioPort::PJ,
            _ => unreachable!("GpioPin only ever stores a valid GpioPort"),
        }
    }
}

impl GpioMode {
    fn from_bits(v: u8) -> Self {
        match v {
            0 => GpioMode::Input,
            1 => GpioMode::InvertedInput,
            2 => GpioMode::Output,
            3 => GpioMode::InvertedOutput,
            4 => GpioMode::AlternateMode,
            5 => GpioMode::Analog,
            _ => unreachable!("GpioPin only ever stores a valid GpioMode"),
        }
    }
}

impl GpioPull {
    fn from_bits(v: u8) -> Self {
        match v {
            0 => GpioPull::None,
            1 => GpioPull::Up,
            2 => GpioPull::Down,
            _ => unreachable!("GpioPin only ever stores a valid GpioPull"),
        }
    }
}

impl GpioSpeed {
    fn from_bits(v: u8) -> Self {
        match v {
            0 => GpioSpeed::Low,
            1 => GpioSpeed::Medium,
            2 => GpioSpeed::High,
            3 => GpioSpeed::VeryHigh,
            _ => unreachable!("GpioPin only ever stores a valid GpioSpeed"),
        }
    }
}

/// Implemented by any type that knows its own physical GPIO identity —
/// e.g. a pin-token type like `stm32g4::gpio::PC6`, or (via a blanket impl
/// over its inner type) a `Peri`-style ownership wrapper around one.
/// [`fake::gpio::GpioFake::claim_pin`](crate::fake::gpio::GpioFake::claim_pin)
/// uses this to register a claimed pin.
pub trait PinToken {
    const PORT: GpioPort;
    const NUMBER: u8;
}

/// Abstract interface implemented by every GPIO driver, real or fake.
pub trait GpioTrait {
    /// Applies `pin`'s mode/pull/speed configuration. On return, a configured
    /// pull-up/pull-down will be active. An output pin will be driven to
    /// follow the pull-up/pull-down configuration and it will be driven low if
    /// GpioPull is None.
    fn configure(&mut self, pin: GpioPin);

    /// Drives `pin` to the given `logical` level. This function is a no-op
    /// unless `pin` is configured as an output (or inverted output).
    fn set(&self, pin: GpioPin, logical: bool);

    /// Reads `pin`'s current logical level.
    fn get(&self, pin: GpioPin) -> bool;
}

/// Calls `$gpio.claim_pin(...)` once per pin resource listed, so a whole
/// batch (e.g. fields just moved out of a `resources::Peripherals`, each a
/// different concrete `Peri<'static, PAx>` type) can be claimed without
/// writing one call per pin by hand:
///
/// ```ignore
/// peripherals::claim_pins!(gpio, PA0, PA1, PA8, PC13);
/// ```
///
/// expands to
///
/// ```ignore
/// gpio.claim_pin(PA0);
/// gpio.claim_pin(PA1);
/// gpio.claim_pin(PA8);
/// gpio.claim_pin(PC13);
/// ```
#[macro_export]
macro_rules! claim_pins {
    ($gpio:ident, $($pin:ident),+ $(,)?) => {
        $($gpio.claim_pin($pin);)+
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn claim_pins_calls_claim_pin_once_per_argument() {
        struct Counter(u32);
        impl Counter {
            fn claim_pin<T>(&mut self, _pin: T) {
                self.0 += 1;
            }
        }

        let mut gpio = Counter(0);
        let pa0 = ();
        let pa1 = 1u8;
        let pc13 = "PC13";
        claim_pins!(gpio, pa0, pa1, pc13);
        assert_eq!(gpio.0, 3);
    }

    #[test]
    fn output_defaults_to_no_pull_and_low_speed() {
        let pin = GpioPin::output(GpioPort::PC, 6);
        assert_eq!(pin.port(), GpioPort::PC);
        assert_eq!(pin.pin_number(), 6);
        assert_eq!(pin.mode(), GpioMode::Output);
        assert_eq!(pin.pull(), GpioPull::None);
        assert_eq!(pin.speed(), GpioSpeed::Low);
        assert_eq!(pin.alternate_function(), 0);
    }

    #[test]
    fn analog_defaults_to_no_pull() {
        let pin = GpioPin::analog(GpioPort::PC, 6);
        assert_eq!(pin.mode(), GpioMode::Analog);
        assert_eq!(pin.pull(), GpioPull::None);
    }

    #[test]
    fn with_pull_up_and_with_high_speed_override_the_defaults() {
        let pin = GpioPin::output(GpioPort::PC, 6)
            .with_pull_up()
            .with_high_speed();
        assert_eq!(pin.pull(), GpioPull::Up);
        assert_eq!(pin.speed(), GpioSpeed::High);
    }

    #[test]
    fn with_pull_down_overrides_a_previous_pull_up() {
        let pin = GpioPin::input(GpioPort::PC, 6)
            .with_pull_up()
            .with_pull_down();
        assert_eq!(pin.pull(), GpioPull::Down);
    }

    #[test]
    fn input_defaults_to_no_pull_and_low_speed() {
        let pin = GpioPin::input(GpioPort::PC, 6);
        assert_eq!(pin.mode(), GpioMode::Input);
        assert_eq!(pin.pull(), GpioPull::None);
        assert_eq!(pin.speed(), GpioSpeed::Low);
    }

    #[test]
    fn alternate_stores_af_index() {
        let pin = GpioPin::alternate(GpioPort::PC, 6, 9).with_very_high_speed();
        assert_eq!(pin.mode(), GpioMode::AlternateMode);
        assert_eq!(pin.alternate_function(), 9);
        assert_eq!(pin.speed(), GpioSpeed::VeryHigh);
    }

    #[test]
    #[should_panic(expected = "af must fit in 4 bits")]
    fn alternate_panics_on_out_of_range_af() {
        GpioPin::alternate(GpioPort::PC, 6, 16);
    }

    #[test]
    fn constructors_and_builders_are_usable_in_const_context() {
        const OUT: GpioPin = GpioPin::output(GpioPort::PC, 6);
        const IN: GpioPin = GpioPin::input(GpioPort::PC, 6).with_pull_up();
        const AF: GpioPin = GpioPin::alternate(GpioPort::PC, 6, 6).with_very_high_speed();
        assert_eq!(OUT.mode(), GpioMode::Output);
        assert_eq!(IN.mode(), GpioMode::Input);
        assert_eq!(IN.pull(), GpioPull::Up);
        assert_eq!(AF.alternate_function(), 6);
        assert_eq!(AF.speed(), GpioSpeed::VeryHigh);
    }
}
