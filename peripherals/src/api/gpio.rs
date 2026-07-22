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

const PIN_NUMBER_BITS: u32 = 5;
const PORT_BITS: u32 = 4;
const MODE_BITS: u32 = 3;
const PULL_BITS: u32 = 2;
const SPEED_BITS: u32 = 2;

const PIN_NUMBER_SHIFT: u32 = 0;
const PORT_SHIFT: u32 = PIN_NUMBER_SHIFT + PIN_NUMBER_BITS;
const MODE_SHIFT: u32 = PORT_SHIFT + PORT_BITS;
const PULL_SHIFT: u32 = MODE_SHIFT + MODE_BITS;
const SPEED_SHIFT: u32 = PULL_SHIFT + PULL_BITS;

const PIN_NUMBER_MASK: u32 = (1 << PIN_NUMBER_BITS) - 1;
const PORT_MASK: u32 = (1 << PORT_BITS) - 1;
const MODE_MASK: u32 = (1 << MODE_BITS) - 1;
const PULL_MASK: u32 = (1 << PULL_BITS) - 1;
const SPEED_MASK: u32 = (1 << SPEED_BITS) - 1;

/// A fully-specified GPIO pin configuration, bit-packed into a single
/// `u32` (port + pin number + mode + pull + speed only need 16 bits, well
/// within the requested 32-bit budget).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct GpioPin(u32);

const _: () = assert!(core::mem::size_of::<GpioPin>() == 4);

impl GpioPin {
    /// # Panics
    /// Panics if `pin_number` doesn't fit in 5 bits (i.e. is greater than 31).
    pub fn new(
        port: GpioPort,
        pin_number: u8,
        mode: GpioMode,
        pull: GpioPull,
        speed: GpioSpeed,
    ) -> Self {
        assert!(
            (pin_number as u32) <= PIN_NUMBER_MASK,
            "pin_number must fit in 5 bits"
        );
        let bits = ((speed as u32) << SPEED_SHIFT)
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

/// A `(port, pin_number)` pair identifying one physical GPIO pin.
///
/// This can only be constructed by naming a real pin type token (e.g.
/// `stm32g4::gpio::pin_and_port::<PC6>()`), so it's impossible to name a
/// pin that doesn't exist on the target chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinAndPort {
    port: GpioPort,
    pin_number: u8,
}

impl PinAndPort {
    pub(crate) fn new(port: GpioPort, pin_number: u8) -> Self {
        Self { port, pin_number }
    }

    pub fn port(&self) -> GpioPort {
        self.port
    }

    pub fn pin_number(&self) -> u8 {
        self.pin_number
    }
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
