//! Hardware-agnostic quadrature-encoder API. Concrete drivers (real or
//! fake) implement [`QuadratureTrait`]; nothing in this module depends on
//! any particular chip.

use common::unit_interval::UnitInterval;

/// A timer instance capable of decoding a quadrature encoder in hardware.
///
/// For simplicity, the `Stm32g4Tim2` peripheral is not supported here, as it
/// is a 32-bit timer that requires more complex handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum QuadratureTimer {
    Stm32g4Tim1 = 0,
    Stm32g4Tim3 = 2,
    Stm32g4Tim4 = 3,
    Stm32g4Tim5 = 4,
    Stm32g4Tim8 = 5,
    Stm32g4Tim20 = 6,
}

/// Which of a timer's two capture channels (channel 1 / channel 2) carries
/// the encoder's "A"/"B" phase signal.
///
/// The physical pins feeding channel 1 and channel 2 are fixed by the
/// board's wiring (whichever alternate function the caller routed them
/// through — see [`QuadratureOptions::input_configuration`]); this only
/// says which of the two is logically "A", i.e. which one increases the
/// count on a rising edge while both are low. Swapping it reverses the
/// encoder's perceived counting direction — the same effect physically
/// swapping the two phase wires would have.
///
/// Not prefixed per-family like [`QuadratureTimer`] is: every classic STM32
/// timer's encoder interface (going back to the F1/F4 era, not just G4)
/// shares the same channel-swap mechanism (RM0440's CC1S/CC2S "alternate
/// mapping"), so this isn't specific to one chip family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuadratureInputConfiguration {
    /// Channel 1 carries input A, channel 2 carries input B.
    Ch12AreInputsAB,
    /// Channel 1 carries input B, channel 2 carries input A — reverses the
    /// counting direction relative to
    /// [`Self::Ch12AreInputsAB`].
    Ch12AreInputsBA,
}

/// Configuration for a quadrature-encoder input, decoded by a hardware
/// timer running in encoder-interface mode.
pub struct QuadratureOptions {
    /// Which of the chosen [`QuadratureTimer`]'s channel-1/channel-2 inputs
    /// carries the encoder's A/B phase — see
    /// [`QuadratureInputConfiguration`]. The pins themselves (routed to
    /// those channels via the correct alternate function) must already be
    /// configured by the caller (e.g. board init, via
    /// [`crate::api::gpio::GpioTrait::configure`]) — this only tells the
    /// driver which physical channel to treat as which phase.
    pub input_configuration: QuadratureInputConfiguration,
    /// The number of distinct counts one full cycle of the encoder
    /// represents (i.e. the hardware counter wraps back to `0` after this
    /// many counts). [`QuadratureTrait::position`] reports the current
    /// count scaled into this range as a [`UnitInterval`].
    pub encoder_counts: u32,
}

/// Abstract interface implemented by every quadrature-encoder driver, real
/// or fake.
pub trait QuadratureTrait {
    /// Configures encoder-interface mode per `options` and starts the
    /// counter at position `0`.
    fn open(&mut self, options: QuadratureOptions);

    /// Stops the counter. [`Self::open`] must be called again before
    /// [`Self::position`].
    fn close(&mut self);

    /// The encoder's current position, as a fraction of one full
    /// [`QuadratureOptions::encoder_counts`] cycle. Only meaningful once
    /// [`Self::open`] has been called.
    fn position(&self) -> UnitInterval;
}
