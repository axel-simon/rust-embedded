//! Backend-selected `Peripherals`/`Peri` aliases: real hardware via
//! [`stm32g4`] (`embassy_stm32::init()` under the hood) on the arm target,
//! or a simulated one via [`fake`] on host builds (so `cargo test` works
//! without real hardware). Both expose the identical shape — same
//! `Peripherals` field names, same `Peri<'d, T>` shape, same
//! `peripherals::NAME` marker types — so board code written against these
//! aliases compiles unchanged either way.
#![cfg_attr(not(test), no_std)]

#[cfg(target_arch = "arm")]
mod stm32g4;
#[cfg(target_arch = "arm")]
pub use stm32g4::*;

#[cfg(not(target_arch = "arm"))]
mod fake;
#[cfg(not(target_arch = "arm"))]
pub use fake::*;

/// The clock configuration of a board.
/// This struct abstractly describes how the MCU's clock tree is configured
/// and assumes that the MCU clock is always provided by scaling an external
/// oscillator or crystal.
pub struct ClockConfiguration {
    pub mcu_frequency: u32,
    pub oscillator_frequency: u32,
    /// `true` if the timepiece is an actual crystal, needing the MCU's own
    /// oscillator amplifier across the input and output pins; `false` if
    /// it's driven by an external active oscillator module instead, which
    /// feeds a clock signal into the input alone and leaves the MCU's
    /// output pin unused.
    pub timepiece_is_crystal: bool,
}

/// Marks a `peripherals::TIMx` marker type (e.g.
/// `resources::peripherals::TIM4`, on either backend) as naming a timer
/// instance that can decode a quadrature encoder — see
/// [`peripherals::api::quadrature`]. Implemented below, per backend
/// (`stm32g4.rs`/`fake_embassy.rs`), only for the specific timers this
/// workspace's driver actually supports (see
/// `peripherals::stm32g4::quadrature`'s doc comment for exactly which), so
/// `claim_quadrature_timer` (defined per backend alongside
/// [`stm32g4::split_off_fake`]/[`fake::split_off_fake`] — see their doc
/// comment for why this can't be a single, backend-agnostic function
/// either) rejects an incompatible timer (e.g. a basic timer with no
/// capture/compare channels at all) at compile time rather than at
/// runtime.
///
/// A dedicated trait rather than reusing `peripherals::fake::gpio::PinToken`
/// or a DMA-channel-style token: unlike a GPIO pin (whose physical identity
/// — a `(port, number)` pair — is meaningful to *any* peripheral routed
/// through it), which specific
/// [`QuadratureTimer`](::peripherals::api::quadrature::QuadratureTimer)
/// variant a timer resource maps to is a fact only the quadrature driver
/// cares about, and the real backend can only legally provide it here:
/// implementing this trait for `embassy_stm32`'s own peripheral types is
/// only possible in a crate that either defines the trait or the type —
/// `embassy_stm32::peripherals::TIM4` is foreign to `peripherals` (which
/// doesn't depend on `embassy_stm32` at all), so the trait has to live
/// wherever the real `impl`s can legally exist: this crate, the one that
/// already owns the `embassy_stm32` dependency.
pub trait QuadratureCapableTimer {
    const TIMER: ::peripherals::api::quadrature::QuadratureTimer;
}

/// Marks a `peripherals::ADCx` marker type (e.g.
/// `resources::peripherals::ADC1`, on either backend) as naming an ADC
/// instance [`peripherals::api::adc::AdcInstance`](::peripherals::api::adc::AdcInstance)
/// has a variant for. Implemented below, per backend
/// (`stm32g4.rs`/`fake_embassy.rs`), only for the specific ADC instances
/// this workspace's driver actually supports on the chip feature it
/// targets, so `claim_adc` (defined per backend alongside
/// [`stm32g4::split_off_fake`]/[`fake::split_off_fake`] — see
/// [`QuadratureCapableTimer`]'s doc comment for why this can't be a single,
/// backend-agnostic function either) rejects an unsupported ADC resource at
/// compile time rather than at runtime.
///
/// A dedicated trait for the same reason [`QuadratureCapableTimer`] is one:
/// which [`AdcInstance`](::peripherals::api::adc::AdcInstance) variant an
/// ADC resource maps to is a fact only the ADC driver cares about, and the
/// real backend can only legally provide it here — `embassy_stm32::
/// peripherals::ADC1` is foreign to `peripherals`, which doesn't depend on
/// `embassy_stm32` at all.
pub trait AdcCapableInstance {
    const INSTANCE: ::peripherals::api::adc::AdcInstance;
}
