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
