//! Re-exports [`rtic_real`]'s `app` attribute macro (real RTIC, with
//! `peripherals = false` forced) on real hardware, or [`rtic_fake`]'s
//! host-only fake with the same surface elsewhere, so firmware written
//! once against `#[rtic_shim::app(...)]` builds real RTIC on hardware and
//! stays unit-testable on the host.
#![no_std]

#[cfg(target_arch = "arm")]
pub use rtic_real::app;

#[cfg(not(target_arch = "arm"))]
pub use rtic_fake::app;
