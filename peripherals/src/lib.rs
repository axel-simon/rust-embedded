// `no_std` for real (embedded) builds; `cfg(test)` always builds for a
// hosted target where `std` is available, which `fake`'s unit tests rely
// on for output/assertions. See peripherals/README.md for how to run them.
#![cfg_attr(not(test), no_std)]

pub mod api;
pub mod fake;
pub mod stm32g4;
