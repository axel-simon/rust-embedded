// `no_std` only for the real (embedded) arm target. Every other target
// gets full `std`, `cfg(test)` or not: `cfg(test)` is only true when
// *this* crate is the one directly under test, not when it's merely a
// dependency of another crate's host-side `cargo test`, and `fake` needs
// `std::rc::Rc` unconditionally on host builds. See peripherals/README.md for
// how to run the tests.
#![cfg_attr(target_arch = "arm", no_std)]

pub mod api;
// `Rc`-backed (see `fake::gpio`/`fake::clock`), so it needs `std` —
// unavailable, and unneeded, on the real arm target: nothing reachable
// when building for real hardware references `peripherals::fake`.
#[cfg(not(target_arch = "arm"))]
pub mod fake;
// The real drivers depend on `stm32-metapac`/`cortex-m`, only pulled in
// for `target_arch = "arm"` (see Cargo.toml), and every item inside is
// only ever used from other arm-gated code.
#[cfg(target_arch = "arm")]
pub mod stm32g4;
