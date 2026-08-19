//! Generic drivers built on top of [`peripherals::api`] traits — real or
//! fake, so nothing here depends on a concrete chip or board. Each module
//! is its own driver; see e.g. [`foc`].
#![cfg_attr(not(test), no_std)]

pub mod foc;
