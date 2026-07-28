//! A register-level [`MathCoprocessorTrait`] driver for a real STM32G4
//! chip's CORDIC co-processor, backed by `stm32-metapac`.
//!
//! Always configured for maximum precision: 24 iterations (6 clock cycles
//! at 32-bit width) and 32-bit (rather than 16-bit) argument/result width.
//! 24 iterations, not the CSR register's largest `PRECISION` setting (60):
//! per Table 115 of the reference manual, 24 iterations already reach
//! 32-bit output accuracy, and any iteration beyond that just spends more
//! clock cycles without improving the result.
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs`: it depends on `stm32-metapac`,
//! which is only pulled in as a dependency for that target (see
//! Cargo.toml), so host-side `cargo test` (see peripherals/README.md)
//! never compiles it.

use common::unit_interval::SymmetricUnitInterval;
use stm32_metapac::cordic::vals;

use crate::api::math_coprocessor::{MathCoprocessorFunction, MathCoprocessorTrait};

/// Register-level [`MathCoprocessorTrait`] driver for a real STM32G4 CORDIC
/// co-processor.
pub struct MathCoprocessor;

impl MathCoprocessor {
    /// Enables the CORDIC's AHB1 clock. This driver does no bookkeeping of
    /// its own beyond that — it stays a zero-sized type; the peripheral's
    /// own CSR/WDATA/RDATA registers are all the state [`Self::compute`]/
    /// [`Self::result`] need.
    pub fn new() -> Self {
        stm32_metapac::RCC
            .ahb1enr()
            .modify(|w| w.set_cordicen(true));
        MathCoprocessor
    }
}

impl Default for MathCoprocessor {
    fn default() -> Self {
        Self::new()
    }
}

impl MathCoprocessorTrait for MathCoprocessor {
    fn compute(&mut self, function: MathCoprocessorFunction) {
        let r = stm32_metapac::CORDIC;

        // `Phase`'s two arguments can each range across the full [-1, 1)
        // input domain, so the vector they form can reach the hardware's
        // internal working range during the CORDIC gain expansion (a
        // factor of ~1.21) partway through the iteration. Scaling the
        // arguments down by 2 (and the modulus result back up by 2) keeps
        // that expansion from overflowing. `SineCosine`'s single argument
        // never needs this: its implicit second argument (the modulus,
        // fixed at 1) is exactly what the unscaled hardware default is
        // designed for.
        let (func, nargs, scale) = match function {
            MathCoprocessorFunction::SineCosine(_) => {
                (vals::Func::SINE, vals::Num::NUM1, vals::Scale::A1_R1)
            }
            MathCoprocessorFunction::Phase(_, _) => {
                (vals::Func::PHASE, vals::Num::NUM2, vals::Scale::A1O2_R2)
            }
        };

        r.csr().modify(|w| {
            w.set_func(func);
            w.set_precision(vals::Precision::ITERS24);
            w.set_scale(scale);
            w.set_nargs(nargs);
            w.set_nres(vals::Num::NUM2);
            w.set_argsize(vals::Size::BITS32);
            w.set_ressize(vals::Size::BITS32);
        });

        match function {
            MathCoprocessorFunction::SineCosine(angle) => {
                r.wdata().write_value(angle.raw() as u32);
            }
            MathCoprocessorFunction::Phase(x, y) => {
                r.wdata().write_value(x.raw() as u32);
                r.wdata().write_value(y.raw() as u32);
            }
        }
    }

    fn result(&mut self) -> (SymmetricUnitInterval, SymmetricUnitInterval) {
        let r = stm32_metapac::CORDIC;
        // No need to poll RRDY first: reading RDATA before the result is
        // ready stalls the bus until it is (reference manual, p.473).
        let first = SymmetricUnitInterval::new(r.rdata().read() as i32);
        let second = SymmetricUnitInterval::new(r.rdata().read() as i32);
        (first, second)
    }
}
