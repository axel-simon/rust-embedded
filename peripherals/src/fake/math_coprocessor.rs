//! A fake [`MathCoprocessorTrait`] implementation for host-side testing,
//! computing results with `std`'s floating-point math instead of talking to
//! any real math-accelerator hardware.
//!
//! Panics if [`MathCoprocessorTrait::compute`]/
//! [`MathCoprocessorTrait::result`] aren't called strictly alternating,
//! starting with `compute` — the same discipline [`MathCoprocessorTrait`]
//! itself requires of every implementation (see its doc comment): only one
//! computation may be in flight at a time, so there's nowhere to stash a
//! second one, and no meaningful result to hand back before one has been
//! started.

use common::unit_interval::SymmetricUnitInterval;

use crate::api::math_coprocessor::{MathCoprocessorFunction, MathCoprocessorTrait};

/// A fake math-coprocessor driver that computes results via `f32` methods
/// from `std`, for host-side tests. See the module doc comment for its
/// `compute`/`result` alternation requirement.
pub struct MathCoprocessor {
    /// The function passed to the last [`Self::compute`] call, if
    /// [`Self::result`] hasn't consumed it yet.
    pending: Option<MathCoprocessorFunction>,
}

impl MathCoprocessor {
    pub fn new() -> Self {
        MathCoprocessor { pending: None }
    }
}

impl MathCoprocessorTrait for MathCoprocessor {
    fn compute(&mut self, function: MathCoprocessorFunction) {
        assert!(
            self.pending.is_none(),
            "MathCoprocessor::compute() called again before the previous call's result() was read"
        );
        self.pending = Some(function);
    }

    fn result(&mut self) -> (SymmetricUnitInterval, SymmetricUnitInterval) {
        let function = self
            .pending
            .take()
            .expect("MathCoprocessor::result() called without a matching compute() first");

        match function {
            MathCoprocessorFunction::SineCosine(angle) => {
                let radians = f32::from(angle) * core::f32::consts::PI;
                (
                    SymmetricUnitInterval::from(radians.sin()),
                    SymmetricUnitInterval::from(radians.cos()),
                )
            }
            MathCoprocessorFunction::Phase(x, y) => {
                let x = f32::from(x);
                let y = f32::from(y);
                let phase = y.atan2(x) / core::f32::consts::PI;
                let modulus = (x * x + y * y).sqrt();
                (
                    SymmetricUnitInterval::from(phase),
                    SymmetricUnitInterval::from(modulus),
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f32 = 1e-3;

    fn approx_eq(actual: SymmetricUnitInterval, expected: f32) -> bool {
        (f32::from(actual) - expected).abs() <= TOLERANCE
    }

    #[test]
    fn sine_cosine_at_zero_angle() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.0),
        ));
        let (sin, cos) = m.result();
        assert!(approx_eq(sin, 0.0));
        assert!(approx_eq(cos, 1.0));
    }

    #[test]
    fn sine_cosine_at_a_quarter_turn() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.5),
        ));
        let (sin, cos) = m.result();
        assert!(approx_eq(sin, 1.0));
        assert!(approx_eq(cos, 0.0));
    }

    #[test]
    fn sine_cosine_at_a_negative_quarter_turn() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(-0.5),
        ));
        let (sin, cos) = m.result();
        assert!(approx_eq(sin, -1.0));
        assert!(approx_eq(cos, 0.0));
    }

    #[test]
    fn sine_cosine_at_negative_one() {
        // `-1` represents `-pi` radians exactly.
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(-1.0),
        ));
        let (sin, cos) = m.result();
        assert!(approx_eq(sin, 0.0));
        assert!(approx_eq(cos, -1.0));
    }

    #[test]
    fn sine_cosine_at_one() {
        // `1` itself isn't representable (`[-1, 1)` is right-open);
        // `from(1.0)` saturates to the largest representable value, just
        // below `1` — i.e. just below `pi` radians, on the other side of
        // the wraparound from `-1`, so it should match `-1`'s result.
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(1.0),
        ));
        let (sin, cos) = m.result();
        assert!(approx_eq(sin, 0.0));
        assert!(approx_eq(cos, -1.0));
    }

    #[test]
    fn phase_of_a_3_4_5_triangle_vector() {
        // x = 0.6, y = 0.8: x*x + y*y = 1.0 exactly, and the angle above
        // the x-axis is the well-known ~53.13 degree (0.9273 rad) angle of
        // a 3-4-5 right triangle.
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::Phase(
            SymmetricUnitInterval::from(0.6),
            SymmetricUnitInterval::from(0.8),
        ));
        let (phase, modulus) = m.result();
        assert!(approx_eq(phase, 0.2952));
        assert!(approx_eq(modulus, 1.0));
    }

    #[test]
    fn phase_of_the_negative_x_axis_unit_vector() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::Phase(
            SymmetricUnitInterval::from(-1.0),
            SymmetricUnitInterval::from(0.0),
        ));
        let (phase, modulus) = m.result();
        // atan2(0, -1) == pi, right at the wraparound edge of the [-1, 1)
        // representable range.
        assert!(approx_eq(phase, -1.0) || approx_eq(phase, 1.0));
        assert!(approx_eq(modulus, 1.0));
    }

    #[test]
    fn compute_then_result_then_compute_again_works() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.0),
        ));
        let _ = m.result();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.5),
        ));
        let (sin, _) = m.result();
        assert!(approx_eq(sin, 1.0));
    }

    #[test]
    #[should_panic(
        expected = "compute() called again before the previous call's result() was read"
    )]
    fn compute_twice_without_result_panics() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.0),
        ));
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.5),
        ));
    }

    #[test]
    #[should_panic(expected = "result() called without a matching compute() first")]
    fn result_before_any_compute_panics() {
        let mut m = MathCoprocessor::new();
        let _ = m.result();
    }

    #[test]
    #[should_panic(expected = "result() called without a matching compute() first")]
    fn result_twice_in_a_row_panics() {
        let mut m = MathCoprocessor::new();
        m.compute(MathCoprocessorFunction::SineCosine(
            SymmetricUnitInterval::from(0.0),
        ));
        let _ = m.result();
        let _ = m.result();
    }
}
