//! Hardware-agnostic math-coprocessor API for trigonometric functions that are
//! no implemented on Cortex-M FPUs. Concrete drivers (real or fake) implement
//! [`MathCoprocessorTrait`].
//!
//! Every angle/ratio is a fraction of a half-turn, represented as a
//! [`SymmetricUnitInterval`], which wraps at its `[-1, 1)` bounds the same
//! way an angle wraps at `±pi` radians / `±180°`:
//!
//! |                         | min     | 0    | max              |
//! |-------------------------|:-------:|:----:|:----------------:|
//! | `SymmetricUnitInterval` | `-1`    | `0`  | just below `1`   |
//! | radians                 | `-pi`   | `0`  | just below `pi`  |
//! | degrees                 | `-180°` | `0°` | just below `180°`|

use common::unit_interval::SymmetricUnitInterval;

/// A function [`MathCoprocessorTrait::compute`] can be asked to evaluate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MathCoprocessorFunction {
    /// `SineCosine(arg)`` computes `(sin(arg * pi), cos(arg * pi))`.
    SineCosine(SymmetricUnitInterval),
    /// Cartesian-to-polar conversion: `Phase(x, y)` computes `(atan2(y, x) /
    /// pi, sqrt(x*x + y*y))`. The second half of the result is only
    /// representable when `x*x + y*y <= 1`. The result will saturate at `1` if
    /// the input is outside that range.
    Phase(SymmetricUnitInterval, SymmetricUnitInterval),
}

/// Abstract interface for a math-coprocessor.
pub trait MathCoprocessorTrait {
    /// Starts computing `function`. Only one computation may be in flight
    /// at a time: call [`Self::result`] to retrieve it before starting
    /// another.
    fn compute(&mut self, function: MathCoprocessorFunction);

    /// Blocks until the computation started by the last [`Self::compute`]
    /// call has finished, then returns its two results — see
    /// [`MathCoprocessorFunction`] for what each variant's pair means.
    fn result(&mut self) -> (SymmetricUnitInterval, SymmetricUnitInterval);
}
