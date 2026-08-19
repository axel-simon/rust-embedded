//! Field-oriented control: per-phase zero-current calibration, rotor
//! electrical-angle tracking via a quadrature encoder, and closed-loop
//! current regulation through the Clarke/Park transform.

use common::filter::LowPassFilter;
use common::unit_interval::{SymmetricUnitInterval, UnitInterval};
use peripherals::api::math_coprocessor::{MathCoprocessorFunction, MathCoprocessorTrait};

/// `sqrt(3)`, used by the Clarke/inverse-Clarke transforms in
/// [`FieldOrientedControl::synthesize_voltage`].
const SQRT_3: f32 = 1.732_050_8;

/// Below this fraction of the maximum bus voltage there's not enough
/// headroom to usefully drive anything — see [`FieldOrientedControl::poll`].
const MIN_BUS_VOLTAGE_FRACTION: f32 = 0.05;

/// Nominal time constant (in poll calls) each per-phase zero-current
/// filter settles to — see [`LowPassFilter`]'s own doc comment for what
/// the unit means.
const ZERO_CURRENT_FILTER_TIME_CONSTANT: f32 = 100.0;

/// What [`FieldOrientedControl::poll`] should do this cycle — commanded by
/// a firmware's own main loop via shared state. `pub`, not private: a
/// firmware typically stores this in an RTIC `#[shared]` struct field,
/// whose generated per-resource proxy type is itself `pub`, and Rust's
/// "private type in public interface" check requires this type to be at
/// least as visible as that — `pub(crate)` alone isn't enough.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentLoopOperation {
    /// No PWM output — every phase's zero-current filter is updated
    /// instead. See [`FieldOrientedControl::poll`].
    Open,
    /// Injects [`FieldOrientedControl::new`]'s `phase_lock_voltage` into
    /// phase U to align the rotor at a known electrical angle, and
    /// records [`FieldOrientedControl::poll`]'s own `physical_angle`
    /// argument as the new zero position — on every call, not just the
    /// first, so the rotor's alignment (and hence the zero position)
    /// keeps tracking reality for as long as this is commanded: a
    /// firmware should hold this for a settling window (long enough for
    /// the rotor to actually rotate into alignment) before switching
    /// away and treating the zero position as final.
    PhaseLock,
    /// Closed-loop current control, targeting this fraction of maximum
    /// current on the rotor's Q axis (`0` is zero current).
    CurrentControl(SymmetricUnitInterval),
}

/// Field-oriented control of a three-phase (U/V/W) motor — generic over
/// the math coprocessor ([`MathCoprocessorTrait`]) driver it's built from
/// (real or fake), so this module depends only on `peripherals::api`,
/// never a concrete chip or board.
///
/// Doesn't own (or know how to read) the quadrature encoder itself:
/// [`Self::poll`] takes the rotor's current physical (mechanical) angle
/// as a plain argument instead, so a firmware reads it once per
/// current-loop cycle from wherever it actually keeps that driver (see
/// [`Self::poll`]'s own doc comment).
///
/// Owns the math coprocessor (sin/cos for the Clarke/Park transform);
/// [`Self::poll`] is the whole interface, called once per current-loop
/// cycle with each phase's raw ADC reading and the rotor's current
/// physical angle.
pub struct FieldOrientedControl<M> {
    math_coprocessor: M,
    /// Per-phase (U, V, W) zero-current filters, updated while
    /// [`CurrentLoopOperation::Open`] — see [`Self::poll`].
    zero_filters: [LowPassFilter; 3],
    /// Each phase's most recent zero-current filter output, already
    /// shifted into the same domain [`UnitInterval::expand`] produces —
    /// storing it pre-shifted avoids re-shifting it on every non-`Open`
    /// [`Self::poll`] call instead.
    bias: [SymmetricUnitInterval; 3],
    /// The physical angle [`CurrentLoopOperation::PhaseLock`] last
    /// recorded as electrical angle zero — subtracted from every later
    /// reading to get an angle relative to phase U.
    zero_position: UnitInterval,
    /// The voltage injected into phase U during
    /// [`CurrentLoopOperation::PhaseLock`] — see [`Self::new`].
    phase_lock_voltage: SymmetricUnitInterval,
    /// Set by [`Self::set_bus_voltage`]; scales every PWM output and
    /// gates it off entirely below [`MIN_BUS_VOLTAGE_FRACTION`].
    bus_voltage: UnitInterval,
    /// The motor's pole-pair count — the multiplier
    /// [`Self::synthesize_voltage`] applies (via `UnitInterval`'s own
    /// wrapping `Mul<u32>`) to a physical-angle difference to get the
    /// corresponding electrical angle: one electrical revolution happens
    /// once per mechanical revolution per pole pair. See [`Self::new`].
    pole_pairs: u32,
}

impl<M: MathCoprocessorTrait> FieldOrientedControl<M> {
    /// `pole_pairs` is the motor's pole-*pair* count, not its raw pole
    /// count (half that) — it's what scales a physical-angle difference
    /// into the corresponding electrical angle (one electrical revolution
    /// per mechanical revolution per pole pair). Taking pole pairs
    /// directly, rather than poles, avoids an implicit "poles must be
    /// even" assumption a `poles / 2` conversion here would otherwise
    /// silently rely on.
    ///
    /// # Panics
    /// Panics if `pole_pairs` is `0` — every real motor has at least one
    /// pole pair, and `0` would collapse every mechanical angle to the
    /// same (zero) electrical angle.
    pub fn new(
        math_coprocessor: M,
        phase_lock_voltage: SymmetricUnitInterval,
        pole_pairs: u32,
    ) -> Self {
        assert!(pole_pairs > 0, "pole_pairs must be at least 1");
        FieldOrientedControl {
            math_coprocessor,
            zero_filters: [
                LowPassFilter::new(ZERO_CURRENT_FILTER_TIME_CONSTANT),
                LowPassFilter::new(ZERO_CURRENT_FILTER_TIME_CONSTANT),
                LowPassFilter::new(ZERO_CURRENT_FILTER_TIME_CONSTANT),
            ],
            bias: [SymmetricUnitInterval::new(0); 3],
            zero_position: UnitInterval::new(0),
            phase_lock_voltage,
            bus_voltage: UnitInterval::new(0),
            pole_pairs,
        }
    }

    /// Records the current bus voltage, as a fraction of the maximum this
    /// board can read — see [`Self::bus_voltage`].
    pub fn set_bus_voltage(&mut self, bus_voltage: UnitInterval) {
        self.bus_voltage = bus_voltage;
    }

    // TODO (no longer motivated by soft-float cost — the workspace now
    // builds for `thumbv7em-none-eabihf`, so this `f32` pipeline already
    // runs on the STM32G4's real single-precision FPU, not a software
    // emulation): `synthesize_voltage`'s Clarke/Park pipeline could still
    // run entirely in fixed point instead, using `SymmetricUnitInterval`'s
    // own Q31 representation throughout (the CORDIC sin/cos it consumes
    // are already fixed-point at the hardware boundary), if some other
    // reason (e.g. determinism, or avoiding the CORDIC round-trip
    // conversions) ever makes that worth revisiting: prescale
    // `i_alpha`/`i_v`/`i_w` (right after bias-subtraction) and `target` by
    // the same `1/4` (a 2-bit shift) before anything else, and no other
    // headroom insertion is needed anywhere else in
    // Clarke/Park/inverse-Park/inverse-Clarke — verified by simulation to
    // keep every intermediate within `[-1, 1)` with margin, and to be an
    // exact `1/4` scaling of today's result (not just differently-scaled:
    // an uneven prescale between the phase-current path and `target` was
    // checked and found to compute a genuinely wrong answer, since
    // anything added/subtracted together must already share the same
    // scale). The compensating `*4` folds into `scale_and_shrink`'s own
    // constant for free.
    //
    /// One current-loop cycle: `u`/`v`/`w` are this cycle's raw phase
    /// current readings, straight off the ADC (mid-scale = zero current);
    /// `physical_angle` is the rotor's current mechanical angle, straight
    /// off the quadrature encoder — reading it is the caller's own
    /// responsibility (see this type's own doc comment), once per call,
    /// regardless of `operation`. Returns the U/V/W duty cycles to drive,
    /// or `None` if nothing should be output this cycle — either
    /// `operation` is [`CurrentLoopOperation::Open`], or the bus voltage
    /// is below [`MIN_BUS_VOLTAGE_FRACTION`] — in which case the caller
    /// should set every phase's duty cycle to `0` instead.
    pub fn poll(
        &mut self,
        u: UnitInterval,
        v: UnitInterval,
        w: UnitInterval,
        physical_angle: UnitInterval,
        operation: CurrentLoopOperation,
    ) -> Option<(UnitInterval, UnitInterval, UnitInterval)> {
        let (u_cmd, v_cmd, w_cmd) = match operation {
            CurrentLoopOperation::Open => {
                for (filter, reading) in self.zero_filters.iter_mut().zip([u, v, w]) {
                    filter.poll(f32::from(reading.expand()));
                }
                self.bias = [
                    SymmetricUnitInterval::from(self.zero_filters[0].output()),
                    SymmetricUnitInterval::from(self.zero_filters[1].output()),
                    SymmetricUnitInterval::from(self.zero_filters[2].output()),
                ];
                return None;
            }
            CurrentLoopOperation::PhaseLock => {
                self.zero_position = physical_angle;
                (
                    self.phase_lock_voltage,
                    SymmetricUnitInterval::new(0),
                    SymmetricUnitInterval::new(0),
                )
            }
            CurrentLoopOperation::CurrentControl(target) => {
                self.synthesize_voltage(u, v, w, physical_angle, target)
            }
        };

        if f32::from(self.bus_voltage) < MIN_BUS_VOLTAGE_FRACTION {
            return None;
        }
        Some((
            self.scale_and_shrink(u_cmd),
            self.scale_and_shrink(v_cmd),
            self.scale_and_shrink(w_cmd),
        ))
    }

    /// Closes the current loop for one cycle: Clarke+Park (sin/cos via the
    /// math coprocessor) turn the bias-corrected phase readings into
    /// measured D/Q current; a unit-gain proportional controller (Vd
    /// targets zero, Vq targets `target`) turns their error into a D/Q
    /// voltage command; inverse Park+Clarke turn that back into the three
    /// phase voltages that synthesize it at the rotor's current angle.
    fn synthesize_voltage(
        &mut self,
        u: UnitInterval,
        v: UnitInterval,
        w: UnitInterval,
        physical_angle: UnitInterval,
        target: SymmetricUnitInterval,
    ) -> (
        SymmetricUnitInterval,
        SymmetricUnitInterval,
        SymmetricUnitInterval,
    ) {
        // Physical angle relative to phase U (see `zero_position`),
        // scaled by `pole_pairs` into the corresponding electrical angle
        // (see `Self::new`'s doc comment), then reinterpreted as a
        // fraction of a half-turn for the math coprocessor — the same
        // cyclic quantity, just re-centered (see `UnitInterval::expand`).
        let angle = ((physical_angle - self.zero_position) * self.pole_pairs).expand();
        self.math_coprocessor
            .compute(MathCoprocessorFunction::SineCosine(angle));
        let (sin, cos) = self.math_coprocessor.result();
        let (sin, cos) = (f32::from(sin), f32::from(cos));

        // Clarke: bias-corrected phase currents -> stationary alpha/beta.
        let i_alpha = f32::from(u.expand().saturating_sub(self.bias[0]));
        let i_v = f32::from(v.expand().saturating_sub(self.bias[1]));
        let i_w = f32::from(w.expand().saturating_sub(self.bias[2]));
        let i_beta = (i_v - i_w) / SQRT_3;

        // Park: alpha/beta -> rotor-synchronous D/Q.
        let i_d = i_alpha * cos + i_beta * sin;
        let i_q = -i_alpha * sin + i_beta * cos;

        let v_d = -i_d;
        let v_q = f32::from(target) - i_q;

        // Inverse Park then inverse Clarke: D/Q voltage command -> the
        // three phase voltages that synthesize it at the current angle.
        let v_alpha = v_d * cos - v_q * sin;
        let v_beta = v_d * sin + v_q * cos;
        (
            SymmetricUnitInterval::from(v_alpha),
            SymmetricUnitInterval::from(-0.5 * v_alpha + (SQRT_3 / 2.0) * v_beta),
            SymmetricUnitInterval::from(-0.5 * v_alpha - (SQRT_3 / 2.0) * v_beta),
        )
    }

    /// Converts `value` (a desired phase voltage, as a fraction of this
    /// board's maximum representable voltage) into the duty cycle that
    /// actually synthesizes it at the *current* bus voltage — dividing,
    /// not multiplying: a half-bridge's average output is `duty *
    /// bus_voltage`, so hitting a fixed target voltage needs *less* duty
    /// spread as the bus voltage rises, not more. Finally maps that
    /// signed voltage fraction to a duty cycle (`0` = zero volts, i.e.
    /// `0.5` duty) — see [`SymmetricUnitInterval::shrink`].
    ///
    /// Safe against dividing by a near-zero `bus_voltage`: [`Self::poll`]
    /// already gates on [`MIN_BUS_VOLTAGE_FRACTION`] before this is ever
    /// called, and [`SymmetricUnitInterval::from`] saturates a result
    /// that would otherwise overflow `[-1, 1)` rather than wrapping.
    fn scale_and_shrink(&self, value: SymmetricUnitInterval) -> UnitInterval {
        SymmetricUnitInterval::from(f32::from(value) / f32::from(self.bus_voltage)).shrink()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use peripherals::fake::math_coprocessor::MathCoprocessor;

    const MID_SCALE: UnitInterval = UnitInterval::new(0x8000_0000); // expand()s to 0
    const FULL_BUS_VOLTAGE: UnitInterval = UnitInterval::new(u32::MAX);
    const TOLERANCE: f32 = 1e-3;

    /// Every test but
    /// [`pole_pairs_scales_the_mechanical_angle_into_the_electrical_angle`]
    /// only cares about `poll`'s behavior at a single fixed angle —
    /// `pole_pairs: 1` (electrical angle == physical angle) keeps their
    /// assertions exactly as simple as before this parameter existed.
    fn foc() -> FieldOrientedControl<MathCoprocessor> {
        foc_with_pole_pairs(1)
    }

    fn foc_with_pole_pairs(pole_pairs: u32) -> FieldOrientedControl<MathCoprocessor> {
        let mut foc = FieldOrientedControl::new(
            MathCoprocessor::new(),
            SymmetricUnitInterval::from(0.1),
            pole_pairs,
        );
        foc.set_bus_voltage(FULL_BUS_VOLTAGE);
        foc
    }

    #[test]
    fn open_returns_none_and_averages_each_channel_to_its_constant_reading() {
        let mut foc = foc();
        let (u, v, w) = (
            UnitInterval::new(0x9000_0000),
            UnitInterval::new(0x7000_0000),
            UnitInterval::new(0x8500_0000),
        );
        for _ in 0..10 {
            assert_eq!(
                foc.poll(u, v, w, UnitInterval::new(0), CurrentLoopOperation::Open),
                None
            );
        }
        assert_eq!(foc.bias, [u.expand(), v.expand(), w.expand()]);
    }

    #[test]
    fn below_minimum_bus_voltage_produces_no_output() {
        let mut foc = foc();
        foc.set_bus_voltage(UnitInterval::from(0.01)); // below the ~5% gate
        assert_eq!(
            foc.poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                UnitInterval::new(0),
                CurrentLoopOperation::PhaseLock
            ),
            None
        );
    }

    #[test]
    fn phase_lock_records_the_given_physical_angle_and_injects_the_configured_voltage() {
        let mut foc = foc();
        let physical_angle = UnitInterval::from(0.3);

        let (u, v, w) = foc
            .poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                physical_angle,
                CurrentLoopOperation::PhaseLock,
            )
            .unwrap();

        // ~0.55 duty (0.1 voltage fraction, shifted to a duty cycle) —
        // approximate: `FULL_BUS_VOLTAGE` is just below `1.0` (`[0, 1)` is
        // right-open), so the bus-voltage scaling isn't quite a no-op.
        assert!((f32::from(u) - 0.55).abs() < TOLERANCE);
        assert_eq!(v, UnitInterval::new(0x8000_0000));
        assert_eq!(w, UnitInterval::new(0x8000_0000));
        assert_eq!(foc.zero_position, physical_angle);
    }

    #[test]
    fn current_control_with_zero_error_produces_zero_output() {
        let mut foc = foc();
        assert_eq!(foc.bias, [SymmetricUnitInterval::new(0); 3]); // no Open window needed: bias already 0

        let (u, v, w) = foc
            .poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                UnitInterval::new(0),
                CurrentLoopOperation::CurrentControl(SymmetricUnitInterval::new(0)), // target current: 0
            )
            .unwrap();

        assert_eq!(u, UnitInterval::new(0x8000_0000));
        assert_eq!(v, UnitInterval::new(0x8000_0000));
        assert_eq!(w, UnitInterval::new(0x8000_0000));
    }

    const CANONICAL_POSITIONS: u32 = 8;

    /// The `k`th of [`CANONICAL_POSITIONS`] evenly-spaced physical
    /// positions around a full turn, as a raw `UnitInterval` fraction —
    /// exact for every `k` here (eighths are exact binary fractions, so
    /// `UnitInterval::from`'s `f32` round trip loses nothing).
    fn canonical_position(k: u32) -> UnitInterval {
        UnitInterval::from(k as f32 / CANONICAL_POSITIONS as f32)
    }

    /// The angle [`synthesize_voltage`] would compute for
    /// [`canonical_position`]`(k)` (with `zero_position` left at its `0`
    /// default, i.e. `pole_pairs: 1` and no
    /// [`CurrentLoopOperation::PhaseLock`] beforehand) — in radians, via
    /// the exact same [`UnitInterval::expand`] this module's own pipeline
    /// uses, so it's the identical `f32` value (not just approximately
    /// equal) [`peripherals::fake::math_coprocessor`]'s `sin`/`cos` will
    /// be asked to evaluate.
    fn canonical_angle_rad(k: u32) -> f32 {
        f32::from(canonical_position(k).expand()) * core::f32::consts::PI
    }

    /// Mirrors [`FieldOrientedControl::scale_and_shrink`], for computing
    /// an expected duty cycle from a plain `f32` symmetric voltage
    /// fraction instead of a [`SymmetricUnitInterval`] — convenient since
    /// the tests below build up expected values via plain `f32` sin/cos
    /// arithmetic first.
    fn expected_duty(symmetric_value: f32, bus_voltage: f32) -> UnitInterval {
        SymmetricUnitInterval::from(symmetric_value / bus_voltage).shrink()
    }

    #[test]
    fn zero_measured_current_makes_the_voltage_vector_track_the_rotor_position() {
        let mut foc = foc();
        let target = 0.4; // arbitrary, nonzero: the point is *some* fixed
                          // target current whose synthesized voltage
                          // vector should visibly rotate with the rotor.
        let bus_voltage = f32::from(FULL_BUS_VOLTAGE);

        for k in 0..CANONICAL_POSITIONS {
            let angle_rad = canonical_angle_rad(k);
            let (sin, cos) = (angle_rad.sin(), angle_rad.cos());

            // Zero measured current (`MID_SCALE` on every phase) means
            // i_d = i_q = 0, so v_d = 0 and v_q = target exactly: the
            // alpha/beta voltage vector is `target` rotated by the
            // rotor's electrical angle — see `synthesize_voltage`'s own
            // Park/Clarke math, mirrored here to build the expected duty
            // cycles independently of calling it.
            let v_alpha = -target * sin;
            let v_beta = target * cos;

            let (u, v, w) = foc
                .poll(
                    MID_SCALE,
                    MID_SCALE,
                    MID_SCALE,
                    canonical_position(k),
                    CurrentLoopOperation::CurrentControl(SymmetricUnitInterval::from(target)),
                )
                .unwrap();

            assert!(
                (f32::from(u) - f32::from(expected_duty(v_alpha, bus_voltage))).abs() < TOLERANCE,
                "k={k}"
            );
            assert!(
                (f32::from(v)
                    - f32::from(expected_duty(
                        -0.5 * v_alpha + (SQRT_3 / 2.0) * v_beta,
                        bus_voltage
                    )))
                .abs()
                    < TOLERANCE,
                "k={k}"
            );
            assert!(
                (f32::from(w)
                    - f32::from(expected_duty(
                        -0.5 * v_alpha - (SQRT_3 / 2.0) * v_beta,
                        bus_voltage
                    )))
                .abs()
                    < TOLERANCE,
                "k={k}"
            );
        }
    }

    #[test]
    fn scale_and_shrink_divides_by_bus_voltage_rather_than_multiplying() {
        // At `FULL_BUS_VOLTAGE` (~1.0), dividing and multiplying are
        // indistinguishable — every other test above uses that bus
        // voltage, so none of them would catch `scale_and_shrink`
        // regressing back to multiplying. Half bus voltage makes the two
        // directions produce clearly different results: dividing needs
        // *more* duty spread to reach the same target voltage, not less.
        let mut foc = foc();
        foc.set_bus_voltage(UnitInterval::from(0.5));
        let target = 0.4;

        // Zero measured current (`MID_SCALE`) and physical angle `0`
        // makes this the same closed-form v_alpha/v_beta as
        // `zero_measured_current_makes_the_voltage_vector_track_the_rotor_position`,
        // just at one fixed angle instead of swept across eight.
        let angle_rad = f32::from(UnitInterval::new(0).expand()) * core::f32::consts::PI;
        let (sin, cos) = (angle_rad.sin(), angle_rad.cos());
        let v_alpha = -target * sin;
        let v_beta = target * cos;

        let (u, v, w) = foc
            .poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                UnitInterval::new(0),
                CurrentLoopOperation::CurrentControl(SymmetricUnitInterval::from(target)),
            )
            .unwrap();

        assert!((f32::from(u) - f32::from(expected_duty(v_alpha, 0.5))).abs() < TOLERANCE);
        assert!(
            (f32::from(v)
                - f32::from(expected_duty(-0.5 * v_alpha + (SQRT_3 / 2.0) * v_beta, 0.5)))
            .abs()
                < TOLERANCE
        );
        assert!(
            (f32::from(w)
                - f32::from(expected_duty(-0.5 * v_alpha - (SQRT_3 / 2.0) * v_beta, 0.5)))
            .abs()
                < TOLERANCE
        );
    }

    #[test]
    fn measured_current_matching_the_target_produces_50_percent_duty_at_every_canonical_angle() {
        let mut foc = foc();
        let target = 0.4; // arbitrary, nonzero: the point is that a
                          // nonzero but *correctly tracked* current
                          // still nets out to a neutral duty cycle.

        for k in 0..CANONICAL_POSITIONS {
            let angle_rad = canonical_angle_rad(k);
            let (sin, cos) = (angle_rad.sin(), angle_rad.cos());

            // Construct the raw phase-current readings that Clarke/Park,
            // at this exact angle, recover as (i_d, i_q) = (0, target) —
            // inverse Park (d=0, q=target) into alpha/beta, then inverse
            // Clarke into three balanced phase readings, the same way
            // `synthesize_voltage` turns a d/q *voltage* command into
            // three phase *voltages*. With bias still `0` (no `Open`
            // window run), these round-trip back through
            // `u.expand()`/`v.expand()`/`w.expand()` exactly.
            let i_alpha = -target * sin;
            let i_beta = target * cos;
            let u = SymmetricUnitInterval::from(i_alpha).shrink();
            let v = SymmetricUnitInterval::from(-0.5 * i_alpha + (SQRT_3 / 2.0) * i_beta).shrink();
            let w = SymmetricUnitInterval::from(-0.5 * i_alpha - (SQRT_3 / 2.0) * i_beta).shrink();

            let (u, v, w) = foc
                .poll(
                    u,
                    v,
                    w,
                    canonical_position(k),
                    CurrentLoopOperation::CurrentControl(SymmetricUnitInterval::from(target)),
                )
                .unwrap();

            assert!((f32::from(u) - 0.5).abs() < TOLERANCE, "k={k}");
            assert!((f32::from(v) - 0.5).abs() < TOLERANCE, "k={k}");
            assert!((f32::from(w) - 0.5).abs() < TOLERANCE, "k={k}");
        }
    }

    #[test]
    fn pole_pairs_scales_the_mechanical_angle_into_the_electrical_angle() {
        // With 2 pole pairs, one electrical revolution happens every half
        // mechanical revolution -- so the synthesized output at physical
        // angle 0 should exactly repeat at physical angle 0.5 (half a
        // turn away), even though the *mechanical* angle is different. A
        // nonzero target current is what makes the output actually
        // depend on angle at all (see
        // `current_control_with_zero_error_produces_zero_output`).
        let mut foc = foc_with_pole_pairs(2);
        let (u, v, w) = (
            UnitInterval::new(0x9000_0000),
            UnitInterval::new(0x7000_0000),
            UnitInterval::new(0x8500_0000),
        );
        let operation = CurrentLoopOperation::CurrentControl(SymmetricUnitInterval::from(0.3));

        let at_physical_zero = foc.poll(u, v, w, UnitInterval::new(0), operation).unwrap();
        let after_half_a_turn = foc
            .poll(u, v, w, UnitInterval::new(0x8000_0000), operation)
            .unwrap();

        assert_eq!(at_physical_zero, after_half_a_turn);
    }

    #[test]
    #[should_panic(expected = "pole_pairs must be at least 1")]
    fn new_panics_if_pole_pairs_is_zero() {
        foc_with_pole_pairs(0);
    }
}
