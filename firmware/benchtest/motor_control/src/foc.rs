//! Field-oriented control: per-phase zero-current calibration, rotor
//! electrical-angle tracking via a quadrature encoder, and closed-loop
//! current regulation through the Clarke/Park transform.

use common::filter::LowPassFilter;
use common::unit_interval::{SymmetricUnitInterval, UnitInterval};
// `backend::x::Y` resolves to `peripherals::stm32g4::x::Y` on real
// hardware or `peripherals::fake::x::Y` elsewhere — see
// `esc1_discovery::backend`'s own doc comment.
use esc1_discovery::backend::{math_coprocessor::MathCoprocessor, quadrature::Quadrature};
use peripherals::api::math_coprocessor::{MathCoprocessorFunction, MathCoprocessorTrait};
use peripherals::api::quadrature::QuadratureTrait;

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
/// the main (DS402) loop via shared state. `pub`, not private: real RTIC's
/// generated per-resource proxy type (for `#[shared]`'s
/// `current_loop_operation` field, in `mod app`) is itself `pub`, and
/// Rust's "private type in public interface" check requires this type to
/// be at least as visible as that — `pub(crate)` alone isn't enough.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentLoopOperation {
    /// No PWM output — every phase's zero-current filter is updated
    /// instead. See [`FieldOrientedControl::poll`].
    Open,
    /// Injects [`FieldOrientedControl::new`]'s `phase_lock_voltage` into
    /// phase U to align the rotor at a known electrical angle, and
    /// samples the quadrature encoder as the new zero position.
    ///
    /// Not yet commanded by this app's DS402 state machine — like
    /// `VoltageControl` once was, when/how to trigger an alignment pass
    /// is deferred.
    #[allow(dead_code)]
    PhaseLock,
    /// Closed-loop current control, targeting this fraction of maximum
    /// current on the rotor's Q axis (mid-scale, `0.5`, is zero — see
    /// [`UnitInterval::expand`]).
    CurrentControl(UnitInterval),
}

/// Field-oriented control of a three-phase (U/V/W) motor.
///
/// Owns the quadrature driver (rotor angle) and the math coprocessor
/// (sin/cos for the Clarke/Park transform); [`Self::poll`] is the whole
/// interface, called once per current-loop cycle with each phase's raw
/// ADC reading.
pub struct FieldOrientedControl {
    quadrature: Quadrature,
    math_coprocessor: MathCoprocessor,
    /// Per-phase (U, V, W) zero-current filters, updated while
    /// [`CurrentLoopOperation::Open`] — see [`Self::poll`].
    zero_filters: [LowPassFilter; 3],
    /// Each phase's most recent zero-current filter output, already
    /// shifted into the same domain [`UnitInterval::expand`] produces —
    /// storing it pre-shifted avoids re-shifting it on every non-`Open`
    /// [`Self::poll`] call instead.
    bias: [SymmetricUnitInterval; 3],
    /// The quadrature reading [`CurrentLoopOperation::PhaseLock`] last
    /// sampled as electrical angle zero — subtracted from every later
    /// reading to get an angle relative to phase U.
    zero_position: UnitInterval,
    /// The voltage injected into phase U during
    /// [`CurrentLoopOperation::PhaseLock`] — see [`Self::new`].
    phase_lock_voltage: SymmetricUnitInterval,
    /// Set by [`Self::set_bus_voltage`]; scales every PWM output and
    /// gates it off entirely below [`MIN_BUS_VOLTAGE_FRACTION`].
    bus_voltage: UnitInterval,
}

impl FieldOrientedControl {
    /// `quadrature` must already be open (see
    /// [`QuadratureTrait::open`]) — this only takes ownership of it, the
    /// same way `bring_up` already owns/opens every other driver before
    /// handing it off.
    pub fn new(
        quadrature: Quadrature,
        math_coprocessor: MathCoprocessor,
        phase_lock_voltage: SymmetricUnitInterval,
    ) -> Self {
        FieldOrientedControl {
            quadrature,
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
        }
    }

    /// Records the current bus voltage, as a fraction of the maximum this
    /// board can read — see [`Self::bus_voltage`].
    pub fn set_bus_voltage(&mut self, bus_voltage: UnitInterval) {
        self.bus_voltage = bus_voltage;
    }

    /// One current-loop cycle: `u`/`v`/`w` are this cycle's raw phase
    /// current readings, straight off the ADC (mid-scale = zero current).
    /// Returns the U/V/W duty cycles to drive, or `None` if nothing
    /// should be output this cycle — either `operation` is
    /// [`CurrentLoopOperation::Open`], or the bus voltage is below
    /// [`MIN_BUS_VOLTAGE_FRACTION`] — in which case the caller should set
    /// every phase's duty cycle to `0` instead.
    pub fn poll(
        &mut self,
        u: UnitInterval,
        v: UnitInterval,
        w: UnitInterval,
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
                self.zero_position = self.quadrature.position();
                (
                    self.phase_lock_voltage,
                    SymmetricUnitInterval::new(0),
                    SymmetricUnitInterval::new(0),
                )
            }
            CurrentLoopOperation::CurrentControl(target) => {
                self.synthesize_voltage(u, v, w, target)
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
        target: UnitInterval,
    ) -> (
        SymmetricUnitInterval,
        SymmetricUnitInterval,
        SymmetricUnitInterval,
    ) {
        // Electrical angle relative to phase U (see `zero_position`),
        // reinterpreted as a fraction of a half-turn for the math
        // coprocessor — the same cyclic quantity, just re-centered (see
        // `UnitInterval::expand`).
        let angle = (self.quadrature.position() - self.zero_position).expand();
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
        let v_q = f32::from(target.expand()) - i_q;

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

    /// Scales `value` by the current bus voltage, then maps it from a
    /// signed voltage fraction to a duty cycle (`0` = zero volts, i.e.
    /// `0.5` duty) — see [`SymmetricUnitInterval::shrink`].
    fn scale_and_shrink(&self, value: SymmetricUnitInterval) -> UnitInterval {
        SymmetricUnitInterval::from(f32::from(value) * f32::from(self.bus_voltage)).shrink()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esc1_discovery::backend::quadrature::FakeQuadrature;
    use peripherals::api::quadrature::{
        QuadratureInputConfiguration, QuadratureOptions, QuadratureTimer,
    };

    const MID_SCALE: UnitInterval = UnitInterval::new(0x8000_0000); // expand()s to 0
    const FULL_BUS_VOLTAGE: UnitInterval = UnitInterval::new(u32::MAX);
    const TOLERANCE: f32 = 1e-3;

    fn foc() -> (FieldOrientedControl, FakeQuadrature) {
        let (mut quadrature, fake_quadrature) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quadrature.open(QuadratureOptions {
            input_configuration: QuadratureInputConfiguration::Ch12AreInputsAB,
            encoder_counts: 128,
        });
        let mut foc = FieldOrientedControl::new(
            quadrature,
            MathCoprocessor::new(),
            SymmetricUnitInterval::from(0.1),
        );
        foc.set_bus_voltage(FULL_BUS_VOLTAGE);
        (foc, fake_quadrature)
    }

    #[test]
    fn open_returns_none_and_averages_each_channel_to_its_constant_reading() {
        let (mut foc, _fake) = foc();
        let (u, v, w) = (
            UnitInterval::new(0x9000_0000),
            UnitInterval::new(0x7000_0000),
            UnitInterval::new(0x8500_0000),
        );
        for _ in 0..10 {
            assert_eq!(foc.poll(u, v, w, CurrentLoopOperation::Open), None);
        }
        assert_eq!(foc.bias, [u.expand(), v.expand(), w.expand()]);
    }

    #[test]
    fn below_minimum_bus_voltage_produces_no_output() {
        let (mut foc, _fake) = foc();
        foc.set_bus_voltage(UnitInterval::from(0.01)); // below the ~5% gate
        assert_eq!(
            foc.poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                CurrentLoopOperation::PhaseLock
            ),
            None
        );
    }

    #[test]
    fn phase_lock_injects_the_configured_voltage_into_u_and_zeroes_v_w() {
        let (mut foc, mut fake_quadrature) = foc();
        fake_quadrature.move_by(30);

        let (u, v, w) = foc
            .poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                CurrentLoopOperation::PhaseLock,
            )
            .unwrap();

        // ~0.55 duty (0.1 voltage fraction, shifted to a duty cycle) —
        // approximate: `FULL_BUS_VOLTAGE` is just below `1.0` (`[0, 1)` is
        // right-open), so the bus-voltage scaling isn't quite a no-op.
        assert!((f32::from(u) - 0.55).abs() < TOLERANCE);
        assert_eq!(v, UnitInterval::new(0x8000_0000));
        assert_eq!(w, UnitInterval::new(0x8000_0000));
        assert_eq!(foc.zero_position, fake_quadrature.position());
    }

    #[test]
    fn current_control_with_zero_error_produces_zero_output() {
        let (mut foc, _fake) = foc();
        assert_eq!(foc.bias, [SymmetricUnitInterval::new(0); 3]); // no Open window needed: bias already 0

        let (u, v, w) = foc
            .poll(
                MID_SCALE,
                MID_SCALE,
                MID_SCALE,
                CurrentLoopOperation::CurrentControl(MID_SCALE), // target current: 0
            )
            .unwrap();

        assert_eq!(u, UnitInterval::new(0x8000_0000));
        assert_eq!(v, UnitInterval::new(0x8000_0000));
        assert_eq!(w, UnitInterval::new(0x8000_0000));
    }
}
