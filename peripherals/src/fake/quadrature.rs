//! A fake [`QuadratureTrait`] implementation for host-side testing, with
//! no real hardware involved.
//!
//! [`Quadrature::new`] hands back two handles onto one shared, simulated
//! encoder: [`Quadrature`] itself, for firmware — its
//! [`QuadratureTrait::open`]/[`QuadratureTrait::position`] read/write the
//! same simulated state a [`FakeQuadrature`] drives — and
//! [`FakeQuadrature`], for a test to simulate the encoder turning (via
//! [`FakeQuadrature::move_by`] or [`FakeQuadrature::set_encoder_reading`]).

use core::cell::Cell;
use std::rc::Rc;

use common::i64_divider::U64Divider;
use common::unit_interval::UnitInterval;

use crate::api::quadrature::{QuadratureOptions, QuadratureTimer, QuadratureTrait};

/// The simulated encoder's state while [`QuadratureTrait::open`] — see
/// [`QuadratureState::open`].
#[derive(Clone, Copy)]
struct OpenState {
    encoder_counts: u32,
    /// Precomputed reciprocal of `encoder_counts`, for
    /// [`QuadratureState::position_as_unit_interval`] — same reasoning
    /// (and the same `.max(2)` panic-avoidance for a power-of-two
    /// `encoder_counts`, possibly `1`) as
    /// [`crate::stm32g4::quadrature::Quadrature`]'s own `divider` field.
    divider: U64Divider,
}

/// The simulated encoder state shared between a [`Quadrature`] and its
/// [`FakeQuadrature`] counterpart (see [`Quadrature::new`]).
struct QuadratureState {
    /// Set by [`QuadratureTrait::open`], cleared by
    /// [`QuadratureTrait::close`] — mirrors
    /// [`crate::stm32g4::quadrature::Quadrature`]'s own `open` field.
    /// `Cell<Option<_>>` rather than a plain field since every method here
    /// takes `&self`, not `&mut self` (see [`Quadrature`]'s doc comment).
    open: Cell<Option<OpenState>>,
    /// Current simulated position, always in `0..encoder_counts` — see
    /// [`FakeQuadrature::move_by`]. Meaningless while `open` is `None`.
    position: Cell<u32>,
}

impl QuadratureState {
    fn open(&self) -> OpenState {
        self.open
            .get()
            .expect("called before open() succeeded on the corresponding Quadrature")
    }

    fn position_as_unit_interval(&self) -> UnitInterval {
        let open = self.open();
        let raw = self.position.get() as u64;
        if open.encoder_counts.is_power_of_two() {
            let shift = 32 - open.encoder_counts.trailing_zeros();
            UnitInterval::new((raw << shift) as u32)
        } else {
            UnitInterval::new(open.divider.divide(raw << 32) as u32)
        }
    }

    /// Applies a signed `delta` (in raw counts) to the simulated position,
    /// wrapping at `encoder_counts` — shared by [`FakeQuadrature::move_by`]
    /// and [`FakeQuadrature::set_encoder_reading`].
    fn apply_delta(&self, delta: i64) {
        let counts = self.open().encoder_counts as i64;
        let new_position = self.position.get() as i64 + delta;
        self.position.set(new_position.rem_euclid(counts) as u32);
    }
}

/// A fake [`QuadratureTrait`] driver simulating an encoder's position,
/// without touching any real hardware. Every [`QuadratureTrait`] method
/// only takes `&self` here (unlike the trait's own `&mut self` signatures)
/// since the underlying state is `Rc`-shared with a [`FakeQuadrature`] —
/// see [`QuadratureState::open`]'s `Cell` field.
pub struct Quadrature(Rc<QuadratureState>);

/// A test's handle onto the same simulated encoder a [`Quadrature`] reads
/// — see [`Quadrature::new`]. Cheap to [`Clone`] (all clones share the same
/// underlying state).
#[derive(Clone)]
pub struct FakeQuadrature(Rc<QuadratureState>);

impl Quadrature {
    /// Creates an unopened fake encoder (see [`QuadratureTrait::open`)],
    /// plus a [`FakeQuadrature`] handle a test can drive it with. `timer`
    /// is accepted (and ignored) — there's no real timer to select here —
    /// purely to mirror
    /// [`crate::stm32g4::quadrature::Quadrature::new`]'s signature, the
    /// same way [`crate::fake::clock::ClockProvider::new`]'s leading
    /// argument mirrors its own real counterpart.
    pub fn new(_timer: QuadratureTimer) -> (Self, FakeQuadrature) {
        let state = Rc::new(QuadratureState {
            open: Cell::new(None),
            position: Cell::new(0),
        });
        (Quadrature(state.clone()), FakeQuadrature(state))
    }
}

impl QuadratureTrait for Quadrature {
    fn open(&mut self, options: QuadratureOptions) {
        self.0.open.set(Some(OpenState {
            encoder_counts: options.encoder_counts,
            divider: U64Divider::new(options.encoder_counts.max(2)),
        }));
        self.0.position.set(0); // mirrors the real driver resetting CNT to 0.
    }

    fn close(&mut self) {
        self.0.open.set(None);
    }

    fn position(&self) -> UnitInterval {
        self.0.position_as_unit_interval()
    }
}

impl FakeQuadrature {
    /// Simulates the encoder turning by `delta` counts (positive = the
    /// counting-up direction described in
    /// [`crate::api::quadrature::QuadratureInputConfiguration`]'s doc
    /// comment, negative = reverse), wrapping the simulated position at
    /// `encoder_counts`.
    ///
    /// # Panics
    /// Panics if the corresponding [`Quadrature`] hasn't had
    /// [`QuadratureTrait::open`] called on it yet.
    pub fn move_by(&mut self, delta: i32) {
        self.0.apply_delta(delta as i64);
    }

    /// Jumps the simulated encoder directly to `reading` — a raw count in
    /// `0..encoder_counts`, e.g. as if it had been read straight off the
    /// counter — by whichever of the two directions (counting up or down)
    /// covers less distance from the current position (a tie, exactly half
    /// an `encoder_counts` cycle away, resolves to counting up).
    ///
    /// # Panics
    /// Panics if `reading >= encoder_counts`, or if the corresponding
    /// [`Quadrature`] hasn't had [`QuadratureTrait::open`] called on it
    /// yet.
    pub fn set_encoder_reading(&mut self, reading: u32) {
        let counts = self.0.open().encoder_counts;
        assert!(
            reading < counts,
            "reading must be less than encoder_counts ({counts}), got {reading}"
        );

        let counts = counts as i64;
        let current = self.0.position.get() as i64;
        let target = reading as i64;

        // Distance travelling up (0..counts) vs. down (0..counts); the
        // shorter one wins, up on a tie.
        let up = (target - current).rem_euclid(counts);
        let down = (current - target).rem_euclid(counts);
        let delta = if up <= down { up } else { -down };

        self.0.apply_delta(delta);
    }

    /// Reads the encoder's current simulated position, without perturbing
    /// any state.
    ///
    /// # Panics
    /// Panics if the corresponding [`Quadrature`] hasn't had
    /// [`QuadratureTrait::open`] called on it yet.
    pub fn position(&self) -> UnitInterval {
        self.0.position_as_unit_interval()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn options(encoder_counts: u32) -> QuadratureOptions {
        use crate::api::quadrature::QuadratureInputConfiguration;

        QuadratureOptions {
            input_configuration: QuadratureInputConfiguration::Ch12AreInputsAB,
            encoder_counts,
        }
    }

    #[test]
    fn starts_at_position_zero() {
        let (mut quad, _fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        assert_eq!(quad.position(), UnitInterval::new(0));
    }

    #[test]
    #[should_panic(expected = "called before open() succeeded")]
    fn position_panics_before_open() {
        let (quad, _fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.position();
    }

    #[test]
    fn move_by_advances_position_proportionally() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.move_by(250);
        // 250 / 1000 of a full cycle.
        assert_eq!(quad.position(), UnitInterval::new(u32::MAX / 4 + 1));
    }

    #[test]
    fn move_by_wraps_forward_at_encoder_counts() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.move_by(1200);
        assert_eq!(fake.position(), quad.position());
        // Wrapped once; left at 200/1000 of a cycle.
        let expected = (((200u64) << 32) / 1000) as u32;
        assert_eq!(fake.position(), UnitInterval::new(expected));
    }

    #[test]
    fn move_by_wraps_backward_below_zero() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.move_by(-1);
        // Wrapped back to the last count, 999/1000 of a cycle.
        let expected = (((999u64) << 32) / 1000) as u32;
        assert_eq!(fake.position(), UnitInterval::new(expected));
    }

    #[test]
    fn open_resets_position_to_zero() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.move_by(500);
        quad.open(options(1000));
        assert_eq!(quad.position(), UnitInterval::new(0));
    }

    #[test]
    fn close_then_reopen_requires_position_to_wait_for_the_new_open() {
        let (mut quad, _fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        quad.close();
        quad.open(options(500));
        assert_eq!(quad.position(), UnitInterval::new(0));
    }

    #[test]
    fn set_encoder_reading_moves_to_the_given_reading() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.set_encoder_reading(400);
        let expected = (((400u64) << 32) / 1000) as u32;
        assert_eq!(quad.position(), UnitInterval::new(expected));
    }

    #[test]
    fn set_encoder_reading_takes_the_shorter_forward_path() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.set_encoder_reading(100); // from 0, forward 100 is shorter than backward 900
        let expected = (((100u64) << 32) / 1000) as u32;
        assert_eq!(quad.position(), UnitInterval::new(expected));
    }

    #[test]
    fn set_encoder_reading_takes_the_shorter_backward_path() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        // From 0, counting down 100 (wrapping through the zero position) is
        // shorter than counting up 900.
        fake.set_encoder_reading(900);
        let expected = (((900u64) << 32) / 1000) as u32;
        assert_eq!(quad.position(), UnitInterval::new(expected));
    }

    #[test]
    fn set_encoder_reading_breaks_an_exact_tie_by_counting_up() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        // Exactly half a cycle away either direction; ties resolve to
        // counting up (0 -> 500).
        fake.set_encoder_reading(500);
        let expected = (((500u64) << 32) / 1000) as u32;
        assert_eq!(quad.position(), UnitInterval::new(expected));
    }

    #[test]
    #[should_panic(expected = "reading must be less than encoder_counts")]
    fn set_encoder_reading_panics_on_a_reading_at_or_past_encoder_counts() {
        let (mut quad, mut fake) = Quadrature::new(QuadratureTimer::Stm32g4Tim1);
        quad.open(options(1000));
        fake.set_encoder_reading(1000);
    }
}
