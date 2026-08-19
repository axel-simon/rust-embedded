//! A fake [`ClockProviderTrait`]/[`ClockTrait`] implementation for
//! host-side testing, with no real hardware involved.
//!
//! [`ClockProvider::new`] hands back two handles onto one shared,
//! simulated clock: [`ClockProvider`] itself, for firmware, and
//! [`FakeClockProvider`], for a test to read the current simulated time
//! or jump it forward directly.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use std::rc::Rc;

use common::duration::Duration;
use common::duration_from_ticks::DurationFromTicks;
use common::uptime::Uptime;

use crate::api::clock::{ClockProviderTrait, ClockTrait};

/// The simulated free-running tick counter and the reference point every
/// [`ClockFake`]/[`FakeClockProvider`] view reads from — shared between a
/// [`ClockProvider`] and its [`FakeClockProvider`] counterpart (see
/// [`ClockProvider::new`]).
struct ClockProviderState {
    ticks_per_second: u32,
    ticks: AtomicU32,
    reference: RefCell<DurationFromTicks>,
}

/// Owns (a handle onto) the simulated free-running tick counter and the
/// reference point every [`ClockFake`] view reads from. Create one per
/// test/application, call [`Self::advance_reference_point`] (via
/// [`ClockProviderTrait::advance_reference_point`]) whenever simulated time
/// should move forward, and hand out [`ClockFake`] views with
/// [`Self::get_clock`].
pub struct ClockProvider(Rc<ClockProviderState>);

/// A test's handle onto the same simulated clock a [`ClockProvider`]
/// drives — see [`ClockProvider::new`]. Cheap to [`Clone`] (all clones
/// share the same underlying state).
#[derive(Clone)]
pub struct FakeClockProvider(Rc<ClockProviderState>);

impl ClockProvider {
    /// Creates a fake clock provider simulating an MCU running at
    /// `mcu_frequency` Hz, with `ticks_now()` starting at 1 (see
    /// [`ClockTrait::ticks_now`]) and its reference point at time zero,
    /// plus a [`FakeClockProvider`] handle onto the same simulated clock.
    ///
    /// Takes (and ignores) one leading argument of any type, mirroring
    /// [`crate::stm32g4::clock::ClockProvider::new`]'s first
    /// `cortex_m::Peripherals` parameter — a generic parameter rather than
    /// a concrete `cortex_m::Peripherals` so this module never needs to
    /// depend on the `cortex-m` crate at all, keeping it platform-agnostic.
    /// Callers that don't have anything meaningful to pass can just pass `()`.
    pub fn new<T>(_unused: T, mcu_frequency: u32) -> (Self, FakeClockProvider) {
        let state = Rc::new(ClockProviderState {
            ticks_per_second: mcu_frequency,
            ticks: AtomicU32::new(0),
            reference: RefCell::new(DurationFromTicks::new(mcu_frequency)),
        });
        (ClockProvider(state.clone()), FakeClockProvider(state))
    }

    /// Advances the simulated counter by one tick and returns it — shared
    /// by [`Self::advance_reference_point`] and every [`ClockFake`] view's
    /// [`ClockTrait::ticks_now`], since they're all reading the same
    /// simulated hardware counter.
    fn ticks_now(&self) -> u32 {
        self.0.ticks.fetch_add(1, Ordering::Relaxed).wrapping_add(1)
    }
}

impl<'a> ClockProviderTrait<'a, ClockFake<'a>> for ClockProvider {
    fn get_clock(&'a self) -> ClockFake<'a> {
        ClockFake {
            ticks_per_second: self.0.ticks_per_second,
            ticks: &self.0.ticks,
            reference: &self.0.reference,
        }
    }

    fn advance_reference_point(&self) {
        let ticks = self.ticks_now();
        self.0.reference.borrow_mut().advance_to(ticks);
    }
}

impl FakeClockProvider {
    /// Reads the current simulated uptime, without perturbing the
    /// simulated tick counter — unlike [`ClockTrait::ticks_now`], which
    /// deliberately advances by one tick on every read (see its caller,
    /// [`ClockTrait::now`]), this is a pure read, so inspecting the time
    /// from a test never itself advances the simulation.
    pub fn now(&self) -> Duration {
        self.0.reference.borrow().now()
    }

    /// Jumps simulated time forward by `duration` directly, without a
    /// firmware handle needing to call
    /// [`ClockTrait::wait_for`]/
    /// [`ClockProviderTrait::advance_reference_point`] — keeps the raw
    /// tick counter (which [`ClockFake::ticks_now`] reads)
    /// and the reference point in sync, the same invariant
    /// [`ClockProviderTrait::advance_reference_point`] maintains.
    pub fn advance_by(&mut self, duration: Duration) {
        let delta_ticks = ((duration.raw() as i128 * self.0.ticks_per_second as i128) >> 32) as u32;
        self.0.ticks.fetch_add(delta_ticks, Ordering::Relaxed);
        self.0.reference.borrow_mut().advance_by(delta_ticks);
    }
}

/// A lightweight, read-only [`ClockTrait`] view onto a [`ClockProvider`]
/// — see [`ClockProviderTrait::get_clock`].
pub struct ClockFake<'a> {
    ticks_per_second: u32,
    ticks: &'a AtomicU32,
    reference: &'a RefCell<DurationFromTicks>,
}

impl<'a> ClockTrait for ClockFake<'a> {
    fn ticks_now(&self) -> u32 {
        self.ticks.fetch_add(1, Ordering::Relaxed).wrapping_add(1)
    }

    fn time_at(&self, ticks: u32) -> Uptime {
        Uptime::epoch() + self.reference.borrow().time_at(ticks)
    }

    fn ticks_per_second(&self) -> u32 {
        self.ticks_per_second
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICKS_PER_SECOND: u32 = 170_000_000;

    #[test]
    fn ticks_now_increments_by_one_each_call() {
        let (provider, _fake) = ClockProvider::new((), TICKS_PER_SECOND);
        let clock = provider.get_clock();
        assert_eq!(clock.ticks_now(), 1);
        assert_eq!(clock.ticks_now(), 2);
        assert_eq!(clock.ticks_now(), 3);
    }

    #[test]
    fn ticks_per_second_matches_what_new_was_given() {
        let (provider, _fake) = ClockProvider::new((), TICKS_PER_SECOND);
        let clock = provider.get_clock();
        assert_eq!(clock.ticks_per_second(), TICKS_PER_SECOND);
    }

    #[test]
    fn now_reflects_ticks_observed_since_the_last_reference_point() {
        let (provider, _fake) = ClockProvider::new((), TICKS_PER_SECOND);
        let clock = provider.get_clock();
        for _ in 0..999 {
            clock.ticks_now();
        }
        // The 1000th tick, consumed by `now()` itself, read against a
        // reference point that's never moved past tick 0.
        let uptime = clock.now();

        let mut reference = DurationFromTicks::new(TICKS_PER_SECOND);
        reference.advance_to(1000);
        assert_eq!(uptime, Uptime::epoch() + reference.now());
    }

    #[test]
    fn advance_reference_point_keeps_now_accurate_after_many_ticks() {
        let (provider, _fake) = ClockProvider::new((), TICKS_PER_SECOND);
        // Move the shared counter far enough that a naive single-shot
        // conversion from `now()`'s reference point (fixed at 0) would be
        // way off, then catch the reference point up.
        for _ in 0..10_000 {
            provider.ticks_now();
        }
        provider.advance_reference_point(); // reference point now at tick 10_001

        let clock = provider.get_clock();
        let uptime = clock.now(); // one more tick: 10_002

        let mut reference = DurationFromTicks::new(TICKS_PER_SECOND);
        reference.advance_to(10_002);
        assert_eq!(uptime, Uptime::epoch() + reference.now());
    }

    #[test]
    fn time_at_converts_a_tick_count_without_calling_ticks_now() {
        let (provider, _fake) = ClockProvider::new((), TICKS_PER_SECOND);
        provider.advance_reference_point(); // reference point at tick 1
        let clock = provider.get_clock();

        // `time_at` takes a plain `u32` tick count; the caller can supply
        // one that wasn't just read via `ticks_now()`.
        let uptime = clock.time_at(501);

        let mut reference = DurationFromTicks::new(TICKS_PER_SECOND);
        reference.advance_to(501);
        assert_eq!(uptime, Uptime::epoch() + reference.now());
    }

    #[test]
    fn wait_for_blocks_until_the_deadline_has_passed() {
        let (provider, _fake) = ClockProvider::new((), TICKS_PER_SECOND);
        let clock = provider.get_clock();
        let before = clock.now();

        // A quarter of a millisecond's worth of ticks at 170 MHz — small
        // enough to keep the busy loop's iteration count (and so this
        // test's run time) modest, but still several thousand ticks.
        let delay = Duration::new(1 << 20);
        clock.wait_for(delay);

        assert!(clock.now() >= before + delay);
    }

    #[test]
    fn fake_now_matches_provider_and_does_not_perturb_ticks() {
        let (provider, fake) = ClockProvider::new((), TICKS_PER_SECOND);
        let clock = provider.get_clock();
        assert_eq!(clock.ticks_now(), 1);

        // Reading `fake.now()` several times shouldn't advance the raw
        // tick counter `ClockFake::ticks_now()` reads.
        let _ = fake.now();
        let _ = fake.now();
        assert_eq!(clock.ticks_now(), 2);
    }

    #[test]
    fn fake_advance_by_moves_simulated_time_forward() {
        let (provider, mut fake) = ClockProvider::new((), TICKS_PER_SECOND);
        let before = fake.now();

        fake.advance_by(Duration::from_millis(500));

        assert_eq!(fake.now() - before, Duration::from_millis(500));
        // Visible through the firmware-facing handle too — `>=` rather
        // than exact equality, since reading `now()` there also advances
        // the simulated tick counter by one further tick (see
        // `ClockFake::ticks_now`'s doc comment).
        assert!(provider.get_clock().now() >= Uptime::epoch() + fake.now());
    }
}
