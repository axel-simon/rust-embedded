//! A fake [`ClockProviderTrait`]/[`ClockTrait`] implementation for
//! host-side testing, with no real hardware involved.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use common::duration_from_ticks::DurationFromTicks;
use common::uptime::Uptime;

use crate::api::clock::{ClockProviderTrait, ClockTrait};

/// The simulated MCU frequency: 170 MHz, the real STM32G431's maximum
/// SYSCLK.
const TICKS_PER_SECOND: u32 = 170_000_000;

/// Owns the simulated free-running tick counter and the reference point
/// every [`ClockFake`] view reads from. Create one per test/application,
/// call [`Self::advance_reference_point`] (via
/// [`ClockProviderTrait::advance_reference_point`]) whenever simulated time
/// should move forward, and hand out [`ClockFake`] views with
/// [`Self::get_clock`].
pub struct ClockProviderFake {
    ticks: AtomicU32,
    reference: RefCell<DurationFromTicks>,
}

impl ClockProviderFake {
    /// Creates a fake clock provider with `ticks_now()` starting at 1 (see
    /// [`ClockTrait::ticks_now`]) and its reference point at time zero.
    pub fn new() -> Self {
        ClockProviderFake {
            ticks: AtomicU32::new(0),
            reference: RefCell::new(DurationFromTicks::new(TICKS_PER_SECOND)),
        }
    }

    /// Advances the simulated counter by one tick and returns it — shared
    /// by [`Self::advance_reference_point`] and every [`ClockFake`] view's
    /// [`ClockTrait::ticks_now`], since they're all reading the same
    /// simulated hardware counter.
    fn ticks_now(&self) -> u32 {
        self.ticks.fetch_add(1, Ordering::Relaxed).wrapping_add(1)
    }
}

impl<'a> ClockProviderTrait<'a, ClockFake<'a>> for ClockProviderFake {
    fn get_clock(&'a self) -> ClockFake<'a> {
        ClockFake {
            ticks: &self.ticks,
            reference: &self.reference,
        }
    }

    fn advance_reference_point(&self) {
        let ticks = self.ticks_now();
        self.reference.borrow_mut().advance_to(ticks);
    }
}

/// A lightweight, read-only [`ClockTrait`] view onto a [`ClockProviderFake`]
/// — see [`ClockProviderFake::get_clock`].
pub struct ClockFake<'a> {
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
        TICKS_PER_SECOND
    }
}

#[cfg(test)]
mod tests {
    use common::duration::Duration;

    use super::*;

    #[test]
    fn ticks_now_increments_by_one_each_call() {
        let provider = ClockProviderFake::new();
        let clock = provider.get_clock();
        assert_eq!(clock.ticks_now(), 1);
        assert_eq!(clock.ticks_now(), 2);
        assert_eq!(clock.ticks_now(), 3);
    }

    #[test]
    fn ticks_per_second_is_170mhz() {
        let provider = ClockProviderFake::new();
        let clock = provider.get_clock();
        assert_eq!(clock.ticks_per_second(), 170_000_000);
    }

    #[test]
    fn now_reflects_ticks_observed_since_the_last_reference_point() {
        let provider = ClockProviderFake::new();
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
        let provider = ClockProviderFake::new();
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
        let provider = ClockProviderFake::new();
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
        let provider = ClockProviderFake::new();
        let clock = provider.get_clock();
        let before = clock.now();

        // A quarter of a millisecond's worth of ticks at 170 MHz — small
        // enough to keep the busy loop's iteration count (and so this
        // test's run time) modest, but still several thousand ticks.
        let delay = Duration::new(1 << 20);
        clock.wait_for(delay);

        assert!(clock.now() >= before + delay);
    }
}
