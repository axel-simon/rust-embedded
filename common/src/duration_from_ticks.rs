use core::sync::atomic::{AtomicBool, Ordering};

use crate::duration::Duration;
use crate::i64_divider::U64Divider;

/// A consistent snapshot of where [`DurationFromTicks`]'s reference point
/// was: an exact [`Duration`] (`value_at_quantum`) as of a known hardware
/// tick count (`last_advance_to_ticks`), plus the leftover sub-quantum tick
/// remainder (`ticks_past_quantum`) needed to reconstruct that instant.
/// [`DurationFromTicks`] keeps two of these (see its doc comment) so a
/// reader never observes one that's only half-written.
#[derive(Debug, Clone, Copy)]
struct ReferencePoint {
    value_at_quantum: Duration,
    ticks_past_quantum: u32,
    last_advance_to_ticks: u32,
}

impl ReferencePoint {
    const ZERO: ReferencePoint = ReferencePoint {
        value_at_quantum: Duration::new(0),
        ticks_past_quantum: 0,
        last_advance_to_ticks: 0,
    };
}

/// Converts a hardware timer's raw tick count into a [`Duration`], given
/// the timer's frequency in Hz.
///
/// Converting an arbitrary tick count exactly needs a 64-bit-by-32-bit
/// division (`ticks * 2^32 / frequency`), which Cortex-M4 has no hardware
/// support for. This avoids that on every call by splitting the tick count
/// into a whole number of *quanta* — the largest chunk of ticks that maps
/// to an exact fraction of a second — plus a remainder smaller than one
/// quantum. The quantum part accumulates via plain integer multiplication;
/// only the remainder ever needs a division, and that one division is done
/// with a precomputed [`U64Divider`] rather than a real division
/// instruction.
///
/// [`Self::advance_to`]/[`Self::advance_by`] (the only methods that need
/// `&mut self`) write a whole new [`ReferencePoint`] into whichever of
/// `states`'s two slots `current` does *not* currently point at, and only
/// then flip `current` — so [`Self::time_at`] (which only ever needs
/// `&self`) always reads a fully-formed reference point through `current`,
/// even if it's called while an `advance_to`/`advance_by` call elsewhere
/// (e.g. one that a timer interrupt preempted this call to reach) is
/// mid-flight: it either reads the old reference point (untouched) or the
/// new one (only visible once completely written), never a torn mix of the
/// two.
#[derive(Debug)]
pub struct DurationFromTicks {
    /// Ticks per quantum: `frequency` with all factors of two divided out.
    quantum_ticks: u32,
    /// The exact duration of one quantum, i.e. what `quantum_ticks` hardware
    /// ticks are worth in time.
    quantum_duration: Duration,
    /// Divides by `quantum_ticks`, used to convert `ticks_past_quantum`
    /// into an exact fraction of `quantum_duration`. `None` when
    /// `quantum_ticks == 1`, since the remainder is then always zero and
    /// `U64Divider` doesn't support a divisor of one.
    divider: Option<U64Divider>,
    states: [ReferencePoint; 2],
    /// `false` selects `states[0]`, `true` selects `states[1]` — whichever
    /// one is the fully-formed, currently-valid reference point.
    current: AtomicBool,
}

impl DurationFromTicks {
    /// Builds a converter for a timer running at `frequency` Hz, with
    /// `now()` initially reporting zero.
    ///
    /// Panics if `frequency` is zero or if it is odd. Requiring `frequency`
    /// even bounds `quantum_ticks` (its odd part) to at most `frequency / 2
    /// <= 2^31 - 1`, which is what keeps `ticks_past_quantum + remainder`
    /// in [`Self::advance_by`] from overflowing `u32` (each term is itself
    /// below `quantum_ticks`, so their sum stays under `2^32 - 2`).
    pub fn new(frequency: u32) -> Self {
        assert!(frequency != 0, "frequency must be non-zero");

        // gcd(frequency, 2^32) is the largest power of two dividing
        // `frequency`, i.e. 2^(number of trailing zero bits of frequency).
        // Dividing it out of both `frequency` and `2^32` gives the largest
        // tick count (`quantum_ticks`) whose corresponding duration
        // (`quantum_duration`) is an exact number of Duration ticks.
        let trailing_zeros = frequency.trailing_zeros();
        assert!(trailing_zeros > 0, "frequency must be even");
        let quantum_ticks = frequency >> trailing_zeros;
        let quantum_duration = Duration::new(1i64 << (32 - trailing_zeros));
        let divider = (quantum_ticks > 1).then(|| U64Divider::new(quantum_ticks));

        Self {
            quantum_ticks,
            quantum_duration,
            divider,
            states: [ReferencePoint::ZERO, ReferencePoint::ZERO],
            current: AtomicBool::new(false),
        }
    }

    /// The currently-valid reference point (see the struct doc comment).
    fn current(&self) -> ReferencePoint {
        // Acquire pairs with the Release store in `publish`, so that if
        // this observes the flip to the other slot, it also observes every
        // write `publish` made to that slot beforehand.
        self.states[self.current.load(Ordering::Acquire) as usize]
    }

    /// Writes `new` into the currently-inactive slot, then atomically makes
    /// it current.
    fn publish(&mut self, new: ReferencePoint) {
        let next = !self.current.load(Ordering::Relaxed);
        self.states[next as usize] = new;
        self.current.store(next, Ordering::Release);
    }

    /// Returns `value_at_quantum + (ticks_past_quantum * quantum_duration /
    /// quantum_ticks)`, i.e. the exact time as of the last
    /// [`Self::advance_to`] or [`Self::advance_by`] call.
    pub fn now(&self) -> Duration {
        self.value_at(self.current())
    }

    fn value_at(&self, reference: ReferencePoint) -> Duration {
        let fraction = match &self.divider {
            Some(divider) => {
                let numerator =
                    reference.ticks_past_quantum as u64 * self.quantum_duration.raw() as u64;
                divider.divide(numerator) as i64
            }
            None => 0,
        };
        reference.value_at_quantum + Duration::new(fraction)
    }

    /// Advances the converter by `delta_ticks` hardware ticks, so that
    /// [`Self::now`] reports `delta_ticks` further ahead than before.
    ///
    /// `delta_ticks` only ever moves forward (unlike [`Self::time_at`]'s
    /// delta, which can go either way), so this needs no signed/Euclidean
    /// division — just a 32-bit-by-32-bit one (`delta_ticks` and
    /// `quantum_ticks` are both `u32`), which Cortex-M4 does in hardware.
    /// Naively computing `reference.ticks_past_quantum + delta_ticks`
    /// first and dividing that would need 64 bits (the sum can exceed
    /// `u32::MAX`), so instead this divides `delta_ticks` alone (safe: two
    /// `u32`s, no overflow) and folds in `reference.ticks_past_quantum`
    /// afterward — it and the remainder from that division are each
    /// already `< quantum_ticks`, so their sum is `< 2 * quantum_ticks`,
    /// never more than one quantum away from being reduced, and a single
    /// comparison finishes the job instead of a second division.
    pub fn advance_by(&mut self, delta_ticks: u32) {
        let reference = self.current();
        let quantum_ticks = self.quantum_ticks;

        let quanta_from_delta = delta_ticks / quantum_ticks;
        let remainder_from_delta = delta_ticks % quantum_ticks;

        let combined = reference.ticks_past_quantum + remainder_from_delta;
        // `quanta_from_delta` can only reach `u32::MAX` when `quantum_ticks
        // == 1`, in which case `combined` is always 0 (both terms above are
        // always 0 too) — so `quanta_from_delta + extra_quanta` below never
        // overflows.
        let (extra_quanta, ticks_past_quantum) = if combined >= quantum_ticks {
            (1, combined - quantum_ticks)
        } else {
            (0, combined)
        };

        let new = ReferencePoint {
            value_at_quantum: reference.value_at_quantum
                + self.quantum_duration * (quanta_from_delta + extra_quanta) as i64,
            ticks_past_quantum,
            last_advance_to_ticks: reference.last_advance_to_ticks.wrapping_add(delta_ticks),
        };
        self.publish(new);
    }

    /// Updates the converter so that [`Self::now`] reports the time at
    /// `now_ticks` hardware ticks, given that `now_ticks` follows a
    /// free-running hardware counter (so it may have wrapped around since
    /// the last call).
    pub fn advance_to(&mut self, now_ticks: u32) {
        self.advance_by(now_ticks.wrapping_sub(self.current().last_advance_to_ticks));
    }

    /// Interprets `ticks` as a hardware tick count sampled close in time to
    /// [`Self::now`] and returns the corresponding [`Duration`]. "Close in
    /// time" means before or after, roughly +/- (2^31/`frequency`)-seconds
    /// from [`Self::now`]. For a 170MHz MCU, the range is +/- 12.6 seconds.
    ///
    /// This method is thread-safe and can be called concurrently with
    /// [`Self::advance_to`] or [`Self::advance_by`].
    pub fn time_at(&self, ticks: u32) -> Duration {
        let reference = self.current();
        let delta_from_now = ticks.wrapping_sub(reference.last_advance_to_ticks) as i32;
        let quantum_ticks = self.quantum_ticks as i32;

        let quanta_delta = delta_from_now.div_euclid(quantum_ticks);
        let ticks_past_quantum = delta_from_now.rem_euclid(quantum_ticks) as u32;

        self.value_at(ReferencePoint {
            value_at_quantum: reference.value_at_quantum
                + self.quantum_duration * quanta_delta as i64,
            ticks_past_quantum: reference.ticks_past_quantum + ticks_past_quantum,
            last_advance_to_ticks: ticks,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::DurationFromTicks;
    use rstest::rstest;

    /// Exact reference conversion via 128-bit arithmetic, used as the test
    /// oracle instead of trusting `DurationFromTicks`'s own math. Takes the
    /// *total* elapsed ticks (not necessarily a `u32`), so it can check
    /// totals that have wrapped the hardware counter one or more times.
    fn expected_ticks(total_ticks: u64, frequency: u32) -> i64 {
        (((total_ticks as u128) << 32) / (frequency as u128)) as i64
    }

    #[rstest]
    #[case(2)] // smallest even: pure power of two, quantum is a single tick
    #[case(6)] // 2 * 3: small odd quantum
    #[case(1 << 20)] // pure power of two: quantum is a single tick
    #[case(1_000_000)] // mixed: 2^6 * 15625
    #[case(999_998)] // 2 * 499_999: large-ish odd quantum
    #[case(4_294_967_294)] // largest even u32: quantum = 2^31 - 1, the max possible
    fn now_matches_exact_conversion(#[case] frequency: u32) {
        let quantum_ticks = frequency >> frequency.trailing_zeros();

        for now_ticks in [
            0,
            1,
            2,
            quantum_ticks.saturating_sub(1),
            quantum_ticks,
            quantum_ticks.saturating_add(1),
            quantum_ticks.saturating_mul(3),
        ] {
            // A fresh clock per sample point, since `advance_to` now tracks
            // history (`last_advance_to_ticks`) and these sample points
            // aren't necessarily monotonically increasing.
            let mut clock = DurationFromTicks::new(frequency);
            clock.advance_to(now_ticks);
            assert_eq!(
                clock.now().raw(),
                expected_ticks(now_ticks as u64, frequency),
                "frequency={frequency} now_ticks={now_ticks}"
            );
        }
    }

    #[test]
    fn large_tick_counts_at_a_high_frequency() {
        let mut clock = DurationFromTicks::new(1_000_000);
        for now_ticks in [0, 15_624, 15_625, 15_626, 1_000_000, u32::MAX] {
            clock.advance_to(now_ticks);
            assert_eq!(
                clock.now().raw(),
                expected_ticks(now_ticks as u64, 1_000_000)
            );
        }
    }

    #[test]
    fn now_error_bounded_by_two_ticks_at_170mhz() {
        // A real STM32 timer frequency (170 MHz). Its odd part
        // (`quantum_ticks`) is 1_328_125 — large enough that a rounded-down
        // per-tick approximation would drift by hundreds of microseconds
        // once `ticks_past_quantum` got close to a full quantum. The exact,
        // divider-based computation should be within a couple of hardware
        // ticks' worth of Duration error even right at that boundary.
        let frequency: u32 = 170_000_000;
        let quantum_ticks = frequency >> frequency.trailing_zeros();

        for now_ticks in [
            0,
            1,
            quantum_ticks / 2,
            quantum_ticks - 1,
            quantum_ticks,
            quantum_ticks + 1,
            quantum_ticks.saturating_mul(3),
            u32::MAX,
        ] {
            let mut clock = DurationFromTicks::new(frequency);
            clock.advance_to(now_ticks);
            let diff = expected_ticks(now_ticks as u64, frequency) - clock.now().raw();
            assert!(
                diff.unsigned_abs() <= 2,
                "now_ticks={now_ticks} diff={diff}"
            );
        }
    }

    #[test]
    fn advance_by_accumulates_across_multiple_calls() {
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_by(500_000);
        clock.advance_by(500_000);
        assert_eq!(clock.now().raw(), expected_ticks(1_000_000, 1_000_000));
    }

    #[test]
    fn advance_by_carries_into_the_next_quantum_when_the_remainder_overflows() {
        // frequency=1_000_000 -> quantum_ticks=15_625. First call leaves a
        // remainder of 4_375; the second's own remainder (15_000) pushes
        // their sum (19_375) past quantum_ticks, so this only passes if
        // `advance_by`'s carry-correction (rather than just its coarse
        // `delta_ticks / quantum_ticks` division) is applied.
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_by(20_000);
        clock.advance_by(15_000);
        assert_eq!(clock.now().raw(), expected_ticks(35_000, 1_000_000));
    }

    #[test]
    fn advance_by_handles_the_largest_possible_combined_remainder() {
        // The largest possible `quantum_ticks` (2^31 - 1, from the largest
        // even frequency). Two calls, each leaving a remainder one short of
        // `quantum_ticks`, make `advance_by`'s internal `combined` value
        // land at its theoretical maximum (2 * quantum_ticks - 2, just 3
        // below `u32::MAX`) — the tightest possible check that combining
        // the two remainders doesn't overflow `u32` before being reduced.
        let frequency: u32 = 4_294_967_294;
        let quantum_ticks = frequency >> frequency.trailing_zeros();
        assert_eq!(quantum_ticks, (1u32 << 31) - 1);

        let delta = quantum_ticks - 1;
        let mut clock = DurationFromTicks::new(frequency);
        clock.advance_by(delta);
        clock.advance_by(delta);
        assert_eq!(
            clock.now().raw(),
            expected_ticks(delta as u64 * 2, frequency)
        );
    }

    #[test]
    fn advance_to_matches_equivalent_advance_by_from_a_fresh_clock() {
        // `last_advance_to_ticks` starts at zero, so the first `advance_to`
        // call is equivalent to `advance_by` of the same amount.
        let mut via_advance_to = DurationFromTicks::new(1_000_000);
        via_advance_to.advance_to(1_234_567);

        let mut via_advance_by = DurationFromTicks::new(1_000_000);
        via_advance_by.advance_by(1_234_567);

        assert_eq!(via_advance_to.now(), via_advance_by.now());
    }

    #[test]
    fn advance_to_follows_the_hardware_counter_through_wraparound() {
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_to(u32::MAX - 10);
        clock.advance_to(9); // wrapped past u32::MAX; 20 ticks further on

        let total_ticks = (u32::MAX as u64 - 10) + 20;
        assert_eq!(clock.now().raw(), expected_ticks(total_ticks, 1_000_000));
    }

    #[test]
    #[should_panic]
    fn new_panics_on_zero_frequency() {
        DurationFromTicks::new(0);
    }

    #[test]
    #[should_panic]
    fn new_panics_on_odd_frequency() {
        DurationFromTicks::new(999_999);
    }

    #[rstest]
    #[case(2)] // smallest even: pure power of two, quantum is a single tick
    #[case(6)] // 2 * 3: small odd quantum
    #[case(1 << 20)] // pure power of two: quantum is a single tick
    #[case(1_000_000)] // mixed: 2^6 * 15625
    #[case(999_998)] // 2 * 499_999: large-ish odd quantum
    #[case(4_294_967_294)] // largest even u32: quantum = 2^31 - 1, the max possible
    fn time_at_interprets_ticks_relative_to_the_last_advance_to_call(#[case] frequency: u32) {
        let quantum_ticks = frequency >> frequency.trailing_zeros();
        // Offset by one past a quantum boundary so the reference point
        // itself usually has a nonzero `ticks_past_quantum` (rather than
        // conveniently landing exactly on one) — still comfortably within
        // `u32` after any delta below is added, so nothing here wraps
        // around (that's covered separately, e.g. by
        // `time_at_picks_the_smaller_magnitude_delta_across_a_wraparound`).
        let reference_ticks = quantum_ticks + 1;

        for delta in [
            -(quantum_ticks as i64 + 1),
            -1,
            0,
            1,
            quantum_ticks as i64 - 1,
            quantum_ticks as i64,
        ] {
            let mut clock = DurationFromTicks::new(frequency);
            clock.advance_to(reference_ticks);

            let total_ticks = reference_ticks as i64 + delta;
            let queried_ticks = total_ticks as u32;

            assert_eq!(
                clock.time_at(queried_ticks).raw(),
                expected_ticks(total_ticks as u64, frequency),
                "frequency={frequency} delta={delta}"
            );
        }
    }

    #[test]
    fn time_at_the_reference_point_matches_now() {
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_to(1_234_567);
        assert_eq!(clock.time_at(1_234_567), clock.now());
    }

    #[test]
    fn time_at_shortly_after_the_reference_point_matches_advance_by() {
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_to(1_000_000);
        let after = clock.time_at(1_000_100);

        let mut reference = DurationFromTicks::new(1_000_000);
        reference.advance_to(1_000_100);
        assert_eq!(after, reference.now());
    }

    #[test]
    fn time_at_shortly_before_the_reference_point_goes_backward() {
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_to(1_000_000);
        let before = clock.time_at(999_900);

        let mut reference = DurationFromTicks::new(1_000_000);
        reference.advance_to(999_900);
        assert_eq!(before, reference.now());
    }

    #[test]
    fn time_at_picks_the_smaller_magnitude_delta_across_a_wraparound() {
        // The reference point sits 50 ticks before the hardware counter's
        // `u32::MAX` wraparound; querying a small tick count just past the
        // wrap should still be interpreted as "shortly after" the
        // reference (100 ticks forward here), not billions of ticks
        // backward.
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_to(u32::MAX - 50);
        let after_wrap = clock.time_at(49); // 100 ticks after `u32::MAX - 50`, wrapped

        let mut reference = DurationFromTicks::new(1_000_000);
        reference.advance_to(u32::MAX - 50);
        reference.advance_by(100); // same 100-tick forward step, via the wrap-following advance_by

        assert_eq!(after_wrap, reference.now());
    }

    #[test]
    fn time_at_across_a_wraparound_with_a_nonzero_quantum_remainder() {
        // Same shape as the test above, but the reference point's own
        // `ticks_past_quantum` is nonzero here (frequency=6 ->
        // quantum_ticks=3, and `u32::MAX - 50` isn't a multiple of 3) — this
        // is what actually exercises `time_at`'s unreduced
        // `reference.ticks_past_quantum + ticks_past_quantum` sum (which can
        // land at or past `quantum_ticks`) rather than a case where the
        // reference point's own remainder happens to be zero.
        let mut clock = DurationFromTicks::new(6);
        clock.advance_to(u32::MAX - 50);
        let after_wrap = clock.time_at(49); // 100 ticks after `u32::MAX - 50`, wrapped

        let mut reference = DurationFromTicks::new(6);
        reference.advance_to(u32::MAX - 50);
        reference.advance_by(100);

        assert_eq!(after_wrap, reference.now());
    }

    #[test]
    fn time_at_carries_into_the_next_quantum_when_the_remainder_overflows() {
        // frequency=1_000_000 -> quantum_ticks=15_625. `advance_by(20_000)`
        // leaves a reference remainder of 4_375; querying 14_000 ticks
        // further on adds a remainder of 14_000, whose sum (18_375)
        // exceeds quantum_ticks — only correct if `value_at`'s linearity
        // (not an explicit carry) handles the unreduced sum right.
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_by(20_000);
        let result = clock.time_at(20_000 + 14_000);
        assert_eq!(result.raw(), expected_ticks(34_000, 1_000_000));
    }

    #[test]
    fn time_at_handles_the_largest_possible_combined_remainder() {
        // Mirrors `advance_by_handles_the_largest_possible_combined_remainder`:
        // the largest possible `quantum_ticks` (2^31 - 1), with both the
        // reference's own remainder and the queried delta's remainder one
        // short of `quantum_ticks`, pushing their sum to its theoretical
        // maximum (2 * quantum_ticks - 2, 3 below `u32::MAX`).
        let frequency: u32 = 4_294_967_294;
        let quantum_ticks = frequency >> frequency.trailing_zeros();
        let delta = quantum_ticks - 1;

        let mut clock = DurationFromTicks::new(frequency);
        clock.advance_by(delta);
        let result = clock.time_at(delta.wrapping_add(delta));
        assert_eq!(result.raw(), expected_ticks(delta as u64 * 2, frequency));
    }

    #[test]
    fn time_at_does_not_require_mutable_access() {
        // Compiles only if `time_at` takes `&self`.
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_to(1_000_000);
        let clock = clock;
        let _ = clock.time_at(1_000_050);
        let _ = clock.time_at(1_000_050); // callable repeatedly through &self
    }
}
