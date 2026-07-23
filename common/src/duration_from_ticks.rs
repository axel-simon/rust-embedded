use crate::duration::Duration;
use crate::i64_divider::U64Divider;

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
#[derive(Debug, Clone, Copy)]
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
    value_at_quantum: Duration,
    ticks_past_quantum: u32,
    /// The hardware tick count last passed to [`Self::advance_to`], used to
    /// compute how many ticks have elapsed since then (wrapping, to follow
    /// a free-running hardware counter through overflow).
    last_advance_to_ticks: u32,
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
            value_at_quantum: Duration::new(0),
            ticks_past_quantum: 0,
            last_advance_to_ticks: 0,
        }
    }

    /// Returns `value_at_quantum + (ticks_past_quantum * quantum_duration /
    /// quantum_ticks)`, i.e. the exact time as of the last
    /// [`Self::advance_to`] or [`Self::advance_by`] call.
    pub fn now(&self) -> Duration {
        let fraction = match &self.divider {
            Some(divider) => {
                let numerator =
                    self.ticks_past_quantum as u64 * self.quantum_duration.ticks() as u64;
                divider.divide(numerator) as i64
            }
            None => 0,
        };
        self.value_at_quantum + Duration::new(fraction)
    }

    /// Advances the converter by `delta_ticks` hardware ticks, so that
    /// [`Self::now`] reports `delta_ticks` further ahead than before.
    pub fn advance_by(&mut self, delta_ticks: u32) {
        // Compute the number of quanta using a 32-bit division, then correct the
        // result if (ticks_past_quantum + delta_ticks) would have been larger
        // than 32-bit.
        let quanta_from_delta = delta_ticks / self.quantum_ticks;
        let remainder = delta_ticks % self.quantum_ticks;

        // `ticks_past_quantum + remainder` can be up to one quantum short of
        // two full quanta (both operands are themselves below one quantum),
        // so at most one extra quantum ever needs to be carried over.
        let sum = self.ticks_past_quantum + remainder;
        let (quanta_delta, ticks_past_quantum) = if sum >= self.quantum_ticks {
            (quanta_from_delta as i64 + 1, sum - self.quantum_ticks)
        } else {
            (quanta_from_delta as i64, sum)
        };
        self.ticks_past_quantum = ticks_past_quantum;
        self.value_at_quantum = self.value_at_quantum + self.quantum_duration * quanta_delta;
    }

    /// Updates the converter so that [`Self::now`] reports the time at
    /// `now_ticks` hardware ticks, given that `now_ticks` follows a
    /// free-running hardware counter (so it may have wrapped around since
    /// the last call).
    pub fn advance_to(&mut self, now_ticks: u32) {
        self.advance_by(now_ticks.wrapping_sub(self.last_advance_to_ticks));
        self.last_advance_to_ticks = now_ticks;
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
                clock.now().ticks(),
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
            assert_eq!(clock.now().ticks(), expected_ticks(now_ticks as u64, 1_000_000));
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
            let diff = expected_ticks(now_ticks as u64, frequency) - clock.now().ticks();
            assert!(diff.unsigned_abs() <= 2, "now_ticks={now_ticks} diff={diff}");
        }
    }

    #[test]
    fn advance_by_accumulates_across_multiple_calls() {
        let mut clock = DurationFromTicks::new(1_000_000);
        clock.advance_by(500_000);
        clock.advance_by(500_000);
        assert_eq!(clock.now().ticks(), expected_ticks(1_000_000, 1_000_000));
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
        assert_eq!(clock.now().ticks(), expected_ticks(total_ticks, 1_000_000));
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
}
