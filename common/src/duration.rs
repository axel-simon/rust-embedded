use core::ops::{Add, Mul, Sub};

use crate::i64_divider::{I64Divider, U64Divider};

/// Divides a raw tick count by a divider, letting [`Duration::divide`]
/// accept either an unsigned or a signed divider.
pub trait TickDivider {
    fn divide_ticks(&self, ticks: i64) -> i64;
}

impl TickDivider for U64Divider {
    /// Divides non-negative ticks via the cheaper unsigned path.
    ///
    /// Panics (debug only) if `ticks` is negative, since casting a negative
    /// `i64` to `u64` would silently produce a huge, wrong value.
    fn divide_ticks(&self, ticks: i64) -> i64 {
        debug_assert!(ticks >= 0, "U64Divider requires a non-negative Duration");
        self.divide(ticks as u64) as i64
    }
}

impl TickDivider for I64Divider {
    fn divide_ticks(&self, ticks: i64) -> i64 {
        self.divide(ticks)
    }
}

/// A duration represented as a fixed-point number of seconds.
///
/// The value is stored as `seconds * 2^32` in a signed 64-bit integer, so
/// `2^32` represents exactly one second. The upper 32 bits hold the whole
/// seconds and the lower 32 bits hold the fractional part.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Duration(i64);

impl Duration {
    /// Builds a `Duration` from a raw tick count (1 tick = 1 / 2^32 second).
    pub const fn new(ticks: i64) -> Self {
        Duration(ticks)
    }

    /// Returns the raw tick count (1 tick = 1 / 2^32 second).
    pub const fn ticks(self) -> i64 {
        self.0
    }

    /// Whole seconds, rounded down (the upper 32 bits of the tick count).
    pub const fn seconds(self) -> i64 {
        self.0 >> 32
    }

    /// Fractional part of a second, as a fraction of 2^32 (the lower 32
    /// bits of the tick count).
    pub const fn fraction(self) -> u32 {
        self.0 as u32
    }

    /// Fractional part of a second, in milliseconds.
    pub fn millis(self) -> u32 {
        ((self.fraction() as u64 * 1_000) >> 32) as u32
    }

    /// Fractional part of a second, in microseconds.
    pub fn micros(self) -> u32 {
        ((self.fraction() as u64 * 1_000_000) >> 32) as u32
    }

    /// Fractional part of a second, in nanoseconds.
    pub fn nanos(self) -> u32 {
        ((self.fraction() as u64 * 1_000_000_000) >> 32) as u32
    }

    /// Divides this duration by `divider`, which may be a [`U64Divider`]
    /// (this duration must be non-negative) or an [`I64Divider`] (works for
    /// negative durations too).
    pub fn divide<D: TickDivider>(self, divider: &D) -> Duration {
        Duration(divider.divide_ticks(self.0))
    }
}

impl Add for Duration {
    type Output = Duration;

    fn add(self, rhs: Duration) -> Duration {
        Duration(self.0 + rhs.0)
    }
}

impl Sub for Duration {
    type Output = Duration;

    fn sub(self, rhs: Duration) -> Duration {
        Duration(self.0 - rhs.0)
    }
}

impl Mul<i64> for Duration {
    type Output = Duration;

    fn mul(self, rhs: i64) -> Duration {
        Duration(self.0 * rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::Duration;
    use crate::i64_divider::{I64Divider, U64Divider};

    const ONE_SECOND: i64 = 1 << 32;

    #[test]
    fn seconds_and_fraction_for_positive_values() {
        let d = Duration::new(ONE_SECOND + (1 << 31));
        assert_eq!(d.seconds(), 1);
        assert_eq!(d.fraction(), 1 << 31);
    }

    #[test]
    fn seconds_and_fraction_for_zero() {
        let d = Duration::new(0);
        assert_eq!(d.seconds(), 0);
        assert_eq!(d.fraction(), 0);
    }

    #[test]
    fn seconds_rounds_down_for_negative_values() {
        // -1 tick is just below zero: -1 second plus almost a full second
        // of fraction, i.e. rounding towards negative infinity.
        let d = Duration::new(-1);
        assert_eq!(d.seconds(), -1);
        assert_eq!(d.fraction(), u32::MAX);

        let d = Duration::new(-ONE_SECOND);
        assert_eq!(d.seconds(), -1);
        assert_eq!(d.fraction(), 0);
    }

    #[test]
    fn addition() {
        let a = Duration::new(ONE_SECOND);
        let b = Duration::new(1 << 31);
        assert_eq!(a + b, Duration::new(ONE_SECOND + (1 << 31)));
    }

    #[test]
    fn subtraction() {
        let a = Duration::new(ONE_SECOND);
        let b = Duration::new(1 << 31);
        assert_eq!(a - b, Duration::new(ONE_SECOND - (1 << 31)));
    }

    #[test]
    fn multiplication_by_constant() {
        let a = Duration::new(ONE_SECOND + (1 << 31));
        assert_eq!(a * 3, Duration::new(3 * (ONE_SECOND + (1 << 31))));
    }

    #[test]
    fn millis_micros_nanos_for_half_second() {
        let d = Duration::new(1 << 31);
        assert_eq!(d.millis(), 500);
        assert_eq!(d.micros(), 500_000);
        assert_eq!(d.nanos(), 500_000_000);
    }

    #[test]
    fn millis_micros_nanos_for_zero() {
        let d = Duration::new(0);
        assert_eq!(d.millis(), 0);
        assert_eq!(d.micros(), 0);
        assert_eq!(d.nanos(), 0);
    }

    #[test]
    fn millis_micros_nanos_round_down_just_under_a_second() {
        let d = Duration::new(-1);
        assert_eq!(d.millis(), 999);
        assert_eq!(d.micros(), 999_999);
        assert_eq!(d.nanos(), 999_999_999);
    }

    #[test]
    fn divide_by_u64_divider() {
        let divider = U64Divider::new(3);
        let d = Duration::new(3 * ONE_SECOND + 1);
        assert_eq!(d.divide(&divider), Duration::new((3 * ONE_SECOND + 1) / 3));
    }

    #[test]
    fn divide_by_i64_divider_positive() {
        let divider = I64Divider::new(3u32);
        let d = Duration::new(3 * ONE_SECOND + 1);
        assert_eq!(d.divide(&divider), Duration::new((3 * ONE_SECOND + 1) / 3));
    }

    #[test]
    fn divide_by_i64_divider_negative() {
        let divider = I64Divider::new(3u32);
        let d = Duration::new(-(3 * ONE_SECOND + 1));
        assert_eq!(d.divide(&divider), Duration::new(-(3 * ONE_SECOND + 1) / 3));
    }
}
