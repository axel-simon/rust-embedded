use core::ops::{Add, Mul, Sub};

use crate::i64_divider::I64Divider;

/// Precomputed dividers for
/// [`Duration::from_millis`]/[`Duration::from_micros`]/
/// [`Duration::from_nanos`] — each `const`, so the reciprocal they're built
/// from (see [`I64Divider::new`]) is computed once at compile time rather
/// than on every call.
const MILLIS_DIVIDER: I64Divider = I64Divider::new(1_000);
const MICROS_DIVIDER: I64Divider = I64Divider::new(1_000_000);
const NANOS_DIVIDER: I64Divider = I64Divider::new(1_000_000_000);

/// A duration represented as a fixed-point number of seconds.
///
/// The value is stored as `seconds * 2^32` in a signed 64-bit integer, so
/// `2^32` represents exactly one second. The upper 32 bits hold the whole
/// seconds and the lower 32 bits hold the fractional part.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Duration(i64);

impl Duration {
    /// Builds a `Duration` from a raw value (1 raw unit = 1 / 2^32 second).
    pub const fn new(raw: i64) -> Self {
        Duration(raw)
    }

    /// Builds a `Duration` of `seconds` whole seconds.
    pub const fn from_seconds(seconds: i32) -> Self {
        Duration((seconds as i64) << 32)
    }

    /// Builds a `Duration` of `millis` thousandths of a second, e.g.
    /// `from_millis(2000)` is a two-second `Duration`.
    pub const fn from_millis(millis: i32) -> Self {
        Duration(MILLIS_DIVIDER.divide((millis as i64) << 32))
    }

    /// Builds a `Duration` of `micros` millionths of a second.
    pub const fn from_micros(micros: i32) -> Self {
        Duration(MICROS_DIVIDER.divide((micros as i64) << 32))
    }

    /// Builds a `Duration` of `nanos` billionths of a second.
    pub const fn from_nanos(nanos: i32) -> Self {
        Duration(NANOS_DIVIDER.divide((nanos as i64) << 32))
    }

    /// Returns the raw value (1 raw unit = 1 / 2^32 second).
    pub const fn raw(self) -> i64 {
        self.0
    }

    /// Whole seconds, rounded down (the upper 32 bits of the raw value).
    pub const fn seconds(self) -> i32 {
        (self.0 >> 32) as i32
    }

    /// Fractional part of a second, as a fraction of 2^32 (the lower 32
    /// bits of the raw value).
    pub const fn fraction(self) -> u32 {
        self.0 as u32
    }

    /// Fractional part of a second, in milliseconds, that is, the returned
    /// value is always in the range 0..=999.
    pub fn fraction_as_millis(self) -> u32 {
        ((self.fraction() as u64 * 1_000) >> 32) as u32
    }

    /// Fractional part of a second, in microseconds, that is, the returned
    /// value is always in the range 0..=999_999.
    pub fn fraction_as_micros(self) -> u32 {
        ((self.fraction() as u64 * 1_000_000) >> 32) as u32
    }

    /// Fractional part of a second, in nanoseconds, that is, the returned
    /// value is always in the range 0..=999_999_999.
    pub fn fraction_as_nanos(self) -> u32 {
        ((self.fraction() as u64 * 1_000_000_000) >> 32) as u32
    }

    /// Divides this duration by `divider`. To divide by a
    /// [`crate::i64_divider::U64Divider`], build an [`I64Divider`] from it
    /// first (`I64Divider::new` accepts one directly).
    pub fn divide(self, divider: &I64Divider) -> Duration {
        Duration(divider.divide(self.0))
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
        // -1 raw unit is just below zero: -1 second plus almost a full
        // second of fraction, i.e. rounding towards negative infinity.
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
    fn fraction_as_millis_micros_nanos_for_half_second() {
        let d = Duration::new(1 << 31);
        assert_eq!(d.fraction_as_millis(), 500);
        assert_eq!(d.fraction_as_micros(), 500_000);
        assert_eq!(d.fraction_as_nanos(), 500_000_000);
    }

    #[test]
    fn fraction_as_millis_micros_nanos_for_zero() {
        let d = Duration::new(0);
        assert_eq!(d.fraction_as_millis(), 0);
        assert_eq!(d.fraction_as_micros(), 0);
        assert_eq!(d.fraction_as_nanos(), 0);
    }

    #[test]
    fn fraction_as_millis_micros_nanos_round_down_just_under_a_second() {
        let d = Duration::new(-1);
        assert_eq!(d.fraction_as_millis(), 999);
        assert_eq!(d.fraction_as_micros(), 999_999);
        assert_eq!(d.fraction_as_nanos(), 999_999_999);
    }

    #[test]
    fn divide_by_i64_divider_built_from_a_u64_divider() {
        let divider = I64Divider::from(U64Divider::new(3));
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

    #[test]
    fn raw_round_trips_through_new() {
        assert_eq!(Duration::new(1_234_567_890).raw(), 1_234_567_890);
        assert_eq!(Duration::new(-1).raw(), -1);
    }

    #[test]
    fn from_seconds_matches_whole_seconds() {
        assert_eq!(Duration::from_seconds(5).seconds(), 5);
        assert_eq!(Duration::from_seconds(5).fraction(), 0);
        assert_eq!(Duration::from_seconds(-5).seconds(), -5);
        assert_eq!(Duration::from_seconds(0), Duration::new(0));
    }

    #[test]
    fn from_millis_2000_is_two_seconds() {
        assert_eq!(Duration::from_millis(2000), Duration::from_seconds(2));
    }

    #[test]
    fn from_millis_micros_nanos_for_half_second() {
        let half_second = Duration::new(1 << 31);
        assert_eq!(Duration::from_millis(500), half_second);
        assert_eq!(Duration::from_micros(500_000), half_second);
        assert_eq!(Duration::from_nanos(500_000_000), half_second);
    }

    #[test]
    fn from_millis_splits_into_seconds_and_a_sub_second_remainder() {
        for ms in [0, 1, 500, 999, 1_500, 12_345] {
            let d = Duration::from_millis(ms);
            assert_eq!(d.seconds(), ms / 1_000);
            // `from_millis` and `.fraction_as_millis()` are each
            // independently a flooring conversion (2^32 isn't a multiple
            // of 1000), so composing them can undershoot the sub-second
            // remainder by up to 1 ms, but never more, and never overshoot.
            let remainder = (ms % 1_000) as u32;
            let got = d.fraction_as_millis();
            assert!(
                got == remainder || got + 1 == remainder,
                "ms={ms} got={got}"
            );
        }
    }
}
