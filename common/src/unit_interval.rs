use core::ops::{Add, Mul, Sub};

/// A value in `[0, 1)`, stored as `raw / 2^32`.
///
/// The whole `u32` range maps onto the interval, so `+`/`-` wrap around at
/// the interval's bounds exactly the way `u32` wrapping arithmetic wraps
/// around `0`/`u32::MAX` — this type is meant for a cyclic quantity (e.g. a
/// turn or phase fraction), not a value that should saturate at 0 or 1. Use
/// [`UnitInterval::saturating_add`]/[`UnitInterval::saturating_sub`] where
/// clamping instead of wrapping is what's wanted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct UnitInterval(u32);

impl UnitInterval {
    /// Builds a `UnitInterval` from a raw value (1 raw unit = 1 / 2^32).
    pub const fn new(raw: u32) -> Self {
        UnitInterval(raw)
    }

    /// Returns the raw value (1 raw unit = 1 / 2^32).
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Saturating addition: clamps at the top of the representable range
    /// (just below `1`) instead of wrapping.
    pub const fn saturating_add(self, rhs: Self) -> Self {
        UnitInterval(self.0.saturating_add(rhs.0))
    }

    /// Saturating subtraction: clamps at `0` instead of wrapping.
    pub const fn saturating_sub(self, rhs: Self) -> Self {
        UnitInterval(self.0.saturating_sub(rhs.0))
    }

    /// Maps this `[0, 1)` value onto `[-1, 1)`, doubling its span: `0` maps
    /// to `-1`, and a value approaching `1` approaches `1`. The inverse of
    /// [`SymmetricUnitInterval::shrink`].
    pub const fn expand(self) -> SymmetricUnitInterval {
        SymmetricUnitInterval(self.0.wrapping_sub(0x8000_0000) as i32)
    }
}

impl Add for UnitInterval {
    type Output = UnitInterval;

    /// Wraps around at the top of the interval, e.g. `0.75 + 0.5` wraps to
    /// `0.25`.
    fn add(self, rhs: UnitInterval) -> UnitInterval {
        UnitInterval(self.0.wrapping_add(rhs.0))
    }
}

impl Sub for UnitInterval {
    type Output = UnitInterval;

    /// Wraps around at the bottom of the interval, e.g. `0.25 - 0.5` wraps
    /// to `0.75`.
    fn sub(self, rhs: UnitInterval) -> UnitInterval {
        UnitInterval(self.0.wrapping_sub(rhs.0))
    }
}

impl Mul<u32> for UnitInterval {
    type Output = UnitInterval;

    /// Scales the interval value by an integer factor, wrapping around on
    /// overflow (e.g. useful for computing a harmonic of a phase).
    fn mul(self, rhs: u32) -> UnitInterval {
        UnitInterval(self.0.wrapping_mul(rhs))
    }
}

impl From<UnitInterval> for SymmetricUnitInterval {
    /// Preserves the interval value: `[0, 1)` is a subset of `[-1, 1)`, so
    /// this always exists, modulo the one bit of precision lost narrowing a
    /// 32-bit fraction to a 31-bit one.
    fn from(u: UnitInterval) -> Self {
        SymmetricUnitInterval((u.0 >> 1) as i32)
    }
}

impl From<f32> for UnitInterval {
    /// Saturating: values `<= 0` (or NaN) become `0`, values `>= 1` become
    /// the largest representable value, just below `1`. Relies on the
    /// float-to-int `as` cast already being saturating (NaN -> 0, out of
    /// range -> the nearest bound) since Rust 1.45.
    fn from(value: f32) -> Self {
        UnitInterval((value as f64 * 4_294_967_296.0) as u32)
    }
}

impl From<UnitInterval> for f32 {
    fn from(u: UnitInterval) -> f32 {
        (u.0 as f64 / 4_294_967_296.0) as f32
    }
}

/// A value in `[-1, 1)`, stored as `raw / 2^31`.
///
/// Like [`UnitInterval`], `+`/`-` wrap around at the interval's bounds by
/// design — this type represents a cyclic quantity of span 2 rather than a
/// value that should saturate at `-1` or `1`. Use
/// [`SymmetricUnitInterval::saturating_add`]/
/// [`SymmetricUnitInterval::saturating_sub`] where clamping is what's
/// wanted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct SymmetricUnitInterval(i32);

impl SymmetricUnitInterval {
    /// Builds a `SymmetricUnitInterval` from a raw value (1 raw unit = 1 /
    /// 2^31).
    pub const fn new(raw: i32) -> Self {
        SymmetricUnitInterval(raw)
    }

    /// Returns the raw value (1 raw unit = 1 / 2^31).
    pub const fn raw(self) -> i32 {
        self.0
    }

    /// Saturating addition: clamps at the top of the representable range
    /// (just below `1`) instead of wrapping.
    pub const fn saturating_add(self, rhs: Self) -> Self {
        SymmetricUnitInterval(self.0.saturating_add(rhs.0))
    }

    /// Saturating subtraction: clamps at `-1` instead of wrapping.
    pub const fn saturating_sub(self, rhs: Self) -> Self {
        SymmetricUnitInterval(self.0.saturating_sub(rhs.0))
    }

    /// Maps this `[-1, 1)` value onto `[0, 1)`, halving its span: `-1` maps
    /// to `0`, and a value approaching `1` approaches `1`. The inverse of
    /// [`UnitInterval::expand`].
    pub const fn shrink(self) -> UnitInterval {
        UnitInterval((self.0 as u32).wrapping_add(0x8000_0000))
    }
}

impl Add for SymmetricUnitInterval {
    type Output = SymmetricUnitInterval;

    /// Wraps around at the top of the interval, e.g. `0.75 + 0.5` wraps to
    /// `-0.75`.
    fn add(self, rhs: SymmetricUnitInterval) -> SymmetricUnitInterval {
        SymmetricUnitInterval(self.0.wrapping_add(rhs.0))
    }
}

impl Sub for SymmetricUnitInterval {
    type Output = SymmetricUnitInterval;

    /// Wraps around at the bottom of the interval, e.g. `-0.75 - 0.5` wraps
    /// to `0.75`.
    fn sub(self, rhs: SymmetricUnitInterval) -> SymmetricUnitInterval {
        SymmetricUnitInterval(self.0.wrapping_sub(rhs.0))
    }
}

impl Mul<u32> for SymmetricUnitInterval {
    type Output = SymmetricUnitInterval;

    /// Scales the interval value by an integer factor, wrapping around on
    /// overflow (e.g. useful for computing a harmonic of a phase).
    fn mul(self, rhs: u32) -> SymmetricUnitInterval {
        SymmetricUnitInterval(self.0.wrapping_mul(rhs as i32))
    }
}

impl From<SymmetricUnitInterval> for UnitInterval {
    /// Preserves the interval value when it's already representable (i.e.
    /// already in `[0, 1)`); negative values wrap by the interval's cyclic
    /// span of `1`, matching [`UnitInterval`]'s own wraparound semantics
    /// (e.g. `-0.25` becomes `0.75`).
    fn from(s: SymmetricUnitInterval) -> Self {
        UnitInterval((s.0 as u32) << 1)
    }
}

impl From<f32> for SymmetricUnitInterval {
    /// Saturating: values `<= -1` become `-1`, values `>= 1` become the
    /// largest representable value (just below `1`), and NaN becomes `0`.
    /// Relies on the float-to-int `as` cast already being saturating
    /// (NaN -> 0, out of range -> the nearest bound) since Rust 1.45.
    fn from(value: f32) -> Self {
        SymmetricUnitInterval((value as f64 * 2_147_483_648.0) as i32)
    }
}

impl From<SymmetricUnitInterval> for f32 {
    fn from(s: SymmetricUnitInterval) -> f32 {
        (s.0 as f64 / 2_147_483_648.0) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::{SymmetricUnitInterval, UnitInterval};
    use rstest::rstest;

    #[test]
    fn raw_round_trips_through_new() {
        assert_eq!(UnitInterval::new(0x1234_5678).raw(), 0x1234_5678);
        assert_eq!(SymmetricUnitInterval::new(-1).raw(), -1);
    }

    #[test]
    fn unit_interval_addition_wraps_at_the_top() {
        let a = UnitInterval::new(u32::MAX);
        let b = UnitInterval::new(2);
        assert_eq!(a + b, UnitInterval::new(1));
    }

    #[test]
    fn unit_interval_subtraction_wraps_at_the_bottom() {
        let a = UnitInterval::new(0);
        let b = UnitInterval::new(1);
        assert_eq!(a - b, UnitInterval::new(u32::MAX));
    }

    #[test]
    fn unit_interval_saturating_add_clamps_at_max() {
        let a = UnitInterval::new(u32::MAX - 1);
        let b = UnitInterval::new(10);
        assert_eq!(a.saturating_add(b), UnitInterval::new(u32::MAX));
    }

    #[test]
    fn unit_interval_saturating_sub_clamps_at_zero() {
        let a = UnitInterval::new(1);
        let b = UnitInterval::new(10);
        assert_eq!(a.saturating_sub(b), UnitInterval::new(0));
    }

    #[test]
    fn unit_interval_mul_scales_and_wraps() {
        let a = UnitInterval::new(0x6000_0000); // 0.375
        assert_eq!(a * 3, UnitInterval::new(0x6000_0000u32.wrapping_mul(3)));
    }

    #[rstest]
    #[case(0, i32::MIN)] // 0 maps to -1
    #[case(0x8000_0000, 0)] // 0.5 maps to 0
    #[case(u32::MAX, i32::MAX)] // just below 1 maps to just below 1
    fn unit_interval_expand_maps_0_to_negative_1_and_1_to_1(
        #[case] raw: u32,
        #[case] expected: i32,
    ) {
        assert_eq!(
            UnitInterval::new(raw).expand(),
            SymmetricUnitInterval::new(expected)
        );
    }

    #[rstest]
    #[case(0)]
    #[case(1)]
    #[case(0x8000_0000)]
    #[case(0x1234_5678)]
    #[case(u32::MAX)]
    fn expand_and_shrink_are_exact_inverses(#[case] raw: u32) {
        let u = UnitInterval::new(raw);
        assert_eq!(u.expand().shrink(), u);
    }

    #[test]
    fn into_symmetric_preserves_value_for_a_representable_unit_value() {
        let u = UnitInterval::new(0x4000_0000); // 0.25
        let s: SymmetricUnitInterval = u.into();
        assert_eq!(s, SymmetricUnitInterval::new(0x2000_0000)); // still 0.25
    }

    #[test]
    fn into_unit_preserves_value_for_a_nonnegative_symmetric_value() {
        let s = SymmetricUnitInterval::new(0x2000_0000); // 0.25
        let u: UnitInterval = s.into();
        assert_eq!(u, UnitInterval::new(0x4000_0000)); // still 0.25
    }

    #[test]
    fn into_unit_wraps_a_negative_symmetric_value_by_its_cyclic_span() {
        let s = SymmetricUnitInterval::new(-1); // just below 0
        let u: UnitInterval = s.into();
        assert_eq!(u, UnitInterval::new(u32::MAX - 1)); // just below 1
    }

    #[test]
    fn symmetric_addition_wraps_at_the_top() {
        let a = SymmetricUnitInterval::new(i32::MAX);
        let b = SymmetricUnitInterval::new(2);
        assert_eq!(a + b, SymmetricUnitInterval::new(i32::MIN + 1));
    }

    #[test]
    fn symmetric_subtraction_wraps_at_the_bottom() {
        let a = SymmetricUnitInterval::new(i32::MIN);
        let b = SymmetricUnitInterval::new(1);
        assert_eq!(a - b, SymmetricUnitInterval::new(i32::MAX));
    }

    #[test]
    fn symmetric_saturating_add_clamps_at_max() {
        let a = SymmetricUnitInterval::new(i32::MAX - 1);
        let b = SymmetricUnitInterval::new(10);
        assert_eq!(a.saturating_add(b), SymmetricUnitInterval::new(i32::MAX));
    }

    #[test]
    fn symmetric_saturating_sub_clamps_at_min() {
        let a = SymmetricUnitInterval::new(i32::MIN + 1);
        let b = SymmetricUnitInterval::new(10);
        assert_eq!(a.saturating_sub(b), SymmetricUnitInterval::new(i32::MIN));
    }

    #[test]
    fn symmetric_mul_scales_and_wraps() {
        let a = SymmetricUnitInterval::new(0x3000_0000); // 0.375
        assert_eq!(
            a * 3,
            SymmetricUnitInterval::new(0x3000_0000i32.wrapping_mul(3))
        );
    }

    #[rstest]
    #[case(0.0, 0)]
    #[case(0.25, 0x4000_0000)]
    #[case(0.5, 0x8000_0000)]
    fn unit_interval_from_f32_matches_expected_raw(#[case] value: f32, #[case] expected: u32) {
        assert_eq!(UnitInterval::from(value), UnitInterval::new(expected));
    }

    #[rstest]
    #[case(-1.0)]
    #[case(-0.001)]
    #[case(f32::NAN)]
    fn unit_interval_from_f32_saturates_to_zero_at_or_below_the_bottom(#[case] value: f32) {
        assert_eq!(UnitInterval::from(value), UnitInterval::new(0));
    }

    #[rstest]
    #[case(1.0)]
    #[case(2.0)]
    #[case(f32::INFINITY)]
    fn unit_interval_from_f32_saturates_to_max_at_or_above_the_top(#[case] value: f32) {
        assert_eq!(UnitInterval::from(value), UnitInterval::new(u32::MAX));
    }

    #[test]
    fn unit_interval_to_f32_round_trip_for_a_quarter() {
        let u = UnitInterval::new(0x4000_0000);
        assert_eq!(f32::from(u), 0.25);
    }

    #[rstest]
    #[case(0.0, 0)]
    #[case(0.25, 0x2000_0000)]
    #[case(-0.5, -0x4000_0000)]
    fn symmetric_from_f32_matches_expected_raw(#[case] value: f32, #[case] expected: i32) {
        assert_eq!(
            SymmetricUnitInterval::from(value),
            SymmetricUnitInterval::new(expected)
        );
    }

    #[rstest]
    #[case(-1.0)]
    #[case(-2.0)]
    #[case(f32::NEG_INFINITY)]
    fn symmetric_from_f32_saturates_to_min_at_or_below_the_bottom(#[case] value: f32) {
        assert_eq!(
            SymmetricUnitInterval::from(value),
            SymmetricUnitInterval::new(i32::MIN)
        );
    }

    #[rstest]
    #[case(1.0)]
    #[case(2.0)]
    #[case(f32::INFINITY)]
    fn symmetric_from_f32_saturates_to_max_at_or_above_the_top(#[case] value: f32) {
        assert_eq!(
            SymmetricUnitInterval::from(value),
            SymmetricUnitInterval::new(i32::MAX)
        );
    }

    #[test]
    fn symmetric_from_f32_nan_becomes_zero() {
        assert_eq!(
            SymmetricUnitInterval::from(f32::NAN),
            SymmetricUnitInterval::new(0)
        );
    }

    #[test]
    fn symmetric_to_f32_round_trip_for_a_quarter() {
        let s = SymmetricUnitInterval::new(0x2000_0000);
        assert_eq!(f32::from(s), 0.25);
    }
}
