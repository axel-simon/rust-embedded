/// Divides `u64` values by a fixed `u32` divisor without using a 64-bit
/// division instruction.
///
/// Cortex-M4 has no hardware support for 64-bit division: `u64 / u32` gets
/// lowered to a call into a software (libgcc/compiler-rt) division routine,
/// which is comparatively slow. This type instead precomputes a 96-bit
/// reciprocal of the divisor once (in [`U64Divider::new`]) and then turns
/// each division into a handful of cheap 32x32 -> 64 bit multiplications
/// (`UMULL` on Cortex-M4) plus additions.
#[derive(Debug, Clone, Copy)]
pub struct U64Divider {
    // 96-bit reciprocal floor(2^96 / divisor) + 1, split into 32-bit limbs:
    // reciprocal == r0 + r1 * 2^32 + r2 * 2^64. See `new` for why it's
    // rounded up rather than down.
    r0: u32,
    r1: u32,
    r2: u32,
}

impl U64Divider {
    /// Precomputes the reciprocal needed to divide by `divisor`.
    ///
    /// Panics if `divisor` is zero or one: `floor(2^96 / 1) == 2^96` doesn't
    /// fit in the 96-bit reciprocal representation.
    pub fn new(divisor: u32) -> Self {
        assert!(divisor > 1, "divisor must be greater than 1");

        // `mul_high96x64` computes `floor(n * reciprocal / 2^96)`. With the
        // plain floored reciprocal `floor(2^96 / divisor)`, that's either
        // the exact quotient or one less — an undershoot that would need a
        // correction step afterwards. Rounding the reciprocal up by one
        // instead fixes this: it adds a fractional `n / 2^96` to the result, just
        // enough to cover the undershooting case without ever pushing an
        // already-exact result past it. See the module tests for the
        // exhaustive check.
        let reciprocal: u128 = (1u128 << 96) / (divisor as u128) + 1;
        Self {
            r0: reciprocal as u32,
            r1: (reciprocal >> 32) as u32,
            r2: (reciprocal >> 64) as u32,
        }
    }

    /// Returns `n / divisor`.
    pub fn divide(&self, n: u64) -> u64 {
        mul_high96x64(n, self.r0, self.r1, self.r2)
    }
}

impl From<u32> for U64Divider {
    fn from(divisor: u32) -> Self {
        U64Divider::new(divisor)
    }
}

/// Computes `n * r`, where `r` is one 32-bit limb of the reciprocal, via two
/// 32x32 -> 64 bit multiplications (splitting `n` into halves the way a
/// 32-bit multiplier has to). The exact result fits in 96 bits, i.e. safely
/// within a `u128`.
fn mul64x32(n: u64, r: u32) -> u128 {
    let n_lo = n as u32 as u64;
    let n_hi = n >> 32;
    let r = r as u64;

    let lo = n_lo * r; // 32x32 -> 64
    let hi = n_hi * r; // 32x32 -> 64

    (lo as u128) + ((hi as u128) << 32)
}

/// Computes `floor(n * (r0 + r1*2^32 + r2*2^64) / 2^96)`, using only
/// 32x32 -> 64 bit multiplications (via [`mul64x32`]).
///
/// The three 96-bit-range terms can't just be shifted into a single `u128`
/// accumulator at their true weights (0, 32, 64): the top term alone would
/// need up to 160 bits. Instead each term is folded in one 32-bit column at
/// a time, discarding the fully-resolved low bits before adding the next
/// term — the same thing a long multiplication by hand does.
fn mul_high96x64(n: u64, r0: u32, r1: u32, r2: u32) -> u64 {
    let acc = mul64x32(n, r0);
    let acc = (acc >> 32) + mul64x32(n, r1);
    let acc = (acc >> 32) + mul64x32(n, r2);
    (acc >> 32) as u64
}

/// Divides `i64` values by a fixed positive `u32` divisor without using a
/// 64-bit division instruction.
///
/// The divisor is always positive, so only the dividend's sign needs
/// tracking: this just divides `n`'s magnitude with [`U64Divider`] and
/// re-applies the sign, which is exactly what truncating-toward-zero
/// signed division (Rust's `/`) means for a positive divisor.
#[derive(Debug, Clone, Copy)]
pub struct I64Divider {
    inner: U64Divider,
}

impl I64Divider {
    /// Builds an `I64Divider` either from a raw `u32` divisor (panicking if
    /// it's zero or one, see [`U64Divider::new`]) or by reusing an
    /// already-built [`U64Divider`] (which has already been validated, so
    /// this can't panic).
    pub fn new(source: impl Into<U64Divider>) -> Self {
        Self {
            inner: source.into(),
        }
    }

    /// Returns `n / divisor`, truncated toward zero (matching Rust's `/`).
    pub fn divide(&self, n: i64) -> i64 {
        // `unsigned_abs` (rather than `abs() as u64`) also handles
        // `i64::MIN`, whose magnitude (2^63) doesn't fit in an `i64`.
        let magnitude = self.inner.divide(n.unsigned_abs()) as i64;
        if n < 0 {
            -magnitude
        } else {
            magnitude
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{I64Divider, U64Divider};
    use rstest::rstest;

    #[rstest]
    // small divisors
    #[case(2)]
    #[case(3)]
    #[case(7)]
    #[case(251)]
    // mid-sized divisors
    #[case(65_536)]
    #[case(1_000_003)]
    #[case(123_456_789)]
    // large divisors, close to 2^32
    #[case(3_000_000_000)]
    #[case(4_294_967_293)]
    #[case(4_294_967_294)]
    #[case(4_294_967_295)] // u32::MAX
    fn u64_divide_matches_true_division_near_0_and_near_u64_max(#[case] divisor: u32) {
        let d = U64Divider::new(divisor);
        let x = divisor as u64;

        for k in 1..=50u64 {
            let base = k * x;

            for n in [base - 1, base, base + 1] {
                assert_eq!(d.divide(n), n / x, "divisor={divisor} n={n} (near 0)");
            }

            let top = u64::MAX - base;
            for n in [top - 1, top, top + 1] {
                assert_eq!(
                    d.divide(n),
                    n / x,
                    "divisor={divisor} n={n} (near u64::MAX)"
                );
            }
        }
    }

    #[test]
    #[should_panic]
    fn u64_new_panics_on_zero_divisor() {
        U64Divider::new(0);
    }

    #[test]
    #[should_panic]
    fn u64_new_panics_on_one_divisor() {
        U64Divider::new(1);
    }

    #[rstest]
    // small divisors
    #[case(2)]
    #[case(3)]
    #[case(7)]
    #[case(251)]
    // mid-sized divisors
    #[case(65_536)]
    #[case(1_000_003)]
    #[case(123_456_789)]
    // large divisors, close to 2^32
    #[case(3_000_000_000)]
    #[case(4_294_967_293)]
    #[case(4_294_967_294)]
    #[case(4_294_967_295)] // u32::MAX
    fn i64_divide_matches_true_division_near_0_and_near_i64_extremes(#[case] divisor: u32) {
        let d = I64Divider::new(divisor);
        let x = divisor as i64;

        for k in 1..=50i64 {
            let base = k * x;

            for n in [base - 1, base, base + 1] {
                assert_eq!(
                    d.divide(n),
                    n / x,
                    "divisor={divisor} n={n} (near 0, positive)"
                );
                let neg_n = -n;
                assert_eq!(
                    d.divide(neg_n),
                    neg_n / x,
                    "divisor={divisor} n={neg_n} (near 0, negative)"
                );
            }

            let top = i64::MAX - base;
            for n in [top - 1, top, top + 1] {
                assert_eq!(
                    d.divide(n),
                    n / x,
                    "divisor={divisor} n={n} (near i64::MAX)"
                );
            }

            let bottom = i64::MIN + base;
            for n in [bottom - 1, bottom, bottom + 1] {
                assert_eq!(
                    d.divide(n),
                    n / x,
                    "divisor={divisor} n={n} (near i64::MIN)"
                );
            }
        }
    }

    #[test]
    #[should_panic]
    fn i64_new_panics_on_zero_divisor() {
        I64Divider::new(0);
    }

    #[test]
    #[should_panic]
    fn i64_new_panics_on_one_divisor() {
        I64Divider::new(1);
    }

    #[test]
    fn i64_new_from_existing_u64_divider() {
        let u64_divider = U64Divider::new(7);
        let i64_divider = I64Divider::new(u64_divider);
        assert_eq!(i64_divider.divide(-100), -100 / 7);
    }
}
