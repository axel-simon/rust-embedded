use core::ops::{Add, Sub};

use crate::duration::Duration;

/// A point in time expressed as the [`Duration`] elapsed since boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Uptime(Duration);

impl Uptime {
    /// The `Uptime` at boot, i.e. zero time elapsed.
    pub const fn epoch() -> Self {
        Uptime(Duration::new(0))
    }

    /// Whole seconds since boot, rounded down.
    pub const fn seconds(self) -> i32 {
        self.0.seconds()
    }

    /// Fractional part of a second since boot, as a fraction of 2^32.
    pub const fn fraction(self) -> u32 {
        self.0.fraction()
    }

    /// Fractional part of a second since boot, in milliseconds.
    pub fn fraction_as_millis(self) -> u32 {
        self.0.fraction_as_millis()
    }

    /// Fractional part of a second since boot, in microseconds.
    pub fn fraction_as_micros(self) -> u32 {
        self.0.fraction_as_micros()
    }

    /// Fractional part of a second since boot, in nanoseconds.
    pub fn fraction_as_nanos(self) -> u32 {
        self.0.fraction_as_nanos()
    }
}

impl Add<Duration> for Uptime {
    type Output = Uptime;

    fn add(self, rhs: Duration) -> Uptime {
        Uptime(self.0 + rhs)
    }
}

impl Sub<Duration> for Uptime {
    type Output = Uptime;

    fn sub(self, rhs: Duration) -> Uptime {
        Uptime(self.0 - rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::Uptime;
    use crate::duration::Duration;

    const ONE_SECOND: i64 = 1 << 32;

    #[test]
    fn epoch_is_zero() {
        let u = Uptime::epoch();
        assert_eq!(u.seconds(), 0);
        assert_eq!(u.fraction(), 0);
    }

    #[test]
    fn addition_of_a_duration() {
        let u = Uptime::epoch() + Duration::new(ONE_SECOND + (1 << 31));
        assert_eq!(u.seconds(), 1);
        assert_eq!(u.fraction(), 1 << 31);
    }

    #[test]
    fn subtraction_of_a_duration() {
        let u = (Uptime::epoch() + Duration::new(2 * ONE_SECOND)) - Duration::new(ONE_SECOND);
        assert_eq!(u.seconds(), 1);
        assert_eq!(u.fraction(), 0);
    }

    #[test]
    fn fraction_as_millis_micros_nanos_for_half_second() {
        let u = Uptime::epoch() + Duration::new(1 << 31);
        assert_eq!(u.fraction_as_millis(), 500);
        assert_eq!(u.fraction_as_micros(), 500_000);
        assert_eq!(u.fraction_as_nanos(), 500_000_000);
    }
}
