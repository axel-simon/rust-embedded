//! A first-order low-pass filter over `f32`.

/// A first-order (single-pole) low-pass filter, implemented as an
/// exponential moving average: `output += (measurement - output) /
/// (time_constant + 1)`.
///
/// The time constant's unit is abstract — it's expressed in units of
/// [`Self::poll`] calls, not any real time unit, so it means whatever the
/// caller's actual polling rate implies (e.g. a time constant of `20.0`
/// polled once per millisecond behaves like a ~20ms RC low-pass; the same
/// `20.0` polled once per PWM period instead tracks 20 periods).
///
/// [`Self::new`]'s time constant only applies once the filter has settled
/// — see [`Self::poll`].
pub struct LowPassFilter {
    nominal_time_constant: f32,
    /// Ramps from `0.0` up to `nominal_time_constant`, `1.0` per
    /// [`Self::poll`] call — see [`Self::poll`]'s doc comment.
    current_time_constant: f32,
    value: f32,
}

impl LowPassFilter {
    /// Builds a `LowPassFilter` with the given nominal time constant
    /// (see the type's doc comment for its unit), initially outputting
    /// `0.0` until the first [`Self::poll`] call.
    pub const fn new(time_constant: f32) -> Self {
        LowPassFilter {
            nominal_time_constant: time_constant,
            current_time_constant: 0.0,
            value: 0.0,
        }
    }

    /// Folds in one new measurement. The *effective* time constant used
    /// starts at `0` (so the very first call sets [`Self::output`] to
    /// exactly that first measurement, rather than slowly decaying in
    /// from `0.0`) and increases by one call's worth on every
    /// subsequent call, up to [`Self::new`]'s nominal time constant —
    /// while it's still ramping up, this makes the filter compute the
    /// exact running mean of every sample seen so far (the `n`th sample
    /// is weighted `1/n`, matching a plain cumulative average), so early
    /// samples get averaged together rather than lost to a slow
    /// exponential decay; once the ramp reaches the nominal time
    /// constant, it settles into a steady-state exponential moving
    /// average from then on.
    pub fn poll(&mut self, measurement: f32) {
        let alpha = 1.0 / (self.current_time_constant + 1.0);
        self.value += alpha * (measurement - self.value);
        self.current_time_constant =
            (self.current_time_constant + 1.0).min(self.nominal_time_constant);
    }

    /// The filter's current output — the most recent value [`Self::poll`]
    /// computed, or `0.0` if it's never been called.
    pub const fn output(&self) -> f32 {
        self.value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_at_zero_before_any_poll() {
        assert_eq!(LowPassFilter::new(10.0).output(), 0.0);
    }

    #[test]
    fn the_first_poll_sets_the_output_to_exactly_that_measurement() {
        let mut filter = LowPassFilter::new(10.0);
        filter.poll(1234.5);
        assert_eq!(filter.output(), 1234.5);
    }

    #[test]
    fn while_ramping_up_the_output_is_the_exact_running_mean() {
        // nominal time constant of 100 -- comfortably larger than the 3
        // samples below, so the ramp never catches up and this stays a
        // plain cumulative average throughout.
        let mut filter = LowPassFilter::new(100.0);
        filter.poll(10.0);
        filter.poll(20.0);
        filter.poll(30.0);
        assert_eq!(filter.output(), 20.0); // (10 + 20 + 30) / 3
    }

    #[test]
    fn settles_into_a_steady_state_exponential_moving_average() {
        // Nominal time constant of 1: after the ramp settles (poll 2
        // onward), alpha is fixed at 1/(1+1) = 0.5 forever.
        let mut filter = LowPassFilter::new(1.0);
        filter.poll(10.0); // ramp: alpha=1 -> output = 10
        filter.poll(10.0); // ramp reaches nominal: alpha=1/2 -> output = 10
        filter.poll(20.0); // steady state: alpha=1/2 -> output = 10 + 0.5*(20-10) = 15
        assert_eq!(filter.output(), 15.0);
        filter.poll(20.0); // alpha=1/2 -> output = 15 + 0.5*(20-15) = 17.5
        assert_eq!(filter.output(), 17.5);
    }

    #[test]
    fn a_zero_time_constant_always_tracks_the_latest_measurement() {
        let mut filter = LowPassFilter::new(0.0);
        filter.poll(5.0);
        assert_eq!(filter.output(), 5.0);
        filter.poll(-3.0);
        assert_eq!(filter.output(), -3.0);
    }
}
