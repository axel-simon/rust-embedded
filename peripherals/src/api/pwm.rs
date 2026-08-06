//! Hardware-agnostic center-aligned PWM API. Concrete drivers (real or
//! fake) implement [`PwmTrait`]; nothing in this module depends on any
//! particular chip.

use common::duration::Duration;
use common::unit_interval::UnitInterval;

/// Every timer instance capable of driving this API's kind of PWM — needs
/// an advanced-control timer's break/dead-time generator and internal
/// channels 5/6, which only `TIM1`/`TIM8`/`TIM20` have.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PwmTimer {
    Stm32g4Tim1 = 0,
    Stm32g4Tim8 = 1,
    Stm32g4Tim20 = 2,
}

/// The largest [`PwmOptions::channels`] can request: channels 1-3
/// (0-indexed 0-2), the only ones with a complementary output pin. Channel
/// 4 has no complementary pin, and channels 5/6 are internal-only (see
/// [`PwmOptions::mid_point_trigger`]) — neither is reachable through
/// [`PwmTrait::set_duty_cycle`].
pub const MAX_PWM_CHANNELS: u8 = 3;

/// Configuration for opening a [`PwmTrait`] driver. The PWM is always
/// center-aligned (the counter runs up then down, rather than free-running
/// and wrapping), so every channel's pulse is symmetric about the same
/// instant each period — the counter's peak, which
/// [`Self::mid_point_trigger`] can raise a trigger signal at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmOptions {
    frequency_hz: u32,
    channels: u8,
    dead_time: Duration,
    mid_point_trigger: bool,
}

impl PwmOptions {
    /// Builds a `PwmOptions` — see each field's accessor below for its
    /// meaning.
    ///
    /// # Panics
    /// Panics if `frequency_hz` is `0`, `channels` is `0` or exceeds
    /// [`MAX_PWM_CHANNELS`], or `dead_time` is negative.
    pub const fn new(
        frequency_hz: u32,
        channels: u8,
        dead_time: Duration,
        mid_point_trigger: bool,
    ) -> Self {
        assert!(frequency_hz > 0, "frequency_hz must be nonzero");
        assert!(
            channels >= 1 && channels <= MAX_PWM_CHANNELS,
            "channels must be in 1..=MAX_PWM_CHANNELS"
        );
        assert!(dead_time.raw() >= 0, "dead_time must not be negative");
        PwmOptions {
            frequency_hz,
            channels,
            dead_time,
            mid_point_trigger,
        }
    }

    /// The PWM switching frequency — how many up/down-counting periods
    /// complete per second.
    pub const fn frequency_hz(&self) -> u32 {
        self.frequency_hz
    }

    /// How many of [`MAX_PWM_CHANNELS`] complementary channel pairs to
    /// configure — the exclusive upper bound on
    /// [`PwmTrait::set_duty_cycle`]'s `channel`.
    pub const fn channels(&self) -> u8 {
        self.channels
    }

    /// The gap enforced, on every channel, between one side of its
    /// complementary pair switching off and the other switching on (so
    /// both are never briefly active together, which would short the
    /// power stage they drive). A zero `Duration` disables dead-time
    /// insertion.
    pub const fn dead_time(&self) -> Duration {
        self.dead_time
    }

    /// Whether to also raise a trigger signal at the mid-point of every
    /// period (the counter's peak), on top of the update-event trigger
    /// every open driver always raises at the start of each period — e.g.
    /// for synchronizing an ADC conversion away from any channel's
    /// switching edges.
    pub const fn mid_point_trigger(&self) -> bool {
        self.mid_point_trigger
    }
}

/// Abstract interface implemented by every center-aligned PWM driver, real
/// or fake — see [`PwmOptions`] for what center alignment, dead time, and
/// the mid-point trigger mean.
pub trait PwmTrait {
    /// Configures the timer per `options`, arms its always-on
    /// update-event trigger (once per period, at the start of the count —
    /// unconditional, unlike [`PwmOptions::mid_point_trigger`]), and
    /// starts generating PWM immediately, with every channel's duty cycle
    /// at `0`.
    fn open(&mut self, options: PwmOptions);

    /// Stops generating PWM and disables every output.
    /// [`Self::open`] must be called again before [`Self::set_duty_cycle`].
    fn close(&mut self);

    /// Sets `channel`'s duty cycle — the fraction of each period its output
    /// is active — to `value`. `channel` is `0`-indexed, within the range
    /// [`Self::open`]'s [`PwmOptions::channels`] configured.
    fn set_duty_cycle(&mut self, channel: u8, value: UnitInterval);

    /// Changes the dead time live, without restarting PWM output (unlike
    /// re-calling [`Self::open`]). Meant for calibrating
    /// [`PwmOptions::dead_time`] by hand in a benchtest setting — e.g.
    /// sweeping it via a potentiometer while watching an oscilloscope for
    /// shoot-through — not for routine use once a board's dead time is
    /// known.
    fn set_dead_time(&mut self, dead_time: Duration);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_stores_the_given_fields() {
        let options = PwmOptions::new(20_000, 3, Duration::from_nanos(500), true);
        assert_eq!(options.frequency_hz(), 20_000);
        assert_eq!(options.channels(), 3);
        assert_eq!(options.dead_time(), Duration::from_nanos(500));
        assert!(options.mid_point_trigger());
    }

    #[test]
    fn new_accepts_the_minimum_and_maximum_channel_count() {
        assert_eq!(PwmOptions::new(1, 1, Duration::new(0), false).channels(), 1);
        assert_eq!(
            PwmOptions::new(1, MAX_PWM_CHANNELS, Duration::new(0), false).channels(),
            MAX_PWM_CHANNELS
        );
    }

    #[test]
    #[should_panic(expected = "frequency_hz must be nonzero")]
    fn new_panics_on_zero_frequency() {
        PwmOptions::new(0, 1, Duration::new(0), false);
    }

    #[test]
    #[should_panic(expected = "channels must be in 1..=MAX_PWM_CHANNELS")]
    fn new_panics_on_zero_channels() {
        PwmOptions::new(20_000, 0, Duration::new(0), false);
    }

    #[test]
    #[should_panic(expected = "channels must be in 1..=MAX_PWM_CHANNELS")]
    fn new_panics_on_too_many_channels() {
        PwmOptions::new(20_000, MAX_PWM_CHANNELS + 1, Duration::new(0), false);
    }

    #[test]
    #[should_panic(expected = "dead_time must not be negative")]
    fn new_panics_on_negative_dead_time() {
        PwmOptions::new(20_000, 1, Duration::new(-1), false);
    }

    #[test]
    fn constructible_in_const_context() {
        const OPTIONS: PwmOptions = PwmOptions::new(20_000, 2, Duration::from_nanos(250), false);
        assert_eq!(OPTIONS.channels(), 2);
    }
}
