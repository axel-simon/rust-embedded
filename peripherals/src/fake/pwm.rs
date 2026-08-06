//! A fake [`PwmTrait`] implementation for host-side testing, with no real
//! hardware involved.
//!
//! [`Pwm::new`] hands back two handles onto one shared, simulated PWM
//! generator: [`Pwm`] itself, for firmware, and [`FakePwm`], for a test to
//! observe what firmware configured via [`PwmTrait::open`]/
//! [`PwmTrait::set_duty_cycle`].

use core::cell::Cell;
use std::rc::Rc;

use common::duration::Duration;
use common::unit_interval::UnitInterval;

use crate::api::pwm::{PwmOptions, PwmTimer, PwmTrait, MAX_PWM_CHANNELS};

/// The simulated PWM's state — see [`SharedState`].
#[derive(Clone, Copy)]
struct PwmState {
    /// `None` while closed. Set by [`PwmTrait::open`], cleared by
    /// [`PwmTrait::close`].
    options: Option<PwmOptions>,
    /// Duty cycles last set via [`PwmTrait::set_duty_cycle`], indexed by
    /// channel. Meaningless (and reset to `0`) for any channel beyond
    /// whatever [`PwmOptions::channels`] `options` currently holds.
    duty_cycles: [UnitInterval; MAX_PWM_CHANNELS as usize],
}

impl Default for PwmState {
    fn default() -> Self {
        PwmState {
            options: None,
            duty_cycles: [UnitInterval::new(0); MAX_PWM_CHANNELS as usize],
        }
    }
}

/// The simulated PWM state shared between a [`Pwm`] and its [`FakePwm`]
/// counterpart (see [`Pwm::new`]).
struct SharedState(Cell<PwmState>);

impl SharedState {
    fn warn(&self, args: core::fmt::Arguments) {
        emit_warning(args);
    }
}

/// A fake [`PwmTrait`] driver simulating a PWM generator's open/close/
/// duty-cycle state, without touching any real hardware.
pub struct Pwm(Rc<SharedState>);

/// A test's handle onto the same simulated PWM generator a [`Pwm`] drives —
/// see [`Pwm::new`]. Cheap to [`Clone`] (all clones share the same
/// underlying state).
#[derive(Clone)]
pub struct FakePwm(Rc<SharedState>);

impl Pwm {
    /// Creates an unopened fake PWM generator (see [`PwmTrait::open`]),
    /// plus a [`FakePwm`] handle a test can observe it with. `_timer`/
    /// `_timer_clock_hz` are accepted and ignored — there's no real timer
    /// instance or clock tree to select or configure here — so that code
    /// choosing between this fake and a real driver at compile time can
    /// call either one's constructor with the same arguments.
    pub fn new(_timer: PwmTimer, _timer_clock_hz: u32) -> (Self, FakePwm) {
        let state = Rc::new(SharedState(Cell::new(PwmState::default())));
        (Pwm(state.clone()), FakePwm(state))
    }
}

impl PwmTrait for Pwm {
    fn open(&mut self, options: PwmOptions) {
        self.0 .0.set(PwmState {
            options: Some(options),
            duty_cycles: [UnitInterval::new(0); MAX_PWM_CHANNELS as usize],
        });
    }

    fn close(&mut self) {
        self.0 .0.set(PwmState::default());
    }

    fn set_duty_cycle(&mut self, channel: u8, value: UnitInterval) {
        let mut state = self.0 .0.get();
        let Some(options) = state.options else {
            self.0.warn(format_args!(
                "set_duty_cycle({channel}) called while the PWM is not open (call open() first)"
            ));
            return;
        };
        if channel >= options.channels() {
            self.0.warn(format_args!(
                "set_duty_cycle({channel}) called outside the {}-channel range configured via open()",
                options.channels()
            ));
            return;
        }
        state.duty_cycles[channel as usize] = value;
        self.0 .0.set(state);
    }

    /// Warns and does nothing if called before [`PwmTrait::open`] has
    /// succeeded, matching this module's own [`PwmTrait::set_duty_cycle`].
    fn set_dead_time(&mut self, dead_time: Duration) {
        let mut state = self.0 .0.get();
        let Some(options) = state.options else {
            self.0.warn(format_args!(
                "set_dead_time() called while the PWM is not open (call open() first)"
            ));
            return;
        };
        state.options = Some(PwmOptions::new(
            options.frequency_hz(),
            options.channels(),
            dead_time,
            options.mid_point_trigger(),
        ));
        self.0 .0.set(state);
    }
}

impl FakePwm {
    /// Whether [`PwmTrait::open`] has been called more recently than
    /// [`PwmTrait::close`].
    pub fn is_open(&self) -> bool {
        self.0 .0.get().options.is_some()
    }

    /// The [`PwmOptions`] passed to the most recent [`PwmTrait::open`]
    /// call, if the PWM is currently open.
    pub fn options(&self) -> Option<PwmOptions> {
        self.0 .0.get().options
    }

    /// The duty cycle last set via [`PwmTrait::set_duty_cycle`] for
    /// `channel`, or `0` if never set (or since the last [`PwmTrait::open`]/
    /// [`PwmTrait::close`], both of which reset every channel back to `0`).
    ///
    /// # Panics
    /// Panics if `channel` is at or beyond [`MAX_PWM_CHANNELS`].
    pub fn duty_cycle(&self, channel: u8) -> UnitInterval {
        self.0 .0.get().duty_cycles[channel as usize]
    }
}

// The embedded target this crate normally builds for has no logging
// backend wired up, so warnings are only surfaced on the host, where
// `cfg(test)` builds have `std` available.
#[cfg(test)]
fn emit_warning(args: core::fmt::Arguments) {
    eprintln!("pwm fake warning: {args}");
}

#[cfg(not(test))]
fn emit_warning(_args: core::fmt::Arguments) {}

#[cfg(test)]
mod tests {
    use common::duration::Duration;

    use super::*;

    fn options(channels: u8) -> PwmOptions {
        PwmOptions::new(20_000, channels, Duration::from_nanos(250), false)
    }

    #[test]
    fn starts_closed_with_no_options() {
        let (_pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        assert!(!fake.is_open());
        assert_eq!(fake.options(), None);
    }

    #[test]
    fn open_records_options_and_marks_open() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        let opts = options(3);
        pwm.open(opts);
        assert!(fake.is_open());
        assert_eq!(fake.options(), Some(opts));
    }

    #[test]
    fn close_marks_closed_and_clears_options() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(3));
        pwm.close();
        assert!(!fake.is_open());
        assert_eq!(fake.options(), None);
    }

    #[test]
    fn every_channel_starts_at_zero_duty_cycle() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(3));
        assert_eq!(fake.duty_cycle(0), UnitInterval::new(0));
        assert_eq!(fake.duty_cycle(1), UnitInterval::new(0));
        assert_eq!(fake.duty_cycle(2), UnitInterval::new(0));
    }

    #[test]
    fn set_duty_cycle_is_observable_via_the_fake() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(3));
        pwm.set_duty_cycle(1, UnitInterval::new(0x8000_0000));
        assert_eq!(fake.duty_cycle(1), UnitInterval::new(0x8000_0000));
        // Untouched channels stay at 0.
        assert_eq!(fake.duty_cycle(0), UnitInterval::new(0));
    }

    #[test]
    fn reopening_resets_every_duty_cycle_to_zero() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(3));
        pwm.set_duty_cycle(0, UnitInterval::new(u32::MAX));
        pwm.open(options(3));
        assert_eq!(fake.duty_cycle(0), UnitInterval::new(0));
    }

    #[test]
    fn set_duty_cycle_while_closed_warns_and_does_nothing() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.set_duty_cycle(0, UnitInterval::new(u32::MAX));
        assert!(!fake.is_open());
    }

    #[test]
    fn set_duty_cycle_outside_the_configured_channel_count_warns_and_does_nothing() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(1));
        pwm.set_duty_cycle(1, UnitInterval::new(u32::MAX));
        assert_eq!(fake.duty_cycle(1), UnitInterval::new(0));
    }

    #[test]
    fn set_dead_time_is_observable_via_the_fakes_options() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(3));
        pwm.set_dead_time(Duration::from_nanos(500));
        assert_eq!(
            fake.options().unwrap().dead_time(),
            Duration::from_nanos(500)
        );
    }

    #[test]
    fn set_dead_time_leaves_every_other_option_untouched() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.open(options(3));
        pwm.set_dead_time(Duration::from_nanos(500));
        let updated = fake.options().unwrap();
        assert_eq!(updated.frequency_hz(), 20_000);
        assert_eq!(updated.channels(), 3);
        assert!(!updated.mid_point_trigger());
    }

    #[test]
    fn set_dead_time_while_closed_warns_and_does_nothing() {
        let (mut pwm, fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.set_dead_time(Duration::from_nanos(500));
        assert!(!fake.is_open());
        assert_eq!(fake.options(), None);
    }
}
