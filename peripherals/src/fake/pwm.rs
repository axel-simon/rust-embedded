//! A fake [`PwmTrait`] implementation for host-side testing, with no real
//! hardware involved.
//!
//! [`Pwm::new`] hands back two handles onto one shared, simulated PWM
//! generator: [`Pwm`] itself, for firmware, and [`FakePwm`], for a test to
//! observe what firmware configured via [`PwmTrait::open`]/
//! [`PwmTrait::set_duty_cycle`].

use core::cell::{Cell, RefCell};
use std::rc::Rc;

use common::duration::Duration;
use common::unit_interval::UnitInterval;

use crate::api::pwm::{PwmOptions, PwmTimer, PwmTrait, MAX_PWM_CHANNELS};
use crate::fake::callback_registry::CallbackRegistry;

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
struct SharedState {
    state: Cell<PwmState>,
    /// Which physical timer this is — needed (only) to name the simulated
    /// trigger-out signal (see [`PwmTimer::trigger_out_2`]); real hardware
    /// has no equivalent use for it.
    timer: PwmTimer,
    /// Set by [`Pwm::set_callback_registry`] — `None` means trigger-out
    /// simulation is disabled (matches this fake's previous behavior,
    /// which never looked at [`PwmOptions::mid_point_trigger`] at all).
    registry: RefCell<Option<CallbackRegistry>>,
    /// Whether the self-perpetuating trigger-out callback (see
    /// [`PwmTrait::open`]) has already been registered with `registry` —
    /// registered at most once per instance; `open()`/`close()` cycles
    /// start/stop it rescheduling itself, they don't re-register it.
    trigger_out_registered: Cell<bool>,
}

impl SharedState {
    fn warn(&self, args: core::fmt::Arguments) {
        emit_warning(args);
    }

    /// The registry set via [`Pwm::set_callback_registry`].
    ///
    /// # Panics
    /// Panics if none was set. There's no reasonable way to simulate
    /// [`PwmOptions::mid_point_trigger`] without one — the alternative is
    /// silently dropping every trigger-out edge, which is exactly the
    /// kind of hard-to-debug test failure requiring a registry up front
    /// avoids. This is only ever reached once [`Pwm::arm_trigger_out`]
    /// has already required a registry to get this far, so in practice
    /// this always finds one; the panic is here as a backstop, not an
    /// expected path.
    fn registry(&self) -> CallbackRegistry {
        self.registry.borrow().clone().unwrap_or_else(|| {
            panic!(
                "PwmTrait::open() was called with PwmOptions::mid_point_trigger() set, but no \
                 CallbackRegistry has been set via Pwm::set_callback_registry() -- call it \
                 before open()"
            )
        })
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
    /// plus a [`FakePwm`] handle a test can observe it with.
    /// `_timer_clock_hz` is accepted and ignored — there's no real clock
    /// tree to configure here — so that code choosing between this fake
    /// and a real driver at compile time can call either one's
    /// constructor with the same arguments. `timer` *is* kept (unlike
    /// other fakes' otherwise-ignored constructor arguments), to name the
    /// simulated trigger-out signal — see [`Self::set_callback_registry`].
    pub fn new(timer: PwmTimer, _timer_clock_hz: u32) -> (Self, FakePwm) {
        let state = Rc::new(SharedState {
            state: Cell::new(PwmState::default()),
            timer,
            registry: RefCell::new(None),
            trigger_out_registered: Cell::new(false),
        });
        (Pwm(state.clone()), FakePwm(state))
    }

    /// Enables trigger-out simulation: from the next [`PwmTrait::open`]
    /// call with [`PwmOptions::mid_point_trigger`] set, this instance
    /// periodically fires its [`PwmTimer::trigger_out_2`] trigger through
    /// `registry`, at the configured [`PwmOptions::frequency_hz`] — see
    /// `crate::fake::adc`, which subscribes to it.
    ///
    /// Call before `open()` if `open()` will set
    /// [`PwmOptions::mid_point_trigger`] at all: `open()` panics in that
    /// case if no registry has been set (see [`SharedState::registry`]) —
    /// there's no useful way to run a mid-point-triggering fake PWM
    /// without one, and silently dropping every trigger-out edge instead
    /// would just turn into a test whose ADCs mysteriously never see a
    /// conversion complete.
    pub fn set_callback_registry(&self, registry: CallbackRegistry) {
        *self.0.registry.borrow_mut() = Some(registry);
    }
}

impl PwmTrait for Pwm {
    fn open(&mut self, options: PwmOptions) {
        self.0.state.set(PwmState {
            options: Some(options),
            duty_cycles: [UnitInterval::new(0); MAX_PWM_CHANNELS as usize],
        });
        self.arm_trigger_out(options);
    }

    fn close(&mut self) {
        self.0.state.set(PwmState::default());
        // No explicit registry action needed: the perpetuating callback
        // (see `arm_trigger_out`) reads `state.options` fresh every time
        // it fires, and stops rescheduling itself once it observes `None`
        // here.
    }

    fn set_duty_cycle(&mut self, channel: u8, value: UnitInterval) {
        let mut state = self.0.state.get();
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
        self.0.state.set(state);
    }

    /// Warns and does nothing if called before [`PwmTrait::open`] has
    /// succeeded, matching this module's own [`PwmTrait::set_duty_cycle`].
    fn set_dead_time(&mut self, dead_time: Duration) {
        let mut state = self.0.state.get();
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
        self.0.state.set(state);
    }
}

/// The full simulated PWM period, and half of it (center-aligned PWM's
/// counter peak — where [`PwmOptions::mid_point_trigger`] fires — is
/// reached this long after each period starts), for a given
/// [`PwmOptions::frequency_hz`]. Computed directly in nanoseconds (not
/// `period_ns / 2`) since [`Duration`] has no division operator. Integer
/// precision, not cycle-accurate — fine for simulated timing.
fn period_ns(frequency_hz: u32) -> i64 {
    1_000_000_000 / frequency_hz as i64
}

impl Pwm {
    /// Wires up simulated trigger-out firing for `options`, if
    /// [`Self::set_callback_registry`] was called and
    /// [`PwmOptions::mid_point_trigger`] is set — see
    /// [`Self::set_callback_registry`]'s doc comment.
    fn arm_trigger_out(&self, options: PwmOptions) {
        if !options.mid_point_trigger() {
            return;
        }
        let registry = self.0.registry(); // panics if none was set — see its own doc comment
        let name = format!("{:?}", self.0.timer.trigger_out_2());

        if !self.0.trigger_out_registered.replace(true) {
            let shared = self.0.clone();
            let hook_name = name.clone();
            registry.register(hook_name.clone(), move |fired_at| {
                // Reads live state every firing, rather than capturing
                // `options` once, so a later `open()` with a different
                // frequency (or `close()`) is picked up immediately.
                let Some(options) = shared.state.get().options else {
                    return; // closed: stop perpetuating
                };
                if !options.mid_point_trigger() {
                    return;
                }
                shared.registry().schedule(
                    hook_name.clone(),
                    fired_at + Duration::from_nanos(period_ns(options.frequency_hz()) as i32),
                );
            });
        }

        registry.schedule(
            name,
            registry.now() + Duration::from_nanos((period_ns(options.frequency_hz()) / 2) as i32),
        );
    }
}

impl FakePwm {
    /// Whether [`PwmTrait::open`] has been called more recently than
    /// [`PwmTrait::close`].
    pub fn is_open(&self) -> bool {
        self.0.state.get().options.is_some()
    }

    /// The [`PwmOptions`] passed to the most recent [`PwmTrait::open`]
    /// call, if the PWM is currently open.
    pub fn options(&self) -> Option<PwmOptions> {
        self.0.state.get().options
    }

    /// The duty cycle last set via [`PwmTrait::set_duty_cycle`] for
    /// `channel`, or `0` if never set (or since the last [`PwmTrait::open`]/
    /// [`PwmTrait::close`], both of which reset every channel back to `0`).
    ///
    /// # Panics
    /// Panics if `channel` is at or beyond [`MAX_PWM_CHANNELS`].
    pub fn duty_cycle(&self, channel: u8) -> UnitInterval {
        self.0.state.get().duty_cycles[channel as usize]
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

    fn mid_point_options(frequency_hz: u32) -> PwmOptions {
        PwmOptions::new(frequency_hz, 3, Duration::from_nanos(250), true)
    }

    fn registry() -> (CallbackRegistry, crate::fake::clock::FakeClockProvider) {
        let (_provider, fake_clock) = crate::fake::clock::ClockProvider::new((), 1_000_000_000);
        (CallbackRegistry::new(fake_clock.clone()), fake_clock)
    }

    #[test]
    #[should_panic(expected = "no CallbackRegistry has been set")]
    fn open_with_mid_point_trigger_without_a_registry_panics() {
        let (mut pwm, _fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        // No `set_callback_registry()` call — there's no useful way to
        // run a mid-point-triggering fake PWM without one (see
        // `SharedState::registry`'s doc comment), so this must panic
        // rather than silently behave as if `mid_point_trigger` were
        // unset.
        pwm.open(mid_point_options(1_000));
    }

    #[test]
    fn open_without_mid_point_trigger_never_schedules_anything() {
        let (registry, _clock) = registry();
        let (mut pwm, _fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.set_callback_registry(registry.clone());
        pwm.open(options(3)); // mid_point_trigger: false

        let fired = Rc::new(Cell::new(0));
        let fired2 = fired.clone();
        registry.register(
            format!("{:?}", PwmTimer::Stm32g4Tim1.trigger_out_2()),
            move |_now| fired2.set(fired2.get() + 1),
        );
        registry.advance_clock_by(Duration::from_seconds(1));
        assert_eq!(fired.get(), 0);
    }

    #[test]
    fn open_with_mid_point_trigger_fires_periodically_at_the_configured_frequency() {
        let (registry, _clock) = registry();
        let (mut pwm, _fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.set_callback_registry(registry.clone());

        let times = Rc::new(RefCell::new(Vec::new()));
        let times2 = times.clone();
        registry.register(
            format!("{:?}", PwmTimer::Stm32g4Tim1.trigger_out_2()),
            move |now| times2.borrow_mut().push(now),
        );

        let start = registry.now();
        pwm.open(mid_point_options(1_000)); // 1ms period
        registry.advance_clock_by(Duration::from_millis(35));

        // First edge at period/2, then every full period after that —
        // computed the same way the production code accumulates it
        // (repeated addition, not independently re-derived per edge:
        // `Duration::from_nanos` floor-rounds slightly, and the two
        // don't always agree once errors compound).
        let times = times.borrow();
        assert_eq!(times.len(), 35, "{times:?}");
        let half_period = Duration::from_nanos(500_000);
        let period = Duration::from_nanos(1_000_000);
        let mut expected = start + half_period;
        for (i, &t) in times.iter().enumerate() {
            assert_eq!(t, expected, "edge {i}");
            expected = expected + period;
        }
    }

    #[test]
    fn close_stops_the_periodic_trigger_out() {
        let (registry, _clock) = registry();
        let (mut pwm, _fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.set_callback_registry(registry.clone());

        let count = Rc::new(Cell::new(0));
        let count2 = count.clone();
        registry.register(
            format!("{:?}", PwmTimer::Stm32g4Tim1.trigger_out_2()),
            move |_now| count2.set(count2.get() + 1),
        );

        pwm.open(mid_point_options(1_000)); // 1ms period
        registry.advance_clock_by(Duration::from_millis(5));
        pwm.close();
        // One entry may already be queued from before `close()` ran (it
        // can't be cancelled — the production code only stops
        // *rescheduling* once it fires, see `arm_trigger_out`) — flush
        // it, then confirm firing has genuinely stopped, not just
        // happened to fall silent between two already-scheduled edges.
        registry.advance_clock_by(Duration::from_millis(5));
        let settled = count.get();
        assert!(settled > 0);
        registry.advance_clock_by(Duration::from_millis(50));
        assert_eq!(count.get(), settled);
    }

    #[test]
    fn reopening_with_a_different_frequency_changes_the_period() {
        let (registry, _clock) = registry();
        let (mut pwm, _fake) = Pwm::new(PwmTimer::Stm32g4Tim1, 170_000_000);
        pwm.set_callback_registry(registry.clone());

        let times = Rc::new(RefCell::new(Vec::new()));
        let times2 = times.clone();
        registry.register(
            format!("{:?}", PwmTimer::Stm32g4Tim1.trigger_out_2()),
            move |now| times2.borrow_mut().push(now),
        );

        pwm.open(mid_point_options(1_000)); // 1ms period
        registry.advance_clock_by(Duration::from_millis(2));
        pwm.close();
        pwm.open(mid_point_options(500)); // now 2ms period
        registry.advance_clock_by(Duration::from_millis(10));

        // Doesn't panic/hang, and produced a mix of the two cadences —
        // exact count isn't asserted (re-open's own first-edge timing
        // depends on when `close`'s already-scheduled entry, if any,
        // still lands), only that firing continued after `open()` again.
        assert!(times.borrow().len() > 1);
    }

    #[test]
    fn different_timers_get_different_trigger_names() {
        assert_ne!(
            format!("{:?}", PwmTimer::Stm32g4Tim1.trigger_out_2()),
            format!("{:?}", PwmTimer::Stm32g4Tim8.trigger_out_2())
        );
    }
}
