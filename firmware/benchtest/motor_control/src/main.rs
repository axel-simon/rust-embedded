#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

mod foc;

use common::debounce::Debouncer;
use common::ds402;
use common::duration::Duration;
use common::unit_interval::{SymmetricUnitInterval, UnitInterval};
use common::uptime::Uptime;
use foc::{CurrentLoopOperation, FieldOrientedControl};
// `backend::x::Y` resolves to `peripherals::stm32g4::x::Y` on real
// hardware or `peripherals::fake::x::Y` elsewhere — see
// `esc1_discovery::backend`'s own doc comment. Used below for the
// backend-selected driver types stored in `Firmware`/`mod app`, instead
// of repeating the `#[cfg(target_arch = "arm")]` branch here too.
use esc1_discovery::backend::{adc::Adc, clock::ClockProvider, gpio::Gpio, pwm::Pwm};
use esc1_discovery::BoardPeripherals;
use peripherals::api::adc::{AdcOptions, AdcSampleBuffer, AdcTrait, AdcTriggerSource};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::gpio::GpioTrait;
use peripherals::api::pwm::{PwmOptions, PwmTrait};
use peripherals::api::quadrature::{
    QuadratureInputConfiguration, QuadratureOptions, QuadratureTrait,
};

/// ADC1 configuration.
/// The opamp channel comes first, so its conversion lands as close as
/// possible to the PWM mid-point trigger; the potentiometer second.
const OPAMP1_CHANNEL: u8 = 13;
const POTENTIOMETER_CHANNEL: u8 = 11;
/// PB12 = ADC1_IN11.
const ADC1_SEQUENCE: [u8; 2] = [OPAMP1_CHANNEL, POTENTIOMETER_CHANNEL];
static ADC1_SAMPLES: AdcSampleBuffer = AdcSampleBuffer::new();

/// ADC2 configuration.
/// The opamp channels come first (OPAMP2's internal output is ADC2 channel 16,
/// OPAMP3's is ADC2 channel 18, VBUS last. This sequence
/// is deliberately longer than ADC1's (3 channels vs. 2), so when its own
/// end-of-sequence interrupt fires, ADC1's sequence is guaranteed to be
/// already converted too.
const OPAMP2_CHANNEL: u8 = 16;
const OPAMP3_CHANNEL: u8 = 18;
const VBUS_CHANNEL: u8 = 1; // PA0 = ADC2_IN1.
const ADC2_SEQUENCE: [u8; 3] = [OPAMP2_CHANNEL, OPAMP3_CHANNEL, VBUS_CHANNEL];
static ADC2_SAMPLES: AdcSampleBuffer = AdcSampleBuffer::new();

/// PWM configuration.
const PWM_FREQUENCY_HZ: u32 = 20_000;
const PWM_CHANNELS: u8 = 3;

/// Quadrature encoder configuration (J8 header) — channel 1 carries input
/// A, channel 2 input B, no swap needed. `ENCODER_COUNTS` is a placeholder
/// matching this workspace's own test rig (see
/// `firmware/benchtest/quadrature`), not a fact about any particular motor
/// — adjust to whatever encoder is actually attached.
const QUADRATURE_INPUT_CONFIGURATION: QuadratureInputConfiguration =
    QuadratureInputConfiguration::Ch12AreInputsAB;
const ENCODER_COUNTS: u32 = 128;

/// The voltage fraction [`FieldOrientedControl`] injects into phase U
/// during [`CurrentLoopOperation::PhaseLock`].
const PHASE_LOCK_VOLTAGE_FRACTION: f32 = 0.1;

/// Push button.
const DEBOUNCE_DURATION: Duration = Duration::from_millis(20);

/// How long to average the zero-current bias per phase before leaving
/// [`ds402::State::NotReadyToSwitchOn`] — see [`FieldOrientedControl`]'s
/// own per-phase filters.
const BIAS_AVERAGE_WINDOW: Duration = Duration::from_millis(5);

/// DS402 transitions 0+1 combined: this app has no separate
/// "initialization" step of its own beyond the zero-current
/// bias-averaging window (see [`Firmware::step`]) — completing that
/// window is what "initialization complete" means here. A no-op from
/// every other state. A free function, not a `ds402::State` method:
/// `ds402::State` is defined in `common`, and this app-specific
/// transition logic isn't generically useful enough to live there too —
/// see `common::ds402`'s own doc comment.
fn complete_initialization(state: ds402::State) -> ds402::State {
    match state {
        ds402::State::NotReadyToSwitchOn => ds402::State::SwitchOnDisabled,
        other => other,
    }
}

/// This app's single "enable" gesture: a confirmed button press,
/// cascading DS402 transitions 2 (Shutdown), 3 (Switch On), and 4
/// (Enable Operation) in one step from [`ds402::State::SwitchOnDisabled`],
/// since there is no fieldbus master issuing them as discrete commands
/// here — or transition 11 (Quick Stop) from
/// [`ds402::State::OperationEnabled`], on a second press. A no-op
/// (returns `state` unchanged) from every other state — this app doesn't
/// yet drive fault handling or resuming from quick stop (transition 16).
/// See [`complete_initialization`] for why this is a free function.
fn on_confirmed_button_press(state: ds402::State) -> ds402::State {
    match state {
        ds402::State::SwitchOnDisabled => ds402::State::OperationEnabled,
        ds402::State::OperationEnabled => ds402::State::QuickStopActive,
        other => other,
    }
}

/// Everything the main loop (the DS402 state machine) does after chip
/// bring-up, one iteration at a time. Kept separate from the `app` module
/// below so it can be driven by unit tests (see the `tests` module)
/// without a real interrupt-driven scheduler — the current loop's own
/// side (the `adc_isr` RTIC task, and everything [`FieldOrientedControl`]
/// does) lives entirely in `mod app` instead.
struct Firmware {
    gpio: Gpio,
    clock_provider: ClockProvider,
    state: ds402::State,
    /// The `Uptime` [`ds402::State::NotReadyToSwitchOn`] was entered at —
    /// lazily captured (`None` means "not yet observed"), by the first
    /// [`Self::step`] call in that state, since [`Self::new`] has no
    /// `Uptime` of its own to record at construction time. Reset to
    /// `None` on leaving the state, ready to be captured again the next
    /// time it's (re-)entered.
    not_ready_since: Option<Uptime>,
    button_debouncer: Debouncer,
    /// The button debouncer's own confirmed state as of the previous
    /// [`Self::step`] call — compared against its current confirmed state
    /// each call to edge-detect a fresh confirmed press (a `false ->
    /// true` transition), so a press held across multiple `step()` calls
    /// only triggers [`on_confirmed_button_press`] once.
    button_was_active: bool,
}

impl Firmware {
    fn new(gpio: Gpio, clock_provider: ClockProvider) -> Self {
        Firmware {
            gpio,
            clock_provider,
            state: ds402::State::NotReadyToSwitchOn,
            not_ready_since: None,
            button_debouncer: Debouncer::new(DEBOUNCE_DURATION, DEBOUNCE_DURATION),
            button_was_active: false,
        }
    }

    /// One DS402 state-machine step: `last_potentiometer` is the current
    /// loop's own last-published reading (via shared state, in `mod
    /// app`); the return value, if `Some`, is what the caller should in
    /// turn publish back to the current loop.
    fn step(&mut self, last_potentiometer: UnitInterval) -> Option<CurrentLoopOperation> {
        self.clock_provider.advance_reference_point();
        let now = self.clock_provider.get_clock().now();

        match self.state {
            ds402::State::NotReadyToSwitchOn => {
                // `current_loop_operation` already starts (and stays)
                // `Open` from `init` — `FieldOrientedControl::poll`
                // averages each phase's zero-current bias on every call
                // while it is, so nothing needs commanding here, only
                // waited out.
                let since = *self.not_ready_since.get_or_insert(now);
                if now >= since + BIAS_AVERAGE_WINDOW {
                    self.not_ready_since = None;
                    self.state = complete_initialization(self.state);
                }
                None
            }
            ds402::State::SwitchOnDisabled => {
                if self.confirmed_button_press(now) {
                    self.state = on_confirmed_button_press(self.state);
                }
                None
            }
            ds402::State::OperationEnabled => {
                if self.confirmed_button_press(now) {
                    self.state = on_confirmed_button_press(self.state);
                    Some(CurrentLoopOperation::Open)
                } else {
                    Some(CurrentLoopOperation::CurrentControl(last_potentiometer))
                }
            }
            // QuickStopActive and beyond aren't driven any further yet.
            _ => None,
        }
    }

    /// Reads the button pin, debounces it, and edge-detects a freshly
    /// confirmed press — see [`Self::button_was_active`].
    fn confirmed_button_press(&mut self, now: Uptime) -> bool {
        let raw_pressed = self.gpio.get(esc1_discovery::BUTTON_PIN);
        self.button_debouncer.poll(now, raw_pressed);
        let debounced = self.button_debouncer.is_active();
        let confirmed_press = debounced && !self.button_was_active;
        self.button_was_active = debounced;
        confirmed_press
    }
}

/// Configures ADC1/ADC2/PWM1/the quadrature encoder and builds a
/// [`Firmware`] plus a [`FieldOrientedControl`] — split out of `mod
/// app`'s `init` purely so it's callable (and its resulting
/// `AdcOptions`/`PwmOptions` inspectable) from tests without going
/// through RTIC's `init` machinery.
fn bring_up(peripherals: BoardPeripherals) -> (Firmware, Adc, Adc, Pwm, FieldOrientedControl) {
    let BoardPeripherals {
        gpio,
        clock_provider,
        dma,
        mut adc1,
        mut adc2,
        mut pwm1,
        mut quadrature,
        math_coprocessor,
        ..
    } = peripherals;

    adc1.open(
        AdcOptions::new(
            &ADC1_SEQUENCE,
            &ADC1_SAMPLES,
            AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
        ),
        &dma,
    );
    adc2.open(
        AdcOptions::new(
            &ADC2_SEQUENCE,
            &ADC2_SAMPLES,
            AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
        ),
        &dma,
    );
    // Both `open()` calls above implicitly enable their own end-of-sequence
    // interrupt (`AdcTriggerSource` isn't `Software` — see
    // `peripherals::stm32g4::adc::Adc::open`), so the shared `ADC1_2` NVIC
    // line fires twice per TRGO2 edge: once when adc1's shorter, 2-channel
    // sequence completes, again when adc2's longer, 3-channel sequence
    // completes right after. `adc_isr` (below) only acts on the second.

    // `mid_point_trigger: true` drives TIM1's TRGO2 wire, which both ADCs
    // above are triggered from.
    pwm1.open(PwmOptions::new(
        PWM_FREQUENCY_HZ,
        PWM_CHANNELS,
        esc1_discovery::TIM1_DEAD_TIME,
        true,
    ));

    // `esc1_discovery::initialize` already claimed TIM4 and constructed
    // `quadrature` — see `BoardPeripherals::quadrature`'s doc comment for
    // why it's this firmware, not the board crate, that `open()`s it.
    quadrature.open(QuadratureOptions {
        input_configuration: QUADRATURE_INPUT_CONFIGURATION,
        encoder_counts: ENCODER_COUNTS,
    });

    let foc = FieldOrientedControl::new(
        quadrature,
        math_coprocessor,
        SymmetricUnitInterval::from(PHASE_LOCK_VOLTAGE_FRACTION),
    );

    (Firmware::new(gpio, clock_provider), adc1, adc2, pwm1, foc)
}

/// `stm32-metapac` doesn't generate a `NVIC_PRIO_BITS` const the way a
/// real svd2rust PAC does — `#[rtic::app]`'s `device` argument needs one
/// under exactly that name (to compute its priority-masking limits). Its
/// glob-imported `Interrupt` enum is also what `#[task(binds = ADC1_2,
/// ...)]` resolves against, below.
#[cfg(target_arch = "arm")]
mod rtic_device {
    pub use esc1_discovery::RTIC_PRIORITY_BITS as NVIC_PRIO_BITS;
    pub use stm32_metapac::*;
}

/// The application's RTIC task graph: `idle` is the DS402 main loop
/// (driving [`Firmware::step`]); `adc_isr` is the current loop, driving
/// [`FieldOrientedControl::poll`] — the first hardware-interrupt-bound
/// task and the first real `#[shared]`/priority-ceiling usage in this
/// workspace. Chip bring-up always goes through
/// `esc1_discovery::initialize()`, not RTIC's own PAC-peripherals claim,
/// which stays disabled here.
#[rtic_shim::app(device = crate::rtic_device)]
mod app {
    use super::{
        bring_up, Adc, CurrentLoopOperation, FieldOrientedControl, Firmware, Pwm, UnitInterval,
    };
    // `#[allow(unused_imports)]`: on real RTIC, the generated `.lock()`
    // resolves without this trait explicitly in scope (its accessor types
    // apparently already bring it in) — kept explicit anyway, since
    // relying on that being true across RTIC versions would be fragile,
    // and the host/fake backend genuinely does need it in scope.
    #[allow(unused_imports)]
    use rtic_shim::Mutex;

    #[shared]
    pub(crate) struct Shared {
        pub(crate) current_loop_operation: CurrentLoopOperation,
        pub(crate) last_potentiometer: UnitInterval,
    }

    // `pub(crate)`: `#[cfg(test)] mod tests` (a sibling of this module, not
    // a descendant) needs to reach these fields directly.
    #[local]
    pub(crate) struct Local {
        pub(crate) firmware: Firmware,
        pub(crate) adc1: Adc,
        pub(crate) adc2: Adc,
        pub(crate) pwm1: Pwm,
        pub(crate) foc: FieldOrientedControl,
        #[cfg(not(target_arch = "arm"))]
        pub(crate) fakes: esc1_discovery::BoardFakePeripherals,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        #[cfg(not(test))]
        defmt::info!("init");

        // RTIC's own `init` prologue already steals `cortex_m::Peripherals`
        // (that's `cx.core`) before calling this, so it's passed through
        // here rather than taken again — taking the core peripherals can
        // only succeed once.
        #[cfg(target_arch = "arm")]
        let peripherals = esc1_discovery::initialize(_cx.core);
        #[cfg(not(target_arch = "arm"))]
        let peripherals = esc1_discovery::initialize(());

        #[cfg(not(target_arch = "arm"))]
        let fakes = peripherals.fakes.clone(); // cheap: Rc-backed

        let (firmware, adc1, adc2, pwm1, foc) = bring_up(peripherals);

        (
            Shared {
                current_loop_operation: CurrentLoopOperation::Open,
                last_potentiometer: UnitInterval::new(0),
            },
            Local {
                firmware,
                adc1,
                adc2,
                pwm1,
                foc,
                #[cfg(not(target_arch = "arm"))]
                fakes,
            },
        )
    }

    // `mut`: only the fake (host) `idle::Context` needs it, since on that
    // backend it owns `Local`/`Shared` by value rather than borrowing them.
    //
    // `#[idle_step]`, not `#[idle]`: this describes a single iteration —
    // `rtic_real` wraps it in the `loop`/`-> !` real RTIC's own `#[idle]`
    // needs, but on the host/fake backend (`rtic_fake`) it stays a single,
    // directly-callable step instead, letting unit tests (see `mod tests`
    // below) invoke `app::idle(cx)` repeatedly to simulate the main loop
    // one iteration at a time. See `macros/rtic_real`/`macros/rtic_fake`.
    #[allow(unused_mut)]
    #[idle_step(
        local = [firmware],
        shared = [current_loop_operation, last_potentiometer]
    )]
    fn idle(mut cx: idle::Context) {
        let last_potentiometer = cx.shared.last_potentiometer.lock(|p| *p);
        if let Some(operation) = cx.local.firmware.step(last_potentiometer) {
            cx.shared.current_loop_operation.lock(|o| *o = operation);
        }
    }

    /// The current loop: once both ADCs have converted this cycle (see
    /// `bring_up`'s doc comment — this task runs twice per TRGO2 edge, but
    /// only the second entry, once `adc2`'s longer sequence has also
    /// completed, has anything fresh to act on), reads every phase's
    /// current and the bus voltage, publishes the potentiometer reading
    /// for the main loop, and drives [`FieldOrientedControl::poll`] —
    /// applying its output to the PWM channels, or zeroing them if it
    /// returns `None`. On the fake backend, the callback registry (see
    /// `peripherals::fake::callback_registry`) drives this automatically
    /// once wired up — see `mod tests`' `Harness`-based tests.
    ///
    /// No early return for the "adc2 not done yet" case (unlike an
    /// earlier version of this function): on the fake backend, this
    /// function's declared return type is rewritten to hand `Context`
    /// back to its caller (see `macros::rtic_fake`), which a bare
    /// `return;` can't satisfy — wrapping the rest of the body in an `if`
    /// instead avoids needing the macro to rewrite early returns too.
    #[allow(unused_mut)]
    #[task(
        binds = ADC1_2,
        priority = 1,
        local = [adc1, adc2, pwm1, foc],
        shared = [current_loop_operation, last_potentiometer]
    )]
    fn adc_isr(mut cx: adc_isr::Context) {
        use peripherals::api::adc::AdcTrait;
        use peripherals::api::pwm::PwmTrait;

        cx.local.adc1.try_retrieve_result();
        if cx.local.adc2.try_retrieve_result() {
            let u = cx.local.adc1.get_sample(0); // OPAMP1
            let potentiometer = cx.local.adc1.get_sample(1);
            let v = cx.local.adc2.get_sample(0); // OPAMP2
            let w = cx.local.adc2.get_sample(1); // OPAMP3
            let vbus = cx.local.adc2.get_sample(2);

            cx.shared.last_potentiometer.lock(|p| *p = potentiometer);
            cx.local.foc.set_bus_voltage(vbus);

            let operation = cx.shared.current_loop_operation.lock(|o| *o);
            // Command 0% duty cycle if the FOC step() function returns None.
            // TODO: Consider adding enable/disable methods in the PWM API if the PWM can
            // be enabled through a hardware (BREAK) event.
            let duty_cycles = cx.local.foc.poll(u, v, w, operation).unwrap_or((
                UnitInterval::new(0),
                UnitInterval::new(0),
                UnitInterval::new(0),
            ));
            cx.local.pwm1.set_duty_cycle(0, duty_cycles.0);
            cx.local.pwm1.set_duty_cycle(1, duty_cycles.1);
            cx.local.pwm1.set_duty_cycle(2, duty_cycles.2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a `UnitInterval` that round-trips exactly through the fake
    /// ADC's 16-bit-wide sample storage — a raw top-16-bits value, matching
    /// what `FakeAdc::set_sample`/`AdcTrait::get_sample` actually keep.
    fn sample(top_16_bits: u16) -> UnitInterval {
        UnitInterval::new((top_16_bits as u32) << 16)
    }

    /// One `app::idle` call — checking `Local`/`Shared` out of `harness`,
    /// calling `app::idle`, checking them back in — the harness-based
    /// equivalent of the pre-`Harness` `cx = app::idle(cx)` pattern (see
    /// `mod app`'s own `idle` doc comment for what `#[idle_step]` means).
    fn step_idle(harness: &app::Harness) {
        let (local, shared) = harness.checkout();
        let cx = app::idle(app::idle::Context { local, shared });
        harness.checkin(cx.local, cx.shared);
    }

    /// Reads `Local`/`Shared` from `harness` for `f` to inspect, then
    /// checks them back in — for a test that only needs to peek, without
    /// advancing anything.
    fn with_local_and_shared<R>(
        harness: &app::Harness,
        f: impl FnOnce(&app::Local, &app::Shared) -> R,
    ) -> R {
        let (local, shared) = harness.checkout();
        let result = f(&local, &shared);
        harness.checkin(local, shared);
        result
    }

    /// Like [`with_local_and_shared`], but `f` can also mutate `Shared` —
    /// for a test injecting a value (e.g. a potentiometer reading) the
    /// same way directly writing `cx.shared.field = ...` used to, before
    /// `Harness` existed.
    fn with_shared_mut(harness: &app::Harness, f: impl FnOnce(&mut app::Shared)) {
        let (local, mut shared) = harness.checkout();
        f(&mut shared);
        harness.checkin(local, shared);
    }

    /// Calls [`step_idle`] repeatedly until `predicate` (given a fresh
    /// look at `Local`/`Shared` after each step) returns `true`. Real
    /// time only advances a couple of simulated MCU-frequency ticks per
    /// call (see `peripherals::fake::clock`), so crossing even
    /// `BIAS_AVERAGE_WINDOW`'s or `DEBOUNCE_DURATION`'s few milliseconds
    /// takes many calls — capped well above what either should ever
    /// need, so a regression that breaks a transition fails fast instead
    /// of hanging.
    fn advance_idle_until(
        harness: &app::Harness,
        mut predicate: impl FnMut(&app::Local, &app::Shared) -> bool,
    ) {
        for _ in 0..10_000_000 {
            step_idle(harness);
            if with_local_and_shared(harness, |local, shared| predicate(local, shared)) {
                return;
            }
        }
        panic!("predicate was never satisfied within the iteration cap");
    }

    const ALL_DS402_STATES: [ds402::State; 8] = [
        ds402::State::NotReadyToSwitchOn,
        ds402::State::SwitchOnDisabled,
        ds402::State::ReadyToSwitchOn,
        ds402::State::SwitchedOn,
        ds402::State::OperationEnabled,
        ds402::State::QuickStopActive,
        ds402::State::FaultReactionActive,
        ds402::State::Fault,
    ];

    #[test]
    fn complete_initialization_only_transitions_out_of_not_ready() {
        for state in ALL_DS402_STATES {
            let expected = if state == ds402::State::NotReadyToSwitchOn {
                ds402::State::SwitchOnDisabled
            } else {
                state
            };
            assert_eq!(complete_initialization(state), expected, "{state:?}");
        }
    }

    #[test]
    fn confirmed_button_press_cascades_switch_on_disabled_to_operation_enabled() {
        assert_eq!(
            on_confirmed_button_press(ds402::State::SwitchOnDisabled),
            ds402::State::OperationEnabled
        );
    }

    #[test]
    fn confirmed_button_press_from_operation_enabled_enters_quick_stop_active() {
        assert_eq!(
            on_confirmed_button_press(ds402::State::OperationEnabled),
            ds402::State::QuickStopActive
        );
    }

    #[test]
    fn confirmed_button_press_is_a_no_op_from_every_other_state() {
        for state in ALL_DS402_STATES {
            if matches!(
                state,
                ds402::State::SwitchOnDisabled | ds402::State::OperationEnabled
            ) {
                continue;
            }
            assert_eq!(on_confirmed_button_press(state), state, "{state:?}");
        }
    }

    #[test]
    fn opens_adc1_adc2_and_pwm_with_the_expected_options() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed

        assert_eq!(
            fakes.adc1.options(),
            Some(AdcOptions::new(
                &ADC1_SEQUENCE,
                &ADC1_SAMPLES,
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ))
        );
        assert_eq!(
            fakes.adc2.options(),
            Some(AdcOptions::new(
                &ADC2_SEQUENCE,
                &ADC2_SAMPLES,
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ))
        );
        assert_eq!(
            fakes.pwm1.options(),
            Some(PwmOptions::new(
                PWM_FREQUENCY_HZ,
                PWM_CHANNELS,
                esc1_discovery::TIM1_DEAD_TIME,
                true,
            ))
        );
    }

    #[test]
    fn adc_isr_runs_after_open_without_a_prior_trigger_call() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed

        // Both ADCs are already armed by open() (a hardware trigger
        // source) — no trigger() call needed, see
        // peripherals::stm32g4::adc's AdcTriggerSource docs. Simulate the
        // shared TRGO2 wire asserting on both.
        fakes.adc1.set_sample(0, sample(u16::MAX)); // opamp1: full-scale
        fakes.adc1.assert_trigger_wire();
        fakes.adc2.assert_trigger_wire();

        // Smoke test only, calling the generated function directly (its
        // `Context` is dropped immediately — there's no way to observe
        // what got published from outside this call) — `foc.rs`'s own
        // tests are what actually exercise `FieldOrientedControl`'s
        // computation; `adc_isr_fires_automatically_from_the_pwm_trigger_out`
        // below exercises the registry-driven path this smoke test
        // bypasses.
        app::adc_isr(app::adc_isr::Context { local, shared });
    }

    #[test]
    fn adc_isr_fires_automatically_from_the_pwm_trigger_out() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        // Nothing here calls `app::adc_isr` directly — only PWM1's
        // simulated periodic trigger-out (see
        // `peripherals::fake::pwm::Pwm::set_callback_registry`), relayed
        // through ADC1's own simulated conversion delay (see
        // `peripherals::fake::adc::Adc::set_callback_registry`), should
        // ever cause the potentiometer reading below to get published.
        let pot = sample(0x1234);
        fakes.adc1.set_sample(1, pot);

        // Comfortably past PWM1's first trigger-out edge (half a period,
        // 25us at PWM_FREQUENCY_HZ = 20kHz) plus ADC1's simulated
        // conversion time.
        fakes
            .callback_registry
            .advance_clock_by(Duration::from_micros(100));

        assert_eq!(
            with_local_and_shared(&harness, |_local, shared| shared.last_potentiometer),
            pot
        );
    }

    #[test]
    fn firmware_leaves_not_ready_to_switch_on_after_the_bias_average_window() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });

        with_local_and_shared(&harness, |local, shared| {
            assert_eq!(local.firmware.state, ds402::State::SwitchOnDisabled);
            // Never explicitly changed away from `init`'s own default —
            // see `Firmware::step`'s `NotReadyToSwitchOn` arm.
            assert_eq!(shared.current_loop_operation, CurrentLoopOperation::Open);
        });
    }

    #[test]
    fn firmware_waits_for_a_confirmed_button_press_in_switch_on_disabled() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });

        // Not pressed: stays in SwitchOnDisabled, however many steps pass.
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, false);
        for _ in 0..1000 {
            step_idle(&harness);
        }
        assert_eq!(
            with_local_and_shared(&harness, |local, _shared| local.firmware.state),
            ds402::State::SwitchOnDisabled
        );

        // Confirmed press: cascades to OperationEnabled.
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::OperationEnabled
        });
        assert_eq!(
            with_local_and_shared(&harness, |local, _shared| local.firmware.state),
            ds402::State::OperationEnabled
        );
    }

    #[test]
    fn firmware_commands_current_control_from_the_potentiometer_while_operation_enabled() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::OperationEnabled
        });
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, false);

        let pot = UnitInterval::new(0x4000_0000);
        with_shared_mut(&harness, |shared| shared.last_potentiometer = pot);
        step_idle(&harness);

        assert_eq!(
            with_local_and_shared(&harness, |_local, shared| shared.current_loop_operation),
            CurrentLoopOperation::CurrentControl(pot)
        );
    }

    #[test]
    fn firmware_enters_quick_stop_active_on_a_second_confirmed_press() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::OperationEnabled
        });

        // Between two presses, the edge detector needs to observe a
        // confirmed release before a fresh press can be confirmed again
        // — see `Firmware::button_was_active`'s doc comment.
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, false);
        advance_idle_until(&harness, |local, _shared| !local.firmware.button_was_active);

        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::QuickStopActive
        });

        with_local_and_shared(&harness, |local, shared| {
            assert_eq!(local.firmware.state, ds402::State::QuickStopActive);
            assert_eq!(shared.current_loop_operation, CurrentLoopOperation::Open);
        });
    }
}
