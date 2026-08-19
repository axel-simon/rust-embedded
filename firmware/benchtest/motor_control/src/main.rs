#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::debounce::Debouncer;
use common::ds402;
use common::duration::Duration;
use common::filter::LowPassFilter;
use common::unit_interval::{SymmetricUnitInterval, UnitInterval};
use common::uptime::Uptime;
use drivers::foc::{CurrentLoopOperation, FieldOrientedControl};
// `backend::x::Y` resolves to `peripherals::stm32g4::x::Y` on real
// hardware or `peripherals::fake::x::Y` elsewhere — see
// `esc1_discovery::backend`'s own doc comment. Used below for the
// backend-selected driver types stored in `Firmware`/`mod app`, instead
// of repeating the `#[cfg(target_arch = "arm")]` branch here too.
use esc1_discovery::backend::{
    adc::Adc, clock::ClockProvider, gpio::Gpio, math_coprocessor::MathCoprocessor, pwm::Pwm,
    quadrature::Quadrature,
};
use esc1_discovery::BoardPeripherals;
use peripherals::api::adc::{AdcOptions, AdcSampleBuffer, AdcTrait, AdcTriggerSource};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::pwm::{PwmOptions, PwmTrait};
use peripherals::api::quadrature::{
    QuadratureInputConfiguration, QuadratureOptions, QuadratureTrait,
};

/// [`FieldOrientedControl`], instantiated against this board's own
/// backend-selected math-coprocessor driver (real hardware or fake,
/// depending on target — see `esc1_discovery::backend`) — `drivers` itself
/// only depends on `peripherals::api`, so this app is what pins it to a
/// concrete board. The quadrature encoder is *not* one of
/// `FieldOrientedControl`'s type parameters: this app owns it directly (see
/// `mod app`'s own `Local::quadrature`) and reads its position itself, once
/// per current-loop cycle, passing it into [`FieldOrientedControl::poll`]
/// rather than handing the driver over.
type Foc = FieldOrientedControl<MathCoprocessor>;

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

/// The attached motor's pole-pair count (see
/// [`FieldOrientedControl::new`]'s doc comment for what this scales) —
/// this test rig's motor is a Anaheim Automation BLWR111D-24V-10000,
/// documented ("Winding Type: Star, 4 Poles" — L010234, the BLWR11
/// series product sheet) as 4 poles, i.e. 2 pole pairs. Adjust if a
/// different motor is attached.
const POLE_PAIRS: u32 = 2;

/// Push button.
const DEBOUNCE_DURATION: Duration = Duration::from_millis(20);

/// How long to average the zero-current bias per phase before leaving
/// [`ds402::State::NotReadyToSwitchOn`] — see [`FieldOrientedControl`]'s
/// own per-phase filters.
const BIAS_AVERAGE_WINDOW: Duration = Duration::from_millis(5);

/// How long to hold [`CurrentLoopOperation::PhaseLock`] on entry to
/// [`ds402::State::SwitchedOn`] before treating the rotor as settled into
/// alignment and accepting a button press onward to
/// [`ds402::State::OperationEnabled`] — the injected phase-lock voltage
/// needs real time to pull the rotor into place; sampling the quadrature
/// encoder as "zero" before it's actually gotten there would misalign
/// every angle `FieldOrientedControl` computes afterward. Unmeasured
/// placeholder (like [`BIAS_AVERAGE_WINDOW`]) — tune against the
/// attached motor/load's actual settling time.
const PHASE_LOCK_SETTLE_WINDOW: Duration = Duration::from_millis(50);

/// Nominal time constant (in [`Firmware::step`] calls, i.e. main-loop
/// iterations — see [`LowPassFilter`]'s own doc comment for what the
/// unit means) the bus-voltage filter settles to. The main loop has no
/// fixed real-time rate the way the current loop does (it's RTIC's
/// `idle` task, running flat-out whenever nothing higher-priority is
/// pending), so unlike [`BIAS_AVERAGE_WINDOW`]/[`PHASE_LOCK_SETTLE_WINDOW`]
/// this can't be expressed as a real-time duration at all — an
/// unmeasured placeholder, like those.
const BUS_VOLTAGE_FILTER_TIME_CONSTANT: f32 = 1000.0;

/// How often [`Firmware::step`] surfaces the rotor's physical position
/// back out for `mod app`'s `idle` to log — see [`Firmware::step`]'s own
/// doc comment for why the rate limiting happens here rather than just
/// logging on every main-loop iteration.
const PHYSICAL_POSITION_LOG_INTERVAL: Duration = Duration::from_seconds(1);

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

// TEMPORARY diagnostic instrumentation — see `mod app`'s `Local::diag_counter`
// doc comment. A method (not a bare `u32`), so `cx.local.diag_counter.tick()`
// works identically whether `Local` fields are owned by value (fake backend)
// or borrowed `&mut` (real RTIC) — same reason every other mutated `Local`
// field in this app (e.g. `firmware`) is only ever accessed through methods.
#[derive(Default)]
struct DiagCounter(u32);

impl DiagCounter {
    fn tick(&mut self) -> u32 {
        self.0 = self.0.wrapping_add(1);
        self.0
    }
}

/// A fault the main loop has observed. `pub`, not private, for the same
/// reason as [`CurrentLoopOperation`]: it's a `#[shared]` field type
/// (`pending_fault`, in `mod app`), and real RTIC's generated
/// per-resource proxy type for that field is itself `pub`.
///
/// Ordered by severity via its derived [`Ord`] (`None` first, so it's
/// the least severe) — see [`Firmware::report_fault`], which relies on
/// that ordering to always keep the worst fault observed so far.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Fault {
    /// No fault observed (yet).
    None,
    /// `adc_isr` expected ADC1's conversion to have already finished by
    /// the time ADC2's (deliberately longer, see `bring_up`'s doc
    /// comment) sequence completes, but it hadn't.
    AdcConversionIncomplete,
}

/// This app's single "enable" gesture: a confirmed button press,
/// cascading DS402 transitions 2 (Shutdown) and 3 (Switch On) in one step
/// from [`ds402::State::SwitchOnDisabled`] to [`ds402::State::SwitchedOn`]
/// (skipping over [`ds402::State::ReadyToSwitchOn`] as an observable
/// state — this app has no fieldbus master issuing them as discrete
/// commands) — or transition 11 (Quick Stop) from
/// [`ds402::State::OperationEnabled`]. A no-op (returns `state`
/// unchanged) from every other state — this app doesn't yet drive fault
/// handling or resuming from quick stop (transition 16).
///
/// Transition 4 (Enable Operation, [`ds402::State::SwitchedOn`] ->
/// [`ds402::State::OperationEnabled`]) is deliberately *not* handled
/// here: unlike every other transition in this function, it also needs
/// [`PHASE_LOCK_SETTLE_WINDOW`] to have elapsed, so [`Firmware::step`]'s
/// own `SwitchedOn` arm drives it directly instead. See
/// [`complete_initialization`] for why this is a free function.
fn on_confirmed_button_press(state: ds402::State) -> ds402::State {
    match state {
        ds402::State::SwitchOnDisabled => ds402::State::SwitchedOn,
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
    clock_provider: ClockProvider,
    state: ds402::State,
    /// The `Uptime` [`ds402::State::NotReadyToSwitchOn`] was entered at —
    /// lazily captured (`None` means "not yet observed"), by the first
    /// [`Self::step`] call in that state, since [`Self::new`] has no
    /// `Uptime` of its own to record at construction time. Reset to
    /// `None` on leaving the state, ready to be captured again the next
    /// time it's (re-)entered.
    not_ready_since: Option<Uptime>,
    /// The `Uptime` [`ds402::State::SwitchedOn`] was (most recently)
    /// entered at — same lazy-capture/reset convention as
    /// [`Self::not_ready_since`], gating [`PHASE_LOCK_SETTLE_WINDOW`]
    /// instead of [`BIAS_AVERAGE_WINDOW`]. Also reset on leaving
    /// [`ds402::State::QuickStopActive`], so a fresh window runs the next
    /// time [`ds402::State::SwitchedOn`] is re-entered rather than
    /// reusing however much of the old one happened to already elapse.
    phase_lock_since: Option<Uptime>,
    button_debouncer: Debouncer,
    /// The button debouncer's own confirmed state as of the previous
    /// [`Self::step`] call — compared against its current confirmed state
    /// each call to edge-detect a fresh confirmed press (a `false ->
    /// true` transition), so a press held across multiple `step()` calls
    /// only triggers [`on_confirmed_button_press`] once.
    button_was_active: bool,
    /// The most severe [`Fault`] observed so far — see [`Self::report_fault`].
    fault: Fault,
    /// Low-pass filters the raw VBUS reading `mod app`'s `adc_isr`
    /// publishes (see [`Self::step`]) — done here, in the slow main
    /// loop, rather than every current-loop cycle: bus voltage changes
    /// slowly, and filtering it 20000 times a second (`adc_isr`'s own
    /// rate) would buy nothing over filtering it once per main-loop
    /// iteration instead.
    bus_voltage_filter: LowPassFilter,
    /// The `Uptime` [`Self::step`] last surfaced the physical position
    /// for logging — `None` means "never yet", which is what makes the
    /// very first [`Self::step`] call always log immediately rather than
    /// waiting a full [`PHYSICAL_POSITION_LOG_INTERVAL`] from
    /// construction first.
    last_logged_physical_position_at: Option<Uptime>,
}

impl Firmware {
    fn new(clock_provider: ClockProvider) -> Self {
        Firmware {
            clock_provider,
            state: ds402::State::NotReadyToSwitchOn,
            not_ready_since: None,
            phase_lock_since: None,
            button_debouncer: Debouncer::new(DEBOUNCE_DURATION, DEBOUNCE_DURATION),
            button_was_active: false,
            fault: Fault::None,
            bus_voltage_filter: LowPassFilter::new(BUS_VOLTAGE_FILTER_TIME_CONSTANT),
            last_logged_physical_position_at: None,
        }
    }

    /// One DS402 state-machine step: `last_potentiometer`/`last_bus_voltage`/
    /// `last_physical_position` are the current loop's own last-published
    /// readings (via shared state, in `mod app`); `raw_button_pressed` is
    /// the button pin's own raw reading, read by `idle` through the
    /// `Shared::gpio` `Mutex` (not owned by `Firmware` itself — see
    /// `Shared::gpio`'s doc comment for why `Gpio` isn't just handed to
    /// `Firmware` directly the way it used to be) — same "read once per
    /// call by `idle`, regardless of which state actually uses it"
    /// convention as the other three. Returns whatever the caller should
    /// in turn publish back to the current loop or log — a
    /// `CurrentLoopOperation`, if this step commands a new one; the
    /// bus-voltage reading, freshly low-pass-filtered (see
    /// [`Self::bus_voltage_filter`]), published on every call regardless
    /// of state; and `last_physical_position` itself, echoed back but
    /// only every [`PHYSICAL_POSITION_LOG_INTERVAL`] — rate-limited here,
    /// in the main loop, rather than in `mod app`'s `idle` directly,
    /// since deciding "has enough time passed" needs the same `Uptime`
    /// this function already computes for its own DS402 timing windows.
    fn step(
        &mut self,
        last_potentiometer: UnitInterval,
        last_bus_voltage: UnitInterval,
        last_physical_position: UnitInterval,
        raw_button_pressed: bool,
    ) -> (
        Option<CurrentLoopOperation>,
        UnitInterval,
        Option<UnitInterval>,
    ) {
        self.clock_provider.advance_reference_point();
        let now = self.clock_provider.get_clock().now();

        self.bus_voltage_filter.poll(f32::from(last_bus_voltage));
        let filtered_bus_voltage = UnitInterval::from(self.bus_voltage_filter.output());

        let due = match self.last_logged_physical_position_at {
            None => true,
            Some(last) => now >= last + PHYSICAL_POSITION_LOG_INTERVAL,
        };
        let physical_position_to_log = due.then(|| {
            self.last_logged_physical_position_at = Some(now);
            last_physical_position
        });

        let operation = match self.state {
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
                if self.confirmed_button_press(now, raw_button_pressed) {
                    self.state = on_confirmed_button_press(self.state);
                }
                None
            }
            ds402::State::SwitchedOn => {
                // Phase-locks for as long as this state is held — see
                // `CurrentLoopOperation::PhaseLock`'s own doc comment for
                // why holding it (not just commanding it once) matters:
                // the rotor needs real time to actually rotate into
                // alignment, and the zero position `FieldOrientedControl`
                // captures keeps tracking reality for as long as this
                // stays commanded. A confirmed press only advances to
                // `OperationEnabled` once `PHASE_LOCK_SETTLE_WINDOW` has
                // had time to elapse -- `confirmed_button_press` is still
                // called unconditionally either way, so a press during
                // that window still edge-detects normally (and is simply
                // not acted on) rather than leaving `button_was_active`
                // stale once the window does elapse.
                let since = *self.phase_lock_since.get_or_insert(now);
                let settled = now >= since + PHASE_LOCK_SETTLE_WINDOW;
                let confirmed_press = self.confirmed_button_press(now, raw_button_pressed);
                if settled && confirmed_press {
                    self.phase_lock_since = None;
                    self.state = ds402::State::OperationEnabled;
                }
                Some(CurrentLoopOperation::PhaseLock)
            }
            ds402::State::OperationEnabled => {
                if self.confirmed_button_press(now, raw_button_pressed) {
                    self.state = on_confirmed_button_press(self.state);
                    Some(CurrentLoopOperation::Open)
                } else {
                    Some(CurrentLoopOperation::CurrentControl(
                        last_potentiometer.expand(),
                    ))
                }
            }
            ds402::State::QuickStopActive => {
                // Clears state and returns to `SwitchOnDisabled`
                // unconditionally -- not gated on a further button press,
                // unlike every other transition here: this app collapses
                // fieldbus-issued DS402 commands into automatic app-level
                // behavior throughout (see e.g. `on_confirmed_button_press`'s
                // own doc comment), and there's no further command this
                // app would wait for once a quick stop has been
                // commanded. `phase_lock_since` is reset so a fresh
                // settle window runs the next time `SwitchedOn` is
                // re-entered, rather than reusing however much of the
                // old one happened to already elapse.
                self.phase_lock_since = None;
                self.state = ds402::State::SwitchOnDisabled;
                Some(CurrentLoopOperation::Open)
            }
            // ReadyToSwitchOn is skipped over as an observable state (see
            // `on_confirmed_button_press`'s doc comment); fault handling
            // isn't wired up yet.
            _ => None,
        };
        (operation, filtered_bus_voltage, physical_position_to_log)
    }

    /// Debounces the button's raw pin reading (read by `mod app`'s `idle`
    /// — see [`Self::step`]'s doc comment for why, mirroring
    /// `last_potentiometer`/etc.) and edge-detects a freshly confirmed
    /// press — see [`Self::button_was_active`].
    fn confirmed_button_press(&mut self, now: Uptime, raw_pressed: bool) -> bool {
        self.button_debouncer.poll(now, raw_pressed);
        let debounced = self.button_debouncer.is_active();
        let confirmed_press = debounced && !self.button_was_active;
        self.button_was_active = debounced;
        confirmed_press
    }

    /// Records a newly observed fault: [`Self::fault`] becomes whichever
    /// of it and the previous fault is more severe (see [`Fault`]'s
    /// derived [`Ord`]), so a fault already recorded is never downgraded
    /// by a later, less severe report.
    fn report_fault(&mut self, fault: Fault) {
        self.fault = self.fault.max(fault);
    }
}

/// Configures ADC1/ADC2/PWM1/the quadrature encoder and builds a
/// [`Firmware`] plus a [`FieldOrientedControl`] — split out of `mod
/// app`'s `init` purely so it's callable (and its resulting
/// `AdcOptions`/`PwmOptions` inspectable) from tests without going
/// through RTIC's `init` machinery. The quadrature encoder is returned
/// alongside, rather than being handed into `FieldOrientedControl`
/// itself — see `Foc`'s own doc comment for why. `Gpio` is returned too
/// — not handed to [`Firmware::new`] (unlike every other peripheral
/// here): both `idle` (reading the button) and `adc_isr` (driving
/// [`esc1_discovery::CAN_SHUTDOWN_PIN`]) need it, so it becomes `mod
/// app`'s `Shared::gpio` instead, the one already-established RTIC
/// mechanism for two tasks safely sharing a single `&mut self`-requiring
/// resource — see `Shared::gpio`'s own doc comment.
fn bring_up(peripherals: BoardPeripherals) -> (Firmware, Adc, Adc, Pwm, Foc, Quadrature, Gpio) {
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

    // TRGO2 (`CNT = ARR`, the counter's peak), not TRGO (`CNT = 0`, the
    // update event): with `CCPx`/`CCNPx` non-inverted and `PWM_MODE1` on
    // channels 1-3, each channel's *high*-side output is active whenever
    // `CNT < CCRx` — the region surrounding `CNT = 0` — so its
    // complementary (*low*-side) output, with dead time inserted, is
    // active in the opposite region, surrounding `CNT = ARR`. This
    // board's current-shunt resistors sit in the low-side leg of each
    // half-bridge (see `esc1_discovery`), so the low-side FETs need to be
    // on — and past dead time — for a shunt reading to reflect real
    // phase current: that's the peak, not the trough. `mid_point_trigger:
    // true` below is this option's actual enable step (`CR2.MMS2` and
    // `CCR5`/`CC5E` on TIM1's channel 5, which has no physical pin of its
    // own — see `PwmOptions::mid_point_trigger`'s own doc comment).
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
        math_coprocessor,
        SymmetricUnitInterval::from(PHASE_LOCK_VOLTAGE_FRACTION),
        POLE_PAIRS,
    );

    (
        Firmware::new(clock_provider),
        adc1,
        adc2,
        pwm1,
        foc,
        quadrature,
        gpio,
    )
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
        bring_up, Adc, CurrentLoopOperation, DiagCounter, Fault, Firmware, Foc, Gpio, Pwm,
        Quadrature, UnitInterval,
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
        /// Set by `adc_isr` when it observes a fault, via the same
        /// escalate-don't-overwrite merge as `Firmware::report_fault`
        /// (so a fault it reports isn't lost if `idle` hasn't drained
        /// this yet by the time `adc_isr` — which runs far more often
        /// than `idle` — fires again); drained into `Firmware::fault`,
        /// and reset back to `Fault::None`, at the top of every `idle`
        /// step.
        pub(crate) pending_fault: Fault,
        /// The raw VBUS reading `adc_isr` published this cycle — read
        /// (and low-pass filtered) by `idle`, which publishes the
        /// filtered result back via `Self::filtered_bus_voltage`. Same
        /// `adc_isr` -> `idle` direction as `Self::last_potentiometer`.
        pub(crate) last_bus_voltage: UnitInterval,
        /// `idle`'s own filtered `Self::last_bus_voltage` (see
        /// `Firmware::step`), consumed by `adc_isr` via
        /// `FieldOrientedControl::set_bus_voltage`. Same `idle` ->
        /// `adc_isr` direction as `Self::current_loop_operation`.
        pub(crate) filtered_bus_voltage: UnitInterval,
        /// The rotor's physical (mechanical) position, as read by
        /// `adc_isr` from `Local::quadrature` and passed into
        /// `FieldOrientedControl::poll` this cycle — published here too
        /// so `idle` can log it (rate-limited — see `Firmware::step`).
        /// Same `adc_isr` -> `idle` direction as `Self::last_potentiometer`/
        /// `Self::last_bus_voltage`.
        pub(crate) last_physical_position: UnitInterval,
        /// The single `Gpio` `bring_up` constructs, shared between `idle`
        /// (reading the button pin, for [`Firmware::step`]'s
        /// `raw_button_pressed` parameter) and `adc_isr` (driving
        /// [`esc1_discovery::CAN_SHUTDOWN_PIN`] high/low around its own
        /// body — a TEMPORARY diagnostic, see `Local::diag_counter`'s
        /// doc comment) — not a second, independently-owned or cloned
        /// handle: `GpioTrait::configure` requires `&mut self`, so
        /// letting each task hold its own owned `Gpio` would let both
        /// independently obtain an exclusive `&mut` and race on the same
        /// MMIO registers if `configure()` were ever called through
        /// both, which is exactly why `Gpio` is deliberately not `Clone`
        /// (see its own doc comment).
        ///
        /// Not `Local` to either task either: RTIC has no mechanism for
        /// two different tasks' `local = [...]` to name the same
        /// resource (it's a compile error — "used by multiple tasks").
        /// It's `Shared`, but every task that lists it does so as
        /// `shared = [&gpio]` (note the `&`) rather than a bare `gpio` —
        /// RTIC's "shared access" resource mode, for a resource no task
        /// ever needs `&mut` through. That hands out a plain `&Gpio`
        /// directly (no `Mutex::lock` at all, on any backend, at any
        /// priority) instead of the usual exclusive-access proxy —
        /// correct here specifically because [`GpioTrait::set`]/
        /// [`GpioTrait::get`] only need `&self` and are single, hardware-
        /// atomic MMIO operations (`BSRR`-style — see
        /// `peripherals::stm32g4::gpio`), so concurrent calls from both
        /// tasks (including one preempting the other mid-call) are
        /// already safe without any software-level mutual exclusion.
        pub(crate) gpio: Gpio,
    }

    // `pub(crate)`: `#[cfg(test)] mod tests` (a sibling of this module, not
    // a descendant) needs to reach these fields directly.
    #[local]
    pub(crate) struct Local {
        pub(crate) firmware: Firmware,
        pub(crate) adc1: Adc,
        pub(crate) adc2: Adc,
        pub(crate) pwm1: Pwm,
        pub(crate) foc: Foc,
        /// Owned here, not by `Foc` — see `Foc`'s own doc comment for why.
        pub(crate) quadrature: Quadrature,
        // TEMPORARY diagnostic instrumentation — see `adc_isr`'s own
        // `#[cfg(not(test))]` diagnostic prints. Remove alongside those
        // once the "physical position always logs 0 on real hardware"
        // issue is root-caused.
        pub(crate) diag_counter: DiagCounter,
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

        let (firmware, adc1, adc2, pwm1, foc, quadrature, gpio) = bring_up(peripherals);

        (
            Shared {
                current_loop_operation: CurrentLoopOperation::Open,
                last_potentiometer: UnitInterval::new(0),
                pending_fault: Fault::None,
                last_bus_voltage: UnitInterval::new(0),
                filtered_bus_voltage: UnitInterval::new(0),
                last_physical_position: UnitInterval::new(0),
                gpio,
            },
            Local {
                firmware,
                adc1,
                adc2,
                pwm1,
                foc,
                quadrature,
                diag_counter: DiagCounter::default(),
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
        shared = [
            current_loop_operation, last_potentiometer, pending_fault, last_bus_voltage,
            filtered_bus_voltage, last_physical_position, &gpio
        ]
    )]
    fn idle(mut cx: idle::Context) {
        use peripherals::api::gpio::GpioTrait;

        let last_potentiometer = cx.shared.last_potentiometer.lock(|p| *p);
        let last_bus_voltage = cx.shared.last_bus_voltage.lock(|v| *v);
        let last_physical_position = cx.shared.last_physical_position.lock(|p| *p);
        // `&gpio` above (not bare `gpio`): `.get()` only needs `&self` and
        // is a single atomic MMIO read (see `Shared::gpio`'s doc comment),
        // so RTIC hands this out as a plain shared reference, with no
        // `Mutex::lock` needed at all -- unlike every other field here.
        let raw_button_pressed = cx.shared.gpio.get(esc1_discovery::BUTTON_PIN);
        let pending_fault = cx
            .shared
            .pending_fault
            .lock(|fault| core::mem::replace(fault, Fault::None));
        cx.local.firmware.report_fault(pending_fault);
        let (operation, filtered_bus_voltage, physical_position_to_log) = cx.local.firmware.step(
            last_potentiometer,
            last_bus_voltage,
            last_physical_position,
            raw_button_pressed,
        );
        if let Some(operation) = operation {
            cx.shared.current_loop_operation.lock(|o| *o = operation);
        }
        cx.shared
            .filtered_bus_voltage
            .lock(|v| *v = filtered_bus_voltage);

        // `defmt::info!` isn't meaningfully callable in test builds (no
        // logging backend is registered — see `init`'s own `#[cfg(not(test))]`
        // guard) — `let _ = ...` still consumes the value there, so it's
        // not reported as unused.
        #[cfg(not(test))]
        if let Some(position) = physical_position_to_log {
            defmt::info!("physical position: {}", f32::from(position));

            // TEMPORARY diagnostic instrumentation, independent of
            // `adc_isr`/RTIC/interrupts entirely — raw register peeks to
            // isolate whether TIM1 is actually counting and whether ADC1
            // has EVER completed a conversion, since `adc_isr` (bound to
            // the ADC1_2 interrupt) has never been observed to fire at
            // all. See `Local::diag_counter`'s doc comment.
            #[cfg(target_arch = "arm")]
            {
                let cnt_before = stm32_metapac::TIM1.cnt().read().cnt();
                cortex_m::asm::delay(1_000_000);
                let cnt_after = stm32_metapac::TIM1.cnt().read().cnt();
                let tim1_cr1 = stm32_metapac::TIM1.cr1().read();
                let tim1_cr2 = stm32_metapac::TIM1.cr2().read();
                let tim1_psc = stm32_metapac::TIM1.psc().read();
                let tim1_arr = stm32_metapac::TIM1.arr().read().arr();
                let tim1_smcr = stm32_metapac::TIM1.smcr().read();
                let tim1_ccmr3 = stm32_metapac::TIM1.ccmr3().read();
                let tim1_ccr5 = stm32_metapac::TIM1.ccr5().read().ccr();
                let tim1_bdtr = stm32_metapac::TIM1.bdtr().read();
                let tim1_sr = stm32_metapac::TIM1.sr().read();
                let rcc_apb2enr = stm32_metapac::RCC.apb2enr().read();
                let rcc_cfgr = stm32_metapac::RCC.cfgr().read();
                let adc1_cr = stm32_metapac::ADC1.cr().read();
                let adc1_isr = stm32_metapac::ADC1.isr().read();
                let adc1_cfgr = stm32_metapac::ADC1.cfgr().read();
                let adc1_ier = stm32_metapac::ADC1.ier().read();
                let nvic_enabled =
                    cortex_m::peripheral::NVIC::is_enabled(stm32_metapac::Interrupt::ADC1_2);
                let nvic_pending =
                    cortex_m::peripheral::NVIC::is_pending(stm32_metapac::Interrupt::ADC1_2);
                let nvic_active =
                    cortex_m::peripheral::NVIC::is_active(stm32_metapac::Interrupt::ADC1_2);
                defmt::info!(
                    "diag: TIM1 cen={} psc={} arr={} cnt {}->{} mms2={} sms={} ts={} ocm5={} ccr5={} moe={} bif={} | RCC tim1en={} sws={} | ADC1 aden={} adstart={} adrdy={} eos={} ovr={} eosie={} extsel={} exten={} | NVIC enabled={} pending={} active={}",
                    tim1_cr1.cen(),
                    tim1_psc,
                    tim1_arr,
                    cnt_before,
                    cnt_after,
                    tim1_cr2.mms2() as u8,
                    tim1_smcr.sms() as u8,
                    tim1_smcr.ts() as u8,
                    tim1_ccmr3.ocm(0) as u8,
                    tim1_ccr5,
                    tim1_bdtr.moe(),
                    tim1_sr.bif(0),
                    rcc_apb2enr.tim1en(),
                    rcc_cfgr.sws() as u8,
                    adc1_cr.aden(),
                    adc1_cr.adstart(),
                    adc1_isr.adrdy(),
                    adc1_isr.eos(),
                    adc1_isr.ovr(),
                    adc1_ier.eosie(),
                    adc1_cfgr.extsel(),
                    adc1_cfgr.exten() as u8,
                    nvic_enabled,
                    nvic_pending,
                    nvic_active
                );
            }
        }
        #[cfg(test)]
        let _ = physical_position_to_log;
    }

    /// The current loop: once both ADCs have converted this cycle (see
    /// `bring_up`'s doc comment — this task runs twice per TRGO2 edge, but
    /// only the second entry, once `adc2`'s longer sequence has also
    /// completed, has anything fresh to act on), reads every phase's
    /// current and the bus voltage, publishes the potentiometer reading
    /// for the main loop, reads the rotor's physical position from
    /// `Local::quadrature` and publishes it too (see
    /// `Shared::last_physical_position`'s doc comment), and drives
    /// [`FieldOrientedControl::poll`] — applying its output to the PWM
    /// channels, or zeroing them if it returns `None`. On the fake
    /// backend, the callback registry (see
    /// `peripherals::fake::callback_registry`) drives this automatically
    /// once wired up — see `mod tests`' `Harness`-based tests.
    ///
    /// Also drives [`esc1_discovery::CAN_SHUTDOWN_PIN`] (PC11 — repurposed
    /// here as a scope/logic-analyzer probe, not for its usual CAN
    /// transceiver role, which this app doesn't use) high for the
    /// function's entire body, low again just before returning — a
    /// TEMPORARY diagnostic (see `Local::diag_counter`'s doc comment)
    /// letting real hardware show this ISR's actual entry-to-exit timing
    /// and firing rate externally, independent of anything `defmt` logs.
    ///
    /// No early return for the "adc2 not done yet" case (unlike an
    /// earlier version of this function): on the fake backend, this
    /// function's declared return type is rewritten to hand `Context`
    /// back to its caller (see `macros::rtic_fake`), which a bare
    /// `return;` can't satisfy — wrapping the rest of the body in an `if`
    /// instead avoids needing the macro to rewrite early returns too.
    ///
    /// Similarly, no spin-waiting on `adc1`'s own `try_retrieve_result()`
    /// either, for the "expected done, but isn't yet" case: by the time
    /// `adc2`'s longer sequence completes, `adc1`'s shorter one should
    /// already have (see `bring_up`'s doc comment) — if it hasn't,
    /// that's reported as [`Fault::AdcConversionIncomplete`] and left at
    /// that, rather than blocked on. The current loop below then reads
    /// whatever's left in `adc1`'s sample buffer from its last
    /// completed conversion instead of this cycle's — stale, but safe
    /// to act on regardless, since the eventual fault reaction (once
    /// `Firmware::step` grows one) is what actually stops the motor,
    /// not this check.
    #[allow(unused_mut)]
    #[task(
        binds = ADC1_2,
        priority = 1,
        local = [adc1, adc2, pwm1, foc, quadrature, diag_counter],
        shared = [
            current_loop_operation, last_potentiometer, pending_fault, last_bus_voltage,
            filtered_bus_voltage, last_physical_position, &gpio
        ]
    )]
    fn adc_isr(mut cx: adc_isr::Context) {
        use peripherals::api::adc::AdcTrait;
        use peripherals::api::gpio::GpioTrait;
        use peripherals::api::pwm::PwmTrait;
        use peripherals::api::quadrature::QuadratureTrait;

        cx.shared.gpio.set(esc1_discovery::CAN_SHUTDOWN_PIN, true); // DO NOT SUBMIT

        // TEMPORARY diagnostic instrumentation (see `Local::diag_counter`'s
        // doc comment) — proves whether `adc_isr` fires at all, whether
        // each ADC reports its sequence done, and what the raw quadrature
        // read returns from inside this interrupt, all in one place.
        // Throttled to roughly twice a second at this ISR's expected 20kHz
        // rate so it doesn't flood the log.
        let diag_count = cx.local.diag_counter.tick();
        let log_this_cycle = diag_count % 10_000 == 1;

        let adc1_done = cx.local.adc1.try_retrieve_result();
        if !adc1_done {
            cx.shared
                .pending_fault
                .lock(|fault| *fault = (*fault).max(Fault::AdcConversionIncomplete));
        }
        let adc2_done = cx.local.adc2.try_retrieve_result();

        #[cfg(not(test))]
        if log_this_cycle {
            defmt::info!(
                "adc_isr diag #{}: adc1_done={} adc2_done={}",
                diag_count,
                adc1_done,
                adc2_done
            );
        }

        if adc2_done {
            let u = cx.local.adc1.get_sample(0); // OPAMP1
            let potentiometer = cx.local.adc1.get_sample(1);
            let v = cx.local.adc2.get_sample(0); // OPAMP2
            let w = cx.local.adc2.get_sample(1); // OPAMP3
            let vbus = cx.local.adc2.get_sample(2);

            cx.shared.last_potentiometer.lock(|p| *p = potentiometer);
            cx.shared.last_bus_voltage.lock(|v| *v = vbus);
            // The *filtered* bus voltage (see `Firmware::step`) is what
            // actually gets applied here -- filtering happens once per
            // main-loop iteration, not every current-loop cycle.
            let filtered_bus_voltage = cx.shared.filtered_bus_voltage.lock(|v| *v);
            cx.local.foc.set_bus_voltage(filtered_bus_voltage);

            let physical_angle = cx.local.quadrature.position();
            cx.shared
                .last_physical_position
                .lock(|p| *p = physical_angle);

            #[cfg(not(test))]
            if log_this_cycle {
                defmt::info!(
                    "adc_isr diag #{}: raw physical_angle={} potentiometer={}",
                    diag_count,
                    f32::from(physical_angle),
                    f32::from(potentiometer)
                );
            }

            let operation = cx.shared.current_loop_operation.lock(|o| *o);
            // Command 0% duty cycle if the FOC step() function returns None.
            // TODO: Consider adding enable/disable methods in the PWM API if the PWM can
            // be enabled through a hardware (BREAK) event.
            let duty_cycles = cx
                .local
                .foc
                .poll(u, v, w, physical_angle, operation)
                .unwrap_or((
                    UnitInterval::new(0),
                    UnitInterval::new(0),
                    UnitInterval::new(0),
                ));
            cx.local.pwm1.set_duty_cycle(0, duty_cycles.0);
            cx.local.pwm1.set_duty_cycle(1, duty_cycles.1);
            cx.local.pwm1.set_duty_cycle(2, duty_cycles.2);
        }

        // `log_this_cycle` only drives `#[cfg(not(test))]` diagnostic
        // prints above — see `physical_position_to_log`'s own
        // `#[cfg(test)]` companion in `idle` for why this needs one too.
        #[cfg(test)]
        let _ = log_this_cycle;

        cx.shared.gpio.set(esc1_discovery::CAN_SHUTDOWN_PIN, false); // DO NOT SUBMIT
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

    /// Advances `harness` far enough to comfortably cross any of this
    /// module's own timing windows (`BIAS_AVERAGE_WINDOW`,
    /// `DEBOUNCE_DURATION`, `PHASE_LOCK_SETTLE_WINDOW`) — for a test that
    /// just needs simulated time to pass, not a specific state change to
    /// wait for.
    fn advance_idle_generously(harness: &app::Harness) {
        for _ in 0..2_000_000 {
            step_idle(harness);
        }
    }

    /// Presses the button, advances until `predicate` is satisfied, then
    /// releases it and advances until the edge-detector has observed a
    /// confirmed release (so a subsequent press can be confirmed fresh —
    /// see `Firmware::button_was_active`'s doc comment) — the
    /// press/wait/release/wait-for-reset sequence more than one test
    /// below needs to drive multiple button-gated transitions in a row.
    fn press_button_until(
        harness: &app::Harness,
        fakes: &esc1_discovery::BoardFakePeripherals,
        predicate: impl FnMut(&app::Local, &app::Shared) -> bool,
    ) {
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(harness, predicate);
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, false);
        advance_idle_until(harness, |local, _shared| !local.firmware.button_was_active);
    }

    /// Advances `harness` from wherever it is, through
    /// `SwitchOnDisabled` -> `SwitchedOn` (one button press) ->
    /// `OperationEnabled` (a second press, once
    /// `PHASE_LOCK_SETTLE_WINDOW` has had time to elapse) — the common
    /// setup several tests below need before they can exercise
    /// `OperationEnabled`'s or `QuickStopActive`'s own behavior.
    fn advance_to_operation_enabled(
        harness: &app::Harness,
        fakes: &esc1_discovery::BoardFakePeripherals,
    ) {
        advance_idle_until(harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });
        press_button_until(harness, fakes, |local, _shared| {
            local.firmware.state == ds402::State::SwitchedOn
        });
        advance_idle_generously(harness);
        press_button_until(harness, fakes, |local, _shared| {
            local.firmware.state == ds402::State::OperationEnabled
        });
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
    fn confirmed_button_press_cascades_switch_on_disabled_to_switched_on() {
        assert_eq!(
            on_confirmed_button_press(ds402::State::SwitchOnDisabled),
            ds402::State::SwitchedOn
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
    fn firmware_report_fault_keeps_the_more_severe_of_the_two_faults() {
        let (_shared, mut local) = app::init(app::init::Context);

        local.firmware.report_fault(Fault::AdcConversionIncomplete);
        assert_eq!(local.firmware.fault, Fault::AdcConversionIncomplete);

        // Reporting `None` afterward doesn't downgrade an
        // already-recorded fault.
        local.firmware.report_fault(Fault::None);
        assert_eq!(local.firmware.fault, Fault::AdcConversionIncomplete);
    }

    #[test]
    fn an_incomplete_adc1_conversion_reports_a_fault_that_idle_drains_into_firmware() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        // Only ADC2's conversion is simulated as complete -- ADC1's is
        // left pending, as if it hadn't finished by the time ADC2's
        // (deliberately longer, see `bring_up`) sequence did. No
        // spin-wait on ADC1 either: `adc_isr` reports this instead, and
        // still proceeds using whatever's left over in ADC1's sample
        // buffer -- see `mod app`'s `adc_isr` doc comment.
        fakes.adc2.assert_trigger_wire();
        let (local, shared) = harness.checkout();
        let cx = app::adc_isr(app::adc_isr::Context { local, shared });
        harness.checkin(cx.local, cx.shared);

        // Not folded into `Firmware::fault` yet -- only `idle` drains
        // `pending_fault`, and it hasn't run since.
        with_local_and_shared(&harness, |local, shared| {
            assert_eq!(shared.pending_fault, Fault::AdcConversionIncomplete);
            assert_eq!(local.firmware.fault, Fault::None);
        });

        step_idle(&harness);

        with_local_and_shared(&harness, |local, shared| {
            assert_eq!(local.firmware.fault, Fault::AdcConversionIncomplete);
            // Drained, not just copied.
            assert_eq!(shared.pending_fault, Fault::None);
        });
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

        // Confirmed press: cascades to SwitchedOn.
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchedOn
        });
        assert_eq!(
            with_local_and_shared(&harness, |local, _shared| local.firmware.state),
            ds402::State::SwitchedOn
        );
    }

    #[test]
    fn firmware_commands_current_control_from_the_potentiometer_while_operation_enabled() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_to_operation_enabled(&harness, &fakes);

        let pot = UnitInterval::new(0x4000_0000);
        with_shared_mut(&harness, |shared| shared.last_potentiometer = pot);
        step_idle(&harness);

        assert_eq!(
            with_local_and_shared(&harness, |_local, shared| shared.current_loop_operation),
            CurrentLoopOperation::CurrentControl(pot.expand())
        );
    }

    #[test]
    fn firmware_enters_quick_stop_active_on_a_second_confirmed_press() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_to_operation_enabled(&harness, &fakes);
        // Not `press_button_until`: `QuickStopActive` auto-clears on the
        // very next step (see `Firmware::step`'s own `QuickStopActive`
        // arm) — `press_button_until`'s own release/wait-for-reset phase
        // would run right past it before this test got a chance to
        // observe it.
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::QuickStopActive
        });

        with_local_and_shared(&harness, |local, shared| {
            assert_eq!(local.firmware.state, ds402::State::QuickStopActive);
            assert_eq!(shared.current_loop_operation, CurrentLoopOperation::Open);
        });
    }

    #[test]
    fn quick_stop_active_automatically_clears_back_to_switch_on_disabled() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_to_operation_enabled(&harness, &fakes);
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::QuickStopActive
        });

        // Unconditional, not gated on a further button press -- the
        // button is still held down from reaching `QuickStopActive`
        // above, and the transition happens anyway. See `Firmware::step`'s
        // own `QuickStopActive` arm.
        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });
        with_local_and_shared(&harness, |local, shared| {
            assert_eq!(local.firmware.state, ds402::State::SwitchOnDisabled);
            assert_eq!(shared.current_loop_operation, CurrentLoopOperation::Open);
        });
    }

    #[test]
    fn switched_on_phase_locks_and_only_enables_operation_once_settled() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        advance_idle_until(&harness, |local, _shared| {
            local.firmware.state == ds402::State::SwitchOnDisabled
        });
        press_button_until(&harness, &fakes, |local, _shared| {
            local.firmware.state == ds402::State::SwitchedOn
        });
        with_local_and_shared(&harness, |_local, shared| {
            assert_eq!(
                shared.current_loop_operation,
                CurrentLoopOperation::PhaseLock
            );
        });

        // A press before `PHASE_LOCK_SETTLE_WINDOW` has elapsed doesn't
        // advance any further -- comfortably within the window (see
        // `advance_idle_until`'s own doc comment on how little simulated
        // time a handful of steps advances).
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, true);
        for _ in 0..5 {
            step_idle(&harness);
        }
        assert_eq!(
            with_local_and_shared(&harness, |local, _shared| local.firmware.state),
            ds402::State::SwitchedOn
        );
        fakes.gpio.set(esc1_discovery::BUTTON_PIN, false);
        advance_idle_until(&harness, |local, _shared| !local.firmware.button_was_active);

        // Once settled, a fresh press enables operation.
        advance_idle_generously(&harness);
        press_button_until(&harness, &fakes, |local, _shared| {
            local.firmware.state == ds402::State::OperationEnabled
        });
    }

    #[test]
    fn bus_voltage_is_filtered_in_the_main_loop_before_reaching_foc() {
        let (shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        // A raw VBUS reading published by one `adc_isr` firing --
        // exactly representable in `f32` (only the top 2 bits of its
        // 32-bit raw value are set), so the round trip through
        // `LowPassFilter`'s `f32` domain and back is exact, not just
        // approximate.
        let vbus = sample(0xC000); // 0.75
        fakes.adc2.set_sample(2, vbus);
        fakes.adc1.assert_trigger_wire();
        fakes.adc2.assert_trigger_wire();
        let (local, shared) = harness.checkout();
        let cx = app::adc_isr(app::adc_isr::Context { local, shared });
        harness.checkin(cx.local, cx.shared);

        with_local_and_shared(&harness, |_local, shared| {
            assert_eq!(shared.last_bus_voltage, vbus);
            // Not filtered into `filtered_bus_voltage` yet -- only `idle`
            // does that, and it hasn't run since.
            assert_eq!(shared.filtered_bus_voltage, UnitInterval::new(0));
        });

        step_idle(&harness);

        with_local_and_shared(&harness, |_local, shared| {
            // `idle`'s first step: `LowPassFilter`'s own ramp-up means
            // its output is exactly the first measurement it's seen --
            // see `common::filter::LowPassFilter::poll`'s doc comment.
            assert_eq!(shared.filtered_bus_voltage, vbus);
        });
    }

    #[test]
    fn adc_isr_publishes_the_physical_position_read_from_the_quadrature_encoder() {
        let (shared, local) = app::init(app::init::Context);
        let mut fakes = local.fakes.clone();
        let harness = app::Harness::new(local, shared, &fakes.callback_registry);

        fakes.quadrature.move_by(42); // arbitrary nonzero position
        let expected_position = fakes.quadrature.position();

        fakes.adc1.assert_trigger_wire();
        fakes.adc2.assert_trigger_wire();
        let (local, shared) = harness.checkout();
        let cx = app::adc_isr(app::adc_isr::Context { local, shared });
        harness.checkin(cx.local, cx.shared);

        with_local_and_shared(&harness, |_local, shared| {
            assert_eq!(shared.last_physical_position, expected_position);
        });
    }

    #[test]
    fn firmware_step_logs_the_physical_position_on_the_first_call_then_rate_limits() {
        let (_shared, mut local) = app::init(app::init::Context);
        let position = UnitInterval::new(0x4000_0000);

        let (_operation, _filtered_bus_voltage, logged) =
            local
                .firmware
                .step(UnitInterval::new(0), UnitInterval::new(0), position, false);
        assert_eq!(logged, Some(position));

        // Immediately again: not yet due -- see
        // `PHYSICAL_POSITION_LOG_INTERVAL`.
        let (_operation, _filtered_bus_voltage, logged) =
            local
                .firmware
                .step(UnitInterval::new(0), UnitInterval::new(0), position, false);
        assert_eq!(logged, None);
    }
}
