//! A fake [`AdcTrait`] implementation for host-side testing, with no real
//! hardware involved.
//!
//! [`Adc::new`] hands back two handles onto one shared, simulated ADC:
//! [`Adc`] itself, for firmware, and [`FakeAdc`], for a test to inject the
//! sample values a triggered conversion should "measure" (simulating
//! whatever's driving the analog input), or to observe what firmware
//! configured via [`AdcTrait::open`].

use core::cell::{Cell, RefCell};
use std::rc::Rc;

use common::duration::Duration;
use common::unit_interval::UnitInterval;

use crate::api::adc::{
    AdcInstance, AdcOptions, AdcTrait, AdcTriggerSource, MAX_ADC_SEQUENCE_LENGTH,
};
use crate::api::dma::{DmaRequest, DmaTrait};
use crate::fake::callback_registry::CallbackRegistry;

/// Bit width the fake treats a raw stored sample as spanning — the shared
/// `AdcSampleBuffer` holds raw `u16` counts (matching what a real driver's
/// DMA target holds), so, absent any real ADC resolution to match (unlike
/// `crate::stm32g4::adc`'s own `ADC_RESOLUTION_BITS`), the fake just
/// treats the full 16 bits as significant.
const FAKE_SAMPLE_BITS: u32 = 16;

/// A rough, deliberately non-cycle-accurate per-channel simulated
/// conversion time — enough that a longer sequence takes simulated-time
/// longer than a shorter one (e.g. `motor_control`'s 3-channel ADC2 vs.
/// its 2-channel ADC1, both triggered off the same edge), which is all
/// [`Adc::arm_trigger_out`] needs.
const CHANNEL_CONVERSION_TIME: Duration = Duration::from_nanos(1_000);

/// The shared NVIC vector name a hardware-triggered ADC's conversion
/// completion is scheduled under — see [`Adc::arm_trigger_out`]. Matches
/// what a `#[task(binds = ADC1_2, ...)]` in `macros::rtic_fake`-generated
/// code registers under (see that crate).
///
/// # Panics
/// Panics for any instance other than ADC1/ADC2 — this fake doesn't yet
/// know ADC3/4/5's real shared vector name; add it here once a board
/// actually uses one of them with a hardware trigger source.
fn nvic_vector_name(instance: AdcInstance) -> &'static str {
    match instance {
        AdcInstance::Stm32g4Adc1 | AdcInstance::Stm32g4Adc2 => "ADC1_2",
        _ => unimplemented!(
            "the fake ADC's trigger-out simulation only knows ADC1/ADC2's shared NVIC vector \
             (\"ADC1_2\") so far"
        ),
    }
}

#[derive(Clone, Copy, Default)]
struct AdcState {
    /// `None` while closed. The fake reads/writes samples straight into
    /// this `AdcOptions`' own [`crate::api::adc::AdcSampleBuffer`] — the
    /// same one a real driver's DMA channel would target — rather than
    /// keeping its own separate copy.
    options: Option<AdcOptions>,
    /// Set by [`Adc::trigger`] (for [`AdcTriggerSource::Software`]) or by
    /// [`FakeAdc::assert_trigger_wire`] (for any other trigger source),
    /// cleared by the first [`Adc::try_retrieve_result`] call that observes it
    /// set — see [`AdcTrait::try_retrieve_result`]'s "exactly once" contract.
    conversion_pending: bool,
    /// Whether the ADC is currently waiting for the configured
    /// [`AdcOptions::trigger_source`] wire's next edge — meaningless for
    /// [`AdcTriggerSource::Software`], which has no wire. Set by
    /// [`Adc::open`] immediately for a hardware trigger source (mirroring
    /// real hardware, where the ADC needs no separate software step to
    /// catch its first edge) and re-set by [`Adc::try_retrieve_result`] after
    /// every conversion it observes complete, so it's always ready for the
    /// wire's next edge with no further calls needed. Cleared by
    /// [`FakeAdc::assert_trigger_wire`], which is what actually consumes it
    /// to raise `conversion_pending`.
    armed: bool,
}

/// The simulated ADC state shared between an [`Adc`] and its [`FakeAdc`]
/// counterpart (see [`Adc::new`]).
struct SharedState {
    state: Cell<AdcState>,
    /// Which physical instance this is — needed (only) to name the
    /// simulated completion interrupt (see [`nvic_vector_name`]); real
    /// hardware has no equivalent use for it.
    instance: AdcInstance,
    /// Set by [`Adc::set_callback_registry`] — `None` means trigger-out
    /// simulation is disabled (matches this fake's previous behavior,
    /// which never modeled conversion timing at all).
    registry: RefCell<Option<CallbackRegistry>>,
    /// Whether [`Adc::arm_trigger_out`]'s two handlers (trigger-out ->
    /// schedule completion, completion -> flag it) have already been
    /// registered with `registry` — registered at most once per instance,
    /// the same way `crate::fake::pwm::Pwm`'s perpetuating hook is.
    handlers_registered: Cell<bool>,
}

impl SharedState {
    fn warn(&self, args: core::fmt::Arguments) {
        emit_warning(args);
    }

    /// The registry set via [`Adc::set_callback_registry`].
    ///
    /// # Panics
    /// Panics if none was set. There's no reasonable way to simulate a
    /// hardware [`AdcTriggerSource`] without one — the alternative is
    /// silently dropping every trigger-out edge, which is exactly the
    /// kind of hard-to-debug test failure requiring a registry up front
    /// avoids. This is only ever reached once [`Adc::arm_trigger_out`]
    /// has already required a registry to get this far, so in practice
    /// this always finds one; the panic is here as a backstop, not an
    /// expected path.
    fn registry(&self) -> CallbackRegistry {
        self.registry.borrow().clone().unwrap_or_else(|| {
            panic!(
                "AdcTrait::open() was called with a hardware AdcTriggerSource, but no \
                 CallbackRegistry has been set via Adc::set_callback_registry() -- call it \
                 before open()"
            )
        })
    }
}

/// A fake ADC driver that simulates a single ADC instance's open/close/
/// trigger/conversion state machine, without touching any real hardware.
pub struct Adc(Rc<SharedState>);

/// A test's handle onto the same simulated ADC an [`Adc`] drives — see
/// [`Adc::new`]. Cheap to [`Clone`] (all clones share the same underlying
/// state).
#[derive(Clone)]
pub struct FakeAdc(Rc<SharedState>);

impl Adc {
    /// Creates a fake ADC, closed, with no sample values set, and a
    /// [`FakeAdc`] handle onto the same simulated state. `_dma_request` is
    /// accepted and ignored — there's no real DMA request line to select
    /// here — purely to mirror [`crate::stm32g4::adc::Adc::new`]'s
    /// signature, the same way [`crate::fake::quadrature::Quadrature::new`]'s
    /// `_timer` parameter mirrors its own real counterpart. `instance` *is*
    /// kept (unlike that ignored parameter), to name the simulated
    /// completion interrupt — see [`Self::set_callback_registry`].
    pub fn new(instance: AdcInstance, _dma_request: DmaRequest) -> (Self, FakeAdc) {
        let state = Rc::new(SharedState {
            state: Cell::new(AdcState::default()),
            instance,
            registry: RefCell::new(None),
            handlers_registered: Cell::new(false),
        });
        (Adc(state.clone()), FakeAdc(state))
    }

    /// Enables trigger-out simulation: from the next [`AdcTrait::open`]
    /// call with a hardware [`AdcOptions::trigger_source`], this instance
    /// listens for that source's simulated trigger-out signal (see
    /// [`crate::fake::pwm::Pwm::set_callback_registry`]) and, once
    /// observed, schedules the shared ADC1/ADC2 interrupt to fire through
    /// `registry` after a simulated conversion time — the same interrupt
    /// a `#[task(binds = ADC1_2, ...)]` fires on real hardware.
    ///
    /// Call before `open()` if `open()` will use a hardware trigger
    /// source at all: `open()` panics in that case if no registry has
    /// been set (see [`SharedState::registry`]) — there's no useful way
    /// to run a hardware-triggered fake ADC without one, and silently
    /// dropping every trigger-out edge instead would just turn into a
    /// test that mysteriously never sees a conversion complete. A test
    /// can still call [`FakeAdc::assert_trigger_wire`] directly to
    /// simulate one edge by hand, registry or not — this only concerns
    /// the *automatic*, PWM-driven trigger-out path.
    pub fn set_callback_registry(&self, registry: CallbackRegistry) {
        *self.0.registry.borrow_mut() = Some(registry);
    }

    /// The fake counterpart of [`crate::stm32g4::adc::Adc::claim_pin`] —
    /// see its doc comment for what it's for on real hardware
    /// (`OpAmpInternalOutput`-style resources routed into one of this
    /// instance's channels, bypassing `Gpio::claim_pin`). Deliberately a
    /// no-op here: unlike a claimed *GPIO* pin (which the fake tracks, so
    /// `GpioTrait::configure` can warn about ones never registered — see
    /// `crate::fake::gpio::Gpio::claim_pin`), there's no equivalent
    /// simulated register for an ADC channel to protect, and mapping `T`
    /// to a specific channel number would need this crate to duplicate
    /// `embassy_stm32`'s own (chip-specific, and privately sealed —
    /// `SealedAdcChannel::channel()` isn't reachable outside that crate at
    /// all) pin/channel table, which is exactly the kind of per-target
    /// bifurcation this fake is meant to stay free of. [`crate::claim_pins!`]
    /// calls this once per pin for a whole list at once.
    pub fn claim_pin<T>(&mut self, _pin: T) {}
}

impl AdcTrait for Adc {
    // `_dma`: the fake never needs DMA — its `get_sample()`/`set_sample()`
    // read/write `options`' own buffer directly, with no real transfer
    // involved. Accepted only so this signature matches `AdcTrait::open`'s.
    /// With any [`AdcOptions::trigger_source`] but
    /// [`AdcTriggerSource::Software`], this also arms the ADC to convert on
    /// the configured wire's next edge — see [`AdcTrait::trigger`].
    fn open<D: DmaTrait>(&mut self, options: AdcOptions, _dma: &D) {
        let mut state = self.0.state.get();
        state.armed = options.trigger_source() != AdcTriggerSource::Software;
        state.options = Some(options);
        state.conversion_pending = false;
        self.0.state.set(state);
        self.arm_trigger_out(options);
    }

    fn close(&mut self) {
        let mut state = self.0.state.get();
        state.options = None;
        state.conversion_pending = false;
        state.armed = false;
        self.0.state.set(state);
        // No explicit registry action needed: both of `arm_trigger_out`'s
        // handlers read `state.options` fresh every time they fire, and
        // no-op once they observe `None` here.
    }

    /// Only meaningful for [`AdcTriggerSource::Software`] — any other
    /// trigger source is already armed by [`Adc::open`] (and re-armed by
    /// every [`Adc::try_retrieve_result`] call after that), so this warns and
    /// does nothing instead.
    fn trigger(&self) {
        let mut state = self.0.state.get();
        let Some(options) = state.options else {
            self.0.warn(format_args!(
                "trigger() called while the ADC is not open (call open() first)"
            ));
            return;
        };
        if options.trigger_source() != AdcTriggerSource::Software {
            self.0.warn(format_args!(
                "trigger() called while configured for a hardware AdcTriggerSource \
                 (open() already arms it; call FakeAdc::assert_trigger_wire() to simulate \
                 the wire instead)"
            ));
            return;
        }
        state.conversion_pending = true;
        self.0.state.set(state);
    }

    fn try_retrieve_result(&self) -> bool {
        let mut state = self.0.state.get();
        let done = state.conversion_pending;
        state.conversion_pending = false;
        // A hardware trigger source re-arms itself for the wire's next
        // edge right after every conversion it completes — see `armed`'s
        // doc comment.
        if done
            && state
                .options
                .is_some_and(|options| options.trigger_source() != AdcTriggerSource::Software)
        {
            state.armed = true;
        }
        self.0.state.set(state);
        done
    }

    fn get_sample(&self, sequence_index: u8) -> UnitInterval {
        let state = self.0.state.get();
        let sequence_len = state.options.map_or(0, |options| options.sequence().len());
        if sequence_index as usize >= sequence_len {
            self.0.warn(format_args!(
                "get_sample({sequence_index}) called outside the {sequence_len}-channel sequence configured via open()"
            ));
            return UnitInterval::default();
        }
        // SAFETY: the fake never starts a real, hardware-timed DMA
        // transfer, so nothing else can be concurrently writing this
        // buffer the way real hardware could — only `FakeAdc::set_sample`
        // ever writes it, and this crate's tests never do so from another
        // thread while reading.
        let raw = unsafe { (*state.options.unwrap().buffer().0.get())[sequence_index as usize] };
        UnitInterval::new((raw as u32) << (32 - FAKE_SAMPLE_BITS))
    }
}

impl Adc {
    /// Wires up simulated trigger-out handling for `options`, if
    /// [`Self::set_callback_registry`] was called and `options` uses a
    /// hardware trigger source — see [`Self::set_callback_registry`]'s
    /// doc comment. Registers (at most once per instance) two handlers:
    ///
    /// - one under the trigger source's own name, that consumes the simulated
    ///   wire edge (mirroring [`FakeAdc::assert_trigger_wire`]'s arm check),
    ///   then schedules a single completion entry — under a name private to
    ///   this instance (see below), *not* the shared [`nvic_vector_name`]
    ///   directly;
    /// - one under that private completion name, that flags *this* conversion
    ///   done, then itself schedules [`nvic_vector_name`] (the ISR-dispatch
    ///   signal `macros::rtic_fake`-generated code subscribes to) — at the
    ///   same instant it just fired at.
    ///
    /// The private completion name matters for two reasons: that flag
    /// must only ever apply to *this* instance's own conversion, never
    /// another ADC's sharing the same vector; and scheduling
    /// `nvic_vector_name` from *within* it (rather than alongside it, as
    /// a second entry the trigger-out handler schedules directly) is what
    /// guarantees the flag is already set by the time the ISR dispatch
    /// actually runs — two entries scheduled for the identical instant
    /// have no defined relative order otherwise, and `adc_isr` running
    /// before its own flag update would see a stale
    /// `try_retrieve_result() == false` for a conversion that, from the
    /// simulation's perspective, already completed.
    fn arm_trigger_out(&self, options: AdcOptions) {
        if options.trigger_source() == AdcTriggerSource::Software {
            return;
        }
        let registry = self.0.registry(); // panics if none was set — see its own doc comment
        if self.0.handlers_registered.replace(true) {
            return;
        }

        let trigger_out_name = format!("{:?}", options.trigger_source());
        let isr_name = nvic_vector_name(self.0.instance).to_string();
        let completion_name = format!("{:?}ConversionComplete", self.0.instance);

        let shared = self.0.clone();
        let completion_name_clone = completion_name.clone();
        registry.register(trigger_out_name, move |now| {
            let mut state = shared.state.get();
            let Some(options) = state.options else {
                return; // closed
            };
            if !state.armed {
                shared.warn(format_args!(
                    "simulated {:?} trigger-out arrived while the ADC was still converting a \
                     previous edge (dropped, matching real hardware ignoring further edges \
                     until the current conversion completes)",
                    options.trigger_source()
                ));
                return;
            }
            state.armed = false;
            shared.state.set(state);

            let conversion_time = CHANNEL_CONVERSION_TIME * options.sequence().len() as i64;
            shared
                .registry()
                .schedule(completion_name_clone.clone(), now + conversion_time);
        });

        let shared = self.0.clone();
        registry.register(completion_name, move |now| {
            let mut state = shared.state.get();
            if state.options.is_none() {
                return; // closed
            }
            state.conversion_pending = true;
            shared.state.set(state);

            // `schedule` (unlike `fire`) only ever touches its own
            // pending-queue cell, never `hooks` — safe to call from
            // within a currently-firing callback, unlike a reentrant
            // `fire`/`poll`/`advance_clock_by` call would be (see
            // `CallbackRegistry`'s own doc comment).
            shared.registry().schedule(isr_name.clone(), now);
        });
    }
}

impl FakeAdc {
    /// Sets the value a subsequent [`AdcTrait::get_sample`] call will read
    /// back for `sequence_index` — simulating whatever external analog
    /// signal is driving that channel. Writes straight into the
    /// currently-open [`AdcOptions`]' own buffer (only the top
    /// [`FAKE_SAMPLE_BITS`] of `value` survive, rounded down — the same
    /// precision [`AdcTrait::get_sample`] reads back); warns and does
    /// nothing if the ADC isn't open (there's no buffer to write into
    /// yet).
    pub fn set_sample(&self, sequence_index: u8, value: UnitInterval) {
        let state = self.0.state.get();
        let Some(options) = state.options else {
            self.0.warn(format_args!(
                "FakeAdc::set_sample({sequence_index}) called while the ADC is not open (call open() first)"
            ));
            return;
        };
        if sequence_index as usize >= MAX_ADC_SEQUENCE_LENGTH {
            self.0.warn(format_args!(
                "FakeAdc::set_sample({sequence_index}) called with an index beyond MAX_ADC_SEQUENCE_LENGTH"
            ));
            return;
        }
        let raw = (value.raw() >> (32 - FAKE_SAMPLE_BITS)) as u16;
        // SAFETY: see `Adc::get_sample`'s doc comment — same reasoning
        // applies to this write.
        unsafe { (*options.buffer().0.get())[sequence_index as usize] = raw };
    }

    /// Simulates the configured [`AdcTriggerSource`] wire asserting — e.g.
    /// a PWM timer's TRGO/TRGO2 firing. [`Adc::open`] already arms the ADC
    /// for a hardware trigger source's first edge (see [`AdcTrait::open`]),
    /// and [`Adc::try_retrieve_result`] re-arms it for every edge after that,
    /// so this needs no other call before it: warns and does nothing if
    /// the ADC isn't open, is configured for
    /// [`AdcTriggerSource::Software`] (which has no wire — call
    /// [`AdcTrait::trigger`] instead), or was already consumed by an
    /// earlier wire assertion the firmware hasn't observed complete yet
    /// (via [`AdcTrait::try_retrieve_result`]) — mirroring real hardware,
    /// where a triggered ADC ignores further edges until it's done with
    /// the current conversion.
    pub fn assert_trigger_wire(&self) {
        let mut state = self.0.state.get();
        let Some(options) = state.options else {
            self.0.warn(format_args!(
                "assert_trigger_wire() called while the ADC is not open (call open() first)"
            ));
            return;
        };
        if options.trigger_source() == AdcTriggerSource::Software {
            self.0.warn(format_args!(
                "assert_trigger_wire() called while configured for AdcTriggerSource::Software \
                 (call AdcTrait::trigger() instead)"
            ));
            return;
        }
        if !state.armed {
            self.0.warn(format_args!(
                "assert_trigger_wire() called while the ADC is still converting a previous \
                 wire assertion (call AdcTrait::try_retrieve_result() first)"
            ));
            return;
        }
        state.armed = false;
        state.conversion_pending = true;
        self.0.state.set(state);
    }

    /// Whether [`AdcTrait::open`] has been called more recently than
    /// [`AdcTrait::close`].
    pub fn is_open(&self) -> bool {
        self.0.state.get().options.is_some()
    }

    /// The [`AdcOptions`] passed to the most recent [`AdcTrait::open`]
    /// call, if the ADC is currently open.
    pub fn options(&self) -> Option<AdcOptions> {
        self.0.state.get().options
    }
}

// See peripherals/src/fake/gpio.rs for why this is cfg(test)-gated rather
// than cfg(not(target_arch = "arm")).
#[cfg(test)]
fn emit_warning(args: core::fmt::Arguments) {
    eprintln!("adc fake warning: {args}");
}

#[cfg(not(test))]
fn emit_warning(_args: core::fmt::Arguments) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::adc::AdcSampleBuffer;
    use crate::fake::dma;

    /// `AdcTrait::open` needs a `&impl DmaTrait` — the fake `Adc` ignores
    /// it entirely (see `open`'s doc comment above), so tests just need
    /// something of the right type, not a meaningfully configured one.
    fn unused_dma() -> dma::Dma {
        dma::Dma::new().0
    }

    /// A fresh, independent buffer for a test to hand to `AdcOptions::new`
    /// — `Box::leak` rather than a shared `static`: unlike
    /// `crate::api::adc`'s own tests, these tests really do read/write a
    /// buffer's contents via `get_sample()`/`set_sample()`, and tests run
    /// in parallel on separate threads, so sharing one buffer across tests
    /// would race.
    fn buffer() -> &'static AdcSampleBuffer {
        Box::leak(Box::new(AdcSampleBuffer::new()))
    }

    /// Builds a `UnitInterval` that round-trips exactly through the fake's
    /// `FAKE_SAMPLE_BITS`-wide storage — a raw top-16-bits value, matching
    /// what `FakeAdc::set_sample`/`Adc::get_sample` actually keep.
    fn sample(top_16_bits: u16) -> UnitInterval {
        UnitInterval::new((top_16_bits as u32) << 16)
    }

    #[test]
    fn starts_closed_with_no_options() {
        let (_adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        assert!(!fake.is_open());
        assert_eq!(fake.options(), None);
    }

    #[test]
    fn open_records_options_and_marks_open() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        let options = AdcOptions::new(&[4], buffer(), AdcTriggerSource::Software);
        adc.open(options, &unused_dma());
        assert!(fake.is_open());
        assert_eq!(fake.options(), Some(options));
    }

    #[test]
    fn close_marks_closed() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(
            AdcOptions::new(&[4], buffer(), AdcTriggerSource::Software),
            &unused_dma(),
        );
        adc.close();
        assert!(!fake.is_open());
        assert_eq!(fake.options(), None);
    }

    #[test]
    fn try_retrieve_result_is_false_before_any_trigger() {
        let (adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        assert!(!adc.try_retrieve_result());
    }

    #[test]
    fn try_retrieve_result_returns_true_exactly_once_per_trigger() {
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(
            AdcOptions::new(&[4], buffer(), AdcTriggerSource::Software),
            &unused_dma(),
        );
        adc.trigger();
        assert!(adc.try_retrieve_result());
        assert!(!adc.try_retrieve_result());
        adc.trigger();
        assert!(adc.try_retrieve_result());
    }

    #[test]
    fn trigger_while_closed_does_not_arm_a_conversion() {
        let (adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.trigger();
        assert!(!adc.try_retrieve_result());
    }

    #[test]
    fn get_sample_reads_back_a_value_set_via_fake() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(
            AdcOptions::new(&[4, 7], buffer(), AdcTriggerSource::Software),
            &unused_dma(),
        );
        fake.set_sample(1, sample(2345));
        adc.trigger();
        assert!(adc.try_retrieve_result());
        assert_eq!(adc.get_sample(1), sample(2345));
    }

    #[test]
    fn get_sample_defaults_to_zero() {
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(
            AdcOptions::new(&[4], buffer(), AdcTriggerSource::Software),
            &unused_dma(),
        );
        assert_eq!(adc.get_sample(0), UnitInterval::default());
    }

    #[test]
    fn get_sample_outside_the_configured_sequence_warns_and_returns_zero() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(
            AdcOptions::new(&[4], buffer(), AdcTriggerSource::Software),
            &unused_dma(),
        );
        fake.set_sample(1, sample(999));
        assert_eq!(adc.get_sample(1), UnitInterval::default());
    }

    #[test]
    fn trigger_with_a_hardware_source_warns_and_does_nothing() {
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising,
            ),
            &unused_dma(),
        );
        adc.trigger();
        assert!(!adc.try_retrieve_result());
    }

    #[test]
    fn open_with_a_hardware_source_arms_immediately() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising,
            ),
            &unused_dma(),
        );
        // No `trigger()` call needed — `open()` alone armed it.
        fake.assert_trigger_wire();
        assert!(adc.try_retrieve_result());
    }

    #[test]
    fn assert_trigger_wire_twice_without_an_intervening_try_retrieve_result_warns_the_second_time()
    {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising,
            ),
            &unused_dma(),
        );
        fake.assert_trigger_wire();
        // The ADC is still "converting" the first assertion (nothing has
        // observed it complete via `try_retrieve_result()` yet), so a second
        // wire assertion should warn and do nothing, just like real
        // hardware ignores further edges until it's done with the current
        // conversion.
        fake.assert_trigger_wire();
        assert!(adc.try_retrieve_result());
        assert!(!adc.try_retrieve_result());
    }

    #[test]
    #[should_panic(expected = "no CallbackRegistry has been set")]
    fn open_with_a_hardware_source_without_a_registry_panics() {
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        // No `set_callback_registry()` call — there's no useful way to
        // run a hardware-triggered fake ADC without one (see
        // `SharedState::registry`'s doc comment), so this must panic
        // rather than silently behave like `AdcTriggerSource::Software`.
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising,
            ),
            &unused_dma(),
        );
    }

    #[test]
    fn try_retrieve_result_rearms_for_the_wires_next_edge() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising,
            ),
            &unused_dma(),
        );
        // Two full cycles, with no `trigger()` call anywhere — `open()`
        // arms the first, and `try_retrieve_result()` re-arms for the second.
        fake.assert_trigger_wire();
        assert!(adc.try_retrieve_result());
        fake.assert_trigger_wire();
        assert!(adc.try_retrieve_result());
    }

    #[test]
    fn assert_trigger_wire_with_software_source_warns_and_does_not_convert() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(
            AdcOptions::new(&[4], buffer(), AdcTriggerSource::Software),
            &unused_dma(),
        );
        fake.assert_trigger_wire();
        assert!(!adc.try_retrieve_result());
    }

    #[test]
    fn assert_trigger_wire_while_closed_warns_and_does_nothing() {
        let (_adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        fake.assert_trigger_wire();
        assert!(!fake.is_open());
    }

    fn registry() -> CallbackRegistry {
        let (_provider, fake_clock) = crate::fake::clock::ClockProvider::new((), 1_000_000_000);
        CallbackRegistry::new(fake_clock)
    }

    #[test]
    fn trigger_out_edge_does_not_complete_the_conversion_immediately() {
        let registry = registry();
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry.clone());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ),
            &unused_dma(),
        );

        registry.fire("Stm32g4Tim1TriggerOut2Rising");
        // The edge only starts the (simulated) conversion — it isn't done
        // until the scheduled "ADC1_2" completion fires, later.
        assert!(!adc.try_retrieve_result());
    }

    #[test]
    fn conversion_completes_after_the_scheduled_conversion_time() {
        let registry = registry();
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry.clone());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ),
            &unused_dma(),
        );

        registry.fire("Stm32g4Tim1TriggerOut2Rising");
        registry.advance_clock_by(Duration::from_millis(1));
        assert!(adc.try_retrieve_result());
    }

    #[test]
    fn a_longer_sequence_takes_simulated_time_longer_to_complete() {
        let registry = registry();
        let (mut adc1, _fake1) =
            Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc1.set_callback_registry(registry.clone());
        adc1.open(
            AdcOptions::new(
                &[4, 7],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ),
            &unused_dma(),
        );
        let (mut adc2, _fake2) =
            Adc::new(AdcInstance::Stm32g4Adc2, DmaRequest::Stm32g4DmamuxReqAdc2);
        adc2.set_callback_registry(registry.clone());
        adc2.open(
            AdcOptions::new(
                &[1, 2, 3],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ),
            &unused_dma(),
        );

        // Both share the same trigger-out edge (mirrors `motor_control`'s
        // ADC1/ADC2 both triggered off TIM1's TRGO2).
        registry.fire("Stm32g4Tim1TriggerOut2Rising");

        // adc1 (2 channels) finishes before adc2 (3 channels) does — the
        // shared ADC1_2 vector fires once for each, at two different
        // simulated times, reproducing the "fires twice per TRGO2 edge"
        // behavior `motor_control`'s ISR already tolerates.
        registry.advance_clock_by(CHANNEL_CONVERSION_TIME * 2);
        assert!(adc1.try_retrieve_result());
        assert!(!adc2.try_retrieve_result());

        // Comfortably past adc2's completion too, rather than landing
        // exactly on its boundary — the tick-based clock's own rounding
        // (see `common::duration_from_ticks`) means an exact boundary
        // isn't guaranteed to land on the same side every time.
        registry.advance_clock_by(CHANNEL_CONVERSION_TIME * 5);
        assert!(adc2.try_retrieve_result());
    }

    #[test]
    fn a_trigger_out_edge_while_still_converting_is_dropped() {
        let registry = registry();
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry.clone());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ),
            &unused_dma(),
        );

        registry.fire("Stm32g4Tim1TriggerOut2Rising"); // starts converting
        registry.fire("Stm32g4Tim1TriggerOut2Rising"); // dropped: still converting
        registry.advance_clock_by(Duration::from_millis(1));

        assert!(adc.try_retrieve_result());
        assert!(!adc.try_retrieve_result()); // only completed once
        assert!(fake.is_open());
    }

    #[test]
    fn closing_stops_a_conversion_already_scheduled_from_completing() {
        let registry = registry();
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.set_callback_registry(registry.clone());
        adc.open(
            AdcOptions::new(
                &[4],
                buffer(),
                AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
            ),
            &unused_dma(),
        );

        registry.fire("Stm32g4Tim1TriggerOut2Rising");
        adc.close();
        registry.advance_clock_by(Duration::from_millis(1));

        assert!(!adc.try_retrieve_result());
    }
}
