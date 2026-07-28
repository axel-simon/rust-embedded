//! A fake [`AdcTrait`] implementation for host-side testing, with no real
//! hardware involved.
//!
//! [`Adc::new`] hands back two handles onto one shared, simulated ADC:
//! [`Adc`] itself, for firmware, and [`FakeAdc`], for a test to inject the
//! sample values a triggered conversion should "measure" (simulating
//! whatever's driving the analog input), or to observe what firmware
//! configured via [`AdcTrait::open`].

use core::cell::Cell;
use std::rc::Rc;

use common::unit_interval::UnitInterval;

use crate::api::adc::{AdcInstance, AdcOptions, AdcTrait, MAX_ADC_SEQUENCE_LENGTH};
use crate::api::dma::{DmaRequest, DmaTrait};

/// Bit width the fake treats a raw stored sample as spanning — the shared
/// `AdcSampleBuffer` holds raw `u16` counts (matching what a real driver's
/// DMA target holds), so, absent any real ADC resolution to match (unlike
/// `crate::stm32g4::adc`'s own `ADC_RESOLUTION_BITS`), the fake just
/// treats the full 16 bits as significant.
const FAKE_SAMPLE_BITS: u32 = 16;

#[derive(Clone, Copy, Default)]
struct AdcState {
    /// `None` while closed. The fake reads/writes samples straight into
    /// this `AdcOptions`' own [`crate::api::adc::AdcSampleBuffer`] — the
    /// same one a real driver's DMA channel would target — rather than
    /// keeping its own separate copy.
    options: Option<AdcOptions>,
    /// Set by [`Adc::trigger`], cleared by the first [`Adc::conversion_done`]
    /// call that observes it set — see [`AdcTrait::conversion_done`]'s
    /// "exactly once" contract.
    conversion_pending: bool,
}

/// The simulated ADC state shared between an [`Adc`] and its [`FakeAdc`]
/// counterpart (see [`Adc::new`]).
struct SharedState(Cell<AdcState>);

impl SharedState {
    fn warn(&self, args: core::fmt::Arguments) {
        emit_warning(args);
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
    /// [`FakeAdc`] handle onto the same simulated state. `_instance`/
    /// `_dma_request` are accepted and ignored — there's no real ADC
    /// instance or DMA request line to select here — purely to mirror
    /// [`crate::stm32g4::adc::Adc::new`]'s signature, the same way
    /// [`crate::fake::quadrature::Quadrature::new`]'s `_timer` parameter
    /// mirrors its own real counterpart.
    pub fn new(_instance: AdcInstance, _dma_request: DmaRequest) -> (Self, FakeAdc) {
        let state = Rc::new(SharedState(Cell::new(AdcState::default())));
        (Adc(state.clone()), FakeAdc(state))
    }
}

impl AdcTrait for Adc {
    // `_dma`: the fake never needs DMA — its `get_sample()`/`set_sample()`
    // read/write `options`' own buffer directly, with no real transfer
    // involved. Accepted only so this signature matches `AdcTrait::open`'s.
    fn open<D: DmaTrait>(&mut self, options: AdcOptions, _dma: &D) {
        let mut state = self.0 .0.get();
        state.options = Some(options);
        state.conversion_pending = false;
        self.0 .0.set(state);
    }

    fn close(&mut self) {
        let mut state = self.0 .0.get();
        state.options = None;
        state.conversion_pending = false;
        self.0 .0.set(state);
    }

    fn trigger(&self) {
        let mut state = self.0 .0.get();
        if state.options.is_none() {
            self.0.warn(format_args!(
                "trigger() called while the ADC is not open (call open() first)"
            ));
            return;
        }
        state.conversion_pending = true;
        self.0 .0.set(state);
    }

    fn conversion_done(&self) -> bool {
        let mut state = self.0 .0.get();
        let done = state.conversion_pending;
        state.conversion_pending = false;
        self.0 .0.set(state);
        done
    }

    fn get_sample(&self, sequence_index: u8) -> UnitInterval {
        let state = self.0 .0.get();
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
        let state = self.0 .0.get();
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

    /// Whether [`AdcTrait::open`] has been called more recently than
    /// [`AdcTrait::close`].
    pub fn is_open(&self) -> bool {
        self.0 .0.get().options.is_some()
    }

    /// The [`AdcOptions`] passed to the most recent [`AdcTrait::open`]
    /// call, if the ADC is currently open.
    pub fn options(&self) -> Option<AdcOptions> {
        self.0 .0.get().options
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
        let options = AdcOptions::new(&[4], buffer());
        adc.open(options, &unused_dma());
        assert!(fake.is_open());
        assert_eq!(fake.options(), Some(options));
    }

    #[test]
    fn close_marks_closed() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(AdcOptions::new(&[4], buffer()), &unused_dma());
        adc.close();
        assert!(!fake.is_open());
        assert_eq!(fake.options(), None);
    }

    #[test]
    fn conversion_done_is_false_before_any_trigger() {
        let (adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        assert!(!adc.conversion_done());
    }

    #[test]
    fn conversion_done_returns_true_exactly_once_per_trigger() {
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(AdcOptions::new(&[4], buffer()), &unused_dma());
        adc.trigger();
        assert!(adc.conversion_done());
        assert!(!adc.conversion_done());
        adc.trigger();
        assert!(adc.conversion_done());
    }

    #[test]
    fn trigger_while_closed_does_not_arm_a_conversion() {
        let (adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.trigger();
        assert!(!adc.conversion_done());
    }

    #[test]
    fn get_sample_reads_back_a_value_set_via_fake() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(AdcOptions::new(&[4, 7], buffer()), &unused_dma());
        fake.set_sample(1, sample(2345));
        adc.trigger();
        assert!(adc.conversion_done());
        assert_eq!(adc.get_sample(1), sample(2345));
    }

    #[test]
    fn get_sample_defaults_to_zero() {
        let (mut adc, _fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(AdcOptions::new(&[4], buffer()), &unused_dma());
        assert_eq!(adc.get_sample(0), UnitInterval::default());
    }

    #[test]
    fn get_sample_outside_the_configured_sequence_warns_and_returns_zero() {
        let (mut adc, fake) = Adc::new(AdcInstance::Stm32g4Adc1, DmaRequest::Stm32g4DmamuxReqAdc1);
        adc.open(AdcOptions::new(&[4], buffer()), &unused_dma());
        fake.set_sample(1, sample(999));
        assert_eq!(adc.get_sample(1), UnitInterval::default());
    }
}
