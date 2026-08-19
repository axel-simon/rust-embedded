//! Hardware-agnostic ADC API. Concrete drivers (real or fake) implement
//! [`AdcTrait`]; nothing in this module depends on any particular chip.

use core::cell::UnsafeCell;

use common::unit_interval::UnitInterval;

use crate::api::dma::DmaTrait;

/// Every ADC instance across all supported MCUs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcInstance {
    Stm32g4Adc1,
    Stm32g4Adc2,
    Stm32g4Adc3,
    Stm32g4Adc4,
    Stm32g4Adc5,
}

/// Maximum number of ADC channels [`AdcOptions`] can hold — the length of
/// the sequence a single [`AdcTrait::trigger`] call converts — and the
/// fixed capacity of every [`AdcSampleBuffer`].
pub const MAX_ADC_SEQUENCE_LENGTH: usize = 16;

/// What starts a triggered [`AdcOptions`] sequence converting — see
/// [`AdcOptions::trigger_source`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcTriggerSource {
    /// Only [`AdcTrait::trigger`] starts a conversion.
    Software,
    /// TIM1's TRGO (update event) wire — e.g. the always-on trigger every
    /// open [`crate::api::pwm::PwmTrait`] driver raises once per period.
    Stm32g4Tim1TriggerOut1Rising,
    /// TIM1's TRGO2 wire — e.g.
    /// [`crate::api::pwm::PwmOptions::mid_point_trigger`].
    Stm32g4Tim1TriggerOut2Rising,
    /// TIM8's TRGO (update event) wire.
    Stm32g4Tim8TriggerOut1Rising,
    /// TIM8's TRGO2 wire.
    Stm32g4Tim8TriggerOut2Rising,
    /// TIM20's TRGO (update event) wire.
    Stm32g4Tim20TriggerOut1Rising,
    /// TIM20's TRGO2 wire.
    Stm32g4Tim20TriggerOut2Rising,
}

/// A fixed-capacity destination buffer for one open ADC's converted
/// samples, holding up to [`MAX_ADC_SEQUENCE_LENGTH`] `u16` results.
/// Opaque by design — a caller only ever needs to create one (typically
/// bound to a `static`, e.g. `static ADC1_SAMPLES: AdcSampleBuffer =
/// AdcSampleBuffer::new();`) and hand a `'static` reference to it to
/// [`AdcOptions::new`]; nothing about its internal representation is
/// meant to be observed from outside this crate.
///
/// Currently always `u16` regardless of chip: fine for now, since every
/// driver in this workspace targets ADCs that report no more than 16 bits
/// per conversion.
pub struct AdcSampleBuffer(pub(crate) UnsafeCell<[u16; MAX_ADC_SEQUENCE_LENGTH]>);

// SAFETY: see the type's doc comment for the invariant that makes sharing
// this across whatever context reads/writes it safe despite the interior
// mutability.
unsafe impl Sync for AdcSampleBuffer {}

impl AdcSampleBuffer {
    /// Creates a zeroed buffer.
    pub const fn new() -> Self {
        AdcSampleBuffer(UnsafeCell::new([0; MAX_ADC_SEQUENCE_LENGTH]))
    }
}

impl Default for AdcSampleBuffer {
    fn default() -> Self {
        Self::new()
    }
}

/// Options for opening an ADC instance.
#[derive(Clone, Copy)]
pub struct AdcOptions {
    // The sequence of channels to convert, in order, on every [`AdcTrait::trigger`] call. Only the
    // first `sequence_length` entries are meaningful; the rest are ignored.
    channels: [u8; MAX_ADC_SEQUENCE_LENGTH],
    sequence_length: u8,
    buffer: &'static AdcSampleBuffer,
    trigger_source: AdcTriggerSource,
}

impl AdcOptions {
    /// Builds an [`AdcOptions`] converting `channels`. `channels[0]` lands in
    /// sequence position 0, readable back via `get_sample(0)`, and so on.
    ///
    /// # Panics
    /// Panics if `channels.len()` exceeds [`MAX_ADC_SEQUENCE_LENGTH`].
    pub const fn new(
        channels: &[u8],
        buffer: &'static AdcSampleBuffer,
        trigger_source: AdcTriggerSource,
    ) -> Self {
        assert!(
            channels.len() <= MAX_ADC_SEQUENCE_LENGTH,
            "AdcOptions sequence exceeds MAX_ADC_SEQUENCE_LENGTH channels"
        );
        let mut buf = [0u8; MAX_ADC_SEQUENCE_LENGTH];
        let mut i = 0;
        while i < channels.len() {
            buf[i] = channels[i];
            i += 1;
        }
        AdcOptions {
            channels: buf,
            sequence_length: channels.len() as u8,
            buffer,
            trigger_source,
        }
    }

    /// The configured channel sequence, in conversion order.
    pub fn sequence(&self) -> &[u8] {
        &self.channels[..self.sequence_length as usize]
    }

    /// What starts a triggered sequence converting — see
    /// [`AdcTriggerSource`].
    pub const fn trigger_source(&self) -> AdcTriggerSource {
        self.trigger_source
    }

    /// The DMA destination buffer this sequence's conversions land in —
    /// for a driver's own `open()`/`get_sample()` to use; not meaningful
    /// to anything outside this crate.
    pub(crate) fn buffer(&self) -> &'static AdcSampleBuffer {
        self.buffer
    }
}

impl core::fmt::Debug for AdcOptions {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AdcOptions")
            .field("channels", &self.sequence())
            .field("buffer", &(self.buffer as *const AdcSampleBuffer))
            .field("trigger_source", &self.trigger_source)
            .finish()
    }
}

// `buffer` compares by address, not contents — `AdcSampleBuffer`'s own
// contents change any time DMA hardware (or, for the fake, `set_sample`)
// writes into it, so comparing them wouldn't mean "these two `AdcOptions`
// configure the same thing," only "these two buffers happened to hold the
// same values just now."
impl PartialEq for AdcOptions {
    fn eq(&self, other: &Self) -> bool {
        self.sequence() == other.sequence()
            && core::ptr::eq(self.buffer, other.buffer)
            && self.trigger_source == other.trigger_source
    }
}

impl Eq for AdcOptions {}

/// Abstract interface implemented by every ADC driver, real or fake.
///
/// The ADC peripheral driver needs to be explicitly [`open`](Self::open)ed
/// (which calibrates it) before use, and [`close`](Self::close)d when done
/// with it. From then on, [`AdcOptions::trigger_source`] decides what
/// starts each conversion: with [`AdcTriggerSource::Software`], each
/// [`trigger`](Self::trigger) call starts converting the whole channel
/// sequence [`open`](Self::open) was configured with; with any other
/// trigger source, [`open`](Self::open) itself already starts (and keeps)
/// converting on that wire's edges, with no further calls needed —
/// [`trigger`](Self::trigger) has nothing left to do. Either way, a caller
/// must still poll [`try_retrieve_result`](Self::try_retrieve_result) to
/// learn when a triggered sequence has finished — for any trigger source
/// but [`AdcTriggerSource::Software`], that call is also what re-arms the
/// ADC for the wire's next edge, so it's not an optional status check —
/// and [`get_sample`](Self::get_sample) then reads back one channel's
/// result by its position in that sequence.
pub trait AdcTrait {
    /// Calibrates the ADC and configures the ADC. With any
    /// [`AdcOptions::trigger_source`] but [`AdcTriggerSource::Software`],
    /// this also starts the ADC converting on the configured wire's next
    /// edge — see [`Self::trigger`].
    ///
    /// `dma` is a separate argument rather than folded into `options`
    /// since it isn't itself part of this ADC's configuration — it's how
    /// `open` finds (via [`DmaTrait::lookup_channel`]) which DMA channel a
    /// board has already [`allocate`](DmaTrait::allocate)d to carry this
    /// ADC's conversions.
    fn open<D: DmaTrait>(&mut self, options: AdcOptions, dma: &D);

    /// Powers the ADC back down. [`Self::open`] must be called again
    /// before any other method.
    fn close(&mut self);

    /// Starts converting the channel sequence. Only relevant if [`Self::open`]
    /// was configured with the [`AdcTriggerSource::Software`] trigger
    /// option. Only meaningful once any previously triggered sequence has
    /// finished converting (see [`Self::try_retrieve_result`]).
    fn trigger(&self);

    /// Tries to retrieve the result of the sequence started by the last
    /// trigger — an explicit [`Self::trigger`] call for
    /// [`AdcTriggerSource::Software`], or the configured wire's last edge
    /// otherwise. Returns `true` exactly once per triggered conversion —
    /// the first call that observes it's finished converting, at which
    /// point [`Self::get_sample`] becomes meaningful — and `false` on
    /// every call before or after that.
    ///
    /// Must be polled regardless of [`AdcOptions::trigger_source`], even
    /// though [`Self::trigger`] itself is a no-op for anything but
    /// [`AdcTriggerSource::Software`]: for every other trigger source,
    /// this call is what re-arms the ADC for the wire's next edge, not
    /// just a status check — skip calling it, and the ADC never converts
    /// again.
    fn try_retrieve_result(&self) -> bool;

    /// Reads back the conversion result for the channel at
    /// `sequence_index` (its position, 0-based, in the sequence
    /// [`Self::open`] was configured with), as a fraction of the ADC's
    /// full-scale reading. Only meaningful after [`Self::try_retrieve_result`]
    /// has returned `true` for the triggered conversion.
    fn get_sample(&self, sequence_index: u8) -> UnitInterval;
}

#[cfg(test)]
mod tests {
    use super::*;

    // Shared across every test below: none of them read/write a buffer's
    // contents (that's `fake::adc`'s tests, which need one buffer per
    // test to avoid racing — see there), only its address, so sharing one
    // `static` here is fine.
    static BUFFER: AdcSampleBuffer = AdcSampleBuffer::new();

    #[test]
    fn new_stores_the_given_channels_in_order() {
        let options = AdcOptions::new(&[3, 7, 1], &BUFFER, AdcTriggerSource::Software);
        assert_eq!(options.sequence(), &[3, 7, 1]);
    }

    #[test]
    fn new_accepts_an_empty_sequence() {
        let options = AdcOptions::new(&[], &BUFFER, AdcTriggerSource::Software);
        assert_eq!(options.sequence(), &[] as &[u8]);
    }

    #[test]
    fn new_accepts_exactly_max_sequence_length_channels() {
        let channels = [5u8; MAX_ADC_SEQUENCE_LENGTH];
        let options = AdcOptions::new(&channels, &BUFFER, AdcTriggerSource::Software);
        assert_eq!(options.sequence(), &channels);
    }

    #[test]
    #[should_panic(expected = "AdcOptions sequence exceeds MAX_ADC_SEQUENCE_LENGTH channels")]
    fn new_panics_when_given_too_many_channels() {
        let channels = [5u8; MAX_ADC_SEQUENCE_LENGTH + 1];
        AdcOptions::new(&channels, &BUFFER, AdcTriggerSource::Software);
    }

    #[test]
    fn constructible_in_const_context() {
        const OPTIONS: AdcOptions = AdcOptions::new(&[2, 9], &BUFFER, AdcTriggerSource::Software);
        assert_eq!(OPTIONS.sequence(), &[2, 9]);
    }

    #[test]
    fn new_stores_the_given_trigger_source() {
        let options = AdcOptions::new(
            &[2],
            &BUFFER,
            AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising,
        );
        assert_eq!(
            options.trigger_source(),
            AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising
        );
    }

    #[test]
    fn equality_compares_buffer_identity_not_contents() {
        static OTHER_BUFFER: AdcSampleBuffer = AdcSampleBuffer::new();
        let a = AdcOptions::new(&[1, 2], &BUFFER, AdcTriggerSource::Software);
        let b = AdcOptions::new(&[1, 2], &BUFFER, AdcTriggerSource::Software);
        let c = AdcOptions::new(&[1, 2], &OTHER_BUFFER, AdcTriggerSource::Software);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn equality_compares_trigger_source() {
        let a = AdcOptions::new(&[1, 2], &BUFFER, AdcTriggerSource::Software);
        let b = AdcOptions::new(
            &[1, 2],
            &BUFFER,
            AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising,
        );
        assert_ne!(a, b);
    }
}
