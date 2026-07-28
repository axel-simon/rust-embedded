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
}

impl AdcOptions {
    /// Builds an [`AdcOptions`] converting `channels`. `channels[0]` lands in
    /// sequence position 0, readable back via `get_sample(0)`, and so on.
    ///
    /// # Panics
    /// Panics if `channels.len()` exceeds [`MAX_ADC_SEQUENCE_LENGTH`].
    pub const fn new(channels: &[u8], buffer: &'static AdcSampleBuffer) -> Self {
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
        }
    }

    /// The configured channel sequence, in conversion order.
    pub fn sequence(&self) -> &[u8] {
        &self.channels[..self.sequence_length as usize]
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
        self.sequence() == other.sequence() && core::ptr::eq(self.buffer, other.buffer)
    }
}

impl Eq for AdcOptions {}

/// Abstract interface implemented by every ADC driver, real or fake.
///
/// The ADC peripheral driver needs to be explicitly [`open`](Self::open)ed
/// (which calibrates it) before use, and [`close`](Self::close)d when done
/// with it. Once open, each [`trigger`](Self::trigger) call starts converting
/// the whole channel sequence [`open`](Self::open) was configured with;
/// [`conversion_done`](Self::conversion_done) reports when that sequence
/// has finished converting, and [`get_sample`](Self::get_sample) reads
/// back one channel's result by its position in that sequence.
pub trait AdcTrait {
    /// Calibrates the ADC and configures the ADC.
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

    /// Starts converting the channel sequence [`Self::open`] was
    /// configured with. Only meaningful once any previously triggered
    /// sequence has finished converting (see [`Self::conversion_done`]).
    fn trigger(&self);

    /// Whether the sequence started by the last [`Self::trigger`] has
    /// finished converting. Returns `true` exactly once per
    /// [`Self::trigger`] call — the first call that observes completion —
    /// and `false` on every call before or after that.
    fn conversion_done(&self) -> bool;

    /// Reads back the conversion result for the channel at
    /// `sequence_index` (its position, 0-based, in the sequence
    /// [`Self::open`] was configured with), as a fraction of the ADC's
    /// full-scale reading. Only meaningful after
    /// [`Self::conversion_done`] has returned `true` for the triggering
    /// [`Self::trigger`] call.
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
        let options = AdcOptions::new(&[3, 7, 1], &BUFFER);
        assert_eq!(options.sequence(), &[3, 7, 1]);
    }

    #[test]
    fn new_accepts_an_empty_sequence() {
        let options = AdcOptions::new(&[], &BUFFER);
        assert_eq!(options.sequence(), &[] as &[u8]);
    }

    #[test]
    fn new_accepts_exactly_max_sequence_length_channels() {
        let channels = [5u8; MAX_ADC_SEQUENCE_LENGTH];
        let options = AdcOptions::new(&channels, &BUFFER);
        assert_eq!(options.sequence(), &channels);
    }

    #[test]
    #[should_panic(expected = "AdcOptions sequence exceeds MAX_ADC_SEQUENCE_LENGTH channels")]
    fn new_panics_when_given_too_many_channels() {
        let channels = [5u8; MAX_ADC_SEQUENCE_LENGTH + 1];
        AdcOptions::new(&channels, &BUFFER);
    }

    #[test]
    fn constructible_in_const_context() {
        const OPTIONS: AdcOptions = AdcOptions::new(&[2, 9], &BUFFER);
        assert_eq!(OPTIONS.sequence(), &[2, 9]);
    }

    #[test]
    fn equality_compares_buffer_identity_not_contents() {
        static OTHER_BUFFER: AdcSampleBuffer = AdcSampleBuffer::new();
        let a = AdcOptions::new(&[1, 2], &BUFFER);
        let b = AdcOptions::new(&[1, 2], &BUFFER);
        let c = AdcOptions::new(&[1, 2], &OTHER_BUFFER);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }
}
