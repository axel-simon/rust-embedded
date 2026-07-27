//! Hardware-agnostic DMA request-routing API. Concrete drivers (real or
//! fake) implement [`DmaTrait`]; nothing in this module depends on any
//! particular chip.
//!
//! This driver doesn't move any data by itself — it only tracks, per (DMA
//! controller instance, channel), which [`DmaRequest`] line feeds that
//! channel, and (on real hardware) programs the chip's DMA request multiplexer
//! to match. Actually configuring or starting a transfer on a channel is done
//! within specific peripheral drivers that need DMA transfers.

/// Every DMA controller instance this workspace knows about, across every
/// chip family it's ever targeted. Grows the same way [`DmaRequest`] does:
/// additive only, one variant per instance, named `<Family><Instance>`
/// (e.g. [`Self::Stm32g4Dma1`]/[`Self::Stm32g4Dma2`] for the STM32G4's
/// `DMA1`/`DMA2`) — a concrete [`DmaTrait`] driver only ever recognizes
/// its own family's variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DmaInstance {
    Stm32g4Dma1,
    Stm32g4Dma2,
}

/// Identifies one DMA channel: a specific channel of a specific
/// [`DmaInstance`] on the chip. What [`Self::channel`] numbers mean (how
/// many exist, whether they start at 0 or 1) is up to whichever
/// [`DmaTrait`] implementation is asked about them — e.g. on STM32G4 it's
/// the same 1-based number as the hardware's own `DMA1_CHn`/`DMA2_CHn`
/// naming (see `crate::stm32g4::dma`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DmaChannel {
    pub instance: DmaInstance,
    pub channel: u8,
}

impl DmaChannel {
    pub const fn new(instance: DmaInstance, channel: u8) -> Self {
        DmaChannel { instance, channel }
    }
}

/// Every DMA request line this workspace knows how to route, across every
/// chip family it's ever targeted. Variants are additive only: adding
/// support for a new chip family appends that family's own request lines
/// at the end, under its own name prefix (`<Family>DmamuxReq<Signal>`, one
/// variant per row of that family's request-multiplexer mapping table —
/// [`Self::Stm32g4DmamuxReqAdc1`] and friends come from RM0440 Table 91)
/// — existing variants are never reordered, renamed, or reused for a
/// different signal, since a variant's discriminant is wired to mean a
/// specific register value on real hardware (see
/// `crate::stm32g4::dma::Dma::claim`).
///
/// [`Self::None`] (0, and the [`Default`]) means no hardware request line
/// selected — e.g. a software-triggered or memory-to-memory transfer. A
/// concrete [`DmaTrait`] driver only ever recognizes [`Self::None`] plus
/// its own family's variants; passing it a variant from a different
/// family is a programming error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum DmaRequest {
    #[default]
    None = 0,
    Stm32g4DmamuxReqGen0 = 1,
    Stm32g4DmamuxReqGen1 = 2,
    Stm32g4DmamuxReqGen2 = 3,
    Stm32g4DmamuxReqGen3 = 4,
    Stm32g4DmamuxReqAdc1 = 5,
    Stm32g4DmamuxReqDac1Ch1 = 6,
    Stm32g4DmamuxReqDac1Ch2 = 7,
    Stm32g4DmamuxReqTim6Up = 8,
    Stm32g4DmamuxReqTim7Up = 9,
    Stm32g4DmamuxReqSpi1Rx = 10,
    Stm32g4DmamuxReqSpi1Tx = 11,
    Stm32g4DmamuxReqSpi2Rx = 12,
    Stm32g4DmamuxReqSpi2Tx = 13,
    Stm32g4DmamuxReqSpi3Rx = 14,
    Stm32g4DmamuxReqSpi3Tx = 15,
    Stm32g4DmamuxReqI2c1Rx = 16,
    Stm32g4DmamuxReqI2c1Tx = 17,
    Stm32g4DmamuxReqI2c2Rx = 18,
    Stm32g4DmamuxReqI2c2Tx = 19,
    Stm32g4DmamuxReqI2c3Rx = 20,
    Stm32g4DmamuxReqI2c3Tx = 21,
    Stm32g4DmamuxReqI2c4Rx = 22,
    Stm32g4DmamuxReqI2c4Tx = 23,
    Stm32g4DmamuxReqUsart1Rx = 24,
    Stm32g4DmamuxReqUsart1Tx = 25,
    Stm32g4DmamuxReqUsart2Rx = 26,
    Stm32g4DmamuxReqUsart2Tx = 27,
    Stm32g4DmamuxReqUsart3Rx = 28,
    Stm32g4DmamuxReqUsart3Tx = 29,
    Stm32g4DmamuxReqUart4Rx = 30,
    Stm32g4DmamuxReqUart4Tx = 31,
    Stm32g4DmamuxReqUart5Rx = 32,
    Stm32g4DmamuxReqUart5Tx = 33,
    Stm32g4DmamuxReqLpuart1Rx = 34,
    Stm32g4DmamuxReqLpuart1Tx = 35,
    Stm32g4DmamuxReqAdc2 = 36,
    Stm32g4DmamuxReqAdc3 = 37,
    Stm32g4DmamuxReqAdc4 = 38,
    Stm32g4DmamuxReqAdc5 = 39,
    // 40 is reserved (no request line uses it) — see RM0440 Table 91.
    Stm32g4DmamuxReqDac2Ch1 = 41,
    Stm32g4DmamuxReqTim1Ch1 = 42,
    Stm32g4DmamuxReqTim1Ch2 = 43,
    Stm32g4DmamuxReqTim1Ch3 = 44,
    Stm32g4DmamuxReqTim1Ch4 = 45,
    Stm32g4DmamuxReqTim1Up = 46,
    Stm32g4DmamuxReqTim1Trig = 47,
    Stm32g4DmamuxReqTim1Com = 48,
    Stm32g4DmamuxReqTim8Ch1 = 49,
    Stm32g4DmamuxReqTim8Ch2 = 50,
    Stm32g4DmamuxReqTim8Ch3 = 51,
    Stm32g4DmamuxReqTim8Ch4 = 52,
    Stm32g4DmamuxReqTim8Up = 53,
    Stm32g4DmamuxReqTim8Trig = 54,
    Stm32g4DmamuxReqTim8Com = 55,
    Stm32g4DmamuxReqTim2Ch1 = 56,
    Stm32g4DmamuxReqTim2Ch2 = 57,
    Stm32g4DmamuxReqTim2Ch3 = 58,
    Stm32g4DmamuxReqTim2Ch4 = 59,
    Stm32g4DmamuxReqTim2Up = 60,
    Stm32g4DmamuxReqTim3Ch1 = 61,
    Stm32g4DmamuxReqTim3Ch2 = 62,
    Stm32g4DmamuxReqTim3Ch3 = 63,
    Stm32g4DmamuxReqTim3Ch4 = 64,
    Stm32g4DmamuxReqTim3Up = 65,
    Stm32g4DmamuxReqTim3Trig = 66,
    Stm32g4DmamuxReqTim4Ch1 = 67,
    Stm32g4DmamuxReqTim4Ch2 = 68,
    Stm32g4DmamuxReqTim4Ch3 = 69,
    Stm32g4DmamuxReqTim4Ch4 = 70,
    Stm32g4DmamuxReqTim4Up = 71,
    Stm32g4DmamuxReqTim5Ch1 = 72,
    Stm32g4DmamuxReqTim5Ch2 = 73,
    Stm32g4DmamuxReqTim5Ch3 = 74,
    Stm32g4DmamuxReqTim5Ch4 = 75,
    Stm32g4DmamuxReqTim5Up = 76,
    Stm32g4DmamuxReqTim5Trig = 77,
    Stm32g4DmamuxReqTim15Ch1 = 78,
    Stm32g4DmamuxReqTim15Up = 79,
    Stm32g4DmamuxReqTim15Trig = 80,
    Stm32g4DmamuxReqTim15Com = 81,
    Stm32g4DmamuxReqTim16Ch1 = 82,
    Stm32g4DmamuxReqTim16Up = 83,
    Stm32g4DmamuxReqTim17Ch1 = 84,
    Stm32g4DmamuxReqTim17Up = 85,
    Stm32g4DmamuxReqTim20Ch1 = 86,
    Stm32g4DmamuxReqTim20Ch2 = 87,
    Stm32g4DmamuxReqTim20Ch3 = 88,
    Stm32g4DmamuxReqTim20Ch4 = 89,
    Stm32g4DmamuxReqTim20Up = 90,
    Stm32g4DmamuxReqAesIn = 91,
    Stm32g4DmamuxReqAesOut = 92,
    Stm32g4DmamuxReqTim20Trig = 93,
    Stm32g4DmamuxReqTim20Com = 94,
    Stm32g4DmamuxReqHrtim1M = 95,
    Stm32g4DmamuxReqHrtim1A = 96,
    Stm32g4DmamuxReqHrtim1B = 97,
    Stm32g4DmamuxReqHrtim1C = 98,
    Stm32g4DmamuxReqHrtim1D = 99,
    Stm32g4DmamuxReqHrtim1E = 100,
    Stm32g4DmamuxReqHrtim1F = 101,
    Stm32g4DmamuxReqDac3Ch1 = 102,
    Stm32g4DmamuxReqDac3Ch2 = 103,
    Stm32g4DmamuxReqDac4Ch1 = 104,
    Stm32g4DmamuxReqDac4Ch2 = 105,
    Stm32g4DmamuxReqSpi4Rx = 106,
    Stm32g4DmamuxReqSpi4Tx = 107,
    Stm32g4DmamuxReqSai1A = 108,
    Stm32g4DmamuxReqSai1B = 109,
    Stm32g4DmamuxReqFmacRead = 110,
    Stm32g4DmamuxReqFmacWrite = 111,
    Stm32g4DmamuxReqCordicRead = 112,
    Stm32g4DmamuxReqCordicWrite = 113,
    Stm32g4DmamuxReqUcpd1Rx = 114,
    Stm32g4DmamuxReqUcpd1Tx = 115,
}

/// Abstract interface implemented by every DMA request-router driver, real
/// or fake. Does nothing but track which [`DmaRequest`] feeds each
/// [`DmaChannel`] — see the module doc comment.
pub trait DmaTrait {
    /// Allocates `channel` to carry `request`: on real hardware, every DMA
    /// transfer subsequently started on `channel` is triggered by
    /// `request`'s hardware signal (or, for [`DmaRequest::None`], never by
    /// a peripheral — only by software/chaining). A channel should only be
    /// allocated once; allocating one that's already been allocated causes
    /// a panic in the fake (see `crate::fake::dma::Dma`).
    fn allocate(&mut self, channel: DmaChannel, request: DmaRequest);

    /// The channel currently routed to carry `request`, if any —
    /// [`DmaRequest::None`] always maps to `None` itself (asking "which
    /// channel carries no request" isn't a meaningful query). Meant for a
    /// peripheral driver that needs to actually use its assigned channel
    /// (see e.g. `crate::stm32g4::adc::Adc::open`) to find which one a
    /// board allocated for it via [`Self::allocate`] — without the
    /// peripheral driver having to know or care which channel that was
    /// ahead of time.
    fn lookup_channel(&self, request: DmaRequest) -> Option<DmaChannel>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_is_the_default_and_discriminant_zero() {
        assert_eq!(DmaRequest::default(), DmaRequest::None);
        assert_eq!(DmaRequest::None as u8, 0);
    }

    #[test]
    fn dma_channel_stores_instance_and_channel() {
        let c = DmaChannel::new(DmaInstance::Stm32g4Dma2, 3);
        assert_eq!(c.instance, DmaInstance::Stm32g4Dma2);
        assert_eq!(c.channel, 3);
    }

    #[test]
    fn constructible_in_const_context() {
        const CHANNEL: DmaChannel = DmaChannel::new(DmaInstance::Stm32g4Dma1, 5);
        assert_eq!(
            CHANNEL,
            DmaChannel {
                instance: DmaInstance::Stm32g4Dma1,
                channel: 5
            }
        );
    }

    #[test]
    fn discriminants_match_rm0440_table_91() {
        // Spot-checks a handful of well-known request lines rather than
        // all 115 — the STM32G4 stm32g4::dma tests exercise the full
        // forward/reverse mapping instead.
        assert_eq!(DmaRequest::Stm32g4DmamuxReqGen0 as u8, 1);
        assert_eq!(DmaRequest::Stm32g4DmamuxReqAdc1 as u8, 5);
        assert_eq!(DmaRequest::Stm32g4DmamuxReqUcpd1Tx as u8, 115);
    }
}
