//! A register-level [`DmaTrait`] driver for a real STM32G4 chip's DMA
//! request multiplexer (DMAMUX1), backed by `stm32-metapac`. Also exposes
//! `dma_registers`, for a sibling driver in `crate::stm32g4` (e.g.
//! `crate::stm32g4::adc`) that's been told via
//! [`DmaTrait::lookup_channel`] which channel it owns to actually
//! configure a transfer on it — [`DmaTrait`] itself only ever deals in
//! DMAMUX request routing, never real transfer registers, so that has to
//! live here instead. `pub(super)`, not `pub`: nothing outside
//! `crate::stm32g4` needs real transfer-register access.
//!
//! RM0440 states that STM32G4 category 2 devices (which includes the
//! STM32G431 this code was tested on) have only 2×6 = 12 DMAMUX channels —
//! DMA1's 6 channels on DMAMUX channels 0-5, DMA2's 6 on channels 6-11, no
//! gap — and the same manual's own DMAMUX channel-mapping table (top of
//! page 420) agrees, listing DMA2 channel 1 on DMAMUX channel 6. Both are
//! contradicted by real hardware: DMA2 channel 1 only responds when routed
//! through DMAMUX channel 8, not 6. In other words, a category 2 device's
//! DMAMUX layout is the 2×8 layout also used for category 3 devices. This
//! driver assumes that every STM32G4 device, category 2 or not, has the
//! category 3 shape — 2×8 = 16 DMAMUX channels, DMA1's (up to) 8 on
//! channels 0-7 and DMA2's (up to) 8 on channels 8-15.
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs` — see
//! `peripherals/src/stm32g4/gpio.rs`'s doc comment for why.

use crate::api::dma::{DmaChannel, DmaInstance, DmaRequest, DmaTrait};

/// Number of DMA channels this driver assumes each of DMA1/DMA2 has.
///
/// Channel numbers are 1-based, matching the hardware's own
/// `DMA1_CHn`/`DMA2_CHn` naming — [`DmaChannel::channel`] takes on values
/// `1..=CHANNELS_PER_INSTANCE`, not `0..CHANNELS_PER_INSTANCE-1`.
const CHANNELS_PER_INSTANCE: u8 = 8;

/// Register-level [`DmaTrait`] driver, backed by `stm32-metapac`'s
/// DMAMUX1. A zero-sized type: DMAMUX1 is a single global peripheral
/// (unlike GPIO's per-port register blocks), and its `DMAMUX_CxCR`
/// registers are themselves this driver's only state, so there's nothing
/// to keep in a field.
pub struct Dma;

impl Dma {
    /// Enables DMAMUX1's, DMA1's, and DMA2's peripheral clocks — like
    /// [`crate::stm32g4::gpio::Gpio::new`] enabling every GPIO port's
    /// clock regardless of which pins a board actually uses, this enables
    /// both DMA controllers regardless of which channels get
    /// [`Self::claim_channel`]ed, so register access through the returned
    /// driver (or through [`registers`]) isn't silently ineffective.
    pub fn new() -> Self {
        stm32_metapac::RCC.ahb1enr().modify(|w| {
            w.set_dmamux1en(true);
            w.set_dma1en(true);
            w.set_dma2en(true);
        });
        Dma
    }

    /// Claims ownership of a channel-resource handle — typically a real
    /// `embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA1_CH1>`.
    /// Purely a Rust move: `_peri` is dropped immediately and this driver
    /// does nothing with it, whereas the fake
    /// [`claim_channel`](crate::fake::dma::Dma::claim_channel) actually
    /// tracks the claimed instance/channel.
    pub fn claim_channel<T>(&mut self, _peri: T) {}
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

impl DmaTrait for Dma {
    fn allocate(&mut self, channel: DmaChannel, request: DmaRequest) {
        let index = dmamux_channel_index(channel);
        stm32_metapac::DMAMUX1
            .ccr(index)
            .modify(|w| w.set_dmareq_id(request as u8));
    }

    fn lookup_channel(&self, request: DmaRequest) -> Option<DmaChannel> {
        if request == DmaRequest::None {
            return None;
        }
        for instance in [DmaInstance::Stm32g4Dma1, DmaInstance::Stm32g4Dma2] {
            for ch in 1..=CHANNELS_PER_INSTANCE {
                let channel = DmaChannel::new(instance, ch);
                if channel_request(channel) == request {
                    return Some(channel);
                }
            }
        }
        None
    }
}

/// The request currently routed to `channel`, read directly off
/// `DMAMUX_CxCR.DMAREQ_ID` — used only by [`DmaTrait::lookup_channel`]'s
/// scan; nothing outside this module needs to read a single channel's
/// request back, so this isn't part of [`DmaTrait`] itself.
fn channel_request(channel: DmaChannel) -> DmaRequest {
    let index = dmamux_channel_index(channel);
    let id = stm32_metapac::DMAMUX1.ccr(index).read().dmareq_id();
    dma_request_from_id(id)
}

/// The real DMA1/DMA2 register handles for `channel`: the parent
/// controller (for `ISR`/`IFCR` — the transfer-complete flag isn't a
/// per-channel register), the channel cluster itself (`CCR`/`CNDTR`/
/// `CPAR`/`CMAR`), and the 0-based index `channel.channel` maps to within
/// that controller (for indexing `ISR`/`IFCR`, which `Ch` itself can't).
/// For a sibling driver in `crate::stm32g4` that's been told via
/// [`DmaTrait::lookup_channel`] which channel it owns — see the module
/// doc comment for why this isn't part of [`DmaTrait`] itself, and why
/// it's `pub(super)` rather than `pub`.
///
/// # Panics
/// Panics if `channel.channel` isn't `1..=CHANNELS_PER_INSTANCE`.
pub(super) fn dma_registers(
    channel: DmaChannel,
) -> (stm32_metapac::bdma::Dma, stm32_metapac::bdma::Ch, usize) {
    let index = zero_based_index(channel);
    let controller = match channel.instance {
        DmaInstance::Stm32g4Dma1 => stm32_metapac::DMA1,
        DmaInstance::Stm32g4Dma2 => stm32_metapac::DMA2,
    };
    (controller, controller.ch(index), index)
}

/// `channel.channel`, converted from the hardware's 1-based `DMAn_CHm`
/// numbering (see [`CHANNELS_PER_INSTANCE`]) to a 0-based index into
/// `stm32-metapac`'s `Dma::ch`/`Isr`/`Ifcr` APIs, which are all 0-based.
///
/// # Panics
/// Panics if `channel.channel` isn't `1..=CHANNELS_PER_INSTANCE`.
fn zero_based_index(channel: DmaChannel) -> usize {
    assert!(
        (1..=CHANNELS_PER_INSTANCE).contains(&channel.channel),
        "DMA channel {} out of range (DMA1/DMA2 channels are numbered 1..={CHANNELS_PER_INSTANCE})",
        channel.channel
    );
    (channel.channel - 1) as usize
}

/// Maps a [`DmaChannel`] to its DMAMUX1 register index: DMA1's channels
/// occupy DMAMUX indices `0..CHANNELS_PER_INSTANCE`, DMA2's occupy
/// `CHANNELS_PER_INSTANCE..2*CHANNELS_PER_INSTANCE` — see the module doc
/// comment for why this driver assumes `CHANNELS_PER_INSTANCE` (8) rather
/// than the 6 RM0440 documents for category 2 devices.
///
/// # Panics
/// Panics if `channel.channel` isn't `1..=CHANNELS_PER_INSTANCE`.
fn dmamux_channel_index(channel: DmaChannel) -> usize {
    let index = zero_based_index(channel);
    match channel.instance {
        DmaInstance::Stm32g4Dma1 => index,
        DmaInstance::Stm32g4Dma2 => CHANNELS_PER_INSTANCE as usize + index,
    }
}

/// Converts a raw `DMAMUX_CxCR.DMAREQ_ID` value (RM0440 Table 91) back
/// into the [`DmaRequest`] variant it names — the inverse of `request as
/// u8` for every `Stm32g4DmamuxReq*` variant (plus [`DmaRequest::None`]).
///
/// # Panics
/// Panics if `id` isn't a value this driver (or [`DmaTrait::allocate`])
/// ever writes into `DMAMUX_CxCR` — e.g. 40 (reserved, see
/// [`DmaRequest`]'s doc comment) or anything above 115.
fn dma_request_from_id(id: u8) -> DmaRequest {
    match id {
        0 => DmaRequest::None,
        1 => DmaRequest::Stm32g4DmamuxReqGen0,
        2 => DmaRequest::Stm32g4DmamuxReqGen1,
        3 => DmaRequest::Stm32g4DmamuxReqGen2,
        4 => DmaRequest::Stm32g4DmamuxReqGen3,
        5 => DmaRequest::Stm32g4DmamuxReqAdc1,
        6 => DmaRequest::Stm32g4DmamuxReqDac1Ch1,
        7 => DmaRequest::Stm32g4DmamuxReqDac1Ch2,
        8 => DmaRequest::Stm32g4DmamuxReqTim6Up,
        9 => DmaRequest::Stm32g4DmamuxReqTim7Up,
        10 => DmaRequest::Stm32g4DmamuxReqSpi1Rx,
        11 => DmaRequest::Stm32g4DmamuxReqSpi1Tx,
        12 => DmaRequest::Stm32g4DmamuxReqSpi2Rx,
        13 => DmaRequest::Stm32g4DmamuxReqSpi2Tx,
        14 => DmaRequest::Stm32g4DmamuxReqSpi3Rx,
        15 => DmaRequest::Stm32g4DmamuxReqSpi3Tx,
        16 => DmaRequest::Stm32g4DmamuxReqI2c1Rx,
        17 => DmaRequest::Stm32g4DmamuxReqI2c1Tx,
        18 => DmaRequest::Stm32g4DmamuxReqI2c2Rx,
        19 => DmaRequest::Stm32g4DmamuxReqI2c2Tx,
        20 => DmaRequest::Stm32g4DmamuxReqI2c3Rx,
        21 => DmaRequest::Stm32g4DmamuxReqI2c3Tx,
        22 => DmaRequest::Stm32g4DmamuxReqI2c4Rx,
        23 => DmaRequest::Stm32g4DmamuxReqI2c4Tx,
        24 => DmaRequest::Stm32g4DmamuxReqUsart1Rx,
        25 => DmaRequest::Stm32g4DmamuxReqUsart1Tx,
        26 => DmaRequest::Stm32g4DmamuxReqUsart2Rx,
        27 => DmaRequest::Stm32g4DmamuxReqUsart2Tx,
        28 => DmaRequest::Stm32g4DmamuxReqUsart3Rx,
        29 => DmaRequest::Stm32g4DmamuxReqUsart3Tx,
        30 => DmaRequest::Stm32g4DmamuxReqUart4Rx,
        31 => DmaRequest::Stm32g4DmamuxReqUart4Tx,
        32 => DmaRequest::Stm32g4DmamuxReqUart5Rx,
        33 => DmaRequest::Stm32g4DmamuxReqUart5Tx,
        34 => DmaRequest::Stm32g4DmamuxReqLpuart1Rx,
        35 => DmaRequest::Stm32g4DmamuxReqLpuart1Tx,
        36 => DmaRequest::Stm32g4DmamuxReqAdc2,
        37 => DmaRequest::Stm32g4DmamuxReqAdc3,
        38 => DmaRequest::Stm32g4DmamuxReqAdc4,
        39 => DmaRequest::Stm32g4DmamuxReqAdc5,
        41 => DmaRequest::Stm32g4DmamuxReqDac2Ch1,
        42 => DmaRequest::Stm32g4DmamuxReqTim1Ch1,
        43 => DmaRequest::Stm32g4DmamuxReqTim1Ch2,
        44 => DmaRequest::Stm32g4DmamuxReqTim1Ch3,
        45 => DmaRequest::Stm32g4DmamuxReqTim1Ch4,
        46 => DmaRequest::Stm32g4DmamuxReqTim1Up,
        47 => DmaRequest::Stm32g4DmamuxReqTim1Trig,
        48 => DmaRequest::Stm32g4DmamuxReqTim1Com,
        49 => DmaRequest::Stm32g4DmamuxReqTim8Ch1,
        50 => DmaRequest::Stm32g4DmamuxReqTim8Ch2,
        51 => DmaRequest::Stm32g4DmamuxReqTim8Ch3,
        52 => DmaRequest::Stm32g4DmamuxReqTim8Ch4,
        53 => DmaRequest::Stm32g4DmamuxReqTim8Up,
        54 => DmaRequest::Stm32g4DmamuxReqTim8Trig,
        55 => DmaRequest::Stm32g4DmamuxReqTim8Com,
        56 => DmaRequest::Stm32g4DmamuxReqTim2Ch1,
        57 => DmaRequest::Stm32g4DmamuxReqTim2Ch2,
        58 => DmaRequest::Stm32g4DmamuxReqTim2Ch3,
        59 => DmaRequest::Stm32g4DmamuxReqTim2Ch4,
        60 => DmaRequest::Stm32g4DmamuxReqTim2Up,
        61 => DmaRequest::Stm32g4DmamuxReqTim3Ch1,
        62 => DmaRequest::Stm32g4DmamuxReqTim3Ch2,
        63 => DmaRequest::Stm32g4DmamuxReqTim3Ch3,
        64 => DmaRequest::Stm32g4DmamuxReqTim3Ch4,
        65 => DmaRequest::Stm32g4DmamuxReqTim3Up,
        66 => DmaRequest::Stm32g4DmamuxReqTim3Trig,
        67 => DmaRequest::Stm32g4DmamuxReqTim4Ch1,
        68 => DmaRequest::Stm32g4DmamuxReqTim4Ch2,
        69 => DmaRequest::Stm32g4DmamuxReqTim4Ch3,
        70 => DmaRequest::Stm32g4DmamuxReqTim4Ch4,
        71 => DmaRequest::Stm32g4DmamuxReqTim4Up,
        72 => DmaRequest::Stm32g4DmamuxReqTim5Ch1,
        73 => DmaRequest::Stm32g4DmamuxReqTim5Ch2,
        74 => DmaRequest::Stm32g4DmamuxReqTim5Ch3,
        75 => DmaRequest::Stm32g4DmamuxReqTim5Ch4,
        76 => DmaRequest::Stm32g4DmamuxReqTim5Up,
        77 => DmaRequest::Stm32g4DmamuxReqTim5Trig,
        78 => DmaRequest::Stm32g4DmamuxReqTim15Ch1,
        79 => DmaRequest::Stm32g4DmamuxReqTim15Up,
        80 => DmaRequest::Stm32g4DmamuxReqTim15Trig,
        81 => DmaRequest::Stm32g4DmamuxReqTim15Com,
        82 => DmaRequest::Stm32g4DmamuxReqTim16Ch1,
        83 => DmaRequest::Stm32g4DmamuxReqTim16Up,
        84 => DmaRequest::Stm32g4DmamuxReqTim17Ch1,
        85 => DmaRequest::Stm32g4DmamuxReqTim17Up,
        86 => DmaRequest::Stm32g4DmamuxReqTim20Ch1,
        87 => DmaRequest::Stm32g4DmamuxReqTim20Ch2,
        88 => DmaRequest::Stm32g4DmamuxReqTim20Ch3,
        89 => DmaRequest::Stm32g4DmamuxReqTim20Ch4,
        90 => DmaRequest::Stm32g4DmamuxReqTim20Up,
        91 => DmaRequest::Stm32g4DmamuxReqAesIn,
        92 => DmaRequest::Stm32g4DmamuxReqAesOut,
        93 => DmaRequest::Stm32g4DmamuxReqTim20Trig,
        94 => DmaRequest::Stm32g4DmamuxReqTim20Com,
        95 => DmaRequest::Stm32g4DmamuxReqHrtim1M,
        96 => DmaRequest::Stm32g4DmamuxReqHrtim1A,
        97 => DmaRequest::Stm32g4DmamuxReqHrtim1B,
        98 => DmaRequest::Stm32g4DmamuxReqHrtim1C,
        99 => DmaRequest::Stm32g4DmamuxReqHrtim1D,
        100 => DmaRequest::Stm32g4DmamuxReqHrtim1E,
        101 => DmaRequest::Stm32g4DmamuxReqHrtim1F,
        102 => DmaRequest::Stm32g4DmamuxReqDac3Ch1,
        103 => DmaRequest::Stm32g4DmamuxReqDac3Ch2,
        104 => DmaRequest::Stm32g4DmamuxReqDac4Ch1,
        105 => DmaRequest::Stm32g4DmamuxReqDac4Ch2,
        106 => DmaRequest::Stm32g4DmamuxReqSpi4Rx,
        107 => DmaRequest::Stm32g4DmamuxReqSpi4Tx,
        108 => DmaRequest::Stm32g4DmamuxReqSai1A,
        109 => DmaRequest::Stm32g4DmamuxReqSai1B,
        110 => DmaRequest::Stm32g4DmamuxReqFmacRead,
        111 => DmaRequest::Stm32g4DmamuxReqFmacWrite,
        112 => DmaRequest::Stm32g4DmamuxReqCordicRead,
        113 => DmaRequest::Stm32g4DmamuxReqCordicWrite,
        114 => DmaRequest::Stm32g4DmamuxReqUcpd1Rx,
        115 => DmaRequest::Stm32g4DmamuxReqUcpd1Tx,
        other => panic!("DMAMUX_CxCR.DMAREQ_ID holds {other}, not a known STM32G4 request line"),
    }
}
