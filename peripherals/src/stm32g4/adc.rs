//! A register-level [`AdcTrait`] driver for a real STM32G4 chip, backed by
//! `stm32-metapac`.
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs` — see
//! `peripherals/src/stm32g4/gpio.rs`'s doc comment for why.

use common::unit_interval::UnitInterval;

use crate::api::adc::{AdcOptions, AdcSampleBuffer, AdcTrait};
use crate::api::dma::{DmaRequest, DmaTrait};
use crate::stm32g4::dma;
use stm32_metapac::adc::vals as adc_vals;
use stm32_metapac::adccommon::vals as adccommon_vals;
use stm32_metapac::bdma::vals as bdma_vals;

/// Cycles to busy-wait after enabling the ADC's internal voltage regulator
/// (`ADVREGEN`) before calibrating — covers `tADCVREG_STUP` (a few µs per
/// the datasheet) even at the fastest HCLK a board using this driver might
/// reasonably run at (170 MHz, the STM32G4's own maximum: 20 µs * 170 MHz =
/// 3400 cycles); at any slower HCLK this just waits longer than strictly
/// necessary.
const ADVREGEN_STARTUP_DELAY_CYCLES: u32 = 6_000;

/// Bit width of a raw regular-conversion result — matches `CFGR.RES`'s
/// reset value (12-bit resolution, RM0440 21.4.14); `open()` never writes
/// `RES`, so this stays accurate. Used only to scale a raw sample
/// (0..4095) up into a [`UnitInterval`]'s full 32-bit fractional range in
/// [`Adc::get_sample`].
const ADC_RESOLUTION_BITS: u32 = 12;

/// Register-level [`AdcTrait`] driver for one of the STM32G431's ADC
/// instances (`ADC1`/`ADC2`), backed by `stm32-metapac`. Converts a
/// sequence of up to
/// [`MAX_ADC_SEQUENCE_LENGTH`](crate::api::adc::MAX_ADC_SEQUENCE_LENGTH)
/// channels per [`AdcTrait::trigger`], reading every rank's result out of the
/// shared data register via DMA (see [`AdcTrait::open`]) before the next
/// conversion in the sequence overwrites it.
pub struct Adc {
    regs: stm32_metapac::adc::Adc,
    /// This instance's own DMAMUX request line (`Stm32g4DmamuxReqAdc1`/
    /// `Stm32g4DmamuxReqAdc2`) — set at construction, used by `open()` to
    /// find (via [`DmaTrait::lookup_channel`]) which DMA channel a board
    /// claimed for this ADC.
    dma_request: DmaRequest,
    /// The DMA channel `open()` found via `dma_request` and configured a
    /// transfer on — `None` until `open()` succeeds. Cached here so
    /// `trigger()`/`conversion_done()`/`close()` don't repeat the
    /// `lookup_channel` scan on every call.
    dma_channel: Option<crate::api::dma::DmaChannel>,
    /// The length of the sequence `open()` last configured — `trigger()`
    /// reloads the DMA channel's `CNDTR` with this every call, and
    /// `get_sample()` bounds-checks against it. Meaningless while
    /// `dma_channel` is `None`.
    sequence_length: u8,
    /// This instance's own DMA destination buffer, as given to the most
    /// recent `open()` via `options.buffer()` — `None` until `open()`
    /// succeeds. A `'static` reference the caller owns, not something
    /// this struct allocates — see [`AdcSampleBuffer`]'s doc comment for
    /// why.
    samples: Option<&'static AdcSampleBuffer>,
}

impl Adc {
    /// Wraps the ADC instance behind `regs` (e.g. `stm32_metapac::ADC1`),
    /// identified to [`AdcTrait::open`] as `dma_request` (e.g.
    /// [`DmaRequest::Stm32g4DmamuxReqAdc1`] for `ADC1`) — see
    /// [`validate_dma_request`]. Enables the ADC12 clock domain shared by
    /// `ADC1`/`ADC2` and selects its clock source (see
    /// [`enable_adc_clock`]), so register access through the returned
    /// driver isn't silently ineffective; does not calibrate or enable the
    /// ADC itself — see [`AdcTrait::open`].
    ///
    /// # Panics
    /// Panics if `dma_request` isn't
    /// [`Stm32g4DmamuxReqAdc1`](DmaRequest::Stm32g4DmamuxReqAdc1) or
    /// [`Stm32g4DmamuxReqAdc2`](DmaRequest::Stm32g4DmamuxReqAdc2).
    pub fn new(regs: stm32_metapac::adc::Adc, dma_request: DmaRequest) -> Self {
        validate_dma_request(dma_request);
        enable_adc_clock();
        Adc {
            regs,
            dma_request,
            dma_channel: None,
            sequence_length: 0,
            samples: None,
        }
    }
}

impl AdcTrait for Adc {
    fn open<D: DmaTrait>(&mut self, options: AdcOptions, dma: &D) {
        let sequence = options.sequence();
        assert!(
            !sequence.is_empty(),
            "stm32g4::adc::Adc::open needs a non-empty channel sequence"
        );

        // Exit deep power-down and start the internal voltage regulator —
        // RM0440 21.4.6, required before calibration or conversion.
        self.regs.cr().modify(|w| {
            w.set_deeppwd(false);
            w.set_advregen(true);
        });
        cortex_m::asm::delay(ADVREGEN_STARTUP_DELAY_CYCLES);

        // Calibrate in single-ended mode — RM0440 21.4.8. ADCAL is cleared
        // by hardware once calibration completes; the resulting factor is
        // then applied automatically to every conversion.
        self.regs.cr().modify(|w| {
            w.set_adcaldif(adc_vals::Adcaldif::SINGLE_ENDED);
            w.set_adcal(true);
        });
        while self.regs.cr().read().adcal() {}

        // Configure the regular sequence — every rank's channel, and a
        // conservative sample time for each — before enabling the ADC —
        // RM0440 21.4.19/21.4.14.
        self.regs
            .sqr1()
            .modify(|w| w.set_l((sequence.len() - 1) as u8));
        for (rank, &channel) in sequence.iter().enumerate() {
            set_sequence_rank(self.regs, rank, channel);
            set_sample_time(self.regs, channel, adc_vals::SampleTime::CYCLES12_5);
        }
        self.sequence_length = sequence.len() as u8;

        // Find the DMA channel a board claimed for this ADC instance (see
        // `crate::stm32g4::dma::Dma::claim_channel`), and configure it to
        // copy every conversion result, in rank order, into this
        // instance's `samples` buffer — RM0440 12.4 (basic DMA) / 21.4.14
        // (ADC side).
        let dma_channel = dma.lookup_channel(self.dma_request).unwrap_or_else(|| {
            panic!(
                "no DMA channel claimed for {:?} — see crate::stm32g4::dma::Dma::claim_channel",
                self.dma_request
            )
        });
        let (_controller, ch, _index) = dma::dma_registers(dma_channel);
        let buffer = options.buffer();

        ch.par().write_value(self.regs.dr().as_ptr() as u32);
        // Written once, here, never again — see `AdcSampleBuffer`'s doc
        // comment for why that's safe: its address is `'static` and
        // caller-owned, not tied to wherever this `Adc` itself ends up.
        ch.mar().write_value(buffer.0.get() as u32);
        ch.cr().write(|w| {
            w.set_dir(bdma_vals::Dir::FROM_PERIPHERAL);
            w.set_msize(bdma_vals::Size::BITS16);
            w.set_psize(bdma_vals::Size::BITS16);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(false);
            w.set_pl(bdma_vals::Pl::MEDIUM);
        });
        self.dma_channel = Some(dma_channel);
        self.samples = Some(buffer);

        // Enable the ADC's own DMA request generation — RM0440 21.4.14
        // (CFGR.DMAEN) — without this the ADC never signals DMAMUX at all,
        // regardless of how the channel above is configured. ONE_SHOT
        // (the reset default, set explicitly for clarity) matches this
        // channel's non-circular config: both need re-arming per
        // `trigger()` call, see there.
        self.regs.cfgr().modify(|w| {
            w.set_dmaen(adc_vals::Dmaen::ENABLE);
            w.set_dmacfg(adc_vals::Dmacfg::ONE_SHOT);
        });

        // Enable the ADC and wait for it to report ready — RM0440 21.4.9.
        self.regs.cr().modify(|w| w.set_aden(true));
        while !self.regs.isr().read().adrdy() {}
    }

    fn close(&mut self) {
        if let Some(dma_channel) = self.dma_channel.take() {
            let (_controller, ch, _index) = dma::dma_registers(dma_channel);
            ch.cr().modify(|w| w.set_en(false));
        }
        self.regs
            .cfgr()
            .modify(|w| w.set_dmaen(adc_vals::Dmaen::DISABLE));
        self.regs.cr().modify(|w| w.set_addis(true));
        while self.regs.cr().read().aden() {}
        self.regs.cr().modify(|w| {
            w.set_advregen(false);
            w.set_deeppwd(true);
        });
    }

    fn trigger(&self) {
        let dma_channel = self
            .dma_channel
            .expect("trigger() called before open() succeeded");
        let (_controller, ch, _index) = dma::dma_registers(dma_channel);

        // Re-arm the DMA channel for a fresh `sequence_length`-item
        // transfer. CNDTR can only be written while the channel is
        // disabled (RM0440 12.4.6), and a non-circular channel needs both
        // NDT reloaded and EN toggled off then on to transfer again after
        // the previous transfer completed — it doesn't restart itself.
        // CMAR isn't touched here: `open()` wrote it once, and — being a
        // `'static`, caller-owned address (see `AdcSampleBuffer`'s doc
        // comment) — it's still correct, without needing a driver-side
        // trigger at all. That matters beyond just saving a register
        // write: a future hardware-triggered sequence (e.g. off a timer)
        // would start a transfer without ever calling this method.
        ch.cr().modify(|w| w.set_en(false));
        clear_transfer_flags(dma_channel);
        ch.ndtr().write(|w| w.set_ndt(self.sequence_length as u16));
        ch.cr().modify(|w| w.set_en(true));

        self.regs.cr().modify(|w| w.set_adstart(true));
    }

    fn conversion_done(&self) -> bool {
        let dma_channel = self
            .dma_channel
            .expect("conversion_done() called before open() succeeded");
        let (controller, _ch, index) = dma::dma_registers(dma_channel);

        // TCIF ("transfer complete interrupt flag") — set once every rank
        // in the sequence (armed by `trigger()`) has landed in `samples`,
        // i.e. once `CNDTR` has counted all the way down to 0. Cleared by
        // writing 1 (rc_w1, like GPIO's BSRR — see `crate::stm32g4::gpio`),
        // so this can only ever observe it set once per `trigger()`.
        if controller.isr().read().tcif(index) {
            controller.ifcr().write(|w| w.set_tcif(index, true));
            true
        } else {
            false
        }
    }

    fn get_sample(&self, sequence_index: u8) -> UnitInterval {
        assert!(
            sequence_index < self.sequence_length,
            "get_sample({sequence_index}) called outside the {}-channel sequence configured via open()",
            self.sequence_length
        );
        let buffer = self
            .samples
            .expect("get_sample() called before open() succeeded");
        // SAFETY: only read here, and only after `conversion_done()`
        // observed the DMA transfer that wrote this buffer — see
        // `Adc::trigger`/`Adc::conversion_done`.
        let raw = unsafe { (*buffer.0.get())[sequence_index as usize] };
        UnitInterval::new((raw as u32) << (32 - ADC_RESOLUTION_BITS))
    }
}

/// Clears every interrupt flag DMAMUX/DMA might have set for
/// `dma_channel` — RM0440 12.4.4 (`IFCR`). Broader than just TCIF (see
/// `Adc::conversion_done`) so a stale HTIF/TEIF/GIF from a previous
/// transfer can never be misread as this one's completion.
fn clear_transfer_flags(dma_channel: crate::api::dma::DmaChannel) {
    let (controller, _ch, index) = dma::dma_registers(dma_channel);
    controller.ifcr().write(|w| {
        w.set_gif(index, true);
        w.set_tcif(index, true);
        w.set_htif(index, true);
        w.set_teif(index, true);
    });
}

/// Checks that `dma_request` is one of `ADC1`'s/`ADC2`'s own DMAMUX
/// request lines — the only two [`Adc::new`] accepts.
///
/// # Panics
/// Panics if `dma_request` is anything else.
fn validate_dma_request(dma_request: DmaRequest) {
    match dma_request {
        DmaRequest::Stm32g4DmamuxReqAdc1 | DmaRequest::Stm32g4DmamuxReqAdc2 => {}
        other => {
            panic!("stm32g4::adc::Adc::new was given {other:?}, not an ADC1/ADC2 DMA request line")
        }
    }
}

/// Sets the regular sequence's `rank`th entry (0-based — rank 0 is
/// converted first) to `channel`, in whichever of SQR1 (ranks 0-3), SQR2
/// (4-8), SQR3 (9-13), or SQR4 (14-15) it lives in — RM0440 21.4.19.
///
/// # Panics
/// Panics if `rank` isn't `0..MAX_ADC_SEQUENCE_LENGTH` — `AdcOptions`
/// bounds sequence length to `MAX_ADC_SEQUENCE_LENGTH` (16) itself, so
/// `Adc::open` (the only caller) never passes a `rank` outside that range.
fn set_sequence_rank(regs: stm32_metapac::adc::Adc, rank: usize, channel: u8) {
    match rank {
        0..=3 => regs.sqr1().modify(|w| w.set_sq(rank, channel)),
        4..=8 => regs.sqr2().modify(|w| w.set_sq(rank - 4, channel)),
        9..=13 => regs.sqr3().modify(|w| w.set_sq(rank - 9, channel)),
        14..=15 => regs.sqr4().modify(|w| w.set_sq(rank - 14, channel)),
        _ => unreachable!("AdcOptions bounds sequence length to MAX_ADC_SEQUENCE_LENGTH (16)"),
    }
}

/// Sets `channel`'s sample time in whichever of SMPR1 (channels 0-9) or
/// SMPR2 (channels 10-18) it lives in — RM0440 21.4.14.
fn set_sample_time(regs: stm32_metapac::adc::Adc, channel: u8, sample_time: adc_vals::SampleTime) {
    if channel < 10 {
        regs.smpr()
            .modify(|w| w.set_smp(channel as usize, sample_time));
    } else {
        regs.smpr2()
            .modify(|w| w.set_smp((channel - 10) as usize, sample_time));
    }
}

/// Enables the AHB2 clock for the ADC12 domain shared by `ADC1`/`ADC2`,
/// and selects its clock source: the AHB bus clock divided by 4
/// (`SYNC_DIV4`, RM0440 21.4.3) rather than the asynchronous kernel clock,
/// so no RCC kernel-clock mux (`RCC_CCIPR.ADC12SEL`) needs configuring —
/// dividing by 4 keeps the ADC clock within its 60 MHz maximum (RM0440/
/// datasheet Table 46) for any HCLK a board using this driver might
/// reasonably run at, up to 240 MHz. Idempotent; safe to call more than
/// once, and safe to call for either instance.
fn enable_adc_clock() {
    stm32_metapac::RCC.ahb2enr().modify(|w| w.set_adc12en(true));
    stm32_metapac::ADC12_COMMON
        .ccr()
        .modify(|w| w.set_ckmode(adccommon_vals::Ckmode::SYNC_DIV4));
}
