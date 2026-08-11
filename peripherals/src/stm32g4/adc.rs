//! A register-level [`AdcTrait`] driver for a real STM32G4 chip, backed by
//! `stm32-metapac`.
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs` — see
//! `peripherals/src/stm32g4/gpio.rs`'s doc comment for why.

use common::unit_interval::UnitInterval;

use crate::api::adc::{AdcInstance, AdcOptions, AdcSampleBuffer, AdcTrait, AdcTriggerSource};
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
/// channels per triggered conversion — an [`AdcTrait::trigger`] call for
/// [`AdcTriggerSource::Software`], or the configured wire's edge for any
/// other [`AdcOptions::trigger_source`] — reading every rank's result out
/// of the shared data register via DMA (see [`AdcTrait::open`]) before the
/// next conversion in the sequence overwrites it.
pub struct Adc {
    regs: stm32_metapac::adc::Adc,
    /// This instance's own DMAMUX request line (`Stm32g4DmamuxReqAdc1`/
    /// `Stm32g4DmamuxReqAdc2`) — set at construction, used by `open()` to
    /// find (via [`DmaTrait::lookup_channel`]) which DMA channel a board
    /// claimed for this ADC.
    dma_request: DmaRequest,
    /// The DMA channel `open()` found via `dma_request` and configured a
    /// transfer on — `None` until `open()` succeeds. Cached here so
    /// `arm()`/`try_retrieve_result()`/`close()` don't repeat the
    /// `lookup_channel` scan on every call.
    dma_channel: Option<crate::api::dma::DmaChannel>,
    /// The length of the sequence `open()` last configured — `arm()`
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
    /// The trigger source `open()` last configured — see
    /// [`Self::arm`]/[`AdcTrait::trigger`]/[`AdcTrait::try_retrieve_result`]
    /// for how it changes their behavior. Meaningless while `dma_channel`
    /// is `None`; defaults to [`AdcTriggerSource::Software`] until then.
    trigger_source: AdcTriggerSource,
}

impl Adc {
    /// Wraps the ADC instance named by `instance` (e.g.
    /// [`AdcInstance::Stm32g4Adc1`] for `ADC1`), identified to
    /// [`AdcTrait::open`] as `dma_request` (e.g.
    /// [`DmaRequest::Stm32g4DmamuxReqAdc1`] for `ADC1`) — see
    /// [`validate_dma_request`]. Resolved to its concrete
    /// `stm32_metapac::adc::Adc` register block here, eagerly — unlike
    /// [`crate::stm32g4::quadrature::Quadrature::new`]'s deferred-to-`open()`
    /// resolution: an `instance` unsupported under `peripherals`'s current
    /// chip feature (see [`adc_registers`]) is treated as a construction-time
    /// programming error here, not a non-panicking `open()`-time failure —
    /// consistent with [`validate_dma_request`] already panicking in this
    /// same function for a bad `dma_request`. Enables the ADC12 clock domain
    /// shared by `ADC1`/`ADC2` and selects its clock source (see
    /// [`enable_adc_clock`]), so register access through the returned
    /// driver isn't silently ineffective; does not calibrate or enable the
    /// ADC itself — see [`AdcTrait::open`].
    ///
    /// # Panics
    /// Panics if `dma_request` isn't one of `ADC1`-`ADC5`'s own DMAMUX
    /// request lines (see [`validate_dma_request`]), or if `instance` names
    /// an ADC not available under `peripherals`'s currently active chip
    /// feature (see [`adc_registers`]).
    pub fn new(instance: AdcInstance, dma_request: DmaRequest) -> Self {
        validate_dma_request(dma_request);
        enable_adc_clock();
        Adc {
            regs: adc_registers(instance),
            dma_request,
            dma_channel: None,
            sequence_length: 0,
            samples: None,
            trigger_source: AdcTriggerSource::Software,
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
        self.trigger_source = options.trigger_source();

        // Enable the ADC's own DMA request generation — RM0440 21.4.14
        // (CFGR.DMAEN) — without this the ADC never signals DMAMUX at all,
        // regardless of how the channel above is configured. ONE_SHOT
        // (the reset default, set explicitly for clarity) matches this
        // channel's non-circular config: both need re-arming per
        // triggered conversion, see `Self::arm`. EXTSEL/EXTEN route
        // `ADSTART` to the requested `AdcTriggerSource` — see
        // `extsel_exten`.
        let (extsel, exten) = extsel_exten(self.trigger_source);
        self.regs.cfgr().modify(|w| {
            w.set_dmaen(adc_vals::Dmaen::ENABLE);
            w.set_dmacfg(adc_vals::Dmacfg::ONE_SHOT);
            w.set_extsel(extsel);
            w.set_exten(exten);
        });

        // Enable the ADC and wait for it to report ready — RM0440 21.4.9.
        self.regs.cr().modify(|w| w.set_aden(true));
        while !self.regs.isr().read().adrdy() {}

        // A hardware `trigger_source` arms itself right away — there's no
        // separate software step to catch its first edge (unlike
        // `AdcTriggerSource::Software`, which only starts converting once
        // `AdcTrait::trigger` is called). Every edge after that re-arms
        // via `AdcTrait::try_retrieve_result` instead — see `Self::arm`.
        // It also enables this instance's own end-of-sequence interrupt
        // (`IER.EOSIE`, RM0440 21.4.14) right here, rather than requiring a
        // separate opt-in call — a board wiring more than one
        // hardware-triggered ADC to the same shared NVIC vector (ADC1/ADC2
        // both share `ADC1_2` on this chip) gets that vector firing once
        // per instance's completion, not once per triggered edge; see
        // `AdcTrait::try_retrieve_result` for the matching implicit
        // acknowledge.
        if self.trigger_source != AdcTriggerSource::Software {
            self.regs.ier().modify(|w| w.set_eosie(true));
            self.arm();
        }
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

    /// Only meaningful for [`AdcTriggerSource::Software`] — any other
    /// [`AdcOptions::trigger_source`] is already armed by [`Self::open`]
    /// (and re-armed by every [`Self::try_retrieve_result`] call after that),
    /// so this has nothing left to do; calling it anyway is a harmless
    /// no-op rather than a re-arm, since re-arming here could clobber a
    /// conversion the configured wire already has in flight.
    fn trigger(&self) {
        self.dma_channel
            .expect("trigger() called before open() succeeded");
        if self.trigger_source == AdcTriggerSource::Software {
            self.arm();
        }
    }

    fn try_retrieve_result(&self) -> bool {
        let dma_channel = self
            .dma_channel
            .expect("try_retrieve_result() called before open() succeeded");
        let (controller, _ch, index) = dma::dma_registers(dma_channel);

        // TCIF ("transfer complete interrupt flag") — set once every rank
        // in the sequence has landed in `samples`, i.e. once `CNDTR` has
        // counted all the way down to 0. Cleared by writing 1 (rc_w1, like
        // GPIO's BSRR — see `crate::stm32g4::gpio`), so this can only ever
        // observe it set once per armed conversion.
        if controller.isr().read().tcif(index) {
            controller.ifcr().write(|w| w.set_tcif(index, true));
            // A hardware `trigger_source` needs re-arming for its next
            // edge — see `Self::open`'s own re-arm and `Self::arm`'s doc
            // comment. `AdcTriggerSource::Software` re-arms only via an
            // explicit `AdcTrait::trigger` call instead. This also clears
            // `ISR.EOS` (`rc_w1`, same clear-by-write-1 pattern used
            // elsewhere in this file) — the matching acknowledge for the
            // end-of-sequence interrupt `Self::open` implicitly enabled;
            // harmless even if this particular instance's `EOSIE` isn't
            // the one whose NVIC line an application is bound to.
            if self.trigger_source != AdcTriggerSource::Software {
                self.regs.isr().write(|w| w.set_eos(true));
                self.arm();
            }
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
        // SAFETY: only read here, and only after `try_retrieve_result()`
        // observed the DMA transfer that wrote this buffer — see
        // `Adc::arm`/`AdcTrait::try_retrieve_result`.
        let raw = unsafe { (*buffer.0.get())[sequence_index as usize] };
        UnitInterval::new((raw as u32) << (32 - ADC_RESOLUTION_BITS))
    }
}

impl Adc {
    /// Arms the DMA channel and the ADC itself for one more triggered
    /// conversion — called by [`AdcTrait::open`] (to catch a hardware
    /// `trigger_source`'s very first edge), [`AdcTrait::trigger`] (for
    /// [`AdcTriggerSource::Software`]), and [`AdcTrait::try_retrieve_result`]
    /// (to re-arm a hardware `trigger_source` for its next edge) — see
    /// each's own doc comment for when it applies. Assumes `open()` has
    /// already succeeded (every caller either just did that itself or
    /// already checked).
    fn arm(&self) {
        let dma_channel = self
            .dma_channel
            .expect("Adc::arm called before open() succeeded");
        let (_controller, ch, _index) = dma::dma_registers(dma_channel);

        // Re-arm the DMA channel for a fresh `sequence_length`-item
        // transfer. CNDTR can only be written while the channel is
        // disabled (RM0440 12.4.6), and a non-circular channel needs both
        // NDT reloaded and EN toggled off then on to transfer again after
        // the previous transfer completed — it doesn't restart itself.
        // CMAR isn't touched here: `open()` wrote it once, and — being a
        // `'static`, caller-owned address (see `AdcSampleBuffer`'s doc
        // comment) — it's still correct without rewriting it here.
        ch.cr().modify(|w| w.set_en(false));
        clear_transfer_flags(dma_channel);
        ch.ndtr().write(|w| w.set_ndt(self.sequence_length as u16));
        ch.cr().modify(|w| w.set_en(true));

        // With `AdcTriggerSource::Software` (`CFGR.EXTEN` disabled — see
        // `extsel_exten`), this starts converting immediately. With any
        // other trigger source, this only arms the ADC to start on the
        // configured wire's next edge instead.
        self.regs.cr().modify(|w| w.set_adstart(true));
    }
}

/// Clears every interrupt flag DMAMUX/DMA might have set for
/// `dma_channel` — RM0440 12.4.4 (`IFCR`). Broader than just TCIF (see
/// `Adc::try_retrieve_result`) so a stale HTIF/TEIF/GIF from a previous
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

/// Checks that `dma_request` is one of `ADC1`-`ADC5`'s own DMAMUX request
/// lines — the only ones [`Adc::new`] accepts.
///
/// # Panics
/// Panics if `dma_request` is anything else.
fn validate_dma_request(dma_request: DmaRequest) {
    match dma_request {
        DmaRequest::Stm32g4DmamuxReqAdc1
        | DmaRequest::Stm32g4DmamuxReqAdc2
        | DmaRequest::Stm32g4DmamuxReqAdc3
        | DmaRequest::Stm32g4DmamuxReqAdc4
        | DmaRequest::Stm32g4DmamuxReqAdc5 => {}
        other => {
            panic!("stm32g4::adc::Adc::new was given {other:?}, not an ADC1-ADC5 DMA request line")
        }
    }
}

/// The real `ADCx` register-block handle for `instance` — see [`Adc::new`]'s
/// doc comment for why this is resolved eagerly rather than deferred like
/// `crate::stm32g4::quadrature::timer_block`. `Stm32g4Adc3`/`Adc4`/`Adc5`
/// mirror `timer_block`'s `Stm32g4Tim5`/`Stm32g4Tim20` handling of a chip
/// feature that doesn't have them (`stm32_metapac::ADC3`/`ADC4`/`ADC5`
/// don't exist as identifiers at all outside `stm32g474re`) — but panic
/// instead of returning `None`, since [`Adc::new`] resolves eagerly rather
/// than deferring to `open()`.
///
/// # Panics
/// Panics if `instance` names an ADC not available under `peripherals`'s
/// currently active chip feature.
fn adc_registers(instance: AdcInstance) -> stm32_metapac::adc::Adc {
    match instance {
        AdcInstance::Stm32g4Adc1 => stm32_metapac::ADC1,
        AdcInstance::Stm32g4Adc2 => stm32_metapac::ADC2,
        AdcInstance::Stm32g4Adc3 => {
            #[cfg(feature = "stm32g474re")]
            {
                stm32_metapac::ADC3
            }
            #[cfg(not(feature = "stm32g474re"))]
            {
                panic!(
                    "Stm32g4Adc3 needs peripherals' stm32g474re feature (see peripherals/Cargo.toml) — not available on the default stm32g431cb"
                )
            }
        }
        AdcInstance::Stm32g4Adc4 => {
            #[cfg(feature = "stm32g474re")]
            {
                stm32_metapac::ADC4
            }
            #[cfg(not(feature = "stm32g474re"))]
            {
                panic!(
                    "Stm32g4Adc4 needs peripherals' stm32g474re feature (see peripherals/Cargo.toml) — not available on the default stm32g431cb"
                )
            }
        }
        AdcInstance::Stm32g4Adc5 => {
            #[cfg(feature = "stm32g474re")]
            {
                stm32_metapac::ADC5
            }
            #[cfg(not(feature = "stm32g474re"))]
            {
                panic!(
                    "Stm32g4Adc5 needs peripherals' stm32g474re feature (see peripherals/Cargo.toml) — not available on the default stm32g431cb"
                )
            }
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

/// Maps an [`AdcTriggerSource`] to `CFGR`'s `EXTSEL`/`EXTEN` fields —
/// RM0440 21.4.18 (Table 152, "External triggers for regular channels" —
/// identical for every `ADCx` instance this driver supports). `EXTSEL` has
/// no named PAC enum (it's a raw 5-bit code), so these values are cross-
/// checked against `stm32g4xx_hal_driver`'s `stm32g4xx_ll_adc.h`
/// (`LL_ADC_REG_TRIG_EXT_TIM*_TRGO*`) rather than transcribed from the
/// reference manual by hand alone. `EXTEN` is always `RISING_EDGE` for a
/// hardware source — this driver has no way to ask for anything else.
fn extsel_exten(trigger_source: AdcTriggerSource) -> (u8, adc_vals::Exten) {
    match trigger_source {
        AdcTriggerSource::Software => (0, adc_vals::Exten::DISABLED),
        AdcTriggerSource::Stm32g4Tim1TriggerOut1Rising => (9, adc_vals::Exten::RISING_EDGE),
        AdcTriggerSource::Stm32g4Tim1TriggerOut2Rising => (10, adc_vals::Exten::RISING_EDGE),
        AdcTriggerSource::Stm32g4Tim8TriggerOut1Rising => (7, adc_vals::Exten::RISING_EDGE),
        AdcTriggerSource::Stm32g4Tim8TriggerOut2Rising => (8, adc_vals::Exten::RISING_EDGE),
        AdcTriggerSource::Stm32g4Tim20TriggerOut1Rising => (16, adc_vals::Exten::RISING_EDGE),
        AdcTriggerSource::Stm32g4Tim20TriggerOut2Rising => (17, adc_vals::Exten::RISING_EDGE),
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
