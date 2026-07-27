//! A register-level
//! [`QuadratureTrait`](crate::api::quadrature::QuadratureTrait) driver for a
//! real STM32G4 chip's timer running in encoder-interface mode, backed by
//! `stm32-metapac`.
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs`, same as
//! [`crate::stm32g4::gpio`]/[`crate::stm32g4::clock`].
//!
//! [`QuadratureTrait::open`] configures the chosen timer for encoder mode 3
//! (RM0440 22.3.7): the counter counts up or down on every edge of both
//! inputs, direction decided by the level of the other input — with both
//! inputs low, a rising edge on input A counts up, matching
//! [`QuadratureInputConfiguration`]'s documented convention. The counter
//! wraps at [`QuadratureOptions::encoder_counts`] (`ARR = encoder_counts -
//! 1`), so [`Quadrature::position`] is just the live counter value scaled
//! into a [`UnitInterval`].
//!
//! # Which timers this driver actually supports
//!
//! By default, this driver only handles 16-bit timers — [`QuadratureTimer`]
//! has no `Stm32g4Tim2` variant (STM32G4's only 32-bit general-purpose
//! timer) at all, and `TIM5`/`TIM20` only get real support when built with
//! one of `peripherals/Cargo.toml`'s chip features that actually has them
//! (e.g. `stm32g474re` — see that file for the full list) enabled —
//! [`QuadratureTrait::open`] logs an error and fails (leaves the driver
//! unopened, same as never calling `open` at all) if asked for either on a
//! chip feature that doesn't. There's no separately-invented capability
//! flag for this: `#[cfg(feature = "stm32g474re")]` (and friends, as more
//! chip features are added) gates it directly, the same feature
//! `stm32-metapac` itself uses to decide which MCU's register/peripheral
//! definitions to generate — see `peripherals/Cargo.toml`. That split
//! exists because, on the STM32G4 chips that actually have `TIM5`
//! (category 3/4 — G473/G474/G483/G484/G491/G4A1), it's a genuine 32-bit
//! general-purpose timer, exactly like the `TIM2` this driver deliberately
//! never supports — its raw counter/auto-reload registers are a different
//! Rust type (`stm32-metapac`'s `TimGp32`) than every 16-bit timer's,
//! needing its own register-access code path. `TIM20` has no such problem
//! (it's a 16-bit advanced-control timer, register-compatible with
//! `TIM1`/`TIM8`), but is gated on the same chip features anyway since both
//! only exist on the same wider set of chips.
//!
//! Every 16-bit timer this driver ever handles is either `TIM1`/`TIM8`/
//! `TIM20` (`stm32-metapac`'s `TimAdv`, advanced-control) or `TIM3`/`TIM4`
//! (`TimGp16`, general-purpose) — and those two share an identical register
//! layout for every field this driver touches (see [`timer_block`]'s doc
//! comment), so this driver treats every 16-bit timer as a single `TimAdv`
//! value rather than branching on which kind it really is underneath.

use common::i64_divider::U64Divider;
use common::unit_interval::UnitInterval;
#[cfg(feature = "stm32g474re")]
use stm32_metapac::timer::TimGp32;
use stm32_metapac::timer::{vals, TimAdv};

use crate::api::quadrature::{
    QuadratureInputConfiguration, QuadratureOptions, QuadratureTimer, QuadratureTrait,
};

/// Register-level [`QuadratureTrait`] driver for a real STM32G4 chip's
/// timer, backed by `stm32-metapac`.
pub struct Quadrature {
    timer: QuadratureTimer,
    /// Set by [`QuadratureTrait::open`] once it's resolved and configured
    /// `timer`'s register block, cleared by [`QuadratureTrait::close`].
    /// `None` means "not open" — every other [`QuadratureTrait`] method
    /// needs it to be `Some` (see [`QuadratureTrait::position`]'s panic).
    /// Resolving the register block (see [`timer_block`]) is deferred all
    /// the way to `open()`, rather than done once in [`Quadrature::new`],
    /// specifically so that asking for an unavailable `TIM5`/`TIM20` (see
    /// this module's doc comment) can be handled as an `open()`-time
    /// failure instead of a `new()`-time panic.
    open: Option<OpenState>,
}

/// `Quadrature`'s state while open — see [`Quadrature::open`].
struct OpenState {
    block: TimerBlock,
    encoder_counts: u32,
    /// Precomputed reciprocal of `encoder_counts`, for
    /// [`Quadrature::position`] — built once here (by [`Quadrature::open`])
    /// rather than on every `position()` call, since [`U64Divider::new`]
    /// itself does a division to build it. Only actually used when
    /// `encoder_counts` isn't a power of two (see `position`'s shift-based
    /// fast path); when it is, `encoder_counts` might be `1`, so `.max(2)`
    /// keeps this construction panic-free (`U64Divider::new` requires a
    /// divisor `> 1`) without needing a second, conditionally-absent field.
    divider: U64Divider,
}

impl Quadrature {
    /// Records which physical timer instance `open()` should later claim
    /// (see [`QuadratureTimer`]) — doesn't touch any hardware yet (not even
    /// to check `timer` is actually available on this chip): see
    /// [`Quadrature::open`].
    pub fn new(timer: QuadratureTimer) -> Self {
        Quadrature { timer, open: None }
    }
}

impl QuadratureTrait for Quadrature {
    /// Enables the timer's peripheral clock, maps it to its register block,
    /// and configures it for encoder-interface mode per `options`.
    ///
    /// Fails (logs an error via `defmt` and returns, leaving this driver
    /// unopened — see [`QuadratureTrait::position`]'s panic) if the timer
    /// given to [`Quadrature::new`] isn't available — see this module's doc
    /// comment for when that happens.
    ///
    /// # Panics
    /// Panics if `options.encoder_counts` doesn't fit the timer's counter
    /// width — 16 bits for every timer except `TIM5` (32-bit, only
    /// reachable when built with a chip feature that has it — see this
    /// module's doc comment). The caller is responsible for having already
    /// configured the pins feeding this timer's channel-1/channel-2 inputs
    /// via [`crate::api::gpio::GpioTrait::configure`] — same division of
    /// responsibility as every `_PIN` constant in a board's `board.rs`.
    fn open(&mut self, options: QuadratureOptions) {
        let Some(block) = timer_block(self.timer) else {
            let name = match self.timer {
                QuadratureTimer::Stm32g4Tim5 => "TIM5",
                QuadratureTimer::Stm32g4Tim20 => "TIM20",
                // `timer_block` only ever returns `None` for TIM5/TIM20 —
                // see its doc comment.
                _ => "timer",
            };
            defmt::error!(
                "quadrature: {} isn't available on this chip feature -- \
                 see peripherals/Cargo.toml's chip-variant features -- \
                 open() failed",
                name
            );
            return;
        };

        let mapping = match options.input_configuration {
            QuadratureInputConfiguration::Ch12AreInputsAB => DIRECT_INPUT_MAPPING,
            QuadratureInputConfiguration::Ch12AreInputsBA => ALTERNATE_INPUT_MAPPING,
        };
        block.configure_encoder_mode(mapping, options.encoder_counts);

        self.open = Some(OpenState {
            block,
            encoder_counts: options.encoder_counts,
            divider: U64Divider::new(options.encoder_counts.max(2)),
        });
    }

    fn close(&mut self) {
        // Nothing to stop if `open()` never succeeded (or already failed
        // this way) — mirrors `crate::stm32g4::adc::Adc::close`'s spirit
        // (safe/idempotent to call regardless of prior state), just via an
        // early return instead of a hardware no-op, since there's no
        // register block resolved to write to yet in that case.
        let Some(open) = self.open.take() else {
            return;
        };
        open.block.stop();
    }

    fn position(&self) -> UnitInterval {
        let open = self
            .open
            .as_ref()
            .expect("position() called before open() succeeded");
        let raw = open.block.position_raw() as u64;
        // `encoder_counts` a power of two is common in practice (encoder
        // resolutions usually are) and cheaper than even the
        // reciprocal-multiply division `open.divider` otherwise does: a
        // single shift. `raw < encoder_counts` always (the counter wraps
        // at `encoder_counts - 1`), so neither path loses bits or exceeds
        // `u32::MAX`.
        if open.encoder_counts.is_power_of_two() {
            let shift = 32 - open.encoder_counts.trailing_zeros();
            UnitInterval::new((raw << shift) as u32)
        } else {
            UnitInterval::new(open.divider.divide(raw << 32) as u32)
        }
    }
}

/// `stm32-metapac` names this enum variant `TI4` — generated once from
/// whichever CCMR register's field docs the code generator happened to
/// see first (CCMR2's IC3/IC4 mapping) and then reused verbatim for CCMR1,
/// since the bit layout (and so the enum) is identical either way. What it
/// actually selects here, applied to CCMR1's CC1S/CC2S fields (see
/// [`TimerBlock::configure_encoder_mode`]), is RM0440's "CC1 channel is
/// configured as input, IC1 is mapped on TI1" / the same for IC2 on TI2 —
/// i.e. the direct (non-swapped) mapping.
const DIRECT_INPUT_MAPPING: vals::CcmrInputCcs = vals::CcmrInputCcs::TI4;

/// Same generated-naming quirk as [`DIRECT_INPUT_MAPPING`] — `TI3` here is
/// RM0440's "CC1 channel is configured as input, IC1 is mapped on TI2" (and
/// the same swapped for IC2/TI1): the alternate mapping, which is exactly
/// the mechanism [`QuadratureInputConfiguration::Ch12AreInputsBA`] needs to
/// swap which physical channel is treated as input A vs B.
const ALTERNATE_INPUT_MAPPING: vals::CcmrInputCcs = vals::CcmrInputCcs::TI3;

/// The two register layouts a timer [`timer_block`] can hand back:
/// `TimAdv` for every 16-bit timer this driver supports (`TIM1`/`TIM3`/
/// `TIM4`/`TIM8`/`TIM20` — see this module's doc comment for why `TIM3`/
/// `TIM4` show up here despite really being `stm32-metapac`'s `TimGp16`),
/// and, only when built with a chip feature that has `TIM5` (see this
/// module's doc comment), `TimGp32` for its genuinely 32-bit counter/
/// auto-reload registers. On the default `stm32g431cb` chip feature (or any
/// other one without `TIM5`) this collapses to a single-variant enum
/// around `TimAdv` alone — no match/branching overhead at all.
#[derive(Clone, Copy)]
enum TimerBlock {
    Adv(TimAdv),
    #[cfg(feature = "stm32g474re")]
    Gp32(TimGp32),
}

impl TimerBlock {
    /// Configures encoder mode 3 (count on every edge of both inputs,
    /// direction from the other input's level — see this module's doc
    /// comment) and starts the counter. `mapping` selects which physical
    /// channel carries input A vs B (see
    /// [`QuadratureInputConfiguration`]/[`DIRECT_INPUT_MAPPING`]/
    /// [`ALTERNATE_INPUT_MAPPING`]); `encoder_counts` becomes the counter's
    /// wrap point (`ARR = encoder_counts - 1`).
    fn configure_encoder_mode(&self, mapping: vals::CcmrInputCcs, encoder_counts: u32) {
        match self {
            TimerBlock::Adv(t) => {
                assert!(
                    (1..=(1 << 16)).contains(&encoder_counts),
                    "16-bit timer: encoder_counts must be in 1..=65536, got {encoder_counts}"
                );
                t.ccmr_input(0).write(|w| {
                    w.set_ccs(0, mapping);
                    w.set_ccs(1, mapping);
                });
                // Non-inverted, rising-edge-active on both inputs — see
                // this module's doc comment for the direction convention
                // this selects.
                t.ccer().write(|w| {
                    w.set_ccp(0, false);
                    w.set_ccnp(0, false);
                    w.set_ccp(1, false);
                    w.set_ccnp(1, false);
                });
                t.smcr().write(|w| w.set_sms(vals::Sms::ENCODER_MODE_3));
                t.cnt().write(|w| w.set_cnt(0));
                t.arr().write(|w| w.set_arr((encoder_counts - 1) as u16));
                t.cr1().write(|w| w.set_cen(true));
            }
            #[cfg(feature = "stm32g474re")]
            TimerBlock::Gp32(t) => {
                assert!(encoder_counts >= 1, "encoder_counts must be at least 1");
                t.ccmr_input(0).write(|w| {
                    w.set_ccs(0, mapping);
                    w.set_ccs(1, mapping);
                });
                t.ccer().write(|w| {
                    w.set_ccp(0, false);
                    w.set_ccnp(0, false);
                    w.set_ccp(1, false);
                    w.set_ccnp(1, false);
                });
                t.smcr().write(|w| w.set_sms(vals::Sms::ENCODER_MODE_3));
                t.cnt().write_value(0);
                t.arr().write_value(encoder_counts - 1);
                t.cr1().write(|w| w.set_cen(true));
            }
        }
    }

    /// The counter's current raw value (`0..encoder_counts`).
    fn position_raw(&self) -> u32 {
        match self {
            TimerBlock::Adv(t) => t.cnt().read().cnt() as u32,
            #[cfg(feature = "stm32g474re")]
            TimerBlock::Gp32(t) => t.cnt().read(),
        }
    }

    /// Stops the counter — see [`Quadrature::close`].
    fn stop(&self) {
        match self {
            TimerBlock::Adv(t) => t.cr1().modify(|w| w.set_cen(false)),
            #[cfg(feature = "stm32g474re")]
            TimerBlock::Gp32(t) => t.cr1().modify(|w| w.set_cen(false)),
        }
    }
}

/// Enables `timer`'s peripheral clock and maps it to its `stm32-metapac`
/// register block, reinterpreted as a [`TimAdv`] regardless of which
/// concrete register-block type `stm32-metapac` really generated for it
/// (`TIM1`/`TIM8`/`TIM20` genuinely are `TimAdv`; `TIM3`/`TIM4` are really
/// `TimGp16`) — mirrors [`crate::stm32g4::gpio::gpio_block`]/
/// [`crate::stm32g4::gpio::enable_gpio_clocks`] for timers.
///
/// Reinterpreting `TIM3`/`TIM4` as `TimAdv` is sound because every register
/// this driver actually touches (`CR1`, `SMCR`, `CCMR1`, `CCER`, `CNT`,
/// `ARR`) sits at the identical byte offset in both `TimAdv` and `TimGp16`
/// (RM0440's timer register map places them there for every classic STM32
/// timer, advanced-control or general-purpose alike), and every field this
/// driver reads or writes on those registers is one `TimGp16` genuinely has
/// too — `TimAdv`'s `SmcrAdv`/`CcerAdv` types are strict supersets of
/// `TimGp16`'s `SmcrGp16`/`CcerGp16` (one extra `occs` bit in `SmcrAdv`;
/// channels 5/6 in `CcerAdv`, vs. `CcerGp16`'s 4), and this driver never
/// reads or writes any of that extra surface — every register write here
/// goes through `.write()` (not `.modify()`), which starts from an
/// all-zero value and only ever sets fields common to both types, so the
/// bits that only exist in `TimAdv`'s view end up written `0` — matching a
/// real `TimGp16`'s own reserved-bit convention (kept at their `0` reset
/// value) exactly.
///
/// `TIM5` (only reachable when built with a chip feature that has it — see
/// this module's doc comment) gets no such treatment: it's genuinely
/// 32-bit (`TimGp32`) on chips that have it, with its own `CNT`/`ARR`
/// register shape, so it's handled as its own [`TimerBlock::Gp32`] variant
/// instead.
///
/// Returns `None` for `TIM5`/`TIM20` on a chip feature that doesn't have
/// them — including the default `stm32g431cb` this crate targets, which
/// has neither timer at all, so `stm32-metapac` doesn't even generate a
/// `TIM5`/`TIM20` register block for it (same situation as
/// [`crate::api::gpio::GpioPort::PH`]/`PI`/`PJ` in
/// [`crate::stm32g4::gpio::gpio_block`]) — see [`QuadratureTrait::open`]
/// for how that's surfaced as a logged, non-panicking failure rather than
/// this function panicking the way it used to.
fn timer_block(timer: QuadratureTimer) -> Option<TimerBlock> {
    match timer {
        QuadratureTimer::Stm32g4Tim1 => {
            stm32_metapac::RCC.apb2enr().modify(|w| w.set_tim1en(true));
            Some(TimerBlock::Adv(stm32_metapac::TIM1))
        }
        QuadratureTimer::Stm32g4Tim3 => {
            stm32_metapac::RCC.apb1enr1().modify(|w| w.set_tim3en(true));
            // SAFETY: see this function's doc comment — TIM3 is really a
            // `TimGp16`, but every register/field this driver uses through
            // the resulting `TimAdv` view is one `TimGp16` genuinely has,
            // at the same offset.
            Some(TimerBlock::Adv(unsafe {
                TimAdv::from_ptr(stm32_metapac::TIM3.as_ptr())
            }))
        }
        QuadratureTimer::Stm32g4Tim4 => {
            stm32_metapac::RCC.apb1enr1().modify(|w| w.set_tim4en(true));
            // SAFETY: see this function's doc comment — same as TIM3 above.
            Some(TimerBlock::Adv(unsafe {
                TimAdv::from_ptr(stm32_metapac::TIM4.as_ptr())
            }))
        }
        QuadratureTimer::Stm32g4Tim8 => {
            stm32_metapac::RCC.apb2enr().modify(|w| w.set_tim8en(true));
            Some(TimerBlock::Adv(stm32_metapac::TIM8))
        }
        QuadratureTimer::Stm32g4Tim5 => {
            #[cfg(feature = "stm32g474re")]
            {
                stm32_metapac::RCC.apb1enr1().modify(|w| w.set_tim5en(true));
                Some(TimerBlock::Gp32(stm32_metapac::TIM5))
            }
            #[cfg(not(feature = "stm32g474re"))]
            {
                None
            }
        }
        QuadratureTimer::Stm32g4Tim20 => {
            #[cfg(feature = "stm32g474re")]
            {
                stm32_metapac::RCC.apb2enr().modify(|w| w.set_tim20en(true));
                Some(TimerBlock::Adv(stm32_metapac::TIM20))
            }
            #[cfg(not(feature = "stm32g474re"))]
            {
                None
            }
        }
    }
}
