//! A register-level [`PwmTrait`](crate::api::pwm::PwmTrait) driver for a
//! real STM32G4 chip's advanced-control timer, backed by `stm32-metapac`.
//!
//! This whole module only compiles for `target_arch = "arm"` — the
//! register access it does needs real hardware, and doesn't build (or
//! make sense) anywhere else.
//!
//! [`PwmTrait::open`] always configures the counter for center-aligned
//! (up/down) counting (RM0440's `CMS` = `01`), channels 1-3 (0-indexed 0-2
//! here) for PWM mode 1 with both the main and complementary output
//! enabled, and the break/dead-time generator (`BDTR`) for the requested
//! dead-time.
//!
//! # The mid-point trigger
//! [`PwmOptions::mid_point_trigger`] is built on channel 5 — present on
//! every [`PwmTimer`], but, unlike channels 1-4, wired to no physical pin
//! at all (RM0440), so using it here never contends with
//! [`PwmTrait::set_duty_cycle`]'s channels. It's configured for PWM mode 2
//! (the inverse of the PWM mode 1 channels 1-3 use) with its compare value
//! pinned [`MID_POINT_TRIGGER_HALF_WIDTH_TICKS`] short of `ARR`, not at
//! `ARR` itself: channel 5's internal reference signal (OC5REF) is then
//! active for `2 * MID_POINT_TRIGGER_HALF_WIDTH_TICKS + 1` counter ticks
//! symmetric around the counter's turnaround (`CNT == ARR`), not for a
//! single tick — see that constant's own doc comment for why a wider pulse
//! is needed. `TRGO2` (`CR2.MMS2`) is then routed to mirror it.
//!
//! # Which timers this driver actually supports: `TIM1`/`TIM8` are
//! unconditionally available registers on every chip feature this crate
//! targets, but `TIM20` only exists on chip features that have it (e.g.
//! `stm32g474re`) — see [`timer_block`].

use common::duration::Duration;
use common::unit_interval::UnitInterval;
use stm32_metapac::timer::{vals, TimAdv};

use crate::api::pwm::{PwmOptions, PwmTimer, PwmTrait};

/// How many counter ticks short of `ARR` [`PwmTrait::open`] pins the
/// mid-point trigger's `CCR5` at, on each side of the counter's turnaround
/// -- see this module's "the mid-point trigger" doc comment.
///
/// A single-tick-wide OC5REF/TRGO2 pulse (`CCR5 == ARR` exactly) measured
/// correct on every TIM1 register this driver touches (`CC5E`, `CR2.MMS2`,
/// `CCMR3.OC5M`, `ARR`/`CCR5`) but never actually started a conversion on
/// real B-G431B-ESC1 hardware, while plain TRGO (the update event, which
/// holds a recognizable state rather than a single-tick pulse) triggered
/// immediately -- consistent with the pulse being too narrow for the ADC's
/// own clock domain (`SYNC_DIV4`, a quarter of this timer's clock) to
/// reliably catch: RM0440 documents no minimum OC5REF pulse width for
/// TRGO2 detection, but a source-domain pulse shorter than one destination
/// clock period is a textbook clock-domain-crossing hazard. 8 ticks (a
/// `2*8+1 = 17`-tick, ~100ns pulse at 170MHz) was confirmed on hardware to
/// fire reliably; at the 20kHz-class PWM frequencies this driver targets
/// (`ARR` in the low thousands), shifting the trigger 8 ticks (order of
/// 10s of ns) before the true peak is negligible against the ADC's own
/// sample-and-conversion time.
const MID_POINT_TRIGGER_HALF_WIDTH_TICKS: u16 = 8;

/// Register-level [`PwmTrait`] driver for a real STM32G4 chip's
/// advanced-control timer, backed by `stm32-metapac`.
pub struct Pwm {
    timer: PwmTimer,
    /// The clock this driver's timer counts at (before its own prescaler),
    /// in Hz — needed to turn [`PwmOptions::frequency_hz`]/
    /// [`PwmOptions::dead_time`] into register values (see
    /// [`compute_psc_arr`]/[`dead_time_dtg`]). This driver doesn't
    /// configure or otherwise know the clock tree itself, so the caller
    /// must supply the timer's actual running frequency.
    timer_clock_hz: u32,
    /// Set by [`PwmTrait::open`] once it's resolved and configured
    /// `timer`'s register block, cleared by [`PwmTrait::close`]. `None`
    /// means "not open" — every other [`PwmTrait`] method needs it to be
    /// `Some` (see [`PwmTrait::set_duty_cycle`]'s panic).
    open: Option<OpenState>,
}

/// `Pwm`'s state while open — see [`Pwm::open`].
struct OpenState {
    block: TimAdv,
    /// The channel count [`PwmTrait::open`] configured — the exclusive
    /// upper bound [`PwmTrait::set_duty_cycle`] checks `channel` against.
    channels: u8,
    /// The auto-reload value [`PwmTrait::open`] computed — the counter's
    /// peak, and so also [`PwmTrait::set_duty_cycle`]'s "always on" `CCR`
    /// value (see its doc comment).
    arr: u16,
}

impl Pwm {
    /// Records which physical timer instance `open()` should later claim
    /// (see [`PwmTimer`]) and the clock frequency that timer counts at —
    /// doesn't touch any hardware yet (not even to check `timer` is
    /// actually available on this chip): see [`Pwm::open`].
    pub fn new(timer: PwmTimer, timer_clock_hz: u32) -> Self {
        Pwm {
            timer,
            timer_clock_hz,
            open: None,
        }
    }
}

impl PwmTrait for Pwm {
    /// Enables the timer's peripheral clock, maps it to its register
    /// block, and configures it per `options` — see this module's doc
    /// comment.
    ///
    /// Fails (logs an error via `defmt` and returns, leaving this driver
    /// unopened — see [`PwmTrait::set_duty_cycle`]'s panic) if the timer
    /// given to [`Pwm::new`] isn't available — see this module's doc
    /// comment for when that happens.
    fn open(&mut self, options: PwmOptions) {
        let Some(block) = timer_block(self.timer) else {
            defmt::error!(
                "pwm: TIM20 isn't available on this chip feature -- see \
                 peripherals/Cargo.toml's chip-variant features -- open() failed"
            );
            return;
        };

        // Stop any previous configuration's output before reconfiguring,
        // so a second open() call can't glitch the physical pins
        // mid-reconfiguration.
        block.bdtr().write(|w| w.set_moe(false));
        block.cr1().write(|w| w.set_cen(false));

        let (psc, arr) = compute_psc_arr(self.timer_clock_hz, options.frequency_hz());
        let dtg = dead_time_dtg(self.timer_clock_hz, options.dead_time());

        block.psc().write_value(psc);
        block.arr().write(|w| w.set_arr(arr));

        // CCMR1 (channels 1-2) always needs writing; CCMR2 (channels 3-4)
        // only when a third channel was requested. Each channel's OCxPE
        // shadows its CCR (see below) so set_duty_cycle() never glitches
        // an in-flight pulse.
        block.ccmr_output(0).write(|w| {
            w.set_ocm(0, vals::Ocm::PWM_MODE1);
            w.set_ocpe(0, true);
            if options.channels() > 1 {
                w.set_ocm(1, vals::Ocm::PWM_MODE1);
                w.set_ocpe(1, true);
            }
        });
        if options.channels() > 2 {
            block.ccmr_output(1).write(|w| {
                w.set_ocm(0, vals::Ocm::PWM_MODE1);
                w.set_ocpe(0, true);
            });
        }

        // Enable each configured channel's main (CCxE) and complementary
        // (CCxNE) outputs, both active-high -- dead time (below) is
        // inserted between the two by the break/dead-time generator, not
        // by any polarity trick here. Channels beyond options.channels()
        // are left disabled (this register's reset value).
        //
        // Channel 5 (index 4) has no physical pin at all (see this
        // module's "mid-point trigger" doc comment) and so no
        // complementary output either -- `set_ccne` panics if asked for
        // one (`assert!(n < 3)`, real for channels 1-3 only) -- but its
        // own CC5E still gates whether OC5REF is generated at all, same
        // as CC1E-CC4E do for their own channels' OCxREF (RM0440's CCER
        // description covers x=1-6 uniformly). Left unset, TRGO2
        // (mirroring OC5REF once `mid_point_trigger` configures CR2.MMS2
        // below) never actually pulses even though CCMR3/CCR5 are
        // otherwise fully configured -- this is `mid_point_trigger`'s
        // actual enable step, not the `ccmr3()`/`ccr5()` writes below.
        block.ccer().write(|w| {
            for channel in 0..options.channels() {
                let n = channel as usize;
                w.set_ccp(n, false);
                w.set_ccnp(n, false);
                w.set_cce(n, true);
                w.set_ccne(n, true);
            }
            if options.mid_point_trigger() {
                w.set_ccp(4, false);
                w.set_cce(4, true);
            }
        });

        for channel in 0..options.channels() {
            block.ccr(channel as usize).write(|w| w.set_ccr(0));
        }

        if options.mid_point_trigger() {
            // See this module's doc comment's "the mid-point trigger"
            // section, and `MID_POINT_TRIGGER_HALF_WIDTH_TICKS`'s own doc
            // comment for why `CCR5` is pinned short of `ARR` rather than
            // at it.
            block.ccmr3().write(|w| w.set_ocm(0, vals::Ocm::PWM_MODE2));
            block
                .ccr5()
                .write(|w| w.set_ccr(arr.saturating_sub(MID_POINT_TRIGGER_HALF_WIDTH_TICKS)));
        }

        // BDTR: dead time, plus the main output enable every physical
        // channel above needs to actually drive its pin.
        block.bdtr().write(|w| {
            w.set_dtg(dtg);
            w.set_moe(true);
        });

        // CR2: TRGO fires on the update event unconditionally; TRGO2
        // mirrors channel 5's internal reference signal only when the
        // caller asked for the mid-point trigger, otherwise stays at its
        // harmless reset default (never asserted by the counter itself).
        block.cr2().write(|w| {
            w.set_mms(vals::Mms::UPDATE);
            if options.mid_point_trigger() {
                w.set_mms2(vals::Mms2::COMPARE_OC5);
            }
        });

        // Forces the CCR/ARR shadow registers just written (see the
        // OCxPE/ARPE preload bits) into their active registers before the
        // first period starts, so it already reflects them instead of
        // whatever was left over from before this UG.
        block.egr().write(|w| w.set_ug(true));

        // CR1 last: center-aligned counting, auto-reload preload (arpe,
        // matching every channel's ocpe above), and finally start the
        // counter.
        block.cr1().write(|w| {
            w.set_cms(vals::Cms::CENTER_ALIGNED1);
            w.set_arpe(true);
            w.set_cen(true);
        });

        self.open = Some(OpenState {
            block,
            channels: options.channels(),
            arr,
        });
    }

    fn close(&mut self) {
        // Nothing to stop if `open()` never succeeded (or already failed)
        // — safe and idempotent to call regardless of prior state.
        let Some(open) = self.open.take() else {
            return;
        };
        open.block.bdtr().write(|w| w.set_moe(false));
        open.block.cr1().write(|w| w.set_cen(false));
    }

    /// # Panics
    /// Panics if called before [`Self::open`] has succeeded, or if
    /// `channel` is at or beyond the channel count [`Self::open`] was
    /// configured with.
    fn set_duty_cycle(&mut self, channel: u8, value: UnitInterval) {
        let open = self
            .open
            .as_ref()
            .expect("set_duty_cycle() called before open() succeeded");
        assert!(
            channel < open.channels,
            "channel must be less than the {} channels open() configured, got {channel}",
            open.channels
        );
        // Scales value's full u32 range onto 0..=arr -- arr is the
        // counter's peak, so ccr == arr means "on for the entire period"
        // and ccr == 0 means "off for the entire period", matching PWM
        // mode 1's convention on the channels open() configured.
        let ccr = ((value.raw() as u64 * open.arr as u64) >> 32) as u16;
        open.block.ccr(channel as usize).write(|w| w.set_ccr(ccr));
    }

    /// `.modify()`, not `.write()` — [`Self::open`] packs `DTG` and `MOE`
    /// into the same `BDTR` write, and a plain `.write()` here would reset
    /// `MOE` back to `false`, cutting every output.
    ///
    /// # Panics
    /// Panics if called before [`Self::open`] has succeeded.
    fn set_dead_time(&mut self, dead_time: Duration) {
        let open = self
            .open
            .as_ref()
            .expect("set_dead_time() called before open() succeeded");
        let dtg = dead_time_dtg(self.timer_clock_hz, dead_time);
        open.block.bdtr().modify(|w| w.set_dtg(dtg));
    }
}

/// Finds `(psc, arr)` — [`TimAdv`]'s prescaler and auto-reload register
/// values — reaching as close to `frequency_hz` as a 16-bit `arr` allows,
/// given the timer's `timer_clock_hz` input clock, for the center-aligned
/// (up/down counting) mode [`PwmTrait::open`] always uses: RM0440's f_PWM =
/// f_CK_CNT / (2 * ARR), where f_CK_CNT = timer_clock_hz / (psc + 1).
/// Starts at `psc = 0` (the finest possible `arr` resolution) and only
/// grows `psc` as far as needed for `arr` to fit 16 bits, rounding `arr` to
/// the nearest representable value at each `psc` rather than always
/// truncating down (which would silently run faster than asked).
///
/// # Panics
/// Panics if `frequency_hz` is `0`, or so high or low that no 16-bit `psc`
/// reaches a representable, nonzero `arr`.
fn compute_psc_arr(timer_clock_hz: u32, frequency_hz: u32) -> (u16, u16) {
    assert!(frequency_hz > 0, "frequency_hz must be nonzero");
    let mut psc: u32 = 0;
    loop {
        let denom = 2 * frequency_hz as u64 * (psc as u64 + 1);
        let arr = (timer_clock_hz as u64 + denom / 2) / denom;
        if (1..=u16::MAX as u64).contains(&arr) {
            return (psc as u16, arr as u16);
        }
        assert!(
            psc < u16::MAX as u32,
            "frequency_hz {frequency_hz} isn't representable at timer_clock_hz {timer_clock_hz}"
        );
        psc += 1;
    }
}

/// Encodes `dead_time` into `BDTR`'s 8-bit `DTG` field (RM0440's
/// break/dead-time register), given the dead-time generator's own clock
/// `t_DTS` — `1 / timer_clock_hz`, since this driver never touches
/// `CR1.CKD` (left at its reset value of `0`, no further division of
/// `t_DTS` from the timer's own input clock).
///
/// `DTG`'s encoding covers four increasingly coarse ranges (RM0440's
/// `BDTR` register description): this picks the finest range that can
/// reach the requested `dead_time` exactly (in units of `t_DTS`), and
/// otherwise the coarsest range's nearest representable value, saturating
/// at its maximum (1008 `t_DTS`, e.g. ~5.9us at a 170MHz `timer_clock_hz`)
/// if `dead_time` asks for more than `DTG` can represent at all.
fn dead_time_dtg(timer_clock_hz: u32, dead_time: Duration) -> u8 {
    // `dead_time`'s raw value packs whole seconds into its upper 32 bits
    // and a fraction of a second into its lower 32 bits, i.e. `seconds *
    // 2^32`. Multiplying by timer_clock_hz (counts/second) and shifting
    // back down by 32 bits gives a count of t_DTS ticks directly, with no
    // separate unit conversion needed. `.max(0)`/`saturating_mul` guard
    // against a negative or overflowing dead_time rather than panicking or
    // wrapping.
    let counts = (dead_time.raw().max(0) as u64).saturating_mul(timer_clock_hz as u64) >> 32;
    if counts <= 127 {
        // DTG[7:5] = 0xx: DT = DTG[7:0] * t_DTS, 0..127 in steps of 1.
        counts as u8
    } else if counts <= 254 {
        // DTG[7:5] = 10x: DT = (64 + DTG[5:0]) * 2 * t_DTS, 128..254 in
        // steps of 2.
        let n = (counts / 2).saturating_sub(64).min(63);
        0b1000_0000 | n as u8
    } else if counts <= 504 {
        // DTG[7:5] = 110: DT = (32 + DTG[4:0]) * 8 * t_DTS, 256..504 in
        // steps of 8.
        let n = (counts / 8).saturating_sub(32).min(31);
        0b1100_0000 | n as u8
    } else {
        // DTG[7:5] = 111: DT = (32 + DTG[4:0]) * 16 * t_DTS, 512..1008 in
        // steps of 16 -- the coarsest range, so also where an
        // out-of-range dead_time saturates (n capped at 31).
        let n = (counts / 16).saturating_sub(32).min(31);
        0b1110_0000 | n as u8
    }
}

/// Enables `timer`'s peripheral clock and maps it to its `stm32-metapac`
/// register block. `TIM1`/`TIM8` are genuinely [`TimAdv`] on every chip
/// feature this crate targets; `TIM20` needs one that actually has it.
///
/// Returns `None` for `TIM20` on a chip feature that doesn't have it,
/// including the default `stm32g431cb` this crate targets — [`PwmTrait::open`]
/// turns that into a logged failure that leaves the driver unopened,
/// rather than a panic.
fn timer_block(timer: PwmTimer) -> Option<TimAdv> {
    match timer {
        PwmTimer::Stm32g4Tim1 => {
            stm32_metapac::RCC.apb2enr().modify(|w| w.set_tim1en(true));
            Some(stm32_metapac::TIM1)
        }
        PwmTimer::Stm32g4Tim8 => {
            stm32_metapac::RCC.apb2enr().modify(|w| w.set_tim8en(true));
            Some(stm32_metapac::TIM8)
        }
        PwmTimer::Stm32g4Tim20 => {
            #[cfg(feature = "stm32g474re")]
            {
                stm32_metapac::RCC.apb2enr().modify(|w| w.set_tim20en(true));
                Some(stm32_metapac::TIM20)
            }
            #[cfg(not(feature = "stm32g474re"))]
            {
                None
            }
        }
    }
}
