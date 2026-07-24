//! STM32G4 cycle-counter-based [`ClockProviderTrait`]/[`ClockTrait`]
//! driver.
//!
//! Backed by the Cortex-M4 DWT free-running cycle counter (`DWT->CYCCNT`),
//! which increments once per core (`HCLK`) clock cycle — so this driver's
//! [`ClockTrait::ticks_per_second`] is the chip's actual running `HCLK`
//! frequency, read from `RCC`'s clock-tree configuration at construction
//! time.

#[cfg(target_arch = "arm")]
use core::cell::RefCell;

#[cfg(target_arch = "arm")]
use common::duration_from_ticks::DurationFromTicks;
#[cfg(target_arch = "arm")]
use common::uptime::Uptime;
#[cfg(target_arch = "arm")]
use cortex_m::peripheral::DWT;
#[cfg(target_arch = "arm")]
use stm32_metapac::rcc::vals;

#[cfg(target_arch = "arm")]
use crate::api::clock::{ClockProviderTrait, ClockTrait};

#[cfg(target_arch = "arm")]
const HSI_FREQUENCY: u32 = 16_000_000;
/// This board's HSE crystal — see the `PF0`/`PF1` doc comments in
/// `boards/esc1_discovery/board.rs` ("HSE crystal input, 8 MHz").
#[cfg(target_arch = "arm")]
const HSE_FREQUENCY: u32 = 8_000_000;

/// Owns the reference point every [`Clock`] view reads from. Create one
/// (via [`Self::new`]) once at startup, call
/// [`Self::advance_reference_point`] periodically (e.g. from a timer
/// interrupt), and hand out [`Clock`] views with [`Self::get_clock`].
#[cfg(target_arch = "arm")]
pub struct ClockProvider {
    ticks_per_second: u32,
    reference: RefCell<DurationFromTicks>,
}

#[cfg(target_arch = "arm")]
impl ClockProvider {
    /// Determines the running core (`HCLK`) clock frequency from `RCC`'s
    /// clock-tree configuration, and starts the Cortex-M4 DWT cycle counter
    /// that [`ClockTrait::ticks_now`] reads.
    ///
    /// `rcc` and `dbgmcu` are borrowed, not consumed: like
    /// [`enable_gpio_clocks`](crate::stm32g4::gpio::enable_gpio_clocks),
    /// the register access below goes straight through the `stm32-metapac`
    /// singletons, which doesn't need ownership of either — their role here
    /// is just to prove the caller already brought the chip up (so the
    /// clock tree these reads depend on is actually configured), while
    /// letting the caller keep using them elsewhere. `dbgmcu` specifically
    /// is used to keep the DWT's clock domain alive across the CPU's
    /// low-power sleep modes (`DBG_SLEEP`/`DBG_STOP`/`DBG_STANDBY`), so a
    /// `WFI`-based idle loop elsewhere in the firmware doesn't silently
    /// freeze `ticks_now()`; enabling the counter itself is unrelated to
    /// `DBGMCU` and goes through the Cortex-M core's own DCB/DWT
    /// peripherals instead (`cortex_m::Peripherals` isn't threaded through
    /// `embassy_stm32::init()`, so those are stolen here).
    pub fn new<R, D>(_rcc: &R, _dbgmcu: &D) -> Self {
        let ticks_per_second = hclk_frequency();

        stm32_metapac::DBGMCU.cr().modify(|w| {
            w.set_dbg_sleep(true);
            w.set_dbg_stop(true);
            w.set_dbg_standby(true);
        });

        // SAFETY: nothing else in this codebase takes `cortex_m::Peripherals`
        // (`embassy_stm32::init()` only takes `embassy_stm32::Peripherals`, a
        // distinct singleton); enabling trace and the cycle counter is also
        // idempotent, so stealing repeatedly is harmless.
        let mut core = unsafe { cortex_m::Peripherals::steal() };
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        ClockProvider {
            ticks_per_second,
            reference: RefCell::new(DurationFromTicks::new(ticks_per_second)),
        }
    }
}

#[cfg(target_arch = "arm")]
impl<'a> ClockProviderTrait<'a, Clock<'a>> for ClockProvider {
    fn get_clock(&'a self) -> Clock<'a> {
        Clock {
            ticks_per_second: self.ticks_per_second,
            reference: &self.reference,
        }
    }

    fn advance_reference_point(&self) {
        self.reference.borrow_mut().advance_to(DWT::cycle_count());
    }
}

/// A lightweight, read-only [`ClockTrait`] view onto a [`ClockProvider`] —
/// see [`ClockProvider::get_clock`].
#[cfg(target_arch = "arm")]
pub struct Clock<'a> {
    ticks_per_second: u32,
    reference: &'a RefCell<DurationFromTicks>,
}

#[cfg(target_arch = "arm")]
impl<'a> ClockTrait for Clock<'a> {
    fn ticks_now(&self) -> u32 {
        DWT::cycle_count()
    }

    fn time_at(&self, ticks: u32) -> Uptime {
        Uptime::epoch() + self.reference.borrow().time_at(ticks)
    }

    fn ticks_per_second(&self) -> u32 {
        self.ticks_per_second
    }
}

/// Reads `RCC`'s clock-tree configuration registers to determine the core
/// (`HCLK`) clock frequency actually running right now — the frequency
/// [`ClockTrait::ticks_now`]'s DWT cycle counter increments at.
#[cfg(target_arch = "arm")]
fn hclk_frequency() -> u32 {
    let cfgr = stm32_metapac::RCC.cfgr().read();

    let sysclk = match cfgr.sws() {
        vals::Sw::HSI => HSI_FREQUENCY,
        vals::Sw::HSE => HSE_FREQUENCY,
        vals::Sw::PLL1_R => {
            let pllcfgr = stm32_metapac::RCC.pllcfgr().read();
            let source = match pllcfgr.pllsrc() {
                vals::Pllsrc::HSI => HSI_FREQUENCY,
                vals::Pllsrc::HSE => HSE_FREQUENCY,
                _ => unreachable!(
                    "PLL is the active SYSCLK source, so its input clock can't be disabled"
                ),
            };
            // PLLM is stored as (divisor - 1); PLLN is the multiplier
            // directly; PLLR is a 2-bit enum naming its four divisor
            // options (2/4/6/8) rather than storing them directly.
            let pllm = pllcfgr.pllm().to_bits() as u32 + 1;
            let plln = pllcfgr.plln().to_bits() as u32;
            let pllr = match pllcfgr.pllr() {
                vals::Pllr::DIV2 => 2,
                vals::Pllr::DIV4 => 4,
                vals::Pllr::DIV6 => 6,
                vals::Pllr::DIV8 => 8,
            };
            (source / pllm) * plln / pllr
        }
        _ => unreachable!("RCC only ever reports a valid SWS value"),
    };

    let hpre = match cfgr.hpre() {
        vals::Hpre::DIV1 => 1,
        vals::Hpre::DIV2 => 2,
        vals::Hpre::DIV4 => 4,
        vals::Hpre::DIV8 => 8,
        vals::Hpre::DIV16 => 16,
        vals::Hpre::DIV64 => 64,
        vals::Hpre::DIV128 => 128,
        vals::Hpre::DIV256 => 256,
        vals::Hpre::DIV512 => 512,
        _ => unreachable!("RCC only ever reports a valid HPRE value"),
    };

    sysclk / hpre
}
