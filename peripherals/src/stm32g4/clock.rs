//! STM32G4 cycle-counter-based [`ClockProviderTrait`]/[`ClockTrait`]
//! driver.
//!
//! Backed by the Cortex-M4 DWT free-running cycle counter (`DWT->CYCCNT`),
//! which increments once per core (`HCLK`) clock cycle — so this driver's
//! [`ClockTrait::ticks_per_second`] is whatever `HCLK` frequency the
//! caller tells [`ClockProvider::new`] the chip is actually running at.
//! This driver doesn't configure (or otherwise know) the clock tree
//! itself — different boards reach different `HCLK` frequencies from
//! different oscillators, so that's the caller's responsibility (see
//! `boards/esc1_discovery::initialize`).
//!
//! This whole module is gated to `cfg(target_arch = "arm")` at its `mod`
//! declaration in `peripherals/src/lib.rs`.

use core::cell::UnsafeCell;

use common::duration_from_ticks::DurationFromTicks;
use common::uptime::Uptime;
use cortex_m::peripheral::DWT;

use crate::api::clock::{ClockProviderTrait, ClockTrait};

/// Owns the reference point every [`Clock`] view reads from. Create one
/// (via [`Self::new`]) once at startup, call
/// [`Self::advance_reference_point`] periodically (e.g. from a timer
/// interrupt), and hand out [`Clock`] views with [`Self::get_clock`].
pub struct ClockProvider {
    ticks_per_second: u32,
    /// Written (through an exclusive reference) only in
    /// [`Self::advance_reference_point`]; read (through a shared
    /// reference) only in [`Clock::time_at`]. `UnsafeCell` rather than
    /// `RefCell` to avoid a runtime borrow check on real hardware:
    /// `DurationFromTicks`'s own double-buffered `advance_to`/`time_at`
    /// (see its doc comment) already guarantees a read never observes a
    /// torn write, even one from an interrupt preempting (or preempted
    /// by) a write in progress — that guarantee is exactly what makes an
    /// aliased shared/exclusive access here sound without an additional
    /// runtime check.
    reference: UnsafeCell<DurationFromTicks>,
}

impl ClockProvider {
    /// Starts the Cortex-M4 DWT cycle counter that [`ClockTrait::ticks_now`]
    /// reads, and records `mcu_frequency` — the `HCLK` frequency the DWT
    /// counter actually increments at, i.e. whatever the caller already
    /// configured the clock tree to reach (see
    /// `boards/esc1_discovery::initialize`'s `MCU_FREQUENCY` constant) —
    /// so [`ClockTrait::ticks_per_second`] can report it without this
    /// driver needing to read back `RCC`'s clock-tree configuration
    /// itself, which would bake in assumptions about a specific board's
    /// oscillator that don't hold for every board this driver might run
    /// on.
    ///
    /// Borrows the whole [`cortex_m::Peripherals`] singleton (the caller —
    /// see `boards/esc1_discovery::initialize` — obtains it via the safe
    /// [`cortex_m::Peripherals::take`], not `steal()`) rather than just the
    /// `DCB`/`DWT` fields this constructor happens to need today, so a
    /// future revision needing another core peripheral doesn't change what
    /// the caller has to pass. Borrowed rather than consumed since nothing
    /// here needs to keep it: enabling trace/the cycle counter is a
    /// one-time setup step, and the caller may still need `DCB`/`DWT` (or
    /// other core peripherals) itself afterwards.
    pub fn new(peripherals: &mut cortex_m::Peripherals, mcu_frequency: u32) -> Self {
        // Keeps the DWT's clock domain alive across the CPU's low-power
        // sleep modes, so a `WFI`-based idle loop elsewhere in the
        // firmware doesn't silently freeze `ticks_now()`.
        stm32_metapac::DBGMCU.cr().modify(|w| {
            w.set_dbg_sleep(true);
            w.set_dbg_stop(true);
            w.set_dbg_standby(true);
        });

        peripherals.DCB.enable_trace();
        peripherals.DWT.enable_cycle_counter();

        ClockProvider {
            ticks_per_second: mcu_frequency,
            reference: UnsafeCell::new(DurationFromTicks::new(mcu_frequency)),
        }
    }
}

impl<'a> ClockProviderTrait<'a, Clock<'a>> for ClockProvider {
    fn get_clock(&'a self) -> Clock<'a> {
        Clock {
            ticks_per_second: self.ticks_per_second,
            reference: &self.reference,
        }
    }

    fn advance_reference_point(&self) {
        // SAFETY: see `Self::reference`'s doc comment — this is the only
        // place an exclusive reference to it is taken, held only for the
        // duration of this call.
        let reference = unsafe { &mut *self.reference.get() };
        reference.advance_to(DWT::cycle_count());
    }
}

/// A lightweight, read-only [`ClockTrait`] view onto a [`ClockProvider`] —
/// see [`ClockProvider::get_clock`].
pub struct Clock<'a> {
    ticks_per_second: u32,
    reference: &'a UnsafeCell<DurationFromTicks>,
}

impl<'a> ClockTrait for Clock<'a> {
    fn ticks_now(&self) -> u32 {
        DWT::cycle_count()
    }

    fn time_at(&self, ticks: u32) -> Uptime {
        // SAFETY: see `ClockProvider::reference`'s doc comment — this is
        // the only place a shared reference to it is taken, held only for
        // the duration of this call.
        let reference = unsafe { &*self.reference.get() };
        Uptime::epoch() + reference.time_at(ticks)
    }

    fn ticks_per_second(&self) -> u32 {
        self.ticks_per_second
    }
}
