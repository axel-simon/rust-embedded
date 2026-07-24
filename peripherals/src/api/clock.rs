//! Hardware-agnostic clock API. Concrete drivers (real or fake) implement
//! [`ClockTrait`] and [`ClockProviderTrait`]; nothing in this module
//! depends on any particular chip.
//!
//! The split between the two traits exists because reading a free-running
//! hardware tick counter and turning it into an exact [`Uptime`] needs a
//! reference point (a `Duration` as of a known tick count) to convert
//! against — and keeping that reference point fresh (via
//! [`ClockProviderTrait::advance_reference_point`]) is a distinct,
//! occasional, `&self`-mutating-through-interior-mutability operation from
//! reading it (via [`ClockTrait::now`]), which happens far more often and
//! needs no mutation at all. A single application-wide `ClockProvider` is
//! expected to be created once (with whatever a concrete implementation's
//! `new()` needs — real hardware peripherals, or nothing for the fake), and
//! [`ClockProviderTrait::get_clock`] hands out lightweight, read-only
//! [`ClockTrait`] views onto it wherever a `Duration`/`Uptime` reading is
//! needed.

use common::duration::Duration;
use common::uptime::Uptime;

/// Abstract interface implemented by every clock driver, real or fake.
/// Backed by a free-running hardware tick counter that wraps every 2^32
/// ticks. Every method here only ever needs `&self` — see the module doc
/// comment for why. [`Self::now`] and [`Self::wait_for`] come first since
/// they're the methods most callers actually want; the rest exist mainly
/// for those two (and other [`ClockTrait`] implementors) to build on.
pub trait ClockTrait {
    /// Returns the current [`Uptime`] (time elapsed since boot).
    fn now(&self) -> Uptime {
        self.time_at(self.ticks_now())
    }

    /// Busy-waits until [`Self::now`] has advanced by at least `delay` past
    /// where it stood when this was called.
    fn wait_for(&self, delay: Duration) {
        let deadline = self.now() + delay;
        while self.now() < deadline {}
    }

    /// Reads the current tick count of the free-running hardware counter.
    fn ticks_now(&self) -> u32;

    /// Converts a hardware tick count obtained through a call to
    /// [`Self::ticks_now`] into the [`Uptime`] it represents.
    fn time_at(&self, ticks: u32) -> Uptime;

    /// This clock's frequency in Hz — how many ticks (see
    /// [`Self::ticks_now`]) make up one second. Fixed for the lifetime of
    /// the driver.
    fn ticks_per_second(&self) -> u32;
}

/// Owns the reference point that every [`ClockTrait`] view produced by
/// [`Self::get_clock`] reads from. `Clock` is a generic parameter (rather
/// than an associated type) naming the concrete, borrowing view type each
/// implementation hands out, so calls through it are statically dispatched.
pub trait ClockProviderTrait<'a, Clock: ClockTrait + 'a> {
    /// Hands back a lightweight, read-only view onto this provider's
    /// current reference point. Cheap enough to call wherever a reading is
    /// needed — it doesn't snapshot anything itself, it just borrows this
    /// provider's state (see [`Self::advance_reference_point`]).
    fn get_clock(&'a self) -> Clock;

    /// Snapshots the current hardware tick count and folds it into the
    /// reference point every [`ClockTrait`] view reads from. Needs to be
    /// called often enough that the hardware tick counter can't have
    /// wrapped more than half its range between calls (see
    /// `DurationFromTicks::time_at`'s doc comment in the `common` crate)
    /// for [`ClockTrait::now`] to stay accurate in between.
    fn advance_reference_point(&self);
}
