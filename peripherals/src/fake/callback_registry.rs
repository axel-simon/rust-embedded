//! A name -> multi-subscriber callback registry, plus a time-ordered
//! priority queue of callbacks scheduled at a future simulated time.
//!
//! Fakes and test code use the registry to trigger and react to interrupts and
//! trigger mechanisms within the MCU. For example, a PWM's periodic
//! trigger-out edge may trigger an ADC conversion; the ADC's invokes a trigger
//! to simulate the end-of-conversion interrupt once a simulated conversion
//! time has elapsed.

use std::cell::RefCell;
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap};
use std::rc::Rc;

use common::duration::Duration;
use common::uptime::Uptime;

use crate::fake::clock::FakeClockProvider;

/// Every callback registered under one trigger name — see
/// [`CallbackRegistry::register`].
#[derive(Default)]
struct CallbackHook(Vec<Box<dyn FnMut(Uptime)>>);

impl CallbackHook {
    /// Runs every registered callback, in registration order, with `now`.
    fn invoke(&mut self, now: Uptime) {
        for callback in &mut self.0 {
            callback(now);
        }
    }
}

/// One entry in [`CallbackRegistryState::pending`] — a trigger name
/// scheduled to fire once simulated time reaches `time`. Ordered by `time`
/// only; `trigger` just breaks ties, for deterministic iteration order
/// when two entries share a `time`.
#[derive(PartialEq, Eq)]
struct PendingCallback {
    time: Uptime,
    trigger: String,
}

impl Ord for PendingCallback {
    fn cmp(&self, other: &Self) -> Ordering {
        self.time
            .cmp(&other.time)
            .then_with(|| self.trigger.cmp(&other.trigger))
    }
}

impl PartialOrd for PendingCallback {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Every field of the shared state is independently interior-mutable, so a
/// callback running inside
/// [`CallbackRegistry::fire`]/[`CallbackRegistry::fire_at`] (which borrows
/// `hooks`) can freely call [`CallbackRegistry::schedule`] (which only borrows
/// `pending`) without conflicting — this is exactly what a self-perpetuating
/// simulated periodic trigger (see `crate::fake::pwm`) needs to do from within
/// its own callback.
struct CallbackRegistryState {
    hooks: RefCell<HashMap<String, CallbackHook>>,
    pending: RefCell<BinaryHeap<Reverse<PendingCallback>>>,
    clock: RefCell<FakeClockProvider>,
}

/// A registry mapping trigger names to callback hooks, shared (via `Rc`)
/// between every fake that needs it and the board/test code that
/// constructs it.
///
/// A callback must not (transitively) call [`Self::fire`]/[`Self::poll`]/
/// [`Self::advance_clock_by`] — i.e. cause another trigger to *fire* —
/// from within its own execution; doing so panics (a `RefCell` re-entrancy
/// conflict on `hooks`). Scheduling a *future* firing via [`Self::schedule`]
/// (including of the trigger currently running, for a self-perpetuating
/// signal) is always fine.
#[derive(Clone)]
pub struct CallbackRegistry(Rc<CallbackRegistryState>);

impl CallbackRegistry {
    /// Builds an empty registry sharing `clock`'s simulated time — every
    /// `now` a callback receives (from [`Self::fire`]/[`Self::schedule`]'s
    /// eventual firing/[`Self::advance_clock_by`]/[`Self::poll`]) is read
    /// from this same clock, so it stays consistent with whatever else in
    /// a test reads [`FakeClockProvider::now`].
    pub fn new(clock: FakeClockProvider) -> Self {
        CallbackRegistry(Rc::new(CallbackRegistryState {
            hooks: RefCell::new(HashMap::new()),
            pending: RefCell::new(BinaryHeap::new()),
            clock: RefCell::new(clock),
        }))
    }

    /// Registers `callback` under `trigger` — several independent callers
    /// may register under the same name; all of them run, in registration
    /// order, whenever `trigger` fires (via [`Self::fire`] or a
    /// [`Self::schedule`]d entry reaching its time).
    pub fn register(&self, trigger: impl Into<String>, callback: impl FnMut(Uptime) + 'static) {
        self.0
            .hooks
            .borrow_mut()
            .entry(trigger.into())
            .or_default()
            .0
            .push(Box::new(callback));
    }

    /// Runs every callback registered under `trigger` right now, i.e. at
    /// the clock's current reading.
    pub fn fire(&self, trigger: &str) {
        self.fire_at(trigger, self.now());
    }

    /// Enqueues `trigger` to fire once simulated time reaches `at` — see
    /// [`Self::advance_clock_by`]/[`Self::poll`].
    pub fn schedule(&self, trigger: impl Into<String>, at: Uptime) {
        self.0.pending.borrow_mut().push(Reverse(PendingCallback {
            time: at,
            trigger: trigger.into(),
        }));
    }

    /// Advances the shared clock by `duration`, firing every scheduled
    /// trigger reached along the way, in time order — each firing receives
    /// its own scheduled [`Uptime`] as `now`, not the final target time,
    /// and the shared clock itself is moved to exactly that time before
    /// the firing runs (so a callback that reads the clock directly, not
    /// just its `now` parameter, still sees a consistent reading). The
    /// fake-mode replacement for calling [`FakeClockProvider::advance_by`]
    /// directly, whenever pending callbacks need a chance to run as time
    /// passes.
    pub fn advance_clock_by(&self, duration: Duration) {
        let target = self.now() + duration;
        loop {
            let due_time = self.0.pending.borrow().peek().map(|Reverse(e)| e.time);
            match due_time {
                Some(time) if time <= target => {
                    // `delta` must be computed before `borrow_mut()`: it
                    // reads the clock itself (via `self.now()`), which
                    // would otherwise conflict with the still-live
                    // mutable borrow `advance_by`'s receiver holds for the
                    // whole call.
                    let delta = time - self.now();
                    self.0.clock.borrow_mut().advance_by(delta);
                    let Reverse(entry) =
                        self.0.pending.borrow_mut().pop().expect("just peeked Some");
                    self.fire_at(&entry.trigger, entry.time);
                }
                _ => break,
            }
        }
        let remaining = target - self.now();
        self.0.clock.borrow_mut().advance_by(remaining);
    }

    /// Fires everything already due at the clock's *current* reading,
    /// without advancing it — lets simulated time that crept forward via
    /// ordinary firmware polling (`ClockTrait::now()`/`ticks_now()`
    /// self-increment on every read — see `crate::fake::clock`) still
    /// trigger pending callbacks, without every caller having to remember
    /// to call [`Self::advance_clock_by`].
    pub fn poll(&self) {
        let now = self.now();
        loop {
            let due = {
                let mut pending = self.0.pending.borrow_mut();
                match pending.peek() {
                    Some(Reverse(entry)) if entry.time <= now => {
                        let Reverse(entry) = pending.pop().expect("just peeked Some");
                        Some(entry)
                    }
                    _ => None,
                }
            };
            let Some(entry) = due else { break };
            self.fire_at(&entry.trigger, entry.time);
        }
    }

    /// The registry's shared clock's current simulated reading — see
    /// [`Self::new`].
    pub fn now(&self) -> Uptime {
        Uptime::epoch() + self.0.clock.borrow().now()
    }

    fn fire_at(&self, trigger: &str, now: Uptime) {
        if let Some(hook) = self.0.hooks.borrow_mut().get_mut(trigger) {
            hook.invoke(now);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake::clock::ClockProvider;
    use std::cell::Cell;

    fn registry() -> (CallbackRegistry, FakeClockProvider) {
        let (_provider, fake_clock) = ClockProvider::new((), 1_000_000);
        (CallbackRegistry::new(fake_clock.clone()), fake_clock)
    }

    fn now(clock: &FakeClockProvider) -> Uptime {
        Uptime::epoch() + clock.now()
    }

    #[test]
    fn fire_runs_every_registered_callback_in_registration_order() {
        let (registry, _clock) = registry();
        let order = Rc::new(RefCell::new(Vec::new()));

        let order1 = order.clone();
        registry.register("x", move |_now| order1.borrow_mut().push(1));
        let order2 = order.clone();
        registry.register("x", move |_now| order2.borrow_mut().push(2));

        registry.fire("x");
        assert_eq!(*order.borrow(), vec![1, 2]);
    }

    #[test]
    fn fire_only_invokes_callbacks_registered_under_that_name() {
        let (registry, _clock) = registry();
        let count = Rc::new(Cell::new(0));
        let count2 = count.clone();
        registry.register("x", move |_now| count2.set(count2.get() + 1));

        registry.fire("y");
        assert_eq!(count.get(), 0);

        registry.fire("x");
        assert_eq!(count.get(), 1);
    }

    #[test]
    fn schedule_does_not_fire_until_the_clock_reaches_it() {
        let (registry, clock) = registry();
        let fired = Rc::new(Cell::new(false));
        let fired2 = fired.clone();
        registry.register("x", move |_now| fired2.set(true));

        registry.schedule("x", now(&clock) + Duration::from_millis(10));
        registry.advance_clock_by(Duration::from_millis(5));
        assert!(!fired.get());
    }

    #[test]
    fn advance_clock_by_fires_a_scheduled_trigger_reached_along_the_way() {
        let (registry, clock) = registry();
        let received = Rc::new(RefCell::new(None));
        let received2 = received.clone();
        registry.register("x", move |now| *received2.borrow_mut() = Some(now));

        let start = now(&clock);
        registry.schedule("x", start + Duration::from_millis(10));
        registry.advance_clock_by(Duration::from_millis(20));

        assert_eq!(*received.borrow(), Some(start + Duration::from_millis(10)));
    }

    #[test]
    fn advance_clock_by_fires_multiple_pending_entries_in_time_order() {
        let (registry, clock) = registry();
        let order = Rc::new(RefCell::new(Vec::new()));

        let order_a = order.clone();
        registry.register("a", move |now| order_a.borrow_mut().push(("a", now)));
        let order_b = order.clone();
        registry.register("b", move |now| order_b.borrow_mut().push(("b", now)));

        let start = now(&clock);
        registry.schedule("a", start + Duration::from_millis(20));
        registry.schedule("b", start + Duration::from_millis(10));
        registry.advance_clock_by(Duration::from_millis(30));

        assert_eq!(
            *order.borrow(),
            vec![
                ("b", start + Duration::from_millis(10)),
                ("a", start + Duration::from_millis(20)),
            ]
        );
    }

    #[test]
    fn advance_clock_by_advances_the_clock_to_the_target_even_with_no_pending_entries() {
        let (registry, clock) = registry();
        let start = now(&clock);
        registry.advance_clock_by(Duration::from_millis(50));
        // `>=` alone is too strict here: `FakeClockProvider::advance_by`
        // floor-rounds a `Duration` through a tick count, so the actual
        // advance can undershoot by a sub-tick sliver — tolerate that.
        let elapsed = now(&clock) - start;
        assert!(
            elapsed >= Duration::from_millis(49) && elapsed <= Duration::from_millis(51),
            "{elapsed:?}"
        );
    }

    #[test]
    fn a_callback_can_reschedule_itself() {
        let (registry, clock) = registry();
        let count = Rc::new(Cell::new(0));
        let count2 = count.clone();
        let registry_clone = registry.clone();
        registry.register("x", move |fired_at| {
            count2.set(count2.get() + 1);
            if count2.get() < 3 {
                registry_clone.schedule("x", fired_at + Duration::from_millis(10));
            }
        });

        registry.schedule("x", now(&clock) + Duration::from_millis(10));
        registry.advance_clock_by(Duration::from_millis(100));

        assert_eq!(count.get(), 3);
    }

    #[test]
    fn poll_fires_entries_already_due_without_advancing_the_clock() {
        let (registry, clock) = registry();
        let fired = Rc::new(Cell::new(false));
        let fired2 = fired.clone();
        registry.register("x", move |_now| fired2.set(true));

        registry.schedule("x", now(&clock)); // already due
        registry.poll();

        assert!(fired.get());
    }

    #[test]
    fn poll_does_not_fire_entries_still_in_the_future() {
        let (registry, clock) = registry();
        let fired = Rc::new(Cell::new(false));
        let fired2 = fired.clone();
        registry.register("x", move |_now| fired2.set(true));

        registry.schedule("x", now(&clock) + Duration::from_millis(10));
        registry.poll();

        assert!(!fired.get());
    }
}
