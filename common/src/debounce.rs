//! A generic two-threshold (asymmetric) boolean-signal debouncer.
//!
//! [`Debouncer`] has no notion of "now" of its own — the caller supplies
//! the current time to every [`Debouncer::poll`] call.

use crate::duration::Duration;
use crate::uptime::Uptime;

/// Debounces a boolean signal (e.g. a push-button reading): a raw value
/// only becomes the confirmed/debounced value once it's been observed
/// continuously for at least the relevant threshold —
/// [`Self::new`]'s `inactive_to_active` for a `false -> true` transition,
/// `active_to_inactive` for the reverse.
pub struct Debouncer {
    inactive_to_active: Duration,
    active_to_inactive: Duration,
    confirmed_active: bool,
    /// The raw value currently being debounced, and the [`Uptime`] at
    /// which it becomes confirmed if it holds that long — `None` while
    /// the raw reading already matches `confirmed_active` (nothing
    /// pending).
    pending: Option<(bool, Uptime)>,
}

impl Debouncer {
    /// Builds a `Debouncer`, initially confirmed inactive (`false`).
    pub const fn new(inactive_to_active: Duration, active_to_inactive: Duration) -> Self {
        Debouncer {
            inactive_to_active,
            active_to_inactive,
            confirmed_active: false,
            pending: None,
        }
    }

    /// Feeds in one new raw reading, taken at `now` — call this once per
    /// raw sample (e.g. once per main-loop iteration); it never blocks or
    /// loops internally, so a transition is never confirmed on the very
    /// same call it's first observed on (even with a zero-length
    /// threshold) — the next `poll()`, at the same or a later `now`, is
    /// what confirms it. Read the result back via [`Self::is_active`].
    pub fn poll(&mut self, now: Uptime, raw_active: bool) {
        match self.pending {
            Some((candidate, deadline)) if candidate == raw_active => {
                if now >= deadline {
                    self.confirmed_active = candidate;
                    self.pending = None;
                }
            }
            _ => {
                self.pending = if raw_active == self.confirmed_active {
                    None
                } else {
                    let threshold = if raw_active {
                        self.inactive_to_active
                    } else {
                        self.active_to_inactive
                    };
                    Some((raw_active, now + threshold))
                };
            }
        }
    }

    /// The confirmed/debounced state as of the last [`Self::poll`] call
    /// (`false` if never polled).
    pub const fn is_active(&self) -> bool {
        self.confirmed_active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(millis: i32) -> Uptime {
        Uptime::epoch() + Duration::from_millis(millis)
    }

    #[test]
    fn starts_confirmed_inactive() {
        let debouncer = Debouncer::new(Duration::from_millis(20), Duration::from_millis(20));
        assert!(!debouncer.is_active());
    }

    #[test]
    fn confirms_active_once_the_threshold_elapses() {
        let mut debouncer = Debouncer::new(Duration::from_millis(20), Duration::from_millis(20));
        debouncer.poll(at(0), true);
        assert!(!debouncer.is_active());
        debouncer.poll(at(19), true);
        assert!(!debouncer.is_active());
        debouncer.poll(at(20), true);
        assert!(debouncer.is_active());
    }

    #[test]
    fn a_bounce_before_the_threshold_resets_the_wait() {
        let mut debouncer = Debouncer::new(Duration::from_millis(20), Duration::from_millis(20));
        debouncer.poll(at(0), true);
        assert!(!debouncer.is_active());
        // Bounces back to inactive before confirming -- already matches
        // confirmed_active, so this just cancels the pending transition.
        debouncer.poll(at(10), false);
        assert!(!debouncer.is_active());
        // Presses again -- the wait restarts from here, not from t=0.
        debouncer.poll(at(10), true);
        assert!(!debouncer.is_active());
        debouncer.poll(at(29), true);
        assert!(!debouncer.is_active());
        debouncer.poll(at(30), true);
        assert!(debouncer.is_active());
    }

    #[test]
    fn confirms_inactive_once_the_threshold_elapses() {
        let mut debouncer = Debouncer::new(Duration::from_millis(20), Duration::from_millis(30));
        // Get to confirmed-active first, so the assertions below are only
        // exercising the active_to_inactive threshold.
        debouncer.poll(at(0), true);
        debouncer.poll(at(20), true);
        assert!(debouncer.is_active());

        debouncer.poll(at(20), false);
        assert!(debouncer.is_active());
        debouncer.poll(at(49), false);
        assert!(debouncer.is_active());
        debouncer.poll(at(50), false);
        assert!(!debouncer.is_active());
    }

    #[test]
    fn a_bounce_before_the_threshold_resets_the_wait_to_inactive() {
        let mut debouncer = Debouncer::new(Duration::from_millis(20), Duration::from_millis(30));
        // Get to confirmed-active first, so the assertions below are only
        // exercising the active_to_inactive threshold.
        debouncer.poll(at(0), true);
        debouncer.poll(at(20), true);
        assert!(debouncer.is_active());

        debouncer.poll(at(20), false);
        assert!(debouncer.is_active());
        // Bounces back to active before confirming -- already matches
        // confirmed_active, so this just cancels the pending transition.
        debouncer.poll(at(30), true);
        assert!(debouncer.is_active());
        // Releases again -- the wait restarts from here, not from t=20.
        debouncer.poll(at(30), false);
        assert!(debouncer.is_active());
        debouncer.poll(at(59), false);
        assert!(debouncer.is_active());
        debouncer.poll(at(60), false);
        assert!(!debouncer.is_active());
    }

    #[test]
    fn confirms_inactive_using_its_own_threshold() {
        let mut debouncer = Debouncer::new(Duration::from_millis(5), Duration::from_millis(50));
        debouncer.poll(at(0), true);
        assert!(!debouncer.is_active());
        debouncer.poll(at(5), true);
        assert!(debouncer.is_active());
        // Released -- still reports active, the 50ms release threshold
        // hasn't elapsed yet.
        debouncer.poll(at(5), false);
        assert!(debouncer.is_active());
        debouncer.poll(at(54), false);
        assert!(debouncer.is_active());
        debouncer.poll(at(55), false);
        assert!(!debouncer.is_active());
    }
}
