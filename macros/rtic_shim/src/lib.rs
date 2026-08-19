//! Re-exports [`rtic_real`]'s `app` attribute macro (real RTIC, with
//! `peripherals = false` forced) on real hardware, or [`rtic_fake`]'s
//! host-only fake with the same surface elsewhere, so firmware written
//! once against `#[rtic_shim::app(...)]` builds real RTIC on hardware and
//! stays unit-testable on the host.
#![no_std]

#[cfg(target_arch = "arm")]
pub use rtic_real::app;

#[cfg(not(target_arch = "arm"))]
pub use rtic_fake::app;

/// `#[shared]` resources are accessed through this trait's `lock()` on
/// both backends, so `#[rtic_shim::app]`-annotated code compiles
/// unchanged either way — import `rtic_shim::Mutex`, not `rtic::Mutex`
/// directly, at any call site that calls `.lock()`.
///
/// On real hardware this *is* real RTIC's own [`rtic::Mutex`]: its
/// generated per-resource proxy types implement it, enforcing the
/// priority-ceiling protocol.
#[cfg(target_arch = "arm")]
pub use rtic::Mutex;

/// See the `target_arch = "arm"` version of [`Mutex`] above for the real
/// thing this stands in for off-target. [`rtic_fake`]'s generated
/// `Context` types have no real concurrency to protect (there's no
/// interrupt-driven scheduler on the host), so every type gets this
/// blanket impl: `lock()` just calls the given closure directly against
/// `self`.
#[cfg(not(target_arch = "arm"))]
pub trait Mutex {
    type T;

    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::T) -> R) -> R;
}

#[cfg(not(target_arch = "arm"))]
impl<T> Mutex for T {
    type T = T;

    fn lock<R>(&mut self, f: impl FnOnce(&mut T) -> R) -> R {
        f(self)
    }
}
