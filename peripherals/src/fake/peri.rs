//! A standalone peripheral-ownership token, in the same spirit as
//! `embassy_hal_internal::Peri` but self-contained: no dependency on any
//! embassy crate. See [`Peri`] for the rationale.

use core::marker::PhantomData;

/// Marker trait for peripheral token types (e.g. a GPIO pin like `PC6`, or
/// eventually `ADC1`, `SPI4`, etc). Implementors are expected to be
/// zero-sized (or otherwise trivially `Copy`).
pub trait PeripheralType: Copy + Sized {}

/// An exclusive reference to a peripheral, functionally equivalent to
/// `&'a mut T` but cheaper: `&mut T` is always pointer-sized even when `T`
/// is zero-sized, whereas `Peri` stores `T` directly. It also avoids
/// monomorphizing driver code separately for `T` vs `&mut T`, since both
/// become `Peri<'_, T>` differing only in lifetime.
pub struct Peri<'a, T: PeripheralType> {
    inner: T,
    _lifetime: PhantomData<&'a mut T>,
}

impl<'a, T: PeripheralType> Peri<'a, T> {
    /// Creates an owned peripheral token.
    ///
    /// # Safety
    /// The caller must ensure only one `Peri` for a given concrete
    /// peripheral (e.g. `PC6`) is alive at a time.
    #[inline]
    pub(crate) const unsafe fn new_unchecked(inner: T) -> Self {
        Self {
            inner,
            _lifetime: PhantomData,
        }
    }

    /// Unsafely clones (duplicates) this peripheral token.
    ///
    /// # Safety
    /// This returns an owned clone; the caller must ensure only one copy
    /// is in use at a time. Prefer [`Peri::reborrow`], which lets the
    /// borrow checker enforce that for you.
    #[inline]
    pub(crate) const unsafe fn clone_unchecked(&self) -> Peri<'a, T> {
        Peri::new_unchecked(self.inner)
    }

    /// Reborrows into a "child" `Peri` that borrows `self` for its
    /// lifetime, so `self` can't be used again until the child is dropped.
    #[inline]
    pub const fn reborrow(&mut self) -> Peri<'_, T> {
        // Safety: the child borrows `self`, so the two can't be used
        // concurrently.
        unsafe { self.clone_unchecked() }
    }

    /// Converts `Peri<'a, T>` into `Peri<'a, U>` via `T: Into<U>`, e.g. to
    /// erase a concrete pin like `PC6` into a type-erased `AnyPin`.
    #[inline]
    pub fn into<U>(self) -> Peri<'a, U>
    where
        T: Into<U>,
        U: PeripheralType,
    {
        unsafe { Peri::new_unchecked(self.inner.into()) }
    }
}

impl<'a, T: PeripheralType> core::ops::Deref for Peri<'a, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<'a, T: PeripheralType + core::fmt::Debug> core::fmt::Debug for Peri<'a, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.inner.fmt(f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct FakeToken(u8);
    impl PeripheralType for FakeToken {}

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct FakeAnyToken(u8);
    impl PeripheralType for FakeAnyToken {}
    impl From<FakeToken> for FakeAnyToken {
        fn from(t: FakeToken) -> Self {
            FakeAnyToken(t.0)
        }
    }

    #[test]
    fn deref_gives_access_to_inner() {
        let p = unsafe { Peri::new_unchecked(FakeToken(6)) };
        assert_eq!(*p, FakeToken(6));
    }

    #[test]
    fn reborrow_yields_equal_token() {
        let mut p = unsafe { Peri::new_unchecked(FakeToken(6)) };
        let r = p.reborrow();
        assert_eq!(*r, FakeToken(6));
    }

    #[test]
    fn into_converts_to_erased_type() {
        let p = unsafe { Peri::new_unchecked(FakeToken(6)) };
        let erased: Peri<FakeAnyToken> = p.into();
        assert_eq!(*erased, FakeAnyToken(6));
    }
}
