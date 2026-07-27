//! A fake [`DmaTrait`] implementation for host-side testing, with no real
//! hardware involved.
//!
//! [`Dma::new`] hands back two handles onto one shared, simulated routing
//! table: [`Dma`] itself, for firmware, and [`FakeDma`], for a test to
//! inspect what's been claimed/[`DmaTrait::allocate`]d.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use crate::api::dma::{DmaChannel, DmaInstance, DmaRequest, DmaTrait};

/// Implemented by any type that knows its own physical DMA channel
/// identity — e.g. a test-local channel-identity type, or (via a blanket
/// impl over its inner type) a `Peri`-style ownership wrapper around one,
/// like `boards/resources`'s fake `Peri<'d, T>`. Mirrors
/// [`crate::fake::gpio::PinToken`] exactly, for the same reason:
/// [`Dma::claim_channel`] uses this to recover a claimed channel's
/// identity — the real driver
/// ([`crate::stm32g4::dma::Dma::claim_channel`]) can't do the same
/// (`DmaChannelToken` is foreign to whichever crate would want to
/// implement it for a real embassy-X backend's own channel types, which
/// is exactly what Rust's orphan rule forbids), so ownership (a plain
/// move) is the only bookkeeping available there instead.
pub trait DmaChannelToken {
    const INSTANCE: DmaInstance;
    const CHANNEL: u8;
}

/// The simulated routing table shared between a [`Dma`] and its
/// [`FakeDma`] counterpart (see [`Dma::new`]). Unlike the real driver (see
/// `crate::stm32g4::dma`), which is bounded by how many DMA channels the
/// chip actually has, this has no fixed capacity — any [`DmaChannel`] can
/// be claimed/allocated.
struct DmaState {
    /// Channels registered as "wired up" via [`Dma::claim_channel`] — see
    /// [`crate::fake::gpio::Gpio`]'s `registered` field for the same idea
    /// applied to pins.
    claimed: HashSet<DmaChannel>,
    table: HashMap<DmaChannel, DmaRequest>,
}

impl DmaState {
    fn warn(&self, args: core::fmt::Arguments) {
        emit_warning(args);
    }
}

/// A fake DMA request-router driver that simulates the routing table,
/// without touching any real hardware.
pub struct Dma(Rc<RefCell<DmaState>>);

/// A test's handle onto the same simulated table a [`Dma`] drives — see
/// [`Dma::new`]. Cheap to [`Clone`] (all clones share the same underlying
/// state).
#[derive(Clone)]
pub struct FakeDma(Rc<RefCell<DmaState>>);

impl Dma {
    /// Creates a fake DMA request router with an empty routing table, and
    /// a [`FakeDma`] handle onto the same simulated state.
    pub fn new() -> (Self, FakeDma) {
        let state = Rc::new(RefCell::new(DmaState {
            claimed: HashSet::new(),
            table: HashMap::new(),
        }));
        (Dma(state.clone()), FakeDma(state))
    }

    /// Claims ownership of a channel-resource handle and registers it as
    /// "wired up" — e.g. a fake `resources::Peri<'static,
    /// resources::peripherals::DMA1_CH1>` stand-in for a real embassy-X
    /// backend's `Peri`. Mirrors [`crate::fake::gpio::Gpio::claim_pin`]
    /// exactly: unlike the real driver
    /// ([`claim_channel`](crate::stm32g4::dma::Dma::claim_channel)), this
    /// does real bookkeeping (via [`DmaChannelToken`]), so
    /// [`DmaTrait::allocate`] can warn if it's called on a channel that
    /// was never claimed.
    pub fn claim_channel<T: DmaChannelToken>(&mut self, _peri: T) {
        let channel = DmaChannel::new(T::INSTANCE, T::CHANNEL);
        self.0.borrow_mut().claimed.insert(channel);
    }
}

impl DmaTrait for Dma {
    fn allocate(&mut self, channel: DmaChannel, request: DmaRequest) {
        let mut state = self.0.borrow_mut();
        if let Some(&existing) = state.table.get(&channel) {
            panic!(
                "allocate({channel:?}, {request:?}) called again for a channel already \
                 allocated to {existing:?} — DMA channels can only be allocated once"
            );
        }
        if !state.claimed.contains(&channel) {
            state.warn(format_args!(
                "allocate() called on channel {channel:?} that was never claimed via \
                 Dma::claim_channel()"
            ));
        }
        state.table.insert(channel, request);
    }

    fn lookup_channel(&self, request: DmaRequest) -> Option<DmaChannel> {
        if request == DmaRequest::None {
            return None;
        }
        self.0
            .borrow()
            .table
            .iter()
            .find_map(|(&channel, &r)| (r == request).then_some(channel))
    }
}

impl FakeDma {
    /// The request currently routed to `channel` — [`DmaRequest::None`] if
    /// [`DmaTrait::allocate`] was never called for it (or was last called
    /// with `DmaRequest::None`). Not part of [`DmaTrait`] itself (nothing
    /// outside this module needs it — see `crate::stm32g4::dma`'s private
    /// `channel_request`), just a test's read-only window onto the table.
    pub fn request(&self, channel: DmaChannel) -> DmaRequest {
        self.0
            .borrow()
            .table
            .get(&channel)
            .copied()
            .unwrap_or_default()
    }

    /// Whether `channel` was claimed via [`Dma::claim_channel`] — the same
    /// check [`DmaTrait::allocate`] makes internally to decide whether to
    /// warn. Lets a test verify claiming happened for exactly the channels
    /// it expects, without going through `allocate`.
    pub fn is_claimed(&self, channel: DmaChannel) -> bool {
        self.0.borrow().claimed.contains(&channel)
    }
}

// See peripherals/src/fake/gpio.rs for why this is cfg(test)-gated rather
// than cfg(not(target_arch = "arm")).
#[cfg(test)]
fn emit_warning(args: core::fmt::Arguments) {
    eprintln!("dma fake warning: {args}");
}

#[cfg(not(test))]
fn emit_warning(_args: core::fmt::Arguments) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::dma::DmaInstance;

    /// An arbitrary channel identity for tests to claim — its
    /// instance/number don't matter, only that it's consistent across a
    /// test. Mirrors `crate::fake::gpio`'s `TestPin`.
    #[derive(Clone, Copy)]
    struct TestChannel;

    impl DmaChannelToken for TestChannel {
        const INSTANCE: DmaInstance = DmaInstance::Stm32g4Dma2;
        const CHANNEL: u8 = 2;
    }

    fn channel() -> DmaChannel {
        DmaChannel::new(TestChannel::INSTANCE, TestChannel::CHANNEL)
    }

    #[test]
    fn channel_starts_unclaimed_and_unallocated() {
        let (_dma, fake) = Dma::new();
        assert!(!fake.is_claimed(channel()));
        assert_eq!(fake.request(channel()), DmaRequest::None);
    }

    #[test]
    fn claim_channel_registers_it() {
        let (mut dma, fake) = Dma::new();
        dma.claim_channel(TestChannel);
        assert!(fake.is_claimed(channel()));
    }

    #[test]
    fn allocate_is_visible_through_both_handles() {
        let (mut dma, fake) = Dma::new();
        dma.claim_channel(TestChannel);
        dma.allocate(channel(), DmaRequest::Stm32g4DmamuxReqAdc1);
        assert_eq!(fake.request(channel()), DmaRequest::Stm32g4DmamuxReqAdc1);
    }

    #[test]
    fn allocating_a_different_channel_does_not_affect_others() {
        let (mut dma, fake) = Dma::new();
        let a = DmaChannel::new(DmaInstance::Stm32g4Dma1, 1);
        let b = DmaChannel::new(DmaInstance::Stm32g4Dma1, 2);
        dma.allocate(a, DmaRequest::Stm32g4DmamuxReqAdc1);
        assert_eq!(fake.request(b), DmaRequest::None);
    }

    #[test]
    #[should_panic(expected = "already allocated to Stm32g4DmamuxReqAdc1")]
    fn reallocating_an_already_allocated_channel_panics() {
        let (mut dma, _fake) = Dma::new();
        dma.claim_channel(TestChannel);
        dma.allocate(channel(), DmaRequest::Stm32g4DmamuxReqAdc1);
        dma.allocate(channel(), DmaRequest::Stm32g4DmamuxReqSpi1Rx);
    }

    #[test]
    fn lookup_channel_finds_the_channel_a_request_was_allocated_to() {
        let (mut dma, _fake) = Dma::new();
        dma.claim_channel(TestChannel);
        dma.allocate(channel(), DmaRequest::Stm32g4DmamuxReqAdc1);
        assert_eq!(
            dma.lookup_channel(DmaRequest::Stm32g4DmamuxReqAdc1),
            Some(channel())
        );
    }

    #[test]
    fn lookup_channel_returns_none_for_an_unallocated_request() {
        let (dma, _fake) = Dma::new();
        assert_eq!(dma.lookup_channel(DmaRequest::Stm32g4DmamuxReqAdc1), None);
    }

    #[test]
    fn lookup_channel_returns_none_for_none_even_if_a_channel_was_allocated_with_it() {
        let (mut dma, _fake) = Dma::new();
        dma.allocate(
            DmaChannel::new(DmaInstance::Stm32g4Dma1, 1),
            DmaRequest::None,
        );
        assert_eq!(dma.lookup_channel(DmaRequest::None), None);
    }
}
