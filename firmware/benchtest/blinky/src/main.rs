#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::duration::Duration;
use esc1_discovery::BoardPeripherals;
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::gpio::GpioTrait;
// `esc1_discovery` no longer re-exports its own backend-selected
// `Gpio`/`ClockProvider` (it names its driver types as
// `backend::gpio::Gpio`/`backend::clock::ClockProvider` internally now) —
// this firmware still needs its own copy of the same cfg'd selection to
// store one in `Firmware` below.
#[cfg(target_arch = "arm")]
use peripherals::stm32g4::{clock::ClockProvider, gpio::Gpio};
#[cfg(not(target_arch = "arm"))]
use peripherals::fake::{clock::ClockProvider, gpio::Gpio};

// `#[rtic_shim::app]` generates its own entry point on real hardware (see
// the `app` module below), replacing the usual `#[cortex_m_rt::entry]`.
// The RTT logging backend/panic handler `defmt`/`no_std` need are
// registered from `boards/resources` instead (see its `stm32g4.rs`).

// Make the LED blink at 1Hz.
const HALF_PERIOD: Duration = Duration::from_millis(500);

/// Everything the firmware does after chip bring-up, one blink at a time.
/// Kept separate from the `app` module below so it can be driven by unit
/// tests (see the `tests` module) without a real interrupt-driven
/// scheduler.
struct Firmware {
    gpio: Gpio,
    clock_provider: ClockProvider,
}

impl Firmware {
    fn new(peripherals: BoardPeripherals) -> Self {
        let BoardPeripherals {
            gpio,
            clock_provider,
            ..
        } = peripherals;
        Firmware {
            gpio,
            clock_provider,
        }
    }

    /// Turns the status LED on, then off, [`HALF_PERIOD`] apart — one full
    /// blink cycle.
    fn step(&mut self) {
        // Keeps the clock's reference point fresh enough for `wait_for`'s
        // `now()` reads to stay accurate — see
        // `ClockProviderTrait::advance_reference_point`'s doc comment.
        self.clock_provider.advance_reference_point();
        let clock = self.clock_provider.get_clock();

        self.gpio.set(esc1_discovery::STATUS_PIN, true);
        clock.wait_for(HALF_PERIOD);
        self.gpio.set(esc1_discovery::STATUS_PIN, false);
        clock.wait_for(HALF_PERIOD);

        #[cfg(not(test))]
        {
            let now = clock.now();
            defmt::info!(
                "LED toggled, uptime: {}.{}s",
                now.seconds(),
                now.fraction_as_nanos()
            );
        }
    }
}

/// `stm32-metapac` mirrors an svd2rust PAC's register blocks (used
/// directly, e.g. `stm32_metapac::GPIOA`, by `peripherals::stm32g4`) but
/// doesn't generate a `NVIC_PRIO_BITS` const the way a real svd2rust PAC
/// does — `#[rtic::app]`'s `device` argument needs one under exactly that
/// name (to compute its priority-masking limits) even with zero
/// interrupt-bound tasks. The actual value is a board fact (it depends on
/// the MCU, not on this application), so it's defined as
/// [`esc1_discovery::RTIC_PRIORITY_BITS`] and just re-exported here under
/// the name RTIC's Cortex-M backend looks for.
#[cfg(target_arch = "arm")]
mod rtic_device {
    // `allow`: nothing here needs `Interrupt`/register-block re-exports
    // until a real interrupt-bound task exists; kept for when one does.
    #[allow(unused_imports)]
    pub use stm32_metapac::*;
    pub use esc1_discovery::RTIC_PRIORITY_BITS as NVIC_PRIO_BITS;
}

/// The application's RTIC task graph — real RTIC on real hardware, or (via
/// [`rtic_shim::app`]) a host-only fake that hands `init`'s `Local`
/// straight to unit tests, so they can drive [`Firmware::step`] directly
/// without a real interrupt-driven scheduler. `rtic_shim::app` always
/// forces `peripherals = false` (see `rtic_real`): chip bring-up in this
/// workspace always goes through a board crate's own `initialize()`
/// (`embassy_stm32::init()` under the hood), not RTIC's own PAC-peripherals
/// claim — `stm32_metapac` doesn't even expose the `Peripherals` type that
/// would need.
#[rtic_shim::app(device = crate::rtic_device)]
mod app {
    use super::Firmware;

    #[shared]
    pub(crate) struct Shared {}

    // `pub(crate)`: `#[cfg(test)] mod tests` (a sibling of this module, not
    // a descendant) needs to reach these fields directly.
    #[local]
    pub(crate) struct Local {
        pub(crate) firmware: Firmware,
        #[cfg(not(target_arch = "arm"))]
        pub(crate) fakes: esc1_discovery::BoardFakePeripherals,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        #[cfg(not(test))]
        defmt::info!("init");

        // RTIC's own `init` prologue already steals `cortex_m::Peripherals`
        // (that's `cx.core`) before calling this, so it's threaded through
        // rather than taken again — see `initialize`'s doc comment.
        #[cfg(target_arch = "arm")]
        let peripherals = esc1_discovery::initialize(_cx.core);
        #[cfg(not(target_arch = "arm"))]
        let peripherals = esc1_discovery::initialize(());

        #[cfg(not(target_arch = "arm"))]
        let fakes = peripherals.fakes.clone(); // cheap: Rc-backed

        let firmware = Firmware::new(peripherals);

        (
            Shared {},
            Local {
                firmware,
                #[cfg(not(target_arch = "arm"))]
                fakes,
            },
        )
    }

    // `mut`: only the fake (host) `idle::Context` needs it, since it owns
    // `Local` by value rather than borrowing it — see `rtic_fake`.
    #[allow(unused_mut)]
    #[idle(local = [firmware])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.local.firmware.step();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn led_blinks_at_roughly_1hz() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        firmware.step();
        // Each `step()` is a full on/off cycle, so the LED is off again by
        // the time it returns.
        assert_eq!(fakes.gpio.get(esc1_discovery::STATUS_PIN), false);
        let after_first_blink = fakes.clock_provider.now();

        firmware.step();
        let after_second_blink = fakes.clock_provider.now();

        let period = after_second_blink - after_first_blink;
        assert!(
            period >= Duration::from_millis(900) && period <= Duration::from_millis(1100),
            "expected ~1s between blinks, got {period:?}"
        );
    }
}
