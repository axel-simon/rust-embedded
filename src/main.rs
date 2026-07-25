#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::duration::Duration;
use esc1_discovery::{BoardPeripherals, ClockProvider, Gpio};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::gpio::GpioTrait;

#[cfg(not(test))]
use cortex_m_rt::entry;
#[cfg(not(test))]
use {defmt_rtt as _, panic_probe as _};

// Make the LED blink at 1Hz.
const HALF_PERIOD: Duration = Duration::from_millis(500);

/// Everything the firmware does after chip bring-up, one blink at a time.
/// Kept separate from [`main`] so it can be driven by unit tests (see the
/// `tests` module below) without a `-> !` loop or real hardware.
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

#[cfg(not(test))]
#[entry]
fn main() -> ! {
    defmt::info!("init");

    let mut firmware = Firmware::new(esc1_discovery::initialize());
    loop {
        firmware.step();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn led_blinks_at_roughly_1hz() {
        let peripherals = esc1_discovery::initialize();
        let fakes = peripherals.fakes.clone(); // cheap: Rc-backed
        let mut firmware = Firmware::new(peripherals);

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
