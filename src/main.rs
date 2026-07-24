#![no_std]
#![no_main]

use common::duration::Duration;
use cortex_m_rt::entry;
use esc1_discovery::BoardPeripherals;
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::gpio::GpioTrait;
use {defmt_rtt as _, panic_probe as _};

// Make the LED blink at 1Hz.
const HALF_PERIOD: Duration = Duration::from_millis(500);

#[entry]
fn main() -> ! {
    defmt::info!("init");

    // `gpio`/`clock_provider` come out named `esc1_discovery::Gpio`/
    // `esc1_discovery::ClockProvider` — backend-selected type aliases, so
    // this destructuring (and anything downstream that wants to name
    // these types, e.g. a helper function argument) doesn't need its own
    // `target_arch = "arm"` cfg-gating.
    let BoardPeripherals {
        gpio,
        clock_provider,
        ..
    } = esc1_discovery::initialize();
    let clock = clock_provider.get_clock();

    loop {
        // Keeps `clock`'s reference point fresh enough for `wait_for`'s
        // `now()` reads to stay accurate — see
        // `ClockProviderTrait::advance_reference_point`'s doc comment.
        clock_provider.advance_reference_point();

        gpio.set(esc1_discovery::STATUS_PIN, true);
        clock.wait_for(HALF_PERIOD);
        gpio.set(esc1_discovery::STATUS_PIN, false);
        clock.wait_for(HALF_PERIOD);
        let now = clock.now();
        defmt::info!(
            "LED toggled, uptime: {}.{}s",
            now.seconds(),
            now.fraction_as_nanos()
        );
    }
}
