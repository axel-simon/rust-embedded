#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use peripherals::api::gpio::GpioTrait;
use {defmt_rtt as _, panic_probe as _};

#[entry]
fn main() -> ! {
    defmt::info!("init");

    let mut p = esc1_discovery::initialize();

    // Matches the 100 ms half-period the RTIC/TIM2 version used.
    loop {
        p.gpio.set(esc1_discovery::STATUS_PIN, true);
        p.delay.delay_ms(100);
        p.gpio.set(esc1_discovery::STATUS_PIN, false);
        p.delay.delay_ms(100);
        defmt::info!("LED toggled");
    }
}
