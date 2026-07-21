#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

#[rtic::app(device = stm32g4xx_hal::stm32, peripherals = true)]
mod app {
    use defmt::info;
    use stm32g4xx_hal::gpio::gpioc::PC6;
    use stm32g4xx_hal::gpio::{Output, PushPull};
    use stm32g4xx_hal::prelude::*;
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::rcc::{Config, RccExt};
    use stm32g4xx_hal::stm32::TIM2;
    use stm32g4xx_hal::time::ExtU32;
    use stm32g4xx_hal::timer::{CountDownTimer, Event, Timer};

    #[shared]
    struct Shared {}

    /// The LED and the timer that drives it are local to (owned
    /// exclusively by) the timer interrupt task - no other task can
    /// touch them.
    #[local]
    struct Local {
        led: PC6<Output<PushPull>>,
        timer: CountDownTimer<TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("init");

        let dp = cx.device;
        let pwr = dp.PWR.constrain().freeze();
        let mut rcc = dp.RCC.freeze(Config::hsi(), pwr);

        let gpioc = dp.GPIOC.split(&mut rcc);
        let led = gpioc.pc6.into_push_pull_output();

        // TIM2 counts up to 100,000 ticks (1 tick = 1 microsecond, so
        // 100 ms) and fires an update interrupt each time it wraps.
        let mut timer = Timer::new(dp.TIM2, &rcc.clocks).start_count_down(100_000.micros());
        timer.listen(Event::TimeOut);

        (Shared {}, Local { led, timer })
    }

    #[task(binds = TIM2, local = [led, timer])]
    fn tim2_tick(cx: tim2_tick::Context) {
        cx.local.timer.clear_interrupt(Event::TimeOut);
        cx.local.led.toggle();
        info!("LED toggled");
    }
}
