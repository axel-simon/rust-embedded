#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::duration::Duration;
use common::unit_interval::UnitInterval;
use esc1_discovery::BoardPeripherals;
use peripherals::api::adc::{AdcOptions, AdcSampleBuffer, AdcTrait};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::pwm::{PwmOptions, PwmTrait};
// Backend-selected driver types: real hardware on `target_arch = "arm"`,
// a fake elsewhere (so host-side `cargo test` works without hardware) —
// stored in `Firmware` below.
#[cfg(not(target_arch = "arm"))]
use peripherals::fake::{adc::Adc, clock::ClockProvider, pwm::Pwm};
#[cfg(target_arch = "arm")]
use peripherals::stm32g4::{adc::Adc, clock::ClockProvider, pwm::Pwm};

/// ADC1's channel wired to the B-G431B-ESC1's potentiometer (PB12) — PB12
/// is ADC1_IN11, per the STM32G431's pin table (RM0440/datasheet Table
/// 12's ADCx_INy annotations).
const POTENTIOMETER_CHANNEL: u8 = 11;

/// ADC1's DMA destination buffer — `'static` so its address stays valid
/// for as long as ADC1 stays open, regardless of `Firmware` (and the
/// `Adc` inside it) getting moved around after `Firmware::new` calls
/// `open()`.
static ADC1_SAMPLES: AdcSampleBuffer = AdcSampleBuffer::new();

/// TIM1's switching frequency — comfortably above the audible range, and a
/// common choice for this kind of gate-driven power stage.
const PWM_FREQUENCY_HZ: u32 = 20_000;

/// Only one of TIM1's three channel pairs (phase U) is driven by this
/// benchtest — channel `0`.
const PWM_CHANNELS: u8 = 1;

/// Fixed duty cycle while `Firmware::step` instead uses the potentiometer
/// to sweep dead time live, for hand-tuning the gate driver's dead time on
/// real hardware. `0.2 * 2^32`, floored.
const DUTY_CYCLE: UnitInterval = UnitInterval::new(858_993_459);

/// Upper bound of the potentiometer's dead-time sweep — the full pot
/// travel covers `0..=500` ns, comfortably inside the dead-time register's
/// representable range at TIM1's ~170MHz clock (up to roughly 5.9us).
const DEAD_TIME_TUNING_MAX: Duration = Duration::from_nanos(500);

/// How often to sample the potentiometer, update the dead time, and log
/// it.
const SAMPLE_PERIOD: Duration = Duration::from_seconds(1);

/// Inverts a potentiometer reading (`0` becomes just below `1` and vice
/// versa) — this board's potentiometer reads the opposite way from what
/// this firmware wants "more" (of duty cycle, dead time, whatever it's
/// driving) to mean.
fn invert(value: UnitInterval) -> UnitInterval {
    UnitInterval::new(u32::MAX) - value
}

/// Scales `max` by `fraction` — e.g. `fraction` at half of `UnitInterval`'s
/// range returns half of `max`. Ad-hoc, only for [`DEAD_TIME_TUNING_MAX`]
/// below — not worth adding to `Duration` itself for this one, temporary
/// caller.
fn scale_duration(fraction: UnitInterval, max: Duration) -> Duration {
    Duration::new(((fraction.raw() as u128 * max.raw() as u128) >> 32) as i64)
}

/// Everything the firmware does after chip bring-up, one sample at a time.
/// Kept separate from the `app` module below so it can be driven by unit
/// tests (see the `tests` module) without a real interrupt-driven
/// scheduler.
struct Firmware {
    adc1: Adc,
    pwm1: Pwm,
    clock_provider: ClockProvider,
}

impl Firmware {
    fn new(peripherals: BoardPeripherals) -> Self {
        let BoardPeripherals {
            mut adc1,
            mut pwm1,
            clock_provider,
            dma,
            ..
        } = peripherals;

        adc1.open(
            AdcOptions::new(&[POTENTIOMETER_CHANNEL], &ADC1_SAMPLES),
            &dma,
        );
        // TIM1 and its pins are already claimed and configured by the
        // board bring-up step below (see `init` in the `app` module); this
        // firmware still needs to `open()` `pwm1` itself, since the
        // frequency/channel count/dead time to open it with are choices
        // this application makes, not board-level facts.
        pwm1.open(PwmOptions::new(
            PWM_FREQUENCY_HZ,
            PWM_CHANNELS,
            esc1_discovery::TIM1_DEAD_TIME,
            false,
        ));

        Firmware {
            adc1,
            pwm1,
            clock_provider,
        }
    }

    /// Reads the potentiometer and, TEMPORARY (see [`DUTY_CYCLE`]), uses
    /// it (inverted — see [`invert`]) to sweep dead time on channel `0`
    /// instead of duty cycle, which stays fixed at [`DUTY_CYCLE`]. Logs
    /// both, then paces itself by [`SAMPLE_PERIOD`] before returning the
    /// dead time applied.
    fn step(&mut self) -> Duration {
        // Refreshes the clock's reference point so `now()`/`wait_for()`
        // stay accurate — needed periodically since the underlying
        // hardware tick counter is free-running and would otherwise
        // eventually wrap without this catching it.
        self.clock_provider.advance_reference_point();
        let clock = self.clock_provider.get_clock();

        self.adc1.trigger();
        while !self.adc1.conversion_done() {}
        let potentiometer = invert(self.adc1.get_sample(0));
        let dead_time = scale_duration(potentiometer, DEAD_TIME_TUNING_MAX);

        self.pwm1.set_dead_time(dead_time);
        self.pwm1.set_duty_cycle(0, DUTY_CYCLE);

        #[cfg(not(test))]
        defmt::info!(
            "duty cycle: {}  dead time: {} ns",
            f32::from(DUTY_CYCLE),
            dead_time.fraction_as_nanos()
        );

        clock.wait_for(SAMPLE_PERIOD);
        dead_time
    }
}

/// `stm32-metapac` doesn't generate a `NVIC_PRIO_BITS` const the way a
/// real svd2rust PAC does — `#[rtic::app]`'s `device` argument needs one
/// under exactly that name (to compute its priority-masking limits) even
/// with zero interrupt-bound tasks. The actual value only depends on the
/// MCU, not this application, so it's re-exported here under the name
/// RTIC's Cortex-M backend looks for.
#[cfg(target_arch = "arm")]
mod rtic_device {
    // `allow`: nothing here needs `Interrupt`/register-block re-exports
    // until a real interrupt-bound task exists; kept for when one does.
    pub use esc1_discovery::RTIC_PRIORITY_BITS as NVIC_PRIO_BITS;
    #[allow(unused_imports)]
    pub use stm32_metapac::*;
}

/// The application's RTIC task graph — real RTIC on real hardware, or a
/// host-only fake that hands `init`'s `Local` straight to unit tests, so
/// they can drive [`Firmware::step`] directly without a real
/// interrupt-driven scheduler. Chip bring-up always goes through
/// `esc1_discovery::initialize()` below, not RTIC's own PAC-peripherals
/// claim, which stays disabled here.
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
        // (that's `cx.core`) before calling this, so it's passed through
        // here rather than taken again — taking the core peripherals can
        // only succeed once.
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

    // `mut`: only the fake (host) `idle::Context` needs it, since on that
    // backend it owns `Local` by value rather than borrowing it.
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

    /// Builds a `UnitInterval` that round-trips exactly through the fake
    /// ADC's 16-bit-wide sample storage: only the top 16 bits survive
    /// being set and read back, so this shifts a raw value into exactly
    /// that range up front.
    fn sample(top_16_bits: u16) -> UnitInterval {
        UnitInterval::new((top_16_bits as u32) << 16)
    }

    #[test]
    fn opens_the_adc_and_pwm_with_the_expected_options() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed

        assert_eq!(
            fakes.adc1.options(),
            Some(AdcOptions::new(&[POTENTIOMETER_CHANNEL], &ADC1_SAMPLES))
        );
        assert_eq!(
            fakes.pwm1.options(),
            Some(PwmOptions::new(
                PWM_FREQUENCY_HZ,
                PWM_CHANNELS,
                esc1_discovery::TIM1_DEAD_TIME,
                false
            ))
        );
    }

    #[test]
    fn step_always_applies_a_fixed_duty_cycle_regardless_of_the_potentiometer() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        fakes.adc1.set_sample(0, sample(0));
        firmware.step();
        assert_eq!(fakes.pwm1.duty_cycle(0), DUTY_CYCLE);

        fakes.adc1.set_sample(0, sample(u16::MAX));
        firmware.step();
        assert_eq!(fakes.pwm1.duty_cycle(0), DUTY_CYCLE);
    }

    #[test]
    fn step_derives_dead_time_from_the_inverted_potentiometer_reading() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        fakes.adc1.set_sample(0, sample(49_152)); // 0.75
        let expected = scale_duration(invert(sample(49_152)), DEAD_TIME_TUNING_MAX);

        assert_eq!(firmware.step(), expected);
        assert_eq!(fakes.pwm1.options().unwrap().dead_time(), expected);
    }

    #[test]
    fn step_inverts_the_potentiometer_reading_for_dead_time() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        // Fully one way on the pot (raw reading 0) should land at the
        // dead-time sweep's far end (just below DEAD_TIME_TUNING_MAX), not
        // its near end (0) -- i.e. the reading really is inverted, not
        // passed through.
        fakes.adc1.set_sample(0, sample(0));
        let dead_time = firmware.step();
        assert!(dead_time > Duration::from_nanos(400));
    }

    #[test]
    fn step_paces_itself_by_roughly_sample_period() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        firmware.step();
        let after_first_sample = fakes.clock_provider.now();

        firmware.step();
        let after_second_sample = fakes.clock_provider.now();

        let period = after_second_sample - after_first_sample;
        assert!(
            period >= Duration::from_millis(900) && period <= Duration::from_millis(1100),
            "expected ~{SAMPLE_PERIOD:?} between samples, got {period:?}"
        );
    }
}
