#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::duration::Duration;
use common::unit_interval::UnitInterval;
// `backend::x::Y` resolves to `peripherals::stm32g4::x::Y` on real
// hardware or `peripherals::fake::x::Y` elsewhere — see
// `esc1_discovery::backend`'s own doc comment. Used below for the
// backend-selected driver types stored in `Firmware`, instead of
// repeating the `#[cfg(target_arch = "arm")]` branch here too.
use esc1_discovery::backend::{adc::Adc, clock::ClockProvider};
use esc1_discovery::BoardPeripherals;
use peripherals::api::adc::{AdcOptions, AdcSampleBuffer, AdcTrait, AdcTriggerSource};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};

/// ADC1's channel wired to the B-G431B-ESC1's potentiometer
/// (`esc1_discovery::POTENTIOMETER_PIN`, PB12) — PB12 is ADC1_IN11, per the
/// STM32G431's pin table (RM0440/datasheet Table 12's ADCx_INy
/// annotations).
const POTENTIOMETER_CHANNEL: u8 = 11;

/// ADC1's channel wired to the B-G431B-ESC1's bus-voltage sense pin
/// (`esc1_discovery::VBUS_PIN`, PA0) — PA0 is ADC1_IN1.
const VBUS_CHANNEL: u8 = 1;

/// The sequence of conversions done with each ADC trigger.
const SEQUENCE: [u8; 2] = [POTENTIOMETER_CHANNEL, VBUS_CHANNEL];

/// ADC1's DMA destination buffer — `'static` so its address stays valid
/// for as long as ADC1 stays open, regardless of `Firmware` (and the
/// `Adc` inside it) getting moved around after `Firmware::new` calls
/// `open()` — see `peripherals::api::adc::AdcSampleBuffer`'s doc comment.
static ADC1_SAMPLES: AdcSampleBuffer = AdcSampleBuffer::new();

/// How often to sample the sequence.
const SAMPLE_PERIOD: Duration = Duration::from_millis(200);

/// The ADC's reference voltage (`VREF+`), tied to the B-G431B-ESC1's 3.3V
/// supply rail.
#[cfg(not(test))]
const VREF_VOLTS: f32 = 3.3;

/// [`VBUS_CHANNEL`] doesn't see the bus voltage directly — the board
/// schematic puts a resistor divider between it and `VBUS_PIN`, scaling
/// the real bus voltage down to `18k / (18k + 169k)` of itself before the
/// ADC sees it.
#[cfg(not(test))]
const VBUS_DIVIDER_LOW_OHMS: f32 = 18_000.0;
#[cfg(not(test))]
const VBUS_DIVIDER_HIGH_OHMS: f32 = 169_000.0;

/// Converts a raw [`VBUS_CHANNEL`] reading (a fraction of [`VREF_VOLTS`])
/// back into the actual bus voltage, undoing the divider's scale-down —
/// see [`VBUS_DIVIDER_LOW_OHMS`]/[`VBUS_DIVIDER_HIGH_OHMS`].
#[cfg(not(test))]
fn vbus_volts(vbus: UnitInterval) -> f32 {
    f32::from(vbus) * VREF_VOLTS * (VBUS_DIVIDER_LOW_OHMS + VBUS_DIVIDER_HIGH_OHMS)
        / VBUS_DIVIDER_LOW_OHMS
}

/// One reading of each channel in [`SEQUENCE`], as a fraction of the
/// ADC's full-scale reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Samples {
    potentiometer: UnitInterval,
    vbus: UnitInterval,
}

/// Everything the firmware does after chip bring-up, one sample at a time.
/// Kept separate from the `app` module below so it can be driven by unit
/// tests (see the `tests` module) without a real interrupt-driven
/// scheduler.
struct Firmware {
    adc1: Adc,
    clock_provider: ClockProvider,
}

impl Firmware {
    fn new(peripherals: BoardPeripherals) -> Self {
        let BoardPeripherals {
            mut adc1,
            clock_provider,
            dma,
            ..
        } = peripherals;
        adc1.open(
            AdcOptions::new(&SEQUENCE, &ADC1_SAMPLES, AdcTriggerSource::Software),
            &dma,
        );
        Firmware {
            adc1,
            clock_provider,
        }
    }

    /// Triggers one DMA-driven conversion of [`SEQUENCE`] and busy-waits
    /// for it to complete, then paces itself by [`SAMPLE_PERIOD`] before
    /// returning both raw readings.
    fn step(&mut self) -> Samples {
        // Keeps the clock's reference point fresh enough for `wait_for`'s
        // `now()` reads to stay accurate — see
        // `ClockProviderTrait::advance_reference_point`'s doc comment.
        self.clock_provider.advance_reference_point();
        let clock = self.clock_provider.get_clock();

        self.adc1.trigger();
        while !self.adc1.try_retrieve_result() {}
        let potentiometer = self.adc1.get_sample(0);
        let vbus = self.adc1.get_sample(1);

        #[cfg(not(test))]
        defmt::info!(
            "potentiometer: {}  vbus: {} V",
            f32::from(potentiometer),
            vbus_volts(vbus)
        );

        clock.wait_for(SAMPLE_PERIOD);
        Samples {
            potentiometer,
            vbus,
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
    pub use esc1_discovery::RTIC_PRIORITY_BITS as NVIC_PRIO_BITS;
    #[allow(unused_imports)]
    pub use stm32_metapac::*;
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

    /// Builds a `UnitInterval` that round-trips exactly through
    /// `peripherals::fake::adc`'s 16-bit-wide sample storage — see that
    /// module's own `sample()` test helper.
    fn sample(top_16_bits: u16) -> UnitInterval {
        UnitInterval::new((top_16_bits as u32) << 16)
    }

    #[test]
    fn opens_both_channels_as_one_sequence_on_init() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed

        assert_eq!(
            fakes.adc1.options(),
            Some(AdcOptions::new(
                &SEQUENCE,
                &ADC1_SAMPLES,
                AdcTriggerSource::Software
            ))
        );
    }

    #[test]
    fn reads_back_both_channels_independently() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        fakes.adc1.set_sample(0, sample(2731)); // potentiometer's rank
        fakes.adc1.set_sample(1, sample(1024)); // vbus's rank
        assert_eq!(
            firmware.step(),
            Samples {
                potentiometer: sample(2731),
                vbus: sample(1024),
            }
        );
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
            period >= Duration::from_millis(180) && period <= Duration::from_millis(220),
            "expected ~{SAMPLE_PERIOD:?} between samples, got {period:?}"
        );
    }
}
