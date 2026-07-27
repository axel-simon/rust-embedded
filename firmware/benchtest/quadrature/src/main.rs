#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::duration::Duration;
use common::unit_interval::UnitInterval;
use esc1_discovery::{BoardPeripherals, ClockProvider};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::quadrature::{
    QuadratureInputConfiguration, QuadratureOptions, QuadratureTimer, QuadratureTrait,
};

/// Backend-selected [`QuadratureTrait`] driver — real hardware on
/// `target_arch = "arm"`, a fake elsewhere (so host-side `cargo test`
/// works without hardware). Local to this firmware (unlike
/// `esc1_discovery::Gpio`/`ClockProvider`/`Adc`, which live in the board
/// crate): `encoder_counts`/[`INPUT_CONFIGURATION`] are this benchtest's
/// own choices, not a fixed board fact the way e.g. ADC1's existence is —
/// see `esc1_discovery::TIM4_CH1_PIN`/`TIM4_CH2_PIN`/`TIM4_CH3_PIN` for the
/// pin routing itself, which *does* live in the board crate.
#[cfg(target_arch = "arm")]
type Quadrature = peripherals::stm32g4::quadrature::Quadrature;
#[cfg(not(target_arch = "arm"))]
type Quadrature = peripherals::fake::quadrature::Quadrature;
#[cfg(not(target_arch = "arm"))]
type FakeQuadrature = peripherals::fake::quadrature::FakeQuadrature;

/// Channel 1 (`esc1_discovery::TIM4_CH1_PIN`, PB6) carries input A,
/// channel 2 (`esc1_discovery::TIM4_CH2_PIN`, PB7) carries input B — no
/// swap needed.
const INPUT_CONFIGURATION: QuadratureInputConfiguration =
    QuadratureInputConfiguration::Ch12AreInputsAB;

/// The encoder under test has 128 steps per revolution.
const ENCODER_COUNTS: u32 = 128;

/// How often to sample and log the encoder's position.
const SAMPLE_PERIOD: Duration = Duration::from_millis(100);

/// Everything the firmware does after chip bring-up, one sample at a time.
/// Kept separate from the `app` module below so it can be driven by unit
/// tests (see the `tests` module) without a real interrupt-driven
/// scheduler.
struct Firmware {
    quadrature: Quadrature,
    clock_provider: ClockProvider,
    #[cfg(not(target_arch = "arm"))]
    fake_quadrature: FakeQuadrature,
}

impl Firmware {
    fn new(peripherals: BoardPeripherals) -> Self {
        let BoardPeripherals { clock_provider, .. } = peripherals;

        // `esc1_discovery::initialize` (called by `init` below, before
        // this runs) already configured PB6/PB7/PB8 as TIM4's channel
        // inputs — see `esc1_discovery::TIM4_CH1_PIN`/`TIM4_CH2_PIN`/
        // `TIM4_CH3_PIN`.
        #[cfg(target_arch = "arm")]
        let mut quadrature = Quadrature::new(QuadratureTimer::Stm32g4Tim4);
        #[cfg(not(target_arch = "arm"))]
        let (mut quadrature, fake_quadrature) = Quadrature::new(QuadratureTimer::Stm32g4Tim4);

        quadrature.open(QuadratureOptions {
            input_configuration: INPUT_CONFIGURATION,
            encoder_counts: ENCODER_COUNTS,
        });

        Firmware {
            quadrature,
            clock_provider,
            #[cfg(not(target_arch = "arm"))]
            fake_quadrature,
        }
    }

    /// Reads the encoder's current position, logs it, then paces itself by
    /// [`SAMPLE_PERIOD`] before returning it.
    fn step(&mut self) -> UnitInterval {
        // Keeps the clock's reference point fresh enough for `wait_for`'s
        // `now()` reads to stay accurate — see
        // `ClockProviderTrait::advance_reference_point`'s doc comment.
        self.clock_provider.advance_reference_point();
        let clock = self.clock_provider.get_clock();

        let position = self.quadrature.position();

        #[cfg(not(test))]
        defmt::info!("encoder position: {}", f32::from(position));

        clock.wait_for(SAMPLE_PERIOD);
        position
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
        #[cfg(not(target_arch = "arm"))]
        pub(crate) fake_quadrature: super::FakeQuadrature,
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
        // Cheap: `Rc`-backed, same as `fakes` above — cloned out of
        // `firmware` (rather than out of `peripherals` like `fakes` is)
        // since it's `Firmware::new` itself that constructs the
        // `Quadrature`/`FakeQuadrature` pair (see `Quadrature`'s doc
        // comment on why it isn't part of `BoardPeripherals`).
        #[cfg(not(target_arch = "arm"))]
        let fake_quadrature = firmware.fake_quadrature.clone();

        (
            Shared {},
            Local {
                firmware,
                #[cfg(not(target_arch = "arm"))]
                fakes,
                #[cfg(not(target_arch = "arm"))]
                fake_quadrature,
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
    fn starts_at_position_zero() {
        let (_shared, local) = app::init(app::init::Context);
        let mut firmware = local.firmware;

        assert_eq!(firmware.step(), UnitInterval::new(0));
    }

    #[test]
    fn step_reflects_the_fake_encoders_reading() {
        let (_shared, local) = app::init(app::init::Context);
        let mut fake_quadrature = local.fake_quadrature.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        fake_quadrature.set_encoder_reading(64); // halfway around 128 steps
        let position = firmware.step();
        assert_eq!(f32::from(position), 0.5);
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
            period >= Duration::from_millis(80) && period <= Duration::from_millis(120),
            "expected ~{SAMPLE_PERIOD:?} between samples, got {period:?}"
        );
    }
}
