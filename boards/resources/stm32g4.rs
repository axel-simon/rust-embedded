//! Real hardware: `embassy_stm32::init()`-backed `Peripherals`/`Peri`
//! aliases, and the STM32G4-specific PLL configuration [`init`] computes
//! from a board's [`crate::ClockConfiguration`].

use crate::{AdcCapableInstance, ClockConfiguration, PwmCapableTimer, QuadratureCapableTimer};

// Neither is referenced by name — `defmt-rtt` registers the RTT logging
// backend `defmt::info!` calls dispatch through, and `panic-probe` registers
// the `#[panic_handler]`. This crate is a good home for them since it's the
// one that already knows this is real hardware, and (being genuinely used)
// actually gets linked into the final binary — see `Cargo.toml`'s comment on
// these two dependencies.
use {defmt_rtt as _, panic_probe as _};

pub type Peripherals = embassy_stm32::Peripherals;
pub type Peri<'d, T> = embassy_stm32::Peri<'d, T>;
pub use embassy_stm32::peripherals;

// The core MCU interface a board hands to whatever's driving it (RTIC, on
// this workspace's real hardware) — on STM32, this is Cortex-M's own core
// peripherals.
pub type McuInterface = cortex_m::Peripherals;

/// Brings up the chip and hands back ownership of every peripheral
/// singleton.
pub fn init(clock_configuration: ClockConfiguration) -> Peripherals {
    let config = clock_config(
        clock_configuration.oscillator_frequency,
        clock_configuration.mcu_frequency,
        clock_configuration.timepiece_is_crystal,
    );
    embassy_stm32::init(config)
}

/// Computes an `embassy_stm32::Config` that brings the STM32G4's PLL up
/// from `oscillator_frequency` (the HSE timepiece) to `mcu_frequency`
/// (SYSCLK/HCLK), searching for integer M/N/R divider values within the
/// datasheet's PLL input/VCO/output frequency ranges (RM0440 p280-282,
/// STM32G474 datasheet Table 46):
/// - PLL input (`oscillator_frequency / M`): 2.66 MHz – 16 MHz
/// - VCO (`PLL input * N`): 96 MHz – 344 MHz
/// - PLL R output (`VCO / R`), used here as SYSCLK/HCLK: 8 MHz – 170 MHz
///
/// `timepiece_is_crystal` selects `HseMode::Oscillator` (a crystal, driven
/// through the MCU's own oscillator amplifier across `OSC_IN`/`OSC_OUT`)
/// vs `HseMode::Bypass` (an external active oscillator module feeding
/// `OSC_IN` alone, `OSC_OUT` unused) — see [`ClockConfiguration`].
///
/// Panics if no M/N/R combination exists (`embassy_stm32::init()` would
/// also assert these ranges itself when applying the resulting `Config`,
/// but a bad board-frequency combination is a programming error worth
/// catching here, with a clearer message, rather than there).
fn clock_config(
    oscillator_frequency: u32,
    mcu_frequency: u32,
    timepiece_is_crystal: bool,
) -> embassy_stm32::Config {
    use core::ops::RangeInclusive;

    use embassy_stm32::rcc::{Hse, HseMode, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
    use embassy_stm32::time::Hertz;

    const PLL_IN_RANGE: RangeInclusive<u64> = 2_660_000..=16_000_000;
    const PLL_VCO_RANGE: RangeInclusive<u64> = 96_000_000..=344_000_000;
    const PLL_N_RANGE: RangeInclusive<u64> = 8..=127;
    const PLL_R_DIVIDERS: [u32; 4] = [2, 4, 6, 8];

    let oscillator_frequency_u64 = oscillator_frequency as u64;

    // Searched with exact integer arithmetic throughout — `M` doesn't need
    // to evenly divide `oscillator_frequency` in whole Hz (a real PLL
    // divider produces a fractional-Hz `PLL input` just fine); what must
    // be exact is the overall `oscillator_frequency / M * N / R ==
    // mcu_frequency` relationship, checked here as
    // `mcu_frequency * R * M == oscillator_frequency * N`.
    let (pllm, plln, pllr) = PLL_R_DIVIDERS
        .into_iter()
        .find_map(|r| {
            let vco = mcu_frequency as u64 * r as u64;
            if !PLL_VCO_RANGE.contains(&vco) {
                return None;
            }
            (1..=16u32).find_map(|m| {
                if oscillator_frequency_u64 < PLL_IN_RANGE.start() * m as u64
                    || oscillator_frequency_u64 > PLL_IN_RANGE.end() * m as u64
                {
                    return None;
                }
                let numerator = vco * m as u64;
                if !numerator.is_multiple_of(oscillator_frequency_u64) {
                    return None;
                }
                let n = numerator / oscillator_frequency_u64;
                PLL_N_RANGE.contains(&n).then_some((m, n as u32, r))
            })
        })
        .unwrap_or_else(|| {
            panic!(
                "no STM32G4 PLL M/N/R combination reaches {mcu_frequency} Hz from an \
                 {oscillator_frequency} Hz oscillator within the datasheet's PLL constraints"
            )
        });

    let pllr_div = match pllr {
        2 => PllRDiv::DIV2,
        4 => PllRDiv::DIV4,
        6 => PllRDiv::DIV6,
        8 => PllRDiv::DIV8,
        _ => unreachable!("PLL_R_DIVIDERS only contains 2/4/6/8"),
    };

    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(oscillator_frequency),
        mode: if timepiece_is_crystal {
            HseMode::Oscillator
        } else {
            HseMode::Bypass
        },
    });
    config.rcc.pll = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::from_bits((pllm - 1) as u8),
        mul: PllMul::from_bits(plln as u8),
        divp: None,
        divq: None,
        divr: Some(pllr_div),
    });
    config.rcc.sys = Sysclk::PLL1_R;
    config
}

/// Unifies a peripheral driver's `new()`'s differing return shape between
/// a real driver (returns a bare value) and its fake counterpart (returns
/// `(value, fake_handle)`, so a test can get a handle onto the same
/// simulated peripheral firmware gets) — so board init code can write
/// `let (value, fake) = split_off_fake(SomeDriver::new(...));`
/// unconditionally.
pub fn split_off_fake<T>(value: T) -> (T, ()) {
    (value, ())
}

/// Claims `timer` (consuming its `Peri` ownership token — the same
/// ownership enforcement `peripherals::claim_pins!`/`Dma::claim_channel`
/// give GPIO pins/DMA channels) and resolves which
/// [`peripherals::api::quadrature::QuadratureTimer`](::peripherals::api::quadrature::QuadratureTimer)
/// it names, ready to pass to
/// `peripherals::stm32g4::quadrature::Quadrature::new`. `T:
/// QuadratureCapableTimer` is what makes passing an incompatible timer
/// resource a compile error — see that trait's doc comment. Defined here
/// (rather than once, backend-agnostically, in resources.rs) for the same
/// reason [`split_off_fake`] is: `Peri<'static, T>` (the real
/// `embassy_stm32::Peri`) additionally requires `T:
/// embassy_stm32::PeripheralType`, a bound the fake `Peri` has no
/// equivalent of at all — see `fake.rs`'s own `claim_quadrature_timer`.
pub fn claim_quadrature_timer<T: QuadratureCapableTimer + embassy_stm32::PeripheralType>(
    _timer: Peri<'static, T>,
) -> ::peripherals::api::quadrature::QuadratureTimer {
    T::TIMER
}

// `QuadratureCapableTimer` impls for the real STM32G4 timer instances
// `peripherals::stm32g4::quadrature` actually supports (see its doc
// comment) — matches `QuadratureTimer`'s own variant list, minus
// `Stm32g4Tim5`/`Stm32g4Tim20` (not physically present as
// `embassy_stm32::peripherals` types on the `stm32g431cb` chip feature
// this crate currently targets — see resources/Cargo.toml) and
// `Stm32g4Tim2` (never supported at all).
impl QuadratureCapableTimer for embassy_stm32::peripherals::TIM1 {
    const TIMER: ::peripherals::api::quadrature::QuadratureTimer =
        ::peripherals::api::quadrature::QuadratureTimer::Stm32g4Tim1;
}
impl QuadratureCapableTimer for embassy_stm32::peripherals::TIM3 {
    const TIMER: ::peripherals::api::quadrature::QuadratureTimer =
        ::peripherals::api::quadrature::QuadratureTimer::Stm32g4Tim3;
}
impl QuadratureCapableTimer for embassy_stm32::peripherals::TIM4 {
    const TIMER: ::peripherals::api::quadrature::QuadratureTimer =
        ::peripherals::api::quadrature::QuadratureTimer::Stm32g4Tim4;
}
impl QuadratureCapableTimer for embassy_stm32::peripherals::TIM8 {
    const TIMER: ::peripherals::api::quadrature::QuadratureTimer =
        ::peripherals::api::quadrature::QuadratureTimer::Stm32g4Tim8;
}

/// Claims `adc` (consuming its `Peri` ownership token — the same ownership
/// enforcement `peripherals::claim_pins!`/`Dma::claim_channel` give GPIO
/// pins/DMA channels) and resolves which
/// [`peripherals::api::adc::AdcInstance`](::peripherals::api::adc::AdcInstance)
/// it names, ready to pass to `peripherals::stm32g4::adc::Adc::new`. `T:
/// AdcCapableInstance` is what makes passing an unsupported ADC resource a
/// compile error — see that trait's doc comment. Defined here (rather than
/// once, backend-agnostically, in resources.rs) for the same reason
/// [`claim_quadrature_timer`] is: `Peri<'static, T>` (the real
/// `embassy_stm32::Peri`) additionally requires `T:
/// embassy_stm32::PeripheralType`, a bound the fake `Peri` has no
/// equivalent of at all — see `fake.rs`'s own `claim_adc`.
pub fn claim_adc<T: AdcCapableInstance + embassy_stm32::PeripheralType>(
    _adc: Peri<'static, T>,
) -> ::peripherals::api::adc::AdcInstance {
    T::INSTANCE
}

// `AdcCapableInstance` impls for the real STM32G4 ADC instances
// `peripherals::stm32g4::adc` actually supports on this crate's
// `stm32g431cb` embassy-stm32 feature (see resources/Cargo.toml) —
// `ADC3`/`ADC4`/`ADC5` aren't generated as `embassy_stm32::peripherals`
// types at all on this chip, the same reason `QuadratureCapableTimer` has
// no `TIM5`/`TIM20` impls above.
impl AdcCapableInstance for embassy_stm32::peripherals::ADC1 {
    const INSTANCE: ::peripherals::api::adc::AdcInstance =
        ::peripherals::api::adc::AdcInstance::Stm32g4Adc1;
}
impl AdcCapableInstance for embassy_stm32::peripherals::ADC2 {
    const INSTANCE: ::peripherals::api::adc::AdcInstance =
        ::peripherals::api::adc::AdcInstance::Stm32g4Adc2;
}

/// Claims `timer` and resolves which
/// [`peripherals::api::pwm::PwmTimer`](::peripherals::api::pwm::PwmTimer)
/// it names, ready to pass to this workspace's real PWM driver's
/// constructor — see [`claim_quadrature_timer`]'s doc comment for why
/// this is defined here, per backend.
pub fn claim_pwm_timer<T: PwmCapableTimer + embassy_stm32::PeripheralType>(
    _timer: Peri<'static, T>,
) -> ::peripherals::api::pwm::PwmTimer {
    T::TIMER
}

// `PwmCapableTimer` impls for the real STM32G4 timer instances this
// workspace's PWM driver actually supports — `TIM20` is absent for the
// same reason it's absent from `QuadratureCapableTimer` above: not
// physically present on the `stm32g431cb` chip feature this crate
// currently targets.
impl PwmCapableTimer for embassy_stm32::peripherals::TIM1 {
    const TIMER: ::peripherals::api::pwm::PwmTimer = ::peripherals::api::pwm::PwmTimer::Stm32g4Tim1;
}
impl PwmCapableTimer for embassy_stm32::peripherals::TIM8 {
    const TIMER: ::peripherals::api::pwm::PwmTimer = ::peripherals::api::pwm::PwmTimer::Stm32g4Tim8;
}
