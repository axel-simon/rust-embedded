//! Real hardware: `embassy_stm32::init()`-backed `Peripherals`/`Peri`
//! aliases, and the STM32G4-specific PLL configuration [`init`] computes
//! from a board's [`crate::ClockConfiguration`].

use crate::ClockConfiguration;

pub type Peripherals = embassy_stm32::Peripherals;
pub type Peri<'d, T> = embassy_stm32::Peri<'d, T>;
pub use embassy_stm32::peripherals;

/// Brings up the chip and hands back ownership of every peripheral
/// singleton.
///
/// `embassy_stm32::init()` only enables a peripheral's clock when its own
/// driver claims that peripheral; the board's register-level `Gpio`
/// driver bypasses that entirely, so `Gpio::new()` enables every GPIO
/// port's clock itself, rather than this doing it up front.
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
                if numerator % oscillator_frequency_u64 != 0 {
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
/// unconditionally, for any peripheral driver following this pattern
/// (e.g. `peripherals::stm32g4::gpio::Gpio`,
/// `peripherals::stm32g4::clock::ClockProvider`), without its own
/// `target_arch` branch for this.
pub fn split_off_fake<T>(value: T) -> (T, ()) {
    (value, ())
}
