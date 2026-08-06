//! Host/test builds: a hand-rolled mirror of `embassy_stm32::Peripherals`
//! ([`fake_embassy`]), so `cargo test` works without real hardware.

// `fake_embassy.rs` stays a sibling of `resources.rs` (this crate's root)
// rather than moving into a `fake/` subdirectory, so an explicit `#[path]`
// is needed here — this module isn't the crate root, so plain `mod
// fake_embassy;` would otherwise look for `fake/fake_embassy.rs`.
#[path = "fake_embassy.rs"]
mod fake_embassy;

use crate::ClockConfiguration;

pub type Peripherals = fake_embassy::Peripherals;
pub type Peri<'d, T> = fake_embassy::Peri<'d, T>;
pub use fake_embassy::peripherals;

/// See `stm32g4::McuInterface`'s doc comment — there's no real
/// `cortex_m::Peripherals` to hand a board's `initialize()` here, so it's
/// just unit.
pub type McuInterface = ();

/// Brings up a simulated chip and hands back ownership of every
/// peripheral singleton. Ignores `_clock_configuration` — there's no real
/// clock tree to configure here.
pub fn init(_clock_configuration: ClockConfiguration) -> Peripherals {
    fake_embassy::init()
}

/// See `stm32g4::split_off_fake`'s doc comment for why this exists — here,
/// a fake peripheral driver's `new()` already returns
/// `(value, fake_handle)`, so this is just the identity function.
pub fn split_off_fake<T, Fake>(pair: (T, Fake)) -> (T, Fake) {
    pair
}

/// See `stm32g4::claim_quadrature_timer`'s doc comment for why this is
/// defined separately per backend. The fake `Peri` (unlike the real
/// `embassy_stm32::Peri`) has no extra trait bound on `T` at all, so this
/// only needs `T: QuadratureCapableTimer`.
pub fn claim_quadrature_timer<T: crate::QuadratureCapableTimer>(
    _timer: Peri<'static, T>,
) -> ::peripherals::api::quadrature::QuadratureTimer {
    T::TIMER
}

/// See `stm32g4::claim_adc`'s doc comment for why this is defined
/// separately per backend. The fake `Peri` (unlike the real
/// `embassy_stm32::Peri`) has no extra trait bound on `T` at all, so this
/// only needs `T: AdcCapableInstance`.
pub fn claim_adc<T: crate::AdcCapableInstance>(
    _adc: Peri<'static, T>,
) -> ::peripherals::api::adc::AdcInstance {
    T::INSTANCE
}

/// This fake `Peri` needs no extra trait bound on `T` (unlike the real
/// backend's `embassy_stm32::Peri`, which additionally requires `T:
/// embassy_stm32::PeripheralType`), so this is defined separately here
/// with just the `T: PwmCapableTimer` bound, rather than once,
/// backend-agnostically.
pub fn claim_pwm_timer<T: crate::PwmCapableTimer>(
    _timer: Peri<'static, T>,
) -> ::peripherals::api::pwm::PwmTimer {
    T::TIMER
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_hands_out_every_peripheral_once() {
        // Mainly a compile-time check (field access and struct
        // construction have to type-check for all 130 fields), plus a
        // sanity check that construction doesn't panic.
        let p = init(ClockConfiguration {
            mcu_frequency: 170_000_000,
            oscillator_frequency: 8_000_000,
            timepiece_is_crystal: true,
        });
        let _pa8: Peri<'static, peripherals::PA8> = p.PA8;
    }

    #[test]
    #[allow(non_snake_case)]
    fn claim_pins_registers_the_fake_pin_via_pin_token() {
        // Leading `::` load-bearing here, not style — see fake_embassy.rs's
        // top-of-file comment: `use super::*;` above pulls this module's
        // own re-exported `peripherals` (the marker-type module) into
        // scope, shadowing the extern crate of the same name.
        use ::peripherals::api::gpio::{GpioPin, GpioPort};
        use ::peripherals::fake::gpio::Gpio;

        let Peripherals { PA8, .. } = init(ClockConfiguration {
            mcu_frequency: 170_000_000,
            oscillator_frequency: 8_000_000,
            timepiece_is_crystal: true,
        });
        let (mut gpio, fake) = Gpio::new();
        ::peripherals::claim_pins!(gpio, PA8);

        // PA8 was claimed via the macro...
        assert!(fake.pin_registered(GpioPin::input(GpioPort::PA, 8)));
        // ...but PA9 was never claimed.
        assert!(!fake.pin_registered(GpioPin::input(GpioPort::PA, 9)));
    }
}
