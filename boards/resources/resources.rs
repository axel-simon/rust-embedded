//! Backend-selected `Peripherals`/`Peri` aliases: real hardware via
//! `embassy_stm32::init()` on the arm target, or a hand-rolled mirror
//! ([`fake_embassy`]) on host builds (so `cargo test` works without real
//! hardware). Both expose the identical shape — same `Peripherals` field
//! names, same `Peri<'d, T>` shape, same `peripherals::NAME` marker types —
//! so board code written against these aliases compiles unchanged either
//! way.
#![cfg_attr(not(test), no_std)]

#[cfg(not(target_arch = "arm"))]
mod fake_embassy;

#[cfg(target_arch = "arm")]
pub type Peripherals = embassy_stm32::Peripherals;
#[cfg(target_arch = "arm")]
pub type Peri<'d, T> = embassy_stm32::Peri<'d, T>;
#[cfg(target_arch = "arm")]
pub use embassy_stm32::peripherals;

#[cfg(not(target_arch = "arm"))]
pub type Peripherals = fake_embassy::Peripherals;
#[cfg(not(target_arch = "arm"))]
pub type Peri<'d, T> = fake_embassy::Peri<'d, T>;
#[cfg(not(target_arch = "arm"))]
pub use fake_embassy::peripherals;

/// Brings up the chip (real hardware) or a simulated one (host/tests) and
/// hands back ownership of every peripheral singleton.
#[cfg(target_arch = "arm")]
pub fn init() -> Peripherals {
    let p = embassy_stm32::init(embassy_stm32::Config::default());
    // embassy_stm32::init() only enables a peripheral's clock when its own
    // driver claims that peripheral; our register-level Gpio driver
    // (peripherals::stm32g4::gpio::Gpio) bypasses that entirely, so enable
    // every GPIO port's clock ourselves. Leading `::` to reach the
    // workspace `peripherals` crate rather than the `peripherals` module
    // re-exported above.
    ::peripherals::stm32g4::gpio::enable_gpio_clocks();
    p
}

#[cfg(not(target_arch = "arm"))]
pub fn init() -> Peripherals {
    fake_embassy::init()
}

#[cfg(all(test, not(target_arch = "arm")))]
mod tests {
    use super::*;

    #[test]
    fn init_hands_out_every_peripheral_once() {
        // Mainly a compile-time check (field access and struct
        // construction have to type-check for all 130 fields), plus a
        // sanity check that construction doesn't panic.
        let p = init();
        let _pa8: Peri<'static, peripherals::PA8> = p.PA8;
    }

    #[test]
    #[allow(non_snake_case)]
    fn claim_pins_registers_the_fake_pin_via_pin_token() {
        use ::peripherals::api::gpio::{GpioPin, GpioPort, GpioTrait};
        use ::peripherals::fake::gpio::GpioFake;

        let Peripherals { PA8, .. } = init();
        let mut gpio = GpioFake::new();
        ::peripherals::claim_pins!(gpio, PA8);

        // PA8 was claimed, so configuring it shouldn't warn...
        gpio.configure(GpioPin::input(GpioPort::PA, 8));
        assert_eq!(gpio.warning_count(), 0);

        // ...but PA9 was never claimed, so it should.
        gpio.configure(GpioPin::input(GpioPort::PA, 9));
        assert_eq!(gpio.warning_count(), 1);
    }
}
