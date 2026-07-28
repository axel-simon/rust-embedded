//! A hand-rolled mirror of an embassy-X backend's `Peripherals` type (and
//! the `peripherals` marker-type module it's built from) — e.g.
//! `embassy_stm32::Peripherals`, or whichever `embassy-*` crate a
//! different board in this workspace ends up targeting — used only on
//! host/test builds, where no real embassy backend exists.
//!
//! This isn't scoped to one chip or one embassy backend: the identifier
//! list passed to [`fake_peripherals!`] below is meant to hold the union
//! of every peripheral singleton any board's real backend exposes, so
//! more boards/chips/backends can add to it over time rather than needing
//! a fake of their own. Right now it only covers the B-G431B-ESC1's
//! STM32G431CB — extracted directly from the
//! `embassy_hal_internal::peripherals_struct!` invocation in
//! `embassy-stm32`'s own build-time-generated `_generated.rs` for that
//! exact chip+feature combination (found under
//! `target/*/build/embassy-stm32-*/out/_generated.rs` after a build) — 131
//! identifiers. Deliberately no `time-driver-*` feature enabled: that
//! would silently remove whichever timer it names from this list (see
//! `resources/Cargo.toml`), so this fake wouldn't even reflect what's
//! really available. Keep each board's contribution in sync by hand as
//! its chip, enabled features, or embassy backend change — same process
//! regardless of which `embassy-*` crate and generated file it comes from.

use core::marker::PhantomData;

// Leading `::` on every import below is load-bearing, not just style: this
// file's own `fake_peripherals!` macro (further down) defines a `pub mod
// peripherals`, which would otherwise shadow the extern crate `peripherals`
// these imports need — rustfmt is aware `::foo` and `foo` are usually
// equivalent and will "simplify" away the leading `::` if asked to
// reformat this block, so don't run it over this file.
use ::peripherals::api::adc::AdcInstance;
use ::peripherals::api::dma::DmaInstance;
use ::peripherals::api::gpio::GpioPort;
use ::peripherals::api::quadrature::QuadratureTimer;
use ::peripherals::fake::dma::DmaChannelToken;
use ::peripherals::fake::gpio::PinToken;

use crate::{AdcCapableInstance, QuadratureCapableTimer};

/// A minimal, self-contained stand-in for an embassy-X backend's
/// `Peri<'d, T>` (e.g. `embassy_stm32::Peri<'d, T>`): just enough shape (a
/// value plus a borrowed lifetime) for board code to hold one as a field.
/// Unlike the real thing, this doesn't need to enforce anything at
/// runtime — it exists purely so generic board code compiles the same way
/// whether it's naming `resources::Peri<'static,
/// resources::peripherals::PA8>` on real hardware or in a host-side test.
pub struct Peri<'d, T>(T, PhantomData<&'d mut T>);

impl<'d, T> Peri<'d, T> {
    pub const fn new(value: T) -> Self {
        Self(value, PhantomData)
    }
}

impl<'d, T> core::ops::Deref for Peri<'d, T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.0
    }
}

/// Lets `peripherals::fake::gpio::Gpio::claim_pin` recover a claimed
/// pin's identity through the `Peri` wrapper, given its inner marker type
/// (e.g. `peripherals::PA8` below) implements `PinToken`.
impl<'d, T: PinToken> PinToken for Peri<'d, T> {
    const PORT: GpioPort = T::PORT;
    const NUMBER: u8 = T::NUMBER;
}

/// See the `PinToken` impl above — the same, for
/// `peripherals::fake::dma::Dma::claim_channel` and `DmaChannelToken`.
impl<'d, T: DmaChannelToken> DmaChannelToken for Peri<'d, T> {
    const INSTANCE: DmaInstance = T::INSTANCE;
    const CHANNEL: u8 = T::CHANNEL;
}

macro_rules! fake_peripherals {
    ($($name:ident),+ $(,)?) => {
        /// Zero-sized marker types, one per peripheral singleton — mirrors
        /// the real embassy-X backend's own `peripherals` module.
        pub mod peripherals {
            $(
                #[doc = concat!("Fake stand-in for the real embassy-X backend's `peripherals::", stringify!($name), "` marker type.")]
                #[allow(non_camel_case_types)]
                #[derive(Debug, Clone, Copy)]
                pub struct $name;
            )+
        }

        /// Mirrors the real embassy-X backend's `Peripherals` struct
        /// field-for-field.
        #[allow(non_snake_case)]
        pub struct Peripherals {
            $(pub $name: Peri<'static, peripherals::$name>,)+
        }

        /// Mirrors the real embassy-X backend's `init()`: hands out one of
        /// every peripheral singleton, all pre-"claimed" since there's no
        /// real hardware to bring up.
        pub fn init() -> Peripherals {
            Peripherals {
                $($name: Peri::new(peripherals::$name),)+
            }
        }
    };
}

fake_peripherals!(
    // GPIO pins (42)
    PA0,
    PA1,
    PA2,
    PA3,
    PA4,
    PA5,
    PA6,
    PA7,
    PA8,
    PA9,
    PA10,
    PA11,
    PA12,
    PA13,
    PA14,
    PA15,
    PB0,
    PB1,
    PB2,
    PB3,
    PB4,
    PB5,
    PB6,
    PB7,
    PB8,
    PB9,
    PB10,
    PB11,
    PB12,
    PB13,
    PB14,
    PB15,
    PC4,
    PC6,
    PC10,
    PC11,
    PC13,
    PC14,
    PC15,
    PF0,
    PF1,
    PG10,
    // Analog (14)
    ADC1,
    ADC12_COMMON,
    ADC2,
    COMP1,
    COMP2,
    COMP3,
    COMP4,
    DAC1,
    DAC3,
    OPAMP1,
    OPAMP2,
    OPAMP3,
    VREFBUF,
    VREFINTCAL,
    // Timers (10)
    TIM1,
    TIM15,
    TIM16,
    TIM17,
    TIM2,
    TIM3,
    TIM4,
    TIM6,
    TIM7,
    TIM8,
    // DMA (3 controller/mux blocks + 12 channels)
    DMA1,
    DMA2,
    DMAMUX1,
    DMA1_CH1,
    DMA1_CH2,
    DMA1_CH3,
    DMA1_CH4,
    DMA1_CH5,
    DMA1_CH6,
    DMA2_CH1,
    DMA2_CH2,
    DMA2_CH3,
    DMA2_CH4,
    DMA2_CH5,
    DMA2_CH6,
    // Communication (15)
    I2C1,
    I2C2,
    I2C3,
    SPI1,
    SPI2,
    SPI3,
    USART1,
    USART2,
    USART3,
    UART4,
    LPUART1,
    USB,
    FDCAN1,
    FDCANRAM1,
    UCPD1,
    // EXTI lines (16)
    EXTI0,
    EXTI1,
    EXTI2,
    EXTI3,
    EXTI4,
    EXTI5,
    EXTI6,
    EXTI7,
    EXTI8,
    EXTI9,
    EXTI10,
    EXTI11,
    EXTI12,
    EXTI13,
    EXTI14,
    EXTI15,
    // Misc / system (19)
    CORDIC,
    CRC,
    CRS,
    DBGMCU,
    FLASH,
    FMAC,
    IWDG,
    LPTIM1,
    PWR,
    MCO,
    RCC,
    RNG,
    RTC,
    SAI1,
    SYSCFG,
    TAMP,
    UID,
    USBRAM,
    WWDG,
);

/// Implements `PinToken` for a batch of the GPIO marker types
/// [`fake_peripherals!`] generated above, each paired with its physical
/// `(port, pin_number)`. Only the 42 GPIO pins have a meaningful
/// port/pin identity — the other 88 peripheral singletons above don't get
/// an impl, since `Gpio::claim_pin` (the only consumer of this trait) is
/// never called with them.
macro_rules! fake_gpio_pins {
    ($($name:ident => ($port:ident, $number:expr)),+ $(,)?) => {
        $(
            impl PinToken for peripherals::$name {
                const PORT: GpioPort = GpioPort::$port;
                const NUMBER: u8 = $number;
            }
        )+
    };
}

fake_gpio_pins!(
    PA0 => (PA, 0), PA1 => (PA, 1), PA2 => (PA, 2), PA3 => (PA, 3), PA4 => (PA, 4),
    PA5 => (PA, 5), PA6 => (PA, 6), PA7 => (PA, 7), PA8 => (PA, 8), PA9 => (PA, 9),
    PA10 => (PA, 10), PA11 => (PA, 11), PA12 => (PA, 12), PA13 => (PA, 13), PA14 => (PA, 14),
    PA15 => (PA, 15),
    PB0 => (PB, 0), PB1 => (PB, 1), PB2 => (PB, 2), PB3 => (PB, 3), PB4 => (PB, 4),
    PB5 => (PB, 5), PB6 => (PB, 6), PB7 => (PB, 7), PB8 => (PB, 8), PB9 => (PB, 9),
    PB10 => (PB, 10), PB11 => (PB, 11), PB12 => (PB, 12), PB13 => (PB, 13), PB14 => (PB, 14),
    PB15 => (PB, 15),
    PC4 => (PC, 4), PC6 => (PC, 6), PC10 => (PC, 10), PC11 => (PC, 11), PC13 => (PC, 13),
    PC14 => (PC, 14), PC15 => (PC, 15),
    PF0 => (PF, 0), PF1 => (PF, 1),
    PG10 => (PG, 10),
);

/// See `fake_gpio_pins!` above — the same, for `DmaChannelToken` and the
/// 12 DMA channel marker types `fake_peripherals!` generated. Only those
/// 12 have a meaningful `(instance, channel)` identity — `DMA1`/`DMA2`/
/// `DMAMUX1` themselves don't, since `Dma::claim_channel` (the only
/// consumer of this trait) is never called with them.
macro_rules! fake_dma_channels {
    ($($name:ident => ($instance:ident, $channel:expr)),+ $(,)?) => {
        $(
            impl DmaChannelToken for peripherals::$name {
                const INSTANCE: DmaInstance = DmaInstance::$instance;
                const CHANNEL: u8 = $channel;
            }
        )+
    };
}

fake_dma_channels!(
    DMA1_CH1 => (Stm32g4Dma1, 1), DMA1_CH2 => (Stm32g4Dma1, 2), DMA1_CH3 => (Stm32g4Dma1, 3),
    DMA1_CH4 => (Stm32g4Dma1, 4), DMA1_CH5 => (Stm32g4Dma1, 5), DMA1_CH6 => (Stm32g4Dma1, 6),
    DMA2_CH1 => (Stm32g4Dma2, 1), DMA2_CH2 => (Stm32g4Dma2, 2), DMA2_CH3 => (Stm32g4Dma2, 3),
    DMA2_CH4 => (Stm32g4Dma2, 4), DMA2_CH5 => (Stm32g4Dma2, 5), DMA2_CH6 => (Stm32g4Dma2, 6),
);

/// See `fake_gpio_pins!`/`fake_dma_channels!` above — the same, for
/// `crate::QuadratureCapableTimer` and the timer marker types
/// `fake_peripherals!` generated. Matches `stm32g4.rs`'s real-hardware
/// impls exactly (see its comment for why `TIM5`/`TIM20`/`TIM2` are
/// absent here too).
macro_rules! fake_quadrature_timers {
    ($($name:ident => $variant:ident),+ $(,)?) => {
        $(
            impl QuadratureCapableTimer for peripherals::$name {
                const TIMER: QuadratureTimer = QuadratureTimer::$variant;
            }
        )+
    };
}

fake_quadrature_timers!(
    TIM1 => Stm32g4Tim1, TIM3 => Stm32g4Tim3, TIM4 => Stm32g4Tim4, TIM8 => Stm32g4Tim8,
);

/// See `fake_gpio_pins!`/`fake_dma_channels!`/`fake_quadrature_timers!`
/// above — the same, for `crate::AdcCapableInstance` and the ADC marker
/// types `fake_peripherals!` generated. Matches `stm32g4.rs`'s
/// real-hardware impls exactly (see its comment for why `ADC3`/`ADC4`/
/// `ADC5` are absent here too).
macro_rules! fake_adc_instances {
    ($($name:ident => $variant:ident),+ $(,)?) => {
        $(
            impl AdcCapableInstance for peripherals::$name {
                const INSTANCE: AdcInstance = AdcInstance::$variant;
            }
        )+
    };
}

fake_adc_instances!(
    ADC1 => Stm32g4Adc1, ADC2 => Stm32g4Adc2,
);

// See boards/resources/resources.rs for tests — they exercise this module
// only through the public `Peripherals`/`init`/`peripherals` aliases
// `resources.rs` re-exports, the same surface board code actually uses.
