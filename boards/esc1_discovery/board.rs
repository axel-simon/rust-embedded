//! Pin map for the B-G431B-ESC1 Discovery kit (electronic speed controller
//! for drones, MB1419, target STM32G431CBU6).
//!
//! Names and pin assignments come from Table 4 "Main board STM32G431CB
//! pinout for motor control" in UM2516 (the kit's user manual).
#![cfg_attr(not(test), no_std)]

use common::uptime::Uptime;
use peripherals::api::gpio::{GpioPin, GpioPort, GpioTrait};

// Motor phase PWM (TIM1): CH1/CH2/CH3 drive the high side (AF6),
// CH1N/CH2N/CH3N the complementary low side (AF4) — one pair per phase U/V/W.
pub const TIM1_CH1_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 8, 6).with_very_high_speed();
pub const TIM1_CH1N_PIN: GpioPin = GpioPin::alternate(GpioPort::PC, 13, 4).with_very_high_speed();
pub const TIM1_CH2_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 9, 6).with_very_high_speed();
pub const TIM1_CH2N_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 12, 6).with_very_high_speed();
pub const TIM1_CH3_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 10, 6).with_very_high_speed();
pub const TIM1_CH3N_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 15, 4).with_very_high_speed();

// Per-phase current shunt feedback through OPAMP1/2/3 (analog in/out).
pub const CURRENT_FEEDBACK1_OPAMP_P_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 1);
pub const CURRENT_FEEDBACK1_OPAMP_N_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 3);
pub const OPAMP1_OUT_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 2);
pub const CURRENT_FEEDBACK2_OPAMP_P_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 7);
pub const CURRENT_FEEDBACK2_OPAMP_N_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 5);
pub const OPAMP2_OUT_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 6);
pub const CURRENT_FEEDBACK3_OPAMP_P_PIN: GpioPin = GpioPin::analog(GpioPort::PB, 0);
pub const CURRENT_FEEDBACK3_OPAMP_N_PIN: GpioPin = GpioPin::analog(GpioPort::PB, 2);

// Back-EMF sensing: per-phase analog taps plus the zero-crossing comparator's
// digital output.
pub const BACK_EMF1_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 4);
pub const BACK_EMF2_PIN: GpioPin = GpioPin::analog(GpioPort::PC, 4);
pub const BACK_EMF3_PIN: GpioPin = GpioPin::analog(GpioPort::PB, 11);
pub const GPIO_BACK_EMF_PIN: GpioPin = GpioPin::input(GpioPort::PB, 5);

// Hall-effect sensor inputs (J8).
pub const HALL_A_PIN: GpioPin = GpioPin::input(GpioPort::PB, 6);
pub const HALL_B_PIN: GpioPin = GpioPin::input(GpioPort::PB, 7);
pub const HALL_Z_PIN: GpioPin = GpioPin::input(GpioPort::PB, 8);

// CAN bus (FDCAN1_RX/TX, AF9) plus its termination switch and transceiver
// shutdown/TP2.
pub const CAN_RX_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 11, 9);
pub const CAN_TX_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 9, 9).with_high_speed();
pub const CAN_TERM_PIN: GpioPin = GpioPin::output(GpioPort::PC, 14);
pub const CAN_SHUTDOWN_PIN: GpioPin = GpioPin::inverted_output(GpioPort::PC, 11);

// UART2 (AF7): TX for telemetry, RX for firmware update, both on J3.
pub const USART2_TX_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 3, 7);
pub const USART2_RX_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 4, 7);

// External PWM input for motor speed regulation (J3); inferred as TIM2_CH1
// (AF1) since UM2516 only names it generically as "PWM". Only the pin mux
// is set up here — no capture logic exists yet, and TIM2 itself isn't
// claimed by anything (see resources/Cargo.toml — no `time-driver-*`
// feature), so it's free for an actual input-capture implementation.
pub const PWM_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 15, 1).with_pull_down();

// Battery/potentiometer/NTC analog sense lines.
pub const VBUS_PIN: GpioPin = GpioPin::analog(GpioPort::PA, 0);
pub const POTENTIOMETER_PIN: GpioPin = GpioPin::analog(GpioPort::PB, 12);
pub const TEMP_FEEDBACK_PIN: GpioPin = GpioPin::analog(GpioPort::PB, 14);

// User interface: status LED and button, both on the daughterboard.
pub const STATUS_PIN: GpioPin = GpioPin::output(GpioPort::PC, 6);
pub const BUTTON_PIN: GpioPin = GpioPin::inverted_input(GpioPort::PC, 10).with_pull_up();

// Unused.
pub const TP3_PIN: GpioPin = GpioPin::analog(GpioPort::PB, 1);

/// This board's HSE crystal — see the [`Self::PF0`]/[`Self::PF1`] doc
/// comments below ("HSE crystal input/output, 8 MHz"). Fed to
/// [`resources::init`] (via [`resources::ClockConfiguration`]) to compute
/// the PLL configuration that reaches [`MCU_FREQUENCY`].
pub const OSCILLATOR_FREQUENCY: u32 = 8_000_000;

/// The CPU (HCLK/SYSCLK) frequency [`initialize`] configures the PLL to
/// reach from [`OSCILLATOR_FREQUENCY`], and what
/// [`peripherals::stm32g4::clock::ClockProvider::new`]'s DWT-cycle-counter
/// conversions assume the chip is actually running at. Also what the fake
/// clock simulates (see `peripherals::fake::clock`'s `TICKS_PER_SECOND`),
/// so host/test builds behave consistently with real hardware.
pub const MCU_FREQUENCY: u32 = 170_000_000;

/// This board's HSE timepiece is an actual crystal (across `PF0`/`PF1`,
/// both pins in use) rather than an external active oscillator module
/// (which would only drive `PF0`, leaving `PF1` unused) — see
/// [`resources::ClockConfiguration::timepiece_is_crystal`].
pub const OSCILLATOR_IS_CRYSTAL: bool = true;

/// Number of priority levels this MCU's interrupt controller implements,
/// as a power-of-two exponent — RTIC needs this to compute its
/// priority-masking limits (see `src/main.rs`'s `rtic_device` shim, which
/// re-exports this under the name RTIC's Cortex-M backend looks for).
/// Named processor-agnostically, rather than after the NVIC (Cortex-M's
/// interrupt controller) specifically, since this workspace might target a
/// non-ARM MCU some day. This STM32G431 is a Cortex-M4 part, which — like
/// every non-Cortex-M0 STM32 family — implements 4 priority bits (16
/// levels); see RM0440. Re-verify against the relevant reference manual if
/// this board file is ever adapted to a different chip/architecture.
pub const RTIC_PRIORITY_BITS: u8 = 4;

/// Backend-selected [`peripherals::api::gpio::GpioTrait`] driver — see
/// [`initialize`]. Real hardware on `target_arch = "arm"`, a fake elsewhere
/// (so host-side `cargo test` works without hardware).
#[cfg(target_arch = "arm")]
pub type Gpio = peripherals::stm32g4::gpio::Gpio;
#[cfg(not(target_arch = "arm"))]
pub type Gpio = peripherals::fake::gpio::Gpio;

/// Backend-selected [`peripherals::api::clock::ClockProviderTrait`]
/// implementation — see [`initialize`]. Real hardware on
/// `target_arch = "arm"`, a fake elsewhere (so host-side `cargo test` works
/// without hardware).
#[cfg(target_arch = "arm")]
pub type ClockProvider = peripherals::stm32g4::clock::ClockProvider;
#[cfg(not(target_arch = "arm"))]
pub type ClockProvider = peripherals::fake::clock::ClockProvider;

/// A test's handle onto the same simulated [`Gpio`] firmware gets — see
/// [`peripherals::fake::gpio::FakeGpio`]. Only exists on host/test builds;
/// there's nothing to fake on real hardware.
#[cfg(not(target_arch = "arm"))]
pub type FakeGpio = peripherals::fake::gpio::FakeGpio;

/// See [`FakeGpio`] — the same test-facing counterpart, for
/// [`ClockProvider`].
#[cfg(not(target_arch = "arm"))]
pub type FakeClockProvider = peripherals::fake::clock::FakeClockProvider;

/// A test's own handles onto the fake peripherals backing a
/// [`BoardPeripherals`] returned by [`initialize`] on host/test builds —
/// the counterpart of [`BoardPeripherals::gpio`]/
/// [`BoardPeripherals::clock_provider`], which firmware gets instead.
/// Doesn't exist on real hardware, where there's nothing to fake.
#[cfg(not(target_arch = "arm"))]
#[derive(Clone)]
pub struct BoardFakePeripherals {
    pub gpio: FakeGpio,
    pub clock_provider: FakeClockProvider,
}

/// Everything this board's firmware gets from bringing up the chip: the
/// GPIO driver (already `configure()`d for every `_PIN` const above), and
/// ownership of every pin/peripheral this board's schematic uses that
/// doesn't have a dedicated `GpioPin` role.
///
/// [`Gpio`]'s underlying type, and the presence of [`Self::fakes`], are the
/// only things that differ between real hardware and host/test builds —
/// see [`initialize`]. The remaining fields keep the same upper-case names
/// they have on `resources::Peripherals` (and, on real hardware,
/// `embassy_stm32::Peripherals`) since they're moved out of it verbatim.
#[allow(non_snake_case)]
pub struct BoardPeripherals {
    pub gpio: Gpio,
    /// Owns the reference point every [`peripherals::api::clock::ClockTrait`]
    /// view this board hands out (via
    /// [`peripherals::api::clock::ClockProviderTrait::get_clock`]) reads
    /// from. Nothing refreshes it on a schedule of its own — call
    /// [`peripherals::api::clock::ClockProviderTrait::advance_reference_point`]
    /// periodically.
    pub clock_provider: ClockProvider,
    /// Time elapsed since boot, as of the last time it was refreshed (see
    /// [`peripherals::api::clock::ClockTrait::now`]). Starts at
    /// [`Uptime::epoch`] here; nothing in [`initialize`] refreshes it yet.
    pub uptime: Uptime,

    /// A test's own handles onto the same fake [`Self::gpio`]/
    /// [`Self::clock_provider`] — absent on real hardware. See
    /// [`BoardFakePeripherals`].
    #[cfg(not(target_arch = "arm"))]
    pub fakes: BoardFakePeripherals,

    /// The Cortex-M core peripherals (NVIC, SCB, MPU, ...) not already
    /// claimed by [`Self::clock_provider`] (which only borrows `DCB`/`DWT`
    /// from this, via
    /// [`peripherals::stm32g4::clock::ClockProvider::new`]) — passed
    /// through from [`initialize`]'s own [`resources::RticContext`]
    /// parameter, so it's `()` on the fake backend, which has nothing to
    /// simulate a core with.
    pub cortex_m_peripherals: resources::RticContext,

    /// SWDIO / JTMS — ST-LINK debug connection (J4 when the daughterboard
    /// is removed). Not managed as a `GpioPin`; firmware never drives it.
    pub PA13: resources::Peri<'static, resources::peripherals::PA13>,
    /// SWCLK / JTCK — see [`Self::PA13`].
    pub PA14: resources::Peri<'static, resources::peripherals::PA14>,
    /// Not connected on this board.
    pub PB10: resources::Peri<'static, resources::peripherals::PB10>,
    /// Not connected on this board.
    pub PB13: resources::Peri<'static, resources::peripherals::PB13>,
    /// Not connected on this board.
    pub PC15: resources::Peri<'static, resources::peripherals::PC15>,
    /// HSE crystal input, 8 MHz. Fixed function while the crystal
    /// (populated on this board) is in circuit.
    pub PF0: resources::Peri<'static, resources::peripherals::PF0>,
    /// HSE crystal output, 8 MHz. See [`Self::PF0`].
    pub PF1: resources::Peri<'static, resources::peripherals::PF1>,
    /// Dedicated NRST pin (shared pad with PG10).
    pub PG10: resources::Peri<'static, resources::peripherals::PG10>,

    /// Motor phase PWM generator driving [`TIM1_CH1_PIN`] and friends.
    /// Claimed but not yet driven by any code.
    pub TIM1: resources::Peri<'static, resources::peripherals::TIM1>,
    /// CAN controller behind [`CAN_RX_PIN`]/[`CAN_TX_PIN`].
    pub FDCAN1: resources::Peri<'static, resources::peripherals::FDCAN1>,
    /// Message RAM for [`Self::FDCAN1`].
    pub FDCANRAM1: resources::Peri<'static, resources::peripherals::FDCANRAM1>,
    /// UART behind [`USART2_TX_PIN`]/[`USART2_RX_PIN`].
    pub USART2: resources::Peri<'static, resources::peripherals::USART2>,
    /// Feeds [`VBUS_PIN`], the [`BACK_EMF1_PIN`]/[`BACK_EMF3_PIN`] taps,
    /// and others — see Table 12's ADCx_INy annotations in the STM32G431
    /// datasheet.
    pub ADC1: resources::Peri<'static, resources::peripherals::ADC1>,
    /// Feeds [`BACK_EMF2_PIN`] and others.
    pub ADC2: resources::Peri<'static, resources::peripherals::ADC2>,
    /// Phase U current-sense amplifier ([`CURRENT_FEEDBACK1_OPAMP_P_PIN`]/
    /// [`CURRENT_FEEDBACK1_OPAMP_N_PIN`]/[`OPAMP1_OUT_PIN`]).
    pub OPAMP1: resources::Peri<'static, resources::peripherals::OPAMP1>,
    /// Phase V current-sense amplifier.
    pub OPAMP2: resources::Peri<'static, resources::peripherals::OPAMP2>,
    /// Phase W current-sense amplifier.
    pub OPAMP3: resources::Peri<'static, resources::peripherals::OPAMP3>,
}

/// Brings up the chip — real hardware via `resources::init()`
/// (`embassy_stm32::init()` under the hood), or a simulated one on
/// host/test builds — and `configure()`s every `_PIN` const above on the
/// resulting [`peripherals::api::gpio::GpioTrait`] driver.
///
/// Takes a [`resources::RticContext`] (`cortex_m::Peripherals` on real
/// hardware, `()` on the fake backend — one argument either way) rather
/// than calling [`cortex_m::Peripherals::take`] itself, since, with RTIC
/// back in the picture, RTIC's own `init` prologue already steals that
/// singleton (via `cortex_m::Peripherals::steal()`) before calling into
/// application code — a second `::take()` here would panic.
#[allow(non_snake_case)]
pub fn initialize(mut cortex_m_peripherals: resources::RticContext) -> BoardPeripherals {
    let resources::Peripherals {
        // The 34 pins this board assigns a role to (see the `_PIN` consts
        // above) — claimed below via `claim_pins!`, not stored on
        // `BoardPeripherals`.
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
        PB11,
        PB12,
        PB14,
        PB15,
        PC4,
        PC6,
        PC10,
        PC11,
        PC13,
        PC14,
        // Everything else this board's schematic uses but doesn't have a
        // `GpioPin` role for — kept on `BoardPeripherals` instead.
        PA13,
        PA14,
        PB10,
        PB13,
        PC15,
        PF0,
        PF1,
        PG10,
        TIM1,
        FDCAN1,
        FDCANRAM1,
        USART2,
        ADC1,
        ADC2,
        OPAMP1,
        OPAMP2,
        OPAMP3,
        ..
    } = resources::init(resources::ClockConfiguration {
        mcu_frequency: MCU_FREQUENCY,
        oscillator_frequency: OSCILLATOR_FREQUENCY,
        timepiece_is_crystal: OSCILLATOR_IS_CRYSTAL,
    });
    // Every other field of the `Peripherals` above — every peripheral this
    // board doesn't use at all — is dropped right here.

    let pins = [
        TIM1_CH1_PIN,
        TIM1_CH1N_PIN,
        TIM1_CH2_PIN,
        TIM1_CH2N_PIN,
        TIM1_CH3_PIN,
        TIM1_CH3N_PIN,
        CURRENT_FEEDBACK1_OPAMP_P_PIN,
        CURRENT_FEEDBACK1_OPAMP_N_PIN,
        OPAMP1_OUT_PIN,
        CURRENT_FEEDBACK2_OPAMP_P_PIN,
        CURRENT_FEEDBACK2_OPAMP_N_PIN,
        OPAMP2_OUT_PIN,
        CURRENT_FEEDBACK3_OPAMP_P_PIN,
        CURRENT_FEEDBACK3_OPAMP_N_PIN,
        BACK_EMF1_PIN,
        BACK_EMF2_PIN,
        BACK_EMF3_PIN,
        GPIO_BACK_EMF_PIN,
        HALL_A_PIN,
        HALL_B_PIN,
        HALL_Z_PIN,
        CAN_RX_PIN,
        CAN_TX_PIN,
        CAN_TERM_PIN,
        CAN_SHUTDOWN_PIN,
        USART2_TX_PIN,
        USART2_RX_PIN,
        PWM_PIN,
        VBUS_PIN,
        POTENTIOMETER_PIN,
        TEMP_FEEDBACK_PIN,
        STATUS_PIN,
        BUTTON_PIN,
        TP3_PIN,
    ];

    let (mut gpio, _fake_gpio) = resources::split_off_fake(Gpio::new());

    // Claims the 34 pins: on real hardware this is just a Rust move (the
    // `Peri` bound above is dropped, so it can't also be claimed through
    // `embassy_stm32`'s own pin API elsewhere); on the fake backend it
    // additionally registers each pin with the gpio peripheral driver so that
    // pins that are used but not registered here trigger warnings.
    peripherals::claim_pins!(
        gpio, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA15, PB0, PB1,
        PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB11, PB12, PB14, PB15, PC4, PC6, PC10, PC11, PC13,
        PC14
    );

    for pin in pins {
        gpio.configure(pin);
    }

    let (clock_provider, _fake_clock_provider) =
        resources::split_off_fake(ClockProvider::new(&mut cortex_m_peripherals, MCU_FREQUENCY));

    #[cfg(not(target_arch = "arm"))]
    let fakes = BoardFakePeripherals {
        gpio: _fake_gpio,
        clock_provider: _fake_clock_provider,
    };

    BoardPeripherals {
        gpio,
        clock_provider,
        uptime: Uptime::epoch(),
        #[cfg(not(target_arch = "arm"))]
        fakes,
        cortex_m_peripherals,
        PA13,
        PA14,
        PB10,
        PB13,
        PC15,
        PF0,
        PF1,
        PG10,
        TIM1,
        FDCAN1,
        FDCANRAM1,
        USART2,
        ADC1,
        ADC2,
        OPAMP1,
        OPAMP2,
        OPAMP3,
    }
}
