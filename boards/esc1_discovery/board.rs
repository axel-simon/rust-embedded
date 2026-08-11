//! Pin map for the B-G431B-ESC1 Discovery kit (electronic speed controller
//! for drones, MB1419, target STM32G431CBU6).
//!
//! Names and pin assignments come from Table 4 "Main board STM32G431CB
//! pinout for motor control" in UM2516 (the kit's user manual).
#![cfg_attr(not(test), no_std)]

use common::duration::Duration;
use peripherals::api::dma::{DmaChannel, DmaInstance, DmaRequest, DmaTrait};
use peripherals::api::gpio::{GpioPin, GpioPort, GpioTrait};
// Real hardware on `target_arch = "arm"`, a fake elsewhere (so host-side
// `cargo test` works without hardware) — every `backend::x::Y` path below
// resolves to `peripherals::stm32g4::x::Y` or `peripherals::fake::x::Y`
// depending on which of these is active, letting `initialize`/
// `BoardPeripherals` name a driver type once instead of pairing a
// `#[cfg(target_arch = "arm")]`/`#[cfg(not(...))]` type alias for each.
// `pub`: firmware crates can name a `BoardPeripherals`/`BoardFakePeripherals`
// field's own type (e.g. `esc1_discovery::backend::adc::Adc`) through this
// same alias too, instead of repeating the `#[cfg(target_arch = "arm")]`
// branch themselves.
#[cfg(not(target_arch = "arm"))]
pub use peripherals::fake as backend;
#[cfg(target_arch = "arm")]
pub use peripherals::stm32g4 as backend;

// Motor phase PWM (TIM1): CH1/CH2/CH3 drive the high side (AF6),
// CH1N/CH2N/CH3N the complementary low side (AF4) — one pair per phase U/V/W.
pub const TIM1_CH1_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 8, 6).with_very_high_speed();
pub const TIM1_CH1N_PIN: GpioPin = GpioPin::alternate(GpioPort::PC, 13, 4).with_very_high_speed();
pub const TIM1_CH2_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 9, 6).with_very_high_speed();
pub const TIM1_CH2N_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 12, 6).with_very_high_speed();
pub const TIM1_CH3_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 10, 6).with_very_high_speed();
pub const TIM1_CH3N_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 15, 4).with_very_high_speed();

// Per-phase current shunt feedback through OPAMP1/2/3 (analog in/out).
// The P/N pins (VINP/VINM0) are documentation only below `initialize`'s own
// opamp bring-up block claims and drives them directly via
// `embassy_stm32::opamp` (differential PGA, internal-only output) — they're
// deliberately *not* in `pins`/`claim_pins!` below, since that would
// double-claim them through this crate's own `Gpio` as well.
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

// Quadrature encoder or hall-effect sensor header (J8).
// These pins are currently used only as encoder, hence we name them that way.
pub const TIM4_QUADRATURE_A: GpioPin = GpioPin::alternate(GpioPort::PB, 6, 2);
pub const TIM4_QUADRATURE_B: GpioPin = GpioPin::alternate(GpioPort::PB, 7, 2);
pub const TIM4_QUADRATURE_Z: GpioPin = GpioPin::alternate(GpioPort::PB, 8, 2);

// CAN bus (FDCAN1_RX/TX, AF9) plus its termination switch and transceiver
// shutdown/TP2.
pub const CAN_RX_PIN: GpioPin = GpioPin::alternate(GpioPort::PA, 11, 9);
pub const CAN_TX_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 9, 9).with_high_speed();
pub const CAN_TERM_PIN: GpioPin = GpioPin::output(GpioPort::PC, 14);
pub const CAN_SHUTDOWN_PIN: GpioPin = GpioPin::inverted_output(GpioPort::PC, 11);

// UART2 (AF7): On J3.
pub const USART2_TX_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 3, 7);
pub const USART2_RX_PIN: GpioPin = GpioPin::alternate(GpioPort::PB, 4, 7);

// External PWM input for motor speed regulation (J3); inferred as TIM2_CH1
// (AF1) since UM2516 only names it generically as "PWM".
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
/// priority-masking limits.
pub const RTIC_PRIORITY_BITS: u8 = 4; // Valid for all ST Cortex-M4 MCUs.

/// TIM1's dead time — the switch-off/switch-on gap enforced between each
/// PWM channel's complementary pair (e.g. [`TIM1_CH1_PIN`]/
/// [`TIM1_CH1N_PIN`]). Measured by hand at 28V bus voltage with no load.
pub const TIM1_DEAD_TIME: Duration = Duration::from_nanos(230);

/// A test's own handles onto the fake peripherals backing a
/// [`BoardPeripherals`] returned by [`initialize`] on host/test builds —
/// the counterpart of [`BoardPeripherals::gpio`]/
/// [`BoardPeripherals::clock_provider`]/[`BoardPeripherals::adc1`]/
/// [`BoardPeripherals::dma`]/..., which firmware gets instead.
#[cfg(not(target_arch = "arm"))]
#[derive(Clone)]
pub struct BoardFakePeripherals {
    pub gpio: backend::gpio::FakeGpio,
    pub clock_provider: backend::clock::FakeClockProvider,
    pub adc1: backend::adc::FakeAdc,
    pub adc2: backend::adc::FakeAdc,
    pub dma: backend::dma::FakeDma,
    pub quadrature: backend::quadrature::FakeQuadrature,
    pub pwm1: backend::pwm::FakePwm,
    /// Shared with [`Self::adc1`]/[`Self::adc2`]/[`Self::pwm1`] — see
    /// `peripherals::fake::callback_registry`. A test can also use this
    /// directly to fire/schedule its own triggers, or advance simulated
    /// time (in place of calling [`Self::clock_provider`]'s own
    /// `advance_by` — this fires anything pending along the way).
    pub callback_registry: peripherals::fake::callback_registry::CallbackRegistry,
}

/// Drivers relevant to this board and Embassy peripheral resources that do not
/// have high-level drivers yet.
#[allow(non_snake_case)]
pub struct BoardPeripherals {
    pub gpio: backend::gpio::Gpio,
    pub clock_provider: backend::clock::ClockProvider,
    pub dma: backend::dma::Dma,
    pub adc1: backend::adc::Adc,
    pub adc2: backend::adc::Adc,
    pub quadrature: backend::quadrature::Quadrature,
    pub pwm1: backend::pwm::Pwm,
    pub math_coprocessor: backend::math_coprocessor::MathCoprocessor,

    /// A test's own handles onto the same fake [`Self::gpio`]/
    /// [`Self::clock_provider`]/[`Self::adc1`]/[`Self::adc2`]/
    /// [`Self::dma`]/[`Self::quadrature`]/[`Self::pwm1`] — absent on real
    /// hardware. See [`BoardFakePeripherals`].
    #[cfg(not(target_arch = "arm"))]
    pub fakes: BoardFakePeripherals,

    /// The Cortex-M core peripherals. Required by RTIC.
    pub mcu_interface: resources::McuInterface,

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

    /// CAN controller behind [`CAN_RX_PIN`]/[`CAN_TX_PIN`].
    pub FDCAN1: resources::Peri<'static, resources::peripherals::FDCAN1>,
    /// Message RAM for [`Self::FDCAN1`].
    pub FDCANRAM1: resources::Peri<'static, resources::peripherals::FDCANRAM1>,
    /// UART behind [`USART2_TX_PIN`]/[`USART2_RX_PIN`].
    pub USART2: resources::Peri<'static, resources::peripherals::USART2>,
}

/// Initializes the MCU and its peripherals by calling `resources::init()`
/// (which calls `embassy_stm32::init()` under the hood if compiled for real
/// hardware).
///
/// Takes a [`resources::McuInterface`] (`cortex_m::Peripherals` on real
/// hardware, `()` on the fake backend) and returns it as part of
/// BoardPeripherals.
#[allow(non_snake_case)]
pub fn initialize(mut mcu_interface: resources::McuInterface) -> BoardPeripherals {
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
        PA13,
        PA14,
        PB10,
        PB13,
        PC15,
        PF0,
        PF1,
        PG10,
        TIM1,
        TIM4,
        FDCAN1,
        FDCANRAM1,
        USART2,
        ADC1,
        ADC2,
        OPAMP1,
        OPAMP2,
        OPAMP3,
        DMA1_CH1,
        DMA2_CH1,
        ..
    } = resources::init(resources::ClockConfiguration {
        mcu_frequency: MCU_FREQUENCY,
        oscillator_frequency: OSCILLATOR_FREQUENCY,
        timepiece_is_crystal: OSCILLATOR_IS_CRYSTAL,
    });
    // Every other field of the `Peripherals` above — every peripheral this
    // board doesn't use at all — is dropped right here.

    //
    // Embassy code for configuration.
    // Board-specific configuration of peripherals that the firmware does not
    // interact with later. Unit testing this code has little value, hence we
    // avoid the complexity of defining abstract driver APIs with fake
    // implementations and use Embassy crates directly for expedieency.
    //

    // Configures the three per-phase current-sense op-amps as differential
    // PGAs (VINP = shunt high side, VINM0 = shunt low side/bias node) with
    // internal-only output, routed straight into ADC1/ADC2:
    //   * OPAMP1 -> ADC1 ch13
    //   * OPAMP2 -> ADC2 ch16
    //   * OPAMP3 -> ADC2 ch18
    //
    // `mem::forget` on each `OpAmpInternalOutput` is what keeps
    // `CSR.OPAMPEN` set permanently — its `Drop` impl would otherwise
    // disable the opamp the instant this function returns. `OpAmp<T>`
    // itself has no `Drop` on g4 (`impl Drop for OpAmp` is
    // `#[cfg(not(any(stm32g4, stm32f3)))]`), so letting `opamp1`/`2`/`3`
    // themselves drop here (at the end of this block) is harmless.
    #[cfg(target_arch = "arm")]
    {
        use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};

        let mut opamp1 = OpAmp::new(OPAMP1, OpAmpSpeed::Normal);
        core::mem::forget(opamp1.pga_biased_int(PA1, PA3, OpAmpGain::Mul64));

        let mut opamp2 = OpAmp::new(OPAMP2, OpAmpSpeed::Normal);
        core::mem::forget(opamp2.pga_biased_int(PA7, PA5, OpAmpGain::Mul64));

        let mut opamp3 = OpAmp::new(OPAMP3, OpAmpSpeed::Normal);
        core::mem::forget(opamp3.pga_biased_int(PB0, PB2, OpAmpGain::Mul64));
    }
    // Drop the opamp-related resources on host.
    #[cfg(not(target_arch = "arm"))]
    let _ = (PA1, PA3, PA5, PA7, PB0, PB2, OPAMP1, OPAMP2, OPAMP3);

    //
    // Abstract peripherals.
    //

    let pins = [
        TIM1_CH1_PIN,
        TIM1_CH1N_PIN,
        TIM1_CH2_PIN,
        TIM1_CH2N_PIN,
        TIM1_CH3_PIN,
        TIM1_CH3N_PIN,
        OPAMP1_OUT_PIN,
        OPAMP2_OUT_PIN,
        BACK_EMF1_PIN,
        BACK_EMF2_PIN,
        BACK_EMF3_PIN,
        GPIO_BACK_EMF_PIN,
        TIM4_QUADRATURE_A,
        TIM4_QUADRATURE_B,
        TIM4_QUADRATURE_Z,
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

    let (mut gpio, _fake_gpio) = resources::split_off_fake(backend::gpio::Gpio::new());

    // Claims these pins: on real hardware this is a no-op but ensures that the
    // pins cannot be claimed through `embassy_stm32`'s own pin API elsewhere;
    // on the fake backend it additionally registers each pin with the gpio
    // peripheral driver so that pins that are later used but not registered
    // here trigger warnings.
    peripherals::claim_pins!(
        gpio, PA0, PA2, PA4, PA6, PA8, PA9, PA10, PA11, PA12, PA15, PB1, PB3, PB4, PB5, PB6, PB7,
        PB8, PB9, PB11, PB12, PB14, PB15, PC4, PC6, PC10, PC11, PC13, PC14
    );

    for pin in pins {
        gpio.configure(pin);
    }

    let (clock_provider, _fake_clock_provider) = resources::split_off_fake(
        backend::clock::ClockProvider::new(&mut mcu_interface, MCU_FREQUENCY),
    );

    let (mut dma, _fake_dma) = resources::split_off_fake(backend::dma::Dma::new());

    // Configures DMA channels.
    dma.claim_channel(DMA1_CH1);
    dma.allocate(
        DmaChannel::new(DmaInstance::Stm32g4Dma1, 1),
        DmaRequest::Stm32g4DmamuxReqAdc1,
    );
    dma.claim_channel(DMA2_CH1);
    dma.allocate(
        DmaChannel::new(DmaInstance::Stm32g4Dma2, 1),
        DmaRequest::Stm32g4DmamuxReqAdc2,
    );

    // Claim the ADC1 and ADC2 resources and constructs adc::Adc drivers from it.
    let adc1_instance = resources::claim_adc(ADC1);
    let (adc1, _fake_adc1) = resources::split_off_fake(backend::adc::Adc::new(
        adc1_instance,
        DmaRequest::Stm32g4DmamuxReqAdc1,
    ));
    let adc2_instance = resources::claim_adc(ADC2);
    let (adc2, _fake_adc2) = resources::split_off_fake(backend::adc::Adc::new(
        adc2_instance,
        DmaRequest::Stm32g4DmamuxReqAdc2,
    ));

    // Claim TIM4 and construct a quadrature driver from it.
    let quadrature_timer = resources::claim_quadrature_timer(TIM4);
    let (quadrature, _fake_quadrature) =
        resources::split_off_fake(backend::quadrature::Quadrature::new(quadrature_timer));

    // Claim TIM1 and construct a pwm::Pwm driver from it. TIM1 sits on
    // APB2, which runs at MCU_FREQUENCY.
    let pwm_timer = resources::claim_pwm_timer(TIM1);
    let (pwm1, _fake_pwm1) =
        resources::split_off_fake(backend::pwm::Pwm::new(pwm_timer, MCU_FREQUENCY));

    // Math coprocessor. This peripheral has no fake counterpart.
    let math_coprocessor = backend::math_coprocessor::MathCoprocessor::new();

    // Construct a registry for interrupts and MCU-internal triggers.
    // Simulates hardware-triggered dynamic behavior between fakes (e.g. a
    // PWM's periodic trigger-out edge causing an ADC's interrupt to fire
    // once a simulated conversion time has elapsed).
    #[cfg(not(target_arch = "arm"))]
    let callback_registry = {
        let registry = peripherals::fake::callback_registry::CallbackRegistry::new(
            _fake_clock_provider.clone(),
        );
        adc1.set_callback_registry(registry.clone());
        adc2.set_callback_registry(registry.clone());
        pwm1.set_callback_registry(registry.clone());
        registry
    };

    #[cfg(not(target_arch = "arm"))]
    let fakes = BoardFakePeripherals {
        gpio: _fake_gpio,
        clock_provider: _fake_clock_provider,
        adc1: _fake_adc1,
        adc2: _fake_adc2,
        dma: _fake_dma,
        quadrature: _fake_quadrature,
        pwm1: _fake_pwm1,
        callback_registry,
    };

    BoardPeripherals {
        gpio,
        clock_provider,
        dma,
        adc1,
        adc2,
        quadrature,
        pwm1,
        math_coprocessor,
        #[cfg(not(target_arch = "arm"))]
        fakes,
        mcu_interface,
        PA13,
        PA14,
        PB10,
        PB13,
        PC15,
        PF0,
        PF1,
        PG10,
        FDCAN1,
        FDCANRAM1,
        USART2,
    }
}
