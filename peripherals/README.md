# peripherals

Hardware-agnostic peripheral abstractions — GPIO, clock, ADC, DMA,
quadrature encoder, PWM, and math coprocessor — each with a fake backend
for host-side testing (`src/fake/`) and a real STM32G4 backend
(`src/stm32g4/`, gated to `cfg(target_arch = "arm")`). The abstract API
each pair implements lives in `src/api/`. Has no dependency on any
external HAL crate; the one real-hardware dependency is `stm32-metapac`.

For everyday use, only `src/api/` matters — that's the abstract trait and
options each backend implements, and it's already documented via doc
comments (`cargo doc`). This file exists for the one thing that isn't part
of the trait and so isn't in that documentation: each backend's
constructor.

## Peripherals

### GPIO
Configures and drives individual pins. No `open`/`close` — a `GpioPin`
(port, number, mode, drive strength, pull) is configured once, then read
or written directly.
- API: `GpioPin`, `GpioTrait` (`configure()`, `set()`, `get()`)
- Fake: `fake::gpio::Gpio::new() -> (Gpio, FakeGpio)`
- Real: `stm32g4::gpio::Gpio::new() -> Gpio`

### Clock
Reports elapsed time and busy-waits, backed by a free-running hardware
tick counter.
- API: `ClockTrait` (`now()`, `wait_for()`, `ticks_now()`, `time_at()`,
  `ticks_per_second()`), `ClockProviderTrait` (`get_clock()`,
  `advance_reference_point()`)
- Fake: `fake::clock::ClockProvider::new<T>(unused: T, mcu_frequency: u32) -> (ClockProvider, FakeClockProvider)`
- Real: `stm32g4::clock::ClockProvider::new(peripherals: &mut cortex_m::Peripherals, mcu_frequency: u32) -> ClockProvider`

### ADC
Converts a configured sequence of channels into `UnitInterval` fractions
of full-scale, DMA-driven.
- API: `AdcOptions`, `AdcSampleBuffer`, `AdcTrait` (`open()`, `close()`,
  `trigger()`, `conversion_done()`, `get_sample()`)
- Fake: `fake::adc::Adc::new(instance: AdcInstance, dma_request: DmaRequest) -> (Adc, FakeAdc)`
- Real: `stm32g4::adc::Adc::new(instance: AdcInstance, dma_request: DmaRequest) -> Adc`

### DMA
Routes DMA requests to channels; doesn't move data or drive a peripheral
itself.
- API: `DmaChannel`, `DmaRequest`, `DmaTrait` (`allocate()`,
  `lookup_channel()`)
- Fake: `fake::dma::Dma::new() -> (Dma, FakeDma)`
- Real: `stm32g4::dma::Dma::new() -> Dma`

### Quadrature encoder
Decodes a quadrature encoder's position via a hardware timer, as a
fraction of one full cycle.
- API: `QuadratureOptions`, `QuadratureTimer`, `QuadratureTrait`
  (`open()`, `close()`, `position()`)
- Fake: `fake::quadrature::Quadrature::new(timer: QuadratureTimer) -> (Quadrature, FakeQuadrature)`
- Real: `stm32g4::quadrature::Quadrature::new(timer: QuadratureTimer) -> Quadrature`

### PWM
Generates center-aligned PWM with per-channel dead time on a
break-and-dead-time-capable timer.
- API: `PwmOptions`, `PwmTimer`, `PwmTrait` (`open()`, `close()`,
  `set_duty_cycle()`, `set_dead_time()`)
- Fake: `fake::pwm::Pwm::new(timer: PwmTimer, timer_clock_hz: u32) -> (Pwm, FakePwm)`
- Real: `stm32g4::pwm::Pwm::new(timer: PwmTimer, timer_clock_hz: u32) -> Pwm`

### Math coprocessor
Computes sine/cosine and vector phase — trigonometric functions the
Cortex-M4's own FPU has no hardware support for.
- API: `MathCoprocessorFunction`, `MathCoprocessorTrait` (`compute()`,
  `result()`)
- Fake: `fake::math_coprocessor::MathCoprocessor::new() -> MathCoprocessor`
  (no separate fake handle — this backend computes real results with
  `std` floating point, so there's nothing to simulate)
- Real: `stm32g4::math_coprocessor::MathCoprocessor::new() -> MathCoprocessor`

## Running tests on the host

This crate is `no_std` only for the real (`target_arch = "arm"`) target —
`fake` needs `std::rc::Rc` unconditionally on every other target, `cfg(test)`
or not (see `src/lib.rs`'s doc comment for why it isn't `cfg(test)`-gated).
The workspace's `.cargo/config.toml` sets no default `cargo` target, so
plain `cargo test` already builds for your host (which has `std`) without
any extra flags:

```sh
cargo test -p peripherals
```

Building for the embedded target instead needs an explicit
`--target thumbv7em-none-eabihf` (see the root [README.md](../README.md)).
`stm32-metapac` (and every `src/stm32g4/` driver, which depends on it) is
only a dependency for `cfg(target_arch = "arm")`, so it's never built by
the host-side test command above — the real drivers aren't exercised by
this crate's test suite, only the fakes are.
