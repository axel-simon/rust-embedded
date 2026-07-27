# peripherals

Hardware-agnostic peripheral abstractions, with a fake backend for
host-side testing and a real STM32G4 GPIO driver. Has no dependency on
any external HAL crate; the one real-hardware dependency is
`stm32-metapac` (target-gated, see below).

## Layout

- `src/api/gpio.rs` — device-agnostic GPIO types: `GpioPin` (bit-packed
  into a `u32`), `GpioTrait`, and `PinToken` (implemented by any type that
  knows its own physical GPIO identity).
- `src/stm32g4/gpio.rs` — STM32G4 GPIO pin type tokens (`PA0`..`PJ15`),
  each a zero-sized type implementing `PinToken`; and, gated to
  `cfg(target_arch = "arm")`, `Gpio`, a `GpioTrait` driver backed by
  `stm32-metapac` register access for the STM32G431CB. `set()`/`get()` use
  the BSRR/IDR registers directly (a single volatile write/read per call —
  atomic, no read-modify-write race with other pins or other code touching
  the same port); `configure()` does a normal (non-atomic) read-modify-write
  of MODER/OTYPER/OSPEEDR/PUPDR, which is fine since pin configuration isn't
  done concurrently with itself. `Gpio::new()` takes no arguments and
  `claim_pin()` is a no-op — the real driver stays a zero-sized type and
  does no bookkeeping of its own; ownership enforcement for `Peri`-backed
  pins is just the Rust-level move (see `boards/resources`).
- `src/fake/gpio.rs` — `Gpio::new()` returns a pair of handles onto one
  shared, simulated chip: `Gpio`, a `GpioTrait` implementation for
  firmware (its `set()` mirrors the real driver, only writing
  `Output`/`InvertedOutput` pins — every other mode warns and no-ops), and
  `FakeGpio`, a test's handle for observing pin state or forcing it
  directly regardless of mode (simulating external hardware, e.g. a
  button press on an input pin). `FakeGpio` is cheap to `Clone` (all
  clones share the same underlying state via `Rc`). Unlike the real
  driver, `claim_pin()` here does real bookkeeping (via `PinToken`): only
  claimed pins are considered "wired up", and `configure()` warns if
  called on one that isn't.
- `src/fake/clock.rs` — likewise, `ClockProvider::new()` returns
  `(ClockProvider, FakeClockProvider)`: `ClockProvider` for firmware (same
  API as the real driver), and `FakeClockProvider` for a test to read the
  current simulated time (`now()`, without perturbing anything) or jump
  it forward directly (`advance_by()`).
- `src/api/adc.rs` — device-agnostic ADC types: `AdcSampleBuffer`, an
  opaque, fixed-capacity (`MAX_ADC_SEQUENCE_LENGTH` `u16`s) destination
  buffer a caller allocates as a `static` (e.g. `static ADC1_SAMPLES:
  AdcSampleBuffer = AdcSampleBuffer::new();`) and hands a `'static`
  reference to; `AdcOptions` (a channel sequence of up to
  `MAX_ADC_SEQUENCE_LENGTH` channels, each identified by a `u8`, plus that
  `'static` buffer reference, both given to `AdcOptions::new`); and
  `AdcTrait`. Unlike `GpioTrait`, an ADC must be explicitly `open()`ed
  (which calibrates it) before use and `close()`d afterwards; `trigger()`
  starts converting the configured sequence, `conversion_done()` reports
  completion (returning `true` exactly once per `trigger()` call), and
  `get_sample()` reads back one channel's result, as a
  `common::unit_interval::UnitInterval` fraction of the ADC's full-scale
  reading, by its position in the sequence. `open()` also takes a `&impl
  DmaTrait` (a separate argument from `AdcOptions`, since it isn't itself
  ADC configuration) — a driver
  that uses DMA looks up its assigned channel through it (see
  `src/stm32g4/adc.rs`); one that doesn't (like the fake) just ignores it.
  The buffer is caller-owned and `'static` rather than something a driver
  allocates internally: a DMA-capable driver's `open()` only ever needs to
  program its address into hardware once, since that address stays valid
  regardless of the driver struct itself moving afterwards (e.g. into a
  `Firmware` struct, which every firmware here does) — and, looking
  ahead, a hardware-triggered sequence (bypassing `AdcTrait::trigger`
  entirely, e.g. off a timer) would never call back into the driver to
  refresh it.
- `src/stm32g4/adc.rs` — gated to `cfg(target_arch = "arm")`, `Adc`, an
  `AdcTrait` driver for one of the STM32G431's ADC instances (`ADC1`/
  `ADC2`), backed by `stm32-metapac` register access. `open()` runs the
  full calibration sequence (voltage regulator startup, `ADCAL`, `ADEN`/
  `ADRDY`), writes the whole regular sequence across `SQR1`-`SQR4` (one
  rank per channel, up to `MAX_ADC_SEQUENCE_LENGTH`), then finds its DMA
  channel via `DmaTrait::lookup_channel` (using the `DmaRequest` —
  `Adc1`'s or `Adc2`'s own request line — `Adc::new` was given) and
  configures it directly (`CCR`/`CNDTR`/`CPAR`/`CMAR`, with `MINC` on so
  each rank's result lands in the next slot, and `CMAR` pointed at
  `options`' own `AdcSampleBuffer`, written once here and never again),
  enabling `CFGR.DMAEN` so the ADC actually drives it. Every `trigger()`
  re-arms the channel for a fresh `sequence_length`-item transfer (`CNDTR`
  reload plus an `EN` disable/re-enable; `CMAR` isn't touched again);
  `conversion_done()` polls the channel's own transfer-complete flag (set
  once every rank has landed) rather than the ADC's; `get_sample(rank)`
  reads the DMA-filled buffer's raw 12-bit count (matching `CFGR.RES`'s
  reset value, which `open()` never changes — see `ADC_RESOLUTION_BITS`)
  and scales it up into a `UnitInterval`'s full 32-bit fractional range.
- `src/fake/adc.rs` — likewise, `Adc::new()` returns `(Adc, FakeAdc)`:
  `Adc` for firmware, and `FakeAdc` for a test to inject the sample
  values a triggered conversion should "measure" (`set_sample()`) or
  observe what firmware configured via `open()` (`is_open()`/
  `options()`). The fake never touches DMA at all — `get_sample()`/
  `set_sample()` read/write straight into the currently-open `AdcOptions`'
  own `AdcSampleBuffer` (the same one a real driver's DMA channel would
  target), indexed by sequence position rather than by channel number, so
  there's no separate buffer for the fake to keep in sync; lacking any
  real hardware resolution to match, it treats the buffer's raw `u16`s as
  spanning the full 16 bits (`FAKE_SAMPLE_BITS`) rather than
  `stm32g4::adc`'s 12.
- `src/api/dma.rs` — device-agnostic DMA request-routing types:
  `DmaInstance` (which DMA controller — `Stm32g4Dma1`/`Stm32g4Dma2` so
  far; grows the same additive-only way `DmaRequest` does as new chip
  families are added), `DmaChannel` (an `{instance, channel}` pair — what
  channel numbers mean, e.g. whether they start at 0 or 1, is up to
  whichever driver you ask), `DmaRequest` (every DMA request line this
  workspace knows about — variants are additive-only and named
  `<Family>DmamuxReq<Signal>`, e.g. the STM32G4's come from RM0440 Table
  91 and are named `Stm32g4DmamuxReqAdc1` etc., with discriminants equal
  to the hardware's own request IDs; `None` is the default), and
  `DmaTrait`. Unlike `GpioTrait`/`AdcTrait`, this driver doesn't move
  data or drive a peripheral itself — `allocate()`/`lookup_channel()` only
  track which request is routed to which channel (`allocate()`, not
  `claim()`: it doesn't take ownership of anything, it just picks which
  request a channel carries — see `Gpio::claim_pin` vs `configure()` for
  the same split applied to pins). `lookup_channel()` is the linear scan
  of every valid channel for one carrying a given `DmaRequest` — the only
  read access `DmaTrait` exposes; nothing outside a `DmaTrait`
  implementation itself needs the inverse (a single channel's currently
  routed request), so that's not part of the trait.
- `src/stm32g4/dma.rs` — gated to `cfg(target_arch = "arm")`, `Dma`, a
  `DmaTrait` driver backed by `stm32-metapac` register access to the
  STM32G431's DMAMUX1. `allocate()`/`lookup_channel()` write/read
  `DMAMUX_CxCR.DMAREQ_ID` directly (the enum discriminant *is* the
  register value); channel numbers are 1-based, matching the hardware's
  own `DMA1_CHn`/`DMA2_CHn` naming. `CHANNELS_PER_INSTANCE` assumes 8
  channels per DMA instance on every STM32G4 device (DMA1→DMAMUX index
  0..8, DMA2→8..16) — RM0440 claims category 2 devices (which includes
  the STM32G431 this driver targets) only have 6 channels per instance
  and no DMAMUX gap, but that's not what real hardware does: see the
  module doc comment and `dmamux_channel_index`'s doc comment for the
  register-level evidence (verified on a STM32G431CB/B-G431B-ESC1) that
  DMA2 channel 1 actually sits on DMAMUX index 8, matching category 3
  devices' layout, not the index 6 RM0440's own category 2 table (top of
  page 420) claims. `new()` enables DMAMUX1's *and* DMA1's/DMA2's clocks
  unconditionally —
  like `Gpio::new()` enabling every GPIO port's clock regardless of which
  pins a board uses — so `claim_channel<T>(&mut self,
  _peri: T)` (which takes ownership of an embassy `Peri`, e.g. `DMA1_CH1`,
  so it can't be double-claimed) is a pure no-op otherwise, exactly
  mirroring `Gpio::claim_pin` on the real backend: unlike the fake, it
  does no bookkeeping and doesn't check that a channel was claimed before
  `allocate()` configures it. The `pub(super)` free function
  `dma_registers()` hands back the real `DMA1`/`DMA2`/channel register
  blocks (plus the 0-based index `ISR`/`IFCR` need) for a sibling driver
  within `stm32g4` (e.g. `stm32g4::adc`) to configure a transfer on once
  it's found its channel via `lookup_channel()` — scoped to `stm32g4`
  rather than `pub`, since nothing outside it needs real transfer-register
  access.
- `src/fake/dma.rs` — likewise, `Dma::new()` returns `(Dma, FakeDma)`.
  `DmaChannelToken` (mirrors `fake::gpio::PinToken`) is implemented for
  each DMA channel marker type in `boards/resources/fake_embassy.rs`, so
  `claim_channel<T: DmaChannelToken>(&mut self, _peri: T)` can recover a
  claimed channel's identity and register it as "wired up" — same idea as
  `Gpio::claim_pin`'s `registered` bitmap, just a `HashSet` here since the
  fake has no fixed channel count. `allocate()` warns (host-only, like
  every other fake's warnings — see `src/fake/gpio.rs`) if called on a
  channel that was never claimed, the same way `configure()`'s fake warns
  about an unclaimed pin; unlike `configure()`, it panics outright if the
  channel was already allocated — a channel should only ever be allocated
  once, and the fake catches that bug immediately rather than let two
  roles quietly fight over the same channel.

## Running tests on the host

This crate is `no_std` only for the real (`target_arch = "arm"`) target —
`fake` needs `std::rc::Rc` unconditionally on every other target, `cfg(test)`
or not (see `src/lib.rs`'s doc comment for why it isn't `cfg(test)`-gated).
The workspace's `.cargo/config.toml` pins the default `cargo` target to
`thumbv7em-none-eabi`, which has no `std` — host-side tests need an
explicit `--target` override for your machine:

```sh
cargo test -p peripherals --target <your-host-triple>
# e.g. --target aarch64-apple-darwin on Apple Silicon macOS
```

`stm32-metapac` (and the `stm32g4::gpio::Gpio` driver that depends on it)
is only a dependency for `cfg(target_arch = "arm")`, so it's never built
by the host-side test command above — the real driver isn't exercised by
this crate's test suite, only the fake is.
