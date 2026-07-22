# peripherals

Hardware-agnostic peripheral abstractions, with a fake backend for
host-side testing and a real STM32G4 GPIO driver. Has no dependency on
any external HAL crate; the one real-hardware dependency is
`stm32-metapac` (target-gated, see below).

## Layout

- `src/api/gpio.rs` — device-agnostic GPIO types: `GpioPin` (bit-packed
  into a `u32`), `GpioTrait`, and `PinAndPort`.
- `src/fake/peri.rs` — `Peri`, a standalone peripheral-ownership token (in the
  spirit of `embassy_hal_internal::Peri`, but self-contained), and the
  `PeripheralType` marker trait it requires.
- `src/stm32g4/gpio.rs` — STM32G4 GPIO pin type tokens (`PA0`..`PJ15`),
  each a zero-sized type implementing `PinToken` (to build a `PinAndPort`
  via `pin_and_port::<PC6>()`) and `PeripheralType` (for use as a
  `Peri<'_, PC6>`); and, gated to `cfg(target_arch = "arm")`, `Gpio`, a
  `GpioTrait` driver backed by `stm32-metapac` register access for the
  STM32G431CB. `set()`/`get()` use the BSRR/IDR registers directly (a
  single volatile write/read per call — atomic, no read-modify-write race
  with other pins or other code touching the same port); `configure()`
  does a normal (non-atomic) read-modify-write of MODER/OTYPER/OSPEEDR/PUPDR,
  which is fine since pin configuration isn't done concurrently with itself.
- `src/fake/gpio.rs` — `GpioFake`, a `GpioTrait` implementation that
  simulates pin state in memory, for unit-testing code that depends on
  `GpioTrait` without any hardware.

## Running tests on the host

This crate is `no_std` outside of `cfg(test)` so it can be built for the
firmware's embedded target. The workspace's `.cargo/config.toml` pins the
default `cargo` target to `thumbv7em-none-eabi`, which has no `std` —
host-side tests need an explicit `--target` override for your machine:

```sh
cargo test -p peripherals --target <your-host-triple>
# e.g. --target aarch64-apple-darwin on Apple Silicon macOS
```

`stm32-metapac` (and the `stm32g4::gpio::Gpio` driver that depends on it)
is only a dependency for `cfg(target_arch = "arm")`, so it's never built
by the host-side test command above — the real driver isn't exercised by
this crate's test suite, only the fake is.
