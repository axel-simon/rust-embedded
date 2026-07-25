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
