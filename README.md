# rust-embedded

Firmware for the STM32G431 (B-G431B-ESC1 Discovery kit). Each binary lives
under [`firmware/benchtest/`](firmware/benchtest), one directory per
program:

- [`blinky`](firmware/benchtest/blinky) — blinks the board's status LED on
  **PC6** at 1Hz. The default binary (see `default-members` in
  [Cargo.toml](Cargo.toml)) — `cargo build`/`cargo run` without `-p`
  target this one. There's no default `--target` (see
  [.cargo/config.toml](.cargo/config.toml)), so building/flashing it for
  real hardware still needs `--target thumbv7em-none-eabihf` explicitly —
  see "Build"/"Flash & run" below.
- [`adc`](firmware/benchtest/adc) — samples ADC1 channel 1 (the
  potentiometer, **PB12**) every 200ms and logs the raw reading via
  `defmt`.
- [`quadrature`](firmware/benchtest/quadrature) — decodes a 128-step
  quadrature encoder wired to **PB6**/**PB7** (TIM4_CH1/CH2, overriding
  this board's own Hall-sensor role for those pins) and logs its position
  every 100ms via `defmt`.

Each firmware's `main.rs` calls
[`esc1_discovery::initialize()`](boards/esc1_discovery/board.rs), which
brings up the chip via `embassy_stm32::init()`, configures every pin
declared in [`boards/esc1_discovery/board.rs`](boards/esc1_discovery/board.rs)
(the B-G431B-ESC1's full Table 4 pin map) through the register-level
`Gpio` driver in the [`peripherals`](peripherals) crate, and hands back a
`BoardPeripherals` bundle (GPIO, clock, ADC1, ...) for the firmware to
drive.

`peripherals` and `boards/esc1_discovery` are hardware-agnostic where it
matters: `peripherals` also ships fake `GpioTrait`/`ClockTrait`/`AdcTrait`
backends for host-side testing (see
[peripherals/README.md](peripherals/README.md)), and
`esc1_discovery::initialize()` uses those fake backends automatically when
built for a non-`arm` target — each firmware's own `#[cfg(test)] mod
tests` drives its `Firmware` struct against them with `cargo test -p
<name>` (no `--target` needed: with no default `--target` configured,
cargo already builds for your host by itself).

Targets the **STM32G431CB** variant (e.g. B-G431B-ESC1) by default. For a
different G431 package/flash size, change the `stm32g431cb` feature in
[boards/esc1_discovery/Cargo.toml](boards/esc1_discovery/Cargo.toml) (see
`embassy-stm32`'s `Cargo.toml` for the full list of chip features) and in
[peripherals/Cargo.toml](peripherals/Cargo.toml), update each firmware's
own `memory.x` (e.g.
[firmware/benchtest/blinky/memory.x](firmware/benchtest/blinky/memory.x))
with the matching flash/RAM sizes, and update the `--chip` value in
[.cargo/config.toml](.cargo/config.toml) to match.

## Build

No default `--target` is configured (see [.cargo/config.toml](.cargo/config.toml)),
so it must be given explicitly to build firmware for real hardware —
otherwise cargo builds for your host instead, which is what you want for
`cargo test`/`cargo clippy`/editor tooling, but not for flashing.

```sh
cargo build --release --target thumbv7em-none-eabihf          # blinky only (the default member)
cargo build --release --target thumbv7em-none-eabihf --workspace   # every crate, including all firmware binaries
cargo build --release --target thumbv7em-none-eabihf -p adc   # a specific firmware binary
```

## Flash & run (requires a probe, e.g. ST-Link, and `probe-rs` installed)

```sh
cargo run --release --target thumbv7em-none-eabihf       # blinky
cargo run --release --target thumbv7em-none-eabihf -p adc
```
