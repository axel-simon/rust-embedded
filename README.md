# rust-embedded

Firmware for the STM32G431 (B-G431B-ESC1 Discovery kit) that blinks the
board's status LED on **PC6**. `main.rs` calls
[`esc1_discovery::initialize()`](boards/esc1_discovery/init.rs), which
brings up the chip via `embassy_stm32::init()`, configures every pin
declared in [`boards/esc1_discovery/board.rs`](boards/esc1_discovery/board.rs)
(the B-G431B-ESC1's full Table 4 pin map) through the register-level
`Gpio` driver in the [`peripherals`](peripherals) crate, and hands back a
`Peripherals { gpio, delay }` bundle. `main.rs` then loops, toggling the
LED with `gpio.set(...)` and pacing itself with `delay.delay_ms(100)`
(`embassy_time::Delay`, blocking — no async executor involved).

`peripherals` and `boards/esc1_discovery` are hardware-agnostic where it
matters: `peripherals` also ships a fake `GpioTrait` backend for host-side
testing (see [peripherals/README.md](peripherals/README.md)), and
`esc1_discovery::initialize()` uses that fake backend automatically when
built for a non-`arm` target.

Targets the **STM32G431CB** variant (e.g. B-G431B-ESC1) by default. For a
different G431 package/flash size, change the `stm32g431cb` feature in
[boards/esc1_discovery/Cargo.toml](boards/esc1_discovery/Cargo.toml) (see
`embassy-stm32`'s `Cargo.toml` for the full list of chip features) and in
[peripherals/Cargo.toml](peripherals/Cargo.toml), update
[memory.x](memory.x) with the matching flash/RAM sizes, and update the
`--chip` value in [.cargo/config.toml](.cargo/config.toml) to match.

## Build

```sh
cargo build --release
```

## Flash & run (requires a probe, e.g. ST-Link, and `probe-rs` installed)

```sh
cargo run --release
```
