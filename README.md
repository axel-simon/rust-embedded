# rust-embedded

RTIC firmware for the STM32G431 that blinks an LED on **PC6**. TIM2 is
configured to count up to 100,000 ticks (1 tick = 1 microsecond, i.e. a
100 ms period) and fires an update interrupt each time it wraps; that
interrupt is bound directly to an RTIC hardware task which toggles the
LED. The LED GPIO pin and the timer are `#[local]` resources owned
exclusively by that task, so no other task or shared state can touch
them.

Targets the **STM32G431CB** variant (e.g. B-G431B-ESC1) by default. For a
different G431 package/flash size, change the `stm32g431` feature set in
[Cargo.toml](Cargo.toml) (see `stm32g4xx-hal`'s `Cargo.toml` for the full
list of chip features), update [memory.x](memory.x) with the matching
flash/RAM sizes, and update the `--chip` value in
[.cargo/config.toml](.cargo/config.toml) to match.

## Build

```sh
cargo build --release
```

## Flash & run (requires a probe, e.g. ST-Link, and `probe-rs` installed)

```sh
cargo run --release
```
