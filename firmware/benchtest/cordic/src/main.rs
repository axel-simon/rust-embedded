#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

use common::duration::Duration;
use common::unit_interval::SymmetricUnitInterval;
use esc1_discovery::{BoardPeripherals, ClockProvider, MathCoprocessor};
use peripherals::api::clock::{ClockProviderTrait, ClockTrait};
use peripherals::api::math_coprocessor::{MathCoprocessorFunction, MathCoprocessorTrait};

/// How often to compute one sine/cosine pair and print its difference
/// from the reference implementation.
const SAMPLE_PERIOD: Duration = Duration::from_millis(200);

/// How much [`Firmware::angle`] advances each [`Firmware::step`]. Twenty
/// steps of this size very nearly span [`SymmetricUnitInterval`]'s whole
/// `[-1, 1)` cyclic range (`20 * 0.1001 == 2.002`, just over the range's
/// width of `2`) — i.e. one "lap" sweeps essentially all of `-pi ..= pi`
/// in twenty samples, then ends up `0.002` units (~0.36 degrees) further
/// along than where it started, so the next lap samples slightly
/// different angles instead of repeating the same twenty forever.
const ANGLE_STEP: f32 = 0.1001;

/// Number of digits [`format_fixed`] prints after the decimal point.
const FIXED_DIGITS: u32 = 6;
/// `10^FIXED_DIGITS`, i.e. how many representable units make up `1.0` in
/// [`format_fixed`]'s fixed-point encoding.
const FIXED_SCALE: i32 = 1_000_000;

/// One angle's worth of CORDIC-vs-reference disagreement.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Difference {
    sin: f32,
    cos: f32,
}

/// Renders `value` in fixed-point notation with exactly [`FIXED_DIGITS`]
/// digits after the decimal point (e.g. `-0.001234`), writing into `buf`
/// and returning the written prefix as a `&str`. `defmt` has no format
/// specifier for a fixed number of float decimals — only integer
/// zero-padding/radix hints — so this formats by hand rather than via a
/// `defmt::info!` format string; `core::fmt::Write`'s own width/zero-pad
/// specifiers (used below) are unrelated to defmt's and work here since
/// this is plain `core::fmt`, not a `defmt` macro.
fn format_fixed(value: f32, buf: &mut [u8; 16]) -> &str {
    // `f32::round()` needs `std`; away-from-zero rounding via a half-unit
    // offset before truncating gets the same result without it.
    let scaled = value * FIXED_SCALE as f32;
    let rounded = if scaled >= 0.0 {
        scaled + 0.5
    } else {
        scaled - 0.5
    };
    let scaled = rounded as i32;

    let magnitude = scaled.unsigned_abs();
    let integer_part = magnitude / FIXED_SCALE as u32;
    let fractional_part = magnitude % FIXED_SCALE as u32;
    let sign = if scaled < 0 { "-" } else { "" };

    let mut cursor = Cursor { buf, len: 0 };
    // `write!` never fails here — `buf` is always sized generously enough
    // for a `+/-D.DDDDDD`-shaped number (see `format_fixed`'s callers).
    core::fmt::write(
        &mut cursor,
        format_args!(
            "{sign}{integer_part}.{fractional_part:0width$}",
            width = FIXED_DIGITS as usize
        ),
    )
    .expect("buf is sized for a full fixed-point number");
    core::str::from_utf8(&cursor.buf[..cursor.len]).expect("only ASCII digits/'.'/'-' were written")
}

/// A `core::fmt::Write` sink over a fixed-size, caller-owned buffer — lets
/// [`format_fixed`] build its output with ordinary `format_args!` without
/// needing an allocator.
struct Cursor<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl core::fmt::Write for Cursor<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let end = self.len + bytes.len();
        if end > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }
}

/// The reference implementation [`Firmware::step`] compares the CORDIC
/// driver's result against: a pure-Rust port of MUSL's libc math library
/// (see Cargo.toml), so it works the same in `no_std` firmware as it
/// would linked against a real target libc.
fn reference_sin_cos(radians: f32) -> (f32, f32) {
    (libm::sinf(radians), libm::cosf(radians))
}

/// Everything the firmware does after chip bring-up, one angle at a time.
/// Kept separate from the `app` module below so it can be driven by unit
/// tests (see the `tests` module) without a real interrupt-driven
/// scheduler.
struct Firmware {
    math_coprocessor: MathCoprocessor,
    clock_provider: ClockProvider,
    /// The angle tested by the next [`Self::step`] call — see
    /// [`ANGLE_STEP`].
    angle: SymmetricUnitInterval,
}

impl Firmware {
    fn new(peripherals: BoardPeripherals) -> Self {
        let BoardPeripherals {
            math_coprocessor,
            clock_provider,
            ..
        } = peripherals;
        Firmware {
            math_coprocessor,
            clock_provider,
            // The minimum representable value, i.e. exactly `-1` — see
            // `SymmetricUnitInterval`'s doc table ("-1" means "-pi
            // radians"), so the very first lap starts at one end of the
            // `-pi ..= pi` sweep [`ANGLE_STEP`]'s doc comment describes.
            angle: SymmetricUnitInterval::new(i32::MIN),
        }
    }

    /// Computes one CORDIC sine/cosine pair, compares it against
    /// [`reference_sin_cos`], prints the difference, then advances
    /// [`Self::angle`] by [`ANGLE_STEP`] and paces itself by
    /// [`SAMPLE_PERIOD`] before returning.
    fn step(&mut self) -> Difference {
        // Keeps the clock's reference point fresh enough for `wait_for`'s
        // `now()` reads to stay accurate — see
        // `ClockProviderTrait::advance_reference_point`'s doc comment.
        self.clock_provider.advance_reference_point();
        let clock = self.clock_provider.get_clock();

        self.math_coprocessor
            .compute(MathCoprocessorFunction::SineCosine(self.angle));
        let (cordic_sin, cordic_cos) = self.math_coprocessor.result();

        let radians = f32::from(self.angle) * core::f32::consts::PI;
        let (reference_sin, reference_cos) = reference_sin_cos(radians);

        let difference = Difference {
            sin: f32::from(cordic_sin) - reference_sin,
            cos: f32::from(cordic_cos) - reference_cos,
        };

        #[cfg(not(test))]
        {
            let mut sin_buf = [0u8; 16];
            let mut cos_buf = [0u8; 16];
            defmt::info!(
                "angle: {}  sin diff: {}  cos diff: {}",
                f32::from(self.angle),
                format_fixed(difference.sin, &mut sin_buf),
                format_fixed(difference.cos, &mut cos_buf),
            );
        }

        self.angle = self.angle + SymmetricUnitInterval::from(ANGLE_STEP);
        clock.wait_for(SAMPLE_PERIOD);
        difference
    }
}

/// `stm32-metapac` mirrors an svd2rust PAC's register blocks (used
/// directly, e.g. `stm32_metapac::GPIOA`, by `peripherals::stm32g4`) but
/// doesn't generate a `NVIC_PRIO_BITS` const the way a real svd2rust PAC
/// does — `#[rtic::app]`'s `device` argument needs one under exactly that
/// name (to compute its priority-masking limits) even with zero
/// interrupt-bound tasks. The actual value is a board fact (it depends on
/// the MCU, not on this application), so it's defined as
/// [`esc1_discovery::RTIC_PRIORITY_BITS`] and just re-exported here under
/// the name RTIC's Cortex-M backend looks for.
#[cfg(target_arch = "arm")]
mod rtic_device {
    // `allow`: nothing here needs `Interrupt`/register-block re-exports
    // until a real interrupt-bound task exists; kept for when one does.
    pub use esc1_discovery::RTIC_PRIORITY_BITS as NVIC_PRIO_BITS;
    #[allow(unused_imports)]
    pub use stm32_metapac::*;
}

/// The application's RTIC task graph — real RTIC on real hardware, or (via
/// [`rtic_shim::app`]) a host-only fake that hands `init`'s `Local`
/// straight to unit tests, so they can drive [`Firmware::step`] directly
/// without a real interrupt-driven scheduler. `rtic_shim::app` always
/// forces `peripherals = false` (see `rtic_real`): chip bring-up in this
/// workspace always goes through a board crate's own `initialize()`
/// (`embassy_stm32::init()` under the hood), not RTIC's own PAC-peripherals
/// claim — `stm32_metapac` doesn't even expose the `Peripherals` type that
/// would need.
#[rtic_shim::app(device = crate::rtic_device)]
mod app {
    use super::Firmware;

    #[shared]
    pub(crate) struct Shared {}

    // `pub(crate)`: `#[cfg(test)] mod tests` (a sibling of this module, not
    // a descendant) needs to reach these fields directly.
    #[local]
    pub(crate) struct Local {
        pub(crate) firmware: Firmware,
        #[cfg(not(target_arch = "arm"))]
        pub(crate) fakes: esc1_discovery::BoardFakePeripherals,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        #[cfg(not(test))]
        defmt::info!("init");

        // RTIC's own `init` prologue already steals `cortex_m::Peripherals`
        // (that's `cx.core`) before calling this, so it's threaded through
        // rather than taken again — see `initialize`'s doc comment.
        #[cfg(target_arch = "arm")]
        let peripherals = esc1_discovery::initialize(_cx.core);
        #[cfg(not(target_arch = "arm"))]
        let peripherals = esc1_discovery::initialize(());

        #[cfg(not(target_arch = "arm"))]
        let fakes = peripherals.fakes.clone(); // cheap: Rc-backed

        let firmware = Firmware::new(peripherals);

        (
            Shared {},
            Local {
                firmware,
                #[cfg(not(target_arch = "arm"))]
                fakes,
            },
        )
    }

    // `mut`: only the fake (host) `idle::Context` needs it, since it owns
    // `Local` by value rather than borrowing it — see `rtic_fake`.
    #[allow(unused_mut)]
    #[idle(local = [firmware])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.local.firmware.step();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn step_advances_angle_by_angle_step_each_call() {
        let (_shared, local) = app::init(app::init::Context);
        let mut firmware = local.firmware;

        let start = firmware.angle;
        firmware.step();
        assert_eq!(
            firmware.angle,
            start + SymmetricUnitInterval::from(ANGLE_STEP)
        );
    }

    #[test]
    fn step_result_matches_the_reference_implementation_closely() {
        let (_shared, local) = app::init(app::init::Context);
        let mut firmware = local.firmware;

        // The fake `MathCoprocessor` computes via `std`'s `f32::sin`/
        // `f32::cos` (see `peripherals::fake::math_coprocessor`), so its
        // disagreement with `libm`'s independent implementation should be
        // tiny — this is really a check that the wiring (angle -> radians
        // -> comparison) is correct, not a hardware accuracy test (that
        // only happens for real on the STM32G4 target).
        for _ in 0..25 {
            let difference = firmware.step();
            assert!(difference.sin.abs() < 1e-3, "{difference:?}");
            assert!(difference.cos.abs() < 1e-3, "{difference:?}");
        }
    }

    #[test]
    fn step_paces_itself_by_roughly_sample_period() {
        let (_shared, local) = app::init(app::init::Context);
        let fakes = local.fakes.clone(); // cheap: Rc-backed
        let mut firmware = local.firmware;

        firmware.step();
        let after_first_sample = fakes.clock_provider.now();

        firmware.step();
        let after_second_sample = fakes.clock_provider.now();

        let period = after_second_sample - after_first_sample;
        assert!(
            period >= Duration::from_millis(180) && period <= Duration::from_millis(220),
            "expected ~{SAMPLE_PERIOD:?} between samples, got {period:?}"
        );
    }

    #[test]
    fn format_fixed_pads_to_a_fixed_number_of_decimal_digits() {
        let mut buf = [0u8; 16];
        assert_eq!(format_fixed(0.5, &mut buf), "0.500000");
        assert_eq!(format_fixed(-0.001, &mut buf), "-0.001000");
        assert_eq!(format_fixed(1.0, &mut buf), "1.000000");
        assert_eq!(format_fixed(0.0, &mut buf), "0.000000");
    }
}
