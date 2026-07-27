use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Host-target test builds (see peripherals/README.md) link with the
    // host's own linker/libc, not cortex-m-rt's `link.x`/defmt's
    // `defmt.x` — those are only meaningful (and only resolvable) when
    // actually targeting the embedded chip.
    if env::var_os("CARGO_CFG_TARGET_ARCH").as_deref() != Some(std::ffi::OsStr::new("arm")) {
        return;
    }

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
