//! On macOS, embed an Info.plist into the binary. CoreBluetooth kills a process that
//! touches Bluetooth without `NSBluetoothAlwaysUsageDescription`, and a bare CLI binary
//! has no bundle to carry one — the `__TEXT,__info_plist` section is where it looks.

fn main() {
    let plist = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("macos/Info.plist");
    println!("cargo:rerun-if-changed={}", plist.display());
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        println!(
            "cargo:rustc-link-arg-bins=-Wl,-sectcreate,__TEXT,__info_plist,{}",
            plist.display()
        );
    }
}
