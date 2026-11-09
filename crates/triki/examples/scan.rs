//! List nearby Triki tokens.
//!
//! ```sh
//! cargo run -p triki-ble --example scan [seconds]
//! ```
//!
//! On macOS the terminal app needs Bluetooth permission (System Settings → Privacy &
//! Security → Bluetooth).

use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: f64 = match std::env::args().nth(1) {
        Some(s) => s.parse().map_err(|_| format!("invalid seconds: {s:?}"))?,
        None => 5.0,
    };
    println!("Scanning for {secs} s...");
    let found = triki_ble::scan(Duration::from_secs_f64(secs)).await?;
    if found.is_empty() {
        println!("No Triki tokens found. Is the token awake and not connected elsewhere?");
    }
    for d in found {
        let rssi = d
            .rssi
            .map_or_else(|| "?".to_owned(), |r| format!("{r} dBm"));
        println!("{:<16} {:<38} RSSI {rssi}", d.name, d.id);
    }
    Ok(())
}
