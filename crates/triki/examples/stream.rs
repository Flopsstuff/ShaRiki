//! Connect to a Triki token, stream the IMU at 104 Hz, and print a frame every ~0.25 s
//! together with the measured frame rate. Ctrl-C disconnects cleanly.
//!
//! ```sh
//! cargo run -p triki-ble --example stream [device-id]
//! ```
//!
//! Without an id, the first token advertising `TRIKI…` is used; ids come from the `scan`
//! example.

use std::time::{Duration, Instant};

use futures::StreamExt;
use triki_ble::{ConnectOptions, Triki};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let opts = ConnectOptions {
        id: std::env::args().nth(1),
        ..ConnectOptions::default()
    };
    println!("Connecting...");
    let triki = Triki::connect(&opts).await?;
    println!("Connected to {} ({})", triki.name(), triki.id());
    match triki.battery().await {
        Ok(Some(pct)) => println!("Battery: {pct}%"),
        Ok(None) => println!("Battery: n/a"),
        Err(e) => println!("Battery read failed: {e}"),
    }

    let mut imu = triki.start_imu(104).await?;
    println!("Streaming at {} Hz; Ctrl-C to stop.", imu.rate_hz());

    let mut window_start = Instant::now();
    let mut window_frames = 0u32;
    let mut ticker = tokio::time::interval(Duration::from_millis(250));
    let mut latest = None;
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    loop {
        tokio::select! {
            _ = &mut ctrl_c => {
                println!("\nCtrl-C: disconnecting...");
                break;
            }
            frame = imu.next() => match frame {
                Some(f) => {
                    window_frames += 1;
                    latest = Some(f);
                }
                None => {
                    println!("Link dropped.");
                    break;
                }
            },
            _ = ticker.tick() => {
                let elapsed = window_start.elapsed().as_secs_f64();
                if let Some(f) = latest.take() {
                    let fps = f64::from(window_frames) / elapsed;
                    println!(
                        "#{:>6} t={:>8.3}s  gyro [{:>8.1} {:>8.1} {:>8.1}] dps  \
                         accel [{:>6.3} {:>6.3} {:>6.3}] g  {fps:>6.1} fps",
                        f.index, f.t,
                        f.gyro_dps[0], f.gyro_dps[1], f.gyro_dps[2],
                        f.accel_g[0], f.accel_g[1], f.accel_g[2],
                    );
                }
                window_start = Instant::now();
                window_frames = 0;
            }
        }
    }

    triki.disconnect().await?;
    println!("Disconnected.");
    Ok(())
}
