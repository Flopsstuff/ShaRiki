//! One-line terminal meter (preset, shake energy, hits, IMU rate, output level) plus a
//! helper for printing messages above it. Works with the terminal in raw mode, so every
//! line ends with an explicit `\r\n`.

use std::io::Write;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use shariki_core::MotionState;

/// State shared between the control loop and the meter task.
#[derive(Default)]
pub struct Status {
    frames: AtomicU64,
    hits: AtomicU64,
    energy: AtomicU32,
    /// e.g. "2/6 Cabasa".
    preset: Mutex<String>,
    /// Key help shown at the end of the line (empty when there is no keyboard).
    keys: Mutex<String>,
}

impl Status {
    pub fn record(&self, m: &MotionState) {
        self.frames.fetch_add(1, Ordering::Relaxed);
        if m.impulse > 0.0 {
            self.hits.fetch_add(1, Ordering::Relaxed);
        }
        self.energy.store(m.energy.to_bits(), Ordering::Relaxed);
    }

    pub fn set_preset(&self, label: String) {
        *self.preset.lock().unwrap_or_else(|e| e.into_inner()) = label;
    }

    pub fn set_keys(&self, help: &str) {
        *self.keys.lock().unwrap_or_else(|e| e.into_inner()) = help.to_string();
    }
}

/// Prints a message on its own line, above the meter.
pub fn say(msg: impl std::fmt::Display) {
    let mut err = std::io::stderr().lock();
    let _ = write!(err, "\r\x1b[2K{msg}\r\n");
    let _ = err.flush();
}

fn bar(value: f32, width: usize) -> String {
    let filled = ((value.clamp(0.0, 1.0) * width as f32).round() as usize).min(width);
    format!("{}{}", "█".repeat(filled), "·".repeat(width - filled))
}

/// Redraws the meter every 100 ms until the task is aborted.
pub fn spawn_meter(status: Arc<Status>, peak: Arc<AtomicU32>) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_millis(100));
        let (mut last_frames, mut last_hits, mut last_t) = (0u64, 0u64, Instant::now());
        let mut rate = 0.0f64;
        let mut hit_flash = 0u8;
        loop {
            tick.tick().await;
            let frames = status.frames.load(Ordering::Relaxed);
            let hits = status.hits.load(Ordering::Relaxed);
            let now = Instant::now();
            let dt = now.duration_since(last_t).as_secs_f64();
            if dt >= 1.0 {
                rate = (frames - last_frames) as f64 / dt;
                (last_frames, last_t) = (frames, now);
            }
            if hits != last_hits {
                hit_flash = 3;
                last_hits = hits;
            }
            let energy = f32::from_bits(status.energy.load(Ordering::Relaxed));
            let level = f32::from_bits(peak.load(Ordering::Relaxed));
            let hit = if hit_flash > 0 { "●" } else { " " };
            hit_flash = hit_flash.saturating_sub(1);
            let preset = status.preset.lock().map(|p| p.clone()).unwrap_or_default();
            let keys = status.keys.lock().map(|k| k.clone()).unwrap_or_default();
            let mut err = std::io::stderr().lock();
            let _ = write!(
                err,
                "\r\x1b[2K{preset:<16} shake {} {hit} out {} {rate:>4.0} Hz  hits {hits:<5} {keys}",
                bar(energy, 16),
                bar(level, 10)
            );
            let _ = err.flush();
        }
    })
}
