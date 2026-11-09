//! Renders every catalog instrument with a synthetic gesture to `target/sounds/<id>.wav`.
//!
//! Shakers get a 4 s gesture: 0.5 s still, 2.5 s of 5 Hz shaking (energy ramping up then
//! down, an impulse at every reversal), 1 s still. The rainstick is turned over once and
//! left to trickle for 5.5 s. Motion states are delivered in bursts of 4, like BLE.
//!
//! ```text
//! cargo run -p shariki-engine --example render [-- <id>...]
//! ```

use std::path::PathBuf;

use shariki_engine::render::{self, stats};

const SAMPLE_RATE: f32 = 48_000.0;
const BLOCK: usize = 256;
const BURST: usize = 4;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let wanted: Vec<String> = std::env::args().skip(1).collect();
    let dir = target_dir().join("sounds");
    std::fs::create_dir_all(&dir)?;

    println!(
        "{:<10} {:>8} {:>8} {:>10} {:>12}  file",
        "id", "rms", "peak", "shake rms", "last 0.25 s"
    );
    for info in shariki_engine::catalog() {
        if !wanted.is_empty() && !wanted.iter().any(|w| w == info.id) {
            continue;
        }
        let mut inst = shariki_engine::create(info.id, SAMPLE_RATE).expect("catalog id");
        let (states, duration) = render::demo_gesture(info.id);
        let audio = render::render(inst.as_mut(), &states, SAMPLE_RATE, duration, BLOCK, BURST);

        let path = dir.join(format!("{}.wav", info.id));
        let spec = hound::WavSpec {
            channels: 1,
            sample_rate: SAMPLE_RATE as u32,
            bits_per_sample: 16,
            sample_format: hound::SampleFormat::Int,
        };
        let mut w = hound::WavWriter::create(&path, spec)?;
        for &s in &audio {
            w.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
        }
        w.finalize()?;

        let all = stats(&audio);
        let sec = |t: f32| ((t * SAMPLE_RATE) as usize).min(audio.len());
        let busy = stats(&audio[sec(0.5)..sec(3.0)]);
        let tail = stats(&audio[sec(duration - 0.25)..]);
        println!(
            "{:<10} {:>8.4} {:>8.4} {:>10.4} {:>12.6}  {}",
            info.id,
            all.rms,
            all.peak,
            busy.rms,
            tail.peak,
            path.display()
        );
    }
    Ok(())
}

/// `<workspace>/target`, honouring `CARGO_TARGET_DIR`.
fn target_dir() -> PathBuf {
    if let Some(d) = std::env::var_os("CARGO_TARGET_DIR") {
        return PathBuf::from(d);
    }
    // crates/engine → workspace root.
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target")
}
