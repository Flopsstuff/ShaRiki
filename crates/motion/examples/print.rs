//! Run a synthetic gesture through the tracker and print one line per ~50 ms:
//! time, energy bar, impulse marker (strongest in the window), up vector, |lin|, |ω|.
//!
//! `cargo run -p shariki-motion --example print [demo|tilt]`

use shariki_motion::synth::{self, Shake, Synth};
use shariki_motion::{MotionConfig, MotionTracker};

fn main() {
    let rate = synth::DEFAULT_RATE_HZ;
    let frames = match std::env::args().nth(1).as_deref() {
        Some("tilt") => Synth::new(rate)
            .still(0.5)
            .tilt(3.0, [1.0, 0.0, 0.0], 180.0)
            .still(0.5)
            .shake(1.5, Shake::new(5.0, 2.0).axis([0.0, 1.0, 0.0]))
            .still(0.5)
            .frames(),
        _ => synth::demo(rate),
    };

    let mut tracker = MotionTracker::new(MotionConfig::default());
    let window = (rate * 0.05).round().max(1.0) as usize;
    let mut imp_count = 0;
    for chunk in frames.chunks(window) {
        let mut imp: f32 = 0.0;
        let mut last = None;
        for f in chunk {
            let s = tracker.push(f);
            if s.impulse > 0.0 {
                imp_count += 1;
            }
            imp = imp.max(s.impulse);
            last = Some(s);
        }
        let s = last.unwrap();
        let n = ((s.energy * 30.0).round() as usize).min(45);
        let marker = if imp <= 0.0 {
            "  "
        } else if imp < 0.33 {
            " ."
        } else if imp < 0.66 {
            " o"
        } else {
            " O"
        };
        println!(
            "{:6.2}s {:<45} e={:4.2}{} up=[{:5.2} {:5.2} {:5.2}] lin={:4.2}g w={:5.0}",
            s.t,
            "#".repeat(n),
            s.energy,
            marker,
            s.up[0],
            s.up[1],
            s.up[2],
            s.linear_accel,
            s.angular_speed
        );
    }
    println!("impulses: {imp_count}");
}
