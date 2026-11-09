use shariki_core::{ImuFrame, MotionState};
use shariki_motion::synth::{self, Noise, Shake, Synth};
use shariki_motion::{MotionConfig, MotionTracker};

const RATE: f64 = 104.0;

fn run(frames: &[ImuFrame]) -> Vec<MotionState> {
    let mut t = MotionTracker::new(MotionConfig::default());
    frames.iter().map(|f| t.push(f)).collect()
}

fn noise() -> Noise {
    Noise {
        accel_g_rms: 0.01,
        gyro_dps_rms: 1.0,
        gyro_bias_dps: [1.5, -1.0, 0.8],
        seed: 42,
    }
}

fn angle_deg(a: [f32; 3], b: [f32; 3]) -> f32 {
    let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let na = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    let nb = (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt();
    (d / (na * nb)).clamp(-1.0, 1.0).acos().to_degrees()
}

fn impulses(states: &[MotionState]) -> usize {
    states.iter().filter(|s| s.impulse > 0.0).count()
}

fn mean_energy(states: &[MotionState]) -> f32 {
    states.iter().map(|s| s.energy).sum::<f32>() / states.len() as f32
}

fn assert_finite(s: &MotionState) {
    assert!(s.t.is_finite(), "{s:?}");
    for v in [
        s.energy,
        s.impulse,
        s.angular_speed,
        s.linear_accel,
        s.up[0],
        s.up[1],
        s.up[2],
    ] {
        assert!(v.is_finite(), "{s:?}");
    }
}

#[test]
fn still_in_six_orientations() {
    let ups = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ];
    for up in ups {
        let frames = Synth::new(RATE)
            .start_up(up)
            .noise(noise())
            .still(3.0)
            .frames();
        let states = run(&frames);
        assert_eq!(impulses(&states), 0, "up {up:?}");
        for s in &states {
            assert!(s.energy < 0.02, "up {up:?}: {s:?}");
            assert!(angle_deg(s.up, up) < 2.0, "up {up:?}: {s:?}");
            let n = (s.up[0] * s.up[0] + s.up[1] * s.up[1] + s.up[2] * s.up[2]).sqrt();
            assert!((n - 1.0).abs() < 1e-3);
        }
        // Noiseless rest reads exactly zero.
        let clean = run(&synth::still(RATE, up, 1.0));
        assert!(clean.iter().all(|s| s.energy == 0.0 && s.impulse == 0.0));
    }
}

#[test]
fn shake_5hz_2g_energy_and_impulses() {
    for wrist in [0.0, 400.0] {
        let frames = Synth::new(RATE)
            .noise(noise())
            .still(1.0)
            .shake(
                4.0,
                Shake::new(5.0, 2.0)
                    .axis([0.0, 1.0, 0.0])
                    .wrist(wrist, [1.0, 0.0, 0.0]),
            )
            .still(1.0)
            .frames();
        let states = run(&frames);
        let n = RATE as usize;
        // Skip the first 0.5 s of the shake for the energy level.
        let shaking = &states[n + n / 2..5 * n];
        let e = mean_energy(shaking);
        assert!((0.4..=1.2).contains(&e), "wrist {wrist}: mean energy {e}");
        // 4 s * 5 Hz = 20 cycles -> ~40 reversals (the very first push does not click).
        let count = impulses(&states[n..5 * n + n / 4]);
        assert!(
            (36..=42).contains(&count),
            "wrist {wrist}: {count} impulses"
        );
        for s in &states[n..5 * n] {
            assert!(s.impulse <= 1.0);
        }
        let strengths: Vec<f32> = shaking
            .iter()
            .filter(|s| s.impulse > 0.0)
            .map(|s| s.impulse)
            .collect();
        let avg = strengths.iter().sum::<f32>() / strengths.len() as f32;
        assert!((0.3..=1.0).contains(&avg), "avg impulse {avg}");
        // Nothing before the shake; settles back to zero after it.
        assert_eq!(impulses(&states[..n]), 0);
        assert!(states.last().unwrap().energy < 0.02);
        // up stays put during a pure linear shake.
        if wrist == 0.0 {
            for s in shaking {
                assert!(angle_deg(s.up, [0.0, 0.0, 1.0]) < 5.0, "{s:?}");
            }
        }
    }
}

#[test]
fn vertical_shake_keeps_up() {
    // Accel swings through -1 g, which fools a plain |a| ~ 1 g gate.
    let frames = Synth::new(RATE)
        .still(0.5)
        .shake(3.0, Shake::new(4.0, 2.0).axis([0.0, 0.0, 1.0]))
        .frames();
    let states = run(&frames);
    for s in &states {
        assert!(angle_deg(s.up, [0.0, 0.0, 1.0]) < 5.0, "{s:?}");
    }
    let count = impulses(&states);
    assert!((22..=24).contains(&count), "{count}");
}

#[test]
fn gentle_is_lower_than_vigorous() {
    let level = |freq, amp, wrist| {
        let frames = Synth::new(RATE)
            .shake(3.0, Shake::new(freq, amp).wrist(wrist, [0.0, 0.0, 1.0]))
            .frames();
        mean_energy(&run(&frames)[RATE as usize..])
    };
    let gentle = level(3.0, 0.5, 60.0);
    let medium = level(5.0, 2.0, 300.0);
    let vigorous = level(6.0, 3.5, 1000.0);
    assert!(gentle < 0.25, "gentle {gentle}");
    assert!(
        gentle < medium && medium < vigorous,
        "{gentle} {medium} {vigorous}"
    );
    assert!((0.8..=1.6).contains(&vigorous), "vigorous {vigorous}");
}

#[test]
fn slow_tilt_is_followed() {
    let frames = Synth::new(RATE)
        .noise(noise())
        .still(0.5)
        .tilt(2.0, [1.0, 0.0, 0.0], 90.0)
        .still(1.0)
        .frames();
    let states = run(&frames);
    for (f, s) in frames.iter().zip(&states) {
        // The noiseless sky direction is the accel direction (no linear motion here).
        assert!(angle_deg(s.up, f.accel_g) < 3.0, "{f:?} {s:?}");
        assert!(s.impulse == 0.0);
        assert!(s.energy < 0.05);
    }
    // Turning about +X by 90 deg brings the sky onto +Y.
    let end = states.last().unwrap().up;
    assert!(angle_deg(end, [0.0, 1.0, 0.0]) < 2.0, "{end:?}");
}

#[test]
fn rainstick_turn_over() {
    let frames = Synth::new(RATE)
        .tilt(3.0, [0.0, 1.0, 0.0], 180.0)
        .still(0.5)
        .frames();
    let end = run(&frames).last().unwrap().up;
    assert!(angle_deg(end, [0.0, 0.0, -1.0]) < 2.0, "{end:?}");
}

#[test]
fn gap_does_not_spike() {
    // Still face up, 0.5 s drop-out during which the token was turned onto its side.
    let mut frames = synth::still(RATE, [0.0, 0.0, 1.0], 1.0);
    let after = synth::still(RATE, [1.0, 0.0, 0.0], 1.0);
    let offset = frames.len() as u64 + 52;
    frames.extend(after.into_iter().map(|mut f| {
        f.index += offset;
        f.t = f.index as f64 / RATE;
        f
    }));
    let states = run(&frames);
    assert_eq!(impulses(&states), 0);
    for s in &states {
        assert!(s.energy < 0.02, "{s:?}");
    }
    assert!(angle_deg(states.last().unwrap().up, [1.0, 0.0, 0.0]) < 1.0);

    // A gap in the middle of a shake: no impulse on the frame after the gap.
    let shake = Synth::new(RATE).shake(2.0, Shake::new(5.0, 2.0)).frames();
    let (a, b) = shake.split_at(100);
    let mut frames = a.to_vec();
    frames.extend(b.iter().skip(30).map(|f| ImuFrame { t: f.t + 0.3, ..*f }));
    let states = run(&frames);
    assert_eq!(states[100].impulse, 0.0);
    assert!(states.iter().all(|s| s.energy < 1.5 && s.impulse <= 1.0));
}

#[test]
fn time_going_backwards_and_duplicates() {
    let frames = synth::still(RATE, [0.0, 0.0, 1.0], 0.5);
    let mut t = MotionTracker::default();
    for f in &frames {
        t.push(f);
    }
    let dup = t.push(frames.last().unwrap());
    assert_eq!(dup.impulse, 0.0);
    let back = t.push(&ImuFrame {
        t: 0.0,
        index: 0,
        ..frames[0]
    });
    assert_finite(&back);
    assert_eq!(back.impulse, 0.0);
    assert!(back.energy < 0.02);
}

#[test]
fn no_nan_for_degenerate_inputs() {
    let mut t = MotionTracker::default();
    let cases: [([f32; 3], [f32; 3]); 7] = [
        ([0.0; 3], [0.0; 3]),
        ([1e30, -1e30, 1e30], [1e30, 1e30, -1e30]),
        ([f32::MAX; 3], [f32::MIN; 3]),
        ([f32::NAN; 3], [f32::NAN; 3]),
        ([f32::INFINITY, 0.0, 0.0], [0.0, f32::NEG_INFINITY, 0.0]),
        ([2000.0, 2000.0, 2000.0], [16.0, -16.0, 16.0]),
        ([0.0; 3], [0.0, 0.0, 1.0]),
    ];
    let mut i = 0u64;
    for _ in 0..50 {
        for (g, a) in cases {
            let s = t.push(&ImuFrame {
                t: i as f64 / RATE,
                index: i,
                gyro_dps: g,
                accel_g: a,
            });
            assert_finite(&s);
            assert!(s.impulse <= 1.0);
            i += 1;
        }
    }
    for tt in [f64::NAN, f64::INFINITY, -1.0, 1e12] {
        let s = t.push(&ImuFrame {
            t: tt,
            index: 0,
            gyro_dps: [0.0; 3],
            accel_g: [0.0, 0.0, 1.0],
        });
        assert_finite(&s);
    }
    // Recovers to a sane rest state afterwards.
    let base = 2e12;
    let mut last = None;
    for f in synth::still(RATE, [0.0, 1.0, 0.0], 3.0) {
        last = Some(t.push(&ImuFrame { t: base + f.t, ..f }));
    }
    let last = last.unwrap();
    assert!(last.energy < 0.02);
    assert!(angle_deg(last.up, [0.0, 1.0, 0.0]) < 1.0);
}

#[test]
fn reset_forgets_history() {
    let mut t = MotionTracker::default();
    for f in Synth::new(RATE).shake(1.0, Shake::new(5.0, 3.0)).frames() {
        t.push(&f);
    }
    t.reset();
    let s = t.push(&synth::still(RATE, [0.0, -1.0, 0.0], 0.1)[0]);
    assert_eq!(s.energy, 0.0);
    assert!(angle_deg(s.up, [0.0, -1.0, 0.0]) < 0.1);
    assert_eq!(t.config(), &MotionConfig::default());
}

#[test]
fn rate_independent() {
    // Same gesture at 52 / 104 / 208 Hz gives similar features.
    let res: Vec<(f32, usize)> = [52.0, 104.0, 208.0]
        .iter()
        .map(|&rate| {
            let frames = Synth::new(rate).shake(3.0, Shake::new(5.0, 2.0)).frames();
            let states = run(&frames);
            (mean_energy(&states[rate as usize..]), impulses(&states))
        })
        .collect();
    for (e, n) in &res {
        assert!((e - res[1].0).abs() < 0.1, "{res:?}");
        assert!(n.abs_diff(res[1].1) <= 1, "{res:?}");
    }
}

#[test]
fn demo_is_contiguous_and_lively() {
    let frames = synth::demo(RATE);
    let dur = frames.len() as f64 / RATE;
    assert!((8.0..=10.5).contains(&dur), "{dur}");
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.index, i as u64);
        assert!((f.t - i as f64 / RATE).abs() < 1e-9);
    }
    let states = run(&frames);
    let n = RATE as usize;
    assert!(
        states[..n]
            .iter()
            .all(|s| s.energy < 0.02 && s.impulse == 0.0)
    );
    let peak = states[n..5 * n]
        .iter()
        .map(|s| s.energy)
        .fold(0.0, f32::max);
    assert!(peak > 0.7, "peak {peak}");
    assert!(impulses(&states[n..5 * n]) > 20);
    // Two flicks after the shake, each at least one click.
    assert!(impulses(&states[6 * n..]) >= 2);
    let end = states.last().unwrap();
    assert!(end.energy < 0.02, "{end:?}");
    assert!(angle_deg(end.up, [0.0, 0.0, 1.0]) < 3.0);
    // Other rates work too.
    let f52 = synth::demo(52.0);
    assert_eq!(f52.last().unwrap().index as usize + 1, f52.len());
}
