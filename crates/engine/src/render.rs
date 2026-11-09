//! Offline rendering helpers for tests, examples and calibration.
//!
//! Not real-time code: these allocate freely. They build synthetic [`MotionState`]
//! sequences (a hand shake, a rainstick turn) and play them into an [`Instrument`] the way
//! the app does — states delivered in bursts between audio blocks.

use std::f32::consts::{PI, TAU};

use shariki_core::{Instrument, MotionState};

/// Nominal motion-state rate of `shariki-motion` (the IMU rate), Hz.
pub const MOTION_RATE: f32 = 104.0;

/// Level statistics of a rendered buffer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    pub rms: f32,
    pub peak: f32,
    pub all_finite: bool,
}

/// RMS, peak and finiteness of `samples` (an empty buffer is silent).
pub fn stats(samples: &[f32]) -> Stats {
    let mut sum = 0.0f64;
    let mut peak = 0.0f32;
    let mut all_finite = true;
    for &s in samples {
        if !s.is_finite() {
            all_finite = false;
            continue;
        }
        sum += (s as f64) * (s as f64);
        peak = peak.max(s.abs());
    }
    let rms = if samples.is_empty() {
        0.0
    } else {
        (sum / samples.len() as f64).sqrt() as f32
    };
    Stats {
        rms,
        peak,
        all_finite,
    }
}

/// A synthetic hand-shake gesture: still, then a periodic shake whose energy ramps up and
/// back down (half-sine envelope) with an impulse at every direction reversal, then still.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShakeGesture {
    /// Seconds of stillness before the shake.
    pub still_before: f32,
    /// Seconds of shaking.
    pub shake: f32,
    /// Seconds of stillness after the shake.
    pub still_after: f32,
    /// Back-and-forth cycles per second (two reversals per cycle).
    pub shake_hz: f32,
    /// Energy (and impulse strength) at the top of the envelope.
    pub peak_energy: f32,
    /// Motion-state rate, Hz.
    pub motion_rate: f32,
}

impl Default for ShakeGesture {
    /// 0.5 s still, 2.5 s of 5 Hz shaking peaking at energy 1.0, 1.0 s still (4 s total).
    fn default() -> Self {
        Self {
            still_before: 0.5,
            shake: 2.5,
            still_after: 1.0,
            shake_hz: 5.0,
            peak_energy: 1.0,
            motion_rate: MOTION_RATE,
        }
    }
}

impl ShakeGesture {
    pub fn duration(&self) -> f32 {
        self.still_before + self.shake + self.still_after
    }

    /// The motion-state sequence at `motion_rate`.
    pub fn states(&self) -> Vec<MotionState> {
        let n = (self.duration() * self.motion_rate).ceil() as usize;
        let dt = 1.0 / self.motion_rate;
        let half_period = 0.5 / self.shake_hz.max(1e-3);
        let mut next_reversal = self.still_before + half_period;
        (0..n)
            .map(|i| {
                let t = i as f32 * dt;
                let u = (t - self.still_before) / self.shake;
                let shaking = (0.0..1.0).contains(&u);
                let env = if shaking { (PI * u).sin() } else { 0.0 };
                let phase = TAU * self.shake_hz * (t - self.still_before);
                let mut impulse = 0.0;
                if shaking && t + 0.5 * dt >= next_reversal {
                    impulse = self.peak_energy * env;
                    next_reversal += half_period;
                }
                MotionState {
                    t: t as f64,
                    energy: self.peak_energy * env,
                    impulse,
                    up: [0.05 * env * phase.sin(), 0.0, 1.0],
                    angular_speed: 400.0 * env * phase.cos().abs(),
                    linear_accel: 2.5 * env * phase.sin().abs(),
                }
            })
            .collect()
    }
}

/// A rainstick gesture: still, then the stick is turned over (the `up` vector rotates by
/// 180° in `turn` seconds, with a little motion energy), then held still while the grains
/// keep falling.
pub fn turn_over_gesture(
    still_before: f32,
    turn: f32,
    hold: f32,
    motion_rate: f32,
) -> Vec<MotionState> {
    let duration = still_before + turn + hold;
    let n = (duration * motion_rate).ceil() as usize;
    (0..n)
        .map(|i| {
            let t = i as f32 / motion_rate;
            let u = ((t - still_before) / turn).clamp(0.0, 1.0);
            let turning = t >= still_before && t < still_before + turn;
            let angle = PI * u;
            MotionState {
                t: t as f64,
                energy: if turning { 0.1 } else { 0.0 },
                impulse: 0.0,
                up: [angle.sin(), 0.0, angle.cos()],
                angular_speed: if turning { 180.0 / turn } else { 0.0 },
                linear_accel: if turning { 0.1 } else { 0.0 },
            }
        })
        .collect()
}

/// The demo gesture for a catalog instrument and its duration in seconds: a 4 s shake for
/// shakers, a 6 s turn-over for the rainstick.
pub fn demo_gesture(id: &str) -> (Vec<MotionState>, f32) {
    if id == "rainstick" {
        (turn_over_gesture(0.5, 0.5, 5.0, MOTION_RATE), 6.0)
    } else {
        let g = ShakeGesture::default();
        (g.states(), g.duration())
    }
}

/// Plays `states` into `inst` and renders `duration` seconds of audio.
///
/// Audio is rendered in blocks of `block_size` samples. Before each block, the states whose
/// time has come are delivered — but only once at least `burst` of them are pending,
/// mimicking BLE notifications that carry several IMU samples at once (`burst = 1` delivers
/// every state as soon as possible).
pub fn render(
    inst: &mut dyn Instrument,
    states: &[MotionState],
    sample_rate: f32,
    duration: f32,
    block_size: usize,
    burst: usize,
) -> Vec<f32> {
    let total = (duration * sample_rate).round() as usize;
    let block_size = block_size.max(1);
    let burst = burst.max(1);
    let mut out = vec![0.0f32; total];
    let mut next = 0usize;
    let mut start = 0usize;
    while start < total {
        let end = (start + block_size).min(total);
        let block_end_t = end as f64 / sample_rate as f64;
        let due = states[next..]
            .iter()
            .take_while(|s| s.t < block_end_t)
            .count();
        if due >= burst || (due > 0 && next + due == states.len()) {
            for s in &states[next..next + due] {
                inst.update(s);
            }
            next += due;
        }
        inst.render(&mut out[start..end]);
        start = end;
    }
    out
}

/// Renders a constant motion state (fed once per motion period) for `duration` seconds.
pub fn render_constant(
    inst: &mut dyn Instrument,
    state: MotionState,
    sample_rate: f32,
    duration: f32,
    block_size: usize,
) -> Vec<f32> {
    let n = (duration * MOTION_RATE).ceil() as usize;
    let states: Vec<MotionState> = (0..n)
        .map(|i| MotionState {
            t: i as f64 / MOTION_RATE as f64,
            ..state
        })
        .collect();
    render(inst, &states, sample_rate, duration, block_size, 1)
}
