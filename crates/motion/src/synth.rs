//! Deterministic synthetic [`ImuFrame`] streams, for tests and for a demo mode without a
//! token.
//!
//! A [`Synth`] is a sequence of segments (still, shake, tilt, flick) played back at a
//! fixed rate. It tracks the token's orientation across segments, so a tilt leaves the
//! following segments in the new orientation, and the gyro always matches the rotation
//! that the accelerometer's gravity vector undergoes.
//!
//! ```
//! use shariki_motion::synth::{Shake, Synth};
//! let frames: Vec<_> = Synth::new(104.0)
//!     .still(1.0)
//!     .shake(2.0, Shake::new(5.0, 2.0).fade(0.3))
//!     .still(1.0)
//!     .frames();
//! assert_eq!(frames.len(), 416);
//! ```
//!
//! Frames have `index = i` and `t = i / rate_hz`, both starting at 0 and contiguous.

use crate::math::{V3, add, normalize, rotate_world_vec, scale};
use shariki_core::ImuFrame;
use std::f32::consts::PI;

/// Nominal output rate of the token's IMU.
pub const DEFAULT_RATE_HZ: f64 = 104.0;

/// Parameters of a sinusoidal shake (see [`Synth::shake`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shake {
    /// Shake frequency in Hz (full back-and-forth cycles per second).
    pub freq_hz: f32,
    /// Peak linear acceleration in g (gravity not included).
    pub amp_g: f32,
    /// Direction of travel, sensor frame (normalised internally).
    pub axis: [f32; 3],
    /// Peak wrist rotation rate in deg/s that accompanies the shake (0 = pure translation).
    pub wrist_dps: f32,
    /// Wrist rotation axis, sensor frame (normalised internally).
    pub wrist_axis: [f32; 3],
    /// Raised-cosine fade-in and fade-out time in seconds. Half the segment duration
    /// makes the amplitude swell up and back down over the whole segment.
    pub fade_s: f32,
}

impl Shake {
    /// Pure translation along +X with no fade.
    pub fn new(freq_hz: f32, amp_g: f32) -> Self {
        Self {
            freq_hz,
            amp_g,
            axis: [1.0, 0.0, 0.0],
            wrist_dps: 0.0,
            wrist_axis: [0.0, 1.0, 0.0],
            fade_s: 0.0,
        }
    }
    /// Direction of travel (sensor frame).
    pub fn axis(mut self, axis: [f32; 3]) -> Self {
        self.axis = axis;
        self
    }
    /// Add a matching oscillating wrist rotation with the given peak rate and axis.
    pub fn wrist(mut self, peak_dps: f32, axis: [f32; 3]) -> Self {
        self.wrist_dps = peak_dps;
        self.wrist_axis = axis;
        self
    }
    /// Raised-cosine fade in/out time.
    pub fn fade(mut self, fade_s: f32) -> Self {
        self.fade_s = fade_s;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    Still,
    Shake(Shake),
    /// Constant-rate rotation about a sensor-frame axis, total `degrees`.
    Tilt {
        axis: V3,
        degrees: f32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Segment {
    duration_s: f64,
    kind: Kind,
}

/// Sensor noise added to every frame (deterministic, uniform, given RMS).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Noise {
    pub accel_g_rms: f32,
    pub gyro_dps_rms: f32,
    /// Constant gyro zero-rate offset.
    pub gyro_bias_dps: [f32; 3],
    pub seed: u64,
}

/// A sequence of synthetic motion segments; build it with the chaining methods and turn
/// it into frames with [`Synth::frames`] or by iterating.
#[derive(Debug, Clone, PartialEq)]
pub struct Synth {
    rate_hz: f64,
    start_up: V3,
    segments: Vec<Segment>,
    noise: Option<Noise>,
}

impl Synth {
    /// Empty sequence at `rate_hz`, token lying PCB face up (`up = +Z`).
    pub fn new(rate_hz: f64) -> Self {
        Self {
            rate_hz: if rate_hz.is_finite() && rate_hz > 0.0 {
                rate_hz
            } else {
                DEFAULT_RATE_HZ
            },
            start_up: [0.0, 0.0, 1.0],
            segments: Vec::new(),
            noise: None,
        }
    }

    /// Initial orientation: the sensor-frame direction that faces the sky.
    pub fn start_up(mut self, up: [f32; 3]) -> Self {
        self.start_up = normalize(up).unwrap_or([0.0, 0.0, 1.0]);
        self
    }

    /// Add sensor noise to every frame.
    pub fn noise(mut self, noise: Noise) -> Self {
        self.noise = Some(noise);
        self
    }

    /// Hold still for `duration_s`.
    pub fn still(mut self, duration_s: f64) -> Self {
        self.push(duration_s, Kind::Still);
        self
    }

    /// Shake for `duration_s` around the current orientation.
    pub fn shake(mut self, duration_s: f64, shake: Shake) -> Self {
        self.push(duration_s, Kind::Shake(shake));
        self
    }

    /// One sharp flick: a single full cycle at `freq_hz` (push, then stop) with peak
    /// `amp_g` along `axis`, with a little wrist snap.
    pub fn flick(self, freq_hz: f32, amp_g: f32, axis: [f32; 3]) -> Self {
        let f = freq_hz.max(0.1);
        let shake = Shake::new(f, amp_g)
            .axis(axis)
            .wrist(amp_g * 150.0, [0.0, 1.0, 0.0]);
        self.shake(f64::from(1.0 / f), shake)
    }

    /// Rotate the token by `degrees` about the sensor-frame `axis` at a constant rate
    /// over `duration_s` (e.g. turning a rainstick over: `tilt(3.0, [1,0,0], 180.0)`).
    pub fn tilt(mut self, duration_s: f64, axis: [f32; 3], degrees: f32) -> Self {
        let axis = normalize(axis).unwrap_or([1.0, 0.0, 0.0]);
        self.push(duration_s, Kind::Tilt { axis, degrees });
        self
    }

    fn push(&mut self, duration_s: f64, kind: Kind) {
        if duration_s.is_finite() && duration_s > 0.0 {
            self.segments.push(Segment { duration_s, kind });
        }
    }

    /// Output rate in Hz.
    pub fn rate_hz(&self) -> f64 {
        self.rate_hz
    }

    /// Total number of frames.
    pub fn len(&self) -> usize {
        self.segments.iter().map(|s| self.seg_len(s) as usize).sum()
    }

    /// `true` if the sequence produces no frames.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Total duration in seconds.
    pub fn duration_s(&self) -> f64 {
        self.len() as f64 / self.rate_hz
    }

    fn seg_len(&self, s: &Segment) -> u64 {
        (s.duration_s * self.rate_hz).round() as u64
    }

    /// Iterate over the frames.
    pub fn iter(&self) -> Frames {
        Frames {
            synth: self.clone(),
            index: 0,
            seg: 0,
            local: 0,
            up: self.start_up,
            rng: self.noise.map_or(1, |n| n.seed.max(1)),
        }
    }

    /// Collect all frames.
    pub fn frames(&self) -> Vec<ImuFrame> {
        self.iter().collect()
    }
}

impl IntoIterator for &Synth {
    type Item = ImuFrame;
    type IntoIter = Frames;
    fn into_iter(self) -> Frames {
        self.iter()
    }
}

/// Iterator over a [`Synth`]'s frames.
#[derive(Debug, Clone)]
pub struct Frames {
    synth: Synth,
    index: u64,
    seg: usize,
    local: u64,
    /// Sky direction in the sensor frame at the current sample.
    up: V3,
    rng: u64,
}

impl Frames {
    /// Uniform in [-1, 1) (xorshift64*).
    fn rand(&mut self) -> f32 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let r = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
        ((r >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
    }
}

/// Gyro rate (deg/s) and linear accel (g) of a segment at local time `tau`.
fn motion_at(kind: &Kind, tau: f32, dur: f32) -> (V3, V3) {
    match *kind {
        Kind::Still => ([0.0; 3], [0.0; 3]),
        Kind::Tilt { axis, degrees } => (scale(axis, degrees / dur), [0.0; 3]),
        Kind::Shake(s) => {
            let env = fade_env(tau, dur, s.fade_s);
            let ph = 2.0 * PI * s.freq_hz * tau;
            let axis = normalize(s.axis).unwrap_or([1.0, 0.0, 0.0]);
            let lin = scale(axis, s.amp_g * env * ph.sin());
            let gyro = match normalize(s.wrist_axis) {
                Some(w) if s.wrist_dps != 0.0 => scale(w, s.wrist_dps * env * ph.cos()),
                _ => [0.0; 3],
            };
            (gyro, lin)
        }
    }
}

fn fade_env(tau: f32, dur: f32, fade: f32) -> f32 {
    if fade <= 0.0 {
        return 1.0;
    }
    let ramp = |x: f32| {
        let x = (x / fade).clamp(0.0, 1.0);
        0.5 - 0.5 * (PI * x).cos()
    };
    ramp(tau) * ramp(dur - tau)
}

impl Iterator for Frames {
    type Item = ImuFrame;

    fn next(&mut self) -> Option<ImuFrame> {
        let rate = self.synth.rate_hz;
        let seg = loop {
            let seg = *self.synth.segments.get(self.seg)?;
            if self.local < self.synth.seg_len(&seg) {
                break seg;
            }
            self.seg += 1;
            self.local = 0;
        };

        let dt = (1.0 / rate) as f32;
        let dur = seg.duration_s as f32;
        let tau = self.local as f32 * dt;
        let (gyro, lin) = motion_at(&seg.kind, tau, dur);
        let mut accel = add(self.up, lin);
        let mut gyro_out = gyro;

        if let Some(n) = self.synth.noise {
            let ka = n.accel_g_rms * 3f32.sqrt();
            let kg = n.gyro_dps_rms * 3f32.sqrt();
            for i in 0..3 {
                accel[i] += ka * self.rand();
                gyro_out[i] += kg * self.rand() + n.gyro_bias_dps[i];
            }
        }

        let frame = ImuFrame {
            t: self.index as f64 / rate,
            index: self.index,
            gyro_dps: gyro_out,
            accel_g: accel,
        };

        // Advance the orientation to the next sample using the mid-interval rate.
        let (gyro_mid, _) = motion_at(&seg.kind, tau + 0.5 * dt, dur);
        self.up = normalize(rotate_world_vec(self.up, gyro_mid, dt)).unwrap_or(self.up);
        self.index += 1;
        self.local += 1;
        Some(frame)
    }
}

/// Token lying still for `duration_s` with `up` facing the sky.
pub fn still(rate_hz: f64, up: [f32; 3], duration_s: f64) -> Vec<ImuFrame> {
    Synth::new(rate_hz).start_up(up).still(duration_s).frames()
}

/// A ~9 s demo gesture at `rate_hz` (for `--demo` without a token): 1 s still, a shake
/// that swells from gentle to vigorous and fades back, 1 s still, two sharp single
/// flicks, then still. `index` / `t` start at 0 and are contiguous; the token ends in
/// the orientation it started in, so the result can be looped.
pub fn demo(rate_hz: f64) -> Vec<ImuFrame> {
    Synth::new(rate_hz)
        .still(1.0)
        .shake(
            4.0,
            Shake::new(5.5, 3.0)
                .axis([1.0, 0.0, 0.2])
                .wrist(700.0, [0.0, 1.0, 0.0])
                .fade(2.0),
        )
        .still(1.0)
        .flick(7.0, 3.0, [1.0, 0.0, 0.0])
        .still(0.6)
        .flick(6.0, 2.5, [0.0, 1.0, 0.0])
        .still(1.6)
        .noise(Noise {
            accel_g_rms: 0.004,
            gyro_dps_rms: 0.3,
            gyro_bias_dps: [0.0; 3],
            seed: 7,
        })
        .frames()
}
