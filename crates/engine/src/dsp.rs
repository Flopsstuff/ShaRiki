//! Small real-time-safe DSP building blocks: RNG, two-pole resonator, soft clipper.
//!
//! Everything here is plain arithmetic on fixed-size state: no allocation, no locks.

use std::f32::consts::TAU;

/// Values below this are flushed to exactly zero to avoid denormals and to let the engine
/// reach true digital silence.
pub(crate) const FLUSH: f32 = 1.0e-9;

/// Free-ringing filter states below this (−120 dBFS) are cleared.
pub(crate) const RING_FLUSH: f32 = 1.0e-6;

/// PCG32 (O'Neill), XSH-RR variant. Deterministic, tiny, allocation-free.
#[derive(Debug, Clone)]
pub(crate) struct Rng {
    state: u64,
    inc: u64,
}

impl Rng {
    const MUL: u64 = 6_364_136_223_846_793_005;

    pub(crate) fn new(seed: u64) -> Self {
        let mut rng = Self {
            state: 0,
            inc: (0x14057B7EF767814F_u64 << 1) | 1,
        };
        rng.next_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.next_u32();
        rng
    }

    #[inline]
    pub(crate) fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old.wrapping_mul(Self::MUL).wrapping_add(self.inc);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    /// Uniform in `[0, 1)`.
    #[inline]
    pub(crate) fn uniform(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }

    /// Uniform in `[-1, 1)`.
    #[inline]
    pub(crate) fn noise(&mut self) -> f32 {
        self.uniform() * 2.0 - 1.0
    }
}

/// Converts a per-sample decay factor specified at `ref_rate` into a time constant in seconds.
pub(crate) fn decay_to_tau(per_sample: f32, ref_rate: f32) -> f32 {
    -1.0 / (ref_rate * per_sample.ln())
}

/// Per-sample one-pole coefficient for a time constant `tau` (seconds).
pub(crate) fn tau_to_coef(tau: f32, sample_rate: f32) -> f32 {
    if tau <= 0.0 {
        0.0
    } else {
        (-1.0 / (tau * sample_rate)).exp()
    }
}

/// Two-pole all-pole resonator `y = g·x − a1·y[n−1] − a2·y[n−2]` as in STK's
/// `Shakers::tickResonance`, normalised to unit white-noise power gain so that the
/// per-resonance `gain` values are directly comparable.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Resonator {
    /// Centre frequency in Hz (after `size` scaling).
    pub freq: f32,
    /// Pole radius at the current sample rate.
    pub radius: f32,
    /// Relative random detune per collision (0 = fixed resonance).
    pub vary: f32,
    /// Mix gain, already multiplied by the noise-power normalisation.
    pub gain: f32,
    a1: f32,
    a2: f32,
    y1: f32,
    y2: f32,
}

impl Resonator {
    /// (Re)configures the filter. `radius` is for `sample_rate` already.
    pub(crate) fn configure(
        &mut self,
        freq: f32,
        radius: f32,
        gain: f32,
        vary: f32,
        sample_rate: f32,
    ) {
        self.freq = freq;
        self.radius = radius;
        self.vary = vary;
        self.a2 = radius * radius;
        self.a1 = -2.0 * radius * (TAU * freq / sample_rate).cos();
        // White-noise power gain of 1 / (1 + a1 z^-1 + a2 z^-2).
        let (a1, a2) = (self.a1, self.a2);
        let power = (1.0 + a2) / ((1.0 - a2) * ((1.0 + a2) * (1.0 + a2) - a1 * a1));
        let norm = if power.is_finite() && power > 0.0 {
            1.0 / power.sqrt()
        } else {
            0.0
        };
        self.gain = gain * norm;
    }

    /// Re-tunes the resonance to `freq` without touching gain or state (per-collision jitter).
    #[inline]
    pub(crate) fn retune(&mut self, freq: f32, sample_rate: f32) {
        self.a1 = -2.0 * self.radius * (TAU * freq / sample_rate).cos();
    }

    #[inline]
    pub(crate) fn tick(&mut self, x: f32) -> f32 {
        let y = self.gain * x - self.a1 * self.y1 - self.a2 * self.y2;
        if x == 0.0 && y.abs() < RING_FLUSH && self.y1.abs() < RING_FLUSH {
            // Free ringing below −120 dB: stop it. Besides denormals this also kills the
            // tiny zero-input limit cycles f32 rounding produces for poles near the unit
            // circle, which would otherwise keep the output from ever reaching true zero.
            self.y1 = 0.0;
            self.y2 = 0.0;
            return 0.0;
        }
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    pub(crate) fn is_silent(&self) -> bool {
        self.y1 == 0.0 && self.y2 == 0.0
    }

    pub(crate) fn clear(&mut self) {
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

/// Soft clipper: identity below `KNEE`, then a tanh knee that approaches ±1 asymptotically.
/// Transparent at normal levels, never exceeds 1.0 in magnitude, maps NaN to 0.
#[inline]
pub(crate) fn soft_clip(x: f32) -> f32 {
    const KNEE: f32 = 0.6;
    const HEAD: f32 = 1.0 - KNEE;
    if x.is_nan() {
        return 0.0;
    }
    let a = x.abs();
    if a <= KNEE {
        x
    } else {
        let y = KNEE + HEAD * ((a - KNEE) / HEAD).tanh();
        // f32 tanh can round to exactly 1.0; keep strictly inside the range.
        y.min(0.999_999).copysign(x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_is_uniform_ish_and_deterministic() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        let mut sum = 0.0f64;
        for _ in 0..100_000 {
            let x = a.uniform();
            assert_eq!(x, b.uniform());
            assert!((0.0..1.0).contains(&x));
            sum += x as f64;
        }
        let mean = sum / 100_000.0;
        assert!((mean - 0.5).abs() < 0.01, "mean {mean}");
        assert_ne!(Rng::new(1).next_u32(), Rng::new(2).next_u32());
    }

    #[test]
    fn soft_clip_is_transparent_then_bounded() {
        assert_eq!(soft_clip(0.3), 0.3);
        assert_eq!(soft_clip(-0.6), -0.6);
        assert!(soft_clip(1.0e9) < 1.0);
        assert!(soft_clip(-1.0e9) > -1.0);
        assert!(soft_clip(f32::INFINITY) <= 1.0);
        assert_eq!(soft_clip(f32::NAN), 0.0);
        // Monotonic.
        let mut prev = -2.0;
        for i in -400..=400 {
            let y = soft_clip(i as f32 * 0.01);
            assert!(y >= prev);
            prev = y;
        }
    }

    #[test]
    fn resonator_noise_gain_is_normalised() {
        let mut r = Resonator::default();
        r.configure(3000.0, 0.96, 1.0, 0.0, 44_100.0);
        let mut rng = Rng::new(3);
        let (mut pin, mut pout) = (0.0f64, 0.0f64);
        for _ in 0..200_000 {
            let x = rng.noise();
            let y = r.tick(x);
            pin += (x * x) as f64;
            pout += (y * y) as f64;
        }
        let ratio = pout / pin;
        assert!((ratio - 1.0).abs() < 0.1, "power ratio {ratio}");
    }
}
