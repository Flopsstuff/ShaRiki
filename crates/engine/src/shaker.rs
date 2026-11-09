//! The generic PhISEM shaker voice.
//!
//! Algorithm after Perry R. Cook's PhISEM as implemented in STK's `Shakers` class
//! (The Synthesis ToolKit in C++, Perry R. Cook & Gary P. Scavone, 1995–2023, MIT-style
//! license). Per sample:
//!
//! 1. the shake energy `E` relaxes towards the motion drive with the system decay,
//! 2. a collision happens with probability `∝ beads × E`; it adds a random-amplitude burst
//!    to the noise envelope `snd` (and re-detunes the "varying" resonances),
//! 3. `snd` decays with the sound decay; the excitation is `snd × white noise`,
//! 4. the excitation runs through 1–4 two-pole resonators and the body filter,
//! 5. output gain and a soft clipper.
//!
//! # Motion → excitation mapping
//!
//! * **Continuous energy.** `MotionState::energy` is gated by a small noise floor
//!   ([`ENERGY_FLOOR`]), scaled by `sensitivity`, and becomes the *drive* target. The drive
//!   is smoothed per sample (τ = [`DRIVE_TAU`]) because motion states arrive in bursts at
//!   ~104 Hz, so the excitation glides instead of stepping. The shake energy `E` then
//!   follows `CONTINUOUS_WEIGHT × drive` as a one-pole low-pass whose time constant is the
//!   `decay` parameter (equivalent to STK's `shakeEnergy *= systemDecay` plus a steady
//!   energy injection). Sustained shaking therefore gives a continuous rattle at roughly
//!   half of full intensity.
//! * **Impulses.** `MotionState::impulse` (one state per reversal) is the accented "chk"
//!   when the beads slam into the wall: it adds `impulse × sensitivity × IMPULSE_GAIN` to
//!   `E` (a dense burst of collisions that dies away with `decay`) and an immediate
//!   [`ACCENT`] burst to the noise envelope for a crisp attack. Impulses received during a
//!   burst of updates are merged (max) and applied at the start of the next block.
//! * **Collision statistics.** Rate = `beads × 43.07/s × E` (STK: `nObjects / 1024` per
//!   sample at 44.1 kHz, at `E = 1`); each collision adds `gain(beads) × √E × U(0.5, 1)` to the
//!   envelope, `gain(beads) = ln(beads + 1) / beads` (STK's `log(n)/n` normalisation, so
//!   loudness grows only logarithmically with the bead count). Loudness therefore rises
//!   steeply with energy (`≈ E^1.5`), and gentle shaking yields fewer, softer clicks.
//! * **Silence.** Once the drive is zero and `E` falls below [`ENERGY_OFF`], collisions
//!   stop; the last clicks ring out and every state is flushed to exact zero, so the output
//!   becomes digital silence within a few hundred milliseconds.
//! * **Rainstick (tilt).** For presets with grain flow, every time `up` has rotated more
//!   than [`TILT_STEP`] from the last reference orientation, `angle / π × sensitivity`
//!   (turning fully over = 1) is added to a grain-flow level that decays with the `decay`
//!   parameter (seconds). Flow adds to `E`, so pebbles keep trickling for seconds after the
//!   stick is turned, ever sparser and softer.

use std::f32::consts::{PI, TAU};

use shariki_core::{Instrument, MotionState, ParamError, ParamSpec};

use crate::bank::{BankError, PresetBank};
use crate::dsp::{FLUSH, RING_FLUSH, Resonator, Rng, decay_to_tau, soft_clip, tau_to_coef};
use crate::model::{
    BODY_LOWPASS_HZ, Body, MAX_RES, N_PARAMS, NO_RESONANCE, P_BEADS, P_DAMPING, P_DECAY, P_GAIN,
    P_SENSITIVITY, P_SIZE, Resonance, STK_RATE, ShakerModel,
};

/// Raw motion energy below this is treated as sensor noise / holding still.
pub const ENERGY_FLOOR: f32 = 0.03;
/// Smoothing time constant of the motion drive, seconds (bridges bursty updates).
pub const DRIVE_TAU: f32 = 0.02;
/// Fraction of the drive that sustains continuous rattling (impulses provide the accents).
pub const CONTINUOUS_WEIGHT: f32 = 0.15;
/// Shake energy added per unit of impulse strength.
pub const IMPULSE_GAIN: f32 = 1.0;
/// Immediate envelope burst per unit of impulse (relative to one collision at `E = 1`).
pub const ACCENT: f32 = 1.5;
/// Below this shake energy (with no drive) collisions stop entirely.
pub const ENERGY_OFF: f32 = 1.0e-3;
/// Below this grain flow the rainstick stops trickling.
pub const FLOW_OFF: f32 = 5.0e-3;
/// Minimum change of the `up` vector (radians) that counts as turning the rainstick.
pub const TILT_STEP: f32 = 0.15;
/// Upper bound of the shake energy state.
const MAX_ENERGY: f32 = 3.0;
/// Upper bound of the rainstick grain flow.
const MAX_FLOW: f32 = 1.5;
/// Collisions per second per object at `E = 1` (STK: probability `nObjects/1024` per sample
/// at 44.1 kHz).
const COLLISIONS_PER_OBJECT: f32 = STK_RATE / 1024.0;
/// Upper bound on collisions rendered in one sample (keeps the per-sample cost bounded).
const MAX_HITS_PER_SAMPLE: u32 = 64;
/// Default RNG seed used by [`crate::create`].
pub const DEFAULT_SEED: u64 = 0x5EED_5A4A_121C;

/// A PhISEM shaker voice configured by a [`ShakerModel`] (see [`crate::PresetBank`]).
///
/// The voice owns a validated copy of its model; all allocation happens on construction.
#[derive(Debug, Clone)]
pub struct Shaker {
    model: ShakerModel,
    /// Parameter specs with this model's defaults.
    specs: [ParamSpec; N_PARAMS],
    /// The model's resonances, unused slots filled with [`NO_RESONANCE`].
    resonances: [Resonance; MAX_RES],
    n_res: usize,
    sample_rate: f32,
    values: [f32; N_PARAMS],
    seed: u64,

    // Derived coefficients (recomputed off the audio thread).
    drive_coef: f32,
    sys_coef: f32,
    flow_coef: f32,
    snd_coef: f32,
    p_per_energy: f32,
    collision_gain: f32,
    out_gain: f32,
    body_diff_gain: f32,
    body_lp_k: f32,
    res: [Resonator; MAX_RES],

    // Inputs from `update`.
    target: f32,
    pending_impulse: f32,
    pending_flow: f32,
    ref_up: [f32; 3],
    ref_valid: bool,

    // Audio state.
    drive: f32,
    energy: f32,
    flow: f32,
    snd: f32,
    body_x1: f32,
    body_lp: f32,
    rng: Rng,
}

impl Shaker {
    /// Creates the voice for a built-in preset id, or `None` if the id is unknown.
    pub fn new(id: &str, sample_rate: f32) -> Option<Self> {
        Self::with_seed(id, sample_rate, DEFAULT_SEED)
    }

    /// Like [`Shaker::new`] with an explicit RNG seed (same seed + same input ⇒ same output).
    pub fn with_seed(id: &str, sample_rate: f32, seed: u64) -> Option<Self> {
        let model = PresetBank::builtin().get(id)?;
        Self::from_model(model, sample_rate, seed).ok()
    }

    /// Creates the voice for `model` (validated first, so a constructed `Shaker` is always
    /// valid). Same model + seed + input ⇒ same output.
    pub fn from_model(model: &ShakerModel, sample_rate: f32, seed: u64) -> Result<Self, BankError> {
        model.validate()?;
        let specs = model.param_specs();
        let mut values = [0.0; N_PARAMS];
        for (slot, spec) in values.iter_mut().zip(specs.iter()) {
            *slot = spec.default;
        }
        let mut resonances = [NO_RESONANCE; MAX_RES];
        resonances[..model.resonances.len()].copy_from_slice(&model.resonances);
        let mut s = Self {
            model: model.clone(),
            specs,
            resonances,
            n_res: model.resonances.len(),
            sample_rate: 48_000.0,
            values,
            seed,
            drive_coef: 0.0,
            sys_coef: 0.0,
            flow_coef: 0.0,
            snd_coef: 0.0,
            p_per_energy: 0.0,
            collision_gain: 0.0,
            out_gain: 0.0,
            body_diff_gain: 0.0,
            body_lp_k: 1.0,
            res: [Resonator::default(); MAX_RES],
            target: 0.0,
            pending_impulse: 0.0,
            pending_flow: 0.0,
            ref_up: [0.0, 0.0, 1.0],
            ref_valid: false,
            drive: 0.0,
            energy: 0.0,
            flow: 0.0,
            snd: 0.0,
            body_x1: 0.0,
            body_lp: 0.0,
            rng: Rng::new(seed),
        };
        s.set_sample_rate(sample_rate);
        Ok(s)
    }

    /// The model this voice was built from (its `params` are the defaults, not the current
    /// values).
    pub fn model(&self) -> &ShakerModel {
        &self.model
    }

    /// Clears all audio and input state and re-seeds the RNG (parameters are kept).
    pub fn reset(&mut self) {
        self.target = 0.0;
        self.pending_impulse = 0.0;
        self.pending_flow = 0.0;
        self.ref_valid = false;
        self.drive = 0.0;
        self.energy = 0.0;
        self.flow = 0.0;
        self.snd = 0.0;
        self.body_x1 = 0.0;
        self.body_lp = 0.0;
        for r in &mut self.res {
            r.clear();
        }
        // Per-collision jitter (`retune`) leaves resonators detuned; restore their tuning.
        self.recompute();
        self.rng = Rng::new(self.seed);
    }

    /// Current internal shake energy (for diagnostics / UI meters).
    pub fn shake_energy(&self) -> f32 {
        self.energy + self.flow
    }

    fn recompute(&mut self) {
        let sr = self.sample_rate;
        let p = &self.model;
        let v = &self.values;
        let beads = v[P_BEADS];
        let damping = v[P_DAMPING];

        self.drive_coef = 1.0 - tau_to_coef(DRIVE_TAU, sr);
        match p.flow_shake_tau {
            Some(shake_tau) => {
                self.sys_coef = tau_to_coef(shake_tau, sr);
                self.flow_coef = tau_to_coef(v[P_DECAY], sr);
            }
            None => {
                self.sys_coef = tau_to_coef(v[P_DECAY], sr);
                self.flow_coef = 0.0;
            }
        }
        let click_tau = decay_to_tau(p.sound_decay, STK_RATE) / damping;
        self.snd_coef = tau_to_coef(click_tau, sr);
        self.p_per_energy = beads * COLLISIONS_PER_OBJECT / sr;
        // STK normalisation log(n)/n; √damping compensates the energy lost by shorter
        // clicks so `damping` mostly changes timbre, not level.
        self.collision_gain = (beads + 1.0).ln() / beads * damping.sqrt();

        let nyquist_guard = 0.45 * sr;
        for i in 0..MAX_RES {
            let r = self.resonances[i];
            let freq = (r.freq / v[P_SIZE]).clamp(20.0, nyquist_guard);
            let radius = r.radius.powf(STK_RATE / sr);
            let gain = if i < self.n_res { r.gain } else { 0.0 };
            self.res[i].configure(freq, radius, gain, r.vary, sr);
        }

        // STK's FIR at 44.1 kHz, re-expressed in physical frequency (see `Body`).
        self.body_diff_gain = sr / STK_RATE;
        self.body_lp_k = match p.body {
            Body::HighPass => 1.0,
            Body::BandPass => 1.0 - (-TAU * BODY_LOWPASS_HZ.min(0.49 * sr) / sr).exp(),
        };
        self.out_gain = p.level * v[P_GAIN];
    }

    #[inline]
    fn is_idle(&self) -> bool {
        self.energy == 0.0
            && self.flow == 0.0
            && self.drive == 0.0
            && self.target == 0.0
            && self.snd == 0.0
            && self.body_x1 == 0.0
            && self.body_lp == 0.0
            && self.res[..self.n_res].iter().all(Resonator::is_silent)
    }

    fn track_tilt(&mut self, up: [f32; 3]) {
        let n = (up[0] * up[0] + up[1] * up[1] + up[2] * up[2]).sqrt();
        if !n.is_finite() || n < 0.1 {
            return;
        }
        let up = [up[0] / n, up[1] / n, up[2] / n];
        if !self.ref_valid {
            self.ref_up = up;
            self.ref_valid = true;
            return;
        }
        let r = self.ref_up;
        let cos = (up[0] * r[0] + up[1] * r[1] + up[2] * r[2]).clamp(-1.0, 1.0);
        let angle = cos.acos();
        if angle > TILT_STEP {
            self.pending_flow += angle / PI * self.values[P_SENSITIVITY];
            self.ref_up = up;
        }
    }
}

#[inline]
fn finite_or_zero(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

impl Instrument for Shaker {
    fn id(&self) -> &str {
        &self.model.id
    }

    fn name(&self) -> &str {
        &self.model.name
    }

    fn set_sample_rate(&mut self, sample_rate: f32) {
        self.sample_rate = if sample_rate.is_finite() {
            sample_rate.clamp(8_000.0, 768_000.0)
        } else {
            48_000.0
        };
        self.recompute();
    }

    fn params(&self) -> &[ParamSpec] {
        &self.specs
    }

    fn param(&self, id: &str) -> Option<f32> {
        let i = self.specs.iter().position(|p| p.id == id)?;
        Some(self.values[i])
    }

    fn set_param(&mut self, id: &str, value: f32) -> Result<(), ParamError> {
        let i = self
            .specs
            .iter()
            .position(|p| p.id == id)
            .ok_or_else(|| ParamError::Unknown(id.to_string()))?;
        let spec = &self.specs[i];
        // `!(a <= b)` also rejects NaN.
        if !(value >= spec.min && value <= spec.max) {
            return Err(ParamError::OutOfRange {
                id: id.to_string(),
                value,
                min: spec.min,
                max: spec.max,
            });
        }
        self.values[i] = value;
        self.recompute();
        Ok(())
    }

    fn update(&mut self, motion: &MotionState) {
        let sens = self.values[P_SENSITIVITY];
        let e = finite_or_zero(motion.energy).clamp(0.0, 4.0);
        let gated = ((e - ENERGY_FLOOR) / (1.0 - ENERGY_FLOOR)).max(0.0);
        self.target = gated * sens;

        let imp = finite_or_zero(motion.impulse).clamp(0.0, 2.0) * sens;
        self.pending_impulse = self.pending_impulse.max(imp);

        if self.model.flow_shake_tau.is_some() {
            self.track_tilt(motion.up);
        }
    }

    fn render(&mut self, out: &mut [f32]) {
        // Apply events gathered since the previous block.
        if self.pending_impulse > 0.0 {
            let imp = self.pending_impulse;
            self.pending_impulse = 0.0;
            self.energy = (self.energy + imp * IMPULSE_GAIN).min(MAX_ENERGY);
            self.snd += ACCENT * self.collision_gain * imp.sqrt();
        }
        if self.pending_flow > 0.0 {
            self.flow = (self.flow + self.pending_flow).min(MAX_FLOW);
            self.pending_flow = 0.0;
        }

        if self.is_idle() {
            out.fill(0.0);
            return;
        }

        let n_res = self.n_res;
        let sr = self.sample_rate;
        let target = self.target * CONTINUOUS_WEIGHT;
        let has_flow = self.model.flow_shake_tau.is_some();

        for o in out.iter_mut() {
            // 1. Excitation dynamics.
            self.drive += (target - self.drive) * self.drive_coef;
            self.energy = self.energy * self.sys_coef + self.drive * (1.0 - self.sys_coef);
            if self.target == 0.0 && self.drive < ENERGY_OFF && self.energy < ENERGY_OFF {
                self.drive = 0.0;
                self.energy = 0.0;
            }
            if has_flow {
                self.flow *= self.flow_coef;
                if self.flow < FLOW_OFF {
                    self.flow = 0.0;
                }
            }
            let e = self.energy + self.flow;

            // 2. Stochastic collisions. The expected count per sample can exceed 1 with many
            //    beads at a low sample rate: take its integer part as certain collisions
            //    and the fraction as a probability, so the rate never saturates (which
            //    would make the level depend on the sample rate).
            let expected = if e > 0.0 {
                self.p_per_energy * e.min(2.0)
            } else {
                0.0
            };
            let mut hits = (expected.floor() as u32).min(MAX_HITS_PER_SAMPLE);
            if e > 0.0 && self.rng.uniform() < expected.fract() {
                hits += 1;
            }
            if hits > 0 {
                let mut amp = 0.0;
                for _ in 0..hits {
                    amp += 0.5 + 0.5 * self.rng.uniform();
                }
                self.snd += self.collision_gain * e.sqrt() * amp;
                for r in &mut self.res[..n_res] {
                    if r.vary > 0.0 {
                        let f = r.freq * (1.0 + r.vary * self.rng.noise());
                        r.retune(f, sr);
                    }
                }
            }

            // 3. Decaying noise envelope.
            self.snd *= self.snd_coef;
            if self.snd < FLUSH {
                self.snd = 0.0;
            }
            let x = if self.snd > 0.0 {
                self.snd * self.rng.noise()
            } else {
                0.0
            };

            // 4. Resonators + body FIR.
            let mut y = 0.0;
            for r in &mut self.res[..n_res] {
                y += r.tick(x);
            }
            let diff = (y - self.body_x1) * self.body_diff_gain;
            self.body_x1 = y;
            self.body_lp += (diff - self.body_lp) * self.body_lp_k;
            if y == 0.0 && self.body_lp.abs() < RING_FLUSH {
                self.body_lp = 0.0;
            }
            let body = self.body_lp;

            // 5. Output.
            *o = soft_clip(body * self.out_gain);
        }

        // Safety net: if anything ever went non-finite, start clean.
        if !(self.snd.is_finite() && self.energy.is_finite() && self.body_lp.is_finite()) {
            self.reset();
        }
    }
}
