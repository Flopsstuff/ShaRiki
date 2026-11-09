//! One PhISEM model description ([`ShakerModel`]), the parameter specs shared by every
//! instrument, and model validation.
//!
//! The six parameter ids, names, help texts and ranges are code constants; a model only
//! chooses the *default* value of each (see [`ShakerModel::params`]).
//!
//! STK constants refer to Perry R. Cook's PhISEM implementation in the Synthesis ToolKit
//! (`src/Shakers.cpp`, The Synthesis ToolKit in C++, Perry R. Cook & Gary P. Scavone,
//! 1995–2023, MIT-style license). STK specifies its decays and pole radii per sample at
//! 44.1 kHz; [`STK_RATE`] is used to convert them to time constants and bandwidths so the
//! engine sounds the same at any sample rate.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize, Serializer};
use shariki_core::ParamSpec;

use crate::bank::BankError;

/// Sample rate STK's per-sample constants (`sound_decay`, resonance `radius`) refer to.
pub const STK_RATE: f32 = 44_100.0;

/// Maximum number of resonances per model (fixed-size arrays, no allocation).
pub const MAX_RESONANCES: usize = 4;
pub(crate) const MAX_RES: usize = MAX_RESONANCES;

/// Highest allowed resonance frequency, Hz.
pub const MAX_RESONANCE_FREQ: f32 = 20_000.0;

// Parameter indices into `Shaker::values`, in the order of `PARAM_IDS`.
pub(crate) const P_BEADS: usize = 0;
pub(crate) const P_DECAY: usize = 1;
pub(crate) const P_DAMPING: usize = 2;
pub(crate) const P_SIZE: usize = 3;
pub(crate) const P_SENSITIVITY: usize = 4;
pub(crate) const P_GAIN: usize = 5;
/// Number of parameters every instrument exposes.
pub const N_PARAMS: usize = 6;

/// The parameter ids every instrument exposes, in [`shariki_core::Instrument::params`] order.
pub const PARAM_IDS: [&str; N_PARAMS] =
    ["beads", "decay", "damping", "size", "sensitivity", "gain"];

/// Corner of [`Body::BandPass`]'s low-pass, Hz (STK: `sr/4` at 44.1 kHz).
pub(crate) const BODY_LOWPASS_HZ: f32 = STK_RATE / 4.0;

/// STK `systemDecay_` (per sample at 44.1 kHz) → seconds.
/// τ = 1 / (44100 · −ln d) ≈ 1 / (44100 · (1 − d)) for d close to 1.
const fn stk_tau(d: f32) -> f32 {
    1.0 / (STK_RATE * (1.0 - d))
}

/// Default `decay` of a shaker (STK Maraca `systemDecay` 0.999 ≈ 22.7 ms).
pub const DEFAULT_SHAKE_DECAY: f32 = stk_tau(0.999);
/// Default `decay` of a grain-flow (tilt-driven) model, seconds.
pub const DEFAULT_FLOW_DECAY: f32 = 2.0;

/// Parameter specs with the global defaults, `decay` as the shake-energy time constant.
const SHAKE_SPECS: [ParamSpec; N_PARAMS] = [
    ParamSpec {
        id: "beads",
        name: "Beads",
        min: 1.0,
        max: 8192.0,
        default: 400.0,
        help: "Number of colliding objects: more beads = denser, louder, smoother rattle",
    },
    ParamSpec {
        id: "decay",
        name: "Decay",
        min: 0.005,
        max: 1.0,
        default: DEFAULT_SHAKE_DECAY,
        help: "Shake-energy time constant in seconds: how long the rattle sustains after a hit",
    },
    ParamSpec {
        id: "damping",
        name: "Damping",
        min: 0.25,
        max: 4.0,
        default: 1.0,
        help: "How fast each click dies (1 = model default, higher = shorter, drier clicks)",
    },
    ParamSpec {
        id: "size",
        name: "Size",
        min: 0.5,
        max: 2.0,
        default: 1.0,
        help: "Instrument size: resonances scale by 1/size (2 = an octave lower)",
    },
    ParamSpec {
        id: "sensitivity",
        name: "Sensitivity",
        min: 0.1,
        max: 4.0,
        default: 1.0,
        help: "Scales how strongly motion drives the instrument",
    },
    ParamSpec {
        id: "gain",
        name: "Gain",
        min: 0.0,
        max: 2.0,
        default: 1.0,
        help: "Output level (linear)",
    },
];

/// `decay` of a grain-flow model (`flow_shake_tau` set).
const FLOW_DECAY_SPEC: ParamSpec = ParamSpec {
    id: "decay",
    name: "Decay",
    min: 0.3,
    max: 8.0,
    default: DEFAULT_FLOW_DECAY,
    help: "Grain-flow time constant in seconds: how long pebbles keep falling after turning",
};

/// The parameter specs with the global defaults (before a model's own defaults are applied).
/// `flow` selects the grain-flow meaning and range of `decay`.
pub fn base_param_specs(flow: bool) -> [ParamSpec; N_PARAMS] {
    let mut specs = SHAKE_SPECS;
    if flow {
        specs[P_DECAY] = FLOW_DECAY_SPEC;
    }
    specs
}

/// One bead/shell resonance (two-pole filter), STK style.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Resonance {
    /// Centre frequency in Hz at `size = 1`, in `(0, 20000]`.
    pub freq: f32,
    /// Pole radius per sample at 44.1 kHz ([`STK_RATE`]), in `[0, 1)`: closer to 1 = narrower,
    /// longer ringing.
    pub radius: f32,
    /// Relative mix gain (≥ 0; defaults to 1).
    #[serde(default = "one")]
    pub gain: f32,
    /// Random relative detune applied on every collision (STK `varyFactor_`), in `[0, 1)`;
    /// 0 (the default) = fixed resonance.
    #[serde(default)]
    pub vary: f32,
}

fn one() -> f32 {
    1.0
}

/// Placeholder for unused resonator slots.
pub(crate) const NO_RESONANCE: Resonance = Resonance {
    freq: 1000.0,
    radius: 0.0,
    gain: 0.0,
    vary: 0.0,
};

/// Output "body" filter. STK's output equalisers are 3-tap FIRs whose response depends on
/// the sample rate; we keep their shape but define them in physical frequency so a model
/// sounds the same at any rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Body {
    /// `"highpass"` — STK `(1, −1, 0)`: first difference, +6 dB/octave tilt (the "gourd"
    /// high-pass). Implemented as a difference scaled by `sr / 44100`.
    HighPass,
    /// `"bandpass"` — STK `(1, 0, −1)`: zeros at DC and Nyquist, i.e. the +6 dB/octave
    /// difference followed by a `(1 + z⁻¹)` roll-off that is −3 dB at 11.025 kHz.
    /// Implemented as the scaled difference followed by a one-pole low-pass at 11.025 kHz.
    BandPass,
}

/// A PhISEM model preset: everything that distinguishes one instrument from another.
///
/// This is the JSON schema of one entry of a preset bank (see [`crate::PresetBank`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShakerModel {
    /// Stable machine id, e.g. `"maracas"` (non-empty, no whitespace, unique in a bank).
    pub id: String,
    /// Human-readable name.
    pub name: String,
    /// One-line description of the sound.
    #[serde(default)]
    pub description: String,
    /// Free-text provenance notes (which STK model the constants come from, etc.).
    #[serde(default)]
    pub source: Option<String>,
    /// Default values of the parameters, keyed by id (see [`PARAM_IDS`]). Missing keys use
    /// the global default; each value must lie in that parameter's range.
    #[serde(default, serialize_with = "serialize_params")]
    pub params: BTreeMap<String, f32>,
    /// Per-sample decay of each collision's noise burst at 44.1 kHz (STK `soundDecay_`), in
    /// `(0, 1)`.
    pub sound_decay: f32,
    /// 1 to 4 bead/shell resonances.
    pub resonances: Vec<Resonance>,
    /// Output body filter.
    pub body: Body,
    /// Output calibration (> 0): chosen so that with default parameters the 99.9th
    /// percentile of |output| during the default [`crate::render::ShakeGesture`] (rainstick:
    /// its turn-over gesture) is ≈ 0.4 (ganza 0.35, rainstick 0.3); peaks then land around
    /// 0.75–0.9.
    pub level: f32,
    /// Tilt-driven grain flow (rainstick). When set, this is the fixed shake-energy time
    /// constant in seconds (> 0), turning the instrument over makes grains trickle, and the
    /// `decay` parameter becomes the grain-flow decay time (range 0.3–8 s).
    #[serde(default)]
    pub flow_shake_tau: Option<f32>,
}

/// Writes `params` in [`PARAM_IDS`] order (then any unknown keys), not alphabetically.
fn serialize_params<S: Serializer>(
    params: &BTreeMap<String, f32>,
    ser: S,
) -> Result<S::Ok, S::Error> {
    use serde::ser::SerializeMap;
    let mut map = ser.serialize_map(Some(params.len()))?;
    for id in PARAM_IDS {
        if let Some(v) = params.get(id) {
            map.serialize_entry(id, v)?;
        }
    }
    for (k, v) in params {
        if !PARAM_IDS.contains(&k.as_str()) {
            map.serialize_entry(k, v)?;
        }
    }
    map.end()
}

impl ShakerModel {
    /// The parameter specs of this model: shared ids/names/ranges with this model's
    /// defaults filled in. Unknown keys in [`ShakerModel::params`] are ignored here (they
    /// are rejected by [`ShakerModel::validate`]).
    pub fn param_specs(&self) -> [ParamSpec; N_PARAMS] {
        let mut specs = base_param_specs(self.flow_shake_tau.is_some());
        for spec in &mut specs {
            if let Some(&v) = self.params.get(spec.id) {
                spec.default = v;
            }
        }
        specs
    }

    /// Checks every field; the error names the preset id and the offending field.
    pub fn validate(&self) -> Result<(), BankError> {
        self.check().map_err(|(field, message)| BankError::Invalid {
            path: None,
            preset: self.id.clone(),
            field,
            message,
        })
    }

    /// Returns `(field, message)` for the first invalid field.
    pub(crate) fn check(&self) -> Result<(), (String, String)> {
        let err = |field: &str, message: String| Err((field.to_string(), message));

        if self.id.is_empty() {
            return err("id", "must not be empty".into());
        }
        if self.id.chars().any(char::is_whitespace) {
            return err(
                "id",
                "must not contain whitespace (use e.g. `my-shaker`)".into(),
            );
        }
        if self.name.trim().is_empty() {
            return err("name", "must not be empty".into());
        }

        // First: it decides the meaning and range of the `decay` parameter.
        if let Some(tau) = self.flow_shake_tau {
            finite("flow_shake_tau", tau)?;
            if tau <= 0.0 {
                return err(
                    "flow_shake_tau",
                    format!("{tau} must be > 0 seconds (or null for no grain flow)"),
                );
            }
        }

        let flow = self.flow_shake_tau.is_some();
        let base = base_param_specs(flow);
        for (key, &value) in &self.params {
            let Some(spec) = base.iter().find(|s| s.id == key) else {
                return err(
                    &format!("params.{key}"),
                    format!(
                        "unknown parameter `{key}` (known parameters: {})",
                        PARAM_IDS.join(", ")
                    ),
                );
            };
            finite(&format!("params.{key}"), value)?;
            if !(spec.min..=spec.max).contains(&value) {
                let note = if spec.id == "decay" {
                    if flow {
                        " (with `flow_shake_tau` set, decay is the grain-flow time in seconds)"
                    } else {
                        " (without `flow_shake_tau`, decay is the shake-energy time in seconds)"
                    }
                } else {
                    ""
                };
                return err(
                    &format!("params.{key}"),
                    format!(
                        "{value} is outside the allowed range [{}, {}]{note}",
                        spec.min, spec.max
                    ),
                );
            }
        }

        finite("sound_decay", self.sound_decay)?;
        if !(self.sound_decay > 0.0 && self.sound_decay < 1.0) {
            return err(
                "sound_decay",
                format!(
                    "{} must be in (0, 1): the per-sample decay of each click at 44.1 kHz (typically 0.9–0.99)",
                    self.sound_decay
                ),
            );
        }

        let n = self.resonances.len();
        if n == 0 || n > MAX_RES {
            return err(
                "resonances",
                format!("has {n} entries, needs 1 to {MAX_RES}"),
            );
        }
        for (i, r) in self.resonances.iter().enumerate() {
            let f = |name: &str| format!("resonances[{i}].{name}");
            finite(&f("freq"), r.freq)?;
            if !(r.freq > 0.0 && r.freq <= MAX_RESONANCE_FREQ) {
                return err(
                    &f("freq"),
                    format!("{} Hz must be in (0, {MAX_RESONANCE_FREQ}]", r.freq),
                );
            }
            finite(&f("radius"), r.radius)?;
            if !(r.radius >= 0.0 && r.radius < 1.0) {
                return err(
                    &f("radius"),
                    format!(
                        "{} must be in [0, 1) (pole radius at 44.1 kHz; 1 or more would ring forever)",
                        r.radius
                    ),
                );
            }
            finite(&f("gain"), r.gain)?;
            if r.gain < 0.0 {
                return err(&f("gain"), format!("{} must be ≥ 0", r.gain));
            }
            finite(&f("vary"), r.vary)?;
            if !(r.vary >= 0.0 && r.vary < 1.0) {
                return err(
                    &f("vary"),
                    format!("{} must be in [0, 1) (relative random detune)", r.vary),
                );
            }
        }

        finite("level", self.level)?;
        if self.level <= 0.0 {
            return err("level", format!("{} must be > 0", self.level));
        }

        Ok(())
    }
}

fn finite(field: &str, value: f32) -> Result<(), (String, String)> {
    if value.is_finite() {
        Ok(())
    } else {
        Err((
            field.to_string(),
            format!("must be a finite number, got {value}"),
        ))
    }
}
