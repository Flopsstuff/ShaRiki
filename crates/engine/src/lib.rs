//! Sound engines for shariki: motion-driven shakers.
//!
//! Every instrument is a [`Shaker`] — Perry R. Cook's **PhISEM** (Physically Informed
//! Stochastic Event Modeling) model, after the `Shakers` class of the Synthesis ToolKit
//! (STK, <https://github.com/thestk/stk>, © Perry R. Cook & Gary P. Scavone, MIT-style
//! license) — configured by a model preset. A shake-energy state decays every sample;
//! with probability proportional to the number of beads × energy a collision adds a
//! random-amplitude burst to a decaying noise envelope; that noise is coloured by two-pole
//! bead/shell resonators and a body filter, then soft-clipped.
//!
//! How [`MotionState`](shariki_core::MotionState) drives the model is documented in the
//! [`shaker`] module. The instrument presets are data: a [`PresetBank`] of [`ShakerModel`]s
//! parsed from JSON. The built-in bank is `crates/engine/presets.json` (embedded, see
//! [`builtin_json`]; each entry's `source` notes where its constants come from); a user bank
//! can be loaded with [`PresetBank::load`].
//!
//! ```
//! use shariki_core::{Instrument, MotionState};
//!
//! let mut inst = shariki_engine::create("maracas", 48_000.0).unwrap();
//! inst.set_param("beads", 40.0).unwrap();
//! inst.update(&MotionState { energy: 1.0, impulse: 0.8, ..Default::default() });
//! let mut block = [0.0f32; 256];
//! inst.render(&mut block);
//! assert!(block.iter().all(|s| s.abs() <= 1.0));
//! ```
//!
//! Every instrument exposes the same parameter ids: `beads`, `decay`, `damping`, `size`,
//! `sensitivity`, `gain` (see [`Instrument::params`](shariki_core::Instrument::params)
//! for ranges and defaults).

mod bank;
mod dsp;
mod model;
pub mod render;
pub mod shaker;

use std::sync::OnceLock;

pub use bank::{BankError, PresetBank, builtin_json};
pub use model::{
    Body, DEFAULT_FLOW_DECAY, DEFAULT_SHAKE_DECAY, MAX_RESONANCE_FREQ, MAX_RESONANCES, N_PARAMS,
    PARAM_IDS, Resonance, STK_RATE, ShakerModel, base_param_specs,
};
pub use shaker::Shaker;

/// Catalog entry describing one built-in instrument.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InstrumentInfo {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
}

/// All built-in instruments ([`PresetBank::builtin`]), in display order.
pub fn catalog() -> &'static [InstrumentInfo] {
    static CATALOG: OnceLock<Vec<InstrumentInfo>> = OnceLock::new();
    CATALOG.get_or_init(|| {
        PresetBank::builtin()
            .presets()
            .iter()
            .map(|m| InstrumentInfo {
                id: &m.id,
                name: &m.name,
                description: &m.description,
            })
            .collect()
    })
}

/// Creates the built-in instrument `id` at `sample_rate`, or `None` if unknown.
///
/// Uses a fixed RNG seed, so two instruments fed the same input produce identical output;
/// use [`Shaker::with_seed`] / [`Shaker::from_model`] for a different seed.
pub fn create(id: &str, sample_rate: f32) -> Option<Box<dyn shariki_core::Instrument>> {
    PresetBank::builtin().create(id, sample_rate)
}
