//! Preset banks: a validated, ordered list of [`ShakerModel`]s loaded from JSON.
//!
//! The built-in bank is `crates/engine/presets.json`, embedded into the crate. Its format:
//!
//! ```json
//! {
//!   "presets": [
//!     {
//!       "id": "maracas",
//!       "name": "Maracas",
//!       "description": "…",
//!       "source": "STK Maraca: …",
//!       "params": { "beads": 400, "decay": 0.022676028 },
//!       "sound_decay": 0.95,
//!       "resonances": [ { "freq": 3200, "radius": 0.96, "gain": 1.0, "vary": 0.0 } ],
//!       "body": "highpass",
//!       "level": 5.5,
//!       "flow_shake_tau": null
//!     }
//!   ]
//! }
//! ```
//!
//! See [`ShakerModel`] for the meaning and valid range of every field. The order of the
//! presets is the display / cycling order.

use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};
use shariki_core::Instrument;

use crate::model::ShakerModel;
use crate::shaker::{DEFAULT_SEED, Shaker};

const BUILTIN_JSON: &str = include_str!("../presets.json");

/// The text of the built-in preset bank (`presets.json` embedded in the crate), e.g. as a
/// starting point for a user's own bank.
pub fn builtin_json() -> &'static str {
    BUILTIN_JSON
}

/// Error loading or validating a preset bank / model. The message names the file (when
/// known), the preset and the field.
#[derive(Debug)]
#[non_exhaustive]
pub enum BankError {
    /// The file could not be read.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// Malformed JSON or wrong structure (missing / unknown field, wrong type, unknown
    /// `body`, ...). `line` and `column` are 1-based.
    Json {
        path: Option<PathBuf>,
        line: usize,
        column: usize,
        message: String,
    },
    /// The bank has no presets.
    Empty { path: Option<PathBuf> },
    /// Two presets share an id. `first` and `second` are 1-based positions in the bank.
    DuplicateId {
        path: Option<PathBuf>,
        id: String,
        first: usize,
        second: usize,
    },
    /// A field of a preset has an invalid value.
    Invalid {
        path: Option<PathBuf>,
        /// The preset id (or `#n`, its 1-based position, when the id itself is empty).
        preset: String,
        /// The offending field, e.g. `resonances[1].radius` or `params.decay`.
        field: String,
        message: String,
    },
}

impl BankError {
    fn with_path(mut self, file: &Path) -> Self {
        match &mut self {
            BankError::Io { .. } => {}
            BankError::Json { path, .. }
            | BankError::Empty { path }
            | BankError::DuplicateId { path, .. }
            | BankError::Invalid { path, .. } => *path = Some(file.to_path_buf()),
        }
        self
    }

    fn from_json_error(e: &serde_json::Error) -> Self {
        let (line, column) = (e.line(), e.column());
        let full = e.to_string();
        let suffix = format!(" at line {line} column {column}");
        let message = full.strip_suffix(&suffix).unwrap_or(&full).to_string();
        BankError::Json {
            path: None,
            line,
            column,
            message,
        }
    }
}

impl fmt::Display for BankError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let prefix = |f: &mut fmt::Formatter<'_>, path: &Option<PathBuf>| match path {
            Some(p) => write!(f, "{}: ", p.display()),
            None => Ok(()),
        };
        match self {
            BankError::Io { path, source } => {
                write!(f, "cannot read preset bank {}: {source}", path.display())
            }
            BankError::Json {
                path,
                line,
                column,
                message,
            } => match path {
                Some(p) => write!(f, "{}:{line}:{column}: {message}", p.display()),
                None => write!(f, "presets JSON, line {line}, column {column}: {message}"),
            },
            BankError::Empty { path } => {
                prefix(f, path)?;
                write!(f, "the preset bank has no presets (`presets` is empty)")
            }
            BankError::DuplicateId {
                path,
                id,
                first,
                second,
            } => {
                prefix(f, path)?;
                write!(
                    f,
                    "duplicate preset id `{id}` (presets #{first} and #{second}); ids must be unique"
                )
            }
            BankError::Invalid {
                path,
                preset,
                field,
                message,
            } => {
                prefix(f, path)?;
                write!(f, "preset `{preset}`, field `{field}`: {message}")
            }
        }
    }
}

impl std::error::Error for BankError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            BankError::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// An ordered, validated set of instrument presets.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PresetBank {
    presets: Vec<ShakerModel>,
}

impl PresetBank {
    /// The built-in bank (parsed once from the embedded `presets.json`).
    ///
    /// # Panics
    /// Only if the embedded file is invalid, which the test suite rules out.
    pub fn builtin() -> &'static PresetBank {
        static BANK: OnceLock<PresetBank> = OnceLock::new();
        BANK.get_or_init(|| {
            PresetBank::from_json(BUILTIN_JSON)
                .unwrap_or_else(|e| panic!("embedded presets.json is invalid: {e}"))
        })
    }

    /// Builds a bank from models, validating each one and the bank as a whole.
    pub fn new(presets: Vec<ShakerModel>) -> Result<PresetBank, BankError> {
        let bank = PresetBank { presets };
        bank.validate()?;
        Ok(bank)
    }

    /// Parses and validates a bank from JSON text.
    pub fn from_json(json: &str) -> Result<PresetBank, BankError> {
        let bank: PresetBank =
            serde_json::from_str(json).map_err(|e| BankError::from_json_error(&e))?;
        bank.validate()?;
        Ok(bank)
    }

    /// Reads, parses and validates a bank file; errors name `path`.
    pub fn load(path: &Path) -> Result<PresetBank, BankError> {
        let text = std::fs::read_to_string(path).map_err(|source| BankError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        PresetBank::from_json(&text).map_err(|e| e.with_path(path))
    }

    /// Pretty-printed JSON that round-trips through [`PresetBank::from_json`].
    pub fn to_json(&self) -> String {
        let mut s = serde_json::to_string_pretty(self).expect("a preset bank always serialises");
        s.push('\n');
        s
    }

    /// All presets, in display order.
    pub fn presets(&self) -> &[ShakerModel] {
        &self.presets
    }

    /// The preset with `id`, if any.
    pub fn get(&self, id: &str) -> Option<&ShakerModel> {
        self.presets.iter().find(|m| m.id == id)
    }

    /// Creates the instrument for preset `id` at `sample_rate` with the default RNG seed,
    /// or `None` if the id is unknown.
    pub fn create(&self, id: &str, sample_rate: f32) -> Option<Box<dyn Instrument>> {
        let model = self.get(id)?;
        Shaker::from_model(model, sample_rate, DEFAULT_SEED)
            .ok()
            .map(|s| Box::new(s) as Box<dyn Instrument>)
    }

    fn validate(&self) -> Result<(), BankError> {
        if self.presets.is_empty() {
            return Err(BankError::Empty { path: None });
        }
        for (i, m) in self.presets.iter().enumerate() {
            m.check().map_err(|(field, message)| BankError::Invalid {
                path: None,
                preset: if m.id.is_empty() {
                    format!("#{}", i + 1)
                } else {
                    m.id.clone()
                },
                field,
                message,
            })?;
            if let Some(first) = self.presets[..i].iter().position(|o| o.id == m.id) {
                return Err(BankError::DuplicateId {
                    path: None,
                    id: m.id.clone(),
                    first: first + 1,
                    second: i + 1,
                });
            }
        }
        Ok(())
    }
}
