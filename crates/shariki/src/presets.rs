//! Where the preset bank comes from: `--presets FILE`, else `./presets.json`, else
//! `~/.config/shariki/presets.json`, else the bank built into the binary.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use shariki_core::Instrument;
use shariki_engine::PresetBank;

/// File name looked up in the current directory and the config directory.
pub const FILE_NAME: &str = "presets.json";

pub struct Presets {
    pub bank: PresetBank,
    /// The file the bank was read from; `None` for the built-in bank.
    pub path: Option<PathBuf>,
}

impl Presets {
    pub fn resolve(explicit: Option<&Path>) -> Result<Self> {
        if let Some(path) = explicit {
            return Self::from_file(path);
        }
        for candidate in [Some(PathBuf::from(FILE_NAME)), config_file()]
            .into_iter()
            .flatten()
        {
            if candidate.is_file() {
                return Self::from_file(&candidate);
            }
        }
        Ok(Self {
            bank: builtin()?,
            path: None,
        })
    }

    fn from_file(path: &Path) -> Result<Self> {
        let bank = PresetBank::load(path)
            .with_context(|| format!("cannot load presets from {}", path.display()))?;
        Ok(Self {
            bank,
            path: Some(path.to_path_buf()),
        })
    }

    /// Re-reads the file (the built-in bank cannot change, so it is just re-parsed).
    pub fn reload(&self) -> Result<PresetBank> {
        match &self.path {
            Some(path) => {
                PresetBank::load(path).with_context(|| format!("cannot reload {}", path.display()))
            }
            None => builtin(),
        }
    }

    /// Human-readable origin, e.g. `presets.json` or `built-in`.
    pub fn source(&self) -> String {
        match &self.path {
            Some(p) => p.display().to_string(),
            None => "built-in".into(),
        }
    }
}

fn builtin() -> Result<PresetBank> {
    PresetBank::from_json(shariki_engine::builtin_json())
        .map_err(|e| anyhow!("built-in presets are invalid: {e}"))
}

/// `$XDG_CONFIG_HOME/shariki/presets.json` or `~/.config/shariki/presets.json`.
pub fn config_file() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("shariki").join(FILE_NAME))
}

/// Writes the built-in bank to `path` so it can be edited.
pub fn export(path: &Path, force: bool) -> Result<()> {
    if path.exists() && !force {
        bail!(
            "{} already exists (use --force to overwrite)",
            path.display()
        );
    }
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).with_context(|| format!("cannot create {}", dir.display()))?;
    }
    std::fs::write(path, shariki_engine::builtin_json())
        .with_context(|| format!("cannot write {}", path.display()))
}

/// Creates preset `id` and applies `-p` overrides. Overrides that don't fit this preset
/// (unknown id, out of range) are returned as warnings rather than failing, so that
/// switching presets with overrides in effect keeps working.
pub fn instantiate(
    bank: &PresetBank,
    id: &str,
    sample_rate: f32,
    overrides: &[(String, f32)],
) -> Result<(Box<dyn Instrument>, Vec<String>)> {
    let mut inst = bank
        .create(id, sample_rate)
        .ok_or_else(|| anyhow!("unknown preset `{id}` (see `shariki list`)"))?;
    let mut warnings = Vec::new();
    for (pid, value) in overrides {
        if let Err(e) = inst.set_param(pid, *value) {
            warnings.push(format!("{id}: {e}"));
        }
    }
    Ok((inst, warnings))
}
