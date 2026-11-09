//! `shariki` — shake a Triki token, hear a shaker.

mod audio;
#[cfg(target_os = "macos")]
mod macos;
mod play;
mod presets;
mod status;

use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use shariki_core::MotionState;
use shariki_motion::{MotionConfig, MotionTracker};

use crate::presets::Presets;

#[derive(Parser)]
#[command(
    name = "shariki",
    version,
    about = "Digital shaker instruments played by a Triki BLE token"
)]
struct Cli {
    /// Preset file (JSON). Default: ./presets.json, then ~/.config/shariki/presets.json,
    /// then the built-in presets.
    #[arg(long, global = true, value_name = "FILE")]
    presets: Option<PathBuf>,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// List the presets and their parameters.
    List,
    /// Scan for nearby Triki tokens.
    Scan {
        /// Scan duration in seconds.
        #[arg(long, default_value_t = 5.0)]
        timeout: f64,
    },
    /// Play with the token (or a synthetic gesture, `--demo`). ↑/↓ switch presets.
    Play(PlayArgs),
    /// Render the built-in demo gesture through the full motion → sound chain to a WAV file.
    Render {
        /// Preset id (see `shariki list`).
        preset: String,
        /// Override a parameter, e.g. `-p beads=40`. Repeatable.
        #[arg(short = 'p', long = "param", value_name = "ID=VALUE", value_parser = parse_param)]
        params: Vec<(String, f32)>,
        /// Output file (default: `<preset>.wav`).
        #[arg(short, long)]
        out: Option<PathBuf>,
        /// Output sample rate in Hz.
        #[arg(long, default_value_t = 48_000)]
        sample_rate: u32,
    },
    /// Manage the preset file.
    #[command(subcommand)]
    Presets(PresetsCmd),
}

#[derive(Subcommand)]
enum PresetsCmd {
    /// Write the built-in presets to a JSON file you can edit.
    Export {
        /// Destination (default: ./presets.json).
        #[arg(short, long, default_value = presets::FILE_NAME)]
        out: PathBuf,
        /// Overwrite an existing file.
        #[arg(long)]
        force: bool,
    },
    /// Show which preset file is in use and where shariki looks for one.
    Where,
}

#[derive(Args)]
struct PlayArgs {
    /// Preset to start with (default: the first one in the file).
    preset: Option<String>,
    /// Override a parameter, e.g. `-p beads=40`. Repeatable; applied to every preset.
    #[arg(short = 'p', long = "param", value_name = "ID=VALUE", value_parser = parse_param)]
    params: Vec<(String, f32)>,
    /// IMU sample rate in Hz: 26, 52, 104, 208 or 416.
    #[arg(long, default_value_t = 416)]
    rate: u16,
    /// Connect to this peripheral id instead of the first token found by name.
    #[arg(long)]
    id: Option<String>,
    /// No token: play a built-in synthetic shake gesture.
    #[arg(long)]
    demo: bool,
    /// Audio buffer in frames (smaller = lower latency, more risk of dropouts).
    #[arg(long, default_value_t = 256)]
    buffer: u32,
}

fn parse_param(s: &str) -> Result<(String, f32), String> {
    let (id, value) = s.split_once('=').ok_or("expected ID=VALUE")?;
    let value: f32 = value
        .trim()
        .parse()
        .map_err(|e| format!("bad value `{value}`: {e}"))?;
    Ok((id.trim().to_string(), value))
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let needs_bluetooth = match &cli.cmd {
        Cmd::Scan { .. } => true,
        Cmd::Play(args) => !args.demo,
        Cmd::List | Cmd::Render { .. } | Cmd::Presets(_) => false,
    };
    #[cfg(target_os = "macos")]
    if needs_bluetooth {
        macos::ensure_self_responsible()?;
    }
    #[cfg(not(target_os = "macos"))]
    let _ = needs_bluetooth;

    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(run(cli))
}

async fn run(cli: Cli) -> Result<()> {
    let presets_arg = cli.presets.as_deref();
    match cli.cmd {
        Cmd::List => list(&Presets::resolve(presets_arg)?),
        Cmd::Scan { timeout } => scan(timeout).await,
        Cmd::Play(args) => {
            let opts = play::PlayOptions {
                preset: args.preset,
                overrides: args.params,
                rate: args.rate,
                id: args.id,
                demo: args.demo,
                buffer: args.buffer,
            };
            play::play(opts, Presets::resolve(presets_arg)?).await
        }
        Cmd::Render {
            preset,
            params,
            out,
            sample_rate,
        } => render(
            &Presets::resolve(presets_arg)?,
            &preset,
            &params,
            out,
            sample_rate,
        ),
        Cmd::Presets(PresetsCmd::Export { out, force }) => {
            presets::export(&out, force)?;
            println!(
                "wrote {} — edit it; shariki picks it up from the current directory",
                out.display()
            );
            Ok(())
        }
        Cmd::Presets(PresetsCmd::Where) => {
            let p = Presets::resolve(presets_arg)?;
            println!(
                "in use: {} ({} presets)",
                p.source(),
                p.bank.presets().len()
            );
            println!("search order: --presets FILE, ./{}", presets::FILE_NAME);
            if let Some(cfg) = presets::config_file() {
                println!("              {}", cfg.display());
            }
            println!("              built-in");
            Ok(())
        }
    }
}

fn render(
    presets: &Presets,
    id: &str,
    params: &[(String, f32)],
    out: Option<PathBuf>,
    sample_rate: u32,
) -> Result<()> {
    const RATE_HZ: f64 = 104.0;
    let (mut inst, warnings) = presets::instantiate(&presets.bank, id, sample_rate as f32, params)?;
    if let Some(w) = warnings.first() {
        anyhow::bail!("{w}");
    }
    let mut tracker = MotionTracker::new(MotionConfig::default());
    let states: Vec<MotionState> = shariki_motion::synth::demo(RATE_HZ)
        .iter()
        .map(|f| tracker.push(f))
        .collect();
    let duration = states.last().map_or(0.0, |s| s.t) as f32 + 1.0;
    // 256-frame blocks, motion delivered in bursts of 4 — roughly what BLE does.
    let samples = shariki_engine::render::render(
        inst.as_mut(),
        &states,
        sample_rate as f32,
        duration,
        256,
        4,
    );

    let path = out.unwrap_or_else(|| format!("{id}.wav").into());
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut wav = hound::WavWriter::create(&path, spec)
        .with_context(|| format!("cannot create {}", path.display()))?;
    for s in &samples {
        wav.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
    }
    wav.finalize()?;

    let st = shariki_engine::render::stats(&samples);
    let db = |x: f32| 20.0 * x.max(1e-9).log10();
    println!(
        "{}: {duration:.1} s, rms {:.3} ({:.1} dBFS), peak {:.3} ({:.1} dBFS)",
        path.display(),
        st.rms,
        db(st.rms),
        st.peak,
        db(st.peak)
    );
    Ok(())
}

fn list(presets: &Presets) -> Result<()> {
    println!("presets: {}\n", presets.source());
    for (i, model) in presets.bank.presets().iter().enumerate() {
        println!(
            "{:>2}. {:<10} {} — {}",
            i + 1,
            model.id,
            model.name,
            model.description
        );
        let (inst, _) = presets::instantiate(&presets.bank, &model.id, 48_000.0, &[])?;
        for p in inst.params() {
            println!(
                "      {:<12} {:>9.4}  [{} … {}]  {}",
                p.id, p.default, p.min, p.max, p.help
            );
        }
    }
    Ok(())
}

async fn scan(timeout: f64) -> Result<()> {
    eprintln!("scanning for {timeout} s…");
    let found = triki_ble::scan(Duration::from_secs_f64(timeout)).await?;
    if found.is_empty() {
        println!("no Triki tokens found");
    }
    for d in found {
        let rssi = d
            .rssi
            .map(|r| format!("{r} dBm"))
            .unwrap_or_else(|| "—".into());
        println!("{:<20} {:>8}  id={}", d.name, rssi, d.id);
    }
    Ok(())
}
