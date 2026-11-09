//! `shariki play`: motion source → tracker → audio, with live preset switching.
//!
//! Keys (when stdin is a terminal): ↑/↓ previous/next preset, `r` reload the preset file,
//! `q` / Esc / Ctrl-C quit.

use std::io::IsTerminal;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use crossterm::event::{Event, EventStream, KeyCode, KeyEventKind, KeyModifiers};
use futures::StreamExt;
use rtrb::{Producer, RingBuffer};
use shariki_core::{ImuFrame, MotionState};
use shariki_engine::PresetBank;
use shariki_motion::{MotionConfig, MotionTracker};
use tokio::sync::{mpsc, oneshot};

use crate::audio::{self, AudioOut};
use crate::presets::{self, Presets};
use crate::status::{self, Status, say};

pub struct PlayOptions {
    pub preset: Option<String>,
    pub overrides: Vec<(String, f32)>,
    pub rate: u16,
    pub id: Option<String>,
    pub demo: bool,
    pub buffer: u32,
}

/// What the motion-source task sends to the control loop.
enum SourceEvent {
    Frame(ImuFrame),
    Message(String),
}

enum Action {
    Step(isize),
    Reload,
    Quit,
}

pub async fn play(opts: PlayOptions, presets: Presets) -> Result<()> {
    let mut bank = presets.reload()?;
    if bank.presets().is_empty() {
        bail!("{} contains no presets", presets.source());
    }
    let mut index = match &opts.preset {
        Some(id) => position(&bank, id)
            .ok_or_else(|| anyhow!("unknown preset `{id}` (see `shariki list`)"))?,
        None => 0,
    };

    let (motion_tx, motion_rx) = RingBuffer::<MotionState>::new(1024);
    let mut warnings = Vec::new();
    let first_id = bank.presets()[index].id.clone();
    let mut out = audio::start(
        |sr| {
            let (inst, w) = presets::instantiate(&bank, &first_id, sr, &opts.overrides)?;
            warnings = w;
            Ok(inst)
        },
        motion_rx,
        Some(opts.buffer),
    )?;

    // Raw mode from here on; the guard restores the terminal on every exit path.
    let keyboard = Keyboard::enable();
    let status = Arc::new(Status::default());
    status.set_preset(label(&bank, index));
    if keyboard.is_some() {
        status.set_keys("↑↓ preset · r reload · q quit");
    }
    let buffer = out
        .buffer
        .map(|b| format!("{b} frames"))
        .unwrap_or_else(|| "default".into());
    say(format_args!(
        "audio: {} @ {} Hz, buffer {buffer} · presets: {} ({})",
        out.device_name,
        out.sample_rate,
        presets.source(),
        bank.presets().len()
    ));
    for w in warnings {
        say(format_args!("warning: {w}"));
    }
    let meter = status::spawn_meter(status.clone(), out.peak.clone());

    let (event_tx, mut events) = mpsc::channel::<SourceEvent>(4096);
    let (stop_tx, stop_rx) = oneshot::channel::<()>();
    let source = if opts.demo {
        tokio::spawn(demo_source(opts.rate, event_tx, stop_rx))
    } else {
        tokio::spawn(token_source(opts.id.clone(), opts.rate, event_tx, stop_rx))
    };

    let mut keys = keyboard.as_ref().map(|_| EventStream::new());
    let mut motion = Motion::new(motion_tx);
    loop {
        let action = tokio::select! {
            ev = events.recv() => match ev {
                Some(SourceEvent::Frame(f)) => { motion.feed(&f, &status); None }
                Some(SourceEvent::Message(m)) => { say(m); None }
                None => break, // the source finished (token gone / error)
            },
            key = next_key(&mut keys) => key,
            _ = tokio::signal::ctrl_c() => Some(Action::Quit),
        };
        match action {
            None => {}
            Some(Action::Quit) => break,
            Some(Action::Step(delta)) => {
                let n = bank.presets().len() as isize;
                index = (index as isize + delta).rem_euclid(n) as usize;
                switch(&mut out, &bank, index, &opts.overrides, &status);
            }
            Some(Action::Reload) => match presets.reload() {
                Ok(new_bank) if new_bank.presets().is_empty() => {
                    say(format_args!(
                        "{} contains no presets; keeping the old ones",
                        presets.source()
                    ));
                }
                Ok(new_bank) => {
                    let current = bank.presets()[index].id.clone();
                    bank = new_bank;
                    index =
                        position(&bank, &current).unwrap_or(index.min(bank.presets().len() - 1));
                    say(format_args!(
                        "reloaded {} presets from {}",
                        bank.presets().len(),
                        presets.source()
                    ));
                    switch(&mut out, &bank, index, &opts.overrides, &status);
                }
                Err(e) => say(format_args!("{e:#}")),
            },
        }
    }

    let _ = stop_tx.send(());
    let outcome = source
        .await
        .unwrap_or_else(|e| Err(anyhow!("motion source crashed: {e}")));
    meter.abort();
    drop(keyboard);
    eprintln!();
    // Let the instrument ring out before the stream is dropped.
    tokio::time::sleep(Duration::from_millis(300)).await;
    drop(out);
    outcome
}

fn position(bank: &PresetBank, id: &str) -> Option<usize> {
    bank.presets().iter().position(|p| p.id == id)
}

fn label(bank: &PresetBank, index: usize) -> String {
    format!(
        "{}/{} {}",
        index + 1,
        bank.presets().len(),
        bank.presets()[index].name
    )
}

fn switch(
    out: &mut AudioOut,
    bank: &PresetBank,
    index: usize,
    overrides: &[(String, f32)],
    status: &Status,
) {
    let id = &bank.presets()[index].id;
    match presets::instantiate(bank, id, out.sample_rate as f32, overrides) {
        Ok((inst, warnings)) => {
            for w in warnings {
                say(format_args!("warning: {w}"));
            }
            match out.swap(inst) {
                Ok(()) => status.set_preset(label(bank, index)),
                Err(e) => say(format_args!("{e:#}")),
            }
        }
        Err(e) => say(format_args!("{e:#}")),
    }
}

/// Tracker + queue to the audio thread.
struct Motion {
    tracker: MotionTracker,
    tx: Producer<MotionState>,
}

impl Motion {
    fn new(tx: Producer<MotionState>) -> Self {
        Self {
            tracker: MotionTracker::new(MotionConfig::default()),
            tx,
        }
    }

    fn feed(&mut self, frame: &ImuFrame, status: &Status) {
        let m = self.tracker.push(frame);
        // A full queue means the audio thread is stalled; dropping motion is the right call.
        let _ = self.tx.push(m);
        status.record(&m);
    }
}

// ---- keyboard ---------------------------------------------------------------------------

/// Raw-mode guard; `None` when stdin is not an interactive terminal.
struct Keyboard;

impl Keyboard {
    fn enable() -> Option<Self> {
        if !std::io::stdin().is_terminal() {
            return None;
        }
        crossterm::terminal::enable_raw_mode()
            .ok()
            .map(|()| Keyboard)
    }
}

impl Drop for Keyboard {
    fn drop(&mut self) {
        let _ = crossterm::terminal::disable_raw_mode();
    }
}

async fn next_key(keys: &mut Option<EventStream>) -> Option<Action> {
    let Some(stream) = keys.as_mut() else {
        return futures::future::pending().await;
    };
    loop {
        let event = match stream.next().await {
            Some(Ok(e)) => e,
            // Keyboard gone: stop listening but keep playing.
            Some(Err(_)) | None => {
                *keys = None;
                return futures::future::pending().await;
            }
        };
        let Event::Key(key) = event else { continue };
        if key.kind == KeyEventKind::Release {
            continue;
        }
        let action = match key.code {
            KeyCode::Up => Action::Step(-1),
            KeyCode::Down => Action::Step(1),
            KeyCode::Char('r') | KeyCode::Char('R') => Action::Reload,
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => Action::Quit,
            KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => Action::Quit,
            _ => continue,
        };
        return Some(action);
    }
}

// ---- motion sources -----------------------------------------------------------------------

async fn token_source(
    id: Option<String>,
    rate: u16,
    tx: mpsc::Sender<SourceEvent>,
    mut stop: oneshot::Receiver<()>,
) -> Result<()> {
    let msg = |m: String| tx.try_send(SourceEvent::Message(m));
    let opts = triki_ble::ConnectOptions {
        id,
        ..Default::default()
    };
    let _ = msg("looking for a Triki token (press its button / shake it to wake it)…".into());
    let triki = tokio::select! {
        r = triki_ble::Triki::connect(&opts) => r.context("cannot connect to the token")?,
        _ = &mut stop => return Ok(()),
    };
    let battery = match triki.battery().await {
        Ok(Some(b)) => format!(", battery {b}%"),
        _ => String::new(),
    };
    let _ = msg(format!("connected: {}{battery} — shake it!", triki.name()));

    let outcome = async {
        let mut stream = triki
            .start_imu(rate)
            .await
            .context("cannot start the IMU stream")?;
        loop {
            tokio::select! {
                frame = stream.next() => match frame {
                    Some(f) => if tx.send(SourceEvent::Frame(f)).await.is_err() { return Ok(()) },
                    None => bail!("the token disconnected"),
                },
                _ = &mut stop => return Ok(()),
            }
        }
    }
    .await;
    let _ = triki.disconnect().await;
    outcome
}

async fn demo_source(
    rate: u16,
    tx: mpsc::Sender<SourceEvent>,
    mut stop: oneshot::Receiver<()>,
) -> Result<()> {
    let _ = tx.try_send(SourceEvent::Message("demo gesture (no token)".into()));
    let rate_hz = rate as f64;
    let frames = shariki_motion::synth::demo(rate_hz);
    let len = frames.len() as u64;
    let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / rate_hz));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Burst);
    // Loop forever, keeping time continuous across repetitions.
    for n in 0u64.. {
        let mut f = frames[(n % len) as usize];
        f.index = n;
        f.t = n as f64 / rate_hz;
        tokio::select! {
            _ = tick.tick() => if tx.send(SourceEvent::Frame(f)).await.is_err() { break },
            _ = &mut stop => break,
        }
    }
    Ok(())
}
