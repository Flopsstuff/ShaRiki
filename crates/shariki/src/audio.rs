//! Audio output: drives an [`Instrument`] from the cpal callback.
//!
//! Real-time rules: the callback never allocates, frees or locks. Motion states and
//! replacement instruments reach it through lock-free SPSC ring buffers (`rtrb`).
//! Switching instruments crossfades over [`FADE_SECONDS`]; the retired instrument is
//! handed back through another ring buffer so it is freed on the control thread.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use anyhow::{Context, Result, bail};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{
    BufferSize, Device, FromSample, OutputCallbackInfo, SampleFormat, SizedSample, StreamConfig,
    SupportedBufferSize,
};
use rtrb::{Consumer, Producer, RingBuffer};
use shariki_core::{Instrument, MotionState};

/// Largest block rendered in one `Instrument::render` call; bigger device buffers are
/// rendered in several chunks.
const MAX_BLOCK: usize = 2048;
/// Crossfade length when switching instruments.
const FADE_SECONDS: f32 = 0.03;
/// Pending swaps / retired instruments in flight. Far more than a human can press keys.
const SWAP_SLOTS: usize = 16;

type BoxedInstrument = Box<dyn Instrument>;

/// A running output stream. Dropping it stops playback.
pub struct AudioOut {
    _stream: cpal::Stream,
    pub device_name: String,
    pub sample_rate: u32,
    pub buffer: Option<u32>,
    /// Peak absolute output level of the most recent callback, as `f32` bits.
    pub peak: Arc<AtomicU32>,
    swap_tx: Producer<BoxedInstrument>,
    retired_rx: Consumer<BoxedInstrument>,
}

impl AudioOut {
    /// Replaces the playing instrument (crossfaded). Call from the control thread only.
    pub fn swap(&mut self, instrument: BoxedInstrument) -> Result<()> {
        self.collect_retired();
        if self.swap_tx.push(instrument).is_err() {
            bail!("too many instrument switches at once");
        }
        Ok(())
    }

    /// Frees instruments the audio thread has finished with.
    pub fn collect_retired(&mut self) {
        while self.retired_rx.pop().is_ok() {}
    }
}

/// Opens the default output device and starts playing.
///
/// `make_instrument` receives the device sample rate (the instrument is created once the
/// device is known). `buffer_frames` is a latency hint, clamped to what the device supports.
pub fn start(
    make_instrument: impl FnOnce(f32) -> Result<BoxedInstrument>,
    motion_rx: Consumer<MotionState>,
    buffer_frames: Option<u32>,
) -> Result<AudioOut> {
    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .context("no default audio output device")?;
    let device_name = device
        .description()
        .map(|d| d.name().to_string())
        .unwrap_or_else(|_| "unknown device".into());
    let supported = device
        .default_output_config()
        .context("cannot query the output device's default config")?;

    let buffer_size = match (buffer_frames, supported.buffer_size()) {
        (None, _) => BufferSize::Default,
        (Some(n), SupportedBufferSize::Range { min, max }) => {
            BufferSize::Fixed(n.clamp(*min, *max))
        }
        (Some(n), SupportedBufferSize::Unknown) => BufferSize::Fixed(n),
    };
    let buffer = match buffer_size {
        BufferSize::Fixed(n) => Some(n),
        BufferSize::Default => None,
    };

    let sample_format = supported.sample_format();
    let mut config: StreamConfig = supported.into();
    config.buffer_size = buffer_size;

    let sample_rate = config.sample_rate;
    let mut instrument = make_instrument(sample_rate as f32)?;
    instrument.set_sample_rate(sample_rate as f32);

    let (swap_tx, swap_rx) = RingBuffer::new(SWAP_SLOTS);
    let (retired_tx, retired_rx) = RingBuffer::new(SWAP_SLOTS);
    let voice = Voice {
        current: instrument,
        fading: None,
        fade_pos: 0,
        fade_len: ((FADE_SECONDS * sample_rate as f32) as usize).max(1),
        motion_rx,
        swap_rx,
        retired_tx,
        mono: vec![0.0; MAX_BLOCK],
        fade_buf: vec![0.0; MAX_BLOCK],
    };

    let peak = Arc::new(AtomicU32::new(0));
    let p = peak.clone();
    let stream = match sample_format {
        SampleFormat::F32 => build::<f32>(&device, config, voice, p),
        SampleFormat::F64 => build::<f64>(&device, config, voice, p),
        SampleFormat::I16 => build::<i16>(&device, config, voice, p),
        SampleFormat::I32 => build::<i32>(&device, config, voice, p),
        SampleFormat::U16 => build::<u16>(&device, config, voice, p),
        other => bail!("unsupported output sample format {other}"),
    }?;
    stream.play().context("cannot start the audio stream")?;

    Ok(AudioOut {
        _stream: stream,
        device_name,
        sample_rate,
        buffer,
        peak,
        swap_tx,
        retired_rx,
    })
}

/// Everything the audio callback owns.
struct Voice {
    current: BoxedInstrument,
    /// The previous instrument while it fades out.
    fading: Option<BoxedInstrument>,
    fade_pos: usize,
    fade_len: usize,
    motion_rx: Consumer<MotionState>,
    swap_rx: Consumer<BoxedInstrument>,
    retired_tx: Producer<BoxedInstrument>,
    mono: Vec<f32>,
    fade_buf: Vec<f32>,
}

impl Voice {
    /// Hands an instrument back to the control thread. If its queue is somehow full the
    /// instrument is dropped here — a rare, bounded cost preferable to leaking it.
    fn retire(&mut self, inst: BoxedInstrument) {
        let _ = self.retired_tx.push(inst);
    }

    fn take_swaps(&mut self) {
        while let Ok(mut next) = self.swap_rx.pop() {
            if let Some(old) = self.fading.take() {
                self.retire(old);
            }
            std::mem::swap(&mut self.current, &mut next);
            self.fading = Some(next);
            self.fade_pos = 0;
        }
    }

    fn take_motion(&mut self) {
        while let Ok(m) = self.motion_rx.pop() {
            self.current.update(&m);
            if let Some(old) = self.fading.as_mut() {
                old.update(&m);
            }
        }
    }

    /// Renders `n <= MAX_BLOCK` mono samples into `self.mono[..n]`.
    fn render(&mut self, n: usize) {
        self.current.render(&mut self.mono[..n]);
        let Some(old) = self.fading.as_mut() else {
            return;
        };
        old.render(&mut self.fade_buf[..n]);
        for (new, &prev) in self.mono[..n].iter_mut().zip(&self.fade_buf[..n]) {
            let g = (self.fade_pos as f32 / self.fade_len as f32).min(1.0);
            *new = *new * g + prev * (1.0 - g);
            self.fade_pos += 1;
        }
        if self.fade_pos >= self.fade_len
            && let Some(done) = self.fading.take()
        {
            self.retire(done);
        }
    }
}

fn build<T>(
    device: &Device,
    config: StreamConfig,
    mut voice: Voice,
    peak: Arc<AtomicU32>,
) -> Result<cpal::Stream>
where
    T: SizedSample + FromSample<f32>,
{
    let channels = config.channels as usize;

    let callback = move |data: &mut [T], _: &OutputCallbackInfo| {
        voice.take_swaps();
        voice.take_motion();

        let frames = data.len() / channels;
        let mut done = 0;
        let mut block_peak = 0.0f32;
        while done < frames {
            let n = (frames - done).min(MAX_BLOCK);
            voice.render(n);
            let out = &mut data[done * channels..(done + n) * channels];
            for (frame, &s) in out.chunks_exact_mut(channels).zip(&voice.mono[..n]) {
                block_peak = block_peak.max(s.abs());
                frame.fill(T::from_sample(s));
            }
            done += n;
        }
        peak.store(block_peak.to_bits(), Ordering::Relaxed);
    };

    let on_error = |err: cpal::Error| eprint!("\r\naudio stream error: {err}\r\n");
    device
        .build_output_stream(config, callback, on_error, None)
        .context("cannot open the audio output stream")
}
