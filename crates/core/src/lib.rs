//! Shared contract between the shariki crates.
//!
//! Data flow:
//! ```text
//! triki-ble ──ImuFrame──▶ shariki-motion ──MotionState──▶ shariki-engine (Instrument) ──▶ audio
//! ```
//! Dependency-free on purpose: every other crate depends on this one, nothing else.

/// One decoded IMU sample from the token, already scaled to physical units.
///
/// Sensor frame (see triki docs, "IMU axis orientation"): right-handed; +Z points out of
/// the PCB face. At rest the accelerometer reads the *reaction* to gravity, i.e. the axis
/// facing the sky reads about +1 g.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuFrame {
    /// Sample time in seconds since the stream started.
    ///
    /// Reconstructed from the sample index and the nominal rate (`index / rate_hz`), not
    /// from the BLE arrival time: notifications arrive in bursts, but the IMU samples on a
    /// fixed clock.
    pub t: f64,
    /// Running sample index since the stream started (0-based).
    pub index: u64,
    /// Angular rate in deg/s, `[x, y, z]`.
    pub gyro_dps: [f32; 3],
    /// Specific force in g, `[x, y, z]` (≈ +1 g on the sky-facing axis at rest).
    pub accel_g: [f32; 3],
}

/// Gesture features extracted from the IMU stream: what the instruments react to.
///
/// Produced by `shariki-motion` at the IMU rate (~104 Hz) and consumed by an
/// [`Instrument`] on the audio thread.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionState {
    /// Time of the IMU sample this state was computed from, in seconds (see [`ImuFrame::t`]).
    pub t: f64,
    /// Continuous shake intensity, normalised so that a vigorous shake is ~1.0.
    /// 0.0 = still. May exceed 1.0 on violent motion; consumers should clamp if needed.
    pub energy: f32,
    /// Strength of a sharp hit or direction reversal detected *on this sample*
    /// (0.0 = none, ~1.0 = hard). Non-zero for a single state per detected event.
    pub impulse: f32,
    /// Unit vector pointing to the sky in the sensor frame (low-passed accel direction).
    /// At rest with the PCB face up this is `[0, 0, 1]`. Used for tilt-driven instruments
    /// (rainstick, ocean drum).
    pub up: [f32; 3],
    /// Magnitude of the angular rate, deg/s.
    pub angular_speed: f32,
    /// Linear acceleration magnitude with gravity removed, in g.
    pub linear_accel: f32,
}

impl Default for MotionState {
    fn default() -> Self {
        Self {
            t: 0.0,
            energy: 0.0,
            impulse: 0.0,
            up: [0.0, 0.0, 1.0],
            angular_speed: 0.0,
            linear_accel: 0.0,
        }
    }
}

/// Description of one tweakable instrument parameter, for the CLI now and a UI later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamSpec {
    /// Stable machine id, kebab-case, e.g. `"beads"`, `"decay"`.
    pub id: &'static str,
    /// Human-readable name.
    pub name: &'static str,
    pub min: f32,
    pub max: f32,
    pub default: f32,
    /// One-line explanation of what the parameter does to the sound.
    pub help: &'static str,
}

/// Error from [`Instrument::set_param`].
#[derive(Debug, Clone, PartialEq)]
pub enum ParamError {
    Unknown(String),
    OutOfRange {
        id: String,
        value: f32,
        min: f32,
        max: f32,
    },
}

impl std::fmt::Display for ParamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParamError::Unknown(id) => write!(f, "unknown parameter `{id}`"),
            ParamError::OutOfRange {
                id,
                value,
                min,
                max,
            } => {
                write!(f, "parameter `{id}` = {value} is outside [{min}, {max}]")
            }
        }
    }
}

impl std::error::Error for ParamError {}

/// A sound engine voice driven by motion.
///
/// Real-time contract: [`Instrument::update`] and [`Instrument::render`] run on the audio
/// thread and MUST NOT allocate, lock, block or do I/O. Do any allocation in the
/// constructor or in [`Instrument::set_sample_rate`] / [`Instrument::set_param`], which are
/// called off the audio thread or before playback starts.
pub trait Instrument: Send {
    /// Stable id, e.g. `"maracas"`.
    fn id(&self) -> &str;
    /// Human-readable name, e.g. `"Maracas"`.
    fn name(&self) -> &str;

    /// Called before playback (and whenever the output device changes).
    fn set_sample_rate(&mut self, sample_rate: f32);

    /// The parameters this instrument exposes.
    fn params(&self) -> &[ParamSpec];
    /// Current value of a parameter, `None` if the id is unknown.
    fn param(&self, id: &str) -> Option<f32>;
    /// Set a parameter. Values outside `[min, max]` are rejected.
    fn set_param(&mut self, id: &str, value: f32) -> Result<(), ParamError>;

    /// Feed the latest gesture state. Called on the audio thread, possibly several times
    /// per block (motion states arrive in bursts) or not at all for a few blocks.
    fn update(&mut self, motion: &MotionState);

    /// Render the next block of mono samples into `out` (overwrite, do not mix),
    /// nominally in `[-1.0, 1.0]`.
    fn render(&mut self, out: &mut [f32]);
}
