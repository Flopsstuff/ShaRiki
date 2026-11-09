//! Gesture features from the token's IMU stream.
//!
//! [`MotionTracker`] turns [`ImuFrame`]s (~104 Hz, gyro in deg/s, accel in g) into
//! [`MotionState`]s: shake `energy`, per-hit `impulse`, the `up` vector, `angular_speed`
//! and gravity-free `linear_accel`. Everything is plain `f32` math, allocation-free and
//! sample-rate independent (all time constants are in seconds and converted with the
//! actual `dt` of each frame).
//!
//! # Pipeline (per frame)
//!
//! 1. **Timing.** `dt` comes from consecutive [`ImuFrame::t`]. The first frame, a frame
//!    whose time goes backwards, a non-finite time, or a jump larger than
//!    [`MotionConfig::max_gap_s`] *re-seeds* the filters (no derivative is taken across
//!    the discontinuity, so no impulse and no energy spike). A frame with exactly the same
//!    `t` as the previous one is treated as a duplicate and returns the previous state
//!    with `impulse = 0`.
//! 2. **Gravity / `up`** — complementary filter. The previous `up` estimate is rotated by
//!    the gyro (trapezoidal rate, exact Rodrigues rotation), then nudged toward the
//!    normalised accel direction with time constant [`MotionConfig::up_tau_s`], scaled by
//!    a *trust* in `[0, 1]`:
//!    * dynamic trust: `1 − |accel − up·1g| / up_trust_residual_g` (squared, clamped) —
//!      samples that disagree with the current estimate by more than the residual are
//!      ignored, which keeps `up` stable while shaking (including the
//!      `accel ≈ −1 g` moments of a vertical shake that a plain `|a| ≈ 1 g` gate lets in);
//!    * still trust: a stillness detector (`||a| − 1g|` and `|ω|` under small thresholds,
//!      held for [`MotionConfig::still_release_s`]) gives full trust regardless of the
//!      residual, so a wrong estimate (e.g. after a re-seed mid-shake) converges as soon
//!      as the token rests.
//!
//!    In addition, `up` is always pulled very slowly ([`MotionConfig::up_motion_tau_s`])
//!    toward the raw accel *vector*: the linear part of a shake is zero-mean, so this
//!    rotation-compensated low-pass converges on gravity and bounds gyro drift during
//!    long shakes. The gyro zero-rate offset is learned while still
//!    ([`MotionConfig::gyro_bias_tau_s`]) and subtracted from every reading, including
//!    the reported `angular_speed`.
//! 3. **`linear_accel`** = `|accel − up·1g|`.
//! 4. **`energy`** — envelope follower (attack [`MotionConfig::energy_attack_s`], release
//!    [`MotionConfig::energy_release_s`]) over
//!    `w_a·max(0, lin − dead_a)/full_a + w_g·max(0, |ω| − dead_g)/full_g`.
//!    The dead-zones make a resting token read exactly `0.0`.
//! 5. **`impulse`** — the shaker "chk" happens when the beads hit the wall, i.e. on a
//!    reversal of the linear acceleration. The gravity-free accel is high-passed
//!    ([`MotionConfig::impulse_hp_tau_s`]) and projected onto its dominant recent axis
//!    (principal eigenvector of an exponentially weighted covariance, by power
//!    iteration). An event fires when this projection crosses from beyond `+h` to beyond
//!    `−h` or vice versa (hysteresis [`MotionConfig::impulse_hysteresis_g`]), outside
//!    the refractory period ([`MotionConfig::impulse_refractory_s`]). Strength is the
//!    projected jerk at the crossing over [`MotionConfig::impulse_jerk_full_gps`],
//!    clamped to `[0.01, 1.0]`: sharp stops click loudly, slow sweeps softly.
//!    A sinusoidal shake therefore gives one impulse per half cycle (two per cycle). After
//!    [`MotionConfig::impulse_rearm_s`] inside the band the detector forgets its side, so
//!    the first push from rest does not click; the stop that follows does.

use shariki_core::{ImuFrame, MotionState};

mod math;
pub mod synth;

use math::{V3, add, dot, norm, normalize, rotate_world_vec, scale, sub};

/// Tuning of [`MotionTracker`]. All time constants are in seconds (rate independent),
/// accelerations in g, angular rates in deg/s.
#[derive(Debug, Clone, PartialEq)]
pub struct MotionConfig {
    /// A forward jump in `t` larger than this re-seeds the filters instead of being
    /// integrated (BLE drop-out, reconnect).
    pub max_gap_s: f32,

    /// Time constant with which `up` converges to the accel direction at full trust.
    /// Smaller = faster tilt correction, noisier during motion.
    pub up_tau_s: f32,
    /// Residual `|accel − up·1g|` (g) at which an accel sample stops being trusted for
    /// gravity. Shaking produces residuals well above this.
    pub up_trust_residual_g: f32,
    /// Time constant of the always-on, untrusted pull of `up` toward the raw accel
    /// *vector* (not normalised). Shake accel is zero-mean, so this averages it out and
    /// bounds gyro drift during long shakes. `0` disables it.
    pub up_motion_tau_s: f32,
    /// Time constant of the gyro zero-rate (bias) estimate, learned while still.
    /// `0` disables bias estimation.
    pub gyro_bias_tau_s: f32,
    /// Bias is only learned while the raw rate is below this (deg/s); the estimate is
    /// also clamped to this magnitude.
    pub gyro_bias_max_dps: f32,
    /// Stillness detector: max `||accel| − 1g|` (g) for the token to count as still.
    pub still_accel_g: f32,
    /// Stillness detector: max `|ω|` (deg/s) for the token to count as still.
    pub still_gyro_dps: f32,
    /// Stillness detector: how long (release time constant) motion keeps the token from
    /// counting as still again.
    pub still_release_s: f32,

    /// Energy envelope attack time constant.
    pub energy_attack_s: f32,
    /// Energy envelope release time constant.
    pub energy_release_s: f32,
    /// Weight of the linear-acceleration term in the energy drive.
    pub energy_accel_weight: f32,
    /// Weight of the angular-speed term in the energy drive.
    pub energy_gyro_weight: f32,
    /// Linear acceleration (g) below which the accel term is zero (noise dead-zone).
    pub energy_accel_dead_g: f32,
    /// Angular speed (deg/s) below which the gyro term is zero (noise/bias dead-zone).
    pub energy_gyro_dead_dps: f32,
    /// Linear acceleration above the dead-zone (g) at which the accel term reaches 1.
    pub energy_accel_full_g: f32,
    /// Angular speed above the dead-zone (deg/s) at which the gyro term reaches 1.
    pub energy_gyro_full_dps: f32,

    /// High-pass time constant applied to the linear acceleration before reversal
    /// detection (removes slow gravity-estimate leaks).
    pub impulse_hp_tau_s: f32,
    /// Averaging time constant of the covariance used to find the dominant shake axis.
    pub impulse_axis_tau_s: f32,
    /// Hysteresis (g): the projected accel must swing from beyond `+h` to beyond `−h`
    /// (or back) to count as a reversal. Also the effective detection threshold.
    pub impulse_hysteresis_g: f32,
    /// Minimum time between two impulses.
    pub impulse_refractory_s: f32,
    /// Projected jerk (g/s) at the crossing that maps to impulse strength 1.0.
    pub impulse_jerk_full_gps: f32,
    /// After the projection has stayed inside the hysteresis band this long, the
    /// detector forgets which side it was on, so the first push of the next gesture does
    /// not click (only the stop that follows it does).
    pub impulse_rearm_s: f32,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            max_gap_s: 0.1,

            up_tau_s: 0.5,
            up_trust_residual_g: 0.3,
            up_motion_tau_s: 3.0,
            gyro_bias_tau_s: 1.0,
            gyro_bias_max_dps: 10.0,
            still_accel_g: 0.08,
            still_gyro_dps: 30.0,
            still_release_s: 0.15,

            energy_attack_s: 0.007,
            energy_release_s: 0.05,
            energy_accel_weight: 0.8,
            energy_gyro_weight: 0.2,
            energy_accel_dead_g: 0.06,
            energy_gyro_dead_dps: 20.0,
            energy_accel_full_g: 2.5,
            energy_gyro_full_dps: 1000.0,

            impulse_hp_tau_s: 0.4,
            impulse_axis_tau_s: 0.25,
            impulse_hysteresis_g: 0.25,
            impulse_refractory_s: 0.05,
            impulse_jerk_full_gps: 140.0,
            impulse_rearm_s: 0.25,
        }
    }
}

/// Inputs are clamped to these magnitudes (well beyond the LSM6DSL's ±16 g / ±2000 dps)
/// so squares and sums can never overflow `f32`.
const ACCEL_CLAMP_G: f32 = 100.0;
const GYRO_CLAMP_DPS: f32 = 10_000.0;

/// Streaming gesture-feature extractor. Feed every frame in order with [`push`](Self::push).
#[derive(Debug, Clone)]
pub struct MotionTracker {
    cfg: MotionConfig,
    last_t: Option<f64>,
    last_state: MotionState,
    prev_gyro: V3,
    gyro_bias: V3,
    up: V3,
    still_env: f32,
    energy: f32,
    lin_mean: V3,
    /// Symmetric covariance of the high-passed linear accel: xx, yy, zz, xy, xz, yz.
    cov: [f32; 6],
    axis: V3,
    /// Previous projections (most recent first) and how many are valid.
    s_hist: [f32; 2],
    s_len: u8,
    /// +1 / −1: side of the hysteresis band the projection was last on; 0: unknown.
    polarity: i8,
    since_impulse: f32,
    /// Time the projection has spent inside the hysteresis band.
    quiet_s: f32,
}

impl MotionTracker {
    pub fn new(cfg: MotionConfig) -> Self {
        Self {
            cfg,
            last_t: None,
            last_state: MotionState::default(),
            prev_gyro: [0.0; 3],
            gyro_bias: [0.0; 3],
            up: [0.0, 0.0, 1.0],
            still_env: 1.0,
            energy: 0.0,
            lin_mean: [0.0; 3],
            cov: [0.0; 6],
            axis: [1.0, 0.0, 0.0],
            s_hist: [0.0; 2],
            s_len: 0,
            polarity: 0,
            since_impulse: f32::INFINITY,
            quiet_s: 0.0,
        }
    }

    pub fn config(&self) -> &MotionConfig {
        &self.cfg
    }

    /// Forget all history (keeps the config); the next frame is treated as the first.
    pub fn reset(&mut self) {
        let cfg = std::mem::take(&mut self.cfg);
        *self = Self::new(cfg);
    }

    /// Process one frame and return the features for it. Never allocates; never returns
    /// NaN (non-finite inputs are treated as 0, huge inputs are clamped).
    pub fn push(&mut self, frame: &ImuFrame) -> MotionState {
        let gyro_raw = sanitize(frame.gyro_dps, GYRO_CLAMP_DPS);
        let gyro = sub(gyro_raw, self.gyro_bias);
        let accel = sanitize(frame.accel_g, ACCEL_CLAMP_G);

        let state = match self.last_t {
            Some(lt) if frame.t.is_finite() && frame.t == lt => {
                // Duplicate sample: nothing new happened.
                return MotionState {
                    impulse: 0.0,
                    ..self.last_state
                };
            }
            Some(lt) if frame.t.is_finite() => {
                let dt = frame.t - lt;
                if dt > 0.0 && dt <= f64::from(self.cfg.max_gap_s) {
                    self.step(frame.t, dt as f32, gyro_raw, gyro, accel)
                } else {
                    let gap = if dt > 0.0 { Some(dt as f32) } else { None };
                    self.reseed(frame.t, gap, gyro, accel)
                }
            }
            _ => self.reseed(frame.t, None, gyro, accel),
        };

        self.last_t = frame.t.is_finite().then_some(frame.t);
        self.last_state = state;
        state
    }

    /// First frame or a discontinuity: restart filters from this sample, take no
    /// derivatives. `gap` is the elapsed time if known (forward jump).
    fn reseed(&mut self, t: f64, gap: Option<f32>, gyro: V3, accel: V3) -> MotionState {
        let a_mag = norm(accel);
        if (0.5..=1.5).contains(&a_mag)
            && let Some(dir) = normalize(accel)
        {
            self.up = dir;
        }
        self.energy = match gap {
            Some(g) => self.energy * (-g / self.cfg.energy_release_s.max(1e-4)).exp(),
            None => 0.0,
        };
        self.still_env = 1.0;
        self.prev_gyro = gyro;
        self.lin_mean = [0.0; 3];
        self.s_hist = [0.0; 2];
        self.s_len = 0;
        self.polarity = 0;
        self.since_impulse = f32::INFINITY;
        self.quiet_s = 0.0;

        let lin = sub(accel, self.up);
        MotionState {
            t: if t.is_finite() { t } else { 0.0 },
            energy: self.energy,
            impulse: 0.0,
            up: self.up,
            angular_speed: norm(gyro),
            linear_accel: norm(lin),
        }
    }

    fn step(&mut self, t: f64, dt: f32, gyro_raw: V3, gyro: V3, accel: V3) -> MotionState {
        let c = &self.cfg;
        let w = norm(gyro);
        let a_mag = norm(accel);

        // --- gravity / up: gyro prediction -------------------------------------------
        let omega_avg = scale(add(self.prev_gyro, gyro), 0.5);
        self.prev_gyro = gyro;
        self.up = normalize(rotate_world_vec(self.up, omega_avg, dt)).unwrap_or(self.up);

        // --- stillness detector -------------------------------------------------------
        let dev =
            ((a_mag - 1.0).abs() / c.still_accel_g.max(1e-6)).max(w / c.still_gyro_dps.max(1e-6));
        if dev >= self.still_env {
            self.still_env = dev;
        } else {
            self.still_env += (dev - self.still_env) * coef(dt, c.still_release_s);
        }
        let still_trust = (2.0 * (1.0 - self.still_env)).clamp(0.0, 1.0);

        // --- gyro bias: learn the zero-rate offset while still ------------------------
        if c.gyro_bias_tau_s > 0.0 && still_trust >= 1.0 && norm(gyro_raw) < c.gyro_bias_max_dps {
            let k = coef(dt, c.gyro_bias_tau_s);
            let lim = c.gyro_bias_max_dps;
            self.gyro_bias = add(self.gyro_bias, scale(sub(gyro_raw, self.gyro_bias), k))
                .map(|b| b.clamp(-lim, lim));
        }

        // --- gravity / up: accel correction ------------------------------------------
        if let Some(a_dir) = normalize(accel).filter(|_| a_mag > 0.1) {
            let residual = norm(sub(accel, self.up));
            let dyn_trust = (1.0 - residual / c.up_trust_residual_g.max(1e-6)).clamp(0.0, 1.0);
            let trust = (dyn_trust * dyn_trust).max(still_trust);
            let k = coef(dt, c.up_tau_s) * trust;
            if k > 0.0 {
                let blended = add(self.up, scale(sub(a_dir, self.up), k));
                self.up = normalize(blended).unwrap_or(self.up);
            }
        }
        if c.up_motion_tau_s > 0.0 {
            let blended = add(
                self.up,
                scale(sub(accel, self.up), coef(dt, c.up_motion_tau_s)),
            );
            self.up = normalize(blended).unwrap_or(self.up);
        }

        // --- linear accel ---------------------------------------------------------------
        let lin_vec = sub(accel, self.up);
        let lin = norm(lin_vec);

        // --- energy -------------------------------------------------------------------
        let drive = c.energy_accel_weight * (lin - c.energy_accel_dead_g).max(0.0)
            / c.energy_accel_full_g.max(1e-6)
            + c.energy_gyro_weight * (w - c.energy_gyro_dead_dps).max(0.0)
                / c.energy_gyro_full_dps.max(1e-6);
        let tau = if drive > self.energy {
            c.energy_attack_s
        } else {
            c.energy_release_s
        };
        self.energy += (drive - self.energy) * coef(dt, tau);
        if self.energy < 1e-6 {
            self.energy = 0.0;
        }

        // --- impulse: reversal of the dominant-axis linear accel ----------------------
        self.lin_mean = add(
            self.lin_mean,
            scale(sub(lin_vec, self.lin_mean), coef(dt, c.impulse_hp_tau_s)),
        );
        let hp = sub(lin_vec, self.lin_mean);
        self.update_axis(hp, coef(dt, c.impulse_axis_tau_s));
        let s = dot(hp, self.axis);

        let c = &self.cfg;
        let jerk = match self.s_len {
            0 => 0.0,
            1 => (s - self.s_hist[0]) / dt,
            _ => (s - self.s_hist[1]) / (2.0 * dt),
        };
        self.s_hist = [s, self.s_hist[0]];
        self.s_len = (self.s_len + 1).min(2);

        self.since_impulse += dt;
        let h = c.impulse_hysteresis_g;
        let side = if s > h {
            1
        } else if s < -h {
            -1
        } else {
            0
        };
        if side == 0 {
            self.quiet_s += dt;
            if self.quiet_s >= c.impulse_rearm_s {
                self.polarity = 0;
            }
        } else {
            self.quiet_s = 0.0;
        }
        let mut impulse = 0.0;
        if side != 0 && side != self.polarity {
            if self.polarity != 0 && self.since_impulse >= c.impulse_refractory_s {
                impulse = (jerk.abs() / c.impulse_jerk_full_gps.max(1e-6)).clamp(0.01, 1.0);
                self.since_impulse = 0.0;
            }
            self.polarity = side;
        }

        MotionState {
            t,
            energy: self.energy,
            impulse,
            up: self.up,
            angular_speed: w,
            linear_accel: lin,
        }
    }

    /// Track the principal axis of the recent high-passed linear accel.
    fn update_axis(&mut self, v: V3, k: f32) {
        let outer = [
            v[0] * v[0],
            v[1] * v[1],
            v[2] * v[2],
            v[0] * v[1],
            v[0] * v[2],
            v[1] * v[2],
        ];
        for (c, o) in self.cov.iter_mut().zip(outer) {
            *c += (o - *c) * k;
        }
        let [xx, yy, zz, xy, xz, yz] = self.cov;
        // Only re-estimate when there is real motion; at rest keep the last axis.
        let floor = 0.25 * self.cfg.impulse_hysteresis_g * self.cfg.impulse_hysteresis_g;
        if xx + yy + zz < floor {
            return;
        }
        let mul = |a: V3| -> V3 {
            [
                xx * a[0] + xy * a[1] + xz * a[2],
                xy * a[0] + yy * a[1] + yz * a[2],
                xz * a[0] + yz * a[1] + zz * a[2],
            ]
        };
        // Seed with the column of the largest diagonal entry (never orthogonal to the
        // principal eigenvector of a PSD matrix), then a few power iterations.
        let mut a = if xx >= yy && xx >= zz {
            [xx, xy, xz]
        } else if yy >= zz {
            [xy, yy, yz]
        } else {
            [xz, yz, zz]
        };
        for _ in 0..3 {
            a = match normalize(mul(a)) {
                Some(n) => n,
                None => return,
            };
        }
        // Keep sign continuity so the projection does not flip spuriously.
        if dot(a, self.axis) < 0.0 {
            a = scale(a, -1.0);
        }
        self.axis = a;
    }
}

impl Default for MotionTracker {
    fn default() -> Self {
        Self::new(MotionConfig::default())
    }
}

/// One-pole smoothing coefficient for a step of `dt` with time constant `tau`.
#[inline]
fn coef(dt: f32, tau: f32) -> f32 {
    if tau <= 0.0 {
        1.0
    } else {
        1.0 - (-dt / tau).exp()
    }
}

/// Replace non-finite components with 0 and clamp to `±limit`.
#[inline]
fn sanitize(v: V3, limit: f32) -> V3 {
    v.map(|x| {
        if x.is_finite() {
            x.clamp(-limit, limit)
        } else {
            0.0
        }
    })
}
