//! Wire protocol of the Żabka Triki token: pure data, no BLE.
//!
//! The token (nRF52810 + LSM6DSL) speaks over the Nordic UART Service (NUS):
//!
//! - **RX** ([`NUS_RX`], host → token): commands. [`start_cmd`] starts the IMU stream.
//! - **TX** ([`NUS_TX`], token → host, notify): 14-byte motion frames, see [`decode`].
//! - **CTRL** ([`NUS_CTRL`]): one byte driving the green LED, see [`led_cmd`].
//!
//! There is no stop command: the stream ends when the link drops.
//!
//! Frames are not aligned to notifications (one notification may carry a partial frame
//! or several frames), so they are reassembled by the stateful [`FrameParser`].
//! [`ImuDecoder`] goes one step further and turns notification chunks into scaled
//! [`ImuFrame`]s with reconstructed timestamps.

use shariki_core::ImuFrame;
use uuid::Uuid;

/// Nordic UART Service.
pub const NUS_SERVICE: Uuid = Uuid::from_u128(0x6e400001_b5a3_f393_e0a9_e50e24dcca9e);
/// NUS RX characteristic: host → token commands (write / write-without-response).
pub const NUS_RX: Uuid = Uuid::from_u128(0x6e400002_b5a3_f393_e0a9_e50e24dcca9e);
/// NUS TX characteristic: token → host motion frames (notify).
pub const NUS_TX: Uuid = Uuid::from_u128(0x6e400003_b5a3_f393_e0a9_e50e24dcca9e);
/// Control register: 1 byte read/write, drives the green LED (`0x01` on, `0x00` off).
pub const NUS_CTRL: Uuid = Uuid::from_u128(0x6e400004_b5a3_f393_e0a9_e50e24dcca9e);
/// Standard Battery Service (0x180F).
pub const BATTERY_SERVICE: Uuid = Uuid::from_u128(0x0000180f_0000_1000_8000_00805f9b34fb);
/// Standard Battery Level characteristic (0x2A19): uint8 percent.
pub const BATTERY_LEVEL: Uuid = Uuid::from_u128(0x00002a19_0000_1000_8000_00805f9b34fb);

/// Advertised name prefix. Tokens advertise `TRIKI <serial>` (some firmware `Triki
/// <serial>`), so match it case-insensitively, see [`name_matches`].
pub const DEVICE_NAME_PREFIX: &str = "TRIKI";

/// Base bytes of the 8-byte IMU start command. Bytes 5–6 carry the sample rate as a
/// little-endian `u16` in Hz (`0x0068` = 104 Hz). Bytes 3–4 (`D0 07`) and 7 (`03`) are a
/// fixed, not-yet-decoded part of the command.
pub const START_BASE: [u8; 8] = [0x20, 0x10, 0x00, 0xd0, 0x07, 0x68, 0x00, 0x03];

/// Motion-frame header (bytes 0–1).
pub const HEADER: [u8; 2] = [0x22, 0x00];
/// Motion-frame length in bytes: header + six `i16` axes.
pub const FRAME_LEN: usize = 14;

/// Gyro scale, LSB per deg/s (LSM6DSL ±2000 dps, 70 mdps/LSB).
///
/// Not 131.0: that is an MPU-6050 ±250 dps value and under-rotates ~9× on this token.
pub const GYRO_LSB_PER_DPS: f32 = 14.286;
/// Accelerometer scale, LSB per g (LSM6DSL ±16 g; a flat token reads +2048 = 1 g).
pub const ACCEL_LSB_PER_G: f32 = 2048.0;

/// Sample rates (Hz) the firmware accepts, a subset of the LSM6DSL ODR ladder.
/// 12.5 Hz is rejected by the firmware; above ~208 Hz BLE may not keep up.
pub const SUPPORTED_RATES_HZ: [u16; 5] = [26, 52, 104, 208, 416];
/// Default sample rate in Hz.
pub const DEFAULT_RATE_HZ: u16 = 104;
/// Frames dropped at the start of a stream by default: the first ~20 frames after the
/// start command are noise.
pub const DEFAULT_SKIP_FRAMES: u64 = 20;

/// Bytes kept while no header is found; past this the buffer is trimmed to its last byte
/// (which may be the first half of a header split across notifications).
const MAX_GARBAGE: usize = 64;

/// Whether `hz` is one of [`SUPPORTED_RATES_HZ`].
pub fn is_supported_rate(hz: u16) -> bool {
    SUPPORTED_RATES_HZ.contains(&hz)
}

/// Build the 8-byte START command for a sample rate (bytes 5–6 = rate, LE `u16`).
///
/// Does not validate the rate; see [`is_supported_rate`].
pub fn start_cmd(hz: u16) -> [u8; 8] {
    let mut cmd = START_BASE;
    cmd[5..7].copy_from_slice(&hz.to_le_bytes());
    cmd
}

/// Build the 1-byte LED control payload for [`NUS_CTRL`].
pub fn led_cmd(on: bool) -> [u8; 1] {
    [u8::from(on)]
}

/// Whether an advertised name belongs to a token with the given prefix
/// (ASCII case-insensitive, so `"TRIKI"` matches both `TRIKI 1234` and `Triki 1234`).
pub fn name_matches(name: &str, prefix: &str) -> bool {
    name.len() >= prefix.len()
        && name.as_bytes()[..prefix.len()].eq_ignore_ascii_case(prefix.as_bytes())
}

/// One decoded frame as raw signed sensor counts (no scaling).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RawFrame {
    /// Gyro counts `[x, y, z]`; divide by [`GYRO_LSB_PER_DPS`] for deg/s.
    pub gyro: [i16; 3],
    /// Accel counts `[x, y, z]`; divide by [`ACCEL_LSB_PER_G`] for g.
    pub accel: [i16; 3],
}

impl RawFrame {
    /// Angular rate in deg/s.
    pub fn gyro_dps(&self) -> [f32; 3] {
        self.gyro.map(|v| f32::from(v) / GYRO_LSB_PER_DPS)
    }

    /// Specific force in g.
    pub fn accel_g(&self) -> [f32; 3] {
        self.accel.map(|v| f32::from(v) / ACCEL_LSB_PER_G)
    }
}

/// Decode one 14-byte frame: `i16` LE at offsets 2/4/6 (gyro) and 8/10/12 (accel).
///
/// The header is not checked (the [`FrameParser`] only hands out header-aligned frames).
pub fn decode(frame: &[u8; FRAME_LEN]) -> RawFrame {
    let at = |o: usize| i16::from_le_bytes([frame[o], frame[o + 1]]);
    RawFrame {
        gyro: [at(2), at(4), at(6)],
        accel: [at(8), at(10), at(12)],
    }
}

/// Stateful framer: accumulates notification chunks, finds the `22 00` header, drops
/// leading garbage and yields complete 14-byte frames.
///
/// A lone trailing `0x22` is retained as a possible first half of a split header.
#[derive(Debug, Default, Clone)]
pub struct FrameParser {
    buf: Vec<u8>,
}

impl FrameParser {
    /// An empty parser.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a notification chunk and call `on_frame` for every frame now complete.
    pub fn push_with(&mut self, chunk: &[u8], mut on_frame: impl FnMut(RawFrame)) {
        self.buf.extend_from_slice(chunk);
        let mut pos = 0;
        loop {
            let rest = &self.buf[pos..];
            let Some(j) = rest.windows(2).position(|w| w == HEADER) else {
                // No header. Keep everything while it is small (a header may still be
                // split across chunks); past that, keep only the last byte.
                if rest.len() > MAX_GARBAGE {
                    pos = self.buf.len() - 1;
                }
                break;
            };
            pos += j; // drop garbage before the header
            let Some(frame) = self.buf.get(pos..pos + FRAME_LEN) else {
                break; // wait for a full frame
            };
            let frame: &[u8; FRAME_LEN] = frame.try_into().expect("slice has FRAME_LEN bytes");
            on_frame(decode(frame));
            pos += FRAME_LEN;
        }
        self.buf.drain(..pos);
    }

    /// Append a notification chunk and return every frame now complete.
    pub fn push(&mut self, chunk: &[u8]) -> Vec<RawFrame> {
        let mut out = Vec::new();
        self.push_with(chunk, |f| out.push(f));
        out
    }

    /// Drop buffered bytes.
    pub fn reset(&mut self) {
        self.buf.clear();
    }

    /// Number of bytes currently buffered (a partial frame or unresolved garbage).
    pub fn buffered(&self) -> usize {
        self.buf.len()
    }
}

/// Turns TX notification chunks into scaled [`ImuFrame`]s.
///
/// Wraps a [`FrameParser`], drops the first `skip_frames` frames (start-up noise), and
/// numbers the remaining ones: the first emitted frame has `index == 0`, and
/// `t = index / rate_hz`. Time is reconstructed from the nominal rate rather than the
/// arrival time, because notifications arrive in bursts while the IMU samples on a fixed
/// clock.
#[derive(Debug, Clone)]
pub struct ImuDecoder {
    parser: FrameParser,
    rate_hz: f64,
    skip_remaining: u64,
    next_index: u64,
}

impl ImuDecoder {
    /// A decoder for a stream at `rate_hz`, dropping the first `skip_frames` frames.
    ///
    /// # Panics
    /// If `rate_hz` is 0.
    pub fn new(rate_hz: u16, skip_frames: u64) -> Self {
        assert!(rate_hz > 0, "rate_hz must be positive");
        Self {
            parser: FrameParser::new(),
            rate_hz: f64::from(rate_hz),
            skip_remaining: skip_frames,
            next_index: 0,
        }
    }

    /// Nominal sample rate in Hz.
    pub fn rate_hz(&self) -> f64 {
        self.rate_hz
    }

    /// Feed a notification chunk and call `on_frame` for every new sample.
    pub fn push_with(&mut self, chunk: &[u8], mut on_frame: impl FnMut(ImuFrame)) {
        let Self {
            parser,
            rate_hz,
            skip_remaining,
            next_index,
        } = self;
        parser.push_with(chunk, |raw| {
            if *skip_remaining > 0 {
                *skip_remaining -= 1;
                return;
            }
            let index = *next_index;
            *next_index += 1;
            on_frame(ImuFrame {
                t: index as f64 / *rate_hz,
                index,
                gyro_dps: raw.gyro_dps(),
                accel_g: raw.accel_g(),
            });
        });
    }

    /// Feed a notification chunk and return every new sample.
    pub fn push(&mut self, chunk: &[u8]) -> Vec<ImuFrame> {
        let mut out = Vec::new();
        self.push_with(chunk, |f| out.push(f));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a valid 14-byte frame from raw counts.
    fn frame(gx: i16, gy: i16, gz: i16, ax: i16, ay: i16, az: i16) -> Vec<u8> {
        let mut f = HEADER.to_vec();
        for v in [gx, gy, gz, ax, ay, az] {
            f.extend_from_slice(&v.to_le_bytes());
        }
        f
    }

    fn gx(n: i16) -> Vec<u8> {
        frame(n, 0, 0, 0, 0, 0)
    }

    fn gxs(frames: &[RawFrame]) -> Vec<i16> {
        frames.iter().map(|f| f.gyro[0]).collect()
    }

    #[test]
    fn start_cmd_for_all_rates() {
        let expected: [(u16, [u8; 2]); 5] = [
            (26, [0x1a, 0x00]),
            (52, [0x34, 0x00]),
            (104, [0x68, 0x00]),
            (208, [0xd0, 0x00]),
            (416, [0xa0, 0x01]),
        ];
        for (hz, [lo, hi]) in expected {
            assert!(is_supported_rate(hz));
            assert_eq!(
                start_cmd(hz),
                [0x20, 0x10, 0x00, 0xd0, 0x07, lo, hi, 0x03],
                "{hz} Hz"
            );
        }
        assert_eq!(start_cmd(DEFAULT_RATE_HZ), START_BASE);
    }

    #[test]
    fn unsupported_rates() {
        for hz in [0, 12, 13, 100, 105, 833, 1000] {
            assert!(!is_supported_rate(hz), "{hz}");
        }
    }

    #[test]
    fn led_payload() {
        assert_eq!(led_cmd(true), [0x01]);
        assert_eq!(led_cmd(false), [0x00]);
    }

    #[test]
    fn name_prefix_case_insensitive() {
        assert!(name_matches("TRIKI 1234", "TRIKI"));
        assert!(name_matches("Triki 1234", "TRIKI"));
        assert!(name_matches("triki", "TRIKI"));
        assert!(!name_matches("TRIK", "TRIKI"));
        assert!(!name_matches("My TRIKI", "TRIKI"));
        assert!(!name_matches("Żabka", "TRIKI"));
        assert!(name_matches("anything", ""));
    }

    #[test]
    fn decode_offsets() {
        let f = frame(1, -2, 3, -4, 5, -6);
        let r = decode(f.as_slice().try_into().unwrap());
        assert_eq!(r.gyro, [1, -2, 3]);
        assert_eq!(r.accel, [-4, 5, -6]);
    }

    #[test]
    fn decode_full_range() {
        let f = frame(-1, 32767, -32768, 0, -32768, 32767);
        let r = decode(f.as_slice().try_into().unwrap());
        assert_eq!(r.gyro, [-1, 32767, -32768]);
        assert_eq!(r.accel, [0, -32768, 32767]);
    }

    #[test]
    fn scaling() {
        let r = RawFrame {
            gyro: [14286, -14286, 0],
            accel: [2048, -1024, 0],
        };
        let g = r.gyro_dps();
        assert!((g[0] - 1000.0).abs() < 1e-3 && (g[1] + 1000.0).abs() < 1e-3 && g[2] == 0.0);
        assert_eq!(r.accel_g(), [1.0, -0.5, 0.0]);
    }

    #[test]
    fn single_clean_frame() {
        let mut p = FrameParser::new();
        let out = p.push(&frame(10, 20, 30, 40, 50, 60));
        assert_eq!(
            out,
            [RawFrame {
                gyro: [10, 20, 30],
                accel: [40, 50, 60]
            }]
        );
        assert_eq!(p.buffered(), 0);
    }

    #[test]
    fn merged_frames_in_one_chunk() {
        let mut p = FrameParser::new();
        let chunk = [gx(1), gx(2), gx(3)].concat();
        assert_eq!(gxs(&p.push(&chunk)), [1, 2, 3]);
    }

    #[test]
    fn frame_split_across_chunks() {
        let mut p = FrameParser::new();
        let f = gx(123);
        assert!(p.push(&f[..8]).is_empty());
        assert_eq!(gxs(&p.push(&f[8..])), [123]);
    }

    #[test]
    fn frame_split_byte_by_byte() {
        let mut p = FrameParser::new();
        let data = [gx(1), gx(2)].concat();
        let mut got = Vec::new();
        for b in &data {
            got.extend(p.push(std::slice::from_ref(b)));
        }
        assert_eq!(gxs(&got), [1, 2]);
    }

    #[test]
    fn split_and_merged_mix() {
        // Two frames and a half in one chunk, the rest in the next.
        let mut p = FrameParser::new();
        let data = [gx(1), gx(2), gx(3)].concat();
        assert_eq!(gxs(&p.push(&data[..35])), [1, 2]);
        assert_eq!(gxs(&p.push(&data[35..])), [3]);
    }

    #[test]
    fn leading_garbage_is_skipped() {
        let mut p = FrameParser::new();
        let chunk = [vec![0x00, 0x99, 0x11], gx(55)].concat();
        assert_eq!(gxs(&p.push(&chunk)), [55]);
    }

    #[test]
    fn non_header_0x22_is_ignored() {
        let mut p = FrameParser::new();
        let chunk = [vec![0x22, 0x01], gx(77)].concat();
        assert_eq!(gxs(&p.push(&chunk)), [77]);
    }

    #[test]
    fn lone_trailing_0x22_is_retained() {
        let mut p = FrameParser::new();
        let f2 = gx(2);
        let chunk = [gx(1), f2[..1].to_vec()].concat();
        assert_eq!(gxs(&p.push(&chunk)), [1]);
        assert_eq!(p.buffered(), 1);
        assert_eq!(gxs(&p.push(&f2[1..])), [2]);
    }

    #[test]
    fn header_split_across_chunks_after_garbage() {
        let mut p = FrameParser::new();
        let f = gx(9);
        assert!(p.push(&[0x11, 0x12, 0x22]).is_empty());
        assert_eq!(gxs(&p.push(&f[1..])), [9]);
    }

    #[test]
    fn long_garbage_is_trimmed_to_last_byte() {
        let mut p = FrameParser::new();
        let mut garbage = vec![0x11; 64];
        garbage.push(0x22);
        assert!(p.push(&garbage).is_empty());
        assert_eq!(p.buffered(), 1);
        assert_eq!(gxs(&p.push(&gx(99)[1..])), [99]);
    }

    #[test]
    fn short_garbage_is_kept_until_header() {
        let mut p = FrameParser::new();
        assert!(p.push(&[0x11; 10]).is_empty());
        assert_eq!(p.buffered(), 10);
        assert_eq!(gxs(&p.push(&gx(5))), [5]);
        assert_eq!(p.buffered(), 0);
    }

    #[test]
    fn reset_drops_partial_frame() {
        let mut p = FrameParser::new();
        let f = gx(42);
        assert!(p.push(&f[..8]).is_empty());
        p.reset();
        assert!(p.push(&f[8..]).is_empty());
        // The leftover tail has no header and may stay buffered; a whole frame still parses.
        assert_eq!(gxs(&p.push(&gx(43))), [43]);
    }

    #[test]
    fn decoder_scales_and_numbers_frames() {
        let mut d = ImuDecoder::new(104, 0);
        let out = d.push(&[frame(14286, 0, 0, 0, 0, 2048), frame(0, 0, 0, 0, 2048, 0)].concat());
        assert_eq!(out.len(), 2);
        assert_eq!((out[0].index, out[0].t), (0, 0.0));
        assert!((out[0].gyro_dps[0] - 1000.0).abs() < 1e-3);
        assert_eq!(out[0].accel_g, [0.0, 0.0, 1.0]);
        assert_eq!(out[1].index, 1);
        assert!((out[1].t - 1.0 / 104.0).abs() < 1e-12);
        assert_eq!(out[1].accel_g, [0.0, 1.0, 0.0]);
    }

    #[test]
    fn decoder_time_from_index_not_arrival() {
        // 52 frames at 52 Hz delivered in uneven bursts: t depends only on the index.
        let mut d = ImuDecoder::new(52, 0);
        let data: Vec<u8> = (0..52).flat_map(gx).collect();
        let mut out = Vec::new();
        for chunk in data.chunks(37) {
            out.extend(d.push(chunk));
        }
        assert_eq!(out.len(), 52);
        for (i, f) in out.iter().enumerate() {
            assert_eq!(f.index, i as u64);
            assert!((f.t - i as f64 / 52.0).abs() < 1e-12);
            assert_eq!(f.gyro_dps[0], i as f32 / GYRO_LSB_PER_DPS);
        }
        assert!((out[51].t - 51.0 / 52.0).abs() < 1e-12);
    }

    #[test]
    fn decoder_drops_initial_frames() {
        let mut d = ImuDecoder::new(26, DEFAULT_SKIP_FRAMES);
        let data: Vec<u8> = (0..25).flat_map(gx).collect();
        // First 15 frames: all dropped.
        assert!(d.push(&data[..15 * FRAME_LEN]).is_empty());
        // Next 10: frames 15..20 dropped, 20..25 emitted with index 0..5.
        let out = d.push(&data[15 * FRAME_LEN..]);
        assert_eq!(out.len(), 5);
        for (i, f) in out.iter().enumerate() {
            assert_eq!(f.index, i as u64);
            assert_eq!(f.t, i as f64 / 26.0);
            assert_eq!(f.gyro_dps[0], (20 + i) as f32 / GYRO_LSB_PER_DPS);
        }
    }

    #[test]
    fn decoder_states_are_independent() {
        let mut a = ImuDecoder::new(104, 0);
        let mut b = ImuDecoder::new(104, 0);
        let f = gx(1);
        assert!(a.push(&f[..5]).is_empty());
        // b has no partial frame: the tail alone is garbage to it.
        assert!(b.push(&f[5..]).is_empty());
        assert_eq!(a.push(&f[5..]).len(), 1);
    }
}
