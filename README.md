# shariki

![A hand shaking the Triki token, surrounded by maracas, cabasa, shekere, egg shaker, ganzá and a rainstick](assets/ai-gen/hero.jpg)

Digital shaker instruments — maracas, cabasa, shekere, egg shaker, ganzá, rainstick —
played by shaking a [Żabka Triki](https://github.com/Flopsstuff/triki) BLE token.
The sound is synthesised in real time with a physically informed stochastic model
(PhISEM, after Perry Cook's STK `Shakers`), so it reacts to how hard and how sharply
you shake.

```
token ──BLE──▶ triki-ble ──ImuFrame──▶ shariki-motion ──MotionState──▶ shariki-engine ──▶ cpal
```

| Crate | Role |
|---|---|
| `crates/core` (`shariki-core`) | Shared types: `ImuFrame`, `MotionState`, the `Instrument` trait |
| `crates/triki` (`triki-ble`) | BLE driver for the token (btleplug): scan, connect, IMU stream |
| `crates/motion` (`shariki-motion`) | IMU → gesture features (shake energy, hits, tilt) + synthetic gestures |
| `crates/engine` (`shariki-engine`) | Sound engines; PhISEM shakers |
| `crates/shariki` | The CLI |

## Usage

Needs a Rust toolchain (`rustup`).

```sh
cargo build --release
B=./target/release/shariki

$B list                          # presets and their parameters
$B scan                          # find tokens nearby
$B play                          # connect to the first token, start with the first preset
$B play cabasa -p size=1.4       # start with a given preset, override a parameter
$B play --demo                   # no token: synthetic gesture
$B render ganza -o ganza.wav     # demo gesture → WAV, no audio device needed
```

While playing: **↑ / ↓** switch preset (30 ms crossfade), **r** reload the preset file,
**q** / Esc / Ctrl-C quit. The IMU runs at 416 Hz by default (`--rate` 26–416).

### Presets

Every preset is defined in JSON — the PhISEM model (resonances, click decay, body filter,
level) and the default parameter values — so new instruments need no recompiling.
shariki uses the first of: `--presets FILE`, `./presets.json`,
`~/.config/shariki/presets.json`, the built-in bank (`crates/engine/presets.json`).

```sh
$B presets export      # write the built-in bank to ./presets.json to edit
$B presets where       # which file is in use
```

Edit `presets.json` while `play` is running and press **r** to hear the change. The
order of presets in the file is the order ↑/↓ cycles through. Invalid files are rejected
with the file, line and field at fault.

### macOS Bluetooth permission

macOS attributes Bluetooth access to the *responsible* app — normally the terminal — and
kills the process (`Abort trap: 6`) if that app's Info.plist lacks
`NSBluetoothAlwaysUsageDescription` (Warp and Terminal.app both lack it). So for `scan` and
`play`, shariki re-spawns itself as its own responsible process
(`crates/shariki/src/macos.rs`), and macOS reads the Info.plist embedded in the binary
(`crates/shariki/build.rs`). The first run asks for Bluetooth access for shariki itself.

### Linux

Needs BlueZ (`bluetoothd`) and the libdbus development package to build
(`libdbus-1-dev` on Debian/Raspberry Pi OS), plus ALSA headers (`libasound2-dev`).

## License

[MIT](LICENSE). The PhISEM shaker model is ported from the Synthesis ToolKit (STK) by
Perry R. Cook and Gary P. Scavone — see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
