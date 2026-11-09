use shariki_core::{Instrument, MotionState, ParamError};
use shariki_engine::render::{self, ShakeGesture, stats};
use shariki_engine::{Shaker, catalog, create};

const SR: f32 = 48_000.0;

fn ids() -> impl Iterator<Item = &'static str> {
    catalog().iter().map(|i| i.id)
}

fn shake(inst: &mut dyn Instrument, block: usize) -> Vec<f32> {
    let g = ShakeGesture::default();
    render::render(inst, &g.states(), SR, g.duration(), block, 4)
}

fn secs(t: f32) -> usize {
    (t * SR) as usize
}

fn zero_crossing_rate(x: &[f32]) -> f32 {
    let n = x
        .windows(2)
        .filter(|w| (w[0] >= 0.0) != (w[1] >= 0.0))
        .count();
    n as f32 / x.len() as f32
}

#[test]
fn every_catalog_instrument_constructs() {
    assert!(catalog().len() >= 5);
    for (i, info) in catalog().iter().enumerate() {
        assert!(
            catalog()[..i].iter().all(|o| o.id != info.id),
            "duplicate id {}",
            info.id
        );
        let inst = create(info.id, SR).unwrap_or_else(|| panic!("create({})", info.id));
        assert_eq!(inst.id(), info.id);
        assert_eq!(inst.name(), info.name);
        assert!(!info.description.is_empty());
        let params = inst.params();
        for id in ["beads", "decay", "damping", "size", "sensitivity", "gain"] {
            let spec = params
                .iter()
                .find(|p| p.id == id)
                .unwrap_or_else(|| panic!("{}: {id}", info.id));
            assert!(spec.min < spec.max && (spec.min..=spec.max).contains(&spec.default));
            assert_eq!(inst.param(id), Some(spec.default));
        }
    }
    for id in ["maracas", "cabasa", "shekere", "egg", "ganza"] {
        assert!(create(id, SR).is_some(), "{id}");
    }
    assert!(create("theremin", SR).is_none());
}

#[test]
fn silence_in_gives_exact_silence_out() {
    for id in ids() {
        let mut inst = create(id, SR).unwrap();
        let out = render::render_constant(inst.as_mut(), MotionState::default(), SR, 1.0, 256);
        assert!(out.iter().all(|&s| s == 0.0), "{id} not silent");
        // Sensor noise below the floor is silence too.
        let noisy = MotionState {
            energy: 0.02,
            ..Default::default()
        };
        let out = render::render_constant(inst.as_mut(), noisy, SR, 1.0, 256);
        assert!(out.iter().all(|&s| s == 0.0), "{id} hisses at rest");
    }
}

#[test]
fn shake_is_audible_bounded_and_decays_to_true_silence() {
    for id in ids() {
        let mut inst = create(id, SR).unwrap();
        let out = shake(inst.as_mut(), 256);
        let still = stats(&out[..secs(0.45)]);
        let busy = stats(&out[secs(0.5)..secs(3.0)]);
        // The gesture stops at 3.0 s; within 0.3 s the output must be exactly zero.
        let tail = &out[secs(3.3)..];
        assert_eq!(still.peak, 0.0, "{id}: sound before the shake");
        assert!(busy.all_finite);
        assert!(busy.rms > 0.01, "{id}: shake too quiet, rms {}", busy.rms);
        assert!(
            busy.peak > 0.2 && busy.peak < 1.0,
            "{id}: peak {}",
            busy.peak
        );
        assert!(tail.iter().all(|&s| s == 0.0), "{id}: residual after shake");
    }
}

#[test]
fn more_energy_is_louder() {
    for id in ids() {
        let rms_at = |energy: f32| {
            let mut inst = create(id, SR).unwrap();
            let m = MotionState {
                energy,
                ..Default::default()
            };
            let out = render::render_constant(inst.as_mut(), m, SR, 2.0, 256);
            stats(&out[secs(0.5)..]).rms
        };
        let (soft, medium, hard) = (rms_at(0.2), rms_at(0.5), rms_at(1.0));
        assert!(soft > 0.0, "{id}: gentle shake silent");
        assert!(
            medium > soft * 1.3 && hard > medium * 1.3,
            "{id}: {soft} {medium} {hard}"
        );

        // Impulses add accents on top of the continuous rattle.
        let mut inst = create(id, SR).unwrap();
        let g = ShakeGesture {
            peak_energy: 0.3,
            ..Default::default()
        };
        let weak = stats(&render::render(
            inst.as_mut(),
            &g.states(),
            SR,
            g.duration(),
            256,
            1,
        ))
        .rms;
        let mut inst = create(id, SR).unwrap();
        let g = ShakeGesture {
            peak_energy: 1.0,
            ..Default::default()
        };
        let strong = stats(&render::render(
            inst.as_mut(),
            &g.states(),
            SR,
            g.duration(),
            256,
            1,
        ))
        .rms;
        assert!(strong > weak * 1.5, "{id}: {weak} vs {strong}");
    }
}

fn extreme_states() -> Vec<MotionState> {
    let mut v = Vec::new();
    let weird = [
        0.0,
        1.0,
        1.0e6,
        f32::MAX,
        f32::INFINITY,
        f32::NAN,
        -5.0,
        f32::NEG_INFINITY,
    ];
    for (i, &e) in weird.iter().enumerate() {
        for (j, &imp) in weird.iter().enumerate() {
            v.push(MotionState {
                t: (v.len()) as f64 / 104.0,
                energy: e,
                impulse: imp,
                up: [weird[(i + j) % weird.len()], 0.0, weird[j]],
                angular_speed: e,
                linear_accel: imp,
            });
        }
    }
    v
}

fn assert_bounded(id: &str, what: &str, out: &[f32]) {
    for &s in out {
        assert!(s.is_finite() && s.abs() <= 1.0, "{id} ({what}): sample {s}");
    }
}

#[test]
fn extreme_inputs_and_parameters_stay_finite_and_bounded() {
    let states = extreme_states();
    let duration = states.len() as f32 / 104.0 + 0.5;
    for id in ids() {
        let specs = create(id, SR).unwrap().params().to_vec();
        // All params at min, all at max, and each param alone at min / max.
        let mut configs: Vec<Vec<(&str, f32)>> = vec![
            specs.iter().map(|p| (p.id, p.min)).collect(),
            specs.iter().map(|p| (p.id, p.max)).collect(),
        ];
        for p in &specs {
            configs.push(vec![(p.id, p.min)]);
            configs.push(vec![(p.id, p.max)]);
        }
        for cfg in &configs {
            for sr in [8_000.0, 48_000.0, 192_000.0] {
                let mut inst = create(id, sr).unwrap();
                for &(pid, v) in cfg {
                    inst.set_param(pid, v).unwrap();
                }
                let out = render::render(inst.as_mut(), &states, sr, duration, 97, 3);
                assert_bounded(id, &format!("{cfg:?} @ {sr}"), &out);
                // Then a vigorous normal shake still works (no stuck NaN state).
                let g = ShakeGesture::default();
                let out = render::render(inst.as_mut(), &g.states(), sr, g.duration(), 256, 4);
                assert_bounded(id, &format!("{cfg:?} @ {sr} after"), &out);
            }
        }
    }
}

#[test]
fn set_param_validates() {
    for id in ids() {
        let mut inst = create(id, SR).unwrap();
        assert_eq!(
            inst.set_param("cowbell", 1.0),
            Err(ParamError::Unknown("cowbell".into()))
        );
        assert_eq!(inst.param("cowbell"), None);
        for spec in inst.params().to_vec() {
            let below = spec.min - 0.01 * (spec.max - spec.min);
            let above = spec.max * 1.01 + 0.01;
            assert!(matches!(
                inst.set_param(spec.id, below),
                Err(ParamError::OutOfRange { .. })
            ));
            assert!(matches!(
                inst.set_param(spec.id, above),
                Err(ParamError::OutOfRange { .. })
            ));
            assert!(inst.set_param(spec.id, f32::NAN).is_err());
            assert_eq!(
                inst.param(spec.id),
                Some(spec.default),
                "rejected value must not stick"
            );
            inst.set_param(spec.id, spec.min).unwrap();
            assert_eq!(inst.param(spec.id), Some(spec.min));
            inst.set_param(spec.id, spec.max).unwrap();
            assert_eq!(inst.param(spec.id), Some(spec.max));
            inst.set_param(spec.id, spec.default).unwrap();
        }
    }
}

#[test]
fn block_size_does_not_matter_much() {
    for id in ids() {
        let rms: Vec<f32> = [1usize, 64, 512]
            .iter()
            .map(|&b| {
                let mut inst = create(id, SR).unwrap();
                let out = shake(inst.as_mut(), b);
                let s = stats(&out);
                assert!(s.all_finite && s.peak <= 1.0, "{id} block {b}");
                assert!(
                    out[secs(3.3)..].iter().all(|&x| x == 0.0),
                    "{id} block {b} tail"
                );
                s.rms
            })
            .collect();
        let (lo, hi) = rms
            .iter()
            .fold((f32::MAX, 0.0f32), |(l, h), &r| (l.min(r), h.max(r)));
        assert!(
            lo > 0.01 && hi / lo < 1.5,
            "{id}: rms by block size {rms:?}"
        );
    }
}

#[test]
fn sample_rate_does_not_change_level_much() {
    for id in ids() {
        let rms: Vec<f32> = [22_050.0f32, 44_100.0, 96_000.0]
            .iter()
            .map(|&sr| {
                let mut inst = create(id, sr).unwrap();
                let g = ShakeGesture::default();
                stats(&render::render(
                    inst.as_mut(),
                    &g.states(),
                    sr,
                    g.duration(),
                    256,
                    4,
                ))
                .rms
            })
            .collect();
        let (lo, hi) = rms
            .iter()
            .fold((f32::MAX, 0.0f32), |(l, h), &r| (l.min(r), h.max(r)));
        assert!(hi / lo < 1.4, "{id}: rms by sample rate {rms:?}");
    }
}

#[test]
fn deterministic_for_same_seed() {
    for id in ids() {
        let a = shake(create(id, SR).unwrap().as_mut(), 128);
        let b = shake(create(id, SR).unwrap().as_mut(), 128);
        assert_eq!(a, b, "{id}");
        let mut s1 = Shaker::with_seed(id, SR, 1).unwrap();
        let mut s2 = Shaker::with_seed(id, SR, 2).unwrap();
        let c = shake(&mut s1, 128);
        assert_ne!(c, shake(&mut s2, 128), "{id}: seed ignored");
        s1.reset();
        assert_eq!(c, shake(&mut s1, 128), "{id}: reset is not a full reset");
    }
}

#[test]
fn size_lowers_and_damping_shortens() {
    let render_with = |param: &str, value: f32| {
        let mut inst = create("maracas", SR).unwrap();
        inst.set_param(param, value).unwrap();
        let m = MotionState {
            energy: 1.0,
            ..Default::default()
        };
        render::render_constant(inst.as_mut(), m, SR, 2.0, 256)
    };
    let small = zero_crossing_rate(&render_with("size", 0.5));
    let big = zero_crossing_rate(&render_with("size", 2.0));
    assert!(small > big * 1.5, "size: zcr {small} vs {big}");
    // `damping` changes the click length, but is level-compensated.
    let dry = stats(&render_with("damping", 4.0)).rms;
    let wet = stats(&render_with("damping", 0.25)).rms;
    assert!(
        dry > 0.0 && wet > 0.0 && (dry / wet) < 3.0 && (wet / dry) < 3.0,
        "{dry} {wet}"
    );
}

#[test]
fn sensitivity_scales_response() {
    let rms_with = |sens: f32| {
        let mut inst = create("cabasa", SR).unwrap();
        inst.set_param("sensitivity", sens).unwrap();
        let g = ShakeGesture {
            peak_energy: 0.4,
            ..Default::default()
        };
        stats(&render::render(
            inst.as_mut(),
            &g.states(),
            SR,
            g.duration(),
            256,
            4,
        ))
        .rms
    };
    assert!(rms_with(2.0) > rms_with(1.0) * 1.3);
    assert!(rms_with(0.5) * 1.3 < rms_with(1.0));
}

#[test]
fn rainstick_trickles_after_turning_over() {
    let mut inst = create("rainstick", SR).unwrap();
    let states = render::turn_over_gesture(0.5, 0.5, 5.0, render::MOTION_RATE);
    let out = render::render(inst.as_mut(), &states, SR, 6.0, 256, 4);
    let at = |t: f32| stats(&out[secs(t)..secs(t + 0.5)]).rms;
    assert_eq!(stats(&out[..secs(0.45)]).peak, 0.0);
    let (early, mid, late) = (at(1.0), at(3.0), at(5.0));
    assert!(early > 0.01, "early {early}");
    assert!(mid > 0.0 && late > 0.0, "stopped too soon: {mid} {late}");
    assert!(
        early > mid && mid > late,
        "flow should decay: {early} {mid} {late}"
    );

    // Holding it still (no turn) is silent.
    let mut inst = create("rainstick", SR).unwrap();
    let out = render::render_constant(inst.as_mut(), MotionState::default(), SR, 2.0, 256);
    assert!(out.iter().all(|&s| s == 0.0));
}
