//! Preset banks: the built-in JSON, round-trips, validation errors, custom presets.

use serde_json::{Value, json};
use shariki_core::MotionState;
use shariki_engine::render::{self, ShakeGesture, stats};
use shariki_engine::{
    BankError, Body, DEFAULT_FLOW_DECAY, DEFAULT_SHAKE_DECAY, PresetBank, Resonance, Shaker,
    ShakerModel, builtin_json, catalog,
};

const SR: f32 = 48_000.0;
const BUILTIN_IDS: [&str; 6] = ["maracas", "cabasa", "shekere", "egg", "ganza", "rainstick"];

/// A small valid preset (a hosho-like gourd rattle), as JSON.
fn hosho() -> Value {
    json!({
        "id": "hosho",
        "name": "Hosho",
        "description": "Pair of Shona gourd rattles filled with hota seeds",
        "source": "variant of the maracas preset: bigger gourd, more seeds",
        "params": { "beads": 600, "decay": 0.03 },
        "sound_decay": 0.94,
        "resonances": [
            { "freq": 2400, "radius": 0.95, "gain": 1.0, "vary": 0.02 },
            { "freq": 4100, "radius": 0.8, "gain": 0.4 }
        ],
        "body": "highpass",
        "level": 5.0
    })
}

fn bank_json(presets: Vec<Value>) -> String {
    json!({ "presets": presets }).to_string()
}

/// Parses a bank containing only `preset` and returns the error message.
fn error_for(preset: Value) -> String {
    match PresetBank::from_json(&bank_json(vec![preset])) {
        Ok(_) => panic!("expected an error"),
        Err(e) => e.to_string(),
    }
}

/// `hosho()` with `pointer` (JSON pointer) set to `value` (object keys are added if missing).
fn hosho_with(pointer: &str, value: Value) -> Value {
    let mut p = hosho();
    let (parent, key) = pointer.rsplit_once('/').unwrap();
    match p
        .pointer_mut(parent)
        .unwrap_or_else(|| panic!("no {parent}"))
    {
        Value::Object(map) => {
            map.insert(key.to_string(), value);
        }
        Value::Array(items) => items[key.parse::<usize>().unwrap()] = value,
        other => panic!("{parent} is {other}"),
    }
    p
}

fn assert_mentions(msg: &str, parts: &[&str]) {
    for part in parts {
        assert!(msg.contains(part), "`{msg}` does not mention `{part}`");
    }
}

#[test]
fn builtin_bank_parses_with_the_six_presets_in_order() {
    let bank = PresetBank::builtin();
    let ids: Vec<&str> = bank.presets().iter().map(|m| m.id.as_str()).collect();
    assert_eq!(ids, BUILTIN_IDS);
    let cat: Vec<&str> = catalog().iter().map(|i| i.id).collect();
    assert_eq!(cat, BUILTIN_IDS);
    for (info, m) in catalog().iter().zip(bank.presets()) {
        assert_eq!((info.name, info.description), (&*m.name, &*m.description));
        assert!(
            m.source.as_deref().is_some_and(|s| !s.is_empty()),
            "{}",
            m.id
        );
    }
    // The embedded text is exactly what the bank was parsed from.
    assert_eq!(&PresetBank::from_json(builtin_json()).unwrap(), bank);
    assert!(bank.get("theremin").is_none());
    assert!(bank.create("theremin", SR).is_none());

    let rain = bank.get("rainstick").unwrap();
    assert_eq!(rain.flow_shake_tau, Some(0.03));
    assert_eq!(rain.resonances.len(), 4);
    assert_eq!(rain.body, Body::BandPass);
    let maracas = bank.get("maracas").unwrap();
    assert_eq!(maracas.params["beads"], 400.0);
    assert_eq!(maracas.params["decay"], DEFAULT_SHAKE_DECAY);
    assert_eq!(bank.get("cabasa").unwrap().params["beads"], 3000.0);
}

#[test]
fn to_json_round_trips() {
    let bank = PresetBank::builtin();
    let text = bank.to_json();
    let again = PresetBank::from_json(&text).unwrap();
    assert_eq!(&again, bank);
    assert_eq!(again.to_json(), text);
    // Params are written in canonical order, not alphabetically.
    let beads = text.find("\"beads\"").unwrap();
    let decay = text.find("\"decay\"").unwrap();
    let damping = text.find("\"damping\"").unwrap();
    assert!(beads < decay && decay < damping);

    // A bank with optional fields left out round-trips too.
    let mut minimal = hosho();
    let obj = minimal.as_object_mut().unwrap();
    obj.remove("params");
    obj.remove("source");
    let custom =
        PresetBank::from_json(&bank_json(vec![minimal, hosho_with("/id", json!("h2"))])).unwrap();
    assert_eq!(PresetBank::from_json(&custom.to_json()).unwrap(), custom);

    // Models built in code go through the same validation.
    let built = PresetBank::new(custom.presets().to_vec()).unwrap();
    assert_eq!(built, custom);
}

#[test]
fn missing_params_use_global_defaults() {
    let mut p = hosho();
    p.as_object_mut().unwrap().remove("params");
    let bank = PresetBank::from_json(&bank_json(vec![p])).unwrap();
    let inst = bank.create("hosho", SR).unwrap();
    for spec in shariki_engine::base_param_specs(false) {
        assert_eq!(inst.param(spec.id), Some(spec.default), "{}", spec.id);
    }
    assert_eq!(inst.param("decay"), Some(DEFAULT_SHAKE_DECAY));
    assert_eq!(inst.param("beads"), Some(400.0));

    // A flow model without `decay` gets the flow default and the flow range.
    let mut p = hosho();
    p.as_object_mut().unwrap().remove("params");
    p["flow_shake_tau"] = json!(0.05);
    let bank = PresetBank::from_json(&bank_json(vec![p])).unwrap();
    let inst = bank.create("hosho", SR).unwrap();
    assert_eq!(inst.param("decay"), Some(DEFAULT_FLOW_DECAY));
    let spec = inst.params().iter().find(|s| s.id == "decay").unwrap();
    assert_eq!((spec.min, spec.max), (0.3, 8.0));

    // Partial params: the given ones win, the rest default.
    let bank = PresetBank::from_json(&bank_json(vec![hosho()])).unwrap();
    let inst = bank.create("hosho", SR).unwrap();
    assert_eq!(inst.param("beads"), Some(600.0));
    assert_eq!(inst.param("decay"), Some(0.03));
    assert_eq!(inst.param("gain"), Some(1.0));
}

#[test]
fn custom_json_preset_renders_sound() {
    let bank = PresetBank::from_json(&bank_json(vec![hosho()])).unwrap();
    let mut inst = bank.create("hosho", SR).unwrap();
    assert_eq!((inst.id(), inst.name()), ("hosho", "Hosho"));
    let g = ShakeGesture::default();
    let out = render::render(inst.as_mut(), &g.states(), SR, g.duration(), 256, 4);
    let busy = stats(&out[(0.5 * SR) as usize..(3.0 * SR) as usize]);
    assert!(busy.all_finite && busy.rms > 0.01, "rms {}", busy.rms);
    assert!(busy.peak <= 1.0);
    assert!(out[(3.3 * SR) as usize..].iter().all(|&s| s == 0.0));

    // Silence in, silence out.
    let mut inst = bank.create("hosho", SR).unwrap();
    let out = render::render_constant(inst.as_mut(), MotionState::default(), SR, 0.5, 256);
    assert!(out.iter().all(|&s| s == 0.0));

    // Same model through `Shaker::from_model` with an explicit seed.
    let model = bank.get("hosho").unwrap();
    let mut s = Shaker::from_model(model, SR, 7).unwrap();
    assert_eq!(s.model(), model);
    let out = render::render(&mut s, &g.states(), SR, g.duration(), 256, 4);
    assert!(stats(&out).rms > 0.01);
}

#[test]
fn from_model_validates() {
    let model: ShakerModel = serde_json::from_value(hosho()).unwrap();
    assert!(Shaker::from_model(&model, SR, 1).is_ok());

    let mut bad = model.clone();
    bad.resonances = vec![
        Resonance {
            freq: 1000.0,
            radius: 0.9,
            gain: 1.0,
            vary: 0.0,
        };
        5
    ];
    let e = Shaker::from_model(&bad, SR, 1).unwrap_err();
    assert_mentions(&e.to_string(), &["hosho", "resonances", "5"]);
    assert!(matches!(e, BankError::Invalid { .. }));

    let mut bad = model.clone();
    bad.level = f32::NAN;
    let e = Shaker::from_model(&bad, SR, 1).unwrap_err().to_string();
    assert_mentions(&e, &["hosho", "level", "finite"]);
    assert!(bad.validate().is_err());

    let mut bad = model;
    bad.params.insert("decay".into(), f32::INFINITY);
    let e = Shaker::from_model(&bad, SR, 1).unwrap_err().to_string();
    assert_mentions(&e, &["hosho", "params.decay", "finite"]);
}

#[test]
fn syntax_errors_report_line_and_column() {
    let e =
        PresetBank::from_json("{\n  \"presets\": [\n    { \"id\": \"x\", }\n  ]\n}").unwrap_err();
    let BankError::Json { line, column, .. } = &e else {
        panic!("{e:?}");
    };
    assert_eq!(*line, 3);
    assert!(*column > 0);
    assert_mentions(&e.to_string(), &["line 3, column"]);
    // The message does not repeat serde's own " at line N column M" suffix.
    assert!(!e.to_string().contains(" at line "), "{e}");

    // Structural errors: unknown field, wrong type, unknown body, missing field.
    let mut p = hosho();
    p["sound_decy"] = json!(0.9);
    assert_mentions(
        &error_for(p),
        &["unknown field `sound_decy`", "line 1, column"],
    );
    assert_mentions(
        &error_for(hosho_with("/level", json!("loud"))),
        &["invalid type", "line 1"],
    );
    assert_mentions(
        &error_for(hosho_with("/body", json!("lowpass"))),
        &["lowpass", "highpass", "bandpass"],
    );
    let mut p = hosho();
    p.as_object_mut().unwrap().remove("sound_decay");
    assert_mentions(&error_for(p), &["missing field `sound_decay`"]);
    let mut p = hosho();
    p["resonances"][0]["q"] = json!(3);
    assert_mentions(&error_for(p), &["unknown field `q`"]);
}

#[test]
fn bank_level_errors() {
    let e = PresetBank::from_json(r#"{ "presets": [] }"#).unwrap_err();
    assert!(matches!(e, BankError::Empty { .. }));
    assert_mentions(&e.to_string(), &["no presets"]);

    let e = PresetBank::from_json(&bank_json(vec![
        hosho(),
        hosho_with("/id", json!("other")),
        hosho(),
    ]))
    .unwrap_err();
    assert_mentions(&e.to_string(), &["duplicate preset id `hosho`", "#1", "#3"]);

    assert_mentions(
        &error_for(hosho_with("/id", json!(""))),
        &["`#1`", "`id`", "empty"],
    );
    assert_mentions(
        &error_for(hosho_with("/id", json!("my hosho"))),
        &["`my hosho`", "`id`", "whitespace"],
    );
    assert_mentions(
        &error_for(hosho_with("/name", json!(" "))),
        &["hosho", "`name`"],
    );
}

#[test]
fn field_errors_name_preset_and_field() {
    let cases: Vec<(&str, Value, &[&str])> = vec![
        (
            "/resonances",
            json!([]),
            &["resonances", "0 entries", "1 to 4"],
        ),
        (
            "/resonances",
            json!(vec![json!({ "freq": 1000, "radius": 0.9 }); 5]),
            &["resonances", "5 entries", "1 to 4"],
        ),
        (
            "/resonances/0/radius",
            json!(1.0),
            &["resonances[0].radius", "[0, 1)"],
        ),
        (
            "/resonances/1/radius",
            json!(-0.1),
            &["resonances[1].radius", "[0, 1)"],
        ),
        (
            "/resonances/0/freq",
            json!(0),
            &["resonances[0].freq", "(0, 20000]"],
        ),
        (
            "/resonances/1/freq",
            json!(25000),
            &["resonances[1].freq", "(0, 20000]"],
        ),
        (
            "/resonances/0/freq",
            json!(1e39),
            &["resonances[0].freq", "finite"],
        ),
        (
            "/resonances/0/gain",
            json!(-1),
            &["resonances[0].gain", "≥ 0"],
        ),
        (
            "/resonances/0/vary",
            json!(1.5),
            &["resonances[0].vary", "[0, 1)"],
        ),
        ("/sound_decay", json!(1.0), &["`sound_decay`", "(0, 1)"]),
        ("/sound_decay", json!(0), &["`sound_decay`", "(0, 1)"]),
        ("/sound_decay", json!(-1e39), &["`sound_decay`", "finite"]),
        ("/level", json!(0), &["`level`", "> 0"]),
        ("/level", json!(-2.5), &["`level`", "> 0"]),
        ("/level", json!(1e39), &["`level`", "finite"]),
        ("/flow_shake_tau", json!(0), &["`flow_shake_tau`", "> 0"]),
        ("/flow_shake_tau", json!(-0.1), &["`flow_shake_tau`", "> 0"]),
        ("/params/beads", json!(0), &["params.beads", "[1, 8192]"]),
        (
            "/params/beads",
            json!(10000),
            &["params.beads", "[1, 8192]"],
        ),
        (
            "/params/decay",
            json!(2.0),
            &["params.decay", "[0.005, 1]", "flow_shake_tau"],
        ),
        ("/params/decay", json!(1e39), &["params.decay", "finite"]),
        ("/params/gain", json!(3), &["params.gain", "[0, 2]"]),
        (
            "/params",
            json!({ "beads": 10, "cowbell": 1 }),
            &[
                "params.cowbell",
                "unknown parameter `cowbell`",
                "beads, decay",
            ],
        ),
    ];
    for (pointer, value, parts) in cases {
        let msg = error_for(hosho_with(pointer, value.clone()));
        assert_mentions(&msg, &["preset `hosho`"]);
        assert_mentions(&msg, parts);
    }

    // `decay` range follows `flow_shake_tau`: 2 s is fine for a flow model, 0.1 s is not.
    let mut p = hosho_with("/params/decay", json!(2.0));
    p["flow_shake_tau"] = json!(0.03);
    assert!(PresetBank::from_json(&bank_json(vec![p.clone()])).is_ok());
    p["params"]["decay"] = json!(0.1);
    assert_mentions(
        &error_for(p),
        &["hosho", "params.decay", "[0.3, 8]", "grain-flow"],
    );
}

#[test]
fn load_names_the_file() {
    let dir = std::env::temp_dir().join(format!("shariki-bank-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let good = dir.join("good.json");
    std::fs::write(&good, bank_json(vec![hosho()])).unwrap();
    let bank = PresetBank::load(&good).unwrap();
    assert_eq!(bank.presets().len(), 1);

    let bad = dir.join("bad.json");
    std::fs::write(&bad, bank_json(vec![hosho_with("/level", json!(0))])).unwrap();
    let msg = PresetBank::load(&bad).unwrap_err().to_string();
    assert_mentions(&msg, &["bad.json: ", "preset `hosho`", "`level`"]);

    let broken = dir.join("broken.json");
    std::fs::write(&broken, "{\n  \"presets\": [\n    oops\n  ]\n}\n").unwrap();
    let msg = PresetBank::load(&broken).unwrap_err().to_string();
    assert_mentions(&msg, &["broken.json:3:"]);

    let missing = dir.join("missing.json");
    let e = PresetBank::load(&missing).unwrap_err();
    assert!(matches!(e, BankError::Io { .. }));
    assert_mentions(&e.to_string(), &["missing.json"]);
    assert!(std::error::Error::source(&e).is_some());

    std::fs::remove_dir_all(&dir).unwrap();
}
