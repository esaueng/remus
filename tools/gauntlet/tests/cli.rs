//! Subprocess and output-contract tests for the gauntlet CLI.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use remus_gauntlet::{ModelResult, Scoreboard};
use remus_operations::primitives::make_box;
use remus_topology::Topology;

fn temp_dir(name: &str) -> PathBuf {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "remus-gauntlet-cli-{}-{nanos}-{name}",
        std::process::id()
    ))
}

fn write_box_step(path: &Path) {
    let mut topology = Topology::new();
    let solid = make_box(&mut topology, 10.0, 8.0, 6.0).unwrap();
    let step = remus_io::step::write_step(&topology, &[solid]).unwrap();
    fs::write(path, step).unwrap();
}

#[test]
fn run_isolates_models_and_writes_all_outputs() {
    let root = temp_dir("outputs");
    fs::create_dir_all(&root).unwrap();
    let good = root.join("good.step");
    let bad = root.join("bad.step");
    let output = root.join("results");
    write_box_step(&good);
    fs::write(&bad, "not STEP").unwrap();

    let status = Command::new(env!("CARGO_BIN_EXE_remus-gauntlet"))
        .args(["run", "--output"])
        .arg(&output)
        .arg(&good)
        .arg(&bad)
        .status()
        .unwrap();
    assert!(status.success());

    let jsonl = fs::read_to_string(output.join("models.jsonl")).unwrap();
    let rows: Vec<ModelResult> = jsonl
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(rows.len(), 2);
    assert!(rows[0].passed, "{:#?}", rows[0]);
    assert!(!rows[1].passed);

    let scoreboard: Scoreboard =
        serde_json::from_slice(&fs::read(output.join("scoreboard.json")).unwrap()).unwrap();
    assert_eq!(scoreboard.models, 2);
    assert_eq!(scoreboard.passed, 1);
    assert_eq!(scoreboard.failed, 1);
    assert!(output.join("scoreboard.md").is_file());

    fs::remove_dir_all(&root).unwrap();
}

#[test]
fn zero_budget_kills_only_its_model_and_reports_resource_limit() {
    let root = temp_dir("timeout");
    fs::create_dir_all(&root).unwrap();
    let first = root.join("first.step");
    let second = root.join("second.step");
    let output = root.join("results");
    write_box_step(&first);
    write_box_step(&second);

    let status = Command::new(env!("CARGO_BIN_EXE_remus-gauntlet"))
        .args(["run", "--timeout-ms", "0", "--output"])
        .arg(&output)
        .arg(&first)
        .arg(&second)
        .status()
        .unwrap();
    assert!(status.success());

    let jsonl = fs::read_to_string(output.join("models.jsonl")).unwrap();
    let rows: Vec<ModelResult> = jsonl
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(rows.len(), 2);
    for row in rows {
        assert!(!row.passed);
        assert_eq!(row.stages.read.diagnostics[0].category, "resource_limit");
        assert_eq!(
            row.stages.read.diagnostics[0].code,
            "model_wall_clock_budget_exceeded"
        );
    }

    fs::remove_dir_all(&root).unwrap();
}
