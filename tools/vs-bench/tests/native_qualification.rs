//! Regression coverage for the native qualification runner.
//!
//! The end-to-end test executes the real kernel through the real runner
//! binary and judges the emitted job with the real scorecard. The mapping
//! tests feed synthetic evidence to the pure mapping functions — the only
//! fault injection in this file, and it never touches the production
//! execution path.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_vs_bench::qualification::{
    CaseEvidence, assemble_job, case_spec, case_specs, check_harness_sha, classify_status,
    execute_case, manifest_sha256, map_to_observation, mesh_watertight,
};
use remus_vs_bench::{Report, evaluate_json};
use serde_json::{Value, json};
use std::process::{Command, Stdio};

fn runner_bin() -> String {
    env!("CARGO_BIN_EXE_remus-native-qualification").to_string()
}

fn temp_paths(tag: &str) -> (String, String) {
    let dir = std::env::temp_dir().join(format!(
        "remus-native-qualification-{}-{}",
        std::process::id(),
        tag
    ));
    std::fs::create_dir_all(&dir).unwrap();
    (
        dir.join("job.json").to_string_lossy().into_owned(),
        dir.join("attempts.json").to_string_lossy().into_owned(),
    )
}

fn run_parent(extra: &[&str]) -> (i32, String) {
    run_parent_with_env(extra, &[])
}

fn run_parent_with_env(extra: &[&str], env: &[(&str, &str)]) -> (i32, String) {
    let mut command = Command::new(runner_bin());
    command
        .args(extra)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    for (key, value) in env {
        command.env(key, value);
    }
    let child = command.spawn().unwrap();
    let output = child.wait_with_output().unwrap();
    (
        output.status.code().unwrap(),
        String::from_utf8(output.stderr).unwrap(),
    )
}

fn is_hex(value: &str, len: usize) -> bool {
    value.len() == len && value.bytes().all(|b| b.is_ascii_hexdigit())
}

fn outcome_count(report: &Report, outcome: &str) -> usize {
    report
        .rows
        .iter()
        .filter(|r| r.outcomes[outcome] == 1)
        .count()
}

fn map_pair(case: &str, first: &CaseEvidence, second: &CaseEvidence) -> Value {
    let spec = case_spec(case).unwrap();
    map_to_observation(&spec, first, second, "test-kernel", 0.05, 0.06, "abc1234").unwrap()
}

fn assemble(cases: &[&str], observations: Vec<Value>) -> Value {
    let specs: Vec<CaseEvidence> = cases.iter().map(|c| execute_case(c).unwrap()).collect();
    let owned: Vec<_> = cases.iter().map(|c| case_spec(c).unwrap()).collect();
    let refs: Vec<(&remus_vs_bench::qualification::CaseSpec, &CaseEvidence)> =
        owned.iter().zip(specs.iter()).collect();
    assemble_job(
        &refs,
        observations,
        "test-run",
        &"a".repeat(40),
        &"b".repeat(64),
        "test-kernel",
        2,
    )
    .unwrap()
}

#[test]
fn end_to_end_real_run_passes_all_gates() {
    let (job_path, attempts_path) = temp_paths("e2e");
    let (code, _) = run_parent(&["--out", &job_path, "--attempts", &attempts_path]);
    assert_eq!(code, 0);
    let job: Value = serde_json::from_str(&std::fs::read_to_string(&job_path).unwrap()).unwrap();
    let attempts: Value =
        serde_json::from_str(&std::fs::read_to_string(&attempts_path).unwrap()).unwrap();
    // Six cases, one native kernel, two repetitions each.
    assert_eq!(job["scenarios"].as_array().unwrap().len(), 6);
    assert_eq!(job["observations"].as_array().unwrap().len(), 6);
    assert_eq!(job["repetitions"], json!(2));
    assert_eq!(job["kernels"].as_array().unwrap().len(), 1);
    assert!(is_hex(job["harness_sha"].as_str().unwrap(), 40));
    assert_eq!(
        job["manifest_sha256"].as_str().unwrap(),
        &manifest_sha256().unwrap()
    );
    assert!(
        job["kernels"][0]
            .as_str()
            .unwrap()
            .contains(job["harness_sha"].as_str().unwrap())
    );
    assert_eq!(attempts.as_array().unwrap().len(), 12);
    for attempt in attempts.as_array().unwrap() {
        assert_eq!(attempt["outcome"], json!("evidence"));
    }
    // The real scorecard judges the real job: every gate passes.
    let report = evaluate_json(&job.to_string()).unwrap();
    assert!(report.passed);
    assert_eq!(outcome_count(&report, "exact_success"), 5);
    assert_eq!(outcome_count(&report, "typed_refusal"), 1);
    for outcome in [
        "correct_success",
        "disclosed_approximate_success",
        "verified_repair_success",
        "untyped_error",
        "silent_wrong",
        "invalid_success",
        "crash",
        "hang_or_budget_overrun",
        "nondeterminism",
    ] {
        assert_eq!(outcome_count(&report, outcome), 0, "{outcome}");
    }
    let refusal = report
        .rows
        .iter()
        .find(|r| r.outcomes["typed_refusal"] == 1)
        .unwrap();
    assert_eq!(refusal.scenario, "box-cut-identical-empty");
    assert!(
        refusal
            .diagnostic
            .as_deref()
            .unwrap()
            .contains("EmptyResult")
    );
    // No timing columns with a single kernel: no speed claim is expressible.
    for row in &report.rows {
        assert!(!row.metrics["resources"].contains_key("runtime_median"));
    }
}

#[test]
fn injected_wrong_success_becomes_silent_wrong() {
    // Real evidence, then a test-side lie about the measured volume: the
    // mapper must report oracle disagreement, never agreement.
    let mut evidence = execute_case("box-fuse-half-overlap").unwrap();
    assert_eq!(evidence.status, "success");
    evidence.volume = Some(99.0);
    let observation = map_pair("box-fuse-half-overlap", &evidence, &evidence);
    assert_eq!(observation["oracle_agrees"], json!(false));
    let report =
        evaluate_json(&assemble(&["box-fuse-half-overlap"], vec![observation]).to_string())
            .unwrap();
    assert!(!report.passed);
    assert_eq!(outcome_count(&report, "silent_wrong"), 1);
    assert!(!report.rows[0].gates["no_silent_wrong"]);
}

#[test]
fn refusal_after_sibling_success_keeps_both_rows() {
    let success = execute_case("box-fuse-identical").unwrap();
    let refusal = execute_case("box-cut-identical-empty").unwrap();
    assert_eq!(refusal.status, "refused");
    let observations = vec![
        map_pair("box-fuse-identical", &success, &success),
        map_pair("box-cut-identical-empty", &refusal, &refusal),
    ];
    let report = evaluate_json(
        &assemble(
            &["box-fuse-identical", "box-cut-identical-empty"],
            observations,
        )
        .to_string(),
    )
    .unwrap();
    assert!(report.passed);
    assert_eq!(outcome_count(&report, "exact_success"), 1);
    assert_eq!(outcome_count(&report, "typed_refusal"), 1);
}

#[test]
fn missing_observation_is_rejected() {
    let (job_path, _) = temp_paths("missing");
    let (code, _) = run_parent(&["--out", &job_path]);
    assert_eq!(code, 0);
    let mut job: Value =
        serde_json::from_str(&std::fs::read_to_string(&job_path).unwrap()).unwrap();
    job["observations"].as_array_mut().unwrap().pop();
    assert!(evaluate_json(&job.to_string()).is_err());
}

#[test]
fn timed_out_child_yields_no_job() {
    // Keep every worker alive past the zero-millisecond ceiling so this
    // exercises the timeout path without racing a fast child exit.
    let (job_path, attempts_path) = temp_paths("timeout");
    let (code, stderr) = run_parent_with_env(
        &[
            "--out",
            &job_path,
            "--attempts",
            &attempts_path,
            "--timeout-ms",
            "0",
        ],
        &[("REMUS_VS_BENCH_TEST_WORKER_DELAY_MS", "1000")],
    );
    assert_eq!(code, 2, "{stderr}");
    assert!(!std::path::Path::new(&job_path).exists());
    let attempts: Value =
        serde_json::from_str(&std::fs::read_to_string(&attempts_path).unwrap()).unwrap();
    let outcomes: Vec<&str> = attempts
        .as_array()
        .unwrap()
        .iter()
        .map(|a| a["outcome"].as_str().unwrap())
        .collect();
    assert!(!outcomes.is_empty());
    assert!(outcomes.iter().all(|o| *o == "timeout"));
    assert!(stderr.contains("incomplete evidence"));
}

#[test]
fn incomplete_mapping_fails_closed() {
    // A claimed success without a measured volume cannot become an
    // observation: the mapper errors instead of emitting a half row.
    let mut evidence = execute_case("box-fuse-identical").unwrap();
    evidence.volume = None;
    let spec = case_spec("box-fuse-identical").unwrap();
    assert!(map_to_observation(&spec, &evidence, &evidence, "k", 0.01, 0.02, "abc1234").is_err());
    // Disagreeing repetitions cannot become one observation either.
    let other = execute_case("box-cut-identical-empty").unwrap();
    assert!(map_to_observation(&spec, &evidence, &other, "k", 0.01, 0.02, "abc1234").is_err());
}

#[test]
fn job_assembly_rejects_wrong_counts() {
    let evidence = execute_case("box-fuse-identical").unwrap();
    let spec = case_spec("box-fuse-identical").unwrap();
    let observation = map_pair("box-fuse-identical", &evidence, &evidence);
    // One observation for two declared cases.
    let other = case_spec("box-fuse-half-overlap").unwrap();
    assert!(
        assemble_job(
            &[(&spec, &evidence), (&other, &evidence)],
            vec![observation],
            "run",
            &"a".repeat(40),
            &"b".repeat(64),
            "k",
            2,
        )
        .is_err()
    );
    // Unknown kernel on the observation.
    let mut foreign = map_pair("box-fuse-identical", &evidence, &evidence);
    foreign["kernel"] = json!("other-kernel");
    assert!(
        assemble_job(
            &[(&spec, &evidence)],
            vec![foreign],
            "run",
            &"a".repeat(40),
            &"b".repeat(64),
            "k",
            2,
        )
        .is_err()
    );
    assert!(check_harness_sha("short").is_err());
}

#[test]
fn every_case_has_a_pinned_repro_identity() {
    for spec in case_specs() {
        assert!(!spec.scope.is_empty());
        assert!(!spec.oracle.is_empty());
        assert!(!spec.tolerance_intent.is_empty());
        assert!(!spec.repro.is_empty());
        assert!(spec.timeout_ms > 0);
        assert!(case_spec(spec.id).is_some());
    }
    assert!(case_spec("no-such-case").is_none());
}

#[test]
fn mesh_watertight_accepts_closed_and_rejects_open_or_degenerate() {
    // Closed tetrahedron: every edge twice, opposite directions.
    let closed = [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2];
    assert!(mesh_watertight(4, &closed));
    // One face removed: boundary edges appear once.
    assert!(!mesh_watertight(4, &[0, 1, 2, 0, 3, 1, 0, 2, 3]));
    // Degenerate triangle and out-of-range index both fail.
    assert!(!mesh_watertight(4, &[0, 0, 1]));
    assert!(!mesh_watertight(2, &[0, 1, 2]));
    assert!(!mesh_watertight(4, &[0, 1]));
}

#[cfg(unix)]
#[test]
fn crash_status_classifies_without_evidence() {
    use std::os::unix::process::ExitStatusExt;
    // SIGKILL wait status, synthesized in-test: no production fault
    // injection, only the pure classifier under test.
    let status = std::process::ExitStatus::from_raw(9);
    assert!(!status.success());
    let outcome = classify_status(status, "killed");
    assert!(
        matches!(
            outcome,
            remus_vs_bench::qualification::ChildOutcome::Crash(_)
        ),
        "{outcome:?}"
    );
    if let remus_vs_bench::qualification::ChildOutcome::Crash(detail) = outcome {
        assert!(detail.contains("killed"));
    }
}

#[test]
fn cli_rejects_unknown_case_workers() {
    let child = Command::new(runner_bin())
        .args(["--worker", "no-such-case"])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let output = child.wait_with_output().unwrap();
    assert_eq!(output.status.code().unwrap(), 2);
}
