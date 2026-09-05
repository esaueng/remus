//! Adversarial scorecard protocol and process-exit contracts.
#![allow(clippy::unwrap_used, clippy::expect_used)]
use remus_vs_bench::{Report, evaluate_json};
use serde_json::{Value, json};
use std::io::Write;
use std::process::{Command, Stdio};

fn fixture() -> Value {
    serde_json::from_str(include_str!("../fixtures/exact-pair.json")).unwrap()
}
fn evaluate(value: &Value) -> Report {
    evaluate_json(&value.to_string()).unwrap()
}
fn no_timing(report: &Report) {
    for row in &report.rows {
        assert!(!row.metrics["resources"].contains_key("runtime_median"));
        assert!(!row.metrics["resources"].contains_key("runtime_p95"));
        assert_eq!(row.outcomes.values().copied().sum::<u8>(), 1);
    }
}

#[test]
fn exact_pair_preserves_metrics_and_deterministic_report() {
    let mut input = fixture();
    let report = evaluate(&input);
    assert!(report.passed);
    assert_eq!(report.rows[0].metrics["resources"]["runtime_median"], 1.0);
    assert_eq!(report.rows[0].outcomes["exact_success"], 1);
    input["observations"].as_array_mut().unwrap().reverse();
    assert_eq!(
        serde_json::to_string(&report).unwrap(),
        serde_json::to_string(&evaluate(&input)).unwrap()
    );
}

#[test]
fn oracle_disagreement_on_either_kernel_is_silent_wrong_even_if_validator_rejects() {
    for kernel in 0..2 {
        let mut input = fixture();
        input["observations"][kernel]["oracle_agrees"] = json!(false);
        input["observations"][kernel]["validator_accepts"] = json!(false);
        input["observations"][kernel]["repeat_agrees"] = json!(false);
        let report = evaluate(&input);
        assert!(!report.passed);
        assert_eq!(
            report
                .rows
                .iter()
                .filter(|r| r.outcomes["silent_wrong"] == 1)
                .count(),
            1
        );
        let failed = report
            .rows
            .iter()
            .find(|r| r.outcomes["silent_wrong"] == 1)
            .unwrap();
        assert!(!failed.gates["valid_success"]);
        assert!(!failed.gates["deterministic"]);
        no_timing(&report);
    }
}

#[test]
fn refusal_excludes_both_kernels_from_speed_table_without_failing_quality_gate() {
    let mut input = fixture();
    input["observations"][0]["reported"] = json!("refusal");
    input["observations"][0]["diagnostic"] = json!("unsupported_pair");
    input["observations"][0]["oracle_agrees"] = Value::Null;
    input["observations"][0]["validator_accepts"] = Value::Null;
    let report = evaluate(&input);
    assert!(report.passed);
    assert_eq!(
        report
            .rows
            .iter()
            .filter(|r| r.outcomes["typed_refusal"] == 1)
            .count(),
        1
    );
    no_timing(&report);
}

#[test]
fn approximate_timing_requires_matching_declared_quality_and_disclosure() {
    let mut input = fixture();
    input["scenarios"][0]["quality"]["representation"] = json!("approximate");
    input["scenarios"][0]["quality"]["error_budget"] = json!(0.001);
    for i in 0..2 {
        input["observations"][i]["quality"] = input["scenarios"][0]["quality"].clone();
        input["observations"][i]["reported"] = json!("approximate_success");
        input["observations"][i]["approximation"] =
            json!({"method":"chordal", "error_bound":0.0005});
    }
    assert!(evaluate(&input).passed);
    assert!(evaluate(&input).rows[0].metrics["resources"].contains_key("runtime_median"));
    for pointer in [
        "/observations/0/quality/deflection",
        "/observations/0/quality/error_budget",
    ] {
        let mut mismatch = input.clone();
        *mismatch.pointer_mut(pointer).unwrap() = json!(0.1);
        let report = evaluate(&mismatch);
        assert!(!report.passed);
        no_timing(&report);
    }
    input["observations"][0]["approximation"] = Value::Null;
    let report = evaluate(&input);
    assert!(!report.passed);
    assert_eq!(
        report
            .rows
            .iter()
            .filter(|r| r.outcomes["disclosed_approximate_success"] == 1)
            .count(),
        1
    );
    no_timing(&report);
}

#[test]
fn verified_repair_is_separate_from_timed_success() {
    let mut input = fixture();
    for observation in input["observations"].as_array_mut().unwrap() {
        observation["reported"] = json!("repaired_success");
        observation["repair_occurred"] = json!(true);
        observation["repairs"] = json!([{"code":"tolerance_growth", "count":2, "verified":true}]);
    }
    let report = evaluate(&input);
    assert!(report.passed);
    assert!(
        report
            .rows
            .iter()
            .all(|r| r.outcomes["verified_repair_success"] == 1)
    );
    no_timing(&report);
    input["observations"][0]["repairs"][0]["verified"] = json!(false);
    assert!(!evaluate(&input).passed);
}

#[test]
fn every_absolute_failure_is_nonzero_without_a_competitor() {
    for (pointer, value, gate) in [
        ("/oracle_agrees", json!(false), "no_silent_wrong"),
        ("/validator_accepts", json!(false), "valid_success"),
        ("/reported", json!("crash"), "no_crash_or_hang"),
        ("/reported", json!("hang"), "no_crash_or_hang"),
        ("/reported", json!("error"), "no_untyped_error"),
        ("/reported", json!("refusal"), "typed_non_success"),
        (
            "/reported",
            json!("approximate_success"),
            "approximation_disclosed",
        ),
        ("/repair_occurred", json!(true), "repair_disclosed_verified"),
        ("/repeat_agrees", json!(false), "deterministic"),
        (
            "/metrics/browser/native_wasm_agreement",
            json!(false),
            "native_wasm_agreement",
        ),
        ("/evolution_explicit", json!(false), "evolution_explicit"),
    ] {
        let mut input = fixture();
        input["kernels"].as_array_mut().unwrap().truncate(1);
        input["observations"].as_array_mut().unwrap().truncate(1);
        *input["observations"][0].pointer_mut(pointer).unwrap() = value;
        let report = evaluate(&input);
        assert!(!report.passed, "{gate}");
        assert!(!report.rows[0].gates[gate], "{gate}");
        no_timing(&report);
        assert_eq!(cli(&input).0, 1, "{gate}");
    }
}

#[test]
fn malformed_and_incomplete_jobs_fail_closed() {
    for (pointer, value) in [
        ("/schema_version", json!(2)),
        ("/harness_sha", json!("unknown")),
        ("/manifest_sha256", json!("")),
        ("/kernels", json!([])),
        ("/observations", json!([])),
        ("/scenarios/0/oracle", json!(" ")),
        ("/scenarios/0/applicable_metrics", json!([])),
        ("/observations/0/oracle_agrees", Value::Null),
        (
            "/observations/0/metrics/resources/runtime_median",
            json!(-1),
        ),
        ("/observations/0/metrics/resources/runtime_p95", json!(0.5)),
        (
            "/observations/0/metrics/history/evolution_completeness",
            json!(1.1),
        ),
        (
            "/observations/0/metrics/browser/native_wasm_agreement",
            json!(1),
        ),
        ("/observations/0/metrics/concurrency", json!({})),
        ("/observations/0/kernel", json!("unknown")),
    ] {
        let mut input = fixture();
        *input.pointer_mut(pointer).unwrap() = value;
        assert!(evaluate_json(&input.to_string()).is_err(), "{pointer}");
        assert_eq!(cli(&input).0, 2, "{pointer}");
    }
    let mut input = fixture();
    input["invented"] = json!(true);
    assert!(evaluate_json(&input.to_string()).is_err());
    let mut input = fixture();
    let duplicate = input["observations"][0].clone();
    input["observations"]
        .as_array_mut()
        .unwrap()
        .push(duplicate);
    assert!(evaluate_json(&input.to_string()).is_err());
}

fn cli(input: &Value) -> (i32, String) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_remus-vs-bench"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(input.to_string().as_bytes())
        .unwrap();
    let output = child.wait_with_output().unwrap();
    (
        output.status.code().unwrap(),
        String::from_utf8(output.stdout).unwrap(),
    )
}

#[test]
fn cli_emits_success_report_and_rejects_report_as_input() {
    let (code, output) = cli(&fixture());
    assert_eq!(code, 0);
    assert!(
        serde_json::from_str::<Value>(&output).unwrap()["passed"]
            .as_bool()
            .unwrap()
    );
    assert!(evaluate_json(&output).is_err());
}

#[test]
fn interchange_components_are_preserved_and_missing_components_rejected() {
    let mut input = fixture();
    for metric in ["round_trip_geometry_fidelity", "assembly_metadata_fidelity"] {
        input["scenarios"][0]["applicable_metrics"]
            .as_array_mut()
            .unwrap()
            .push(json!(metric));
    }
    for observation in input["observations"].as_array_mut().unwrap() {
        observation["metrics"]["interchange"]["round_trip_geometry_fidelity"] = json!({"volume_error":0.001,"area_error":0.002,"centroid_error":0.003,"bounds_error":0.004});
        observation["metrics"]["interchange"]["assembly_metadata_fidelity"] =
            json!({"tree":1.0,"transforms":1.0,"names":0.5,"colors":0.6,"materials":0.7});
    }
    let report = evaluate(&input);
    assert_eq!(
        report.rows[0].metrics["interchange"]["round_trip_geometry_fidelity"]["bounds_error"],
        0.004
    );
    input["observations"][0]["metrics"]["interchange"]["round_trip_geometry_fidelity"]
        .as_object_mut()
        .unwrap()
        .remove("bounds_error");
    assert!(evaluate_json(&input.to_string()).is_err());
}

#[test]
fn reproduction_reference_does_not_waive_a_defect_and_repair_cannot_hide_as_exact() {
    let mut input = fixture();
    input["observations"][0]["repeat_agrees"] = json!(false);
    assert!(
        evaluate(&input)
            .rows
            .iter()
            .any(|r| !r.gates["defect_reproduction"])
    );
    input["observations"][0]["defect_repro"] = json!("fixture:repeat-disagreement");
    let report = evaluate(&input);
    assert!(!report.passed);
    assert!(report.rows.iter().all(|r| r.gates["defect_reproduction"]));
    let mut input = fixture();
    input["observations"][0]["repairs"] = json!([{"code":"heal_wire", "count":1, "verified":true}]);
    let report = evaluate(&input);
    assert!(!report.passed);
    no_timing(&report);
}

#[test]
fn success_labels_cannot_suppress_approximation_or_repetition_evidence() {
    let mut input = fixture();
    input["repetitions"] = json!(1);
    assert!(evaluate_json(&input.to_string()).is_err());
    let mut input = fixture();
    input["observations"][0]["reported"] = json!("approximate_success");
    input["observations"][0]["approximation"] = json!({"method":"mesh", "error_bound":0.0});
    let report = evaluate(&input);
    assert!(!report.passed);
    no_timing(&report);
}

#[test]
fn generic_success_cannot_disguise_approximation_as_exact_quality() {
    let mut input = fixture();
    input["observations"][0]["reported"] = json!("correct_success");
    input["observations"][0]["approximation"] = json!({"method":"mesh", "error_bound":0.0});
    let report = evaluate(&input);
    assert!(!report.passed);
    assert!(
        report
            .rows
            .iter()
            .any(|r| !r.gates["quality_label_consistent"])
    );
    no_timing(&report);
}
