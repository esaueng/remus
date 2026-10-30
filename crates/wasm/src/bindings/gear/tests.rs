//! Contract tests for `makeGearDetailed`, direct and batch.
//!
//! The direct side runs `make_gear_detailed_impl` (a `JsError` cannot be
//! built off-wasm); the batch side goes through `execute_batch_v2`, whose
//! `ok` value is the same envelope. Volumes are checked against a native
//! `make_gear` on its own topology, and refusals against unchanged session
//! counts.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use remus_operations::gear::{GearKind, GearParams, GearPlacement, make_gear};
use remus_topology::Topology;

use crate::kernel::RemusKernel;

fn envelope(kernel: &mut RemusKernel, spec: &Value) -> Value {
    serde_json::to_value(kernel.make_gear_detailed_impl(spec)).unwrap()
}

fn batch_v2(kernel: &mut RemusKernel, args: &Value) -> Value {
    let ops = json!([{ "op": "makeGearDetailed", "args": args }]);
    let results: Value = serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap();
    results[0].clone()
}

fn live_counts(kernel: &RemusKernel) -> (usize, usize, usize, usize) {
    let topo = kernel.topo();
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    )
}

fn native_volume(params: &GearParams) -> f64 {
    let mut topo = Topology::new();
    let outcome = make_gear(&mut topo, params, &GearPlacement::default()).unwrap();
    remus_operations::measure::solid_volume(&topo, outcome.solid, 1e-2).unwrap()
}

fn assert_valid(kernel: &RemusKernel, solid: u32) {
    let id = kernel.resolve_solid(solid).unwrap();
    let report = remus_operations::validate::validate_solid(kernel.topo(), id).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

#[test]
fn external_gear_reports_approximate_flanks_with_their_bound_and_dimensions() {
    let mut kernel = RemusKernel::new();
    let spec = json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0 });
    let result = envelope(&mut kernel, &spec);
    assert_eq!(result["status"], "ok", "{result}");
    let details = &result["details"];
    assert_eq!(details["quality"], "approximate");
    assert_eq!(details["approximated"], "involute flanks");
    let bound = details["flankDeviationBound"].as_f64().unwrap();
    assert!(bound > 0.0 && bound <= 2e-5, "{bound}");
    assert!(details["spansPerFlank"].as_u64().unwrap() >= 1);
    assert!((details["geometry"]["pitchDiameter"].as_f64().unwrap() - 40.0).abs() < 1e-12);
    assert!((details["geometry"]["tipDiameter"].as_f64().unwrap() - 44.0).abs() < 1e-12);
    assert_eq!(details["warnings"], json!([]));

    let solid = handle(&result["value"]);
    let expected = native_volume(&GearParams::standard(GearKind::External, 2.0, 20, 5.0));
    let volume = kernel.volume(solid, 1e-2).unwrap();
    assert!(
        ((volume - expected) / expected).abs() < 1e-9,
        "{volume} vs {expected}"
    );
    assert_valid(&kernel, solid);
}

#[test]
fn batch_and_direct_results_are_identical() {
    let spec = json!({
        "kind": "external", "module": 1.5, "teeth": 13, "faceWidth": 4.0,
        "pressureAngle": 25.0, "profileShift": 0.2, "backlash": 0.05,
        "boreDiameter": 6.0, "origin": [1.0, 2.0, 3.0], "axis": [0.0, 1.0, 0.0],
        "xDirection": [1.0, 0.0, 0.0], "phase": 7.5
    });
    let mut direct_kernel = RemusKernel::new();
    let direct = envelope(&mut direct_kernel, &spec);
    let mut batch_kernel = RemusKernel::new();
    let batch = batch_v2(&mut batch_kernel, &spec);
    assert_eq!(direct["status"], "ok", "{direct}");
    assert_eq!(batch["ok"], direct);
    assert_eq!(live_counts(&direct_kernel), live_counts(&batch_kernel));
}

#[test]
fn rack_is_exact_and_internal_gear_needs_its_rim() {
    let mut kernel = RemusKernel::new();
    let rack = envelope(
        &mut kernel,
        &json!({ "kind": "rack", "module": 1.0, "teeth": 9, "faceWidth": 3.0 }),
    );
    assert_eq!(rack["status"], "ok", "{rack}");
    assert_eq!(rack["details"]["quality"], "exact");
    assert!(rack["details"].get("approximated").is_none());
    assert_eq!(rack["details"]["flankDeviationBound"], 0.0);
    assert!(rack["details"]["geometry"]["pitchDiameter"].is_null());

    let before = live_counts(&kernel);
    let missing_rim = envelope(
        &mut kernel,
        &json!({ "kind": "internal", "module": 1.0, "teeth": 40, "faceWidth": 3.0 }),
    );
    assert_eq!(missing_rim["status"], "error");
    assert_eq!(missing_rim["code"], "gear:invalid-parameter");
    assert_eq!(missing_rim["category"], "invalid_input");
    assert_eq!(live_counts(&kernel), before);

    let ring = envelope(
        &mut kernel,
        &json!({ "kind": "internal", "module": 1.0, "teeth": 40, "faceWidth": 3.0, "rimDiameter": 52.0 }),
    );
    assert_eq!(ring["status"], "ok", "{ring}");
    assert_valid(&kernel, handle(&ring["value"]));
}

#[test]
fn profile_refusals_carry_dimensions_and_leave_the_session_unchanged() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    let pointed = json!({
        "kind": "external", "module": 2.0, "teeth": 8, "faceWidth": 5.0, "profileShift": 1.2
    });
    let result = envelope(&mut kernel, &pointed);
    assert_eq!(result["status"], "error", "{result}");
    assert_eq!(result["code"], "gear:pointed-tip");
    assert!(result["value"].is_null());
    assert!(
        result["details"]["geometry"]["tipThickness"]
            .as_f64()
            .unwrap()
            <= 0.0
    );
    assert_eq!(result["details"]["operation"], "makeGear");
    assert_eq!(live_counts(&kernel), before);

    let batch = batch_v2(&mut kernel, &pointed);
    assert_eq!(batch["ok"], result);

    let bore = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "boreDiameter": 36.0 }),
    );
    assert_eq!(bore["code"], "gear:bore-too-large");
    assert_eq!(bore["details"]["argument"], "boreDiameter");
    assert_eq!(live_counts(&kernel), before);
}

#[test]
fn undercut_warning_names_the_avoiding_shift() {
    let mut kernel = RemusKernel::new();
    let result = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 1.0, "teeth": 10, "faceWidth": 2.0 }),
    );
    assert_eq!(result["status"], "ok", "{result}");
    let warnings = result["details"]["warnings"].as_array().unwrap();
    let undercut = warnings.iter().find(|w| w["code"] == "undercut").unwrap();
    let expected = 1.0 - 5.0 * 20.0_f64.to_radians().sin().powi(2);
    assert!((undercut["minProfileShift"].as_f64().unwrap() - expected).abs() < 1e-12);
}

#[test]
fn malformed_specs_refuse_as_invalid_arguments() {
    let mut kernel = RemusKernel::new();
    let before = live_counts(&kernel);
    for spec in [
        json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "boreDiamter": 4.0 }),
        json!({ "kind": "external", "module": 2.0, "faceWidth": 5.0 }),
        json!({ "kind": "external", "module": 2.0, "teeth": 20.5, "faceWidth": 5.0 }),
        json!({ "kind": "helical", "module": 2.0, "teeth": 20, "faceWidth": 5.0 }),
        json!({ "kind": "external", "module": "2", "teeth": 20, "faceWidth": 5.0 }),
        json!({ "kind": "external", "module": 2.0, "teeth": 20, "faceWidth": 5.0, "axis": [0.0, 1.0] }),
        json!([1, 2, 3]),
    ] {
        let result = envelope(&mut kernel, &spec);
        assert_eq!(result["status"], "error", "{spec}: {result}");
        assert_eq!(result["code"], "invalid_argument", "{spec}: {result}");
        assert_eq!(result["category"], "invalid_input");
        let batch = batch_v2(&mut kernel, &spec);
        assert_eq!(batch["ok"], result, "{spec}");
    }
    let zero_module = envelope(
        &mut kernel,
        &json!({ "kind": "external", "module": 0.0, "teeth": 20, "faceWidth": 5.0 }),
    );
    assert_eq!(zero_module["code"], "gear:invalid-parameter");
    assert!(zero_module["details"].get("geometry").is_none());
    assert_eq!(live_counts(&kernel), before);
}
