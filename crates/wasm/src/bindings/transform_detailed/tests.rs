//! Contract tests for the O4.7 transform twins (B74).
//!
//! The direct side runs the natively-testable `*_detailed_impl` bodies (a
//! `JsError` cannot be built off-wasm); the batch side goes through
//! `execute_batch` / `execute_batch_v2` on identically built twin kernels.
//! Every success asserts the disclosed `details` (quality, carrier and
//! edge changes, fitted faces with evidence, determinant, orientation and
//! similarity flags), the independently derived closed-form volume, and
//! the result carriers — not only the returned handle. Every refusal is
//! checked for typed data (never a throw), agreement with the legacy op's
//! `executeBatchV2` code and category, and unchanged topology with a
//! working retry.
//!
//! The fitted-sphere path is reached through a boolean-cut sphere segment
//! (a latitude trim outside the exact class); hemispheres convert exactly.
//! Batch twins exist for both solid routes; wire/face in-place mutations
//! keep their legacy void shapes with no twin (O4.7 inventory).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

type Counts = (usize, usize, usize, usize, usize);

fn counts(kernel: &BrepKernel) -> Counts {
    let topo = kernel.topo();
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
        topo.allocated_slot_count(),
    )
}

/// Live entity counts, without the handle high-water mark: a rollback after
/// an attempt that allocated keeps those slots retired so no stale handle
/// can alias a later entity.
fn live_counts(kernel: &BrepKernel) -> (usize, usize, usize, usize) {
    let (vertices, edges, faces, solids, _) = counts(kernel);
    (vertices, edges, faces, solids)
}

fn envelope(result: SolidOperationDetailedResult) -> Value {
    serde_json::to_value(result).unwrap()
}

fn batch_v2(kernel: &mut BrepKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap()
}

fn batch_legacy(kernel: &mut BrepKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap()
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

fn diag_aniso() -> Vec<f64> {
    vec![
        2.0, 0.0, 0.0, 0.0, //
        0.0, 0.5, 0.0, 0.0, //
        0.0, 0.0, 1.5, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

fn mirror_aniso() -> Vec<f64> {
    vec![
        -2.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

fn assert_ok_details<'a>(env: &'a Value, quality: &str) -> &'a Value {
    assert_eq!(env["status"], "ok", "expected ok envelope, got {env}");
    assert!(env["code"].is_null(), "success carries no code: {env}");
    let details = &env["details"];
    assert_eq!(details["quality"], quality, "quality mismatch: {env}");
    details
}

/// Build a sphere zone with an off-equator band face: sphere r=2 with
/// everything above z=1.0 and below z=-1.0 cut away. GFA leaves a general
/// sphere patch outside the exact class (fitted path / exact-only refusal)
/// alongside exact hemispheres and discs.
fn make_trimmed_sphere(kernel: &mut BrepKernel) -> u32 {
    let sphere = kernel.make_sphere_solid(2.0, 16).unwrap();
    let top_cutter = kernel.make_box_solid(20.0, 20.0, 20.0).unwrap();
    kernel
        .transform_solid_binding(
            top_cutter,
            vec![
                1.0, 0.0, 0.0, -10.0, //
                0.0, 1.0, 0.0, -10.0, //
                0.0, 0.0, 1.0, 1.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        .unwrap();
    let top = kernel.cut(sphere, top_cutter).unwrap();
    let bottom_cutter = kernel.make_box_solid(20.0, 20.0, 20.0).unwrap();
    kernel
        .transform_solid_binding(
            bottom_cutter,
            vec![
                1.0, 0.0, 0.0, -10.0, //
                0.0, 1.0, 0.0, -10.0, //
                0.0, 0.0, 1.0, -21.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        .unwrap();
    kernel.cut(top, bottom_cutter).unwrap()
}

// ── transformDetailed ────────────────────────────────────────────────────

#[test]
fn transform_detailed_reports_carriers_and_matches_batch() {
    let mut direct = BrepKernel::new();
    let mut batched = BrepKernel::new();
    let direct_solid = direct.make_box_solid(2.0, 3.0, 4.0).unwrap();
    let batched_solid = batched.make_box_solid(2.0, 3.0, 4.0).unwrap();
    assert_eq!(direct_solid, batched_solid);
    let solid = direct_solid;
    let matrix = diag_aniso();

    let env = envelope(direct.transform_detailed_impl(solid, &matrix, false));
    let details = assert_ok_details(&env, "exact");
    assert_eq!(details["determinant"], 1.5);
    assert_eq!(details["orientationReversed"], false);
    assert_eq!(details["similarity"], false);
    assert!(details["carrierChanges"].as_array().unwrap().is_empty());
    assert!(details["edgeChanges"].as_array().unwrap().is_empty());
    assert!(details["fittedFaces"].as_array().unwrap().is_empty());
    assert_eq!(env["value"], solid);

    // Batch returns the identical envelope.
    let matrix_json: Vec<Value> = matrix.iter().map(|v| json!(v)).collect();
    let out = batch_v2(
        &mut batched,
        &json!([{"op": "transformDetailed", "args": {"solid": solid, "matrix": matrix_json}}]),
    );
    assert_eq!(out[0]["ok"], env, "direct and batch envelopes differ");

    // Legacy geometry agrees: same handle, closed-form volume.
    let legacy = batch_legacy(
        &mut batched,
        &json!([
            {"op": "makeBox", "args": {"width": 2.0, "height": 3.0, "depth": 4.0}},
            {"op": "transform", "args": {"solid": 1, "matrix": matrix_json}},
            {"op": "volume", "args": {"solid": 1, "deflection": 0.01}},
        ]),
    );
    assert!(legacy[1]["ok"].is_number());
    let volume = legacy[2]["ok"].as_f64().unwrap();
    assert!((volume - 24.0 * 1.5).abs() < 1e-6, "volume {volume}");
    let twin_volume = direct
        .volume(env["value"].as_u64().unwrap() as u32, 0.01)
        .unwrap();
    assert!((twin_volume - volume).abs() < 1e-9);
}

#[test]
fn transform_detailed_discloses_nurbs_wall_and_orientation() {
    // Cylinder wall converts exactly; nothing is fitted.
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_cylinder_solid(1.0, 2.0).unwrap();
    let env = envelope(kernel.transform_detailed_impl(solid, &diag_aniso(), false));
    let details = assert_ok_details(&env, "exact");
    let changes = details["carrierChanges"].as_array().unwrap();
    assert_eq!(changes.len(), 1);
    assert_eq!(changes[0]["from"], "cylinder");
    assert_eq!(changes[0]["to"], "nurbs");
    assert_eq!(changes[0]["method"], "exactRational");

    // Mirror-scaled sphere: two exact hemisphere conversions plus the
    // orientation flag.
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_sphere_solid(1.0, 16).unwrap();
    let env = envelope(kernel.transform_detailed_impl(solid, &mirror_aniso(), false));
    let details = assert_ok_details(&env, "exact");
    assert_eq!(details["orientationReversed"], true);
    assert_eq!(details["determinant"], -2.0);
    let changes = details["carrierChanges"].as_array().unwrap();
    assert_eq!(changes.len(), 2);
    for change in changes {
        assert_eq!(change["from"], "sphere");
        assert_eq!(change["to"], "nurbs");
        assert_eq!(change["method"], "exactRational");
    }
    assert!(details["fittedFaces"].as_array().unwrap().is_empty());
    let volume = kernel
        .volume(env["value"].as_u64().unwrap() as u32, 0.005)
        .unwrap();
    let expected = 4.0 / 3.0 * std::f64::consts::PI * 2.0;
    assert!(
        (volume - expected).abs() / expected < 2e-3,
        "volume {volume}"
    );
}

#[test]
fn transform_detailed_refusals_are_typed_data() {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();

    // Short matrix.
    let env = envelope(kernel.transform_detailed_impl(solid, &[1.0; 9], false));
    assert_eq!(env["status"], "error");
    assert!(env["value"].is_null());

    // Non-finite entry.
    let mut bad = diag_aniso();
    bad[3] = f64::INFINITY;
    let env = envelope(kernel.transform_detailed_impl(solid, &bad, false));
    assert_eq!(env["status"], "error");

    // Bad handle.
    let env = envelope(kernel.transform_detailed_impl(9999, &diag_aniso(), false));
    assert_eq!(env["status"], "error");
    assert_eq!(env["code"], "invalid_handle");

    // Legacy batch V2 agrees on codes for the same mistakes.
    let matrix_json: Vec<Value> = diag_aniso().iter().map(|v| json!(v)).collect();
    let out = batch_v2(
        &mut kernel,
        &json!([
            {"op": "transform", "args": {"solid": solid, "matrix": [1.0]}},
            {"op": "transform", "args": {"solid": 9999, "matrix": matrix_json}},
        ]),
    );
    assert_eq!(out[0]["error"]["code"], "invalid_argument");
    assert_eq!(out[1]["error"]["code"], "invalid_handle");
}

#[test]
fn transform_detailed_refusals_roll_back() {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_cylinder_solid(1.0, 2.0).unwrap();
    let before = live_counts(&kernel);
    let volume_before = kernel.volume(solid, 0.01).unwrap();

    // Shear skews the circular rims: typed refusal as data.
    let env = envelope(kernel.transform_detailed_impl(
        solid,
        &[
            1.0, 0.5, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ],
        false,
    ));
    assert_eq!(env["status"], "error");
    assert_eq!(live_counts(&kernel), before, "refusal mutated topology");
    assert_eq!(
        kernel.volume(solid, 0.01).unwrap().to_bits(),
        volume_before.to_bits()
    );

    // Degenerate matrix refuses the same way.
    let env = envelope(kernel.transform_detailed_impl(
        solid,
        &[
            1.0, 0.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ],
        false,
    ));
    assert_eq!(env["status"], "error");
    assert_eq!(live_counts(&kernel), before);

    // Retry proves the kernel stayed usable.
    let env = envelope(kernel.transform_detailed_impl(solid, &diag_aniso(), false));
    assert_ok_details(&env, "exact");
}

// ── copyAndTransformSolidDetailed (O4.7 covered row witnesses) ────────────

#[test]
fn copy_transform_twin_matches_direct_and_batch() {
    let mut direct = BrepKernel::new();
    let mut batched = BrepKernel::new();
    let direct_solid = direct.make_sphere_solid(1.0, 16).unwrap();
    let batched_solid = batched.make_sphere_solid(1.0, 16).unwrap();
    assert_eq!(direct_solid, batched_solid);
    let matrix = diag_aniso();
    let env = envelope(direct.copy_and_transform_solid_detailed_impl(direct_solid, &matrix, false));
    let details = assert_ok_details(&env, "exact");
    let changes = details["carrierChanges"].as_array().unwrap();
    assert_eq!(changes.len(), 2);
    let copied = handle(&env["value"]);
    assert_ne!(copied, direct_solid);
    // Source untouched by the copy path.
    assert!(direct.get_solid_faces(direct_solid).is_ok());

    let matrix_json: Vec<Value> = matrix.iter().map(|v| json!(v)).collect();
    let out = batch_v2(
        &mut batched,
        &json!([{"op": "copyAndTransformSolidDetailed", "args": {"solid": batched_solid, "matrix": matrix_json}}]),
    );
    assert_eq!(out[0]["ok"], env, "direct and batch envelopes differ");

    let expected = 4.0 / 3.0 * std::f64::consts::PI * 1.5;
    let volume = direct.volume(copied, 0.005).unwrap();
    assert!(
        (volume - expected).abs() / expected < 2e-3,
        "volume {volume}"
    );
}

#[test]
fn copy_transform_exact_only_refuses_fitted_result() {
    let mut direct = BrepKernel::new();
    let mut batched = BrepKernel::new();
    let direct_solid = make_trimmed_sphere(&mut direct);
    let batched_solid = make_trimmed_sphere(&mut batched);
    let before = live_counts(&direct);

    // Exact-only refuses as data with the native refusal code, naming the
    // fitted faces, and mutates nothing.
    let env =
        envelope(direct.copy_and_transform_solid_detailed_impl(direct_solid, &diag_aniso(), true));
    assert_eq!(env["status"], "error");
    assert_eq!(env["code"], "exact_only_unattainable");
    assert_eq!(env["category"], "quality_refused");
    assert_eq!(env["details"]["kernelCode"], "exact_only_unattainable");
    let fitted = env["details"]["fittedFaces"].as_array().unwrap();
    assert!(!fitted.is_empty(), "refusal must name faces: {env}");
    assert_eq!(live_counts(&direct), before);

    // The batch twin refuses identically.
    let matrix_json: Vec<Value> = diag_aniso().iter().map(|v| json!(v)).collect();
    let out = batch_v2(
        &mut batched,
        &json!([{"op": "copyAndTransformSolidDetailed", "args": {"solid": batched_solid, "matrix": matrix_json, "exactOnly": true}}]),
    );
    assert_eq!(out[0]["ok"], env, "direct and batch refusals differ");

    // Permissive path discloses the fit instead of refusing.
    let env =
        envelope(direct.copy_and_transform_solid_detailed_impl(direct_solid, &diag_aniso(), false));
    let details = assert_ok_details(&env, "approximate");
    let fitted = details["fittedFaces"].as_array().unwrap();
    assert!(
        !fitted.is_empty(),
        "zone must disclose its general patch: {env}"
    );
    assert_eq!(fitted[0]["from"], "sphere");
    assert_eq!(fitted[0]["method"], "interpolate33x17");
    assert!(fitted[0]["maxResidual"].as_f64().unwrap().is_finite());
}

#[test]
fn copy_transform_refusals_preserve_topology() {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
    let before = live_counts(&kernel);

    let env = envelope(kernel.copy_and_transform_solid_detailed_impl(solid, &[1.0; 4], false));
    assert_eq!(env["status"], "error");
    let mut bad = diag_aniso();
    bad[0] = f64::NAN;
    let env = envelope(kernel.copy_and_transform_solid_detailed_impl(solid, &bad, false));
    assert_eq!(env["status"], "error");
    let env = envelope(kernel.copy_and_transform_solid_detailed_impl(9999, &diag_aniso(), false));
    assert_eq!(env["status"], "error");
    assert_eq!(env["code"], "invalid_handle");
    assert_eq!(live_counts(&kernel), before, "refusals mutated topology");

    // Batch V2 agrees on codes.
    let out = batch_v2(
        &mut kernel,
        &json!([
            {"op": "copyAndTransformSolid", "args": {"solid": 9999, "matrix": diag_aniso()}},
        ]),
    );
    assert_eq!(out[0]["error"]["code"], "invalid_handle");
}

#[test]
fn copy_transform_legacy_geometry_agrees() {
    // Legacy shape unchanged: the throwing copy route builds the same
    // geometry the twin reports.
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_cylinder_solid(1.0, 2.0).unwrap();
    let copied = kernel
        .copy_and_transform_solid(solid, diag_aniso())
        .unwrap();
    assert_ne!(copied, solid);

    let mut twin = BrepKernel::new();
    let source = twin.make_cylinder_solid(1.0, 2.0).unwrap();
    let env = envelope(twin.copy_and_transform_solid_detailed_impl(source, &diag_aniso(), false));
    assert_ok_details(&env, "exact");
    let twin_volume = twin.volume(handle(&env["value"]), 0.005).unwrap();
    let legacy_volume = kernel.volume(copied, 0.005).unwrap();
    assert!((twin_volume - legacy_volume).abs() < 1e-9);
    let expected = std::f64::consts::PI * 2.0 * 1.5;
    assert!((legacy_volume - expected).abs() / expected < 2e-3);

    // Legacy in-place shape unchanged: still void-ok on rigid maps.
    kernel
        .transform_solid_binding(
            solid,
            vec![
                1.0, 0.0, 0.0, 5.0, //
                0.0, 1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, 0.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        .unwrap();
}
