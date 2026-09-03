//! WASM contract tests for the K-S1 fillet fail-closed migration: no public
//! fillet/chamfer entry point may report success by returning the unchanged
//! input handle, return a geometrically invalid result, or leave the input
//! half-mutated behind an error.
//!
//! Two contract surfaces are exercised for each case:
//!
//! - the legacy `executeBatch` string contract, where the refusal is the
//!   error message itself, and
//! - the `executeBatchV2` structured contract, which must additionally carry
//!   the stable `kernelCode` detail (`radius-too-large`, `edges-not-blended`,
//!   …) that the direct `fillet` binding prefixes onto its messages.
//!
//! The direct `#[wasm_bindgen]` methods (`fillet`, `filletVariable`,
//! `filletV2`, `chamfer*`) are not callable from native tests (`JsError`
//! cannot be constructed off-wasm); their logic lives in
//! `crate::helpers::{fillet_whole_selection, try_fillet, try_chamfer}` and the
//! operations-layer engines, both covered natively
//! (`helpers::fillet_tests`, `crates/operations/tests/regress_fillet_fail_closed.rs`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;
use remus_math::vec::Point3;

fn run(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch(&json)).unwrap()
}

fn run_v2(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch_v2(&json)).unwrap()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn make_box(kernel: &mut BrepKernel, w: f64, h: f64, d: f64) -> u32 {
    let out = run(
        kernel,
        &[op(
            "makeBox",
            serde_json::json!({"width": w, "height": h, "depth": d}),
        )],
    );
    u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap()
}

fn edge_handles(kernel: &mut BrepKernel, solid: u32) -> Vec<u32> {
    let out = run(
        kernel,
        &[op("solidEdges", serde_json::json!({"solid": solid}))],
    );
    out[0]["ok"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| u32::try_from(v.as_u64().unwrap()).unwrap())
        .collect()
}

fn volume(kernel: &mut BrepKernel, solid: u32) -> f64 {
    let out = run(
        kernel,
        &[op(
            "volume",
            serde_json::json!({"solid": solid, "deflection": 0.01}),
        )],
    );
    out[0]["ok"].as_f64().unwrap()
}

/// Closed-form material a convex fillet removes: (1−π/4)·r² per unit length.
fn convex_fillet_removed(radius: f64, length: f64) -> f64 {
    (1.0 - std::f64::consts::FRAC_PI_4) * radius * radius * length
}

fn origin_setback_specs(
    kernel: &mut BrepKernel,
    solid: u32,
    mismatched: bool,
) -> Vec<serde_json::Value> {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let mut selected = Vec::new();
    for handle in edge_handles(kernel, solid) {
        let edge_id = kernel.resolve_edge(handle).unwrap();
        let edge = kernel.topo().edge(edge_id).unwrap();
        let start = kernel.topo().vertex(edge.start()).unwrap().point();
        let end = kernel.topo().vertex(edge.end()).unwrap().point();
        if (start - origin).length() < 1e-10 {
            selected.push((handle, true));
        } else if (end - origin).length() < 1e-10 {
            selected.push((handle, false));
        }
    }
    assert_eq!(selected.len(), 3);

    selected
        .into_iter()
        .zip([1.2_f64, 1.4, 1.6])
        .enumerate()
        .map(|(index, ((edge, origin_is_start), far_radius))| {
            let setback = if mismatched && index == 0 { 0.8 } else { 1.0 };
            if mismatched {
                serde_json::json!({
                    "edge": edge,
                    "law": "constant",
                    "radius": 1.0,
                    "startSetback": if origin_is_start { setback } else { 0.0 },
                    "endSetback": if origin_is_start { 0.0 } else { setback },
                })
            } else {
                serde_json::json!({
                    "edge": edge,
                    "law": "scurve",
                    "start": if origin_is_start { 1.0 } else { far_radius },
                    "end": if origin_is_start { far_radius } else { 1.0 },
                    "startSetback": if origin_is_start { setback } else { 0.0 },
                    "endSetback": if origin_is_start { 0.0 } else { setback },
                })
            }
        })
        .collect()
}

// ── executeBatch: success must be a genuinely changed, valid solid ──

/// A supported batch fillet returns a NEW handle, removes the closed-form
/// sliver, and leaves the input measuring exactly as before.
#[test]
fn batch_fillet_success_changes_the_answer_and_keeps_the_input() {
    let mut k = BrepKernel::new();
    let solid = make_box(&mut k, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut k, solid);
    let before = volume(&mut k, solid);

    let out = run(
        &mut k,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "radius": 1.0, "edges": [edges[0]]}),
        )],
    );
    let result = u32::try_from(out[0]["ok"].as_u64().expect("fillet must succeed")).unwrap();
    assert_ne!(result, solid, "fillet must not echo the input handle");

    let expected = 1000.0 - convex_fillet_removed(1.0, 10.0);
    let after = volume(&mut k, result);
    assert!(
        (after - expected).abs() < 0.5,
        "expected ≈{expected:.3} mm³ (closed form 997.854), got {after:.3}"
    );
    let valid = run(
        &mut k,
        &[op("validateSolid", serde_json::json!({"solid": result}))],
    );
    assert_eq!(
        valid[0]["ok"], 0,
        "the filleted solid must validate: {valid:?}"
    );

    let input_after = volume(&mut k, solid);
    assert!(
        (input_after - before).abs() < 1e-9,
        "the input must measure exactly as before: {before} -> {input_after}"
    );
}

/// Baseline defect, batch surface: `filletVariable` with r=50 on a 10 mm box
/// returned `ok` and a new handle measuring 3242.011 mm³ — the volume GREW.
/// Now both batch contracts refuse it, and the input is untouched.
#[test]
fn batch_fillet_variable_oversized_radius_is_a_typed_refusal() {
    let mut k = BrepKernel::new();
    let solid = make_box(&mut k, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut k, solid);

    let legacy = run(
        &mut k,
        &[
            op(
                "filletVariable",
                serde_json::json!({"solid": solid, "specs": [{"edge": edges[0], "law": "constant", "radius": 50.0}]}),
            ),
            op(
                "volume",
                serde_json::json!({"solid": solid, "deflection": 0.01}),
            ),
        ],
    );
    let message = legacy[0]["error"]
        .as_str()
        .expect("legacy batch must refuse the oversized variable fillet");
    assert!(
        message.contains("convex"),
        "the refusal must name the geometric impossibility, got: {message}"
    );
    let v = legacy[1]["ok"].as_f64().unwrap();
    assert!(
        (v - 1000.0).abs() < 1e-9,
        "the refused op must leave the input measuring 1000 mm³ exactly, got {v}"
    );

    let structured = run_v2(
        &mut k,
        &[op(
            "filletVariable",
            serde_json::json!({"solid": solid, "specs": [{"edge": edges[0], "law": "constant", "radius": 50.0}]}),
        )],
    );
    let error = &structured[0]["error"];
    assert_eq!(
        error["details"]["kernelCode"], "invalid-input",
        "the structured refusal must carry the fine-grained code: {error}"
    );
}

/// A supported `filletVariable` still works: new handle, closed-form sliver.
#[test]
fn batch_fillet_variable_success_is_a_real_fillet() {
    let mut k = BrepKernel::new();
    let solid = make_box(&mut k, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut k, solid);

    let out = run(
        &mut k,
        &[op(
            "filletVariable",
            serde_json::json!({"solid": solid, "specs": [{"edge": edges[0], "law": "linear", "start": 0.5, "end": 1.5}]}),
        )],
    );
    let result =
        u32::try_from(out[0]["ok"].as_u64().expect("variable fillet must succeed")).unwrap();
    assert_ne!(result, solid, "must not echo the input handle");
    let after = volume(&mut k, result);
    assert!(
        after < 1000.0 && after > 990.0,
        "a single-edge variable fillet removes only a sliver, got {after}"
    );
}

#[test]
fn direct_and_batch_variable_setbacks_are_parity_watertight() {
    let mut direct_kernel = BrepKernel::new();
    let direct_input = make_box(&mut direct_kernel, 10.0, 10.0, 10.0);
    let direct_specs = origin_setback_specs(&mut direct_kernel, direct_input, false);
    let direct_result = direct_kernel
        .fillet_variable(
            direct_input,
            &serde_json::Value::Array(direct_specs).to_string(),
        )
        .expect("direct filletVariable setback call");
    let direct_volume = volume(&mut direct_kernel, direct_result);

    let mut batch_kernel = BrepKernel::new();
    let batch_input = make_box(&mut batch_kernel, 10.0, 10.0, 10.0);
    let batch_specs = origin_setback_specs(&mut batch_kernel, batch_input, false);
    let out = run(
        &mut batch_kernel,
        &[op(
            "filletVariable",
            serde_json::json!({"solid": batch_input, "specs": batch_specs}),
        )],
    );
    let batch_result = u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap();
    let batch_volume = volume(&mut batch_kernel, batch_result);
    assert!((direct_volume - batch_volume).abs() < 1e-9);

    for (kernel, result) in [
        (&mut direct_kernel, direct_result),
        (&mut batch_kernel, batch_result),
    ] {
        let quality = run(
            kernel,
            &[op(
                "meshQuality",
                serde_json::json!({"solid": result, "deflection": 0.01}),
            )],
        );
        assert_eq!(quality[0]["ok"]["isWatertight"], true);
        assert_eq!(quality[0]["ok"]["boundaryEdges"], 0);
        assert_eq!(quality[0]["ok"]["nonManifoldEdges"], 0);
    }
}

#[test]
fn batch_variable_setback_mismatch_keeps_the_input_and_code() {
    let mut kernel = BrepKernel::new();
    let input = make_box(&mut kernel, 10.0, 10.0, 10.0);
    let specs = origin_setback_specs(&mut kernel, input, true);
    let out = run_v2(
        &mut kernel,
        &[
            op(
                "filletVariable",
                serde_json::json!({"solid": input, "specs": specs}),
            ),
            op(
                "volume",
                serde_json::json!({"solid": input, "deflection": 0.01}),
            ),
        ],
    );
    assert_eq!(out[0]["error"]["details"]["kernelCode"], "setback-mismatch");
    assert!((out[1]["ok"].as_f64().unwrap() - 1000.0).abs() < 1e-9);
}

#[test]
fn batch_variable_setback_rejects_a_present_nonnumeric_distance() {
    let mut kernel = BrepKernel::new();
    let input = make_box(&mut kernel, 10.0, 10.0, 10.0);
    let edge = edge_handles(&mut kernel, input)[0];
    let out = run_v2(
        &mut kernel,
        &[op(
            "filletVariable",
            serde_json::json!({
                "solid": input,
                "specs": [{
                    "edge": edge,
                    "law": "constant",
                    "radius": 1.0,
                    "startSetback": "one",
                }],
            }),
        )],
    );
    assert_eq!(out[0]["error"]["code"], "invalid_argument");
    assert_eq!(out[0]["error"]["category"], "invalid_input");
    assert!(
        out[0]["error"]["message"]
            .as_str()
            .unwrap()
            .contains("'startSetback' in fillet spec must be a number")
    );
}

/// The walking engine's typed refusal (`radius-too-large`) reaches the
/// structured batch contract unchanged, and the box survives.
#[test]
fn batch_fillet_v2_oversized_radius_names_the_cause() {
    let mut k = BrepKernel::new();
    let solid = make_box(&mut k, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut k, solid);

    let structured = run_v2(
        &mut k,
        &[
            op(
                "filletV2",
                serde_json::json!({"solid": solid, "radius": 50.0, "edges": [edges[0]]}),
            ),
            op(
                "volume",
                serde_json::json!({"solid": solid, "deflection": 0.01}),
            ),
        ],
    );
    let error = &structured[0]["error"];
    assert_eq!(
        error["details"]["kernelCode"], "radius-too-large",
        "the structured refusal must name the cause: {error}"
    );
    let v = structured[1]["ok"].as_f64().unwrap();
    assert!(
        (v - 1000.0).abs() < 1e-9,
        "input must be untouched, got {v}"
    );
}

/// A mixed selection whose only succeeding subset is the planar one is a
/// whole-selection refusal on the batch path too — the same
/// `edges-not-blended` the direct `fillet` binding reports — never a quietly
/// reduced answer.
///
/// The fixture is the OpenZCAD plate case: one straight top-perimeter edge
/// (planar, blendable on its own) plus the bore rim (a closed circle that is
/// an inner loop of the top face — the walking trimmer cannot retrim it, the
/// rolling-ball engine declines closed edges, and the flat bevel cannot touch
/// a curved face). Every engine refuses the pair, the planar subset exists,
/// so the answer must name the rim edge it could not blend.
#[test]
fn batch_fillet_mixed_selection_is_refused_as_a_whole() {
    let mut k = BrepKernel::new();
    // Plate with a through bore: straight perimeter edges + the bore rim.
    let out = run(
        &mut k,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 80.0, "height": 60.0, "depth": 6.0}),
            ),
            op(
                "makeCylinder",
                serde_json::json!({"radius": 2.25, "height": 20.0}),
            ),
            op(
                "transform",
                serde_json::json!({"solid": 1, "matrix": [1.0,0.0,0.0,40.0, 0.0,1.0,0.0,30.0, 0.0,0.0,1.0,-4.0, 0.0,0.0,0.0,1.0]}),
            ),
            op("cut", serde_json::json!({"solidA": 0, "solidB": 1})),
        ],
    );
    let plate = u32::try_from(out[3]["ok"].as_u64().unwrap()).unwrap();

    // One straight z=6 top-perimeter edge plus one bore rim circle.
    let edges = edge_handles(&mut k, plate);
    let mut perimeter = None;
    let mut rim = None;
    for &handle in &edges {
        let eid = k.resolve_edge(handle).unwrap();
        let edge = k.topo.edge(eid).unwrap();
        let a = k.topo.vertex(edge.start()).unwrap().point();
        let b = k.topo.vertex(edge.end()).unwrap().point();
        let is_circle = matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_));
        if is_circle && rim.is_none() {
            rim = Some(handle);
            continue;
        }
        let on_top = (a.z() - 6.0).abs() < 1e-9 && (b.z() - 6.0).abs() < 1e-9;
        let spans_x = (a.x() - b.x()).abs() > 1.0 && (a.y() - b.y()).abs() < 1e-9;
        if on_top && spans_x && perimeter.is_none() {
            perimeter = Some(handle);
        }
    }
    let perimeter = perimeter.expect("plate must have a straight top-perimeter edge");
    let rim = rim.expect("plate must have a circular bore rim");

    // r=4: at r=2 the per-feature path rounds both features legitimately
    // (that is the covered case); at r=4 the rim blends are beyond the bore's
    // clearance and every engine refuses the pair.
    let before = volume(&mut k, plate);
    let structured = run_v2(
        &mut k,
        &[op(
            "fillet",
            serde_json::json!({"solid": plate, "radius": 4.0, "edges": [perimeter, rim]}),
        )],
    );
    let error = &structured[0]["error"];
    assert_eq!(
        error["details"]["kernelCode"], "edges-not-blended",
        "a partially-coverable selection must be refused whole: {error}"
    );
    assert!(
        error["message"].as_str().unwrap().contains("not blended"),
        "the message must name the unblended edges: {error}"
    );
    let after_vol = volume(&mut k, plate);
    assert!(
        (after_vol - before).abs() < 1e-9,
        "the refused mixed fillet must leave the plate exactly as it was: {before} -> {after_vol}"
    );

    // The perimeter edge alone rounds cleanly (the subset answer exists — it
    // is just never passed off as the answer to the mixed request).
    let solo = run(
        &mut k,
        &[op(
            "fillet",
            serde_json::json!({"solid": plate, "radius": 2.0, "edges": [perimeter]}),
        )],
    );
    let solo_handle = u32::try_from(
        solo[0]["ok"]
            .as_u64()
            .expect("the rim-free selection rounds"),
    )
    .unwrap();
    assert_ne!(solo_handle, plate);
    let solo_volume = volume(&mut k, solo_handle);
    assert!(
        solo_volume < before,
        "rounding a convex perimeter edge removes material: {before} -> {solo_volume}"
    );
}

/// Chamfer batch arms share the same contract: typed refusal with the cause
/// in `kernelCode`, input untouched.
#[test]
fn batch_chamfer_oversized_distance_is_a_typed_refusal() {
    let mut k = BrepKernel::new();
    let solid = make_box(&mut k, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut k, solid);

    for op_name in ["chamfer", "chamferV2", "chamferDistanceAngle"] {
        let args = match op_name {
            "chamferDistanceAngle" => {
                serde_json::json!({"solid": solid, "distance": 50.0, "angle": 0.7, "edges": [edges[0]]})
            }
            "chamferV2" => {
                serde_json::json!({"solid": solid, "d1": 50.0, "d2": 50.0, "edges": [edges[0]]})
            }
            _ => serde_json::json!({"solid": solid, "distance": 50.0, "edges": [edges[0]]}),
        };
        let structured = run_v2(
            &mut k,
            &[
                op(op_name, args),
                op(
                    "volume",
                    serde_json::json!({"solid": solid, "deflection": 0.01}),
                ),
            ],
        );
        let error = &structured[0]["error"];
        assert!(
            error["details"]["kernelCode"].is_string(),
            "{op_name}: the refusal must carry a kernelCode: {error}"
        );
        let v = structured[1]["ok"].as_f64().unwrap();
        assert!(
            (v - 1000.0).abs() < 1e-9,
            "{op_name}: the refused chamfer must leave the input at 1000 mm³, got {v}"
        );
    }
}

/// A journaled fillet shares the fail-closed contract: on refusal the blend
/// AND the journal roll back together, so a subsequent journaled operation
/// sees no half-recorded state.
#[test]
fn batch_journaled_fillet_refusal_leaves_no_trace() {
    let mut k = BrepKernel::new();
    let solid = make_box(&mut k, 10.0, 10.0, 10.0);
    let edges = edge_handles(&mut k, solid);

    let journal_before = run(&mut k, &[op("journalSummary", serde_json::json!({}))]);
    let structured = run_v2(
        &mut k,
        &[
            op(
                "filletJournaled",
                serde_json::json!({"solid": solid, "radius": 50.0, "edges": [edges[0]]}),
            ),
            op(
                "volume",
                serde_json::json!({"solid": solid, "deflection": 0.01}),
            ),
        ],
    );
    let error = &structured[0]["error"];
    assert!(
        error["details"]["kernelCode"].is_string(),
        "the journaled refusal must carry a kernelCode: {error}"
    );
    let v = structured[1]["ok"].as_f64().unwrap();
    assert!(
        (v - 1000.0).abs() < 1e-9,
        "input must be untouched, got {v}"
    );

    let journal_after = run(&mut k, &[op("journalSummary", serde_json::json!({}))]);
    assert_eq!(
        journal_before[0]["ok"], journal_after[0]["ok"],
        "a refused journaled fillet must not append to the journal"
    );
}
