//! Direct/batch WASM qualification for the B6 plane-section/split matrix.
//!
//! Same closed-form oracles as the native qualification suite
//! (`crates/operations/tests/qualify_section_split.rs`), exercised through
//! direct kernel calls on success paths and through `execute_batch_v2`
//! everywhere (including typed refusals): `JsError` cannot be constructed on
//! non-wasm targets, so direct `#[wasm_bindgen]` methods are only callable
//! on their success paths. Batch error payloads carry the typed operation
//! name so refusal cells assert the contract, not a bare failure.
//!
//! Representative cells (unit scale, origin placement unless noted): box
//! transverse section, hollow-box cavity section with an explicit hole,
//! empty miss, edge-touch degenerate refusal, box split halves, hollow-box
//! cavity-split refusal with session preservation, and plate clear-of-hole
//! split parity. Full scale/placement matrices live natively.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use remus_operations::measure::{face_area, solid_volume};

use crate::kernel::BrepKernel;

fn assert_relative(label: &str, actual: f64, expected: f64, limit: f64) {
    let relative = (actual - expected).abs() / expected.abs().max(1e-300);
    assert!(
        relative <= limit,
        "{label}: expected {expected:.12e}, got {actual:.12e}, relative error {relative:.3e} > {limit:.3e}"
    );
}

fn parse(response: &str) -> serde_json::Value {
    serde_json::from_str(response).expect("batch response must be valid JSON")
}

fn translation_matrix(tx: f64, ty: f64, tz: f64) -> Vec<f64> {
    vec![
        1.0, 0.0, 0.0, tx, //
        0.0, 1.0, 0.0, ty, //
        0.0, 0.0, 1.0, tz, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

/// Direct `section` on a unit 4-box at z=2: one face, area 16, on the plane.
#[test]
fn direct_section_box_transverse_matches_area() {
    let mut kernel = BrepKernel::new();
    let solid = kernel.make_box_solid(4.0, 4.0, 4.0).unwrap();
    let faces = kernel
        .section_solid(solid, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0)
        .unwrap();
    assert_eq!(faces.len(), 1, "box section must be one face");
    let fid = kernel.resolve_face(faces[0]).unwrap();
    let area = face_area(kernel.topo(), fid, 0.01).expect("section area");
    assert_relative("direct section box area", area, 16.0, 1e-9);
    assert_eq!(
        kernel.validate_solid(solid).unwrap(),
        0,
        "input stays valid"
    );
}

/// Batch hollow-box section through the cavity: one face, area 12, one hole.
/// Builds the hollow body through batch `cut`, so the WASM path exercises
/// the same boolean-constructed cavity the native matrix qualifies.
#[test]
fn batch_section_hollow_through_cavity_has_hole() {
    let mut kernel = BrepKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
        {"op": "transform", "args": {"solid": 1, "matrix": translation_matrix(1.0, 1.0, 1.0)}},
        {"op": "cut", "args": {"solidA": 0, "solidB": 1}},
        {"op": "volume", "args": {"solid": 2, "deflection": 0.01}},
        {"op": "section", "args": {"solid": 2, "px": 0.0, "py": 0.0, "pz": 2.0, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    assert_relative(
        "hollow input volume",
        response[4]["ok"].as_f64().expect("hollow volume"),
        56.0,
        1e-9,
    );
    let faces = response[5]["ok"].as_array().expect("section faces").clone();
    assert_eq!(faces.len(), 1, "cavity section must be one face");
    let handle = faces[0].as_u64().expect("face handle") as u32;
    let fid = kernel.resolve_face(handle).unwrap();
    let area = face_area(kernel.topo(), fid, 0.01).expect("section area");
    assert_relative("batch cavity section area", area, 12.0, 1e-9);
    assert_eq!(
        kernel.topo().face(fid).unwrap().inner_wires().len(),
        1,
        "cavity section must carry its hole"
    );
}

/// Batch miss returns the documented empty-face-list success.
#[test]
fn batch_section_miss_returns_empty() {
    let mut kernel = BrepKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "section", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": 50.0, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    let faces = response[1]["ok"].as_array().expect("empty section").clone();
    assert!(faces.is_empty(), "miss must be an empty success");
}

/// Batch edge-touch section refuses typed (`InvalidInput`, no closed wire).
#[test]
fn batch_section_edge_touch_refuses_typed() {
    let mut kernel = BrepKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "section", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": 2.0, "nx": 1.0, "ny": 1.0, "nz": 0.0}},
        {"op": "volume", "args": {"solid": 0, "deflection": 0.01}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    assert!(
        response[1].get("error").is_some(),
        "edge touch must refuse, got {response:?}"
    );
    let message = response[1]["error"].to_string();
    assert!(
        message.contains("no closed cross-section"),
        "refusal must name the assembly failure, got {message}"
    );
    // The logical session survives the refusal.
    assert_relative(
        "input volume after refused section",
        response[2]["ok"].as_f64().expect("volume after refusal"),
        64.0,
        1e-9,
    );
}

/// Direct `split` on a unit 4-box at z=2: 32 + 32, direct/batch parity.
#[test]
fn direct_and_batch_split_box_match_halves() {
    // Direct.
    let mut direct = BrepKernel::new();
    let solid = direct.make_box_solid(4.0, 4.0, 4.0).unwrap();
    let halves = direct
        .split_solid(solid, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0)
        .unwrap();
    assert_eq!(halves.len(), 2, "split must return two halves");
    let pos = direct.resolve_solid(halves[0]).unwrap();
    let neg = direct.resolve_solid(halves[1]).unwrap();
    let direct_pos = solid_volume(direct.topo(), pos, 0.01).expect("pos volume");
    let direct_neg = solid_volume(direct.topo(), neg, 0.01).expect("neg volume");
    assert_relative("direct split pos", direct_pos, 32.0, 1e-9);
    assert_relative("direct split neg", direct_neg, 32.0, 1e-9);

    // Batch: the same cut through `execute_batch_v2`, pinned to direct bits.
    let mut batch = BrepKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "split", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": 2.0, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
        {"op": "volume", "args": {"solid": 1, "deflection": 0.01}},
        {"op": "volume", "args": {"solid": 2, "deflection": 0.01}},
        {"op": "validateSolid", "args": {"solid": 1}},
        {"op": "validateSolid", "args": {"solid": 2}},
    ])
    .to_string();
    let response = parse(&batch.execute_batch_v2(&program));
    let batch_pos = response[2]["ok"].as_f64().expect("batch pos");
    let batch_neg = response[3]["ok"].as_f64().expect("batch neg");
    assert!(
        batch_pos.to_bits() == direct_pos.to_bits(),
        "batch pos {batch_pos} must equal direct {direct_pos} bit-for-bit"
    );
    assert!(
        batch_neg.to_bits() == direct_neg.to_bits(),
        "batch neg {batch_neg} must equal direct {direct_neg} bit-for-bit"
    );
    assert_eq!(response[4]["ok"], serde_json::json!(0), "pos validates");
    assert_eq!(response[5]["ok"], serde_json::json!(0), "neg validates");
}

/// Batch cavity-split refusal preserves the session: the hollow input still
/// measures 56 afterwards.
#[test]
fn batch_split_cavity_refuses_typed_and_preserves_session() {
    let mut kernel = BrepKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
        {"op": "transform", "args": {"solid": 1, "matrix": translation_matrix(1.0, 1.0, 1.0)}},
        {"op": "cut", "args": {"solidA": 0, "solidB": 1}},
        {"op": "split", "args": {"solid": 2, "px": 0.0, "py": 0.0, "pz": 0.5, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
        {"op": "volume", "args": {"solid": 2, "deflection": 0.01}},
        {"op": "validateSolid", "args": {"solid": 2}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    assert!(
        response[4].get("error").is_some(),
        "cavity split must refuse, got {response:?}"
    );
    let message = response[4]["error"].to_string();
    assert!(
        message.contains("split") && message.contains("cavity"),
        "refusal must name the split cavity contract, got {message}"
    );
    assert_relative(
        "input volume after refused split",
        response[5]["ok"].as_f64().expect("volume after refusal"),
        56.0,
        1e-9,
    );
    assert_eq!(
        response[6]["ok"],
        serde_json::json!(0),
        "input still validates"
    );
}

/// Batch plate split clear of the hole: 172 + 20 with direct parity on the
/// negative half (the thin slab most sensitive to plane placement).
#[test]
fn batch_split_plate_clear_of_hole_matches_closed_form() {
    let mut kernel = BrepKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 2.0}},
        {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 4.0}},
        {"op": "transform", "args": {"solid": 1, "matrix": translation_matrix(4.0, 4.0, -1.0)}},
        {"op": "cut", "args": {"solidA": 0, "solidB": 1}},
        {"op": "split", "args": {"solid": 2, "px": 1.0, "py": 0.0, "pz": 0.0, "nx": 1.0, "ny": 0.0, "nz": 0.0}},
        {"op": "volume", "args": {"solid": 3, "deflection": 0.01}},
        {"op": "volume", "args": {"solid": 4, "deflection": 0.01}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    assert_relative(
        "plate clear split pos",
        response[5]["ok"].as_f64().expect("pos"),
        172.0,
        1e-9,
    );
    assert_relative(
        "plate clear split neg",
        response[6]["ok"].as_f64().expect("neg"),
        20.0,
        1e-9,
    );
}
