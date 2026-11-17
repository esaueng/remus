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

use crate::kernel::RemusKernel;

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
    let mut kernel = RemusKernel::new();
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

/// Direct `section` of an 80×40×8 plate with two r=3 through holes at
/// mid-thickness, read the way a consumer reads it: the holes are one closed
/// `CIRCLE` edge each (not 64 `LINE` chords), `faceArea` is the analytic
/// 3200 − 18π (it was the polygon's 3143.542127), and the edge and face
/// samplers work on a wire of one closed edge.
#[test]
fn direct_section_plate_with_holes_keeps_circle_edges() {
    use std::f64::consts::{PI, TAU};

    let mut kernel = RemusKernel::new();
    let mut solid = kernel.make_box_solid(80.0, 40.0, 8.0).unwrap();
    for x in [16.0, 64.0] {
        let hole = kernel.make_cylinder_solid(3.0, 12.0).unwrap();
        kernel
            .transform_solid_binding(hole, translation_matrix(x, 20.0, -2.0))
            .unwrap();
        solid = kernel.cut(solid, hole).unwrap();
    }
    let faces = kernel
        .section_solid(solid, 0.0, 0.0, 4.0, 0.0, 0.0, 1.0)
        .unwrap();
    assert_eq!(faces.len(), 1, "plate section must be one face");
    let face = faces[0];

    let outer = kernel.get_face_outer_wire(face).unwrap();
    let wires = kernel.get_face_wires(face).unwrap();
    assert_eq!(wires.len(), 3, "the outline and two holes");
    let mut centers = Vec::new();
    for &wire in &wires {
        let edges = kernel.get_wire_edges(wire).unwrap();
        if wire == outer {
            assert_eq!(edges.len(), 4, "the outline is four lines");
            for &edge in &edges {
                assert_eq!(kernel.get_edge_curve_type(edge).unwrap(), "LINE");
            }
            continue;
        }
        assert_eq!(edges.len(), 1, "a hole is one closed edge");
        let edge = edges[0];
        assert_eq!(kernel.get_edge_curve_type(edge).unwrap(), "CIRCLE");
        let ends = kernel.get_edge_vertices(edge).unwrap();
        assert_eq!(ends[..3], ends[3..], "closed on one vertex");
        let span = kernel.get_edge_param_span(edge).unwrap();
        assert!(span[0] == 0.0 && (span[1] - TAU).abs() <= 1e-12, "{span:?}");
        // Opposite points of the edge are a diameter apart, about its centre.
        let (a, b) = (
            kernel.evaluate_edge_curve(edge, 0.0).unwrap(),
            kernel.evaluate_edge_curve(edge, PI).unwrap(),
        );
        let diameter = (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt();
        assert_relative("hole diameter", diameter, 6.0, 1e-12);
        centers.push([
            f64::midpoint(a[0], b[0]),
            f64::midpoint(a[1], b[1]),
            f64::midpoint(a[2], b[2]),
        ]);
        // The span-true sampler closes the loop on the edge's vertex, with
        // every sample on the circle.
        let samples = kernel.sample_edge_polyline(edge, 0.01).unwrap();
        let count = samples.len() / 3;
        assert!(count > 8, "{count} samples");
        assert_eq!(samples[..3], samples[samples.len() - 3..]);
        let center = centers[centers.len() - 1];
        for p in samples.chunks_exact(3) {
            let radius = (p[0] - center[0]).hypot(p[1] - center[1]);
            assert!((radius - 3.0).abs() <= 1e-9 && (p[2] - 4.0).abs() <= 1e-9);
        }
    }
    assert_eq!(centers.len(), 2);
    if centers[0][0] > centers[1][0] {
        centers.swap(0, 1);
    }
    for (center, x) in centers.iter().zip([16.0, 64.0]) {
        let off = (center[0] - x)
            .hypot(center[1] - 20.0)
            .hypot(center[2] - 4.0);
        assert!(off <= 1e-12, "hole centre {center:?}");
    }

    let exact = 18.0f64.mul_add(-PI, 3200.0);
    assert_relative(
        "plate section area",
        kernel.face_area(face, 0.01).unwrap(),
        exact,
        1e-9,
    );
    // The display mesh fills the same region, short of the holes' arcs by
    // no more than its deflection along their circumference.
    let mesh = kernel.tessellate_face(face, 0.01, None).unwrap();
    let (positions, indices) = (mesh.positions(), mesh.indices());
    let corner = |i: u32| {
        let at = 3 * i as usize;
        (positions[at], positions[at + 1])
    };
    let meshed: f64 = indices
        .chunks_exact(3)
        .map(|t| {
            let (a, b, c) = (corner(t[0]), corner(t[1]), corner(t[2]));
            0.5 * ((b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1)).abs()
        })
        .sum();
    assert!(
        (meshed - exact).abs() <= 0.01 * 2.0 * TAU * 3.0,
        "mesh area {meshed} against {exact}"
    );
}

/// Batch hollow-box section through the cavity: one face, area 12, one hole.
/// Builds the hollow body through batch `cut`, so the WASM path exercises
/// the same boolean-constructed cavity the native matrix qualifies.
#[test]
fn batch_section_hollow_through_cavity_has_hole() {
    let mut kernel = RemusKernel::new();
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
    let mut kernel = RemusKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "section", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": 50.0, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    let faces = response[1]["ok"].as_array().expect("empty section").clone();
    assert!(faces.is_empty(), "miss must be an empty success");
}

#[test]
fn direct_and_batch_cylinder_sections_outside_caps_are_empty() {
    let mut kernel = RemusKernel::new();
    let solid = kernel.make_cylinder_solid(1.0, 2.0).unwrap();
    assert!(
        kernel
            .section_solid(solid, 0., 0., 3., 0., 0., 1.)
            .unwrap()
            .is_empty()
    );
    let program = serde_json::json!([
        {"op":"makeCylinder","args":{"radius":1.0,"height":2.0}},
        {"op":"section","args":{"solid":0,"px":0.0,"py":0.0,"pz":-1.0,"nx":0.0,"ny":0.0,"nz":1.0}},
        {"op":"section","args":{"solid":0,"px":0.0,"py":0.0,"pz":1.0,"nx":0.0,"ny":0.0,"nz":1.0}}
    ])
    .to_string();
    let result = parse(&kernel.execute_batch_v2(&program));
    assert!(result[1]["ok"].as_array().unwrap().is_empty());
    assert_eq!(result[2]["ok"].as_array().unwrap().len(), 1);
}

#[test]
fn batch_cylinder_cap_crossing_has_explicit_refusal() {
    let mut kernel = RemusKernel::new();
    let program = serde_json::json!([
        {"op":"makeCylinder","args":{"radius":1.0,"height":2.0}},
        {"op":"section","args":{"solid":0,"px":0.0,"py":0.0,"pz":0.1,"nx":1.0,"ny":0.0,"nz":1.0}}
    ])
    .to_string();
    let result = parse(&kernel.execute_batch_v2(&program));
    assert!(
        result[1]["error"].to_string().contains("cap clipping"),
        "{result}"
    );
}

/// Batch edge-touch section refuses typed (`InvalidInput`, no closed wire).
#[test]
fn batch_section_edge_touch_refuses_typed() {
    let mut kernel = RemusKernel::new();
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
    let mut direct = RemusKernel::new();
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
    let mut batch = RemusKernel::new();
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
    let mut kernel = RemusKernel::new();
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
    let mut kernel = RemusKernel::new();
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

/// #1745: `split` through a cylinder's curved edges used to throw "crosses
/// curved edge 0". It now returns both halves through the exact half-space
/// boolean route, direct and batch bit-identical, each half valid with its
/// closed-form volume.
#[test]
fn split_cylinder_through_curved_edges_returns_both_halves() {
    let full = std::f64::consts::PI * 100.0 * 20.0;
    let segment = (100.0 * 0.3_f64.acos() - 3.0 * 91.0_f64.sqrt()) * 20.0;
    for (px, pos_expected, neg_expected) in [
        (0.0, full / 2.0, full / 2.0),
        (3.0, segment, full - segment),
    ] {
        let mut direct = RemusKernel::new();
        let solid = direct.make_cylinder_solid(10.0, 20.0).unwrap();
        let halves = direct
            .split_solid(solid, px, 0.0, 0.0, 1.0, 0.0, 0.0)
            .unwrap();
        assert_eq!(halves.len(), 2, "split must return two halves");
        let pos = direct.resolve_solid(halves[0]).unwrap();
        let neg = direct.resolve_solid(halves[1]).unwrap();
        let direct_pos = solid_volume(direct.topo(), pos, 0.01).unwrap();
        let direct_neg = solid_volume(direct.topo(), neg, 0.01).unwrap();
        assert_relative(&format!("x={px} pos"), direct_pos, pos_expected, 1e-9);
        assert_relative(&format!("x={px} neg"), direct_neg, neg_expected, 1e-9);

        // The fallback mints (and retires) its half-space tools, so the
        // halves' handles come from the split's own answer, not from a
        // presumed allocation order.
        let mut batch = RemusKernel::new();
        let program = serde_json::json!([
            {"op": "makeCylinder", "args": {"radius": 10.0, "height": 20.0}},
            {"op": "split", "args": {"solid": 0, "px": px, "py": 0.0, "pz": 0.0, "nx": 1.0, "ny": 0.0, "nz": 0.0}},
        ])
        .to_string();
        let split = parse(&batch.execute_batch_v2(&program));
        assert!(
            split[1].get("error").is_none(),
            "x={px}: batch split must succeed, got {split:?}"
        );
        let pos_handle = split[1]["ok"]["positive"]
            .as_u64()
            .expect("positive handle");
        let neg_handle = split[1]["ok"]["negative"]
            .as_u64()
            .expect("negative handle");
        let program = serde_json::json!([
            {"op": "volume", "args": {"solid": pos_handle, "deflection": 0.01}},
            {"op": "volume", "args": {"solid": neg_handle, "deflection": 0.01}},
            {"op": "validateSolid", "args": {"solid": pos_handle}},
            {"op": "validateSolid", "args": {"solid": neg_handle}},
        ])
        .to_string();
        let response = parse(&batch.execute_batch_v2(&program));
        let batch_pos = response[0]["ok"].as_f64().expect("batch pos");
        let batch_neg = response[1]["ok"].as_f64().expect("batch neg");
        assert!(
            batch_pos.to_bits() == direct_pos.to_bits(),
            "x={px}: batch pos {batch_pos} must equal direct {direct_pos}"
        );
        assert!(
            batch_neg.to_bits() == direct_neg.to_bits(),
            "x={px}: batch neg {batch_neg} must equal direct {direct_neg}"
        );
        assert_eq!(response[2]["ok"], serde_json::json!(0), "pos validates");
        assert_eq!(response[3]["ok"], serde_json::json!(0), "neg validates");
    }
}

/// A plane lying on a face of the body (z = 0 on a box based at z = 0) still
/// refuses typed, and now says the plane does not cross the interior.
#[test]
fn batch_split_on_a_face_says_it_does_not_cross_the_interior() {
    let mut kernel = RemusKernel::new();
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 4.0, "height": 4.0, "depth": 4.0}},
        {"op": "split", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": 0.0, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
        {"op": "volume", "args": {"solid": 0, "deflection": 0.01}},
    ])
    .to_string();
    let response = parse(&kernel.execute_batch_v2(&program));
    let message = response[1]["error"].to_string();
    assert!(
        message.contains("split") && message.contains("does not cross the solid's interior"),
        "refusal must say the plane does not cross the interior, got {response:?}"
    );
    assert_relative(
        "input volume after refused split",
        response[2]["ok"].as_f64().expect("volume after refusal"),
        64.0,
        1e-9,
    );
}
