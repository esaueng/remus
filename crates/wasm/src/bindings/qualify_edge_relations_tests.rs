//! B16 contract tests for per-edge convexity and face material sense.
//!
//! Every kernel call goes through `execute_batch`: `JsError` cannot be
//! constructed on non-wasm targets, so the `#[wasm_bindgen]` methods are
//! not directly testable on their error paths. Success paths additionally
//! compare the batch payload against the shared `*_impl` fns the direct
//! bindings call, so batch == direct by construction and both match the
//! native `remus_operations::query` fixtures.
//!
//! Fixture table (edge → relation, face → sense):
//!
//! | Fixture | Edge | Expected relation | Face | Expected sense |
//! |---|---|---|---|---|
//! | unit box | all 12 line edges | `convex`, dihedral `+pi/2` | — | — |
//! | box minus pocket | 4 floor–wall edges | `concave`, dihedral `-pi/2` | — | — |
//! | plate + boss | foot rim (plate top) | `concave` | boss wall | `outward` |
//! | plate + boss | top rim (boss cap) | `convex` | — | — |
//! | through-bore | top + bottom rims | `convex` (`+pi/2`) | bore wall | `inward` |
//! | blind hole | opening rim | `convex` | hole wall | `inward` |
//! | blind hole | floor rim (wall meets floor) | `concave` (`-pi/2`) | — | — |
//! | filleted box | 2 spring contacts | `tangent` (`~0`) | — | — |
//! | cone frustum | base rim (wall meets base) | `convex` | cone wall | `outward` |
//! | cylinder primitive | wall seam | `unknown`, angle `null` | — | — |
//! | box face | — | — | plane face | typed unsupported |
//!
//! The probe default is per-edge `0.05 * local_scale` with
//! `local_scale = max(min(face spans), edge span)`; an explicit probe above
//! 25 % of the local scale reports `unknown`, and a zero or negative probe
//! is `InvalidInput`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;
use crate::types::{EdgeConvexityRelation, FaceMaterialSense};

fn run(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&k.execute_batch(&json)).unwrap()
}

fn run_all_ok(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let results = run(k, ops);
    for (i, r) in results.iter().enumerate() {
        assert!(
            r.get("ok").is_some(),
            "op {i} ({}) failed: {r}",
            ops[i]["op"]
        );
    }
    results
        .into_iter()
        .map(|r| r.get("ok").cloned().unwrap())
        .collect()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn as_u32(v: &serde_json::Value) -> u32 {
    u32::try_from(v.as_u64().unwrap()).unwrap()
}

fn batch_solid(k: &mut BrepKernel, make: &str, args: serde_json::Value) -> u32 {
    as_u32(&run_all_ok(k, &[op(make, args)])[0])
}

fn batch_edges(k: &mut BrepKernel, solid: u32) -> Vec<u32> {
    let out = run_all_ok(k, &[op("solidEdges", serde_json::json!({"solid": solid}))]);
    out[0]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| u32::try_from(v.as_u64().unwrap()).unwrap())
        .collect()
}

/// One `edgeConvexity` batch cell as `(relation, dihedral_angle)`.
fn batch_convexity(k: &mut BrepKernel, solid: u32, edge: u32) -> (String, Option<f64>) {
    let out = run_all_ok(
        k,
        &[op(
            "edgeConvexity",
            serde_json::json!({"solid": solid, "edge": edge}),
        )],
    );
    let relation = out[0]["relation"].as_str().unwrap().to_string();
    let angle = out[0]["dihedralAngle"].as_f64();
    (relation, angle)
}

/// Direct `*_impl` verdict for the same cell (batch == direct).
fn direct_convexity(k: &BrepKernel, solid: u32, edge: u32) -> (String, Option<f64>) {
    let result = k.edge_convexity_impl(solid, edge, None).unwrap();
    let relation = match result.relation {
        EdgeConvexityRelation::Convex => "convex",
        EdgeConvexityRelation::Concave => "concave",
        EdgeConvexityRelation::Tangent => "tangent",
        EdgeConvexityRelation::Unknown => "unknown",
    }
    .to_string();
    (relation, result.dihedral_angle)
}

fn assert_batch_matches_direct(k: &mut BrepKernel, solid: u32, edge: u32) {
    let batch = batch_convexity(k, solid, edge);
    let direct = direct_convexity(k, solid, edge);
    assert_eq!(
        batch.0, direct.0,
        "batch != direct relation for edge {edge}"
    );
    match (batch.1, direct.1) {
        (Some(b), Some(d)) => assert!(
            (b - d).abs() < 1e-12,
            "batch != direct angle for edge {edge}: {b} vs {d}"
        ),
        (None, None) => {}
        (b, d) => panic!("batch != direct angle presence for edge {edge}: {b:?} vs {d:?}"),
    }
}

fn transform_rigid(k: &mut BrepKernel, solid: u32, scale: f64) {
    let c = 0.37_f64.cos();
    let s = 0.37_f64.sin();
    let _ = run_all_ok(
        k,
        &[op(
            "transform",
            serde_json::json!({"solid": solid, "matrix": [
                c, 0.0, s, 17.0 * scale,
                0.0, 1.0, 0.0, -23.0 * scale,
                -s, 0.0, c, 31.0 * scale,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
}

fn circle_rim_at_z(k: &BrepKernel, solid: u32, z: f64, tol: f64) -> u32 {
    let solid_id = k.resolve_solid(solid).unwrap();
    remus_topology::explorer::solid_edges(k.topo(), solid_id)
        .unwrap()
        .into_iter()
        .find(|&eid| {
            let data = k.topo().edge(eid).unwrap();
            matches!(data.curve(), remus_topology::edge::EdgeCurve::Circle(_))
                && (k.topo().vertex(data.start()).unwrap().point().z() - z).abs() < tol
        })
        .map(crate::handles::edge_id_to_u32)
        .expect("circle rim")
}

fn cylinder_walls(k: &BrepKernel, solid: u32) -> Vec<u32> {
    let solid_id = k.resolve_solid(solid).unwrap();
    remus_topology::explorer::solid_faces(k.topo(), solid_id)
        .unwrap()
        .into_iter()
        .filter(|&fid| {
            matches!(
                k.topo().face(fid).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .map(crate::handles::face_id_to_u32)
        .collect()
}

fn batch_sense(k: &mut BrepKernel, solid: u32, face: u32) -> String {
    run_all_ok(
        k,
        &[op(
            "faceMaterialSense",
            serde_json::json!({"solid": solid, "face": face}),
        )],
    )[0]
    .as_str()
    .unwrap()
    .to_string()
}

#[test]
fn batch_box_edges_convex_with_pi_half() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        let mut k = BrepKernel::new();
        let solid = batch_solid(
            &mut k,
            "makeBox",
            serde_json::json!({"width": scale, "height": scale, "depth": scale}),
        );
        let edges = batch_edges(&mut k, solid);
        assert_eq!(edges.len(), 12);
        for &edge in &edges {
            let (relation, angle) = batch_convexity(&mut k, solid, edge);
            assert_eq!(relation, "convex");
            let angle = angle.expect("convex carries +pi/2");
            assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
            assert_batch_matches_direct(&mut k, solid, edge);
        }
        // Bulk rows agree edge-for-edge.
        let rows = run_all_ok(
            &mut k,
            &[op(
                "solidEdgeRelations",
                serde_json::json!({"solid": solid}),
            )],
        );
        let rows = rows[0].as_array().unwrap();
        assert_eq!(rows.len(), 12);
        for row in rows {
            assert_eq!(row["relation"], "convex");
            assert!(
                (row["dihedralAngle"].as_f64().unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-9
            );
        }
        // Direct bulk impl agrees with the batch rows.
        let direct_rows = k.solid_edge_relations_impl(solid, None).unwrap();
        assert_eq!(direct_rows.len(), rows.len());
        for (direct, batch) in direct_rows.iter().zip(rows) {
            let expected = match direct.relation {
                EdgeConvexityRelation::Convex => "convex",
                EdgeConvexityRelation::Concave => "concave",
                EdgeConvexityRelation::Tangent => "tangent",
                EdgeConvexityRelation::Unknown => "unknown",
            };
            assert_eq!(batch["relation"], expected);
        }
    }
}

#[test]
fn batch_box_convexity_survives_rigid_placement() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 4.0, "height": 2.0, "depth": 1.0}),
    );
    transform_rigid(&mut k, solid, 1.0);
    for &edge in &batch_edges(&mut k, solid) {
        let (relation, angle) = batch_convexity(&mut k, solid, edge);
        assert_eq!(relation, "convex");
        assert!((angle.unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert_batch_matches_direct(&mut k, solid, edge);
    }
}

#[test]
fn batch_boss_foot_concave_top_convex_wall_outward() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        let mut k = BrepKernel::new();
        let plate = batch_solid(
            &mut k,
            "makeBox",
            serde_json::json!({"width": 80.0 * scale, "height": 40.0 * scale, "depth": 8.0 * scale}),
        );
        let post = batch_solid(
            &mut k,
            "makeCylinder",
            serde_json::json!({"radius": 10.0 * scale, "height": 32.0 * scale}),
        );
        run_all_ok(
            &mut k,
            &[op(
                "transform",
                serde_json::json!({"solid": post, "matrix": [
                    1.0, 0.0, 0.0, 40.0 * scale,
                    0.0, 1.0, 0.0, 20.0 * scale,
                    0.0, 0.0, 1.0, 8.0 * scale,
                    0.0, 0.0, 0.0, 1.0,
                ]}),
            )],
        );
        let posted = as_u32(
            &run_all_ok(
                &mut k,
                &[op(
                    "fuse",
                    serde_json::json!({"solidA": plate, "solidB": post}),
                )],
            )[0],
        );
        let tol = 1e-9 * scale.max(1.0);
        let foot = circle_rim_at_z(&k, posted, 8.0 * scale, tol);
        let top = circle_rim_at_z(&k, posted, 40.0 * scale, tol);
        assert_eq!(batch_convexity(&mut k, posted, foot).0, "concave");
        assert_eq!(batch_convexity(&mut k, posted, top).0, "convex");
        assert_batch_matches_direct(&mut k, posted, foot);
        assert_batch_matches_direct(&mut k, posted, top);
        for wall in cylinder_walls(&k, posted) {
            assert_eq!(batch_sense(&mut k, posted, wall), "outward");
            assert_eq!(
                k.face_material_sense_impl(posted, wall).unwrap(),
                FaceMaterialSense::Outward
            );
        }
    }
}

#[test]
fn batch_through_bore_rims_convex_wall_inward() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        let mut k = BrepKernel::new();
        let plate = batch_solid(
            &mut k,
            "makeBox",
            serde_json::json!({"width": 20.0 * scale, "height": 20.0 * scale, "depth": 6.0 * scale}),
        );
        let drill = batch_solid(
            &mut k,
            "makeCylinder",
            serde_json::json!({"radius": 3.0 * scale, "height": 10.0 * scale}),
        );
        run_all_ok(
            &mut k,
            &[op(
                "transform",
                serde_json::json!({"solid": drill, "matrix": [
                    1.0, 0.0, 0.0, 10.0 * scale,
                    0.0, 1.0, 0.0, 10.0 * scale,
                    0.0, 0.0, 1.0, -2.0 * scale,
                    0.0, 0.0, 0.0, 1.0,
                ]}),
            )],
        );
        let bored = as_u32(
            &run_all_ok(
                &mut k,
                &[op(
                    "cut",
                    serde_json::json!({"solidA": plate, "solidB": drill}),
                )],
            )[0],
        );
        let tol = 1e-9 * scale.max(1.0);
        // A 90-degree material wedge at both rims: convex, while the wall
        // itself is inward. The two questions must not be collapsed.
        for z in [0.0, 6.0 * scale] {
            let rim = circle_rim_at_z(&k, bored, z, tol);
            let (relation, angle) = batch_convexity(&mut k, bored, rim);
            assert_eq!(relation, "convex", "bore rim at z={z}");
            assert!((angle.unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
            assert_batch_matches_direct(&mut k, bored, rim);
        }
        for wall in cylinder_walls(&k, bored) {
            assert_eq!(batch_sense(&mut k, bored, wall), "inward");
        }
    }
}

#[test]
fn batch_blind_hole_floor_concave() {
    let mut k = BrepKernel::new();
    let plate = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 20.0, "height": 20.0, "depth": 6.0}),
    );
    let drill = batch_solid(
        &mut k,
        "makeCylinder",
        serde_json::json!({"radius": 3.0, "height": 4.0}),
    );
    run_all_ok(
        &mut k,
        &[op(
            "transform",
            serde_json::json!({"solid": drill, "matrix": [
                1.0, 0.0, 0.0, 10.0,
                0.0, 1.0, 0.0, 10.0,
                0.0, 0.0, 1.0, 2.0,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    let blind = as_u32(
        &run_all_ok(
            &mut k,
            &[op(
                "cut",
                serde_json::json!({"solidA": plate, "solidB": drill}),
            )],
        )[0],
    );
    let opening = circle_rim_at_z(&k, blind, 6.0, 1e-9);
    let floor = circle_rim_at_z(&k, blind, 2.0, 1e-9);
    assert_eq!(batch_convexity(&mut k, blind, opening).0, "convex");
    let (relation, angle) = batch_convexity(&mut k, blind, floor);
    assert_eq!(relation, "concave");
    assert!(angle.unwrap() < 0.0);
    assert_batch_matches_direct(&mut k, blind, floor);
}

#[test]
fn batch_pocket_floor_walls_concave() {
    let mut k = BrepKernel::new();
    let base = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 20.0, "height": 20.0, "depth": 10.0}),
    );
    let tool = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 10.0, "height": 10.0, "depth": 6.0}),
    );
    run_all_ok(
        &mut k,
        &[op(
            "transform",
            serde_json::json!({"solid": tool, "matrix": [
                1.0, 0.0, 0.0, 5.0,
                0.0, 1.0, 0.0, 5.0,
                0.0, 0.0, 1.0, 4.0,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    let pocket = as_u32(
        &run_all_ok(
            &mut k,
            &[op(
                "cut",
                serde_json::json!({"solidA": base, "solidB": tool}),
            )],
        )[0],
    );
    let rows = run_all_ok(
        &mut k,
        &[op(
            "solidEdgeRelations",
            serde_json::json!({"solid": pocket}),
        )],
    );
    let concave = rows[0]
        .as_array()
        .unwrap()
        .iter()
        .filter(|row| row["relation"] == "concave")
        .count();
    assert!(concave >= 4, "pocket floor must contribute concave edges");
}

#[test]
fn batch_fillet_band_edges_tangent() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 10.0, "height": 10.0, "depth": 10.0}),
    );
    let edges = batch_edges(&mut k, solid);
    let filleted = as_u32(
        &run_all_ok(
            &mut k,
            &[op(
                "fillet",
                serde_json::json!({"solid": solid, "edges": [edges[0]], "radius": 1.0}),
            )],
        )[0],
    );
    let rows = run_all_ok(
        &mut k,
        &[op(
            "solidEdgeRelations",
            serde_json::json!({"solid": filleted}),
        )],
    );
    let tangent: Vec<_> = rows[0]
        .as_array()
        .unwrap()
        .iter()
        .filter(|row| row["relation"] == "tangent")
        .collect();
    assert_eq!(tangent.len(), 2, "one spring contact per band side");
    for row in tangent {
        assert!(row["dihedralAngle"].as_f64().unwrap().abs() < 1e-6);
    }
}

#[test]
fn batch_cone_rim_convex_wall_outward() {
    let mut k = BrepKernel::new();
    let cone = batch_solid(
        &mut k,
        "makeCone",
        serde_json::json!({"bottomRadius": 2.0, "topRadius": 1.0, "height": 2.0}),
    );
    let rim = circle_rim_at_z(&k, cone, 0.0, 1e-9);
    let (relation, angle) = batch_convexity(&mut k, cone, rim);
    assert_eq!(relation, "convex");
    let angle = angle.unwrap();
    assert!(angle > 0.0 && angle < std::f64::consts::PI);
    assert_batch_matches_direct(&mut k, cone, rim);
}

#[test]
fn batch_seam_is_unknown_never_a_guess() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeCylinder",
        serde_json::json!({"radius": 2.0, "height": 4.0}),
    );
    let rows = run_all_ok(
        &mut k,
        &[op(
            "solidEdgeRelations",
            serde_json::json!({"solid": solid}),
        )],
    );
    let unknown: Vec<_> = rows[0]
        .as_array()
        .unwrap()
        .iter()
        .filter(|row| row["relation"] == "unknown")
        .collect();
    assert!(!unknown.is_empty(), "the wall seam must be unknown");
    for row in unknown {
        assert!(row["dihedralAngle"].is_null(), "unknown carries no angle");
    }
}

#[test]
fn batch_refuses_foreign_and_deleted_handles_naming_them() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
    );
    let edge = batch_edges(&mut k, solid)[0];
    // Foreign handles.
    for (name, args) in [
        (
            "edgeConvexity",
            serde_json::json!({"solid": solid, "edge": 999_999_u32}),
        ),
        (
            "edgeConvexity",
            serde_json::json!({"solid": 999_999_u32, "edge": edge}),
        ),
        (
            "solidEdgeRelations",
            serde_json::json!({"solid": 999_999_u32}),
        ),
        (
            "faceMaterialSense",
            serde_json::json!({"solid": solid, "face": 999_999_u32}),
        ),
    ] {
        let results = run(&mut k, &[op(name, args)]);
        assert!(
            results[0].get("error").is_some(),
            "{name} must refuse a foreign handle: {results:?}"
        );
        let message = results[0]["error"].as_str().unwrap_or_default();
        assert!(
            message.contains("invalid"),
            "{name} refusal must be typed, got: {message}"
        );
    }
    // Deleted solid handle names the handle (`deleteSolid` has no batch
    // arm; the direct call succeeds natively and retires the handle).
    k.delete_solid(solid).unwrap();
    let results = run(
        &mut k,
        &[op(
            "solidEdgeRelations",
            serde_json::json!({"solid": solid}),
        )],
    );
    assert!(results[0].get("error").is_some());
}

/// Two valid solids in one session: an existing edge or face of one is
/// refused when queried against the other, on both the batch and the direct
/// path, instead of reading as an `unknown` relation.
#[test]
fn batch_and_direct_refuse_an_entity_of_another_valid_solid() {
    let mut k = BrepKernel::new();
    let boxed = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
    );
    let cylinder = batch_solid(
        &mut k,
        "makeCylinder",
        serde_json::json!({"radius": 1.0, "height": 2.0}),
    );
    let box_edge = batch_edges(&mut k, boxed)[0];
    let cylinder_edge = batch_edges(&mut k, cylinder)[0];
    for (solid, edge) in [(cylinder, box_edge), (boxed, cylinder_edge)] {
        let results = run(
            &mut k,
            &[op(
                "edgeConvexity",
                serde_json::json!({"solid": solid, "edge": edge}),
            )],
        );
        let message = results[0]["error"].as_str().unwrap_or_default();
        assert!(
            message.contains("not part of the solid"),
            "batch edgeConvexity must refuse edge {edge} of another solid: {results:?}"
        );
        let direct = k.edge_convexity_impl(solid, edge, None);
        assert!(
            direct.is_err(),
            "direct edgeConvexity must refuse edge {edge} of another solid: {direct:?}"
        );
    }
    let wall = cylinder_walls(&k, cylinder)[0];
    let results = run(
        &mut k,
        &[op(
            "faceMaterialSense",
            serde_json::json!({"solid": boxed, "face": wall}),
        )],
    );
    let message = results[0]["error"].as_str().unwrap_or_default();
    assert!(
        message.contains("not part of the solid"),
        "batch faceMaterialSense must refuse a face of another solid: {results:?}"
    );
    assert!(k.face_material_sense_impl(boxed, wall).is_err());
    // Each solid still answers for its own entities.
    assert_eq!(batch_convexity(&mut k, boxed, box_edge).0, "convex");
    assert_eq!(batch_sense(&mut k, cylinder, wall), "outward");
}

#[test]
fn batch_refuses_bad_probes_and_plane_sense() {
    let mut k = BrepKernel::new();
    let solid = batch_solid(
        &mut k,
        "makeBox",
        serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
    );
    let edge = batch_edges(&mut k, solid)[0];
    for probe in [0.0, -1.0] {
        for (name, args) in [
            (
                "edgeConvexity",
                serde_json::json!({"solid": solid, "edge": edge, "probe": probe}),
            ),
            (
                "solidEdgeRelations",
                serde_json::json!({"solid": solid, "probe": probe}),
            ),
        ] {
            let results = run(&mut k, &[op(name, args)]);
            assert!(
                results[0].get("error").is_some(),
                "{name} probe={probe} must be InvalidInput: {results:?}"
            );
        }
    }
    // Oversized probe is Unknown, never a guess.
    let out = run_all_ok(
        &mut k,
        &[op(
            "edgeConvexity",
            serde_json::json!({"solid": solid, "edge": edge, "probe": 100.0}),
        )],
    );
    assert_eq!(out[0]["relation"], "unknown");
    assert!(out[0]["dihedralAngle"].is_null());
    // Plane faces refuse material sense with a typed unsupported.
    let faces = run_all_ok(
        &mut k,
        &[op("getSolidFaces", serde_json::json!({"solid": solid}))],
    );
    let face = as_u32(&faces[0].as_array().unwrap()[0]);
    let results = run(
        &mut k,
        &[op(
            "faceMaterialSense",
            serde_json::json!({"solid": solid, "face": face}),
        )],
    );
    assert!(
        results[0].get("error").is_some(),
        "plane face must refuse material sense: {results:?}"
    );
}
