//! WASM contract tests for the in-place helical sweep of a circle
//! (esaueng/remus#1623), built the way OpenZCAD builds it: `makeCircleEdge`,
//! `makeWire`, `makeFaceFromWire`, `helicalSweepInPlace`.
//!
//! Before, the side was two NURBS patches joined by two tangent helical seam
//! edges, so each cap was two half circles with the same endpoints and
//! length, the seams drew as part lines along the coil, and `filletDetailed`
//! / `chamferDetailed` on a cap rim refused at every size. Now each cap is
//! one closed circle, the side is one face seamed by one self-owned edge
//! that `getSolidSmoothEdges` reports as smooth, and both blends round
//! either rim.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

use crate::kernel::RemusKernel;

fn run(kernel: &mut RemusKernel, ops: &serde_json::Value) -> Vec<serde_json::Value> {
    serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap()
}

/// The issue's coil: a radius-5 circle 25 from the Z axis in the XZ plane,
/// pitch 23.5, two turns.
fn coil(kernel: &mut RemusKernel) -> u32 {
    let edge = kernel
        .make_circle_edge(25.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0)
        .unwrap();
    let wire = kernel.make_wire_impl(&[edge], true).unwrap();
    let face = kernel.make_face_from_wire(wire).unwrap();
    kernel
        .helical_sweep_in_place_impl(face, [0.0; 3], [0.0, 0.0, 1.0], 23.5, 2.0)
        .unwrap()
}

/// `(cap rim handles, seam handle, side face)` of the coil.
fn rims_and_seam(kernel: &RemusKernel, solid: u32) -> (Vec<u32>, u32, u32) {
    let solid_id = kernel.resolve_solid(solid).unwrap();
    let adjacency = AdjacencyIndex::build(kernel.topo(), solid_id).unwrap();
    let mut rims = Vec::new();
    let mut seams = Vec::new();
    for handle in kernel.get_solid_edges(solid).unwrap() {
        let edge = kernel.resolve_edge(handle).unwrap();
        match adjacency.faces_for_edge(edge) {
            &[a, b] if a == b => seams.push((handle, a)),
            &[_, _] => rims.push(handle),
            other => panic!("edge {handle} has faces {other:?}"),
        }
    }
    assert_eq!(rims.len(), 2, "one rim per cap");
    assert_eq!(seams.len(), 1, "one seam along the coil");
    let (seam, side) = seams[0];
    (rims, seam, crate::handles::face_id_to_u32(side))
}

fn volume(kernel: &RemusKernel, solid: u32) -> f64 {
    let solid_id = kernel.resolve_solid(solid).unwrap();
    remus_operations::measure::mass_properties(kernel.topo(), solid_id)
        .unwrap()
        .mass
}

#[test]
fn circle_coil_caps_are_closed_circles_and_its_seam_is_smooth_and_self_owned() {
    let mut kernel = RemusKernel::new();
    let solid = coil(&mut kernel);
    let (rims, seam, side) = rims_and_seam(&kernel, solid);
    assert_eq!(kernel.get_solid_faces(solid).unwrap().len(), 3);

    for &rim in &rims {
        let edge = kernel
            .topo()
            .edge(kernel.resolve_edge(rim).unwrap())
            .unwrap();
        assert_eq!(edge.start(), edge.end(), "rim {rim} is one closed edge");
        let EdgeCurve::Circle(circle) = edge.curve() else {
            panic!("rim {rim} is an exact circle");
        };
        assert!((circle.radius() - 5.0).abs() < 1e-12);
    }

    // `edgeToFaceMap` lists the side face twice for its seam.
    let map: std::collections::HashMap<String, Vec<u32>> =
        serde_json::from_str(&kernel.edge_to_face_map(solid).unwrap()).unwrap();
    assert_eq!(map[&seam.to_string()], vec![side, side]);

    // The kernel's G1 verdict lists the seam, and nothing else: the rims
    // are sharp.
    assert_eq!(kernel.solid_smooth_edges(solid).unwrap(), vec![seam]);

    // The side is one B-spline face closed across the profile (clamped,
    // its two ends meeting), open along the coil.
    let parity = kernel.free_form_surface_data_parity(side).unwrap();
    assert_eq!(parity["isPeriodicU"], serde_json::json!(true));
    assert_eq!(parity["isPeriodicV"], serde_json::json!(false));

    let exact = PI * 25.0 * TAU * 25.0 * 2.0;
    let measured = volume(&kernel, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-3,
        "volume {measured} vs Pappus {exact}"
    );
}

#[test]
fn fillet_and_chamfer_detailed_round_each_cap_rim_in_a_batch() {
    for op in ["filletDetailed", "chamferDetailed"] {
        for cap in 0..2 {
            let mut kernel = RemusKernel::new();
            let solid = coil(&mut kernel);
            let (rims, ..) = rims_and_seam(&kernel, solid);
            let before = volume(&kernel, solid);
            let size_key = if op == "filletDetailed" {
                "radius"
            } else {
                "distance"
            };
            let out = run(
                &mut kernel,
                &serde_json::json!([
                    {"op": op, "args": {"solid": solid, "edges": [rims[cap]], size_key: 1.0}}
                ]),
            );
            let envelope = &out[0]["ok"];
            assert_eq!(
                envelope["status"],
                serde_json::json!("ok"),
                "{op} on cap {cap}: {}",
                out[0]
            );
            let result = u32::try_from(envelope["value"].as_u64().unwrap()).unwrap();
            let after = volume(&kernel, result);
            assert!(
                after < before - 1.0,
                "{op} on cap {cap} must remove material: {before} -> {after}"
            );
            let result_id = kernel.resolve_solid(result).unwrap();
            let report =
                remus_operations::validate::validate_solid(kernel.topo(), result_id).unwrap();
            assert!(report.is_valid(), "{op} on cap {cap}: {report:?}");
            // The new band's seam and contact rims are new edges; the coil's
            // own seam is still self-owned and smooth.
            let faces = kernel.get_solid_faces(result).unwrap();
            assert_eq!(faces.len(), 4, "{op} on cap {cap}: one band face added");
            let smooth = kernel.solid_smooth_edges(result).unwrap();
            let solid_id = kernel.resolve_solid(result).unwrap();
            let adjacency = AdjacencyIndex::build(kernel.topo(), solid_id).unwrap();
            let tube_seam_smooth = smooth.iter().any(|&handle| {
                let edge = kernel.resolve_edge(handle).unwrap();
                matches!(adjacency.faces_for_edge(edge), &[a, b] if a == b
                    && matches!(kernel.topo().face(a).unwrap().surface(), FaceSurface::Nurbs(s)
                        if s.control_points().len() > 3))
            });
            assert!(
                tube_seam_smooth,
                "{op} on cap {cap}: the tube seam stays smooth"
            );
        }
    }
}
