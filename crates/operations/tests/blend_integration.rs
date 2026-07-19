//! End-to-end integration tests for the validated fillet and chamfer APIs.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_operations::blend_ops::{chamfer_distance_angle, chamfer_v2, fillet_v2};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::{make_box, make_cone, make_cylinder};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::explorer::{solid_edges, solid_faces};

const BOX_VOLUME: f64 = 1000.0; // 10 x 10 x 10

/// Create a 10x10x10 box and fillet a single edge.
#[test]
fn fillet_box_single_edge() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    assert!(!edges.is_empty(), "box must have edges");

    let result = fillet_v2(&mut topo, solid, &edges[..1], 1.0).unwrap();

    let faces = solid_faces(&topo, result.solid).unwrap();
    assert!(
        faces.len() > 6,
        "filleted box should have more than 6 faces"
    );
    assert!(
        !result.succeeded.is_empty(),
        "at least one edge should succeed"
    );

    let vol = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - BOX_VOLUME).abs() > 0.01,
        "filleted volume {vol} should differ from original {BOX_VOLUME}"
    );
}

/// Fillet 4 edges of a box (e.g. the first 4 found).
#[test]
fn fillet_box_multiple_edges() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    let n = edges.len().min(4);
    let target = &edges[..n];

    let result = fillet_v2(&mut topo, solid, target, 0.5).unwrap();
    assert!(
        !result.succeeded.is_empty(),
        "at least some edges should succeed"
    );

    let vol = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - BOX_VOLUME).abs() > 0.01,
        "filleted volume {vol} should differ from original {BOX_VOLUME}"
    );
}

/// Symmetric chamfer on a single edge.
#[test]
fn chamfer_box_symmetric() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    let result = chamfer_v2(&mut topo, solid, &edges[..1], 1.0, 1.0).unwrap();

    let faces = solid_faces(&topo, result.solid).unwrap();
    assert!(
        faces.len() > 6,
        "chamfered box should have more than 6 faces"
    );

    let vol = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - BOX_VOLUME).abs() > 0.01,
        "chamfered volume {vol} should differ from original {BOX_VOLUME}"
    );
}

/// Distance-angle chamfer on a single edge.
#[test]
fn chamfer_box_distance_angle() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    let result = chamfer_distance_angle(
        &mut topo,
        solid,
        &edges[..1],
        1.0,
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();

    assert!(
        !result.succeeded.is_empty(),
        "distance-angle chamfer should succeed on at least one edge"
    );
}

#[test]
fn fillet_zero_radius_error() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    assert!(fillet_v2(&mut topo, solid, &edges[..1], 0.0).is_err());
}

#[test]
fn chamfer_zero_distance_error() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    assert!(chamfer_v2(&mut topo, solid, &edges[..1], 0.0, 1.0).is_err());
}

#[test]
fn fillet_empty_edges_error() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    assert!(fillet_v2(&mut topo, solid, &[], 1.0).is_err());
}

fn bottom_circle_edge(topo: &Topology, solid: brepkit_topology::solid::SolidId) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            matches!(edge.curve(), EdgeCurve::Circle(_))
                && topo.vertex(edge.start()).unwrap().point().z().abs() < 1e-9
        })
        .expect("primitive must have a bottom circle edge")
}

fn assert_closed_edge_rejected(
    result: Result<brepkit_blend::BlendResult, brepkit_operations::OperationsError>,
) {
    let error = match result {
        Ok(_) => panic!("closed-edge operation must fail closed"),
        Err(error) => error,
    };
    let message = error.to_string();
    assert!(
        message.contains("closed-edge"),
        "unexpected error: {message}"
    );
    assert!(
        message.contains("invalid solid"),
        "error must explain the safety postcondition: {message}"
    );
}

fn assert_valid_closed_fillet(topo: &Topology, result: &brepkit_blend::BlendResult) {
    assert!(!result.is_partial);
    assert_eq!(result.succeeded.len(), 1);
    assert!(result.failed.is_empty());

    let report = brepkit_check::validate::validate_solid(
        topo,
        result.solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);

    let torus_count = solid_faces(topo, result.solid)
        .unwrap()
        .into_iter()
        .filter(|&face_id| {
            matches!(
                topo.face(face_id).unwrap().surface(),
                brepkit_topology::face::FaceSurface::Torus(_)
            )
        })
        .count();
    assert_eq!(torus_count, 1, "closed-rim fillet must add one torus band");
}

/// Closed circular fillets use the exact analytic rim assembler and must pass
/// the same strict solid validation as planar production fillets.
#[test]
fn fillet_cylinder_closed_rim_is_valid() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    let result = fillet_v2(&mut topo, solid, &[rim], 0.3).unwrap();
    assert_valid_closed_fillet(&topo, &result);
}

#[test]
fn fillet_cone_closed_rim_is_valid() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    let result = fillet_v2(&mut topo, solid, &[rim], 0.3).unwrap();
    assert_valid_closed_fillet(&topo, &result);
}

#[test]
fn chamfer_cylinder_closed_rim_fails_closed() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    assert_closed_edge_rejected(chamfer_v2(&mut topo, solid, &[rim], 0.4, 0.4));
}

#[test]
fn chamfer_cone_closed_rim_fails_closed() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    assert_closed_edge_rejected(chamfer_v2(&mut topo, solid, &[rim], 0.4, 0.4));
}
