//! Explicit edge trims through boolean assembly (RFC 0002, Stage 3).
//!
//! The GFA pave filler and builder record exact sub-span trims on split
//! edges inside the boolean's working store. Result assembly, analytic
//! shortcuts, solid copies, transforms, and the arena format must carry those
//! intervals without reconstructing them from endpoints.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::TAU;

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::copy::copy_solid;
use brepkit_operations::primitives::make_cylinder;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::{SolidId, Topology};

/// A cylinder whose rim circle edge gets an explicit (partial) trim
/// stamped on it, standing in for a boolean split arc.
fn cylinder_with_trimmed_rim() -> (Topology, brepkit_topology::SolidId) {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 3.0, 4.0).unwrap();
    let rims: Vec<_> = topo
        .edges()
        .iter()
        .filter(|(_, e)| matches!(e.curve(), EdgeCurve::Circle(_)))
        .map(|(id, _)| id)
        .collect();
    for &rim in &rims {
        topo.edge_mut(rim).unwrap().set_trim(None);
    }
    // Not geometrically meaningful for the closed rim; the point is purely
    // that the stored interval survives every copy path bit-for-bit.
    let mut edge = topo.edge(rims[0]).unwrap().clone();
    edge.set_trim(Some((0.5, 2.5)));
    *topo.edge_mut(rims[0]).unwrap() = edge;
    (topo, solid)
}

fn trimmed_edges(topo: &Topology) -> usize {
    topo.edges()
        .iter()
        .filter(|(_, e)| e.trim().is_some())
        .count()
}

fn solid_trims(topo: &Topology, solid: SolidId) -> Vec<(f64, f64)> {
    let mut trims: Vec<_> = brepkit_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|edge| topo.edge(edge).unwrap().trim())
        .collect();
    trims.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    trims
}

fn stamp_exact_full_circle_trims(topo: &mut Topology, solid: SolidId) {
    for edge_id in brepkit_topology::explorer::solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        let start = topo.vertex(edge.start()).unwrap().point();
        let parameter = circle.project(start);
        topo.edge_mut(edge_id)
            .unwrap()
            .set_trim(Some((parameter, parameter + TAU)));
    }
}

#[test]
fn trims_survive_solid_copy() {
    let (mut topo, solid) = cylinder_with_trimmed_rim();
    assert_eq!(trimmed_edges(&topo), 1);
    let copied = copy_solid(&mut topo, solid).unwrap();
    assert_ne!(copied, solid);
    assert_eq!(
        trimmed_edges(&topo),
        2,
        "the copy must carry the stored trim"
    );
}

#[test]
fn trims_survive_arena_round_trip() {
    let (topo, solid) = cylinder_with_trimmed_rim();
    let bytes = brepkit_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let mut restored = Topology::new();
    let _ = brepkit_io::arena_io::deserialize_solid(&bytes, &mut restored).unwrap();
    let trims: Vec<_> = restored
        .edges()
        .iter()
        .filter_map(|(_, e)| e.trim())
        .collect();
    assert_eq!(trims, vec![(0.5, 2.5)]);
}

#[test]
fn coaxial_cylinder_fast_path_preserves_full_circle_trims() {
    let mut topo = Topology::new();
    let lower = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    let upper = make_cylinder(&mut topo, 2.0, 2.0).unwrap();
    stamp_exact_full_circle_trims(&mut topo, lower);
    stamp_exact_full_circle_trims(&mut topo, upper);
    transform_solid(&mut topo, upper, &Mat4::translation(0.0, 0.0, 1.0)).unwrap();

    let result = boolean(&mut topo, BooleanOp::Fuse, lower, upper).unwrap();
    assert_eq!(
        brepkit_topology::explorer::solid_entity_counts(&topo, result).unwrap(),
        (3, 3, 2),
        "coaxial shortcut must return one analytic cylinder"
    );
    let report = brepkit_operations::validate::validate_solid(&topo, result).unwrap();
    assert!(
        report.is_valid(),
        "result must validate: {:?}",
        report.issues
    );
    let result_trims = solid_trims(&topo, result);
    assert_eq!(result_trims.len(), 2);
    assert!(
        result_trims
            .iter()
            .all(|(start, end)| ((end - start) - TAU).abs() < 1e-12),
        "both rebuilt circular rims must retain exact full-turn domains: {result_trims:?}"
    );

    let bytes = brepkit_io::arena_io::serialize_solid(&topo, result).unwrap();
    let mut restored = Topology::new();
    let restored_solid = brepkit_io::arena_io::deserialize_solid(&bytes, &mut restored).unwrap();
    assert_eq!(
        solid_trims(&restored, restored_solid),
        result_trims,
        "arena round trip must preserve every assembled interval bit-for-bit"
    );
}
