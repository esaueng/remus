//! A fillet must build its result from its own topology. Duplicating a body in
//! place and filleting both copies used to stitch the second result onto
//! vertices minted for the first: the trimmer reused any vertex in the whole
//! arena within 1e-7 of a contact point, so moving one result dragged the
//! other's corners along with it while `validate_solid` still passed both.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashSet;

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_solid;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::query::filter_filletable_edges;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_vertices};
use remus_topology::solid::SolidId;

const PLATE: (f64, f64, f64) = (160.0, 100.0, 16.0);
const POCKET: (f64, f64, f64) = (18.0, 14.0, 8.0);
const RADIUS: f64 = 6.0;

/// A 160x100x16 plate with a 4x3 grid of 18x14x8 pockets cut from its top.
fn pocketed_plate(topo: &mut Topology) -> SolidId {
    let mut plate = make_box(topo, PLATE.0, PLATE.1, PLATE.2).unwrap();
    for i in 0..4_u32 {
        for j in 0..3_u32 {
            let pocket = make_box(topo, POCKET.0, POCKET.1, POCKET.2).unwrap();
            let x = 20.0 + f64::from(i) * 32.0;
            let y = 15.0 + f64::from(j) * 28.0;
            transform_solid(topo, pocket, &Mat4::translation(x, y, PLATE.2 - POCKET.2)).unwrap();
            plate = boolean(topo, BooleanOp::Cut, plate, pocket).unwrap();
        }
    }
    plate
}

/// The plate's four outer vertical corner edges, as the fillet UI selects
/// them.
fn outer_vertical_corners(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let tol = 1e-6;
    let on_corner = |v: f64, hi: f64| v.abs() < tol || (v - hi).abs() < tol;
    let corners: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&eid| {
            let edge = topo.edge(eid).unwrap();
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return false;
            }
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            (a.x() - b.x()).abs() < tol
                && (a.y() - b.y()).abs() < tol
                && (a.z() - b.z()).abs() > tol
                && on_corner(a.x(), PLATE.0)
                && on_corner(a.y(), PLATE.1)
        })
        .collect();
    assert_eq!(corners.len(), 4, "plate must expose its 4 vertical corners");
    let filletable = filter_filletable_edges(topo, solid, &corners).unwrap();
    assert_eq!(filletable.len(), 4, "all 4 corners must be filletable");
    filletable
}

#[test]
fn fillet_of_in_place_copy_shares_no_topology_with_original() {
    let mut topo = Topology::new();
    let original = pocketed_plate(&mut topo);
    let twin = copy_solid(&mut topo, original).unwrap();

    let twin_edges = outer_vertical_corners(&topo, twin);
    let twin_result = fillet_cascade(&mut topo, twin, &twin_edges, RADIUS).unwrap();
    let original_edges = outer_vertical_corners(&topo, original);
    let original_result = fillet_cascade(&mut topo, original, &original_edges, RADIUS).unwrap();

    let vertex_set = |topo: &Topology, solid| -> HashSet<_> {
        solid_vertices(topo, solid).unwrap().into_iter().collect()
    };
    let twin_vertices = vertex_set(&topo, twin_result.solid);
    let original_vertices = vertex_set(&topo, original_result.solid);
    let mut shared: Vec<_> = twin_vertices.intersection(&original_vertices).collect();
    shared.sort();
    assert!(
        shared.is_empty(),
        "fillet results share {} vertices: {shared:?}",
        shared.len()
    );
    let edge_set = |topo: &Topology, solid| -> HashSet<_> {
        solid_edges(topo, solid).unwrap().into_iter().collect()
    };
    assert!(
        edge_set(&topo, twin_result.solid).is_disjoint(&edge_set(&topo, original_result.solid)),
        "fillet results share edges"
    );

    // Moving the original's result must leave the twin exactly where it was.
    let twin_volume = solid_volume(&topo, twin_result.solid, 0.01).unwrap();
    let plate_volume = PLATE.0 * PLATE.1 * PLATE.2 - 12.0 * POCKET.0 * POCKET.1 * POCKET.2;
    assert!(
        twin_volume < plate_volume,
        "convex corner fillets must remove material: {twin_volume} vs {plate_volume}"
    );
    transform_solid(
        &mut topo,
        original_result.solid,
        &Mat4::translation(0.0, 0.0, 50.0),
    )
    .unwrap();
    let twin_volume_after = solid_volume(&topo, twin_result.solid, 0.01).unwrap();
    assert!(
        (twin_volume_after - twin_volume).abs() <= 1e-9 * twin_volume,
        "moving the original's fillet changed the twin: {twin_volume} -> {twin_volume_after}"
    );
    let report = validate_solid(&topo, twin_result.solid).unwrap();
    assert!(report.is_valid(), "twin must stay valid: {report:?}");
}
