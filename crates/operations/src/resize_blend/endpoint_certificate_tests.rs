//! Regression: a blend-translation move must carry an edge's exact endpoint
//! certificate forward, as the rigid transform and copy paths already do.
//!
//! Production case: a STEP import stamped a NURBS blend-boundary edge's
//! tolerance at its measured end residual, `1.1450987948266277e-5`: curve
//! end [`IMPORTED_CURVE_END`] against vertex [`IMPORTED_END_VERTEX`]. A
//! 10 mm move of the adjacent −x face took this translation path, which
//! shifted the edge by −10 in x and re-rounded its control points and vertex
//! independently. The recomputed residual, `1.145098794962767e-5`, came out
//! `1.4e-15` over the stamp, and the strict arena I/O gate refused the whole
//! body on STEP export.
//!
//! The fixture rebuilds that edge's end on a side-blended box: the box is
//! placed so one translated band rim ends at the production vertex, the rim
//! becomes the exact rational NURBS of its arc with the production curve end
//! as its last control point, and its tolerance is stamped at exactly the
//! measured residual, which is the importer's contract.

#![allow(clippy::unwrap_used, clippy::panic)]

use super::*;
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_math::nurbs::curve::NurbsCurve;

/// The production edge's end vertex.
const IMPORTED_END_VERTEX: [f64; 3] = [-24.936_374_269, 28.791_381_669, 17.901_728_54];
/// The production edge curve's end: its last control point.
const IMPORTED_CURVE_END: [f64; 3] = [-24.936_378_657, 28.791_383_359, 17.901_738_981];
/// The production stamp, the residual between the two.
const IMPORTED_STAMP: f64 = 1.145_098_794_826_627_7e-5;

const fn point([x, y, z]: [f64; 3]) -> Point3 {
    Point3::new(x, y, z)
}

/// The importer's stamp: the larger endpoint residual over the edge's own
/// parameter authority.
fn endpoint_residual(topo: &Topology, edge: EdgeId) -> f64 {
    let edge = topo.edge(edge).unwrap();
    let (a, b) = edge.strict_domain().unwrap();
    let p = topo.vertex(edge.start()).unwrap().point();
    let q = topo.vertex(edge.end()).unwrap().point();
    let first = (edge.curve().evaluate_with_endpoints(a, p, q) - p).length();
    let second = (edge.curve().evaluate_with_endpoints(b, p, q) - q).length();
    first.max(second)
}

/// A box with its four vertical edges blended, the face facing −x (the
/// production move) and one band rim that face's move translates, rebuilt
/// as the production edge's end.
fn imported_rim_fixture() -> (Topology, SolidId, FaceId, EdgeId) {
    let mut topo = Topology::new();
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 8.0, 6.0).unwrap();
    let vertical: Vec<EdgeId> = remus_topology::explorer::solid_edges(&topo, sharp)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let edge = topo.edge(edge).unwrap();
            let span = topo.vertex(edge.end()).unwrap().point()
                - topo.vertex(edge.start()).unwrap().point();
            span.cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
        })
        .collect();
    let solid = fillet_v2(&mut topo, sharp, &vertical, 1.5).unwrap().solid;
    let face = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(-1.0, 0.0, 0.0)) > 0.99)
        })
        .unwrap();
    let analysis = analyze_translation(&topo, solid, &[face]).unwrap();
    let rim = analysis
        .entities
        .as_ref()
        .unwrap()
        .edges
        .iter()
        .copied()
        .find(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)))
        .unwrap_or_else(|| panic!("the move translates no band rim"));

    // Place the box so the rim ends at the production vertex, then pin that
    // vertex to it exactly; the placement's own rounding is a few ulps.
    let end_vertex = topo.edge(rim).unwrap().end();
    let placement = point(IMPORTED_END_VERTEX) - topo.vertex(end_vertex).unwrap().point();
    crate::transform::transform_solid(
        &mut topo,
        solid,
        &remus_math::mat::Mat4::translation(placement.x(), placement.y(), placement.z()),
    )
    .unwrap();
    topo.vertex_mut(end_vertex)
        .unwrap()
        .set_point(point(IMPORTED_END_VERTEX));

    let edge = topo.edge(rim).unwrap();
    let EdgeCurve::Circle(circle) = edge.curve() else {
        unreachable!()
    };
    let (a, b) = edge.strict_domain().unwrap();
    let exact = remus_geometry::convert::curve_to_nurbs::circle_to_nurbs(circle, a, b).unwrap();
    let mut control_points = exact.control_points().to_vec();
    *control_points.last_mut().unwrap() = point(IMPORTED_CURVE_END);
    let imported = NurbsCurve::new(
        exact.degree(),
        exact.knots().to_vec(),
        control_points,
        exact.weights().to_vec(),
    )
    .unwrap();
    let domain = imported.domain();
    let edge = topo.edge_mut(rim).unwrap();
    edge.set_curve(EdgeCurve::NurbsCurve(imported));
    edge.set_trim(Some(domain));
    let stamp = endpoint_residual(&topo, rim);
    assert_eq!(stamp.to_bits(), IMPORTED_STAMP.to_bits(), "stamp {stamp}");
    topo.edge_mut(rim)
        .unwrap()
        .set_tolerance(Some(stamp))
        .unwrap();
    (topo, solid, face, rim)
}

#[test]
fn blend_translation_carries_a_stamped_endpoint_certificate_through_export() {
    let (mut work, solid, face, rim) = imported_rim_fixture();
    // The stamped state round-trips by construction.
    deserialize_solids(
        &serialize_solids(&work, &[solid]).unwrap(),
        &mut Topology::new(),
    )
    .unwrap();

    let moved = move_translation_invariant_blend_region(&mut work, solid, &[face], 10.0).unwrap();
    let bytes = serialize_solids(&work, &[moved.solid]).unwrap();
    if let Err(error) = deserialize_solids(&bytes, &mut Topology::new()) {
        panic!("the translated body fails the arena gate: {error}");
    }

    // The carry is a coordinate-scale roundoff allowance, never a repair: the
    // rim's image, the one NURBS edge of the moved body, keeps its stamp plus
    // at most the excess and the headroom, each capped at 1e-8 of the stamp.
    let images: Vec<EdgeId> = remus_topology::explorer::solid_edges(&work, moved.solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(work.edge(edge).unwrap().curve(), EdgeCurve::NurbsCurve(_)))
        .collect();
    let [image] = images[..] else {
        panic!("rim {} has images {images:?}", rim.index());
    };
    let carried = work.edge(image).unwrap().tolerance().unwrap();
    assert!(
        carried >= IMPORTED_STAMP && carried - IMPORTED_STAMP <= IMPORTED_STAMP * 2e-8,
        "carried tolerance {carried}"
    );
    assert!(endpoint_residual(&work, image) <= carried);
}
