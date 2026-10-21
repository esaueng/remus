//! A planar face bounded by a NURBS edge that no analytic form recognizes
//! integrates that edge exactly, rather than over the 32-chord outline the
//! chord-polygon fallback used.
//!
//! The fallback's sliver reached `solid_volume` once S1 read closed bodies off
//! these face integrals: an extruded closed spline read 4.7e-4 of its volume
//! short, and the closure certificate could not see it, because an extrude's
//! two caps miss the same sliver. The edge is now integrated span by span,
//! each span halved until its Gauss rule converges, and it is read over its
//! own trimmed span.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::properties::face_integrator::integrate_face;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const Z: FaceSurface = FaceSurface::Plane {
    normal: Vec3::new(0.0, 0.0, 1.0),
    d: 0.0,
};

/// A face on `z = 0` bounded by `curve`, trimmed to `trim`, from `a` to `b`,
/// then by straight sides through `corners` back to `a`.
fn face(topo: &mut Topology, curve: NurbsCurve, trim: (f64, f64), corners: &[Point3]) -> Face {
    let points: Vec<Point3> = corners.to_vec();
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();
    let mut edge = Edge::new(vertices[0], vertices[1], EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some(trim));
    let mut edges = vec![OrientedEdge::new(topo.add_edge(edge), true)];
    for i in 1..vertices.len() {
        let line = Edge::new(
            vertices[i],
            vertices[(i + 1) % vertices.len()],
            EdgeCurve::Line,
        );
        edges.push(OrientedEdge::new(topo.add_edge(line), true));
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    Face::new(wire, vec![], Z)
}

/// A square whose bottom side is a rational cubic with weights far from one:
/// a single 16-point rule over its span is off, so the span is halved until
/// it converges. The area is a shoelace over 10⁶ chords of the curve.
#[test]
fn a_rational_free_form_edge_integrates_to_its_area() {
    let side = 2.0;
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(side / 3.0, -0.8, 0.0),
            Point3::new(2.0 * side / 3.0, 0.6, 0.0),
            Point3::new(side, 0.0, 0.0),
        ],
        vec![1.0, 50.0, 0.02, 1.0],
    )
    .unwrap();
    let n = 1_000_000;
    let mut loop_points: Vec<Point3> = (0..=n)
        .map(|i| curve.evaluate(f64::from(i) / f64::from(n)))
        .collect();
    loop_points.push(Point3::new(side, side, 0.0));
    loop_points.push(Point3::new(0.0, side, 0.0));
    let area = loop_points
        .iter()
        .zip(loop_points.iter().cycle().skip(1))
        .map(|(a, b)| a.x().mul_add(b.y(), -(b.x() * a.y())))
        .sum::<f64>()
        / 2.0;

    let mut topo = Topology::new();
    let face = face(
        &mut topo,
        curve,
        (0.0, 1.0),
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(side, 0.0, 0.0),
            Point3::new(side, side, 0.0),
            Point3::new(0.0, side, 0.0),
        ],
    );
    let id = topo.add_face(face);
    let measured = integrate_face(&topo, id, 5).unwrap().area;
    // The 32-chord outline misses this by about 1e-3.
    assert!(
        ((measured - area) / area).abs() < 1e-10,
        "{measured} against {area}"
    );
}

/// An edge that runs the first two spans of a degree-1 carrier whose third
/// span climbs out of the face's plane: the boundary, and so the face's
/// plane, is read over the edge's own trim, never the whole carrier.
#[test]
fn a_trimmed_edge_is_read_over_its_own_span() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 5.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let mut topo = Topology::new();
    let face = face(
        &mut topo,
        curve,
        (0.0, 2.0),
        &[Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
    );
    let id = topo.add_face(face);
    let measured = integrate_face(&topo, id, 5).unwrap().area;
    assert!((measured - 0.5).abs() < 1e-12, "{measured}");
}
