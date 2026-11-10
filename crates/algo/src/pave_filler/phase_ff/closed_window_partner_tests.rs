//! A closed section whose partner face is
//! genuinely (transversally) crossed at its band end just before the
//! excluding face's edge. Ellipse = exact plane z=0 x tilted-cylinder section,
//! a valid production raw carrier (no wobble, no rim riding).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_1_SQRT_2, TAU};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;
use remus_topology::face::Face;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{
    Aabb3, Edge, EdgeCurve, FaceBand, FaceExtents, FaceId, FaceSurface, JunctionRegistry, Point3,
    RawCurve, Tolerance, Topology, Vec3, Vertex, restrict_curves_to_faces,
};

fn box_top(topo: &mut Topology, x_a: f64) -> (FaceId, FaceSurface) {
    let corners = [(0.0, 0.0), (x_a, 0.0), (x_a, 3.0), (0.0, 3.0)]
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)));
    let edges = (0..4)
        .map(|i| {
            let e = topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line));
            OrientedEdge::new(e, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let surface = FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 0.0,
    };
    (
        topo.add_face(Face::new(wire, vec![], surface.clone())),
        surface,
    )
}

const ORIGIN: (f64, f64) = (1.0, 1.5);

fn axis() -> Vec3 {
    Vec3::new(FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2)
}

/// Radius-0.5 cylinder about a 45-degree axis through (1, 1.5, 0); its rim
/// edge is the circle at v = v1.
fn tilted_wall(topo: &mut Topology, v1: f64) -> (FaceId, FaceSurface) {
    let o = Point3::new(ORIGIN.0, ORIGIN.1, 0.0);
    let surface = FaceSurface::Cylinder(CylindricalSurface::new(o, axis(), 0.5).unwrap());
    let circle = Circle3D::new(o + axis() * v1, axis(), 0.5).unwrap();
    let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::with_tolerance(v, v, EdgeCurve::Circle(circle), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let e = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
    (
        topo.add_face(Face::new(wire, vec![], surface.clone())),
        surface,
    )
}

fn section() -> RawCurve {
    let c = Point3::new(ORIGIN.0, ORIGIN.1, 0.0);
    let e = Ellipse3D::new_with_ref(
        c,
        Vec3::new(0.0, 0.0, 1.0),
        0.5 / FRAC_1_SQRT_2,
        0.5,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let p0 = e.evaluate(0.0);
    let curve = EdgeCurve::Ellipse(e);
    RawCurve {
        bbox: Aabb3::from_points([Point3::new(0.29, 1.0, -1e-9), Point3::new(1.71, 2.0, 1e-9)]),
        t_range: (0.0, TAU),
        p_start: p0,
        p_end: p0,
        curve,
    }
}

fn v_of(p: Point3) -> f64 {
    (p - Point3::new(ORIGIN.0, ORIGIN.1, 0.0)).dot(axis())
}

#[test]
fn partner_band_end_crossed_transversally_near_plane_edge() {
    // Exact in-both window: x <= x_b (wall band end) inside x <= x_a (box).
    let x_b = 1.40;
    let v1 = (x_b - ORIGIN.0) * FRAC_1_SQRT_2;
    let mut failures = Vec::new();
    for x_a in [1.42, 1.60] {
        let mut topo = Topology::new();
        let (top, top_surface) = box_top(&mut topo, x_a);
        let (wall, wall_surface) = tilted_wall(&mut topo, v1);
        let raw = section();
        let pieces = restrict_curves_to_faces(
            &topo,
            top,
            wall,
            &top_surface,
            &wall_surface,
            &FaceBand::NONE,
            &FaceBand::bounded(Some((-10.0, v1))),
            vec![raw.clone()],
            Tolerance::default(),
            &mut JunctionRegistry::default(),
            &mut FaceExtents::default(),
        )
        .unwrap();
        assert_eq!(
            pieces.len(),
            2,
            "the in-both window is split at its seam for x_a {x_a}"
        );
        for piece in &pieces {
            for (t, p) in [
                (piece.t_range.0, piece.p_start),
                (piece.t_range.1, piece.p_end),
            ] {
                let on = raw.curve.evaluate_with_endpoints(t, raw.p_start, raw.p_end);
                if v_of(p) - v1 > 1e-7 || v_of(on) - v1 > 1e-7 {
                    failures.push(format!(
                        "x_a {x_a}: end {p:?} lies {:.3e} past the wall's band end v1 (x {} > x_b {x_b})",
                        v_of(p) - v1,
                        p.x()
                    ));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
