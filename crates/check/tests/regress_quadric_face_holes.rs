//! A curved face must measure its outer wire MINUS its holes, and a curved
//! face bounded by closed edges must measure something at all.
//!
//! `integrate_face` measures a quadric face by Gauss quadrature over its UV
//! domain. Two things went wrong on that path, and both are visible on a bare
//! cylinder with no boolean in the loop:
//!
//! 1. The domain came from the outer wire alone, and the containment test that
//!    trimmed the abscissae tested that same outer boundary. A face's inner
//!    wires were never consulted, so a hole in a curved face was integrated as
//!    material — a bored wall measured as though it were unbored.
//! 2. `face_uv_bounds` reads the boundary VERTICES. A face whose whole
//!    boundary is one closed edge has exactly one, so the bounds collapsed and
//!    the full analytic domain was substituted — and a cylinder's and a cone's
//!    is unbounded in `v`. The infinite patch made every abscissa non-finite,
//!    the containment test rejected all of them, and the face contributed
//!    exactly zero area and zero volume.
//!
//! Every expected value below is a closed form in the dimension constants. A
//! cylinder of radius `R` has the metric `|∂u × ∂v| = R`, so a region of the
//! `(u, v)` domain of area `A` is a patch of area `R·A` — every area here is
//! that product, and every volume is `(1/3)·R·area`, because `P · n̂ = R` at
//! every point of the wall.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use brepkit_check::properties::face_integrator::integrate_face;
use brepkit_math::curves::Circle3D;
use brepkit_math::surfaces::CylindricalSurface;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve};
use brepkit_topology::face::{Face, FaceSurface};
use brepkit_topology::vertex::Vertex;
use brepkit_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;
/// Wall radius. `P · n̂ = R` on it, so volume is `(1/3)·R·area`.
const R: f64 = 5.0;
/// The trimming outlines are chord polylines, so a curved boundary is the
/// accuracy limit. A boundary made only of constant-`u` and constant-`v` edges
/// — every loop below except the closed rings, which are exact circles the
/// quadrature integrates in closed form — is a polyline already, and is exact.
const EXACT: f64 = 1e-9;

fn cylinder() -> CylindricalSurface {
    CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), R).unwrap()
}

/// A UV-axis-aligned loop on the cylinder: two constant-`v` arcs joined by two
/// constant-`u` lines. `ccw` builds it in the `+u` sense, otherwise in `−u` —
/// a hole is wound opposite the boundary it sits inside.
fn uv_rect_wire(
    topo: &mut Topology,
    s: &CylindricalSurface,
    u: (f64, f64),
    v: (f64, f64),
    ccw: bool,
) -> brepkit_topology::wire::WireId {
    let p = |uu: f64, vv: f64| s.evaluate(uu, vv);
    let (u0, u1) = u;
    let (v0, v1) = v;
    let a = topo.add_vertex(Vertex::new(p(u0, v0), TOL));
    let b = topo.add_vertex(Vertex::new(p(u1, v0), TOL));
    let c = topo.add_vertex(Vertex::new(p(u1, v1), TOL));
    let d = topo.add_vertex(Vertex::new(p(u0, v1), TOL));
    // `EdgeCurve::Circle` spans start to end the CCW way about the circle's
    // own normal, so which arc an edge denotes is the normal's choice, not the
    // endpoints'. `a → b` climbs in `u`, so its ring is about `+z`; `c → d`
    // descends, so its ring must be about `−z` or the edge names the MAJOR arc
    // — the complement of the patch this loop is meant to bound.
    let ring = |vv: f64, up: bool| {
        let axis = Vec3::new(0.0, 0.0, if up { 1.0 } else { -1.0 });
        Circle3D::new(Point3::new(0.0, 0.0, vv), axis, R).unwrap()
    };
    let e0 = topo.add_edge(Edge::new(a, b, EdgeCurve::Circle(ring(v0, true))));
    let e1 = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(c, d, EdgeCurve::Circle(ring(v1, false))));
    let e3 = topo.add_edge(Edge::new(d, a, EdgeCurve::Line));
    let oriented = if ccw {
        vec![
            OrientedEdge::new(e0, true),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, true),
            OrientedEdge::new(e3, true),
        ]
    } else {
        vec![
            OrientedEdge::new(e3, false),
            OrientedEdge::new(e2, false),
            OrientedEdge::new(e1, false),
            OrientedEdge::new(e0, false),
        ]
    };
    topo.add_wire(Wire::new(oriented, true).unwrap())
}

/// A wire of ONE closed edge: the full circle of the wall at height `v`.
/// `forward` picks the sense it is traversed in.
fn ring_wire(topo: &mut Topology, v: f64, forward: bool) -> brepkit_topology::wire::WireId {
    let c = Circle3D::new(Point3::new(0.0, 0.0, v), Vec3::new(0.0, 0.0, 1.0), R).unwrap();
    let seam = topo.add_vertex(Vertex::new(c.evaluate(0.0), TOL));
    let e = topo.add_edge(Edge::new(seam, seam, EdgeCurve::Circle(c)));
    topo.add_wire(Wire::new(vec![OrientedEdge::new(e, forward)], true).unwrap())
}

fn assert_close(actual: f64, expected: f64, scale: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= EXACT * scale.abs(),
        "{what}: expected the closed form {expected:.9}, got {actual:.9} \
         ({:+.3e}, {:+.6} % of {scale:.6})",
        actual - expected,
        100.0 * (actual - expected) / scale
    );
}

/// Defect 1, on a patch the outer boundary really does trim.
///
/// The face is the `u ∈ [0, 2]`, `v ∈ [0, 10]` patch of the wall with the
/// `u ∈ [0.5, 1.5]`, `v ∈ [3, 7]` patch removed. Pre-fix the hole was never
/// consulted and the face measured the whole 100 mm² patch.
#[test]
fn a_hole_in_a_trimmed_curved_patch_is_not_material() {
    let s = cylinder();
    let mut topo = Topology::new();
    let outer = uv_rect_wire(&mut topo, &s, (0.0, 2.0), (0.0, 10.0), true);
    let hole = uv_rect_wire(&mut topo, &s, (0.5, 1.5), (3.0, 7.0), false);
    let face = topo.add_face(Face::new(outer, vec![hole], FaceSurface::Cylinder(s)));

    let c = integrate_face(&topo, face, 8).unwrap();

    let untrimmed = R * 2.0 * 10.0;
    let removed = R * 1.0 * 4.0;
    let area = untrimmed - removed;
    assert_close(c.area, area, untrimmed, "trimmed patch area");
    assert_close(c.volume, R / 3.0 * area, untrimmed, "trimmed patch volume");
    assert!(
        (c.area - untrimmed).abs() > removed * 0.5,
        "the hole must be clearly absent, not merely close: {} vs the \
         hole-filled {untrimmed}",
        c.area
    );
}

/// Defect 1, on a wall the outer boundary does NOT trim.
///
/// A full-revolution band is integrated over its whole analytic `u` period —
/// its boundary wraps the seam and cannot bound a sub-region — so the hole is
/// the only thing that can remove anything, and nothing removed it.
#[test]
fn a_hole_in_a_full_revolution_wall_is_not_material() {
    let s = cylinder();
    let mut topo = Topology::new();

    // The wall as `make_cylinder` builds it: bottom ring, up the seam, top
    // ring, back down the seam.
    let vb = topo.add_vertex(Vertex::new(s.evaluate(0.0, 0.0), TOL));
    let vt = topo.add_vertex(Vertex::new(s.evaluate(0.0, 10.0), TOL));
    let bottom = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), R).unwrap();
    let top = Circle3D::new(Point3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0), R).unwrap();
    let eb = topo.add_edge(Edge::new(vb, vb, EdgeCurve::Circle(bottom)));
    let et = topo.add_edge(Edge::new(vt, vt, EdgeCurve::Circle(top)));
    let seam = topo.add_edge(Edge::new(vb, vt, EdgeCurve::Line));
    let outer = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(eb, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(et, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let hole = uv_rect_wire(&mut topo, &s, (1.0, 2.0), (3.0, 7.0), false);
    let face = topo.add_face(Face::new(outer, vec![hole], FaceSurface::Cylinder(s)));

    let c = integrate_face(&topo, face, 8).unwrap();

    let untrimmed = R * TAU * 10.0;
    let removed = R * 1.0 * 4.0;
    let area = untrimmed - removed;
    assert_close(c.area, area, untrimmed, "bored wall area");
    assert_close(c.volume, R / 3.0 * area, untrimmed, "bored wall volume");
}

/// Defect 2: a wall whose whole boundary is closed edges.
///
/// The outer wire is one closed ring at `v = 0` and the inner wire one closed
/// ring at `v = ±H`; the face is the band between them. There is one boundary
/// VERTEX on the outer wire, so the UV bounds collapsed and the cylinder's
/// unbounded analytic `v` was substituted — every abscissa came out non-finite
/// and the wall measured exactly zero. Both windings are checked: which side
/// of the ring is material is the boundary's own orientation, not a
/// convention, so a wall running down must measure the same as one running up.
#[test]
fn a_wall_bounded_by_closed_edges_is_not_zero() {
    const H: f64 = 10.0;
    for (h, forward) in [(H, true), (-H, false)] {
        let s = cylinder();
        let mut topo = Topology::new();
        let outer = ring_wire(&mut topo, 0.0, forward);
        let far = ring_wire(&mut topo, h, !forward);
        let face = topo.add_face(Face::new(outer, vec![far], FaceSurface::Cylinder(s)));

        let c = integrate_face(&topo, face, 8).unwrap();

        let area = R * TAU * H;
        assert!(
            c.area.is_finite() && c.volume.is_finite(),
            "the wall running to v = {h} measured non-finite: area {} volume {}",
            c.area,
            c.volume
        );
        assert_close(c.area, area, area, "closed-edge wall area");
        assert_close(
            c.volume.abs(),
            R / 3.0 * area,
            area,
            "closed-edge wall volume",
        );
    }
}

/// Trim sampling and quadrature both multiply the work represented by inner
/// wires. Reject a face before sampling once its trim-point budget is exceeded
/// so untrusted topology cannot drive unbounded property-measurement work.
#[test]
fn excessive_curved_trim_complexity_is_rejected() {
    let s = cylinder();
    let mut topo = Topology::new();
    let outer = ring_wire(&mut topo, 0.0, true);
    let holes = (1..=32)
        .map(|v| ring_wire(&mut topo, f64::from(v), false))
        .collect();
    let face = topo.add_face(Face::new(outer, holes, FaceSurface::Cylinder(s)));

    let error = integrate_face(&topo, face, 8).expect_err("trim budget must be enforced");
    assert!(
        error.to_string().contains("trim exceeds the 4096-point"),
        "unexpected error: {error}"
    );
}
