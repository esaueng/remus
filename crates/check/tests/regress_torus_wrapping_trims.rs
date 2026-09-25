//! Face containment on a torus whose boundary loops WRAP the surface, and on a
//! face whose hole is bounded by open arcs.
//!
//! `surface_point_in_face` and the ray-cast `classify_point` share one trim
//! test. It projected each boundary loop to the torus `(u, v)` domain and ran
//! point-in-polygon on it. A loop that runs once around the tube (or once
//! around the axis) comes back shifted by a whole period, so the polygon it
//! makes is degenerate or arbitrary. The face a boolean leaves between two such
//! loops — a tool piercing the tube twice, B45 — rejected every hit, and 12 %
//! of grid points near the fused band classified wrongly.
//!
//! The expected answers here come from the construction alone: the face lies to
//! the LEFT of each of its loops in the torus's own `(u, v)` frame, whichever
//! way its normal flag points.
//!
//! The last test covers the second defect the B39 composite disc hole exposed:
//! a hole's open arcs were outlined by one chord each, so the lens between an
//! arc and its chord counted as material.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_check::classify::surface_point_in_face;
use remus_math::curves::Circle3D;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

const TOL: f64 = 1e-7;
const MAJOR: f64 = 4.0;
const MINOR: f64 = 0.5;

fn torus() -> ToroidalSurface {
    ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), MAJOR, MINOR).unwrap()
}

/// A circle edge spanning `start` to `end` counter-clockwise about the
/// circle's normal (the full turn when `start == end`).
fn circle_edge(topo: &mut Topology, start: VertexId, end: VertexId, circle: Circle3D) -> EdgeId {
    let start_parameter = circle.project(topo.vertex(start).unwrap().point());
    let canonical_end = circle.project(topo.vertex(end).unwrap().point());
    let end_parameter = if start == end {
        start_parameter + TAU
    } else if canonical_end <= start_parameter {
        canonical_end + TAU
    } else {
        canonical_end
    };
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
    edge.set_trim(Some((start_parameter, end_parameter)));
    topo.add_edge(edge)
}

/// The tube circle at angle `u`, parameterized toward `+v` when `up`.
fn meridian(u: f64, up: bool) -> Circle3D {
    let radial = Vec3::new(u.cos(), u.sin(), 0.0);
    // Counter-clockwise about `radial × z` climbs from `+radial` toward `+z`.
    let normal = radial.cross(Vec3::new(0.0, 0.0, 1.0));
    Circle3D::new(
        Point3::new(MAJOR * u.cos(), MAJOR * u.sin(), 0.0),
        if up { normal } else { -normal },
        MINOR,
    )
    .unwrap()
}

/// The ring circle at tube angle `v`, parameterized toward `+u` when `ahead`.
fn latitude(v: f64, ahead: bool) -> Circle3D {
    Circle3D::new(
        Point3::new(0.0, 0.0, MINOR * v.sin()),
        Vec3::new(0.0, 0.0, if ahead { 1.0 } else { -1.0 }),
        MINOR.mul_add(v.cos(), MAJOR),
    )
    .unwrap()
}

/// A wire of one closed circle edge, seamed at `seam`.
fn ring(topo: &mut Topology, circle: Circle3D, seam: Point3) -> WireId {
    let vertex = topo.add_vertex(Vertex::new(seam, TOL));
    let edge = circle_edge(topo, vertex, vertex, circle);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

/// Net turns a wire makes in `(u, v)`: guards that each fixture winds the way
/// the test says it does.
fn turns(topo: &Topology, wire: WireId) -> (f64, f64) {
    let s = torus();
    let uv: Vec<_> = remus_check::util::wire_polygon(topo, wire)
        .unwrap()
        .into_iter()
        .map(|p| s.project_point(p))
        .collect();
    let step = |a: f64, b: f64| (b - a + PI).rem_euclid(TAU) - PI;
    let (mut du, mut dv) = (0.0, 0.0);
    for i in 0..uv.len() {
        let (a, b) = (uv[i], uv[(i + 1) % uv.len()]);
        du += step(a.0, b.0);
        dv += step(a.1, b.1);
    }
    ((du / TAU).round(), (dv / TAU).round())
}

/// Angular distance between two angles, in `[0, π]`.
fn gap(a: f64, b: f64) -> f64 {
    ((a - b + PI).rem_euclid(TAU) - PI).abs()
}

/// `u ∈ (lo, hi)` read going `+u` from `lo`, across the seam if need be.
fn in_arc(x: f64, lo: f64, hi: f64) -> bool {
    (x - lo).rem_euclid(TAU) < (hi - lo).rem_euclid(TAU)
}

/// Every sample on the torus clear of the loops answers as `expected` says,
/// with the face's normal flag either way.
fn assert_containment(
    topo: &mut Topology,
    face: FaceId,
    near_a_loop: impl Fn(f64, f64) -> bool,
    expected: impl Fn(f64, f64) -> bool,
    what: &str,
) {
    let s = torus();
    for reversed in [false, true] {
        topo.face_mut(face).unwrap().set_reversed(reversed);
        let (mut inside, mut wrong) = (0, Vec::new());
        for i in 0..96 {
            for j in 0..48 {
                let u = TAU * (f64::from(i) + 0.31) / 96.0;
                let v = TAU * (f64::from(j) + 0.57) / 48.0;
                if near_a_loop(u, v) {
                    continue;
                }
                let got = surface_point_in_face(topo, face, s.evaluate(u, v)).unwrap();
                inside += usize::from(got);
                if got != expected(u, v) {
                    wrong.push((u, v, got));
                }
            }
        }
        assert!(inside > 0, "{what} (reversed={reversed}): no sample inside");
        assert!(
            wrong.is_empty(),
            "{what} (reversed={reversed}): {} samples misread, e.g. (u, v, got) {:?}",
            wrong.len(),
            &wrong[..wrong.len().min(4)]
        );
    }
}

/// The B45 shape in isolation: a band between two tube-wrapping loops, in both
/// senses. Swapping the loops' senses keeps the same two loops but selects the
/// complementary band, which only the orientation can tell apart.
#[test]
fn a_band_between_tube_wrapping_loops_is_the_side_its_loops_bound() {
    let s = torus();
    let (a, b) = (4.0, 1.0);
    for toward_plus_v in [true, false] {
        let mut topo = Topology::new();
        let outer = ring(&mut topo, meridian(a, toward_plus_v), s.evaluate(a, 0.3));
        let inner = ring(&mut topo, meridian(b, !toward_plus_v), s.evaluate(b, 2.0));
        let sense = if toward_plus_v { 1.0 } else { -1.0 };
        assert_eq!(turns(&topo, outer), (0.0, sense), "outer loop winding");
        assert_eq!(turns(&topo, inner), (0.0, -sense), "inner loop winding");
        let face = topo.add_face(Face::new(outer, vec![inner], FaceSurface::Torus(s.clone())));
        // Left of a loop climbing in `v` is the `-u` side: the band runs from
        // `b` up to `a`. Reversing both loops selects the other band.
        let expected = |u: f64, _v: f64| in_arc(u, b, a) == toward_plus_v;
        assert_containment(
            &mut topo,
            face,
            |u, _| gap(u, a) < 0.05 || gap(u, b) < 0.05,
            expected,
            &format!("meridian band, outer toward +v = {toward_plus_v}"),
        );
    }
}

/// Loops that wrap the axis instead of the tube: the band between two ring
/// circles, which no `+u` ray ever crosses.
#[test]
fn a_band_between_axis_wrapping_loops_is_the_side_its_loops_bound() {
    let s = torus();
    let (lo, hi) = (-1.0, 1.2);
    let mut topo = Topology::new();
    let outer = ring(&mut topo, latitude(lo, true), s.evaluate(0.4, lo));
    let inner = ring(&mut topo, latitude(hi, false), s.evaluate(2.5, hi));
    assert_eq!(turns(&topo, outer), (1.0, 0.0), "outer loop winding");
    assert_eq!(turns(&topo, inner), (-1.0, 0.0), "inner loop winding");
    let face = topo.add_face(Face::new(outer, vec![inner], FaceSurface::Torus(s)));
    // Left of a loop running toward `+u` is the `+v` side.
    assert_containment(
        &mut topo,
        face,
        |_, v| gap(v, lo) < 0.05 || gap(v, hi) < 0.05,
        |_, v| in_arc(v, lo, hi),
        "latitude band",
    );
}

/// A contractible hole inside a wrapping band: the orientation test meets the
/// hole's own boundary first for samples beside it, and must still read the
/// hole as empty.
#[test]
fn a_hole_inside_a_wrapping_band_is_not_material() {
    let s = torus();
    let (a, b) = (4.0, 1.0);
    let (u0, u1, v0, v1) = (2.0, 3.0, -0.5, 0.5);
    let mut topo = Topology::new();
    let outer = ring(&mut topo, meridian(a, true), s.evaluate(a, 0.3));
    let inner = ring(&mut topo, meridian(b, false), s.evaluate(b, 2.0));
    // The hole, clockwise in `(u, v)` so the face stays on its left: up the
    // `u0` side, along the top, down the `u1` side, back along the bottom.
    let corner =
        |topo: &mut Topology, u: f64, v: f64| topo.add_vertex(Vertex::new(s.evaluate(u, v), TOL));
    let (p0, p1, p2, p3) = (
        corner(&mut topo, u0, v0),
        corner(&mut topo, u0, v1),
        corner(&mut topo, u1, v1),
        corner(&mut topo, u1, v0),
    );
    let edges = [
        circle_edge(&mut topo, p0, p1, meridian(u0, true)),
        circle_edge(&mut topo, p1, p2, latitude(v1, true)),
        circle_edge(&mut topo, p2, p3, meridian(u1, false)),
        circle_edge(&mut topo, p3, p0, latitude(v0, false)),
    ];
    let hole = topo.add_wire(
        Wire::new(
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap(),
    );
    assert_eq!(turns(&topo, hole), (0.0, 0.0), "the hole is contractible");
    let face = topo.add_face(Face::new(outer, vec![inner, hole], FaceSurface::Torus(s)));
    let in_hole = |u: f64, v: f64| in_arc(u, u0, u1) && in_arc(v, v0, v1);
    assert_containment(
        &mut topo,
        face,
        |u, v| {
            gap(u, a) < 0.05
                || gap(u, b) < 0.05
                || (in_arc(v, v0 - 0.05, v1 + 0.05) && (gap(u, u0) < 0.05 || gap(u, u1) < 0.05))
                || (in_arc(u, u0 - 0.05, u1 + 0.05) && (gap(v, v0) < 0.05 || gap(v, v1) < 0.05))
        },
        |u, v| in_arc(u, b, a) && !in_hole(u, v),
        "band with a hole",
    );
}

/// A hole bounded by three 120° arcs used to be outlined by the triangle of
/// its three vertices, so the three lenses between each arc and its chord
/// were material. B39's composite disc hole on the torus lost ~0.06 rad of
/// `u` to the same chords.
#[test]
fn a_hole_bounded_by_open_arcs_is_not_material_up_to_the_arcs() {
    let mut topo = Topology::new();
    let (half, rho) = (2.0, 1.0);
    let corners: Vec<_> = [(-half, -half), (half, -half), (half, half), (-half, half)]
        .into_iter()
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), TOL)))
        .collect();
    let sides: Vec<_> = (0..4)
        .map(|i| {
            let edge = topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line));
            OrientedEdge::new(edge, true)
        })
        .collect();
    let outer = topo.add_wire(Wire::new(sides, true).unwrap());

    // Three arcs, walked clockwise (each traversed against its circle).
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), rho).unwrap();
    let knots: Vec<_> = (0..3)
        .map(|k| {
            let t = f64::from(k) * TAU / 3.0;
            topo.add_vertex(Vertex::new(
                Point3::new(rho * t.cos(), rho * t.sin(), 0.0),
                TOL,
            ))
        })
        .collect();
    let arcs: Vec<_> = (0..3)
        .map(|k| circle_edge(&mut topo, knots[k], knots[(k + 1) % 3], circle.clone()))
        .collect();
    let hole = topo.add_wire(
        Wire::new(
            arcs.iter()
                .rev()
                .map(|&e| OrientedEdge::new(e, false))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(
        outer,
        vec![hole],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));

    // Lens samples: past each chord (at 0.5 rho) but inside the arc.
    for k in 0..3 {
        let mid = (f64::from(k) + 0.5) * TAU / 3.0;
        for r in [0.6, 0.75, 0.9] {
            let p = Point3::new(r * rho * mid.cos(), r * rho * mid.sin(), 0.0);
            assert!(
                !surface_point_in_face(&topo, face, p).unwrap(),
                "{p:?} lies in the hole, between arc {k} and its chord"
            );
        }
    }
    for p in [Point3::new(0.0, 0.0, 0.0), Point3::new(0.2, -0.3, 0.0)] {
        assert!(
            !surface_point_in_face(&topo, face, p).unwrap(),
            "{p:?} is in the hole"
        );
    }
    for p in [Point3::new(1.5, 1.5, 0.0), Point3::new(-1.2, 0.0, 0.0)] {
        assert!(
            surface_point_in_face(&topo, face, p).unwrap(),
            "{p:?} is material"
        );
    }
}
