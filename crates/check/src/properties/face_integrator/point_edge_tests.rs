//! A NURBS edge whose control hull is one point to rounding (the
//! zero-radius arc a partial revolve draws at a profile vertex on its axis)
//! encloses no area. Its moments are rounding noise that the adaptive rule's
//! edge-relative allowance could not certify off the origin, so the whole
//! planar face fell to the chord-polygon fan. It is read as its chord.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

use std::f64::consts::FRAC_1_SQRT_2;

use super::*;
use remus_math::nurbs::curve::NurbsCurve;
use remus_topology::edge::Edge;
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// The rational quarter arc of radius `r` about `c` in the `z = c.z` plane.
fn quarter_arc(c: Point3, r: f64) -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(c.x() + r, c.y(), c.z()),
            Point3::new(c.x() + r, c.y() + r, c.z()),
            Point3::new(c.x(), c.y() + r, c.z()),
        ],
        vec![1.0, FRAC_1_SQRT_2, 1.0],
    )
    .unwrap()
}

/// The right triangle `c, c + (2, 0, 0), c + (0, 2, 0)` whose corner at `c`
/// is closed by a zero-radius arc between two vertices at `c`, as a partial
/// revolve builds it (one control point one rounding off the others).
fn triangle_with_point_edge(topo: &mut Topology, c: Point3) -> FaceId {
    let ulp_off = Point3::new(f64::from_bits(c.x().to_bits() + 1), c.y(), c.z());
    let point_arc = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![c, ulp_off, c],
        vec![1.0, FRAC_1_SQRT_2, 1.0],
    )
    .unwrap();
    let a = topo.add_vertex(Vertex::new(c, 1e-9));
    let b = topo.add_vertex(Vertex::new(Point3::new(c.x() + 2.0, c.y(), c.z()), 1e-9));
    let p = topo.add_vertex(Vertex::new(Point3::new(c.x(), c.y() + 2.0, c.z()), 1e-9));
    let a2 = topo.add_vertex(Vertex::new(c, 1e-9));
    let mut point_edge = Edge::new(a2, a, EdgeCurve::NurbsCurve(point_arc));
    point_edge.set_trim(Some((0.0, 1.0)));
    let edges = [
        Edge::new(a, b, EdgeCurve::Line),
        Edge::new(b, p, EdgeCurve::Line),
        Edge::new(p, a2, EdgeCurve::Line),
        point_edge,
    ]
    .into_iter()
    .map(|e| OrientedEdge::new(topo.add_edge(e), true))
    .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: c.z(),
        },
    ))
}

#[test]
fn a_point_edge_off_the_origin_keeps_the_exact_planar_reading() {
    for c in [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(5.0, 3.0, 1.0),
        Point3::new(0.05, 0.03, 0.01),
        Point3::new(500.0, -300.0, 100.0),
    ] {
        let mut topo = Topology::new();
        let face = triangle_with_point_edge(&mut topo, c);
        let contribution = integrate_planar_face_exact(
            &topo,
            face,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
        )
        .unwrap()
        .unwrap_or_else(|| panic!("{c:?}: the face fell to the fan"));
        // Area 2, centroid c + (2/3, 2/3, 0): first moments are area times it.
        let scale = c.x().abs().max(c.y().abs()).max(1.0);
        let checks = [
            ("area", contribution.area, 2.0),
            ("∫x", contribution.centroid_x, 2.0 * (c.x() + 2.0 / 3.0)),
            ("∫y", contribution.centroid_y, 2.0 * (c.y() + 2.0 / 3.0)),
            ("∫z", contribution.centroid_z, 2.0 * c.z()),
        ];
        for (name, got, want) in checks {
            assert!(
                (got - want).abs() <= 1e-13 * scale,
                "{c:?}: {name} reads {got}, closed form {want}"
            );
        }
    }
}

#[test]
fn only_a_hull_within_rounding_of_one_point_is_read_as_a_point() {
    let c = Point3::new(5.0, 3.0, 1.0);
    // 64 roundings of x near 5 and y near 3 are 7.1e-14 and 4.3e-14: an arc
    // of radius 1e-14 is a point, one of radius 1e-12 or more is a curve.
    assert!(nurbs_hull_is_point(&quarter_arc(c, 1e-14)));
    assert!(nurbs_hull_is_point(&quarter_arc(c, 0.0)));
    for r in [1e-12, 1e-6, 1.0] {
        assert!(!nurbs_hull_is_point(&quarter_arc(c, r)), "radius {r}");
    }
    // About the origin the bound is relative to the hull's own coordinates.
    let origin = Point3::new(0.0, 0.0, 0.0);
    assert!(nurbs_hull_is_point(&quarter_arc(origin, 0.0)));
    assert!(!nurbs_hull_is_point(&quarter_arc(origin, 1e-300)));
}

/// The corner `(0,0) → (1,0)`, the unit-weight quadratic Bézier `(1,0),
/// (1,1), (0,1)` and `(0,1) → (0,0)`, in the plane `z = lift`: area
/// `1/2 + 1/3 = 5/6` (the parabolic segment is two thirds of its control
/// triangle).
pub(in crate::properties) fn bezier_corner(topo: &mut Topology, lift: f64) -> FaceId {
    let at = |x: f64, y: f64| Point3::new(x, y, lift);
    let bezier = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![at(1.0, 0.0), at(1.0, 1.0), at(0.0, 1.0)],
        vec![1.0; 3],
    )
    .unwrap();
    let o = topo.add_vertex(Vertex::new(at(0.0, 0.0), 1e-9));
    let x = topo.add_vertex(Vertex::new(at(1.0, 0.0), 1e-9));
    let y = topo.add_vertex(Vertex::new(at(0.0, 1.0), 1e-9));
    let mut curve = Edge::new(x, y, EdgeCurve::NurbsCurve(bezier));
    curve.set_trim(Some((0.0, 1.0)));
    let edges = [
        Edge::new(o, x, EdgeCurve::Line),
        curve,
        Edge::new(y, o, EdgeCurve::Line),
    ]
    .into_iter()
    .map(|e| OrientedEdge::new(topo.add_edge(e), true))
    .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: lift,
        },
    ))
}

/// A large coordinate on one axis never excuses a difference on another: a
/// unit curve in the plane `z = 2^46` is a curve, not its chord (#1874
/// review: it read 1/2 instead of 5/6).
#[test]
fn an_out_of_plane_offset_never_turns_a_curve_into_its_chord() {
    for lift in [0.0, 1e3, 2.0_f64.powi(46), 1e300] {
        let mut topo = Topology::new();
        let face = bezier_corner(&mut topo, lift);
        let area = planar_face_exact_area(&topo, face).unwrap().unwrap();
        assert!(
            (area - 5.0 / 6.0).abs() <= 1e-14,
            "z = {lift:e}: area {area}, closed form 5/6"
        );
    }
}

/// A genuine small curve, its extent far above 64 ulps of its own
/// coordinates on every axis it spans, is never read as a point wherever it
/// sits: offsets on each axis alone and together, in and out of its plane.
#[test]
fn a_small_genuine_curve_at_moderate_offsets_is_never_a_point() {
    for offset in [0.0, 1.0, -37.5, 1e3, 1e6] {
        for shift in [
            Vec3::new(offset, 0.0, 0.0),
            Vec3::new(0.0, offset, 0.0),
            Vec3::new(0.0, 0.0, offset),
            Vec3::new(offset, -offset, offset),
        ] {
            // Extent 1e-6, against 64 ε · 1e6 ≈ 1.4e-8 at the largest offset.
            let c = Point3::new(0.25, -0.5, 0.75) + shift;
            assert!(
                !nurbs_hull_is_point(&quarter_arc(c, 1e-6)),
                "shift {shift:?}"
            );
            let far = Point3::new(c.x(), c.y(), 1e300);
            assert!(
                !nurbs_hull_is_point(&quarter_arc(far, 1e-6)),
                "shift {shift:?}, z = 1e300"
            );
        }
    }
}
