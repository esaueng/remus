#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]
//! The plane of a face built from a wire follows the wire's real winding.
//!
//! `make_planar_face_from_wire` orients the plane normal by Newell's method
//! over boundary samples. Those samples used to take an open conic arc the
//! short way between its endpoints (so an arc past π traced its complement),
//! ignore which way a single closed conic was used, and reduce a NURBS edge
//! to its chord. A counter-clockwise 270° arc closed by its chord came out
//! with a −z plane, and `thicken` then grew the body on the wrong side. The
//! samples now follow each edge's traced parameter span in traversal order.

use std::f64::consts::{FRAC_1_SQRT_2, PI};

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_ellipse_arc, make_planar_face_from_wire};
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::FaceSurface;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn plane_normal(topo: &Topology, wire: Wire) -> Vec3 {
    let mut topo = topo.clone();
    let wid = topo.add_wire(wire);
    let fid = make_planar_face_from_wire(&mut topo, wid).expect("planar face");
    match topo.face(fid).unwrap().surface() {
        FaceSurface::Plane { normal, .. } => *normal,
        other => panic!("expected plane, got {}", other.type_tag()),
    }
}

/// 270-degree CCW circle arc A -> B (A = c(0), B = c(1.5 pi)) closed by the
/// chord B -> A. The wire traverses its region counter-clockwise about +z.
#[test]
fn circle_270_arc_plus_chord_normal_follows_winding() {
    let mut topo = Topology::new();
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let a = c.evaluate(0.0);
    let b = c.evaluate(1.5 * PI);
    let va = topo.add_vertex(Vertex::new(a, 1e-7));
    let vb = topo.add_vertex(Vertex::new(b, 1e-7));
    let mut arc = Edge::new(va, vb, EdgeCurve::Circle(c));
    arc.set_trim(Some((0.0, 1.5 * PI)));
    let arc = topo.add_edge(arc);
    let chord = topo.add_edge(Edge::new(vb, va, EdgeCurve::Line));
    let wire = Wire::new(
        vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
        true,
    )
    .unwrap();
    let n = plane_normal(&topo, wire);
    assert!(
        n.z() > 0.5,
        "CCW 270-degree arc + chord must give +z normal, got {n:?}"
    );
}

/// Control: a 90-degree arc + two lines (short arc) is oriented correctly.
#[test]
fn control_circle_90_arc_normal_follows_winding() {
    let mut topo = Topology::new();
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let a = c.evaluate(0.0);
    let b = c.evaluate(0.5 * PI);
    let va = topo.add_vertex(Vertex::new(a, 1e-7));
    let vb = topo.add_vertex(Vertex::new(b, 1e-7));
    let vo = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    let mut arc = Edge::new(va, vb, EdgeCurve::Circle(c));
    arc.set_trim(Some((0.0, 0.5 * PI)));
    let arc = topo.add_edge(arc);
    let l1 = topo.add_edge(Edge::new(vb, vo, EdgeCurve::Line));
    let l2 = topo.add_edge(Edge::new(vo, va, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(arc, true),
            OrientedEdge::new(l1, true),
            OrientedEdge::new(l2, true),
        ],
        true,
    )
    .unwrap();
    let n = plane_normal(&topo, wire);
    assert!(n.z() > 0.5, "control: got {n:?}");
}

/// Same defect with an ellipse arc built by the public `make_ellipse_arc`.
#[test]
fn ellipse_270_arc_plus_chord_normal_follows_winding() {
    let mut topo = Topology::new();
    let o = Point3::new(0.0, 0.0, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let e = Ellipse3D::new_with_ref(o, z, 2.0, 1.0, x).unwrap();
    let a = e.evaluate(0.0);
    let b = e.evaluate(1.5 * PI);
    let arc = make_ellipse_arc(&mut topo, o, z, 2.0, 1.0, x, a, b, 1e-7).unwrap();
    let (va, vb) = {
        let ed = topo.edge(arc).unwrap();
        (ed.start(), ed.end())
    };
    let chord = topo.add_edge(Edge::new(vb, va, EdgeCurve::Line));
    let wire = Wire::new(
        vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
        true,
    )
    .unwrap();
    let n = plane_normal(&topo, wire);
    assert!(
        n.z() > 0.5,
        "CCW 270-degree ellipse arc + chord must give +z normal, got {n:?}"
    );
}

/// A single closed circle edge used reversed winds clockwise about +z, so
/// the plane normal must be -z.
#[test]
fn reversed_closed_circle_normal_is_minus_z() {
    let mut topo = Topology::new();
    let e = make_circle_edge(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        1e-7,
    )
    .unwrap();
    let fwd = plane_normal(
        &topo,
        Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap(),
    );
    let rev = plane_normal(
        &topo,
        Wire::new(vec![OrientedEdge::new(e, false)], true).unwrap(),
    );
    assert!(fwd.z() > 0.5, "forward closed circle: got {fwd:?}");
    assert!(
        rev.z() < -0.5,
        "reversed closed circle must give -z normal, got {rev:?}"
    );
}

fn nurbs_unit_circle_ccw() -> NurbsCurve {
    let w = FRAC_1_SQRT_2;
    let pts = vec![
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(-1.0, 1.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(-1.0, -1.0, 0.0),
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(1.0, -1.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    ];
    let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    NurbsCurve::new(2, knots, pts, weights).unwrap()
}

fn nurbs_unit_circle_cw() -> NurbsCurve {
    let c = nurbs_unit_circle_ccw();
    let mut pts = c.control_points().to_vec();
    for p in &mut pts {
        *p = Point3::new(p.x(), -p.y(), p.z());
    }
    let w = FRAC_1_SQRT_2;
    let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    NurbsCurve::new(2, knots, pts, weights).unwrap()
}

/// A single closed NURBS circle edge: the CCW and CW versions must get
/// opposite normals (+z and -z). Reduced to its chord, the edge gave Newell
/// nothing to wind, and both versions took the sign of an order-independent
/// fit.
#[test]
fn closed_nurbs_circle_normal_follows_winding() {
    let mut topo = Topology::new();
    let v = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
    let ccw = topo.add_edge(Edge::new(
        v,
        v,
        EdgeCurve::NurbsCurve(nurbs_unit_circle_ccw()),
    ));
    let cw = topo.add_edge(Edge::new(
        v,
        v,
        EdgeCurve::NurbsCurve(nurbs_unit_circle_cw()),
    ));
    let n_ccw = plane_normal(
        &topo,
        Wire::new(vec![OrientedEdge::new(ccw, true)], true).unwrap(),
    );
    let n_cw = plane_normal(
        &topo,
        Wire::new(vec![OrientedEdge::new(cw, true)], true).unwrap(),
    );
    assert!(
        n_ccw.z() > 0.5,
        "CCW closed NURBS circle must give +z, got {n_ccw:?}"
    );
    assert!(
        n_cw.z() < -0.5,
        "CW closed NURBS circle must give -z, got {n_cw:?}"
    );
}

/// An open 270-degree NURBS arc + chord. Reduced to chords, the boundary was
/// the collinear [A, mid(AB), B, mid(BA)], which Newell cannot wind. Both
/// winding directions must give opposite normals.
#[test]
fn nurbs_270_arc_plus_chord_normal_follows_winding() {
    let w = FRAC_1_SQRT_2;
    let mk = |sign: f64| {
        let pts = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, sign, 0.0),
            Point3::new(0.0, sign, 0.0),
            Point3::new(-1.0, sign, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-1.0, -sign, 0.0),
            Point3::new(0.0, -sign, 0.0),
        ];
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0];
        let third = 1.0 / 3.0;
        let knots = vec![
            0.0,
            0.0,
            0.0,
            third,
            third,
            2.0 * third,
            2.0 * third,
            1.0,
            1.0,
            1.0,
        ];
        NurbsCurve::new(2, knots, pts, weights).unwrap()
    };
    let mut results = Vec::new();
    for sign in [1.0, -1.0] {
        let mut topo = Topology::new();
        let va = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(Point3::new(0.0, -sign, 0.0), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::NurbsCurve(mk(sign))));
        let chord = topo.add_edge(Edge::new(vb, va, EdgeCurve::Line));
        let wire = Wire::new(
            vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
            true,
        )
        .unwrap();
        results.push(plane_normal(&topo, wire));
    }
    assert!(
        results[0].z() > 0.5 && results[1].z() < -0.5,
        "CCW wire must give +z and CW wire -z; got ccw {:?}, cw {:?}",
        results[0],
        results[1]
    );
}

/// The same closed CCW NURBS circle edge used reversed winds clockwise, so
/// the normal must flip to -z. The scatter fit only sees control-point
/// positions, which do not change with the oriented-edge direction.
#[test]
fn reversed_closed_nurbs_circle_normal_is_minus_z() {
    let mut topo = Topology::new();
    let v = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
    let ccw = topo.add_edge(Edge::new(
        v,
        v,
        EdgeCurve::NurbsCurve(nurbs_unit_circle_ccw()),
    ));
    let rev = plane_normal(
        &topo,
        Wire::new(vec![OrientedEdge::new(ccw, false)], true).unwrap(),
    );
    assert!(
        rev.z() < -0.5,
        "reversed CCW NURBS circle winds CW and must give -z, got {rev:?}"
    );
}

/// The same 270-degree NURBS arc used reversed (B -> A) closed by the chord
/// A -> B winds clockwise: normal must be -z.
#[test]
fn reversed_nurbs_270_arc_plus_chord_normal_is_minus_z() {
    let w = FRAC_1_SQRT_2;
    let pts = vec![
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(-1.0, 1.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(-1.0, -1.0, 0.0),
        Point3::new(0.0, -1.0, 0.0),
    ];
    let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0];
    let third = 1.0 / 3.0;
    let knots = vec![
        0.0,
        0.0,
        0.0,
        third,
        third,
        2.0 * third,
        2.0 * third,
        1.0,
        1.0,
        1.0,
    ];
    let curve = NurbsCurve::new(2, knots, pts, weights).unwrap();
    let mut topo = Topology::new();
    let va = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
    let vb = topo.add_vertex(Vertex::new(Point3::new(0.0, -1.0, 0.0), 1e-7));
    let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::NurbsCurve(curve)));
    let chord = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
    let wire = Wire::new(
        vec![
            OrientedEdge::new(arc, false),
            OrientedEdge::new(chord, true),
        ],
        true,
    )
    .unwrap();
    let n = plane_normal(&topo, wire);
    assert!(n.z() < -0.5, "CW wire must give -z, got {n:?}");
}
