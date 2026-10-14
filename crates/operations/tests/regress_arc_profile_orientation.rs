#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]
//! Extrude and thicken of profiles bounded by circular arcs past π.
//!
//! Two defects shared one cause, a chord standing in for an arc that bends
//! past π. `make_planar_face_from_wire` gave a counter-clockwise 270° profile
//! a −z plane, so `thicken(+1)` built the body below it; and `extrude`
//! oriented the cylinder wall from the chord at the arc's start, which points
//! the wrong way past π, so even a correctly oriented profile extruded
//! inside-out (0.952 against 2.856).

use std::f64::consts::PI;

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::{face_area, solid_bounding_box, solid_volume};
use remus_operations::thicken::thicken;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_planar_face_from_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn ccw_270_face(topo: &mut Topology) -> FaceId {
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let a = c.evaluate(0.0);
    let b = c.evaluate(1.5 * PI);
    let va = topo.add_vertex(Vertex::new(a, 1e-7));
    let vb = topo.add_vertex(Vertex::new(b, 1e-7));
    let mut arc = Edge::new(va, vb, EdgeCurve::Circle(c));
    arc.set_trim(Some((0.0, 1.5 * PI)));
    let arc = topo.add_edge(arc);
    let chord = topo.add_edge(Edge::new(vb, va, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
            true,
        )
        .unwrap(),
    );
    make_planar_face_from_wire(topo, wire).unwrap()
}

#[test]
fn thicken_ccw_270_profile_goes_along_winding_normal() {
    let mut topo = Topology::new();
    let face = ccw_270_face(&mut topo);
    let area = face_area(&topo, face, 1e-4).unwrap();
    let true_area = 0.75 * PI + 0.5;

    let solid = thicken(&mut topo, face, 1.0).unwrap();
    let bb = solid_bounding_box(&topo, solid).unwrap();
    let vol = solid_volume(&topo, solid, 1e-5).unwrap();
    let report = validate_solid(&topo, solid).unwrap();
    assert!(
        (area - true_area).abs() < 1e-3,
        "face area {area} vs true {true_area}"
    );
    assert!(
        bb.min.z() > -1e-9 && (bb.max.z() - 1.0).abs() < 1e-9,
        "thicken(+1) of a CCW-about-+z profile must span z in [0, 1], got [{}, {}]",
        bb.min.z(),
        bb.max.z()
    );
    assert!(report.is_valid(), "{:#?}", report.issues);
    assert!(
        (vol - true_area).abs() < 1e-3,
        "volume {vol}, expected {true_area}"
    );
}

fn extrude_report(topo: &mut Topology, face: FaceId, dir: Vec3) -> (f64, bool, f64, f64) {
    let solid = remus_operations::extrude::extrude(topo, face, dir, 1.0).unwrap();
    let bb = solid_bounding_box(topo, solid).unwrap();
    let vol = solid_volume(topo, solid, 1e-5).unwrap();
    let valid = validate_solid(topo, solid).unwrap().is_valid();
    (vol, valid, bb.min.z(), bb.max.z())
}

/// Extrude of a 270° arc + chord profile, from the builder's face and from a
/// hand-oriented +z face, both directions: valid with the exact volume.
#[test]
fn extrude_of_270_degree_arc_profile_is_outward() {
    let true_vol = 0.75 * PI + 0.5;
    let mut results = Vec::new();
    for dir in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0)] {
        let mut topo = Topology::new();
        let face = ccw_270_face(&mut topo);
        let r = extrude_report(&mut topo, face, dir);
        results.push(r);
    }
    for dir in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0)] {
        let mut topo = Topology::new();
        let face = ccw_270_face(&mut topo);
        let wire = topo.face(face).unwrap().outer_wire();
        let good = topo.add_face(remus_topology::face::Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let r = extrude_report(&mut topo, good, dir);
        results.push(r);
    }
    for (vol, valid, _, _) in &results {
        assert!(
            *valid && (vol - true_vol).abs() < 1e-3,
            "extrude of 270-degree arc + chord profile: vol {vol} (true {true_vol}), valid {valid}"
        );
    }
}

fn pacman_face(topo: &mut Topology) -> FaceId {
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let a = c.evaluate(0.0);
    let b = c.evaluate(1.5 * PI);
    let va = topo.add_vertex(Vertex::new(a, 1e-7));
    let vb = topo.add_vertex(Vertex::new(b, 1e-7));
    let vo = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    let mut arc = Edge::new(va, vb, EdgeCurve::Circle(c));
    arc.set_trim(Some((0.0, 1.5 * PI)));
    let arc = topo.add_edge(arc);
    let l1 = topo.add_edge(Edge::new(vb, vo, EdgeCurve::Line));
    let l2 = topo.add_edge(Edge::new(vo, va, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(arc, true),
                OrientedEdge::new(l1, true),
                OrientedEdge::new(l2, true),
            ],
            true,
        )
        .unwrap(),
    );
    make_planar_face_from_wire(topo, wire).unwrap()
}

/// Pac-man (270° arc + two radii, counter-clockwise about +z): `thicken(+1)`
/// builds the body above the profile.
#[test]
fn thicken_ccw_pacman_goes_along_winding_normal() {
    let mut topo = Topology::new();
    let face = pacman_face(&mut topo);
    let solid = thicken(&mut topo, face, 1.0).unwrap();
    let bb = solid_bounding_box(&topo, solid).unwrap();
    let vol = solid_volume(&topo, solid, 1e-5).unwrap();
    let report = validate_solid(&topo, solid).unwrap();
    assert!(
        bb.min.z() > -1e-9 && (bb.max.z() - 1.0).abs() < 1e-9,
        "thicken(+1) of a CCW pac-man must span z in [0, 1], got [{}, {}]",
        bb.min.z(),
        bb.max.z()
    );
    assert!(report.is_valid(), "{:#?}", report.issues);
    let expected = 0.75 * PI;
    assert!(
        (vol - expected).abs() < 1e-3,
        "volume {vol}, expected {expected}"
    );
}

/// A disc whose single closed circle edge is used reversed winds clockwise
/// about +z, so its plane is −z and `thicken(+1)` spans z in [−1, 0].
#[test]
fn thicken_reversed_disc_goes_along_winding_normal() {
    let mut topo = Topology::new();
    let e = remus_topology::builder::make_circle_edge(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        1e-7,
    )
    .unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, false)], true).unwrap());
    let face = make_planar_face_from_wire(&mut topo, wire).unwrap();
    let solid = thicken(&mut topo, face, 1.0).unwrap();
    let bb = solid_bounding_box(&topo, solid).unwrap();
    let vol = solid_volume(&topo, solid, 1e-5).unwrap();
    let report = validate_solid(&topo, solid).unwrap();
    assert!(
        bb.max.z() < 1e-9 && (bb.min.z() + 1.0).abs() < 1e-9,
        "thicken(+1) of a CW-about-+z disc must span z in [-1, 0], got [{}, {}]",
        bb.min.z(),
        bb.max.z()
    );
    assert!(report.is_valid(), "{:#?}", report.issues);
    assert!((vol - PI).abs() < 1e-3, "volume {vol}, expected {PI}");
}
