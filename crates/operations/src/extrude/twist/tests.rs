#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use super::*;
use crate::classify::{PointClassification, classify_point};
use crate::measure::solid_volume;
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::validate::validate_solid;
use remus_math::curves::Circle3D;
use remus_topology::builder::{make_circle_edge_with_ref, make_planar_face, make_polygon_wire};
use remus_topology::edge::Edge;
use remus_topology::explorer::solid_faces;
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn origin() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

fn square(topo: &mut Topology, half: f64) -> FaceId {
    make_planar_face(
        topo,
        &[
            Point3::new(-half, -half, 0.0),
            Point3::new(half, -half, 0.0),
            Point3::new(half, half, 0.0),
            Point3::new(-half, half, 0.0),
        ],
        1e-7,
    )
    .unwrap()
}

fn twist(topo: &mut Topology, face: FaceId, height: f64, turn: f64) -> TwistedExtrusion {
    twisted_extrude_impl(topo, face, origin(), Z, height, turn / height, 1e-7, 256)
        .unwrap_or_else(|error| panic!("{error}"))
}

/// `(planes, cylinders, nurbs)` over the solid's faces.
fn census(topo: &Topology, solid: remus_topology::solid::SolidId) -> (usize, usize, usize) {
    let mut counts = (0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts.0 += 1,
            FaceSurface::Cylinder(_) => counts.1 += 1,
            FaceSurface::Nurbs(_) => counts.2 += 1,
            other => panic!("unexpected surface {other:?}"),
        }
    }
    counts
}

fn assert_valid_and_watertight(topo: &Topology, solid: remus_topology::solid::SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.2).unwrap();
    assert!(is_watertight(&mesh));
}

fn assert_volume(topo: &Topology, solid: remus_topology::solid::SolidId, expected: f64) {
    // A twist keeps every cross-section, so Cavalieri gives the exact volume.
    let volume = solid_volume(topo, solid, 1e-5).unwrap();
    assert!(
        ((volume - expected) / expected).abs() < 1e-6,
        "volume {volume} vs exact {expected}"
    );
}

#[test]
fn a_twisted_square_prism_turns_its_top_and_keeps_its_volume() {
    let mut topo = Topology::new();
    let face = square(&mut topo, 2.0);
    let twisted = twist(&mut topo, face, 10.0, FRAC_PI_2);
    assert_eq!(census(&topo, twisted.solid), (2, 0, 4));
    assert!(twisted.deviation_bound <= 1e-7);
    assert_valid_and_watertight(&topo, twisted.solid);
    assert_volume(&topo, twisted.solid, 160.0);

    // A positive rate turns right-handed about +z: the corner at 45° ends
    // at 135°.
    let top: Vec<Point3> = twisted
        .construction
        .outer
        .top_verts
        .iter()
        .map(|&v| topo.vertex(v).unwrap().point())
        .collect();
    assert!(
        top.iter()
            .any(|p| (*p - Point3::new(-2.0, 2.0, 10.0)).length() < 1e-12),
        "{top:?}"
    );

    // Halfway up, the square has turned 45°: its corners sit on the axes.
    let classify_at = |p: Point3| classify_point(&topo, twisted.solid, p, 0.01, 1e-6).unwrap();
    assert_eq!(
        classify_at(Point3::new(2.7, 0.0, 5.0)),
        PointClassification::Inside
    );
    assert_eq!(
        classify_at(Point3::new(1.6, 1.6, 5.0)),
        PointClassification::Outside
    );
}

#[test]
fn a_concentric_bore_stays_an_exact_untwisted_cylinder() {
    let mut topo = Topology::new();
    let outer = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(-2.0, -2.0, 0.0),
            Point3::new(2.0, -2.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(-2.0, 2.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let bore =
        make_circle_edge_with_ref(&mut topo, origin(), -Z, 1.0, Vec3::new(1.0, 0.0, 0.0), 1e-7)
            .unwrap();
    let inner = topo.add_wire(Wire::new(vec![OrientedEdge::new(bore, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        outer,
        vec![inner],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    let twisted = twist(&mut topo, face, 6.0, PI / 3.0);
    assert_eq!(census(&topo, twisted.solid), (2, 1, 4));
    // The bore's seam is still a straight line.
    let hole = &twisted.construction.holes[0].inner;
    assert!(matches!(
        topo.edge(hole.vertical_edge_ids[0]).unwrap().curve(),
        EdgeCurve::Line
    ));
    assert_valid_and_watertight(&topo, twisted.solid);
    assert_volume(&topo, twisted.solid, (16.0 - PI) * 6.0);
}

/// A circular segment: the arc of radius `r` about the axis from `−θ/2` to
/// `θ/2`, closed by its chord.
fn segment(topo: &mut Topology, r: f64, theta: f64) -> FaceId {
    let circle = Circle3D::new_with_ref(origin(), Z, r, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let (a, b) = (-0.5 * theta, 0.5 * theta);
    let start = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
    let end = topo.add_vertex(Vertex::new(circle.evaluate(b), 1e-7));
    let mut arc = Edge::with_tolerance(start, end, EdgeCurve::Circle(circle), Some(1e-7));
    arc.set_trim(Some((a, b)));
    let arc = topo.add_edge(arc);
    let chord = topo.add_edge(Edge::new(end, start, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ))
}

#[test]
fn a_twisted_segment_keeps_its_land_cylinder_and_sweeps_its_chord() {
    let mut topo = Topology::new();
    let theta = 2.0 * PI / 3.0;
    let face = segment(&mut topo, 3.0, theta);
    let twisted = twist(&mut topo, face, 8.0, -0.8);
    assert_eq!(census(&topo, twisted.solid), (2, 1, 1));
    assert_valid_and_watertight(&topo, twisted.solid);
    assert_volume(&topo, twisted.solid, 4.5 * (theta - theta.sin()) * 8.0);
    // The land's helix edges carry the certified bound as their tolerance.
    for &edge in &twisted.construction.outer.vertical_edge_ids {
        let edge = topo.edge(edge).unwrap();
        assert!(matches!(edge.curve(), EdgeCurve::NurbsCurve(_)));
        assert!(edge.tolerance().unwrap() >= twisted.deviation_bound);
    }
}

/// A square of half-width `h` with every corner rounded to radius `r`, wound
/// counter-clockwise about `+z`.
fn rounded_square(topo: &mut Topology, h: f64, r: f64) -> FaceId {
    let c = h - r;
    // Corner centers counter-clockwise from (+, −), with the angle at which
    // each corner's arc starts.
    let corners = [
        (Point3::new(c, -c, 0.0), -FRAC_PI_2),
        (Point3::new(c, c, 0.0), 0.0),
        (Point3::new(-c, c, 0.0), FRAC_PI_2),
        (Point3::new(-c, -c, 0.0), PI),
    ];
    let mut arcs = Vec::new();
    for (center, start) in corners {
        let circle = Circle3D::new_with_ref(center, Z, r, Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let (a, b) = (start, start + FRAC_PI_2);
        let from = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
        let to = topo.add_vertex(Vertex::new(circle.evaluate(b), 1e-7));
        let mut edge = Edge::with_tolerance(from, to, EdgeCurve::Circle(circle), Some(1e-7));
        edge.set_trim(Some((a, b)));
        arcs.push((topo.add_edge(edge), from, to));
    }
    let mut edges = Vec::new();
    for i in 0..4 {
        let (arc, _, to) = arcs[i];
        let (_, next_from, _) = arcs[(i + 1) % 4];
        edges.push(OrientedEdge::new(arc, true));
        let line = topo.add_edge(Edge::new(to, next_from, EdgeCurve::Line));
        edges.push(OrientedEdge::new(line, true));
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ))
}

#[test]
fn off_axis_corner_arcs_sweep_into_certified_surfaces() {
    let mut topo = Topology::new();
    let face = rounded_square(&mut topo, 2.0, 0.5);
    let twisted = twist(&mut topo, face, 5.0, 1.1);
    assert_eq!(census(&topo, twisted.solid), (2, 0, 8));
    assert_valid_and_watertight(&topo, twisted.solid);
    let area = 0.25f64.mul_add(-(4.0 - PI), 16.0);
    assert_volume(&topo, twisted.solid, area * 5.0);
}

#[test]
fn a_closed_off_axis_circle_is_refused() {
    // A disc of radius 1 centered at (3, 0) twisted about the z axis: its
    // closed circle is off the axis, so it is refused rather than mis-swept.
    let mut topo = Topology::new();
    let edge = make_circle_edge_with_ref(
        &mut topo,
        Point3::new(3.0, 0.0, 0.0),
        Z,
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
        1e-7,
    )
    .unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    let result = twisted_extrude_impl(&mut topo, face, origin(), Z, 4.0, 0.2, 1e-7, 256);
    assert!(matches!(result, Err(OperationsError::InvalidInput { .. })));
}

#[test]
fn a_profile_not_perpendicular_to_the_axis_is_refused() {
    let mut topo = Topology::new();
    let face = square(&mut topo, 1.0);
    let result = twisted_extrude_impl(
        &mut topo,
        face,
        origin(),
        Vec3::new(1.0, 0.0, 1.0),
        4.0,
        0.2,
        1e-7,
        256,
    );
    assert!(matches!(result, Err(OperationsError::InvalidInput { .. })));
}
