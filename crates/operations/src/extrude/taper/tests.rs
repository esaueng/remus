#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use super::*;
use crate::classify::{PointClassification, classify_point};
use crate::measure::solid_volume;
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::validate::validate_solid;
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

fn apex(height: f64) -> Point3 {
    Point3::new(0.0, 0.0, height)
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

/// `[planes, cylinders, cones, nurbs]` over the solid's faces.
fn census(topo: &Topology, solid: remus_topology::solid::SolidId) -> [usize; 4] {
    let mut counts = [0; 4];
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts[0] += 1,
            FaceSurface::Cylinder(_) => counts[1] += 1,
            FaceSurface::Cone(_) => counts[2] += 1,
            FaceSurface::Nurbs(_) => counts[3] += 1,
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

/// Every section is the profile scaled by `t ∈ [s, 1]` at height `t·h` from
/// the apex, so the volume is `A·h·(1 − s³)/3`.
fn frustum_volume(area: f64, apex_height: f64, scale: f64) -> f64 {
    area * apex_height * scale.mul_add(-scale * scale, 1.0) / 3.0
}

fn assert_volume(topo: &Topology, solid: remus_topology::solid::SolidId, expected: f64) {
    let volume = solid_volume(topo, solid, 1e-5).unwrap();
    assert!(
        ((volume - expected) / expected).abs() < 1e-6,
        "volume {volume} vs exact {expected}"
    );
}

#[test]
fn a_tapered_square_is_a_pyramid_frustum() {
    let mut topo = Topology::new();
    let face = square(&mut topo, 2.0);
    let built = tapered_extrude_impl(&mut topo, face, apex(10.0), Z, 5.0).unwrap();
    assert_eq!(census(&topo, built.solid), [6, 0, 0, 0]);
    assert_valid_and_watertight(&topo, built.solid);
    assert_volume(&topo, built.solid, frustum_volume(16.0, 10.0, 0.5));
    // The far corners are the profile's halved about the apex.
    for &v in &built.construction.outer.top_verts {
        let p = topo.vertex(v).unwrap().point();
        assert!((p.x().abs() - 1.0).abs() < 1e-12, "{p:?}");
        assert!((p.y().abs() - 1.0).abs() < 1e-12, "{p:?}");
        assert!((p.z() - 5.0).abs() < 1e-12, "{p:?}");
    }
    // Every side plane passes through the apex.
    for &side in &built.construction.outer_sides {
        let FaceSurface::Plane { normal, d } = topo.face(side).unwrap().surface() else {
            panic!("a tapered line sweeps a plane");
        };
        assert!((normal.dot(Vec3::new(0.0, 0.0, 10.0)) - d).abs() < 1e-12);
    }
    let classify_at = |p: Point3| classify_point(&topo, built.solid, p, 0.01, 1e-6).unwrap();
    assert_eq!(
        classify_at(Point3::new(1.4, 0.0, 2.5)),
        PointClassification::Inside
    );
    assert_eq!(
        classify_at(Point3::new(1.6, 0.0, 2.5)),
        PointClassification::Outside
    );
}

#[test]
fn a_tapered_disc_is_a_cone_frustum() {
    let mut topo = Topology::new();
    let edge =
        make_circle_edge_with_ref(&mut topo, origin(), Z, 3.0, Vec3::new(1.0, 0.0, 0.0), 1e-7)
            .unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    let built = tapered_extrude_impl(&mut topo, face, apex(9.0), Z, 6.0).unwrap();
    assert_eq!(census(&topo, built.solid), [2, 0, 1, 0]);
    assert_valid_and_watertight(&topo, built.solid);
    assert_volume(&topo, built.solid, frustum_volume(9.0 * PI, 9.0, 1.0 / 3.0));
}

#[test]
fn a_concentric_bore_stays_a_straight_cylinder() {
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
        make_circle_edge_with_ref(&mut topo, origin(), -Z, 0.5, Vec3::new(1.0, 0.0, 0.0), 1e-7)
            .unwrap();
    let inner = topo.add_wire(Wire::new(vec![OrientedEdge::new(bore, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        outer,
        vec![inner],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    let built = tapered_extrude_impl(&mut topo, face, apex(10.0), Z, 4.0).unwrap();
    assert_eq!(census(&topo, built.solid), [6, 1, 0, 0]);
    assert_valid_and_watertight(&topo, built.solid);
    let expected = frustum_volume(16.0, 10.0, 0.6) - PI * 0.25 * 4.0;
    assert_volume(&topo, built.solid, expected);
}

/// A square of half-width `h` with every corner rounded to radius `r`.
fn rounded_square(topo: &mut Topology, h: f64, r: f64) -> FaceId {
    let c = h - r;
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
fn off_axis_corner_arcs_sweep_into_exact_ruled_surfaces() {
    let mut topo = Topology::new();
    let face = rounded_square(&mut topo, 2.0, 0.5);
    let built = tapered_extrude_impl(&mut topo, face, apex(8.0), Z, 4.0).unwrap();
    assert_eq!(census(&topo, built.solid), [6, 0, 0, 4]);
    assert_valid_and_watertight(&topo, built.solid);
    let area = 0.25f64.mul_add(-(4.0 - PI), 16.0);
    assert_volume(&topo, built.solid, frustum_volume(area, 8.0, 0.5));
}

#[test]
fn an_apex_short_of_the_far_cap_is_refused() {
    let mut topo = Topology::new();
    let face = square(&mut topo, 1.0);
    let result = tapered_extrude_impl(&mut topo, face, apex(3.0), Z, 3.0);
    assert!(matches!(result, Err(OperationsError::InvalidInput { .. })));
    let mut topo = Topology::new();
    let face = square(&mut topo, 1.0);
    let result = tapered_extrude_impl(&mut topo, face, apex(-5.0), Z, 1.0);
    assert!(matches!(result, Err(OperationsError::InvalidInput { .. })));
}
