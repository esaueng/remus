#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use super::*;
use crate::measure::solid_volume;
use crate::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use crate::validate::validate_solid;
use remus_topology::builder::make_circle_edge_with_ref;
use remus_topology::edge::Edge;
use remus_topology::explorer::solid_faces;
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);

fn origin() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

fn polar(radius: f64, angle: f64) -> Point3 {
    Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0)
}

/// A square-thread section about the origin: an arc of `outer` over
/// `[0, wide]`, a radial step down to `inner`, the arc of `inner` over the
/// rest of the turn and a radial step back up, around a seam circle.
fn stepped_ring(topo: &mut Topology, outer: f64, inner: f64, wide: f64, seam: f64) -> FaceId {
    let tol = 1e-7;
    let corners = [
        polar(outer, 0.0),
        polar(outer, wide),
        polar(inner, wide),
        polar(inner, 0.0),
    ];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, tol)))
        .collect();
    let arc = |topo: &mut Topology, k: usize, radius: f64, from: f64, to: f64| {
        let radial = Vec3::new(from.cos(), from.sin(), 0.0);
        let circle = Circle3D::new_with_ref(origin(), Z, radius, radial).unwrap();
        let t0 = circle.project(origin() + radial * radius);
        let mut edge = Edge::with_tolerance(
            vertices[k],
            vertices[(k + 1) % 4],
            EdgeCurve::Circle(circle),
            Some(tol),
        );
        edge.set_trim(Some((t0, t0 + (to - from))));
        topo.add_edge(edge)
    };
    let edges = [
        arc(topo, 0, outer, 0.0, wide),
        topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line)),
        arc(topo, 2, inner, wide, 2.0 * PI),
        topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line)),
    ];
    let thread = topo.add_wire(
        Wire::new(
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap(),
    );
    let circle = make_circle_edge_with_ref(topo, origin(), -Z, seam, X, tol).unwrap();
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(circle, true)], true).unwrap());
    topo.add_face(Face::new(
        thread,
        vec![hole],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ))
}

/// The ring's volume in closed form: its section at height `s` is the same
/// stepped ring with both radii moved by `rate · s`, a quadratic in `s`
/// that Simpson's rule integrates exactly.
fn stepped_volume(outer: f64, inner: f64, wide: f64, seam: f64, rate: f64, height: f64) -> f64 {
    let area = |s: f64| {
        let (a, b) = (rate.mul_add(s, outer), rate.mul_add(s, inner));
        0.5 * wide * a * a + 0.5 * 2.0f64.mul_add(PI, -wide) * b * b - PI * seam * seam
    };
    height / 6.0 * (4.0f64.mul_add(area(0.5 * height), area(0.0)) + area(height))
}

#[test]
fn a_stepped_ring_sweeps_cones_and_ruled_flanks() {
    for radial_rate in [0.05, -0.05] {
        let mut topo = Topology::new();
        let (outer, inner, wide, seam) = (5.0, 4.0, 1.2, 3.0);
        let face = stepped_ring(&mut topo, outer, inner, wide, seam);
        // 0.35 of a turn over the height: the outer land's strip stays under
        // a turn (a cone face), the inner one's passes it (swept as a NURBS).
        let (height, rate) = (2.0, 0.35 * PI);
        let mut nets = RadialNets::new();
        let swept = conical_twisted_extrude_impl(
            &mut topo,
            face,
            origin(),
            Z,
            height,
            rate,
            radial_rate,
            1e-7,
            256,
            &mut nets,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        assert!(swept.deviation_bound <= 1e-7);
        let mut cones = 0;
        for face in solid_faces(&topo, swept.solid).unwrap() {
            if let FaceSurface::Cone(cone) = topo.face(face).unwrap().surface() {
                cones += 1;
                // The half-angle from the radial plane: atan(1 / rate).
                assert!((cone.half_angle().tan() * radial_rate.abs() - 1.0).abs() < 1e-12);
            }
        }
        assert_eq!(cones, 1, "the outer land is one cone face");
        let report = validate_solid(&topo, swept.solid).unwrap();
        assert!(report.is_valid(), "{:?}", report.issues);
        let mesh = tessellate_solid_with_tolerance(&topo, swept.solid, 0.01, 0.2).unwrap();
        assert!(is_watertight(&mesh));
        let expected = stepped_volume(outer, inner, wide, seam, radial_rate, height);
        let volume = solid_volume(&topo, swept.solid, 1e-5).unwrap();
        assert!(
            ((volume - expected) / expected).abs() < 1e-6,
            "rate {radial_rate}: volume {volume} vs exact {expected}"
        );
        // The top cap's corners sit on the moved radii, turned with the
        // sweep.
        let top = &swept.construction.outer;
        let first = topo.vertex(top.top_verts()[0]).unwrap().point();
        let grown = radial_rate.mul_add(height, outer);
        let turn = rate * height;
        let expected = Point3::new(grown * turn.cos(), grown * turn.sin(), height);
        assert!((first - expected).length() < 1e-12);
    }
}

#[test]
fn unsupported_profiles_are_refused() {
    let mut topo = Topology::new();
    // A square: its sides are lines that are not radial.
    let square = remus_topology::builder::make_planar_face(
        &mut topo,
        &[
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let mut nets = RadialNets::new();
    let refused = conical_twisted_extrude_impl(
        &mut topo,
        square,
        origin(),
        Z,
        1.0,
        0.5,
        0.05,
        1e-7,
        64,
        &mut nets,
    );
    assert!(matches!(
        refused,
        Err(OperationsError::InvalidInput { reason }) if reason.contains("not radial")
    ));
    // A ring that would shrink through the axis.
    let face = stepped_ring(&mut topo, 5.0, 4.0, 1.2, 3.0);
    let refused = conical_twisted_extrude_impl(
        &mut topo,
        face,
        origin(),
        Z,
        2.0,
        0.5,
        -2.5,
        1e-7,
        64,
        &mut nets,
    );
    assert!(matches!(
        refused,
        Err(OperationsError::InvalidInput { reason }) if reason.contains("shrinks through")
    ));
}
