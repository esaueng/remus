//! Oracles for [`section_clear_of_face`], the certificate that lets the
//! left-out dispatch of [`split_face_2d`] skip a section its face cannot
//! contain. The faces are the ones the cylinder-gear fuse hands it: a seam
//! disc cap, a bored cap whose hole holds the internal teeth's tip lands, and
//! a collar lateral beside the helical tooth band. Each certified case has a
//! crossing or interior counterpart that must stay uncertified.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::surfaces::CylindricalSurface;
use remus_topology::{
    edge::Edge,
    face::Face,
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};
use std::f64::consts::TAU;

const TOL: f64 = 1e-7;
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A whole circle at height `z` as a one-edge closed wire.
fn circle_wire(topo: &mut Topology, r: f64, z: f64) -> remus_topology::wire::WireId {
    let v = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z), TOL));
    let circle = Circle3D::new(Point3::new(0.0, 0.0, z), Z, r).unwrap();
    let t0 = circle.project(Point3::new(r, 0.0, z));
    let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
    edge.set_trim(Some((t0, t0 + TAU)));
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

/// A plane cap at `z = 0` bounded by a circle of radius `r`, holed by
/// concentric circles of the given radii.
fn cap(r: f64, holes: &[f64]) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let outer = circle_wire(&mut topo, r, 0.0);
    let inner = holes
        .iter()
        .map(|&h| circle_wire(&mut topo, h, 0.0))
        .collect();
    let face = topo.add_face(Face::new(
        outer,
        inner,
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    (topo, face)
}

/// A primitive-style lateral of radius `r` between `z0` and `z1`.
fn lateral(r: f64, z0: f64, z1: f64) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let surface = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, r).unwrap();
    let v_bot = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z0), TOL));
    let v_top = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, z1), TOL));
    let bot = Circle3D::new(Point3::new(0.0, 0.0, z0), Z, r).unwrap();
    let top = Circle3D::new(Point3::new(0.0, 0.0, z1), Z, r).unwrap();
    let b0 = bot.project(Point3::new(r, 0.0, z0));
    let t0 = top.project(Point3::new(r, 0.0, z1));
    let mut be = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
    be.set_trim(Some((b0, b0 + TAU)));
    let mut te = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
    te.set_trim(Some((t0, t0 + TAU)));
    let be = topo.add_edge(be);
    let te = topo.add_edge(te);
    let seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(be, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(te, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(surface)));
    (topo, face)
}

/// A fitted cubic section through `pts`.
fn fitted(pts: &[Point3]) -> SectionEdge {
    let fit = remus_math::nurbs::fitting::interpolate(pts, 3).unwrap();
    let dummy = remus_math::curves2d::Curve2D::Line(
        remus_math::curves2d::Line2D::new(
            Point2::new(0.0, 0.0),
            remus_math::vec::Vec2::new(1.0, 0.0),
        )
        .unwrap(),
    );
    SectionEdge {
        curve_3d: EdgeCurve::NurbsCurve(fit),
        trim: None,
        pcurve_a: dummy.clone(),
        pcurve_b: dummy,
        start: pts[0],
        end: *pts.last().unwrap(),
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: None,
    }
}

/// A gently turning radial run in the plane `z`, from `r0` to `r1` about
/// the polar angle `theta`.
fn radial(theta: f64, r0: f64, r1: f64, z: f64) -> SectionEdge {
    let pts: Vec<Point3> = (0..=6)
        .map(|k| {
            let s = f64::from(k) / 6.0;
            let r = r0 + (r1 - r0) * s;
            let a = theta + 0.02 * s * s;
            Point3::new(r * a.cos(), r * a.sin(), z)
        })
        .collect();
    fitted(&pts)
}

/// A helix-like run on the carrier of radius `r`, rising from `z0` to `z1`.
fn rising(r: f64, theta: f64, z0: f64, z1: f64) -> SectionEdge {
    let pts: Vec<Point3> = (0..=6)
        .map(|k| {
            let s = f64::from(k) / 6.0;
            let a = theta + 0.1 * s;
            Point3::new(r * a.cos(), r * a.sin(), z0 + (z1 - z0) * s)
        })
        .collect();
    fitted(&pts)
}

/// The helical boss's seam disc (r 8.723): a flank foot beyond the rim is
/// certified clear on every diagonal; a run crossing the rim or lying inside
/// is not.
#[test]
fn seam_disc_certifies_only_sections_beyond_its_rim() {
    let (topo, face) = cap(8.723_021_019_373_233, &[]);
    for k in 0..8 {
        let theta = f64::from(k) * TAU / 8.0 + 0.1;
        let beyond = radial(theta, 8.82, 9.181, 0.0);
        assert!(section_clear_of_face(&topo, face, &beyond, TOL), "k={k}");
        let crossing = radial(theta, 8.0, 9.181, 0.0);
        assert!(!section_clear_of_face(&topo, face, &crossing, TOL), "k={k}");
        let inside = radial(theta, 2.0, 8.5, 0.0);
        assert!(!section_clear_of_face(&topo, face, &inside, TOL), "k={k}");
    }
}

/// A run ending exactly on the rim meets the disc only there; one reaching
/// ten tolerances past the rim into the disc does not qualify.
#[test]
fn rim_contact_is_clear_but_a_tolerance_scale_dip_is_not() {
    let (topo, face) = cap(5.0, &[]);
    let straight = |r0: f64| {
        let pts: Vec<Point3> = (0..=4)
            .map(|k| Point3::new(r0 + (6.0 - r0) * f64::from(k) / 4.0, 0.0, 0.0))
            .collect();
        fitted(&pts)
    };
    assert!(section_clear_of_face(&topo, face, &straight(5.0), TOL));
    assert!(!section_clear_of_face(
        &topo,
        face,
        &straight(5.0 - 10.0 * TOL),
        TOL
    ));
}

/// The internal ring's bored cap (r 30 outside, r 21.27 hole): the internal
/// teeth's tip lands sit inside the hole and are clear; a run crossing the
/// hole's rim into the material, or lying in it, is not.
#[test]
fn bored_cap_certifies_only_sections_inside_its_hole() {
    let (topo, face) = cap(30.0, &[21.265_862_145_445_155]);
    for k in 0..6 {
        let theta = f64::from(k) * TAU / 6.0 + 0.3;
        let in_hole = radial(theta, 19.0, 21.169, 0.0);
        assert!(section_clear_of_face(&topo, face, &in_hole, TOL), "k={k}");
        let crossing = radial(theta, 19.0, 22.0, 0.0);
        assert!(!section_clear_of_face(&topo, face, &crossing, TOL), "k={k}");
        let material = radial(theta, 23.0, 28.0, 0.0);
        assert!(!section_clear_of_face(&topo, face, &material, TOL), "k={k}");
    }
}

/// A collar lateral (r 12, z 8..10) beside the helical band: tip edges below
/// it, including one ending on its rim, are clear; one rising through the
/// collar is not.
#[test]
fn collar_lateral_certifies_only_sections_outside_its_band() {
    let (topo, face) = lateral(12.0, 8.0, 10.0);
    assert!(section_clear_of_face(
        &topo,
        face,
        &rising(12.0, 0.4, 2.0, 7.8),
        TOL
    ));
    assert!(section_clear_of_face(
        &topo,
        face,
        &rising(12.0, 1.4, 7.0, 8.0),
        TOL
    ));
    assert!(section_clear_of_face(
        &topo,
        face,
        &rising(12.0, 2.4, 10.0, 11.0),
        TOL
    ));
    assert!(!section_clear_of_face(
        &topo,
        face,
        &rising(12.0, 3.4, 7.0, 9.0),
        TOL
    ));
    assert!(!section_clear_of_face(
        &topo,
        face,
        &rising(12.0, 4.4, 8.5, 9.5),
        TOL
    ));
}

/// Only positive-weight NURBS sections carry the hull certificate: a circle
/// beyond the disc stays in the left-out check.
#[test]
fn analytic_sections_are_never_certified() {
    let (topo, face) = cap(5.0, &[]);
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Z, 7.0).unwrap();
    let ring = SectionEdge {
        curve_3d: EdgeCurve::Circle(circle),
        trim: Some((0.0, TAU)),
        ..radial(0.0, 7.0, 7.5, 0.0)
    };
    assert!(!section_clear_of_face(&topo, face, &ring, TOL));
}

/// A closed vertex pair alone cannot establish a whole circular hole.
/// These malformed helper inputs must retain their sections for the gate.
#[test]
fn malformed_hole_domain_does_not_certify_clearance() {
    for span in [None, Some(1e-9), Some(TAU / 2.0), Some(TAU)] {
        let (mut topo, face) = cap(5.0, &[4.0]);
        let hole = topo.face(face).unwrap().inner_wires()[0];
        let edge = topo.wire(hole).unwrap().edges()[0].edge();
        let start = topo.edge(edge).unwrap().strict_domain().unwrap().0;
        topo.edge_mut(edge)
            .unwrap()
            .set_trim(span.map(|d| (start, start + d)));
        let section = fitted(&[
            Point3::new(1.0, 0.1, 0.0),
            Point3::new(1.7, 0.13, 0.0),
            Point3::new(2.4, 0.17, 0.0),
            Point3::new(3.0, 0.2, 0.0),
        ]);
        assert_eq!(
            section_clear_of_face(&topo, face, &section, TOL),
            span == Some(TAU),
            "hole span {span:?}"
        );
    }
}

/// A deliberately non-planar helper section makes centroid projection
/// nearly normal to the face. It must not turn a height tolerance into a
/// certificate for a chord about 99 tolerances inside the material.
#[test]
fn normal_dominated_direction_does_not_certify_clearance() {
    let n = Vec3::new(0.0, 1e-4, 1.0).normalize().unwrap();
    let v = Vec3::new(0.0, n.z(), -n.y());
    let point = |p: Vec3| Point3::new(p.x(), p.y(), p.z());
    let mut topo = Topology::new();
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), n, 5.0).unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: n, d: 0.0 },
    ));
    for side in [1.0, -1.0] {
        let y = side * (5.0 - 1e-5);
        let a = point(v * y + Vec3::new(-0.001, 0.0, 0.0));
        let b = point(v * y + Vec3::new(0.001, 0.0, 0.0));
        assert!(5.0 - (a - Point3::new(0.0, 0.0, 0.0)).length() > 90.0 * TOL);
        let lifted = point(v * (-y) + n * (side * 1024.0));
        let curve = remus_math::nurbs::NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0],
            vec![a, b, lifted, lifted],
            vec![1.0; 4],
        )
        .unwrap();
        let section = SectionEdge {
            start: a,
            end: lifted,
            curve_3d: EdgeCurve::NurbsCurve(curve),
            trim: None,
            ..radial(0.0, 6.0, 7.0, 0.0)
        };
        assert!(
            !section_clear_of_face(&topo, face, &section, TOL),
            "side {side}"
        );
    }
    // A genuine section along x remains eligible on the same tilted plane.
    let start = Point3::new(6.0, 0.0, 0.0);
    let end = Point3::new(7.0, 0.0, 0.0);
    let curve = remus_math::nurbs::NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![start, end],
        vec![1.0; 2],
    )
    .unwrap();
    let section = SectionEdge {
        start,
        end,
        curve_3d: EdgeCurve::NurbsCurve(curve),
        trim: None,
        ..radial(0.0, 6.0, 7.0, 0.0)
    };
    assert!(section_clear_of_face(&topo, face, &section, TOL));
}
