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

/// A retained, trimmed circle NURBS with highly unequal knot spans. Its
/// parameter midpoint lies near 75°, far from the angular midpoint near
/// 178°; cylinder orientation must use the actual tangent at its witness.
#[test]
fn nonuniform_trimmed_nurbs_circle_wall_uses_its_local_tangent() {
    use remus_math::nurbs::curve::NurbsCurve;
    let weights = vec![
        1.,
        std::f64::consts::FRAC_1_SQRT_2,
        1.,
        std::f64::consts::FRAC_1_SQRT_2,
        1.,
        std::f64::consts::FRAC_1_SQRT_2,
        1.,
        std::f64::consts::FRAC_1_SQRT_2,
        1.,
    ];
    for scale in [0.1, 1., 1000.] {
        for reversed in [false, true] {
            for sweep_sign in [-1., 1.] {
                let p = |x: f64, y: f64| Point3::new(x * scale, y * scale, 0.);
                let curve = NurbsCurve::new(
                    2,
                    vec![0., 0., 0., 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 1., 1., 1.],
                    vec![
                        p(1., 0.),
                        p(1., 1.),
                        p(0., 1.),
                        p(-1., 1.),
                        p(-1., 0.),
                        p(-1., -1.),
                        p(0., -1.),
                        p(1., -1.),
                        p(1., 0.),
                    ],
                    weights.clone(),
                )
                .unwrap();
                assert!(matches!(
                    remus_geometry::convert::recognize_curve(&curve, 1e-5),
                    remus_geometry::convert::RecognizedCurve::Circle { .. }
                ));
                let mut topo = Topology::new();
                let a = curve.evaluate(0.);
                let b = curve.evaluate(0.99);
                let span = b.y().atan2(b.x()).rem_euclid(std::f64::consts::TAU);
                let va = topo.add_vertex(Vertex::new(a, 1e-7));
                let vb = topo.add_vertex(Vertex::new(b, 1e-7));
                let mut arc = Edge::new(va, vb, EdgeCurve::NurbsCurve(curve));
                arc.set_trim(Some((0., 0.99)));
                let arc = topo.add_edge(arc);
                let chord = topo.add_edge(Edge::new(vb, va, EdgeCurve::Line));
                let edges = if reversed {
                    vec![
                        OrientedEdge::new(chord, false),
                        OrientedEdge::new(arc, false),
                    ]
                } else {
                    vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)]
                };
                let wire = topo.add_wire(Wire::new(edges, true).unwrap());
                let face = make_planar_face_from_wire(&mut topo, wire).unwrap();
                let solid = remus_operations::extrude::extrude(
                    &mut topo,
                    face,
                    Vec3::new(0., 0., sweep_sign),
                    scale,
                )
                .unwrap();
                let expected_volume = 0.5 * (span - span.sin()) * scale.powi(3);
                let actual = solid_volume(&topo, solid, scale * 1e-5).unwrap();
                assert!(
                    (actual - expected_volume).abs() < expected_volume * 1e-3,
                    "scale {scale}, reversed {reversed}, sweep {sweep_sign}: volume {actual} vs {expected_volume}"
                );
                assert!(validate_solid(&topo, solid).unwrap().is_valid());
                let bbox = solid_bounding_box(&topo, solid).unwrap();
                assert!((bbox.min.z() - (sweep_sign * scale).min(0.)).abs() < scale * 1e-9);
                assert!((bbox.max.z() - (sweep_sign * scale).max(0.)).abs() < scale * 1e-9);
            }
        }
    }
}

#[test]
fn knot_localized_nurbs_profile_thickens_along_its_winding() {
    use remus_math::nurbs::curve::NurbsCurve;
    for scale in [0.001, 1., 1000.] {
        for reversed in [false, true] {
            let p = |x: f64, y: f64| Point3::new(x * scale, y * scale, 0.);
            let a = p(0.1, 0.);
            let b = p(0.1, -0.1);
            let c = p(0.2, -0.02);
            let curve = NurbsCurve::new(
                1,
                vec![0., 0., 1. / 3., 0.4, 0.6, 2. / 3., 1., 1.],
                vec![
                    a,
                    p(100., 100.),
                    p(-100., 100.),
                    p(-100., -100.),
                    p(100., -100.),
                    b,
                ],
                vec![1.; 6],
            )
            .unwrap();
            let mut topo = Topology::new();
            let va = topo.add_vertex(Vertex::new(a, 1e-7));
            let vb = topo.add_vertex(Vertex::new(b, 1e-7));
            let vc = topo.add_vertex(Vertex::new(c, 1e-7));
            let mut edge = Edge::new(va, vb, EdgeCurve::NurbsCurve(curve));
            edge.set_trim(Some((0., 1.)));
            let edge = topo.add_edge(edge);
            let bc = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
            let ca = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
            let edges = if reversed {
                vec![
                    OrientedEdge::new(ca, false),
                    OrientedEdge::new(bc, false),
                    OrientedEdge::new(edge, false),
                ]
            } else {
                vec![
                    OrientedEdge::new(edge, true),
                    OrientedEdge::new(bc, true),
                    OrientedEdge::new(ca, true),
                ]
            };
            let wire = topo.add_wire(Wire::new(edges, true).unwrap());
            let face = make_planar_face_from_wire(&mut topo, wire).unwrap();
            let solid = thicken(&mut topo, face, scale).unwrap();
            let bbox = solid_bounding_box(&topo, solid).unwrap();
            let expected = if reversed { (-scale, 0.) } else { (0., scale) };
            assert!((bbox.min.z() - expected.0).abs() < scale * 1e-9);
            assert!((bbox.max.z() - expected.1).abs() < scale * 1e-9);
        }
    }
}

#[test]
fn ambiguous_profile_winding_refusal_rolls_back_extrude_allocations() {
    let mut topo = Topology::new();
    let edge = remus_topology::builder::make_circle_edge(
        &mut topo,
        Point3::new(0., 0., 0.),
        Vec3::new(0., 0., 1.),
        1.,
        1e-7,
    )
    .unwrap();
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(edge, true),
                OrientedEdge::new(edge, false),
            ],
            true,
        )
        .unwrap(),
    );
    // A caller-supplied planar carrier cannot override ambiguous traversal.
    let face = topo.add_face(remus_topology::face::Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0., 0., 1.),
            d: 0.,
        },
    ));
    let counts = |t: &Topology| {
        (
            t.vertices().len(),
            t.edges().len(),
            t.wires().len(),
            t.faces().len(),
            t.shells().len(),
            t.solids().len(),
        )
    };
    let before = counts(&topo);
    let allocated_before = topo.allocated_slot_count();
    let trim = topo.edge(edge).unwrap().trim();
    let error =
        remus_operations::extrude::extrude(&mut topo, face, Vec3::new(0., 0., 1.), 1.).unwrap_err();
    assert!(error.to_string().contains("winding"), "{error}");
    assert_eq!(counts(&topo), before);
    assert_eq!(topo.edge(edge).unwrap().trim(), trim);
    assert_eq!(topo.face(face).unwrap().outer_wire(), wire);
    assert!(
        topo.allocated_slot_count() > allocated_before,
        "failed allocation handles must remain retired after rollback"
    );
}
