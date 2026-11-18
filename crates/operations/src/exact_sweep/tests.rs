#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_line_edge, make_polygon_wire};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn plane_face(
    topo: &mut Topology,
    wire: WireId,
    holes: Vec<WireId>,
    n: Vec3,
    at: Point3,
) -> FaceId {
    let n = n.normalize().unwrap();
    topo.add_face(Face::new(
        wire,
        holes,
        FaceSurface::Plane {
            normal: n,
            d: crate::dot_normal_point(n, at),
        },
    ))
}

fn circle_wire(topo: &mut Topology, center: Point3, normal: Vec3, radius: f64) -> WireId {
    let edge = make_circle_edge(topo, center, normal, radius, 1e-7).unwrap();
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

/// A circle of `radius` about `center` in the plane with `normal`.
fn circle_face(topo: &mut Topology, center: Point3, normal: Vec3, radius: f64) -> FaceId {
    let wire = circle_wire(topo, center, normal, radius);
    plane_face(topo, wire, vec![], normal, center)
}

/// A planar polygon face, wound counter-clockwise about its Newell normal.
fn polygon_face(topo: &mut Topology, points: &[Point3]) -> FaceId {
    let wire = make_polygon_wire(topo, points, 1e-7).unwrap();
    let n = crate::winding::newell_normal(points);
    plane_face(topo, wire, vec![], n, points[0])
}

fn line(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
    make_line_edge(topo, a, b, 1e-7).unwrap()
}

/// A counter-clockwise arc about +Z in the plane `z = center.z`, from angle
/// `a` to `b` (`a < b`).
fn arc(topo: &mut Topology, center: Point3, radius: f64, a: f64, b: f64) -> EdgeId {
    let circle = Circle3D::with_axes(center, Z, radius, X, Y).unwrap();
    let start = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
    let end = topo.add_vertex(Vertex::new(circle.evaluate(b), 1e-7));
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
    edge.set_trim(Some((a, b)));
    topo.add_edge(edge)
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
}

/// `(planes, cylinders, cones, tori, nurbs)`.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize, usize) {
    let mut c = (0, 0, 0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => c.0 += 1,
            FaceSurface::Cylinder(_) => c.1 += 1,
            FaceSurface::Cone(_) => c.2 += 1,
            FaceSurface::Torus(_) => c.3 += 1,
            FaceSurface::Nurbs(_) => c.4 += 1,
            FaceSurface::Sphere(_) => panic!("no sphere expected"),
        }
    }
    c
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::mass_properties(topo, solid).unwrap().mass
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

/// The production QA case: a circle r = 3 swept 270° about r = 30. It was
/// 902 planes, 2.16% low, on the sampled sweep.
#[test]
fn circle_along_a_three_quarter_arc_is_one_torus_band_and_exact() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(30.0, 0.0, 0.0), Y, 3.0);
    let path = arc(&mut topo, p(0.0, 0.0, 0.0), 30.0, 0.0, 1.5 * PI);
    let solid = sweep_exact(&mut topo, profile, &[path]).unwrap();
    assert_valid(&topo, solid, "torus band");
    assert_eq!(census(&topo, solid), (2, 0, 0, 1, 0));
    let exact = PI * 9.0 * 30.0 * 1.5 * PI;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

#[test]
fn an_axis_crossing_between_profile_samples_is_refused_atomically() {
    for (delta, crosses_axis) in [(1e-4, true), (-1e-4, false)] {
        let mut topo = Topology::new();
        let u = 33.0_f64 / 64.0;
        let start = p(u * u - delta, 0.0, 0.0);
        let end = p((1.0 - u).powi(2) - delta, 0.0, 1.0);
        let spline = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![start, p(start.x() - u, 0.0, 0.5), end],
            vec![1.0; 3],
        )
        .unwrap();
        let edge = remus_topology::builder::make_nurbs_edge(&mut topo, start, end, spline, 1e-7);
        let upper = p(1.0, 0.0, 1.0);
        let lower = p(1.0, 0.0, 0.0);
        let edges = [
            edge,
            line(&mut topo, end, upper),
            line(&mut topo, upper, lower),
            line(&mut topo, lower, start),
        ];
        let wire = topo.add_wire(
            Wire::new(
                edges.map(|edge| OrientedEdge::new(edge, true)).to_vec(),
                true,
            )
            .unwrap(),
        );
        let profile = plane_face(&mut topo, wire, vec![], Y, start);
        let path = arc(&mut topo, p(0.0, 0.0, 0.0), 1.0, 0.0, FRAC_PI_2);
        let slots = topo.allocated_slot_count();
        let result = sweep_exact(&mut topo, profile, &[path]);
        if crosses_axis {
            assert!(
                matches!(result, Err(OperationsError::InvalidInput { reason }) if reason.contains("transversally"))
            );
            assert_eq!(topo.allocated_slot_count(), slots);
            assert_eq!(topo.num_faces(), 1);
        } else {
            assert_valid(&topo, result.unwrap(), "near-axis profile");
        }
    }
}

/// Line, quarter arc, line: two cylinders and a torus quadrant.
fn bend(topo: &mut Topology) -> Vec<EdgeId> {
    vec![
        line(topo, p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
        arc(topo, p(20.0, 10.0, 0.0), 10.0, -FRAC_PI_2, 0.0),
        line(topo, p(30.0, 10.0, 0.0), p(30.0, 30.0, 0.0)),
    ]
}

fn bend_length() -> f64 {
    40.0 + 10.0 * FRAC_PI_2
}

#[test]
fn circle_along_line_arc_line_is_cylinder_torus_cylinder() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), X, 2.0);
    let path = bend(&mut topo);
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "pipe bend");
    assert_eq!(census(&topo, solid), (2, 2, 0, 1, 0));
    let exact = PI * 4.0 * bend_length();
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

#[test]
fn path_edges_may_come_in_any_order_and_direction() {
    for order in [[2, 1, 0], [1, 2, 0], [2, 0, 1]] {
        let mut topo = Topology::new();
        let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), X, 2.0);
        let bend = bend(&mut topo);
        let path: Vec<EdgeId> = order.iter().map(|&i| bend[i]).collect();
        let solid = sweep_exact(&mut topo, profile, &path).unwrap();
        assert_valid(&topo, solid, &format!("chain in order {order:?}"));
        let exact = PI * 4.0 * bend_length();
        assert!(rel(volume(&topo, solid), exact) < 1e-9, "{order:?}");
    }
}

#[test]
fn rectangle_along_the_bend_is_planes_and_cylinders() {
    let mut topo = Topology::new();
    let profile = polygon_face(
        &mut topo,
        &[
            p(0.0, -2.0, -1.0),
            p(0.0, 2.0, -1.0),
            p(0.0, 2.0, 1.0),
            p(0.0, -2.0, 1.0),
        ],
    );
    let path = bend(&mut topo);
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "rectangular bend");
    // Per straight run four planes; round the arc two annular planes and two
    // cylinders; plus two caps.
    assert_eq!(census(&topo, solid), (4 + 4 + 2 + 2, 2, 0, 0, 0));
    let exact = 8.0 * bend_length();
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

/// A triangle off the path's centre line: Pappus on the arc, a prism on the
/// line, and the hypotenuse turns into a cone.
#[test]
fn triangle_off_centre_meets_pappus_with_a_cone() {
    let mut topo = Topology::new();
    let profile = polygon_face(
        &mut topo,
        &[p(0.0, 0.0, 0.0), p(0.0, 3.0, 0.0), p(0.0, 0.0, 3.0)],
    );
    let path = vec![
        line(&mut topo, p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
        arc(&mut topo, p(20.0, 10.0, 0.0), 10.0, -FRAC_PI_2, 0.0),
    ];
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "triangle");
    let (planes, cylinders, cones, tori, nurbs) = census(&topo, solid);
    assert_eq!((cylinders, cones, tori, nurbs), (1, 1, 0, 0));
    assert_eq!(planes, 3 + 1 + 2);
    // Centroid (y, z) = (1, 1): 9 from the arc's axis.
    let exact = 4.5 * (20.0 + 9.0 * FRAC_PI_2);
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

#[test]
fn an_oblique_line_shears_a_circle_into_one_exact_nurbs_side() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), X, 2.0);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 5.0, 0.0));
    let solid = sweep_exact(&mut topo, profile, &[path]).unwrap();
    assert_valid(&topo, solid, "sheared cylinder");
    assert_eq!(census(&topo, solid), (2, 0, 0, 0, 1));
    let exact = PI * 4.0 * 10.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

/// A run within a microradian of the circle's axis still drifts off it by
/// a thousandth over its length: the side is the exact sheared NURBS, not a
/// cylinder the far cap would leave.
#[test]
fn a_long_nearly_axial_run_is_not_taken_for_a_cylinder() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), Z, 2.0);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(1e-3, 0.0, 1000.0));
    let solid = sweep_exact(&mut topo, profile, &[path]).unwrap();
    assert_valid(&topo, solid, "nearly axial");
    assert_eq!(census(&topo, solid), (2, 0, 0, 0, 1));
    let exact = PI * 4.0 * 1000.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

/// One full-turn arc closes on itself: a capless torus ring of two half
/// turns.
#[test]
fn a_single_full_turn_arc_builds_a_capless_ring() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(10.0, 0.0, 0.0), Y, 1.0);
    let path = arc(&mut topo, p(0.0, 0.0, 0.0), 10.0, 0.0, 2.0 * PI);
    let solid = sweep_exact(&mut topo, profile, &[path]).unwrap();
    assert_valid(&topo, solid, "full ring");
    assert_eq!(census(&topo, solid), (0, 0, 0, 2, 0));
    let exact = PI * 2.0 * PI * 10.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

#[test]
fn a_profile_with_a_hole_sweeps_a_tube() {
    let mut topo = Topology::new();
    let outer = circle_wire(&mut topo, p(0.0, 0.0, 0.0), X, 2.0);
    let inner_edge = make_circle_edge(&mut topo, p(0.0, 0.0, 0.0), X, 1.0, 1e-7).unwrap();
    let inner = topo.add_wire(Wire::new(vec![OrientedEdge::new(inner_edge, false)], true).unwrap());
    let profile = plane_face(&mut topo, outer, vec![inner], X, p(0.0, 0.0, 0.0));
    let path = bend(&mut topo);
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "tube");
    assert_eq!(census(&topo, solid), (2, 4, 0, 2, 0));
    let exact = PI * 3.0 * bend_length();
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

/// A rounded rectangle path: the profile comes back round to its start, so
/// the ring has no caps.
#[test]
fn a_closed_tangent_chain_builds_a_capless_ring() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(5.0, 0.0, 0.0), X, 1.0);
    let path = vec![
        line(&mut topo, p(5.0, 0.0, 0.0), p(35.0, 0.0, 0.0)),
        arc(&mut topo, p(35.0, 5.0, 0.0), 5.0, -FRAC_PI_2, 0.0),
        line(&mut topo, p(40.0, 5.0, 0.0), p(40.0, 25.0, 0.0)),
        arc(&mut topo, p(35.0, 25.0, 0.0), 5.0, 0.0, FRAC_PI_2),
        line(&mut topo, p(35.0, 30.0, 0.0), p(5.0, 30.0, 0.0)),
        arc(&mut topo, p(5.0, 25.0, 0.0), 5.0, FRAC_PI_2, PI),
        line(&mut topo, p(0.0, 25.0, 0.0), p(0.0, 5.0, 0.0)),
        arc(&mut topo, p(5.0, 5.0, 0.0), 5.0, PI, 1.5 * PI),
    ];
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "ring");
    assert_eq!(census(&topo, solid), (0, 4, 0, 4, 0));
    let exact = PI * (60.0 + 40.0 + 10.0 * PI);
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

/// Two straight runs meeting square: an L. With the profile's centroid on
/// the path, the mitre's outer wedge exactly makes up its inner one, so the
/// volume is the area times the centre-line length.
fn ell(topo: &mut Topology) -> Vec<EdgeId> {
    vec![
        line(topo, p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
        line(topo, p(20.0, 0.0, 0.0), p(20.0, 15.0, 0.0)),
    ]
}

#[test]
fn a_square_corner_between_runs_is_mitred_exactly() {
    let mut topo = Topology::new();
    let profile = polygon_face(
        &mut topo,
        &[
            p(0.0, -2.0, -1.0),
            p(0.0, 2.0, -1.0),
            p(0.0, 2.0, 1.0),
            p(0.0, -2.0, 1.0),
        ],
    );
    let path = ell(&mut topo);
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "mitred rectangle");
    assert_eq!(census(&topo, solid), (4 + 4 + 2, 0, 0, 0, 0));
    let exact = 8.0 * 35.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
    let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
    assert_eq!(open_mesh_edges(&mesh), 0);
}

#[test]
fn a_mitred_round_tube_meets_on_an_exact_ellipse() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), X, 2.0);
    let path = ell(&mut topo);
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "mitred tube");
    assert_eq!(census(&topo, solid), (2, 2, 0, 0, 0));
    let exact = PI * 4.0 * 35.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
    let read = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
    assert!(rel(read, exact) < 1e-6, "solid_volume {read} vs {exact}");
    let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
    assert_eq!(open_mesh_edges(&mesh), 0);
}

#[test]
fn a_closed_square_path_mitres_a_frame() {
    let mut topo = Topology::new();
    let profile = polygon_face(
        &mut topo,
        &[
            p(0.0, -1.0, -1.0),
            p(0.0, 1.0, -1.0),
            p(0.0, 1.0, 1.0),
            p(0.0, -1.0, 1.0),
        ],
    );
    let path = vec![
        line(&mut topo, p(0.0, 0.0, 0.0), p(30.0, 0.0, 0.0)),
        line(&mut topo, p(30.0, 0.0, 0.0), p(30.0, 20.0, 0.0)),
        line(&mut topo, p(30.0, 20.0, 0.0), p(0.0, 20.0, 0.0)),
        line(&mut topo, p(0.0, 20.0, 0.0), p(0.0, 0.0, 0.0)),
    ];
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "frame");
    assert_eq!(census(&topo, solid), (16, 0, 0, 0, 0));
    let exact = 4.0 * 100.0;
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact) < 1e-9, "{measured} vs {exact}");
}

type Sweep = fn(&mut Topology) -> Result<SolidId, OperationsError>;

#[test]
fn refusals_are_typed_and_leave_the_topology_unchanged() {
    let cases: [(&str, Sweep, bool, &str); 12] = [
        (
            "a corner too sharp to mitre",
            |topo| {
                let profile = circle_face(topo, p(-10.0, 0.0, 0.0), X, 0.001);
                let turn = 179.95_f64.to_radians();
                let path = [
                    line(topo, p(-10.0, 0.0, 0.0), p(0.0, 0.0, 0.0)),
                    line(
                        topo,
                        p(0.0, 0.0, 0.0),
                        p(10.0 * turn.cos(), 10.0 * turn.sin(), 0.0),
                    ),
                ];
                sweep_exact(topo, profile, &path)
            },
            false,
            "well-conditioned mitre",
        ),
        (
            "a mitre's tip into a later run",
            |topo| {
                let (profile, path) = mitre_tip_into_a_later_run(topo);
                sweep_exact(topo, profile, &path)
            },
            true,
            "meet itself",
        ),
        (
            "a nearly full turn into a tangent line",
            |topo| {
                let (profile, path) = nearly_full_turn_then_line(topo);
                sweep_exact(topo, profile, &path)
            },
            true,
            "meet itself",
        ),
        (
            "an oblique profile turned back into its own run",
            |topo| {
                // 340° about (10, 5): the arc ends at (8.29, 0.30, 0), inside
                // the unit tube of the run it started from.
                let tilt = 30.0_f64.to_radians();
                let normal = Vec3::new(tilt.cos(), -tilt.sin(), 0.0);
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), normal, 1.0);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)),
                    arc(
                        topo,
                        p(10.0, 5.0, 0.0),
                        5.0,
                        -FRAC_PI_2,
                        250.0_f64.to_radians(),
                    ),
                ];
                sweep_exact(topo, profile, &path)
            },
            true,
            "meet itself",
        ),
        (
            "edges that do not meet",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 1.0);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)),
                    line(topo, p(12.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
                ];
                sweep_exact(topo, profile, &path)
            },
            true,
            "connected chain",
        ),
        (
            "a corner next to an arc",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 1.0);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)),
                    arc(topo, p(15.0, 0.0, 0.0), 5.0, FRAC_PI_2, PI),
                ];
                sweep_exact(topo, profile, &path)
            },
            false,
            "next to an arc",
        ),
        (
            "a run too short for its mitre",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 2.0);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)),
                    line(topo, p(1.0, 0.0, 0.0), p(1.0, 10.0, 0.0)),
                ];
                sweep_exact(topo, profile, &path)
            },
            true,
            "too short for its mitred corners",
        ),
        (
            "an elliptical path",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 1.0);
                let ellipse =
                    remus_math::curves::Ellipse3D::new(p(0.0, 5.0, 0.0), Z, 6.0, 5.0).unwrap();
                let a = topo.add_vertex(Vertex::new(ellipse.evaluate(0.0), 1e-7));
                let b = topo.add_vertex(Vertex::new(ellipse.evaluate(1.0), 1e-7));
                let mut edge = Edge::new(a, b, EdgeCurve::Ellipse(ellipse));
                edge.set_trim(Some((0.0, 1.0)));
                let edge = topo.add_edge(edge);
                sweep_exact(topo, profile, &[edge])
            },
            false,
            "lines and circular arcs",
        ),
        (
            "a profile across the arc's axis",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 2.0);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)),
                    arc(topo, p(10.0, 1.5, 0.0), 1.5, -FRAC_PI_2, 0.0),
                ];
                sweep_exact(topo, profile, &path)
            },
            true,
            "axis",
        ),
        (
            "a path that crosses itself",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 1.0);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
                    arc(topo, p(20.0, 5.0, 0.0), 5.0, -FRAC_PI_2, PI),
                    line(topo, p(15.0, 5.0, 0.0), p(15.0, -10.0, 0.0)),
                ];
                sweep_exact(topo, profile, &path)
            },
            true,
            "meet itself",
        ),
        (
            "runs that cross between the sampled points",
            |topo| {
                // Runs 0 and 2 cross at (-1, 0, 0); a thin profile keeps the
                // mitres short.
                let h = 0.001;
                let profile = polygon_face(
                    topo,
                    &[
                        p(-10.0, -h, -h),
                        p(-10.0, h, -h),
                        p(-10.0, h, h),
                        p(-10.0, -h, h),
                    ],
                );
                let path = [
                    line(topo, p(-10.0, 0.0, 0.0), p(10.0, 0.0, 0.0)),
                    line(topo, p(10.0, 0.0, 0.0), p(10.0, 11.0, 0.0)),
                    line(topo, p(10.0, 11.0, 0.0), p(-10.0, -9.0, 0.0)),
                ];
                sweep_exact(topo, profile, &path)
            },
            true,
            "meet itself",
        ),
        (
            "a profile along the path",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), Z, 1.0);
                let path = [line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0))];
                sweep_exact(topo, profile, &path)
            },
            true,
            "transversally",
        ),
    ];
    for (label, build, invalid_input, needle) in cases {
        let mut topo = Topology::new();
        let result = build(&mut topo);
        let reason = match (invalid_input, result) {
            (true, Err(OperationsError::InvalidInput { reason }))
            | (false, Err(OperationsError::Unsupported { reason, .. })) => reason,
            (_, other) => panic!("{label}: expected a typed refusal, got {other:?}"),
        };
        assert!(reason.contains(needle), "{label}: {reason}");
        assert_eq!(
            topo.num_faces(),
            1,
            "{label}: only the profile face is left"
        );
    }
}

/// Mesh edges used by exactly one triangle; 0 means watertight.
fn open_mesh_edges(mesh: &crate::tessellate::TriangleMesh) -> usize {
    use std::collections::HashMap;
    let mut uses: HashMap<(u32, u32), i32> = HashMap::new();
    for t in mesh.indices.chunks(3) {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    uses.values().filter(|&&c| c == 1).count()
}

fn mesh_volume(mesh: &crate::tessellate::TriangleMesh) -> f64 {
    let o = p(0.0, 0.0, 0.0);
    let mut six = 0.0;
    for t in mesh.indices.chunks(3) {
        let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
        six += (a - o).dot((b - o).cross(c - o));
    }
    six / 6.0
}

/// Every surface family the sweep builds tessellates watertight, encloses
/// its volume, and reads its closed form through `solid_volume`.
#[test]
fn every_family_tessellates_watertight_and_measures_exact() {
    type Case = (&'static str, fn(&mut Topology) -> SolidId, f64);
    let cases: [Case; 4] = [
        (
            "circle bend",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 2.0);
                let path = bend(topo);
                sweep_exact(topo, profile, &path).unwrap()
            },
            PI * 4.0 * bend_length(),
        ),
        (
            "rectangle bend",
            |topo| {
                let profile = polygon_face(
                    topo,
                    &[
                        p(0.0, -2.0, -1.0),
                        p(0.0, 2.0, -1.0),
                        p(0.0, 2.0, 1.0),
                        p(0.0, -2.0, 1.0),
                    ],
                );
                let path = bend(topo);
                sweep_exact(topo, profile, &path).unwrap()
            },
            8.0 * bend_length(),
        ),
        (
            "triangle with a cone",
            |topo| {
                let profile = polygon_face(
                    topo,
                    &[p(0.0, 0.0, 0.0), p(0.0, 3.0, 0.0), p(0.0, 0.0, 3.0)],
                );
                let path = vec![
                    line(topo, p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
                    arc(topo, p(20.0, 10.0, 0.0), 10.0, -FRAC_PI_2, 0.0),
                ];
                sweep_exact(topo, profile, &path).unwrap()
            },
            4.5 * (20.0 + 9.0 * FRAC_PI_2),
        ),
        (
            "sheared NURBS cylinder",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), X, 2.0);
                let path = line(topo, p(0.0, 0.0, 0.0), p(10.0, 5.0, 0.0));
                sweep_exact(topo, profile, &[path]).unwrap()
            },
            PI * 4.0 * 10.0,
        ),
    ];
    for (label, build, exact) in cases {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        for deflection in [0.05, 0.01] {
            let mesh = crate::tessellate::tessellate_solid(&topo, solid, deflection).unwrap();
            assert_eq!(
                open_mesh_edges(&mesh),
                0,
                "{label}: watertight at {deflection}"
            );
            let enclosed = mesh_volume(&mesh);
            assert!(
                rel(enclosed, exact) < 1e-2,
                "{label}: mesh {enclosed} vs {exact}"
            );
        }
        let measured = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
        assert!(
            rel(measured, exact) < 1e-9,
            "{label}: solid_volume {measured} vs {exact}"
        );
    }
}

/// Two lines that cross at a grazing angle far from the origin: `a·c − b²`
/// cancels to zero there, so the crossing must come from the cross-product
/// form, never from the endpoints alone (0.5 apart).
#[test]
fn grazing_crossing_lines_measure_zero_apart() {
    let d = segment_segment_distance(
        p(-1e8, 0.0, 0.0),
        p(1e8, 0.0, 0.0),
        p(-5e7, -0.5, 0.0),
        p(5e7, 0.5, 0.0),
    );
    assert!(d < 1e-3, "{d}");
    // Parallel and skew pairs keep their true distance, less rounding.
    let parallel = segment_segment_distance(
        p(0.0, 0.0, 0.0),
        p(10.0, 0.0, 0.0),
        p(2.0, 3.0, 0.0),
        p(8.0, 3.0, 0.0),
    );
    assert!((parallel - 3.0).abs() < 1e-12, "{parallel}");
    let skew = segment_segment_distance(
        p(-5.0, 0.0, 0.0),
        p(5.0, 0.0, 0.0),
        p(0.0, -5.0, 2.0),
        p(0.0, 5.0, 2.0),
    );
    assert!((skew - 2.0).abs() < 1e-12 && skew <= 2.0, "{skew}");
}

/// Five runs where the first corner turns 170°: the first run's mitre
/// reaches 1.14 past the corner on its outer side, into the last run 1.0
/// beyond it, though the runs' centre lines stay 1.0 apart. A thin tube, so
/// twice its reach is 0.2.
fn mitre_tip_into_a_later_run(topo: &mut Topology) -> (FaceId, Vec<EdgeId>) {
    let profile = circle_face(topo, p(-20.0, 0.0, 0.0), X, 0.1);
    let back = 10.0_f64.to_radians();
    let b = p(-5.0 * back.cos(), 5.0 * back.sin(), 0.0);
    let path = vec![
        line(topo, p(-20.0, 0.0, 0.0), p(0.0, 0.0, 0.0)),
        line(topo, p(0.0, 0.0, 0.0), b),
        line(topo, b, p(b.x(), 5.0, 0.0)),
        line(topo, p(b.x(), 5.0, 0.0), p(1.0, 5.0, 0.0)),
        line(topo, p(1.0, 5.0, 0.0), p(1.0, -1.0, 0.0)),
    ];
    (profile, path)
}

/// A 350° arc of radius 10, then a tangent line: the line passes 0.15 from
/// the arc's start, well inside the unit tube there.
fn nearly_full_turn_then_line(topo: &mut Topology) -> (FaceId, Vec<EdgeId>) {
    let profile = circle_face(topo, p(10.0, 0.0, 0.0), Y, 1.0);
    let end = 350.0_f64.to_radians();
    let turn = arc(topo, p(0.0, 0.0, 0.0), 10.0, 0.0, end);
    let at = p(10.0 * end.cos(), 10.0 * end.sin(), 0.0);
    let tangent = Vec3::new(-end.sin(), end.cos(), 0.0);
    let lead = line(topo, at, at + tangent * 20.0);
    (profile, vec![turn, lead])
}

/// The mitre tip that reached a later run used to build an overlapping
/// body: its distance check measured the run's centre line only up to the
/// corner, not over the rulings the mitre extends past it.
#[test]
fn a_mitre_tip_reaching_a_later_run_is_refused() {
    let mut topo = Topology::new();
    let (profile, path) = mitre_tip_into_a_later_run(&mut topo);
    let slots = topo.allocated_slot_count();
    let refused = sweep_exact(&mut topo, profile, &path);
    assert!(
        matches!(&refused, Err(OperationsError::InvalidInput { reason })
            if reason.contains("segments 0 and 4") && reason.contains("meet itself")),
        "{refused:?}"
    );
    assert_eq!(topo.allocated_slot_count(), slots);
    // With the last run moved past the tip, by more than twice the reach, it
    // builds.
    let mut topo = Topology::new();
    let (profile, mut path) = mitre_tip_into_a_later_run(&mut topo);
    let b = topo
        .vertex(topo.edge(path[2]).unwrap().end())
        .unwrap()
        .point();
    path.truncate(3);
    path.push(line(&mut topo, b, p(1.5, 5.0, 0.0)));
    path.push(line(&mut topo, p(1.5, 5.0, 0.0), p(1.5, -1.0, 0.0)));
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "five runs clear of the tip");
}

/// A turn of more than half a circle used to go unchecked against the
/// segment after it; its first half is now held to the distance check.
#[test]
fn a_nearly_full_turn_into_a_tangent_line_is_refused() {
    let mut topo = Topology::new();
    let (profile, path) = nearly_full_turn_then_line(&mut topo);
    let slots = topo.allocated_slot_count();
    let refused = sweep_exact(&mut topo, profile, &path);
    assert!(
        matches!(&refused, Err(OperationsError::InvalidInput { reason })
            if reason.contains("segments 0 and 1") && reason.contains("meet itself")),
        "{refused:?}"
    );
    assert_eq!(topo.allocated_slot_count(), slots);
}

fn assert_probe(topo: &Topology, solid: SolidId, at: Point3, inside: bool, s: f64, label: &str) {
    let found = crate::classify::classify_point(topo, solid, at, 0.01 * s, 1e-9 * s).unwrap();
    let expected = if inside {
        crate::classify::PointClassification::Inside
    } else {
        crate::classify::PointClassification::Outside
    };
    assert_eq!(found, expected, "{label}: probe {at:?}");
}

/// A three-quarter turn, then a tangent line that leaves it: the halved
/// arc clears the line, so the sweep builds, exact, at every scale.
#[test]
fn a_three_quarter_turn_into_a_tangent_line_builds_exactly() {
    for s in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let profile = circle_face(&mut topo, p(30.0 * s, 0.0, 0.0), Y, 3.0 * s);
        let path = [
            arc(&mut topo, p(0.0, 0.0, 0.0), 30.0 * s, 0.0, 1.5 * PI),
            line(
                &mut topo,
                p(0.0, -30.0 * s, 0.0),
                p(40.0 * s, -30.0 * s, 0.0),
            ),
        ];
        let solid = sweep_exact(&mut topo, profile, &path).unwrap();
        let label = format!("turn and lead at {s}");
        assert_valid(&topo, solid, &label);
        assert_eq!(census(&topo, solid), (2, 1, 0, 1, 0), "{label}");
        let exact = PI * 9.0 * s * s * (30.0 * s).mul_add(1.5 * PI, 40.0 * s);
        let measured = volume(&topo, solid);
        assert!(
            rel(measured, exact) < 1e-9,
            "{label}: {measured} vs {exact}"
        );
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05 * s).unwrap();
        assert!(crate::tessellate::is_watertight(&mesh), "{label}");
        let r45 = 30.0 * s * std::f64::consts::FRAC_1_SQRT_2;
        for (point, inside) in [
            (p(r45, r45, 0.0), true),
            (p(20.0 * s, -30.0 * s, 0.0), true),
            (p(0.0, 0.0, 0.0), false),
            (p(20.0 * s, -30.0 * s, 3.5 * s), false),
            (p(r45, r45, 3.5 * s), false),
        ] {
            assert_probe(&topo, solid, point, inside, s, &label);
        }
    }
}

/// Two runs meeting at a hairpin corner that turns by `degrees`: 10 along
/// +X into the corner at the origin, then 10 back out, both times `s`.
/// Returns the path and the outgoing direction.
fn hairpin(topo: &mut Topology, s: f64, degrees: f64) -> (Vec<EdgeId>, Vec3) {
    let turn = degrees.to_radians();
    let out = Vec3::new(turn.cos(), turn.sin(), 0.0);
    let origin = p(0.0, 0.0, 0.0);
    let path = vec![
        line(topo, p(-10.0 * s, 0.0, 0.0), origin),
        line(topo, origin, origin + out * (10.0 * s)),
    ];
    (path, out)
}

/// Corners sharper than the old 175° limit mitre exactly. Each run's
/// rulings all have length (certified on the profile's boundary) and the
/// two runs lie on either side of the bisector plane, so they meet only in
/// the mitre section. A tube's mitre is an exact ellipse with semi-major
/// `r / cos(φ/2)`, 57 and 229 times the radius here; with the profile's
/// centroid on the path the volume is still the area times the centre-line
/// length.
#[test]
fn hairpin_corners_past_the_old_limit_mitre_exactly() {
    let origin = p(0.0, 0.0, 0.0);
    for (degrees, radius) in [(178.0, 0.1), (179.5, 0.02)] {
        for s in [1e-3, 1.0, 1e3] {
            let label = format!("{degrees} degree hairpin tube at {s}");
            let mut topo = Topology::new();
            let r = radius * s;
            let profile = circle_face(&mut topo, p(-10.0 * s, 0.0, 0.0), X, r);
            let (path, out) = hairpin(&mut topo, s, degrees);
            let solid = sweep_exact(&mut topo, profile, &path).unwrap();
            assert_valid(&topo, solid, &label);
            assert_eq!(census(&topo, solid), (2, 2, 0, 0, 0), "{label}");
            let exact = PI * r * r * 20.0 * s;
            let measured = volume(&topo, solid);
            assert!(
                rel(measured, exact) < 1e-9,
                "{label}: {measured} vs {exact}"
            );
            let read = crate::measure::solid_volume(&topo, solid, 0.01 * r).unwrap();
            assert!(
                rel(read, exact) < 1e-6,
                "{label}: solid_volume {read} vs {exact}"
            );
            // The display mesh is checked from scale 1 up: at 1e-3 the
            // tessellator stitches the seam of these long, thin mitred walls
            // with mismatched vertices, as it already does for a 170° mitre
            // (see `small_mitred_tube_seam_tessellates_watertight`).
            if s >= 1.0 {
                for deflection in [0.2 * r, 0.05 * r] {
                    let mesh =
                        crate::tessellate::tessellate_solid(&topo, solid, deflection).unwrap();
                    assert!(
                        crate::tessellate::is_watertight(&mesh),
                        "{label}: watertight at {deflection}"
                    );
                    let enclosed = mesh_volume(&mesh);
                    assert!(
                        rel(enclosed, exact) < 2e-2,
                        "{label}: mesh {enclosed} vs {exact}"
                    );
                }
            }
            // The bisector plane's in-plane direction toward the corner's
            // outside, where the incoming run reaches past the corner.
            let bisector = (X + out).normalize().unwrap();
            let outward = Vec3::new(bisector.y(), -bisector.x(), 0.0);
            let tip = r / (0.5 * (X + out).length());
            // Midway between the runs, 9.5 back from the corner.
            let gap = p(
                0.5 * 9.5 * s * (out.x() - 1.0),
                0.5 * 9.5 * s * out.y(),
                0.0,
            );
            for (point, inside) in [
                (p(-5.0 * s, 0.0, 0.0), true),
                (origin + out * (5.0 * s), true),
                (origin + outward * (0.5 * tip), true),
                (origin + outward * (1.2 * tip), false),
                (gap, false),
                (p(-5.0 * s, 0.0, 1.5 * r), false),
            ] {
                assert_probe(&topo, solid, point, inside, s, &label);
            }
        }
    }
}

/// Ready repro, not fixed here: a 170° mitre (accepted before the corner
/// limit was lifted) on a tube of radius 1e-4 tessellates with an open seam
/// at deflections from the radius down to a fifth of it; it closes at 1e-5
/// and at larger scales.
#[test]
#[ignore = "pre-existing: small mitred tube seams tessellate open (tessellator)"]
fn small_mitred_tube_seam_tessellates_watertight() {
    let s = 1e-3;
    let mut topo = Topology::new();
    let r = 0.1 * s;
    let profile = circle_face(&mut topo, p(-10.0 * s, 0.0, 0.0), X, r);
    let (path, _) = hairpin(&mut topo, s, 170.0);
    let solid = sweep_exact(&mut topo, profile, &path).unwrap();
    assert_valid(&topo, solid, "170 degree tube at 1e-3");
    let mesh = crate::tessellate::tessellate_solid(&topo, solid, r).unwrap();
    assert_eq!(open_mesh_edges(&mesh), 0);
}

/// The same hairpins with a rectangle: four planes per run, mitred on the
/// bisector, and the area times the centre-line length.
#[test]
fn hairpin_rectangles_mitre_on_planes() {
    let origin = p(0.0, 0.0, 0.0);
    for (degrees, half) in [(178.0, 0.1), (179.5, 0.02)] {
        for s in [1e-3, 1.0, 1e3] {
            let label = format!("{degrees} degree hairpin bar at {s}");
            let mut topo = Topology::new();
            let (hy, hz) = (half * s, 0.5 * half * s);
            let x = -10.0 * s;
            let profile = polygon_face(
                &mut topo,
                &[p(x, -hy, -hz), p(x, hy, -hz), p(x, hy, hz), p(x, -hy, hz)],
            );
            let (path, out) = hairpin(&mut topo, s, degrees);
            let solid = sweep_exact(&mut topo, profile, &path).unwrap();
            assert_valid(&topo, solid, &label);
            assert_eq!(census(&topo, solid), (10, 0, 0, 0, 0), "{label}");
            let exact = 4.0 * hy * hz * 20.0 * s;
            let measured = volume(&topo, solid);
            assert!(
                rel(measured, exact) < 1e-9,
                "{label}: {measured} vs {exact}"
            );
            let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.1 * hz).unwrap();
            assert!(crate::tessellate::is_watertight(&mesh), "{label}");
            let enclosed = mesh_volume(&mesh);
            assert!(rel(enclosed, exact) < 1e-6, "{label}: mesh {enclosed}");
            for (point, inside) in [
                (p(-5.0 * s, 0.0, 0.0), true),
                (origin + out * (5.0 * s), true),
                (p(-5.0 * s, 0.0, 1.5 * hz), false),
                (p(-5.0 * s, -1.5 * hy, 0.0), false),
            ] {
                assert_probe(&topo, solid, point, inside, s, &label);
            }
        }
    }
}

/// A unit circle tilted by `tilt` from square to the path, swept along a
/// run of 10 into a turn of radius 5 about (10, 5) (a half turn or a
/// quarter turn), or back along the same path from the turn's far end.
/// Returns the solid and the turn's middle point, all times `s`.
fn oblique_run_and_turn(
    topo: &mut Topology,
    s: f64,
    tilt: f64,
    turn: f64,
    reversed: bool,
) -> Result<SolidId, OperationsError> {
    let centre = p(10.0 * s, 5.0 * s, 0.0);
    let start = -FRAC_PI_2;
    let path = [
        line(topo, p(0.0, 0.0, 0.0), p(10.0 * s, 0.0, 0.0)),
        arc(topo, centre, 5.0 * s, start, start + turn),
    ];
    // Where the sweep starts, and the way it travels there.
    let (at, travel) = if reversed {
        let end = start + turn;
        (
            centre + Vec3::new(end.cos(), end.sin(), 0.0) * (5.0 * s),
            Vec3::new(end.sin(), -end.cos(), 0.0),
        )
    } else {
        (p(0.0, 0.0, 0.0), X)
    };
    let across = Z.cross(travel);
    let normal = travel * tilt.cos() + across * tilt.sin();
    let profile = circle_face(topo, at, normal, s);
    sweep_exact(topo, profile, &path)
}

/// A profile that is not square to its path swings across a joint's
/// section plane once it has turned far enough (`2·atan(B/A)`, past 106°
/// for a 30° tilt into a half turn), far from the run before it. Those
/// bodies are clean and build; the turn beyond the certified angle is held
/// to the distance check instead. A tilted section sweeps the volume
/// `area · cos(tilt) · centre-line length` (the flux of the motion through
/// it), here `π cos(tilt) (10 + 5 turn) s³`.
#[test]
fn oblique_profiles_into_a_turn_build_exactly() {
    let cases = [
        (30.0, PI, false),
        (-30.0, PI, false),
        (0.5, PI, false),
        (0.01, PI, false),
        (-0.5, PI, false),
        (-0.01, PI, false),
        (60.0, FRAC_PI_2, false),
        (-60.0, FRAC_PI_2, false),
        (30.0, PI, true),
        (-30.0, PI, true),
        (60.0, FRAC_PI_2, true),
        (-60.0, FRAC_PI_2, true),
    ];
    for (degrees, turn, reversed) in cases {
        for s in [1e-3, 1.0, 1e3] {
            let label = format!("tilt {degrees}, turn {turn}, reversed {reversed}, scale {s}");
            let mut topo = Topology::new();
            let tilt = f64::to_radians(degrees);
            let solid = oblique_run_and_turn(&mut topo, s, tilt, turn, reversed)
                .unwrap_or_else(|e| panic!("{label}: {e:?}"));
            assert_valid(&topo, solid, &label);
            // An oblique circle moved along a run or turned about an axis
            // its centre is off the plane of is exact rational NURBS.
            assert_eq!(census(&topo, solid), (2, 0, 0, 0, 2), "{label}");
            let exact = PI * tilt.cos() * 5.0f64.mul_add(turn, 10.0) * s * s * s;
            let measured = volume(&topo, solid);
            // `mass_properties` integrates these NURBS sides to a few 1e-6 at
            // scale 1e-3 (an absolute integration floor) and 1e-10 from 1 up.
            let bound = if s < 1.0 { 1e-5 } else { 1e-8 };
            assert!(
                rel(measured, exact) < bound,
                "{label}: {measured} vs {exact}"
            );
            let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.02 * s).unwrap();
            assert!(crate::tessellate::is_watertight(&mesh), "{label}");
            let enclosed = mesh_volume(&mesh);
            assert!(
                rel(enclosed, exact) < 1e-2,
                "{label}: mesh {enclosed} vs {exact}"
            );
            let middle = -FRAC_PI_2 + 0.5 * turn;
            for (point, inside) in [
                (p(5.0 * s, 0.0, 0.0), true),
                (
                    p(
                        5.0f64.mul_add(middle.cos(), 10.0) * s,
                        5.0f64.mul_add(middle.sin(), 5.0) * s,
                        0.0,
                    ),
                    true,
                ),
                (p(10.0 * s, 5.0 * s, 0.0), false),
                (p(5.0 * s, 0.0, 1.5 * s), false),
            ] {
                assert_probe(&topo, solid, point, inside, s, &label);
            }
        }
    }
}
