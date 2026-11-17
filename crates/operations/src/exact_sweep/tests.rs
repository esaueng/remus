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
    let cases: [(&str, Sweep, bool, &str); 8] = [
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
