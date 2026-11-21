//! `sweep_exact_with_options`: twist and scale along a straight path.
//!
//! Every section of these sweeps is the profile scaled by `σ(s)` (and
//! turned), so by Cavalieri the volume is `A·L·(1 + σ₁ + σ₁²)/3` whatever
//! the twist.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_line_edge, make_polygon_wire};
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);

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

fn circle_face(topo: &mut Topology, center: Point3, radius: f64) -> FaceId {
    let wire = circle_wire(topo, center, X, radius);
    plane_face(topo, wire, vec![], X, center)
}

/// A planar polygon face in `x = 0`, wound counter-clockwise about +X.
fn polygon_face(topo: &mut Topology, points: &[Point3]) -> FaceId {
    let wire = make_polygon_wire(topo, points, 1e-7).unwrap();
    let n = crate::winding::newell_normal(points);
    plane_face(topo, wire, vec![], n, points[0])
}

/// A 4 × 4 square about the X axis in `x = 0`.
fn square(topo: &mut Topology) -> FaceId {
    polygon_face(
        topo,
        &[
            p(0.0, -2.0, -2.0),
            p(0.0, 2.0, -2.0),
            p(0.0, 2.0, 2.0),
            p(0.0, -2.0, 2.0),
        ],
    )
}

fn line(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
    make_line_edge(topo, a, b, 1e-7).unwrap()
}

fn options(twist: f64, end_scale: f64) -> ExactSweepOptions {
    ExactSweepOptions { twist, end_scale }
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
}

/// `(planes, cylinders, cones, nurbs)`.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let mut c = (0, 0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => c.0 += 1,
            FaceSurface::Cylinder(_) => c.1 += 1,
            FaceSurface::Cone(_) => c.2 += 1,
            FaceSurface::Nurbs(_) => c.3 += 1,
            other => panic!("unexpected {}", other.type_tag()),
        }
    }
    c
}

fn assert_inside(topo: &Topology, solid: SolidId, at: Point3, inside: bool, label: &str) {
    let found = crate::classify::classify_point(topo, solid, at, 0.01, 1e-9).unwrap();
    let expected = if inside {
        crate::classify::PointClassification::Inside
    } else {
        crate::classify::PointClassification::Outside
    };
    assert_eq!(found, expected, "{label}: probe {at:?}");
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

fn frustum(area: f64, length: f64, end_scale: f64) -> f64 {
    area * length * end_scale.mul_add(end_scale, 1.0 + end_scale) / 3.0
}

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

/// Valid, watertight at two deflections, the mesh near the closed form and
/// the certified volume within `tolerance` of it.
fn assert_solid(topo: &Topology, solid: SolidId, exact: f64, tolerance: f64, label: &str) {
    assert_valid(topo, solid, label);
    for deflection in [0.05, 0.01] {
        let mesh = crate::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(
            open_mesh_edges(&mesh),
            0,
            "{label}: watertight at {deflection}"
        );
        let enclosed = mesh_volume(&mesh);
        // A chorded mesh: a sanity check, not the measurement.
        assert!(
            rel(enclosed, exact) < 5e-2,
            "{label}: mesh {enclosed} vs {exact}"
        );
    }
    let measured = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
    assert!(
        rel(measured, exact) < tolerance,
        "{label}: solid_volume {measured} vs {exact}"
    );
}

#[test]
fn default_options_are_the_plain_exact_sweep() {
    let mut a = Topology::new();
    let profile = square(&mut a);
    let path = line(&mut a, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let plain = sweep_exact(&mut a, profile, &[path]).unwrap();
    let mut b = Topology::new();
    let profile = square(&mut b);
    let path = line(&mut b, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let with =
        sweep_exact_with_options(&mut b, profile, &[path], &ExactSweepOptions::default()).unwrap();
    assert_eq!(census(&a, plain), census(&b, with));
    assert_eq!(a.num_faces(), b.num_faces());
    assert_solid(&b, with, 160.0, 1e-12, "default");
}

#[test]
fn a_scaled_square_is_a_frustum_of_planes() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let solid = sweep_exact_with_options(&mut topo, profile, &[path], &options(0.0, 0.5)).unwrap();
    assert_eq!(census(&topo, solid), (6, 0, 0, 0));
    assert_solid(
        &topo,
        solid,
        frustum(16.0, 10.0, 0.5),
        1e-9,
        "square frustum",
    );
}

#[test]
fn a_scaled_circle_on_the_path_is_one_exact_cone() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), 2.0);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let solid = sweep_exact_with_options(&mut topo, profile, &[path], &options(0.0, 1.5)).unwrap();
    assert_eq!(census(&topo, solid), (2, 0, 1, 0));
    assert_solid(&topo, solid, frustum(PI * 4.0, 10.0, 1.5), 1e-9, "cone");
}

#[test]
fn a_twisted_square_keeps_its_volume() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let solid =
        sweep_exact_with_options(&mut topo, profile, &[path], &options(FRAC_PI_2, 1.0)).unwrap();
    assert_eq!(census(&topo, solid), (2, 0, 0, 4));
    assert_solid(&topo, solid, 160.0, 1e-6, "twisted square");
}

#[test]
fn a_twisted_circle_on_the_path_stays_a_cylinder() {
    let mut topo = Topology::new();
    let profile = circle_face(&mut topo, p(0.0, 0.0, 0.0), 2.0);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let solid = sweep_exact_with_options(&mut topo, profile, &[path], &options(PI, 1.0)).unwrap();
    assert_eq!(census(&topo, solid), (2, 1, 0, 0));
    assert_solid(&topo, solid, PI * 40.0, 1e-9, "twisted cylinder");
}

#[test]
fn a_twisted_and_scaled_square_meets_cavalieri() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let solid =
        sweep_exact_with_options(&mut topo, profile, &[path], &options(-0.25 * PI, 2.0)).unwrap();
    assert_eq!(census(&topo, solid), (2, 0, 0, 4));
    assert_solid(
        &topo,
        solid,
        frustum(16.0, 10.0, 2.0),
        1e-6,
        "twisted frustum",
    );
    // The far cap is the square doubled and turned an eighth of a turn: a
    // diamond reaching 4√2 along each axis, clear of the unturned corners.
    assert_inside(
        &topo,
        solid,
        p(10.0 - 1e-2, 5.5, 0.0),
        true,
        "turned corner on +Y",
    );
    assert_inside(
        &topo,
        solid,
        p(10.0 - 1e-2, 0.0, -5.5),
        true,
        "turned corner on -Z",
    );
    assert_inside(
        &topo,
        solid,
        p(10.0 - 1e-2, 3.9, 3.9),
        false,
        "unturned corner",
    );
}

#[test]
fn an_off_axis_triangle_scales_and_turns_about_the_path() {
    let mut topo = Topology::new();
    // Clear of the axis: area 4.5.
    let profile = polygon_face(
        &mut topo,
        &[p(0.0, 1.0, 1.0), p(0.0, 4.0, 1.0), p(0.0, 1.0, 4.0)],
    );
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(12.0, 0.0, 0.0));
    let solid = sweep_exact_with_options(&mut topo, profile, &[path], &options(1.0, 0.25)).unwrap();
    assert_solid(&topo, solid, frustum(4.5, 12.0, 0.25), 1e-6, "triangle");
}

#[test]
fn a_ring_with_a_concentric_bore_scales_both_walls_into_cones() {
    let mut topo = Topology::new();
    let outer = circle_wire(&mut topo, p(0.0, 0.0, 0.0), X, 3.0);
    let inner = circle_wire(&mut topo, p(0.0, 0.0, 0.0), -X, 1.0);
    let profile = plane_face(&mut topo, outer, vec![inner], X, p(0.0, 0.0, 0.0));
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(8.0, 0.0, 0.0));
    let solid = sweep_exact_with_options(&mut topo, profile, &[path], &options(0.5, 0.5)).unwrap();
    assert_eq!(census(&topo, solid), (2, 0, 2, 0));
    assert_solid(&topo, solid, frustum(PI * 8.0, 8.0, 0.5), 1e-9, "ring");
}

#[test]
fn the_path_may_be_split_reversed_and_start_off_the_profile() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    // Collinear pieces given backwards, starting 2 behind the profile: the
    // sweep starts at the end nearer the profile and travels 15.
    let path = [
        line(&mut topo, p(13.0, 0.0, 0.0), p(5.0, 0.0, 0.0)),
        line(&mut topo, p(5.0, 0.0, 0.0), p(-2.0, 0.0, 0.0)),
    ];
    let solid = sweep_exact_with_options(&mut topo, profile, &path, &options(0.0, 0.5)).unwrap();
    assert_solid(&topo, solid, frustum(16.0, 15.0, 0.5), 1e-9, "split path");
    assert_inside(&topo, solid, p(14.9, 0.0, 0.0), true, "far end");
    assert_inside(&topo, solid, p(-1.0, 0.0, 0.0), false, "behind the profile");
}

#[test]
fn twist_and_scale_refusals_are_typed_and_leave_the_topology_unchanged() {
    type Build = fn(&mut Topology) -> Result<SolidId, OperationsError>;
    let cases: [(&str, Build, bool, &str); 7] = [
        (
            "an arc in the path",
            |topo| {
                let profile = circle_face(topo, p(0.0, 0.0, 0.0), 1.0);
                let circle = remus_math::curves::Circle3D::with_axes(
                    p(0.0, 10.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    10.0,
                    Vec3::new(0.0, -1.0, 0.0),
                    X,
                )
                .unwrap();
                let a = topo.add_vertex(remus_topology::vertex::Vertex::new(
                    circle.evaluate(0.0),
                    1e-7,
                ));
                let b = topo.add_vertex(remus_topology::vertex::Vertex::new(
                    circle.evaluate(FRAC_PI_2),
                    1e-7,
                ));
                let mut edge = remus_topology::edge::Edge::new(
                    a,
                    b,
                    remus_topology::edge::EdgeCurve::Circle(circle),
                );
                edge.set_trim(Some((0.0, FRAC_PI_2)));
                let path = topo.add_edge(edge);
                sweep_exact_with_options(topo, profile, &[path], &options(1.0, 1.0))
            },
            true,
            "is an arc",
        ),
        (
            "a corner in the path",
            |topo| {
                let profile = square(topo);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0)),
                    line(topo, p(10.0, 0.0, 0.0), p(10.0, 10.0, 0.0)),
                ];
                sweep_exact_with_options(topo, profile, &path, &options(0.0, 0.5))
            },
            true,
            "turns a corner",
        ),
        (
            "a tilted profile",
            |topo| {
                let profile = square(topo);
                let path = line(topo, p(0.0, 0.0, 0.0), p(10.0, 3.0, 0.0));
                sweep_exact_with_options(topo, profile, &[path], &options(1.0, 1.0))
            },
            true,
            "square to the path",
        ),
        (
            "a zero end scale",
            |topo| {
                let profile = square(topo);
                let path = line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
                sweep_exact_with_options(topo, profile, &[path], &options(0.0, 0.0))
            },
            false,
            "finite and positive",
        ),
        (
            "a non-finite twist",
            |topo| {
                let profile = square(topo);
                let path = line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
                sweep_exact_with_options(topo, profile, &[path], &options(f64::NAN, 1.0))
            },
            false,
            "must be finite",
        ),
        (
            "a closed hole off the path",
            |topo| {
                let outer = make_polygon_wire(
                    topo,
                    &[
                        p(0.0, -4.0, -4.0),
                        p(0.0, 4.0, -4.0),
                        p(0.0, 4.0, 4.0),
                        p(0.0, -4.0, 4.0),
                    ],
                    1e-7,
                )
                .unwrap();
                let hole = circle_wire(topo, p(0.0, 2.0, 0.0), -X, 1.0);
                let profile = plane_face(topo, outer, vec![hole], X, p(0.0, 0.0, 0.0));
                let path = line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
                sweep_exact_with_options(topo, profile, &[path], &options(0.0, 0.5))
            },
            false,
            "closed edge off the axis",
        ),
        (
            "a twisted closed hole off the path",
            |topo| {
                let outer = make_polygon_wire(
                    topo,
                    &[
                        p(0.0, -4.0, -4.0),
                        p(0.0, 4.0, -4.0),
                        p(0.0, 4.0, 4.0),
                        p(0.0, -4.0, 4.0),
                    ],
                    1e-7,
                )
                .unwrap();
                let hole = circle_wire(topo, p(0.0, 2.0, 0.0), -X, 1.0);
                let profile = plane_face(topo, outer, vec![hole], X, p(0.0, 0.0, 0.0));
                let path = line(topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
                sweep_exact_with_options(topo, profile, &[path], &options(1.0, 1.0))
            },
            false,
            "closed edge off the axis",
        ),
    ];
    for (label, build, is_unsupported, phrase) in cases {
        let mut topo = Topology::new();
        let error = build(&mut topo).expect_err(label);
        let reason = match &error {
            OperationsError::Unsupported { reason, .. } => {
                assert!(is_unsupported, "{label}: {error:?}");
                reason.clone()
            }
            OperationsError::InvalidInput { reason } => {
                assert!(!is_unsupported, "{label}: {error:?}");
                reason.clone()
            }
            other => panic!("{label}: untyped {other:?}"),
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
        // Atomic: a refused call leaves only its one profile face behind.
        assert_eq!(topo.num_faces(), 1, "{label}: faces left behind");
    }
}
