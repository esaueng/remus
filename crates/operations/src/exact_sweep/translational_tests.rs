//! Exact sweeps that keep the profile's orientation.
//!
//! Each plane `n·x = h` cuts such a body in one translated copy of the
//! profile (the path runs steadily along `n`), so by Cavalieri its volume is
//! the profile's area times the path's rise along `n`, whatever the path's
//! shape.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_6, PI};

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_line_edge, make_polygon_wire};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn keep() -> ExactSweepOptions {
    ExactSweepOptions {
        orientation: ExactSweepOrientation::KeepProfile,
        ..ExactSweepOptions::default()
    }
}

fn plane_face(topo: &mut Topology, wire: WireId, holes: Vec<WireId>, at: Point3) -> FaceId {
    topo.add_face(Face::new(
        wire,
        holes,
        FaceSurface::Plane {
            normal: X,
            d: crate::dot_normal_point(X, at),
        },
    ))
}

fn circle_wire(topo: &mut Topology, center: Point3, normal: Vec3, radius: f64) -> WireId {
    let edge = make_circle_edge(topo, center, normal, radius, 1e-7).unwrap();
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
}

/// A radius-1 circle about the origin in `x = 0`.
fn disc(topo: &mut Topology) -> FaceId {
    let wire = circle_wire(topo, p(0.0, 0.0, 0.0), X, 1.0);
    plane_face(topo, wire, vec![], p(0.0, 0.0, 0.0))
}

/// A 2 × 2 square about the origin in `x = 0`.
fn square(topo: &mut Topology) -> FaceId {
    let wire = make_polygon_wire(
        topo,
        &[
            p(0.0, -1.0, -1.0),
            p(0.0, 1.0, -1.0),
            p(0.0, 1.0, 1.0),
            p(0.0, -1.0, 1.0),
        ],
        1e-7,
    )
    .unwrap();
    plane_face(topo, wire, vec![], p(0.0, 0.0, 0.0))
}

/// A 4 × 4 square in `x = 0` with a radius-1 bore at its centre.
fn holed_square(topo: &mut Topology) -> FaceId {
    let outer = make_polygon_wire(
        topo,
        &[
            p(0.0, -2.0, -2.0),
            p(0.0, 2.0, -2.0),
            p(0.0, 2.0, 2.0),
            p(0.0, -2.0, 2.0),
        ],
        1e-7,
    )
    .unwrap();
    let hole = circle_wire(topo, p(0.0, 0.0, 0.0), -X, 1.0);
    plane_face(topo, outer, vec![hole], p(0.0, 0.0, 0.0))
}

fn line(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
    make_line_edge(topo, a, b, 1e-7).unwrap()
}

/// An arc of radius 10 about `(0, 10, 0)` in `z = 0`, counter-clockwise
/// about +Z from angle `a` to `b`; angle −π/2 is the origin.
fn arc(topo: &mut Topology, a: f64, b: f64) -> EdgeId {
    let circle = Circle3D::with_axes(p(0.0, 10.0, 0.0), Z, 10.0, X, Y).unwrap();
    let start = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
    let end = topo.add_vertex(Vertex::new(circle.evaluate(b), 1e-7));
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
    edge.set_trim(Some((a, b)));
    topo.add_edge(edge)
}

/// A cubic spline through the given control points, clamped.
fn spline(topo: &mut Topology, points: &[Point3]) -> EdgeId {
    let curve = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        points.to_vec(),
        vec![1.0; 4],
    )
    .unwrap();
    let start = topo.add_vertex(Vertex::new(points[0], 1e-7));
    let end = topo.add_vertex(Vertex::new(points[3], 1e-7));
    let mut edge = Edge::new(start, end, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some((0.0, 1.0)));
    topo.add_edge(edge)
}

fn sweep(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
) -> Result<SolidId, OperationsError> {
    sweep_exact_with_options(topo, profile, path, &keep())
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

fn open_mesh_edges(mesh: &crate::tessellate::TriangleMesh) -> usize {
    use std::collections::BTreeMap;
    let mut uses: BTreeMap<(u32, u32), i32> = BTreeMap::new();
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

/// Valid, watertight at two deflections, the chorded mesh near the closed
/// form and the measured volume within `tolerance` of it.
fn assert_solid(topo: &Topology, solid: SolidId, exact: f64, tolerance: f64, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
    for deflection in [0.05, 0.01] {
        let mesh = crate::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(
            open_mesh_edges(&mesh),
            0,
            "{label}: watertight at {deflection}"
        );
        let enclosed = mesh_volume(&mesh);
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

fn assert_inside(topo: &Topology, solid: SolidId, at: Point3, inside: bool, label: &str) {
    let found = crate::classify::classify_point(topo, solid, at, 0.01, 1e-9).unwrap();
    let expected = if inside {
        crate::classify::PointClassification::Inside
    } else {
        crate::classify::PointClassification::Outside
    };
    assert_eq!(found, expected, "{label}: probe {at:?}");
}

/// `(planes, cylinders, nurbs)`.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    let mut c = (0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => c.0 += 1,
            FaceSurface::Cylinder(_) => c.1 += 1,
            FaceSurface::Nurbs(_) => c.2 += 1,
            other => panic!("unexpected {}", other.type_tag()),
        }
    }
    c
}

#[test]
fn along_a_line_it_is_the_plain_exact_sweep() {
    let mut topo = Topology::new();
    let profile = disc(&mut topo);
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
    let solid = sweep(&mut topo, profile, &[path]).unwrap();
    assert_eq!(census(&topo, solid), (2, 1, 0));
    assert_solid(&topo, solid, PI * 10.0, 1e-9, "line");
}

#[test]
fn along_an_arc_the_square_stays_parallel_to_itself() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = arc(&mut topo, -FRAC_PI_2, -FRAC_PI_6);
    let solid = sweep(&mut topo, profile, &[path]).unwrap();
    assert_eq!(census(&topo, solid), (2, 0, 4));
    // The rise along +X: 10·cos(−π/6).
    let rise = 10.0 * FRAC_PI_6.cos();
    assert_solid(&topo, solid, 4.0 * rise, 1e-6, "arc");
    // The far cap is the square moved to (8.66, 5, 0), unturned: its corner
    // region is inside, and nothing beyond its edge is.
    assert_inside(
        &topo,
        solid,
        p(rise - 1e-2, 5.95, 0.95),
        true,
        "unturned corner",
    );
    assert_inside(
        &topo,
        solid,
        p(rise - 1e-2, 6.2, 0.0),
        false,
        "past the edge",
    );
}

#[test]
fn along_a_spline_it_is_exact() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = spline(
        &mut topo,
        &[
            p(0.0, 0.0, 0.0),
            p(3.0, 2.0, 1.0),
            p(6.0, -1.0, 3.0),
            p(10.0, 1.0, 0.0),
        ],
    );
    let solid = sweep(&mut topo, profile, &[path]).unwrap();
    assert_eq!(census(&topo, solid), (2, 0, 4));
    assert_solid(&topo, solid, 4.0 * 10.0, 1e-6, "spline");
    // Halfway along the spline, the section stands on C(1/2).
    let mid = p(4.625, 0.5, 1.5);
    assert_inside(&topo, solid, mid, true, "on the spline's midpoint");
}

#[test]
fn a_corner_creases_the_body_with_no_mitre() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = [
        line(&mut topo, p(0.0, 0.0, 0.0), p(5.0, 0.0, 0.0)),
        line(&mut topo, p(5.0, 0.0, 0.0), p(10.0, 5.0, 0.0)),
    ];
    let solid = sweep(&mut topo, profile, &path).unwrap();
    assert_eq!(census(&topo, solid), (10, 0, 0));
    assert_solid(&topo, solid, 4.0 * 10.0, 1e-9, "corner");
}

#[test]
fn a_corner_next_to_an_arc_builds_exactly() {
    let mut topo = Topology::new();
    let profile = holed_square(&mut topo);
    // A line along +X, then the arc leaving it at 30° (not tangent).
    let start = arc_start(-FRAC_PI_2 + FRAC_PI_6);
    let path = [
        line(&mut topo, p(start.x() - 4.0, start.y(), 0.0), start),
        arc(&mut topo, -FRAC_PI_2 + FRAC_PI_6, -FRAC_PI_6),
    ];
    let solid = sweep(&mut topo, profile, &path).unwrap();
    let rise = 10.0 * FRAC_PI_6.cos() - (start.x() - 4.0);
    assert_solid(&topo, solid, (16.0 - PI) * rise, 1e-6, "corner by an arc");
}

fn arc_start(angle: f64) -> Point3 {
    p(10.0 * angle.cos(), 10.0 + 10.0 * angle.sin(), 0.0)
}

#[test]
fn the_path_may_come_in_any_order_and_start_off_the_profile() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = [
        line(&mut topo, p(9.0, 4.0, 0.0), p(4.0, 0.0, 0.0)),
        line(&mut topo, p(4.0, 0.0, 0.0), p(-1.0, 0.0, 0.0)),
    ];
    let solid = sweep(&mut topo, profile, &path).unwrap();
    assert_solid(&topo, solid, 4.0 * 10.0, 1e-9, "reordered");
    assert_inside(&topo, solid, p(9.0 - 1e-2, 4.0, 0.0), true, "far end");
}

#[test]
fn refusals_are_typed_and_leave_the_topology_unchanged() {
    type Build = fn(&mut Topology) -> Result<SolidId, OperationsError>;
    let cases: [(&str, Build, &str); 5] = [
        (
            "a path that turns back through the profile's plane",
            |topo| {
                let profile = square(topo);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(5.0, 0.0, 0.0)),
                    line(topo, p(5.0, 0.0, 0.0), p(3.0, 5.0, 0.0)),
                ];
                sweep(topo, profile, &path)
            },
            "turns back",
        ),
        (
            "an arc that ends square to the profile's normal",
            |topo| {
                let profile = disc(topo);
                let path = arc(topo, -FRAC_PI_2, 0.0);
                sweep(topo, profile, &[path])
            },
            "cannot be shown to run steadily",
        ),
        (
            "a path in the profile's plane",
            |topo| {
                let profile = disc(topo);
                let path = line(topo, p(0.0, 0.0, 0.0), p(0.0, 10.0, 0.0));
                sweep(topo, profile, &[path])
            },
            "cannot be shown to run steadily",
        ),
        (
            "a closed path",
            |topo| {
                let profile = disc(topo);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(5.0, 0.0, 0.0)),
                    line(topo, p(5.0, 0.0, 0.0), p(5.0, 5.0, 0.0)),
                    line(topo, p(5.0, 5.0, 0.0), p(0.0, 0.0, 0.0)),
                ];
                sweep(topo, profile, &path)
            },
            "closed sweep path",
        ),
        (
            "a broken path",
            |topo| {
                let profile = disc(topo);
                let path = [
                    line(topo, p(0.0, 0.0, 0.0), p(5.0, 0.0, 0.0)),
                    line(topo, p(6.0, 0.0, 0.0), p(9.0, 0.0, 0.0)),
                ];
                sweep(topo, profile, &path)
            },
            "connected chain",
        ),
    ];
    for (label, build, phrase) in cases {
        let mut topo = Topology::new();
        let error = build(&mut topo).expect_err(label);
        let OperationsError::InvalidInput { reason } = &error else {
            panic!("{label}: untyped {error:?}");
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
        assert_eq!(topo.num_faces(), 1, "{label}: faces left behind");
    }
}

/// Review regression: a join within the relative join tolerance but well
/// off by the absolute one (0.5 back, far from the origin) builds sections
/// where each step ends, so the shell closes.
#[test]
fn sections_follow_each_step_across_a_loose_join() {
    let mut topo = Topology::new();
    let at = 1e9;
    let wire = make_polygon_wire(
        &mut topo,
        &[
            p(at - 1.0, -1.0, 0.0),
            p(at + 1.0, -1.0, 0.0),
            p(at + 1.0, 1.0, 0.0),
            p(at - 1.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![p(at, 0.0, 0.0), p(at + 1.0, 0.0, 5.0), p(at, 0.0, 10.0)],
        vec![1.0; 3],
    )
    .unwrap();
    let a = topo.add_vertex(Vertex::new(p(at, 0.0, 0.0), 1e-7));
    let b = topo.add_vertex(Vertex::new(p(at, 0.0, 10.0), 1e-7));
    let mut bend = Edge::new(a, b, EdgeCurve::NurbsCurve(curve));
    bend.set_trim(Some((0.0, 1.0)));
    let bend = topo.add_edge(bend);
    let rest = line(&mut topo, p(at, 0.0, 9.5), p(at, 0.0, 20.0));
    match sweep(&mut topo, profile, &[bend, rest]) {
        Ok(solid) => {
            let report = crate::validate::validate_solid(&topo, solid).unwrap();
            assert!(report.is_valid(), "{report:?}");
            let measured = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
            assert!(rel(measured, 4.0 * 20.5) < 1e-6, "{measured}");
        }
        Err(OperationsError::InvalidInput { .. }) => {}
        Err(other) => panic!("untyped {other:?}"),
    }
}

/// Review regression: a profile whose single closed spline edge spends most
/// of its parameter on two short spans still winds the right way.
#[test]
fn a_spline_loop_winds_by_its_whole_shape() {
    let mut topo = Topology::new();
    let corners = [
        p(0.0, 0.0, 0.0),
        p(0.0, 1.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(2.0, 2.0, 0.0),
        p(-1.0, 2.0, 0.0),
        p(-1.0, 0.0, 0.0),
        p(0.0, 0.0, 0.0),
    ];
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.25, 0.968_75, 0.972, 0.978, 0.984, 1.0, 1.0],
        corners.to_vec(),
        vec![1.0; 7],
    )
    .unwrap();
    let v = topo.add_vertex(Vertex::new(corners[0], 1e-7));
    let mut edge = Edge::new(v, v, EdgeCurve::NurbsCurve(curve));
    edge.set_trim(Some((0.0, 1.0)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: Z, d: 0.0 },
    ));
    let path = line(&mut topo, p(0.0, 0.0, 0.0), p(0.0, 0.0, 5.0));
    let solid = sweep(&mut topo, profile, &[path]).unwrap();
    let report = crate::validate::validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let measured = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
    assert!(rel(measured, 4.5 * 5.0) < 1e-9, "{measured}");
}

/// Review regressions: an unclamped spline path and one the Bézier
/// decomposition cannot split are refused by type.
#[test]
fn uncertifiable_spline_paths_are_refused_by_type() {
    for (label, degree, knots, heights, trim, phrase) in [
        (
            "unclamped",
            2,
            vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            vec![0.0, 2.0, 4.0, 6.0],
            (2.0, 4.0),
            "unclamped",
        ),
        (
            "near-coincident knots",
            1,
            vec![0.0, 0.0, 0.5, 0.5 + f64::EPSILON, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            (0.0, 1.0),
            "cannot be shown",
        ),
    ] {
        let mut topo = Topology::new();
        let profile = disc(&mut topo);
        let curve = NurbsCurve::new(
            degree,
            knots,
            heights.iter().map(|&h| p(h, 0.1 * h * h, 0.0)).collect(),
            vec![1.0; heights.len()],
        )
        .unwrap();
        let a = topo.add_vertex(Vertex::new(curve.evaluate(trim.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(curve.evaluate(trim.1), 1e-7));
        let mut edge = Edge::new(a, b, EdgeCurve::NurbsCurve(curve));
        edge.set_trim(Some(trim));
        let edge = topo.add_edge(edge);
        let error = sweep(&mut topo, profile, &[edge]).expect_err(label);
        let OperationsError::InvalidInput { reason } = &error else {
            panic!("{label}: untyped {error:?}");
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
        assert_eq!(topo.num_faces(), 1, "{label}: faces left behind");
    }
}
