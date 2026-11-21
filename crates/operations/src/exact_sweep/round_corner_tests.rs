//! Round corners: a corner between straight runs replaced by the tangent arc
//! of the given radius. The profile's centroid rides the centre line, so by
//! Pappus every volume is the profile's area times the rounded centre line's
//! length.

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

fn round(radius: f64) -> ExactSweepOptions {
    ExactSweepOptions {
        corner: ExactSweepCorner::Round { radius },
        ..ExactSweepOptions::default()
    }
}

/// A disc of radius `r` about the origin in `x = 0`.
fn disc(topo: &mut Topology, r: f64) -> FaceId {
    let edge = make_circle_edge(topo, p(0.0, 0.0, 0.0), X, r, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: X, d: 0.0 },
    ))
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
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: X, d: 0.0 },
    ))
}

fn line(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
    make_line_edge(topo, a, b, 1e-7).unwrap()
}

/// 20 along +X, then 20 along +Y.
fn ell(topo: &mut Topology) -> Vec<EdgeId> {
    vec![
        line(topo, p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0)),
        line(topo, p(20.0, 0.0, 0.0), p(20.0, 20.0, 0.0)),
    ]
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

fn assert_solid(topo: &Topology, solid: SolidId, exact: f64, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
    for deflection in [0.05, 0.01] {
        let mesh = crate::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(
            open_mesh_edges(&mesh),
            0,
            "{label}: watertight at {deflection}"
        );
    }
    let measured = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
    assert!(
        rel(measured, exact) < 1e-9,
        "{label}: solid_volume {measured} vs {exact}"
    );
}

/// `(planes, cylinders, tori, nurbs)`.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let mut c = (0, 0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => c.0 += 1,
            FaceSurface::Cylinder(_) => c.1 += 1,
            FaceSurface::Torus(_) => c.2 += 1,
            FaceSurface::Nurbs(_) => c.3 += 1,
            other => panic!("unexpected {}", other.type_tag()),
        }
    }
    c
}

#[test]
fn a_round_corner_bends_a_tube_through_one_torus_band() {
    let mut topo = Topology::new();
    let profile = disc(&mut topo, 1.0);
    let path = ell(&mut topo);
    let solid = sweep_exact_with_options(&mut topo, profile, &path, &round(5.0)).unwrap();
    assert_eq!(census(&topo, solid), (2, 2, 1, 0));
    let centre_line = 2.0 * 15.0 + 5.0 * FRAC_PI_2;
    assert_solid(&topo, solid, PI * centre_line, "tube");
}

#[test]
fn a_round_corner_keeps_a_square_flat_and_round() {
    let mut topo = Topology::new();
    let profile = square(&mut topo);
    let path = ell(&mut topo);
    let solid = sweep_exact_with_options(&mut topo, profile, &path, &round(3.0)).unwrap();
    let centre_line = 2.0 * 17.0 + 3.0 * FRAC_PI_2;
    assert_solid(&topo, solid, 4.0 * centre_line, "square");
}

#[test]
fn every_corner_of_a_closed_square_path_is_rounded_into_a_ring() {
    let mut topo = Topology::new();
    let corners = [
        p(0.0, -10.0, 0.0),
        p(0.0, 10.0, 0.0),
        p(-20.0, 10.0, 0.0),
        p(-20.0, -10.0, 0.0),
    ];
    // Closed: the profile in x = 0 crosses the first run, at its middle.
    let path: Vec<EdgeId> = (0..4)
        .map(|k| line(&mut topo, corners[k], corners[(k + 1) % 4]))
        .collect();
    let profile = {
        // A disc square to +Y, on the first run.
        let edge = make_circle_edge(&mut topo, p(0.0, 0.0, 0.0), Y, 1.0, 1e-7).unwrap();
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane { normal: Y, d: 0.0 },
        ))
    };
    let solid = sweep_exact_with_options(&mut topo, profile, &path, &round(4.0)).unwrap();
    let centre_line = 2.0 * (20.0 - 8.0) + 2.0 * (20.0 - 8.0) + 2.0 * PI * 4.0;
    assert_solid(&topo, solid, PI * centre_line, "ring");
}

/// A quarter arc of radius 10 about `centre` in `z = 0`, counter-clockwise
/// about +Z from angle `a` to `a + π/2`.
fn quarter(topo: &mut Topology, centre: Point3, a: f64) -> EdgeId {
    let circle = Circle3D::with_axes(centre, Z, 10.0, X, Y).unwrap();
    let start = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
    let end = topo.add_vertex(Vertex::new(circle.evaluate(a + FRAC_PI_2), 1e-7));
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
    edge.set_trim(Some((a, a + FRAC_PI_2)));
    topo.add_edge(edge)
}

/// A disc of radius 1 at `at`, square to `normal`.
fn disc_at(topo: &mut Topology, at: Point3, normal: Vec3) -> FaceId {
    let edge = make_circle_edge(topo, at, normal, 1.0, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: crate::dot_normal_point(normal, at),
        },
    ))
}

/// A line arriving along +Y at (10, 0) meets, square, a radius-10 arc about
/// (10, 10) leaving along +X. Rounded with radius 2: the line offset to x = 2
/// meets the arc's circle offset outward to 12 at (2, 10 − √140) from the
/// corner, so the line keeps 20 − (√140 − 10), the fillet turns π/2 − θ and
/// the arc keeps π/2 − θ, with sin θ = 2/12.
#[test]
fn a_round_corner_next_to_an_arc_is_exact() {
    let mut topo = Topology::new();
    let profile = disc_at(&mut topo, p(10.0, -20.0, 0.0), Y);
    let path = [
        line(&mut topo, p(10.0, -20.0, 0.0), p(10.0, 0.0, 0.0)),
        quarter(&mut topo, p(10.0, 10.0, 0.0), -FRAC_PI_2),
    ];
    let solid = sweep_exact_with_options(&mut topo, profile, &path, &round(2.0)).unwrap();
    let theta = (2.0_f64 / 12.0).asin();
    let centre_line =
        (20.0 - (140.0_f64.sqrt() - 10.0)) + 2.0 * (FRAC_PI_2 - theta) + 10.0 * (FRAC_PI_2 - theta);
    assert_eq!(census(&topo, solid), (2, 1, 2, 0));
    assert_solid(&topo, solid, PI * centre_line, "line then arc");
}

/// Two radius-10 quarter arcs meeting square at (10, 0): one about the
/// origin arriving along +Y, one about (10, 10) leaving along +X. Their
/// circles offset outward to 12 meet at the fillet's centre q on the line
/// x + y = 10; each arc keeps the turn from its start to its foot.
#[test]
fn a_round_corner_between_two_arcs_is_exact() {
    let mut topo = Topology::new();
    let profile = disc_at(&mut topo, p(0.0, -10.0, 0.0), X);
    let path = [
        quarter(&mut topo, p(0.0, 0.0, 0.0), -FRAC_PI_2),
        quarter(&mut topo, p(10.0, 10.0, 0.0), -FRAC_PI_2),
    ];
    let solid = sweep_exact_with_options(&mut topo, profile, &path, &round(2.0)).unwrap();
    // q = (5, 5) + h (1, −1)/√2 with h = √(144 − 50).
    let h = (144.0_f64 - 50.0).sqrt();
    let q = (5.0 + h / 2.0_f64.sqrt(), 5.0 - h / 2.0_f64.sqrt());
    // Each foot's turn from its arc's start, by symmetry the same.
    let kept = FRAC_PI_2 + q.1.atan2(q.0);
    let foot_a = (q.0 * 10.0 / 12.0, q.1 * 10.0 / 12.0);
    let foot_b = (
        10.0 + (q.0 - 10.0) * 10.0 / 12.0,
        10.0 + (q.1 - 10.0) * 10.0 / 12.0,
    );
    let (u, v) = (
        (foot_a.0 - q.0, foot_a.1 - q.1),
        (foot_b.0 - q.0, foot_b.1 - q.1),
    );
    let fillet = (u.0 * v.1 - u.1 * v.0).abs().atan2(u.0 * v.0 + u.1 * v.1);
    let centre_line = 2.0 * 10.0 * kept + 2.0 * fillet;
    assert_solid(&topo, solid, PI * centre_line, "arc then arc");
}

#[test]
fn round_corner_refusals_are_typed_and_leave_the_topology_unchanged() {
    type Build = fn(&mut Topology) -> Result<SolidId, OperationsError>;
    let cases: [(&str, Build, bool, &str); 6] = [
        (
            "a radius too large for its run",
            |topo| {
                let profile = disc(topo, 1.0);
                let path = ell(topo);
                sweep_exact_with_options(topo, profile, &path, &round(25.0))
            },
            false,
            "no round corner of radius 25 fits",
        ),
        (
            "a radius smaller than the profile",
            |topo| {
                let profile = disc(topo, 2.0);
                let path = ell(topo);
                sweep_exact_with_options(topo, profile, &path, &round(1.0))
            },
            false,
            "",
        ),
        (
            "a corner out of the plane of the arc beside it",
            |topo| {
                let profile = disc(topo, 1.0);
                // An arc in the plane x = 10 (about +X), leaving the corner
                // along +Z; the line arrives along +Y.
                let circle = Circle3D::with_axes(p(10.0, 0.0, 10.0), X, 10.0, -Z, Y).unwrap();
                let a = topo.add_vertex(Vertex::new(p(10.0, 0.0, 0.0), 1e-7));
                let b = topo.add_vertex(Vertex::new(circle.evaluate(FRAC_PI_2), 1e-7));
                let mut arc = Edge::new(a, b, EdgeCurve::Circle(circle));
                arc.set_trim(Some((0.0, FRAC_PI_2)));
                let arc = topo.add_edge(arc);
                let path = [line(topo, p(10.0, -20.0, 0.0), p(10.0, 0.0, 0.0)), arc];
                sweep_exact_with_options(topo, profile, &path, &round(2.0))
            },
            true,
            "out of the plane",
        ),
        (
            "a round corner on a sweep that keeps its orientation",
            |topo| {
                let profile = disc(topo, 1.0);
                let path = ell(topo);
                let options = ExactSweepOptions {
                    orientation: ExactSweepOrientation::KeepProfile,
                    ..round(3.0)
                };
                sweep_exact_with_options(topo, profile, &path, &options)
            },
            true,
            "turns with its path",
        ),
        (
            "a non-positive radius",
            |topo| {
                let profile = disc(topo, 1.0);
                let path = ell(topo);
                sweep_exact_with_options(topo, profile, &path, &round(0.0))
            },
            false,
            "finite positive radius",
        ),
        (
            "a round corner with a scale",
            |topo| {
                let profile = disc(topo, 1.0);
                let path = ell(topo);
                let options = ExactSweepOptions {
                    end_scale: 2.0,
                    ..round(3.0)
                };
                sweep_exact_with_options(topo, profile, &path, &options)
            },
            true,
            "turns with its path",
        ),
    ];
    for (label, build, is_unsupported, phrase) in cases {
        let mut topo = Topology::new();
        let error = build(&mut topo).expect_err(label);
        let reason = match &error {
            OperationsError::Unsupported { reason, .. } if is_unsupported => reason.clone(),
            OperationsError::InvalidInput { reason } if !is_unsupported => reason.clone(),
            other => panic!("{label}: wrong refusal {other:?}"),
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
        assert_eq!(topo.num_faces(), 1, "{label}: faces left behind");
    }
}
