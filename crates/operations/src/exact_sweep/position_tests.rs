//! Profile position: where on the path the profile stands. The profile is
//! never moved; the path's motions run from the anchor to both ends.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_4, PI};

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_line_edge};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
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

fn at(position: ExactSweepPosition) -> ExactSweepOptions {
    ExactSweepOptions {
        position: Some(position),
        ..ExactSweepOptions::default()
    }
}

/// A radius-1 disc about `centre`, square to `normal`.
fn disc(topo: &mut Topology, centre: Point3, normal: Vec3) -> FaceId {
    let edge = make_circle_edge(topo, centre, normal, 1.0, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: crate::dot_normal_point(normal, centre),
        },
    ))
}

fn line(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
    make_line_edge(topo, a, b, 1e-7).unwrap()
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

fn assert_solid(topo: &Topology, solid: SolidId, exact: f64, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
    let mesh = crate::tessellate::tessellate_solid(topo, solid, 0.02).unwrap();
    assert!(!mesh.indices.is_empty(), "{label}: empty mesh");
    let measured = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
    assert!(
        rel(measured, exact) < 1e-9,
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

#[test]
fn from_the_path_intersection_the_sweep_runs_both_ways() {
    for position in [
        ExactSweepPosition::Auto,
        ExactSweepPosition::PathIntersection,
    ] {
        let mut topo = Topology::new();
        let profile = disc(&mut topo, p(0.0, 0.0, 0.0), X);
        let path = line(&mut topo, p(-10.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
        let solid = sweep_exact_with_options(&mut topo, profile, &[path], &at(position)).unwrap();
        assert_solid(&topo, solid, PI * 20.0, "both ways");
        assert_inside(&topo, solid, p(-9.0, 0.0, 0.0), true, "behind");
        assert_inside(&topo, solid, p(9.0, 0.0, 0.0), true, "ahead");
        assert_inside(&topo, solid, p(11.0, 0.0, 0.0), false, "past the end");
    }
}

#[test]
fn from_the_closest_endpoint_the_profile_travels_the_whole_path() {
    let mut topo = Topology::new();
    let profile = disc(&mut topo, p(0.0, 0.0, 0.0), X);
    // The end at x = 4 is nearer the profile than the one at x = −10.
    let path = line(&mut topo, p(-10.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let solid = sweep_exact_with_options(
        &mut topo,
        profile,
        &[path],
        &at(ExactSweepPosition::ClosestEndpoint),
    )
    .unwrap();
    // Carried from x = 4 back to x = −10: the profile travels 14 along −X.
    assert_solid(&topo, solid, PI * 14.0, "endpoint");
    assert_inside(&topo, solid, p(-13.0, 0.0, 0.0), true, "far end");
    assert_inside(
        &topo,
        solid,
        p(1.0, 0.0, 0.0),
        false,
        "ahead of the profile",
    );
}

#[test]
fn from_the_closest_point_a_path_beside_the_profile_runs_both_ways() {
    let mut topo = Topology::new();
    let profile = disc(&mut topo, p(0.0, 0.0, 0.0), X);
    // The path runs 5 beside the profile: its nearest point is (0, 5, 0).
    let path = line(&mut topo, p(-6.0, 5.0, 0.0), p(10.0, 5.0, 0.0));
    let solid = sweep_exact_with_options(
        &mut topo,
        profile,
        &[path],
        &at(ExactSweepPosition::ClosestPoint),
    )
    .unwrap();
    assert_solid(&topo, solid, PI * 16.0, "closest point");
    assert_inside(&topo, solid, p(-5.5, 0.0, 0.0), true, "behind");
    assert_inside(&topo, solid, p(9.5, 0.0, 0.0), true, "ahead");
}

#[test]
fn an_arc_crossing_the_profile_sweeps_both_ways_round_it() {
    let mut topo = Topology::new();
    // The profile square to the arc where it crosses y = 0.
    let profile = disc(&mut topo, p(10.0, 0.0, 0.0), Y);
    let circle = Circle3D::with_axes(p(0.0, 0.0, 0.0), Z, 10.0, X, Y).unwrap();
    let a = topo.add_vertex(Vertex::new(circle.evaluate(-FRAC_PI_4), 1e-7));
    let b = topo.add_vertex(Vertex::new(circle.evaluate(FRAC_PI_4), 1e-7));
    let mut arc = Edge::new(a, b, EdgeCurve::Circle(circle));
    arc.set_trim(Some((-FRAC_PI_4, FRAC_PI_4)));
    let arc = topo.add_edge(arc);
    let solid = sweep_exact_with_options(&mut topo, profile, &[arc], &at(ExactSweepPosition::Auto))
        .unwrap();
    // Pappus: π r² · 10 · π/2.
    assert_solid(&topo, solid, PI * 10.0 * (PI / 2.0), "arc both ways");
}

#[test]
fn position_refusals_are_typed_and_leave_the_topology_unchanged() {
    type Build = fn(&mut Topology) -> Result<SolidId, OperationsError>;
    let cases: [(&str, Build, bool, &str); 4] = [
        (
            "a path intersection the path never reaches",
            |topo| {
                let profile = disc(topo, p(0.0, 0.0, 0.0), X);
                let path = line(topo, p(2.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
                sweep_exact_with_options(
                    topo,
                    profile,
                    &[path],
                    &at(ExactSweepPosition::PathIntersection),
                )
            },
            false,
            "does not cross the profile's plane",
        ),
        (
            "a position on a sweep that keeps its orientation",
            |topo| {
                let profile = disc(topo, p(0.0, 0.0, 0.0), X);
                let path = line(topo, p(-10.0, 0.0, 0.0), p(10.0, 0.0, 0.0));
                let options = ExactSweepOptions {
                    orientation: ExactSweepOrientation::KeepProfile,
                    ..at(ExactSweepPosition::Auto)
                };
                sweep_exact_with_options(topo, profile, &[path], &options)
            },
            true,
            "turns with its path",
        ),
        (
            "a position on a closed path",
            |topo| {
                let profile = disc(topo, p(0.0, 0.0, 0.0), Y);
                let corners = [
                    p(0.0, -10.0, 0.0),
                    p(0.0, 10.0, 0.0),
                    p(-20.0, 10.0, 0.0),
                    p(-20.0, -10.0, 0.0),
                ];
                let path: Vec<EdgeId> = (0..4)
                    .map(|k| line(topo, corners[k], corners[(k + 1) % 4]))
                    .collect();
                sweep_exact_with_options(topo, profile, &path, &at(ExactSweepPosition::Auto))
            },
            true,
            "closed path",
        ),
        (
            "a profile standing a hair past a mitred corner",
            |topo| {
                let profile = disc(topo, p(0.0, 5e-7, 0.0), Y);
                let path = [
                    line(topo, p(-10.0, 0.0, 0.0), p(0.0, 0.0, 0.0)),
                    line(topo, p(0.0, 0.0, 0.0), p(0.0, 10.0, 0.0)),
                ];
                sweep_exact_with_options(
                    topo,
                    profile,
                    &path,
                    &at(ExactSweepPosition::PathIntersection),
                )
            },
            true,
            "stands on the path's corner",
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

/// Review regression: far from the origin an arc's end in the profile's
/// plane is still a path intersection.
#[test]
fn an_arc_ending_in_the_profile_plane_is_an_intersection() {
    let mut topo = Topology::new();
    let centre = p(1_000_000.0, 0.0, 0.0);
    let circle = Circle3D::with_axes(centre, Z, 10.0, X, Y).unwrap();
    let end = circle.evaluate(0.3);
    let a = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let b = topo.add_vertex(Vertex::new(end, 1e-7));
    let mut arc = Edge::new(a, b, EdgeCurve::Circle(circle));
    arc.set_trim(Some((0.0, 0.3)));
    let arc = topo.add_edge(arc);
    // A small disc at the arc's end, square to its tangent there.
    let tangent = Vec3::new(-(0.3_f64).sin(), 0.3_f64.cos(), 0.0);
    let profile = {
        let edge = make_circle_edge(&mut topo, end, tangent, 0.5, 1e-7).unwrap();
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: tangent,
                d: crate::dot_normal_point(tangent, end),
            },
        ))
    };
    let solid = sweep_exact_with_options(
        &mut topo,
        profile,
        &[arc],
        &at(ExactSweepPosition::PathIntersection),
    )
    .unwrap();
    let measured = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
    let exact = PI * 0.25 * 10.0 * 0.3;
    assert!(rel(measured, exact) < 1e-6, "{measured} vs {exact}");
}
