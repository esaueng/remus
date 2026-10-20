//! Closed-form oracles for [`super::project_silhouettes`]: every silhouette is
//! checked against where the exact outline must lie in the view plane.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_4};

use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::silhouette::silhouettes_within;
use super::{ProjectedSilhouettes, project_silhouettes};
use crate::boolean::{BooleanOp, boolean};
use crate::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use crate::transform::transform_solid;

const DEFLECTION: f64 = 0.05;

/// Trim crossings are bisected to far below this; the bound is the trim
/// test's own boundary tolerance.
const ENDPOINT_TOL: f64 = 1e-6;

fn project(topo: &Topology, solid: SolidId, direction: Vec3, x_axis: Vec3) -> ProjectedSilhouettes {
    project_silhouettes(
        topo,
        solid,
        Point3::new(0.0, 0.0, 0.0),
        direction,
        x_axis,
        true,
        DEFLECTION,
    )
    .unwrap()
}

/// Each polyline's first and last point, ordered by `(x, y)`.
fn ends(polyline: &[Point2]) -> (Point2, Point2) {
    let (a, b) = (polyline[0], polyline[polyline.len() - 1]);
    if (a.x(), a.y()) <= (b.x(), b.y()) {
        (a, b)
    } else {
        (b, a)
    }
}

fn assert_near(actual: Point2, expected: (f64, f64), tol: f64) {
    assert!(
        (actual.x() - expected.0).abs() < tol && (actual.y() - expected.1).abs() < tol,
        "expected {expected:?}, got ({}, {})",
        actual.x(),
        actual.y()
    );
}

/// Every polyline's points, sorted by projected `x`.
fn sorted_by_x(polylines: &[Vec<Point2>]) -> Vec<Vec<Point2>> {
    let mut sorted = polylines.to_vec();
    sorted.sort_by(|a, b| ends(a).0.x().total_cmp(&ends(b).0.x()));
    sorted
}

/// A side view of a cylinder off its seam: one visible line down each side,
/// running rim to rim.
#[test]
fn cylinder_side_view_is_two_rulings() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    // Looking along (1, 1, 0)/√2: the outline rulings sit at ±5 along the
    // in-plane x axis (−1, 1, 0)/√2, at the cylinder points ±5·(−1, 1, 0)/√2.
    let out = project(
        &topo,
        cylinder,
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
    );
    assert!(out.unsupported.is_empty());
    assert!(out.hidden.is_empty(), "{:?}", out.hidden);
    let lines = sorted_by_x(&out.visible);
    assert_eq!(lines.len(), 2, "{lines:?}");
    // In-plane y is x × dir = (−1, 1, 0)/√2 × (1, 1, 0)/√2 = (0, 0, −1).
    for (line, x) in lines.iter().zip([-5.0, 5.0]) {
        let (lo, hi) = ends(line);
        assert_near(lo, (x, -10.0), ENDPOINT_TOL);
        assert_near(hi, (x, 0.0), ENDPOINT_TOL);
        assert!(line.iter().all(|p| (p.x() - x).abs() < 1e-9));
    }
}

/// The same outline when a ruling falls on the lateral face's seam: the
/// seam is a trim of the face, and the silhouette still runs rim to rim.
#[test]
fn cylinder_outline_on_the_seam_is_whole() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    for direction in [Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)] {
        let x_axis = Vec3::new(0.0, 0.0, 1.0).cross(direction);
        let out = project(&topo, cylinder, direction, x_axis);
        let lines = sorted_by_x(&out.visible);
        assert_eq!(lines.len(), 2, "{direction:?}: {lines:?}");
        for line in &lines {
            let (lo, hi) = ends(line);
            assert!((hi.y() - lo.y()).abs() > 10.0 - ENDPOINT_TOL, "{line:?}");
        }
    }
}

/// Looking down the axis there is no ruling silhouette: the rims are the
/// outline, and they are edges.
#[test]
fn axial_view_has_no_cylinder_silhouette() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let out = project(
        &topo,
        cylinder,
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.visible.is_empty() && out.hidden.is_empty());
}

/// A sphere's outline from any direction is a circle of its radius about the
/// projected centre, all of it visible.
#[test]
fn sphere_outline_is_a_full_visible_circle() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 4.0, 8).unwrap();
    for (direction, x_axis) in [
        (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        (Vec3::new(1.0, 2.0, -0.5), Vec3::new(0.0, 0.0, 1.0)),
    ] {
        let out = project(&topo, sphere, direction, x_axis);
        assert!(out.hidden.is_empty(), "{direction:?}: {:?}", out.hidden);
        assert!(!out.visible.is_empty());
        // The runs cover the circle once, and each chord stays within the
        // deflection of the arc it cuts.
        let mut turned = 0.0;
        for line in &out.visible {
            for w in line.windows(2) {
                assert!((w[0].x().hypot(w[0].y()) - 4.0).abs() < 1e-9, "{w:?}");
                let half_chord = 0.5 * (w[1].x() - w[0].x()).hypot(w[1].y() - w[0].y());
                assert!(4.0 - (16.0 - half_chord * half_chord).sqrt() <= DEFLECTION);
                turned += (w[0].x() * w[1].y() - w[0].y() * w[1].x())
                    .atan2(w[0].x() * w[1].x() + w[0].y() * w[1].y())
                    .abs();
            }
        }
        assert!(
            (turned - std::f64::consts::TAU).abs() < 1e-9,
            "{direction:?}: turned {turned}"
        );
    }
}

/// A frustum from the side: two rulings joining the rims' extreme points,
/// (±4, 0) at the base to (±2, 6) at the top.
#[test]
fn cone_side_view_rulings_join_the_rims() {
    let mut topo = Topology::new();
    let cone = make_cone(&mut topo, 4.0, 2.0, 6.0).unwrap();
    let out = project(
        &topo,
        cone,
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
    );
    assert!(out.hidden.is_empty(), "{:?}", out.hidden);
    let lines = sorted_by_x(&out.visible);
    assert_eq!(lines.len(), 2, "{lines:?}");
    // In-plane y is −z, as for the cylinder.
    for (line, sign) in lines.iter().zip([-1.0, 1.0]) {
        let (lo, hi) = ends(line);
        let (base, top) = if lo.y() > hi.y() { (lo, hi) } else { (hi, lo) };
        assert_near(base, (4.0 * sign, 0.0), ENDPOINT_TOL);
        assert_near(top, (2.0 * sign, -6.0), ENDPOINT_TOL);
    }
}

/// Viewed from above at 45°, a cone of half-angle atan(6/4) from its radial
/// plane is seen from outside its nappe, and its outline rulings are where
/// cos(u − φ) = (axis·d)·cot a / |d_radial| = (1/√2)·(4/6)/(1/√2) = 2/3.
#[test]
fn oblique_cone_rulings_follow_the_closed_form() {
    let mut topo = Topology::new();
    let cone = make_cone(&mut topo, 4.0, 0.0, 6.0).unwrap();
    // Down onto the apex, tilted toward +x: d = (1, 0, −1)/√2.
    let direction = Vec3::new(FRAC_1_SQRT_2, 0.0, -FRAC_1_SQRT_2);
    let out = project(&topo, cone, direction, Vec3::new(0.0, 1.0, 0.0));
    let rulings: Vec<_> = out.visible.iter().chain(&out.hidden).collect();
    assert_eq!(rulings.len(), 2, "{rulings:?}");
    // The cone's normal is radial·sin a − axis·cos a with axis −z for this
    // pointed-up cone's frame, or +z: either way the base points of the two
    // rulings are 4·(cos u, sin u, 0) with cos u = ±2/3 (the sign set by the
    // frame), so their projected x (along +y) are ±4·√(1 − 4/9).
    let half_chord = 4.0 * (1.0_f64 - 4.0 / 9.0).sqrt();
    let mut base_xs: Vec<f64> = rulings
        .iter()
        .map(|line| {
            let (a, b) = ends(line);
            if a.x().abs() > b.x().abs() {
                a.x()
            } else {
                b.x()
            }
        })
        .collect();
    base_xs.sort_by(f64::total_cmp);
    assert!(
        (base_xs[0] + half_chord).abs() < ENDPOINT_TOL,
        "{base_xs:?}"
    );
    assert!(
        (base_xs[1] - half_chord).abs() < ENDPOINT_TOL,
        "{base_xs:?}"
    );
    // Both rulings end at the apex, projected to x = 0.
    for line in &rulings {
        let (a, b) = ends(line);
        assert!(a.x().abs().min(b.x().abs()) < ENDPOINT_TOL, "{line:?}");
    }
}

/// The two rulings of a bore through a block lie inside the material, so a
/// side view draws them hidden, at the bore's walls.
fn bored_block(scale: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 20.0 * scale, 20.0 * scale, 10.0 * scale).unwrap();
    let bore = make_cylinder(&mut topo, 3.0 * scale, 30.0 * scale).unwrap();
    transform_solid(
        &mut topo,
        bore,
        &Mat4::translation(10.0 * scale, 10.0 * scale, -10.0 * scale),
    )
    .unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, block, bore).unwrap();
    (topo, solid)
}

fn assert_bore_hidden(scale: f64) {
    let (topo, solid) = bored_block(scale);
    // Along +y, in-plane x is +x and y is x × dir = +z: the bore's walls at
    // x = 7 and 13 (scaled), through the block's height.
    let out = project_silhouettes(
        &topo,
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        true,
        DEFLECTION * scale,
    )
    .unwrap();
    assert!(out.visible.is_empty(), "scale {scale}: {:?}", out.visible);
    let lines = sorted_by_x(&out.hidden);
    assert_eq!(lines.len(), 2, "scale {scale}: {lines:?}");
    let tol = ENDPOINT_TOL * scale.max(1.0);
    for (line, x) in lines.iter().zip([7.0, 13.0]) {
        let (lo, hi) = ends(line);
        assert_near(lo, (x * scale, 0.0), tol);
        assert_near(hi, (x * scale, 10.0 * scale), tol);
    }
}

/// From a 0.2 mm block to a 20 m one: the occlusion step scales with the
/// model and is capped at 1e-2, so at the large end it is far smaller than
/// the bore and the probe enters the material by only eps²/2r.
#[test]
fn bore_silhouettes_are_hidden_across_scales() {
    for scale in [0.01, 1.0, 1000.0] {
        assert_bore_hidden(scale);
    }
}

/// A boss on a plate seen from the side: its outline is in front of nothing
/// and visible, and ends on the plate's top face.
#[test]
fn boss_silhouettes_are_visible_and_end_on_the_plate() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 20.0, 20.0, 2.0).unwrap();
    let boss = make_cylinder(&mut topo, 3.0, 8.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(10.0, 10.0, 1.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Fuse, plate, boss).unwrap();
    let out = project(
        &topo,
        solid,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.hidden.is_empty(), "{:?}", out.hidden);
    let lines = sorted_by_x(&out.visible);
    assert_eq!(lines.len(), 2, "{lines:?}");
    for (line, x) in lines.iter().zip([7.0, 13.0]) {
        let (lo, hi) = ends(line);
        assert_near(lo, (x, 2.0), ENDPOINT_TOL);
        assert_near(hi, (x, 9.0), ENDPOINT_TOL);
    }
}

/// A box notch through one side of a cylinder interrupts the ruling on
/// that side: two runs, broken at the notch's floor and ceiling.
#[test]
fn notch_splits_a_ruling_at_its_trims() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let notch = make_box(&mut topo, 4.0, 20.0, 2.0).unwrap();
    transform_solid(&mut topo, notch, &Mat4::translation(3.0, -10.0, 4.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, cylinder, notch).unwrap();
    let out = project(
        &topo,
        solid,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    // In-plane y is x × dir = (0, 0, 1): heights read directly.
    let mut right: Vec<(f64, f64)> = out
        .visible
        .iter()
        .chain(&out.hidden)
        .filter(|line| line[0].x() > 0.0)
        .map(|line| {
            let (a, b) = ends(line);
            (a.y().min(b.y()), a.y().max(b.y()))
        })
        .collect();
    right.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(right.len(), 2, "{right:?}");
    for (actual, expected) in right.iter().zip([(0.0, 4.0), (6.0, 10.0)]) {
        assert!(
            (actual.0 - expected.0).abs() < ENDPOINT_TOL
                && (actual.1 - expected.1).abs() < ENDPOINT_TOL,
            "{right:?}"
        );
    }
}

/// A torus has no closed-form outline here: it is named, not approximated.
#[test]
fn torus_is_reported_unsupported() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 6.0, 1.5, 8).unwrap();
    let out = project(
        &topo,
        torus,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.visible.is_empty() && out.hidden.is_empty());
    assert!(!out.unsupported.is_empty());
    assert!(out.unsupported.iter().all(|u| u.surface == "torus"));
}

/// A box has no curved faces and so no silhouettes; a rotated box too.
#[test]
fn planar_solid_has_no_silhouettes() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 3.0, 4.0, 5.0).unwrap();
    transform_solid(&mut topo, block, &Mat4::rotation_z(FRAC_PI_4)).unwrap();
    let out = project(
        &topo,
        block,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.visible.is_empty() && out.hidden.is_empty() && out.unsupported.is_empty());
}

#[test]
fn rejects_bad_input_and_over_budget_sampling() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 4.0, 8).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let (d, x) = (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    for deflection in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(project_silhouettes(&topo, sphere, origin, d, x, true, deflection).is_err());
    }
    assert!(
        project_silhouettes(
            &topo,
            sphere,
            origin,
            Vec3::new(0.0, 0.0, 0.0),
            x,
            true,
            0.1
        )
        .is_err()
    );
    assert!(project_silhouettes(&topo, sphere, origin, d, d, true, 0.1).is_err());

    // One sample per deflection of length: two 10-long rulings at 1e-4 are
    // 200 000 trim tests, over the 100 000 budget.
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let err = project_silhouettes(&topo, cylinder, origin, d, x, true, 1e-4).unwrap_err();
    assert!(err.to_string().contains("increase deflection"), "{err}");
}

/// Hidden lines off: hidden silhouettes are dropped like hidden edges.
#[test]
fn hidden_lines_off_drops_hidden_silhouettes() {
    let (topo, solid) = bored_block(1.0);
    let out = project_silhouettes(
        &topo,
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        false,
        DEFLECTION,
    )
    .unwrap();
    assert!(out.visible.is_empty() && out.hidden.is_empty());
}

/// A view along a cone generator: the one tangent ruling runs along the view
/// and projects to a point, so there is no outline, not the same ruling twice.
/// A 45° cone seen at 45° to its axis is exactly this case.
#[test]
fn view_along_a_cone_generator_has_no_ruling() {
    let mut topo = Topology::new();
    // Base radius = height: half-angle 45° from the radial plane.
    let cone = make_cone(&mut topo, 5.0, 0.0, 5.0).unwrap();
    // Every one of these azimuths, from above and from below, rounds |c| to
    // just inside 1, where an exact `|c| > 1` test let the doubled ruling
    // through.
    for (step, dz) in [38, 39, 41, 45, 47, 59, 60]
        .into_iter()
        .flat_map(|step| [(step, -1.0), (step, 1.0)])
    {
        let azimuth = f64::from(step) * std::f64::consts::TAU / 72.0;
        let direction = Vec3::new(azimuth.cos(), azimuth.sin(), dz);
        let x_axis = Vec3::new(0.0, 0.0, 1.0).cross(direction);
        let out = project(&topo, cone, direction, x_axis);
        assert!(
            out.visible.is_empty() && out.hidden.is_empty(),
            "{direction:?}: {:?} {:?}",
            out.visible,
            out.hidden
        );
    }
}

/// Placing each trim crossing costs bisection queries the sample preflight
/// cannot count, so they draw on their own budget: the notched cylinder's
/// six crossings (two rims on each ruling, the notch's floor and ceiling)
/// need 240 and are refused one short.
#[test]
fn trim_refinement_has_its_own_budget() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    let notch = make_box(&mut topo, 4.0, 20.0, 2.0).unwrap();
    transform_solid(&mut topo, notch, &Mat4::translation(3.0, -10.0, 4.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, cylinder, notch).unwrap();
    let within = |queries| {
        silhouettes_within(
            &topo,
            solid,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            true,
            DEFLECTION,
            queries,
        )
    };
    let out = within(6 * 40).unwrap();
    assert_eq!(out.visible.len() + out.hidden.len(), 3);
    let err = within(6 * 40 - 1).unwrap_err();
    assert!(err.to_string().contains("work limit"), "{err}");
}
