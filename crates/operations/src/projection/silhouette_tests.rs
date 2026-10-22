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
    // Along the pole axis the outline is the equator both hemisphere faces
    // own: it is drawn once, not once per face.
    for (direction, x_axis) in [
        (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        (Vec3::new(1.0, 2.0, -0.5), Vec3::new(0.0, 0.0, 1.0)),
        (Vec3::new(0.0, 0.0, -1.0), Vec3::new(1.0, 0.0, 0.0)),
        (Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 0.0)),
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

/// Distance of `p` from the tube centre circle of the torus `(c, z, R)`
/// and the unit vector from that circle to `p`: the tube's outward normal.
fn torus_tube(p: Point3, center: Point3, z: Vec3, major: f64) -> (f64, Vec3) {
    let rel = p - center;
    let in_plane = rel - z * rel.dot(z);
    let ring = center + in_plane.normalize().unwrap() * major;
    let off = p - ring;
    (off.length(), off.normalize().unwrap())
}

/// Every sampled point of both torus branches, in views from straight down
/// the axis to just off the equatorial plane, lies on the torus with its
/// normal perpendicular to the view, and neighbouring samples stay within
/// the deflection of the curve and a tube chord of each other.
#[test]
fn torus_branches_satisfy_the_silhouette_condition() {
    let surface = remus_math::surfaces::ToroidalSurface::with_axis(
        Point3::new(1.0, -2.0, 3.0),
        6.0,
        1.5,
        Vec3::new(0.2, -0.3, 1.0),
    )
    .unwrap();
    let z = surface.z_axis();
    let face = remus_topology::face::FaceSurface::Torus(surface.clone());
    let topo = Topology::new();
    for direction in [
        -z,
        Vec3::new(1.0, 2.0, -3.0),
        Vec3::new(0.3, -1.0, 0.7),
        // 1e-6 off the equatorial plane: each branch turns through a
        // meridian in a u-interval about 3e-6 wide.
        (surface.x_axis() + z * 1e-6).normalize().unwrap(),
    ] {
        let d = direction.normalize().unwrap();
        let curves = super::silhouette::carrier_silhouettes(
            &topo,
            &face,
            &[],
            d,
            DEFLECTION,
            &mut { usize::MAX },
            usize::MAX,
        )
        .unwrap();
        assert_eq!(curves.len(), 2, "{d:?}");
        let max_chord = 2.0 * (DEFLECTION * (3.0 - DEFLECTION)).sqrt();
        for curve in &curves {
            let ts = curve.sample_ts(DEFLECTION, 100_000).unwrap();
            for pair in ts.windows(2) {
                let (a, b) = (curve.at(pair[0]), curve.at(pair[1]));
                assert!(
                    (b - a).length() <= max_chord + 1e-12,
                    "{d:?}: gap at {pair:?}"
                );
            }
            for &t in &ts {
                let p = curve.at(t);
                let (distance, normal) = torus_tube(p, surface.center(), z, 6.0);
                assert!(
                    (distance - 1.5).abs() < 1e-9,
                    "{d:?} t={t}: off the torus by {}",
                    distance - 1.5
                );
                assert!(
                    normal.dot(d).abs() < 1e-9,
                    "{d:?} t={t}: n·d = {}",
                    normal.dot(d)
                );
            }
        }
    }
}

/// Seen down its axis, a torus's outline is its outer and inner equators,
/// circles of radius R + r and R − r, all visible.
#[test]
fn torus_axial_view_is_its_equators() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 6.0, 1.5, 8).unwrap();
    let out = project(
        &topo,
        torus,
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.unsupported.is_empty());
    assert!(out.hidden.is_empty(), "{:?}", out.hidden);
    let mut turned = [0.0_f64; 2];
    for line in &out.visible {
        for w in line.windows(2) {
            let radius = w[0].x().hypot(w[0].y());
            let which = if (radius - 7.5).abs() < 1e-9 {
                0
            } else {
                assert!((radius - 4.5).abs() < 1e-9, "radius {radius}");
                1
            };
            turned[which] += (w[0].x() * w[1].y() - w[0].y() * w[1].x())
                .atan2(w[0].x() * w[1].x() + w[0].y() * w[1].y())
                .abs();
        }
    }
    for angle in turned {
        assert!((angle - std::f64::consts::TAU).abs() < 1e-9, "{turned:?}");
    }
}

/// A deflection past the tube diameter puts no tube-chord bound on a thin
/// ring: its equators are sampled at their own chord spacing, about 140
/// intervals each, instead of 2-long chords that overrun the sample limit.
#[test]
fn thin_torus_coarse_deflection_is_sampled() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 20_000.0, 1.0, 8).unwrap();
    let out = project_silhouettes(
        &topo,
        torus,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
        true,
        5.0,
    )
    .unwrap();
    assert!(out.unsupported.is_empty());
    let points: usize = out.visible.iter().map(Vec::len).sum();
    assert!(points < 1_000, "{points} points");
    let mut turned = [0.0_f64; 2];
    for line in &out.visible {
        for w in line.windows(2) {
            let radius = w[0].x().hypot(w[0].y());
            let which = usize::from((radius - 19_999.0).abs() < 1e-6);
            assert!(
                which == 1 || (radius - 20_001.0).abs() < 1e-6,
                "radius {radius}"
            );
            turned[which] += (w[0].x() * w[1].y() - w[0].y() * w[1].x())
                .atan2(w[0].x() * w[1].x() + w[0].y() * w[1].y())
                .abs();
        }
    }
    for angle in turned {
        assert!((angle - std::f64::consts::TAU).abs() < 1e-9, "{turned:?}");
    }
}

/// Seen in its equatorial plane, a torus's outline is four circles: the top
/// and bottom circles, edge-on as lines at height ±r, and the two meridians
/// at the sides. A meridian's outer half is in front of nothing; its inner
/// half lies behind the near side of the ring.
#[test]
fn torus_side_view_is_lines_and_meridians() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 6.0, 1.5, 8).unwrap();
    // Along +y: in-plane x is +x and y is +z.
    let out = project(
        &topo,
        torus,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    let mut meridian_points = 0;
    for (lines, hidden) in [(&out.visible, false), (&out.hidden, true)] {
        for line in lines {
            for p in line {
                let on_line = (p.y().abs() - 1.5).abs() < 1e-9 && p.x().abs() <= 6.0 + 1e-9;
                let on_meridian = ((p.x().abs() - 6.0).hypot(p.y()) - 1.5).abs() < 1e-9;
                assert!(on_line || on_meridian, "{p:?}");
                // Only meridian points strictly off the top and bottom
                // lines carry a visibility the oracle decides.
                if on_meridian && !on_line && (p.x().abs() - 6.0).abs() > 0.1 {
                    meridian_points += 1;
                    assert_eq!(hidden, p.x().abs() < 6.0, "{p:?} hidden = {hidden}");
                }
            }
        }
    }
    assert!(meridian_points > 20, "{meridian_points}");
}

/// A box notch through the +x side of a torus (R = 10, r = 3; the box
/// [6, 14] × [−4, 4] × [−4, 4]) seen from above cuts both equators: the
/// outer one (radius 13) where it leaves the box at y = ±4, the inner one
/// (radius 7) where it meets the box face x = 6, at y = ±√13.
#[test]
fn notched_torus_equators_end_on_the_notch() {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 10.0, 3.0, 32).unwrap();
    let notch = make_box(&mut topo, 8.0, 8.0, 8.0).unwrap();
    transform_solid(&mut topo, notch, &Mat4::translation(6.0, -4.0, -4.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, torus, notch).unwrap();
    // Down −z with in-plane x = +x: in-plane y is +y.
    let out = project(
        &topo,
        solid,
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.unsupported.is_empty());
    let lines: Vec<_> = out.visible.iter().chain(&out.hidden).collect();
    let inner_gap = 13.0_f64.sqrt();
    let mut turned = [0.0_f64; 2];
    for line in &lines {
        for p in *line {
            let radius = p.x().hypot(p.y());
            let outer = (radius - 13.0).abs() < 1e-9;
            assert!(outer || (radius - 7.0).abs() < 1e-9, "{p:?}");
            let gap = if outer { 4.0 } else { inner_gap };
            assert!(
                p.x() < 0.0 || p.y().abs() > gap - ENDPOINT_TOL,
                "{p:?} is in the notch"
            );
        }
        for w in line.windows(2) {
            let which = usize::from(w[0].x().hypot(w[0].y()) < 10.0);
            turned[which] += (w[0].x() * w[1].y() - w[0].y() * w[1].x())
                .atan2(w[0].x() * w[1].x() + w[0].y() * w[1].y())
                .abs();
        }
    }
    // The run ends sit on the notch edges, plane ∩ torus curves the trim
    // test samples into a polygon: they are as exact as that sampling.
    let ends: Vec<Point2> = lines
        .iter()
        .flat_map(|line| [line[0], line[line.len() - 1]])
        .collect();
    let outer_x = (13.0_f64 * 13.0 - 16.0).sqrt();
    let expected = [
        (outer_x, 4.0),
        (outer_x, -4.0),
        (6.0, inner_gap),
        (6.0, -inner_gap),
    ];
    let mut worst = 0.0_f64;
    for (x, y) in expected {
        let error = ends
            .iter()
            .map(|p| (p.x() - x).hypot(p.y() - y))
            .fold(f64::INFINITY, f64::min);
        worst = worst.max(error);
    }
    // 1.04e-4 at this fixture's trim sampling, far inside the deflection.
    assert!(worst < 0.1 * DEFLECTION, "worst endpoint error {worst}");
    // Each equator is drawn once, whole but for its notch.
    let tau = std::f64::consts::TAU;
    let expected = [
        tau - 2.0 * 4.0_f64.atan2(outer_x),
        tau - 2.0 * inner_gap.atan2(6.0),
    ];
    for (angle, expected) in turned.iter().zip(expected) {
        assert!((angle - expected).abs() < 1e-4, "{turned:?} vs {expected}");
    }
}

/// A cylinder converted to NURBS has the exact cylinder's outline: the
/// traced contour's vertices are on the rulings at x = ±5 and run rim to
/// rim, and the face is reported as traced.
#[test]
fn nurbs_cylinder_traces_the_analytic_rulings() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    crate::heal::convert_to_bspline(&mut topo, cylinder).unwrap();
    let out = project(
        &topo,
        cylinder,
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
    );
    assert!(out.unsupported.is_empty());
    assert!(!out.approximate.is_empty());
    assert!(out.hidden.is_empty(), "{:?}", out.hidden);
    let mut extent = [(f64::INFINITY, f64::NEG_INFINITY); 2];
    for line in &out.visible {
        for p in line {
            let side = usize::from(p.x() > 0.0);
            assert!((p.x().abs() - 5.0).abs() < 1e-8, "{p:?} is off the ruling");
            extent[side] = (extent[side].0.min(p.y()), extent[side].1.max(p.y()));
        }
    }
    // In-plane y is −z: each ruling spans the rims at y = −10 and 0.
    for (lo, hi) in extent {
        assert!((lo + 10.0).abs() < 1e-4 && hi.abs() < 1e-4, "{extent:?}");
    }
}

/// The exact rational sphere's traced outline is the great circle: every
/// vertex at the radius, its normal perpendicular to the view.
/// (`convert_to_bspline` makes spheres and tori faceted degree-1 surfaces,
/// whose silhouettes are their facet edges, so this traces the exact
/// rational form instead.)
#[test]
fn traced_nurbs_sphere_vertices_are_on_its_great_circle() {
    let sphere =
        remus_math::surfaces::SphericalSurface::new(Point3::new(1.0, -2.0, 0.5), 4.0).unwrap();
    let nurbs = remus_heal::construct::convert_surface::sphere_to_nurbs(&sphere).unwrap();
    for direction in [Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 2.0, -0.5)] {
        let d = direction.normalize().unwrap();
        let contours = super::silhouette::nurbs::contours(
            &nurbs,
            d,
            DEFLECTION,
            &mut { usize::MAX },
            usize::MAX,
        )
        .unwrap();
        assert!(!contours.is_empty(), "{d:?}");
        for contour in &contours {
            for &(u, v) in &contour.uv {
                let offset = nurbs.evaluate(u, v) - sphere.center();
                assert!((offset.length() - 4.0).abs() < 1e-8, "{d:?}: {offset:?}");
                assert!(
                    offset.dot(d).abs() < 1e-7,
                    "{d:?}: n·d = {}",
                    offset.dot(d) / 4.0
                );
            }
        }
    }
}

/// The contours traced on the exact rational torus lie on the torus with
/// its exact normal perpendicular to the view, and their chords stay
/// within the deflection of it.
#[test]
fn traced_torus_contours_satisfy_the_silhouette_condition() {
    let torus = remus_math::surfaces::ToroidalSurface::with_axis(
        Point3::new(1.0, -2.0, 3.0),
        6.0,
        1.5,
        Vec3::new(0.2, -0.3, 1.0),
    )
    .unwrap();
    // The exact rational torus: `ToroidalSurface::to_nurbs` is a faceted
    // degree-1 approximation, whose normal is constant on each facet.
    let nurbs = remus_heal::construct::convert_surface::torus_to_nurbs(&torus).unwrap();
    let z = torus.z_axis();
    for direction in [Vec3::new(1.0, 2.0, -3.0), Vec3::new(0.3, -1.0, 0.7), -z] {
        let d = direction.normalize().unwrap();
        let contours = super::silhouette::nurbs::contours(
            &nurbs,
            d,
            DEFLECTION,
            &mut { usize::MAX },
            usize::MAX,
        )
        .unwrap();
        assert!(!contours.is_empty(), "{d:?}");
        for contour in &contours {
            for &(u, v) in &contour.uv {
                let p = nurbs.evaluate(u, v);
                let (distance, normal) = torus_tube(p, torus.center(), z, 6.0);
                assert!(
                    (distance - 1.5).abs() < 1e-8,
                    "{d:?}: off the torus by {}",
                    distance - 1.5
                );
                assert!(normal.dot(d).abs() < 1e-8, "{d:?}: n·d = {}", normal.dot(d));
            }
            // Refinement keeps each chord within the deflection of the
            // silhouette, so its middle is within it of the torus too.
            let n = contour.uv.len();
            let segments = if contour.closed { n } else { n - 1 };
            for k in 0..segments {
                let (a, b) = (contour.uv[k], contour.uv[(k + 1) % n]);
                let (pa, pb) = (nurbs.evaluate(a.0, a.1), nurbs.evaluate(b.0, b.1));
                let (distance, _) = torus_tube(pa + (pb - pa) * 0.5, torus.center(), z, 6.0);
                assert!(
                    (distance - 1.5).abs() <= DEFLECTION,
                    "{d:?}: chord {k} sags {}",
                    distance - 1.5
                );
            }
        }
    }
}

/// A box converted to NURBS is all flat faces: each is either seen edge-on
/// (n·d = 0 everywhere, outline already its edges) or face-on (no sign
/// change), so there is no silhouette and nothing approximate.
#[test]
fn nurbs_planar_faces_have_no_silhouette() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 3.0, 4.0, 5.0).unwrap();
    crate::heal::convert_to_bspline(&mut topo, block).unwrap();
    for direction in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0)] {
        let out = project(&topo, block, direction, Vec3::new(0.0, 0.0, 1.0));
        assert!(
            out.visible.is_empty() && out.hidden.is_empty(),
            "{direction:?}"
        );
        assert!(out.approximate.is_empty(), "{direction:?}");
    }
}

/// Faces split from one NURBS face carry clones of its surface: they are
/// one carrier, traced once and trimmed together, as split analytic faces
/// are. A different NURBS surface is not the same carrier.
#[test]
fn split_nurbs_faces_share_one_carrier() {
    use remus_topology::face::FaceSurface;
    let sphere = |r: f64| {
        let s = remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), r).unwrap();
        remus_heal::construct::convert_surface::sphere_to_nurbs(&s).unwrap()
    };
    let surface = sphere(4.0);
    let (a, b) = (
        FaceSurface::Nurbs(surface.clone()),
        FaceSurface::Nurbs(surface),
    );
    assert!(super::silhouette::same_carrier(&a, &b));
    assert!(!super::silhouette::same_carrier(
        &a,
        &FaceSurface::Nurbs(sphere(5.0))
    ));
}

/// A trace whose grid holds more sign changes than the vertex budget is
/// refused when they are counted, before any of them is bisected.
#[test]
fn nurbs_tracing_refuses_more_crossings_than_vertices() {
    let sphere =
        remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0).unwrap();
    let nurbs = remus_heal::construct::convert_surface::sphere_to_nurbs(&sphere).unwrap();
    let view = Vec3::new(0.0, 1.0, 0.0);
    let traced =
        super::silhouette::nurbs::contours(&nurbs, view, 0.05, &mut { usize::MAX }, usize::MAX)
            .unwrap();
    assert!(traced.iter().map(|c| c.uv.len()).sum::<usize>() > 4);
    assert!(
        super::silhouette::nurbs::contours(&nurbs, view, 0.05, &mut { usize::MAX }, 4).is_none()
    );
}

/// A closed traced contour is periodic like the circles: a run joined
/// across its seam carries parameters past 1, which must land on the
/// contour's start again rather than stick at its last point.
#[test]
fn closed_contour_parameters_wrap() {
    let sphere =
        remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0).unwrap();
    let nurbs = remus_heal::construct::convert_surface::sphere_to_nurbs(&sphere).unwrap();
    let ((u0, u1), (v0, v1)) = (nurbs.domain_u(), nurbs.domain_v());
    let at = |fu: f64, fv: f64| (u0 + (u1 - u0) * fu, v0 + (v1 - v0) * fv);
    let curve = super::silhouette::Curve::Contour {
        surface: &nurbs,
        uv: vec![at(0.3, 0.3), at(0.6, 0.3), at(0.6, 0.6), at(0.3, 0.6)],
        closed: true,
    };
    for t in [0.0, 0.1, 0.3, 0.55, 0.9] {
        let (p, q) = (curve.at(t), curve.at(t + 1.0));
        assert!((p - q).length() < 1e-9, "t = {t}: {p:?} vs {q:?}");
    }
}

/// A traced face is disclosed even when tracing finds no contour: a
/// NURBS cylinder seen down its axis is edge-on, so it draws nothing, but
/// only a certified flat face is exempt, and the caps are the only ones.
#[test]
fn nurbs_faces_with_no_contour_are_still_approximate() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    crate::heal::convert_to_bspline(&mut topo, cylinder).unwrap();
    let out = project(
        &topo,
        cylinder,
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.visible.is_empty() && out.hidden.is_empty());
    let faces = remus_topology::explorer::solid_faces(&topo, cylinder).unwrap();
    let curved: Vec<_> = faces
        .into_iter()
        .filter(|&face| {
            if let remus_topology::face::FaceSurface::Nurbs(n) = topo.face(face).unwrap().surface()
            {
                !super::silhouette::nurbs::is_certified_plane(n)
            } else {
                false
            }
        })
        .collect();
    assert!(!curved.is_empty());
    let mut approximate = out.approximate;
    approximate.sort_unstable_by_key(|face| face.index());
    let mut expected = curved;
    expected.sort_unstable_by_key(|face| face.index());
    assert_eq!(approximate, expected);
}

/// The bored block converted to NURBS still hides its bore's outline, at
/// the bore's walls.
#[test]
fn nurbs_bore_silhouettes_are_hidden() {
    let (mut topo, solid) = bored_block(1.0);
    crate::heal::convert_to_bspline(&mut topo, solid).unwrap();
    let out = project(
        &topo,
        solid,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert!(out.visible.is_empty(), "{:?}", out.visible);
    assert!(!out.hidden.is_empty());
    for line in &out.hidden {
        for p in line {
            let wall = (p.x() - 7.0).abs().min((p.x() - 13.0).abs());
            assert!(wall < 1e-8, "{p:?} is off the bore walls");
            assert!(p.y() > -1e-4 && p.y() < 10.0 + 1e-4, "{p:?}");
        }
    }
}

/// Tracing is refused, not truncated, past its grid budget or its vertex
/// budget.
#[test]
fn nurbs_tracing_refuses_past_its_budgets() {
    let nurbs = remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0)
        .unwrap()
        .to_nurbs()
        .unwrap();
    let d = Vec3::new(0.0, 1.0, 0.0);
    let traced = |budget: &mut usize, vertices| {
        super::silhouette::nurbs::contours(&nurbs, d, DEFLECTION, budget, vertices)
    };
    assert!(traced(&mut 10, usize::MAX).is_none());
    assert!(traced(&mut { usize::MAX }, 3).is_none());
    let mut budget = usize::MAX;
    assert!(traced(&mut budget, usize::MAX).is_some());
    assert!(budget < usize::MAX, "the grid was not charged");
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

/// Every trim sample is tested against each face of its surface, and the
/// budget counts each test: a sphere's equator, tested against both
/// hemispheres, is refused at a sampling one hemisphere alone would pass.
#[test]
fn sample_budget_counts_each_face_tested() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 4.0, 8).unwrap();
    // About 70 000 samples on the r = 4 circle at this chord deflection:
    // under the 100 000 budget once, over it for two faces.
    let deflection = 4.0e-9;
    let step = 2.0 * (1.0 - deflection / 4.0_f64).acos();
    let samples = (std::f64::consts::TAU / step).ceil();
    assert!((50_000.0..100_000.0).contains(&samples), "{samples}");
    let err = project_silhouettes(
        &topo,
        sphere,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        true,
        deflection,
    )
    .unwrap_err();
    assert!(err.to_string().contains("increase deflection"), "{err}");
}
