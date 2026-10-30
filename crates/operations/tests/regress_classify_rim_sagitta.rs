//! RFC 0008 F1: a ray that crosses a circular planar cap between the rim and
//! its chords must still count the cap.
//!
//! The classifier trimmed planar caps by a 32-chord polygon, whose inradius
//! is `r·cos(π/32)`, so a ray through the band between chords and rim missed
//! the cap while still crossing the exact cylinder wall. Points just outside
//! a cap classified `Inside`, and points just inside it `Outside`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::vec::Point3;
use remus_operations::primitives::make_cylinder;
use remus_topology::Topology;

const RADIUS: f64 = 3.0;
const HEIGHT: f64 = 8.0;

/// Closed form for `make_cylinder(RADIUS, HEIGHT)`: axis +Z from z = 0.
fn expected(p: Point3) -> PointClassification {
    let inside = p.x().hypot(p.y()) < RADIUS && p.z() > 0.0 && p.z() < HEIGHT;
    if inside {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    }
}

fn assert_classifies(topo: &Topology, solid: remus_topology::solid::SolidId, points: &[Point3]) {
    let prepared = PreparedSolid::prepare(topo, solid).unwrap();
    let options = ClassifyOptions::default();
    for &p in points {
        let want = expected(p);
        assert_eq!(
            prepared.classify_point(p, &options).unwrap(),
            want,
            "prepared at {p:?}"
        );
        assert_eq!(
            classify_point(topo, solid, p, &options).unwrap(),
            want,
            "one-shot at {p:?}"
        );
    }
}

#[test]
fn rfc_0008_f1_repro_points() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, RADIUS, HEIGHT).unwrap();
    assert_classifies(
        &topo,
        solid,
        &[
            // Below the bottom cap, inside the rim's sagitta band.
            Point3::new(2.395_539, -1.804_909, -0.011_615),
            // Inside, just under the top cap, in the band.
            Point3::new(-1.809_4, -2.391_1, 7.988_4),
        ],
    );
}

/// Every angle around both rims, in the band between the 32-chord inradius
/// `3·cos(π/32) ≈ 2.9856` and the rim, just inside and just outside each cap.
#[test]
fn points_in_the_rim_sagitta_band_classify_by_closed_form() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, RADIUS, HEIGHT).unwrap();
    let mut points = Vec::new();
    for k in 0..64 {
        let angle = f64::from(k) * std::f64::consts::TAU / 64.0 + 0.013;
        for r in [2.99, 2.998] {
            for z in [-0.01, 0.01, HEIGHT - 0.01, HEIGHT + 0.01] {
                points.push(Point3::new(r * angle.cos(), r * angle.sin(), z));
            }
        }
    }
    assert_classifies(&topo, solid, &points);
}
