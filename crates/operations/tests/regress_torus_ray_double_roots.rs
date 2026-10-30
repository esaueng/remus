//! Regression (RFC 0008 torus double-root hazard): a classification ray
//! tangent to a torus meets it in a double root, which the quartic solver's
//! de-dup often merged into a single root, so the check classifier counted
//! one crossing. Points outside a torus whose base rays 0 and 1 both graze
//! the tube classified `Inside` — 198 of 199 such points at tube radius
//! 1e-3, 60 of 199 at 0.37, 43 at 1 and 1 at 13.
//!
//! Each point is built from the tangency condition in closed form (plus a
//! bisection for where the two tangent lines meet), and its truth comes
//! from the torus implicit, never from a classifier.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::primitives::make_torus;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

/// The check classifier's first two base ray directions (`classify/mod.rs`).
const D0: Vec3 = Vec3::new(
    0.573_576_436_351_046,
    0.740_535_693_464_567_5,
    0.350_889_803_483_932_2,
);
const D1: Vec3 = Vec3::new(
    -0.350_889_803_483_932_2,
    0.573_576_436_351_046,
    0.740_535_693_464_567_5,
);

/// The point on a z-axis torus at the origin, at azimuth `u`, where the line
/// along `d` is tangent: the normal `(cos v cos u, cos v sin u, sin v)` is
/// orthogonal to `d`. `outer` picks the convex outer part (`cos v > 0`),
/// otherwise the saddle-shaped inner part.
fn tangent_point(big_r: f64, r: f64, d: Vec3, u: f64, outer: bool) -> Point3 {
    let a = d.x() * u.cos() + d.y() * u.sin();
    let v = (-a).atan2(d.z());
    let v = if (v.cos() > 0.0) == outer {
        v
    } else {
        v - std::f64::consts::PI
    };
    let rho = big_r + r * v.cos();
    Point3::new(rho * u.cos(), rho * u.sin(), r * v.sin())
}

/// Points outside the torus where the lines along `D0` and `D1`, both
/// running forward from the point, are tangent to it.
fn doubly_tangent_points(big_r: f64, r: f64, outer: bool) -> Vec<Point3> {
    use std::f64::consts::TAU;
    let normal = D0.cross(D1).normalize().unwrap();
    let mut points = Vec::new();
    for i in 0..100 {
        let u0 = f64::from(i) * TAU / 100.0;
        let t0 = tangent_point(big_r, r, D0, u0, outer);
        // The two tangent lines meet where their offset has no component
        // along the common normal.
        let gap = |u1: f64| (tangent_point(big_r, r, D1, u1, outer) - t0).dot(normal);
        let steps = 360;
        for j in 0..steps {
            let (mut lo, mut hi) = (
                f64::from(j) * TAU / f64::from(steps),
                f64::from(j + 1) * TAU / f64::from(steps),
            );
            if gap(lo).signum() == gap(hi).signum() {
                continue;
            }
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if gap(lo).signum() == gap(mid).signum() {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let t1 = tangent_point(big_r, r, D1, 0.5 * (lo + hi), outer);
            let w = t0 - t1;
            let (aa, bb, cc) = (D0.dot(D0), D0.dot(D1), D1.dot(D1));
            let (dd, ee) = (D0.dot(w), D1.dot(w));
            let den = aa * cc - bb * bb;
            let a = (bb * ee - cc * dd) / den;
            let b = (aa * ee - bb * dd) / den;
            // Both tangent points ahead of the point.
            if a > -0.01 * r || b > -0.01 * r {
                continue;
            }
            let p = t0 + D0 * a;
            if (p - (t1 + D1 * b)).length() > 1e-9 * (1.0 + big_r) {
                continue;
            }
            let outside = (p.x().hypot(p.y()) - big_r).hypot(p.z()) - r;
            if outside > 1e-3 * r {
                points.push(p);
            }
        }
    }
    points
}

#[test]
fn rays_tangent_to_a_torus_add_no_parity() {
    let options = ClassifyOptions::default();
    let mut failures = Vec::new();
    let mut tried = 0;
    for &r in &[1e-3, 0.37, 1.0, 13.0] {
        for &offset in &[0.0, 100.0 * r] {
            let big_r = 3.0 * r;
            let mut topo = Topology::new();
            let solid = make_torus(&mut topo, big_r, r, 32).unwrap();
            let by = Vec3::new(offset, -0.5 * offset, offset / 3.0);
            if offset > 0.0 {
                transform_solid(&mut topo, solid, &Mat4::translation(by.x(), by.y(), by.z()))
                    .unwrap();
            }
            let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
            for outer in [true, false] {
                let points = doubly_tangent_points(big_r, r, outer);
                assert!(
                    !points.is_empty(),
                    "no fixture points at r {r:e} outer {outer}"
                );
                for p in points {
                    let p = p + by;
                    tried += 1;
                    let got = [
                        classify_point(&topo, solid, p, &options).unwrap(),
                        prepared.classify_point(p, &options).unwrap(),
                    ];
                    if got != [PointClassification::Outside; 2] {
                        failures.push(format!(
                            "r {r:e} offset {offset:e} outer {outer} {p:?}: {got:?}"
                        ));
                    }
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {tried} doubly tangent points misclassified:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
