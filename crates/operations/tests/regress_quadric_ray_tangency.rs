//! Regression (RFC 0008 F3): a classification ray exactly tangent to a
//! cylinder or sphere wall touches it without crossing, so it must add no
//! crossing parity. The check classifier's absolute `1e-15` discriminant
//! test snapped a tangent contact to one root at small scale only: a point
//! outside a radius-0.37 cylinder whose first two base rays are tangent to
//! the wall classified `Inside`, while radii 1, 2 and 13 were right.
//!
//! Each case builds the point from the closed-form tangency condition at
//! many radii and translations, so the expectation never depends on the
//! classifier under test.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::primitives::{make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// The check classifier's first two base ray directions (`classify/mod.rs`).
const D0: [f64; 3] = [
    0.573_576_436_351_046,
    0.740_535_693_464_567_5,
    0.350_889_803_483_932_2,
];
const D1: [f64; 3] = [
    -0.350_889_803_483_932_2,
    0.573_576_436_351_046,
    0.740_535_693_464_567_5,
];

const RADII: [f64; 9] = [1e-3, 0.01, 0.37, 1.0, 2.0, 13.0, 1e3, 1e5, 1e6];
const OFFSETS: [f64; 5] = [0.0, 1e3, 1e6, 1e8, 1e9];

/// The translation applied at magnitude `offset`: off-axis in every
/// coordinate so no component stays exactly zero.
fn shift(offset: f64) -> Vec3 {
    Vec3::new(offset, -0.5 * offset, offset / 3.0)
}

fn translated(topo: &mut Topology, solid: SolidId, by: Vec3) -> SolidId {
    if by.length() > 0.0 {
        transform_solid(topo, solid, &Mat4::translation(by.x(), by.y(), by.z())).unwrap();
    }
    solid
}

/// Both classification paths, which share one vote loop but not one
/// crossing source.
fn classify_both(topo: &Topology, solid: SolidId, point: Point3) -> [PointClassification; 2] {
    let options = ClassifyOptions::default();
    let one_shot = classify_point(topo, solid, point, &options).unwrap();
    let prepared = PreparedSolid::prepare(topo, solid)
        .unwrap()
        .classify_point(point, &options)
        .unwrap();
    [one_shot, prepared]
}

/// The point outside a z-axis cylinder of radius `r` centred on the origin
/// where the lines along `D0` and `D1` are both tangent to the wall: its xy
/// projection lies at signed distance `+r` from the projected `D0` line and
/// `-r` from the projected `D1` line, which puts it outside the circle.
fn cylinder_tangent_point_xy(r: f64) -> (f64, f64) {
    let normal = |d: [f64; 3]| {
        let len = d[0].hypot(d[1]);
        (d[1] / len, -d[0] / len)
    };
    let (a0, b0) = normal(D0);
    let (a1, b1) = normal(D1);
    // a0 x + b0 y = r, a1 x + b1 y = -r.
    let det = a0 * b1 - a1 * b0;
    ((r * b1 + r * b0) / det, (-a0 * r - a1 * r) / det)
}

#[test]
fn rays_tangent_to_a_cylinder_wall_add_no_parity_at_any_scale() {
    let mut failures = Vec::new();
    for &r in &RADII {
        for &offset in &OFFSETS {
            let height = 1000.0 * r;
            let mut topo = Topology::new();
            let solid = make_cylinder(&mut topo, r, height).unwrap();
            let by = shift(offset);
            let solid = translated(&mut topo, solid, by);
            let (x, y) = cylinder_tangent_point_xy(r);
            let point = Point3::new(x + by.x(), y + by.y(), 0.5 * height + by.z());
            let got = classify_both(&topo, solid, point);
            if got != [PointClassification::Outside; 2] {
                failures.push(format!("r {r:e}, offset {offset:e}: {got:?}"));
            }
            // A point just inside the axis, on the same tangent rays' level,
            // stays inside: the fix must not drop real crossings.
            let inside = Point3::new(0.25 * x + by.x(), 0.25 * y + by.y(), 0.5 * height + by.z());
            let got = classify_both(&topo, solid, inside);
            if got != [PointClassification::Inside; 2] {
                failures.push(format!("inside r {r:e}, offset {offset:e}: {got:?}"));
            }
        }
    }
    assert!(
        failures.is_empty(),
        "tangent-ray parity failures:\n{}",
        failures.join("\n")
    );
}

/// The RFC 0008 appendix repro, verbatim: cylinder r 0.37, h 1000.
#[test]
fn rfc_0008_f3_repro_classifies_outside() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 0.37, 1000.0).unwrap();
    let (x, y) = cylinder_tangent_point_xy(0.37);
    assert!((x - -0.035_813).abs() < 1e-6 && (y - -0.650_472).abs() < 1e-6);
    let got = classify_both(&topo, solid, Point3::new(x, y, 500.0));
    assert_eq!(got, [PointClassification::Outside; 2]);
}

/// The point outside a sphere of radius `r` centred on the origin where the
/// lines along `D0` and `D1` are both tangent to it, ahead of the point.
///
/// With `w` the unit bisector of the two directions and `n` their unit
/// normal, `p = a n + b w` is at distance `sqrt(|p|^2 - (p.d)^2)` from each
/// line, and `p.d = b k` is the same for both (`k = w.d`). `b = -1.5 r` puts
/// the tangent points ahead (`t = -b k > 0`); `a` then solves the distance
/// equation, and `|p|^2 = a^2 + b^2 > r^2` puts the point outside.
fn sphere_tangent_point(r: f64) -> Vec3 {
    let d0 = Vec3::new(D0[0], D0[1], D0[2]).normalize().unwrap();
    let d1 = Vec3::new(D1[0], D1[1], D1[2]).normalize().unwrap();
    let w = (d0 + d1).normalize().unwrap();
    let n = d0.cross(d1).normalize().unwrap();
    let k = w.dot(d0);
    let b = -1.5 * r;
    let a = (r * r - b * b * (1.0 - k * k)).sqrt();
    n * a + w * b
}

#[test]
fn rays_tangent_to_a_sphere_add_no_parity_at_any_scale() {
    let mut failures = Vec::new();
    for &r in &RADII {
        for &offset in &OFFSETS {
            let mut topo = Topology::new();
            let solid = make_sphere(&mut topo, r, 16).unwrap();
            let by = shift(offset);
            let solid = translated(&mut topo, solid, by);
            let p = sphere_tangent_point(r) + by;
            let point = Point3::new(p.x(), p.y(), p.z());
            let got = classify_both(&topo, solid, point);
            if got != [PointClassification::Outside; 2] {
                failures.push(format!("r {r:e}, offset {offset:e}: {got:?}"));
            }
        }
    }
    assert!(
        failures.is_empty(),
        "tangent-ray parity failures:\n{}",
        failures.join("\n")
    );
}
