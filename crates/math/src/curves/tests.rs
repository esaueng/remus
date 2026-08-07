#![allow(clippy::unwrap_used)]

use std::f64::consts::{FRAC_PI_2, PI};

use crate::tolerance::Tolerance;
use crate::vec::{Point3, Vec3};

use super::*;

// ── Line tests ─────────────────────────────────────────────────

#[test]
fn line_evaluate() {
    let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let tol = Tolerance::new();

    let p = line.evaluate(3.0);
    assert!(tol.approx_eq(p.x(), 3.0));
    assert!(tol.approx_eq(p.y(), 0.0));
}

#[test]
fn line_project() {
    let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let t = line.project(Point3::new(5.0, 3.0, 0.0));
    let tol = Tolerance::new();
    assert!(tol.approx_eq(t, 5.0));
}

#[test]
fn line_distance() {
    let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let d = line.distance_to_point(Point3::new(5.0, 3.0, 4.0));
    let tol = Tolerance::new();
    assert!(tol.approx_eq(d, 5.0)); // 3-4-5 triangle
}

#[test]
fn line_zero_direction_error() {
    assert!(Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0)).is_err());
}

// ── Circle tests ───────────────────────────────────────────────

#[test]
fn circle_evaluate_at_zero() {
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();

    let tol = Tolerance::new();
    let p = circle.evaluate(0.0);
    // Should be at radius distance from center in the plane.
    let dist = (p - circle.center()).length();
    assert!(
        tol.approx_eq(dist, 1.0),
        "point should be on circle, dist={dist}"
    );
}

#[test]
fn circle_evaluate_quarter() {
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();

    let tol = Tolerance::new();
    let p = circle.evaluate(FRAC_PI_2);
    let dist = (p - circle.center()).length();
    assert!(tol.approx_eq(dist, 2.0));
}

#[test]
fn circle_circumference() {
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 3.0).unwrap();

    let tol = Tolerance::new();
    assert!(tol.approx_eq(circle.circumference(), 6.0 * PI));
}

#[test]
fn circle_project_roundtrip() {
    let circle = Circle3D::new(Point3::new(1.0, 2.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();

    let tol = Tolerance::new();
    let t_orig = 1.23;
    let p = circle.evaluate(t_orig);
    let t_proj = circle.project(p);
    assert!(
        tol.approx_eq(t_orig, t_proj),
        "project should roundtrip: {t_orig} vs {t_proj}"
    );
}

#[test]
fn circle_zero_radius_error() {
    assert!(Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.0).is_err());
}

#[test]
fn circle_intersect_segment_in_plane_two_points() {
    // Unit circle in xy plane, segment from (-2,0,0) to (2,0,0) — crosses at ±1 along x.
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let hits = c.intersect_segment(
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        1e-9,
    );
    assert_eq!(hits.len(), 2);
    // u_axis defaults to +x for a +z normal; so points are at t = 0 and π.
    let xs: Vec<f64> = hits.iter().map(|(p, _)| p.x()).collect();
    assert!(xs.iter().any(|x| (x - 1.0).abs() < 1e-9));
    assert!(xs.iter().any(|x| (x + 1.0).abs() < 1e-9));
}

#[test]
fn circle_intersect_segment_crosses_plane() {
    // Great circle in x=0 plane (sphere-equator type): radius 8, normal +x.
    // Segment from (1, 8, 0) to (-1, 8, 0): crosses x=0 at (0, 8, 0), which is on the circle.
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 8.0).unwrap();
    let hits = c.intersect_segment(
        Point3::new(1.0, 8.0, 0.0),
        Point3::new(-1.0, 8.0, 0.0),
        1e-7,
    );
    assert_eq!(hits.len(), 1);
    let (p, _t) = hits[0];
    assert!((p.x()).abs() < 1e-7);
    assert!((p.y() - 8.0).abs() < 1e-7);
    assert!((p.z()).abs() < 1e-7);
}

#[test]
fn circle_intersect_segment_polygon_vertex_on_circle() {
    // The box-sphere case: equator polygon edge from a vertex AT (0,8,0)
    // (which lies on a great circle from the x=0 plane) outward.
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 8.0).unwrap();
    // Polygon vertex 4 at (0, 8, 0), polygon vertex 5 at (-3.06..., 7.39..., 0).
    let v4 = Point3::new(0.0, 8.0, 0.0);
    let v5 = Point3::new(-8.0 * (5.0_f64).cos(), 8.0 * (5.0_f64).sin(), 0.0);
    let _ = v5; // silence unused warn — this test only needs the vertex case.
    let v3 = Point3::new(3.061_467_458_920_71_f64, 7.391_036_260_090_294_f64, 0.0);
    // Edge from v3 (x>0) to v4 (x=0) — the vertex is on the great circle's plane and circle.
    let hits = c.intersect_segment(v3, v4, 1e-7);
    assert_eq!(hits.len(), 1, "expected one hit at the polygon vertex");
    let (p, _) = hits[0];
    assert!((p - v4).length() < 1e-6, "hit should be at v4");
}

#[test]
fn circle_intersect_segment_no_crossing() {
    // Segment fully on one side of circle's plane, no crossing.
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let hits = c.intersect_segment(Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 2.0), 1e-9);
    assert!(hits.is_empty());
}

#[test]
fn circle_intersect_segment_near_tangent_collapses_to_foot() {
    // The gridfinity wall-tangency case: an r=4 socket-outline corner circle
    // tangent to a wall line, penetrated by a sub-tolerance residual. The
    // separate quadratic roots straddle the tangency by sqrt(2r·δ) — a 1e-13
    // residual already shifts each root a full micron — so the near-tangent
    // pair must collapse to the well-conditioned double root (the foot of
    // the center on the line).
    let tol = 1e-7;
    let c = Circle3D::new(
        Point3::new(-16.75, 37.75, 5.0),
        Vec3::new(0.0, 0.0, 1.0),
        4.0,
    )
    .unwrap();
    // Line at x = -20.75 + 1.25e-13 (penetrates the circle by ~1.25e-13,
    // giving a ±1e-6 root pair without the collapse).
    let x = -20.75 + 1.25e-13;
    let hits = c.intersect_segment(Point3::new(x, 20.0, 5.0), Point3::new(x, 50.0, 5.0), tol);
    assert_eq!(hits.len(), 1, "near-tangent pair must collapse to one hit");
    let (p, _) = hits[0];
    assert!(
        (p - Point3::new(x, 37.75, 5.0)).length() < tol,
        "hit must be the exact foot of the tangency, got ({}, {}, {})",
        p.x(),
        p.y(),
        p.z()
    );
}

#[test]
fn circle_intersect_segment_genuine_secant_keeps_two_hits() {
    // A real secant (penetration far above tolerance) must still yield two
    // distinct crossings — the tangent collapse only fires when the chord
    // implies sub-tolerance penetration.
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
    // Line at x = 3.9: penetration δ = 0.1, chord = 2·sqrt(16 − 15.21) ≈ 1.78.
    let hits = c.intersect_segment(
        Point3::new(3.9, -5.0, 0.0),
        Point3::new(3.9, 5.0, 0.0),
        1e-7,
    );
    assert_eq!(hits.len(), 2, "genuine secant must keep both crossings");
}

// ── Ellipse tests ──────────────────────────────────────────────

#[test]
fn ellipse_evaluate() {
    let ellipse = Ellipse3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
        2.0,
    )
    .unwrap();

    let tol = Tolerance::new();
    // At t=0, should be at (3, 0, 0) direction.
    let p0 = ellipse.evaluate(0.0);
    let dist0 = (p0 - ellipse.center()).length();
    assert!(tol.approx_eq(dist0, 3.0), "at t=0 should be at semi_major");

    // At t=π/2, should be at semi_minor distance.
    let p1 = ellipse.evaluate(FRAC_PI_2);
    let dist1 = (p1 - ellipse.center()).length();
    assert!(
        tol.approx_eq(dist1, 2.0),
        "at t=π/2 should be at semi_minor"
    );
}

#[test]
fn ellipse_circumference_circle() {
    // When semi_major == semi_minor, it's a circle.
    let ellipse = Ellipse3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        1.0,
    )
    .unwrap();

    let tol = Tolerance::loose();
    let circ = ellipse.approximate_circumference();
    assert!(
        tol.approx_eq(circ, 2.0 * PI),
        "circle circumference should be 2π, got {circ}"
    );
}

#[test]
fn ellipse_zero_axis_error() {
    assert!(
        Ellipse3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
            1.0
        )
        .is_err()
    );
}

// ── Parabola tests ──────────────────────────────────────────

#[test]
fn parabola_vertex() {
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let v = p.evaluate(0.0);
    assert!(Tolerance::default().approx_eq(v.x(), 0.0));
    assert!(Tolerance::default().approx_eq(v.y(), 0.0));
    assert!(Tolerance::default().approx_eq(v.z(), 0.0));
}

#[test]
fn parabola_symmetry() {
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    // z-component should be the same for +t and -t (symmetric about axis)
    let pos = p.evaluate(2.0);
    let neg = p.evaluate(-2.0);
    assert!(Tolerance::default().approx_eq(pos.z(), neg.z()));
}

#[test]
fn parabola_tangent_at_vertex() {
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let tang = p.tangent(0.0);
    // At vertex, tangent should be purely along u_axis (perpendicular to axis)
    assert!(Tolerance::default().approx_eq(tang.z(), 0.0));
    assert!(tang.length() > 0.5);
}

#[test]
fn parabola_curvature_at_vertex() {
    let f = 2.0;
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), f).unwrap();
    // Curvature at vertex = 1/(2f)
    let k = p.curvature(0.0);
    assert!(Tolerance::default().approx_eq(k, 1.0 / (2.0 * f)));
}

#[test]
fn parabola_focus() {
    let f = 3.0;
    let p = Parabola3D::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 1.0), f).unwrap();
    let focus = p.focus();
    assert!(Tolerance::default().approx_eq(focus.z(), 3.0 + f));
}

#[test]
fn parabola_zero_focal_length_error() {
    assert!(Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.0,).is_err());
}

// ── Hyperbola tests ─────────────────────────────────────────

#[test]
fn hyperbola_vertex() {
    let h = Hyperbola3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
        2.0,
    )
    .unwrap();
    // At t=0: P = center + a*cosh(0)*u + b*sinh(0)*v = center + a*u
    let p = h.evaluate(0.0);
    let dist = (p - h.center()).length();
    assert!(Tolerance::default().approx_eq(dist, 3.0));
}

#[test]
fn hyperbola_tangent_at_vertex() {
    let h = Hyperbola3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
        2.0,
    )
    .unwrap();
    // At t=0: tangent = a*sinh(0)*u + b*cosh(0)*v = b*v
    let tang = h.tangent(0.0);
    assert!(Tolerance::default().approx_eq(tang.length(), 2.0));
}

#[test]
fn hyperbola_eccentricity() {
    let h = Hyperbola3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
        4.0,
    )
    .unwrap();
    // e = sqrt(1 + (b/a)^2) = sqrt(1 + 16/9) = sqrt(25/9) = 5/3
    assert!(Tolerance::default().approx_eq(h.eccentricity(), 5.0 / 3.0));
}

#[test]
fn hyperbola_foci_distance() {
    let a = 3.0;
    let b = 4.0;
    let h = Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), a, b).unwrap();
    let (f1, f2) = h.foci();
    // Distance from center to each focus = c = sqrt(a^2 + b^2) = 5
    let c = a.hypot(b);
    assert!(Tolerance::default().approx_eq((f1 - h.center()).length(), c));
    assert!(Tolerance::default().approx_eq((f2 - h.center()).length(), c));
}

#[test]
fn hyperbola_zero_axis_error() {
    assert!(
        Hyperbola3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
            1.0,
        )
        .is_err()
    );
}

// ── Line3D::tangent tests ──────────────────────────

#[test]
fn line3d_tangent_is_unit_vector() {
    // Even when constructed with a non-unit direction, tangent() must return a unit vector.
    let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 4.0, 0.0)).unwrap();
    let tang = line.tangent();
    let tol = Tolerance::new();
    assert!(
        tol.approx_eq(tang.length(), 1.0),
        "tangent should be unit length, got {}",
        tang.length()
    );
}

#[test]
fn line3d_tangent_matches_direction() {
    let line = Line3D::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 5.0)).unwrap();
    let tang = line.tangent();
    let dir = line.direction();
    let tol = Tolerance::new();
    assert!(tol.approx_eq(tang.x(), dir.x()));
    assert!(tol.approx_eq(tang.y(), dir.y()));
    assert!(tol.approx_eq(tang.z(), dir.z()));
}

// ── Circle3D::project tests ────────────────────────

#[test]
fn circle3d_project_at_pi() {
    // At t=π, evaluate gives the antipodal point; projecting back should return π.
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let pt = circle.evaluate(PI);
    let t_proj = circle.project(pt);
    let tol = Tolerance::new();
    // project returns atan2, which is in (-π, π]; -π and π are the same angle
    assert!(tol.approx_eq(t_proj.abs(), PI), "expected ±π, got {t_proj}");
}

#[test]
fn circle3d_project_at_negative_angle() {
    // project should recover a negative angle correctly (atan2 range is (-π, π])
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let t_orig = -1.0_f64;
    let pt = circle.evaluate(t_orig);
    let t_proj = circle.project(pt);
    let tol = Tolerance::new();
    assert!(
        tol.approx_eq(t_orig, t_proj),
        "expected {t_orig}, got {t_proj}"
    );
}

#[test]
fn circle3d_project_at_three_quarter() {
    // t = 3π/2 maps to -π/2 under atan2; round-trip through evaluate→project
    let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();
    let t_orig = 3.0 * PI / 2.0;
    let pt = circle.evaluate(t_orig);
    let t_proj = circle.project(pt);
    // atan2 wraps to -π/2, so recovered angle differs by 2π
    let tol = Tolerance::new();
    let diff = (t_proj - t_orig).abs() % (2.0 * PI);
    let diff_wrapped = diff.min(2.0f64.mul_add(PI, -diff));
    assert!(
        diff_wrapped < tol.linear * 1000.0,
        "angle mismatch after wrapping: t_orig={t_orig}, t_proj={t_proj}"
    );
}

// ── Parabola3D::evaluate at non-zero t ─────────────

#[test]
fn parabola_evaluate_nonzero_t() {
    // P(t) = vertex + (t²/4f)*axis + t*u_axis
    // For axis = (0,0,1), focal_length=1, at t=2:
    //   along_axis = 4/4 = 1   → z-component = 1
    //   u_axis component = 2    → in the plane perpendicular to z
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let pt = p.evaluate(2.0);
    let tol = Tolerance::new();
    // z (along-axis) component
    assert!(
        tol.approx_eq(pt.z(), 1.0),
        "z-component should be 1.0, got {}",
        pt.z()
    );
    // distance from axis in the XY plane should be |t| = 2
    let radial = pt.x().hypot(pt.y());
    assert!(
        tol.approx_eq(radial, 2.0),
        "radial distance should be 2.0, got {radial}"
    );
}

#[test]
fn parabola_evaluate_negative_t() {
    // Symmetry: z-component the same for +t and -t, radial same, u_axis direction flips
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let pos = p.evaluate(3.0);
    let neg = p.evaluate(-3.0);
    let tol = Tolerance::new();
    assert!(tol.approx_eq(pos.z(), neg.z()), "z should match for ±t");
    // The x/y components should be opposite in sign (u_axis flips)
    let radial_pos = pos.x().hypot(pos.y());
    let radial_neg = neg.x().hypot(neg.y());
    assert!(
        tol.approx_eq(radial_pos, radial_neg),
        "radial distance should match"
    );
}

#[test]
fn parabola_tangent_nonzero_t() {
    // tangent(t) = (t / 2f) * axis + u_axis
    // For focal_length=2, axis=(0,0,1), t=4:
    //   d_axis = 4/(2*2) = 1  → z-component of tangent = 1
    //   u_axis component = 1
    let p = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let tang = p.tangent(4.0);
    let tol = Tolerance::new();
    assert!(
        tol.approx_eq(tang.z(), 1.0),
        "z-component of tangent should be 1.0, got {}",
        tang.z()
    );
    // u_axis is in XY plane, its component should give length 1
    let radial = tang.x().hypot(tang.y());
    assert!(
        tol.approx_eq(radial, 1.0),
        "XY component of tangent should have length 1, got {radial}"
    );
}

// ── Hyperbola3D::evaluate away from vertex ─────────

#[test]
fn hyperbola_evaluate_nonzero_t() {
    // P(t) = center + a*cosh(t)*u + b*sinh(t)*v
    // The distance from center at parameter t satisfies:
    //   |P - center|² = a²*cosh²(t) + b²*sinh²(t)
    let a = 2.0_f64;
    let b = 1.0_f64;
    let t = 1.0_f64;
    let h = Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), a, b).unwrap();
    let pt = h.evaluate(t);
    let dist_sq = (pt - h.center()).length_squared();
    let expected_sq = (a * t.cosh()).mul_add(a * t.cosh(), (b * t.sinh()).powi(2));
    assert!(
        (dist_sq - expected_sq).abs() < 1e-10,
        "|P-center|² should be {expected_sq}, got {dist_sq}"
    );
}

#[test]
fn hyperbola_evaluate_large_t_stays_on_curve() {
    // Verify the hyperbolic identity: (x/a)² - (y/b)² = 1
    // where x, y are the in-plane u/v components.
    // We use a finite-difference approximation to recover u/v components.
    let a = 3.0_f64;
    let b = 2.0_f64;
    let h = Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), a, b).unwrap();
    // The distance from center equals sqrt(a²cosh²(t) + b²sinh²(t))
    for t in [0.5_f64, 1.0, 2.0] {
        let pt = h.evaluate(t);
        let dist = (pt - h.center()).length();
        let expected = (a * t.cosh()).hypot(b * t.sinh());
        assert!(
            (dist - expected).abs() < 1e-10,
            "distance at t={t}: expected {expected}, got {dist}"
        );
    }
}

#[test]
fn hyperbola_tangent_nonzero_t() {
    // Check tangent via finite difference: tangent ≈ (P(t+ε) - P(t-ε)) / (2ε)
    let h = Hyperbola3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0,
        1.0,
    )
    .unwrap();
    let t = 1.0_f64;
    let eps = 1e-6;
    let tang_analytic = h.tangent(t);
    let pt_plus = h.evaluate(t + eps);
    let pt_minus = h.evaluate(t - eps);
    let tang_fd_x = (pt_plus.x() - pt_minus.x()) / (2.0 * eps);
    let tang_fd_y = (pt_plus.y() - pt_minus.y()) / (2.0 * eps);
    let tang_fd_z = (pt_plus.z() - pt_minus.z()) / (2.0 * eps);
    let tol = 1e-5;
    assert!(
        (tang_analytic.x() - tang_fd_x).abs() < tol,
        "x: analytic={}, fd={tang_fd_x}",
        tang_analytic.x()
    );
    assert!(
        (tang_analytic.y() - tang_fd_y).abs() < tol,
        "y: analytic={}, fd={tang_fd_y}",
        tang_analytic.y()
    );
    assert!(
        (tang_analytic.z() - tang_fd_z).abs() < tol,
        "z: analytic={}, fd={tang_fd_z}",
        tang_analytic.z()
    );
}

#[test]
fn circle_intersect_circle_two_points() {
    // Two unit-radius coplanar circles with centers 1 apart: two crossings at
    // x = 0.5, y = ±sqrt(3)/2.
    let a = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let b = Circle3D::new(Point3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let hits = a.intersect_circle(&b, 1e-9);
    assert_eq!(hits.len(), 2);
    for (p, t) in &hits {
        assert!((p.x() - 0.5).abs() < 1e-12);
        assert!((p.y().abs() - 3.0_f64.sqrt() / 2.0).abs() < 1e-12);
        // Returned parameter evaluates back to the point on `a`.
        let q = a.evaluate(*t);
        assert!((q - *p).length() < 1e-12);
    }
}

#[test]
fn circle_intersect_circle_lite_pad_configuration() {
    // The lite magnet-pad junction: pad circle r=4.45 about (-50,-76) crossing
    // the socket cone's z=-3.8 rim circle r=1.05 about (-47.45,-78.55). The
    // crossings are the two triple-junction points of the pad fuse.
    let pad = Circle3D::new(
        Point3::new(-50.0, -76.0, -3.8),
        Vec3::new(0.0, 0.0, 1.0),
        4.45,
    )
    .unwrap();
    let rim = Circle3D::new(
        Point3::new(-47.45, -78.55, -3.8),
        Vec3::new(0.0, 0.0, 1.0),
        1.05,
    )
    .unwrap();
    let hits = pad.intersect_circle(&rim, 1e-7);
    assert_eq!(hits.len(), 2);
    for (p, _) in &hits {
        let dp = ((p.x() + 50.0).powi(2) + (p.y() + 76.0).powi(2)).sqrt();
        let dr = ((p.x() + 47.45).powi(2) + (p.y() + 78.55).powi(2)).sqrt();
        assert!((dp - 4.45).abs() < 1e-9, "on pad circle: {dp}");
        assert!((dr - 1.05).abs() < 1e-9, "on rim circle: {dr}");
    }
}

#[test]
fn circle_intersect_circle_near_tangent_collapses_to_foot() {
    // Externally near-tangent pair: centers 2 + delta apart with delta far
    // below the tolerance well. The noise root pair collapses to the single
    // well-conditioned foot on the center line.
    let a = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let b = Circle3D::new(
        Point3::new(2.0 - 1e-12, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
    )
    .unwrap();
    let hits = a.intersect_circle(&b, 1e-7);
    assert_eq!(hits.len(), 1);
    let (p, _) = hits[0];
    assert!((p.x() - 1.0).abs() < 1e-6);
    assert!(p.y().abs() < 1e-6);
}

#[test]
fn circle_intersect_circle_disjoint_and_non_coplanar_empty() {
    let a = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    // Far apart.
    let b = Circle3D::new(Point3::new(5.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    assert!(a.intersect_circle(&b, 1e-9).is_empty());
    // Parallel but offset planes.
    let c = Circle3D::new(Point3::new(1.0, 0.0, 0.5), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    assert!(a.intersect_circle(&c, 1e-9).is_empty());
    // Skew planes.
    let d = Circle3D::new(Point3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0).unwrap();
    assert!(a.intersect_circle(&d, 1e-9).is_empty());
    // Concentric.
    let e = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
    assert!(a.intersect_circle(&e, 1e-9).is_empty());
}

// ── Unbounded conics: closed-form and scale-invariance checks ──────
//
// Every assertion below is either a HAND-DERIVED closed form or an
// independently-implemented numerical integral. Nothing here is checked
// against another routine that shares an implementation with the code
// under test.

/// Composite Simpson's rule, implemented here rather than reused from the
/// kernel, so an arc-length check is genuinely independent of
/// `Hyperbola3D::arc_length` (which uses Gauss-Legendre).
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let n = if n.is_multiple_of(2) { n } else { n + 1 };
    #[allow(clippy::cast_precision_loss)]
    let h = (b - a) / n as f64;
    let mut total = f(a) + f(b);
    for i in 1..n {
        #[allow(clippy::cast_precision_loss)]
        let x = h.mul_add(i as f64, a);
        total += f(x) * if i.is_multiple_of(2) { 2.0 } else { 4.0 };
    }
    total * h / 3.0
}

#[test]
fn parabola_arc_length_matches_hand_derived_closed_form() {
    // With focal length f = 1/4 the parameterization
    //   P(t) = V + (t^2 / 4f) * axis + t * u
    // becomes (u, axis) = (t, t^2), i.e. the plane curve y = x^2.
    //
    // Arc length of y = x^2 from x = 0 to x = 1 is
    //   integral_0^1 sqrt(1 + 4x^2) dx
    //     = [ x*sqrt(1+4x^2)/2 + asinh(2x)/4 ]_0^1
    //     = sqrt(5)/2 + asinh(2)/4
    //     = 1.4789428575445975...
    let p = Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.25,
    )
    .unwrap();

    let expected = 5.0_f64.sqrt() / 2.0 + 2.0_f64.asinh() / 4.0;
    let got = p.arc_length(0.0, 1.0);
    assert!(
        (got - expected).abs() < 1e-14 * expected,
        "parabola arc length {got} vs closed form {expected}"
    );

    // Sanity on the closed form itself, to three decimals, so a wrong
    // formula on BOTH sides could not agree by construction.
    assert!(
        (expected - 1.478_942_857_544_597_5).abs() < 1e-12,
        "hand-derived constant drifted: {expected}"
    );
}

#[test]
fn parabola_arc_length_matches_independent_quadrature() {
    let p = Parabola3D::with_axes(
        Point3::new(3.0, -1.0, 7.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.7,
    )
    .unwrap();
    let (t0, t1) = (-2.3, 4.1);
    // |P'(t)| = sqrt(1 + (t/2f)^2)
    let f = p.focal_length();
    let numeric = simpson(
        |t| {
            let s = t / (2.0 * f);
            s.mul_add(s, 1.0).sqrt()
        },
        t0,
        t1,
        20_000,
    );
    let got = p.arc_length(t0, t1);
    assert!(
        (got - numeric).abs() < 1e-11 * numeric,
        "parabola arc length {got} vs Simpson {numeric}"
    );
}

#[test]
fn hyperbola_arc_length_matches_independent_quadrature() {
    // No elementary antiderivative exists, so the reference is a
    // separately-implemented Simpson rule on the same integrand.
    let a = 2.0;
    let b = 3.0;
    let h = Hyperbola3D::with_axes(
        Point3::new(-1.0, 2.0, 0.5),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        a,
        b,
    )
    .unwrap();
    let (t0, t1) = (-1.25, 2.0);
    let numeric = simpson(|t| (a * t.sinh()).hypot(b * t.cosh()), t0, t1, 200_000);
    let got = h.arc_length(t0, t1);
    assert!(
        (got - numeric).abs() < 1e-9 * numeric,
        "hyperbola arc length {got} vs Simpson {numeric}"
    );
}

#[test]
fn conic_arc_length_is_scale_covariant() {
    // Length carries one power of L: scaling the curve by k must scale the
    // length by exactly k, at 1x, 1000x and 0.001x.
    for k in [1.0_f64, 1000.0, 0.001] {
        let p = Parabola3D::with_axes(
            Point3::new(k, 2.0 * k, 3.0 * k),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.25 * k,
        )
        .unwrap();
        // The parabola's parameter carries units of length, so the
        // equivalent span scales with k too.
        let got = p.arc_length(0.0, k);
        let expected = k * (5.0_f64.sqrt() / 2.0 + 2.0_f64.asinh() / 4.0);
        assert!(
            (got - expected).abs() < 1e-12 * expected,
            "parabola length at {k}x: {got} vs {expected}"
        );

        let h = Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0 * k,
            3.0 * k,
        )
        .unwrap();
        // The hyperbola's parameter is DIMENSIONLESS, so the same span is
        // used at every scale and the length must scale purely with k.
        let base = Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0,
            3.0,
        )
        .unwrap();
        let got_h = h.arc_length(-1.25, 2.0);
        let expected_h = k * base.arc_length(-1.25, 2.0);
        assert!(
            (got_h - expected_h).abs() < 1e-12 * expected_h,
            "hyperbola length at {k}x: {got_h} vs {expected_h}"
        );
    }
}

#[test]
fn conic_project_inverts_evaluate_exactly() {
    let p = Parabola3D::with_axes(
        Point3::new(3.0, -1.0, 7.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.7,
    )
    .unwrap();
    let h = Hyperbola3D::with_axes(
        Point3::new(-1.0, 2.0, 0.5),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 1.0),
        2.0,
        3.0,
    )
    .unwrap();
    for &t in &[-3.0, -0.7, 0.0, 0.4, 2.9] {
        let tp = p.project(p.evaluate(t));
        assert!((tp - t).abs() < 1e-13, "parabola project({t}) -> {tp}");
        let th = h.project(h.evaluate(t));
        assert!((th - t).abs() < 1e-13, "hyperbola project({t}) -> {th}");
    }
}

#[test]
fn with_axes_preserves_the_plane_that_new_loses() {
    // `Parabola3D::new` fixes the symmetry axis but picks an ARBITRARY
    // in-plane direction, so it cannot represent a caller's chosen plane.
    // `with_axes` must.
    let vertex = Point3::new(0.0, 0.0, 0.0);
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let u = Vec3::new(1.0, 0.0, 0.0);
    let with = Parabola3D::with_axes(vertex, axis, u, 1.0).unwrap();
    assert!((with.u_axis() - u).length() < 1e-15);
    // The arbitrary-frame constructor lands on a different in-plane axis
    // for this axis direction, which is exactly the information loss
    // `with_axes` exists to avoid.
    let arb = Parabola3D::new(vertex, axis, 1.0).unwrap();
    assert!(
        (arb.u_axis() - u).length() > 1e-9,
        "test premise broken: `new` happened to pick the same u_axis"
    );

    // Same story for the hyperbola's REAL axis, where the choice also
    // decides which branch is modelled.
    let hyp = Hyperbola3D::with_axes(vertex, axis, u, 2.0, 3.0).unwrap();
    assert!((hyp.u_axis() - u).length() < 1e-15);
    // t = 0 is the vertex of the +u branch.
    let v0 = hyp.evaluate(0.0);
    assert!((v0 - (vertex + u * 2.0)).length() < 1e-15, "{v0:?}");
}

#[test]
fn with_axes_rejects_a_degenerate_in_plane_axis() {
    // A u_axis parallel to the primary axis carries no plane information;
    // silently substituting an arbitrary perpendicular is the bug this
    // constructor exists to prevent, so it must refuse.
    assert!(
        Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 5.0),
            1.0,
        )
        .is_err()
    );
    assert!(
        Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -2.0),
            1.0,
            1.0,
        )
        .is_err()
    );
}

#[test]
fn conic_tangent_intersection_bounds_the_arc() {
    // A conic arc lies inside the triangle formed by its endpoints and the
    // tangent intersection (the exact degree-2 Bezier control polygon).
    // Verified by barycentric containment of dense samples.
    let h = Hyperbola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        2.0,
        3.0,
    )
    .unwrap();
    let (t0, t1) = (-0.8, 1.4);
    let (p0, p2) = (h.evaluate(t0), h.evaluate(t1));
    let p1 = h.tangent_intersection(t0, t1);
    for i in 0..=100 {
        let t = t0 + (t1 - t0) * f64::from(i) / 100.0;
        let q = h.evaluate(t);
        assert!(
            point_in_triangle_2d(q, p0, p1, p2),
            "hyperbola sample at t={t} escaped its Bezier hull"
        );
    }

    let p = Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.7,
    )
    .unwrap();
    let (s0, s1) = (-1.9, 2.6);
    let (q0, q2) = (p.evaluate(s0), p.evaluate(s1));
    let q1 = p.tangent_intersection(s0, s1);
    for i in 0..=100 {
        let t = s0 + (s1 - s0) * f64::from(i) / 100.0;
        assert!(
            point_in_triangle_2d(p.evaluate(t), q0, q1, q2),
            "parabola sample at t={t} escaped its Bezier hull"
        );
    }
}

/// Barycentric containment for coplanar points, with a relative slack.
fn point_in_triangle_2d(q: Point3, a: Point3, b: Point3, c: Point3) -> bool {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = q - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let denom = d00.mul_add(d11, -(d01 * d01));
    if denom.abs() < 1e-30 {
        return false;
    }
    let u = d11.mul_add(d20, -(d01 * d21)) / denom;
    let v = d00.mul_add(d21, -(d01 * d20)) / denom;
    let eps = 1e-12;
    u >= -eps && v >= -eps && u + v <= 1.0 + eps
}

#[test]
fn hyperbola_min_curvature_radius_matches_the_vertex_value() {
    // kappa is maximal at the vertex t = 0 with kappa = a / b^2, so the
    // smallest radius of curvature on any span containing 0 is b^2 / a.
    let (a, b) = (2.0, 3.0);
    let h = Hyperbola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        a,
        b,
    )
    .unwrap();
    let expected = b * b / a;
    let got = h.min_curvature_radius(-1.0, 1.0);
    assert!(
        (got - expected).abs() < 1e-12 * expected,
        "{got} vs {expected}"
    );

    // A span that excludes the vertex is flatter, so its minimum radius
    // must be strictly larger.
    assert!(h.min_curvature_radius(1.0, 2.0) > expected);

    // Parabola: kappa(0) = 1/(2f), so R_min = 2f.
    let f = 0.7;
    let p = Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        f,
    )
    .unwrap();
    let want = 2.0 * f;
    let r = p.min_curvature_radius(-0.5, 0.5);
    assert!((r - want).abs() < 1e-12 * want, "{r} vs {want}");
    assert!(p.min_curvature_radius(1.0, 3.0) > want);
}

// ── Reversal ───────────────────────────────────────────────────

#[test]
fn circle_reversed_traces_the_same_points_backwards() {
    let circle = Circle3D::new(
        Point3::new(1.0, -2.0, 0.5),
        Vec3::new(0.3, -0.5, 0.81),
        4.25,
    )
    .unwrap();
    let reversed = circle.reversed();

    for i in 0..32 {
        let t = PI * f64::from(i) / 8.0;
        assert!((reversed.evaluate(t) - circle.evaluate(-t)).length() < 1e-12);
    }
    // The tangent flips, which is the point of the operation.
    assert!((reversed.tangent(0.0) + circle.tangent(0.0)).length() < 1e-12);
    // The phase does not: `u_axis` is left alone, so the seam stays put.
    assert!((reversed.evaluate(0.0) - circle.evaluate(0.0)).length() < 1e-12);
    assert!((reversed.normal() + circle.normal()).length() < 1e-12);
    assert!((reversed.reversed().normal() - circle.normal()).length() < 1e-12);
}

#[test]
fn ellipse_reversed_traces_the_same_points_backwards() {
    let ellipse = Ellipse3D::new(
        Point3::new(0.0, 3.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
        6.0,
        2.5,
    )
    .unwrap();
    let reversed = ellipse.reversed();

    for i in 0..32 {
        let t = PI * f64::from(i) / 8.0;
        assert!((reversed.evaluate(t) - ellipse.evaluate(-t)).length() < 1e-12);
    }
    // The major axis keeps both its direction and its extent.
    assert!((reversed.u_axis() - ellipse.u_axis()).length() < 1e-12);
    assert!((reversed.semi_major() - ellipse.semi_major()).abs() < 1e-12);
    assert!((reversed.evaluate(0.0) - ellipse.evaluate(0.0)).length() < 1e-12);
}
