//! B10 conic distance / classification cells (`geometry::extrema`).
//!
//! Distance and projection cells over the analytic conic curves
//! (line, circle, ellipse — the three `ParametricCurve` carriers) with
//! independent closed-form oracles, at scales 1e-3/1/1e3. Every cell
//! asserts the DISTANCE against the closed form, the REPORTED POINTS
//! against the oracle positions, and the STATIONARITY condition
//! (residual ⊥ tangent) so a right-distance/wrong-point answer cannot
//! pass.
//!
//! The three former seeds (2026-09-16) are live cells since the B10
//! conic-distance fix (2026-09-25):
//! - parabola and hyperbola now implement `ParametricCurve` (unbounded
//!   domain; solvers take an explicit finite range). Both seeds' oracles
//!   were also wrong: each query lay beyond the vertex's centre of
//!   curvature, where the vertex is a local distance MAXIMUM, so the
//!   cells now pin the closed-form off-axis minima next to a vertex case.
//! - the ellipse twin is exact; the seed read an implicit residual of ~3
//!   because it evaluated the ellipse in a fixed x-major frame while
//!   `Ellipse3D::new` puts the major axis on +Y for a +Z normal.
//! - `point_to_curve` / `curve_to_curve` dropped the curvature term of
//!   the Newton derivative (Gauss-Newton), which is not small at a
//!   distance: the step was 2x too long at an ellipse minor vertex
//!   (orbit), 3x at a hyperbola vertex (divergence), and the Gauss-Newton
//!   matrix is singular for parallel closest tangents (line vs parabola
//!   vertex). They now take a safeguarded full Newton step with exact
//!   carrier derivatives and scale-relative convergence.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_geometry::convert::curve_to_nurbs::line_to_nurbs;
use remus_geometry::extrema::{
    curve_to_curve, line_to_line, point_to_circle, point_to_curve, point_to_line,
};
use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Line3D, Parabola3D};
use remus_math::traits::ParametricCurve;
use remus_math::vec::{Point3, Vec3};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

fn assert_stationarity<C: ParametricCurve>(
    curve: &C,
    t: f64,
    query: Point3,
    band: f64,
    name: &str,
) {
    let p = curve.evaluate(t);
    let tan = curve.tangent(t);
    let diff = p - query;
    let dot = diff.dot(tan);
    let scale = diff.length() * tan.length();
    assert!(
        dot.abs() <= band * scale.max(1e-300),
        "{name}: stationarity violated: dot={dot:.3e} at t={t}",
    );
}

// ── point → line ──────────────────────────────────────────────────────────

#[test]
fn b10_point_line_distance_cells() {
    for scale in SCALES {
        let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        // Query above x=5s: oracle foot (5s,0,0), distance 3s, t=5s.
        let q = Point3::new(5.0 * scale, 3.0 * scale, 0.0);
        let proj = point_to_line(q, &line, 0.0, 10.0 * scale);
        assert!(
            (proj.distance - 3.0 * scale).abs() <= 1e-9 * scale.max(1.0),
            "point-line distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.parameter - 5.0 * scale).abs() <= 1e-9 * scale.max(1.0),
            "point-line parameter @ {scale}: {}",
            proj.parameter,
        );
        assert!(
            (proj.point - Point3::new(5.0 * scale, 0.0, 0.0)).length() <= 1e-9 * scale.max(1.0),
            "point-line foot @ {scale}",
        );
    }
}

// ── point → circle ────────────────────────────────────────────────────────

#[test]
fn b10_point_circle_distance_cells() {
    for scale in SCALES {
        let r = scale;
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        // In-plane query at 5s on +Y: foot (0,r,0), distance 4s.
        let q = Point3::new(0.0, 5.0 * scale, 0.0);
        let proj = point_to_circle(q, &circle);
        assert!(
            (proj.distance - 4.0 * scale).abs() <= 1e-9 * scale.max(1.0),
            "point-circle distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.point - Point3::new(0.0, r, 0.0)).length() <= 1e-9 * scale.max(1.0),
            "point-circle foot @ {scale}: {:?}",
            proj.point,
        );
        // Axial query: equidistant ring — distance is the hypotenuse.
        let axial = point_to_circle(Point3::new(0.0, 0.0, 5.0 * scale), &circle);
        let want = (r * r + 25.0 * scale * scale).sqrt();
        assert!(
            (axial.distance - want).abs() <= 1e-9 * scale.max(1.0),
            "point-circle axial @ {scale}: {} vs {want}",
            axial.distance,
        );
    }
}

// ── line × line (analytic extrema) ────────────────────────────────────────

#[test]
fn b10_line_line_extrema_cells() {
    // Crossing, parallel (gap oracle), skew, and near-tangent (10·tol).
    let l1 = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let l2 = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap();
    let cross = line_to_line(&l1, (-5.0, 5.0), &l2, (-5.0, 5.0));
    assert!(cross.distance < 1e-12, "crossing lines: {}", cross.distance);

    for scale in SCALES {
        // Parallel at gap s: distance exactly s.
        let p1 = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let p2 = Line3D::new(Point3::new(0.0, scale, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let sol = line_to_line(&p1, (0.0, 10.0 * scale), &p2, (0.0, 10.0 * scale));
        assert!(
            (sol.distance - scale).abs() <= 1e-9 * scale,
            "parallel gap @ {scale}: {}",
            sol.distance,
        );
        // Skew: L1 along X at z=0, L2 along Y at z=s → distance s,
        // feet (0,0,0)/(0,0,s).
        let s1 = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let s2 = Line3D::new(Point3::new(0.0, 0.0, scale), Vec3::new(0.0, 1.0, 0.0)).unwrap();
        let skew = line_to_line(
            &s1,
            (-5.0 * scale, 5.0 * scale),
            &s2,
            (-5.0 * scale, 5.0 * scale),
        );
        assert!(
            (skew.distance - scale).abs() <= 1e-9 * scale.max(1e-12),
            "skew distance @ {scale}: {}",
            skew.distance,
        );
        assert!(
            (skew.point_a - Point3::new(0.0, 0.0, 0.0)).length() <= 1e-9 * scale.max(1.0),
            "skew foot A @ {scale}",
        );
        assert!(
            (skew.point_b - Point3::new(0.0, 0.0, scale)).length() <= 1e-9 * scale.max(1.0),
            "skew foot B @ {scale}",
        );
        // Near-tangent within 10·tol: parallel gap 10·tol·s resolves exactly.
        let gap = 10.0 * 1e-7 * scale;
        let n2 = Line3D::new(Point3::new(0.0, gap, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let near = line_to_line(&p1, (0.0, 10.0 * scale), &n2, (0.0, 10.0 * scale));
        assert!(
            (near.distance - gap).abs() <= 1e-12 * scale.max(1.0),
            "near-tangent gap @ {scale}: {} vs {gap}",
            near.distance,
        );
    }
}

// ── circle × circle (generic extrema vs closed form) ──────────────────────

#[test]
fn b10_circle_circle_extrema_cells() {
    for scale in SCALES {
        // Concentric, radii s and 2s: distance exactly s (closed form).
        let c1 =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), scale).unwrap();
        let c2 = Circle3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0 * scale,
        )
        .unwrap();
        let sol = curve_to_curve(&c1, (0.0, TAU), &c2, (0.0, TAU));
        assert!(
            (sol.distance - scale).abs() <= 1e-6 * scale.max(1.0),
            "concentric circles @ {scale}: {}",
            sol.distance,
        );
        // Coaxial stack, both radius s, planes s apart... axial gap 3s:
        // distance exactly 3s, feet share the circle angle.
        let d1 =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), scale).unwrap();
        let d2 = Circle3D::new(
            Point3::new(0.0, 0.0, 3.0 * scale),
            Vec3::new(0.0, 0.0, 1.0),
            scale,
        )
        .unwrap();
        let stacked = curve_to_curve(&d1, (0.0, TAU), &d2, (0.0, TAU));
        assert!(
            (stacked.distance - 3.0 * scale).abs() <= 1e-6 * scale.max(1.0),
            "stacked circles @ {scale}: {}",
            stacked.distance,
        );
        assert_stationarity(&d1, stacked.param_a, stacked.point_b, 1e-4, "stacked-a");
        assert_stationarity(&d2, stacked.param_b, stacked.point_a, 1e-4, "stacked-b");
    }
}

// ── ellipse extrema (analytic carrier) ────────────────────────────────────

#[test]
fn b10_ellipse_point_distance_cells() {
    // Oracle (closed form, independent): the query (5s,0,0) lies on the
    // minor axis; the closest ellipse point is the minor-axis vertex
    // (s,0,0) with distance 4s. (With `Ellipse3D::new`'s axis choice for
    // +Z normal, u=(0,1,0) carries a=2s and v=(-1,0,0) carries b=s, so
    // the vertex is at angle 3π/2, foot (s,0,0).)
    //
    // Before B10's fix `point_to_curve` stalled here (t=4.6868 short of
    // 4.7124, d=4.00065s, stationarity dot=-0.2 at scale 1): its
    // Gauss-Newton step dropped the curvature term, which at a minor-axis
    // vertex seen from 4 radii equals the kept term, so every step was
    // twice too long and the iterate orbited the minimum. The solver now
    // takes the full Newton step; the cell pins the closed form and
    // stationarity at every scale.
    for scale in SCALES {
        let e = Ellipse3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0 * scale,
            scale,
        )
        .unwrap();
        let q = Point3::new(5.0 * scale, 0.0, 0.0);
        let proj = point_to_curve(q, &e, 0.0, TAU);
        assert!(
            (proj.distance - 4.0 * scale).abs() <= 1e-12 * scale,
            "ellipse point distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.point - Point3::new(scale, 0.0, 0.0)).length() <= 1e-9 * scale,
            "ellipse point foot @ {scale}: {:?}",
            proj.point,
        );
        assert_stationarity(&e, proj.parameter, q, 1e-9, "ellipse-minor-vertex");
    }
}

// ── parabola / hyperbola extrema ──────────────────────────────────────────

/// Parabola `P(t) = (t, t²/(4f), 0)` with vertex at the origin, axis +Y,
/// in-plane axis +X and focal length `f` (`t` carries units of length).
fn parabola(f: f64) -> Parabola3D {
    Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        f,
    )
    .unwrap()
}

/// Right branch `H(t) = (a·cosh t, b·sinh t, 0)` centred at the origin.
fn hyperbola(a: f64, b: f64) -> Hyperbola3D {
    Hyperbola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        a,
        b,
    )
    .unwrap()
}

#[test]
fn b10_parabola_extrema_cells() {
    // Closed form for a query (0, y0) on the axis of y = x²/(4f):
    // d²(t) = t² + (t²/4f − y0)² has d/dt = t·(t²/(8f²) + 1 − y0/f)·2,
    // so the vertex is the unique minimum while y0 <= 2f (the vertex's
    // centre of curvature) and becomes a local MAXIMUM beyond it, where
    // the minima move to t = ±2·sqrt(f·(y0 − 2f)).
    //
    // The original seed expected the vertex for y0 = 4f; the closed form
    // puts the minima at t = ±2√2·f, foot (±2√2·f, 2f), distance 2√3·f.
    for scale in SCALES {
        let f = scale;
        let par = parabola(f);
        let range = (-10.0 * scale, 10.0 * scale);

        // Inside the curvature circle: the vertex, distance y0.
        let q = Point3::new(0.0, 1.5 * f, 0.0);
        let proj = point_to_curve(q, &par, range.0, range.1);
        assert!(
            (proj.distance - 1.5 * f).abs() <= 1e-9 * scale,
            "parabola vertex distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.point - Point3::new(0.0, 0.0, 0.0)).length() <= 1e-6 * scale,
            "parabola vertex foot @ {scale}"
        );
        assert_stationarity(&par, proj.parameter, q, 1e-9, "parabola-vertex");

        // Beyond it: two symmetric off-axis minima.
        let q = Point3::new(0.0, 4.0 * f, 0.0);
        let proj = point_to_curve(q, &par, range.0, range.1);
        let t_star = 2.0 * 2.0_f64.sqrt() * f;
        assert!(
            (proj.distance - 2.0 * 3.0_f64.sqrt() * f).abs() <= 1e-9 * scale,
            "parabola off-axis distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.parameter.abs() - t_star).abs() <= 1e-6 * scale,
            "parabola off-axis foot @ {scale}: t={}",
            proj.parameter,
        );
        assert!(
            (proj.point - Point3::new(proj.parameter.signum() * t_star, 2.0 * f, 0.0)).length()
                <= 1e-6 * scale,
            "parabola off-axis foot position @ {scale}: {:?}",
            proj.point,
        );
        assert_stationarity(&par, proj.parameter, q, 1e-9, "parabola-off-axis");

        // Line y = −f (parallel to the vertex tangent) vs the parabola:
        // the gap is t²/(4f) + f, minimal at the vertex, distance f.
        let line = line_to_nurbs(
            Point3::new(-10.0 * scale, -f, 0.0),
            Point3::new(10.0 * scale, -f, 0.0),
        )
        .unwrap();
        let sol = curve_to_curve(&par, range, &line, line.domain());
        assert!(
            (sol.distance - f).abs() <= 1e-9 * scale,
            "parabola-line distance @ {scale}: {}",
            sol.distance,
        );
        assert!(
            (sol.point_a - Point3::new(0.0, 0.0, 0.0)).length() <= 1e-6 * scale,
            "parabola-line foot @ {scale}"
        );
    }
}

#[test]
fn b10_hyperbola_extrema_cells() {
    // Closed form for a query (q, 0) on the real axis of the right branch
    // (a·cosh t, b·sinh t): d/dt of d² is 2·sinh t·((a² + b²)·cosh t − a·q),
    // so the vertex is the unique minimum while a·q <= a² + b² and the
    // minima move to cosh t = a·q/(a² + b²) beyond it.
    //
    // The original seed expected the vertex (distance 3) for a=2, b=1,
    // q=5; the closed form puts the minima at cosh t = 2, foot (4, ±√3),
    // distance 2.
    for scale in SCALES {
        let (a, b) = (2.0 * scale, scale);
        let hyp = hyperbola(a, b);
        let range = (-3.0, 3.0);

        // Vertex case: q = 1·s, vertex (2s, 0), distance s.
        let q = Point3::new(scale, 0.0, 0.0);
        let proj = point_to_curve(q, &hyp, range.0, range.1);
        assert!(
            (proj.distance - scale).abs() <= 1e-9 * scale,
            "hyperbola vertex distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.point - Point3::new(a, 0.0, 0.0)).length() <= 1e-6 * scale,
            "hyperbola vertex foot @ {scale}: {:?}",
            proj.point,
        );
        assert_stationarity(&hyp, proj.parameter, q, 1e-9, "hyperbola-vertex");

        // Off-axis case: q = 5s, minima at cosh t = 2, distance 2s.
        let q = Point3::new(5.0 * scale, 0.0, 0.0);
        let proj = point_to_curve(q, &hyp, range.0, range.1);
        let t_star = 2.0_f64.acosh();
        assert!(
            (proj.distance - 2.0 * scale).abs() <= 1e-9 * scale,
            "hyperbola off-axis distance @ {scale}: {}",
            proj.distance,
        );
        assert!(
            (proj.parameter.abs() - t_star).abs() <= 1e-7,
            "hyperbola off-axis parameter @ {scale}: {}",
            proj.parameter,
        );
        assert!(
            (proj.point
                - Point3::new(
                    4.0 * scale,
                    proj.parameter.signum() * 3.0_f64.sqrt() * scale,
                    0.0
                ))
            .length()
                <= 1e-6 * scale,
            "hyperbola off-axis foot @ {scale}: {:?}",
            proj.point,
        );
        assert_stationarity(&hyp, proj.parameter, q, 1e-9, "hyperbola-off-axis");
    }
}

// ── twin parity (positions after projection + tangent/curvature) ──────────

/// Closed-form curvature of `(a·cos t, b·sin t)`.
fn ellipse_curvature(a: f64, b: f64, t: f64) -> f64 {
    let (s, c) = t.sin_cos();
    a * b / (a * a * s * s + b * b * c * c).powf(1.5)
}

#[test]
fn b10_conic_twin_parity_cells() {
    // Per the roadmap lesson, twin comparison asserts POSITIONS after
    // projection plus tangent direction and curvature, never parameter
    // speed. The oracle works in the ellipse's own frame (`u_axis`
    // carries a, `v_axis` carries b): the original seed measured the
    // implicit residual with a fixed x-major frame, but `Ellipse3D::new`
    // puts the major axis on +Y for a +Z normal, so it read a residual of
    // ~3 on an exact twin.
    use remus_geometry::convert::curve_to_nurbs::ellipse_to_nurbs;
    for scale in SCALES {
        let (a, b) = (2.0 * scale, scale);
        let e = Ellipse3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), a, b).unwrap();
        let twin = ellipse_to_nurbs(&e, 0.0, TAU).unwrap();
        let (t0, t1) = twin.domain();
        let frame = |p: Point3| {
            let v = p - e.center();
            (v.dot(e.u_axis()), v.dot(e.v_axis()))
        };

        // 1. The twin lies on the ellipse.
        let mut max_resid = 0.0_f64;
        for i in 0..=256 {
            let u = f64::from(i).mul_add((t1 - t0) / 256.0, t0);
            let (x, y) = frame(twin.evaluate(u));
            max_resid = max_resid.max(((x / a).powi(2) + (y / b).powi(2) - 1.0).abs());
        }
        assert!(
            max_resid <= 1e-12,
            "twin off-ellipse @ {scale}: residual {max_resid:.3e}"
        );

        // 2. Projections agree in position, distance, tangent direction and
        //    curvature, including from the minor-axis query the Gauss-Newton
        //    step used to orbit on (4 radii off the minor vertex).
        let queries = [
            e.center() + e.v_axis() * (5.0 * scale),
            e.center() + e.u_axis() * (3.0 * scale) + e.v_axis() * (0.7 * scale),
            e.center() + e.u_axis() * (-0.4 * scale) + e.v_axis() * (-2.5 * scale),
            e.center() + e.u_axis() * (0.3 * scale) + e.v_axis() * (0.2 * scale),
        ];
        for q in queries {
            let on_e = point_to_curve(q, &e, 0.0, TAU);
            let on_twin = point_to_curve(q, &twin, t0, t1);
            let name = format!("twin parity @ scale {scale} query {q:?}");
            assert!(
                (on_e.point - on_twin.point).length() <= 1e-9 * scale,
                "{name}: feet differ by {:.3e}",
                (on_e.point - on_twin.point).length(),
            );
            assert!(
                (on_e.distance - on_twin.distance).abs() <= 1e-9 * scale,
                "{name}: distance"
            );
            assert_stationarity(&e, on_e.parameter, q, 1e-9, &name);
            assert_stationarity(&twin, on_twin.parameter, q, 1e-9, &name);
            let (te, tt) = (
                e.tangent(on_e.parameter).normalize().unwrap(),
                ParametricCurve::tangent(&twin, on_twin.parameter)
                    .normalize()
                    .unwrap(),
            );
            assert!(
                te.cross(tt).length() <= 1e-9,
                "{name}: tangent directions differ"
            );
            let k_e = ellipse_curvature(a, b, on_e.parameter);
            let k_t = twin.curvature(on_twin.parameter).unwrap();
            assert!(
                (k_e - k_t).abs() <= 1e-7 * k_e,
                "{name}: curvature {k_e} vs {k_t}"
            );
        }
    }
}
