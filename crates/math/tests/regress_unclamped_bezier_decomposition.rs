//! Bezier decomposition and degree elevation of unclamped NURBS (kernel
//! review 2026-10-14, MATH-4).
//!
//! `curve_to_bezier_segments` and `surface_to_bezier_patches` brought every
//! interior knot to multiplicity `p` and then read each window of `p + 1`
//! control points as a Bezier segment. That holds only where the domain ends
//! also carry multiplicity `p`; on an unclamped knot vector the control
//! points at the ends are de Boor points, not Bezier points, so every
//! segment touching an end was a different curve. `curve_degree_elevate`
//! inherits the error. Both now clamp to the domain first.
//!
//! The oracle is the closed-form conversion of a uniform cubic B-spline span
//! to Bezier form: on a span with unit knot spacing and de Boor points
//! `P0..P3`,
//!
//! ```text
//! B0 = (P0 + 4 P1 + P2) / 6      B1 = (4 P1 + 2 P2) / 6
//! B2 = (2 P1 + 4 P2) / 6         B3 = (P1 + 4 P2 + P3) / 6
//! ```
//!
//! and a surface patch is that matrix applied along both directions.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

use remus_math::MathError;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::decompose::{
    curve_degree_elevate, curve_to_bezier_segments, surface_to_bezier_patches,
};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;

/// Uniform cubic B-spline span → Bezier, rows sum to 1.
const M: [[f64; 4]; 4] = [
    [1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0, 0.0],
    [0.0, 4.0 / 6.0, 2.0 / 6.0, 0.0],
    [0.0, 2.0 / 6.0, 4.0 / 6.0, 0.0],
    [0.0, 1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0],
];

fn combine(weights: &[f64], points: &[Point3]) -> Point3 {
    let mut acc = [0.0; 3];
    for (w, p) in weights.iter().zip(points) {
        acc[0] += w * p.x();
        acc[1] += w * p.y();
        acc[2] += w * p.z();
    }
    Point3::new(acc[0], acc[1], acc[2])
}

fn bezier_from_uniform(de_boor: &[Point3]) -> Vec<Point3> {
    M.iter().map(|row| combine(row, de_boor)).collect()
}

fn uniform_knots(count: usize) -> Vec<f64> {
    (0..count).map(|i| i as f64).collect()
}

fn de_boor_points(n: usize) -> Vec<Point3> {
    // A non-symmetric, non-planar polygon so no coordinate hides an error.
    (0..n)
        .map(|i| {
            let t = i as f64;
            Point3::new(t, (t * 1.7).sin() * 2.0 + t, (t * t) * 0.25 - 1.0)
        })
        .collect()
}

fn unclamped_cubic(n: usize) -> NurbsCurve {
    NurbsCurve::new(3, uniform_knots(n + 4), de_boor_points(n), vec![1.0; n]).unwrap()
}

fn assert_close(a: Point3, b: Point3, tol: f64, what: &str) {
    let dev = (a - b).length();
    assert!(
        dev <= tol,
        "{what}: deviation {dev:e} > {tol:e} ({a:?} vs {b:?})"
    );
}

/// Largest deviation between `a` and `b` over `samples` + 1 parameters in
/// `a`'s domain, evaluated by each curve's own de Boor evaluation.
fn max_deviation(a: &NurbsCurve, b: &NurbsCurve, samples: usize) -> f64 {
    let (lo, hi) = a.domain();
    (0..=samples)
        .map(|i| {
            let t = lo + (hi - lo) * (i as f64) / (samples as f64);
            (a.evaluate(t) - b.evaluate(t)).length()
        })
        .fold(0.0, f64::max)
}

#[test]
fn single_span_unclamped_cubic_decomposes_to_its_closed_form_bezier() {
    let curve = unclamped_cubic(4);
    assert_eq!(curve.domain(), (3.0, 4.0));
    let segments = curve_to_bezier_segments(&curve).unwrap();
    assert_eq!(segments.len(), 1);
    let segment = &segments[0];
    assert_eq!(segment.knots(), &[3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0]);
    let expected = bezier_from_uniform(curve.control_points());
    for (got, want) in segment.control_points().iter().zip(&expected) {
        assert_close(*got, *want, 1e-14, "Bezier point");
    }
    assert!(max_deviation(&curve, segment, 64) <= 1e-13);
}

#[test]
fn multi_span_unclamped_cubic_decomposes_span_by_span() {
    // Six de Boor points on knots 0..=9: three spans over [3, 6]; span j's
    // Bezier points come from de Boor points j..j+4 alone.
    let curve = unclamped_cubic(6);
    assert_eq!(curve.domain(), (3.0, 6.0));
    let segments = curve_to_bezier_segments(&curve).unwrap();
    assert_eq!(segments.len(), 3);
    for (j, segment) in segments.iter().enumerate() {
        let start = 3.0 + j as f64;
        assert_eq!(segment.domain(), (start, start + 1.0));
        let expected = bezier_from_uniform(&curve.control_points()[j..j + 4]);
        for (got, want) in segment.control_points().iter().zip(&expected) {
            assert_close(*got, *want, 1e-13, "Bezier point");
        }
        for i in 0..=16 {
            let t = start + f64::from(i) / 16.0;
            assert_close(segment.evaluate(t), curve.evaluate(t), 1e-13, "segment");
        }
    }
}

#[test]
fn degree_elevated_unclamped_cubic_is_the_same_curve() {
    // The review's repro: main's elevated curve deviated by about 2 here.
    for n in [4, 6] {
        let curve = unclamped_cubic(n);
        for t in [1, 2] {
            let elevated = curve_degree_elevate(&curve, t).unwrap();
            assert_eq!(elevated.degree(), 3 + t);
            assert_eq!(elevated.domain(), curve.domain());
            let dev = max_deviation(&curve, &elevated, 96);
            assert!(dev <= 1e-12, "n = {n}, t = {t}: elevated deviates {dev:e}");
        }
    }
}

#[test]
fn degree_elevated_single_span_matches_closed_form_bezier_elevation() {
    // Elevating a Bezier by one: Q_i = (i/4) B_{i-1} + (1 - i/4) B_i.
    let curve = unclamped_cubic(4);
    let b = bezier_from_uniform(curve.control_points());
    let elevated = curve_degree_elevate(&curve, 1).unwrap();
    assert_eq!(
        elevated.knots(),
        &[3.0, 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0, 4.0]
    );
    let q = elevated.control_points();
    assert_eq!(q.len(), 5);
    for i in 0..5 {
        let a = i as f64 / 4.0;
        let want = match i {
            0 => b[0],
            4 => b[3],
            _ => combine(&[a, 1.0 - a], &[b[i - 1], b[i]]),
        };
        assert_close(q[i], want, 1e-14, "elevated point");
    }
}

#[test]
fn rational_unclamped_curve_keeps_its_geometry() {
    let n = 5;
    let weights = vec![1.0, 2.5, 0.75, 1.5, 1.0];
    let curve = NurbsCurve::new(3, uniform_knots(n + 4), de_boor_points(n), weights).unwrap();
    let segments = curve_to_bezier_segments(&curve).unwrap();
    assert_eq!(segments.len(), 2);
    for segment in &segments {
        assert!(max_deviation(segment, &curve, 32) <= 1e-13);
    }
    let elevated = curve_degree_elevate(&curve, 2).unwrap();
    assert!(max_deviation(&curve, &elevated, 96) <= 1e-12);
}

#[test]
fn end_multiplicity_p_keeps_the_input_control_points() {
    // Ends at multiplicity exactly p with one outer knot each: already
    // interpolating, so the Bezier points are the stored control points.
    let points = de_boor_points(4);
    let curve = NurbsCurve::new(
        3,
        vec![-1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0],
        points.clone(),
        vec![1.0; 4],
    )
    .unwrap();
    let segments = curve_to_bezier_segments(&curve).unwrap();
    assert_eq!(segments.len(), 1);
    assert_eq!(segments[0].control_points(), points.as_slice());
    assert_eq!(
        segments[0].knots(),
        &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
    );
}

fn grid(rows: usize, cols: usize) -> Vec<Vec<Point3>> {
    (0..rows)
        .map(|i| {
            (0..cols)
                .map(|j| {
                    let (u, v) = (i as f64, j as f64);
                    Point3::new(u, v, (u * 0.9).sin() + (v * 1.3).cos() + 0.1 * u * v)
                })
                .collect()
        })
        .collect()
}

/// Bezier patch of a uniform bicubic span: `M · P · Mᵀ`, or `P · Mᵀ` /
/// `M · P` when one direction is already Bezier.
fn patch_from_grid(points: &[Vec<Point3>], convert_u: bool, convert_v: bool) -> Vec<Vec<Point3>> {
    let identity = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    let mu = if convert_u { M } else { identity };
    let mv = if convert_v { M } else { identity };
    let rows: Vec<Vec<Point3>> = points
        .iter()
        .map(|row| mv.iter().map(|r| combine(r, row)).collect())
        .collect();
    (0..4)
        .map(|i| {
            (0..4)
                .map(|j| {
                    let column: Vec<Point3> = rows.iter().map(|row| row[j]).collect();
                    combine(&mu[i], &column)
                })
                .collect()
        })
        .collect()
}

fn assert_patch(surface: &NurbsSurface, convert_u: bool, convert_v: bool) {
    let patches = surface_to_bezier_patches(surface).unwrap();
    assert_eq!(patches.len(), 1);
    let patch = &patches[0];
    assert_eq!(patch.u_range, surface.domain_u());
    assert_eq!(patch.v_range, surface.domain_v());
    let expected = patch_from_grid(surface.control_points(), convert_u, convert_v);
    for (got_row, want_row) in patch.surface.control_points().iter().zip(&expected) {
        for (got, want) in got_row.iter().zip(want_row) {
            assert_close(*got, *want, 1e-13, "patch point");
        }
    }
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    for i in 0..=8 {
        for j in 0..=8 {
            let u = u0 + (u1 - u0) * f64::from(i) / 8.0;
            let v = v0 + (v1 - v0) * f64::from(j) / 8.0;
            assert_close(
                patch.surface.evaluate(u, v),
                surface.evaluate(u, v),
                1e-13,
                "patch surface",
            );
        }
    }
}

#[test]
fn unclamped_bicubic_patch_is_the_closed_form_tensor_product() {
    let surface = NurbsSurface::new(
        3,
        3,
        uniform_knots(8),
        uniform_knots(8),
        grid(4, 4),
        vec![vec![1.0; 4]; 4],
    )
    .unwrap();
    assert_patch(&surface, true, true);
}

#[test]
fn surface_clamped_in_u_and_unclamped_in_v_converts_only_v() {
    let surface = NurbsSurface::new(
        3,
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        uniform_knots(8),
        grid(4, 4),
        vec![vec![1.0; 4]; 4],
    )
    .unwrap();
    assert_patch(&surface, false, true);

    let transposed = NurbsSurface::new(
        3,
        3,
        uniform_knots(8),
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        grid(4, 4),
        vec![vec![1.0; 4]; 4],
    )
    .unwrap();
    assert_patch(&transposed, true, false);
}

#[test]
fn multi_span_unclamped_surface_patches_reproduce_the_surface() {
    let surface = NurbsSurface::new(
        3,
        3,
        uniform_knots(10),
        uniform_knots(9),
        grid(6, 5),
        vec![vec![1.0; 5]; 6],
    )
    .unwrap();
    let patches = surface_to_bezier_patches(&surface).unwrap();
    assert_eq!(patches.len(), 3 * 2);
    for patch in &patches {
        let ((u0, u1), (v0, v1)) = (patch.u_range, patch.v_range);
        for i in 0..=4 {
            for j in 0..=4 {
                let u = u0 + (u1 - u0) * f64::from(i) / 4.0;
                let v = v0 + (v1 - v0) * f64::from(j) / 4.0;
                assert_close(
                    patch.surface.evaluate(u, v),
                    surface.evaluate(u, v),
                    1e-12,
                    "patch surface",
                );
            }
        }
    }
}

#[test]
fn start_multiplicity_above_degree_is_refused_as_on_the_clamped_path() {
    // Review follow-up: degree 3 with knots [-1, 0, 0, 0, 0, 1, 1, 1, 1]
    // leaves the first span [knots[3], knots[4]] = [0, 0] empty, so the
    // curve has no value at its domain start. Main refused it at that span
    // (`InvalidKnotValue { index: 4 }`); clamping must not drop a side and
    // accept it instead.
    let points = de_boor_points(5);
    let knots = vec![-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let curve = NurbsCurve::new(3, knots.clone(), points, vec![1.0; 5]).unwrap();
    for result in [
        curve_to_bezier_segments(&curve).map(|_| ()),
        curve_degree_elevate(&curve, 1).map(|_| ()),
    ] {
        assert!(
            matches!(
                result,
                Err(MathError::InvalidKnotValue { index: 4, value }) if value == 0.0
            ),
            "{result:?}"
        );
    }
    let surface = NurbsSurface::new(
        3,
        3,
        knots,
        uniform_knots(8),
        grid(5, 4),
        vec![vec![1.0; 4]; 5],
    )
    .unwrap();
    assert!(matches!(
        surface_to_bezier_patches(&surface),
        Err(MathError::InvalidKnotValue { index: 4, .. })
    ));
}
