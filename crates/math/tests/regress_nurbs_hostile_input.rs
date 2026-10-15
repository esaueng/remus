//! NURBS construction and evaluation stay panic-free on degenerate input.
//!
//! Three paths reachable from a STEP file or a WASM call used to panic or
//! return NaN:
//! - `NurbsCurve2D::new` checked only vector lengths, so a degree-3 curve with
//!   two poles (a STEP 2D B-spline pcurve) was accepted and `evaluate` then
//!   underflowed `span - degree`;
//! - an end knot repeated more than `degree + 1` times passes construction,
//!   and `find_span` returned the empty last span, so evaluation at the domain
//!   end divided zero by zero;
//! - degree elevation built its coefficients from a `usize` binomial that
//!   overflows (degree 31 on wasm32, ~63 natively) and panicked.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]

use remus_math::MathError;
use remus_math::curves2d::NurbsCurve2D;
use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::decompose::curve_degree_elevate;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point2, Point3};

fn cubic() -> NurbsCurve {
    NurbsCurve::new(
        3,
        vec![0., 0., 0., 0., 1., 1., 1., 1.],
        vec![
            Point3::new(0., 0., 0.),
            Point3::new(1., 2., 0.),
            Point3::new(3., 2., 0.),
            Point3::new(4., 0., 0.),
        ],
        vec![1.; 4],
    )
    .unwrap()
}

#[test]
fn nurbs2d_refuses_degree_not_below_pole_count() {
    let result = NurbsCurve2D::new(
        3,
        vec![0., 0., 0., 1., 1., 1.],
        vec![Point2::new(0., 0.), Point2::new(1., 0.)],
        vec![1., 1.],
    );
    assert!(
        matches!(
            result,
            Err(MathError::InvalidDegree {
                degree: 3,
                control_points: 2
            })
        ),
        "{result:?}"
    );
}

#[test]
fn nurbs2d_huge_degree_is_refused_before_the_knot_count() {
    // `n + degree + 1` overflows for this degree, so the degree check has to
    // run before the knot count is computed.
    let result = std::panic::catch_unwind(|| {
        NurbsCurve2D::new(
            usize::MAX,
            vec![0., 0., 1., 1.],
            vec![Point2::new(0., 0.), Point2::new(1., 0.)],
            vec![1., 1.],
        )
    });
    assert!(
        matches!(
            result,
            Ok(Err(MathError::InvalidDegree {
                degree: usize::MAX,
                control_points: 2
            }))
        ),
        "{result:?}"
    );
}

#[test]
fn nurbs2d_refuses_bad_knots_and_weights() {
    let points = vec![
        Point2::new(0., 0.),
        Point2::new(1., 0.),
        Point2::new(2., 1.),
    ];
    let decreasing = NurbsCurve2D::new(1, vec![0., 0., 1., 0.5, 0.5], points.clone(), vec![1.; 3]);
    assert!(decreasing.is_err(), "{decreasing:?}");
    let collapsed = NurbsCurve2D::new(1, vec![0., 0., 0., 0., 0.], points.clone(), vec![1.; 3]);
    assert!(collapsed.is_err(), "{collapsed:?}");
    let zero_weight = NurbsCurve2D::new(1, vec![0., 0., 0.5, 1., 1.], points, vec![1., 0., 1.]);
    assert!(zero_weight.is_err(), "{zero_weight:?}");
}

#[test]
fn over_repeated_end_knots_evaluate_finitely() {
    // p = 2 with the end knot repeated four times: the last pole has no
    // support, but the curve is still the quadratic through the first three.
    let curve = NurbsCurve::new(
        2,
        vec![0., 0., 0., 1., 1., 1., 1.],
        vec![
            Point3::new(0., 0., 0.),
            Point3::new(1., 1., 0.),
            Point3::new(2., 0., 0.),
            Point3::new(3., 0., 0.),
        ],
        vec![1.; 4],
    )
    .unwrap();
    let (a, b) = curve.domain();
    let start = curve.evaluate(a);
    let end = curve.evaluate(b);
    assert!(
        (start - Point3::new(0., 0., 0.)).length() < 1e-12,
        "{start:?}"
    );
    assert!((end - Point3::new(2., 0., 0.)).length() < 1e-12, "{end:?}");

    // The same at the start, and on a surface.
    let leading = NurbsCurve::new(
        2,
        vec![0., 0., 0., 0., 1., 1., 1.],
        vec![
            Point3::new(-1., 0., 0.),
            Point3::new(0., 0., 0.),
            Point3::new(1., 1., 0.),
            Point3::new(2., 0., 0.),
        ],
        vec![1.; 4],
    )
    .unwrap();
    let p = leading.evaluate(leading.domain().0);
    assert!((p - Point3::new(0., 0., 0.)).length() < 1e-12, "{p:?}");

    let surface = NurbsSurface::new(
        1,
        2,
        vec![0., 0., 1., 1.],
        vec![0., 0., 0., 1., 1., 1., 1.],
        vec![
            vec![
                Point3::new(0., 0., 0.),
                Point3::new(1., 0., 1.),
                Point3::new(2., 0., 0.),
                Point3::new(3., 0., 0.),
            ],
            vec![
                Point3::new(0., 1., 0.),
                Point3::new(1., 1., 1.),
                Point3::new(2., 1., 0.),
                Point3::new(3., 1., 0.),
            ],
        ],
        vec![vec![1.; 4]; 2],
    )
    .unwrap();
    let corner = surface.evaluate(1.0, 1.0);
    assert!(
        (corner - Point3::new(2., 1., 0.)).length() < 1e-12,
        "{corner:?}"
    );
}

#[test]
fn degree_elevation_past_binomial_range_is_refused_not_a_panic() {
    let c = cubic();
    let result = std::panic::catch_unwind(|| curve_degree_elevate(&c, 60));
    let result = result.expect("degree elevation panicked");
    assert!(
        matches!(result, Err(MathError::InvalidDegree { .. })),
        "{result:?}"
    );
}

#[test]
fn degree_elevation_in_range_is_unchanged_geometry() {
    // Degree 31 overflowed `usize` on wasm32; with exact u64 binomials it is
    // an ordinary elevation.
    let c = cubic();
    let e = curve_degree_elevate(&c, 28).unwrap();
    assert_eq!(e.degree(), 31);
    for i in 0..=8 {
        let t = f64::from(i) / 8.0;
        let d = (e.evaluate(t) - c.evaluate(t)).length();
        assert!(d < 1e-9, "deviation {d} at t = {t}");
    }
}

#[test]
fn huge_degree_elevation_is_refused_before_allocating() {
    // The coefficient table for this many degrees could never be allocated;
    // the binomial range check has to refuse it first.
    let c = cubic();
    let result = std::panic::catch_unwind(|| curve_degree_elevate(&c, usize::MAX / 4));
    let result = result.expect("degree elevation panicked");
    assert!(
        matches!(result, Err(MathError::InvalidDegree { .. })),
        "{result:?}"
    );
}
