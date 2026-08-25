//! Panic-freedom pins for the NURBS knot operations.
//!
//! On `wasm32-unknown-unknown` the panic strategy is `abort`: a panic inside
//! any kernel method traps mid-call and leaves the wasm-bindgen borrow flag
//! on `BrepKernel` set, so every later call throws "recursive use of an
//! object" and the only recovery is a new kernel (see
//! `crates/wasm/src/panics.rs`). A panic is therefore not a recoverable
//! error here — it destroys the session. These operations are reachable from
//! JS (`curveSplit`, `curveKnotRemove`), from STEP import, from the GFA
//! boolean, and from sweep, so they must be total.
//!
//! Each case below panicked before the domain guards existed:
//!
//! | call | mechanism |
//! | --- | --- |
//! | `curve_split(c, u_max)` | `cps[..=last_u - p]` past the end |
//! | `curve_knot_remove(c, u_min)` | `pw[k + 1]` past the end |
//! | `curve_knot_remove(c, u_max)` | `pw[k - p]` underflows |
//! | `curve_to_bezier_segments(c^-1)` | `p - mult` underflows |
//!
//! None of these are adversarial inputs: `u_min` and `u_max` are the curve's
//! own domain endpoints, the most natural values a caller has to hand.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::MathError;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::decompose::{curve_degree_elevate, curve_to_bezier_segments};
use remus_math::nurbs::knot_ops::{curve_knot_insert, curve_knot_remove, curve_split};
use remus_math::vec::Point3;

fn cubic_bezier() -> NurbsCurve {
    NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        (0..4)
            .map(|i| Point3::new(f64::from(i), f64::from(i % 2), 0.0))
            .collect(),
        vec![1.0; 4],
    )
    .unwrap()
}

fn quadratic_two_span() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        (0..4)
            .map(|i| Point3::new(f64::from(i), f64::from(i % 2), 0.0))
            .collect(),
        vec![1.0; 4],
    )
    .unwrap()
}

fn curves() -> Vec<NurbsCurve> {
    vec![cubic_bezier(), quadratic_two_span()]
}

#[test]
fn split_refuses_the_domain_endpoints() {
    for c in curves() {
        let (lo, hi) = c.domain();
        for u in [lo, hi] {
            assert!(
                matches!(
                    curve_split(&c, u),
                    Err(MathError::ParameterOutOfRange { .. })
                ),
                "split at endpoint {u} must be refused, not panic"
            );
        }
    }
}

#[test]
fn split_refuses_outside_the_domain() {
    for c in curves() {
        for u in [-1.0, -1e-9, 1.0 + 1e-9, 2.0, 1e12, -1e12] {
            assert!(
                matches!(
                    curve_split(&c, u),
                    Err(MathError::ParameterOutOfRange { .. })
                ),
                "split outside the domain at {u} must be refused"
            );
        }
    }
}

#[test]
fn split_still_works_strictly_inside() {
    for c in curves() {
        for u in [0.1, 0.25, 0.5, 0.75, 0.9] {
            let (left, right) = curve_split(&c, u).expect("interior split still works");
            // The halves must partition the domain and be joinable at u.
            assert!((left.domain().1 - u).abs() < 1e-12);
            assert!((right.domain().0 - u).abs() < 1e-12);
            let join = (left.evaluate(u) - right.evaluate(u)).length();
            assert!(join < 1e-9, "halves must meet at u={u}, gap {join:e}");
            // Each half must reproduce the original.
            for f in [0.25_f64, 0.5, 0.75] {
                let ul = left.domain().0 + (left.domain().1 - left.domain().0) * f;
                let dev = (left.evaluate(ul) - c.evaluate(ul)).length();
                assert!(dev < 1e-9, "left half deviates {dev:e} at {ul}");
            }
        }
    }
}

#[test]
fn knot_remove_refuses_the_domain_endpoints() {
    for c in curves() {
        let (lo, hi) = c.domain();
        for u in [lo, hi] {
            for tol in [1e-9, 1.0, 1e9] {
                assert!(
                    matches!(
                        curve_knot_remove(&c, u, tol),
                        Err(MathError::ParameterOutOfRange { .. })
                    ),
                    "removing end knot {u} must be refused, not panic"
                );
            }
        }
    }
}

#[test]
fn knot_remove_still_round_trips_an_inserted_interior_knot() {
    let c = cubic_bezier();
    let u = 0.375;
    let inserted = curve_knot_insert(&c, u, 1).expect("insert");
    let removed = curve_knot_remove(&inserted, u, 1e-6).expect("remove the knot just inserted");
    assert_eq!(removed.knots().len(), c.knots().len());
    for i in 0..=10 {
        let t = f64::from(i) / 10.0;
        let dev = (removed.evaluate(t) - c.evaluate(t)).length();
        assert!(dev < 1e-9, "round trip deviates {dev:e} at {t}");
    }
}

#[test]
fn knot_remove_leaves_a_curve_unchanged_when_the_knot_is_absent() {
    // Interior-but-absent stays the documented no-op; only the endpoints
    // became a typed refusal.
    let c = cubic_bezier();
    let out = curve_knot_remove(&c, 0.5, 1e-6).expect("absent interior knot is a no-op");
    assert_eq!(out.knots(), c.knots());
}

#[test]
fn bezier_decomposition_refuses_a_c_minus_1_curve() {
    // Interior multiplicity p+1 = 3 is a C^-1 break. It is constructible, but
    // the segment walk assumes consecutive segments share a control point,
    // which only holds at multiplicity exactly p — it used to underflow, and
    // the saturating form emitted a collapsed-domain segment instead.
    let c = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0],
        (0..6)
            .map(|i| Point3::new(f64::from(i), f64::from(i % 2), 0.0))
            .collect(),
        vec![1.0; 6],
    )
    .unwrap();
    assert!(matches!(
        curve_to_bezier_segments(&c),
        Err(MathError::InvalidKnotValue { .. })
    ));
    // `curve_degree_elevate` decomposes first, so it inherits the refusal.
    assert!(matches!(
        curve_degree_elevate(&c, 1),
        Err(MathError::InvalidKnotValue { .. })
    ));
}

#[test]
fn bezier_decomposition_still_handles_multiplicity_p() {
    // Multiplicity exactly p is a C^0 corner and must still decompose.
    let c = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        (0..5)
            .map(|i| Point3::new(f64::from(i), f64::from(i % 2), 0.0))
            .collect(),
        vec![1.0; 5],
    )
    .unwrap();
    let segs = curve_to_bezier_segments(&c).expect("C^0 curve decomposes");
    assert_eq!(segs.len(), 2);
    for s in &segs {
        let (a, b) = s.domain();
        assert!(b > a, "no segment may have a collapsed domain: [{a}, {b}]");
    }
}

#[test]
fn collapsed_parameter_domain_is_rejected_at_construction() {
    // The backstop that would have caught the decomposition defect at the
    // point the bad curve was created rather than where it was used.
    assert!(matches!(
        NurbsCurve::new(
            2,
            vec![0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
            (0..3)
                .map(|i| Point3::new(f64::from(i), 0.0, 0.0))
                .collect(),
            vec![1.0; 3],
        ),
        Err(MathError::InvalidKnotValue { .. })
    ));
}
