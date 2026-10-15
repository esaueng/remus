//! Degree-1 knot removal must honour its tolerance (kernel review
//! 2026-10-14, MATH-3).
//!
//! `curve_knot_remove` accepted a removal when its left and right sweeps
//! agreed where they overlap. At degree 1 they do not overlap, so nothing
//! was compared and every interior knot came off: a polyline corner was
//! flattened to its chord at any tolerance. Piegl & Tiller A5.8 handles this
//! case with its odd-count branch — the dropped control point must lie on
//! the chord of its two neighbours at the knot.
//!
//! Every oracle below is closed form. For a degree-1 curve the point at a
//! knot is the control point there, and after removal it is the chord
//! `(1 − α)·P[k−1] + α·P[k+1]` with `α = (u − t_a)/(t_b − t_a)` over the
//! surviving span `[t_a, t_b]`; both curves are linear on either side of the
//! knot and share their ends, so that distance is the curve's deviation.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use remus_math::MathError;
use remus_math::context::OperationContext;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::knot_ops::{curve_knot_insert, curve_knot_remove};
use remus_math::nurbs::reduction::simplify_curve;
use remus_math::nurbs::reuse::{ReductionOptions, ReuseError};
use remus_math::vec::Point3;

fn polyline(knots: Vec<f64>, points: &[[f64; 3]], weights: Vec<f64>) -> NurbsCurve {
    NurbsCurve::new(
        1,
        knots,
        points
            .iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect(),
        weights,
    )
    .unwrap()
}

fn corner() -> NurbsCurve {
    polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0; 3],
    )
}

fn is_refused(result: &Result<NurbsCurve, MathError>) -> bool {
    matches!(result, Err(MathError::ConvergenceFailure { .. }))
}

#[test]
fn polyline_corner_is_not_removable_within_a_tight_tolerance() {
    // The review's repro: C(0.5) = (1, 1, 0) and the chord gives (1, 0, 0),
    // a deviation of exactly 1.0. Main removed it at 1e-6.
    let c = corner();
    assert_eq!(c.evaluate(0.5), Point3::new(1.0, 1.0, 0.0));
    for tol in [1e-12, 1e-6, 0.5, 0.999_999] {
        assert!(
            is_refused(&curve_knot_remove(&c, 0.5, tol)),
            "corner deviating 1.0 must be refused at tol {tol}"
        );
    }
}

#[test]
fn polyline_corner_removal_is_accepted_at_its_exact_deviation() {
    // The bound carries a rounding allowance of about 1e-14 here, so a
    // deviation of exactly 1.0 is accepted just above tol = 1.0, not at it,
    // and the result is the closed-form chord.
    let c = corner();
    assert!(is_refused(&curve_knot_remove(&c, 0.5, 1.0)));
    let removed = curve_knot_remove(&c, 0.5, 1.0 + 1e-13).expect("deviation 1.0 within tol");
    assert_eq!(removed.knots(), &[0.0, 0.0, 1.0, 1.0]);
    assert_eq!(
        removed.control_points(),
        &[Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]
    );
    assert_eq!(removed.evaluate(0.5), Point3::new(1.0, 0.0, 0.0));
    assert_eq!((removed.evaluate(0.5) - c.evaluate(0.5)).length(), 1.0);
}

#[test]
fn collinear_point_off_its_parameter_is_still_a_deviation() {
    // Same point set as the straight segment, different parameterisation:
    // C(0.5) = (0.5, 0, 0) but the one-span segment through the same ends
    // reads (1, 0, 0) there. Knot removal preserves the parametric curve,
    // so a 0.5 deviation is refused although nothing moves off the line.
    let c = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0; 3],
    );
    assert!(is_refused(&curve_knot_remove(&c, 0.5, 1e-6)));
    assert!(is_refused(&curve_knot_remove(&c, 0.5, 0.499)));
    assert!(curve_knot_remove(&c, 0.5, 0.5 + 1e-13).is_ok());
}

#[test]
fn point_on_the_chord_at_its_parameter_is_removed_exactly() {
    // Off-centre knot: α = 0.25 puts the chord at (0.5, 0, 0), which is
    // exactly the stored point, so the removal is exact. Its bound still
    // carries the rounding allowance, so tolerance 0 cannot certify it.
    let c = polyline(
        vec![0.0, 0.0, 0.25, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0; 3],
    );
    assert!(is_refused(&curve_knot_remove(&c, 0.25, 0.0)));
    let removed = curve_knot_remove(&c, 0.25, 1e-13).expect("exactly removable");
    assert_eq!(
        removed.control_points(),
        &[Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]
    );
    for i in 0..=8 {
        let t = f64::from(i) / 8.0;
        let dev = (removed.evaluate(t) - c.evaluate(t)).length();
        assert!(dev <= 1e-15, "deviates {dev:e} at t = {t}");
    }
}

#[test]
fn interior_corner_of_a_longer_polyline_is_refused_and_a_flat_one_removed() {
    // Four points, knots at 1/3 and 2/3. The middle-left vertex sits on its
    // neighbours' chord at α = 1/2 only in x; y = 3 is off by exactly 3.
    let c = polyline(
        vec![0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0],
        &[
            [0.0, 0.0, 0.0],
            [1.0, 3.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
        vec![1.0; 4],
    );
    assert!(is_refused(&curve_knot_remove(&c, 1.0 / 3.0, 1e-6)));
    // The second vertex sits between (1, 3) and (3, 0) at α = 1/2 only if it
    // were (2, 1.5); at (2, 0) it is off by 1.5.
    assert!(is_refused(&curve_knot_remove(&c, 2.0 / 3.0, 1e-6)));
    assert!(curve_knot_remove(&c, 2.0 / 3.0, 1.5 + 1e-12).is_ok());
}

#[test]
fn degree_one_insert_then_remove_round_trips() {
    let c = corner();
    for u in [0.125, 0.3, 0.75] {
        let inserted = curve_knot_insert(&c, u, 1).expect("insert");
        assert_eq!(inserted.control_points().len(), 4);
        let removed = curve_knot_remove(&inserted, u, 1e-12).expect("inserted knot is removable");
        assert_eq!(removed.knots(), c.knots());
        for i in 0..=16 {
            let t = f64::from(i) / 16.0;
            let dev = (removed.evaluate(t) - c.evaluate(t)).length();
            assert!(
                dev <= 1e-15,
                "round trip at u = {u} deviates {dev:e} at {t}"
            );
        }
    }
}

#[test]
fn rational_degree_one_corner_is_refused_and_inserted_knot_removed() {
    // Weight 2 on the corner leaves its position (1, 1, 0); the chord at
    // α = 1/2 is still (1, 0, 0), so the deviation stays 1.0.
    let weighted = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0, 2.0, 1.0],
    );
    assert!(is_refused(&curve_knot_remove(&weighted, 0.5, 1e-6)));

    // Inserting 0.5 into the segment (0,0,0; w 1) – (2,0,0; w 3) blends the
    // homogeneous points to (3, 0, 0, 2), i.e. (1.5, 0, 0) with weight 2.
    // That point is on the chord by construction and must come back off.
    let segment = polyline(
        vec![0.0, 0.0, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0, 3.0],
    );
    let inserted = curve_knot_insert(&segment, 0.5, 1).expect("insert");
    assert_eq!(inserted.control_points()[1], Point3::new(1.5, 0.0, 0.0));
    assert_eq!(inserted.weights()[1], 2.0);
    let removed = curve_knot_remove(&inserted, 0.5, 1e-12).expect("removable");
    assert_eq!(removed.control_points(), segment.control_points());
    assert_eq!(removed.weights(), segment.weights());
}

/// Largest distance between `a` and `b` over `samples + 1` parameters of
/// their shared domain.
fn sampled_deviation(a: &NurbsCurve, b: &NurbsCurve, samples: u32) -> f64 {
    let (lo, hi) = a.domain();
    (0..=samples)
        .map(|i| {
            let t = lo + (hi - lo) * f64::from(i) / f64::from(samples);
            (a.evaluate(t) - b.evaluate(t)).length()
        })
        .fold(0.0, f64::max)
}

#[test]
fn collinear_rational_point_with_a_mismatched_weight_is_refused() {
    // Review follow-up: x = 0, 1, 2 with weights 1, 100, 1. The knot point
    // sits exactly on its Cartesian chord, but the weight pulls the whole
    // first span toward x = 1. With s = 2t the old curve is
    // x = 100s / (1 + 99s) against the chord's x = s; their gap peaks at
    // s = 1/11 with 9/11. The previous check compared positions at the knot
    // only and removed this at tolerance 0.
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0, 100.0, 1.0],
    );
    let chord = polyline(
        vec![0.0, 0.0, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        vec![1.0, 1.0],
    );
    let peak = sampled_deviation(&curve, &chord, 4400);
    assert!((peak - 9.0 / 11.0).abs() < 1e-9, "premise: peak {peak}");
    for tol in [0.0, 1e-6, 0.5, 0.8] {
        assert!(
            is_refused(&curve_knot_remove(&curve, 0.5, tol)),
            "removed at tolerance {tol} despite a 9/11 deviation"
        );
    }
    // The bound is 0.99 (|e - εP| / w_k with e = ε = 99); a tolerance
    // above it accepts, and the result is the chord.
    let removed = curve_knot_remove(&curve, 0.5, 1.0).expect("within tolerance 1");
    assert!(sampled_deviation(&removed, &chord, 64) < 1e-15);
}

#[test]
fn every_accepted_rational_removal_stays_within_its_tolerance() {
    // Sweep weights, knot positions and an off-chord offset; whenever the
    // removal is accepted, the sampled deviation of the result from the
    // original must not exceed the tolerance it was accepted at.
    let mut accepted = 0;
    for &w_mid in &[0.2, 0.9, 1.0, 1.1, 3.0, 50.0] {
        for &w_end in &[0.5, 1.0, 4.0] {
            for &u in &[0.2, 0.5, 0.75] {
                for &lift in &[0.0, 1e-3, 0.1] {
                    for &tol in &[1e-9, 1e-3, 0.05, 0.3, 1.0] {
                        let curve = polyline(
                            vec![0.0, 0.0, u, 1.0, 1.0],
                            &[[0.0, 0.0, 0.0], [3.0 * u, lift, 0.0], [3.0, 0.0, 0.0]],
                            vec![1.0, w_mid, w_end],
                        );
                        if let Ok(removed) = curve_knot_remove(&curve, u, tol) {
                            accepted += 1;
                            let dev = sampled_deviation(&curve, &removed, 2000);
                            assert!(
                                dev <= tol + 1e-12,
                                "w_mid {w_mid}, w_end {w_end}, u {u}, lift {lift}: \
                                 accepted at {tol} with deviation {dev}"
                            );
                        }
                    }
                }
            }
        }
    }
    // The sweep must exercise acceptance, not only refusal.
    assert!(accepted > 50, "only {accepted} removals accepted");
}

#[test]
fn uniformly_scaled_weights_give_the_unit_weight_answer() {
    // Review follow-up: weights sharing a projective scale describe the same
    // curve. At 1e-320 the homogeneous products underflowed, the bound came
    // out 0 and the unit corner (deviation 1) was removed at tolerance 0.
    for scale in [1e-320, 1e-300, 1e300] {
        let scaled = polyline(
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            &[[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
            vec![scale; 3],
        );
        for tol in [0.0, 1e-6, 0.999_999] {
            assert!(
                is_refused(&curve_knot_remove(&scaled, 0.5, tol)),
                "weights {scale:e}: removed at tolerance {tol}"
            );
        }
        let removed = curve_knot_remove(&scaled, 0.5, 1.0 + 1e-13).expect("deviation is exactly 1");
        assert_eq!(removed.control_points().len(), 2);
    }
}

#[test]
fn chord_position_comes_from_the_stored_knot_not_the_query() {
    // Review follow-up: x = 0, 5e14, 1e15 has its middle point exactly on
    // the chord at the stored knot 0.5. A query seven ulps away still matches
    // that knot (within the knot epsilon) but shifted α by ~7e-16, which over
    // a 1e15 chord read as a deviation of ~0.78. The bound's rounding
    // allowance at this scale is about 5.3, so tolerance 5.7 accepts the
    // stored-knot bound (5.3) and still refuses the query's (about 6.1).
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [5e14, 0.0, 0.0], [1e15, 0.0, 0.0]],
        vec![1.0; 3],
    );
    let u = 0.5 + 7.0 * f64::EPSILON / 2.0;
    assert!(u > 0.5 && u - 0.5 < 1e-15, "premise: query {u} matches 0.5");
    let removed = curve_knot_remove(&curve, u, 5.7).expect("exactly removable");
    assert_eq!(removed.knots(), &[0.0, 0.0, 1.0, 1.0]);
}

#[test]
fn weights_spanning_the_f64_range_refuse_rather_than_accept() {
    // Review follow-up: with weights [MAX, MIN_POSITIVE, MAX] the middle
    // weight normalizes to zero, the bound was 0/0 = NaN and `NaN > tol` let
    // the removal through at tolerance 0, though the curve passes (0, 1, 0)
    // at the knot and the chord is the origin.
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]],
        vec![f64::MAX, f64::MIN_POSITIVE, f64::MAX],
    );
    for tol in [0.0, 1e-6, 0.5] {
        assert!(
            is_refused(&curve_knot_remove(&curve, 0.5, tol)),
            "removed at tolerance {tol}"
        );
    }
}

#[test]
fn a_large_common_offset_does_not_cancel_the_deviation() {
    // Review follow-up: x = p, p + ulp, p + 3·ulp at p = 1e12 with the knot
    // at 0.2. The chord there is p + 0.6·ulp, so the middle point is 0.4 ulp
    // off it (4.9e-5). Forming the chord in absolute coordinates rounded it
    // onto the middle point and the bound came out 0.
    let p = 1e12_f64;
    let ulp = f64::from_bits(p.to_bits() + 1) - p;
    let curve = polyline(
        vec![0.0, 0.0, 0.2, 1.0, 1.0],
        &[
            [p, 0.0, 0.0],
            [p + ulp, 0.0, 0.0],
            [p + 3.0 * ulp, 0.0, 0.0],
        ],
        vec![1.0; 3],
    );
    assert!(is_refused(&curve_knot_remove(&curve, 0.2, 1e-6)));
    assert!(curve_knot_remove(&curve, 0.2, 0.5 * ulp).is_ok());
}

#[test]
fn a_tiny_deviation_does_not_underflow_to_zero() {
    // Review follow-up: the corner (0, 1e-200, 0) squares to 1e-400, which
    // underflowed to 0 and let the removal through at tolerance 0.
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [0.0, 1e-200, 0.0], [0.0, 0.0, 0.0]],
        vec![1.0; 3],
    );
    assert!(is_refused(&curve_knot_remove(&curve, 0.5, 0.0)));
    assert!(is_refused(&curve_knot_remove(&curve, 0.5, 0.5e-200)));
    assert!(curve_knot_remove(&curve, 0.5, 1e-200 * (1.0 + 1e-13)).is_ok());
}

#[test]
fn a_tiny_weight_and_a_tiny_offset_together_still_count() {
    // Review follow-up: middle point (0, 1e-200, 0) with weight 1e-200
    // between two points at the origin. The deviation at the knot is
    // 1e-200, but weight × offset = 1e-400 underflowed to 0 before the norm
    // could protect it, and the removal passed at tolerance 0.
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [0.0, 1e-200, 0.0], [0.0, 0.0, 0.0]],
        vec![1.0, 1e-200, 1.0],
    );
    assert!(is_refused(&curve_knot_remove(&curve, 0.5, 0.0)));
    assert!(is_refused(&curve_knot_remove(&curve, 0.5, 0.5e-200)));
}

#[test]
fn a_weight_ratio_below_the_normal_range_refuses() {
    // Review follow-up: MIN_POSITIVE / 1e16 rounds to zero, which dropped
    // the far control point's ρ·dn ≈ 2.2e-16 and removed the knot at
    // tolerance 0 although the curve moves by about 1.1e-16 at the knot.
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1e308, 0.0, 0.0]],
        vec![2e16, 1e16, f64::MIN_POSITIVE],
    );
    assert!(is_refused(&curve_knot_remove(&curve, 0.5, 0.0)));
    assert!(is_refused(&curve_knot_remove(&curve, 0.5, 1e-17)));
}

#[test]
fn certified_reduction_still_gets_its_candidate_past_the_ratio_gate() {
    // Review follow-up: a rational line with end weights 1e-200 and 1e108,
    // with u = 0.5 inserted, has the ratio w_prev / w_mid ≈ 2e-308, which is
    // subnormal. The public removal refuses it, but the certified reduction
    // takes its candidates from an unchecked generator and certifies them
    // itself, so the gate must not pre-empt it: the decision belongs to the
    // certificate, as on `main`.
    let line = polyline(
        vec![0.0, 0.0, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]],
        vec![1e-200, 1e108],
    );
    let inserted = curve_knot_insert(&line, 0.5, 1).unwrap();
    let w = inserted.weights();
    assert!(
        (w[0] / w[1]).is_subnormal(),
        "premise: ratio {:e}",
        w[0] / w[1]
    );
    // The public removal refuses at every tolerance, f64::MAX included:
    // without a bound there is nothing to hold the tolerance against.
    for tol in [1.0, f64::MAX] {
        assert!(is_refused(&curve_knot_remove(&inserted, 0.5, tol)));
    }
    // The certificate, not the candidate generator, now rules on this knot.
    // Its Bernstein bound cannot resolve weights 308 decades apart, which is
    // a typed refusal; a silent "no removable knot" would mean the
    // generator had swallowed the candidate.
    match simplify_curve(
        &inserted,
        &ReductionOptions::default(),
        &OperationContext::new(),
    ) {
        Ok(reduced) => assert_eq!(reduced.removed_knots, 1),
        Err(ReuseError::BoundUnavailable { .. }) => {}
        Err(other) => panic!("unexpected refusal {other:?}"),
    }
}

#[test]
fn an_overflowed_coordinate_difference_does_not_vanish() {
    // Review follow-up: with x = -MAX, MAX, MAX both differences from the
    // first point overflow to ∞ and the excess is ∞ − ∞ = NaN, which
    // `f64::max` dropped, so the bound was 0 and the removal was accepted at
    // tolerance 0 although the curve sits at MAX at the knot.
    let curve = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[
            [-f64::MAX, 0.0, 0.0],
            [f64::MAX, 0.0, 0.0],
            [f64::MAX, 0.0, 0.0],
        ],
        vec![1.0; 3],
    );
    for tol in [0.0, 1.0, 1e300, f64::MAX] {
        assert!(
            is_refused(&curve_knot_remove(&curve, 0.5, tol)),
            "removed at tolerance {tol:e}"
        );
    }
    // Review follow-up: with the ends at -MAX and the middle at MAX the
    // true deviation is 2·MAX, which exceeds even tolerance f64::MAX; an
    // unusable bound must not pass as one.
    let spike = polyline(
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        &[
            [-f64::MAX, 0.0, 0.0],
            [f64::MAX, 0.0, 0.0],
            [-f64::MAX, 0.0, 0.0],
        ],
        vec![1.0; 3],
    );
    assert!(is_refused(&curve_knot_remove(&spike, 0.5, f64::MAX)));
}

#[test]
fn a_weight_excess_across_widely_separated_spans_is_kept() {
    // Review follow-up: knots 0, 1, 1e100 put the removed knot at
    // α = 1e-100. The weight excess ε/w_k = −9e-100 cancelled to 0 in
    // `1 − α·ρ_next − (1 − α)·ρ_prev`, and with it ε·dn ≈ 9e100, the whole
    // bound; the removal was accepted at tolerance 0 although the curve
    // moves (by about 1.55e85 at u = 0.9).
    let curve = polyline(
        vec![0.0, 0.0, 1.0, 1e100, 1e100],
        &[[1.0, 0.0, 0.0], [-1e101, 0.0, 0.0], [-1e200, 0.0, 0.0]],
        vec![1.0, 1.0, 10.0],
    );
    for tol in [0.0, 1e85, 1e100] {
        assert!(
            is_refused(&curve_knot_remove(&curve, 1.0, tol)),
            "removed at tolerance {tol:e}"
        );
    }
}

#[test]
fn cancellation_in_the_weight_excess_does_not_hide_a_deviation() {
    // Review follow-up: the two terms of ε rounded to exact opposites, so
    // the bound came out 1.44e163 while the curves differ by 2.21e163 at the
    // stored knot, and the removal passed at 1.5e163. The bound now carries
    // a forward error allowance for its own evaluation.
    let curve = polyline(
        vec![0.0, 0.0, 0.3, 1.0, 1.0],
        &[
            [0.0, 0.0, 0.0],
            [-5.352_990_896_342_235e178, 0.0, 0.0],
            [-5.357_887_885_145_676e178, 0.0, 0.0],
        ],
        vec![0.001_305_682_097_234_999_2, 1.0, 3.330_286_741_773_118],
    );
    for tol in [1.5e163, 2.2e163] {
        assert!(
            is_refused(&curve_knot_remove(&curve, 0.3, tol)),
            "removed at tolerance {tol:e} despite a 2.21e163 deviation"
        );
    }

    // The same with plain weights: x = 0, 1e11, 1e12 at α = 0.1 puts the
    // chord at 1e11 + 5.55e-6, below half an ulp of 1e11, and the bound
    // rounded that away. Below the allowance nothing is certified.
    let curve = polyline(
        vec![0.0, 0.0, 0.1, 1.0, 1.0],
        &[[0.0, 0.0, 0.0], [1e11, 0.0, 0.0], [1e12, 0.0, 0.0]],
        vec![1.0; 3],
    );
    for tol in [0.0, 1e-6, 5e-6] {
        assert!(
            is_refused(&curve_knot_remove(&curve, 0.1, tol)),
            "removed at tolerance {tol:e} despite a 5.55e-6 deviation"
        );
    }
    assert!(curve_knot_remove(&curve, 0.1, 1e-2).is_ok());
}
