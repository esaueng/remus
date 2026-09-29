//! Closed-form oracles for the O2.3b arrangement engine (B19 splitter
//! mutant tranche, 2026-09-27 run).
//!
//! Every expected value here is derived by hand from the fixture geometry,
//! never read back from the engine:
//! - `roundoff` is asserted against its own formula at three magnitudes;
//! - `validate_use` gets one malformed input per disjunct, each tripping
//!   exactly one condition, with the exact `ArrangementError` variant and a
//!   neighbouring accepted input;
//! - `intersections` uses 3-4-5 triangles, unit-direction lines on integer
//!   grids and circles of radii 5 and 3 whose crossings are `atan2` of
//!   integers; the tangency bands are probed with dyadic offsets so the
//!   discriminant is an exact multiple of `64·ε`;
//! - `tangent_order` is a fan of eight compass directions;
//! - `ray_crossing` and `distance` use probe tables with the crossing counts
//!   and clamp cases written out;
//! - `quotient` uses cylinder strips whose Euler characteristic, winding
//!   signs, seam identifications and `area · radius` are hand-derived, plus
//!   corrupted inputs that must be refused with a named variant;
//! - budget assertions count the events a fixture needs (a full circle is one
//!   seam vertex plus three interior cardinal cuts) or state that a refusal
//!   happens before any work is charged.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::too_many_lines
)]

use super::geometry::{
    distance, intersections, ray_crossing, roundoff, tangent_order, validate_use,
};
use super::*;
use remus_math::context::{OperationContext, WorkBudgets};
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Circle2D, Line2D};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Vec2, Vec3};
use std::cmp::Ordering;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

// ── Fixture builders ────────────────────────────────────────────────────

fn context() -> OperationContext {
    OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_march_steps(2_000_000)
            .with_queue_size(20_000)
            .with_segments(10_000),
    )
}
fn budget(march_steps: usize) -> OperationContext {
    OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_march_steps(march_steps)
            .with_queue_size(20_000)
            .with_segments(10_000),
    )
}
fn source(id: u64) -> CurveSource {
    CurveSource {
        use_id: id,
        boundary: None,
        section: Some(id as usize),
        source_edge_idx: Some(id as usize),
        pave_block_id: Some(id as usize + 1000),
    }
}
fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}
fn xyz(p: Point2) -> Point3 {
    Point3::new(p.x(), p.y(), 0.0)
}
/// A straight use from `a` to `b` (unit-speed pcurve, range `[0, |b−a|]`).
fn line(id: u64, a: Point2, b: Point2, endpoints: [u64; 2], boundary: Option<u64>) -> CurveUse {
    CurveUse {
        source: source(id),
        pcurve: Curve2D::Line(Line2D::new(a, b - a).unwrap()),
        range: [0.0, (b - a).length()],
        curve_3d: EdgeCurve::Line,
        source_range: [0.0, 1.0],
        endpoints_3d: [xyz(a), xyz(b)],
        endpoints,
        boundary_loop: boundary,
    }
}
/// A straight use whose pcurve origin and range are chosen explicitly, so
/// `range[0]` need not be zero.
fn line_ranged(
    id: u64,
    origin: Point2,
    direction: Vec2,
    range: [f64; 2],
    endpoints: [u64; 2],
) -> CurveUse {
    let pcurve = Line2D::new(origin, direction).unwrap();
    let a = pcurve.evaluate(range[0]);
    let b = pcurve.evaluate(range[1]);
    CurveUse {
        source: source(id),
        pcurve: Curve2D::Line(pcurve),
        range,
        curve_3d: EdgeCurve::Line,
        source_range: [0.0, 1.0],
        endpoints_3d: [xyz(a), xyz(b)],
        endpoints,
        boundary_loop: None,
    }
}
fn rectangle(id: u64, x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<CurveUse> {
    let points = [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
    (0..4)
        .map(|i| {
            line(
                id + i as u64,
                points[i],
                points[(i + 1) % 4],
                [id + i as u64, id + ((i + 1) % 4) as u64],
                Some(id),
            )
        })
        .collect()
}
/// A circular arc use over pcurve angles `range`, with a consistent 3D
/// circle in the plane `z = 0`.
fn arc(id: u64, center: Point2, radius: f64, range: [f64; 2], endpoints: [u64; 2]) -> CurveUse {
    let c = Circle3D::new_with_ref(
        xyz(center),
        Vec3::new(0.0, 0.0, 1.0),
        radius,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    CurveUse {
        source: source(id),
        pcurve: Curve2D::Circle(Circle2D::new(center, radius).unwrap()),
        range,
        curve_3d: EdgeCurve::Circle(c.clone()),
        source_range: range,
        endpoints_3d: [c.evaluate(range[0]), c.evaluate(range[1])],
        endpoints,
        boundary_loop: None,
    }
}
fn circle(id: u64, center: Point2, radius: f64, boundary: Option<u64>) -> CurveUse {
    let mut u = arc(id, center, radius, [0.0, TAU], [id, id]);
    u.boundary_loop = boundary;
    u
}
fn build(uses: &[CurveUse]) -> Result<Arrangement> {
    build_arrangement(&ArrangementInput {
        uses,
        domain: ParamDomain::Plane,
        context: &context(),
    })
}
fn build_with(uses: &[CurveUse], context: &OperationContext) -> Result<Arrangement> {
    build_arrangement(&ArrangementInput {
        uses,
        domain: ParamDomain::Plane,
        context,
    })
}
fn lookup(uses: &[CurveUse]) -> BTreeMap<u64, &CurveUse> {
    uses.iter().map(|u| (u.source.use_id, u)).collect()
}
fn near(actual: f64, expected: f64, abs: f64) {
    assert!(
        (actual - expected).abs() <= abs,
        "{actual} != {expected} (|Δ| = {}, bound {abs})",
        (actual - expected).abs()
    );
}
/// Parameter pairs compared at 1e-12: a wrong operator moves every root by
/// a visible amount on these integer-friendly figures.
fn assert_hits(actual: &[(f64, f64)], expected: &[(f64, f64)]) {
    assert_eq!(actual.len(), expected.len(), "{actual:?} vs {expected:?}");
    for (a, e) in actual.iter().zip(expected) {
        near(a.0, e.0, 1e-12);
        near(a.1, e.1, 1e-12);
    }
}
fn hits(a: &CurveUse, b: &CurveUse) -> Result<Vec<(f64, f64)>> {
    let ctx = context();
    let mut work = Work::new(&ctx);
    intersections(a, b, &mut work)
}
/// `intersections` with no work budget at all: only a pair that is certified
/// disjoint by the exact orientation signs (and so charges no step) succeeds.
fn hits_unbudgeted(a: &CurveUse, b: &CurveUse) -> Result<Vec<(f64, f64)>> {
    let ctx = budget(0);
    let mut work = Work::new(&ctx);
    intersections(a, b, &mut work)
}

// ── roundoff / finite (geometry.rs 18, 21) ──────────────────────────────

#[test]
fn roundoff_is_the_stated_formula_and_monotone_in_the_coordinate_sum() {
    for (x, y) in [(0.0, 0.0), (3.0, -4.0), (1.0e6, -2.0e6), (-1.0e-9, 5.0e12)] {
        assert_eq!(
            roundoff(p(x, y)),
            64.0 * f64::EPSILON * (1.0 + x.abs() + y.abs()),
            "roundoff({x}, {y})"
        );
    }
    assert_eq!(roundoff(p(0.0, 0.0)), 64.0 * f64::EPSILON);
    assert_eq!(roundoff(p(3.0, -4.0)), 64.0 * f64::EPSILON * 8.0);
    assert!(roundoff(p(3.0, -4.0)) < roundoff(p(3.0, -5.0)));
    assert!(roundoff(p(3.0, -5.0)) < roundoff(p(-4.0, -5.0)));
    assert!(roundoff(p(1.0e6, 0.0)) < roundoff(p(0.0, 2.0e6)));
}

// ── validate_use (geometry.rs 31–78) ────────────────────────────────────

fn valid_line() -> CurveUse {
    line(7, p(1.0, 2.0), p(4.0, 6.0), [70, 71], Some(7))
}
fn valid_arc() -> CurveUse {
    arc(8, p(0.0, 0.0), 1.0, [0.0, FRAC_PI_2], [80, 81])
}
fn validate(u: &CurveUse) -> Result<()> {
    validate_use(u, 1e-7)
}

#[test]
fn well_formed_line_and_arc_uses_validate() {
    assert_eq!(validate(&valid_line()), Ok(()));
    assert_eq!(validate(&valid_arc()), Ok(()));
}

/// Each malformed input trips exactly one disjunct of the finiteness chain
/// (lines 32–37), so an `||` turned into `&&` lets it through.
#[test]
fn every_non_finite_disjunct_refuses_on_its_own() {
    // Only `source_range` is non-finite (range, 3D ends and 2D ends are fine).
    let mut u = valid_line();
    u.source_range = [0.0, f64::NAN];
    assert_eq!(validate(&u), Err(ArrangementError::NonFiniteInput));
    // Only one 3D endpoint coordinate is non-finite: x, then y, then z.
    for axis in 0..3 {
        let mut u = valid_line();
        let e = u.endpoints_3d[0];
        u.endpoints_3d[0] = match axis {
            0 => Point3::new(f64::NAN, e.y(), e.z()),
            1 => Point3::new(e.x(), f64::INFINITY, e.z()),
            _ => Point3::new(e.x(), e.y(), f64::NEG_INFINITY),
        };
        assert_eq!(
            validate(&u),
            Err(ArrangementError::NonFiniteInput),
            "axis {axis}"
        );
    }
    // Only the pcurve end at `range[1]` is non-finite: a huge circle whose
    // start `center + r·(cos π, sin π)` is finite but whose end overflows.
    let mut u = valid_line();
    u.pcurve = Curve2D::Circle(Circle2D::new(p(f64::MAX, 0.0), f64::MAX).unwrap());
    u.range = [PI, TAU];
    assert!(u.point(u.range[0]).x().is_finite() && u.point(u.range[0]).y().is_finite());
    assert!(!u.point(u.range[1]).x().is_finite());
    assert_eq!(validate(&u), Err(ArrangementError::NonFiniteInput));
    // Only the pcurve end at `range[0]` is non-finite (the mirror case).
    u.range = [TAU, PI];
    assert_eq!(validate(&u), Err(ArrangementError::NonFiniteInput));
    // A pcurve point with one finite and one NaN coordinate is not finite.
    let mut u = valid_line();
    u.pcurve = Curve2D::Line(Line2D::new(p(0.0, f64::NAN), Vec2::new(1.0, 0.0)).unwrap());
    assert!(u.point(u.range[0]).x().is_finite());
    assert_eq!(validate(&u), Err(ArrangementError::NonFiniteInput));
    let mut u = valid_line();
    u.pcurve = Curve2D::Line(Line2D::new(p(f64::NAN, 0.0), Vec2::new(1.0, 0.0)).unwrap());
    assert_eq!(validate(&u), Err(ArrangementError::NonFiniteInput));
}

#[test]
fn degenerate_ranges_refuse_separately_and_unsupported_curves_are_named() {
    // Only the source interval is degenerate.
    let mut u = valid_line();
    u.source_range = [5.0, 5.0];
    assert_eq!(validate(&u), Err(ArrangementError::InvalidBoundary));
    // Only the pcurve interval is degenerate.
    let mut u = valid_line();
    u.range = [2.0, 2.0];
    assert_eq!(validate(&u), Err(ArrangementError::InvalidBoundary));
    // Signed zero is the same parameter.
    let mut u = valid_line();
    u.range = [0.0, -0.0];
    assert_eq!(validate(&u), Err(ArrangementError::InvalidBoundary));
    // Unsupported 3D carriers are refused before the interval checks.
    let mut u = valid_line();
    u.curve_3d = EdgeCurve::NurbsCurve(
        remus_math::nurbs::curve::NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap(),
    );
    assert_eq!(validate(&u), Err(ArrangementError::UnsupportedCurve));
    // A circle pcurve outside the native `[0, 2π]` domain is unsupported.
    let u = arc(8, p(0.0, 0.0), 1.0, [-0.5, 1.0], [80, 81]);
    assert_eq!(validate(&u), Err(ArrangementError::UnsupportedCurve));
    let u = arc(8, p(0.0, 0.0), 1.0, [1.0, TAU + 0.5], [80, 81]);
    assert_eq!(validate(&u), Err(ArrangementError::UnsupportedCurve));
}

/// The 3D circle end check (lines 50–59): a 3D circle whose evaluation has
/// exactly one non-finite coordinate is refused, and an end exactly
/// `tolerance` away from its stored 3D endpoint is accepted while one
/// farther is refused.
#[test]
fn circle_3d_end_checks_probe_each_coordinate_and_the_exact_tolerance() {
    let bad_center = |c: Point3| {
        let mut u = valid_arc();
        u.curve_3d = EdgeCurve::Circle(
            Circle3D::new_with_ref(c, Vec3::new(0.0, 0.0, 1.0), 1.0, Vec3::new(1.0, 0.0, 0.0))
                .unwrap(),
        );
        u
    };
    for c in [
        Point3::new(f64::NAN, 0.0, 0.0),
        Point3::new(0.0, f64::NAN, 0.0),
        Point3::new(0.0, 0.0, f64::NAN),
    ] {
        let u = bad_center(c);
        let e = if let EdgeCurve::Circle(circle) = &u.curve_3d {
            circle.evaluate(u.source_range[0])
        } else {
            unreachable!()
        };
        assert_eq!(
            [e.x(), e.y(), e.z()]
                .iter()
                .filter(|v| !v.is_finite())
                .count(),
            1,
            "{c:?} must poison exactly one coordinate"
        );
        assert_eq!(validate(&u), Err(ArrangementError::NonFiniteInput), "{c:?}");
    }
    // The 3D circle evaluates exactly (1, 0, 0) at angle 0; store the start
    // endpoint 0.5 above it and validate with tolerance 0.5: on the bound.
    let mut u = valid_arc();
    u.endpoints_3d[0] = Point3::new(1.0, 0.0, 0.5);
    assert_eq!(validate_use(&u, 0.5), Ok(()));
    assert_eq!(
        validate_use(&u, 0.5 - 1e-9),
        Err(ArrangementError::InvalidBoundary)
    );
    u.endpoints_3d[0] = Point3::new(1.0, 0.0, 0.5 + 1e-9);
    assert_eq!(
        validate_use(&u, 0.5),
        Err(ArrangementError::InvalidBoundary)
    );
    // The end endpoint is checked the same way.
    let mut u = valid_arc();
    u.endpoints_3d[1] = Point3::new(0.0, 1.0, -0.5);
    assert_eq!(validate_use(&u, 0.5), Ok(()));
    assert_eq!(
        validate_use(&u, 0.25),
        Err(ArrangementError::InvalidBoundary)
    );
}

// ── intersections: line–line (geometry.rs 100–148) ──────────────────────

/// `b` is a vertical segment on integer coordinates; every orientation sign
/// below is the 2×2 determinant written out in the comment.
fn vertical(id: u64, x: f64, y0: f64, y1: f64) -> CurveUse {
    line(id, p(x, y0), p(x, y1), [id * 10, id * 10 + 1], None)
}
fn horizontal(id: u64, x0: f64, x1: f64, y: f64) -> CurveUse {
    line(id, p(x0, y), p(x1, y), [id * 10, id * 10 + 1], None)
}

#[test]
fn line_pairs_certified_disjoint_by_orientation_charge_no_work() {
    // b strictly above a's support: ar, as_ > 0 (first disjunct alone).
    assert_eq!(
        hits_unbudgeted(&horizontal(1, 0.0, 4.0, 0.0), &vertical(2, 2.0, 1.0, 3.0)),
        Ok(vec![])
    );
    // b strictly below a's support: ar, as_ < 0 (second disjunct alone).
    assert_eq!(
        hits_unbudgeted(&horizontal(1, 0.0, 4.0, 0.0), &vertical(2, 2.0, -3.0, -1.0)),
        Ok(vec![])
    );
    // a strictly left of b's upward support: bp, bq > 0 (third alone).
    assert_eq!(
        hits_unbudgeted(
            &horizontal(1, -2.0, -1.0, 0.0),
            &vertical(2, 0.0, -1.0, 1.0)
        ),
        Ok(vec![])
    );
    // a strictly right of b's upward support: bp, bq < 0 (fourth alone).
    assert_eq!(
        hits_unbudgeted(&horizontal(1, 1.0, 2.0, 0.0), &vertical(2, 0.0, -1.0, 1.0)),
        Ok(vec![])
    );
    // Parallel, offset supports.
    assert_eq!(
        hits_unbudgeted(&horizontal(1, 0.0, 2.0, 0.0), &horizontal(2, 0.0, 2.0, 1.0)),
        Ok(vec![])
    );
}

#[test]
fn line_endpoint_on_the_other_support_is_reported_at_the_exact_parameters() {
    // b starts on a's support (ar = 0) and ends below it (as_ < 0): the hit
    // is a's parameter of r = 1 and b's own start, 0.
    let a = horizontal(1, -1.0, 1.0, 0.0);
    let b = line(2, p(0.0, 0.0), p(0.0, -2.0), [20, 21], None);
    assert_hits(&hits(&a, &b).unwrap(), &[(1.0, 0.0)]);
    // a starts on b's support (bp = 0) with q on b's positive side (bq = 6).
    let a = line(1, p(0.0, 0.0), p(-3.0, 0.0), [10, 11], None);
    let b = vertical(2, 0.0, -1.0, 1.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(0.0, 1.0)]);
    // a ends on b's support (bq = 0) with p on the positive side (bp = 4).
    let a = horizontal(1, -2.0, 0.0, 0.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(2.0, 1.0)]);
    // a starts on b's support (bp = 0) with q on the negative side (bq = −4).
    let a = horizontal(1, 0.0, 2.0, 0.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(0.0, 1.0)]);
    // a ends on b's support (bq = 0) with p on the negative side (bp = −4).
    let a = line(1, p(2.0, 0.0), p(0.0, 0.0), [10, 11], None);
    assert_hits(&hits(&a, &b).unwrap(), &[(2.0, 1.0)]);
}

#[test]
fn transverse_line_crossings_solve_the_3_4_5_geometry() {
    // Horizontal 0..4 against vertical x = 3 from y = −4: crossing at (3, 0),
    // 3 along a and 4 along b.
    let a = horizontal(1, 0.0, 4.0, 0.0);
    let b = vertical(2, 3.0, -4.0, 4.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(3.0, 4.0)]);
    // Two length-10 diagonals of an 8×6 box cross at its centre, 5 along each.
    let a = line(1, p(0.0, 0.0), p(8.0, 6.0), [10, 11], None);
    let b = line(2, p(8.0, 0.0), p(0.0, 6.0), [20, 21], None);
    assert_hits(&hits(&a, &b).unwrap(), &[(5.0, 5.0)]);
    // Collinear segments touching end to start: one hit at a's end, b's start.
    let a = horizontal(1, 0.0, 2.0, 0.0);
    let b = line(2, p(2.0, 0.0), p(5.0, 0.0), [11, 21], None);
    assert_hits(&hits(&a, &b).unwrap(), &[(2.0, 0.0)]);
    // Collinear segments sharing an interval refuse.
    let b = line(2, p(1.0, 0.0), p(5.0, 0.0), [20, 21], None);
    assert_eq!(hits(&a, &b), Err(ArrangementError::AmbiguousOverlap));
}

/// Shared endpoint certificates (lines 237–244) snap a hit to the endpoint
/// parameters only when the hit lies within roundoff of BOTH uses' ends. A
/// certificate that agrees on one end but not the other must not move the
/// hit: snapping it would report a 2 apart pair as one point.
#[test]
fn endpoint_certificates_snap_only_when_both_ends_agree() {
    // Genuine shared corner: a ends where b starts, both carrying id 6.
    let a = line(1, p(0.0, 0.0), p(4.0, 0.0), [5, 6], None);
    let b = line(2, p(4.0, 0.0), p(4.0, 3.0), [6, 7], None);
    assert_eq!(hits(&a, &b).unwrap(), vec![(4.0, 0.0)]);
    // b starts on a's interior, yet claims a's start id 5: the hit is 2 from
    // a's start and must stay at (2, 0).
    let b = line(2, p(2.0, 0.0), p(2.0, 2.0), [5, 7], None);
    assert_eq!(hits(&a, &b).unwrap(), vec![(2.0, 0.0)]);
    // a starts on b's interior, yet claims b's start id 5: the hit is 2 from
    // b's start and must stay at (0, 2).
    let a = line(1, p(2.0, 0.0), p(6.0, 0.0), [5, 6], None);
    let b = line(2, p(2.0, -2.0), p(2.0, 2.0), [5, 7], None);
    assert_eq!(hits(&a, &b).unwrap(), vec![(0.0, 2.0)]);
}

// ── intersections: line–circle (geometry.rs 150–178) ────────────────────

#[test]
fn line_circle_roots_are_the_3_4_5_chord() {
    // y = 3 across the radius-5 circle: roots (∓4, 3), i.e. 2 and 10 along the
    // line from (−6, 3); angles π − atan2(3, 4) and atan2(3, 4).
    let a = horizontal(1, -6.0, 6.0, 3.0);
    let b = circle(2, p(0.0, 0.0), 5.0, None);
    let alpha = 3.0_f64.atan2(4.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(2.0, PI - alpha), (10.0, alpha)]);
    // The reversed pair swaps the coordinates and keeps the line's order.
    assert_hits(&hits(&b, &a).unwrap(), &[(PI - alpha, 2.0), (alpha, 10.0)]);
    // A chord clipped by the line's range keeps only the in-range root.
    let a = horizontal(1, -6.0, 0.0, 3.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(2.0, PI - alpha)]);
    // A line clear of the circle has no root.
    assert_eq!(hits(&horizontal(1, -6.0, 6.0, 6.0), &b), Ok(vec![]));
}

/// The tangency band is `64·ε·(proj² + |aa·cc| + r²)`. With the line origin
/// 8 to the side of the foot (proj = 8, proj² = 64) and radius 3 the band is
/// `64·ε·(64 + 64 + 9 + 6δ) ≈ 137·64·ε`, while the discriminant of a line
/// `δ` past the tangent height is `−6δ`. So `6δ = 133·64·ε` is inside the
/// band and `6δ = 300·64·ε` is outside it; the roundoff of the two
/// squarings is under one unit of `64·ε`.
#[test]
fn line_circle_tangency_band_is_the_stated_multiple_of_epsilon() {
    let unit = 64.0 * f64::EPSILON;
    let tangent_line = |delta: f64, range: [f64; 2]| {
        line_ranged(1, p(8.0, 3.0 + delta), Vec2::new(1.0, 0.0), range, [10, 11])
    };
    let b = circle(2, p(0.0, 0.0), 3.0, None);
    // Exactly tangent, foot at t = −8 inside the range: refused.
    assert_eq!(
        hits(&tangent_line(0.0, [-10.0, 10.0]), &b),
        Err(ArrangementError::AmbiguousContact)
    );
    // Same tangency with the foot outside the line's range: nothing to report.
    assert_eq!(hits(&tangent_line(0.0, [0.0, 5.0]), &b), Ok(vec![]));
    // 133 units inside the 137-unit band: still tangent.
    assert_eq!(
        hits(&tangent_line(133.0 * unit / 6.0, [-10.0, 10.0]), &b),
        Err(ArrangementError::AmbiguousContact)
    );
    // 300 units: past the band, and the line misses the circle.
    assert_eq!(
        hits(&tangent_line(300.0 * unit / 6.0, [-10.0, 10.0]), &b),
        Ok(vec![])
    );
    // A line one unit inside the tangent height cuts the circle at
    // x = ±√5: t = −8 ∓ √5 along the line from x = 8.
    let s5 = 5.0_f64.sqrt();
    let cut = hits(&tangent_line(-1.0, [-12.0, 10.0]), &b).unwrap();
    assert_hits(
        &cut,
        &[
            (-8.0 - s5, (2.0_f64).atan2(-s5)),
            (-8.0 + s5, 2.0_f64.atan2(s5)),
        ],
    );
}

// ── intersections: circle–circle (geometry.rs 185–224) ──────────────────

#[test]
fn circle_pair_crossings_are_the_3_4_5_points() {
    // Radius-5 circles 8 apart meet at (4, ±3): x = (25 − 25 + 64) / 16 = 4,
    // height² = 25 − 16 = 9.
    let a = circle(1, p(0.0, 0.0), 5.0, None);
    let b = circle(2, p(8.0, 0.0), 5.0, None);
    let alpha = 3.0_f64.atan2(4.0);
    assert_hits(
        &hits(&a, &b).unwrap(),
        &[(alpha, PI - alpha), (TAU - alpha, PI + alpha)],
    );
    // Radii 5 and 3 at distance 5 (internal 3-4-5): x = (25 − 9 + 25) / 10
    // = 4.1, height² = 25 − 16.81 = 8.19.
    let b = circle(2, p(5.0, 0.0), 3.0, None);
    let h = 8.19_f64.sqrt();
    let ya = h.atan2(4.1);
    let yb = h.atan2(4.1 - 5.0);
    assert_hits(&hits(&a, &b).unwrap(), &[(ya, yb), (TAU - ya, TAU - yb)]);
    // Concentric circles of different radii never meet.
    assert_eq!(hits(&a, &circle(2, p(0.0, 0.0), 3.0, None)), Ok(vec![]));
}

#[test]
fn external_tangency_refuses_only_when_the_contact_is_on_both_arcs() {
    // Radii 5 and 3 exactly 8 apart touch at (5, 0): angle 0 on a, π on b.
    let a = circle(1, p(0.0, 0.0), 5.0, None);
    let full = circle(2, p(8.0, 0.0), 3.0, None);
    assert_eq!(hits(&a, &full), Err(ArrangementError::AmbiguousContact));
    // b restricted to its left half still contains the contact.
    let left = arc(2, p(8.0, 0.0), 3.0, [FRAC_PI_2, 3.0 * FRAC_PI_2], [20, 21]);
    assert_eq!(hits(&a, &left), Err(ArrangementError::AmbiguousContact));
    // b restricted to its first quadrant does not: nothing to report.
    let quadrant = arc(2, p(8.0, 0.0), 3.0, [0.0, FRAC_PI_2], [20, 21]);
    assert_eq!(hits(&a, &quadrant), Ok(vec![]));
    // a restricted away from angle 0 does not either.
    let a_left = arc(1, p(0.0, 0.0), 5.0, [FRAC_PI_2, 3.0 * FRAC_PI_2], [10, 11]);
    assert_eq!(hits(&a_left, &full), Ok(vec![]));
}

/// Moving the radius-3 circle `δ` beyond external tangency (from d = 8) gives
/// x ≈ 5 + 0.375δ and height² ≈ −3.75δ, against a band of
/// `64·ε·(25 + x²) ≈ 50·64·ε`. Forty units is inside, three hundred outside.
#[test]
fn circle_circle_tangency_band_is_the_stated_multiple_of_epsilon() {
    let unit = 64.0 * f64::EPSILON;
    let a = circle(1, p(0.0, 0.0), 5.0, None);
    let at = |k: f64| circle(2, p(8.0 + k * unit / 3.75, 0.0), 3.0, None);
    assert_eq!(hits(&a, &at(40.0)), Err(ArrangementError::AmbiguousContact));
    assert_eq!(
        hits(&a, &at(-40.0)),
        Err(ArrangementError::AmbiguousContact)
    );
    assert_eq!(hits(&a, &at(300.0)), Ok(vec![]));
    // Pulled 300 units inward the circles overlap: two crossings.
    assert_eq!(hits(&a, &at(-300.0)).unwrap().len(), 2);
}

#[test]
fn circle_pair_with_an_overflowing_radius_is_refused_as_non_finite() {
    // ra² overflows, so x and height² are non-finite while d = 8 is finite.
    let a = circle(1, p(0.0, 0.0), 1.0e200, None);
    let b = circle(2, p(8.0, 0.0), 3.0, None);
    assert_eq!(hits(&a, &b), Err(ArrangementError::NonFiniteInput));
}

#[test]
fn coincident_carrier_arcs_meet_only_at_a_shared_native_parameter() {
    // Adjacent quarter arcs: the shared parameter π/2 is one exact event.
    let a = arc(1, p(0.0, 0.0), 2.0, [0.0, FRAC_PI_2], [10, 11]);
    let b = arc(2, p(0.0, 0.0), 2.0, [FRAC_PI_2, PI], [11, 12]);
    assert_eq!(hits(&a, &b).unwrap(), vec![(FRAC_PI_2, FRAC_PI_2)]);
    // Arcs meeting across the seam: 0 on a is 2π on b.
    let b = arc(2, p(0.0, 0.0), 2.0, [3.0 * FRAC_PI_2, TAU], [12, 10]);
    assert_eq!(hits(&a, &b).unwrap(), vec![(0.0, TAU)]);
    // Any shared interval, however small, is an overlap.
    let b = arc(2, p(0.0, 0.0), 2.0, [FRAC_PI_2 - 1e-9, PI], [11, 12]);
    assert_eq!(hits(&a, &b), Err(ArrangementError::AmbiguousOverlap));
    // Disjoint arcs on one carrier have no event.
    let b = arc(2, p(0.0, 0.0), 2.0, [2.0, 3.0], [11, 12]);
    assert_eq!(hits(&a, &b), Ok(vec![]));
}

// ── tangent_order (geometry.rs 261–279) ─────────────────────────────────

/// A fan of unit half-edges leaving the origin at the given angles (degrees);
/// `reversed` entries traverse their use backwards, so their tangent is the
/// opposite direction.
fn fan(angles_deg: &[(f64, bool)]) -> (Vec<CurveUse>, Vec<ArrangementHalfEdge>) {
    let uses: Vec<CurveUse> = angles_deg
        .iter()
        .enumerate()
        .map(|(i, &(deg, _))| {
            let d = Vec2::new(deg.to_radians().cos(), deg.to_radians().sin());
            line_ranged(i as u64, p(0.0, 0.0), d, [0.0, 1.0], [0, 100 + i as u64])
        })
        .collect();
    let halves = angles_deg
        .iter()
        .enumerate()
        .map(|(i, &(_, reversed))| ArrangementHalfEdge {
            from: 0,
            to: 1,
            twin: 0,
            next: 0,
            source: uses[i].source.clone(),
            range: if reversed { [1.0, 0.0] } else { [0.0, 1.0] },
            source_range: [0.0, 1.0],
            endpoints_3d: uses[i].endpoints_3d,
        })
        .collect();
    (uses, halves)
}

#[test]
fn tangent_order_is_upper_half_plane_first_then_counterclockwise() {
    // Compass fan in a scrambled input order; 180° is the negative x axis,
    // which belongs to the second (lower) half.
    let angles = [
        (135.0, false),
        (0.0, false),
        (270.0, false),
        (45.0, false),
        (315.0, false),
        (180.0, false),
        (90.0, false),
        (225.0, false),
    ];
    let (uses, halves) = fan(&angles);
    let table = lookup(&uses);
    let mut order: Vec<usize> = (0..halves.len()).collect();
    order.sort_by(|&a, &b| tangent_order(&halves[a], &halves[b], &table));
    let sorted: Vec<f64> = order.iter().map(|&i| angles[i].0).collect();
    assert_eq!(sorted, [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]);
    // Every adjacent pair compares strictly, both ways.
    for w in order.windows(2) {
        assert_eq!(
            tangent_order(&halves[w[0]], &halves[w[1]], &table),
            Ordering::Less
        );
        assert_eq!(
            tangent_order(&halves[w[1]], &halves[w[0]], &table),
            Ordering::Greater
        );
    }
    // 0° (upper half, on the axis) sorts before 180° (lower half) and before
    // 359°; 180° sorts before 181°.
    let (uses, halves) = fan(&[(0.0, false), (180.0, false), (359.0, false), (181.0, false)]);
    let table = lookup(&uses);
    assert_eq!(
        tangent_order(&halves[0], &halves[1], &table),
        Ordering::Less
    );
    assert_eq!(
        tangent_order(&halves[0], &halves[2], &table),
        Ordering::Less
    );
    assert_eq!(
        tangent_order(&halves[1], &halves[3], &table),
        Ordering::Less
    );
    assert_eq!(
        tangent_order(&halves[3], &halves[1], &table),
        Ordering::Greater
    );
}

#[test]
fn parallel_tangents_compare_equal_and_reversal_flips_the_half_plane() {
    // Two distinct uses along +x: Equal both ways (the caller refuses this).
    let (uses, halves) = fan(&[(0.0, false), (0.0, false), (90.0, false), (90.0, true)]);
    let table = lookup(&uses);
    assert_eq!(
        tangent_order(&halves[0], &halves[1], &table),
        Ordering::Equal
    );
    assert_eq!(
        tangent_order(&halves[1], &halves[0], &table),
        Ordering::Equal
    );
    // A reversed +y use points along −y: lower half, after every upper one.
    assert_eq!(
        tangent_order(&halves[2], &halves[3], &table),
        Ordering::Less
    );
    assert_eq!(
        tangent_order(&halves[3], &halves[2], &table),
        Ordering::Greater
    );
    assert_eq!(
        tangent_order(&halves[3], &halves[0], &table),
        Ordering::Greater
    );
}

// ── ray_crossing (geometry.rs 296–319) ──────────────────────────────────

#[test]
fn ray_crossing_counts_half_open_monotone_spans_strictly_right_of_the_probe() {
    // Vertical segment x = 2, y ∈ [0, 2], traversed upward (range [0, 2]) and
    // downward (range [2, 0]). Rows: probe, upward count, downward count.
    let seg = vertical(1, 2.0, 0.0, 2.0);
    let table: [((f64, f64), i32, i32); 7] = [
        ((0.0, 1.0), 1, -1), // interior height, probe left of the span
        ((3.0, 1.0), 0, 0),  // probe right of the span
        ((2.0, 1.0), 0, 0),  // probe exactly on the span: not strictly right
        ((0.0, 0.0), 1, -1), // bottom end is included (a.y ≤ p.y)
        ((0.0, 2.0), 0, 0),  // top end is excluded (b.y > p.y fails)
        ((0.0, -1.0), 0, 0), // below the span
        ((0.0, 3.0), 0, 0),  // above the span
    ];
    for ((x, y), up, down) in table {
        assert_eq!(
            ray_crossing(&seg, [0.0, 2.0], p(x, y)),
            up,
            "up at ({x}, {y})"
        );
        assert_eq!(
            ray_crossing(&seg, [2.0, 0.0], p(x, y)),
            down,
            "down at ({x}, {y})"
        );
    }
    // The diagonal (0,0)→(4,4): at height 3 it sits at x = 3.
    let diag = line(2, p(0.0, 0.0), p(4.0, 4.0), [20, 21], None);
    let len = 32.0_f64.sqrt();
    assert_eq!(ray_crossing(&diag, [0.0, len], p(1.0, 3.0)), 1);
    assert_eq!(ray_crossing(&diag, [0.0, len], p(3.0, 3.0)), 0);
    assert_eq!(ray_crossing(&diag, [0.0, len], p(3.5, 3.0)), 0);
    assert_eq!(ray_crossing(&diag, [len, 0.0], p(1.0, 3.0)), -1);
}

#[test]
fn ray_crossing_on_cardinal_arcs_uses_the_3_4_5_height() {
    // Radius-5 circle; at height 4 the right quadrant sits at x = 3 and the
    // left quadrant at x = −3.
    let c = circle(1, p(0.0, 0.0), 5.0, None);
    let right = [0.0, FRAC_PI_2];
    let left = [FRAC_PI_2, PI];
    assert_eq!(ray_crossing(&c, right, p(0.0, 4.0)), 1);
    assert_eq!(ray_crossing(&c, right, p(2.999_999, 4.0)), 1);
    assert_eq!(ray_crossing(&c, right, p(3.0, 4.0)), 0);
    assert_eq!(ray_crossing(&c, right, p(3.5, 4.0)), 0);
    assert_eq!(ray_crossing(&c, [FRAC_PI_2, 0.0], p(0.0, 4.0)), -1);
    // Height 0 is the right quadrant's start (included), height 5 its end
    // (excluded).
    assert_eq!(ray_crossing(&c, right, p(0.0, 0.0)), 1);
    assert_eq!(ray_crossing(&c, right, p(0.0, 5.0)), 0);
    // Left quadrant runs downward from (0, 5) to (−5, 0).
    assert_eq!(ray_crossing(&c, left, p(-4.0, 4.0)), -1);
    assert_eq!(ray_crossing(&c, left, p(-3.0, 4.0)), 0);
    assert_eq!(ray_crossing(&c, left, p(-2.0, 4.0)), 0);
    assert_eq!(ray_crossing(&c, [PI, FRAC_PI_2], p(-4.0, 4.0)), 1);
}

// ── distance (geometry.rs 321–339) ──────────────────────────────────────

#[test]
fn distance_clamps_to_the_span_and_picks_the_nearer_arc_end() {
    let seg = horizontal(1, 0.0, 10.0, 0.0);
    assert_eq!(distance(&seg, [0.0, 10.0], p(3.0, 4.0)), 4.0);
    assert_eq!(distance(&seg, [0.0, 10.0], p(-3.0, 4.0)), 5.0);
    assert_eq!(distance(&seg, [0.0, 10.0], p(13.0, 4.0)), 5.0);
    assert_eq!(distance(&seg, [2.0, 8.0], p(0.0, 0.0)), 2.0);
    assert_eq!(distance(&seg, [8.0, 2.0], p(0.0, 0.0)), 2.0);
    // Unit quarter arc from (1, 0) to (0, 1).
    let c = circle(2, p(0.0, 0.0), 1.0, None);
    let q = [0.0, FRAC_PI_2];
    near(distance(&c, q, p(2.0, 2.0)), 8.0_f64.sqrt() - 1.0, 1e-12);
    // Angle 7π/4 is outside the arc: the start (1, 0) is at distance 1, the
    // end (0, 1) at √5.
    near(distance(&c, q, p(1.0, -1.0)), 1.0, 1e-12);
    near(distance(&c, [FRAC_PI_2, 0.0], p(1.0, -1.0)), 1.0, 1e-12);
    // Angle 3π/4: the end (0, 1) is the nearer one.
    near(distance(&c, q, p(-1.0, 1.0)), 1.0, 1e-12);
    // Equidistant from both ends: √5 either way.
    near(distance(&c, q, p(-1.0, -1.0)), 5.0_f64.sqrt(), 1e-12);
    // Angle inside the arc, point inside the circle.
    near(distance(&c, q, p(0.3, 0.4)), 0.5, 1e-12);
}

// ── arrangement.rs: parameter maps, work accounting, refusals ───────────

#[test]
fn source_parameter_and_point_3d_are_affine_on_a_range_not_starting_at_zero() {
    let mut u = line_ranged(1, p(0.0, 0.0), Vec2::new(1.0, 0.0), [2.0, 6.0], [1, 2]);
    u.source_range = [10.0, 20.0];
    u.endpoints_3d = [Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 8.0, 12.0)];
    assert_eq!(u.source_parameter(2.0), 10.0);
    assert_eq!(u.source_parameter(6.0), 20.0);
    assert_eq!(u.source_parameter(3.0), 12.5);
    assert_eq!(u.source_parameter(5.0), 17.5);
    assert_eq!(u.point_3d(2.0), u.endpoints_3d[0]);
    assert_eq!(u.point_3d(6.0), u.endpoints_3d[1]);
    assert_eq!(u.point_3d(3.0), Point3::new(1.0, 2.0, 3.0));
    assert_eq!(u.point_3d(5.0), Point3::new(3.0, 6.0, 9.0));
    // A circle use maps through its source angle: quarter arc [1, 2] on the
    // pcurve is angles [1, 2] on the 3D circle.
    let c = arc(2, p(0.0, 0.0), 2.0, [1.0, 2.0], [20, 21]);
    let mid = c.point_3d(1.5);
    near(mid.x(), 2.0 * 1.5_f64.cos(), 1e-12);
    near(mid.y(), 2.0 * 1.5_f64.sin(), 1e-12);
}

#[test]
fn uv_distance_scales_u_by_the_domain_radius() {
    let ctx = context();
    let mut work = Work::new(&ctx);
    assert_eq!(work.uv_distance(p(0.0, 0.0), p(3.0, 4.0)), 5.0);
    assert_eq!(work.uv_distance(p(1.0, 1.0), p(1.0, 1.0)), 0.0);
    work.u_scale = 2.0;
    near(
        work.uv_distance(p(0.0, 0.0), p(3.0, 4.0)),
        52.0_f64.sqrt(),
        1e-12,
    );
    near(
        work.uv_distance(p(3.0, 4.0), p(0.0, 0.0)),
        52.0_f64.sqrt(),
        1e-12,
    );
    assert_eq!(work.uv_distance(p(0.0, 0.0), p(0.0, 4.0)), 4.0);
    assert_eq!(work.uv_distance(p(0.0, 0.0), p(3.0, 0.0)), 6.0);
}

#[test]
fn capacity_accepts_the_queue_size_itself_and_charges_one_step() {
    let ctx = OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_march_steps(3)
            .with_queue_size(5)
            .with_segments(10),
    );
    let mut work = Work::new(&ctx);
    assert_eq!(work.capacity(5), Ok(()));
    assert_eq!(work.capacity(6), Err(ArrangementError::WorkBudgetExceeded));
    assert_eq!(work.capacity(0), Ok(()));
    // Three steps are spent: the fourth call fails on the march budget.
    assert_eq!(work.capacity(0), Err(ArrangementError::WorkBudgetExceeded));
}

#[test]
fn unsupported_strip_radii_are_refused_before_any_work_is_charged() {
    let uses = rectangle(0, 0.0, 0.0, TAU, 3.0);
    let zero = budget(0);
    for radius in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            build_arrangement(&ArrangementInput {
                uses: &uses,
                domain: ParamDomain::CylinderStrip {
                    seam_uses: [3, 1],
                    radius,
                },
                context: &zero,
            })
            .unwrap_err(),
            ArrangementError::UnsupportedDomain,
            "radius {radius}"
        );
    }
    // The same budget is exhausted at once by a supported domain.
    assert_eq!(
        build_with(&uses, &zero).unwrap_err(),
        ArrangementError::WorkBudgetExceeded
    );
    assert_eq!(
        build_arrangement(&ArrangementInput {
            uses: &uses,
            domain: ParamDomain::CylinderStrip {
                seam_uses: [3, 1],
                radius: 2.0,
            },
            context: &zero,
        })
        .unwrap_err(),
        ArrangementError::WorkBudgetExceeded
    );
}

#[test]
fn duplicate_use_ids_are_an_invalid_boundary() {
    let mut uses = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    // A section reusing side 2's id, with its own endpoint certificates.
    uses.push(line(2, p(0.0, 1.0), p(2.0, 1.0), [20, 21], None));
    assert_eq!(build(&uses).err(), Some(ArrangementError::InvalidBoundary));
    // The same section under a fresh id splits the square in two.
    uses[4].source.use_id = 9;
    let a = build(&uses).unwrap();
    assert_eq!(a.regions.iter().filter(|r| r.material).count(), 2);
}

/// A shared endpoint certificate tolerates a positional gap of exactly
/// `tolerance.linear` (2D, then 3D) and refuses one any larger.
#[test]
fn shared_endpoint_gaps_at_exactly_the_tolerance_are_accepted() {
    let make = |uv_gap: f64, xyz_gap: f64| {
        let mut uses = vec![
            line(0, p(0.0, 0.0), p(2.0, 0.0), [0, 1], Some(0)),
            line(1, p(2.0, uv_gap), p(2.0, 2.0), [1, 2], Some(0)),
            line(2, p(2.0, 2.0), p(0.0, 2.0), [2, 3], Some(0)),
            line(3, p(0.0, 2.0), p(0.0, 0.0), [3, 0], Some(0)),
        ];
        // Endpoint 1 is (2, 0, 0) on use 0 and (2, 0, xyz_gap) on use 1.
        uses[0].endpoints_3d[1] = Point3::new(2.0, 0.0, 0.0);
        uses[1].endpoints_3d[0] = Point3::new(2.0, 0.0, xyz_gap);
        uses
    };
    let with_tol = |linear: f64| {
        context().with_tolerance(Tolerance {
            linear,
            ..Tolerance::default()
        })
    };
    // 1/16 keeps the gap below the cycle's interior seed height (1/8), so
    // the square with a notched corner still classifies as one region.
    let gap = 0.0625;
    for (uv_gap, xyz_gap) in [(gap, 0.0), (0.0, gap)] {
        let uses = make(uv_gap, xyz_gap);
        let a = build_with(&uses, &with_tol(gap)).unwrap();
        assert_eq!(a.regions.len(), 1, "gap ({uv_gap}, {xyz_gap})");
        assert_eq!(
            build_with(&uses, &with_tol(gap - 1e-12)).err(),
            Some(ArrangementError::InvalidBoundary),
            "gap ({uv_gap}, {xyz_gap})"
        );
    }
    // Far apart at the default tolerance: refused as a boundary defect, not
    // as a refinement failure.
    for (uv_gap, xyz_gap) in [(1.0, 0.0), (0.0, 1.0)] {
        assert_eq!(
            build(&make(uv_gap, xyz_gap)).err(),
            Some(ArrangementError::InvalidBoundary),
            "gap ({uv_gap}, {xyz_gap})"
        );
    }
}

/// A full circle needs exactly four event records: its seam vertex and the
/// three cardinal cuts strictly inside `(0, 2π)`. A quarter arc whose ends
/// ARE cardinal parameters needs none beyond its two ends and the two
/// end-on-end hits with its chord.
#[test]
fn cardinal_cuts_are_the_interior_multiples_of_a_quarter_turn() {
    let with_queue = |n: usize| {
        OperationContext::new().with_budgets(
            WorkBudgets::new()
                .with_march_steps(100_000)
                .with_queue_size(n)
                .with_segments(1_000),
        )
    };
    let disc = vec![circle(0, p(0.0, 0.0), 1.0, Some(0))];
    let a = build_with(&disc, &with_queue(4)).unwrap();
    assert_eq!(a.vertices.len(), 4);
    let mut xs: Vec<(i32, i32)> = a
        .vertices
        .iter()
        .map(|v| {
            (
                (v.uv.x() * 2.0).round() as i32,
                (v.uv.y() * 2.0).round() as i32,
            )
        })
        .collect();
    xs.sort_unstable();
    assert_eq!(xs, [(-2, 0), (0, -2), (0, 2), (2, 0)]);
    for v in &a.vertices {
        near(v.uv.x().abs() + v.uv.y().abs(), 1.0, 1e-12);
    }
    assert_eq!(
        build_with(&disc, &with_queue(3)).err(),
        Some(ArrangementError::WorkBudgetExceeded)
    );
    // Quarter arc [π/2, π] closed by its chord: two events only.
    let mut quarter = arc(0, p(0.0, 0.0), 2.0, [FRAC_PI_2, PI], [0, 1]);
    quarter.boundary_loop = Some(0);
    let chord = line(1, p(-2.0, 0.0), p(0.0, 2.0), [1, 0], Some(0));
    let sector = vec![quarter, chord];
    let a = build_with(&sector, &with_queue(4)).unwrap();
    assert_eq!(a.vertices.len(), 2);
    near(a.regions[0].area, PI - 2.0, 1e-12);
    assert_eq!(
        build_with(&sector, &with_queue(3)).err(),
        Some(ArrangementError::WorkBudgetExceeded)
    );
}

#[test]
fn segment_budget_is_the_number_of_undirected_edges() {
    let with_segments = |n: usize| {
        OperationContext::new().with_budgets(
            WorkBudgets::new()
                .with_march_steps(100_000)
                .with_queue_size(1_000)
                .with_segments(n),
        )
    };
    let square = rectangle(0, 0.0, 0.0, 2.0, 2.0);
    let a = build_with(&square, &with_segments(4)).unwrap();
    assert_eq!(a.half_edges.len(), 8);
    assert_eq!(
        build_with(&square, &with_segments(3)).err(),
        Some(ArrangementError::WorkBudgetExceeded)
    );
}

/// An interior 3D point with exactly one overflowing coordinate is refused as
/// non-finite, not as a refinement failure: the square's 3D corners sit at
/// ±1e308, so every interior point on a side overflows in one axis only.
#[test]
fn a_single_overflowing_3d_coordinate_is_refused_as_non_finite() {
    let big = 1.0e308;
    let mut uses = rectangle(0, -2.0, -2.0, 2.0, 2.0);
    let corner = |q: Point2| Point3::new(q.x().signum() * big, q.y().signum() * big, 0.0);
    for u in &mut uses {
        u.endpoints_3d = [corner(u.point(u.range[0])), corner(u.point(u.range[1]))];
    }
    let mut section = line(30, p(-2.0, 0.0), p(2.0, 0.0), [30, 31], None);
    section.endpoints_3d = [Point3::new(-big, 0.0, 0.0), Point3::new(big, 0.0, 0.0)];
    uses.push(section);
    // The right side's midpoint: x stays 1e308, y overflows.
    let mid = uses[1].point_3d(2.0);
    assert!(mid.x().is_finite() && !mid.y().is_finite() && mid.z().is_finite());
    assert_eq!(build(&uses).err(), Some(ArrangementError::NonFiniteInput));
}

// ── periodic quotient (periodic.rs) ─────────────────────────────────────

/// The lifted chart of a radius-2 cylinder strip `u ∈ [0, 2π], v ∈ [0, 3]`:
/// rims are 3D circles, everything else a ruling line. `right_x` places the
/// right seam (normally `2π`); `seam_up` draws the left seam bottom-to-top
/// and the right seam top-to-bottom instead of the loop's own sense.
fn strip(right_x: f64, seam_up: bool, rulings: &[f64], bands: &[f64]) -> Vec<CurveUse> {
    let (x0, x1, y0, y1) = (0.0, right_x, 0.0, 3.0);
    let mut uses = vec![
        line(0, p(x0, y0), p(x1, y0), [0, 1], Some(0)),
        if seam_up {
            line(1, p(x1, y1), p(x1, y0), [2, 1], Some(0))
        } else {
            line(1, p(x1, y0), p(x1, y1), [1, 2], Some(0))
        },
        line(2, p(x1, y1), p(x0, y1), [2, 3], Some(0)),
        if seam_up {
            line(3, p(x0, y0), p(x0, y1), [0, 3], Some(0))
        } else {
            line(3, p(x0, y1), p(x0, y0), [3, 0], Some(0))
        },
    ];
    for (i, &u) in rulings.iter().enumerate() {
        uses.push(line(
            10 + i as u64,
            p(u, y0),
            p(u, y1),
            [100 + i as u64 * 2, 101 + i as u64 * 2],
            None,
        ));
    }
    for (i, &v) in bands.iter().enumerate() {
        uses.push(line(
            30 + i as u64,
            p(x0, v),
            p(x1, v),
            [300 + i as u64 * 2, 301 + i as u64 * 2],
            None,
        ));
    }
    lift_to_cylinder(&mut uses);
    uses
}
fn lift_to_cylinder(uses: &mut [CurveUse]) {
    for u in uses.iter_mut() {
        let a = u.point(u.range[0]);
        let b = u.point(u.range[1]);
        u.endpoints_3d = [
            Point3::new(a.x().cos() * 2.0, a.x().sin() * 2.0, a.y()),
            Point3::new(b.x().cos() * 2.0, b.x().sin() * 2.0, b.y()),
        ];
        if (a.y() - b.y()).abs() < 1e-12 {
            u.curve_3d = EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, a.y()),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            );
            u.source_range = [a.x(), b.x()];
        }
    }
}
fn cylinder_domain() -> ParamDomain {
    ParamDomain::CylinderStrip {
        seam_uses: [3, 1],
        radius: 2.0,
    }
}
fn build_cylinder(uses: &[CurveUse], ctx: &OperationContext) -> Result<Arrangement> {
    build_arrangement(&ArrangementInput {
        uses,
        domain: cylinder_domain(),
        context: ctx,
    })
}
/// Build the planar arrangement, then run the quotient on its own with the
/// given use table, domain and work context.
fn quotient_of(
    uses: &[CurveUse],
    table: &[CurveUse],
    domain: ParamDomain,
    ctx: &OperationContext,
) -> Result<Arrangement> {
    let mut a = build(uses)?;
    let mut work = Work::new(ctx);
    super::periodic::quotient(&mut a, &lookup(table), domain, &mut work)?;
    Ok(a)
}
fn seams(left: u64, right: u64) -> ParamDomain {
    ParamDomain::CylinderStrip {
        seam_uses: [left, right],
        radius: 2.0,
    }
}
/// Winding of the boundary cycle that contains a half-edge of `use_id`.
fn winding_of(a: &Arrangement, region: &PeriodicRegion, use_id: u64) -> i32 {
    region
        .boundaries
        .iter()
        .find(|(cycle, _)| {
            cycle
                .iter()
                .any(|&h| a.half_edges[h].source.use_id == use_id)
        })
        .unwrap_or_else(|| panic!("use {use_id} is on no boundary"))
        .1
}

#[test]
fn plain_strip_is_one_annulus_with_signed_rim_windings() {
    for seam_up in [false, true] {
        let uses = strip(TAU, seam_up, &[], &[]);
        let a = build_cylinder(&uses, &context()).unwrap();
        assert_eq!(a.periodic_regions.len(), 1, "seam_up {seam_up}");
        let r = &a.periodic_regions[0];
        assert_eq!(r.cells.len(), 1);
        assert_eq!(r.euler_characteristic, 0);
        assert_eq!(r.boundaries.len(), 2);
        near(r.area, 12.0 * PI, 1e-9);
        // The bottom rim is traversed in +u by the material cycle, the top
        // rim in −u.
        assert_eq!(winding_of(&a, r, 0), 1, "seam_up {seam_up}");
        assert_eq!(winding_of(&a, r, 2), -1, "seam_up {seam_up}");
        // One identification per rim height, each a unit lift.
        assert_eq!(a.identifications.len(), 2);
    }
}

/// Ready-repro (found 2026-09-29 while writing these oracles, not by a
/// mutant): a strip cut by ONE ruling is two cells joined across the seam
/// into one annulus (χ = 0, two rim boundaries), but `quotient` treats every
/// non-seam half-edge as boundary, so the ruling's two halves, both inside
/// the joined region, are walked as a slit: the rim cycles merge into one
/// zero-winding cycle and the χ check refuses with `NonManifoldEmbedding`.
/// Two rulings pass only because each ruling then has one half per region.
#[test]
#[ignore = "open: quotient refuses a strip cut by a single ruling (interior edge shared by two cells of one periodic region is walked as boundary)"]
fn one_ruling_joins_two_sectors_into_one_annulus() {
    let uses = strip(TAU, false, &[3.0], &[]);
    let a = build_cylinder(&uses, &context()).unwrap();
    assert_eq!(a.periodic_regions.len(), 1);
    let r = &a.periodic_regions[0];
    assert_eq!(r.cells.len(), 2);
    // V = 4 (two seam heights identified plus the ruling's ends), E = 6
    // (four rim pieces, the ruling once, the seam once), F = 2.
    assert_eq!(r.euler_characteristic, 0);
    assert_eq!(r.boundaries.len(), 2);
    assert_eq!(winding_of(&a, r, 0), 1);
    assert_eq!(winding_of(&a, r, 2), -1);
    near(r.area, 12.0 * PI, 1e-9);
}

/// The accepted shape of the same geometry today: two rulings, so each
/// ruling has one half per periodic region.
#[test]
fn two_rulings_give_a_joined_sector_and_a_separate_one() {
    let uses = strip(TAU, false, &[1.0, 3.0], &[]);
    let a = build_cylinder(&uses, &context()).unwrap();
    assert_eq!(a.periodic_regions.len(), 2);
    let joined = a
        .periodic_regions
        .iter()
        .find(|r| r.cells.len() == 2)
        .unwrap();
    let lone = a
        .periodic_regions
        .iter()
        .find(|r| r.cells.len() == 1)
        .unwrap();
    for r in [joined, lone] {
        assert_eq!(r.euler_characteristic, 1);
        assert_eq!(r.boundaries.len(), 1);
        assert_eq!(r.boundaries[0].1, 0);
    }
    // Sector widths 2 and 2π − 2, height 3, radius 2.
    near(lone.area, 12.0, 1e-9);
    near(joined.area, 6.0 * (TAU - 2.0), 1e-9);
    assert_eq!(a.identifications.len(), 2);
}

#[test]
fn one_band_identifies_three_seam_heights_once_each() {
    let uses = strip(TAU, false, &[], &[1.5]);
    let a = build_cylinder(&uses, &context()).unwrap();
    assert_eq!(a.periodic_regions.len(), 2);
    let mut heights: Vec<f64> = a
        .identifications
        .iter()
        .map(|id| {
            let [x, y] = id.vertices;
            assert_eq!(id.u_lift, 1);
            near(a.vertices[x].uv.x(), 0.0, 1e-12);
            near(a.vertices[y].uv.x(), TAU, 1e-12);
            assert_eq!(a.vertices[x].uv.y(), a.vertices[y].uv.y());
            a.vertices[x].uv.y()
        })
        .collect();
    heights.sort_by(f64::total_cmp);
    assert_eq!(heights, [0.0, 1.5, 3.0]);
    for r in &a.periodic_regions {
        assert_eq!(r.euler_characteristic, 0);
        assert_eq!(r.boundaries.len(), 2);
    }
    let mut areas: Vec<f64> = a.periodic_regions.iter().map(|r| r.area).collect();
    areas.sort_by(f64::total_cmp);
    near(areas[0], 6.0 * PI, 1e-9);
    near(areas[1], 6.0 * PI, 1e-9);
}

#[test]
fn quotient_domain_refusals_happen_before_any_work_is_charged() {
    let uses = strip(TAU, false, &[], &[]);
    let zero = budget(0);
    for domain in [
        ParamDomain::CylinderStrip {
            seam_uses: [3, 1],
            radius: -1.0,
        },
        ParamDomain::CylinderStrip {
            seam_uses: [3, 1],
            radius: 0.0,
        },
        ParamDomain::CylinderStrip {
            seam_uses: [3, 1],
            radius: f64::NAN,
        },
        seams(3, 3),
        seams(3, 99),
        // Left seam is the bottom rim: not vertical.
        seams(0, 1),
        // Right seam is the top rim: not vertical (spacing is still 2π).
        seams(3, 2),
    ] {
        assert_eq!(
            quotient_of(&uses, &uses, domain, &zero).err(),
            Some(ArrangementError::UnsupportedDomain),
            "{domain:?}"
        );
    }
    // Loop-key disjuncts, read from a table whose seam entries were altered.
    let mut no_loop = uses.clone();
    no_loop[3].boundary_loop = None;
    assert_eq!(
        quotient_of(&uses, &no_loop, cylinder_domain(), &zero).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    let mut other_loop = uses.clone();
    other_loop[1].boundary_loop = Some(1);
    assert_eq!(
        quotient_of(&uses, &other_loop, cylinder_domain(), &zero).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    // Seam spacing 5 instead of 2π.
    let narrow = strip(5.0, false, &[], &[]);
    assert_eq!(
        quotient_of(&narrow, &narrow, cylinder_domain(), &zero).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    // The well-formed strip spends its first step immediately.
    assert_eq!(
        quotient_of(&uses, &uses, cylinder_domain(), &zero).err(),
        Some(ArrangementError::WorkBudgetExceeded)
    );
}

/// The seam spacing check tolerates a miss of exactly `roundoff(right
/// origin)`. With the right seam's origin at `(2π + 2⁻⁴³, y₀)` and
/// `y₀ = 8 − (1 + 2π + 2⁻⁴³)`, the bound `64·ε·(1 + x + |y|)` is exactly
/// `2⁻⁴⁶ · 8 = 2⁻⁴³`: the spacing passes (and the strip is then refused,
/// after work, by the rim winding check, whose bound has no `y` term).
#[test]
fn seam_spacing_exactly_at_the_roundoff_bound_passes_the_spacing_check() {
    let k = 2.0_f64.powi(-43);
    let x1 = TAU + k;
    let y0 = 8.0 - (1.0 + x1);
    assert_eq!(1.0 + x1 + y0.abs(), 8.0);
    assert_eq!(roundoff(p(x1, y0)), k);
    let mut uses = vec![
        line(0, p(0.0, y0), p(x1, y0), [0, 1], Some(0)),
        line(1, p(x1, y0), p(x1, 3.0), [1, 2], Some(0)),
        line(2, p(x1, 3.0), p(0.0, 3.0), [2, 3], Some(0)),
        line(3, p(0.0, 3.0), p(0.0, y0), [3, 0], Some(0)),
    ];
    lift_to_cylinder(&mut uses);
    // No work is charged before the spacing check, so with a zero budget the
    // first refusal is the budget itself: the spacing was accepted.
    assert_eq!(
        quotient_of(&uses, &uses, cylinder_domain(), &budget(0)).err(),
        Some(ArrangementError::WorkBudgetExceeded)
    );
    assert_eq!(
        quotient_of(&uses, &uses, cylinder_domain(), &context()).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    // One ulp of 2π wider and the spacing check itself refuses at once.
    let mut wider = uses.clone();
    let x2 = x1 + 2.0_f64.powi(-50);
    wider[0] = line(0, p(0.0, y0), p(x2, y0), [0, 1], Some(0));
    wider[1] = line(1, p(x2, y0), p(x2, 3.0), [1, 2], Some(0));
    wider[2] = line(2, p(x2, 3.0), p(0.0, 3.0), [2, 3], Some(0));
    lift_to_cylinder(&mut wider);
    assert_eq!(
        quotient_of(&wider, &wider, cylinder_domain(), &budget(0)).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
}

#[test]
fn material_outside_the_seam_interval_is_refused_on_either_side() {
    for (x0, x1) in [(-3.0, -1.0), (7.0, 8.0)] {
        let mut uses = strip(TAU, false, &[], &[]);
        let mut island = rectangle(60, x0, 1.0, x1, 2.0);
        lift_to_cylinder(&mut island);
        uses.extend(island);
        assert_eq!(
            quotient_of(&uses, &uses, cylinder_domain(), &context()).err(),
            Some(ArrangementError::UnsupportedDomain),
            "island at [{x0}, {x1}]"
        );
    }
    // The same island inside the strip is a separate disc region.
    let mut uses = strip(TAU, false, &[], &[]);
    let mut island = rectangle(60, 1.0, 1.0, 2.0, 2.0);
    lift_to_cylinder(&mut island);
    uses.extend(island);
    let a = build_cylinder(&uses, &context()).unwrap();
    assert_eq!(a.periodic_regions.len(), 1);
    assert_eq!(a.periodic_regions[0].euler_characteristic, -1);
}

/// Seam-paired vertices must agree in `v` exactly and in 3D within the
/// tolerance; a 3D gap of exactly the tolerance is accepted. The rims are
/// carried as 3D lines here so that the right column can sit `g` higher in
/// 3D while every use stays self-consistent.
#[test]
fn seam_vertex_3d_gap_is_accepted_up_to_exactly_the_tolerance() {
    let with_gap = |g: f64| {
        let mut uses = strip(TAU, false, &[], &[]);
        for u in &mut uses {
            u.curve_3d = EdgeCurve::Line;
            u.source_range = [0.0, 1.0];
            for end in 0..2 {
                if u.endpoints[end] == 1 || u.endpoints[end] == 2 {
                    let e = u.endpoints_3d[end];
                    u.endpoints_3d[end] = Point3::new(e.x(), e.y(), e.z() + g);
                }
            }
        }
        uses
    };
    let with_tol = |linear: f64| {
        context().with_tolerance(Tolerance {
            linear,
            ..Tolerance::default()
        })
    };
    let a = build_cylinder(&with_gap(0.25), &with_tol(0.25)).unwrap();
    assert_eq!(a.periodic_regions.len(), 1);
    assert_eq!(
        build_cylinder(&with_gap(0.25), &with_tol(0.25 - 1e-9)).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    assert_eq!(
        build_cylinder(&with_gap(1.0), &context()).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    // The lines-only chart with no gap is accepted at the default tolerance.
    assert_eq!(
        build_cylinder(&with_gap(0.0), &context())
            .unwrap()
            .periodic_regions
            .len(),
        1
    );
}

/// The rim winding check tolerates `|Δu − 2π| ≤ 64·ε·(1 + |Δu|)`, about
/// 1.04e-13 here, while the seam spacing check (whose bound also counts the
/// right origin's `|y| = 3`) tolerates about 1.46e-13. A strip 135·2⁻⁵⁰
/// (1.2e-13) too wide passes the spacing check and fails the winding
/// check; one 56·2⁻⁵⁰ (5e-14) too wide passes both.
#[test]
fn rim_winding_must_be_a_whole_turn_within_its_own_roundoff_bound() {
    let ulp = 2.0_f64.powi(-50);
    let wide = strip(TAU + 135.0 * ulp, true, &[], &[]);
    assert_eq!(
        build_cylinder(&wide, &context()).err(),
        Some(ArrangementError::UnsupportedDomain)
    );
    let slightly = strip(TAU + 56.0 * ulp, true, &[], &[]);
    let a = build_cylinder(&slightly, &context()).unwrap();
    assert_eq!(a.periodic_regions.len(), 1);
    assert_eq!(a.periodic_regions[0].boundaries.len(), 2);
}

/// A boundary walk that detours through a half-edge outside the region's
/// boundary set is refused as open, even when the detour returns to its
/// start with consistent vertices.
#[test]
fn boundary_walk_through_a_non_boundary_half_edge_is_open() {
    let uses = strip(TAU, false, &[], &[]);
    let mut a = build(&uses).unwrap();
    let h0 = (0..a.half_edges.len())
        .find(|&h| {
            let e = &a.half_edges[h];
            e.source.use_id == 0 && a.vertices[e.from].uv.x().abs() < 1e-12
        })
        .unwrap();
    let twin = a.half_edges[h0].twin;
    a.half_edges[h0].next = twin;
    a.half_edges[twin].next = h0;
    let ctx = context();
    let mut work = Work::new(&ctx);
    assert_eq!(
        super::periodic::quotient(&mut a, &lookup(&uses), cylinder_domain(), &mut work),
        Err(ArrangementError::OpenRegion)
    );
}

/// The quotient re-derives `χ = V − E + F` from the cells it joins; a cell
/// claiming a hole twice breaks it and is refused even though every winding
/// still sums to zero.
#[test]
fn euler_characteristic_mismatch_is_refused_even_with_balanced_windings() {
    let mut uses = strip(TAU, false, &[], &[]);
    let mut hole = rectangle(50, 1.0, 1.0, 2.0, 2.0);
    lift_to_cylinder(&mut hole);
    uses.extend(hole);
    let sound = build_cylinder(&uses, &context()).unwrap();
    assert_eq!(sound.periodic_regions.len(), 1);
    assert_eq!(sound.periodic_regions[0].euler_characteristic, -1);
    near(sound.periodic_regions[0].area, 12.0 * PI - 2.0, 1e-9);
    let mut a = build(&uses).unwrap();
    let annulus = a.regions.iter().position(|r| !r.holes.is_empty()).unwrap();
    let hole_cycle = a.regions[annulus].holes[0];
    a.regions[annulus].holes.push(hole_cycle);
    let ctx = context();
    let mut work = Work::new(&ctx);
    assert_eq!(
        super::periodic::quotient(&mut a, &lookup(&uses), cylinder_domain(), &mut work),
        Err(ArrangementError::NonManifoldEmbedding)
    );
}
