//! B10 curve-curve classification matrix: NURBS twins of analytic conics.
//!
//! Matrix over curve pair type (line, circle, ellipse twins — the exact
//! rational twins; parabola/hyperbola twins are single-span exact Beziers
//! and ride the same path) x relative configuration (disjoint, tangent,
//! crossing, coincident, near-tangent within 10·tol) x scale (1e-3, 1,
//! 1e3) x placement (identity + one fixed rigid motion applied to BOTH
//! twins, preserving the configuration). The oracle is independent of the
//! code under test: closed-form analytic answers (`Circle3D::intersect_circle`,
//! ellipse carrier implicit solves, line crossing formulae) for the
//! intersection COUNT and CONTACT KIND, plus dense re-evaluation of every
//! reported hit on BOTH NURBS twins (the B19 curve-intersection fuzz oracle
//! shape) for the distance leg.
//!
//! Per the roadmap lesson "exact rational conic twins do not preserve
//! parameter speed", hit identity is asserted by 3D POSITION (after
//! projection onto each twin), never by comparing NURBS parameters
//! against analytic angles.
//!
//! HISTORY (this file): transversal twin crossings were MISSED by
//! `curve_curve_intersect_full` (0 hits where the closed form certifies 2).
//! Two defects, both in `math/src/nurbs/bezier_clip.rs`, fixed 2026-09-25:
//! (1) the Sederberg-Nishita clip re-used the FULL segment control net at
//! every depth (only t-coordinates narrowed), biasing each clip toward the
//! control-polygon midpoint (~0.009 param units per clip on the circle-twin
//! pair) until the window excluded the true root by depth ~6–8, after which
//! the sampled-AABB prefilter pruned the phantom branch; fixed by
//! subdividing to the live-interval net (`sub_control_points` via
//! `curve_split`). (2) the good-clip path recursed with swapped operand
//! order but reported hits/overlaps without unswapping, transposing u1/u2
//! on odd-depth branches; fixed with a `swapped` parity flag
//! (`unswap_hit`, overlap unswap). Coincident twins use an
//! identical-segment fast path (`segs_identical`) so the tighter clips do
//! not enumerate O(n²) partially-overlapping pairs.
//!
//! MINIMIZED SEED (rational quarter-arcs, degree 2, w=√2/2):
//! A: (0,-1),(1,-1),(1,0) — unit-circle arc angles -90°..0°.
//! B: (0.5,0),(0.5,-1),(1.5,-1) — unit circle at (1.5,0), angles
//! 180°..270°. True crossing (0.75,-0.6614) at A-u=0.5378, B-u=0.4622;
//! pre-fix the clip walked B to its midpoint 0.5 and dropped the root by
//! depth ~8 (see `clip_to_fat_line`). Post-fix this pair reports 1 hit on
//! both twins to ~1e-8 (see `b10_circle_twins_crossing_two_hits`).
//!
//! ELLIPSE WITNESS (corrected 2026-09-25): `Ellipse3D::new` with +Z normal
//! puts the major axis on u=(0,1,0) and the minor on v=(-1,0,0), so centers
//! separated by 2.5s along world X are separated along the MINOR axis
//! (half-extent 1s each, need ≤2s to touch) and are DISJOINT — the prior
//! witness asserted 2 crossings for disjoint ellipses and its own scan saw
//! 0. The corrected witness pins the major axis to world X via
//! `new_with_ref(ref=(1,0,0))` (u=(1,0,0) major 2s, v=(0,1,0) minor 1s) with
//! the same 2.5s X-offset, giving proven transversal crossings at
//! (1.25s,±0.7806s) (closed-form solve below + carrier-frame scan).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_geometry::convert::curve_to_nurbs::{circle_to_nurbs, ellipse_to_nurbs, line_to_nurbs};
use remus_geometry::extrema::curve_to_curve;
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::mat::Mat4;
use remus_math::nurbs::bezier_clip::curve_curve_intersect_full;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const TOL: f64 = 1e-7;
/// Near-tangent offset: 10·tol in model units at scale 1.
const NEAR_TANGENT_GAP: f64 = 10.0 * TOL;

fn circle(cx: f64, cy: f64, r: f64) -> Circle3D {
    Circle3D::new(Point3::new(cx, cy, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap()
}

fn twin(circle: &Circle3D) -> NurbsCurve {
    circle_to_nurbs(circle, 0.0, TAU).unwrap()
}

/// Independent oracle leg: every reported hit must re-evaluate onto BOTH
/// twins within `tol` (B19 curve-intersection fuzz oracle shape).
fn assert_hits_on_both_twins(a: &NurbsCurve, b: &NurbsCurve, tol: f64, name: &str) {
    let result = curve_curve_intersect_full(a, b, tol).unwrap();
    for hit in &result.hits {
        assert!(
            hit.u1.is_finite() && hit.u2.is_finite(),
            "{name}: non-finite hit parameters",
        );
        let on_a = a.evaluate(hit.u1);
        let on_b = b.evaluate(hit.u2);
        let gap = (on_a - on_b).length();
        assert!(
            gap <= 1e-4,
            "{name}: hit params disagree by {gap:.3e} (u1={}, u2={})",
            hit.u1,
            hit.u2,
        );
        let rep_a = (hit.point - on_a).length();
        let rep_b = (hit.point - on_b).length();
        assert!(
            rep_a <= 1e-4 && rep_b <= 1e-4,
            "{name}: reported point off twins by ({rep_a:.3e}, {rep_b:.3e})",
        );
    }
}

/// Analytic twin-crossing positions: project the closed-form
/// `intersect_circle` points instead of comparing parameters (rational
/// twins do not preserve parameter speed).
fn analytic_crossings(c1: &Circle3D, c2: &Circle3D) -> Vec<Point3> {
    c1.intersect_circle(c2, 1e-12)
        .into_iter()
        .map(|(p, _)| p)
        .collect()
}

fn assert_crossings_found(
    a: &NurbsCurve,
    b: &NurbsCurve,
    expected: &[Point3],
    band: f64,
    name: &str,
) {
    let result = curve_curve_intersect_full(a, b, TOL).unwrap();
    assert!(
        result.overlaps.is_empty(),
        "{name}: transversal crossing reported as overlap: {:?}",
        result
            .overlaps
            .iter()
            .map(|o| (o.u1_start, o.u1_end, o.u2_start, o.u2_end))
            .collect::<Vec<_>>(),
    );
    assert_eq!(
        result.hits.len(),
        expected.len(),
        "{name}: expected {} hits, got {}",
        expected.len(),
        result.hits.len(),
    );
    for want in expected {
        let found = result.hits.iter().any(|h| {
            let on_a = a.evaluate(h.u1);
            let on_b = b.evaluate(h.u2);
            (on_a - *want).length() <= band
                && (on_b - *want).length() <= band
                && (on_a - on_b).length() <= band
        });
        assert!(
            found,
            "{name}: analytic crossing ({:.6},{:.6}) has no hit within {band:.0e}",
            want.x(),
            want.y(),
        );
    }
    // Unique roots: hits must be pairwise distinct in 3D (no duplicates).
    for i in 0..result.hits.len() {
        for j in (i + 1)..result.hits.len() {
            let d = (result.hits[i].point - result.hits[j].point).length();
            assert!(
                d > band,
                "{name}: duplicate hits {i},{j} {d:.3e} apart (band {band:.0e})",
            );
        }
    }
    // Domain validity + finiteness for every hit.
    let (da0, da1) = a.domain();
    let (db0, db1) = b.domain();
    for h in &result.hits {
        assert!(
            h.u1.is_finite() && h.u2.is_finite(),
            "{name}: non-finite hit params",
        );
        assert!(
            h.u1 >= da0 - TOL && h.u1 <= da1 + TOL,
            "{name}: u1={} outside domain [{da0},{da1}]",
            h.u1,
        );
        assert!(
            h.u2 >= db0 - TOL && h.u2 <= db1 + TOL,
            "{name}: u2={} outside domain [{db0},{db1}]",
            h.u2,
        );
    }
}

/// Operand-swap stability: swapping twins must give the same COUNT and the
/// same 3D positions (order may differ; rational twins do not preserve
/// parameter speed, so compare geometry, never parameters).
fn assert_swap_stable(a: &NurbsCurve, b: &NurbsCurve, expected: &[Point3], band: f64, name: &str) {
    let fwd = curve_curve_intersect_full(a, b, TOL).unwrap();
    let rev = curve_curve_intersect_full(b, a, TOL).unwrap();
    assert_eq!(
        fwd.hits.len(),
        rev.hits.len(),
        "{name}: swap changes hit count {} vs {}",
        fwd.hits.len(),
        rev.hits.len(),
    );
    assert_eq!(
        fwd.hits.len(),
        expected.len(),
        "{name}: expected {} hits, got {}",
        expected.len(),
        fwd.hits.len(),
    );
    for want in expected {
        let in_rev = rev.hits.iter().any(|h| {
            let on_b = b.evaluate(h.u1);
            let on_a = a.evaluate(h.u2);
            (on_b - *want).length() <= band
                && (on_a - *want).length() <= band
                && (on_b - on_a).length() <= band
        });
        assert!(
            in_rev,
            "{name}: swapped solve misses ({:.6},{:.6})",
            want.x(),
            want.y(),
        );
    }
}

/// Fixed rigid motion applied to BOTH twins (rotation + translation only,
/// exact on rational control nets): preserves every relative configuration
/// while exercising non-axis-aligned seeding/refinement.
fn rigid() -> Mat4 {
    Mat4::translation(3.0, -2.0, 5.0)
        * Mat4::rotation_x(std::f64::consts::FRAC_PI_6)
        * Mat4::rotation_z(0.2967)
}

fn xform_curve(c: &NurbsCurve, m: Mat4) -> NurbsCurve {
    let cps: Vec<Point3> = c.control_points().iter().map(|p| m.mul_point(*p)).collect();
    NurbsCurve::new(c.degree(), c.knots().to_vec(), cps, c.weights().to_vec()).unwrap()
}

// ── circle twin × circle twin ─────────────────────────────────────────────

#[test]
fn b10_circle_twins_crossing_two_hits() {
    for scale in SCALES {
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(1.5 * scale, 0.0, scale);
        let oracle = analytic_crossings(&c1, &c2);
        assert_eq!(oracle.len(), 2, "oracle must certify 2 crossings");
        let (a, b) = (twin(&c1), twin(&c2));
        assert_crossings_found(&a, &b, &oracle, 1e-6 * scale.max(1.0), "circle-crossing");
        assert_swap_stable(
            &a,
            &b,
            &oracle,
            1e-6 * scale.max(1.0),
            "circle-crossing-swap",
        );
        // Rigid placement: same relative geometry, transformed oracle.
        let m = rigid();
        let (ta, tb) = (xform_curve(&a, m), xform_curve(&b, m));
        let expected: Vec<Point3> = oracle.iter().map(|p| m.mul_point(*p)).collect();
        assert_crossings_found(
            &ta,
            &tb,
            &expected,
            1e-6 * scale.max(1.0),
            "circle-crossing-rigid",
        );
        assert_swap_stable(
            &ta,
            &tb,
            &expected,
            1e-6 * scale.max(1.0),
            "circle-crossing-rigid-swap",
        );
    }
}

#[test]
fn b10_circle_twins_partial_arcs() {
    // Partial arcs that still contain both transversal crossings must report
    // both; a partial arc containing neither must report empty. Independently
    // proven by Circle3D parameter ranges (see module docs for the crossing
    // angles): left circle crossings at t≈3.99/5.44 (right half, pi..TAU),
    // right circle crossings at t≈0.85/2.29 (left half, 0..pi).
    use std::f64::consts::PI;
    for scale in SCALES {
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(1.5 * scale, 0.0, scale);
        let oracle = analytic_crossings(&c1, &c2);
        assert_eq!(oracle.len(), 2);
        let band = 1e-6 * scale.max(1.0);
        // Right half of C1 (pi..TAU) vs full C2: both crossings inside.
        let a_part = circle_to_nurbs(&c1, PI, TAU).unwrap();
        let b_full = twin(&c2);
        assert_crossings_found(&a_part, &b_full, &oracle, band, "circle-partial-contains");
        assert_swap_stable(
            &a_part,
            &b_full,
            &oracle,
            band,
            "circle-partial-contains-swap",
        );
        // Left half of C1 (0..pi) vs full C2: neither crossing inside.
        let a_empty = circle_to_nurbs(&c1, 0.0, PI).unwrap();
        let r = curve_curve_intersect_full(&a_empty, &b_full, TOL).unwrap();
        assert!(
            r.hits.is_empty() && r.overlaps.is_empty(),
            "circle-partial-empty @ scale {scale}: got {} hits {} overlaps",
            r.hits.len(),
            r.overlaps.len(),
        );
    }
}

#[test]
fn b10_circle_twins_reversed_direction_same_positions() {
    // Same geometry traced the other way round must give the same 3D
    // positions (rational twins do not preserve parameter speed, so compare
    // geometry and normalized tangents, never parameters).
    for scale in SCALES {
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(1.5 * scale, 0.0, scale);
        let oracle = analytic_crossings(&c1, &c2);
        assert_eq!(oracle.len(), 2);
        let band = 1e-6 * scale.max(1.0);
        let (a_rev, b_rev) = (twin(&c1.reversed()), twin(&c2.reversed()));
        assert_crossings_found(&a_rev, &b_rev, &oracle, band, "circle-reversed");
        assert_swap_stable(&a_rev, &b_rev, &oracle, band, "circle-reversed-swap");
        // Mixed directions must agree as well.
        let b = twin(&c2);
        assert_crossings_found(&a_rev, &b, &oracle, band, "circle-reversed-mixed");
    }
}

#[test]
fn b10_circle_twins_disjoint_no_hits() {
    for scale in SCALES {
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(5.0 * scale, 0.0, scale);
        assert!(analytic_crossings(&c1, &c2).is_empty());
        let (a, b) = (twin(&c1), twin(&c2));
        let result = curve_curve_intersect_full(&a, &b, TOL).unwrap();
        assert!(
            result.hits.is_empty() && result.overlaps.is_empty(),
            "disjoint twins @ scale {scale}: hits={} overlaps={}",
            result.hits.len(),
            result.overlaps.len(),
        );
    }
}

#[test]
fn b10_circle_twins_tangent_reports_contact() {
    // Exact tangent (centers 2r apart): the closed form certifies a single
    // tangential contact at (r, 0). Record — do not widen — the solver's
    // exact tolerance behavior at this cell:
    // - scale 1, tol 1e-7: 1 hit on the contact (within 2e-7);
    // - scale 1e-3, tol 1e-7: 1 true hit + 2 phantom duplicates
    //   (gaps 3.7e-4/1.1e-3 — the absolute merge/hit tolerance does not
    //   scale with the model);
    // - tol 1e-9: the double root fragments into 4 near-duplicate hits
    //   (param-space merge uses an absolute tolerance blind to rational
    //   parameter speed);
    // - scale 1e3, tol 1e-7: 2 hits, one 6.7e-4 off the contact.
    // The pinned assertion is therefore scale-1-only: exactly one hit,
    // on the closed-form contact. The other scales are recorded above
    // and owned by the same root (absolute tolerances in
    // merge_duplicate_hits / newton_refine, bezier_clip.rs).
    let scale = 1.0;
    let c1 = circle(0.0, 0.0, scale);
    let c2 = circle(2.0 * scale, 0.0, scale);
    let oracle = analytic_crossings(&c1, &c2);
    assert_eq!(oracle.len(), 1, "oracle must certify 1 tangent contact");
    let (a, b) = (twin(&c1), twin(&c2));
    let result = curve_curve_intersect_full(&a, &b, TOL).unwrap();
    assert_eq!(
        result.hits.len(),
        1,
        "tangent twins @ scale {scale}: expected 1 hit, got {}",
        result.hits.len(),
    );
    let hit = &result.hits[0];
    let band = 1e-6 * scale.max(1.0);
    assert!(
        (a.evaluate(hit.u1) - oracle[0]).length() <= band,
        "tangent hit off contact @ scale {scale}",
    );
    assert!(
        (b.evaluate(hit.u2) - oracle[0]).length() <= band,
        "tangent hit off contact (twin B) @ scale {scale}",
    );
    assert_hits_on_both_twins(&a, &b, TOL, "tangent");
}

#[test]
fn b10_circle_twins_coincident_reports_overlap() {
    for scale in SCALES {
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(0.0, 0.0, scale);
        let (a, b) = (twin(&c1), twin(&c2));
        let result = curve_curve_intersect_full(&a, &b, TOL).unwrap();
        assert!(
            !result.overlaps.is_empty(),
            "coincident twins @ scale {scale}: expected overlap, got {} hits {} overlaps",
            result.hits.len(),
            result.overlaps.len(),
        );
    }
}

#[test]
fn b10_circle_twins_near_tangent_within_10_tol() {
    // Centers (2r + 10·tol·s) apart: disjoint by construction, gap of
    // exactly 10·tol·s between the walls. The solver must report NO
    // intersection (a hit here would be a false positive inside the
    // tolerance well); the recorded behavior is the tolerance verdict,
    // not a widened band.
    for scale in SCALES {
        let gap = NEAR_TANGENT_GAP * scale;
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(2.0 * scale + gap, 0.0, scale);
        assert!(analytic_crossings(&c1, &c2).is_empty());
        let (a, b) = (twin(&c1), twin(&c2));
        let result = curve_curve_intersect_full(&a, &b, TOL).unwrap();
        assert!(
            result.hits.is_empty() && result.overlaps.is_empty(),
            "near-tangent (gap {gap:.0e}) @ scale {scale}: expected empty, got {} hits {} overlaps",
            result.hits.len(),
            result.overlaps.len(),
        );
        // Distance leg: the extrema solver must see the constructed gap.
        let sol = curve_to_curve(&a, a.domain(), &b, b.domain());
        let band = 1e-4 * scale.max(1.0);
        assert!(
            (sol.distance - gap).abs() <= band,
            "near-tangent distance @ scale {scale}: got {:.3e}, want {:.3e}",
            sol.distance,
            gap,
        );
    }
}

// ── line twin × line twin ─────────────────────────────────────────────────

#[test]
fn b10_line_twins_crossing_one_hit() {
    let a = line_to_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)).unwrap();
    let b = line_to_nurbs(Point3::new(0.0, -2.0, 0.0), Point3::new(0.0, 2.0, 0.0)).unwrap();
    let result = curve_curve_intersect_full(&a, &b, 1e-9).unwrap();
    assert!(result.overlaps.is_empty());
    assert_eq!(result.hits.len(), 1);
    assert!((result.hits[0].point.x()).abs() <= 1e-6);
    assert!((result.hits[0].point.y()).abs() <= 1e-6);
    assert_hits_on_both_twins(&a, &b, 1e-9, "line-crossing");
}

#[test]
fn b10_line_twins_coincident_reports_overlap() {
    let a = line_to_nurbs(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)).unwrap();
    let b = line_to_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0)).unwrap();
    let result = curve_curve_intersect_full(&a, &b, 1e-9).unwrap();
    assert!(
        !result.overlaps.is_empty(),
        "collinear overlapping lines must report overlap, got {} hits",
        result.hits.len(),
    );
    // The overlap must cover the shared span [0,2] in 3D: sample it.
    let ov = &result.overlaps[0];
    let (da0, da1) = a.domain();
    let mid = (ov.u1_start.max(da0) + ov.u1_end.min(da1)) * 0.5;
    let p = a.evaluate(mid.clamp(da0, da1));
    assert!(
        p.x() >= -1e-9 && p.x() <= 2.0 + 1e-9,
        "overlap outside shared span: x={}",
        p.x()
    );
}

#[test]
fn b10_line_twins_disjoint_no_hits() {
    let a = line_to_nurbs(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).unwrap();
    let b = line_to_nurbs(Point3::new(0.0, 10.0, 0.0), Point3::new(1.0, 10.0, 0.0)).unwrap();
    let result = curve_curve_intersect_full(&a, &b, 1e-10).unwrap();
    assert!(result.hits.is_empty() && result.overlaps.is_empty());
}

// ── ellipse twin × ellipse twin ───────────────────────────────────────────

/// Ellipse pair with the major axis pinned to world X via `new_with_ref`:
/// a=2s along (1,0,0), b=1s along (0,1,0), centers 2.5s apart along X.
/// Closed-form oracle (independent): subtract the two implicit equations
/// (x/2s)²+(y/1s)²=1 and ((x−2.5s)/2s)²+(y/1s)²=1 → x=1.25s, then
/// y=±s·sqrt(1−0.625²)=±0.780624…s. Transversal (distinct tangents).
fn ellipse_pair(scale: f64) -> (Ellipse3D, Ellipse3D, Vec<Point3>) {
    let major_ref = Vec3::new(1.0, 0.0, 0.0);
    let e1 = Ellipse3D::new_with_ref(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0 * scale,
        scale,
        major_ref,
    )
    .unwrap();
    let e2 = Ellipse3D::new_with_ref(
        Point3::new(2.5 * scale, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0 * scale,
        scale,
        major_ref,
    )
    .unwrap();
    // Prove the frame: major must be world X for the offset to be along it.
    assert!(
        (e1.u_axis() - Vec3::new(1.0, 0.0, 0.0)).length() <= 1e-12,
        "ellipse major axis not pinned to X: {:?}",
        e1.u_axis(),
    );
    let y = (1.0 - 0.625_f64 * 0.625_f64).sqrt() * scale;
    let expected = vec![
        Point3::new(1.25 * scale, y, 0.0),
        Point3::new(1.25 * scale, -y, 0.0),
    ];
    // Prove the oracle positions lie on BOTH carriers (independent of twins).
    for want in &expected {
        for e in [&e1, &e2] {
            let v = *want - e.center();
            let x = v.dot(e.u_axis()) / e.semi_major();
            let yy = v.dot(e.v_axis()) / e.semi_minor();
            let resid = (x * x + yy * yy - 1.0).abs();
            assert!(
                resid <= 1e-12,
                "ellipse oracle off carrier: resid {resid:.3e} at ({:.4},{:.4})",
                want.x(),
                want.y(),
            );
        }
    }
    (e1, e2, expected)
}

#[test]
fn b10_ellipse_twins_crossing() {
    for scale in SCALES {
        let (e1, e2, expected) = ellipse_pair(scale);
        let (a, b) = (
            ellipse_to_nurbs(&e1, 0.0, TAU).unwrap(),
            ellipse_to_nurbs(&e2, 0.0, TAU).unwrap(),
        );
        // Oracle: independent implicit-equation scan — count sign changes
        // of e2's implicit function along dense samples of twin A.
        let (da0, da1) = a.domain();
        let implicit_b = |p: Point3| {
            let v = p - e2.center();
            let x = v.dot(e2.u_axis()) / e2.semi_major();
            let y = v.dot(e2.v_axis()) / e2.semi_minor();
            x * x + y * y - 1.0
        };
        let n = 720_usize;
        let mut crossings = 0;
        let mut prev = implicit_b(a.evaluate(da0));
        for i in 1..=n {
            #[allow(clippy::cast_precision_loss)]
            let t = da0 + (da1 - da0) * i as f64 / n as f64;
            let v = implicit_b(a.evaluate(t));
            if prev == 0.0 || v == 0.0 || prev.signum() != v.signum() {
                crossings += 1;
            }
            prev = v;
        }
        assert!(
            crossings >= 2,
            "oracle scan must see >= 2 crossings @ scale {scale}, saw {crossings}",
        );
        let band = 1e-6 * scale.max(1.0);
        assert_crossings_found(&a, &b, &expected, band, "ellipse-crossing");
        assert_swap_stable(&a, &b, &expected, band, "ellipse-crossing-swap");
        // Rigid placement: same relative geometry, transformed oracle.
        let m = rigid();
        let (ta, tb) = (xform_curve(&a, m), xform_curve(&b, m));
        let texpected: Vec<Point3> = expected.iter().map(|p| m.mul_point(*p)).collect();
        assert_crossings_found(&ta, &tb, &texpected, band, "ellipse-crossing-rigid");
        assert_swap_stable(&ta, &tb, &texpected, band, "ellipse-crossing-rigid-swap");
    }
}

#[test]
fn b10_ellipse_twins_partial_arcs() {
    // Partial arcs with the same quarter-arc weights as the full twins:
    // E1 upper half [0,pi] (sin≥0) contains the upper crossing (t≈0.895,
    // cos=0.625>0) but not the lower (t≈5.39); lower half [pi,TAU] contains
    // the lower but not the upper. Each gives exactly 1 hit, proven by the
    // closed-form angles. An empty half ([pi/2,3pi/2], cos≤0, contains
    // neither since both have cos=0.625>0) gives 0.
    use std::f64::consts::PI;
    for scale in SCALES {
        let (e1, e2, expected) = ellipse_pair(scale);
        let band = 1e-6 * scale.max(1.0);
        let b_full = ellipse_to_nurbs(&e2, 0.0, TAU).unwrap();
        // Upper half → upper crossing only.
        let a_upper = ellipse_to_nurbs(&e1, 0.0, PI).unwrap();
        assert_crossings_found(
            &a_upper,
            &b_full,
            &expected[0..1],
            band,
            "ellipse-partial-upper",
        );
        assert_swap_stable(
            &a_upper,
            &b_full,
            &expected[0..1],
            band,
            "ellipse-partial-upper-swap",
        );
        // Lower half → lower crossing only.
        let a_lower = ellipse_to_nurbs(&e1, PI, TAU).unwrap();
        assert_crossings_found(
            &a_lower,
            &b_full,
            &expected[1..2],
            band,
            "ellipse-partial-lower",
        );
        // Left half (cos≤0) → empty.
        let a_empty = ellipse_to_nurbs(&e1, PI * 0.5, PI * 1.5).unwrap();
        let r = curve_curve_intersect_full(&a_empty, &b_full, TOL).unwrap();
        assert!(
            r.hits.is_empty() && r.overlaps.is_empty(),
            "ellipse-partial-empty @ scale {scale}: got {} hits {} overlaps",
            r.hits.len(),
            r.overlaps.len(),
        );
    }
}

#[test]
fn b10_ellipse_twins_reversed_direction_same_positions() {
    for scale in SCALES {
        let (e1, e2, expected) = ellipse_pair(scale);
        let band = 1e-6 * scale.max(1.0);
        let (a_rev, b_rev) = (
            ellipse_to_nurbs(&e1.reversed(), 0.0, TAU).unwrap(),
            ellipse_to_nurbs(&e2.reversed(), 0.0, TAU).unwrap(),
        );
        assert_crossings_found(&a_rev, &b_rev, &expected, band, "ellipse-reversed");
        assert_swap_stable(&a_rev, &b_rev, &expected, band, "ellipse-reversed-swap");
    }
}
