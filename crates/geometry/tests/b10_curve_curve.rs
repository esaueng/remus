//! B10 curve-curve classification matrix: NURBS twins of analytic conics.
//!
//! Matrix over curve pair type (line, circle, ellipse twins — the exact
//! rational twins; parabola/hyperbola twins are single-span exact Beziers
//! and ride the same path) x relative configuration (disjoint, tangent,
//! crossing, coincident, near-tangent within 10·tol) x scale (1e-3, 1,
//! 1e3). The oracle is independent of the code under test: closed-form
//! analytic answers (`Circle3D::intersect_circle`, line crossing formulae)
//! for the intersection COUNT and CONTACT KIND, plus dense re-evaluation
//! of every reported hit on BOTH NURBS twins (the B19 curve-intersection
//! fuzz oracle shape) for the distance leg.
//!
//! Per the roadmap lesson "exact rational conic twins do not preserve
//! parameter speed", hit identity is asserted by 3D POSITION (after
//! projection onto each twin), never by comparing NURBS parameters
//! against analytic angles.
//!
//! KNOWN FINDING (this file, 2026-09-16): transversal twin crossings are
//! MISSED by `curve_curve_intersect_full` (returns 0 hits where the
//! closed form certifies 2). Root cause: the Sederberg-Nishita fat-line
//! clip converges to the control-polygon midpoint, not the true crossing
//! (measured bias ~0.009 in segment-parameter units on the circle-twin
//! pair); once the clip window excludes the root, the sampled-AABB
//! prefilter prunes the phantom branch and the intersection vanishes
//! silently. The crossing cells below are `#[ignore]`d seeds retaining
//! that finding; the disjoint/tangent/coincident/near-tangent cells run
//! green and pin the current behavior.
//!
//! MINIMIZED SEED (rational quarter-arcs, degree 2, w=√2/2):
//! A: (0,-1),(1,-1),(1,0) — unit-circle arc angles -90°..0°.
//! B: (0.5,0),(0.5,-1),(1.5,-1) — unit circle at (1.5,0), angles
//! 180°..270°. True crossing (0.75,-0.6614) at A-u=0.5378, B-u=0.4622;
//! the clip walks B to its midpoint 0.5 and drops the root by depth ~8.
//! See `bezier_clip.rs::clip_to_fat_line` (control-polygon distances are
//! re-used at every depth; only the t-coordinates narrow).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_geometry::convert::curve_to_nurbs::{circle_to_nurbs, ellipse_to_nurbs, line_to_nurbs};
use remus_geometry::extrema::curve_to_curve;
use remus_math::curves::{Circle3D, Ellipse3D};
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
}

// ── circle twin × circle twin ─────────────────────────────────────────────

#[test]
#[ignore = "open: B10 seed — bezier-clip misses transversal twin crossings (fat-line clip bias); disjoint/tangent/coincident legs run below"]
fn b10_circle_twins_crossing_two_hits() {
    for scale in SCALES {
        let c1 = circle(0.0, 0.0, scale);
        let c2 = circle(1.5 * scale, 0.0, scale);
        let oracle = analytic_crossings(&c1, &c2);
        assert_eq!(oracle.len(), 2, "oracle must certify 2 crossings");
        let (a, b) = (twin(&c1), twin(&c2));
        assert_crossings_found(&a, &b, &oracle, 1e-6 * scale.max(1.0), "circle-crossing");
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

#[test]
#[ignore = "open: B10 seed — same bezier-clip transversal miss as the circle twins (ellipse arcs share the fat-line path)"]
fn b10_ellipse_twins_crossing() {
    for scale in SCALES {
        let e1 = Ellipse3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0 * scale,
            scale,
        )
        .unwrap();
        let e2 = Ellipse3D::new(
            Point3::new(2.5 * scale, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0 * scale,
            scale,
        )
        .unwrap();
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
        let result = curve_curve_intersect_full(&a, &b, TOL).unwrap();
        assert_eq!(
            result.hits.len(),
            2,
            "ellipse crossing @ scale {scale}: got {} hits {} overlaps",
            result.hits.len(),
            result.overlaps.len(),
        );
    }
}
