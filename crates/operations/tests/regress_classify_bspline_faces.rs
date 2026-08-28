//! Point classification must survive conversion to b-spline.
//!
//! `convert_solid_to_bspline` changes representation, not geometry: the solid
//! it returns occupies exactly the same space. Point-in-solid classification
//! disagreed anyway, and badly — a plain box misclassified a quarter of its
//! own interior.
//!
//! Two independent mechanisms, both in the NURBS path that
//! `remus_check::classify` takes:
//!
//! 1. `refine_line_surface_point` built its Gauss-Newton normal matrix from the
//!    raw surface tangents rather than their ray-perpendicular components, so
//!    every step was under-relaxed and the iteration budget expired with the
//!    intersection undiscovered — reported as no intersection at all. Fixed in
//!    `remus-math`.
//! 2. `build_uv_boundary` unwrapped u by 2*PI unconditionally, on the stated
//!    assumption that u is angular "for all analytic surfaces". It is also
//!    called for NURBS, whose u is a knot parameter: a converted box face spans
//!    12.0, so consecutive boundary vertices picked up a spurious 2*PI shift.
//!    And where a NURBS direction genuinely does close, the period is its knot
//!    span, not 2*PI — without that the seam projects onto the wrong branch and
//!    the trim polygon collapses, rejecting every hit on the face.
//!
//! Ground truth is analytic per shape. Points within a band of the true surface
//! are skipped rather than scored, because the correct answer there is
//! genuinely ambiguous; the band is stated per test. Sampling is a
//! deterministic Halton sequence, so a failure reproduces exactly.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_heal::custom::convert_to_bspline::convert_solid_to_bspline;
use remus_math::vec::Point3;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;

/// Deterministic low-discrepancy sample in [0, 1).
fn halton(mut i: u32, base: u32) -> f64 {
    let (mut f, mut r) = (1.0_f64, 0.0_f64);
    while i > 0 {
        f /= f64::from(base);
        r += f * f64::from(i % base);
        i /= base;
    }
    r
}

/// Score `count` Halton points against an analytic predicate.
///
/// The predicate returns `None` inside the ambiguous band, and those points are
/// not counted either way. Returns (misclassified, scored).
fn score(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
    lo: Point3,
    hi: Point3,
    count: u32,
    truth: impl Fn(Point3) -> Option<bool>,
) -> (usize, usize) {
    let opts = ClassifyOptions::default();
    let (mut wrong, mut scored) = (0usize, 0usize);
    for i in 1..=count {
        let p = Point3::new(
            (hi.x() - lo.x()).mul_add(halton(i, 2), lo.x()),
            (hi.y() - lo.y()).mul_add(halton(i, 3), lo.y()),
            (hi.z() - lo.z()).mul_add(halton(i, 5), lo.z()),
        );
        let Some(expected) = truth(p) else { continue };
        scored += 1;
        let got = classify_point(topo, solid, p, &opts).unwrap() == PointClassification::Inside;
        if got != expected {
            wrong += 1;
        }
    }
    (wrong, scored)
}

/// Signed distance to the faces of a box spanning 0..s on every axis.
fn box_truth(p: Point3, s: f64, band: f64) -> Option<bool> {
    let d = [p.x(), p.y(), p.z()]
        .iter()
        .map(|&c| c.min(s - c))
        .fold(f64::INFINITY, f64::min);
    if d.abs() < band { None } else { Some(d > 0.0) }
}

/// The headline case: a b-spline box must classify exactly like a box.
///
/// Measured 1193/4743 = 25.15% misclassified before either fix, and 6.05% with
/// only the `remus-math` refinement fix. Every failure was Inside reported as
/// Outside — a missed exit crossing flips the ray parity.
#[test]
fn bspline_box_classifies_identically_to_the_analytic_box() {
    let s = 10.0;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, s, s, s).unwrap();

    // Control: the analytic box is correct, so any failure below is the
    // conversion's doing and not the test's predicate.
    let (analytic_wrong, analytic_n) = score(
        &topo,
        solid,
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(11.0, 11.0, 11.0),
        700,
        |p| box_truth(p, s, 0.05),
    );
    assert!(
        analytic_n > 400,
        "test setup: only {analytic_n} points scored"
    );
    assert_eq!(
        analytic_wrong, 0,
        "control: the ANALYTIC box already misclassifies {analytic_wrong}/{analytic_n}"
    );

    let converted = convert_solid_to_bspline(&mut topo, solid).unwrap();
    assert!(converted > 0, "test setup: nothing was converted");

    let (wrong, n) = score(
        &topo,
        solid,
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(11.0, 11.0, 11.0),
        700,
        |p| box_truth(p, s, 0.05),
    );
    assert_eq!(
        wrong,
        0,
        "b-spline box misclassified {wrong}/{n} = {:.2}% of points it shares with the analytic box",
        100.0 * wrong as f64 / n as f64
    );
}

/// A curved b-spline face is still not right, and this pins how far it got.
///
/// The cylinder measured 44.97% misclassified before the UV-period fix and
/// 25.24% after. The remainder is a THIRD defect, in `intersect_line_nurbs`'s
/// seed grid: it accepts a seed within `|corner_00 - corner_11| / n`, a
/// corner-to-corner diagonal unrelated to how far apart the samples actually
/// are. On this cylinder that threshold is 0.500 against a real sample spacing
/// of 1.567, so roughly 30% of hits are never seeded at all. The box escapes it
/// only because its diagonal happens to exceed its spacing (0.849 vs 0.632).
///
/// This bound is a regression guard on a known-bad number, not a statement that
/// 30% is acceptable. Tighten it to 0 when the seeding defect is fixed.
#[test]
fn bspline_cylinder_is_no_worse_than_the_pinned_rate() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();

    let truth = |p: Point3| {
        let d = (5.0 - p.x().hypot(p.y())).min(p.z()).min(10.0 - p.z());
        if d.abs() < 0.3 { None } else { Some(d > 0.0) }
    };

    let (analytic_wrong, _) = score(
        &topo,
        solid,
        Point3::new(-6.0, -6.0, -1.0),
        Point3::new(6.0, 6.0, 11.0),
        700,
        truth,
    );
    assert_eq!(
        analytic_wrong, 0,
        "control: the ANALYTIC cylinder must be exact"
    );

    convert_solid_to_bspline(&mut topo, solid).unwrap();
    let (wrong, n) = score(
        &topo,
        solid,
        Point3::new(-6.0, -6.0, -1.0),
        Point3::new(6.0, 6.0, 11.0),
        700,
        truth,
    );
    let rate = 100.0 * wrong as f64 / n as f64;
    assert!(
        rate < 30.0,
        "b-spline cylinder regressed past its pinned rate: {wrong}/{n} = {rate:.2}% (was 44.97% \
         before the UV-period fix, 25.24% after)"
    );
}
