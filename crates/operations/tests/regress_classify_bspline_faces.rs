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
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_torus};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

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

/// A full torus closes in both directions -- the worst case for the seed grid,
/// whose threshold collapsed to its 0.1 floor there, and for the trim, whose
/// boundary is four coincident seam endpoints.
///
/// Measured 17.31% misclassified until BOTH were fixed; neither alone moved it.
#[test]
fn bspline_torus_classifies_identically_to_the_analytic_torus() {
    let mut topo = Topology::new();
    let solid = make_torus(&mut topo, 3.0, 1.0, 48).unwrap();

    let truth = |p: Point3| {
        let q = p.x().hypot(p.y()) - 3.0;
        let d = 1.0 - q.hypot(p.z());
        if d.abs() < 0.3 { None } else { Some(d > 0.0) }
    };

    let (analytic_wrong, _) = score(
        &topo,
        solid,
        Point3::new(-4.5, -4.5, -1.5),
        Point3::new(4.5, 4.5, 1.5),
        700,
        truth,
    );
    assert_eq!(
        analytic_wrong, 0,
        "control: the ANALYTIC torus must be exact"
    );

    convert_solid_to_bspline(&mut topo, solid).unwrap();
    let (wrong, n) = score(
        &topo,
        solid,
        Point3::new(-4.5, -4.5, -1.5),
        Point3::new(4.5, 4.5, 1.5),
        700,
        truth,
    );
    assert_eq!(
        wrong,
        0,
        "b-spline torus misclassified {wrong}/{n} = {:.2}%",
        100.0 * wrong as f64 / n as f64
    );
}

/// The cylinder took all four fixes: 44.97% -> 25.24% (UV period) -> 13.74%
/// (seed spacing) -> 0.00% (patch extent).
///
/// The last one was not in the classifier at all. `plane_face_to_nurbs` sized a
/// converted planar patch from the face's VERTEX positions, and a cap bounded by
/// one closed circle has `start == end`, so the box collapsed to a point, took
/// the +/-1.0 fallback, and produced a patch spanning [-1.2, 1.2] for a disc of
/// radius 5. Rays crossing the cap outside that square found no surface, because
/// refinement clamps (u, v) to the domain.
#[test]
fn bspline_cylinder_classifies_identically_to_the_analytic_cylinder() {
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
    assert_eq!(
        wrong,
        0,
        "b-spline cylinder misclassified {wrong}/{n} = {:.2}%",
        100.0 * wrong as f64 / n as f64
    );
}

/// The cone needed a fix of its own. `cone_to_nurbs` documents `v_range` as the
/// extent along the cone's GENERATOR (its ruling line) from the apex, and it was
/// handed an extent along the AXIS. Those differ by `cos(half_angle)` -- 5.1%
/// for this cone -- so the lateral patch spanned z[0.924, 12.308] instead of
/// z[0, 12] and stopped 0.92 short of its own base circle.
///
/// 21.14% misclassified originally; 3.66% once the seeding and trim were fixed;
/// 0.00% once the patch covers the face.
#[test]
fn bspline_cone_classifies_identically_to_the_analytic_cone() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 6.0, 2.0, 12.0).unwrap();

    let truth = |p: Point3| {
        if p.z() < 0.0 || p.z() > 12.0 {
            let outside_by = (-p.z()).max(p.z() - 12.0);
            return if outside_by < 0.3 { None } else { Some(false) };
        }
        let r_at = (2.0 - 6.0f64).mul_add(p.z() / 12.0, 6.0);
        let d = (r_at - p.x().hypot(p.y())).min(p.z()).min(12.0 - p.z());
        if d.abs() < 0.3 { None } else { Some(d > 0.0) }
    };

    let (analytic_wrong, _) = score(
        &topo,
        solid,
        Point3::new(-7.0, -7.0, -1.0),
        Point3::new(7.0, 7.0, 13.0),
        700,
        truth,
    );
    assert_eq!(
        analytic_wrong, 0,
        "control: the ANALYTIC cone must be exact"
    );

    convert_solid_to_bspline(&mut topo, solid).unwrap();
    let (wrong, n) = score(
        &topo,
        solid,
        Point3::new(-7.0, -7.0, -1.0),
        Point3::new(7.0, 7.0, 13.0),
        700,
        truth,
    );
    assert_eq!(
        wrong,
        0,
        "b-spline cone misclassified {wrong}/{n} = {:.2}%",
        100.0 * wrong as f64 / n as f64
    );
}

/// The invariant the patch-extent fix restores, stated directly: a converted
/// surface must cover the face it was built for.
///
/// Measured before the fix, on a cylinder of radius 5: the cap patches spanned
/// x[-1.20, 1.20] y[-1.20, 1.20] -- about 5% of the disc they were supposed to
/// carry.
#[test]
fn a_converted_patch_covers_its_own_face_boundary() {
    for (label, build) in [
        (
            "cylinder",
            (|t: &mut Topology| make_cylinder(t, 5.0, 10.0).unwrap())
                as fn(&mut Topology) -> remus_topology::solid::SolidId,
        ),
        ("cone", |t: &mut Topology| {
            make_cone(t, 6.0, 2.0, 12.0).unwrap()
        }),
        ("box", |t: &mut Topology| {
            make_box(t, 10.0, 10.0, 10.0).unwrap()
        }),
    ] {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        convert_solid_to_bspline(&mut topo, solid).unwrap();

        for fid in solid_faces(&topo, solid).unwrap() {
            let face = topo.face(fid).unwrap();
            let FaceSurface::Nurbs(nurbs) = face.surface() else {
                continue;
            };

            // The face's own boundary, sampled along the edge curves rather than
            // read off the vertices -- the distinction this fix is about.
            let mut b_lo = [f64::MAX; 3];
            let mut b_hi = [f64::MIN; 3];
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    let edge = topo.edge(oe.edge()).unwrap();
                    let sp = topo.vertex(edge.start()).unwrap().point();
                    let ep = topo.vertex(edge.end()).unwrap().point();
                    let (t0, t1) = edge.domain_with_endpoints(sp, ep);
                    for i in 0..=24 {
                        let t = (t1 - t0).mul_add(f64::from(i) / 24.0, t0);
                        let p = edge.curve().evaluate_with_endpoints(t, sp, ep);
                        for (a, c) in [p.x(), p.y(), p.z()].iter().enumerate() {
                            b_lo[a] = b_lo[a].min(*c);
                            b_hi[a] = b_hi[a].max(*c);
                        }
                    }
                }
            }

            // What the patch actually reaches over its own domain.
            let (u0, u1) = nurbs.domain_u();
            let (v0, v1) = nurbs.domain_v();
            let mut s_lo = [f64::MAX; 3];
            let mut s_hi = [f64::MIN; 3];
            for iu in 0..=40 {
                for iv in 0..=40 {
                    let p = nurbs.evaluate(
                        (u1 - u0).mul_add(f64::from(iu) / 40.0, u0),
                        (v1 - v0).mul_add(f64::from(iv) / 40.0, v0),
                    );
                    for (a, c) in [p.x(), p.y(), p.z()].iter().enumerate() {
                        s_lo[a] = s_lo[a].min(*c);
                        s_hi[a] = s_hi[a].max(*c);
                    }
                }
            }

            for a in 0..3 {
                let slack = 1e-6 * (b_hi[a] - b_lo[a]).abs().max(1.0);
                assert!(
                    s_lo[a] <= b_lo[a] + slack && s_hi[a] >= b_hi[a] - slack,
                    "{label} face {fid:?}: axis {a} -- the converted patch reaches \
                     [{:.3}, {:.3}] but its own boundary needs [{:.3}, {:.3}]",
                    s_lo[a],
                    s_hi[a],
                    b_lo[a],
                    b_hi[a]
                );
            }
        }
    }
}
