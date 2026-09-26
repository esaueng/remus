//! Prepared point-classification reuse (PERF-Q01 substrate).
//!
//! [`remus_check::classify::PreparedSolid`] must answer exactly like the
//! one-shot [`remus_check::classify::classify_point`] on every shape family
//! the kernel classifies: boxes, hollow bodies (inner shells), cylinders with
//! seams, trimmed curved patches (sphere caps, torus bands), and NURBS input.
//! Points carry independent analytic/material expectations (Inside/Outside)
//! where the truth is unambiguous, plus exact differential equality to the
//! one-shot path on every probe including the ambiguous boundary band.
//!
//! Sampling is a deterministic Halton sequence, so a failure reproduces
//! exactly.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::classify_points;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

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

fn halton_box(n: u32, lo: Point3, hi: Point3) -> Vec<Point3> {
    (1..=n)
        .map(|i| {
            Point3::new(
                (hi.x() - lo.x()).mul_add(halton(i, 2), lo.x()),
                (hi.y() - lo.y()).mul_add(halton(i, 3), lo.y()),
                (hi.z() - lo.z()).mul_add(halton(i, 5), lo.z()),
            )
        })
        .collect()
}

/// Assert prepared == one-shot on every probe, and prepared == analytic truth
/// wherever the truth is unambiguous (outside `band` of the surface).
fn assert_prepared_matches(
    topo: &Topology,
    solid: SolidId,
    probes: &[Point3],
    truth: impl Fn(Point3) -> Option<bool>,
    label: &str,
) {
    let options = ClassifyOptions::default();
    let prepared = PreparedSolid::prepare(topo, solid).unwrap();
    let mut scored = 0usize;
    for &point in probes {
        let expected_one_shot = classify_point(topo, solid, point, &options).unwrap();
        let got = prepared.classify_point(point, &options).unwrap();
        assert_eq!(got, expected_one_shot, "{label}: probe {point:?}");
        if let Some(inside) = truth(point) {
            scored += 1;
            assert_eq!(
                got == PointClassification::Inside,
                inside,
                "{label}: probe {point:?} disagrees with analytic truth"
            );
        }
    }
    assert!(
        scored > 0,
        "{label}: no probe scored against analytic truth"
    );
    // The batch entry point agrees too.
    let batch = prepared.classify_points(probes, &options).unwrap();
    let scalar: Vec<_> = probes
        .iter()
        .map(|&point| prepared.classify_point(point, &options).unwrap())
        .collect();
    assert_eq!(batch, scalar, "{label}: batch differs from scalar");
}

/// Signed distance to the faces of a box spanning 0..s on every axis.
fn box_truth(point: Point3, s: f64, band: f64) -> Option<bool> {
    let d = [point.x(), point.y(), point.z()]
        .iter()
        .map(|&c| c.min(s - c))
        .fold(f64::INFINITY, f64::min);
    if d.abs() < band { None } else { Some(d > 0.0) }
}

#[test]
fn prepared_matches_box() {
    let s = 10.0;
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, s, s, s).unwrap();
    let mut probes = halton_box(
        300,
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(11.0, 11.0, 11.0),
    );
    // Exact boundary verdicts must agree as well, not just In/Out.
    probes.extend([
        Point3::new(5.0, 5.0, 0.0),
        Point3::new(5.0, 5.0, 10.0),
        Point3::new(0.0, 5.0, 5.0),
        Point3::new(5.0, 5.0, 5.0),
        Point3::new(-1.0, -1.0, -1.0),
    ]);
    assert_prepared_matches(&topo, solid, &probes, |p| box_truth(p, s, 1e-4), "box");
}

#[test]
fn prepared_matches_hollow_body() {
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let inner = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    transform_solid(&mut topo, inner, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, inner).unwrap();
    assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);

    let mut probes = halton_box(
        300,
        Point3::new(-0.5, -0.5, -0.5),
        Point3::new(3.5, 3.5, 3.5),
    );
    probes.extend([
        // Cavity centre: outside the material despite being inside the outer box.
        Point3::new(1.5, 1.5, 1.5),
        Point3::new(0.5, 0.5, 0.5),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(2.0, 2.0, 2.0),
    ]);
    assert_prepared_matches(
        &topo,
        hollow,
        &probes,
        |p| {
            let in_outer = (0.0..3.0).contains(&p.x())
                && (0.0..3.0).contains(&p.y())
                && (0.0..3.0).contains(&p.z());
            let in_inner = (1.0..2.0).contains(&p.x())
                && (1.0..2.0).contains(&p.y())
                && (1.0..2.0).contains(&p.z());
            let near = [
                p.x(),
                p.y(),
                p.z(),
                3.0 - p.x(),
                3.0 - p.y(),
                3.0 - p.z(),
                (p.x() - 1.0).abs(),
                (p.x() - 2.0).abs(),
                (p.y() - 1.0).abs(),
                (p.y() - 2.0).abs(),
                (p.z() - 1.0).abs(),
                (p.z() - 2.0).abs(),
            ]
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b.abs()));
            if near < 1e-4 {
                None
            } else {
                Some(in_outer && !in_inner)
            }
        },
        "hollow",
    );
}

#[test]
fn prepared_matches_cylinder_with_seams() {
    let (r, h) = (5.0, 10.0);
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, r, h).unwrap();
    let mut probes = halton_box(
        300,
        Point3::new(-6.0, -6.0, -1.0),
        Point3::new(6.0, 6.0, 11.0),
    );
    probes.extend([
        Point3::new(0.0, 0.0, 5.0),
        Point3::new(r, 0.0, 5.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, h),
        Point3::new(6.0, 0.0, 5.0),
    ]);
    assert_prepared_matches(
        &topo,
        solid,
        &probes,
        |p| {
            let d = (r - p.x().hypot(p.y())).min(p.z()).min(h - p.z());
            if d.abs() < 1e-4 { None } else { Some(d > 0.0) }
        },
        "cylinder",
    );
}

#[test]
fn prepared_matches_sphere_caps_and_torus_band() {
    let mut topo = Topology::new();
    let sphere = make_sphere(&mut topo, 5.0, 32).unwrap();
    let probes = halton_box(
        300,
        Point3::new(-6.0, -6.0, -6.0),
        Point3::new(6.0, 6.0, 6.0),
    );
    assert_prepared_matches(
        &topo,
        sphere,
        &probes,
        |p| {
            let d = 5.0 - p.x().hypot(p.y()).hypot(p.z());
            if d.abs() < 0.05 { None } else { Some(d > 0.0) }
        },
        "sphere",
    );

    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 3.0, 1.0, 32).unwrap();
    let probes = halton_box(
        300,
        Point3::new(-4.5, -4.5, -1.5),
        Point3::new(4.5, 4.5, 1.5),
    );
    assert_prepared_matches(
        &topo,
        torus,
        &probes,
        |p| {
            let q = p.x().hypot(p.y()) - 3.0;
            let d = 1.0 - q.hypot(p.z());
            if d.abs() < 0.05 { None } else { Some(d > 0.0) }
        },
        "torus",
    );
}

#[test]
fn prepared_matches_nurbs_solids() {
    for (label, build, truth) in [
        (
            "nurbs-box",
            (|t: &mut Topology| make_box(t, 10.0, 10.0, 10.0).unwrap())
                as fn(&mut Topology) -> SolidId,
            (|p: Point3| box_truth(p, 10.0, 0.05)) as fn(Point3) -> Option<bool>,
        ),
        (
            "nurbs-cylinder",
            (|t: &mut Topology| make_cylinder(t, 5.0, 10.0).unwrap())
                as fn(&mut Topology) -> SolidId,
            (|p: Point3| {
                let d = (5.0 - p.x().hypot(p.y())).min(p.z()).min(10.0 - p.z());
                if d.abs() < 0.3 { None } else { Some(d > 0.0) }
            }) as fn(Point3) -> Option<bool>,
        ),
    ] {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        remus_heal::custom::convert_to_bspline::convert_solid_to_bspline(&mut topo, solid).unwrap();
        let probes = halton_box(
            60,
            Point3::new(-6.0, -6.0, -1.0),
            Point3::new(11.0, 11.0, 11.0),
        );
        assert_prepared_matches(&topo, solid, &probes, truth, label);
    }
}

#[test]
fn prepared_matches_across_translations_and_scales() {
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
        transform_solid(&mut topo, solid, &Mat4::scale(scale, scale, scale)).unwrap();
        transform_solid(&mut topo, solid, &Mat4::translation(37.0, -11.0, 5.0)).unwrap();

        let (r, h) = (5.0 * scale, 10.0 * scale);
        let (cx, cy, cz) = (37.0, -11.0, 5.0);
        let pad = scale.max(1e-6);
        let lo = Point3::new(cx - r - pad, cy - r - pad, cz - pad);
        let hi = Point3::new(cx + r + pad, cy + r + pad, cz + h + pad);
        let mut probes = halton_box(200, lo, hi);
        // Near-boundary probes on both sides of the lateral wall.
        for delta in [1e-9, 1e-6, 1e-4] {
            probes.push(Point3::new(cx + r + delta, cy, cz + h / 2.0));
            probes.push(Point3::new(cx + r - delta, cy, cz + h / 2.0));
        }
        probes.extend([
            Point3::new(cx, cy, cz + h / 2.0),
            Point3::new(cx + r, cy, cz + h / 2.0),
            Point3::new(cx, cy, cz),
        ]);
        assert_prepared_matches(
            &topo,
            solid,
            &probes,
            |p| {
                let qx = p.x() - cx;
                let qy = p.y() - cy;
                let qz = p.z() - cz;
                let d = (r - qx.hypot(qy)).min(qz).min(h - qz);
                // The tolerance is absolute while the geometry scales: keep
                // the truth band proportional to the scale.
                let band = (1e-4 * scale).max(1e-9);
                if d.abs() < band { None } else { Some(d > 0.0) }
            },
            &format!("scaled-cylinder-{scale:e}"),
        );
    }

    // A large translation alone: float coordinates far from the origin.
    // Exact-surface probes only assert differential equality (both paths must
    // agree); analytic truth skips a band around the faces, since the
    // classifier's absolute tolerance band is genuinely ambiguous there.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, solid, &Mat4::translation(1.0e6, -5.0e5, 2.5e5)).unwrap();
    let (ox, oy, oz) = (1.0e6, -5.0e5, 2.5e5);
    let probes = [
        Point3::new(ox + 1.0, oy + 1.0, oz + 1.0),
        Point3::new(ox - 1.0, oy - 1.0, oz - 1.0),
        Point3::new(ox + 1.0, oy + 1.0, oz + 2.0),
        Point3::new(ox, oy, oz),
    ];
    assert_prepared_matches(
        &topo,
        solid,
        &probes,
        |p| {
            let dx = (p.x() - ox).min(ox + 2.0 - p.x());
            let dy = (p.y() - oy).min(oy + 2.0 - p.y());
            let dz = (p.z() - oz).min(oz + 2.0 - p.z());
            let d = dx.min(dy).min(dz);
            if d.abs() < 1e-3 { None } else { Some(d > 0.0) }
        },
        "translated-box",
    );
}

/// The acceptance comparison: 1, 100 and 10000 points on one solid, prepared
/// vs one-shot, with identical verdicts throughout.
#[test]
fn prepared_matches_one_shot_at_1_100_and_10000_points() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let options = ClassifyOptions::default();

    for n in [1u32, 100, 10_000] {
        let probes = halton_box(
            n,
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(11.0, 11.0, 11.0),
        );
        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
        let prepared_results = prepared.classify_points(&probes, &options).unwrap();
        let batch_results = classify_points(&topo, solid, &probes, 0.1, options.tolerance).unwrap();
        for (index, point) in probes.iter().enumerate() {
            let expected = classify_point(&topo, solid, *point, &options).unwrap();
            assert_eq!(prepared_results[index], expected, "n={n} probe {point:?}");
            assert_eq!(
                batch_results[index],
                remus_operations::classify::PointClassification::from(expected),
                "n={n} batch probe {point:?}"
            );
        }
        // The corpus must not be degenerate: every verdict appears at 10000.
        if n == 10_000 {
            assert!(prepared_results.contains(&PointClassification::Inside));
            assert!(prepared_results.contains(&PointClassification::Outside));
        }
    }
}
