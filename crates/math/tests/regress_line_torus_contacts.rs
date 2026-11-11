//! Line–torus contacts of every multiplicity, against the exact tube distance.
//!
//! The quartic solver once accepted a real root only when its iterate's
//! imaginary part fell under an absolute `1e-7`. A root of multiplicity `m`
//! resolves only to about `ε^(1/m)` relative, so the cutoff decided whether
//! a multiple contact survived. A line along an asymptotic direction of the
//! tube's saddle-shaped inner half meets it in a triple root, where the line
//! crosses the surface: its three iterates were all discarded, and a ray from
//! outside counted one crossing. Tangencies of torus size above a few units
//! went the same way.
//!
//! Each line here is built analytically on the torus, and every claim is
//! checked against an independent double-double reference on the same
//! represented line (`line_torus_reference`): the roots it finds are the
//! roots reported, each inside its window, and the crossings reported near a
//! contact have the parity of the exact tube distance's sign change across
//! it.
//!
//! A line built to be tangent is tangent only to the rounding of its
//! construction. The represented line then misses by a hair (no contact),
//! cuts a shallow chord (two roots) or, when the rounding happens to be
//! zero, is tangent (one unresolved contact). All three are right; which one
//! applies is the reference's call, not the construction's.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

mod line_torus_reference;

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use line_torus_reference::{
    Dd, Rng, Tally, Tangent, cases, frame, inner_direction, judge, reference, tangent,
};
use remus_math::analytic_intersection::{intersect_line_torus, intersect_line_torus_contacts};
use remus_math::intersect::ContactKind;
use remus_math::ray::{RayHit, ray_torus};
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};

/// A seeded torus of size `scale`: radii, axis and centre offset vary.
fn random_torus(rng: &mut Rng, scale: f64) -> ToroidalSurface {
    let small = rng.range(0.3, 1.0) * scale;
    let big = small * rng.range(1.5, 5.0);
    let centre = Point3::new(
        rng.range(-3.0, 3.0) * scale,
        rng.range(-3.0, 3.0) * scale,
        rng.range(-3.0, 3.0) * scale,
    );
    ToroidalSurface::with_axis(centre, big, small, rng.direction()).unwrap()
}

/// The exact tube distance at `t` along `origin + t·dir` (negative inside),
/// evaluated in the torus frame from the origin's offset to the centre.
fn distance_along(torus: &ToroidalSurface, origin: Point3, dir: Vec3, t: f64) -> f64 {
    let q = (origin - torus.center()) + dir * t;
    let (a, b, c) = (
        torus.x_axis().dot(q),
        torus.y_axis().dot(q),
        torus.z_axis().dot(q),
    );
    (a.hypot(b) - torus.major_radius()).hypot(c) - torus.minor_radius()
}

/// What a hit adds to a crossing count.
fn crossings(hit: &RayHit) -> usize {
    usize::from(hit.kind != ContactKind::Tangential)
}

fn in_window(hits: &[RayHit], lo: f64, hi: f64) -> Vec<RayHit> {
    hits.iter()
        .copied()
        .filter(|h| h.t >= lo && h.t <= hi)
        .collect()
}

/// The reviewed ray (round 2): an asymptotic tangent at `v = 2π/3` on the
/// unit-scale torus. An absolute cutoff on the iterative solver's imaginary
/// parts dropped its near-triple root at `t ≈ 0.2`, and the ray, from
/// outside, counted the far crossing alone. The represented line has one
/// simple root there; it is reported, resolved, where the reference puts it.
#[test]
fn reviewed_inflectional_ray_keeps_its_triple_crossing() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    let origin = Point3::new(
        1.320_710_678_118_654_8,
        -0.182_574_185_835_055_39,
        0.473_837_530_938_605_64,
    );
    let dir = Vec3::new(
        -0.353_553_390_593_273_73,
        0.912_870_929_175_276_9,
        -0.204_124_145_231_931_4,
    );
    let hits = ray_torus(origin, dir, &torus);
    let truth = reference(&torus, origin, dir);
    let context = format!("{hits:?} vs {truth:?}");
    assert_eq!(
        judge(&hits, &truth, &mut Tally::default()),
        None,
        "{context}"
    );
    // The distance changes sign across t = 0.2: a real crossing.
    assert!(distance_along(&torus, origin, dir, 0.19) > 0.0, "{context}");
    assert!(distance_along(&torus, origin, dir, 0.21) < 0.0, "{context}");
    let triple: Vec<_> = hits.iter().filter(|h| (h.t - 0.2).abs() <= 1e-5).collect();
    assert_eq!(triple.len(), 1, "{context}");
    assert_ne!(triple[0].kind, ContactKind::Tangential, "{context}");
    assert!(triple[0].spread <= 1e-14, "{context}");
    let far: Vec<_> = hits
        .iter()
        .filter(|h| (h.t - 2.321_320_343_559_642_6).abs() <= 1e-9)
        .collect();
    assert_eq!(far.len(), 1, "{context}");
    assert_eq!(far[0].kind, ContactKind::Transversal, "{context}");
    // Odd parity across the contact, and the ray's count from its outside
    // origin is even.
    let near = in_window(&hits, 0.19, 0.21);
    assert_eq!(
        near.iter().map(crossings).sum::<usize>() % 2,
        1,
        "{context}"
    );
    let outside = distance_along(&torus, origin, dir, 0.0) > 0.0;
    let ahead: usize = hits.iter().filter(|h| h.t > 0.0).map(crossings).sum();
    assert_eq!(ahead.is_multiple_of(2), outside, "{context}");
    // `intersect_line_torus` reports the same contacts.
    let plain = intersect_line_torus(&torus, origin, dir);
    assert_eq!(plain.len(), hits.len());
}

/// A seeded family of asymptotic tangent lines on the inner half of the tube
/// (a triple contact: the line crosses), at three scales, with origins from
/// a tenth to ten thousand torus sizes away and direction lengths over six
/// decades. Each contact must be reported as one crossing (or, if rounding
/// splits it, an odd number), never dropped.
#[test]
fn asymptotic_tangents_cross_with_odd_parity() {
    let mut rng = Rng(0x7A1F_93C4_0D2B_5E68);
    let (mut checked, mut skipped) = (0_usize, 0_usize);
    for scale in [1e-3, 1.0, 1e6] {
        for back in [0.1, 1.0, 10.0, 100.0, 1e3, 1e4] {
            for _ in 0..40 {
                let torus = random_torus(&mut rng, scale);
                let size = torus.major_radius() + torus.minor_radius();
                let u = rng.range(0.0, TAU);
                let v = rng.range(FRAC_PI_2 + 0.02, 3.0 * FRAC_PI_2 - 0.02);
                let sign = if rng.unit() < 0.5 { 1.0 } else { -1.0 };
                let unit = inner_direction(&torus, u, v, 1.0, sign);
                let length = 10f64.powf(rng.range(-3.0, 3.0));
                let (point, ..) = frame(&torus, u, v);
                let origin = point - unit * (back * size);
                let dir = unit * length;
                let hits = ray_torus(origin, dir, &torus);
                // The contact sits at arc length `back · size`.
                let at = back * size / length;
                let w = 1e-2 * size / length;
                let (before, after) = (
                    distance_along(&torus, origin, dir, at - w),
                    distance_along(&torus, origin, dir, at + w),
                );
                // Well above the distance's own rounding at the window ends.
                let floor = 1e2 * f64::EPSILON * (back + 10.0) * size;
                if before.abs() <= floor || after.abs() <= floor {
                    skipped += 1;
                    continue;
                }
                checked += 1;
                let context = format!(
                    "scale {scale:e} back {back:e} u {u} v {v} length {length:e}: \
                     distance {before:e} -> {after:e}, hits {hits:?}"
                );
                assert!(
                    before.signum() != after.signum(),
                    "the construction crosses: {context}"
                );
                let near = in_window(&hits, at - w, at + w);
                assert_eq!(
                    near.iter().map(crossings).sum::<usize>() % 2,
                    1,
                    "{context}"
                );
            }
        }
    }
    assert!(
        skipped * 20 < checked,
        "{skipped} skipped, {checked} checked"
    );
}

/// Seeded tangencies that do not cross: exterior touches on the convex outer
/// half in every tangent direction, interior touches on the inner half in
/// directions that enter the tube, and quadruple touches along the
/// asymptotic directions at the inner equator, where symmetry makes the
/// contact even. Whatever the rounding of the construction made of each
/// (a hairline miss, a shallow chord, a tangency), the hits are the
/// reference's roots, they add no parity, and none is a lone transversal
/// crossing.
#[test]
fn touching_lines_add_no_parity() {
    let mut rng = Rng(0x2C4E_6A8B_1D3F_5072);
    let mut checked = [0_usize; 3];
    // Hairline misses, unresolved touches and shallow chords.
    let mut outcomes = [0_usize; 3];
    for scale in [1e-3, 1.0, 1e6] {
        for back in [0.1, 1.0, 10.0, 100.0, 1e3, 1e4] {
            for k in 0..60 {
                let torus = random_torus(&mut rng, scale);
                let size = torus.major_radius() + torus.minor_radius();
                let u = rng.range(0.0, TAU);
                let sign = if rng.unit() < 0.5 { 1.0 } else { -1.0 };
                let kind = k % 3;
                let (v, unit) = match kind {
                    0 => {
                        let v = rng.range(-FRAC_PI_2 + 0.05, FRAC_PI_2 - 0.05);
                        let (_, e_u, e_v, _) = frame(&torus, u, v);
                        let theta = rng.range(0.0, PI);
                        (v, e_u * theta.cos() + e_v * theta.sin())
                    }
                    1 => {
                        let v = rng.range(FRAC_PI_2 + 0.05, 3.0 * FRAC_PI_2 - 0.05);
                        let ratio = rng.range(0.0, 0.8);
                        (v, inner_direction(&torus, u, v, ratio, sign))
                    }
                    _ => (PI, inner_direction(&torus, u, PI, 1.0, sign)),
                };
                let (point, ..) = frame(&torus, u, v);
                let origin = point - unit * (back * size);
                let hits = ray_torus(origin, unit, &torus);
                let truth = reference(&torus, origin, unit);
                let at = back * size;
                let w = 1e-3 * size;
                let (before, after) = (
                    distance_along(&torus, origin, unit, at - w),
                    distance_along(&torus, origin, unit, at + w),
                );
                let context = format!(
                    "kind {kind} scale {scale:e} back {back:e} u {u} v {v}: \
                     distance {before:e} .. {after:e}, hits {hits:?}, truth {truth:?}"
                );
                assert_eq!(
                    judge(&hits, &truth, &mut Tally::default()),
                    None,
                    "{context}"
                );
                assert!((before > 0.0) == (after > 0.0), "{context}");
                let near = in_window(&hits, at - w, at + w);
                assert_eq!(
                    near.iter().map(crossings).sum::<usize>() % 2,
                    0,
                    "{context}"
                );
                assert!(
                    !(near.len() == 1 && near[0].kind == ContactKind::Transversal),
                    "{context}"
                );
                // As many hits as the reference has roots there, unless it
                // too cannot tell the contact from a tangency.
                let roots = truth
                    .roots
                    .iter()
                    .filter(|r| (r.value() - at).abs() <= w)
                    .count();
                if truth.ambiguous.is_empty() {
                    assert_eq!(near.len(), roots, "{context}");
                }
                outcomes[near.len().min(2)] += 1;
                checked[kind] += 1;
            }
        }
    }
    assert!(checked.iter().all(|&n| n >= 300), "{checked:?}");
    assert!(outcomes[0] >= 100 && outcomes[2] >= 100, "{outcomes:?}");
}

/// Lines moved off an exterior tangency and off an asymptotic tangency along
/// the normal, by `1e-15` to `1e-3` torus sizes, either side. Inward of an
/// exterior touch the line crosses twice; outward it misses; across an
/// asymptotic tangency it crosses once either way. The hits are the
/// reference's roots (so a pair is never merged, however close, and never
/// dropped), and the crossings near the contact have the parity the
/// distance's sign change gives.
#[test]
fn near_tangent_lines_keep_parity_either_side() {
    let mut rng = Rng(0x55AA_1234_9876_F00D);
    let mut resolved_pairs = 0_usize;
    let mut closest = f64::INFINITY;
    for scale in [1e-3, 1.0, 1e6] {
        for _ in 0..40 {
            let torus = random_torus(&mut rng, scale);
            let size = torus.major_radius() + torus.minor_radius();
            let u = rng.range(0.0, TAU);
            let back = 10f64.powf(rng.range(-1.0, 3.0)) * size;
            let outer_v = rng.range(-1.2, 1.2);
            // Clear of the inner equator, where the contact approaches a
            // quadruple one and the line's next crossing comes close.
            let inner_v =
                PI + rng.range(0.3, FRAC_PI_2 - 0.1) * if rng.unit() < 0.5 { 1.0 } else { -1.0 };
            let (_, e_u, e_v, _) = frame(&torus, u, outer_v);
            let theta = rng.range(0.0, PI);
            let lines = [
                (outer_v, e_u * theta.cos() + e_v * theta.sin(), false),
                (inner_v, inner_direction(&torus, u, inner_v, 1.0, 1.0), true),
            ];
            for (v, unit, asymptotic) in lines {
                let (point, _, _, normal) = frame(&torus, u, v);
                for offset in [1e-15, 1e-12, 1e-9, 1e-6, 1e-3] {
                    for side in [-1.0, 1.0] {
                        let shift = normal * (side * offset * size);
                        let origin = point + shift - unit * back;
                        let hits = ray_torus(origin, unit, &torus);
                        let truth = reference(&torus, origin, unit);
                        let w = 0.05 * size;
                        let (before, after) = (
                            distance_along(&torus, origin, unit, back - w),
                            distance_along(&torus, origin, unit, back + w),
                        );
                        let context = format!(
                            "scale {scale:e} asymptotic {asymptotic} offset {offset:e} \
                             side {side} v {v}: distance {before:e} .. {after:e}, \
                             hits {hits:?}, truth {truth:?}"
                        );
                        assert_eq!(
                            judge(&hits, &truth, &mut Tally::default()),
                            None,
                            "{context}"
                        );
                        let near = in_window(&hits, back - w, back + w);
                        let count: usize = near.iter().map(crossings).sum();
                        let expected = usize::from(before.signum() != after.signum());
                        assert_eq!(count % 2, expected, "{context}");
                        if asymptotic {
                            if offset <= 1e-9 {
                                assert_eq!(expected, 1, "{context}");
                            }
                        } else if side < 0.0 && (1e-9..1e-5).contains(&offset) {
                            // Inward by more than the construction's
                            // rounding: both crossings, each on its own.
                            assert_eq!(count, 2, "{context}");
                            assert_eq!(near.len(), 2, "{context}");
                            resolved_pairs += 1;
                            closest = closest.min((near[1].t - near[0].t) / size);
                        }
                    }
                }
            }
        }
    }
    assert!(resolved_pairs >= 200, "{resolved_pairs}");
    // Pairs down to a few 1e-5 torus sizes apart were told apart.
    assert!(closest < 1e-3, "{closest}");
}

/// Exact tangencies, on tori whose numbers are dyadic so the tangency is
/// exact in `f64`: the only contacts that stay unresolved. Each is one
/// window with equal end signs, of the width its multiplicity allows,
/// holding the contact; the reference cannot resolve them either.
#[test]
fn exact_tangencies_are_the_unresolved_contacts() {
    for scale in [1.0 / 1024.0, 1.0, 1_048_576.0] {
        let (big, small) = (1.5 * scale, 0.5 * scale);
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), big, small).unwrap();
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        // (origin, direction, contact parameters, widest window in sizes).
        let lines: [(Point3, Vec3, Vec<f64>, f64); 5] = [
            // Outer equator along its circle, and across it (vertical).
            (
                Point3::new(big + small, -3.0 * scale, 0.0),
                y,
                vec![3.0 * scale],
                1e-13,
            ),
            (Point3::new(big + small, 0.0, -scale), z, vec![scale], 1e-13),
            // Inner equator, vertical: a touch from the hole's side.
            (Point3::new(big - small, 0.0, -scale), z, vec![scale], 1e-13),
            // Tube top across its circle: two touches.
            (
                Point3::new(-4.0 * scale, 0.0, small),
                x,
                vec![2.5 * scale, 5.5 * scale],
                1e-13,
            ),
            // Tube top along its circle: a quadruple contact.
            (
                Point3::new(big, -2.0 * scale, small),
                y,
                vec![2.0 * scale],
                1e-6,
            ),
        ];
        for (origin, dir, at, widest) in lines {
            for length in [1.0, 0.25, 64.0] {
                let dir = dir * length;
                let contacts = intersect_line_torus_contacts(&torus, origin, dir);
                let truth = reference(&torus, origin, dir);
                let context = format!("{origin:?} {dir:?}: {contacts:?} vs {truth:?}");
                assert!(truth.roots.is_empty(), "{context}");
                assert_eq!(truth.ambiguous.len(), at.len(), "{context}");
                assert_eq!(contacts.len(), at.len(), "{context}");
                for (contact, at) in contacts.iter().zip(&at) {
                    let at = at / length;
                    assert!(!contact.resolved, "{context}");
                    assert_eq!(contact.kind, ContactKind::Tangential, "{context}");
                    assert!(contact.t_lo <= at && at <= contact.t_hi, "{context}");
                    assert!(
                        contact.t_lo <= contact.t && contact.t <= contact.t_hi,
                        "{context}"
                    );
                    assert!(
                        contact.t_hi - contact.t_lo <= widest * scale / length,
                        "{context}"
                    );
                }
                let hits = ray_torus(origin, dir, &torus);
                assert_eq!(
                    judge(&hits, &truth, &mut Tally::default()),
                    None,
                    "{context}"
                );
            }
        }
    }
}

/// The windows of [`intersect_line_torus_contacts`] over every family of the
/// differential harness: a resolved contact's window holds exactly one
/// reference root (compared in double-double, with no allowance) and is at
/// most 128 units in the last place wide either side; every reference root
/// is in one; an unresolved window sits on a contact the reference cannot
/// resolve either.
#[test]
fn every_window_holds_its_roots() {
    let mut widest: f64 = 0.0;
    let (mut resolved, mut unresolved) = (0_usize, 0_usize);
    // Windows by half-width in ulps: up to 4, 16, 64 and 130.
    let mut widths = [0_usize; 4];
    for case in cases() {
        let contacts = intersect_line_torus_contacts(&case.torus, case.origin, case.dir);
        let truth = reference(&case.torus, case.origin, case.dir);
        let context = format!(
            "{}: {:?} {:?} {:?}\n{contacts:?}\n{truth:?}",
            case.family, case.torus, case.origin, case.dir
        );
        let inside = |root: Dd, lo: f64, hi: f64| {
            (root - Dd::of(lo)).sign() >= 0 && (Dd::of(hi) - root).sign() >= 0
        };
        let mut owned = 0;
        for pair in contacts.windows(2) {
            assert!(pair[0].t <= pair[1].t, "{context}");
        }
        for contact in &contacts {
            assert!(
                contact.t_lo <= contact.t && contact.t <= contact.t_hi,
                "{context}"
            );
            let held = truth
                .roots
                .iter()
                .filter(|&&r| inside(r, contact.t_lo, contact.t_hi))
                .count();
            if contact.resolved {
                assert_eq!(held, 1, "{context}");
                assert_ne!(contact.kind, ContactKind::Tangential, "{context}");
                owned += held;
                let scale = contact.t.abs().max(truth.size_t);
                let half = (contact.t - contact.t_lo).max(contact.t_hi - contact.t);
                assert!(half <= 130.0 * f64::EPSILON * scale, "{context}");
                let ulps = half / (f64::EPSILON * scale);
                widest = widest.max(ulps);
                widths[usize::from(ulps > 4.0)
                    + usize::from(ulps > 16.0)
                    + usize::from(ulps > 64.0)] += 1;
                resolved += 1;
            } else {
                assert_eq!(held, 0, "{context}");
                assert!(
                    truth.ambiguous.iter().any(|&(at, odd)| {
                        (at - contact.t).abs() <= 1e-5 * truth.size_t
                            && odd == (contact.kind == ContactKind::Unclassified)
                    }),
                    "{context}"
                );
                unresolved += 1;
            }
        }
        assert_eq!(owned, truth.roots.len(), "{context}");
    }
    assert!(resolved >= 10_000, "{resolved}");
    assert!(unresolved >= 1, "{unresolved}");
    // Three windows in four are within 16 ulps (plain arithmetic certifies
    // a crossing no closer than a few ulps of the torus's size over the
    // crossing angle's sine); all are within the bound.
    assert!((widths[0] + widths[1]) * 4 >= resolved * 3, "{widths:?}");
    assert!(widest <= 130.0, "{widest}");
}

/// A tangent of each kind from the shared families, moved inward by an
/// offset that the represented line resolves: the two roots (or the three
/// about an asymptotic contact) come back in separate windows, each a few
/// ulps wide, however shallow the chord. Round 3's merged representative
/// sat between such a pair.
#[test]
fn shallow_chords_are_resolved_pairs() {
    let mut rng = Rng(0x0BAD_C0DE_5EED_0003);
    let mut pairs = 0_usize;
    let mut closest = f64::INFINITY;
    for scale in [1e-3, 1.0, 1e6] {
        for aspect in [1.5, 20.0, 3000.0] {
            let small = scale / (1.0 + aspect);
            let torus = ToroidalSurface::with_axis(
                Point3::new(scale, 0.0, 0.0),
                small * aspect,
                small,
                rng.direction(),
            )
            .unwrap();
            let size = torus.major_radius() + torus.minor_radius();
            for kind in [Tangent::Exterior, Tangent::Quadruple] {
                for exponent in 4..=12 {
                    let (p, unit, n) = tangent(&mut rng, &torus, kind);
                    // Down to four decades above the construction's
                    // rounding.
                    let depth = 10f64.powi(-exponent) * size;
                    let origin = p - n * depth - unit * (0.7 * size);
                    let contacts = intersect_line_torus_contacts(&torus, origin, unit);
                    let truth = reference(&torus, origin, unit);
                    let near: Vec<_> = contacts
                        .iter()
                        .filter(|c| (c.t - 0.7 * size).abs() <= 0.2 * size)
                        .collect();
                    let roots: Vec<_> = truth
                        .roots
                        .iter()
                        .filter(|r| (r.value() - 0.7 * size).abs() <= 0.2 * size)
                        .collect();
                    let context = format!("{kind:?} depth {depth:e}: {contacts:?} vs {truth:?}");
                    assert_eq!(roots.len(), 2, "{context}");
                    assert_eq!(near.len(), 2, "{context}");
                    for (contact, root) in near.iter().zip(&roots) {
                        assert!(contact.resolved, "{context}");
                        assert!(
                            (**root - Dd::of(contact.t_lo)).sign() >= 0
                                && (Dd::of(contact.t_hi) - **root).sign() >= 0,
                            "{context}"
                        );
                    }
                    pairs += 1;
                    closest = closest.min((near[1].t - near[0].t) / size);
                }
            }
        }
    }
    assert!(pairs >= 150, "{pairs}");
    assert!(closest < 1e-5, "{closest}");
}
