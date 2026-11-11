//! The line–torus contract in the caller's parameter (review round 4).
//!
//! The solver isolates roots in its own centre-relative chart, where neither
//! the torus's distance from the world origin nor the ray origin's distance
//! from the torus costs anything. Its answer, though, is stated in the
//! caller's `t`, whose `f64` values are a grid of their own. Two defects
//! came from mixing the two:
//!
//! 1. the isolation stopped at a spacing proportional to `|t|`, so four
//!    clean crossings eight units in the last place of `t` apart came back
//!    as two unresolved tangencies;
//! 2. chart enclosures were rounded outward to `t` and still called
//!    resolved, although two roots less than one unit in the last place of
//!    `t` apart then shared both windows, each with equal signs at its ends.
//!
//! The windows here are exact statements about `f64` values of `t`: every
//! expected end, representative and count below is a closed form of the
//! literal inputs.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::analytic_intersection::{
    LineTorusContact, intersect_line_torus, intersect_line_torus_contacts,
};
use remus_math::intersect::ContactKind;
use remus_math::ray::ray_torus;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};

/// Windows in ascending order, each ending strictly below the next one's
/// start, each holding its representative.
fn assert_disjoint(contacts: &[LineTorusContact]) {
    for contact in contacts {
        assert!(
            contact.t_lo < contact.t_hi && contact.t_lo <= contact.t && contact.t <= contact.t_hi,
            "{contact:?}"
        );
    }
    for pair in contacts.windows(2) {
        assert!(pair[0].t_hi < pair[1].t_lo, "{pair:?}");
    }
}

/// Review round 4, finding 1. A unit torus `1e15` along the line: the roots
/// are `1e15 + {−2, −1, 1, 2}`, each an `f64`, eight units in the last
/// place apart (the spacing there is `0.125`), and the quartic is `−2.1875`
/// half way between the members of each pair. The solve returned two
/// unresolved tangential contacts at `1e15 ± 1`.
#[test]
fn far_centre_crossings_eight_ulps_apart_are_four_resolved_roots() {
    let centre = 1e15;
    let torus = ToroidalSurface::new(Point3::new(centre, 0.0, 0.0), 1.5, 0.5).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let roots = [centre - 2.0, centre - 1.0, centre + 1.0, centre + 2.0];
    let spacing = 0.125;

    let contacts = intersect_line_torus_contacts(&torus, origin, dir);
    assert_eq!(contacts.len(), 4, "{contacts:?}");
    assert_disjoint(&contacts);
    for (contact, root) in contacts.iter().zip(roots) {
        assert!(contact.resolved, "{contact:?}");
        assert_eq!(contact.kind, ContactKind::Transversal, "{contact:?}");
        assert_eq!(contact.roots, 1, "{contact:?}");
        // The root is an `f64`: it is the representative, and the window
        // is the two `f64` values either side of it.
        assert_eq!(contact.t.to_bits(), root.to_bits(), "{contact:?}");
        assert_eq!(contact.t_lo.to_bits(), (root - spacing).to_bits());
        assert_eq!(contact.t_hi.to_bits(), (root + spacing).to_bits());
    }
    // The bare list holds all four intersections.
    let plain = intersect_line_torus(&torus, origin, dir);
    assert_eq!(plain, roots.to_vec());
    let hits = ray_torus(origin, dir, &torus);
    assert_eq!(hits.len(), 4, "{hits:?}");
    for (hit, root) in hits.iter().zip(roots) {
        assert_eq!(hit.kind, ContactKind::Transversal, "{hit:?}");
        assert_eq!(hit.t.to_bits(), root.to_bits(), "{hit:?}");
    }
}

/// Review round 5. Four roots a rounding step from the midpoints between
/// doubles of `t`: with `C = 1e15` (spacing `0.125`) they are
/// `C ± (0.125 − 2⁻⁵⁸)` and `C ± (0.0625 + 2⁻⁵⁸)`. No two `f64` values lie
/// strictly between neighbours, so they are one even contact holding four
/// roots. The bare list cannot place each root on its nearest double (the
/// `2⁻⁵⁸` that decides it is far below what `t` holds there); what it does
/// promise is a value within one spacing of every root, and no value
/// farther than that from all of them.
#[test]
fn fused_roots_at_rounding_midpoints_are_listed_within_one_spacing() {
    let centre = 1e15;
    let spacing = 0.125;
    let minor = 0.031_25_f64.next_down();
    let torus = ToroidalSurface::new(Point3::new(centre, 0.0, 0.0), 0.093_75, minor).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let dir = Vec3::new(1.0, 0.0, 0.0);
    // Offsets from `C`, to far more than the spacing needs.
    let offsets = [-0.125, -0.0625, 0.0625, 0.125];

    let contacts = intersect_line_torus_contacts(&torus, origin, dir);
    assert_eq!(contacts.len(), 1, "{contacts:?}");
    let contact = contacts[0];
    assert!(!contact.resolved, "{contact:?}");
    assert_eq!(contact.kind, ContactKind::Tangential, "{contact:?}");
    assert_eq!(contact.roots, 4, "{contact:?}");
    assert_eq!(contact.t_lo.to_bits(), (centre - 0.25).to_bits());
    assert_eq!(contact.t_hi.to_bits(), (centre + 0.25).to_bits());

    let plain = intersect_line_torus(&torus, origin, dir);
    assert!(plain.windows(2).all(|pair| pair[0] < pair[1]), "{plain:?}");
    for &value in &plain {
        assert!(contact.t_lo <= value && value <= contact.t_hi, "{plain:?}");
        assert!(
            offsets
                .iter()
                .any(|offset| ((value - centre) - offset).abs() <= spacing),
            "{value} is a spacing from no root: {plain:?}"
        );
    }
    for offset in offsets {
        assert!(
            plain
                .iter()
                .any(|value| ((value - centre) - offset).abs() <= spacing),
            "no value within a spacing of C{offset:+}: {plain:?}"
        );
    }
}

/// Review round 4, finding 2. A thin torus `1500` along the line, crossed a
/// hair inside its outer equator: the two roots are
/// `1500 ± 8.387 888 541 3e-14`, and the `f64` values about `1500` are
/// `2.27e-13` apart. Exactly one `f64` (`1500` itself) lies between the
/// roots, so no two disjoint windows hold one each. The solve returned two
/// resolved contacts, both at `1500`, whose windows each held both roots
/// with a positive quartic at every end.
///
/// The roots are one contact: unresolved, even (the quartic is positive at
/// both ends: two crossings that cancel), known to hold two roots, in the
/// narrowest window the parameter has for them.
#[test]
fn roots_the_parameter_cannot_separate_are_one_even_contact() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 0.0015, 0.000_000_15).unwrap();
    let origin = Point3::new(0.001_500_15, 0.0, -1500.0);
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let half_chord = 8.387_888_541_3e-14;
    let (below, above) = (1500.0_f64.next_down(), 1500.0_f64.next_up());
    // The premise: both roots lie strictly inside the `f64` cells either
    // side of 1500.
    assert!(1500.0 - below > half_chord && above - 1500.0 > half_chord);

    let contacts = intersect_line_torus_contacts(&torus, origin, dir);
    assert_eq!(contacts.len(), 1, "{contacts:?}");
    let contact = contacts[0];
    assert!(!contact.resolved, "{contact:?}");
    assert_eq!(contact.kind, ContactKind::Tangential, "{contact:?}");
    assert_eq!(contact.roots, 2, "{contact:?}");
    assert_eq!(contact.t.to_bits(), 1500.0_f64.to_bits(), "{contact:?}");
    assert_eq!(contact.t_lo.to_bits(), below.to_bits(), "{contact:?}");
    assert_eq!(contact.t_hi.to_bits(), above.to_bits(), "{contact:?}");

    // Both roots round to the one `f64` the bare list can name them by.
    assert_eq!(intersect_line_torus(&torus, origin, dir), vec![1500.0]);
    // A ray reads the pair as an even contact in a window, never as one
    // exact crossing.
    let hits = ray_torus(origin, dir, &torus);
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].kind, ContactKind::Tangential, "{hits:?}");
    assert!(
        hits[0].t - hits[0].spread <= below && hits[0].t + hits[0].spread >= above,
        "{hits:?}"
    );

    // From an origin on the tube's level the same two crossings are
    // `±8.39e-14`, the parameter resolves them easily, and they are two
    // transversal roots: the limit is the parameter's, not the solver's.
    let near = intersect_line_torus_contacts(&torus, Point3::new(0.001_500_15, 0.0, 0.0), dir);
    assert_eq!(near.len(), 2, "{near:?}");
    assert_disjoint(&near);
    for (contact, root) in near.iter().zip([-half_chord, half_chord]) {
        assert!(contact.resolved && contact.roots == 1, "{contact:?}");
        assert!(
            contact.t_lo < root && root < contact.t_hi,
            "{contact:?} vs {root}"
        );
        assert!(contact.t_hi - contact.t_lo <= 1e-18, "{contact:?}");
    }
}

/// One expected contact: resolved, roots, window ends relative to the
/// centre.
type Want = (bool, u8, f64, f64);

/// A torus scaled about a centre `1e15` along the line, where `f64` values
/// of `t` are `0.125` apart. With radii `1.5 s` and `0.5 s` the roots are
/// `1e15 + s·{−2, −1, 1, 2}`: exact for every `s` below, so each expected
/// window, count and bare root is a closed form. The same four crossings go
/// from four resolved roots to pairs the parameter cannot separate to one
/// contact holding all four as `s` shrinks through the spacing.
#[test]
fn a_far_torus_shrinking_through_the_parameter_spacing() {
    let centre = 1e15;
    let u = 0.125;
    let origin = Point3::new(0.0, 0.0, 0.0);
    let dir = Vec3::new(1.0, 0.0, 0.0);
    let cases: [(f64, &[Want], &[f64]); 6] = [
        // Eight spacings apart (finding 1).
        (
            1.0,
            &[
                (true, 1, -2.0 - u, -2.0 + u),
                (true, 1, -1.0 - u, -1.0 + u),
                (true, 1, 1.0 - u, 1.0 + u),
                (true, 1, 2.0 - u, 2.0 + u),
            ],
            &[-2.0, -1.0, 1.0, 2.0],
        ),
        // Three spacings apart: two `f64` values strictly between the
        // roots of each pair, so they resolve, in windows one spacing
        // apart.
        (
            0.375,
            &[
                (true, 1, -0.75 - u, -0.75 + u),
                (true, 1, -0.375 - u, -0.375 + u),
                (true, 1, 0.375 - u, 0.375 + u),
                (true, 1, 0.75 - u, 0.75 + u),
            ],
            &[-0.75, -0.375, 0.375, 0.75],
        ),
        // Two spacings apart, the roots themselves `f64` values: one `f64`
        // strictly between the members of each pair. Each pair is one even
        // contact; the pairs are four spacings apart and stay separate.
        // The bare list still names all four roots.
        (
            0.25,
            &[
                (false, 2, -0.5 - u, -0.25 + u),
                (false, 2, 0.25 - u, 0.5 + u),
            ],
            &[-0.5, -0.25, 0.25, 0.5],
        ),
        // Off the grid, at `±1.375` and `±2.75` spacings: each root is
        // alone in its cell, but the cells of a pair are neighbours, with
        // one `f64` between the two roots. The inner gap holds three.
        (
            0.171_875,
            &[(false, 2, -0.375, -0.125), (false, 2, 0.125, 0.375)],
            &[-0.375, -0.125, 0.125, 0.375],
        ),
        // At `±0.375` and `±0.75` spacings: two roots in each cell about
        // the centre, which is the one `f64` between the inner two. The
        // bare list has the three `f64` values the four roots round to.
        (0.046_875, &[(false, 4, -u, u)], &[-0.125, 0.0, 0.125]),
        // The whole torus a hundredth of a spacing across.
        (1.0 / 1024.0, &[(false, 4, -u, u)], &[0.0]),
    ];
    for (s, want, bare) in cases {
        let torus = ToroidalSurface::new(Point3::new(centre, 0.0, 0.0), 1.5 * s, 0.5 * s).unwrap();
        let contacts = intersect_line_torus_contacts(&torus, origin, dir);
        let context = format!("s = {s}: {contacts:?}");
        assert_eq!(contacts.len(), want.len(), "{context}");
        assert_disjoint(&contacts);
        for (contact, &(resolved, roots, lo, hi)) in contacts.iter().zip(want) {
            assert_eq!(contact.resolved, resolved, "{context}");
            assert_eq!(contact.roots, roots, "{context}");
            assert_eq!(contact.t_lo.to_bits(), (centre + lo).to_bits(), "{context}");
            assert_eq!(contact.t_hi.to_bits(), (centre + hi).to_bits(), "{context}");
            let kind = if resolved {
                ContactKind::Transversal
            } else {
                // Every fused contact here holds an even number of roots.
                ContactKind::Tangential
            };
            assert_eq!(contact.kind, kind, "{context}");
        }
        // Every true root is strictly inside a window, and a window holds
        // as many as it says.
        let roots: Vec<f64> = [-2.0, -1.0, 1.0, 2.0].iter().map(|k| k * s).collect();
        for contact in &contacts {
            let held = roots
                .iter()
                .filter(|&&r| contact.t_lo - centre < r && r < contact.t_hi - centre)
                .count();
            assert_eq!(held, usize::from(contact.roots), "{context}");
        }
        let total: usize = contacts.iter().map(|c| usize::from(c.roots)).sum();
        assert_eq!(total, 4, "{context}");
        let plain = intersect_line_torus(&torus, origin, dir);
        let bare: Vec<f64> = bare.iter().map(|b| centre + b).collect();
        assert_eq!(plain, bare, "{context}");
    }
}

/// A parameter that overflows cannot place anything: a direction `1e-300`
/// long against a torus `1e300` across puts the crossings near `t = 1e600`.
/// The four roots come back as one even contact whose window is open to
/// infinity, not as a miss and not as resolved roots at infinity.
#[test]
fn an_overflowing_parameter_is_one_open_window() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5e300, 0.5e300).unwrap();
    let contacts = intersect_line_torus_contacts(
        &torus,
        Point3::new(-3e300, 0.0, 0.0),
        Vec3::new(1e-300, 0.0, 0.0),
    );
    assert_eq!(contacts.len(), 1, "{contacts:?}");
    let contact = contacts[0];
    assert!(!contact.resolved && contact.roots == 4, "{contact:?}");
    assert_eq!(contact.kind, ContactKind::Tangential, "{contact:?}");
    assert!(
        contact.t_lo.is_finite() && contact.t_lo >= 1e300 && contact.t_hi == f64::INFINITY,
        "{contact:?}"
    );
}
