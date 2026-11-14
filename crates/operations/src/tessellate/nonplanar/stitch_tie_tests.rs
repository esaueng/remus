//! `stitch_rings` must pick the same quad diagonals when aligned ring
//! longitudes differ only by rounding: the boundary ring's longitudes come
//! from `atan2`, whose last bit differs between the native C runtime and
//! wasm32's libm, and a strict comparison let that bit choose the diagonal
//! (B36: `cone-sphere` at 1e-3 rigid meshed its sphere differently per
//! target, leaving the mesh fallback one edge and one vertex apart).
#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use std::f64::consts::TAU;

use super::{LatRing, TriangleMesh, stitch_advances_lower, stitch_rings};

fn ring(n: usize, first_id: u32, nudge: impl Fn(f64) -> f64) -> LatRing {
    (0..n)
        .map(|k| (nudge(TAU * k as f64 / n as f64), first_id + k as u32))
        .collect()
}

fn stitched(lo: &LatRing, hi: &LatRing) -> Vec<[u32; 3]> {
    let mut mesh = TriangleMesh::default();
    stitch_rings(&mut mesh, lo, hi, &|m: &mut TriangleMesh, a, b, c| {
        m.indices.extend_from_slice(&[a, b, c]);
    });
    mesh.indices
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .collect()
}

#[test]
fn aligned_rings_stitch_identically_under_one_ulp_of_longitude_noise() {
    let n = 24;
    let hi = ring(n, 100, |u| u);
    let exact = stitched(&ring(n, 0, |u| u), &hi);
    assert_eq!(exact.len(), 2 * n, "one triangle per advance");
    // A tie advances the lower ring first: the apex stays on `hi`.
    assert_eq!(exact[0], [0, 1, 100]);
    for (label, nudge) in [
        ("up", f64::next_up as fn(f64) -> f64),
        ("down", f64::next_down as fn(f64) -> f64),
    ] {
        assert_eq!(
            stitched(&ring(n, 0, nudge), &hi),
            exact,
            "lower ring {label} one ulp"
        );
        assert_eq!(
            stitched(&ring(n, 0, |u| u), &ring(n, 100, nudge)),
            exact,
            "upper ring {label} one ulp"
        );
    }
}

#[test]
fn offset_rings_still_follow_longitude_order() {
    // Half-step phase: genuinely interleaved columns are not ties.
    let n = 8;
    let lo = ring(n, 0, |u| u);
    let hi = ring(n, 100, |u| u + TAU / (2 * n) as f64);
    let tris = stitched(&lo, &hi);
    assert_eq!(tris.len(), 2 * n);
    // lo's next vertex (pi/4) precedes hi's next (3pi/8), which precedes
    // lo's following one (pi/2): the rings alternate.
    assert_eq!(tris[0], [0, 1, 100]);
    assert_eq!(tris[1], [1, 101, 100]);
}

#[test]
fn the_tie_band_is_a_few_ulps_wide() {
    for x in [0.25, TAU, 2.0 * TAU, 7.0 * TAU] {
        let mut above = x;
        for _ in 0..4 {
            above = above.next_up();
        }
        // Four ulps of rounding: a tie, which advances the lower ring.
        assert!(stitch_advances_lower(above, x), "4 ulp at {x}");
    }
    // A deterministic near-coincidence (8e-14 rad, the smallest seen between
    // unequal rings in the suites, at unwrapped longitudes up to 4π) keeps
    // its longitude order.
    for x in [0.25, TAU, 2.0 * TAU] {
        assert!(!stitch_advances_lower(x + 8e-14, x), "8e-14 at {x}");
        assert!(stitch_advances_lower(x, x + 8e-14));
    }
    // A finished ring never advances again.
    assert!(!stitch_advances_lower(f64::INFINITY, TAU));
    assert!(stitch_advances_lower(TAU, f64::INFINITY));
}
