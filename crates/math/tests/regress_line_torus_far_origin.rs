//! Line–torus roots from far origins and raw (unnormalised) directions.
//!
//! `intersect_line_torus` once formed its quartic about the caller's origin
//! and in units of the caller's direction. From an origin thousands of torus
//! sizes away the four roots cluster near `t = distance` and the solver's
//! absolute filters dropped or merged real crossings (1% of chords through
//! the tube lost a root at 1500 units, 94% at 5000); a raw direction shorter
//! than about 3.2e-4 made the leading coefficient `|dir|⁴` fall under the
//! degree-drop threshold and the quartic was solved as a cubic.
//!
//! Every root is checked here against an independent reference: the torus's
//! signed distance bisected along the same line, parameterised from a point
//! next to the torus.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::analytic_intersection::intersect_line_torus;
use remus_math::intersect::ContactKind;
use remus_math::ray::ray_torus;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};

/// `SplitMix64`: deterministic samples without a dependency.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    #[allow(clippy::cast_precision_loss)]
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn direction(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                2.0f64.mul_add(self.unit(), -1.0),
                2.0f64.mul_add(self.unit(), -1.0),
                2.0f64.mul_add(self.unit(), -1.0),
            );
            let l = v.length();
            if l > 0.1 && l <= 1.0 {
                return v * (1.0 / l);
            }
        }
    }
}

/// Signed distance to the torus (negative inside the tube).
fn signed_distance(t: &ToroidalSurface, p: Point3) -> f64 {
    let q = p - t.center();
    let (a, b, c) = (t.x_axis().dot(q), t.y_axis().dot(q), t.z_axis().dot(q));
    (a.hypot(b) - t.major_radius()).hypot(c) - t.minor_radius()
}

/// Reference crossings along `base + s·d` (unit `d`, `base` inside the tube),
/// by sampling the signed distance and bisecting each sign change. The flag
/// is false when a crossing is near-tangent or the line nearly touches the
/// surface without crossing: those lines are left out of the strict checks,
/// since a merged or dropped double root is a legitimate solver answer there.
fn reference(t: &ToroidalSurface, base: Point3, d: Vec3) -> (Vec<f64>, bool) {
    let reach = 2.0 * (t.major_radius() + t.minor_radius());
    // Samples a twentieth of a tube radius apart at most, so no crossing
    // pair hides between two samples unless it is a near-graze.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = ((2.0 * reach / (0.05 * t.minor_radius())).ceil() as u32).max(4000);
    let f = |s: f64| signed_distance(t, base + d * s);
    let h = 2.0 * reach / f64::from(n);
    let mut roots = Vec::new();
    let mut conditioned = true;
    let (mut s0, mut f0) = (-reach, f(-reach));
    let (mut prev_abs, mut falling, mut f_before) = (f0.abs(), false, f0);
    for i in 1..=n {
        let s1 = h.mul_add(f64::from(i), -reach);
        let f1 = f(s1);
        if f0 == 0.0 || (f0 < 0.0) != (f1 < 0.0) {
            let (mut lo, mut hi) = (s0, s1);
            let low_negative = f(lo) < 0.0;
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if (f(mid) < 0.0) == low_negative {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let r = 0.5 * (lo + hi);
            let eps = 1e-6 * t.minor_radius();
            if ((f(r + eps) - f(r - eps)) / (2.0 * eps)).abs() < 0.05 {
                conditioned = false;
            }
            roots.push(r);
        }
        let a = f1.abs();
        if falling
            && a > prev_abs
            && prev_abs < 0.02 * t.minor_radius()
            && (f_before < 0.0) == (f1 < 0.0)
            && (f0 < 0.0) == (f1 < 0.0)
        {
            conditioned = false;
        }
        falling = a < prev_abs;
        f_before = f0;
        prev_abs = a;
        s0 = s1;
        f0 = f1;
    }
    (roots, conditioned)
}

#[derive(Default, Debug)]
struct Tally {
    conditioned: usize,
    lost: usize,
    spurious: usize,
    max_error: f64,
}

/// Chords through the tube of `torus`, each from an origin `far` back along
/// its unit direction `d`, passed to the solver as `d · length`. Roots are
/// compared in model units.
fn chords(torus: &ToroidalSurface, far: f64, length: f64, n: usize, seed: u64) -> Tally {
    let mut rng = Rng(seed);
    let (big_r, small_r) = (torus.major_radius(), torus.minor_radius());
    let mut tally = Tally::default();
    for _ in 0..n {
        let phi = rng.unit() * std::f64::consts::TAU;
        let rad = small_r * rng.unit().sqrt() * 0.95;
        let psi = rng.unit() * std::f64::consts::TAU;
        let rho = rad.mul_add(psi.cos(), big_r);
        let p = torus.center()
            + torus.x_axis() * (rho * phi.cos())
            + torus.y_axis() * (rho * phi.sin())
            + torus.z_axis() * (rad * psi.sin());
        let d = rng.direction();
        let origin = p - d * far;
        // The line point `far` along from the origin, as the caller's
        // parameterisation reaches it.
        let base = origin + d * far;
        let (refs, conditioned) = reference(torus, base, d);
        if !conditioned {
            continue;
        }
        let got: Vec<f64> = intersect_line_torus(torus, origin, d * length)
            .into_iter()
            .map(|t| t * length)
            .collect();
        tally.conditioned += 1;
        let tol = 1e-3 * small_r;
        // Root errors relative to the magnitude of `t` near the torus.
        let magnitude = 1.0 + far + big_r + small_r;
        for s in &refs {
            let want = far + s;
            let best = got
                .iter()
                .map(|g| (g - want).abs())
                .fold(f64::INFINITY, f64::min);
            if best > tol {
                tally.lost += 1;
            } else {
                tally.max_error = tally.max_error.max(best / magnitude);
            }
        }
        tally.spurious += got
            .iter()
            .filter(|g| refs.iter().all(|s| ((far + s) - **g).abs() > tol))
            .count();
    }
    tally
}

fn assert_exact(label: &str, tally: &Tally, error_bound: f64) {
    assert!(tally.conditioned > 0, "{label}: no conditioned chord");
    assert_eq!(tally.lost, 0, "{label}: lost roots {tally:?}");
    assert_eq!(tally.spurious, 0, "{label}: spurious roots {tally:?}");
    assert!(
        tally.max_error <= error_bound,
        "{label}: error {:.3e} above {error_bound:.0e} ({tally:?})",
        tally.max_error
    );
}

/// Far origins: no chord loses a root, and every root is within a relative
/// `1e-12` of its reference (`|t - t_ref| <= 1e-12 (1 + far + R + r)`), up
/// to 10 000 tube-sized tori away. Unit-length directions, as the ray classifiers pass.
#[test]
fn far_origins_lose_no_roots() {
    let tilted = ToroidalSurface::with_axis(
        Point3::new(120.0, -40.0, 7.0),
        1.5,
        0.5,
        Vec3::new(0.3, -0.5, 0.81).normalize().unwrap(),
    )
    .unwrap();
    let tori = [
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap(),
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 0.3, 0.1).unwrap(),
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap(),
        tilted,
    ];
    for (k, torus) in tori.iter().enumerate() {
        for &far in &[0.0, 10.0, 1000.0, 3000.0, 5000.0, 10_000.0] {
            let tally = chords(torus, far, 1.0, 300, 0xC0FF_EE00 ^ far.to_bits() ^ k as u64);
            assert_exact(&format!("torus {k} far {far}"), &tally, 1e-12);
        }
    }
}

/// Raw direction vectors from 1e-5 to 1e4 long, as the section trimmer
/// passes a box edge's vector: roots come back in units of the vector and
/// none is lost, below the old `|dir|⁴ < 1e-14` degree-drop threshold too.
#[test]
fn raw_direction_lengths_lose_no_roots() {
    let unit = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    let small = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1e-3, 3e-4).unwrap();
    let tiny = ToroidalSurface::new(Point3::new(2e-4, -1e-4, 1e-3), 1e-4, 4e-5).unwrap();
    let lengths: [f64; 9] = [1e-5, 1e-4, 3e-4, 3.2e-4, 1e-3, 1.0, 37.0, 1e3, 1e4];
    for (k, torus) in [unit, small, tiny].iter().enumerate() {
        let size = torus.major_radius() + torus.minor_radius();
        for &length in &lengths {
            // Edge-like: the origin half an edge back, and a far-start edge.
            for &far in &[0.5 * length.min(10.0 * size), 1e3 * size] {
                let tally = chords(torus, far, length, 200, 0xB0B ^ length.to_bits() ^ k as u64);
                assert_exact(
                    &format!("torus {k} length {length:e} far {far:e}"),
                    &tally,
                    1e-12,
                );
            }
        }
    }
}

/// Large major-to-minor ratios keep every root, from next to the tube and
/// from the torus's centre.
#[test]
fn large_ratio_tori_lose_no_roots() {
    for &big in &[20.0, 300.0, 1500.0] {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), big, 0.5).unwrap();
        for &far in &[0.0, big] {
            let tally = chords(
                &torus,
                far,
                1.0,
                120,
                0x5EED ^ far.to_bits() ^ big.to_bits(),
            );
            assert_exact(&format!("R {big} far {far}"), &tally, 1e-12);
        }
    }
}

/// Zero and non-finite directions return what they always returned: no
/// roots for a zero direction, and the unshifted solve's output otherwise.
#[test]
fn degenerate_directions_find_no_roots() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    let origin = Point3::new(-5.0, 0.2, 0.1);
    assert!(intersect_line_torus(&torus, origin, Vec3::new(0.0, 0.0, 0.0)).is_empty());
    assert!(ray_torus(origin, Vec3::new(0.0, 0.0, 0.0), &torus).is_empty());
}

/// A tangent line from near and far: the contact is reported as one
/// tangential contact (the origin-based solve dropped it), its root agrees with
/// `intersect_line_torus` bit for bit, and `spread` is the honest window of
/// the contact, measured on the line's chart next to the torus and so the
/// same in the torus's units whatever the direction's length.
///
/// Along the tube top's circle the circumferential direction is asymptotic
/// (the top circle is parabolic), so this contact is a quadruple root: its
/// roots are fixed only to about the fourth root of the quartic's rounding
/// bound, a thousandth of the torus here, growing to a few thousandths as the
/// origin's own rounding grows with its distance. The solver used to report a
/// `1e-9` merge radius, which did not hold its roots.
#[test]
fn far_tangent_spread_is_the_local_merge_radius() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.5, 0.5).unwrap();
    // Along +y over the tube top at x = R: tangent at (R, 0, r).
    for &far in &[2.0, 1e3, 5e3] {
        for &length in &[1.0, 1e-2, 1e-5] {
            let origin = Point3::new(1.5, -far, 0.5);
            let d = Vec3::new(0.0, length, 0.0);
            let hits = ray_torus(origin, d, &torus);
            let plain = intersect_line_torus(&torus, origin, d);
            assert_eq!(hits.len(), plain.len());
            for (h, t) in hits.iter().zip(&plain) {
                assert_eq!(h.t.to_bits(), t.to_bits());
            }
            let contact = far / length;
            let local: Vec<_> = hits
                .iter()
                .filter(|h| (h.t - contact).abs() < 1e-3 / length)
                .collect();
            let context = format!("far {far} length {length:e}: {hits:?}");
            // One merged tangential contact at the closest point.
            assert_eq!(local.len(), 1, "{context}");
            assert_eq!(local[0].kind, ContactKind::Tangential, "{context}");
            // The exact contact is inside the window, and the window is set
            // by the torus, not by the origin's distance in units of `t`.
            assert!((local[0].t - contact).abs() <= local[0].spread, "{context}");
            assert!(
                local[0].spread > 0.0 && local[0].spread * length <= 1e-2,
                "{context}"
            );
        }
    }
}
