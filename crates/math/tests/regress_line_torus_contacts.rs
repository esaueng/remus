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
//! checked against the sign of the exact tube distance along the same line:
//! the number of crossings `ray_torus` reports near a contact (transversal and
//! unclassified hits; a tangential one adds none) must have the parity of the
//! distance's sign change across it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::analytic_intersection::intersect_line_torus;
use remus_math::intersect::ContactKind;
use remus_math::ray::{RayHit, ray_torus};
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

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.unit(), lo)
    }

    fn direction(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            let l = v.length();
            if l > 0.1 && l <= 1.0 {
                return v * (1.0 / l);
            }
        }
    }
}

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

/// The point at `(u, v)` with the unit circumferential and meridional
/// tangents and the outward normal there.
fn frame(t: &ToroidalSurface, u: f64, v: f64) -> (Point3, Vec3, Vec3, Vec3) {
    let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
    let e_u = t.y_axis() * u.cos() - t.x_axis() * u.sin();
    let e_v = t.z_axis() * v.cos() - radial * v.sin();
    let n = radial * v.cos() + t.z_axis() * v.sin();
    (t.evaluate(u, v), e_u, e_v, n)
}

/// The tangent direction at `(u, v)` whose normal curvature is `ratio` times
/// the asymptotic ratio: `ratio = 1` is an asymptotic direction (the second
/// fundamental form vanishes on it), below one the line enters the tube.
/// Only on the inner half (`cos v < 0`), where the Gaussian curvature is
/// negative.
fn inner_direction(t: &ToroidalSurface, u: f64, v: f64, ratio: f64, sign: f64) -> Vec3 {
    let (_, e_u, e_v, _) = frame(t, u, v);
    let rho = t.major_radius() + t.minor_radius() * v.cos();
    // cos v / ρ · a² + 1 / r · b² = 0 on an asymptotic direction.
    let slope = ratio * (-t.minor_radius() * v.cos() / rho).sqrt();
    (e_u + e_v * (sign * slope)).normalize().unwrap()
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

/// The reviewed ray: an asymptotic tangent at `v = 2π/3` on the unit-scale
/// torus. The solver dropped its triple root at `t = 0.2`, and the ray, from
/// outside, counted the far crossing alone.
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
    let context = format!("{hits:?}");
    // The distance changes sign across t = 0.2: a real crossing.
    assert!(distance_along(&torus, origin, dir, 0.19) > 0.0, "{context}");
    assert!(distance_along(&torus, origin, dir, 0.21) < 0.0, "{context}");
    let triple: Vec<_> = hits.iter().filter(|h| (h.t - 0.2).abs() <= 1e-6).collect();
    assert_eq!(triple.len(), 1, "{context}");
    assert_ne!(triple[0].kind, ContactKind::Tangential, "{context}");
    assert!(triple[0].spread.is_finite(), "{context}");
    let far: Vec<_> = hits
        .iter()
        .filter(|h| (h.t - 2.321_320_343_559_642_6).abs() <= 1e-9)
        .collect();
    assert_eq!(far.len(), 1, "{context}");
    assert_eq!(far[0].kind, ContactKind::Transversal, "{context}");
    // Odd parity across the triple contact, and the ray's count from its
    // outside origin is even.
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
/// contact even. Each is reported, once, as a contact that adds no crossing
/// and is never transversal.
#[test]
fn touching_lines_are_one_even_contact() {
    let mut rng = Rng(0x2C4E_6A8B_1D3F_5072);
    let mut checked = [0_usize; 3];
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
                let at = back * size;
                let w = 1e-3 * size;
                let (before, after) = (
                    distance_along(&torus, origin, unit, at - w),
                    distance_along(&torus, origin, unit, at + w),
                );
                let context = format!(
                    "kind {kind} scale {scale:e} back {back:e} u {u} v {v}: \
                     distance {before:e} .. {after:e}, hits {hits:?}"
                );
                assert!((before > 0.0) == (after > 0.0), "{context}");
                let near = in_window(&hits, at - w, at + w);
                assert_eq!(near.len(), 1, "one contact: {context}");
                assert_eq!(near[0].kind, ContactKind::Tangential, "{context}");
                assert!(
                    (near[0].t - at).abs() <= near[0].spread.max(1e-9 * size),
                    "the window holds the contact: {context}"
                );
                checked[kind] += 1;
            }
        }
    }
    assert!(checked.iter().all(|&n| n >= 300), "{checked:?}");
}

/// Lines moved off an exterior tangency and off an asymptotic tangency along
/// the normal, by `1e-15` to `1e-3` torus sizes, either side. Inward of an
/// exterior touch the line crosses twice (from about `1e-6` on, far enough
/// apart to report both); outward it misses; across an asymptotic tangency
/// it crosses once either way. The reported crossings near the contact must
/// have the parity the distance's sign change gives, and a line that meets
/// the tube inward of a touch must get a contact, however close its two
/// crossings are.
#[test]
fn near_tangent_lines_keep_parity_either_side() {
    let mut rng = Rng(0x55AA_1234_9876_F00D);
    let mut resolved_pairs = 0_usize;
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
                        let w = 0.05 * size;
                        let (before, after) = (
                            distance_along(&torus, origin, unit, back - w),
                            distance_along(&torus, origin, unit, back + w),
                        );
                        let context = format!(
                            "scale {scale:e} asymptotic {asymptotic} offset {offset:e} \
                             side {side} v {v}: distance {before:e} .. {after:e}, hits {hits:?}"
                        );
                        let near = in_window(&hits, back - w, back + w);
                        let count: usize = near.iter().map(crossings).sum();
                        let truth = usize::from(before.signum() != after.signum());
                        assert_eq!(count % 2, truth, "{context}");
                        if asymptotic {
                            if offset <= 1e-9 {
                                assert_eq!(truth, 1, "{context}");
                            }
                        } else if side < 0.0 && (1e-7..1e-4).contains(&offset) {
                            // Inward far enough to separate: both crossings.
                            assert_eq!(count, 2, "{context}");
                            resolved_pairs += 1;
                        }
                        if !asymptotic && side < 0.0 && offset <= 1e-6 {
                            // Inward of a touch the line meets the tube: the
                            // contact is reported (as one tangential contact
                            // or as both crossings), never dropped.
                            assert!(!near.is_empty(), "{context}");
                        }
                    }
                }
            }
        }
    }
    assert!(resolved_pairs >= 100, "{resolved_pairs}");
}
