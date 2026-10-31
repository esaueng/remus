#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use proptest::prelude::*;

use super::*;

const RADII: [f64; 7] = [1e-3, 0.01, 0.37, 1.0, 13.0, 1e3, 1e6];
const OFFSETS: [f64; 5] = [0.0, 1e3, 1e6, 1e8, 1e9];

fn shift(offset: f64) -> Vec3 {
    Vec3::new(offset, -0.5 * offset, offset / 3.0)
}

fn kinds(hits: &RayHits) -> Vec<ContactKind> {
    hits.iter().map(|h| h.kind).collect()
}

/// Hits within `window` of `t`.
fn near(hits: &RayHits, t: f64, window: f64) -> Vec<RayHit> {
    hits.iter()
        .copied()
        .filter(|h| (h.t - t).abs() <= window)
        .collect()
}

/// A unit vector orthogonal to `n`, rotated by `theta` about it.
fn tangent_dir(n: Vec3, theta: f64) -> Vec3 {
    let helper = if n.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let e1 = n.cross(helper).normalize().unwrap();
    let e2 = n.cross(e1);
    e1 * theta.cos() + e2 * theta.sin()
}

/// Distance band within which a contact at this scale cannot be resolved,
/// with `d_ratio = |d| / |d⊥|` (1 for a sphere).
fn band(scale: f64, d_ratio: f64) -> f64 {
    ERR * scale * d_ratio
}

#[test]
fn quadratic_roots_cover_every_branch() {
    // Simple roots, ordered.
    let hits = quadratic_roots(&Quadratic::from_coefficients(1.0, -2.0, 3.0 - 4.0));
    assert_eq!(kinds(&hits), [ContactKind::Transversal; 2]);
    assert!((hits[0].t - (2.0 - 5.0_f64.sqrt())).abs() < 1e-15);
    assert!((hits[1].t - (2.0 + 5.0_f64.sqrt())).abs() < 1e-15);
    // Exact double root: tangential with a tiny spread.
    let hits = quadratic_roots(&Quadratic::from_coefficients(1.0, -1.0, 1.0));
    assert_eq!(kinds(&hits), [ContactKind::Tangential]);
    assert!((hits[0].t - 1.0).abs() < 1e-15 && hits[0].spread < 1e-6);
    // No real roots.
    assert!(quadratic_roots(&Quadratic::from_coefficients(1.0, 0.0, 1.0)).is_empty());
    // Linear: one transversal root.
    let hits = quadratic_roots(&Quadratic::from_coefficients(0.0, 1.0, -4.0));
    assert_eq!(kinds(&hits), [ContactKind::Transversal]);
    assert!((hits[0].t - 2.0).abs() < 1e-15);
    // Leading coefficient within its bound: the far root is unresolved.
    let q = Quadratic {
        a_err: 1e-12,
        ..Quadratic::from_coefficients(1e-13, 1.0, -4.0)
    };
    let hits = quadratic_roots(&q);
    assert_eq!(
        kinds(&hits),
        [ContactKind::Unclassified, ContactKind::Transversal]
    );
    assert!(hits[0].spread.is_infinite() && (hits[1].t - 2.0).abs() < 1e-12);
    // Identically zero: the line lies in the surface.
    let hits = quadratic_roots(&Quadratic::from_coefficients(0.0, 0.0, 0.0));
    assert_eq!(kinds(&hits), [ContactKind::Unclassified]);
    // Non-finite input: nothing.
    assert!(quadratic_roots(&Quadratic::from_coefficients(f64::NAN, 1.0, 1.0)).is_empty());
}

/// The cancellation-free cone discriminant equals the direct `h² − a c` on
/// well-conditioned input.
#[test]
fn cone_discriminant_identity_matches_direct_form() {
    let cone =
        ConicalSurface::new(Point3::new(0.3, -0.2, 0.1), Vec3::new(0.2, 0.3, 1.0), 0.6).unwrap();
    let origin = Point3::new(2.0, 1.0, -3.0);
    let direction = Vec3::new(-0.3, 0.1, 0.9);
    let hits = ray_cone(origin, direction, &cone);
    let (sn, cs) = cone.half_angle().sin_cos();
    let axis = cone.axis();
    let ov = origin - cone.apex();
    let q = |x: Vec3, y: Vec3| {
        cs * cs * x.dot(axis) * y.dot(axis) - sn * sn * (x.dot(y) - x.dot(axis) * y.dot(axis))
    };
    let (a, h, c) = (q(direction, direction), q(ov, direction), q(ov, ov));
    let direct = quadratic_roots(&Quadratic::from_coefficients(a, h, c));
    assert_eq!(hits.len(), direct.len());
    for (x, y) in hits.iter().zip(direct.iter()) {
        assert_eq!(x.kind, y.kind);
        assert!((x.t - y.t).abs() < 1e-12 * (1.0 + y.t.abs()));
    }
}

/// RFC 0008 F3 at the solver: the two lines tangent to a z-axis cylinder
/// along the check classifier's first two base directions are tangential,
/// at every radius and translation, and so are lines pushed outward or
/// inward by less than the band; pushed further they miss or cross twice.
#[test]
fn cylinder_tangent_lines_are_tangential_across_scales() {
    let dirs = [
        Vec3::new(
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
            0.350_889_803_483_932_2,
        ),
        Vec3::new(
            -0.350_889_803_483_932_2,
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
        ),
    ];
    for &r in &RADII {
        for &offset in &OFFSETS {
            let by = shift(offset);
            let center = Point3::new(by.x(), by.y(), by.z());
            let cyl = CylindricalSurface::new(center, Vec3::new(0.0, 0.0, 1.0), r).unwrap();
            for &d in &dirs {
                let d_ratio = d.length() / d.x().hypot(d.y());
                // Tangent point at angle phi, line along d through it.
                let phi = 1.1_f64;
                let radial = Vec3::new(phi.cos(), phi.sin(), 0.0);
                let tangent_point = center + radial * r + Vec3::new(0.0, 0.0, 3.0 * r);
                // Remove d's radial component so the line stays tangent.
                let d = d - radial * d.dot(radial);
                let lambda = 2.0 * r;
                let origin = tangent_point - d * (lambda / d.length());
                let scale = l1(origin) + l1(center) + r;
                let hits = ray_cylinder(origin, d, &cyl);
                assert_eq!(
                    kinds(&hits),
                    [ContactKind::Tangential],
                    "exact tangent r {r:e} offset {offset:e}: {hits:?}"
                );
                let t_true = lambda / d.length();
                assert!(
                    (hits[0].t - t_true).abs() <= hits[0].spread + 1e-9 * t_true,
                    "tangent t r {r:e} offset {offset:e}: {hits:?} vs {t_true}"
                );
                let b = band(scale, d_ratio);
                // Out of the band: a miss outward, two crossings inward.
                if 1e3 * b < 1e-2 * r {
                    let out = ray_cylinder(origin + radial * (1e3 * b), d, &cyl);
                    assert!(out.is_empty(), "outward r {r:e} offset {offset:e}: {out:?}");
                    let inward = ray_cylinder(origin - radial * (1e3 * b), d, &cyl);
                    assert_eq!(
                        kinds(&inward),
                        [ContactKind::Transversal; 2],
                        "inward r {r:e} offset {offset:e}: {inward:?}"
                    );
                    assert!(inward[0].t < t_true && t_true < inward[1].t);
                }
            }
        }
    }
}

/// Axis-parallel rays: off the wall they meet nothing; on it they lie in the
/// surface and come back unresolved. Rays perpendicular to the axis (in the
/// radial plane) through the axis cross twice at `±r` around it.
#[test]
fn cylinder_axis_parallel_and_in_plane_rays() {
    for &r in &RADII {
        for &offset in &OFFSETS {
            let by = shift(offset);
            let center = Point3::new(by.x(), by.y(), by.z());
            let cyl = CylindricalSurface::new(center, Vec3::new(0.0, 0.0, 1.0), r).unwrap();
            let up = Vec3::new(0.0, 0.0, 1.0);
            assert!(ray_cylinder(center + Vec3::new(0.5 * r, 0.0, 0.0), up, &cyl).is_empty());
            assert!(ray_cylinder(center + Vec3::new(2.0 * r, 0.0, 0.0), up, &cyl).is_empty());
            let on = ray_cylinder(center + Vec3::new(0.0, r, 0.0), up, &cyl);
            assert_eq!(
                kinds(&on),
                [ContactKind::Unclassified],
                "r {r:e} offset {offset:e}"
            );
            let side = Vec3::new(0.6, 0.8, 0.0);
            let origin = center - side * (5.0 * r) + up * r;
            let hits = ray_cylinder(origin, side, &cyl);
            assert_eq!(kinds(&hits), [ContactKind::Transversal; 2]);
            let tol = 1e3 * ERR * (l1(origin) + l1(center) + r);
            assert!(
                (hits[0].t - 4.0 * r).abs() <= tol,
                "{hits:?} r {r:e} offset {offset:e}"
            );
            assert!(
                (hits[1].t - 6.0 * r).abs() <= tol,
                "{hits:?} r {r:e} offset {offset:e}"
            );
        }
    }
}

#[test]
fn sphere_tangent_lines_are_tangential_across_scales() {
    for &r in &RADII {
        for &offset in &OFFSETS {
            let by = shift(offset);
            let center = Point3::new(by.x(), by.y(), by.z());
            let sph = SphericalSurface::new(center, r).unwrap();
            for theta in [0.0, 0.7, 2.9] {
                let n = Vec3::new(0.3, -0.5, 0.8).normalize().unwrap();
                let d = tangent_dir(n, theta);
                let tangent_point = center + n * r;
                let origin = tangent_point - d * (2.0 * r);
                let hits = ray_sphere(origin, d, &sph);
                assert_eq!(
                    kinds(&hits),
                    [ContactKind::Tangential],
                    "r {r:e} offset {offset:e}"
                );
                assert!((hits[0].t - 2.0 * r).abs() <= hits[0].spread + 1e-9 * r);
                let b = band(l1(origin) + l1(center) + r, 1.0);
                if 1e3 * b < 1e-2 * r {
                    assert!(ray_sphere(origin + n * (1e3 * b), d, &sph).is_empty());
                    let inward = ray_sphere(origin - n * (1e3 * b), d, &sph);
                    assert_eq!(kinds(&inward), [ContactKind::Transversal; 2]);
                }
            }
        }
    }
}

#[test]
fn cone_tangent_lines_and_apex_rays() {
    for &r in &RADII {
        for &offset in &OFFSETS {
            let by = shift(offset);
            let apex = Point3::new(by.x(), by.y(), by.z());
            let cone = ConicalSurface::new(apex, Vec3::new(0.1, -0.2, 1.0), 0.9).unwrap();
            let (u, v) = (0.8, 5.0 * r);
            let tangent_point = cone.evaluate(u, v);
            let n = cone.normal(u, v);
            // Tangent-plane direction away from the generator, so the line
            // touches the cone at one point.
            let generator = (tangent_point - apex).normalize().unwrap();
            let d = n.cross(generator).normalize().unwrap();
            let origin = tangent_point - d * r;
            let hits = ray_cone(origin, d, &cone);
            let window = 0.5 * r;
            let local = near(&hits, r, window);
            assert_eq!(local.len(), 1, "r {r:e} offset {offset:e}: {hits:?}");
            assert_eq!(
                local[0].kind,
                ContactKind::Tangential,
                "r {r:e} offset {offset:e}"
            );
            let b = band(l1(origin) + l1(apex), 1.0);
            if 1e4 * b < 1e-3 * r {
                let out = ray_cone(origin + n * (1e4 * b), d, &cone);
                assert!(
                    near(&out, r, window).is_empty(),
                    "outward r {r:e} offset {offset:e}"
                );
                let inward = ray_cone(origin - n * (1e4 * b), d, &cone);
                let local = near(&inward, r, window);
                assert_eq!(
                    local.len(),
                    2,
                    "inward r {r:e} offset {offset:e}: {inward:?}"
                );
                assert!(local.iter().all(|h| h.kind == ContactKind::Transversal));
            }
            // Through the apex: a singular contact, never a certified one.
            let through = ray_cone(
                apex - Vec3::new(0.3, 0.9, 0.2) * r,
                Vec3::new(0.3, 0.9, 0.2),
                &cone,
            );
            assert!(
                through.iter().any(|h| h.kind == ContactKind::Unclassified),
                "apex r {r:e} offset {offset:e}: {through:?}"
            );
        }
    }
}

fn log_uniform(lo: f64, hi: f64) -> impl Strategy<Value = f64> {
    (lo.ln()..hi.ln()).prop_map(f64::exp)
}

fn unit_vec() -> impl Strategy<Value = Vec3> {
    (0.0..std::f64::consts::TAU, -1.0..1.0_f64).prop_map(|(phi, z): (f64, f64)| {
        let s = (1.0 - z * z).sqrt();
        Vec3::new(s * phi.cos(), s * phi.sin(), z)
    })
}

fn offset_vec() -> impl Strategy<Value = Vec3> {
    (unit_vec(), prop_oneof![Just(0.0), log_uniform(1e-3, 1e9)]).prop_map(|(u, m)| u * m)
}

/// Check computed roots against the ones the ray was built through: `pts`
/// lie on the surface at `t_true` along `origin + t·d`; each must match a
/// transversal root within the rounding the construction and the solve can
/// take, amplified by `1 / sin(crossing angle)`.
fn assert_matches(hits: &RayHits, t_true: [f64; 2], sin_angle: f64, scale: f64) {
    assert_eq!(hits.len(), 2, "{hits:?}");
    for (hit, t) in hits.iter().zip(t_true) {
        assert_eq!(hit.kind, ContactKind::Transversal, "{hits:?}");
        let tol = 1e4 * f64::EPSILON * scale / sin_angle;
        assert!(
            (hit.t - t).abs() <= tol,
            "{hits:?} vs {t_true:?} (tol {tol:e})"
        );
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    /// Random secants of a random cylinder: the solver returns exactly the
    /// two parameters the line was built through.
    #[test]
    fn cylinder_secants_match_their_construction(
        r in log_uniform(1e-3, 1e6),
        center in offset_vec(),
        axis in unit_vec(),
        (u1, du) in (0.0..std::f64::consts::TAU, 0.3..(std::f64::consts::TAU - 0.3)),
        (v1, v2) in (-10.0..10.0_f64, -10.0..10.0_f64),
        lambda in 0.1..10.0_f64,
    ) {
        let cyl = CylindricalSurface::new(Point3::new(center.x(), center.y(), center.z()), axis, r)
            .unwrap();
        let p1 = cyl.evaluate(u1, v1 * r);
        let p2 = cyl.evaluate(u1 + du, v2 * r);
        let chord = p2 - p1;
        let len = chord.length();
        let d = chord * (1.0 / len);
        let origin = p1 - d * (lambda * len);
        let hits = ray_cylinder(origin, d, &cyl);
        // In the radial plane the chord meets the wall at angle du/2; the
        // axial component tilts it further out of the wall's tangent plane.
        let d_perp = (d - cyl.axis() * d.dot(cyl.axis())).length();
        let sin_angle = (0.5 * du).sin() * d_perp;
        let scale = l1(origin) + l1(cyl.origin()) + r;
        // A secant closer to tangency than the rounding band is rightly
        // reported tangential; keep to the ones the coordinates resolve.
        prop_assume!(r * (1.0 - (0.5 * du).cos()) > 1e3 * band(scale, 1.0 / d_perp));
        assert_matches(&hits, [lambda * len, (lambda + 1.0) * len], sin_angle, scale);
    }

    #[test]
    fn sphere_secants_match_their_construction(
        r in log_uniform(1e-3, 1e6),
        center in offset_vec(),
        n1 in unit_vec(),
        n2 in unit_vec(),
        lambda in 0.1..10.0_f64,
    ) {
        let gap = n1.dot(n2);
        prop_assume!(gap < 0.95);
        let c = Point3::new(center.x(), center.y(), center.z());
        let sph = SphericalSurface::new(c, r).unwrap();
        let (p1, p2) = (c + n1 * r, c + n2 * r);
        let chord = p2 - p1;
        let len = chord.length();
        let d = chord * (1.0 / len);
        let origin = p1 - d * (lambda * len);
        let hits = ray_sphere(origin, d, &sph);
        let sin_angle = (0.5 * (1.0 - gap)).sqrt();
        let scale = l1(origin) + l1(c) + r;
        // Distance of the chord from tangency: r (1 − cos(half angle)).
        prop_assume!(r * (1.0 - (0.5 * (1.0 + gap)).sqrt()) > 1e3 * band(scale, 1.0));
        assert_matches(&hits, [lambda * len, (lambda + 1.0) * len], sin_angle, scale);
    }

    /// Random tangent lines of a random sphere are tangential, never a pair
    /// of crossings or a miss, and land on the tangent point.
    #[test]
    fn sphere_tangent_lines_are_tangential(
        r in log_uniform(1e-3, 1e6),
        center in offset_vec(),
        n in unit_vec(),
        theta in 0.0..std::f64::consts::TAU,
        lambda in 0.1..10.0_f64,
    ) {
        let c = Point3::new(center.x(), center.y(), center.z());
        let sph = SphericalSurface::new(c, r).unwrap();
        let d = tangent_dir(n, theta);
        let origin = c + n * r - d * (lambda * r);
        let hits = ray_sphere(origin, d, &sph);
        prop_assert_eq!(kinds(&hits), vec![ContactKind::Tangential]);
        prop_assert!((hits[0].t - lambda * r).abs() <= hits[0].spread + 1e-9 * r);
    }

    /// Random tangent lines of a random cylinder, kept off the axis
    /// direction (a line along the axis lies in the wall).
    #[test]
    fn cylinder_tangent_lines_are_tangential(
        r in log_uniform(1e-3, 1e6),
        center in offset_vec(),
        axis in unit_vec(),
        (u, v) in (0.0..std::f64::consts::TAU, -10.0..10.0_f64),
        theta in 0.0..std::f64::consts::TAU,
        lambda in 0.1..10.0_f64,
    ) {
        let cyl = CylindricalSurface::new(Point3::new(center.x(), center.y(), center.z()), axis, r)
            .unwrap();
        let n = cyl.normal(u, v * r);
        let d = tangent_dir(n, theta);
        prop_assume!(d.cross(cyl.axis()).length() > 0.1);
        let origin = cyl.evaluate(u, v * r) - d * (lambda * r);
        let hits = ray_cylinder(origin, d, &cyl);
        prop_assert_eq!(kinds(&hits), vec![ContactKind::Tangential], "{:?}", hits);
        prop_assert!((hits[0].t - lambda * r).abs() <= hits[0].spread + 1e-9 * r);
    }
}

/// Hits within `window` of `t` from a torus solve.
fn near_torus(hits: &[RayHit], t: f64, window: f64) -> Vec<RayHit> {
    hits.iter()
        .copied()
        .filter(|h| (h.t - t).abs() <= window)
        .collect()
}

/// Whether the hits near a contact give a certified odd parity there: exactly
/// one transversal root and nothing else. A tangent contact must never.
fn reads_as_one_crossing(local: &[RayHit]) -> bool {
    local.len() == 1 && local[0].kind == ContactKind::Transversal
}

/// Tangent lines to a torus, on the convex outer part (exterior tangency) and
/// on the saddle-shaped inner part including the inner equator, across
/// scales and translations. `intersect_line_torus` merges a double root into
/// one root for a large share of them (about 18% of the outer ones before
/// this change); `ray_torus` must never return such a contact as a single
/// transversal crossing.
#[test]
fn torus_tangent_lines_never_read_as_one_crossing() {
    let mut merged = 0usize;
    let mut total = 0usize;
    for &s in &[1e-3, 0.37, 1.0, 13.0, 1e3] {
        for &offset in &[0.0, 1e3, 1e6] {
            let by = shift(offset);
            let tor =
                ToroidalSurface::new(Point3::new(by.x(), by.y(), by.z()), 3.0 * s, s).unwrap();
            for iu in 0..24 {
                for iv in 0..24 {
                    for it in 0..4 {
                        let u = 0.1 + f64::from(iu) * 0.26;
                        // v = π exactly is on the grid: the inner equator.
                        let v = f64::from(iv) * std::f64::consts::TAU / 24.0;
                        let p = tor.evaluate(u, v);
                        let n = tor.normal(u, v);
                        let d = tangent_dir(n, 0.2 + f64::from(it) * 0.8);
                        let origin = p - d * (2.0 * s);
                        let hits = ray_torus(origin, d, &tor);
                        let local = near_torus(&hits, 2.0 * s, 0.05 * s);
                        total += 1;
                        let plain =
                            crate::analytic_intersection::intersect_line_torus(&tor, origin, d);
                        if plain
                            .iter()
                            .filter(|&&t| (t - 2.0 * s).abs() <= 0.05 * s)
                            .count()
                            == 1
                        {
                            merged += 1;
                        }
                        // True parity over the window, from the implicit's
                        // sign at its ends. Near an asymptotic direction of
                        // the saddle-shaped inner part the line also crosses
                        // close by, so the window may hold an odd count.
                        let implicit = |t: f64| {
                            let q = origin + d * t - tor.center();
                            let rho = q.x().hypot(q.y());
                            (rho - tor.major_radius()).hypot(q.z()) - tor.minor_radius()
                        };
                        let (f0, f1) = (implicit(1.95 * s), implicit(2.05 * s));
                        if f0.abs() > 1e-6 * s && f1.abs() > 1e-6 * s && f0.signum() == f1.signum()
                        {
                            assert!(
                                !reads_as_one_crossing(&local),
                                "s {s:e} offset {offset:e} u {u} v {v}: {hits:?}"
                            );
                        }
                        // Same roots as the plain solve, in the same order.
                        assert_eq!(hits.len(), plain.len());
                        for (h, t) in hits.iter().zip(&plain) {
                            assert_eq!(h.t.to_bits(), t.to_bits());
                        }
                    }
                }
            }
        }
    }
    // The fixture exercises the merged case it guards against.
    assert!(
        merged > total / 20,
        "only {merged} of {total} tangents merged"
    );
}

/// Merged double roots carry an even multiplicity.
#[test]
fn torus_tangent_through_tube_top_reports_an_even_multiplicity() {
    // Horizontal line along +y over the tube top at x = R: tangent at (R, 0, r).
    let tor = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
    let counted = crate::analytic_intersection::intersect_line_torus_counted(
        &tor,
        Point3::new(3.0, -2.0, 1.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    let local: Vec<_> = counted
        .iter()
        .filter(|(t, _)| (t - 2.0).abs() < 1e-3)
        .collect();
    let total: usize = local.iter().map(|(_, m)| m).sum();
    assert!(total.is_multiple_of(2), "{counted:?}");
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    /// Random secants of a random torus through two known surface points,
    /// crossing at a resolvable angle: both come back transversal at the
    /// constructed parameters.
    #[test]
    fn torus_secants_match_their_construction(
        s in log_uniform(1e-3, 1e3),
        center in offset_vec(),
        (u1, v1) in (0.0..std::f64::consts::TAU, 0.0..std::f64::consts::TAU),
        (u2, v2) in (0.0..std::f64::consts::TAU, 0.0..std::f64::consts::TAU),
        lambda in 0.1..10.0_f64,
    ) {
        let c = Point3::new(center.x(), center.y(), center.z());
        let tor = ToroidalSurface::new(c, 3.0 * s, s).unwrap();
        let (p1, p2) = (tor.evaluate(u1, v1), tor.evaluate(u2, v2));
        let chord = p2 - p1;
        let len = chord.length();
        prop_assume!(len > 0.5 * s);
        let d = chord * (1.0 / len);
        // Keep crossings well away from tangency at both points.
        prop_assume!(tor.normal(u1, v1).dot(d).abs() > 0.2);
        prop_assume!(tor.normal(u2, v2).dot(d).abs() > 0.2);
        prop_assume!(l1(c) < 1e5 * s);
        let origin = p1 - d * (lambda * len);
        let hits = ray_torus(origin, d, &tor);
        for t in [lambda * len, (lambda + 1.0) * len] {
            let tol = 1e-6 * s;
            let local = near_torus(&hits, t, tol);
            prop_assert_eq!(local.len(), 1, "t {} in {:?}", t, hits);
            prop_assert_eq!(local[0].kind, ContactKind::Transversal);
        }
    }
}
