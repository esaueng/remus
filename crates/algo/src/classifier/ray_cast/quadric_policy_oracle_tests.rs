//! Bit-identity oracle for the ray-cast classifier's quadric and plane hits
//! routed through `remus_math::ray` (RFC 0008 stage 1).
//!
//! `plane_hit`, `ray_cylinder_crossings` and `ray_cone_crossings` now take
//! their roots from `plane_parameter` and `ray_{cylinder,cone}_with` under
//! `TangentPolicy::DropRelative`. The functions as they were before are kept
//! verbatim below, and both are run on random rays, exactly and nearly
//! tangent rays, axis-parallel and generator-parallel rays, rays through the
//! apex and rays lying in a plane, at scales from 1e-3 to 1e6. The roots must
//! agree bit for bit and the crossing counts, suspicion flags and plane hits
//! exactly.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops,
    clippy::too_many_arguments
)]

use std::f64::consts::TAU;

use proptest::prelude::*;
use remus_math::intersect::ContactKind;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface};

use super::*;

/// The old `plane_hit`, verbatim but for its work counter.
fn reference_plane_hit(
    origin: Point3,
    ray_dir: Vec3,
    denom: f64,
    numer: f64,
    tol: Tolerance,
) -> PlaneHit {
    let near = 10.0 * tol.linear;
    if denom.abs() < tol.angular {
        return PlaneHit::Parallel(numer.abs() <= near);
    }
    let t = numer / denom;
    if t <= tol.linear {
        return PlaneHit::Behind;
    }
    PlaneHit::Ahead(Point3::new(
        origin.x() + ray_dir.x() * t,
        origin.y() + ray_dir.y() * t,
        origin.z() + ray_dir.z() * t,
    ))
}

/// The old root computation of `ray_cylinder_crossings`, verbatim.
fn reference_cylinder_roots(
    origin: Point3,
    ray_dir: Vec3,
    surface: &CylindricalSurface,
) -> Vec<f64> {
    let axis = surface.axis();
    let m = origin - surface.origin();
    let d_perp = ray_dir - axis * ray_dir.dot(axis);
    let m_perp = m - axis * m.dot(axis);

    let a = d_perp.dot(d_perp);
    if a < 1e-14 {
        return Vec::new();
    }
    let b = 2.0 * m_perp.dot(d_perp);
    let c = surface
        .radius()
        .mul_add(-surface.radius(), m_perp.dot(m_perp));
    let disc = b.mul_add(b, -4.0 * a * c);
    if disc < 1e-12 * a * surface.radius() * surface.radius() {
        return Vec::new();
    }
    let sqrt_disc = disc.sqrt();
    vec![(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)]
}

/// The old `ray_cylinder_crossings`, verbatim.
fn reference_cylinder_crossings(
    origin: Point3,
    ray_dir: Vec3,
    surface: &CylindricalSurface,
    v_range: (f64, f64),
    hole_bands: &[(f64, f64)],
    u_gap: Option<(f64, f64)>,
    tol: Tolerance,
) -> (i32, bool) {
    let near = 10.0 * tol.linear;
    let (v_min, v_max) = v_range;
    let axis = surface.axis();
    let m = origin - surface.origin();
    let d_perp = ray_dir - axis * ray_dir.dot(axis);
    let m_perp = m - axis * m.dot(axis);

    let a = d_perp.dot(d_perp);
    if a < 1e-14 {
        return (0, false);
    }
    let b = 2.0 * m_perp.dot(d_perp);
    let c = surface
        .radius()
        .mul_add(-surface.radius(), m_perp.dot(m_perp));
    let disc = b.mul_add(b, -4.0 * a * c);
    // Treat near-tangent rays as misses: counting one graze flips parity.
    if disc < 1e-12 * a * surface.radius() * surface.radius() {
        return (0, false);
    }
    let sqrt_disc = disc.sqrt();
    let mut crossings = 0;
    let mut suspicious = false;
    let near_angle = near / surface.radius().max(near);
    for t in [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
        if t <= tol.linear {
            continue;
        }
        let hit = Point3::new(
            origin.x() + ray_dir.x() * t,
            origin.y() + ray_dir.y() * t,
            origin.z() + ray_dir.z() * t,
        );
        let v = axis.dot(hit - surface.origin());
        suspicious |= (v - v_min).abs() <= near || (v - v_max).abs() <= near;
        if v < v_min - tol.linear || v > v_max + tol.linear {
            continue;
        }
        suspicious |= hole_bands
            .iter()
            .any(|&(lo, hi)| (v - lo).abs() <= near || (v - hi).abs() <= near);
        if hole_bands
            .iter()
            .any(|&(lo, hi)| v > lo + tol.linear && v < hi - tol.linear)
        {
            continue;
        }
        if let Some(gap) = u_gap {
            let (u, _) = surface.project_point(hit);
            suspicious |= near_gap_border(u, gap, near_angle);
            if u_in_gap(u, gap) {
                continue;
            }
        }
        crossings += 1;
    }
    (crossings, suspicious)
}

/// The old root computation of `ray_cone_crossings`, verbatim.
fn reference_cone_roots(
    origin: Point3,
    ray_dir: Vec3,
    surface: &ConicalSurface,
    r_max: f64,
) -> Vec<f64> {
    let axis = surface.axis();
    let m = origin - surface.apex();
    let sin_a = surface.half_angle().sin();
    let sin2 = sin_a * sin_a;
    let d_a = ray_dir.dot(axis);
    let m_a = m.dot(axis);
    let a = sin2.mul_add(-ray_dir.dot(ray_dir), d_a * d_a);
    let half_b = sin2.mul_add(-ray_dir.dot(m), d_a * m_a);
    let c = sin2.mul_add(-m.dot(m), m_a * m_a);
    if a.abs() < 1e-14 {
        if half_b.abs() < 1e-14 {
            return Vec::new();
        }
        return vec![-c / (2.0 * half_b)];
    }
    let disc = half_b.mul_add(half_b, -(a * c));
    if disc < 1e-12 * a.abs() * r_max * r_max {
        return Vec::new();
    }
    let sqrt_disc = disc.sqrt();
    vec![(-half_b - sqrt_disc) / a, (-half_b + sqrt_disc) / a]
}

/// The old `ray_cone_crossings`, verbatim.
fn reference_cone_crossings(
    origin: Point3,
    ray_dir: Vec3,
    surface: &ConicalSurface,
    v_range: (f64, f64),
    u_gap: Option<(f64, f64)>,
    tol: Tolerance,
) -> (i32, bool) {
    let near = 10.0 * tol.linear;
    let (v_min, v_max) = v_range;
    let axis = surface.axis();
    let m = origin - surface.apex();
    let sin_a = surface.half_angle().sin();
    let sin2 = sin_a * sin_a;

    let d_a = ray_dir.dot(axis);
    let m_a = m.dot(axis);
    let a = sin2.mul_add(-ray_dir.dot(ray_dir), d_a * d_a);
    let half_b = sin2.mul_add(-ray_dir.dot(m), d_a * m_a);
    let c = sin2.mul_add(-m.dot(m), m_a * m_a);

    let r_max = surface.radius_at(v_min.abs().max(v_max.abs()));
    let mut roots = [None, None];
    if a.abs() < 1e-14 {
        if half_b.abs() < 1e-14 {
            return (0, false);
        }
        roots[0] = Some(-c / (2.0 * half_b));
    } else {
        let disc = half_b.mul_add(half_b, -(a * c));
        if disc < 1e-12 * a.abs() * r_max * r_max {
            return (0, false);
        }
        let sqrt_disc = disc.sqrt();
        roots[0] = Some((-half_b - sqrt_disc) / a);
        roots[1] = Some((-half_b + sqrt_disc) / a);
    }

    let mut crossings = 0;
    let mut suspicious = false;
    for t in roots.into_iter().flatten() {
        if t <= tol.linear {
            continue;
        }
        let hit = Point3::new(
            origin.x() + ray_dir.x() * t,
            origin.y() + ray_dir.y() * t,
            origin.z() + ray_dir.z() * t,
        );
        let (u, v) = surface.project_point(hit);
        suspicious |= (v - v_min).abs() <= near || (v - v_max).abs() <= near;
        if v < v_min - tol.linear || v > v_max + tol.linear {
            continue;
        }
        if let Some(gap) = u_gap {
            let near_angle = near / surface.radius_at(v).max(near);
            suspicious |= near_gap_border(u, gap, near_angle);
            if u_in_gap(u, gap) {
                continue;
            }
        }
        crossings += 1;
    }
    (crossings, suspicious)
}

/// Roots as sorted bit patterns, so `-0.0`, `0.0` and NaN payloads differ.
fn bits(roots: Vec<f64>) -> Vec<u64> {
    let mut out: Vec<u64> = roots.into_iter().map(f64::to_bits).collect();
    out.sort_unstable();
    out
}

fn hit_bits(hits: &remus_math::ray::RayHits) -> Vec<u64> {
    assert!(hits.iter().all(|h| h.kind == ContactKind::Transversal));
    bits(hits.iter().map(|h| h.t).collect())
}

/// A unit vector from three uniform draws in `[-1, 1)`, or `None` when they
/// are too short to normalize.
fn unit(x: f64, y: f64, z: f64) -> Option<Vec3> {
    Vec3::new(x, y, z).normalize().ok()
}

/// Any unit vector perpendicular to `axis`, turned by `angle` about it.
fn perpendicular(axis: Vec3, angle: f64) -> Vec3 {
    let helper = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let e1 = axis.cross(helper).normalize().unwrap();
    let e2 = axis.cross(e1);
    e1 * angle.cos() + e2 * angle.sin()
}

/// Ray families: 0 random, 1 exactly tangent, 2 nearly tangent,
/// 3 axis/generator-parallel, 4 through the apex or on the surface,
/// 5 nearly axis/generator-parallel.
const FAMILIES: u8 = 6;

fn unit_triple() -> impl Strategy<Value = (f64, f64, f64)> {
    (-1.0..1.0f64, -1.0..1.0f64, -1.0..1.0f64)
}

/// The scale exponent's range: 1e-3 to 1e6.
fn scale_exponent() -> impl Strategy<Value = f64> {
    -3.0..6.0f64
}

#[allow(clippy::many_single_char_names)]
fn cylinder_case(
    scale: f64,
    family: u8,
    axis: Vec3,
    centre: (f64, f64, f64),
    dir: (f64, f64, f64),
    s: (f64, f64, f64),
) -> Option<(CylindricalSurface, Point3, Vec3)> {
    let radius = scale * (0.05 + s.0.abs());
    let origin = Point3::new(centre.0 * scale, centre.1 * scale, centre.2 * scale);
    let cyl = CylindricalSurface::new(origin, axis, radius).ok()?;
    let random_dir = unit(dir.0, dir.1, dir.2)?;
    let along = s.1 * 4.0 * scale;
    let (o, d) = match family {
        0 => (
            Point3::new(
                origin.x() + 3.0 * radius * dir.1,
                origin.y() + 3.0 * radius * dir.2,
                origin.z() + 3.0 * radius * dir.0,
            ),
            random_dir,
        ),
        1 | 2 => {
            // A line at distance `radius · (1 + offset)` from the axis,
            // running across it.
            let n = perpendicular(axis, s.2 * TAU);
            let across = axis.cross(n);
            let slope = dir.0;
            let d = (across + axis * slope).normalize().ok()?;
            let offset = if family == 1 { 0.0 } else { s.2 * 1e-6 };
            let foot = origin + n * (radius * (1.0 + offset)) + axis * along;
            (foot - d * (scale * (1.0 + dir.1.abs())), d)
        }
        3 | 5 => {
            let tilt = if family == 3 { 0.0 } else { s.2 * 1e-6 };
            let d = (axis + perpendicular(axis, dir.0 * TAU) * tilt)
                .normalize()
                .ok()?;
            let r = radius * (1.0 + dir.1);
            (
                origin + perpendicular(axis, dir.2 * TAU) * r + axis * along,
                d,
            )
        }
        _ => {
            let n = perpendicular(axis, s.2 * TAU);
            (origin + n * radius + axis * along, random_dir)
        }
    };
    Some((cyl, o, d))
}

#[allow(clippy::many_single_char_names)]
fn cone_case(
    scale: f64,
    family: u8,
    axis: Vec3,
    centre: (f64, f64, f64),
    dir: (f64, f64, f64),
    s: (f64, f64, f64),
) -> Option<(ConicalSurface, Point3, Vec3)> {
    let half_angle = 0.05 + 1.35 * s.0.abs();
    let apex = Point3::new(centre.0 * scale, centre.1 * scale, centre.2 * scale);
    let cone = ConicalSurface::new(apex, axis, half_angle).ok()?;
    let random_dir = unit(dir.0, dir.1, dir.2)?;
    let (sn, cs) = half_angle.sin_cos();
    let radial = perpendicular(axis, s.2 * TAU);
    // `half_angle` is measured from the radial plane: `|h| = sin a · |w|`.
    let generator = axis * sn + radial * cs;
    let slant = scale * (0.1 + 2.0 * s.1.abs());
    let on_surface = apex + generator * slant;
    let (o, d) = match family {
        0 => (
            Point3::new(
                apex.x() + scale * dir.1,
                apex.y() + scale * dir.2,
                apex.z() + scale * dir.0,
            ),
            random_dir,
        ),
        1 | 2 => {
            // A line in the tangent plane along the generator, across it.
            let normal = (radial * sn - axis * cs).normalize().ok()?;
            let across = normal.cross(generator);
            let d = (across + generator * dir.0 * 0.5).normalize().ok()?;
            let offset = if family == 1 { 0.0 } else { s.2 * 1e-6 * scale };
            let foot = on_surface + normal * offset;
            (foot - d * (scale * (1.0 + dir.1.abs())), d)
        }
        3 | 5 => {
            let tilt = if family == 3 { 0.0 } else { s.2 * 1e-6 };
            let d = (generator + radial.cross(axis) * tilt).normalize().ok()?;
            (
                on_surface - generator * (slant * dir.1.abs()) + radial * (scale * dir.2),
                d,
            )
        }
        _ => {
            if dir.2 > 0.0 {
                (
                    apex - random_dir * (scale * (1.0 + dir.1.abs())),
                    random_dir,
                )
            } else {
                (on_surface, random_dir)
            }
        }
    };
    Some((cone, o, d))
}

fn ranges(scale: f64, s: (f64, f64, f64)) -> ((f64, f64), Option<(f64, f64)>) {
    let lo = s.0 * 2.0 * scale;
    let v_range = (lo, lo + scale * (0.2 + 2.0 * s.1.abs()));
    let u_gap = (s.2 > 0.0).then(|| (s.2 * TAU, s.2 * TAU + 1.0 + s.1.abs() * 3.0));
    (v_range, u_gap)
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 4096, ..ProptestConfig::default() })]

    #[test]
    fn cylinder_crossings_match_the_reference(
        exponent in scale_exponent(),
        family in 0..FAMILIES,
        axis in unit_triple(),
        centre in unit_triple(),
        dir in unit_triple(),
        s in unit_triple(),
        trim in unit_triple(),
    ) {
        let scale = 10f64.powf(exponent);
        let Some(axis) = unit(axis.0, axis.1, axis.2) else { return Ok(()) };
        let Some((cyl, o, d)) = cylinder_case(scale, family, axis, centre, dir, s) else {
            return Ok(());
        };
        let tol = Tolerance::default();
        let routed = remus_math::ray::ray_cylinder_with(
            o,
            d,
            &cyl,
            TangentPolicy::DropRelative { scale: cyl.radius() },
        );
        prop_assert_eq!(hit_bits(&routed), bits(reference_cylinder_roots(o, d, &cyl)));
        let (v_range, u_gap) = ranges(scale, trim);
        let band = (v_range.0 + 0.25 * (v_range.1 - v_range.0), v_range.0 + 0.5 * (v_range.1 - v_range.0));
        for holes in [&[][..], &[band][..]] {
            for gap in [None, u_gap] {
                prop_assert_eq!(
                    ray_cylinder_crossings(o, d, &cyl, v_range, holes, gap, tol),
                    reference_cylinder_crossings(o, d, &cyl, v_range, holes, gap, tol)
                );
            }
        }
    }

    #[test]
    fn cone_crossings_match_the_reference(
        exponent in scale_exponent(),
        family in 0..FAMILIES,
        axis in unit_triple(),
        centre in unit_triple(),
        dir in unit_triple(),
        s in unit_triple(),
        trim in unit_triple(),
    ) {
        let scale = 10f64.powf(exponent);
        let Some(axis) = unit(axis.0, axis.1, axis.2) else { return Ok(()) };
        let Some((cone, o, d)) = cone_case(scale, family, axis, centre, dir, s) else {
            return Ok(());
        };
        let tol = Tolerance::default();
        let (v_range, u_gap) = ranges(scale, trim);
        let v_range = (v_range.0.abs(), v_range.0.abs() + (v_range.1 - v_range.0));
        let r_max = cone.radius_at(v_range.0.abs().max(v_range.1.abs()));
        let routed = remus_math::ray::ray_cone_with(
            o,
            d,
            &cone,
            TangentPolicy::DropRelative { scale: r_max },
        );
        prop_assert_eq!(hit_bits(&routed), bits(reference_cone_roots(o, d, &cone, r_max)));
        for gap in [None, u_gap] {
            prop_assert_eq!(
                ray_cone_crossings(o, d, &cone, v_range, gap, tol),
                reference_cone_crossings(o, d, &cone, v_range, gap, tol)
            );
        }
    }

    #[test]
    fn plane_hits_match_the_reference(
        exponent in scale_exponent(),
        o in unit_triple(),
        dir in unit_triple(),
        denom_kind in 0..5u8,
        numer_kind in 0..4u8,
        raw in unit_triple(),
    ) {
        let scale = 10f64.powf(exponent);
        let tol = Tolerance::default();
        let Some(d) = unit(dir.0, dir.1, dir.2) else { return Ok(()) };
        let origin = Point3::new(o.0 * scale, o.1 * scale, o.2 * scale);
        // Denominators at, around and far from the parallel threshold.
        let denom = match denom_kind {
            0 => 0.0,
            1 => tol.angular * (1.0 + raw.0),
            2 => tol.angular,
            3 => -tol.angular * (1.0 + raw.0 * 1e-3),
            _ => raw.0,
        };
        // Numerators in and around the in-plane band and the behind cut-off.
        let near = 10.0 * tol.linear;
        let numer = match numer_kind {
            0 => 0.0,
            1 => near * (1.0 + raw.1),
            2 => tol.linear * denom * (1.0 + raw.1 * 1e-3),
            _ => raw.1 * scale,
        };
        prop_assert_eq!(
            plane_hit(origin, d, denom, numer, tol),
            reference_plane_hit(origin, d, denom, numer, tol)
        );
    }
}

/// Every ray family the property tests draw is actually exercised in a way
/// that matters: tangent rays dropped, axis-parallel rays missing, a cone
/// generator ray giving its single linear root.
#[test]
fn constructed_families_hit_the_special_branches() {
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let policy = |scale| TangentPolicy::DropRelative { scale };
    // Exact tangent: the old rule drops it, so both agree on no roots.
    let (cyl, o, d) = cylinder_case(
        1.0,
        1,
        axis,
        (0.1, 0.2, 0.3),
        (0.3, 0.5, 0.0),
        (0.7, 0.1, 0.2),
    )
    .unwrap();
    assert!(reference_cylinder_roots(o, d, &cyl).is_empty());
    assert!(remus_math::ray::ray_cylinder_with(o, d, &cyl, policy(cyl.radius())).is_empty());
    // Axis-parallel: no roots.
    let (cyl, o, d) = cylinder_case(
        1.0,
        3,
        axis,
        (0.1, 0.2, 0.3),
        (0.3, 0.5, 0.0),
        (0.7, 0.1, 0.2),
    )
    .unwrap();
    assert!(remus_math::ray::ray_cylinder_with(o, d, &cyl, policy(cyl.radius())).is_empty());
    // Along a generator: one linear root.
    let (cone, o, d) = cone_case(
        1.0,
        3,
        axis,
        (0.1, 0.2, 0.3),
        (0.3, 0.5, 0.4),
        (0.4, 0.5, 0.2),
    )
    .unwrap();
    let roots = reference_cone_roots(o, d, &cone, 2.0);
    let routed = remus_math::ray::ray_cone_with(o, d, &cone, policy(2.0));
    assert_eq!(roots.len(), 1);
    assert_eq!(hit_bits(&routed), bits(roots));
}
