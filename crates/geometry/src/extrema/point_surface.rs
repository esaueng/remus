//! Point-to-surface projection algorithms.
//!
//! Provides analytic fast paths for all standard analytic surface types
//! ([`CylindricalSurface`], [`ConicalSurface`], [`SphericalSurface`],
//! [`ToroidalSurface`]) and a generic Newton-Raphson solver for any
//! [`ParametricSurface`].

use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point3, Vec3};

use super::SurfaceProjection;

/// Normalize an angle from `atan2` range `(−π, π]` to `[0, 2π)`.
#[inline]
fn normalize_angle(angle: f64) -> f64 {
    if angle < 0.0 {
        angle + std::f64::consts::TAU
    } else {
        angle
    }
}

// ── Analytic fast paths ──────────────────────────────────────────────────────

/// Project a point onto an infinite plane.
///
/// The plane is defined by an `origin` point and a unit `normal` vector.
/// The u and v parameters are measured in the plane's local orthonormal frame
/// derived from the normal via the stable cross-product method.
///
/// Returns a [`SurfaceProjection`] with the perpendicular foot and the signed
/// distance (`distance` is always non-negative).
#[must_use]
pub fn point_to_plane(point: Point3, origin: Point3, normal: Vec3) -> SurfaceProjection {
    let d = (point - origin).dot(normal);
    let closest = point - normal * d;

    // Derive stable orthonormal u/v axes from normal.
    // Pick a candidate that is not parallel to `normal`, then normalize.
    let candidate = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u_raw = normal.cross(candidate);
    let u_len = u_raw.length();
    if u_len < 1e-15 {
        // Degenerate (zero or near-zero) normal — UV is meaningless.
        return SurfaceProjection {
            distance: d.abs(),
            point: closest,
            u: 0.0,
            v: 0.0,
        };
    }
    let u_axis = u_raw * (1.0 / u_len);
    let v_raw = normal.cross(u_axis);
    let v_axis = v_raw * (1.0 / v_raw.length());

    let delta = closest - origin;
    SurfaceProjection {
        distance: d.abs(),
        point: closest,
        u: delta.dot(u_axis),
        v: delta.dot(v_axis),
    }
}

/// Project a point onto an infinite cylinder.
///
/// Projects the point onto the cylinder's axis to find the height parameter
/// `v`, then computes the radial closest point at that height. When the
/// point lies on the axis (degenerate case) the u-parameter is 0.
#[must_use]
pub fn point_to_cylinder(point: Point3, cyl: &CylindricalSurface) -> SurfaceProjection {
    let pv = Vec3::new(
        point.x() - cyl.origin().x(),
        point.y() - cyl.origin().y(),
        point.z() - cyl.origin().z(),
    );
    // Height along axis (v parameter).
    let h = pv.dot(cyl.axis());
    // Radial vector: pv - h * axis.
    let radial = Vec3::new(
        pv.x() - h * cyl.axis().x(),
        pv.y() - h * cyl.axis().y(),
        pv.z() - h * cyl.axis().z(),
    );
    let r_len = radial.length();

    if r_len < 1e-15 {
        // Point is on the axis — pick u = 0 arbitrarily.
        let closest = cyl.evaluate(0.0, h);
        SurfaceProjection {
            distance: (point - closest).length(),
            point: closest,
            u: 0.0,
            v: h,
        }
    } else {
        // u = atan2(radial · y_axis, radial · x_axis), normalized to [0, 2π).
        let u = normalize_angle((radial.dot(cyl.y_axis())).atan2(radial.dot(cyl.x_axis())));
        let scale = cyl.radius() / r_len;
        let closest = Point3::new(
            cyl.origin().x() + radial.x() * scale + h * cyl.axis().x(),
            cyl.origin().y() + radial.y() * scale + h * cyl.axis().y(),
            cyl.origin().z() + radial.z() * scale + h * cyl.axis().z(),
        );
        SurfaceProjection {
            distance: (point - closest).length(),
            point: closest,
            u,
            v: h,
        }
    }
}

/// Project a point onto a cone.
///
/// Handles the apex case, points interior to the cone that project to the apex,
/// and the general case where the point projects onto a generator line.
#[must_use]
pub fn point_to_cone(point: Point3, cone: &ConicalSurface) -> SurfaceProjection {
    let pv = Vec3::new(
        point.x() - cone.apex().x(),
        point.y() - cone.apex().y(),
        point.z() - cone.apex().z(),
    );
    let h = pv.dot(cone.axis());

    // Radial component perpendicular to axis.
    let radial = Vec3::new(
        pv.x() - h * cone.axis().x(),
        pv.y() - h * cone.axis().y(),
        pv.z() - h * cone.axis().z(),
    );
    let r_len = radial.length();

    if h <= 0.0 && r_len < 1e-15 {
        // Very close to apex — return apex with u=0, v=0.
        return SurfaceProjection {
            distance: (point - cone.apex()).length(),
            point: cone.apex(),
            u: 0.0,
            v: 0.0,
        };
    }

    // Project onto the cone's generatrix direction.
    let (sin_a, cos_a) = cone.half_angle().sin_cos();
    let v = h.mul_add(sin_a, r_len * cos_a);

    if v <= 0.0 {
        // Closest point is the apex.
        return SurfaceProjection {
            distance: (point - cone.apex()).length(),
            point: cone.apex(),
            u: 0.0,
            v: 0.0,
        };
    }

    // Cone surface point at parameter v along the generatrix.
    let cone_r = v * cos_a;
    let cone_h = v * sin_a;

    let (closest, u) = if r_len < 1e-15 {
        let closest = Point3::new(
            cone.apex().x() + cone_h * cone.axis().x(),
            cone.apex().y() + cone_h * cone.axis().y(),
            cone.apex().z() + cone_h * cone.axis().z(),
        );
        (closest, 0.0_f64)
    } else {
        let radial_dir_x = radial.x() / r_len;
        let radial_dir_y = radial.y() / r_len;
        let radial_dir_z = radial.z() / r_len;
        let closest = Point3::new(
            cone.apex().x() + cone_h * cone.axis().x() + cone_r * radial_dir_x,
            cone.apex().y() + cone_h * cone.axis().y() + cone_r * radial_dir_y,
            cone.apex().z() + cone_h * cone.axis().z() + cone_r * radial_dir_z,
        );
        let radial_vec = Vec3::new(radial_dir_x, radial_dir_y, radial_dir_z);
        let u = normalize_angle(
            radial_vec
                .dot(cone.y_axis())
                .atan2(radial_vec.dot(cone.x_axis())),
        );
        (closest, u)
    };

    SurfaceProjection {
        distance: (point - closest).length(),
        point: closest,
        u,
        v,
    }
}

/// Project a point onto a sphere.
///
/// Projects radially from the sphere center. When the point is at the center,
/// an arbitrary surface point is returned.
#[must_use]
pub fn point_to_sphere(point: Point3, sphere: &SphericalSurface) -> SurfaceProjection {
    let pv = Vec3::new(
        point.x() - sphere.center().x(),
        point.y() - sphere.center().y(),
        point.z() - sphere.center().z(),
    );
    let dist_to_center = pv.length();

    if dist_to_center < 1e-15 {
        // Point at center — arbitrary direction, u=0, v=0.
        let closest = Point3::new(
            sphere.center().x() + sphere.radius(),
            sphere.center().y(),
            sphere.center().z(),
        );
        return SurfaceProjection {
            distance: sphere.radius(),
            point: closest,
            u: 0.0,
            v: 0.0,
        };
    }

    let scale = sphere.radius() / dist_to_center;
    let closest = Point3::new(
        sphere.center().x() + pv.x() * scale,
        sphere.center().y() + pv.y() * scale,
        sphere.center().z() + pv.z() * scale,
    );

    // u = azimuth angle, v = elevation angle.
    let (u, v) = sphere.project_point(point);

    SurfaceProjection {
        distance: (dist_to_center - sphere.radius()).abs(),
        point: closest,
        u,
        v,
    }
}

/// Project a point onto a torus.
///
/// Projects onto the major circle first, then onto the minor circle tube.
#[must_use]
pub fn point_to_torus(point: Point3, torus: &ToroidalSurface) -> SurfaceProjection {
    let pv = Vec3::new(
        point.x() - torus.center().x(),
        point.y() - torus.center().y(),
        point.z() - torus.center().z(),
    );

    let z_axis = torus.z_axis();
    let h = pv.dot(z_axis);

    // Radial projection in the equatorial plane.
    let radial = Vec3::new(
        pv.x() - h * z_axis.x(),
        pv.y() - h * z_axis.y(),
        pv.z() - h * z_axis.z(),
    );
    let r_len = radial.length();

    let major_r = torus.major_radius();
    let minor_r = torus.minor_radius();

    // Closest point on major circle.
    let (major_cx, major_cy, major_cz, u) = if r_len < 1e-15 {
        // On the axis — pick u = 0.
        (
            torus.center().x() + major_r * torus.x_axis().x(),
            torus.center().y() + major_r * torus.x_axis().y(),
            torus.center().z() + major_r * torus.x_axis().z(),
            0.0_f64,
        )
    } else {
        let scale = major_r / r_len;
        let u = normalize_angle(radial.dot(torus.y_axis()).atan2(radial.dot(torus.x_axis())));
        (
            torus.center().x() + radial.x() * scale,
            torus.center().y() + radial.y() * scale,
            torus.center().z() + radial.z() * scale,
            u,
        )
    };

    // Vector from major circle point to query point.
    let tube_vec = Vec3::new(
        point.x() - major_cx,
        point.y() - major_cy,
        point.z() - major_cz,
    );
    let tube_dist = tube_vec.length();

    if tube_dist < 1e-15 {
        // Point is on the major circle — closest point is minor_r away.
        let dir = if r_len < 1e-15 {
            torus.x_axis()
        } else {
            Vec3::new(radial.x() / r_len, radial.y() / r_len, radial.z() / r_len)
        };
        let closest = Point3::new(
            major_cx + minor_r * dir.x(),
            major_cy + minor_r * dir.y(),
            major_cz + minor_r * dir.z(),
        );
        return SurfaceProjection {
            distance: minor_r,
            point: closest,
            u,
            v: 0.0,
        };
    }

    let tube_scale = minor_r / tube_dist;
    let closest = Point3::new(
        major_cx + tube_vec.x() * tube_scale,
        major_cy + tube_vec.y() * tube_scale,
        major_cz + tube_vec.z() * tube_scale,
    );

    // v = angle of tube vector relative to radial/z directions.
    // v = atan2(tube component along z_axis, tube component along radial dir)
    let v = normalize_angle(if r_len < 1e-15 {
        tube_vec.dot(z_axis).atan2(tube_vec.dot(torus.x_axis()))
    } else {
        let radial_dir = Vec3::new(radial.x() / r_len, radial.y() / r_len, radial.z() / r_len);
        tube_vec.dot(z_axis).atan2(tube_vec.dot(radial_dir))
    });

    SurfaceProjection {
        distance: (tube_dist - minor_r).abs(),
        point: closest,
        u,
        v,
    }
}

// ── NURBS surface grid-search projection ────────────────────────────────────

/// Project a point onto a NURBS surface using a grid search.
///
/// Samples an 11x11 grid in parameter space and returns the closest point.
/// This is a lightweight alternative to the full Newton-Raphson
/// [`point_to_surface`] solver — suitable when only approximate UV coordinates
/// are needed (e.g. PCurve seeding).
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn point_to_nurbs_surface(
    point: Point3,
    surface: &remus_math::nurbs::surface::NurbsSurface,
) -> SurfaceProjection {
    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();
    let n = 10;
    let mut best_u = u_min;
    let mut best_v = v_min;
    let mut best_dist_sq = f64::MAX;

    for i in 0..=n {
        for j in 0..=n {
            let u = u_min + (u_max - u_min) * (i as f64) / (n as f64);
            let v = v_min + (v_max - v_min) * (j as f64) / (n as f64);
            let pt = surface.evaluate(u, v);
            let dx = pt.x() - point.x();
            let dy = pt.y() - point.y();
            let dz = pt.z() - point.z();
            let dist_sq = dx.mul_add(dx, dy.mul_add(dy, dz * dz));
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_u = u;
                best_v = v;
            }
        }
    }

    let closest = surface.evaluate(best_u, best_v);
    SurfaceProjection {
        distance: best_dist_sq.sqrt(),
        point: closest,
        u: best_u,
        v: best_v,
    }
}

// ── Generic Newton-Raphson solver ────────────────────────────────────────────

/// Maximum Newton iterations.
const MAX_ITER: usize = 50;

/// Convergence tolerance on the parameter update magnitude.
const PARAM_TOL: f64 = 1e-10;

/// Grid size for initial sampling phase.
const GRID_N: usize = 8;

/// Project a point onto any [`ParametricSurface`] over `[u_range] × [v_range]`.
///
/// **Algorithm:**
/// 1. Sample the surface on a `GRID_N`×`GRID_N` grid to find the global
///    closest sample (avoids local-minimum traps on non-convex surfaces).
/// 2. Refine the best sample using Newton-Raphson on the two-variable
///    stationarity conditions:
///    - `dot(S(u,v) - P, ∂S/∂u) = 0`
///    - `dot(S(u,v) - P, ∂S/∂v) = 0`
///
/// Parameters are clamped to their ranges after each Newton step.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn point_to_surface<S: ParametricSurface>(
    point: Point3,
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
) -> SurfaceProjection {
    let (u0, u1) = u_range;
    let (v0, v1) = v_range;

    // ── Phase 1: grid search ──────────────────────────────────────────────────
    let mut best_u = (u0 + u1) * 0.5;
    let mut best_v = (v0 + v1) * 0.5;
    let mut best_dist_sq = f64::INFINITY;

    for iu in 0..GRID_N {
        let u = u0 + (u1 - u0) * (iu as f64) / ((GRID_N - 1) as f64);
        for iv in 0..GRID_N {
            let v = v0 + (v1 - v0) * (iv as f64) / ((GRID_N - 1) as f64);
            let p = surface.evaluate(u, v);
            let diff = p - point;
            let d2 = diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z();
            if d2 < best_dist_sq {
                best_dist_sq = d2;
                best_u = u;
                best_v = v;
            }
        }
    }

    // ── Phase 2: Newton-Raphson ───────────────────────────────────────────────
    // Solve the 2×2 system:
    //   f1(u,v) = dot(S(u,v) - P, Su) = 0
    //   f2(u,v) = dot(S(u,v) - P, Sv) = 0
    //
    // Using Gauss-Newton: J^T J Δx = -J^T f  where J_ij = ∂fi/∂xj
    //   J11 ≈ |Su|^2,  J12 = Su·Sv
    //   J21 = Su·Sv,   J22 ≈ |Sv|^2
    let mut u = best_u;
    let mut v = best_v;

    for _ in 0..MAX_ITER {
        let p = surface.evaluate(u, v);
        let diff = Vec3::new(p.x() - point.x(), p.y() - point.y(), p.z() - point.z());
        let su = surface.partial_u(u, v);
        let sv = surface.partial_v(u, v);

        let f1 = diff.dot(su);
        let f2 = diff.dot(sv);

        let j11 = su.dot(su);
        let j12 = su.dot(sv);
        let j22 = sv.dot(sv);
        let det = j11 * j22 - j12 * j12;

        if det.abs() < f64::EPSILON {
            break;
        }

        let du = (f1 * j22 - f2 * j12) / det;
        let dv = (f2 * j11 - f1 * j12) / det;

        let u_new = (u - du).clamp(u0, u1);
        let v_new = (v - dv).clamp(v0, v1);

        if (u_new - u).abs() < PARAM_TOL && (v_new - v).abs() < PARAM_TOL {
            u = u_new;
            v = v_new;
            break;
        }
        u = u_new;
        v = v_new;
    }

    let closest = surface.evaluate(u, v);
    let diff = closest - point;
    let distance = (diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z()).sqrt();

    SurfaceProjection {
        distance,
        point: closest,
        u,
        v,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::items_after_statements
    )]

    use std::f64::consts::{FRAC_PI_2, TAU};

    use super::*;
    use remus_math::vec::Vec3;

    // Helpers
    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── point_to_plane ───────────────────────────────────────────────────────

    #[test]
    fn plane_perpendicular_projection() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let point = Point3::new(3.0, 4.0, 5.0);
        let proj = point_to_plane(point, origin, normal);
        assert!(approx(proj.distance, 5.0, 1e-12), "dist={}", proj.distance);
        assert!(approx(proj.point.x(), 3.0, 1e-12));
        assert!(approx(proj.point.y(), 4.0, 1e-12));
        assert!(approx(proj.point.z(), 0.0, 1e-12));
        // For normal=(0,0,1), u_axis=(0,1,0), v_axis=(-1,0,0) via cross-product method.
        // closest=(3,4,0), delta=(3,4,0), u=delta·u_axis=4, v=delta·v_axis=-3.
        assert!(approx(proj.u, 4.0, 1e-12), "u={}", proj.u);
        assert!(approx(proj.v, -3.0, 1e-12), "v={}", proj.v);
    }

    #[test]
    fn plane_point_on_plane_zero_distance() {
        let origin = Point3::new(1.0, 2.0, 3.0);
        let normal = Vec3::new(0.0, 1.0, 0.0);
        // Point on the plane (y=2).
        let point = Point3::new(5.0, 2.0, 7.0);
        let proj = point_to_plane(point, origin, normal);
        assert!(proj.distance < 1e-12, "dist={}", proj.distance);
        // For normal=(0,1,0), u_axis=(0,0,-1), v_axis=(-1,0,0) via cross-product method.
        // closest=(5,2,7), delta=(4,0,4), u=delta·u_axis=-4, v=delta·v_axis=-4.
        assert!(approx(proj.u, -4.0, 1e-12), "u={}", proj.u);
        assert!(approx(proj.v, -4.0, 1e-12), "v={}", proj.v);
    }

    #[test]
    fn plane_x_dominant_normal() {
        // Exercises the `else` branch of the candidate selection (normal.x >= 0.9).
        let origin = Point3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(1.0, 0.0, 0.0);
        let point = Point3::new(7.0, 3.0, 4.0);
        let proj = point_to_plane(point, origin, normal);
        assert!(approx(proj.distance, 7.0, 1e-12), "dist={}", proj.distance);
        assert!(approx(proj.point.x(), 0.0, 1e-12));
        assert!(approx(proj.point.y(), 3.0, 1e-12));
        assert!(approx(proj.point.z(), 4.0, 1e-12));
        // For normal=(1,0,0), candidate=(0,1,0):
        // u_axis = normalize((1,0,0)×(0,1,0)) = (0,0,1)
        // v_axis = (1,0,0)×(0,0,1) = (0,-1,0)
        // delta=(0,3,4), u=4, v=-3
        assert!(approx(proj.u, 4.0, 1e-12), "u={}", proj.u);
        assert!(approx(proj.v, -3.0, 1e-12), "v={}", proj.v);
    }

    // ── point_to_sphere ──────────────────────────────────────────────────────

    #[test]
    fn sphere_point_outside() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 2.0).unwrap();
        let point = Point3::new(0.0, 0.0, 5.0);
        let proj = point_to_sphere(point, &sphere);
        // Expected: distance = 5 - 2 = 3, closest = (0,0,2).
        assert!(approx(proj.distance, 3.0, 1e-12), "dist={}", proj.distance);
        assert!(approx(proj.point.z(), 2.0, 1e-12));
        assert!(proj.point.x().abs() < 1e-12);
        assert!(proj.point.y().abs() < 1e-12);
    }

    #[test]
    fn sphere_point_inside() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0).unwrap();
        let point = Point3::new(0.0, 3.0, 0.0);
        let proj = point_to_sphere(point, &sphere);
        // Distance from center = 3; distance to surface = 5 - 3 = 2.
        assert!(approx(proj.distance, 2.0, 1e-12), "dist={}", proj.distance);
        // Closest must be on the sphere.
        let r = (proj.point.x() * proj.point.x()
            + proj.point.y() * proj.point.y()
            + proj.point.z() * proj.point.z())
        .sqrt();
        assert!(approx(r, 5.0, 1e-12), "not on sphere: r={r}");
    }

    #[test]
    fn sphere_point_at_center_returns_surface_point() {
        let sphere = SphericalSurface::new(Point3::new(1.0, 2.0, 3.0), 4.0).unwrap();
        let proj = point_to_sphere(sphere.center(), &sphere);
        assert!(approx(proj.distance, 4.0, 1e-12), "dist={}", proj.distance);
    }

    // ── point_to_cylinder ────────────────────────────────────────────────────

    #[test]
    fn cylinder_point_perpendicular_to_axis() {
        // Cylinder along Z-axis, radius 3; point at (5,0,2) (perpendicular to axis at h=2).
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 3.0)
                .unwrap();
        let point = Point3::new(5.0, 0.0, 2.0);
        let proj = point_to_cylinder(point, &cyl);
        // Distance from axis = 5, so distance to cylinder = 5 - 3 = 2.
        assert!(approx(proj.distance, 2.0, 1e-12), "dist={}", proj.distance);
        // Closest must be on the cylinder surface.
        let ox = proj.point.x();
        let oy = proj.point.y();
        let r = (ox * ox + oy * oy).sqrt();
        assert!(approx(r, 3.0, 1e-12), "not on cylinder: r={r}");
        // Height preserved.
        assert!(approx(proj.point.z(), 2.0, 1e-12), "z={}", proj.point.z());
    }

    #[test]
    fn cylinder_point_on_axis_uses_u_zero() {
        // Point on the axis — should pick u=0 and distance = radius.
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0)
                .unwrap();
        let point = Point3::new(0.0, 0.0, 4.0);
        let proj = point_to_cylinder(point, &cyl);
        assert!(approx(proj.distance, 2.0, 1e-12), "dist={}", proj.distance);
        assert!(approx(proj.u, 0.0, 1e-12), "u={}", proj.u);
    }

    // ── point_to_torus ───────────────────────────────────────────────────────

    #[test]
    fn torus_point_on_major_circle_returns_minor_radius() {
        // Torus with major=3, minor=1; point on the major circle at (3,0,0).
        let torus = ToroidalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            3.0, // major
            1.0, // minor
        )
        .unwrap();
        let point = Point3::new(3.0, 0.0, 0.0);
        let proj = point_to_torus(point, &torus);
        assert!(approx(proj.distance, 1.0, 1e-12), "dist={}", proj.distance);
    }

    #[test]
    fn torus_point_on_surface_zero_distance() {
        // Point on the torus surface: major=3, minor=1.
        // Surface point at u=0, v=0: (major+minor, 0, 0) = (4,0,0).
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        let point = torus.evaluate(0.0, 0.0);
        let proj = point_to_torus(point, &torus);
        assert!(proj.distance < 1e-10, "dist={}", proj.distance);
    }

    // ── point_to_surface (generic) ───────────────────────────────────────────

    #[test]
    fn generic_cylinder_matches_analytic() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 3.0)
                .unwrap();
        let point = Point3::new(5.0, 0.0, 2.0);

        let analytic = point_to_cylinder(point, &cyl);
        let generic = point_to_surface(point, &cyl, (0.0, TAU), (-10.0, 10.0));

        assert!(
            approx(analytic.distance, generic.distance, 1e-4),
            "analytic={} generic={}",
            analytic.distance,
            generic.distance
        );
    }

    #[test]
    fn generic_sphere_matches_analytic() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 2.0).unwrap();
        let point = Point3::new(0.0, 0.0, 5.0);

        let analytic = point_to_sphere(point, &sphere);
        let generic = point_to_surface(point, &sphere, (0.0, TAU), (-FRAC_PI_2, FRAC_PI_2));

        assert!(
            approx(analytic.distance, generic.distance, 1e-4),
            "analytic={} generic={}",
            analytic.distance,
            generic.distance
        );
    }

    // ── Contract helpers ─────────────────────────────────────────────────────

    fn pt_approx(a: Point3, b: Point3, tol: f64) -> bool {
        approx(a.x(), b.x(), tol) && approx(a.y(), b.y(), tol) && approx(a.z(), b.z(), tol)
    }

    /// Structural contract shared by every analytic fast path:
    ///
    /// * the result is finite,
    /// * `u` lies in the documented `[0, 2π)` range,
    /// * `distance` is the distance from the query point to `point`,
    /// * `point` is the surface evaluated at the returned `(u, v)`.
    fn assert_projection_contract(
        query: Point3,
        proj: &SurfaceProjection,
        eval: impl Fn(f64, f64) -> Point3,
        tol: f64,
    ) {
        assert!(
            proj.distance.is_finite() && proj.u.is_finite() && proj.v.is_finite(),
            "non-finite projection: d={} u={} v={}",
            proj.distance,
            proj.u,
            proj.v
        );
        assert!(
            proj.point.x().is_finite() && proj.point.y().is_finite() && proj.point.z().is_finite(),
            "non-finite closest point"
        );
        assert!(
            proj.u >= 0.0 && proj.u < TAU,
            "u outside [0, 2pi): {}",
            proj.u
        );
        let measured = (query - proj.point).length();
        assert!(
            approx(proj.distance, measured, tol),
            "distance {} != |P - closest| {}",
            proj.distance,
            measured
        );
        let s = eval(proj.u, proj.v);
        assert!(
            pt_approx(s, proj.point, tol),
            "closest ({}, {}, {}) != S(u, v) ({}, {}, {})",
            proj.point.x(),
            proj.point.y(),
            proj.point.z(),
            s.x(),
            s.y(),
            s.z()
        );
    }

    // Off-origin, non-axis-aligned fixtures with radii that are neither 0 nor 1.
    fn tilted_cylinder() -> CylindricalSurface {
        CylindricalSurface::new(Point3::new(2.5, -1.5, 3.25), Vec3::new(2.0, 3.0, 6.0), 2.75)
            .unwrap()
    }

    fn tilted_cone() -> ConicalSurface {
        ConicalSurface::new(Point3::new(-1.5, 2.25, 0.75), Vec3::new(2.0, 3.0, 6.0), 0.6).unwrap()
    }

    fn tilted_sphere() -> SphericalSurface {
        SphericalSurface::with_axis(Point3::new(2.5, -1.5, 3.25), 3.75, Vec3::new(2.0, 3.0, 6.0))
            .unwrap()
    }

    fn tilted_torus() -> ToroidalSurface {
        ToroidalSurface::with_axis(
            Point3::new(2.5, -1.5, 3.25),
            4.5,
            1.75,
            Vec3::new(2.0, 3.0, 6.0),
        )
        .unwrap()
    }

    /// Closed form: `| |P − C| ⟂ axis − R |` combined with the tube radius.
    /// `|sqrt((ρ − R)² + z²) − r|` in the torus frame.
    fn torus_closed_form(p: Point3, t: &ToroidalSurface) -> f64 {
        let pv = p - t.center();
        let h = pv.dot(t.z_axis());
        let rho = (pv - t.z_axis() * h).length();
        ((rho - t.major_radius()).hypot(h) - t.minor_radius()).abs()
    }

    /// Closed form: distance to the generating line in the meridian half-plane,
    /// or to the apex when the perpendicular foot falls behind it.
    fn cone_closed_form(p: Point3, c: &ConicalSurface) -> f64 {
        let pv = p - c.apex();
        let h = pv.dot(c.axis());
        let rho = (pv - c.axis() * h).length();
        let (sin_a, cos_a) = c.half_angle().sin_cos();
        if h.mul_add(sin_a, rho * cos_a) <= 0.0 {
            pv.length()
        } else {
            rho.mul_add(sin_a, -(h * cos_a)).abs()
        }
    }

    /// Unit radial direction at angle `u` in a local `(x, y)` frame.
    fn radial_at(x: Vec3, y: Vec3, u: f64) -> Vec3 {
        x * u.cos() + y * u.sin()
    }

    // ── point_to_plane ───────────────────────────────────────────────────────

    #[test]
    fn plane_uv_axes_are_normalized_not_scaled() {
        // normal = (0.6, 0, 0.8): |n.x| < 0.9 so candidate = (1,0,0) and
        // u_raw = n × (1,0,0) = (0, 0.8, 0) has length 0.8 ≠ 1, so the
        // normalization by 1/|u_raw| is observable.
        // u_axis = (0,1,0), v_axis = n × u_axis = (-0.8, 0, 0.6).
        let origin = Point3::new(1.5, -2.5, 3.5);
        let normal = Vec3::new(0.6, 0.0, 0.8);
        let point = Point3::new(4.5, 1.5, 6.0);
        let proj = point_to_plane(point, origin, normal);
        // d = (P-o)·n = 3.0*0.6 + 4.0*0.0 + 2.5*0.8 = 3.8
        assert!(approx(proj.distance, 3.8, 1e-12), "dist={}", proj.distance);
        assert!(
            pt_approx(proj.point, Point3::new(2.22, 1.5, 2.96), 1e-12),
            "closest=({}, {}, {})",
            proj.point.x(),
            proj.point.y(),
            proj.point.z()
        );
        // delta = closest - origin = (0.72, 4.0, -0.54)
        assert!(approx(proj.u, 4.0, 1e-12), "u={}", proj.u);
        assert!(approx(proj.v, -0.9, 1e-12), "v={}", proj.v);
    }

    #[test]
    fn plane_normal_x_exactly_at_candidate_threshold() {
        // |n.x| == 0.9 is NOT < 0.9, so the y-candidate branch is taken:
        // u_axis = normalize(n × (0,1,0)) = (-n.z, 0, n.x) (already unit).
        let nz = (1.0_f64 - 0.81).sqrt();
        let normal = Vec3::new(0.9, 0.0, nz);
        let origin = Point3::new(1.0, -2.0, 0.5);
        let point = Point3::new(3.0, 2.5, 4.0);
        let proj = point_to_plane(point, origin, normal);
        let u_axis = Vec3::new(-nz, 0.0, 0.9);
        let v_axis = normal.cross(u_axis);
        let delta = proj.point - origin;
        assert!(approx(proj.u, delta.dot(u_axis), 1e-12), "u={}", proj.u);
        assert!(approx(proj.v, delta.dot(v_axis), 1e-12), "v={}", proj.v);
        // The x-candidate branch would give u_axis = (0,1,0), i.e. u = delta.y.
        assert!(
            !approx(proj.u, delta.y(), 1e-6),
            "fixture does not separate the two candidate branches"
        );
    }

    #[test]
    fn plane_zero_normal_returns_finite_zero_uv() {
        // Documented degenerate guard: a zero normal yields u = v = 0 rather
        // than a division by the zero cross-product length.
        let origin = Point3::new(1.5, -2.5, 3.5);
        let point = Point3::new(4.5, 1.5, 6.0);
        let proj = point_to_plane(point, origin, Vec3::new(0.0, 0.0, 0.0));
        assert!(proj.u.is_finite() && proj.v.is_finite(), "uv not finite");
        assert!(proj.u.abs() < 1e-15 && proj.v.abs() < 1e-15);
        assert!(proj.distance.abs() < 1e-15, "dist={}", proj.distance);
        assert!(pt_approx(proj.point, point, 1e-15));
    }

    // ── point_to_cylinder ────────────────────────────────────────────────────

    #[test]
    fn cylinder_normal_offset_recovers_parameters() {
        let cyl = tilted_cylinder();
        for (u, v, offset) in [(1.1_f64, 2.3_f64, 1.8_f64), (5.0, -4.25, -1.2)] {
            let surf = cyl.evaluate(u, v);
            let query = surf + cyl.normal(u, v) * offset;
            let proj = point_to_cylinder(query, &cyl);
            assert_projection_contract(query, &proj, |a, b| cyl.evaluate(a, b), 1e-9);
            assert!(approx(proj.u, u, 1e-9), "u={} want {u}", proj.u);
            assert!(approx(proj.v, v, 1e-9), "v={} want {v}", proj.v);
            assert!(pt_approx(proj.point, surf, 1e-9));
            // Closed form: |ρ − radius| with ρ measured perpendicular to the axis.
            let pv = query - cyl.origin();
            let rho = (pv - cyl.axis() * pv.dot(cyl.axis())).length();
            let expected = (rho - cyl.radius()).abs();
            assert!(approx(proj.distance, offset.abs(), 1e-9));
            assert!(approx(proj.distance, expected, 1e-9));
        }
    }

    #[test]
    fn cylinder_zero_angle_and_axis_point() {
        // Axis-aligned so the probe arithmetic is exact: x_axis = (0,1,0),
        // y_axis = (-1,0,0) for axis = (0,0,1).
        let cyl =
            CylindricalSurface::new(Point3::new(2.5, -1.5, 3.25), Vec3::new(0.0, 0.0, 1.0), 2.75)
                .unwrap();
        // origin + 3.5*x_axis + 2.0*axis: atan2(0.0, 3.5) is exactly 0.0, so a
        // normalization that wraps 0 to 2π is observable.
        let query = Point3::new(2.5, 2.0, 5.25);
        let proj = point_to_cylinder(query, &cyl);
        assert!(proj.u.abs() < 1e-12, "u={}", proj.u);
        assert!(approx(proj.v, 2.0, 1e-12), "v={}", proj.v);
        assert!(approx(proj.distance, 0.75, 1e-12), "dist={}", proj.distance);
        assert_projection_contract(query, &proj, |a, b| cyl.evaluate(a, b), 1e-12);

        // On the axis: u collapses to 0 and the distance is the radius.
        let on_axis = Point3::new(2.5, -1.5, 7.65);
        let proj = point_to_cylinder(on_axis, &cyl);
        assert!(proj.u.abs() < 1e-12, "u={}", proj.u);
        assert!(approx(proj.v, 4.4, 1e-12), "v={}", proj.v);
        assert!(approx(proj.distance, 2.75, 1e-12), "dist={}", proj.distance);
        assert_projection_contract(on_axis, &proj, |a, b| cyl.evaluate(a, b), 1e-12);
    }

    // ── point_to_cone ────────────────────────────────────────────────────────

    #[test]
    fn cone_normal_offset_recovers_parameters() {
        let cone = tilted_cone();
        for (u, v, offset) in [(2.2_f64, 5.5_f64, 1.6_f64), (5.0, 3.25, -1.2)] {
            let surf = cone.evaluate(u, v);
            let query = surf + cone.normal(u, v) * offset;
            let proj = point_to_cone(query, &cone);
            assert_projection_contract(query, &proj, |a, b| cone.evaluate(a, b), 1e-9);
            assert!(approx(proj.u, u, 1e-9), "u={} want {u}", proj.u);
            assert!(approx(proj.v, v, 1e-9), "v={} want {v}", proj.v);
            assert!(pt_approx(proj.point, surf, 1e-9));
            assert!(approx(proj.distance, offset.abs(), 1e-9));
            assert!(approx(proj.distance, cone_closed_form(query, &cone), 1e-9));
        }
    }

    #[test]
    fn cone_point_behind_apex_projects_to_apex() {
        let cone = tilted_cone();
        // h = -3, ρ = 0.5 ⇒ v = −3 sin a + 0.5 cos a < 0.
        let query = cone.apex() + cone.axis() * -3.0 + cone.x_axis() * 0.5;
        let proj = point_to_cone(query, &cone);
        assert!(
            approx(proj.distance, 3.0_f64.hypot(0.5), 1e-9),
            "dist={}",
            proj.distance
        );
        assert!(pt_approx(proj.point, cone.apex(), 1e-12));
        assert!(proj.u.abs() < 1e-12 && proj.v.abs() < 1e-12);
        assert!(approx(proj.distance, cone_closed_form(query, &cone), 1e-9));
    }

    #[test]
    fn cone_below_apex_plane_but_outside_uses_generator() {
        let cone = tilted_cone();
        let (sin_a, cos_a) = 0.6_f64.sin_cos();
        // h = -2, ρ = 8 ⇒ v = 8 cos a − 2 sin a > 0: the apex is NOT the answer.
        let dir = radial_at(cone.x_axis(), cone.y_axis(), 2.2);
        let query = cone.apex() + cone.axis() * -2.0 + dir * 8.0;
        let proj = point_to_cone(query, &cone);
        assert_projection_contract(query, &proj, |a, b| cone.evaluate(a, b), 1e-9);
        assert!(approx(proj.u, 2.2, 1e-9), "u={}", proj.u);
        let v_expected = 8.0_f64.mul_add(cos_a, -2.0 * sin_a);
        assert!(v_expected > 0.0);
        assert!(approx(proj.v, v_expected, 1e-9), "v={}", proj.v);
        // Perpendicular distance to the generator: |ρ sin a − h cos a|.
        let expected = 8.0_f64.mul_add(sin_a, 2.0 * cos_a);
        assert!(
            approx(proj.distance, expected, 1e-9),
            "dist={}",
            proj.distance
        );
        assert!(approx(proj.distance, cone_closed_form(query, &cone), 1e-9));
    }

    #[test]
    fn cone_interior_axis_point_keeps_generator_parameter() {
        // Axis-aligned so the radial component is exactly zero.
        let cone =
            ConicalSurface::new(Point3::new(-1.5, 2.25, 0.75), Vec3::new(0.0, 0.0, 1.0), 0.6)
                .unwrap();
        let query = Point3::new(-1.5, 2.25, 6.75); // apex + 6·axis
        let proj = point_to_cone(query, &cone);
        assert!(
            proj.distance.is_finite() && proj.u.is_finite() && proj.v.is_finite(),
            "non-finite: d={} u={} v={}",
            proj.distance,
            proj.u,
            proj.v
        );
        assert!(proj.u.abs() < 1e-12, "u={}", proj.u);
        // v is the generator arc-length of the perpendicular foot: h·sin a.
        assert!(approx(proj.v, 6.0 * 0.6_f64.sin(), 1e-12), "v={}", proj.v);
        let measured = (query - proj.point).length();
        assert!(approx(proj.distance, measured, 1e-12));
    }

    // ── point_to_sphere ──────────────────────────────────────────────────────

    #[test]
    fn sphere_normal_offset_recovers_parameters() {
        let sphere = tilted_sphere();
        for (u, v, offset) in [(1.3_f64, 0.4_f64, 2.2_f64), (4.7, -0.85, -1.5)] {
            let surf = sphere.evaluate(u, v);
            let query = surf + sphere.normal(u, v) * offset;
            let proj = point_to_sphere(query, &sphere);
            assert_projection_contract(query, &proj, |a, b| sphere.evaluate(a, b), 1e-9);
            assert!(approx(proj.u, u, 1e-9), "u={} want {u}", proj.u);
            assert!(approx(proj.v, v, 1e-9), "v={} want {v}", proj.v);
            assert!(pt_approx(proj.point, surf, 1e-9));
            assert!(approx(proj.distance, offset.abs(), 1e-9));
            let expected = ((query - sphere.center()).length() - sphere.radius()).abs();
            assert!(approx(proj.distance, expected, 1e-9));
        }
    }

    #[test]
    fn sphere_center_query_returns_a_point_on_the_sphere() {
        let sphere = tilted_sphere();
        let proj = point_to_sphere(sphere.center(), &sphere);
        assert!(
            proj.point.x().is_finite() && proj.point.y().is_finite() && proj.point.z().is_finite(),
            "non-finite closest point"
        );
        let radius = (proj.point - sphere.center()).length();
        assert!(
            approx(radius, 3.75, 1e-12),
            "closest off the sphere: {radius}"
        );
        assert!(approx(proj.distance, 3.75, 1e-12), "dist={}", proj.distance);
        let measured = (sphere.center() - proj.point).length();
        assert!(approx(proj.distance, measured, 1e-12));
    }

    // ── point_to_torus ───────────────────────────────────────────────────────

    #[test]
    fn torus_normal_offset_recovers_parameters() {
        let torus = tilted_torus();
        for (u, v, offset) in [(2.1_f64, 0.9_f64, 1.4_f64), (5.3, 4.1, -1.05)] {
            let surf = torus.evaluate(u, v);
            let query = surf + torus.normal(u, v) * offset;
            let proj = point_to_torus(query, &torus);
            assert_projection_contract(query, &proj, |a, b| torus.evaluate(a, b), 1e-9);
            assert!(approx(proj.u, u, 1e-9), "u={} want {u}", proj.u);
            assert!(approx(proj.v, v, 1e-9), "v={} want {v}", proj.v);
            assert!(pt_approx(proj.point, surf, 1e-9));
            assert!(approx(proj.distance, offset.abs(), 1e-9));
            assert!(approx(
                proj.distance,
                torus_closed_form(query, &torus),
                1e-9
            ));
        }
    }

    #[test]
    fn torus_point_inside_the_ring_hole() {
        let torus = tilted_torus();
        let dir = radial_at(torus.x_axis(), torus.y_axis(), 2.1);
        // ρ = 1.2 (well inside the hole), z = 0.6.
        let query = torus.center() + dir * 1.2 + torus.z_axis() * 0.6;
        let proj = point_to_torus(query, &torus);
        assert_projection_contract(query, &proj, |a, b| torus.evaluate(a, b), 1e-9);
        assert!(approx(proj.u, 2.1, 1e-9), "u={}", proj.u);
        // |sqrt((1.2 − 4.5)² + 0.6²) − 1.75|
        let expected = (3.3_f64.hypot(0.6) - 1.75).abs();
        assert!(
            approx(proj.distance, expected, 1e-9),
            "dist={}",
            proj.distance
        );
        assert!(approx(
            proj.distance,
            torus_closed_form(query, &torus),
            1e-9
        ));
    }

    #[test]
    fn torus_axis_point_seeds_the_x_axis_meridian() {
        // Axis-aligned so the radial component is exactly zero.
        let torus = ToroidalSurface::new(Point3::new(2.5, -1.5, 3.25), 4.5, 1.75).unwrap();
        let query = Point3::new(2.5, -1.5, 6.25); // center + 3·z
        let proj = point_to_torus(query, &torus);
        assert_projection_contract(query, &proj, |a, b| torus.evaluate(a, b), 1e-12);
        assert!(proj.u.abs() < 1e-12, "u={}", proj.u);
        // v is the meridian angle atan2(z, ρ − R) = atan2(3, −4.5).
        assert!(approx(proj.v, 3.0_f64.atan2(-4.5), 1e-12), "v={}", proj.v);
        let expected = (4.5_f64.hypot(3.0) - 1.75).abs();
        assert!(
            approx(proj.distance, expected, 1e-12),
            "dist={}",
            proj.distance
        );
        assert!(approx(
            proj.distance,
            torus_closed_form(query, &torus),
            1e-12
        ));
    }

    #[test]
    fn torus_axis_point_with_rotated_frames() {
        // Two frames whose x_axis is exactly (0,1,0) and (0,0,-1): the axis
        // probes stay exact while every component of `centre + R·x_axis` is
        // exercised at least once with a non-zero factor.
        let centre = Point3::new(2.5, -1.5, 3.25);
        for (axis, query) in [
            (Vec3::new(0.0, 0.0, 1.0), Point3::new(2.5, -1.5, 6.25)),
            (Vec3::new(0.0, 1.0, 0.0), Point3::new(2.5, 1.5, 3.25)),
        ] {
            let torus = ToroidalSurface::with_axis(centre, 4.5, 1.75, axis).unwrap();
            let proj = point_to_torus(query, &torus);
            assert_projection_contract(query, &proj, |a, b| torus.evaluate(a, b), 1e-12);
            assert!(proj.u.abs() < 1e-12, "u={}", proj.u);
            // The seeded meridian is the one through centre + R·x_axis.
            let major = centre + torus.x_axis() * 4.5;
            assert!(
                approx((proj.point - major).length(), 1.75, 1e-12),
                "closest is not on the seeded meridian circle"
            );
            let expected = (4.5_f64.hypot(3.0) - 1.75).abs();
            assert!(
                approx(proj.distance, expected, 1e-12),
                "dist={}",
                proj.distance
            );
            assert!(approx(
                proj.distance,
                torus_closed_form(query, &torus),
                1e-12
            ));
        }
    }

    #[test]
    fn torus_points_on_the_major_circle_return_the_minor_radius() {
        let center = Point3::new(2.5, -1.5, 3.25);
        let torus = ToroidalSurface::new(center, 4.5, 1.75).unwrap();
        // x_axis = (1,0,0), y_axis = (0,1,0): both probes are exact.
        for (query, dir) in [
            (Point3::new(7.0, -1.5, 3.25), Vec3::new(1.0, 0.0, 0.0)),
            (Point3::new(2.5, 3.0, 3.25), Vec3::new(0.0, 1.0, 0.0)),
        ] {
            let proj = point_to_torus(query, &torus);
            assert_projection_contract(query, &proj, |a, b| torus.evaluate(a, b), 1e-12);
            assert!(approx(proj.distance, 1.75, 1e-12), "dist={}", proj.distance);
            assert!(pt_approx(proj.point, query + dir * 1.75, 1e-12));
        }
        // A frame whose x_axis is (0,0,-1) exercises the radial z component.
        let tilted =
            ToroidalSurface::with_axis(center, 4.5, 1.75, Vec3::new(0.0, 1.0, 0.0)).unwrap();
        let query = Point3::new(2.5, -1.5, -1.25); // center + 4.5·x_axis
        let proj = point_to_torus(query, &tilted);
        assert_projection_contract(query, &proj, |a, b| tilted.evaluate(a, b), 1e-12);
        assert!(approx(proj.distance, 1.75, 1e-12), "dist={}", proj.distance);
        assert!(pt_approx(proj.point, Point3::new(2.5, -1.5, -3.0), 1e-12));
    }

    // ── point_to_nurbs_surface ───────────────────────────────────────────────

    fn wavy_nurbs() -> remus_math::nurbs::surface::NurbsSurface {
        // Bi-quadratic patch over u ∈ [0.5, 2.0], v ∈ [-1.0, 1.5] so neither
        // domain starts at 0 nor has unit width.
        let knots_u = vec![0.5, 0.5, 0.5, 2.0, 2.0, 2.0];
        let knots_v = vec![-1.0, -1.0, -1.0, 1.5, 1.5, 1.5];
        let cps = vec![
            vec![
                Point3::new(-2.0, 1.0, 0.5),
                Point3::new(0.5, 1.5, 2.0),
                Point3::new(3.0, 0.75, -1.0),
            ],
            vec![
                Point3::new(-1.5, 3.5, 1.25),
                Point3::new(1.0, 4.0, 3.5),
                Point3::new(3.5, 3.0, 0.25),
            ],
            vec![
                Point3::new(-3.0, 6.0, -0.5),
                Point3::new(0.25, 6.5, 1.5),
                Point3::new(4.0, 5.5, 2.5),
            ],
        ];
        let weights = vec![vec![1.0; 3]; 3];
        remus_math::nurbs::surface::NurbsSurface::new(2, 2, knots_u, knots_v, cps, weights).unwrap()
    }

    #[test]
    fn nurbs_grid_search_matches_the_documented_11x11_grid() {
        let surface = wavy_nurbs();
        let query = Point3::new(1.2, 2.8, 5.0);
        let (u_min, u_max) = surface.domain_u();
        let (v_min, v_max) = surface.domain_v();

        // Independent evaluation of the documented 11×11 sampling.
        let mut best = (f64::MAX, u_min, v_min);
        for i in 0..=10_u32 {
            for j in 0..=10_u32 {
                let u = (u_max - u_min).mul_add(f64::from(i) / 10.0, u_min);
                let v = (v_max - v_min).mul_add(f64::from(j) / 10.0, v_min);
                let d2 = (surface.evaluate(u, v) - query).length_squared();
                if d2 < best.0 {
                    best = (d2, u, v);
                }
            }
        }
        // The minimum must be interior, otherwise a corrupted grid could still
        // land on the same sample.
        assert!(best.1 > u_min && best.1 < u_max, "u at the domain edge");
        assert!(best.2 > v_min && best.2 < v_max, "v at the domain edge");

        let proj = point_to_nurbs_surface(query, &surface);
        assert!(
            approx(proj.u, best.1, 1e-12),
            "u={} want {}",
            proj.u,
            best.1
        );
        assert!(
            approx(proj.v, best.2, 1e-12),
            "v={} want {}",
            proj.v,
            best.2
        );
        assert!(
            approx(proj.distance, best.0.sqrt(), 1e-12),
            "dist={} want {}",
            proj.distance,
            best.0.sqrt()
        );
        assert!(pt_approx(
            proj.point,
            surface.evaluate(best.1, best.2),
            1e-12
        ));
        let measured = (query - proj.point).length();
        assert!(
            approx(proj.distance, measured, 1e-9),
            "dist != |P - closest|"
        );
    }

    // ── point_to_surface (generic Newton) ────────────────────────────────────

    #[test]
    fn generic_solver_matches_the_analytic_torus_distance() {
        let torus = tilted_torus();
        for (u, v, offset) in [(2.1_f64, 0.9_f64, 1.4_f64), (4.4, 3.7, -1.05)] {
            let surf = torus.evaluate(u, v);
            let query = surf + torus.normal(u, v) * offset;
            let proj = point_to_surface(query, &torus, (-1.0, 5.5), (-2.0, 5.0));
            assert!(
                proj.distance.is_finite() && proj.u.is_finite() && proj.v.is_finite(),
                "non-finite result"
            );
            assert!(approx(proj.u, u, 1e-5), "u={} want {u}", proj.u);
            assert!(approx(proj.v, v, 1e-5), "v={} want {v}", proj.v);
            assert!(pt_approx(proj.point, surf, 1e-5));
            assert!(
                approx(proj.distance, offset.abs(), 1e-8),
                "dist={} want {}",
                proj.distance,
                offset.abs()
            );
            assert!(approx(
                proj.distance,
                torus_closed_form(query, &torus),
                1e-8
            ));
            let measured = (query - proj.point).length();
            assert!(
                approx(proj.distance, measured, 1e-12),
                "dist != |P - closest|"
            );
            assert!(pt_approx(proj.point, torus.evaluate(proj.u, proj.v), 1e-12));
        }
    }

    #[test]
    fn generic_solver_matches_the_analytic_cylinder_distance() {
        let cyl = tilted_cylinder();
        let (u, v, offset) = (1.1_f64, -2.75_f64, 1.9_f64);
        let surf = cyl.evaluate(u, v);
        let query = surf + cyl.normal(u, v) * offset;
        let proj = point_to_surface(query, &cyl, (-1.0, 5.5), (-3.5, 6.5));
        assert!(approx(proj.u, u, 1e-5), "u={}", proj.u);
        assert!(approx(proj.v, v, 1e-5), "v={}", proj.v);
        assert!(
            approx(proj.distance, offset, 1e-8),
            "dist={} want {offset}",
            proj.distance
        );
        assert!(pt_approx(proj.point, surf, 1e-5));
        let measured = (query - proj.point).length();
        assert!(
            approx(proj.distance, measured, 1e-12),
            "dist != |P - closest|"
        );
    }

    #[test]
    fn generic_solver_grid_seed_beats_the_range_midpoint() {
        // Documented purpose of the grid phase: avoid local-minimum traps.
        // The midpoint of both ranges is the far side of the ring and the
        // inner side of the tube, which is a stationary point of the distance
        // function, so a solver seeded there converges to the wrong extremum.
        let torus = tilted_torus();
        let (u, v, offset) = (0.2_f64, 0.6_f64, 1.3_f64);
        let surf = torus.evaluate(u, v);
        let query = surf + torus.normal(u, v) * offset;
        let proj = point_to_surface(query, &torus, (0.0, TAU), (0.0, TAU));
        assert!(
            approx(proj.distance, offset, 1e-8),
            "dist={} want {offset}",
            proj.distance
        );
        assert!(approx(
            proj.distance,
            torus_closed_form(query, &torus),
            1e-8
        ));
        assert!(pt_approx(proj.point, surf, 1e-5));
        assert!(approx(proj.u, u, 1e-5), "u={}", proj.u);
        assert!(approx(proj.v, v, 1e-5), "v={}", proj.v);
    }

    #[test]
    fn generic_solver_grid_must_span_the_u_range() {
        // A grid that collapses onto u0 seeds the solver more than a quarter
        // turn away from the answer, which lands it on the opposite
        // stationary point of the ring.
        let torus = tilted_torus();
        let (u, v, offset) = (4.0_f64, 4.0_f64, 1.3_f64);
        let surf = torus.evaluate(u, v);
        let query = surf + torus.normal(u, v) * offset;
        let proj = point_to_surface(query, &torus, (0.5, 7.5), (0.5, 7.5));
        assert!(
            approx(proj.distance, offset, 1e-8),
            "dist={} want {offset}",
            proj.distance
        );
        assert!(approx(
            proj.distance,
            torus_closed_form(query, &torus),
            1e-8
        ));
        assert!(pt_approx(proj.point, surf, 1e-5));
    }

    /// A gently warped, sheared patch: `Su·Sv ≠ 0`, so the Gauss-Newton cross
    /// terms are live, but the surface is tame enough for the solver to
    /// converge.
    fn sheared_nurbs() -> remus_math::nurbs::surface::NurbsSurface {
        let knots_u = vec![0.5, 0.5, 0.5, 2.0, 2.0, 2.0];
        let knots_v = vec![-1.0, -1.0, -1.0, 1.5, 1.5, 1.5];
        let cps = vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.6, 1.5, 0.0),
                Point3::new(1.2, 3.0, 0.25),
            ],
            vec![
                Point3::new(2.0, 0.4, 0.1),
                Point3::new(2.6, 1.9, 0.45),
                Point3::new(3.2, 3.4, 0.8),
            ],
            vec![
                Point3::new(4.0, 0.8, 0.0),
                Point3::new(4.6, 2.3, 0.5),
                Point3::new(5.2, 3.8, 1.1),
            ],
        ];
        let weights = vec![vec![1.0; 3]; 3];
        remus_math::nurbs::surface::NurbsSurface::new(2, 2, knots_u, knots_v, cps, weights).unwrap()
    }

    #[test]
    fn generic_solver_meets_stationarity_on_a_non_orthogonal_patch() {
        // A sheared NURBS patch has Su·Sv ≠ 0, so the cross terms of the
        // Gauss-Newton system are live. The documented stopping conditions are
        // dot(S − P, Su) = 0 and dot(S − P, Sv) = 0.
        let surface = sheared_nurbs();
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let (u, v, offset) = (1.3_f64, 0.2_f64, 1.2_f64);
        let surf = surface.evaluate(u, v);
        let normal = surface
            .partial_u(u, v)
            .cross(surface.partial_v(u, v))
            .normalize()
            .unwrap();
        let query = surf + normal * offset;
        let proj = point_to_surface(query, &surface, (u0, u1), (v0, v1));
        assert!(
            proj.u > u0 && proj.u < u1 && proj.v > v0 && proj.v < v1,
            "solution sits on the clamp boundary: u={} v={}",
            proj.u,
            proj.v
        );
        let su = surface.partial_u(proj.u, proj.v);
        let sv = surface.partial_v(proj.u, proj.v);
        assert!(
            su.dot(sv).abs() > 1e-3,
            "fixture is orthogonal, cross terms unexercised"
        );
        let diff = proj.point - query;
        assert!(diff.dot(su).abs() < 1e-8, "dot(S-P, Su)={}", diff.dot(su));
        assert!(diff.dot(sv).abs() < 1e-8, "dot(S-P, Sv)={}", diff.dot(sv));
        assert!(approx(proj.u, u, 1e-6), "u={}", proj.u);
        assert!(approx(proj.v, v, 1e-6), "v={}", proj.v);
        assert!(
            approx(proj.distance, offset, 1e-8),
            "dist={}",
            proj.distance
        );
        assert!(pt_approx(
            proj.point,
            surface.evaluate(proj.u, proj.v),
            1e-12
        ));
        let measured = (query - proj.point).length();
        assert!(approx(proj.distance, measured, 1e-12));
    }

    #[test]
    fn generic_solver_at_a_sphere_pole_stays_finite() {
        // The pole is a parametric singularity: ∂S/∂u vanishes, so the
        // Gauss-Newton determinant degenerates and the solver must bail out.
        let sphere = SphericalSurface::new(Point3::new(2.5, -1.5, 3.25), 3.75).unwrap();
        let query = Point3::new(2.5, -1.5, 12.25); // 9 above the centre
        let proj = point_to_surface(query, &sphere, (0.0, TAU), (-FRAC_PI_2, FRAC_PI_2));
        assert!(
            proj.distance.is_finite() && proj.u.is_finite() && proj.v.is_finite(),
            "non-finite result: d={} u={} v={}",
            proj.distance,
            proj.u,
            proj.v
        );
        assert!(
            approx(proj.distance, 9.0 - 3.75, 1e-9),
            "dist={}",
            proj.distance
        );
        assert!(approx(proj.v, FRAC_PI_2, 1e-9), "v={}", proj.v);
        let measured = (query - proj.point).length();
        assert!(approx(proj.distance, measured, 1e-12));
    }
}
