//! Ray-surface intersection for all analytic surface types and NURBS.
//!
//! Each function takes a ray (origin + direction) and a surface, returning
//! the positive-t intersection parameters along the ray.

// Functions used by boundary.rs for ray-face crossing counts.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::ray::{RayHit, RayHits};
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

use crate::CheckError;

// ---------------------------------------------------------------------------
// Tolerance constants
// ---------------------------------------------------------------------------

/// Denominator below this is treated as zero (parallel ray).
const NEAR_ZERO: f64 = 1e-15;

/// Minimum positive t to accept (reject self-intersections at origin).
const RAY_T_MIN: f64 = 1e-12;

// ---------------------------------------------------------------------------
// Ray-plane
// ---------------------------------------------------------------------------

/// Intersect a ray with a plane defined by `normal . X = d`.
///
/// Returns `Some(t)` if the ray hits the plane at `origin + t * direction`
/// with `t > RAY_T_MIN`, or `None` if the ray is parallel or behind.
pub fn ray_plane(origin: Point3, direction: Vec3, normal: Vec3, d: f64) -> Option<f64> {
    let origin_vec = Vec3::new(origin.x(), origin.y(), origin.z());
    let t = remus_math::ray::plane_parameter(
        normal.dot(direction),
        d - normal.dot(origin_vec),
        NEAR_ZERO,
    )?;
    if t <= RAY_T_MIN { None } else { Some(t) }
}

// ---------------------------------------------------------------------------
// Ray-cylinder, ray-cone, ray-sphere
// ---------------------------------------------------------------------------

/// Intersect a ray with an infinite cylindrical surface: up to two qualified
/// hits, unfiltered by sign ([`remus_math::ray::ray_cylinder`]).
pub fn ray_cylinder(origin: Point3, direction: Vec3, cyl: &CylindricalSurface) -> RayHits {
    remus_math::ray::ray_cylinder(origin, direction, cyl)
}

/// Intersect a ray with an infinite double cone: up to two qualified hits
/// over both nappes, unfiltered by sign ([`remus_math::ray::ray_cone`]).
pub fn ray_cone(origin: Point3, direction: Vec3, cone: &ConicalSurface) -> RayHits {
    remus_math::ray::ray_cone(origin, direction, cone)
}

/// Intersect a ray with a sphere: up to two qualified hits, unfiltered by
/// sign ([`remus_math::ray::ray_sphere`]).
pub fn ray_sphere(origin: Point3, direction: Vec3, sph: &SphericalSurface) -> RayHits {
    remus_math::ray::ray_sphere(origin, direction, sph)
}

// ---------------------------------------------------------------------------
// Ray-torus
// ---------------------------------------------------------------------------

/// Intersect a ray with a torus: every real root, qualified so that a merged
/// double root is not read as one crossing
/// ([`remus_math::ray::ray_torus`]), unfiltered by sign.
pub fn ray_torus(origin: Point3, direction: Vec3, tor: &ToroidalSurface) -> Vec<RayHit> {
    remus_math::ray::ray_torus(origin, direction, tor)
}

// ---------------------------------------------------------------------------
// Ray-NURBS
// ---------------------------------------------------------------------------

/// Intersect a ray with a NURBS surface.
///
/// Returns a vector of `(t, u, v)` where `t` is the ray parameter and
/// `(u, v)` are the surface parameters at each hit.
///
/// # Errors
/// Propagates math errors from the NURBS intersection routine.
pub fn ray_nurbs(
    origin: Point3,
    direction: Vec3,
    surface: &NurbsSurface,
    n_samples: usize,
) -> Result<Vec<(f64, f64, f64)>, CheckError> {
    use remus_math::nurbs::intersection::intersect_line_nurbs;

    let hits = intersect_line_nurbs(surface, origin, direction, n_samples)?;
    Ok(forward_nurbs_hits(origin, direction, &hits))
}

/// [`ray_nurbs`] with the line-intersection seed grid supplied rather than
/// rebuilt per ray.
///
/// Returns exactly what [`ray_nurbs`] would when `grid` was built from
/// `surface` with the same sample count
/// ([`remus_math::nurbs::intersection::intersect_line_nurbs_with_grid`]).
///
/// # Errors
/// Propagates math errors from the NURBS intersection routine.
pub fn ray_nurbs_with_grid(
    origin: Point3,
    direction: Vec3,
    surface: &NurbsSurface,
    grid: &remus_math::nurbs::intersection::LineSurfaceSeedGrid,
) -> Result<Vec<(f64, f64, f64)>, CheckError> {
    use remus_math::nurbs::intersection::intersect_line_nurbs_with_grid;

    let hits = intersect_line_nurbs_with_grid(surface, grid, origin, direction)?;
    Ok(forward_nurbs_hits(origin, direction, &hits))
}

/// Forward (`t > RAY_T_MIN`) hits as `(t, u, v)`.
fn forward_nurbs_hits(
    origin: Point3,
    direction: Vec3,
    hits: &[remus_math::nurbs::intersection::IntersectionPoint],
) -> Vec<(f64, f64, f64)> {
    let dir_dot_dir = direction.dot(direction);
    let mut results = Vec::new();
    for hit in hits {
        let diff = hit.point - origin;
        let t = diff.dot(direction) / dir_dot_dir;
        if t > RAY_T_MIN {
            results.push((t, hit.param1.0, hit.param1.1));
        }
    }
    results
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    const TOL: f64 = 1e-10;

    #[test]
    fn ray_sphere_through_center() {
        let sph = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
        let origin = Point3::new(0.0, 0.0, -5.0);
        let dir = Vec3::new(0.0, 0.0, 1.0);
        let hits = ray_sphere(origin, dir, &sph);
        assert_eq!(hits.len(), 2, "expected 2 hits, got {hits:?}");
        assert!((hits[0].t - 4.0).abs() < TOL, "{hits:?}, expected 4.0");
        assert!((hits[1].t - 6.0).abs() < TOL, "{hits:?}, expected 6.0");
    }

    #[test]
    fn ray_plane_parallel() {
        let origin = Point3::new(0.0, 0.0, 5.0);
        let dir = Vec3::new(1.0, 0.0, 0.0); // parallel to XY plane
        let normal = Vec3::new(0.0, 0.0, 1.0);
        assert!(ray_plane(origin, dir, normal, 0.0).is_none());
    }

    #[test]
    fn ray_plane_forward() {
        let origin = Point3::new(0.0, 0.0, 5.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let t = ray_plane(origin, dir, normal, 0.0).expect("should hit plane");
        assert!((t - 5.0).abs() < TOL, "t={t}, expected 5.0");
    }

    #[test]
    fn ray_cylinder_perpendicular() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                .unwrap();
        let origin = Point3::new(5.0, 0.0, 0.0);
        let dir = Vec3::new(-1.0, 0.0, 0.0);
        let hits = ray_cylinder(origin, dir, &cyl);
        assert_eq!(hits.len(), 2, "expected 2 hits, got {hits:?}");
        assert!((hits[0].t - 4.0).abs() < TOL, "{hits:?}, expected 4.0");
        assert!((hits[1].t - 6.0).abs() < TOL, "{hits:?}, expected 6.0");
    }

    #[test]
    fn ray_torus_through_center() {
        // Torus R=3, r=1 centered at origin, lying in XY plane.
        let tor = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        let origin = Point3::new(-5.0, 0.0, 0.0);
        let dir = Vec3::new(1.0, 0.0, 0.0);
        let hits = ray_torus(origin, dir, &tor);
        // Ray along X through torus center: hits at x=-4, -2, 2, 4
        assert_eq!(
            hits.len(),
            4,
            "expected 4 hits, got {}: {:?}",
            hits.len(),
            hits
        );
        // t values: origin.x=-5, so t = x - (-5) = x + 5
        assert!(
            (hits[0].t - 1.0).abs() < TOL,
            "t0={}, expected 1.0 (x=-4)",
            hits[0].t
        );
        assert!(
            (hits[1].t - 3.0).abs() < TOL,
            "t1={}, expected 3.0 (x=-2)",
            hits[1].t
        );
        assert!(
            (hits[2].t - 7.0).abs() < TOL,
            "t2={}, expected 7.0 (x=2)",
            hits[2].t
        );
        assert!(
            (hits[3].t - 9.0).abs() < TOL,
            "t3={}, expected 9.0 (x=4)",
            hits[3].t
        );
    }

    #[test]
    fn ray_torus_off_axis() {
        // Torus R=3, r=1 in XY plane. Vertical ray through tube center at (3,0,0).
        // Tube cross-section at z=±1, so hits at t=4 and t=6 from origin z=5.
        let tor = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        let origin = Point3::new(3.0, 0.0, 5.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);

        let hits = ray_torus(origin, dir, &tor);
        assert_eq!(
            hits.len(),
            2,
            "vertical ray through tube should hit twice, got {}: {:?}",
            hits.len(),
            hits
        );
        // t = 5 - z_hit. Tube at (3,0,0) has z = ±1, so hits at z=1 (t=4) and z=-1 (t=6).
        assert!(
            (hits[0].t - 4.0).abs() < 0.1,
            "t0={}, expected ~4.0",
            hits[0].t
        );
        assert!(
            (hits[1].t - 6.0).abs() < 0.1,
            "t1={}, expected ~6.0",
            hits[1].t
        );
        // Verify hits are on the torus surface
        for t in hits.iter().map(|h| h.t) {
            let p = origin + dir * t;
            let pv = p - tor.center();
            let z = pv.dot(tor.z_axis());
            let r_major = (pv - tor.z_axis() * z).length();
            let dist_to_tube = ((r_major - tor.major_radius()).powi(2) + z * z).sqrt();
            assert!(
                (dist_to_tube - tor.minor_radius()).abs() < 1e-6,
                "hit not on torus surface"
            );
        }
    }

    /// Regression: oblique irrational rays from inside the tube of an
    /// R=6, r=2 torus. The old Ferrari solver returned zero roots for some of
    /// these rays and off-surface spurious roots for others (hits at |z| > r
    /// on a torus spanning z in `[-r, r]`), flipping classification parity.
    #[test]
    fn ray_torus_oblique_from_inside_tube() {
        let tor = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 6.0, 2.0).unwrap();
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
            Vec3::new(
                0.740_535_693_464_567_5,
                -0.350_889_803_483_932_2,
                0.573_576_436_351_046,
            ),
        ];
        // Tube-center origins at several azimuths, plus off-center interiors.
        let mut origins = Vec::new();
        for theta in [0.05_f64, std::f64::consts::PI / 3.0, 2.0, 4.5] {
            origins.push(Point3::new(6.0 * theta.cos(), 6.0 * theta.sin(), 0.0));
            origins.push(Point3::new(4.5 * theta.cos(), 4.5 * theta.sin(), 0.5));
        }
        for origin in origins {
            for dir in dirs {
                let hits = ray_torus(origin, dir, &tor);
                // A forward ray from inside the tube must exit: at least one hit.
                assert!(
                    hits.iter().any(|h| h.t > RAY_T_MIN),
                    "no roots for origin {origin:?} dir {dir:?}"
                );
                for t in hits.iter().map(|h| h.t) {
                    let p = origin + dir * t;
                    let pv = p - tor.center();
                    let z = pv.dot(tor.z_axis());
                    let r_major = (pv - tor.z_axis() * z).length();
                    let dist_to_tube = ((r_major - tor.major_radius()).powi(2) + z * z).sqrt();
                    assert!(
                        (dist_to_tube - tor.minor_radius()).abs() < 1e-6,
                        "spurious off-surface root t={t} for origin {origin:?} dir {dir:?}"
                    );
                }
            }
        }
    }
}
