//! Outward surface normals of face carriers at a 3D point.
//!
//! Shared by the mesh assembler and the fillet engine, so it lives with the
//! lower of the two (RFC 0007 stage 0).

use remus_math::nurbs::projection::{
    SurfaceSeedGrid, project_point_to_surface, project_point_to_surface_with_grid,
};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::face::FaceSurface;

/// Compute the outward surface normal of a `FaceSurface` at a given 3D point.
///
/// For analytic surfaces this is exact (no parameter-space projection needed).
/// For NURBS surfaces, uses the midpoint normal as an approximation (full
/// point-inversion would be needed for exactness, but this suffices for fillet
/// cross-section geometry where the point is known to lie on the surface).
pub fn face_surface_normal_at(surface: &FaceSurface, point: Point3) -> Option<Vec3> {
    match surface {
        FaceSurface::Plane { normal, .. } => Some(*normal),
        FaceSurface::Cylinder(cyl) => {
            // Project point onto cylinder axis to find closest axis point,
            // then the normal is the radial direction from axis to point.
            let dp = point - cyl.origin();
            let along_axis = dp.dot(cyl.axis());
            let on_axis = cyl.origin() + cyl.axis() * along_axis;
            (point - on_axis).normalize().ok()
        }
        FaceSurface::Cone(cone) => {
            // For a cone, the normal is perpendicular to the surface.
            // Project point onto axis, compute the radial direction,
            // then rotate by (90° - half_angle) around the tangent.
            let dp = point - cone.apex();
            let along_axis = dp.dot(cone.axis());
            let radial = dp - cone.axis() * along_axis;
            let radial_n = radial.normalize().ok()?;
            let (sin_a, cos_a) = cone.half_angle().sin_cos();
            Some(radial_n * sin_a + cone.axis() * (-cos_a))
        }
        FaceSurface::Sphere(sph) => (point - sph.center()).normalize().ok(),
        FaceSurface::Torus(tor) => {
            // Project point onto the major circle plane to find the closest
            // point on the major circle, then the normal is from the tube
            // center toward the point.
            let dp = point - tor.center();
            let along_axis = dp.dot(tor.z_axis());
            let in_plane = dp - tor.z_axis() * along_axis;
            let ring_dir = in_plane.normalize().ok()?;
            let tube_center = tor.center() + ring_dir * tor.major_radius();
            (point - tube_center).normalize().ok()
        }
        FaceSurface::Nurbs(srf) => {
            // Project the point onto the NURBS surface to find (u, v), then
            // evaluate the exact normal at that parameter.  Tolerance 1e-4 is
            // sufficient here — we only need (u,v) accurate enough for a
            // reliable normal direction, not for position reconstruction.
            // Use a projection tolerance derived from the standard linear
            // tolerance (×1000) — loose enough for normal-direction accuracy
            // without over-iterating, and scale-aware via Tolerance::new().
            nurbs_normal_at(srf, point, None)
        }
    }
}

/// [`face_surface_normal_at`] for a NURBS face whose coarse projection grid
/// the caller keeps, built once per surface: bit for bit the same normal,
/// without re-evaluating the grid on every call (see
/// [`SurfaceSeedGrid`]). Other surfaces never project and take
/// [`face_surface_normal_at`] directly.
pub fn face_surface_normal_with_grid(
    surface: &FaceSurface,
    point: Point3,
    grid: &mut Option<SurfaceSeedGrid>,
) -> Option<Vec3> {
    match surface {
        FaceSurface::Nurbs(srf) => {
            let grid = grid.get_or_insert_with(|| SurfaceSeedGrid::for_surface(srf));
            nurbs_normal_at(srf, point, Some(grid))
        }
        FaceSurface::Plane { .. }
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => face_surface_normal_at(surface, point),
    }
}

fn nurbs_normal_at(
    srf: &remus_math::nurbs::surface::NurbsSurface,
    point: Point3,
    grid: Option<&SurfaceSeedGrid>,
) -> Option<Vec3> {
    let proj_tol = Tolerance::new().linear * 1e3;
    let projection = match grid {
        Some(grid) => project_point_to_surface_with_grid(srf, point, proj_tol, grid),
        None => project_point_to_surface(srf, point, proj_tol),
    };
    match projection {
        Ok(proj) => srf.normal(proj.u, proj.v).ok(),
        Err(_) => srf.normal(0.5, 0.5).ok(), // fallback to midpoint
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use remus_math::nurbs::surface::NurbsSurface;

    use super::*;

    #[test]
    fn face_surface_normal_with_grid_matches_the_per_call_projection() {
        // A doubly curved bicubic patch, so the coarse grid seed and Newton both
        // matter; the kept grid must give the per-call normal bit for bit.
        let rows: Vec<Vec<Point3>> = (0..4)
            .map(|i| {
                (0..4)
                    .map(|j| {
                        let (x, y) = (f64::from(i), f64::from(j));
                        Point3::new(x, y, 0.3 * (x - 1.5).powi(2) - 0.2 * (y - 1.2).powi(2))
                    })
                    .collect()
            })
            .collect();
        let srf = NurbsSurface::new(
            3,
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            rows,
            vec![vec![1.0, 1.2, 0.9, 1.0]; 4],
        )
        .expect("bicubic patch");
        let surface = FaceSurface::Nurbs(srf.clone());
        let mut grid = None;
        for k in 0..40 {
            let (u, v) = (f64::from(k % 7) / 6.0, f64::from(k % 5) / 4.0);
            // On-surface and slightly lifted query points.
            let point = srf.evaluate(u, v) + Vec3::new(0.0, 0.0, f64::from(k % 3) * 1e-3);
            let kept = face_surface_normal_with_grid(&surface, point, &mut grid);
            let fresh = face_surface_normal_at(&surface, point);
            let bits = |n: Option<Vec3>| n.map(|n| [n.x(), n.y(), n.z()].map(f64::to_bits));
            assert_eq!(bits(kept), bits(fresh), "query {k} at {point:?}");
        }
        assert!(grid.is_some(), "the grid is built on first use and kept");
    }
}
