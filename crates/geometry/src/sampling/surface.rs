//! Regular grid sampling of parametric surfaces.

use remus_math::traits::ParametricSurface;
use remus_math::vec::Point3;

/// Sample a regular N×M grid of surface points.
///
/// Returns `nu` rows, each with `nv` points.
///
/// - Row `i` corresponds to `u = u_range.0 + i*(u_range.1 - u_range.0) / (nu - 1)`.
/// - Column `j` corresponds to `v = v_range.0 + j*(v_range.1 - v_range.0) / (nv - 1)`.
///
/// Edge cases:
/// - If `nu == 0` or `nv == 0`, an empty `Vec` is returned.
/// - If `nu == 1`, the single row is evaluated at `u_range.0`.
/// - If `nv == 1`, each row contains one point evaluated at `v_range.0`.
#[must_use]
pub fn surface_grid<S: ParametricSurface>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    nu: usize,
    nv: usize,
) -> Vec<Vec<Point3>> {
    if nu == 0 || nv == 0 {
        return Vec::new();
    }

    (0..nu)
        .map(|i| {
            let u = if nu == 1 {
                u_range.0
            } else if i == nu - 1 {
                u_range.1
            } else {
                u_range.0 + i as f64 * (u_range.1 - u_range.0) / (nu - 1) as f64
            };

            (0..nv)
                .map(|j| {
                    let v = if nv == 1 {
                        v_range.0
                    } else if j == nv - 1 {
                        v_range.1
                    } else {
                        v_range.0 + j as f64 * (v_range.1 - v_range.0) / (nv - 1) as f64
                    };
                    surface.evaluate(u, v)
                })
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point3, Vec3};
    use std::f64::consts::TAU;

    fn unit_cylinder() -> CylindricalSurface {
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
    }

    #[test]
    fn zero_nu_returns_empty() {
        let s = unit_cylinder();
        let grid = surface_grid(&s, (0.0, TAU), (0.0, 1.0), 0, 4);
        assert!(grid.is_empty());
    }

    #[test]
    fn zero_nv_returns_empty() {
        let s = unit_cylinder();
        let grid = surface_grid(&s, (0.0, TAU), (0.0, 1.0), 4, 0);
        assert!(grid.is_empty());
    }

    #[test]
    fn grid_dimensions_correct() {
        let s = unit_cylinder();
        let grid = surface_grid(&s, (0.0, TAU), (0.0, 1.0), 5, 3);
        assert_eq!(grid.len(), 5, "expected 5 rows");
        for (i, row) in grid.iter().enumerate() {
            assert_eq!(row.len(), 3, "row {i} should have 3 columns");
        }
    }

    #[test]
    fn all_points_on_cylinder_surface() {
        // Cylinder P(u,v) = (cos u, sin u, v).  All points must satisfy x²+y²=1.
        let s = unit_cylinder();
        let grid = surface_grid(&s, (0.0, TAU), (0.0, 2.0), 8, 4);
        for row in &grid {
            for p in row {
                let r = (p.x() * p.x() + p.y() * p.y()).sqrt();
                assert!((r - 1.0).abs() < 1e-10, "point not on cylinder: r={r}");
            }
        }
    }

    #[test]
    fn first_and_last_u_are_endpoints() {
        let s = unit_cylinder();
        let u0 = 0.0_f64;
        let u1 = TAU;
        let grid = surface_grid(&s, (u0, u1), (0.0, 1.0), 4, 2);
        assert_eq!(grid.len(), 4);

        // First row: u = u0 → point should equal surface.evaluate(u0, 0.0)
        let expected_first = s.evaluate(u0, 0.0);
        let actual_first = grid[0][0];
        let d0 = {
            let dx = actual_first.x() - expected_first.x();
            let dy = actual_first.y() - expected_first.y();
            let dz = actual_first.z() - expected_first.z();
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        assert!(d0 < 1e-12, "first row u mismatch: dist={d0}");

        // Last row: u = u1.
        let expected_last = s.evaluate(u1, 0.0);
        let actual_last = grid[3][0];
        let d1 = {
            let dx = actual_last.x() - expected_last.x();
            let dy = actual_last.y() - expected_last.y();
            let dz = actual_last.z() - expected_last.z();
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        assert!(d1 < 1e-12, "last row u mismatch: dist={d1}");
    }

    #[test]
    fn single_row_and_column() {
        let s = unit_cylinder();
        let grid = surface_grid(&s, (0.0, TAU), (0.5, 1.5), 1, 1);
        assert_eq!(grid.len(), 1);
        assert_eq!(grid[0].len(), 1);
        let expected = s.evaluate(0.0, 0.5);
        let p = grid[0][0];
        let d = {
            let dx = p.x() - expected.x();
            let dy = p.y() - expected.y();
            let dz = p.z() - expected.z();
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        assert!(d < 1e-12);
    }

    #[test]
    fn v_endpoint_included_in_last_column() {
        let s = unit_cylinder();
        let v0 = 0.0_f64;
        let v1 = 3.0_f64;
        let grid = surface_grid(&s, (0.0, 1.0), (v0, v1), 2, 4);
        // Last column (j=3) should have v = v1.
        for (i, row) in grid.iter().enumerate() {
            let expected = s.evaluate(if i == 0 { 0.0 } else { 1.0 }, v1);
            let actual = row[3];
            let d = {
                let dx = actual.x() - expected.x();
                let dy = actual.y() - expected.y();
                let dz = actual.z() - expected.z();
                (dx * dx + dy * dy + dz * dz).sqrt()
            };
            assert!(d < 1e-12, "v endpoint mismatch at row {i}: dist={d}");
        }
    }

    fn point_distance(a: Point3, b: Point3) -> f64 {
        ((a.x() - b.x()).powi(2) + (a.y() - b.y()).powi(2) + (a.z() - b.z()).powi(2)).sqrt()
    }

    fn same_point_bitwise(a: Point3, b: Point3) -> bool {
        a.x().to_bits() == b.x().to_bits()
            && a.y().to_bits() == b.y().to_bits()
            && a.z().to_bits() == b.z().to_bits()
    }

    #[test]
    fn interior_grid_parameters_follow_the_documented_formula() {
        // u in [0.5, 2.9] with nu = 4: 0.5, 1.3, 2.1, 2.9.
        // v in [0.4, 3.1] with nv = 4: 0.4, 1.3, 2.2, 3.1.
        // Neither range is [0, 1] nor symmetric, and the two step sizes differ
        // (0.8 vs 0.9), so every mutated form of the index arithmetic lands on a
        // different (u, v).
        let s = unit_cylinder();
        let us = [0.5, 1.3, 2.1, 2.9];
        let vs = [0.4, 1.3, 2.2, 3.1];
        let grid = surface_grid(&s, (0.5, 2.9), (0.4, 3.1), 4, 4);
        assert_eq!(grid.len(), 4, "expected 4 rows");
        for (i, row) in grid.iter().enumerate() {
            assert_eq!(row.len(), 4, "row {i} should have 4 columns");
            for (j, p) in row.iter().enumerate() {
                let expected = s.evaluate(us[i], vs[j]);
                let d = point_distance(*p, expected);
                assert!(
                    d < 1e-9,
                    "grid[{i}][{j}] is not the surface at (u={}, v={}): dist={d}",
                    us[i],
                    vs[j]
                );
            }
        }
    }

    #[test]
    fn last_row_and_column_sit_exactly_on_the_range_ends() {
        // The grid must span the full uv domain: the last row is u_range.1 and
        // the last column is v_range.1 exactly, not the accumulated closed form.
        // These ranges make the difference observable -- with the code's
        // evaluation order, 0.1 + (3*(2.9 - 0.1))/3 is one ulp below 2.9 and
        // 0.1 + (3*(3.4 - 0.1))/3 one ulp below 3.4.
        let (u0, u1, nu) = (0.1_f64, 2.9_f64, 4_usize);
        let (v0, v1, nv) = (0.1_f64, 3.4_f64, 4_usize);
        assert_ne!(
            (u0 + (3.0 * (u1 - u0)) / 3.0).to_bits(),
            u1.to_bits(),
            "fixture is degenerate: the unsnapped u already equals u_range.1"
        );
        assert_ne!(
            (v0 + (3.0 * (v1 - v0)) / 3.0).to_bits(),
            v1.to_bits(),
            "fixture is degenerate: the unsnapped v already equals v_range.1"
        );

        let s = unit_cylinder();
        let grid = surface_grid(&s, (u0, u1), (v0, v1), nu, nv);
        assert_eq!(grid.len(), nu);

        // Column 0 is v_range.0 exactly, so the last row pins u_range.1.
        assert!(
            same_point_bitwise(grid[nu - 1][0], s.evaluate(u1, v0)),
            "last row is not evaluated at u_range.1"
        );
        // Row 0 is u_range.0 exactly, so the last column pins v_range.1.
        assert!(
            same_point_bitwise(grid[0][nv - 1], s.evaluate(u0, v1)),
            "last column is not evaluated at v_range.1"
        );
    }
}
