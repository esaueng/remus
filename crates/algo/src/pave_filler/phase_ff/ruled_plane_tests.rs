//! Algebraic sections must preserve rational profiles and refuse uncertainty.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;

use super::exact_ruled_plane_section;

fn wall(degree: usize, rational: bool) -> NurbsSurface {
    let points = if degree == 2 {
        vec![
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(1.0, -2.0, 10.0),
            Point3::new(2.0, 0.0, 10.0),
        ]
    } else {
        vec![
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(0.5, -2.0, 10.0),
            Point3::new(1.5, -2.0, 10.0),
            Point3::new(2.0, 0.0, 10.0),
        ]
    };
    let mut weights = vec![1.0; points.len()];
    if rational {
        weights[1] = 0.75;
    }
    let top = points
        .iter()
        .map(|&p| p + Vec3::new(0.0, 0.0, -2.0))
        .collect();
    NurbsSurface::new(
        1,
        degree,
        vec![2.0, 2.0, 5.0, 5.0],
        [vec![3.0; degree + 1], vec![7.0; degree + 1]].concat(),
        vec![points, top],
        vec![weights.clone(), weights],
    )
    .unwrap()
}

#[test]
fn exact_sections_cover_both_caps_and_interior_rational_and_cubic_profiles() {
    for degree in [2, 3] {
        for rational in [false, true] {
            let wall = wall(degree, rational);
            for (z, u) in [(10.0, 2.0), (9.0, 3.5), (8.0, 5.0)] {
                let sections =
                    exact_ruled_plane_section(Vec3::new(0.0, 0.0, 1.0), z, &wall, Tolerance::new())
                        .unwrap()
                        .unwrap();
                assert_eq!(sections.len(), 1);
                let EdgeCurve::NurbsCurve(curve) = &sections[0].curve else {
                    unreachable!()
                };
                assert_eq!(curve.degree(), degree);
                assert_eq!(curve.knots(), wall.knots_v());
                assert_eq!(curve.weights(), &wall.weights()[0]);
                assert_eq!(sections[0].t_range, (3.0, 7.0));
                for k in 0..=20 {
                    let v = 3.0 + 4.0 * f64::from(k) / 20.0;
                    let point = curve.evaluate(v);
                    assert!((point - wall.evaluate(u, v)).length() < 1e-12);
                    assert!((point.z() - z).abs() < 1e-12);
                }
            }
        }
    }
}

#[test]
fn exact_disjoint_section_does_not_snap_a_nearby_cap_into_the_plane() {
    let wall = wall(2, false);
    for z in [10.01, 10.0 + 1e-9, 7.99] {
        let sections =
            exact_ruled_plane_section(Vec3::new(0.0, 0.0, 1.0), z, &wall, Tolerance::new())
                .unwrap()
                .unwrap();
        assert!(sections.is_empty());
    }
}

#[test]
fn non_parallel_control_rows_and_unequal_paired_weights_remain_unresolved() {
    let wall = wall(2, true);
    let mut points = wall.control_points().to_vec();
    points[0][1] = points[0][1] + Vec3::new(0.0, 0.0, 0.001);
    let warped = NurbsSurface::new(
        1,
        2,
        wall.knots_u().to_vec(),
        wall.knots_v().to_vec(),
        points,
        wall.weights().to_vec(),
    )
    .unwrap();
    let mut weights = wall.weights().to_vec();
    weights[1][1] = 0.5;
    let unequal = NurbsSurface::new(
        1,
        2,
        wall.knots_u().to_vec(),
        wall.knots_v().to_vec(),
        wall.control_points().to_vec(),
        weights,
    )
    .unwrap();
    for surface in [warped, unequal] {
        assert!(
            exact_ruled_plane_section(Vec3::new(0.0, 0.0, 1.0), 10.0, &surface, Tolerance::new())
                .unwrap()
                .is_none()
        );
    }
}

#[test]
fn a_longitudinal_or_coincident_plane_is_not_a_transverse_section() {
    let wall = wall(2, false);
    for normal in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)] {
        assert!(
            exact_ruled_plane_section(normal, 0.0, &wall, Tolerance::new())
                .unwrap()
                .is_none()
        );
    }
}

/// A quadratic-in-u sweep that touches z = 10 only along its u = 2 row
/// (every other row strictly below) meets that plane exactly in the row's
/// curve. A sweep that crosses the plane, or whose touching row bows off it,
/// is left to the general path.
#[test]
fn a_surface_touching_a_plane_along_one_boundary_row_meets_it_in_that_row() {
    use super::exact_boundary_plane_section;
    let row = |z: f64, twist: f64| -> Vec<Point3> {
        [(0.0, 0.0), (1.0, -2.0), (2.0, 0.0)]
            .iter()
            .map(|&(x, y)| Point3::new(x + twist * y, y - twist * x, z))
            .collect()
    };
    let sweep = |zs: [f64; 3], bow: f64| {
        let mut rows = vec![row(zs[0], 0.0), row(zs[1], 0.1), row(zs[2], 0.2)];
        rows[0][1] = rows[0][1] + Vec3::new(0.0, 0.0, bow);
        NurbsSurface::new(
            2,
            2,
            vec![2.0, 2.0, 2.0, 5.0, 5.0, 5.0],
            vec![3.0, 3.0, 3.0, 7.0, 7.0, 7.0],
            rows,
            vec![vec![1.0, 0.75, 1.0]; 3],
        )
        .unwrap()
    };
    let up = Vec3::new(0.0, 0.0, 1.0);
    let sections =
        exact_boundary_plane_section(up, 10.0, &sweep([10.0, 9.0, 8.0], 0.0), Tolerance::new())
            .unwrap()
            .unwrap();
    assert_eq!(sections.len(), 1);
    let EdgeCurve::NurbsCurve(curve) = &sections[0].curve else {
        unreachable!()
    };
    assert_eq!(curve.control_points(), row(10.0, 0.0).as_slice());
    assert_eq!(curve.knots(), &[3.0, 3.0, 3.0, 7.0, 7.0, 7.0]);
    for k in 0..=8 {
        let t = 3.0 + 0.5 * f64::from(k);
        assert!((curve.evaluate(t).z() - 10.0).abs() < 1e-12);
    }
    // The far end row touches the plane the same way.
    assert!(
        exact_boundary_plane_section(up, 8.0, &sweep([10.0, 9.0, 8.0], 0.0), Tolerance::new())
            .unwrap()
            .is_some()
    );
    // A plane through the middle row crosses the surface.
    assert!(
        exact_boundary_plane_section(up, 9.0, &sweep([10.0, 9.0, 8.0], 0.0), Tolerance::new())
            .unwrap()
            .is_none()
    );
    // A boundary row with a control point off the plane is not certified.
    assert!(
        exact_boundary_plane_section(up, 10.0, &sweep([10.0, 9.0, 8.0], 1e-3), Tolerance::new())
            .unwrap()
            .is_none()
    );
    // Rows on both sides of the plane through the boundary row: not certified.
    assert!(
        exact_boundary_plane_section(up, 10.0, &sweep([10.0, 11.0, 9.0], 0.0), Tolerance::new())
            .unwrap()
            .is_none()
    );
}
