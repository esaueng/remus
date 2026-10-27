//! Keep every original quadrature sample, contribution bit and flux bit.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use remus_math::surfaces::CylindricalSurface;

fn bits(c: &FaceContribution) -> [u64; 14] {
    [
        c.area,
        c.volume,
        c.volume_moment_x,
        c.volume_moment_y,
        c.volume_moment_z,
        c.volume_second_x,
        c.volume_second_y,
        c.volume_second_z,
        c.volume_product_xy,
        c.volume_product_xz,
        c.volume_product_yz,
        c.centroid_x,
        c.centroid_y,
        c.centroid_z,
    ]
    .map(f64::to_bits)
}

fn integrate<const AREA_ONLY: bool>(
    surface: &CylindricalSurface,
    order: usize,
    volume_only: bool,
    reuse: bool,
    trim: &UvTrim<'_>,
    sign: f64,
) -> (FaceContribution, Vec3, usize) {
    let flux = Cell::new(None);
    NORMAL_EVALUATIONS.with(|count| count.set(0));
    let result = integrate_parametric::<_, AREA_ONLY>(
        surface,
        (0.0, 1.0),
        (-1.0, 8.0),
        IntegrationRule {
            order,
            adaptive: None,
            knots: None,
            reference: Point3::new(0.2, -0.3, 0.4),
            volume_only,
            flux: Some(&flux),
            cone_apex: None,
            cylinder_rows: reuse,
        },
        sign,
        trim,
        PatchScale::ANGULAR,
    )
    .unwrap();
    (
        result,
        flux.get().unwrap_or(Vec3::new(0.0, 0.0, 0.0)),
        NORMAL_EVALUATIONS.with(Cell::get),
    )
}

#[test]
fn cylinder_rows_preserve_full_area_volume_and_flux_bits() {
    let hole = UvLoop::new(
        vec![
            Point2::new(0.2, 0.5),
            Point2::new(0.8, 1.0),
            Point2::new(0.7, 3.0),
            Point2::new(0.3, 2.0),
        ],
        false,
        false,
    );
    let outer = UvLoop::new(
        vec![
            Point2::new(0.0, -1.0),
            Point2::new(1.0, -0.8),
            Point2::new(1.0, 8.0),
            Point2::new(0.0, 7.7),
        ],
        false,
        false,
    );
    let holes = [hole];
    for scale in [1e-3, 1.0, 1e3] {
        for axis in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.3, -0.4, 0.8)] {
            let surface = CylindricalSurface::new(
                Point3::new(13.0 * scale, -7.0 * scale, 5.0 * scale),
                axis,
                3.0 * scale,
            )
            .unwrap();
            for order in [1, 3, 5, 8, 12] {
                for sign in [-1.0, 1.0] {
                    for trim in [
                        UvTrim::new(None, &[], &[], false),
                        UvTrim::new(Some(&outer), &holes, &[], false),
                    ] {
                        for volume_only in [false, true] {
                            let fresh = integrate::<false>(
                                &surface,
                                order,
                                volume_only,
                                false,
                                &trim,
                                sign,
                            );
                            let reused =
                                integrate::<false>(&surface, order, volume_only, true, &trim, sign);
                            assert_eq!(bits(&fresh.0), bits(&reused.0));
                            assert_eq!(fresh.1.0.map(f64::to_bits), reused.1.0.map(f64::to_bits));
                            assert!(reused.2 <= fresh.2);
                        }
                        let fresh = integrate::<true>(&surface, order, false, false, &trim, sign);
                        let reused = integrate::<true>(&surface, order, false, true, &trim, sign);
                        assert_eq!(bits(&fresh.0), bits(&reused.0));
                    }
                }
            }
        }
    }
}

#[test]
fn normal_work_follows_fixed_u_runs_instead_of_all_abscissae() {
    let surface =
        CylindricalSurface::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.3, -0.4, 0.8), 3.0)
            .unwrap();
    let trim = UvTrim::new(None, &[], &[], false);
    let fresh = integrate::<false>(&surface, 5, true, false, &trim, 1.0);
    let reused = integrate::<false>(&surface, 5, true, true, &trim, 1.0);
    let u_rows = patch_count(1.0, PatchScale::ANGULAR.u) * 5;
    let v_samples = patch_count(9.0, PatchScale::ANGULAR.v) * 5;
    assert_eq!(fresh.2, u_rows * v_samples);
    // Untrimmed traversal remains (iu, iv, gpu, gpv): each v patch repeats
    // the u rows; sliced domains keep u outside all of their v patches.
    assert_eq!(reused.2, u_rows * v_samples / 5);
    assert_eq!(bits(&fresh.0), bits(&reused.0));
}

#[test]
fn row_key_keeps_signed_zero_and_nonfinite_parameters_distinct() {
    let surface =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.3, -0.4, 0.8), 3.0)
            .unwrap();
    let mut row = CylinderRowNormal::default();
    for u in [
        0.0,
        -0.0,
        0.7,
        0.7,
        -0.2,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ] {
        let (_, du, dv) = surface.point_and_partials(u, 1.0);
        let expected = sample_normal(du, dv);
        let actual = row.normal(u, du, dv);
        assert_eq!(expected.0.0.map(f64::to_bits), actual.0.0.map(f64::to_bits));
        assert_eq!(expected.1.to_bits(), actual.1.to_bits());
    }
}
