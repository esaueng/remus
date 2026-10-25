//! Exact segment extrema are independent of model scale and endpoint order.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use proptest::prelude::*;
use remus_geometry::extrema::{line_to_line, segment_segment_distance};
use remus_math::curves::Line3D;
use remus_math::vec::{Point3, Vec3};

#[test]
fn crossing_segments_keep_their_interior_minimum_at_small_scales() {
    // The projected lines cross at (0.4, 0.2), at s=0.4 and t=0.6.
    // Non-orthogonal directions ensure endpoint projection cannot recover it.
    for scale in [1.0_f64, 1e-4, 1e-8, 1e-12, 1e-16, 1e-100, 1e100] {
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(scale, 0.5 * scale, 0.0);
        let b0 = Point3::new(0.1 * scale, -0.4 * scale, 0.0);
        let b1 = Point3::new(0.6 * scale, 0.6 * scale, 0.0);
        let expected = Point3::new(0.4 * scale, 0.2 * scale, 0.0);
        for (a0, a1) in [(a0, a1), (a1, a0)] {
            for (b0, b1) in [(b0, b1), (b1, b0)] {
                let (distance, pa, pb) = segment_segment_distance(a0, a1, b0, b1);
                assert!(
                    distance <= 1e-13 * scale,
                    "scale={scale} distance={distance}"
                );
                assert!((pa - expected).length() <= 1e-13 * scale, "{pa:?}");
                assert!((pb - expected).length() <= 1e-13 * scale, "{pb:?}");
            }
        }
    }
}

#[test]
fn nearly_parallel_segments_keep_their_interior_minimum() {
    // The angle is small enough that |a|²|b|² - (a.b)² rounds to zero.
    for height in [0.0, 1e-10] {
        let (distance, pa, pb) = segment_segment_distance(
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(-1.0, -1e-9, height),
            Point3::new(1.0, 1e-9, height),
        );
        assert!((distance - height).abs() < 1e-15, "{distance}");
        assert!(pa.x().abs() < 1e-12, "{pa:?}");
        assert!(pb.x().abs() < 1e-12, "{pb:?}");
    }
}

#[test]
fn nearly_parallel_bounded_lines_keep_their_interior_minimum() {
    let a = Line3D::new(Point3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let b = Line3D::new(Point3::new(-1.0, -1e-9, 0.0), Vec3::new(1.0, 1e-9, 0.0)).unwrap();
    let result = line_to_line(&a, (0.0, 2.0), &b, (0.0, 2.0));
    assert!(result.distance < 1e-15, "{result:?}");
    assert!(result.point_a.x().abs() < 1e-12, "{result:?}");
    assert!(result.point_b.x().abs() < 1e-12, "{result:?}");
}

#[test]
fn nearly_parallel_oblique_segments_keep_the_exact_crossing_witness() {
    let delta = 2.0_f64.powi(-30);
    // Both segments are symmetric about their unique intersection at zero.
    // Normalizing by the largest component (3) perturbs their small angle;
    // checking only distance would miss the resulting witness displacement.
    let a0 = Point3::new(-1.5, -1.05, -0.45);
    let a1 = Point3::new(1.5, 1.05, 0.45);
    let b0 = Point3::new(-1.5, -1.05 - delta, -0.45);
    let b1 = Point3::new(1.5, 1.05 + delta, 0.45);
    for (a0, a1, b0, b1) in [(a0, a1, b0, b1), (b1, b0, a1, a0)] {
        let (distance, pa, pb) = segment_segment_distance(a0, a1, b0, b1);
        assert!(distance < 1e-15);
        assert!((pa - Point3::new(0.0, 0.0, 0.0)).length() < 1e-14, "{pa:?}");
        assert!((pb - Point3::new(0.0, 0.0, 0.0)).length() < 1e-14, "{pb:?}");
    }
}

#[test]
fn subnormal_angles_keep_finite_crossing_witnesses() {
    for delta in (0..52).map(|bit| f64::from_bits(1 << bit)).chain([1e-310]) {
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(1.0, 0.0, 0.0);
        let b0 = Point3::new(0.0, -delta, 0.0);
        let b1 = Point3::new(1.0, delta, 0.0);
        let expected = Point3::new(0.5, 0.0, 0.0);
        for (a0, a1, b0, b1) in [(a0, a1, b0, b1), (b1, b0, a1, a0)] {
            let (distance, pa, pb) = segment_segment_distance(a0, a1, b0, b1);
            assert!(distance.is_finite() && distance < 1e-15, "{distance}");
            assert!((pa - expected).length() < 1e-14, "{pa:?}");
            assert!((pb - expected).length() < 1e-14, "{pb:?}");
        }
        let a = Line3D::new(a0, a1 - a0).unwrap();
        let b = Line3D::new(b0, b1 - b0).unwrap();
        let result = line_to_line(&a, (0.0, 1.0), &b, (0.0, 1.0));
        assert!(
            result.distance.is_finite() && result.distance < 1e-15,
            "{result:?}"
        );
        assert!((result.point_a - expected).length() < 1e-14, "{result:?}");
        assert!((result.point_b - expected).length() < 1e-14, "{result:?}");
    }
}

#[test]
fn dyadic_crossings_keep_their_witnesses_at_extreme_model_scales() {
    for scale in [f64::from_bits(16), 2.0_f64.powi(-1022), 2.0_f64.powi(1000)] {
        let (distance, pa, pb) = segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(scale, 0.5 * scale, 0.0),
            Point3::new(0.25 * scale, -0.25 * scale, 0.0),
            Point3::new(0.75 * scale, 0.75 * scale, 0.0),
        );
        let expected = Point3::new(0.5 * scale, 0.25 * scale, 0.0);
        assert!(distance.is_finite() && distance.abs() <= f64::from_bits(1));
        assert_eq!(pa, expected);
        assert_eq!(pb, expected);
    }
    // Native line parameters retain physical units, exercising the exponent
    // chunks when the foot itself is extremely far from (or close to) zero.
    for length in [2.0_f64.powi(-1000), 2.0_f64.powi(1000)] {
        let a = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let b = Line3D::new(Point3::new(length, -length, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap();
        let result = line_to_line(&a, (0.0, 2.0 * length), &b, (0.0, 2.0 * length));
        assert!(result.distance.is_finite() && result.distance.abs() <= f64::from_bits(1));
        assert_eq!(result.point_a, Point3::new(length, 0.0, 0.0));
        assert_eq!(result.point_b, result.point_a);
    }
}

/// Enumerate the active sets of the convex squared-distance problem.
/// Small integer coordinates keep the oracle's Gram determinant exact;
/// it uses neither direction normalization nor cross products.
fn active_set_minimum(a0: Point3, a1: Point3, b0: Point3, b1: Point3) -> f64 {
    let point_segment = |point: Point3, start: Point3, end: Point3| {
        let d = end - start;
        let fraction = if d.length_squared() > 0.0 {
            ((point - start).dot(d) / d.length_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (point - (start + d * fraction)).length()
    };
    let mut minimum = point_segment(a0, b0, b1)
        .min(point_segment(a1, b0, b1))
        .min(point_segment(b0, a0, a1))
        .min(point_segment(b1, a0, a1));
    let (u, v, r) = (a1 - a0, b1 - b0, a0 - b0);
    let (a, b, c, d, e) = (u.dot(u), u.dot(v), v.dot(v), u.dot(r), v.dot(r));
    let determinant = a * c - b * b;
    if determinant > 0.0 {
        let s = (b * e - c * d) / determinant;
        let t = (a * e - b * d) / determinant;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            minimum = minimum.min((r + u * s - v * t).length());
        }
    }
    minimum
}

proptest! {
    #[test]
    fn segment_minimum_matches_independent_active_sets(
        coordinates in prop::array::uniform12(-8_i32..=8),
        exponent in -16_i32..=16,
    ) {
        let point = |i| Point3::new(
            f64::from(coordinates[i]),
            f64::from(coordinates[i + 1]),
            f64::from(coordinates[i + 2]),
        );
        let (a0, a1, b0, b1) = (point(0), point(3), point(6), point(9));
        let expected = active_set_minimum(a0, a1, b0, b1);
        let scale = 10.0_f64.powi(exponent);
        let scaled = |p: Point3| Point3::new(p.x() * scale, p.y() * scale, p.z() * scale);
        for (a0, a1, b0, b1) in [(a0, a1, b0, b1), (b1, b0, a1, a0)] {
            let (distance, pa, pb) = segment_segment_distance(
                scaled(a0), scaled(a1), scaled(b0), scaled(b1),
            );
            prop_assert!((distance / scale - expected).abs() <= 1e-12);
            let gap = pa - pb;
            prop_assert!((distance - gap.x().hypot(gap.y()).hypot(gap.z())).abs() <= 1e-12 * scale);
        }
    }
}
