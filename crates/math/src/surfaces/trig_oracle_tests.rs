//! Bit-identity oracle for the analytic surfaces' trigonometry-sharing entry
//! points.
//!
//! The `*_pre_trig` methods are each surface's `evaluate` and `normal` as
//! they stood before `point_and_normal_trig` was added, kept verbatim (with
//! the already shared `evaluate_trig`/`tube_terms` bodies inlined), so a fault
//! in a shared helper cannot move the oracle with it. Every case compares, bit
//! for bit, the current `evaluate`/`normal` and `point_and_normal_trig` fed
//! `u.sin_cos()`/`v.sin_cos()` against them on several frames, at parameter
//! grids spanning several periods, at the period and pole landmarks, at large
//! and signed-zero parameters, and at non-finite ones (where only NaN-ness
//! is compared, since NaN payloads are not part of any contract).

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use super::*;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

impl CylindricalSurface {
    /// `evaluate` before `point_and_normal_trig`, verbatim.
    fn evaluate_pre_trig(&self, u: f64, v: f64) -> Point3 {
        let (sin_u, cos_u) = u.sin_cos();
        self.origin
            + self.x_axis * (self.radius * cos_u)
            + self.y_axis * (self.radius * sin_u)
            + self.axis * v
    }

    /// `normal` before `point_and_normal_trig`, verbatim.
    fn normal_pre_trig(&self, u: f64, _v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        self.x_axis * cos_u + self.y_axis * sin_u
    }
}

impl ConicalSurface {
    /// `evaluate` before `point_and_normal_trig`, verbatim.
    fn evaluate_pre_trig(&self, u: f64, v: f64) -> Point3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_a, cos_a) = self.half_angle.sin_cos();
        let radial = self.x_axis * cos_u + self.y_axis * sin_u;
        self.apex + (radial * cos_a + self.axis * sin_a) * v
    }

    /// `normal` before `point_and_normal_trig`, verbatim.
    fn normal_pre_trig(&self, u: f64, _v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_a, cos_a) = self.half_angle.sin_cos();
        let radial = self.x_axis * cos_u + self.y_axis * sin_u;
        // Normal points outward: radial * sin(a) - axis * cos(a)
        radial * sin_a + self.axis * (-cos_a)
    }
}

impl SphericalSurface {
    /// `evaluate` before `point_and_normal_trig`, verbatim.
    fn evaluate_pre_trig(&self, u: f64, v: f64) -> Point3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_v, cos_v) = v.sin_cos();
        self.center
            + self.x_axis * (self.radius * cos_v * cos_u)
            + self.y_axis * (self.radius * cos_v * sin_u)
            + self.z_axis * (self.radius * sin_v)
    }

    /// `normal` before `point_and_normal_trig`, verbatim.
    fn normal_pre_trig(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_v, cos_v) = v.sin_cos();
        self.x_axis * (cos_v * cos_u) + self.y_axis * (cos_v * sin_u) + self.z_axis * sin_v
    }
}

impl ToroidalSurface {
    /// `evaluate` before `point_and_normal_trig`, verbatim.
    fn evaluate_pre_trig(&self, u: f64, v: f64) -> Point3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_v, cos_v) = v.sin_cos();
        let tube_radius = self.minor_radius.fma(cos_v, self.major_radius);
        let height = self.minor_radius * sin_v;
        self.center
            + self.x_axis * (tube_radius * cos_u)
            + self.y_axis * (tube_radius * sin_u)
            + self.z_axis * height
    }

    /// `normal` before `point_and_normal_trig`, verbatim.
    fn normal_pre_trig(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_v, cos_v) = v.sin_cos();
        let radial = self.x_axis * cos_u + self.y_axis * sin_u;
        radial * cos_v + self.z_axis * sin_v
    }
}

/// Bitwise equality, except that any NaN matches any NaN.
fn same(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan())
}

fn assert_same_point(got: Point3, want: Point3, what: &str, u: f64, v: f64) {
    assert!(
        same(got.x(), want.x()) && same(got.y(), want.y()) && same(got.z(), want.z()),
        "{what} at (u, v) = ({u:e}, {v:e}): {got:?} != {want:?}"
    );
}

fn assert_same_vec(got: Vec3, want: Vec3, what: &str, u: f64, v: f64) {
    assert!(
        same(got.x(), want.x()) && same(got.y(), want.y()) && same(got.z(), want.z()),
        "{what} at (u, v) = ({u:e}, {v:e}): {got:?} != {want:?}"
    );
}

/// Angles: a 61-point sweep over three periods (both signs) plus the
/// period, pole and quadrant landmarks, signed zeros, large magnitudes and
/// non-finite values.
fn angles() -> Vec<f64> {
    let mut out: Vec<f64> = (0..61)
        .map(|i| -TAU + 3.0 * TAU * i as f64 / 60.0)
        .collect();
    out.extend([
        0.0,
        -0.0,
        FRAC_PI_2,
        -FRAC_PI_2,
        PI,
        -PI,
        1.5 * PI,
        TAU,
        TAU - 1e-12,
        1e-300,
        f64::MIN_POSITIVE,
        0.1,
        1.0e6 + 0.3,
        -7.25e8,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ]);
    out
}

/// Linear parameters (cylinder/cone `v`): mixed scales, signs and
/// non-finite values.
fn lengths() -> Vec<f64> {
    let mut out: Vec<f64> = (0..21).map(|i| -10.0 + i as f64 * 1.37).collect();
    out.extend([0.0, -0.0, 1e-9, 3.0e7, -2.5e-3, f64::NAN, f64::INFINITY]);
    out
}

fn frames() -> Vec<(Point3, Vec3, Vec3)> {
    vec![
        (
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        (
            Point3::new(1.5, -2.25, 7.0),
            Vec3::new(0.3, -0.4, 0.866),
            Vec3::new(0.0, 1.0, 0.2),
        ),
        (
            Point3::new(-1.0e4, 3.3, 0.125),
            Vec3::new(-1.0, 2.0, -0.5),
            Vec3::new(0.7, 0.1, 0.0),
        ),
    ]
}

#[test]
fn cylinder_trig_entry_points_match_pre_trig_bit_for_bit() {
    for (origin, axis, ref_dir) in frames() {
        for radius in [1.0, 0.75, 1.0e3] {
            for cyl in [
                CylindricalSurface::new(origin, axis, radius).unwrap(),
                CylindricalSurface::with_ref_dir(origin, axis, radius, ref_dir).unwrap(),
            ] {
                for u in angles() {
                    let (sin_u, cos_u) = u.sin_cos();
                    for v in lengths() {
                        let point = cyl.evaluate_pre_trig(u, v);
                        let normal = cyl.normal_pre_trig(u, v);
                        assert_same_point(cyl.evaluate(u, v), point, "cylinder evaluate", u, v);
                        assert_same_vec(cyl.normal(u, v), normal, "cylinder normal", u, v);
                        let (p, n) = cyl.point_and_normal_trig(sin_u, cos_u, v);
                        assert_same_point(p, point, "cylinder trig point", u, v);
                        assert_same_vec(n, normal, "cylinder trig normal", u, v);
                    }
                }
            }
        }
    }
}

#[test]
fn cone_trig_entry_points_match_pre_trig_bit_for_bit() {
    for (apex, axis, ref_dir) in frames() {
        for half_angle in [0.1, 0.25 * PI, 1.3, FRAC_PI_2 - 1e-9] {
            for cone in [
                ConicalSurface::new(apex, axis, half_angle).unwrap(),
                ConicalSurface::with_ref_dir(apex, axis, half_angle, ref_dir).unwrap(),
            ] {
                for u in angles() {
                    let (sin_u, cos_u) = u.sin_cos();
                    for v in lengths() {
                        let point = cone.evaluate_pre_trig(u, v);
                        let normal = cone.normal_pre_trig(u, v);
                        assert_same_point(cone.evaluate(u, v), point, "cone evaluate", u, v);
                        assert_same_vec(cone.normal(u, v), normal, "cone normal", u, v);
                        let (p, n) = cone.point_and_normal_trig(sin_u, cos_u, v);
                        assert_same_point(p, point, "cone trig point", u, v);
                        assert_same_vec(n, normal, "cone trig normal", u, v);
                    }
                }
            }
        }
    }
}

#[test]
fn sphere_trig_entry_points_match_pre_trig_bit_for_bit() {
    for (center, axis, ref_dir) in frames() {
        for radius in [1.0, 10.0, 3.7e-3] {
            for sphere in [
                SphericalSurface::new(center, radius).unwrap(),
                SphericalSurface::with_axis(center, radius, axis).unwrap(),
                SphericalSurface::with_frame(center, radius, axis, ref_dir).unwrap(),
            ] {
                for u in angles() {
                    let (sin_u, cos_u) = u.sin_cos();
                    for v in angles() {
                        let (sin_v, cos_v) = v.sin_cos();
                        let point = sphere.evaluate_pre_trig(u, v);
                        let normal = sphere.normal_pre_trig(u, v);
                        assert_same_point(sphere.evaluate(u, v), point, "sphere evaluate", u, v);
                        assert_same_vec(sphere.normal(u, v), normal, "sphere normal", u, v);
                        let (p, n) = sphere.point_and_normal_trig(sin_u, cos_u, sin_v, cos_v);
                        assert_same_point(p, point, "sphere trig point", u, v);
                        assert_same_vec(n, normal, "sphere trig normal", u, v);
                    }
                }
            }
        }
    }
}

#[test]
fn torus_trig_entry_points_match_pre_trig_bit_for_bit() {
    for (center, axis, ref_dir) in frames() {
        for (major, minor) in [(10.0, 3.0), (1.0, 0.999), (4.0, 1.25), (2.0e3, 1.0e-2)] {
            for torus in [
                ToroidalSurface::new(center, major, minor).unwrap(),
                ToroidalSurface::with_axis(center, major, minor, axis).unwrap(),
                ToroidalSurface::with_axis_and_ref_dir(center, major, minor, axis, ref_dir)
                    .unwrap(),
            ] {
                for u in angles() {
                    let (sin_u, cos_u) = u.sin_cos();
                    for v in angles() {
                        let (sin_v, cos_v) = v.sin_cos();
                        let point = torus.evaluate_pre_trig(u, v);
                        let normal = torus.normal_pre_trig(u, v);
                        assert_same_point(torus.evaluate(u, v), point, "torus evaluate", u, v);
                        assert_same_vec(torus.normal(u, v), normal, "torus normal", u, v);
                        let (p, n) = torus.point_and_normal_trig(sin_u, cos_u, sin_v, cos_v);
                        assert_same_point(p, point, "torus trig point", u, v);
                        assert_same_vec(n, normal, "torus trig normal", u, v);
                    }
                }
            }
        }
    }
}
