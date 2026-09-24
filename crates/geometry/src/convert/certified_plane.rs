//! Algebraic qualification of affine, degree-one NURBS plane patches.
//!
//! This is deliberately narrower than sampled elementary-surface recognition.
//! The original carrier is never replaced. Equal evaluator-safe weights and a clamped 2×2
//! control net reduce the surface to a bilinear polynomial. Error-free sums prove
//! its mixed coefficient is exactly zero for the represented control points.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};

/// A bounded-domain affine plane certificate derived from the complete control net.
#[derive(Debug, Clone)]
pub struct CertifiedAffinePlane {
    surface: NurbsSurface,
    origin: Point3,
    u: Vec3,
    v: Vec3,
    normal: Vec3,
    domain_u: (f64, f64),
    domain_v: (f64, f64),
    max_deviation: f64,
}

impl CertifiedAffinePlane {
    /// Unit normal in increasing-u cross increasing-v orientation.
    #[must_use]
    pub const fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Plane offset in `normal.dot(point) = offset` form.
    #[must_use]
    pub fn offset(&self) -> f64 {
        self.normal
            .dot(Vec3::new(self.origin.x(), self.origin.y(), self.origin.z()))
    }

    /// Control-net bound on the carrier's deviation from this plane (model units).
    #[must_use]
    pub const fn max_deviation(&self) -> f64 {
        self.max_deviation
    }

    /// Map a point on this bounded patch to its original NURBS parameters.
    ///
    /// The bound covers the carrier residual, not merely distance to an infinite
    /// plane. A point outside the original domain is refused; no surface extension
    /// or extrapolation is implied by the plane certificate.
    #[must_use]
    pub fn parameters(&self, point: Point3, tolerance: f64) -> Option<(f64, f64)> {
        if !tolerance.is_finite() || tolerance < self.max_deviation || tolerance < 0.0 {
            return None;
        }
        let delta = point - self.origin;
        let cross = self.u.cross(self.v);
        let denominator = cross.dot(cross);
        let s = delta.cross(self.v).dot(cross) / denominator;
        let t = self.u.cross(delta).dot(cross) / denominator;
        if !s.is_finite()
            || !t.is_finite()
            || !(0.0..=1.0).contains(&s)
            || !(0.0..=1.0).contains(&t)
        {
            return None;
        }
        let residual = (point - (self.origin + self.u * s + self.v * t)).length();
        if !residual.is_finite() || residual + self.max_deviation > tolerance {
            return None;
        }
        let u = (self.domain_u.1 - self.domain_u.0).mul_add(s, self.domain_u.0);
        let v = (self.domain_v.1 - self.domain_v.0).mul_add(t, self.domain_v.0);
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        let actual_residual = (point - self.surface.evaluate(u, v)).length();
        (actual_residual.is_finite() && actual_residual <= tolerance).then_some((u, v))
    }
}

/// Qualify a clamped, equal-weight, 2×2 degree-one NURBS patch as affine.
///
/// Error-free sums must prove the mixed bilinear coefficient is exactly zero.
/// Tolerance never promotes a nearly affine patch. Unequal or evaluator-unsafe weights, unclamped
/// knots, collapsed parameter domains, curved or degenerate control nets refuse.
/// Sampling and best-fit recognition are never used to authorize a result.
#[must_use]
pub fn certify_affine_nurbs_plane(
    surface: &NurbsSurface,
    tolerance: f64,
) -> Option<CertifiedAffinePlane> {
    if !tolerance.is_finite()
        || tolerance < 0.0
        || surface.degree_u() != 1
        || surface.degree_v() != 1
    {
        return None;
    }
    let cp = surface.control_points();
    let weights = surface.weights();
    if cp.len() != 2
        || cp.iter().any(|row| row.len() != 2)
        || weights.len() != 2
        || weights.iter().any(|row| row.len() != 2)
    {
        return None;
    }
    let weight = weights[0][0];
    // The evaluator normalizes by max(Nu*Nv*w). Degree-one basis maxima are
    // at least 1/2 in each direction, so this range keeps that scale nonzero
    // and finite and bounds the normalized ratios. Common factors outside it
    // are algebraically cancellable but unsafe in the actual evaluator.
    if !weight.is_finite()
        || !(f64::MIN_POSITIVE..=f64::MAX / 4.0).contains(&weight)
        || weights
            .iter()
            .flatten()
            .any(|w| w.to_bits() != weight.to_bits())
    {
        return None;
    }
    let clamped_domain = |knots: &[f64]| -> Option<(f64, f64)> {
        let [a, b, c, d] = knots else {
            return None;
        };
        if !a.is_finite()
            || !c.is_finite()
            || c <= a
            || !(c - a).is_finite()
            || a.to_bits() != b.to_bits()
            || c.to_bits() != d.to_bits()
        {
            return None;
        }
        Some((*a, *c))
    };
    let domain_u = clamped_domain(surface.knots_u())?;
    let domain_v = clamped_domain(surface.knots_v())?;
    let origin = cp[0][0];
    let u = cp[1][0] - origin;
    let v = cp[0][1] - origin;
    for coordinate in [Point3::x, Point3::y, Point3::z] {
        if !exact_sum_equal(
            coordinate(cp[0][0]),
            coordinate(cp[1][1]),
            coordinate(cp[1][0]),
            coordinate(cp[0][1]),
        ) {
            return None;
        }
    }
    let cross = u.cross(v);
    let area = cross.length();
    if !area.is_finite() || area <= 64.0 * f64::EPSILON * u.length() * v.length() {
        return None;
    }
    let normal = cross.normalize().ok()?;
    Some(CertifiedAffinePlane {
        surface: surface.clone(),
        origin,
        u,
        v,
        normal,
        domain_u,
        domain_v,
        max_deviation: 0.0,
    })
}

// Knuth's TwoSum yields an exact expansion of a+b for finite, nonoverflowing
// IEEE sums. Compare both components so cancellation cannot conceal curvature.
fn exact_sum_equal(a: f64, b: f64, c: f64, d: f64) -> bool {
    let sum = |a: f64, b: f64| -> Option<(u64, u64)> {
        if !a.is_finite() || !b.is_finite() {
            return None;
        }
        let high = a + b;
        if !high.is_finite() {
            return None;
        }
        let bb = high - a;
        let low = (a - (high - bb)) + (b - bb);
        if !low.is_finite() {
            return None;
        }
        let bits = |value: f64| {
            if value.abs().to_bits() == 0 {
                0
            } else {
                value.to_bits()
            }
        };
        Some((bits(high), bits(low)))
    };
    match (sum(a, b), sum(c, d)) {
        (Some(first), Some(second)) => first == second,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    fn patch(twist: f64, weights: Vec<Vec<f64>>) -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![2.0, 2.0, 6.0, 6.0],
            vec![-3.0, -3.0, 5.0, 5.0],
            vec![
                vec![Point3::new(1.0, 2.0, 3.0), Point3::new(1.0, 6.0, 3.0)],
                vec![
                    Point3::new(4.0, 2.0, 3.0),
                    Point3::new(4.0, 6.0, 3.0 + twist),
                ],
            ],
            weights,
        )
        .unwrap()
    }

    #[test]
    fn certifies_full_domain_and_preserves_original_parameters() {
        let surface = patch(0.0, vec![vec![1.0; 2]; 2]);
        let before = surface.clone();
        let certificate = certify_affine_nurbs_plane(&surface, 1e-7).unwrap();
        assert!(certificate.max_deviation() <= f64::EPSILON);
        assert!((certificate.offset() - 3.0).abs() <= f64::EPSILON);
        for u in [2.0, 3.0, 6.0] {
            for v in [-3.0, 1.0, 5.0] {
                let mapped = certificate
                    .parameters(surface.evaluate(u, v), 1e-7)
                    .unwrap();
                assert!((mapped.0 - u).abs() < 1e-12 && (mapped.1 - v).abs() < 1e-12);
            }
        }
        assert_eq!(surface, before);
        assert!(
            certificate
                .parameters(Point3::new(5.0, 3.0, 3.0), 1e-7)
                .is_none()
        );
        assert!(
            certificate
                .parameters(Point3::new(2.0, 3.0, 3.001), 1e-7)
                .is_none()
        );
    }

    #[test]
    fn rotated_scaled_control_net_has_a_global_bound() {
        let direction_u = Vec3::new(1.0, 2.0, 3.0);
        let direction_v = Vec3::new(-2.0, 1.0, 0.0);
        for scale in [1.0 / 1024.0, 1.0, 1024.0] {
            let origin = Point3::new(8.0 * scale, -7.0 * scale, 6.0 * scale);
            let u = direction_u * (3.0 * scale);
            let v = direction_v * (4.0 * scale);
            let surface = NurbsSurface::new(
                1,
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![vec![origin, origin + v], vec![origin + u, origin + u + v]],
                vec![vec![1.0; 2]; 2],
            )
            .unwrap();
            let tolerance = 1e-9 * scale;
            let certificate = certify_affine_nurbs_plane(&surface, tolerance).unwrap();
            for (s, t) in [(0.25, 0.5), (0.5, 0.75), (0.8, 0.2)] {
                let p = surface.evaluate(s, t);
                let uv = certificate.parameters(p, tolerance).unwrap();
                assert!((uv.0 - s).abs() < 1e-12 && (uv.1 - t).abs() < 1e-12);
                assert!((p - surface.evaluate(uv.0, uv.1)).length() < tolerance);
            }
        }
    }

    #[test]
    fn refuses_curvature_even_inside_a_loose_modeling_tolerance() {
        assert!(certify_affine_nurbs_plane(&patch(1e-14, vec![vec![1.0; 2]; 2]), 0.1).is_none());
        assert!(certify_affine_nurbs_plane(&patch(1e-5, vec![vec![1.0; 2]; 2]), 0.1).is_none());
        assert!(
            certify_affine_nurbs_plane(&patch(0.0, vec![vec![1.0, 2.0], vec![1.0, 1.0]]), 1e-7)
                .is_none()
        );
        assert!(certify_affine_nurbs_plane(&patch(0.0, vec![vec![1.0; 2]; 2]), f64::NAN).is_none());
    }

    #[test]
    fn refuses_evaluator_unsafe_common_weight_scales() {
        for weight in [f64::from_bits(1), f64::MAX / 2.0] {
            assert!(
                certify_affine_nurbs_plane(&patch(0.0, vec![vec![weight; 2]; 2]), 1e-7).is_none()
            );
        }
        assert!(certify_affine_nurbs_plane(&patch(0.0, vec![vec![3.0; 2]; 2]), 1e-7).is_some());
    }

    #[test]
    fn refuses_overflowing_finite_knot_span() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![-f64::MAX, -f64::MAX, f64::MAX, f64::MAX],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();

        assert!(certify_affine_nurbs_plane(&surface, 1e-7).is_none());
    }

    #[test]
    fn translation_and_cancellation_cannot_hide_mixed_coefficients() {
        // The two rounded high sums coincide, but their exact low parts differ.
        assert_eq!((1e16_f64 + 1.0).to_bits(), 1e16_f64.to_bits());
        assert!(!exact_sum_equal(1e16, 1.0, 1e16, 0.0));
        for offset in [0.0, 1e12] {
            for displacement in [Vec3::new(0.0, 0.0, 0.001), Vec3::new(0.001, 0.0, 0.0)] {
                let origin = Point3::new(offset, offset, offset);
                let u = Vec3::new(3.0, 0.0, 0.0);
                let v = Vec3::new(0.0, 4.0, 0.0);
                let surface = NurbsSurface::new(
                    1,
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![
                        vec![origin, origin + v],
                        vec![origin + u, origin + u + v + displacement],
                    ],
                    vec![vec![1.0; 2]; 2],
                )
                .unwrap();
                assert!(certify_affine_nurbs_plane(&surface, 1.0).is_none());
            }
        }
    }
}
