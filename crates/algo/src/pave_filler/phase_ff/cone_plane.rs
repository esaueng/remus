//! Rational sections of a cone by planes parallel to its axis.
use super::RawCurve;
use remus_math::{
    aabb::Aabb3,
    nurbs::curve::NurbsCurve,
    surfaces::ConicalSurface,
    vec::{Point3, Vec3},
};
use remus_topology::edge::EdgeCurve;

/// Central projection of a rational circle arc preserves its degree and
/// yields the exact hyperbola, rather than interpolating samples on it.
pub(super) fn parallel_section(
    cone: &ConicalSurface,
    normal: Vec3,
    d: f64,
    v_max: f64,
) -> Option<Vec<RawCurve>> {
    let length = normal.length();
    if !length.is_finite() || length <= 0.0 || !v_max.is_finite() || v_max <= 0.0 {
        return None;
    }
    let normal = normal * (1.0 / length);
    let d = d / length;
    let axis = cone.axis();
    if normal.dot(axis).abs() > 1e-12 {
        return None;
    }
    let apex = cone.apex();
    let distance = d - normal.dot(apex - Point3::new(0.0, 0.0, 0.0));
    if distance.abs() <= 1e-12 {
        return None;
    }
    let radial = (normal - axis * normal.dot(axis)).normalize().ok()? * distance.signum();
    let tangent = axis.cross(radial);
    let (s, c) = cone.half_angle().sin_cos();
    let fraction = distance.abs() / (v_max * c);
    if !(0.0..1.0).contains(&fraction) {
        return Some(Vec::new());
    }
    let limit = fraction.acos();
    // Two arcs keep every projective control weight positive, even when
    // the finite section approaches the asymptotes.
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for (arc, (lo, hi)) in [(-limit, 0.0), (0.0, limit)].into_iter().enumerate() {
        let mid = f64::midpoint(lo, hi);
        let w = ((hi - lo) / 2.0).cos();
        for (control, (u, cw)) in [(lo, 1.0), (mid, w), (hi, 1.0)].into_iter().enumerate() {
            if arc == 1 && control == 0 {
                continue;
            }
            let g = axis * s + (radial * u.cos() + tangent * u.sin()) * (c / cw);
            let denominator = normal.dot(g);
            let weight = cw * denominator / distance;
            if !weight.is_finite() || weight <= 0.0 {
                return None;
            }
            points.push(apex + g * (distance / denominator));
            weights.push(weight);
        }
    }
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        points,
        weights,
    )
    .ok()?;
    let p_start = curve.evaluate(0.0);
    let p_end = curve.evaluate(1.0);
    let bbox = Aabb3::try_from_points(curve.control_points().iter().copied())?;
    Some(vec![RawCurve {
        curve: EdgeCurve::NurbsCurve(curve),
        bbox,
        t_range: (0.0, 1.0),
        p_start,
        p_end,
    }])
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    /// The independent cone equation and plane equation hold between knots,
    /// including on reversed and rigidly placed carriers. Interpolating a
    /// chain of cone samples does not satisfy these identities.
    #[test]
    fn rational_sections_satisfy_both_carriers_between_knots() {
        for axis in [
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.6, 0.0, 0.8),
        ] {
            let apex = Point3::new(13.0, -7.0, 9.0);
            let cone = ConicalSurface::new(apex, axis, 1.0).unwrap();
            let n = Vec3::new(0.0, 1.0, 0.0);
            for offset in [-0.4, 0.25] {
                let d = n.dot(apex - Point3::new(0.0, 0.0, 0.0)) + offset;
                for normal_scale in [1.0, 3.0] {
                    let sections =
                        parallel_section(&cone, n * normal_scale, d * normal_scale, 8.0).unwrap();
                    assert_eq!(sections.len(), 1);
                    let EdgeCurve::NurbsCurve(curve) = &sections[0].curve else {
                        unreachable!()
                    };
                    assert_eq!(curve.degree(), 2);
                    assert!(curve.is_rational());
                    for k in 0..=80 {
                        let point = curve.evaluate(f64::from(k) / 80.0);
                        let relative = point - apex;
                        let axial = relative.dot(axis);
                        let radial = (relative - axis * axial).length();
                        assert!((n.dot(point - Point3::new(0.0, 0.0, 0.0)) - d).abs() < 2e-13);
                        assert!((radial - axial / 1.0_f64.tan()).abs() < 2e-13);
                        assert!(axial > 0.0);
                        assert!(relative.length() <= 8.0 + 2e-12);
                    }
                }
            }
        }
    }

    #[test]
    fn singular_and_oblique_planes_do_not_claim_parallel_section() {
        let cone =
            ConicalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        assert!(parallel_section(&cone, Vec3::new(1.0, 0.0, 0.0), 0.0, 8.0).is_none());
        assert!(parallel_section(&cone, Vec3::new(1.0, 0.0, 0.1), 0.5, 8.0).is_none());
        assert!(parallel_section(&cone, Vec3::new(1.0, 0.0, 0.0), 0.5, f64::NAN).is_none());
        assert!(
            parallel_section(&cone, Vec3::new(1.0, 0.0, 0.0), 9.0, 8.0)
                .unwrap()
                .is_empty()
        );
    }
}
