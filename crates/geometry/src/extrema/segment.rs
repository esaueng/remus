//! Segment-to-segment minimum distance.
//!
//! Provides [`segment_segment_distance`] using the standard closed-form
//! algorithm (Shoemake / Ericson "Real-Time Collision Detection" §5.1).

use remus_math::fma::fma;
use remus_math::vec::{Point3, Vec3};

/// Bounded closest approach for `r + s*d1 - t*d2`, shared by the segment
/// and native line-parameter APIs. Only a zero direction collapses to a point.
pub(super) fn bounded_line_parameters(
    d1: Vec3,
    s_range: (f64, f64),
    d2: Vec3,
    t_range: (f64, f64),
    r: Vec3,
) -> (f64, f64) {
    let scale1 = direction_scale(d1);
    let scale2 = direction_scale(d2);
    let u = scaled_direction(d1, scale1);
    let v = scaled_direction(d2, scale2);
    let normal = cross_difference(u, v);
    let normal_scale = direction_scale(normal);

    // |d1 x d2|² equals the Gram determinant, but does not subtract two
    // nearly equal squared dot products. Normalize each direction first:
    // a length^4 cutoff must not turn a small crossing into parallel lines.
    // Scaling the normal also avoids squaring a very small crossing angle.
    let s = if normal_scale > 0.0 {
        let scaled_normal = scaled_direction(normal, normal_scale);
        let r_scale = direction_scale(r);
        let numerator = cross_difference(v, scaled_direction(r, r_scale)).dot(scaled_normal);
        let parameter = if r_scale > 0.0 {
            // Form both dot products in normalized coordinates. Multiplying
            // the subnormal normal back in would discard its significant bits
            // (or underflow to zero) before taking the quotient. The remaining
            // scale factors are exact powers of two; combine their exponents
            // before applying them so cancelling large/small factors stay finite.
            scale_by_power_of_two(
                numerator / scaled_normal.length_squared(),
                power_of_two_exponent(r_scale)
                    - power_of_two_exponent(normal_scale)
                    - power_of_two_exponent(scale1),
            )
        } else {
            0.0
        };
        parameter.clamp(s_range.0, s_range.1)
    } else {
        s_range.0
    };

    let t = if scale2 > 0.0 {
        ((v.dot(r + d1 * s) / v.length_squared()) / scale2).clamp(t_range.0, t_range.1)
    } else {
        t_range.0
    };
    // If t hit an endpoint, the closest point on the other line changes.
    let s = if scale1 > 0.0 {
        ((u.dot(d2 * t - r) / u.length_squared()) / scale1).clamp(s_range.0, s_range.1)
    } else {
        s_range.0
    };
    (s, t)
}

fn power_of_two_exponent(scale: f64) -> i32 {
    let bits = scale.to_bits();
    let exponent = ((bits >> 52) & 0x7ff) as i32;
    if exponent == 0 {
        i32::from(bits.ilog2() as u8) - 1074
    } else {
        exponent - 1023
    }
}

fn scale_by_power_of_two(mut value: f64, mut exponent: i32) -> f64 {
    // Every factor is normal and finite. Applying same-sign exponent chunks
    // cannot underflow or overflow before the final value would do so.
    while exponent > 512 {
        value *= f64::from_bits((1023 + 512) << 52);
        exponent -= 512;
    }
    while exponent < -512 {
        value *= f64::from_bits((1023 - 512) << 52);
        exponent += 512;
    }
    value * f64::from_bits(((exponent + 1023) as u64) << 52)
}

fn direction_scale(v: Vec3) -> f64 {
    let largest = v.x().abs().max(v.y().abs()).max(v.z().abs());
    if largest > 0.0 && largest.is_finite() {
        // A power of two changes only the exponent. Dividing by the largest
        // component itself perturbs the directions and can move the foot
        // substantially when two oblique lines are nearly parallel.
        let bits = largest.to_bits();
        let exponent = bits & 0x7ff0_0000_0000_0000;
        if exponent == 0 {
            // Subnormals have no stored exponent. Their highest mantissa
            // bit still names an exact power of two. Flooring at MIN_POSITIVE
            // would leave a subnormal normal tiny after scaling, allowing
            // normal.dot(scaled_normal) to underflow back to zero.
            f64::from_bits(1_u64 << bits.ilog2())
        } else {
            f64::from_bits(exponent)
        }
    } else {
        largest
    }
}

fn scaled_direction(v: Vec3, scale: f64) -> Vec3 {
    if scale > 0.0 {
        // Divide components directly so subnormal lengths do not need an
        // overflowing reciprocal.
        Vec3::new(v.x() / scale, v.y() / scale, v.z() / scale)
    } else {
        v
    }
}

fn cross_difference(a: Vec3, b: Vec3) -> Vec3 {
    // Recover the rounding error of the subtracted product as well. A
    // one-sided FMA leaves a nonzero residual even for cross(a, a).
    let difference = |a: f64, b: f64, c: f64, d: f64| {
        let cd = c * d;
        fma(a, b, -cd) + fma(-c, d, cd)
    };
    Vec3::new(
        difference(a.y(), b.z(), a.z(), b.y()),
        difference(a.z(), b.x(), a.x(), b.z()),
        difference(a.x(), b.y(), a.y(), b.x()),
    )
}

/// Compute the minimum distance between two line segments `(a0, a1)` and
/// `(b0, b1)`.
///
/// Returns `(distance, closest_point_on_a, closest_point_on_b)`.
///
/// The algorithm handles all degenerate cases (one or both segments collapse
/// to a point, parallel segments) without `panic` or division by zero.
///
/// # Examples
///
/// ```
/// use remus_math::vec::Point3;
/// use remus_geometry::extrema::segment_segment_distance;
///
/// // Perpendicular segments in 3-D; closest approach is at (0,0,0) on both.
/// let (dist, pa, pb) = segment_segment_distance(
///     Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, -1.0, 1.0), Point3::new(0.0, 1.0, 1.0),
/// );
/// assert!((dist - 1.0).abs() < 1e-12);
/// ```
#[must_use]
pub fn segment_segment_distance(
    a0: Point3,
    a1: Point3,
    b0: Point3,
    b1: Point3,
) -> (f64, Point3, Point3) {
    let d1 = a1 - a0; // Direction of segment A
    let d2 = b1 - b0; // Direction of segment B
    let r = a0 - b0;

    let (s, t) = bounded_line_parameters(d1, (0.0, 1.0), d2, (0.0, 1.0), r);

    let closest_a = a0 + d1 * s;
    let closest_b = b0 + d2 * t;
    let diff = closest_a - closest_b;
    let dist = diff.x().hypot(diff.y()).hypot(diff.z());
    (dist, closest_a, closest_b)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    // Helper: assert two f64 values are close.
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // Helper: 3-D Euclidean distance between two points.
    fn dist3(p: Point3, q: Point3) -> f64 {
        let d = p - q;
        (d.x() * d.x() + d.y() * d.y() + d.z() * d.z()).sqrt()
    }

    #[test]
    fn parallel_segments_same_line() {
        // Co-linear segments with a gap: A=[0,1], B=[3,4] on X-axis.
        let (dist, pa, pb) = segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        );
        assert!(approx_eq(dist, 2.0, 1e-12), "dist={dist}");
        // Closest points: a1=(1,0,0) and b0=(3,0,0).
        assert!(approx_eq(pa.x(), 1.0, 1e-12), "pa.x={}", pa.x());
        assert!(approx_eq(pb.x(), 3.0, 1e-12), "pb.x={}", pb.x());
    }

    #[test]
    fn skew_segments_closest_approach() {
        // Two skew segments: A along X, B along Y at z=1.
        // Closest approach: (0,0,0) on A and (0,0,1) on B → distance = 1.
        let (dist, pa, pb) = segment_segment_distance(
            Point3::new(-2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, -2.0, 1.0),
            Point3::new(0.0, 2.0, 1.0),
        );
        assert!(approx_eq(dist, 1.0, 1e-12), "dist={dist}");
        assert!(
            approx_eq(pa.x(), 0.0, 1e-12)
                && approx_eq(pa.y(), 0.0, 1e-12)
                && approx_eq(pa.z(), 0.0, 1e-12),
            "pa={:?}",
            pa
        );
        assert!(
            approx_eq(pb.x(), 0.0, 1e-12)
                && approx_eq(pb.y(), 0.0, 1e-12)
                && approx_eq(pb.z(), 1.0, 1e-12),
            "pb={:?}",
            pb
        );
    }

    #[test]
    fn intersecting_segments_zero_distance() {
        // Two segments that cross at (1,1,0).
        let (dist, pa, pb) = segment_segment_distance(
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        );
        assert!(approx_eq(dist, 0.0, 1e-12), "dist={dist}");
        assert!(dist3(pa, pb) < 1e-12, "pa≠pb: pa={:?} pb={:?}", pa, pb);
    }

    #[test]
    fn degenerate_both_points() {
        // Both segments collapse to points: A=(1,0,0), B=(4,0,0) → distance=3.
        let (dist, pa, pb) = segment_segment_distance(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        );
        assert!(approx_eq(dist, 3.0, 1e-12), "dist={dist}");
        assert!(dist3(pa, Point3::new(1.0, 0.0, 0.0)) < 1e-12, "pa={:?}", pa);
        assert!(dist3(pb, Point3::new(4.0, 0.0, 0.0)) < 1e-12, "pb={:?}", pb);
    }

    #[test]
    fn degenerate_a_is_point() {
        // A=(0,0,0), B from (0,2,0) to (0,4,0).
        // Closest point on B to A is (0,2,0), distance=2.
        let (dist, _pa, pb) = segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        );
        assert!(approx_eq(dist, 2.0, 1e-12), "dist={dist}");
        assert!(dist3(pb, Point3::new(0.0, 2.0, 0.0)) < 1e-12, "pb={:?}", pb);
    }

    #[test]
    fn degenerate_b_is_point() {
        // B=(0,0,3), A from (0,0,0) to (0,0,1).
        // Closest on A is endpoint (0,0,1), distance=2.
        let (dist, pa, _pb) = segment_segment_distance(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(0.0, 0.0, 3.0),
        );
        assert!(approx_eq(dist, 2.0, 1e-12), "dist={dist}");
        assert!(dist3(pa, Point3::new(0.0, 0.0, 1.0)) < 1e-12, "pa={:?}", pa);
    }

    #[test]
    fn closest_points_lie_on_respective_segments() {
        // Property: returned closest points must lie on their respective segments.
        let a0 = Point3::new(1.0, 0.0, 0.0);
        let a1 = Point3::new(5.0, 0.0, 0.0);
        let b0 = Point3::new(3.0, 1.0, 2.0);
        let b1 = Point3::new(3.0, 4.0, 5.0);

        let (dist, pa, pb) = segment_segment_distance(a0, a1, b0, b1);

        // pa must lie on segment A: pa = a0 + s*(a1-a0) for s in [0,1].
        let da = a1 - a0;
        let len_a = (da.x() * da.x() + da.y() * da.y() + da.z() * da.z()).sqrt();
        let pa_a0 = pa - a0;
        let s = (pa_a0.x() * da.x() + pa_a0.y() * da.y() + pa_a0.z() * da.z()) / (len_a * len_a);
        assert!((-1e-12..=1.0 + 1e-12).contains(&s), "s out of [0,1]: s={s}");

        // pb must lie on segment B.
        let db = b1 - b0;
        let len_b = (db.x() * db.x() + db.y() * db.y() + db.z() * db.z()).sqrt();
        let pb_b0 = pb - b0;
        let t = (pb_b0.x() * db.x() + pb_b0.y() * db.y() + pb_b0.z() * db.z()) / (len_b * len_b);
        assert!((-1e-12..=1.0 + 1e-12).contains(&t), "t out of [0,1]: t={t}");

        // dist must equal the distance between pa and pb.
        assert!(
            approx_eq(dist, dist3(pa, pb), 1e-12),
            "dist mismatch: dist={dist} actual={}",
            dist3(pa, pb)
        );
    }

    /// Point-to-point degenerate case with all three coordinate deltas
    /// distinct and non-zero: (-2, -3, -6) has length exactly 7, so the
    /// squared-norm sum `4 + 9 + 36 = 49` is pinned by the Pythagorean
    /// closed form rather than by the current output.
    #[test]
    fn degenerate_both_points_distance_uses_every_component() {
        let pa_in = Point3::new(1.0, -2.0, 0.5);
        let pb_in = Point3::new(3.0, 1.0, 6.5);
        let (dist, pa, pb) = segment_segment_distance(pa_in, pa_in, pb_in, pb_in);

        // |(-2, -3, -6)| = sqrt(4 + 9 + 36) = sqrt(49) = 7.
        assert!(approx_eq(dist, 7.0, 1e-12), "dist={dist}");
        assert!(dist3(pa, pa_in) < 1e-12, "pa={pa:?}");
        assert!(dist3(pb, pb_in) < 1e-12, "pb={pb:?}");
        assert!(approx_eq(dist, dist3(pa, pb), 1e-12), "dist vs |pa-pb|");
    }

    /// Segment A collapses to a point whose foot on B falls *strictly inside*
    /// B (t = 0.375), so the projection value is observable instead of being
    /// swallowed by the clamp to [0, 1].
    #[test]
    fn degenerate_a_point_projects_to_interior_of_b() {
        // B: b0 = (0.5, 0.5, 0.5), direction (2, 1, 0), |d2|^2 = 5.
        let b0 = Point3::new(0.5, 0.5, 0.5);
        let b1 = Point3::new(2.5, 1.5, 0.5);
        // Foot at t = 0.375 is (1.25, 0.875, 0.5); lift it by 3 along +z.
        let foot = Point3::new(1.25, 0.875, 0.5);
        let a = Point3::new(1.25, 0.875, 3.5);

        let (dist, pa, pb) = segment_segment_distance(a, a, b0, b1);

        // r = a - b0 = (0.75, 0.375, 3), f = d2 . r = 1.875, t = f/e = 0.375.
        assert!(approx_eq(dist, 3.0, 1e-12), "dist={dist}");
        assert!(dist3(pa, a) < 1e-12, "pa={pa:?}");
        assert!(dist3(pb, foot) < 1e-12, "pb={pb:?}");
        assert!(approx_eq(dist, dist3(pa, pb), 1e-12), "dist vs |pa-pb|");

        // Recovered parameter must be the interior value 0.375, in [0, 1].
        let d2 = b1 - b0;
        let w = pb - b0;
        let e = d2.x() * d2.x() + d2.y() * d2.y() + d2.z() * d2.z();
        let t = (w.x() * d2.x() + w.y() * d2.y() + w.z() * d2.z()) / e;
        assert!((-1e-12..=1.0 + 1e-12).contains(&t), "t out of [0,1]: t={t}");
        assert!(approx_eq(t, 0.375, 1e-12), "t={t}");
    }

    /// Skew, *non-perpendicular* segments whose closest approach is strictly
    /// interior to both (s = 0.4, t = 0.6). The two support lines lie in the
    /// parallel planes z = 0.5 and z = 2.5 and cross in projection, so the
    /// minimum distance is exactly the plane separation, 2.
    ///
    /// Because d1 . d2 = 4 != 0, the single "recompute s from t" step cannot
    /// recover the answer on its own: the full `(b*f - c*e) / (a*e - b*b)`
    /// solve has to be right.
    #[test]
    fn skew_non_perpendicular_interior_closest_approach() {
        // d1 = (2, 1, 0), d2 = (1, 2, 0); a = e = 5, b = 4, denom = 9.
        let a0 = Point3::new(0.3, -0.25, 0.5);
        let a1 = Point3::new(2.3, 0.75, 0.5);
        let b0 = Point3::new(0.5, -1.05, 2.5);
        let b1 = Point3::new(1.5, 0.95, 2.5);

        let (dist, pa, pb) = segment_segment_distance(a0, a1, b0, b1);

        // s = (b*f - c*e)/denom = (4*1.4 - 0.4*5)/9 = 0.4
        // t = (b*s + f)/e       = (4*0.4 + 1.4)/5   = 0.6
        assert!(approx_eq(dist, 2.0, 1e-12), "dist={dist}");
        assert!(dist3(pa, Point3::new(1.1, 0.15, 0.5)) < 1e-12, "pa={pa:?}");
        assert!(dist3(pb, Point3::new(1.1, 0.15, 2.5)) < 1e-12, "pb={pb:?}");
        assert!(approx_eq(dist, dist3(pa, pb), 1e-12), "dist vs |pa-pb|");

        // Both parameters are strictly interior and reproduce the points.
        let da = a1 - a0;
        let wa = pa - a0;
        let aa = da.x() * da.x() + da.y() * da.y() + da.z() * da.z();
        let s = (wa.x() * da.x() + wa.y() * da.y() + wa.z() * da.z()) / aa;
        let db = b1 - b0;
        let wb = pb - b0;
        let bb = db.x() * db.x() + db.y() * db.y() + db.z() * db.z();
        let t = (wb.x() * db.x() + wb.y() * db.y() + wb.z() * db.z()) / bb;
        assert!((-1e-12..=1.0 + 1e-12).contains(&s), "s out of [0,1]: s={s}");
        assert!((-1e-12..=1.0 + 1e-12).contains(&t), "t out of [0,1]: t={t}");
        assert!(approx_eq(s, 0.4, 1e-12), "s={s}");
        assert!(approx_eq(t, 0.6, 1e-12), "t={t}");
    }
}
