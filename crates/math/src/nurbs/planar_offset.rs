//! Bounded parallel curves for monotone planar spline profiles.
//!
//! Hermite samples construct candidates. Outward-rounded rational Bernstein
//! constraints, rather than those samples, authorize every complete span.

use super::{
    NurbsCurve,
    reuse::{ReuseBudget, ReuseError},
    reuse_bounds::{OffsetFrame, planar_offset_on_interval, validate_curve_domain},
};
use crate::{
    context::OperationContext,
    vec::{Point3, Vec3},
};

/// Explicit approximation and resource limits for a planar parallel curve.
#[derive(Debug, Clone, Copy)]
pub struct PlanarOffsetOptions {
    /// Maximum full-vector contact error, in model units.
    pub tolerance: f64,
    /// Maximum aggregate coefficient and construction work.
    pub max_work: usize,
    /// Maximum binary subdivisions of an original knot span.
    pub max_depth: usize,
    /// Maximum cubic output pieces.
    pub max_segments: usize,
}

impl PlanarOffsetOptions {
    /// Construct a bounded request; context budgets may further restrict it.
    #[must_use]
    pub const fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            max_work: 2_000_000,
            max_depth: 12,
            max_segments: 512,
        }
    }
}

/// A disclosed approximate parallel curve with a complete contact bound.
#[derive(Debug, Clone)]
pub struct PlanarOffsetFit {
    /// Piecewise cubic curve on the unchanged source parameter domain.
    pub curve: NurbsCurve,
    /// Full-vector deviation from the selected normal offset over every span.
    pub position_bound: f64,
    /// Number of accepted cubic pieces.
    pub segments: usize,
    /// Charged coefficient and construction work.
    pub work_used: usize,
    /// Always true: a general spline's parallel curve is approximated.
    pub approximate: bool,
}

struct IntervalWork {
    range: (f64, f64),
    depth: usize,
}

/// Fit the left normal offset of a monotone planar spline.
///
/// `normal` chooses the side through `normal × tangent`; negate it for the
/// other side. Source and contact derivatives must have positive projection
/// on one fixed chord direction. Input and output domains and source knots
/// are preserved. Supports clamped degree 1–7 curves with positive weights
/// of condition at most 16, finite coordinates up to 1e100, and knot spans
/// of at least 1e-12. No partial fit is returned.
///
/// # Errors
/// Returns typed refusals for invalid options, unsupported carriers or
/// monotonicity, unresolved contact bounds, cancellation and work limits.
#[allow(clippy::float_cmp)]
pub fn fit_planar_offset(
    source: &NurbsCurve,
    normal: Vec3,
    distance: f64,
    options: &PlanarOffsetOptions,
    context: &OperationContext,
) -> Result<PlanarOffsetFit, ReuseError> {
    let mut budget = ReuseBudget::new(options.max_work, context)?;
    budget.spend(source.control_points().len() + source.knots().len() + source.weights().len())?;
    validate_curve_domain(source)?;
    if !distance.is_finite()
        || distance <= 0.0
        || !options.tolerance.is_finite()
        || options.tolerance <= 0.0
    {
        return Err(ReuseError::InvalidOptions {
            reason: "positive finite offset distance and tolerance required",
        });
    }
    let approximation = context.fallback.budget().ok_or(ReuseError::Unsupported {
        reason: "planar offset fitting requires approximation permission",
    })?;
    if !approximation.is_finite() || approximation <= 0.0 {
        return Err(ReuseError::InvalidOptions {
            reason: "positive finite approximation budget required",
        });
    }
    let tolerance = options.tolerance.min(approximation);
    let limit = options.max_segments.min(context.budgets.segments);
    let depth_limit = options.max_depth.min(context.budgets.subdivision_depth);
    if limit == 0 {
        return Err(ReuseError::InvalidOptions {
            reason: "positive offset segment budget required",
        });
    }
    let low = source
        .weights()
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let high = source.weights().iter().copied().fold(0.0, f64::max);
    if source.degree() > 7
        || high / low > 16.0
        || low < 2.0_f64.powi(-128)
        || high > 2.0_f64.powi(128)
        || source
            .control_points()
            .iter()
            .any(|p| p.0.iter().any(|v| v.abs() > 1e100))
    {
        return Err(ReuseError::Unsupported {
            reason: "offset carrier degree, coordinate or weight condition exceeds its bounded domain",
        });
    }
    let mut breaks = Vec::new();
    for &knot in source.knots() {
        if breaks.last().is_none_or(|previous| *previous != knot) {
            breaks.push(knot);
        }
    }
    if breaks.windows(2).any(|w| w[1] - w[0] < 1e-12) {
        return Err(ReuseError::Unsupported {
            reason: "offset knot spans must be at least 1e-12",
        });
    }
    let normal = normal.normalize()?;
    let plane_origin = source.control_points()[0];
    if source
        .control_points()
        .iter()
        .any(|&p| normal.dot(p - plane_origin).abs() > context.tolerance.linear * 0.01)
    {
        return Err(ReuseError::Unsupported {
            reason: "offset source must lie in its planar chart",
        });
    }
    let (a, b) = source.domain();
    let axis = (source.evaluate(b) - source.evaluate(a)).normalize()?;
    let frame = OffsetFrame {
        normal,
        axis,
        distance,
    };
    if breaks.len() - 1 > limit {
        return Err(ReuseError::SegmentLimit { limit });
    }
    let mut pending = breaks
        .windows(2)
        .rev()
        .map(|w| IntervalWork {
            range: (w[0], w[1]),
            depth: 0,
        })
        .collect::<Vec<_>>();
    let mut pieces = Vec::new();
    let mut position_bound = 0.0_f64;
    while let Some(work) = pending.pop() {
        budget.spend(64 * (source.degree() + 1).pow(2))?;
        let candidate = hermite_piece(source, normal, distance, work.range)?;
        let bound = planar_offset_on_interval(source, &candidate, work.range, &frame, &mut budget);
        match bound {
            Ok(bound) if bound <= tolerance => {
                position_bound = position_bound.max(bound);
                pieces.push(candidate);
            }
            result => {
                if let Err(error @ (ReuseError::WorkLimit { .. } | ReuseError::Math(_))) = result {
                    return Err(error);
                }
                if work.depth >= depth_limit {
                    return match result {
                        Ok(bound) => Err(ReuseError::ToleranceExceeded { bound, tolerance }),
                        Err(error) => Err(error),
                    };
                }
                if pieces.len() + pending.len() + 2 > limit {
                    return Err(ReuseError::SegmentLimit { limit });
                }
                let mid = work.range.0.midpoint(work.range.1);
                if mid <= work.range.0 || mid >= work.range.1 {
                    return Err(ReuseError::BoundUnavailable {
                        reason: "offset subdivision has no interior parameter",
                    });
                }
                pending.push(IntervalWork {
                    range: (mid, work.range.1),
                    depth: work.depth + 1,
                });
                pending.push(IntervalWork {
                    range: (work.range.0, mid),
                    depth: work.depth + 1,
                });
            }
        }
    }
    if pieces.is_empty() || pieces.len() > limit {
        return Err(ReuseError::SegmentLimit { limit });
    }
    let segments = pieces.len();
    budget.spend(4 * segments)?;
    let mut points = Vec::with_capacity(3 * segments + 1);
    let mut knots = vec![a; 4];
    for (i, piece) in pieces.iter().enumerate() {
        if i == 0 {
            points.extend_from_slice(piece.control_points());
        } else {
            points.extend_from_slice(&piece.control_points()[1..]);
        }
        if i + 1 < segments {
            knots.extend([piece.domain().1; 3]);
        }
    }
    knots.extend([b; 4]);
    let weights = vec![1.0; points.len()];
    let curve = NurbsCurve::new(3, knots, points, weights)?;
    Ok(PlanarOffsetFit {
        curve,
        position_bound,
        segments,
        work_used: budget.used(),
        approximate: true,
    })
}

fn sample(
    source: &NurbsCurve,
    normal: Vec3,
    distance: f64,
    t: f64,
    left_derivative: bool,
) -> Result<(Point3, Vec3), ReuseError> {
    let point = source.evaluate(t);
    // Both incident pieces share precisely the same endpoint position. Only
    // the derivative takes a one-sided value at a curvature discontinuity.
    let exact_velocity =
        source
            .derivatives(t, 1)
            .get(1)
            .copied()
            .ok_or(ReuseError::BoundUnavailable {
                reason: "offset source has no tangent",
            })?;
    let exact_unit = normal.cross(exact_velocity).normalize()?;
    let station = if left_derivative && t > source.domain().0 {
        t.next_down()
    } else {
        t
    };
    let derivatives = source.derivatives(station, 2);
    let velocity = derivatives
        .get(1)
        .copied()
        .ok_or(ReuseError::BoundUnavailable {
            reason: "offset source has no tangent",
        })?;
    let acceleration = derivatives
        .get(2)
        .copied()
        .unwrap_or(Vec3::new(0.0, 0.0, 0.0));
    let speed = velocity.length();
    if !speed.is_finite() || speed <= 0.0 {
        return Err(ReuseError::BoundUnavailable {
            reason: "offset source tangent is singular",
        });
    }
    let tangent = velocity * (1.0 / speed);
    let tangent_derivative = (acceleration - tangent * tangent.dot(acceleration)) * (1.0 / speed);
    let across = normal.cross(tangent);
    let across_speed = across.length();
    if !across_speed.is_finite() || across_speed <= 0.0 {
        return Err(ReuseError::BoundUnavailable {
            reason: "offset normal is parallel to its source tangent",
        });
    }
    let unit = across * (1.0 / across_speed);
    let across_derivative = normal.cross(tangent_derivative);
    let unit_derivative =
        (across_derivative - unit * unit.dot(across_derivative)) * (1.0 / across_speed);
    let result = (
        point + exact_unit * distance,
        velocity + unit_derivative * distance,
    );
    if result
        .0
        .0
        .iter()
        .chain(result.1.0.iter())
        .any(|v| !v.is_finite())
    {
        return Err(ReuseError::BoundUnavailable {
            reason: "offset sample is not finite",
        });
    }
    Ok(result)
}

fn hermite_piece(
    source: &NurbsCurve,
    normal: Vec3,
    distance: f64,
    range: (f64, f64),
) -> Result<NurbsCurve, ReuseError> {
    let (a, da) = sample(source, normal, distance, range.0, false)?;
    let (b, db) = sample(source, normal, distance, range.1, true)?;
    let h = (range.1 - range.0) / 3.0;
    Ok(NurbsCurve::new(
        3,
        vec![
            range.0, range.0, range.0, range.0, range.1, range.1, range.1, range.1,
        ],
        vec![a, a + da * h, b - db * h, b],
        vec![1.0; 4],
    )?)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::context::{FallbackPolicy, WorkBudgets};

    fn parabola() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.5, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            vec![1.0; 3],
        )
        .unwrap()
    }
    fn context() -> OperationContext {
        OperationContext::new().with_budgets(
            WorkBudgets::new()
                .with_segments(512)
                .with_subdivision_depth(12),
        )
    }

    #[test]
    fn offset_certificate_covers_the_independent_parabola_normal() {
        let fit = fit_planar_offset(
            &parabola(),
            Vec3::new(0.0, 0.0, 1.0),
            0.1,
            &PlanarOffsetOptions::new(1e-8),
            &context(),
        )
        .unwrap();
        assert!(fit.approximate && fit.position_bound <= 1e-8 && fit.segments > 1);
        for i in 0..=2000 {
            let t = f64::from(i) / 2000.0;
            let normal = Vec3::new(-2.0 * t, 1.0, 0.0) * (1.0 / (1.0 + 4.0 * t * t).sqrt());
            let expected = Point3::new(t, t * t, 0.0) + normal * 0.1;
            assert!((fit.curve.evaluate(t) - expected).length() <= fit.position_bound);
        }
    }

    #[test]
    fn offset_certificate_handles_a_large_rotated_multispan_profile() {
        use crate::mat::Mat4;
        let matrix = Mat4::translation(17.0, -23.0, 31.0)
            * Mat4::rotation_z(0.37)
            * Mat4::rotation_x(0.23)
            * Mat4::scale(100.0, 100.0, 100.0);
        let points = [
            (10.0, 10.0),
            (8.0, 10.0),
            (7.0, 14.0),
            (3.0, 14.0),
            (2.0, 10.0),
            (0.0, 10.0),
        ]
        .map(|(x, y)| matrix.mul_point(Point3::new(x, y, 0.0)));
        let source = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.4, 0.7, 1.0, 1.0, 1.0, 1.0],
            points.to_vec(),
            vec![1.0; 6],
        )
        .unwrap();
        let normal = (matrix.mul_point(Point3::new(0.0, 0.0, 1.0))
            - matrix.mul_point(Point3::new(0.0, 0.0, 0.0)))
        .normalize()
        .unwrap();
        let options = PlanarOffsetOptions {
            max_segments: 1364,
            max_depth: 14,
            max_work: 16_000_000,
            ..PlanarOffsetOptions::new(1e-8)
        };
        let context = OperationContext::new().with_budgets(
            WorkBudgets::new()
                .with_segments(1364)
                .with_subdivision_depth(14),
        );
        let fit = fit_planar_offset(&source, normal, 30.0, &options, &context).unwrap();
        for i in 0..=2000 {
            let t = f64::from(i) / 2000.0;
            let expected = source.evaluate(t) + normal.cross(source.tangent(t).unwrap()) * 30.0;
            assert!((fit.curve.evaluate(t) - expected).length() <= fit.position_bound);
        }
    }

    #[test]
    fn offsets_refuse_cusps_exact_only_and_exhausted_budgets() {
        let options = PlanarOffsetOptions::new(1e-8);
        assert!(
            fit_planar_offset(
                &parabola(),
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
                &options,
                &context()
            )
            .is_err()
        );
        let exact = context().with_fallback(FallbackPolicy::ExactOnly);
        assert!(
            fit_planar_offset(&parabola(), Vec3::new(0.0, 0.0, 1.0), 0.1, &options, &exact)
                .is_err()
        );
        let limited = PlanarOffsetOptions {
            max_work: 1,
            ..options
        };
        assert!(matches!(
            fit_planar_offset(
                &parabola(),
                Vec3::new(0.0, 0.0, 1.0),
                0.1,
                &limited,
                &context()
            ),
            Err(ReuseError::WorkLimit { .. })
        ));
    }
}
