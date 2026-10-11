//! Opt-in cubic approximation with whole-interval position and derivative bounds.
//!
//! Adapted and modified from Truck's `BSplineCurve::cubic_bezier_interpolation`,
//! `sub_cubic_approximation`, and `cubic_approximation` in
//! `truck-geometry/src/nurbs/bspcurve.rs`, commit
//! `88ed005249e5e3a6b07f62425399435905cd3ab6` (Apache-2.0).
//! The endpoint/tangent construction and adaptive subdivision were adapted to
//! Remus types and resource policy. Truck's sampled acceptance and hash-derived
//! sample location are replaced with conservative, rounding-inclusive bounds.
//! See `docs/production-readiness/truck-reuse-provenance.md` for attribution.
//!
//! This operation always discloses approximation and never replaces a default
//! construction path. Tolerances apply to the original parameter: derivative
//! tolerance has units of model distance per input parameter unit. The output
//! retains the source domain, with cubic pieces meeting at shared endpoints.

use crate::context::OperationContext;
use crate::nurbs::basis;
use crate::nurbs::curve::NurbsCurve;
use crate::nurbs::reuse::{ReuseBudget, ReuseError};
use crate::nurbs::reuse_bounds::{curve_difference_on_interval, validate_curve_domain};
use crate::vec::{Point3, Vec3};

/// Explicit approximation tolerances and deterministic resource limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubicFitOptions {
    /// Maximum whole-domain position difference, in model units.
    pub position_tolerance: f64,
    /// Maximum first derivative difference per source parameter unit.
    pub derivative_tolerance: f64,
    /// Maximum charged construction, evaluation, and certificate work.
    pub max_work: usize,
    /// Maximum binary subdivision depth within any source knot span.
    pub max_depth: usize,
    /// Maximum number of cubic pieces in the returned curve.
    pub max_segments: usize,
}

impl CubicFitOptions {
    /// Creates explicit tolerances with finite work, depth, and segment limits.
    ///
    /// Depth and segment limits are also capped by the operation context.
    #[must_use]
    pub const fn new(position_tolerance: f64, derivative_tolerance: f64) -> Self {
        Self {
            position_tolerance,
            derivative_tolerance,
            max_work: 1_000_000,
            max_depth: 12,
            max_segments: 1024,
        }
    }
}

/// A disclosed approximate curve with bounds over the complete source domain.
#[derive(Debug, Clone)]
pub struct CubicFitOutcome {
    /// Piecewise cubic, non-rational approximation on the unchanged domain.
    pub curve: NurbsCurve,
    /// Conservative position difference from the original over its domain.
    pub position_bound: f64,
    /// Conservative first derivative difference per source parameter unit.
    pub derivative_bound: f64,
    /// Number of cubic pieces, including all source knot spans.
    pub segments: usize,
    /// Charged work used, never greater than the requested work limit.
    pub work_used: usize,
    /// Always true: invoking cubic fitting explicitly selects approximation.
    pub approximate: bool,
}

#[derive(Clone, Copy)]
struct PendingInterval {
    range: (f64, f64),
    depth: usize,
}

/// Approximates a clamped, positive-weight NURBS curve by certified cubic pieces.
///
/// Supports finite curves of degree 1 through 12 with exact monotonic knots and
/// a continuously differentiable interior (knot multiplicity below the degree).
/// Single-span lines and rational Bezier curves are included. Hermite endpoint
/// positions and tangents are interpolated subject to floating-point rounding;
/// certificates include that rounding and cover entire intervals, not samples.
/// Source knots are retained as piece boundaries and the parameter domain is
/// never normalized. The input remains unchanged on every success or failure.
///
/// The context must permit approximation. Its approximation budget caps the
/// position tolerance; its subdivision and segment limits cap the options.
/// The derivative tolerance is independent of the position budget.
///
/// # Errors
///
/// Returns typed errors for invalid options, unsupported source continuity,
/// unavailable finite certificates, cancellation, and exhausted work/segment
/// limits. If the tolerances cannot be certified within the depth limit, returns
/// [`ReuseError::ToleranceExceeded`] instead of returning a partial fit.
#[allow(clippy::too_many_lines)]
pub fn fit_cubic_curve(
    curve: &NurbsCurve,
    options: &CubicFitOptions,
    context: &OperationContext,
) -> Result<CubicFitOutcome, ReuseError> {
    let position_tolerance = effective_position_tolerance(options, context)?;
    let max_segments = options.max_segments.min(context.budgets.segments);
    let max_depth = options.max_depth.min(context.budgets.subdivision_depth);
    if max_segments == 0 {
        return Err(ReuseError::InvalidOptions {
            reason: "cubic fit segment limit must be positive",
        });
    }

    let mut budget = ReuseBudget::new(options.max_work, context)?;
    // Charge before input validation or any input-sized allocation.
    budget.spend(
        curve
            .knots()
            .len()
            .saturating_add(curve.control_points().len()),
    )?;
    validate_curve_domain(curve)?;
    validate_smooth_interior(curve)?;

    let degree = curve.degree();
    let knots = curve.knots();
    let mut pending = Vec::new();
    for pair in knots[degree..=curve.control_points().len()].windows(2) {
        if pair[0] < pair[1] {
            if pending.len() >= max_segments {
                return Err(ReuseError::SegmentLimit {
                    limit: max_segments,
                });
            }
            pending.push(PendingInterval {
                range: (pair[0], pair[1]),
                depth: 0,
            });
        }
    }
    pending.reverse();
    let mut accepted = Vec::new();
    while let Some(interval) = pending.pop() {
        budget.spend((degree + 1).saturating_mul(degree + 1).saturating_mul(2) + 32)?;
        let candidate = hermite_piece(curve, interval.range)?;
        let bounds = curve_difference_on_interval(curve, &candidate, interval.range, &mut budget)?;
        if bounds.position <= position_tolerance
            && bounds.derivative <= options.derivative_tolerance
        {
            accepted.push(candidate);
            continue;
        }
        if interval.depth >= max_depth {
            return Err(tolerance_error(
                bounds.position,
                bounds.derivative,
                position_tolerance,
                options.derivative_tolerance,
            ));
        }
        if accepted
            .len()
            .saturating_add(pending.len())
            .saturating_add(2)
            > max_segments
        {
            return Err(ReuseError::SegmentLimit {
                limit: max_segments,
            });
        }
        let midpoint = interval.range.0.midpoint(interval.range.1);
        if midpoint <= interval.range.0 || midpoint >= interval.range.1 {
            return Err(ReuseError::BoundUnavailable {
                reason: "cubic fit interval cannot be subdivided in floating point",
            });
        }
        let depth = interval.depth + 1;
        pending.push(PendingInterval {
            range: (midpoint, interval.range.1),
            depth,
        });
        pending.push(PendingInterval {
            range: (interval.range.0, midpoint),
            depth,
        });
    }

    budget.spend(accepted.len().saturating_mul(16))?;
    let segments = accepted.len();
    let fitted = concatenate_pieces(&accepted)?;
    // Certify the assembled representation too: acceptance applies to the
    // actual returned spline, including knot junctions and concatenation.
    let bounds = curve_difference_on_interval(curve, &fitted, curve.domain(), &mut budget)?;
    if bounds.position > position_tolerance || bounds.derivative > options.derivative_tolerance {
        return Err(tolerance_error(
            bounds.position,
            bounds.derivative,
            position_tolerance,
            options.derivative_tolerance,
        ));
    }
    Ok(CubicFitOutcome {
        curve: fitted,
        position_bound: bounds.position,
        derivative_bound: bounds.derivative,
        segments,
        work_used: budget.used(),
        approximate: true,
    })
}

fn effective_position_tolerance(
    options: &CubicFitOptions,
    context: &OperationContext,
) -> Result<f64, ReuseError> {
    if !options.position_tolerance.is_finite() || options.position_tolerance <= 0.0 {
        return Err(ReuseError::InvalidOptions {
            reason: "cubic fit position tolerance must be finite and positive",
        });
    }
    if !options.derivative_tolerance.is_finite() || options.derivative_tolerance <= 0.0 {
        return Err(ReuseError::InvalidOptions {
            reason: "cubic fit derivative tolerance must be finite and positive",
        });
    }
    let approximation_budget = context.fallback.budget().ok_or(ReuseError::Unsupported {
        reason: "cubic fitting requires a context that permits approximation",
    })?;
    if !approximation_budget.is_finite() || approximation_budget <= 0.0 {
        return Err(ReuseError::InvalidOptions {
            reason: "cubic fit context approximation budget must be finite and positive",
        });
    }
    Ok(options.position_tolerance.min(approximation_budget))
}

#[allow(clippy::float_cmp)] // Exact knot multiplicities are part of this supported domain.
fn validate_smooth_interior(curve: &NurbsCurve) -> Result<(), ReuseError> {
    let degree = curve.degree();
    let (start, end) = curve.domain();
    let mut previous = start;
    let mut multiplicity = 0;
    for &knot in curve.knots() {
        if knot <= start || knot >= end {
            continue;
        }
        if knot == previous {
            multiplicity += 1;
        } else {
            previous = knot;
            multiplicity = 1;
        }
        if multiplicity >= degree {
            return Err(ReuseError::Unsupported {
                reason: "cubic fitting requires continuously differentiable interior knots",
            });
        }
    }
    Ok(())
}

fn hermite_piece(curve: &NurbsCurve, range: (f64, f64)) -> Result<NurbsCurve, ReuseError> {
    let width = range.1 - range.0;
    if !width.is_finite() || width <= 0.0 {
        return Err(ReuseError::BoundUnavailable {
            reason: "cubic fit parameter interval has no finite positive width",
        });
    }
    let (start_point, start_derivative) = checked_endpoint(curve, range.0)?;
    let (end_point, end_derivative) = checked_endpoint(curve, range.1)?;
    let points = vec![
        start_point,
        start_point + start_derivative * (width / 3.0),
        end_point - end_derivative * (width / 3.0),
        end_point,
    ];
    if points
        .iter()
        .any(|point| !point.0.iter().all(|coordinate| coordinate.is_finite()))
    {
        return Err(ReuseError::BoundUnavailable {
            reason: "cubic fit endpoint position or derivative is not finite",
        });
    }
    let mut knots = vec![range.0; 4];
    knots.extend(std::iter::repeat_n(range.1, 4));
    Ok(NurbsCurve::new(3, knots, points, vec![1.0; 4])?)
}

/// The ordinary evaluator asserts that its rational denominator is finite and
/// positive. A finite valid source can still violate that numerical premise:
/// weight normalization may underflow, or a tiny knot span may overflow basis
/// reciprocals. Check first-order evaluation explicitly and refuse these cases
/// before any asserted division. The whole-interval certificate subsequently
/// qualifies the actual cubic constructed from these floating-point values.
fn checked_endpoint(curve: &NurbsCurve, parameter: f64) -> Result<(Point3, Vec3), ReuseError> {
    let degree = curve.degree();
    let stride = degree + 1;
    // The caller validates the certificate domain (degree 1 through 12).
    let mut coefficients = [0.0; 26];
    let used = 2 * stride;
    let span = basis::find_span(
        curve.control_points().len(),
        degree,
        parameter,
        curve.knots(),
    );
    basis::ders_basis_funs_into(
        span,
        parameter,
        degree,
        1,
        curve.knots(),
        &mut coefficients[..used],
    );
    if coefficients[..used].iter().any(|value| !value.is_finite()) {
        return Err(ReuseError::BoundUnavailable {
            reason: "cubic fit endpoint basis or derivative is not finite",
        });
    }

    let weight_scale = curve.max_weight();
    if !weight_scale.is_finite() || weight_scale <= 0.0 {
        return Err(ReuseError::BoundUnavailable {
            reason: "cubic fit endpoint weight scale is not finite and positive",
        });
    }
    let first = span - degree;
    let origin = curve.control_points()[first];
    let mut homogeneous_position = [0.0; 3];
    let mut homogeneous_derivative = [0.0; 3];
    let mut denominator = 0.0;
    let mut denominator_derivative = 0.0;
    for local in 0..stride {
        let index = first + local;
        let weight = curve.weights()[index] / weight_scale;
        if !weight.is_finite() || weight <= 0.0 {
            return Err(ReuseError::BoundUnavailable {
                reason: "cubic fit endpoint weight normalization lost strict positivity",
            });
        }
        let position_factor = coefficients[local] * weight;
        let derivative_factor = coefficients[stride + local] * weight;
        denominator += position_factor;
        denominator_derivative += derivative_factor;
        let offset = curve.control_points()[index] - origin;
        for axis in 0..3 {
            homogeneous_position[axis] += position_factor * offset.0[axis];
            homogeneous_derivative[axis] += derivative_factor * offset.0[axis];
        }
    }
    if !denominator.is_finite() || denominator <= 0.0 {
        return Err(ReuseError::BoundUnavailable {
            reason: "cubic fit endpoint denominator is not finite and positive",
        });
    }
    let mut position = [0.0; 3];
    let mut derivative = [0.0; 3];
    for axis in 0..3 {
        let relative = homogeneous_position[axis] / denominator;
        position[axis] = origin.0[axis] + relative;
        derivative[axis] =
            (homogeneous_derivative[axis] - relative * denominator_derivative) / denominator;
    }
    if position
        .iter()
        .chain(&derivative)
        .any(|value| !value.is_finite())
    {
        return Err(ReuseError::BoundUnavailable {
            reason: "cubic fit endpoint position or derivative cannot be represented finitely",
        });
    }
    Ok((
        Point3::new(position[0], position[1], position[2]),
        Vec3::new(derivative[0], derivative[1], derivative[2]),
    ))
}

fn concatenate_pieces(pieces: &[NurbsCurve]) -> Result<NurbsCurve, ReuseError> {
    let first = pieces.first().ok_or(ReuseError::BoundUnavailable {
        reason: "cubic fit has no nonempty source spans",
    })?;
    let mut knots = vec![first.domain().0; 4];
    let mut points = first.control_points().to_vec();
    for piece in &pieces[1..] {
        knots.extend(std::iter::repeat_n(piece.domain().0, 3));
        points.extend_from_slice(&piece.control_points()[1..]);
    }
    let end = pieces
        .last()
        .map_or_else(|| first.domain().1, |piece| piece.domain().1);
    knots.extend(std::iter::repeat_n(end, 4));
    let weights = vec![1.0; points.len()];
    Ok(NurbsCurve::new(3, knots, points, weights)?)
}

fn tolerance_error(
    position: f64,
    derivative: f64,
    position_tolerance: f64,
    derivative_tolerance: f64,
) -> ReuseError {
    if position > position_tolerance {
        ReuseError::ToleranceExceeded {
            bound: position,
            tolerance: position_tolerance,
        }
    } else {
        ReuseError::ToleranceExceeded {
            bound: derivative,
            tolerance: derivative_tolerance,
        }
    }
}

#[cfg(test)]
mod tests;
