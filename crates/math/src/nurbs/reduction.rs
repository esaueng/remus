//! Opt-in, whole-domain-qualified NURBS knot reduction.
//!
//! Modified for Remus; surface inverse insertion and descending optimization
//! strategies are adapted from Truck's `BSplineSurface::try_remove_uknot`,
//! `try_remove_vknot`, `optimize`, and `BSplineCurve::optimize`, pinned at
//! `88ed005249e5e3a6b07f62425399435905cd3ab6`, under Apache-2.0.
//! See `docs/production-readiness/truck-reuse-provenance.md` for attribution.
//! Remus's existing curve removal is used only to propose curve candidates.
//! Acceptance comes exclusively from independent whole-domain bounds.
//! Positive bounds require an approximation policy and must fit both its
//! budget and the requested tolerance. `ExactOnly` simplification may return
//! the original unchanged geometry with a zero bound.

use crate::context::OperationContext;
use crate::nurbs::curve::NurbsCurve;
use crate::nurbs::knot_ops::curve_knot_remove;
use crate::nurbs::reuse::{ReductionOptions, ReductionOutcome, ReuseBudget, ReuseError};
use crate::nurbs::reuse_bounds::{
    curve_deviation_bound, surface_deviation_bound, validate_curve_domain, validate_surface_domain,
};
use crate::nurbs::surface::NurbsSurface;
use crate::vec::Point3;

#[derive(Clone, Copy)]
enum Direction {
    U,
    V,
}

/// Remove one exact occurrence of an interior u knot, with a whole-domain bound.
///
/// The input remains immutable. Weights remain positive and the parameter
/// domain is preserved. The reported deviation includes numerical error and
/// is not an exact-geometry claim. No trim or topology is modified.
///
/// # Errors
///
/// Returns a typed refusal for unsupported domains, absent/end knots, an
/// unqualified candidate, invalid options/policy, cancellation, or work limits.
pub fn surface_knot_remove_u(
    surface: &NurbsSurface,
    knot: f64,
    options: &ReductionOptions,
    context: &OperationContext,
) -> Result<ReductionOutcome<NurbsSurface>, ReuseError> {
    remove_surface(surface, knot, Direction::U, options, context)
}

/// Remove one exact occurrence of an interior v knot, with a whole-domain bound.
///
/// See [`surface_knot_remove_u`] for the numerical and immutable-result contract.
///
/// # Errors
///
/// Returns a typed refusal for unsupported domains, absent/end knots, an
/// unqualified candidate, invalid options/policy, cancellation, or work limits.
pub fn surface_knot_remove_v(
    surface: &NurbsSurface,
    knot: f64,
    options: &ReductionOptions,
    context: &OperationContext,
) -> Result<ReductionOutcome<NurbsSurface>, ReuseError> {
    remove_surface(surface, knot, Direction::V, options, context)
}

/// Repeatedly remove curve interior knots in deterministic descending order.
///
/// Every accepted candidate is compared with the original input, so the
/// returned bound covers the complete sequence rather than its final step.
/// A result with `removed_knots == 0` is the original geometry and has zero
/// deviation. The operation is opt-in and does not modify topology or trims.
///
/// # Errors
///
/// Returns a typed error for unsupported domains, invalid options/policy,
/// unavailable numerical bounds, cancellation, or the aggregate work limit.
pub fn simplify_curve(
    curve: &NurbsCurve,
    options: &ReductionOptions,
    context: &OperationContext,
) -> Result<ReductionOutcome<NurbsCurve>, ReuseError> {
    options.validate()?;
    let tolerance = allowed_tolerance(options, context)?;
    let mut budget = ReuseBudget::new(options.max_work, context)?;
    budget.spend(
        curve
            .knots()
            .len()
            .saturating_add(curve.control_points().len())
            .saturating_add(curve.weights().len()),
    )?;
    validate_curve_domain(curve)?;
    let mut current = curve.clone();
    let mut removed = 0;
    let mut deviation = 0.0;
    loop {
        budget.spend(current.knots().len())?;
        let knots = current.knots().to_vec();
        let before = removed;
        for &knot in knots[current.degree() + 1..knots.len() - current.degree() - 1]
            .iter()
            .rev()
        {
            budget.spend(1)?;
            let candidate = match curve_candidate(&current, knot, &mut budget) {
                Ok(candidate) => candidate,
                Err(ReuseError::KnotNotRemovable) => continue,
                Err(error) => return Err(error),
            };
            let bound = curve_deviation_bound(curve, &candidate, &mut budget)?;
            if bound > tolerance {
                continue;
            }
            current = candidate;
            deviation = bound;
            removed += 1;
        }
        if removed == before {
            break;
        }
    }
    Ok(ReductionOutcome {
        geometry: current,
        removed_knots: removed,
        deviation_bound: deviation,
        work_used: budget.used(),
    })
}

/// Repeatedly remove u then v interior knots in descending order.
///
/// All candidates are bounded against the original surface over its whole
/// rectangular parameter domain. No boundary representation is reconstructed:
/// callers remain responsible for any trims and associated pcurves.
/// Unchanged results have `removed_knots == 0` and a zero deviation bound.
///
/// # Errors
///
/// Returns a typed error for unsupported domains, invalid options/policy,
/// unavailable numerical bounds, cancellation, or the aggregate work limit.
pub fn simplify_surface(
    surface: &NurbsSurface,
    options: &ReductionOptions,
    context: &OperationContext,
) -> Result<ReductionOutcome<NurbsSurface>, ReuseError> {
    options.validate()?;
    let tolerance = allowed_tolerance(options, context)?;
    let mut budget = ReuseBudget::new(options.max_work, context)?;
    charge_surface_validation(surface, &mut budget)?;
    validate_surface_domain(surface)?;
    let mut current = surface.clone();
    let mut removed = 0;
    let mut deviation = 0.0;
    loop {
        let before = removed;
        for direction in [Direction::U, Direction::V] {
            let (knots, degree) = direction_data(&current, direction);
            budget.spend(knots.len())?;
            let snapshot = knots.to_vec();
            for &knot in snapshot[degree + 1..snapshot.len() - degree - 1]
                .iter()
                .rev()
            {
                budget.spend(1)?;
                let candidate = match surface_candidate(&current, knot, direction, &mut budget) {
                    Ok(candidate) => candidate,
                    Err(ReuseError::KnotNotRemovable) => continue,
                    Err(error) => return Err(error),
                };
                let bound = surface_deviation_bound(surface, &candidate, &mut budget)?;
                if bound > tolerance {
                    continue;
                }
                current = candidate;
                deviation = bound;
                removed += 1;
            }
        }
        if before == removed {
            break;
        }
    }
    Ok(ReductionOutcome {
        geometry: current,
        removed_knots: removed,
        deviation_bound: deviation,
        work_used: budget.used(),
    })
}

fn remove_surface(
    surface: &NurbsSurface,
    knot: f64,
    direction: Direction,
    options: &ReductionOptions,
    context: &OperationContext,
) -> Result<ReductionOutcome<NurbsSurface>, ReuseError> {
    options.validate()?;
    let tolerance = allowed_tolerance(options, context)?;
    let mut budget = ReuseBudget::new(options.max_work, context)?;
    charge_surface_validation(surface, &mut budget)?;
    validate_surface_domain(surface)?;
    let candidate = surface_candidate(surface, knot, direction, &mut budget)?;
    let bound = surface_deviation_bound(surface, &candidate, &mut budget)?;
    if bound > tolerance {
        return Err(ReuseError::ToleranceExceeded { bound, tolerance });
    }
    Ok(ReductionOutcome {
        geometry: candidate,
        removed_knots: 1,
        deviation_bound: bound,
        work_used: budget.used(),
    })
}

fn allowed_tolerance(
    options: &ReductionOptions,
    context: &OperationContext,
) -> Result<f64, ReuseError> {
    match context.fallback.budget() {
        None => Ok(0.0),
        Some(budget) if budget.is_finite() && budget > 0.0 => Ok(options.tolerance.min(budget)),
        Some(_) => Err(ReuseError::InvalidOptions {
            reason: "context approximation budget must be finite and positive",
        }),
    }
}

fn direction_data(surface: &NurbsSurface, direction: Direction) -> (&[f64], usize) {
    match direction {
        Direction::U => (surface.knots_u(), surface.degree_u()),
        Direction::V => (surface.knots_v(), surface.degree_v()),
    }
}

fn charge_surface_validation(
    surface: &NurbsSurface,
    budget: &mut ReuseBudget<'_>,
) -> Result<(), ReuseError> {
    budget.spend(surface.control_points().len())?;
    budget.spend(surface.weights().len())?;
    budget.spend(surface.knots_u().len())?;
    budget.spend(surface.knots_v().len())?;
    for row in surface.control_points() {
        budget.spend(row.len())?;
    }
    for row in surface.weights() {
        budget.spend(row.len())?;
    }
    Ok(())
}

#[allow(clippy::float_cmp)] // Knot identity is structural, never tolerance equality.
fn removable_index(knots: &[f64], degree: usize, knot: f64) -> Result<usize, ReuseError> {
    let count = knots.len() - degree - 1;
    if !knot.is_finite() || knot <= knots[degree] || knot >= knots[count] {
        return Err(ReuseError::KnotNotRemovable);
    }
    // Truck's forward inverse recurrence removes the first occurrence.
    // Choosing a later repeated knot introduces a zero insertion coefficient
    // before the affected control sequence has been reconstructed.
    knots
        .iter()
        .position(|&value| value == knot)
        .filter(|&index| index > degree && index < count)
        .ok_or(ReuseError::KnotNotRemovable)
}

// This is Truck's direction-wise inverse insertion over rational homogeneous
// control points. Its pointwise `near` terminal check is deliberately absent:
// the independent certificate, not a control-net comparison, accepts a result.
#[allow(clippy::too_many_lines)]
fn surface_candidate(
    surface: &NurbsSurface,
    knot: f64,
    direction: Direction,
    budget: &mut ReuseBudget<'_>,
) -> Result<NurbsSurface, ReuseError> {
    let (knots, degree) = direction_data(surface, direction);
    budget.spend(knots.len())?;
    let index = removable_index(knots, degree, knot)?;
    let rows = surface.control_points().len();
    let cols = surface.control_points()[0].len();
    let cells = rows.saturating_mul(cols);
    budget.spend(cells.saturating_mul(degree.saturating_add(5)))?;
    let anchor = surface.control_points()[0][0];
    let max_weight = surface
        .weights()
        .iter()
        .flat_map(|row| row.iter())
        .copied()
        .fold(0.0_f64, f64::max);
    let mut homogeneous = Vec::with_capacity(rows);
    for (points, weights) in surface.control_points().iter().zip(surface.weights()) {
        budget.spend(0)?;
        let mut row = Vec::with_capacity(cols);
        for (&point, &weight) in points.iter().zip(weights) {
            row.push(to_homogeneous(point, weight / max_weight, anchor)?);
        }
        homogeneous.push(row);
    }

    let transverse = match direction {
        Direction::U => cols,
        Direction::V => rows,
    };
    // Inverse insertion propagates the p affected homogeneous controls from
    // their known predecessor. The original end control is then removed.
    for transverse_index in 0..transverse {
        budget.spend(degree)?;
        let mut previous = read_homogeneous(
            &homogeneous,
            direction,
            index - degree - 1,
            transverse_index,
        );
        for position in index - degree..index {
            let denominator = knots[position + degree + 1] - knots[position];
            let alpha = (knot - knots[position]) / denominator;
            if !alpha.is_finite() || alpha <= 0.0 || alpha > 1.0 {
                return Err(ReuseError::KnotNotRemovable);
            }
            let point = read_homogeneous(&homogeneous, direction, position, transverse_index);
            let next = std::array::from_fn(|component| {
                previous[component] + (point[component] - previous[component]) / alpha
            });
            if next.iter().any(|component| !component.is_finite()) || next[3] <= 0.0 {
                return Err(ReuseError::KnotNotRemovable);
            }
            write_homogeneous(
                &mut homogeneous,
                direction,
                position,
                transverse_index,
                next,
            );
            previous = next;
        }
    }
    match direction {
        Direction::U => {
            homogeneous.remove(index);
        }
        Direction::V => {
            for row in &mut homogeneous {
                row.remove(index);
            }
        }
    }

    budget.spend(cells.saturating_add(knots.len()))?;
    let mut points = Vec::with_capacity(homogeneous.len());
    let mut weights = Vec::with_capacity(homogeneous.len());
    for row in homogeneous {
        budget.spend(0)?;
        let mut point_row = Vec::with_capacity(row.len());
        let mut weight_row = Vec::with_capacity(row.len());
        for point in row {
            point_row.push(from_homogeneous(point, anchor)?);
            weight_row.push(point[3]);
        }
        points.push(point_row);
        weights.push(weight_row);
    }
    let mut remaining_knots = knots.to_vec();
    remaining_knots.remove(index);
    let (knots_u, knots_v) = match direction {
        Direction::U => (remaining_knots, surface.knots_v().to_vec()),
        Direction::V => (surface.knots_u().to_vec(), remaining_knots),
    };
    Ok(NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        knots_u,
        knots_v,
        points,
        weights,
    )?)
}

fn read_homogeneous(
    grid: &[Vec<[f64; 4]>],
    direction: Direction,
    position: usize,
    transverse: usize,
) -> [f64; 4] {
    match direction {
        Direction::U => grid[position][transverse],
        Direction::V => grid[transverse][position],
    }
}

fn write_homogeneous(
    grid: &mut [Vec<[f64; 4]>],
    direction: Direction,
    position: usize,
    transverse: usize,
    value: [f64; 4],
) {
    match direction {
        Direction::U => grid[position][transverse] = value,
        Direction::V => grid[transverse][position] = value,
    }
}

fn to_homogeneous(point: Point3, weight: f64, anchor: Point3) -> Result<[f64; 4], ReuseError> {
    let homogeneous = [
        (point.x() - anchor.x()) * weight,
        (point.y() - anchor.y()) * weight,
        (point.z() - anchor.z()) * weight,
        weight,
    ];
    if weight <= 0.0 || homogeneous.iter().any(|value| !value.is_finite()) {
        return Err(ReuseError::BoundUnavailable {
            reason: "homogeneous normalization is not finite and positive",
        });
    }
    Ok(homogeneous)
}

fn from_homogeneous(point: [f64; 4], anchor: Point3) -> Result<Point3, ReuseError> {
    let restored = Point3::new(
        point[0] / point[3] + anchor.x(),
        point[1] / point[3] + anchor.y(),
        point[2] / point[3] + anchor.z(),
    );
    if point[3] <= 0.0
        || [restored.x(), restored.y(), restored.z()]
            .iter()
            .any(|v| !v.is_finite())
    {
        return Err(ReuseError::KnotNotRemovable);
    }
    Ok(restored)
}

fn curve_candidate(
    curve: &NurbsCurve,
    knot: f64,
    budget: &mut ReuseBudget<'_>,
) -> Result<NurbsCurve, ReuseError> {
    budget.spend(curve.knots().len())?;
    let index = removable_index(curve.knots(), curve.degree(), knot)?;
    budget.spend(
        curve
            .control_points()
            .len()
            .saturating_mul(curve.degree().saturating_add(10))
            .saturating_add(curve.knots().len().saturating_mul(3)),
    )?;
    let (start, end) = curve.domain();
    let width = end - start;
    if !width.is_finite() || width <= 0.0 {
        return Err(ReuseError::BoundUnavailable {
            reason: "curve domain cannot be normalized",
        });
    }
    let normalized_knots: Vec<_> = curve
        .knots()
        .iter()
        .map(|value| (value - start) / width)
        .collect();
    // Existing Remus removal has a fixed 1e-15 knot threshold. Refuse spans
    // that it cannot distinguish; acceptance never merges near knot values.
    if normalized_knots
        .windows(2)
        .zip(curve.knots().windows(2))
        .any(|(normalized, original)| {
            original[1] > original[0] && normalized[1] - normalized[0] <= 1e-15
        })
    {
        return Err(ReuseError::Unsupported {
            reason: "curve knot spacing is too small for the existing candidate generator",
        });
    }
    let anchor = curve.control_points()[0];
    let normalized_weights: Vec<_> = curve
        .weights()
        .iter()
        .map(|weight| weight / curve.max_weight())
        .collect();
    let points: Vec<_> = curve
        .control_points()
        .iter()
        .map(|&point| {
            Point3::new(
                point.x() - anchor.x(),
                point.y() - anchor.y(),
                point.z() - anchor.z(),
            )
        })
        .collect();
    let normalized = NurbsCurve::new(curve.degree(), normalized_knots, points, normalized_weights)?;
    // Existing internal Remus implementation proposes a candidate only. Its
    // local comparison is not a geometric certificate and does not accept it.
    let candidate = match curve_knot_remove(&normalized, (knot - start) / width, f64::MAX) {
        Ok(candidate) if candidate.control_points().len() + 1 == curve.control_points().len() => {
            candidate
        }
        _ => return Err(ReuseError::KnotNotRemovable),
    };
    let mut knots = curve.knots().to_vec();
    knots.remove(index);
    let points = candidate
        .control_points()
        .iter()
        .map(|point| {
            Point3::new(
                point.x() + anchor.x(),
                point.y() + anchor.y(),
                point.z() + anchor.z(),
            )
        })
        .collect();
    Ok(NurbsCurve::new(
        curve.degree(),
        knots,
        points,
        candidate.weights().to_vec(),
    )?)
}

#[cfg(test)]
#[path = "reduction/tests.rs"]
mod tests;
