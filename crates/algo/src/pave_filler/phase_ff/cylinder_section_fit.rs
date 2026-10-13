//! Certified local refit of an open, regular cylinder-cylinder section.
//!
//! The caller qualifies two full primitive cylinder faces. This component
//! accepts one monotone angular branch no longer than a quarter turn. Its
//! stored cubic Bezier controls interpolate the supplied boundary anchors;
//! outward-rounded whole-span bounds certify both original cylinder carriers
//! and, when supplied, both finite axial intervals within the reserved
//! operation tolerance. Unresolved bounds refine under the caller's work policy
//! or return a typed refusal.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::context::OperationContext;
use remus_math::fma::FusedMulAdd;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};

use crate::error::AlgoError;
use crate::pave_filler::curved_section_clip::{
    bezier_cylinder_angular_direction, bezier_cylinder_finite_distance_bound,
    bezier_cylinder_residual_bound, point_cylinder_cap_distance_bound,
};

const MAX_POINTS: usize = 2048;
const MAX_DEPTH: usize = 12;

fn unresolved(reason: &str) -> AlgoError {
    AlgoError::IntersectionFailed(format!("cylinder section refit unresolved: {reason}"))
}

fn finite_point(p: Point3) -> bool {
    p.0.iter().all(|x| x.is_finite())
}

#[derive(Clone, Copy)]
struct Station {
    fraction: f64,
    u: f64,
    point: Point3,
    tangent: Vec3,
}

#[derive(Clone, Copy)]
struct Branch<'a> {
    first: &'a CylindricalSurface,
    second: &'a CylindricalSurface,
    projected_axis: Vec3,
    axis_squared: f64,
    quadratic: f64,
    index: usize,
}

impl Branch<'_> {
    fn roots(&self, u: f64) -> Option<([f64; 2], Vec3, Vec3)> {
        let (sin, cos) = u.sin_cos();
        let radial = self.first.x_axis() * (self.first.radius() * cos)
            + self.first.y_axis() * (self.first.radius() * sin);
        let derivative = self.first.x_axis() * (-self.first.radius() * sin)
            + self.first.y_axis() * (self.first.radius() * cos);
        let q = self.first.origin() + radial - self.second.origin();
        let axis = self.second.axis();
        let q = q - axis * (q.dot(axis) / self.axis_squared);
        let derivative = derivative - axis * (derivative.dot(axis) / self.axis_squared);
        let b = 2.0 * q.dot(self.projected_axis);
        let c = q.dot(q) - self.second.radius() * self.second.radius();
        let discriminant = b.fma(b, -4.0 * self.quadratic * c);
        if !discriminant.is_finite() || discriminant <= 0.0 {
            return None;
        }
        let root = discriminant.sqrt();
        // Cancellation-free root pair; no tolerance clamp of an absent branch.
        let q_root = -0.5 * (b + root.copysign(b));
        if q_root == 0.0 || !q_root.is_finite() {
            return None;
        }
        let (a, b_root) = (q_root / self.quadratic, c / q_root);
        if !a.is_finite() || !b_root.is_finite() {
            return None;
        }
        Some(([a.min(b_root), a.max(b_root)], q, derivative))
    }

    fn station(&self, u: f64, fraction: f64) -> Option<Station> {
        let (roots, q, derivative) = self.roots(u)?;
        let v = roots[self.index];
        let b = 2.0 * q.dot(self.projected_axis);
        let denominator = (2.0 * self.quadratic).fma(v, b);
        if denominator == 0.0 || !denominator.is_finite() {
            return None;
        }
        let dv = -2.0 * derivative.dot(q + self.projected_axis * v) / denominator;
        let (sin, cos) = u.sin_cos();
        let tangent = self.first.x_axis() * (-self.first.radius() * sin)
            + self.first.y_axis() * (self.first.radius() * cos)
            + self.first.axis() * dv;
        let point = self.first.evaluate(u, v);
        (finite_point(point) && tangent.0.iter().all(|x| x.is_finite())).then_some(Station {
            fraction,
            u,
            point,
            tangent,
        })
    }

    fn nearest_branch(&self, u: f64, point: Point3, tolerance: f64) -> Option<usize> {
        let (roots, _, _) = self.roots(u)?;
        let distances = roots.map(|v| (self.first.evaluate(u, v) - point).length());
        if distances.iter().any(|x| !x.is_finite())
            || (distances[0] - distances[1]).abs() <= tolerance
        {
            return None;
        }
        Some(usize::from(distances[1] < distances[0]))
    }
}

fn controls(a: Station, b: Station) -> [Point3; 4] {
    let step = (b.u - a.u) / 3.0;
    [
        a.point,
        a.point + a.tangent * step,
        b.point - b.tangent * step,
        b.point,
    ]
}

fn certified(
    controls: &[Point3; 4],
    branch: &Branch<'_>,
    tolerance: f64,
    step: f64,
    limits: Option<[(f64, f64); 2]>,
) -> bool {
    let weights = [1.0; 4];
    let Some((first, _)) =
        bezier_cylinder_residual_bound(controls, &weights, branch.first, branch.first.axis())
    else {
        return false;
    };
    let Some((second, sign)) =
        bezier_cylinder_residual_bound(controls, &weights, branch.second, branch.first.axis())
    else {
        return false;
    };
    let Some(angular) = bezier_cylinder_angular_direction(controls, &weights, branch.first) else {
        return false;
    };
    // Reserve half the caller band; all bounds include arithmetic roundoff.
    // A fixed sign of the axial normal projection also certifies that the
    // entire fitted piece stays on the selected regular quadratic branch.
    first <= tolerance * 0.5
        && second <= tolerance * 0.5
        && finite_faces_bound(controls, &weights, branch, limits, tolerance)
        && if step > 0.0 {
            angular[0] > 0.0
        } else {
            angular[1] < 0.0
        }
        && if branch.index == 0 {
            sign[1] < 0.0
        } else {
            sign[0] > 0.0
        }
}

fn finite_faces_bound(
    controls: &[Point3],
    weights: &[f64],
    branch: &Branch<'_>,
    limits: Option<[(f64, f64); 2]>,
    tolerance: f64,
) -> bool {
    limits.is_none_or(|limits| {
        [branch.first, branch.second]
            .into_iter()
            .zip(limits)
            .all(|(cylinder, limits)| {
                bezier_cylinder_finite_distance_bound(controls, weights, cylinder, limits)
                    .is_some_and(|bound| bound <= tolerance * 0.5)
            })
    })
}

fn same_point_bits(a: Point3, b: Point3) -> bool {
    a.0.into_iter()
        .zip(b.0)
        .all(|(a, b)| a.to_bits() == b.to_bits())
}

fn point_on_carriers(
    point: Point3,
    first: &CylindricalSurface,
    second: &CylindricalSurface,
    tolerance: f64,
) -> bool {
    [first, second].into_iter().all(|cylinder| {
        bezier_cylinder_residual_bound(&[point, point], &[1.0; 2], cylinder, cylinder.axis())
            .is_some_and(|(bound, _)| bound <= tolerance)
    })
}

/// An untrusted earlier pave endpoint becomes a boundary junction only after
/// independent whole-point carrier/cap bounds, finite-face extents, branch and
/// local angular identity all qualify. Distinct qualifying points refuse.
#[allow(clippy::too_many_arguments)]
fn recover_anchors(
    branch: &Branch<'_>,
    guide: &NurbsCurve,
    range: (f64, f64),
    anchors: [Point3; 2],
    limits: Option<[(f64, f64); 2]>,
    early_candidates: &[Point3],
    candidates: &[Point3],
    context: &OperationContext,
) -> Result<[Point3; 2], AlgoError> {
    let tolerance = context.tolerance.linear;
    let mut result = anchors;
    for index in 0..2 {
        context.check_cancelled()?;
        let anchor = anchors[index];
        let u = branch.first.project_point(anchor).0;
        let Some(branch_index) = branch.nearest_branch(u, anchor, tolerance) else {
            continue;
        };
        let station_branch = Branch {
            index: branch_index,
            ..*branch
        };
        if point_on_carriers(anchor, branch.first, branch.second, tolerance * 0.125)
            && station_branch
                .station(u, 0.0)
                .is_some_and(|s| (s.point - anchor).length() <= tolerance)
        {
            continue;
        }
        let Some(limits) = limits else {
            continue;
        };
        let surfaces = [branch.first, branch.second];
        let mut caps = Vec::new();
        for owner in 0..2 {
            for cap in [limits[owner].0, limits[owner].1] {
                if point_cylinder_cap_distance_bound(anchor, surfaces[owner], cap)
                    .is_some_and(|distance| distance <= tolerance)
                {
                    caps.push((owner, cap));
                }
            }
        }
        let [(owner, cap)] = caps.as_slice() else {
            return Err(unresolved("untrusted endpoint has no unique finite cap"));
        };
        let fraction = if index == 0 { 0.125 } else { 0.875 };
        let point = guide.evaluate(range.0 + (range.1 - range.0) * fraction);
        let adjacent = branch.first.project_point(point).0;
        let radius = ((adjacent - u + PI).rem_euclid(TAU) - PI)
            .abs()
            .min(FRAC_PI_2 * 0.5);
        if !radius.is_finite() || radius <= 0.0 {
            return Err(unresolved("endpoint guide has no local angular window"));
        }
        let mut selected = None;
        for pool in [early_candidates, candidates] {
            for &candidate in pool {
                context.check_cancelled()?;
                if !finite_point(candidate)
                    || !point_on_carriers(candidate, branch.first, branch.second, tolerance * 0.125)
                    || point_cylinder_cap_distance_bound(candidate, surfaces[*owner], *cap)
                        .is_none_or(|distance| distance > tolerance * 0.125)
                {
                    continue;
                }
                let candidate_u = branch.first.project_point(candidate).0;
                let angular = (candidate_u - u + PI).rem_euclid(TAU) - PI;
                if !angular.is_finite()
                    || angular.abs() > radius
                    || station_branch.nearest_branch(candidate_u, candidate, tolerance)
                        != Some(branch_index)
                {
                    continue;
                }
                if (0..2).any(|owner| {
                    let v = surfaces[owner].project_point(candidate).1;
                    !v.is_finite()
                        || v < limits[owner].0 - tolerance * 0.125
                        || v > limits[owner].1 + tolerance * 0.125
                }) {
                    continue;
                }
                if selected.is_some_and(|previous| !same_point_bits(previous, candidate)) {
                    return Err(unresolved("distinct boundary junction candidates remain"));
                }
                selected = Some(candidate);
            }
            if selected.is_some() {
                break;
            }
        }
        result[index] = selected
            .ok_or_else(|| unresolved("no independently certified boundary junction candidate"))?;
    }
    Ok(result)
}

fn guide_has_branch_identity(
    branch: &Branch<'_>,
    guide: &NurbsCurve,
    range: (f64, f64),
    u0: f64,
    delta: f64,
    context: &OperationContext,
) -> Result<bool, AlgoError> {
    let mut previous = u0;
    for i in 1..8 {
        context.check_cancelled()?;
        let fraction = f64::from(i) / 8.0;
        let point = guide.evaluate(range.0 + (range.1 - range.0) * fraction);
        let u = branch.first.project_point(point).0;
        let near = u0 + delta * fraction;
        let u = near + (u - near + PI).rem_euclid(TAU) - PI;
        if !finite_point(point)
            || (u - previous) * delta <= 0.0
            || (u0 + delta - u) * delta <= 0.0
            || branch.nearest_branch(u, point, context.tolerance.linear) != Some(branch.index)
        {
            return Ok(false);
        }
        previous = u;
    }
    Ok(true)
}

/// Gate early cap-event work to an eligible regular quarter-branch section
/// whose supplied anchor actually lacks an independent carrier certificate.
#[allow(clippy::float_cmp)]
pub(super) fn needs_anchor_recovery(
    first: &CylindricalSurface,
    second: &CylindricalSurface,
    guide: &NurbsCurve,
    range: (f64, f64),
    anchors: [Point3; 2],
    context: &OperationContext,
) -> Result<bool, AlgoError> {
    context.check_cancelled()?;
    let tolerance = context.tolerance.linear;
    let domain = guide.domain();
    if !tolerance.is_finite()
        || tolerance <= 0.0
        || context.budgets.segments == 0
        || !anchors.into_iter().all(finite_point)
        || ![range.0, range.1]
            .into_iter()
            .all(|t| t.is_finite() && t >= domain.0 && t <= domain.1)
        || range.0 == range.1
        || (anchors[1] - anchors[0]).length() <= tolerance
    {
        return Ok(false);
    }
    let u0 = first.project_point(anchors[0]).0;
    let delta = (first.project_point(anchors[1]).0 - u0 + PI).rem_euclid(TAU) - PI;
    if !delta.is_finite() || delta == 0.0 || delta.abs() > FRAC_PI_2 {
        return Ok(false);
    }
    let axis_squared = second.axis().dot(second.axis());
    if !axis_squared.is_finite() || axis_squared <= 0.0 {
        return Ok(false);
    }
    let projected_axis =
        first.axis() - second.axis() * (first.axis().dot(second.axis()) / axis_squared);
    let quadratic = projected_axis.dot(projected_axis);
    if !quadratic.is_finite() || quadratic <= 1e-12 {
        return Ok(false);
    }
    let mut branch = Branch {
        first,
        second,
        projected_axis,
        axis_squared,
        quadratic,
        index: 0,
    };
    let Some(index) = branch.nearest_branch(u0, anchors[0], tolerance) else {
        return Ok(false);
    };
    if branch.nearest_branch(u0 + delta, anchors[1], tolerance) != Some(index) {
        return Ok(false);
    }
    branch.index = index;
    if branch.station(u0, 0.0).is_none()
        || branch.station(u0 + delta, 1.0).is_none()
        || !guide_has_branch_identity(&branch, guide, range, u0, delta, context)?
    {
        return Ok(false);
    }
    Ok(anchors.into_iter().any(|anchor| {
        !point_on_carriers(anchor, first, second, tolerance * 0.125)
            || branch
                .station(first.project_point(anchor).0, 0.0)
                .is_none_or(|s| (s.point - anchor).length() > tolerance)
    }))
}

/// `None` declines the declared angular/regular-branch domain. Once a branch
/// is qualified, failed certificates or exhausted work refuse explicitly.
// Exact equality detects a zero parameter span or a stalled floating-point bisection.
#[allow(clippy::float_cmp, clippy::too_many_arguments)]
pub(super) fn fit_open_cylinder_section(
    first: &CylindricalSurface,
    second: &CylindricalSurface,
    guide: &NurbsCurve,
    guide_range: (f64, f64),
    anchors: [Point3; 2],
    limits: Option<[(f64, f64); 2]>,
    early_candidates: &[Point3],
    candidates: &[Point3],
    context: &OperationContext,
) -> Result<Option<NurbsCurve>, AlgoError> {
    context.check_cancelled()?;
    let tolerance = context.tolerance.linear;
    let domain = guide.domain();
    if !tolerance.is_finite()
        || tolerance <= 0.0
        || !anchors.into_iter().all(finite_point)
        || ![guide_range.0, guide_range.1]
            .into_iter()
            .all(|t| t.is_finite() && t >= domain.0 && t <= domain.1)
        || guide_range.0 == guide_range.1
        || (anchors[1] - anchors[0]).length() <= tolerance
    {
        return Ok(None);
    }
    let axis = second.axis();
    let axis_squared = axis.dot(axis);
    if !axis_squared.is_finite() || axis_squared <= 0.0 {
        return Ok(None);
    }
    let projected_axis = first.axis() - axis * (first.axis().dot(axis) / axis_squared);
    let quadratic = projected_axis.dot(projected_axis);
    if !quadratic.is_finite() || quadratic <= 1e-12 {
        return Ok(None);
    }
    let mut branch = Branch {
        first,
        second,
        projected_axis,
        axis_squared,
        quadratic,
        index: 0,
    };
    // Decline unsupported guide domains before any endpoint recovery work.
    let old_u0 = first.project_point(anchors[0]).0;
    let old_delta = (first.project_point(anchors[1]).0 - old_u0 + PI).rem_euclid(TAU) - PI;
    if !old_delta.is_finite() || old_delta == 0.0 || old_delta.abs() > FRAC_PI_2 {
        return Ok(None);
    }
    let Some(old_index) = branch.nearest_branch(old_u0, anchors[0], tolerance) else {
        return Ok(None);
    };
    if branch.nearest_branch(old_u0 + old_delta, anchors[1], tolerance) != Some(old_index)
        || branch.station(old_u0, 0.0).is_none()
        || branch.station(old_u0 + old_delta, 1.0).is_none()
    {
        return Ok(None);
    }
    branch.index = old_index;
    if !guide_has_branch_identity(&branch, guide, guide_range, old_u0, old_delta, context)? {
        return Ok(None);
    }
    let anchors = recover_anchors(
        &branch,
        guide,
        guide_range,
        anchors,
        limits,
        early_candidates,
        candidates,
        context,
    )?;
    let u0 = first.project_point(anchors[0]).0;
    let raw_end = first.project_point(anchors[1]).0;
    let delta = (raw_end - u0 + PI).rem_euclid(TAU) - PI;
    if !delta.is_finite() || delta == 0.0 || delta.abs() > FRAC_PI_2 {
        return Ok(None);
    }
    let u1 = u0 + delta;
    let Some(start_index) = branch.nearest_branch(u0, anchors[0], tolerance) else {
        return Ok(None);
    };
    let Some(end_index) = branch.nearest_branch(u1, anchors[1], tolerance) else {
        return Ok(None);
    };
    if start_index != end_index {
        return Ok(None);
    }
    branch.index = start_index;
    let Some(mut start) = branch.station(u0, 0.0) else {
        return Ok(None);
    };
    let Some(mut end) = branch.station(u1, 1.0) else {
        return Ok(None);
    };
    if (start.point - anchors[0]).length() > tolerance
        || (end.point - anchors[1]).length() > tolerance
    {
        return Err(unresolved(
            "boundary anchor is not on the selected analytic branch",
        ));
    }
    // The legacy fitted guide chooses identity only. It never supplies final
    // geometry, endpoint coordinates, or a residual acceptance certificate.
    if !guide_has_branch_identity(&branch, guide, guide_range, u0, delta, context)? {
        return Ok(None);
    }
    start.point = anchors[0];
    end.point = anchors[1];
    // One returned component consumes one segment budget. Bezier pieces
    // are refinement cells, not separately traced intersection components.
    if context.budgets.segments == 0 {
        return Err(AlgoError::ResourceLimitExceeded {
            resource: "cylinder section fit components",
            limit: 0,
            actual: 1,
        });
    }
    // Partition the selected analytic angular span into eight initial subarcs.
    // The guide probes choose identity only; they do not certify its shape.
    // Build stations from the analytic branch, never from guide points.
    // The borrowed context depth bounds recursion inside each initial subarc;
    // the global station cap independently bounds the complete returned fit.
    let mut stations = Vec::with_capacity(9);
    stations.push(start);
    for i in 1..8 {
        context.check_cancelled()?;
        let fraction = f64::from(i) / 8.0;
        let Some(station) = branch.station(u0 + delta * fraction, fraction) else {
            return Err(unresolved(
                "branch is singular or absent in an initial subarc",
            ));
        };
        stations.push(station);
    }
    stations.push(end);
    let depth_limit = context.budgets.subdivision_depth.min(MAX_DEPTH);
    for depth in 0..=depth_limit {
        context.check_cancelled()?;
        let mut refine = Vec::new();
        for (index, pair) in stations.windows(2).enumerate() {
            context.check_cancelled()?;
            // Refinement shrinks a control-hull overestimate. It cannot repair
            // an analytic branch station already outside either finite face.
            for station in pair {
                if !finite_faces_bound(
                    &[station.point, station.point],
                    &[1.0; 2],
                    &branch,
                    limits,
                    tolerance,
                ) {
                    return Err(unresolved(
                        "selected branch leaves a finite face or its point bound is unresolved",
                    ));
                }
            }
            if !certified(
                &controls(pair[0], pair[1]),
                &branch,
                tolerance,
                pair[1].u - pair[0].u,
                limits,
            ) {
                refine.push(index);
            }
        }
        if refine.is_empty() {
            let mut points = Vec::with_capacity(3 * (stations.len() - 1) + 1);
            let mut knots = vec![0.0; 4];
            for (index, pair) in stations.windows(2).enumerate() {
                let span = controls(pair[0], pair[1]);
                if index == 0 {
                    points.push(span[0]);
                }
                points.extend_from_slice(&span[1..]);
                if index + 2 < stations.len() {
                    knots.extend(std::iter::repeat_n(pair[1].fraction, 3));
                }
            }
            knots.extend([1.0; 4]);
            let weights = vec![1.0; points.len()];
            return Ok(Some(NurbsCurve::new(3, knots, points, weights)?));
        }
        if depth == depth_limit {
            return Err(AlgoError::ResourceLimitExceeded {
                resource: "cylinder section fit refinement depth",
                limit: depth_limit,
                actual: depth + 1,
            });
        }
        let required = stations.len().saturating_add(refine.len());
        if required > MAX_POINTS {
            return Err(AlgoError::ResourceLimitExceeded {
                resource: "cylinder section fit stations",
                limit: MAX_POINTS,
                actual: required,
            });
        }
        let mut next = Vec::with_capacity(required);
        let mut selected = refine.into_iter().peekable();
        for (index, pair) in stations.windows(2).enumerate() {
            next.push(pair[0]);
            if selected.peek() == Some(&index) {
                selected.next();
                let u = f64::midpoint(pair[0].u, pair[1].u);
                let fraction = f64::midpoint(pair[0].fraction, pair[1].fraction);
                if u == pair[0].u || u == pair[1].u {
                    return Err(unresolved("parameter bisection stalled"));
                }
                let Some(midpoint) = branch.station(u, fraction) else {
                    return Err(unresolved(
                        "branch is singular or absent inside the section",
                    ));
                };
                next.push(midpoint);
            }
        }
        next.push(end);
        stations = next;
    }
    Err(unresolved("refinement did not produce a certified curve"))
}

#[cfg(test)]
mod tests;
