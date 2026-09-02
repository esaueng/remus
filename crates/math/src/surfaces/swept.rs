//! Exact math-layer carriers for STEP swept surfaces.

use std::f64::consts::{FRAC_PI_2, TAU};

use crate::MathError;
use crate::context::OperationContext;
use crate::curvature::{SurfaceCurvature, curvature_from_fundamental_forms};
use crate::curves::{Circle3D, Ellipse3D, Hyperbola3D, Line3D, Parabola3D};
use crate::nurbs::{NurbsCurve, NurbsSurface, curve_split};
use crate::vec::{Point3, Vec3};

const PARAM_EPS: f64 = 1.0e-14;
const SEED_COUNT: usize = 33;

/// A self-contained curve suitable for use as a swept-surface profile.
#[derive(Debug, Clone)]
pub enum SweptCurve {
    /// Infinite line with explicit placement.
    Line(Line3D),
    /// Full circle.
    Circle(Circle3D),
    /// Full ellipse.
    Ellipse(Ellipse3D),
    /// Positive hyperbola branch.
    Hyperbola(Hyperbola3D),
    /// Parabola.
    Parabola(Parabola3D),
    /// General rational B-spline curve.
    Nurbs(NurbsCurve),
}

/// How a bounded projection solve converged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProjectionConvergence {
    /// A stationary point in the interior of the requested profile span.
    Interior,
    /// The closest point is an endpoint of the requested profile span.
    Boundary,
}

/// Qualified closest-point result on a swept profile curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweptCurveProjection {
    /// Profile parameter.
    pub parameter: f64,
    /// Euclidean residual in model units.
    pub residual: f64,
    /// Solver disposition.
    pub convergence: ProjectionConvergence,
    /// Safeguarded Newton iterations consumed after deterministic seeding.
    pub iterations: usize,
}

/// Qualified closest-point result on a swept surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweptProjection {
    /// First surface parameter.
    pub u: f64,
    /// Second surface parameter.
    pub v: f64,
    /// Euclidean residual in model units.
    pub residual: f64,
    /// Solver disposition of the bounded profile parameter.
    pub convergence: ProjectionConvergence,
    /// Safeguarded Newton iterations consumed after deterministic seeding.
    pub iterations: usize,
}

#[derive(Clone, Copy)]
struct ObjectiveSample {
    value: f64,
    gradient: f64,
    curvature: f64,
}

impl SweptCurve {
    /// Evaluate the profile and reject non-finite input or output.
    ///
    /// # Errors
    ///
    /// Returns a typed math error for non-finite parameters or geometry.
    pub fn evaluate_checked(&self, parameter: f64) -> Result<Point3, MathError> {
        if !parameter.is_finite() {
            return Err(parameter_error(parameter, f64::NEG_INFINITY, f64::INFINITY));
        }
        let point = match self {
            Self::Line(curve) => curve.evaluate(parameter),
            Self::Circle(curve) => curve.evaluate(parameter),
            Self::Ellipse(curve) => curve.evaluate(parameter),
            Self::Hyperbola(curve) => curve.evaluate(parameter),
            Self::Parabola(curve) => curve.evaluate(parameter),
            Self::Nurbs(curve) => curve.evaluate(parameter),
        };
        finite_point(point, 0)?;
        Ok(point)
    }

    /// Position plus first and second profile derivatives.
    ///
    /// # Errors
    ///
    /// Returns a typed math error for non-finite parameters or results.
    pub fn derivatives_checked(&self, parameter: f64) -> Result<(Point3, Vec3, Vec3), MathError> {
        let point = self.evaluate_checked(parameter)?;
        let zero = Vec3::new(0.0, 0.0, 0.0);
        let (first, second) = match self {
            Self::Line(curve) => (curve.tangent(), zero),
            Self::Circle(curve) => (
                curve.tangent(parameter) * curve.radius(),
                curve.center() - point,
            ),
            Self::Ellipse(curve) => (curve.tangent(parameter), curve.center() - point),
            Self::Hyperbola(curve) => (curve.tangent(parameter), point - curve.center()),
            Self::Parabola(curve) => (
                curve.tangent(parameter),
                curve.axis_dir() * (0.5 / curve.focal_length()),
            ),
            Self::Nurbs(curve) => {
                let derivatives = curve.derivatives(parameter, 2);
                (derivatives[1], derivatives[2])
            }
        };
        finite_vector(first, 1)?;
        finite_vector(second, 2)?;
        Ok((point, first, second))
    }

    /// Natural carrier domain. Open conics and lines report infinite bounds.
    #[must_use]
    pub fn natural_domain(&self) -> (f64, f64) {
        match self {
            Self::Circle(_) | Self::Ellipse(_) => (0.0, TAU),
            Self::Nurbs(curve) => curve.domain(),
            Self::Line(_) | Self::Hyperbola(_) | Self::Parabola(_) => {
                (f64::NEG_INFINITY, f64::INFINITY)
            }
        }
    }

    /// Optional exact period of the stored profile parameterization.
    #[must_use]
    pub fn period(&self) -> Option<f64> {
        match self {
            Self::Circle(_) | Self::Ellipse(_) => Some(TAU),
            Self::Line(_) | Self::Hyperbola(_) | Self::Parabola(_) | Self::Nurbs(_) => None,
        }
    }

    /// Stable semantic type tag.
    #[must_use]
    pub const fn type_tag(&self) -> &'static str {
        match self {
            Self::Line(_) => "line",
            Self::Circle(_) => "circle",
            Self::Ellipse(_) => "ellipse",
            Self::Hyperbola(_) => "hyperbola",
            Self::Parabola(_) => "parabola",
            Self::Nurbs(_) => "nurbs",
        }
    }

    /// Validate finite stored data and a regular representative derivative.
    ///
    /// # Errors
    ///
    /// Returns a typed math error for malformed NURBS data, non-finite
    /// analytic data, or an everywhere-degenerate representative tangent.
    pub fn validate(&self) -> Result<(), MathError> {
        if let Self::Nurbs(curve) = self {
            curve.validate()?;
        }
        let parameter = match self {
            Self::Nurbs(curve) => f64::midpoint(curve.domain().0, curve.domain().1),
            _ => 0.0,
        };
        let (_, first, _) = self.derivatives_checked(parameter)?;
        if first.length() < f64::MIN_POSITIVE {
            return Err(MathError::ZeroVector);
        }
        Ok(())
    }

    /// Project onto a caller-bounded profile span.
    ///
    /// # Errors
    ///
    /// Returns a typed refusal for invalid bounds, cancellation, or exhausted
    /// Newton work budget.
    pub fn project_point_checked(
        &self,
        point: Point3,
        bounds: (f64, f64),
        context: &OperationContext,
    ) -> Result<SweptCurveProjection, MathError> {
        finite_point(point, 0)?;
        let (parameter, convergence, iterations) = bounded_minimize(bounds, context, |t| {
            let (curve_point, first, second) = self.derivatives_checked(t)?;
            let residual = curve_point - point;
            Ok(ObjectiveSample {
                value: residual.length_squared(),
                gradient: 2.0 * residual.dot(first),
                curvature: 2.0 * (first.dot(first) + residual.dot(second)),
            })
        })?;
        let residual = (self.evaluate_checked(parameter)? - point).length();
        Ok(SweptCurveProjection {
            parameter,
            residual,
            convergence,
            iterations,
        })
    }

    /// Exact rational NURBS representation over a finite, directed span.
    ///
    /// The returned NURBS domain is ascending. For a reversed input span its
    /// control polygon is reversed, so the lower returned knot evaluates to
    /// the original span start.
    ///
    /// # Errors
    ///
    /// Returns a typed refusal for non-finite, zero, out-of-domain, or
    /// numerically unrepresentable spans.
    pub fn to_nurbs(&self, start: f64, end: f64) -> Result<NurbsCurve, MathError> {
        validate_span(start, end)?;
        let reverse = end < start;
        let (lo, hi) = if reverse { (end, start) } else { (start, end) };
        let curve = match self {
            Self::Line(line) => NurbsCurve::new(
                1,
                vec![lo, lo, hi, hi],
                vec![line.evaluate(lo), line.evaluate(hi)],
                vec![1.0, 1.0],
            )?,
            Self::Circle(circle) => elliptic_arc_to_nurbs(lo, hi, |t| {
                (circle.evaluate(t), circle.tangent(t) * circle.radius())
            })?,
            Self::Ellipse(ellipse) => {
                elliptic_arc_to_nurbs(lo, hi, |t| (ellipse.evaluate(t), ellipse.tangent(t)))?
            }
            Self::Hyperbola(hyperbola) => {
                let half = 0.5 * (hi - lo);
                NurbsCurve::new(
                    2,
                    vec![lo, lo, lo, hi, hi, hi],
                    vec![
                        hyperbola.evaluate(lo),
                        hyperbola.tangent_intersection(lo, hi),
                        hyperbola.evaluate(hi),
                    ],
                    vec![1.0, half.cosh(), 1.0],
                )?
            }
            Self::Parabola(parabola) => NurbsCurve::new(
                2,
                vec![lo, lo, lo, hi, hi, hi],
                vec![
                    parabola.evaluate(lo),
                    parabola.tangent_intersection(lo, hi),
                    parabola.evaluate(hi),
                ],
                vec![1.0, 1.0, 1.0],
            )?,
            Self::Nurbs(curve) => trim_nurbs(curve, lo, hi)?,
        };
        Ok(if reverse { curve.reversed() } else { curve })
    }

    pub(crate) fn compatibility_bounds(&self, point: Point3) -> (f64, f64) {
        match self {
            Self::Circle(_) | Self::Ellipse(_) => (0.0, TAU),
            Self::Nurbs(curve) => curve.domain(),
            Self::Line(line) => {
                let center = line.project(point);
                let half = (point - line.origin()).length().max(1.0);
                (center - half, center + half)
            }
            Self::Parabola(curve) => {
                let center = curve.project(point);
                let half = center.abs().max(curve.focal_length()).max(1.0) * 2.0;
                (center - half, center + half)
            }
            Self::Hyperbola(curve) => {
                let center = curve.project(point);
                (center - 4.0, center + 4.0)
            }
        }
    }

    fn all_control_points_on_axis(&self, axis: &Line3D) -> bool {
        match self {
            Self::Line(line) => {
                point_axis_distance(line.origin(), axis) <= PARAM_EPS
                    && line.direction().cross(axis.direction()).length() <= PARAM_EPS
            }
            Self::Nurbs(curve) => curve
                .control_points()
                .iter()
                .all(|&point| point_axis_distance(point, axis) <= PARAM_EPS),
            Self::Circle(_) | Self::Ellipse(_) | Self::Hyperbola(_) | Self::Parabola(_) => false,
        }
    }

    fn all_control_directions_parallel(&self, direction: Vec3) -> bool {
        let Ok(unit) = direction.normalize() else {
            return true;
        };
        match self {
            Self::Line(line) => line.direction().cross(unit).length() <= PARAM_EPS,
            Self::Nurbs(curve) => {
                let Some(&first) = curve.control_points().first() else {
                    return true;
                };
                curve.control_points().iter().skip(1).all(|&point| {
                    let delta = point - first;
                    delta.length() <= PARAM_EPS
                        || delta.cross(unit).length() <= PARAM_EPS * delta.length()
                })
            }
            Self::Circle(_) | Self::Ellipse(_) | Self::Hyperbola(_) | Self::Parabola(_) => false,
        }
    }
}

/// Exact carrier surface produced by revolving a profile around a line.
#[derive(Debug, Clone)]
pub struct SurfaceOfRevolution {
    profile: SweptCurve,
    axis: Line3D,
}

impl SurfaceOfRevolution {
    /// Construct a regular revolution carrier.
    ///
    /// # Errors
    ///
    /// Refuses malformed profiles and profiles lying entirely on the axis.
    pub fn new(profile: SweptCurve, axis: Line3D) -> Result<Self, MathError> {
        profile.validate()?;
        finite_point(axis.origin(), 0)?;
        finite_vector(axis.direction(), 1)?;
        if profile.all_control_points_on_axis(&axis) {
            return Err(MathError::ZeroVector);
        }
        Ok(Self { profile, axis })
    }

    /// Stored profile.
    #[must_use]
    pub const fn profile(&self) -> &SweptCurve {
        &self.profile
    }

    /// Stored unit-axis line.
    #[must_use]
    pub const fn axis(&self) -> &Line3D {
        &self.axis
    }

    /// Revolution period in radians.
    #[must_use]
    #[allow(clippy::unused_self, clippy::unnecessary_wraps)]
    pub const fn u_period(&self) -> Option<f64> {
        Some(TAU)
    }

    /// Optional profile period.
    #[must_use]
    pub fn v_period(&self) -> Option<f64> {
        self.profile.period()
    }

    /// Evaluate with finite-data checks.
    ///
    /// # Errors
    ///
    /// Returns a typed error for non-finite parameters or results.
    pub fn evaluate_checked(&self, u: f64, v: f64) -> Result<Point3, MathError> {
        if !u.is_finite() {
            return Err(parameter_error(u, f64::NEG_INFINITY, f64::INFINITY));
        }
        let profile_point = self.profile.evaluate_checked(v)?;
        let result = rotate_point(profile_point, &self.axis, u);
        finite_point(result, 0)?;
        Ok(result)
    }

    /// Position and exact first/second surface partials.
    ///
    /// # Errors
    ///
    /// Returns a typed error for non-finite parameters or results.
    pub fn derivatives_checked(
        &self,
        u: f64,
        v: f64,
    ) -> Result<(Point3, Vec3, Vec3, Vec3, Vec3, Vec3), MathError> {
        if !u.is_finite() {
            return Err(parameter_error(u, f64::NEG_INFINITY, f64::INFINITY));
        }
        let (profile_point, profile_first, profile_second) = self.profile.derivatives_checked(v)?;
        let origin = self.axis.origin();
        let axis = self.axis.direction();
        let rotated = rotate_vector(profile_point - origin, axis, u);
        let rotated_first = rotate_vector(profile_first, axis, u);
        let rotated_second = rotate_vector(profile_second, axis, u);
        let point = origin + rotated;
        let partial_u = axis.cross(rotated);
        let partial_v = rotated_first;
        let partial_uu = axis.cross(partial_u);
        let partial_uv = axis.cross(rotated_first);
        finite_vector(partial_u, 1)?;
        finite_vector(partial_v, 2)?;
        finite_vector(partial_uu, 3)?;
        finite_vector(partial_uv, 4)?;
        finite_vector(rotated_second, 5)?;
        Ok((
            point,
            partial_u,
            partial_v,
            partial_uu,
            partial_uv,
            rotated_second,
        ))
    }

    /// Checked unit normal, including the limiting normal at a regular pole.
    ///
    /// # Errors
    ///
    /// Returns [`MathError::ZeroVector`] for a genuinely singular patch.
    pub fn normal_checked(&self, u: f64, v: f64) -> Result<Vec3, MathError> {
        let (_, partial_u, partial_v, _, _, _) = self.derivatives_checked(u, v)?;
        if let Ok(normal) = partial_u.cross(partial_v).normalize() {
            return Ok(normal);
        }
        let axis = self.axis.direction();
        axis.cross(partial_v).cross(partial_v).normalize()
    }

    /// Principal curvatures from exact first and second derivatives.
    ///
    /// # Errors
    ///
    /// Returns a typed error at singular parameter values.
    pub fn curvature(&self, u: f64, v: f64) -> Result<SurfaceCurvature, MathError> {
        let (_, xu, xv, xuu, xuv, xvv) = self.derivatives_checked(u, v)?;
        curvature_from_fundamental_forms(xu, xv, xuu, xuv, xvv)
    }

    /// Checked closest-point projection over a finite profile span.
    ///
    /// # Errors
    ///
    /// Returns a typed refusal for invalid bounds, cancellation, or exhausted
    /// Newton work budget.
    pub fn project_point_checked(
        &self,
        point: Point3,
        profile_bounds: (f64, f64),
        context: &OperationContext,
    ) -> Result<SweptProjection, MathError> {
        finite_point(point, 0)?;
        let origin = self.axis.origin();
        let axis = self.axis.direction();
        let target_delta = point - origin;
        let target_height = target_delta.dot(axis);
        let target_radial = target_delta - axis * target_height;
        let target_radius = target_radial.length();
        let (v, convergence, iterations) =
            bounded_minimize(profile_bounds, context, |parameter| {
                let (curve_point, first, second) = self.profile.derivatives_checked(parameter)?;
                let delta = curve_point - origin;
                let height = delta.dot(axis);
                let height_first = first.dot(axis);
                let height_second = second.dot(axis);
                let radial = delta - axis * height;
                let radial_first = first - axis * height_first;
                let radial_second = second - axis * height_second;
                let radius = radial.length();
                let (radius_first, radius_second) = if radius > PARAM_EPS {
                    let first_value = radial.dot(radial_first) / radius;
                    let second_value = (radial_first.dot(radial_first) + radial.dot(radial_second)
                        - first_value * first_value)
                        / radius;
                    (first_value, second_value)
                } else {
                    // Radius is an absolute value at an axis crossing. Choosing
                    // the negative-parameter one-sided derivative gives Newton a
                    // deterministic branch instead of falsely treating the pole
                    // as stationary for every nearby off-axis target. The other
                    // branch represents the same revolved point with u shifted by
                    // pi, so this does not change the closest-point set.
                    (-radial_first.length(), 0.0)
                };
                let height_residual = height - target_height;
                let radius_residual = radius - target_radius;
                Ok(ObjectiveSample {
                    value: height_residual
                        .mul_add(height_residual, radius_residual * radius_residual),
                    gradient: 2.0
                        * height_residual.mul_add(height_first, radius_residual * radius_first),
                    curvature: 2.0
                        * (height_first.mul_add(height_first, height_residual * height_second)
                            + radius_first.mul_add(radius_first, radius_residual * radius_second)),
                })
            })?;
        let profile_point = self.profile.evaluate_checked(v)?;
        let profile_delta = profile_point - origin;
        let profile_height = profile_delta.dot(axis);
        let profile_radial = profile_delta - axis * profile_height;
        let u = signed_angle(profile_radial, target_radial, axis);
        let residual = (self.evaluate_checked(u, v)? - point).length();
        Ok(SweptProjection {
            u,
            v,
            residual,
            convergence,
            iterations,
        })
    }

    /// Exact rational NURBS lowering over finite directed parameter spans.
    ///
    /// # Errors
    ///
    /// Returns a typed refusal for invalid or unrepresentable spans.
    pub fn to_nurbs(
        &self,
        u_bounds: (f64, f64),
        v_bounds: (f64, f64),
    ) -> Result<NurbsSurface, MathError> {
        validate_span(u_bounds.0, u_bounds.1)?;
        let profile = self.profile.to_nurbs(v_bounds.0, v_bounds.1)?;
        let ring = rotation_arc(u_bounds.0, u_bounds.1)?;
        let origin = self.axis.origin();
        let axis = self.axis.direction();
        let mut control_points = Vec::with_capacity(ring.nodes.len());
        let mut weights = Vec::with_capacity(ring.nodes.len());
        for node in ring.nodes {
            let mut row = Vec::with_capacity(profile.control_points().len());
            let mut weight_row = Vec::with_capacity(profile.control_points().len());
            for (&point, &weight) in profile.control_points().iter().zip(profile.weights()) {
                let delta = point - origin;
                let axial = axis * delta.dot(axis);
                let radial = delta - axial;
                row.push(origin + axial + rotate_vector(radial, axis, node.angle) * node.scale);
                weight_row.push(weight * node.weight);
            }
            control_points.push(row);
            weights.push(weight_row);
        }
        NurbsSurface::new(
            2,
            profile.degree(),
            ring.knots,
            profile.knots().to_vec(),
            control_points,
            weights,
        )
    }
}

/// Exact carrier surface produced by translating a profile along a vector.
#[derive(Debug, Clone)]
pub struct SurfaceOfLinearExtrusion {
    profile: SweptCurve,
    direction: Vec3,
}

impl SurfaceOfLinearExtrusion {
    /// Construct a regular linear-extrusion carrier.
    ///
    /// The full direction magnitude is retained as the v-parameter scale.
    ///
    /// # Errors
    ///
    /// Refuses malformed profiles, zero/non-finite directions, and profiles
    /// whose entire carrier is parallel to the extrusion direction.
    pub fn new(profile: SweptCurve, direction: Vec3) -> Result<Self, MathError> {
        profile.validate()?;
        finite_vector(direction, 0)?;
        if direction.length() < f64::MIN_POSITIVE
            || profile.all_control_directions_parallel(direction)
        {
            return Err(MathError::ZeroVector);
        }
        Ok(Self { profile, direction })
    }

    /// Stored profile.
    #[must_use]
    pub const fn profile(&self) -> &SweptCurve {
        &self.profile
    }

    /// Full, non-normalized STEP vector.
    #[must_use]
    pub const fn direction(&self) -> Vec3 {
        self.direction
    }

    /// Optional profile period.
    #[must_use]
    pub fn u_period(&self) -> Option<f64> {
        self.profile.period()
    }

    /// Linear sweep is not periodic.
    #[must_use]
    #[allow(clippy::unused_self)]
    pub const fn v_period(&self) -> Option<f64> {
        None
    }

    /// Evaluate with finite-data checks.
    ///
    /// # Errors
    ///
    /// Returns a typed error for non-finite parameters or results.
    pub fn evaluate_checked(&self, u: f64, v: f64) -> Result<Point3, MathError> {
        if !v.is_finite() {
            return Err(parameter_error(v, f64::NEG_INFINITY, f64::INFINITY));
        }
        let result = self.profile.evaluate_checked(u)? + self.direction * v;
        finite_point(result, 0)?;
        Ok(result)
    }

    /// Position and exact first/second surface partials.
    ///
    /// # Errors
    ///
    /// Returns a typed error for non-finite parameters or results.
    pub fn derivatives_checked(
        &self,
        u: f64,
        v: f64,
    ) -> Result<(Point3, Vec3, Vec3, Vec3, Vec3, Vec3), MathError> {
        if !v.is_finite() {
            return Err(parameter_error(v, f64::NEG_INFINITY, f64::INFINITY));
        }
        let (profile_point, profile_first, profile_second) = self.profile.derivatives_checked(u)?;
        let zero = Vec3::new(0.0, 0.0, 0.0);
        Ok((
            profile_point + self.direction * v,
            profile_first,
            self.direction,
            profile_second,
            zero,
            zero,
        ))
    }

    /// Checked unit normal.
    ///
    /// # Errors
    ///
    /// Returns [`MathError::ZeroVector`] for a singular patch.
    pub fn normal_checked(&self, u: f64, v: f64) -> Result<Vec3, MathError> {
        let (_, partial_u, partial_v, _, _, _) = self.derivatives_checked(u, v)?;
        partial_u.cross(partial_v).normalize()
    }

    /// Principal curvatures from exact first and second derivatives.
    ///
    /// # Errors
    ///
    /// Returns a typed error at singular parameter values.
    pub fn curvature(&self, u: f64, v: f64) -> Result<SurfaceCurvature, MathError> {
        let (_, xu, xv, xuu, xuv, xvv) = self.derivatives_checked(u, v)?;
        curvature_from_fundamental_forms(xu, xv, xuu, xuv, xvv)
    }

    /// Checked closest-point projection over a finite profile span.
    ///
    /// # Errors
    ///
    /// Returns a typed refusal for invalid bounds, cancellation, or exhausted
    /// Newton work budget.
    pub fn project_point_checked(
        &self,
        point: Point3,
        profile_bounds: (f64, f64),
        context: &OperationContext,
    ) -> Result<SweptProjection, MathError> {
        finite_point(point, 0)?;
        let direction_sq = self.direction.length_squared();
        let (u, convergence, iterations) =
            bounded_minimize(profile_bounds, context, |parameter| {
                let (curve_point, first, second) = self.profile.derivatives_checked(parameter)?;
                let delta = curve_point - point;
                let residual = reject_from_direction(delta, self.direction, direction_sq);
                let first_perp = reject_from_direction(first, self.direction, direction_sq);
                let second_perp = reject_from_direction(second, self.direction, direction_sq);
                Ok(ObjectiveSample {
                    value: residual.length_squared(),
                    gradient: 2.0 * residual.dot(first_perp),
                    curvature: 2.0 * (first_perp.dot(first_perp) + residual.dot(second_perp)),
                })
            })?;
        let curve_point = self.profile.evaluate_checked(u)?;
        let v = (point - curve_point).dot(self.direction) / direction_sq;
        let residual = (self.evaluate_checked(u, v)? - point).length();
        Ok(SweptProjection {
            u,
            v,
            residual,
            convergence,
            iterations,
        })
    }

    /// Exact rational NURBS lowering over finite directed parameter spans.
    ///
    /// # Errors
    ///
    /// Returns a typed refusal for invalid or unrepresentable spans.
    pub fn to_nurbs(
        &self,
        u_bounds: (f64, f64),
        v_bounds: (f64, f64),
    ) -> Result<NurbsSurface, MathError> {
        validate_span(v_bounds.0, v_bounds.1)?;
        let profile = self.profile.to_nurbs(u_bounds.0, u_bounds.1)?;
        let (v_lo, v_hi) = if v_bounds.0 < v_bounds.1 {
            v_bounds
        } else {
            (v_bounds.1, v_bounds.0)
        };
        let mut control_points = Vec::with_capacity(profile.control_points().len());
        let mut weights = Vec::with_capacity(profile.control_points().len());
        for (&point, &weight) in profile.control_points().iter().zip(profile.weights()) {
            control_points.push(vec![
                point + self.direction * v_bounds.0,
                point + self.direction * v_bounds.1,
            ]);
            weights.push(vec![weight, weight]);
        }
        NurbsSurface::new(
            profile.degree(),
            1,
            profile.knots().to_vec(),
            vec![v_lo, v_lo, v_hi, v_hi],
            control_points,
            weights,
        )
    }
}

fn finite_point(point: Point3, index: usize) -> Result<(), MathError> {
    if point.0.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(MathError::InvalidControlPointValue {
            index,
            x: point.x(),
            y: point.y(),
            z: point.z(),
        })
    }
}

fn finite_vector(vector: Vec3, index: usize) -> Result<(), MathError> {
    finite_point(Point3::new(vector.x(), vector.y(), vector.z()), index)
}

fn parameter_error(value: f64, min: f64, max: f64) -> MathError {
    MathError::ParameterOutOfRange { value, min, max }
}

fn validate_span(start: f64, end: f64) -> Result<(), MathError> {
    if !start.is_finite() || !end.is_finite() || (end - start).abs() <= PARAM_EPS {
        return Err(parameter_error(end, start, f64::INFINITY));
    }
    Ok(())
}

fn bounded_minimize(
    bounds: (f64, f64),
    context: &OperationContext,
    mut evaluate: impl FnMut(f64) -> Result<ObjectiveSample, MathError>,
) -> Result<(f64, ProjectionConvergence, usize), MathError> {
    validate_span(bounds.0, bounds.1)?;
    let (lo, hi) = if bounds.0 < bounds.1 {
        bounds
    } else {
        (bounds.1, bounds.0)
    };
    let budget = context.budgets.newton_iterations;
    if budget == 0 {
        return Err(MathError::ConvergenceFailure { iterations: 0 });
    }
    context.check_cancelled()?;
    let mut samples = Vec::with_capacity(SEED_COUNT);
    let mut best_index = 0;
    let mut best_value = f64::INFINITY;
    for index in 0..SEED_COUNT {
        #[allow(clippy::cast_precision_loss)]
        let fraction = index as f64 / (SEED_COUNT - 1) as f64;
        let parameter = (hi - lo).mul_add(fraction, lo);
        let sample = evaluate(parameter)?;
        if !sample.value.is_finite()
            || !sample.gradient.is_finite()
            || !sample.curvature.is_finite()
        {
            return Err(MathError::ConvergenceFailure { iterations: 0 });
        }
        if sample.value < best_value {
            best_value = sample.value;
            best_index = index;
        }
        samples.push((parameter, sample));
    }
    let (seed, seed_sample) = samples[best_index];
    if (best_index == 0 && seed_sample.gradient >= 0.0)
        || (best_index + 1 == SEED_COUNT && seed_sample.gradient <= 0.0)
    {
        return Ok((seed, ProjectionConvergence::Boundary, 0));
    }
    let mut left = samples[best_index.saturating_sub(1)].0;
    let mut right = samples[(best_index + 1).min(SEED_COUNT - 1)].0;
    let mut parameter = seed;
    let numeric_parameter_tolerance = PARAM_EPS * lo.abs().max(hi.abs()).max(1.0);
    for iteration in 1..=budget {
        context.check_cancelled()?;
        let sample = evaluate(parameter)?;
        let derivative_scale = (0.5 * sample.curvature.abs()).sqrt();
        let parameter_tolerance =
            numeric_parameter_tolerance.max(context.tolerance.parametric(derivative_scale));
        let newton_step = if sample.curvature > 0.0 {
            -sample.gradient / sample.curvature
        } else {
            f64::NAN
        };
        if newton_step.is_finite() && newton_step.abs() <= parameter_tolerance {
            return Ok((parameter, ProjectionConvergence::Interior, iteration));
        }
        let mut candidate = parameter + newton_step;
        if !candidate.is_finite() || candidate <= left || candidate >= right {
            candidate = f64::midpoint(left, right);
        }
        let candidate_sample = evaluate(candidate)?;
        if candidate_sample.gradient < 0.0 {
            left = candidate;
        } else {
            right = candidate;
        }
        if (candidate - parameter).abs() <= parameter_tolerance
            || (right - left).abs() <= parameter_tolerance
        {
            return Ok((candidate, ProjectionConvergence::Interior, iteration));
        }
        parameter = candidate;
    }
    Err(MathError::ConvergenceFailure { iterations: budget })
}

fn trim_nurbs(curve: &NurbsCurve, lo: f64, hi: f64) -> Result<NurbsCurve, MathError> {
    curve.validate()?;
    let (domain_lo, domain_hi) = curve.domain();
    let scale = domain_lo.abs().max(domain_hi.abs()).max(1.0);
    let tolerance = PARAM_EPS * scale;
    if lo < domain_lo - tolerance || hi > domain_hi + tolerance {
        return Err(parameter_error(
            if lo < domain_lo { lo } else { hi },
            domain_lo,
            domain_hi,
        ));
    }
    let lo = lo.max(domain_lo);
    let hi = hi.min(domain_hi);
    let mut trimmed = curve.clone();
    if lo > domain_lo + tolerance {
        let (_, right) = curve_split(&trimmed, lo)?;
        trimmed = right;
    }
    if hi < domain_hi - tolerance {
        let (left, _) = curve_split(&trimmed, hi)?;
        trimmed = left;
    }
    Ok(trimmed)
}

fn elliptic_arc_to_nurbs(
    start: f64,
    end: f64,
    evaluate: impl Fn(f64) -> (Point3, Vec3),
) -> Result<NurbsCurve, MathError> {
    let ratio = (end - start) / FRAC_PI_2;
    let snapped = if (ratio - ratio.round()).abs() <= 1.0e-12 {
        ratio.round()
    } else {
        ratio
    };
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let segments = (snapped.ceil() as usize).max(1);
    #[allow(clippy::cast_precision_loss)]
    let delta = (end - start) / segments as f64;
    let mut knots = vec![start; 3];
    for index in 1..segments {
        #[allow(clippy::cast_precision_loss)]
        let knot = delta.mul_add(index as f64, start);
        knots.extend([knot, knot]);
    }
    knots.extend([end, end, end]);
    let mut control_points = Vec::with_capacity(2 * segments + 1);
    let mut weights = Vec::with_capacity(2 * segments + 1);
    for index in 0..segments {
        #[allow(clippy::cast_precision_loss)]
        let t0 = delta.mul_add(index as f64, start);
        let t1 = t0 + delta;
        let (p0, tangent0) = evaluate(t0);
        let (p1, tangent1) = evaluate(t1);
        if index == 0 {
            control_points.push(p0);
            weights.push(1.0);
        }
        control_points.push(tangent_intersection(p0, tangent0, p1, tangent1)?);
        weights.push((0.5 * delta).cos());
        control_points.push(p1);
        weights.push(1.0);
    }
    NurbsCurve::new(2, knots, control_points, weights)
}

fn tangent_intersection(
    point0: Point3,
    direction0: Vec3,
    point1: Point3,
    direction1: Vec3,
) -> Result<Point3, MathError> {
    let rhs = point1 - point0;
    let cross = direction0.cross(direction1);
    let (a00, a01, b0, a10, a11, b1) =
        if cross.z().abs() >= cross.x().abs() && cross.z().abs() >= cross.y().abs() {
            (
                direction0.x(),
                -direction1.x(),
                rhs.x(),
                direction0.y(),
                -direction1.y(),
                rhs.y(),
            )
        } else if cross.y().abs() >= cross.x().abs() {
            (
                direction0.x(),
                -direction1.x(),
                rhs.x(),
                direction0.z(),
                -direction1.z(),
                rhs.z(),
            )
        } else {
            (
                direction0.y(),
                -direction1.y(),
                rhs.y(),
                direction0.z(),
                -direction1.z(),
                rhs.z(),
            )
        };
    let determinant = a00.mul_add(a11, -(a01 * a10));
    if determinant.abs() <= f64::MIN_POSITIVE {
        return Err(MathError::SingularMatrix);
    }
    let scale = b0.mul_add(a11, -(b1 * a01)) / determinant;
    Ok(point0 + direction0 * scale)
}

struct RotationNode {
    angle: f64,
    scale: f64,
    weight: f64,
}

struct RotationArc {
    knots: Vec<f64>,
    nodes: Vec<RotationNode>,
}

fn rotation_arc(start: f64, end: f64) -> Result<RotationArc, MathError> {
    validate_span(start, end)?;
    let span = end - start;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let segments = ((span.abs() / FRAC_PI_2 - PARAM_EPS).ceil() as usize).max(1);
    #[allow(clippy::cast_precision_loss)]
    let delta = span / segments as f64;
    let (knot_lo, knot_hi) = if start < end {
        (start, end)
    } else {
        (end, start)
    };
    let knot_delta = (knot_hi - knot_lo) / segments as f64;
    let mut knots = vec![knot_lo; 3];
    let mut nodes = Vec::with_capacity(2 * segments + 1);
    for index in 0..segments {
        #[allow(clippy::cast_precision_loss)]
        let angle0 = delta.mul_add(index as f64, start);
        let angle1 = angle0 + delta;
        let half = 0.5 * delta;
        if index == 0 {
            nodes.push(RotationNode {
                angle: angle0,
                scale: 1.0,
                weight: 1.0,
            });
        }
        nodes.push(RotationNode {
            angle: f64::midpoint(angle0, angle1),
            scale: 1.0 / half.cos(),
            weight: half.cos(),
        });
        nodes.push(RotationNode {
            angle: angle1,
            scale: 1.0,
            weight: 1.0,
        });
        if index + 1 < segments {
            #[allow(clippy::cast_precision_loss)]
            let knot = knot_delta.mul_add((index + 1) as f64, knot_lo);
            knots.extend([knot, knot]);
        }
    }
    knots.extend([knot_hi, knot_hi, knot_hi]);
    Ok(RotationArc { knots, nodes })
}

fn rotate_point(point: Point3, axis: &Line3D, angle: f64) -> Point3 {
    axis.origin() + rotate_vector(point - axis.origin(), axis.direction(), angle)
}

fn rotate_vector(vector: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    vector * cos + axis.cross(vector) * sin + axis * (axis.dot(vector) * (1.0 - cos))
}

fn point_axis_distance(point: Point3, axis: &Line3D) -> f64 {
    let delta = point - axis.origin();
    (delta - axis.direction() * delta.dot(axis.direction())).length()
}

fn reject_from_direction(vector: Vec3, direction: Vec3, direction_sq: f64) -> Vec3 {
    vector - direction * (vector.dot(direction) / direction_sq)
}

fn signed_angle(from: Vec3, to: Vec3, axis: Vec3) -> f64 {
    let from_length = from.length();
    let to_length = to.length();
    if from_length <= PARAM_EPS || to_length <= PARAM_EPS {
        return 0.0;
    }
    let from = from * (1.0 / from_length);
    let to = to * (1.0 / to_length);
    axis.dot(from.cross(to)).atan2(from.dot(to)).rem_euclid(TAU)
}

#[cfg(test)]
mod tests;
