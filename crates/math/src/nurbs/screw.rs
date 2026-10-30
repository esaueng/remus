//! Certified NURBS forms of curves and surfaces swept by a screw motion.
//!
//! A screw motion about the axis through `o` along the unit vector `a` turns
//! a point by `k·s` radians about the axis while it advances `s` along it:
//!
//! ```text
//! H_p(s) = o + (p − o)∥ + cos(ks)·(p − o)⊥ + sin(ks)·a × (p − o)⊥ + s·a
//! ```
//!
//! with `(p − o)∥` and `(p − o)⊥` the components of `p − o` along and across
//! `a`. `H_p` is a helix of radius `ρ = ‖(p − o)⊥‖`, and
//! `‖H_p''''(s)‖ = ρ·k⁴` exactly.
//!
//! ## Surfaces
//!
//! Sweeping a NURBS curve `C(v) = Σ R_i(v)·P_i` (rational basis `R_i`, which
//! sums to one) gives `S(s, v) = Σ R_i(v)·H_{P_i}(s)`: the motion is affine
//! in `p` for each `s`. Every control point traces its own helix, so fitting
//! each `H_{P_i}` with [`fit_cubic_hermite`] on one shared span count yields
//! a tensor-product NURBS whose `s` rows are the fitted helices. Where every
//! weight is positive the `R_i(v)` are a convex combination, so
//!
//! ```text
//! ‖S̃(s, v) − S(s, v)‖ ≤ max_i ‖H̃_{P_i}(s) − H_{P_i}(s)‖
//! ```
//!
//! and the per-helix Hermite bound certifies the whole surface. Sharing one
//! fourth-derivative bound `M = ρ_max·k⁴` (the largest radius over every
//! curve swept together) gives every fit the same spans and knots: two
//! surfaces that sweep the same control point share the same boundary helix,
//! bit for bit, and so does a helix edge fitted from that point.
//!
//! The rows interpolate the exact helices at their ends, so the `s = 0` row
//! is the input curve and the `s = height` row is the curve moved by
//! [`ScrewMotion::point`] exactly.

use crate::MathError;
use crate::nurbs::hermite_fit::fit_cubic_hermite;
use crate::nurbs::{NurbsCurve, NurbsSurface};
use crate::vec::{Point3, Vec3};

/// A screw motion of a given height about an axis (see the
/// [module docs](self)).
#[derive(Debug, Clone, Copy)]
pub struct ScrewMotion {
    origin: Point3,
    axis: Vec3,
    rate: f64,
    height: f64,
}

/// The span count and certified bound shared by every fit of one sweep.
#[derive(Debug, Clone, Copy)]
pub struct ScrewPlan {
    /// Largest radius the plan covers.
    pub max_radius: f64,
    /// Uniform Hermite spans along the axis.
    pub spans: usize,
    /// Proven bound, in exact arithmetic, on the distance from any fitted
    /// helix or surface of this plan to the exact one.
    pub remainder_bound: f64,
    tolerance: f64,
    max_spans: usize,
}

impl ScrewMotion {
    /// A motion turning `rate` radians per unit of advance along `axis`
    /// (right-handed about `axis` for a positive rate) over `height`.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] when a value is non-finite or the
    /// height is not positive; [`MathError::ZeroVector`] for a zero axis.
    pub fn new(origin: Point3, axis: Vec3, rate: f64, height: f64) -> Result<Self, MathError> {
        if !origin.0.iter().all(|c| c.is_finite()) || !rate.is_finite() {
            return Err(MathError::ParameterOutOfRange {
                value: rate,
                min: f64::NEG_INFINITY,
                max: f64::INFINITY,
            });
        }
        if !height.is_finite() || height <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: height,
                min: f64::MIN_POSITIVE,
                max: f64::INFINITY,
            });
        }
        Ok(Self {
            origin,
            axis: axis.normalize()?,
            rate,
            height,
        })
    }

    /// The motion's height.
    #[must_use]
    pub const fn height(&self) -> f64 {
        self.height
    }

    /// The motion's unit axis.
    #[must_use]
    pub const fn axis(&self) -> Vec3 {
        self.axis
    }

    /// `v` turned by the motion's turn at advance `s`, without advancing.
    #[must_use]
    pub fn turn_vector(&self, v: Vec3, s: f64) -> Vec3 {
        let along = self.axis * v.dot(self.axis);
        let across = v - along;
        let (sin, cos) = (self.rate * s).sin_cos();
        along + across * cos + self.axis.cross(across) * sin
    }

    /// `p` turned about the axis by the motion's turn at advance `s`, without
    /// advancing.
    #[must_use]
    pub fn turn_point(&self, p: Point3, s: f64) -> Point3 {
        self.origin + self.turn_vector(p - self.origin, s)
    }

    /// Total turn at the far end, `rate · height`.
    #[must_use]
    pub fn total_turn(&self) -> f64 {
        self.rate * self.height
    }

    /// Distance of `p` from the axis.
    #[must_use]
    pub fn radius(&self, p: Point3) -> f64 {
        self.split(p).1.length()
    }

    /// `p − o` split into its components along and across the axis.
    fn split(&self, p: Point3) -> (Vec3, Vec3) {
        let d = p - self.origin;
        let along = self.axis * d.dot(self.axis);
        (along, d - along)
    }

    /// `H_p(s)`: `p` moved by the motion up to advance `s`.
    #[must_use]
    pub fn point(&self, p: Point3, s: f64) -> Point3 {
        self.sample(p, s).0
    }

    /// `H_p(s)` and `H_p'(s)`.
    fn sample(&self, p: Point3, s: f64) -> (Point3, Vec3) {
        let (along, across) = self.split(p);
        let side = self.axis.cross(across);
        let (sin, cos) = (self.rate * s).sin_cos();
        let point = self.origin + along + across * cos + side * sin + self.axis * s;
        let derivative = (side * cos - across * sin) * self.rate + self.axis;
        (point, derivative)
    }

    /// The span count and bound for fits reaching out to `max_radius`.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] for a negative or non-finite radius
    /// or a non-positive tolerance; [`MathError::ResourceLimitExceeded`] when
    /// `tolerance` needs more than `max_spans` spans.
    pub fn plan(
        &self,
        max_radius: f64,
        tolerance: f64,
        max_spans: usize,
    ) -> Result<ScrewPlan, MathError> {
        if !max_radius.is_finite() || max_radius < 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: max_radius,
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        // Fit the farthest helix itself: it fixes the spans every fit of the
        // plan shares, and its bound is the plan's.
        let probe = self.origin + any_perpendicular(self.axis) * max_radius;
        let fit = self.fit(probe, max_radius, tolerance, max_spans)?;
        Ok(ScrewPlan {
            max_radius,
            spans: fit.spans,
            remainder_bound: fit.remainder_bound,
            tolerance,
            max_spans,
        })
    }

    fn fit(
        &self,
        p: Point3,
        max_radius: f64,
        tolerance: f64,
        max_spans: usize,
    ) -> Result<crate::nurbs::hermite_fit::HermiteFit, MathError> {
        let bound = max_radius * self.rate.powi(4);
        fit_cubic_hermite(
            |s| self.sample(p, s),
            0.0,
            self.height,
            bound,
            tolerance,
            max_spans,
        )
    }

    /// The fitted helix `H̃_p` over `[0, height]`, within the plan's bound of
    /// `H_p`.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] when `p` lies farther from the axis
    /// than the plan covers.
    pub fn helix(&self, plan: &ScrewPlan, p: Point3) -> Result<NurbsCurve, MathError> {
        let radius = self.radius(p);
        if radius > plan.max_radius {
            return Err(MathError::ParameterOutOfRange {
                value: radius,
                min: 0.0,
                max: plan.max_radius,
            });
        }
        Ok(self
            .fit(p, plan.max_radius, plan.tolerance, plan.max_spans)?
            .curve)
    }

    /// The swept surface of `curve`: `u` runs along the motion over
    /// `[0, height]` (cubic, the plan's spans) and `v` is `curve`'s own
    /// parameter, degree, knots and weights. Within the plan's bound of the
    /// exact swept surface.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] when a control point lies farther
    /// from the axis than the plan covers, or surface construction errors.
    pub fn surface(&self, plan: &ScrewPlan, curve: &NurbsCurve) -> Result<NurbsSurface, MathError> {
        let helices = curve
            .control_points()
            .iter()
            .map(|&p| self.helix(plan, p))
            .collect::<Result<Vec<_>, _>>()?;
        let rows = helices
            .first()
            .map_or(0, |helix| helix.control_points().len());
        let control_points: Vec<Vec<Point3>> = (0..rows)
            .map(|row| {
                helices
                    .iter()
                    .map(|helix| helix.control_points()[row])
                    .collect()
            })
            .collect();
        let weights = vec![curve.weights().to_vec(); rows];
        let knots_u = helices
            .first()
            .map(|helix| helix.knots().to_vec())
            .unwrap_or_default();
        NurbsSurface::new(
            3,
            curve.degree(),
            knots_u,
            curve.knots().to_vec(),
            control_points,
            weights,
        )
    }
}

/// A unit vector perpendicular to the unit vector `axis`.
fn any_perpendicular(axis: Vec3) -> Vec3 {
    let seed = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let across = seed - axis * seed.dot(axis);
    across * (1.0 / across.length())
}

#[cfg(test)]
mod tests;
