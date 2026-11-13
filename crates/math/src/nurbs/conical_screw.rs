//! Certified NURBS forms of curves and surfaces swept by a conical screw
//! motion: a screw motion whose radius also changes linearly with the
//! advance (tapered threads).
//!
//! A point `p` is carried together with a direction `u` across the axis, the
//! way its distance from the axis changes. About the axis through `o` along
//! the unit vector `a`, turning `k` radians and moving `τ·u` outward per unit
//! of advance `s`:
//!
//! ```text
//! H_{p,u}(s) = o + (p − o)∥ + R(ks)·[(p − o)⊥ + τ·s·u] + s·a
//! ```
//!
//! with `R(φ)` the rotation by `φ` about `a`. For `u` the unit radial
//! direction of `p` this is a conical helix: radius `ρ + τ·s`, turn `k·s`.
//! With `g(s) = R(ks)·[q + τ·s·u]`,
//!
//! ```text
//! g⁽ⁿ⁾(s) = kⁿ·Jⁿ·R(ks)·[q + τ·s·u] + n·kⁿ⁻¹·Jⁿ⁻¹·R(ks)·τ·u
//! ```
//!
//! (`J` the quarter turn about `a`), so
//! `‖H''''(s)‖ ≤ k⁴·‖q + τ·s·u‖ + 4·|k|³·|τ|·‖u‖`, and `‖q + τ·s·u‖` is
//! convex in `s`: its largest value over the sweep is at one of its ends.
//!
//! ## Surfaces
//!
//! The motion is not affine in `p` alone (a radial offset is not), but it is
//! affine in the pair `(p, u)`. A profile curve `C(v) = Σ R_i(v)·P_i` with a
//! direction curve `U(v) = Σ R_i(v)·U_i` on the same basis sweeps
//!
//! ```text
//! S(s, v) = H_{C(v),U(v)}(s) = Σ R_i(v)·H_{P_i,U_i}(s)
//! ```
//!
//! so, as for [`screw`](super::screw), fitting every `H_{P_i,U_i}` with
//! [`fit_cubic_hermite`] on one shared span count yields a tensor-product
//! NURBS whose `s` rows are the fitted paths, and where every weight is
//! positive
//!
//! ```text
//! ‖S̃(s, v) − S(s, v)‖ ≤ max_i ‖H̃_{P_i,U_i}(s) − H_{P_i,U_i}(s)‖.
//! ```
//!
//! That certifies the surface against the sweep of the pair `(C, U)`. Where
//! `U` is exactly the unit radial direction of `C` (a concentric arc with
//! `U_i = (P_i − o)⊥ / ρ`, a radial line with one constant `U`), `S` is the
//! exact conical screw surface of `C`. Where `C` and `U` are themselves fits
//! (a spiral flank and the unit circle on its knots), the caller adds their
//! own remainders: at advance `s` the section is off by at most
//! `‖C̃ − C‖ + |τ|·s·‖Ũ − U‖`.
//!
//! Sharing one fourth-derivative bound gives every fit the same spans and
//! knots: two surfaces that sweep the same `(P_i, U_i)` share that boundary
//! path bit for bit, and so does an edge fitted from the same pair.

use crate::MathError;
use crate::nurbs::hermite_fit::fit_cubic_hermite;
use crate::nurbs::{NurbsCurve, NurbsSurface};
use crate::vec::{Point3, Vec3};

/// A conical screw motion of a given height about an axis (see the
/// [module docs](self)).
#[derive(Debug, Clone, Copy)]
pub struct ConicalScrewMotion {
    origin: Point3,
    axis: Vec3,
    rate: f64,
    radial_rate: f64,
    height: f64,
}

/// The span count and certified bound shared by every fit of one sweep.
#[derive(Debug, Clone, Copy)]
pub struct ConicalScrewPlan {
    /// Largest distance from the axis the plan covers, anywhere along a path.
    pub max_radius: f64,
    /// Longest direction vector the plan covers.
    pub max_direction: f64,
    /// Uniform Hermite spans along the axis.
    pub spans: usize,
    /// Proven bound, in exact arithmetic, on the distance from any fitted
    /// path or surface of this plan to the exact one.
    pub remainder_bound: f64,
    tolerance: f64,
    max_spans: usize,
}

impl ConicalScrewMotion {
    /// A motion turning `rate` radians (right-handed about `axis` for a
    /// positive rate) and moving `radial_rate` along a point's direction per
    /// unit of advance along `axis`, over `height`.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] when a value is non-finite or the
    /// height is not positive; [`MathError::ZeroVector`] for a zero axis.
    pub fn new(
        origin: Point3,
        axis: Vec3,
        rate: f64,
        radial_rate: f64,
        height: f64,
    ) -> Result<Self, MathError> {
        for value in [rate, radial_rate] {
            if !origin.0.iter().all(|c| c.is_finite()) || !value.is_finite() {
                return Err(MathError::ParameterOutOfRange {
                    value,
                    min: f64::NEG_INFINITY,
                    max: f64::INFINITY,
                });
            }
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
            radial_rate,
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

    /// Radial advance per unit of height.
    #[must_use]
    pub const fn radial_rate(&self) -> f64 {
        self.radial_rate
    }

    /// Total turn at the far end, `rate · height`.
    #[must_use]
    pub fn total_turn(&self) -> f64 {
        self.rate * self.height
    }

    /// `v` turned by the motion's turn at advance `s`.
    #[must_use]
    pub fn turn_vector(&self, v: Vec3, s: f64) -> Vec3 {
        let along = self.axis * v.dot(self.axis);
        let across = v - along;
        let (sin, cos) = (self.rate * s).sin_cos();
        along + across * cos + self.axis.cross(across) * sin
    }

    /// `p − o` split into its components along and across the axis.
    fn split(&self, p: Point3) -> (Vec3, Vec3) {
        let d = p - self.origin;
        let along = self.axis * d.dot(self.axis);
        (along, d - along)
    }

    /// `u` without its component along the axis.
    fn across(&self, u: Vec3) -> Vec3 {
        u - self.axis * u.dot(self.axis)
    }

    /// Distance of `p` from the axis.
    #[must_use]
    pub fn radius(&self, p: Point3) -> f64 {
        self.split(p).1.length()
    }

    /// The unit direction from the axis to `p`.
    ///
    /// # Errors
    ///
    /// [`MathError::ZeroVector`] for a point on the axis.
    pub fn radial(&self, p: Point3) -> Result<Vec3, MathError> {
        self.split(p).1.normalize()
    }

    /// Largest distance from the axis along the path of `(p, u)`: the
    /// distance is convex in the advance, so it is the larger of its ends'.
    #[must_use]
    pub fn reach(&self, p: Point3, u: Vec3) -> f64 {
        let across = self.split(p).1;
        let far = across + self.across(u) * (self.radial_rate * self.height);
        across.length().max(far.length())
    }

    /// `H_{p,u}(s)`: `p` moved by the motion up to advance `s`.
    #[must_use]
    pub fn point(&self, p: Point3, u: Vec3, s: f64) -> Point3 {
        self.sample(p, u, s).0
    }

    /// `H_{p,u}(s)` and its derivative.
    fn sample(&self, p: Point3, u: Vec3, s: f64) -> (Point3, Vec3) {
        let across = self.split(p).1;
        let u = self.across(u);
        let (sin, cos) = (self.rate * s).sin_cos();
        let turn = |v: Vec3| v * cos + self.axis.cross(v) * sin;
        let arm = turn(across + u * (self.radial_rate * s));
        // From `p` itself rather than its recomposed parts, so that the
        // path starts on `p` exactly: at `s = 0` the arm is `across`.
        let point = p + (arm - across) + self.axis * s;
        let derivative = self.axis.cross(arm) * self.rate + turn(u) * self.radial_rate + self.axis;
        (point, derivative)
    }

    /// `k⁴·ρ + 4·|k|³·|τ|·υ`: the fourth-derivative bound of every path that
    /// stays within `max_radius` of the axis with a direction no longer than
    /// `max_direction`.
    fn fourth_derivative_bound(&self, max_radius: f64, max_direction: f64) -> f64 {
        let k = self.rate.abs();
        k.powi(4) * max_radius + 4.0 * k.powi(3) * self.radial_rate.abs() * max_direction
    }

    /// The span count and bound for fits of paths that stay within
    /// `max_radius` of the axis (see [`Self::reach`]) with directions no
    /// longer than `max_direction`.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] for a negative or non-finite radius
    /// or direction length or a non-positive tolerance;
    /// [`MathError::ResourceLimitExceeded`] when `tolerance` needs more than
    /// `max_spans` spans.
    pub fn plan(
        &self,
        max_radius: f64,
        max_direction: f64,
        tolerance: f64,
        max_spans: usize,
    ) -> Result<ConicalScrewPlan, MathError> {
        for value in [max_radius, max_direction] {
            if !value.is_finite() || value < 0.0 {
                return Err(MathError::ParameterOutOfRange {
                    value,
                    min: 0.0,
                    max: f64::INFINITY,
                });
            }
        }
        // Fit one path with the shared bound: it fixes the spans every fit
        // of the plan uses, and its remainder bound is the plan's.
        let direction = any_perpendicular(self.axis);
        let probe = self.origin + direction * max_radius;
        let fit = self.fit(
            probe,
            Vec3::new(0.0, 0.0, 0.0),
            self.fourth_derivative_bound(max_radius, max_direction),
            tolerance,
            max_spans,
        )?;
        Ok(ConicalScrewPlan {
            max_radius,
            max_direction,
            spans: fit.spans,
            remainder_bound: fit.remainder_bound,
            tolerance,
            max_spans,
        })
    }

    fn fit(
        &self,
        p: Point3,
        u: Vec3,
        bound: f64,
        tolerance: f64,
        max_spans: usize,
    ) -> Result<crate::nurbs::hermite_fit::HermiteFit, MathError> {
        fit_cubic_hermite(
            |s| self.sample(p, u, s),
            0.0,
            self.height,
            bound,
            tolerance,
            max_spans,
        )
    }

    /// The fitted path `H̃_{p,u}` over `[0, height]`, within the plan's bound
    /// of `H_{p,u}`.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] when the path leaves the radius the
    /// plan covers or `u` is longer than the plan's directions.
    pub fn helix(
        &self,
        plan: &ConicalScrewPlan,
        p: Point3,
        u: Vec3,
    ) -> Result<NurbsCurve, MathError> {
        let reach = self.reach(p, u);
        if reach > plan.max_radius {
            return Err(MathError::ParameterOutOfRange {
                value: reach,
                min: 0.0,
                max: plan.max_radius,
            });
        }
        let length = self.across(u).length();
        if length > plan.max_direction {
            return Err(MathError::ParameterOutOfRange {
                value: length,
                min: 0.0,
                max: plan.max_direction,
            });
        }
        Ok(self
            .fit(
                p,
                u,
                self.fourth_derivative_bound(plan.max_radius, plan.max_direction),
                plan.tolerance,
                plan.max_spans,
            )?
            .curve)
    }

    /// The swept surface of `curve` carried with `directions`, one per
    /// control point: `u` runs along the motion over `[0, height]` (cubic,
    /// the plan's spans) and `v` is `curve`'s own parameter, degree, knots
    /// and weights. Within the plan's bound of the exact sweep of the pair.
    ///
    /// # Errors
    ///
    /// [`MathError::ParameterOutOfRange`] when `directions` does not match
    /// the control points or a path leaves what the plan covers, or surface
    /// construction errors.
    pub fn surface(
        &self,
        plan: &ConicalScrewPlan,
        curve: &NurbsCurve,
        directions: &[Vec3],
    ) -> Result<NurbsSurface, MathError> {
        let points = curve.control_points();
        if directions.len() != points.len() {
            #[allow(clippy::cast_precision_loss)]
            return Err(MathError::ParameterOutOfRange {
                value: directions.len() as f64,
                min: points.len() as f64,
                max: points.len() as f64,
            });
        }
        let helices = points
            .iter()
            .zip(directions)
            .map(|(&p, &u)| self.helix(plan, p, u))
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
