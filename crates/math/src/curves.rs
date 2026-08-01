//! Analytic 3D curve types: lines, circles, and ellipses.
//!
//! These provide exact evaluation (no NURBS approximation) for the
//! most common curve types in CAD.

use std::f64::consts::PI;

use crate::MathError;
use crate::frame::Frame3;
use crate::vec::{Point3, Vec3};

// ── Line3D ─────────────────────────────────────────────────────────

/// A 3D line defined by origin and direction.
///
/// Parameterized as `P(t) = origin + t * direction`.
#[derive(Debug, Clone)]
pub struct Line3D {
    origin: Point3,
    direction: Vec3,
}

impl Line3D {
    /// Create a new line.
    ///
    /// # Errors
    ///
    /// Returns an error if `direction` is zero-length.
    pub fn new(origin: Point3, direction: Vec3) -> Result<Self, MathError> {
        let len = direction.length();
        if len < 1e-15 {
            return Err(MathError::ZeroVector);
        }
        Ok(Self {
            origin,
            direction: Vec3::new(
                direction.x() / len,
                direction.y() / len,
                direction.z() / len,
            ),
        })
    }

    /// Evaluate the line at parameter `t`.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> Point3 {
        self.origin + self.direction * t
    }

    /// The tangent direction (constant for a line).
    #[must_use]
    pub const fn tangent(&self) -> Vec3 {
        self.direction
    }

    /// Project a point onto the line, returning the parameter.
    #[must_use]
    pub fn project(&self, point: Point3) -> f64 {
        let v = point - self.origin;
        self.direction.dot(v)
    }

    /// Distance from a point to the line.
    #[must_use]
    pub fn distance_to_point(&self, point: Point3) -> f64 {
        let v = point - self.origin;
        let proj = self.direction * self.direction.dot(v);
        (v - proj).length()
    }

    /// The line origin.
    #[must_use]
    pub const fn origin(&self) -> Point3 {
        self.origin
    }

    /// The unit direction.
    #[must_use]
    pub const fn direction(&self) -> Vec3 {
        self.direction
    }
}

// ── Circle3D ───────────────────────────────────────────────────────

/// A 3D circle defined by center, normal (axis), and radius.
///
/// Parameterized as `P(t) = center + radius*(cos(t)*u + sin(t)*v)`
/// where `u` and `v` form an orthonormal basis in the circle plane.
/// `t` ranges from 0 to 2π for a full circle.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle3D {
    center: Point3,
    normal: Vec3,
    radius: f64,
    u_axis: Vec3,
    v_axis: Vec3,
}

impl Circle3D {
    /// Create a new circle.
    ///
    /// # Errors
    ///
    /// Returns an error if `radius` is non-positive or `normal` is zero.
    pub fn new(center: Point3, normal: Vec3, radius: f64) -> Result<Self, MathError> {
        if radius <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: radius,
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        let f = Frame3::from_normal(center, normal)?;
        Ok(Self {
            center,
            normal: f.z,
            radius,
            u_axis: f.x,
            v_axis: f.y,
        })
    }

    /// Create a new circle with a caller-supplied reference x-direction.
    ///
    /// `ref_dir` is projected onto the plane perpendicular to `normal` to
    /// produce `u_axis`. Circles are radially symmetric so the choice of
    /// `u_axis` has no geometric effect — but it does fix the seam vertex
    /// at `evaluate(0.0)`, which downstream code (closed-edge construction,
    /// PCurve computation) can depend on.
    ///
    /// # Errors
    ///
    /// Returns an error if `radius` is non-positive or `normal` is zero.
    pub fn new_with_ref(
        center: Point3,
        normal: Vec3,
        radius: f64,
        ref_dir: Vec3,
    ) -> Result<Self, MathError> {
        if radius <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: radius,
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        let f = Frame3::from_normal_and_ref(center, normal, ref_dir)?;
        Ok(Self {
            center,
            normal: f.z,
            radius,
            u_axis: f.x,
            v_axis: f.y,
        })
    }

    /// Evaluate the circle at angle `t` (radians).
    #[must_use]
    pub fn evaluate(&self, t: f64) -> Point3 {
        let cos_t = t.cos();
        let sin_t = t.sin();
        self.center + self.u_axis * (self.radius * cos_t) + self.v_axis * (self.radius * sin_t)
    }

    /// Tangent at angle `t` (unit-length).
    #[must_use]
    pub fn tangent(&self, t: f64) -> Vec3 {
        let cos_t = t.cos();
        let sin_t = t.sin();
        self.u_axis * (-sin_t) + self.v_axis * cos_t
    }

    /// The circle circumference.
    #[must_use]
    pub fn circumference(&self) -> f64 {
        2.0 * PI * self.radius
    }

    /// The circle center.
    #[must_use]
    pub const fn center(&self) -> Point3 {
        self.center
    }

    /// The circle radius.
    #[must_use]
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// The circle normal (axis direction).
    #[must_use]
    pub const fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Project a point onto the circle, returning the angle parameter.
    #[must_use]
    pub fn project(&self, point: Point3) -> f64 {
        let v = point - self.center;
        let u_comp = self.u_axis.dot(v);
        let v_comp = self.v_axis.dot(v);
        v_comp.atan2(u_comp)
    }

    /// The u-axis direction (major axis in the circle plane).
    #[must_use]
    pub const fn u_axis(&self) -> Vec3 {
        self.u_axis
    }

    /// The v-axis direction (minor axis in the circle plane).
    #[must_use]
    pub const fn v_axis(&self) -> Vec3 {
        self.v_axis
    }

    /// Create a circle with explicit basis vectors (for transform/copy).
    ///
    /// # Errors
    ///
    /// Returns an error if `radius` is non-positive.
    pub fn with_axes(
        center: Point3,
        normal: Vec3,
        radius: f64,
        u_axis: Vec3,
        v_axis: Vec3,
    ) -> Result<Self, MathError> {
        if radius <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: radius,
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        Ok(Self {
            center,
            normal,
            radius,
            u_axis,
            v_axis,
        })
    }

    /// Intersect the circle with a 3D line segment.
    ///
    /// Returns up to 2 intersection points along with their angle parameter
    /// `t` on the circle. Points returned are restricted to the segment
    /// `[seg_start, seg_end]` (with `tol` slack on the endpoints).
    ///
    /// Cases:
    /// - Segment crosses the circle's plane at one point: at most 1
    ///   intersection (when that crossing is on the circle, within `tol`).
    /// - Segment lies in the circle's plane: up to 2 intersections.
    /// - Segment is parallel to the plane but offset: 0 intersections.
    ///
    /// `tol` is the absolute linear tolerance for "on the plane" and
    /// "on the circle" tests, and for clamping the segment parameter.
    #[must_use]
    pub fn intersect_segment(
        &self,
        seg_start: Point3,
        seg_end: Point3,
        tol: f64,
    ) -> Vec<(Point3, f64)> {
        let mut out = Vec::new();
        let d = seg_end - seg_start;
        let seg_len_sq = d.length_squared();
        if seg_len_sq < tol * tol {
            return out;
        }

        // Signed distance of each endpoint to the circle's plane.
        let h0 = (seg_start - self.center).dot(self.normal);
        let h1 = (seg_end - self.center).dot(self.normal);

        let on_plane = |p: Point3| -> bool {
            let v = p - self.center;
            let in_plane = v.dot(self.normal).abs() < tol;
            let r = v.length();
            in_plane && (r - self.radius).abs() < tol
        };

        // Helper: append `t_seg` (segment parameter) → intersection point with
        // `tol` slack on the endpoints; drop duplicates within `tol`.
        let mut push_if_unique = |p: Point3| {
            let v = p - self.center;
            // angle in [0, 2π)
            let mut t = v.dot(self.v_axis).atan2(v.dot(self.u_axis));
            if t < 0.0 {
                t += std::f64::consts::TAU;
            }
            if out
                .iter()
                .any(|(q, _): &(Point3, f64)| (*q - p).length() < tol)
            {
                return;
            }
            out.push((p, t));
        };

        if h0.abs() < tol && h1.abs() < tol {
            // Segment lies in the circle's plane: solve 2D line-circle.
            // Project everything into UV coordinates centered at the circle.
            let p0_u = (seg_start - self.center).dot(self.u_axis);
            let p0_v = (seg_start - self.center).dot(self.v_axis);
            let p1_u = (seg_end - self.center).dot(self.u_axis);
            let p1_v = (seg_end - self.center).dot(self.v_axis);
            let du = p1_u - p0_u;
            let dv = p1_v - p0_v;
            // |P0 + s*(P1-P0)|² = r²
            // a*s² + 2*b*s + c = 0 where
            //   a = du² + dv²
            //   b = p0_u*du + p0_v*dv
            //   c = p0_u² + p0_v² - r²
            let a = du * du + dv * dv;
            let b = p0_u * du + p0_v * dv;
            let c = p0_u * p0_u + p0_v * p0_v - self.radius * self.radius;
            let disc = b * b - a * c;
            // `disc` has units of length^4 (it's b² - a·c, both products of
            // squared coordinates). Compare against a scale-aware threshold
            // `(tol² · a)` rather than raw `tol` (which is length).
            // Negative discriminants smaller than this in magnitude are
            // floating-point noise on a tangent intersection — clamp to 0.
            if a < tol * tol || disc < -tol * tol * a {
                return out;
            }
            let disc = disc.max(0.0);
            let s_slack = tol / seg_len_sq.sqrt();
            // Near-tangent collapse. The two roots straddle the foot of the
            // circle center on the line by half_chord = sqrt(disc/a); the
            // line's penetration into the circle is δ ≈ half_chord²/(2r).
            // When δ ≤ tol the configuration is tangent AT TOLERANCE and the
            // separate roots are conditioning noise (position error grows as
            // sqrt(2rδ): a 1e-13 residual at r=4 already shifts each root a
            // full micron, minting near-duplicate vertices next to an exact
            // tangency vertex). Emit the well-conditioned double root — the
            // foot itself — instead of the noise pair.
            let sqrt_disc = disc.sqrt();
            let roots: &[f64] = if disc <= 2.0 * self.radius * tol * a {
                &[-b / a]
            } else {
                &[(-b - sqrt_disc) / a, (-b + sqrt_disc) / a]
            };
            for &s in roots {
                if s >= -s_slack && s <= 1.0 + s_slack {
                    let s = s.clamp(0.0, 1.0);
                    let p = Point3::new(
                        seg_start.x() + s * d.x(),
                        seg_start.y() + s * d.y(),
                        seg_start.z() + s * d.z(),
                    );
                    push_if_unique(p);
                }
            }
        } else if h0 * h1 <= tol * tol {
            // Segment crosses the circle's plane (or touches it). Solve
            // for the unique s where signed-distance = 0:
            //   h0 + s*(h1 - h0) = 0  →  s = h0 / (h0 - h1)
            let denom = h0 - h1;
            if denom.abs() < tol {
                return out;
            }
            let s = h0 / denom;
            let s_slack = tol / seg_len_sq.sqrt();
            if s < -s_slack || s > 1.0 + s_slack {
                return out;
            }
            let s = s.clamp(0.0, 1.0);
            let p = Point3::new(
                seg_start.x() + s * d.x(),
                seg_start.y() + s * d.y(),
                seg_start.z() + s * d.z(),
            );
            if on_plane(p) {
                push_if_unique(p);
            }
        }
        // else: segment is on one side of the plane → no crossings.

        out
    }

    /// Intersect the circle with another COPLANAR circle.
    ///
    /// Returns up to 2 intersection points along with their angle parameter
    /// `t` on `self`. Non-coplanar pairs (skew or offset planes) and
    /// coincident/concentric pairs return no points — callers own those
    /// configurations separately.
    ///
    /// Near-tangent conditioning: when the circles graze (the chord implied
    /// by the root pair penetrates by less than `tol`), the two roots are
    /// noise straddling the tangency foot — position error grows as
    /// `sqrt(2·r·δ)`, the recurring tangential-contact class. The
    /// well-conditioned double root (the foot on the center line) is emitted
    /// instead of the pair.
    #[must_use]
    pub fn intersect_circle(&self, other: &Self, tol: f64) -> Vec<(Point3, f64)> {
        let mut out = Vec::new();
        if self.normal.cross(other.normal).length() > 1e-9 {
            return out; // Skew planes — not this primitive's case.
        }
        let dvec = other.center - self.center;
        if dvec.dot(self.normal).abs() > tol {
            return out; // Parallel but offset planes.
        }
        let du = dvec.dot(self.u_axis);
        let dv = dvec.dot(self.v_axis);
        let d2 = du * du + dv * dv;
        let d = d2.sqrt();
        if d < tol {
            return out; // Concentric (incl. coincident) — no discrete crossings.
        }
        let (r1, r2) = (self.radius, other.radius);
        let a = (d2 + r1 * r1 - r2 * r2) / (2.0 * d);
        let h2 = r1 * r1 - a * a;
        let r_eff = r1.min(r2);
        if h2 < -2.0 * r_eff * tol {
            return out; // Separated (or nested) beyond the tangency well.
        }
        let ux = Vec3::new(
            (self.u_axis.x() * du + self.v_axis.x() * dv) / d,
            (self.u_axis.y() * du + self.v_axis.y() * dv) / d,
            (self.u_axis.z() * du + self.v_axis.z() * dv) / d,
        );
        let vx = self.normal.cross(ux);
        let foot = self.center + ux * a;
        let mut push = |p: Point3| {
            let v = p - self.center;
            let mut t = v.dot(self.v_axis).atan2(v.dot(self.u_axis));
            if t < 0.0 {
                t += std::f64::consts::TAU;
            }
            if !out
                .iter()
                .any(|(q, _): &(Point3, f64)| (*q - p).length() < tol)
            {
                out.push((p, t));
            }
        };
        if h2 <= 2.0 * r_eff * tol {
            push(foot);
        } else {
            let h = h2.sqrt();
            push(foot + vx * h);
            push(foot - vx * h);
        }
        out
    }
}

// ── Ellipse3D ──────────────────────────────────────────────────────

/// A 3D ellipse defined by center, normal, and two semi-axis lengths.
///
/// Parameterized as `P(t) = center + a*cos(t)*u + b*sin(t)*v`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse3D {
    center: Point3,
    normal: Vec3,
    semi_major: f64,
    semi_minor: f64,
    u_axis: Vec3,
    v_axis: Vec3,
}

impl Ellipse3D {
    /// Create a new ellipse.
    ///
    /// `semi_major` is the larger radius, `semi_minor` the smaller.
    /// The major axis lies along the `u_axis` direction (computed from normal).
    ///
    /// # Errors
    ///
    /// Returns an error if either semi-axis is non-positive.
    pub fn new(
        center: Point3,
        normal: Vec3,
        semi_major: f64,
        semi_minor: f64,
    ) -> Result<Self, MathError> {
        if semi_major <= 0.0 || semi_minor <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: semi_major.min(semi_minor),
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        if semi_minor > semi_major {
            return Err(MathError::ParameterOutOfRange {
                value: semi_minor,
                min: 0.0,
                max: semi_major,
            });
        }
        let f = Frame3::from_normal(center, normal)?;
        Ok(Self {
            center,
            normal: f.z,
            semi_major,
            semi_minor,
            u_axis: f.x,
            v_axis: f.y,
        })
    }

    /// Create a new ellipse with a caller-supplied reference major-axis direction.
    ///
    /// `ref_dir` is projected onto the plane perpendicular to `normal` to
    /// produce `u_axis` (which carries the `semi_major` extent). If
    /// `ref_dir` is parallel to `normal`, falls back to an arbitrary
    /// perpendicular choice per [`Frame3::from_normal_and_ref`].
    ///
    /// # Errors
    ///
    /// Returns an error if either semi-axis is non-positive, `semi_minor`
    /// exceeds `semi_major`, or `normal` is zero.
    pub fn new_with_ref(
        center: Point3,
        normal: Vec3,
        semi_major: f64,
        semi_minor: f64,
        ref_dir: Vec3,
    ) -> Result<Self, MathError> {
        if semi_major <= 0.0 || semi_minor <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: semi_major.min(semi_minor),
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        if semi_minor > semi_major {
            return Err(MathError::ParameterOutOfRange {
                value: semi_minor,
                min: 0.0,
                max: semi_major,
            });
        }
        let f = Frame3::from_normal_and_ref(center, normal, ref_dir)?;
        Ok(Self {
            center,
            normal: f.z,
            semi_major,
            semi_minor,
            u_axis: f.x,
            v_axis: f.y,
        })
    }

    /// Evaluate the ellipse at angle `t`.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> Point3 {
        let cos_t = t.cos();
        let sin_t = t.sin();
        self.center
            + self.u_axis * (self.semi_major * cos_t)
            + self.v_axis * (self.semi_minor * sin_t)
    }

    /// Tangent at angle `t` (not unit-length).
    #[must_use]
    pub fn tangent(&self, t: f64) -> Vec3 {
        let cos_t = t.cos();
        let sin_t = t.sin();
        self.u_axis * (-self.semi_major * sin_t) + self.v_axis * (self.semi_minor * cos_t)
    }

    /// The ellipse center.
    #[must_use]
    pub const fn center(&self) -> Point3 {
        self.center
    }

    /// Semi-major axis length.
    #[must_use]
    pub const fn semi_major(&self) -> f64 {
        self.semi_major
    }

    /// Semi-minor axis length.
    #[must_use]
    pub const fn semi_minor(&self) -> f64 {
        self.semi_minor
    }

    /// The ellipse normal (axis direction).
    #[must_use]
    pub const fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Approximate circumference using Ramanujan's formula.
    #[must_use]
    pub fn approximate_circumference(&self) -> f64 {
        let a = self.semi_major;
        let b = self.semi_minor;
        let h = (a - b) * (a - b) / ((a + b) * (a + b));
        PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (3.0f64.mul_add(-h, 4.0)).sqrt()))
    }

    /// Project a point onto the ellipse, returning the angle parameter.
    #[must_use]
    pub fn project(&self, point: Point3) -> f64 {
        let v = point - self.center;
        let u_comp = self.u_axis.dot(v) / self.semi_major;
        let v_comp = self.v_axis.dot(v) / self.semi_minor;
        v_comp.atan2(u_comp)
    }

    /// The u-axis direction (major axis direction).
    #[must_use]
    pub const fn u_axis(&self) -> Vec3 {
        self.u_axis
    }

    /// The v-axis direction (minor axis direction).
    #[must_use]
    pub const fn v_axis(&self) -> Vec3 {
        self.v_axis
    }

    /// Create an ellipse with explicit basis vectors (for transform/copy).
    ///
    /// # Errors
    ///
    /// Returns an error if either semi-axis is non-positive.
    pub fn with_axes(
        center: Point3,
        normal: Vec3,
        semi_major: f64,
        semi_minor: f64,
        u_axis: Vec3,
        v_axis: Vec3,
    ) -> Result<Self, MathError> {
        if semi_major <= 0.0 || semi_minor <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: semi_major.min(semi_minor),
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        Ok(Self {
            center,
            normal,
            semi_major,
            semi_minor,
            u_axis,
            v_axis,
        })
    }
}

/// A 3D parabola defined by vertex, axis direction, and focal length.
///
/// Parameterized as `P(t) = vertex + (t²/(4f)) * axis_dir + t * u_axis`
/// where `f` is the focal length and `u_axis` is perpendicular to the axis
/// in the parabola plane.
///
/// The parameter `t` ranges over all reals; `t = 0` is the vertex.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Parabola3D {
    vertex: Point3,
    axis_dir: Vec3,
    focal_length: f64,
    u_axis: Vec3,
}

impl Parabola3D {
    /// Creates a new parabola.
    ///
    /// `axis_dir` is the direction from vertex toward the interior of the
    /// parabola (the axis of symmetry). `focal_length` is the distance
    /// from vertex to focus.
    ///
    /// # Errors
    /// Returns an error if `focal_length` is not positive or `axis_dir` is zero.
    pub fn new(vertex: Point3, axis_dir: Vec3, focal_length: f64) -> Result<Self, MathError> {
        if focal_length <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: focal_length,
                min: f64::EPSILON,
                max: f64::MAX,
            });
        }
        let f = Frame3::from_normal(vertex, axis_dir)?;
        Ok(Self {
            vertex,
            axis_dir: f.z,
            focal_length,
            u_axis: f.x,
        })
    }

    /// Creates a parabola with an explicit in-plane `u_axis`.
    ///
    /// [`Parabola3D::new`] derives `u_axis` from an arbitrary perpendicular
    /// of `axis_dir`, which pins the symmetry axis but leaves the parabola's
    /// *plane* unspecified — two parabolas sharing a vertex, axis and focal
    /// length but lying in different planes are different point sets. Use
    /// this constructor whenever the plane is known (recognition, import,
    /// transform) so the orientation survives the round trip.
    ///
    /// `u_axis` is orthogonalized against `axis_dir`; only its component
    /// perpendicular to the axis is retained.
    ///
    /// # Errors
    /// Returns an error if `focal_length` is not positive, `axis_dir` is
    /// zero, or `u_axis` is zero or parallel to `axis_dir`.
    pub fn with_axes(
        vertex: Point3,
        axis_dir: Vec3,
        u_axis: Vec3,
        focal_length: f64,
    ) -> Result<Self, MathError> {
        if focal_length <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: focal_length,
                min: f64::EPSILON,
                max: f64::MAX,
            });
        }
        let axis = axis_dir.normalize()?;
        // Gram-Schmidt: drop the component of u along the axis. If u is
        // parallel to the axis the remainder is zero and `normalize`
        // reports `ZeroVector` rather than silently inventing a plane.
        let u = (u_axis - axis * u_axis.dot(axis)).normalize()?;
        Ok(Self {
            vertex,
            axis_dir: axis,
            focal_length,
            u_axis: u,
        })
    }

    /// Projects a point onto the parabola, returning the parameter `t`.
    ///
    /// Exact inverse of [`Parabola3D::evaluate`] for on-curve points: the
    /// parameterization is `P(t) = vertex + (t²/4f)·axis + t·u_axis`, so
    /// `t = (P − vertex)·u_axis`. The result is scale-covariant (`t` carries
    /// units of length) and involves no tolerance. For off-curve points this
    /// returns the parameter matching the point's `u_axis` coordinate, which
    /// is not in general the closest point on the curve.
    #[must_use]
    pub fn project(&self, point: Point3) -> f64 {
        (point - self.vertex).dot(self.u_axis)
    }

    /// The normal of the plane containing the parabola.
    ///
    /// Equal to `axis_dir × u_axis`, a unit vector.
    #[must_use]
    pub fn normal(&self) -> Vec3 {
        self.axis_dir.cross(self.u_axis)
    }

    /// Intersection of the tangent lines at parameters `t0` and `t1`.
    ///
    /// This is the middle control point of the arc's exact degree-2 Bézier
    /// form. Because a parabola is convex, the arc `[t0, t1]` lies inside
    /// the triangle `P(t0), tangent_intersection(t0, t1), P(t1)` — which
    /// makes those three points an exact convex hull for bounding-box and
    /// culling purposes, with no sampling and no tolerance.
    #[must_use]
    pub fn tangent_intersection(&self, t0: f64, t1: f64) -> Point3 {
        self.vertex
            + self.axis_dir * (t0 * t1 / (4.0 * self.focal_length))
            + self.u_axis * f64::midpoint(t0, t1)
    }

    /// Smallest radius of curvature over the parameter span `[t0, t1]`.
    ///
    /// Curvature is maximal at the vertex (`t = 0`, `κ = 1/2f`) and
    /// decreases monotonically with `|t|`, so the tightest point of any
    /// span is whichever of `0`, `t0`, `t1` lies in the span and is closest
    /// to zero. Carries units of length.
    #[must_use]
    pub fn min_curvature_radius(&self, t0: f64, t1: f64) -> f64 {
        let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        let k = self.curvature(0.0_f64.clamp(lo, hi));
        if k > 0.0 && k.is_finite() {
            1.0 / k
        } else {
            f64::INFINITY
        }
    }

    /// Exact arc length of the parabola between parameters `t0` and `t1`.
    ///
    /// For `P(t) = vertex + (t²/4f)·axis + t·u_axis` the speed is
    /// `|P'(t)| = sqrt(1 + (t/2f)²)`, whose antiderivative is
    /// `f·(s·sqrt(1+s²) + asinh(s))` with `s = t/(2f)`. Returned length is
    /// always non-negative.
    #[must_use]
    pub fn arc_length(&self, t0: f64, t1: f64) -> f64 {
        let two_f = 2.0 * self.focal_length;
        // F(t) = f * ( s*sqrt(1+s^2) + asinh(s) ), s = t / (2f)
        let antideriv = |t: f64| {
            let s = t / two_f;
            self.focal_length * s.mul_add((1.0 + s * s).sqrt(), s.asinh())
        };
        (antideriv(t1) - antideriv(t0)).abs()
    }

    /// Evaluates the parabola at parameter `t`.
    ///
    /// At `t = 0` this returns the vertex.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> Point3 {
        let along_axis = (t * t) / (4.0 * self.focal_length);
        self.vertex + self.axis_dir * along_axis + self.u_axis * t
    }

    /// Returns the tangent vector at parameter `t`.
    #[must_use]
    pub fn tangent(&self, t: f64) -> Vec3 {
        let d_axis = t / (2.0 * self.focal_length);
        self.axis_dir * d_axis + self.u_axis
    }

    /// Returns the curvature at parameter `t`.
    #[must_use]
    pub fn curvature(&self, t: f64) -> f64 {
        let two_f = 2.0 * self.focal_length;
        let ratio = t / two_f;
        let denom = ratio.mul_add(ratio, 1.0);
        1.0 / (two_f * denom.powf(1.5))
    }

    /// Returns the vertex.
    #[must_use]
    pub const fn vertex(&self) -> Point3 {
        self.vertex
    }

    /// Returns the focal length.
    #[must_use]
    pub const fn focal_length(&self) -> f64 {
        self.focal_length
    }

    /// Returns the axis direction (normalized).
    #[must_use]
    pub const fn axis_dir(&self) -> Vec3 {
        self.axis_dir
    }

    /// Returns the in-plane u-axis (perpendicular to `axis_dir`).
    /// At parameter `t`, the parabola is offset by `t * u_axis` from
    /// the symmetry axis.
    #[must_use]
    pub const fn u_axis(&self) -> Vec3 {
        self.u_axis
    }

    /// Returns the focus point.
    #[must_use]
    pub fn focus(&self) -> Point3 {
        self.vertex + self.axis_dir * self.focal_length
    }
}

/// A 3D hyperbola defined by center, axis, and two semi-axis lengths.
///
/// Parameterized as `P(t) = center + a * cosh(t) * u_axis + b * sinh(t) * v_axis`.
///
/// The parameter `t` ranges over all reals; `t = 0` gives the vertex
/// closest to center on the positive branch.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Hyperbola3D {
    center: Point3,
    normal: Vec3,
    semi_major: f64,
    semi_minor: f64,
    u_axis: Vec3,
    v_axis: Vec3,
}

impl Hyperbola3D {
    /// Creates a new hyperbola.
    ///
    /// `semi_major` is the real semi-axis (distance from center to vertex),
    /// `semi_minor` is the imaginary semi-axis.
    ///
    /// # Errors
    /// Returns an error if either semi-axis is non-positive.
    pub fn new(
        center: Point3,
        normal: Vec3,
        semi_major: f64,
        semi_minor: f64,
    ) -> Result<Self, MathError> {
        if semi_major <= 0.0 || semi_minor <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: semi_major.min(semi_minor),
                min: f64::EPSILON,
                max: f64::MAX,
            });
        }
        let f = Frame3::from_normal(center, normal)?;
        Ok(Self {
            center,
            normal: f.z,
            semi_major,
            semi_minor,
            u_axis: f.x,
            v_axis: f.y,
        })
    }

    /// Creates a hyperbola with an explicit real-axis direction.
    ///
    /// [`Hyperbola3D::new`] derives `u_axis` from an arbitrary perpendicular
    /// of `normal`, so the branch is rotated to an unspecified direction
    /// within its plane — a different point set from the intended one
    /// whenever the real axis is not that arbitrary choice. Use this
    /// constructor whenever the real axis is known.
    ///
    /// `u_axis` is orthogonalized against `normal`; only its in-plane
    /// component is retained. `v_axis` is `normal × u_axis`.
    ///
    /// # Errors
    /// Returns an error if either semi-axis is non-positive, `normal` is
    /// zero, or `u_axis` is zero or parallel to `normal`.
    pub fn with_axes(
        center: Point3,
        normal: Vec3,
        u_axis: Vec3,
        semi_major: f64,
        semi_minor: f64,
    ) -> Result<Self, MathError> {
        if semi_major <= 0.0 || semi_minor <= 0.0 {
            return Err(MathError::ParameterOutOfRange {
                value: semi_major.min(semi_minor),
                min: f64::EPSILON,
                max: f64::MAX,
            });
        }
        let n = normal.normalize()?;
        let u = (u_axis - n * u_axis.dot(n)).normalize()?;
        Ok(Self {
            center,
            normal: n,
            semi_major,
            semi_minor,
            u_axis: u,
            v_axis: n.cross(u),
        })
    }

    /// Projects a point onto the hyperbola, returning the parameter `t`.
    ///
    /// Exact inverse of [`Hyperbola3D::evaluate`] for on-curve points: with
    /// `P(t) = center + a·cosh(t)·u + b·sinh(t)·v`, the `v` coordinate gives
    /// `sinh(t) = ((P − center)·v)/b`, so `t = asinh(…)`. `asinh` is a
    /// bijection on ℝ, so the branch is unambiguous and no tolerance is
    /// involved. The result is dimensionless and scale-invariant: scaling
    /// the hyperbola and the point together leaves `t` unchanged.
    #[must_use]
    pub fn project(&self, point: Point3) -> f64 {
        ((point - self.center).dot(self.v_axis) / self.semi_minor).asinh()
    }

    /// Curvature at parameter `t`.
    ///
    /// For `r(t) = (a·cosh t, b·sinh t)` the cross product `r' × r''` is the
    /// constant `−ab`, so `κ(t) = ab / |r'(t)|³`. Curvature carries units of
    /// 1/length and is largest at the vertex (`t = 0`, `κ = a/b²`).
    #[must_use]
    pub fn curvature(&self, t: f64) -> f64 {
        let speed = (self.semi_major * t.sinh()).hypot(self.semi_minor * t.cosh());
        if speed <= 0.0 {
            return f64::INFINITY;
        }
        self.semi_major * self.semi_minor / (speed * speed * speed)
    }

    /// Smallest radius of curvature over the parameter span `[t0, t1]`.
    ///
    /// Curvature is maximal at the vertex (`t = 0`) and decreases
    /// monotonically with `|t|`, so the tightest point of any span is
    /// whichever of `0`, `t0`, `t1` lies in the span and is closest to zero.
    /// The result carries units of length and scales with the model, which
    /// makes it a safe extent-relative basis for tessellation density.
    #[must_use]
    pub fn min_curvature_radius(&self, t0: f64, t1: f64) -> f64 {
        let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        let t_star = 0.0_f64.clamp(lo, hi);
        let k = self.curvature(t_star);
        if k > 0.0 && k.is_finite() {
            1.0 / k
        } else {
            f64::INFINITY
        }
    }

    /// Arc length of the hyperbola between parameters `t0` and `t1`.
    ///
    /// The speed is `|H'(t)| = sqrt(a²sinh²t + b²cosh²t)`, an incomplete
    /// elliptic integral with no elementary antiderivative — unlike
    /// [`Parabola3D::arc_length`], this is quadrature, not a closed form.
    /// The span is split into chunks of at most `0.5` in `t` (a
    /// dimensionless parameter, so the chunking is scale-invariant) and
    /// each chunk gets a 20-point Gauss–Legendre rule, which resolves the
    /// integrand to near machine precision. Returned length is always
    /// non-negative and scales linearly with the model.
    #[must_use]
    pub fn arc_length(&self, t0: f64, t1: f64) -> f64 {
        use crate::quadrature::gauss_legendre_points;
        const MAX_CHUNK: f64 = 0.5;
        let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        let span = hi - lo;
        if !span.is_finite() || span <= 0.0 {
            return 0.0;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let chunks = ((span / MAX_CHUNK).ceil() as usize).max(1);
        #[allow(clippy::cast_precision_loss)]
        let dt = span / chunks as f64;
        let speed = |t: f64| {
            let (sh, ch) = (t.sinh(), t.cosh());
            (self.semi_major * sh).hypot(self.semi_minor * ch)
        };
        let mut total = 0.0;
        for k in 0..chunks {
            #[allow(clippy::cast_precision_loss)]
            let a = dt.mul_add(k as f64, lo);
            let b = a + dt;
            let half = 0.5 * (b - a);
            let mid = f64::midpoint(a, b);
            for gp in gauss_legendre_points(20) {
                total += gp.w * half * speed(half.mul_add(gp.x, mid));
            }
        }
        total
    }

    /// Intersection of the tangent lines at parameters `t0` and `t1`.
    ///
    /// This is the middle control point of the arc's exact rational
    /// degree-2 Bézier form (Piegl–Tiller §7.4). A single hyperbola branch
    /// is convex, so the arc `[t0, t1]` lies inside the triangle
    /// `H(t0), tangent_intersection(t0, t1), H(t1)` — an exact convex hull
    /// for bounding and culling, with no sampling and no tolerance.
    #[must_use]
    pub fn tangent_intersection(&self, t0: f64, t1: f64) -> Point3 {
        let tanh_b = (0.5 * (t1 - t0)).tanh();
        let x = self.semi_major * t0.cosh().mul_add(1.0, tanh_b * t0.sinh());
        let y = self.semi_minor * t0.sinh().mul_add(1.0, tanh_b * t0.cosh());
        self.center + self.u_axis * x + self.v_axis * y
    }

    /// Evaluates the hyperbola at parameter `t`.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> Point3 {
        self.center
            + self.u_axis * (self.semi_major * t.cosh())
            + self.v_axis * (self.semi_minor * t.sinh())
    }

    /// Returns the tangent vector at parameter `t`.
    #[must_use]
    pub fn tangent(&self, t: f64) -> Vec3 {
        self.u_axis * (self.semi_major * t.sinh()) + self.v_axis * (self.semi_minor * t.cosh())
    }

    /// Returns the center.
    #[must_use]
    pub const fn center(&self) -> Point3 {
        self.center
    }

    /// Returns the semi-major axis (real axis).
    #[must_use]
    pub const fn semi_major(&self) -> f64 {
        self.semi_major
    }

    /// Returns the semi-minor axis (imaginary axis).
    #[must_use]
    pub const fn semi_minor(&self) -> f64 {
        self.semi_minor
    }

    /// Returns the normal (axis perpendicular to the hyperbola plane).
    #[must_use]
    pub const fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Returns the in-plane u-axis (real semi-axis direction).
    /// At parameter `t`, the hyperbola is at offset
    /// `semi_major * cosh(t) * u_axis + semi_minor * sinh(t) * v_axis`
    /// from the center.
    #[must_use]
    pub const fn u_axis(&self) -> Vec3 {
        self.u_axis
    }

    /// Returns the in-plane v-axis (imaginary semi-axis direction).
    #[must_use]
    pub const fn v_axis(&self) -> Vec3 {
        self.v_axis
    }

    /// Returns the eccentricity: `e = sqrt(1 + (b/a)²)`.
    #[must_use]
    pub fn eccentricity(&self) -> f64 {
        let ratio = self.semi_minor / self.semi_major;
        ratio.mul_add(ratio, 1.0).sqrt()
    }

    /// Returns the two foci.
    #[must_use]
    pub fn foci(&self) -> (Point3, Point3) {
        let c = self.semi_major.hypot(self.semi_minor);
        (
            self.center + self.u_axis * c,
            self.center + self.u_axis * (-c),
        )
    }
}

#[cfg(test)]
mod tests;
