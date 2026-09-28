//! Conservative finite-domain bounds with explicit confidence.
//!
//! This module provides the PERF-Q05 bound utilities: axis-aligned boxes that
//! are *certified* to contain a trimmed curve span or surface patch, together
//! with an explicit [`BoundConfidence`] that tells the caller whether the box
//! may be used for distance pruning.
//!
//! # Confidence contract
//!
//! - [`BoundConfidence::Conservative`] — every point of the requested span or
//!   patch lies inside [`FiniteBound::aabb`]. A distance lower bound derived
//!   from the box is mathematically justified and the face may be pruned.
//! - [`BoundConfidence::Unknown`] — no tight bound could be established
//!   (unbounded carrier with no finite trim, non-finite input, or an invalid
//!   weight/knot/trim configuration). The accompanying box is a *fallback*
//!   that is never empty (typically the whole-space box, whose distance to
//!   any query point is zero so it can never prune) and the face must be kept
//!   on the mandatory exhaustive side path.
//!
//! An unknown bound never means an empty box: an empty box would report a
//! large distance to every query point and wrongly prune the face it claims
//! to bound.
//!
//! # Construction principles (proof sketches)
//!
//! - *Line segments*: the box of the two endpoints is exact.
//! - *Circle/ellipse arcs* (`curve::circle_arc_bounds`,
//!   `curve::ellipse_arc_bounds`): each world coordinate along the arc is a
//!   sinusoid `c + R·cos(t − φ)` with closed-form amplitude `R` and phase `φ`
//!   derived from the carrier frame. Extrema can only occur at the span ends
//!   or at the phases `φ`, `φ + π` (modulo full turns); boxing those finitely
//!   many points contains the arc. The formulas use the world-axis
//!   projections of the carrier frame, so tilted carriers are handled
//!   exactly. Reversed trims are normalized to ascending spans (bounds are
//!   set-based; traversal direction is irrelevant). Anchored full turns such
//!   as `[2.8, 2.8 + 2π]` compare equal to a full turn up to roundoff and
//!   take the whole-circle extent, which is a superset either way.
//! - *Parabola/hyperbola arcs* (`curve::parabola_arc_bounds`,
//!   `curve::hyperbola_arc_bounds`): each world coordinate is a quadratic
//!   (parabola) or a `P·eᵗ + Q·e⁻ᵗ` combination (hyperbola) with at most one
//!   closed-form stationary point per axis; boxing the ends plus any interior
//!   stationary point contains the arc over a finite span. A non-finite
//!   evaluation (overflow at large `|t|`) refuses with `Unknown` instead of
//!   boxing a subset.
//! - *NURBS spans* (`curve::nurbs_curve_bounds`,
//!   `surface::nurbs_surface_bounds`): with strictly positive finite weights,
//!   the (rational) span is a convex combination of its supporting control
//!   points (Piegl & Tiller, *The NURBS Book*, Property P7.3 / Eq. 4.90: local
//!   support plus the partition of unity, preserved under the perspective
//!   divide for positive weights), so the box of the control points whose
//!   basis support overlaps the clipped span contains the span. Whole-domain
//!   requests degenerate to the full control hull. Non-positive or
//!   non-finite weights, non-finite knots/control points, or a span that
//!   clips to empty refuse with `Unknown`.
//! - *Analytic patches* (`surface::*`): whole-carrier finite boxes (sphere,
//!   torus) and full-turn slabs over a finite axial range (cylinder, cone)
//!   are exact supersets of any trim they clip.
//!
//! Sampling alone never certifies containment: every conservative box above
//! is derived from closed-form extrema or control hulls. Dense sampling
//! appears only in tests, as a supplement to the proofs, never as the proof.
//!
//! # What this module does not do
//!
//! These boxes bound the *carrier geometry over a parameter span*. Restricting
//! a box to a trimmed face (wire loops, holes) is the check layer's job
//! (`remus-check` face bounds), which unions edge-span boxes with the
//! carrier critical points that can carry a coordinate extremum inside the
//! trim. Global minimization of a NURBS narrow phase is likewise out of
//! scope: a conservative box makes *pruning* safe, it does not make a local
//! Newton projection globally exact.

pub mod curve;
pub mod surface;

use remus_math::aabb::Aabb3;
use remus_math::vec::Point3;

/// Confidence attached to a [`FiniteBound`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundConfidence {
    /// The box is certified to contain the requested span or patch and may
    /// be used to prune distance candidates.
    Conservative,
    /// No tight bound could be established. The box is a non-empty fallback
    /// (see module docs) and the face must stay on the exhaustive path.
    Unknown {
        /// Machine-readable reason code (`"non_finite_input"`,
        /// `"unbounded_carrier"`, `"invalid_weights"`, `"invalid_knots"`,
        /// `"empty_span"`, `"non_finite_control_net"`, ...).
        reason: &'static str,
    },
}

/// An axis-aligned bound over a finite parameter span or patch.
#[derive(Debug, Clone, Copy)]
pub struct FiniteBound {
    /// The bounding box. For [`BoundConfidence::Unknown`] this is a fallback
    /// that is never empty; it must not be used for pruning.
    aabb: Aabb3,
    /// Whether the box is certified conservative or an unknown fallback.
    confidence: BoundConfidence,
}

impl FiniteBound {
    /// A certified conservative box.
    #[must_use]
    pub const fn conservative(aabb: Aabb3) -> Self {
        Self {
            aabb,
            confidence: BoundConfidence::Conservative,
        }
    }

    /// An unknown fallback box with an explicit reason.
    ///
    /// Callers must ensure `aabb` is non-empty; use
    /// [`FiniteBound::infinite_unknown`] when no finite fallback exists.
    #[must_use]
    pub const fn unknown(aabb: Aabb3, reason: &'static str) -> Self {
        Self {
            aabb,
            confidence: BoundConfidence::Unknown { reason },
        }
    }

    /// The whole-space fallback: contains every finite point, has distance
    /// zero to every query point, and therefore can never justify pruning.
    #[must_use]
    pub fn infinite_unknown(reason: &'static str) -> Self {
        Self {
            aabb: whole_space_box(),
            confidence: BoundConfidence::Unknown { reason },
        }
    }

    /// The bounding box (valid for pruning only when
    /// [`FiniteBound::is_prunable`] holds).
    #[must_use]
    pub const fn aabb(&self) -> Aabb3 {
        self.aabb
    }

    /// The confidence attached to this bound.
    #[must_use]
    pub const fn confidence(&self) -> BoundConfidence {
        self.confidence
    }

    /// Whether the box may be used to prune distance candidates.
    #[must_use]
    pub const fn is_prunable(&self) -> bool {
        matches!(self.confidence, BoundConfidence::Conservative)
    }
}

/// The whole-space box used as the unknown fallback.
///
/// Distance from any finite query point to this box is zero, so it never
/// prunes. It is never empty.
fn whole_space_box() -> Aabb3 {
    Aabb3 {
        min: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        max: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
    }
}

/// Whether every coordinate of every point is finite.
fn all_points_finite(points: &[Point3]) -> bool {
    points
        .iter()
        .all(|p| p.x().is_finite() && p.y().is_finite() && p.z().is_finite())
}

/// Box the given points, or `None` when the set is empty or non-finite.
fn box_of_points(points: &[Point3]) -> Option<Aabb3> {
    if points.is_empty() || !all_points_finite(points) {
        return None;
    }
    let mut min = points[0];
    let mut max = points[0];
    for &p in &points[1..] {
        min = Point3::new(min.x().min(p.x()), min.y().min(p.y()), min.z().min(p.z()));
        max = Point3::new(max.x().max(p.x()), max.y().max(p.y()), max.z().max(p.z()));
    }
    Some(Aabb3 { min, max })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn whole_space_box_has_zero_distance_and_is_not_empty() {
        let query = Point3::new(1.0, 2.0, 3.0);
        let distance_sq = whole_space_box().distance_squared_to_point(query);
        assert!(
            distance_sq <= 0.0,
            "unknown fallback must never prune, got {distance_sq}"
        );
    }

    #[test]
    fn unknown_confidence_is_not_prunable() {
        let bound = FiniteBound::infinite_unknown("test");
        assert!(!bound.is_prunable());
        assert!(matches!(
            bound.confidence(),
            BoundConfidence::Unknown { .. }
        ));
    }

    #[test]
    fn conservative_confidence_is_prunable() {
        let aabb = Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(1.0, 1.0, 1.0),
        };
        let bound = FiniteBound::conservative(aabb);
        assert!(bound.is_prunable());
    }
}
