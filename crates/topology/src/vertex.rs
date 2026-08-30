//! Vertex — a point in 3D space with an associated tolerance.

use remus_math::vec::Point3;

use crate::TopologyError;
use crate::arena;

/// Typed handle for a [`Vertex`] stored in an [`Arena`](crate::Arena).
pub type VertexId = arena::Id<Vertex>;

/// A topological vertex: a point location with a tolerance ball.
///
/// Two geometric points that fall within `tolerance` of each other
/// are considered the same vertex.
#[derive(Debug, Clone)]
pub struct Vertex {
    /// Position of the vertex in model space.
    point: Point3,
    /// Radius of the tolerance ball around the vertex point.
    tolerance: f64,
}

impl Vertex {
    /// Creates a new vertex at the given point with the specified tolerance.
    #[must_use]
    pub const fn new(point: Point3, tolerance: f64) -> Self {
        Self { point, tolerance }
    }

    /// Returns the position of this vertex.
    #[must_use]
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Returns the tolerance of this vertex.
    #[must_use]
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Sets the position of this vertex.
    pub const fn set_point(&mut self, point: Point3) {
        self.point = point;
    }

    /// Sets the radius of the vertex's tolerance ball (RFC 0004, Stage 1).
    ///
    /// Operations may **raise** a tolerance and nothing else; the value is a
    /// claim, and this setter only guards its sanity: it must be finite and
    /// non-negative, or a predicate could never compare it honestly. Whether
    /// the claim actually covers the ball-containment invariant — every
    /// incident edge end's curve evaluation inside the ball — is the
    /// validator's job, not the setter's: see
    /// [`crate::validation::validate_vertex_ball`], and record every raise
    /// in the journal as an [`EntityEvent::Modified`](crate::journal::EntityEvent::Modified).
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::InvalidToleranceValue`] when `tolerance` is
    /// non-finite or negative; the previous value is left unchanged.
    pub fn set_tolerance(&mut self, tolerance: f64) -> Result<(), TopologyError> {
        if !tolerance.is_finite() || tolerance.is_sign_negative() {
            return Err(TopologyError::InvalidToleranceValue {
                entity: "vertex",
                value: tolerance,
            });
        }
        self.tolerance = tolerance;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

    use super::*;

    #[test]
    fn set_tolerance_stores_a_sane_raise() {
        let mut v = Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7);
        v.set_tolerance(2e-5).unwrap();
        assert_eq!(v.tolerance(), 2e-5);

        // Zero is a legal claim of exactness.
        v.set_tolerance(0.0).unwrap();
        assert_eq!(v.tolerance(), 0.0);
    }

    #[test]
    fn set_tolerance_rejects_non_finite_and_negative_balls() {
        let mut v = Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7);

        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1e-9] {
            let err = v.set_tolerance(bad).unwrap_err();
            assert!(matches!(
                err,
                crate::TopologyError::InvalidToleranceValue {
                    entity: "vertex",
                    ..
                }
            ));
        }

        // A rejected raise leaves the stored ball unchanged.
        assert_eq!(v.tolerance(), 1e-7);
    }
}
