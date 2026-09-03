//! Radius law types for variable-radius fillets.

use crate::BlendError;
use remus_topology::edge::EdgeId;

/// Cloneable standard radius laws shared by public modeling APIs.
#[derive(Debug, Clone)]
pub enum StandardRadiusLaw {
    /// Constant radius.
    Constant(f64),
    /// Linear interpolation from `start` to `end`.
    Linear {
        /// Radius at the start of the edge.
        start: f64,
        /// Radius at the end of the edge.
        end: f64,
    },
    /// Smooth Hermite ramp: `3t² - 2t³`.
    SCurve {
        /// Radius at the start of the edge.
        start: f64,
        /// Radius at the end of the edge.
        end: f64,
    },
}

impl StandardRadiusLaw {
    /// Evaluate the radius at normalized parameter `t`, clamped to `[0, 1]`.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, 1.0);
        match self {
            Self::Constant(radius) => *radius,
            Self::Linear { start, end } => (end - start).mul_add(t, *start),
            Self::SCurve { start, end } => {
                let smooth = t * t * (-2.0f64).mul_add(t, 3.0);
                (end - start).mul_add(smooth, *start)
            }
        }
    }

    /// Exact minimum and maximum over the normalized law domain `[0, 1]`.
    ///
    /// All standard laws are monotone between their endpoint values, so no
    /// sampling is involved in this bound.
    #[must_use]
    pub fn bounds(&self) -> (f64, f64) {
        let (start, end) = match self {
            Self::Constant(radius) => (*radius, *radius),
            Self::Linear { start, end } | Self::SCurve { start, end } => (*start, *end),
        };
        if !start.is_finite() || !end.is_finite() {
            return (f64::NAN, f64::NAN);
        }
        (start.min(end), start.max(end))
    }

    /// Validate the complete normalized law domain and return its exact
    /// `(minimum, maximum)` radius.
    ///
    /// `minimum_radius` is exclusive: a radius equal to the modeling
    /// tolerance has no resolvable blend band and is refused before topology
    /// allocation.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError::InvalidInput`] when either endpoint is non-finite,
    /// unsafe for squared geometry, or the law reaches `minimum_radius`.
    pub fn validated_bounds(&self, minimum_radius: f64) -> Result<(f64, f64), BlendError> {
        let (minimum, maximum) = self.bounds();
        if !minimum_radius.is_finite() || minimum_radius < 0.0 {
            return Err(BlendError::InvalidInput {
                reason: "minimum radius must be finite and non-negative".into(),
            });
        }
        if !minimum.is_finite() || !maximum.is_finite() || maximum > f64::MAX.sqrt() {
            return Err(BlendError::InvalidInput {
                reason: "radius law must remain finite and safe for squared geometry on [0, 1]"
                    .into(),
            });
        }
        if minimum <= minimum_radius {
            return Err(BlendError::InvalidInput {
                reason: format!(
                    "radius law minimum {minimum} must be greater than {minimum_radius} on [0, 1]"
                ),
            });
        }
        Ok((minimum, maximum))
    }

    /// Validate the complete law against an edge's exclusive local radius
    /// limit.
    ///
    /// Geometry-specific callers derive `local_limit` from the two supports;
    /// keeping that derivation outside the law type lets the same law be used
    /// on edges with different clearances. Equality is refused because the
    /// contact reaches the neighbouring boundary and collapses the remaining
    /// support face.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError::InvalidInput`] for an invalid law or limit, and
    /// [`BlendError::RadiusTooLarge`] when any point in the law reaches the
    /// local limit.
    pub fn validated_for_edge(
        &self,
        edge: EdgeId,
        minimum_radius: f64,
        local_limit: f64,
    ) -> Result<(f64, f64), BlendError> {
        let bounds = self.validated_bounds(minimum_radius)?;
        if !local_limit.is_finite() || local_limit <= minimum_radius {
            return Err(BlendError::InvalidInput {
                reason: format!(
                    "local radius limit {local_limit} must be finite and greater than {minimum_radius}"
                ),
            });
        }
        if bounds.1 >= local_limit {
            return Err(BlendError::RadiusTooLarge {
                edge,
                max_radius: local_limit,
            });
        }
        Ok(bounds)
    }
}

impl From<StandardRadiusLaw> for RadiusLaw {
    fn from(law: StandardRadiusLaw) -> Self {
        match law {
            StandardRadiusLaw::Constant(radius) => Self::Constant(radius),
            StandardRadiusLaw::Linear { start, end } => Self::Linear { start, end },
            StandardRadiusLaw::SCurve { start, end } => Self::SCurve { start, end },
        }
    }
}

/// Defines how the fillet radius varies along an edge.
pub enum RadiusLaw {
    /// Constant radius.
    Constant(f64),
    /// Linear interpolation from `start` to `end`.
    Linear {
        /// Radius at the start of the edge.
        start: f64,
        /// Radius at the end of the edge.
        end: f64,
    },
    /// Smooth Hermite ramp: `3t^2 - 2t^3`.
    SCurve {
        /// Radius at the start of the edge.
        start: f64,
        /// Radius at the end of the edge.
        end: f64,
    },
    /// Custom law: boxed closure mapping `t in [0,1]` to radius.
    ///
    /// The callback must be deterministic and free of observable side effects:
    /// validation and Newton evaluation may call it repeatedly at the same
    /// parameter. Because arbitrary closure behavior cannot be proven between
    /// samples, whole-domain qualification is available only to the standard
    /// laws above.
    Custom(Box<dyn Fn(f64) -> f64 + Send + Sync>),
}

impl std::fmt::Debug for RadiusLaw {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Constant(r) => f.debug_tuple("Constant").field(r).finish(),
            Self::Linear { start, end } => f
                .debug_struct("Linear")
                .field("start", start)
                .field("end", end)
                .finish(),
            Self::SCurve { start, end } => f
                .debug_struct("SCurve")
                .field("start", start)
                .field("end", end)
                .finish(),
            Self::Custom(_) => f.debug_tuple("Custom").field(&"<fn>").finish(),
        }
    }
}

impl RadiusLaw {
    /// Evaluate the radius at parameter `t in [0, 1]`.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> f64 {
        match self {
            Self::Constant(r) => *r,
            Self::Linear { start, end } => start + (end - start) * t,
            Self::SCurve { start, end } => {
                let s = t * t * (3.0 - 2.0 * t);
                start + (end - start) * s
            }
            Self::Custom(f) => f(t),
        }
    }

    /// Exact bounds for standard laws, or `None` for an opaque callback.
    ///
    /// A custom function cannot be proven bounded from its Rust closure, so
    /// callers that need a whole-domain preflight must keep that case
    /// unqualified rather than substituting an endpoint interpolation.
    #[must_use]
    pub fn exact_bounds(&self) -> Option<(f64, f64)> {
        match self {
            Self::Constant(radius) => Some((*radius, *radius)),
            Self::Linear { start, end } | Self::SCurve { start, end } => {
                if start.is_finite() && end.is_finite() {
                    Some((start.min(*end), start.max(*end)))
                } else {
                    Some((f64::NAN, f64::NAN))
                }
            }
            Self::Custom(_) => None,
        }
    }

    /// Validate the radius evaluated at one walker station.
    ///
    /// This is the run-time guard for opaque custom laws. Standard public
    /// operations additionally use [`StandardRadiusLaw::validated_bounds`] to
    /// prove the entire domain before entering the transaction.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError::InvalidInput`] when the callback produces a
    /// non-finite, non-positive, or numerically unsafe radius.
    pub fn validate_at(&self, t: f64, minimum_radius: f64) -> Result<f64, BlendError> {
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {
            return Err(BlendError::InvalidInput {
                reason: format!("radius-law parameter {t} must be finite and in [0, 1]"),
            });
        }
        if !minimum_radius.is_finite() || minimum_radius < 0.0 {
            return Err(BlendError::InvalidInput {
                reason: "minimum radius must be finite and non-negative".into(),
            });
        }
        let radius = self.evaluate(t);
        if !radius.is_finite() || radius > f64::MAX.sqrt() {
            return Err(BlendError::InvalidInput {
                reason: format!(
                    "radius law produced a non-finite or numerically unsafe radius at t={t}"
                ),
            });
        }
        if radius <= minimum_radius {
            return Err(BlendError::InvalidInput {
                reason: format!(
                    "radius law produced {radius} at t={t}; radius must be greater than {minimum_radius}"
                ),
            });
        }
        Ok(radius)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_math::vec::Point3;
    use remus_topology::Topology;
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    #[test]
    fn constant_law_returns_same_value() {
        let law = RadiusLaw::Constant(5.0);
        assert!((law.evaluate(0.0) - 5.0).abs() < f64::EPSILON);
        assert!((law.evaluate(0.5) - 5.0).abs() < f64::EPSILON);
        assert!((law.evaluate(1.0) - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn linear_law_interpolates() {
        let law = RadiusLaw::Linear {
            start: 1.0,
            end: 3.0,
        };
        assert!((law.evaluate(0.0) - 1.0).abs() < f64::EPSILON);
        assert!((law.evaluate(0.5) - 2.0).abs() < f64::EPSILON);
        assert!((law.evaluate(1.0) - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn scurve_law_is_smooth() {
        let law = RadiusLaw::SCurve {
            start: 1.0,
            end: 3.0,
        };
        assert!((law.evaluate(0.0) - 1.0).abs() < f64::EPSILON);
        assert!((law.evaluate(1.0) - 3.0).abs() < f64::EPSILON);
        // Midpoint: 3*(0.5)^2 - 2*(0.5)^3 = 0.5
        assert!((law.evaluate(0.5) - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn standard_law_clamps_normalized_parameter() {
        let law = StandardRadiusLaw::Linear {
            start: 1.0,
            end: 3.0,
        };
        assert!((law.evaluate(-1.0) - 1.0).abs() < f64::EPSILON);
        assert!((law.evaluate(2.0) - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn standard_law_bounds_are_exact_for_both_directions() {
        for law in [
            StandardRadiusLaw::Constant(2.0),
            StandardRadiusLaw::Linear {
                start: 3.0,
                end: 1.0,
            },
            StandardRadiusLaw::SCurve {
                start: 1.0,
                end: 3.0,
            },
        ] {
            let (minimum, maximum) = law.validated_bounds(1e-7).unwrap();
            assert!((minimum - law.evaluate(0.0).min(law.evaluate(1.0))).abs() < f64::EPSILON);
            assert!((maximum - law.evaluate(0.0).max(law.evaluate(1.0))).abs() < f64::EPSILON);
            for i in 0..=100 {
                let radius = law.evaluate(f64::from(i) / 100.0);
                assert!(radius >= minimum && radius <= maximum);
            }
        }
    }

    #[test]
    fn standard_law_rejects_complete_domain_boundaries() {
        for law in [
            StandardRadiusLaw::Linear {
                start: 1.0,
                end: 0.0,
            },
            StandardRadiusLaw::Constant(1e-7),
            StandardRadiusLaw::SCurve {
                start: f64::NAN,
                end: 1.0,
            },
            StandardRadiusLaw::Constant(f64::INFINITY),
        ] {
            assert!(matches!(
                law.validated_bounds(1e-7),
                Err(BlendError::InvalidInput { .. })
            ));
        }
    }

    #[test]
    fn custom_law_is_not_claimed_as_exactly_bounded() {
        let law = RadiusLaw::Custom(Box::new(|t| 1.0 + t * t));
        assert_eq!(law.exact_bounds(), None);
        assert!((law.validate_at(0.5, 1e-7).unwrap() - 1.25).abs() < f64::EPSILON);
    }

    #[test]
    fn local_limit_is_an_exclusive_typed_boundary() {
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(start, end, remus_topology::edge::EdgeCurve::Line));

        let allowed = StandardRadiusLaw::Linear {
            start: 0.5,
            end: 1.5,
        };
        assert_eq!(
            allowed.validated_for_edge(edge, 1e-7, 2.0).unwrap(),
            (0.5, 1.5)
        );

        for law in [
            StandardRadiusLaw::Linear {
                start: 0.5,
                end: 2.0,
            },
            StandardRadiusLaw::SCurve {
                start: 3.0,
                end: 0.5,
            },
        ] {
            assert!(matches!(
                law.validated_for_edge(edge, 1e-7, 2.0),
                Err(BlendError::RadiusTooLarge {
                    edge: failed_edge,
                    max_radius: 2.0,
                }) if failed_edge == edge
            ));
        }
    }
}
