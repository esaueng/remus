//! Radius law types for variable-radius fillets.

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
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

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
}
