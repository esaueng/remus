//! Radius law types for variable-radius fillets.

use crate::BlendError;
use remus_topology::edge::EdgeId;

/// How a [`StandardRadiusLaw::Stations`] law runs between two neighbouring
/// stations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationBlend {
    /// Straight interpolation: the radius is continuous at every station and
    /// its slope may change there.
    Linear,
    /// The `3s² - 2s³` ramp on each segment: zero slope at every station, so
    /// the radius is continuously differentiable along the whole edge.
    SCurve,
}

/// One cubic piece of a standard law over `[t0, t1]` of the normalized
/// domain, with the radius and its slope `dr/dt` at both ends.
///
/// Every standard law is a cubic polynomial on each piece, so these four
/// numbers are the law there exactly (cubic Hermite form), not a fit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadiusLawSegment {
    /// Normalized start parameter.
    pub t0: f64,
    /// Normalized end parameter.
    pub t1: f64,
    /// Radius at `t0`.
    pub r0: f64,
    /// Radius at `t1`.
    pub r1: f64,
    /// `dr/dt` at `t0`, taken from inside the piece.
    pub slope0: f64,
    /// `dr/dt` at `t1`, taken from inside the piece.
    pub slope1: f64,
}

/// Smallest normalized spacing allowed between neighbouring stations.
pub const MIN_STATION_SPACING: f64 = 1.0e-6;

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
    /// Piecewise law through radius stations `(t, r)`.
    ///
    /// The first station is at `t = 0` and the last at `t = 1`; parameters
    /// increase strictly, at least [`MIN_STATION_SPACING`] apart. Each
    /// segment is monotone between its two station radii under `blend`, so
    /// the law's extrema are station radii. Malformed stations are refused by
    /// [`Self::validated_bounds`].
    Stations {
        /// The segment shape between neighbouring stations.
        blend: StationBlend,
        /// `(normalized parameter, radius)` pairs, endpoints included.
        stations: Vec<(f64, f64)>,
    },
}

impl StandardRadiusLaw {
    /// The exact cubic pieces of this law, in order, covering `[0, 1]`.
    ///
    /// A malformed station list (fewer than two stations) yields no pieces;
    /// [`Self::validated_bounds`] refuses it first on every public path.
    #[must_use]
    pub fn segments(&self) -> Vec<RadiusLawSegment> {
        let piece = |t0: f64, t1: f64, r0: f64, r1: f64, blend: StationBlend| {
            let slope = match blend {
                StationBlend::Linear => (r1 - r0) / (t1 - t0),
                StationBlend::SCurve => 0.0,
            };
            RadiusLawSegment {
                t0,
                t1,
                r0,
                r1,
                slope0: slope,
                slope1: slope,
            }
        };
        match self {
            Self::Constant(radius) => vec![piece(0.0, 1.0, *radius, *radius, StationBlend::Linear)],
            Self::Linear { start, end } => {
                vec![piece(0.0, 1.0, *start, *end, StationBlend::Linear)]
            }
            Self::SCurve { start, end } => {
                vec![piece(0.0, 1.0, *start, *end, StationBlend::SCurve)]
            }
            Self::Stations { blend, stations } => stations
                .windows(2)
                .map(|pair| piece(pair[0].0, pair[1].0, pair[0].1, pair[1].1, *blend))
                .collect(),
        }
    }

    /// Interior station parameters, strictly inside `(0, 1)`.
    #[must_use]
    pub fn breakpoints(&self) -> Vec<f64> {
        match self {
            Self::Stations { stations, .. } if stations.len() > 2 => stations
                [1..stations.len() - 1]
                .iter()
                .map(|(t, _)| *t)
                .collect(),
            _ => Vec::new(),
        }
    }

    /// The piece containing `t`; at a station the later piece wins, except
    /// at `t = 1`.
    fn segment_at(&self, t: f64) -> Option<RadiusLawSegment> {
        let segments = self.segments();
        segments
            .iter()
            .copied()
            .find(|segment| t < segment.t1)
            .or_else(|| segments.last().copied())
    }

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
            Self::Stations { blend, .. } => {
                let Some(segment) = self.segment_at(t) else {
                    return f64::NAN;
                };
                let s = ((t - segment.t0) / (segment.t1 - segment.t0)).clamp(0.0, 1.0);
                let shape = match blend {
                    StationBlend::Linear => s,
                    StationBlend::SCurve => s * s * (-2.0f64).mul_add(s, 3.0),
                };
                (segment.r1 - segment.r0).mul_add(shape, segment.r0)
            }
        }
    }

    /// Derivative with respect to the normalized law parameter.
    ///
    /// At an interior station of a linear [`Self::Stations`] law the slope
    /// is taken from the later piece.
    #[must_use]
    pub fn derivative(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, 1.0);
        match self {
            Self::Constant(_) => 0.0,
            Self::Linear { start, end } => end - start,
            Self::SCurve { start, end } => (end - start) * 6.0 * t * (1.0 - t),
            Self::Stations { blend, .. } => {
                let Some(segment) = self.segment_at(t) else {
                    return f64::NAN;
                };
                let span = segment.t1 - segment.t0;
                match blend {
                    StationBlend::Linear => (segment.r1 - segment.r0) / span,
                    StationBlend::SCurve => {
                        let s = ((t - segment.t0) / span).clamp(0.0, 1.0);
                        (segment.r1 - segment.r0) * 6.0 * s * (1.0 - s) / span
                    }
                }
            }
        }
    }

    /// Exact minimum and maximum over the normalized law domain `[0, 1]`.
    ///
    /// Every standard law, and every piece of a station law, is monotone
    /// between its endpoint values, so no sampling is involved in this bound.
    #[must_use]
    pub fn bounds(&self) -> (f64, f64) {
        let (start, end) = match self {
            Self::Constant(radius) => (*radius, *radius),
            Self::Linear { start, end } | Self::SCurve { start, end } => (*start, *end),
            Self::Stations { stations, .. } => {
                if stations.is_empty() || stations.iter().any(|(_, r)| !r.is_finite()) {
                    return (f64::NAN, f64::NAN);
                }
                return stations
                    .iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (_, r)| {
                        (lo.min(*r), hi.max(*r))
                    });
            }
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
        if let Self::Stations { stations, .. } = self {
            validate_stations(stations)?;
        }
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

/// Structural check of a [`StandardRadiusLaw::Stations`] list.
fn validate_stations(stations: &[(f64, f64)]) -> Result<(), BlendError> {
    let invalid = |reason: String| Err(BlendError::InvalidInput { reason });
    if stations.len() < 2 {
        return invalid("a station radius law needs at least its two end stations".into());
    }
    if stations.iter().any(|(t, _)| !t.is_finite()) {
        return invalid("radius-law station parameters must be finite".into());
    }
    if stations[0].0 != 0.0 || stations[stations.len() - 1].0 != 1.0 {
        return invalid("a station radius law must start at t = 0 and end at t = 1".into());
    }
    if let Some(pair) = stations
        .windows(2)
        .find(|pair| pair[1].0 - pair[0].0 < MIN_STATION_SPACING)
    {
        return invalid(format!(
            "radius-law stations must increase by at least {MIN_STATION_SPACING}: {} then {}",
            pair[0].0, pair[1].0
        ));
    }
    Ok(())
}

impl From<StandardRadiusLaw> for RadiusLaw {
    fn from(law: StandardRadiusLaw) -> Self {
        match law {
            StandardRadiusLaw::Constant(radius) => Self::Constant(radius),
            StandardRadiusLaw::Linear { start, end } => Self::Linear { start, end },
            StandardRadiusLaw::SCurve { start, end } => Self::SCurve { start, end },
            // An opaque callback for the walking engines, which qualify a
            // custom law only station by station.
            law @ StandardRadiusLaw::Stations { .. } => {
                Self::Custom(Box::new(move |t| law.evaluate(t)))
            }
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
    fn standard_law_derivatives_name_stationary_endpoints() {
        let scurve = StandardRadiusLaw::SCurve {
            start: 1.0,
            end: 3.0,
        };
        assert!(scurve.derivative(0.0).abs() < f64::EPSILON);
        assert!((scurve.derivative(0.5) - 3.0).abs() < f64::EPSILON);
        assert!(scurve.derivative(1.0).abs() < f64::EPSILON);
        assert!(StandardRadiusLaw::Constant(2.0).derivative(0.5).abs() < f64::EPSILON);
        assert!(
            (StandardRadiusLaw::Linear {
                start: 1.0,
                end: 3.0,
            }
            .derivative(0.5)
                - 2.0)
                .abs()
                < f64::EPSILON
        );
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

    #[test]
    fn station_laws_pass_their_stations_and_stay_inside_them() {
        for blend in [StationBlend::Linear, StationBlend::SCurve] {
            let law = StandardRadiusLaw::Stations {
                blend,
                stations: vec![(0.0, 1.0), (0.25, 3.0), (0.6, 1.5), (1.0, 2.0)],
            };
            for (t, r) in [(0.0, 1.0), (0.25, 3.0), (0.6, 1.5), (1.0, 2.0)] {
                assert!((law.evaluate(t) - r).abs() < 1e-15, "{blend:?} at {t}");
            }
            assert_eq!(law.bounds(), (1.0, 3.0));
            assert_eq!(law.validated_bounds(1e-7).unwrap(), (1.0, 3.0));
            assert_eq!(law.breakpoints(), vec![0.25, 0.6]);
            let segments = law.segments();
            assert_eq!(segments.len(), 3);
            for i in 0..=1000 {
                let t = f64::from(i) / 1000.0;
                let r = law.evaluate(t);
                assert!((1.0..=3.0).contains(&r), "{blend:?} {t} {r}");
            }
            // Each piece's Hermite data reproduces the law inside it.
            for piece in &segments {
                let h = 1e-7;
                let inside0 = law.derivative(piece.t0 + h);
                let inside1 = law.derivative(piece.t1 - h);
                assert!((inside0 - piece.slope0).abs() < 1e-4, "{piece:?}");
                assert!((inside1 - piece.slope1).abs() < 1e-4, "{piece:?}");
            }
        }
        let smooth = StandardRadiusLaw::Stations {
            blend: StationBlend::SCurve,
            stations: vec![(0.0, 1.0), (0.5, 3.0), (1.0, 2.0)],
        };
        assert!(smooth.derivative(0.0).abs() < f64::EPSILON);
        assert!(smooth.derivative(0.5).abs() < f64::EPSILON);
        assert!(smooth.derivative(1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn standard_laws_are_one_exact_piece() {
        let linear = StandardRadiusLaw::Linear {
            start: 1.0,
            end: 3.0,
        };
        assert_eq!(
            linear.segments(),
            vec![RadiusLawSegment {
                t0: 0.0,
                t1: 1.0,
                r0: 1.0,
                r1: 3.0,
                slope0: 2.0,
                slope1: 2.0,
            }]
        );
        assert!(linear.breakpoints().is_empty());
        let scurve = StandardRadiusLaw::SCurve {
            start: 1.0,
            end: 3.0,
        };
        assert_eq!(scurve.segments()[0].slope0, 0.0);
        assert_eq!(scurve.segments()[0].slope1, 0.0);
        assert_eq!(StandardRadiusLaw::Constant(2.0).segments()[0].slope0, 0.0);
    }

    #[test]
    fn malformed_station_lists_are_refused() {
        for stations in [
            vec![],
            vec![(0.0, 1.0)],
            vec![(0.0, 1.0), (0.9, 1.0)],
            vec![(0.1, 1.0), (1.0, 1.0)],
            vec![(0.0, 1.0), (0.5, 1.0), (0.5, 1.0), (1.0, 1.0)],
            vec![(0.0, 1.0), (0.5, 1.0), (0.5 + 1e-9, 1.0), (1.0, 1.0)],
            vec![(0.0, 1.0), (f64::NAN, 1.0), (1.0, 1.0)],
            vec![(0.0, 1.0), (0.5, f64::INFINITY), (1.0, 1.0)],
            vec![(0.0, 1.0), (0.5, 0.0), (1.0, 1.0)],
        ] {
            let law = StandardRadiusLaw::Stations {
                blend: StationBlend::Linear,
                stations: stations.clone(),
            };
            assert!(
                matches!(
                    law.validated_bounds(1e-7),
                    Err(BlendError::InvalidInput { .. })
                ),
                "{stations:?}"
            );
        }
    }
}
