//! Edge — a curve bounded by two vertices.

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::tolerance::Tolerance;
use remus_math::traits::ParametricCurve;
use remus_math::vec::{Point3, Vec3};

use crate::arena;
use crate::vertex::VertexId;

/// Typed handle for an [`Edge`] stored in an [`Arena`](crate::Arena).
pub type EdgeId = arena::Id<Edge>;

/// Failure to obtain an authoritative parameter range for an edge.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum EdgeDomainError {
    /// A non-Line edge has no stored trim interval.
    #[error("{curve_type} edge has no authoritative parameter range")]
    Missing {
        /// Stable edge-curve type tag.
        curve_type: &'static str,
    },
    /// A stored interval violates the edge-domain invariant.
    #[error("{curve_type} edge has invalid parameter range [{start}, {end}]")]
    Invalid {
        /// Stable edge-curve type tag.
        curve_type: &'static str,
        /// Stored range start.
        start: f64,
        /// Stored range end.
        end: f64,
    },
}

impl remus_math::diagnostic::ToDiagnostic for EdgeDomainError {
    fn diagnostic(&self) -> remus_math::diagnostic::Diagnostic {
        use remus_math::diagnostic::{Diagnostic, FailureCategory};

        let message = self.to_string();
        match self {
            Self::Missing { curve_type } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "edge_domain_missing",
                message,
            )
            .with_detail("curveType", *curve_type),
            Self::Invalid {
                curve_type,
                start,
                end,
            } => {
                let diagnostic = Diagnostic::new(
                    FailureCategory::InvalidTopology,
                    "edge_domain_invalid",
                    message,
                )
                .with_detail("curveType", *curve_type);
                if start.is_finite() && end.is_finite() {
                    diagnostic
                        .with_detail("start", *start)
                        .with_detail("end", *end)
                } else {
                    diagnostic
                }
            }
        }
    }
}

/// The geometric curve associated with an edge.
#[derive(Debug, Clone)]
pub enum EdgeCurve {
    /// A straight line segment (geometry is fully determined by the vertices).
    Line,
    /// A NURBS curve defining the edge geometry.
    NurbsCurve(NurbsCurve),
    /// A circular arc (or full circle when the edge is closed).
    Circle(Circle3D),
    /// An elliptical arc (or full ellipse when the edge is closed).
    Ellipse(Ellipse3D),
    /// A hyperbolic arc.
    ///
    /// A hyperbola branch is unbounded and never closes, so the edge's
    /// vertices always trim it: the parameter range comes from projecting
    /// both vertices onto the curve (see
    /// [`EdgeCurve::domain_with_endpoints`]).
    Hyperbola(Hyperbola3D),
    /// A parabolic arc.
    ///
    /// A parabola is unbounded and never closes, so the edge's vertices
    /// always trim it (see [`EdgeCurve::domain_with_endpoints`]).
    Parabola(Parabola3D),
}

impl EdgeCurve {
    /// Evaluate the curve at parameter `t`.
    ///
    /// `Line` has no stored geometry, so it linearly interpolates between
    /// `start` and `end` with `t` in `[0, 1]`. Circle, Ellipse, and NURBS
    /// dispatch to their [`ParametricCurve`] implementations.
    #[must_use]
    pub fn evaluate_with_endpoints(&self, t: f64, start: Point3, end: Point3) -> Point3 {
        match self {
            Self::Line => start + (end - start) * t,
            Self::Circle(c) => ParametricCurve::evaluate(c, t),
            Self::Ellipse(e) => ParametricCurve::evaluate(e, t),
            Self::Hyperbola(h) => h.evaluate(t),
            Self::Parabola(p) => p.evaluate(t),
            Self::NurbsCurve(n) => ParametricCurve::evaluate(n, t),
        }
    }

    /// Tangent vector at parameter `t`.
    ///
    /// For `Line`, returns the normalized `start → end` direction. For curves
    /// with stored geometry, dispatches to [`ParametricCurve::tangent`].
    #[must_use]
    pub fn tangent_with_endpoints(&self, t: f64, start: Point3, end: Point3) -> Vec3 {
        match self {
            Self::Line => {
                let dir = end - start;
                dir.normalize().unwrap_or(Vec3::new(1.0, 0.0, 0.0))
            }
            Self::Circle(c) => ParametricCurve::tangent(c, t),
            Self::Ellipse(e) => ParametricCurve::tangent(e, t),
            Self::Hyperbola(h) => h.tangent(t),
            Self::Parabola(p) => p.tangent(t),
            Self::NurbsCurve(n) => ParametricCurve::tangent(n, t),
        }
    }

    /// Reconstructs a parameter domain from endpoint geometry.
    ///
    /// `Line` uses `[0, 1]`. Closed Circle and Ellipse edges (`start ≈ end`)
    /// use the full `[0, 2π]` domain. Open arcs project both endpoints onto
    /// the curve and return the CCW angular range `[a₀, a₁]` with `a₁ > a₀`,
    /// so sampling the domain traces exactly the trimmed arc rather than the
    /// full curve. NURBS edges whose endpoints sit at the curve's natural
    /// ends (either orientation), or whose endpoint projections fail to
    /// validate as a forward interior sub-span, use the full knot span; a
    /// validated open sub-span returns the projected `[t₀, t₁]` so the edge
    /// samples only its own piece of a shared curve.
    #[must_use]
    pub fn reconstruct_domain_from_endpoints(&self, start: Point3, end: Point3) -> (f64, f64) {
        const TAU: f64 = std::f64::consts::TAU;
        // Below this chord the endpoints are considered coincident and the
        // edge is treated as a closed (full) curve.
        const CLOSED_EPS: f64 = 1e-9;
        // NURBS whole-edge match band: split vertices on marched/fit curves sit
        // up to the fit error (~1e-6) off the exact curve, well above vertex
        // tolerance.
        const END_EPS: f64 = 1e-6;
        // NURBS sub-span on-curve band (the weld scale).
        const WELD_EPS: f64 = 1e-5;
        match self {
            Self::Line => (0.0, 1.0),
            Self::Circle(c) => {
                if (start - end).length() < CLOSED_EPS {
                    ParametricCurve::domain(c)
                } else {
                    let a0 = c.project(start);
                    let delta = (c.project(end) - a0).rem_euclid(TAU);
                    let delta = if delta < 1e-12 { TAU } else { delta };
                    (a0, a0 + delta)
                }
            }
            Self::Ellipse(e) => {
                if (start - end).length() < CLOSED_EPS {
                    ParametricCurve::domain(e)
                } else {
                    let a0 = e.project(start);
                    let delta = (e.project(end) - a0).rem_euclid(TAU);
                    let delta = if delta < 1e-12 { TAU } else { delta };
                    (a0, a0 + delta)
                }
            }
            // Hyperbola and parabola branches are unbounded and never
            // closed, so there is no "full curve" fallback to take: the
            // vertices are the only thing that bounds the edge. Both
            // parameterizations have an exact closed-form inverse
            // (`t = asinh(v/b)` and `t = (P − vertex)·u` respectively), so
            // the projection is tolerance-free and needs no on-curve test.
            // A reversed span (`t₀ > t₁`) is returned as-is — it traces
            // start → end, matching the NURBS open-curve convention below.
            Self::Hyperbola(h) => (h.project(start), h.project(end)),
            Self::Parabola(p) => (p.project(start), p.project(end)),
            Self::NurbsCurve(n) => {
                let (d0, d1) = ParametricCurve::domain(n);
                if (start - end).length() < CLOSED_EPS {
                    return (d0, d1);
                }
                let p0 = ParametricCurve::evaluate(n, d0);
                let p1 = ParametricCurve::evaluate(n, d1);
                if ((p0 - start).length() < END_EPS && (p1 - end).length() < END_EPS)
                    || ((p0 - end).length() < END_EPS && (p1 - start).length() < END_EPS)
                {
                    return (d0, d1);
                }
                let proj = |p| remus_math::nurbs::projection::project_point_to_curve(n, p, 1e-9);
                if let (Ok(pa), Ok(pb)) = (proj(start), proj(end)) {
                    // Accept a non-degenerate on-curve span. A reversed pair
                    // (`t₀ > t₁`, interpolation still traces start→end) is
                    // trusted only on a clearly OPEN curve: on a (nearly)
                    // closed curve a reversed projection pair is usually a
                    // seam-crossing forward sub-arc, and interpolating
                    // backward would trace the complement arc. Degenerate
                    // spans and off-curve endpoints keep the historical
                    // full-domain behaviour.
                    let dt = pb.parameter - pa.parameter;
                    let curve_open = (p0 - p1).length() >= WELD_EPS;
                    if pa.distance < WELD_EPS
                        && pb.distance < WELD_EPS
                        && dt.abs() > 1e-6 * (d1 - d0)
                        && (dt > 0.0 || curve_open)
                    {
                        return (pa.parameter, pb.parameter);
                    }
                }
                (d0, d1)
            }
        }
    }

    /// Parameter domain reconstructed from endpoint geometry.
    ///
    /// This compatibility accessor is retained for raw construction and
    /// controlled import/healing adapters. Stored topology readers use
    /// [`Edge::strict_domain`] instead so a missing trim cannot silently
    /// select a different arc.
    #[must_use]
    pub fn domain_with_endpoints(&self, start: Point3, end: Point3) -> (f64, f64) {
        self.reconstruct_domain_from_endpoints(start, end)
    }

    /// Type tag string for debugging and serialization.
    #[must_use]
    pub const fn type_tag(&self) -> &'static str {
        match self {
            Self::Line => "line",
            Self::Circle(_) => "circle",
            Self::Ellipse(_) => "ellipse",
            Self::Hyperbola(_) => "hyperbola",
            Self::Parabola(_) => "parabola",
            Self::NurbsCurve(_) => "nurbs_curve",
        }
    }
}

/// A topological edge: a curve bounded by a start and end vertex.
///
/// An edge where `start == end` is a closed (degenerate) edge such as
/// a full circle.
#[derive(Debug, Clone)]
pub struct Edge {
    /// The vertex at the start of the edge.
    start: VertexId,
    /// The vertex at the end of the edge.
    end: VertexId,
    /// The geometric curve of the edge.
    curve: EdgeCurve,
    /// Optional edge-specific tolerance. When `None`, the edge inherits the
    /// tolerance from its bounding vertices.
    tolerance: Option<f64>,
    /// Explicit trim interval `(t0, t1)` on the curve's parameterization
    /// (RFC 0002, Stage 3). When present, this — not endpoint projection —
    /// is the edge's parameter domain. A reversed span (`t0 > t1`) traces
    /// start → end, matching the open-curve projection convention.
    trim: Option<(f64, f64)>,
}

impl Edge {
    /// Creates a new edge between two vertices with the given curve.
    ///
    /// The edge tolerance defaults to `None`, meaning the edge inherits
    /// tolerance from its bounding vertices.
    #[must_use]
    pub const fn new(start: VertexId, end: VertexId, curve: EdgeCurve) -> Self {
        Self {
            start,
            end,
            curve,
            tolerance: None,
            trim: None,
        }
    }

    /// Creates a new edge with an explicit tolerance.
    ///
    /// Pass `None` to inherit the vertex tolerance, or `Some(tol)` to set
    /// an edge-specific tolerance.
    #[must_use]
    pub const fn with_tolerance(
        start: VertexId,
        end: VertexId,
        curve: EdgeCurve,
        tol: Option<f64>,
    ) -> Self {
        Self {
            start,
            end,
            curve,
            tolerance: tol,
            trim: None,
        }
    }

    /// Returns the start vertex of this edge.
    #[must_use]
    pub const fn start(&self) -> VertexId {
        self.start
    }

    /// Returns the end vertex of this edge.
    #[must_use]
    pub const fn end(&self) -> VertexId {
        self.end
    }

    /// Returns a reference to the curve geometry of this edge.
    #[must_use]
    pub const fn curve(&self) -> &EdgeCurve {
        &self.curve
    }

    /// Returns `true` if the edge is closed (start equals end).
    #[must_use]
    pub fn is_closed(&self) -> bool {
        self.start == self.end
    }

    /// Sets the start vertex of this edge.
    pub fn set_start(&mut self, start: VertexId) {
        self.start = start;
    }

    /// Sets the end vertex of this edge.
    pub fn set_end(&mut self, end: VertexId) {
        self.end = end;
    }

    /// Sets the curve geometry of this edge.
    ///
    /// Clears any stored trim interval: a trim is meaningful only on the
    /// parameterization it was recorded against.
    pub fn set_curve(&mut self, curve: EdgeCurve) {
        self.curve = curve;
        self.trim = None;
    }

    /// The explicit trim interval on the curve's parameterization, when one
    /// is stored.
    #[must_use]
    pub const fn trim(&self) -> Option<(f64, f64)> {
        self.trim
    }

    /// Stores (or clears) the explicit trim interval.
    ///
    /// Callers that know the exact sub-span of a shared or split curve —
    /// e.g. the boolean pave filler, which has the pave parameters in hand —
    /// record it here so the domain never has to be reconstructed by
    /// endpoint projection. Non-finite bounds are ignored (`None` stored).
    pub fn set_trim(&mut self, trim: Option<(f64, f64)>) {
        self.trim = trim.filter(|(t0, t1)| t0.is_finite() && t1.is_finite());
    }

    /// The edge's parameter domain: the stored trim when present, otherwise
    /// reconstructed from the endpoints via
    /// [`EdgeCurve::reconstruct_domain_from_endpoints`].
    ///
    /// This is the preferred domain accessor (RFC 0002, Stage 3): explicit
    /// trims are exact where projection-based reconstruction depends on
    /// tolerance bands and can mistake a sub-span edge for a whole-curve
    /// edge when split vertices sit off the fitted curve.
    #[must_use]
    pub fn domain_with_endpoints(&self, start: Point3, end: Point3) -> (f64, f64) {
        self.trim
            .unwrap_or_else(|| self.curve.reconstruct_domain_from_endpoints(start, end))
    }

    /// Returns the edge's authoritative parameter range.
    ///
    /// Lines are intrinsically endpoint-local on `[0, 1]`. Every other curve
    /// requires a finite stored trim; `None` means the topology has not yet
    /// established parameter authority and callers must refuse or enter an
    /// explicitly named reconstruction adapter.
    ///
    /// # Errors
    ///
    /// Returns [`EdgeDomainError::Missing`] when a non-Line edge lacks a trim,
    /// or [`EdgeDomainError::Invalid`] when stored authority violates the
    /// edge-domain invariant.
    pub fn strict_domain(&self) -> Result<(f64, f64), EdgeDomainError> {
        if matches!(self.curve, EdgeCurve::Line) {
            return if let Some((start, end)) = self.trim {
                if start.partial_cmp(&0.0) == Some(std::cmp::Ordering::Equal)
                    && end.partial_cmp(&1.0) == Some(std::cmp::Ordering::Equal)
                {
                    Ok((0.0, 1.0))
                } else {
                    Err(EdgeDomainError::Invalid {
                        curve_type: self.curve.type_tag(),
                        start,
                        end,
                    })
                }
            } else {
                Ok((0.0, 1.0))
            };
        }

        let (start, end) = self.trim.ok_or_else(|| EdgeDomainError::Missing {
            curve_type: self.curve.type_tag(),
        })?;
        let invalid_common = !start.is_finite()
            || !end.is_finite()
            || start.partial_cmp(&end) == Some(std::cmp::Ordering::Equal);
        let invalid_for_curve = match &self.curve {
            EdgeCurve::Line => true,
            EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => !periodic_curve_domain_is_valid(
                &self.curve,
                start,
                end,
                self.is_closed(),
                self.tolerance.unwrap_or(Tolerance::new().linear),
            ),
            EdgeCurve::NurbsCurve(curve) => {
                let (domain_start, domain_end) = curve.domain();
                start < domain_start || start > domain_end || end < domain_start || end > domain_end
            }
            EdgeCurve::Hyperbola(curve) => {
                [curve.evaluate(start), curve.evaluate(end)]
                    .iter()
                    .any(|point| point.0.iter().any(|value| !value.is_finite()))
                    || [curve.tangent(start), curve.tangent(end)]
                        .iter()
                        .any(|tangent| tangent.0.iter().any(|value| !value.is_finite()))
            }
            EdgeCurve::Parabola(curve) => {
                [curve.evaluate(start), curve.evaluate(end)]
                    .iter()
                    .any(|point| point.0.iter().any(|value| !value.is_finite()))
                    || [curve.tangent(start), curve.tangent(end)]
                        .iter()
                        .any(|tangent| tangent.0.iter().any(|value| !value.is_finite()))
            }
        };
        if invalid_common || invalid_for_curve {
            return Err(EdgeDomainError::Invalid {
                curve_type: self.curve.type_tag(),
                start,
                end,
            });
        }
        Ok((start, end))
    }

    /// Returns the edge-specific tolerance, or `None` if the edge inherits
    /// tolerance from its bounding vertices.
    #[must_use]
    pub const fn tolerance(&self) -> Option<f64> {
        self.tolerance
    }

    /// Sets the edge-specific tolerance.
    ///
    /// Pass `None` to revert to inheriting the vertex tolerance.
    pub fn set_tolerance(&mut self, tol: Option<f64>) {
        self.tolerance = tol;
    }

    /// Returns the effective tolerance for this edge.
    ///
    /// If the edge has its own tolerance, that value is returned. Otherwise
    /// the provided `vertex_tol` (typically the maximum of the two bounding
    /// vertex tolerances) is used as a fallback.
    #[must_use]
    pub fn effective_tolerance(&self, vertex_tol: f64) -> f64 {
        self.tolerance.unwrap_or(vertex_tol)
    }
}

fn periodic_domain_is_valid(
    start: f64,
    end: f64,
    closed: bool,
    tolerance: f64,
    evaluate: impl Fn(f64) -> Point3,
) -> bool {
    let span = (end - start).abs();
    let roundoff_allowance = 4.0 * f64::EPSILON * std::f64::consts::TAU;
    if !closed {
        return span <= std::f64::consts::TAU;
    }
    // Anchored full turns can subtract to one ULP above TAU. Accept that
    // roundoff only when the numeric curve still closes within tolerance.
    if !tolerance.is_finite()
        || tolerance.is_sign_negative()
        || (span - std::f64::consts::TAU).abs() > roundoff_allowance
    {
        return false;
    }

    let closure = (evaluate(start) - evaluate(end)).length();
    closure.is_finite() && closure <= tolerance
}

pub(crate) fn periodic_curve_domain_is_valid(
    curve: &EdgeCurve,
    start: f64,
    end: f64,
    closed: bool,
    tolerance: f64,
) -> bool {
    match curve {
        EdgeCurve::Circle(circle) => {
            periodic_curve_is_finite(
                circle.center(),
                circle.u_axis(),
                circle.v_axis(),
                circle.radius(),
                circle.radius(),
            ) && periodic_domain_is_valid(start, end, closed, tolerance, |parameter| {
                circle.evaluate(parameter)
            })
        }
        EdgeCurve::Ellipse(ellipse) => {
            periodic_curve_is_finite(
                ellipse.center(),
                ellipse.u_axis(),
                ellipse.v_axis(),
                ellipse.semi_major(),
                ellipse.semi_minor(),
            ) && periodic_domain_is_valid(start, end, closed, tolerance, |parameter| {
                ellipse.evaluate(parameter)
            })
        }
        EdgeCurve::Line
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::NurbsCurve(_) => false,
    }
}

fn periodic_curve_is_finite(
    center: Point3,
    u_axis: Vec3,
    v_axis: Vec3,
    u_extent: f64,
    v_extent: f64,
) -> bool {
    (0..3).all(|component| {
        let u_bound = u_axis.0[component].abs() * u_extent;
        let v_bound = v_axis.0[component].abs() * v_extent;
        let tangent_bound = u_bound + v_bound;
        let position_bound = center.0[component].abs() + tangent_bound;
        tangent_bound.is_finite() && position_bound.is_finite()
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use crate::arena::Arena;
    use crate::vertex::Vertex;
    use remus_math::vec::Point3;

    fn make_test_vertices() -> (VertexId, VertexId) {
        let mut arena: Arena<Vertex> = Arena::new();
        let v0 = arena.alloc(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = arena.alloc(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        (v0, v1)
    }

    #[test]
    fn new_defaults_tolerance_to_none() {
        let (v0, v1) = make_test_vertices();
        let edge = Edge::new(v0, v1, EdgeCurve::Line);
        assert!(edge.tolerance().is_none());
    }

    #[test]
    fn with_tolerance_stores_value() {
        let (v0, v1) = make_test_vertices();
        let edge = Edge::with_tolerance(v0, v1, EdgeCurve::Line, Some(1e-5));
        assert_eq!(edge.tolerance(), Some(1e-5));
    }

    #[test]
    fn with_tolerance_none() {
        let (v0, v1) = make_test_vertices();
        let edge = Edge::with_tolerance(v0, v1, EdgeCurve::Line, None);
        assert!(edge.tolerance().is_none());
    }

    #[test]
    fn set_tolerance_round_trip() {
        let (v0, v1) = make_test_vertices();
        let mut edge = Edge::new(v0, v1, EdgeCurve::Line);

        edge.set_tolerance(Some(0.001));
        assert_eq!(edge.tolerance(), Some(0.001));

        edge.set_tolerance(None);
        assert!(edge.tolerance().is_none());
    }

    #[test]
    fn effective_tolerance_uses_own_when_set() {
        let (v0, v1) = make_test_vertices();
        let edge = Edge::with_tolerance(v0, v1, EdgeCurve::Line, Some(1e-5));
        assert!((edge.effective_tolerance(1e-7) - 1e-5).abs() < f64::EPSILON);
    }

    #[test]
    fn effective_tolerance_falls_back_to_vertex_tol() {
        let (v0, v1) = make_test_vertices();
        let edge = Edge::new(v0, v1, EdgeCurve::Line);
        assert!((edge.effective_tolerance(1e-7) - 1e-7).abs() < f64::EPSILON);
    }

    fn open_nurbs() -> EdgeCurve {
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 1.5, 0.0),
            Point3::new(3.0, 1.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        ];
        EdgeCurve::NurbsCurve(remus_math::nurbs::fitting::interpolate(&pts, 3).unwrap())
    }

    fn assert_full_domain(got: (f64, f64), d0: f64, d1: f64) {
        assert!(
            (got.0 - d0).abs() < f64::EPSILON,
            "t0={} expected {d0}",
            got.0
        );
        assert!(
            (got.1 - d1).abs() < f64::EPSILON,
            "t1={} expected {d1}",
            got.1
        );
    }

    #[test]
    fn nurbs_domain_whole_edge_keeps_full_span_both_orientations() {
        let curve = open_nurbs();
        let EdgeCurve::NurbsCurve(n) = &curve else {
            unreachable!()
        };
        let (d0, d1) = remus_math::traits::ParametricCurve::domain(n);
        let p0 = remus_math::traits::ParametricCurve::evaluate(n, d0);
        let p1 = remus_math::traits::ParametricCurve::evaluate(n, d1);
        assert_full_domain(curve.domain_with_endpoints(p0, p1), d0, d1);
        assert_full_domain(curve.domain_with_endpoints(p1, p0), d0, d1);
    }

    #[test]
    fn nurbs_domain_forward_sub_span_is_trimmed() {
        let curve = open_nurbs();
        let EdgeCurve::NurbsCurve(n) = &curve else {
            unreachable!()
        };
        let (d0, d1) = remus_math::traits::ParametricCurve::domain(n);
        let ta = d0 + 0.25 * (d1 - d0);
        let tb = d0 + 0.7 * (d1 - d0);
        let pa = remus_math::traits::ParametricCurve::evaluate(n, ta);
        let pb = remus_math::traits::ParametricCurve::evaluate(n, tb);
        let (t0, t1) = curve.domain_with_endpoints(pa, pb);
        assert!((t0 - ta).abs() < 1e-6, "t0={t0} expected {ta}");
        assert!((t1 - tb).abs() < 1e-6, "t1={t1} expected {tb}");
        // The trimmed domain evaluates back to the endpoints, so a consumer
        // sampling it traces only the edge's own piece of the shared curve.
        let s = curve.evaluate_with_endpoints(t0, pa, pb);
        let e = curve.evaluate_with_endpoints(t1, pa, pb);
        assert!((s - pa).length() < 1e-6);
        assert!((e - pb).length() < 1e-6);
    }

    #[test]
    fn nurbs_domain_reversed_sub_span_on_open_curve_trims_backward() {
        let curve = open_nurbs();
        let EdgeCurve::NurbsCurve(n) = &curve else {
            unreachable!()
        };
        let (d0, d1) = remus_math::traits::ParametricCurve::domain(n);
        let ta = d0 + 0.25 * (d1 - d0);
        let tb = d0 + 0.7 * (d1 - d0);
        let pa = remus_math::traits::ParametricCurve::evaluate(n, ta);
        let pb = remus_math::traits::ParametricCurve::evaluate(n, tb);
        // Edge runs pb -> pa (reversed relative to curve parameterization):
        // the trimmed domain keeps t0 > t1 so t0->t1 interpolation still
        // traces start -> end.
        let (t0, t1) = curve.domain_with_endpoints(pb, pa);
        assert!(t0 > t1, "expected reversed span, got ({t0}, {t1})");
        assert!((curve.evaluate_with_endpoints(t0, pb, pa) - pb).length() < 1e-6);
        assert!((curve.evaluate_with_endpoints(t1, pb, pa) - pa).length() < 1e-6);
    }

    #[test]
    fn nurbs_domain_reversed_pair_on_closed_curve_falls_back_to_full_domain() {
        // A closed fitted loop: a reversed projection pair here is ambiguous
        // (usually a seam-crossing forward sub-arc), so the full domain wins.
        let pts: Vec<Point3> = (0..=12)
            .map(|k| {
                let a = std::f64::consts::TAU * f64::from(k) / 12.0;
                Point3::new(a.cos(), a.sin(), 0.0)
            })
            .collect();
        let n = remus_math::nurbs::fitting::interpolate(&pts, 3).unwrap();
        let (d0, d1) = remus_math::traits::ParametricCurve::domain(&n);
        let ta = d0 + 0.6 * (d1 - d0);
        let tb = d0 + 0.2 * (d1 - d0);
        let pa = remus_math::traits::ParametricCurve::evaluate(&n, ta);
        let pb = remus_math::traits::ParametricCurve::evaluate(&n, tb);
        let curve = EdgeCurve::NurbsCurve(n);
        assert_full_domain(curve.domain_with_endpoints(pa, pb), d0, d1);
    }

    #[test]
    fn nurbs_domain_off_curve_endpoints_fall_back_to_full_domain() {
        let curve = open_nurbs();
        let EdgeCurve::NurbsCurve(n) = &curve else {
            unreachable!()
        };
        let (d0, d1) = remus_math::traits::ParametricCurve::domain(n);
        let ta = d0 + 0.25 * (d1 - d0);
        let tb = d0 + 0.7 * (d1 - d0);
        let off = Vec3::new(0.0, 0.0, 1.0) * 0.5;
        let pa = remus_math::traits::ParametricCurve::evaluate(n, ta) + off;
        let pb = remus_math::traits::ParametricCurve::evaluate(n, tb) + off;
        assert_full_domain(curve.domain_with_endpoints(pa, pb), d0, d1);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod conic_domain_tests {
    use super::*;
    use remus_math::curves::{Hyperbola3D, Parabola3D};

    #[test]
    fn hyperbola_domain_comes_from_the_vertices_and_is_exact() {
        // An unbounded branch has no "full curve" fallback: the two vertices
        // are the only trim, and `project` inverts the parameterization
        // exactly, so the recovered span must reproduce the source
        // parameters to round-off.
        let h = Hyperbola3D::with_axes(
            Point3::new(-1.0, 2.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0,
            3.0,
        )
        .unwrap();
        let curve = EdgeCurve::Hyperbola(h.clone());
        let (t0, t1) = (-1.25, 2.0);
        let (d0, d1) = curve.domain_with_endpoints(h.evaluate(t0), h.evaluate(t1));
        assert!((d0 - t0).abs() < 1e-13, "{d0} vs {t0}");
        assert!((d1 - t1).abs() < 1e-13, "{d1} vs {t1}");

        // Reversed endpoints yield the reversed span, tracing start -> end
        // (the same convention the NURBS open-curve path uses).
        let (r0, r1) = curve.domain_with_endpoints(h.evaluate(t1), h.evaluate(t0));
        assert!((r0 - t1).abs() < 1e-13 && (r1 - t0).abs() < 1e-13);
    }

    #[test]
    fn parabola_domain_comes_from_the_vertices_and_is_exact() {
        let p = Parabola3D::with_axes(
            Point3::new(3.0, -1.0, 7.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.7,
        )
        .unwrap();
        let curve = EdgeCurve::Parabola(p.clone());
        let (t0, t1) = (-2.3, 4.1);
        let (d0, d1) = curve.domain_with_endpoints(p.evaluate(t0), p.evaluate(t1));
        assert!((d0 - t0).abs() < 1e-13, "{d0} vs {t0}");
        assert!((d1 - t1).abs() < 1e-13, "{d1} vs {t1}");
    }

    #[test]
    fn conic_evaluate_with_endpoints_traces_the_trimmed_arc() {
        // Sampling the recovered domain must land exactly on the curve, and
        // the ends must be the vertices — a chord or full-curve fallback
        // would fail both.
        let h = Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0,
            3.0,
        )
        .unwrap();
        let curve = EdgeCurve::Hyperbola(h.clone());
        let (start, end) = (h.evaluate(-0.8), h.evaluate(1.4));
        let (d0, d1) = curve.domain_with_endpoints(start, end);
        assert!((curve.evaluate_with_endpoints(d0, start, end) - start).length() < 1e-12);
        assert!((curve.evaluate_with_endpoints(d1, start, end) - end).length() < 1e-12);
        for i in 0..=20 {
            let t = d0 + (d1 - d0) * f64::from(i) / 20.0;
            let q = curve.evaluate_with_endpoints(t, start, end);
            // On-curve: re-projecting must return the same parameter.
            assert!(
                (h.project(q) - t).abs() < 1e-12,
                "sample at {t} left the curve"
            );
        }
    }

    #[test]
    fn conic_type_tags_are_distinct() {
        let h = Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            1.0,
        )
        .unwrap();
        let p = Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        )
        .unwrap();
        assert_eq!(EdgeCurve::Hyperbola(h).type_tag(), "hyperbola");
        assert_eq!(EdgeCurve::Parabola(p).type_tag(), "parabola");
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod trim_tests {
    use remus_math::traits::ParametricCurve;
    use remus_math::vec::{Point3, Vec3};

    use super::*;
    use crate::arena::Arena;
    use crate::vertex::Vertex;

    fn vertices(a: Point3, b: Point3) -> (VertexId, VertexId) {
        let mut arena: Arena<Vertex> = Arena::new();
        (
            arena.alloc(Vertex::new(a, 1e-7)),
            arena.alloc(Vertex::new(b, 1e-7)),
        )
    }

    fn fitted_open_nurbs() -> NurbsCurve {
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 1.5, 0.0),
            Point3::new(3.0, 1.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        ];
        remus_math::nurbs::fitting::interpolate(&pts, 3).unwrap()
    }

    #[test]
    fn stored_trim_wins_over_projection() {
        let n = fitted_open_nurbs();
        let (d0, d1) = ParametricCurve::domain(&n);
        let (ta, tb) = (d0 + 0.25 * (d1 - d0), d0 + 0.7 * (d1 - d0));
        let pa = ParametricCurve::evaluate(&n, ta);
        let pb = ParametricCurve::evaluate(&n, tb);
        let (v0, v1) = vertices(pa, pb);

        let mut edge = Edge::new(v0, v1, EdgeCurve::NurbsCurve(n));
        edge.set_trim(Some((ta, tb)));
        let (t0, t1) = edge.domain_with_endpoints(pa, pb);
        assert!((t0 - ta).abs() < f64::EPSILON && (t1 - tb).abs() < f64::EPSILON);
    }

    #[test]
    fn stored_trim_survives_where_projection_reconstruction_fails() {
        // The failure mode explicit trims exist to kill: a sub-span edge
        // whose split vertices sit further off the fitted curve than the
        // projection weld band (1e-5). Reconstruction cannot validate the
        // sub-span and silently falls back to the FULL domain — sampling
        // the whole shared curve instead of the edge's own piece. The
        // stored trim is exact regardless.
        let n = fitted_open_nurbs();
        let (d0, d1) = ParametricCurve::domain(&n);
        let (ta, tb) = (d0 + 0.25 * (d1 - d0), d0 + 0.7 * (d1 - d0));
        let off = Vec3::new(0.0, 0.0, 1.0) * 1e-3; // >> weld band
        let pa = ParametricCurve::evaluate(&n, ta) + off;
        let pb = ParametricCurve::evaluate(&n, tb) + off;
        let (v0, v1) = vertices(pa, pb);

        // Without a trim: legacy fallback to the full knot span.
        let bare = Edge::new(v0, v1, EdgeCurve::NurbsCurve(n.clone()));
        let (f0, f1) = bare.domain_with_endpoints(pa, pb);
        assert!(
            (f0 - d0).abs() < f64::EPSILON && (f1 - d1).abs() < f64::EPSILON,
            "characterizes the legacy fallback this test exists to beat"
        );

        // With the trim the exact sub-span survives.
        let mut trimmed = Edge::new(v0, v1, EdgeCurve::NurbsCurve(n));
        trimmed.set_trim(Some((ta, tb)));
        let (t0, t1) = trimmed.domain_with_endpoints(pa, pb);
        assert!((t0 - ta).abs() < f64::EPSILON && (t1 - tb).abs() < f64::EPSILON);
    }

    #[test]
    fn set_curve_clears_the_trim() {
        let (v0, v1) = vertices(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let mut edge = Edge::new(v0, v1, EdgeCurve::Line);
        edge.set_trim(Some((0.2, 0.8)));
        assert_eq!(edge.trim(), Some((0.2, 0.8)));
        edge.set_curve(EdgeCurve::Line);
        assert_eq!(edge.trim(), None, "a trim is meaningless on a new curve");
    }

    #[test]
    fn non_finite_trims_are_refused() {
        let (v0, v1) = vertices(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let mut edge = Edge::new(v0, v1, EdgeCurve::Line);
        edge.set_trim(Some((f64::NAN, 1.0)));
        assert_eq!(edge.trim(), None);
        edge.set_trim(Some((0.0, f64::INFINITY)));
        assert_eq!(edge.trim(), None);
    }

    #[test]
    fn strict_domain_requires_non_line_authority() {
        use remus_math::curves::Circle3D;
        use remus_math::diagnostic::{FailureCategory, ToDiagnostic};

        let (v0, v1) = vertices(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0));
        let line = Edge::new(v0, v1, EdgeCurve::Line);
        assert_eq!(line.strict_domain().unwrap(), (0.0, 1.0));

        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let mut curved = Edge::new(v0, v1, EdgeCurve::Circle(circle));
        let missing = curved.strict_domain().unwrap_err();
        assert_eq!(
            missing.diagnostic().category(),
            FailureCategory::InvalidTopology
        );
        assert_eq!(missing.diagnostic().code(), "edge_domain_missing");

        curved.set_trim(Some((2.0, -1.0)));
        assert_eq!(curved.strict_domain().unwrap(), (2.0, -1.0));

        curved.set_trim(Some((0.5, 0.5)));
        let invalid = curved.strict_domain().unwrap_err();
        assert_eq!(
            invalid.diagnostic().category(),
            FailureCategory::InvalidTopology
        );
        assert_eq!(invalid.diagnostic().code(), "edge_domain_invalid");

        curved.set_trim(Some((0.0, std::f64::consts::TAU + 1e-6)));
        assert!(matches!(
            curved.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));

        let nurbs = fitted_open_nurbs();
        let (d0, d1) = nurbs.domain();
        let mut nurbs_edge = Edge::new(v0, v1, EdgeCurve::NurbsCurve(nurbs));
        nurbs_edge.set_trim(Some((d1, d0)));
        assert_eq!(nurbs_edge.strict_domain().unwrap(), (d1, d0));
        nurbs_edge.set_trim(Some((d0 - 1.0, d1)));
        assert!(matches!(
            nurbs_edge.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));
    }

    #[test]
    fn strict_domain_accepts_only_canonical_line_trim_and_rejects_non_finite_storage() {
        use remus_math::curves::Circle3D;

        let (v0, v1) = vertices(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0));
        let mut line = Edge::new(v0, v1, EdgeCurve::Line);
        line.set_trim(Some((0.2, 0.8)));
        assert!(matches!(
            line.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));
        assert_eq!(
            line.domain_with_endpoints(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)),
            (0.2, 0.8),
            "compatibility access must continue honoring an explicit Line trim"
        );
        line.set_trim(Some((0.0, 1.0)));
        assert_eq!(line.strict_domain().unwrap(), (0.0, 1.0));

        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let mut curved = Edge::new(v0, v1, EdgeCurve::Circle(circle));
        curved.trim = Some((f64::NAN, 1.0));
        assert!(matches!(
            curved.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));
    }

    #[test]
    fn strict_domain_rejects_periodic_overrun_and_open_conic_overflow() {
        use remus_math::curves::{Circle3D, Hyperbola3D, Parabola3D};

        let (v0, v1) = vertices(Point3::new(1e15, 0.0, 0.0), Point3::new(1e15, 0.0, 0.0));
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1e15).unwrap();
        let periodic_end = std::f64::consts::TAU + 5e-13;
        assert!((circle.evaluate(periodic_end) - circle.evaluate(0.0)).length() > 100.0);
        let mut periodic = Edge::new(v0, v1, EdgeCurve::Circle(circle));
        periodic.set_trim(Some((0.0, periodic_end)));
        assert!(matches!(
            periodic.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));

        let normal_circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();
        for trim in [
            (2.8, 2.8 + std::f64::consts::TAU),
            (2.8, 2.8 - std::f64::consts::TAU),
        ] {
            let mut anchored = Edge::new(v0, v0, EdgeCurve::Circle(normal_circle.clone()));
            anchored.set_trim(Some(trim));
            assert_eq!(anchored.strict_domain().unwrap(), trim);
        }

        let tiny_circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1e-9).unwrap();
        let unresolved_anchor = (1e16, 1e16 + 2.0);
        assert!(
            (tiny_circle.evaluate(unresolved_anchor.0) - tiny_circle.evaluate(unresolved_anchor.1))
                .length()
                < Tolerance::new().linear
        );
        let mut unresolved = Edge::new(v0, v0, EdgeCurve::Circle(tiny_circle));
        unresolved.set_trim(Some(unresolved_anchor));
        assert!(matches!(
            unresolved.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));

        let overflowing_circle = Circle3D::new_with_ref(
            Point3::new(-1e308, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1e308,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        for trim in [(0.0, std::f64::consts::TAU), (std::f64::consts::TAU, 0.0)] {
            assert!(
                [
                    overflowing_circle.evaluate(trim.0),
                    overflowing_circle.evaluate(trim.1),
                ]
                .iter()
                .all(|point| point.0.iter().all(|value| value.is_finite()))
            );
            assert!(
                overflowing_circle
                    .evaluate(trim.0.midpoint(trim.1))
                    .0
                    .iter()
                    .any(|value| !value.is_finite())
            );
            let mut periodic = Edge::new(v0, v1, EdgeCurve::Circle(overflowing_circle.clone()));
            periodic.set_trim(Some(trim));
            assert!(matches!(
                periodic.strict_domain(),
                Err(EdgeDomainError::Invalid { .. })
            ));
        }

        let huge_circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1e15).unwrap();
        for trim in [
            (2.8, 2.8 + std::f64::consts::TAU),
            (2.8, 2.8 - std::f64::consts::TAU),
        ] {
            assert!((huge_circle.evaluate(trim.0) - huge_circle.evaluate(trim.1)).length() > 0.1);
            let mut anchored = Edge::new(v0, v0, EdgeCurve::Circle(huge_circle.clone()));
            anchored.set_trim(Some(trim));
            assert!(matches!(
                anchored.strict_domain(),
                Err(EdgeDomainError::Invalid { .. })
            ));
        }

        let hyperbola = Hyperbola3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            2.0,
        )
        .unwrap();
        assert!(
            hyperbola
                .evaluate(1000.0)
                .0
                .iter()
                .any(|value| !value.is_finite())
        );
        let mut hyperbolic = Edge::new(v0, v1, EdgeCurve::Hyperbola(hyperbola));
        hyperbolic.set_trim(Some((0.0, 1000.0)));
        assert!(matches!(
            hyperbolic.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));

        let tangent_overflow = Hyperbola3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            1e308,
        )
        .unwrap();
        assert!(
            tangent_overflow
                .evaluate(1.2)
                .0
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            tangent_overflow
                .tangent(1.2)
                .0
                .iter()
                .any(|value| !value.is_finite())
        );
        for trim in [(0.0, 1.2), (1.2, 0.0)] {
            let mut hyperbolic = Edge::new(v0, v1, EdgeCurve::Hyperbola(tangent_overflow.clone()));
            hyperbolic.set_trim(Some(trim));
            assert!(matches!(
                hyperbolic.strict_domain(),
                Err(EdgeDomainError::Invalid { .. })
            ));
        }

        let parabola =
            Parabola3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        assert!(
            parabola
                .evaluate(1e200)
                .0
                .iter()
                .any(|value| !value.is_finite())
        );
        let mut parabolic = Edge::new(v0, v1, EdgeCurve::Parabola(parabola));
        parabolic.set_trim(Some((0.0, 1e200)));
        assert!(matches!(
            parabolic.strict_domain(),
            Err(EdgeDomainError::Invalid { .. })
        ));
    }
}
