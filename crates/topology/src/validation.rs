//! Topology validation utilities.
//!
//! These functions check structural invariants of topological entities
//! such as wire closure and shell manifoldness.

use std::collections::HashMap;

use crate::Topology;
use crate::TopologyError;
use crate::shell::Shell;
use crate::wire::{Wire, WireId};

/// Linear tolerance for treating two distinct vertices as coincident during
/// wire-closure checks. Matches the default linear tolerance (`1e-7`) and the
/// quantization step used when wires are chained by endpoint position, so a
/// loop chained through position-equal but ID-distinct vertices still
/// validates as closed.
const CLOSURE_POS_TOL: f64 = 1e-7;

/// Validates that a wire forms a closed loop.
///
/// A closed wire requires that for each consecutive pair of oriented edges
/// the end vertex of the first connects to the start vertex of the second,
/// and that the last edge connects back to the first. Connection is by
/// `VertexId` equality, falling back to coincident position (within
/// `CLOSURE_POS_TOL`) so wires assembled by chaining edges on endpoint
/// position — which can leave distinct-but-coincident vertex IDs — are not
/// rejected as open.
///
/// # Errors
///
/// Returns [`TopologyError::WireNotClosed`] if the wire is not closed.
/// Returns [`TopologyError::EdgeNotFound`] if any edge id is invalid.
pub fn validate_wire_closed(wire: &Wire, topo: &Topology) -> Result<(), TopologyError> {
    if !wire.is_closed() {
        return Err(TopologyError::WireNotClosed);
    }

    let connects =
        |a: crate::vertex::VertexId, b: crate::vertex::VertexId| -> Result<bool, TopologyError> {
            if a == b {
                return Ok(true);
            }
            let pa = topo.vertex(a)?.point();
            let pb = topo.vertex(b)?.point();
            Ok((pa - pb).length() <= CLOSURE_POS_TOL)
        };

    let oriented = wire.edges();
    for window in oriented.windows(2) {
        let current = &window[0];
        let next = &window[1];

        let current_edge = topo.edge(current.edge())?;
        let next_edge = topo.edge(next.edge())?;

        if !connects(
            current.oriented_end(current_edge),
            next.oriented_start(next_edge),
        )? {
            return Err(TopologyError::WireNotClosed);
        }
    }

    if let (Some(last), Some(first)) = (oriented.last(), oriented.first()) {
        let last_edge = topo.edge(last.edge())?;
        let first_edge = topo.edge(first.edge())?;

        if !connects(
            last.oriented_end(last_edge),
            first.oriented_start(first_edge),
        )? {
            return Err(TopologyError::WireNotClosed);
        }
    }

    Ok(())
}

/// Collects all edge usage counts for a given wire.
fn count_wire_edges(
    wire_id: WireId,
    topo: &Topology,
    counts: &mut HashMap<usize, usize>,
) -> Result<(), TopologyError> {
    let wire = topo.wire(wire_id)?;
    for oe in wire.edges() {
        *counts.entry(oe.edge().index()).or_insert(0) += 1;
    }
    Ok(())
}

/// Validates that a shell is manifold.
///
/// A manifold shell requires that every edge is shared by at most two faces.
/// This function walks shell -> faces -> wires -> edges, counts each edge's
/// usage, and reports any edge shared by more than two faces.
///
/// Note: [`AdjacencyIndex::is_manifold()`](crate::adjacency::AdjacencyIndex::is_manifold)
/// performs the same check but also detects boundary edges (shared by only 1 face).
/// Use `AdjacencyIndex` when you need the full adjacency data; use this function
/// for a lightweight pass/fail manifold check.
///
/// # Errors
///
/// Returns [`TopologyError::NonManifold`] if any edge is shared by more
/// than two faces.
/// Returns entity-not-found errors if any referenced ID is invalid.
pub fn validate_shell_manifold(shell: &Shell, topo: &Topology) -> Result<(), TopologyError> {
    let mut edge_counts: HashMap<usize, usize> = HashMap::new();

    for &face_id in shell.faces() {
        let face = topo.face(face_id)?;

        count_wire_edges(face.outer_wire(), topo, &mut edge_counts)?;

        for &inner_wire_id in face.inner_wires() {
            count_wire_edges(inner_wire_id, topo, &mut edge_counts)?;
        }
    }

    for (&edge_index, &count) in &edge_counts {
        if count > 2 {
            return Err(TopologyError::NonManifold {
                reason: format!(
                    "edge index {edge_index} is shared by {count} faces (max 2 for manifold)"
                ),
            });
        }
    }

    Ok(())
}

/// Validates that a shell is a closed 2-manifold.
///
/// Stricter than [`validate_shell_manifold`]: every edge must be used by
/// exactly two oriented-edge occurrences across the shell's wires. Edges
/// used once (free/boundary edges of an open shell) are rejected, not just
/// edges shared by 3+ faces.
///
/// # Errors
///
/// Returns [`TopologyError::NonManifold`] if any edge usage count differs
/// from two. Returns entity-not-found errors if any referenced ID is
/// invalid.
pub fn validate_shell_closed(shell: &Shell, topo: &Topology) -> Result<(), TopologyError> {
    let mut edge_counts: HashMap<usize, usize> = HashMap::new();

    for &face_id in shell.faces() {
        let face = topo.face(face_id)?;
        count_wire_edges(face.outer_wire(), topo, &mut edge_counts)?;
        for &inner_wire_id in face.inner_wires() {
            count_wire_edges(inner_wire_id, topo, &mut edge_counts)?;
        }
    }

    for (&edge_index, &count) in &edge_counts {
        if count != 2 {
            let kind = if count == 1 {
                "free edge"
            } else {
                "over-shared"
            };
            return Err(TopologyError::NonManifold {
                reason: format!(
                    "edge index {edge_index} is used by {count} wires ({kind}; closed manifold \
                     requires exactly 2)"
                ),
            });
        }
    }

    Ok(())
}

/// Validates a face's derived loops against its authoritative wires
/// (RFC 0002, Stage 1 consistency invariant).
///
/// A face with no derivation (no [`Topology::build_face_loops`] call)
/// passes vacuously. A face with a derivation must agree with its wires
/// exactly: loop count and order (outer first, then inner), per-loop
/// closure flag, and per-position edge identity and orientation. Loops are
/// only ever written by the kernel, so any divergence is a kernel bug.
///
/// # Errors
///
/// Returns [`TopologyError::LoopWireMismatch`] on divergence, or a
/// not-found error when a stored id is stale.
pub fn validate_face_loops(
    topo: &Topology,
    face_id: crate::face::FaceId,
) -> Result<(), TopologyError> {
    let Some(loop_ids) = topo.loops_of_face(face_id) else {
        return Ok(());
    };
    let face = topo.face(face_id)?;
    let mut wire_ids = vec![face.outer_wire()];
    wire_ids.extend(face.inner_wires().iter().copied());

    if loop_ids.len() != wire_ids.len() {
        return Err(TopologyError::LoopWireMismatch { face: face_id });
    }
    for (&loop_id, &wire_id) in loop_ids.iter().zip(&wire_ids) {
        let boundary_loop = topo.face_loop(loop_id)?;
        let wire = topo.wire(wire_id)?;
        if boundary_loop.face() != face_id
            || boundary_loop.is_closed() != wire.is_closed()
            || boundary_loop.coedges().len() != wire.edges().len()
        {
            return Err(TopologyError::LoopWireMismatch { face: face_id });
        }
        for (&coedge_id, oriented) in boundary_loop.coedges().iter().zip(wire.edges()) {
            let coedge = topo.coedge(coedge_id)?;
            if coedge.edge() != oriented.edge()
                || coedge.is_forward() != oriented.is_forward()
                || coedge.parent_loop() != loop_id
            {
                return Err(TopologyError::LoopWireMismatch { face: face_id });
            }
        }
    }
    Ok(())
}

/// Validates that a loop's coedges connect end-to-start under their
/// orientations, and (for a closed loop) that the last connects back to
/// the first.
///
/// Connection follows the same rule as [`validate_wire_closed`]: `VertexId`
/// equality, falling back to coincident position within `CLOSURE_POS_TOL`.
///
/// # Errors
///
/// Returns [`TopologyError::LoopNotConnected`] when adjacent coedges do
/// not connect, or a not-found error when a referenced entity is stale.
pub fn validate_loop_connected(
    topo: &Topology,
    loop_id: crate::face_loop::LoopId,
) -> Result<(), TopologyError> {
    let boundary_loop = topo.face_loop(loop_id)?;
    let face = boundary_loop.face();
    let coedges = boundary_loop.coedges();
    if coedges.is_empty() {
        return Err(TopologyError::LoopNotConnected { face });
    }

    let connects = |a: crate::vertex::VertexId, b: crate::vertex::VertexId| {
        if a == b {
            return Ok(true);
        }
        let pa = topo.vertex(a)?.point();
        let pb = topo.vertex(b)?.point();
        Ok::<bool, TopologyError>((pa - pb).length() <= CLOSURE_POS_TOL)
    };
    let oriented_end = |coedge: &crate::coedge::Coedge| -> Result<_, TopologyError> {
        let edge = topo.edge(coedge.edge())?;
        Ok(if coedge.is_forward() {
            edge.end()
        } else {
            edge.start()
        })
    };
    let oriented_start = |coedge: &crate::coedge::Coedge| -> Result<_, TopologyError> {
        let edge = topo.edge(coedge.edge())?;
        Ok(if coedge.is_forward() {
            edge.start()
        } else {
            edge.end()
        })
    };

    for window in coedges.windows(2) {
        let current = topo.coedge(window[0])?;
        let next = topo.coedge(window[1])?;
        if !connects(oriented_end(current)?, oriented_start(next)?)? {
            return Err(TopologyError::LoopNotConnected { face });
        }
    }
    if boundary_loop.is_closed() {
        let last = topo.coedge(coedges[coedges.len() - 1])?;
        let first = topo.coedge(coedges[0])?;
        if !connects(oriented_end(last)?, oriented_start(first)?)? {
            return Err(TopologyError::LoopNotConnected { face });
        }
    }
    Ok(())
}

/// Result of a `SameParameter` deviation check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SameParameterReport {
    /// Largest measured deviation or certified upper bound between the 3D
    /// curve and the pcurve's surface image, in model units.
    pub max_deviation: f64,
    /// Associated measured witness parameter. A certified upper bound need
    /// not be attained at this parameter.
    pub at_parameter: f64,
    /// Number of evaluation points or proof witnesses used.
    pub samples: usize,
}

/// Failure while proving one oriented edge-use curve contract.
///
/// This additive envelope preserves the stable diagnostics of topology and
/// edge-domain failures without extending the exhaustive [`TopologyError`]
/// enum.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CurveUseValidationError {
    /// A topology lookup or tolerance contract failed.
    #[error(transparent)]
    Topology(#[from] TopologyError),
    /// The edge lacks a valid authoritative parameter range.
    #[error(transparent)]
    EdgeDomain(#[from] crate::edge::EdgeDomainError),
    /// A stored pcurve use contains or evaluates to non-finite geometry.
    #[error(
        "pcurve of edge {edge:?} on face {face:?} ({orientation}) is non-finite in {component}"
    )]
    NonFinitePcurveUse {
        /// Edge carrying the poisoned pcurve use.
        edge: crate::edge::EdgeId,
        /// Face whose parameter space contains the pcurve.
        face: crate::face::FaceId,
        /// Stable orientation label.
        orientation: &'static str,
        /// Stable component label identifying what was non-finite.
        component: &'static str,
    },
    /// A validation tolerance is negative or non-finite.
    #[error("invalid curve-use validation tolerance {tolerance}")]
    InvalidTolerance {
        /// Rejected tolerance.
        tolerance: f64,
    },
    /// The stored curve-use combination has no certified SameParameter proof.
    #[error(
        "SameParameter proof is unavailable for {pcurve_type} pcurve / {surface_type} surface / {edge_type} edge on {orientation} use of edge {edge:?}, face {face:?}"
    )]
    SameParameterProofUnavailable {
        /// Edge whose curve use cannot yet be certified.
        edge: crate::edge::EdgeId,
        /// Face whose surface image cannot yet be certified.
        face: crate::face::FaceId,
        /// Stable orientation label.
        orientation: &'static str,
        /// Stable pcurve type label.
        pcurve_type: &'static str,
        /// Stable face-surface type label.
        surface_type: &'static str,
        /// Stable 3D edge-curve type label.
        edge_type: &'static str,
    },
    /// The stored curve-use combination has no certified SameRange proof.
    #[error(
        "SameRange proof is unavailable for {surface_type} surface on {orientation} use of edge {edge:?}, face {face:?}"
    )]
    SameRangeProofUnavailable {
        /// Edge whose range cannot yet be certified.
        edge: crate::edge::EdgeId,
        /// Face whose surface image cannot yet be certified.
        face: crate::face::FaceId,
        /// Stable orientation label.
        orientation: &'static str,
        /// Stable face-surface type label.
        surface_type: &'static str,
    },
}

impl remus_math::diagnostic::ToDiagnostic for CurveUseValidationError {
    fn diagnostic(&self) -> remus_math::diagnostic::Diagnostic {
        use remus_math::diagnostic::{Diagnostic, FailureCategory};

        match self {
            Self::Topology(error) => error.diagnostic(),
            Self::EdgeDomain(error) => error.diagnostic(),
            Self::NonFinitePcurveUse {
                edge,
                face,
                orientation,
                component,
            } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "pcurve_non_finite",
                self.to_string(),
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("orientation", *orientation)
            .with_detail("component", *component),
            Self::InvalidTolerance { tolerance } => {
                let diagnostic = Diagnostic::new(
                    FailureCategory::InvalidInput,
                    "curve_use_tolerance_invalid",
                    self.to_string(),
                );
                if tolerance.is_finite() {
                    diagnostic.with_detail("tolerance", *tolerance)
                } else {
                    diagnostic
                }
            }
            Self::SameParameterProofUnavailable {
                edge,
                face,
                orientation,
                pcurve_type,
                surface_type,
                edge_type,
            } => Diagnostic::new(
                FailureCategory::Unsupported,
                "same_parameter_proof_unavailable",
                self.to_string(),
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("orientation", *orientation)
            .with_detail("pcurve_type", *pcurve_type)
            .with_detail("surface_type", *surface_type)
            .with_detail("edge_type", *edge_type),
            Self::SameRangeProofUnavailable {
                edge,
                face,
                orientation,
                surface_type,
            } => Diagnostic::new(
                FailureCategory::Unsupported,
                "same_range_proof_unavailable",
                self.to_string(),
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("orientation", *orientation)
            .with_detail("surface_type", *surface_type),
        }
    }
}

const fn orientation_label(forward: bool) -> &'static str {
    if forward { "forward" } else { "reversed" }
}

fn non_finite_pcurve_use(
    edge: crate::edge::EdgeId,
    face: crate::face::FaceId,
    forward: bool,
    component: &'static str,
) -> CurveUseValidationError {
    CurveUseValidationError::NonFinitePcurveUse {
        edge,
        face,
        orientation: orientation_label(forward),
        component,
    }
}

fn pcurve_type_label(curve: &remus_math::curves2d::Curve2D) -> &'static str {
    match curve {
        remus_math::curves2d::Curve2D::Line(_) => "line",
        remus_math::curves2d::Curve2D::Circle(_) => "circle",
        remus_math::curves2d::Curve2D::Ellipse(_) => "ellipse",
        remus_math::curves2d::Curve2D::Nurbs(_) => "nurbs",
    }
}

fn pcurve_definition_is_finite(curve: &remus_math::curves2d::Curve2D) -> bool {
    match curve {
        remus_math::curves2d::Curve2D::Line(line) => {
            line.origin().0.iter().all(|value| value.is_finite())
                && line.direction().0.iter().all(|value| value.is_finite())
        }
        remus_math::curves2d::Curve2D::Circle(circle) => {
            circle.center().0.iter().all(|value| value.is_finite()) && circle.radius().is_finite()
        }
        remus_math::curves2d::Curve2D::Ellipse(ellipse) => {
            ellipse.center().0.iter().all(|value| value.is_finite())
                && ellipse.semi_major().is_finite()
                && ellipse.semi_minor().is_finite()
                && ellipse.rotation().is_finite()
        }
        remus_math::curves2d::Curve2D::Nurbs(nurbs) => {
            nurbs.knots().iter().all(|value| value.is_finite())
                && nurbs.weights().iter().all(|value| value.is_finite())
                && nurbs
                    .control_points()
                    .iter()
                    .all(|point| point.0.iter().all(|value| value.is_finite()))
        }
    }
}

/// Measures how far a pcurve's surface image deviates from its 3D edge
/// under the shared normalized parameterization (`SameParameter`).
///
/// Samples `samples + 1` points. The 3D edge parameter comes from the
/// edge's stored trim when present ([`crate::edge::Edge::trim`]), otherwise
/// from endpoint-projection reconstruction. Returns `Ok(None)` when the
/// check does not apply: no pcurve stored for that use, or a surface with
/// no UV evaluation (planes).
///
/// # Errors
///
/// Returns a not-found error when the edge, face, or a bounding vertex is
/// stale.
pub fn check_same_parameter(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    samples: usize,
) -> Result<Option<SameParameterReport>, TopologyError> {
    let Some(pcurve) = topo.pcurve_oriented(edge_id, face_id, forward) else {
        return Ok(None);
    };
    let surface = topo.face(face_id)?.surface().clone();
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge.domain_with_endpoints(start, end);
    let (p0, p1) = (pcurve.t_start(), pcurve.t_end());

    let samples = samples.max(1);
    let mut max_deviation = 0.0_f64;
    let mut at_parameter = if p0.is_finite() { p0 } else { 0.0 };
    if !p0.is_finite() || !p1.is_finite() {
        return Ok(Some(SameParameterReport {
            max_deviation: f64::MAX,
            at_parameter,
            samples: samples + 1,
        }));
    }
    for k in 0..=samples {
        #[allow(clippy::cast_precision_loss)]
        let f = k as f64 / samples as f64;
        let tp = p0 + f * (p1 - p0);
        let uv = pcurve.evaluate(tp);
        if !tp.is_finite() || !uv.0.iter().all(|value| value.is_finite()) {
            max_deviation = f64::MAX;
            at_parameter = if tp.is_finite() { tp } else { 0.0 };
            break;
        }
        let Some(on_surface) = surface.evaluate(uv.x(), uv.y()) else {
            return Ok(None);
        };
        // The pcurve traces the USE: its start maps to the oriented start,
        // which is the edge's end for a reversed use.
        let g = if forward {
            t0 + f * (t1 - t0)
        } else {
            t1 - f * (t1 - t0)
        };
        let on_curve = edge.curve().evaluate_with_endpoints(g, start, end);
        let deviation = (on_surface - on_curve).length();
        if !g.is_finite()
            || !on_surface.0.iter().all(|value| value.is_finite())
            || !on_curve.0.iter().all(|value| value.is_finite())
            || !deviation.is_finite()
        {
            max_deviation = f64::MAX;
            at_parameter = tp;
            break;
        }
        if deviation > max_deviation {
            max_deviation = deviation;
            at_parameter = tp;
        }
    }
    Ok(Some(SameParameterReport {
        max_deviation,
        at_parameter,
        samples: samples + 1,
    }))
}

/// Strictly measures `SameParameter` for one oriented edge use.
///
/// Unlike [`check_same_parameter`], this function never reconstructs a
/// non-Line edge domain from its endpoints. A stored pcurve is inspected for
/// non-finite data, and every unsupported curve/surface combination is a typed
/// proof refusal rather than a vacuous success.
///
/// # Errors
///
/// Returns a pinned [`CurveUseValidationError`] for stale topology, missing or
/// invalid edge-domain authority, non-finite pcurve geometry, or a
/// curve/surface combination without a supported proof.
pub fn check_same_parameter_strict(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    _samples: usize,
) -> Result<Option<SameParameterReport>, CurveUseValidationError> {
    let surface = topo.face(face_id)?.surface().clone();
    let edge = topo.edge(edge_id)?;
    let Some(pcurve) = topo.pcurve_oriented(edge_id, face_id, forward) else {
        return Ok(None);
    };
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let _domain = edge.strict_domain()?;
    let (p0, p1) = (pcurve.t_start(), pcurve.t_end());
    if !p0.is_finite() || !p1.is_finite() {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "parameter_bounds",
        ));
    }
    if !pcurve_definition_is_finite(pcurve.curve()) {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "curve_definition",
        ));
    }
    let uv0 = pcurve.evaluate(p0);
    let uv1 = pcurve.evaluate(p1);
    if !uv0.0.iter().all(|value| value.is_finite()) || !uv1.0.iter().all(|value| value.is_finite())
    {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "pcurve_evaluation",
        ));
    }
    if let (
        remus_math::curves2d::Curve2D::Line(line),
        crate::face::FaceSurface::Cylinder(cylinder),
        crate::edge::EdgeCurve::Line,
    ) = (pcurve.curve(), &surface, edge.curve())
        && matches!(line.direction().x().to_bits(), 0 | 0x8000_0000_0000_0000)
        && uv0.x().to_bits() == uv1.x().to_bits()
    {
        let on_surface_start = cylinder.evaluate(uv0.x(), uv0.y());
        let on_surface_end = cylinder.evaluate(uv1.x(), uv1.y());
        let (oriented_start, oriented_end) = if forward { (start, end) } else { (end, start) };
        let d0 = (on_surface_start - oriented_start).length();
        let d1 = (on_surface_end - oriented_end).length();
        if !on_surface_start.0.iter().all(|value| value.is_finite())
            || !on_surface_end.0.iter().all(|value| value.is_finite())
            || !d0.is_finite()
            || !d1.is_finite()
        {
            return Err(non_finite_pcurve_use(
                edge_id,
                face_id,
                forward,
                "surface_or_curve_evaluation",
            ));
        }
        let mut coordinate_scale = 1.0_f64;
        for value in on_surface_start
            .0
            .iter()
            .chain(on_surface_end.0.iter())
            .chain(oriented_start.0.iter())
            .chain(oriented_end.0.iter())
            .chain(uv0.0.iter())
            .chain(uv1.0.iter())
            .chain(line.origin().0.iter())
            .chain(line.direction().0.iter())
            .chain(cylinder.origin().0.iter())
            .chain(cylinder.axis().0.iter())
            .chain(cylinder.x_axis().0.iter())
            .chain(cylinder.y_axis().0.iter())
        {
            coordinate_scale = coordinate_scale.max(value.abs());
        }
        for value in [p0, p1, cylinder.radius()] {
            coordinate_scale = coordinate_scale.max(value.abs());
        }
        let arithmetic_bound = 64.0 * f64::EPSILON * coordinate_scale;
        if arithmetic_bound > remus_math::tolerance::Tolerance::new().linear {
            return Err(CurveUseValidationError::SameParameterProofUnavailable {
                edge: edge_id,
                face: face_id,
                orientation: orientation_label(forward),
                pcurve_type: pcurve_type_label(pcurve.curve()),
                surface_type: surface.type_tag(),
                edge_type: edge.curve().type_tag(),
            });
        }
        let (endpoint_deviation, at_parameter) = if d0 >= d1 { (d0, p0) } else { (d1, p1) };
        return Ok(Some(SameParameterReport {
            max_deviation: endpoint_deviation + arithmetic_bound,
            at_parameter,
            samples: 2,
        }));
    }

    Err(CurveUseValidationError::SameParameterProofUnavailable {
        edge: edge_id,
        face: face_id,
        orientation: orientation_label(forward),
        pcurve_type: pcurve_type_label(pcurve.curve()),
        surface_type: surface.type_tag(),
        edge_type: edge.curve().type_tag(),
    })
}

/// Strictly enforces `SameParameter` for one oriented edge use.
///
/// # Errors
///
/// Returns [`CurveUseValidationError`] when validation cannot be proved or
/// the measured deviation exceeds `tolerance`.
pub fn validate_same_parameter_strict(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    tolerance: f64,
    samples: usize,
) -> Result<(), CurveUseValidationError> {
    if !tolerance.is_finite() || tolerance.is_sign_negative() {
        return Err(CurveUseValidationError::InvalidTolerance { tolerance });
    }
    if let Some(report) = check_same_parameter_strict(topo, edge_id, face_id, forward, samples)?
        && report.max_deviation > tolerance
    {
        return Err(TopologyError::SameParameterExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation: report.max_deviation,
            at_parameter: report.at_parameter,
            tolerance,
        }
        .into());
    }
    Ok(())
}

/// Enforces `SameParameter` within `tolerance`.
///
/// Not-applicable configurations (no pcurve, planar face) pass vacuously.
///
/// # Errors
///
/// Returns [`TopologyError::SameParameterExceeded`] when the sampled
/// deviation exceeds `tolerance`, or a not-found error for stale entities.
pub fn validate_same_parameter(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    tolerance: f64,
    samples: usize,
) -> Result<(), TopologyError> {
    if let Some(report) = check_same_parameter(topo, edge_id, face_id, forward, samples)?
        && report.max_deviation > tolerance
    {
        return Err(TopologyError::SameParameterExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation: report.max_deviation,
            at_parameter: report.at_parameter,
            tolerance,
        });
    }
    Ok(())
}

/// Measures how far a pcurve's endpoints miss the edge's bounding vertices
/// on the surface (`SameRange`). Returns the larger of the two endpoint
/// deviations, or `Ok(None)` when the check does not apply.
///
/// # Errors
///
/// Returns a not-found error when the edge, face, or a bounding vertex is
/// stale.
pub fn check_same_range(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
) -> Result<Option<f64>, TopologyError> {
    let Some(pcurve) = topo.pcurve_oriented(edge_id, face_id, forward) else {
        return Ok(None);
    };
    let surface = topo.face(face_id)?.surface().clone();
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (oriented_start, oriented_end) = if forward { (start, end) } else { (end, start) };

    let uv0 = pcurve.evaluate(pcurve.t_start());
    let uv1 = pcurve.evaluate(pcurve.t_end());
    if !pcurve.t_start().is_finite()
        || !pcurve.t_end().is_finite()
        || !uv0.0.iter().all(|value| value.is_finite())
        || !uv1.0.iter().all(|value| value.is_finite())
    {
        return Ok(Some(f64::MAX));
    }
    let (Some(s0), Some(s1)) = (
        surface.evaluate(uv0.x(), uv0.y()),
        surface.evaluate(uv1.x(), uv1.y()),
    ) else {
        return Ok(None);
    };
    if !s0.0.iter().all(|value| value.is_finite())
        || !s1.0.iter().all(|value| value.is_finite())
        || !oriented_start.0.iter().all(|value| value.is_finite())
        || !oriented_end.0.iter().all(|value| value.is_finite())
    {
        return Ok(Some(f64::MAX));
    }
    let d0 = (s0 - oriented_start).length();
    let d1 = (s1 - oriented_end).length();
    Ok(Some(d0.max(d1)))
}

/// Strictly measures `SameRange` for one oriented edge use.
///
/// The edge must carry authoritative domain data even though SameRange itself
/// compares only the pcurve endpoints: otherwise endpoint agreement could
/// certify a use whose interior 3D parameterization is unknown.
///
/// # Errors
///
/// Returns a pinned [`CurveUseValidationError`] for stale topology, missing or
/// invalid edge-domain authority, non-finite pcurve geometry, or a surface
/// without a supported endpoint proof.
pub fn check_same_range_strict(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
) -> Result<Option<f64>, CurveUseValidationError> {
    let surface = topo.face(face_id)?.surface().clone();
    let edge = topo.edge(edge_id)?;
    let Some(pcurve) = topo.pcurve_oriented(edge_id, face_id, forward) else {
        return Ok(None);
    };
    let _domain = edge.strict_domain()?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (oriented_start, oriented_end) = if forward { (start, end) } else { (end, start) };

    if !pcurve.t_start().is_finite() || !pcurve.t_end().is_finite() {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "parameter_bounds",
        ));
    }
    if !pcurve_definition_is_finite(pcurve.curve()) {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "curve_definition",
        ));
    }
    let uv0 = pcurve.evaluate(pcurve.t_start());
    let uv1 = pcurve.evaluate(pcurve.t_end());
    if !uv0.0.iter().all(|value| value.is_finite()) || !uv1.0.iter().all(|value| value.is_finite())
    {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "pcurve_evaluation",
        ));
    }
    let (Some(s0), Some(s1)) = (
        surface.evaluate(uv0.x(), uv0.y()),
        surface.evaluate(uv1.x(), uv1.y()),
    ) else {
        return Err(CurveUseValidationError::SameRangeProofUnavailable {
            edge: edge_id,
            face: face_id,
            orientation: orientation_label(forward),
            surface_type: surface.type_tag(),
        });
    };
    if !s0.0.iter().all(|value| value.is_finite())
        || !s1.0.iter().all(|value| value.is_finite())
        || !oriented_start.0.iter().all(|value| value.is_finite())
        || !oriented_end.0.iter().all(|value| value.is_finite())
    {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "surface_or_endpoint_evaluation",
        ));
    }
    let d0 = (s0 - oriented_start).length();
    let d1 = (s1 - oriented_end).length();
    let max_deviation = d0.max(d1);
    if !max_deviation.is_finite() {
        return Err(non_finite_pcurve_use(
            edge_id,
            face_id,
            forward,
            "range_deviation",
        ));
    }
    Ok(Some(max_deviation))
}

/// Strictly enforces `SameRange` for one oriented edge use.
///
/// # Errors
///
/// Returns [`CurveUseValidationError`] when validation cannot be proved or
/// the measured deviation exceeds `tolerance`.
pub fn validate_same_range_strict(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    tolerance: f64,
) -> Result<(), CurveUseValidationError> {
    if !tolerance.is_finite() || tolerance.is_sign_negative() {
        return Err(CurveUseValidationError::InvalidTolerance { tolerance });
    }
    if let Some(max_deviation) = check_same_range_strict(topo, edge_id, face_id, forward)?
        && max_deviation > tolerance
    {
        return Err(TopologyError::SameRangeExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation,
            tolerance,
        }
        .into());
    }
    Ok(())
}

/// Coverage evidence from strict pcurve validation over a solid boundary.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PcurveContractSummary {
    /// Total oriented edge uses visited across outer and cavity shells.
    pub boundary_uses: usize,
    /// Boundary uses carrying an oriented pcurve.
    pub stored_pcurves: usize,
    /// Stored uses for which both strict comparisons were proved.
    pub validated_uses: usize,
}

/// Strictly validates every stored oriented pcurve use of a solid.
///
/// Missing pcurves are not treated as proof. Every stored pcurve must have a
/// supported, certified curve/surface combination; unsupported combinations
/// return a typed refusal. The returned summary lets callers require
/// non-vacuous coverage for fixtures that are expected to carry pcurves.
///
/// # Errors
///
/// Returns [`CurveUseValidationError`] on stale topology, invalid tolerance,
/// missing edge-domain authority for a stored use, non-finite pcurve data, an
/// unsupported proof combination, or a SameParameter/SameRange tolerance
/// violation.
pub fn validate_solid_pcurve_contracts(
    topo: &Topology,
    solid: crate::solid::SolidId,
    tolerance: f64,
    samples: usize,
) -> Result<PcurveContractSummary, CurveUseValidationError> {
    if !tolerance.is_finite() || tolerance.is_sign_negative() {
        return Err(CurveUseValidationError::InvalidTolerance { tolerance });
    }

    let mut summary = PcurveContractSummary::default();
    for face_id in crate::explorer::solid_faces(topo, solid)? {
        let face = topo.face(face_id)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                summary.boundary_uses += 1;
                let edge_id = oriented.edge();
                let forward = oriented.is_forward();
                topo.edge(edge_id)?;
                if topo.pcurve_oriented(edge_id, face_id, forward).is_none() {
                    continue;
                }
                summary.stored_pcurves += 1;
                let parameter =
                    check_same_parameter_strict(topo, edge_id, face_id, forward, samples)?;
                let range = check_same_range_strict(topo, edge_id, face_id, forward)?;
                if let Some(report) = parameter
                    && report.max_deviation > tolerance
                {
                    return Err(TopologyError::SameParameterExceeded {
                        edge: edge_id,
                        face: face_id,
                        max_deviation: report.max_deviation,
                        at_parameter: report.at_parameter,
                        tolerance,
                    }
                    .into());
                }
                if let Some(max_deviation) = range
                    && max_deviation > tolerance
                {
                    return Err(TopologyError::SameRangeExceeded {
                        edge: edge_id,
                        face: face_id,
                        max_deviation,
                        tolerance,
                    }
                    .into());
                }
                if parameter.is_some() && range.is_some() {
                    summary.validated_uses += 1;
                }
            }
        }
    }
    Ok(summary)
}

/// Enforces `SameRange` within `tolerance`.
///
/// # Errors
///
/// Returns [`TopologyError::SameRangeExceeded`] when either endpoint
/// deviation exceeds `tolerance`, or a not-found error for stale entities.
pub fn validate_same_range(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    tolerance: f64,
) -> Result<(), TopologyError> {
    if let Some(max_deviation) = check_same_range(topo, edge_id, face_id, forward)?
        && max_deviation > tolerance
    {
        return Err(TopologyError::SameRangeExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation,
            tolerance,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::vec::{Point3, Vec3};

    use crate::edge::{Edge, EdgeCurve};
    use crate::face::{Face, FaceSurface};
    use crate::topology::Topology;
    use crate::wire::OrientedEdge;

    use super::*;

    /// Helper: builds a closed triangular wire from 3 vertices.
    fn make_triangle(topo: &mut Topology) -> WireId {
        use crate::vertex::Vertex;

        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));

        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));

        topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e2, true),
                ],
                true,
            )
            .unwrap(),
        )
    }

    #[test]
    fn validate_wire_closed_triangle() {
        let mut topo = Topology::new();
        let wid = make_triangle(&mut topo);
        let wire = topo.wire(wid).unwrap();
        assert!(validate_wire_closed(wire, &topo).is_ok());
    }

    #[test]
    fn manifold_two_face_shell() {
        // Two triangular faces sharing one edge — each edge used at most 2 times.
        let mut topo = Topology::new();

        let v0 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let v3 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 1.0, 0.0), 1e-7));

        let shared = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let ea0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e_a1 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        let eb0 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
        let eb1 = topo.add_edge(Edge::new(v3, v1, EdgeCurve::Line));

        let w0 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(ea0, true),
                    OrientedEdge::new(shared, true),
                    OrientedEdge::new(e_a1, true),
                ],
                true,
            )
            .unwrap(),
        );
        let w1 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(shared, false),
                    OrientedEdge::new(eb0, true),
                    OrientedEdge::new(eb1, true),
                ],
                true,
            )
            .unwrap(),
        );

        let normal = Vec3::new(0.0, 0.0, 1.0);
        let f0 = topo.add_face(Face::new(w0, vec![], FaceSurface::Plane { normal, d: 0.0 }));
        let f1 = topo.add_face(Face::new(w1, vec![], FaceSurface::Plane { normal, d: 0.0 }));

        let shell = Shell::new(vec![f0, f1]).unwrap();
        assert!(validate_shell_manifold(&shell, &topo).is_ok());
    }

    #[test]
    fn closed_validation_rejects_free_edges() {
        // A single triangular face: every edge is used once -> open shell.
        let mut topo = Topology::new();
        let wid = make_triangle(&mut topo);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let f = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane { normal, d: 0.0 },
        ));
        let shell = crate::shell::Shell::new(vec![f]).unwrap();

        assert!(validate_shell_manifold(&shell, &topo).is_ok());
        let err = validate_shell_closed(&shell, &topo).unwrap_err();
        assert!(
            matches!(err, TopologyError::NonManifold { .. }),
            "expected NonManifold for free edges, got {err:?}"
        );
    }

    #[test]
    fn closed_validation_accepts_two_sided_triangle() {
        // Two faces over the same three edges (front + back): every edge
        // is used exactly twice.
        let mut topo = Topology::new();
        let wid = make_triangle(&mut topo);
        let back_oes: Vec<OrientedEdge> = topo
            .wire(wid)
            .unwrap()
            .edges()
            .iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
        let back_wid = topo.add_wire(Wire::new(back_oes, true).unwrap());
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let f0 = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane { normal, d: 0.0 },
        ));
        let f1 = topo.add_face(Face::new(
            back_wid,
            vec![],
            FaceSurface::Plane {
                normal: -normal,
                d: 0.0,
            },
        ));
        let shell = crate::shell::Shell::new(vec![f0, f1]).unwrap();

        assert!(validate_shell_closed(&shell, &topo).is_ok());
    }

    #[test]
    fn non_manifold_three_face_shared_edge() {
        // Three faces sharing a single edge -> non-manifold.
        let mut topo = Topology::new();

        let v0 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let v3 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 1.0, 0.0), 1e-7));
        let v4 = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.5, 0.5, 1.0), 1e-7));

        let shared = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        let e_a = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e_b = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        let w0 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(shared, true),
                    OrientedEdge::new(e_a, true),
                    OrientedEdge::new(e_b, true),
                ],
                true,
            )
            .unwrap(),
        );

        let e_c = topo.add_edge(Edge::new(v1, v3, EdgeCurve::Line));
        let e_d = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));
        let w1 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(shared, true),
                    OrientedEdge::new(e_c, true),
                    OrientedEdge::new(e_d, true),
                ],
                true,
            )
            .unwrap(),
        );

        // Face 3: v0-v1-v4 — third face sharing the same edge
        let e_e = topo.add_edge(Edge::new(v1, v4, EdgeCurve::Line));
        let e_f = topo.add_edge(Edge::new(v4, v0, EdgeCurve::Line));
        let w2 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(shared, true),
                    OrientedEdge::new(e_e, true),
                    OrientedEdge::new(e_f, true),
                ],
                true,
            )
            .unwrap(),
        );

        let normal = Vec3::new(0.0, 0.0, 1.0);
        let f0 = topo.add_face(Face::new(w0, vec![], FaceSurface::Plane { normal, d: 0.0 }));
        let f1 = topo.add_face(Face::new(w1, vec![], FaceSurface::Plane { normal, d: 0.0 }));
        let f2 = topo.add_face(Face::new(w2, vec![], FaceSurface::Plane { normal, d: 0.0 }));

        let shell = Shell::new(vec![f0, f1, f2]).unwrap();
        let result = validate_shell_manifold(&shell, &topo);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            matches!(err, TopologyError::NonManifold { .. }),
            "expected NonManifold, got {err:?}"
        );
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod same_parameter_tests {
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
    use remus_math::diagnostic::ToDiagnostic;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};

    use crate::TopologyError;
    use crate::edge::{Edge, EdgeCurve, EdgeId};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::pcurve::PCurve;
    use crate::shell::Shell;
    use crate::solid::Solid;
    use crate::topology::Topology;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    use super::{
        CurveUseValidationError, check_same_parameter_strict, validate_same_parameter_strict,
        validate_same_range_strict, validate_solid_pcurve_contracts,
    };

    const TAU: f64 = std::f64::consts::TAU;

    /// Cylinder side face bounded by its bottom rim circle (full circle,
    /// closed wire). The rim's exact pcurve is the line v = 0, u = t.
    fn cylinder_with_rim() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        // Anchor the seam vertex to the curve's own zero-angle point so the
        // fixture does not assume how the reference frame is derived.
        let p0 = remus_math::traits::ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(p0, 1e-7));
        let mut rim_edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        rim_edge.set_trim(Some((0.0, TAU)));
        let rim = topo.add_edge(rim_edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::with_ref_dir(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    1.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
        ));
        (topo, rim, face)
    }

    fn rim_pcurve(v_offset: f64) -> PCurve {
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(0.0, v_offset), Vec2::new(1.0, 0.0)).unwrap()),
            0.0,
            TAU,
        )
    }

    fn cylinder_seam() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let bottom = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let top = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 1.0), 1e-7));
        let seam = topo.add_edge(Edge::new(bottom, top, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::with_ref_dir(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    1.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
        ));
        (topo, seam, face)
    }

    fn seam_pcurve(u: f64, forward: bool) -> PCurve {
        let (v0, dv) = if forward { (0.0, 1.0) } else { (1.0, -1.0) };
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(u, v0), Vec2::new(0.0, dv)).unwrap()),
            0.0,
            1.0,
        )
    }

    #[test]
    fn periodic_rim_refuses_without_a_certified_same_parameter_bound() {
        let (mut topo, rim, face) = cylinder_with_rim();
        topo.set_pcurve_oriented(rim, face, true, rim_pcurve(0.0));

        let error = check_same_parameter_strict(&topo, rim, face, true, 32).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
        assert_eq!(
            error.diagnostic().code(),
            "same_parameter_proof_unavailable"
        );
        validate_same_range_strict(&topo, rim, face, true, 1e-7).unwrap();
    }

    #[test]
    fn offset_pcurve_fails_with_typed_tolerance_violation() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3, true));

        let err = validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap_err();
        let CurveUseValidationError::Topology(TopologyError::SameParameterExceeded {
            max_deviation,
            ..
        }) = err
        else {
            unreachable!("expected SameParameterExceeded, got {err:?}")
        };
        let expected_chord = 2.0 * (0.3_f64 / 2.0).sin();
        assert!((max_deviation - expected_chord).abs() < 1e-9);

        assert!(matches!(
            validate_same_range_strict(&topo, seam, face, true, 1e-7),
            Err(CurveUseValidationError::Topology(
                TopologyError::SameRangeExceeded { .. }
            ))
        ));
    }

    #[test]
    fn seam_branches_validate_independently_by_orientation() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true));
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false));

        for forward in [true, false] {
            validate_same_parameter_strict(&topo, seam, face, forward, 1e-7, 32).unwrap();
            validate_same_range_strict(&topo, seam, face, forward, 1e-7).unwrap();
        }

        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU + 0.2, false));
        validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap();
        validate_same_range_strict(&topo, seam, face, true, 1e-7).unwrap();
        assert!(matches!(
            validate_same_parameter_strict(&topo, seam, face, false, 1e-7, 32),
            Err(CurveUseValidationError::Topology(
                TopologyError::SameParameterExceeded { .. }
            ))
        ));
        assert!(matches!(
            validate_same_range_strict(&topo, seam, face, false, 1e-7),
            Err(CurveUseValidationError::Topology(
                TopologyError::SameRangeExceeded { .. }
            ))
        ));
    }

    #[test]
    fn translated_seam_refuses_when_roundoff_exceeds_the_certified_bound() {
        let mut topo = Topology::new();
        let cylinder = CylindricalSurface::with_ref_dir(
            Point3::new(1e15, -1e15, 1e15),
            Vec3::new(1.0, 0.0, 1.0),
            1.0,
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let bottom_point = cylinder.evaluate(0.0, 0.0);
        let top_point = cylinder.evaluate(0.0, 1.0);
        let bottom = topo.add_vertex(Vertex::new(bottom_point, 1e-7));
        let top = topo.add_vertex(Vertex::new(top_point, 1e-7));
        let seam = topo.add_edge(Edge::new(bottom, top, EdgeCurve::Line));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(seam, true)], false).unwrap());
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true));

        let error = validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
        assert_eq!(
            error.diagnostic().code(),
            "same_parameter_proof_unavailable"
        );
    }

    #[test]
    fn cancellation_heavy_pcurve_parameters_are_not_certified_from_endpoints() {
        let (mut topo, seam, face) = cylinder_seam();
        let cancellation_line = Line2D::new(Point2::new(0.0, -1e16), Vec2::new(0.0, 1.0)).unwrap();
        let p0 = 1e16;
        let p1 = 1e16 + 2.0;
        assert!((cancellation_line.evaluate(p0) - Point2::new(0.0, 0.0)).length() < 1e-15);
        assert!((cancellation_line.evaluate(p1) - Point2::new(0.0, 2.0)).length() < 1e-15);
        let seam_end = topo.edge(seam).unwrap().end();
        topo.vertex_mut(seam_end)
            .unwrap()
            .set_point(Point3::new(1.0, 0.0, 2.0));
        topo.set_pcurve_oriented(
            seam,
            face,
            true,
            PCurve::new(Curve2D::Line(cancellation_line), p0, p1),
        );

        let error = validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
        assert_eq!(
            error.diagnostic().code(),
            "same_parameter_proof_unavailable"
        );
    }

    #[test]
    fn rounded_equal_u_endpoints_do_not_hide_a_nonzero_u_slope() {
        let mut topo = Topology::new();
        let radius = 1e6;
        let u0 = 1e6;
        let cylinder = CylindricalSurface::with_ref_dir(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            radius,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let bottom = topo.add_vertex(Vertex::new(cylinder.evaluate(u0, 0.0), 1e-7));
        let top = topo.add_vertex(Vertex::new(cylinder.evaluate(u0, 1.0), 1e-7));
        let seam = topo.add_edge(Edge::new(bottom, top, EdgeCurve::Line));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(seam, true)], false).unwrap());
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
        let almost_vertical = Line2D::new(Point2::new(u0, 0.0), Vec2::new(1e-11, 1.0)).unwrap();
        assert_eq!(
            almost_vertical.evaluate(0.0).x().to_bits(),
            almost_vertical.evaluate(1.0).x().to_bits()
        );
        topo.set_pcurve_oriented(
            seam,
            face,
            true,
            PCurve::new(Curve2D::Line(almost_vertical), 0.0, 1.0),
        );

        let error = validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
        assert_eq!(
            error.diagnostic().code(),
            "same_parameter_proof_unavailable"
        );
    }

    #[test]
    fn stored_planar_pcurve_is_typed_unproven() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            });
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true));

        let parameter =
            validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap_err();
        assert!(matches!(
            parameter,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
        assert_eq!(
            parameter.diagnostic().code(),
            "same_parameter_proof_unavailable"
        );

        let range = validate_same_range_strict(&topo, seam, face, true, 1e-7).unwrap_err();
        assert!(matches!(
            range,
            CurveUseValidationError::SameRangeProofUnavailable { .. }
        ));
        assert_eq!(range.diagnostic().code(), "same_range_proof_unavailable");
    }

    #[test]
    fn localized_nurbs_bump_is_typed_unproven_instead_of_sampled_clean() {
        let (mut topo, seam, face) = cylinder_seam();
        let bowed = NurbsCurve2D::new(
            1,
            vec![0.0, 0.0, 0.1, 0.11, 0.12, 1.0, 1.0],
            vec![
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 0.1),
                Point2::new(0.3, 0.11),
                Point2::new(0.0, 0.12),
                Point2::new(0.0, 1.0),
            ],
            vec![1.0; 5],
        )
        .unwrap();
        topo.set_pcurve_oriented(
            seam,
            face,
            true,
            PCurve::new(Curve2D::Nurbs(bowed), 0.0, 1.0),
        );

        validate_same_range_strict(&topo, seam, face, true, 1e-7).unwrap();
        let error = validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
        assert_eq!(
            error.diagnostic().code(),
            "same_parameter_proof_unavailable"
        );
    }

    #[test]
    fn solid_pcurve_contract_summary_is_non_vacuous_and_oriented() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true));
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        let summary = validate_solid_pcurve_contracts(&topo, solid, 1e-7, 32).unwrap();
        assert_eq!(summary.boundary_uses, 2);
        assert_eq!(summary.stored_pcurves, 2);
        assert_eq!(summary.validated_uses, 2);

        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU + 0.2, false));
        assert!(matches!(
            validate_solid_pcurve_contracts(&topo, solid, 1e-7, 32),
            Err(CurveUseValidationError::Topology(
                TopologyError::SameParameterExceeded { .. }
            ))
        ));
    }

    #[test]
    fn strict_validation_tolerance_diagnostic_is_pinned() {
        use remus_math::diagnostic::FailureCategory;

        let (topo, rim, face) = cylinder_with_rim();
        for tolerance in [f64::NAN, f64::INFINITY, -1.0] {
            let error =
                validate_same_parameter_strict(&topo, rim, face, true, tolerance, 8).unwrap_err();
            assert!(matches!(
                error,
                CurveUseValidationError::InvalidTolerance { .. }
            ));
            assert_eq!(error.diagnostic().category(), FailureCategory::InvalidInput);
            assert_eq!(error.diagnostic().code(), "curve_use_tolerance_invalid");
        }
    }

    #[test]
    fn strict_validation_refuses_stale_ids_before_pcurve_vacuity() {
        let (mut topo, rim, face) = cylinder_with_rim();
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        topo.delete_solid(solid).unwrap();

        assert!(matches!(
            check_same_parameter_strict(&topo, rim, face, true, 8),
            Err(CurveUseValidationError::Topology(
                TopologyError::FaceNotFound(_) | TopologyError::EdgeNotFound(_)
            ))
        ));
    }

    #[test]
    fn non_finite_pcurve_fails_closed_with_pinned_diagnostics() {
        let (mut topo, rim, face) = cylinder_with_rim();
        topo.set_pcurve_oriented(
            rim,
            face,
            true,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
                f64::NAN,
                TAU,
            ),
        );

        let parameter =
            validate_same_parameter_strict(&topo, rim, face, true, 1e-7, 8).unwrap_err();
        assert!(matches!(
            parameter,
            CurveUseValidationError::NonFinitePcurveUse { .. }
        ));
        assert_eq!(parameter.diagnostic().code(), "pcurve_non_finite");

        let range = validate_same_range_strict(&topo, rim, face, true, 1e-7).unwrap_err();
        assert!(matches!(
            range,
            CurveUseValidationError::NonFinitePcurveUse { .. }
        ));
        assert_eq!(range.diagnostic().code(), "pcurve_non_finite");

        let poisoned = NurbsCurve2D::new(
            1,
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            vec![
                Point2::new(0.0, 0.0),
                Point2::new(f64::NAN, 0.5),
                Point2::new(0.0, 1.0),
            ],
            vec![1.0; 3],
        )
        .unwrap();
        topo.set_pcurve_oriented(
            rim,
            face,
            true,
            PCurve::new(Curve2D::Nurbs(poisoned), 0.0, 1.0),
        );
        for error in [
            validate_same_parameter_strict(&topo, rim, face, true, 1e-7, 8).unwrap_err(),
            validate_same_range_strict(&topo, rim, face, true, 1e-7).unwrap_err(),
        ] {
            assert!(matches!(
                error,
                CurveUseValidationError::NonFinitePcurveUse { .. }
            ));
            assert_eq!(error.diagnostic().code(), "pcurve_non_finite");
        }
    }

    #[test]
    fn same_parameter_refuses_missing_edge_domain() {
        let (mut topo, rim, face) = cylinder_with_rim();
        topo.edge_mut(rim).unwrap().set_trim(None);
        topo.set_pcurve_oriented(rim, face, true, rim_pcurve(0.0));
        let error = validate_same_parameter_strict(&topo, rim, face, true, 1e-7, 8).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::EdgeDomain(crate::edge::EdgeDomainError::Missing { .. })
        ));
        assert_eq!(error.diagnostic().code(), "edge_domain_missing");
    }

    #[test]
    fn missing_pcurve_passes_vacuously() {
        let (topo, rim, face) = cylinder_with_rim();
        assert!(
            check_same_parameter_strict(&topo, rim, face, true, 8)
                .unwrap()
                .is_none()
        );
        validate_same_parameter_strict(&topo, rim, face, true, 1e-7, 8).unwrap();
    }

    #[test]
    fn tolerance_violation_diagnostics_are_pinned() {
        use remus_math::diagnostic::FailureCategory;
        let (topo, rim, face) = cylinder_with_rim();
        drop(topo);
        let d = TopologyError::SameParameterExceeded {
            edge: rim,
            face,
            max_deviation: 0.3,
            at_parameter: 1.0,
            tolerance: 1e-7,
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::ToleranceViolation);
        assert_eq!(d.code(), "same_parameter_exceeded");

        let d = TopologyError::SameRangeExceeded {
            edge: rim,
            face,
            max_deviation: 0.3,
            tolerance: 1e-7,
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::ToleranceViolation);
        assert_eq!(d.code(), "same_range_exceeded");
    }
}
