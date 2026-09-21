//! Topology validation utilities.
//!
//! These functions check structural invariants of topological entities
//! such as wire closure and shell manifoldness.

use std::collections::HashMap;

use remus_math::tolerance::Tolerance;

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

/// Validates a face's authoritative loops against its compatibility wires
/// (RFC 0002 boundary-authority invariant).
///
/// Every valid face must have authoritative loops. Its compatibility wires
/// must agree exactly: loop count and order (outer first, then inner),
/// per-loop closure, and per-position edge identity and orientation. Loops
/// are written only by the kernel, so absence or divergence is a kernel bug.
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
        return Err(TopologyError::LoopWireMismatch { face: face_id });
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

/// Coverage evidence from whole-topology Loop/Coedge validation.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BoundaryAuthoritySummary {
    /// Live faces whose authoritative boundaries were checked.
    pub faces: usize,
    /// Live loops owned by those faces.
    pub loops: usize,
    /// Live coedges owned by those loops.
    pub coedges: usize,
    /// Edge/face pairs used twice as independently addressable seam branches.
    pub seam_edges: usize,
    /// Stored pcurve branches across complete seams.
    pub stored_seam_branches: usize,
}

/// Failure while validating whole-topology Loop/Coedge authority.
///
/// This additive envelope keeps the public exhaustive [`TopologyError`] enum
/// source-compatible while assigning stable diagnostics to ownership and seam
/// completeness failures introduced by RFC 0002.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum BoundaryAuthorityError {
    /// A referenced topology entity is stale or a local structural invariant failed.
    #[error(transparent)]
    Topology(#[from] TopologyError),
    /// A live loop is unowned or reused by several faces.
    #[error("loop {loop_id:?} has {owners} face owners (expected exactly one)")]
    LoopOwnershipInvalid {
        /// Invalid live loop.
        loop_id: crate::face_loop::LoopId,
        /// Number of owning faces found.
        owners: usize,
    },
    /// A live coedge is unowned or reused by several loops.
    #[error("coedge {coedge:?} has {owners} loop owners (expected exactly one)")]
    CoedgeOwnershipInvalid {
        /// Invalid live coedge.
        coedge: crate::coedge::CoedgeId,
        /// Number of owning faces or loops found.
        owners: usize,
    },
    /// Two uses share one compatibility key and cannot be addressed independently.
    #[error("edge {edge:?} on face {face:?} has duplicate {orientation} boundary uses")]
    DuplicateOrientedUse {
        /// Repeated edge.
        edge: crate::edge::EdgeId,
        /// Owning face.
        face: crate::face::FaceId,
        /// Stable orientation label.
        orientation: &'static str,
    },
    /// The compatibility index does not resolve to the authoritative coedge.
    #[error(
        "{orientation} use of edge {edge:?} on face {face:?} is not indexed to coedge {coedge:?}"
    )]
    IndexMismatch {
        /// Edge used by the coedge.
        edge: crate::edge::EdgeId,
        /// Owning face.
        face: crate::face::FaceId,
        /// Authoritative coedge.
        coedge: crate::coedge::CoedgeId,
        /// Stable orientation label.
        orientation: &'static str,
    },
    /// One branch of a repeated seam has pcurve authority while another does not.
    #[error(
        "seam edge {edge:?} on face {face:?} stores {stored} of {uses} required pcurve branches"
    )]
    IncompleteSeamPcurves {
        /// Repeated seam edge.
        edge: crate::edge::EdgeId,
        /// Owning face.
        face: crate::face::FaceId,
        /// Number of authoritative seam uses.
        uses: usize,
        /// Number of those uses carrying pcurves.
        stored: usize,
    },
    /// Periodic lift metadata exists without a pcurve branch to lift.
    #[error("coedge {coedge:?} carries periodic winding without a pcurve")]
    WindingWithoutPcurve {
        /// Invalid coedge use.
        coedge: crate::coedge::CoedgeId,
    },
}

impl remus_math::diagnostic::ToDiagnostic for BoundaryAuthorityError {
    fn diagnostic(&self) -> remus_math::diagnostic::Diagnostic {
        use remus_math::diagnostic::{Diagnostic, FailureCategory};

        match self {
            Self::Topology(error) => error.diagnostic(),
            Self::LoopOwnershipInvalid { loop_id, owners } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "loop_ownership_invalid",
                self.to_string(),
            )
            .with_detail("loop", loop_id.index())
            .with_detail("owners", *owners),
            Self::CoedgeOwnershipInvalid { coedge, owners } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "coedge_ownership_invalid",
                self.to_string(),
            )
            .with_detail("coedge", coedge.index())
            .with_detail("owners", *owners),
            Self::DuplicateOrientedUse {
                edge,
                face,
                orientation,
            } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "boundary_use_ambiguous",
                self.to_string(),
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("orientation", *orientation),
            Self::IndexMismatch {
                edge,
                face,
                coedge,
                orientation,
            } => Diagnostic::new(
                FailureCategory::Internal,
                "coedge_index_mismatch",
                self.to_string(),
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("coedge", coedge.index())
            .with_detail("orientation", *orientation),
            Self::IncompleteSeamPcurves {
                edge,
                face,
                uses,
                stored,
            } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "seam_pcurve_incomplete",
                self.to_string(),
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("uses", *uses)
            .with_detail("stored", *stored),
            Self::WindingWithoutPcurve { coedge } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "coedge_winding_without_pcurve",
                self.to_string(),
            )
            .with_detail("coedge", coedge.index()),
        }
    }
}

/// Validates every live Loop/Coedge authority record in a topology.
///
/// Every face must own a synchronized, connected loop set; every live loop
/// and coedge must have exactly one matching owner; and the compatibility
/// `(edge, face, orientation)` index must resolve each use back to its coedge.
/// A repeated edge on one face is a seam: it must have one forward and one
/// reverse use, and it may carry either zero pcurves (an honest unsupported
/// capability) or both branches. A half-populated seam is invalid.
///
/// # Errors
///
/// Returns [`BoundaryAuthorityError`] for stale references, facade
/// divergence, disconnected loops, invalid ownership, ambiguous oriented
/// uses, stale compatibility indexing, incomplete seam branches, or winding
/// metadata without a pcurve.
pub fn validate_boundary_authority(
    topo: &Topology,
) -> Result<BoundaryAuthoritySummary, BoundaryAuthorityError> {
    let mut summary = BoundaryAuthoritySummary::default();
    let mut loop_owners = HashMap::new();
    let mut coedge_owners = HashMap::new();

    for (face_id, face) in topo.faces().iter() {
        summary.faces += 1;
        validate_face_loops(topo, face_id)?;
        let mut face_uses: HashMap<crate::edge::EdgeId, Vec<_>> = HashMap::new();

        for &loop_id in face.boundary_loops() {
            *loop_owners.entry(loop_id).or_insert(0usize) += 1;
            validate_loop_connected(topo, loop_id)?;
            let boundary_loop = topo.face_loop(loop_id)?;
            for &coedge_id in boundary_loop.coedges() {
                *coedge_owners.entry(coedge_id).or_insert(0usize) += 1;
                let coedge = topo.coedge(coedge_id)?;
                topo.edge(coedge.edge())?;
                if coedge.pcurve().is_none()
                    && coedge.periodic_winding() != crate::coedge::PeriodicWinding::ZERO
                {
                    return Err(BoundaryAuthorityError::WindingWithoutPcurve { coedge: coedge_id });
                }
                face_uses.entry(coedge.edge()).or_default().push((
                    coedge_id,
                    coedge.is_forward(),
                    coedge.pcurve().is_some(),
                ));
            }
        }

        for (edge, uses) in face_uses {
            if uses.len() > 2 {
                return Err(TopologyError::NonManifold {
                    reason: format!(
                        "edge {edge:?} is used {} times by face {face_id:?}",
                        uses.len()
                    ),
                }
                .into());
            }
            if uses.len() == 2 && uses[0].1 == uses[1].1 {
                return Err(BoundaryAuthorityError::DuplicateOrientedUse {
                    edge,
                    face: face_id,
                    orientation: orientation_label(uses[0].1),
                });
            }
            for &(coedge, forward, _) in &uses {
                if topo.indexed_coedge_use(edge, face_id, forward) != Some(coedge) {
                    return Err(BoundaryAuthorityError::IndexMismatch {
                        edge,
                        face: face_id,
                        coedge,
                        orientation: orientation_label(forward),
                    });
                }
            }
            if uses.len() == 1 {
                continue;
            }
            summary.seam_edges += 1;
            let stored = uses.iter().filter(|(_, _, stored)| *stored).count();
            if stored != 0 && stored != uses.len() {
                return Err(BoundaryAuthorityError::IncompleteSeamPcurves {
                    edge,
                    face: face_id,
                    uses: uses.len(),
                    stored,
                });
            }
            summary.stored_seam_branches += stored;
        }
    }

    for loop_id in topo.live_loop_ids() {
        let owners = loop_owners.get(&loop_id).copied().unwrap_or(0);
        if owners != 1 {
            return Err(BoundaryAuthorityError::LoopOwnershipInvalid { loop_id, owners });
        }
        summary.loops += 1;
    }
    for coedge_id in topo.live_coedge_ids() {
        let owners = coedge_owners.get(&coedge_id).copied().unwrap_or(0);
        if owners != 1 {
            return Err(BoundaryAuthorityError::CoedgeOwnershipInvalid {
                coedge: coedge_id,
                owners,
            });
        }
        summary.coedges += 1;
    }

    Ok(summary)
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
    let (t0, t1) = edge
        .trim()
        .unwrap_or_else(|| edge.curve().reconstruct_domain_from_endpoints(start, end));
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
/// the measured deviation exceeds the larger of `tolerance` and the edge's
/// effective entity tolerance.
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
    let effective_tolerance = effective_edge_validation_tolerance(topo, edge_id, tolerance)?;
    if let Some(report) = check_same_parameter_strict(topo, edge_id, face_id, forward, samples)?
        && report.max_deviation > effective_tolerance
    {
        return Err(TopologyError::SameParameterExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation: report.max_deviation,
            at_parameter: report.at_parameter,
            tolerance: effective_tolerance,
        }
        .into());
    }
    Ok(())
}

/// Enforces `SameParameter` within the larger of `tolerance` and the edge's
/// effective entity tolerance.
///
/// Not-applicable configurations (no pcurve, planar face) pass vacuously.
///
/// # Errors
///
/// Returns [`TopologyError::SameParameterExceeded`] when the sampled
/// deviation exceeds the effective bound, or a not-found error for stale
/// entities.
pub fn validate_same_parameter(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    tolerance: f64,
    samples: usize,
) -> Result<(), TopologyError> {
    if !tolerance.is_finite() || tolerance.is_sign_negative() {
        return Err(TopologyError::InvalidToleranceValue {
            entity: "same-parameter validation",
            value: tolerance,
        });
    }
    let effective_tolerance = effective_edge_validation_tolerance(topo, edge_id, tolerance)?;
    if let Some(report) = check_same_parameter(topo, edge_id, face_id, forward, samples)?
        && report.max_deviation > effective_tolerance
    {
        return Err(TopologyError::SameParameterExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation: report.max_deviation,
            at_parameter: report.at_parameter,
            tolerance: effective_tolerance,
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
/// the measured deviation exceeds the larger of `tolerance` and the edge's
/// effective entity tolerance.
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
    let effective_tolerance = effective_edge_validation_tolerance(topo, edge_id, tolerance)?;
    if let Some(max_deviation) = check_same_range_strict(topo, edge_id, face_id, forward)?
        && max_deviation > effective_tolerance
    {
        return Err(TopologyError::SameRangeExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation,
            tolerance: effective_tolerance,
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
        validate_face_loops(topo, face_id)?;
        for &loop_id in face.boundary_loops() {
            for &coedge_id in topo.face_loop(loop_id)?.coedges() {
                let coedge = topo.coedge(coedge_id)?;
                summary.boundary_uses += 1;
                let edge_id = coedge.edge();
                let forward = coedge.is_forward();
                topo.edge(edge_id)?;
                if coedge.pcurve().is_none() {
                    continue;
                }
                summary.stored_pcurves += 1;
                let parameter =
                    check_same_parameter_strict(topo, edge_id, face_id, forward, samples)?;
                let range = check_same_range_strict(topo, edge_id, face_id, forward)?;
                let effective_tolerance =
                    effective_edge_validation_tolerance(topo, edge_id, tolerance)?;
                if let Some(report) = parameter
                    && report.max_deviation > effective_tolerance
                {
                    return Err(TopologyError::SameParameterExceeded {
                        edge: edge_id,
                        face: face_id,
                        max_deviation: report.max_deviation,
                        at_parameter: report.at_parameter,
                        tolerance: effective_tolerance,
                    }
                    .into());
                }
                if let Some(max_deviation) = range
                    && max_deviation > effective_tolerance
                {
                    return Err(TopologyError::SameRangeExceeded {
                        edge: edge_id,
                        face: face_id,
                        max_deviation,
                        tolerance: effective_tolerance,
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

/// Enforces `SameRange` within the larger of `tolerance` and the edge's
/// effective entity tolerance.
///
/// # Errors
///
/// Returns [`TopologyError::SameRangeExceeded`] when either endpoint
/// deviation exceeds the effective bound, or a not-found error for stale
/// entities.
pub fn validate_same_range(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    tolerance: f64,
) -> Result<(), TopologyError> {
    if !tolerance.is_finite() || tolerance.is_sign_negative() {
        return Err(TopologyError::InvalidToleranceValue {
            entity: "same-range validation",
            value: tolerance,
        });
    }
    let effective_tolerance = effective_edge_validation_tolerance(topo, edge_id, tolerance)?;
    if let Some(max_deviation) = check_same_range(topo, edge_id, face_id, forward)?
        && max_deviation > effective_tolerance
    {
        return Err(TopologyError::SameRangeExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation,
            tolerance: effective_tolerance,
        });
    }
    Ok(())
}

fn effective_edge_validation_tolerance(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    caller_tolerance: f64,
) -> Result<f64, TopologyError> {
    let edge = topo.edge(edge_id)?;
    let start_tolerance = topo.vertex(edge.start())?.tolerance();
    let end_tolerance = topo.vertex(edge.end())?.tolerance();
    for value in [start_tolerance, end_tolerance] {
        if !value.is_finite() || value.is_sign_negative() {
            return Err(TopologyError::InvalidToleranceValue {
                entity: "vertex",
                value,
            });
        }
    }
    let vertex_tolerance = start_tolerance.max(end_tolerance);
    let entity_tolerance = edge.effective_tolerance(vertex_tolerance);
    if !entity_tolerance.is_finite() || entity_tolerance.is_sign_negative() {
        return Err(TopologyError::InvalidToleranceValue {
            entity: "edge",
            value: entity_tolerance,
        });
    }
    Ok(caller_tolerance.max(entity_tolerance))
}

// ── Entity-tolerance checks (RFC 0004, Stage 1) ─────────────────────────
//
// Two validator-enforced containment invariants, in the existing
// `tolerance_violation` diagnostic family:
//
// 1. **Ball containment** — every incident edge end's curve evaluation lies
//    within its vertex's ball (`vertex_ball_violation`).
// 2. **Tube containment** — every stored pcurve use's sampled 3D↔p-curve
//    deviation is within the edge's effective tolerance
//    (`edge_tube_violation`), measured with the same machinery as
//    `check_same_parameter`/`check_same_range` rather than duplicated.
//
// Both checks derive their bound from **entity tolerance** from the start
// (the vertex ball / the edge's effective tolerance), unlike
// `validate_same_parameter`/`validate_same_range`, whose bound stays
// caller-supplied this stage and flips to the entity-derived bound in a
// later stage. Both pass vacuously at default tolerances on exact geometry.

/// One incident edge end's distance from the vertex's tolerance ball.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VertexBallReport {
    /// The incident edge whose end was measured.
    pub edge: crate::edge::EdgeId,
    /// Whether the measured end is the edge's start (`true`) or end (`false`).
    pub at_start: bool,
    /// Distance from the curve's endpoint evaluation to the vertex point,
    /// in model units. Non-finite curve evaluations report [`f64::MAX`].
    pub deviation: f64,
}

/// Measures the ball-containment invariant (RFC 0004, invariant 1) for one
/// vertex.
///
/// Every incident edge end's curve evaluation, at that end's parameter
/// under the edge's domain, must lie within the vertex's ball —
/// `|curve(t_end) − vertex.point()| ≤ vertex.tolerance()`.
///
/// The vertex's ball is checked as **claimed** (no floor clamp): the
/// validator's job is to prove the claim, and a ball that does not cover
/// its measured gap is exactly what a later raise must fix. A vertex with
/// no incident edges passes vacuously.
///
/// # Errors
///
/// Returns a not-found error when the vertex or an incident edge's
/// vertices are stale. An incident edge without a valid authoritative
/// parameter range reports [`f64::MAX`], so validation fails closed without
/// changing this compatibility API's error type.
pub fn check_vertex_ball(
    topo: &Topology,
    vertex_id: crate::vertex::VertexId,
) -> Result<Vec<VertexBallReport>, TopologyError> {
    let point = topo.vertex(vertex_id)?.point();
    let mut reports = Vec::new();
    for (edge_id, edge) in topo.edges().iter() {
        let at_start = edge.start() == vertex_id;
        if !at_start && edge.end() != vertex_id {
            continue;
        }
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let deviation = if let Ok((d0, d1)) = edge.strict_domain() {
            let t_end = if at_start { d0 } else { d1 };
            let on_curve = edge.curve().evaluate_with_endpoints(t_end, start, end);
            if t_end.is_finite() && on_curve.0.iter().all(|value| value.is_finite()) {
                (on_curve - point).length()
            } else {
                f64::MAX
            }
        } else {
            f64::MAX
        };
        reports.push(VertexBallReport {
            edge: edge_id,
            at_start,
            deviation,
        });
    }
    reports.sort_by_key(|report| (report.edge.index(), report.at_start));
    Ok(reports)
}

/// Enforces the ball-containment invariant (RFC 0004, invariant 1).
///
/// Every incident edge end's curve evaluation must lie within the vertex's
/// ball as claimed. A violation is a claim the stored tolerance cannot
/// cover — the honest fix is a validated raise
/// ([`Vertex::set_tolerance`](crate::vertex::Vertex::set_tolerance)), not a
/// silent widening.
///
/// Vacuously passes when every incident edge end's curve evaluation is
/// exactly on the vertex point, as for primitives and line edges.
///
/// # Errors
///
/// Returns [`TopologyError::VertexBallExceeded`] for the first (in arena
/// edge order) incident edge end whose curve evaluation lies outside the
/// ball, or a not-found error for stale entities.
pub fn validate_vertex_ball(
    topo: &Topology,
    vertex_id: crate::vertex::VertexId,
) -> Result<(), TopologyError> {
    let ball = topo.vertex(vertex_id)?.tolerance();
    for report in check_vertex_ball(topo, vertex_id)? {
        if report.deviation > ball {
            return Err(TopologyError::VertexBallExceeded {
                vertex: vertex_id,
                edge: report.edge,
                deviation: report.deviation,
                tolerance: ball,
            });
        }
    }
    Ok(())
}

/// Result of the edge-tube containment check (RFC 0004, invariant 2).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeTubeReport {
    /// Largest measured 3D↔p-curve deviation: the larger of the
    /// `SameParameter` interior deviation and the `SameRange` endpoint
    /// deviation, in model units.
    pub max_deviation: f64,
    /// Witness parameter of the SameParameter deviation.
    pub at_parameter: f64,
    /// Evaluation points used for the sampled measurement.
    pub samples: usize,
    /// The deviation from the SameParameter measurement alone.
    pub parameter_deviation: f64,
    /// The endpoint deviation from the SameRange measurement, when it
    /// applies (`0.0` otherwise).
    pub range_deviation: f64,
    /// The effective tube tolerance this use claims: the edge's declared
    /// tolerance, falling back to the wider of its bounding vertices' balls,
    /// clamped below by the global floor (RFC 0004, floor rule) — an entity
    /// tolerance only widens bands, never narrows them below the floor.
    pub effective_tolerance: f64,
}

/// Measures the edge-tube containment invariant (RFC 0004, invariant 2)
/// for one oriented edge use with a stored pcurve.
///
/// Reuses the [`check_same_parameter`] and [`check_same_range`]
/// measurements rather than duplicating them: the reported deviation is the
/// larger of the two, and the effective tube bound is
/// `max(global floor, edge.effective_tolerance(max(ball_start, ball_end)))`
/// — the entity-tolerance rule with the global default as the passed
/// tolerance. Returns `Ok(None)` when the check does not apply (no stored
/// pcurve for that use).
///
/// Unlike the strict proof path, this uses the sampled
/// [`check_same_parameter`]/[`check_same_range`] measurements, so a use
/// whose proof combination is unsupported is measured by sampling (and
/// reported at its measured deviation) rather than refused.
///
/// # Errors
///
/// Returns a not-found error when the edge, face, or a bounding vertex is
/// stale.
pub fn check_edge_tube(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    samples: usize,
) -> Result<Option<EdgeTubeReport>, TopologyError> {
    let edge = topo.edge(edge_id)?;
    let ball_start = topo.vertex(edge.start())?.tolerance();
    let ball_end = topo.vertex(edge.end())?.tolerance();
    let vertex_tol = ball_start.max(ball_end);
    let effective = edge
        .effective_tolerance(vertex_tol)
        .max(Tolerance::new().linear);

    let Some(parameter) = check_same_parameter(topo, edge_id, face_id, forward, samples)? else {
        return Ok(None);
    };
    let range_deviation = check_same_range(topo, edge_id, face_id, forward)?.unwrap_or(0.0);
    let max_deviation = parameter.max_deviation.max(range_deviation);
    Ok(Some(EdgeTubeReport {
        max_deviation,
        at_parameter: parameter.at_parameter,
        samples: parameter.samples,
        parameter_deviation: parameter.max_deviation,
        range_deviation,
        effective_tolerance: effective,
    }))
}

/// Enforces the edge-tube containment invariant (RFC 0004, invariant 2):
/// the use's sampled 3D↔p-curve deviation must lie within the edge's
/// effective tolerance.
///
/// The bound is entity-derived from the start:
/// `max(global floor, edge.effective_tolerance(max(ball_start, ball_end)))` —
/// the edge's declared tolerance when present, its bounding vertices' balls
/// otherwise, never below the global floor. This is the RFC's
/// `validate_same_parameter` bound rule with the global default as the
/// caller-supplied tolerance; the existing [`validate_same_parameter`] keeps
/// its caller-supplied bound unchanged this stage.
///
/// Not-applicable configurations (no stored pcurve, planar face) pass
/// vacuously, exactly like [`validate_same_parameter`].
///
/// # Errors
///
/// Returns [`TopologyError::EdgeTubeExceeded`] when the measured deviation
/// exceeds the effective tolerance, or a not-found error for stale
/// entities.
pub fn validate_edge_tube(
    topo: &Topology,
    edge_id: crate::edge::EdgeId,
    face_id: crate::face::FaceId,
    forward: bool,
    samples: usize,
) -> Result<(), TopologyError> {
    let Some(report) = check_edge_tube(topo, edge_id, face_id, forward, samples)? else {
        return Ok(());
    };
    if report.max_deviation > report.effective_tolerance {
        return Err(TopologyError::EdgeTubeExceeded {
            edge: edge_id,
            face: face_id,
            max_deviation: report.max_deviation,
            at_parameter: report.at_parameter,
            tolerance: report.effective_tolerance,
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
    fn whole_topology_boundary_gate_rejects_duplicate_oriented_uses() {
        use remus_math::diagnostic::ToDiagnostic;

        let mut topo = Topology::new();
        let vertex = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(vertex, vertex, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(edge, true), OrientedEdge::new(edge, true)],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));

        let error = validate_boundary_authority(&topo).unwrap_err();
        assert!(matches!(
            error,
            BoundaryAuthorityError::DuplicateOrientedUse {
                edge: found_edge,
                face: found_face,
                orientation: "forward",
            } if found_edge == edge && found_face == face
        ));
        assert_eq!(error.diagnostic().code(), "boundary_use_ambiguous");
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

    /// Plane surface used by the loop/wire divergence fixtures.
    fn unit_plane() -> FaceSurface {
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        }
    }

    #[test]
    fn wire_closure_rejects_an_open_chain_and_a_broken_ring() {
        use crate::vertex::Vertex;

        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        // Far enough that no position fallback can bridge the gap.
        let far = topo.add_vertex(Vertex::new(Point3::new(5.0, 5.0, 5.0), 1e-7));

        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        // Same ring, but its second edge starts a long way from the first's end.
        let gap = topo.add_edge(Edge::new(far, v2, EdgeCurve::Line));

        let open_ring = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e2, true),
                ],
                false,
            )
            .unwrap(),
        );
        let broken_ring = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(gap, true),
                    OrientedEdge::new(e2, true),
                ],
                true,
            )
            .unwrap(),
        );
        // The wrap-around alone is broken: consecutive pairs all connect.
        let unclosed_ring = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
                true,
            )
            .unwrap(),
        );

        for wid in [open_ring, broken_ring, unclosed_ring] {
            let wire = topo.wire(wid).unwrap();
            let err = validate_wire_closed(wire, &topo).unwrap_err();
            assert!(
                matches!(err, TopologyError::WireNotClosed),
                "expected WireNotClosed, got {err:?}"
            );
        }
    }

    #[test]
    fn wire_closure_accepts_coincident_but_distinct_endpoint_vertices() {
        use crate::vertex::Vertex;

        // Every junction is chained through a *distinct* vertex id at the
        // same position, so closure can only be proved by the coincident
        // position fallback, never by id equality.
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let starts: Vec<_> = corners
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let ends: Vec<_> = corners
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        assert_ne!(starts[1], ends[1], "the fixture needs distinct ids");

        let edges: Vec<_> = (0..3)
            .map(|i| topo.add_edge(Edge::new(starts[i], ends[(i + 1) % 3], EdgeCurve::Line)))
            .collect();
        let wid = topo.add_wire(
            Wire::new(
                edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );

        let wire = topo.wire(wid).unwrap();
        validate_wire_closed(wire, &topo).unwrap();
    }

    #[test]
    fn closed_shell_report_names_free_and_over_shared_edges() {
        // One triangular face: every edge is used once.
        let mut topo = Topology::new();
        let wid = make_triangle(&mut topo);
        let open_face = topo.add_face(Face::new(wid, vec![], unit_plane()));
        let open_shell = Shell::new(vec![open_face]).unwrap();
        let err = validate_shell_closed(&open_shell, &topo).unwrap_err();
        let TopologyError::NonManifold { reason } = err else {
            unreachable!("expected NonManifold, got {err:?}")
        };
        assert!(
            reason.contains("free edge"),
            "a once-used edge is a free edge, got {reason}"
        );

        // Three faces over the same ring: every edge is used three times.
        let back: Vec<OrientedEdge> = topo
            .wire(wid)
            .unwrap()
            .edges()
            .iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
        let back_wid = topo.add_wire(Wire::new(back.clone(), true).unwrap());
        let third_wid = topo.add_wire(Wire::new(back, true).unwrap());
        let f1 = topo.add_face(Face::new(back_wid, vec![], unit_plane()));
        let f2 = topo.add_face(Face::new(third_wid, vec![], unit_plane()));
        let crowded = Shell::new(vec![open_face, f1, f2]).unwrap();
        let err = validate_shell_closed(&crowded, &topo).unwrap_err();
        let TopologyError::NonManifold { reason } = err else {
            unreachable!("expected NonManifold, got {err:?}")
        };
        assert!(
            reason.contains("over-shared"),
            "a thrice-used edge is over-shared, got {reason}"
        );
    }

    /// A face over a closed two-edge ring, plus the ring's edges.
    fn two_gon_face(
        topo: &mut Topology,
    ) -> (
        crate::face::FaceId,
        crate::edge::EdgeId,
        crate::edge::EdgeId,
    ) {
        use crate::vertex::Vertex;

        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v0, EdgeCurve::Line));
        let wid = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(wid, vec![], unit_plane()));
        (face, e0, e1)
    }

    #[test]
    #[allow(deprecated)] // the divergence under test needs the unsynchronized setter
    fn face_loops_reject_a_diverging_closure_flag() {
        let mut topo = Topology::new();
        let (face, e0, e1) = two_gon_face(&mut topo);
        validate_face_loops(&topo, face).unwrap();

        // Same edges, same order, same count — only the closure flag moves.
        let open_copy = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
                false,
            )
            .unwrap(),
        );
        topo.face_mut(face).unwrap().set_outer_wire(open_copy);
        let err = validate_face_loops(&topo, face).unwrap_err();
        assert!(
            matches!(err, TopologyError::LoopWireMismatch { face: f } if f == face),
            "expected LoopWireMismatch, got {err:?}"
        );
    }

    #[test]
    #[allow(deprecated)] // the divergence under test needs the unsynchronized setter
    fn face_loops_reject_a_diverging_use_count() {
        let mut topo = Topology::new();
        let (face, e0, e1) = two_gon_face(&mut topo);
        validate_face_loops(&topo, face).unwrap();

        // Same closure flag, same leading edges — only the count moves.
        let longer = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e0, false),
                ],
                true,
            )
            .unwrap(),
        );
        topo.face_mut(face).unwrap().set_outer_wire(longer);
        let err = validate_face_loops(&topo, face).unwrap_err();
        assert!(
            matches!(err, TopologyError::LoopWireMismatch { face: f } if f == face),
            "expected LoopWireMismatch, got {err:?}"
        );
    }

    #[test]
    fn face_loops_reject_a_flipped_compatibility_orientation() {
        let mut topo = Topology::new();
        let (face, e0, _) = two_gon_face(&mut topo);
        validate_face_loops(&topo, face).unwrap();

        // Same edge at the same position, same loop parent — only the
        // traversal direction of the compatibility wire moves.
        let wire_id = topo.face(face).unwrap().outer_wire();
        topo.wire_mut(wire_id).unwrap().edges_mut()[0] = OrientedEdge::new(e0, false);
        let err = validate_face_loops(&topo, face).unwrap_err();
        assert!(
            matches!(err, TopologyError::LoopWireMismatch { face: f } if f == face),
            "expected LoopWireMismatch, got {err:?}"
        );
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod same_parameter_tests {
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Circle2D, Curve2D, Ellipse2D, Line2D, NurbsCurve2D};
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
        BoundaryAuthorityError, CurveUseValidationError, check_same_parameter,
        check_same_parameter_strict, check_same_range, check_same_range_strict,
        validate_boundary_authority, validate_same_parameter, validate_same_parameter_strict,
        validate_same_range, validate_same_range_strict, validate_solid_pcurve_contracts,
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
        topo.set_pcurve_oriented(rim, face, true, rim_pcurve(0.0))
            .unwrap();

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
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3, true))
            .unwrap();

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
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false))
            .unwrap();

        for forward in [true, false] {
            validate_same_parameter_strict(&topo, seam, face, forward, 1e-7, 32).unwrap();
            validate_same_range_strict(&topo, seam, face, forward, 1e-7).unwrap();
        }

        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU + 0.2, false))
            .unwrap();
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
    fn whole_topology_boundary_gate_requires_all_or_no_seam_pcurves() {
        use remus_math::diagnostic::ToDiagnostic;

        let (mut topo, seam, face) = cylinder_seam();
        let empty = validate_boundary_authority(&topo).unwrap();
        assert_eq!(empty.faces, 1);
        assert_eq!(empty.loops, 1);
        assert_eq!(empty.coedges, 2);
        assert_eq!(empty.seam_edges, 1);
        assert_eq!(empty.stored_seam_branches, 0);

        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();
        let error = validate_boundary_authority(&topo).unwrap_err();
        assert!(matches!(
            error,
            BoundaryAuthorityError::IncompleteSeamPcurves {
                uses: 2,
                stored: 1,
                ..
            }
        ));
        assert_eq!(error.diagnostic().code(), "seam_pcurve_incomplete");

        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false))
            .unwrap();
        let complete = validate_boundary_authority(&topo).unwrap();
        assert_eq!(complete.stored_seam_branches, 2);
    }

    #[test]
    fn whole_topology_boundary_gate_rejects_winding_without_a_branch() {
        use remus_math::diagnostic::ToDiagnostic;

        let (mut topo, seam, _) = cylinder_seam();
        let lifted = topo
            .coedges_of_edge(seam)
            .into_iter()
            .find(|&coedge| !topo.coedge(coedge).unwrap().is_forward())
            .unwrap();
        topo.set_coedge_periodic_winding(lifted, crate::PeriodicWinding::new(1, 0))
            .unwrap();

        let error = validate_boundary_authority(&topo).unwrap_err();
        assert!(matches!(
            error,
            BoundaryAuthorityError::WindingWithoutPcurve { coedge } if coedge == lifted
        ));
        assert_eq!(error.diagnostic().code(), "coedge_winding_without_pcurve");
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
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();

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
        )
        .unwrap();

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
        )
        .unwrap();

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
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();

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
        )
        .unwrap();

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
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false))
            .unwrap();
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        let summary = validate_solid_pcurve_contracts(&topo, solid, 1e-7, 32).unwrap();
        assert_eq!(summary.boundary_uses, 2);
        assert_eq!(summary.stored_pcurves, 2);
        assert_eq!(summary.validated_uses, 2);

        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU + 0.2, false))
            .unwrap();
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
        )
        .unwrap();

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
        )
        .unwrap();
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
        topo.set_pcurve_oriented(rim, face, true, rim_pcurve(0.0))
            .unwrap();
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

    // ── Typed refusal payloads ──────────────────────────────────────────

    fn stored_seam_pcurve(curve: Curve2D) -> (Topology, EdgeId, FaceId) {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, PCurve::new(curve, 0.0, 1.0))
            .unwrap();
        (topo, seam, face)
    }

    fn proof_refusal_pcurve_type(curve: Curve2D) -> &'static str {
        let (topo, seam, face) = stored_seam_pcurve(curve);
        let error = check_same_parameter_strict(&topo, seam, face, true, 8).unwrap_err();
        let CurveUseValidationError::SameParameterProofUnavailable { pcurve_type, .. } = error
        else {
            unreachable!("expected a typed proof refusal, got {error:?}")
        };
        pcurve_type
    }

    #[test]
    fn proof_refusal_names_the_stored_pcurve_type() {
        // A slanted line on the cylinder is not the certified vertical-line
        // case, so each stored kind reaches the refusal and must name itself.
        assert_eq!(
            proof_refusal_pcurve_type(Curve2D::Line(
                Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.6, 0.8)).unwrap()
            )),
            "line"
        );
        assert_eq!(
            proof_refusal_pcurve_type(Curve2D::Circle(
                Circle2D::new(Point2::new(0.0, 0.0), 1.0).unwrap()
            )),
            "circle"
        );
        assert_eq!(
            proof_refusal_pcurve_type(Curve2D::Ellipse(
                Ellipse2D::new(Point2::new(0.0, 0.0), 1.0, 0.5, 0.0).unwrap()
            )),
            "ellipse"
        );
        assert_eq!(
            proof_refusal_pcurve_type(Curve2D::Nurbs(
                NurbsCurve2D::new(
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![Point2::new(0.0, 0.0), Point2::new(0.5, 1.0)],
                    vec![1.0; 2],
                )
                .unwrap()
            )),
            "nurbs"
        );
    }

    fn non_finite_component(curve: Curve2D) -> &'static str {
        let (topo, seam, face) = stored_seam_pcurve(curve);
        let error = check_same_parameter_strict(&topo, seam, face, true, 8).unwrap_err();
        let CurveUseValidationError::NonFinitePcurveUse { component, .. } = error else {
            unreachable!("expected a non-finite refusal, got {error:?}")
        };
        component
    }

    #[test]
    fn non_finite_pcurve_definitions_are_caught_before_evaluation() {
        // One poisoned sub-field per curve kind: the definition scan must
        // catch each of them, and name the definition — not a downstream
        // evaluation — as the failing component.
        assert_eq!(
            non_finite_component(Curve2D::Line(
                Line2D::new(Point2::new(f64::NAN, 0.0), Vec2::new(0.0, 1.0)).unwrap()
            )),
            "curve_definition",
            "a line's origin is part of its definition"
        );
        assert_eq!(
            non_finite_component(Curve2D::Circle(
                Circle2D::new(Point2::new(f64::NAN, 0.0), 1.0).unwrap()
            )),
            "curve_definition",
            "a circle's centre is part of its definition"
        );
        assert_eq!(
            non_finite_component(Curve2D::Ellipse(
                Ellipse2D::new(Point2::new(f64::NAN, 0.0), 1.0, 0.5, 0.0).unwrap()
            )),
            "curve_definition",
            "an ellipse's centre is part of its definition"
        );
        assert_eq!(
            non_finite_component(Curve2D::Nurbs(
                NurbsCurve2D::new(
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![Point2::new(0.0, 0.0), Point2::new(0.0, 1.0)],
                    vec![1.0, f64::NAN],
                )
                .unwrap()
            )),
            "curve_definition",
            "a NURBS weight is part of its definition"
        );
    }

    #[test]
    fn non_finite_parameter_bounds_are_caught_before_evaluation() {
        // Only the END bound is poisoned: a guard that demanded *both*
        // bounds be non-finite would fall through to the evaluation check.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(
            seam,
            face,
            true,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
                0.0,
                f64::NAN,
            ),
        )
        .unwrap();

        for error in [
            check_same_parameter_strict(&topo, seam, face, true, 8).unwrap_err(),
            check_same_range_strict(&topo, seam, face, true).unwrap_err(),
        ] {
            let CurveUseValidationError::NonFinitePcurveUse { component, .. } = error else {
                unreachable!("expected a non-finite refusal, got {error:?}")
            };
            assert_eq!(component, "parameter_bounds");
        }
    }

    /// A pcurve whose definition is finite but whose evaluation overflows at
    /// its START bound only (a huge homogeneous weight at the first control
    /// point); the end bound evaluates cleanly.
    fn one_sided_overflowing_pcurve() -> PCurve {
        PCurve::new(
            Curve2D::Nurbs(
                NurbsCurve2D::new(
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![Point2::new(1e10, 0.0), Point2::new(0.0, 1.0)],
                    vec![1e308, 1.0],
                )
                .unwrap(),
            ),
            0.0,
            1.0,
        )
    }

    #[test]
    fn a_single_non_finite_pcurve_endpoint_is_caught_before_the_surface() {
        let pcurve = one_sided_overflowing_pcurve();
        assert!(
            !pcurve.evaluate(0.0).0.iter().all(|v| v.is_finite()),
            "fixture: the start bound must evaluate non-finite"
        );
        assert!(
            pcurve.evaluate(1.0).0.iter().all(|v| v.is_finite()),
            "fixture: the end bound must evaluate finite"
        );

        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, pcurve).unwrap();

        for error in [
            check_same_parameter_strict(&topo, seam, face, true, 8).unwrap_err(),
            check_same_range_strict(&topo, seam, face, true).unwrap_err(),
        ] {
            let CurveUseValidationError::NonFinitePcurveUse { component, .. } = error else {
                unreachable!("expected a non-finite refusal, got {error:?}")
            };
            assert_eq!(component, "pcurve_evaluation");
        }
    }

    #[test]
    fn a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof() {
        // The surface images stay finite; only the oriented START vertex is
        // poisoned, so exactly one of the two endpoint distances is NaN.
        let (mut topo, seam, face) = cylinder_seam();
        let bottom = topo.edge(seam).unwrap().start();
        topo.vertex_mut(bottom)
            .unwrap()
            .set_point(Point3::new(f64::NAN, 0.0, 0.0));
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();

        let error = check_same_parameter_strict(&topo, seam, face, true, 8).unwrap_err();
        let CurveUseValidationError::NonFinitePcurveUse { component, .. } = error else {
            unreachable!("expected a non-finite refusal, got {error:?}")
        };
        assert_eq!(component, "surface_or_curve_evaluation");

        // The same one-sided poisoning must fail both range checks closed.
        // `f64::max` swallows a NaN operand, so a guard that missed the
        // poisoned endpoint would report the *clean* endpoint's distance
        // as the whole measurement.
        assert_eq!(
            check_same_range(&topo, seam, face, true).unwrap(),
            Some(f64::MAX)
        );
        let error = check_same_range_strict(&topo, seam, face, true).unwrap_err();
        let CurveUseValidationError::NonFinitePcurveUse { component, .. } = error else {
            unreachable!("expected a non-finite refusal, got {error:?}")
        };
        assert_eq!(component, "surface_or_endpoint_evaluation");
    }

    /// A cylinder so large that its surface point at u = 0 overflows while
    /// its point at u = π/2 stays finite, with a horizontal pcurve joining
    /// the two and finite bounding vertices.
    fn overflowing_surface_use() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let cylinder = CylindricalSurface::with_ref_dir(
            Point3::new(1e308, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1e308,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
        topo.set_pcurve_oriented(
            edge,
            face,
            true,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
                0.0,
                std::f64::consts::FRAC_PI_2,
            ),
        )
        .unwrap();
        (topo, edge, face)
    }

    #[test]
    fn a_single_non_finite_surface_endpoint_fails_the_range_checks_closed() {
        let (topo, edge, face) = overflowing_surface_use();
        let surface = topo.face(face).unwrap().surface().clone();
        assert!(
            !surface
                .evaluate(0.0, 0.0)
                .unwrap()
                .0
                .iter()
                .all(|v| v.is_finite()),
            "fixture: the start image must overflow"
        );
        assert!(
            surface
                .evaluate(std::f64::consts::FRAC_PI_2, 0.0)
                .unwrap()
                .0
                .iter()
                .all(|v| v.is_finite()),
            "fixture: the end image must stay finite"
        );

        assert_eq!(
            check_same_range(&topo, edge, face, true).unwrap(),
            Some(f64::MAX),
            "the sampled check reports its fail-closed sentinel, not an overflowed distance"
        );

        let error = check_same_range_strict(&topo, edge, face, true).unwrap_err();
        let CurveUseValidationError::NonFinitePcurveUse { component, .. } = error else {
            unreachable!("expected a non-finite refusal, got {error:?}")
        };
        assert_eq!(component, "surface_or_endpoint_evaluation");
    }

    #[test]
    fn a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines() {
        // A planar face has no UV evaluation, so a guard that let the
        // non-finite uv through would report "not applicable" instead of
        // the fail-closed sentinel.
        let (mut topo, seam, face) = cylinder_seam();
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            });
        topo.set_pcurve_oriented(seam, face, true, one_sided_overflowing_pcurve())
            .unwrap();

        assert_eq!(
            check_same_range(&topo, seam, face, true).unwrap(),
            Some(f64::MAX)
        );
        let report = check_same_parameter(&topo, seam, face, true, 8)
            .unwrap()
            .expect("a non-finite uv is measured, not declined");
        assert_eq!(report.max_deviation, f64::MAX);
        assert_eq!(report.at_parameter, 0.0);
    }

    #[test]
    fn an_overflowing_sampled_deviation_reports_the_fail_closed_sentinel() {
        // Both evaluations are finite; only their difference overflows.
        let mut topo = Topology::new();
        let cylinder = CylindricalSurface::with_ref_dir(
            Point3::new(1e308, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(-1e308, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(-1e308, 0.0, 1.0), 1e-7));
        let edge = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
        topo.set_pcurve_oriented(edge, face, true, seam_pcurve(0.0, true))
            .unwrap();

        let surface = topo.face(face).unwrap().surface().clone();
        assert!(
            surface
                .evaluate(0.0, 0.0)
                .unwrap()
                .0
                .iter()
                .all(|v| v.is_finite()),
            "fixture: the surface image itself must stay finite"
        );

        let report = check_same_parameter(&topo, edge, face, true, 4)
            .unwrap()
            .unwrap();
        assert_eq!(report.max_deviation, f64::MAX);
    }

    // ── Sampled SameParameter: the parameter maps ───────────────────────

    /// A unit-cylinder seam whose stored pcurve deliberately disagrees with
    /// the edge, with a pcurve range (`[0.6, 1.8]`) and an edge domain
    /// (`[1.0, 0.4]`) that share neither an origin nor a span — so every
    /// term of both parameter maps is observable.
    fn mismatched_parameter_maps(forward: bool) -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let cylinder = CylindricalSurface::with_ref_dir(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let bottom = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let top = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 1.0), 1e-7));
        let mut edge = Edge::new(bottom, top, EdgeCurve::Line);
        edge.set_trim(Some((1.0, 0.4)));
        let edge_id = topo.add_edge(edge);
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(edge_id, true),
                    OrientedEdge::new(edge_id, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
        topo.set_pcurve_oriented(
            edge_id,
            face,
            forward,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 2.0), Vec2::new(1.0, 0.0)).unwrap()),
                0.6,
                1.8,
            ),
        )
        .unwrap();
        (topo, edge_id, face)
    }

    /// Closed form of the fixture's deviation: the surface point is
    /// `(cos u, sin u, 2)` and the line's point is `(1, 0, g)`.
    fn mismatch_deviation(u: f64, g: f64) -> f64 {
        (2.0 - 2.0 * u.cos() + (2.0 - g) * (2.0 - g)).sqrt()
    }

    #[test]
    fn sampled_parameter_pins_the_forward_map() {
        let (topo, edge, face) = mismatched_parameter_maps(true);
        let report = check_same_parameter(&topo, edge, face, true, 4)
            .unwrap()
            .unwrap();

        // At the last sample the pcurve parameter is its end bound 1.8 and
        // the edge parameter is its domain end 0.4.
        assert!((report.max_deviation - mismatch_deviation(1.8, 0.4)).abs() < 1e-12);
        assert!((report.at_parameter - 1.8).abs() < 1e-12);
        assert_eq!(report.samples, 5, "samples + 1 evaluation points");
    }

    #[test]
    fn sampled_parameter_pins_the_reversed_map() {
        // A reversed use walks the edge domain backwards: the pcurve's end
        // bound pairs with the edge domain's START.
        let (topo, edge, face) = mismatched_parameter_maps(false);
        let report = check_same_parameter(&topo, edge, face, false, 4)
            .unwrap()
            .unwrap();

        assert!((report.max_deviation - mismatch_deviation(1.8, 1.0)).abs() < 1e-12);
        assert!((report.at_parameter - 1.8).abs() < 1e-12);
    }

    #[test]
    fn sampled_parameter_reports_max_for_a_half_open_parameter_range() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(
            seam,
            face,
            true,
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
                2.0,
                f64::INFINITY,
            ),
        )
        .unwrap();

        let report = check_same_parameter(&topo, seam, face, true, 8)
            .unwrap()
            .unwrap();
        assert_eq!(report.max_deviation, f64::MAX);
        assert_eq!(
            report.at_parameter, 2.0,
            "the finite start bound is the witness"
        );
        assert_eq!(report.samples, 9);
    }

    #[test]
    fn sampled_parameter_witnesses_the_first_maximal_sample() {
        // A pcurve offset in u alone deviates by the same chord at every
        // sample, so the witness parameter pins which sample is kept.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3, true))
            .unwrap();

        let report = check_same_parameter(&topo, seam, face, true, 8)
            .unwrap()
            .unwrap();
        assert!((report.max_deviation - 2.0 * 0.15_f64.sin()).abs() < 1e-12);
        assert_eq!(
            report.at_parameter, 0.0,
            "the first sample attaining the maximum is the witness"
        );
    }

    // ── Strict SameParameter: the certified endpoint bound ──────────────

    #[test]
    fn strict_parameter_certifies_exactly_at_the_linear_tolerance() {
        // Scale chosen so the arithmetic bound lands exactly on the global
        // linear tolerance: the certificate is issued at the bound, and
        // refused once past it.
        let scale = 1e-7 * f64::from(1u32 << 23) * f64::from(1u32 << 23);
        let mut topo = Topology::new();
        let cylinder = CylindricalSurface::with_ref_dir(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let bottom = topo.add_vertex(Vertex::new(cylinder.evaluate(0.0, 0.0), 1e-7));
        let top = topo.add_vertex(Vertex::new(cylinder.evaluate(0.0, scale), 1e-7));
        let seam = topo.add_edge(Edge::new(bottom, top, EdgeCurve::Line));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(seam, true)], false).unwrap());
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(cylinder)));
        let vertical = |t_end: f64| {
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
                0.0,
                t_end,
            )
        };

        topo.set_pcurve_oriented(seam, face, true, vertical(scale))
            .unwrap();
        let report = check_same_parameter_strict(&topo, seam, face, true, 2)
            .unwrap()
            .expect("a bound exactly at the linear tolerance is still certified");
        assert!((report.max_deviation - 1e-7).abs() < 1e-20);

        topo.set_pcurve_oriented(seam, face, true, vertical(scale * 2.0))
            .unwrap();
        let error = check_same_parameter_strict(&topo, seam, face, true, 2).unwrap_err();
        assert!(matches!(
            error,
            CurveUseValidationError::SameParameterProofUnavailable { .. }
        ));
    }

    #[test]
    fn strict_parameter_reports_the_larger_endpoint_deviation_plus_its_bound() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();

        // Exact use: the reported deviation is the arithmetic bound alone,
        // which is strictly positive and well inside the linear tolerance.
        let exact = check_same_parameter_strict(&topo, seam, face, true, 2)
            .unwrap()
            .unwrap();
        assert!(
            exact.max_deviation > 0.0 && exact.max_deviation < 1e-7,
            "the certificate carries its own round-off allowance, got {}",
            exact.max_deviation
        );

        // Push the END vertex off the surface: the larger of the two
        // endpoint deviations, and its parameter, must be the one reported.
        let top = topo.edge(seam).unwrap().end();
        topo.vertex_mut(top)
            .unwrap()
            .set_point(Point3::new(1.01, 0.0, 1.0));
        let report = check_same_parameter_strict(&topo, seam, face, true, 2)
            .unwrap()
            .unwrap();
        assert!((report.max_deviation - 0.01).abs() < 1e-12);
        assert_eq!(report.at_parameter, 1.0, "the end bound is the witness");
    }

    // ── Tolerance guards ────────────────────────────────────────────────

    #[test]
    fn every_tolerance_guard_rejects_a_negative_and_a_nan_bound() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false))
            .unwrap();
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        for bad in [-1.0, f64::NAN] {
            let err = validate_same_parameter(&topo, seam, face, true, bad, 8).unwrap_err();
            assert!(
                matches!(
                    err,
                    TopologyError::InvalidToleranceValue {
                        entity: "same-parameter validation",
                        ..
                    }
                ),
                "got {err:?}"
            );
            let err = validate_same_range(&topo, seam, face, true, bad).unwrap_err();
            assert!(
                matches!(
                    err,
                    TopologyError::InvalidToleranceValue {
                        entity: "same-range validation",
                        ..
                    }
                ),
                "got {err:?}"
            );
            assert!(matches!(
                validate_same_range_strict(&topo, seam, face, true, bad),
                Err(CurveUseValidationError::InvalidTolerance { .. })
            ));
            assert!(matches!(
                validate_solid_pcurve_contracts(&topo, solid, bad, 8),
                Err(CurveUseValidationError::InvalidTolerance { .. })
            ));
        }
    }

    #[test]
    fn entity_tolerance_guards_reject_negative_vertex_and_edge_claims() {
        let plane = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let build = |vertex_tolerance: f64, edge_tolerance: Option<f64>| {
            let mut topo = Topology::new();
            let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), vertex_tolerance));
            let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
            let edge = topo.add_edge(Edge::with_tolerance(
                v0,
                v1,
                EdgeCurve::Line,
                edge_tolerance,
            ));
            let wire =
                topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());
            let face = topo.add_face(Face::new(wire, vec![], plane.clone()));
            (topo, edge, face)
        };

        let (topo, edge, face) = build(1e-7, None);
        validate_same_range(&topo, edge, face, true, 1e-7).unwrap();

        let (topo, edge, face) = build(-1.0, None);
        let err = validate_same_range(&topo, edge, face, true, 1e-7).unwrap_err();
        assert!(
            matches!(
                err,
                TopologyError::InvalidToleranceValue {
                    entity: "vertex",
                    ..
                }
            ),
            "got {err:?}"
        );

        let (topo, edge, face) = build(1e-7, Some(-1.0));
        let err = validate_same_range(&topo, edge, face, true, 1e-7).unwrap_err();
        assert!(
            matches!(
                err,
                TopologyError::InvalidToleranceValue { entity: "edge", .. }
            ),
            "got {err:?}"
        );
    }

    #[test]
    fn same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3, true))
            .unwrap();
        let measured = check_same_range(&topo, seam, face, true).unwrap().unwrap();
        assert!(
            measured > 1e-7,
            "the fixture must deviate past the entity tolerance"
        );

        let err = validate_same_range(&topo, seam, face, true, 1e-7).unwrap_err();
        assert!(
            matches!(err, TopologyError::SameRangeExceeded { .. }),
            "got {err:?}"
        );
        // Exactly at its own measured deviation the use is inside the band.
        validate_same_range(&topo, seam, face, true, measured).unwrap();
    }

    #[test]
    fn a_deviation_exactly_at_the_bound_is_inside_every_band() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3, true))
            .unwrap();

        let sampled = check_same_parameter(&topo, seam, face, true, 16)
            .unwrap()
            .unwrap()
            .max_deviation;
        assert!(sampled > 1e-7);
        validate_same_parameter(&topo, seam, face, true, sampled, 16).unwrap();
        assert!(validate_same_parameter(&topo, seam, face, true, 1e-7, 16).is_err());

        let strict = check_same_parameter_strict(&topo, seam, face, true, 16)
            .unwrap()
            .unwrap()
            .max_deviation;
        assert!(strict > 1e-7);
        validate_same_parameter_strict(&topo, seam, face, true, strict, 16).unwrap();
        assert!(validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 16).is_err());

        let range = check_same_range_strict(&topo, seam, face, true)
            .unwrap()
            .unwrap();
        assert!(range > 1e-7);
        validate_same_range_strict(&topo, seam, face, true, range).unwrap();
        assert!(validate_same_range_strict(&topo, seam, face, true, 1e-7).is_err());
    }

    #[test]
    fn solid_pcurve_contracts_accept_a_use_exactly_at_the_bound() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face, false, seam_pcurve(TAU, false))
            .unwrap();
        // Zero balls so the caller's tolerance alone is the acting bound.
        for vid in [
            topo.edge(seam).unwrap().start(),
            topo.edge(seam).unwrap().end(),
        ] {
            topo.vertex_mut(vid).unwrap().set_tolerance(0.0).unwrap();
        }
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        let widest = [true, false]
            .into_iter()
            .map(|forward| {
                check_same_parameter_strict(&topo, seam, face, forward, 8)
                    .unwrap()
                    .unwrap()
                    .max_deviation
            })
            .fold(0.0_f64, f64::max);
        assert!(widest > 0.0);

        let summary = validate_solid_pcurve_contracts(&topo, solid, widest, 8).unwrap();
        assert_eq!(summary.validated_uses, 2);
        assert!(validate_solid_pcurve_contracts(&topo, solid, 0.0, 8).is_err());
    }
}

/// RFC 0004 Stage 1: the entity-tolerance validators.
///
/// `check_vertex_ball` / `validate_vertex_ball` enforce invariant 1 (ball
/// containment); `check_edge_tube` / `validate_edge_tube` enforce invariant
/// 2 (tube containment) with an entity-derived bound. Both pass vacuously
/// at default tolerances on exact geometry, and both sides of each bound
/// are pinned here so a later stage's flips are visible diffs.
#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
#[allow(clippy::panic, clippy::float_cmp)]
mod tolerant_checks_tests {
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::diagnostic::{FailureCategory, ToDiagnostic};
    use remus_math::traits::ParametricCurve;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};

    use crate::TopologyError;
    use crate::edge::{Edge, EdgeCurve, EdgeId};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::pcurve::PCurve;
    use crate::vertex::{Vertex, VertexId};
    use crate::wire::{OrientedEdge, Wire};

    use super::{
        Topology, check_edge_tube, check_vertex_ball, validate_edge_tube, validate_same_parameter,
        validate_vertex_ball,
    };

    const TAU: f64 = std::f64::consts::TAU;

    /// A vertical seam edge on a unit cylinder (bottom (1,0,0) to top
    /// (1,0,1)) with its side face.
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
                remus_math::surfaces::CylindricalSurface::with_ref_dir(
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

    /// The seam's pcurve at horizontal offset `u`: the surface image is a
    /// chord `2*sin(u/2)` away from the 3D seam edge.
    fn seam_pcurve(u: f64) -> PCurve {
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(u, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
            0.0,
            1.0,
        )
    }

    /// Vertex V anchors at curve(0) = (1, 0, 0) but the edge's trim starts
    /// at `gap_angle`, so the curve's endpoint evaluation misses V's point
    /// by the chord `2*sin(gap_angle/2)`.
    fn circle_arc_edge(gap_angle: f64) -> (Topology, VertexId, EdgeId) {
        let mut topo = Topology::new();
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let anchor = ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(anchor, 1e-7));
        let v2 = topo.add_vertex(Vertex::new(ParametricCurve::evaluate(&circle, 0.5), 1e-7));
        let mut edge = Edge::new(v, v2, EdgeCurve::Circle(circle));
        edge.set_trim(Some((gap_angle, 0.5)));
        let edge_id = topo.add_edge(edge);
        (topo, v, edge_id)
    }

    #[test]
    fn vertex_ball_passes_vacuously_for_exact_endpoints() {
        // A full-circle rim anchored at its own zero-angle point: the curve
        // endpoint evaluations land exactly on the vertex point, so the
        // default ball is vacuously sufficient.
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let anchor = ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(anchor, 1e-7));
        let mut rim = Edge::new(v, v, EdgeCurve::Circle(circle));
        rim.set_trim(Some((0.0, TAU)));
        topo.add_edge(rim);

        validate_vertex_ball(&topo, v).unwrap();
        let reports = check_vertex_ball(&topo, v).unwrap();
        assert_eq!(reports.len(), 1);
        assert!(reports[0].deviation < 1e-12);
    }

    #[test]
    fn vertex_ball_fails_closed_without_curved_parameter_authority() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let start = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(circle.evaluate(0.5), 1e-7));
        let edge = topo.add_edge(Edge::new(start, end, EdgeCurve::Circle(circle)));

        let reports = check_vertex_ball(&topo, start).unwrap();
        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].edge, edge);
        assert_eq!(reports[0].deviation, f64::MAX);
        assert!(matches!(
            validate_vertex_ball(&topo, start),
            Err(TopologyError::VertexBallExceeded { deviation, .. }) if deviation == f64::MAX
        ));
    }

    #[test]
    fn vertex_ball_passes_for_an_isolated_vertex() {
        let mut topo = Topology::new();
        let v = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        validate_vertex_ball(&topo, v).unwrap();
        assert!(check_vertex_ball(&topo, v).unwrap().is_empty());
    }

    #[test]
    fn vertex_ball_violation_fires_when_the_ball_understates_the_curve_gap() {
        let (topo, v, edge) = circle_arc_edge(1e-3);
        let deviation = 2.0 * (5e-4_f64).sin();

        let reports = check_vertex_ball(&topo, v).unwrap();
        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].edge, edge);
        assert!(reports[0].at_start);
        assert!((reports[0].deviation - deviation).abs() < 1e-9);

        let err = validate_vertex_ball(&topo, v).unwrap_err();
        let TopologyError::VertexBallExceeded {
            vertex, deviation, ..
        } = err
        else {
            panic!("expected VertexBallExceeded, got {err:?}")
        };
        assert_eq!(vertex.index(), v.index());
        assert!((deviation - 2.0 * (5e-4_f64).sin()).abs() < 1e-9);
    }

    #[test]
    fn a_raise_that_covers_the_gap_clears_the_ball_violation() {
        // Both sides of the ball bound: a raise just below the measured gap
        // still fails (the claim does not cover the deviation it papers
        // over), and a raise at the measured gap passes.
        let (mut topo, v, _edge) = circle_arc_edge(1e-3);
        let deviation = 2.0 * (5e-4_f64).sin();
        assert!(validate_vertex_ball(&topo, v).is_err());

        let undersized = deviation * 0.999;
        topo.vertex_mut(v)
            .unwrap()
            .set_tolerance(undersized)
            .unwrap();
        assert!(validate_vertex_ball(&topo, v).is_err());

        topo.vertex_mut(v)
            .unwrap()
            .set_tolerance(deviation)
            .unwrap();
        validate_vertex_ball(&topo, v).unwrap();
    }

    #[test]
    fn vertex_ball_checks_each_incident_edge_end() {
        // Two incident edges: one exact (a closed rim anchored on the
        // vertex), one with a gap. The report carries both ends and the
        // validator fires on the gappy one.
        let (mut topo, v, gappy) = circle_arc_edge(1e-3);
        // The rim must share the gappy edge's frame so its zero-angle point
        // is the same (1, 0, 0) anchor the vertex sits on.
        let rim_circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let mut rim = Edge::new(v, v, EdgeCurve::Circle(rim_circle));
        rim.set_trim(Some((0.0, TAU)));
        topo.add_edge(rim);

        let reports = check_vertex_ball(&topo, v).unwrap();
        assert_eq!(reports.len(), 2, "both incident ends are measured");
        let gappy_report = reports.iter().find(|r| r.edge == gappy).unwrap();
        assert!((gappy_report.deviation - 2.0 * (5e-4_f64).sin()).abs() < 1e-9);
        let exact_report = reports.iter().find(|r| r.edge != gappy).unwrap();
        assert!(exact_report.deviation < 1e-12);
        assert!(validate_vertex_ball(&topo, v).is_err());
    }

    // ── Edge tube (invariant 2) ─────────────────────────────────────────

    #[test]
    fn edge_tube_passes_vacuously_without_a_stored_pcurve() {
        let (topo, seam, face) = cylinder_seam();
        assert!(
            check_edge_tube(&topo, seam, face, true, 32)
                .unwrap()
                .is_none()
        );
        validate_edge_tube(&topo, seam, face, true, 32).unwrap();
    }

    #[test]
    fn edge_tube_passes_vacuously_at_default_tolerances() {
        // An exact pcurve (the seam's true image): the sampled deviation is
        // round-off only, far below the default bound.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.0))
            .unwrap();

        let report = check_edge_tube(&topo, seam, face, true, 32)
            .unwrap()
            .unwrap();
        assert!(report.max_deviation < 1e-12);
        assert_eq!(
            report.effective_tolerance, 1e-7,
            "default bound = the floor"
        );
        validate_edge_tube(&topo, seam, face, true, 32).unwrap();
        validate_edge_tube(&topo, seam, face, false, 32).unwrap();
    }

    #[test]
    fn edge_tube_violation_fires_beyond_the_effective_tolerance() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3))
            .unwrap();
        let deviation = 2.0 * (0.15_f64).sin();

        let report = check_edge_tube(&topo, seam, face, true, 32)
            .unwrap()
            .unwrap();
        assert!((report.parameter_deviation - deviation).abs() < 1e-9);
        assert!((report.range_deviation - deviation).abs() < 1e-9);
        assert!((report.max_deviation - deviation).abs() < 1e-9);
        assert!((report.effective_tolerance - 1e-7).abs() < 1e-18);

        let err = validate_edge_tube(&topo, seam, face, true, 32).unwrap_err();
        let TopologyError::EdgeTubeExceeded {
            edge, tolerance, ..
        } = err
        else {
            panic!("expected EdgeTubeExceeded, got {err:?}")
        };
        assert_eq!(edge.index(), seam.index());
        assert!((tolerance - 1e-7).abs() < 1e-20);
    }

    #[test]
    fn a_declared_tolerance_must_cover_the_deviation_it_claims() {
        // The checked-not-asserted rule: a raise past the measured
        // deviation clears the validator; a raise that still understates
        // the deviation is rejected by the validator, not papered over.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3))
            .unwrap();
        let deviation = 2.0 * (0.15_f64).sin();

        assert!(validate_edge_tube(&topo, seam, face, true, 32).is_err());

        topo.edge_mut(seam)
            .unwrap()
            .set_tolerance(Some(0.31))
            .unwrap();
        validate_edge_tube(&topo, seam, face, true, 32).unwrap();

        topo.edge_mut(seam)
            .unwrap()
            .set_tolerance(Some(0.1))
            .unwrap();
        let err = validate_edge_tube(&topo, seam, face, true, 32).unwrap_err();
        assert!(matches!(err, TopologyError::EdgeTubeExceeded { .. }));
        assert!(deviation > 0.29);
    }

    #[test]
    fn edge_tube_bound_falls_back_to_the_bounding_vertex_balls() {
        // No edge-declared tolerance: the effective bound is the wider
        // bounding ball, floored at the global linear tolerance.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3))
            .unwrap();

        for vid in [
            topo.edge(seam).unwrap().start(),
            topo.edge(seam).unwrap().end(),
        ] {
            topo.vertex_mut(vid).unwrap().set_tolerance(0.2).unwrap();
        }
        assert!(validate_edge_tube(&topo, seam, face, true, 32).is_err());

        for vid in [
            topo.edge(seam).unwrap().start(),
            topo.edge(seam).unwrap().end(),
        ] {
            topo.vertex_mut(vid).unwrap().set_tolerance(0.35).unwrap();
        }
        validate_edge_tube(&topo, seam, face, true, 32).unwrap();
    }

    #[test]
    fn edge_tube_clamps_sub_floor_claims_to_the_global_floor() {
        // A sub-floor declared tube (extra precision, like sewing's 3.5e-8)
        // never narrows the check below the global floor — but it also
        // cannot cover a deviation above the floor.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(5e-8))
            .unwrap();
        topo.edge_mut(seam)
            .unwrap()
            .set_tolerance(Some(1e-8))
            .unwrap();

        // Deviation ~5e-8: above the declared 1e-8 claim, but the floor
        // rule clamps the acting bound to 1e-7, so the use passes.
        let report = check_edge_tube(&topo, seam, face, true, 32)
            .unwrap()
            .unwrap();
        assert!((report.effective_tolerance - 1e-7).abs() < 1e-20);
        assert!(report.max_deviation > 1e-8, "the claim alone would fail");
        validate_edge_tube(&topo, seam, face, true, 32).unwrap();

        // A deviation above the floor fires even against the widened bound.
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(3e-7))
            .unwrap();
        assert!(validate_edge_tube(&topo, seam, face, true, 32).is_err());
    }

    #[test]
    fn validate_same_parameter_bound_includes_entity_tolerance() {
        // RFC 0004 Stage 2 flips the Stage 1 characterization: a declared
        // edge tube widens the caller's SameParameter bound.
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3))
            .unwrap();
        topo.edge_mut(seam)
            .unwrap()
            .set_tolerance(Some(0.5))
            .unwrap();

        validate_same_parameter(&topo, seam, face, true, 1e-7, 32).unwrap();
        super::validate_same_parameter_strict(&topo, seam, face, true, 1e-7, 32).unwrap();
        super::validate_same_range(&topo, seam, face, true, 1e-7).unwrap();
        super::validate_same_range_strict(&topo, seam, face, true, 1e-7).unwrap();
        validate_same_parameter(&topo, seam, face, true, 0.31, 32).unwrap();

        topo.edge_mut(seam).unwrap().set_tolerance(None).unwrap();
        assert!(matches!(
            validate_same_parameter(&topo, seam, face, true, 1e-7, 32),
            Err(TopologyError::SameParameterExceeded { .. })
        ));
    }

    #[test]
    fn entity_tolerance_diagnostics_are_pinned() {
        let (topo, seam, face) = cylinder_seam();
        let bottom = topo.edge(seam).unwrap().start();
        let d = TopologyError::EdgeTubeExceeded {
            edge: seam,
            face,
            max_deviation: 0.3,
            at_parameter: 0.5,
            tolerance: 1e-7,
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::ToleranceViolation);
        assert_eq!(d.code(), "edge_tube_violation");

        let d = TopologyError::VertexBallExceeded {
            vertex: bottom,
            edge: seam,
            deviation: 0.3,
            tolerance: 1e-7,
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::ToleranceViolation);
        assert_eq!(d.code(), "vertex_ball_violation");

        let d = TopologyError::InvalidToleranceValue {
            entity: "vertex",
            value: f64::NAN,
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "entity_tolerance_invalid");
    }

    #[test]
    fn vertex_ball_reports_exactly_the_incident_edge_ends() {
        // One edge meets the vertex at its END, one at its START, and one
        // misses it entirely.
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), 1e-7));
        let incoming = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let outgoing = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let bypass = topo.add_edge(Edge::new(v0, v2, EdgeCurve::Line));

        let reports = check_vertex_ball(&topo, v1).unwrap();
        assert_eq!(reports.len(), 2, "exactly the two incident ends");
        let incoming_report = reports
            .iter()
            .find(|report| report.edge == incoming)
            .expect("an edge that ENDS at the vertex is incident to it");
        assert!(!incoming_report.at_start);
        let outgoing_report = reports
            .iter()
            .find(|report| report.edge == outgoing)
            .expect("an edge that STARTS at the vertex is incident to it");
        assert!(outgoing_report.at_start);
        assert!(
            reports.iter().all(|report| report.edge != bypass),
            "an edge touching neither end is never measured"
        );
    }

    #[test]
    fn vertex_ball_reports_max_for_a_non_finite_curve_evaluation() {
        // The parameter is finite but the far endpoint poisons the curve
        // evaluation: the measurement must fail closed, not report NaN.
        let mut topo = Topology::new();
        let anchored = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let poisoned = topo.add_vertex(Vertex::new(Point3::new(f64::NAN, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(anchored, poisoned, EdgeCurve::Line));

        let reports = check_vertex_ball(&topo, anchored).unwrap();
        assert_eq!(reports.len(), 1);
        assert_eq!(reports[0].edge, edge);
        assert_eq!(reports[0].deviation, f64::MAX);
    }

    #[test]
    fn edge_tube_accepts_a_declared_bound_exactly_at_its_measured_deviation() {
        let (mut topo, seam, face) = cylinder_seam();
        topo.set_pcurve_oriented(seam, face, true, seam_pcurve(0.3))
            .unwrap();
        let measured = check_edge_tube(&topo, seam, face, true, 32)
            .unwrap()
            .unwrap()
            .max_deviation;
        assert!(measured > 1e-7, "the fixture must deviate past the floor");
        assert!(validate_edge_tube(&topo, seam, face, true, 32).is_err());

        // A claim that exactly covers the measured deviation is covered.
        topo.edge_mut(seam)
            .unwrap()
            .set_tolerance(Some(measured))
            .unwrap();
        let report = check_edge_tube(&topo, seam, face, true, 32)
            .unwrap()
            .unwrap();
        assert_eq!(report.effective_tolerance, measured);
        validate_edge_tube(&topo, seam, face, true, 32).unwrap();
    }
}
