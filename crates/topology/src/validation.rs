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

/// Result of a `SameParameter` deviation scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SameParameterReport {
    /// Largest sampled deviation between the 3D curve and the pcurve's
    /// surface image, in model units.
    pub max_deviation: f64,
    /// The pcurve parameter at which it occurred.
    pub at_parameter: f64,
    /// Number of samples taken.
    pub samples: usize,
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
    let mut at_parameter = p0;
    for k in 0..=samples {
        #[allow(clippy::cast_precision_loss)]
        let f = k as f64 / samples as f64;
        let tp = p0 + f * (p1 - p0);
        let uv = pcurve.evaluate(tp);
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
    let (Some(s0), Some(s1)) = (
        surface.evaluate(uv0.x(), uv0.y()),
        surface.evaluate(uv1.x(), uv1.y()),
    ) else {
        return Ok(None);
    };
    let d0 = (s0 - oriented_start).length();
    let d1 = (s1 - oriented_end).length();
    Ok(Some(d0.max(d1)))
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
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};

    use crate::TopologyError;
    use crate::edge::{Edge, EdgeCurve, EdgeId};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::pcurve::PCurve;
    use crate::topology::Topology;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    use super::{check_same_parameter, validate_same_parameter, validate_same_range};

    const TAU: f64 = std::f64::consts::TAU;

    /// Cylinder side face bounded by its bottom rim circle (full circle,
    /// closed wire). The rim's exact pcurve is the line v = 0, u = t.
    fn cylinder_with_rim() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        // Anchor the seam vertex to the curve's own zero-angle point so the
        // fixture does not assume how the reference frame is derived.
        let p0 = remus_math::traits::ParametricCurve::evaluate(&circle, 0.0);
        let v = topo.add_vertex(Vertex::new(p0, 1e-7));
        let rim = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
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

    #[test]
    fn exact_pcurve_passes_same_parameter_and_range() {
        let (mut topo, rim, face) = cylinder_with_rim();
        topo.set_pcurve_oriented(rim, face, true, rim_pcurve(0.0));

        let report = check_same_parameter(&topo, rim, face, true, 32)
            .unwrap()
            .unwrap();
        assert!(
            report.max_deviation < 1e-9,
            "exact rim pcurve must have ~zero deviation, got {}",
            report.max_deviation
        );
        validate_same_parameter(&topo, rim, face, true, 1e-7, 32).unwrap();
        validate_same_range(&topo, rim, face, true, 1e-7).unwrap();
    }

    #[test]
    fn offset_pcurve_fails_with_typed_tolerance_violation() {
        let (mut topo, rim, face) = cylinder_with_rim();
        // v = 0.3: the surface image floats 0.3 above the rim everywhere.
        topo.set_pcurve_oriented(rim, face, true, rim_pcurve(0.3));

        let err = validate_same_parameter(&topo, rim, face, true, 1e-7, 32).unwrap_err();
        let TopologyError::SameParameterExceeded { max_deviation, .. } = err else {
            unreachable!("expected SameParameterExceeded, got {err:?}")
        };
        assert!((max_deviation - 0.3).abs() < 1e-9);

        assert!(matches!(
            validate_same_range(&topo, rim, face, true, 1e-7),
            Err(TopologyError::SameRangeExceeded { .. })
        ));
    }

    #[test]
    fn missing_pcurve_and_planar_faces_pass_vacuously() {
        let (topo, rim, face) = cylinder_with_rim();
        assert!(
            check_same_parameter(&topo, rim, face, true, 8)
                .unwrap()
                .is_none()
        );
        validate_same_parameter(&topo, rim, face, true, 1e-7, 8).unwrap();
    }

    #[test]
    fn tolerance_violation_diagnostics_are_pinned() {
        use remus_math::diagnostic::{FailureCategory, ToDiagnostic};
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
