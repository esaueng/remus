//! Edge filleting (rounding edges with a constant or variable radius).
//!
//! Replaces sharp edges with a smooth cylindrical fillet surface.
//! Supports edges between planar faces, analytic faces (cylinder, cone,
//! sphere, torus), and NURBS faces from a prior fillet operation.  Each
//! filleted edge is replaced by a true rolling-ball NURBS blend surface
//! with G1 tangent continuity.
//!
//! For NURBS adjacent faces the outward normal is computed by projecting
//! the edge sample point onto the surface, giving accurate cross-section
//! geometry (see `face_surface_normal_at`).  Non-planar faces containing
//! target edges are trimmed by offsetting boundary vertices at fillet
//! contact locations along face boundary directions.
//!
//! The rolling-ball algorithm:
//! 1. For each target edge, find the two adjacent planar faces
//! 2. Offset each face plane inward by radius R
//! 3. Intersect the offset planes to find the fillet center line (spine)
//! 4. Compute contact points where the rolling ball touches each face
//! 5. Build a degree (2,1) rational NURBS surface: circular arc cross-section
//!    swept along the edge
//! 6. Trim the adjacent faces along the contact lines
//! 7. Assemble the result with modified faces + NURBS fillet faces
//!
//! The NURBS fillet surface uses the exact rational circular arc
//! representation (3 control points, weights [1, cos(α/2), 1]),
//! giving mathematically exact G1 continuity with both adjacent faces.

mod geometry;
mod helpers;
mod rolling_ball;
#[cfg(test)]
mod tests;

pub(crate) use geometry::face_surface_normal_at;
pub use remus_blend::radius_law::StandardRadiusLaw as FilletRadiusLaw;
#[allow(deprecated)]
pub use rolling_ball::fillet_rolling_ball;
pub(crate) use rolling_ball::fillet_rolling_ball_with_origins;

use std::collections::{HashMap, HashSet};

use remus_math::frame::Frame3;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use crate::boolean::FaceSpec;
use crate::dot_normal_point;

use helpers::{FacePolygon, FilletEdgeData, extract_inner_wire_positions, record_fillet_point};

/// Variable-radius law and explicit endpoint setbacks for one fillet spine.
///
/// Setbacks are physical distances measured from the stored start/end vertex
/// along the original edge. The radius law is normalized over the remaining
/// active stripe, so its `start` and `end` values remain the built endpoints.
#[derive(Debug, Clone)]
pub struct FilletEdgeSetback {
    /// Edge carrying the variable-radius stripe.
    pub edge: EdgeId,
    /// Radius law over the normalized active stripe domain.
    pub law: FilletRadiusLaw,
    /// Distance from the edge's stored start vertex to the stripe start.
    pub start_setback: f64,
    /// Distance from the edge's stored end vertex to the stripe end.
    pub end_setback: f64,
}

/// Fillet `seed_edges` and all G1-continuous edges connected to them.
///
/// [`fillet_rolling_ball`] performs the same shared G1-chain expansion
/// internally, so this backward-compatible wrapper forwards directly.
///
/// # Errors
///
/// Returns the same errors as [`fillet_rolling_ball`].
#[allow(deprecated)]
pub fn fillet_rolling_ball_propagate_g1(
    topo: &mut Topology,
    solid: SolidId,
    seed_edges: &[EdgeId],
    radius: f64,
) -> Result<SolidId, crate::OperationsError> {
    fillet_rolling_ball(topo, solid, seed_edges, radius)
}

/// Fillet one or more edges of a solid with a constant radius (flat chamfer).
///
/// **Deprecated**: This creates flat bevel faces, not rounded fillets.
/// Use [`fillet_rolling_ball`] for true G1-continuous NURBS blend surfaces.
///
/// Each target edge is replaced by a flat bevel face (chamfer-like
/// approximation of a fillet arc).
///
/// The call is transactional and fail-closed: a requested edge that cannot
/// carry a bevel is reported by name ([`remus_blend::BlendError::EdgesNotBlended`]),
/// the result is validated against the input before it is returned, and any
/// failure leaves the topology exactly as it was. It never answers with the
/// input handle or a quietly reduced subset of the selection.
///
/// # Errors
///
/// Returns an error if:
/// - `radius` is non-positive
/// - `edges` is empty
/// - Any edge is not shared by exactly two faces
/// - A target edge is adjacent to a non-planar face
/// - The assembled result regresses validation against the input or moves an
///   impossible amount of material
#[deprecated(
    since = "0.8.0",
    note = "Use fillet_rolling_ball for true rounded fillets"
)]
pub fn fillet(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<SolidId, crate::OperationsError> {
    remus_topology::transaction::run_transacted(topo, |t| {
        fillet_transacted(t, solid, edges, radius)
    })
}

/// Transaction body of [`fillet`]: builds the bevel, then proves the result
/// against the input before committing.
#[allow(clippy::too_many_lines)]
fn fillet_transacted(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();

    if radius <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("fillet radius must be positive, got {radius}"),
        });
    }
    if edges.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no edges specified for fillet".into(),
        });
    }

    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let shell_face_ids: Vec<_> = shell.faces().to_vec();

    let mut edge_to_faces: HashMap<usize, Vec<_>> = HashMap::new();
    let mut face_polygons: HashMap<usize, FacePolygon> = HashMap::new();

    for &face_id in &shell_face_ids {
        let face = topo.face(face_id)?;

        let wire = topo.wire(face.outer_wire())?;
        let mut vertex_ids = Vec::with_capacity(wire.edges().len());
        let mut positions = Vec::with_capacity(wire.edges().len());
        let mut wire_edge_ids = Vec::with_capacity(wire.edges().len());

        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let vid = oe.oriented_start(edge);
            vertex_ids.push(vid);
            positions.push(topo.vertex(vid)?.point());
            wire_edge_ids.push(oe.edge());

            edge_to_faces
                .entry(oe.edge().index())
                .or_default()
                .push(face_id);
        }

        // Inner wire edges also contribute to adjacency: an edge shared
        // between a face's inner wire (hole boundary) and another face's
        // outer wire should be counted for both faces.
        // Also extract inner wire vertex positions for preservation.
        let mut face_inner_wires = Vec::new();
        for &inner_wid in face.inner_wires() {
            let inner_wire = topo.wire(inner_wid)?;
            let mut iw_positions = Vec::new();
            for oe in inner_wire.edges() {
                edge_to_faces
                    .entry(oe.edge().index())
                    .or_default()
                    .push(face_id);
                let edge = topo.edge(oe.edge())?;
                let vid = oe.oriented_start(edge);
                iw_positions.push(topo.vertex(vid)?.point());
            }
            if !iw_positions.is_empty() {
                face_inner_wires.push(iw_positions);
            }
        }

        // Only build polygon data for planar faces. Non-planar faces
        // will be passed through unchanged if they don't contain target edges.
        let normal = match face.surface() {
            FaceSurface::Plane { normal, .. } => *normal,
            _ => continue,
        };
        if positions.is_empty() {
            continue;
        }
        let d = dot_normal_point(normal, positions[0]);

        face_polygons.insert(
            face_id.index(),
            FacePolygon {
                vertex_ids,
                positions,
                wire_edge_ids,
                normal,
                d,
                inner_wires: face_inner_wires,
            },
        );
    }

    // Every requested edge must be a manifold edge of this solid (shared by
    // exactly two faces). Dropping the rest quietly would return a valid,
    // plausibly-sized solid that simply lacks some of the blends it was asked
    // for — indistinguishable from success to the caller. Name them instead.
    let mut dropped: Vec<EdgeId> = Vec::new();
    let filtered_edges: Vec<EdgeId> = edges
        .iter()
        .copied()
        .filter(|edge_id| {
            let manifold = edge_to_faces
                .get(&edge_id.index())
                .is_some_and(|faces| faces.len() == 2);
            if !manifold && !dropped.contains(edge_id) {
                dropped.push(*edge_id);
            }
            manifold
        })
        .collect();

    if !dropped.is_empty() {
        return Err(crate::OperationsError::Blend(
            remus_blend::BlendError::EdgesNotBlended {
                edges: dropped,
                reason: "not manifold edges of this solid (boundary, seam, or foreign edge); \
                         the flat-bevel engine only blends edges shared by exactly two faces"
                    .into(),
            },
        ));
    }

    if filtered_edges.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no manifold edges to fillet (all edges are boundary or missing)".into(),
        });
    }

    let target_set: HashSet<usize> = filtered_edges.iter().map(|e| e.index()).collect();

    // Vertices at endpoints of filleted edges (used to detect side-face corners).
    let mut vertex_fillet_endpoints: HashSet<usize> = HashSet::new();
    for &edge_id in &filtered_edges {
        let edge = topo.edge(edge_id)?;
        vertex_fillet_endpoints.insert(edge.start().index());
        vertex_fillet_endpoints.insert(edge.end().index());
    }

    // Strategy: identical to chamfer but with more offset segments to
    // approximate the circular fillet.
    let mut fillet_data: HashMap<usize, FilletEdgeData> = HashMap::new();
    let mut result_specs: Vec<FaceSpec> = Vec::new();

    for &face_id in &shell_face_ids {
        // Non-planar faces pass through unchanged.
        let Some(poly) = face_polygons.get(&face_id.index()) else {
            let face = topo.face(face_id)?;
            let verts = crate::boolean::face_polygon(topo, face_id)?;
            let np_inner = extract_inner_wire_positions(topo, face)?;
            result_specs.push(FaceSpec::Surface {
                vertices: verts,
                surface: face.surface().clone(),
                reversed: false,
                inner_wires: np_inner,
            });
            continue;
        };
        let n = poly.positions.len();
        let mut new_verts: Vec<Point3> = Vec::with_capacity(n + target_set.len());

        for i in 0..n {
            let prev_i = if i == 0 { n - 1 } else { i - 1 };
            let next_i = (i + 1) % n;

            let before_filleted = target_set.contains(&poly.wire_edge_ids[prev_i].index());
            let after_filleted = target_set.contains(&poly.wire_edge_ids[i].index());

            let pos = poly.positions[i];
            let prev_pos = poly.positions[prev_i];
            let next_pos = poly.positions[next_i];

            // Check if vertex sits at a fillet endpoint even though neither
            // adjacent edge of THIS face is the filleted edge (side face case).
            let at_fillet_endpoint = vertex_fillet_endpoints.contains(&poly.vertex_ids[i].index());

            match (before_filleted, after_filleted, at_fillet_endpoint) {
                (false, false, false) => {
                    new_verts.push(pos);
                }
                (false, false, true) => {
                    // Side face corner: split into two contact points.
                    let dir_prev = (prev_pos - pos).normalize()?;
                    new_verts.push(pos + dir_prev * radius);

                    let dir_next = (next_pos - pos).normalize()?;
                    new_verts.push(pos + dir_next * radius);
                }
                (true, false, _) => {
                    let dir = (next_pos - pos).normalize()?;
                    let c = pos + dir * radius;
                    new_verts.push(c);
                    record_fillet_point(
                        &mut fillet_data,
                        poly.wire_edge_ids[prev_i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        c,
                    );
                }
                (false, true, _) => {
                    let dir = (prev_pos - pos).normalize()?;
                    let c = pos + dir * radius;
                    new_verts.push(c);
                    record_fillet_point(
                        &mut fillet_data,
                        poly.wire_edge_ids[i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        c,
                    );
                }
                (true, true, _) => {
                    let dir_prev = (prev_pos - pos).normalize()?;
                    let c_after = pos + dir_prev * radius;
                    new_verts.push(c_after);
                    record_fillet_point(
                        &mut fillet_data,
                        poly.wire_edge_ids[i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        c_after,
                    );

                    let dir_next = (next_pos - pos).normalize()?;
                    let c_before = pos + dir_next * radius;
                    new_verts.push(c_before);
                    record_fillet_point(
                        &mut fillet_data,
                        poly.wire_edge_ids[prev_i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        c_before,
                    );
                }
            }
        }

        let new_d = dot_normal_point(poly.normal, new_verts[0]);
        result_specs.push(FaceSpec::Planar {
            vertices: new_verts,
            normal: poly.normal,
            d: new_d,
            inner_wires: poly.inner_wires.clone(),
        });
    }

    for &edge_id in &filtered_edges {
        let data = fillet_data.get(&edge_id.index()).ok_or_else(|| {
            crate::OperationsError::InvalidInput {
                reason: format!("failed to compute fillet data for edge {}", edge_id.index()),
            }
        })?;

        let edge = topo.edge(edge_id)?;
        let v_start = edge.start();
        let v_end = edge.end();

        let Some(face_list) = edge_to_faces.get(&edge_id.index()) else {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!(
                    "fillet: edge {} not found in edge-to-face map",
                    edge_id.index()
                ),
            });
        };
        if face_list.len() < 2 {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!(
                    "fillet: edge {} has {} adjacent faces, expected 2",
                    edge_id.index(),
                    face_list.len()
                ),
            });
        }
        let f1 = face_list[0];
        let f2 = face_list[1];

        let c1_start = data.get_point(f1, v_start)?;
        let c1_end = data.get_point(f1, v_end)?;
        let c2_start = data.get_point(f2, v_start)?;
        let c2_end = data.get_point(f2, v_end)?;

        let n1 = face_polygons[&f1.index()].normal;
        let n2 = face_polygons[&f2.index()].normal;
        let avg_normal = n1 + n2;

        let edge_a = c2_start - c1_start;
        let edge_b = c1_end - c1_start;
        let raw_normal = edge_a.cross(edge_b);

        let (quad, normal) = if raw_normal.dot(avg_normal) >= 0.0 {
            (
                vec![c1_start, c2_start, c2_end, c1_end],
                raw_normal.normalize()?,
            )
        } else {
            let flipped = edge_b.cross(edge_a);
            (
                vec![c1_start, c1_end, c2_end, c2_start],
                flipped.normalize()?,
            )
        };

        let d = dot_normal_point(normal, quad[0]);
        result_specs.push(FaceSpec::Planar {
            vertices: quad,
            normal,
            d,
            inner_wires: vec![],
        });
    }

    let result = crate::boolean::assemble_solid_mixed(topo, &result_specs, tol)?;

    // Fail closed on a plausible-but-wrong result: the bevel must not regress
    // validation against the input, and a convex-edge bevel must REMOVE a
    // bounded amount of material (an oversized radius used to "succeed" with
    // the volume growing — e.g. r=50 on a 10 mm box returned 3833 mm³).
    crate::blend_ops::validate_blend_solid_against_input(topo, "fillet", solid, result)?;
    crate::blend_ops::validate_blend_volume(
        topo,
        "fillet",
        solid,
        result,
        edges,
        crate::blend_ops::BlendSize::Fillet { radius },
    )?;
    Ok(result)
}

/// Fillet edges with variable radius using canal surface generation.
///
/// Each edge gets a [`FilletRadiusLaw`] that defines how the radius
/// changes along the edge. The fillet surface is a canal surface:
/// the envelope of a sphere of varying radius moving along the edge.
///
/// The implementation samples the radius law at multiple points along
/// each edge, computes rolling-ball arc cross-sections at each sample,
/// and interpolates a NURBS surface through all cross-sections using
/// tensor-product surface fitting.
///
/// For constant radius, use `FilletRadiusLaw::Constant(r)` or the
/// simpler [`fillet_rolling_ball`] function.
///
/// The call is transactional and fail-closed: every requested edge must carry
/// a blend or the call fails with [`remus_blend::BlendError::EdgesNotBlended`]
/// naming the ones it could not; the assembled result is validated against the
/// input (no new validation errors, and the volume change must be one a blend
/// of this size can physically produce) before it is returned; and any failure
/// leaves the topology exactly as it was. It never answers with the input
/// handle or a quietly reduced subset of the selection.
///
/// # Errors
///
/// Returns errors similar to [`fillet_rolling_ball`].
pub fn fillet_variable(
    topo: &mut Topology,
    solid: SolidId,
    edge_laws: &[(EdgeId, FilletRadiusLaw)],
) -> Result<SolidId, crate::OperationsError> {
    remus_topology::transaction::run_transacted(topo, |t| {
        fillet_variable_transacted(t, solid, edge_laws, &HashMap::new())
    })
}

/// Fillet variable-radius edges over explicitly setback spine intervals.
///
/// A non-zero setback is currently qualified at a planar vertex where three
/// or more selected stripes meet one consistently oriented tangent ball. The
/// laws may differ away from the corner, but their radii must agree and have
/// zero slope at the declared stations. This preserves exact G1
/// stripe-to-corner seams; incompatible distances return a typed
/// [`remus_blend::BlendError::SetbackMismatch`] instead of emitting a merely
/// positional patch.
///
/// # Errors
///
/// Returns an error for malformed distances, unsupported endpoint topology,
/// incompatible corner stations, or the same construction/postcondition
/// failures as [`fillet_variable`]. The call is transactional.
pub fn fillet_variable_with_setbacks(
    topo: &mut Topology,
    solid: SolidId,
    specs: &[FilletEdgeSetback],
) -> Result<SolidId, crate::OperationsError> {
    let mut seen = HashSet::with_capacity(specs.len());
    let mut edge_laws = Vec::with_capacity(specs.len());
    let mut setbacks = HashMap::with_capacity(specs.len());
    for spec in specs {
        if seen.insert(spec.edge) {
            edge_laws.push((spec.edge, spec.law.clone()));
            setbacks.insert(spec.edge.index(), (spec.start_setback, spec.end_setback));
        }
    }
    remus_topology::transaction::run_transacted(topo, |t| {
        fillet_variable_transacted(t, solid, &edge_laws, &setbacks)
    })
}

/// Transaction body of [`fillet_variable`]: builds the canal surfaces, then
/// proves coverage and result validity before committing.
#[allow(clippy::too_many_lines)]
fn fillet_variable_transacted(
    topo: &mut Topology,
    solid: SolidId,
    edge_laws: &[(EdgeId, FilletRadiusLaw)],
    requested_setbacks: &HashMap<usize, (f64, f64)>,
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();

    if edge_laws.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no edges specified for fillet".into(),
        });
    }

    // Collapse repeated edges, keeping the caller's order: a repeated seed
    // would emit two coincident canal surfaces for one edge. First occurrence
    // wins, matching the constant-radius engines.
    let mut seen_edges = HashSet::with_capacity(edge_laws.len());
    let edge_laws: Vec<(EdgeId, FilletRadiusLaw)> = edge_laws
        .iter()
        .filter(|(edge_id, _)| seen_edges.insert(*edge_id))
        .cloned()
        .collect();
    let edge_laws = edge_laws.as_slice();

    for (_, law) in edge_laws {
        law.validated_bounds(tol.linear)?;
    }

    // Convert physical endpoint setbacks into the original edge parameter
    // domain. The first qualified cell is deliberately exact: straight
    // spines only, where distance and parameter are affine. Curved-spine arc
    // length inversion remains outside this issue rather than being sampled.
    let mut active_intervals: HashMap<usize, (f64, f64)> = HashMap::with_capacity(edge_laws.len());
    let mut declared_endpoint_setbacks: HashMap<(usize, usize), f64> = HashMap::new();
    for (edge_id, _) in edge_laws {
        let edge = topo.edge(*edge_id)?;
        let (mut start_setback, mut end_setback) = requested_setbacks
            .get(&edge_id.index())
            .copied()
            .unwrap_or((0.0, 0.0));
        if !start_setback.is_finite()
            || !end_setback.is_finite()
            || start_setback < 0.0
            || end_setback < 0.0
        {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!(
                    "fillet setbacks for edge {edge_id:?} must be finite and non-negative"
                ),
            });
        }
        if start_setback <= tol.linear {
            start_setback = 0.0;
        }
        if end_setback <= tol.linear {
            end_setback = 0.0;
        }
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let edge_length = (end - start).length();
        if edge_length <= tol.linear {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!("fillet edge {edge_id:?} is tolerance-collapsed"),
            });
        }
        if (start_setback > tol.linear || end_setback > tol.linear)
            && !matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line)
        {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!(
                    "explicit fillet setbacks currently require a straight spine; edge {edge_id:?} is curved"
                ),
            });
        }
        if start_setback + end_setback >= edge_length - tol.linear {
            return Err(crate::OperationsError::InvalidInput {
                reason: format!(
                    "fillet setbacks on edge {edge_id:?} consume its full length: {start_setback} + {end_setback} >= {edge_length}"
                ),
            });
        }
        active_intervals.insert(
            edge_id.index(),
            (start_setback / edge_length, 1.0 - end_setback / edge_length),
        );
        if start_setback > tol.linear {
            declared_endpoint_setbacks
                .insert((edge_id.index(), edge.start().index()), start_setback);
        }
        if end_setback > tol.linear {
            declared_endpoint_setbacks.insert((edge_id.index(), edge.end().index()), end_setback);
        }
    }

    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let shell_face_ids: Vec<_> = shell.faces().to_vec();

    let mut edge_to_faces: std::collections::HashMap<usize, Vec<_>> =
        std::collections::HashMap::new();
    let mut face_polygons: std::collections::HashMap<usize, FacePolygon> =
        std::collections::HashMap::new();
    let mut face_surfaces: std::collections::HashMap<usize, FaceSurface> =
        std::collections::HashMap::new();
    let target_set: std::collections::HashSet<usize> =
        edge_laws.iter().map(|(e, _)| e.index()).collect();

    for &face_id in &shell_face_ids {
        let face = topo.face(face_id)?;
        face_surfaces.insert(face_id.index(), face.surface().clone());

        let wire = topo.wire(face.outer_wire())?;
        let mut vertex_ids = Vec::new();
        let mut positions = Vec::new();
        let mut wire_edge_ids = Vec::new();

        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let vid = oe.oriented_start(edge);
            vertex_ids.push(vid);
            positions.push(topo.vertex(vid)?.point());
            wire_edge_ids.push(oe.edge());
            edge_to_faces
                .entry(oe.edge().index())
                .or_default()
                .push(face_id);
        }

        // Extract inner wire vertex positions for preservation.
        let mut face_inner_wires = Vec::new();
        for &inner_wid in face.inner_wires() {
            let inner_wire = topo.wire(inner_wid)?;
            let mut iw_positions = Vec::new();
            for oe in inner_wire.edges() {
                edge_to_faces
                    .entry(oe.edge().index())
                    .or_default()
                    .push(face_id);
                let edge_data = topo.edge(oe.edge())?;
                let vid = oe.oriented_start(edge_data);
                iw_positions.push(topo.vertex(vid)?.point());
            }
            if !iw_positions.is_empty() {
                face_inner_wires.push(iw_positions);
            }
        }

        // Build polygon data for planar faces (used for trimming).
        let normal = match face.surface() {
            FaceSurface::Plane { normal, .. } => *normal,
            _ => continue,
        };

        face_polygons.insert(
            face_id.index(),
            FacePolygon {
                vertex_ids,
                positions,
                wire_edge_ids,
                normal,
                d: 0.0,
                inner_wires: face_inner_wires,
            },
        );
    }

    // Build a map from edge index to radius law for per-vertex radius lookup.
    // Each vertex adjacent to a filleted edge uses that edge's actual radius
    // at the vertex (start=0.0, end=1.0) instead of a global average.
    let edge_law_map: HashMap<usize, &FilletRadiusLaw> = edge_laws
        .iter()
        .map(|(eid, law)| (eid.index(), law))
        .collect();

    let mut vertex_fillet_edges: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for (edge_id, _) in edge_laws {
        let edge = topo.edge(*edge_id)?;
        vertex_fillet_edges
            .entry(edge.start().index())
            .or_default()
            .push(*edge_id);
        vertex_fillet_edges
            .entry(edge.end().index())
            .or_default()
            .push(*edge_id);
    }

    // Qualify every endpoint gap introduced by a declared setback. A smooth
    // three-way variable corner exists in this tranche when all cropped
    // stripes reach the same radius with zero endpoint slope and the signed
    // support planes admit one tangent ball. The ball then fixes each spine
    // station uniquely; caller distances are checked against those
    // projections instead of trusted.
    let mut qualified_setback_corners: HashMap<
        usize,
        (remus_topology::vertex::VertexId, Point3, f64, bool),
    > = HashMap::new();
    let mut selected_vertex_indices: Vec<_> = vertex_fillet_edges.keys().copied().collect();
    selected_vertex_indices.sort_unstable();
    for vertex_index in selected_vertex_indices {
        let incident_edges = &vertex_fillet_edges[&vertex_index];
        let declared_count = incident_edges
            .iter()
            .filter(|edge_id| {
                declared_endpoint_setbacks.contains_key(&(edge_id.index(), vertex_index))
            })
            .count();
        if declared_count == 0 {
            continue;
        }
        let vertex_id = incident_edges
            .iter()
            .find_map(|edge_id| {
                let edge = topo.edge(*edge_id).ok()?;
                [edge.start(), edge.end()]
                    .into_iter()
                    .find(|vertex| vertex.index() == vertex_index)
            })
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason: format!("setback endpoint {vertex_index} is not on its selected edges"),
            })?;
        if incident_edges.len() < 3 || declared_count != incident_edges.len() {
            return Err(crate::OperationsError::Blend(
                remus_blend::BlendError::UnsupportedSetbackCorner {
                    vertex: vertex_id,
                    stripes: incident_edges.len(),
                    reason: "every incident selected stripe must declare a positive setback at a 3+-way corner"
                        .into(),
                },
            ));
        }

        let mut radii = Vec::with_capacity(incident_edges.len());
        let mut planar_edge_sides = HashMap::with_capacity(incident_edges.len());
        for edge_id in incident_edges {
            let edge = topo.edge(*edge_id)?;
            let law_t = if edge.start().index() == vertex_index {
                0.0
            } else {
                1.0
            };
            let law = edge_law_map[&edge_id.index()];
            let radius = law.evaluate(law_t);
            radii.push((*edge_id, radius));
            let slope_tol = tol.linear.max(radius.abs() * 1.0e-10);
            if law.derivative(law_t).abs() > slope_tol {
                return Err(crate::OperationsError::Blend(
                    remus_blend::BlendError::UnsupportedSetbackCorner {
                        vertex: vertex_id,
                        stripes: incident_edges.len(),
                        reason: format!(
                            "edge {edge_id:?} radius law must have zero slope at the setback station for a G1 corner seam"
                        ),
                    },
                ));
            }

            let faces = edge_to_faces.get(&edge_id.index()).ok_or_else(|| {
                crate::OperationsError::Blend(remus_blend::BlendError::UnsupportedSetbackCorner {
                    vertex: vertex_id,
                    stripes: incident_edges.len(),
                    reason: format!("edge {edge_id:?} has no support-face pair"),
                })
            })?;
            let supports_are_planar = if faces.len() == 2 {
                let mut planar = true;
                for face_id in faces {
                    if topo.face(*face_id)?.effective_plane_normal().is_none() {
                        planar = false;
                        break;
                    }
                }
                planar
            } else {
                false
            };
            if !supports_are_planar {
                return Err(crate::OperationsError::Blend(
                    remus_blend::BlendError::UnsupportedSetbackCorner {
                        vertex: vertex_id,
                        stripes: incident_edges.len(),
                        reason:
                            "the qualified setback corner requires two planar supports per stripe"
                                .into(),
                    },
                ));
            }
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let probe = radius
                .min((end - start).length() * 0.1)
                .max(tol.linear * 10.0);
            let side = match crate::query::edge_concavity_from_faces(
                topo, solid, *edge_id, faces[0], faces[1], probe,
            )? {
                crate::query::EdgeConcavity::Convex => -1.0,
                crate::query::EdgeConcavity::Concave => 1.0,
                crate::query::EdgeConcavity::Tangent | crate::query::EdgeConcavity::Unknown => {
                    return Err(crate::OperationsError::Blend(
                        remus_blend::BlendError::UnsupportedSetbackCorner {
                            vertex: vertex_id,
                            stripes: incident_edges.len(),
                            reason: format!(
                                "edge {edge_id:?} has no qualified material-side orientation"
                            ),
                        },
                    ));
                }
            };
            planar_edge_sides.insert(edge_id.index(), side);
        }

        let common_radius = radii[0].1;
        let radius_tol = tol.linear.max(common_radius.abs() * 1.0e-8);
        if radii
            .iter()
            .any(|(_, radius)| (*radius - common_radius).abs() > radius_tol)
        {
            return Err(crate::OperationsError::Blend(
                remus_blend::BlendError::UnsupportedSetbackCorner {
                    vertex: vertex_id,
                    stripes: incident_edges.len(),
                    reason: "incident radius laws do not reach one common corner radius at the declared stations"
                        .into(),
                },
            ));
        }
        let Some(center) = rolling_ball::exact_planar_corner_ball(
            topo,
            vertex_index,
            incident_edges,
            &edge_to_faces,
            &planar_edge_sides,
            common_radius,
            tol,
        ) else {
            return Err(crate::OperationsError::Blend(
                remus_blend::BlendError::UnsupportedSetbackCorner {
                    vertex: vertex_id,
                    stripes: incident_edges.len(),
                    reason:
                        "signed support planes do not admit one consistently oriented tangent ball"
                            .into(),
                },
            ));
        };

        let vertex = topo.vertex(vertex_id)?.point();
        for (edge_id, _) in &radii {
            let edge = topo.edge(*edge_id)?;
            let other = if edge.start() == vertex_id {
                topo.vertex(edge.end())?.point()
            } else {
                topo.vertex(edge.start())?.point()
            };
            let away = (other - vertex).normalize()?;
            let required = (center - vertex).dot(away);
            let declared = declared_endpoint_setbacks[&(edge_id.index(), vertex_index)];
            let setback_tol = tol.linear.max(required.abs() * 1.0e-8);
            if required <= tol.linear || (declared - required).abs() > setback_tol {
                return Err(crate::OperationsError::Blend(
                    remus_blend::BlendError::SetbackMismatch {
                        edge: *edge_id,
                        vertex: vertex_id,
                        declared,
                        required,
                    },
                ));
            }
        }
        let is_concave = planar_edge_sides.values().all(|side| *side > 0.0);
        qualified_setback_corners
            .insert(vertex_index, (vertex_id, center, common_radius, is_concave));
    }

    // Shared contact map: the SAME inward contact point used both to trim the
    // adjacent faces and to anchor the blend boundary, keyed by
    // (vertex_index, edge_index, face_index). Computing it once guarantees the
    // trimmed face boundary and the blend boundary coincide (watertight shell).
    // Geometry uses the active (possibly setback) station; the normalized law
    // still uses 0/1 at the built stripe endpoints.
    let fillet_contact_map: HashMap<(usize, usize, usize), Point3> = {
        let mut map = HashMap::new();
        for (edge_id, law) in edge_laws {
            let edge = topo.edge(*edge_id)?;
            let p_start = topo.vertex(edge.start())?.point();
            let p_end = topo.vertex(edge.end())?.point();
            let (t_start, t_end) = active_intervals[&edge_id.index()];

            let Some(face_list) = edge_to_faces.get(&edge_id.index()) else {
                continue;
            };
            if face_list.len() < 2 {
                continue;
            }
            let f1 = face_list[0];
            let f2 = face_list[1];

            let (Some(surf1), Some(surf2)) = (
                face_surfaces.get(&f1.index()),
                face_surfaces.get(&f2.index()),
            ) else {
                continue;
            };

            let edge_curve = edge.curve().clone();
            if geometry::sample_edge_tangent(&edge_curve, p_start, p_end, t_start).length()
                < tol.linear
            {
                continue;
            }

            for &(geometry_t, law_t, vid) in
                &[(t_start, 0.0, edge.start()), (t_end, 1.0, edge.end())]
            {
                let r = law.evaluate(law_t);
                let p = geometry::sample_edge_point(&edge_curve, p_start, p_end, geometry_t);
                let tan = geometry::sample_edge_tangent(&edge_curve, p_start, p_end, geometry_t);
                let Ok(local_dir) = tan.normalize() else {
                    continue;
                };
                let (Some(n1), Some(n2)) = (
                    face_surface_normal_at(surf1, p),
                    face_surface_normal_at(surf2, p),
                ) else {
                    continue;
                };
                if let Some(&(_, center, _, _)) = qualified_setback_corners.get(&vid.index()) {
                    let project_to_plane = |surface: &FaceSurface| -> Option<Point3> {
                        let FaceSurface::Plane { normal, d } = surface else {
                            return None;
                        };
                        let denominator = normal.dot(*normal);
                        (denominator > tol.linear * tol.linear).then(|| {
                            center
                                - *normal * ((dot_normal_point(*normal, center) - *d) / denominator)
                        })
                    };
                    if let (Some(contact1), Some(contact2)) =
                        (project_to_plane(surf1), project_to_plane(surf2))
                    {
                        map.insert((vid.index(), edge_id.index(), f1.index()), contact1);
                        map.insert((vid.index(), edge_id.index(), f2.index()), contact2);
                        continue;
                    }
                }
                let cs = geometry::cross_section_dirs(local_dir, n1, n2, local_dir, local_dir);
                map.insert((vid.index(), edge_id.index(), f1.index()), p + cs.ld1 * r);
                map.insert((vid.index(), edge_id.index(), f2.index()), p + cs.ld2 * r);
            }
        }
        map
    };

    // Vertices at endpoints of filleted edges. A side face (one that shares
    // such a vertex but whose own edges are not filleted) must split that
    // corner into the two blend contact points, or the blend boundary is left
    // unmatched and the shell becomes non-manifold.
    let mut vertex_fillet_endpoints: HashSet<usize> = HashSet::new();
    for (edge_id, _) in edge_laws {
        let edge = topo.edge(*edge_id)?;
        vertex_fillet_endpoints.insert(edge.start().index());
        vertex_fillet_endpoints.insert(edge.end().index());
    }

    // Trim planar faces by replacing each filleted-edge boundary vertex with
    // the shared contact point. The NURBS canal surface replaces the fillet face.
    let mut all_specs: Vec<FaceSpec> = Vec::new();

    for &face_id in &shell_face_ids {
        let Some(poly) = face_polygons.get(&face_id.index()) else {
            let face = topo.face(face_id)?;
            let verts = crate::boolean::face_polygon(topo, face_id)?;
            let np_inner = extract_inner_wire_positions(topo, face)?;
            all_specs.push(FaceSpec::Surface {
                vertices: verts,
                surface: face.surface().clone(),
                reversed: false,
                inner_wires: np_inner,
            });
            continue;
        };
        let n = poly.positions.len();

        // Skip polygon trimming for degenerate faces (e.g., disc caps).
        if n < 3 {
            all_specs.push(FaceSpec::Planar {
                vertices: poly.positions.clone(),
                normal: poly.normal,
                d: poly.d,
                inner_wires: poly.inner_wires.clone(),
            });
            continue;
        }

        let mut new_verts: Vec<Point3> = Vec::with_capacity(n + target_set.len());
        let fi = face_id.index();

        for i in 0..n {
            let prev_i = if i == 0 { n - 1 } else { i - 1 };
            let next_i = (i + 1) % n;
            let before_filleted = target_set.contains(&poly.wire_edge_ids[prev_i].index());
            let after_filleted = target_set.contains(&poly.wire_edge_ids[i].index());
            let pos = poly.positions[i];
            let prev_pos = poly.positions[prev_i];
            let next_pos = poly.positions[next_i];
            let vi = poly.vertex_ids[i].index();
            let at_fillet_endpoint = vertex_fillet_endpoints.contains(&vi);

            match (before_filleted, after_filleted, at_fillet_endpoint) {
                (false, false, false) => new_verts.push(pos),
                // Side face: vertex sits at a filleted-edge endpoint but neither
                // of this face's edges is filleted. Split the corner into the two
                // blend contacts at this vertex (one per filleted-adjacent face),
                // ordered toward prev/next to keep the wire convex.
                (false, false, true) => {
                    let mut unique_contacts: Vec<Point3> = Vec::new();
                    for (&(vi_k, _, _), &pt) in &fillet_contact_map {
                        if vi_k == vi
                            && !unique_contacts
                                .iter()
                                .any(|uc| (*uc - pt).length() < tol.linear)
                        {
                            unique_contacts.push(pt);
                        }
                    }
                    if unique_contacts.len() >= 2 {
                        let approx_prev = (prev_pos - pos)
                            .normalize()
                            .map_or(pos, |d| pos + d * tol.linear);
                        let d0 = (unique_contacts[0] - approx_prev).length();
                        let d1 = (unique_contacts[1] - approx_prev).length();
                        if d0 <= d1 {
                            new_verts.push(unique_contacts[0]);
                            new_verts.push(unique_contacts[1]);
                        } else {
                            new_verts.push(unique_contacts[1]);
                            new_verts.push(unique_contacts[0]);
                        }
                    } else {
                        new_verts.push(pos);
                    }
                }
                (true, false, _) => {
                    let ei = poly.wire_edge_ids[prev_i].index();
                    if let Some(&pt) = fillet_contact_map.get(&(vi, ei, fi)) {
                        new_verts.push(pt);
                    } else {
                        let dir = (next_pos - pos).normalize()?;
                        new_verts.push(pos + dir * edge_law_map[&ei].evaluate(1.0));
                    }
                }
                (false, true, _) => {
                    let ei = poly.wire_edge_ids[i].index();
                    if let Some(&pt) = fillet_contact_map.get(&(vi, ei, fi)) {
                        new_verts.push(pt);
                    } else {
                        let dir = (prev_pos - pos).normalize()?;
                        new_verts.push(pos + dir * edge_law_map[&ei].evaluate(0.0));
                    }
                }
                (true, true, _) => {
                    let ei_after = poly.wire_edge_ids[i].index();
                    if let Some(&pt) = fillet_contact_map.get(&(vi, ei_after, fi)) {
                        new_verts.push(pt);
                    } else {
                        let dir_prev = (prev_pos - pos).normalize()?;
                        new_verts.push(pos + dir_prev * edge_law_map[&ei_after].evaluate(0.0));
                    }
                    let ei_before = poly.wire_edge_ids[prev_i].index();
                    if let Some(&pt) = fillet_contact_map.get(&(vi, ei_before, fi)) {
                        new_verts.push(pt);
                    } else {
                        let dir_next = (next_pos - pos).normalize()?;
                        new_verts.push(pos + dir_next * edge_law_map[&ei_before].evaluate(1.0));
                    }
                }
            }
        }

        let new_d = dot_normal_point(poly.normal, new_verts[0]);
        all_specs.push(FaceSpec::Planar {
            vertices: new_verts,
            normal: poly.normal,
            d: new_d,
            inner_wires: poly.inner_wires.clone(),
        });
    }

    let n_samples = 5; // Number of cross-sections along each edge
    // Every requested edge must end up carrying a blend surface; the loop's
    // early `continue`s are where a silent subset used to come from.
    let mut blended_edges: HashSet<usize> = HashSet::new();

    for (edge_id, law) in edge_laws {
        let edge = topo.edge(*edge_id)?;
        let p_start = topo.vertex(edge.start())?.point();
        let p_end = topo.vertex(edge.end())?.point();
        let (t_start, t_end) = active_intervals[&edge_id.index()];
        let active_start = geometry::sample_edge_point(edge.curve(), p_start, p_end, t_start);

        let Some(face_list) = edge_to_faces.get(&edge_id.index()) else {
            continue;
        };
        if face_list.len() < 2 {
            continue;
        }
        let f1 = face_list[0];
        let f2 = face_list[1];

        // Get face surfaces for normal evaluation on curved faces.
        let (Some(surf1), Some(surf2)) = (
            face_surfaces.get(&f1.index()),
            face_surfaces.get(&f2.index()),
        ) else {
            continue;
        };

        let Some(n1_start) = face_surface_normal_at(surf1, active_start) else {
            continue;
        };
        let Some(n2_start) = face_surface_normal_at(surf2, active_start) else {
            continue;
        };

        let edge_curve = edge.curve().clone();

        let edge_tan = geometry::sample_edge_tangent(&edge_curve, p_start, p_end, t_start);
        if edge_tan.length() < tol.linear {
            continue;
        }
        let edge_dir = edge_tan.normalize()?;

        // Reference cross-section at t=0 for fallback directions.
        let cs_ref = geometry::cross_section_dirs(edge_dir, n1_start, n2_start, edge_dir, edge_dir);
        let d1_ref = cs_ref.ld1;
        let d2_ref = cs_ref.ld2;

        if cs_ref.half_angle.abs() < tol.angular {
            continue;
        }

        // Use more samples for curved faces or curved edges.
        let both_planar = matches!(surf1, FaceSurface::Plane { .. })
            && matches!(surf2, FaceSurface::Plane { .. });
        let n_v = if both_planar {
            geometry::edge_v_samples(&edge_curve).max(n_samples)
        } else {
            geometry::edge_v_samples(&edge_curve).max(n_samples).max(7)
        };

        // Build interpolation grid: n_v rows × 3 columns (arc CPs).
        let mut grid: Vec<Vec<Point3>> = Vec::with_capacity(n_v);
        let mut sample_weights: Vec<f64> = Vec::with_capacity(n_v);

        #[allow(clippy::cast_precision_loss)]
        for s in 0..n_v {
            let fraction = s as f64 / (n_v - 1).max(1) as f64;
            let geometry_t = (t_end - t_start).mul_add(fraction, t_start);
            let r = law.evaluate(fraction);
            let p = geometry::sample_edge_point(&edge_curve, p_start, p_end, geometry_t);
            let tan = geometry::sample_edge_tangent(&edge_curve, p_start, p_end, geometry_t);
            let local_dir = tan.normalize().unwrap_or(edge_dir);

            let ln1 = face_surface_normal_at(surf1, p).unwrap_or(n1_start);
            let ln2 = face_surface_normal_at(surf2, p).unwrap_or(n2_start);

            let cs = geometry::cross_section_dirs(local_dir, ln1, ln2, d1_ref, d2_ref);

            // cos(φ/2) is the rational-quadratic arc weight; clamp to a positive
            // floor so nearly-coplanar faces (φ/2 → π/2) don't yield a zero
            // weight (degenerate control point).
            let w = cs.half_angle.cos().max(0.01);
            let contact1 = p + cs.ld1 * r;
            let contact2 = p + cs.ld2 * r;
            // The middle control point is the apex of the tangent cone — the
            // intersection of the two contact tangents. For a rolling ball on
            // surfaces meeting at the edge this is the edge point itself, so the
            // weighted arc bulges concavely toward the solid interior (cutting
            // material). Placing it on the bisector ray past the ball center
            // would bulge the blend outward and add volume.
            let mid_cp = p;

            sample_weights.push(w);
            grid.push(vec![contact1, mid_cp, contact2]);
        }

        // Anchor the blend boundary contacts to the shared contact map so the
        // interpolated NURBS boundary coincides exactly with the trimmed-face
        // vertices (bitwise-identical, no duplicate vertices in assembly).
        let v_start = edge.start().index();
        let v_end = edge.end().index();
        if let Some(&pt) = fillet_contact_map.get(&(v_start, edge_id.index(), f1.index())) {
            grid[0][0] = pt;
        }
        if let Some(&pt) = fillet_contact_map.get(&(v_start, edge_id.index(), f2.index())) {
            grid[0][2] = pt;
        }
        if let Some(&pt) = fillet_contact_map.get(&(v_end, edge_id.index(), f1.index())) {
            grid[n_v - 1][0] = pt;
        }
        if let Some(&pt) = fillet_contact_map.get(&(v_end, edge_id.index(), f2.index())) {
            grid[n_v - 1][2] = pt;
        }

        // Build a rational NURBS surface with exact circular arc cross-sections.
        // On a straight spine between planar supports, every row is an exact
        // cubic Hermite curve: this preserves the zero endpoint derivative of
        // an S-curve law, which is what makes a setback sphere seam truly G1.
        // The general curved-support path retains sampled interpolation.
        let row_contact1: Vec<Point3> = (0..n_v).map(|i| grid[i][0]).collect();
        let row_mid: Vec<Point3> = (0..n_v).map(|i| grid[i][1]).collect();
        let row_contact2: Vec<Point3> = (0..n_v).map(|i| grid[i][2]).collect();
        let (degree_v, knots_v, control_points, mid_weights) =
            if both_planar && matches!(edge_curve, EdgeCurve::Line) {
                let active_vector = (p_end - p_start) * (t_end - t_start);
                let start_slope = law.derivative(0.0);
                let end_slope = law.derivative(1.0);
                let controls = |start: Point3, end: Point3, direction: Vec3| {
                    vec![
                        start,
                        start + (active_vector + direction * start_slope) * (1.0 / 3.0),
                        end - (active_vector + direction * end_slope) * (1.0 / 3.0),
                        end,
                    ]
                };
                (
                    3,
                    vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                    vec![
                        controls(row_contact1[0], row_contact1[n_v - 1], d1_ref),
                        controls(row_mid[0], row_mid[n_v - 1], Vec3::new(0.0, 0.0, 0.0)),
                        controls(row_contact2[0], row_contact2[n_v - 1], d2_ref),
                    ],
                    vec![sample_weights[0]; 4],
                )
            } else {
                let degree = (n_v - 1).min(3);
                let crv0 = remus_math::nurbs::fitting::interpolate(&row_contact1, degree)
                    .map_err(crate::OperationsError::Math)?;
                let crv1 = remus_math::nurbs::fitting::interpolate(&row_mid, degree)
                    .map_err(crate::OperationsError::Math)?;
                let crv2 = remus_math::nurbs::fitting::interpolate(&row_contact2, degree)
                    .map_err(crate::OperationsError::Math)?;
                let n_cp = crv0.control_points().len();
                #[allow(clippy::cast_precision_loss)]
                let weights = if n_cp == sample_weights.len() {
                    sample_weights.clone()
                } else {
                    (0..n_cp)
                        .map(|i| {
                            let t = i as f64 / (n_cp - 1).max(1) as f64;
                            let idx_f = t * (sample_weights.len() - 1).max(1) as f64;
                            let lo = (idx_f.floor() as usize).min(sample_weights.len() - 1);
                            let hi = (lo + 1).min(sample_weights.len() - 1);
                            let frac = idx_f - lo as f64;
                            sample_weights[lo] * (1.0 - frac) + sample_weights[hi] * frac
                        })
                        .collect()
                };
                (
                    crv0.degree(),
                    crv0.knots().to_vec(),
                    vec![
                        crv0.control_points().to_vec(),
                        crv1.control_points().to_vec(),
                        crv2.control_points().to_vec(),
                    ],
                    weights,
                )
            };
        let n_cp_v = control_points[0].len();

        let surface = remus_math::nurbs::surface::NurbsSurface::new(
            2, // degree_u (circular arc)
            degree_v,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], // knots_u
            knots_v,
            control_points,
            vec![vec![1.0; n_cp_v], mid_weights, vec![1.0; n_cp_v]],
        )
        .map_err(crate::OperationsError::Math)?;

        let c1s = grid[0][0];
        let c2s = grid[0][2];
        let c1e = grid[n_v - 1][0];
        let c2e = grid[n_v - 1][2];

        // Reverse the stored surface when its parametric mid-normal points
        // into the dihedral. Keeping the decision on the spec is stable even
        // when analytic cap specs are assembled ahead of ordinary surfaces.
        let srf_mid_normal = surface.normal(0.5, 0.5).unwrap_or(cs_ref.bisector);
        let reversed = srf_mid_normal.dot(cs_ref.bisector) > 0.0;
        all_specs.push(FaceSpec::Surface {
            vertices: vec![c1s, c2s, c2e, c1e],
            surface: FaceSurface::Nurbs(surface),
            reversed,
            inner_wires: vec![],
        });
        blended_edges.insert(edge_id.index());
    }

    // Close each qualified setback junction with the exact common tangent
    // ball. Contacts are deduplicated because each support face contributes
    // the same tangency point through two incident stripes, then ordered
    // counter-clockwise about the cap's outward radial direction.
    let mut corner_indices: Vec<_> = qualified_setback_corners.keys().copied().collect();
    corner_indices.sort_unstable();
    for vertex_index in corner_indices {
        let (vertex, center, radius, is_concave) = qualified_setback_corners[&vertex_index];
        let mut contacts = Vec::new();
        let contact_tol = tol.linear.max(radius * 1.0e-9);
        for (&(contact_vertex, _, _), &point) in &fillet_contact_map {
            if contact_vertex == vertex_index
                && !contacts
                    .iter()
                    .any(|known: &Point3| (*known - point).length() <= contact_tol)
            {
                contacts.push(point);
            }
        }
        let incident_count = vertex_fillet_edges[&vertex_index].len();
        if contacts.len() < 3 {
            return Err(crate::OperationsError::Blend(
                remus_blend::BlendError::UnsupportedSetbackCorner {
                    vertex,
                    stripes: incident_count,
                    reason: format!(
                        "the common tangent ball produced only {} distinct contacts",
                        contacts.len()
                    ),
                },
            ));
        }

        let radial_sum = contacts
            .iter()
            .map(|point| *point - center)
            .fold(Vec3::new(0.0, 0.0, 0.0), |sum, radial| sum + radial);
        let cap_normal = radial_sum.normalize().map_err(|_| {
            crate::OperationsError::Blend(remus_blend::BlendError::UnsupportedSetbackCorner {
                vertex,
                stripes: incident_count,
                reason: "the common tangent ball contacts have no stable cap direction".into(),
            })
        })?;
        let frame = Frame3::from_normal(center, cap_normal).map_err(|_| {
            crate::OperationsError::InvalidInput {
                reason: format!("setback cap at vertex {vertex_index} has no stable frame"),
            }
        })?;
        contacts.sort_by(|a, b| {
            let radial_a = *a - center;
            let radial_b = *b - center;
            let angle_a = radial_a.dot(frame.y).atan2(radial_a.dot(frame.x));
            let angle_b = radial_b.dot(frame.y).atan2(radial_b.dot(frame.x));
            angle_a
                .partial_cmp(&angle_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let cap = rolling_ball::build_sphere_cap(&contacts, center, is_concave).ok_or_else(|| {
            crate::OperationsError::Unsupported {
                operation: "fillet_variable_with_setbacks",
                reason: format!(
                    "qualified tangent ball at vertex {vertex_index} did not produce an exact spherical cap"
                ),
            }
        })?;
        all_specs.push(cap);
    }

    // Coverage check before assembly: a requested edge that every `continue`
    // above skipped would otherwise vanish from the result without a word —
    // the caller would get a closed, valid solid that simply lacks the blend
    // it named (the silent no-op in disguise). Name those edges instead.
    let missing: Vec<EdgeId> = edge_laws
        .iter()
        .map(|(edge_id, _)| *edge_id)
        .filter(|edge_id| !blended_edges.contains(&edge_id.index()))
        .collect();
    if !missing.is_empty() {
        return Err(crate::OperationsError::Blend(
            remus_blend::BlendError::EdgesNotBlended {
                edges: missing,
                reason: "the variable-radius engine produced no blend surface for them \
                         (foreign or non-manifold edge, unreadable surface normal, or a \
                         dihedral too flat to round)"
                    .into(),
            },
        ));
    }

    let solid_id = crate::boolean::assemble_solid_mixed(topo, &all_specs, tol)?;

    // Fail closed on a plausible-but-wrong result: no new validation errors
    // against the input baseline, and the volume change must be one a blend
    // of this size can physically produce — an oversized radius used to
    // "succeed" here with the volume GROWING (r=50 on a 10 mm box returned
    // 3242 mm³), and holed-plate selections returned invalid shells as Ok.
    crate::blend_ops::validate_blend_solid_against_input(topo, "fillet", solid, solid_id)?;
    let (min_radius, max_radius) = edge_laws.iter().fold(
        (f64::INFINITY, 0.0_f64),
        |(min_radius, max_radius), (_, law)| {
            let (law_min, law_max) = law.bounds();
            (min_radius.min(law_min), max_radius.max(law_max))
        },
    );
    let edges: Vec<EdgeId> = edge_laws.iter().map(|(edge_id, _)| *edge_id).collect();
    crate::blend_ops::validate_blend_volume(
        topo,
        "fillet",
        solid,
        solid_id,
        &edges,
        crate::blend_ops::BlendSize::VariableFillet {
            min_radius,
            max_radius,
        },
    )?;
    Ok(solid_id)
}
