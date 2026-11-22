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

pub(crate) mod arc_torus;
mod geometry;
mod helpers;
mod mitre;
pub(crate) mod notch_torus;
mod oblique_arc;
#[cfg(test)]
mod retained_support_tests;
mod rolling_ball;
#[cfg(test)]
mod tests;
mod vertex_snap;

pub(crate) use crate::tessellate::surface_normal::face_surface_normal_at;
pub use remus_blend::radius_law::StandardRadiusLaw as FilletRadiusLaw;
pub use remus_blend::radius_law::StationBlend as FilletStationBlend;
#[allow(deprecated)]
pub use rolling_ball::fillet_rolling_ball;
pub(crate) use rolling_ball::fillet_rolling_ball_with_origins;

use remus_math::compact_sort::CompactSort;
use remus_math::det_hash::{DetHashMap as HashMap, DetHashSet as HashSet, DetState};

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
    crate::blend_ops::preview_checks::transaction(topo, |t| {
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

    if !radius.is_finite() || radius <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("fillet radius must be finite and positive, got {radius}"),
        });
    }
    if edges.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no edges specified for fillet".into(),
        });
    }
    crate::refuse_cavity_shells(topo, solid, "fillet")?;

    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let shell_face_ids: Vec<_> = shell.faces().to_vec();

    let mut edge_to_faces: HashMap<usize, Vec<_>> = HashMap::default();
    let mut face_polygons: HashMap<usize, FacePolygon> = HashMap::default();

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
        let Some(normal) = face.effective_plane_normal() else {
            continue;
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
                reversed: face.is_reversed(),
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
    let mut vertex_fillet_endpoints: HashSet<usize> = HashSet::default();
    for &edge_id in &filtered_edges {
        let edge = topo.edge(edge_id)?;
        vertex_fillet_endpoints.insert(edge.start().index());
        vertex_fillet_endpoints.insert(edge.end().index());
    }

    // Strategy: identical to chamfer but with more offset segments to
    // approximate the circular fillet.
    let mut fillet_data: HashMap<usize, FilletEdgeData> = HashMap::default();
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
        result_specs.push(poly.spec(new_verts, new_d));
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
    let mut baseline = crate::blend_ops::BlendBaseline::new(topo, solid);
    crate::blend_ops::validate_blend_solid_against_input(&mut baseline, "fillet", result)?;
    crate::blend_ops::validate_blend_volume(
        &mut baseline,
        "fillet",
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
    crate::blend_ops::preview_checks::transaction(topo, |t| {
        fillet_variable_transacted(
            t,
            solid,
            edge_laws,
            &HashMap::default(),
            VariableCorners::Declared,
        )
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
    fillet_variable_with_corners(topo, solid, specs, VariableCorners::Declared)
}

/// How a variable-radius fillet closes a corner where three or more selected
/// stripes meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VariableCorners {
    /// Only where every incident stripe declares its setback, and each
    /// declared distance must be the tangent ball's station
    /// ([`fillet_variable_with_setbacks`]); an undeclared corner is refused.
    Declared,
    /// At every undeclared corner of three or more selected stripes on
    /// planar supports, set each stripe back to the station of the one ball
    /// tangent to all their supports, and close the corner with that ball's
    /// exact spherical cap. The qualification is the declared one's: the
    /// stripes' radii must agree at the corner with zero slope there, or the
    /// corner is refused by type.
    TangentBall,
}

/// [`fillet_variable_with_setbacks`] with an explicit corner mode.
///
/// # Errors
///
/// As [`fillet_variable_with_setbacks`]; with
/// [`VariableCorners::TangentBall`], also `InvalidInput` when a spec declares
/// its own setback (the ball sets them), and `InvalidInput` when the ball's
/// stations would consume a stripe.
pub fn fillet_variable_with_corners(
    topo: &mut Topology,
    solid: SolidId,
    specs: &[FilletEdgeSetback],
    corners: VariableCorners,
) -> Result<SolidId, crate::OperationsError> {
    if corners == VariableCorners::TangentBall
        && specs
            .iter()
            .any(|spec| spec.start_setback != 0.0 || spec.end_setback != 0.0)
    {
        return Err(crate::OperationsError::InvalidInput {
            reason: "tangent-ball corners set every setback themselves; declare none".into(),
        });
    }
    let mut seen = HashSet::with_capacity_and_hasher(specs.len(), DetState);
    let mut edge_laws = Vec::with_capacity(specs.len());
    let mut setbacks = HashMap::with_capacity_and_hasher(specs.len(), DetState);
    for spec in specs {
        if seen.insert(spec.edge) {
            edge_laws.push((spec.edge, spec.law.clone()));
            setbacks.insert(spec.edge.index(), (spec.start_setback, spec.end_setback));
        }
    }
    crate::blend_ops::preview_checks::transaction(topo, |t| {
        fillet_variable_transacted(t, solid, &edge_laws, &setbacks, corners)
    })
}

/// Refuse a selected edge of `solid` whose spine is not straight, before any
/// spine-parameter bookkeeping (a closed rim would otherwise read as a
/// collapsed edge). Edges of other solids are left to the engine's
/// `EdgesNotBlended` refusal.
///
/// # Errors
///
/// [`remus_blend::BlendError::UnsupportedVariableSupport`] naming the edge
/// and its curve.
fn reject_curved_variable_spines(
    topo: &Topology,
    solid: SolidId,
    edge_laws: &[(EdgeId, FilletRadiusLaw)],
) -> Result<(), crate::OperationsError> {
    let own: HashSet<EdgeId> = remus_topology::explorer::solid_edges(topo, solid)?
        .into_iter()
        .collect();
    for (edge_id, _) in edge_laws {
        let curve = topo.edge(*edge_id)?.curve();
        if own.contains(edge_id) && !matches!(curve, EdgeCurve::Line) {
            return Err(crate::OperationsError::Blend(
                remus_blend::BlendError::UnsupportedVariableSupport {
                    edge: *edge_id,
                    reason: format!(
                        "its spine is a {}; an exact variable section needs a straight spine",
                        edge_curve_tag(curve)
                    ),
                },
            ));
        }
    }
    Ok(())
}

/// Refuse every selected stripe outside the family whose section this engine
/// builds exactly: a straight spine (checked first by
/// [`reject_curved_variable_spines`]) between two planar supports.
///
/// There each section is a rational circular arc in a fixed plane frame (see
/// [`VariableSection`]) and each row is the radius law itself, so the band is
/// exact. Concave edges are refused here as well: their section is not yet
/// qualified. A curved spine or
/// a curved support would need sampled sections interpolated into a NURBS
/// band — an approximation that can still assemble and validate — so it is
/// refused by type before any topology is allocated.
///
/// # Errors
///
/// [`remus_blend::BlendError::UnsupportedVariableSupport`] naming the first
/// stripe outside the family and what takes it out.
fn reject_inexact_variable_stripes(
    topo: &Topology,
    solid: SolidId,
    edge_laws: &[(EdgeId, FilletRadiusLaw)],
    edge_to_faces: &HashMap<usize, Vec<remus_topology::face::FaceId>>,
    face_surfaces: &HashMap<usize, FaceSurface>,
    tol: Tolerance,
) -> Result<HashMap<usize, VariableSection>, crate::OperationsError> {
    let refuse = |edge: EdgeId, reason: String| {
        Err(crate::OperationsError::Blend(
            remus_blend::BlendError::UnsupportedVariableSupport { edge, reason },
        ))
    };
    let mut sections = HashMap::with_capacity_and_hasher(edge_laws.len(), DetState);
    for (edge_id, law) in edge_laws {
        let faces = edge_to_faces
            .get(&edge_id.index())
            .map_or(&[][..], Vec::as_slice);
        if faces.is_empty() {
            // Not an edge of this solid: the engine names it in its
            // `EdgesNotBlended` refusal.
            continue;
        }
        if faces.len() != 2 || faces[0] == faces[1] {
            return refuse(
                *edge_id,
                "it does not separate two distinct support faces".into(),
            );
        }
        let mut normals = [Vec3::new(0.0, 0.0, 0.0); 2];
        for (slot, face) in faces.iter().enumerate() {
            match face_surfaces.get(&face.index()) {
                Some(FaceSurface::Plane { normal, .. }) => normals[slot] = *normal,
                other => {
                    return refuse(
                        *edge_id,
                        format!(
                            "support face {face:?} is {}; an exact variable section needs planar supports",
                            other.map_or("missing", face_surface_tag)
                        ),
                    );
                }
            }
        }
        let edge = topo.edge(*edge_id)?;
        let length =
            (topo.vertex(edge.end())?.point() - topo.vertex(edge.start())?.point()).length();
        // The side comes from the loops, exactly: each face's interior lies
        // to the left of its loops' effective direction about its outward
        // normal. The probe classifier only cross-checks it: a feature
        // thinner than its probe (a lip it steps over) makes the two disagree,
        // which refuses, but never decides the side. The probe is local, as
        // a probe as long as the radius can step past a narrow support (a
        // 1.5-wide groove lip under a 2.0 request) before the overflow check
        // gets to name that support.
        let from_loops = loop_convexity(topo, *edge_id, [faces[0], faces[1]], normals, tol)?;
        let probe = (law.bounds().0 * 0.1)
            .min(length * 0.01)
            .max(tol.linear * 10.0);
        let probed = crate::query::edge_concavity_from_faces(
            topo, solid, *edge_id, faces[0], faces[1], probe,
        )?;
        match (from_loops, probed) {
            (Some(true), crate::query::EdgeConcavity::Convex) => {}
            (Some(false), crate::query::EdgeConcavity::Concave) => {
                return refuse(
                    *edge_id,
                    "it is a concave edge; the exact variable section is qualified on convex edges"
                        .into(),
                );
            }
            _ => {
                return refuse(
                    *edge_id,
                    "its supports have no qualified material side".into(),
                );
            }
        }
        let Some(section) = VariableSection::convex(normals[0], normals[1], tol) else {
            return refuse(*edge_id, "its supports are parallel".into());
        };
        sections.insert(edge_id.index(), section);
    }
    Ok(sections)
}

/// Whether a straight edge between two planar faces is convex, from the
/// faces' loops: `Some(true)` convex, `Some(false)` concave, `None` when the
/// two faces disagree or the edge is degenerate.
///
/// A face's interior lies to the left of each of its loops' effective
/// direction about its outward normal (`is_forward` flipped once more for a
/// reversed face, as shell orientation counts it). That left direction runs
/// along the face away from the edge; the edge is convex exactly when it
/// points against the other face's outward normal, and both faces must say
/// the same.
fn loop_convexity(
    topo: &Topology,
    edge_id: EdgeId,
    faces: [remus_topology::face::FaceId; 2],
    outward: [Vec3; 2],
    tol: Tolerance,
) -> Result<Option<bool>, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let along = topo.vertex(edge.end())?.point() - topo.vertex(edge.start())?.point();
    let mut verdicts = [None, None];
    for (slot, &face_id) in faces.iter().enumerate() {
        let face = topo.face(face_id)?;
        let mut forward = None;
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            if let Some(oriented) = topo
                .wire(wire)?
                .edges()
                .iter()
                .find(|oriented| oriented.edge() == edge_id)
            {
                forward = Some(oriented.is_forward() != face.is_reversed());
                break;
            }
        }
        let Some(forward) = forward else {
            return Ok(None);
        };
        let direction = if forward { along } else { -along };
        let into_face = outward[slot].cross(direction);
        let other = outward[1 - slot];
        let side = into_face.dot(other);
        let scale = into_face.length() * other.length();
        if scale.is_nan() || scale <= 0.0 || side.abs() <= tol.angular * scale {
            return Ok(None);
        }
        verdicts[slot] = Some(side < 0.0);
    }
    Ok(match verdicts {
        [Some(a), Some(b)] if a == b => Some(a),
        _ => None,
    })
}

/// The exact section of a convex variable stripe between two planes, per
/// unit radius.
///
/// On a straight edge between planes the tangent ball of radius `r` touches
/// the supports at `p + contact·r`: its centre lies on the dihedral's
/// bisector `r / sin(α/2)` from the edge, `α` the interior angle, and its
/// contacts are the centre's projections onto the planes, `r·cot(α/2)` along
/// each. The section through those contacts with the edge point as apex is a
/// rational quadratic circle of radius `r` with apex weight `sin(α/2)`. The
/// unit offsets come from the constant engine's exact rolling-ball section.
#[derive(Debug, Clone, Copy)]
struct VariableSection {
    /// Contact offset on the first support per unit radius.
    contact1: Vec3,
    /// Contact offset on the second support per unit radius.
    contact2: Vec3,
    /// Apex weight of the circular section, `sin(α/2)`.
    weight: f64,
}

impl VariableSection {
    fn convex(normal1: Vec3, normal2: Vec3, tol: Tolerance) -> Option<Self> {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let (center, contact1, contact2) =
            rolling_ball::planar_rolling_ball_section(origin, normal1, normal2, -1.0, 1.0, tol)?;
        let apex_distance = (center - origin).length();
        (apex_distance.is_finite() && apex_distance > 1.0).then(|| Self {
            contact1: contact1 - origin,
            contact2: contact2 - origin,
            weight: 1.0 / apex_distance,
        })
    }
}

/// Refuse a corner where two or more selected stripes meet without declared
/// setbacks, unless they all continue one straight line.
///
/// Each stripe ends in the plane normal to its own spine, so two stripes
/// meeting at an angle leave a wedge between their end sections that no
/// patch of this engine fills; the assembled shell would only fail its
/// postcondition later. Saying so up front names the vertex instead. A
/// qualified 3+-way corner declares its setbacks and is checked by the caller.
///
/// # Errors
///
/// [`remus_blend::BlendError::UnsupportedVertexBlend`] for the vertex.
fn reject_unset_stripe_corner(
    topo: &Topology,
    vertex_index: usize,
    incident_edges: &[EdgeId],
    edge_to_faces: &HashMap<usize, Vec<remus_topology::face::FaceId>>,
    face_surfaces: &HashMap<usize, FaceSurface>,
    tol: Tolerance,
) -> Result<(), crate::OperationsError> {
    if incident_edges.len() < 2 {
        return Ok(());
    }
    let mut vertex = None;
    let mut directions = Vec::with_capacity(incident_edges.len());
    for edge_id in incident_edges {
        let edge = topo.edge(*edge_id)?;
        let (here, other) = if edge.start().index() == vertex_index {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        vertex = Some(here);
        let from = topo.vertex(here)?.point();
        let to = topo.vertex(other)?.point();
        directions.push((to - from).normalize()?);
    }
    let collinear = directions
        .iter()
        .all(|direction| direction.cross(directions[0]).length() <= tol.angular);
    // Two stripes continuing one line meet in one section plane, and their
    // end sections are the same arc only when their supports are the same
    // two planes: a support that turns there (a 90° wall going on as a 60°
    // one) leaves sections that differ, so the pair is refused as well.
    if collinear
        && incident_edges.len() == 2
        && same_support_planes(
            incident_edges[0],
            incident_edges[1],
            edge_to_faces,
            face_surfaces,
            tol,
        )
    {
        return Ok(());
    }
    let Some(vertex) = vertex else {
        return Ok(());
    };
    Err(crate::OperationsError::Blend(
        remus_blend::BlendError::UnsupportedVertexBlend {
            vertex,
            stripes: incident_edges.len(),
        },
    ))
}

/// Whether two stripes run between the same two planes (as oriented planes,
/// in either order), so their sections at a shared vertex coincide.
fn same_support_planes(
    first: EdgeId,
    second: EdgeId,
    edge_to_faces: &HashMap<usize, Vec<remus_topology::face::FaceId>>,
    face_surfaces: &HashMap<usize, FaceSurface>,
    tol: Tolerance,
) -> bool {
    let planes = |edge: EdgeId| -> Option<[(Vec3, f64); 2]> {
        let faces = edge_to_faces.get(&edge.index())?;
        if faces.len() != 2 {
            return None;
        }
        let plane = |face: &remus_topology::face::FaceId| {
            let Some(FaceSurface::Plane { normal, d }) = face_surfaces.get(&face.index()) else {
                return None;
            };
            let length = normal.length();
            (length > 0.0).then(|| (*normal * (1.0 / length), *d / length))
        };
        Some([plane(&faces[0])?, plane(&faces[1])?])
    };
    let same = |a: (Vec3, f64), b: (Vec3, f64)| {
        (a.0 - b.0).length() <= tol.angular && (a.1 - b.1).abs() <= tol.linear
    };
    let (Some([a0, a1]), Some([b0, b1])) = (planes(first), planes(second)) else {
        return false;
    };
    (same(a0, b0) && same(a1, b1)) || (same(a0, b1) && same(a1, b0))
}

fn edge_curve_tag(curve: &EdgeCurve) -> &'static str {
    match curve {
        EdgeCurve::Line => "line",
        EdgeCurve::Circle(_) => "circle",
        EdgeCurve::Ellipse(_) => "ellipse",
        EdgeCurve::Hyperbola(_) => "hyperbola",
        EdgeCurve::Parabola(_) => "parabola",
        EdgeCurve::NurbsCurve(_) => "NURBS curve",
    }
}

fn face_surface_tag(surface: &FaceSurface) -> &'static str {
    match surface {
        FaceSurface::Plane { .. } => "a plane",
        FaceSurface::Cylinder(_) => "a cylinder",
        FaceSurface::Cone(_) => "a cone",
        FaceSurface::Sphere(_) => "a sphere",
        FaceSurface::Torus(_) => "a torus",
        FaceSurface::Nurbs(_) => "a NURBS surface",
    }
}

/// Refuse a variable-radius stripe whose contact would land on or past the far
/// boundary of one of its planar support faces (B63).
///
/// This engine places each contact `r(t)` away from the spine along the
/// support face, perpendicular to the spine, and trims the face there. When
/// the face is not that wide the trim walks off it, yet the assembled shell
/// still closes and validates: a 10³ box with a radius-11 stripe used to
/// return a solid measuring 474 mm³. The rolling-ball and walking engines
/// refuse the same request with [`remus_blend::BlendError::CliffEncountered`],
/// and so does this one now, reporting the width the face actually has.
///
/// The check is exact for polygonal boundaries: in the support plane the
/// band's footprint lies under the contact curve `q = K·r(τ)`, a ray from the
/// spine meets each boundary segment at a height linear in τ, and the law is
/// a cubic on each piece, so the worst overrun is found at the interval ends
/// and the roots of a quadratic. Curved boundary edges are replaced by
/// regions that contain them (a lens per chord for a conic arc, the control
/// hull for a NURBS curve), so the check stays conservative there. Segments
/// parallel to the ray are skipped: the ray runs along them rather than out
/// through them, and the next loop edge is met at their far end.
///
/// # Errors
///
/// [`remus_blend::BlendError::CliffEncountered`] naming the edge, the
/// support face, and the worst station's requested radius and available
/// width.
/// Refuses a setback corner that another face reaches into: the corner's
/// cap is assembled against its stripes' support faces only, so any other
/// face inside the corner ball (a bore near the corner) would cross the cap
/// unsplit. Conservative: any face whose enclosing box meets the ball's
/// refuses the corner, NURBS faces by their carrier's whole control hull.
#[allow(clippy::too_many_arguments)]
fn reject_faces_in_corner_ball(
    topo: &Topology,
    solid: SolidId,
    vertex: remus_topology::vertex::VertexId,
    stripes: &[EdgeId],
    edge_to_faces: &HashMap<usize, Vec<remus_topology::face::FaceId>>,
    center: Point3,
    radius: f64,
    tol: Tolerance,
) -> Result<(), crate::OperationsError> {
    let supports: HashSet<usize> = stripes
        .iter()
        .filter_map(|edge| edge_to_faces.get(&edge.index()))
        .flatten()
        .map(|face| face.index())
        .collect();
    let reach = Vec3::new(radius, radius, radius);
    let ball = remus_math::aabb::Aabb3::from_points([center - reach, center + reach])
        .expanded(-tol.linear);
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        if supports.contains(&face.index()) {
            continue;
        }
        // A NURBS face is bounded by its whole carrier's control hull, which
        // encloses it whatever its trim (positive weights); the trimmed
        // patch box is recovered from sampled trims and can miss a lobe
        // (review). Analytic faces' boxes are exact.
        let bounds = match topo.face(face)?.surface() {
            FaceSurface::Nurbs(nurbs) => remus_math::aabb::Aabb3::try_from_points(
                nurbs.control_points().iter().flatten().copied(),
            )
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason: format!("face {face:?} has a NURBS surface with no control points"),
            })?,
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => crate::measure::face_bounding_box(topo, face)?,
        };
        if bounds.intersects(ball) {
            return Err(crate::OperationsError::Blend(
                remus_blend::BlendError::UnsupportedSetbackCorner {
                    vertex,
                    stripes: stripes.len(),
                    reason: format!(
                        "face {face:?} reaches into the corner's tangent ball, so the cap would cross it"
                    ),
                },
            ));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn reject_variable_support_cliffs(
    topo: &Topology,
    edge_laws: &[(EdgeId, FilletRadiusLaw)],
    sections: &HashMap<usize, VariableSection>,
    active_intervals: &HashMap<usize, (f64, f64)>,
    edge_to_faces: &HashMap<usize, Vec<remus_topology::face::FaceId>>,
    face_surfaces: &HashMap<usize, FaceSurface>,
    skip_edges: &HashSet<usize>,
    tol: Tolerance,
) -> Result<(), crate::OperationsError> {
    const CURVE_CHORDS: u32 = 32;

    for (edge_id, law) in edge_laws {
        let Some(faces) = edge_to_faces.get(&edge_id.index()) else {
            continue;
        };
        if faces.len() != 2 {
            continue;
        }
        let edge = topo.edge(*edge_id)?;
        let p_start = topo.vertex(edge.start())?.point();
        let p_end = topo.vertex(edge.end())?.point();
        let Some(&(t_start, t_end)) = active_intervals.get(&edge_id.index()) else {
            continue;
        };

        for (side, &face_id) in faces.iter().enumerate() {
            let Some(FaceSurface::Plane { normal, .. }) = face_surfaces.get(&face_id.index())
            else {
                continue;
            };
            let Ok(plane_normal) = normal.normalize() else {
                continue;
            };

            // Every boundary segment of the support except the stripe's own
            // edge, whichever loop it belongs to.
            let face = topo.face(face_id)?;
            let mut segments: Vec<(Point3, Point3)> = Vec::new();
            for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().to_vec()) {
                for oriented in topo.wire(wire_id)?.edges() {
                    if oriented.edge() == *edge_id || skip_edges.contains(&oriented.edge().index())
                    {
                        continue;
                    }
                    let boundary = topo.edge(oriented.edge())?;
                    let a = topo.vertex(boundary.start())?.point();
                    let b = topo.vertex(boundary.end())?.point();
                    if matches!(boundary.curve(), EdgeCurve::Line) {
                        segments.push((a, b));
                        continue;
                    }
                    if let EdgeCurve::NurbsCurve(nurbs) = boundary.curve() {
                        // The curve lies in its control points' convex hull,
                        // so a ray meets the hull no later than the curve.
                        segments.extend(planar_hull_segments(nurbs.control_points(), plane_normal));
                        continue;
                    }
                    // A circle or ellipse is replaced by the closed polygon
                    // circumscribed about the whole of it: an affine image
                    // of a regular polygon about a circle, so it contains the
                    // curve and every arc of it, and being closed, no ray can
                    // pass between its pieces. It over-reaches by at most
                    // 1/cos(π/n) − 1 of the semi-axis.
                    let (center, a_axis, b_axis) = match boundary.curve() {
                        EdgeCurve::Circle(circle) => (
                            circle.center(),
                            circle.u_axis() * circle.radius(),
                            circle.v_axis() * circle.radius(),
                        ),
                        EdgeCurve::Ellipse(ellipse) => (
                            ellipse.center(),
                            ellipse.u_axis() * ellipse.semi_major(),
                            ellipse.v_axis() * ellipse.semi_minor(),
                        ),
                        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                            return Err(crate::OperationsError::Blend(
                                remus_blend::BlendError::UnsupportedVariableSupport {
                                    edge: *edge_id,
                                    reason: format!(
                                        "support face {face_id:?} is bounded by a {} edge, whose clearance this check does not certify",
                                        edge_curve_tag(boundary.curve())
                                    ),
                                },
                            ));
                        }
                        EdgeCurve::Line | EdgeCurve::NurbsCurve(_) => continue,
                    };
                    let reach = 1.0 / (std::f64::consts::PI / f64::from(CURVE_CHORDS)).cos();
                    let corner = |k: u32| {
                        let angle = std::f64::consts::TAU * f64::from(k) / f64::from(CURVE_CHORDS);
                        center + (a_axis * angle.cos() + b_axis * angle.sin()) * reach
                    };
                    segments.extend((0..CURVE_CHORDS).map(|k| (corner(k), corner(k + 1))));
                }
            }

            // The band's footprint on this support is the region between
            // the spine and the contact curve `q = K·r(τ)`, in plane
            // coordinates `s` along the spine and `q` along the contact
            // direction. A ray from the spine along `q` meets each boundary
            // segment at a height linear in τ, and the law is a cubic on
            // each piece, so the worst overrun against each segment is the
            // maximum of a cubic: found exactly at the interval ends and the
            // roots of its derivative, not by sampling stations.
            let Some(section) = sections.get(&edge_id.index()) else {
                continue;
            };
            let offset = if side == 0 {
                section.contact1
            } else {
                section.contact2
            };
            let reach = offset.length();
            let Ok(across) = offset.normalize() else {
                continue;
            };
            let Ok(along) = (p_end - p_start).normalize() else {
                continue;
            };
            let length = (p_end - p_start).length();
            let (s0, s1) = (length * t_start, length * t_end);
            let pieces = law.segments();
            let mut worst: Option<(f64, f64)> = None;
            for &(sa, sb) in &segments {
                let (a_s, a_q) = ((sa - p_start).dot(along), (sa - p_start).dot(across));
                let (b_s, b_q) = ((sb - p_start).dot(along), (sb - p_start).dot(across));
                let ds = b_s - a_s;
                if ds.abs() <= 1.0e-12 * (sb - sa).length() {
                    continue;
                }
                let slope = (b_q - a_q) / ds;
                // Height of the hit above the spine at law fraction τ.
                let hit_at = |tau: f64| slope.mul_add((s1 - s0).mul_add(tau, s0) - a_s, a_q);
                let to_tau = |s_value: f64| (s_value - s0) / (s1 - s0);
                let mut lo = to_tau(a_s.min(b_s)).max(0.0);
                let mut hi = to_tau(a_s.max(b_s)).min(1.0);
                // Hits at the spine itself (within 10 tol) do not bound it.
                let (q_lo, q_hi) = (hit_at(lo), hit_at(hi));
                let floor = 10.0 * tol.linear;
                if q_lo <= floor && q_hi <= floor {
                    continue;
                }
                if q_lo <= floor || q_hi <= floor {
                    let crossing = lo + (hi - lo) * (floor - q_lo) / (q_hi - q_lo);
                    if q_lo <= floor {
                        lo = crossing;
                    } else {
                        hi = crossing;
                    }
                }
                if hi.is_nan() || lo.is_nan() || hi < lo {
                    continue;
                }
                let hit_slope = (hit_at(1.0) - hit_at(0.0)) / reach;
                for piece in &pieces {
                    let (u_lo, u_hi) = (lo.max(piece.t0), hi.min(piece.t1));
                    if u_hi < u_lo {
                        continue;
                    }
                    let mut candidates = vec![u_lo, u_hi];
                    candidates.extend(piece.overrun_extrema(hit_slope, u_lo, u_hi));
                    for tau in candidates {
                        let radius = law.evaluate(tau);
                        let available = hit_at(tau) / reach;
                        if reach.mul_add(radius, -hit_at(tau)) >= -tol.linear
                            && worst.is_none_or(|(r, w)| radius * w > r * available)
                        {
                            worst = Some((radius, available));
                        }
                    }
                }
            }
            if let Some((requested_radius, available_radius)) = worst {
                return Err(crate::OperationsError::Blend(
                    remus_blend::BlendError::CliffEncountered {
                        edge: *edge_id,
                        face: face_id,
                        requested_radius,
                        available_radius,
                    },
                ));
            }
        }
    }
    Ok(())
}

/// The edges of the convex hull of `points` projected into the plane with
/// unit normal `normal`, lifted back onto the points themselves.
fn planar_hull_segments(points: &[Point3], normal: Vec3) -> Vec<(Point3, Point3)> {
    let Some(&origin) = points.first() else {
        return Vec::new();
    };
    let helper = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let (Ok(x_axis), Ok(n)) = (normal.cross(helper).normalize(), normal.normalize()) else {
        return Vec::new();
    };
    let y_axis = n.cross(x_axis);
    let mut planar: Vec<(f64, f64, Point3)> = points
        .iter()
        .map(|p| ((*p - origin).dot(x_axis), (*p - origin).dot(y_axis), *p))
        .collect();
    planar.compact_sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let cross = |o: &(f64, f64, Point3), a: &(f64, f64, Point3), b: &(f64, f64, Point3)| {
        (a.0 - o.0).mul_add(b.1 - o.1, -(a.1 - o.1) * (b.0 - o.0))
    };
    let mut hull: Vec<(f64, f64, Point3)> = Vec::with_capacity(planar.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &(f64, f64, Point3)>> = if pass == 0 {
            Box::new(planar.iter())
        } else {
            Box::new(planar.iter().rev())
        };
        for point in ordered {
            while hull.len() >= start + 2
                && cross(&hull[hull.len() - 2], &hull[hull.len() - 1], point) <= 0.0
            {
                hull.pop();
            }
            hull.push(*point);
        }
        hull.pop();
    }
    if hull.len() < 2 {
        return planar.windows(2).map(|w| (w[0].2, w[1].2)).collect();
    }
    (0..hull.len())
        .map(|i| (hull[i].2, hull[(i + 1) % hull.len()].2))
        .collect()
}

/// Extract the complete native boundary of a clamped variable-fillet stripe.
fn variable_stripe_boundaries(
    surface: &remus_math::nurbs::surface::NurbsSurface,
) -> Result<Vec<crate::boolean::BoundaryCurveOverride>, crate::OperationsError> {
    use remus_math::nurbs::curve::NurbsCurve;

    // Every stripe is a clamped tensor-product patch. Its four boundary
    // curves are therefore the corresponding control-net rows/columns,
    // retaining their native knots and rational weights, not chords between
    // the corner positions. The same curves must bound the support/cap faces.
    let points = surface.control_points();
    let weights = surface.weights();
    let last_u = points.len() - 1;
    let last_v = points[0].len() - 1;
    let curves = [
        NurbsCurve::new(
            surface.degree_u(),
            surface.knots_u().to_vec(),
            points.iter().map(|row| row[0]).collect(),
            weights.iter().map(|row| row[0]).collect(),
        )?,
        NurbsCurve::new(
            surface.degree_v(),
            surface.knots_v().to_vec(),
            points[last_u].clone(),
            weights[last_u].clone(),
        )?,
        NurbsCurve::new(
            surface.degree_u(),
            surface.knots_u().to_vec(),
            points.iter().map(|row| row[last_v]).collect(),
            weights.iter().map(|row| row[last_v]).collect(),
        )?,
        NurbsCurve::new(
            surface.degree_v(),
            surface.knots_v().to_vec(),
            points[0].clone(),
            weights[0].clone(),
        )?,
    ];
    Ok(curves
        .into_iter()
        .map(|curve| {
            let trim = curve.domain();
            crate::boolean::BoundaryCurveOverride {
                start: curve.evaluate(trim.0),
                end: curve.evaluate(trim.1),
                curve: EdgeCurve::NurbsCurve(curve),
                trim,
            }
        })
        .collect())
}

mod variable_pcurves {
    //! Exact pcurves for straight variable stripes with planar neighboring faces.

    use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
    use remus_math::tolerance::Tolerance;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::EdgeCurve;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;
    use remus_topology::pcurve::PCurve;
    use remus_topology::solid::SolidId;

    fn unavailable() -> crate::OperationsError {
        crate::OperationsError::Unsupported {
            operation: "fillet_variable",
            reason: "native stripe boundary lacks a certified pcurve on both incident faces".into(),
        }
    }

    pub(super) fn register(
        topo: &mut Topology,
        solid: SolidId,
        tol: Tolerance,
    ) -> Result<(), crate::OperationsError> {
        let mut uses = Vec::new();
        for face in solid_faces(topo, solid)? {
            let data = topo.face(face)?;
            let surface = data.surface().clone();
            let boundaries = topo.wire(data.outer_wire())?.edges().to_vec();
            for usage in boundaries {
                let edge = topo.edge(usage.edge())?;
                let EdgeCurve::NurbsCurve(curve) = edge.curve() else {
                    continue;
                };
                let (t0, t1) = edge.strict_domain().map_err(|_| unavailable())?;
                let (p0, p1) = if usage.is_forward() {
                    (t0, t1)
                } else {
                    (t1, t0)
                };
                let pc = match &surface {
                    FaceSurface::Plane { normal, d } => {
                        let denominator = normal.dot(*normal);
                        let origin = Point3::new(
                            normal.x() * d / denominator,
                            normal.y() * d / denominator,
                            normal.z() * d / denominator,
                        );
                        let n = normal.normalize().map_err(|_| unavailable())?;
                        let seed = if n.x().abs() < 0.9 {
                            Vec3::new(1.0, 0.0, 0.0)
                        } else {
                            Vec3::new(0.0, 1.0, 0.0)
                        };
                        let u = n.cross(seed).normalize().map_err(|_| unavailable())?;
                        let v = n.cross(u);
                        let points = curve
                            .control_points()
                            .iter()
                            .map(|p| {
                                let delta = *p - origin;
                                Point2::new(delta.dot(u), delta.dot(v))
                            })
                            .collect();
                        Curve2D::Nurbs(NurbsCurve2D::new(
                            curve.degree(),
                            curve.knots().to_vec(),
                            points,
                            curve.weights().to_vec(),
                        )?)
                    }
                    FaceSurface::Nurbs(surface) => {
                        let index = super::variable_stripe_boundaries(surface)?.iter().position(|b| matches!(&b.curve, EdgeCurve::NurbsCurve(candidate) if candidate == curve)).ok_or_else(unavailable)?;
                        let (u0, u1) = surface.domain_u();
                        let (v0, v1) = surface.domain_v();
                        let (origin, direction) = match index {
                            0 => (Point2::new(0.0, v0), Vec2::new(1.0, 0.0)),
                            1 => (Point2::new(u1, 0.0), Vec2::new(0.0, 1.0)),
                            2 => (Point2::new(0.0, v1), Vec2::new(1.0, 0.0)),
                            _ => (Point2::new(u0, 0.0), Vec2::new(0.0, 1.0)),
                        };
                        Curve2D::Line(Line2D::new(origin, direction)?)
                    }
                    FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_) => return Err(unavailable()),
                };
                uses.push((
                    usage.edge(),
                    face,
                    usage.is_forward(),
                    PCurve::new(pc, p0, p1),
                ));
            }
        }
        for (edge, face, forward, pc) in &uses {
            topo.set_pcurve_oriented(*edge, *face, *forward, pc.clone())?;
        }
        for (edge, face, forward, _) in uses {
            remus_topology::validation::validate_same_parameter_strict(
                topo, edge, face, forward, tol.linear, 0,
            )
            .map_err(|_| unavailable())?;
            remus_topology::validation::validate_same_range_strict(
                topo, edge, face, forward, tol.linear,
            )
            .map_err(|_| unavailable())?;
        }
        Ok(())
    }
}

/// Transaction body of [`fillet_variable`]: builds the canal surfaces, then
/// proves coverage and result validity before committing.
#[allow(clippy::too_many_lines)]
fn fillet_variable_transacted(
    topo: &mut Topology,
    solid: SolidId,
    edge_laws: &[(EdgeId, FilletRadiusLaw)],
    requested_setbacks: &HashMap<usize, (f64, f64)>,
    corners: VariableCorners,
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
    let mut seen_edges = HashSet::with_capacity_and_hasher(edge_laws.len(), DetState);
    let edge_laws: Vec<(EdgeId, FilletRadiusLaw)> = edge_laws
        .iter()
        .filter(|(edge_id, _)| seen_edges.insert(*edge_id))
        .cloned()
        .collect();
    let edge_laws = edge_laws.as_slice();

    for (_, law) in edge_laws {
        law.validated_bounds(tol.linear)?;
    }
    reject_curved_variable_spines(topo, solid, edge_laws)?;

    // Convert physical endpoint setbacks into the original edge parameter
    // domain. The first qualified cell is deliberately exact: straight
    // spines only, where distance and parameter are affine. Curved-spine arc
    // length inversion remains outside this issue rather than being sampled.
    let mut active_intervals: HashMap<usize, (f64, f64)> =
        HashMap::with_capacity_and_hasher(edge_laws.len(), DetState);
    let mut declared_endpoint_setbacks: HashMap<(usize, usize), f64> = HashMap::default();
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

    crate::refuse_cavity_shells(topo, solid, "variable fillet")?;
    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let shell_face_ids: Vec<_> = shell.faces().to_vec();

    let mut edge_to_faces: HashMap<usize, Vec<_>> = HashMap::default();
    let mut face_polygons: HashMap<usize, FacePolygon> = HashMap::default();
    let mut face_surfaces: HashMap<usize, FaceSurface> = HashMap::default();
    let target_set: HashSet<usize> = edge_laws.iter().map(|(e, _)| e.index()).collect();

    for &face_id in &shell_face_ids {
        let face = topo.face(face_id)?;
        // Normal evaluation throughout this path expects outward planes.
        // Keep the source wire order and restore its reversal on rebuild.
        let surface = match face.surface() {
            FaceSurface::Plane { normal, d } if face.is_reversed() => FaceSurface::Plane {
                normal: -*normal,
                d: -*d,
            },
            other => other.clone(),
        };
        face_surfaces.insert(face_id.index(), surface);

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
        let (normal, d) = match face_surfaces.get(&face_id.index()) {
            Some(FaceSurface::Plane { normal, d }) => (*normal, *d),
            _ => continue,
        };

        face_polygons.insert(
            face_id.index(),
            FacePolygon {
                vertex_ids,
                positions,
                wire_edge_ids,
                normal,
                d,
                reversed: face.is_reversed(),
                inner_wires: face_inner_wires,
            },
        );
    }

    let sections = reject_inexact_variable_stripes(
        topo,
        solid,
        edge_laws,
        &edge_to_faces,
        &face_surfaces,
        tol,
    )?;

    // The clearance check reads each law over its active spine interval. A
    // declared setback fixes that interval already; a tangent-ball corner
    // crops it below, and the built band runs its law over the cropped span,
    // so in that mode the check waits for the crop (#1944).
    if corners == VariableCorners::Declared {
        reject_variable_support_cliffs(
            topo,
            edge_laws,
            &sections,
            &active_intervals,
            &edge_to_faces,
            &face_surfaces,
            &HashSet::default(),
            tol,
        )?;
    }

    // Build a map from edge index to radius law for per-vertex radius lookup.
    // Each vertex adjacent to a filleted edge uses that edge's actual radius
    // at the vertex (start=0.0, end=1.0) instead of a global average.
    let edge_law_map: HashMap<usize, &FilletRadiusLaw> = edge_laws
        .iter()
        .map(|(eid, law)| (eid.index(), law))
        .collect();

    let mut vertex_fillet_edges: HashMap<usize, Vec<EdgeId>> = HashMap::default();
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
    > = HashMap::default();
    let mut selected_vertex_indices: Vec<_> = vertex_fillet_edges.keys().copied().collect();
    selected_vertex_indices.compact_sort_unstable();
    for vertex_index in selected_vertex_indices {
        let incident_edges = &vertex_fillet_edges[&vertex_index];
        let declared_count = incident_edges
            .iter()
            .filter(|edge_id| {
                declared_endpoint_setbacks.contains_key(&(edge_id.index(), vertex_index))
            })
            .count();
        // A tangent-ball corner sets every incident stripe back itself, to the
        // ball's station, and is then qualified as a declared one is.
        let ball = corners == VariableCorners::TangentBall
            && declared_count == 0
            && incident_edges.len() >= 3;
        if declared_count == 0 && !ball {
            reject_unset_stripe_corner(
                topo,
                vertex_index,
                incident_edges,
                &edge_to_faces,
                &face_surfaces,
                tol,
            )?;
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
        if !ball && (incident_edges.len() < 3 || declared_count != incident_edges.len()) {
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
        let mut planar_edge_sides =
            HashMap::with_capacity_and_hasher(incident_edges.len(), DetState);
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
            // Exactly zero, not small: the law is mapped onto the cropped
            // stripe, so any slope at the station grows as the stripe
            // shortens and opens a tangent break against the cap. The
            // S-curve and constant laws (and S-curve stations) give an
            // exact 0.0 at their ends; a ramp never does.
            if law.derivative(law_t).abs() > 0.0 {
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
        // Absolute: each stripe's corner contacts are set from the common
        // radius, so a difference becomes a law error of the same size along
        // the stripe, and only one within the linear tolerance stays exact.
        let radius_tol = tol.linear;
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
        // Each ball-cropped stripe's corner zone, vertex to station, in the
        // edge's parameter: the cap covers it, not the stripe.
        let mut corner_zones: HashMap<usize, (f64, f64)> = HashMap::default();
        for (edge_id, _) in &radii {
            let edge = topo.edge(*edge_id)?;
            let other = if edge.start() == vertex_id {
                topo.vertex(edge.end())?.point()
            } else {
                topo.vertex(edge.start())?.point()
            };
            let away = (other - vertex).normalize()?;
            let required = (center - vertex).dot(away);
            let declared = if ball {
                required
            } else {
                declared_endpoint_setbacks[&(edge_id.index(), vertex_index)]
            };
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
            if ball {
                // Crop the stripe at the ball's station, as a declared
                // setback would have been cropped.
                let length = (other - vertex).length();
                let interval = active_intervals.get_mut(&edge_id.index()).ok_or_else(|| {
                    crate::OperationsError::InvalidInput {
                        reason: format!("corner stripe {edge_id:?} has no active interval"),
                    }
                })?;
                if edge.start() == vertex_id {
                    interval.0 = required / length;
                    corner_zones.insert(edge_id.index(), (0.0, interval.0));
                } else {
                    interval.1 = 1.0 - required / length;
                    corner_zones.insert(edge_id.index(), (interval.1, 1.0));
                }
                if interval.1 - interval.0 <= tol.linear / length {
                    return Err(crate::OperationsError::InvalidInput {
                        reason: format!(
                            "tangent-ball corners consume the full length of edge {edge_id:?}"
                        ),
                    });
                }
                declared_endpoint_setbacks.insert((edge_id.index(), vertex_index), required);
            }
        }
        if ball {
            // The cropped spans start at the ball's station, so the stripe
            // check below never sees the zone between it and the vertex,
            // which the corner's cap covers instead (#1945 review). On each
            // support the cap stays inside the footprint the stripe would
            // have over that zone at the corner radius (two such rectangles
            // cover the kite vertex, station, contact, station), so check
            // the zone at that radius. The corner's own stripes bound it and
            // are not obstacles. Runs before the corner-ball guard, so an
            // obstacle on a support is refused with its radius details.
            let zone_laws: Vec<(EdgeId, FilletRadiusLaw)> = radii
                .iter()
                .filter(|(edge_id, _)| corner_zones.contains_key(&edge_id.index()))
                .map(|(edge_id, _)| (*edge_id, FilletRadiusLaw::Constant(common_radius)))
                .collect();
            let stripes: HashSet<usize> = incident_edges.iter().map(|edge| edge.index()).collect();
            reject_variable_support_cliffs(
                topo,
                &zone_laws,
                &sections,
                &corner_zones,
                &edge_to_faces,
                &face_surfaces,
                &stripes,
                tol,
            )?;
        }
        reject_faces_in_corner_ball(
            topo,
            solid,
            vertex_id,
            incident_edges,
            &edge_to_faces,
            center,
            common_radius,
            tol,
        )?;
        let is_concave = planar_edge_sides.values().all(|side| *side > 0.0);
        qualified_setback_corners
            .insert(vertex_index, (vertex_id, center, common_radius, is_concave));
    }

    if corners == VariableCorners::TangentBall {
        reject_variable_support_cliffs(
            topo,
            edge_laws,
            &sections,
            &active_intervals,
            &edge_to_faces,
            &face_surfaces,
            &HashSet::default(),
            tol,
        )?;
    }

    // Shared contact map: the SAME inward contact point used both to trim the
    // adjacent faces and to anchor the blend boundary, keyed by
    // (vertex_index, edge_index, face_index). Computing it once guarantees the
    // trimmed face boundary and the blend boundary coincide (watertight shell).
    // Geometry uses the active (possibly setback) station; the normalized law
    // still uses 0/1 at the built stripe endpoints.
    let fillet_contact_map: HashMap<(usize, usize, usize), Point3> = {
        let mut map = HashMap::default();
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
                let Some(section) = sections.get(&edge_id.index()) else {
                    continue;
                };
                map.insert(
                    (vid.index(), edge_id.index(), f1.index()),
                    p + section.contact1 * r,
                );
                map.insert(
                    (vid.index(), edge_id.index(), f2.index()),
                    p + section.contact2 * r,
                );
            }
        }
        map
    };

    // Vertices at endpoints of filleted edges. A side face (one that shares
    // such a vertex but whose own edges are not filleted) must split that
    // corner into the two blend contact points, or the blend boundary is left
    // unmatched and the shell becomes non-manifold.
    let mut vertex_fillet_endpoints: HashSet<usize> = HashSet::default();
    for (edge_id, _) in edge_laws {
        let edge = topo.edge(*edge_id)?;
        vertex_fillet_endpoints.insert(edge.start().index());
        vertex_fillet_endpoints.insert(edge.end().index());
    }

    // Faces the positional `FaceSpec` vocabulary cannot describe (#1789).
    //
    // Every spec below except `FaceSpec::Existing` states a wire as a list of
    // vertex positions, from which the assembler mints straight edges. A bore
    // or boss rim is ONE closed circle edge, a single position, so a cap that
    // carries it lost the hole and the cylinder wall bounded by it fell out of
    // the shell as a separate, open component: any body with a circular edge
    // anywhere refused the round. A face the blend does not reach that has a
    // hole or a curved edge is copied whole; a trimmed planar face whose holes
    // the blend does not reach keeps them verbatim under its rebuilt outer
    // wire. Neighbours rebuilt from positions share the copied edges through
    // the assembler's vertex dedup.
    let touches_blend = |edge: &remus_topology::edge::Edge, edge_id: EdgeId| {
        target_set.contains(&edge_id.index())
            || vertex_fillet_endpoints.contains(&edge.start().index())
            || vertex_fillet_endpoints.contains(&edge.end().index())
    };
    let mut verbatim_faces: HashSet<usize> = HashSet::default();
    let mut untouched_faces: HashSet<usize> = HashSet::default();
    let mut untouched_inner_faces: HashSet<usize> = HashSet::default();
    for &face_id in &shell_face_ids {
        let face = topo.face(face_id)?;
        let mut closed_edge = false;
        let mut curved_edge = false;
        let mut outer_touched = false;
        let mut inner_touched = false;
        for (index, wid) in std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .enumerate()
        {
            for oe in topo.wire(wid)?.edges() {
                let edge = topo.edge(oe.edge())?;
                closed_edge |= edge.start() == edge.end();
                curved_edge |= !matches!(edge.curve(), EdgeCurve::Line);
                let touched = touches_blend(edge, oe.edge());
                if index == 0 {
                    outer_touched |= touched;
                } else {
                    inner_touched |= touched;
                }
            }
        }
        let touched = outer_touched || inner_touched;
        if !face.inner_wires().is_empty() || closed_edge || (curved_edge && !touched) {
            verbatim_faces.insert(face_id.index());
        }
        if !touched {
            untouched_faces.insert(face_id.index());
        }
        if !inner_touched {
            untouched_inner_faces.insert(face_id.index());
        }
    }

    // Trim planar faces by replacing each filleted-edge boundary vertex with
    // the shared contact point. The NURBS canal surface replaces the fillet face.
    let mut all_specs: Vec<FaceSpec> = Vec::new();
    let mut boundary_curves = Vec::new();

    for &face_id in &shell_face_ids {
        if verbatim_faces.contains(&face_id.index()) && untouched_faces.contains(&face_id.index()) {
            all_specs.push(FaceSpec::Existing {
                face: face_id,
                outer: None,
            });
            continue;
        }
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
            all_specs.push(poly.spec(poly.positions.clone(), poly.d));
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

        if verbatim_faces.contains(&face_id.index())
            && untouched_inner_faces.contains(&face_id.index())
        {
            // A trimmed face whose holes the blend does not reach: rebuild
            // the outer wire from the contacts, copy each hole verbatim so a
            // bore rim stays the edge its wall is bounded by.
            all_specs.push(FaceSpec::Existing {
                face: face_id,
                outer: Some(new_verts),
            });
            continue;
        }
        let new_d = dot_normal_point(poly.normal, new_verts[0]);
        all_specs.push(poly.spec(new_verts, new_d));
    }

    let n_samples = 5; // Number of cross-sections along each edge
    // Every requested edge must end up carrying a blend surface; the loop's
    // early `continue`s are where a silent subset used to come from.
    let mut blended_edges: HashSet<usize> = HashSet::default();

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
        if cs_ref.half_angle.abs() < tol.angular {
            continue;
        }
        // Every stripe here passed `reject_inexact_variable_stripes`, which
        // recorded its exact section.
        let Some(section) = sections.get(&edge_id.index()).copied() else {
            continue;
        };

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
            // The exact tangent-ball section (see `VariableSection`): its
            // contacts scale with r, its apex weight is sin(α/2).
            let w = section.weight;
            let contact1 = p + section.contact1 * r;
            let contact2 = p + section.contact2 * r;
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
        // Every stripe reaching here has a straight spine between planar
        // supports (`reject_inexact_variable_stripes`), so each row is the
        // law itself in cubic Hermite form, one Bézier piece per law segment
        // joined at triple knots: the contact rows are `p(t) + d·r(t)` with a
        // fixed direction `d`, a cubic on every piece. This preserves the
        // zero endpoint derivative of an S-curve law, which is what makes a
        // setback sphere seam truly G1.
        let active_vector = (p_end - p_start) * (t_end - t_start);
        let pieces = law.segments();
        let position = |fraction: f64| {
            p_start + (p_end - p_start) * (t_end - t_start).mul_add(fraction, t_start)
        };
        let row = |first: Point3, last: Point3, direction: Vec3| {
            let mut controls = Vec::with_capacity(3 * pieces.len() + 1);
            for (index, piece) in pieces.iter().enumerate() {
                let span = piece.t1 - piece.t0;
                let start = if index == 0 {
                    first
                } else {
                    position(piece.t0) + direction * piece.r0
                };
                let end = if index + 1 == pieces.len() {
                    last
                } else {
                    position(piece.t1) + direction * piece.r1
                };
                if index == 0 {
                    controls.push(start);
                }
                controls.push(start + (active_vector + direction * piece.slope0) * (span / 3.0));
                controls.push(end - (active_vector + direction * piece.slope1) * (span / 3.0));
                controls.push(end);
            }
            controls
        };
        let mut knots_v = vec![0.0; 4];
        for piece in &pieces[..pieces.len() - 1] {
            knots_v.extend([piece.t1; 3]);
        }
        knots_v.extend([1.0; 4]);
        let control_points = vec![
            row(row_contact1[0], row_contact1[n_v - 1], section.contact1),
            row(row_mid[0], row_mid[n_v - 1], Vec3::new(0.0, 0.0, 0.0)),
            row(row_contact2[0], row_contact2[n_v - 1], section.contact2),
        ];
        let degree_v = 3;
        let mid_weights = vec![sample_weights[0]; control_points[0].len()];
        let n_cp_v = control_points[0].len();

        let surface = remus_math::nurbs::surface::NurbsSurface::new(
            2, // degree_u (circular arc)
            degree_v,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], // knots_u
            knots_v,
            control_points,
            vec![vec![1.0; n_cp_v], mid_weights, vec![1.0; n_cp_v]],
        )
        .map_err(crate::OperationsError::Math)?
        // Exact by construction: `reject_inexact_variable_stripes` admits
        // only straight convex spines between planes, whose sections are the
        // tangent ball's arcs (`VariableSection`) and whose rows are the law.
        .with_exact_form(remus_math::nurbs::surface::ExactForm::VariableRollingBall);

        let c1s = grid[0][0];
        let c2s = grid[0][2];
        let c1e = grid[n_v - 1][0];
        let c2e = grid[n_v - 1][2];

        // Reverse the stored surface when its parametric mid-normal points
        // into the dihedral. Keeping the decision on the spec is stable even
        // when analytic cap specs are assembled ahead of ordinary surfaces.
        let srf_mid_normal = surface.normal(0.5, 0.5).unwrap_or(cs_ref.bisector);
        let reversed = srf_mid_normal.dot(cs_ref.bisector) > 0.0;
        boundary_curves.extend(variable_stripe_boundaries(&surface)?);
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
    corner_indices.compact_sort_unstable();
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
        contacts.compact_sort_by(|a, b| {
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

    let solid_id = crate::boolean::assemble_solid_mixed_with_history_and_curves(
        topo,
        &all_specs,
        &boundary_curves,
        tol,
    )?
    .solid;

    // This certificate family covers the straight-spine, planar-neighbor
    // construction without spherical setback corners. Other existing
    // variable-blend families retain their previous pcurve behavior.
    let planar_boundary_family = qualified_setback_corners.is_empty()
        && shell_face_ids.iter().all(|&face| {
            matches!(
                topo.face(face).map(remus_topology::face::Face::surface),
                Ok(FaceSurface::Plane { .. })
            )
        })
        && edge_laws.iter().all(|(edge, _)| {
            matches!(
                topo.edge(*edge).map(remus_topology::edge::Edge::curve),
                Ok(EdgeCurve::Line)
            )
        });
    if planar_boundary_family {
        variable_pcurves::register(topo, solid_id, tol)?;
    }

    // Fail closed on a plausible-but-wrong result: no new validation errors
    // against the input baseline, and the volume change must be one a blend
    // of this size can physically produce — an oversized radius used to
    // "succeed" here with the volume GROWING (r=50 on a 10 mm box returned
    // 3242 mm³), and holed-plate selections returned invalid shells as Ok.
    let mut baseline = crate::blend_ops::BlendBaseline::new(topo, solid);
    crate::blend_ops::validate_blend_solid_against_input(&mut baseline, "fillet", solid_id)?;
    let (min_radius, max_radius) = edge_laws.iter().fold(
        (f64::INFINITY, 0.0_f64),
        |(min_radius, max_radius), (_, law)| {
            let (law_min, law_max) = law.bounds();
            (min_radius.min(law_min), max_radius.max(law_max))
        },
    );
    let edges: Vec<EdgeId> = edge_laws.iter().map(|(edge_id, _)| *edge_id).collect();
    crate::blend_ops::validate_blend_volume(
        &mut baseline,
        "fillet",
        solid_id,
        &edges,
        crate::blend_ops::BlendSize::VariableFillet {
            min_radius,
            max_radius,
        },
    )?;
    Ok(solid_id)
}
