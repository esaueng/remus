//! Edge chamfering (cutting edges at an angle).
//!
//! Chamfer replaces each target edge with a flat bevel face. The algorithm
//! works by rebuilding face polygons with offset vertices and inserting new
//! quadrilateral chamfer faces, then assembling the result using the same
//! spatial-hash dedup pattern as [`crate::boolean`].
//!
//! Only the faces the bevel actually cuts back are rebuilt. A face the
//! chamfer does not move — including every curved face — is carried through
//! verbatim, keeping its exact surface, its curved edges, its orientation and
//! all of its inner wires. A face the chamfer *does* move gets a replacement
//! outer wire and still keeps its inner wires; if the bevel would cut into one
//! of those holes the operation is refused rather than approximated. Only the
//! bevel strips and corner patches are newly minted geometry, and new geometry
//! cannot carry a hole.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use brepkit_blend::BlendFaceOrigins;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;
use brepkit_topology::vertex::VertexId;
use brepkit_topology::wire::WireId;

use crate::OperationsError;
use crate::boolean::{FaceSpec, assemble_solid_mixed_with_history};
use crate::dot_normal_point;

/// Operation name carried by [`OperationsError::Unsupported`] refusals.
const OP: &str = "chamfer";

/// Deflection for the closing volume check. Only the sign and gross magnitude
/// matter, so this is deliberately coarse.
const VOLUME_DEFLECTION: f64 = 0.05;

/// Samples taken along a closed inner-wire curve when testing whether the
/// bevel cuts into it. Matches the boolean pipeline's own sampling so a bore
/// rim is tested against the same polygon the rest of the kernel sees.
const HOLE_SAMPLES: usize = 32;

enum FaceSpecOrigin {
    Modified(FaceId),
    Generated(Vec<FaceId>),
}

fn push_face_spec(
    specs: &mut Vec<FaceSpec>,
    origins: &mut Vec<FaceSpecOrigin>,
    spec: FaceSpec,
    origin: FaceSpecOrigin,
) {
    specs.push(spec);
    origins.push(origin);
}

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: OP,
        reason: reason.into(),
    }
}

/// Whether `from -> to` occurs as one boundary segment in a positional face
/// specification. The mixed assembler shares edges by position, so orienting
/// a new bevel against this traversal is stronger than inferring its winding
/// from a near-cancelling pair of face normals at an obtuse edge.
fn has_directed_segment(spec: &FaceSpec, from: Point3, to: Point3, eps: f64) -> bool {
    let vertices = spec.vertices();
    !vertices.is_empty()
        && vertices.iter().enumerate().any(|(i, &start)| {
            let end = vertices[(i + 1) % vertices.len()];
            (start - from).length() <= eps && (end - to).length() <= eps
        })
}

/// Chamfer one or more edges of a solid.
///
/// Each target edge is replaced by a flat bevel face. The `distance`
/// parameter controls how far from each vertex the bevel is placed
/// along the adjacent edges.
///
/// Faces the bevel does not move are carried through verbatim — exact
/// surface, curved edges, orientation and every inner wire. Faces it does
/// move keep their inner wires while their outer wire is rebuilt. The result
/// is checked with [`crate::validate::validate_solid`] and required to enclose
/// a positive volume strictly smaller than the input's before it is returned.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] if `distance` is zero or
/// negative, no edges are given, none of them is shared by exactly two faces,
/// or a setback does not fit on the face it slides across.
///
/// Returns [`OperationsError::Unsupported`] when the bevel has no exact
/// construction here — a target edge on a curved face or on a hole's rim, a
/// corner of a bevelled edge that also lies on a curved face or on a hole's
/// rim, a rebuilt boundary carrying a curved edge, a bevel that would cut into
/// a hole in the face it trims, or a result that fails validation, encloses no
/// volume, or does not come back smaller than the input.
pub fn chamfer(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
) -> Result<SolidId, crate::OperationsError> {
    Ok(chamfer_with_origins(topo, solid, edges, distance)?.0)
}

/// [`chamfer`] with construction-derived face provenance.
pub(crate) fn chamfer_with_origins(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
) -> Result<(SolidId, BlendFaceOrigins), crate::OperationsError> {
    if distance <= 0.0 {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("chamfer distance must be positive, got {distance}"),
        });
    }
    if edges.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no edges specified for chamfer".into(),
        });
    }
    chamfer_core(topo, solid, edges, ChamferDistances::Symmetric(distance))
}

/// Asymmetric chamfer: `d1` on the first adjacent face, `d2` on the second.
///
/// Each target edge is replaced by a flat bevel face. Unlike [`chamfer()`],
/// the two adjacent faces can have different setback distances, producing
/// a non-symmetric bevel. Faces, holes and orientation are preserved exactly
/// as described for [`chamfer()`], and the result passes the same gates.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] if either distance is zero or
/// negative, no edges are given, none of them is shared by exactly two faces,
/// or a setback does not fit on the face it slides across.
///
/// Returns [`OperationsError::Unsupported`] for the same configurations
/// [`chamfer()`] declines.
pub fn chamfer_asymmetric(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<SolidId, crate::OperationsError> {
    Ok(chamfer_asymmetric_with_origins(topo, solid, edges, d1, d2)?.0)
}

/// [`chamfer_asymmetric`] with construction-derived face provenance.
pub(crate) fn chamfer_asymmetric_with_origins(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<(SolidId, BlendFaceOrigins), crate::OperationsError> {
    if d1 <= 0.0 || d2 <= 0.0 {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("chamfer distances must be positive, got d1={d1}, d2={d2}"),
        });
    }
    if edges.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no edges specified for chamfer".into(),
        });
    }
    chamfer_core(topo, solid, edges, ChamferDistances::Asymmetric { d1, d2 })
}

/// How chamfer distances are assigned to edges.
enum ChamferDistances {
    /// Same distance on both adjacent faces.
    Symmetric(f64),
    /// `d1` on face\[0\], `d2` on face\[1\] (per `edge_to_faces` order).
    Asymmetric { d1: f64, d2: f64 },
}

impl ChamferDistances {
    /// Resolve the chamfer distance for a specific edge on a specific face.
    fn distance_for(
        &self,
        edge_index: usize,
        face_id: FaceId,
        edge_to_faces: &HashMap<usize, Vec<FaceId>>,
    ) -> f64 {
        match self {
            Self::Symmetric(d) => *d,
            Self::Asymmetric { d1, d2 } => {
                if let Some(faces) = edge_to_faces.get(&edge_index)
                    && faces.len() == 2
                {
                    if faces[0] == face_id {
                        return *d1;
                    }
                    if faces[1] == face_id {
                        return *d2;
                    }
                }
                // Fallback (shouldn't happen for filtered manifold edges).
                *d1
            }
        }
    }

    /// Maximum distance across all faces (used for side-face corner offsets).
    fn max_distance(&self) -> f64 {
        match self {
            Self::Symmetric(d) => *d,
            Self::Asymmetric { d1, d2 } => d1.max(*d2),
        }
    }
}

#[allow(clippy::too_many_lines)]
fn chamfer_core(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distances: ChamferDistances,
) -> Result<(SolidId, BlendFaceOrigins), crate::OperationsError> {
    let tol = Tolerance::new();

    // Measured before anything is built so the closing gate can insist the
    // bevel actually took material off.
    let before = crate::measure::solid_volume(topo, solid, VOLUME_DEFLECTION)?;

    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let shell_face_ids: Vec<FaceId> = shell.faces().to_vec();

    let mut edge_to_faces: HashMap<usize, Vec<FaceId>> = HashMap::new();
    let mut face_polygons: HashMap<usize, FacePolygon> = HashMap::new();
    // Faces the polygon rebuild cannot express, kept so a bevel that would
    // need one re-trimmed can be refused by name instead of leaving it behind.
    let mut curved_faces: BTreeSet<usize> = BTreeSet::new();
    // Which faces meet each vertex, over every wire — a corner shared with a
    // curved face or with a hole's rim is not one this rebuild can relocate.
    let mut faces_at_vertex: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
    // Edges and vertices that belong to some face's inner wire. The rebuild
    // below only ever touches outer wires.
    let mut inner_wire_edges: BTreeSet<usize> = BTreeSet::new();
    let mut inner_wire_vertices: BTreeSet<usize> = BTreeSet::new();

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
            for v in [edge.start(), edge.end()] {
                faces_at_vertex
                    .entry(v.index())
                    .or_default()
                    .insert(face_id.index());
            }
        }

        // Include inner wire edges in adjacency map so hole-boundary
        // edges are correctly counted as shared by 2 faces.
        for &inner_wire_id in face.inner_wires() {
            let inner_wire = topo.wire(inner_wire_id)?;
            for oe in inner_wire.edges() {
                edge_to_faces
                    .entry(oe.edge().index())
                    .or_default()
                    .push(face_id);
                inner_wire_edges.insert(oe.edge().index());
                let edge = topo.edge(oe.edge())?;
                for v in [edge.start(), edge.end()] {
                    inner_wire_vertices.insert(v.index());
                    faces_at_vertex
                        .entry(v.index())
                        .or_default()
                        .insert(face_id.index());
                }
            }
        }

        // Only build polygon data for planar faces. Non-planar faces are
        // carried through verbatim, so they may not be touched by the bevel.
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            curved_faces.insert(face_id.index());
            continue;
        };
        let normal = *normal;

        face_polygons.insert(
            face_id.index(),
            FacePolygon {
                vertex_ids,
                positions,
                wire_edge_ids,
                normal,
                // A face's outer wire winds CCW about its *stored* surface
                // normal whether or not the face is reversed, so `normal`
                // stays the reference for in-plane constructions while
                // `outward` is what the bevel must face away from.
                outward: if face.is_reversed() { -normal } else { normal },
            },
        );
    }

    // Like fillet, silently skip non-manifold edges (shared by != 2 faces)
    // which commonly occur in boolean operation output.
    let filtered_edges: Vec<EdgeId> = edges
        .iter()
        .copied()
        .filter(|edge_id| {
            edge_to_faces
                .get(&edge_id.index())
                .is_some_and(|faces| faces.len() == 2)
        })
        .collect();

    if filtered_edges.is_empty() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "no manifold edges to chamfer (all edges are boundary or missing)".into(),
        });
    }

    // Record convexity while the input topology is untouched. The exact
    // planar construction supports both material-removing convex bevels and
    // material-filling concave notch bevels; its closing sign gate must not
    // misclassify the latter as a folded result.
    let convexity_by_edge: HashMap<usize, bool> = filtered_edges
        .iter()
        .filter_map(|&edge| {
            crate::blend_ops::edge_is_convex(topo, solid, edge, distances.max_distance() * 0.25)
                .map(|convex| (edge.index(), convex))
        })
        .collect();
    let all_convex = convexity_by_edge.len() == filtered_edges.len()
        && convexity_by_edge.values().all(|&convex| convex);
    let all_concave = convexity_by_edge.len() == filtered_edges.len()
        && convexity_by_edge.values().all(|&convex| !convex);

    let target_set: HashSet<usize> = filtered_edges.iter().map(|e| e.index()).collect();

    // Everything the bevel touches has to be rebuildable exactly. Walk the
    // targets in the caller's order so the edge named in a refusal never
    // depends on hash order.
    for &edge_id in &filtered_edges {
        if inner_wire_edges.contains(&edge_id.index()) {
            return Err(unsupported(format!(
                "edge {} is part of a hole's rim; bevelling it would have to rebuild \
                 an inner wire, and this chamfer only re-trims outer boundaries",
                edge_id.index()
            )));
        }
        for face_id in &edge_to_faces[&edge_id.index()] {
            if curved_faces.contains(&face_id.index()) {
                return Err(unsupported(format!(
                    "edge {} lies on curved face {}; a bevel is cut from the two \
                     planes meeting at the edge, and this one has no plane",
                    edge_id.index(),
                    face_id.index()
                )));
            }
        }
        let edge = topo.edge(edge_id)?;
        for vid in [edge.start(), edge.end()] {
            if inner_wire_vertices.contains(&vid.index()) {
                return Err(unsupported(format!(
                    "corner {} of bevelled edge {} sits on a hole's rim; moving it \
                     would drag the hole off the wall that owns it",
                    vid.index(),
                    edge_id.index()
                )));
            }
            let Some(at) = faces_at_vertex.get(&vid.index()) else {
                continue;
            };
            for &pos in at {
                if curved_faces.contains(&pos) {
                    return Err(unsupported(format!(
                        "corner {} of bevelled edge {} also lies on curved face {}; \
                         re-trimming a curved neighbour against the bevel is not \
                         implemented",
                        vid.index(),
                        edge_id.index(),
                        pos
                    )));
                }
            }
        }
    }

    // Vertices at endpoints of chamfered edges (used to detect side-face corners).
    let mut vertex_chamfer_endpoints: HashSet<usize> = HashSet::new();
    // For side-face corners, compute max distance from any chamfered edge meeting
    // at each vertex so the offset stays consistent with the largest adjacent bevel.
    let mut vertex_max_distance: HashMap<usize, f64> = HashMap::new();
    // Which chamfered edges meet each vertex. A side-face corner needs this to
    // work out, per direction, which neighbouring face's setback it must match.
    let mut vertex_chamfer_edges: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &edge_id in &filtered_edges {
        let edge = topo.edge(edge_id)?;
        let max_d = distances.max_distance();
        for vid in [edge.start(), edge.end()] {
            vertex_chamfer_endpoints.insert(vid.index());
            let entry = vertex_max_distance.entry(vid.index()).or_insert(0.0_f64);
            if max_d > *entry {
                *entry = max_d;
            }
            vertex_chamfer_edges
                .entry(vid.index())
                .or_default()
                .push(edge_id);
        }
    }

    // A setback larger than the face it slides across produces an inverted
    // polygon, not a chamfer. Reject before touching the arena so the caller
    // gets a precise error instead of a plausible-looking wrong solid.
    SetbackCheck {
        edge_to_faces: &edge_to_faces,
        target_set: &target_set,
        distances: &distances,
        vertex_chamfer_endpoints: &vertex_chamfer_endpoints,
        vertex_max_distance: &vertex_max_distance,
        vertex_chamfer_edges: &vertex_chamfer_edges,
    }
    .run(&shell_face_ids, &face_polygons, tol)?;

    let mut chamfer_data: HashMap<usize, ChamferEdgeData> = HashMap::new();
    let mut result_specs: Vec<FaceSpec> = Vec::new();
    let mut result_spec_origins: Vec<FaceSpecOrigin> = Vec::new();

    // Track corner vertices where all adjacent edges are chamfered.
    // Maps vertex_id → the (face, trim-plane intersection) each face contributed.
    // Ordered so the emitted face list never depends on hash iteration order.
    let mut corner_data: BTreeMap<usize, Vec<(FaceId, Point3)>> = BTreeMap::new();

    // Count how many faces reference each vertex (to detect full-corner chamfer).
    let mut vertex_face_count: HashMap<usize, usize> = HashMap::new();
    for poly in face_polygons.values() {
        for vid in &poly.vertex_ids {
            *vertex_face_count.entry(vid.index()).or_default() += 1;
        }
    }

    // Scale-relative slack for "is this point still on that plane", following
    // the crate's own `approx_eq` convention. Never loosened to admit a case.
    let eps = tol.linear.max(model_span(&face_polygons) * tol.relative);

    for &face_id in &shell_face_ids {
        // A curved face is never moved by the bevel — the checks above refused
        // every configuration that would need one re-trimmed — so it travels
        // verbatim: exact surface, curved edges, orientation, inner wires.
        let Some(poly) = face_polygons.get(&face_id.index()) else {
            push_face_spec(
                &mut result_specs,
                &mut result_spec_origins,
                FaceSpec::Existing {
                    face: face_id,
                    outer: None,
                },
                FaceSpecOrigin::Modified(face_id),
            );
            continue;
        };
        // A planar face none of whose corners the bevel disturbs is likewise
        // carried through whole rather than rebuilt from its corner positions,
        // which is the only way its holes survive at all.
        if !poly
            .vertex_ids
            .iter()
            .any(|v| vertex_chamfer_endpoints.contains(&v.index()))
        {
            push_face_spec(
                &mut result_specs,
                &mut result_spec_origins,
                FaceSpec::Existing {
                    face: face_id,
                    outer: None,
                },
                FaceSpecOrigin::Modified(face_id),
            );
            continue;
        }
        // This face's boundary is about to be rebuilt from corner positions,
        // which can only express straight chords.
        require_line_outer_wire(topo, face_id)?;
        let n = poly.positions.len();
        let mut new_verts: Vec<Point3> = Vec::with_capacity(n + target_set.len());

        for i in 0..n {
            let prev_i = if i == 0 { n - 1 } else { i - 1 };
            let next_i = (i + 1) % n;

            // Edge before vertex i: wire_edge_ids[prev_i] connects V[prev_i]→V[i]
            // Edge after vertex i:  wire_edge_ids[i]      connects V[i]→V[next_i]
            let before_chamfered = target_set.contains(&poly.wire_edge_ids[prev_i].index());
            let after_chamfered = target_set.contains(&poly.wire_edge_ids[i].index());

            let pos = poly.positions[i];
            let prev_pos = poly.positions[prev_i];
            let next_pos = poly.positions[next_i];

            let at_chamfer_endpoint =
                vertex_chamfer_endpoints.contains(&poly.vertex_ids[i].index());

            match (before_chamfered, after_chamfered, at_chamfer_endpoint) {
                (false, false, false) => {
                    // No chamfer at this vertex — keep as-is.
                    new_verts.push(pos);
                }
                (false, false, true) => {
                    // Side face corner: the vertex is at a chamfered edge's
                    // endpoint, but neither of THIS face's edges is chamfered.
                    // It splits into two points, one along each of its edges.
                    //
                    // Each of those edges is shared with a face that is being
                    // bevelled, and that neighbour has already placed its own
                    // chamfer point along the same edge — at *its* setback. The
                    // split point has to land on top of it or the two faces stop
                    // sharing a boundary and the shell opens up. So each
                    // direction takes the setback of the face across that edge,
                    // which is what makes an asymmetric chamfer close.
                    let fallback = vertex_max_distance
                        .get(&poly.vertex_ids[i].index())
                        .copied()
                        .unwrap_or_else(|| distances.max_distance());
                    let side_dist = |wire_edge: EdgeId| -> f64 {
                        neighbour_setback(
                            wire_edge,
                            face_id,
                            poly.vertex_ids[i],
                            &vertex_chamfer_edges,
                            &edge_to_faces,
                            &distances,
                        )
                        .unwrap_or(fallback)
                    };

                    let dir_prev = (prev_pos - pos).normalize()?;
                    new_verts.push(pos + dir_prev * side_dist(poly.wire_edge_ids[prev_i]));

                    let dir_next = (next_pos - pos).normalize()?;
                    new_verts.push(pos + dir_next * side_dist(poly.wire_edge_ids[i]));
                }
                (true, false, _) => {
                    // Only the edge before is chamfered. Offset toward V[next].
                    let dist = distances.distance_for(
                        poly.wire_edge_ids[prev_i].index(),
                        face_id,
                        &edge_to_faces,
                    );
                    let dir = (next_pos - pos).normalize()?;
                    let c = pos + dir * dist;
                    new_verts.push(c);

                    record_chamfer_point(
                        &mut chamfer_data,
                        poly.wire_edge_ids[prev_i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        c,
                    );
                }
                (false, true, _) => {
                    // Only the edge after is chamfered. Offset toward V[prev].
                    let dist = distances.distance_for(
                        poly.wire_edge_ids[i].index(),
                        face_id,
                        &edge_to_faces,
                    );
                    let dir = (prev_pos - pos).normalize()?;
                    let c = pos + dir * dist;
                    new_verts.push(c);

                    record_chamfer_point(
                        &mut chamfer_data,
                        poly.wire_edge_ids[i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        c,
                    );
                }
                (true, true, _) => {
                    // Both adjacent edges are chamfered. Compute a single
                    // intersection point where the two trim planes meet on
                    // this face, rather than two separate offset points.
                    let dist_after = distances.distance_for(
                        poly.wire_edge_ids[i].index(),
                        face_id,
                        &edge_to_faces,
                    );
                    let dist_before = distances.distance_for(
                        poly.wire_edge_ids[prev_i].index(),
                        face_id,
                        &edge_to_faces,
                    );

                    let dir_next = (next_pos - pos).normalize()?;
                    let dir_from_prev = (pos - prev_pos).normalize()?;

                    // Inward perpendiculars within the face plane.
                    // For CCW winding (matching outward normal), inward = n × d.
                    let p1 = poly.normal.cross(dir_next);
                    let p2 = poly.normal.cross(dir_from_prev);

                    let cos_angle = p1.dot(p2);
                    let denom = 1.0 + cos_angle;

                    let intersection = if denom.abs() < 1e-12 {
                        // Nearly antiparallel — fall back to midpoint.
                        let mid = Point3::new(
                            (prev_pos.x() + next_pos.x()) * 0.5,
                            (prev_pos.y() + next_pos.y()) * 0.5,
                            (prev_pos.z() + next_pos.z()) * 0.5,
                        );
                        let dir = (mid - pos).normalize()?;
                        let avg_dist = (dist_before + dist_after) * 0.5;
                        pos + dir * avg_dist
                    } else {
                        // General asymmetric case: find intersection of two
                        // offset lines in the face plane.
                        //
                        // Trim line from "after" edge: pos + dist_after * p1 + t * dir_next
                        // Trim line from "before" edge: pos + dist_before * p2 + s * (-dir_from_prev)
                        //
                        // Equating and solving in the 2D face-plane basis
                        // (p1, dir_next) vs (p2, dir_from_prev):
                        //
                        // For the symmetric case (dist_after == dist_before == d):
                        //   intersection = pos + d * (p1 + p2) / (1 + cos_angle)
                        //
                        // For asymmetric, we solve the 2×2 system directly.
                        // The offset vectors from `pos` are:
                        //   a = dist_after * p1 (point on after-trim-line)
                        //   b = dist_before * p2 (point on before-trim-line)
                        // The directions along the trim lines are:
                        //   u = dir_next (along after-edge direction)
                        //   v = -dir_from_prev (along before-edge direction, toward prev)
                        //
                        // We need: a + t*u = b + s*v
                        //   => t*u - s*v = b - a
                        //
                        // Using cross products (projected onto face normal) to solve:
                        let a = p1 * dist_after;
                        let b = p2 * dist_before;
                        let diff = b - a;
                        let v = dir_from_prev * (-1.0); // direction along before-trim-line

                        // t = (diff × v) · n / (u × v) · n
                        let u_cross_v = dir_next.cross(v);
                        let det = u_cross_v.dot(poly.normal);

                        if det.abs() < 1e-12 {
                            // Parallel trim lines — use weighted average.
                            pos + (a + b) * 0.5
                        } else {
                            let diff_cross_v = diff.cross(v);
                            let t = diff_cross_v.dot(poly.normal) / det;
                            pos + a + dir_next * t
                        }
                    };

                    new_verts.push(intersection);

                    // Record this point for both adjacent chamfered edges.
                    record_chamfer_point(
                        &mut chamfer_data,
                        poly.wire_edge_ids[i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        intersection,
                    );
                    record_chamfer_point(
                        &mut chamfer_data,
                        poly.wire_edge_ids[prev_i].index(),
                        poly.vertex_ids[i],
                        face_id,
                        intersection,
                    );

                    // Track for corner triangle generation.
                    corner_data
                        .entry(poly.vertex_ids[i].index())
                        .or_default()
                        .push((face_id, intersection));
                }
            }
        }

        if new_verts.len() < 3 {
            return Err(unsupported(format!(
                "the bevel collapses face {} to {} corner(s)",
                face_id.index(),
                new_verts.len()
            )));
        }
        // Every corner slid along an edge of the face, so the rebuilt boundary
        // must still lie on the face's own plane; the face therefore keeps its
        // surface exactly, rather than being handed a freshly derived one.
        let plane_d = dot_normal_point(poly.normal, poly.positions[0]);
        for p in &new_verts {
            if (dot_normal_point(poly.normal, *p) - plane_d).abs() > eps {
                return Err(unsupported(format!(
                    "re-trimming face {} against the bevel would pull it off its own \
                     plane",
                    face_id.index()
                )));
            }
        }
        require_holes_clear_of_trim(topo, face_id, poly, &new_verts, eps)?;
        // Replacement outer wire; inner wires copied verbatim, holes and all.
        push_face_spec(
            &mut result_specs,
            &mut result_spec_origins,
            FaceSpec::Existing {
                face: face_id,
                outer: Some(new_verts),
            },
            FaceSpecOrigin::Modified(face_id),
        );
    }

    for &edge_id in &filtered_edges {
        let data = chamfer_data.get(&edge_id.index()).ok_or_else(|| {
            crate::OperationsError::InvalidInput {
                reason: format!(
                    "failed to compute chamfer data for edge {}",
                    edge_id.index()
                ),
            }
        })?;

        let edge = topo.edge(edge_id)?;
        let v_start = edge.start();
        let v_end = edge.end();

        let face_list = &edge_to_faces[&edge_id.index()];
        let f1 = face_list[0];
        let f2 = face_list[1];

        let c1_start = data.get_point(f1, v_start)?;
        let c1_end = data.get_point(f1, v_end)?;
        let c2_start = data.get_point(f2, v_start)?;
        let c2_end = data.get_point(f2, v_end)?;

        // Build the chamfer quad. Prefer the structural answer: the bevel must
        // traverse its shared contact edge opposite to the rebuilt adjacent
        // face. This stays well-conditioned on a near-flat concave ridge where
        // the two outward normals nearly cancel. Fall back to the normal test
        // only if the positional spec no longer exposes that contact segment.
        let n1 = face_polygons[&f1.index()].outward;
        let n2 = face_polygons[&f2.index()].outward;
        let avg_normal = n1 + n2;

        let edge_a = c2_start - c1_start;
        let edge_b = c1_end - c1_start;
        let raw_normal = edge_a.cross(edge_b);

        let f1_spec = result_spec_origins
            .iter()
            .zip(&result_specs)
            .find_map(|(origin, spec)| {
                matches!(origin, FaceSpecOrigin::Modified(source) if *source == f1).then_some(spec)
            });
        let f1_runs_start_to_end =
            f1_spec.is_some_and(|spec| has_directed_segment(spec, c1_start, c1_end, eps));
        let f1_runs_end_to_start =
            f1_spec.is_some_and(|spec| has_directed_segment(spec, c1_end, c1_start, eps));
        // Preserve the fork's fail-closed convex modifier contract. The
        // structural repair is specific to concave notches; convex and
        // unclassified edges keep the established normal-based winding and
        // its existing validation refusal where that construction is unsound.
        let edge_is_concave = convexity_by_edge.get(&edge_id.index()) == Some(&false);
        let use_raw_winding = if edge_is_concave && f1_runs_start_to_end {
            true
        } else if edge_is_concave && f1_runs_end_to_start {
            false
        } else {
            raw_normal.dot(avg_normal) >= 0.0
        };

        let quad = if use_raw_winding {
            vec![c1_start, c2_start, c2_end, c1_end]
        } else {
            vec![c1_start, c1_end, c2_end, c2_start]
        };
        // The boundary order above is constrained by shared-edge traversal,
        // while the plane normal is constrained by which side is outside the
        // solid. Those are independent on a concave edge: forcing the surface
        // normal to follow the structurally correct wire points it into the
        // material and corrupts signed volume by the whole bevel-face prism.
        let normal = if raw_normal.dot(avg_normal) >= 0.0 {
            raw_normal.normalize()?
        } else {
            (-raw_normal).normalize()?
        };

        let d = dot_normal_point(normal, quad[0]);
        push_face_spec(
            &mut result_specs,
            &mut result_spec_origins,
            FaceSpec::Planar {
                vertices: quad,
                normal,
                d,
                inner_wires: vec![],
            },
            FaceSpecOrigin::Generated(vec![f1, f2]),
        );
    }

    // At each original vertex where ALL adjacent edges are chamfered,
    // the trim-plane intersections from each face create a polygonal gap
    // (triangle for box vertices, k-gon for degree-k vertices).
    for (vid, entries) in corner_data {
        // Only create a corner face if ALL faces at this vertex contributed
        // (i.e. all edges at this vertex are chamfered).
        let expected = vertex_face_count.get(&vid).copied().unwrap_or(0);
        if entries.len() != expected || entries.len() < 3 {
            continue;
        }

        // Which way is out, taken from the faces that meet at this corner
        // rather than from the body's centroid: a corner of a pocket faces
        // into the cavity, and a centroid cannot tell that apart from a
        // corner of the block. Each face contributes its outward normal, so
        // reversed faces point the patch the right way round.
        let outward = entries
            .iter()
            .fold(Vec3::new(0.0, 0.0, 0.0), |acc, (fid, _)| {
                acc + face_polygons[&fid.index()].outward
            })
            .normalize()
            .map_err(|_| {
                unsupported(format!(
                    "the faces meeting at corner {vid} face in opposing directions, so \
                     the corner patch closing the bevels there has no outward side"
                ))
            })?;

        // For a triangle (3 entries), compute the normal and ensure it agrees
        // with the outward direction worked out above.
        let pts: Vec<Point3> = entries.iter().map(|(_, p)| *p).collect();
        let e1 = pts[1] - pts[0];
        let e2 = pts[2] - pts[0];
        let tri_normal = e1.cross(e2);

        let mut corner_verts: Vec<Point3> = if tri_normal.dot(outward) >= 0.0 {
            pts
        } else {
            let mut rev = pts;
            rev.reverse();
            rev
        };

        // For k > 3 corner faces, sort by angle around the outward axis.
        if corner_verts.len() > 3 {
            let center = Point3::new(
                corner_verts.iter().map(|p| p.x()).sum::<f64>() / corner_verts.len() as f64,
                corner_verts.iter().map(|p| p.y()).sum::<f64>() / corner_verts.len() as f64,
                corner_verts.iter().map(|p| p.z()).sum::<f64>() / corner_verts.len() as f64,
            );
            // Pick a reference direction in the corner face plane.
            let ref_dir = (corner_verts[0] - center)
                .normalize()
                .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
            let binormal = outward.cross(ref_dir);
            corner_verts.sort_by(|a, b| {
                let da = *a - center;
                let db = *b - center;
                let angle_a = da.dot(binormal).atan2(da.dot(ref_dir));
                let angle_b = db.dot(binormal).atan2(db.dot(ref_dir));
                angle_a
                    .partial_cmp(&angle_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            // Re-check winding after sort.
            let se1 = corner_verts[1] - corner_verts[0];
            let se2 = corner_verts[2] - corner_verts[0];
            if se1.cross(se2).dot(outward) < 0.0 {
                corner_verts.reverse();
            }
        }

        let cn = {
            let ce1 = corner_verts[1] - corner_verts[0];
            let ce2 = corner_verts[2] - corner_verts[0];
            ce1.cross(ce2)
                .normalize()
                .unwrap_or(Vec3::new(0.0, 0.0, 1.0))
        };
        let cd = dot_normal_point(cn, corner_verts[0]);
        // Newly minted geometry: a corner patch is a fresh polygon spanning
        // the gap the bevels leave, so it has no hole to carry.
        push_face_spec(
            &mut result_specs,
            &mut result_spec_origins,
            FaceSpec::Planar {
                vertices: corner_verts,
                normal: cn,
                d: cd,
                inner_wires: vec![],
            },
            FaceSpecOrigin::Generated(entries.iter().map(|(face, _)| *face).collect()),
        );
    }

    let assembly = assemble_solid_mixed_with_history(topo, &result_specs, tol)?;
    let result = assembly.solid;

    let report = crate::validate::validate_solid(topo, result)?;
    if !report.is_valid() {
        let detail: Vec<&str> = report
            .issues
            .iter()
            .filter(|i| i.severity == crate::validate::Severity::Error)
            .map(|i| i.description.as_str())
            .collect();
        return Err(unsupported(format!(
            "chamfered shell failed validation ({})",
            detail.join("; ")
        )));
    }
    // A shell can pass the structural checks and still be turned inside out,
    // and a chamfer that comes back no smaller than its input did not cut a
    // bevel — it folded, or it filled something in.
    let after = crate::measure::solid_volume(topo, result, VOLUME_DEFLECTION)?;
    if !after.is_finite() || after <= 0.0 {
        return Err(unsupported(format!(
            "chamfered shell encloses no volume ({after}); the bevel turned the body \
             inside out"
        )));
    }
    if all_convex && after >= before {
        return Err(unsupported(format!(
            "a chamfer removes material, but the result encloses {after} against the \
             input's {before}"
        )));
    }
    if all_concave && after <= before {
        return Err(unsupported(format!(
            "a chamfer on concave edges adds material, but the result encloses {after} \
             against the input's {before}"
        )));
    }

    let mut face_origins = BlendFaceOrigins::default();
    let mut named: HashSet<usize> = HashSet::new();
    let mut modified_presence: HashMap<usize, (FaceId, bool)> = HashMap::new();
    for (origin, face) in result_spec_origins.iter().zip(&assembly.faces_by_spec) {
        if let FaceSpecOrigin::Modified(source) = origin {
            modified_presence
                .entry(source.index())
                .and_modify(|(_, present)| *present |= face.is_some())
                .or_insert_with(|| (*source, face.is_some()));
        }
        let Some(face) = face else { continue };
        named.insert(face.index());
        match origin {
            FaceSpecOrigin::Modified(source) => face_origins.survived.push((*source, *face)),
            FaceSpecOrigin::Generated(sources) => {
                let mut sources = sources.clone();
                sources.sort_by_key(|source| source.index());
                sources.dedup();
                if sources.is_empty() {
                    face_origins.created_unattributed.push(*face);
                } else {
                    face_origins.created.push((*face, sources));
                }
            }
        }
    }
    face_origins.deleted = modified_presence
        .into_values()
        .filter_map(|(source, present)| (!present).then_some(source))
        .collect();
    face_origins.deleted.sort_by_key(|source| source.index());
    for face in brepkit_topology::explorer::solid_faces(topo, result)? {
        if !named.contains(&face.index()) {
            face_origins.created_unattributed.push(face);
        }
    }

    Ok((result, face_origins))
}

/// Per-face polygon data collected from the solid.
struct FacePolygon {
    vertex_ids: Vec<VertexId>,
    positions: Vec<Point3>,
    /// The `EdgeId` for each wire edge: `wire_edge_ids[i]` connects
    /// `vertex_ids[i]` to `vertex_ids[(i+1) % n]`.
    wire_edge_ids: Vec<EdgeId>,
    /// The face's *stored* surface normal. The outer wire winds CCW about this
    /// vector whether or not the face is reversed, so every in-plane
    /// construction below (`inward = normal x direction`) refers to it.
    normal: Vec3,
    /// The face's outward normal — [`Self::normal`] flipped when the face is
    /// reversed. What the bevel has to face away from.
    outward: Vec3,
}

/// Bounding-box diagonal over every planar face corner, used to scale the
/// "is this point still on that plane" slack with the model.
fn model_span(face_polygons: &HashMap<usize, FacePolygon>) -> f64 {
    let mut bounds: Option<(Point3, Point3)> = None;
    for poly in face_polygons.values() {
        for &p in &poly.positions {
            bounds = Some(match bounds {
                None => (p, p),
                Some((lo, hi)) => (
                    Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z())),
                    Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z())),
                ),
            });
        }
    }
    bounds.map_or(0.0, |(lo, hi)| (hi - lo).length())
}

/// Refuse a face whose boundary the bevel must rebuild when that boundary is
/// not made of straight edges to begin with.
///
/// The rebuild describes a wire as a list of corner positions, so the
/// assembler can only mint chords between them; an arc in the boundary would
/// come back flattened.
fn require_line_outer_wire(topo: &Topology, fid: FaceId) -> Result<(), OperationsError> {
    let wire = topo.face(fid)?.outer_wire();
    for oe in topo.wire(wire)?.edges() {
        if !matches!(topo.edge(oe.edge())?.curve(), EdgeCurve::Line) {
            return Err(unsupported(format!(
                "the chamfer has to rebuild the boundary of face {}, which carries a \
                 curved edge; re-trimming a curved boundary against the bevel is not \
                 implemented",
                fid.index()
            )));
        }
    }
    Ok(())
}

/// The points of `wire`, sampling any closed curve edge.
///
/// A drilled hole's rim is a single closed circle edge whose start and end are
/// the same vertex — one position. Testing that one point against the trimmed
/// boundary would say nothing about the rest of the circle, so closed curves
/// are sampled the way the boolean pipeline samples them.
fn wire_points(topo: &Topology, wire: WireId) -> Result<Vec<Point3>, OperationsError> {
    let mut pts = Vec::new();
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        let closed = edge.start() == edge.end()
            && matches!(
                edge.curve(),
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
            );
        if closed {
            pts.extend(crate::boolean::sample_edge_curve(
                edge.curve(),
                HOLE_SAMPLES,
            ));
        } else {
            pts.push(topo.vertex(oe.oriented_start(edge))?.point());
        }
    }
    Ok(pts)
}

/// An orthonormal basis of the plane with normal `n`.
fn plane_basis(n: Vec3) -> Result<(Vec3, Vec3), OperationsError> {
    let seed = if n.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = n.cross(seed).normalize()?;
    Ok((u, n.cross(u)))
}

/// Whether `p` is inside the closed polygon `poly` by the even-odd rule.
fn point_in_polygon(p: (f64, f64), poly: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[(i + n - 1) % n];
        if (yi > p.1) != (yj > p.1) {
            let t = (p.1 - yi) / (yj - yi);
            if p.0 < t.mul_add(xj - xi, xi) {
                inside = !inside;
            }
        }
    }
    inside
}

/// Whether the closed segments `a[i]→a[i+1]` and `b[j]→b[j+1]` ever cross.
fn polylines_cross(a: &[(f64, f64)], b: &[(f64, f64)]) -> bool {
    let cross = |o: (f64, f64), p: (f64, f64), q: (f64, f64)| {
        (p.0 - o.0).mul_add(q.1 - o.1, -((p.1 - o.1) * (q.0 - o.0)))
    };
    let straddles = |p1, p2, q1, q2| {
        let d1 = cross(q1, q2, p1);
        let d2 = cross(q1, q2, p2);
        let d3 = cross(p1, p2, q1);
        let d4 = cross(p1, p2, q2);
        ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))
    };
    for i in 0..a.len() {
        let (p1, p2) = (a[i], a[(i + 1) % a.len()]);
        for j in 0..b.len() {
            let (q1, q2) = (b[j], b[(j + 1) % b.len()]);
            if straddles(p1, p2, q1, q2) {
                return true;
            }
        }
    }
    false
}

/// Refuse a bevel that would cut into a hole in the face it trims.
///
/// The rebuilt face keeps its inner wires verbatim, which is only honest while
/// those holes still fit inside the trimmed boundary. A setback wide enough to
/// reach a bore would leave the rim floating across the new edge — a face
/// whose hole is no longer in it. That has no exact construction here, so it
/// is named rather than approximated.
fn require_holes_clear_of_trim(
    topo: &Topology,
    face_id: FaceId,
    poly: &FacePolygon,
    new_outer: &[Point3],
    eps: f64,
) -> Result<(), OperationsError> {
    let inner = topo.face(face_id)?.inner_wires().to_vec();
    if inner.is_empty() {
        return Ok(());
    }
    let (u, v) = plane_basis(poly.normal)?;
    let flatten = |p: Point3| (dot_normal_point(u, p), dot_normal_point(v, p));
    let outer2: Vec<(f64, f64)> = new_outer.iter().map(|&p| flatten(p)).collect();

    for (slot, &wid) in inner.iter().enumerate() {
        let pts = wire_points(topo, wid)?;
        let inner2: Vec<(f64, f64)> = pts.iter().map(|&p| flatten(p)).collect();
        if inner2.len() < 3 {
            continue;
        }
        let escaped = inner2.iter().any(|p| !point_in_polygon(*p, &outer2));
        if escaped || polylines_cross(&inner2, &outer2) {
            return Err(unsupported(format!(
                "the bevel cuts into inner wire {slot} of face {}; the setback reaches \
                 the hole, so the trimmed face can no longer carry it",
                face_id.index()
            )));
        }
        // A hole that merely grazes the new boundary is no better: the rim and
        // the trimmed edge would land on top of each other.
        let touching = inner2.iter().any(|p| {
            (0..outer2.len()).any(|i| {
                point_segment_distance(*p, outer2[i], outer2[(i + 1) % outer2.len()]) <= eps
            })
        });
        if touching {
            return Err(unsupported(format!(
                "the bevel lands on inner wire {slot} of face {}; the trimmed boundary \
                 and the hole's rim coincide",
                face_id.index()
            )));
        }
    }
    Ok(())
}

/// Distance from `p` to the segment `a`–`b`.
fn point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len_sq = dx.mul_add(dx, dy * dy);
    let t = if len_sq <= f64::MIN_POSITIVE {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len_sq).clamp(0.0, 1.0)
    };
    let (qx, qy) = (t.mul_add(dx, a.0), t.mul_add(dy, a.1));
    (p.0 - qx).hypot(p.1 - qy)
}

/// The setback that governs how far a side-face corner travels along one of
/// its own edges.
///
/// `wire_edge` is an edge of `this_face` running out of `vertex`, and `vertex`
/// is the endpoint of some chamfered edge. The face on the far side of
/// `wire_edge` is the one being bevelled there, so its setback is the distance
/// the split point must travel to meet the chamfer point that face placed on
/// the same edge.
///
/// Returns `None` when the answer would be a guess: more than one chamfered
/// edge meeting the vertex (so which bevel governs is ambiguous), or a
/// non-manifold wire edge. Callers fall back to the previous behaviour there.
fn neighbour_setback(
    wire_edge: EdgeId,
    this_face: FaceId,
    vertex: VertexId,
    vertex_chamfer_edges: &HashMap<usize, Vec<EdgeId>>,
    edge_to_faces: &HashMap<usize, Vec<FaceId>>,
    distances: &ChamferDistances,
) -> Option<f64> {
    let chamfered = vertex_chamfer_edges.get(&vertex.index())?;
    let [chamfered_edge] = chamfered.as_slice() else {
        return None;
    };

    let faces = edge_to_faces.get(&wire_edge.index())?;
    let [a, b] = faces.as_slice() else {
        return None;
    };
    let neighbour = if *a == this_face {
        *b
    } else if *b == this_face {
        *a
    } else {
        return None;
    };

    Some(distances.distance_for(chamfered_edge.index(), neighbour, edge_to_faces))
}

/// How far a face vertex slides along each of its two incident wire edges.
///
/// Mirrors the offsetting arms in [`chamfer_core`] one-for-one, so the check
/// stays true to what the builder will actually do.
#[derive(Debug, Clone, Copy, Default)]
struct VertexSlide {
    /// Distance travelled toward the previous vertex, i.e. consumed from the
    /// wire edge entering this vertex.
    toward_prev: f64,
    /// Distance travelled toward the next vertex, i.e. consumed from the wire
    /// edge leaving this vertex.
    toward_next: f64,
}

/// Shared lookups for the setback fit check.
struct SetbackCheck<'a> {
    edge_to_faces: &'a HashMap<usize, Vec<FaceId>>,
    target_set: &'a HashSet<usize>,
    distances: &'a ChamferDistances,
    vertex_chamfer_endpoints: &'a HashSet<usize>,
    vertex_max_distance: &'a HashMap<usize, f64>,
    vertex_chamfer_edges: &'a HashMap<usize, Vec<EdgeId>>,
}

impl SetbackCheck<'_> {
    /// How far vertex `i` of `poly` slides along its incident wire edges.
    fn vertex_slide(&self, poly: &FacePolygon, i: usize, face_id: FaceId) -> VertexSlide {
        let n = poly.positions.len();
        let prev_i = if i == 0 { n - 1 } else { i - 1 };

        let before_chamfered = self
            .target_set
            .contains(&poly.wire_edge_ids[prev_i].index());
        let after_chamfered = self.target_set.contains(&poly.wire_edge_ids[i].index());
        let at_endpoint = self
            .vertex_chamfer_endpoints
            .contains(&poly.vertex_ids[i].index());

        match (before_chamfered, after_chamfered, at_endpoint) {
            // Untouched vertex.
            (false, false, false) => VertexSlide::default(),
            // Side-face corner: splits into two points, one along each edge,
            // each travelling by the setback of the face across that edge.
            (false, false, true) => {
                let fallback = self
                    .vertex_max_distance
                    .get(&poly.vertex_ids[i].index())
                    .copied()
                    .unwrap_or_else(|| self.distances.max_distance());
                let side_dist = |wire_edge: EdgeId| -> f64 {
                    neighbour_setback(
                        wire_edge,
                        face_id,
                        poly.vertex_ids[i],
                        self.vertex_chamfer_edges,
                        self.edge_to_faces,
                        self.distances,
                    )
                    .unwrap_or(fallback)
                };
                VertexSlide {
                    toward_prev: side_dist(poly.wire_edge_ids[prev_i]),
                    toward_next: side_dist(poly.wire_edge_ids[i]),
                }
            }
            // Only the entering edge is chamfered: slides toward the next vertex.
            (true, false, _) => VertexSlide {
                toward_prev: 0.0,
                toward_next: self.distances.distance_for(
                    poly.wire_edge_ids[prev_i].index(),
                    face_id,
                    self.edge_to_faces,
                ),
            },
            // Only the leaving edge is chamfered: slides toward the previous vertex.
            (false, true, _) => VertexSlide {
                toward_prev: self.distances.distance_for(
                    poly.wire_edge_ids[i].index(),
                    face_id,
                    self.edge_to_faces,
                ),
                toward_next: 0.0,
            },
            // Both adjacent edges chamfered: the vertex moves inward to where
            // the two trim lines meet. That point still displaces along both
            // incident edges, and by more than the setback once the corner is
            // acute — so the displacement is computed exactly rather than
            // approximated by the distances themselves.
            (true, true, _) => {
                let next_i = (i + 1) % n;
                let dist_after = self.distances.distance_for(
                    poly.wire_edge_ids[i].index(),
                    face_id,
                    self.edge_to_faces,
                );
                let dist_before = self.distances.distance_for(
                    poly.wire_edge_ids[prev_i].index(),
                    face_id,
                    self.edge_to_faces,
                );

                let pos = poly.positions[i];
                let (Ok(u), Ok(v)) = (
                    (pos - poly.positions[prev_i]).normalize(),
                    (poly.positions[next_i] - pos).normalize(),
                ) else {
                    return VertexSlide::default();
                };

                // The trim lines are offset from each incident edge by its own
                // setback; their intersection sits `d/sin` along each edge,
                // skewed by how far the corner is from square.
                let cos_phi = u.dot(v);
                let sin_phi = u.cross(v).length();
                if sin_phi < 1e-9 {
                    // Collinear edges: the builder falls back to a midpoint
                    // average, which does not slide along either edge.
                    return VertexSlide::default();
                }

                VertexSlide {
                    toward_prev: (dist_before.mul_add(cos_phi, -dist_after) / sin_phi).abs(),
                    toward_next: (dist_after.mul_add(cos_phi, -dist_before) / sin_phi).abs(),
                }
            }
        }
    }

    /// Reject setbacks that do not fit on the faces they slide across.
    ///
    /// Each chamfered edge pushes its neighbouring face vertices along the
    /// wire edges that meet it. If the two ends of one wire edge together
    /// travel at least its whole length, the rebuilt polygon folds through
    /// itself: the result is still a closed, manifold, "valid" solid, but it
    /// is not a chamfer — on a 10 mm box a 40 mm setback came back *larger*
    /// than the input. Catch it here, before the arena is touched, where the
    /// offending distance and the length it overran can both be named.
    fn run(
        &self,
        shell_face_ids: &[FaceId],
        face_polygons: &HashMap<usize, FacePolygon>,
        tol: Tolerance,
    ) -> Result<(), crate::OperationsError> {
        // Walk shell order so the reported edge never depends on hash order.
        for &face_id in shell_face_ids {
            let Some(poly) = face_polygons.get(&face_id.index()) else {
                continue;
            };
            let n = poly.positions.len();
            if n < 3 {
                continue;
            }

            let slides: Vec<VertexSlide> = (0..n)
                .map(|i| self.vertex_slide(poly, i, face_id))
                .collect();

            // Every wire edge is checked, chamfered ones included. A chamfered
            // edge is replaced by its bevel, but the bevel still spans what is
            // left of the original edge after both endpoints have moved; when
            // neighbouring chamfers eat it from both ends the bevel inverts.
            // Endpoints of a chamfered edge only slide along it when the
            // *adjacent* edge is chamfered too, so a lone chamfer scores zero
            // here and is never wrongly refused.
            for i in 0..n {
                let next_i = (i + 1) % n;
                let length = (poly.positions[next_i] - poly.positions[i]).length();
                let consumed = slides[i].toward_next + slides[next_i].toward_prev;

                // Scale the guard with the edge so it means the same thing on
                // a 0.1 mm feature and a 1 m one.
                let slack = tol.linear * length.max(1.0);
                if consumed + slack >= length {
                    return Err(crate::OperationsError::InvalidInput {
                        reason: format!(
                            "chamfer setback does not fit: {consumed:.6} of material must be \
                             taken from an edge only {length:.6} long. Reduce the chamfer \
                             distance below {length:.6}."
                        ),
                    });
                }
            }
        }

        Ok(())
    }
}

/// Chamfer point data collected during polygon rebuilding.
///
/// Maps `(face_index, vertex_index)` → chamfer point position.
struct ChamferEdgeData {
    points: HashMap<(usize, usize), Point3>,
}

impl ChamferEdgeData {
    fn new() -> Self {
        Self {
            points: HashMap::new(),
        }
    }

    fn insert(&mut self, face_id: FaceId, vertex_id: VertexId, point: Point3) {
        self.points
            .insert((face_id.index(), vertex_id.index()), point);
    }

    fn get_point(
        &self,
        face_id: FaceId,
        vertex_id: VertexId,
    ) -> Result<Point3, crate::OperationsError> {
        self.points
            .get(&(face_id.index(), vertex_id.index()))
            .copied()
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason: format!(
                    "missing chamfer point for face {} vertex {}",
                    face_id.index(),
                    vertex_id.index()
                ),
            })
    }
}

/// Record a chamfer point for a target edge at a specific face and vertex.
fn record_chamfer_point(
    data: &mut HashMap<usize, ChamferEdgeData>,
    edge_index: usize,
    vertex_id: VertexId,
    face_id: FaceId,
    point: Point3,
) {
    data.entry(edge_index)
        .or_insert_with(ChamferEdgeData::new)
        .insert(face_id, vertex_id, point);
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic, deprecated)]
mod tests {
    use brepkit_topology::test_utils::make_unit_cube_manifold;
    use brepkit_topology::validation::validate_shell_manifold;

    use super::*;

    /// Helper: get all unique edge IDs from a solid's outer shell.
    fn solid_edge_ids(topo: &Topology, solid_id: SolidId) -> Vec<EdgeId> {
        let solid = topo.solid(solid_id).expect("test solid");
        let shell = topo.shell(solid.outer_shell()).expect("test shell");
        let mut seen = HashSet::new();
        let mut edges = Vec::new();
        for &fid in shell.faces() {
            let face = topo.face(fid).expect("test face");
            let wire = topo.wire(face.outer_wire()).expect("test wire");
            for oe in wire.edges() {
                if seen.insert(oe.edge().index()) {
                    edges.push(oe.edge());
                }
            }
        }
        edges
    }

    #[test]
    fn chamfer_single_edge() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let edges = solid_edge_ids(&topo, cube);
        let target = edges[0];

        let result = chamfer(&mut topo, cube, &[target], 0.2).expect("chamfer should succeed");

        // Original cube has 6 faces. Chamfering one edge modifies 2 faces
        // (they keep same vertex count, just shifted) and adds 1 chamfer face → 7.
        let result_solid = topo.solid(result).expect("result solid");
        let result_shell = topo.shell(result_solid.outer_shell()).expect("shell");
        assert_eq!(
            result_shell.faces().len(),
            7,
            "expected 7 faces after single-edge chamfer"
        );
    }

    #[test]
    fn chamfer_zero_distance_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer(&mut topo, cube, &[edges[0]], 0.0);
        assert!(result.is_err(), "zero distance should fail");
    }

    #[test]
    fn chamfer_negative_distance_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer(&mut topo, cube, &[edges[0]], -0.5);
        assert!(result.is_err(), "negative distance should fail");
    }

    #[test]
    fn chamfer_invalid_edge_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        // Create a stray edge not part of the cube.
        let v0 = topo.add_vertex(brepkit_topology::vertex::Vertex::new(
            Point3::new(99.0, 99.0, 99.0),
            1e-7,
        ));
        let v1 = topo.add_vertex(brepkit_topology::vertex::Vertex::new(
            Point3::new(100.0, 100.0, 100.0),
            1e-7,
        ));
        let stray = topo.add_edge(brepkit_topology::edge::Edge::new(
            v0,
            v1,
            brepkit_topology::edge::EdgeCurve::Line,
        ));

        let result = chamfer(&mut topo, cube, &[stray], 0.2);
        assert!(result.is_err(), "invalid edge should fail");
    }

    #[test]
    fn chamfer_result_is_manifold() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer(&mut topo, cube, &[edges[0]], 0.2).expect("chamfer should succeed");

        let result_solid = topo.solid(result).expect("result solid");
        let result_shell = topo.shell(result_solid.outer_shell()).expect("shell");
        validate_shell_manifold(result_shell, &topo).expect("result should be manifold");
    }

    #[test]
    fn chamfer_parallel_edges() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        // Find two edges that don't share a vertex (parallel on a cube).
        let mut pair = None;
        'outer: for (i, &ea) in edges.iter().enumerate() {
            let data_a = topo.edge(ea).expect("edge");
            let va = [data_a.start().index(), data_a.end().index()];
            for &eb in &edges[i + 1..] {
                let data_b = topo.edge(eb).expect("edge");
                let vb = [data_b.start().index(), data_b.end().index()];
                if !va.iter().any(|v| vb.contains(v)) {
                    pair = Some([ea, eb]);
                    break 'outer;
                }
            }
        }
        let targets = pair.expect("should find non-adjacent edges on a cube");

        let result =
            chamfer(&mut topo, cube, &targets, 0.2).expect("parallel chamfer should succeed");

        // 2 chamfered edges → 2 new chamfer faces.
        // Non-adjacent edges on a cube: 6 original + 2 chamfer = 8 faces.
        let result_solid = topo.solid(result).expect("result solid");
        let result_shell = topo.shell(result_solid.outer_shell()).expect("shell");
        assert_eq!(
            result_shell.faces().len(),
            8,
            "expected 8 faces after 2 non-adjacent chamfers"
        );

        validate_shell_manifold(result_shell, &topo).expect("result should be manifold");
    }

    /// Chamfer all 12 edges of a 10³ box with d=1.0.
    ///
    /// Volume derivation for 10³ box chamfered at d=1:
    ///   Each edge removes a right-triangular prism with legs d, length L-2d=8:
    ///     12 × (d²/2) × (L-2d) = 12 × 0.5 × 8 = 48
    ///   Each corner removes a tetrahedron with volume d³/6:
    ///     8 × (1/6) ≈ 1.333
    ///   Total removed ≈ 49.333, expected ≈ 950.7
    ///
    /// Use 5% tolerance to account for implementation variations in corner
    /// treatment (the actual value depends on whether edge prisms use full
    /// edge length L=10 or trimmed L-2d=8).
    #[test]
    fn chamfer_all_edges_volume() {
        let mut topo = Topology::new();
        let cube = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edge_ids(&topo, cube);

        assert_eq!(edges.len(), 12, "box should have 12 edges");
        let result = chamfer(&mut topo, cube, &edges, 1.0).unwrap();

        let s = topo.solid(result).unwrap();
        let sh = topo.shell(s.outer_shell()).unwrap();

        // 6 trimmed faces + 12 chamfer strips + 8 corner triangles = 26 faces.
        assert_eq!(sh.faces().len(), 26, "chamfered box should have 26 faces");

        let vol = crate::measure::solid_volume(&topo, result, 0.1).unwrap();
        // Expected ≈ 950.7 (see doc comment). 5% tolerance window
        // covers the range [903, 998], catching gross errors while
        // allowing for implementation variations in corner treatment.
        let expected = 950.0;
        let rel_err = (vol - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "chamfered 10³ box with d=1 should have volume ~{expected}, got {vol} \
             (rel_err={rel_err:.2e}). Was previously 800-1000 tolerance."
        );
    }

    /// Single-edge chamfer on a unit cube: d=0.2.
    ///
    /// Removes a right-triangular prism: legs = 0.2, length = 1.0.
    /// V_removed = (0.2²/2) × 1.0 = 0.02.
    /// V_expected = 1.0 - 0.02 = 0.98.
    #[test]
    fn chamfer_single_edge_volume() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer(&mut topo, cube, &[edges[0]], 0.2).unwrap();

        let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
        // V = 1.0 - (0.2²/2 × 1.0) = 1.0 - 0.02 = 0.98
        let expected = 0.98;
        let rel_err = (vol - expected).abs() / expected;
        assert!(
            rel_err < 1e-4,
            "single-edge chamfer d=0.2 on unit cube: expected {expected}, got {vol} \
             (rel_err={rel_err:.2e})"
        );
    }

    #[test]
    fn chamfer_asymmetric_single_edge() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer_asymmetric(&mut topo, cube, &[edges[0]], 0.2, 0.3)
            .expect("asymmetric chamfer should succeed");

        let result_solid = topo.solid(result).expect("result solid");
        let result_shell = topo.shell(result_solid.outer_shell()).expect("shell");
        assert_eq!(
            result_shell.faces().len(),
            7,
            "expected 7 faces after single-edge chamfer"
        );

        validate_shell_manifold(result_shell, &topo).expect("result should be manifold");
    }

    /// Asymmetric single-edge chamfer volume on a unit cube.
    ///
    /// Removes a right-triangular prism with legs d1=0.2 and d2=0.3 over the
    /// edge's length 1.0, so `V = 1 - (d1·d2/2)·L = 0.97` exactly. There is
    /// nothing else to take: a chamfer of one edge touches no other feature.
    ///
    /// This previously expected 0.965 and explained the 0.005 as "extra
    /// triangular wedges" from the side-face corners using `max(d1, d2)`.
    /// Those wedges were the defect, not a design choice — the same offsets
    /// left the shell open with 6 free edges and Euler 0. The test only ever
    /// measured volume, so it passed on a solid that was not closed, which is
    /// how the bug survived. It now checks closure too.
    #[test]
    fn chamfer_asymmetric_single_edge_volume() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer_asymmetric(&mut topo, cube, &[edges[0]], 0.2, 0.3).unwrap();

        let report = brepkit_check::validate::validate_solid(
            &topo,
            result,
            &brepkit_check::validate::ValidateOptions::default(),
        )
        .expect("validation should run");
        assert!(
            report.is_valid(),
            "asymmetric chamfer must produce a closed solid: {:#?}",
            report.issues
        );

        let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
        let expected = 1.0 - 0.5 * 0.2 * 0.3;
        assert!(
            (vol - expected).abs() < 1e-9,
            "asymmetric chamfer d1=0.2, d2=0.3 on unit cube: expected {expected}, got {vol}"
        );
    }

    /// Asymmetric chamfer with d1 == d2 should match symmetric chamfer.
    #[test]
    fn chamfer_asymmetric_equal_matches_symmetric() {
        let mut topo_sym = Topology::new();
        let cube_sym = make_unit_cube_manifold(&mut topo_sym);
        let edges_sym = solid_edge_ids(&topo_sym, cube_sym);
        let result_sym = chamfer(&mut topo_sym, cube_sym, &[edges_sym[0]], 0.2).unwrap();
        let vol_sym = crate::measure::solid_volume(&topo_sym, result_sym, 0.01).unwrap();

        let mut topo_asym = Topology::new();
        let cube_asym = make_unit_cube_manifold(&mut topo_asym);
        let edges_asym = solid_edge_ids(&topo_asym, cube_asym);
        let result_asym =
            chamfer_asymmetric(&mut topo_asym, cube_asym, &[edges_asym[0]], 0.2, 0.2).unwrap();
        let vol_asym = crate::measure::solid_volume(&topo_asym, result_asym, 0.01).unwrap();

        let rel_err = (vol_sym - vol_asym).abs() / vol_sym;
        assert!(
            rel_err < 1e-6,
            "asymmetric(d,d) should match symmetric(d): sym={vol_sym}, asym={vol_asym} \
             (rel_err={rel_err:.2e})"
        );
    }

    #[test]
    fn chamfer_asymmetric_zero_d1_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer_asymmetric(&mut topo, cube, &[edges[0]], 0.0, 0.3);
        assert!(result.is_err(), "zero d1 should fail");
    }

    #[test]
    fn chamfer_asymmetric_negative_d2_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let edges = solid_edge_ids(&topo, cube);

        let result = chamfer_asymmetric(&mut topo, cube, &[edges[0]], 0.2, -0.1);
        assert!(result.is_err(), "negative d2 should fail");
    }

    /// A setback wider than the face it cuts across is refused outright
    /// rather than folding the polygon through itself.
    ///
    /// On the unit cube every wire edge is 1 long, so any setback at or above
    /// 1 overruns. This is the unit-level guard behind the
    /// `regress_failed_blend_leaves_input_intact` regression, where an
    /// oversized chamfer used to come back as a *larger* solid that still
    /// passed `validate_solid`.
    #[test]
    fn chamfer_setback_wider_than_the_face_is_refused() {
        for d in [1.0, 1.5, 4.0] {
            let mut topo = Topology::new();
            let cube = make_unit_cube_manifold(&mut topo);
            let edges = solid_edge_ids(&topo, cube);

            let result = chamfer(&mut topo, cube, &[edges[0]], d);
            match result {
                Err(e) => assert!(
                    e.to_string().contains("does not fit"),
                    "d={d} should be refused by the fit check, got: {e}"
                ),
                Ok(_) => panic!("d={d} overruns a unit face and must be refused"),
            }
        }
    }

    /// The guard must not narrow the working range: setbacks that do fit are
    /// still accepted and still remove exactly the right prism.
    #[test]
    fn chamfer_setback_that_fits_is_still_accepted() {
        for d in [0.05, 0.25, 0.49] {
            let mut topo = Topology::new();
            let cube = make_unit_cube_manifold(&mut topo);
            let edges = solid_edge_ids(&topo, cube);

            let result = chamfer(&mut topo, cube, &[edges[0]], d)
                .unwrap_or_else(|e| panic!("d={d} fits on a unit face but was refused: {e}"));
            let volume = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
            // One edge of a unit cube: a triangular prism of 0.5*d*d*1.
            let expected = 1.0 - 0.5 * d * d;
            assert!(
                (volume - expected).abs() < 1e-9,
                "d={d}: expected {expected}, got {volume}"
            );
        }
    }

    /// A lopsided chamfer must apply *both* setbacks, not one of them twice.
    ///
    /// The side faces at each end of a chamfered edge split their corner into
    /// two points, one per adjacent face. Those points have to land on the
    /// chamfer points the neighbouring faces placed, which sit at that face's
    /// own setback — so the split has to use d1 in one direction and d2 in the
    /// other. Using a single distance for both (previously `max(d1, d2)`) tore
    /// the shell open at every asymmetric chamfer.
    ///
    /// Volume is the oracle: `1 - (d1·d2/2)·L` is only reached when both
    /// setbacks are honoured. Using max twice would give `1 - max²/2`, using
    /// min twice `1 - min²/2`; at 40:1 those are far apart and easy to tell.
    #[test]
    fn asymmetric_chamfer_applies_both_setbacks() {
        for (d1, d2) in [(0.2, 0.3), (0.05, 0.4), (0.4, 0.05), (0.02, 0.8)] {
            let mut topo = Topology::new();
            let cube = make_unit_cube_manifold(&mut topo);
            let edges = solid_edge_ids(&topo, cube);

            let result = chamfer_asymmetric(&mut topo, cube, &[edges[0]], d1, d2)
                .unwrap_or_else(|e| panic!("d1={d1}, d2={d2} should chamfer cleanly: {e}"));

            let report = brepkit_check::validate::validate_solid(
                &topo,
                result,
                &brepkit_check::validate::ValidateOptions::default(),
            )
            .expect("validation should run");
            assert!(
                report.is_valid(),
                "d1={d1}, d2={d2}: shell must close, got {:#?}",
                report.issues
            );

            let vol = crate::measure::solid_volume(&topo, result, 0.01).unwrap();
            let expected = 1.0 - 0.5 * d1 * d2;
            assert!(
                (vol - expected).abs() < 1e-9,
                "d1={d1}, d2={d2}: expected {expected}, got {vol} \
                 (max-twice would give {}, min-twice {})",
                1.0 - 0.5 * d1.max(d2) * d1.max(d2),
                1.0 - 0.5 * d1.min(d2) * d1.min(d2)
            );
        }
    }

    /// Swapping the two distances mirrors the bevel; it must not change how
    /// much material comes off, and both orders must close.
    #[test]
    fn asymmetric_chamfer_is_order_independent_in_volume() {
        let measure = |d1: f64, d2: f64| {
            let mut topo = Topology::new();
            let cube = make_unit_cube_manifold(&mut topo);
            let edges = solid_edge_ids(&topo, cube);
            let result = chamfer_asymmetric(&mut topo, cube, &[edges[0]], d1, d2).unwrap();
            let report = brepkit_check::validate::validate_solid(
                &topo,
                result,
                &brepkit_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert!(report.is_valid(), "d1={d1}, d2={d2} must close");
            crate::measure::solid_volume(&topo, result, 0.01).unwrap()
        };

        let forward = measure(0.15, 0.45);
        let reversed = measure(0.45, 0.15);
        assert!(
            (forward - reversed).abs() < 1e-9,
            "swapping d1/d2 must remove the same volume: {forward} vs {reversed}"
        );
    }
}
