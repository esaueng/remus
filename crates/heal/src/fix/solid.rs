//! Solid fixing — top-level fix orchestration.
//!
//! Orchestrates shell, face, wire, and edge fixes, plus solid-level
//! repairs such as coincident vertex merging and small face removal.

use std::collections::HashMap;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use super::FixResult;
use super::config::FixConfig;
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Fix a solid: orchestrates shell, face, wire, and edge fixes.
///
/// 1. Fixes the outer shell (face-level + orientation).
/// 2. If `config.fix_coincident_vertices` permits, merges coincident
///    vertices across the solid.
/// 3. If `config.fix_small_faces` permits, removes degenerate small faces.
/// 4. If `config.fix_duplicate_faces` permits, removes geometrically
///    coincident duplicate faces.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn fix_solid(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    let mut result = FixResult::ok();

    let solid = topo.solid(solid_id)?;
    let shell_id = solid.outer_shell();

    let shell_result = super::shell::fix_shell(topo, shell_id, ctx, config)?;
    result.merge(&shell_result);

    let solid = topo.solid(solid_id)?;
    let inner_shells: Vec<_> = solid.inner_shells().to_vec();
    for shell_id in inner_shells {
        let inner_result = super::shell::fix_shell(topo, shell_id, ctx, config)?;
        result.merge(&inner_result);
    }

    // Always detected (cheap), applied per config mode.
    let should_merge = config.fix_coincident_vertices.should_fix(true);
    if should_merge {
        let merge_result = merge_coincident_vertices(topo, solid_id, ctx)?;
        result.merge(&merge_result);
    }

    let should_fix_small = config.fix_small_faces.should_fix(true);
    if should_fix_small {
        let small_result = super::small_face::fix_small_faces(topo, solid_id, ctx, config)?;
        result.merge(&small_result);
    }

    let should_fix_dupes = config.fix_duplicate_faces.should_fix(true);
    if should_fix_dupes {
        let dup_result = fix_duplicate_faces(topo, solid_id, ctx)?;
        result.merge(&dup_result);
    }

    Ok(result)
}

/// Cosine threshold for treating two face normals as parallel/anti-parallel.
/// Fixed (not derived from the model's linear tolerance) so a coarse linear
/// tolerance can't widen the angular test into matching clearly-different
/// orientations: `1 - cos θ ≈ θ²/2`, so 1e-6 ≈ 0.08°.
const NORMAL_PARALLEL_COS_TOL: f64 = 1e-6;

/// Detect and remove geometrically duplicate faces in a solid's outer shell.
///
/// This conservative pass only compares unperforated planar polygon faces.
/// Their ordered outer-boundary vertices must coincide with the same winding
/// (allowing a cyclic shift), and their effective normals must agree. Oppositely
/// oriented coincident faces are preserved: removing one would silently choose
/// a side of a zero-thickness or otherwise malformed region. Curved edges and
/// surfaces, NURBS, and perforated faces are skipped because proving their
/// trimmed regions equal requires a parameter-space comparison.
///
/// This must be a solid-scoped pass: a duplicate can only be found by comparing
/// a face against the others, so a per-face fix (which sees one face in
/// isolation) is structurally unable to detect it.
fn fix_duplicate_faces(
    topo: &Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let tol = ctx.tolerance.linear;

    let solid_data = topo.solid(solid_id)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let face_ids: Vec<_> = shell.faces().to_vec();

    // (face, plane normal, ordered outer-boundary vertices).
    let mut face_data: Vec<(FaceId, Vec3, Vec<Point3>)> = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        if !face.inner_wires().is_empty() {
            continue;
        }
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            continue;
        };
        let normal = if face.is_reversed() {
            -*normal
        } else {
            *normal
        };

        let wire = topo.wire(face.outer_wire())?;
        let mut points = Vec::with_capacity(wire.edges().len());
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
                points.clear();
                break;
            }
            points.push(topo.vertex(oe.oriented_start(edge))?.point());
        }
        if !points.is_empty() {
            face_data.push((fid, normal, points));
        }
    }

    // O(n²) pair scan — mark the later face of each duplicate pair.
    let mut duplicates: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for i in 0..face_data.len() {
        if duplicates.contains(&face_data[i].0.index()) {
            continue;
        }
        for j in (i + 1)..face_data.len() {
            if duplicates.contains(&face_data[j].0.index()) {
                continue;
            }
            let (_, na, points_a) = &face_data[i];
            let (fid_j, nb, points_b) = &face_data[j];
            if points_a.len() != points_b.len() {
                continue;
            }
            if na.dot(*nb) < 1.0 - NORMAL_PARALLEL_COS_TOL {
                continue;
            }
            if boundaries_coincide_with_same_winding(points_a, points_b, tol) {
                duplicates.insert(fid_j.index());
            }
        }
    }

    if duplicates.is_empty() {
        return Ok(FixResult::ok());
    }

    // The first non-NURBS face is always the outer-loop anchor (`i`) and is
    // never inserted as a duplicate (`j > i`), so at least one face always
    // survives — the shell can't be emptied.
    let mut removed = 0usize;
    for &fid in &face_ids {
        if duplicates.contains(&fid.index()) {
            ctx.reshape.remove_face(fid);
            removed += 1;
        }
    }

    ctx.info(format!("removed {removed} duplicate face(s)"));

    Ok(FixResult {
        status: Status::DONE2,
        actions_taken: removed,
    })
}

fn boundaries_coincide_with_same_winding(a: &[Point3], b: &[Point3], tolerance: f64) -> bool {
    if a.is_empty() || a.len() != b.len() {
        return false;
    }

    (0..b.len()).any(|offset| {
        (a[0] - b[offset]).length() < tolerance
            && (0..a.len())
                .all(|index| (a[index] - b[(offset + index) % b.len()]).length() < tolerance)
    })
}

/// Merge coincident vertices across a solid.
///
/// For each pair of vertices within `ctx.tolerance.linear` distance,
/// the higher-index vertex is replaced by the lower-index one. Edge
/// start/end references are updated via the `ReShape` replacement tracker.
///
/// Ported from `remus-operations` `heal::merge_coincident_vertices`.
#[allow(clippy::too_many_lines)]
fn merge_coincident_vertices(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let tol = ctx.tolerance.linear;
    let tol_sq = tol * tol;

    let solid_data = topo.solid(solid_id)?;
    let shell_id = solid_data.outer_shell();
    let shell = topo.shell(shell_id)?;
    let face_ids: Vec<_> = shell.faces().to_vec();

    let mut vertex_ids: Vec<VertexId> = Vec::new();
    let mut positions: Vec<Point3> = Vec::new();
    let mut seen = std::collections::HashSet::new();

    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge())?;
                for &vid in &[edge.start(), edge.end()] {
                    if seen.insert(vid.index()) {
                        let point = topo.vertex(vid)?.point();
                        vertex_ids.push(vid);
                        positions.push(point);
                    }
                }
            }
        }
    }

    // Build merge map: higher-index merges into lower-index (canonical).
    let num_verts = vertex_ids.len();
    let mut merge_to: HashMap<usize, VertexId> = HashMap::new();
    let mut merged_count = 0usize;

    for i in 0..num_verts {
        if merge_to.contains_key(&vertex_ids[i].index()) {
            continue;
        }
        for j in (i + 1)..num_verts {
            if merge_to.contains_key(&vertex_ids[j].index()) {
                continue;
            }
            let dist_sq = (positions[i] - positions[j]).length_squared();
            if dist_sq < tol_sq {
                merge_to.insert(vertex_ids[j].index(), vertex_ids[i]);
                merged_count += 1;
            }
        }
    }

    if merged_count == 0 {
        return Ok(FixResult::ok());
    }

    for (&from_idx, &to_vid) in &merge_to {
        if let Some(&from_vid) = vertex_ids.iter().find(|v| v.index() == from_idx) {
            ctx.reshape.replace_vertex(from_vid, to_vid);
        }
    }

    // Also apply directly to edges (snapshot then allocate pattern).
    let mut edge_ids = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                edge_ids.push(oe.edge());
            }
        }
    }
    edge_ids.sort_by_key(|e| e.index());
    edge_ids.dedup_by_key(|e| e.index());

    let updates: Vec<_> = edge_ids
        .iter()
        .filter_map(|&eid| {
            let edge = topo.edge(eid).ok()?;
            let new_start = merge_to
                .get(&edge.start().index())
                .copied()
                .unwrap_or_else(|| edge.start());
            let new_end = merge_to
                .get(&edge.end().index())
                .copied()
                .unwrap_or_else(|| edge.end());
            if new_start != edge.start() || new_end != edge.end() {
                Some((eid, new_start, new_end))
            } else {
                None
            }
        })
        .collect();

    for (eid, new_start, new_end) in updates {
        let edge = topo.edge_mut(eid)?;
        // `set_start`/`set_end`, never a whole-`Edge` rebuild: an explicit
        // trim (RFC 0002, Stage 3) and an edge-specific tolerance are not
        // recoverable from the endpoints, and a vertex merge changes neither
        // the curve nor the parameter interval on it.
        edge.set_start(new_start);
        edge.set_end(new_end);
    }

    ctx.info(format!("merged {merged_count} coincident vertices"));

    Ok(FixResult {
        status: Status::DONE2,
        actions_taken: merged_count,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use remus_math::vec::Vec3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    /// Add a planar (+Z) triangle face with the given corner points.
    fn add_triangle(topo: &mut Topology, a: Point3, b: Point3, c: Point3) -> FaceId {
        let va = topo.add_vertex(Vertex::new(a, 1e-7));
        let vb = topo.add_vertex(Vertex::new(b, 1e-7));
        let vc = topo.add_vertex(Vertex::new(c, 1e-7));
        let eab = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
        let ebc = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let eca = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(eab, true),
                OrientedEdge::new(ebc, true),
                OrientedEdge::new(eca, true),
            ],
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn flags_and_removes_a_coincident_duplicate_face() {
        let mut topo = Topology::new();
        // Three distinct faces plus an exact geometric duplicate of the first.
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
        );
        let c = add_triangle(
            &mut topo,
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(1.0, 5.0, 0.0),
            Point3::new(0.0, 6.0, 0.0),
        );
        let dup = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b, c, dup]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 1, "exactly one duplicate expected");
        assert!(
            ctx.reshape.is_face_removed(dup),
            "the later face is removed"
        );
        assert!(!ctx.reshape.is_face_removed(a), "the original is kept");
        assert!(!ctx.reshape.is_face_removed(b));

        // Applying the reshape drops the duplicate from the shell.
        ctx.reshape.apply(&mut topo, solid_id).unwrap();
        let faces = topo
            .shell(topo.solid(solid_id).unwrap().outer_shell())
            .unwrap();
        assert_eq!(faces.faces().len(), 3, "shell drops the duplicate");
    }

    #[test]
    fn keeps_all_distinct_faces() {
        let mut topo = Topology::new();
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();
        assert_eq!(
            result.actions_taken, 0,
            "no duplicates among distinct faces"
        );
    }

    #[test]
    fn keeps_same_centroid_faces_with_different_boundaries() {
        let mut topo = Topology::new();
        let small = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        );
        let large = add_triangle(
            &mut topo,
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(5.0, -1.0, 0.0),
            Point3::new(-1.0, 5.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![small, large]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(small));
        assert!(!ctx.reshape.is_face_removed(large));
    }

    #[test]
    fn keeps_coincident_face_with_opposite_winding() {
        let mut topo = Topology::new();
        let original = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let opposite = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(original));
        assert!(!ctx.reshape.is_face_removed(opposite));
    }

    #[test]
    fn keeps_coincident_face_with_opposite_effective_normal() {
        let mut topo = Topology::new();
        let original = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let opposite = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        topo.face_mut(opposite).unwrap().set_reversed(true);
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(original));
        assert!(!ctx.reshape.is_face_removed(opposite));
    }
}
