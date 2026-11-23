//! Simultaneous moves of several planar faces, each along its own outward
//! normal (B287).
//!
//! [`super::move_faces`] moves one coplanar group along its shared normal.
//! [`move_faces_multi_with_entity_map`] lifts that guard: every selected
//! planar support is offset by its own signed distance first, every other
//! support stays fixed, and each source edge and vertex is then rebuilt once
//! from its final supports — the same construction, with the same
//! topology-preserving postconditions, as the coplanar move. Edges shared by
//! two selected faces (the corners of a pocket) are re-intersected from both
//! moved planes, so nothing is re-limited twice.
//!
//! Selections may hold faces on different parallel planes (stepped tops),
//! faces with different normals (the walls of a pocket or a boss) and faces
//! with opposite normals (the two sides of a rib): each moves along its own
//! outward normal. A distance at which two faces' edges reach or pass each
//! other is refused with [`OffsetError::TopologyChange`] naming a selected
//! face on the edge that collapses or inverts.
//!
//! A selection that is one coplanar group moved by one distance is the
//! coplanar move itself, so its results stay bit-identical to
//! [`super::move_faces_with_entity_map`].

use std::collections::{BTreeMap, HashMap, HashSet};

use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::{
    BoundaryMaps, MoveFacesEntityResult, MoveFacesResult, MoveShifts, effective_plane,
    move_faces_impl, rebuild_planar_moves,
};
use crate::data::{OffsetData, OffsetOptions};
use crate::error::OffsetError;

/// The most faces one multi-face move accepts.
pub const MAX_MULTI_MOVE_FACES: usize = 64;

/// Move several planar faces of `solid` at once, each along its own outward
/// normal.
///
/// `distances` holds either one signed distance applied to every face, or
/// one per face in `faces` order. A positive distance moves a face outward
/// (adds material), a negative one inward. Up to
/// [`MAX_MULTI_MOVE_FACES`] faces are accepted.
///
/// # Errors
///
/// Returns [`OffsetError::InvalidInput`] for an empty or oversized
/// selection, a repeated face, a distance list of the wrong length, or a
/// zero or non-finite distance; [`OffsetError::FaceNotInSolid`];
/// [`OffsetError::UnsupportedMoveFace`] for a non-planar selected face or an
/// unsupported neighbour; and [`OffsetError::TopologyChange`] naming the
/// face (and edge, where known) when the move collapses, inverts, splits or
/// otherwise changes the source topology. Any failure restores the
/// pre-call topology.
pub fn move_faces_multi(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<SolidId, OffsetError> {
    Ok(move_faces_multi_with_face_map(topo, solid, faces, distances)?.solid)
}

/// [`move_faces_multi`] with the exact source-to-result face map.
///
/// # Errors
///
/// Returns the same typed refusals as [`move_faces_multi`].
pub fn move_faces_multi_with_face_map(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<MoveFacesResult, OffsetError> {
    move_faces_multi_with_entity_map(topo, solid, faces, distances).map(Into::into)
}

/// [`move_faces_multi`] with construction-derived face, edge, and vertex maps.
///
/// # Errors
///
/// Returns the same typed refusals as [`move_faces_multi`].
pub fn move_faces_multi_with_entity_map(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<MoveFacesEntityResult, OffsetError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = move_faces_multi_impl(topo, solid, faces, distances);
    if result.is_err() {
        snapshot.restore(topo);
    }
    result
}

/// The one distance of a selection that the coplanar move handles itself.
///
/// That is a single face, or faces on one plane with one outward normal,
/// all moved by the same distance. `None` for a selection that needs the
/// multi-face solve, and for one the multi-face move refuses.
///
/// # Errors
///
/// Returns an error when a selected face is missing from `topo`.
pub fn coplanar_group_distance(
    topo: &Topology,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<Option<f64>, OffsetError> {
    let Some((&reference, &first_distance)) = faces.first().zip(distances.first()) else {
        return Ok(None);
    };
    if distances.len() != 1 && distances.len() != faces.len() {
        return Ok(None);
    }
    if distances
        .iter()
        .any(|distance| distance.to_bits() != first_distance.to_bits())
    {
        return Ok(None);
    }
    if faces.len() == 1 {
        return Ok(Some(first_distance));
    }
    let tolerance = OffsetOptions::default().tolerance;
    let face_data = topo.face(reference)?;
    if !matches!(face_data.surface(), FaceSurface::Plane { .. }) {
        return Ok(None);
    }
    let (reference_normal, reference_d) = effective_plane(topo, reference)?;
    for &face in &faces[1..] {
        if !matches!(topo.face(face)?.surface(), FaceSurface::Plane { .. }) {
            return Ok(None);
        }
        let (normal, d) = effective_plane(topo, face)?;
        if normal.dot(reference_normal) < 1.0 - tolerance.angular
            || !tolerance.approx_eq(d, reference_d)
        {
            return Ok(None);
        }
    }
    Ok(Some(first_distance))
}

fn move_faces_multi_impl(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<MoveFacesEntityResult, OffsetError> {
    let options = OffsetOptions::default();
    if faces.is_empty() {
        return Err(OffsetError::InvalidInput {
            reason: "move-face requires at least one selected face".into(),
        });
    }
    if faces.len() > MAX_MULTI_MOVE_FACES {
        return Err(OffsetError::InvalidInput {
            reason: format!(
                "multi-face move accepts at most {MAX_MULTI_MOVE_FACES} faces, got {}",
                faces.len()
            ),
        });
    }
    if distances.len() != 1 && distances.len() != faces.len() {
        return Err(OffsetError::InvalidInput {
            reason: format!(
                "multi-face move needs one distance or one per face ({} faces), got {}",
                faces.len(),
                distances.len()
            ),
        });
    }
    let distance_of = |index: usize| {
        if distances.len() == 1 {
            distances[0]
        } else {
            distances[index]
        }
    };
    for (index, &face) in faces.iter().enumerate() {
        let distance = distance_of(index);
        if !distance.is_finite() || distance.abs() <= options.tolerance.linear {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "move-face distance for face {} must be non-zero and finite",
                    face.index()
                ),
            });
        }
    }

    // Selection checks run before the coplanar shortcut so that the
    // shortcut never changes which refusal a bad selection gets.
    let source_faces = solid_faces(topo, solid)?;
    let source_face_set: HashSet<_> = source_faces.iter().map(|face| face.index()).collect();
    let mut selected = HashSet::with_capacity(faces.len());
    for &face in faces {
        if !source_face_set.contains(&face.index()) {
            return Err(OffsetError::FaceNotInSolid { face, solid });
        }
        if !selected.insert(face.index()) {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "move-face selection contains face {} more than once",
                    face.index()
                ),
            });
        }
    }
    if let Some(distance) = coplanar_group_distance(topo, faces, distances)? {
        return move_faces_impl(topo, solid, faces, distance);
    }

    let mut signed = HashMap::with_capacity(faces.len());
    let mut shifts = HashMap::with_capacity(faces.len());
    let mut leading = 0.0_f64;
    for (index, &face) in faces.iter().enumerate() {
        let distance = distance_of(index);
        let (normal, _) = effective_plane(topo, face)?;
        signed.insert(face.index(), distance);
        shifts.insert(face.index(), normal * distance);
        if distance.abs() > leading.abs() {
            leading = distance;
        }
    }
    rebuild_planar_moves(
        topo,
        solid,
        &source_faces,
        &selected,
        &signed,
        &MoveShifts::per_face(shifts),
        leading,
    )
}

/// The first estimate of a moved vertex: the point nearest the source that
/// lies on every planar support of the vertex, moved and fixed (moved ones
/// take precedence where the supports are dependent). The rebuild then
/// snaps it onto the re-intersected edges; this only ranks candidates.
pub(super) fn predict_vertex<V: std::ops::Deref<Target = [FaceId]>>(
    source: Point3,
    incident_edges: &[EdgeId],
    source_edge_faces: &BTreeMap<usize, V>,
    data: &OffsetData,
) -> Point3 {
    let mut faces: Vec<FaceId> = incident_edges
        .iter()
        .filter_map(|edge| source_edge_faces.get(&edge.index()))
        .flat_map(|faces| faces.iter().copied())
        .collect();
    faces.compact_sort_unstable_by_key(|face| face.index());
    faces.dedup();
    let mut rows = Vec::with_capacity(faces.len());
    for moved_pass in [true, false] {
        for face in &faces {
            let Some(offset_face) = data.offset_faces.get(face) else {
                continue;
            };
            if (offset_face.distance != 0.0) != moved_pass {
                continue;
            }
            if let FaceSurface::Plane { normal, d } = offset_face.surface {
                rows.push((normal, d - normal.dot(source - Point3::new(0.0, 0.0, 0.0))));
            }
        }
    }
    source + least_norm_shift(&rows)
}

/// The shortest `δ` with `n·δ = r` for a maximal independent prefix-greedy
/// subset of `rows` (at most three).
fn least_norm_shift(rows: &[(Vec3, f64)]) -> Vec3 {
    let mut basis: Vec<Vec3> = Vec::with_capacity(3);
    let mut chosen: Vec<(Vec3, f64)> = Vec::with_capacity(3);
    for &(normal, residual) in rows {
        if chosen.len() == 3 {
            break;
        }
        let mut orthogonal = normal;
        for &unit in &basis {
            orthogonal = orthogonal - unit * orthogonal.dot(unit);
        }
        let length = orthogonal.length();
        if length <= 1e-6 * normal.length().max(f64::MIN_POSITIVE) {
            continue;
        }
        basis.push(orthogonal * (1.0 / length));
        chosen.push((normal, residual));
    }
    // δ = Σ c_j n_j with Gram matrix G c = r.
    let k = chosen.len();
    let mut matrix = [[0.0_f64; 4]; 3];
    for (i, &(first, residual)) in chosen.iter().enumerate() {
        for (j, &(second, _)) in chosen.iter().enumerate() {
            matrix[i][j] = first.dot(second);
        }
        matrix[i][3] = residual;
    }
    for column in 0..k {
        let Some(pivot) = (column..k)
            .max_by(|&a, &b| matrix[a][column].abs().total_cmp(&matrix[b][column].abs()))
        else {
            break;
        };
        matrix.swap(column, pivot);
        let lead = matrix[column][column];
        if lead.abs() <= f64::EPSILON {
            return Vec3::new(0.0, 0.0, 0.0);
        }
        for row in 0..k {
            if row == column {
                continue;
            }
            let factor = matrix[row][column] / lead;
            for entry in column..4 {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
        }
    }
    let mut shift = Vec3::new(0.0, 0.0, 0.0);
    for (i, &(normal, _)) in chosen.iter().enumerate() {
        shift += normal * (matrix[i][3] / matrix[i][i]);
    }
    shift
}

/// The face a refusal on an edge names: its first face, or, for a per-face
/// move, a selected (moved) face on it when it has one.
pub(super) fn named_face(
    faces: &[FaceId],
    data: &OffsetData,
    prefer_moved: bool,
) -> Option<FaceId> {
    if prefer_moved
        && let Some(&face) = faces.iter().find(|face| {
            data.offset_faces
                .get(face)
                .is_some_and(|surface| surface.distance != 0.0)
        })
    {
        return Some(face);
    }
    faces.first().copied()
}

/// Refuses a per-face move that turns a straight source edge around: its
/// ends passed each other, so the faces it bounds folded over (a pocket
/// wall moved past the opposite wall).
pub(super) fn validate_edge_directions<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    source_edge_faces: &BTreeMap<usize, V>,
    selected: &HashSet<usize>,
    maps: &BoundaryMaps,
) -> Result<(), OffsetError> {
    for (&index, faces) in source_edge_faces {
        let Some(source_edge) = topo.edge_id_from_index(index) else {
            continue;
        };
        let source = topo.edge(source_edge)?;
        if !matches!(source.curve(), EdgeCurve::Line) || source.start() == source.end() {
            continue;
        }
        let Some(&rebuilt) = maps.edges.get(&index) else {
            continue;
        };
        let before = topo.vertex(source.end())?.point() - topo.vertex(source.start())?.point();
        let rebuilt = topo.edge(rebuilt)?;
        let after = topo.vertex(rebuilt.end())?.point() - topo.vertex(rebuilt.start())?.point();
        if before.dot(after) <= 0.0 {
            let face = faces
                .iter()
                .find(|face| selected.contains(&face.index()))
                .or_else(|| faces.first())
                .copied();
            return Err(OffsetError::TopologyChange {
                face,
                edge: Some(source_edge),
                reason: format!(
                    "move inverts source edge {index}: its ends pass each other, so the faces \
                     it bounds fold over"
                ),
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        (a - b).length() <= 1e-12
    }

    #[test]
    fn least_norm_shift_solves_orthogonal_and_oblique_rows() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        assert!(close(
            least_norm_shift(&[(x, 2.0), (y, -1.0), (z, 0.0)]),
            Vec3::new(2.0, -1.0, 0.0)
        ));
        // One row: along its normal.
        assert!(close(
            least_norm_shift(&[(z, 3.0)]),
            Vec3::new(0.0, 0.0, 3.0)
        ));
        // A duplicate (dependent) row is skipped.
        assert!(close(
            least_norm_shift(&[(z, 3.0), (z, 5.0)]),
            Vec3::new(0.0, 0.0, 3.0)
        ));
        // Oblique: n1 = x, n2 = (x + y)/√2; δ must satisfy both.
        let oblique = Vec3::new(1.0, 1.0, 0.0) * std::f64::consts::FRAC_1_SQRT_2;
        let shift = least_norm_shift(&[(x, 1.0), (oblique, 2.0)]);
        assert!((shift.dot(x) - 1.0).abs() <= 1e-12);
        assert!((shift.dot(oblique) - 2.0).abs() <= 1e-12);
        assert!(shift.dot(z).abs() <= 1e-12);
    }
}
