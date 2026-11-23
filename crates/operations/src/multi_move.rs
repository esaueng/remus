//! Multi-face moves: several planar faces of one solid moved at once, each
//! along its own outward normal (B287).
//!
//! A selection may hold faces on different parallel planes (the tops beside
//! a slot and the slot floor), faces with different normals sharing edges
//! (the walls of a pocket or a boss) and faces with opposite normals (the
//! two sides of a rib). Every selected support is offset by its own signed
//! distance and every source edge and vertex is rebuilt once from its final
//! supports ([`remus_offset::move_faces_multi_with_entity_map`]), so the
//! result has the source's adjacency and an exact, total face, edge and
//! vertex map. Each face's swept region is refused when it reaches a
//! nonadjacent face that stays fixed, as for one coplanar group; a distance
//! at which selected faces reach or pass each other is refused by the
//! engine naming the face that folds. The result passes strict validation.
//!
//! A selection that is one coplanar group moved by one distance (or a
//! single face) is the ordinary face move: it runs
//! `push_pull::move_faces_with_entity_evolution` unchanged, blend
//! handling included, so its results are byte-identical to `move_faces`.
//!
//! Distances: either one signed distance for every face, or one per face in
//! selection order. Positive moves a face outward (adds material).

use remus_math::det_hash::DetHashSet;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::journal_ops::JournaledSolidOp;
use crate::push_pull::{
    DirectEditEvolution, MoveFacesResult, PreparedPlanarMove, SweptRegionCheck,
    boundary_entity_pairs, exact_face_evolution, validate_moved_solid,
};

/// Moves `faces` of `solid` together, each along its own outward normal.
///
/// See the [module docs](self) for the selection, the distances and the
/// postconditions.
///
/// # Errors
///
/// Returns a typed [`remus_offset::OffsetError`] through
/// [`OperationsError::Offset`]: `InvalidInput` for an empty selection, more
/// than [`remus_offset::MAX_MULTI_MOVE_FACES`] faces, a distance list of the
/// wrong length or a zero or non-finite distance; `TopologyChange` naming
/// the face (and edge, where known) when the move collapses, inverts or
/// splits a face or sweeps it into a fixed face. Any failure restores the
/// pre-call topology.
pub fn move_faces_multi(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<SolidId, OperationsError> {
    Ok(move_faces_multi_with_evolution(topo, solid, faces, distances)?.solid)
}

/// [`move_faces_multi`] with its exact, total face evolution.
///
/// # Errors
///
/// Returns the same typed refusals as [`move_faces_multi`].
pub fn move_faces_multi_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<MoveFacesResult, OperationsError> {
    Ok(move_faces_multi_with_entity_evolution(topo, solid, faces, distances)?.0)
}

/// [`move_faces_multi`] journaled as one `move_faces` entry with its exact
/// face, edge and vertex maps. Topology and history form one transaction.
///
/// # Errors
///
/// Returns the refusals of [`move_faces_multi`] or a recording error.
pub fn move_faces_multi_journaled(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<JournaledSolidOp, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let pending = crate::journal_ops::begin_scoped(topo, "move_faces", &[solid])?;
        let edit = move_faces_multi_with_entity_evolution(topo, solid, faces, distances)?;
        crate::journal_ops::record_face_move(topo, pending, edit)
    })
}

pub(crate) fn move_faces_multi_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
) -> Result<DirectEditEvolution, OperationsError> {
    check_counts(faces, distances)?;
    if let Some(distance) = remus_offset::coplanar_group_distance(topo, faces, distances)? {
        return crate::push_pull::move_faces_with_entity_evolution(topo, solid, faces, distance);
    }
    let checks = MultiChecks::prepare(topo, solid, faces)?;
    multi_edit(topo, solid, faces, distances, checks.as_ref())
}

/// The selection-size refusals, before anything reads the faces.
fn check_counts(faces: &[FaceId], distances: &[f64]) -> Result<(), OperationsError> {
    let reason = if faces.is_empty() {
        "move-face requires at least one selected face".to_owned()
    } else if faces.len() > remus_offset::MAX_MULTI_MOVE_FACES {
        format!(
            "multi-face move accepts at most {} faces, got {}",
            remus_offset::MAX_MULTI_MOVE_FACES,
            faces.len()
        )
    } else if distances.len() != 1 && distances.len() != faces.len() {
        format!(
            "multi-face move needs one distance or one per face ({} faces), got {}",
            faces.len(),
            distances.len()
        )
    } else {
        return Ok(());
    };
    Err(remus_offset::OffsetError::InvalidInput { reason }.into())
}

/// What a multi-face move reads from the source alone: its face list and,
/// for each selected face, its swept-region boxes against every fixed face.
#[derive(Debug, Clone)]
pub(crate) struct MultiChecks {
    source_faces: Vec<FaceId>,
    /// One per selected face, in selection order.
    swept: Vec<SweptRegionCheck>,
}

impl MultiChecks {
    /// `Ok(None)` for a selection the engine refuses on its own (a face
    /// outside `solid`, a repeated or non-planar face): the engine then
    /// names the refusal.
    pub(crate) fn prepare(
        topo: &Topology,
        solid: SolidId,
        faces: &[FaceId],
    ) -> Result<Option<Self>, OperationsError> {
        let source_faces = solid_faces(topo, solid)?;
        let selected: DetHashSet<FaceId> = faces.iter().copied().collect();
        if selected.len() != faces.len() || !faces.iter().all(|face| source_faces.contains(face)) {
            return Ok(None);
        }
        let mut swept = Vec::with_capacity(faces.len());
        for &face in faces {
            let Some(normal) = topo.face(face)?.effective_plane_normal() else {
                return Ok(None);
            };
            // Other selected faces move too: the engine judges where they
            // meet, so they are no obstacles here.
            let mut others = selected.clone();
            others.remove(&face);
            swept.push(SweptRegionCheck::prepare(
                topo,
                solid,
                // `SweptRegionCheck` takes std sets (`push_pull.rs` is on the
                // det-hash grandfather list) and only looks faces up in them.
                &std::iter::once(face).collect(),
                &others.iter().copied().collect(),
                normal,
            )?);
        }
        Ok(Some(Self {
            source_faces,
            swept,
        }))
    }
}

/// The multi-face solve, its swept-region refusals and postconditions.
fn multi_edit(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distances: &[f64],
    checks: Option<&MultiChecks>,
) -> Result<DirectEditEvolution, OperationsError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let outcome = (|| -> Result<DirectEditEvolution, OperationsError> {
        if let Some(checks) = checks {
            for (index, check) in checks.swept.iter().enumerate() {
                let distance = distances.get(index).copied().unwrap_or(distances[0]);
                if distance.is_finite() {
                    check.check_offset(topo, distance)?;
                }
            }
        }
        let moved = crate::performance::timed("move_faces.multi_engine", || {
            remus_offset::move_faces_multi_with_entity_map(topo, solid, faces, distances)
        })?;
        let boundary_pairs = boundary_entity_pairs(&moved);
        let source_faces = match checks {
            Some(checks) => checks.source_faces.clone(),
            None => solid_faces(topo, solid)?,
        };
        let result = MoveFacesResult {
            solid: moved.solid,
            evolution: exact_face_evolution(topo, &source_faces, moved.solid, moved.face_map)?,
        };
        validate_moved_solid(topo, faces, result.solid)?;
        Ok((result, boundary_pairs))
    })();
    if outcome.is_err() {
        snapshot.restore(topo);
    }
    outcome
}

/// A multi-face selection prepared for repeated moves by a scalar drag
/// distance: face `i` moves `weights[i] × distance`.
pub(crate) struct PreparedMultiMove {
    weights: Vec<f64>,
    /// The coplanar move's preparation, when every weight is the same and
    /// the selection is one coplanar group (so every call is that move).
    group: Option<PreparedPlanarMove>,
    checks: Option<MultiChecks>,
}

impl PreparedMultiMove {
    /// Prepares `faces` with per-face `weights` (one per face, or empty for
    /// all ones).
    pub(crate) fn prepare(
        topo: &Topology,
        solid: SolidId,
        faces: &[FaceId],
        weights: &[f64],
    ) -> Result<Self, OperationsError> {
        let weights = if weights.is_empty() {
            vec![1.0; faces.len()]
        } else {
            weights.to_vec()
        };
        if weights.len() != faces.len() || weights.iter().any(|weight| !weight.is_finite()) {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "multi-face move weights must be finite, one per face ({} faces), got {}",
                    faces.len(),
                    weights.len()
                ),
            });
        }
        check_counts(faces, &weights)?;
        let uniform = weights
            .iter()
            .all(|weight| weight.to_bits() == weights[0].to_bits());
        let group =
            if uniform && remus_offset::coplanar_group_distance(topo, faces, &[1.0])?.is_some() {
                PreparedPlanarMove::prepare(topo, solid, faces)?
            } else {
                None
            };
        Ok(Self {
            weights,
            group,
            checks: MultiChecks::prepare(topo, solid, faces)?,
        })
    }

    /// The per-face distances of a drag by `distance`.
    pub(crate) fn distances(&self, distance: f64) -> Vec<f64> {
        self.weights
            .iter()
            .map(|weight| weight * distance)
            .collect()
    }

    /// [`move_faces_multi_with_entity_evolution`] of the prepared selection
    /// by `distance` (scaled by the weights).
    pub(crate) fn move_faces(
        &mut self,
        topo: &mut Topology,
        solid: SolidId,
        faces: &[FaceId],
        distance: f64,
    ) -> Result<DirectEditEvolution, OperationsError> {
        let distances = self.distances(distance);
        if let Some(group_distance) =
            remus_offset::coplanar_group_distance(topo, faces, &distances)?
        {
            return match &mut self.group {
                Some(group) => group.move_faces(topo, solid, faces, group_distance),
                None => crate::push_pull::move_faces_with_entity_evolution(
                    topo,
                    solid,
                    faces,
                    group_distance,
                ),
            };
        }
        multi_edit(topo, solid, faces, &distances, self.checks.as_ref())
    }
}
