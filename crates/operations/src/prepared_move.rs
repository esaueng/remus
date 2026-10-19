//! Prepared face moves: repeated moves of one source to different distances.
//!
//! An interactive face drag restores the same source every frame and moves
//! the same selection to a new distance. [`PreparedFaceMove`] reads what
//! such a move needs from the source alone once — the source's face list and
//! journal scope, the selection's blend classification, the blend-aware
//! translation's region, carriers and boundary classification, the
//! swept-region boxes, and the prismatic volume oracle's source readings —
//! and each call runs only the distance-dependent rest of the very same code
//! as [`crate::push_pull::move_faces_with_evolution`] and
//! [`crate::journal_ops::move_faces_journaled`]. A blend move's
//! remove/rebuild attempt that refuses before it reaches the distance is
//! learned on the first call and skipped after it, as it refuses every
//! distance alike.
//!
//! # Equivalence
//!
//! A prepared call returns what the unprepared move returns for the same
//! source content, selection and distance: the same result content and
//! history, the same refusal text, with the same strict postconditions
//! (volume oracles, strict validation, total face maps). Result handles may
//! differ, because the unprepared move allocates and rolls back entities the
//! prepared one never creates; the content does not.
//!
//! # Staleness
//!
//! Preparation records the source's complete content key (every entity,
//! handle and real of the solid a move reads; the volume memo's key). Each
//! call recomputes it and refuses with [`OperationsError::InvalidInput`]
//! when it differs, so a source edited after preparation is never moved
//! from stale analysis. A checkpoint restore that brings the same content
//! back under the same handles — the drag loop's restore — is the same
//! source, even though it advances the topology's cache generation.
//! Selections the preparation does not cover (a bore move, an empty or
//! invalid selection) run the unprepared move on every call.

use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::journal::EntityKey;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::journal_ops::JournaledSolidOp;
use crate::measure::volume_memo::{ContentKey, solid_content_key};
use crate::push_pull::{DirectEditEvolution, MoveFacesResult, PreparedPlanarMove};

/// A face selection of one source solid, prepared for repeated moves.
///
/// See the [module docs](self) for what is prepared, the equivalence with
/// the unprepared move and the staleness contract.
pub struct PreparedFaceMove {
    solid: SolidId,
    faces: Vec<FaceId>,
    source: ContentKey,
    /// The journal's pre-operation scope: every entity of the source.
    scope: Vec<EntityKey>,
    /// `None` runs the unprepared move on every call.
    planar: Option<PreparedPlanarMove>,
}

impl std::fmt::Debug for PreparedFaceMove {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PreparedFaceMove")
            .field("solid", &self.solid)
            .field("faces", &self.faces)
            .field("prepared", &self.planar.is_some())
            .finish_non_exhaustive()
    }
}

impl PreparedFaceMove {
    /// Prepares moves of `faces` of `solid`. Reads the topology only.
    ///
    /// # Errors
    ///
    /// Returns an error when `solid` or one of its entities is missing, or a
    /// distance-independent reading of the source fails.
    pub fn prepare(
        topo: &Topology,
        solid: SolidId,
        faces: &[FaceId],
    ) -> Result<Self, OperationsError> {
        let _span = crate::performance::span("move_faces.prepare");
        topo.solid(solid)?;
        let source =
            solid_content_key(topo, solid).ok_or_else(|| OperationsError::InvalidInput {
                reason: format!("solid {} has missing entities", solid.index()),
            })?;
        Ok(Self {
            solid,
            faces: faces.to_vec(),
            source,
            scope: crate::journal_ops::solid_entity_keys(topo, solid)?,
            planar: PreparedPlanarMove::prepare(topo, solid, faces)?,
        })
    }

    /// The prepared source solid.
    #[must_use]
    pub const fn solid(&self) -> SolidId {
        self.solid
    }

    /// The prepared face selection.
    #[must_use]
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    /// Whether `topo` still holds the prepared source, entity for entity.
    #[must_use]
    pub fn is_current(&self, topo: &Topology) -> bool {
        let _span = crate::performance::span("move_faces.prepared_source_check");
        solid_content_key(topo, self.solid).is_some_and(|key| key == self.source)
    }

    /// [`crate::push_pull::move_faces_with_evolution`] of the prepared
    /// selection by `distance`.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError::InvalidInput`] when the source changed
    /// since preparation, otherwise exactly the refusals of the unprepared
    /// move. Any failure restores the topology to its pre-call state.
    pub fn move_faces(
        &mut self,
        topo: &mut Topology,
        distance: f64,
    ) -> Result<MoveFacesResult, OperationsError> {
        self.require_current(topo)?;
        Ok(self.edit(topo, distance)?.0)
    }

    /// [`crate::journal_ops::move_faces_journaled`] of the prepared
    /// selection by `distance`: the move and its evolution entry (kind
    /// `move_faces`) in one transaction.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError::InvalidInput`] when the source changed
    /// since preparation, otherwise exactly the refusals of the unprepared
    /// journaled move. Any failure restores topology and history.
    pub fn move_faces_journaled(
        &mut self,
        topo: &mut Topology,
        distance: f64,
    ) -> Result<JournaledSolidOp, OperationsError> {
        let _total = crate::performance::span("move_faces.journaled");
        self.require_current(topo)?;
        if self.planar.is_none() {
            return crate::journal_ops::move_faces_journaled(
                topo,
                self.solid,
                &self.faces,
                distance,
            );
        }
        let scope = std::mem::take(&mut self.scope);
        let result = crate::journal_ops::move_faces_journaled_with_scope(topo, &scope, |topo| {
            self.edit(topo, distance)
        });
        self.scope = scope;
        result
    }

    fn require_current(&self, topo: &Topology) -> Result<(), OperationsError> {
        if self.is_current(topo) {
            Ok(())
        } else {
            Err(OperationsError::InvalidInput {
                reason: format!(
                    "prepared face move is stale: solid {} changed since it was prepared",
                    self.solid.index()
                ),
            })
        }
    }

    fn edit(
        &mut self,
        topo: &mut Topology,
        distance: f64,
    ) -> Result<DirectEditEvolution, OperationsError> {
        match &mut self.planar {
            Some(planar) => planar.move_faces(topo, self.solid, &self.faces, distance),
            None => crate::push_pull::move_faces_with_entity_evolution(
                topo,
                self.solid,
                &self.faces,
                distance,
            ),
        }
    }
}

#[cfg(test)]
mod tests;
