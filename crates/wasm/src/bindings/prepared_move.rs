//! Prepared face moves for interactive drags.
//!
//! Tier: core — a prepared move is session state (a handle into the kernel)
//! that edits the live topology and writes the journal on every call.
//!
//! A drag restores the same source every frame and moves the same faces to
//! a new distance. `prepareMoveFaces` reads what does not depend on the
//! distance once ([`remus_operations::prepared_move`]); `moveFacesPrepared`
//! and `moveFacesPreparedJournaled` then return exactly what `moveFaces` and
//! `moveFacesJournaled` return for the same source, faces and distance, and
//! refuse, without editing anything, once the source has changed since
//! preparation. A checkpoint `restore` that brings the prepared source back
//! keeps the preparation valid.

use std::collections::BTreeMap;

use remus_operations::prepared_move::PreparedFaceMove;
use wasm_bindgen::prelude::*;

use crate::error::{StructuredWasmError, WasmError, validate_finite, validate_work_count};
use crate::handles::solid_id_to_u32;
use crate::kernel::RemusKernel;

/// Live prepared moves one kernel holds at most; a drag needs one.
pub const MAX_PREPARED_MOVES: usize = 16;

/// The kernel's prepared moves under their opaque handles. Handles are never
/// reused, so a released one cannot alias a later preparation.
#[derive(Debug, Default)]
pub struct PreparedMoves {
    live: BTreeMap<u32, PreparedFaceMove>,
    next: u32,
}

#[wasm_bindgen]
impl RemusKernel {
    /// Prepare repeated moves of `faces` of `solid` for an interactive drag.
    ///
    /// Reads, once, everything a face move takes from the source rather than
    /// from the distance. Returns an opaque handle for `moveFacesPrepared`
    /// and `moveFacesPreparedJournaled`; release it with
    /// `releasePreparedMove` when the drag ends. Preparing edits nothing.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid handle, a selection over the
    /// `moveFaces` work limits, or when 16 prepared moves are already live.
    #[wasm_bindgen(js_name = "prepareMoveFaces")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn prepare_move_faces(&mut self, solid: u32, faces: Vec<u32>) -> Result<u32, JsError> {
        Ok(self.prepare_move_faces_impl(solid, &faces)?)
    }

    /// `moveFaces` of a prepared selection: moves it `distance` from the
    /// prepared source and returns the new solid handle.
    ///
    /// The result and every refusal are those of `moveFaces` for the same
    /// source, faces and distance; distances are source-relative, as in
    /// `moveFaces`.
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown or released handle, a non-finite
    /// distance, a source that changed since preparation (prepare again), or
    /// any `moveFaces` refusal. A refusal leaves the topology unchanged.
    #[wasm_bindgen(js_name = "moveFacesPrepared")]
    pub fn move_faces_prepared(&mut self, prepared: u32, distance: f64) -> Result<u32, JsError> {
        Ok(self.move_faces_prepared_impl(prepared, distance)?)
    }

    /// `moveFacesJournaled` of a prepared selection.
    ///
    /// Returns the same JSON `{"solid", "op"}` and records the same
    /// `move_faces` evolution entry as `moveFacesJournaled` for the same
    /// source, faces and distance.
    ///
    /// # Errors
    ///
    /// As `moveFacesPrepared`; a refusal restores topology and history.
    #[wasm_bindgen(js_name = "moveFacesPreparedJournaled")]
    pub fn move_faces_prepared_journaled(
        &mut self,
        prepared: u32,
        distance: f64,
    ) -> Result<String, JsError> {
        validate_finite(distance, "distance")?;
        self.move_faces_prepared_journaled_json(prepared, distance)
            .map(|value| value.to_string())
            .map_err(|error| JsError::new(error.message()))
    }

    /// Release a prepared move. Returns whether `prepared` was live.
    #[wasm_bindgen(js_name = "releasePreparedMove")]
    pub fn release_prepared_move(&mut self, prepared: u32) -> bool {
        self.prepared_moves.live.remove(&prepared).is_some()
    }
}

/// Natively testable bodies of the bindings above.
impl RemusKernel {
    pub(crate) fn prepare_move_faces_impl(
        &mut self,
        solid: u32,
        faces: &[u32],
    ) -> Result<u32, WasmError> {
        validate_work_count(u32::try_from(faces.len()).unwrap_or(u32::MAX), "faces")?;
        let solid_id = self.resolve_solid(solid)?;
        let face_ids = faces
            .iter()
            .map(|&face| self.resolve_face(face))
            .collect::<Result<Vec<_>, _>>()?;
        super::operations::validate_move_faces_topology_work(self.topo(), solid_id, &face_ids)?;
        if self.prepared_moves.live.len() >= MAX_PREPARED_MOVES {
            return Err(WasmError::InvalidInput {
                reason: format!("{MAX_PREPARED_MOVES} prepared moves are live; release one first"),
            });
        }
        let handle = self.prepared_moves.next;
        let next = handle
            .checked_add(1)
            .ok_or_else(|| WasmError::InvalidInput {
                reason: "prepared move handles are exhausted".into(),
            })?;
        let prepared = PreparedFaceMove::prepare(self.topo(), solid_id, &face_ids)?;
        self.prepared_moves.live.insert(handle, prepared);
        self.prepared_moves.next = next;
        Ok(handle)
    }

    pub(crate) fn move_faces_prepared_impl(
        &mut self,
        prepared: u32,
        distance: f64,
    ) -> Result<u32, WasmError> {
        validate_finite(distance, "distance")?;
        let result =
            self.with_prepared_move(prepared, |topo, move_| move_.move_faces(topo, distance))??;
        Ok(solid_id_to_u32(result.solid))
    }

    pub(crate) fn move_faces_prepared_journaled_json(
        &mut self,
        prepared: u32,
        distance: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let result = self
            .with_prepared_move(prepared, |topo, move_| {
                move_.move_faces_journaled(topo, distance)
            })
            .map_err(StructuredWasmError::from)?
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    /// Runs `edit` on the live prepared move `prepared` and the topology.
    fn with_prepared_move<T>(
        &mut self,
        prepared: u32,
        edit: impl FnOnce(&mut remus_topology::Topology, &mut PreparedFaceMove) -> T,
    ) -> Result<T, WasmError> {
        // Taken out for the call so the topology can be borrowed mutably.
        let mut move_ =
            self.prepared_moves
                .live
                .remove(&prepared)
                .ok_or(WasmError::InvalidHandle {
                    entity: "prepared move",
                    index: prepared as usize,
                })?;
        let result = edit(self.topo_mut(), &mut move_);
        self.prepared_moves.live.insert(prepared, move_);
        Ok(result)
    }
}

#[cfg(test)]
mod tests;
