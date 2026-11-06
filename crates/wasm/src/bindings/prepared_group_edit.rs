//! Prepared blend-group edits for live band-radius and boss-diameter drags.
//!
//! Tier: core — a prepared group edit is session state (a handle into the
//! kernel) whose frames and commits edit the live topology, and whose
//! journaled commit writes the evolution journal.
//!
//! A drag on a blended body (a fillet band's radius, a boss wall's diameter)
//! restores the same source every frame and edits it to a new value.
//! `prepareResizeBlend` / `prepareResizeCylindricalFace` recognize the blend
//! group once, remove it to its sharp parent once and learn whether the
//! per-band or partial-turn attempt in front of the group route refuses
//! every value ([`remus_operations::blend_cluster::PreparedGroupEdit`]).
//! `previewPreparedGroupEdit` then re-blends that sharp parent for each value
//! with display-only blends under a structural gate (closed shell, strict
//! validation); `commitPreparedGroupEdit` and
//! `commitPreparedGroupEditJournaled` return exactly what `resizeBlend` /
//! `resizeCylindricalFace` and their journaled variants return for the same
//! source, face and value, under every guard. Every call refuses, editing
//! nothing, once the source has changed since preparation; a checkpoint
//! `restore` that brings the prepared source back keeps it valid.

use std::collections::BTreeMap;

use remus_operations::OperationsError;
use remus_operations::blend_cluster::PreparedGroupEdit;
use remus_operations::resize_blend::resize_blend_failure_code;
use wasm_bindgen::prelude::*;

use crate::error::{StructuredWasmError, WasmError, validate_finite, validate_positive};
use crate::handles::solid_id_to_u32;
use crate::kernel::RemusKernel;

/// Live prepared group edits one kernel holds at most; a drag needs one.
pub const MAX_PREPARED_GROUP_EDITS: usize = 16;

/// The kernel's prepared group edits under their opaque handles. Handles
/// are never reused, so a released one cannot alias a later preparation.
#[derive(Debug, Default)]
pub struct PreparedGroupEdits {
    live: BTreeMap<u32, PreparedGroupEdit>,
    next: u32,
}

#[wasm_bindgen]
impl RemusKernel {
    /// Prepare live radius edits of the blend band containing `face`
    /// (`resizeBlend` through its blend group) for an interactive drag.
    ///
    /// `expected_radius` is the band's current radius, as `resizeBlend`
    /// takes it. Returns an opaque handle for `previewPreparedGroupEdit`,
    /// `commitPreparedGroupEdit` and `commitPreparedGroupEditJournaled`;
    /// release it with `releasePreparedGroupEdit` when the drag ends.
    /// Preparing edits nothing.
    ///
    /// # Errors
    ///
    /// A code-prefixed refusal: `blend-radius-mismatch` for a wrong witness,
    /// `blend-cluster-not-prepared` when the band is not a member of a
    /// recognized blend group that removes to sharp (use `resizeBlend`), or
    /// an invalid handle; also when 16 prepared group edits are live.
    #[wasm_bindgen(js_name = "prepareResizeBlend")]
    pub fn prepare_resize_blend(
        &mut self,
        solid: u32,
        face: u32,
        expected_radius: f64,
    ) -> Result<u32, JsError> {
        validate_positive(expected_radius, "expected_radius")?;
        self.prepare_group_edit_impl(solid, face, Some(expected_radius))
            .map_err(|error| JsError::new(&error))
    }

    /// Prepare live radius edits of the cylindrical support `face` (a boss
    /// wall resized through the blend group it carries, as
    /// `resizeCylindricalFace` does) for an interactive drag.
    ///
    /// # Errors
    ///
    /// As `prepareResizeBlend`; `blend-cluster-not-prepared` when the wall
    /// carries no recognized blend group that removes to sharp (use
    /// `resizeCylindricalFace`).
    #[wasm_bindgen(js_name = "prepareResizeCylindricalFace")]
    pub fn prepare_resize_cylindrical_face(
        &mut self,
        solid: u32,
        face: u32,
    ) -> Result<u32, JsError> {
        self.prepare_group_edit_impl(solid, face, None)
            .map_err(|error| JsError::new(&error))
    }

    /// One preview frame of a prepared group edit at `value` (the band's new
    /// radius, or the wall's new radius): returns the frame's solid handle.
    ///
    /// The geometry is the group route's for `value`, built with
    /// display-only blends and accepted on a closed shell and strict
    /// validation. Values are source-relative: restore the drag's checkpoint
    /// before each frame, as for `moveFacesPrepared`. Keep a value with
    /// `commitPreparedGroupEdit` or `commitPreparedGroupEditJournaled`.
    ///
    /// # Errors
    ///
    /// A code-prefixed refusal (the codes `resizeBlend` uses), an unknown or
    /// released handle, a non-finite value, or a source that changed since
    /// preparation (`invalid input: prepared group edit is stale`; prepare
    /// again). A refusal leaves the topology unchanged.
    #[wasm_bindgen(js_name = "previewPreparedGroupEdit")]
    pub fn preview_prepared_group_edit(
        &mut self,
        prepared: u32,
        value: f64,
    ) -> Result<u32, JsError> {
        validate_finite(value, "value")?;
        self.preview_prepared_group_edit_impl(prepared, value)
            .map_err(|error| JsError::new(&error))
    }

    /// Keep a prepared group edit at `value`: returns what `resizeBlend` or
    /// `resizeCylindricalFace` returns for the prepared source, face and
    /// value, under every guard.
    ///
    /// # Errors
    ///
    /// As `previewPreparedGroupEdit`, with the unprepared call's refusals.
    #[wasm_bindgen(js_name = "commitPreparedGroupEdit")]
    pub fn commit_prepared_group_edit(
        &mut self,
        prepared: u32,
        value: f64,
    ) -> Result<u32, JsError> {
        validate_finite(value, "value")?;
        self.commit_prepared_group_edit_impl(prepared, value)
            .map_err(|error| JsError::new(&error))
    }

    /// Keep a prepared group edit at `value` with its evolution entry: the
    /// JSON `{"solid", "op"}` and the journal entry `resizeBlendJournaled` or
    /// `resizeCylindricalFaceJournaled` returns and records for the prepared
    /// source, face and value.
    ///
    /// # Errors
    ///
    /// The unprepared journaled call's structured refusals, an unknown
    /// handle or a stale source. A refusal restores topology and history.
    #[wasm_bindgen(js_name = "commitPreparedGroupEditJournaled")]
    pub fn commit_prepared_group_edit_journaled(
        &mut self,
        prepared: u32,
        value: f64,
    ) -> Result<String, JsError> {
        validate_finite(value, "value")?;
        self.commit_prepared_group_edit_journaled_json(prepared, value)
            .map(|value| value.to_string())
            .map_err(|error| JsError::new(error.message()))
    }

    /// Release a prepared group edit. Returns whether `prepared` was live.
    #[wasm_bindgen(js_name = "releasePreparedGroupEdit")]
    pub fn release_prepared_group_edit(&mut self, prepared: u32) -> bool {
        self.prepared_group_edits.live.remove(&prepared).is_some()
    }
}

/// `"{code}: {message}"`, as `resizeBlend` reports its refusals.
fn coded(error: &OperationsError) -> String {
    format!("{}: {error}", resize_blend_failure_code(error))
}

/// Natively testable bodies of the bindings above. Refusals are the
/// code-prefixed strings the bindings throw.
impl RemusKernel {
    /// `expected_radius` prepares a band; `None` a cylindrical support.
    pub(crate) fn prepare_group_edit_impl(
        &mut self,
        solid: u32,
        face: u32,
        expected_radius: Option<f64>,
    ) -> Result<u32, String> {
        let solid_id = self.resolve_solid(solid).map_err(|e| e.to_string())?;
        let face_id = self.resolve_face(face).map_err(|e| e.to_string())?;
        if self.prepared_group_edits.live.len() >= MAX_PREPARED_GROUP_EDITS {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "{MAX_PREPARED_GROUP_EDITS} prepared group edits are live; release one first"
                ),
            }
            .to_string());
        }
        let handle = self.prepared_group_edits.next;
        let next = handle.checked_add(1).ok_or_else(|| {
            WasmError::InvalidInput {
                reason: "prepared group edit handles are exhausted".into(),
            }
            .to_string()
        })?;
        let prepared = match expected_radius {
            Some(radius) => PreparedGroupEdit::prepare_band(self.topo(), solid_id, face_id, radius),
            None => PreparedGroupEdit::prepare_cylinder(self.topo(), solid_id, face_id),
        }
        .map_err(|error| coded(&error))?;
        self.prepared_group_edits.live.insert(handle, prepared);
        self.prepared_group_edits.next = next;
        Ok(handle)
    }

    pub(crate) fn preview_prepared_group_edit_impl(
        &mut self,
        prepared: u32,
        value: f64,
    ) -> Result<u32, String> {
        self.with_prepared_group_edit(prepared, |topo, edit| edit.preview(topo, value))?
            .map(solid_id_to_u32)
            .map_err(|error| coded(&error))
    }

    pub(crate) fn commit_prepared_group_edit_impl(
        &mut self,
        prepared: u32,
        value: f64,
    ) -> Result<u32, String> {
        self.with_prepared_group_edit(prepared, |topo, edit| edit.commit(topo, value))?
            .map(solid_id_to_u32)
            .map_err(|error| coded(&error))
    }

    pub(crate) fn commit_prepared_group_edit_journaled_json(
        &mut self,
        prepared: u32,
        value: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let result = self
            .with_prepared_group_edit(prepared, |topo, edit| edit.commit_journaled(topo, value))
            .map_err(|reason| StructuredWasmError::from(WasmError::InvalidInput { reason }))?
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    /// Runs `edit` on the live prepared group edit `prepared` and the
    /// topology.
    fn with_prepared_group_edit<T>(
        &mut self,
        prepared: u32,
        edit: impl FnOnce(&mut remus_topology::Topology, &mut PreparedGroupEdit) -> T,
    ) -> Result<T, String> {
        // Taken out for the call so the topology can be borrowed mutably.
        let mut group_edit = self
            .prepared_group_edits
            .live
            .remove(&prepared)
            .ok_or_else(|| {
                WasmError::InvalidHandle {
                    entity: "prepared group edit",
                    index: prepared as usize,
                }
                .to_string()
            })?;
        let result = edit(self.topo_mut(), &mut group_edit);
        self.prepared_group_edits.live.insert(prepared, group_edit);
        Ok(result)
    }
}

#[cfg(test)]
mod tests;
