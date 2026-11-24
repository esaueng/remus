//! Multi-face moves (B287): several planar faces of one solid moved at
//! once, each along its own outward normal.
//!
//! Tier: core — the moves edit live solid handles and write the journal.
//!
//! `moveFacesMulti`, `moveFacesMultiJournaled` and `moveFacesMultiDetailed`
//! are the multi-face twins of `moveFaces`, `moveFacesJournaled` and
//! `moveFacesDetailed` ([`remus_operations::multi_move`]): `distances` holds
//! one signed distance for every face, or one per face in `faces` order. A
//! selection that is one coplanar group moved by one distance (or a single
//! face) returns exactly what the single-group call returns.
//! `prepareMoveFacesMulti` prepares a drag: `moveFacesPrepared` and
//! `moveFacesPreparedJournaled` then move face `i` by `weights[i] ×
//! distance`; release it with `releasePreparedMove`.

use remus_operations::OperationsError;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;
use wasm_bindgen::prelude::*;

use crate::error::{StructuredWasmError, WasmError, validate_finite, validate_work_count};
use crate::handles::{edge_id_to_u32, face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;

#[wasm_bindgen]
impl RemusKernel {
    /// Move several planar faces together, each along its own outward
    /// normal, and return the new solid handle.
    ///
    /// `distances` holds one signed distance for every face, or one per
    /// face. Faces may lie on different parallel planes, have different or
    /// opposite normals, and share edges; every edge and vertex is rebuilt
    /// once from its final supports, keeping the source topology.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid handle, an empty selection or one of
    /// more than 64 faces, a distance list of the wrong length, a zero or
    /// non-finite distance, or a move that collapses, inverts or splits a
    /// face (the error names the face). A refusal leaves the topology
    /// unchanged.
    #[wasm_bindgen(js_name = "moveFacesMulti")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn move_faces_multi_binding(
        &mut self,
        solid: u32,
        faces: Vec<u32>,
        distances: Vec<f64>,
    ) -> Result<u32, JsError> {
        Ok(self.move_faces_multi_impl(solid, &faces, &distances)?)
    }

    /// `moveFacesMulti` journaled as one `move_faces` entry with exact face,
    /// edge and vertex history. Returns JSON `{"solid", "op"}`.
    ///
    /// # Errors
    ///
    /// As `moveFacesMulti`; a refusal restores topology and history.
    #[wasm_bindgen(js_name = "moveFacesMultiJournaled")]
    pub fn move_faces_multi_journaled_js(
        &mut self,
        solid: u32,
        faces: &[u32],
        distances: &[f64],
    ) -> Result<String, JsError> {
        self.move_faces_multi_journaled_json(solid, faces, distances)
            .map(|value| value.to_string())
            .map_err(|error| JsError::new(error.message()))
    }

    /// Prepare repeated multi-face moves for an interactive drag.
    ///
    /// `moveFacesPrepared(handle, distance)` and
    /// `moveFacesPreparedJournaled(handle, distance)` then return what
    /// `moveFacesMulti` and `moveFacesMultiJournaled` return with face `i`
    /// moved `weights[i] × distance` (omit `weights` for all ones), and
    /// refuse once the source changed since preparation. Release with
    /// `releasePreparedMove`. Preparing edits nothing.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid handle, weights that are not finite
    /// or not one per face, a selection over the work limits, or when 16
    /// prepared moves are already live.
    #[wasm_bindgen(js_name = "prepareMoveFacesMulti")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn prepare_move_faces_multi(
        &mut self,
        solid: u32,
        faces: Vec<u32>,
        weights: Option<Vec<f64>>,
    ) -> Result<u32, JsError> {
        Ok(self.prepare_move_faces_multi_impl(solid, &faces, weights.as_deref().unwrap_or(&[]))?)
    }
}

/// Natively testable bodies of the bindings above.
impl RemusKernel {
    /// Resolves and bounds a multi-face selection, as `moveFaces` does.
    pub(crate) fn resolve_multi_move(
        &self,
        solid: u32,
        faces: &[u32],
        distances: &[f64],
    ) -> Result<(SolidId, Vec<FaceId>), WasmError> {
        for &distance in distances {
            validate_finite(distance, "distance")?;
        }
        validate_work_count(u32::try_from(faces.len()).unwrap_or(u32::MAX), "faces")?;
        let solid_id = self.resolve_solid(solid)?;
        let face_ids = faces
            .iter()
            .map(|&face| self.resolve_face(face))
            .collect::<Result<Vec<_>, _>>()?;
        super::operations::validate_move_faces_topology_work(self.topo(), solid_id, &face_ids)?;
        Ok((solid_id, face_ids))
    }

    pub(crate) fn move_faces_multi_impl(
        &mut self,
        solid: u32,
        faces: &[u32],
        distances: &[f64],
    ) -> Result<u32, WasmError> {
        let (solid_id, face_ids) = self.resolve_multi_move(solid, faces, distances)?;
        // The operation owns its rollback, as `moveFaces` does.
        let result = remus_operations::multi_move::move_faces_multi(
            self.topo_mut(),
            solid_id,
            &face_ids,
            distances,
        )?;
        Ok(solid_id_to_u32(result))
    }

    /// [`Self::move_faces_multi_impl`] with its refusal projected as
    /// structured data (the batch op).
    pub(crate) fn move_faces_multi_structured(
        &mut self,
        solid: u32,
        faces: &[u32],
        distances: &[f64],
    ) -> Result<u32, StructuredWasmError> {
        let (solid_id, face_ids) = self
            .resolve_multi_move(solid, faces, distances)
            .map_err(StructuredWasmError::from)?;
        let input = MoveInput::capture(self.topo(), solid_id).map_err(StructuredWasmError::from)?;
        let result = remus_operations::multi_move::move_faces_multi(
            self.topo_mut(),
            solid_id,
            &face_ids,
            distances,
        )
        .map_err(|error| input.refusal(error))?;
        Ok(solid_id_to_u32(result))
    }

    pub(crate) fn move_faces_multi_journaled_json(
        &mut self,
        solid: u32,
        faces: &[u32],
        distances: &[f64],
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let (solid_id, face_ids) = self
            .resolve_multi_move(solid, faces, distances)
            .map_err(StructuredWasmError::from)?;
        let input = MoveInput::capture(self.topo(), solid_id).map_err(StructuredWasmError::from)?;
        let result = remus_operations::multi_move::move_faces_multi_journaled(
            self.topo_mut(),
            solid_id,
            &face_ids,
            distances,
        )
        .map_err(|error| input.refusal(error))?;
        Ok(serde_json::json!({
            "solid": solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    pub(crate) fn prepare_move_faces_multi_impl(
        &mut self,
        solid: u32,
        faces: &[u32],
        weights: &[f64],
    ) -> Result<u32, WasmError> {
        let (solid_id, face_ids) = self.resolve_multi_move(solid, faces, weights)?;
        self.insert_prepared_move(|topo| {
            remus_operations::prepared_move::PreparedFaceMove::prepare_multi(
                topo, solid_id, &face_ids, weights,
            )
        })
    }
}

/// The input solid's faces and edges, read before a face move so that a
/// refusal can name them: a move restores its source on failure, so these
/// handles stay live.
pub struct MoveInput {
    faces: Vec<FaceId>,
    edges: Vec<EdgeId>,
}

impl MoveInput {
    /// Reads `solid`'s faces and edges.
    ///
    /// # Errors
    ///
    /// Returns an error when `solid` is not a live solid of `topo`.
    pub fn capture(topo: &Topology, solid: SolidId) -> Result<Self, WasmError> {
        Ok(Self {
            faces: solid_faces(topo, solid)?,
            edges: solid_edges(topo, solid)?,
        })
    }

    /// Projects a face-move failure. A `topology_change` refusal carries the
    /// `face` and `edge` it names as handles, each only when it belongs to
    /// the input solid; an `ambiguous_rim_chamfer` refusal its band `face`,
    /// `wall`, `cap` and cap `edge`.
    pub fn refusal(&self, error: OperationsError) -> StructuredWasmError {
        if let OperationsError::AmbiguousRimChamfer(band) = &error {
            let band = *band;
            let mut structured = StructuredWasmError::from(error);
            for (key, face) in [("face", band.face), ("wall", band.wall), ("cap", band.cap)] {
                if self.faces.contains(&face) {
                    structured = structured.with_detail(key, face_id_to_u32(face));
                }
            }
            if self.edges.contains(&band.edge) {
                structured = structured.with_detail("edge", edge_id_to_u32(band.edge));
            }
            return structured;
        }
        let named = match &error {
            OperationsError::Offset(remus_offset::OffsetError::TopologyChange {
                face,
                edge,
                ..
            }) => Some((
                face.filter(|face| self.faces.contains(face)),
                edge.filter(|edge| self.edges.contains(edge)),
            )),
            _ => None,
        };
        let mut structured = StructuredWasmError::from(error);
        if let Some((face, edge)) = named {
            if let Some(face) = face {
                structured = structured.with_detail("face", face_id_to_u32(face));
            }
            if let Some(edge) = edge {
                structured = structured.with_detail("edge", edge_id_to_u32(edge));
            }
        }
        structured
    }
}

#[cfg(test)]
mod tests;
