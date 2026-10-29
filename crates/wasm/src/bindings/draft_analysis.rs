//! Read-only signed draft-angle queries over live session handles.

use remus_check::analyze::draft::{self, DraftAnalysisError};
use remus_math::vec::Vec3;
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::handles::face_id_to_u32;
use crate::kernel::BrepKernel;
use crate::types::{DraftAngleResult, PlanarFaceDraftResult};

fn failure(error: DraftAnalysisError) -> StructuredWasmError {
    let (code, structured) = match error {
        DraftAnalysisError::Topology(error) => (
            "draft-analysis:invalid-topology",
            StructuredWasmError::from(error),
        ),
        DraftAnalysisError::InvalidPullDirection => (
            "draft-analysis:invalid-pull",
            StructuredWasmError::invalid_argument("pull must be finite and nonzero", Some("pull")),
        ),
        DraftAnalysisError::InvalidParameters => (
            "draft-analysis:invalid-parameters",
            StructuredWasmError::invalid_argument("parameters outside the surface domain", None),
        ),
        DraftAnalysisError::UndefinedNormal => (
            "draft-analysis:undefined-normal",
            StructuredWasmError::unsupported("surface has no qualified regular normal"),
        ),
        DraftAnalysisError::NonPlanarFace { face } => (
            "draft-analysis:non-planar-face",
            StructuredWasmError::unsupported("constant draft report requires planar faces")
                .with_detail("face", face_id_to_u32(face)),
        ),
    };
    let message = format!("{code}: {}", structured.message());
    structured
        .with_message(message)
        .with_detail("kernelCode", code)
}

fn direct_error(error: StructuredWasmError) -> JsError {
    let message = error.message().to_owned();
    let (prefix, _, _) = error.into_direct_parts();
    if message.starts_with(&format!("{prefix}:")) {
        JsError::new(&message)
    } else {
        JsError::new(&format!("{prefix}: {message}"))
    }
}

impl BrepKernel {
    pub(crate) fn face_draft_result(
        &self,
        face: u32,
        u: f64,
        v: f64,
        pull: Vec3,
    ) -> Result<DraftAngleResult, StructuredWasmError> {
        let face = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let result = draft::surface_draft_angle(&self.topo, face, u, v, pull).map_err(failure)?;
        Ok(DraftAngleResult {
            angle_radians: result.angle_radians,
            outward_normal: vec![
                result.outward_normal.x(),
                result.outward_normal.y(),
                result.outward_normal.z(),
            ],
        })
    }

    pub(crate) fn planar_draft_result(
        &self,
        solid: u32,
        pull: Vec3,
    ) -> Result<Vec<PlanarFaceDraftResult>, StructuredWasmError> {
        let solid = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let result = draft::planar_solid_draft(&self.topo, solid, pull).map_err(failure)?;
        Ok(result
            .into_iter()
            .map(|row| PlanarFaceDraftResult {
                face: face_id_to_u32(row.face),
                angle_radians: row.sample.angle_radians,
                outward_normal: vec![
                    row.sample.outward_normal.x(),
                    row.sample.outward_normal.y(),
                    row.sample.outward_normal.z(),
                ],
            })
            .collect())
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Signed local surface-carrier draft in radians, as a JSON string
    /// `{angleRadians,outwardNormal}`. `(u,v)` does not test trimmed membership.
    /// Negative draft is not a global undercut or accessibility verdict.
    ///
    /// # Errors
    /// Refuses invalid handles, pull directions, parameters and undefined normals.
    #[wasm_bindgen(js_name = "getFaceDraftAngle")]
    pub fn get_face_draft_angle(
        &self,
        face: u32,
        u: f64,
        v: f64,
        pull_x: f64,
        pull_y: f64,
        pull_z: f64,
    ) -> Result<String, JsError> {
        let result = self
            .face_draft_result(face, u, v, Vec3::new(pull_x, pull_y, pull_z))
            .map_err(direct_error)?;
        serde_json::to_string(&result).map_err(|error| JsError::new(&error.to_string()))
    }

    /// JSON array of `{face,angleRadians,outwardNormal}` for every outer/cavity
    /// planar face in explorer order. A curved face refuses the whole report.
    /// This read-only query does not change topology, handles or checkpoints.
    ///
    /// # Errors
    /// Refuses invalid handles/pull directions, undefined normals and curved faces.
    #[wasm_bindgen(js_name = "getPlanarSolidDraft")]
    pub fn get_planar_solid_draft(
        &self,
        solid: u32,
        pull_x: f64,
        pull_y: f64,
        pull_z: f64,
    ) -> Result<String, JsError> {
        let result = self
            .planar_draft_result(solid, Vec3::new(pull_x, pull_y, pull_z))
            .map_err(direct_error)?;
        serde_json::to_string(&result).map_err(|error| JsError::new(&error.to_string()))
    }
}

#[cfg(test)]
mod tests;
