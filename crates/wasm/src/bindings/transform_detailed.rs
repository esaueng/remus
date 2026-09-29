//! O4.7 typed transform results (B74).
//!
//! `transformDetailed` (twin of `transformSolid`) and
//! `copyAndTransformSolidDetailed` (twin of `copyAndTransformSolid`) run
//! the same engine as their legacy methods and return the typed
//! [`SolidOperationDetailedResult`] envelope, never throwing on a refusal:
//!
//! - **exact**: `status: "ok"`, `details.quality: "exact"`, plus
//!   `carrierChanges`/`edgeChanges` naming every family change;
//! - **approximate, disclosed**: `status: "ok"`, `details.quality:
//!   "approximate"`, plus `fittedFaces` with the sampled evidence;
//! - **refused**: `status: "error"` with the kernel diagnostic `code` and
//!   `category`, and the topology rolled back to its pre-call state.
//!
//! With `exactOnly: true` a fitted-sphere need refuses before any mutation
//! (`quality_refused` / `exact_only_unattainable`) and names the refused
//! faces. The legacy methods keep their signatures, validation order,
//! and return shapes unchanged.
//!
//! The same bodies back the `executeBatch`/`executeBatchV2` ops of the
//! same names, whose `ok` value is this envelope, so direct and batch
//! results are identical by construction.

#![allow(clippy::missing_errors_doc)]

use serde_json::{Map, Value};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use remus_math::mat::Mat4;
use remus_operations::transform::{TransformPolicy, TransformReport};

use crate::error::StructuredWasmError;
use crate::handles::{edge_id_to_u32, face_id_to_u32, solid_id_to_u32};
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

/// Quality label of an exact result.
const QUALITY_EXACT: &str = "exact";
/// Quality label of a disclosed approximate result.
const QUALITY_APPROXIMATE: &str = "approximate";

/// Validate a 16-element row-major matrix like the legacy direct methods
/// do (length, then finiteness), returning data errors instead of throwing.
pub fn parse_transform_matrix(matrix: &[f64]) -> Result<Mat4, StructuredWasmError> {
    if matrix.len() != 16 {
        return Err(StructuredWasmError::invalid_argument(
            format!(
                "transform matrix must have 16 elements, got {}",
                matrix.len()
            ),
            Some("matrix"),
        ));
    }
    if let Some(pos) = matrix.iter().position(|v| !v.is_finite()) {
        return Err(StructuredWasmError::invalid_argument(
            format!("matrix element at index {pos} is not finite"),
            Some("matrix"),
        ));
    }
    let rows = std::array::from_fn(|i| std::array::from_fn(|j| matrix[i * 4 + j]));
    Ok(Mat4(rows))
}

/// Parse a batch-JSON matrix value with the same row-major contract.
pub fn parse_transform_matrix_json(value: &serde_json::Value) -> Result<Mat4, StructuredWasmError> {
    let array = value.as_array().ok_or("missing or invalid 'matrix'")?;
    if array.len() != 16 {
        return Err(StructuredWasmError::invalid_argument(
            format!(
                "transform matrix must have 16 elements, got {}",
                array.len()
            ),
            Some("matrix"),
        ));
    }
    let mut elems = Vec::with_capacity(16);
    for (i, entry) in array.iter().enumerate() {
        elems.push(
            entry
                .as_f64()
                .ok_or_else(|| format!("matrix[{i}] is not a number"))?,
        );
    }
    parse_transform_matrix(&elems)
}

/// Render a [`TransformReport`] into the envelope's details map.
pub fn report_details(report: &TransformReport) -> Map<String, Value> {
    let mut details = Map::new();
    details.insert(
        "quality".into(),
        Value::from(match report.quality {
            remus_operations::transform::TransformQuality::Exact => QUALITY_EXACT,
            remus_operations::transform::TransformQuality::Approximate => QUALITY_APPROXIMATE,
        }),
    );
    details.insert("determinant".into(), Value::from(report.determinant));
    details.insert(
        "orientationReversed".into(),
        Value::from(report.orientation_reversed),
    );
    details.insert("similarity".into(), Value::from(report.similarity));
    details.insert(
        "carrierChanges".into(),
        report
            .face_changes
            .iter()
            .map(|change| {
                serde_json::json!({
                    "face": face_id_to_u32(change.face),
                    "from": change.from,
                    "to": change.to,
                    "method": change.method.as_str(),
                })
            })
            .collect(),
    );
    details.insert(
        "edgeChanges".into(),
        report
            .edge_changes
            .iter()
            .map(|change| {
                serde_json::json!({
                    "edge": edge_id_to_u32(change.edge),
                    "from": change.from,
                    "to": change.to,
                })
            })
            .collect(),
    );
    details.insert(
        "fittedFaces".into(),
        report
            .fitted_faces
            .iter()
            .map(|fit| {
                serde_json::json!({
                    "face": face_id_to_u32(fit.face),
                    "from": fit.from,
                    "method": fit.method,
                    "gridU": fit.grid.0,
                    "gridV": fit.grid.1,
                    "maxResidual": fit.max_residual,
                    "checkPoints": fit.check_points,
                })
            })
            .collect(),
    );
    details.insert(
        "controlPoints".into(),
        Value::from(report.control_points_total),
    );
    details
}

#[wasm_bindgen]
impl BrepKernel {
    /// Transform a solid and return the exact, disclosed-approximate, or
    /// refused result as typed data.
    ///
    /// Additive twin of [`transformSolid`](Self::transform_solid_binding):
    /// the same engine, matrix contract (16 row-major values), and
    /// validation order, with the legacy method's void return and thrown
    /// errors unchanged. On success `value` is the (same) solid handle and
    /// `details` carries `quality`, `determinant`, `orientationReversed`,
    /// `similarity`, `carrierChanges`, `edgeChanges`, `fittedFaces`, and
    /// `controlPoints`. With `exactOnly: true` a fitted-sphere need is
    /// rolled back and refused (`quality_refused` /
    /// `exact_only_unattainable`), naming the refused faces.
    #[wasm_bindgen(js_name = "transformDetailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn transform_detailed(
        &mut self,
        solid: u32,
        matrix: Vec<f64>,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .transform_detailed_impl(solid, &matrix, exact_only.unwrap_or(false))
            .into_ts()?)
    }

    /// Copy a solid through an affine transform and return the exact,
    /// disclosed-approximate, or refused result as typed data.
    ///
    /// Additive twin of
    /// [`copyAndTransformSolid`](Self::copy_and_transform_solid): the same
    /// single-pass copy engine and matrix contract, with the legacy
    /// method's return and errors unchanged. On success `value` is the new
    /// solid handle; quality and `exactOnly` follow
    /// [`transformDetailed`](Self::transform_detailed).
    #[wasm_bindgen(js_name = "copyAndTransformSolidDetailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn copy_and_transform_solid_detailed(
        &mut self,
        solid: u32,
        matrix: Vec<f64>,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .copy_and_transform_solid_detailed_impl(solid, &matrix, exact_only.unwrap_or(false))
            .into_ts()?)
    }
}

/// Natively-testable bodies shared by the direct twins and the batch ops.
impl BrepKernel {
    pub(crate) fn transform_detailed_impl(
        &mut self,
        solid: u32,
        matrix: &[f64],
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        let policy = if exact_only {
            TransformPolicy::ExactOnly
        } else {
            TransformPolicy::AllowApproximate
        };
        let mut execution_refusal = None;
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let mat = parse_transform_matrix(matrix)?;
            let solid_id = self
                .resolve_solid(solid)
                .map_err(StructuredWasmError::from)?;
            // The native twin owns rollback (transacted); no additional
            // full-session snapshot is taken here.
            let report = remus_operations::transform::transform_solid_detailed_with_refusal(
                self.topo_mut(),
                solid_id,
                &mat,
                policy,
                &mut execution_refusal,
            )
            .map_err(StructuredWasmError::from)?;
            Ok((solid_id_to_u32(solid_id), report_details(&report)))
        })();

        match result {
            Ok((value, details)) => {
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Err(error) => SolidOperationDetailedResult::error(enrich_exact_only_refusal(
                self,
                error.with_direct_operation("transform"),
                matrix,
                solid,
                execution_refusal,
            )),
        }
    }

    pub(crate) fn copy_and_transform_solid_detailed_impl(
        &mut self,
        solid: u32,
        matrix: &[f64],
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        let policy = if exact_only {
            TransformPolicy::ExactOnly
        } else {
            TransformPolicy::AllowApproximate
        };
        let mut execution_refusal = None;
        let result = (|| -> Result<(u32, Map<String, Value>), StructuredWasmError> {
            let mat = parse_transform_matrix(matrix)?;
            let solid_id = self
                .resolve_solid(solid)
                .map_err(StructuredWasmError::from)?;
            let (copied, report) =
                remus_operations::copy::copy_and_transform_solid_detailed_with_refusal(
                    self.topo_mut(),
                    solid_id,
                    &mat,
                    policy,
                    &mut execution_refusal,
                )
                .map_err(StructuredWasmError::from)?;
            Ok((solid_id_to_u32(copied), report_details(&report)))
        })();

        match result {
            Ok((value, details)) => {
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Err(error) => SolidOperationDetailedResult::error(enrich_exact_only_refusal(
                self,
                error.with_direct_operation("copyAndTransformSolid"),
                matrix,
                solid,
                execution_refusal,
            )),
        }
    }
}

/// Name the refused faces on an exact-only refusal.
///
/// Execution records the source face before rollback. A preflight refusal
/// reruns the read-only plan to name the faces without mutating topology.
fn enrich_exact_only_refusal(
    kernel: &BrepKernel,
    error: StructuredWasmError,
    matrix: &[f64],
    solid: u32,
    execution_refusal: Option<remus_topology::face::FaceId>,
) -> StructuredWasmError {
    if error.kernel_code() != Some("exact_only_unattainable") {
        return error;
    }
    if let Some(face) = execution_refusal {
        return error.with_detail(
            "fittedFaces",
            Value::Array(vec![
                serde_json::json!({"face": face_id_to_u32(face), "from": "sphere"}),
            ]),
        );
    }
    let Ok(mat) = parse_transform_matrix(matrix) else {
        return error;
    };
    let Ok(solid_id) = kernel.resolve_solid(solid) else {
        return error;
    };
    let Ok(plans) =
        remus_operations::transform::preflight_solid_transform(kernel.topo(), solid_id, &mat)
    else {
        return error;
    };
    let fitted: Vec<Value> = plans
        .iter()
        .filter(|plan| {
            matches!(
                plan.method,
                remus_operations::transform::SurfaceMethod::SampledFit
            )
        })
        .map(|plan| {
            serde_json::json!({
                "face": face_id_to_u32(plan.face),
                "from": plan.from,
            })
        })
        .collect();
    error.with_detail("fittedFaces", Value::Array(fitted))
}

#[cfg(test)]
mod tests;
