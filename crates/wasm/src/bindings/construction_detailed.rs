//! O4.7 typed direct-method results for the construction family.
//!
//! `extrudeDetailed` and `revolveDetailed` are additive twins of `extrude`
//! and `revolve`; the legacy methods keep their return values and
//! thrown-error behavior. Each runs the same engine path as its legacy
//! method and returns the typed [`SolidOperationDetailedResult`] the boolean
//! twins return, never throwing on a refusal:
//!
//! - **exact**: `status: "ok"`, `details.quality: "exact"`;
//! - **refused**: `status: "error"` with the kernel diagnostic `code` and
//!   `category`, and the topology rolled back to its pre-call state.
//!
//! Both engines are exact-only by construction: an extruded wall is the
//! translated profile curve (an analytic cylinder for circles, an exact
//! ruled NURBS for ellipse/conic/spline edges) and a revolved band is the
//! exact surface of revolution (analytic cylinder/cone/plane/torus where the
//! profile classifies, otherwise the exact rational revolution NURBS). There
//! is no sampled-refit fallback, so there is no `approximate` success to
//! disclose and no `exactOnly` flag to accept: the approximation cells of the
//! acceptance matrix are unreachable by design, and every supported profile
//! in the matrix commits with `quality: "exact"`.
//!
//! WASM `revolve` accepts degrees in `(0, 360]`; the native operation
//! receives radians. Both the direct twin and the batch op preserve that
//! conversion exactly (`angle_degrees.to_radians()`), matching the legacy
//! direct and batch paths.
//!
//! Transactionality reuses the native coordination: `extrude` and `revolve`
//! both run inside `run_transacted`, so a refusal already restores the exact
//! pre-call topology. The twins take no additional full-session snapshot;
//! the batch layer's per-op rollback covers the dispatch boundary as for
//! every other mutating op.
//!
//! The same bodies back the `executeBatch`/`executeBatchV2` ops of the same
//! names, whose `ok` value is this envelope, so direct and batch results are
//! identical by construction.

#![allow(clippy::missing_errors_doc)]

use serde_json::{Map, Value};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use remus_math::vec::{Point3, Vec3};

use crate::error::{StructuredWasmError, WasmError, validate_finite};
use crate::handles::solid_id_to_u32;
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

/// Quality label of every successful construction result.
const QUALITY_EXACT: &str = "exact";

#[wasm_bindgen]
impl BrepKernel {
    /// Extrude a planar face along a direction vector, as typed data.
    ///
    /// Additive twin of [`extrude`](Self::extrude_face): the same validation
    /// order (finite `dir_x`, `dir_y`, `dir_z`, `distance`, then the face
    /// handle) and the same engine, with the legacy method unchanged. On
    /// success `details.quality` is `"exact"`; there is no approximate
    /// extrusion to disclose. A refusal carries the legacy failure's code
    /// and category with `details.operation: "extrude"`.
    #[wasm_bindgen(js_name = "extrudeDetailed")]
    pub fn extrude_detailed(
        &mut self,
        face: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        distance: f64,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .extrude_detailed_impl(face, dir_x, dir_y, dir_z, distance)
            .into_ts()?)
    }

    /// Revolve a planar face around an axis, as typed data.
    ///
    /// Additive twin of [`revolve`](Self::revolve_face): the same validation
    /// order (finite `ox`, `oy`, `oz`, `dx`, `dy`, `dz`, `angle_degrees`,
    /// then the `(0, 360]` range check, then the face handle), the same
    /// degrees-to-radians conversion, and the same engine, with the legacy
    /// method unchanged. On success `details.quality` is `"exact"`. A
    /// refusal carries the legacy failure's code and category with
    /// `details.operation: "revolve"`.
    #[wasm_bindgen(js_name = "revolveDetailed")]
    #[allow(clippy::too_many_arguments)]
    pub fn revolve_detailed(
        &mut self,
        face: u32,
        ox: f64,
        oy: f64,
        oz: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle_degrees: f64,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .revolve_detailed_impl(face, ox, oy, oz, dx, dy, dz, angle_degrees)
            .into_ts()?)
    }
}

/// Natively-testable bodies shared by the direct twins and the batch ops.
impl BrepKernel {
    pub(crate) fn extrude_detailed_impl(
        &mut self,
        face: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        distance: f64,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<u32, StructuredWasmError> {
            validate_finite(dir_x, "dir_x").map_err(StructuredWasmError::from)?;
            validate_finite(dir_y, "dir_y").map_err(StructuredWasmError::from)?;
            validate_finite(dir_z, "dir_z").map_err(StructuredWasmError::from)?;
            validate_finite(distance, "distance").map_err(StructuredWasmError::from)?;
            let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
            let direction = Vec3::new(dir_x, dir_y, dir_z);
            // `extrude` owns its native rollback transaction; no additional
            // full-session snapshot is taken here.
            let solid =
                remus_operations::extrude::extrude(self.topo_mut(), face_id, direction, distance)
                    .map_err(StructuredWasmError::from)?;
            Ok(solid_id_to_u32(solid))
        })();

        match result {
            Ok(value) => {
                let mut details = Map::new();
                details.insert("quality".into(), Value::from(QUALITY_EXACT));
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("extrude"))
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn revolve_detailed_impl(
        &mut self,
        face: u32,
        ox: f64,
        oy: f64,
        oz: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle_degrees: f64,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<u32, StructuredWasmError> {
            validate_finite(ox, "ox").map_err(StructuredWasmError::from)?;
            validate_finite(oy, "oy").map_err(StructuredWasmError::from)?;
            validate_finite(oz, "oz").map_err(StructuredWasmError::from)?;
            validate_finite(dx, "dx").map_err(StructuredWasmError::from)?;
            validate_finite(dy, "dy").map_err(StructuredWasmError::from)?;
            validate_finite(dz, "dz").map_err(StructuredWasmError::from)?;
            validate_finite(angle_degrees, "angle_degrees").map_err(StructuredWasmError::from)?;
            if angle_degrees <= 0.0 || angle_degrees > 360.0 {
                return Err(StructuredWasmError::from(WasmError::InvalidInput {
                    reason: format!("angle_degrees must be in (0, 360], got {angle_degrees}"),
                }));
            }
            let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
            let origin = Point3::new(ox, oy, oz);
            let direction = Vec3::new(dx, dy, dz);
            let angle_radians = angle_degrees.to_radians();
            // `revolve` owns its native rollback transaction; no additional
            // full-session snapshot is taken here.
            let solid = remus_operations::revolve::revolve(
                self.topo_mut(),
                face_id,
                origin,
                direction,
                angle_radians,
            )
            .map_err(StructuredWasmError::from)?;
            Ok(solid_id_to_u32(solid))
        })();

        match result {
            Ok(value) => {
                let mut details = Map::new();
                details.insert("quality".into(), Value::from(QUALITY_EXACT));
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("revolve"))
            }
        }
    }
}

#[cfg(test)]
mod tests;
