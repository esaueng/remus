//! O4.7 typed direct-method results for the modifier family.
//!
//! `filletDetailed`, `chamferDetailed`, `shellDetailed` and `offsetDetailed`
//! are additive twins of `fillet`, `chamfer`, `shell` and `offsetSolid`; the
//! blend variants `filletV2Detailed`, `chamferV2Detailed`,
//! `chamferDistanceAngleDetailed` and `filletVariableDetailed` twin
//! `filletV2`, `chamferV2`, `chamferDistanceAngle` and `filletVariable`. Each
//! runs the same engine path as its legacy method and returns the typed
//! [`SolidOperationDetailedResult`] the boolean twins return, never throwing
//! on a refusal:
//!
//! - **exact**: `status: "ok"`, `details.quality: "exact"`;
//! - **approximate, disclosed**: `status: "ok"`, `details.quality:
//!   "approximate"`, plus what was approximated;
//! - **refused**: `status: "error"` with the kernel diagnostic `code` and
//!   `category`, and the topology rolled back to its pre-call state.
//!
//! An exact-only request never publishes an approximate result. For offset
//! and every blend twin, `exactOnly: true` refuses with category
//! `quality_refused` and code `exact_only_unattainable`. Shell, like its
//! legacy method, is exact-only unless `approximationSpacing` is given; it
//! keeps that method's refusal of a kept NURBS face, which today projects as
//! `operation_failed` / `internal` because the kernel's `Unsupported` error
//! has no registry code yet (the same code `executeBatchV2` reports for
//! `shell`).
//!
//! The same bodies back the `executeBatch`/`executeBatchV2` ops of the same
//! names, whose `ok` value is this envelope, so direct and batch results are
//! identical by construction.

#![allow(clippy::missing_errors_doc)]

use std::collections::HashSet;

use serde_json::{Map, Value};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use remus_operations::blend_ops::BlendEngine;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use remus_operations::OperationsError;
use remus_operations::blend_ops::BlendResult;
use remus_topology::edge::EdgeId;

use super::operations::parse_variable_fillet_specs;
use crate::error::{StructuredWasmError, WasmError, validate_finite, validate_positive};
use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::helpers::panic_message;
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

/// Quality label of an exact result.
const QUALITY_EXACT: &str = "exact";
/// Quality label of a disclosed approximate result.
const QUALITY_APPROXIMATE: &str = "approximate";

#[wasm_bindgen]
impl BrepKernel {
    /// Fillet edges and return exact, disclosed-approximate, or refused
    /// results as typed data.
    ///
    /// Additive twin of [`fillet`](Self::fillet_solid): the same engine
    /// cascade and whole-selection rule, with the legacy method's return
    /// value and thrown errors unchanged. On success `details.engine` names
    /// the blend engine (`walking`, `rollingBall`, or `mixed`). A result
    /// that introduces a NURBS face — a sampled blend wall with no closed
    /// form — reports `quality: "approximate"` and lists those faces under
    /// `approximateFaces`. With `exactOnly: true` such a result is rolled
    /// back and refused (`quality_refused` / `exact_only_unattainable`).
    #[wasm_bindgen(js_name = "filletDetailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_detailed(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .fillet_detailed_impl(solid, &edge_handles, radius, exact_only.unwrap_or(false))
            .into_ts()?)
    }

    /// Chamfer edges and return exact, disclosed-approximate, or refused
    /// results as typed data.
    ///
    /// Additive twin of [`chamfer`](Self::chamfer_solid): the planar bevel
    /// first, then the walking builder, with the legacy method unchanged.
    /// `details.engine` is `planarBevel`, `walking`, or `mixed`; quality and
    /// `exactOnly` follow [`filletDetailed`](Self::fillet_detailed).
    #[wasm_bindgen(js_name = "chamferDetailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_detailed(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        distance: f64,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .chamfer_detailed_impl(solid, &edge_handles, distance, exact_only.unwrap_or(false))
            .into_ts()?)
    }

    /// Hollow a solid and return exact, disclosed-approximate, or refused
    /// results as typed data.
    ///
    /// Additive twin of [`shell`](Self::shell_solid). Like the legacy method
    /// it is exact-only by default: a kept NURBS face is refused with the
    /// legacy refusal (`operation_failed` / `internal`, message naming the
    /// NURBS face) and the topology rolled back. A positive finite
    /// `approximation_spacing` opts into the sampled NURBS inner skin, which
    /// is then disclosed as `quality: "approximate"` with the model-unit
    /// `deflection` and the `sampledFaces` handles of the input solid.
    #[wasm_bindgen(js_name = "shellDetailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn shell_detailed(
        &mut self,
        solid: u32,
        thickness: f64,
        open_faces: Vec<u32>,
        approximation_spacing: Option<f64>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .shell_detailed_impl(solid, thickness, &open_faces, approximation_spacing)
            .into_ts()?)
    }

    /// Offset every face of a solid and return exact, disclosed-approximate,
    /// or refused results as typed data.
    ///
    /// Additive twin of [`offsetSolid`](Self::offset_solid), on the same
    /// engine. Analytic faces offset exactly. A NURBS input face would be
    /// offset by a sampled refit, so a result reports `quality:
    /// "approximate"` and lists those input faces under `sampledFaces`; with
    /// `exactOnly: true` the call refuses before touching topology
    /// (`quality_refused` / `exact_only_unattainable`). The offset
    /// intersector does not yet join a NURBS face to any neighbour, so a
    /// permissive NURBS offset currently ends in that engine's own refusal.
    #[wasm_bindgen(js_name = "offsetDetailed")]
    pub fn offset_detailed(
        &mut self,
        solid: u32,
        distance: f64,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .offset_detailed_impl(solid, distance, exact_only.unwrap_or(false))
            .into_ts()?)
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Fillet edges on the v2 walking engine alone and return exact,
    /// disclosed-approximate, or refused results as typed data.
    ///
    /// Additive twin of [`filletV2`](Self::fillet_v2), unchanged. Quality,
    /// `details.engine` and `exactOnly` follow
    /// [`filletDetailed`](Self::fillet_detailed).
    #[wasm_bindgen(js_name = "filletV2Detailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_v2_detailed(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .fillet_v2_detailed_impl(solid, &edge_handles, radius, exact_only.unwrap_or(false))
            .into_ts()?)
    }

    /// Two-distance chamfer on the v2 blend engine, as typed data.
    ///
    /// Additive twin of [`chamferV2`](Self::chamfer_v2), unchanged. Quality,
    /// `details.engine` and `exactOnly` follow
    /// [`filletDetailed`](Self::fillet_detailed).
    #[wasm_bindgen(js_name = "chamferV2Detailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_v2_detailed(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        d1: f64,
        d2: f64,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .chamfer_v2_detailed_impl(solid, &edge_handles, d1, d2, exact_only.unwrap_or(false))
            .into_ts()?)
    }

    /// Distance-angle chamfer on the v2 blend engine, as typed data.
    ///
    /// Additive twin of [`chamferDistanceAngle`](Self::chamfer_distance_angle),
    /// unchanged; `angle` is in radians, inside `(0, π/2)`. Quality,
    /// `details.engine` and `exactOnly` follow
    /// [`filletDetailed`](Self::fillet_detailed).
    #[wasm_bindgen(js_name = "chamferDistanceAngleDetailed")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_distance_angle_detailed(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        distance: f64,
        angle: f64,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .chamfer_distance_angle_detailed_impl(
                solid,
                &edge_handles,
                distance,
                angle,
                exact_only.unwrap_or(false),
            )
            .into_ts()?)
    }

    /// Variable-radius fillet, as typed data.
    ///
    /// Additive twin of [`filletVariable`](Self::fillet_variable), unchanged:
    /// `json` is the same spec array. That engine carries no engine tag, so
    /// success details hold `quality` (and `approximateFaces`) but no
    /// `engine`; `exactOnly` follows
    /// [`filletDetailed`](Self::fillet_detailed). Malformed JSON is an
    /// `invalid_argument` refusal, not a thrown error.
    #[wasm_bindgen(js_name = "filletVariableDetailed")]
    pub fn fillet_variable_detailed(
        &mut self,
        solid: u32,
        json: &str,
        exact_only: Option<bool>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        let specs =
            serde_json::from_str::<Vec<Value>>(json).map_err(|error| WasmError::InvalidInput {
                reason: format!("invalid JSON: {error}"),
            });
        let result = match specs {
            Ok(specs) => {
                self.fillet_variable_detailed_impl(solid, &specs, exact_only.unwrap_or(false))
            }
            Err(error) => SolidOperationDetailedResult::error(
                StructuredWasmError::from(error).with_direct_operation("filletVariable"),
            ),
        };
        Ok(result.into_ts()?)
    }
}

/// Natively-testable bodies shared by the direct twins and the batch ops.
impl BrepKernel {
    pub(crate) fn fillet_detailed_impl(
        &mut self,
        solid: u32,
        edge_handles: &[u32],
        radius: f64,
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.run_modifier_detailed("fillet", "Fillet", |kernel| {
            validate_positive(radius, "radius")?;
            let solid_id = kernel.resolve_solid(solid)?;
            let edge_ids = edge_handles
                .iter()
                .map(|&handle| kernel.resolve_edge(handle))
                .collect::<Result<Vec<_>, _>>()?;
            let input_faces = face_set(kernel.topo(), solid_id)?;
            let result = crate::helpers::fillet_whole_selection_result(
                kernel.topo_mut(),
                solid_id,
                &edge_ids,
                radius,
            )
            .map_err(StructuredWasmError::blend_failure)?;
            blend_outcome(
                kernel.topo(),
                "fillet",
                &input_faces,
                result.solid,
                Some(result.engine),
                exact_only,
            )
        })
    }

    pub(crate) fn chamfer_detailed_impl(
        &mut self,
        solid: u32,
        edge_handles: &[u32],
        distance: f64,
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.run_modifier_detailed("chamfer", "Chamfer", |kernel| {
            validate_positive(distance, "distance")?;
            let solid_id = kernel.resolve_solid(solid)?;
            let edge_ids = edge_handles
                .iter()
                .map(|&handle| kernel.resolve_edge(handle))
                .collect::<Result<Vec<_>, _>>()?;
            let input_faces = face_set(kernel.topo(), solid_id)?;
            let (result, _, engine) = crate::helpers::try_chamfer_with_engine(
                kernel.topo_mut(),
                solid_id,
                &edge_ids,
                distance,
            )
            .map_err(StructuredWasmError::blend_failure)?;
            blend_outcome(
                kernel.topo(),
                "chamfer",
                &input_faces,
                result,
                Some(engine),
                exact_only,
            )
        })
    }

    pub(crate) fn shell_detailed_impl(
        &mut self,
        solid: u32,
        thickness: f64,
        open_faces: &[u32],
        approximation_spacing: Option<f64>,
    ) -> SolidOperationDetailedResult {
        use remus_operations::shell_op::{ShellQuality, shell_outcome_with_evolution};

        self.run_modifier_detailed("shell", "Shell", |kernel| {
            validate_positive(thickness, "thickness")?;
            if let Some(spacing) = approximation_spacing {
                validate_positive(spacing, "approximationSpacing")?;
            }
            let solid_id = kernel.resolve_solid(solid)?;
            let open_face_ids = open_faces
                .iter()
                .map(|&handle| kernel.resolve_face(handle))
                .collect::<Result<Vec<_>, _>>()?;
            let outcome = shell_outcome_with_evolution(
                kernel.topo_mut(),
                solid_id,
                thickness,
                &open_face_ids,
                approximation_spacing,
            )?
            .outcome;
            let mut details = Map::new();
            match outcome.quality {
                ShellQuality::Exact => {
                    details.insert("quality".into(), Value::from(QUALITY_EXACT));
                }
                ShellQuality::Approximate {
                    deflection,
                    sampled_faces,
                } => {
                    details.insert("quality".into(), Value::from(QUALITY_APPROXIMATE));
                    details.insert("deflection".into(), Value::from(deflection));
                    details.insert("sampledFaces".into(), Value::from(sampled_faces));
                }
            }
            Ok((solid_id_to_u32(outcome.solid), details))
        })
    }

    pub(crate) fn offset_detailed_impl(
        &mut self,
        solid: u32,
        distance: f64,
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.run_modifier_detailed("offset", "Offset", |kernel| {
            validate_finite(distance, "distance")?;
            let solid_id = kernel.resolve_solid(solid)?;
            // The offset engine refits every NURBS face from a sample grid
            // (`remus_offset::offset`); every analytic face offsets exactly.
            // That is known before the engine runs, so an exact-only request
            // is refused without touching topology.
            let mut sampled = Vec::new();
            for face in solid_faces(kernel.topo(), solid_id)? {
                if matches!(kernel.topo().face(face)?.surface(), FaceSurface::Nurbs(_)) {
                    sampled.push(face_id_to_u32(face));
                }
            }
            if exact_only && !sampled.is_empty() {
                return Err(StructuredWasmError::exact_only_unattainable(format!(
                    "exact-only policy: offset would refit {} NURBS face(s) from samples; \
                     the approximate offset was declined",
                    sampled.len()
                ))
                .with_detail("sampledFaces", sampled));
            }
            let result = remus_operations::offset_v2::offset_solid_v2(
                kernel.topo_mut(),
                solid_id,
                distance,
            )?;
            let mut details = Map::new();
            if sampled.is_empty() {
                details.insert("quality".into(), Value::from(QUALITY_EXACT));
            } else {
                details.insert("quality".into(), Value::from(QUALITY_APPROXIMATE));
                details.insert("sampledFaces".into(), Value::from(sampled));
            }
            Ok((solid_id_to_u32(result), details))
        })
    }

    pub(crate) fn fillet_v2_detailed_impl(
        &mut self,
        solid: u32,
        edge_handles: &[u32],
        radius: f64,
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.walking_blend_detailed(
            "filletV2",
            || validate_positive(radius, "radius"),
            solid,
            edge_handles,
            exact_only,
            |topo, solid, edges| remus_operations::blend_ops::fillet_v2(topo, solid, edges, radius),
        )
    }

    pub(crate) fn chamfer_v2_detailed_impl(
        &mut self,
        solid: u32,
        edge_handles: &[u32],
        d1: f64,
        d2: f64,
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.walking_blend_detailed(
            "chamferV2",
            || {
                validate_positive(d1, "d1")?;
                validate_positive(d2, "d2")
            },
            solid,
            edge_handles,
            exact_only,
            |topo, solid, edges| {
                remus_operations::blend_ops::chamfer_v2(topo, solid, edges, d1, d2)
            },
        )
    }

    pub(crate) fn chamfer_distance_angle_detailed_impl(
        &mut self,
        solid: u32,
        edge_handles: &[u32],
        distance: f64,
        angle: f64,
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.walking_blend_detailed(
            "chamferDistanceAngle",
            || {
                validate_positive(distance, "distance")?;
                validate_positive(angle, "angle")?;
                if angle >= std::f64::consts::FRAC_PI_2 {
                    return Err(WasmError::InvalidInput {
                        reason: "angle must be less than π/2".into(),
                    });
                }
                Ok(())
            },
            solid,
            edge_handles,
            exact_only,
            |topo, solid, edges| {
                remus_operations::blend_ops::chamfer_distance_angle(
                    topo, solid, edges, distance, angle,
                )
            },
        )
    }

    pub(crate) fn fillet_variable_detailed_impl(
        &mut self,
        solid: u32,
        specs: &[Value],
        exact_only: bool,
    ) -> SolidOperationDetailedResult {
        self.run_modifier_detailed("filletVariable", "Variable fillet", |kernel| {
            let solid_id = kernel.resolve_solid(solid)?;
            let edge_specs = parse_variable_fillet_specs(kernel, specs)?;
            let input_faces = face_set(kernel.topo(), solid_id)?;
            let result = remus_operations::fillet::fillet_variable_with_setbacks(
                kernel.topo_mut(),
                solid_id,
                &edge_specs,
            )
            .map_err(StructuredWasmError::blend_failure)?;
            blend_outcome(
                kernel.topo(),
                "variable fillet",
                &input_faces,
                result,
                None,
                exact_only,
            )
        })
    }

    /// Shared body of the single-engine walking-blend twins: argument checks,
    /// handle resolution, the engine call, then quality classification.
    fn walking_blend_detailed(
        &mut self,
        operation: &'static str,
        validate: impl FnOnce() -> Result<(), WasmError>,
        solid: u32,
        edge_handles: &[u32],
        exact_only: bool,
        engine: impl FnOnce(&mut Topology, SolidId, &[EdgeId]) -> Result<BlendResult, OperationsError>,
    ) -> SolidOperationDetailedResult {
        self.run_modifier_detailed(operation, operation, |kernel| {
            validate()?;
            let solid_id = kernel.resolve_solid(solid)?;
            let edge_ids = edge_handles
                .iter()
                .map(|&handle| kernel.resolve_edge(handle))
                .collect::<Result<Vec<_>, _>>()?;
            let input_faces = face_set(kernel.topo(), solid_id)?;
            let result = engine(kernel.topo_mut(), solid_id, &edge_ids)
                .map_err(StructuredWasmError::blend_failure)?;
            blend_outcome(
                kernel.topo(),
                operation,
                &input_faces,
                result.solid,
                Some(result.engine),
                exact_only,
            )
        })
    }

    /// Shared envelope discipline for the modifier twins.
    ///
    /// Refuses on a poisoned kernel, snapshots topology, and restores it on
    /// any refusal with [`Topology::restore_for_rollback`]: the refused call
    /// was never observed, so its allocations *and* retirements are undone
    /// and every live entity is exactly as it was. Handles the attempt
    /// allocated stay permanently invalid (slots are never reused). The
    /// unwind guard only helps native hosts (wasm32 aborts on
    /// panic; `panics.rs` records the message there); a caught panic poisons
    /// the kernel exactly as the legacy `fillet` binding does.
    fn run_modifier_detailed(
        &mut self,
        operation: &'static str,
        panic_label: &str,
        body: impl FnOnce(&mut Self) -> Result<(u32, Map<String, Value>), StructuredWasmError>,
    ) -> SolidOperationDetailedResult {
        if self.poisoned {
            return SolidOperationDetailedResult::error(
                StructuredWasmError::operation_failed(
                    "Kernel poisoned after panic. Create a new BrepKernel instance.",
                )
                .with_direct_operation(operation),
            );
        }
        let snapshot = self.topo().clone();
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| body(self)));
        match outcome {
            Ok(Ok((value, details))) => {
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Ok(Err(error)) => {
                self.topo_mut().restore_for_rollback(&snapshot);
                SolidOperationDetailedResult::error(error.with_direct_operation(operation))
            }
            Err(panic_info) => {
                self.poisoned = true;
                self.topo_mut().restore_for_rollback(&snapshot);
                SolidOperationDetailedResult::error(
                    StructuredWasmError::operation_failed(panic_message(&panic_info, panic_label))
                        .with_direct_operation(operation),
                )
            }
        }
    }
}

/// Face set of a solid before a modifier runs.
fn face_set(
    topo: &Topology,
    solid: SolidId,
) -> Result<HashSet<FaceId>, remus_topology::TopologyError> {
    Ok(solid_faces(topo, solid)?.into_iter().collect())
}

/// Stable wire name of a blend engine.
const fn engine_name(engine: BlendEngine) -> &'static str {
    match engine {
        BlendEngine::Walking => "walking",
        BlendEngine::RollingBall => "rollingBall",
        BlendEngine::PlanarBevel => "planarBevel",
        BlendEngine::Mixed => "mixed",
    }
}

/// Classify a committed blend result and apply the exact-only policy.
///
/// A result face that is not an input face and carries a NURBS surface is a
/// blend wall (or a re-minted neighbour) with no certified closed form, so
/// the result is disclosed as approximate. This errs toward disclosure: a
/// re-minted input NURBS face is also listed. Analytic walls (the rolling
/// ball's cylinders, the walking builder's analytic fast paths, the planar
/// bevel) are exact.
fn blend_outcome(
    topo: &Topology,
    operation: &str,
    input_faces: &HashSet<FaceId>,
    result: SolidId,
    engine: Option<BlendEngine>,
    exact_only: bool,
) -> Result<(u32, Map<String, Value>), StructuredWasmError> {
    let mut approximate = Vec::new();
    for face in solid_faces(topo, result)? {
        if !input_faces.contains(&face)
            && matches!(topo.face(face)?.surface(), FaceSurface::Nurbs(_))
        {
            approximate.push(face_id_to_u32(face));
        }
    }
    if exact_only && !approximate.is_empty() {
        let mut refusal = StructuredWasmError::exact_only_unattainable(format!(
            "exact-only policy: the {operation} produced {} NURBS face(s) with no exact \
             form; the approximate result was declined",
            approximate.len()
        ))
        .with_detail("approximateFaceCount", approximate.len());
        if let Some(engine) = engine {
            refusal = refusal.with_detail("engine", engine_name(engine));
        }
        return Err(refusal);
    }
    let mut details = Map::new();
    if let Some(engine) = engine {
        details.insert("engine".into(), Value::from(engine_name(engine)));
    }
    if approximate.is_empty() {
        details.insert("quality".into(), Value::from(QUALITY_EXACT));
    } else {
        details.insert("quality".into(), Value::from(QUALITY_APPROXIMATE));
        details.insert("approximateFaces".into(), Value::from(approximate));
    }
    Ok((solid_id_to_u32(result), details))
}

#[cfg(test)]
mod tests;
