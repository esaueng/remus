//! Shape healing, validation, and feature recognition bindings.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use brepkit_topology::face::Face;

use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::helpers::{TOL, serialize_feature};
use crate::kernel::BrepKernel;

#[wasm_bindgen]
impl BrepKernel {
    // -- Sewing ----------------------------------------------------------------

    /// Sew loose faces into a connected solid.
    ///
    /// `face_handles` is an array of face handles. Returns a solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if fewer than 2 faces or sewing fails.
    #[wasm_bindgen(js_name = "sewFaces")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn sew_faces(&mut self, face_handles: Vec<u32>, tolerance: f64) -> Result<u32, JsError> {
        let face_ids: Vec<brepkit_topology::face::FaceId> = face_handles
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let solid = brepkit_operations::sew::sew_faces(self.topo_mut(), &face_ids, tolerance)?;
        Ok(solid_id_to_u32(solid))
    }

    /// Create a solid from a set of faces by sewing them together.
    ///
    /// Alias for `sewFaces` with a default tolerance. This is the equivalent
    /// of sewing faces into a closed shell and building a solid.
    #[wasm_bindgen(js_name = "makeSolid")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn make_solid_from_faces(&mut self, face_handles: Vec<u32>) -> Result<u32, JsError> {
        let face_ids: Vec<brepkit_topology::face::FaceId> = face_handles
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let tolerance = brepkit_math::tolerance::Tolerance::new().linear;
        let solid = brepkit_operations::sew::sew_faces(self.topo_mut(), &face_ids, tolerance)?;
        Ok(solid_id_to_u32(solid))
    }

    /// Remove all holes from a face, returning a new face with only the outer wire.
    #[wasm_bindgen(js_name = "removeHolesFromFace")]
    pub fn remove_holes_from_face(&mut self, face: u32) -> Result<u32, JsError> {
        let face_id = self.resolve_face(face)?;
        let face_data = self.topo.face(face_id)?;
        let outer_wire = face_data.outer_wire();
        let surface = face_data.surface().clone();
        let new_face = Face::new(outer_wire, vec![], surface);
        let fid = self.topo_mut().add_face(new_face);
        Ok(face_id_to_u32(fid))
    }

    /// Weld shells and faces into a single solid by sewing.
    ///
    /// Accepts an array of face handles from potentially different shells.
    /// Sews all faces together into a single solid.
    #[wasm_bindgen(js_name = "weldShellsAndFaces")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn weld_shells_and_faces(
        &mut self,
        face_handles: Vec<u32>,
        tolerance: f64,
    ) -> Result<u32, JsError> {
        let face_ids: Vec<brepkit_topology::face::FaceId> = face_handles
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let tol = if tolerance > 0.0 {
            tolerance
        } else {
            brepkit_math::tolerance::Tolerance::new().linear
        };
        let solid = brepkit_operations::sew::sew_faces(self.topo_mut(), &face_ids, tol)?;
        Ok(solid_id_to_u32(solid))
    }

    // -- Healing ---------------------------------------------------------------

    /// Unify adjacent faces that lie on the same geometric surface.
    ///
    /// Merges co-surface face fragments (produced by boolean operations)
    /// back into single faces, reducing face count and improving topology.
    /// Returns the number of faces removed.
    #[wasm_bindgen(js_name = "unifyFaces")]
    pub fn unify_faces(&mut self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let removed = brepkit_operations::heal::unify_faces(self.topo_mut(), solid_id)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(removed as u32)
    }

    /// Convert all analytic geometry in a solid to NURBS representation.
    ///
    /// Replaces planes, cylinders, cones, spheres, tori with NURBS surfaces and
    /// lines, circles, ellipses with NURBS curves. NURBS surfaces and curves
    /// already in the model are left untouched. Returns the number of entities
    /// converted.
    ///
    /// Converts every analytic surface and curve to a NURBS representation.
    /// Stored pcurves are dropped during conversion — callers that depend on
    /// pcurves should recompute them afterwards.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or conversion fails.
    #[wasm_bindgen(js_name = "convertToBspline")]
    pub fn convert_to_bspline(&mut self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let count = brepkit_operations::heal::convert_to_bspline(self.topo_mut(), solid_id)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(count as u32)
    }

    /// Recognize and replace NURBS faces and edges with their analytic
    /// (elementary) forms wherever possible (Plane/Cylinder/Sphere/
    /// Cone/Torus surfaces; Line/Circle/Ellipse edges).
    ///
    /// Inverse of `convertToBspline`: useful after STEP/IGES import
    /// to recover analytic types from B-spline-only exports.
    /// Returns the total number of faces and edges converted.
    ///
    /// # Errors
    ///
    /// Returns an error if topology lookups fail.
    #[wasm_bindgen(js_name = "convertToElementary")]
    pub fn convert_to_elementary(&mut self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let count =
            brepkit_operations::heal::convert_to_elementary(self.topo_mut(), solid_id, TOL)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(count as u32)
    }

    /// Heal a solid topology.
    ///
    /// Returns the number of issues fixed.
    #[wasm_bindgen(js_name = "healSolid")]
    pub fn heal_solid(&mut self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let report = brepkit_operations::heal::heal_solid(self.topo_mut(), solid_id, TOL)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok((report.vertices_merged
            + report.degenerate_edges_removed
            + report.orientations_fixed
            + report.wire_gaps_closed
            + report.small_faces_removed
            + report.duplicate_faces_removed) as u32)
    }

    /// Validate, heal, and re-validate a solid in one pass.
    ///
    /// Returns the number of remaining validation errors after repair.
    /// A return value of 0 means the solid is valid after repair.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    #[wasm_bindgen(js_name = "repairSolid")]
    pub fn repair_solid(&mut self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let report = brepkit_operations::heal::repair_solid(self.topo_mut(), solid_id, TOL)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(report.after.error_count() as u32)
    }

    /// Remove degenerate (zero-length) edges from a solid.
    ///
    /// Returns the number of edges removed.
    #[wasm_bindgen(js_name = "removeDegenerateEdges")]
    pub fn remove_degenerate_edges(&mut self, solid: u32, tolerance: f64) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let count = brepkit_operations::heal::remove_degenerate_edges(
            self.topo_mut(),
            solid_id,
            tolerance,
        )?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(count as u32)
    }

    /// Fix face orientations to ensure consistent outward normals.
    ///
    /// Returns the number of faces fixed.
    #[wasm_bindgen(js_name = "fixFaceOrientations")]
    pub fn fix_face_orientations(&mut self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let count = brepkit_operations::heal::fix_face_orientations(self.topo_mut(), solid_id)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(count as u32)
    }

    // -- Defeaturing & Feature Recognition -------------------------------------

    /// Remove specified faces from a solid (defeaturing).
    ///
    /// `face_handles` is an array of face handles to remove.
    /// Returns a new solid handle.
    #[wasm_bindgen(js_name = "defeature")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn defeature(&mut self, solid: u32, face_handles: Vec<u32>) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let face_ids: Vec<_> = face_handles
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<Vec<_>, _>>()?;
        let result =
            brepkit_operations::defeature::defeature(self.topo_mut(), solid_id, &face_ids)?;
        Ok(solid_id_to_u32(result))
    }

    /// Detect small features (faces below an area threshold).
    ///
    /// Returns an array of face handles.
    #[wasm_bindgen(js_name = "detectSmallFeatures")]
    pub fn detect_small_features(
        &self,
        solid: u32,
        area_threshold: f64,
        deflection: f64,
    ) -> Result<Vec<u32>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let faces = brepkit_operations::defeature::detect_small_features(
            &self.topo,
            solid_id,
            area_threshold,
            deflection,
        )?;
        Ok(faces.iter().map(|f| face_id_to_u32(*f)).collect())
    }

    /// Recognize geometric features in a solid.
    ///
    /// Returns a JSON string describing the recognized features.
    #[wasm_bindgen(js_name = "recognizeFeatures")]
    pub fn recognize_features(&self, solid: u32, deflection: f64) -> Result<String, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let features = brepkit_operations::feature_recognition::recognize_features(
            &self.topo, solid_id, deflection,
        )?;
        let json_features: Vec<serde_json::Value> =
            features.iter().map(serialize_feature).collect();
        Ok(serde_json::Value::Array(json_features).to_string())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use crate::kernel::BrepKernel;

    #[test]
    fn convert_to_bspline_returns_count_and_solid() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeCylinder", "args": {"radius": 1, "height": 2}},
                {"op": "convertToBspline", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let ok = parsed[1]["ok"].as_object().expect("expected ok object");
        assert!(ok.get("solid").is_some(), "missing 'solid' field");
        let converted = ok["converted"].as_u64().expect("expected 'converted' u64");
        // Cylinder has 3 faces (lateral + 2 caps) and 3 edges (2 circles + 1 seam)
        // → 6 conversions on first run.
        assert!(converted >= 5, "expected >=5 conversions, got {converted}");
    }

    #[test]
    fn convert_to_bspline_invalid_handle_errors() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "convertToBspline", "args": {"solid": 999}}]"#);
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert!(
            parsed[0]["error"].is_string(),
            "expected error for invalid handle, got: {}",
            parsed[0]
        );
    }

    #[test]
    fn convert_to_bspline_idempotent_second_call_is_zero() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "convertToBspline", "args": {"solid": 0}},
                {"op": "convertToBspline", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let first = parsed[1]["ok"]["converted"].as_u64().unwrap();
        let second = parsed[2]["ok"]["converted"].as_u64().unwrap();
        assert!(first > 0);
        assert_eq!(second, 0, "second pass should convert nothing");
    }

    #[test]
    fn convert_to_elementary_via_batch_round_trip() {
        // Round-trip a cylinder through the batch dispatch: NURBS-ify
        // it, then recognize back. The convertToElementary entry was
        // missing from `dispatch_op` before this PR, so prior to the
        // fix this test would have hit the catch-all "unknown
        // operation" arm.
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeCylinder", "args": {"radius": 1, "height": 2}},
                {"op": "convertToBspline", "args": {"solid": 0}},
                {"op": "convertToElementary", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        // convertToElementary must reach the dispatch arm — not the
        // catch-all — so the result is `ok`, not `error`.
        let ok = parsed[2]["ok"]
            .as_object()
            .expect("convertToElementary should be dispatched, got error");
        assert!(ok.get("solid").is_some(), "missing 'solid' field");
        let converted = ok["converted"].as_u64().expect("expected 'converted' u64");
        assert!(
            converted > 0,
            "expected >=1 recognition (the cylinder lateral face at minimum), got {converted}"
        );
    }

    #[test]
    fn convert_to_elementary_via_batch_invalid_handle_errors() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "convertToElementary", "args": {"solid": 999}}]"#);
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert!(
            parsed[0]["error"].is_string(),
            "expected error for invalid handle, got: {}",
            parsed[0]
        );
    }
}

// ── Configurable healing ────────────────────────────────────────────────

/// Parse a `{tolerance?, fixes?}` JSON object into a [`FixConfig`] plus an
/// optional tolerance override.
///
/// `fixes` maps camelCase fix names to `"off" | "auto" | "on"`. Unknown fix
/// names and unknown mode strings are errors — a typo must not silently run
/// with defaults.
fn parse_heal_config(
    json: &str,
) -> Result<(brepkit_heal::fix::config::FixConfig, Option<f64>), crate::error::WasmError> {
    use brepkit_heal::fix::config::{FixConfig, FixMode};

    use crate::error::WasmError;

    let val: serde_json::Value =
        serde_json::from_str(json).map_err(|e| WasmError::InvalidInput {
            reason: format!("invalid heal config JSON: {e}"),
        })?;

    let tolerance = match val.get("tolerance") {
        None | Some(serde_json::Value::Null) => None,
        Some(t) => {
            let t = t.as_f64().ok_or_else(|| WasmError::InvalidInput {
                reason: "heal config 'tolerance' must be a number".into(),
            })?;
            if !(t.is_finite() && t > 0.0) {
                return Err(WasmError::InvalidInput {
                    reason: format!("heal config 'tolerance' must be positive and finite, got {t}"),
                });
            }
            Some(t)
        }
    };

    let mut config = FixConfig::default();
    if let Some(fixes) = val.get("fixes") {
        let map = fixes.as_object().ok_or_else(|| WasmError::InvalidInput {
            reason: "heal config 'fixes' must be an object".into(),
        })?;
        for (name, mode_val) in map {
            let mode = match mode_val.as_str() {
                Some("off") => FixMode::Off,
                Some("auto") => FixMode::Auto,
                Some("on") => FixMode::On,
                _ => {
                    return Err(WasmError::InvalidInput {
                        reason: format!(
                            "fix '{name}' mode must be \"off\", \"auto\", or \"on\", got {mode_val}"
                        ),
                    });
                }
            };
            let slot: &mut FixMode = match name.as_str() {
                "reorder" => &mut config.fix_reorder,
                "connectivity" => &mut config.fix_connectivity,
                "closure" => &mut config.fix_closure,
                "smallEdges" => &mut config.fix_small_edges,
                "selfIntersection" => &mut config.fix_self_intersection,
                "degenerateEdges" => &mut config.fix_degenerate_edges,
                "gaps2d" => &mut config.fix_gaps_2d,
                "gaps3d" => &mut config.fix_gaps_3d,
                "lacking" => &mut config.fix_lacking,
                "notched" => &mut config.fix_notched,
                "tail" => &mut config.fix_tail,
                "intersectingEdges" => &mut config.fix_intersecting_edges,
                "wireOrientation" => &mut config.fix_wire_orientation,
                "addNaturalBound" => &mut config.fix_add_natural_bound,
                "missingSeam" => &mut config.fix_missing_seam,
                "smallArea" => &mut config.fix_small_area,
                "duplicateFaces" => &mut config.fix_duplicate_faces,
                "intersectingWires" => &mut config.fix_intersecting_wires,
                "orientation" => &mut config.fix_orientation,
                "sameParameter" => &mut config.fix_same_parameter,
                "vertexTolerance" => &mut config.fix_vertex_tolerance,
                "pcurve" => &mut config.fix_pcurve,
                "coincidentVertices" => &mut config.fix_coincident_vertices,
                "wireframe" => &mut config.fix_wireframe,
                "splitCommonVertex" => &mut config.fix_split_common_vertex,
                "smallFaces" => &mut config.fix_small_faces,
                other => {
                    return Err(WasmError::InvalidInput {
                        reason: format!("unknown fix name: '{other}'"),
                    });
                }
            };
            *slot = mode;
        }
    }
    Ok((config, tolerance))
}

/// Natively-testable implementations (`JsError` cannot be constructed on
/// non-wasm targets).
impl BrepKernel {
    pub(crate) fn fix_shape_with_config_impl(
        &mut self,
        solid: u32,
        config_json: &str,
    ) -> Result<crate::types::HealFixResult, crate::error::WasmError> {
        let (config, tolerance) = parse_heal_config(config_json)?;
        let solid_id =
            self.resolve_solid(solid)
                .map_err(|_| crate::error::WasmError::InvalidHandle {
                    entity: "solid",
                    index: solid as usize,
                })?;
        let (new_solid, result) = match tolerance {
            Some(t) => {
                brepkit_heal::fix::fix_shape_with_tolerance(self.topo_mut(), solid_id, &config, t)
            }
            None => brepkit_heal::fix::fix_shape(self.topo_mut(), solid_id, &config),
        }
        .map_err(|e| crate::error::WasmError::InvalidInput {
            reason: format!("heal: {e}"),
        })?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(crate::types::HealFixResult {
            solid: solid_id_to_u32(new_solid),
            actions_taken: result.actions_taken as u32,
            done: result.status.is_done(),
            failed: result.status.is_fail(),
        })
    }

    #[allow(clippy::needless_pass_by_value)]
    pub(crate) fn run_heal_pipeline_impl(
        &mut self,
        solid: u32,
        steps: Vec<String>,
    ) -> Result<crate::types::HealPipelineResult, crate::error::WasmError> {
        const MAX_HEAL_PIPELINE_STEPS: usize = 32;
        if steps.is_empty() {
            return Err(crate::error::WasmError::InvalidInput {
                reason: "heal pipeline needs at least one step".into(),
            });
        }
        if steps.len() > MAX_HEAL_PIPELINE_STEPS {
            return Err(crate::error::WasmError::InvalidInput {
                reason: format!("heal pipeline accepts at most {MAX_HEAL_PIPELINE_STEPS} steps"),
            });
        }
        let solid_id =
            self.resolve_solid(solid)
                .map_err(|_| crate::error::WasmError::InvalidHandle {
                    entity: "solid",
                    index: solid as usize,
                })?;
        let mut process = brepkit_heal::pipeline::process::HealProcess::new();
        let valid_steps: std::collections::HashSet<&str> =
            process.registry_mut().names().into_iter().collect();
        if let Some(step) = steps
            .iter()
            .find(|step| !valid_steps.contains(step.as_str()))
        {
            return Err(crate::error::WasmError::InvalidInput {
                reason: format!("heal pipeline: unknown operator: {step}"),
            });
        }
        for step in &steps {
            process.add_step(step);
        }
        let (new_solid, results) = self
            .with_topology_transaction(|topo| process.execute(topo, solid_id))
            .map_err(|e| crate::error::WasmError::InvalidInput {
                reason: format!("heal pipeline: {e}"),
            })?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(crate::types::HealPipelineResult {
            solid: solid_id_to_u32(new_solid),
            steps: steps
                .iter()
                .zip(results.iter())
                .map(|(name, r)| crate::types::HealStepResult {
                    step: name.clone(),
                    actions_taken: r.actions_taken as u32,
                    done: r.status.is_done(),
                    failed: r.status.is_fail(),
                })
                .collect(),
        })
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Heal a solid with a per-fix configuration instead of the fixed
    /// [`healSolid`](Self::heal_solid) recipe.
    ///
    /// `configJson` is `{ "tolerance"?: number, "fixes"?: { <name>: mode } }`
    /// where every mode is `"off"`, `"auto"` (apply when analysis detects the
    /// issue — the default), or `"on"` (always attempt). Unknown fix names
    /// error rather than silently running with defaults. Fix names:
    /// `reorder`, `connectivity`, `closure`, `smallEdges`, `selfIntersection`,
    /// `degenerateEdges`, `gaps2d`, `gaps3d`, `lacking`, `notched`, `tail`,
    /// `intersectingEdges`, `wireOrientation`, `addNaturalBound`,
    /// `missingSeam`, `smallArea`, `duplicateFaces`, `intersectingWires`,
    /// `orientation`, `sameParameter`, `vertexTolerance`, `pcurve`,
    /// `coincidentVertices`, `wireframe`, `splitCommonVertex`, `smallFaces`.
    ///
    /// Returns a JSON string `{ solid, actionsTaken, done, failed }` (see the
    /// `HealFixResult` TypeScript type); `solid` is the healed solid's handle
    /// and may differ from the input.
    #[wasm_bindgen(js_name = "fixShapeWithConfig")]
    pub fn fix_shape_with_config(
        &mut self,
        solid: u32,
        config_json: &str,
    ) -> Result<JsValue, JsError> {
        let result = self.fix_shape_with_config_impl(solid, config_json)?;
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Run a custom sequence of healing operators on a solid.
    ///
    /// `steps` names built-in operators, executed in order: `fix_shape`,
    /// `unify_same_domain`, `direct_faces`, `same_parameter`,
    /// `merge_vertices`, `drop_small_edges`, `drop_small_faces`,
    /// `remove_internal_wires`, `sew_shells`, `split_common_vertex`,
    /// `convert_to_bspline`, `convert_to_elementary`, `fix_wireframe`
    /// (see [`heal_pipeline_steps`](Self::heal_pipeline_steps)). An unknown
    /// step name fails the whole run before any mutation of later steps.
    ///
    /// Returns a JSON string `{ solid, steps: [{step, actionsTaken, done,
    /// failed}] }` (see the `HealPipelineResult` TypeScript type).
    #[wasm_bindgen(js_name = "runHealPipeline")]
    pub fn run_heal_pipeline(
        &mut self,
        solid: u32,
        steps: Vec<String>,
    ) -> Result<JsValue, JsError> {
        let result = self.run_heal_pipeline_impl(solid, steps)?;
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Names of the built-in healing pipeline operators accepted by
    /// [`run_heal_pipeline`](Self::run_heal_pipeline).
    #[wasm_bindgen(js_name = "healPipelineSteps")]
    #[must_use]
    #[allow(clippy::unused_self)] // instance method so JS finds it on the kernel
    pub fn heal_pipeline_steps(&self) -> Vec<String> {
        let mut process = brepkit_heal::pipeline::process::HealProcess::new();
        process
            .registry_mut()
            .names()
            .into_iter()
            .map(String::from)
            .collect()
    }
}

#[cfg(test)]
mod heal_config_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use crate::kernel::BrepKernel;

    fn make_box(k: &mut BrepKernel) -> u32 {
        k.make_box_solid(2.0, 2.0, 2.0).unwrap()
    }

    #[test]
    fn fix_with_default_config_keeps_valid_box_intact() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k);
        let r = k
            .fix_shape_with_config_impl(solid, r#"{"tolerance": 1e-6}"#)
            .unwrap();
        assert!(!r.failed, "healing a valid box must not fail: {r:?}");
        // The healed solid still resolves and keeps its volume.
        let vol = k.volume(r.solid, 0.01).unwrap();
        assert!((vol - 8.0).abs() < 0.05, "volume after heal = {vol}");
    }

    #[test]
    fn fix_config_all_off_takes_no_actions() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k);
        let all_off: Vec<String> = [
            "reorder",
            "connectivity",
            "closure",
            "smallEdges",
            "selfIntersection",
            "degenerateEdges",
            "gaps2d",
            "gaps3d",
            "lacking",
            "notched",
            "tail",
            "intersectingEdges",
            "wireOrientation",
            "addNaturalBound",
            "missingSeam",
            "smallArea",
            "duplicateFaces",
            "intersectingWires",
            "orientation",
            "sameParameter",
            "vertexTolerance",
            "pcurve",
            "coincidentVertices",
            "wireframe",
            "splitCommonVertex",
            "smallFaces",
        ]
        .iter()
        .map(|n| format!(r#""{n}": "off""#))
        .collect();
        let json = format!(r#"{{"fixes": {{{}}}}}"#, all_off.join(","));
        let r = k.fix_shape_with_config_impl(solid, &json).unwrap();
        assert_eq!(r.actions_taken, 0, "all-off config must take no actions");
        assert!(!r.done);
    }

    #[test]
    fn unknown_fix_name_and_bad_mode_error() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k);
        assert!(
            k.fix_shape_with_config_impl(solid, r#"{"fixes": {"warpDrive": "on"}}"#)
                .is_err()
        );
        assert!(
            k.fix_shape_with_config_impl(solid, r#"{"fixes": {"reorder": "sometimes"}}"#)
                .is_err()
        );
        assert!(
            k.fix_shape_with_config_impl(solid, r#"{"tolerance": -1.0}"#)
                .is_err()
        );
    }

    #[test]
    fn pipeline_runs_steps_in_order_and_reports_each() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k);
        let r = k
            .run_heal_pipeline_impl(
                solid,
                vec!["merge_vertices".into(), "drop_small_edges".into()],
            )
            .unwrap();
        assert_eq!(r.steps.len(), 2);
        assert_eq!(r.steps[0].step, "merge_vertices");
        assert_eq!(r.steps[1].step, "drop_small_edges");
        let vol = k.volume(r.solid, 0.01).unwrap();
        assert!((vol - 8.0).abs() < 0.05);
    }

    #[test]
    fn pipeline_rejects_unknown_step_and_empty_list() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k);
        assert!(
            k.run_heal_pipeline_impl(solid, vec!["definitely_not_an_op".into()])
                .is_err()
        );
        assert!(k.run_heal_pipeline_impl(solid, vec![]).is_err());
        assert!(
            k.run_heal_pipeline_impl(solid, vec!["merge_vertices".into(); 33])
                .is_err()
        );
    }

    #[test]
    fn pipeline_step_names_are_exposed() {
        let k = BrepKernel::new();
        let names = k.heal_pipeline_steps();
        assert!(names.iter().any(|n| n == "fix_shape"));
        assert!(names.iter().any(|n| n == "unify_same_domain"));
        assert!(names.len() >= 13, "expected ≥13 builtins, got {names:?}");
    }
}
