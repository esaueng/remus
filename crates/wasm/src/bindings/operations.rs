//! Modeling operation bindings (extrude, revolve, sweep, loft, fillet, etc.).

#![allow(
    clippy::missing_errors_doc,
    clippy::too_many_arguments,
    clippy::too_many_lines
)]

use wasm_bindgen::prelude::*;

use brepkit_math::nurbs::curve::NurbsCurve;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::edge::{Edge, EdgeCurve};
use brepkit_topology::face::{Face, FaceSurface};

use crate::error::{WasmError, validate_finite, validate_positive};
use crate::handles::{edge_id_to_u32, face_id_to_u32, solid_id_to_u32, wire_id_to_u32};
use brepkit_geometry::extrema::point_to_nurbs_surface;

use crate::helpers::{
    TOL, classify_to_string, create_apex_face, fillet_failure_js_error, panic_message,
    parse_points, try_chamfer_with_origins, try_fillet_with_origins,
};
use crate::kernel::BrepKernel;
use crate::types::FaceEvolutionPayloadV1;

use brepkit_operations::extrude::extrude;
use brepkit_operations::offset_wire::JoinType;
use brepkit_operations::push_pull::{push_pull_face, resize_cylindrical_face};
use brepkit_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use brepkit_operations::revolve::revolve;
use brepkit_operations::sweep::sweep;

fn wasm_blend_evolution(
    topo: &brepkit_topology::Topology,
    result: brepkit_topology::solid::SolidId,
    origins: Option<&brepkit_operations::blend_ops::BlendFaceOrigins>,
) -> Result<brepkit_operations::evolution::EvolutionMap, brepkit_operations::OperationsError> {
    let Some(origins) = origins else {
        // The stable WASM contract never runs the legacy normal/centroid
        // matcher. An engine without construction history is an explicit
        // refusal, which the payload expands to complete unresolved domains.
        return Ok(brepkit_operations::evolution::EvolutionMap::new());
    };
    brepkit_operations::blend_ops::evolution_from_blend_origins(topo, result, Some(origins), &[])
}

/// Parse a join type string into a [`JoinType`] enum value.
///
/// Used by both the direct WASM binding and the batch dispatcher.
pub fn parse_join_type_str(s: &str) -> Result<JoinType, WasmError> {
    match s {
        "intersection" => Ok(JoinType::Intersection),
        "arc" => Ok(JoinType::Arc),
        "chamfer" => Ok(JoinType::Chamfer),
        _ => Err(WasmError::InvalidInput {
            reason: format!(
                "unknown join type '{s}', expected 'intersection', 'arc', or 'chamfer'"
            ),
        }),
    }
}

/// Shared implementation for the direct and batch `loftWithOptions` entry
/// points.
pub(super) fn loft_with_options_impl(
    topo: &mut brepkit_topology::Topology,
    mut face_ids: Vec<brepkit_topology::face::FaceId>,
    options: &serde_json::Value,
) -> Result<brepkit_topology::solid::SolidId, WasmError> {
    if let Some(sp) = options.get("startPoint").and_then(|value| value.as_array())
        && sp.len() >= 3
    {
        let point = Point3::new(
            sp[0].as_f64().unwrap_or(0.0),
            sp[1].as_f64().unwrap_or(0.0),
            sp[2].as_f64().unwrap_or(0.0),
        );
        let apex_face = create_apex_face(topo, point, &face_ids)?;
        face_ids.insert(0, apex_face);
    }

    if let Some(ep) = options.get("endPoint").and_then(|value| value.as_array())
        && ep.len() >= 3
    {
        let point = Point3::new(
            ep[0].as_f64().unwrap_or(0.0),
            ep[1].as_f64().unwrap_or(0.0),
            ep[2].as_f64().unwrap_or(0.0),
        );
        let apex_face = create_apex_face(topo, point, &face_ids)?;
        face_ids.push(apex_face);
    }

    let ruled = options
        .get("ruled")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(true);
    if ruled {
        Ok(brepkit_operations::loft::loft(topo, &face_ids)?)
    } else {
        Ok(brepkit_operations::loft::loft_smooth(topo, &face_ids)?)
    }
}

/// Parse the options shared by direct and batch `sweepWithOptions` calls.
pub(super) fn parse_sweep_options(
    contact_mode: &str,
    scale_values: Vec<f64>,
    segments: u32,
    corner_mode: &str,
) -> Result<brepkit_operations::sweep::SweepOptions, String> {
    use brepkit_operations::sweep::{SweepContactMode, SweepCornerMode, SweepOptions};

    let contact_mode = if contact_mode == "fixed" {
        SweepContactMode::Fixed
    } else if let Some(rest) = contact_mode.strip_prefix("constantNormal:") {
        let parts: Vec<f64> = rest
            .split(',')
            .filter_map(|part| part.trim().parse().ok())
            .collect();
        if parts.len() >= 3 {
            SweepContactMode::ConstantNormal(Vec3::new(parts[0], parts[1], parts[2]))
        } else {
            SweepContactMode::RotationMinimizing
        }
    } else {
        SweepContactMode::RotationMinimizing
    };

    let scale_law: Option<Box<dyn Fn(f64) -> f64 + Send + Sync>> =
        if scale_values.len() >= 4 && scale_values.len().is_multiple_of(2) {
            let pairs: Vec<(f64, f64)> = scale_values
                .chunks_exact(2)
                .map(|pair| (pair[0], pair[1]))
                .collect();
            Some(Box::new(move |t: f64| -> f64 {
                if t <= pairs[0].0 {
                    return pairs[0].1;
                }
                if t >= pairs[pairs.len() - 1].0 {
                    return pairs[pairs.len() - 1].1;
                }
                for window in pairs.windows(2) {
                    if t >= window[0].0 && t <= window[1].0 {
                        let fraction = (t - window[0].0) / (window[1].0 - window[0].0);
                        return window[0].1 + fraction * (window[1].1 - window[0].1);
                    }
                }
                1.0
            }))
        } else {
            None
        };

    let corner_mode = if corner_mode == "miter" {
        SweepCornerMode::Miter
    } else if let Some(rest) = corner_mode.strip_prefix("round:") {
        let radius = rest.trim().parse::<f64>().map_err(|_| {
            format!("corner mode \"{corner_mode}\": expected a corner radius, as in \"round:2.5\"")
        })?;
        SweepCornerMode::Round { radius }
    } else if corner_mode == "round" {
        return Err("corner mode \"round\" needs a corner radius, as in \"round:2.5\"".into());
    } else {
        SweepCornerMode::Smooth
    };

    Ok(SweepOptions {
        contact_mode,
        corner_mode,
        scale_law,
        segments: segments as usize,
        aux_spine: None,
    })
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Section ───────────────────────────────────────────────────

    /// Section a solid with a plane, returning cross-section face handles.
    ///
    /// Returns an array of face handles (`u32[]`).
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or the plane doesn't
    /// intersect the solid.
    #[wasm_bindgen(js_name = "section")]
    #[allow(clippy::too_many_arguments)]
    pub fn section_solid(
        &mut self,
        solid: u32,
        px: f64,
        py: f64,
        pz: f64,
        nx: f64,
        ny: f64,
        nz: f64,
    ) -> Result<Vec<u32>, JsError> {
        validate_finite(px, "px")?;
        validate_finite(py, "py")?;
        validate_finite(pz, "pz")?;
        validate_finite(nx, "nx")?;
        validate_finite(ny, "ny")?;
        validate_finite(nz, "nz")?;
        let solid_id = self.resolve_solid(solid)?;
        let result = brepkit_operations::section::section(
            self.topo_mut(),
            solid_id,
            Point3::new(px, py, pz),
            Vec3::new(nx, ny, nz),
        )?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(result.faces.iter().map(|f| f.index() as u32).collect())
    }

    // ── Loft ──────────────────────────────────────────────────────

    /// Loft two or more profile faces into a solid.
    ///
    /// Takes an array of face handles. Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if fewer than 2 faces or profiles have
    /// different vertex counts.
    #[wasm_bindgen(js_name = "loft")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn loft_faces(&mut self, faces: Vec<u32>) -> Result<u32, JsError> {
        let face_ids: Vec<brepkit_topology::face::FaceId> = faces
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let solid_id =
            self.with_topology_transaction(|topo| brepkit_operations::loft::loft(topo, &face_ids))?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Loft profiles with smooth NURBS interpolation.
    ///
    /// Like `loft()`, but produces smooth NURBS side surfaces for 3+
    /// profiles instead of piecewise-planar quads. The surfaces
    /// interpolate through all intermediate profiles with C1+ continuity.
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if fewer than 2 profiles are given, profiles have
    /// different vertex counts, or surface fitting fails.
    #[wasm_bindgen(js_name = "loftSmooth")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn loft_smooth_faces(&mut self, faces: Vec<u32>) -> Result<u32, JsError> {
        let face_ids: Vec<brepkit_topology::face::FaceId> = faces
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let solid_id = self.with_topology_transaction(|topo| {
            brepkit_operations::loft::loft_smooth(topo, &face_ids)
        })?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Loft profiles with options for start/end points and ruled mode.
    ///
    /// `options` is a JSON string with optional fields:
    /// - `startPoint: [x, y, z]` — apex point before first profile
    /// - `endPoint: [x, y, z]` — apex point after last profile
    /// - `ruled: bool` — true for ruled (linear) surfaces (default), false for smooth
    #[wasm_bindgen(js_name = "loftWithOptions")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn loft_with_options(&mut self, faces: Vec<u32>, options: &str) -> Result<u32, JsError> {
        let opts: serde_json::Value =
            serde_json::from_str(options).unwrap_or(serde_json::Value::Null);

        let face_ids: Vec<brepkit_topology::face::FaceId> = faces
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let solid_id = loft_with_options_impl(self.topo_mut(), face_ids, &opts)?;
        Ok(solid_id_to_u32(solid_id))
    }

    // ── Shell ─────────────────────────────────────────────────────

    /// Hollow a solid with uniform wall thickness.
    ///
    /// `open_faces` is an array of face handles to remove (creating openings).
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if thickness is non-positive or the solid is invalid.
    #[wasm_bindgen(js_name = "shell")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn shell_solid(
        &mut self,
        solid: u32,
        thickness: f64,
        open_faces: Vec<u32>,
    ) -> Result<u32, JsError> {
        validate_positive(thickness, "thickness")?;
        let solid_id = self.resolve_solid(solid)?;
        let open_face_ids: Vec<brepkit_topology::face::FaceId> = open_faces
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let result = brepkit_operations::shell_op::shell(
            self.topo_mut(),
            solid_id,
            thickness,
            &open_face_ids,
        )?;
        Ok(solid_id_to_u32(result))
    }

    // ── Chamfer ───────────────────────────────────────────────────

    /// Chamfer edges of a solid.
    ///
    /// `edge_handles` is an array of edge handles. Returns a solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if distance is non-positive or edges are invalid.
    #[wasm_bindgen(js_name = "chamfer")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_solid(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        distance: f64,
    ) -> Result<u32, JsError> {
        validate_positive(distance, "distance")?;
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<brepkit_topology::edge::EdgeId> = edge_handles
            .iter()
            .map(|&h| self.resolve_edge(h))
            .collect::<Result<_, _>>()?;
        let result = crate::helpers::try_chamfer(self.topo_mut(), solid_id, &edge_ids, distance)?;
        Ok(solid_id_to_u32(result))
    }

    /// Chamfer edges and return versioned face-evolution tracking data.
    ///
    /// This runs the same production engine cascade as [`chamfer`](Self::chamfer_solid):
    /// the established planar bevel first, then the walking builder for
    /// supported curved topology. The returned solid is therefore the same
    /// exact B-Rep the non-evolution entry point produces.
    ///
    /// Generated bevel/corner faces name the input faces the builder used to
    /// construct them. If an engine cannot provide construction history, the
    /// payload reports explicit unresolved source/result sets instead of
    /// inferring lineage geometrically.
    ///
    /// # Errors
    ///
    /// Returns an error if a handle is invalid, the distance is non-positive,
    /// or the chamfer fails.
    #[wasm_bindgen(js_name = "chamferWithEvolution")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_with_evolution(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        distance: f64,
    ) -> Result<FaceEvolutionPayloadV1, JsError> {
        validate_positive(distance, "distance")?;
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<brepkit_topology::edge::EdgeId> = edge_handles
            .iter()
            .map(|&handle| self.resolve_edge(handle))
            .collect::<Result<_, _>>()?;
        let source_faces: Vec<u32> = brepkit_topology::explorer::solid_faces(&self.topo, solid_id)?
            .into_iter()
            .map(face_id_to_u32)
            .collect();
        let (result, origins) =
            try_chamfer_with_origins(self.topo_mut(), solid_id, &edge_ids, distance)?;
        let evolution = wasm_blend_evolution(&self.topo, result, origins.as_ref())?;
        let result_faces: Vec<u32> = brepkit_topology::explorer::solid_faces(&self.topo, result)?
            .into_iter()
            .map(face_id_to_u32)
            .collect();
        FaceEvolutionPayloadV1::from_map(
            solid,
            solid_id_to_u32(result),
            source_faces,
            result_faces,
            &evolution,
        )
        .map_err(|error| JsError::new(&error))
    }

    // ── Fillet ────────────────────────────────────────────────────

    /// Fillet (round) edges of a solid.
    ///
    /// `edge_handles` is an array of edge handles. Returns a solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if radius is non-positive or edges are invalid.
    #[wasm_bindgen(js_name = "fillet")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_solid(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
    ) -> Result<u32, JsError> {
        validate_positive(radius, "radius")?;
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<brepkit_topology::edge::EdgeId> = edge_handles
            .iter()
            .map(|&h| self.resolve_edge(h))
            .collect::<Result<_, _>>()?;
        // `fillet_whole_selection` runs the engine chain and enforces the rule
        // that the answer covers every edge named (see its doc comment for what
        // used to happen instead). Wrap in catch_unwind to prevent panics from
        // propagating across the WASM FFI boundary, which would abort the
        // entire WASM instance.
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            crate::helpers::fillet_whole_selection(self.topo_mut(), solid_id, &edge_ids, radius)
        }));
        match result {
            Ok(Ok(solid)) => Ok(solid_id_to_u32(solid)),
            Ok(Err(e)) => Err(fillet_failure_js_error(&e)),
            Err(panic_info) => {
                let msg = panic_message(&panic_info, "Fillet");
                Err(JsError::new(&msg))
            }
        }
    }

    /// Apply a constant-radius fillet and return face-evolution tracking data.
    ///
    /// Returns a validated [`FaceEvolutionPayloadV1`] object. Blend faces
    /// appear under `generated` and surviving faces under `modified`.
    ///
    /// A blend band is listed under **both** faces its rounded edge separated.
    /// It was built between them, so both are its origin; `generated` is an
    /// adjacency record, not an identity, and naming two sources for one new
    /// face is the normal case. A band is never listed under `modified` — it is
    /// not any input face cut back, and a selection stored against one of those
    /// faces must not acquire it.
    ///
    /// The payload exposes construction history only. If an engine cannot
    /// report history, every source/result is explicit under the unresolved
    /// sets with `provenance: "unavailable"`; the binding never infers lineage
    /// from proximity, traversal order, or approximate surface matching.
    ///
    /// # Errors
    ///
    /// Returns an error if a handle is invalid, the radius is non-positive, or
    /// the fillet fails.
    #[wasm_bindgen(js_name = "filletWithEvolution")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_with_evolution(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
    ) -> Result<FaceEvolutionPayloadV1, JsError> {
        validate_positive(radius, "radius")?;
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<brepkit_topology::edge::EdgeId> = edge_handles
            .iter()
            .map(|&h| self.resolve_edge(h))
            .collect::<Result<_, _>>()?;

        // Wrap in catch_unwind like `fillet` does: a fillet panic must not
        // abort the whole WASM instance.
        let source_faces: Vec<u32> = brepkit_topology::explorer::solid_faces(&self.topo, solid_id)?
            .into_iter()
            .map(face_id_to_u32)
            .collect();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(
            || -> Result<FaceEvolutionPayloadV1, JsError> {
                let (result, origins) =
                    try_fillet_with_origins(self.topo_mut(), solid_id, &edge_ids, radius)
                        .map_err(|e| fillet_failure_js_error(&e))?;
                let evo = wasm_blend_evolution(&self.topo, result, origins.as_ref())?;
                let result_faces: Vec<u32> =
                    brepkit_topology::explorer::solid_faces(&self.topo, result)?
                        .into_iter()
                        .map(face_id_to_u32)
                        .collect();
                FaceEvolutionPayloadV1::from_map(
                    solid,
                    solid_id_to_u32(result),
                    source_faces.clone(),
                    result_faces,
                    &evo,
                )
                .map_err(|error| JsError::new(&error))
            },
        ));
        match result {
            Ok(inner) => inner,
            Err(panic_info) => Err(JsError::new(&panic_message(&panic_info, "Fillet"))),
        }
    }

    // ── Operations ─────────────────────────────────────────────────

    /// Move a planar face of a solid along its outward normal.
    ///
    /// A positive `distance` adds material, a negative one removes it.
    /// Returns a new solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if the handles are invalid, the face is not planar or
    /// not part of the solid, or the edit does not produce a valid solid.
    #[wasm_bindgen(js_name = "pushPullFace")]
    pub fn push_pull_face_binding(
        &mut self,
        solid: u32,
        face: u32,
        distance: f64,
    ) -> Result<u32, JsError> {
        validate_finite(distance, "distance")?;
        let solid_id = self.resolve_solid(solid)?;
        let face_id = self.resolve_face(face)?;
        let result = self
            .with_topology_transaction(|topo| push_pull_face(topo, solid_id, face_id, distance))?;
        Ok(solid_id_to_u32(result))
    }

    /// Change the radius of a cylindrical face of a solid.
    ///
    /// Handles both bores and bosses. Returns a new solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if the handles are invalid, the face is not
    /// cylindrical or not part of the solid, `new_radius` is not positive, or
    /// the edit does not produce a valid solid.
    #[wasm_bindgen(js_name = "resizeCylindricalFace")]
    pub fn resize_cylindrical_face_binding(
        &mut self,
        solid: u32,
        face: u32,
        new_radius: f64,
    ) -> Result<u32, JsError> {
        validate_positive(new_radius, "new_radius")?;
        let solid_id = self.resolve_solid(solid)?;
        let face_id = self.resolve_face(face)?;
        let result = self.with_topology_transaction(|topo| {
            resize_cylindrical_face(topo, solid_id, face_id, new_radius)
        })?;
        Ok(solid_id_to_u32(result))
    }

    /// Resize or remove an exact constant-radius analytic blend band.
    ///
    /// `face` is only a seed: the kernel re-derives the complete band, its
    /// supports, and its current radius. `expected_radius` must match that
    /// exact measurement. `new_radius == 0` restores the sharp support
    /// intersection; positive values rebuild the band. Returns a new solid
    /// handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns a stable-code-prefixed refusal if the band is ambiguous,
    /// freeform, stale, unsupported, or cannot be rebuilt exactly. Failure is
    /// transactional and leaves all pre-existing handles valid.
    #[wasm_bindgen(js_name = "resizeBlend")]
    pub fn resize_blend_binding(
        &mut self,
        solid: u32,
        face: u32,
        expected_radius: f64,
        new_radius: f64,
    ) -> Result<u32, JsError> {
        validate_positive(expected_radius, "expected_radius")?;
        validate_finite(new_radius, "new_radius")?;
        if new_radius < 0.0 {
            return Err(JsError::new("new_radius must be non-negative"));
        }
        let solid_id = self.resolve_solid(solid)?;
        let face_id = self.resolve_face(face)?;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            resize_blend(
                self.topo_mut(),
                solid_id,
                face_id,
                expected_radius,
                new_radius,
            )
        }));
        match result {
            Ok(Ok(result)) => Ok(solid_id_to_u32(result.solid)),
            Ok(Err(error)) => Err(JsError::new(&format!(
                "{}: {error}",
                resize_blend_failure_code(&error)
            ))),
            Err(panic_info) => Err(JsError::new(&panic_message(&panic_info, "Resize blend"))),
        }
    }

    /// [`Self::resize_blend_binding`] with versioned face evolution.
    ///
    /// The payload uses the existing [`FaceEvolutionPayloadV1`] schema. New
    /// band faces are `generated` from both recovered support faces; removed
    /// input band faces are `deleted`.
    ///
    /// # Errors
    ///
    /// Returns the same stable-code-prefixed refusals as `resizeBlend`, or a
    /// payload validation error if the construction record is incomplete.
    #[wasm_bindgen(js_name = "resizeBlendWithEvolution")]
    pub fn resize_blend_with_evolution_binding(
        &mut self,
        solid: u32,
        face: u32,
        expected_radius: f64,
        new_radius: f64,
    ) -> Result<FaceEvolutionPayloadV1, JsError> {
        validate_positive(expected_radius, "expected_radius")?;
        validate_finite(new_radius, "new_radius")?;
        if new_radius < 0.0 {
            return Err(JsError::new("new_radius must be non-negative"));
        }
        let solid_id = self.resolve_solid(solid)?;
        let face_id = self.resolve_face(face)?;
        let source_faces: Vec<u32> = brepkit_topology::explorer::solid_faces(&self.topo, solid_id)?
            .into_iter()
            .map(face_id_to_u32)
            .collect();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(
            || -> Result<FaceEvolutionPayloadV1, JsError> {
                let result = resize_blend(
                    self.topo_mut(),
                    solid_id,
                    face_id,
                    expected_radius,
                    new_radius,
                )
                .map_err(|error| {
                    JsError::new(&format!("{}: {error}", resize_blend_failure_code(&error)))
                })?;
                let result_faces: Vec<u32> =
                    brepkit_topology::explorer::solid_faces(&self.topo, result.solid)?
                        .into_iter()
                        .map(face_id_to_u32)
                        .collect();
                FaceEvolutionPayloadV1::from_map(
                    solid,
                    solid_id_to_u32(result.solid),
                    source_faces.clone(),
                    result_faces,
                    &result.evolution,
                )
                .map_err(|error| JsError::new(&error))
            },
        ));
        match result {
            Ok(inner) => inner,
            Err(panic_info) => Err(JsError::new(&panic_message(&panic_info, "Resize blend"))),
        }
    }

    /// Extrude a planar face along a direction vector to create a solid.
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid or the extrusion fails.
    #[wasm_bindgen(js_name = "extrude")]
    pub fn extrude_face(
        &mut self,
        face: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        distance: f64,
    ) -> Result<u32, JsError> {
        validate_finite(dir_x, "dir_x")?;
        validate_finite(dir_y, "dir_y")?;
        validate_finite(dir_z, "dir_z")?;
        validate_finite(distance, "distance")?;

        let face_id = self.resolve_face(face)?;
        let direction = Vec3::new(dir_x, dir_y, dir_z);
        let solid_id = extrude(self.topo_mut(), face_id, direction, distance)?;

        Ok(solid_id_to_u32(solid_id))
    }

    /// Revolve a planar face around an axis to create a solid of revolution.
    ///
    /// The axis is defined by an origin point `(ox, oy, oz)` and a direction
    /// `(dx, dy, dz)`. The angle is in degrees and must be in (0, 360].
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if any input is non-finite, the face handle is
    /// invalid, or the revolve operation fails.
    #[wasm_bindgen(js_name = "revolve")]
    #[allow(clippy::too_many_arguments)]
    pub fn revolve_face(
        &mut self,
        face: u32,
        ox: f64,
        oy: f64,
        oz: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle_degrees: f64,
    ) -> Result<u32, JsError> {
        validate_finite(ox, "ox")?;
        validate_finite(oy, "oy")?;
        validate_finite(oz, "oz")?;
        validate_finite(dx, "dx")?;
        validate_finite(dy, "dy")?;
        validate_finite(dz, "dz")?;
        validate_finite(angle_degrees, "angle_degrees")?;
        if angle_degrees <= 0.0 || angle_degrees > 360.0 {
            return Err(WasmError::InvalidInput {
                reason: format!("angle_degrees must be in (0, 360], got {angle_degrees}"),
            }
            .into());
        }

        let face_id = self.resolve_face(face)?;
        let origin = Point3::new(ox, oy, oz);
        let direction = Vec3::new(dx, dy, dz);
        let angle_radians = angle_degrees.to_radians();

        let solid_id = revolve(self.topo_mut(), face_id, origin, direction, angle_radians)?;

        Ok(solid_id_to_u32(solid_id))
    }

    /// Sweep a planar face along a NURBS curve path to create a solid.
    ///
    /// The path is specified as flat arrays for JS interop:
    /// - `path_degree` — polynomial degree of the path curve
    /// - `path_knots` — knot vector
    /// - `path_control_points` — flat `[x,y,z, ...]` control point coordinates
    /// - `path_weights` — per-control-point weights
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid, the NURBS arrays have
    /// inconsistent lengths, or the sweep operation fails.
    #[wasm_bindgen(js_name = "sweep")]
    #[allow(clippy::needless_pass_by_value)] // wasm-bindgen requires owned Vec
    pub fn sweep_face(
        &mut self,
        face: u32,
        path_degree: u32,
        path_knots: Vec<f64>,
        path_control_points: Vec<f64>,
        path_weights: Vec<f64>,
    ) -> Result<u32, JsError> {
        // Validate coordinate array length.
        if !path_control_points.len().is_multiple_of(3) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "path_control_points length must be a multiple of 3, got {}",
                    path_control_points.len()
                ),
            }
            .into());
        }
        let num_pts = path_control_points.len() / 3;

        if path_weights.len() != num_pts {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "path_weights length ({}) must match number of control points ({num_pts})",
                    path_weights.len()
                ),
            }
            .into());
        }

        // Validate all values are finite.
        if let Some(pos) = path_knots.iter().position(|v| !v.is_finite()) {
            return Err(WasmError::InvalidInput {
                reason: format!("path_knots[{pos}] is not finite"),
            }
            .into());
        }
        if let Some(pos) = path_control_points.iter().position(|v| !v.is_finite()) {
            return Err(WasmError::InvalidInput {
                reason: format!("path_control_points[{pos}] is not finite"),
            }
            .into());
        }
        if let Some(pos) = path_weights.iter().position(|v| !v.is_finite()) {
            return Err(WasmError::InvalidInput {
                reason: format!("path_weights[{pos}] is not finite"),
            }
            .into());
        }

        let face_id = self.resolve_face(face)?;

        let control_points: Vec<Point3> = path_control_points
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();

        let path_curve = NurbsCurve::new(
            path_degree as usize,
            path_knots,
            control_points,
            path_weights,
        )?;

        let solid_id = sweep(self.topo_mut(), face_id, &path_curve)?;

        Ok(solid_id_to_u32(solid_id))
    }

    /// Sweep through multiple section profiles along a spine, lofting the
    /// rotation-minimizing-frame-placed profiles.
    ///
    /// `face_handles` and `params` are parallel arrays: each planar profile and
    /// its parameter in `[0, 1]` along the spine (given as raw NURBS data).
    /// `ruled` selects ruled (planar bands) vs smooth (NURBS) lofted sides.
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error for fewer than two sections, mismatched array lengths, a
    /// non-finite or out-of-range value, a non-planar profile, or loft failure.
    #[wasm_bindgen(js_name = "multiSectionSweep")]
    #[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
    pub fn multi_section_sweep(
        &mut self,
        face_handles: Vec<u32>,
        params: Vec<f64>,
        spine_degree: u32,
        spine_knots: Vec<f64>,
        spine_control_points: Vec<f64>,
        spine_weights: Vec<f64>,
        ruled: bool,
    ) -> Result<u32, JsError> {
        if face_handles.len() != params.len() {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "face_handles ({}) and params ({}) must have equal length",
                    face_handles.len(),
                    params.len()
                ),
            }
            .into());
        }
        if !spine_control_points.len().is_multiple_of(3) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "spine_control_points length must be a multiple of 3, got {}",
                    spine_control_points.len()
                ),
            }
            .into());
        }
        let num_pts = spine_control_points.len() / 3;
        if spine_weights.len() != num_pts {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "spine_weights length ({}) must match control point count ({num_pts})",
                    spine_weights.len()
                ),
            }
            .into());
        }
        for p in &params {
            validate_finite(*p, "param")?;
        }
        for (name, arr) in [
            ("spine_knots", &spine_knots),
            ("spine_control_points", &spine_control_points),
            ("spine_weights", &spine_weights),
        ] {
            if let Some(pos) = arr.iter().position(|v| !v.is_finite()) {
                return Err(WasmError::InvalidInput {
                    reason: format!("{name}[{pos}] is not finite"),
                }
                .into());
            }
        }

        let control_points: Vec<Point3> = spine_control_points
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        let spine = NurbsCurve::new(
            spine_degree as usize,
            spine_knots,
            control_points,
            spine_weights,
        )?;

        let sections: Vec<(brepkit_topology::face::FaceId, f64)> = face_handles
            .iter()
            .zip(params.iter())
            .map(|(&h, &p)| self.resolve_face(h).map(|f| (f, p)))
            .collect::<Result<_, _>>()?;

        let solid_id = brepkit_operations::sweep::multi_section_sweep(
            self.topo_mut(),
            &spine,
            &sections,
            ruled,
        )?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Sweep a face along a path with smooth NURBS side surfaces.
    ///
    /// Like `sweep()`, but produces a single NURBS surface per edge strip
    /// instead of multiple flat quads, giving smooth geometry that
    /// tessellates to arbitrary quality.
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if the face or path is invalid, or surface fitting fails.
    #[wasm_bindgen(js_name = "sweepSmooth")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn sweep_smooth_face(
        &mut self,
        face: u32,
        path_degree: u32,
        path_knots: Vec<f64>,
        path_control_points: Vec<f64>,
        path_weights: Vec<f64>,
    ) -> Result<u32, JsError> {
        if !path_control_points.len().is_multiple_of(3) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "path_control_points length must be a multiple of 3, got {}",
                    path_control_points.len()
                ),
            }
            .into());
        }

        let face_id = self.resolve_face(face)?;
        let n_cp = path_control_points.len() / 3;
        let control_points: Vec<Point3> = (0..n_cp)
            .map(|i| {
                Point3::new(
                    path_control_points[i * 3],
                    path_control_points[i * 3 + 1],
                    path_control_points[i * 3 + 2],
                )
            })
            .collect();

        let weights = if path_weights.is_empty() {
            vec![1.0; n_cp]
        } else {
            path_weights
        };

        #[allow(clippy::cast_possible_truncation)]
        let path_curve = brepkit_math::nurbs::curve::NurbsCurve::new(
            path_degree as usize,
            path_knots,
            control_points,
            weights,
        )?;

        let solid_id = self.with_topology_transaction(|topo| {
            brepkit_operations::sweep::sweep_smooth(topo, face_id, &path_curve)
        })?;
        Ok(solid_id_to_u32(solid_id))
    }

    // ── Offset Face ──────────────────────────────────────────────

    /// Offset a face by a distance along its surface normal.
    ///
    /// Returns the new offset face handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid or the operation fails.
    #[wasm_bindgen(js_name = "offsetFace")]
    pub fn offset_face(&mut self, face: u32, distance: f64, samples: u32) -> Result<u32, JsError> {
        validate_finite(distance, "distance")?;
        let face_id = self.resolve_face(face)?;
        let result = brepkit_operations::offset_face::offset_face(
            self.topo_mut(),
            face_id,
            distance,
            samples as usize,
        )?;
        Ok(face_id_to_u32(result))
    }

    // ── Helical Sweep ───────────────────────────────────────────

    /// Create a helical sweep of a profile face.
    ///
    /// Sweeps the profile along a helix defined by axis, radius, pitch,
    /// and number of turns. Used for generating thread geometry.
    ///
    /// # Errors
    ///
    /// Returns an error if parameters are invalid or the sweep fails.
    #[wasm_bindgen(js_name = "helicalSweep")]
    #[allow(clippy::too_many_arguments)]
    pub fn helical_sweep(
        &mut self,
        profile: u32,
        axis_origin_x: f64,
        axis_origin_y: f64,
        axis_origin_z: f64,
        axis_dir_x: f64,
        axis_dir_y: f64,
        axis_dir_z: f64,
        radius: f64,
        pitch: f64,
        turns: f64,
    ) -> Result<u32, JsError> {
        validate_positive(radius, "radius")?;
        validate_positive(pitch, "pitch")?;
        let face_id = self.resolve_face(profile)?;

        let origin = brepkit_math::vec::Point3::new(axis_origin_x, axis_origin_y, axis_origin_z);
        let axis_dir = brepkit_math::vec::Vec3::new(axis_dir_x, axis_dir_y, axis_dir_z);

        let solid_id = brepkit_operations::helix::helical_sweep(
            self.topo_mut(),
            face_id,
            origin,
            axis_dir,
            radius,
            pitch,
            turns,
            8,
        )?;
        Ok(solid_id_to_u32(solid_id))
    }

    // ── Split ─────────────────────────────────────────────────────

    /// Split a solid into two halves along a plane.
    ///
    /// Returns `[positive_solid_handle, negative_solid_handle]`.
    ///
    /// # Errors
    ///
    /// Returns an error if the plane doesn't intersect the solid.
    #[wasm_bindgen(js_name = "split")]
    #[allow(clippy::too_many_arguments)]
    pub fn split_solid(
        &mut self,
        solid: u32,
        px: f64,
        py: f64,
        pz: f64,
        nx: f64,
        ny: f64,
        nz: f64,
    ) -> Result<Vec<u32>, JsError> {
        validate_finite(px, "px")?;
        validate_finite(py, "py")?;
        validate_finite(pz, "pz")?;
        validate_finite(nx, "nx")?;
        validate_finite(ny, "ny")?;
        validate_finite(nz, "nz")?;
        let solid_id = self.resolve_solid(solid)?;
        let result = brepkit_operations::split::split(
            self.topo_mut(),
            solid_id,
            Point3::new(px, py, pz),
            Vec3::new(nx, ny, nz),
        )?;
        Ok(vec![
            solid_id_to_u32(result.positive),
            solid_id_to_u32(result.negative),
        ])
    }

    // ── Draft ─────────────────────────────────────────────────────

    /// Apply draft angle to faces of a solid.
    ///
    /// `face_handles` is an array of face handles to draft.
    /// Returns a solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if angle is zero or faces are invalid.
    #[wasm_bindgen(js_name = "draft")]
    #[allow(clippy::too_many_arguments, clippy::needless_pass_by_value)]
    pub fn draft_solid(
        &mut self,
        solid: u32,
        face_handles: Vec<u32>,
        pull_x: f64,
        pull_y: f64,
        pull_z: f64,
        neutral_x: f64,
        neutral_y: f64,
        neutral_z: f64,
        angle_degrees: f64,
    ) -> Result<u32, JsError> {
        validate_finite(angle_degrees, "angle_degrees")?;
        for (value, name) in [
            (pull_x, "pull_x"),
            (pull_y, "pull_y"),
            (pull_z, "pull_z"),
            (neutral_x, "neutral_x"),
            (neutral_y, "neutral_y"),
            (neutral_z, "neutral_z"),
        ] {
            validate_finite(value, name)?;
        }
        let solid_id = self.resolve_solid(solid)?;
        let face_ids: Vec<brepkit_topology::face::FaceId> = face_handles
            .iter()
            .map(|&h| self.resolve_face(h))
            .collect::<Result<_, _>>()?;
        let result = brepkit_operations::draft::draft(
            self.topo_mut(),
            solid_id,
            &face_ids,
            Vec3::new(pull_x, pull_y, pull_z),
            Point3::new(neutral_x, neutral_y, neutral_z),
            angle_degrees.to_radians(),
        )?;
        Ok(solid_id_to_u32(result))
    }

    // ── Pipe ──────────────────────────────────────────────────────

    /// Pipe sweep: sweep a profile along a NURBS path (no guide).
    ///
    /// Returns a solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the face or path is invalid.
    #[wasm_bindgen(js_name = "pipe")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn pipe_solid(
        &mut self,
        face: u32,
        path_degree: u32,
        path_knots: Vec<f64>,
        path_control_points: Vec<f64>,
        path_weights: Vec<f64>,
    ) -> Result<u32, JsError> {
        if !path_control_points.len().is_multiple_of(3) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "path_control_points length must be a multiple of 3, got {}",
                    path_control_points.len()
                ),
            }
            .into());
        }

        let face_id = self.resolve_face(face)?;
        let control_points: Vec<Point3> = path_control_points
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();

        let path_curve = NurbsCurve::new(
            path_degree as usize,
            path_knots,
            control_points,
            path_weights,
        )?;

        let solid_id = self.with_topology_transaction(|topo| {
            brepkit_operations::pipe::pipe(topo, face_id, &path_curve, None)
        })?;
        Ok(solid_id_to_u32(solid_id))
    }

    // ── Sweep Along Edges ─────────────────────────────────────────

    /// Sweep a face along a path defined by a chain of edges.
    ///
    /// Collects points from the edges, fits an interpolating NURBS curve,
    /// then sweeps the profile along that curve.
    ///
    /// Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if fewer than 2 edges or the fit fails.
    #[wasm_bindgen(js_name = "sweepAlongEdges")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn sweep_along_edges(&mut self, face: u32, edge_handles: Vec<u32>) -> Result<u32, JsError> {
        if edge_handles.is_empty() {
            return Err(WasmError::InvalidInput {
                reason: "sweepAlongEdges requires at least one edge".into(),
            }
            .into());
        }

        // Collect ordered points from the edge chain.
        let mut points = Vec::new();
        for &eh in &edge_handles {
            let eid = self.resolve_edge(eh)?;
            let edge_data = self.topo.edge(eid)?;
            let start = self.topo.vertex(edge_data.start())?.point();

            // Only push start if it's not a duplicate of the last point.
            if points
                .last()
                .is_none_or(|p: &Point3| (*p - start).length() > TOL)
            {
                points.push(start);
            }

            // For non-line edges, sample interior points for better fidelity.
            match edge_data.curve() {
                EdgeCurve::NurbsCurve(curve) => {
                    let (u0, u1) = curve.domain();
                    let n_samples = 4;
                    for i in 1..n_samples {
                        #[allow(clippy::cast_precision_loss)]
                        let frac = i as f64 / n_samples as f64;
                        let u = u0 + frac * (u1 - u0);
                        points.push(curve.evaluate(u));
                    }
                }
                EdgeCurve::Circle(circle) => {
                    let t_start = circle.project(start);
                    let end_pt = self.topo.vertex(edge_data.end())?.point();
                    let mut t_end = circle.project(end_pt);
                    // Ensure forward traversal (handle wrap-around)
                    if t_end <= t_start {
                        t_end += std::f64::consts::TAU;
                    }
                    let n_samples = 8;
                    for i in 1..n_samples {
                        #[allow(clippy::cast_precision_loss)]
                        let t = t_start + (t_end - t_start) * (i as f64) / (n_samples as f64);
                        points.push(circle.evaluate(t));
                    }
                }
                EdgeCurve::Ellipse(ellipse) => {
                    let t_start = ellipse.project(start);
                    let end_pt = self.topo.vertex(edge_data.end())?.point();
                    let mut t_end = ellipse.project(end_pt);
                    if t_end <= t_start {
                        t_end += std::f64::consts::TAU;
                    }
                    let n_samples = 8;
                    for i in 1..n_samples {
                        #[allow(clippy::cast_precision_loss)]
                        let t = t_start + (t_end - t_start) * (i as f64) / (n_samples as f64);
                        points.push(ellipse.evaluate(t));
                    }
                }
                // Unbounded branches: `project` inverts the parameterization
                // exactly, so the sampled span is the arc between the edge's
                // two vertices — no periodic wrap correction.
                EdgeCurve::Hyperbola(h) => {
                    let end_pt = self.topo.vertex(edge_data.end())?.point();
                    let (t_start, t_end) = (h.project(start), h.project(end_pt));
                    let n_samples = 8;
                    for i in 1..n_samples {
                        #[allow(clippy::cast_precision_loss)]
                        let t = t_start + (t_end - t_start) * (i as f64) / (n_samples as f64);
                        points.push(h.evaluate(t));
                    }
                }
                EdgeCurve::Parabola(pb) => {
                    let end_pt = self.topo.vertex(edge_data.end())?.point();
                    let (t_start, t_end) = (pb.project(start), pb.project(end_pt));
                    let n_samples = 8;
                    for i in 1..n_samples {
                        #[allow(clippy::cast_precision_loss)]
                        let t = t_start + (t_end - t_start) * (i as f64) / (n_samples as f64);
                        points.push(pb.evaluate(t));
                    }
                }
                EdgeCurve::Line => {}
            }

            let end = self.topo.vertex(edge_data.end())?.point();
            points.push(end);
        }

        if points.len() < 2 {
            return Err(WasmError::InvalidInput {
                reason: "sweepAlongEdges: need at least 2 distinct points".into(),
            }
            .into());
        }

        // Densify long, sparsely-sampled spans (e.g. long straight spine edges
        // sampled only at endpoints) so the global interpolating fit below does
        // not overshoot at adjacent high-curvature corners (e.g. non-square
        // rounded-rect spines).
        let points = brepkit_operations::sweep::densify_path_points(&points);

        // Fit an interpolating NURBS curve through the points.
        let degree = std::cmp::min(3, points.len() - 1);
        let path_curve = brepkit_math::nurbs::fitting::interpolate(&points, degree)?;

        let face_id = self.resolve_face(face)?;
        let solid_id = self.with_topology_transaction(|topo| sweep(topo, face_id, &path_curve))?;
        Ok(solid_id_to_u32(solid_id))
    }

    // ── Offset Solid ──────────────────────────────────────────────

    /// Offset (shell) a solid by a distance.
    ///
    /// Returns a new solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the distance is zero or the solid is invalid.
    #[wasm_bindgen(js_name = "offsetSolid")]
    pub fn offset_solid(&mut self, solid: u32, distance: f64) -> Result<u32, JsError> {
        validate_finite(distance, "distance")?;
        let solid_id = self.resolve_solid(solid)?;
        let result =
            brepkit_operations::offset_v2::offset_solid_v2(self.topo_mut(), solid_id, distance)?;
        Ok(solid_id_to_u32(result))
    }

    /// Offset all faces of a solid outward or inward (V2 pipeline).
    ///
    /// Uses the new `brepkit-offset` engine with intersection-based joints.
    ///
    /// # Errors
    ///
    /// Returns an error if the distance is not finite or the solid is invalid.
    #[wasm_bindgen(js_name = "offsetSolidV2")]
    pub fn offset_solid_v2(&mut self, solid: u32, distance: f64) -> Result<u32, JsError> {
        validate_finite(distance, "distance")?;
        let sid = self.resolve_solid(solid)?;
        let result =
            brepkit_operations::offset_v2::offset_solid_v2(self.topo_mut(), sid, distance)?;
        Ok(solid_id_to_u32(result))
    }

    /// Thicken a face into a solid by offsetting it by the given distance.
    ///
    /// Creates a solid from a face by extruding it along its normal by
    /// `thickness`. Positive values offset outward, negative inward.
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid or thickness is zero.
    #[wasm_bindgen(js_name = "thicken")]
    pub fn thicken_face(&mut self, face: u32, thickness: f64) -> Result<u32, JsError> {
        validate_finite(thickness, "thickness")?;
        let face_id = self.resolve_face(face)?;
        let result = brepkit_operations::thicken::thicken(self.topo_mut(), face_id, thickness)?;
        Ok(solid_id_to_u32(result))
    }

    // ── Variable Fillet ───────────────────────────────────────────

    /// Apply variable-radius fillets to edges.
    ///
    /// `json` is a JSON string: `[{"edge": u32, "law": "constant"|"linear"|"scurve", "start": f64, "end": f64}]`
    ///
    /// Also accepts brepjs-style fields: `startRadius`/`endRadius` as aliases for `start`/`end`.
    /// When `law` is omitted and `startRadius` != `endRadius`, the law auto-detects as `"linear"`.
    ///
    /// Returns a new solid handle.
    #[wasm_bindgen(js_name = "filletVariable")]
    pub fn fillet_variable(&mut self, solid: u32, json: &str) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let specs: Vec<serde_json::Value> =
            serde_json::from_str(json).map_err(|e| WasmError::InvalidInput {
                reason: format!("invalid JSON: {e}"),
            })?;
        let mut edge_laws = Vec::with_capacity(specs.len());
        for spec in &specs {
            let edge_handle = spec["edge"]
                .as_u64()
                .ok_or_else(|| WasmError::InvalidInput {
                    reason: "missing 'edge' in fillet spec".into(),
                })? as u32;
            let edge_id = self.resolve_edge(edge_handle)?;
            // Accept both brepkit-native ("start"/"end") and brepjs ("startRadius"/"endRadius")
            let start_val = spec["start"]
                .as_f64()
                .or_else(|| spec["startRadius"].as_f64());
            let end_val = spec["end"].as_f64().or_else(|| spec["endRadius"].as_f64());

            // Auto-detect law: if no "law" field but start != end, use "linear"
            let law_str = spec["law"]
                .as_str()
                .unwrap_or_else(|| match (start_val, end_val) {
                    (Some(s), Some(e)) if (s - e).abs() > f64::EPSILON => "linear",
                    _ => "constant",
                });
            let law = match law_str {
                "linear" => {
                    let s = start_val.unwrap_or(1.0);
                    let e = end_val.unwrap_or(1.0);
                    brepkit_operations::fillet::FilletRadiusLaw::Linear { start: s, end: e }
                }
                "scurve" => {
                    let s = start_val.unwrap_or(1.0);
                    let e = end_val.unwrap_or(1.0);
                    brepkit_operations::fillet::FilletRadiusLaw::SCurve { start: s, end: e }
                }
                _ => {
                    let r = spec["radius"].as_f64().or(start_val).unwrap_or(1.0);
                    brepkit_operations::fillet::FilletRadiusLaw::Constant(r)
                }
            };
            edge_laws.push((edge_id, law));
        }
        let result =
            brepkit_operations::fillet::fillet_variable(self.topo_mut(), solid_id, &edge_laws)?;
        Ok(solid_id_to_u32(result))
    }

    /// Sweep a face along a NURBS path with advanced options.
    ///
    /// `contact_mode`: "rmf" (default), "fixed", or "constantNormal:x,y,z"
    /// `scale_values`: flat `[t0,s0,t1,s1,...]` pairs for piecewise-linear scale law.
    /// `corner_mode`: "smooth" (default), "miter", or "round:&lt;radius&gt;"
    ///   (e.g. `"round:2.5"` — rounding a corner needs a radius).
    /// Returns a solid handle.
    #[wasm_bindgen(js_name = "sweepWithOptions")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn sweep_with_options(
        &mut self,
        profile: u32,
        path_edge: u32,
        contact_mode: &str,
        scale_values: Vec<f64>,
        segments: u32,
        corner_mode: &str,
    ) -> Result<u32, JsError> {
        let face_id = self.resolve_face(profile)?;
        let path_curve = self.extract_nurbs_curve(path_edge)?;
        let options = parse_sweep_options(contact_mode, scale_values, segments, corner_mode)
            .map_err(|error| JsError::new(&error))?;

        let result = self.with_topology_transaction(|topo| {
            brepkit_operations::sweep::sweep_with_options(topo, face_id, &path_curve, &options)
        })?;
        Ok(solid_id_to_u32(result))
    }

    /// Guided (two-rail) sweep: sweep `face` along a spine, orienting the
    /// profile so its up-vector tracks an auxiliary spine.
    ///
    /// The spine and auxiliary spine are each passed as raw NURBS data
    /// (`degree`, `knots`, flat `control_points`, `weights`). Returns a solid
    /// handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error for a non-finite or malformed curve, a non-planar
    /// profile, or a degenerate path.
    #[wasm_bindgen(js_name = "guidedSweep")]
    #[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
    pub fn guided_sweep(
        &mut self,
        face: u32,
        spine_degree: u32,
        spine_knots: Vec<f64>,
        spine_control_points: Vec<f64>,
        spine_weights: Vec<f64>,
        aux_degree: u32,
        aux_knots: Vec<f64>,
        aux_control_points: Vec<f64>,
        aux_weights: Vec<f64>,
    ) -> Result<u32, JsError> {
        let build = |degree: u32,
                     knots: Vec<f64>,
                     cps: Vec<f64>,
                     weights: Vec<f64>,
                     label: &str|
         -> Result<NurbsCurve, JsError> {
            if degree < 1 {
                return Err(WasmError::InvalidInput {
                    reason: format!("{label}_degree must be at least 1"),
                }
                .into());
            }
            if !cps.len().is_multiple_of(3) {
                return Err(WasmError::InvalidInput {
                    reason: format!("{label}_control_points length must be a multiple of 3"),
                }
                .into());
            }
            if weights.len() != cps.len() / 3 {
                return Err(WasmError::InvalidInput {
                    reason: format!("{label}_weights length must match control point count"),
                }
                .into());
            }
            for (name, arr) in [
                ("knots", &knots),
                ("control_points", &cps),
                ("weights", &weights),
            ] {
                if let Some(pos) = arr.iter().position(|v| !v.is_finite()) {
                    return Err(WasmError::InvalidInput {
                        reason: format!("{label}_{name}[{pos}] is not finite"),
                    }
                    .into());
                }
            }
            let control_points: Vec<Point3> = cps
                .chunks_exact(3)
                .map(|c| Point3::new(c[0], c[1], c[2]))
                .collect();
            Ok(NurbsCurve::new(
                degree as usize,
                knots,
                control_points,
                weights,
            )?)
        };

        let spine = build(
            spine_degree,
            spine_knots,
            spine_control_points,
            spine_weights,
            "spine",
        )?;
        let aux = build(
            aux_degree,
            aux_knots,
            aux_control_points,
            aux_weights,
            "aux",
        )?;
        let face_id = self.resolve_face(face)?;
        let solid = brepkit_operations::sweep::sweep_guided(self.topo_mut(), face_id, &spine, aux)?;
        Ok(solid_id_to_u32(solid))
    }

    /// Convex Minkowski sum of two solids (`A ⊕ B`).
    ///
    /// Returns the convex hull of all pairwise vertex sums — exact for convex
    /// polytopes (boxes, or a tessellated-sphere rolling tool), a convex
    /// over-approximation otherwise. Returns a solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if either handle is invalid, either solid is empty, or
    /// the summed points are degenerate so no hull can be built.
    #[wasm_bindgen(js_name = "minkowskiSum")]
    pub fn minkowski_sum(&mut self, solid_a: u32, solid_b: u32) -> Result<u32, JsError> {
        let a = self.resolve_solid(solid_a)?;
        let b = self.resolve_solid(solid_b)?;
        let result = brepkit_operations::primitives::make_minkowski_sum(self.topo_mut(), a, b)?;
        Ok(solid_id_to_u32(result))
    }

    /// Project a solid's edges onto a view plane with hidden-line removal.
    ///
    /// Viewed along `dir` (orthographic) through `origin`, with in-plane x-axis
    /// `x_axis`. Returns a JSON string `{"visible": [[x,y,…]], "hidden": [[…]]}`
    /// — flat 2D polylines in view coordinates. `hidden_lines = false` drops the
    /// hidden set. Occlusion is an exact point-in-solid test.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid handle, a non-positive `deflection`, or a
    /// degenerate `dir`/`x_axis`.
    #[wasm_bindgen(js_name = "projectEdges")]
    #[allow(clippy::too_many_arguments)]
    pub fn project_edges(
        &self,
        solid: u32,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        x_axis_x: f64,
        x_axis_y: f64,
        x_axis_z: f64,
        hidden_lines: bool,
        deflection: f64,
    ) -> Result<JsValue, JsError> {
        validate_positive(deflection, "deflection")?;
        for (v, name) in [
            (origin_x, "origin_x"),
            (origin_y, "origin_y"),
            (origin_z, "origin_z"),
            (dir_x, "dir_x"),
            (dir_y, "dir_y"),
            (dir_z, "dir_z"),
            (x_axis_x, "x_axis_x"),
            (x_axis_y, "x_axis_y"),
            (x_axis_z, "x_axis_z"),
        ] {
            validate_finite(v, name)?;
        }
        let solid_id = self.resolve_solid(solid)?;
        let result = brepkit_operations::projection::project_edges(
            &self.topo,
            solid_id,
            Point3::new(origin_x, origin_y, origin_z),
            Vec3::new(dir_x, dir_y, dir_z),
            Vec3::new(x_axis_x, x_axis_y, x_axis_z),
            hidden_lines,
            deflection,
        )?;
        let flatten = |polys: &[Vec<brepkit_math::vec::Point2>]| -> Vec<Vec<f64>> {
            polys
                .iter()
                .map(|poly| poly.iter().flat_map(|p| [p.x(), p.y()]).collect())
                .collect()
        };
        let json = serde_json::json!({
            "visible": flatten(&result.visible),
            "hidden": flatten(&result.hidden),
        });
        Ok(JsValue::from_str(&json.to_string()))
    }

    // ── Point Classification ──────────────────────────────────────

    /// Classify a point relative to a solid using generalized winding numbers.
    ///
    /// Returns "inside", "outside", or "boundary".
    #[wasm_bindgen(js_name = "classifyPointWinding")]
    pub fn classify_point_winding(
        &self,
        solid: u32,
        x: f64,
        y: f64,
        z: f64,
        tolerance: f64,
    ) -> Result<String, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let point = Point3::new(x, y, z);
        let result = brepkit_operations::classify::classify_point_winding(
            &self.topo, solid_id, point, 0.1, tolerance,
        )?;
        Ok(classify_to_string(result))
    }

    /// Classify a point using robust dual-method (winding + ray casting).
    ///
    /// Returns "inside", "outside", or "boundary".
    #[wasm_bindgen(js_name = "classifyPointRobust")]
    pub fn classify_point_robust(
        &self,
        solid: u32,
        x: f64,
        y: f64,
        z: f64,
        tolerance: f64,
    ) -> Result<String, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let point = Point3::new(x, y, z);
        let result = brepkit_operations::classify::classify_point_robust(
            &self.topo, solid_id, point, 0.1, tolerance,
        )?;
        Ok(classify_to_string(result))
    }

    // ── Fill / Untrim / Offset Wire ───────────────────────────────

    /// Fill a 4-sided boundary with a Coons patch surface.
    ///
    /// `boundary_coords` is flat `[x,y,z, ...]` for all 4 curves concatenated.
    /// `curve_lengths` is `[n0, n1, n2, n3]` — number of points per curve.
    /// Returns a face handle.
    #[wasm_bindgen(js_name = "fillCoonsPatch")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fill_coons_patch(
        &mut self,
        boundary_coords: Vec<f64>,
        curve_lengths: Vec<u32>,
    ) -> Result<u32, JsError> {
        if curve_lengths.len() != 4 {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "Coons patch requires exactly 4 boundary curves, got {}",
                    curve_lengths.len()
                ),
            }
            .into());
        }
        let points = parse_points(&boundary_coords)?;
        let mut curves: Vec<Vec<Point3>> = Vec::with_capacity(4);
        let mut offset = 0usize;
        for &len in &curve_lengths {
            let l = len as usize;
            if offset + l > points.len() {
                return Err(WasmError::InvalidInput {
                    reason: "curve_lengths exceed total coordinate count".into(),
                }
                .into());
            }
            curves.push(points[offset..offset + l].to_vec());
            offset += l;
        }
        let face_id = brepkit_operations::fill_face::fill_coons_patch(self.topo_mut(), &curves)?;
        Ok(face_id_to_u32(face_id))
    }

    /// Untrim a NURBS face by fitting a new surface to the trimmed region.
    ///
    /// Returns a new face handle.
    #[wasm_bindgen(js_name = "untrimFace")]
    pub fn untrim_face(
        &mut self,
        face: u32,
        samples_per_curve: u32,
        interior_samples: u32,
    ) -> Result<u32, JsError> {
        let face_id = self.resolve_face(face)?;
        let face_data = self.topo.face(face_id)?;
        let surface = match face_data.surface() {
            FaceSurface::Nurbs(s) => s.clone(),
            _ => {
                return Err(WasmError::InvalidInput {
                    reason: "untrim only works on NURBS faces".into(),
                }
                .into());
            }
        };
        // Build trim curves from wire edges projected to UV space
        let wire_id = face_data.outer_wire();
        let wire = self.topo.wire(wire_id)?;
        let mut trim_curves = Vec::new();
        for oe in wire.edges() {
            let edge = self.topo.edge(oe.edge())?;
            let v_start = self.topo.vertex(edge.start())?;
            let v_end = self.topo.vertex(edge.end())?;
            // Project endpoints to UV
            let proj_start = point_to_nurbs_surface(v_start.point(), &surface);
            let uv_start = brepkit_math::vec::Point2::new(proj_start.u, proj_start.v);
            let proj_end = point_to_nurbs_surface(v_end.point(), &surface);
            let uv_end = brepkit_math::vec::Point2::new(proj_end.u, proj_end.v);
            trim_curves.push(brepkit_operations::untrim::TrimCurve {
                curve: vec![uv_start, uv_end],
            });
        }
        let new_surface = brepkit_operations::untrim::untrim_face(
            &surface,
            &trim_curves,
            samples_per_curve as usize,
            interior_samples as usize,
        )?;
        Ok(face_id_to_u32(self.nurbs_surface_to_face(new_surface)?))
    }

    /// Offset a wire on a planar face.
    ///
    /// Returns a new wire handle.
    #[wasm_bindgen(js_name = "offsetWire")]
    pub fn offset_wire(&mut self, face: u32, distance: f64) -> Result<u32, JsError> {
        let face_id = self.resolve_face(face)?;
        let wire_id =
            brepkit_operations::offset_wire::offset_wire(self.topo_mut(), face_id, distance)?;
        Ok(wire_id_to_u32(wire_id))
    }

    /// Offset a wire on a planar face with a specific join type.
    ///
    /// `join_type` must be one of `"intersection"`, `"arc"`, or `"chamfer"`.
    /// Returns a new wire handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid, the join type string
    /// is unrecognized, or the offset operation fails.
    #[wasm_bindgen(js_name = "offsetWireWithJoinType")]
    pub fn offset_wire_with_join_type(
        &mut self,
        face: u32,
        distance: f64,
        join_type: &str,
    ) -> Result<u32, JsError> {
        let face_id = self.resolve_face(face)?;
        let jt = parse_join_type_str(join_type)?;
        let wire_id = brepkit_operations::offset_wire::offset_wire_with_join(
            self.topo_mut(),
            face_id,
            distance,
            jt,
        )?;
        Ok(wire_id_to_u32(wire_id))
    }

    /// Offset a planar wire directly by a distance with a specific join type.
    ///
    /// Builds a planar face from the wire internally, then offsets it with
    /// the requested corner join. This is the wire-based counterpart to
    /// [`offset_wire_with_join_type`](Self::offset_wire_with_join_type),
    /// which requires a face handle. Consumers that only hold a wire (such
    /// as 2D sketch offsets) can route a join type through this entry point
    /// without first constructing a face.
    ///
    /// `join_type` must be one of `"intersection"`, `"arc"`, or `"chamfer"`.
    /// Returns a new wire handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the wire handle is invalid, the wire is not
    /// planar, the join type string is unrecognized, or the offset
    /// operation fails.
    #[wasm_bindgen(js_name = "offsetWire2DWithJoin")]
    pub fn offset_wire_2d_with_join(
        &mut self,
        wire: u32,
        distance: f64,
        join_type: &str,
    ) -> Result<u32, JsError> {
        let wire_id = self.resolve_wire(wire)?;
        let jt = parse_join_type_str(join_type)?;
        let face_id =
            brepkit_topology::builder::make_planar_face_from_wire(self.topo_mut(), wire_id)?;
        let result = brepkit_operations::offset_wire::offset_wire_with_join(
            self.topo_mut(),
            face_id,
            distance,
            jt,
        )?;
        Ok(wire_id_to_u32(result))
    }

    // ── Orientation ───────────────────────────────────────────────

    /// Get the orientation of a shape.
    ///
    /// Returns `"forward"` for all faces (brepkit faces don't have an
    /// independent orientation flag; the normal direction is canonical).
    #[allow(clippy::unused_self)]
    #[must_use]
    #[wasm_bindgen(js_name = "getShapeOrientation")]
    pub fn get_shape_orientation(&self, _id: u32) -> String {
        // In brepkit, face normals are always canonical (outward-pointing).
        // There is no separate orientation flag.
        "forward".to_string()
    }

    /// Reverse the orientation of a face or edge.
    ///
    /// For faces: creates a new face with negated plane normal.
    /// For edges: creates a new edge with swapped start/end vertices.
    /// Returns the handle of the new reversed shape.
    ///
    /// # Errors
    ///
    /// Returns an error if the handle is neither a valid face nor edge.
    #[wasm_bindgen(js_name = "reverseShape")]
    pub fn reverse_shape(&mut self, id: u32) -> Result<u32, JsError> {
        // Try as face
        if let Ok(face_id) = self.resolve_face(id) {
            let face = self.topo.face(face_id)?;
            let outer_wire = face.outer_wire();
            let inner_wires: Vec<_> = face.inner_wires().to_vec();
            let new_surface = match face.surface() {
                FaceSurface::Plane { normal, d } => FaceSurface::Plane {
                    normal: -*normal,
                    d: -*d,
                },
                other => other.clone(),
            };
            let new_face = Face::new(outer_wire, inner_wires, new_surface);
            let new_fid = self.topo_mut().add_face(new_face);
            return Ok(face_id_to_u32(new_fid));
        }
        // Try as edge
        if let Ok(edge_id) = self.resolve_edge(id) {
            let edge = self.topo.edge(edge_id)?;
            let new_edge = Edge::new(edge.end(), edge.start(), edge.curve().clone());
            let new_eid = self.topo_mut().add_edge(new_edge);
            return Ok(edge_id_to_u32(new_eid));
        }
        Err(WasmError::InvalidInput {
            reason: "reverseShape requires a face or edge handle".into(),
        }
        .into())
    }

    // ── Blend V2 (walking engine) ────────────────────────────────

    /// Fillet edges using the v2 walking-based blend engine.
    ///
    /// Returns a new solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid or edge handles are invalid, or the
    /// blend computation fails.
    #[wasm_bindgen(js_name = "filletV2")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_v2(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
    ) -> Result<u32, JsError> {
        validate_positive(radius, "radius")?;
        // The bound on a selection is the solid's own edge count, enforced once
        // in `blend_ops::fillet_v2` rather than duplicated as a constant here:
        // "fillet every edge" of a real part is ordinary work.
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<_> = edge_handles
            .iter()
            .map(|&h| self.resolve_edge(h))
            .collect::<Result<_, _>>()?;
        let result =
            brepkit_operations::blend_ops::fillet_v2(self.topo_mut(), solid_id, &edge_ids, radius)?;
        Ok(solid_id_to_u32(result.solid))
    }

    /// Chamfer edges with two distances using the v2 blend engine.
    ///
    /// Returns a new solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid or edge handles are invalid, or the
    /// blend computation fails.
    #[wasm_bindgen(js_name = "chamferV2")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_v2(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        d1: f64,
        d2: f64,
    ) -> Result<u32, JsError> {
        validate_positive(d1, "d1")?;
        validate_positive(d2, "d2")?;
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<_> = edge_handles
            .iter()
            .map(|&h| self.resolve_edge(h))
            .collect::<Result<_, _>>()?;
        let result = brepkit_operations::blend_ops::chamfer_v2(
            self.topo_mut(),
            solid_id,
            &edge_ids,
            d1,
            d2,
        )?;
        Ok(solid_id_to_u32(result.solid))
    }

    /// Chamfer edges with distance and angle using the v2 blend engine.
    ///
    /// Returns a new solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid or edge handles are invalid, or the
    /// blend computation fails.
    #[wasm_bindgen(js_name = "chamferDistanceAngle")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn chamfer_distance_angle(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        distance: f64,
        angle: f64,
    ) -> Result<u32, JsError> {
        validate_positive(distance, "distance")?;
        validate_positive(angle, "angle")?;
        if angle >= std::f64::consts::FRAC_PI_2 {
            return Err(JsError::new("angle must be less than π/2"));
        }
        let solid_id = self.resolve_solid(solid)?;
        let edge_ids: Vec<_> = edge_handles
            .iter()
            .map(|&h| self.resolve_edge(h))
            .collect::<Result<_, _>>()?;
        let result = brepkit_operations::blend_ops::chamfer_distance_angle(
            self.topo_mut(),
            solid_id,
            &edge_ids,
            distance,
            angle,
        )?;
        Ok(solid_id_to_u32(result.solid))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use std::collections::HashSet;

    use brepkit_math::vec::Point3;
    use brepkit_topology::builder::make_polygon_wire;

    use crate::handles::{solid_id_to_u32, wire_id_to_u32};
    use crate::helpers::TOL;
    use crate::kernel::BrepKernel;

    fn square_wire(k: &mut BrepKernel) -> u32 {
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 10.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
        ];
        let wid = make_polygon_wire(k.topo_mut(), &pts, TOL).unwrap();
        wire_id_to_u32(wid)
    }

    fn dispatch(k: &mut BrepKernel, op: &str, args: serde_json::Value) -> serde_json::Value {
        let batch = serde_json::json!([{ "op": op, "args": args }]);
        let out = k.execute_batch(&batch.to_string());
        let parsed: Vec<serde_json::Value> = serde_json::from_str(&out).unwrap();
        parsed[0].clone()
    }

    fn batch_solid_handle(result: &serde_json::Value, label: &str) -> u32 {
        result
            .get("ok")
            .and_then(serde_json::Value::as_u64)
            .unwrap_or_else(|| panic!("{label}: expected an ok solid handle, got {result}"))
            as u32
    }

    fn assert_batch_solid_geometry(k: &BrepKernel, handle: u32, label: &str) {
        let solid = k.resolve_solid(handle).unwrap();
        let shell = k
            .topo
            .shell(k.topo.solid(solid).unwrap().outer_shell())
            .unwrap();
        brepkit_topology::validation::validate_shell_closed(shell, &k.topo)
            .unwrap_or_else(|error| panic!("{label}: solid must be closed: {error:?}"));
        brepkit_topology::validation::validate_shell_manifold(shell, &k.topo)
            .unwrap_or_else(|error| panic!("{label}: solid must be manifold: {error:?}"));

        let coarse = brepkit_operations::measure::solid_volume(&k.topo, solid, 0.25).unwrap();
        let fine = brepkit_operations::measure::solid_volume(&k.topo, solid, 0.05).unwrap();
        assert!(fine > 0.0, "{label}: volume must be positive, got {fine}");
        assert!(
            (coarse - fine).abs() / fine < 0.02,
            "{label}: volume must converge under mesh refinement: coarse={coarse}, fine={fine}"
        );
    }

    fn wire_perimeter(k: &BrepKernel, wire_handle: u32) -> f64 {
        let wid = k.resolve_wire(wire_handle).unwrap();
        brepkit_operations::measure::wire_length(&k.topo, wid).unwrap()
    }

    // ── Closed-rim chamfer fixture (shared by both entry points) ──
    //
    // The flange rim from OpenZCAD, reduced: a cylinder whose end cap is
    // bounded by ONE closed circular edge. The v1 flat-bevel engine is
    // planar-only and fails it ("cannot normalize zero vector"); only the v2
    // fallback inside `try_chamfer` can build it. Both the single-call binding
    // and the batch dispatch must therefore reach that fallback.

    const RIM_R: f64 = 45.0;
    const RIM_H: f64 = 10.0;

    /// Cylinder solid handle plus the handles of its two closed rim circles.
    fn rim_cylinder(k: &mut BrepKernel) -> (u32, Vec<u32>) {
        use brepkit_topology::explorer::solid_faces;

        let cyl =
            brepkit_operations::primitives::make_cylinder(k.topo_mut(), RIM_R, RIM_H).unwrap();
        let topo = &k.topo;
        let mut rims = Vec::new();
        for fid in solid_faces(topo, cyl).unwrap() {
            let f = topo.face(fid).unwrap();
            for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
                for oe in topo.wire(wid).unwrap().edges() {
                    let ed = topo.edge(oe.edge()).unwrap();
                    if ed.start() == ed.end()
                        && ed.curve().type_tag() == "circle"
                        && !rims.contains(&oe.edge())
                    {
                        rims.push(oe.edge());
                    }
                }
            }
        }
        assert_eq!(rims.len(), 2, "a cylinder has two closed rim circles");
        (
            solid_id_to_u32(cyl),
            rims.into_iter()
                .map(crate::handles::edge_id_to_u32)
                .collect(),
        )
    }

    /// Material left after a symmetric chamfer of setback `d` on a rim of
    /// radius `RIM_R`, by Pappus: the right triangle (legs `d`, `d`, area
    /// `d²/2`) revolved about the axis at centroid radius `RIM_R − d/3`.
    fn rim_chamfer_volume(d: f64) -> f64 {
        let full = std::f64::consts::PI * RIM_R * RIM_R * RIM_H;
        full - 0.5 * d * d * std::f64::consts::TAU * (RIM_R - d / 3.0)
    }

    fn assert_complete_evolution(payload: &crate::types::FaceEvolutionPayloadV1) {
        let source: HashSet<u32> = payload.source.faces.iter().copied().collect();
        let result: HashSet<u32> = payload.result.faces.iter().copied().collect();
        let accounted_sources: HashSet<u32> = payload
            .evolution
            .modified
            .iter()
            .map(|claim| claim.source)
            .chain(payload.evolution.deleted.iter().copied())
            .chain(payload.evolution.unresolved_sources.iter().copied())
            .collect();
        let accounted_results: HashSet<u32> = payload
            .evolution
            .modified
            .iter()
            .chain(&payload.evolution.generated)
            .flat_map(|claim| claim.results.iter().copied())
            .chain(
                payload
                    .evolution
                    .unresolved_results
                    .iter()
                    .map(|claim| claim.result),
            )
            .collect();
        assert_eq!(accounted_sources, source);
        assert_eq!(accounted_results, result);
    }

    #[test]
    fn fillet_evolution_payload_covers_single_and_multi_edge_boxes() {
        for edge_slots in [&[0_usize][..], &[0_usize, 2][..]] {
            let mut kernel = BrepKernel::new();
            let solid = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
            let edges = kernel.get_solid_edges(solid).unwrap();
            let selected: Vec<u32> = edge_slots.iter().map(|&slot| edges[slot]).collect();
            let payload = kernel.fillet_with_evolution(solid, selected, 1.0).unwrap();

            assert_eq!(payload.schema_version, 1);
            assert_eq!(payload.source.solid, solid);
            assert_eq!(
                payload.evolution.provenance,
                crate::types::EvolutionProvenanceV1::Construction
            );
            assert!(payload.evolution.deleted.is_empty());
            assert!(payload.evolution.unresolved_sources.is_empty());
            assert!(payload.evolution.unresolved_results.is_empty());
            let generated: HashSet<u32> = payload
                .evolution
                .generated
                .iter()
                .flat_map(|claim| claim.results.iter().copied())
                .collect();
            assert_eq!(generated.len(), edge_slots.len());
            assert_complete_evolution(&payload);
        }
    }

    /// The binding must not impose a ceiling of its own on how many edge
    /// handles a selection carries, and repeated handles — how face-adjacency
    /// selections come out of JS — must be collapsed, not refused.
    #[test]
    fn fillet_v2_binding_takes_a_repeated_selection_past_the_old_handle_cap() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let edges = kernel.get_solid_edges(solid).unwrap();
        let selection: Vec<u32> = std::iter::repeat_n(edges[0], 300).collect();

        let filleted = kernel.fillet_v2(solid, selection, 1.0).unwrap();

        let solid_id = kernel.resolve_solid(filleted).unwrap();
        let faces = brepkit_topology::explorer::solid_faces(&kernel.topo, solid_id)
            .unwrap()
            .len();
        let volume =
            brepkit_operations::measure::solid_volume(&kernel.topo, solid_id, 0.01).unwrap();
        assert!(faces > 6, "fillet must add a face, got {faces}");
        assert!(
            volume < 1000.0,
            "convex fillet must remove material, got {volume}"
        );
    }

    #[test]
    fn resize_blend_bindings_preserve_v1_evolution_contract() {
        let mut kernel = BrepKernel::new();
        let sharp = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let edge = kernel.get_solid_edges(sharp).unwrap()[0];
        let fillet = kernel
            .fillet_with_evolution(sharp, vec![edge], 1.0)
            .unwrap();
        let bands: HashSet<u32> = fillet
            .evolution
            .generated
            .iter()
            .flat_map(|claim| claim.results.iter().copied())
            .collect();
        assert_eq!(bands.len(), 1);
        let band = *bands.iter().next().unwrap();

        let resized = kernel
            .resize_blend_with_evolution_binding(fillet.result.solid, band, 1.0, 2.0)
            .unwrap();
        assert_eq!(resized.schema_version, 1);
        assert_eq!(
            resized.evolution.provenance,
            crate::types::EvolutionProvenanceV1::Construction
        );
        assert!(resized.evolution.deleted.contains(&band));
        assert_complete_evolution(&resized);

        let mut direct_kernel = BrepKernel::new();
        let sharp = direct_kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let edge = direct_kernel.get_solid_edges(sharp).unwrap()[0];
        let fillet = direct_kernel
            .fillet_with_evolution(sharp, vec![edge], 1.0)
            .unwrap();
        let band = fillet.evolution.generated[0].results[0];
        let result = direct_kernel
            .resize_blend_binding(fillet.result.solid, band, 1.0, 0.0)
            .unwrap();
        assert_eq!(direct_kernel.get_solid_faces(result).unwrap().len(), 6);
    }

    #[test]
    fn fillet_and_chamfer_evolution_cover_cylinder_rims() {
        let mut fillet_kernel = BrepKernel::new();
        let (fillet_solid, fillet_rims) = rim_cylinder(&mut fillet_kernel);
        let fillet = fillet_kernel
            .fillet_with_evolution(fillet_solid, vec![fillet_rims[0]], 2.0)
            .unwrap();
        assert_eq!(
            fillet.evolution.provenance,
            crate::types::EvolutionProvenanceV1::Construction
        );
        assert!(!fillet.evolution.generated.is_empty());
        assert_complete_evolution(&fillet);

        let mut chamfer_kernel = BrepKernel::new();
        let (chamfer_solid, chamfer_rims) = rim_cylinder(&mut chamfer_kernel);
        let chamfer = chamfer_kernel
            .chamfer_with_evolution(chamfer_solid, vec![chamfer_rims[0]], 2.0)
            .unwrap();
        assert_eq!(
            chamfer.evolution.provenance,
            crate::types::EvolutionProvenanceV1::Construction
        );
        assert!(!chamfer.evolution.generated.is_empty());
        assert_complete_evolution(&chamfer);
    }

    #[test]
    fn chamfer_evolution_payload_covers_box_bevels() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let edge = kernel.get_solid_edges(solid).unwrap()[0];
        let payload = kernel
            .chamfer_with_evolution(solid, vec![edge], 1.0)
            .unwrap();
        assert_eq!(
            payload.evolution.provenance,
            crate::types::EvolutionProvenanceV1::Construction
        );
        let bevels: HashSet<u32> = payload
            .evolution
            .generated
            .iter()
            .flat_map(|claim| claim.results.iter().copied())
            .collect();
        assert_eq!(bevels.len(), 1);
        assert_complete_evolution(&payload);
    }

    /// Which JS entry point ran the chamfer. Both must reach the same engine
    /// chain (`try_chamfer`: v1, roll back, then v2).
    #[derive(Clone, Copy)]
    enum Entry {
        /// The single-call `chamfer` binding.
        Binding,
        /// The `"chamfer"` op inside `executeBatch`.
        Batch,
    }

    impl Entry {
        const fn label(self) -> &'static str {
            match self {
                Self::Binding => "binding",
                Self::Batch => "batch",
            }
        }
    }

    /// Chamfer one edge through `entry`, returning the result solid handle.
    fn chamfer_via(k: &mut BrepKernel, entry: Entry, solid: u32, edge: u32, d: f64) -> u32 {
        match entry {
            Entry::Binding => k.chamfer_solid(solid, vec![edge], d).unwrap(),
            Entry::Batch => {
                let out = dispatch(
                    k,
                    "chamfer",
                    serde_json::json!({ "solid": solid, "edges": [edge], "distance": d }),
                );
                assert!(
                    out.get("error").is_none(),
                    "batch chamfer d={d} errored: {out}"
                );
                out["ok"].as_u64().unwrap() as u32
            }
        }
    }

    /// Assert the chamfered solid is a closed, manifold B-Rep whose mesh is
    /// also watertight, and whose volume matches the analytic ring wedge.
    /// A closed B-Rep can still tessellate open, so both are checked.
    /// Returns the measured volume so callers can compare entry points.
    fn assert_rim_chamfer_ok(k: &BrepKernel, handle: u32, d: f64, label: &str) -> f64 {
        let sid = k.resolve_solid(handle).unwrap();
        let shell = k
            .topo
            .shell(k.topo.solid(sid).unwrap().outer_shell())
            .unwrap();
        brepkit_topology::validation::validate_shell_closed(shell, &k.topo)
            .unwrap_or_else(|e| panic!("{label}: chamfered solid must be closed: {e:?}"));
        brepkit_topology::validation::validate_shell_manifold(shell, &k.topo)
            .unwrap_or_else(|e| panic!("{label}: chamfered solid must be manifold: {e:?}"));

        let mesh = brepkit_operations::tessellate::tessellate_solid_with_tolerance(
            &k.topo, sid, 0.01, 0.1,
        )
        .unwrap();
        let quality = brepkit_operations::tessellate::welded_mesh_quality(&mesh);
        assert!(
            quality.is_watertight(),
            "{label}: tessellation must be watertight ({} boundary, {} non-manifold edges)",
            quality.boundary_edges,
            quality.non_manifold_edges
        );

        let vol = brepkit_operations::measure::solid_volume(&k.topo, sid, 0.02).unwrap();
        let want = rim_chamfer_volume(d);
        assert!(
            (vol - want).abs() / want < 1e-6,
            "{label}: volume {vol} vs Pappus {want}"
        );
        vol
    }

    #[test]
    fn both_entry_points_chamfer_a_closed_rim_identically() {
        // The batch arm used to call `brepkit_operations::chamfer::chamfer`
        // directly, skipping the v2 fallback that `try_chamfer` provides — so
        // this exact geometry errored through `executeBatch` ("cannot normalize
        // zero vector") while succeeding through the single-call binding.
        // Asserting the analytic volume, not merely `Ok`, is what makes this
        // guard the fix; asserting the two agree is the property that broke.
        for d in [0.5_f64, 1.5] {
            for rim_index in 0..2 {
                let mut volumes = Vec::new();
                for entry in [Entry::Binding, Entry::Batch] {
                    let mut k = BrepKernel::new();
                    let (cyl, rims) = rim_cylinder(&mut k);
                    let out = chamfer_via(&mut k, entry, cyl, rims[rim_index], d);
                    let label = format!("{} d={d} rim={rim_index}", entry.label());
                    volumes.push(assert_rim_chamfer_ok(&k, out, d, &label));
                }
                assert!(
                    (volumes[0] - volumes[1]).abs() < 1e-9,
                    "d={d} rim={rim_index}: entry points disagree: \
                     binding {} vs batch {}",
                    volumes[0],
                    volumes[1]
                );
            }
        }
    }

    #[test]
    fn both_entry_points_agree_on_a_planar_box_edge() {
        // The case the v1 flat-bevel engine builds on its own: routing the
        // batch arm through the fallback chain must leave it unchanged.
        // A 1×1 bevel along one 10-long edge removes 0.5 · 1² · 10 = 5.
        let d = 1.0;
        let mut volumes = Vec::new();
        for entry in [Entry::Binding, Entry::Batch] {
            let mut k = BrepKernel::new();
            let cube =
                brepkit_operations::primitives::make_box(k.topo_mut(), 10.0, 10.0, 10.0).unwrap();
            let edge = first_box_edge(&k, cube);
            let out = chamfer_via(&mut k, entry, solid_id_to_u32(cube), edge, d);
            let vol = k.volume(out, 0.05).unwrap();
            assert!(
                (vol - 995.0).abs() < 0.05,
                "{}: chamfered box volume {vol}, expected ~995",
                entry.label()
            );
            volumes.push(vol);
        }
        assert!(
            (volumes[0] - volumes[1]).abs() < 1e-9,
            "entry points disagree: binding {} vs batch {}",
            volumes[0],
            volumes[1]
        );
    }

    fn first_box_edge(k: &BrepKernel, solid: brepkit_topology::solid::SolidId) -> u32 {
        let topo = &k.topo;
        let shell = topo
            .shell(topo.solid(solid).unwrap().outer_shell())
            .unwrap();
        let face = topo.face(shell.faces()[0]).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        crate::handles::edge_id_to_u32(wire.edges()[0].edge())
    }

    #[test]
    fn multi_section_sweep_lofts_circles_along_line() {
        let mut k = BrepKernel::new();
        let big = k.make_circle_face(10.0, 24).unwrap();
        let small = k.make_circle_face(5.0, 24).unwrap();
        // Degree-1 line spine from the origin to (0, 0, 50).
        let solid = k
            .multi_section_sweep(
                vec![big, small],
                vec![0.0, 1.0],
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
                vec![1.0, 1.0],
                true,
            )
            .unwrap();
        let vol = k.volume(solid, 0.5).unwrap();
        assert!(
            vol > 0.0,
            "tapered tube should have positive volume, got {vol}"
        );
        // (Validation paths — <2 sections, length mismatch, out-of-range param —
        // are covered by the operations-layer tests; the error path can't be
        // exercised here because JsError can't be constructed off-wasm.)
    }

    #[test]
    fn multi_section_sweep_batch_accepts_analytic_line_spine() {
        let mut k = BrepKernel::new();
        let big = k.make_circle_face(10.0, 24).unwrap();
        let small = k.make_circle_face(5.0, 24).unwrap();
        let spine = k.make_line_edge(0.0, 0.0, 0.0, 0.0, 0.0, 50.0).unwrap();
        let out = dispatch(
            &mut k,
            "multiSectionSweep",
            serde_json::json!({
                "faces": [big, small],
                "params": [0.0, 1.0],
                "spineEdge": spine,
                "ruled": true,
            }),
        );
        let solid = batch_solid_handle(&out, "multiSectionSweep with Line spine");
        assert_batch_solid_geometry(&k, solid, "multiSectionSweep with Line spine");
    }

    #[test]
    fn guided_sweep_produces_solid() {
        let mut k = BrepKernel::new();
        let profile = k.make_circle_face(2.0, 24).unwrap();
        // Spine: line (0,0,0)→(0,0,20). Aux: parallel guide offset +10 in X.
        let solid = k
            .guided_sweep(
                profile,
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 20.0],
                vec![1.0, 1.0],
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![10.0, 0.0, 0.0, 10.0, 0.0, 20.0],
                vec![1.0, 1.0],
            )
            .unwrap();
        let vol = k.volume(solid, 0.5).unwrap();
        assert!(vol > 0.0, "guided sweep volume, got {vol}");
    }

    #[test]
    fn guided_sweep_batch_accepts_analytic_line_spine_and_guide() {
        let mut k = BrepKernel::new();
        let profile = k.make_circle_face(2.0, 24).unwrap();
        let mk_line =
            |k: &mut BrepKernel, x: f64| k.make_line_edge(x, 0.0, 0.0, x, 0.0, 20.0).unwrap();
        let spine = mk_line(&mut k, 0.0);
        let aux = mk_line(&mut k, 10.0);
        let out = dispatch(
            &mut k,
            "guidedSweep",
            serde_json::json!({ "face": profile, "spineEdge": spine, "auxEdge": aux }),
        );
        let solid = batch_solid_handle(&out, "guidedSweep with Line rails");
        assert_batch_solid_geometry(&k, solid, "guidedSweep with Line rails");
    }

    #[test]
    fn sweep_batch_accepts_analytic_line_path() {
        let mut k = BrepKernel::new();
        let profile = k.make_rectangle(2.0, 3.0).unwrap();
        let path = k.make_line_edge(0.0, 0.0, 0.0, 0.0, 0.0, 12.0).unwrap();
        let out = dispatch(
            &mut k,
            "sweep",
            serde_json::json!({"face": profile, "pathEdge": path}),
        );
        let solid = batch_solid_handle(&out, "sweep with Line path");
        assert_batch_solid_geometry(&k, solid, "sweep with Line path");
    }

    #[test]
    fn pipe_batch_accepts_analytic_line_path() {
        let mut k = BrepKernel::new();
        let profile = k.make_rectangle(2.0, 3.0).unwrap();
        let path = k.make_line_edge(0.0, 0.0, 0.0, 0.0, 0.0, 12.0).unwrap();
        let out = dispatch(
            &mut k,
            "pipe",
            serde_json::json!({"face": profile, "pathEdge": path}),
        );
        let solid = batch_solid_handle(&out, "pipe with Line path");
        assert_batch_solid_geometry(&k, solid, "pipe with Line path");
    }

    #[test]
    fn sweep_with_options_batch_dispatches_all_options() {
        let mut k = BrepKernel::new();
        let profile = k.make_rectangle(2.0, 3.0).unwrap();
        let path = k.make_line_edge(0.0, 0.0, 0.0, 0.0, 0.0, 12.0).unwrap();
        let out = dispatch(
            &mut k,
            "sweepWithOptions",
            serde_json::json!({
                "profile": profile,
                "pathEdge": path,
                "contactMode": "fixed",
                "scaleValues": [0.0, 1.0, 1.0, 0.75],
                "segments": 6,
                "cornerMode": "miter",
            }),
        );
        let solid = batch_solid_handle(&out, "sweepWithOptions");
        assert_batch_solid_geometry(&k, solid, "sweepWithOptions");
    }

    #[test]
    fn loft_with_options_batch_dispatches_options_object() {
        let mut k = BrepKernel::new();
        let lower = k.make_circle_face(4.0, 24).unwrap();
        let upper = k.make_circle_face(2.0, 24).unwrap();
        k.transform_face(
            upper,
            vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0, 0.0, 1.0,
            ],
        )
        .unwrap();
        let out = dispatch(
            &mut k,
            "loftWithOptions",
            serde_json::json!({
                "faces": [lower, upper],
                "options": {"ruled": true},
            }),
        );
        let solid = batch_solid_handle(&out, "loftWithOptions");
        assert_batch_solid_geometry(&k, solid, "loftWithOptions");
    }

    #[test]
    fn helical_sweep_batch_dispatches_parameters() {
        let mut k = BrepKernel::new();
        let profile = k.make_rectangle(1.0, 1.0).unwrap();
        let out = dispatch(
            &mut k,
            "helicalSweep",
            serde_json::json!({
                "profile": profile,
                "axisOriginX": 0.0,
                "axisOriginY": 0.0,
                "axisOriginZ": 0.0,
                "axisDirX": 0.0,
                "axisDirY": 0.0,
                "axisDirZ": 1.0,
                "radius": 3.75,
                "pitch": 12.0,
                "turns": 0.5,
            }),
        );
        let solid = batch_solid_handle(&out, "helicalSweep");
        assert_batch_solid_geometry(&k, solid, "helicalSweep");
    }

    #[test]
    fn validate_solid_batch_returns_error_count() {
        let mut k = BrepKernel::new();
        let solid = brepkit_operations::primitives::make_box(k.topo_mut(), 2.0, 3.0, 4.0).unwrap();
        let out = dispatch(
            &mut k,
            "validateSolid",
            serde_json::json!({"solid": solid_id_to_u32(solid)}),
        );
        assert_eq!(out["ok"], serde_json::json!(0));
    }

    #[test]
    fn boolean_evolution_batch_variants_marshal_solid_and_evolution() {
        for op in [
            "fuseWithEvolution",
            "cutWithEvolution",
            "intersectWithEvolution",
        ] {
            let mut k = BrepKernel::new();
            let a = k.make_box_solid(4.0, 4.0, 4.0).unwrap();
            let b = k
                .copy_and_transform_solid(
                    a,
                    vec![
                        1.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                        1.0,
                    ],
                )
                .unwrap();
            let out = dispatch(&mut k, op, serde_json::json!({"solidA": a, "solidB": b}));
            let ok = out
                .get("ok")
                .unwrap_or_else(|| panic!("{op}: expected ok result, got {out}"));
            let solid = ok["solid"].as_u64().unwrap() as u32;
            assert!(ok["evolution"].is_object(), "{op}: {out}");
            assert_batch_solid_geometry(&k, solid, op);
        }
    }

    #[test]
    fn fillet_2d_batch_returns_rounded_polygon() {
        let mut k = BrepKernel::new();
        let coords = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0];
        let out = dispatch(
            &mut k,
            "fillet2d",
            serde_json::json!({"coords": coords, "radius": 1.0}),
        );
        let rounded = out["ok"].as_array().unwrap();
        assert!(rounded.len() > coords.len());
    }

    #[test]
    fn chamfer_2d_batch_returns_beveled_polygon() {
        let mut k = BrepKernel::new();
        let coords = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0];
        let out = dispatch(
            &mut k,
            "chamfer2d",
            serde_json::json!({"coords": coords, "distance": 1.0}),
        );
        let beveled = out["ok"].as_array().unwrap();
        assert!(beveled.len() > coords.len());
    }

    #[test]
    fn minkowski_sum_binding_box10_box2_is_box12() {
        let mut k = BrepKernel::new();
        let a = brepkit_operations::primitives::make_box(k.topo_mut(), 10.0, 10.0, 10.0).unwrap();
        let b = brepkit_operations::primitives::make_box(k.topo_mut(), 2.0, 2.0, 2.0).unwrap();
        let sum = k
            .minkowski_sum(solid_id_to_u32(a), solid_id_to_u32(b))
            .unwrap();
        let vol = k.volume(sum, 0.1).unwrap();
        assert!(
            (vol - 1728.0).abs() < 0.5,
            "expected ~1728 (12³), got {vol}"
        );
    }

    #[test]
    fn minkowski_sum_batch_dispatch() {
        let mut k = BrepKernel::new();
        let a = brepkit_operations::primitives::make_box(k.topo_mut(), 4.0, 4.0, 4.0).unwrap();
        let b = brepkit_operations::primitives::make_box(k.topo_mut(), 2.0, 2.0, 2.0).unwrap();
        let out = dispatch(
            &mut k,
            "minkowskiSum",
            serde_json::json!({ "solidA": solid_id_to_u32(a), "solidB": solid_id_to_u32(b) }),
        );
        assert!(
            out.get("ok").and_then(serde_json::Value::as_u64).is_some(),
            "expected an ok solid handle, got {out}"
        );
    }

    #[test]
    fn project_edges_batch_dispatch_box_oblique() {
        let mut k = BrepKernel::new();
        let solid =
            brepkit_operations::primitives::make_box(k.topo_mut(), 10.0, 10.0, 10.0).unwrap();
        let out = dispatch(
            &mut k,
            "projectEdges",
            serde_json::json!({
                "solid": solid_id_to_u32(solid),
                "originX": -100.0, "originY": -100.0, "originZ": -100.0,
                "dirX": 1.0, "dirY": 1.0, "dirZ": 1.0,
                "xAxisX": 1.0, "xAxisY": -1.0, "xAxisZ": 0.0,
                "hiddenLines": true, "deflection": 0.1,
            }),
        );
        let ok = out.get("ok").expect("projectEdges batch should return ok");
        let nonempty = |key: &str| {
            ok.get(key)
                .and_then(serde_json::Value::as_array)
                .is_some_and(|a| !a.is_empty())
        };
        assert!(nonempty("visible"), "visible polylines expected, got {out}");
        assert!(nonempty("hidden"), "hidden polylines expected, got {out}");
    }

    #[test]
    fn offset_wire_2d_with_join_routes_arc_distinct_from_chamfer() {
        let mut k = BrepKernel::new();
        let w_int = square_wire(&mut k);
        let w_arc = square_wire(&mut k);
        let w_chamfer = square_wire(&mut k);

        let intersection = dispatch(
            &mut k,
            "offsetWire2DWithJoin",
            serde_json::json!({"wire": w_int, "distance": 2.0, "joinType": "intersection"}),
        );
        let arc = dispatch(
            &mut k,
            "offsetWire2DWithJoin",
            serde_json::json!({"wire": w_arc, "distance": 2.0, "joinType": "arc"}),
        );
        let chamfer = dispatch(
            &mut k,
            "offsetWire2DWithJoin",
            serde_json::json!({"wire": w_chamfer, "distance": 2.0, "joinType": "chamfer"}),
        );

        for (label, entry) in [
            ("intersection", &intersection),
            ("arc", &arc),
            ("chamfer", &chamfer),
        ] {
            assert!(entry.get("error").is_none(), "{label} errored: {entry}");
        }

        let int_wire = intersection["ok"].as_u64().unwrap() as u32;
        let arc_wire = arc["ok"].as_u64().unwrap() as u32;
        let chamfer_wire = chamfer["ok"].as_u64().unwrap() as u32;

        let int_len = wire_perimeter(&k, int_wire);
        let arc_len = wire_perimeter(&k, arc_wire);
        let chamfer_len = wire_perimeter(&k, chamfer_wire);

        assert!(int_len > 0.0 && arc_len > 0.0 && chamfer_len > 0.0);
        // Arc joins round the four convex corners, so the perimeter must
        // differ from the sharp/chamfer join shapes — proving the join type
        // is actually threaded through rather than silently ignored.
        assert!(
            (arc_len - chamfer_len).abs() > 1.0,
            "arc ({arc_len}) should differ from chamfer ({chamfer_len})"
        );
    }

    #[test]
    fn offset_wire_2d_with_join_rejects_unknown_join_type() {
        let mut k = BrepKernel::new();
        let w = square_wire(&mut k);
        let entry = dispatch(
            &mut k,
            "offsetWire2DWithJoin",
            serde_json::json!({"wire": w, "distance": 2.0, "joinType": "bogus"}),
        );
        assert!(
            entry.get("error").is_some(),
            "unknown join type should error: {entry}"
        );
    }
}
