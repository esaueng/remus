//! Entity-evolution surfacing (Issue 12 → JS).
//!
//! Exposes the construction-derived vertex/edge/face history of GFA
//! booleans, one-call journaled blends, offsets, and patterns, and a
//! read-only journal summary. Event encodings are stable JSON; `unresolved`
//! is an honest event, never hidden.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use remus_operations::boolean::{BooleanOp, EdgeEvent, EntityEvolution, VertexEvent};
use remus_operations::journal_ops;
use remus_topology::journal::EntryPayload;

use crate::error::{StructuredWasmError, validate_finite};
use crate::helpers::{get_f64, get_u32, get_u32_array};
use crate::kernel::BrepKernel;

fn index_u32(index: usize) -> u32 {
    u32::try_from(index).unwrap_or(u32::MAX)
}

/// The stable JSON encoding of an Issue 12 entity-evolution payload.
fn entity_evolution_json(evolution: &EntityEvolution) -> serde_json::Value {
    let faces: Vec<serde_json::Value> = evolution
        .faces
        .iter()
        .map(|&(face, source)| {
            serde_json::json!({
                "face": index_u32(face),
                "source": source.map(index_u32),
            })
        })
        .collect();
    let edges: Vec<serde_json::Value> = evolution
        .edges
        .iter()
        .map(|(edge, event)| {
            let edge = index_u32(*edge);
            match event {
                EdgeEvent::Preserved(from) => serde_json::json!({
                    "edge": edge, "event": "preserved", "from": index_u32(*from),
                }),
                EdgeEvent::Modified(from) => serde_json::json!({
                    "edge": edge, "event": "modified", "from": index_u32(*from),
                }),
                EdgeEvent::Generated { face_a, face_b } => serde_json::json!({
                    "edge": edge,
                    "event": "generated",
                    "faceA": face_a.map(index_u32),
                    "faceB": face_b.map(index_u32),
                }),
                EdgeEvent::Unresolved => serde_json::json!({
                    "edge": edge, "event": "unresolved",
                }),
            }
        })
        .collect();
    let vertices: Vec<serde_json::Value> = evolution
        .vertices
        .iter()
        .map(|(vertex, event)| {
            let vertex = index_u32(*vertex);
            match event {
                VertexEvent::Preserved(from) => serde_json::json!({
                    "vertex": vertex, "event": "preserved", "from": index_u32(*from),
                }),
                VertexEvent::Created => serde_json::json!({
                    "vertex": vertex, "event": "created",
                }),
            }
        })
        .collect();
    serde_json::json!({ "faces": faces, "edges": edges, "vertices": vertices })
}

fn replacement_vector(
    value: &serde_json::Value,
    key: &str,
) -> Result<remus_math::vec::Vec3, StructuredWasmError> {
    let values = crate::helpers::get_f64_array(value, key)?;
    let [x, y, z] = values.as_slice() else {
        return Err(StructuredWasmError::invalid_argument(
            format!("'{key}' must have exactly 3 components"),
            Some(key),
        ));
    };
    if values.iter().any(|component| !component.is_finite()) {
        return Err(StructuredWasmError::invalid_argument(
            format!("'{key}' must be finite"),
            Some(key),
        ));
    }
    Ok(remus_math::vec::Vec3::new(*x, *y, *z))
}

fn parse_replacement_surface(
    value: &serde_json::Value,
    source: &remus_topology::face::FaceSurface,
) -> Result<remus_topology::face::FaceSurface, StructuredWasmError> {
    use remus_topology::face::FaceSurface;
    let object = value.as_object().ok_or_else(|| {
        StructuredWasmError::invalid_argument(
            "replacement must be a surface object",
            Some("replacement"),
        )
    })?;
    let kind = value["type"].as_str().ok_or_else(|| {
        StructuredWasmError::invalid_argument(
            "replacement requires a surface type",
            Some("replacement.type"),
        )
    })?;
    let fields: &[&str] = match kind {
        "plane" => &["type", "normal", "d"],
        "cylinder" => &["type", "origin", "axis", "radius"],
        _ => {
            return Err(StructuredWasmError::invalid_argument(
                "replacement type must be plane or cylinder",
                Some("replacement.type"),
            ));
        }
    };
    if let Some(key) = object.keys().find(|key| !fields.contains(&key.as_str())) {
        return Err(StructuredWasmError::invalid_argument(
            format!("unknown replacement field '{key}'"),
            Some(key),
        ));
    }
    if kind == "plane" {
        return Ok(FaceSurface::Plane {
            normal: replacement_vector(value, "normal")?,
            d: get_f64(value, "d")?,
        });
    }
    let origin = replacement_vector(value, "origin")?;
    let origin = remus_math::vec::Point3::new(origin.x(), origin.y(), origin.z());
    let axis = replacement_vector(value, "axis")?;
    let radius = get_f64(value, "radius")?;
    let cylinder = if let FaceSurface::Cylinder(source) = source {
        // Changing the radius must not silently rotate the periodic parameter seam.
        remus_math::surfaces::CylindricalSurface::with_ref_dir(
            origin,
            axis,
            radius,
            source.x_axis(),
        )
    } else {
        remus_math::surfaces::CylindricalSurface::new(origin, axis, radius)
    }
    .map_err(StructuredWasmError::from)?;
    Ok(FaceSurface::Cylinder(cylinder))
}

impl BrepKernel {
    fn boolean_entity_evolution_json(
        &mut self,
        op: BooleanOp,
        a: u32,
        b: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
        let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
        let (solid, evolution) = remus_operations::boolean::boolean_with_entity_evolution(
            self.topo_mut(),
            op,
            a_id,
            b_id,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(solid),
            "evolution": entity_evolution_json(&evolution),
        }))
    }

    fn fillet_journaled_json(
        &mut self,
        solid: u32,
        edges: &[u32],
        radius: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let edge_ids = edges
            .iter()
            .map(|&handle| self.resolve_edge(handle))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::fillet_journaled(self.topo_mut(), solid_id, &edge_ids, radius)
            .map_err(StructuredWasmError::blend_failure)?;
        Ok(Self::blend_json(&journaled))
    }

    fn chamfer_journaled_json(
        &mut self,
        solid: u32,
        edges: &[u32],
        d1: f64,
        d2: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let edge_ids = edges
            .iter()
            .map(|&handle| self.resolve_edge(handle))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        let journaled =
            journal_ops::chamfer_journaled(self.topo_mut(), solid_id, &edge_ids, d1, d2)
                .map_err(StructuredWasmError::blend_failure)?;
        Ok(Self::blend_json(&journaled))
    }

    fn blend_json(journaled: &journal_ops::JournaledBlend) -> serde_json::Value {
        serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(journaled.result.solid),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
            "isPartial": journaled.result.is_partial,
            "failedEdges": journaled
                .result
                .failed
                .iter()
                .map(|(edge, _)| crate::handles::edge_id_to_u32(*edge))
                .collect::<Vec<_>>(),
        })
    }

    fn linear_pattern_journaled_json(
        &mut self,
        solid: u32,
        direction: [f64; 3],
        spacing: f64,
        count: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let count =
            crate::error::validate_work_count(count, "count").map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::linear_pattern_journaled(
            self.topo_mut(),
            solid_id,
            remus_math::vec::Vec3::new(direction[0], direction[1], direction[2]),
            spacing,
            count,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "compound": crate::handles::compound_id_to_u32(journaled.compound),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn imprint_json(
        &mut self,
        target: u32,
        tool: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let target_id = self
            .resolve_solid(target)
            .map_err(StructuredWasmError::from)?;
        let tool_id = self
            .resolve_solid(tool)
            .map_err(StructuredWasmError::from)?;
        let result = remus_operations::imprint::imprint(self.topo_mut(), target_id, tool_id)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn replace_surface_journaled_json(
        &mut self,
        solid: u32,
        face: u32,
        replacement: &serde_json::Value,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let source = self
            .topo()
            .face(face_id)
            .map_err(StructuredWasmError::from)?
            .surface();
        let replacement = parse_replacement_surface(replacement, source)?;
        super::operations::validate_move_faces_topology_work(self.topo(), solid_id, &[face_id])
            .map_err(StructuredWasmError::from)?;
        let result =
            journal_ops::replace_surface_journaled(self.topo_mut(), solid_id, face_id, replacement)
                .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn move_faces_journaled_json(
        &mut self,
        solid: u32,
        faces: &[u32],
        distance: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let face_count = u32::try_from(faces.len()).unwrap_or(u32::MAX);
        crate::error::validate_work_count(face_count, "faces")
            .map_err(StructuredWasmError::from)?;
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face_ids = faces
            .iter()
            .map(|&handle| self.resolve_face(handle))
            .collect::<Result<Vec<_>, _>>()
            .map_err(StructuredWasmError::from)?;
        super::operations::validate_move_faces_topology_work(self.topo(), solid_id, &face_ids)
            .map_err(StructuredWasmError::from)?;
        let result =
            journal_ops::move_faces_journaled(self.topo_mut(), solid_id, &face_ids, distance)
                .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": u32::try_from(result.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn offset_journaled_json(
        &mut self,
        solid: u32,
        distance: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::offset_journaled(self.topo_mut(), solid_id, distance)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(journaled.solid),
            "op": u32::try_from(journaled.op.value()).unwrap_or(u32::MAX),
        }))
    }

    fn journal_summary_json(&self) -> serde_json::Value {
        let entries: Vec<serde_json::Value> = self
            .topo()
            .journal()
            .entries()
            .iter()
            .map(|entry| {
                let (entry_type, detail) = match entry.payload() {
                    EntryPayload::Evolution { origin, events, .. } => (
                        "evolution",
                        serde_json::json!({
                            "origin": origin.as_str(),
                            "events": events.len(),
                        }),
                    ),
                    EntryPayload::Barrier { affected } => {
                        ("barrier", serde_json::json!({ "affected": affected.len() }))
                    }
                    EntryPayload::GlobalBarrier => ("globalBarrier", serde_json::json!({})),
                };
                serde_json::json!({
                    "op": entry.op().value(),
                    "kind": entry.kind(),
                    "type": entry_type,
                    "detail": detail,
                })
            })
            .collect();
        serde_json::Value::Array(entries)
    }

    /// Batch dispatch for the evolution-surfacing ops; `None` when `op`
    /// is not one of them.
    #[allow(clippy::redundant_pub_crate)]
    pub(crate) fn dispatch_evolution_op(
        &mut self,
        op: &str,
        args: &serde_json::Value,
    ) -> Option<Result<serde_json::Value, StructuredWasmError>> {
        let result = match op {
            "fuseWithEntityEvolution"
            | "cutWithEntityEvolution"
            | "intersectWithEntityEvolution" => {
                let bool_op = match op {
                    "fuseWithEntityEvolution" => BooleanOp::Fuse,
                    "cutWithEntityEvolution" => BooleanOp::Cut,
                    _ => BooleanOp::Intersect,
                };
                get_u32(args, "solidA").and_then(|a| {
                    get_u32(args, "solidB")
                        .and_then(|b| self.boolean_entity_evolution_json(bool_op, a, b))
                })
            }
            "filletJournaled" => get_u32(args, "solid").and_then(|solid| {
                get_u32_array(args, "edges").and_then(|edges| {
                    get_f64(args, "radius")
                        .and_then(|radius| self.fillet_journaled_json(solid, &edges, radius))
                })
            }),
            "chamferJournaled" => get_u32(args, "solid").and_then(|solid| {
                get_u32_array(args, "edges").and_then(|edges| {
                    get_f64(args, "d1").and_then(|d1| {
                        get_f64(args, "d2")
                            .and_then(|d2| self.chamfer_journaled_json(solid, &edges, d1, d2))
                    })
                })
            }),
            "linearPatternJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let direction = crate::helpers::get_f64_array(args, "direction")?;
                let [dx, dy, dz] = direction.as_slice() else {
                    return Err(StructuredWasmError::invalid_argument(
                        "'direction' must have exactly 3 components",
                        Some("direction"),
                    ));
                };
                let spacing = get_f64(args, "spacing")?;
                let count = get_u32(args, "count")?;
                self.linear_pattern_journaled_json(solid, [*dx, *dy, *dz], spacing, count)
            })(),
            "imprint" => get_u32(args, "target").and_then(|target| {
                get_u32(args, "tool").and_then(|tool| self.imprint_json(target, tool))
            }),
            "replaceSurfaceJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let face = get_u32(args, "face")?;
                self.replace_surface_journaled_json(solid, face, &args["replacement"])
            })(),
            "moveFacesJournaled" => (|| {
                let solid = get_u32(args, "solid")?;
                let faces = get_u32_array(args, "faces")?;
                let distance = get_f64(args, "distance")?;
                self.move_faces_journaled_json(solid, &faces, distance)
            })(),
            "offsetJournaled" => get_u32(args, "solid").and_then(|solid| {
                get_f64(args, "distance")
                    .and_then(|distance| self.offset_journaled_json(solid, distance))
            }),
            "journalSummary" => Ok(self.journal_summary_json()),
            _ => return None,
        };
        Some(result)
    }
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Entity evolution (Issue 12) ─────────────────────────────────

    /// Fuse with full construction-derived vertex/edge/face history.
    ///
    /// Returns JSON `{"solid", "evolution": {"faces", "edges",
    /// "vertices"}}`; edge events are `preserved`/`modified` (with
    /// `from`), `generated` (with the generating `faceA`/`faceB` when
    /// they map), or the honest `unresolved`.
    #[wasm_bindgen(js_name = "fuseWithEntityEvolution")]
    pub fn fuse_with_entity_evolution(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.boolean_entity_evolution_json(BooleanOp::Fuse, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Cut with full entity history; see `fuseWithEntityEvolution`.
    #[wasm_bindgen(js_name = "cutWithEntityEvolution")]
    pub fn cut_with_entity_evolution(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.boolean_entity_evolution_json(BooleanOp::Cut, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Intersect with full entity history; see `fuseWithEntityEvolution`.
    #[wasm_bindgen(js_name = "intersectWithEntityEvolution")]
    pub fn intersect_with_entity_evolution(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.boolean_entity_evolution_json(BooleanOp::Intersect, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// V2 fillet journaled as one evolution entry (kind `fillet`).
    ///
    /// Returns JSON `{"solid", "op", "isPartial", "failedEdges"}`.
    #[wasm_bindgen(js_name = "filletJournaled")]
    pub fn fillet_journaled_js(
        &mut self,
        solid: u32,
        edges: Vec<u32>,
        radius: f64,
    ) -> Result<String, JsError> {
        validate_finite(radius, "radius")?;
        self.fillet_journaled_json(solid, &edges, radius)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// V2 chamfer journaled as one evolution entry (kind `chamfer`).
    #[wasm_bindgen(js_name = "chamferJournaled")]
    pub fn chamfer_journaled_js(
        &mut self,
        solid: u32,
        edges: Vec<u32>,
        d1: f64,
        d2: f64,
    ) -> Result<String, JsError> {
        validate_finite(d1, "d1")?;
        validate_finite(d2, "d2")?;
        self.chamfer_journaled_json(solid, &edges, d1, d2)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Linear pattern journaled as one evolution entry (kind
    /// `linear_pattern`). Returns JSON `{"compound", "op"}`.
    #[wasm_bindgen(js_name = "linearPatternJournaled")]
    pub fn linear_pattern_journaled_js(
        &mut self,
        solid: u32,
        dx: f64,
        dy: f64,
        dz: f64,
        spacing: f64,
        count: u32,
    ) -> Result<String, JsError> {
        for (name, value) in [("dx", dx), ("dy", dy), ("dz", dz), ("spacing", spacing)] {
            validate_finite(value, name)?;
        }
        self.linear_pattern_journaled_json(solid, [dx, dy, dz], spacing, count)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Imprints one solid's intersection edges onto another without removing
    /// material. Returns JSON `{"solid", "op"}` for the new target and its
    /// construction-derived journal entry.
    #[wasm_bindgen(js_name = "imprint")]
    pub fn imprint_js(&mut self, target: u32, tool: u32) -> Result<String, JsError> {
        self.imprint_json(target, tool)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Replace a support surface with exact face, edge, and vertex history.
    ///
    /// `replacement` is JSON: `{type:"plane", normal:[x,y,z], d}` or
    /// `{type:"cylinder", origin:[x,y,z], axis:[x,y,z], radius}`. Plane
    /// coefficients represent `normal dot point = d`; cylinder replacements
    /// retain the source parameter reference direction. Unknown fields refuse.
    /// Returns JSON `{"solid", "op"}`. Batch calls pass the replacement object
    /// directly as `args.replacement`.
    #[wasm_bindgen(js_name = "replaceSurfaceJournaled")]
    pub fn replace_surface_journaled_js(
        &mut self,
        solid: u32,
        face: u32,
        replacement: &str,
    ) -> Result<String, JsError> {
        let replacement = serde_json::from_str(replacement)
            .map_err(|error| structured_to_js(StructuredWasmError::invalid_json(&error)))?;
        self.replace_surface_journaled_json(solid, face, &replacement)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// Move faces with construction history.
    ///
    /// Planar re-limitation and coaxial bore moves include edge and vertex
    /// history. Blend moves retain copy-derived history or a complete, unique
    /// boundary correspondence anchored on construction face identities.
    /// Ambiguous reconstructed boundaries retain faces-only history.
    /// Returns JSON `{"solid", "op"}`.
    #[wasm_bindgen(js_name = "moveFacesJournaled")]
    pub fn move_faces_journaled_js(
        &mut self,
        solid: u32,
        faces: &[u32],
        distance: f64,
    ) -> Result<String, JsError> {
        validate_finite(distance, "distance")?;
        self.move_faces_journaled_json(solid, faces, distance)
            .map(|value| value.to_string())
            .map_err(structured_to_js)
    }

    /// V2 offset journaled as one construction-derived face-evolution entry
    /// (kind `offset`). Returns JSON `{"solid", "op"}`.
    #[wasm_bindgen(js_name = "offsetJournaled")]
    pub fn offset_journaled_js(&mut self, solid: u32, distance: f64) -> Result<String, JsError> {
        validate_finite(distance, "distance")?;
        self.offset_journaled_json(solid, distance)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// A read-only summary of the evolution journal: JSON array of
    /// `{"op", "kind", "type", "detail"}` where `type` is `evolution`
    /// (detail: origin, event count), `barrier` (detail: affected
    /// count), or `globalBarrier`.
    #[wasm_bindgen(js_name = "journalSummary")]
    #[must_use]
    pub fn journal_summary(&self) -> String {
        self.journal_summary_json().to_string()
    }
}

fn structured_to_js(error: StructuredWasmError) -> JsError {
    JsError::new(error.message())
}

#[cfg(test)]
mod evolution_contract_tests {
    #![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

    use crate::kernel::BrepKernel;

    fn run(kernel: &mut BrepKernel, ops: serde_json::Value) -> Vec<serde_json::Value> {
        let response = kernel.execute_batch(&ops.to_string());
        let parsed: serde_json::Value =
            serde_json::from_str(&response).expect("batch response must be valid JSON");
        parsed
            .as_array()
            .expect("batch response is an array")
            .iter()
            .map(|entry| {
                assert!(
                    entry.get("error").is_none(),
                    "unexpected batch error: {entry}"
                );
                entry["ok"].clone()
            })
            .collect()
    }

    fn assert_assembly_rebuilds_are_typed(evolution: &serde_json::Value) {
        let events = evolution["edges"].as_array().unwrap();
        assert!(
            events.iter().any(|edge| edge["event"] == "modified"),
            "fixture must exercise typed edge reconstruction: {events:?}"
        );
        assert!(
            events.iter().all(|edge| edge["event"] != "unresolved"),
            "recorded assembly rebuilds must not surface as unresolved: {events:?}"
        );
    }

    #[test]
    fn direct_wasm_entity_evolution_types_assembly_rebuilds() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let b = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        kernel
            .transform_solid_binding(
                b,
                vec![
                    1.0, 0.0, 0.0, 5.0, 0.0, 1.0, 0.0, 5.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 0.0, 1.0,
                ],
            )
            .unwrap();

        let payload: serde_json::Value =
            serde_json::from_str(&kernel.fuse_with_entity_evolution(a, b).unwrap()).unwrap();
        assert_assembly_rebuilds_are_typed(&payload["evolution"]);
    }

    #[test]
    fn entity_evolution_surfaces_all_three_claim_strengths() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "transform", "args": {"solid": 1, "matrix":
                    [1.0,0.0,0.0,5.0, 0.0,1.0,0.0,5.0, 0.0,0.0,1.0,5.0, 0.0,0.0,0.0,1.0]}},
                {"op": "fuseWithEntityEvolution", "args": {"solidA": 0, "solidB": 1}},
            ]),
        );
        let payload = &results[3];
        assert!(payload["solid"].as_u64().is_some());
        let evolution = &payload["evolution"];
        assert!(!evolution["faces"].as_array().unwrap().is_empty());
        assert!(!evolution["vertices"].as_array().unwrap().is_empty());

        let edge_events: Vec<&str> = evolution["edges"]
            .as_array()
            .unwrap()
            .iter()
            .map(|e| e["event"].as_str().unwrap())
            .collect();
        for expected in ["preserved", "modified", "generated"] {
            assert!(
                edge_events.contains(&expected),
                "a cube fuse must show {expected} edges: {edge_events:?}"
            );
        }
        assert_assembly_rebuilds_are_typed(evolution);
        // Generated edges name their generating faces when they map.
        assert!(
            evolution["edges"]
                .as_array()
                .unwrap()
                .iter()
                .any(|e| e["event"] == "generated" && e["faceA"].is_u64()),
            "section edges name their generating faces"
        );
    }

    #[test]
    fn journaled_blends_and_patterns_populate_the_journal() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "solidEdges", "args": {"solid": 0}},
            ]),
        );
        let edge = results[1].as_array().expect("solidEdges returns an array")[0]
            .as_u64()
            .expect("edge handle");

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "setFaceName", "args": {"face": 0, "name": "base"}},
                {"op": "filletJournaled", "args": {"solid": 0, "edges": [edge], "radius": 1.0}},
            ]),
        );
        let fillet = &results[1];
        let fillet_op = fillet["op"].as_u64().unwrap();
        assert_eq!(fillet["isPartial"], serde_json::json!(false));
        assert_eq!(fillet["failedEdges"], serde_json::json!([]));
        let fillet_solid = fillet["solid"].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "propagateAttributesForOp", "args": {"op": fillet_op}},
                {"op": "linearPatternJournaled", "args": {
                    "solid": fillet_solid, "direction": [1.0, 0.0, 0.0],
                    "spacing": 20.0, "count": 3}},
                {"op": "journalSummary", "args": {}},
            ]),
        );
        assert!(
            results[0]["carried"].as_u64().unwrap() > 0,
            "the named face must ride the fillet's journal entry: {}",
            results[0]
        );
        let pattern_op = results[1]["op"].as_u64().unwrap();
        assert!(results[1]["compound"].as_u64().is_some());

        // The journal now holds both entries as construction evolution.
        let summary = results[2].as_array().unwrap();
        let find = |op: u64| {
            summary
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(op))
                .unwrap_or_else(|| panic!("op {op} missing from journal summary"))
        };
        let fillet_entry = find(fillet_op);
        assert_eq!(fillet_entry["kind"], "fillet");
        assert_eq!(fillet_entry["type"], "evolution");
        let pattern_entry = find(pattern_op);
        assert_eq!(pattern_entry["kind"], "linear_pattern");
        assert_eq!(pattern_entry["type"], "evolution");
        assert_eq!(pattern_entry["detail"]["origin"], "construction");

        // Pattern provenance carries names onto instances.
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "propagateAttributesForOp", "args": {"op": pattern_op}},
            ]),
        );
        assert!(
            results[0]["carried"].as_u64().unwrap() > 0,
            "instance faces must inherit the original's name: {}",
            results[0]
        );
    }

    fn replacement_box() -> (BrepKernel, u32, u32) {
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(3.0, 5.0, 7.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        let face = remus_topology::explorer::solid_faces(kernel.topo(), id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        (kernel, source, super::index_u32(face.index()))
    }

    #[test]
    fn replacement_history_has_direct_batch_reference_parity() {
        use remus_topology::journal::{EntityKind, OpId};
        use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
        let mut results = Vec::new();
        for batch in [false, true] {
            let (mut kernel, source, face) = replacement_box();
            let first = kernel
                .replace_surface_journaled_json(
                    source,
                    face,
                    &serde_json::json!({"type":"plane", "normal":[0,0,1], "d":8}),
                )
                .unwrap();
            let anchor = OpId::from_value(first["op"].as_u64().unwrap());
            let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
            let solid_id = kernel.resolve_solid(solid).unwrap();
            let face = remus_topology::explorer::solid_faces(kernel.topo(), solid_id)
                .unwrap()
                .into_iter()
                .find(|&face| {
                    kernel
                        .topo()
                        .face(face)
                        .unwrap()
                        .effective_plane_normal()
                        .is_some_and(|normal| normal.z() > 0.9)
                })
                .unwrap();
            let face = super::index_u32(face.index());
            let replacement = serde_json::json!({"type":"plane", "normal":[-0.1,0,1], "d":7.85});
            let second = if batch {
                run(&mut kernel, serde_json::json!([
                    {"op":"replaceSurfaceJournaled", "args":{"solid":solid,"face":face,"replacement":replacement}}
                ])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .replace_surface_journaled_js(solid, face, &replacement.to_string())
                        .unwrap(),
                )
                .unwrap()
            };
            let solid = u32::try_from(second["solid"].as_u64().unwrap()).unwrap();
            assert!((kernel.volume(solid, 0.01).unwrap() - 120.0).abs() < 1e-8);
            let solid_id = kernel.resolve_solid(solid).unwrap();
            let live: std::collections::BTreeSet<_> =
                remus_operations::journal_ops::solid_entity_keys(kernel.topo(), solid_id)
                    .unwrap()
                    .into_iter()
                    .collect();
            let mut resolved = std::collections::BTreeSet::new();
            for (kind, count) in [
                (EntityKind::Face, 6),
                (EntityKind::Edge, 12),
                (EntityKind::Vertex, 8),
            ] {
                for index in 0..count {
                    let resolution = resolve(
                        kernel.topo(),
                        &PersistentRef::operation_output(anchor, kind, index),
                    );
                    let Resolution::Bound { entity, provenance } = resolution else {
                        panic!("{resolution:?}")
                    };
                    assert_eq!(provenance, Provenance::Construction);
                    assert!(resolved.insert(entity));
                }
            }
            assert_eq!(resolved, live);
            results.push(second);
        }
        assert_eq!(results[0], results[1]);
    }

    #[test]
    fn replacement_refusals_have_matching_errors_and_restore_history() {
        let (mut kernel, source, face) = replacement_box();
        let first = kernel
            .replace_surface_journaled_json(
                source,
                face,
                &serde_json::json!({"type":"plane", "normal":[0,0,1], "d":8}),
            )
            .unwrap();
        let source = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
        let source_id = kernel.resolve_solid(source).unwrap();
        let face = remus_topology::explorer::solid_faces(kernel.topo(), source_id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        let face = super::index_u32(face.index());
        let before = kernel.topo().journal().snapshot();
        let counts = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
            kernel.topo().num_pcurves(),
        );
        for replacement in [
            serde_json::json!(null),
            serde_json::json!({}),
            serde_json::json!({"type":"sphere"}),
            serde_json::json!({"type":"plane", "normal":[0,0], "d":9}),
            serde_json::json!({"type":"plane", "normal":[0,0,0], "d":9}),
            serde_json::json!({"type":"plane", "normal":[0,0,1], "d":-1}),
            serde_json::json!({"type":"plane", "normal":[0,0,1], "d":9, "offset":1}),
            serde_json::json!({"type":"cylinder", "origin":[0,0,0], "axis":[0,0,1], "radius":1}),
        ] {
            let direct = serde_json::to_value(
                kernel
                    .replace_surface_journaled_json(source, face, &replacement)
                    .unwrap_err(),
            )
            .unwrap();
            let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
                {"op":"replaceSurfaceJournaled", "args":{"solid":source,"face":face,"replacement":replacement}}
            ]).to_string())).unwrap();
            let mut batch = response[0]["error"].clone();
            let details = batch["details"].as_object_mut().unwrap();
            assert_eq!(
                details.remove("operation"),
                Some(serde_json::json!("replaceSurfaceJournaled"))
            );
            assert_eq!(details.remove("operationIndex"), Some(serde_json::json!(0)));
            assert_eq!(batch, direct);
            assert_eq!(kernel.topo().journal().snapshot(), before);
            assert_eq!(
                (
                    kernel.topo().num_vertices(),
                    kernel.topo().num_edges(),
                    kernel.topo().num_faces(),
                    kernel.topo().num_pcurves()
                ),
                counts
            );
        }
    }

    #[test]
    fn replacement_preserves_the_topology_work_budget() {
        let mut kernel = BrepKernel::new();
        let wire = kernel.make_regular_polygon_wire(10.0, 500).unwrap();
        let profile = kernel.make_face_from_wire(wire).unwrap();
        let source = kernel.extrude_face(profile, 0.0, 0.0, 1.0, 1.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        assert!(
            remus_operations::validate::validate_solid(kernel.topo(), id)
                .unwrap()
                .is_valid()
        );
        let face = remus_topology::explorer::solid_faces(kernel.topo(), id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        let face = super::index_u32(face.index());
        let before = kernel.topo().journal().snapshot();
        let replacement = serde_json::json!({"type":"plane","normal":[0,0,1],"d":1.25});
        let error = kernel
            .replace_surface_journaled_json(source, face, &replacement)
            .unwrap_err();
        assert!(
            error
                .message()
                .contains("moveFaces topology work must be at most"),
            "{}",
            error.message()
        );
        let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
            {"op":"replaceSurfaceJournaled","args":{"solid":source,"face":face,"replacement":replacement}}
        ]).to_string())).unwrap();
        assert_eq!(response[0]["error"]["message"], error.message());
        assert_eq!(kernel.topo().journal().snapshot(), before);
    }

    #[test]
    fn replacement_cylinder_parser_preserves_the_source_parameter_direction() {
        use remus_math::surfaces::CylindricalSurface;
        use remus_math::vec::{Point3, Vec3};
        use remus_topology::face::FaceSurface;
        let source = CylindricalSurface::with_ref_dir(
            Point3::new(3.0, 4.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let replacement = super::parse_replacement_surface(
            &serde_json::json!({
                "type":"cylinder", "origin":[3,4,5], "axis":[0,0,1], "radius":2
            }),
            &FaceSurface::Cylinder(source.clone()),
        )
        .unwrap();
        let FaceSurface::Cylinder(replacement) = replacement else {
            panic!("cylinder required")
        };
        assert!((replacement.x_axis() - source.x_axis()).length() < 1e-12);
        assert!((replacement.radius() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn move_faces_journaled_preserves_the_face_count_budget() {
        let mut kernel = BrepKernel::new();
        let source = kernel.make_box_solid(3.0, 5.0, 7.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        let face = super::index_u32(
            remus_topology::explorer::solid_faces(kernel.topo(), id).unwrap()[0].index(),
        );
        let faces = vec![face; crate::error::MAX_WASM_WORK_ITEMS as usize + 1];
        let before = kernel.topo().journal().snapshot();
        let error = kernel
            .move_faces_journaled_json(source, &faces, 0.25)
            .unwrap_err();
        assert!(
            error.message().contains("faces must be at most"),
            "{}",
            error.message()
        );
        let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
            {"op":"moveFacesJournaled", "args":{"solid":source,"faces":faces,"distance":0.25}}
        ]).to_string())).unwrap();
        assert_eq!(response[0]["error"]["message"], error.message());
        assert_eq!(kernel.topo().journal().snapshot(), before);
    }

    #[test]
    fn move_faces_journaled_preserves_the_topology_work_budget() {
        let mut kernel = BrepKernel::new();
        let wire = kernel.make_regular_polygon_wire(10.0, 500).unwrap();
        let face = kernel.make_face_from_wire(wire).unwrap();
        let source = kernel.extrude_face(face, 0.0, 0.0, 1.0, 1.0).unwrap();
        let id = kernel.resolve_solid(source).unwrap();
        assert!(
            remus_operations::validate::validate_solid(kernel.topo(), id)
                .unwrap()
                .is_valid()
        );
        let face = remus_topology::explorer::solid_faces(kernel.topo(), id)
            .unwrap()
            .into_iter()
            .find(|&face| {
                kernel
                    .topo()
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.z() > 0.9)
            })
            .unwrap();
        let face = super::index_u32(face.index());
        let before = kernel.topo().journal().snapshot();
        let error = kernel
            .move_faces_journaled_json(source, &[face], 0.25)
            .unwrap_err();
        assert!(
            error
                .message()
                .contains("moveFaces topology work must be at most"),
            "{}",
            error.message()
        );
        let response: serde_json::Value = serde_json::from_str(&kernel.execute_batch_v2(&serde_json::json!([
            {"op":"moveFacesJournaled", "args":{"solid":source,"faces":[face],"distance":0.25}}
        ]).to_string())).unwrap();
        assert_eq!(response[0]["error"]["message"], error.message());
        assert_eq!(kernel.topo().journal().snapshot(), before);
    }

    #[test]
    fn move_faces_journaled_preserves_all_entity_refs_in_direct_and_batch_calls() {
        use super::index_u32;
        use remus_operations::journal_ops;
        use remus_topology::journal::{EntityKind, OpId};
        use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

        let mut payloads = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let source = kernel.make_box_solid(3.0, 5.0, 7.0).unwrap();
            let source_id = kernel.resolve_solid(source).unwrap();
            let face = remus_topology::explorer::solid_faces(kernel.topo(), source_id).unwrap()[0];
            let face = index_u32(face.index());
            let first = kernel
                .move_faces_journaled_json(source, &[face], 0.25)
                .unwrap();
            let solid = u32::try_from(first["solid"].as_u64().unwrap()).unwrap();
            let anchor = OpId::from_value(first["op"].as_u64().unwrap());
            let first_id = kernel.resolve_solid(solid).unwrap();
            let face = remus_topology::explorer::solid_faces(kernel.topo(), first_id).unwrap()[0];
            let face = index_u32(face.index());
            let second = if batch {
                run(&mut kernel, serde_json::json!([
                    {"op": "moveFacesJournaled", "args": {"solid": solid, "faces": [face], "distance": -0.125}}
                ])).remove(0)
            } else {
                serde_json::from_str(
                    &kernel
                        .move_faces_journaled_js(solid, &[face], -0.125)
                        .unwrap(),
                )
                .unwrap()
            };
            let result = u32::try_from(second["solid"].as_u64().unwrap()).unwrap();
            let result_id = kernel.resolve_solid(result).unwrap();
            let live: std::collections::BTreeSet<_> =
                journal_ops::solid_entity_keys(kernel.topo(), result_id)
                    .unwrap()
                    .into_iter()
                    .collect();
            let mut resolved = std::collections::BTreeSet::new();
            for (kind, count) in [
                (EntityKind::Face, 6),
                (EntityKind::Edge, 12),
                (EntityKind::Vertex, 8),
            ] {
                for index in 0..count {
                    let reference = PersistentRef::operation_output(anchor, kind, index);
                    let resolution = resolve(kernel.topo(), &reference);
                    let Resolution::Bound { entity, provenance } = resolution else {
                        panic!("lost direct-edit reference: {resolution:?}");
                    };
                    assert_eq!(provenance, Provenance::Construction);
                    assert!(resolved.insert(entity));
                }
            }
            assert_eq!(resolved, live);
            payloads.push(second);
        }
        assert_eq!(payloads[0], payloads[1]);
    }

    #[test]
    fn offset_journaled_has_direct_and_batch_contract_parity() {
        let mut direct = BrepKernel::new();
        let source = direct.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let payload: serde_json::Value =
            serde_json::from_str(&direct.offset_journaled_js(source, 0.5).unwrap()).unwrap();
        let result = u32::try_from(payload["solid"].as_u64().unwrap()).unwrap();
        assert!((direct.volume(result, 0.1).unwrap() - 27.0).abs() < 1e-9);
        let summary: serde_json::Value = serde_json::from_str(&direct.journal_summary()).unwrap();
        let entry = summary.as_array().unwrap().last().unwrap();
        assert_eq!(entry["kind"], "offset");
        assert_eq!(entry["type"], "evolution");
        assert_eq!(entry["detail"]["origin"], "construction");
        assert_eq!(entry["detail"]["events"], 6);

        let mut batch = BrepKernel::new();
        run(
            &mut batch,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 2.0, "height": 2.0, "depth": 2.0}},
            ]),
        );
        let results = run(
            &mut batch,
            serde_json::json!([
                {"op": "offsetJournaled", "args": {"solid": 0, "distance": 0.5}},
                {"op": "journalSummary", "args": {}},
            ]),
        );
        assert_eq!(results[0], payload);
        let batch_entry = results[1].as_array().unwrap().last().unwrap();
        assert_eq!(batch_entry["kind"], "offset");
        assert_eq!(batch_entry["type"], "evolution");
        assert_eq!(batch_entry["detail"]["events"], 6);
    }

    #[test]
    fn chamfer_journaled_severs_edge_refs_like_any_faces_only_entry() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "transform", "args": {"solid": 1, "matrix":
                    [1.0,0.0,0.0,5.0, 0.0,1.0,0.0,5.0, 0.0,0.0,1.0,5.0, 0.0,0.0,0.0,1.0]}},
                {"op": "fuseJournaled", "args": {"solidA": 0, "solidB": 1}},
            ]),
        );
        let fuse_op = results[3]["op"].as_u64().unwrap();
        let fused = results[3]["solid"].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "resolveOperationOutput",
                 "args": {"op": fuse_op, "kind": "edge", "index": 0}},
                {"op": "solidEdges", "args": {"solid": fused}},
            ]),
        );
        assert_eq!(results[0]["status"], "bound");
        let edge = results[1].as_array().unwrap()[0].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "chamferJournaled",
                 "args": {"solid": fused, "edges": [edge], "d1": 0.5, "d2": 0.5}},
                {"op": "resolveOperationOutput",
                 "args": {"op": fuse_op, "kind": "edge", "index": 0}},
            ]),
        );
        assert_eq!(results[1]["status"], "unresolvedAcrossOperation");
        assert_eq!(results[1]["operationKind"], "chamfer");
    }

    fn imprint_handles(kernel: &mut BrepKernel) -> (u32, u32) {
        let target = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let tool = kernel.make_box_solid(6.0, 6.0, 6.0).unwrap();
        kernel
            .transform_solid_binding(
                tool,
                vec![
                    1.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, -3.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
                ],
            )
            .unwrap();
        (target, tool)
    }

    #[test]
    fn direct_and_batch_imprint_match_and_surface_the_journal_entry() {
        let mut signatures = Vec::new();
        for batch in [false, true] {
            let mut kernel = BrepKernel::new();
            let (target, tool) = imprint_handles(&mut kernel);
            let payload = if batch {
                run(
                    &mut kernel,
                    serde_json::json!([{
                        "op": "imprint",
                        "args": {"target": target, "tool": tool},
                    }]),
                )[0]
                .clone()
            } else {
                serde_json::from_str(&kernel.imprint_js(target, tool).unwrap()).unwrap()
            };
            let solid = payload["solid"].as_u64().unwrap() as u32;
            let op = payload["op"].as_u64().unwrap();
            let volume = kernel.volume(solid, 0.01).unwrap();
            let faces = kernel.get_solid_faces(solid).unwrap();
            assert!((volume - 1000.0).abs() < 1.0e-9);
            assert!(faces.len() > 6);

            let summary = run(
                &mut kernel,
                serde_json::json!([{"op": "journalSummary", "args": {}}]),
            );
            let entry = summary[0]
                .as_array()
                .unwrap()
                .iter()
                .find(|entry| entry["op"].as_u64() == Some(op))
                .unwrap();
            assert_eq!(entry["kind"], "imprint");
            assert_eq!(entry["type"], "evolution");
            assert_eq!(entry["detail"]["origin"], "construction");
            signatures.push(((volume * 1.0e9).round() as i64, faces.len()));
        }
        assert_eq!(signatures[0], signatures[1]);
    }

    #[test]
    fn batch_v2_imprint_preserves_typed_atomic_refusal() {
        let mut kernel = BrepKernel::new();
        let target = kernel.make_box_solid(10.0, 10.0, 10.0).unwrap();
        let counts_before = kernel.topo().allocated_slot_count();
        let response: Vec<serde_json::Value> = serde_json::from_str(
            &kernel.execute_batch_v2(
                &serde_json::json!([{
                    "op": "imprint",
                    "args": {"target": target, "tool": target},
                }])
                .to_string(),
            ),
        )
        .unwrap();
        assert_eq!(
            response[0]["error"]["details"]["kernelCode"],
            "unsupported_imprint"
        );
        assert_eq!(response[0]["error"]["category"], "unsupported");
        assert_eq!(kernel.topo().allocated_slot_count(), counts_before);
    }
}
