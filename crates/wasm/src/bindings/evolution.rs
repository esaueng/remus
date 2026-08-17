//! Entity-evolution surfacing (Issue 12 → JS).
//!
//! Exposes the construction-derived vertex/edge/face history of GFA
//! booleans, one-call journaled blends and patterns, and a read-only
//! journal summary. Event encodings are stable JSON; `unresolved` is an
//! honest event, never hidden.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use remus_operations::boolean::{BooleanOp, EdgeEvent, EntityEvolution, VertexEvent};
use remus_operations::journal_ops;
use remus_topology::journal::EntryPayload;

use crate::error::StructuredWasmError;
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
            .map_err(StructuredWasmError::from)?;
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
                .map_err(StructuredWasmError::from)?;
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
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let journaled = journal_ops::linear_pattern_journaled(
            self.topo_mut(),
            solid_id,
            remus_math::vec::Vec3::new(direction[0], direction[1], direction[2]),
            spacing,
            count as usize,
        )
        .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "compound": crate::handles::compound_id_to_u32(journaled.compound),
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
        self.linear_pattern_journaled_json(solid, [dx, dy, dz], spacing, count)
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
}
