//! Persistent-naming bindings (RFC 0003 WASM reference API).
//!
//! Exposes the evolution journal and persistent references to JS:
//! journaled booleans (construction-derived history), explicit barriers,
//! journal-driven attribute propagation, reference resolution, and — with
//! the `io` feature — the serialized reference codec (references travel
//! as versioned JSON strings, opaque to JS).
//!
//! Resolution outcomes are **data, not errors**: `resolveRef` and
//! `resolveOperationOutput` return a JSON object whose `status` field is
//! one of `bound`, `boundMany`, `ambiguous`, `dangling`,
//! `unresolvedAcrossOperation`, `unknownOperation`, or `noMatch` — a
//! severed reference is an answer the caller handles, not an exception.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use brepkit_algo::bop::BooleanOp;
use brepkit_operations::journal_ops;
use brepkit_topology::journal::{EntityKind, JournalAttributePropagation, OpId};
// Discriminators are only constructed by the serialized-reference codec,
// which is `io`-gated.
#[cfg(feature = "io")]
use brepkit_topology::naming::Discriminator;
use brepkit_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

use crate::error::StructuredWasmError;
use crate::helpers::get_u32;
use crate::kernel::BrepKernel;

/// Visibility note: `pub(crate)` triggers `clippy::redundant_pub_crate`
/// because `bindings` is a private module; kept to make the
/// cross-module-but-crate-internal sharing explicit.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn parse_entity_kind(name: &str) -> Result<EntityKind, StructuredWasmError> {
    match name {
        "vertex" => Ok(EntityKind::Vertex),
        "edge" => Ok(EntityKind::Edge),
        "face" => Ok(EntityKind::Face),
        other => Err(StructuredWasmError::invalid_argument(
            format!("unknown entity kind '{other}' (expected vertex, edge, or face)"),
            Some("kind"),
        )),
    }
}

fn get_str<'a>(args: &'a serde_json::Value, key: &str) -> Result<&'a str, StructuredWasmError> {
    args[key].as_str().ok_or_else(|| {
        StructuredWasmError::invalid_argument(format!("missing or invalid '{key}'"), Some(key))
    })
}

fn op_to_u32(op: OpId) -> Result<u32, StructuredWasmError> {
    u32::try_from(op.value()).map_err(|_| {
        StructuredWasmError::invalid_argument("journal op id exceeds the u32 range", Some("op"))
    })
}

fn entity_json(key: brepkit_topology::journal::EntityKey) -> serde_json::Value {
    serde_json::json!({
        "kind": key.kind.as_str(),
        "handle": u32::try_from(key.index).unwrap_or(u32::MAX),
    })
}

fn provenance_str(provenance: Provenance) -> &'static str {
    match provenance {
        Provenance::Construction => "construction",
        Provenance::Inferred => "inferred",
    }
}

/// The stable JSON encoding of a resolution outcome.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn resolution_json(resolution: &Resolution) -> serde_json::Value {
    match resolution {
        Resolution::Bound { entity, provenance } => serde_json::json!({
            "status": "bound",
            "entities": [entity_json(*entity)],
            "provenance": provenance_str(*provenance),
        }),
        Resolution::BoundMany {
            entities,
            provenance,
        } => serde_json::json!({
            "status": "boundMany",
            "entities": entities.iter().map(|&e| entity_json(e)).collect::<Vec<_>>(),
            "provenance": provenance_str(*provenance),
        }),
        Resolution::Ambiguous { candidates, reason } => serde_json::json!({
            "status": "ambiguous",
            "candidates": candidates.iter().map(|&e| entity_json(e)).collect::<Vec<_>>(),
            "reason": reason,
        }),
        Resolution::Dangling { deleted_at } => serde_json::json!({
            "status": "dangling",
            "deletedAt": deleted_at.value(),
        }),
        Resolution::UnresolvedAcrossOperation { op, kind } => serde_json::json!({
            "status": "unresolvedAcrossOperation",
            "op": op.value(),
            "operationKind": kind,
        }),
        Resolution::UnknownOperation { op } => serde_json::json!({
            "status": "unknownOperation",
            "op": op.value(),
        }),
        Resolution::NoMatch { reason } => serde_json::json!({
            "status": "noMatch",
            "reason": reason,
        }),
    }
}

fn propagation_json(report: JournalAttributePropagation) -> serde_json::Value {
    serde_json::json!({
        "carried": report.carried,
        "unresolvedOutputs": report.unresolved_outputs,
        "mergeConflicts": report.merge_conflicts,
        "refusedInferred": report.refused_inferred,
    })
}

impl BrepKernel {
    fn journaled_boolean_json(
        &mut self,
        op: BooleanOp,
        a: u32,
        b: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let a_id = self.resolve_solid(a).map_err(StructuredWasmError::from)?;
        let b_id = self.resolve_solid(b).map_err(StructuredWasmError::from)?;
        let result = journal_ops::boolean_journaled(self.topo_mut(), op, a_id, b_id)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({
            "solid": crate::handles::solid_id_to_u32(result.solid),
            "op": op_to_u32(result.op)?,
        }))
    }

    fn journal_barrier_json(
        &mut self,
        kind: &str,
        solid: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let solid_id = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let topo = self.topo_mut();
        // Recorded after the operation ran: any preceding unjournaled
        // mutations surface as a global barrier first — both sever, so
        // the one-call form is exactly as fail-closed as the native
        // two-phase API for barrier use.
        let pending = topo.journal_begin(kind);
        let op = journal_ops::record_barrier_over_solid(topo, pending, solid_id)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!({ "op": op_to_u32(op)? }))
    }

    fn propagate_attributes_json(
        &mut self,
        op: u32,
        allow_inferred: bool,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let report = self
            .topo_mut()
            .propagate_attributes_for_op(OpId::from_value(u64::from(op)), allow_inferred)
            .map_err(StructuredWasmError::from)?;
        Ok(propagation_json(report))
    }

    fn resolve_operation_output_json(
        &self,
        op: u32,
        kind: &str,
        index: u32,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let reference = PersistentRef::operation_output(
            OpId::from_value(u64::from(op)),
            parse_entity_kind(kind)?,
            index as usize,
        );
        Ok(resolution_json(&resolve(self.topo(), &reference)))
    }

    fn set_face_name_json(
        &mut self,
        face: u32,
        name: Option<&str>,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let mut attributes = self
            .topo()
            .attributes()
            .face(face_id)
            .cloned()
            .unwrap_or_default();
        attributes.name = name.filter(|n| !n.is_empty()).map(str::to_owned);
        self.topo_mut()
            .set_face_attributes(face_id, attributes)
            .map_err(StructuredWasmError::from)?;
        Ok(serde_json::json!(true))
    }

    fn get_face_name_json(&self, face: u32) -> Result<serde_json::Value, StructuredWasmError> {
        let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        let name = self
            .topo()
            .attributes()
            .face(face_id)
            .and_then(|attributes| attributes.name.clone());
        Ok(name.map_or(serde_json::Value::Null, serde_json::Value::String))
    }

    #[cfg(feature = "io")]
    fn ref_from_json(reference: &str) -> Result<PersistentRef, StructuredWasmError> {
        brepkit_io::naming_io::deserialize_persistent_ref(reference.as_bytes())
            .map_err(StructuredWasmError::from)
    }

    #[cfg(feature = "io")]
    fn ref_to_json(reference: &PersistentRef) -> Result<serde_json::Value, StructuredWasmError> {
        let bytes = brepkit_io::naming_io::serialize_persistent_ref(reference)
            .map_err(StructuredWasmError::from)?;
        let text = String::from_utf8(bytes).map_err(|_| {
            StructuredWasmError::invalid_argument("reference encoding was not UTF-8", None)
        })?;
        Ok(serde_json::json!({ "ref": text }))
    }

    #[cfg(feature = "io")]
    fn capture_signature_ref_json(
        &self,
        kind: &str,
        handle: u32,
        quantum: f64,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        use brepkit_topology::naming::EntitySignature;
        let signature = match parse_entity_kind(kind)? {
            EntityKind::Face => {
                let id = self
                    .resolve_face(handle)
                    .map_err(StructuredWasmError::from)?;
                EntitySignature::capture_face(self.topo(), id, quantum)
            }
            EntityKind::Edge => {
                let id = self
                    .resolve_edge(handle)
                    .map_err(StructuredWasmError::from)?;
                EntitySignature::capture_edge(self.topo(), id, quantum)
            }
            EntityKind::Vertex => {
                let id = self
                    .resolve_vertex(handle)
                    .map_err(StructuredWasmError::from)?;
                EntitySignature::capture_vertex(self.topo(), id, quantum)
            }
        }
        .map_err(StructuredWasmError::from)?;
        Self::ref_to_json(&PersistentRef::signature(signature))
    }

    #[cfg(feature = "io")]
    fn add_ref_discriminator_json(
        reference: &str,
        discriminator: &str,
        tag: &str,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let parsed = Self::ref_from_json(reference)?;
        let discriminator = match discriminator {
            "surfaceType" => Discriminator::SurfaceType(tag.to_owned()),
            "curveType" => Discriminator::CurveType(tag.to_owned()),
            other => {
                return Err(StructuredWasmError::invalid_argument(
                    format!("unknown discriminator '{other}' (expected surfaceType or curveType)"),
                    Some("discriminator"),
                ));
            }
        };
        Self::ref_to_json(&parsed.with_discriminator(discriminator))
    }

    #[cfg(feature = "io")]
    fn resolve_ref_json(&self, reference: &str) -> Result<serde_json::Value, StructuredWasmError> {
        let parsed = Self::ref_from_json(reference)?;
        Ok(resolution_json(&resolve(self.topo(), &parsed)))
    }

    #[cfg(feature = "io")]
    fn resolve_ref_face_attributes_json(
        &self,
        reference: &str,
    ) -> Result<serde_json::Value, StructuredWasmError> {
        use brepkit_topology::naming::resolve_face_attributes;
        let parsed = Self::ref_from_json(reference)?;
        let bound =
            resolve_face_attributes(self.topo(), &parsed).map_err(StructuredWasmError::from)?;
        let entries: Vec<serde_json::Value> = bound
            .into_iter()
            .map(|(key, attributes)| {
                serde_json::json!({
                    "kind": key.kind.as_str(),
                    "handle": u32::try_from(key.index).unwrap_or(u32::MAX),
                    "name": attributes.and_then(|a| a.name.clone()),
                })
            })
            .collect();
        Ok(serde_json::Value::Array(entries))
    }

    /// Batch dispatch for the naming ops; `None` when `op` is not a
    /// naming operation.
    #[allow(clippy::redundant_pub_crate)]
    pub(crate) fn dispatch_naming_op(
        &mut self,
        op: &str,
        args: &serde_json::Value,
    ) -> Option<Result<serde_json::Value, StructuredWasmError>> {
        let result = match op {
            "fuseJournaled" | "cutJournaled" | "intersectJournaled" => {
                let bool_op = match op {
                    "fuseJournaled" => BooleanOp::Fuse,
                    "cutJournaled" => BooleanOp::Cut,
                    _ => BooleanOp::Intersect,
                };
                get_u32(args, "solidA").and_then(|a| {
                    get_u32(args, "solidB").and_then(|b| self.journaled_boolean_json(bool_op, a, b))
                })
            }
            "journalBarrier" => get_str(args, "kind").map(str::to_owned).and_then(|kind| {
                get_u32(args, "solid").and_then(|solid| self.journal_barrier_json(&kind, solid))
            }),
            "propagateAttributesForOp" => get_u32(args, "op").and_then(|op_id| {
                let allow_inferred = args["allowInferred"].as_bool().unwrap_or(false);
                self.propagate_attributes_json(op_id, allow_inferred)
            }),
            "resolveOperationOutput" => get_u32(args, "op").and_then(|op_id| {
                get_str(args, "kind").map(str::to_owned).and_then(|kind| {
                    get_u32(args, "index")
                        .and_then(|index| self.resolve_operation_output_json(op_id, &kind, index))
                })
            }),
            "setFaceName" => get_u32(args, "face").and_then(|face| {
                let name = args["name"].as_str();
                self.set_face_name_json(face, name)
            }),
            "getFaceName" => get_u32(args, "face").and_then(|face| self.get_face_name_json(face)),
            #[cfg(feature = "io")]
            "makeOperationOutputRef" => get_u32(args, "op").and_then(|op_id| {
                get_str(args, "kind").map(str::to_owned).and_then(|kind| {
                    get_u32(args, "index").and_then(|index| {
                        let reference = PersistentRef::operation_output(
                            OpId::from_value(u64::from(op_id)),
                            parse_entity_kind(&kind)?,
                            index as usize,
                        );
                        Self::ref_to_json(&reference)
                    })
                })
            }),
            #[cfg(feature = "io")]
            "captureSignatureRef" => get_str(args, "kind").map(str::to_owned).and_then(|kind| {
                get_u32(args, "handle").and_then(|handle| {
                    let quantum = args["quantum"].as_f64().unwrap_or(1e-7);
                    self.capture_signature_ref_json(&kind, handle, quantum)
                })
            }),
            #[cfg(feature = "io")]
            "addRefDiscriminator" => get_str(args, "ref").map(str::to_owned).and_then(|r| {
                get_str(args, "discriminator")
                    .map(str::to_owned)
                    .and_then(|d| {
                        get_str(args, "tag")
                            .and_then(|tag| Self::add_ref_discriminator_json(&r, &d, tag))
                    })
            }),
            #[cfg(feature = "io")]
            "resolveRef" => get_str(args, "ref")
                .map(str::to_owned)
                .and_then(|r| self.resolve_ref_json(&r)),
            #[cfg(feature = "io")]
            "resolveRefFaceAttributes" => get_str(args, "ref")
                .map(str::to_owned)
                .and_then(|r| self.resolve_ref_face_attributes_json(&r)),
            _ => return None,
        };
        Some(result)
    }
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Persistent naming (RFC 0003) ────────────────────────────────

    /// Fuse two solids with journaled construction history.
    ///
    /// Returns JSON `{"solid": handle, "op": journalOp}`; feed `op` to
    /// `resolveOperationOutput` / `propagateAttributesForOp`.
    #[wasm_bindgen(js_name = "fuseJournaled")]
    pub fn fuse_journaled(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.journaled_boolean_json(BooleanOp::Fuse, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Cut solid `b` from `a` with journaled construction history.
    #[wasm_bindgen(js_name = "cutJournaled")]
    pub fn cut_journaled(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.journaled_boolean_json(BooleanOp::Cut, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Intersect two solids with journaled construction history.
    #[wasm_bindgen(js_name = "intersectJournaled")]
    pub fn intersect_journaled(&mut self, a: u32, b: u32) -> Result<String, JsError> {
        self.journaled_boolean_json(BooleanOp::Intersect, a, b)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Journals an explicit barrier over every entity of `solid` for an
    /// operation without evolution records. Returns the journal op id.
    #[wasm_bindgen(js_name = "journalBarrier")]
    pub fn journal_barrier(&mut self, kind: &str, solid: u32) -> Result<u32, JsError> {
        let value = self
            .journal_barrier_json(kind, solid)
            .map_err(structured_to_js)?;
        u32::try_from(value["op"].as_u64().unwrap_or(u64::MAX))
            .map_err(|_| JsError::new("journal op id exceeds the u32 range"))
    }

    /// Propagates face attributes across one journaled operation.
    ///
    /// Returns JSON `{"carried", "unresolvedOutputs", "mergeConflicts",
    /// "refusedInferred"}`.
    #[wasm_bindgen(js_name = "propagateAttributesForOp")]
    pub fn propagate_attributes_for_op_js(
        &mut self,
        op: u32,
        allow_inferred: bool,
    ) -> Result<String, JsError> {
        self.propagate_attributes_json(op, allow_inferred)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Resolves "the `index`-th `kind` output of journal operation `op`"
    /// against the current model. Returns the resolution JSON (`status`
    /// plus status-specific fields); severed references are data, not
    /// errors.
    #[wasm_bindgen(js_name = "resolveOperationOutput")]
    pub fn resolve_operation_output(
        &self,
        op: u32,
        kind: &str,
        index: u32,
    ) -> Result<String, JsError> {
        self.resolve_operation_output_json(op, kind, index)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Sets (or clears, when `name` is null/empty) a face's semantic
    /// name, preserving its other attributes.
    #[wasm_bindgen(js_name = "setFaceName")]
    pub fn set_face_name(&mut self, face: u32, name: Option<String>) -> Result<(), JsError> {
        self.set_face_name_json(face, name.as_deref())
            .map(|_| ())
            .map_err(structured_to_js)
    }

    /// A face's semantic name, or null.
    #[wasm_bindgen(js_name = "getFaceName")]
    pub fn get_face_name(&self, face: u32) -> Result<Option<String>, JsError> {
        self.get_face_name_json(face)
            .map(|v| v.as_str().map(str::to_owned))
            .map_err(structured_to_js)
    }
}

#[cfg(feature = "io")]
#[wasm_bindgen]
impl BrepKernel {
    /// Serializes "the `index`-th `kind` output of journal operation
    /// `op`" as a portable reference string (versioned JSON, opaque).
    #[wasm_bindgen(js_name = "makeOperationOutputRef")]
    // Instance method on purpose: every kernel API is called on the
    // kernel instance in JS, and a future variant may validate against
    // the journal.
    #[allow(clippy::unused_self)]
    pub fn make_operation_output_ref(
        &self,
        op: u32,
        kind: &str,
        index: u32,
    ) -> Result<String, JsError> {
        let kind = parse_entity_kind(kind).map_err(structured_to_js)?;
        let reference =
            PersistentRef::operation_output(OpId::from_value(u64::from(op)), kind, index as usize);
        Self::ref_to_json(&reference)
            .map(|v| v["ref"].as_str().unwrap_or_default().to_owned())
            .map_err(structured_to_js)
    }

    /// Captures an entity's geometric signature as a portable reference
    /// string — the inference-tier recovery anchor. `quantum` is the
    /// tolerance-derived quantization (pass the model's linear
    /// tolerance).
    #[wasm_bindgen(js_name = "captureSignatureRef")]
    pub fn capture_signature_ref(
        &self,
        kind: &str,
        handle: u32,
        quantum: f64,
    ) -> Result<String, JsError> {
        self.capture_signature_ref_json(kind, handle, quantum)
            .map(|v| v["ref"].as_str().unwrap_or_default().to_owned())
            .map_err(structured_to_js)
    }

    /// Returns a copy of the reference with a type discriminator
    /// (`"surfaceType"` or `"curveType"`) appended.
    #[wasm_bindgen(js_name = "addRefDiscriminator")]
    // Instance method on purpose; see makeOperationOutputRef.
    #[allow(clippy::unused_self)]
    pub fn add_ref_discriminator(
        &self,
        reference: &str,
        discriminator: &str,
        tag: &str,
    ) -> Result<String, JsError> {
        Self::add_ref_discriminator_json(reference, discriminator, tag)
            .map(|v| v["ref"].as_str().unwrap_or_default().to_owned())
            .map_err(structured_to_js)
    }

    /// Resolves a serialized reference against the current model.
    /// Returns the resolution JSON; severed references are data, not
    /// errors.
    #[wasm_bindgen(js_name = "resolveRef")]
    pub fn resolve_ref(&self, reference: &str) -> Result<String, JsError> {
        self.resolve_ref_json(reference)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }

    /// Resolves a serialized reference and reads the bound faces'
    /// attributes: JSON array of `{"kind", "handle", "name"}`. Errors on
    /// non-binding resolutions (`ref_*` diagnostics) — an attribute is
    /// never read through a dangling, severed, or ambiguous reference.
    #[wasm_bindgen(js_name = "resolveRefFaceAttributes")]
    pub fn resolve_ref_face_attributes(&self, reference: &str) -> Result<String, JsError> {
        self.resolve_ref_face_attributes_json(reference)
            .map(|v| v.to_string())
            .map_err(structured_to_js)
    }
}

fn structured_to_js(error: StructuredWasmError) -> JsError {
    JsError::new(error.message())
}

#[cfg(test)]
mod naming_contract_tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

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

    /// The full journey: name a face, run a journaled fuse, propagate,
    /// resolve through the journal, and read the name back through a
    /// serialized reference.
    // Exercises the io-gated serialized-reference ops; those batch
    // operations are not registered in a no-default-features build.
    #[cfg(feature = "io")]
    #[test]
    fn names_survive_a_journaled_fuse_end_to_end() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "makeBox", "args": {"width": 10.0, "height": 10.0, "depth": 10.0}},
                {"op": "transform", "args": {"solid": 1, "matrix":
                    [1.0,0.0,0.0,5.0, 0.0,1.0,0.0,5.0, 0.0,0.0,1.0,5.0, 0.0,0.0,0.0,1.0]}},
                {"op": "setFaceName", "args": {"face": 0, "name": "datum face"}},
                {"op": "getFaceName", "args": {"face": 0}},
            ]),
        );
        assert_eq!(results[4], serde_json::json!("datum face"));

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "fuseJournaled", "args": {"solidA": 0, "solidB": 1}},
            ]),
        );
        let journal_op = results[0]["op"].as_u64().expect("journal op id");
        assert!(results[0]["solid"].as_u64().is_some());

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "propagateAttributesForOp", "args": {"op": journal_op}},
                {"op": "resolveOperationOutput",
                 "args": {"op": journal_op, "kind": "face", "index": 0}},
                {"op": "makeOperationOutputRef",
                 "args": {"op": journal_op, "kind": "face", "index": 0}},
            ]),
        );
        assert!(
            results[0]["carried"].as_u64().unwrap_or(0) > 0,
            "the named face must ride the fuse: {}",
            results[0]
        );
        assert!(!results[0]["refusedInferred"].as_bool().unwrap());
        assert_eq!(results[1]["status"], "bound");
        assert_eq!(results[1]["provenance"], "construction");
        assert_eq!(results[1]["entities"][0]["kind"], "face");

        // The serialized reference resolves identically, and every face
        // output's attribute read agrees with the journal.
        let reference = results[2]["ref"].as_str().expect("ref string").to_owned();
        let resolved_direct = results[1].clone();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "resolveRef", "args": {"ref": reference}},
            ]),
        );
        assert_eq!(results[0], resolved_direct);

        // At least one output reference reads a carried, unmodified name.
        let mut named = 0;
        for index in 0..12 {
            let results = run(
                &mut kernel,
                serde_json::json!([
                    {"op": "makeOperationOutputRef",
                     "args": {"op": journal_op, "kind": "face", "index": index}},
                ]),
            );
            let reference = results[0]["ref"].as_str().unwrap().to_owned();
            let response = kernel.execute_batch(
                &serde_json::json!([
                    {"op": "resolveRefFaceAttributes", "args": {"ref": reference}},
                ])
                .to_string(),
            );
            let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
            if let Some(entries) = parsed[0]["ok"].as_array() {
                for entry in entries {
                    if entry["name"] == serde_json::json!("datum face") {
                        named += 1;
                    }
                }
            }
        }
        assert!(
            named > 0,
            "a named face must be readable through a reference"
        );
    }

    /// A barrier severs journal resolution, honestly and typed.
    #[test]
    fn barriers_sever_resolution_as_data_not_errors() {
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
        let journal_op = results[3]["op"].as_u64().unwrap();
        let fused = results[3]["solid"].as_u64().unwrap();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "journalBarrier", "args": {"kind": "offset_solid", "solid": fused}},
                {"op": "resolveOperationOutput",
                 "args": {"op": journal_op, "kind": "face", "index": 0}},
                {"op": "resolveOperationOutput",
                 "args": {"op": 999, "kind": "face", "index": 0}},
            ]),
        );
        assert_eq!(results[1]["status"], "unresolvedAcrossOperation");
        assert_eq!(results[1]["operationKind"], "offset_solid");
        assert_eq!(results[2]["status"], "unknownOperation");
    }

    /// Signature references: capture, discriminate, resolve — inferred.
    // Exercises the io-gated serialized-reference ops; those batch
    // operations are not registered in a no-default-features build.
    #[cfg(feature = "io")]
    #[test]
    fn signature_refs_capture_and_discriminate() {
        let mut kernel = BrepKernel::new();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "makeBox", "args": {"width": 10.0, "height": 20.0, "depth": 30.0}},
                {"op": "captureSignatureRef", "args": {"kind": "face", "handle": 0}},
            ]),
        );
        let reference = results[1]["ref"].as_str().unwrap().to_owned();

        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "resolveRef", "args": {"ref": reference}},
                {"op": "addRefDiscriminator",
                 "args": {"ref": reference, "discriminator": "surfaceType", "tag": "cylinder"}},
            ]),
        );
        assert_eq!(results[0]["status"], "bound");
        assert_eq!(results[0]["provenance"], "inferred");
        assert_eq!(results[0]["entities"][0]["handle"], 0);

        let discriminated = results[1]["ref"].as_str().unwrap().to_owned();
        let results = run(
            &mut kernel,
            serde_json::json!([
                {"op": "resolveRef", "args": {"ref": discriminated}},
            ]),
        );
        assert_eq!(results[0]["status"], "noMatch", "{}", results[0]);
    }
}
