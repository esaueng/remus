//! Facade half of the extended O1.5 per-operation parity harness.
//!
//! The JS driver (`parity-matrix.mjs`) emits one `executeBatchV2`-shaped case
//! per cell and feeds it to this binary on stdin. This runner executes the
//! same case through the **native facade** (`remus::Model`) — the surface the
//! task names — rather than through a natively-compiled WASM kernel: the
//! batch op list is dispatched onto `Model` methods (plus the underlying
//! `operations` entry points where the facade has no method yet: shell,
//! offset, draft, helical sweep), while measurement, validation, census,
//! carrier histogram, mesh quality, and arena serialization are read back
//! through the same facade topology.
//!
//! Handle references (`{fromOp}`) resolve against earlier case responses,
//! mirroring the WASM driver's chunked dispatch. A failed op records its
//! disclosure string without stopping later ops — the `executeBatchV2`
//! envelope contract on both surfaces. The disclosure vocabulary mirrors
//! `StructuredWasmError`'s `From<OperationsError>` mapping in
//! `crates/wasm/src/error.rs` (coarse wire `code`, kernel-wide `category`,
//! stable `kernelCode` diagnostic detail), so `diagnostics_agreement`
//! compares like with like.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::{Display, Formatter};
use std::io::{self, Read};
use std::time::Instant;

use remus::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};

#[derive(Debug)]
struct RunnerError(String);

impl Display for RunnerError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl Error for RunnerError {}

/// A dispatch failure: either a kernel `OperationsError` (mapped through the
/// wire disclosure vocabulary) or a stale-handle resolution failure (mapped
/// to the wire `invalid_handle` code, exactly as the WASM `resolve_*`
/// failures surface).
#[derive(Debug)]
enum DispatchError {
    Ops(OperationsError),
    BadHandle(String),
}

impl Display for DispatchError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Ops(error) => write!(formatter, "{error}"),
            Self::BadHandle(message) => write!(formatter, "{message}"),
        }
    }
}

impl From<OperationsError> for DispatchError {
    fn from(error: OperationsError) -> Self {
        Self::Ops(error)
    }
}

impl From<TopologyError> for DispatchError {
    fn from(error: TopologyError) -> Self {
        Self::Ops(OperationsError::Topology(error))
    }
}

#[derive(Deserialize)]
struct Case {
    schema_version: u32,
    id: String,
    batch: Vec<Value>,
    result: ResultSpec,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ResultSpec {
    handle: HandleRef,
    #[serde(default)]
    op_index: Option<usize>,
    #[serde(default)]
    boolean_index: Option<usize>,
    volume_index: usize,
    validation_index: usize,
    mesh_quality_index: usize,
    faces_index: usize,
}

impl ResultSpec {
    /// The boolean/modifier producing-op index under either schema: schema 2
    /// uses `opIndex`, the first-slice schema 1 uses `booleanIndex`.
    fn producing_op(&self) -> Result<usize, RunnerError> {
        self.op_index
            .or(self.boolean_index)
            .ok_or_else(|| err("result spec is missing opIndex/booleanIndex"))
    }
}

/// A result handle: either a concrete numeric handle (boolean and modifier
/// cells) or a symbolic `{fromOp}` reference to the producing op
/// (sweep-family cells, whose solid arena index depends on how many
/// construction entities precede it).
#[derive(Deserialize)]
#[serde(untagged)]
enum HandleRef {
    Direct(u32),
    Symbolic {
        #[serde(rename = "fromOp")]
        from_op: usize,
    },
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Census {
    faces: u32,
    edges: u32,
    vertices: u32,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Observation {
    schema_version: u32,
    id: String,
    surface: &'static str,
    outcome: &'static str,
    diagnostic_codes: Vec<String>,
    quality: Option<String>,
    result_handle: Option<u32>,
    volume: Option<f64>,
    validation_errors: Option<u64>,
    census: Option<Census>,
    surface_types: Option<BTreeMap<String, u32>>,
    mesh_quality: Option<Value>,
    serialized_bytes: Option<usize>,
    serialized_sha256: Option<String>,
    cold_init_ms: f64,
    batch_duration_ms: f64,
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    rollback_volumes: Vec<f64>,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    rollback_faces: Option<usize>,
    /// The producing op's evolution report (`*WithEvolution` ops only),
    /// passed through verbatim in the WASM wire shape so the contract
    /// scorer compares the same buckets on every surface.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    evolution: Option<Value>,
    /// Every succeeding `volume` response keyed by batch index, so a
    /// contract cell can read an operand-preservation probe beside the
    /// result volume without a second envelope.
    #[serde(skip_serializing_if = "BTreeMap::is_empty", default)]
    volumes_by_index: BTreeMap<usize, f64>,
}

/// Whether a batch op answers with a `{solid, ...}` object rather than a bare
/// handle: the quality-disclosing booleans and the evolution-reporting
/// booleans, exactly as the WASM batch arms shape them.
fn returns_solid_object(op: &str) -> bool {
    matches!(
        op,
        "booleanWithQuality"
            | "booleanWithCancelledContext"
            | "fuseWithEvolution"
            | "cutWithEvolution"
            | "intersectWithEvolution"
    )
}

fn boolean_op_from_name(operation: &str) -> Result<BooleanOp, DispatchError> {
    match operation {
        "fuse" | "union" => Ok(BooleanOp::Fuse),
        "cut" | "difference" => Ok(BooleanOp::Cut),
        "intersect" | "intersection" => Ok(BooleanOp::Intersect),
        _ => Err(ops_err(&format!("unknown boolean operation '{operation}'"))),
    }
}

fn err(message: impl Into<String>) -> RunnerError {
    RunnerError(message.into())
}

fn response_ok<'a>(
    responses: &'a [Value],
    index: usize,
    label: &str,
) -> Result<&'a Value, RunnerError> {
    responses
        .get(index)
        .and_then(|response| response.get("ok"))
        .ok_or_else(|| {
            err(format!(
                "missing successful {label} response at index {index}"
            ))
        })
}

fn diagnostics(responses: &[Value]) -> Vec<String> {
    responses
        .iter()
        .filter_map(|response| response.get("error"))
        .map(|error| {
            error
                .get("code")
                .and_then(Value::as_str)
                .unwrap_or("untyped_error")
                .to_owned()
        })
        .collect()
}

fn sha256_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        output.push(char::from(HEX[usize::from(byte >> 4)]));
        output.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    output
}

fn detail(mut details: serde_json::Map<String, Value>, operation: &str, index: usize) -> Value {
    details.insert("operation".to_owned(), Value::from(operation.to_owned()));
    details.insert(
        "operationIndex".to_owned(),
        Value::from(u64::try_from(index).unwrap_or(u64::MAX)),
    );
    Value::Object(details)
}

fn kernel_code_detail(code: &str) -> serde_json::Map<String, Value> {
    serde_json::json!({"kernelCode": code})
        .as_object()
        .cloned()
        .unwrap_or_default()
}

/// Wire disclosure for one failed batch op, mirroring
/// `StructuredWasmError`'s `From` mapping in `crates/wasm/src/error.rs`.
///
/// The harness compares the coarse envelope `code` in
/// `diagnostics_agreement`; `category` and the stable `kernelCode`
/// diagnostic detail ride along for triage. `TopologyError` and
/// `MathError` implement `ToDiagnostic`, so their kernel codes ride along
/// exactly as on the WASM side; the `OperationsError` arms mirror the
/// crate's `From` mapping by hand (`OperationsError` itself has no
/// `ToDiagnostic` impl).
fn disclosure(operation: &str, index: usize, error: &DispatchError) -> Value {
    match error {
        DispatchError::BadHandle(message) => serde_json::json!({ "error": {
            "code": "invalid_handle",
            "category": "invalid_input",
            "message": message,
            "details": detail(serde_json::Map::new(), operation, index),
        }}),
        DispatchError::Ops(error) => ops_disclosure(operation, index, error),
    }
}

#[allow(clippy::too_many_lines)]
fn ops_disclosure(operation: &str, index: usize, error: &OperationsError) -> Value {
    use remus_math::diagnostic::ToDiagnostic;
    let message = error.to_string();
    let error_value = match error {
        OperationsError::ExactOnlyUnattainable => serde_json::json!({
            "code": "operation_failed",
            "category": "quality_refused",
            "message": message,
            "details": detail(kernel_code_detail("exact_only_unattainable"), operation, index),
        }),
        OperationsError::InvalidInput { .. } => serde_json::json!({
            "code": "invalid_argument",
            "category": "invalid_input",
            "message": message,
            "details": detail(serde_json::Map::new(), operation, index),
        }),
        OperationsError::Topology(inner) => {
            let diagnostic = inner.diagnostic();
            serde_json::json!({
                "code": "topology_error",
                "category": diagnostic.category().as_str(),
                "message": message,
                "details": detail(kernel_code_detail(diagnostic.code()), operation, index),
            })
        }
        OperationsError::Math(inner) => {
            let diagnostic = inner.diagnostic();
            let code = match inner {
                remus_math::MathError::ConvergenceFailure { .. } => "operation_failed",
                remus_math::MathError::Cancelled => "cancelled",
                _ => "invalid_argument",
            };
            serde_json::json!({
                "code": code,
                "category": diagnostic.category().as_str(),
                "message": message,
                "details": detail(kernel_code_detail(diagnostic.code()), operation, index),
            })
        }
        OperationsError::Algo(inner) => {
            let diagnostic = inner.diagnostic();
            serde_json::json!({
                "code": "operation_failed",
                "category": diagnostic.category().as_str(),
                "message": message,
                "details": detail(kernel_code_detail(diagnostic.code()), operation, index),
            })
        }
        OperationsError::Check(inner) => return check_disclosure(operation, index, inner),
        OperationsError::Blend(_) => {
            // Mirror `blend_failure`: the generic wire mapping plus the
            // stable blend code, with the unsupported category only for the
            // face-face variant.
            let code = remus_operations::blend_ops::blend_failure_code(error);
            let category = if code == "unsupported-face-face-blend" {
                "unsupported"
            } else {
                "internal"
            };
            serde_json::json!({
                "code": "operation_failed",
                "category": category,
                "message": message,
                "details": detail(kernel_code_detail(code), operation, index),
            })
        }
        OperationsError::PatternInstancesOverlap { .. } => serde_json::json!({
            "code": "operation_failed",
            "category": "unsupported",
            "message": message,
            "details": detail(kernel_code_detail("pattern_instances_overlap"), operation, index),
        }),
        OperationsError::BodyClassMeasureMismatch { .. } => serde_json::json!({
            "code": "invalid_argument",
            "category": "invalid_input",
            "message": message,
            "details": detail(kernel_code_detail("body_class_measure_mismatch"), operation, index),
        }),
        OperationsError::BodyClassOperationUnsupported { .. } => serde_json::json!({
            "code": "operation_failed",
            "category": "unsupported",
            "message": message,
            "details": detail(kernel_code_detail("body_class_operand_unsupported"), operation, index),
        }),
        // Every remaining variant falls through to the crate's catch-all: a
        // plain operation failure, exactly as `From<OperationsError>`'s
        // trailing `_` arm produces.
        _ => serde_json::json!({
            "code": "operation_failed",
            "category": "internal",
            "message": message,
            "details": detail(serde_json::Map::new(), operation, index),
        }),
    };
    serde_json::json!({ "error": error_value })
}

fn check_disclosure(operation: &str, index: usize, error: &remus_check::CheckError) -> Value {
    use remus_math::diagnostic::ToDiagnostic;
    let message = error.to_string();
    // Mirror `From<CheckError>`: topology/math delegate, everything else is
    // a plain operation failure.
    let error_value = match error {
        remus_check::CheckError::Topology(inner) => {
            let diagnostic = inner.diagnostic();
            serde_json::json!({
                "code": "topology_error",
                "category": diagnostic.category().as_str(),
                "message": message,
                "details": detail(kernel_code_detail(diagnostic.code()), operation, index),
            })
        }
        remus_check::CheckError::Math(inner) => {
            let diagnostic = inner.diagnostic();
            let code = match inner {
                remus_math::MathError::ConvergenceFailure { .. } => "operation_failed",
                remus_math::MathError::Cancelled => "cancelled",
                _ => "invalid_argument",
            };
            serde_json::json!({
                "code": code,
                "category": diagnostic.category().as_str(),
                "message": message,
                "details": detail(kernel_code_detail(diagnostic.code()), operation, index),
            })
        }
        _ => serde_json::json!({
            "code": "operation_failed",
            "category": "internal",
            "message": message,
            "details": detail(serde_json::Map::new(), operation, index),
        }),
    };
    serde_json::json!({ "error": error_value })
}

fn get_f64(args: &Value, key: &str) -> Result<f64, RunnerError> {
    args.get(key)
        .and_then(Value::as_f64)
        .ok_or_else(|| err(format!("missing or invalid '{key}'")))
}

fn get_f64_or(args: &Value, key: &str, default: f64) -> f64 {
    args.get(key).and_then(Value::as_f64).unwrap_or(default)
}

fn get_matrix(args: &Value) -> Result<Mat4, RunnerError> {
    let elems: Vec<f64> = args
        .get("matrix")
        .and_then(Value::as_array)
        .ok_or_else(|| err("missing or invalid 'matrix'"))?
        .iter()
        .map(|v| v.as_f64().ok_or_else(|| err("non-numeric matrix element")))
        .collect::<Result<Vec<_>, _>>()?;
    if elems.len() != 16 {
        return Err(err("transform matrix must have 16 elements"));
    }
    Ok(Mat4([
        [elems[0], elems[1], elems[2], elems[3]],
        [elems[4], elems[5], elems[6], elems[7]],
        [elems[8], elems[9], elems[10], elems[11]],
        [elems[12], elems[13], elems[14], elems[15]],
    ]))
}

/// Resolve one `{fromOp}` reference against completed ok-values.
fn resolve_ref(value: &Value, ok_values: &[Value]) -> Result<Value, RunnerError> {
    let Some(from) = value.get("fromOp").and_then(Value::as_u64) else {
        return Ok(value.clone());
    };
    let source = ok_values
        .get(from as usize)
        .ok_or_else(|| err(format!("fromOp {from} out of range")))?
        .clone();
    if let Some(pick) = value.get("pick").and_then(Value::as_str) {
        let list = source
            .as_array()
            .ok_or_else(|| err("pick source must be a handle list"))?;
        match pick {
            "first" => list
                .first()
                .cloned()
                .ok_or_else(|| err("empty handle list")),
            "none" => Ok(Value::Array(Vec::new())),
            // `top`/`open-top` narrow below once face normals are readable.
            _ => Ok(Value::Array(list.clone())),
        }
    } else if value.get("pickNormal").is_some() {
        Err(err("pickNormal must resolve through the model driver"))
    } else {
        Ok(source)
    }
}

#[allow(clippy::too_many_lines)]
fn resolve_args(args: &Value, ok_values: &[Value], model: &Model) -> Result<Value, RunnerError> {
    let mut out = args.clone();
    let map = out
        .as_object_mut()
        .ok_or_else(|| err("args must be an object"))?;
    // Loft carries its face list as a direct array of `{fromOp}` refs, and
    // makeWire carries its edge list the same way.
    for key in ["faces", "edges"] {
        if let Some(Value::Array(list)) = map.get(key).cloned() {
            let mut resolved: Vec<Value> = Vec::with_capacity(list.len());
            for entry in &list {
                if entry.get("fromOp").is_some() {
                    let source = resolve_ref(entry, ok_values)?;
                    // `{fromOp}` on a single handle resolves to the bare
                    // value; a list source flattens (makeWire edges).
                    if key == "edges" && source.is_array() {
                        resolved.push(source);
                    } else if let Some(list) = source.as_array() {
                        return Err(err(format!(
                            "unexpected list source with {} entries",
                            list.len()
                        )));
                    } else {
                        resolved.push(source);
                    }
                } else {
                    resolved.push(entry.clone());
                }
            }
            // makeWire edge refs flatten one level.
            if key == "edges" {
                let flat: Vec<Value> = resolved
                    .into_iter()
                    .flat_map(|entry| match entry {
                        Value::Array(items) => items,
                        single => vec![single],
                    })
                    .collect();
                map.insert(key.to_owned(), Value::Array(flat));
            } else {
                map.insert(key.to_owned(), Value::Array(resolved));
            }
        }
    }
    if let Some(Value::Object(obj)) = map.get("edges").cloned()
        && obj.contains_key("fromOp")
    {
        let edges = map.get("edges").cloned().unwrap_or(Value::Null);
        let resolved = resolve_ref(&edges, ok_values)?;
        // `pick: first` on solidEdges yields the one-element list the
        // blend entry points expect.
        if let Some(list) = resolved.as_array() {
            map.insert("edges".to_owned(), Value::Array(list.clone()));
        } else {
            map.insert("edges".to_owned(), Value::Array(vec![resolved]));
        }
    }
    for key in ["face", "profile", "pathEdge", "wire", "solid"] {
        if let Some(entry) = map.get(key).cloned()
            && entry.get("fromOp").is_some()
        {
            map.insert(key.to_owned(), resolve_ref(&entry, ok_values)?);
        }
    }
    // Deferred selectors needing kernel queries.
    if let Some(Value::Object(obj)) = map.get("faces").cloned() {
        if let Some(pick) = obj.get("pick").and_then(Value::as_str) {
            let source = obj
                .get("fromOp")
                .and_then(Value::as_u64)
                .and_then(|i| ok_values.get(i as usize))
                .and_then(Value::as_array)
                .ok_or_else(|| err("pick source must be a handle list"))?;
            let picked = match pick {
                "none" => Vec::new(),
                "open-top" | "top" => vec![pick_open_top(model, source)?],
                _ => {
                    return Err(err(format!("unknown face pick '{pick}'")));
                }
            };
            map.insert(
                "faces".to_owned(),
                Value::Array(
                    picked
                        .into_iter()
                        .map(|h| Value::from(u64::from(h)))
                        .collect(),
                ),
            );
        } else if let Some(target) = obj.get("pickNormal") {
            let target: Vec<f64> = target
                .as_array()
                .ok_or_else(|| err("pickNormal must be a 3-vector"))?
                .iter()
                .map(|v| v.as_f64().ok_or_else(|| err("non-numeric pickNormal")))
                .collect::<Result<Vec<_>, _>>()?;
            if target.len() != 3 {
                return Err(err("pickNormal must be a 3-vector"));
            }
            let source = obj
                .get("fromOp")
                .and_then(Value::as_u64)
                .and_then(|i| ok_values.get(i as usize))
                .and_then(Value::as_array)
                .ok_or_else(|| err("pickNormal source must be a handle list"))?;
            let picked = pick_by_normal(model, source, Vec3::new(target[0], target[1], target[2]))?;
            map.insert(
                "faces".to_owned(),
                Value::Array(vec![Value::from(u64::from(picked))]),
            );
        }
    }
    Ok(out)
}

fn resolve_face(model: &Model, handle: u32) -> Result<FaceId, DispatchError> {
    model
        .topology()
        .face_id_from_index(handle as usize)
        .ok_or_else(|| DispatchError::BadHandle(format!("stale face handle {handle}")))
}

fn resolve_solid(model: &Model, handle: u32) -> Result<SolidId, DispatchError> {
    model
        .topology()
        .solid_id_from_index(handle as usize)
        .ok_or_else(|| DispatchError::BadHandle(format!("stale solid handle {handle}")))
}

fn resolve_edge(model: &Model, handle: u32) -> Result<EdgeId, DispatchError> {
    model
        .topology()
        .edge_id_from_index(handle as usize)
        .ok_or_else(|| DispatchError::BadHandle(format!("stale edge handle {handle}")))
}

fn resolve_wire(model: &Model, handle: u32) -> Result<WireId, DispatchError> {
    model
        .topology()
        .wire_id_from_index(handle as usize)
        .ok_or_else(|| DispatchError::BadHandle(format!("stale wire handle {handle}")))
}

fn face_normal(model: &Model, face: u32) -> Result<Vec3, RunnerError> {
    use remus_topology::face::FaceSurface;
    let fid = model
        .topology()
        .face_id_from_index(face as usize)
        .ok_or_else(|| err("face handle is stale"))?;
    let normal = match model.topology().face(fid) {
        Ok(face_data) => match face_data.surface() {
            FaceSurface::Plane { normal, .. } => *normal,
            other => other.normal(0.0, 0.0),
        },
        Err(_) => return Err(err("face handle is stale")),
    };
    Ok(normal)
}

fn pick_open_top(model: &Model, faces: &[Value]) -> Result<u32, RunnerError> {
    let mut best = None;
    let mut best_dot = f64::NEG_INFINITY;
    for face in faces {
        let handle = face
            .as_u64()
            .ok_or_else(|| err("face handle must be an integer"))?;
        let handle_u32 = u32::try_from(handle).map_err(|_| err("face handle exceeds u32"))?;
        let normal = face_normal(model, handle_u32)?;
        if normal.z() > best_dot {
            best_dot = normal.z();
            best = Some(handle_u32);
        }
    }
    let picked = best.ok_or_else(|| err("empty face list"))?;
    if best_dot < 0.99 {
        return Err(err("no top face for open shell"));
    }
    Ok(picked)
}

fn pick_by_normal(model: &Model, faces: &[Value], target: Vec3) -> Result<u32, RunnerError> {
    let mut best = None;
    let mut best_dot = f64::NEG_INFINITY;
    for face in faces {
        let handle = face
            .as_u64()
            .ok_or_else(|| err("face handle must be an integer"))?;
        let handle_u32 = u32::try_from(handle).map_err(|_| err("face handle exceeds u32"))?;
        let normal = face_normal(model, handle_u32)?;
        let dot = normal.x() * target.x() + normal.y() * target.y() + normal.z() * target.z();
        if dot > best_dot {
            best_dot = dot;
            best = Some(handle_u32);
        }
    }
    let picked = best.ok_or_else(|| err("empty face list"))?;
    if best_dot < 0.99 {
        return Err(err("no wall matching normal"));
    }
    Ok(picked)
}

fn as_solid(model: &Model, value: &Value) -> Result<SolidId, DispatchError> {
    let handle = value
        .as_u64()
        .ok_or_else(|| DispatchError::BadHandle("solid handle must be an integer".to_owned()))?;
    let handle_u32 = u32::try_from(handle)
        .map_err(|_| DispatchError::BadHandle(format!("solid handle {handle} exceeds u32")))?;
    resolve_solid(model, handle_u32)
}

fn as_face(model: &Model, value: &Value) -> Result<FaceId, DispatchError> {
    let handle = value
        .as_u64()
        .ok_or_else(|| DispatchError::BadHandle("face handle must be an integer".to_owned()))?;
    let handle_u32 = u32::try_from(handle)
        .map_err(|_| DispatchError::BadHandle(format!("face handle {handle} exceeds u32")))?;
    resolve_face(model, handle_u32)
}

fn as_edge(model: &Model, value: &Value) -> Result<EdgeId, DispatchError> {
    let handle = value
        .as_u64()
        .ok_or_else(|| DispatchError::BadHandle("edge handle must be an integer".to_owned()))?;
    let handle_u32 = u32::try_from(handle)
        .map_err(|_| DispatchError::BadHandle(format!("edge handle {handle} exceeds u32")))?;
    resolve_edge(model, handle_u32)
}

fn as_wire(model: &Model, value: &Value) -> Result<WireId, DispatchError> {
    let handle = value
        .as_u64()
        .ok_or_else(|| DispatchError::BadHandle("wire handle must be an integer".to_owned()))?;
    let handle_u32 = u32::try_from(handle)
        .map_err(|_| DispatchError::BadHandle(format!("wire handle {handle} exceeds u32")))?;
    resolve_wire(model, handle_u32)
}

fn solid_handle(id: SolidId) -> Value {
    Value::from(u64::from(id.index() as u32))
}

fn face_handle(id: FaceId) -> Value {
    Value::from(u64::from(id.index() as u32))
}

fn wire_handle(id: WireId) -> Value {
    Value::from(u64::from(id.index() as u32))
}

fn edge_handle(id: EdgeId) -> Value {
    Value::from(u64::from(id.index() as u32))
}

fn ops_err(message: &str) -> DispatchError {
    DispatchError::Ops(OperationsError::InvalidInput {
        reason: message.to_owned(),
    })
}

#[allow(clippy::too_many_lines)]
fn dispatch(
    model: &mut Model,
    op: &str,
    args: &Value,
    quality: &mut Option<String>,
) -> Result<Value, DispatchError> {
    match op {
        "makeBox" => {
            let solid = model.make_box(
                get_f64(args, "width").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "height").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "depth").map_err(|e| ops_err(&e.0))?,
            )?;
            Ok(solid_handle(solid))
        }
        "makeCylinder" => {
            let solid = model.make_cylinder(
                get_f64(args, "radius").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "height").map_err(|e| ops_err(&e.0))?,
            )?;
            Ok(solid_handle(solid))
        }
        "makeCone" => {
            let solid = model.make_cone(
                get_f64(args, "bottomRadius").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "topRadius").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "height").map_err(|e| ops_err(&e.0))?,
            )?;
            Ok(solid_handle(solid))
        }
        "makeSphere" => {
            let segments = args.get("segments").and_then(Value::as_u64).unwrap_or(16);
            let segments = usize::try_from(segments).unwrap_or(usize::MAX);
            let solid = model.make_sphere(
                get_f64(args, "radius").map_err(|e| ops_err(&e.0))?,
                segments,
            )?;
            Ok(solid_handle(solid))
        }
        "makeTorus" => {
            let segments = args.get("segments").and_then(Value::as_u64).unwrap_or(16);
            let segments = usize::try_from(segments).unwrap_or(usize::MAX);
            let solid = model.make_torus(
                get_f64(args, "majorRadius").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "minorRadius").map_err(|e| ops_err(&e.0))?,
                segments,
            )?;
            Ok(solid_handle(solid))
        }
        "transform" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let matrix = get_matrix(args).map_err(|e| ops_err(&e.0))?;
            model.transform(solid, &matrix)?;
            Ok(solid_handle(solid))
        }
        "transformFace" => {
            let face = as_face(
                model,
                args.get("face").ok_or_else(|| ops_err("missing 'face'"))?,
            )?;
            let matrix = get_matrix(args).map_err(|e| ops_err(&e.0))?;
            remus_operations::transform::transform_face(model.topology_mut(), face, &matrix)?;
            Ok(Value::Null)
        }
        "makeLineEdge" => {
            let tolerance = model.context().tolerance.linear;
            let edge = remus_topology::builder::make_line_edge(
                model.topology_mut(),
                Point3::new(
                    get_f64(args, "x1").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "y1").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "z1").map_err(|e| ops_err(&e.0))?,
                ),
                Point3::new(
                    get_f64(args, "x2").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "y2").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "z2").map_err(|e| ops_err(&e.0))?,
                ),
                tolerance,
            )?;
            Ok(edge_handle(edge))
        }
        "makeWire" => {
            let edges: Vec<EdgeId> = args
                .get("edges")
                .and_then(Value::as_array)
                .ok_or_else(|| ops_err("missing or invalid 'edges'"))?
                .iter()
                .map(|v| as_edge(model, v))
                .collect::<Result<Vec<_>, _>>()?;
            let closed = args.get("closed").and_then(Value::as_bool).unwrap_or(true);
            // Same weld-and-orient discipline as the WASM `makeWire` batch
            // arm (`make_wire_impl` in `bindings/shapes.rs`): consecutive
            // line edges share vertices within linear tolerance before the
            // wire is built.
            let tolerance = model.context().tolerance.linear;
            let merged = edges;
            if merged.len() > 1 {
                for i in 0..merged.len() {
                    let next = if i + 1 < merged.len() {
                        i + 1
                    } else if closed {
                        0
                    } else {
                        continue;
                    };
                    if next == i {
                        continue;
                    }
                    let (end_vertex, start_vertex) = {
                        let end = model
                            .topology()
                            .edge(merged[i])
                            .map_err(OperationsError::from)?
                            .end();
                        let start = model
                            .topology()
                            .edge(merged[next])
                            .map_err(OperationsError::from)?
                            .start();
                        (end, start)
                    };
                    if end_vertex == start_vertex {
                        continue;
                    }
                    let (end_pos, start_pos) = {
                        let end_pos = model
                            .topology()
                            .vertex(end_vertex)
                            .map_err(OperationsError::from)?
                            .point();
                        let start_pos = model
                            .topology()
                            .vertex(start_vertex)
                            .map_err(OperationsError::from)?
                            .point();
                        (end_pos, start_pos)
                    };
                    if (end_pos - start_pos).length() < tolerance {
                        model
                            .topology_mut()
                            .edge_mut(merged[next])
                            .map_err(OperationsError::from)?
                            .set_start(end_vertex);
                    }
                }
            }
            let oriented: Vec<remus_topology::OrientedEdge> = merged
                .iter()
                .map(|edge| remus_topology::OrientedEdge::new(*edge, true))
                .collect();
            let wire = remus_topology::wire::Wire::new(oriented, closed)?;
            let wire = model.topology_mut().add_wire(wire);
            Ok(wire_handle(wire))
        }
        "makePlanarFaceFromWire" => {
            let wire = as_wire(
                model,
                args.get("wire").ok_or_else(|| ops_err("missing 'wire'"))?,
            )?;
            let face =
                remus_topology::builder::make_planar_face_from_wire(model.topology_mut(), wire)?;
            Ok(face_handle(face))
        }
        "booleanWithQuality" => {
            let operation = args
                .get("operation")
                .and_then(Value::as_str)
                .ok_or_else(|| ops_err("missing or invalid 'operation'"))?;
            let boolean_op = boolean_op_from_name(operation)?;
            let a = as_solid(
                model,
                args.get("solidA")
                    .ok_or_else(|| ops_err("missing 'solidA'"))?,
            )?;
            let b = as_solid(
                model,
                args.get("solidB")
                    .ok_or_else(|| ops_err("missing 'solidB'"))?,
            )?;
            // Honor the batch `exactOnly` flag exactly as the WASM
            // `booleanWithQuality` arm does: omitted/null means the default
            // allow-approximate policy, `true` selects the exact-only
            // refusal. Ignoring it would let the native side disclose
            // `approximate` where the WASM side refuses, breaking the
            // contract matrix's refusal/approximation cells.
            let exact_only = match args.get("exactOnly") {
                None | Some(Value::Null) => false,
                Some(value) => value
                    .as_bool()
                    .ok_or_else(|| ops_err("invalid 'exactOnly': expected boolean"))?,
            };
            let outcome = if exact_only {
                let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
                boolean_with_context(model.topology_mut(), boolean_op, a, b, &context)?
            } else {
                model.boolean(boolean_op, a, b)?
            };
            *quality = Some(match outcome.quality {
                BooleanQuality::Exact => "exact".to_owned(),
                BooleanQuality::Approximate { .. } => "approximate".to_owned(),
            });
            Ok(serde_json::json!({
                "solid": u64::from(outcome.solid.index() as u32),
                "quality": quality.clone(),
            }))
        }
        "booleanWithCancelledContext" => {
            // Contract-matrix cancellation probe: runs the boolean under a
            // pre-cancelled cooperative token, mirroring the WASM direct
            // `booleanWithCancellation` path with `token.cancel()` called
            // before the call. A synchronous WASM call cannot process a
            // later JS cancellation message on the same thread, so only
            // the pre-cancelled scope is asserted on either surface.
            let operation = args
                .get("operation")
                .and_then(Value::as_str)
                .ok_or_else(|| ops_err("missing or invalid 'operation'"))?;
            let boolean_op = boolean_op_from_name(operation)?;
            let a = as_solid(
                model,
                args.get("solidA")
                    .ok_or_else(|| ops_err("missing 'solidA'"))?,
            )?;
            let b = as_solid(
                model,
                args.get("solidB")
                    .ok_or_else(|| ops_err("missing 'solidB'"))?,
            )?;
            let token = CancellationToken::new();
            token.cancel();
            let context = OperationContext::new().with_cancellation(token);
            let outcome = boolean_with_context(model.topology_mut(), boolean_op, a, b, &context)?;
            *quality = Some(match outcome.quality {
                BooleanQuality::Exact => "exact".to_owned(),
                BooleanQuality::Approximate { .. } => "approximate".to_owned(),
            });
            Ok(serde_json::json!({
                "solid": u64::from(outcome.solid.index() as u32),
                "quality": quality.clone(),
            }))
        }
        "fuseWithEvolution" | "cutWithEvolution" | "intersectWithEvolution" => {
            // Same entry point and wire shape as the WASM batch arm
            // (`bindings/batch.rs`): the exact-only boolean with a
            // construction-derived evolution report, serialized through
            // `EvolutionMap::to_json` so the buckets (`modified`,
            // `generated`, `deleted`, `unresolved`, `origin`) compare like
            // with like across surfaces.
            let boolean_op = match op {
                "fuseWithEvolution" => BooleanOp::Fuse,
                "cutWithEvolution" => BooleanOp::Cut,
                _ => BooleanOp::Intersect,
            };
            let a = as_solid(
                model,
                args.get("solidA")
                    .ok_or_else(|| ops_err("missing 'solidA'"))?,
            )?;
            let b = as_solid(
                model,
                args.get("solidB")
                    .ok_or_else(|| ops_err("missing 'solidB'"))?,
            )?;
            let (result, evolution) = remus_operations::boolean::boolean_with_evolution(
                model.topology_mut(),
                boolean_op,
                a,
                b,
            )?;
            let evolution: Value = serde_json::from_str(&evolution.to_json())
                .map_err(|e| ops_err(&format!("evolution report is not JSON: {e}")))?;
            // The evolution path never approximates (a mesh result has no
            // construction history), so a success is exact by contract.
            *quality = Some("exact".to_owned());
            Ok(serde_json::json!({
                "solid": solid_handle(result),
                "evolution": evolution,
            }))
        }
        "fillet" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let edges: Vec<EdgeId> = args
                .get("edges")
                .and_then(Value::as_array)
                .ok_or_else(|| ops_err("missing or invalid 'edges'"))?
                .iter()
                .map(|v| as_edge(model, v))
                .collect::<Result<Vec<_>, _>>()?;
            // The batch `fillet` binding runs the same cascade the facade
            // exposes (`fillet_cascade` via `fillet_whole_selection`), so the
            // facade call is the parity surface here, not a divergent engine.
            let result = model.fillet(
                solid,
                &edges,
                get_f64(args, "radius").map_err(|e| ops_err(&e.0))?,
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result.solid))
        }
        "chamfer" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let edges: Vec<EdgeId> = args
                .get("edges")
                .and_then(Value::as_array)
                .ok_or_else(|| ops_err("missing or invalid 'edges'"))?
                .iter()
                .map(|v| as_edge(model, v))
                .collect::<Result<Vec<_>, _>>()?;
            // The batch `chamfer` binding runs the v1-then-v2 `try_chamfer`
            // chain; mirror it here rather than the facade's v2-only path.
            // `try_chamfer` lives in the WASM crate, which the facade runner
            // must not depend on (layer rules), so replay its two legs in
            // order: v1 bevel first, then the same `chamfer_v2` the facade
            // calls, keeping the first success.
            let distance = get_f64(args, "distance").map_err(|e| ops_err(&e.0))?;
            let snapshot = model.topology().clone();
            let outcome =
                remus_operations::chamfer::chamfer(model.topology_mut(), solid, &edges, distance);
            match outcome {
                Ok(result) => {
                    *quality = Some("exact".to_owned());
                    Ok(solid_handle(result))
                }
                Err(first) => {
                    *model.topology_mut() = snapshot;
                    let result = remus_operations::blend_ops::chamfer_v2(
                        model.topology_mut(),
                        solid,
                        &edges,
                        distance,
                        distance,
                    )
                    .map_err(|_| first)?;
                    *quality = Some("exact".to_owned());
                    Ok(solid_handle(result.solid))
                }
            }
        }
        "shell" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let faces: Vec<FaceId> = args
                .get("faces")
                .and_then(Value::as_array)
                .ok_or_else(|| ops_err("missing or invalid 'faces'"))?
                .iter()
                .map(|v| as_face(model, v))
                .collect::<Result<Vec<_>, _>>()?;
            let result = remus_operations::shell_op::shell(
                model.topology_mut(),
                solid,
                get_f64(args, "thickness").map_err(|e| ops_err(&e.0))?,
                &faces,
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "offsetSolid" | "offsetSolidV2" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let result = remus_operations::offset_v2::offset_solid_v2(
                model.topology_mut(),
                solid,
                get_f64(args, "distance").map_err(|e| ops_err(&e.0))?,
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "draft" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let faces: Vec<FaceId> = args
                .get("faces")
                .and_then(Value::as_array)
                .ok_or_else(|| ops_err("missing or invalid 'faces'"))?
                .iter()
                .map(|v| as_face(model, v))
                .collect::<Result<Vec<_>, _>>()?;
            // The batch `angle` is in degrees, matching the direct `draft`
            // binding (see `batch_draft_volume_matches_closed_form`).
            let angle_radians = get_f64(args, "angle")
                .map_err(|e| ops_err(&e.0))?
                .to_radians();
            let result = remus_operations::draft::draft(
                model.topology_mut(),
                solid,
                &faces,
                Vec3::new(
                    get_f64_or(args, "dirX", 0.0),
                    get_f64_or(args, "dirY", 0.0),
                    get_f64_or(args, "dirZ", 1.0),
                ),
                Point3::new(
                    get_f64_or(args, "neutralX", 0.0),
                    get_f64_or(args, "neutralY", 0.0),
                    get_f64_or(args, "neutralZ", 0.0),
                ),
                angle_radians,
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "extrude" => {
            let face = as_face(
                model,
                args.get("face").ok_or_else(|| ops_err("missing 'face'"))?,
            )?;
            let result = model.extrude(
                face,
                Vec3::new(
                    get_f64_or(args, "dx", 0.0),
                    get_f64_or(args, "dy", 0.0),
                    get_f64_or(args, "dz", 1.0),
                ),
                get_f64_or(args, "distance", 1.0),
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "revolve" => {
            let face = as_face(
                model,
                args.get("face").ok_or_else(|| ops_err("missing 'face'"))?,
            )?;
            // The batch `angle` is in degrees, matching the direct binding.
            let result = model.revolve(
                face,
                Point3::new(
                    get_f64_or(args, "originX", 0.0),
                    get_f64_or(args, "originY", 0.0),
                    get_f64_or(args, "originZ", 0.0),
                ),
                Vec3::new(
                    get_f64_or(args, "axisX", 0.0),
                    get_f64_or(args, "axisY", 0.0),
                    get_f64_or(args, "axisZ", 1.0),
                ),
                get_f64(args, "angle")
                    .map_err(|e| ops_err(&e.0))?
                    .to_radians(),
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "sweep" => {
            let face = as_face(
                model,
                args.get("face").ok_or_else(|| ops_err("missing 'face'"))?,
            )?;
            let path_edge = as_edge(
                model,
                args.get("pathEdge")
                    .ok_or_else(|| ops_err("missing 'pathEdge'"))?,
            )?;
            let curve = nurbs_of_edge(model, path_edge)?;
            let result = model.sweep(face, &curve)?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "pipe" => {
            let face = as_face(
                model,
                args.get("face").ok_or_else(|| ops_err("missing 'face'"))?,
            )?;
            let path_edge = as_edge(
                model,
                args.get("pathEdge")
                    .ok_or_else(|| ops_err("missing 'pathEdge'"))?,
            )?;
            let curve = nurbs_of_edge(model, path_edge)?;
            let result = model.pipe(face, &curve, None)?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "helicalSweep" => {
            let face = as_face(
                model,
                args.get("profile")
                    .ok_or_else(|| ops_err("missing 'profile'"))?,
            )?;
            // Segments-per-turn 8 matches the batch arm.
            let result = remus_operations::helix::helical_sweep(
                model.topology_mut(),
                face,
                Point3::new(
                    get_f64(args, "axisOriginX").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "axisOriginY").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "axisOriginZ").map_err(|e| ops_err(&e.0))?,
                ),
                Vec3::new(
                    get_f64(args, "axisDirX").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "axisDirY").map_err(|e| ops_err(&e.0))?,
                    get_f64(args, "axisDirZ").map_err(|e| ops_err(&e.0))?,
                ),
                get_f64(args, "radius").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "pitch").map_err(|e| ops_err(&e.0))?,
                get_f64(args, "turns").map_err(|e| ops_err(&e.0))?,
                8,
            )?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "loft" => {
            let faces: Vec<FaceId> = args
                .get("faces")
                .and_then(Value::as_array)
                .ok_or_else(|| ops_err("missing or invalid 'faces'"))?
                .iter()
                .map(|v| as_face(model, v))
                .collect::<Result<Vec<_>, _>>()?;
            let result = model.loft(&faces)?;
            *quality = Some("exact".to_owned());
            Ok(solid_handle(result))
        }
        "volume" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let deflection = get_f64_or(args, "deflection", 0.1);
            let volume = model.volume(solid, deflection)?;
            Ok(serde_json::json!(volume))
        }
        "validateSolid" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let report = model.validate(solid)?;
            Ok(serde_json::json!(report.error_count()))
        }
        "meshQuality" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let deflection = get_f64_or(args, "deflection", 0.1);
            let mesh = model.tessellate(solid, deflection)?;
            let quality = welded_mesh_quality(&mesh);
            Ok(serde_json::json!({
                "triangleCount": quality.triangle_count,
                "boundaryEdges": quality.boundary_edges,
                "nonManifoldEdges": quality.non_manifold_edges,
                "eulerCharacteristic": quality.euler_characteristic,
                "isWatertight": quality.is_watertight(),
            }))
        }
        "getSolidFaces" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let faces = remus_topology::explorer::solid_faces(model.topology(), solid)?;
            Ok(Value::Array(faces.into_iter().map(face_handle).collect()))
        }
        "solidEdges" => {
            let solid = as_solid(
                model,
                args.get("solid")
                    .ok_or_else(|| ops_err("missing 'solid'"))?,
            )?;
            let edges = model.solid_edges(solid)?;
            Ok(Value::Array(edges.into_iter().map(edge_handle).collect()))
        }
        "getFaceNormal" => {
            let face = as_face(
                model,
                args.get("face").ok_or_else(|| ops_err("missing 'face'"))?,
            )?;
            let face_data = model.topology().face(face)?;
            let normal = match face_data.surface() {
                remus_topology::face::FaceSurface::Plane { normal, .. } => *normal,
                other => other.normal(0.0, 0.0),
            };
            Ok(serde_json::json!([normal.x(), normal.y(), normal.z()]))
        }
        _ => Err(ops_err(&format!("unsupported batch op '{op}'"))),
    }
}

fn nurbs_of_edge(model: &Model, edge: EdgeId) -> Result<NurbsCurve, DispatchError> {
    use remus_topology::edge::EdgeCurve;
    let curve = model
        .topology()
        .edge(edge)
        .map_err(OperationsError::from)?
        .curve()
        .clone();
    match curve {
        EdgeCurve::Line => {
            let start = model
                .topology()
                .vertex(
                    model
                        .topology()
                        .edge(edge)
                        .map_err(OperationsError::from)?
                        .start(),
                )
                .map_err(OperationsError::from)?
                .point();
            let end = model
                .topology()
                .vertex(
                    model
                        .topology()
                        .edge(edge)
                        .map_err(OperationsError::from)?
                        .end(),
                )
                .map_err(OperationsError::from)?
                .point();
            NurbsCurve::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![start, end],
                vec![1.0, 1.0],
            )
            .map_err(|_| {
                DispatchError::Ops(OperationsError::InvalidInput {
                    reason: "degenerate sweep path".to_owned(),
                })
            })
        }
        EdgeCurve::NurbsCurve(curve) => Ok(curve),
        _ => Err(DispatchError::Ops(OperationsError::InvalidInput {
            reason: "sweep path edge must be a line edge".to_owned(),
        })),
    }
}

fn observe(case: Case) -> Result<Observation, RunnerError> {
    // The extended slice uses schema 2; the first slice (`quadric-boolean`)
    // still emits schema 1 with `booleanIndex` instead of `opIndex`. The
    // same release binary serves both slices from `test-o15-parity.sh`.
    if case.schema_version != 1 && case.schema_version != 2 {
        return Err(err(format!(
            "unsupported case schema version {}",
            case.schema_version
        )));
    }
    let response_schema = case.schema_version;
    let op_index = case.result.producing_op()?;

    let init_started = Instant::now();
    let mut model = Model::new();
    let cold_init_ms = init_started.elapsed().as_secs_f64() * 1_000.0;
    let batch_started = Instant::now();
    // Sequential dispatch: references resolve against earlier responses, and
    // a failed op records its wire disclosure without stopping later ops —
    // the executeBatchV2 envelope contract on both surfaces.
    let mut responses: Vec<Value> = Vec::with_capacity(case.batch.len());
    let mut ok_values: Vec<Value> = Vec::with_capacity(case.batch.len());
    let mut last_quality: Option<String> = None;
    for (index, item) in case.batch.iter().enumerate() {
        let op = item
            .get("op")
            .and_then(Value::as_str)
            .ok_or_else(|| err("batch item is missing 'op'"))?;
        let args = item.get("args").cloned().unwrap_or(Value::Null);
        let resolved = resolve_args(&args, &ok_values, &model)?;
        // Mirror the WASM `dispatch_with_rollback` envelope: a failed op
        // restores the exact pre-operation topology so later ops observe
        // the same state on both surfaces. Without this, a native partial
        // mutation would diverge from the WASM rollback the contract
        // matrix's rollback cell asserts.
        let snapshot = model.topology().clone();
        let mut quality = None;
        match dispatch(&mut model, op, &resolved, &mut quality) {
            Ok(ok) => {
                if quality.is_some() {
                    last_quality = quality;
                }
                responses.push(serde_json::json!({"ok": ok}));
                ok_values.push(
                    responses
                        .last()
                        .and_then(|r| r.get("ok"))
                        .cloned()
                        .unwrap_or(Value::Null),
                );
            }
            Err(error) => {
                *model.topology_mut() = snapshot;
                responses.push(disclosure(op, index, &error));
                ok_values.push(Value::Null);
            }
        }
    }
    let batch_duration_ms = batch_started.elapsed().as_secs_f64() * 1_000.0;
    let diagnostic_codes = diagnostics(&responses);

    if !diagnostic_codes.is_empty() {
        // Rollback probes for the contract matrix: every succeeding `volume`
        // ok-value and the last succeeding `getSolidFaces` length ride along
        // so the scorer can assert later ops still observe pre-failure state.
        // Geometric scorers ignore these fields.
        let mut rollback_volumes = Vec::new();
        let mut rollback_faces = None;
        for (response, item) in responses.iter().zip(case.batch.iter()) {
            let Some(ok) = response.get("ok") else {
                continue;
            };
            match item.get("op").and_then(Value::as_str) {
                Some("volume") => {
                    if let Some(volume) = ok.as_f64() {
                        rollback_volumes.push(volume);
                    }
                }
                Some("getSolidFaces") => {
                    if let Some(faces) = ok.as_array() {
                        rollback_faces = Some(faces.len());
                    }
                }
                _ => {}
            }
        }
        return Ok(Observation {
            schema_version: response_schema,
            id: case.id,
            surface: "native",
            outcome: "batch_error",
            diagnostic_codes,
            quality: None,
            result_handle: None,
            volume: None,
            validation_errors: None,
            census: None,
            surface_types: None,
            mesh_quality: None,
            serialized_bytes: None,
            serialized_sha256: None,
            cold_init_ms,
            batch_duration_ms,
            rollback_volumes,
            rollback_faces,
            evolution: None,
            volumes_by_index: BTreeMap::new(),
        });
    }

    let producing_op = case.batch[op_index]
        .get("op")
        .and_then(Value::as_str)
        .unwrap_or_default();
    let quality = if producing_op == "booleanWithQuality" {
        response_ok(&responses, op_index, "boolean")?
            .get("quality")
            .and_then(Value::as_str)
            .ok_or_else(|| err("boolean response is missing quality"))?
            .to_owned()
    } else {
        last_quality.ok_or_else(|| err("modifier response is missing quality"))?
    };
    let result_handle_value = if returns_solid_object(producing_op) {
        response_ok(&responses, op_index, "boolean")?
            .get("solid")
            .cloned()
            .ok_or_else(|| err("boolean response is missing solid handle"))?
    } else {
        response_ok(&responses, op_index, "operation")?.clone()
    };
    // `*WithEvolution` responses carry the evolution report beside the
    // handle; pass it through untouched for the contract scorer.
    let evolution = response_ok(&responses, op_index, "operation")?
        .get("evolution")
        .cloned();
    let result_handle_u64 = result_handle_value
        .as_u64()
        .ok_or_else(|| err("result handle is not an integer"))?;
    let result_handle = u32::try_from(result_handle_u64)
        .map_err(|_| err(format!("solid handle {result_handle_u64} exceeds u32")))?;
    // Boolean mesh-fallback results legitimately mint fresh arena entities,
    // so the observed handle can differ from the fixture's positional guess
    // (2): resolve whatever the op returned. Sweep-family symbolic handles
    // ({fromOp}) assert the reference points at the producing op.
    match case.result.handle {
        HandleRef::Direct(expected) => {
            if !returns_solid_object(producing_op) && result_handle != expected {
                return Err(err(format!(
                    "result handle {result_handle} does not match fixture handle {expected}"
                )));
            }
        }
        HandleRef::Symbolic { from_op } => {
            if from_op != op_index {
                return Err(err(format!(
                    "symbolic result handle must reference op_index {}, got {from_op}",
                    op_index
                )));
            }
        }
    }
    let solid = resolve_solid(&model, result_handle)
        .map_err(|e| err(format!("result solid unresolvable: {e}")))?;

    let volume = response_ok(&responses, case.result.volume_index, "volume")?
        .as_f64()
        .ok_or_else(|| err("volume response is not numeric"))?;
    let volumes_by_index: BTreeMap<usize, f64> = responses
        .iter()
        .zip(case.batch.iter())
        .enumerate()
        .filter(|(_, (_, item))| item.get("op").and_then(Value::as_str) == Some("volume"))
        .filter_map(|(index, (response, _))| {
            response
                .get("ok")
                .and_then(Value::as_f64)
                .map(|volume| (index, volume))
        })
        .collect();
    let validation_errors = response_ok(&responses, case.result.validation_index, "validation")?
        .as_u64()
        .ok_or_else(|| err("validation response is not an integer"))?;
    let mesh_quality =
        response_ok(&responses, case.result.mesh_quality_index, "mesh quality")?.clone();
    let faces = response_ok(&responses, case.result.faces_index, "solid faces")?
        .as_array()
        .ok_or_else(|| err("solid faces response is not an array"))?;

    let face_ids: Vec<FaceId> = faces
        .iter()
        .map(|face| {
            face.as_u64()
                .ok_or_else(|| err("solid face handle is not an integer"))
                .and_then(|handle| {
                    u32::try_from(handle)
                        .map_err(|_| err(format!("face handle {handle} exceeds u32")))
                })
                .and_then(|handle| {
                    resolve_face(&model, handle)
                        .map_err(|e| err(format!("solid face unresolvable: {e}")))
                })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let census_faces = u32::try_from(face_ids.len()).map_err(|_| err("face census exceeds u32"))?;
    let edge_count = remus_topology::explorer::solid_edges(model.topology(), solid)
        .map_err(|e| err(e.to_string()))?
        .len();
    let vertex_count = {
        use std::collections::BTreeSet;
        let mut vertices = BTreeSet::new();
        for edge in remus_topology::explorer::solid_edges(model.topology(), solid)
            .map_err(|e| err(e.to_string()))?
        {
            let edge_data = model
                .topology()
                .edge(edge)
                .map_err(|e| err(e.to_string()))?;
            vertices.insert(edge_data.start().index());
            vertices.insert(edge_data.end().index());
        }
        vertices.len()
    };
    let census = Census {
        faces: census_faces,
        edges: u32::try_from(edge_count).map_err(|_| err("edge census exceeds u32"))?,
        vertices: u32::try_from(vertex_count).map_err(|_| err("vertex census exceeds u32"))?,
    };

    let mut surface_types = BTreeMap::new();
    for face in &face_ids {
        let surface = model
            .topology()
            .face(*face)
            .map_err(|e| err(e.to_string()))?
            .surface()
            .type_tag();
        // Mirror `getSurfaceType`: NURBS carriers report their detected
        // analytic kind, so the facade histogram compares like with like.
        let kind = if surface == "nurbs" {
            detect_nurbs_kind(&model, *face)?
        } else {
            surface.to_owned()
        };
        *surface_types.entry(kind).or_insert(0) += 1;
    }

    let serialized = remus_io::arena_io::serialize_solids(model.topology(), &[solid])
        .map_err(|e| err(e.to_string()))?;
    let serialized_sha256 = sha256_hex(&serialized);

    Ok(Observation {
        schema_version: response_schema,
        id: case.id,
        surface: "native",
        outcome: "success",
        diagnostic_codes,
        quality: Some(quality),
        result_handle: Some(result_handle),
        volume: Some(volume),
        validation_errors: Some(validation_errors),
        census: Some(census),
        surface_types: Some(surface_types),
        mesh_quality: Some(mesh_quality),
        serialized_bytes: Some(serialized.len()),
        serialized_sha256: Some(serialized_sha256),
        cold_init_ms,
        batch_duration_ms,
        rollback_volumes: Vec::new(),
        rollback_faces: None,
        evolution,
        volumes_by_index,
    })
}

fn detect_nurbs_kind(model: &Model, face: FaceId) -> Result<String, RunnerError> {
    use remus_topology::face::FaceSurface;
    let surface = &model
        .topology()
        .face(face)
        .map_err(|e| err(e.to_string()))?
        .surface()
        .clone();
    match surface {
        FaceSurface::Nurbs(surface) => Ok(remus_geometry::convert::detect_surface_kind(surface)
            .as_str()
            .to_owned()),
        other => Ok(other.type_tag().to_owned()),
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut input = String::new();
    io::stdin().read_to_string(&mut input)?;
    let case: Case = serde_json::from_str(&input)?;
    let observation = observe(case)?;
    serde_json::to_writer(io::stdout(), &observation)?;
    Ok(())
}
