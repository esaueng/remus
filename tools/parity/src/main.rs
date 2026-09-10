//! Native half of the per-operation native/WASM parity harness.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::{Display, Formatter};
use std::io::{self, Read};
use std::time::Instant;

use remus_wasm::kernel::BrepKernel;
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
    handle: u32,
    boolean_index: usize,
    volume_index: usize,
    validation_index: usize,
    mesh_quality_index: usize,
    faces_index: usize,
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

fn observe(case: Case) -> Result<Observation, RunnerError> {
    if case.schema_version != 1 {
        return Err(err(format!(
            "unsupported case schema version {}",
            case.schema_version
        )));
    }

    let init_started = Instant::now();
    let mut kernel = BrepKernel::new();
    let cold_init_ms = init_started.elapsed().as_secs_f64() * 1_000.0;
    let batch_json = serde_json::to_string(&case.batch)
        .map_err(|error| err(format!("serialize batch: {error}")))?;
    let batch_started = Instant::now();
    let response_json = kernel.execute_batch_v2(&batch_json);
    let batch_duration_ms = batch_started.elapsed().as_secs_f64() * 1_000.0;
    let responses: Vec<Value> = serde_json::from_str(&response_json)
        .map_err(|error| err(format!("parse batch response: {error}")))?;
    let diagnostic_codes = diagnostics(&responses);

    if !diagnostic_codes.is_empty() {
        return Ok(Observation {
            schema_version: 1,
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
        });
    }

    let boolean = response_ok(&responses, case.result.boolean_index, "boolean")?;
    let quality = boolean
        .get("quality")
        .and_then(Value::as_str)
        .ok_or_else(|| err("boolean response is missing quality"))?
        .to_owned();
    let result_handle_u64 = boolean
        .get("solid")
        .and_then(Value::as_u64)
        .ok_or_else(|| err("boolean response is missing solid handle"))?;
    let result_handle = u32::try_from(result_handle_u64)
        .map_err(|_| err(format!("solid handle {result_handle_u64} exceeds u32")))?;
    if result_handle != case.result.handle {
        return Err(err(format!(
            "result handle {result_handle} does not match fixture handle {}",
            case.result.handle
        )));
    }

    let volume = response_ok(&responses, case.result.volume_index, "volume")?
        .as_f64()
        .ok_or_else(|| err("volume response is not numeric"))?;
    let validation_errors = response_ok(&responses, case.result.validation_index, "validation")?
        .as_u64()
        .ok_or_else(|| err("validation response is not an integer"))?;
    let mesh_quality =
        response_ok(&responses, case.result.mesh_quality_index, "mesh quality")?.clone();
    let faces = response_ok(&responses, case.result.faces_index, "solid faces")?
        .as_array()
        .ok_or_else(|| err("solid faces response is not an array"))?;

    let counts = kernel
        .get_entity_counts(result_handle)
        .map_err(|error| err(format!("get entity counts: {error:?}")))?;
    let census = Census {
        faces: *counts
            .first()
            .ok_or_else(|| err("face census is missing"))?,
        edges: *counts.get(1).ok_or_else(|| err("edge census is missing"))?,
        vertices: *counts
            .get(2)
            .ok_or_else(|| err("vertex census is missing"))?,
    };

    let mut surface_types = BTreeMap::new();
    for face in faces {
        let face_u64 = face
            .as_u64()
            .ok_or_else(|| err("solid face handle is not an integer"))?;
        let face_handle = u32::try_from(face_u64)
            .map_err(|_| err(format!("face handle {face_u64} exceeds u32")))?;
        let surface_type = kernel
            .get_surface_type(face_handle)
            .map_err(|error| err(format!("get surface type: {error:?}")))?;
        *surface_types.entry(surface_type).or_insert(0) += 1;
    }

    let serialized = kernel
        .serialize_solid(result_handle)
        .map_err(|error| err(format!("serialize solid: {error:?}")))?;
    let serialized_sha256 = sha256_hex(&serialized);

    Ok(Observation {
        schema_version: 1,
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
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut input = String::new();
    io::stdin().read_to_string(&mut input)?;
    let case: Case = serde_json::from_str(&input)?;
    let observation = observe(case)?;
    serde_json::to_writer(io::stdout(), &observation)?;
    Ok(())
}
