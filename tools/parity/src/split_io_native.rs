//! Native facade half of the O1.5 split import/export workflow harness.
//!
//! The JS driver (`split-io-matrix.mjs`) emits one high-level case per cell
//! and feeds it to this binary on stdin. This runner executes the same cell
//! through the **native facade** (`remus::Model` plus `remus_io` directly) —
//! the surface the O1.5 task names — while the WASM driver executes it
//! through freshly packed/installed `remus-wasm` plus `remus-wasm-io` direct
//! calls (`serializeSolids`/`deserializeSolids` on the kernel,
//! `importStep`/`exportStep` on the translator). The observation schema is
//! shared so the scorer compares like with like.
//!
//! Cells (see `split-io-matrix.mjs::expandSplitMatrix`):
//! - `split/step-import-box`: import the shared box STEP fixture.
//! - `split/box-hollow-roundtrip`: create box + hollow cut, STEP round-trip.
//! - `split/periodic-seam`: import the shared filleted-plate STEP fixture.
//! - `split/malformed-refusal`: tiny-limit STEP refusal, session preserved.
//! - `split/arena-truncation`: truncated arena refusal, session preserved.
//! - `split/two-session-isolation`: two models, success/refusal isolation.

use std::collections::BTreeMap;
use std::io::{self, Read, Write};
use std::time::Instant;

use remus::Model;
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Deserialize)]
struct Case {
    schema_version: u32,
    id: String,
    #[serde(default)]
    box_step: Option<String>,
    #[serde(default)]
    plate_step: Option<String>,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct Observation {
    schema_version: u32,
    id: String,
    surface: &'static str,
    outcome: &'static str,
    diagnostic_codes: Vec<String>,
    volume: Option<f64>,
    volumes: Option<BTreeMap<String, f64>>,
    face_count: Option<usize>,
    face_counts: Option<BTreeMap<String, usize>>,
    shell_count: Option<usize>,
    shell_counts: Option<BTreeMap<String, usize>>,
    validation_errors: Option<u64>,
    census: Option<BTreeMap<String, u32>>,
    probes: Option<BTreeMap<String, String>>,
    pcurve_count: Option<usize>,
    step_pcurve_count: Option<usize>,
    rollback_preserved: Option<bool>,
    preserved_bytes_equal: Option<bool>,
    cross_session_stale_refused: Option<bool>,
    no_partial_solids: Option<bool>,
    cold_init_ms: f64,
    batch_duration_ms: f64,
}

fn code_of(error: &remus::IoError) -> String {
    match error {
        remus::IoError::LimitExceeded { .. } => "limit_exceeded".to_owned(),
        remus::IoError::ParseError { .. } => "parse_error".to_owned(),
        remus::IoError::UnsupportedEntity { .. } => "unsupported_entity".to_owned(),
        remus::IoError::InvalidTopology { .. } => "invalid_topology".to_owned(),
        _ => "operation_failed".to_owned(),
    }
}

fn census_of(model: &Model, solid: remus::SolidId) -> BTreeMap<String, u32> {
    let mut map = BTreeMap::new();
    if let Ok(faces) = remus_topology::explorer::solid_faces(model.topology(), solid) {
        for face in faces {
            if let Ok(data) = model.topology().face(face) {
                *map.entry(data.surface().type_tag().to_owned()).or_insert(0) += 1;
            }
        }
    }
    map
}

fn probe(model: &Model, solid: remus::SolidId, x: f64, y: f64, z: f64) -> String {
    match remus_operations::classify::classify_point(
        model.topology(),
        solid,
        remus_math::vec::Point3::new(x, y, z),
        0.1,
        1e-7,
    ) {
        Ok(remus_operations::classify::PointClassification::Inside) => "inside".to_owned(),
        Ok(remus_operations::classify::PointClassification::Outside) => "outside".to_owned(),
        Ok(remus_operations::classify::PointClassification::OnBoundary) => "boundary".to_owned(),
        Err(_) => "error".to_owned(),
    }
}

fn counts_of(model: &Model, solid: remus::SolidId) -> (f64, usize, usize) {
    let volume = model.volume(solid, 0.05).unwrap_or(f64::NAN);
    let faces = remus_topology::explorer::solid_faces(model.topology(), solid)
        .map(|faces| faces.len())
        .unwrap_or(usize::MAX);
    let shells = model
        .topology()
        .solid(solid)
        .map(|data| 1 + data.inner_shells().len())
        .unwrap_or(usize::MAX);
    (volume, faces, shells)
}

fn serialized_of(model: &Model, solid: remus::SolidId) -> Vec<u8> {
    remus_io::arena_io::serialize_solids(model.topology(), &[solid]).unwrap_or_default()
}

#[allow(clippy::too_many_lines)]
fn observe(case: &Case, cold_init_ms: f64) -> Observation {
    let started = Instant::now();
    let base = |outcome: &'static str| Observation {
        schema_version: 1,
        id: case.id.clone(),
        surface: "native",
        outcome,
        diagnostic_codes: Vec::new(),
        volume: None,
        volumes: None,
        face_count: None,
        face_counts: None,
        shell_count: None,
        shell_counts: None,
        validation_errors: None,
        census: None,
        probes: None,
        pcurve_count: None,
        step_pcurve_count: None,
        rollback_preserved: None,
        preserved_bytes_equal: None,
        cross_session_stale_refused: None,
        no_partial_solids: None,
        cold_init_ms,
        batch_duration_ms: 0.0,
    };

    let mut observation = match case.id.as_str() {
        "split/step-import-box" => {
            let mut observation = base("success");
            let step = case.box_step.clone().unwrap_or_default();
            let mut model = Model::new();
            match model.read_step(&step) {
                Ok(solids) if solids.len() == 1 => {
                    let solid = solids[0];
                    observation.volume = model.volume(solid, 0.05).ok();
                    observation.face_count =
                        remus_topology::explorer::solid_faces(model.topology(), solid)
                            .ok()
                            .map(|faces| faces.len());
                    observation.shell_count = model
                        .topology()
                        .solid(solid)
                        .ok()
                        .map(|data| 1 + data.inner_shells().len());
                    observation.validation_errors = model
                        .validate(solid)
                        .ok()
                        .map(|report| u64::try_from(report.error_count()).unwrap_or(u64::MAX));
                    observation.census = Some(census_of(&model, solid));
                    let mut probes = BTreeMap::new();
                    probes.insert("inside".to_owned(), probe(&model, solid, 1.0, 1.0, 1.0));
                    probes.insert("outside".to_owned(), probe(&model, solid, 10.0, 10.0, 10.0));
                    observation.probes = Some(probes);
                }
                Ok(_) => {
                    observation.outcome = "batch_error";
                    observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                }
                Err(error) => {
                    observation.outcome = "batch_error";
                    observation.diagnostic_codes = vec![code_of(&error)];
                }
            }
            observation
        }
        "split/box-hollow-roundtrip" => {
            let mut observation = base("success");
            let mut model = Model::new();
            let hollow_result = (|| -> Result<(remus::SolidId, remus::SolidId), remus::IoError> {
                let outer = model.make_box(10.0, 10.0, 10.0).map_err(|error| {
                    remus::IoError::InvalidTopology {
                        reason: error.to_string(),
                    }
                })?;
                let inner = model.make_box(8.0, 8.0, 8.0).map_err(|error| {
                    remus::IoError::InvalidTopology {
                        reason: error.to_string(),
                    }
                })?;
                model
                    .transform(
                        inner,
                        &remus_math::mat::Mat4([
                            [1.0, 0.0, 0.0, 1.0],
                            [0.0, 1.0, 0.0, 1.0],
                            [0.0, 0.0, 1.0, 1.0],
                            [0.0, 0.0, 0.0, 1.0],
                        ]),
                    )
                    .map_err(|error| remus::IoError::InvalidTopology {
                        reason: error.to_string(),
                    })?;
                let cut = model
                    .cut(outer, inner)
                    .map_err(|error| remus::IoError::InvalidTopology {
                        reason: error.to_string(),
                    })?
                    .solid;
                let fresh_box = model.make_box(2.0, 3.0, 4.0).map_err(|error| {
                    remus::IoError::InvalidTopology {
                        reason: error.to_string(),
                    }
                })?;
                Ok((fresh_box, cut))
            })();
            match hollow_result {
                Ok((plain_box, hollow)) => {
                    let step = match model.write_step(&[plain_box, hollow]) {
                        Ok(step) => step,
                        Err(error) => {
                            observation.outcome = "batch_error";
                            observation.diagnostic_codes = vec![code_of(&error)];
                            observation.batch_duration_ms =
                                started.elapsed().as_secs_f64() * 1_000.0;
                            return observation;
                        }
                    };
                    let mut fresh = Model::new();
                    match fresh.read_step(&step) {
                        Ok(solids) if solids.len() == 2 => {
                            let mut volumes = BTreeMap::new();
                            let mut face_counts = BTreeMap::new();
                            let mut shell_counts = BTreeMap::new();
                            let mut probes = BTreeMap::new();
                            // Order is file order: box first, hollow second.
                            // Identify by volume: 24 vs 488.
                            for solid in solids {
                                let volume = fresh.volume(solid, 0.05).unwrap_or(f64::NAN);
                                let key = if (volume - 24.0).abs() < 1e-6 {
                                    "box"
                                } else {
                                    "hollow"
                                };
                                volumes.insert(key.to_owned(), volume);
                                if let Ok(faces) =
                                    remus_topology::explorer::solid_faces(fresh.topology(), solid)
                                {
                                    face_counts.insert(key.to_owned(), faces.len());
                                }
                                if let Ok(data) = fresh.topology().solid(solid) {
                                    shell_counts
                                        .insert(key.to_owned(), 1 + data.inner_shells().len());
                                }
                                if key == "hollow" {
                                    probes.insert(
                                        "wall".to_owned(),
                                        probe(&fresh, solid, 0.5, 5.0, 5.0),
                                    );
                                    probes.insert(
                                        "cavity".to_owned(),
                                        probe(&fresh, solid, 5.0, 5.0, 5.0),
                                    );
                                    probes.insert(
                                        "outside".to_owned(),
                                        probe(&fresh, solid, 20.0, 20.0, 20.0),
                                    );
                                    observation.census = Some(census_of(&fresh, solid));
                                    observation.validation_errors =
                                        fresh.validate(solid).ok().map(|report| {
                                            u64::try_from(report.error_count()).unwrap_or(u64::MAX)
                                        });
                                }
                            }
                            observation.volumes = Some(volumes);
                            observation.face_counts = Some(face_counts);
                            observation.shell_counts = Some(shell_counts);
                            observation.probes = Some(probes);
                        }
                        Ok(_) => {
                            observation.outcome = "batch_error";
                            observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                        }
                        Err(error) => {
                            observation.outcome = "batch_error";
                            observation.diagnostic_codes = vec![code_of(&error)];
                        }
                    }
                }
                Err(error) => {
                    observation.outcome = "batch_error";
                    observation.diagnostic_codes = vec![code_of(&error)];
                }
            }
            observation
        }
        "split/periodic-seam" => {
            let mut observation = base("success");
            let step = case.plate_step.clone().unwrap_or_default();
            let mut model = Model::new();
            match model.read_step(&step) {
                Ok(solids) if solids.len() == 1 => {
                    let solid = solids[0];
                    observation.volume = model.volume(solid, 0.05).ok();
                    observation.face_count =
                        remus_topology::explorer::solid_faces(model.topology(), solid)
                            .ok()
                            .map(|faces| faces.len());
                    observation.validation_errors = model
                        .validate(solid)
                        .ok()
                        .map(|report| u64::try_from(report.error_count()).unwrap_or(u64::MAX));
                    observation.census = Some(census_of(&model, solid));
                    observation.pcurve_count = Some(model.topology().num_pcurves());
                    if let Ok(exported) = model.write_step(&[solid]) {
                        observation.step_pcurve_count = Some(exported.matches("PCURVE(").count());
                    }
                }
                Ok(_) => {
                    observation.outcome = "batch_error";
                    observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                }
                Err(error) => {
                    observation.outcome = "batch_error";
                    observation.diagnostic_codes = vec![code_of(&error)];
                }
            }
            observation
        }
        "split/malformed-refusal" => {
            let mut observation = base("batch_error");
            let mut model = Model::new();
            let Some(keep) = model.make_box(2.0, 3.0, 4.0).ok() else {
                observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                observation.batch_duration_ms = started.elapsed().as_secs_f64() * 1_000.0;
                return observation;
            };
            let before = counts_of(&model, keep);
            let before_bytes = serialized_of(&model, keep);
            let limits = remus_io::ImportLimits {
                max_input_bytes: 4,
                ..Default::default()
            };
            let Err(refusal) = model.read_step_with_limits("not a STEP file", limits) else {
                observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                observation.batch_duration_ms = started.elapsed().as_secs_f64() * 1_000.0;
                return observation;
            };
            observation.diagnostic_codes = vec![code_of(&refusal)];
            let after = counts_of(&model, keep);
            let after_bytes = serialized_of(&model, keep);
            observation.rollback_preserved = Some(
                (after.0 - before.0).abs() < 1e-12 && after.1 == before.1 && after.2 == before.2,
            );
            observation.preserved_bytes_equal = Some(after_bytes == before_bytes);
            observation.volume = model.volume(keep, 0.05).ok();
            observation.face_count = remus_topology::explorer::solid_faces(model.topology(), keep)
                .ok()
                .map(|faces| faces.len());
            observation
        }
        "split/arena-truncation" => {
            let mut observation = base("batch_error");
            let mut model = Model::new();
            let Some(keep) = model.make_box(2.0, 3.0, 4.0).ok() else {
                observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                observation.batch_duration_ms = started.elapsed().as_secs_f64() * 1_000.0;
                return observation;
            };
            let before = counts_of(&model, keep);
            let before_bytes = serialized_of(&model, keep);
            let full = serialized_of(&model, keep);
            let truncated = full.get(..full.len() / 2).unwrap_or(&[]).to_vec();
            let code =
                match remus_io::arena_io::deserialize_solids(&truncated, model.topology_mut()) {
                    Ok(_) => "operation_failed".to_owned(),
                    Err(error) => code_of(&error),
                };
            // An empty document is a second typed refusal on the same session.
            let empty_code = match remus_io::arena_io::deserialize_solids(&[], model.topology_mut())
            {
                Ok(_) => "operation_failed".to_owned(),
                Err(error) => code_of(&error),
            };
            observation.diagnostic_codes = vec![code, empty_code];
            let after = counts_of(&model, keep);
            let after_bytes = serialized_of(&model, keep);
            observation.rollback_preserved = Some(
                (after.0 - before.0).abs() < 1e-12 && after.1 == before.1 && after.2 == before.2,
            );
            observation.preserved_bytes_equal = Some(after_bytes == before_bytes);
            observation.volume = model.volume(keep, 0.05).ok();
            observation.face_count = remus_topology::explorer::solid_faces(model.topology(), keep)
                .ok()
                .map(|faces| faces.len());
            // A fabricated success would claim outcome success here; the
            // scorer requires batch_error plus preservation.
            observation
        }
        "split/two-session-isolation" => {
            let mut observation = base("success");
            let step = case.box_step.clone().unwrap_or_default();
            let mut session_a = Model::new();
            let mut session_b = Model::new();
            // A imports valid.
            let imported_a = session_a.read_step(&step);
            let imported_a_len = imported_a.as_ref().map(Vec::len).unwrap_or(0);
            let volume_a = imported_a
                .as_ref()
                .ok()
                .and_then(|solids| solids.first().copied())
                .and_then(|solid| session_a.volume(solid, 0.05).ok());
            // B refuses malformed; no partial solids may appear.
            let faces_before_b = session_b.topology().num_faces();
            let solids_before_b = session_b.topology().num_solids();
            let limits = remus_io::ImportLimits {
                max_input_bytes: 4,
                ..Default::default()
            };
            let Err(refused) = session_b.read_step_with_limits("not a STEP file", limits) else {
                observation.outcome = "batch_error";
                observation.diagnostic_codes = vec!["operation_failed".to_owned()];
                observation.batch_duration_ms = started.elapsed().as_secs_f64() * 1_000.0;
                return observation;
            };
            let faces_after_refusal = session_b.topology().num_faces();
            let solids_after_refusal = session_b.topology().num_solids();
            // B then imports valid.
            let imported_b = session_b.read_step(&step);
            let imported_b_len = imported_b.as_ref().map(Vec::len).unwrap_or(0);
            let volume_b = imported_b
                .as_ref()
                .ok()
                .and_then(|solids| solids.first().copied())
                .and_then(|solid| session_b.volume(solid, 0.05).ok());
            // Cross-session handle: an out-of-range index must not resolve in B.
            let stale_handle = session_a.topology().num_solids() + 1000;
            let stale_refused = session_b
                .topology()
                .solid_id_from_index(stale_handle)
                .is_none();
            let no_partial = faces_after_refusal == faces_before_b
                && solids_after_refusal == solids_before_b
                && imported_b_len == 1
                && imported_a_len == 1;
            observation.no_partial_solids = Some(no_partial);
            observation.cross_session_stale_refused = Some(stale_refused);
            observation.diagnostic_codes = vec![code_of(&refused)];
            let mut volumes = BTreeMap::new();
            if let Some(volume) = volume_a {
                volumes.insert("sessionA".to_owned(), volume);
            }
            if let Some(volume) = volume_b {
                volumes.insert("sessionB".to_owned(), volume);
            }
            observation.volumes = Some(volumes);
            observation.outcome = if no_partial && stale_refused {
                "success"
            } else {
                "batch_error"
            };
            if observation.outcome == "batch_error" {
                observation
                    .diagnostic_codes
                    .push("operation_failed".to_owned());
            }
            observation
        }
        _ => {
            let mut observation = base("batch_error");
            observation.diagnostic_codes = vec!["operation_failed".to_owned()];
            observation
        }
    };
    observation.batch_duration_ms = started.elapsed().as_secs_f64() * 1_000.0;
    // Surface the post-run volume for refusal cells that kept a box.
    if observation.volume.is_none()
        && matches!(
            case.id.as_str(),
            "split/malformed-refusal" | "split/arena-truncation"
        )
    {
        observation.volume = Some(f64::NAN);
    }
    observation
}

fn emit(observation: &Observation) {
    if serde_json::to_writer(io::stdout(), observation).is_err() {
        let _ = io::stdout().write_all(b"{}");
    }
}

fn crash_observation(cold_init_ms: f64) -> Observation {
    Observation {
        schema_version: 1,
        id: "unknown".to_owned(),
        surface: "native",
        outcome: "crash",
        diagnostic_codes: Vec::new(),
        volume: None,
        volumes: None,
        face_count: None,
        face_counts: None,
        shell_count: None,
        shell_counts: None,
        validation_errors: None,
        census: None,
        probes: None,
        pcurve_count: None,
        step_pcurve_count: None,
        rollback_preserved: None,
        preserved_bytes_equal: None,
        cross_session_stale_refused: None,
        no_partial_solids: None,
        cold_init_ms,
        batch_duration_ms: 0.0,
    }
}

fn main() {
    let init_started = Instant::now();
    let mut input = String::new();
    if io::stdin().read_to_string(&mut input).is_err() {
        return;
    }
    let cold_init_ms = init_started.elapsed().as_secs_f64() * 1_000.0;
    let Ok(case) = serde_json::from_str::<Case>(&input) else {
        emit(&crash_observation(cold_init_ms));
        return;
    };
    if case.schema_version != 1 {
        let mut observation = observe(&case, cold_init_ms);
        observation.outcome = "crash";
        emit(&observation);
        return;
    }
    let observation = observe(&case, cold_init_ms);
    emit(&observation);
    let _ = Value::Null;
}
