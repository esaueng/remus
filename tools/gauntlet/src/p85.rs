//! P-Class 8.5 bounded real-model operation/export qualification slice.
//!
//! The base gauntlet ([`crate`]) runs import, validation, a centered probe
//! cut, tessellation, and STEP round-trip. This module adds the bounded 8.5
//! extension: explicit per-fixture operation recipes with preservation checks
//! across edit → export → reimport.
//!
//! ## Selection rule (declared before observing any operation outcome)
//!
//! Population: `tools/gauntlet/manifests/mambo.json` (113 models, Apache-2.0
//! at pinned commit `302b8bf33f5126d0c749f60226b76dbe94f21728`).
//! Stratified deterministic sample with seed [`P85_SELECTION_SEED`]:
//! the 6 lowest `sha256-rank-v1` basic models, the 3 lowest simple models,
//! and the 2 lowest medium models (11 total, manifest order preserved).
//! Entries are reused verbatim (id, URL, SHA-256, license class, size); no
//! corpus bytes are committed. Every selected model stays in the denominator:
//! refused and failed models are reported, never dropped.
//!
//! ## Per-fixture recipe (identical rule for every fixture)
//!
//! 1. **Rigid transform** (exact): rotate 30° about Z through the solid's
//!    bounding-box center, then translate by `(0.3, -0.2, 0.1)` diagonal
//!    lengths. Expected: volume/area invariant, entity counts, cavities,
//!    analytic carrier census, and material occupancy preserved.
//! 2. **Declared exact operation**: [`boolean_regions`]-style exact-only fuse
//!    of each transformed solid with a disjoint analytic box of side
//!    `0.2` diagonals placed one half-diagonal beyond the bounding-box
//!    maximum. Expected: two regions per solid, old-region volume/area and
//!    census unchanged, box region matching the closed-form `s³` / `6s²`
//!    oracles, cavities unchanged. Mesh fallback is impossible on this path;
//!    an exact-only refusal is a supported `Refused` outcome, never a pass.
//! 3. **Export**: STEP write of every result solid. Expected: millimetre
//!    units declared, one manifold root per solid.
//! 4. **Reimport**: read the export back under the same limits. Expected:
//!    solid count, validation, volume/area (deflection-scaled bounds),
//!    carrier census, cavities, and occupancy preserved.
//!
//! Same-kernel round-trip agreement is only one check among several: the box
//! closed forms, the disjoint-fuse inclusion identity
//! (`V_fuse = V_old + s³`), the ray-cast occupancy probes, the STEP-text unit
//! and carrier-entity cross-checks, and the CAx-IF validation-property count
//! (expected zero for the MAMBO slice — an explicit oracle gap) are
//! independent of the round-trip path.
//!
//! [`boolean_regions`]: remus_operations::boolean::boolean_regions

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use remus_check::CheckError;
use remus_check::properties::{PropertiesOptions, solid_area, solid_volume};
use remus_check::validate::{Severity, ValidateOptions, validate_solid};
use remus_io::{ImportLimits, IoError};
use remus_math::diagnostic::{FailureCategory, ToDiagnostic};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean_regions};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_bounding_box;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{TriangleMesh, tessellate_solid};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces, solid_vertices};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use serde::{Deserialize, Serialize};

use crate::{
    DiagnosticRecord, GauntletError, MetricValue, PipelineConfig, StageResult, StageStatus,
};

/// Report schema version for the 8.5 slice rows.
pub const P85_SCHEMA_VERSION: u32 = 1;

/// Deterministic selection seed for the 8.5 slice (see module docs).
pub const P85_SELECTION_SEED: u64 = 8505;

/// Rigid rotation applied by the recipe, in degrees about Z.
pub const P85_ROTATION_DEG: f64 = 30.0;

/// Probe-box side as a fraction of the solid bounding-box diagonal.
pub const P85_BOX_FRACTION: f64 = 0.2;

/// Probe-box clearance beyond the bounding-box maximum, in diagonals.
pub const P85_BOX_GAP_FRACTIONS: f64 = 0.5;

/// Occupancy probe grid resolution per axis (27 probes per solid).
pub const P85_OCCUPANCY_GRID: usize = 3;

/// Stage names in pipeline order.
pub const P85_STAGE_NAMES: [&str; 6] = [
    "read",
    "validate",
    "transform",
    "exact_op",
    "export",
    "reimport",
];

/// Per-model verdict.
///
/// `Failed` covers incorrect success (a stage returned `Ok` but a
/// postcondition was violated) and ordinary stage errors. `Refused` is a
/// typed capability refusal, `Crashed` a worker or internal failure, and
/// `Resource` a budget or limit failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum P85Verdict {
    /// Every stage passed.
    Pass,
    /// A stage failed: incorrect success or an ordinary stage error.
    Failed,
    /// A typed capability refusal (`unsupported` / `quality_refused`).
    Refused,
    /// A worker or internal failure (crash, spawn failure, bad output).
    Crashed,
    /// A budget or limit failure (`resource_limit` et al.).
    Resource,
}

/// The deterministic recipe applied to one fixture, with actuals recorded.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct P85Recipe {
    /// Rotation about Z through each solid's bbox center, in degrees.
    pub rotation_deg: f64,
    /// Translation as fractions of each solid's bbox diagonal.
    pub translation_fractions: [f64; 3],
    /// Probe-box side as a fraction of the bbox diagonal.
    pub box_fraction: f64,
    /// Probe-box clearance beyond the bbox maximum, in diagonals.
    pub box_gap_fractions: f64,
    /// Flattened row-major rigid matrices actually applied, one per solid.
    pub applied_matrices: Vec<[f64; 16]>,
    /// Probe-box sides actually used, one per solid.
    pub box_sides: Vec<f64>,
}

/// Six required per-model stages for the 8.5 slice.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct P85Stages {
    /// Bounded STEP import with body, cavity, carrier, and unit census.
    pub read: StageResult,
    /// Per-solid L3 validation.
    pub validate: StageResult,
    /// Rigid transform with invariance and occupancy checks.
    pub transform: StageResult,
    /// Exact-only disjoint-box fuse with closed-form oracles.
    pub exact_op: StageResult,
    /// STEP export with root-count and millimetre-unit checks.
    pub export: StageResult,
    /// STEP reimport with property, carrier, cavity, and occupancy checks.
    pub reimport: StageResult,
}

impl P85Stages {
    fn named(&self) -> [(&'static str, &StageResult); 6] {
        [
            ("read", &self.read),
            ("validate", &self.validate),
            ("transform", &self.transform),
            ("exact_op", &self.exact_op),
            ("export", &self.export),
            ("reimport", &self.reimport),
        ]
    }
}

/// One JSONL row for one slice model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct P85ModelResult {
    /// Report schema version.
    pub schema_version: u32,
    /// Input path supplied to the runner.
    pub model: String,
    /// Stable manifest model identifier.
    pub model_id: String,
    /// SHA-256 of the model bytes the worker read.
    pub model_sha256: String,
    /// Kernel commit the result claims to exercise.
    pub kernel_sha: String,
    /// SHA-256 of the slice manifest bytes.
    pub manifest_sha256: String,
    /// Recipe rule parameters with per-solid actuals.
    pub recipe: P85Recipe,
    /// Per-model verdict.
    pub verdict: P85Verdict,
    /// First failing stage in pipeline order, if any.
    pub first_failing_stage: Option<String>,
    /// End-to-end worker time.
    pub total_duration_ms: u64,
    /// Required stage results.
    pub stages: P85Stages,
}

/// Aggregate scoreboard for the 8.5 slice.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct P85Scoreboard {
    /// Report schema version.
    pub schema_version: u32,
    /// Kernel commit the scoreboard claims to exercise.
    pub kernel_sha: String,
    /// SHA-256 of the slice manifest bytes.
    pub manifest_sha256: String,
    /// Total model count (the full denominator).
    pub models: usize,
    /// Models passing every stage.
    pub passed: usize,
    /// Models with an incorrect-success or ordinary stage failure.
    pub failed: usize,
    /// Models ending in a typed capability refusal.
    pub refused: usize,
    /// Models lost to worker or internal failure.
    pub crashed: usize,
    /// Models lost to a budget or limit failure.
    pub resource: usize,
    /// Per-stage pass/fail counts.
    pub stages: BTreeMap<String, crate::StageSummary>,
    /// Primary failure count by stable taxonomy category.
    pub failure_categories: BTreeMap<String, usize>,
    /// Models whose declared exact operation completed exactly.
    pub exact_op_exact: usize,
}

/// Deterministic reproduction bundle for one fixture and recipe.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct P85ReplayBundle {
    /// Report schema version.
    pub schema_version: u32,
    /// Kernel commit the bundled result was produced at.
    pub kernel_sha: String,
    /// SHA-256 of the slice manifest bytes.
    pub manifest_sha256: String,
    /// Stable manifest model identifier.
    pub model_id: String,
    /// SHA-256 of the model bytes.
    pub model_sha256: String,
    /// Model size in bytes.
    pub model_size: u64,
    /// Import limits applied to both STEP reads.
    pub import_limits: P85Limits,
    /// Tessellation deflection and property-comparison scale.
    pub deflection: f64,
    /// Per-model wall-clock budget in milliseconds.
    pub model_timeout_ms: u64,
    /// Recipe rule parameters with per-solid actuals.
    pub recipe: P85Recipe,
    /// Expected outcome to compare a replay against.
    pub result: P85ModelResult,
}

/// Serializable import limits for the replay bundle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct P85Limits {
    /// Maximum STEP input bytes accepted by a read.
    pub max_input_bytes: usize,
    /// Maximum parsed model entities accepted by a read.
    pub max_model_entities: usize,
}

impl From<ImportLimits> for P85Limits {
    fn from(limits: ImportLimits) -> Self {
        Self {
            max_input_bytes: limits.max_input_bytes,
            max_model_entities: limits.max_model_entities,
        }
    }
}

/// Parent-process configuration for an 8.5 slice run.
#[derive(Debug, Clone)]
pub struct P85RunConfig {
    /// Configuration passed to each worker.
    pub pipeline: PipelineConfig,
    /// Hard wall-clock budget for the worker process.
    pub model_timeout: Duration,
    /// Maximum isolated model workers executed concurrently.
    pub max_parallel_models: usize,
    /// Kernel commit recorded as provenance on every row.
    pub kernel_sha: String,
    /// SHA-256 of the slice manifest bytes.
    pub manifest_sha256: String,
}

/// Run the six-stage 8.5 pipeline for one model in the current process.
///
/// Public for deterministic unit tests and worker execution; production
/// callers should use [`run_p85_isolated`].
#[must_use]
pub fn process_p85_model(
    path: &Path,
    model_id: &str,
    model_sha256: &str,
    config: PipelineConfig,
    kernel_sha: &str,
    manifest_sha256: &str,
) -> P85ModelResult {
    let total_started = Instant::now();
    let mut recipe = P85Recipe {
        rotation_deg: P85_ROTATION_DEG,
        translation_fractions: [0.3, -0.2, 0.1],
        box_fraction: P85_BOX_FRACTION,
        box_gap_fractions: P85_BOX_GAP_FRACTIONS,
        applied_matrices: Vec::new(),
        box_sides: Vec::new(),
    };
    let fail = |recipe: P85Recipe, stages: P85Stages| {
        finish_p85(
            path,
            model_id,
            model_sha256,
            kernel_sha,
            manifest_sha256,
            recipe,
            total_started,
            stages,
        )
    };

    // ---- read ----
    let read_started = Instant::now();
    let input = match read_limited_utf8(path, config.import_limits.max_input_bytes) {
        Ok(input) => input,
        Err(diagnostic) => {
            let read = fail_stage(read_started, diagnostic);
            return fail(recipe.clone(), all_failed_after(&read, "read"));
        }
    };
    let mut topology = Topology::new();
    let report = match remus_io::step::read_step_bodies_with_limits(
        &input,
        &mut topology,
        config.import_limits,
    ) {
        Ok(report) => report,
        Err(error) => {
            let read = fail_stage(read_started, io_diagnostic(&error));
            return fail(recipe.clone(), all_failed_after(&read, "read"));
        }
    };
    let solids = report.solids().to_vec();
    if solids.is_empty() {
        let read = fail_stage(
            read_started,
            error_diagnostic(
                FailureCategory::InvalidInput,
                "step_contains_no_solids",
                "STEP input contains no solid B-Reps",
            ),
        );
        return fail(recipe.clone(), all_failed_after(&read, "read"));
    }
    let mut read = pass_stage(read_started);
    read.metric("solid_count", MetricValue::Integer(as_u64(solids.len())));
    read.metric(
        "sheet_count",
        MetricValue::Integer(as_u64(report.sheets().len())),
    );
    read.metric("input_bytes", MetricValue::Integer(as_u64(input.len())));
    read.metric(
        "length_unit",
        MetricValue::Text(detect_length_unit(&input).to_owned()),
    );
    let census = solid_census(&topology, &solids);
    read.metric("face_count", MetricValue::Integer(census.faces));
    read.metric("edge_count", MetricValue::Integer(census.edges));
    read.metric("vertex_count", MetricValue::Integer(census.vertices));
    read.metric("cavity_count", MetricValue::Integer(census.cavities));
    for (carrier, count) in &census.carriers {
        read.metric(&format!("carrier_{carrier}"), MetricValue::Integer(*count));
    }
    // CAx-IF validation properties, when the source carries them, are the
    // only source-side independent area/volume expectations. MAMBO fixtures
    // carry none; the count below documents that oracle gap per model.
    let validation_reports = read_step_validation_count(&input, config.import_limits);
    read.metric(
        "validation_property_declarations",
        MetricValue::Integer(validation_reports),
    );

    // ---- validate ----
    let validate_started = Instant::now();
    let validate = validate_all(&topology, &solids, validate_started);
    if validate.status == StageStatus::Fail {
        return fail(recipe.clone(), after_read(read, validate, "validate"));
    }

    // ---- transform (rigid, exact) ----
    let transform_started = Instant::now();
    let snapshots: Vec<SolidSnapshot> = match solids
        .iter()
        .map(|&solid| SolidSnapshot::capture(&topology, solid, config.deflection))
        .collect()
    {
        Ok(snapshots) => snapshots,
        Err(diagnostic) => {
            let transform = fail_stage(transform_started, diagnostic);
            return fail(
                recipe.clone(),
                after_validate(read, validate, transform, "transform"),
            );
        }
    };
    for (solid, snapshot) in solids.iter().zip(&snapshots) {
        let matrix = rigid_matrix(snapshot.bbox_center, snapshot.diagonal, &recipe);
        recipe.applied_matrices.push(matrix_row_major(&matrix));
        if let Err(error) = transform_solid(&mut topology, *solid, &matrix) {
            let transform = fail_stage(transform_started, operations_diagnostic(&error));
            return fail(
                recipe.clone(),
                after_validate(read, validate, transform, "transform"),
            );
        }
    }
    let transform =
        match verify_transform(&topology, &solids, &snapshots, &recipe, transform_started) {
            Ok(stage) => stage,
            Err(stage) => {
                return fail(
                    recipe.clone(),
                    after_validate(read, validate, stage, "transform"),
                );
            }
        };

    // ---- exact_op (exact-only disjoint-box fuse per solid) ----
    let exact_started = Instant::now();
    let pre_fuse: Vec<SolidSnapshot> = match solids
        .iter()
        .map(|&solid| SolidSnapshot::capture(&topology, solid, config.deflection))
        .collect()
    {
        Ok(snapshots) => snapshots,
        Err(diagnostic) => {
            let exact_op = fail_stage(exact_started, diagnostic);
            return fail(
                recipe.clone(),
                after_transform(read, validate, transform, exact_op, "exact_op"),
            );
        }
    };
    let mut result_solids = Vec::new();
    let mut exact_op = pass_stage(exact_started);
    let mut fused_regions = 0_u64;
    for (solid, snapshot) in solids.iter().zip(&pre_fuse) {
        let side = snapshot.diagonal * recipe.box_fraction;
        recipe.box_sides.push(side);
        match fuse_disjoint_box(
            &mut topology,
            *solid,
            snapshot,
            side,
            recipe.box_gap_fractions,
        ) {
            Ok(regions) => {
                fused_regions += as_u64(regions.len());
                result_solids.extend(regions);
            }
            Err(diagnostic) => {
                let exact_op = fail_stage(exact_started, diagnostic);
                return fail(
                    recipe.clone(),
                    after_transform(read, validate, transform, exact_op, "exact_op"),
                );
            }
        }
    }
    exact_op.metric("fused_region_count", MetricValue::Integer(fused_regions));
    exact_op.metric(
        "result_solid_count",
        MetricValue::Integer(as_u64(result_solids.len())),
    );

    // ---- export ----
    let export_started = Instant::now();
    let step = match remus_io::step::write_step(&topology, &result_solids) {
        Ok(step) => step,
        Err(error) => {
            let export = fail_stage(export_started, io_diagnostic(&error));
            return fail(
                recipe.clone(),
                after_exact_op(read, validate, transform, exact_op, export, "export"),
            );
        }
    };
    let export = match verify_export(&step, result_solids.len(), export_started) {
        Ok(stage) => stage,
        Err(stage) => {
            return fail(
                recipe.clone(),
                after_exact_op(read, validate, transform, exact_op, stage, "export"),
            );
        }
    };

    // ---- reimport ----
    let reimport_started = Instant::now();
    let mut reimported_topology = Topology::new();
    let reimported = match remus_io::step::read_step_with_limits(
        &step,
        &mut reimported_topology,
        config.import_limits,
    ) {
        Ok(solids) => solids,
        Err(error) => {
            let reimport = fail_stage(reimport_started, io_diagnostic(&error));
            return fail(
                recipe.clone(),
                after_export(
                    read, validate, transform, exact_op, export, reimport, "reimport",
                ),
            );
        }
    };
    let reimport = match verify_reimport(
        &topology,
        &result_solids,
        &reimported_topology,
        &reimported,
        config,
        reimport_started,
    ) {
        Ok(stage) => stage,
        Err(stage) => {
            return fail(
                recipe.clone(),
                after_export(
                    read, validate, transform, exact_op, export, stage, "reimport",
                ),
            );
        }
    };

    fail(
        recipe,
        P85Stages {
            read,
            validate,
            transform,
            exact_op,
            export,
            reimport,
        },
    )
}

/// Run slice models in isolated worker subprocesses with a hard per-model
/// timeout. Results preserve input order.
#[must_use]
pub fn run_p85_isolated(
    executable: &Path,
    models: &[(String, String, PathBuf)],
    config: &P85RunConfig,
) -> Vec<P85ModelResult> {
    if models.is_empty() {
        return Vec::new();
    }
    let workers = config.max_parallel_models.max(1).min(models.len());
    if workers == 1 {
        return models
            .iter()
            .map(|model| run_p85_isolated_one(executable, model, config))
            .collect();
    }
    let mut ordered: Vec<Option<P85ModelResult>> =
        std::iter::repeat_with(|| None).take(models.len()).collect();
    let (sender, receiver) = std::sync::mpsc::channel();
    std::thread::scope(|scope| {
        for worker in 0..workers {
            let sender = sender.clone();
            scope.spawn(move || {
                for index in (worker..models.len()).step_by(workers) {
                    let result = run_p85_isolated_one(executable, &models[index], config);
                    if sender.send((index, result)).is_err() {
                        return;
                    }
                }
            });
        }
        drop(sender);
        for (index, result) in receiver {
            ordered[index] = Some(result);
        }
    });
    ordered
        .into_iter()
        .enumerate()
        .map(|(index, result)| match result {
            Some(result) => result,
            None => isolated_crash(
                &models[index].2,
                &models[index].0,
                &models[index].1,
                Instant::now(),
                &config.kernel_sha,
                &config.manifest_sha256,
                "worker_result_missing",
                "parallel worker returned no model result",
            ),
        })
        .collect()
}

/// Aggregate slice rows into a scoreboard. The denominator is every selected
/// model, including refused, failed, crashed, and resource rows.
#[must_use]
pub fn aggregate_p85(
    results: &[P85ModelResult],
    kernel_sha: &str,
    manifest_sha256: &str,
) -> P85Scoreboard {
    let mut stages = BTreeMap::new();
    for name in P85_STAGE_NAMES {
        stages.insert(name.to_owned(), crate::StageSummary::default());
    }
    let mut failure_categories = BTreeMap::new();
    let mut passed = 0_usize;
    let mut failed = 0_usize;
    let mut refused = 0_usize;
    let mut crashed = 0_usize;
    let mut resource = 0_usize;
    let mut exact_op_exact = 0_usize;
    for result in results {
        match result.verdict {
            P85Verdict::Pass => passed += 1,
            P85Verdict::Failed => failed += 1,
            P85Verdict::Refused => refused += 1,
            P85Verdict::Crashed => crashed += 1,
            P85Verdict::Resource => resource += 1,
        }
        for (name, stage) in result.stages.named() {
            if let Some(summary) = stages.get_mut(name) {
                match stage.status {
                    StageStatus::Pass => summary.passed += 1,
                    StageStatus::Fail => summary.failed += 1,
                }
            }
        }
        if result.verdict != P85Verdict::Pass
            && let Some(diagnostic) = result
                .stages
                .named()
                .into_iter()
                .flat_map(|(_, stage)| &stage.diagnostics)
                .find(|diagnostic| diagnostic.severity == "error")
        {
            *failure_categories
                .entry(diagnostic.category.clone())
                .or_insert(0) += 1;
        }
        if result.stages.exact_op.status == StageStatus::Pass {
            exact_op_exact += 1;
        }
    }
    P85Scoreboard {
        schema_version: P85_SCHEMA_VERSION,
        kernel_sha: kernel_sha.to_owned(),
        manifest_sha256: manifest_sha256.to_owned(),
        models: results.len(),
        passed,
        failed,
        refused,
        crashed,
        resource,
        stages,
        failure_categories,
        exact_op_exact,
    }
}

/// Write JSONL rows plus JSON and Markdown scoreboards for the slice.
///
/// # Errors
///
/// Returns an I/O or serialization error when an output cannot be written.
pub fn write_p85_outputs(
    output_dir: &Path,
    results: &[P85ModelResult],
    kernel_sha: &str,
    manifest_sha256: &str,
) -> Result<(), GauntletError> {
    fs::create_dir_all(output_dir).map_err(GauntletError::io)?;
    let mut jsonl = File::create(output_dir.join("p85-models.jsonl")).map_err(GauntletError::io)?;
    for result in results {
        serde_json::to_writer(&mut jsonl, result).map_err(GauntletError::json)?;
        jsonl.write_all(b"\n").map_err(GauntletError::io)?;
    }
    let scoreboard = aggregate_p85(results, kernel_sha, manifest_sha256);
    let json = serde_json::to_vec_pretty(&scoreboard).map_err(GauntletError::json)?;
    fs::write(output_dir.join("p85-scoreboard.json"), json).map_err(GauntletError::io)?;
    fs::write(
        output_dir.join("p85-scoreboard.md"),
        p85_scoreboard_markdown(&scoreboard),
    )
    .map_err(GauntletError::io)?;
    Ok(())
}

/// Render the human-readable slice scoreboard.
#[must_use]
pub fn p85_scoreboard_markdown(scoreboard: &P85Scoreboard) -> String {
    use std::fmt::Write as _;
    let mut output = String::from("# Remus P-Class 8.5 slice scoreboard\n\n");
    let _ = writeln!(
        output,
        "Models: {} passed / {} total ({} failed, {} refused, {} crashed, {} resource).",
        scoreboard.passed,
        scoreboard.models,
        scoreboard.failed,
        scoreboard.refused,
        scoreboard.crashed,
        scoreboard.resource
    );
    let _ = writeln!(output, "Kernel: `{}`", scoreboard.kernel_sha);
    let _ = writeln!(output, "Manifest: `{}`\n", scoreboard.manifest_sha256);
    output.push_str("| Stage | Passed | Failed | Pass rate |\n");
    output.push_str("| --- | ---: | ---: | ---: |\n");
    for name in P85_STAGE_NAMES {
        let Some(summary) = scoreboard.stages.get(name) else {
            continue;
        };
        let total = summary.passed + summary.failed;
        let rate = if total == 0 {
            0.0
        } else {
            100.0 * summary.passed as f64 / total as f64
        };
        let _ = writeln!(
            output,
            "| {name} | {} | {} | {rate:.2}% |",
            summary.passed, summary.failed
        );
    }
    let _ = writeln!(
        output,
        "\nExact-only fused regions come from {} models with exact_op passing.",
        scoreboard.exact_op_exact
    );
    output.push_str("\n## Failure taxonomy\n\n");
    if scoreboard.failure_categories.is_empty() {
        output.push_str("No failures.\n");
    } else {
        for (category, count) in &scoreboard.failure_categories {
            let _ = writeln!(output, "- `{category}`: {count}");
        }
    }
    output
}

/// Build a deterministic reproduction bundle for one fixture and recipe.
///
/// The bundle pins the model bytes (SHA-256), recipe, limits, deflection,
/// timeout, manifest bytes, and the claimed kernel SHA. Replaying at an
/// arbitrary source revision is `git checkout <kernel_sha>` followed by
/// `remus-gauntlet p85-replay --model <bytes> ...` with the bundle's inputs;
/// the bundle's result is the expected outcome to compare against.
#[must_use]
pub fn replay_bundle(
    result: &P85ModelResult,
    model_size: u64,
    limits: ImportLimits,
    deflection: f64,
    model_timeout: Duration,
) -> P85ReplayBundle {
    P85ReplayBundle {
        schema_version: P85_SCHEMA_VERSION,
        kernel_sha: result.kernel_sha.clone(),
        manifest_sha256: result.manifest_sha256.clone(),
        model_id: result.model_id.clone(),
        model_sha256: result.model_sha256.clone(),
        model_size,
        import_limits: P85Limits::from(limits),
        deflection,
        model_timeout_ms: u64::try_from(model_timeout.as_millis()).unwrap_or(u64::MAX),
        recipe: result.recipe.clone(),
        result: result.clone(),
    }
}

// ---- snapshots and census ----

struct SolidSnapshot {
    volume: f64,
    area: f64,
    bbox_max: Point3,
    bbox_center: Point3,
    diagonal: f64,
    faces: u64,
    edges: u64,
    vertices: u64,
    /// Vertex positions in [`solid_vertices`] order; the rigid-motion oracle.
    vertex_positions: Vec<Point3>,
    cavities: u64,
    carriers: BTreeMap<String, u64>,
    occupancy: Vec<Option<PointClassification>>,
    probe_points: Vec<Point3>,
}

impl SolidSnapshot {
    fn capture(
        topology: &Topology,
        solid: SolidId,
        deflection: f64,
    ) -> Result<Self, DiagnosticRecord> {
        let options = PropertiesOptions::default();
        let volume = solid_volume(topology, solid, &options)
            .map(f64::abs)
            .map_err(|error| check_diagnostic(&error))?;
        let area = solid_area(topology, solid, &options)
            .map(f64::abs)
            .map_err(|error| check_diagnostic(&error))?;
        let bbox =
            solid_bounding_box(topology, solid).map_err(|error| operations_diagnostic(&error))?;
        let diagonal = (bbox.max - bbox.min).length();
        if !diagonal.is_finite() || diagonal <= 0.0 {
            return Err(error_diagnostic(
                FailureCategory::InvalidInput,
                "degenerate_model_bounds",
                "model bounding box has no finite positive diagonal",
            ));
        }
        let (faces, edges, vertices) = solid_entity_counts(topology, solid).map_err(|error| {
            error_diagnostic(
                FailureCategory::InvalidTopology,
                "topology_error",
                error.to_string(),
            )
        })?;
        let cavities = topology
            .solid(solid)
            .map(|solid| solid.inner_shells().len())
            .map_err(|error| {
                error_diagnostic(
                    FailureCategory::InvalidTopology,
                    "topology_error",
                    error.to_string(),
                )
            })?;
        let carriers = carrier_census(topology, solid).map_err(|error| {
            error_diagnostic(
                FailureCategory::InvalidTopology,
                "topology_error",
                error.to_string(),
            )
        })?;
        let probe_points = occupancy_grid(bbox.min, bbox.max);
        let mut occupancy = Vec::with_capacity(probe_points.len());
        for point in &probe_points {
            match classify_point(topology, solid, *point, deflection, 1e-6) {
                Ok(classification) => occupancy.push(Some(classification)),
                Err(_) => occupancy.push(None),
            }
        }
        let mut vertex_positions = Vec::new();
        for vertex in solid_vertices(topology, solid).map_err(|error| {
            error_diagnostic(
                FailureCategory::InvalidTopology,
                "topology_error",
                error.to_string(),
            )
        })? {
            vertex_positions.push(
                topology
                    .vertex(vertex)
                    .map_err(|error| {
                        error_diagnostic(
                            FailureCategory::InvalidTopology,
                            "topology_error",
                            error.to_string(),
                        )
                    })?
                    .point(),
            );
        }
        Ok(Self {
            volume,
            area,
            bbox_max: bbox.max,
            bbox_center: bbox.center(),
            diagonal,
            faces: as_u64(faces),
            edges: as_u64(edges),
            vertices: as_u64(vertices),
            vertex_positions,
            cavities: as_u64(cavities),
            carriers,
            occupancy,
            probe_points,
        })
    }
}

struct ModelCensus {
    faces: u64,
    edges: u64,
    vertices: u64,
    cavities: u64,
    carriers: BTreeMap<String, u64>,
}

fn solid_census(topology: &Topology, solids: &[SolidId]) -> ModelCensus {
    let mut census = ModelCensus {
        faces: 0,
        edges: 0,
        vertices: 0,
        cavities: 0,
        carriers: BTreeMap::new(),
    };
    for &solid in solids {
        if let Ok((faces, edges, vertices)) = solid_entity_counts(topology, solid) {
            census.faces += as_u64(faces);
            census.edges += as_u64(edges);
            census.vertices += as_u64(vertices);
        }
        if let Ok(solid_data) = topology.solid(solid) {
            census.cavities += as_u64(solid_data.inner_shells().len());
        }
        if let Ok(carriers) = carrier_census(topology, solid) {
            for (carrier, count) in carriers {
                *census.carriers.entry(carrier).or_insert(0) += count;
            }
        }
    }
    census
}

fn carrier_census(
    topology: &Topology,
    solid: SolidId,
) -> Result<BTreeMap<String, u64>, remus_topology::TopologyError> {
    let mut carriers = BTreeMap::new();
    for face in solid_faces(topology, solid)? {
        let tag = match topology.face(face)?.surface() {
            FaceSurface::Plane { .. } => "plane",
            FaceSurface::Nurbs(_) => "nurbs",
            FaceSurface::Cylinder(_) => "cylinder",
            FaceSurface::Cone(_) => "cone",
            FaceSurface::Sphere(_) => "sphere",
            FaceSurface::Torus(_) => "torus",
        };
        *carriers.entry(tag.to_owned()).or_insert(0) += 1;
    }
    Ok(carriers)
}

fn occupancy_grid(min: Point3, max: Point3) -> Vec<Point3> {
    let mut points = Vec::with_capacity(P85_OCCUPANCY_GRID.pow(3));
    for i in 0..P85_OCCUPANCY_GRID {
        for j in 0..P85_OCCUPANCY_GRID {
            for k in 0..P85_OCCUPANCY_GRID {
                let t = |index: usize| {
                    (f64::from(index as u32) + 0.5) / f64::from(P85_OCCUPANCY_GRID as u32)
                };
                points.push(Point3::new(
                    min.x() + (max.x() - min.x()) * t(i),
                    min.y() + (max.y() - min.y()) * t(j),
                    min.z() + (max.z() - min.z()) * t(k),
                ));
            }
        }
    }
    points
}

// ---- recipe geometry ----

fn rigid_matrix(center: Point3, diagonal: f64, recipe: &P85Recipe) -> Mat4 {
    let angle = recipe.rotation_deg.to_radians();
    let to_origin = Mat4::translation(-center.x(), -center.y(), -center.z());
    let rotation = Mat4::rotation_z(angle);
    let back = Mat4::translation(center.x(), center.y(), center.z());
    let shift = Mat4::translation(
        recipe.translation_fractions[0] * diagonal,
        recipe.translation_fractions[1] * diagonal,
        recipe.translation_fractions[2] * diagonal,
    );
    shift * back * rotation * to_origin
}

fn matrix_row_major(matrix: &Mat4) -> [f64; 16] {
    let mut rows = [0.0_f64; 16];
    for row in 0..4 {
        for column in 0..4 {
            rows[row * 4 + column] = matrix.0[row][column];
        }
    }
    rows
}

fn verify_transform(
    topology: &Topology,
    solids: &[SolidId],
    snapshots: &[SolidSnapshot],
    recipe: &P85Recipe,
    started: Instant,
) -> Result<StageResult, StageResult> {
    let mut stage = pass_stage(started);
    stage.metric(
        "transformed_count",
        MetricValue::Integer(as_u64(solids.len())),
    );
    for (index, (solid, before)) in solids.iter().zip(snapshots).enumerate() {
        let matrix = rigid_matrix(before.bbox_center, before.diagonal, recipe);
        // The rigid-motion oracle is the vertex set itself: every vertex
        // must land exactly on its mapped position, and the recomputed box
        // must match the box of the mapped vertices. AABB extents
        // legitimately change under rotation, so the pre-transform box is
        // never compared directly.
        let after = SolidSnapshot::capture(topology, *solid, crate::DEFAULT_DEFLECTION)
            .map_err(|diagnostic| fail_stage(started, diagnostic))?;
        if before.vertex_positions.len() != after.vertex_positions.len() {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "transform_topology_changed",
                    format!("rigid transform changed solid {index} vertex count"),
                ),
            ));
        }
        let mut max_vertex_shift = 0.0_f64;
        for (position, moved) in before.vertex_positions.iter().zip(&after.vertex_positions) {
            let expected = matrix.mul_point(*position);
            max_vertex_shift = max_vertex_shift.max((expected - *moved).length());
        }
        let vertex_bound = before.diagonal * 1e-9;
        stage.metric(
            &format!("solid_{index}_max_vertex_shift"),
            MetricValue::Float(max_vertex_shift),
        );
        if max_vertex_shift > vertex_bound {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "transform_vertices_moved",
                    format!("rigid transform moved solid {index} vertices by {max_vertex_shift:e}"),
                ),
            ));
        }
        // The kernel box expands the vertex box by sampling non-planar
        // faces, so it is not a rigid-motion-exact oracle and carries no
        // gate. Diagonals are recorded for provenance; the vertex set
        // above is the exact oracle.
        stage.metric(
            &format!("solid_{index}_bbox_diag_before"),
            MetricValue::Float(before.diagonal),
        );
        stage.metric(
            &format!("solid_{index}_bbox_diag_after"),
            MetricValue::Float(after.diagonal),
        );
        // Volume and area are rigid-motion invariants of the true solid; a
        // change beyond the bound is incorrect success even when the
        // vertices land exactly (placement-sensitive measurement path).
        let volume_bound = (before.volume * 1e-9).max(1e-9);
        let area_bound = (before.area * 1e-9).max(1e-9);
        if (before.volume - after.volume).abs() > volume_bound
            || (before.area - after.area).abs() > area_bound
        {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "transform_property_changed",
                    format!(
                        "rigid transform changed solid {index} volume by {:e} or area by {:e}",
                        (before.volume - after.volume).abs(),
                        (before.area - after.area).abs(),
                    ),
                ),
            ));
        }
        if before.faces != after.faces
            || before.edges != after.edges
            || before.vertices != after.vertices
            || before.cavities != after.cavities
            || before.carriers != after.carriers
        {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "transform_topology_changed",
                    format!("rigid transform changed solid {index} entity counts or carriers"),
                ),
            ));
        }
        let mut occupancy_mismatches = 0_u64;
        for (point, before_class) in before.probe_points.iter().zip(&before.occupancy) {
            let moved = matrix.mul_point(*point);
            let after_class =
                classify_point(topology, *solid, moved, crate::DEFAULT_DEFLECTION, 1e-6).ok();
            if decisive_mismatch(*before_class, after_class) {
                occupancy_mismatches += 1;
            }
        }
        if occupancy_mismatches > 0 {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "transform_occupancy_changed",
                    format!(
                        "rigid transform flipped {occupancy_mismatches} decisive occupancy probes on solid {index}"
                    ),
                ),
            ));
        }
        if validate_solid(topology, *solid, &ValidateOptions::default()).is_err() {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::InvalidTopology,
                    "transform_broke_validation",
                    format!("solid {index} fails validation after the rigid transform"),
                ),
            ));
        }
    }
    Ok(stage)
}

/// Decisive occupancy must be rigid-motion invariant. `OnBoundary` and
/// unclassified probes are excluded: a measure-zero boundary flip under
/// floating-point motion is not an incorrect success.
fn decisive_mismatch(
    before: Option<PointClassification>,
    after: Option<PointClassification>,
) -> bool {
    match (decisive_class(before), decisive_class(after)) {
        (Some(expected), Some(actual)) => expected != actual,
        (Some(_), None) => true,
        _ => false,
    }
}

fn decisive_class(class: Option<PointClassification>) -> Option<bool> {
    match class {
        Some(PointClassification::Inside) => Some(true),
        Some(PointClassification::Outside) => Some(false),
        _ => None,
    }
}

fn fuse_disjoint_box(
    topology: &mut Topology,
    solid: SolidId,
    snapshot: &SolidSnapshot,
    side: f64,
    gap_fractions: f64,
) -> Result<Vec<SolidId>, DiagnosticRecord> {
    if !side.is_finite() || side <= 0.0 {
        return Err(error_diagnostic(
            FailureCategory::InvalidInput,
            "degenerate_probe_box",
            "probe-box side is not finite and positive",
        ));
    }
    let probe =
        make_box(topology, side, side, side).map_err(|error| operations_diagnostic(&error))?;
    let gap = snapshot.diagonal * gap_fractions;
    let origin = Point3::new(
        snapshot.bbox_max.x() + gap,
        snapshot.bbox_max.y() + gap,
        snapshot.bbox_max.z() + gap,
    );
    transform_solid(
        topology,
        probe,
        &Mat4::translation(origin.x(), origin.y(), origin.z()),
    )
    .map_err(|error| operations_diagnostic(&error))?;
    let probe_bbox =
        solid_bounding_box(topology, probe).map_err(|error| operations_diagnostic(&error))?;
    if probe_bbox.min.x() <= snapshot.bbox_max.x()
        || probe_bbox.min.y() <= snapshot.bbox_max.y()
        || probe_bbox.min.z() <= snapshot.bbox_max.z()
    {
        return Err(error_diagnostic(
            FailureCategory::Internal,
            "probe_box_not_disjoint",
            "probe-box placement overlaps the operand bounding box",
        ));
    }
    let probe_volume = side * side * side;
    let probe_area = 6.0 * side * side;
    let outcome = boolean_regions(topology, BooleanOp::Fuse, solid, probe)
        .map_err(|error| operations_diagnostic(&error))?;
    let regions = topology
        .compound(outcome.compound)
        .map(|compound| compound.solids().to_vec())
        .map_err(|error| {
            error_diagnostic(
                FailureCategory::InvalidTopology,
                "topology_error",
                error.to_string(),
            )
        })?;
    if regions.len() != 2 {
        return Err(error_diagnostic(
            FailureCategory::ToleranceViolation,
            "fuse_region_count_changed",
            format!(
                "disjoint fuse produced {} regions, expected 2 (old solid plus probe box)",
                regions.len()
            ),
        ));
    }
    let options = PropertiesOptions::default();
    let mut identified_old = false;
    let mut identified_box = false;
    for region in &regions {
        validate_solid(topology, *region, &ValidateOptions::default()).map_err(|_| {
            error_diagnostic(
                FailureCategory::InvalidTopology,
                "fuse_region_invalid",
                "a disjoint-fuse region fails validation",
            )
        })?;
        let volume = solid_volume(topology, *region, &options)
            .map(f64::abs)
            .map_err(|error| check_diagnostic(&error))?;
        let area = solid_area(topology, *region, &options)
            .map(f64::abs)
            .map_err(|error| check_diagnostic(&error))?;
        let old_volume_delta = (volume - snapshot.volume).abs();
        let box_volume_delta = (volume - probe_volume).abs();
        let old_volume_bound = (snapshot.volume * 1e-6).max(1e-9);
        let box_volume_bound = (probe_volume * 1e-9).max(1e-12);
        if old_volume_delta <= old_volume_bound {
            if identified_old {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_region_ambiguous",
                    "both disjoint-fuse regions match the old solid volume",
                ));
            }
            identified_old = true;
            let area_bound = (snapshot.area * 1e-6).max(1e-9);
            if (area - snapshot.area).abs() > area_bound {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_old_region_area_changed",
                    format!(
                        "disjoint fuse changed the old region area by {:e}",
                        (area - snapshot.area).abs()
                    ),
                ));
            }
            let carriers = carrier_census(topology, *region).map_err(|error| {
                error_diagnostic(
                    FailureCategory::InvalidTopology,
                    "topology_error",
                    error.to_string(),
                )
            })?;
            if carriers != snapshot.carriers {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_old_region_carriers_changed",
                    "disjoint fuse changed the old region analytic carrier census",
                ));
            }
            let cavities = topology
                .solid(*region)
                .map(|solid| solid.inner_shells().len())
                .map_err(|error| {
                    error_diagnostic(
                        FailureCategory::InvalidTopology,
                        "topology_error",
                        error.to_string(),
                    )
                })?;
            if as_u64(cavities) != snapshot.cavities {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_old_region_cavities_changed",
                    "disjoint fuse changed the old region cavity count",
                ));
            }
            // Inclusion identity: the disjoint box must be fully outside the
            // old region, and a far point outside both.
            if !matches!(
                classify_point(topology, *region, origin, crate::DEFAULT_DEFLECTION, 1e-6),
                Ok(PointClassification::Outside)
            ) {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_occupancy_changed",
                    "the disjoint probe-box corner classifies inside the old region after fuse",
                ));
            }
        } else if box_volume_delta <= box_volume_bound {
            if identified_box {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_region_ambiguous",
                    "both disjoint-fuse regions match the probe-box volume",
                ));
            }
            identified_box = true;
            let area_bound = (probe_area * 1e-9).max(1e-12);
            if (area - probe_area).abs() > area_bound {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_box_region_area_changed",
                    format!(
                        "disjoint fuse changed the probe-box region area by {:e}",
                        (area - probe_area).abs()
                    ),
                ));
            }
            let box_center = Point3::new(
                origin.x() + side * 0.5,
                origin.y() + side * 0.5,
                origin.z() + side * 0.5,
            );
            if !matches!(
                classify_point(
                    topology,
                    *region,
                    box_center,
                    crate::DEFAULT_DEFLECTION,
                    1e-6
                ),
                Ok(PointClassification::Inside)
            ) {
                return Err(error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "fuse_occupancy_changed",
                    "the probe-box center does not classify inside its fuse region",
                ));
            }
        } else {
            return Err(error_diagnostic(
                FailureCategory::ToleranceViolation,
                "fuse_region_volume_unexpected",
                format!(
                    "a disjoint-fuse region matches neither operand volume (deltas {old_volume_delta:e} and {box_volume_delta:e})"
                ),
            ));
        }
    }
    if !identified_old || !identified_box {
        return Err(error_diagnostic(
            FailureCategory::ToleranceViolation,
            "fuse_region_unidentified",
            "disjoint fuse regions do not partition into the old solid and the probe box",
        ));
    }
    Ok(regions)
}

fn verify_export(
    step: &str,
    solid_count: usize,
    started: Instant,
) -> Result<StageResult, StageResult> {
    let mut stage = pass_stage(started);
    stage.metric("step_bytes", MetricValue::Integer(as_u64(step.len())));
    let manifold_roots = step.matches("MANIFOLD_SOLID_BREP").count();
    let void_roots = step.matches("BREP_WITH_VOIDS").count();
    let roots = manifold_roots + void_roots;
    stage.metric("solid_root_count", MetricValue::Integer(as_u64(roots)));
    if roots != solid_count {
        return Err(fail_stage(
            started,
            error_diagnostic(
                FailureCategory::ToleranceViolation,
                "export_root_count_changed",
                format!("STEP export carries {roots} solid roots for {solid_count} solids"),
            ),
        ));
    }
    if !step.contains(".MILLI.,.METRE.") {
        return Err(fail_stage(
            started,
            error_diagnostic(
                FailureCategory::ToleranceViolation,
                "export_units_not_millimetre",
                "STEP export does not declare millimetre SI length units",
            ),
        ));
    }
    // Cross-representation carrier check: count the analytic surface
    // entities the writer emitted. Planes serialize as `PLANE` (matched
    // with its assignment syntax to avoid the `PLANE_ANGLE_UNIT` header),
    // NURBS as `B_SPLINE_SURFACE*`.
    for (metric, entity) in [
        ("step_plane", "= PLANE("),
        ("step_cylindrical_surface", "CYLINDRICAL_SURFACE"),
        ("step_conical_surface", "CONICAL_SURFACE"),
        ("step_spherical_surface", "SPHERICAL_SURFACE"),
        ("step_toroidal_surface", "TOROIDAL_SURFACE"),
        ("step_b_spline_surface", "B_SPLINE_SURFACE"),
    ] {
        stage.metric(
            metric,
            MetricValue::Integer(as_u64(step.matches(entity).count())),
        );
    }
    Ok(stage)
}

fn verify_reimport(
    topology: &Topology,
    solids: &[SolidId],
    reimported_topology: &Topology,
    reimported: &[SolidId],
    config: PipelineConfig,
    started: Instant,
) -> Result<StageResult, StageResult> {
    if reimported.len() != solids.len() {
        return Err(fail_stage(
            started,
            error_diagnostic(
                FailureCategory::ToleranceViolation,
                "reimport_solid_count_changed",
                format!(
                    "STEP reimport changed solid count from {} to {}",
                    solids.len(),
                    reimported.len()
                ),
            ),
        ));
    }
    let mut stage = pass_stage(started);
    let options = PropertiesOptions::default();
    let mut max_volume_delta = 0.0_f64;
    let mut max_area_delta = 0.0_f64;
    for (index, (&original, &round_tripped)) in solids.iter().zip(reimported).enumerate() {
        let reimport_report = validate_solid(
            reimported_topology,
            round_tripped,
            &ValidateOptions::default(),
        )
        .map_err(|_| {
            fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::InvalidTopology,
                    "reimport_validation_failed",
                    format!("reimported solid {index} fails validation"),
                ),
            )
        })?;
        let trim_issues = reimport_report
            .issues
            .iter()
            .filter(|issue| {
                matches!(
                    issue.check,
                    remus_check::validate::CheckId::EdgeSameParameter
                )
            })
            .count();
        stage.metric(
            &format!("solid_{index}_trim_same_parameter_issues"),
            MetricValue::Integer(as_u64(trim_issues)),
        );
        let original_volume = solid_volume(topology, original, &options)
            .map(f64::abs)
            .map_err(|error| fail_stage(started, check_diagnostic(&error)))?;
        let original_area = solid_area(topology, original, &options)
            .map(f64::abs)
            .map_err(|error| fail_stage(started, check_diagnostic(&error)))?;
        let imported_volume = solid_volume(reimported_topology, round_tripped, &options)
            .map(f64::abs)
            .map_err(|error| fail_stage(started, check_diagnostic(&error)))?;
        let imported_area = solid_area(reimported_topology, round_tripped, &options)
            .map(f64::abs)
            .map_err(|error| fail_stage(started, check_diagnostic(&error)))?;
        let bbox = solid_bounding_box(topology, original)
            .map_err(|error| fail_stage(started, operations_diagnostic(&error)))?;
        let diagonal = (bbox.max - bbox.min).length().max(config.deflection);
        let volume_bound = (original_area * config.deflection * 4.0)
            .max(original_volume * 1e-8)
            .max(1e-12);
        let area_bound = (diagonal * config.deflection * 8.0)
            .max(original_area * 1e-8)
            .max(1e-10);
        let volume_delta = (original_volume - imported_volume).abs();
        let area_delta = (original_area - imported_area).abs();
        max_volume_delta = max_volume_delta.max(volume_delta);
        max_area_delta = max_area_delta.max(area_delta);
        if !volume_delta.is_finite()
            || !area_delta.is_finite()
            || volume_delta > volume_bound
            || area_delta > area_bound
        {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "reimport_properties_changed",
                    format!(
                        "reimported solid {index} volume delta {volume_delta:e} (bound {volume_bound:e}), area delta {area_delta:e} (bound {area_bound:e})"
                    ),
                ),
            ));
        }
        let before = carrier_census(topology, original).map_err(|error| {
            fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::InvalidTopology,
                    "topology_error",
                    error.to_string(),
                ),
            )
        })?;
        let after = carrier_census(reimported_topology, round_tripped).map_err(|error| {
            fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::InvalidTopology,
                    "topology_error",
                    error.to_string(),
                ),
            )
        })?;
        if before != after {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "reimport_carriers_changed",
                    format!("reimported solid {index} analytic carrier census changed"),
                ),
            ));
        }
        let before_cavities = topology
            .solid(original)
            .map(|solid| solid.inner_shells().len())
            .unwrap_or(usize::MAX);
        let after_cavities = reimported_topology
            .solid(round_tripped)
            .map(|solid| solid.inner_shells().len())
            .unwrap_or(usize::MAX);
        if before_cavities != after_cavities {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "reimport_cavities_changed",
                    format!("reimported solid {index} cavity count changed"),
                ),
            ));
        }
        // Occupancy cross-check on the reimported solid: the bbox center
        // keeps the classification it had before the export.
        let center = bbox.center();
        let before_class = classify_point(topology, original, center, config.deflection, 1e-6).ok();
        let after_class = classify_point(
            reimported_topology,
            round_tripped,
            center,
            config.deflection,
            1e-6,
        )
        .ok();
        if decisive_mismatch(before_class, after_class) {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "reimport_occupancy_changed",
                    format!("reimported solid {index} bbox-center occupancy flipped"),
                ),
            ));
        }
        // Independent mesh-volume cross-check (recorded, gated at 5%): the
        // tessellated mesh integrates through a different path than the
        // Gauss face integrator.
        let mesh = tessellate_solid(reimported_topology, round_tripped, config.deflection)
            .map_err(|error| fail_stage(started, operations_diagnostic(&error)))?;
        let mesh_volume = mesh_signed_volume(&mesh).abs();
        let mesh_bound = (imported_volume * 0.05).max(1e-9);
        if !mesh_volume.is_finite() || (mesh_volume - imported_volume).abs() > mesh_bound {
            return Err(fail_stage(
                started,
                error_diagnostic(
                    FailureCategory::ToleranceViolation,
                    "reimport_mesh_volume_disagrees",
                    format!(
                        "reimported solid {index} mesh volume {mesh_volume:e} disagrees with B-Rep volume {imported_volume:e}"
                    ),
                ),
            ));
        }
    }
    stage.metric("max_volume_delta", MetricValue::Float(max_volume_delta));
    stage.metric("max_area_delta", MetricValue::Float(max_area_delta));
    Ok(stage)
}

fn mesh_signed_volume(mesh: &TriangleMesh) -> f64 {
    let mut volume = 0.0_f64;
    for triangle in mesh.indices.chunks_exact(3) {
        let a = mesh.positions[triangle[0] as usize];
        let b = mesh.positions[triangle[1] as usize];
        let c = mesh.positions[triangle[2] as usize];
        let ab = Vec3::new(b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
        let ac = Vec3::new(c.x() - a.x(), c.y() - a.y(), c.z() - a.z());
        let cross = Vec3::new(
            ab.y() * ac.z() - ab.z() * ac.y(),
            ab.z() * ac.x() - ab.x() * ac.z(),
            ab.x() * ac.y() - ab.y() * ac.x(),
        );
        volume += a.x() * cross.x() + a.y() * cross.y() + a.z() * cross.z();
    }
    volume / 6.0
}

// ---- stage plumbing ----

#[allow(clippy::too_many_arguments)]
fn finish_p85(
    path: &Path,
    model_id: &str,
    model_sha256: &str,
    kernel_sha: &str,
    manifest_sha256: &str,
    recipe: P85Recipe,
    started: Instant,
    stages: P85Stages,
) -> P85ModelResult {
    let first_failing_stage = stages
        .named()
        .into_iter()
        .find(|(_, stage)| stage.status == StageStatus::Fail)
        .map(|(name, _)| name.to_owned());
    let verdict = match first_failing_stage.as_deref() {
        None => P85Verdict::Pass,
        Some(stage) => verdict_for(
            stages
                .named()
                .into_iter()
                .find(|(name, _)| *name == stage)
                .and_then(|(_, result)| {
                    result
                        .diagnostics
                        .iter()
                        .find(|diagnostic| diagnostic.severity == "error")
                }),
        ),
    };
    P85ModelResult {
        schema_version: P85_SCHEMA_VERSION,
        model: path.to_string_lossy().into_owned(),
        model_id: model_id.to_owned(),
        model_sha256: model_sha256.to_owned(),
        kernel_sha: kernel_sha.to_owned(),
        manifest_sha256: manifest_sha256.to_owned(),
        recipe,
        verdict,
        first_failing_stage,
        total_duration_ms: elapsed_ms(started),
        stages,
    }
}

fn verdict_for(diagnostic: Option<&DiagnosticRecord>) -> P85Verdict {
    let Some(diagnostic) = diagnostic else {
        return P85Verdict::Crashed;
    };
    if diagnostic.code == "worker_failed"
        || diagnostic.code == "worker_output_invalid"
        || diagnostic.code == "worker_result_missing"
        || diagnostic.code == "worker_spawn_failed"
        || diagnostic.code == "worker_wait_failed"
    {
        return P85Verdict::Crashed;
    }
    match diagnostic.category.as_str() {
        "unsupported" | "quality_refused" => P85Verdict::Refused,
        "resource_limit" | "cancelled" | "nonconvergence" => P85Verdict::Resource,
        // A returned `internal` diagnostic is an honest, inspectable failure
        // with a stable code — not a process crash. Only the worker-level
        // codes above map to `Crashed`.
        _ => P85Verdict::Failed,
    }
}

fn all_failed_after(read: &StageResult, failed_stage: &str) -> P85Stages {
    let propagated = propagated_from(read, failed_stage);
    P85Stages {
        read: read.clone(),
        validate: propagated.clone(),
        transform: propagated.clone(),
        exact_op: propagated.clone(),
        export: propagated.clone(),
        reimport: propagated,
    }
}

fn after_read(read: StageResult, validate: StageResult, failed_stage: &str) -> P85Stages {
    let propagated = propagated_from(&validate, failed_stage);
    P85Stages {
        read,
        validate,
        transform: propagated.clone(),
        exact_op: propagated.clone(),
        export: propagated.clone(),
        reimport: propagated,
    }
}

fn after_validate(
    read: StageResult,
    validate: StageResult,
    transform: StageResult,
    failed_stage: &str,
) -> P85Stages {
    let propagated = propagated_from(&transform, failed_stage);
    P85Stages {
        read,
        validate,
        transform,
        exact_op: propagated.clone(),
        export: propagated.clone(),
        reimport: propagated,
    }
}

fn after_transform(
    read: StageResult,
    validate: StageResult,
    transform: StageResult,
    exact_op: StageResult,
    failed_stage: &str,
) -> P85Stages {
    let propagated = propagated_from(&exact_op, failed_stage);
    P85Stages {
        read,
        validate,
        transform,
        exact_op,
        export: propagated.clone(),
        reimport: propagated,
    }
}

#[allow(clippy::too_many_arguments)]
fn after_exact_op(
    read: StageResult,
    validate: StageResult,
    transform: StageResult,
    exact_op: StageResult,
    export: StageResult,
    failed_stage: &str,
) -> P85Stages {
    let propagated = propagated_from(&export, failed_stage);
    P85Stages {
        read,
        validate,
        transform,
        exact_op,
        export,
        reimport: propagated,
    }
}

#[allow(clippy::too_many_arguments)]
fn after_export(
    read: StageResult,
    validate: StageResult,
    transform: StageResult,
    exact_op: StageResult,
    export: StageResult,
    reimport: StageResult,
    _failed_stage: &str,
) -> P85Stages {
    P85Stages {
        read,
        validate,
        transform,
        exact_op,
        export,
        reimport,
    }
}

fn propagated_from(stage: &StageResult, failed_stage: &str) -> StageResult {
    let diagnostic = stage
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.severity == "error")
        .cloned()
        .unwrap_or_else(|| {
            error_diagnostic(
                FailureCategory::Internal,
                "stage_failed_without_diagnostic",
                "stage failed without a structured diagnostic",
            )
        });
    StageResult {
        status: StageStatus::Fail,
        duration_ms: 0,
        diagnostics: vec![DiagnosticRecord {
            severity: "error".into(),
            category: diagnostic.category,
            code: "prerequisite_failed".into(),
            message: format!(
                "stage could not run because {failed_stage} failed with {}",
                diagnostic.code
            ),
        }],
        metrics: BTreeMap::new(),
    }
}

fn run_p85_isolated_one(
    executable: &Path,
    model: &(String, String, PathBuf),
    config: &P85RunConfig,
) -> P85ModelResult {
    let (model_id, model_sha256, path) = model;
    let started = Instant::now();
    let mut command = Command::new(executable);
    command
        .arg("p85-worker")
        .arg("--deflection")
        .arg(config.pipeline.deflection.to_string())
        .arg("--max-input-bytes")
        .arg(config.pipeline.import_limits.max_input_bytes.to_string())
        .arg("--max-model-entities")
        .arg(config.pipeline.import_limits.max_model_entities.to_string())
        .arg("--model-id")
        .arg(model_id)
        .arg("--model-sha256")
        .arg(model_sha256)
        .arg("--kernel-sha")
        .arg(&config.kernel_sha)
        .arg("--manifest-sha256")
        .arg(&config.manifest_sha256)
        .arg(path)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    let mut child = match command.spawn() {
        Ok(child) => child,
        Err(error) => {
            return isolated_crash(
                path,
                model_id,
                model_sha256,
                started,
                &config.kernel_sha,
                &config.manifest_sha256,
                "worker_spawn_failed",
                error.to_string(),
            );
        }
    };

    if config.model_timeout.is_zero() {
        let _ = child.kill();
        let _ = child.wait();
        return isolated_timeout(
            path,
            model_id,
            model_sha256,
            started,
            &config.kernel_sha,
            &config.manifest_sha256,
            config.model_timeout,
        );
    }

    let stdout_reader = child.stdout.take().map(spawn_pipe_reader);
    let stderr_reader = child.stderr.take().map(spawn_pipe_reader);

    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) if started.elapsed() >= config.model_timeout => {
                let _ = child.kill();
                let _ = child.wait();
                return isolated_timeout(
                    path,
                    model_id,
                    model_sha256,
                    started,
                    &config.kernel_sha,
                    &config.manifest_sha256,
                    config.model_timeout,
                );
            }
            Ok(None) => thread::sleep(Duration::from_millis(5)),
            Err(error) => {
                let _ = child.kill();
                let _ = child.wait();
                return isolated_crash(
                    path,
                    model_id,
                    model_sha256,
                    started,
                    &config.kernel_sha,
                    &config.manifest_sha256,
                    "worker_wait_failed",
                    error.to_string(),
                );
            }
        }
    };

    let stdout = join_pipe_reader(stdout_reader);
    let stderr = join_pipe_reader(stderr_reader);
    if !status.success() {
        let stderr = String::from_utf8_lossy(&stderr);
        return isolated_crash(
            path,
            model_id,
            model_sha256,
            started,
            &config.kernel_sha,
            &config.manifest_sha256,
            "worker_failed",
            stderr.trim(),
        );
    }
    serde_json::from_slice(&stdout).unwrap_or_else(|error| {
        isolated_crash(
            path,
            model_id,
            model_sha256,
            started,
            &config.kernel_sha,
            &config.manifest_sha256,
            "worker_output_invalid",
            error.to_string(),
        )
    })
}

fn isolated_timeout(
    path: &Path,
    model_id: &str,
    model_sha256: &str,
    started: Instant,
    kernel_sha: &str,
    manifest_sha256: &str,
    timeout: Duration,
) -> P85ModelResult {
    let diagnostic = error_diagnostic(
        FailureCategory::ResourceLimit,
        "model_wall_clock_budget_exceeded",
        format!(
            "model exceeded its {} ms wall-clock budget",
            timeout.as_millis()
        ),
    );
    finish_p85(
        path,
        model_id,
        model_sha256,
        kernel_sha,
        manifest_sha256,
        empty_recipe(),
        started,
        P85Stages {
            read: failed_with(diagnostic.clone()),
            validate: failed_with(propagated_msg(&diagnostic, "worker")),
            transform: failed_with(propagated_msg(&diagnostic, "worker")),
            exact_op: failed_with(propagated_msg(&diagnostic, "worker")),
            export: failed_with(propagated_msg(&diagnostic, "worker")),
            reimport: failed_with(propagated_msg(&diagnostic, "worker")),
        },
    )
}

#[allow(clippy::too_many_arguments)]
fn isolated_crash(
    path: &Path,
    model_id: &str,
    model_sha256: &str,
    started: Instant,
    kernel_sha: &str,
    manifest_sha256: &str,
    code: &'static str,
    message: impl Into<String>,
) -> P85ModelResult {
    let diagnostic = error_diagnostic(FailureCategory::Internal, code, message);
    finish_p85(
        path,
        model_id,
        model_sha256,
        kernel_sha,
        manifest_sha256,
        empty_recipe(),
        started,
        P85Stages {
            read: failed_with(diagnostic.clone()),
            validate: failed_with(propagated_msg(&diagnostic, "worker")),
            transform: failed_with(propagated_msg(&diagnostic, "worker")),
            exact_op: failed_with(propagated_msg(&diagnostic, "worker")),
            export: failed_with(propagated_msg(&diagnostic, "worker")),
            reimport: failed_with(propagated_msg(&diagnostic, "worker")),
        },
    )
}

fn empty_recipe() -> P85Recipe {
    P85Recipe {
        rotation_deg: P85_ROTATION_DEG,
        translation_fractions: [0.3, -0.2, 0.1],
        box_fraction: P85_BOX_FRACTION,
        box_gap_fractions: P85_BOX_GAP_FRACTIONS,
        applied_matrices: Vec::new(),
        box_sides: Vec::new(),
    }
}

// ---- small helpers ----

fn pass_stage(started: Instant) -> StageResult {
    StageResult {
        status: StageStatus::Pass,
        duration_ms: elapsed_ms(started),
        diagnostics: Vec::new(),
        metrics: BTreeMap::new(),
    }
}

fn fail_stage(started: Instant, diagnostic: DiagnosticRecord) -> StageResult {
    StageResult {
        status: StageStatus::Fail,
        duration_ms: elapsed_ms(started),
        diagnostics: vec![diagnostic],
        metrics: BTreeMap::new(),
    }
}

fn failed_with(diagnostic: DiagnosticRecord) -> StageResult {
    StageResult {
        status: StageStatus::Fail,
        duration_ms: 0,
        diagnostics: vec![diagnostic],
        metrics: BTreeMap::new(),
    }
}

fn propagated_msg(diagnostic: &DiagnosticRecord, stage: &str) -> DiagnosticRecord {
    error_diagnostic(
        category_from_str(&diagnostic.category),
        "prerequisite_failed",
        format!(
            "stage could not run because {stage} failed with {}",
            diagnostic.code
        ),
    )
}

fn error_diagnostic(
    category: FailureCategory,
    code: impl Into<String>,
    message: impl Into<String>,
) -> DiagnosticRecord {
    DiagnosticRecord {
        severity: "error".into(),
        category: category.as_str().into(),
        code: code.into(),
        message: message.into(),
    }
}

fn validate_all(topology: &Topology, solids: &[SolidId], started: Instant) -> StageResult {
    let options = ValidateOptions::default();
    let mut diagnostics = Vec::new();
    let mut errors = 0;
    let mut warnings = 0;
    let mut same_parameter_issues = 0;
    for &solid in solids {
        let report = match validate_solid(topology, solid, &options) {
            Ok(report) => report,
            Err(error) => return fail_stage(started, check_diagnostic(&error)),
        };
        for issue in report.issues {
            let code = match issue.check {
                remus_check::validate::CheckId::VertexOnCurve => "vertex_on_curve",
                remus_check::validate::CheckId::VertexOnSurface => "vertex_on_surface",
                remus_check::validate::CheckId::EdgeNoCurve3D => "edge_no_curve_3d",
                remus_check::validate::CheckId::EdgeSameParameter => "edge_same_parameter",
                remus_check::validate::CheckId::EdgeRangeValid => "edge_range_valid",
                remus_check::validate::CheckId::EdgeDegenerate => "edge_degenerate",
                remus_check::validate::CheckId::EdgeCurveDirection => "edge_curve_direction",
                remus_check::validate::CheckId::WireEmpty => "wire_empty",
                remus_check::validate::CheckId::WireNotConnected => "wire_not_connected",
                remus_check::validate::CheckId::WireClosure3D => "wire_closure_3d",
                remus_check::validate::CheckId::WireRedundantEdge => "wire_redundant_edge",
                remus_check::validate::CheckId::WireSelfIntersection => "wire_self_intersection",
                remus_check::validate::CheckId::FaceNoSurface => "face_no_surface",
                remus_check::validate::CheckId::FaceOrientationConsistency => {
                    "face_orientation_consistency"
                }
                remus_check::validate::CheckId::ShellEmpty => "shell_empty",
                remus_check::validate::CheckId::ShellConnected => "shell_connected",
                remus_check::validate::CheckId::ShellClosed => "shell_closed",
                remus_check::validate::CheckId::ShellFreeBoundary => "shell_free_boundary",
                remus_check::validate::CheckId::ShellOrientationConsistent => {
                    "shell_orientation_consistent"
                }
                remus_check::validate::CheckId::SheetOrientationConsistent => {
                    "sheet_orientation_inconsistent"
                }
                remus_check::validate::CheckId::BodyClassResolved => "body_class_unresolved",
                remus_check::validate::CheckId::SolidEulerCharacteristic => {
                    "solid_euler_characteristic"
                }
                remus_check::validate::CheckId::SolidDuplicateFaces => "solid_duplicate_faces",
                remus_check::validate::CheckId::GeometryFinite => "geometry_finite",
            };
            match issue.severity {
                Severity::Error => {
                    errors += 1;
                    diagnostics.push(error_diagnostic(
                        FailureCategory::InvalidTopology,
                        code,
                        issue.description,
                    ));
                }
                Severity::Warning => {
                    warnings += 1;
                    diagnostics.push(DiagnosticRecord {
                        severity: "warning".into(),
                        category: FailureCategory::ToleranceViolation.as_str().into(),
                        code: code.into(),
                        message: issue.description,
                    });
                }
                Severity::Info => {}
            }
            // Trim health: SameParameter deviations are the 2D/3D trim
            // agreement signal; surfaced as a dedicated metric.
            if code == "edge_same_parameter" {
                same_parameter_issues += 1;
            }
        }
    }
    let mut result = StageResult {
        status: if errors == 0 {
            StageStatus::Pass
        } else {
            StageStatus::Fail
        },
        duration_ms: elapsed_ms(started),
        diagnostics,
        metrics: BTreeMap::new(),
    };
    result.metrics.insert(
        "error_count".to_owned(),
        MetricValue::Integer(as_u64(errors)),
    );
    result.metrics.insert(
        "warning_count".to_owned(),
        MetricValue::Integer(as_u64(warnings)),
    );
    result.metrics.insert(
        "trim_same_parameter_issues".to_owned(),
        MetricValue::Integer(as_u64(same_parameter_issues)),
    );
    result
}

fn read_limited_utf8(path: &Path, max_bytes: usize) -> Result<String, DiagnosticRecord> {
    let file = File::open(path).map_err(|error| {
        error_diagnostic(
            FailureCategory::InvalidInput,
            "model_io_error",
            error.to_string(),
        )
    })?;
    let limit = u64::try_from(max_bytes).unwrap_or(u64::MAX);
    let mut bytes = Vec::new();
    file.take(limit.saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(|error| {
            error_diagnostic(
                FailureCategory::InvalidInput,
                "model_io_error",
                error.to_string(),
            )
        })?;
    if bytes.len() > max_bytes {
        return Err(error_diagnostic(
            FailureCategory::ResourceLimit,
            "import_limit_exceeded",
            format!(
                "input bytes {} exceed configured limit {max_bytes}",
                bytes.len()
            ),
        ));
    }
    String::from_utf8(bytes).map_err(|error| {
        error_diagnostic(
            FailureCategory::InvalidInput,
            "step_input_not_utf8",
            error.to_string(),
        )
    })
}

fn detect_length_unit(input: &str) -> &str {
    if input.contains(".MILLI.,.METRE.") {
        "mm"
    } else if input.contains("SI_UNIT") {
        "si-other"
    } else {
        "absent-or-undeclared"
    }
}

fn read_step_validation_count(input: &str, limits: ImportLimits) -> u64 {
    // A second bounded import requesting CAx-IF validation-property
    // comparison; only declarations actually embedded in the file count.
    // Models without embedded properties report zero (the documented
    // oracle gap): the per-solid reports always carry recomputed values,
    // but `declared` is `None` unless the file declares them.
    let options = remus_io::step::StepValidationOptions::default();
    let mut topology = Topology::new();
    match remus_io::step::read_step_with_validation(input, &mut topology, limits, options) {
        Ok(report) => as_u64(
            report
                .validation()
                .iter()
                .filter(|entry| entry.declared.is_some())
                .count(),
        ),
        Err(_) => 0,
    }
}

fn spawn_pipe_reader<R: Read + Send + 'static>(mut pipe: R) -> thread::JoinHandle<Vec<u8>> {
    thread::spawn(move || {
        let mut bytes = Vec::new();
        let _ = pipe.read_to_end(&mut bytes);
        bytes
    })
}

fn join_pipe_reader(reader: Option<thread::JoinHandle<Vec<u8>>>) -> Vec<u8> {
    reader
        .and_then(|handle| handle.join().ok())
        .unwrap_or_default()
}

fn category_from_str(category: &str) -> FailureCategory {
    match category {
        "invalid_input" => FailureCategory::InvalidInput,
        "invalid_topology" => FailureCategory::InvalidTopology,
        "unsupported" => FailureCategory::Unsupported,
        "nonconvergence" => FailureCategory::Nonconvergence,
        "resource_limit" => FailureCategory::ResourceLimit,
        "tolerance_violation" => FailureCategory::ToleranceViolation,
        "quality_refused" => FailureCategory::QualityRefused,
        "cancelled" => FailureCategory::Cancelled,
        _ => FailureCategory::Internal,
    }
}

fn io_diagnostic(error: &IoError) -> DiagnosticRecord {
    match error {
        IoError::LimitExceeded { .. } => error_diagnostic(
            FailureCategory::ResourceLimit,
            "import_limit_exceeded",
            error.to_string(),
        ),
        IoError::ParseError { .. } => error_diagnostic(
            FailureCategory::InvalidInput,
            "step_parse_error",
            error.to_string(),
        ),
        IoError::InvalidValidationProperties { code, .. } => {
            error_diagnostic(FailureCategory::InvalidInput, *code, error.to_string())
        }
        IoError::UnsupportedEntity { .. } => error_diagnostic(
            FailureCategory::Unsupported,
            "unsupported_step_entity",
            error.to_string(),
        ),
        IoError::InvalidTopology { .. } | IoError::Topology(_) => error_diagnostic(
            FailureCategory::InvalidTopology,
            "step_topology_error",
            error.to_string(),
        ),
        IoError::Io(_) => error_diagnostic(
            FailureCategory::InvalidInput,
            "model_io_error",
            error.to_string(),
        ),
        IoError::Operations(inner) => operations_diagnostic(inner),
        IoError::Zip(_) => error_diagnostic(
            FailureCategory::InvalidInput,
            "archive_error",
            error.to_string(),
        ),
    }
}

fn check_diagnostic(error: &CheckError) -> DiagnosticRecord {
    match error {
        CheckError::Topology(_) | CheckError::ValidationFailed(_) => error_diagnostic(
            FailureCategory::InvalidTopology,
            "validation_error",
            error.to_string(),
        ),
        CheckError::Math(inner) => native_diagnostic(inner.diagnostic()),
        CheckError::IntegrationFailed(_) => error_diagnostic(
            FailureCategory::Nonconvergence,
            "property_integration_failed",
            error.to_string(),
        ),
        CheckError::ClassificationFailed(_)
        | CheckError::DistanceFailed(_)
        | CheckError::CurvatureFailed(_) => error_diagnostic(
            FailureCategory::Internal,
            "check_operation_failed",
            error.to_string(),
        ),
    }
}

fn operations_diagnostic(error: &OperationsError) -> DiagnosticRecord {
    match error {
        OperationsError::ExactOnlyUnattainable => error_diagnostic(
            FailureCategory::QualityRefused,
            "exact_only_unattainable",
            error.to_string(),
        ),
        OperationsError::InvalidInput { .. } | OperationsError::EmptyResult { .. } => {
            error_diagnostic(
                FailureCategory::InvalidInput,
                "operation_invalid_input",
                error.to_string(),
            )
        }
        OperationsError::NonManifoldResult | OperationsError::Topology(_) => error_diagnostic(
            FailureCategory::InvalidTopology,
            "operation_invalid_topology",
            error.to_string(),
        ),
        OperationsError::BodyValidationFailed { .. } => error_diagnostic(
            FailureCategory::InvalidTopology,
            "body_validation_failed",
            error.to_string(),
        ),
        OperationsError::HealingValidationFailed { .. }
        | OperationsError::ConfiguredHealingValidationFailed { .. } => error_diagnostic(
            FailureCategory::InvalidTopology,
            "healing_validation_failed",
            error.to_string(),
        ),
        OperationsError::HealingVerificationUnavailable { .. }
        | OperationsError::ConfiguredHealingVerificationUnavailable { .. } => error_diagnostic(
            FailureCategory::Internal,
            "healing_verification_unavailable",
            error.to_string(),
        ),
        OperationsError::HealingRepairRefused { .. } => error_diagnostic(
            FailureCategory::Unsupported,
            "healing_repair_refused",
            error.to_string(),
        ),
        OperationsError::BodyClassMeasureMismatch { .. } => error_diagnostic(
            FailureCategory::InvalidInput,
            "body_class_measure_mismatch",
            error.to_string(),
        ),
        OperationsError::BodyClassOperationUnsupported { .. } => error_diagnostic(
            FailureCategory::Unsupported,
            "body_class_operand_unsupported",
            error.to_string(),
        ),
        OperationsError::Unsupported { .. } | OperationsError::PatternInstancesOverlap { .. } => {
            error_diagnostic(
                FailureCategory::Unsupported,
                "unsupported_configuration",
                error.to_string(),
            )
        }
        OperationsError::Math(inner) => native_diagnostic(inner.diagnostic()),
        OperationsError::Algo(inner) => native_diagnostic(inner.diagnostic()),
        OperationsError::Check(inner) => check_diagnostic(inner),
        OperationsError::Blend(_)
        | OperationsError::ResizeBlend(_)
        | OperationsError::Geometry(_)
        | OperationsError::Heal(_)
        | OperationsError::Offset(_)
        | OperationsError::PartialResult { .. } => error_diagnostic(
            FailureCategory::Internal,
            "operation_failed",
            error.to_string(),
        ),
    }
}

fn native_diagnostic(diagnostic: remus_math::diagnostic::Diagnostic) -> DiagnosticRecord {
    error_diagnostic(
        diagnostic.category(),
        diagnostic.code(),
        diagnostic.message(),
    )
}

fn elapsed_ms(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

fn as_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use super::*;
    use crate::PipelineConfig;

    fn temp_path(name: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!("remus-p85-{}-{nanos}-{name}", std::process::id()))
    }

    fn box_step() -> String {
        let mut topology = Topology::new();
        let solid = make_box(&mut topology, 10.0, 8.0, 6.0).unwrap();
        remus_io::step::write_step(&topology, &[solid]).unwrap()
    }

    fn run_box() -> P85ModelResult {
        let path = temp_path("box.step");
        let bytes = box_step();
        fs::write(&path, &bytes).unwrap();
        let sha = sha_hex(bytes.as_bytes());
        let result = process_p85_model(
            &path,
            "synthetic-box",
            &sha,
            PipelineConfig::default(),
            "test-kernel",
            "test-manifest",
        );
        fs::remove_file(&path).unwrap();
        result
    }

    fn sha_hex(bytes: &[u8]) -> String {
        use sha2::{Digest, Sha256};
        use std::fmt::Write as _;
        let digest = Sha256::digest(bytes);
        let mut hex = String::with_capacity(64);
        for byte in digest {
            let _ = write!(hex, "{byte:02x}");
        }
        hex
    }

    #[test]
    fn synthetic_box_passes_all_six_stages() {
        let result = run_box();
        assert_eq!(result.verdict, P85Verdict::Pass, "{result:#?}");
        assert!(result.first_failing_stage.is_none());
        assert_eq!(result.recipe.box_sides.len(), 1);
        assert_eq!(result.recipe.applied_matrices.len(), 1);
        assert_eq!(
            result.stages.export.metrics["solid_root_count"],
            MetricValue::Integer(2)
        );
    }

    #[test]
    fn hostile_input_limit_records_read_as_first_failure() {
        let path = temp_path("limited.step");
        let bytes = box_step();
        fs::write(&path, &bytes).unwrap();
        let config = PipelineConfig {
            import_limits: ImportLimits {
                max_input_bytes: 8,
                ..ImportLimits::default()
            },
            ..PipelineConfig::default()
        };
        let result = process_p85_model(
            &path,
            "limited-box",
            &sha_hex(bytes.as_bytes()),
            config,
            "test-kernel",
            "test-manifest",
        );
        fs::remove_file(&path).unwrap();
        assert_eq!(result.verdict, P85Verdict::Resource);
        assert_eq!(result.first_failing_stage.as_deref(), Some("read"));
        assert_eq!(result.stages.read.diagnostics[0].category, "resource_limit");
    }

    #[test]
    fn replay_bundle_round_trips_with_provenance() {
        let result = run_box();
        let bundle = replay_bundle(
            &result,
            1024,
            ImportLimits::default(),
            PipelineConfig::default().deflection,
            Duration::from_secs(60),
        );
        let json = serde_json::to_vec(&bundle).unwrap();
        let decoded: P85ReplayBundle = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, bundle);
        assert_eq!(decoded.kernel_sha, "test-kernel");
        assert_eq!(decoded.result.verdict, P85Verdict::Pass);
    }

    #[test]
    fn scoreboard_keeps_refused_models_in_the_denominator() {
        let pass = run_box();
        let mut refused = pass.clone();
        refused.verdict = P85Verdict::Refused;
        refused.first_failing_stage = Some("exact_op".to_owned());
        refused.stages.exact_op = failed_with(error_diagnostic(
            FailureCategory::QualityRefused,
            "exact_only_unattainable",
            "refused",
        ));
        let scoreboard = aggregate_p85(&[pass, refused], "test-kernel", "test-manifest");
        assert_eq!(scoreboard.models, 2);
        assert_eq!(scoreboard.passed, 1);
        assert_eq!(scoreboard.refused, 1);
        assert_eq!(scoreboard.failed, 0);
        assert!(p85_scoreboard_markdown(&scoreboard).contains("`quality_refused`: 1"));
    }
}
