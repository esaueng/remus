//! Isolated, per-model robustness gauntlet.

pub mod manifest;
pub mod trend;

use std::collections::BTreeMap;
use std::ffi::OsStr;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use remus_check::CheckError;
use remus_check::properties::{PropertiesOptions, solid_area, solid_volume};
use remus_check::validate::{CheckId, Severity, ValidateOptions, validate_solid};
use remus_io::{ImportLimits, IoError};
use remus_math::diagnostic::{FailureCategory, ToDiagnostic};
use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanOutcome, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_bounding_box;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, is_watertight, non_manifold_edge_count, tessellate_solid,
    welded_mesh_quality,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;
use serde::{Deserialize, Serialize};

/// Stable report schema version.
pub const SCHEMA_VERSION: u32 = 1;

/// Default wall-clock budget for one isolated model.
pub const DEFAULT_MODEL_TIMEOUT: Duration = Duration::from_secs(60);

/// Default tessellation and round-trip comparison deflection.
pub const DEFAULT_DEFLECTION: f64 = 0.05;

/// Configuration applied inside one model worker.
#[derive(Debug, Clone, Copy)]
pub struct PipelineConfig {
    /// Hostile-input limits applied to both STEP reads.
    pub import_limits: ImportLimits,
    /// Tessellation deflection and property-comparison scale.
    pub deflection: f64,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            import_limits: ImportLimits::default(),
            deflection: DEFAULT_DEFLECTION,
        }
    }
}

/// Parent-process configuration.
#[derive(Debug, Clone, Copy)]
pub struct RunConfig {
    /// Configuration passed to each worker.
    pub pipeline: PipelineConfig,
    /// Hard wall-clock budget for the worker process.
    pub model_timeout: Duration,
    /// Maximum isolated model workers executed concurrently.
    pub max_parallel_models: usize,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            pipeline: PipelineConfig::default(),
            model_timeout: DEFAULT_MODEL_TIMEOUT,
            max_parallel_models: 1,
        }
    }
}

/// Pass/fail state of one pipeline stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StageStatus {
    /// The stage completed and satisfied its invariant.
    Pass,
    /// The stage failed or could not run because an earlier stage failed.
    Fail,
}

/// Structured diagnostic emitted by the gauntlet.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DiagnosticRecord {
    /// `error` or `warning`.
    pub severity: String,
    /// Stable failure-taxonomy category.
    pub category: String,
    /// Stable lowercase code.
    pub code: String,
    /// Human-readable context; never parsed as a contract.
    pub message: String,
}

impl DiagnosticRecord {
    fn error(
        category: FailureCategory,
        code: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            severity: "error".into(),
            category: category.as_str().into(),
            code: code.into(),
            message: message.into(),
        }
    }

    fn warning(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            severity: "warning".into(),
            category: FailureCategory::ToleranceViolation.as_str().into(),
            code: code.into(),
            message: message.into(),
        }
    }
}

/// Typed metric value used by stage reports.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MetricValue {
    /// Integer count.
    Integer(u64),
    /// Floating-point measurement.
    Float(f64),
    /// Boolean flag.
    Bool(bool),
    /// Text label.
    Text(String),
    /// Floating-point series.
    Floats(Vec<f64>),
}

/// Result of one pipeline stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StageResult {
    /// Pass/fail state.
    pub status: StageStatus,
    /// Wall time spent in this stage.
    pub duration_ms: u64,
    /// Stable structured diagnostics.
    pub diagnostics: Vec<DiagnosticRecord>,
    /// Stage-specific measurements.
    pub metrics: BTreeMap<String, MetricValue>,
}

impl StageResult {
    fn pass(started: Instant) -> Self {
        Self {
            status: StageStatus::Pass,
            duration_ms: elapsed_ms(started),
            diagnostics: Vec::new(),
            metrics: BTreeMap::new(),
        }
    }

    fn fail(started: Instant, diagnostic: DiagnosticRecord) -> Self {
        Self {
            status: StageStatus::Fail,
            duration_ms: elapsed_ms(started),
            diagnostics: vec![diagnostic],
            metrics: BTreeMap::new(),
        }
    }

    fn failed_with(diagnostic: DiagnosticRecord) -> Self {
        Self {
            status: StageStatus::Fail,
            duration_ms: 0,
            diagnostics: vec![diagnostic],
            metrics: BTreeMap::new(),
        }
    }

    fn metric(&mut self, key: &str, value: MetricValue) {
        self.metrics.insert(key.to_owned(), value);
    }
}

/// The five required per-model stages.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PipelineStages {
    /// Bounded STEP import.
    pub read: StageResult,
    /// Per-solid L3 validation.
    pub validate: StageResult,
    /// Centered probe cut with exact/fallback disclosure.
    pub boolean: StageResult,
    /// Watertight/manifold tessellation.
    pub tessellate: StageResult,
    /// STEP round-trip property comparison.
    pub round_trip: StageResult,
}

impl PipelineStages {
    fn all_pass(&self) -> bool {
        [
            &self.read,
            &self.validate,
            &self.boolean,
            &self.tessellate,
            &self.round_trip,
        ]
        .into_iter()
        .all(|stage| stage.status == StageStatus::Pass)
    }

    fn named(&self) -> [(&'static str, &StageResult); 5] {
        [
            ("read", &self.read),
            ("validate", &self.validate),
            ("boolean", &self.boolean),
            ("tessellate", &self.tessellate),
            ("round_trip", &self.round_trip),
        ]
    }
}

/// One JSONL row for one model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelResult {
    /// Report schema version.
    pub schema_version: u32,
    /// Input path supplied to the runner.
    pub model: String,
    /// True only when every stage passes.
    pub passed: bool,
    /// End-to-end worker time.
    pub total_duration_ms: u64,
    /// Required stage results.
    pub stages: PipelineStages,
}

/// Aggregate counts for one stage.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct StageSummary {
    /// Passing model count.
    pub passed: usize,
    /// Failing model count.
    pub failed: usize,
}

/// Machine-readable aggregate scoreboard.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Scoreboard {
    /// Report schema version.
    pub schema_version: u32,
    /// Total model count.
    pub models: usize,
    /// Models passing every stage.
    pub passed: usize,
    /// Models failing at least one stage.
    pub failed: usize,
    /// Per-stage pass/fail counts.
    pub stages: BTreeMap<String, StageSummary>,
    /// Primary failure count by stable taxonomy category.
    pub failure_categories: BTreeMap<String, usize>,
    /// Probe booleans completed on the exact path.
    pub boolean_exact: usize,
    /// Probe booleans completed through disclosed approximation.
    pub boolean_approximate: usize,
}

/// Run the full five-stage pipeline for one model in the current process.
///
/// Production callers should use [`run_models_isolated`]; this entry point is
/// public for deterministic unit tests and worker execution.
#[must_use]
pub fn process_model(path: &Path, config: PipelineConfig) -> ModelResult {
    let total_started = Instant::now();
    let model = path.to_string_lossy().into_owned();

    let read_started = Instant::now();
    let input = match read_limited_utf8(path, config.import_limits.max_input_bytes) {
        Ok(input) => input,
        Err(diagnostic) => {
            return failed_model(
                model,
                total_started,
                StageResult::fail(read_started, diagnostic.clone()),
                diagnostic,
            );
        }
    };
    let mut topology = Topology::new();
    let solids =
        match remus_io::step::read_step_with_limits(&input, &mut topology, config.import_limits) {
            Ok(solids) if !solids.is_empty() => solids,
            Ok(_) => {
                let diagnostic = DiagnosticRecord::error(
                    FailureCategory::InvalidInput,
                    "step_contains_no_solids",
                    "STEP input contains no solid B-Reps",
                );
                return failed_model(
                    model,
                    total_started,
                    StageResult::fail(read_started, diagnostic.clone()),
                    diagnostic,
                );
            }
            Err(error) => {
                let diagnostic = io_diagnostic(&error);
                return failed_model(
                    model,
                    total_started,
                    StageResult::fail(read_started, diagnostic.clone()),
                    diagnostic,
                );
            }
        };
    let mut read = StageResult::pass(read_started);
    read.metric("solid_count", MetricValue::Integer(as_u64(solids.len())));
    read.metric("input_bytes", MetricValue::Integer(as_u64(input.len())));

    let validate_started = Instant::now();
    let validate = validate_all(&topology, &solids, validate_started);
    if validate.status == StageStatus::Fail {
        let diagnostic = primary_diagnostic(&validate);
        return failed_after_read(model, total_started, read, validate, diagnostic);
    }

    let boolean_started = Instant::now();
    let (boolean, outcomes) = probe_all(&mut topology, &solids, boolean_started);
    if boolean.status == StageStatus::Fail {
        let diagnostic = primary_diagnostic(&boolean);
        return failed_after_boolean(model, total_started, read, validate, boolean, diagnostic);
    }

    let result_solids: Vec<_> = outcomes.iter().map(|outcome| outcome.solid).collect();
    let tessellate_started = Instant::now();
    let tessellate = tessellate_all(
        &topology,
        &result_solids,
        config.deflection,
        tessellate_started,
    );
    if tessellate.status == StageStatus::Fail {
        let diagnostic = primary_diagnostic(&tessellate);
        let stages = PipelineStages {
            read,
            validate,
            boolean,
            tessellate,
            round_trip: StageResult::failed_with(propagated(&diagnostic, "tessellate")),
        };
        return finish_model(model, total_started, stages);
    }

    let round_trip_started = Instant::now();
    let round_trip = round_trip_all(&topology, &result_solids, config, round_trip_started);
    let stages = PipelineStages {
        read,
        validate,
        boolean,
        tessellate,
        round_trip,
    };
    finish_model(model, total_started, stages)
}

/// Run models in isolated worker subprocesses with a hard per-model timeout.
/// Results preserve input order even when more than one worker runs.
#[must_use]
pub fn run_models_isolated(
    executable: &Path,
    models: &[PathBuf],
    config: RunConfig,
) -> Vec<ModelResult> {
    if models.is_empty() {
        return Vec::new();
    }
    let workers = config.max_parallel_models.max(1).min(models.len());
    if workers == 1 {
        return models
            .iter()
            .map(|model| run_isolated_model(executable, model, config))
            .collect();
    }

    let mut ordered: Vec<Option<ModelResult>> =
        std::iter::repeat_with(|| None).take(models.len()).collect();
    let (sender, receiver) = std::sync::mpsc::channel();
    std::thread::scope(|scope| {
        for worker in 0..workers {
            let sender = sender.clone();
            scope.spawn(move || {
                for index in (worker..models.len()).step_by(workers) {
                    let result = run_isolated_model(executable, &models[index], config);
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
            None => isolated_failure(
                &models[index],
                Instant::now(),
                FailureCategory::Internal,
                "worker_result_missing",
                "parallel worker returned no model result",
            ),
        })
        .collect()
}

/// Aggregate model rows into a scoreboard.
#[must_use]
pub fn aggregate(results: &[ModelResult]) -> Scoreboard {
    let mut stages = BTreeMap::new();
    for name in ["read", "validate", "boolean", "tessellate", "round_trip"] {
        stages.insert(name.to_owned(), StageSummary::default());
    }

    let mut failure_categories = BTreeMap::new();
    let mut boolean_exact = 0;
    let mut boolean_approximate = 0;
    for result in results {
        for (name, stage) in result.stages.named() {
            if let Some(summary) = stages.get_mut(name) {
                match stage.status {
                    StageStatus::Pass => summary.passed += 1,
                    StageStatus::Fail => summary.failed += 1,
                }
            }
        }
        if !result.passed
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
        boolean_exact += metric_usize(&result.stages.boolean, "exact_count");
        boolean_approximate += metric_usize(&result.stages.boolean, "approximate_count");
    }

    let passed = results.iter().filter(|result| result.passed).count();
    Scoreboard {
        schema_version: SCHEMA_VERSION,
        models: results.len(),
        passed,
        failed: results.len().saturating_sub(passed),
        stages,
        failure_categories,
        boolean_exact,
        boolean_approximate,
    }
}

/// Write JSONL rows plus JSON and Markdown scoreboards.
///
/// # Errors
///
/// Returns an I/O or serialization error when an output cannot be written.
pub fn write_outputs(output_dir: &Path, results: &[ModelResult]) -> Result<(), GauntletError> {
    fs::create_dir_all(output_dir).map_err(GauntletError::io)?;

    let mut jsonl = File::create(output_dir.join("models.jsonl")).map_err(GauntletError::io)?;
    for result in results {
        serde_json::to_writer(&mut jsonl, result).map_err(GauntletError::json)?;
        jsonl.write_all(b"\n").map_err(GauntletError::io)?;
    }

    let scoreboard = aggregate(results);
    let json = serde_json::to_vec_pretty(&scoreboard).map_err(GauntletError::json)?;
    fs::write(output_dir.join("scoreboard.json"), json).map_err(GauntletError::io)?;
    fs::write(
        output_dir.join("scoreboard.md"),
        scoreboard_markdown(&scoreboard),
    )
    .map_err(GauntletError::io)?;
    Ok(())
}

/// Render the human-readable scoreboard.
#[must_use]
pub fn scoreboard_markdown(scoreboard: &Scoreboard) -> String {
    let mut output = String::from("# Remus gauntlet scoreboard\n\n");
    let overall_rate = pass_rate(scoreboard.passed, scoreboard.models);
    output.push_str("Models: ");
    output.push_str(&scoreboard.passed.to_string());
    output.push_str(" passed / ");
    output.push_str(&scoreboard.models.to_string());
    output.push_str(" total (");
    output.push_str(&format_percentage(overall_rate));
    output.push_str("%).\n\n");
    output.push_str("| Stage | Passed | Failed | Pass rate |\n");
    output.push_str("| --- | ---: | ---: | ---: |\n");
    for name in ["read", "validate", "boolean", "tessellate", "round_trip"] {
        let Some(summary) = scoreboard.stages.get(name) else {
            continue;
        };
        let total = summary.passed + summary.failed;
        let rate = pass_rate(summary.passed, total);
        output.push_str("| ");
        output.push_str(name);
        output.push_str(" | ");
        output.push_str(&summary.passed.to_string());
        output.push_str(" | ");
        output.push_str(&summary.failed.to_string());
        output.push_str(" | ");
        output.push_str(&format_percentage(rate));
        output.push_str("% |\n");
    }
    output.push_str("\n## Boolean quality\n\n");
    output.push_str("- Exact: ");
    output.push_str(&scoreboard.boolean_exact.to_string());
    output.push_str("\n- Approximate: ");
    output.push_str(&scoreboard.boolean_approximate.to_string());
    output.push('\n');
    output.push_str("\n## Failure taxonomy\n\n");
    if scoreboard.failure_categories.is_empty() {
        output.push_str("No failures.\n");
    } else {
        for (category, count) in &scoreboard.failure_categories {
            output.push_str("- `");
            output.push_str(category);
            output.push_str("`: ");
            output.push_str(&count.to_string());
            output.push('\n');
        }
    }
    output
}

/// Tool-level error for output and CLI infrastructure failures.
#[derive(Debug)]
pub struct GauntletError {
    message: String,
}

impl GauntletError {
    fn io(error: std::io::Error) -> Self {
        Self {
            message: error.to_string(),
        }
    }

    fn json(error: serde_json::Error) -> Self {
        Self {
            message: error.to_string(),
        }
    }

    /// Construct an argument or infrastructure error.
    #[must_use]
    pub fn message(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl std::fmt::Display for GauntletError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl std::error::Error for GauntletError {}

fn validate_all(topology: &Topology, solids: &[SolidId], started: Instant) -> StageResult {
    let options = ValidateOptions::default();
    let mut diagnostics = Vec::new();
    let mut errors = 0;
    let mut warnings = 0;
    for &solid in solids {
        let report = match validate_solid(topology, solid, &options) {
            Ok(report) => report,
            Err(error) => return StageResult::fail(started, check_diagnostic(&error)),
        };
        for issue in report.issues {
            let code = check_id_code(issue.check);
            match issue.severity {
                Severity::Error => {
                    errors += 1;
                    diagnostics.push(DiagnosticRecord::error(
                        FailureCategory::InvalidTopology,
                        code,
                        issue.description,
                    ));
                }
                Severity::Warning => {
                    warnings += 1;
                    diagnostics.push(DiagnosticRecord::warning(code, issue.description));
                }
                Severity::Info => {}
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
    result.metric("error_count", MetricValue::Integer(as_u64(errors)));
    result.metric("warning_count", MetricValue::Integer(as_u64(warnings)));
    result
}

fn probe_all(
    topology: &mut Topology,
    solids: &[SolidId],
    started: Instant,
) -> (StageResult, Vec<BooleanOutcome>) {
    let mut outcomes = Vec::with_capacity(solids.len());
    let mut exact_count = 0;
    let mut approximate_count = 0;
    let mut deflections = Vec::new();
    for &solid in solids {
        let bbox = match solid_bounding_box(topology, solid) {
            Ok(bbox) => bbox,
            Err(error) => {
                return (
                    StageResult::fail(started, operations_diagnostic(&error)),
                    outcomes,
                );
            }
        };
        let diagonal = (bbox.max - bbox.min).length();
        let side = diagonal * 0.5;
        if !side.is_finite() || side <= 0.0 {
            return (
                StageResult::fail(
                    started,
                    DiagnosticRecord::error(
                        FailureCategory::InvalidInput,
                        "degenerate_model_bounds",
                        "model bounding box has no finite positive diagonal",
                    ),
                ),
                outcomes,
            );
        }
        let probe = match make_box(topology, side, side, side) {
            Ok(probe) => probe,
            Err(error) => {
                return (
                    StageResult::fail(started, operations_diagnostic(&error)),
                    outcomes,
                );
            }
        };
        let center = bbox.center();
        let origin = center - remus_math::vec::Vec3::new(side * 0.5, side * 0.5, side * 0.5);
        let translation = Mat4::translation(origin.x(), origin.y(), origin.z());
        if let Err(error) = transform_solid(topology, probe, &translation) {
            return (
                StageResult::fail(started, operations_diagnostic(&error)),
                outcomes,
            );
        }
        let context = remus_math::context::OperationContext::new();
        let outcome = match boolean_with_context(topology, BooleanOp::Cut, solid, probe, &context) {
            Ok(outcome) => outcome,
            Err(error) => {
                return (
                    StageResult::fail(started, operations_diagnostic(&error)),
                    outcomes,
                );
            }
        };
        match outcome.quality {
            BooleanQuality::Exact => exact_count += 1,
            BooleanQuality::Approximate { deflection } => {
                approximate_count += 1;
                deflections.push(deflection);
            }
        }
        outcomes.push(outcome);
    }
    let mut result = StageResult::pass(started);
    result.metric("exact_count", MetricValue::Integer(as_u64(exact_count)));
    result.metric(
        "approximate_count",
        MetricValue::Integer(as_u64(approximate_count)),
    );
    result.metric("approximate_deflections", MetricValue::Floats(deflections));
    (result, outcomes)
}

fn tessellate_all(
    topology: &Topology,
    solids: &[SolidId],
    deflection: f64,
    started: Instant,
) -> StageResult {
    if !deflection.is_finite() || deflection <= 0.0 {
        return StageResult::fail(
            started,
            DiagnosticRecord::error(
                FailureCategory::InvalidInput,
                "invalid_deflection",
                "deflection must be finite and positive",
            ),
        );
    }
    let mut triangles = 0;
    let mut index_boundary = 0;
    let mut index_non_manifold = 0;
    let mut welded_boundary = 0;
    let mut welded_non_manifold = 0;
    for &solid in solids {
        let mesh = match tessellate_solid(topology, solid, deflection) {
            Ok(mesh) => mesh,
            Err(error) => return StageResult::fail(started, operations_diagnostic(&error)),
        };
        let boundary = boundary_edge_count(&mesh);
        let non_manifold = non_manifold_edge_count(&mesh);
        let welded = welded_mesh_quality(&mesh);
        triangles += mesh.indices.len() / 3;
        index_boundary += boundary;
        index_non_manifold += non_manifold;
        welded_boundary += welded.boundary_edges;
        welded_non_manifold += welded.non_manifold_edges;
        if !is_watertight(&mesh) || !welded.is_watertight() {
            let mut result = StageResult::fail(
                started,
                DiagnosticRecord::error(
                    FailureCategory::ToleranceViolation,
                    "tessellation_not_watertight",
                    format!(
                        "mesh has {boundary} indexed boundary edges, {non_manifold} indexed non-manifold edges, {} welded boundary edges, and {} welded non-manifold edges",
                        welded.boundary_edges, welded.non_manifold_edges
                    ),
                ),
            );
            add_mesh_metrics(
                &mut result,
                triangles,
                index_boundary,
                index_non_manifold,
                welded_boundary,
                welded_non_manifold,
            );
            return result;
        }
    }
    let mut result = StageResult::pass(started);
    add_mesh_metrics(
        &mut result,
        triangles,
        index_boundary,
        index_non_manifold,
        welded_boundary,
        welded_non_manifold,
    );
    result
}

fn round_trip_all(
    topology: &Topology,
    solids: &[SolidId],
    config: PipelineConfig,
    started: Instant,
) -> StageResult {
    let step = match remus_io::step::write_step(topology, solids) {
        Ok(step) => step,
        Err(error) => return StageResult::fail(started, io_diagnostic(&error)),
    };
    let mut imported_topology = Topology::new();
    let imported = match remus_io::step::read_step_with_limits(
        &step,
        &mut imported_topology,
        config.import_limits,
    ) {
        Ok(imported) => imported,
        Err(error) => return StageResult::fail(started, io_diagnostic(&error)),
    };
    if imported.len() != solids.len() {
        return StageResult::fail(
            started,
            DiagnosticRecord::error(
                FailureCategory::ToleranceViolation,
                "round_trip_solid_count_changed",
                format!(
                    "STEP round-trip changed solid count from {} to {}",
                    solids.len(),
                    imported.len()
                ),
            ),
        );
    }

    let options = PropertiesOptions::default();
    let mut max_volume_delta = 0.0_f64;
    let mut max_area_delta = 0.0_f64;
    let mut max_volume_bound = 0.0_f64;
    let mut max_area_bound = 0.0_f64;
    for (&original, &round_tripped) in solids.iter().zip(&imported) {
        let original_volume = match solid_volume(topology, original, &options) {
            Ok(value) => value.abs(),
            Err(error) => return StageResult::fail(started, check_diagnostic(&error)),
        };
        let original_area = match solid_area(topology, original, &options) {
            Ok(value) => value.abs(),
            Err(error) => return StageResult::fail(started, check_diagnostic(&error)),
        };
        let imported_volume = match solid_volume(&imported_topology, round_tripped, &options) {
            Ok(value) => value.abs(),
            Err(error) => return StageResult::fail(started, check_diagnostic(&error)),
        };
        let imported_area = match solid_area(&imported_topology, round_tripped, &options) {
            Ok(value) => value.abs(),
            Err(error) => return StageResult::fail(started, check_diagnostic(&error)),
        };
        let bbox = match solid_bounding_box(topology, original) {
            Ok(bbox) => bbox,
            Err(error) => return StageResult::fail(started, operations_diagnostic(&error)),
        };
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
        max_volume_bound = max_volume_bound.max(volume_bound);
        max_area_bound = max_area_bound.max(area_bound);
        if !volume_delta.is_finite()
            || !area_delta.is_finite()
            || volume_delta > volume_bound
            || area_delta > area_bound
        {
            return StageResult::fail(
                started,
                DiagnosticRecord::error(
                    FailureCategory::ToleranceViolation,
                    "round_trip_properties_changed",
                    format!(
                        "STEP round-trip volume delta {volume_delta:e} (bound {volume_bound:e}), area delta {area_delta:e} (bound {area_bound:e})"
                    ),
                ),
            );
        }
    }
    let mut result = StageResult::pass(started);
    result.metric("step_bytes", MetricValue::Integer(as_u64(step.len())));
    result.metric("max_volume_delta", MetricValue::Float(max_volume_delta));
    result.metric("max_volume_bound", MetricValue::Float(max_volume_bound));
    result.metric("max_area_delta", MetricValue::Float(max_area_delta));
    result.metric("max_area_bound", MetricValue::Float(max_area_bound));
    result
}

fn run_isolated_model(executable: &Path, model: &Path, config: RunConfig) -> ModelResult {
    let started = Instant::now();
    let mut command = Command::new(executable);
    command
        .arg("worker")
        .arg("--deflection")
        .arg(config.pipeline.deflection.to_string())
        .arg("--max-input-bytes")
        .arg(config.pipeline.import_limits.max_input_bytes.to_string())
        .arg("--max-model-entities")
        .arg(config.pipeline.import_limits.max_model_entities.to_string())
        .arg(model)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    let mut child = match command.spawn() {
        Ok(child) => child,
        Err(error) => {
            return isolated_failure(
                model,
                started,
                FailureCategory::Internal,
                "worker_spawn_failed",
                error.to_string(),
            );
        }
    };

    if config.model_timeout.is_zero() {
        let _ = child.kill();
        let _ = child.wait();
        return isolated_failure(
            model,
            started,
            FailureCategory::ResourceLimit,
            "model_wall_clock_budget_exceeded",
            "model exceeded its 0 ms wall-clock budget",
        );
    }

    // Drain both pipes on background threads while polling: a worker whose
    // JSON row outgrows the OS pipe buffer would otherwise block on write,
    // never exit, and be misreported as a wall-clock budget failure.
    let stdout_reader = child.stdout.take().map(spawn_pipe_reader);
    let stderr_reader = child.stderr.take().map(spawn_pipe_reader);

    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) if started.elapsed() >= config.model_timeout => {
                let _ = child.kill();
                let _ = child.wait();
                return isolated_failure(
                    model,
                    started,
                    FailureCategory::ResourceLimit,
                    "model_wall_clock_budget_exceeded",
                    format!(
                        "model exceeded its {} ms wall-clock budget",
                        config.model_timeout.as_millis()
                    ),
                );
            }
            Ok(None) => thread::sleep(Duration::from_millis(5)),
            Err(error) => {
                let _ = child.kill();
                let _ = child.wait();
                return isolated_failure(
                    model,
                    started,
                    FailureCategory::Internal,
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
        return isolated_failure(
            model,
            started,
            FailureCategory::Internal,
            "worker_failed",
            stderr.trim(),
        );
    }
    serde_json::from_slice(&stdout).unwrap_or_else(|error| {
        isolated_failure(
            model,
            started,
            FailureCategory::Internal,
            "worker_output_invalid",
            error.to_string(),
        )
    })
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

fn read_limited_utf8(path: &Path, max_bytes: usize) -> Result<String, DiagnosticRecord> {
    let file = File::open(path).map_err(|error| {
        DiagnosticRecord::error(
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
            DiagnosticRecord::error(
                FailureCategory::InvalidInput,
                "model_io_error",
                error.to_string(),
            )
        })?;
    if bytes.len() > max_bytes {
        return Err(DiagnosticRecord::error(
            FailureCategory::ResourceLimit,
            "import_limit_exceeded",
            format!(
                "input bytes {} exceed configured limit {max_bytes}",
                bytes.len()
            ),
        ));
    }
    String::from_utf8(bytes).map_err(|error| {
        DiagnosticRecord::error(
            FailureCategory::InvalidInput,
            "step_input_not_utf8",
            error.to_string(),
        )
    })
}

fn failed_model(
    model: String,
    started: Instant,
    read: StageResult,
    diagnostic: DiagnosticRecord,
) -> ModelResult {
    let stages = PipelineStages {
        read,
        validate: StageResult::failed_with(propagated(&diagnostic, "read")),
        boolean: StageResult::failed_with(propagated(&diagnostic, "read")),
        tessellate: StageResult::failed_with(propagated(&diagnostic, "read")),
        round_trip: StageResult::failed_with(propagated(&diagnostic, "read")),
    };
    finish_model(model, started, stages)
}

fn failed_after_read(
    model: String,
    started: Instant,
    read: StageResult,
    validate: StageResult,
    diagnostic: DiagnosticRecord,
) -> ModelResult {
    let stages = PipelineStages {
        read,
        validate,
        boolean: StageResult::failed_with(propagated(&diagnostic, "validate")),
        tessellate: StageResult::failed_with(propagated(&diagnostic, "validate")),
        round_trip: StageResult::failed_with(propagated(&diagnostic, "validate")),
    };
    finish_model(model, started, stages)
}

fn failed_after_boolean(
    model: String,
    started: Instant,
    read: StageResult,
    validate: StageResult,
    boolean: StageResult,
    diagnostic: DiagnosticRecord,
) -> ModelResult {
    let stages = PipelineStages {
        read,
        validate,
        boolean,
        tessellate: StageResult::failed_with(propagated(&diagnostic, "boolean")),
        round_trip: StageResult::failed_with(propagated(&diagnostic, "boolean")),
    };
    finish_model(model, started, stages)
}

fn finish_model(model: String, started: Instant, stages: PipelineStages) -> ModelResult {
    ModelResult {
        schema_version: SCHEMA_VERSION,
        model,
        passed: stages.all_pass(),
        total_duration_ms: elapsed_ms(started),
        stages,
    }
}

fn isolated_failure(
    model: &Path,
    started: Instant,
    category: FailureCategory,
    code: &'static str,
    message: impl Into<String>,
) -> ModelResult {
    let diagnostic = DiagnosticRecord::error(category, code, message);
    let stages = PipelineStages {
        read: StageResult::failed_with(diagnostic.clone()),
        validate: StageResult::failed_with(propagated(&diagnostic, "worker")),
        boolean: StageResult::failed_with(propagated(&diagnostic, "worker")),
        tessellate: StageResult::failed_with(propagated(&diagnostic, "worker")),
        round_trip: StageResult::failed_with(propagated(&diagnostic, "worker")),
    };
    finish_model(model.to_string_lossy().into_owned(), started, stages)
}

fn propagated(diagnostic: &DiagnosticRecord, stage: &str) -> DiagnosticRecord {
    DiagnosticRecord::error(
        category_from_str(&diagnostic.category),
        "prerequisite_failed",
        format!(
            "stage could not run because {stage} failed with {}",
            diagnostic.code
        ),
    )
}

fn primary_diagnostic(stage: &StageResult) -> DiagnosticRecord {
    stage
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.severity == "error")
        .cloned()
        .unwrap_or_else(|| {
            DiagnosticRecord::error(
                FailureCategory::Internal,
                "stage_failed_without_diagnostic",
                "stage failed without a structured diagnostic",
            )
        })
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
        IoError::LimitExceeded { .. } => DiagnosticRecord::error(
            FailureCategory::ResourceLimit,
            "import_limit_exceeded",
            error.to_string(),
        ),
        IoError::ParseError { .. } => DiagnosticRecord::error(
            FailureCategory::InvalidInput,
            "step_parse_error",
            error.to_string(),
        ),
        IoError::UnsupportedEntity { .. } => DiagnosticRecord::error(
            FailureCategory::Unsupported,
            "unsupported_step_entity",
            error.to_string(),
        ),
        IoError::InvalidTopology { .. } | IoError::Topology(_) => DiagnosticRecord::error(
            FailureCategory::InvalidTopology,
            "step_topology_error",
            error.to_string(),
        ),
        IoError::Io(_) => DiagnosticRecord::error(
            FailureCategory::InvalidInput,
            "model_io_error",
            error.to_string(),
        ),
        IoError::Operations(inner) => operations_diagnostic(inner),
        IoError::Zip(_) => DiagnosticRecord::error(
            FailureCategory::InvalidInput,
            "archive_error",
            error.to_string(),
        ),
    }
}

fn check_diagnostic(error: &CheckError) -> DiagnosticRecord {
    match error {
        CheckError::Topology(_) | CheckError::ValidationFailed(_) => DiagnosticRecord::error(
            FailureCategory::InvalidTopology,
            "validation_error",
            error.to_string(),
        ),
        CheckError::Math(inner) => diagnostic_record(inner.diagnostic()),
        CheckError::IntegrationFailed(_) => DiagnosticRecord::error(
            FailureCategory::Nonconvergence,
            "property_integration_failed",
            error.to_string(),
        ),
        CheckError::ClassificationFailed(_)
        | CheckError::DistanceFailed(_)
        | CheckError::CurvatureFailed(_) => DiagnosticRecord::error(
            FailureCategory::Internal,
            "check_operation_failed",
            error.to_string(),
        ),
    }
}

fn operations_diagnostic(error: &OperationsError) -> DiagnosticRecord {
    match error {
        OperationsError::ExactOnlyUnattainable => DiagnosticRecord::error(
            FailureCategory::QualityRefused,
            "exact_only_unattainable",
            error.to_string(),
        ),
        OperationsError::InvalidInput { .. } | OperationsError::EmptyResult { .. } => {
            DiagnosticRecord::error(
                FailureCategory::InvalidInput,
                "operation_invalid_input",
                error.to_string(),
            )
        }
        OperationsError::NonManifoldResult | OperationsError::Topology(_) => {
            DiagnosticRecord::error(
                FailureCategory::InvalidTopology,
                "operation_invalid_topology",
                error.to_string(),
            )
        }
        OperationsError::Unsupported { .. } | OperationsError::PatternInstancesOverlap { .. } => {
            DiagnosticRecord::error(
                FailureCategory::Unsupported,
                "unsupported_configuration",
                error.to_string(),
            )
        }
        OperationsError::Math(inner) => diagnostic_record(inner.diagnostic()),
        OperationsError::Algo(inner) => diagnostic_record(inner.diagnostic()),
        OperationsError::Check(inner) => check_diagnostic(inner),
        OperationsError::Blend(_)
        | OperationsError::ResizeBlend(_)
        | OperationsError::Geometry(_)
        | OperationsError::Heal(_)
        | OperationsError::Offset(_)
        | OperationsError::PartialResult { .. } => DiagnosticRecord::error(
            FailureCategory::Internal,
            "operation_failed",
            error.to_string(),
        ),
    }
}

fn diagnostic_record(diagnostic: remus_math::diagnostic::Diagnostic) -> DiagnosticRecord {
    DiagnosticRecord::error(
        diagnostic.category(),
        diagnostic.code(),
        diagnostic.message(),
    )
}

fn check_id_code(check: CheckId) -> &'static str {
    match check {
        CheckId::VertexOnCurve => "vertex_on_curve",
        CheckId::VertexOnSurface => "vertex_on_surface",
        CheckId::EdgeNoCurve3D => "edge_no_curve_3d",
        CheckId::EdgeSameParameter => "edge_same_parameter",
        CheckId::EdgeRangeValid => "edge_range_valid",
        CheckId::EdgeDegenerate => "edge_degenerate",
        CheckId::WireEmpty => "wire_empty",
        CheckId::WireNotConnected => "wire_not_connected",
        CheckId::WireClosure3D => "wire_closure_3d",
        CheckId::WireRedundantEdge => "wire_redundant_edge",
        CheckId::WireSelfIntersection => "wire_self_intersection",
        CheckId::FaceNoSurface => "face_no_surface",
        CheckId::FaceOrientationConsistency => "face_orientation_consistency",
        CheckId::ShellEmpty => "shell_empty",
        CheckId::ShellConnected => "shell_connected",
        CheckId::ShellClosed => "shell_closed",
        CheckId::ShellOrientationConsistent => "shell_orientation_consistent",
        CheckId::SolidEulerCharacteristic => "solid_euler_characteristic",
        CheckId::SolidDuplicateFaces => "solid_duplicate_faces",
        CheckId::GeometryFinite => "geometry_finite",
    }
}

fn add_mesh_metrics(
    result: &mut StageResult,
    triangles: usize,
    index_boundary: usize,
    index_non_manifold: usize,
    welded_boundary: usize,
    welded_non_manifold: usize,
) {
    result.metric("triangle_count", MetricValue::Integer(as_u64(triangles)));
    result.metric(
        "boundary_edge_count",
        MetricValue::Integer(as_u64(index_boundary)),
    );
    result.metric(
        "non_manifold_edge_count",
        MetricValue::Integer(as_u64(index_non_manifold)),
    );
    result.metric(
        "welded_boundary_edge_count",
        MetricValue::Integer(as_u64(welded_boundary)),
    );
    result.metric(
        "welded_non_manifold_edge_count",
        MetricValue::Integer(as_u64(welded_non_manifold)),
    );
}

fn elapsed_ms(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

fn as_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

fn metric_usize(stage: &StageResult, key: &str) -> usize {
    match stage.metrics.get(key) {
        Some(MetricValue::Integer(value)) => usize::try_from(*value).unwrap_or(usize::MAX),
        _ => 0,
    }
}

fn pass_rate(passed: usize, total: usize) -> f64 {
    if total == 0 {
        0.0
    } else {
        100.0 * passed as f64 / total as f64
    }
}

fn format_percentage(value: f64) -> String {
    format!("{value:.2}")
}

/// Return true when an argument exactly matches an ASCII flag.
#[must_use]
pub fn arg_is(value: &OsStr, expected: &str) -> bool {
    value == OsStr::new(expected)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use super::*;

    fn temp_path(name: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!(
            "remus-gauntlet-{}-{nanos}-{name}",
            std::process::id()
        ))
    }

    fn box_step() -> String {
        let mut topology = Topology::new();
        let solid = make_box(&mut topology, 10.0, 8.0, 6.0).unwrap();
        remus_io::step::write_step(&topology, &[solid]).unwrap()
    }

    #[test]
    fn box_model_passes_all_five_stages() {
        let path = temp_path("box.step");
        fs::write(&path, box_step()).unwrap();
        let result = process_model(&path, PipelineConfig::default());
        fs::remove_file(&path).unwrap();

        assert!(result.passed, "{result:#?}");
        assert_eq!(result.stages.boolean.status, StageStatus::Pass);
        assert_eq!(metric_usize(&result.stages.boolean, "exact_count"), 1);
        assert_eq!(
            result.stages.tessellate.metrics["boundary_edge_count"],
            MetricValue::Integer(0)
        );
        assert_eq!(result.stages.round_trip.status, StageStatus::Pass);
    }

    #[test]
    fn hostile_input_limit_is_a_resource_limit() {
        let path = temp_path("limited.step");
        fs::write(&path, box_step()).unwrap();
        let config = PipelineConfig {
            import_limits: ImportLimits {
                max_input_bytes: 8,
                ..ImportLimits::default()
            },
            ..PipelineConfig::default()
        };
        let result = process_model(&path, config);
        fs::remove_file(&path).unwrap();

        assert!(!result.passed);
        assert_eq!(result.stages.read.status, StageStatus::Fail);
        assert_eq!(result.stages.read.diagnostics[0].category, "resource_limit");
        assert_eq!(
            result.stages.read.diagnostics[0].code,
            "import_limit_exceeded"
        );
    }

    #[test]
    fn aggregate_counts_primary_failure_once() {
        let diagnostic = DiagnosticRecord::error(
            FailureCategory::InvalidInput,
            "step_parse_error",
            "bad STEP",
        );
        let failed = failed_model(
            "bad.step".into(),
            Instant::now(),
            StageResult::failed_with(diagnostic.clone()),
            diagnostic,
        );
        let scoreboard = aggregate(&[failed]);
        assert_eq!(scoreboard.models, 1);
        assert_eq!(scoreboard.failed, 1);
        assert_eq!(scoreboard.failure_categories["invalid_input"], 1);
        assert!(scoreboard_markdown(&scoreboard).contains("`invalid_input`: 1"));
    }
}
