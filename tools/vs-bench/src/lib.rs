//! Versioned, fail-closed scorecards. Runner execution belongs to O1.2a.
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Wire version for job specifications, observations, and generated reports.
pub const SCHEMA_VERSION: u32 = 1;
/// Mutually exclusive outcomes, in report order.
pub const OUTCOMES: [&str; 11] = [
    "correct_success",
    "exact_success",
    "disclosed_approximate_success",
    "verified_repair_success",
    "typed_refusal",
    "untyped_error",
    "silent_wrong",
    "invalid_success",
    "crash",
    "hang_or_budget_overrun",
    "nondeterminism",
];
/// Metric groups and their mandatory columns. Inapplicable measurements use null.
pub const GROUPS: &[(&str, &[&str])] = &[
    (
        "history",
        &["evolution_completeness", "persistent_ref_survival"],
    ),
    (
        "interchange",
        &[
            "import_validity",
            "post_import_operation_success",
            "round_trip_geometry_fidelity",
            "assembly_metadata_fidelity",
        ],
    ),
    (
        "geometry_quality",
        &[
            "tessellation_watertight",
            "volume_error",
            "area_error",
            "centroid_error",
            "inertia_error",
        ],
    ),
    (
        "resources",
        &[
            "runtime_median",
            "runtime_p95",
            "peak_memory",
            "entity_growth",
            "cancellation_latency",
        ],
    ),
    (
        "browser",
        &[
            "wasm_cold_init",
            "module_size_raw",
            "module_size_gzip",
            "module_size_brotli",
            "native_wasm_agreement",
        ],
    ),
    ("concurrency", &["thread_scaling_efficiency"]),
];

/// Invalid input is distinct from a well-formed report containing failed gates.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// Malformed or unknown JSON fields.
    #[error("invalid_json: {0}")]
    Json(#[from] serde_json::Error),
    /// A schema or evidence contract was violated.
    #[error("invalid_scorecard: {0}")]
    Contract(String),
}

/// Complete batch, with explicit expected scenario/kernel pairs.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Run {
    schema_version: u32,
    #[serde(rename = "run_id")]
    id: String,
    repetitions: u32,
    harness_sha: String,
    manifest_sha256: String,
    kernels: Vec<String>,
    scenarios: Vec<Scenario>,
    observations: Vec<Observation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    id: String,
    oracle: String,
    quality: Quality,
    applicable_metrics: Vec<String>,
    topology_producing: bool,
    native_wasm_required: bool,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Quality {
    representation: Representation,
    deflection: f64,
    tolerance_model: String,
    error_budget: f64,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum Representation {
    Exact,
    Approximate,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    scenario: String,
    kernel: String,
    reported: Reported,
    diagnostic: Option<String>,
    oracle_agrees: Option<bool>,
    validator_accepts: Option<bool>,
    repeat_agrees: bool,
    quality: Quality,
    approximation: Option<Approximation>,
    repairs: Vec<Repair>,
    repair_occurred: bool,
    evolution_explicit: bool,
    defect_repro: Option<String>,
    metrics: BTreeMap<String, BTreeMap<String, Value>>,
}

#[derive(Debug, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Reported {
    CorrectSuccess,
    ExactSuccess,
    ApproximateSuccess,
    RepairedSuccess,
    Refusal,
    Error,
    Crash,
    Hang,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Approximation {
    method: String,
    error_bound: f64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Repair {
    code: String,
    count: u64,
    verified: bool,
}

/// Generated result, including all failed absolute gates even without a competitor.
#[derive(Debug, Serialize)]
pub struct Report {
    /// Report protocol version.
    pub schema_version: u32,
    /// Caller-assigned run identity.
    pub run_id: String,
    /// Number of repeated executions represented by each observation.
    pub repetitions: u32,
    /// Reproducibility identity supplied by the harness.
    pub harness_sha: String,
    /// Existing gauntlet manifest content identity.
    pub manifest_sha256: String,
    /// True only if every row passes every gate.
    pub passed: bool,
    /// Per-scenario, per-kernel results.
    pub rows: Vec<Row>,
}

/// A single mutually exclusive outcome plus measurements and pass/fail gates.
#[derive(Debug, Serialize)]
pub struct Row {
    /// Scenario identity.
    pub scenario: String,
    /// Kernel/build identity.
    pub kernel: String,
    /// Explicit oracle from the job specification.
    pub oracle: String,
    /// Stable producer diagnostic, retained for refusal analysis.
    pub diagnostic: Option<String>,
    /// Permanent defect reproduction reference when applicable.
    pub defect_repro: Option<String>,
    /// Declared output-quality requirement.
    quality_requirement: Quality,
    approximation: Option<Approximation>,
    repairs: Vec<Repair>,
    /// Eleven one-hot outcome columns.
    pub outcomes: BTreeMap<String, u8>,
    /// All metric groups; timing fields are absent unless comparison is admissible.
    pub metrics: BTreeMap<String, BTreeMap<String, Value>>,
    /// Absolute gate verdicts, independent of competitor scores.
    pub gates: BTreeMap<String, bool>,
}

fn require(condition: bool, reason: impl Into<String>) -> Result<(), Error> {
    if condition {
        Ok(())
    } else {
        Err(Error::Contract(reason.into()))
    }
}

fn named(value: &str) -> bool {
    !value.trim().is_empty()
}
fn digest(value: &str, length: usize) -> bool {
    value.len() == length && value.bytes().all(|b| b.is_ascii_hexdigit())
}
fn valid_quality(quality: &Quality) -> bool {
    quality.deflection.is_finite()
        && quality.deflection > 0.0
        && quality.error_budget.is_finite()
        && quality.error_budget >= 0.0
        && named(&quality.tolerance_model)
}
fn success(reported: &Reported) -> bool {
    matches!(
        reported,
        Reported::CorrectSuccess
            | Reported::ExactSuccess
            | Reported::ApproximateSuccess
            | Reported::RepairedSuccess
    )
}
fn outcome(observation: &Observation) -> &'static str {
    // Oracle disagreement must never be hidden by another failure or a claimed quality.
    if success(&observation.reported) && observation.oracle_agrees == Some(false) {
        return "silent_wrong";
    }
    if success(&observation.reported) && observation.validator_accepts == Some(false) {
        return "invalid_success";
    }
    if !observation.repeat_agrees {
        return "nondeterminism";
    }
    match observation.reported {
        Reported::CorrectSuccess => "correct_success",
        Reported::ExactSuccess => "exact_success",
        Reported::ApproximateSuccess => "disclosed_approximate_success",
        Reported::RepairedSuccess => "verified_repair_success",
        Reported::Refusal if observation.diagnostic.as_deref().is_some_and(named) => {
            "typed_refusal"
        }
        Reported::Refusal | Reported::Error => "untyped_error",
        Reported::Crash => "crash",
        Reported::Hang => "hang_or_budget_overrun",
    }
}

fn validate_metrics(observation: &Observation, scenario: &Scenario) -> Result<(), Error> {
    require(
        observation.metrics.len() == GROUPS.len(),
        "metric group mismatch",
    )?;
    for &(group, columns) in GROUPS {
        let values = observation
            .metrics
            .get(group)
            .ok_or_else(|| Error::Contract(format!("missing metric group {group}")))?;
        require(
            values.len() == columns.len(),
            format!("column mismatch in {group}"),
        )?;
        for &column in columns {
            let value = values
                .get(column)
                .ok_or_else(|| Error::Contract(format!("missing metric {column}")))?;
            let applicable = scenario.applicable_metrics.iter().any(|m| m == column);
            require(
                applicable != value.is_null(),
                format!("applicability mismatch for {column}"),
            )?;
            if !applicable {
                continue;
            }
            if matches!(
                column,
                "round_trip_geometry_fidelity" | "assembly_metadata_fidelity"
            ) {
                let components: &[&str] = if column == "round_trip_geometry_fidelity" {
                    &[
                        "volume_error",
                        "area_error",
                        "centroid_error",
                        "bounds_error",
                    ]
                } else {
                    &["tree", "transforms", "names", "colors", "materials"]
                };
                let object = value
                    .as_object()
                    .ok_or_else(|| Error::Contract(format!("{column} must be an object")))?;
                require(
                    object.len() == components.len(),
                    format!("incomplete {column}"),
                )?;
                for component in components {
                    let number =
                        object
                            .get(*component)
                            .and_then(Value::as_f64)
                            .ok_or_else(|| {
                                Error::Contract(format!("missing numeric {column}.{component}"))
                            })?;
                    require(
                        number.is_finite()
                            && number >= 0.0
                            && (column != "assembly_metadata_fidelity" || number <= 1.0),
                        format!("invalid {column}.{component}"),
                    )?;
                }
                continue;
            }
            let boolean = matches!(
                column,
                "import_validity"
                    | "post_import_operation_success"
                    | "tessellation_watertight"
                    | "native_wasm_agreement"
            );
            if boolean {
                require(value.is_boolean(), format!("{column} must be boolean"))?;
            } else {
                let number = value
                    .as_f64()
                    .ok_or_else(|| Error::Contract(format!("{column} must be numeric")))?;
                require(
                    number.is_finite() && number >= 0.0,
                    format!("invalid {column}"),
                )?;
                if matches!(
                    column,
                    "evolution_completeness"
                        | "persistent_ref_survival"
                        | "assembly_metadata_fidelity"
                ) {
                    require(number <= 1.0, format!("{column} exceeds one"))?;
                }
            }
        }
    }
    let resources = &observation.metrics["resources"];
    if let (Some(median), Some(p95)) = (
        resources["runtime_median"].as_f64(),
        resources["runtime_p95"].as_f64(),
    ) {
        require(p95 >= median, "runtime_p95 below median")?;
    }
    Ok(())
}

/// Parse a strict versioned observation batch and evaluate its absolute gates.
///
/// # Errors
/// Rejects unknown versions, malformed evidence, missing pairs or metric columns.
pub fn evaluate_json(input: &str) -> Result<Report, Error> {
    let run: Run = serde_json::from_str(input)?;
    evaluate(run)
}

/// Evaluate a decoded batch without trusting producer-supplied outcome labels.
///
/// # Errors
/// Returns a contract error for ambiguous or incomplete job specifications.
#[allow(clippy::too_many_lines)]
pub fn evaluate(run: Run) -> Result<Report, Error> {
    require(
        run.schema_version == SCHEMA_VERSION,
        "schema_version mismatch",
    )?;
    require(
        named(&run.id) && run.repetitions >= 2,
        "run identity and at least two repetitions required",
    )?;
    require(
        digest(&run.harness_sha, 40),
        "harness_sha must be a full commit SHA",
    )?;
    require(
        digest(&run.manifest_sha256, 64),
        "manifest_sha256 must be a content hash",
    )?;
    require(
        !run.kernels.is_empty() && !run.scenarios.is_empty(),
        "empty job",
    )?;
    let kernels: BTreeSet<_> = run.kernels.iter().collect();
    require(
        kernels.len() == run.kernels.len() && kernels.iter().all(|k| named(k)),
        "duplicate or empty kernel",
    )?;
    let known_metrics: BTreeSet<_> = GROUPS.iter().flat_map(|(_, c)| c.iter().copied()).collect();
    let mut scenarios = BTreeMap::new();
    for scenario in &run.scenarios {
        require(
            named(&scenario.id) && named(&scenario.oracle) && valid_quality(&scenario.quality),
            "invalid scenario identity, oracle or quality",
        )?;
        require(
            scenarios.insert(&scenario.id, scenario).is_none(),
            "duplicate scenario",
        )?;
        let applicable: BTreeSet<_> = scenario
            .applicable_metrics
            .iter()
            .map(String::as_str)
            .collect();
        require(
            applicable.len() == scenario.applicable_metrics.len()
                && applicable.is_subset(&known_metrics),
            "invalid applicable metrics",
        )?;
        require(
            applicable.contains("runtime_median") == applicable.contains("runtime_p95"),
            "timing requires both median and p95",
        )?;
        require(
            !scenario.native_wasm_required || applicable.contains("native_wasm_agreement"),
            "native/WASM evidence required",
        )?;
        require(
            !scenario.topology_producing || applicable.contains("evolution_completeness"),
            "evolution evidence required",
        )?;
    }
    let mut pairs = BTreeSet::new();
    let mut rows = Vec::new();
    for observation in &run.observations {
        let scenario = scenarios
            .get(&observation.scenario)
            .ok_or_else(|| Error::Contract("unknown scenario".into()))?;
        require(kernels.contains(&observation.kernel), "unknown kernel")?;
        require(
            pairs.insert((&observation.scenario, &observation.kernel)),
            "duplicate observation",
        )?;
        require(
            valid_quality(&observation.quality),
            "invalid observed quality",
        )?;
        if success(&observation.reported) {
            require(
                observation.oracle_agrees.is_some() && observation.validator_accepts.is_some(),
                "success requires oracle and validator evidence",
            )?;
        }
        validate_metrics(observation, scenario)?;
        let approximation_disclosed = observation.approximation.as_ref().is_some_and(|a| {
            named(&a.method)
                && a.error_bound.is_finite()
                && a.error_bound >= 0.0
                && a.error_bound <= scenario.quality.error_budget
        });
        let approximate = success(&observation.reported)
            && (observation.reported == Reported::ApproximateSuccess
                || observation.quality.representation == Representation::Approximate
                || observation.approximation.is_some());
        let repair_disclosed = !observation.repairs.is_empty()
            && observation
                .repairs
                .iter()
                .all(|r| named(&r.code) && r.count > 0 && r.verified);
        let repaired = observation.repair_occurred
            || observation.reported == Reported::RepairedSuccess
            || !observation.repairs.is_empty();
        let classified = outcome(observation);
        let code = if success(&observation.reported)
            && !matches!(
                classified,
                "silent_wrong" | "invalid_success" | "nondeterminism"
            )
            && ((approximate && !approximation_disclosed) || (repaired && !repair_disclosed))
        {
            "untyped_error"
        } else {
            classified
        };
        let mut gates = BTreeMap::from([
            ("no_silent_wrong".into(), code != "silent_wrong"),
            ("valid_success".into(), code != "invalid_success"),
            (
                "no_crash_or_hang".into(),
                !matches!(observation.reported, Reported::Crash | Reported::Hang),
            ),
            (
                "approximation_disclosed".into(),
                !approximate || approximation_disclosed,
            ),
            (
                "repair_disclosed_verified".into(),
                !repaired || repair_disclosed,
            ),
            (
                "typed_non_success".into(),
                success(&observation.reported)
                    || observation.diagnostic.as_deref().is_some_and(named),
            ),
            ("no_untyped_error".into(), code != "untyped_error"),
            ("deterministic".into(), observation.repeat_agrees),
            (
                "native_wasm_agreement".into(),
                !scenario.native_wasm_required
                    || observation.metrics["browser"]["native_wasm_agreement"] == true,
            ),
            (
                "evolution_explicit".into(),
                !scenario.topology_producing || observation.evolution_explicit,
            ),
            ("reproducible_identity".into(), true),
            (
                "quality_requirement".into(),
                !success(&observation.reported) || observation.quality == scenario.quality,
            ),
            (
                "quality_label_consistent".into(),
                !((observation.reported == Reported::ExactSuccess && approximate)
                    || (observation.reported == Reported::ApproximateSuccess
                        && observation.quality.representation != Representation::Approximate)
                    || (repaired
                        && success(&observation.reported)
                        && observation.reported != Reported::RepairedSuccess)),
            ),
        ]);
        let defect = gates.values().any(|passed| !passed);
        gates.insert(
            "defect_reproduction".into(),
            !defect || observation.defect_repro.as_deref().is_some_and(named),
        );
        rows.push(Row {
            scenario: observation.scenario.clone(),
            kernel: observation.kernel.clone(),
            oracle: scenario.oracle.clone(),
            diagnostic: observation.diagnostic.clone(),
            defect_repro: observation.defect_repro.clone(),
            quality_requirement: scenario.quality.clone(),
            approximation: observation.approximation.clone(),
            repairs: observation.repairs.clone(),
            outcomes: OUTCOMES
                .iter()
                .map(|&o| (o.into(), u8::from(o == code)))
                .collect(),
            metrics: observation.metrics.clone(),
            gates,
        });
    }
    require(
        pairs.len() == run.scenarios.len() * run.kernels.len(),
        "missing scenario/kernel observation",
    )?;
    // A refusal stays in the outcome table but cannot enter a speed comparison.
    for scenario in scenarios.keys() {
        let comparable = kernels.len() >= 2
            && rows.iter().filter(|r| &r.scenario == *scenario).all(|r| {
                [
                    "correct_success",
                    "exact_success",
                    "disclosed_approximate_success",
                ]
                .iter()
                .any(|o| r.outcomes[*o] == 1)
                    && r.gates.values().all(|v| *v)
            });
        if !comparable {
            for row in rows.iter_mut().filter(|r| &r.scenario == *scenario) {
                if let Some(resources) = row.metrics.get_mut("resources") {
                    resources.remove("runtime_median");
                    resources.remove("runtime_p95");
                }
            }
        }
    }
    rows.sort_by(|a, b| (&a.scenario, &a.kernel).cmp(&(&b.scenario, &b.kernel)));
    Ok(Report {
        schema_version: SCHEMA_VERSION,
        run_id: run.id,
        repetitions: run.repetitions,
        harness_sha: run.harness_sha,
        manifest_sha256: run.manifest_sha256,
        passed: rows.iter().all(|r| r.gates.values().all(|v| *v)),
        rows,
    })
}
