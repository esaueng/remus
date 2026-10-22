//! Bounded aggregate triage hints, without model names or diagnostic messages.
//!
//! STEP parse errors currently carry prose rather than typed subcodes. Family
//! recognition below is deliberately conservative and is never a parser contract
//! or an input-validity decision. Unrecognized or changed prose remains unknown.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{DiagnosticRecord, GauntletError, ModelResult, StageStatus};

/// Finite hints for selecting the next import investigation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImportFailureFamily {
    /// File access or text encoding failed before STEP parsing.
    InputAccess,
    /// A configured budget or worker timeout was exceeded.
    ResourceLimit,
    /// STEP lexical or section structure was refused.
    Syntax,
    /// Length/angle unit declarations or owning contexts were refused.
    UnitsOrContext,
    /// A referenced entity was absent.
    MissingReference,
    /// A curve endpoint did not satisfy its carrier's acceptance bound.
    EndpointAuthority,
    /// An authoritative curve trim/domain could not be established.
    CurveDomain,
    /// A per-face parameter curve was refused.
    Pcurve,
    /// An explicit face-bound reconstruction was refused.
    FaceBoundary,
    /// The importer reported an unsupported entity or construction.
    UnsupportedEntity,
    /// No solid roots were returned by the solid-only importer.
    NoSolids,
    /// No recognized, bounded hint is available.
    Unknown,
}

impl ImportFailureFamily {
    /// Stable label used by the aggregate report.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InputAccess => "input_access",
            Self::ResourceLimit => "resource_limit",
            Self::Syntax => "syntax",
            Self::UnitsOrContext => "units_or_context",
            Self::MissingReference => "missing_reference",
            Self::EndpointAuthority => "endpoint_authority",
            Self::CurveDomain => "curve_domain",
            Self::Pcurve => "pcurve",
            Self::FaceBoundary => "face_boundary",
            Self::UnsupportedEntity => "unsupported_entity",
            Self::NoSolids => "no_solids",
            Self::Unknown => "unknown",
        }
    }
}

/// First-failure counts that can be retained after per-model rows are discarded.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FailureDetails {
    /// One primary diagnostic code per failed model, grouped by first stage.
    pub first_failure_codes: BTreeMap<String, BTreeMap<String, usize>>,
    /// Exactly one finite hint per model whose first failing stage is import.
    pub import_families: BTreeMap<ImportFailureFamily, usize>,
}

impl FailureDetails {
    pub(crate) fn record(&mut self, stage: &str, diagnostic: &DiagnosticRecord) {
        let code = safe_code(&diagnostic.code);
        *self
            .first_failure_codes
            .entry(stage.to_owned())
            .or_default()
            .entry(code.to_owned())
            .or_default() += 1;
        if stage == "read" {
            *self
                .import_families
                .entry(import_family(diagnostic))
                .or_default() += 1;
        }
    }

    pub(crate) fn validate(&self, failed: usize, read_failed: usize) -> Result<(), GauntletError> {
        let invalid = || GauntletError::message("gauntlet_trend_inconsistent_failure_details");
        let mut first_failures = 0_usize;
        let mut read_failures = 0_usize;
        for (stage, codes) in &self.first_failure_codes {
            if !matches!(
                stage.as_str(),
                "read" | "validate" | "boolean" | "tessellate" | "round_trip" | "worker"
            ) {
                return Err(invalid());
            }
            for (code, &count) in codes {
                if count == 0 || safe_code(code) != code {
                    return Err(invalid());
                }
                first_failures = first_failures.checked_add(count).ok_or_else(invalid)?;
                if stage == "read" {
                    read_failures = read_failures.checked_add(count).ok_or_else(invalid)?;
                }
            }
        }
        let import_failures = self
            .import_families
            .values()
            .try_fold(0_usize, |sum, &count| {
                if count == 0 {
                    None
                } else {
                    sum.checked_add(count)
                }
            });
        if first_failures != failed
            || read_failures != read_failed
            || import_failures != Some(read_failed)
        {
            return Err(invalid());
        }
        Ok(())
    }
}

pub(crate) fn first_failure(result: &ModelResult) -> (&'static str, DiagnosticRecord) {
    for (name, stage) in result.stages.named() {
        if stage.status == StageStatus::Fail {
            return (name, crate::primary_diagnostic(stage));
        }
    }
    // A malformed worker result must still remain in the failure denominator.
    (
        "worker",
        DiagnosticRecord::error(
            remus_math::diagnostic::FailureCategory::Internal,
            "inconsistent_model_result",
            "worker marked an all-passing result as failed",
        ),
    )
}

fn safe_code(code: &str) -> &str {
    if !code.is_empty()
        && code.len() <= 96
        && code
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
    {
        code
    } else {
        "unclassified_error"
    }
}

fn import_family(diagnostic: &DiagnosticRecord) -> ImportFailureFamily {
    use ImportFailureFamily as Family;

    match diagnostic.code.as_str() {
        "import_limit_exceeded" | "model_wall_clock_budget_exceeded" => {
            return Family::ResourceLimit;
        }
        "model_io_error" | "step_input_not_utf8" => return Family::InputAccess,
        "unsupported_step_entity" => return Family::UnsupportedEntity,
        "step_contains_no_solids" => return Family::NoSolids,
        "step_parse_error" => {}
        _ => return Family::Unknown,
    }
    // Never retain prose. Limit inspection too, so a malicious or unexpectedly
    // verbose diagnostic cannot turn aggregate triage into unbounded scanning.
    if diagnostic.message.len() > 8_192 {
        return Family::Unknown;
    }
    let Some(reason) = diagnostic.message.strip_prefix("parse error: ") else {
        return Family::Unknown;
    };
    if matches!(reason, "no DATA section found" | "no ENDSEC after DATA")
        || reason.starts_with("unterminated STEP ")
    {
        Family::Syntax
    } else if reason.starts_with("PCURVE #") {
        Family::Pcurve
    } else if reason.starts_with("EDGE_CURVE #") && reason.contains("endpoint misses its carrier") {
        Family::EndpointAuthority
    } else if reason.starts_with("EDGE_CURVE #")
        && (reason.contains("untrimmed NURBS") || reason.contains("parameter range"))
    {
        Family::CurveDomain
    } else if reason.starts_with("FACE_BOUND ")
        || (reason.starts_with("ADVANCED_FACE #") && reason.contains("bound"))
    {
        Family::FaceBoundary
    } else if reason.starts_with("STEP file declares no LENGTH_UNIT")
        || reason.starts_with("SI_UNIT #")
        || reason.starts_with("GLOBAL_UNIT_ASSIGNED_CONTEXT #")
        || reason.starts_with("unit entity #")
        || (reason.starts_with("solid B-Rep #")
            && reason.ends_with("has no unambiguous shape-representation context"))
    {
        Family::UnitsOrContext
    } else if reason.starts_with("entity #") && reason.ends_with(" not found") {
        Family::MissingReference
    } else {
        Family::Unknown
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use super::*;
    use crate::{PipelineStages, StageResult};
    use remus_math::diagnostic::FailureCategory;
    use std::time::Instant;

    fn result_with_failure(stage: &str, diagnostic: DiagnosticRecord) -> ModelResult {
        let mut stages = PipelineStages {
            read: StageResult::pass(Instant::now()),
            validate: StageResult::pass(Instant::now()),
            boolean: StageResult::pass(Instant::now()),
            tessellate: StageResult::pass(Instant::now()),
            round_trip: StageResult::pass(Instant::now()),
        };
        // Include a preceding warning and propagated errors, as real validation
        // failures do. Neither may replace or multiply the primary failure.
        stages.read.diagnostics.push(DiagnosticRecord::warning(
            "earlier_warning",
            "not a primary failure",
        ));
        let failed = StageResult::failed_with(diagnostic);
        if stage == "validate" {
            stages.validate = failed;
            stages.boolean = StageResult::failed_with(DiagnosticRecord::error(
                FailureCategory::InvalidTopology,
                "prerequisite_failed",
                "upstream failure",
            ));
        } else {
            stages.read = failed;
        }
        ModelResult {
            schema_version: crate::SCHEMA_VERSION,
            model: "/private/model.step".into(),
            passed: false,
            total_duration_ms: 0,
            stages,
        }
    }

    #[test]
    fn aggregate_reconciles_primary_stages_and_unknown_imports_without_prose() {
        let results = [
            result_with_failure(
                "validate",
                DiagnosticRecord::error(
                    FailureCategory::InvalidTopology,
                    "shell_free_boundary",
                    "/private/topology or model contents",
                ),
            ),
            result_with_failure(
                "read",
                DiagnosticRecord::error(
                    FailureCategory::InvalidInput,
                    "step_parse_error",
                    "parse error: unrecognized new refusal /private/input.step",
                ),
            ),
            result_with_failure(
                "read",
                DiagnosticRecord::error(
                    FailureCategory::InvalidInput,
                    "/private/arbitrary-code|\n",
                    "parse error: PCURVE #1 has private contents",
                ),
            ),
        ];
        let scoreboard = crate::aggregate(&results);
        let details = scoreboard.failure_details.as_ref().unwrap();
        details.validate(3, 2).unwrap();
        assert_eq!(
            details.first_failure_codes["validate"]["shell_free_boundary"],
            1
        );
        assert_eq!(details.first_failure_codes["read"]["unclassified_error"], 1);
        assert_eq!(details.import_families[&ImportFailureFamily::Unknown], 2);
        let json = serde_json::to_string(&scoreboard).unwrap();
        let markdown = crate::scoreboard_markdown(&scoreboard);
        for text in [&json, &markdown] {
            assert!(!text.contains("/private"));
            assert!(!text.contains("prerequisite_failed"));
            assert!(!text.contains("earlier_warning"));
            assert!(!text.contains("unrecognized new refusal"));
        }
    }

    #[test]
    fn oversized_parse_prose_stays_unknown() {
        let diagnostic = DiagnosticRecord::error(
            FailureCategory::InvalidInput,
            "step_parse_error",
            format!("parse error: PCURVE #1 {}", "x".repeat(8_192)),
        );
        assert_eq!(import_family(&diagnostic), ImportFailureFamily::Unknown);
    }

    #[test]
    fn count_mismatches_and_overflow_are_refused() {
        let mut details = FailureDetails::default();
        let diagnostic = DiagnosticRecord::error(
            FailureCategory::ResourceLimit,
            "import_limit_exceeded",
            "private resource context",
        );
        details.record("read", &diagnostic);
        details.validate(1, 1).unwrap();
        assert!(details.validate(2, 1).is_err());
        assert!(details.validate(1, 2).is_err());
        details.first_failure_codes.insert(
            "validate".into(),
            BTreeMap::from([("other_failure".into(), usize::MAX)]),
        );
        assert!(details.validate(1, 1).is_err());
    }
}
