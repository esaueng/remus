//! Reproducible pass-rate trend rows and regression ratchets.

use std::collections::BTreeMap;
use std::fs;
use std::io::ErrorKind;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{GauntletError, SCHEMA_VERSION, Scoreboard};

/// Stable trend-row schema.
pub const TREND_SCHEMA: &str = "remus-gauntlet-trend-v1";

const STAGE_NAMES: [&str; 5] = ["read", "validate", "boolean", "tessellate", "round_trip"];

/// Pass-rate data retained for one pipeline stage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrendStage {
    /// Models that passed the stage.
    pub passed: usize,
    /// Models that reached the scoreboard, including propagated failures.
    pub total: usize,
    /// Human-readable percentage derived exactly from the integer counts.
    pub pass_rate_percent: f64,
}

/// One append-only trend row for one corpus run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrendRow {
    /// Trend schema identifier.
    pub schema: String,
    /// Manifest tier (`smoke` or `abc-1k` in scheduled CI).
    pub tier: String,
    /// UTC calendar date (`YYYY-MM-DD`).
    pub date: String,
    /// Exact kernel commit exercised by the run.
    pub kernel_sha: String,
    /// SHA-256 of the manifest bytes used by the run.
    pub manifest_sha256: String,
    /// Total models represented by the scoreboard.
    pub models: usize,
    /// Per-stage pass rates and their source counts.
    pub stages: BTreeMap<String, TrendStage>,
    /// Primary failures grouped by stable taxonomy category.
    pub failure_categories: BTreeMap<String, usize>,
    /// Probe booleans completed exactly.
    pub boolean_exact: usize,
    /// Probe booleans completed through disclosed approximation.
    pub boolean_approximate: usize,
}

/// Construct and validate a trend row from a scoreboard.
///
/// # Errors
///
/// Returns an error for empty scoreboards, malformed provenance, missing
/// stages, or inconsistent stage totals.
pub fn build_trend_row(
    scoreboard: &Scoreboard,
    tier: &str,
    date: &str,
    kernel_sha: &str,
    manifest_sha256: &str,
) -> Result<TrendRow, GauntletError> {
    validate_tier(tier)?;
    validate_date(date)?;
    validate_hex(kernel_sha, 40, "kernel SHA")?;
    validate_hex(manifest_sha256, 64, "manifest SHA-256")?;
    if scoreboard.schema_version != SCHEMA_VERSION {
        return Err(GauntletError::message(format!(
            "gauntlet_trend_scoreboard_schema: expected {SCHEMA_VERSION}, got {}",
            scoreboard.schema_version
        )));
    }
    if scoreboard.models == 0 {
        return Err(GauntletError::message(
            "gauntlet_trend_empty_scoreboard: refusing to record a zero-model run",
        ));
    }
    let overall_total = scoreboard.passed.saturating_add(scoreboard.failed);
    if overall_total != scoreboard.models {
        return Err(GauntletError::message(format!(
            "gauntlet_trend_inconsistent_scoreboard: overall totals {overall_total}, expected {}",
            scoreboard.models
        )));
    }

    let mut stages = BTreeMap::new();
    for name in STAGE_NAMES {
        let summary = scoreboard.stages.get(name).ok_or_else(|| {
            GauntletError::message(format!(
                "gauntlet_trend_missing_stage: scoreboard has no {name} stage"
            ))
        })?;
        let total = summary.passed.saturating_add(summary.failed);
        if total != scoreboard.models {
            return Err(GauntletError::message(format!(
                "gauntlet_trend_inconsistent_stage: {name} totals {total}, expected {}",
                scoreboard.models
            )));
        }
        stages.insert(
            name.to_owned(),
            TrendStage {
                passed: summary.passed,
                total,
                pass_rate_percent: pass_rate_percent(summary.passed, total),
            },
        );
    }

    Ok(TrendRow {
        schema: TREND_SCHEMA.to_owned(),
        tier: tier.to_owned(),
        date: date.to_owned(),
        kernel_sha: kernel_sha.to_owned(),
        manifest_sha256: manifest_sha256.to_owned(),
        models: scoreboard.models,
        stages,
        failure_categories: scoreboard.failure_categories.clone(),
        boolean_exact: scoreboard.boolean_exact,
        boolean_approximate: scoreboard.boolean_approximate,
    })
}

/// Read the newest trend row for `tier` from an append-only JSONL history.
///
/// A missing history file is the supported first-run state. Malformed or
/// unknown-schema rows fail closed instead of silently resetting the ratchet.
///
/// # Errors
///
/// Returns an error when an existing history cannot be read or parsed.
pub fn load_latest_trend(history: &Path, tier: &str) -> Result<Option<TrendRow>, GauntletError> {
    validate_tier(tier)?;
    let contents = match fs::read_to_string(history) {
        Ok(contents) => contents,
        Err(error) if error.kind() == ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(GauntletError::message(error.to_string())),
    };
    let mut latest = None;
    for (index, line) in contents.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let row: TrendRow = serde_json::from_str(line).map_err(|error| {
            GauntletError::message(format!(
                "gauntlet_trend_history_invalid: line {}: {error}",
                index + 1
            ))
        })?;
        validate_history_row(&row, index + 1)?;
        if row.tier == tier {
            latest = Some(row);
        }
    }
    Ok(latest)
}

/// Refuse a per-stage pass-rate regression beyond `max_drop_basis_points`.
///
/// One basis point is 0.01 percentage points. Equality with the declared
/// allowance passes; only a strictly larger drop fails.
///
/// # Errors
///
/// Returns a stable `gauntlet_pass_rate_regression` error naming every stage
/// whose drop exceeds the allowance.
pub fn enforce_ratchet(
    previous: &TrendRow,
    current: &TrendRow,
    max_drop_basis_points: u32,
) -> Result<(), GauntletError> {
    if previous.tier != current.tier {
        return Err(GauntletError::message(format!(
            "gauntlet_trend_tier_mismatch: previous tier {} does not match current tier {}",
            previous.tier, current.tier
        )));
    }
    let mut regressions = Vec::new();
    for name in STAGE_NAMES {
        let old = previous.stages.get(name).ok_or_else(|| {
            GauntletError::message(format!(
                "gauntlet_trend_missing_stage: previous row has no {name} stage"
            ))
        })?;
        let new = current.stages.get(name).ok_or_else(|| {
            GauntletError::message(format!(
                "gauntlet_trend_missing_stage: current row has no {name} stage"
            ))
        })?;
        let old_rate = pass_rate_basis_points(old.passed, old.total)?;
        let new_rate = pass_rate_basis_points(new.passed, new.total)?;
        let drop = old_rate.saturating_sub(new_rate);
        if drop > max_drop_basis_points {
            regressions.push(format!(
                "{name} {:.2}% -> {:.2}% (drop {:.2}pp)",
                f64::from(old_rate) / 100.0,
                f64::from(new_rate) / 100.0,
                f64::from(drop) / 100.0
            ));
        }
    }
    if regressions.is_empty() {
        Ok(())
    } else {
        Err(GauntletError::message(format!(
            "gauntlet_pass_rate_regression: allowed drop {:.2}pp; {}",
            f64::from(max_drop_basis_points) / 100.0,
            regressions.join(", ")
        )))
    }
}

/// Write one standalone JSON trend row. The caller appends it to the results
/// branch only after the corpus run has produced all aggregate artifacts.
///
/// # Errors
///
/// Returns an error when the parent directory or row cannot be written.
pub fn write_trend_row(path: &Path, row: &TrendRow) -> Result<(), GauntletError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|error| GauntletError::message(error.to_string()))?;
    }
    let mut json =
        serde_json::to_vec(row).map_err(|error| GauntletError::message(error.to_string()))?;
    json.push(b'\n');
    fs::write(path, json).map_err(|error| GauntletError::message(error.to_string()))
}

fn validate_history_row(row: &TrendRow, line: usize) -> Result<(), GauntletError> {
    if row.schema != TREND_SCHEMA {
        return Err(GauntletError::message(format!(
            "gauntlet_trend_history_invalid: line {line}: unsupported schema {}",
            row.schema
        )));
    }
    validate_tier(&row.tier)?;
    validate_date(&row.date)?;
    validate_hex(&row.kernel_sha, 40, "kernel SHA")?;
    validate_hex(&row.manifest_sha256, 64, "manifest SHA-256")?;
    for name in STAGE_NAMES {
        let stage = row.stages.get(name).ok_or_else(|| {
            GauntletError::message(format!(
                "gauntlet_trend_history_invalid: line {line}: missing {name} stage"
            ))
        })?;
        if stage.total != row.models || stage.passed > stage.total || stage.total == 0 {
            return Err(GauntletError::message(format!(
                "gauntlet_trend_history_invalid: line {line}: inconsistent {name} counts"
            )));
        }
    }
    Ok(())
}

fn validate_tier(tier: &str) -> Result<(), GauntletError> {
    if tier.is_empty()
        || !tier
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'-')
    {
        return Err(GauntletError::message(
            "gauntlet_trend_invalid_tier: expected lowercase letters, digits, or hyphens",
        ));
    }
    Ok(())
}

fn validate_date(date: &str) -> Result<(), GauntletError> {
    let bytes = date.as_bytes();
    if bytes.len() != 10
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes
            .iter()
            .enumerate()
            .any(|(index, byte)| index != 4 && index != 7 && !byte.is_ascii_digit())
    {
        return Err(GauntletError::message(
            "gauntlet_trend_invalid_date: expected YYYY-MM-DD",
        ));
    }
    Ok(())
}

fn validate_hex(value: &str, length: usize, name: &str) -> Result<(), GauntletError> {
    if value.len() != length || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(GauntletError::message(format!(
            "gauntlet_trend_invalid_provenance: {name} must be {length} hexadecimal characters"
        )));
    }
    Ok(())
}

fn pass_rate_percent(passed: usize, total: usize) -> f64 {
    if total == 0 {
        0.0
    } else {
        100.0 * passed as f64 / total as f64
    }
}

fn pass_rate_basis_points(passed: usize, total: usize) -> Result<u32, GauntletError> {
    if total == 0 || passed > total {
        return Err(GauntletError::message(
            "gauntlet_trend_invalid_rate: pass counts must be non-empty and bounded by total",
        ));
    }
    let numerator = (passed as u128) * 10_000;
    u32::try_from(numerator / total as u128)
        .map_err(|_| GauntletError::message("gauntlet_trend_invalid_rate: rate overflow"))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use super::*;
    use crate::{SCHEMA_VERSION, StageSummary};

    fn scoreboard(passed: usize, models: usize) -> Scoreboard {
        let mut stages = BTreeMap::new();
        for name in STAGE_NAMES {
            stages.insert(
                name.to_owned(),
                StageSummary {
                    passed,
                    failed: models - passed,
                },
            );
        }
        Scoreboard {
            schema_version: SCHEMA_VERSION,
            models,
            passed,
            failed: models - passed,
            stages,
            failure_categories: BTreeMap::new(),
            boolean_exact: passed,
            boolean_approximate: 0,
        }
    }

    fn row(passed: usize, models: usize) -> TrendRow {
        build_trend_row(
            &scoreboard(passed, models),
            "smoke",
            "2026-08-31",
            "0123456789abcdef0123456789abcdef01234567",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        )
        .unwrap()
    }

    #[test]
    fn ratchet_accepts_equal_and_declared_drop() {
        let previous = row(1000, 1000);
        let current = row(995, 1000);
        enforce_ratchet(&previous, &current, 50).unwrap();
        enforce_ratchet(&current, &current, 0).unwrap();
    }

    #[test]
    fn ratchet_refuses_drop_beyond_declared_threshold() {
        let error = enforce_ratchet(&row(50, 50), &row(49, 50), 50).unwrap_err();
        assert!(
            error.to_string().contains("gauntlet_pass_rate_regression"),
            "{error}"
        );
        assert!(error.to_string().contains("drop 2.00pp"), "{error}");
    }

    #[test]
    fn missing_history_is_an_explicit_first_run() {
        let path = std::env::temp_dir().join(format!(
            "remus-gauntlet-missing-history-{}",
            std::process::id()
        ));
        assert_eq!(load_latest_trend(&path, "smoke").unwrap(), None);
    }

    #[test]
    fn malformed_history_fails_closed() {
        let path =
            std::env::temp_dir().join(format!("remus-gauntlet-bad-history-{}", std::process::id()));
        fs::write(&path, "not-json\n").unwrap();
        let error = load_latest_trend(&path, "smoke").unwrap_err();
        fs::remove_file(path).unwrap();
        assert!(
            error.to_string().contains("gauntlet_trend_history_invalid"),
            "{error}"
        );
    }

    #[test]
    fn trend_row_records_reproducible_counts_and_provenance() {
        let row = row(49, 50);
        assert!((row.stages["read"].pass_rate_percent - 98.0).abs() < f64::EPSILON);
        assert_eq!(row.models, 50);
        assert_eq!(row.schema, TREND_SCHEMA);
        assert_eq!(row.kernel_sha.len(), 40);
        assert_eq!(row.manifest_sha256.len(), 64);
    }

    #[test]
    fn inconsistent_or_unknown_scoreboard_is_refused() {
        let mut unknown = scoreboard(49, 50);
        unknown.schema_version += 1;
        let error = build_trend_row(
            &unknown,
            "smoke",
            "2026-08-31",
            "0123456789abcdef0123456789abcdef01234567",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        )
        .unwrap_err();
        assert!(error.to_string().contains("scoreboard_schema"), "{error}");

        let mut inconsistent = scoreboard(49, 50);
        inconsistent.failed = 0;
        let error = build_trend_row(
            &inconsistent,
            "smoke",
            "2026-08-31",
            "0123456789abcdef0123456789abcdef01234567",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        )
        .unwrap_err();
        assert!(
            error.to_string().contains("inconsistent_scoreboard"),
            "{error}"
        );
    }
}
