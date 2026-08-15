//! Deterministic reproduction bundles.
//!
//! A reproduction bundle is a versioned JSON document that captures a modeling
//! failure (or any behavior worth pinning) as a replayable artifact: the
//! operation sequence that produces it plus the invariant results expected
//! from each step. The runner replays the sequence through the batch dispatch
//! path ([`BrepKernel::execute_batch_v2`]), so a bundle exercises exactly the
//! contract a JavaScript caller sees, and replays identically on native and
//! WASM builds.
//!
//! Bundles are the canonical carrier for new regressions: minimize the
//! failing case to a bundle, commit it under `crates/wasm/tests/repro/`, and
//! the fixture suite replays it forever.
//!
//! # Schema (version 1)
//!
//! ```json
//! {
//!   "schema": 1,
//!   "name": "box-cut-volume",
//!   "description": "what this pins and why",
//!   "revision": "git revision that recorded the bundle (informational)",
//!   "operations": [
//!     {"op": "makeBox", "args": {"width": 20, "height": 20, "depth": 10}},
//!     {"op": "volume", "args": {"solid": 0, "deflection": 0.1}}
//!   ],
//!   "expect": [
//!     {"ok": 0},
//!     {"okNear": 4000.0, "tol": 1e-9}
//!   ]
//! }
//! ```
//!
//! - `operations` is the exact array accepted by `executeBatchV2`.
//! - `expect` is optional and may be shorter than `operations`; each entry is
//!   either `null` (no expectation for that step) or exactly one of:
//!   - `{"ok": <json>}` — the step succeeds with exactly this value;
//!   - `{"okNear": <number>, "tol": <number>}` — the step succeeds with a
//!     numeric value within `tol` (default `1e-9`) of `okNear`;
//!   - `{"errorCode": "<code>"}` — the step fails with this stable
//!     `executeBatchV2` error code (expected failures are first-class:
//!     a typed refusal is a pinnable behavior, not a broken bundle).
//! - `context` is reserved for the future operation context and must be
//!   empty in schema 1, so today's bundles stay replayable when it lands.
//! - `revision` records where the bundle was captured; the runner does not
//!   gate on it.
//!
//! Unknown fields are rejected: a bundle either matches a schema this runner
//! understands or fails loudly, never silently half-replays.
//!
//! # Determinism
//!
//! [`ReproBundle::run`] replays the sequence in two fresh kernels and
//! requires byte-identical result JSON before any expectation is checked.
//! Handle values are part of that contract (arena allocation is append-only
//! and deterministic), which is what lets expectations pin them.

use serde::Deserialize;
use thiserror::Error;

use crate::kernel::BrepKernel;

/// The bundle schema version this runner reads.
pub const SCHEMA_VERSION: u32 = 1;

/// Numeric tolerance used by `okNear` expectations that do not set `tol`.
pub const DEFAULT_NEAR_TOL: f64 = 1e-9;

/// Fresh-kernel replays performed to assert deterministic output.
const DETERMINISM_RUNS: usize = 2;

/// Failure modes of bundle parsing, validation, and replay.
#[derive(Debug, Error)]
pub enum ReproError {
    /// The bundle document is not valid JSON or does not match the schema.
    #[error("bundle parse error: {0}")]
    Parse(#[from] serde_json::Error),
    /// The bundle declares a schema version this runner does not read.
    #[error("bundle schema {found} unsupported (runner reads {SCHEMA_VERSION})")]
    UnsupportedSchema {
        /// The version the bundle declared.
        found: u32,
    },
    /// Schema 1 reserves `context`; a non-empty value needs a newer schema.
    #[error("`context` is reserved and must be empty in schema {SCHEMA_VERSION}")]
    ReservedContext,
    /// More expectations than operations.
    #[error("bundle has {expectations} expectations but only {operations} operations")]
    TooManyExpectations {
        /// Number of `expect` entries.
        expectations: usize,
        /// Number of `operations` entries.
        operations: usize,
    },
    /// An expectation sets more than one of `ok`, `okNear`, `errorCode`,
    /// or none of them, or sets `tol` without `okNear`.
    #[error("expectation {index}: {reason}")]
    InvalidExpectation {
        /// Index of the offending `expect` entry.
        index: usize,
        /// What is wrong with it.
        reason: &'static str,
    },
    /// Two fresh-kernel replays produced different result JSON.
    #[error("nondeterministic replay: run {run} differs from run 0")]
    Nondeterministic {
        /// The replay index that diverged.
        run: usize,
    },
    /// The batch runner returned JSON the checker could not read.
    #[error("malformed batch result JSON: {0}")]
    MalformedResult(String),
    /// Replay succeeded but one or more expectations did not hold.
    #[error("bundle `{name}` failed:\n{}", failures.join("\n"))]
    Failed {
        /// The bundle's `name`.
        name: String,
        /// One line per unmet expectation.
        failures: Vec<String>,
    },
}

/// One per-operation expectation. See the module docs for the JSON forms.
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
pub struct Expectation {
    /// Exact expected success value.
    #[serde(default)]
    pub ok: Option<serde_json::Value>,
    /// Expected numeric success value, compared within `tol`.
    #[serde(default)]
    pub ok_near: Option<f64>,
    /// Tolerance for `ok_near` (defaults to [`DEFAULT_NEAR_TOL`]).
    #[serde(default)]
    pub tol: Option<f64>,
    /// Expected stable `executeBatchV2` error code.
    #[serde(default)]
    pub error_code: Option<String>,
}

impl Expectation {
    fn validate(&self, index: usize) -> Result<(), ReproError> {
        let set = usize::from(self.ok.is_some())
            + usize::from(self.ok_near.is_some())
            + usize::from(self.error_code.is_some());
        if set != 1 {
            return Err(ReproError::InvalidExpectation {
                index,
                reason: "exactly one of `ok`, `okNear`, `errorCode` must be set",
            });
        }
        if self.tol.is_some() && self.ok_near.is_none() {
            return Err(ReproError::InvalidExpectation {
                index,
                reason: "`tol` requires `okNear`",
            });
        }
        Ok(())
    }

    /// Checks one batch result against this expectation, appending a
    /// human-readable line to `failures` on mismatch.
    fn check(&self, index: usize, result: &serde_json::Value, failures: &mut Vec<String>) {
        if let Some(expected) = &self.ok {
            match result.get("ok") {
                Some(actual) if actual == expected => {}
                _ => failures.push(format!(
                    "op {index}: expected {{\"ok\": {expected}}}, got {result}"
                )),
            }
            return;
        }
        if let Some(expected) = self.ok_near {
            let tol = self.tol.unwrap_or(DEFAULT_NEAR_TOL);
            match result.get("ok").and_then(serde_json::Value::as_f64) {
                Some(actual) if (actual - expected).abs() <= tol => {}
                _ => failures.push(format!(
                    "op {index}: expected ok within {tol} of {expected}, got {result}"
                )),
            }
            return;
        }
        if let Some(expected) = &self.error_code {
            match result
                .get("error")
                .and_then(|e| e.get("code"))
                .and_then(serde_json::Value::as_str)
            {
                Some(actual) if actual == expected => {}
                _ => failures.push(format!(
                    "op {index}: expected error code `{expected}`, got {result}"
                )),
            }
        }
    }
}

/// A parsed, validated reproduction bundle.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
pub struct ReproBundle {
    /// Declared schema version; must equal [`SCHEMA_VERSION`].
    pub schema: u32,
    /// Short kebab-case identifier, used in failure reports.
    pub name: String,
    /// What the bundle pins and why.
    #[serde(default)]
    pub description: String,
    /// Git revision the bundle was captured at (informational).
    #[serde(default)]
    pub revision: String,
    /// Reserved for the future operation context; must be empty in schema 1.
    #[serde(default)]
    pub context: serde_json::Map<String, serde_json::Value>,
    /// The exact operation array accepted by `executeBatchV2`.
    pub operations: Vec<serde_json::Value>,
    /// Optional per-operation expectations, aligned by index.
    #[serde(default)]
    pub expect: Vec<Option<Expectation>>,
}

impl ReproBundle {
    /// Parses and validates a bundle document.
    ///
    /// # Errors
    ///
    /// Returns [`ReproError`] when the document is not valid JSON, does not
    /// match the schema, declares an unsupported version, or contains an
    /// invalid expectation.
    pub fn from_json(json: &str) -> Result<Self, ReproError> {
        let bundle: Self = serde_json::from_str(json)?;
        if bundle.schema != SCHEMA_VERSION {
            return Err(ReproError::UnsupportedSchema {
                found: bundle.schema,
            });
        }
        if !bundle.context.is_empty() {
            return Err(ReproError::ReservedContext);
        }
        if bundle.expect.len() > bundle.operations.len() {
            return Err(ReproError::TooManyExpectations {
                expectations: bundle.expect.len(),
                operations: bundle.operations.len(),
            });
        }
        for (index, expectation) in bundle.expect.iter().enumerate() {
            if let Some(expectation) = expectation {
                expectation.validate(index)?;
            }
        }
        Ok(bundle)
    }

    /// Replays the bundle and checks every expectation.
    ///
    /// The operation sequence runs in [`DETERMINISM_RUNS`] fresh kernels and
    /// the raw result JSON must be byte-identical across runs before any
    /// expectation is evaluated.
    ///
    /// # Errors
    ///
    /// Returns [`ReproError::Nondeterministic`] when replays diverge and
    /// [`ReproError::Failed`] listing every unmet expectation.
    pub fn run(&self) -> Result<(), ReproError> {
        let ops_json = serde_json::Value::Array(self.operations.clone()).to_string();

        let mut first: Option<String> = None;
        for run in 0..DETERMINISM_RUNS {
            let mut kernel = BrepKernel::new();
            let output = kernel.execute_batch_v2(&ops_json);
            match &first {
                None => first = Some(output),
                Some(reference) if *reference != output => {
                    return Err(ReproError::Nondeterministic { run });
                }
                Some(_) => {}
            }
        }
        let Some(output) = first else {
            return Err(ReproError::MalformedResult(
                "no replay was executed".to_owned(),
            ));
        };

        let results: Vec<serde_json::Value> = serde_json::from_str(&output)
            .map_err(|e| ReproError::MalformedResult(e.to_string()))?;

        let mut failures = Vec::new();
        if results.len() == self.operations.len() {
            for (index, expectation) in self.expect.iter().enumerate() {
                if let (Some(expectation), Some(result)) = (expectation, results.get(index)) {
                    expectation.check(index, result, &mut failures);
                }
            }
        } else {
            // A whole-batch failure (parse error, batch limit) collapses the
            // result array; surface it instead of index-mismatched checks.
            failures.push(format!(
                "expected {} results, got {}: {output}",
                self.operations.len(),
                results.len()
            ));
        }

        if failures.is_empty() {
            Ok(())
        } else {
            Err(ReproError::Failed {
                name: self.name.clone(),
                failures,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn minimal(schema: u32, extra: &str) -> String {
        format!(
            r#"{{"schema": {schema}, "name": "t", "operations": [
                {{"op": "makeBox", "args": {{"width": 1, "height": 1, "depth": 1}}}}
            ]{extra}}}"#
        )
    }

    #[test]
    fn parses_and_replays_minimal_bundle() {
        let bundle = ReproBundle::from_json(&minimal(1, r#", "expect": [{"ok": 0}]"#)).unwrap();
        bundle.run().unwrap();
    }

    #[test]
    fn rejects_unsupported_schema() {
        let err = ReproBundle::from_json(&minimal(2, "")).unwrap_err();
        assert!(matches!(err, ReproError::UnsupportedSchema { found: 2 }));
    }

    #[test]
    fn rejects_unknown_fields() {
        let err = ReproBundle::from_json(&minimal(1, r#", "unexpected": true"#)).unwrap_err();
        assert!(matches!(err, ReproError::Parse(_)));
    }

    #[test]
    fn rejects_reserved_context() {
        let err =
            ReproBundle::from_json(&minimal(1, r#", "context": {"tolerance": 1e-6}"#)).unwrap_err();
        assert!(matches!(err, ReproError::ReservedContext));
    }

    #[test]
    fn rejects_ambiguous_expectation() {
        let err = ReproBundle::from_json(&minimal(1, r#", "expect": [{"ok": 0, "okNear": 1.0}]"#))
            .unwrap_err();
        assert!(matches!(
            err,
            ReproError::InvalidExpectation { index: 0, .. }
        ));
    }

    #[test]
    fn rejects_excess_expectations() {
        let err = ReproBundle::from_json(&minimal(1, r#", "expect": [{"ok": 0}, {"ok": 1}]"#))
            .unwrap_err();
        assert!(matches!(err, ReproError::TooManyExpectations { .. }));
    }

    #[test]
    fn reports_unmet_expectation_with_context() {
        let bundle = ReproBundle::from_json(&minimal(1, r#", "expect": [{"ok": 7}]"#)).unwrap();
        let err = bundle.run().unwrap_err();
        let ReproError::Failed { name, failures } = err else {
            unreachable!("expected Failed, got {err:?}")
        };
        assert_eq!(name, "t");
        assert_eq!(failures.len(), 1);
        assert!(failures[0].contains("op 0"));
    }

    #[test]
    fn expected_failure_is_first_class() {
        let bundle = ReproBundle::from_json(
            r#"{"schema": 1, "name": "bad-handle", "operations": [
                {"op": "volume", "args": {"solid": 42, "deflection": 0.1}}
            ], "expect": [{"errorCode": "invalid_handle"}]}"#,
        )
        .unwrap();
        bundle.run().unwrap();
    }

    #[test]
    fn near_expectation_applies_tolerance() {
        let bundle = ReproBundle::from_json(
            r#"{"schema": 1, "name": "vol", "operations": [
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "volume", "args": {"solid": 0, "deflection": 0.1}}
            ], "expect": [{"ok": 0}, {"okNear": 8.0, "tol": 1e-9}]}"#,
        )
        .unwrap();
        bundle.run().unwrap();
    }
}
