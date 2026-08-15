//! Kernel-wide structured diagnostics.
//!
//! Every kernel failure is describable by a stable **category** (the coarse,
//! contract-level classification from
//! `docs/kernel-maturity/failure-taxonomy.md`) plus a stable **code** (the
//! fine-grained registry entry), independent of the Rust error type that
//! carried it. [`ToDiagnostic`] is implemented by each crate's error enum;
//! the WASM boundary projects these onto its wire registry.
//!
//! # Registry rules
//!
//! - Codes are lowercase ASCII snake case and are **never** derived from
//!   `Display` strings, `Debug` output, or Rust enum/type names — every code
//!   is an explicit literal in a `match`, so renaming a Rust variant cannot
//!   silently change the public code.
//! - The registry is additive: new codes may be added; existing meanings are
//!   never broadened or reassigned. Semantics that diverge get a new code.
//! - Moving a failure between categories is a breaking change.
//! - Codes marked *transitional* in their doc comment classify a stringly
//!   lower-layer failure as [`FailureCategory::Internal`] until typed
//!   context exists; replacing one with precise codes is expected, reusing
//!   its name for something else is not.

use crate::MathError;

/// Coarse, contract-level failure classification.
///
/// Mirrors the failure taxonomy in
/// `docs/kernel-maturity/failure-taxonomy.md`; the variant set changes only
/// with that document.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FailureCategory {
    /// The request is malformed independent of geometric difficulty.
    InvalidInput,
    /// Referenced topology exists but is inconsistent or fails validation
    /// preconditions.
    InvalidTopology,
    /// The operation is well-formed but the configuration is declared
    /// unsupported.
    Unsupported,
    /// An iterative algorithm exhausted its budget without certifying an
    /// answer.
    Nonconvergence,
    /// A byte/entity/work/memory budget was exceeded.
    ResourceLimit,
    /// A result was produced but failed its own tolerance contract.
    ToleranceViolation,
    /// The only achievable result would degrade quality beyond the caller's
    /// fallback policy.
    QualityRefused,
    /// The operation observed a cancellation request and stopped at a safe
    /// point.
    Cancelled,
    /// A failure that cannot be safely classified. Reaching this category
    /// is itself a defect to burn down.
    Internal,
}

impl FailureCategory {
    /// Stable lowercase name used on every wire and in every report.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidInput => "invalid_input",
            Self::InvalidTopology => "invalid_topology",
            Self::Unsupported => "unsupported",
            Self::Nonconvergence => "nonconvergence",
            Self::ResourceLimit => "resource_limit",
            Self::ToleranceViolation => "tolerance_violation",
            Self::QualityRefused => "quality_refused",
            Self::Cancelled => "cancelled",
            Self::Internal => "internal",
        }
    }
}

impl std::fmt::Display for FailureCategory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A machine-actionable detail value attached to a [`Diagnostic`].
#[derive(Debug, Clone, PartialEq)]
pub enum DetailValue {
    /// An integer detail (indices, counts, limits). Values above `i64::MAX`
    /// saturate.
    Int(i64),
    /// A floating-point detail (measured deviations, parameters).
    Float(f64),
    /// A text detail (entity kinds, type tags).
    Text(String),
}

impl From<usize> for DetailValue {
    fn from(value: usize) -> Self {
        Self::Int(i64::try_from(value).unwrap_or(i64::MAX))
    }
}

impl From<f64> for DetailValue {
    fn from(value: f64) -> Self {
        Self::Float(value)
    }
}

impl From<&str> for DetailValue {
    fn from(value: &str) -> Self {
        Self::Text(value.to_owned())
    }
}

/// A structured, stable description of one kernel failure or warning.
#[derive(Debug, Clone, PartialEq)]
pub struct Diagnostic {
    category: FailureCategory,
    code: &'static str,
    message: String,
    details: Vec<(&'static str, DetailValue)>,
}

impl Diagnostic {
    /// Creates a diagnostic with no details.
    #[must_use]
    pub fn new(category: FailureCategory, code: &'static str, message: impl Into<String>) -> Self {
        Self {
            category,
            code,
            message: message.into(),
            details: Vec::new(),
        }
    }

    /// Appends one machine-actionable detail.
    #[must_use]
    pub fn with_detail(mut self, key: &'static str, value: impl Into<DetailValue>) -> Self {
        self.details.push((key, value.into()));
        self
    }

    /// The coarse failure category.
    #[must_use]
    pub const fn category(&self) -> FailureCategory {
        self.category
    }

    /// The stable registry code.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        self.code
    }

    /// The human-readable message (not a stable contract).
    #[must_use]
    pub fn message(&self) -> &str {
        &self.message
    }

    /// The attached details, in insertion order.
    #[must_use]
    pub fn details(&self) -> &[(&'static str, DetailValue)] {
        &self.details
    }
}

/// Conversion from a typed error to its stable [`Diagnostic`].
///
/// Wrapper variants (`#[from]` of a lower-layer error) delegate to the inner
/// error's implementation so one failure has one code regardless of which
/// layer reports it.
pub trait ToDiagnostic {
    /// The stable diagnostic for this error value.
    fn diagnostic(&self) -> Diagnostic;
}

impl ToDiagnostic for MathError {
    fn diagnostic(&self) -> Diagnostic {
        use FailureCategory::{InvalidInput, Nonconvergence};
        let message = self.to_string();
        match self {
            Self::InvalidDegree {
                degree,
                control_points,
            } => Diagnostic::new(InvalidInput, "invalid_nurbs_degree", message)
                .with_detail("degree", *degree)
                .with_detail("controlPoints", *control_points),
            Self::InvalidKnotVector { expected, got } => {
                Diagnostic::new(InvalidInput, "invalid_knot_vector", message)
                    .with_detail("expected", *expected)
                    .with_detail("got", *got)
            }
            Self::InvalidKnotValue { index, value } => {
                Diagnostic::new(InvalidInput, "invalid_knot_value", message)
                    .with_detail("index", *index)
                    .with_detail("value", *value)
            }
            Self::InvalidWeights { expected, got } => {
                Diagnostic::new(InvalidInput, "invalid_weights", message)
                    .with_detail("expected", *expected)
                    .with_detail("got", *got)
            }
            Self::InvalidWeightValue { index, value } => {
                Diagnostic::new(InvalidInput, "invalid_weight_value", message)
                    .with_detail("index", *index)
                    .with_detail("value", *value)
            }
            Self::InvalidControlPointGrid {
                expected_rows,
                expected_cols,
            } => Diagnostic::new(InvalidInput, "invalid_control_point_grid", message)
                .with_detail("expectedRows", *expected_rows)
                .with_detail("expectedCols", *expected_cols),
            Self::ZeroVector => Diagnostic::new(InvalidInput, "zero_vector", message),
            Self::SingularMatrix => Diagnostic::new(InvalidInput, "singular_matrix", message),
            Self::EmptyInput => Diagnostic::new(InvalidInput, "empty_input", message),
            Self::ParameterOutOfRange { value, min, max } => {
                Diagnostic::new(InvalidInput, "parameter_out_of_range", message)
                    .with_detail("value", *value)
                    .with_detail("min", *min)
                    .with_detail("max", *max)
            }
            Self::ConvergenceFailure { iterations } => {
                Diagnostic::new(Nonconvergence, "newton_nonconvergence", message)
                    .with_detail("iterations", *iterations)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    fn is_snake_case(code: &str) -> bool {
        !code.is_empty()
            && code
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
    }

    #[test]
    fn category_names_are_snake_case_and_distinct() {
        let all = [
            FailureCategory::InvalidInput,
            FailureCategory::InvalidTopology,
            FailureCategory::Unsupported,
            FailureCategory::Nonconvergence,
            FailureCategory::ResourceLimit,
            FailureCategory::ToleranceViolation,
            FailureCategory::QualityRefused,
            FailureCategory::Cancelled,
            FailureCategory::Internal,
        ];
        let names: std::collections::HashSet<_> = all.iter().map(|c| c.as_str()).collect();
        assert_eq!(names.len(), all.len());
        assert!(names.iter().all(|n| is_snake_case(n)));
    }

    #[test]
    fn math_error_registry_is_pinned() {
        // Stable-code registry pins: changing any of these strings is a
        // public contract change, not a refactor.
        let cases: Vec<(MathError, FailureCategory, &str)> = vec![
            (
                MathError::InvalidDegree {
                    degree: 0,
                    control_points: 4,
                },
                FailureCategory::InvalidInput,
                "invalid_nurbs_degree",
            ),
            (
                MathError::ZeroVector,
                FailureCategory::InvalidInput,
                "zero_vector",
            ),
            (
                MathError::SingularMatrix,
                FailureCategory::InvalidInput,
                "singular_matrix",
            ),
            (
                MathError::EmptyInput,
                FailureCategory::InvalidInput,
                "empty_input",
            ),
            (
                MathError::ConvergenceFailure { iterations: 20 },
                FailureCategory::Nonconvergence,
                "newton_nonconvergence",
            ),
        ];
        for (error, category, code) in cases {
            let d = error.diagnostic();
            assert_eq!(d.category(), category, "{error}");
            assert_eq!(d.code(), code, "{error}");
            assert!(is_snake_case(d.code()));
            assert!(!d.message().is_empty());
        }
    }

    #[test]
    fn details_carry_typed_values() {
        let d = MathError::ConvergenceFailure { iterations: 20 }.diagnostic();
        assert_eq!(d.details(), &[("iterations", DetailValue::Int(20))]);
    }

    #[test]
    fn usize_detail_saturates_instead_of_wrapping() {
        let d = Diagnostic::new(FailureCategory::Internal, "internal_error", "m")
            .with_detail("huge", usize::MAX);
        assert_eq!(d.details()[0].1, DetailValue::Int(i64::MAX));
    }
}
