//! Explicit contracts for opt-in, certified NURBS reduction and approximation.
//!
//! These contracts are independently authored Remus code. A positive reported
//! deviation bound includes numerical roundoff and does not assert exactness.

use crate::MathError;
use crate::context::OperationContext;
use crate::diagnostic::{Diagnostic, FailureCategory, ToDiagnostic};

/// Whole-interval bounds between two rational curves in the same parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveDifferenceBounds {
    /// Upper bound on position difference in model units.
    pub position: f64,
    /// Upper bound on first-derivative difference per parameter unit.
    pub derivative: f64,
    /// Charged coefficient and control-net work.
    pub work_used: usize,
}

/// Certify two curves on a common parameter interval without modifying them.
///
/// Outward-rounded Bernstein cross products cover every common knot span,
/// including curves with different degrees, weights or knot refinements.
///
/// # Errors
/// Returns typed refusals for unsupported domains, unavailable finite bounds,
/// cancellation and exhausted work. Sampling is never an acceptance test.
pub fn bound_curve_difference(
    original: &super::NurbsCurve,
    candidate: &super::NurbsCurve,
    range: (f64, f64),
    max_work: usize,
    context: &OperationContext,
) -> Result<CurveDifferenceBounds, ReuseError> {
    let mut budget = ReuseBudget::new(max_work, context)?;
    let bound =
        super::reuse_bounds::curve_difference_on_interval(original, candidate, range, &mut budget)?;
    Ok(CurveDifferenceBounds {
        position: bound.position,
        derivative: bound.derivative,
        work_used: budget.used(),
    })
}

/// Certify only the position difference on a shared native interval.
///
/// Uses the same whole-span rational Bernstein certificate as
/// [`bound_curve_difference`], without charging for an unused derivative bound.
/// Neither input carrier is modified.
///
/// # Errors
/// Returns typed refusals for unsupported domains, unavailable finite bounds,
/// cancellation and exhausted work.
pub fn bound_curve_position(
    original: &super::NurbsCurve,
    candidate: &super::NurbsCurve,
    range: (f64, f64),
    max_work: usize,
    context: &OperationContext,
) -> Result<f64, ReuseError> {
    let mut budget = ReuseBudget::new(max_work, context)?;
    super::reuse_bounds::curve_position_on_interval(original, candidate, range, &mut budget)
}

/// Limits for an explicit curve or surface reduction request.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReductionOptions {
    /// Maximum permitted whole-domain position deviation, in model units.
    pub tolerance: f64,
    /// Maximum charged control-net and coefficient work for the whole request.
    pub max_work: usize,
}

impl Default for ReductionOptions {
    fn default() -> Self {
        Self {
            tolerance: 1e-7,
            max_work: 1_000_000,
        }
    }
}

impl ReductionOptions {
    pub(crate) fn validate(self) -> Result<(), ReuseError> {
        if !self.tolerance.is_finite() || self.tolerance <= 0.0 {
            return Err(ReuseError::InvalidOptions {
                reason: "tolerance must be finite and positive",
            });
        }
        if self.max_work == 0 {
            return Err(ReuseError::InvalidOptions {
                reason: "max_work must be positive",
            });
        }
        Ok(())
    }
}

/// A geometry result qualified against the original input geometry.
#[derive(Debug, Clone)]
pub struct ReductionOutcome<T> {
    /// The reduced geometry; the input is never modified.
    pub geometry: T,
    /// Number of removed knots, counting multiplicity and both surface axes.
    pub removed_knots: usize,
    /// Conservative position bound over the complete original domain.
    /// A positive value discloses an approximate result, including roundoff.
    pub deviation_bound: f64,
    /// Charged work used by candidate generation and certification.
    pub work_used: usize,
}

/// Typed refusals from certified NURBS reduction and approximation.
#[derive(Debug, thiserror::Error)]
pub enum ReuseError {
    /// A caller-supplied tolerance, range, or work limit is malformed.
    #[error("invalid NURBS reuse options: {reason}")]
    InvalidOptions {
        /// The invalid option's contract.
        reason: &'static str,
    },
    /// The geometry lies outside the explicitly supported domain.
    #[error("unsupported NURBS reuse configuration: {reason}")]
    Unsupported {
        /// The unsupported configuration.
        reason: &'static str,
    },
    /// The operation exhausted its explicit aggregate work budget.
    #[error("NURBS reuse work limit exceeded ({limit})")]
    WorkLimit {
        /// The caller's maximum charged work.
        limit: usize,
    },
    /// Adaptive fitting would require more output segments than permitted.
    #[error("NURBS reuse segment limit exceeded ({limit})")]
    SegmentLimit {
        /// Maximum number of output segments permitted by the caller.
        limit: usize,
    },
    /// Finite, strictly positive interval bounds could not be established.
    #[error("NURBS reuse bound unavailable: {reason}")]
    BoundUnavailable {
        /// The numerical reason for declining certification.
        reason: &'static str,
    },
    /// A candidate's conservative bound exceeds the permitted tolerance.
    #[error("NURBS reuse bound {bound} exceeds tolerance {tolerance}")]
    ToleranceExceeded {
        /// The computed conservative bound.
        bound: f64,
        /// The caller's tolerance.
        tolerance: f64,
    },
    /// The requested interior knot cannot be removed in the supported domain.
    #[error("NURBS knot is not removable")]
    KnotNotRemovable,
    /// A lower-level mathematical failure, including cancellation.
    #[error(transparent)]
    Math(#[from] MathError),
}

impl ToDiagnostic for ReuseError {
    fn diagnostic(&self) -> Diagnostic {
        let message = self.to_string();
        match self {
            Self::InvalidOptions { reason } => Diagnostic::new(
                FailureCategory::InvalidInput,
                "invalid_nurbs_reuse_options",
                message,
            )
            .with_detail("reason", *reason),
            Self::Unsupported { reason } => Diagnostic::new(
                FailureCategory::Unsupported,
                "unsupported_nurbs_reuse",
                message,
            )
            .with_detail("reason", *reason),
            Self::WorkLimit { limit } => Diagnostic::new(
                FailureCategory::ResourceLimit,
                "nurbs_reuse_work_limit",
                message,
            )
            .with_detail("limit", *limit),
            Self::SegmentLimit { limit } => Diagnostic::new(
                FailureCategory::ResourceLimit,
                "nurbs_reuse_segment_limit",
                message,
            )
            .with_detail("limit", *limit),
            Self::BoundUnavailable { reason } => Diagnostic::new(
                FailureCategory::QualityRefused,
                "nurbs_reuse_bound_unavailable",
                message,
            )
            .with_detail("reason", *reason),
            Self::ToleranceExceeded { bound, tolerance } => Diagnostic::new(
                FailureCategory::ToleranceViolation,
                "nurbs_reuse_tolerance_exceeded",
                message,
            )
            .with_detail("bound", *bound)
            .with_detail("tolerance", *tolerance),
            Self::KnotNotRemovable => Diagnostic::new(
                FailureCategory::QualityRefused,
                "nurbs_knot_not_removable",
                message,
            ),
            Self::Math(error) => error.diagnostic(),
        }
    }
}

/// Aggregate work accounting shared by generation and numerical certification.
pub(crate) struct ReuseBudget<'a> {
    limit: usize,
    used: usize,
    context: &'a OperationContext,
}

impl<'a> ReuseBudget<'a> {
    pub(crate) fn new(max_work: usize, context: &'a OperationContext) -> Result<Self, ReuseError> {
        context.check_cancelled()?;
        if max_work == 0 {
            return Err(ReuseError::InvalidOptions {
                reason: "max_work must be positive",
            });
        }
        Ok(Self {
            limit: max_work,
            used: 0,
            context,
        })
    }

    /// Charge before any corresponding work or allocation, without overflow.
    #[cfg_attr(target_arch = "wasm32", inline(never))]
    pub(crate) fn spend(&mut self, amount: usize) -> Result<(), ReuseError> {
        self.context.check_cancelled()?;
        if amount > self.limit - self.used {
            return Err(ReuseError::WorkLimit { limit: self.limit });
        }
        self.used += amount;
        Ok(())
    }

    pub(crate) const fn used(&self) -> usize {
        self.used
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use crate::context::CancellationToken;

    #[test]
    fn public_difference_certificate_covers_refinement_and_displacement() {
        use crate::nurbs::{NurbsCurve, curve_knot_insert};
        use crate::vec::{Point3, Vec3};
        let original = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
            ],
            vec![1.0, 0.7, 1.0],
        )
        .unwrap();
        let refined = curve_knot_insert(&original, 0.4, 1).unwrap();
        let context = OperationContext::new();
        let bound =
            bound_curve_difference(&original, &refined, (0.1, 0.9), 100_000, &context).unwrap();
        assert!(bound.position < 1e-9 && bound.derivative < 1e-8 && bound.work_used > 0);
        let position = bound_curve_position(
            &original,
            &refined,
            (0.1, 0.9),
            bound.work_used - 1,
            &context,
        )
        .unwrap();
        assert!(position < 1e-9);
        let shifted = NurbsCurve::new(
            refined.degree(),
            refined.knots().to_vec(),
            refined
                .control_points()
                .iter()
                .map(|&p| p + Vec3::new(0.0, 0.0, 0.1))
                .collect(),
            refined.weights().to_vec(),
        )
        .unwrap();
        let bound =
            bound_curve_difference(&original, &shifted, (0.1, 0.9), 100_000, &context).unwrap();
        assert!(bound.position >= 0.1 && bound.position < 0.3, "{bound:?}");
        assert!(
            bound_curve_position(&original, &shifted, (0.1, 0.9), 100_000, &context).unwrap()
                >= 0.1
        );
        assert!(matches!(
            bound_curve_difference(&original, &refined, (0.1, 0.9), 1, &context),
            Err(ReuseError::WorkLimit { .. })
        ));
    }

    #[test]
    fn options_reject_nonpositive_nonfinite_values() {
        for tolerance in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                ReductionOptions {
                    tolerance,
                    max_work: 1
                }
                .validate()
                .is_err()
            );
        }
        assert!(
            ReductionOptions {
                tolerance: 1.0,
                max_work: 0
            }
            .validate()
            .is_err()
        );
        assert!(ReductionOptions::default().validate().is_ok());
    }

    #[test]
    fn budget_checks_before_work_and_does_not_wrap() {
        let context = OperationContext::new();
        let mut budget = ReuseBudget::new(usize::MAX, &context).unwrap();
        budget.spend(usize::MAX - 1).unwrap();
        assert!(matches!(budget.spend(2), Err(ReuseError::WorkLimit { .. })));
        assert_eq!(budget.used(), usize::MAX - 1);
        budget.spend(1).unwrap();
        assert!(matches!(budget.spend(1), Err(ReuseError::WorkLimit { .. })));
    }

    #[test]
    fn cancelled_budget_delegates_the_stable_math_diagnostic() {
        let token = CancellationToken::new();
        let context = OperationContext::new().with_cancellation(token.clone());
        let mut budget = ReuseBudget::new(10, &context).unwrap();
        token.cancel();
        let error = budget.spend(1).unwrap_err();
        assert_eq!(error.diagnostic().code(), "operation_cancelled");
        assert_eq!(error.diagnostic().category(), FailureCategory::Cancelled);
        assert_eq!(budget.used(), 0);
    }

    #[test]
    fn reuse_error_registry_is_explicit_and_pinned() {
        let errors = [
            (
                ReuseError::InvalidOptions { reason: "test" },
                "invalid_nurbs_reuse_options",
                FailureCategory::InvalidInput,
            ),
            (
                ReuseError::Unsupported { reason: "test" },
                "unsupported_nurbs_reuse",
                FailureCategory::Unsupported,
            ),
            (
                ReuseError::WorkLimit { limit: 1 },
                "nurbs_reuse_work_limit",
                FailureCategory::ResourceLimit,
            ),
            (
                ReuseError::SegmentLimit { limit: 1 },
                "nurbs_reuse_segment_limit",
                FailureCategory::ResourceLimit,
            ),
            (
                ReuseError::BoundUnavailable { reason: "test" },
                "nurbs_reuse_bound_unavailable",
                FailureCategory::QualityRefused,
            ),
            (
                ReuseError::ToleranceExceeded {
                    bound: 2.0,
                    tolerance: 1.0,
                },
                "nurbs_reuse_tolerance_exceeded",
                FailureCategory::ToleranceViolation,
            ),
            (
                ReuseError::KnotNotRemovable,
                "nurbs_knot_not_removable",
                FailureCategory::QualityRefused,
            ),
        ];
        for (error, code, category) in errors {
            assert_eq!(error.diagnostic().code(), code);
            assert_eq!(error.diagnostic().category(), category);
        }
    }
}
