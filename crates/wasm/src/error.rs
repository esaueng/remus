//! WASM-boundary error types.
//!
//! [`WasmError`] aggregates errors from all lower layers. Because
//! `wasm-bindgen` provides a blanket `impl<E: Error> From<E> for JsError`,
//! any `WasmError` can be converted to `JsError` automatically via `?`.

use remus_math::diagnostic::{FailureCategory, ToDiagnostic};
use serde_json::{Map, Value};

/// Maximum JS-controlled work count accepted by scalar WASM parameters.
pub const MAX_WASM_WORK_ITEMS: u32 = 10_000;

/// Errors that can occur in WASM-exposed operations.
#[derive(Debug, thiserror::Error)]
pub enum WasmError {
    /// A JS-provided handle index does not correspond to a valid entity.
    #[error("invalid {entity} handle: index {index} is out of bounds")]
    InvalidHandle {
        /// The kind of entity (e.g. "face", "solid").
        entity: &'static str,
        /// The raw index that was provided.
        index: usize,
    },

    /// An input value is invalid (NaN, infinite, out of range, etc.).
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// Description of what is wrong.
        reason: String,
    },

    /// An error from a modeling operation.
    #[error(transparent)]
    Operations(#[from] remus_operations::OperationsError),

    /// An error from topology lookup.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),

    /// A math error (e.g. singular matrix).
    #[error(transparent)]
    Math(#[from] remus_math::MathError),

    /// An error from a geometric check (outlining, containment, distance).
    #[error(transparent)]
    Check(#[from] remus_check::CheckError),
}

/// Stable machine-readable error codes used by structured WASM contracts.
///
/// These names are a public wire contract. They deliberately do not mirror
/// Rust error variant names, which may evolve independently.
#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum WasmErrorCode {
    InvalidJson,
    BatchLimitExceeded,
    MissingOperation,
    UnknownOperation,
    InvalidArgument,
    InvalidHandle,
    TopologyError,
    OperationFailed,
    #[cfg_attr(not(feature = "io"), allow(dead_code))]
    ResourceLimitExceeded,
    InternalError,
}

impl WasmErrorCode {
    /// The kernel-wide failure category this wire code projects
    /// (`remus_math::diagnostic::FailureCategory`). Explicit per code:
    /// never derived from names.
    pub(crate) const fn category(self) -> FailureCategory {
        match self {
            Self::InvalidJson
            | Self::MissingOperation
            | Self::UnknownOperation
            | Self::InvalidArgument
            | Self::InvalidHandle => FailureCategory::InvalidInput,
            Self::BatchLimitExceeded | Self::ResourceLimitExceeded => {
                FailureCategory::ResourceLimit
            }
            Self::TopologyError => FailureCategory::InvalidTopology,
            Self::OperationFailed | Self::InternalError => FailureCategory::Internal,
        }
    }
}

/// Structured error carried internally until the selected WASM contract is
/// serialized.
#[derive(Debug, serde::Serialize)]
pub(crate) struct StructuredWasmError {
    code: WasmErrorCode,
    category: &'static str,
    message: String,
    details: Map<String, Value>,
}

impl StructuredWasmError {
    pub(crate) fn new(code: WasmErrorCode, message: impl Into<String>) -> Self {
        Self {
            code,
            category: code.category().as_str(),
            message: message.into(),
            details: Map::new(),
        }
    }

    /// Attach the native kernel registry entry (`kernelCode`) from a typed
    /// error's diagnostic, giving v2 consumers the fine-grained code
    /// alongside the coarse wire code.
    fn with_kernel_diagnostic(mut self, source: &impl ToDiagnostic) -> Self {
        self.details.insert(
            "kernelCode".to_string(),
            Value::from(source.diagnostic().code()),
        );
        self
    }

    pub(crate) fn invalid_argument(message: impl Into<String>, argument: Option<&str>) -> Self {
        let mut error = Self::new(WasmErrorCode::InvalidArgument, message);
        if let Some(argument) = argument {
            error
                .details
                .insert("argument".to_string(), Value::from(argument));
        }
        error
    }

    pub(crate) fn invalid_json(error: &serde_json::Error) -> Self {
        let mut structured =
            Self::new(WasmErrorCode::InvalidJson, format!("invalid JSON: {error}"));
        structured
            .details
            .insert("line".to_string(), Value::from(error.line()));
        structured
            .details
            .insert("column".to_string(), Value::from(error.column()));
        structured
    }

    pub(crate) fn batch_limit(
        message: impl Into<String>,
        resource: &'static str,
        limit: usize,
        actual: usize,
    ) -> Self {
        let mut error = Self::new(WasmErrorCode::BatchLimitExceeded, message);
        error
            .details
            .insert("resource".to_string(), Value::from(resource));
        error
            .details
            .insert("limit".to_string(), Value::from(limit));
        error
            .details
            .insert("actual".to_string(), Value::from(actual));
        error
    }

    pub(crate) fn missing_operation(operation_index: usize) -> Self {
        let mut error = Self::new(
            WasmErrorCode::MissingOperation,
            "missing or invalid 'op' field",
        );
        error
            .details
            .insert("operationIndex".to_string(), Value::from(operation_index));
        error
    }

    pub(crate) fn unknown_operation(operation: &str) -> Self {
        let mut error = Self::new(
            WasmErrorCode::UnknownOperation,
            format!("unknown operation: {operation}"),
        );
        error
            .details
            .insert("operation".to_string(), Value::from(operation));
        error
    }

    pub(crate) fn operation_failed(message: impl Into<String>) -> Self {
        Self::new(WasmErrorCode::OperationFailed, message)
    }

    #[cfg_attr(not(feature = "io"), allow(dead_code))]
    pub(crate) fn resource_limit(
        message: impl Into<String>,
        resource: &'static str,
        limit: usize,
        actual: usize,
    ) -> Self {
        let mut error = Self::new(WasmErrorCode::ResourceLimitExceeded, message);
        error
            .details
            .insert("resource".to_string(), Value::from(resource));
        error
            .details
            .insert("limit".to_string(), Value::from(limit));
        error
            .details
            .insert("actual".to_string(), Value::from(actual));
        error
    }

    pub(crate) fn internal(message: impl Into<String>) -> Self {
        Self::new(WasmErrorCode::InternalError, message)
    }

    pub(crate) fn with_operation_context(
        mut self,
        operation_index: usize,
        operation: &str,
    ) -> Self {
        self.details
            .insert("operationIndex".to_string(), Value::from(operation_index));
        self.details
            .insert("operation".to_string(), Value::from(operation));
        self
    }

    pub(crate) fn message(&self) -> &str {
        &self.message
    }
}

impl From<String> for StructuredWasmError {
    fn from(message: String) -> Self {
        Self::invalid_argument(message, None)
    }
}

impl From<&str> for StructuredWasmError {
    fn from(message: &str) -> Self {
        Self::invalid_argument(message, None)
    }
}

impl From<serde_json::Error> for StructuredWasmError {
    fn from(error: serde_json::Error) -> Self {
        Self::internal(error.to_string())
    }
}

impl From<WasmError> for StructuredWasmError {
    fn from(error: WasmError) -> Self {
        let message = error.to_string();
        match error {
            WasmError::InvalidHandle { entity, index } => {
                let mut structured = Self::new(WasmErrorCode::InvalidHandle, message);
                structured
                    .details
                    .insert("entity".to_string(), Value::from(entity));
                structured
                    .details
                    .insert("index".to_string(), Value::from(index));
                structured
            }
            WasmError::InvalidInput { .. } => Self::invalid_argument(message, None),
            WasmError::Operations(error) => Self::from(error),
            WasmError::Topology(error) => Self::from(error),
            WasmError::Math(error) => Self::from(error),
            WasmError::Check(error) => Self::from(error),
        }
    }
}

impl From<remus_topology::TopologyError> for StructuredWasmError {
    fn from(error: remus_topology::TopologyError) -> Self {
        let message = error.to_string();
        let kernel = &error;
        let (entity, index) = match &error {
            remus_topology::TopologyError::VertexNotFound(id) => ("vertex", Some(id.index())),
            remus_topology::TopologyError::EdgeNotFound(id) => ("edge", Some(id.index())),
            remus_topology::TopologyError::WireNotFound(id) => ("wire", Some(id.index())),
            remus_topology::TopologyError::FaceNotFound(id) => ("face", Some(id.index())),
            remus_topology::TopologyError::ShellNotFound(id) => ("shell", Some(id.index())),
            remus_topology::TopologyError::SolidNotFound(id) => ("solid", Some(id.index())),
            remus_topology::TopologyError::CompoundNotFound(id) => ("compound", Some(id.index())),
            remus_topology::TopologyError::CompSolidNotFound(id) => ("compsolid", Some(id.index())),
            remus_topology::TopologyError::WireNotClosed
            | remus_topology::TopologyError::NotPlanar => ("wire", None),
            remus_topology::TopologyError::InvalidColorChannel { .. } => ("attributes", None),
            remus_topology::TopologyError::JournalDuplicateEvent { .. }
            | remus_topology::TopologyError::RefAmbiguous { .. }
            | remus_topology::TopologyError::RefDangling { .. }
            | remus_topology::TopologyError::RefUnresolvedAcrossOperation { .. }
            | remus_topology::TopologyError::RefUnknownOperation { .. }
            | remus_topology::TopologyError::RefNoMatch { .. }
            | remus_topology::TopologyError::JournalSnapshotInvalid { .. } => ("journal", None),
            remus_topology::TopologyError::LoopNotFound(id) => ("loop", Some(id.index())),
            remus_topology::TopologyError::CoedgeNotFound(id) => ("coedge", Some(id.index())),
            remus_topology::TopologyError::LoopWireMismatch { face }
            | remus_topology::TopologyError::LoopNotConnected { face }
            | remus_topology::TopologyError::SeamPcurveAmbiguous { face, .. }
            | remus_topology::TopologyError::SameParameterExceeded { face, .. }
            | remus_topology::TopologyError::SameRangeExceeded { face, .. } => {
                ("face", Some(face.index()))
            }
            remus_topology::TopologyError::Empty { entity } => (*entity, None),
            remus_topology::TopologyError::NonManifold { .. } => ("topology", None),
        };
        let mut structured = Self::new(WasmErrorCode::TopologyError, message);
        structured
            .details
            .insert("entity".to_string(), Value::from(entity));
        if let Some(index) = index {
            structured
                .details
                .insert("index".to_string(), Value::from(index));
        }
        structured.with_kernel_diagnostic(kernel)
    }
}

impl From<remus_math::MathError> for StructuredWasmError {
    fn from(error: remus_math::MathError) -> Self {
        let code = match &error {
            remus_math::MathError::ConvergenceFailure { .. } => WasmErrorCode::OperationFailed,
            _ => WasmErrorCode::InvalidArgument,
        };
        Self::new(code, error.to_string()).with_kernel_diagnostic(&error)
    }
}

impl From<remus_check::CheckError> for StructuredWasmError {
    fn from(error: remus_check::CheckError) -> Self {
        let message = error.to_string();
        match error {
            remus_check::CheckError::Topology(error) => Self::from(error),
            remus_check::CheckError::Math(error) => Self::from(error),
            _ => Self::operation_failed(message),
        }
    }
}

impl From<remus_operations::OperationsError> for StructuredWasmError {
    fn from(error: remus_operations::OperationsError) -> Self {
        let message = error.to_string();
        match error {
            remus_operations::OperationsError::InvalidInput { .. } => {
                Self::invalid_argument(message, None)
            }
            remus_operations::OperationsError::Topology(error) => Self::from(error),
            remus_operations::OperationsError::Math(error) => Self::from(error),
            remus_operations::OperationsError::Check(error) => {
                let mut structured = Self::from(error);
                structured.message = message;
                structured
            }
            _ => Self::operation_failed(message),
        }
    }
}

impl From<remus_geometry::error::GeomError> for StructuredWasmError {
    fn from(error: remus_geometry::error::GeomError) -> Self {
        Self::operation_failed(error.to_string())
    }
}

impl From<remus_heal::HealError> for StructuredWasmError {
    fn from(error: remus_heal::HealError) -> Self {
        Self::operation_failed(error.to_string())
    }
}

impl From<remus_algo::error::AlgoError> for StructuredWasmError {
    fn from(error: remus_algo::error::AlgoError) -> Self {
        Self::operation_failed(error.to_string()).with_kernel_diagnostic(&error)
    }
}

#[cfg(feature = "io")]
impl From<remus_io::IoError> for StructuredWasmError {
    fn from(error: remus_io::IoError) -> Self {
        let message = error.to_string();
        match error {
            remus_io::IoError::LimitExceeded {
                resource,
                limit,
                actual,
            } => Self::resource_limit(message, resource, limit, actual),
            remus_io::IoError::ParseError { .. } => Self::invalid_argument(message, None),
            remus_io::IoError::InvalidTopology { .. } => {
                Self::new(WasmErrorCode::TopologyError, message)
            }
            remus_io::IoError::Topology(error) => Self::from(error),
            remus_io::IoError::Operations(error) => Self::from(error),
            _ => Self::internal(message),
        }
    }
}

/// Validate that a `f64` value is finite (not NaN or infinite).
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] if `value` is NaN or infinite.
pub fn validate_finite(value: f64, name: &str) -> Result<(), WasmError> {
    value
        .is_finite()
        .then_some(())
        .ok_or_else(|| WasmError::InvalidInput {
            reason: format!("{name} must be finite, got {value}"),
        })
}

/// Validate that a `f64` value is finite and strictly positive.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] if `value` is NaN, infinite, zero,
/// or negative.
pub fn validate_positive(value: f64, name: &str) -> Result<(), WasmError> {
    validate_finite(value, name)?;
    (value > 0.0)
        .then_some(())
        .ok_or_else(|| WasmError::InvalidInput {
            reason: format!("{name} must be positive, got {value}"),
        })
}

/// Validate a JS-controlled scalar before it reaches a loop or allocation.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] when `value` exceeds the public work
/// budget.
pub fn validate_work_count(value: u32, name: &str) -> Result<usize, WasmError> {
    if value > MAX_WASM_WORK_ITEMS {
        return Err(WasmError::InvalidInput {
            reason: format!("{name} must be at most {MAX_WASM_WORK_ITEMS}, got {value}"),
        });
    }
    Ok(value as usize)
}

/// Validate the total work implied by two multiplicative JS counts.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] when the product exceeds the public
/// work budget.
pub fn validate_work_product(left: u32, right: u32, name: &str) -> Result<usize, WasmError> {
    let work = u64::from(left) * u64::from(right);
    if work > u64::from(MAX_WASM_WORK_ITEMS) {
        return Err(WasmError::InvalidInput {
            reason: format!("{name} must be at most {MAX_WASM_WORK_ITEMS}, got {work}"),
        });
    }
    Ok(work as usize)
}

#[cfg(test)]
mod work_limit_tests {
    #![allow(clippy::unwrap_used)]

    use super::{MAX_WASM_WORK_ITEMS, validate_work_count, validate_work_product};

    #[test]
    fn scalar_work_count_accepts_limit_and_rejects_larger_values() {
        assert_eq!(
            validate_work_count(MAX_WASM_WORK_ITEMS, "segments").unwrap(),
            MAX_WASM_WORK_ITEMS as usize
        );
        let error = validate_work_count(MAX_WASM_WORK_ITEMS + 1, "segments").unwrap_err();
        assert_eq!(
            error.to_string(),
            "invalid input: segments must be at most 10000, got 10001"
        );
        assert!(validate_work_count(u32::MAX, "segments").is_err());
    }

    #[test]
    fn multiplicative_work_count_is_limited_by_total() {
        assert_eq!(
            validate_work_product(100, 100, "grid copies").unwrap(),
            10_000
        );
        assert!(validate_work_product(101, 100, "grid copies").is_err());
        assert!(validate_work_product(u32::MAX, u32::MAX, "grid copies").is_err());
    }
}
