//! WASM-boundary error types.
//!
//! [`WasmError`] aggregates errors from all lower layers. Because
//! `wasm-bindgen` provides a blanket `impl<E: Error> From<E> for JsError`,
//! any `WasmError` can be converted to `JsError` automatically via `?`.

use brepkit_math::diagnostic::{FailureCategory, ToDiagnostic};
use serde_json::{Map, Value};

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
    Operations(#[from] brepkit_operations::OperationsError),

    /// An error from topology lookup.
    #[error(transparent)]
    Topology(#[from] brepkit_topology::TopologyError),

    /// A math error (e.g. singular matrix).
    #[error(transparent)]
    Math(#[from] brepkit_math::MathError),

    /// An error from a geometric check (outlining, containment, distance).
    #[error(transparent)]
    Check(#[from] brepkit_check::CheckError),
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
    /// (`brepkit_math::diagnostic::FailureCategory`). Explicit per code:
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

impl From<brepkit_topology::TopologyError> for StructuredWasmError {
    fn from(error: brepkit_topology::TopologyError) -> Self {
        let message = error.to_string();
        let kernel = &error;
        let (entity, index) = match &error {
            brepkit_topology::TopologyError::VertexNotFound(id) => ("vertex", Some(id.index())),
            brepkit_topology::TopologyError::EdgeNotFound(id) => ("edge", Some(id.index())),
            brepkit_topology::TopologyError::WireNotFound(id) => ("wire", Some(id.index())),
            brepkit_topology::TopologyError::FaceNotFound(id) => ("face", Some(id.index())),
            brepkit_topology::TopologyError::ShellNotFound(id) => ("shell", Some(id.index())),
            brepkit_topology::TopologyError::SolidNotFound(id) => ("solid", Some(id.index())),
            brepkit_topology::TopologyError::CompoundNotFound(id) => ("compound", Some(id.index())),
            brepkit_topology::TopologyError::CompSolidNotFound(id) => {
                ("compsolid", Some(id.index()))
            }
            brepkit_topology::TopologyError::WireNotClosed
            | brepkit_topology::TopologyError::NotPlanar => ("wire", None),
            brepkit_topology::TopologyError::LoopNotFound(id) => ("loop", Some(id.index())),
            brepkit_topology::TopologyError::CoedgeNotFound(id) => ("coedge", Some(id.index())),
            brepkit_topology::TopologyError::LoopWireMismatch { face }
            | brepkit_topology::TopologyError::LoopNotConnected { face }
            | brepkit_topology::TopologyError::SeamPcurveAmbiguous { face, .. } => {
                ("face", Some(face.index()))
            }
            brepkit_topology::TopologyError::Empty { entity } => (*entity, None),
            brepkit_topology::TopologyError::NonManifold { .. } => ("topology", None),
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

impl From<brepkit_math::MathError> for StructuredWasmError {
    fn from(error: brepkit_math::MathError) -> Self {
        let code = match &error {
            brepkit_math::MathError::ConvergenceFailure { .. } => WasmErrorCode::OperationFailed,
            _ => WasmErrorCode::InvalidArgument,
        };
        Self::new(code, error.to_string()).with_kernel_diagnostic(&error)
    }
}

impl From<brepkit_check::CheckError> for StructuredWasmError {
    fn from(error: brepkit_check::CheckError) -> Self {
        let message = error.to_string();
        match error {
            brepkit_check::CheckError::Topology(error) => Self::from(error),
            brepkit_check::CheckError::Math(error) => Self::from(error),
            _ => Self::operation_failed(message),
        }
    }
}

impl From<brepkit_operations::OperationsError> for StructuredWasmError {
    fn from(error: brepkit_operations::OperationsError) -> Self {
        let message = error.to_string();
        match error {
            brepkit_operations::OperationsError::InvalidInput { .. } => {
                Self::invalid_argument(message, None)
            }
            brepkit_operations::OperationsError::Topology(error) => Self::from(error),
            brepkit_operations::OperationsError::Math(error) => Self::from(error),
            brepkit_operations::OperationsError::Check(error) => {
                let mut structured = Self::from(error);
                structured.message = message;
                structured
            }
            _ => Self::operation_failed(message),
        }
    }
}

impl From<brepkit_geometry::error::GeomError> for StructuredWasmError {
    fn from(error: brepkit_geometry::error::GeomError) -> Self {
        Self::operation_failed(error.to_string())
    }
}

impl From<brepkit_heal::HealError> for StructuredWasmError {
    fn from(error: brepkit_heal::HealError) -> Self {
        Self::operation_failed(error.to_string())
    }
}

impl From<brepkit_algo::error::AlgoError> for StructuredWasmError {
    fn from(error: brepkit_algo::error::AlgoError) -> Self {
        Self::operation_failed(error.to_string()).with_kernel_diagnostic(&error)
    }
}

#[cfg(feature = "io")]
impl From<brepkit_io::IoError> for StructuredWasmError {
    fn from(error: brepkit_io::IoError) -> Self {
        let message = error.to_string();
        match error {
            brepkit_io::IoError::LimitExceeded {
                resource,
                limit,
                actual,
            } => Self::resource_limit(message, resource, limit, actual),
            brepkit_io::IoError::ParseError { .. } => Self::invalid_argument(message, None),
            brepkit_io::IoError::InvalidTopology { .. } => {
                Self::new(WasmErrorCode::TopologyError, message)
            }
            brepkit_io::IoError::Topology(error) => Self::from(error),
            brepkit_io::IoError::Operations(error) => Self::from(error),
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
