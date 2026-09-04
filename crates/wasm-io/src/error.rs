//! Errors crossing the translator module's JavaScript boundary.
//!
//! `wasm-bindgen` provides a blanket `impl<E: Error> From<E> for JsError`,
//! so every [`IoWasmError`] converts with `?` in the exported methods.

/// Errors from a translator call.
#[derive(Debug, thiserror::Error)]
pub enum IoWasmError {
    /// A JavaScript-supplied argument is unusable.
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// What is wrong with the argument.
        reason: String,
    },

    /// A read, write, or arena codec failure.
    #[error(transparent)]
    Io(#[from] remus_io::IoError),

    /// A report could not be encoded as JSON.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

impl IoWasmError {
    pub(crate) fn invalid(reason: impl Into<String>) -> Self {
        Self::InvalidInput {
            reason: reason.into(),
        }
    }
}

/// Reject non-finite or non-positive scalars before they reach a writer.
pub(crate) fn validate_positive(value: f64, name: &str) -> Result<(), IoWasmError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(IoWasmError::invalid(format!(
            "{name} must be a finite positive number, got {value}"
        )));
    }
    Ok(())
}
