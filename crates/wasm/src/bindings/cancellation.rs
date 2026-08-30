//! Cooperative-cancellation bindings for long-running operations.

#![allow(clippy::missing_errors_doc)]

use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext};
use remus_operations::boolean::{BooleanQuality, boolean_with_context};
use wasm_bindgen::prelude::*;

use crate::error::WasmError;
use crate::handles::solid_id_to_u32;
use crate::kernel::BrepKernel;
use crate::types::{BooleanQualityResult, CancellableBooleanResult, CancellableOperationStatus};

/// A one-shot cooperative cancellation signal for a modeling operation.
///
/// Clones share one monotonic flag. Native multithreaded hosts can signal it
/// concurrently; cancelling before a call also refuses that call without
/// touching topology. A single-threaded browser worker cannot process a new
/// JS call while WASM is running, so active browser cancellation still needs
/// the app's worker/shared-memory transport.
#[wasm_bindgen(js_name = "OperationCancellationToken")]
pub struct WasmCancellationToken {
    inner: CancellationToken,
}

#[wasm_bindgen]
impl WasmCancellationToken {
    /// Creates an uncancelled token.
    #[wasm_bindgen(constructor)]
    #[must_use]
    pub fn new() -> Self {
        Self {
            inner: CancellationToken::new(),
        }
    }

    /// Requests cancellation. The request cannot be reset.
    pub fn cancel(&self) {
        self.inner.cancel();
    }

    /// Whether cancellation has been requested.
    #[wasm_bindgen(js_name = "isCancelled")]
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.inner.is_cancelled()
    }
}

impl Default for WasmCancellationToken {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Performs a boolean governed by a cooperative cancellation token.
    ///
    /// Cancellation is typed (`operation_cancelled`) and transactional: no
    /// partial topology is retained. Result quality follows
    /// `booleanWithQuality`, including the optional exact-only policy.
    #[wasm_bindgen(js_name = "booleanWithCancellation")]
    pub fn boolean_with_cancellation(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        token: &WasmCancellationToken,
        exact_only: Option<bool>,
    ) -> Result<CancellableBooleanResult, JsError> {
        match self.boolean_with_cancellation_impl(op, a, b, token, exact_only) {
            Ok(result) => Ok(CancellableBooleanResult {
                status: CancellableOperationStatus::Completed,
                code: None,
                result: Some(result),
            }),
            Err(error) if is_cancellation(&error) => Ok(CancellableBooleanResult {
                status: CancellableOperationStatus::Cancelled,
                code: Some("operation_cancelled".to_string()),
                result: None,
            }),
            Err(error) => Err(JsError::new(&error.to_string())),
        }
    }
}

fn is_cancellation(error: &WasmError) -> bool {
    matches!(
        error,
        WasmError::Math(remus_math::MathError::Cancelled)
            | WasmError::Operations(
                remus_operations::OperationsError::Math(remus_math::MathError::Cancelled)
                    | remus_operations::OperationsError::Algo(remus_algo::error::AlgoError::Math(
                        remus_math::MathError::Cancelled
                    ))
            )
    )
}

impl BrepKernel {
    fn boolean_with_cancellation_impl(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        token: &WasmCancellationToken,
        exact_only: Option<bool>,
    ) -> Result<BooleanQualityResult, WasmError> {
        let boolean_op = match op.to_ascii_lowercase().as_str() {
            "fuse" | "union" => remus_operations::boolean::BooleanOp::Fuse,
            "cut" | "difference" => remus_operations::boolean::BooleanOp::Cut,
            "intersect" | "intersection" => remus_operations::boolean::BooleanOp::Intersect,
            _ => {
                return Err(WasmError::InvalidInput {
                    reason: format!("unknown boolean operation: {op}"),
                });
            }
        };
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let mut context = OperationContext::new().with_cancellation(token.inner.clone());
        if exact_only == Some(true) {
            context = context.with_fallback(FallbackPolicy::ExactOnly);
        }
        let outcome = boolean_with_context(self.topo_mut(), boolean_op, a_id, b_id, &context)?;
        let (quality, deflection) = match outcome.quality {
            BooleanQuality::Exact => ("exact".to_string(), None),
            BooleanQuality::Approximate { deflection } => {
                ("approximate".to_string(), Some(deflection))
            }
        };
        Ok(BooleanQualityResult {
            solid: solid_id_to_u32(outcome.solid),
            quality,
            deflection,
        })
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::MathError;
    use remus_operations::OperationsError;

    use super::*;
    use crate::error::StructuredWasmError;

    #[test]
    fn cancelled_boolean_is_typed_and_preserves_topology() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let b = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let counts_before = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
            kernel.topo().num_solids(),
        );
        let slots_before = kernel.topo().allocated_slot_count();

        let token = WasmCancellationToken::new();
        token.cancel();
        let error = kernel
            .boolean_with_cancellation_impl("fuse", a, b, &token, Some(true))
            .err()
            .unwrap();

        assert!(matches!(
            error,
            WasmError::Operations(OperationsError::Math(MathError::Cancelled))
        ));
        assert_eq!(
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_faces(),
                kernel.topo().num_solids(),
            ),
            counts_before
        );
        assert_eq!(kernel.topo().allocated_slot_count(), slots_before);
        assert!(kernel.resolve_solid(a).is_ok());
        assert!(kernel.resolve_solid(b).is_ok());

        let structured = serde_json::to_value(StructuredWasmError::from(OperationsError::Math(
            MathError::Cancelled,
        )))
        .unwrap();
        assert_eq!(structured["code"], "cancelled");
        assert_eq!(structured["category"], "cancelled");
        assert_eq!(structured["details"]["kernelCode"], "operation_cancelled");

        let public = kernel
            .boolean_with_cancellation("fuse", a, b, &token, Some(true))
            .unwrap();
        assert_eq!(public.status, CancellableOperationStatus::Cancelled);
        assert_eq!(public.code.as_deref(), Some("operation_cancelled"));
        assert!(public.result.is_none());
    }

    #[test]
    fn uncancelled_boolean_returns_a_completed_typed_result() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let b = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let token = WasmCancellationToken::new();

        let public = kernel
            .boolean_with_cancellation("fuse", a, b, &token, Some(true))
            .unwrap();

        assert_eq!(public.status, CancellableOperationStatus::Completed);
        assert!(public.code.is_none());
        let result = public.result.unwrap();
        assert_eq!(result.quality, "exact");
        assert!(result.deflection.is_none());
        assert!(kernel.resolve_solid(result.solid).is_ok());
    }
}
