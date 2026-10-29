//! Bounded live-session planar polygon imprint and batch adapter.
#![allow(clippy::missing_errors_doc)]

use serde_json::Value;
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::helpers::{get_f64_array, get_u32, parse_points_checked};
use crate::kernel::BrepKernel;

fn count(coordinates: usize) -> Result<(), StructuredWasmError> {
    if coordinates > 48 {
        return Err(remus_math::MathError::ResourceLimitExceeded {
            resource: "imprint_polygon_coordinates",
            limit: 48,
            actual: coordinates,
        }
        .into());
    }
    Ok(())
}

fn handle(index: usize) -> Result<u32, StructuredWasmError> {
    u32::try_from(index).map_err(|_| {
        remus_math::MathError::ResourceLimitExceeded {
            resource: "imprint_result_handle",
            limit: u32::MAX as usize,
            actual: index,
        }
        .into()
    })
}

impl BrepKernel {
    /// This topology-only operation returns its new handles only on success.
    /// Validation refusals expose counts, not private result entities. Keep an
    /// already checkpoint-shared owner on Err; an unshared call keeps the
    /// ordinary mutation-local rollback and private-slot high-water behavior.
    /// This wrapper must surround the outermost transaction, before topo_mut.
    pub(crate) fn with_imprint_rollback<T, E>(
        &mut self,
        operation: impl FnOnce(&mut Self) -> Result<T, E>,
    ) -> Result<T, E> {
        let shared =
            (std::rc::Rc::strong_count(&self.topo) > 1).then(|| std::rc::Rc::clone(&self.topo));
        let result = operation(self);
        if result.is_err()
            && let Some(original) = shared
        {
            self.topo = original;
        }
        result
    }

    pub(crate) fn imprint_planar_polygon_impl(
        &mut self,
        solid: u32,
        face: u32,
        coordinates: &[f64],
    ) -> Result<Value, StructuredWasmError> {
        count(coordinates.len())?;
        let points = parse_points_checked(coordinates).map_err(StructuredWasmError::from)?;
        let solid = self
            .resolve_solid(solid)
            .map_err(StructuredWasmError::from)?;
        let face = self.resolve_face(face).map_err(StructuredWasmError::from)?;
        // Native construction coordinates topology/history atomically. This
        // mutation-local outer scope also covers JS handle/op-ID conversion.
        self.with_imprint_rollback(|kernel| {
            kernel.with_topology_transaction(|topo| {
                let result =
                    remus_operations::imprint::imprint_planar_polygon(topo, solid, face, &points)
                        .map_err(StructuredWasmError::from)?;
                // Conversion refusals only report values outside the public
                // u32 handle/op namespace; no usable private handle escapes.
                let op = u32::try_from(result.op.value()).map_err(|_| {
                    StructuredWasmError::from(remus_math::MathError::ResourceLimitExceeded {
                        resource: "imprint_operation_handle",
                        limit: u32::MAX as usize,
                        actual: usize::try_from(result.op.value()).unwrap_or(usize::MAX),
                    })
                })?;
                Ok(serde_json::json!({
                    "solid": handle(result.solid.index())?,
                    "innerFace": handle(result.inner_face.index())?,
                    "surroundFace": handle(result.surround_face.index())?,
                    "op": op,
                }))
            })
        })
    }

    pub(crate) fn imprint_planar_polygon_args(
        &mut self,
        args: &Value,
    ) -> Result<Value, StructuredWasmError> {
        let values = args
            .get("points")
            .and_then(Value::as_array)
            .ok_or_else(|| {
                StructuredWasmError::invalid_argument(
                    "'points' must be a flat coordinate array",
                    Some("points"),
                )
            })?;
        count(values.len())?;
        let coordinates = get_f64_array(args, "points")?;
        self.imprint_planar_polygon_impl(
            get_u32(args, "solid")?,
            get_u32(args, "face")?,
            &coordinates,
        )
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Partition one planar convex solid face with a strictly interior convex
    /// 3--16 point ring. `points` is flat world `[x,y,z,...]` data.
    /// Returns JSON `{solid,innerFace,surroundFace,op}`. Both patches retain
    /// the original plane and material, with exact construction lineage.
    #[wasm_bindgen(js_name = "imprintPlanarPolygon")]
    pub fn imprint_planar_polygon_js(
        &mut self,
        solid: u32,
        face: u32,
        points: &[f64],
    ) -> Result<String, JsError> {
        self.imprint_planar_polygon_impl(solid, face, points)
            .map(|result| result.to_string())
            .map_err(|error| JsError::new(error.message()))
    }
}

#[cfg(test)]
mod tests;
