//! Bounded fillet twins: `filletDetailedWithCancellation` and
//! `filletV2DetailedWithCancellation`.
//!
//! The same engines, selection rule and typed envelope as `filletDetailed` /
//! `filletV2Detailed`, under an [`OperationContext`] carrying the caller's
//! [`OperationCancellationToken`] and an optional cap on the blend walker's
//! step attempts. A run that is cancelled at one of the engines' safe points,
//! or that spends its step budget, refuses with category `cancelled`
//! (`operation_cancelled`) or `resource_limit` (`resource_limit_exceeded`,
//! resource `blend_walk_steps`, with the limit and the amount consumed), and
//! the topology is rolled back. A successful result also reports the
//! deterministic work it took as `details.walkSteps`.
//!
//! As for `booleanWithCancellation`, a single-threaded browser worker cannot
//! process a `cancel()` call while WASM runs; there the step budget is the
//! bound, and the token covers pre-cancellation and shared-memory hosts.

use remus_math::context::{OperationContext, WorkBudgets};
use serde_json::Value;
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use super::{blend_outcome, face_set};
use crate::bindings::booleans::OperationCancellationToken;
use crate::error::{StructuredWasmError, WasmError, validate_iteration_budget, validate_positive};
use crate::kernel::RemusKernel;
use crate::types::SolidOperationDetailedResult;

/// The context a bounded fillet runs under.
fn blend_context(
    token: &OperationCancellationToken,
    max_walk_steps: Option<f64>,
) -> Result<OperationContext, WasmError> {
    let mut budgets = WorkBudgets::new();
    if let Some(steps) = max_walk_steps {
        budgets = budgets.with_blend_walk_steps(validate_iteration_budget(steps, "maxWalkSteps")?);
    }
    Ok(OperationContext::new()
        .with_budgets(budgets)
        .with_cancellation(token.inner.clone()))
}

#[wasm_bindgen]
impl RemusKernel {
    /// [`filletDetailed`](Self::fillet_detailed) bounded by a cancellation
    /// token and an optional walker step budget.
    ///
    /// `maxWalkSteps` caps the blend walker's step attempts across every
    /// engine the cascade tries; omitted or `null` leaves it unbounded. A
    /// cancelled or over-budget call returns `status: "error"` with category
    /// `cancelled` or `resource_limit` and leaves the topology untouched;
    /// success adds `details.walkSteps`.
    #[wasm_bindgen(js_name = "filletDetailedWithCancellation")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_detailed_with_cancellation(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
        token: &OperationCancellationToken,
        exact_only: Option<bool>,
        max_walk_steps: Option<f64>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .governed_fillet_detailed_impl(
                "fillet",
                false,
                solid,
                &edge_handles,
                radius,
                token,
                exact_only.unwrap_or(false),
                max_walk_steps,
            )
            .into_ts()?)
    }

    /// [`filletV2Detailed`](Self::fillet_v2_detailed) bounded as
    /// [`filletDetailedWithCancellation`](Self::fillet_detailed_with_cancellation)
    /// is.
    #[wasm_bindgen(js_name = "filletV2DetailedWithCancellation")]
    #[allow(clippy::needless_pass_by_value)]
    pub fn fillet_v2_detailed_with_cancellation(
        &mut self,
        solid: u32,
        edge_handles: Vec<u32>,
        radius: f64,
        token: &OperationCancellationToken,
        exact_only: Option<bool>,
        max_walk_steps: Option<f64>,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .governed_fillet_detailed_impl(
                "filletV2",
                true,
                solid,
                &edge_handles,
                radius,
                token,
                exact_only.unwrap_or(false),
                max_walk_steps,
            )
            .into_ts()?)
    }
}

impl RemusKernel {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn governed_fillet_detailed_impl(
        &mut self,
        operation: &'static str,
        v2: bool,
        solid: u32,
        edge_handles: &[u32],
        radius: f64,
        token: &OperationCancellationToken,
        exact_only: bool,
        max_walk_steps: Option<f64>,
    ) -> SolidOperationDetailedResult {
        let label = if v2 { "filletV2" } else { "Fillet" };
        self.run_modifier_detailed(operation, label, |kernel| {
            validate_positive(radius, "radius")?;
            let context = blend_context(token, max_walk_steps)?;
            let solid_id = kernel.resolve_solid(solid)?;
            let edge_ids = edge_handles
                .iter()
                .map(|&handle| kernel.resolve_edge(handle))
                .collect::<Result<Vec<_>, _>>()?;
            let input_faces = face_set(kernel.topo(), solid_id)?;
            let governed = if v2 {
                remus_operations::blend_ops::fillet_v2_with_context(
                    kernel.topo_mut(),
                    solid_id,
                    &edge_ids,
                    radius,
                    &context,
                )
            } else {
                crate::helpers::fillet_whole_selection_with_context(
                    kernel.topo_mut(),
                    solid_id,
                    &edge_ids,
                    radius,
                    &context,
                )
            }
            .map_err(StructuredWasmError::blend_failure)?;
            let (value, mut details) = blend_outcome(
                kernel.topo(),
                operation,
                &input_faces,
                governed.result.solid,
                Some(governed.result.engine),
                exact_only,
            )?;
            details.insert("walkSteps".to_string(), Value::from(governed.walk_steps));
            Ok((value, details))
        })
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::compact_sort::CompactSort;
    use remus_math::mat::Mat4;
    use remus_topology::Topology;
    use remus_topology::face::FaceSurface;

    use super::*;
    use crate::handles::{edge_id_to_u32, solid_id_to_u32};

    /// The OpenZCAD K02/W01 sphere boss: kernel, fused solid handle and its
    /// four boss/sphere seam edge handles.
    fn sphere_boss(tilted: bool) -> (RemusKernel, u32, Vec<u32>) {
        let mut kernel = RemusKernel::new();
        let topo = kernel.topo_mut();
        let sphere = remus_operations::primitives::make_sphere(topo, 73.5, 32).unwrap();
        remus_operations::transform::transform_solid(
            topo,
            sphere,
            &Mat4::translation(25.5, -1.5, 0.0),
        )
        .unwrap();
        let boss = remus_operations::primitives::make_cylinder(topo, 6.0, 64.1).unwrap();
        let place = if tilted {
            Mat4::translation(0.0, 0.0, 58.0)
                * Mat4::rotation_y(10f64.to_radians())
                * Mat4::rotation_x(20f64.to_radians())
        } else {
            Mat4::translation(0.0, 0.0, 58.0)
        };
        remus_operations::transform::transform_solid(topo, boss, &place).unwrap();
        let fused = remus_operations::boolean::boolean(
            topo,
            remus_operations::boolean::BooleanOp::Fuse,
            sphere,
            boss,
        )
        .unwrap();
        let adjacency = topo.build_adjacency(fused).unwrap();
        let mut seams: Vec<_> = adjacency
            .edge_faces_iter()
            .filter(|(_, faces)| {
                let kinds: Vec<_> = faces
                    .iter()
                    .map(|&face| match topo.face(face).unwrap().surface() {
                        FaceSurface::Sphere(_) => 's',
                        FaceSurface::Cylinder(_) => 'c',
                        _ => 'o',
                    })
                    .collect();
                kinds.len() == 2 && kinds.contains(&'s') && kinds.contains(&'c')
            })
            .map(|(edge, _)| edge)
            .collect();
        seams.compact_sort_by_key(|edge| edge.index());
        assert_eq!(seams.len(), 4);
        let seams = seams.into_iter().map(edge_id_to_u32).collect();
        (kernel, solid_id_to_u32(fused), seams)
    }

    fn live_state(topo: &Topology) -> String {
        // Live arena content: a rolled-back attempt keeps the slots it allocated
        // retired (so no stale handle aliases a later entity), which a whole-arena
        // `Debug` would count as a difference.
        let loops: Vec<_> = topo
            .faces()
            .iter()
            .flat_map(|(_, face)| face.boundary_loops())
            .map(|id| topo.face_loop(*id).unwrap())
            .collect();
        let uses: Vec<_> = loops
            .iter()
            .flat_map(|loop_| loop_.coedges())
            .map(|id| topo.coedge(*id).unwrap())
            .collect();
        format!(
            "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
            topo.vertices().iter().collect::<Vec<_>>(),
            topo.edges().iter().collect::<Vec<_>>(),
            topo.wires().iter().collect::<Vec<_>>(),
            topo.faces().iter().collect::<Vec<_>>(),
            topo.shells().iter().collect::<Vec<_>>(),
            topo.solids().iter().collect::<Vec<_>>(),
            loops,
            uses,
            topo.attributes(),
            topo.journal().entries()
        )
    }

    fn json(result: &SolidOperationDetailedResult) -> Value {
        serde_json::to_value(result).unwrap()
    }

    #[test]
    fn bounded_fillet_matches_the_unbounded_twin_and_reports_walk_steps() {
        for v2 in [false, true] {
            let (mut plain, solid, seams) = sphere_boss(true);
            let unbounded = if v2 {
                plain.fillet_v2_detailed_impl(solid, &seams, 2.0, false)
            } else {
                plain.fillet_detailed_impl(solid, &seams, 2.0, false)
            };
            let (mut kernel, solid, seams) = sphere_boss(true);
            let token = OperationCancellationToken::new();
            let operation = if v2 { "filletV2" } else { "fillet" };
            let bounded = kernel.governed_fillet_detailed_impl(
                operation, v2, solid, &seams, 2.0, &token, false, None,
            );
            let (unbounded, bounded) = (json(&unbounded), json(&bounded));
            assert_eq!(bounded["status"], "ok", "v2={v2}: {bounded}");
            assert_eq!(bounded["value"], unbounded["value"], "v2={v2}");
            let steps = bounded["details"]["walkSteps"].as_u64().unwrap();
            assert!(steps > 0, "v2={v2}: the seam is walked");
            let mut details = bounded["details"].clone();
            details.as_object_mut().unwrap().remove("walkSteps");
            assert_eq!(details, unbounded["details"], "v2={v2}");
        }
    }

    #[test]
    fn bounded_fillet_refuses_typed_and_rolls_back() {
        let (mut kernel, solid, seams) = sphere_boss(false);
        let before = live_state(kernel.topo());
        let token = OperationCancellationToken::new();
        let budget = json(&kernel.governed_fillet_detailed_impl(
            "fillet",
            false,
            solid,
            &seams,
            2.0,
            &token,
            false,
            Some(2.0),
        ));
        assert_eq!(budget["status"], "error", "{budget}");
        assert_eq!(budget["category"], "resource_limit", "{budget}");
        assert_eq!(budget["details"]["kernelCode"], "resource-limit");
        assert_eq!(live_state(kernel.topo()), before);

        token.cancel();
        let cancelled = json(&kernel.governed_fillet_detailed_impl(
            "filletV2", true, solid, &seams, 2.0, &token, false, None,
        ));
        assert_eq!(cancelled["status"], "error", "{cancelled}");
        assert_eq!(cancelled["category"], "cancelled", "{cancelled}");
        assert_eq!(live_state(kernel.topo()), before);

        let malformed = json(&kernel.governed_fillet_detailed_impl(
            "fillet",
            false,
            solid,
            &seams,
            2.0,
            &OperationCancellationToken::new(),
            false,
            Some(1.5),
        ));
        assert_eq!(malformed["category"], "invalid_input", "{malformed}");
    }
}
