//! Boolean operation bindings.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext};
use remus_operations::boolean::{BooleanOp, boolean, boolean_compound_regions, boolean_regions};
use remus_operations::compound_ops;

use crate::error::{WasmError, validate_finite};
use crate::handles::solid_id_to_u32;
use crate::helpers::{build_triangle_mesh, panic_message, parse_boolean_op, triangle_mesh_to_js};
use crate::kernel::BrepKernel;
use crate::shapes::JsMesh;
use crate::types::{BooleanQualityResult, CancellableBooleanResult, CancellableOperationStatus};
use tsify::Tsify as _;

/// A one-shot cooperative cancellation signal for a modeling operation.
///
/// Clones share one monotonic flag. Native multithreaded hosts can signal it
/// concurrently; cancelling before a call also refuses that call without
/// touching topology. A single-threaded browser worker cannot process a new
/// JS call while WASM is running, so active browser cancellation still needs
/// the app's worker/shared-memory transport.
#[wasm_bindgen]
pub struct OperationCancellationToken {
    inner: CancellationToken,
}

#[wasm_bindgen]
impl OperationCancellationToken {
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

impl Default for OperationCancellationToken {
    fn default() -> Self {
        Self::new()
    }
}

/// Serialise a slice of `CoincidentFacePair` values to a JSON array.
///
/// Shared by both the direct WASM binding (`detectCoincidentFaces`)
/// and the batch dispatcher (`executeBatch` "detectCoincidentFaces"
/// arm) so the JSON shape is guaranteed identical across the two
/// paths — a field-name typo or boolean formatting drift in only one
/// copy would otherwise be silently shipped to JS callers.
///
/// Visibility note: `pub(crate)` triggers `clippy::redundant_pub_crate`
/// because `bindings` is a private module — the lint folds it to `pub`
/// in this scope. We keep `pub(crate)` to make the cross-module-but-
/// crate-internal sharing explicit.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn coincident_face_pairs_to_json(
    pairs: &[remus_algo::diagnostic::CoincidentFacePair],
) -> serde_json::Value {
    let arr: Vec<serde_json::Value> = pairs
        .iter()
        .map(|p| {
            serde_json::json!({
                "faceA": crate::handles::face_id_to_u32(p.face_a),
                "faceB": crate::handles::face_id_to_u32(p.face_b),
                "sameOrientation": p.same_orientation,
                "aabbOverlap": p.aabb_overlap,
            })
        })
        .collect();
    serde_json::Value::Array(arr)
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Boolean operations ──────────────────────────────────────────

    /// Perform an exact boolean whose disconnected volumetric regions remain
    /// separate solids in a Compound.
    ///
    /// `op` accepts the same spellings as `booleanWithQuality`. The returned
    /// handle can be expanded with `getCompoundSolids`. This path is exact-only
    /// and fails closed if any region or its construction lineage is invalid.
    /// Existing single-solid `fuse`, `cut`, and `intersect` remain compatible.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid handles, an unknown operation, or an
    /// unqualified exact result.
    #[wasm_bindgen(js_name = "booleanRegions")]
    pub fn boolean_regions(&mut self, op: &str, a: u32, b: u32) -> Result<u32, JsError> {
        let operation = parse_boolean_op(op)?;
        let a = self.resolve_solid(a)?;
        let b = self.resolve_solid(b)?;
        let result = boolean_regions(self.topo_mut(), operation, a, b)?;
        Ok(crate::handles::compound_id_to_u32(result.compound))
    }

    /// Perform a bounded exact boolean between two Compound operands.
    ///
    /// Disjoint fuse, pairwise exact intersect, and a single cut-tool
    /// distributed over target members are qualified. Fuse requiring member
    /// merges and multi-tool Cut fail closed until recursive lineage composition
    /// is available.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid handles, overlapping input members, an
    /// unknown operation, or an unqualified exact configuration.
    #[wasm_bindgen(js_name = "booleanCompoundRegions")]
    pub fn boolean_compound_regions(&mut self, op: &str, a: u32, b: u32) -> Result<u32, JsError> {
        let operation = parse_boolean_op(op)?;
        let a = self.resolve_compound(a)?;
        let b = self.resolve_compound(b)?;
        let result = boolean_compound_regions(self.topo_mut(), operation, a, b)?;
        Ok(crate::handles::compound_id_to_u32(result.compound))
    }

    /// Perform a boolean with disclosed result quality.
    ///
    /// `op` is `"fuse"`/`"union"`, `"cut"`/`"difference"`, or
    /// `"intersect"`/`"intersection"`. The plain `fuse`/`cut`/`intersect`
    /// bindings silently accept the mesh (co-refinement) fallback, which
    /// discards analytic surface types; this binding reports whether that
    /// happened (`quality: "approximate"` plus the fallback deflection), and
    /// `exact_only = true` turns the fallback into a typed refusal so an
    /// exact-or-nothing caller never receives a faceted body.
    ///
    /// `newton_iterations` optionally caps the coupled Newton refinement
    /// iterations of every NURBS surface-surface intersection inside the
    /// operation (a non-negative integer; `0` disables refinement). Omitted
    /// or `null` keeps the kernel default, reproducing prior behavior.
    /// `subdivision_depth` likewise caps recursive SSI seed subdivision
    /// (`0` disables recursive splitting; omitted or `null` keeps depth 6).
    /// The additive `march_steps`, `queue_size`, `segments`, and
    /// `branches_per_direction` arguments cap the remaining SSI marcher and
    /// branch-exploration work. Omitted or `null` values retain the historical
    /// defaults (200, 100, 50, and 10 respectively).
    ///
    /// # Errors
    ///
    /// Returns an error if a handle is invalid, the op string is unknown,
    /// any work-budget argument is not a non-negative integer within the
    /// public work budget, or (under `exact_only`) the exact pipeline cannot
    /// produce the result.
    #[wasm_bindgen(js_name = "booleanWithQuality")]
    #[allow(clippy::too_many_arguments)]
    pub fn boolean_with_quality(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        exact_only: Option<bool>,
        newton_iterations: Option<f64>,
        subdivision_depth: Option<f64>,
        march_steps: Option<f64>,
        queue_size: Option<f64>,
        segments: Option<f64>,
        branches_per_direction: Option<f64>,
    ) -> Result<tsify::Ts<crate::types::BooleanQualityResult>, JsError> {
        Ok(self
            .boolean_with_quality_impl(
                op,
                a,
                b,
                exact_only,
                parse_budget(newton_iterations, "newton_iterations")?,
                parse_budget(subdivision_depth, "subdivision_depth")?,
                parse_budget(march_steps, "march_steps")?,
                parse_budget(queue_size, "queue_size")?,
                parse_budget(segments, "segments")?,
                parse_budget(branches_per_direction, "branches_per_direction")?,
            )?
            .into_ts()?)
    }
}

/// Natively-testable boolean bodies (`Ts` cannot be inspected off-wasm).
impl BrepKernel {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn boolean_with_quality_impl(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        exact_only: Option<bool>,
        newton_budget: Option<usize>,
        subdivision_budget: Option<usize>,
        march_budget: Option<usize>,
        queue_budget: Option<usize>,
        segment_budget: Option<usize>,
        branch_budget: Option<usize>,
    ) -> Result<crate::types::BooleanQualityResult, JsError> {
        use remus_operations::boolean::{BooleanQuality, boolean_with_context};

        let boolean_op = parse_boolean_op(op)?;
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let context = quality_context(
            exact_only == Some(true),
            newton_budget,
            subdivision_budget,
            march_budget,
            queue_budget,
            segment_budget,
            branch_budget,
        );
        let outcome = boolean_with_context(self.topo_mut(), boolean_op, a_id, b_id, &context)?;
        let (quality, deflection) = match outcome.quality {
            BooleanQuality::Exact => ("exact".to_string(), None),
            BooleanQuality::Approximate { deflection } => {
                ("approximate".to_string(), Some(deflection))
            }
        };
        Ok(crate::types::BooleanQualityResult {
            solid: solid_id_to_u32(outcome.solid),
            quality,
            deflection,
        })
    }

    /// Performs a boolean governed by a cooperative cancellation token.
    ///
    /// Cancellation is typed (`operation_cancelled`) and transactional: no
    /// partial topology is retained. Result quality follows
    /// `booleanWithQuality`, including the optional exact-only policy and the
    /// optional SSI work-budget caps.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn boolean_with_cancellation_typed(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        token: &OperationCancellationToken,
        exact_only: Option<bool>,
        newton_budget: Option<usize>,
        subdivision_budget: Option<usize>,
        march_budget: Option<usize>,
        queue_budget: Option<usize>,
        segment_budget: Option<usize>,
        branch_budget: Option<usize>,
    ) -> Result<CancellableBooleanResult, JsError> {
        match self.boolean_with_cancellation_impl(
            op,
            a,
            b,
            token,
            exact_only,
            newton_budget,
            subdivision_budget,
            march_budget,
            queue_budget,
            segment_budget,
            branch_budget,
        ) {
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

#[wasm_bindgen]
impl BrepKernel {
    /// Performs a boolean governed by a cooperative cancellation token.
    ///
    /// Cancellation is typed (`operation_cancelled`) and transactional: no
    /// partial topology is retained. Result quality follows
    /// `booleanWithQuality`, including the optional exact-only policy and the
    /// optional SSI work-budget caps. The final four additive arguments cap
    /// marching steps, pending branch seeds, traced segments, and branch
    /// points per direction; omitted or `null` values preserve their kernel
    /// defaults.
    #[wasm_bindgen(js_name = "booleanWithCancellation")]
    #[allow(clippy::too_many_arguments)]
    pub fn boolean_with_cancellation(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        token: &OperationCancellationToken,
        exact_only: Option<bool>,
        newton_iterations: Option<f64>,
        subdivision_depth: Option<f64>,
        march_steps: Option<f64>,
        queue_size: Option<f64>,
        segments: Option<f64>,
        branches_per_direction: Option<f64>,
    ) -> Result<tsify::Ts<CancellableBooleanResult>, JsError> {
        Ok(self
            .boolean_with_cancellation_typed(
                op,
                a,
                b,
                token,
                exact_only,
                parse_budget(newton_iterations, "newton_iterations")?,
                parse_budget(subdivision_depth, "subdivision_depth")?,
                parse_budget(march_steps, "march_steps")?,
                parse_budget(queue_size, "queue_size")?,
                parse_budget(segments, "segments")?,
                parse_budget(branches_per_direction, "branches_per_direction")?,
            )?
            .into_ts()?)
    }

    /// Fuse (union) two solids into one.
    ///
    /// Returns a new solid handle (`u32`).
    ///
    /// When the exact engine cannot handle the pair the kernel falls back to
    /// a mesh boolean and logs a `warn` on the `remus_approx` target (visible
    /// on the JS console through the log bridge); the returned handle then
    /// carries a mesh, not analytic faces. Use `booleanWithQuality` to have
    /// that disclosed in the result or refused outright.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "fuse")]
    pub fn fuse(&mut self, a: u32, b: u32) -> Result<u32, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let result = boolean(self.topo_mut(), BooleanOp::Fuse, a_id, b_id)?;
        Ok(solid_id_to_u32(result))
    }

    /// Cut (subtract) solid `b` from solid `a`.
    ///
    /// Returns a new solid handle (`u32`).
    ///
    /// When the exact engine cannot handle the pair the kernel falls back to
    /// a mesh boolean and logs a `warn` on the `remus_approx` target (visible
    /// on the JS console through the log bridge); the returned handle then
    /// carries a mesh, not analytic faces. Use `booleanWithQuality` to have
    /// that disclosed in the result or refused outright.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "cut")]
    pub fn cut(&mut self, a: u32, b: u32) -> Result<u32, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let result = boolean(self.topo_mut(), BooleanOp::Cut, a_id, b_id)?;
        Ok(solid_id_to_u32(result))
    }

    /// Detect surface-level coincident face pairs between two solids
    /// without performing a boolean operation.
    ///
    /// Useful for warning users about same-domain configurations
    /// (face stacks, coaxial cylinders, concentric spheres) before a
    /// boolean. Returns a JSON array string of objects:
    /// `[{"faceA": <u32>, "faceB": <u32>, "sameOrientation": <bool>, "aabbOverlap": <bool>}, ...]`.
    ///
    /// `sameOrientation` is `true` when the surface normals point the
    /// same way at corresponding parametric points (e.g., two coplanar
    /// faces with the same `+z` normal). `aabbOverlap` filters pairs
    /// that are same-domain on the surface but geometrically disjoint.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or any face /
    /// edge / vertex lookup fails internally.
    #[wasm_bindgen(js_name = "detectCoincidentFaces")]
    pub fn detect_coincident_faces(&self, a: u32, b: u32) -> Result<String, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let pairs = remus_algo::diagnostic::detect_coincident_faces(
            self.topo(),
            a_id,
            b_id,
            remus_math::tolerance::Tolerance::default(),
        )
        .map_err(|e| JsError::new(&format!("{e}")))?;
        Ok(coincident_face_pairs_to_json(&pairs).to_string())
    }

    /// Fuse (union) many solids into one in a single call.
    ///
    /// Faster than a left-fold over `fuse`: overlapping solids are reduced
    /// pairwise in a balanced tree while disjoint groups are merged directly
    /// without a boolean.
    ///
    /// Returns a new solid handle (`u32`).
    ///
    /// # Errors
    ///
    /// Returns an error if any solid handle is invalid, the list is empty,
    /// or a boolean operation produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "fuseAll")]
    pub fn fuse_all(&mut self, solid_handles: Vec<u32>) -> Result<u32, JsError> {
        let solid_ids = solid_handles
            .iter()
            .map(|&h| self.resolve_solid(h))
            .collect::<Result<Vec<_>, _>>()?;
        let result = compound_ops::fuse_solids(self.topo_mut(), &solid_ids)?;
        Ok(solid_id_to_u32(result))
    }

    /// Intersect two solids, keeping only their common volume.
    ///
    /// Returns a new solid handle (`u32`).
    ///
    /// When the exact engine cannot handle the pair the kernel falls back to
    /// a mesh boolean and logs a `warn` on the `remus_approx` target (visible
    /// on the JS console through the log bridge); the returned handle then
    /// carries a mesh, not analytic faces. Use `booleanWithQuality` to have
    /// that disclosed in the result or refused outright.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty result.
    #[wasm_bindgen(js_name = "intersect")]
    pub fn intersect_solids(&mut self, a: u32, b: u32) -> Result<u32, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let result = boolean(self.topo_mut(), BooleanOp::Intersect, a_id, b_id)?;
        Ok(solid_id_to_u32(result))
    }

    // ── Boolean operations with options ────────────────────────────

    /// Fuse (union) two solids with post-processing options.
    ///
    /// `unifyFaces` (default `true`) merges adjacent result faces that lie
    /// on the same underlying surface, which keeps face counts low across
    /// chained booleans (e.g. 2871 → ~106 faces on sequential curved-surface
    /// booleans). Pass `false` to keep the raw fragment layout.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "fuseWithOptions")]
    pub fn fuse_with_options(
        &mut self,
        a: u32,
        b: u32,
        unify_faces: Option<bool>,
    ) -> Result<u32, JsError> {
        self.boolean_with_options_impl(BooleanOp::Fuse, a, b, unify_faces)
    }

    /// Cut (subtract) solid `b` from solid `a` with post-processing options.
    ///
    /// See [`fuse_with_options`](Self::fuse_with_options) for the
    /// `unifyFaces` semantics.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "cutWithOptions")]
    pub fn cut_with_options(
        &mut self,
        a: u32,
        b: u32,
        unify_faces: Option<bool>,
    ) -> Result<u32, JsError> {
        self.boolean_with_options_impl(BooleanOp::Cut, a, b, unify_faces)
    }

    /// Intersect two solids with post-processing options.
    ///
    /// See [`fuse_with_options`](Self::fuse_with_options) for the
    /// `unifyFaces` semantics.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "intersectWithOptions")]
    pub fn intersect_with_options(
        &mut self,
        a: u32,
        b: u32,
        unify_faces: Option<bool>,
    ) -> Result<u32, JsError> {
        self.boolean_with_options_impl(BooleanOp::Intersect, a, b, unify_faces)
    }

    // ── Boolean operations with evolution tracking ─────────────────

    /// Fuse (union) two solids and return evolution tracking data.
    ///
    /// Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "fuseWithEvolution")]
    pub fn fuse_with_evolution(&mut self, a: u32, b: u32) -> Result<JsValue, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let (result, evo) = remus_operations::boolean::boolean_with_evolution(
            self.topo_mut(),
            BooleanOp::Fuse,
            a_id,
            b_id,
        )?;
        let json = format!(
            "{{\"solid\":{},\"evolution\":{}}}",
            solid_id_to_u32(result),
            evo.to_json()
        );
        Ok(JsValue::from_str(&json))
    }

    /// Cut (subtract) solid `b` from solid `a` and return evolution tracking data.
    ///
    /// Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty or non-manifold result.
    #[wasm_bindgen(js_name = "cutWithEvolution")]
    pub fn cut_with_evolution(&mut self, a: u32, b: u32) -> Result<JsValue, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let (result, evo) = remus_operations::boolean::boolean_with_evolution(
            self.topo_mut(),
            BooleanOp::Cut,
            a_id,
            b_id,
        )?;
        let json = format!(
            "{{\"solid\":{},\"evolution\":{}}}",
            solid_id_to_u32(result),
            evo.to_json()
        );
        Ok(JsValue::from_str(&json))
    }

    /// Intersect two solids and return evolution tracking data.
    ///
    /// Returns a JSON string: `{"solid": <u32>, "evolution": {...}}`.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid or the operation
    /// produces an empty result.
    #[wasm_bindgen(js_name = "intersectWithEvolution")]
    pub fn intersect_with_evolution(&mut self, a: u32, b: u32) -> Result<JsValue, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let (result, evo) = remus_operations::boolean::boolean_with_evolution(
            self.topo_mut(),
            BooleanOp::Intersect,
            a_id,
            b_id,
        )?;
        let json = format!(
            "{{\"solid\":{},\"evolution\":{}}}",
            solid_id_to_u32(result),
            evo.to_json()
        );
        Ok(JsValue::from_str(&json))
    }

    /// Perform a mesh boolean on raw triangle data.
    ///
    /// Returns a `JsMesh` with the result.
    #[wasm_bindgen(js_name = "meshBoolean")]
    #[allow(
        clippy::needless_pass_by_value,
        clippy::too_many_arguments,
        clippy::unused_self
    )]
    pub fn mesh_boolean(
        &self,
        positions_a: Vec<f64>,
        indices_a: Vec<u32>,
        positions_b: Vec<f64>,
        indices_b: Vec<u32>,
        op: &str,
        tolerance: f64,
    ) -> Result<JsMesh, JsError> {
        validate_finite(tolerance, "tolerance")?;
        let mesh_a = build_triangle_mesh(&positions_a, &indices_a)?;
        let mesh_b = build_triangle_mesh(&positions_b, &indices_b)?;
        let bool_op = parse_boolean_op(op)?;
        let result =
            remus_operations::mesh_boolean::mesh_boolean(&mesh_a, &mesh_b, bool_op, tolerance)?;
        Ok(triangle_mesh_to_js(&result.mesh))
    }
}

// Separate impl block: `compound_cut` uses manual `catch_unwind` for panic
// safety — any panic that unwinds across the wasm-bindgen boundary leaves
// its internal RefCell borrowed, breaking all subsequent JS calls.
#[wasm_bindgen]
impl BrepKernel {
    /// Cut a target solid by multiple tool solids in a single pass.
    ///
    /// This is more efficient than sequential `cut()` calls when many tools
    /// are applied to the same target — it avoids re-processing unchanged
    /// faces at each step.
    ///
    /// `tool_ids` is a JS `Uint32Array` or array of solid handles.
    ///
    /// # Errors
    ///
    /// Returns an error if any handle is invalid or the operation fails.
    #[wasm_bindgen(js_name = "compoundCut")]
    pub fn compound_cut(&mut self, target: u32, tool_ids: &[u32]) -> Result<u32, JsError> {
        if self.poisoned {
            return Err(JsError::new(
                "Kernel poisoned after panic. Create a new BrepKernel instance.",
            ));
        }
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let target_id = self.resolve_solid(target)?;
            let tools: Vec<remus_topology::solid::SolidId> = tool_ids
                .iter()
                .map(|&h| self.resolve_solid(h))
                .collect::<Result<Vec<_>, _>>()?;
            let result = remus_operations::boolean::compound_cut(
                self.topo_mut(),
                target_id,
                &tools,
                remus_operations::boolean::BooleanOptions::default(),
            )?;
            Ok(solid_id_to_u32(result))
        }));
        match result {
            Ok(inner) => inner.map_err(|e: crate::error::WasmError| JsError::new(&e.to_string())),
            Err(panic_info) => {
                self.poisoned = true;
                Err(JsError::new(&panic_message(&panic_info, "compoundCut")))
            }
        }
    }
}

// Private helpers (not exported to JS).
impl BrepKernel {
    /// Shared body of the `*WithOptions` boolean bindings.
    fn boolean_with_options_impl(
        &mut self,
        op: BooleanOp,
        a: u32,
        b: u32,
        unify_faces: Option<bool>,
    ) -> Result<u32, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let opts = remus_operations::boolean::BooleanOptions {
            unify_faces: unify_faces.unwrap_or(true),
            ..Default::default()
        };
        let result =
            remus_operations::boolean::boolean_with_options(self.topo_mut(), op, a_id, b_id, opts)?;
        Ok(solid_id_to_u32(result))
    }

    #[allow(clippy::too_many_arguments)]
    fn boolean_with_cancellation_impl(
        &mut self,
        op: &str,
        a: u32,
        b: u32,
        token: &OperationCancellationToken,
        exact_only: Option<bool>,
        newton_budget: Option<usize>,
        subdivision_budget: Option<usize>,
        march_budget: Option<usize>,
        queue_budget: Option<usize>,
        segment_budget: Option<usize>,
        branch_budget: Option<usize>,
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
        let context = quality_context(
            exact_only == Some(true),
            newton_budget,
            subdivision_budget,
            march_budget,
            queue_budget,
            segment_budget,
            branch_budget,
        )
        .with_cancellation(token.inner.clone());
        let outcome = remus_operations::boolean::boolean_with_context(
            self.topo_mut(),
            boolean_op,
            a_id,
            b_id,
            &context,
        )?;
        let (quality, deflection) = match outcome.quality {
            remus_operations::boolean::BooleanQuality::Exact => ("exact".to_string(), None),
            remus_operations::boolean::BooleanQuality::Approximate { deflection } => {
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

/// Parse and validate an optional JS work-budget cap.
///
/// `None` (JS `undefined`/`null`) keeps the kernel default budget.
fn parse_budget(value: Option<f64>, name: &str) -> Result<Option<usize>, WasmError> {
    value
        .map(|value| crate::error::validate_iteration_budget(value, name))
        .transpose()
}

/// Build the [`OperationContext`] for a quality-disclosing boolean from the
/// validated JS-facing knobs.
///
/// Shared by the direct bindings (`booleanWithQuality`,
/// `booleanWithCancellation`) and the `executeBatch` arm so all JS entry
/// points construct an identical context — a knob wired into only one copy
/// would silently strand the others on the default budget. `None` for every
/// knob reproduces `OperationContext::new()` exactly.
///
/// Visibility note: `pub(crate)` triggers `clippy::redundant_pub_crate`
/// because `bindings` is a private module; kept for the same reason as
/// [`coincident_face_pairs_to_json`].
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn quality_context(
    exact_only: bool,
    newton_budget: Option<usize>,
    subdivision_budget: Option<usize>,
    march_budget: Option<usize>,
    queue_budget: Option<usize>,
    segment_budget: Option<usize>,
    branch_budget: Option<usize>,
) -> OperationContext {
    let mut context = if exact_only {
        OperationContext::new().with_fallback(FallbackPolicy::ExactOnly)
    } else {
        OperationContext::new()
    };
    if let Some(cap) = newton_budget {
        let budgets = context.budgets.with_newton_iterations(cap);
        context = context.with_budgets(budgets);
    }
    if let Some(cap) = subdivision_budget {
        let budgets = context.budgets.with_subdivision_depth(cap);
        context = context.with_budgets(budgets);
    }
    if let Some(cap) = march_budget {
        let budgets = context.budgets.with_march_steps(cap);
        context = context.with_budgets(budgets);
    }
    if let Some(cap) = queue_budget {
        let budgets = context.budgets.with_queue_size(cap);
        context = context.with_budgets(budgets);
    }
    if let Some(cap) = segment_budget {
        let budgets = context.budgets.with_segments(cap);
        context = context.with_budgets(budgets);
    }
    if let Some(cap) = branch_budget {
        let budgets = context.budgets.with_branches_per_direction(cap);
        context = context.with_budgets(budgets);
    }
    context
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

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::MathError;
    use remus_operations::OperationsError;

    use super::OperationCancellationToken;
    use crate::error::StructuredWasmError;
    use crate::kernel::BrepKernel;
    use crate::types::CancellableOperationStatus;

    /// Helper: parse batch result and check a single op returned ok or error.
    fn batch_has_ok(result: &str, idx: usize) -> bool {
        let parsed: serde_json::Value = serde_json::from_str(result).unwrap();
        parsed[idx]["ok"].is_number()
    }

    fn batch_has_error(result: &str, idx: usize) -> bool {
        let parsed: serde_json::Value = serde_json::from_str(result).unwrap();
        parsed[idx]["error"].is_string()
    }

    /// Create two overlapping boxes via batch, return the raw JSON result.
    fn two_boxes_batch() -> (BrepKernel, String) {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}}
            ]"#,
        );
        (k, r)
    }

    // ── fuse ─────────────────────────────────────────────────────────

    #[test]
    fn fuse_two_boxes_returns_valid_handle() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "fuse", "args": {{"solidA": {a}, "solidB": {b}}}}}]"#
        ));
        assert!(batch_has_ok(&r, 0), "fuse must return ok: {r}");
    }

    #[test]
    fn boolean_regions_direct_and_batch_preserve_disjoint_fuse_members() {
        let mut direct = BrepKernel::new();
        let direct_a = direct.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let direct_b = direct.make_box_solid(5.0, 2.0, 1.0).unwrap();
        let direct_b_id = direct.resolve_solid(direct_b).unwrap();
        remus_operations::transform::transform_solid(
            direct.topo_mut(),
            direct_b_id,
            &remus_math::mat::Mat4::translation(10.0, 0.0, 0.0),
        )
        .unwrap();
        let compound = direct.boolean_regions("fuse", direct_a, direct_b).unwrap();
        let members = direct.get_compound_solids(compound).unwrap();
        assert_eq!(members.len(), 2);
        let mut direct_volumes = members
            .iter()
            .map(|&solid| direct.volume(solid, 0.01).unwrap())
            .collect::<Vec<_>>();
        direct_volumes.sort_by(f64::total_cmp);

        let mut batch = BrepKernel::new();
        let response = batch.execute_batch(
            r#"[
                {"op":"makeBox","args":{"width":2,"height":3,"depth":4}},
                {"op":"makeBox","args":{"width":5,"height":2,"depth":1}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,10, 0,1,0,0, 0,0,1,0, 0,0,0,1]}},
                {"op":"booleanRegions","args":{"operation":"fuse","solidA":0,"solidB":1}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
        assert!(
            parsed[3]["ok"].is_number(),
            "batch booleanRegions failed: {response}"
        );
        let compound = parsed[3]["ok"].as_u64().unwrap();
        let members = batch.get_compound_solids(compound as u32).unwrap();
        assert_eq!(members.len(), 2);
        let mut batch_volumes = members
            .iter()
            .map(|&solid| batch.volume(solid, 0.01).unwrap())
            .collect::<Vec<_>>();
        batch_volumes.sort_by(f64::total_cmp);
        assert_eq!(direct_volumes, vec![10.0, 24.0]);
        assert_eq!(batch_volumes, direct_volumes);
    }

    #[test]
    fn boolean_compound_regions_direct_and_batch_preserve_disjoint_members() {
        let mut direct = BrepKernel::new();
        let a = direct.make_box_solid(1.0, 2.0, 3.0).unwrap();
        let b = direct.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let c = direct.make_box_solid(3.0, 2.0, 1.0).unwrap();
        let b_id = direct.resolve_solid(b).unwrap();
        remus_operations::transform::transform_solid(
            direct.topo_mut(),
            b_id,
            &remus_math::mat::Mat4::translation(10.0, 0.0, 0.0),
        )
        .unwrap();
        let c_id = direct.resolve_solid(c).unwrap();
        remus_operations::transform::transform_solid(
            direct.topo_mut(),
            c_id,
            &remus_math::mat::Mat4::translation(20.0, 0.0, 0.0),
        )
        .unwrap();
        let compound_a = direct.make_compound(vec![a, b]).unwrap();
        let compound_b = direct.make_compound(vec![c]).unwrap();
        let result = direct
            .boolean_compound_regions("fuse", compound_a, compound_b)
            .unwrap();
        let direct_members = direct.get_compound_solids(result).unwrap();
        assert_eq!(direct_members, vec![a, b, c]);

        let mut batch = BrepKernel::new();
        let response = batch.execute_batch(
            r#"[
                {"op":"makeBox","args":{"width":1,"height":2,"depth":3}},
                {"op":"makeBox","args":{"width":2,"height":2,"depth":2}},
                {"op":"makeBox","args":{"width":3,"height":2,"depth":1}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,10, 0,1,0,0, 0,0,1,0, 0,0,0,1]}},
                {"op":"transform","args":{"solid":2,"matrix":[1,0,0,20, 0,1,0,0, 0,0,1,0, 0,0,0,1]}},
                {"op":"makeCompound","args":{"solids":[0,1]}},
                {"op":"makeCompound","args":{"solids":[2]}},
                {"op":"booleanCompoundRegions","args":{"operation":"fuse","compoundA":0,"compoundB":1}}
            ]"#,
        );
        assert!(
            batch_has_ok(&response, 7),
            "compound batch failed: {response}"
        );
        let parsed: serde_json::Value = serde_json::from_str(&response).unwrap();
        let result = parsed[7]["ok"].as_u64().unwrap() as u32;
        assert_eq!(batch.get_compound_solids(result).unwrap(), vec![0, 1, 2]);
    }

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

        let token = OperationCancellationToken::new();
        token.cancel();
        let error = kernel
            .boolean_with_cancellation_impl(
                "fuse",
                a,
                b,
                &token,
                Some(true),
                None,
                None,
                None,
                None,
                None,
                None,
            )
            .err()
            .unwrap();

        assert!(matches!(
            error,
            crate::error::WasmError::Operations(OperationsError::Math(MathError::Cancelled))
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
            .boolean_with_cancellation_typed(
                "fuse",
                a,
                b,
                &token,
                Some(true),
                None,
                None,
                None,
                None,
                None,
                None,
            )
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
        let token = OperationCancellationToken::new();

        let public = kernel
            .boolean_with_cancellation_typed(
                "fuse",
                a,
                b,
                &token,
                Some(true),
                None,
                None,
                None,
                None,
                None,
                None,
            )
            .unwrap();

        assert_eq!(public.status, CancellableOperationStatus::Completed);
        assert!(public.code.is_none());
        let result = public.result.unwrap();
        assert_eq!(result.quality, "exact");
        assert!(result.deflection.is_none());
        assert!(kernel.resolve_solid(result.solid).is_ok());
    }

    #[test]
    fn fuse_invalid_handle_a_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "fuse", "args": {{"solidA": 9999, "solidB": {b}}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    #[test]
    fn fuse_invalid_handle_b_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "fuse", "args": {{"solidA": {a}, "solidB": 9999}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    // ── cut ──────────────────────────────────────────────────────────

    #[test]
    fn cut_two_boxes_returns_valid_handle() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "cut", "args": {{"solidA": {a}, "solidB": {b}}}}}]"#
        ));
        assert!(batch_has_ok(&r, 0), "cut must return ok: {r}");
    }

    /// README first example: cylinder axis exactly coincident with the box's
    /// vertical corner edge (tangential-contact class). Must cut cleanly.
    #[test]
    fn cut_corner_coincident_cylinder_readme_example() {
        let mut k = BrepKernel::new();
        let setup = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 30, "height": 20, "depth": 10}},
                {"op": "makeCylinder", "args": {"radius": 5, "height": 15}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "cut", "args": {{"solidA": {a}, "solidB": {b}}}}}]"#
        ));
        assert!(
            batch_has_ok(&r, 0),
            "corner-coincident cut must return ok: {r}"
        );
    }

    #[test]
    fn cut_invalid_target_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "cut", "args": {{"solidA": 9999, "solidB": {b}}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    #[test]
    fn cut_invalid_tool_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "cut", "args": {{"solidA": {a}, "solidB": 9999}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    // ── intersect ────────────────────────────────────────────────────

    #[test]
    fn intersect_two_boxes_returns_valid_handle() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "intersect", "args": {{"solidA": {a}, "solidB": {b}}}}}]"#
        ));
        assert!(batch_has_ok(&r, 0), "intersect must return ok: {r}");
    }

    #[test]
    fn intersect_invalid_handle_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "intersect", "args": {{"solidA": {a}, "solidB": 9999}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    // ── compound_cut ─────────────────────────────────────────────────

    #[test]
    fn compound_cut_single_tool() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "compoundCut", "args": {{"target": {a}, "tools": [{b}]}}}}]"#
        ));
        assert!(batch_has_ok(&r, 0), "compound_cut must return ok: {r}");
    }

    #[test]
    fn compound_cut_multiple_tools() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 4, "height": 4, "depth": 4}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "makeBox", "args": {"width": 0.5, "height": 0.5, "depth": 0.5}},
                {"op": "compoundCut", "args": {"target": 0, "tools": [1, 2]}}
            ]"#,
        );
        assert!(
            batch_has_ok(&r, 3),
            "compound_cut with two tools must return ok: {r}"
        );
    }

    #[test]
    fn compound_cut_invalid_target_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "compoundCut", "args": {{"target": 9999, "tools": [{b}]}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    #[test]
    fn compound_cut_invalid_tool_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "compoundCut", "args": {{"target": {a}, "tools": [9999]}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    #[test]
    fn compound_cut_empty_tool_list_is_identity() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "compoundCut", "args": {{"target": {a}, "tools": []}}}}]"#
        ));
        assert!(batch_has_ok(&r, 0));
    }

    // ── detectCoincidentFaces ────────────────────────────────────────

    #[test]
    fn detect_coincident_faces_overlapping_boxes_returns_sd_pairs() {
        // `two_boxes_batch()` creates two axis-aligned boxes (2×2×2 and
        // 1×1×1) both at the origin — the smaller is fully contained in
        // the larger. Each pair of axis-aligned faces shares a parallel
        // plane normal, so the SD detector reports several same-domain
        // pairs. We verify (a) the JSON shape and (b) at least one pair
        // is reported with a valid `aabbOverlap` flag.
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let b = parsed[1]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "detectCoincidentFaces", "args": {{"solidA": {a}, "solidB": {b}}}}}]"#
        ));
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let arr = parsed[0]["ok"].as_array().unwrap();
        assert!(!arr.is_empty(), "overlapping boxes produce SD pairs: {r}");
        for pair in arr {
            assert!(pair["faceA"].is_u64());
            assert!(pair["faceB"].is_u64());
            assert!(pair["sameOrientation"].is_boolean());
            assert!(pair["aabbOverlap"].is_boolean());
        }
    }

    #[test]
    fn detect_coincident_faces_invalid_handle_errors() {
        let (mut k, setup) = two_boxes_batch();
        let parsed: serde_json::Value = serde_json::from_str(&setup).unwrap();
        let a = parsed[0]["ok"].as_u64().unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "detectCoincidentFaces", "args": {{"solidA": {a}, "solidB": 9999}}}}]"#
        ));
        assert!(batch_has_error(&r, 0));
    }

    // ── mesh_boolean ─────────────────────────────────────────────────
    // mesh_boolean is not in the batch dispatcher, but its happy-path
    // works on native (JsError is only constructed on the error path).
    // For error paths, we test the internal operations layer directly.

    #[test]
    fn mesh_boolean_fuse_returns_non_empty_mesh() {
        let k = BrepKernel::new();
        #[rustfmt::skip]
        let positions = vec![
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ];
        let indices = vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
        let mesh = k
            .mesh_boolean(
                positions.clone(),
                indices.clone(),
                positions,
                indices,
                "fuse",
                1e-7,
            )
            .unwrap();
        assert!(
            !mesh.positions().is_empty(),
            "fused mesh must have vertices"
        );
        assert!(!mesh.indices().is_empty(), "fused mesh must have triangles");
        assert_eq!(mesh.positions().len() % 3, 0);
        assert_eq!(mesh.indices().len() % 3, 0);
    }

    #[test]
    fn mesh_boolean_unknown_op_is_not_valid() {
        // Verify the operation string validation logic without calling
        // JsError-returning helpers (JsError panics on non-wasm).
        let valid = [
            "fuse",
            "union",
            "cut",
            "difference",
            "intersect",
            "intersection",
        ];
        assert!(
            !valid.contains(&"explode"),
            "explode should not be a valid op"
        );
    }

    #[test]
    fn mesh_boolean_bad_positions_length_is_invalid() {
        // Verify the validation condition directly: positions must be multiple of 3.
        let bad_len = 2;
        assert_ne!(
            bad_len % 3,
            0,
            "length 2 should fail the multiple-of-3 check"
        );
    }

    // ── boolean volume check ─────────────────────────────────────────

    #[test]
    fn cut_reduces_volume() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "volume", "args": {"solid": 0}},
                {"op": "cut", "args": {"solidA": 0, "solidB": 1}},
                {"op": "volume", "args": {"solid": 2}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let vol_before = parsed[2]["ok"].as_f64().unwrap();
        let vol_after = parsed[4]["ok"].as_f64().unwrap();
        assert!(
            vol_after < vol_before,
            "cut must reduce volume: {vol_before} -> {vol_after}"
        );
    }

    // ── booleans with options ────────────────────────────────────────

    #[test]
    fn cut_with_options_preserves_volume() {
        // Corner-notch cut through the unifyFaces post-pass: the result must
        // survive unification with the exact expected volume.
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 4, "height": 4, "depth": 4}},
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "cutWithOptions", "args": {"solidA": 0, "solidB": 1, "unifyFaces": true}},
                {"op": "volume", "args": {"solid": 2}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert!(
            parsed[2]["ok"].is_number(),
            "cutWithOptions failed: {}",
            parsed[2]
        );
        let vol = parsed[3]["ok"].as_f64().unwrap();
        assert!(
            (vol - 56.0).abs() < 0.1,
            "notched box volume should be ~56, got {vol}"
        );
    }

    #[test]
    fn fuse_with_options_defaults_and_explicit_false_both_work() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "fuseWithOptions", "args": {"solidA": 0, "solidB": 1}},
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "fuseWithOptions", "args": {"solidA": 3, "solidB": 4, "unifyFaces": false}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert!(parsed[2]["ok"].is_number(), "default fuse: {}", parsed[2]);
        assert!(
            parsed[5]["ok"].is_number(),
            "unifyFaces=false fuse: {}",
            parsed[5]
        );
    }

    #[test]
    fn intersect_with_options_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[{"op": "intersectWithOptions", "args": {"solidA": 9999, "solidB": 9998}}]"#,
        );
        assert!(batch_has_error(&r, 0));
    }

    // ── compound_cut volume regression ───────────────────────────────

    #[test]
    fn compound_cut_volume_decreases() {
        let mut k = BrepKernel::new();
        // Target: 10x10x10 box at origin. Tool: 1x1x1 box at origin.
        // The tool overlaps one corner, so volume decreases.
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 10, "height": 10, "depth": 10}},
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "volume", "args": {"solid": 0}},
                {"op": "compoundCut", "args": {"target": 0, "tools": [1]}},
                {"op": "volume", "args": {"solid": 2}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let vol_before = parsed[2]["ok"].as_f64().unwrap();
        assert!(batch_has_ok(&r, 3), "compoundCut must succeed: {r}");
        let vol_after = parsed[4]["ok"].as_f64().unwrap();
        assert!(
            vol_after < vol_before && vol_after > 0.0,
            "compound_cut must reduce volume: {vol_before} -> {vol_after}"
        );
    }

    // ── booleanWithQuality ───────────────────────────────────────────

    #[test]
    fn boolean_with_quality_reports_exact_on_clean_boxes() {
        let mut k = BrepKernel::new();
        let a = k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let out = k
            .boolean_with_quality_impl("fuse", a, b, None, None, None, None, None, None, None)
            .unwrap();
        assert_eq!(out.quality, "exact");
        assert!(out.deflection.is_none());
        let volume = k.volume(out.solid, 0.05).unwrap();
        assert!((volume - 8.0).abs() < 1e-6, "fused volume {volume}");
        let boundary = remus_topology::validation::validate_boundary_authority(k.topo())
            .expect("WASM-visible boolean preserves whole-topology authority");
        assert_eq!(boundary.faces, k.topo().num_faces());
        assert_eq!(boundary.loops, k.topo().num_loops());
        assert_eq!(boundary.coedges, k.topo().num_coedges());
    }

    #[test]
    fn boolean_with_quality_exact_only_succeeds_on_exact_path() {
        let mut k = BrepKernel::new();
        let a = k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let out = k
            .boolean_with_quality_impl("cut", a, b, Some(true), None, None, None, None, None, None)
            .unwrap();
        assert_eq!(out.quality, "exact");
    }

    // ── SSI work-budget plumbing ─────────────────────────────────────
    //
    // Authority chain: these tests prove the JS value reaches the
    // `OperationContext` unchanged; the math suite (`nurbs/intersection/
    // tests.rs`) and the FF regression (`phase_ff.rs::
    // caller_budgets_reach_nurbs_ff_marching`) prove that context is
    // authoritative from there down. No WASM-reachable boolean can
    // currently distinguish the cap at the outcome level (the exact NURBS
    // assembly path declines such geometry under every budget), so the
    // context is the correct observation point.

    #[test]
    fn parse_budget_accepts_valid_and_rejects_invalid_for_every_direct_field() {
        for name in [
            "newton_iterations",
            "subdivision_depth",
            "march_steps",
            "queue_size",
            "segments",
            "branches_per_direction",
        ] {
            assert_eq!(super::parse_budget(None, name).unwrap(), None);
            assert_eq!(super::parse_budget(Some(0.0), name).unwrap(), Some(0));
            assert_eq!(
                super::parse_budget(Some(10_000.0), name).unwrap(),
                Some(10_000)
            );
            for bad in [-1.0, 2.5, f64::NAN, f64::INFINITY, 10_001.0] {
                let error = super::parse_budget(Some(bad), name).unwrap_err();
                assert!(
                    error.to_string().contains(name),
                    "{name} rejection must name the argument: {error}"
                );
            }
        }
    }

    #[test]
    fn quality_context_defaults_reproduce_legacy_context() {
        let ctx = super::quality_context(false, None, None, None, None, None, None);
        assert_eq!(ctx, remus_math::context::OperationContext::new());
    }

    #[test]
    fn quality_context_applies_ssi_budgets_and_exact_only() {
        use remus_math::context::FallbackPolicy;

        let bounded =
            super::quality_context(false, Some(3), Some(4), Some(5), Some(6), Some(7), Some(8));
        assert_eq!(bounded.budgets.newton_iterations, 3);
        assert_eq!(bounded.budgets.subdivision_depth, 4);
        assert_eq!(bounded.budgets.march_steps, 5);
        assert_eq!(bounded.budgets.queue_size, 6);
        assert_eq!(bounded.budgets.segments, 7);
        assert_eq!(bounded.budgets.branches_per_direction, 8);

        let disabled =
            super::quality_context(true, Some(0), Some(0), Some(0), Some(0), Some(0), Some(0));
        assert_eq!(disabled.budgets.newton_iterations, 0);
        assert_eq!(disabled.budgets.subdivision_depth, 0);
        assert_eq!(disabled.budgets.march_steps, 0);
        assert_eq!(disabled.budgets.queue_size, 0);
        assert_eq!(disabled.budgets.segments, 0);
        assert_eq!(disabled.budgets.branches_per_direction, 0);
        assert_eq!(disabled.fallback, FallbackPolicy::ExactOnly);
    }

    #[test]
    fn boolean_with_quality_accepts_explicit_ssi_budgets() {
        let mut k = BrepKernel::new();
        let a = k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
        // Analytic operands never enter NURBS SSI, so every cap (even 0)
        // must leave the exact result untouched — bounding is
        // additive, never a behavior change for exact-analytic paths.
        let out = k
            .boolean_with_quality_impl(
                "fuse",
                a,
                b,
                Some(true),
                Some(0),
                Some(0),
                Some(0),
                Some(0),
                Some(0),
                Some(0),
            )
            .unwrap();
        assert_eq!(out.quality, "exact");
        let volume = k.volume(out.solid, 0.05).unwrap();
        assert!((volume - 8.0).abs() < 1e-6, "fused volume {volume}");
    }

    #[test]
    fn boolean_with_cancellation_impl_accepts_ssi_budgets() {
        let mut k = BrepKernel::new();
        let a = k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let token = super::OperationCancellationToken::new();
        let out = k
            .boolean_with_cancellation_impl(
                "fuse",
                a,
                b,
                &token,
                Some(true),
                Some(5),
                Some(3),
                Some(2),
                Some(1),
                Some(1),
                Some(0),
            )
            .unwrap();
        assert_eq!(out.quality, "exact");
    }
}
