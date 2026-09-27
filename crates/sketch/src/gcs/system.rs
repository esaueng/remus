//! GCS system: CRUD operations, parameter management, and solve orchestration.

use std::collections::HashMap;

use crate::SketchError;

use super::constraint::{
    Constraint, ConstraintEntry, ConstraintId, EntitySnapshot, JacobianWriter, eval_jacobian,
    eval_residuals, residual_count,
};
use super::diagnostics::{ConstraintResidual, SolveDiagnostics, classify as classify_solve};
use super::dof::{self, DofAnalysis};
use super::entity::{
    ArcData, ArcId, CircleData, CircleId, GenArena, LineData, LineId, ParamRef, PointData, PointId,
};
use super::final_eval::FinalEvaluation;
use super::solver::{DoglegWorkspace, SolveResult, SolveStats};

/// The geometric constraint system.
///
/// Owns all entities (points, lines, circles) and constraints.
/// Provides CRUD operations and orchestrates the solver.
#[derive(Debug)]
pub struct GcsSystem {
    points: GenArena<PointData>,
    lines: GenArena<LineData>,
    circles: GenArena<CircleData>,
    arcs: GenArena<ArcData>,
    constraints: GenArena<ConstraintEntry>,
    /// Internal constraints auto-added by `add_arc` (center–end distance).
    /// Keyed by `ArcId` so they can be removed with the arc.
    arc_internal_constraints: HashMap<ArcId, ConstraintId>,
    /// Cached parameter map (rebuilt when dirty).
    param_map: Vec<ParamRef>,
    /// Map from `ParamRef` to index in param_map.
    param_index: HashMap<ParamRef, usize>,
    /// Whether the param map needs rebuilding.
    dirty: bool,
}

impl Clone for GcsSystem {
    fn clone(&self) -> Self {
        Self {
            points: self.points.clone(),
            lines: self.lines.clone(),
            circles: self.circles.clone(),
            arcs: self.arcs.clone(),
            constraints: self.constraints.clone(),
            arc_internal_constraints: self.arc_internal_constraints.clone(),
            param_map: self.param_map.clone(),
            param_index: self.param_index.clone(),
            dirty: self.dirty,
        }
    }
}

impl Default for GcsSystem {
    fn default() -> Self {
        Self::new()
    }
}

/// Evaluation counts for one `solve_detailed` call (PERF-S06).
///
/// Splits the call into solver-loop work ([`SolveStats`]) and post-solve
/// analysis work so a measurement can report where a residual, Jacobian or
/// QR evaluation happened. The shared path performs exactly one analysis
/// Jacobian plus one QR; the only residual re-evaluations it performs are
/// the fallback (identity mismatch or defensive length check, unreachable in
/// production) and the restored-state pass after a rollback (a genuinely
/// different state, so sharing would be wrong there).
///
/// Fields are read by the in-crate PERF-S06 tests, which do not compile into
/// the lib target — hence the allowance, mirroring `solve_dogleg`.
#[derive(Debug, Default, Clone, Copy)]
#[allow(dead_code)]
pub struct DetailedCounts {
    /// Solver-loop evaluations (residuals, Jacobians, factorizations).
    pub solver: SolveStats,
    /// Fresh Jacobian evaluations for the rank analysis (0 or 1).
    pub analysis_jacobian_evals: usize,
    /// Fresh QR factorizations for the rank analysis (0 or 1).
    pub analysis_qr_factorizations: usize,
    /// Residual re-evaluations the shared path declined (fresh fallback).
    pub fallback_residual_passes: usize,
    /// Residual evaluations at the restored state after a rollback.
    pub restored_state_passes: usize,
    /// Whether the per-constraint report came from the shared vector.
    pub shared_residuals_used: bool,
}

impl GcsSystem {
    /// Create a new empty GCS.
    #[must_use]
    pub fn new() -> Self {
        Self {
            points: GenArena::new(),
            lines: GenArena::new(),
            circles: GenArena::new(),
            arcs: GenArena::new(),
            constraints: GenArena::new(),
            arc_internal_constraints: HashMap::new(),
            param_map: Vec::new(),
            param_index: HashMap::new(),
            dirty: false,
        }
    }

    /// Add a point. Returns its handle.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::InvalidValue` if either coordinate is NaN or
    /// infinite. A poisoned coordinate is not caught downstream — every
    /// tolerance comparison against NaN is false — so it is rejected here,
    /// where the bad value is still attributable to the caller's input.
    ///
    /// Finite extreme magnitudes (e.g. `1e300`) are accepted: scale handling
    /// is the solver's job, and rejecting large-but-finite inputs here would
    /// invent a range contract no caller asked for.
    pub fn add_point(&mut self, data: PointData) -> Result<PointId, SketchError> {
        if !data.x.is_finite() || !data.y.is_finite() {
            return Err(SketchError::InvalidValue);
        }
        self.dirty = true;
        Ok(self.points.insert(data))
    }

    /// Get a point by handle.
    #[must_use]
    pub fn point(&self, id: PointId) -> Option<&PointData> {
        self.points.get(id)
    }

    /// Get a mutable reference to a point.
    pub fn point_mut(&mut self, id: PointId) -> Option<&mut PointData> {
        self.points.get_mut(id)
    }

    /// Remove a point. Fails if referenced by any line, circle, or constraint.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::EntityInUse` if the point is referenced by a line,
    /// circle, or constraint. Returns `SketchError::InvalidHandle` if the handle
    /// is stale or invalid.
    pub fn remove_point(&mut self, id: PointId) -> Result<PointData, SketchError> {
        for (_, line) in self.lines.iter() {
            if line.p1 == id || line.p2 == id {
                return Err(SketchError::EntityInUse);
            }
        }
        for (_, circle) in self.circles.iter() {
            if circle.center == id {
                return Err(SketchError::EntityInUse);
            }
        }
        for (_, arc) in self.arcs.iter() {
            if arc.center == id || arc.start == id || arc.end == id {
                return Err(SketchError::EntityInUse);
            }
        }
        for (_, entry) in self.constraints.iter() {
            if constraint_references_point(&entry.constraint, id) {
                return Err(SketchError::EntityInUse);
            }
        }
        self.dirty = true;
        self.points.remove(id).ok_or(SketchError::InvalidHandle)
    }

    /// Add a line between two existing points.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::InvalidHandle` if either point handle is invalid.
    pub fn add_line(&mut self, p1: PointId, p2: PointId) -> Result<LineId, SketchError> {
        if !self.points.contains(p1) || !self.points.contains(p2) {
            return Err(SketchError::InvalidHandle);
        }
        Ok(self.lines.insert(LineData { p1, p2 }))
    }

    /// Get a line by handle.
    #[must_use]
    pub fn line(&self, id: LineId) -> Option<&LineData> {
        self.lines.get(id)
    }

    /// Remove a line. Fails if referenced by any constraint.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::EntityInUse` if the line is referenced by a constraint.
    /// Returns `SketchError::InvalidHandle` if the handle is stale or invalid.
    pub fn remove_line(&mut self, id: LineId) -> Result<LineData, SketchError> {
        for (_, entry) in self.constraints.iter() {
            if constraint_references_line(&entry.constraint, id) {
                return Err(SketchError::EntityInUse);
            }
        }
        self.lines.remove(id).ok_or(SketchError::InvalidHandle)
    }

    /// Add a circle with a center point and radius.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::InvalidHandle` if the center point handle is invalid.
    /// Returns `SketchError::InvalidValue` if the radius is NaN, infinite,
    /// or non-positive.
    pub fn add_circle(&mut self, center: PointId, radius: f64) -> Result<CircleId, SketchError> {
        if !self.points.contains(center) {
            return Err(SketchError::InvalidHandle);
        }
        if !(radius.is_finite() && radius > 0.0) {
            return Err(SketchError::InvalidValue);
        }
        self.dirty = true;
        Ok(self.circles.insert(CircleData { center, radius }))
    }

    /// Get a circle by handle.
    #[must_use]
    pub fn circle(&self, id: CircleId) -> Option<&CircleData> {
        self.circles.get(id)
    }

    /// Remove a circle. Fails if referenced by any constraint.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::EntityInUse` if the circle is referenced by a constraint.
    /// Returns `SketchError::InvalidHandle` if the handle is stale or invalid.
    pub fn remove_circle(&mut self, id: CircleId) -> Result<CircleData, SketchError> {
        for (_, entry) in self.constraints.iter() {
            if constraint_references_circle(&entry.constraint, id) {
                return Err(SketchError::EntityInUse);
            }
        }
        self.dirty = true;
        self.circles.remove(id).ok_or(SketchError::InvalidHandle)
    }

    /// Add an arc defined by center, start, and end points.
    ///
    /// Auto-adds an internal `PointOnArc(end, arc)` constraint so that
    /// `dist(center, end) == dist(center, start)` is maintained dynamically
    /// as the start point moves.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::InvalidHandle` if any point handle is invalid.
    pub fn add_arc(
        &mut self,
        center: PointId,
        start: PointId,
        end: PointId,
    ) -> Result<ArcId, SketchError> {
        self.check_point(center)?;
        self.check_point(start)?;
        self.check_point(end)?;

        let arc_id = self.arcs.insert(ArcData { center, start, end });

        // Internal constraint: end point must lie on the arc's circle
        // (dynamically tracks dist(center, start) rather than a frozen radius)
        let cid = self.constraints.insert(ConstraintEntry {
            constraint: Constraint::PointOnArc(end, arc_id),
        });
        self.arc_internal_constraints.insert(arc_id, cid);

        self.dirty = true;
        Ok(arc_id)
    }

    /// Get an arc by handle.
    #[must_use]
    pub fn arc(&self, id: ArcId) -> Option<&ArcData> {
        self.arcs.get(id)
    }

    /// Get a mutable reference to an arc.
    pub fn arc_mut(&mut self, id: ArcId) -> Option<&mut ArcData> {
        self.arcs.get_mut(id)
    }

    /// Remove an arc. Fails if referenced by any user constraint.
    ///
    /// Also removes the internal distance constraint that was auto-added
    /// by [`add_arc`](Self::add_arc).
    ///
    /// # Errors
    ///
    /// Returns `SketchError::EntityInUse` if the arc is referenced by a constraint.
    /// Returns `SketchError::InvalidHandle` if the handle is stale or invalid.
    pub fn remove_arc(&mut self, id: ArcId) -> Result<ArcData, SketchError> {
        for (cid, entry) in self.constraints.iter() {
            if self.arc_internal_constraints.get(&id) == Some(&cid) {
                continue;
            }
            if constraint_references_arc(&entry.constraint, id) {
                return Err(SketchError::EntityInUse);
            }
        }

        if let Some(cid) = self.arc_internal_constraints.remove(&id) {
            self.constraints.remove(cid);
        }

        self.dirty = true;
        self.arcs.remove(id).ok_or(SketchError::InvalidHandle)
    }

    /// Number of arcs.
    #[must_use]
    pub fn arc_count(&self) -> usize {
        self.arcs.len()
    }

    /// Iterate over all arcs.
    pub fn arcs(&self) -> impl Iterator<Item = (ArcId, &ArcData)> {
        self.arcs.iter()
    }

    /// Add a constraint. Validates that all referenced entities exist and
    /// that every numeric argument is finite (and positive where a positive
    /// magnitude is required).
    ///
    /// # Errors
    ///
    /// Returns `SketchError::InvalidHandle` if any entity referenced by the
    /// constraint does not exist. Returns `SketchError::InvalidValue` if a
    /// numeric argument is NaN, infinite, or outside its permitted range.
    pub fn add_constraint(&mut self, constraint: Constraint) -> Result<ConstraintId, SketchError> {
        self.validate_constraint(&constraint)?;
        self.dirty = true;
        Ok(self.constraints.insert(ConstraintEntry { constraint }))
    }

    /// Remove a constraint by handle.
    ///
    /// # Errors
    ///
    /// Returns `SketchError::InvalidHandle` if the handle is stale or invalid.
    pub fn remove_constraint(&mut self, id: ConstraintId) -> Result<(), SketchError> {
        self.constraints
            .remove(id)
            .map(|_| {
                self.dirty = true;
            })
            .ok_or(SketchError::InvalidHandle)
    }

    /// Get a constraint by handle.
    #[must_use]
    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint> {
        self.constraints.get(id).map(|e| &e.constraint)
    }

    /// Number of constraints (includes internal arc constraints).
    #[must_use]
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    /// Number of points.
    #[must_use]
    pub fn point_count(&self) -> usize {
        self.points.len()
    }

    /// Number of lines.
    #[must_use]
    pub fn line_count(&self) -> usize {
        self.lines.len()
    }

    /// Number of circles.
    #[must_use]
    pub fn circle_count(&self) -> usize {
        self.circles.len()
    }

    /// Solve the constraint system.
    ///
    /// Modifies entity positions in-place to satisfy all constraints.
    ///
    /// # Errors
    ///
    /// Returns `SketchError` if the system parameters are in an invalid state.
    /// The `Result` wrapper is retained for future error paths (e.g. singular
    /// Jacobian detection).
    #[allow(clippy::unnecessary_wraps)]
    pub fn solve(
        &mut self,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<SolveResult, SketchError> {
        let (result, _eval, _stats) = self.solve_impl(max_iterations, tolerance)?;
        Ok(result)
    }

    /// Solve, capturing the final-iterate evaluation and loop counts.
    ///
    /// Behavior matches [`solve`](Self::solve) exactly (same params written,
    /// same result); the capture lets [`solve_detailed`](Self::solve_detailed)
    /// reuse the final evaluation instead of re-measuring it. Nothing is
    /// retained on the system: the capture is returned to the caller and
    /// dropped unless consumed by the same diagnostics call.
    ///
    /// The `Result` wrapper is retained for future error paths, matching
    /// [`solve`](Self::solve).
    #[allow(clippy::unnecessary_wraps)]
    fn solve_impl(
        &mut self,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<(SolveResult, FinalEvaluation, SolveStats), SketchError> {
        self.rebuild_if_dirty();

        let n = self.param_map.len();
        let m: usize = self
            .constraints
            .iter()
            .map(|(_, e)| residual_count(&e.constraint))
            .sum();

        if n == 0 {
            // No free params — just check residuals. NaN propagates via
            // `max_abs_residual` so a poisoned value can never read as
            // converged (see the solver's fold for why a plain max is wrong).
            let snap = self.build_snapshot();
            let mut residuals = Vec::with_capacity(m);
            for (_, entry) in self.constraints.iter() {
                eval_residuals(&entry.constraint, &snap, &mut residuals);
            }
            let max_r = max_abs_residual(&residuals);
            let result = SolveResult {
                converged: max_r < tolerance,
                iterations: 0,
                max_residual: max_r,
            };
            let eval = FinalEvaluation::capture(&[], &residuals, self.constraint_rows());
            // One full residual evaluation happened above (counted for
            // PERF-S06 even though the solver loop never ran).
            let stats = SolveStats {
                residual_evals: 1,
                ..SolveStats::default()
            };
            return Ok((result, eval, stats));
        }

        let mut params = self.extract_params();
        let param_index = self.param_index.clone();

        let constraints: Vec<Constraint> = self
            .constraints
            .iter()
            .map(|(_, e)| e.constraint.clone())
            .collect();

        // Solve-local snapshot storage (PERF-S03): cleared and refilled on
        // every residual/Jacobian evaluation instead of reallocated.
        // Separate stores for the residual and Jacobian paths so the two
        // fill closures own disjoint scratch. Nothing is retained across
        // solves; capacities are per-solve only.
        let mut snap_r = EntitySnapshot {
            points: HashMap::with_capacity(self.points.len()),
            lines: HashMap::with_capacity(self.lines.len()),
            circles: HashMap::with_capacity(self.circles.len()),
            arcs: HashMap::with_capacity(self.arcs.len()),
        };
        let mut snap_j = EntitySnapshot {
            points: HashMap::with_capacity(self.points.len()),
            lines: HashMap::with_capacity(self.lines.len()),
            circles: HashMap::with_capacity(self.circles.len()),
            arcs: HashMap::with_capacity(self.arcs.len()),
        };

        let mut residual_fill = |p: &[f64], out: &mut Vec<f64>| {
            refresh_snapshot_from_params(&mut snap_r, p, &param_index, self);
            out.clear();
            for c in &constraints {
                eval_residuals(c, &snap_r, out);
            }
        };

        let mut jacobian_fill = |p: &[f64], out: &mut [f64]| {
            refresh_snapshot_from_params(&mut snap_j, p, &param_index, self);
            out.fill(0.0);
            let mut row = 0;
            {
                let mut jw = JacobianWriter {
                    data: out,
                    ncols: n,
                    param_index: &param_index,
                };
                for c in &constraints {
                    eval_jacobian(c, &snap_j, &mut jw, row);
                    row += residual_count(c);
                }
            }
        };

        let mut workspace = DoglegWorkspace::new();
        let mut stats = SolveStats::default();
        let result = super::solver::solve_dogleg_fill(
            &mut params,
            &mut residual_fill,
            &mut jacobian_fill,
            m,
            max_iterations,
            tolerance,
            &mut workspace,
            &mut stats,
        );

        self.write_params(&params);

        let eval =
            FinalEvaluation::capture(&params, workspace.final_residuals(), self.constraint_rows());
        Ok((result, eval, stats))
    }

    /// `(constraint id, residual row count)` in evaluation (arena) order.
    ///
    /// Part of the [`FinalEvaluation`] identity: any add/remove between
    /// capture and reuse changes this layout and forces fresh evaluation.
    fn constraint_rows(&self) -> Vec<(ConstraintId, usize)> {
        self.constraints
            .iter()
            .map(|(cid, e)| (cid, residual_count(&e.constraint)))
            .collect()
    }

    /// Solve, then report everything the attempt established — transactionally.
    ///
    /// This is [`solve`](Self::solve) plus measurement, with one behavioural
    /// difference: **an attempt that does not converge is rolled back**. The
    /// pre-solve coordinates and radii are restored, so a rejected solve never
    /// leaves half-moved geometry published. A converged solve publishes its
    /// result exactly as [`solve`](Self::solve) does.
    ///
    /// [`solve`](Self::solve) itself is unchanged and still publishes whatever
    /// iterate it finished on; callers depending on that behaviour keep it.
    ///
    /// Residuals are reported per constraint at the solver's final iterate —
    /// its best attempt, which is where an unsatisfiable constraint stands
    /// out from the ones the system could satisfy. Kernel-internal
    /// constraints are flagged as such rather than being attributed to a
    /// caller's constraint. See [`SolveDiagnostics`].
    ///
    /// Measurement reuses the solver's own final evaluation (PERF-S06): the
    /// per-constraint residuals are derived from the final residual vector
    /// after verifying it still describes the live system, and a single
    /// snapshot backs the fresh Jacobian the rank analysis needs. No
    /// factorization is ever reused — an accepted final step can invalidate
    /// the loop's previous one — and any identity mismatch falls back to
    /// fully fresh evaluation with identical results.
    ///
    /// # Errors
    ///
    /// Propagates any error from [`solve`](Self::solve).
    pub fn solve_detailed(
        &mut self,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<SolveDiagnostics, SketchError> {
        let (diagnostics, _counts) = self.solve_detailed_counted(max_iterations, tolerance)?;
        Ok(diagnostics)
    }

    /// [`solve_detailed`](Self::solve_detailed) plus evaluation counts.
    ///
    /// `solve_detailed` discards the counts; tests and the PERF-S06 report
    /// use them to tell solver-loop work apart from post-solve analysis.
    ///
    /// # Errors
    ///
    /// Propagates any error from [`solve`](Self::solve).
    pub fn solve_detailed_counted(
        &mut self,
        max_iterations: usize,
        tolerance: f64,
    ) -> Result<(SolveDiagnostics, DetailedCounts), SketchError> {
        self.rebuild_if_dirty();

        // Snapshot for rollback before anything is mutated.
        let before = self.extract_params();

        let (result, eval, stats) = self.solve_impl(max_iterations, tolerance)?;
        let mut counts = DetailedCounts {
            solver: stats,
            ..DetailedCounts::default()
        };

        // Measure at the solver's final iterate, *before* any rollback. This
        // is the informative state: constraints the system can satisfy have
        // driven their residual to ~0, so whatever residual remains marks
        // where the system could not reconcile. Measuring after a rollback
        // would instead report the untouched starting geometry, where every
        // constraint — satisfiable or not — still reads large.
        let n = self.param_map.len();
        let m: usize = self
            .constraints
            .iter()
            .map(|(_, e)| residual_count(&e.constraint))
            .sum();

        let (analysis, residuals, internal_max_residual) = if n == 0 || m == 0 {
            // Degenerate dimensions have no Jacobian to share: measure
            // everything fresh, exactly as before.
            counts.fallback_residual_passes += 1;
            let analysis = self.dof();
            let (residuals, internal_max) = self.constraint_residuals();
            (analysis, residuals, internal_max)
        } else if self.eval_matches(&eval, m, n) {
            // Shared path: one snapshot backs the fresh Jacobian the rank
            // needs; per-constraint residuals slice the verified final
            // vector instead of re-evaluating every constraint.
            let snap = self.build_snapshot();
            let jac = self.jacobian_for_snapshot(&snap, m, n);
            counts.analysis_jacobian_evals += 1;
            let analysis = dof::analyze(&jac, m, n);
            counts.analysis_qr_factorizations += 1;
            if let Some((residuals, internal_max)) = self.derive_shared_residuals(&eval) {
                counts.shared_residuals_used = true;
                (analysis, residuals, internal_max)
            } else {
                // Defensive length mismatch: same fresh fallback.
                // Unreachable — the solver emits exactly one entry per
                // equation — but a silent wrong slice is worse than a
                // repeated evaluation.
                counts.fallback_residual_passes += 1;
                let (residuals, internal_max) = self.constraint_residuals();
                (analysis, residuals, internal_max)
            }
        } else {
            // Identity mismatch means the capture is not this state: measure
            // everything fresh (identical results, one extra residual pass).
            // Unreachable in production — capture and measurement bracket no
            // mutation — but proven by tests.
            counts.fallback_residual_passes += 1;
            let analysis = self.dof();
            let (residuals, internal_max) = self.constraint_residuals();
            (analysis, residuals, internal_max)
        };

        let rolled_back = !result.converged;
        let published_max_residual = if rolled_back {
            self.write_params(&before);
            // Restored geometry is a different state from the attempt, so it
            // is always freshly measured — never shared. Only the fold max is
            // needed (the attempt's per-constraint report is already filed),
            // which skips the discarded report allocation with identical value.
            counts.restored_state_passes += 1;
            self.current_max_residual()
        } else {
            fold_max_residual(&residuals)
        };

        let redundant = analysis.rank < analysis.num_equations;

        Ok((
            SolveDiagnostics {
                converged: result.converged,
                iterations: result.iterations,
                max_residual: result.max_residual,
                published_max_residual,
                dof: analysis.dof,
                rank: analysis.rank,
                num_params: analysis.num_params,
                num_equations: analysis.num_equations,
                residuals,
                internal_max_residual,
                rolled_back,
                redundant,
                classification: classify_solve(
                    result.converged,
                    analysis.dof,
                    analysis.rank,
                    analysis.num_equations,
                ),
            },
            counts,
        ))
    }

    /// Whether a solve-local capture still describes the live system.
    fn eval_matches(&self, eval: &FinalEvaluation, m: usize, n: usize) -> bool {
        eval.matches(&self.extract_params(), &self.constraint_rows(), m, n)
    }

    /// Derive the per-constraint residual report from a verified capture.
    ///
    /// Slices the shared flat residual vector by the capture's row layout
    /// and applies the same NaN-propagating fold and internal-flag rules as
    /// [`constraint_residuals`](Self::constraint_residuals). Returns `None`
    /// when the flat vector does not match the row layout (defensive only:
    /// the solver always emits exactly one entry per equation); the caller
    /// then measures fresh. Call only after [`eval_matches`](Self::eval_matches).
    fn derive_shared_residuals(
        &self,
        eval: &FinalEvaluation,
    ) -> Option<(Vec<ConstraintResidual>, f64)> {
        let flat = eval.residuals();
        let rows = eval.rows();
        let total: usize = rows.iter().map(|(_, count)| *count).sum();
        if flat.len() != total {
            return None;
        }
        let internal: std::collections::HashSet<ConstraintId> =
            self.arc_internal_constraints.values().copied().collect();

        let mut out = Vec::with_capacity(rows.len());
        let mut internal_max = 0.0_f64;
        let mut offset = 0_usize;
        for (cid, count) in rows {
            let slice = flat.get(offset..offset.saturating_add(*count))?;
            if slice.len() != *count {
                return None;
            }
            offset += *count;
            let max_abs = max_abs_residual(slice);
            let is_internal = internal.contains(cid);
            if is_internal && (max_abs.is_nan() || max_abs > internal_max) {
                internal_max = max_abs;
            }
            out.push(ConstraintResidual {
                constraint: *cid,
                max_abs_residual: max_abs,
                internal: is_internal,
            });
        }
        Some((out, internal_max))
    }

    /// Whether a constraint was created by the kernel rather than the caller.
    ///
    /// [`add_arc`](Self::add_arc) installs an internal constraint tying the
    /// arc's end point to its start radius. It has no caller-facing handle, so
    /// diagnostics must not attribute its residual to a user constraint.
    #[must_use]
    pub fn is_internal_constraint(&self, id: ConstraintId) -> bool {
        self.arc_internal_constraints.values().any(|&c| c == id)
    }

    /// Residual magnitude of every live constraint at the current state,
    /// plus the largest magnitude over internal constraints alone.
    ///
    /// Order follows the constraint arena's slot order, which is stable for a
    /// given sequence of add/remove calls.
    ///
    /// Fresh evaluation, never shared: the PERF-S06 tests use this as the
    /// independent oracle the shared diagnostics path is compared against.
    #[must_use]
    pub fn constraint_residuals(&self) -> (Vec<ConstraintResidual>, f64) {
        let snap = self.build_snapshot();
        let internal: std::collections::HashSet<ConstraintId> =
            self.arc_internal_constraints.values().copied().collect();

        let mut out = Vec::with_capacity(self.constraints.len());
        let mut internal_max = 0.0_f64;
        let mut buf = Vec::new();

        for (cid, entry) in self.constraints.iter() {
            buf.clear();
            eval_residuals(&entry.constraint, &snap, &mut buf);
            let max_abs = max_abs_residual(&buf);
            let is_internal = internal.contains(&cid);
            if is_internal && (max_abs.is_nan() || max_abs > internal_max) {
                internal_max = max_abs;
            }
            out.push(ConstraintResidual {
                constraint: cid,
                max_abs_residual: max_abs,
                internal: is_internal,
            });
        }

        (out, internal_max)
    }

    /// Analyze degrees of freedom in the current system.
    pub fn dof(&mut self) -> DofAnalysis {
        self.rebuild_if_dirty();

        let n = self.param_map.len();
        let m: usize = self
            .constraints
            .iter()
            .map(|(_, e)| residual_count(&e.constraint))
            .sum();

        if n == 0 || m == 0 {
            return DofAnalysis {
                dof: n,
                rank: 0,
                num_params: n,
                num_equations: m,
            };
        }

        let snap = self.build_snapshot();
        let jac = self.jacobian_for_snapshot(&snap, m, n);
        dof::analyze(&jac, m, n)
    }

    /// Row-major Jacobian at the entities held by `snap`.
    ///
    /// Shared by the fresh [`dof`](Self::dof) path and the PERF-S06 fused
    /// diagnostics path so both factorize the same matrix. Neither path
    /// reuses a factorization: the caller factorizes the returned matrix at
    /// the state it was evaluated at.
    fn jacobian_for_snapshot(&self, snap: &EntitySnapshot, m: usize, n: usize) -> Vec<f64> {
        let mut jac = vec![0.0; m.saturating_mul(n)];
        let mut row = 0;
        {
            let mut jw = JacobianWriter {
                data: &mut jac,
                ncols: n,
                param_index: &self.param_index,
            };
            for (_, entry) in self.constraints.iter() {
                eval_jacobian(&entry.constraint, snap, &mut jw, row);
                row += residual_count(&entry.constraint);
            }
        }
        jac
    }

    /// Largest absolute residual over all equations at the current state.
    ///
    /// Same value as folding [`constraint_residuals`](Self::constraint_residuals)'s
    /// report (max over the union, NaN-propagating both ways), without
    /// building the per-constraint report. Used for `published_max_residual`
    /// after a rollback, where only the fold max is reported.
    fn current_max_residual(&self) -> f64 {
        let snap = self.build_snapshot();
        let mut residuals = Vec::new();
        for (_, entry) in self.constraints.iter() {
            eval_residuals(&entry.constraint, &snap, &mut residuals);
        }
        max_abs_residual(&residuals)
    }

    /// Iterate over all points.
    pub fn points(&self) -> impl Iterator<Item = (PointId, &PointData)> {
        self.points.iter()
    }

    /// Iterate over all lines.
    pub fn lines(&self) -> impl Iterator<Item = (LineId, &LineData)> {
        self.lines.iter()
    }

    /// Iterate over all circles.
    pub fn circles(&self) -> impl Iterator<Item = (CircleId, &CircleData)> {
        self.circles.iter()
    }

    /// Rebuild parameter map if dirty.
    ///
    /// Visible within the crate for the PERF-S06 oracle tests, which must snapshot
    /// pre-solve parameters in the same rebuilt state production measures.
    pub fn rebuild_if_dirty(&mut self) {
        if !self.dirty {
            return;
        }
        self.param_map.clear();
        self.param_index.clear();

        for (id, data) in self.points.iter() {
            if !data.fixed {
                let idx = self.param_map.len();
                self.param_map.push(ParamRef::PointX(id));
                self.param_index.insert(ParamRef::PointX(id), idx);
                let idx = self.param_map.len();
                self.param_map.push(ParamRef::PointY(id));
                self.param_index.insert(ParamRef::PointY(id), idx);
            }
        }

        for (id, _) in self.circles.iter() {
            let idx = self.param_map.len();
            self.param_map.push(ParamRef::CircleRadius(id));
            self.param_index.insert(ParamRef::CircleRadius(id), idx);
        }

        self.dirty = false;
    }

    /// Extract parameter values from entities.
    ///
    /// Visible within the crate for the PERF-S06 oracle tests, which snapshot geometry
    /// through the same path the rollback uses.
    #[must_use]
    pub fn extract_params(&self) -> Vec<f64> {
        self.param_map
            .iter()
            .map(|pr| match pr {
                ParamRef::PointX(id) => self.points.get(*id).map_or(0.0, |p| p.x),
                ParamRef::PointY(id) => self.points.get(*id).map_or(0.0, |p| p.y),
                ParamRef::CircleRadius(id) => self.circles.get(*id).map_or(0.0, |c| c.radius),
            })
            .collect()
    }

    /// Write parameter values back to entities.
    ///
    /// Visible within the crate for the PERF-S06 oracle tests, which roll back through
    /// the same path production uses (the independence under test is the
    /// fresh residual/Jacobian evaluation, not the param write).
    pub fn write_params(&mut self, params: &[f64]) {
        for (i, pr) in self.param_map.iter().enumerate() {
            match pr {
                ParamRef::PointX(id) => {
                    if let Some(p) = self.points.get_mut(*id) {
                        p.x = params[i];
                    }
                }
                ParamRef::PointY(id) => {
                    if let Some(p) = self.points.get_mut(*id) {
                        p.y = params[i];
                    }
                }
                ParamRef::CircleRadius(id) => {
                    if let Some(c) = self.circles.get_mut(*id) {
                        c.radius = params[i];
                    }
                }
            }
        }
    }

    /// Build an entity snapshot for residual/Jacobian evaluation.
    fn build_snapshot(&self) -> EntitySnapshot {
        EntitySnapshot {
            points: self.points.iter().map(|(id, d)| (id, (d.x, d.y))).collect(),
            lines: self
                .lines
                .iter()
                .map(|(id, d)| (id, (d.p1, d.p2)))
                .collect(),
            circles: self
                .circles
                .iter()
                .map(|(id, d)| (id, (d.center, d.radius)))
                .collect(),
            arcs: self
                .arcs
                .iter()
                .map(|(id, d)| (id, (d.center, d.start, d.end)))
                .collect(),
        }
    }

    /// Validate all entity references in a constraint.
    fn validate_constraint(&self, c: &Constraint) -> Result<(), SketchError> {
        // Every `_` arm below is a finite check on a caller-supplied scalar:
        // NaN/infinite targets are not caught downstream (a comparison
        // against NaN is always false), so they are rejected here, where the
        // bad value is still attributable to the caller's input. Only
        // `CircleRadius` additionally requires positivity.
        match c {
            Constraint::Coincident(p1, p2) => {
                self.check_point(*p1)?;
                self.check_point(*p2)?;
            }
            Constraint::Distance(p1, p2, d) => {
                self.check_point(*p1)?;
                self.check_point(*p2)?;
                if !d.is_finite() {
                    return Err(SketchError::InvalidValue);
                }
            }
            Constraint::PointLineDistance(pt, line, d) => {
                self.check_point(*pt)?;
                self.check_line(*line)?;
                if !d.is_finite() {
                    return Err(SketchError::InvalidValue);
                }
            }
            Constraint::FixX(p, v) | Constraint::FixY(p, v) => {
                self.check_point(*p)?;
                if !v.is_finite() {
                    return Err(SketchError::InvalidValue);
                }
            }
            Constraint::Horizontal(line) | Constraint::Vertical(line) => {
                self.check_line(*line)?;
            }
            Constraint::Angle(l1, l2, theta) => {
                self.check_line(*l1)?;
                self.check_line(*l2)?;
                if !theta.is_finite() {
                    return Err(SketchError::InvalidValue);
                }
            }
            Constraint::Perpendicular(l1, l2) | Constraint::Parallel(l1, l2) => {
                self.check_line(*l1)?;
                self.check_line(*l2)?;
            }
            Constraint::PointOnCircle(pt, circ) => {
                self.check_point(*pt)?;
                self.check_circle(*circ)?;
            }
            Constraint::PointOnArc(pt, arc) => {
                self.check_point(*pt)?;
                self.check_arc(*arc)?;
            }
            Constraint::TangentLineArc(line, arc, shared) => {
                self.check_line(*line)?;
                self.check_arc(*arc)?;
                self.check_point(*shared)?;
            }
            Constraint::TangentArcArc(arc1, arc2, shared) => {
                self.check_arc(*arc1)?;
                self.check_arc(*arc2)?;
                self.check_point(*shared)?;
            }
            Constraint::EqualRadiusArcArc(arc1, arc2) => {
                self.check_arc(*arc1)?;
                self.check_arc(*arc2)?;
            }
            Constraint::EqualRadiusArcCircle(arc, circ) => {
                self.check_arc(*arc)?;
                self.check_circle(*circ)?;
            }
            Constraint::ArcLength(arc, target) => {
                self.check_arc(*arc)?;
                if !target.is_finite() {
                    return Err(SketchError::InvalidValue);
                }
            }
            Constraint::ConcentricArcArc(arc1, arc2) => {
                self.check_arc(*arc1)?;
                self.check_arc(*arc2)?;
            }
            Constraint::ConcentricArcCircle(arc, circ) => {
                self.check_arc(*arc)?;
                self.check_circle(*circ)?;
            }
            Constraint::CircleRadius(circ, value) => {
                self.check_circle(*circ)?;
                if !(value.is_finite() && *value > 0.0) {
                    return Err(SketchError::InvalidValue);
                }
            }
            Constraint::EqualRadiusCircleCircle(c1, c2) => {
                self.check_circle(*c1)?;
                self.check_circle(*c2)?;
            }
            Constraint::EqualLength(l1, l2) => {
                self.check_line(*l1)?;
                self.check_line(*l2)?;
            }
            Constraint::Midpoint(pt, line) => {
                self.check_point(*pt)?;
                self.check_line(*line)?;
            }
            Constraint::Symmetric(p1, p2, axis) => {
                self.check_point(*p1)?;
                self.check_point(*p2)?;
                self.check_line(*axis)?;
            }
            Constraint::TangentLineCircle(line, circ) => {
                self.check_line(*line)?;
                self.check_circle(*circ)?;
            }
            Constraint::SymmetricAboutPoint(p1, p2, center) => {
                self.check_point(*p1)?;
                self.check_point(*p2)?;
                self.check_point(*center)?;
            }
        }
        Ok(())
    }

    fn check_point(&self, id: PointId) -> Result<(), SketchError> {
        if self.points.contains(id) {
            Ok(())
        } else {
            Err(SketchError::InvalidHandle)
        }
    }

    fn check_line(&self, id: LineId) -> Result<(), SketchError> {
        if self.lines.contains(id) {
            Ok(())
        } else {
            Err(SketchError::InvalidHandle)
        }
    }

    fn check_circle(&self, id: CircleId) -> Result<(), SketchError> {
        if self.circles.contains(id) {
            Ok(())
        } else {
            Err(SketchError::InvalidHandle)
        }
    }

    fn check_arc(&self, id: ArcId) -> Result<(), SketchError> {
        if self.arcs.contains(id) {
            Ok(())
        } else {
            Err(SketchError::InvalidHandle)
        }
    }
}

/// Largest per-constraint residual in a report, propagating NaN.
pub fn fold_max_residual(residuals: &[ConstraintResidual]) -> f64 {
    let mut max = 0.0_f64;
    for r in residuals {
        if r.max_abs_residual.is_nan() {
            return f64::NAN;
        }
        if r.max_abs_residual > max {
            max = r.max_abs_residual;
        }
    }
    max
}

/// Largest absolute value in `values`, propagating NaN rather than dropping it.
///
/// `f64::max` returns the non-NaN operand, which would silently turn a poisoned
/// residual (from a stale handle) into a clean zero. Diagnostics must not lie
/// about that, so NaN short-circuits.
pub fn max_abs_residual(values: &[f64]) -> f64 {
    let mut max = 0.0_f64;
    for &v in values {
        let a = v.abs();
        if a.is_nan() {
            return f64::NAN;
        }
        if a > max {
            max = a;
        }
    }
    max
}

/// Refresh solve-local snapshot storage from parameter values (PERF-S03).
///
/// Values and arena iteration order match the previous per-evaluation
/// snapshot build; the four maps are `clear`ed (capacity retained) and
/// refilled instead of reallocated. No topology change occurs during a solve, so capacities
/// stabilize after the first evaluation and later evaluations allocate
/// nothing. `clear` removes every entry, so a resized later solve cannot
/// observe a previous solve's leftovers (workspaces are solve-local anyway).
fn refresh_snapshot_from_params(
    snap: &mut EntitySnapshot,
    params: &[f64],
    param_index: &HashMap<ParamRef, usize>,
    sys: &GcsSystem,
) {
    snap.points.clear();
    for (id, data) in sys.points.iter() {
        let x = param_index
            .get(&ParamRef::PointX(id))
            .map_or(data.x, |&i| params[i]);
        let y = param_index
            .get(&ParamRef::PointY(id))
            .map_or(data.y, |&i| params[i]);
        snap.points.insert(id, (x, y));
    }

    snap.lines.clear();
    for (id, d) in sys.lines.iter() {
        snap.lines.insert(id, (d.p1, d.p2));
    }

    snap.circles.clear();
    for (id, data) in sys.circles.iter() {
        let r = param_index
            .get(&ParamRef::CircleRadius(id))
            .map_or(data.radius, |&i| params[i]);
        snap.circles.insert(id, (data.center, r));
    }

    snap.arcs.clear();
    for (id, d) in sys.arcs.iter() {
        snap.arcs.insert(id, (d.center, d.start, d.end));
    }
}

/// Check if a constraint references a specific point.
fn constraint_references_point(c: &Constraint, id: PointId) -> bool {
    match c {
        Constraint::Coincident(p1, p2) | Constraint::Distance(p1, p2, _) => *p1 == id || *p2 == id,
        Constraint::PointLineDistance(pt, _, _)
        | Constraint::PointOnCircle(pt, _)
        | Constraint::PointOnArc(pt, _) => *pt == id,
        Constraint::FixX(p, _) | Constraint::FixY(p, _) => *p == id,
        Constraint::TangentLineArc(_, _, shared) | Constraint::TangentArcArc(_, _, shared) => {
            *shared == id
        }
        Constraint::Midpoint(pt, _) => *pt == id,
        Constraint::Symmetric(p1, p2, _) => *p1 == id || *p2 == id,
        Constraint::SymmetricAboutPoint(p1, p2, center) => *p1 == id || *p2 == id || *center == id,
        Constraint::Horizontal(_)
        | Constraint::Vertical(_)
        | Constraint::Angle(_, _, _)
        | Constraint::Perpendicular(_, _)
        | Constraint::Parallel(_, _)
        | Constraint::EqualRadiusArcArc(_, _)
        | Constraint::EqualRadiusArcCircle(_, _)
        | Constraint::ArcLength(_, _)
        | Constraint::ConcentricArcArc(_, _)
        | Constraint::ConcentricArcCircle(_, _)
        | Constraint::CircleRadius(_, _)
        | Constraint::EqualRadiusCircleCircle(_, _)
        | Constraint::EqualLength(_, _)
        | Constraint::TangentLineCircle(_, _) => false,
    }
}

/// Check if a constraint references a specific line.
fn constraint_references_line(c: &Constraint, id: LineId) -> bool {
    match c {
        Constraint::Horizontal(l) | Constraint::Vertical(l) => *l == id,
        Constraint::PointLineDistance(_, l, _) => *l == id,
        Constraint::TangentLineArc(l, _, _) | Constraint::TangentLineCircle(l, _) => *l == id,
        Constraint::Midpoint(_, l) | Constraint::Symmetric(_, _, l) => *l == id,
        Constraint::Angle(l1, l2, _)
        | Constraint::Perpendicular(l1, l2)
        | Constraint::Parallel(l1, l2)
        | Constraint::EqualLength(l1, l2) => *l1 == id || *l2 == id,
        Constraint::Coincident(_, _)
        | Constraint::Distance(_, _, _)
        | Constraint::FixX(_, _)
        | Constraint::FixY(_, _)
        | Constraint::PointOnCircle(_, _)
        | Constraint::PointOnArc(_, _)
        | Constraint::TangentArcArc(_, _, _)
        | Constraint::EqualRadiusArcArc(_, _)
        | Constraint::EqualRadiusArcCircle(_, _)
        | Constraint::ArcLength(_, _)
        | Constraint::ConcentricArcArc(_, _)
        | Constraint::ConcentricArcCircle(_, _)
        | Constraint::CircleRadius(_, _)
        | Constraint::EqualRadiusCircleCircle(_, _)
        | Constraint::SymmetricAboutPoint(_, _, _) => false,
    }
}

/// Check if a constraint references a specific circle.
fn constraint_references_circle(c: &Constraint, id: CircleId) -> bool {
    match c {
        Constraint::PointOnCircle(_, circ) | Constraint::TangentLineCircle(_, circ) => *circ == id,
        Constraint::EqualRadiusArcCircle(_, circ) | Constraint::ConcentricArcCircle(_, circ) => {
            *circ == id
        }
        Constraint::CircleRadius(circ, _) => *circ == id,
        Constraint::EqualRadiusCircleCircle(c1, c2) => *c1 == id || *c2 == id,
        Constraint::Coincident(_, _)
        | Constraint::Distance(_, _, _)
        | Constraint::PointLineDistance(_, _, _)
        | Constraint::FixX(_, _)
        | Constraint::FixY(_, _)
        | Constraint::Horizontal(_)
        | Constraint::Vertical(_)
        | Constraint::Angle(_, _, _)
        | Constraint::Perpendicular(_, _)
        | Constraint::Parallel(_, _)
        | Constraint::PointOnArc(_, _)
        | Constraint::TangentLineArc(_, _, _)
        | Constraint::TangentArcArc(_, _, _)
        | Constraint::EqualRadiusArcArc(_, _)
        | Constraint::ArcLength(_, _)
        | Constraint::ConcentricArcArc(_, _)
        | Constraint::EqualLength(_, _)
        | Constraint::Midpoint(_, _)
        | Constraint::Symmetric(_, _, _)
        | Constraint::SymmetricAboutPoint(_, _, _) => false,
    }
}

/// Check if a constraint references a specific arc.
fn constraint_references_arc(c: &Constraint, id: ArcId) -> bool {
    match c {
        Constraint::PointOnArc(_, arc) | Constraint::ArcLength(arc, _) => *arc == id,
        Constraint::TangentLineArc(_, arc, _) => *arc == id,
        Constraint::TangentArcArc(a1, a2, _)
        | Constraint::EqualRadiusArcArc(a1, a2)
        | Constraint::ConcentricArcArc(a1, a2) => *a1 == id || *a2 == id,
        Constraint::EqualRadiusArcCircle(arc, _) | Constraint::ConcentricArcCircle(arc, _) => {
            *arc == id
        }
        Constraint::Coincident(_, _)
        | Constraint::Distance(_, _, _)
        | Constraint::PointLineDistance(_, _, _)
        | Constraint::FixX(_, _)
        | Constraint::FixY(_, _)
        | Constraint::Horizontal(_)
        | Constraint::Vertical(_)
        | Constraint::Angle(_, _, _)
        | Constraint::Perpendicular(_, _)
        | Constraint::Parallel(_, _)
        | Constraint::PointOnCircle(_, _)
        | Constraint::CircleRadius(_, _)
        | Constraint::EqualRadiusCircleCircle(_, _)
        | Constraint::EqualLength(_, _)
        | Constraint::Midpoint(_, _)
        | Constraint::Symmetric(_, _, _)
        | Constraint::TangentLineCircle(_, _)
        | Constraint::SymmetricAboutPoint(_, _, _) => false,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests;
