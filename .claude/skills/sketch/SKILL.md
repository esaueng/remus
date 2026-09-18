---
name: sketch
description: Use when working on the remus-sketch 2D parametric GCS constraint solver: adding a Constraint variant, debugging a solve or solve_detailed non-convergence or misclassification, checking an analytic Jacobian at scale, or exposing sketch ops through the typed gcs* WASM bindings.
---

# Sketch (GCS Constraint Solver)

## When to use

- You are touching `crates/sketch/src/gcs/` (system, constraint, solver, qr, dof, diagnostics, entity), `crates/operations/src/sketch.rs`, or `crates/wasm/src/bindings/gcs_sketch.rs`, or diagnosing a `solve` / `solve_detailed` outcome.
- Adding a `Constraint` variant: do NOT copy any match-site list here — the exhaustive sites (`residual_count`, `eval_residuals`, `eval_jacobian`, `validate_constraint`, the four `constraint_references_*` helpers, `parse_gcs_constraint`) are named in the Module Map sketch rows of CLAUDE.md / AGENTS.md. Follow that list.

## Quick reference

| Need | Call | Note |
|---|---|---|
| Add geometry / constraints (entries validated) | `add_point`, `add_constraint` | Return `Result<_, SketchError>`; `add_point` / `add_circle` became `Result` in PR #453 |
| Solve in place, publishing the final iterate | `solve` | `crates/sketch/src/gcs/system.rs`; a miss still moves geometry |
| Solve transactionally, with rank + per-constraint report | `solve_detailed` | Rolls back on a miss; residuals read at the best attempt |
| Remaining freedom at the current state | `dof` | Returns `DofAnalysis { dof, rank, num_params, num_equations }` |

Verbatim signatures (`crates/sketch/src/gcs/system.rs`):

```rust
pub fn add_point(&mut self, data: PointData) -> Result<PointId, SketchError> {
pub fn add_constraint(&mut self, constraint: Constraint) -> Result<ConstraintId, SketchError> {
pub fn solve(
    &mut self,
    max_iterations: usize,
    tolerance: f64,
) -> Result<SolveResult, SketchError> {
pub fn solve_detailed(
    &mut self,
    max_iterations: usize,
    tolerance: f64,
) -> Result<SolveDiagnostics, SketchError> {
pub fn dof(&mut self) -> DofAnalysis {
```

## Architecture in ten lines

- `gcs/entity.rs`: generational `GenArena` plus `PointData` / `LineData` / `CircleData` / `ArcData`; stale handles fail closed as `InvalidHandle`.
- `gcs/constraint.rs`: 26 `Constraint` variants (the `lib.rs` header still says 24), each with analytic `eval_residuals` + `eval_jacobian`; `residual_count` is 1 or 2 per variant.
- `gcs/solver.rs`: DogLeg trust-region `solve_dogleg`, the FreeCAD PlaneGCS family; residual folds are NaN-propagating since PR #453.
- `gcs/qr.rs`: Householder QR with column pivoting; serves both the least-squares step and rank detection.
- `gcs/dof.rs`: `analyze` yields DOF as params minus QR rank; redundant rows remove no freedom.
- `gcs/diagnostics.rs`: `classify` precedence is Unsatisfied, then UnderConstrained, then Redundant, then Solved; residuals never name a culprit.
- `gcs/system.rs`: `GcsSystem` owns the arenas plus the param map (free point coordinates and circle radii only); `solve` publishes, `solve_detailed` restores pre-solve state on a miss.
- `add_arc` installs one internal `PointOnArc` tie with no caller handle; reports flag it via `internal` and `is_internal_constraint` (`diagnostics_separate_internal_arc_constraints`).
- Degenerate geometry (coincident line endpoints, zero-length axis, point at circle center) drops the gradient through `line_len_dir` / `1e-300` guards — never divides (`constraint.rs`).
- JS surface is `parse_gcs_constraint` in `crates/wasm/src/bindings/gcs_sketch.rs`; the crate has zero workspace deps per the AGENTS.md layer table.

## Symptom-to-cause

| Symptom | Cause | Fix pointer |
|---|---|---|
| Solve reports `converged: true, max_residual: 0.0, iterations: 0` on poisoned input | Solver folded residuals with `f64::max`, which drops NaN; the diagnostics layer already had a NaN-propagating fold the solver did not use | PR #453 (`966473c7`): NaN-propagating `max_abs_residual` in all solver folds plus the `n == 0` fast path; pinned by `nan_point_never_reports_convergence` |
| `add_point` / `add_circle` / `add_constraint` accept NaN, infinite, or non-positive magnitudes | Value-carrying entries did not validate despite `SketchError::InvalidValue` existing for that purpose | PR #453: entry validation everywhere plus `Result` returns (BREAKING for `add_point` / `add_circle`); pinned by `non_finite_values_rejected_at_entry` |
| New analytic Jacobian passes at unit scale, fails at 1e5 | Fixed-step (`1e-7`) `check_jacobian_fd` loses to cancellation at large coordinates | `crates/sketch/src/gcs/constraint/tests.rs`: scale-relative `check_jacobian_central` (`eps = 1e-6 * scale`, `SCALES = [1e-3, 1.0, 1e5]`); introduced in `7916966f`, extended in `1e3eef74` (#56) |

## Traps

- `solve` and `solve_detailed` publish differently on failure: `solve` keeps its last iterate, `solve_detailed` restores pre-solve coordinates exactly (`diagnostics_roll_back_a_failed_solve`, `crates/sketch/src/gcs/system/tests.rs`).
- Per-constraint residuals describe the solver's best attempt, NOT the rolled-back state — re-reading them against current geometry misattributes (`system.rs::solve_detailed`, `diagnostics.rs::ConstraintResidual`).
- A fully-pinned system (every point `fixed`, no circles) carrying any constraint classifies `Redundant`: zero Jacobian columns make every row dependent (`diagnostics_classify_a_fully_pinned_system`).
- The arc internal tie counts in `num_equations`; filtering `internal` residuals out of the report breaks the equation accounting (`diagnostics_separate_internal_arc_constraints`).
- Residuals are not raw lengths: `Distance` uses squared form scaled by `max(1, 2d)`, `Symmetric` normalizes by axis length — set tolerances in residual units (`constraint.rs`).
- Radius is the kernel unit; diameter callers convert at their boundary (`Constraint::CircleRadius` doc, `constraint.rs`).

## Anti-patterns

- "Converged, so the inputs were sane." Pre-#453 NaN folded to `converged: true` — validate at entry and read `max_residual` (`nan_point_never_reports_convergence`).
- "The largest residual names the broken constraint." One bad constraint pushes error into every constraint sharing its parameters (`diagnostics.rs` module doc; `diagnostics_report_contradictory_constraints_without_blaming_one`).
- "DOF is equations minus params." DOF is params minus QR rank; dependent rows add equations but no constraint (`dof.rs::analyze`).
- "Fixed-step FD cover is enough for a Jacobian." Not at 1e5 — use `check_jacobian_central` at all three `SCALES` (`constraint/tests.rs`).
- "A removed constraint still shows up in reports." Removed handles are rejected and excluded; stale snapshot lookups poison to NaN instead (`diagnostics_ignore_stale_constraint_handles`, `constraint.rs::EntitySnapshot`).

## Related skills

debugging-doctrine (the vary-one-variable method), solid-verification (never sign off on handles alone), testing (FD/Jacobian cover and regression fixtures), wasm-bindings (`parse_gcs_constraint`, batch parity), roadmap (PERF-S01–S06 own sketch perf, all Proposed; B16 still owes the GCS qualification matrix).
