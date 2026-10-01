# PERF-S04 / B30: bounded sparse coupled-sketch solver

Owner row: PERF-S04 (sparse Jacobian + rank-revealing sparse solve for large
coupled sketches, dense as small-system baseline). Builds on PERF-S02's
component decomposition, PERF-S03's solve-local workspaces and PERF-S06's
final-iterate capture; changes none of their contracts. No new sketch
entities, no new constraints, no topology change, no cross-call caching, no
public solve semantics or JS result-shape changes, no new workspace or
external dependency.

- Baseline source: `4a93e758` (origin/main; clean worktree).
- Candidate code commit: (this PR; report/roadmap/package commits stack on top).
- Candidate packages: paired `remus-wasm` + `remus-wasm-io` `2026.1.22`,
  fresh `cargo xtask wasm-build` from the candidate source (dual-target merge
  + wasm-opt, smoke plus installed-tarball consumer checks pass; version
  advanced `2026.1.21 → 2026.1.22` under the shared-version policy for
  changed WASM bytes `9681642 → 9757223`).
- Toolchain: rustc/cargo 1.96.0 (`ac68faa20 2026-05-25`), node v24.14.0,
  Python 3.12.3, Linux x86_64, AMD Ryzen 9 5900XT (32 logical). Native worker:
  release profile for criterion (`opt-level 3`, LTO as configured) and
  profiling release (`opt-level 3`, debuginfo, no LTO) for the paired
  dense/sparse cells below. Solver settings: tolerance `1e-10`, `max_iter`
  100 (50 for inconsistent), same as S01.

Reproduce:

```bash
cargo test -p remus-sketch
cargo test -p remus-sketch --test s04_identity
cargo test -p remus-operations --lib
cargo test -p remus-wasm --lib gcs
cargo bench -p remus-sketch --bench gcs_perf -- sketch/s04_coupled
cargo bench -p remus-sketch --bench gcs_perf -- sketch/coupled_chain/solve/1000
REMUS_SKETCH_FORCE_DENSE=1 cargo bench -p remus-sketch --bench gcs_perf -- sketch/s04_coupled/chain_solve/300
node scripts/performance/sketch/wasm.cjs ./crates/wasm/pkg independent_solved 100 solve 3 1
```

## 1. Current-state semantics (pinned before optimizing)

Unchanged from PERF-S02/S06 (`sketch-perf-s02.md` §1, `sketch-perf-s06.md`
§1): `solve` publishes its final iterate (a miss still moves geometry),
`solve_detailed` measures at the attempt then restores pre-solve geometry on
a miss, per-constraint residuals describe the attempt, `published_max_residual`
the published state. Convergence is NaN-propagating `max_abs_residual <
tolerance` within `max_iter` DogLeg steps; trust region opens at
`clamp(max(1, 0.1·|p|), …, 1e4)` with the 0.75-double / 0.25-quarter rho rule
and the `1e-15·(1+|p|)` small-step exit; rank is Householder-QR-with-pivoting
`#{|Rii| > 1e-10·|R00|}`, `dof = n − rank`, classified Unsatisfied >
UnderConstrained > Redundant > Solved. Entry validation rejects nonfinite /
non-positive magnitudes (`InvalidValue`); fixed points contribute no
parameters; circular-ellipse (`a ≈ b`) phi columns vanish truthfully.

## 2. Design: structural CSR plus banded Givens, per-component dispatch

New module `crates/sketch/src/gcs/sparse.rs`; `constraint.rs` gains a
`JacobianSink` trait so the single `eval_jacobian` formula source serves
dense (`JacobianWriter`) and sparse (`SparseWriter`) layouts; `solver.rs`
exposes `max_abs_residual` and `dogleg_step_into` for the sparse loop (same
norms, same interpolation — any change must be mirrored); `system.rs` routes
` solve_component` and `solve_dense` through the bounded slice with dense
fallback.

- **Structural, not numerical.** Patterns come from
  `components::constraint_param_indices` (the exhaustive union over rows,
  formula facts true at every iterate, including degenerate `1e-300` guards).
  Each constraint's union is replicated per residual row (conservative: may
  include structural zeros where a formula never reads that parameter in that
  row, but never misses a nonzero). No wildcard: `direct_refs` is exhaustive,
  so a new `Constraint` variant fails to compile until declared. Proven by
  `dense_entries_outside_pattern_are_zero` (chain at base, perturbed and
  coincident-degenerate states: every dense entry outside the union is
  exactly `0.0`).
- **Banded Givens, no normal equations.** The GN step solves
  `min ||J h + r||` via orthogonal Givens rotations on CSR (never forming
  `JᵀJ`, which would square conditioning). Gradient (`Jᵀr`) and products
  (`J·g`, `J·h`) are sparse matvecs `O(nnz)` in the same order as dense
  skipping zeros (rows in order, columns sorted) — finite results match dense
  bit-for-bit (`x+0 == x`); any nonfinite input fails closed to all-`NaN`
  (matching dense `0·NaN/Inf = NaN` poisoning where dense poisons everything
  and sparse would otherwise poison only coupled entries).
- **Bounded dispatch (explicit).** `should_use_sparse(m,n,band)` requires
  `n >= 128` (small systems keep dense bit-for-bit, preserving the S02
  `single_component_stays_bitwise_dense` contract at 10/100), `m >= n`
  (square/overdetermined; underconstrained `m < n` needs column pivoting and
  falls back), and scaled-diagonal bandwidth `bw <= 64` where
  `bw = ceil(max|j·m − i·n|/m)` over structural nonzeros. Chain `bw≈2`,
  grid `bw≈24`, interleaved linkage `bw≈6`; tangent `bw≈199` and ellipse
  `bw≈183` (point+scalar mixing under points-first param order) exceed the
  bound and fall back — correctness preserved, no win claimed there.
  `REMUS_SKETCH_FORCE_DENSE=1` forces `false` for paired same-binary dense
  baselines (measurement only; unset in production; absent on `wasm32`).
- **Full-rank commit, otherwise fallback.** Givens uses row swaps (smallest
  suitable row, deterministic) for extra rows whose pivot starts below `k`;
  column order is never permuted. Rank uses the single global
  `1e-10·max_col_norm` policy (same scale as `dof::analyze`: `max_col_norm`
  equals pivoted `|R00|`). Any tiny pivot, nonfinite, or structural miss
  returns `None` so the caller reruns dense from the same start — rank, DOF
  and classification never silently change. Per-iteration solves use the
  component's own scale (per-component independent loops, S02); final-rank
  analysis uses the global max across blocks (S02 policy) via
  `sparse_full_rank_with_threshold`, falling back per-block to dense on any
  miss.
- **Per-component, solve-local, deterministic.** Each S02 component decides
  independently (single-component large chains go through `solve_dense_sparse`
  with full snapshots and a single write-back). Workspaces (`SparseWorkspace`)
  are sized per call, reused across iterations, dropped after — never retained
  across solves (no S05 factor reuse). Union-find ordering, `BTreeMap`
  grouping/rows, sorted CSR/Givens, and fixed scan windows keep repeated
  builds identical.

## 3. Correctness evidence

- `gcs/sparse/tests.rs` (7 tests): dispatch pins (small/wide/under fallback),
  `s04_bands_pin_expected_dispatch` (chain/grid/linkage dispatch, tangent/
  ellipse fallback — prevents silent dispatch drift), matrix contract
  (`dense_entries_outside_pattern_are_zero` at base/perturbed/degenerate),
  matvec identity + fail-closed NaN/Inf, Givens on banded full-rank
  (independent residual check) + rank-deficient/NaN fallback, and
  `chain_200_dense_and_sparse_agree_on_contracted_outcomes` (forced dense via
  `solve_dense_direct` vs public sparse-dispatched: converged, rank/dof/
  classification/Solved, residuals `<1e-10`, iterations within one — currently
  exactly equal at 5/5).
- `crates/sketch/tests/s04_identity.rs` (7 tests): dimensions plus cold
  outcomes for all five families at ~100 params (chain Solved, grid
  Solved/Redundant, linkage converged, tangent converged UnderConstrained
  with 90 iters, ellipse Solved with 18 iters) and chain redundant
  (Redundant) + inconsistent (Unsatisfied + rollback) variants. No wall-time
  gates.
- Full position: `cargo test -p remus-sketch` 207 lib + 11 identity + 7 s04 +
  5 workspace-reuse + 1 doc-test green; `cargo test -p remus-operations
  --lib` 1234 green (7 ignored, pre-existing); `cargo test -p remus-wasm
  --lib gcs` 35 green (including `gcs_ellipse_batch_matches_direct` and
  `gcs_batch_v2_envelope_matches_v1_results`); `cargo clippy -p remus-sketch
  --all-targets -- -D warnings` clean; `cargo fmt` applied;
  `check-boundaries`, `check-det-hash`, `check-doc-paths` pass.
- Preserved: failed-solve rollback (`inconsistent_matches_dense_with_whole_
  call_rollback` + s04 inconsistent variant), nonfinite rejection
  (`non_finite_values_rejected_at_entry`, `nan_point_never_reports_
  convergence`, sparse matvecs/Givens fail closed to all-NaN/None),
  fixed-parameter behavior (fixed points contribute nothing — pattern unions
  exclude them, sharing-only-fixed stays independent per S02), and
  circular-ellipse gauge (`point_on_ellipse_phi_column_vanishes_at_equal_axes`
  still reports truthful rank loss; sparse tiny-pivot falls back to dense so
  the gauge never silently changes).

## 4. Measurements: same-host paired dense vs sparse (complete solves)

Native release criterion, same binary, same host back-to-back (sparse via
structural gate, dense via `REMUS_SKETCH_FORCE_DENSE=1`); medians with
estimate bounds, no p95 (sample counts do not qualify tail independence).
Iterations/rank/residuals/counts from cold `solve_detailed_counted` (same
source, profiling release for counts, release for timings — counts carry no
timing meaning).

| Case / mode | Dense median | Sparse median | Factor | Iters (cold) | Rank/dof/class |
| --- | ---: | ---: | ---: | ---: | --- |
| chain_300 solve | 59.6 ms | 0.88 ms | 67x | 5/5 | 300/0 Solved, max_r ~1e-61 |
| chain_1000 solve | 3.67 s | 3.41 ms | 1076x | 5/5 | 1000/0 Solved, max_r ~5e-60 |
| chain_1000 detailed | 4.45 s | 3.85 ms | 1156x | 5/5, solver 11/5/5, analysis 1/1, shared | same |
| grid_300 solve | 150 ms | 23 ms | 6.5x | 4/4 | 300/0 Redundant (m=554), max_r ~5e-18 |
| linkage_300 solve | 120 ms | 1.57 ms | 76x | 4/4 | 300/0 Redundant (m=449), max_r ~1e-13 |
| tangent_99 solve (dense, fallback) | 44 ms (n<128 dense) | same (fallback) | — | 90, rank 98/dof 1 UnderConstrained, max_r 4e-15 | solver 181/90/90 |
| tangent_300 solve (dense fallback) | 1.18 s | same | — | — (fallback, no win claimed) | band 199>64 |
| ellipse_100 detailed (dense fallback) | 7.6 ms bench | same | — | 18, rank 100/dof 0 Solved, max_r 3e-11 | solver 37/18/18 |
| ellipse_300 solve (dense fallback) | 220 ms | same | — | — | band 183>64 |
| chain_under_100 detailed (dense, m<n) | — | fallback | — | 7, rank 50/dof 50 UnderConstrained | m=50<n=100 |
| chain_ill_100 detailed (dense, n<128) | — | dense | — | 6 (vs 5 well-conditioned), Solved | 1 extra iter, still converges |
| chain_redundant_100 | — | dense (n<128) | — | 8, Redundant | S01 8/8 |
| chain_inconsistent_100 | — | dense | — | 49/50, Unsatisfied + rollback | S01 49/49 |

Growth: chain 100→1000 dense `2.86 ms → 3.67 s` (~1280x, cubic) vs sparse
`~2.2 ms → 3.4 ms` (~1.5x, linear `O(n·bw²)` with `bw=2`). Grid 98→300 dense
would be ~27x cubic; sparse 3.9 ms → 23 ms (~6x). No material regression on
small systems: 10/100-param cells keep dense bit-for-bit (dispatch `n>=128`;
S02 `single_component_stays_bitwise_dense` still passes).

Packaged WASM (`2026.1.22`, same source as native): `wasm.cjs` representative
cells all `validation passed` with exact native agreement (outcomes,
dimensions, iterations, classifications, oracle errors); e.g.
`independent_solved_100` 9/9 iters (S02 component fingerprint, sparse not
triggered at n=100), `coupled_chain_100` 5/5. Built-WASM ellipse batch
(`executeBatch`, 9 ops: New/AddPoint/AddEllipse/3×AddConstraint/Solve/
SolveDetailed/Dof) converges (`converged true`, 4 iters, `Solved`, rank 3/
dof 0); native `gcs_ellipse_batch_matches_direct` + `gcs_batch_v2_envelope_
matches_v1_results` prove direct-vs-batch and v1-vs-v2 parity (built-WASM
v1/v2 string compare differs only in envelope key order, solver payloads
agree). No cross-runtime speedup is inferred (different builds, same solver
— S01 discipline).

## 5. Allocations: structural accounting (no instrumented allocator)

As in S01/S02/S06, allocations are accounted structurally — every eliminated
heap object named (no allocation-counting harness):

Per converged large `solve` at 1000×1000 chain (5 iters): the 8,000,000-byte
global Jacobian plus its same-sized QR working copy (`16 MB` together) plus
`n`/`m`-sized step vectors collapse to one CSR pattern (`row_ptr` 8·(m+1) ≈
8 KB, `col_idx` 8·nnz ≈ 24 KB for nnz≈3000, `values` 8·nnz ≈ 24 KB → ~56 KB)
with proportionally tiny Givens row maps (`m` ordered maps, ≤~7 entries each
for `bw=2`, ~100 KB with tree overhead) and same-sized step vectors. Measured
effect: chain-1000 solve `3.67 s → 3.4 ms` with identical 5/5 iterations and
`~1e-60` residuals; detailed `4.45 s → 3.9 ms` (solver 11/5/5 plus one sparse
analysis 1/1, shared residuals — counts unchanged from S06, only storage
sparse). Per-evaluation subset snapshots still visit only the block's own
entities (S02), so per-evaluation work scales with block size. Still
allocated per call: one CSR values fill per Jacobian eval, one Givens
factorization per iteration plus one for rank (never shared — an accepted
final step can invalidate the loop's previous one), the per-constraint report
itself, and the assembled capture (`(n+m)·8` plus row layout, S06 shape).

## 6. Limitations and what stays open

- Tangent-heavy and ellipse-chain systems (any point+radius/scalar mixing
  under points-first param order) exceed `bw=64` (`199`/`183`) and fall back
  to dense — correctness preserved (tangent99 converges in 90 iters
  UnderConstrained, ellipse100 in 18 iters Solved), no speedup claimed there.
  Column reordering (e.g. RCM interleaving nearby points and scalars) would
  bring them into the slice; explicitly deferred as future work with no
  silent behavior change.
- Underconstrained (`m < n`, e.g. chain-under 50×100) needs column pivoting
  to align the diagonal and falls back (honest `Unsatisfied`/
  `UnderConstrained` preserved).
- Far loop closures (one row coupling distant parameters) blow the scaled
  bandwidth the same way trailing bracing did before interleaving (linkage
  band `202 → 6` after interleaving); genuinely far closures still fall back.
- Rank-deficient / below-threshold ill-conditioned systems fall back to dense
  for exact rank (same `1e-10` policy, no re-ranking). Mixed-scale
  per-component relative thresholds remain rejected (S02); final analysis
  uses the single global max.
- Iteration counts on sparse-dispatched full-rank banded systems currently
  match dense exactly (chain 5/5, grid/linkage 4/4); any future divergence
  from roundoff (GN via Givens vs Householder) is allowed at most ±1 with an
  explicit explanation — same honesty rule as S02's 11-vs-8 component shifts.
- Timing evidence is same-host sequential with pinned sources (criterion
  estimates + bounds; 10–20 samples per cell; min/max in raw outputs), not a
  quiet-room study. No timing assertion exists in any test. `REMUS_SKETCH_
  FORCE_DENSE` is measurement-only (unset in production; absent on wasm32).
- Cross-call sharing (warm-start drag solves, persistent patterns/factors)
  remains explicitly declined (S05): patterns/workspaces rebuild per call.

## 7. Evidence files

- Committed: `crates/sketch/benches/gcs_perf.rs` (S04 builders + `bench_s04_
  coupled`/`bench_s04_variants`), `crates/sketch/tests/s04_identity.rs`,
  `crates/sketch/src/gcs/sparse.rs` + `sparse/tests.rs` (contract, adversarial,
  agreement, band pins).
- Run outputs (ephemeral, never overwritten): criterion raw outputs quoted in
  §4 (sparse vs `REMUS_SKETCH_FORCE_DENSE=1` same-binary pairs); native
  `solve_detailed_counted` counts quoted in §4 (solver 11/5/5 etc., shared
  true); packaged-WASM `wasm.cjs` JSON samples (validation `passed`, 9/5/100
  fingerprints) plus the Node `executeBatch` ellipse batch (converged,
  Solved, dof 0).
- Packages: `remus-wasm` + `remus-wasm-io` `2026.1.22` (WASM bytes
  `9757223`/`2802402`, hashes in `run` artifacts / package files).
