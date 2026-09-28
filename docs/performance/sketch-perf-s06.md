# PERF-S06 / B30: share final-iterate evaluation in `solve_detailed`

Owner row: PERF-S06 (bounded optimization). Builds on PERF-S03's solve-local
workspaces; does not repeat that work. No component decomposition (S02), no
sparse solving (S04), no new sketch entities, no public solve semantics or JS
result-shape changes.

- Baseline source: `bf46fdc0af611d480bf0830ff1da6c475d8b96c9` (origin/main).
- Candidate source: `a3cc1f4c4d0f02739d943b828cf3d99026ceb9d1` (this PR's code
  commit; report/roadmap/package commits stack on top).
- Baseline package: committed `remus-wasm` `2026.1.2`, built from
  `68c4293aaa1b` — `crates/sketch` and `crates/wasm/src` are byte-identical
  between `68c4293aaa1b` and `bf46fdc0` (that refresh touched package files
  only), so the baseline WASM cells are sketch-source-pinned.
- Candidate package: `remus-wasm` `2026.1.3`, fresh `cargo xtask wasm-build`
  from the candidate source (validated: dual-target merge + wasm-opt, smoke
  plus installed-tarball consumer checks pass).
- Toolchain (both builds): rustc/cargo 1.96.0, node v24.14.0, Python 3.12.3,
  Linux x86_64, AMD Ryzen 9 5900XT (32 logical). Native worker: profiling
  release (`opt-level 3`, debuginfo, no LTO). Manifest
  `scripts/performance/sketch/workloads.json` sha256 `714f71ad…` (identical
  for every run below). Workload solver settings: tolerance `1e-10`,
  `max_iter` 100 (50 for inconsistent), 20 drag steps.

Reproduce:

```bash
cargo test -p remus-sketch
PERF_S06_REPORT=/tmp/perf-s06-counts.txt cargo test -p remus-sketch --lib final_eval::tests::report_counts
python3 scripts/performance/sketch/run.py --offline --smoke
python3 scripts/performance/sketch/run.py --offline --case independent_solved_100 --case independent_solved_1000 --case coupled_chain_100 --case coupled_chain_1000 --case redundant_100 --case inconsistent_100 --case drag_100
cargo bench -p remus-sketch --bench gcs_perf -- 100
```

## 1. Current state semantics (pinned before optimizing)

Documented in `crates/sketch/src/gcs/final_eval.rs` and pinned by tests in
`crates/sketch/src/gcs/final_eval/tests.rs`:

- `solve` publishes its final iterate. A miss still moves geometry
  (`solve_publishes_its_final_iterate`: params change, and the reported max
  equals a fresh fold at the published state).
- `solve_detailed` restores the starting geometry after non-convergence
  (`detailed_restores_starting_geometry_on_miss`: pre-solve params return
  bit-identically, `rolled_back` set).
- Attempted-state residuals remain informative: at the best attempt the
  satisfiable constraints read ~0 while the system as a whole misses; the
  restored starting geometry reads large on those same constraints
  (`attempted_state_residuals_stay_informative`).
- `published_max_residual` describes the state actually left published:
  equal to `max_residual` when converged, re-measured at the restored state
  after a rollback (`published_max_matches_attempt_when_converged` and the
  rollback half of `attempted_state_residuals_stay_informative`).

## 2. Duplicate-work census (measured, candidate build)

Exact per-call counts from `solve_detailed_counted` vs the fresh oracle
(`evidence/perf-s06/counts-candidate.txt`; oracle replicates the pre-S06
`solve` + fresh `dof` + fresh `constraint_residuals` + fresh restored-state
pass without consulting a capture):

| Case | Iters | Solver res/jac/QR | Shared? | Analysis jac/QR | Restored passes | Oracle extra final passes |
| --- | ---: | --- | --- | ---: | ---: | ---: |
| independent_solved_100 | 8 | 17 / 8 / 8 | yes | 1 / 1 | 0 | 1 |
| independent_under_100 | 8 | 17 / 8 / 8 | yes | 1 / 1 | 0 | 1 |
| coupled_chain_100 | 5 | 11 / 5 / 5 | yes | 1 / 1 | 0 | 1 |
| redundant_100 | 8 | 17 / 8 / 8 | yes | 1 / 1 | 0 | 1 |
| inconsistent_100 | 49 | 99 / 49 / 49 | yes | 1 / 1 | 1 | 1 |

Reading: every `solve_detailed` call eliminates exactly one full residual
pass over all `m` equations plus the second fresh snapshot build that backed
it (the oracle's `final_state_residual_passes = 1` column). Jacobian and QR
counts are unchanged by construction — the loop's factorizations are untouched
and the rank analysis still performs exactly one fresh Jacobian evaluation
plus one fresh factorization per call. The restored-state pass after a
rollback is retained on both paths: it measures a genuinely different state,
so sharing there would be wrong.

Cost of what was removed, honestly: one residual pass is `m` analytic
constraint evaluations plus one `EntitySnapshot` build (4 hash tables sized
to the entity counts, cleared and refilled). It is linear work beside the
dense Householder QR's cubic term — Section 5 shows it is below the
wall-time noise floor at every measured size, which is why this report
claims counts, not speedups.

## 3. Design: solve-local evaluation with explicit identity

New module `crates/sketch/src/gcs/final_eval.rs`; solver exposes exact loop
counts (`SolveStats`) and the canonical final residual vector
(`DoglegWorkspace::final_residuals`, valid on every exit path — the
small-step exit copies its trial evaluation across); `system.rs` routes
`solve`/`solve_detailed` through one `solve_impl` and measures diagnostics
from a single snapshot plus the verified vector.

- An accepted final step can invalidate the previous factorization, so **no
  factorization is ever shared**. Rank always comes from a fresh
  factorization of a Jacobian evaluated at the verified final params. Only
  the residual vector is shared.
- Reuse requires bitwise-equal final params, the exact constraint row layout
  (`(id, row count)` in evaluation order), and both dimensions
  (`FinalEvaluation::matches`). The solve tolerance needs no separate field:
  it selects *which* iterate is final, and the identity compares the actual
  final params, so any tolerance-driven difference surfaces as a parameter
  mismatch. Rank uses the shared `dof::analyze` threshold on both paths.
- No persistent cache: the capture is created and consumed inside one
  `solve_detailed` call. Edits between solves (moved point, added/removed
  constraint) change params or row layout, fail the check, and measure
  fresh. The defensive length fallback and the degenerate-dimension
  (`n == 0 || m == 0`) fresh path run the same `dof()` +
  `constraint_residuals()` calls as before.
- Classifications, rank, DOF, iteration counts, per-constraint attribution,
  internal-arc flags, and rollback behavior are preserved by construction
  (same folds, same `classify`, same write-back order) and verified bitwise
  below.

## 4. Correctness evidence

`crates/sketch/src/gcs/final_eval/tests.rs` (21 tests) compares the shared
path against the independent fresh oracle with bitwise (`to_bits`,
NaN-aware) agreement on every `SolveDiagnostics` field, the full
per-constraint report, and the published params: converged solved (4 and 20
params), underconstrained, coupled chain, redundant, inconsistent with
rollback, empty systems (no params / no constraints), zero iterations,
iteration-limited, arc internal-constraint attribution, edits between solves
(geometry perturb, add contradiction, remove it), constraint removal
(redundant → solved), extreme scales (1e-3/1/1e5 × offset 0/1e4 with an
independent hypotenuse oracle), the four state-semantics pins, the identity
contract (bit-flip/row-count/row-order/dimension/removal rejection), and
solver-final vs fresh-max agreement.

Counts are asserted per case: shared path on, fallback off, exactly one
analysis Jacobian plus one QR (zero of each when degenerate), exactly one
restored-state pass on rollback and none otherwise, solver residual
evaluations ≥ 1 with factorizations ≤ Jacobian evaluations.

Full workspace position at the code commit: `cargo test -p remus-sketch`
135 lib + 9 identity + 5 workspace-reuse + 1 doc-test green;
`cargo clippy -p remus-sketch --all-targets -- -D warnings` clean;
`cargo fmt` applied; every sketch-runner sample below passed its
identity/dimension/outcome gates natively and through packaged WASM.

## 5. Measurements: elapsed time (no effect claimed)

Native profiling-release runner, 3 processes × 5 retained samples + 1
warmup (15 retained per cell), sequential builds, same host:

| Case / mode | Baseline median | Candidate median | Candidate rep2 median |
| --- | ---: | ---: | ---: |
| solved_100 solve / detailed | 3.959 / 4.322 | 4.108 / 4.347 | — / 4.196 |
| solved_1000 solve / detailed | 4797.9 / 5493.1 | 4916.2 / 5575.3 | — / 5387.8 |
| chain_100 solve / detailed | 2.456 / 2.897 | 2.503 / 2.981 | — / 2.820 |
| chain_1000 solve / detailed | 3091.9 / 3708.1 | 3098.7 / 3699.4 | — / 3609.3 |
| redundant_100 solve / detailed | 6.581 / 7.405 | 6.654 / 7.478 | — / 7.316 |
| inconsistent_100 solve / detailed | 9.014 / 8.949 | 9.028 / 9.090 | — / 8.918 |
| drag_100 solve (20 steps) / detailed | 47.762 / 56.217 | 48.137 / 57.051 | 47.169 / 56.017 |

(All ms; min/max plus per-process medians in
`evidence/perf-s06/summary-*.json`. The `solve` column is untouched loop
code — its ±2.5% drift between runs is host noise, confirmed by the mixed
signs and overlapping min/max bands.)

Criterion release profile, 20 samples, single filter run per build
(`evidence/perf-s06/criterion-*.txt`, estimate column):

| Bench | Baseline | Candidate |
| --- | ---: | ---: |
| under_100 solve / detailed | 1.3639 / 1.4769 ms | 1.3498 / 1.4857 ms |
| under_1000 solve / detailed | 1.3896 / 1.5573 s | 1.4134 / 1.5892 s |
| solved_100 solve / detailed | 3.7022 / 4.0870 ms | 3.6753 / 4.0886 ms |
| solved_1000 solve / detailed | 4.7676 / 5.3368 s | 4.7926 / 5.3855 s |
| chain_100 solve / detailed | 2.3003 / 2.7472 ms | 2.3129 / 2.6805 ms |
| chain_1000 solve / detailed | 3.0006 / 3.5654 s | 3.0273 / 3.6274 s |
| redundant_100 solve / detailed | 6.3947 / 7.0473 ms | 6.3612 / 6.9907 ms |
| inconsistent_100 solve / detailed | 8.5484 / 8.6708 ms | 8.5217 / 8.6003 ms |
| drag_100 solve_step / detailed_step | 2.3296 / 2.7354 ms | 2.3220 / 2.7135 ms |

Every cell is within ±2.4% with mixed signs: no regression and no speedup
claimed. That is the expected outcome — the removed work (Section 2) is one
linear pass beside 5–49 cubic factorizations. QR's asymptotic cost has not
disappeared and is not claimed to have: factorizations per call are
identical (loop count + 1) before and after.

Packaged WASM, same sources as native, 1 process × 5 retained samples + 1
warmup, existing `gcs*` bindings (no binding changes in this PR):

| Case / mode | Baseline pkg 2026.1.2 | Candidate pkg 2026.1.3 |
| --- | ---: | ---: |
| solved_100 solve / detailed | 4.730 / 5.124 | 4.615 / 5.065 |
| chain_100 solve / detailed | 2.910 / 3.475 | 2.952 / 3.516 |
| drag_100 solve (20 steps) / detailed | 56.693 / 67.889 | 55.734 / 66.139 |

Outcomes, dimensions, iteration counts, and classifications agree exactly
with native on both packages (the runner rejects any mismatch); wall times
agree within run-to-run noise. Different builds, same solver: no
cross-runtime conclusion is drawn, matching the S01 discipline.

## 6. Allocations: structural accounting (no instrumented allocator)

This build has no allocation-counting harness (the S01 baseline likewise
reports deterministic buffer sizes, not allocator traffic), so allocations
are accounted structurally — every eliminated heap object named:

Per converged `solve_detailed` call, the shared path no longer allocates:

- one `EntitySnapshot`: 4 hash tables (`points`, `lines`, `circles`,
  `arcs`) with capacities for the live entity counts — e.g. at 100 params /
  50 Distance constraints, ~101 + 0 + 0 + 0 entries across the tables; at
  1000 params / 500 constraints, ~1001 entries;
- one flat residual `Vec<f64>` of `m` entries (`m * 8` bytes: 800 B at
  m = 100, 8 KB at m = 1000);
- the per-constraint scratch buffer reuse is unchanged (single `buf` in the
  fresh path either way).

Per rolled-back call it additionally avoids the discarded restored-state
report `Vec<ConstraintResidual>` (`k * 24` bytes for `k` constraints),
keeping only the max-only fold over a flat evaluation, which is value-
identical (max over the union, NaN-propagating both ways — verified
bitwise by the oracle tests).

Still allocated exactly as before per call: the solver workspaces
(PERF-S03, unchanged), one fresh Jacobian (`m * n * 8` bytes: 80 KB at
100×100, 8 MB at 1000×1000) plus its QR factorization, and the published
per-constraint report itself (part of the return value, not overhead).
`solve_detailed` clones the final params, residuals, and row layout
into its solve-local capture (`(n + m) * 8` bytes plus `k` id/count pairs
— ~1.6 KB at 100 params). Plain `solve` skips this capture and its
allocations.

## 7. Limitations and what stays open

- The dense Householder QR cubic term still dominates every cell (100→1000
  params is ~1000× time for 100× Jacobian bytes). PERF-S02 (component
  split) and PERF-S04 (sparse solve) own that cost; this slice only removes
  the linear diagnostics duplicate beside it.
- Cross-call sharing (a `dof()` after `solve_detailed` at the same state,
  warm-start drag solves) is explicitly declined: it needs a persistent
  cache whose invalidation is not proven here (PERF-S05 scope, with its
  "do not reuse stale numeric factors" constraint).
- The `n == 0 || m == 0` and any identity-mismatch paths still evaluate
  fully fresh; the mismatch arm is unreachable in production by
  construction (capture and measurement bracket no mutation) and is covered
  by the degenerate tests plus the identity unit tests sharing the same
  fallback calls.
- Timing evidence is same-host sequential with pinned sources, not a
  quiet-room study: min/max bands overlap on the small cells and the 1000-
  param cells show single-sample excursions (e.g. baseline solved_1000
  solve max 4967→ candidate 6191). No timing assertion exists in any test.

## 8. Evidence files

- `docs/performance/evidence/perf-s06/counts-candidate.txt` — exact
  evaluation counts (Section 2).
- `docs/performance/evidence/perf-s06/summary-baseline-rep.json`,
  `summary-candidate-rep.json`, `summary-candidate-rep2-freshpkg.json` —
  retained-sample medians/min/max plus per-process medians.
- `docs/performance/evidence/perf-s06/run-baseline-rep.json`,
  `run-candidate-rep.json`, `run-candidate-rep2-freshpkg.json` — source
  head, working-tree hash, manifest hash, toolchain, build command, native
  binary and package file hashes.
- `docs/performance/evidence/perf-s06/criterion-baseline.txt`,
  `criterion-candidate.txt` — full criterion output for the 18 benches.
