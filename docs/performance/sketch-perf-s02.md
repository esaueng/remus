# PERF-S02 / B30: independent-component sketch solving

Owner row: PERF-S02 (component split before dense solving). Builds on
PERF-S01's workloads/runner, PERF-S03's solve-local workspaces and PERF-S06's
final-iterate capture; changes none of their contracts. No sparse-solver
dependency, no new sketch entities, no cross-call caching, no public solve
semantics or JS result-shape changes.

- Baseline source: `6c277fc79fcaea3eee9c1bf07106ced5250b1d23` (origin/main;
  clean worktree `/tmp/perf-s02-baseline`). `crates/sketch` is byte-identical
  between `6c277fc7` and the candidate branch base `c8075051` (verified empty
  `git diff` on `crates/sketch`), so the baseline native cells are
  sketch-source-pinned to the branch base.
- Candidate source: `20491ecc27497617e6c76b2dce827bb2122e3eb9` (this PR's code
  commit; bench/test/docs/package commits stack on top).
- Baseline package: committed `remus-wasm` `2026.1.6` (pre-S02 solver).
- Candidate package: `remus-wasm` `2026.1.7`, fresh `cargo xtask wasm-build`
  from the candidate source (validated: dual-target merge + wasm-opt, smoke
  plus installed-tarball consumer checks pass).
- Toolchain (both builds): rustc/cargo 1.96.0 (`ac68faa20 2026-05-25`), node
  v24.14.0, Python 3.12.3, Linux x86_64, AMD Ryzen 9 5900XT (32 logical).
  Native worker: profiling release (`opt-level 3`, debuginfo, no LTO).
  Manifest `scripts/performance/sketch/workloads.json` sha256 `714f71ad…`
  (identical for every run below; harness hashes identical too — see
  `run.json` in each run directory). Workload solver settings: tolerance
  `1e-10`, `max_iter` 100 (50 for inconsistent), 20 drag steps.

Reproduce:

```bash
cargo test -p remus-sketch
cargo bench -p remus-sketch --bench gcs_perf -- sketch/mixed_sizes
cargo bench -p remus-sketch --bench gcs_perf -- sketch/independent_solved_large
python3 scripts/performance/sketch/run.py --offline --processes 2 --samples 5 --warmup 1 \
  --case independent_solved_100 --case independent_solved_1000 \
  --case independent_under_100 --case independent_under_1000 \
  --case coupled_chain_1000 --case redundant_100 --case inconsistent_100 --case drag_100
node scripts/test-wasm-smoke.mjs
node scripts/performance/sketch/wasm.cjs ./crates/wasm/pkg independent_solved 100 solve 5 1
```

## 1. Current-state semantics (pinned before optimizing)

Unchanged from PERF-S06 (see `sketch-perf-s06.md` §1): `solve` publishes its
final iterate (a miss still moves geometry), `solve_detailed` measures at the
attempt then restores pre-solve geometry on a miss, per-constraint residuals
describe the attempt, `published_max_residual` the published state.
Convergence is NaN-propagating `max_abs_residual < tolerance` within
`max_iter` DogLeg steps; the trust region opens at
`clamp(max(1, 0.1·|p|), …, 1e4)` with the 0.75-double / 0.25-quarter rho rule
and the `1e-15·(1+|p|)` small-step exit; rank is Householder-QR-with-pivoting
`#{|Rii| > 1e-10·|R00|}`, `dof = n − rank`, classified Unsatisfied >
UnderConstrained > Redundant > Solved.

## 2. Design: structural components, per-component loops, global policies

New module `crates/sketch/src/gcs/components.rs`; solver exposes nothing new
except two QR readers (`leading_magnitude`, `rank_absolute`) and
`dof::analyze_blocks`; `system.rs` routes `solve`/`solve_detailed`/`dof`
through one decomposition.

- **Structural, not numerical.** A constraint names the free parameters
  reachable through its entities with coordinate precision (`FixX` → X only,
  `Horizontal` → endpoint Y only, `CircleRadius` → radius only, arcs → full
  triple). Fixed points contribute nothing. Degenerate zero-gradient rows
  (e.g. `Distance(p, p)`) keep their edge. The match is exhaustive with no
  wildcard: a new `Constraint` variant fails to compile until its references
  are declared.
- **Deterministic.** Union-find over the `[params, constraints]` layout in
  arena order, `BTreeMap` grouping, components sorted by smallest parameter
  index, constraints in arena order. Pinned constraints (no free parameters)
  merge into one trailing group; isolated free parameters into one
  equation-free group.
- **Per-component DogLeg loops, deliberately.** Each block gets the full
  `max_iterations` and its own trust region (see `components.rs` for the
  order-dependence and scale-coupling analysis of the alternatives). Reported
  `iterations` is the max over blocks (parallel critical path); evaluation
  counts sum. `converged` is the AND; `max_residual` the NaN-propagating max —
  identical to the max over all equations. Whole-call rollback restores every
  block when any fails.
- **Global rank policy kept.** Blocks factorize independently; pivots count
  against one absolute threshold from the global leading magnitude (the max
  over blocks, equal to a global factorization's `|R00|`). Per-block relative
  thresholds were rejected: they silently re-rank mixed-scale systems.
- **Single-component systems run the dense loop bit-for-bit** (small-system
  path). Diagnostics on multi-component systems build per-block Jacobians
  only — no giant dense matrix — and slice the assembled capture (S06
  composition: per-block finals concatenated in arena order through the same
  identity check).

## 3. Correctness evidence

- `gcs/components/tests.rs` (15 tests): disjoint/split/join behavior, fixed
  anchors sharing nothing, line/circle/arc coupling (endpoints, shared
  centers, internal ties), degenerate zero-gradient edges kept, pinned/free
  groups, empty shapes, join/split roundtrip, determinism, exact free-parameter
  sets for all 26 variants, and 1000-parameter block-dimension/byte pins
  (500× 2×2 blocks = 16,000 Jacobian bytes vs 8,000,000 dense; chain stays one
  1000×1000 block; mixed is one 500×500 plus 250× 2×2).
- `gcs/system/agreement_tests.rs` (12 tests): dense loop (private
  `solve_dense` + dense DOF/residuals assembled by a `dense_detailed` oracle)
  vs public entry points. Single-component chains assert bitwise equality
  (iterations, residuals, published geometry). Multi-component systems assert
  exact agreement on converged/classification/dof/rank/dimensions/redundant/
  rolled_back plus analytic geometry oracles (pair distance 5, `y = 4`, chain
  grid, isolated point unmoved) across solved, underconstrained, redundant,
  inconsistent-with-rollback, mixed 1.0/1e3 scales, near-singular degenerate
  rows, zero iterations, pinned+free mixes, edits between solves, and
  line/circle/arc-coupled systems.
- `gcs/final_eval/tests.rs`: the S06 shared-vs-fresh oracle now runs against
  the component path on both sides and still agrees bitwise on every
  diagnostic and the published params; count gates assert one analysis
  Jacobian+QR per factorizable block (2/10/5/1 by workload, 0 when
  degenerate).
- Full position: `cargo test -p remus-sketch` 170 lib + 9 identity + 5
  workspace-reuse + 1 doc-test green; `cargo test -p remus-operations --lib`
  1224 green; `cargo test -p remus-wasm --lib` 615 green;
  `cargo clippy -p remus-sketch --all-targets -- -D warnings` clean;
  `cargo fmt` applied; `check-boundaries`, `check-det-hash`, `check-doc-paths`
  pass; every sketch-runner sample below passed its
  identity/dimension/outcome gates natively and through packaged WASM.

## 4. Measurements: same-host sequential baseline vs candidate

Native profiling-release runner, 2 processes × 5 retained samples + 1 warmup
(10 retained per cell), same host back-to-back. Baseline run directory
`target/performance-sketch/20260929T025510.341381Z` (in `/tmp`,
`6c277fc7`, native binary sha256 `bb330669…`); candidate run directory
`target/performance-sketch/20260929T024434.687984Z` (this worktree,
`20491ecc` + bench/test-only uncommitted additions that do not enter the
worker binary, native binary sha256 `20f2bb30…`). All cells below passed
every per-sample gate on both runs.

| Case / mode | Baseline median | Candidate median | Factor |
| --- | ---: | ---: | ---: |
| under_100 solve / detailed | 1.467 / 1.612 ms | 0.266 / 0.349 ms | 5.5x / 4.6x |
| under_1000 solve / detailed | 1441 / 1654 ms | 3.07 / 3.37 ms | 469x / 491x |
| solved_100 solve / detailed | 4.08 / 4.37 ms | 0.322 / 0.349 ms | 12.7x / 12.5x |
| solved_1000 solve / detailed | 5121 / 5740 ms | 3.70 / 4.08 ms | 1384x / 1407x |
| chain_1000 solve / detailed (control) | 3198 / 3815 ms | 3045 / 3667 ms | 1.05x / 1.04x |
| redundant_100 solve / detailed | 6.82 / 7.59 ms | 0.395 / 0.442 ms | 17.3x / 17.2x |
| inconsistent_100 solve / detailed | 9.08 / 9.24 ms | 0.284 / 0.322 ms | 32x / 29x |
| drag_100 solve / detailed (control) | (S01 52.2 / 60.4) | 51.0 / 58.9 ms | unchanged path |

Rank/DOF/dimensions/classification agree exactly on every cell (e.g.
solved_1000: rank 1000, dof 0, solved; inconsistent_100: rank 50, dof 50,
unsatisfied, max residual 0.54 on both). Iteration counts differ honestly —
per-component trust regions traverse different iterates (solved_1000: 11 vs
8; solved_100: 9 vs 8; under_1000: 10 vs 8; inconsistent_100: 36 vs 49 with
the same refusal verdict; chain_1000: 5 == 5; redundant_100: 8 == 8).
Geometric oracles agree (~1e-10 worst error both; chain grid exact both).
Peak RSS: solved_1000 21.7 → 3.2 MiB; under_1000 12.1 → 3.1 MiB; chain_1000
21.5 → 21.7 MiB (control, same dense allocation).

Criterion release profile (same host, two adjacent runs; second run under
load average ~48, so ranges are reported honestly and the runner table above
stays primary):

| Bench (estimate) | Run 1 | Run 2 (loaded) |
| --- | ---: | ---: |
| mixed_100 solve / detailed (50-chain + 25 pairs) | 0.60 / 0.67 ms | 1.41 / 1.38 ms |
| mixed_1000 solve / detailed (500-chain + 250 pairs) | 0.39 s / — | 0.63–0.64 / 0.78–0.80 s |
| independent_solved_2000 solve / detailed | 7.3 / 8.1 ms | 15.3 / 9.9 ms |
| independent_solved_5000 solve / detailed | 19.7 / 21.2 ms | 39.3 / 48.8 ms |
| independent_solved_10000 solve / detailed | 40.8 / 43.3 ms | 92.3 / 100.8 ms |

Raw outputs: `evidence/perf-s02/criterion-mixed.txt` (run 2; run-1
mixed_100 figures from the session log) and
`evidence/perf-s02/criterion-large.txt` (run 2; run-1 figures from the
session log). Mixed_1000 is dominated by its single 500×500 chain block by
design — the pairs beside it cost microseconds.

The 10000-parameter rows the S01 runner refuses (800 MB dense Jacobian) solve
in ~41 ms with linear block scaling (2.5x params → ~2.7x time).

Packaged WASM, candidate package `2026.1.7` built from the candidate source,
existing `gcs*` bindings (no binding changes in this PR), `wasm.cjs` cells
(all validation-passed, 5 retained samples each):

| Case / mode | Fresh pkg time | Fresh pkg iters | Native cand iters |
| --- | ---: | ---: | ---: |
| solved_100 solve / detailed | ~1.5 / ~5.0 ms | 9 / 9 | 9 / 9 |
| chain_100 solve / detailed | ~5.4 / ~6–12 ms | 5 / 5 | 5 / 5 |
| drag_100 solve (20 steps) / detailed | ~112–115 / ~130–135 ms | 100 / 100 | 100 / 100 |

The pre-S02 committed package reports 8 iterations on solved_100 (dense) in
~4.8 ms; the fresh package reports 9 (component path), matching native
exactly — the iteration fingerprint proves the shipped artifact carries the
new solver. Different builds, same solver: no cross-runtime speedup is
claimed, matching the S01 discipline.

## 5. Allocations: structural accounting (no instrumented allocator)

As in S01/S06, allocations are accounted structurally — every eliminated heap
object named (this build has no allocation-counting harness):

Per converged multi-component `solve` vs the dense loop it replaces, at
1000×1000 (solved_1000): the 8,000,000-byte global Jacobian plus its
same-sized QR working copy and `n`/`m`-sized step vectors collapse to 500
independent 2×2 Jacobians (32 bytes each, 16,000 total) with proportionally
tiny workspaces — the measured 21.7 → 3.2 MiB peak RSS is the process-wide
effect including setup. Per-component subset snapshots visit only the block's
own entities per evaluation (2–3 points here), so per-evaluation work scales
with block size rather than document size; the equation-free and pinned
groups perform no factorization at all. Still allocated per call: one fresh
Jacobian plus one QR factorization per factorizable block for the rank
analysis (never shared), the per-constraint report itself, and the assembled
capture (`(n + m) * 8` bytes plus row layout — same shape as S06's).

## 6. Limitations and what stays open

- One large connected component keeps the dense cubic (chain_1000 is
  unchanged by design — the control cells prove it). Banded/sparse solving of
  genuinely coupled systems is PERF-S04, which now composes per block.
- Iteration counts on independent systems shift by small amounts (11 vs 8 on
  solved_1000) because trust regions are per block; the contracted outcomes do
  not. Any caller asserting exact dense iteration counts on multi-component
  systems must expect the component counts (9 instead of 8 on solved_100).
- The shared runner's 256 MiB upfront dense-Jacobian budget still refuses the
  10000-parameter rows: the runner estimates `m*n*8` before solving and does
  not know about blocks. Criterion covers 10k instead; teaching the runner a
  component-aware budget is follow-up harness work, explicitly not claimed
  here.
- Cross-call sharing (warm-start drag solves, persistent block plans) is
  explicitly declined: it needs the PERF-S05 invalidation design. Decomposition
  itself rebuilds per call (linear in references) with nothing retained.
- Timing evidence is same-host sequential with pinned sources (10 retained
  native samples per cell, 5 per WASM cell), not a quiet-room study; min/max
  bands are in the run `summary.json` files. No timing assertion exists in any
  test.

## 7. Evidence files

- Committed: `evidence/perf-s02/summary-{candidate,baseline}.json`,
  `run-{candidate,baseline}.json`, `samples-{candidate,baseline}.jsonl`
  (copies of the run artifacts, including per-sample iterations, rank, DOF,
  residuals, classifications, oracle errors and peak RSS), plus
  `criterion-{mixed,large}.txt` (run-2 raw outputs).
- Run directories (ephemeral, never overwritten): baseline
  `/tmp/perf-s02-baseline/target/performance-sketch/20260929T025510.341381Z`
  (`run.json`: source `6c277fc7` clean, manifest `714f71ad…`, harness hashes,
  native binary `bb330669…`); candidate
  `target/performance-sketch/20260929T024434.687984Z` (`run.json`: source
  `20491ecc` + bench/test-only additions, native binary `20f2bb30…`,
  committed package `2026.1.6` for the old-solver WASM cells).
- The S06 evidence files are untouched historical record.
- Fresh package `2026.1.7` cell outputs: `wasm.cjs` JSON samples quoted in §4
  (validation `passed`, iterations fingerprint 9/5/100).
