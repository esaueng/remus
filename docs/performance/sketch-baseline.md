# PERF-S01 / B30: sketch solve and drag performance baseline

Owner row: PERF-S01 (measurement only). No solver math, convergence
tolerances, storage layout, or numerical dependency was changed in this PR.
B30 remains open (umbrella); PERF-S02–S06 remain proposed.

Source: `895fc75d5a82bfaeebe6f63ad259c1e8228aa82a` (origin/main at task start).
Native worker: `crates/wasm/examples/sketch_baseline.rs` built with
`--locked --profile profiling -p remus-wasm --example sketch_baseline`
(rustc 1.96.0, cargo 1.96.0, Linux x86_64, AMD Ryzen 9 5900XT).
Manifest: `scripts/performance/sketch/workloads.json`
(sha256 `714f71ad1c09e4ff…`; full hash in `run.json`).
Native binary sha256 `2cdad38a5816958d…` (full hash in `run.json`).
Full run directory: `target/performance-sketch/20260926T072346.692025Z`
(3 processes × 5 retained samples + 1 warmup per process; 15 retained
samples per native cell; 1 process × 5 retained samples for WASM).
Committed WASM: package `2.130.54` (last refresh commit `e25d0913`,
from `09cf1cd7`); package file hashes in `run.json`. The package predates
this PR's native source (which adds only benchmark harnesses), so no
cross-runtime speedup is inferred.

Reproduce:

```bash
python3 scripts/performance/sketch/run.py --offline --smoke   # bounded smoke
python3 scripts/performance/sketch/run.py --offline           # full baseline
python3 -m unittest discover -s scripts/performance/sketch -p 'test_*.py' -v
cargo test -p remus-sketch --test gcs_perf_identity
cargo bench -p remus-sketch --bench gcs_perf
```

## Workloads (pinned, no RNG)

Closed-form grids; drag deltas fixed (+0.5, +0.25) on chain point 25.
`solve` tolerance `1e-10`, `max_iter` 100 (50 for inconsistent), 20 drag steps.
Construction and geometric oracles stay outside the timer; only
`solve` / `solve_detailed` (or the 20 warm re-solves) are timed.
Solved and refused workloads are never compared as equivalent work.

| Case | Params | Equations | Expected outcome |
| --- | ---: | ---: | --- |
| independent_under 10/100/1000 | 10/100/1000 | 5/50/500 | underConstrained |
| independent_under 10000 | 10000 | 5000 | resource_refused (400 MB Jacobian) |
| independent_solved 10/100/1000 | 10/100/1000 | 10/100/1000 | solved |
| independent_solved 10000 | 10000 | 10000 | resource_refused (800 MB) |
| coupled_chain 10/100/1000 | 10/100/1000 | 10/100/1000 | solved (sparse band, dense solve) |
| coupled_chain 10000 | 10000 | 10000 | resource_refused (800 MB) |
| redundant 100/1000 | 100/1000 | 150/1500 | redundant (converged) |
| inconsistent 100/1000 | 100/1000 | 51/501 | unsatisfied (50/37 iters, then refusal) |
| drag_100 (20 warm steps) | 100 | 100 | solved per step |

The 256 MiB upfront Jacobian budget (`m*n*8`) refuses the three
10000-parameter rows before allocating; the runner records them as
`resource_refused` rows with dimensions, never drops them.

## Native results (profiling release, 15 retained samples per cell)

Median of retained samples; min/max in `summary.json`. Iterations,
rank/DOF/residuals below are from one retained sample (all samples in a
cell agree on outcome and dimensions; iteration counts are stable).

| Case / mode | Median ms | Iters | Params×Eqs (rank/dof) | Max residual | Jacobian bytes |
| --- | ---: | ---: | --- | --- | ---: |
| under_10 solve / detailed | 0.027 / 0.030 | 5 / 5 | 10×5 (5/5) | 3.1e-11 | 400 |
| under_100 solve / detailed | 1.89 / 2.07 | 8 / 8 | 100×50 (50/50) | 3.9e-15 | 40,000 |
| under_1000 solve / detailed | 1811 / 2004 | 8 / 8 | 1000×500 (500/500) | 2.8e-13 | 4,000,000 |
| solved_10 solve / detailed | 0.039 / 0.042 | 6 / 6 | 10×10 (10/0) | 5.7e-13 | 800 |
| solved_100 solve / detailed | 4.91 / 5.63 | 8 / 8 | 100×100 (100/0) | 4.1e-11 | 80,000 |
| solved_1000 solve / detailed | 5882 / 6357 | 8 / 8 | 1000×1000 (1000/0) | 1.1e-12 | 8,000,000 |
| chain_10 solve / detailed | 0.036 / 0.041 | 5 / 5 | 10×10 (10/0) | 4.5e-26 | 800 |
| chain_100 solve / detailed | 2.86 / 3.34 | 5 / 5 | 100×100 (100/0) | 6.7e-25 | 80,000 |
| chain_1000 solve / detailed | 3504 / 4241 | 5 / 5 | 1000×1000 (1000/0) | 3.4e-23 | 8,000,000 |
| redundant_100 solve / detailed | 7.40 / 8.16 | 8 / 8 | 100×150 (100/0) | 1.8e-11 | 120,000 |
| redundant_1000 solve / detailed | 13030 / 14549 | 8 / 8 | 1000×1500 (1000/0) | 1.4e-13 | 12,000,000 |
| inconsistent_100 solve / detailed | 10.20 / 10.24 | 49 / 49 | 100×51 (50/50) | 0.54 | 40,800 |
| inconsistent_1000 solve / detailed | 6930 / 6978 | 37 / 37 | 1000×501 (500/500) | 0.54 | 4,008,000 |
| drag_100 solve (20 steps) / detailed | 52.2 / 60.4 | 100 total (5/step) | 100×100 (100/0) | — (per-step converged) | 80,000 |
| 10000 rows (all, both modes) | refused | — | dims as manifest | — | 400–800 MB over budget |

Per-iteration cost (median / iters): ~0.2–1.1 ms at 100 params,
~230–1820 ms at 1000 params. The 100→1000 step multiplies Jacobian
bytes 100× but wall time ~1000× — the dense Householder QR cubic term,
not construction or validation (both outside the timer).

Cold vs warm: one drag re-solve step costs ~2.6 ms (52.2/20), vs cold
chain_100 solve 2.86 ms. Warm re-solves cost the same as cold solves;
no warm-start reuse exists (expected — none is implemented).

## Packaged-WASM results (separate provenance, no speedup claim)

Fresh runs against committed package `2.130.54` through the existing
`gcs*` bindings (`gcsNew/AddPoint/AddLine/AddConstraint/Solve/
SolveDetailed/Dof/SetPoint/PointPosition`); 5 retained samples per cell.

| Case / mode | WASM median ms | Native median ms (same fixture) |
| --- | ---: | ---: |
| independent_solved_100 solve / detailed | 4.67 / 5.19 | 4.91 / 5.63 |
| coupled_chain_100 solve / detailed | 3.08 / 3.66 | 2.86 / 3.34 |
| drag_100 solve (20 steps) / detailed | 56.3 / 66.1 | 52.2 / 60.4 |

Outcomes, dimensions, iteration counts, and classifications agree
exactly with native. Wall times agree within run-to-run noise on this
host; no cross-runtime conclusion is drawn (different builds, same
solver).

## Top measured costs

1. **Dense QR per iteration dominates everything.** At 1000 params a
   single DogLeg iteration costs 0.2–1.8 s (8 MB Jacobian, 5–8
   iterations per solve). Scaling from 100→1000 params is ~1000× in
   time for 100× in Jacobian bytes. This is the cost PERF-S02 (component
   split) and PERF-S04 (sparse solve) exist to address.
2. **Independent components pay the dense price for block-diagonal
   work.** `independent_solved_1000` (500 disjoint 2-param pairs) costs
   5.9 s in one 1000×1000 dense QR per iteration; per-component QR
   would be trivial. `independent_under_1000` (500×1000 rectangular)
   costs 1.8 s for the same reason. The coupled chain (genuinely
   coupled, banded-sparse) costs 3.5 s through the same dense path —
   the solver cannot tell the two structures apart.
3. **`solve_detailed` adds 8–21% over `solve`.** Chain_1000: +21%
   (3504→4241 ms); solved_1000: +8%; drag_100: +16%. The delta is the
   extra Jacobian build + QR inside `dof()` plus the residual pass —
   the duplicate-final-iterate work PERF-S06 names.
4. **Refusals burn `max_iter` dense iterations.** Inconsistent_1000
   runs 37 iterations (6.9 s) to report `unsatisfied`; inconsistent_100
   runs 49 iterations (10 ms, 2× its solved twin). Refusal cost scales
   with the iteration budget, not with solution quality.
5. **Redundant rows are pure overhead.** Redundant_1000 carries 1500
   equations at rank 1000 and is the single most expensive row
   (13.0/14.5 s) — 500 dependent rows add QR rows with zero rank.

## Next optimization proposal (measurement-backed)

**Do PERF-S02 next: split the constraint graph into independent
connected components before dense solving.** The baseline proves the
prize: `independent_solved_1000` spends 5.9 s per solve on a 1000×1000
dense factorization of 500 disconnected 2-parameter blocks. Component
time would follow block sizes (500 × microseconds), with identical
rank/DOF/classification per the existing `dof`/`classify` contracts.
Acceptance stays in this harness: same workload IDs, same expected
outcomes/dimensions, wall-time distributions compared run-to-run, no
timing gates in tests. PERF-S06 (share the final Jacobian between
`solve_detailed`/`dof`/diagnostics) is the second candidate (+8–21%
on every detailed call), but S02's measured waste is an order of
magnitude larger.

## Counters: what is and is not measured

- Reported per sample: wall time (ns), iterations, num_params,
  num_equations, rank, dof, max_residual (and published variant for
  detailed), classification, redundant/rolled_back flags, deterministic
  Jacobian/residual/param byte sizes (`m*n*8`, `m*8`, `n*8`), oracle
  error, Linux peak RSS (`VmHWM`, process-wide including setup).
- **Not available in this build and stated honestly:** per-iteration
  global allocation count/bytes and allocator peak live bytes (no
  instrumented allocator; the numbers above are deterministic buffer
  sizes, not allocator traffic); WASM linear-memory high-water
  (reported process timings only); p95/p99 (sample counts do not
  qualify tail independence — min/median/max plus per-process medians
  only).
- No timing assertion exists in any test. `gcs_perf_identity.rs` gates
  construction counts, dimensions, classifications, oracles,
  determinism, and the solve-vs-detailed rollback contract at 10/100
  params; 1000-param construction counts are gated there while
  1000-param solve outcomes are validated per sample by this runner in
  the profiling build. `test_run.py` gates the collector's failure
  paths (missing/duplicate samples, wrong identity, bad durations,
  refusal shapes).
