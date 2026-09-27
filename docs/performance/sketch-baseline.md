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

## Fresh packaged-WASM evidence from `49099bc6` (`2.130.56`) — same-source native/WASM

Historical rows above (committed package `2.130.54` from `09cf1cd7`
against native source `895fc75d`) are kept separately labeled and are not
rewritten. This section adds representative same-source evidence: the
native worker and the freshly built/installed kernel package come from
the same recorded source (`49099bc6b4e7f1d2a0273e15fb120c021ce641e6`,
origin/main). Workload IDs, dimensions, expected outcomes, and solver
settings are unchanged (manifest sha256
`714f71ad1c09e4ff381994172598d594e1a86b1dd91de33aca4704175963c81e`,
identical to the historical manifest).

Source: `49099bc6b4e7f1d2a0273e15fb120c021ce641e6` (origin/main at task
start; Rust source unchanged by the harness-only changes in this slice —
new JS witnesses plus docs/smoke wiring — working-tree sha256
`5812d029ae6ea7f555eef2f9d3e1af9b64db88cb448d98983b1013f1e596d74f`
at run time).
Native worker: `crates/wasm/examples/sketch_baseline.rs` built with
`cargo build --locked --profile profiling -p remus-wasm --example
sketch_baseline --message-format=json --offline`
(rustc 1.96.0 `ac68faa20 2026-05-25`, cargo 1.96.0, node v24.14.0,
python 3.12.3, Linux 7.1.5 x86_64, AMD Ryzen 9 5900XT, 32 logical CPUs).
Native binary sha256
`7a5e524e076aa91b4afa5961b45ea3833c7b92deb1bb097aecf7438bdc108284`.
Manifest sha256 above; harness sha256 `run.py 7a7ee92f…`,
`wasm.cjs baa068d0…`, `sketch_baseline.rs 3b992144…` (full hashes in
`run.json`).
Kernel package `2.130.56` (fresh `cargo xtask wasm-build` from the same
source; matches active refresh `codex/wasm-refresh-49099bc6…` → `2.130.56`):
`remus_wasm_bg.wasm d4729644528601f2205dedd21670fdc4049e98aada921b6337fd9babdb7e5464`,
`remus_wasm_node.cjs d4e0f7b21634882dbd3d10ae5855545c59a19d99e2e2eace4ccefb465d33a914`,
`package.json c32b0af0df3df75ed255b4b715a2c8db9bdc398e1eb8505e41fa71378168e81e`.
Translator package `2.130.56` from the same build:
`remus_wasm_io_bg.wasm 0677102098aedea4a81febc2239879d5061fa68f5d322a26dcc75e0706f779be`,
`remus_wasm_io_node.cjs e90089cf522f2a785ce7c057393b5b053a1e725720f29a0b599e1e00cc329d37`.
Both packages validated by `cargo xtask wasm-build` (dual-target merge +
wasm-opt, smoke plus installed-tarball consumer checks pass).
Representative run directory:
`target/performance-sketch/20260927T020658.910192Z`
(3 processes × 5 retained samples + 1 warmup per process native — 15
retained per native cell; 1 process × 5 retained samples + 1 warmup for
WASM — 5 retained per WASM cell).
Bounded smoke directory:
`target/performance-sketch/20260927T020428.463948Z`
(1 process × 1 retained sample + 1 warmup; all smoke cells correctness-passed,
including the same three representative WASM cells).

Reproduce the recorded run from the witness commit
`787bb1732185ed3241616a9c7255a54ba3273bb5`, whose Rust source is
unchanged from `49099bc6b4e7f1d2a0273e15fb120c021ce641e6`. The
witness commit supplies `scripts/circular-pattern-packaged.mjs` and its
smoke-suite wiring; the source commit alone does not contain them.

```bash
git fetch origin pull/724/head
test "$(git rev-parse FETCH_HEAD)" = 787bb1732185ed3241616a9c7255a54ba3273bb5
git switch --detach 787bb1732185ed3241616a9c7255a54ba3273bb5
cargo xtask wasm-build
node scripts/test-wasm-smoke.mjs
node scripts/circular-pattern-packaged.mjs
python3 scripts/performance/sketch/run.py --offline --smoke
python3 scripts/performance/sketch/run.py --offline --case independent_solved_100 --case coupled_chain_100 --case drag_100
python3 -m unittest discover -s scripts/performance/sketch -p 'test_*.py' -v
cargo test -p remus-sketch --test gcs_perf_identity
```

Representative same-source correctness (all retained samples agree;
solver `tolerance 1e-10`, `max_iter 100`, drag 20 steps of
`(+0.5, +0.25)` on chain point 25; construction outside the timer):

| Case / mode | Params×Eqs | Expected | Native iters (rank/dof) | WASM iters (rank/dof) | Agreement |
| --- | ---: | --- | ---: | ---: | --- |
| independent_solved_100 solve / detailed | 100×100 | solved | 8 / 8 (100/0) | 8 / 8 (100/0) | exact |
| coupled_chain_100 solve / detailed | 100×100 | solved | 5 / 5 (100/0) | 5 / 5 (100/0) | exact |
| drag_100 solve (20 steps) / detailed | 100×100 | solved per step | 100 total (5/step) | 100 total (5/step) | exact |

Provisional timings from the noisy host (diagnostic only — no
performance conclusion; see limitations):

| Case / mode | Native median ms (15 samples) | WASM median ms (5 samples) |
| --- | ---: | ---: |
| independent_solved_100 solve / detailed | 4.20 / 4.83 | 4.96 / 5.48 |
| coupled_chain_100 solve / detailed | 2.63 / 3.14 | 3.20 / 4.03 |
| drag_100 solve (20 steps) / detailed | 56.28 / 65.54 | 60.08 / 72.07 |

Native process medians stay within ~10% of the cell median except one
`independent_solved_100/detailed` process at 5.11 vs 4.83 cell median;
WASM ranges span 4.82–6.90 (solved/solve) and 60.02–63.09 (drag/solve).
Full min/max plus per-process medians are in `summary.json`; no
p95/p99 is estimated (sample counts do not qualify tail independence).

Limitations stated honestly:

- The host was not quiet: concurrent profiling builds in sibling
  worktrees drove load average ~60 during the representative run
  (multiple `rustc --profile profiling` jobs). Wall times are therefore
  provisional and are not a performance baseline update; correctness and
  provenance are the claimed evidence, timing is reported as observed
  without a speedup or regression conclusion.
- Different builds, same solver: native (profiling release) and WASM
  (optimized wasm-pack + wasm-opt) timings are never compared as
  speedups, matching the historical section's discipline.
- Coverage is representative, not full: only the three 100-param
  `WASM_COVERAGE` cells ran through freshly installed packages; the
  10/1000/10000-param, redundant, and inconsistent rows ran natively
  only in the smoke/full harnesses or remain historical. Workload IDs,
  dimensions, expected outcomes, and solver settings were not changed.
- No timing assertion exists in any test; `gcs_perf_identity.rs` and
  `test_run.py` still gate identity/dimensions/classifications and
  collector failure paths only.

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
