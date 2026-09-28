# B29 checkpoint log — PERF-T02 / PERF-T03

Branch `codex/b29-perf-t02-t03` from `origin/main` (`bf46fdc0`).
Worktree `/tmp/opencode/remus-b29-t02-t03`. This file is the durable log;
each entry is appended when the step completes, never rewritten.

## 2026-09-28 — setup + ownership

- Fetched `origin/main` (`bf46fdc0`). Local `main` had diverged; work proceeds
  on a fresh worktree at the fetched head, branch `codex/b29-perf-t02-t03`.
- Ownership check: open PRs (#788 sketch, #787 shell, #786 render, #785 io)
  do not touch topology transaction/arena internals. No active owner for
  PERF-T02/T03; roadmap rows are `Proposed`. Proceeding.
- Read: `AGENTS.md`, `docs/design/perf-t01-savepoints.md`,
  `docs/performance/perf-t01-savepoints.md`, skills solid-verification,
  testing, profiling, debugging-doctrine.

## 2026-09-28 — contract written (milestone 1)

- `docs/design/perf-t02-t03-transactions.md`: semantic contract before any
  implementation. Enumerates every state component, every mutable access
  route, nesting/failure/restore semantics, stale-handle guarantees, the
  undo-vs-COW selection criteria, the append-only guard design, and the
  equivalence/benchmark qualification plan. Frozen TopologyError (no new
  variant) to avoid ripple into the WASM error match.

## 2026-09-28 — session interrupt + recovery

- Session was interrupted during the baseline re-run. Main checkout
  (`/home/peter/code/Remus`) has since moved to `codex/b74-anisotropic-transform-contract`
  (transform scope, no overlap with this topology-transaction work).
- Recovered: worktree `codex/b29-perf-t02-t03` at `bf46fdc0` intact; contract,
  checkpoint log, and harness (with the 1-box fix) all present; first-run
  output survived in `/tmp/txn_baseline.jsonl`.

## 2026-09-28 — baseline measured (milestone 2)

- Harness `crates/topology/examples/txn_baseline.rs` (release, rustc 1.96.0,
  11 in-process reps, median). One `run_transacted` scope per op = one full
  clone per op by construction.
- Fixed local edit (vertex + face attribute + journal, then rollback):
  0.003ms (1 box) -> 0.558ms (100) -> 6.853ms (1000). Commit variant:
  0.002 -> 0.076 -> 0.797ms. Single box build in transaction:
  0.004 -> 0.558 -> 6.427ms. 150 sequential box builds: 10.6 -> 21.8 -> 137.8ms.
- NURBS doc (100 boxes + 24 48x48 faces, 2.59MB est. payload, 6760 slots):
  edit rollback 0.372ms, commit 0.177ms, box build 0.190ms — cost follows
  payload bytes more closely than slot count.
- Retained-checkpoint cells show an unexplained but repeatable ~3-4x faster
  `edit_rollback` vs no-checkpoint (0.147 vs 0.558 at 100 boxes; 2.137 vs
  6.853 at 1000). Same-harness candidate comparison controls for it; the
  anomaly and its disappearance/persistence will be reported with candidate
  numbers, not explained away.
- Evidence: `docs/performance/evidence/2026-09-28/baseline.jsonl` +
  `baseline-provenance.json` (source `bf46fdc0`).

## 2026-09-28 — candidate measured vs baseline (milestones 2+4 intersect)

- Same-harness comparison (`TXN_IMPL=full-snapshot|undo`, 11 in-process
  medians, release, rustc 1.96.0). Evidence: `2026-09-28/baseline.jsonl`,
  `candidate-undo.jsonl`, `provenance.json` (both at `bf46fdc0` + harness).
- Empty transaction (`txn_empty`): baseline 2.7-3.9ms at 1000 boxes (a full
  clone even when the closure does nothing); candidate 0.000ms at all sizes.
- Fixed local edit with real writes, rollback: 7.47ms -> 0.002ms at 1000
  boxes; commit: 1.1-3.6ms -> 0.001-0.002ms. Flat across 1/100/1000.
- 150 sequential transactional box builds: 136-161ms -> 2.35-2.6ms at 1000
  pre-existing boxes (~60x); 24.5ms -> 1.2ms at 100 boxes.
- Decomposition cells (`box_naked` vs `box_build`) show the residual
  single-box cost is loop/coedge arena growth during construction
  (identical with and without a transaction; PERF-T07 territory), not
  transaction cost. Single-box absolute times carry realloc/rehash lottery
  noise; the amortized and empty-scope cells are the robust signal.
- Earlier `cp1 faster than cp0` wobble did not reproduce: it was shared-host
  noise. Retained checkpoints change nothing on either implementation here
  (native path has no Rc sharing; the WASM checkpoint copy remains).

## 2026-09-28 — undo implemented, COW compared, selection recorded

- `undo_log.rs` (child module of `Topology`): per-slot old-value records,
  per-key registry/attribute records, journal created-pair tracking with
  high-water preservation, O(1) marks, lazy drop-commit via liveness tokens,
  same-lineage foreign-restore truncation, append-only guard with slot marks.
- `cow_prototype.rs` (cfg(test) only): scattered writes amplify COW 42.7x
  (2,048,000B vs 48,000B); append metadata favors COW 133x (1,152B vs
  153,600B, mooted by the append-only path); coresident 48x48 net amplifies
  COW 427x (20,480B vs 48B); escaped-`&mut` barrier bypass demonstrated.
- Selection: undo ships (scattered + coresidency wins, and the only
  mechanism covering `&mut`-escape APIs without signature changes).
  Production keeps only undo.
- Real bug found by the new guard tests: refusals did not mark the guard
  tripped, so fallback never triggered. Fixed (guards mark tripped before
  returning the trip error).

## 2026-09-28 — equivalence + append qualification

- `savepoint_tests.rs` fault matrix extended 6 -> 14 stages (edge trim,
  surface swap, wire reorder, winding + pcurve removal, boundary
  re-derivation, compound/compsolid alloc, attribute clear, journal
  evolution with fresh ordinals, retirement, barrier); legacy-vs-undo
  oracle passes every prefix plus nesting/validation/stale-handle cases.
- New 16-kind append-guard matrix: every pre-existing write kind trips
  before landing and matches the full-path result structurally (counts,
  pre-existing values, boundary structure, pcurves, attributes, journal
  kinds); journal appends stay fast-path. New: typed-trip unit test,
  failure-atomicity, nested append scopes, compound/compsolid + arena-level
  rewind tests.
- `make_box` wrapped in `run_append_only` (behavior-preserving; mid-build
  failures now atomic instead of partial). New
  `operations/tests/qualify_append_only_box.rs`: 150 builds x 3 doc sizes
  all `AppendOnly` with exact 64-slots-per-box growth; per-stage injection;
  fallback equivalence; invalid-dims atomicity.
- Integration found a second real defect: `UndoLog::next_seq` leaked into
  `Topology`'s `Debug`, breaking whole-topology snapshot tests. Fixed with
  constant coordination rendering per the contract; the catching test
  (`qualify_curved_boss_heal`) passes unmodified.

## 2026-09-28 — environment: disk full (twice), recovered

- Shared sandbox disk hit 100% during the workspace test build (this
  worktree's `target/` ~94G with full debuginfo test binaries).
- Recovered by `cargo clean` on this worktree only (no other checkouts
  touched) and re-ran the remainder with `CARGO_INCREMENTAL=0
  RUSTFLAGS="-C debuginfo=0"` (env-only). All suites green; see evidence
  report for the rollup and the two disclosed caveats (allocator lottery
  in single-box absolutes; no browser-wide claim).
