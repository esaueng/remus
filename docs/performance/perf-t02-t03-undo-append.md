# PERF-T02 / PERF-T03: mutation-local undo and append-only construction

Production implementation: mutation-local undo records in `remus-topology`
(`undo_log.rs`), with a guarded append-only fast path (`run_append_only`)
integrated into native `make_box`. Chunk/page copy-on-write was prototyped
narrowly, measured, and rejected; its prototype is retained test-only in
`cow_prototype.rs`.

Baseline: `bf46fdc0` (fork point). Candidate: this branch. Both measured with
the identical harness `crates/topology/examples/txn_baseline.rs` (release,
rustc 1.96.0, 11 in-process medians per cell). Raw samples:
`evidence/2026-09-28/baseline.jsonl`, `evidence/2026-09-28/candidate-undo.jsonl`,
`evidence/2026-09-28/provenance.json`. One `run_transacted` scope per
operation on both sides; the baseline scope clones the whole document once,
the candidate records only touched state.

## Measurement

| Workload, 1 / 100 / 1000 unrelated boxes | Baseline ms | Candidate ms |
| --- | ---: | ---: |
| Fixed local edit, rollback | 0.003 / 0.473 / 7.470 | 0.000 / 0.001 / 0.002 |
| Fixed local edit, commit | 0.002 / 0.241 / 1.118 | 0.000 / 0.001 / 0.002 |
| Empty transaction (no-op closure) | 0.001 / 0.081 / 2.746 | 0.000 / 0.000 / 0.000 |
| Single box build in transaction | 0.004 / 0.372 / 1.333 | 0.005 / 0.137 / 1.307 |
| 150 sequential box builds | 11.6 / 24.5 / 161.3 | 0.74 / 1.20 / 2.35 |
| NURBS doc (100 boxes + 24 48x48 faces, 2.6 MB), edit rollback | — / 0.404 / — | — / 0.001 / — |
| NURBS doc, single box build | — / 0.204 / — | — / 0.013 / — |

(cp0/cp1 cells agree within noise on both implementations; retained native
clones change nothing here. The earlier `cp1 faster` wobble did not
reproduce and is recorded as shared-host noise.)

Reading the table honestly:

- The transaction mechanism itself is O(1) capture/commit/rewind
  (`txn_empty`: 2.7 ms → 0.000 ms at 1000 boxes — the baseline pays a full
  clone even when the closure does nothing) and O(touched) for real edits
  (fixed edit flat at ~1–2 µs across all sizes and payloads).
- The 150-box headline (161 → 2.4 ms, ~65x at 1000 boxes) is the removed
  per-scope clones.
- The residual single-box cost is construction, not transaction:
  `box_naked` (no scope) and `box_build` agree within noise on the
  candidate (1.290 vs 1.307 ms at 1000 boxes), so the transaction delta is
  ~0. The residual scales with loop/coedge arena growth during construction
  (those arenas are outside `reserve`'s six) — PERF-T07 territory, disclosed
  as remaining cost, not claimed here.
- Single-box absolutes carry realloc/rehash lottery noise (e.g. baseline
  `box_naked` 0.100 vs 1.346 ms across checkpoint cells); the empty-scope,
  fixed-edit, and amortized cells are the robust signal.

## Storage-mechanism selection (undo vs chunk/page COW)

Narrow prototypes over identical scripted workloads, exact byte counts
asserted in `cow_prototype.rs`:

| Workload | COW copied | Undo recorded |
| --- | ---: | ---: |
| 1000 scattered single-slot writes (64-slot pages) | 2,048,000 B (whole pages) | 48,000 B (touched slots) |
| 150 x 64-slot clustered appends | 1,152 B (one tail page) | 153,600 B (per-alloc identity) |
| One small write sharing a page with a 48x48 net | 20,480 B (whole page) | 48 B (the slot) |

Plus the decisive API finding, demonstrated in-test: any write path that
skips the exclusivity check aliases the snapshot, and every
`Topology::*_mut` accessor hands out `&mut T` whose later writes perform no
arena-level check. Covering the audited routes under COW would require
changing every accessor's return type in every crate (outside this slice's
scope); undo captures before escape with no signature change.

Selection: undo ships. COW's sole win (append metadata) is mooted by the
append-only path, which needs neither mechanism durably. Production keeps
only undo; COW stays as the documented test-only comparison.

## Fault-injection evidence

- The pre-existing legacy-vs-new oracle harness (`savepoint_tests.rs`)
  compares the undo implementation against explicit clone-and-restore for
  every fault prefix, propagated/outer failures, validation veto, repeated
  failure, stale handles, and journal-gap checks — extended in this slice
  from 6 to 14 mutation stages (edge trim, face surface swap, wire reorder,
  winding + pcurve removal, boundary re-derivation, compound/compsolid
  allocation, attribute clear, journal evolution with fresh ordinals,
  retirement, barrier). All pass.
- New: a 16-kind append-only guard matrix proves every pre-existing write
  kind trips before landing, falls back, and matches the full-path result
  structurally (counts, pre-existing values, boundary structure, pcurves,
  attributes, journal kinds); journal appends provably stay on the fast
  path; direct guard tests prove the typed trip precedes any write.
- New: compound/compsolid allocation rollback, arena-level slot rewind,
  nested append scopes, and per-stage construction failure injection
  (operations `qualify_append_only_box`, including the 150-box run: all
  `AppendOnly`, exact slot growth of 64 per box, stale handles, untouched
  pre-existing solids).
- Integration found one real defect: coordination state (`next_seq`)
  leaked into `Topology`'s `Debug`, breaking whole-topology snapshot tests
  (`qualify_curved_boss_heal`, and the same pattern in algo). Fixed by
  rendering coordination constantly, as the contract requires; the test
  that caught it now passes unmodified.

## Qualification boundary

- Topology (386), operations lib (1224), wasm lib (615, incl. batch and
  transaction contracts), algo lib (419), io lib (406) plus the io,
  operations-integration, and L0–L2 suites: green (see the PR body for the
  per-crate rollup and the two environment notes below).
- `cargo clippy --workspace --all-targets`, `check-boundaries.sh`,
  `check-det-hash.sh`, `check-doc-paths.sh`: green.
- Per-item batch commit semantics preserved: batch dispatch is untouched;
  the boolean batch arms pick up undo through the unchanged
  `RollbackSnapshot` signatures, other arms keep their clone policy.
- Environment notes: (1) the sandbox disk filled mid-run (shared host);
  recovered by clearing this worktree's disposable `target/` and re-running
  the remainder with `CARGO_INCREMENTAL=0 RUSTFLAGS="-C debuginfo=0"`
  (env-only, no repo change). (2) Single-box absolutes carry allocator
  lottery noise as documented above.

## Remaining costs (not claimed)

- The outer WASM snapshots remain: checkpoint first-write copies
  (`Rc::make_mut`), non-boolean batch dispatch clones, and the
  `with_topology_transaction` helper. No browser-wide saving is claimed.
- Changed-state work still copies touched payloads (e.g. a rewritten NURBS
  face clones its net into the log); only untouched state is now free.
- Loop/coedge arena growth during bulk construction (PERF-T07), checkpoint
  retention and first-write cost (PERF-W04), immutable carrier sharing
  (PERF-T04), and compaction/ID reuse (P-Class 8.6) are untouched.
- `RollbackSnapshot` must be restored into the same `Topology` value it was
  captured from; the old implementation accidentally survived object
  replacement via its carried clone. No audited caller replaces the value
  mid-scope; the contract now states the requirement explicitly.
