# PERF-I02 (STEP slice): atomic append-only STEP import

Bounded slice of [PERF-I02](../../kernel-maturity/roadmap.md#perf-i02):
import new STEP bodies atomically without copying all unrelated existing
topology. Owned work: `crates/io/src/step/reader.rs` (`read_step_impl` and
its private helpers), dedicated STEP tests/benches/examples, this note.
Not touched: topology transactions, shared `ImportLimits`, the STEP writer,
other formats, other agents' areas. Topology internals are consumed via the
existing `run_append_only` / `RollbackSnapshot` APIs; missing primitives
are handed off, not changed here.

## What changed

`read_step_impl` no longer takes `topo.clone()` before construction:

- **Mutation-local transaction through construction, postprocessing and
  validation.** The whole import (builder construction, `merge_split_rim_arcs`
  rim repair, `compare_validation_properties` validation tail) runs inside
  `run_append_only`. Capture is O(1); records scale with touched state, never
  with unrelated document size. No scope clones the document.
- **Append-only fast path for the qualified import.** Every write targets
  newly allocated entities (see inventory below), so the guard never trips
  for solid-only, sheet-inclusive and validation imports. A trip would rewind
  at O(new content) and retry once under the full transaction with an
  identical result; only the cost differs.
- **Restartable operation closure.** Builders, vertex/edge caches, diagnostic
  buffers and intermediate `built_*` handles are recreated inside the closure
  on every try, so a guard-trip retry never emits duplicate diagnostics nor
  leaks handles from the abandoned try (those handles stay stale via
  high-water preservation).
- **No parsing or format change.** Units, multiple contexts, exact carriers,
  rational weights, orientations, cavities, body-root order, names,
  sheet/solid-only behavior, resource limits and typed errors are unchanged;
  parsing runs before any topology mutation and is shared across retries.

## Allocation inventory (per import, after parsing)

| Site | Before (clone) | After |
|---|---|---|
| Entry snapshot (`topo.clone()`) | 1 full-document copy (all arenas, attributes, pcurves, journal) | 0 (O(1) scope mark) |
| Construction (`StepBuilder`: vertices, edges, wires, faces, shells, solids, derived loops/coedges) | allocations + clone | allocations only (per-alloc identity records) |
| In-import `edge_mut` overwrite (`normalize_planar_inner_winding`) | overwrite + clone | overwrite of new slot (index ≥ mark, guard passes) |
| Pcurve uses for new `(edge, face)` pairs | index writes + clone | pure additions (`old` is `None`, guard passes) |
| Solid/face attributes + `BodyClass::Sheet` for new shells | attribute writes + clone | new-index writes (guard passes) |
| Rim repair (`merge_split_rim_arcs`: one closed edge + wire/loop rewrites of new solids) | allocations/overwrites + clone | new-slot only (guard passes) |
| Diagnostics/validation reports | host-owned `Vec`s + clone | host-owned `Vec`s returned by value, recreated per try |
| Validation tail (`compare_validation_properties`) | read-only + clone | read-only, no mutation |
| Failure rollback | `restore_preserving_handle_slots` (full restore, sticky retirements) | `undo_rewind_scope` (O(touched), retirements undone, handles stay stale) |

## Measurements

Paired quiet runs, same machine, release profile, identical harness
(`crates/io/examples/step_import_scaling.rs`, 11 in-process medians per
cell). Raw samples: `evidence/perf-i02/baseline.jsonl`,
`evidence/perf-i02/candidate.jsonl`, `evidence/perf-i02/provenance.json`.
Baseline: `origin/main 594cd308`. Candidate: this branch. One fixed box
STEP import per cell; `slots_growth` is exact new-body slots.

| Workload, 1 / 100 / 1000 unrelated boxes | Baseline median ms | Candidate median ms |
|---|---|---|
| `clone_only` (snapshot cost alone) | 0.001 / 0.40 / 1.06–1.35 | 0.001 / 0.08–0.40 / 1.06–1.21 (unchanged — still the clone probe) |
| `parse_only` (scan + index + units, 0 bodies) | 0.005 / 0.09 / 1.02–1.26 | 0.003 / 0.004–0.005 / 0.005–0.009 |
| `import_box` (fixed box import) | 0.09 / 0.52–0.54 / 4.4–6.8 | 0.09 / 0.11–0.37 / 0.27–0.64 |
| `import_box_validated` (import + validation tail) | 0.10 / 0.52–0.54 / 4.4–4.5 | 0.10 / 0.12–0.39 / 0.28–0.59 |
| NURBS doc (100 boxes + 24 48×48 faces, 2.2 MB), `clone_only` | 0.19–0.22 | 0.19–0.20 |
| NURBS doc, `import_box` | 0.31–0.33 | 0.11–0.12 |

Reading the table honestly:

- The snapshot itself scales with document size (1 → 1000 boxes: ~1 ms;
  NURBS payload adds ~0.2 ms). The baseline pays it even for `parse_only`
  (0 bodies): 1.0 ms at 1000 boxes for doing nothing.
- The candidate removes it: `parse_only` is flat at ~4 µs across all sizes
  and payloads; `import_box` at 1000 boxes is 0.27–0.64 ms vs baseline
  4.4–6.8 ms (**~10–24×**; the range is allocator/realloc noise, the
  flat shape is the signal).
- The residual import cost is construction, not transaction: `slots_growth`
  is exactly 64 per box at every size (8 vertices + 12 edges + 6 wires +
  6 faces + 1 shell + 1 solid + 6 loops + 24 coedges), and retained
  checkpoints (`cp1`) change nothing for the candidate. The residual scales
  with loop/coedge arena growth during construction (those arenas are
  outside `reserve`'s six) — PERF-T07 territory, disclosed as remaining
  cost, not claimed here.
- Single-box absolutes carry realloc/rehash lottery noise (e.g. candidate
  `import_box` 0.11 vs 0.37 ms across checkpoint cells at 100 boxes); the
  empty-scope, parse-only and amortized cells are the robust signal.

Peak RSS (process VmHWM, includes setup): baseline 40–48 MB at 1000 boxes;
candidate 35–43 MB — same shape, no material RSS change at this scale; the
win is copied bytes and time, not peak retention. A dedicated peak-live-bytes
profiler for the import phase alone remains open (PERF-M05).

## Compatibility evidence

- `cargo test -p remus-io --lib`: 417 passed (414 pre-existing + 3 new
  append-only path qualifications). STEP integration targets (`cross_format`,
  `multi_solid_step_roundtrip`, `degenerate_torus_step`, `naming_roundtrip`,
  `nurbs_weight_cache_roundtrip`, `oring_nested_holes`, `multicavity_cut`):
  all green. `cargo test -p remus-wasm-io`: 8 passed, incl.
  `step_round_trip_through_arena_documents_is_exact`.
- New focused regressions:
  - `reader.rs` unit: `step_import_takes_the_append_only_path` (solid),
    `step_sheet_and_validation_imports_take_the_append_only_path`,
    `step_failed_import_retires_without_reusing_handles`.
  - Integration `crates/io/tests/step_atomic_import.rs` (8 tests):
    failure after vertices/edges/loops/pcurves/faces/shells/solids restores
    full logical state (live counts, volumes, attributes, journal contents,
    persistent references), malformed later body publishes nothing,
    attributes/names preserved, rim + validation commit atomically,
    semantics (units, carriers, weights, orientations, order, sheets,
    limits, typed errors) match empty-doc imports, cavities/names round
    trip, checkpoint-restore atomicity.
- Differential: imports into empty vs 10–20-box docs match exactly (volumes
  to 1e-9, face counts, body-root order, sheet counts, limit refusals).
  The 31-file repository STEP corpus is exercised through the existing
  round-trip suites, which all run through the new transaction path with no
  parsing change; no corpus file changed outcome.
- Deterministic re-import: scaling harness run twice → identical slot growth
  and volumes.

## Browser translator qualification

- `cargo build -p remus-wasm-io --target wasm32-unknown-unknown --release`:
  3.4 MB WASM, builds clean. No package versions touched.
- Native simulation of the split-module path (translator `read_step` +
  `serialize_solids` → kernel `deserialize_solids`, hammer_holder fixture):
  - translator read ~19 ms, serialize ~1.3 ms, arena bytes 637,352
    (exact transfer preserved);
  - kernel deserialize ~2.6 ms, volume 50241.714060.
  - The translator always imports into a fresh scratch topology, so the
    large-document clone this slice removes never existed there; native
    import improvement does not make deserialization or JS transfer cheaper.
    Residual costs (serialize + deserialize + JS transfer, ~4 ms, ~18% of
    the ~22 ms end-to-end) are reported separately and unchanged.
- In-browser linear-memory high-water via consumer instrumentation remains
  consumer-side work.

## Fault-injection evidence

- Every construction stage (vertices, edges, loops, pcurves, faces, shells,
  solids) plus attributes, rim repair and validation is covered: crafted
  malformed files fail at each stage and restore complete pre-existing
  logical state (live counts, volumes to 1e-9, attribute store, journal
  contents, persistent-reference resolution to Bound/BoundMany/Ambiguous —
  never Dangling/NoMatch), with abandoned handles staying stale and fresh
  builds landing above the preserved high-water mark.
- Multi-body late failure (second body malformed after first body fully
  built) publishes nothing: 0 new solids, journal unchanged, abandoned slot
  stale.
- Validation with invalid options fails before any topology mutation
  (`InvalidValidationProperties`, no state change); successful validation
  commits on the append-only path.
- Imports after checkpoint restore (`restore_preserving_handle_slots`)
  remain atomic on both success and failure.

## Remaining costs (explicitly out of this slice)

- The outer WASM snapshots remain: checkpoint first-write copies
  (`Rc::make_mut`), non-STEP batch dispatch clones, and the
  `with_topology_transaction` helper. No browser-wide saving is claimed.
- Changed-state work still copies touched payloads (e.g. a rewritten NURBS
  face clones its net into the log); only untouched state is now free.
- A changed-clone or unrelated full restore inside a live STEP scope would
  store one document-sized inverse. The measured import scopes do not use it.
- Loop/coedge arena growth during bulk construction (PERF-T07), checkpoint
  retention and first-write cost (PERF-W04), immutable carrier sharing
  (PERF-T04), and compaction/ID reuse (P-Class 8.6) are untouched.
- Other import formats (IGES, STL, 3MF, OBJ, PLY, glTF) still use their
  existing snapshot policies; only the STEP path is qualified here.
- `RollbackSnapshot` must be restored into the same `Topology` value it was
  captured from; no audited STEP caller replaces the value mid-scope.
