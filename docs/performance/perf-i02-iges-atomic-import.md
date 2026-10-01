# PERF-I02 (IGES slice): atomic append-only IGES import

Bounded slice of [PERF-I02](../../kernel-maturity/roadmap.md#perf-i02):
import new IGES bodies atomically without exposing partial geometry on
malformed input. Owned work: `crates/io/src/iges/reader.rs`
(`read_iges_with_limits`, `read_iges_impl_with_path`, `parse_iges_entities`,
`build_topology`, `build_plane_face`, `parse_float_params`), dedicated IGES
tests/benches/examples, this note. Not touched: topology transactions, shared
`ImportLimits`, STEP, arena schemas, extrusion history, shared WASM dispatch,
other formats, other agents' areas. Topology internals are consumed via the
existing `run_append_only` / `RollbackSnapshot` APIs; missing primitives are
handed off, not changed here.

## What changed

`read_iges_with_limits` no longer builds topology directly with silent
per-face skips:

- **Mutation-local transaction through construction.** The whole import
  (plane-face construction plus shell/solid assembly) runs inside
  `run_append_only`. Capture is O(1); records scale with touched state, never
  with unrelated document size. No scope clones the document — and none ever
  did on this path (the baseline built directly with no snapshot, verified by
  inspection of `origin/main` `iges/reader.rs`).
- **Append-only fast path for the qualified import.** Every write targets
  newly allocated entities (see inventory below), so the guard never trips
  for valid imports. A trip would rewind at O(new content) and retry once
  under the full transaction with an identical result; only the cost differs.
- **Restartable operation closure.** The parsed `entities` slice is shared
  across retries; intermediate `face_ids` handles are recreated inside the
  closure on every try, so a guard-trip retry never emits duplicates nor
  leaks handles from the abandoned try (those handles stay stale via
  high-water preservation).
- **Strict plane errors (no silent partial).** A malformed type-108 plane
  (short params, unparsable float, nonfinite value, zero/nonfinite normal,
  nonfinite frame or corners) is a hard `IoError::ParseError`, not a silent
  skip. Previously a zero-normal later plane after earlier good planes
  returned `Ok` with 5 faces (reproduced: seeded 1-box doc grew 8v/6f/1s to
  28v/11f/2s with a 5-face solid). Now the same file refuses typed with live
  counts unchanged (8v/6f/1s) and abandoned slots stale.
- **Checked limits before allocation.** Generated-entity bound
  (`planes × 15 + 2`, checked `mul`/`add`; overflow is `LimitExceeded`) is
  enforced before any topology write, alongside the existing record/entity
  caps (now checked arithmetic, not saturating). `parse_float_params` rejects
  malformed/nonfinite tokens typed; fixed-width slicing stays panic-free on
  ASCII (non-ASCII refused up front).
- **No parsing or format change for valid files.** Units, coordinate
  conventions (1×1 preview squares at `normal*d/|n|²`, half-extent 0.5),
  entity coverage (108 only; 110/126/128 still skipped by design), and default
  units remain unchanged. No IGES unit conversion was added; no geometric
  support was broadened.

## Allocation inventory (per import, after parsing)

| Site | Before (direct) | After |
|---|---|---|
| Entry snapshot | 0 (no clone ever existed on this path) | 0 (O(1) scope mark) |
| Construction (6 planes: 4v+4e+1w+1f+1loop+4coedges each = 90, +1 shell +1 solid = 92) | allocations only | allocations only (per-alloc identity records) |
| `add_face` loop/coedge derivation | new-slot only | new-slot only (guard passes) |
| Attributes / pcurves / journal | none written | none written (preserved on success and failure) |
| Failure rollback | n/a (silent partial success) | `undo_rewind_scope` (O(touched), handles stay stale) |

## Measurements

Paired quiet runs, same machine, release profile, identical harness
(`crates/io/examples/iges_import_scaling.rs`, 11 in-process medians per
cell). Raw samples: `evidence/perf-i02/iges_baseline.jsonl`,
`evidence/perf-i02/iges_candidate.jsonl`,
`evidence/perf-i02/iges_provenance.json`.
Baseline: `origin/main 4a93e758`. Candidate: this branch. One fixed box IGES
import per cell; `slots_growth` is exact new-body slots.

| Workload, 1 / 100 / 1000 unrelated boxes | Baseline median ms | Candidate median ms |
|---|---|---|
| `clone_only` (clone probe, not removed) | 0.003 / 0.14–0.60 / 1.7–2.1 | 0.003 / 0.14–0.62 / 2.5–2.9 (unchanged — still the clone probe) |
| `parse_only` (empty input, 0 bodies) | 0.000 / 0.000 / 0.000–0.001 | 0.000 / 0.000–0.001 / 0.003–0.004 (flat) |
| `import_box` (fixed IGES box import) | 0.035 / 0.058–0.47 / 0.75–4.9 | 0.037–0.038 / 0.064–0.52 / 1.07–1.19 |

Reading the table honestly:

- The IGES path never cloned: baseline `parse_only` is already flat at ~0 ms
  at every size, and baseline `import_box` at 1 box is 0.035 ms (no snapshot
  to remove). The candidate does not claim a snapshot win — none existed.
- The candidate adds only O(1) scope overhead: 1-box import 0.037 ms vs
  baseline 0.035 ms (noise); 100-box cells overlap within allocator noise
  (cp0 0.52 vs 0.47, cp1 0.064 vs 0.058); 1000-box cells overlap within the
  same realloc lottery (candidate 1.07–1.19, baseline 0.75–4.9 — the range is
  noise, the shape is the signal).
- The residual import cost is construction, not transaction: `slots_growth`
  is exactly 92 per box at every size (6 planes × 15 + 2), and retained
  checkpoints (`cp1`) change nothing systematically. The residual scales with
  arena growth during construction (those arenas are outside `reserve`'s six)
  — PERF-T07 territory, disclosed as remaining cost, not claimed here.
- Single-box absolutes carry realloc noise (e.g. 100-box `import_box` 0.06 vs
  0.5 ms across checkpoint cells); the empty-scope, parse-only and exact slot
  growth are the robust signal.

Peak RSS (process VmHWM, includes setup): baseline ~30 MB at 1000 boxes;
candidate ~31–41 MB — same shape, no material RSS change; the win is
correctness (atomicity), not bytes. A dedicated peak-live-bytes profiler for
the import phase alone remains open (PERF-M05).

## Compatibility evidence

- `cargo test -p remus-io --lib`: 421 passed (417 pre-existing incl. STEP
  slice + 4 new IGES unit qualifications). IGES integration
  `crates/io/tests/iges_atomic_import.rs` (10 tests): all green.
  `cargo test -p remus-wasm-io`: 8 passed, incl.
  `mesh_formats_round_trip_a_box` (export_iges still non-empty).
- New focused regressions:
  - `reader.rs` unit: `iges_import_takes_the_append_only_path` (exact 92-slot
    growth, `AppendOnly`), `iges_failed_import_retires_without_reusing_handles`,
    `parse_float_params_rejects_malformed_and_nonfinite`,
    `generated_entity_bound_fails_before_allocation` (10 refuses, 92 succeeds).
  - Integration `iges_atomic_import.rs` (10 tests): failure after each of 6
    face depths restores full logical state (live counts, volumes, attributes,
    journal contents, persistent refs to Bound/BoundMany/Ambiguous); malformed
    float/nonfinite planes restore; importer produces no attributes/pcurves;
    truncated/malformed records never panic; oversized counts and limit
    boundaries honest (10 refuses pre-allocation, 92 succeeds, 16-byte cap
    refuses); semantics (1×1 preview faces, normals/d, empty-vs-nonempty
    equality, limits, typed errors) match; checkpoint-restore atomic; 200-case
    deterministic fuzz (no panic, typed refusals only, refusals leak nothing);
    split-translator arena route exact.
- Differential: valid imports into empty vs 20-box docs match exactly (6 faces,
  same plane data); pre-existing volumes unchanged to 1e-12.
- Deterministic re-import: scaling harness run twice → identical slot growth.

## Browser translator qualification

- `cargo build -p remus-wasm-io --target wasm32-unknown-unknown --release`:
  3.4 MB WASM, builds clean. No package versions touched.
- Native simulation of the split-module path (translator `read_iges` +
  `serialize_solids` → kernel `deserialize_solids`, box IGES):
  - translator read + serialize to arena bytes (exact transfer preserved);
  - kernel deserialize restores 1 solid with 6 faces; pre-existing kernel
    volumes unchanged to 1e-12.
  - Failed translator import (zero-normal plane) refuses typed with empty
    scratch (0 solids, 0 vertices).
  - The translator always imports into a fresh scratch topology, so the
    large-document scope this slice adds never existed there; native import
    improvement does not make deserialization or JS transfer cheaper.
    Residual costs (serialize + deserialize + JS transfer) are reported
    separately and unchanged.
- In-browser linear-memory high-water via consumer instrumentation remains
  consumer-side work.

## Fault-injection evidence

- Every face depth (0–5 earlier planes allocated, 6th malformed) plus
  malformed-float, nonfinite, truncated, bad-DE-pointer, bad directory field,
  oversized-count, and input-byte-limit cases is covered: crafted malformed
  files fail typed and restore complete pre-existing logical state (live
  counts, volumes, attribute store, journal contents, persistent-reference
  resolution — never Dangling/NoMatch), with abandoned handles staying stale
  and fresh builds landing above the preserved high-water mark.
- Minimized witnesses preserved with typed expected errors: zero-normal plane
  (`ParseError`), `abc` float (`ParseError`), `inf`/`NaN`/`1e999`
  (`ParseError`), short-line truncation (empty import, no panic), odd D-line
  truncation (≤6 faces, no panic), `ABCDEFGH` DE pointer (typed or benign),
  `ABCDEFGH` directory field (`ParseError`), 10-entity budget
  (`LimitExceeded`), 16-byte budget (`LimitExceeded`).
- 200-case deterministic single-byte fuzz: no panic, only typed refusals
  (`ParseError`/`LimitExceeded`/`InvalidTopology`/`Topology`/`Operations`/
  `UnsupportedEntity`), every refusal leaks nothing.
- Imports after checkpoint restore (`restore_preserving_handle_slots`)
  remain atomic on both success and failure.

## Discovered unit/trim/solid-versus-sheet issues (filed separately, not fixed)

No IGES unit conversion was added and no geometric support was broadened.
Witnesses observed during qualification (existing lossy-preview behavior,
unchanged):

- Preview squares, not true trims: each 108 plane yields a fixed 1×1 square
  (half-extent 0.5) at its plane offset, not the file's true loop geometry.
  Witness: box 2×3×4 IGES imports 6 faces of area 1.0 each (not 2×3 etc.).
- Single solid per file, no sheets/cavities/multi-roots: all 108 faces join
  one shell/solid; open shells still become solids. Witness: 6-plane file → 1
  solid; no sheet/solid distinction.
- Source units uninterpreted: Global section units are skipped, so
  non-mm files import unscaled (STEP fallback remains the exchange path).
  Witness: Global `2HMM` vs `2HIN` yields identical geometry.

These remain declared preview limitations per O5.7; STEP is the exchange path.

## Remaining costs (explicitly out of this slice)

- The outer WASM snapshots remain: checkpoint first-write copies
  (`Rc::make_mut`), non-IGES batch dispatch clones, and the
  `with_topology_transaction` helper. No browser-wide saving is claimed.
- Changed-state work still copies touched payloads; only untouched state is now free.
- A changed-clone or unrelated full restore inside a live IGES scope would
  store one document-sized inverse. The measured import scopes do not use it.
- Loop/coedge arena growth during bulk construction (PERF-T07), checkpoint
  retention and first-write cost (PERF-W04), immutable carrier sharing
  (PERF-T04), and compaction/ID reuse (P-Class 8.6) are untouched.
- Other import formats (STL, 3MF, OBJ, PLY, glTF) still use their existing
  snapshot policies; only the IGES path is qualified here.
- `RollbackSnapshot` must be restored into the same `Topology` value it was
  captured from; no audited IGES caller replaces the value mid-scope.
