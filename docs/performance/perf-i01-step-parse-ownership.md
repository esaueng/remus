# PERF-I01: STEP parsing ownership — borrowed spans, compact tags, lazy attributes

Bounded slice of [PERF-I01](../../kernel-maturity/roadmap.md#perf-i01):
reduce STEP parsing ownership and repeated parsing while preserving import
semantics. Owned work: `crates/io/src/step/reader.rs` and its private
helpers, dedicated STEP tests/benches, this note. Not touched: topology
transactions, shared `ImportLimits`, the STEP writer, the 3MF writer, other
agents' areas.

## What changed

`StepEntity` no longer owns two `String`s per entity (`entity_type`
uppercased copy + `attrs` copy of the source substring):

- **Borrowed source spans.** Entities are `StepEntity<'a>` with
  `type_raw: Cow<'a, str>` / `attrs: Cow<'a, str>` borrowing the input in the
  common path. Statements without block comments are sliced verbatim out of
  the source by a byte scanner (string- and comment-aware for `;`
  terminators, `''` escapes, quoted semicolons). The rare comment-bearing
  statement is normalized once (comment → single space) and stored owned.
- **Compact entity tags.** The per-entity uppercased `String` is replaced by
  `EntityKind`, a `Copy` enum classified once at index time (fast path:
  exact canonical match; slow path: stack-buffer ASCII case-fold, so
  `plane` still imports). Unknown types keep their raw token; the uppercase
  label is built only on the refusal path.
- **Lazy attribute views.** `AttrSlot::Text(String)` (one decoded allocation
  per name slot) became `AttrSlot::Text(&str)` borrowing the raw inner with
  `''` preserved. Decoding happens only where the value is used
  (`text_equals` compares without allocating; `attr_text_value` decodes on
  the cold path). `split_attr_slots` still returns a `Vec` per call — kept
  deliberately (see remaining costs).
- **No more `attrs.clone()` / `entity_text()` on hot paths.** `get_entity`
  returns `&'a` decoupled from the `&self` borrow, so builders copy the
  `Copy` tag and `Cow::Borrowed` spans (pointer copies) instead of heap
  cloning whole attribute strings. `entity_text` (which rebuilt
  `TYPE(attrs` per call) survives only on the cold unit/validation path.
- **Limits before materialization.** Every delivered statement is checked
  against `max_input_bytes` before the caller extracts anything
  (`"STEP statement bytes"`); entity-count enforcement is unchanged.
- **Source positions.** Each entity keeps its statement byte offset (`pos`),
  now surfaced in duplicate-ID and solid-context diagnostics.

## Allocation inventory (per entity, DATA section)

| Site | Before | After |
|---|---|---|
| Statement scan buffer (`current: String` push per char) | 1 alloc per statement | 0 (borrowed slice; owned only with `/* */`) |
| `entity_type` uppercased `String` | 1 alloc | 0 (`EntityKind`, `Copy`) |
| `attrs` owned copy of source substring | 1 alloc | 0 (`Cow::Borrowed` span) |
| `Text` name slots (decoded `String` each) | 1 alloc per name | 0 (borrowed raw; decode on use) |
| Builder `attrs.clone()` per entity use | 1 alloc per use | 0 (`Cow` pointer copy) |
| `entity_text()` rebuilds (`TYPE(attrs`) | 1 alloc per unit/validation lookup | unchanged count, cold path only |

## Measurements

Paired quiet runs, same machine, release profile, criterion
(`crates/io/benches/step_import.rs`, `--sample-size 10`,
`--measurement-time 5`); medians:

| Case | Baseline (origin/main `bf46fdc0`) | Candidate | Δ |
|---|---|---|---|
| small_box (writer round-trip import) | 140.7 µs | 100.4 µs | **−29%** |
| hammer_holder (932 KB, 14k statements, full topology) | 19.8 ms | 18.7 ms | **−5%** |
| repeated_entities_5k (scan + index only) | 1.54 ms | 0.883 ms | **−43%** |
| rational_net_2k (attribute-heavy B-spline) | 0.975 ms | 0.656 ms | **−33%** |
| malformed rejection | 433 ns | 270 ns | **−38%** |

End-to-end hammer-holder gains are modest because topology construction
dominates there; the parse-heavy shapes (repeated entities, large control
nets, small files) win 29–43%.

Peak RSS (whole-corpus probe, 31 files incl. tessellation + volumes,
`/usr/bin/time -v`, release): baseline 76,420 kB; candidate 78,008 /
80,004 kB across two runs — within run-to-run noise for a probe dominated
by meshing and volume integration. No material RSS change at corpus scale;
the win is allocation count and parse time. A dedicated peak-live-bytes
profiler for the parse phase alone remains open (see below).

## Compatibility evidence

- 31-file repository STEP corpus differential (solids, face counts, analytic
  face counts, volumes to 6 dp, typed refusal): **bit-identical**
  baseline vs candidate, including the `mambo_b1_untrimmed_nurbs` typed
  recovery-cap refusal with identical residuals.
- Deterministic re-import: candidate probe run twice → identical output.
- `cargo test -p remus-io --lib`: 413 passed. STEP integration targets
  (`cross_format`, `multi_solid_step_roundtrip`, `degenerate_torus_step`,
  `naming_roundtrip`, `nurbs_weight_cache_roundtrip`, `oring_nested_holes`,
  `multicavity_cut`): all green. `cargo test -p remus-wasm-io`: 8 passed,
  incl. `step_round_trip_through_arena_documents_is_exact`.
- New focused regressions (all in `reader.rs` tests): borrowed-vs-owned
  span selection, comment normalization, case-insensitive classification,
  complex detection, duplicate-ID byte offsets, statement-start positions,
  oversized-statement refusal, failed-import document preservation.
- Export/re-import is covered by the existing `cross_format` round-trips
  plus the corpus differential above; the writer is used as a transport,
  with independent volume/carrier/topology oracles asserting the result
  (face `type_tag`s, analytic counts, closed-form volumes).

## Browser translator qualification

- `cargo build -p remus-wasm-io --target wasm32-unknown-unknown --release`:
  baseline 3,405,378 B / candidate 3,427,323 B (+21,945 B, +0.6% for the
  tag tables and borrowed paths). Local uncommitted builds; no package
  versions touched.
- wasm-pack nodejs packages built from each worktree and exercised with
  `importStep(hammer_holder)` × 5 under `node --expose-gc`:
  - arena bytes **identical**: 637,353 (exact transfer preserved);
  - per-import time: baseline 91.9 ms → candidate **51.6 ms (−44%)**.
    The wasm win exceeds native end-to-end because the linear-memory
    allocator is more sensitive to `String` churn than the native one.
  - node RSS delta over 5 reps: 75 MB → 87 MB (retained linear-memory
    growth, noisy; no leak signal — repeated runs complete flat).
- Native heap vs translator memory are reported separately throughout
  (criterion + `/usr/bin/time` RSS above vs node RSS here). In-browser
  linear-memory high-water via consumer instrumentation remains
  consumer-side work.

## Fuzz

`cargo +nightly fuzz run step_reader -max_total_time=300`: 6,744,560 runs,
22,407 exec/s, peak RSS 588 MB, **zero crashes** (empty artifacts dir).
45,330 new coverage seeds were generated and discarded (not committed);
the two committed seeds remain the corpus. No findings to minimize.

## Remaining costs (explicitly out of this slice)

- `split_attr_slots` still builds a `Vec` per entity use; points,
  directions and placements are re-parsed per reference (no cross-entity
  parse cache — caching would be an AST larger than this slice budgets).
- `entity_text` still allocates on the cold unit/validation path.
- No dedicated parse-phase peak-live-bytes profiler yet (RSS measured at
  whole-probe granularity above).
- Full-workspace suite not re-run here (99 io integration targets include
  multi-minute boolean cases); STEP-scoped targets plus lib plus wasm-io
  are green. CI on the PR gives the full verdict.
