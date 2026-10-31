# RFC 0009: Explicit cache context

Status: **proposed** (design only, no production code). Tier: `core`. The
context holds session state: content caches the kernel reads on every edit,
plus the per-kernel classification cache and prepared moves.

Builds on [RFC 0001](rfc-0001-operation-context.md) (the `OperationContext`
carrier), [PERF-Q02](perf-q02-cache-identity.md) (topology cache identity),
[PERF-V02](../performance/perf-v02-orientation-probe-memos.md),
[PERF-D01](../performance/perf-d01-tessellation-reuse.md) and the
[browser kernel size architecture](kernel-wasm-size-architecture.md) (owner
decision 1: no speed regression).

Baseline: `main` at `cc7b260`; line references are to that revision. The
kernel struct is `RemusKernel` (renamed in #1358, after that revision); this
RFC says "the kernel".

## 1. Problem

The performance work added eleven production caches. Each one keeps its state
in a `thread_local!` behind a module-wide on/off switch. The WASM constructor
(`crates/wasm/src/kernel.rs:70-87`) turns all eleven on for the whole module.
The native library leaves them off.

- **Two code paths.** Native and WASM run different code. The bit-identity
  oracles cover the difference for the exact caches. Two caches are not
  exact (§2.3), so a WASM result can depend on what the module computed
  earlier.
- **Implicit lifetime.** Nothing owns the entries. They live as long as the
  thread, and dropping a kernel releases none of them. WASM linear memory
  never shrinks.
- **No aggregate bound.** The per-cache byte budgets add up to about
  120 MiB per thread, plus 32 MiB per kernel for classification. No API
  reports the total. `checkpointMemoryStats` explicitly excludes caches.
- **Sharing is all or nothing.** Two kernels in one thread always share every
  cache. The constructor comment calls that deliberate: a short-lived probe
  kernel gets content-keyed hits. No caller can ask for isolation, or for
  sharing across threads.
- **No native control.** `remus::Model` and server batch workers have only
  the per-thread free functions. A rayon or tokio worker thread starts with
  every cache off, whatever its caller enabled.
- **Configuration escapes transactions.** `setStoreLocalCuts`,
  `setMeshReuseEnabled` and `setMeasurementReuseEnabled` change module state.
  `executeBatch` classifies `setStoreLocalCuts` as `ReadOnly`
  (`bindings/batch.rs:274`), so a batch rollback does not undo it.

## 2. Inventory

### 2.1 Production caches and memos (thread-local)

Some abbreviations used in the tables:

- *exact*: a hit returns the bits a fresh computation returns.
- *ordinals*: entity handles are replaced by their first-occurrence ordinal
  inside the key.
- *FIFO*: deterministic oldest-first eviction.
- Sizes are WASM function bytes. The method is in §2.4.

| # | Cache | Key → value | Bound / eviction | Native default · WASM | Hit evidence | Correctness and oracles | WASM size |
|---|---|---|---|---|---|---|---|
| 1 | `algo::pcurve_memo` (planar arc pcurves) | Content: exact bits of the curve (kind, centre, normal, radii, axes), the endpoints, the domain, the plane and the plane frame (fixed 43-word key) → `Curve2D` | 4096 entries, no byte bound (est. a few MB). FIFO via `BoundedMemo` (`algo/src/memo.rs`) | off · on | 100 slots through a 324-hole plate: −7% instructions warm against −2% cold (`perf-local-boolean.md`, Stage 3 part 2). No hit counts. | Exact. Failures are not stored. Covered by `operations/src/boolean/tests.rs` (memo on/off) and the in-module tests. | < 0.1 KiB own code (inlined into callers) + `BoundedMemo` 3.9 KiB |
| 2 | `algo::wire_area_memo` (revolved-face wire areas) | Content: surface + the wire's edges in traversal order → `Option<f64>` | 8192 entries, no byte bound. FIFO | off · on | Measured together with #1: a slot costs −13% with memos on against −9% off (`perf-local-boolean.md`, Frozen untouched faces). | Exact. Covered by `builder/builder_solid/helper_oracle_tests.rs` and `boolean/tests.rs`. | inlined; shares `BoundedMemo` |
| 3 | `algo::ray_geom_memo` (torus trim polygons) | Content: torus + outer-wire edges + classifier tolerance → `Option<TorusTrim>` | 1024 entries, 8 MiB. FIFO weighted | off · on | **None measured.** A source comment says the holder's four trims take ≈ 255 KB. | Exact. A declined trim is stored as declined. Covered by `io/tests/hammer_ray_geom_oracle.rs` (result arenas off, cold and warm). | inlined |
| 4 | `check::properties::face_cache::FaceIntegralCache` (fixed-order face integrals) | Content: request family + Gauss order + reversal + every surface field + wires/edges/vertices (ordinals). Positions are relative to an anchor, so a translate also matches → integrals + vector area | 8192 entries, 16 MiB of keys. FIFO | off · on | `validateSolid` repeat 1484 → 2.2 ms (W). Probe-kernel copy 1479 → 1.4 ms. `moveFacesJournaled` 1083 → 45 ms. Recognition: 50 hits / 21 misses (`perf-v02-*`, `perf-o06-recognition-graph.md`). | Full and area readings are exact. **Re-referenced and translated volume-term readings are not exact** (a few ulps, §2.3). They feed the orientation sign, the qualified face-integral volume route of `solid_volume`, and the blend-move volume change. Covered by `face_cache/tests.rs`, `io/tests/b28_probe_memo_identity.rs`, `io/tests/edit_aware_measurement.rs` and the `key_reads_every_geometry_field` size pin. | 21.3 KiB |
| 5 | `operations::measure::volume_memo` | Identity: `CacheIdentity` + solid + deflection. Then content: a whole-solid key (handles verbatim, or canonical ranks under reuse) → volume | 64 readings, 16 MiB. FIFO. `scoped()` puts in a 16-entry stand-in for one operation (blend cascade, prepared move) | off (on inside `scoped`) · on | Repeat < 0.05 ms native. W 892 → 0.0 ms. | Exact. Covered by `operations/tests/measurement_memos.rs`, `measure/reuse/tests.rs`, `wasm/bindings/checkpoint.rs` tests (restore invalidates) and `io/tests/measurement_reuse.rs`. | 25.5 KiB |
| 6 | `operations::measure::reuse` (face-area reuse + the volume reuse flag) | Identity, then content with canonical ranks → face area | 512 readings, 8 MiB. FIFO. `enabled: Option<bool>` keeps an explicit user choice when a later kernel is built | off · on (`enable_default_measurement_reuse`) | **No measured numbers in the docs.** | Exact. Covered by `measure/reuse/tests.rs` and `io/tests/measurement_reuse.rs`. | 1.9 KiB |
| 7 | `operations::measure::nurbs_area_memo` | Content: complete face boundary + pcurves (ordinals) + deflection + angular tolerance + policy → area | 512 entries, 8 MiB. FIFO | off · on | 40 hits / 2 misses per recognition (W) | Exact. Covered by `measure/nurbs_area_memo/tests.rs` and `resize_blend/translation_oracle_tests.rs`. | 18.7 KiB |
| 8 | `operations::measure::face_box_memo` | Content: surface + boundary. Vertex positions are added in handle order only when signed zeros or NaN make that order matter → `Aabb3` | 4096 entries, 8 MiB. FIFO | off · on | **None measured.** | Exact. Covered by `measure/face_box_memo/tests.rs`. | 13.1 KiB |
| 9 | `operations::tessellate::face_cache` (per-face meshes) | Content: carrier, wires, trims, pcurves, final boundary-plan chains (ordinals), deflection, angular tolerance, circle floor and cap flag. A translate is accepted only under `TranslationRule` → captured pool slice + triangles | 4096 faces, 32 MiB. Display entries have priority over quantity entries, FIFO within each class | off · on | Holder edit: 127 exact / 11 translated hits. Warm display 4× native, ≈ 5× W (PERF-D01 §8). Drag p95 246 → 48.5 ms (`perf-face-edit-responsiveness.md`). | Exact hits match fresh output exactly. **Translated hits are value-equal, not bit-equal**: positions within 1.5e-14, normals bit-identical. Covered by `tessellate/tests/face_cache.rs`, `io/tests/tessellation_reuse_hammer.rs` and `tessellation_digest_hammer.rs`. | 29.9 KiB |
| 10 | `operations::tessellate::reuse` (mesh-stage reuse: boundary plans, edge samples, holed-planar CDT, weld/dedupe connectivity) | Complete input words, edge identities remapped in sorted order | 4096 entries, 16 MiB. Separate from #9. Quantity entries cannot evict display entries | off · on | **No measured numbers** (`interactive-mesh-reuse.md` gives bounds only) | Exact for the reused stages. Lifting and reconciliation still run. Covered by `tessellate/reuse/tests.rs` and `io/tests/tessellation_incremental.rs`. | 21.5 KiB |
| 11 | `operations::query::crossing_memo` (edge-concavity ray crossings) | Content interned to an id (`face_content_words`), plus the ray's origin and direction bits → `u32`. Planar faces are skipped | 65 536 counts, 4096 contents, 8 MiB. FIFO each | off · on | `solidEdgeRelations` 275–338 → 55–63 ms. `recognizeFeatures` 321 → 64–74 ms (W). Native relations 110 → 52 ms (roadmap PERF-O06-CM). | Exact. Covered by `io/tests/hammer_crossing_memo_oracle.rs`. Already **injected**: `remus_check::classify::FaceCrossingMemo` is a `Send + Sync` trait object on `PreparedSolid`. | 4.2 KiB (roadmap: +14 KiB raw at landing) |

### 2.2 Per-kernel and operation-scoped caches (not thread-local)

| Cache | Key → value | Bound | Owner | Evidence | Oracles |
|---|---|---|---|---|---|
| `check::classify::ClassificationCache` (PERF-Q02) | `(lineage, generation, solid, tolerance bits, recovery attempts)` → prepared classification | 16 solids, 32 MiB. FIFO. ≈ 1.8 KB per entry | `RemusKernel::classify_cache` (`RefCell`) | 5.5× on repeated queries. Thrashes above 16 solids (`classify_cache_perf`). | `wasm/bindings/checkpoint.rs` restore-invalidation tests, plus the identity audit in PERF-Q02 |
| `wasm::bindings::prepared_move::PreparedMoves` | Opaque never-reused handle → `PreparedFaceMove` | 16 live | Kernel. Not saved or restored by checkpoints. | Prepared p95 29 against 43 ms (W, `face-drag-session-2026-10-19.md`) | `prepared_move/tests/scaling.rs`, `face_move_preview_tests.rs` |
| `resize_blend::DISCOVERY_MEMO` | Tagged with `CacheIdentity`. `(edge, face pair)` verdicts + adjacency | One move | `DiscoveryMemoScope` guard (thread-local, set only during the call) | None | `resize_blend/translation_oracle_tests.rs` |
| `volume_memo::scoped` stand-in | as #5 | 16 | One operation; the disabled memo is restored on unwind | (blend cascade) | as #5 |
| `math` `DerivativeScratch` basis memo | Clamped `u` bits + span + local knots | 1 | Caller-owned scratch. Already explicit. | PERF-V02 | NURBS bit-identity oracles |

### 2.3 Exact and value-equivalent caches

Nine of the eleven thread-local caches are exact. The comment at
`kernel.rs:78` says "every hit is bit-identical to recomputing". Two caches
make that claim only for part of what they return:

- **Face-integral volume terms (#4).** A reading taken about another
  reference point, or from a translated source, differs from a fresh
  integration by a few ulps. Three paths read these terms through
  `integrate_face_volume_about_memoized`:
  - the orientation probe (`measure/volume.rs:2487`), which uses only the
    sign;
  - `qualified_face_integral_volume` (`measure/volume.rs:716`), which feeds
    the value `solid_volume` returns;
  - `translated_faces_volume_change` (`resize_blend.rs:1884`), the blend-move
    check.

  So a WASM `volume` reading can differ in its last bits depending on what
  the module integrated before.
- **Translated face-mesh hits (#9).** Positions can differ from a fresh mesh
  by up to 1.5e-14.

So a WASM volume or mesh can depend, in its last bits, on which bodies the
module measured or meshed earlier. Native output never depends on history,
because the caches are off. Meanwhile `kernel.rs:78` says "every hit is
bit-identical to recomputing". That comment should be corrected
independently of this RFC.

This classification decides the sharing rules in §3.3. Every cache in a
shared scope must be exact, or must let its owner turn off the
value-equivalent path.

### 2.4 Non-cache thread-locals (kept out of the cache context)

| Kind | Sites | Compiled in |
|---|---|---|
| Deterministic work counters | `algo/src/perf.rs`, `check/src/perf.rs`, `operations/src/performance.rs::BOOLEAN_ROUTES` | `perf-counters` feature |
| Test-only counters, forcing switches and logs | `math/cdt` work, `math/nurbs/bezier_clip`, `algo/classifier/torus_patch`, `blend/body_edit`, `check/face_integrator`, `io/step/reader` audits, `operations/{pattern,validate,heal,classify,copy,boolean/mod,boolean/local_cut,boolean/store_local::BUILT,measure/volume,tessellate/pool_index}`, `*_oracle_tests.rs`, `wasm/bindings/batch.rs` test clock | `cfg(test)` |
| Trace clock | `operations/src/performance.rs::TRACE` (`setPerformanceTracing`) and `algo/src/perf.rs` phase clock | always / feature |
| Behaviour switch | `operations/src/boolean/store_local.rs::ENABLED` (`setStoreLocalCuts`) | always |

Counters and trace clocks stay thread-local. They observe work and do not
choose it, and they are off or compiled out in shipped builds.
`store_local::ENABLED` is **policy**, not a cache: a different route gives a
different entity order. It belongs in `OperationContext` under RFC 0001, and
it is out of scope here. It is listed so the migration does not move it into
the cache context by accident.

### 2.5 Code size (measured, with the caveats below)

Method:

- Build: `RUSTFLAGS="-C target-feature=+simd128" cargo build -p remus-wasm
  --target wasm32-unknown-unknown --release --no-default-features` on
  `cc7b260`. That is the shipped Cargo configuration, before `wasm-opt`.
  Raw output: 14,414,866 B, 1.37 MB of it the name section.
- Measurement: `twiggy top`, shallow bytes, grouped by the module path in each
  function name. Generic instances that carry the module's type in their
  name count toward the module.

Results:

- Memo modules #1–#11 together come to **≈ 141 KiB** before `wasm-opt`.
  That fits the ≈ 107 KB shipped figure after `wasm-opt`'s typical
  reduction.
- `ClassificationCache` is 6.3 KiB. Prepared moves are 12.8 KiB (operations +
  binding).
- The thread-local machinery itself (`no_threads::LazyStorage::initialize`
  and `LocalKey::with`) is **≈ 7 KiB** across all sites. On `wasm32` without
  atomics a thread-local is a plain static with an init flag.

Caveat: this attribution undercounts. LTO inlines the key builders and
lookups into their callers, and the small algo memos show almost no own code
for that reason.

## 3. Design

### 3.1 Shape

There is one owned object, split by whether its contents can be shared. Each
layer defines the part it owns, so the layer table needs no new
dependencies:

```text
CacheContext                                   (remus-operations)
├── content: Arc<ContentCaches>                shareable, content-keyed, exact
│     ├── algo:  remus_algo::AlgoCaches        { pcurve, wire_area, ray_geom }
│     ├── check: remus_check::CheckCaches      { face_integrals }
│     └── ops:   measurement { nurbs_area, face_box, face_area_reuse },
│                tessellation { face_mesh, mesh_stage }, crossing
└── session: SessionCaches                     owned by one kernel/Model
      ├── volume            (identity + content; per session, see 3.3)
      ├── classification    (ClassificationCache, PERF-Q02)
      └── limits/stats bookkeeping
```

`PreparedMoves` stays a kernel field. It is a handle store, not a cache. It
appears in the memory report (§3.4) but not in `CacheContext`.

Each leaf keeps its current type and code: `BoundedMemo`,
`FaceIntegralCache`, `FaceMeshCache`, and so on. The migration moves storage
and leaves the algorithms alone.

### 3.2 Carrying it: `OperationContext::caches` plus an ambient scope

RFC 0001's context lives in L0 `math`, which cannot name algo or operations
types. The context therefore carries an opaque handle, in the same way it
already carries `CancellationToken`:

> **As built (stage 1).** The scope landed as a type-indexed set of parts,
> not a provider trait: `CacheScope(Arc<Parts>)`, where each layer defines
> its part type and calls `scope.part::<T>()` (created on first use) or
> `scope.existing::<T>()`. A provider struct that names every part would
> have to live in the lowest operations module tier and name types from the
> higher tiers, which `scripts/check-ops-module-graph.py` (RFC 0007) rejects.
> With a type-indexed set, no layer names a part above it. The design below
> is kept as proposed; §5 records what each stage built.

```rust
// remus-math (L0)
pub trait CacheProvider: Send + Sync + fmt::Debug {
    /// The cache set a layer owns, found by type (`AlgoCaches`, `CheckCaches`, ...).
    fn part(&self, id: TypeId) -> Option<&(dyn Any + Send + Sync)>;
}
#[derive(Clone, Debug)]
pub struct CacheScope(Arc<dyn CacheProvider>);   // PartialEq = Arc::ptr_eq, like CancellationToken

pub struct OperationContext { /* existing fields */ pub caches: Option<CacheScope> }
```

How each layer reads it:

- A layer resolves its part **once per operation entry**:
  `ctx.caches.as_ref().and_then(|s| s.part::<AlgoCaches>())`. That costs one
  dynamic call and one `TypeId` comparison.
- From there the `&AlgoCaches` travels in the state the operation already
  carries: the GFA arena, the tessellation session, `PreparedSolid`. Inner
  loops never touch the scope.
- `caches: None` (the `OperationContext::new()` default) means every cache is
  off, which is today's native behaviour. That satisfies RFC 0001 ground
  rule 2. Per ground rule 3, the field lands with its first consumer
  (stage 1).

**Ambient scope, the bridge for context-free entry points.** Only three
operations take a context today: `boolean_with_context`,
`solid_to_solid_maximum_distance_with_context` and
`imprint_planar_polygon_with_context`. Measurement, tessellation, validation,
queries and `move_faces` take none. Adding `*_with_context` to all of them
first would make the migration an API project.

Instead, `CacheScope::enter(&self, f)` installs the scope in **one**
thread-local slot for the duration of `f`, restores the previous value on
exit (also on unwind), and supports nesting. A context-free entry point reads
the ambient scope where it reads its `thread_local!` today. A
`*_with_context` entry point uses `ctx.caches` and ignores the ambient one.

This is still a thread-local, but a different kind:

- one slot instead of eleven;
- set only inside an owner's call, never module-wide;
- empty outside any scope, so library callers see today's defaults.

The slot is a convenience for the migration. The ownership model does not
depend on it. Once an entry point gains a context, it stops reading the
slot.

**Rayon workers.** The ambient scope does not propagate to worker threads.
That is today's behaviour, and it is correct: in `tessellate/solid.rs:836`
and `:1737` the parallel closures (`sample_edge_with_params`,
`run_planar_cdt`) are pure, and every cache lookup and store happens on the
calling thread before or after the parallel map. The RFC makes that an
invariant: caches are consulted only on the thread that owns the operation.
A `debug_assert` on the ambient slot can enforce it. An explicitly passed
`&ContentCaches` is `Sync`, so a later parallel path may use it on purpose.

### 3.3 Lifetime and sharing

| Owner | Content caches | Session caches |
|---|---|---|
| Kernel, default constructor | Clones one module-level `Arc<ContentCaches>` (lazily created). All default kernels share it, as they share the thread caches today, so the probe-kernel content hits are preserved. | Its own, created and dropped with the kernel |
| Kernel, document copy (`compactDocument`, `arena.rs:357`) | Inherits the parent's `Arc` | Fresh |
| Kernel, explicit isolation (new constructor option) | A private `ContentCaches`, dropped with the kernel | Its own |
| `remus::Model::new()` | Its own `ContentCaches` | Its own |
| `Model::with_caches(scope)` / `Model::share_caches(&other)` | Shared `Arc` | Its own |
| Library free functions, no scope | none (caches off) | none |

The rules:

- **A shared content cache survives across sessions** for exactly as long as
  some owner holds its `Arc`. That is explicit, reference-counted lifetime
  instead of "until the thread exits". When the last default kernel is freed
  from JS, the shared entries go with it. A consumer that wants the probe
  hit keeps its main kernel alive, which every consumer measured so far
  already does.
- **Only exact caches are shareable by default.** Two paths are
  value-equivalent (§2.3): the face-integral translated and re-referenced
  probe readings, and translated face-mesh hits. They are gated by a
  per-owner policy, `CacheOptions::translated_reuse` (default on for the
  kernel, matching today). When a scope is shared, that policy is part of
  the scope, so two owners never disagree about it.
- **Volume stays per session.** Its identity path is lineage-keyed, so it
  cannot hit across documents anyway. Its content path keys on handles or
  canonical ranks, which a probe kernel's document copy does not share.
  Moving it into the session makes its memory follow the document.
- **The ambient scope is set by the owner.** The kernel enters its scope in
  `topo()` / `topo_mut()` and in the handful of bindings that touch no
  topology (stats, limits). Those accessors are the existing choke points.
  When every live kernel uses the default shared scope, entering is a single
  pointer comparison against the slot. With an isolated kernel the scope
  swaps per call.

### 3.4 Memory accounting and limits

- Every leaf already reports estimated retained bytes and its bounds. The
  estimates are not RSS, the same convention as
  `topology::memory::MemoryEstimate`.
- `CacheContext::memory_estimate()` returns one row per cache (`entries`,
  `retained_bytes`, `max_entries`, `max_bytes`, `hits`, `misses`,
  `evictions`) plus a total. A shared `ContentCaches` is counted once per
  report, deduplicated by `Arc` pointer, exactly as `checkpointMemoryStats`
  deduplicates shared topologies.
- New binding `cacheMemoryStats()` (camelCase JSON, no geometry, no handles).
  `checkpointMemoryStats` keeps its meaning (it excludes caches, as
  documented) and gains one additive field, `cacheEstimatedBytes`, so a
  consumer can see both totals in one call. `meshReuseStats` and
  `measurementReuseStats` keep their shapes and now read the kernel's scope.
- `CacheLimits` lets an owner set per-cache bounds and an **aggregate byte
  budget**. Its default is the sum of today's per-cache defaults, so default
  behaviour does not change. When the aggregate is exceeded, eviction runs
  in a fixed order: quantity meshes, then mesh-stage entries, crossing
  counts, face boxes, NURBS areas, face integrals, display meshes. FIFO
  holds within each cache. A fixed order keeps eviction deterministic for a
  given call sequence.
- The pcurve and wire-area memos (#1, #2) gain estimated weights
  (`insert_weighted`), so they count toward the aggregate. Their entry
  bounds stay as they are.
- `setCacheLimits(json)` and `clearCaches()` on the kernel. Both are `Tier:
  core`, because they edit session state.

### 3.5 Invalidation

Nothing new is needed, and the RFC keeps it that way:

- **Content caches** need no invalidation: an edited face stops matching. The
  standing obligation is key completeness. Any field a cached computation
  reads must be in its key. The `key_reads_every_geometry_field`-style size
  pins carry that today.
- **New rule, from moving onto the context.** When a cached path starts
  reading an `OperationContext` field (a tolerance, a budget, a fallback
  policy), that field joins the key in the same change. Today's keys hold
  the constants those paths use, such as the classifier tolerance in #3 and
  deflection and angular tolerance in #7 and #9. A context-supplied value
  that the key omits would serve one policy's result to another.
- **Identity caches** (volume identity path, classification, discovery memo)
  keep PERF-Q02: `(lineage, generation)` bumps on every mutation, restore and
  rollback.
- **No persistence.** No cache is ever serialized: not into arena documents,
  not into repro bundles, not into checkpoints. Keys may therefore depend on
  the build (field layout, algorithm version) without a version word.
- **Owner drop** releases everything the owner held exclusively. `clear()`
  empties a scope for every owner that shares it, and the binding's
  documentation says so.

### 3.6 Thread safety

- `ContentCaches: Send + Sync`. Each leaf sits behind its own
  `std::sync::Mutex`, so cache contention never serializes across families.
- On `wasm32-unknown-unknown` without atomics, std's `Mutex` is the
  no-threads implementation: a flag check, the same cost as today's
  `RefCell`.
- **The fallback semantics stay the same.** Every current lookup uses
  `try_borrow_mut` and takes the uncached path when the borrow is
  unavailable (re-entrancy). Each one becomes `try_lock` with the same
  fallback. The design adds no blocking acquisition, so it cannot deadlock.
- **Native contention.** Lookups go through `try_lock`, so a contended
  lookup misses instead of waiting. Results stay deterministic for the exact
  caches. Only hit rates and timings depend on scheduling. A shared scope
  across threads must run with `translated_reuse` off, or accept
  history-dependent last bits (§2.3); `Model::share_caches` documents that.
- **Default for native multi-threaded work:** one `CacheContext` per worker
  (per `Model`), no sharing. Sharing is opt-in. Sharded leaves are deferred
  until a measured server workload shows contention.
- **Lock granularity.** Hot multi-lookup sites acquire a guard per batch of
  work instead of per lookup:
  - the crossing memo, per solid probe set;
  - the face-mesh cache, per tessellation (it already uses a session);
  - the face-integral cache, per shell.

  This replaces one thread-local access per lookup and keeps the native
  atomic cost off the inner loop.

### 3.7 Native defaults

- **Library free functions: unchanged (caches off).** Library callers,
  criterion cold benches and the B28 guard keep measuring the uncached code.
- **`remus::Model`: caches on by default, with the kernel's defaults.**
  `Model` is the native counterpart of the kernel, and turning the caches on
  closes the native/WASM code-path gap for the main native front door.
  Every exact cache changes only timing. With `translated_reuse` on, mesh
  positions can move by up to 1.5e-14, and face-integral volumes by a few
  ulps, depending on history, as in WASM today. `Model::with_caches(CacheContext::disabled())` gives the cold path.

  **Owner decision needed:** whether `Model`'s default enables
  `translated_reuse`, matching WASM, or only the exact paths. The RFC
  recommends matching WASM, so that one configuration is what everyone
  qualifies.
- Server batch work constructs one `Model` per job or per worker. A job that
  replays the same body many times shares a scope explicitly.

### 3.8 How the oracles carry over

The oracles compare memo off, cold and warm, and they toggle state with the
`enable_thread_*` / `set_thread_*` / `clear_thread_*` functions (50 public
free functions across algo, check and operations).

- **During migration** each of those functions stays as a deprecated shim
  over a per-thread default scope. Stage N moves storage, and the family's
  oracle tests run unmodified against the shim. **That is the stage's
  proof**: unchanged tests, unchanged expected values.
- **After a family has moved**, its tests construct `CacheContext::disabled()`
  and `CacheContext::with_defaults()` explicitly and run the same comparison
  through `enter`. Tests become independent of test-thread reuse, which today
  is avoided only because each libtest test runs on its own thread.
- Each stage adds two contract tests:
  - two kernels with isolated scopes do not hit each other, and dropping one
    releases its bytes (`cacheMemoryStats`);
  - a document copy, or a default probe kernel, still hits the parent's
    content (the PERF-V02 probe case).
- WASM contract tests (`execute_batch`) assert per-call hit/miss deltas
  through the stats bindings, not through the Rust thread functions.

## 4. Size and speed

**Speed, WASM.** Today a lookup is a static load, an init-flag check and a
`RefCell` borrow. Afterwards it is a pointer the operation already holds,
plus a no-atomics `Mutex` flag check. Resolving the scope happens once per
operation. The RFC expects no measurable change, and must show it.

**Speed, native.** Library paths with caches off get one `Option` test per
operation entry instead of one thread-local read per lookup site. Paths with
caches on (Model, tests) pay one uncontended atomic acquire/release per
guarded batch (§3.6). Per-lookup locking at the crossing-memo granularity, up
to 65 536 counts, would cost about 10–20 ns × lookups. That is why the guard
is per batch and why the stage gate measures it.

**How to measure.** No stage merges without all of the following.

1. **Instruction counts** on the existing suites: `scripts/performance/run.py`
   (main), `sketch/`, `transactions/`, `complex-boolean/`, and
   `store-local/qualify.cjs` for correctness. Interleaved baseline and
   candidate, with "within noise" as defined by owner decision 1.
2. **Warm-cache workloads.** The main suite has none today: its cases are
   transforms, booleans, chains and NURBS, all cold. Stage 0 adds the
   existing diagnostics as suite cases in native and WASM variants:
   `crates/remus/examples/{measurement_memo_perf,tessellation_reuse_perf,
   offset_face_perf,recognition_perf}.rs` on the hammer holder, plus the
   324-hole plate slot sequence from `perf-local-boolean.md`. Each case runs
   cold, warm and copy-in-probe-kernel.
3. **Criterion**, cold and warm variants: `operations` `classify_cache_perf`,
   `perf_d01_quantity`, `boolean_perf`, `cad_operations`; `io`
   `nurbs_properties`; `math` `torus_ray_crossings`.
4. **Work equality.** Every case records the cache stats before and after.
   The hit, miss and eviction counts must be identical between baseline and
   candidate on every workload. That proves the same work is skipped, which
   timing alone cannot.

**Code size (estimate, not measured).**

- Removed: about 11 lazy thread-local initializers and their `with` glue,
  ≈ 5–7 KiB of the measured 7 KiB.
- Added: `Arc` + `dyn Any` resolution, 11 `Mutex` wrappers (no-atomics), the
  stats/limits bindings and the aggregate accountant, ≈ 6–12 KiB raw.
- Net: **−1 to +7 KiB raw**, far below the 64 KiB `WASM-Size:` threshold.
- The memo modules themselves (≈ 107 KB shipped) do not shrink. This RFC
  makes them owned, not smaller. Shrinking them is a W3-style task, for
  example a shared key builder for #4, #7, #8 and #11, which re-walk the
  same face content.

## 5. Staged migration

Every stage moves one cache family. Each stage ships with:

- its oracles unmodified (§3.8);
- the stats-equality gate and the speed gate (§4);
- no change to defaults.

| Stage | Family | Work | Public API |
|---|---|---|---|
| 0 | none | Warm-cache suite cases and the stats-equality recorder (§4.2, 4.4). Baselines recorded. **Landed:** `run.py --family warm`, `cache_stats.py`; baseline in [`cache-context-stage0-baseline-2026-10-31.md`](../performance/cache-context-stage0-baseline-2026-10-31.md). | none |
| 1 | Crossing memo (#11), the pilot | `CacheScope`, `CacheProvider` and `OperationContext::caches` (first consumer). `ContentCaches` holding only `crossing`. Kernel owns `CacheContext`, enters in `topo()`/`topo_mut()`. `SolidCrossingMemo` holds the leaf instead of reading `MEMO`. It is already injected through `FaceCrossingMemo`, so this is the smallest change. **Landed** as follows. `remus_math::context::CacheScope` is a type-indexed part set with the ambient slot (`ambient`, `enter`, `install`), see §3.2. The crossing memo is a part of the scope, and `SolidCrossingMemo` holds an `Arc` of it; every access uses `try_lock` with the old `try_borrow` fallback. `remus_operations::cache::thread_default_scope` is the thread's scope, created on first use; the kernel holds it (`RemusKernel::caches`), so kernels on one thread still share content hits. Changes from the plan: `OperationContext::caches` waits for stage 2, because no entry point that takes a context reaches the crossing memo (RFC 0001 ground rule 3). The kernel does not enter its scope per call: while every kernel holds the thread's default scope there is nothing to switch, so per-call entering lands with isolated kernels (stage 6). The `CacheContext` owner type lands with the session caches (stage 6). `cacheMemoryStats` reports all eleven content caches plus classification, each tagged `scope: "session"` or `"thread"`. | Additive: `remus_math::context::CacheScope`; `remus_operations::cache::thread_default_scope`; `query::{enable,disable}_crossing_memo`, `set_crossing_memo_limits`, `crossing_memo_stats`, `crossing_memo_content_bytes` (scope-taking); JS `cacheMemoryStats`. The `*_thread_crossing_memo*` functions now act on the thread's default scope and are documented as remaining until stage 8; no `#[deprecated]` attribute, so the oracles run unmodified. |
| 2 | Algo memos (#1–#3) | `OperationContext::caches` (deferred from stage 1, its first consumer). `AlgoCaches`. Resolved once in `boolean_with_context` and carried on the GFA state. The only family on a path that already carries a context, so no ambient read. | `remus_algo::AlgoCaches`; `enable_thread_{pcurve,wire_area,ray_geom}_memo` deprecated |
| 3 | Face integrals (#4) | `CheckCaches`. `_memoized` functions take `&FaceIntegralCache`; the ambient bridge serves validate and measure. Gate `translated_reuse`. | `remus_check::CheckCaches`; `*_thread_face_cache*` deprecated |
| 4 | Measurement (#5–#8) | Volume moves to `SessionCaches`. NURBS area, face box and face-area reuse move to `ContentCaches`. `volume_memo::scoped` becomes an operation-local stand-in passed by `&mut`. Moved as one family because #6 builds on #5's key. | `measure::*_thread_*` deprecated; `setMeasurementReuseEnabled` reads/writes the kernel scope |
| 5 | Tessellation (#9, #10) | One `TessellationCaches` holding both, sharing one display/quantity budget class. Gate translated hits by `translated_reuse`. | `tessellate::{enable_face_mesh_cache,enable_mesh_reuse,...}` deprecated; `setMeshReuseEnabled` reads/writes the kernel scope |
| 6 | Session caches | `ClassificationCache` moves from its kernel field into `SessionCaches` (no behaviour change; uniform accounting). `CacheLimits`, aggregate budget, `setCacheLimits`, `clearCaches`, isolated-kernel constructor option. `DiscoveryMemo` becomes an operation-local `&mut`. | Additive JS: `setCacheLimits`, `clearCaches`, constructor option. `checkpointMemoryStats` gains `cacheEstimatedBytes` (additive). |
| 7 | Native defaults | `Model` owns a `CacheContext` (on, §3.7); `Model::{with_caches, share_caches, caches}`. | Additive Rust API. **Behaviour change for `Model` users**: timing, plus last-bit mesh history dependence if `translated_reuse` is on. Owner decision. |
| 8 | Retire | Delete the deprecated free functions and the eleven `thread_local!`s. Remove the constructor's enable calls. The ambient slot stays until every cached entry point has a `*_with_context` form. | **Breaking (Rust)**: ~50 `pub fn`s in `remus-algo`, `remus-check` and `remus-operations` are removed after one release of deprecation. JS surface unchanged. |

### 5.1 `executeBatch` and checkpoints

- **Rollback.** A rolled-back batch leaves content-cache entries in place.
  They are correct for any topology with that content, so the only cost is
  memory, which the aggregate budget bounds. Identity entries miss after
  rollback (PERF-Q02 generation bump). This is today's behaviour, now
  stated.
- **Configuration ops inside a batch.** `setMeshReuseEnabled`,
  `setMeasurementReuseEnabled`, the new `setCacheLimits` and `clearCaches`,
  and (outside this RFC, but the same defect) `setStoreLocalCuts` change
  owner state that a rollback cannot restore. The RFC proposes a new
  `BatchOpKind::Session` that the dispatcher applies only after the batch
  commits, or that it refuses inside a batch. Owner choice; refusing is
  simpler. Today `setStoreLocalCuts` is `ReadOnly` and leaks through a
  failed batch.
- **Checkpoints** neither save nor restore caches, as today. After a restore,
  content entries are warm hits for the restored state, which is desired.
  Identity entries miss because restore bumps the generation. `PreparedMoves`
  stays outside checkpoints and re-checks its source on every call
  (unchanged).
- **The fault matrix** (`bindings/transaction_tests.rs`) gains one row per
  stage: an injected failure mid-batch leaves the cache stats consistent and
  every later read exact.

## 6. Alternatives considered

- **Keep thread-locals, add an owner-controlled `reset` on kernel drop.**
  This fixes only lifetime. Sharing, threads, accounting and the native gap
  remain.
- **Put concrete cache fields in `OperationContext`.** Not possible: L0 cannot
  name L2/L3 types, and it would also break `Clone + PartialEq` value
  semantics and the planned replayable context serialization.
- **Separate `&CacheContext` parameter on every entry point.** This is the
  right end state for context-taking APIs, and stage 8 gets there for them.
  As a first step it would mean doubling roughly 100 public measurement,
  tessellation, validation and query functions before any cache moves.
- **One global `Mutex` around all caches.** Simple, but it serializes
  unrelated families natively, and it makes the re-entrant `try_lock`
  fallback trip on legitimate nesting (measurement inside tessellation
  inside a move).

## 7. Open questions for the owner

1. Should `Model` default to `translated_reuse` on, matching WASM, or only
   to the exact paths? (§3.7)
2. Should configuration ops inside `executeBatch` be deferred to commit or
   refused? (§5.1)
3. Is a probe-kernel hit worth keeping the module-level default shared scope
   once kernels have explicit lifetime? The alternative is that consumers
   pass the parent's scope to the probe kernel, which is more explicit. It
   would need a consumer change in OpenZCAD. The RFC keeps the shared
   default.
