# Remus kernel performance audit — evidence and design reference

**Audit date:** September 12, 2026. **Source baseline:** `ca91b956cdde99f78f048dd32b656b559ad14e7d`, refreshed `origin/main`, latest commit “fix(blend): complete B4 planar walking-trimmer gaps (#398).”

**Recommendation:** prioritize expensive NURBS validation/properties, document-wide transaction snapshots, repeated query preparation, and remaining pairwise algorithms. Build on those gains with incremental tessellation and memory management. Broader parallelism, explicit SIMD, GPU work and compiler tuning should follow measured workload evidence.

This audit provides **111 actionable work items across 14 areas**, with priorities, implementation approaches, acceptance checks, source anchors and execution phases. It is a comprehensive architecture-wide optimization inventory, not proof that every function has been profiled or that every proposed optimization will help. Some experiments should finish with a documented “no benefit” decision. The objective is the fastest correct kernel for declared workloads and resource budgets, including useful tail latency and memory behavior.

The evidence includes source inspection across all 15 workspace crates, executable benchmark/CI inspection, new bounded native probes, an existing real-model Criterion benchmark, and actual execution of the committed WASM package. It does **not** include a complete CPU/allocation profile census, every operation/corpus case, GPU measurements, real-browser timing, or a current head-to-head competitor run. This document records the audit baseline. The accompanying diagnostic harnesses and logs support the roadmap; no kernel optimization is implemented by publishing it.

## Using this roadmap

The [master roadmap](../kernel-maturity/roadmap.md#performance-work-packages) owns all 111 work packages, their dependencies, priorities, owners and implementation state. Its phase map replaces this audit's separate first-twelve queue. The [CSV](optimization-backlog.csv) and [JSON](optimization-backlog.json) are generated specification exports; regenerate them with `python3 scripts/sync-roadmap-inventory.py` rather than editing them independently.

Keep this audit's measurements, source references and evidence files fixed to their recorded SHA. Later measurements require a new dated evidence directory. The maintained [baseline runner](baseline.md) is the bounded O3.1a slice; it does not complete the architecture-wide M01/M06/M10 packages.

## Read this first

1. **NURBS-heavy validation and measurement already cost seconds on a real fixture.** Strict validation took about **1.616 s** and mass properties **3.677 s** on the committed hammer-holder STEP model. Make this a primary profiling workload, alongside procedural booleans. Start with evaluator work, allocation, trim preparation and duplicated calls; preserve all mathematical checks.
2. **Small edits pay for unrelated document state.** An otherwise empty topology transaction rose from **0.035 ms at 50 boxes to 13.797 ms at 3,200 boxes**. At the latter size, this nearly consumes a 16.7 ms frame interval before any modeling, although a UI may schedule modeling asynchronously. Redesign snapshot ownership and mutation-local rollback; preserve failure atomicity and stale-handle safety.
3. **Spatial acceleration often gets rebuilt before it can help.** `check` classification builds a face BVH inside each ray-crossing call; distance entry points rebuild bounds/BVHs and still prepare whole-face candidate lists. Introduce prepared queries, then safely invalidated persistent caches.
4. **Several remaining loops enumerate all pairs.** VE, EE and FF broad phases, body overlap partitioning, vertex healing, sewing and intersection-point chaining warrant explicit complexity work. An AABB rejection inside a nested loop reduces narrow-phase cost but leaves pair enumeration quadratic.
5. **Tessellation needs incrementality and dependency handling.** Native edge sampling and holed-planar CDT are already parallel. Shared-boundary Steiner insertion means unrestricted face-level parallelism would be unsafe. Resolve that dependency before broadening parallel execution.
6. **Do not adopt a “faster” evaluator by name.** The existing cached power-basis evaluator was approximately **6.5% slower at degree 3 and 44.6% slower at degree 9** on the two tested single-span rational grids, even after preparation was amortized. Multi-span surfaces and derivative workloads still need evaluation.

## What was measured

All native measurements below use Rust **1.96.0**, x86-64 Linux, an **AMD Ryzen 9 5900XT, 16 cores / 32 logical CPUs**, and the repository’s `profiling` profile: optimized code, debug information, no LTO. Most probes set `RAYON_NUM_THREADS=1`. They are diagnostic baselines, not production-release or cross-machine guarantees. CPU affinity/frequency and other machine activity were not controlled; raw logs and harnesses are supplied.

### Native structural probes at the audit source SHA

The scratch harness performs one warmup and records min/median/max of nine timed batches. Setup is excluded; clone/transaction measurements include dropping the copied state. The medians below are batch-normalized operation times, not p95 estimates.

| Probe                                    |                                 Smaller input |                                                Larger input | Interpretation                                                                                                                                                                                                       |
| ---------------------------------------- | --------------------------------------------: | ----------------------------------------------------------: | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| No-op transaction                        |                        50 boxes: **35.19 µs** | 3,200 boxes / 204,800 allocated entity slots: **13,797 µs** | Strong evidence that the whole-topology snapshot imposes document-size-dependent cost. This is an artificial isolation probe, not an ordinary no-op user command.                                                    |
| Topology clone                           |                        50 boxes: **38.27 µs** |                                  3,200 boxes: **13,324 µs** | Similar to the no-op transaction, including allocation/copy/destruction.                                                                                                                                             |
| Chain intersection points                |       128: **75.90 µs**; 512: **1,155.62 µs** |                                     2,048: **18,137.87 µs** | Approximately 15–16× runtime for each 4× input increase on a sparse straight chain.                                                                                                                                  |
| Cubic interpolation                      |          128: **16.20 µs**; 512: **94.35 µs** |                                      2,048: **1,069.02 µs** | Solver arithmetic is banded, but the matrix allocation still requests 8·n² bytes: 128 KiB, 2 MiB and 32 MiB respectively, excluding container overhead. These are source-derived allocation sizes, not measured RSS. |
| Standalone BVH build                     | 100 boxes: **35.91 µs**; 1,000: **964.33 µs** |                                    10,000: **15,663.90 µs** | Current SAH construction sorts on each axis and allocates suffix arrays recursively.                                                                                                                                 |
| Reused BVH overlap query                 |                      100 boxes: **0.0598 µs** |                                       10,000: **0.0887 µs** | One highly selective, warm query. Illustrates amortization; it is not a comparable full distance query or a claimed 100,000× kernel speedup.                                                                         |
| Surface evaluation, 100 points, degree 3 |                         Direct: **11.395 µs** |                                       Cached: **12.139 µs** | Existing cached evaluator loses on this fixture.                                                                                                                                                                     |
| Surface evaluation, 100 points, degree 9 |                         Direct: **52.148 µs** |                                       Cached: **75.416 µs** | Same conclusion; no general NURBS conclusion follows from two surfaces.                                                                                                                                              |

The harness asserts unchanged transaction slot counts, chain counts/lengths, interpolation endpoints, selective BVH results and agreement between direct/cached point evaluation. It does not establish full geometry qualification for proposed replacements.

### Existing real-model benchmark at the audit source SHA

`crates/io/benches/nurbs_properties.rs` imports the committed Shapr3D hammer-holder fixture, described by the benchmark as 42 NURBS faces including 38×58 bicubic control nets. Fixture import is outside the timed operations. The strict validation benchmark asserts that the report is valid; the mass-properties benchmark requires successful computation. This run does not add an independent mass-property oracle.

```text
nurbs_properties/validate_solid strict (hammer holder)
                        time:   [1.6129 s 1.6158 s 1.6191 s]
nurbs_properties/mass_properties (hammer holder)
                        time:   [3.6701 s 3.6772 s 3.6844 s]
```

These are Criterion time estimates and confidence intervals, not per-call minimum/median/p95. Ten samples were collected for each. Criterion extended the requested measurement time because one operation exceeded the one-second request. The raw log retains those warnings. This confirms expensive end-to-end work but does not yet quantify which internal function dominates; CPU/allocation attribution is the next task.

### Additional native probes

The maintained `batch_profile` example ran three repetitions and reports minima. At 50 versus 400 preexisting boxes:

| 150-operation workload      |  50 boxes | 400 boxes |
| --------------------------- | --------: | --------: |
| Read-only bbox/volume batch |  2.427 ms |  2.534 ms |
| makeBox batch               | 16.169 ms | 58.435 ms |
| Copy/transform/bbox batch   |  7.378 ms | 35.451 ms |
| Direct transform loop       |  6.357 ms | 50.082 ms |

The existing direct-transform example discards its `Result`, so its timing is supporting evidence, not a correctness gate. The new WASM harness lets thrown direct-call errors fail the run and checks batch error entries.

A 64-cylinder `compound_cut` diagnostic completed in **59.5 ms / 70 faces** at one Rayon thread and **57.1 ms / 70 faces** at four. Each is a single run with tessellation disabled; this provides no defensible thread-speedup claim. The sphere probe produced **9,800 triangles** at linear deflection 0.01/angular tolerance 0.1, with individual tessellations roughly 5.5–8.2 ms. Its sampled centroid sag was 0.00877; that sampled check is not a proof of global maximum deviation. These are smoke/profiling observations, not substitutes for exact geometry validation.

### Actual committed WASM package

The Node probe executes the package in the audited Git tree, using **Node 24.14.0**, package **2.130.14**, and WASM SHA-256 **`d6fad954f2ac834f0bf7209ee15d67e066d8f5d174e19a404ad1c94aa5633e2d`**. The raw WASM is **8,253,737 bytes**. A single cold `require` took 20.46 ms; it includes file reading/compilation/instantiation and is not a browser startup baseline.

**Provenance boundary:** package refresh commit `f19685684714003255f9a98e8e6a885d1419e5dd` declares source `8bc1d81e968d4d82bf7e642f4bf5ad464008472e`. It predates the native audit SHA. Consequently, these numbers establish behavior of the committed distribution and must not be used to calculate native-versus-WASM slowdown or exact-source parity.

Nine retained repetitions follow one discarded repetition for each scenario; values are medians. JSON output validation occurs after the timer. Seeds are built before the timer.

| 150-operation workload      | 50 existing boxes | 200 existing boxes | 400 existing boxes |
| --------------------------- | ----------------: | -----------------: | -----------------: |
| Read-only bbox/volume batch |          4.364 ms |           3.130 ms |           3.080 ms |
| makeBox batch               |         16.732 ms |          35.739 ms |          61.265 ms |
| Copy/transform/bbox batch   |          7.856 ms |          19.987 ms |          37.356 ms |
| Direct transform calls      |          6.836 ms |          25.412 ms |          51.549 ms |

The nearly flat read-only path agrees with the existing shared-snapshot fast path. Mutation remains sensitive to document size. This is the strongest browser-distribution evidence for prioritizing transaction architecture; browser-engine and allocation profiles remain outstanding.

## Existing optimizations and roadmap reconciliation

The audit used an isolated archive of freshly fetched `origin/main` at the source SHA above. A live PR/issue inventory found two dependency PRs (#401 and #402) and no open issues at the start of this audit; that is a point-in-time observation, not a reservation of future work.

| Area                          | Verified current state                                                                                                                  | Roadmap treatment                                                                                                                          |
| ----------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| O3.1 benchmarks               | Completed: 11 Criterion target files across math, algo, blend, operations and IO. The hosted workflow runs six target suites.           | Expand coverage and scenario quality; do not recreate the already-completed initial benchmark work.                                        |
| Profiling documentation       | Still refers to five operation benches, a retired competitor script and old workflow behavior.                                          | Reconcile M10 with actual executables. `bench-full` only runs operations.                                                                  |
| O3.2 spatial cache            | Pending; some contexts/cache structures exist but general revisioned reuse is absent from inspected query paths.                        | Q01 first, Q02 only after an invalidation design; extend rather than duplicate existing caches.                                            |
| O3.3 SIMD                     | Explicit algorithmic SIMD evidence remains pending; WASM build flags already enable simd128.                                            | Instruction availability is not proof of useful vectorization or measured gain.                                                            |
| O3.4 incremental tessellation | Pending. Shared edge data is constructed within a tessellation call.                                                                    | D01–D03, with dependency-aware invalidation.                                                                                               |
| M8.2 performance gates        | Pending; benchmark PR activation is label-gated, threshold is 200%, and `fail-on-alert` is false.                                       | M03 needs an actual enforced policy, calibrated to runner variance.                                                                        |
| M8.3 parallel tessellation    | Status says pending; code already parallelizes native edge sampling at ≥32 edges and holed-planar CDT at ≥2 jobs.                       | Record partial implementation, then qualify and extend the rest of the pipeline. WASM branches remain sequential.                          |
| M8.4 parallel booleans        | Pending.                                                                                                                                | B11 requires pure task outputs, stable commit order, budgeting and counter aggregation.                                                    |
| M8.6 arena compaction         | Pending, with a dedicated deferred design describing stable-ID hazards.                                                                 | T06 is a versioned architecture project, not removal/reuse of array slots.                                                                 |
| M8.7 WASM threads             | Pending, owner-gated in the existing program.                                                                                           | W07 follows native isolation and application deployment compatibility.                                                                     |
| Compound booleans             | `fuse_all` partitions bodies, uses analytic cylinder clusters and one GFA arrangement for connected groups with fallbacks.              | Optimize partitioning and remaining fallbacks; “replace sequential fuse with batching” is not an accurate description of all current code. |
| Numerical kernels             | Stack-backed basis buffers, cached max weights, uniform-knot/power-basis support and banded interpolation arithmetic already exist.     | Focus on remaining result allocations, dense storage, repeated work and demonstrated workloads.                                            |
| Transactions/checkpoints      | Native transactions still deep-clone; read-only/unknown WASM dispatch uses an Rc snapshot and checkpoints share topology.               | Optimize mutating operations and first write after checkpoint, preserving existing cheap reads.                                            |
| Healing/classification        | The previous unify_faces group rescans and repeated per-subface GFA geometry builds have already received fixes.                        | Preserve those guards; the remaining pair loops and cross-call reuse are distinct work.                                                    |
| Distribution                  | Kernel/translators are separate packages; grouped binary mesh output already exists; current size policy is 9 MiB review / 10 MiB hard. | Measure actual consumer use and transport costs; do not propose these as new features.                                                     |

The previous O3.1 profile census remains useful historical evidence, but its recorded source is `86c57df...`; the old CPU percentages have not been presented as current measurements here.

## Highest-value implementation designs

### Transactions: make local edits pay for local changes

Measure every snapshot on a representative direct/batch call stack before changing ownership. A transaction coordinator can give nested operations savepoints under a single outer transaction. The inner operation still needs a valid independent transaction when called directly. Batch semantics commit or roll back each item; moving the snapshot outside the whole batch would silently change behavior.

Evaluate two approaches: a mutation log with before-images and allocation watermarks, and chunk/page copy-on-write with shared immutable geometry. Mutation logs make small writes cheap but require complete write interception, including callers holding mutable entity references. Chunking simplifies some snapshot semantics but may complicate existing contiguous iteration APIs and increase pointer overhead. Use a prototype comparison and select one coherent design.

Rollback must restore topology, retirement, attributes, journal and auxiliary session state as appropriate. Newly allocated handles from a failed operation stay permanently invalid. Nested failure, cancellation, rejected validation and checkpoint restore all need independent tests. Compaction is a later, explicitly versioned contract; removing tombstones in place is not safe.

### Queries: prepare once, invalidate completely

Start with explicit operation-local `PreparedSolidQueries`-style state, which avoids cache lifetime ambiguity. Reuse face IDs, bounds, analytic classifications and projected trim loops across rays/points. Benchmark preparation and query work separately. A persistent cache can follow after mutation tracking is complete.

A safe key needs topology identity/incarnation, body identity, relevant geometry/boundary revision, tolerance and query policy. The current mutation tick is tied to journal/restore behavior; it must not simply be treated as a universally monotonic cache generation. Restore or rollback can revisit an old tick. Introduce a cache invalidation generation that cannot collide across those transitions, or clear the cache. Conservative whole-topology invalidation is a sound first step; journal-only invalidation is insufficient when unjournaled mutable access exists.

Bounds used for pruning must enclose the entire qualified finite entity. Sampled boxes and endpoint boxes for curved edges are not generally certificates. Unknown bounds require a non-prunable side path. Keep results deterministic and place derived caches in a layer that respects the existing dependency graph; topology must not acquire a dependency on check/operations or carry expensive derived caches into every snapshot by accident.

### Pairwise algorithms: reduce candidate enumeration

For sparse data, generate nearby pairs with sweep-and-prune, grids or BVHs before doing geometry. Track candidate enumeration separately from successful interferences. Retain worst-case budgets because densely overlapping geometry can still create a quadratic number of real interactions.

In intersection chaining, two explicit quadratic steps remain: the threshold-neighbor graph and nearest-unused-point ordering. The endpoint selection also uses `comp.contains` inside neighbor counting even though the components came from that same graph; dense inputs can amplify this further. Preserve the existing graph and tie/order semantics during an optimization. A nearest-neighbor heuristic that jumps between close branches is not a safe replacement.

### NURBS properties: fuse useful work, preserve precision

At each Gauss point, `Accumulator::add` calls `evaluate` and `partials`. The latter already computes both derivatives together, so another proposal to “combine du and dv” alone is stale. A more useful primitive returns position and derivatives from one basis/homogeneous accumulation, with a caller-provided result buffer. `NurbsSurface::derivatives` already uses stack scratch internally but allocates its nested result vector. These are source-supported targets; profile their share of the measured seconds-long fixture first.

Reuse trim preparation and basis values at repeated quadrature coordinates. Adaptive quadrature is a separate, higher-risk experiment: this kernel has deliberate trim-aligned integration, periodic branch handling and convergence requirements. Fewer samples alone is not evidence of equivalent accuracy. Property caches and validation certificates must refer to the same immutable geometry, tolerance and options; keep independent full validation at required trust boundaries.

### Tessellation: establish boundary dependencies before concurrency

The current pipeline builds shared edge samples, runs some planar jobs in parallel, then may splice constraint-recovery Steiner points into shared boundaries before tessellating neighbors. Treat those splices as dependencies. Prepare/refine a common boundary plan, then allow independent local face triangulation, followed by deterministic indexed assembly. Alternatively, stage results and reconcile/refine affected neighbors explicitly. Benchmark the complete dependency graph, not only local face meshing.

An incremental cache must invalidate both sides of an edited edge, including adjacent faces whose own surface did not change. Include display versus boolean purpose in the key; those paths intentionally use different sample policies. A visual preview may choose a coarser requested deflection, but the kernel must not secretly loosen tolerances to improve a benchmark.

Source evidence for the principal findings: [Transactions](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/topology/src/transaction.rs#L40), [NURBS derivative results](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/surface.rs#L389), [Quadrature accumulation](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/properties/face_integrator.rs#L1905), [Classification BVH rebuild](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/classify/mod.rs#L311), [Distance preparation](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/distance/mod.rs#L50), [Intersection chaining](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/intersection/chaining.rs#L184), [Dense interpolation storage](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/fitting.rs#L218).

## Roadmap and dependencies

Execution phases and all package dependencies now live in the [master roadmap](../kernel-maturity/roadmap.md#performance-phases). Select by measured workload frequency × time/memory share × reducible fraction, adjusted for confidence, risk and effort. Treat latency, throughput, cold startup and memory as separate objectives.

Amdahl's law bounds an expected total speedup by `1 / ((1-f) + f/s)`; retime the full workflow after each change. The audit's suggested 10% target improvement is a proposed adoption rule, not an enforced project policy. O3.3 retains its named native/WASM SIMD ≥1.5× benchmark-or-no-gain gate plus full-workload and code-size checks.

## Qualification matrix

| Dimension         | Required workload variation                                                                                                                                        |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Geometry          | Planar, analytic curved and NURBS; high degrees and weights; mixed carriers; primitives and imported models.                                                       |
| Topology          | Solid, sheet, wire, Compound, disjoint regions, cavities, shared aliases, periodic seams, holes and nonmanifold/malformed refusals.                                |
| Scale             | Tiny through large extents, large translations, anisotropy, near tangency, coincident boundaries and thin walls; choose fixtures within declared support.          |
| Size              | Geometric progressions of faces/edges/points/constraints/bodies; sparse interactions and dense legitimate worst cases.                                             |
| Lifetime          | Fresh document, long edit history, many checkpoints, create/delete, failed/cancelled edits, restore and fixed-live-size churn.                                     |
| Execution         | Native facade, direct WASM, executeBatch/executeBatchV2, split translators; cold and warm runs; headless Node and real browser engines.                            |
| Parallelism       | Serial and several thread counts, different scheduling, repeated process runs, deterministic output/diagnostics and combined work counters.                        |
| Resource behavior | Latency distribution, total CPU, allocations, peak live memory, RSS, WASM high-water pages, entity growth and cancellation latency.                                |
| Quality           | Independent volume/material/area/distance/derivative oracles as relevant, valid topology, watertightness, declared deflection, normals, lineage and typed refusal. |
| Delivery          | Source and fixture hashes, exact generated WASM hash, compiler/features, actual installed consumer execution and exact-head hosted checks.                         |

### Cache and transaction invariants

- Raw mutable access, pcurve/domain changes, retirement, transform and tolerance changes must invalidate affected derived state.
- Restore/rollback cannot reuse a cache entry from a different state that happens to have the same handle or mutation tick.
- Cavity shells and inner wires participate in invalidation, measurements and traversal.
- Shared boundaries invalidate all dependents; equal carrier equations do not imply equal trimmed faces.
- Cache entries are bounded and evictable; adding a cache must not multiply snapshot size or retain entire dead models.
- A cancelled/failed operation preserves exact logical state and keeps failed handles permanently invalid.
- New parallel operations preserve a documented deterministic error, accumulation and allocation-order contract.

### What not to ship as an optimization

Do not loosen modeling tolerance, reduce required validation, silently substitute a mesh, remove difficult branches, change curve parameterization, use unsafe bounds, reuse stale state, change public units/formats, or make all cores the default regardless of workload. Do not use fast-math reassociation or f32 in exact geometry without a separately justified mathematical contract. Reduced precision may be appropriate at a documented render-output boundary; it is not a kernel-wide optimization.

SIMD and PGO are experiments, not assumed wins. Rust's PGO workflow requires instrumentation, representative training, profile merging and a rebuild; the proposal is to test this on held-out kernel workloads after structural work. See the [rustc PGO documentation](https://doc.rust-lang.org/rustc/profile-guided-optimization.html). Native thread-pool sizing can be controlled and benchmarked via [Rayon's ThreadPoolBuilder](https://docs.rs/rayon/latest/rayon/struct.ThreadPoolBuilder.html). Browser threading additionally involves worker/shared-memory and hosting requirements, as illustrated by the [wasm-bindgen parallel example](https://wasm-bindgen.github.io/wasm-bindgen/examples/raytrace.html); any resulting deployment changes require their own review and authorization.

## Reproducing this audit

All files below are in the accompanying evidence directory. Native binaries were built from the isolated source archive; only the diagnostic `audit_probe.rs` was added to that archive. The production implementation was unchanged.

Run from the repository root. Archive the pinned baseline rather than benchmarking whichever commit is currently checked out. This leaves the working tree untouched. The diagnostics are copied into the temporary source tree before building; dependencies must be cached for the offline build.

```bash
audit_source=$(mktemp -d)
audit_evidence="$PWD/docs/performance/evidence/2026-09-12"
git archive ca91b956cdde99f78f048dd32b656b559ad14e7d | tar -x -C "$audit_source"
cp "$audit_evidence/audit_probe.rs" "$audit_source/crates/operations/examples/audit_probe.rs"
export CARGO_TARGET_DIR="$audit_source/target"
export CARGO_BUILD_JOBS=2
export RAYON_NUM_THREADS=1

(
  cd "$audit_source"
  cargo build --offline --locked --profile profiling \
    -p remus-operations --example perf_probe --example profile_boolean \
    -p remus-wasm --example batch_profile
  cargo build --offline --locked --profile profiling \
    -p remus-operations --example audit_probe

  "$CARGO_TARGET_DIR/profiling/examples/audit_probe"
  "$CARGO_TARGET_DIR/profiling/examples/batch_profile" 3
  "$CARGO_TARGET_DIR/profiling/examples/perf_probe"
  "$CARGO_TARGET_DIR/profiling/examples/profile_boolean" cylinders
  RAYON_NUM_THREADS=4 "$CARGO_TARGET_DIR/profiling/examples/profile_boolean" cylinders

  cargo bench --offline --locked --profile profiling \
    -p remus-io --bench nurbs_properties -- \
    --warm-up-time 0.5 --measurement-time 1 --sample-size 10
)

node "$audit_evidence/wasm_probe.cjs" "$audit_source/crates/wasm/pkg"
```

The Node script accepts the package directory as its first argument, and defaults to the current checkout's package when omitted. It records the actual version and WASM SHA-256. Compare those with the recorded provenance before comparing timings. The original native build compiled operations and WASM examples together; the subsequent isolated example/bench builds may resolve a different feature union and rebuild dependencies. Timing comparisons must pin selected targets/features as well as source. The archived logs retain the original run output with temporary paths normalized to `<audit-source>` and `<audit-target>`.

Future implementation PRs should run the repository's documented build/test/lint commands and relevant semantic/scaling regressions, regenerate distributed packages when needed, and execute the actual installed direct/batch WASM paths. Broad hosted validation remains separate from these audit probes. **This audit did not run the full workspace test suite or claim release readiness.**

## Evidence index

- [evidence/audit_probe.rs](evidence/2026-09-12/audit_probe.rs), [evidence/audit_probe.txt](evidence/2026-09-12/audit_probe.txt): structural probe source and raw min/median/max data.
- [evidence/batch_profile.txt](evidence/2026-09-12/batch_profile.txt): maintained native batch example; minimum-only output, three repetitions.
- [evidence/perf_probe.txt](evidence/2026-09-12/perf_probe.txt): maintained cut/sphere probe; sampled mesh checks.
- [evidence/cylinders-threads-1.txt](evidence/2026-09-12/cylinders-threads-1.txt), [evidence/cylinders-threads-4.txt](evidence/2026-09-12/cylinders-threads-4.txt): single-run compound cut smoke measurements.
- [evidence/nurbs-properties.txt](evidence/2026-09-12/nurbs-properties.txt): current-source real-model Criterion output, including all warnings.
- [evidence/wasm_probe.cjs](evidence/2026-09-12/wasm_probe.cjs), [evidence/wasm_probe.txt](evidence/2026-09-12/wasm_probe.txt): actual committed-package direct/batch execution and artifact provenance.
- [evidence/run_metadata.json](evidence/2026-09-12/run_metadata.json), [evidence/provenance.json](evidence/2026-09-12/provenance.json): source, environment, hashes and run outcomes.
- [source-references.json](source-references.json): verified source anchors for every inventory area.
- [optimization-backlog.csv](optimization-backlog.csv) / [JSON](optimization-backlog.json): all work items, including priorities, phases, acceptance checks and explicit dependencies.

The report is the actionable roadmap; the raw measurements are its bounded evidence. Re-run the baseline when source, compiler, package, fixture or relevant hardware changes.

## Complete work inventory

The full inventory has moved to the [master performance register](../kernel-maturity/roadmap.md#performance-work-packages). All 111 IDs, priorities, phases, explicit dependencies, work descriptions, acceptance checks, effort/risk and source anchors are preserved. The CSV/JSON exports retain their existing schema and short IDs.
