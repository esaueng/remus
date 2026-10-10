# Edit-to-frame performance

This change supports OpenZCAD's H02/W02 work: publish accepted geometry before
quantities and feature proofs, then complete those analyses for the same
document revision. OpenZCAD owns scheduling, transport and rendering; Remus
keeps exact geometry and numerical policy authoritative.

The kernel changes are:

- Immutable NURBS knots, control nets and weights share `Arc` storage across
  topology clones. Evaluation, validation, constructors and serialized array
  formats retain their contracts. Derived evaluation caches are separate.
  Clone bookkeeping is outlined in release builds to avoid duplicating it
  throughout modeling code.
- `checkpointMemoryStats()` deduplicates topology snapshots and NURBS storage,
  includes retired slots, and estimates the next mutation's copy cost.
  `linearMemoryBytes` reports allocated WASM pages (null in native builds).
  Estimates exclude allocator overhead, evaluation caches, pcurves, attributes,
  journals, undo/session state and GPU data. They are admission diagnostics,
  not a bound on process RSS. The consumer additionally keeps its existing
  checkpoint-count and replay-work limits.
- `setPerformanceTracing(true)` and `drainPerformanceTrace()` expose bounded
  module-local phase timings. Tracing is off by default and retains at most
  4096 events. Labels contain no model contents. Nested durations overlap;
  do not sum them as independent costs. Restore and the first copy-on-write
  mutation are timed alongside volume, meshing and recognition substages.
- `solidEdgeRelationsSubset()` batches selected edges with the same default
  probe and whole-solid material classification as `solidEdgeRelations()`.
  Caller order is preserved; duplicate, foreign, retired and invalid handles
  fail closed. Full feature recognition still evaluates its full evidence.
  Verdicts from a different probe policy are never substituted.
- Grouped mesh outputs support consume-once `takePositions()`, `takeIndices()`
  and `takeFaceOffsets()`. Existing getters still copy. Returned arrays are
  owned by JavaScript; subsequent consumption returns empty arrays.
- Volume tessellation omits boundary shading-normal reconstruction. It uses
  the existing positions, indices, angular/linear tolerances, welding,
  deduplication, repair, route selection and integration. Display/export
  meshing keeps its normal path. Cold and warm quantity meshes must match
  ordinary closed meshes bit for bit, and later display normals must match.

The public 160-face STEP fixture identifies curved-face meshing as the dominant
volume substage; boundary normals are a smaller avoidable cost. Content-key
construction and checkpoint copying were small in that fixture. These probes
justify skipping shading work, not changing measurement tolerance, seeding a
derived quantity, or reusing unqualified trimmed-face integrals.

Verification covers cold/warm mesh parity, NURBS clone/evaluation/wire-format
parity, checkpoint sharing/retirement, consume-once ownership, selected-edge
bit parity, recognition goldens, cavity/orientation, open-mesh refusal/fallback,
NURBS volume and placement invariance. The paired WASM packages must pass the
ordinary `cargo xtask wasm-build` validation and shared-version guard. The
kernel optimizer runs `-Oz --enable-simd`; consumer size thresholds
remain unchanged.

The optimized kernel is 10,478,269 raw bytes, 2,451 bytes smaller than the
10,480,720-byte committed base. It fits the existing 10 MiB hard limit and
still crosses the existing 9 MiB advisory review threshold. Numerical policy
and the build optimizer flags remain unchanged.

Consumer qualification also requires valid on-face text embosses. The paired
package update includes a planar face-splitter repair: nested section contours
belong to their immediate enclosing region, while only outermost contours
remain holes in the remainder. Exact curve uses, containment policy and
numerical tolerances are preserved. Coplanar glyph fuse regressions now check
strict orientation as well as the existing volume/topology/trim assertions.
OpenZCAD keeps strict union acceptance and checks existing holes independently
with closed-form volume and exported point classification.

Consumer timings and full application gate results belong to the coordinated
OpenZCAD implementation plan. A single WASM call is still synchronous; this
change does not introduce browser threads, GPU modeling or a second analysis
arena. General mixed-history reuse remains conditional on workload evidence
and an audited persistent-reference/dependency contract.

The [face-edit responsiveness follow-up](performance/perf-face-edit-responsiveness.md)
documents allocation-light NURBS partials, orientation-only accumulation,
standalone area memoization and display-priority retention within the existing
face-mesh budget. Those changes preserve the numerical policies above;
consumer scheduling and time-to-frame still need independent qualification.
