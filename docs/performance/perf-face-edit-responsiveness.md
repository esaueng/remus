# Filleted face-edit responsiveness

The implemented slice reduces repeated analysis and avoids fine measurement
meshes evicting the display working set. It does not change push/pull geometry,
strict validation policy, public measurement accuracy or meshing tolerance.
Application scheduling remains the consumer's responsibility.

The motivating workload is the imported Shapr3D hammer holder at
`crates/io/tests/data/shapr3d_hammer_holder.step`: 160 faces, including 42 NURBS
surfaces and analytic blend patches. A face drag returns a new body while most
face support surfaces are unchanged or translated. The kernel must both reuse
unchanged work and calculate new requests under their original policies.

## Implemented changes

### Reusable first-order NURBS solves in Newton iteration

`crates/math/src/nurbs/intersection/line.rs` uses `DerivativeScratch::partials_from`
instead of populating a general derivative table. Its position and residual
still come from the original independent `evaluate` call; substituting the
scratch solve's position would change arithmetic and potentially Newton's
iteration history. Seed-grid selection, cycle handling, budgets, exits and
intersection results retain their policies.

`crates/math/src/nurbs/projection.rs` reuses one scratch object through each
surface Newton refinement. Position and first partials retain the values of
the old `derivatives(u, v, 1)` table. The negligible-step exit retains its
separate historical position evaluator. Parameter wrapping, seeds, tolerance,
convergence tests and iteration budgets remain unchanged. A verbatim copy of
the old Newton path supplies a differential oracle.

The existing span hints, repeated-u basis reuse and equal-weight shortcuts are
reused; they are not new algorithms introduced by this change.

### Orientation requests omit unused moments

`crates/check/src/properties/face_integrator.rs` adds a runtime `volume_only`
policy to the existing fixed-rule integrator and accumulator. Full and volume
requests share the same integration functions and carrier specializations;
there is no additional const-generic volume specialization or duplicated
integration pipeline. The existing area-only specialization is unchanged.

On fixed parametric paths, the accumulator keeps the same surface evaluations,
Gauss samples, order and accumulation arithmetic for area, signed volume and
vector area, then skips centroid and inertia updates when `volume_only` is
set. Planar faces retain their existing shared boundary integrators, which
still calculate full contributions. Adaptive and full-property requests keep
the flag unset. Physical area still participates in a volume request: the
rigid face-move volume-change guard compares it before and after a translation.

The content-keyed face-integral cache records whether an entry contains a full
contribution. A partial entry cannot answer a full-property request, even at
the same content and reference. A full entry can answer a volume request.
Existing reference and translation reuse remain available for volume terms;
their existing rounding allowances remain unchanged. Full mass properties,
default Gauss order, orientation thresholds and validation verdicts keep
their contracts.

### Exact scalar memo for standalone NURBS areas

`crates/operations/src/measure/nurbs_area_memo.rs` memoizes only areas produced
by the existing standalone NURBS tessellation and triangle-area summation.
That path reads the complete support surface, rather than its outer-wire
trim. Reusing an unrelated solid display mesh or trimmed quadrature would
change the value and is deliberately excluded.

The complete key records degrees, knot vectors, control-point and weight grids
including row lengths, reversal, deflection and the standalone sampling policy.
Reals compare by bits, including signed zero. Hash equality only selects
candidates; complete keys must agree. Thus equal support content under different
handles can reuse a value, while changed geometry or policy misses. Translated
or approximately equal content is not reused.

Faces with inner wires bypass the memo before lookup and keep their established
standalone-tessellation refusal. Errors are never stored. An oversized key,
failed initial key reservation or unavailable cache borrow takes the original
calculation. No borrow is held across that calculation.

The new NURBS-area library memo is opt-in per thread, with defaults of 512
entries and 8 MiB estimated retained key/entry bytes. FIFO eviction enforces
both limits. This is not a bound on allocator slack, process RSS or WASM
linear-memory reservation.
`BrepKernel::new` initializes this memo for browser consumers, alongside the
existing face-integral, volume and face-mesh caches. It stores standalone face
area readings, not complete feature-recognition results. Repeated initialization
preserves existing entries, counters and caller-chosen limits, including a zero
bound used to disable retention. Clear, disable, limit and statistics functions
are exported from `measure`.

### Display priority in the existing face-mesh cache

`crates/operations/src/tessellate/face_cache.rs` still uses a shared per-thread
cache with the same defaults: 4096 entries and 32 MiB estimated bytes. It now
tracks separate display and quantity FIFO queues. Quantity admission can evict
only quantity entries; when protected display entries consume the remaining
budget, the new quantity mesh is calculated normally and is not retained.
Display admission and explicit reductions of the bounds evict quantity entries
first, then display entries.

A quantity entry that successfully replays for display is promoted in place.
There is no second retained mesh and no increase in configured total limits.
Promotion occurs before the whole tessellation finishes, so a later failure
can leave existing retention metadata changed; new captures still commit only
after the complete tessellation succeeds.

Retention is not an input to meshing. Geometry, shared boundary-plan chains,
linear/angular tolerances, mesher policy, translation checks and pool-conflict
fallbacks remain in force. Measurement meshing still skips shading-only
boundary normal reconstruction. A later display request evaluates any missing
normal contributions under its original policy. Protected entries retain FIFO
order within their class; this is not an LRU or an unbounded per-document cache.

## Remaining costs and limits

Volume remains a separate numerical dispatcher. Analytic qualified families
keep their existing integrals; mixed NURBS/blend bodies may require a closed
whole-solid mesh. The measurement tolerance is still capped at
`min(requested, max(bbox_diagonal * 5e-5, 1e-9))`. A face edit can change the
diagonal and therefore the effective tolerance. Exact per-face mesh keys
remain tolerance-specific, even if the geometry of many faces is unchanged.
Display-priority retention addresses eviction, not those compulsory misses.

A newly edited body's first `solid_volume` reading remains a real measurement.
The removed source-volume-plus-swept-prism seed is not restored: source and
result meshes have different bias, and that seed made a reading depend on
cache warmth. The orientation probe establishes a sign and is not substituted
for the public volume. Lower-order quadrature, coarser hidden tolerances and
skipped strict checks are not part of this implementation.

The consumer can publish an accepted mesh before completing volume and feature
recognition for that same revision. Doing so needs request coalescing, stale
analysis rejection and explicit pending quantities. This kernel change does
not implement the ZCAD event loop, worker protocol or renderer, and it does not
establish a browser time-to-frame or an OCCT face-edit speed ratio.

## Qualification and measurement

The focused regressions cover:

- Scalar and SIMD first-partial equivalence and the pre-change projection
  Newton oracle, including rational weights, repeated knots, higher degrees,
  shifted domains, periodic carriers, collapsed surfaces and budget exits.
- Full-versus-partial area/volume/vector-area bits, reversed and periodic
  NURBS faces, reference/translation reuse and a partial-to-full cache request
  at exactly the same reference.
- Standalone NURBS area bits against fresh tessellation, changed support and
  request data, clone/rollback reuse, collision handling, holes, errors,
  oversized keys, borrow fallback, count/byte limits and repeated initialization.
- Quantity churn under the shared mesh count/byte limits, quantity-to-display
  promotion, clear/disable/shrink behavior and fresh-reference geometry/normals.
  Feature-recognition goldens and edited-holder volume/mesh checks remain
  required integration gates.

### Measured environment and baselines

The final measurements ran on Linux x86_64, an Intel Xeon Platinum 8573C,
with the runner and children pinned to CPU 0. Native builds used Rust 1.96,
release optimization, LTO and one codegen unit; Rayon was restricted to one
worker. The native baseline is source `61ed862` (after #976), so its matched
comparison isolates this patch. Native processes alternate old/new launch
order and reverse variant order in the second round. Each variant has 24
measured edits: 12 per process across two rounds, after three warmup edits.

WASM ran under Node 24.19.0, using the production dual-package SIMD build
with SIMD128 and production 32 MiB face-mesh bounds. Its frozen shipped
`2026.1.45` baseline was built at `a04b5c86` (#975); the candidate source also
includes #976. WASM therefore measures the package-to-current-source change,
not this patch alone. Each schedule has 12 measured edits per package: six
per process across two rounds, after two warmups. Process order alternates.

Both workloads move the same outward +X cap by `5 + iteration * 0.02` mm.
Display uses deflection/angle `0.02/0.35`; volume and recognition request
`0.08`. Native restores each source with `Topology::clone`; WASM uses arena
serialization/deserialization. Native all-face area and repeat-volume
diagnostics run between edits outside timed intervals, but affect subsequent
cache history. WASM has neither diagnostic pass. These are different warmed
workloads, not interchangeable native/WASM scaling measurements.

Tables report milliseconds. p95 is the nearest-rank sample percentile; with
12 WASM samples it is the largest recorded value. Cloud tail variation and
the small sample counts limit extrapolation.

### Native stage measurements

Both versions use matched limits: 4096 entries with 32 MiB or 128 MiB for
face meshes. The 128 MiB variant is diagnostic, not a new production default.
All sampled retained count/byte values stayed within their configured bounds.
`Preview work` sums move, validate, bbox and display intervals; when volume
runs first, it excludes that preceding wait and is not first-visible latency.
`Total` sums all six outer intervals, excluding source cloning and diagnostics.

| Face-mesh bound / order | Stage | Old median | New median | Old p95 | New p95 |
|---|---|---:|---:|---:|---:|
| 32 MiB / volume first | Display mesh | 40.7 | 26.7 | 200.8 | 35.2 |
| 32 MiB / volume first | Feature recognition | 246.9 | 158.0 | 428.5 | 263.0 |
| 32 MiB / volume first | Volume | 498.2 | 385.9 | 644.6 | 463.2 |
| 32 MiB / volume first | Preview work | 78.9 | 56.1 | 228.9 | 71.0 |
| 32 MiB / volume first | Total | 835.3 | 603.1 | 1227.2 | 744.0 |
| 32 MiB / geometry first | Display mesh | 42.8 | 27.5 | 234.3 | 60.8 |
| 32 MiB / geometry first | Feature recognition | 224.9 | 158.3 | 342.1 | 254.9 |
| 32 MiB / geometry first | Volume | 435.8 | 444.4 | 786.2 | 585.6 |
| 32 MiB / geometry first | Preview work | 74.1 | 55.6 | 292.4 | 114.5 |
| 32 MiB / geometry first | Total | 776.9 | 696.5 | 1489.8 | 879.4 |
| 128 MiB / volume first | Display mesh | 27.1 | 25.8 | 53.5 | 51.1 |
| 128 MiB / volume first | Feature recognition | 218.4 | 162.7 | 328.7 | 309.8 |
| 128 MiB / volume first | Volume | 425.0 | 414.8 | 563.3 | 648.0 |
| 128 MiB / volume first | Preview work | 58.4 | 54.8 | 83.5 | 82.6 |
| 128 MiB / volume first | Total | 757.1 | 642.2 | 956.9 | 1040.4 |

The 32 MiB volume-first display stage had 290 timed-stage evictions before
and 10 after, with no candidate display call exceeding 100 ms in these 24
timed edits. Feature recognition falls at both cache bounds. Volume is still
costly and variable: the 128 MiB volume p95 increases from 563.3 to 648.0 ms.
This does not establish a dependable volume speedup.

Native source and edit checks match validity, volume bits, complete feature
records, triangle counts, recorded coordinate diagnostics and effective
measurement deflection. The coordinate diagnostic filters raw plane normals;
it does not independently prove movement of the reversed selected cap. The
WASM harness checks that cap's effective normal and edited vertex positions.
The 128 MiB pair also matches display fingerprints. At 32 MiB,
some old/new fingerprints differ; every candidate 32 MiB recorded display
fingerprint matches the corresponding baseline 128 MiB record. Earlier raw
baseline 32/128 diagnostics found only position differences up to
`2.84e-14` mm, with exact normals and indices. Candidate raw-array diagnostics
also pass at the representative snapshots described below. These fingerprints
and diagnostics do not establish native arena byte identity.

### WASM stage measurements

`Display mesh` includes the grouped binary tessellation call and bulk typed
array transfer. `Kernel pipeline` excludes the bulk getters, constructor and
source restore; it sums move, validate, bbox, meshing, volume and recognition.

| Order | Stage | Old median | New median | Old p95 | New p95 |
|---|---|---:|---:|---:|---:|
| Geometry first | Move | 81.2 | 81.8 | 116.8 | 143.6 |
| Geometry first | Display mesh | 67.1 | 49.2 | 377.8 | 140.4 |
| Geometry first | Feature recognition | 621.1 | 468.0 | 870.0 | 750.2 |
| Geometry first | Volume | 842.9 | 752.2 | 1045.2 | 1182.1 |
| Geometry first | Kernel pipeline | 1646.5 | 1296.1 | 2133.3 | 2038.6 |
| Volume first | Move | 78.4 | 67.3 | 111.5 | 142.2 |
| Volume first | Display mesh | 40.0 | 42.7 | 293.7 | 85.3 |
| Volume first | Feature recognition | 518.9 | 404.1 | 719.3 | 569.3 |
| Volume first | Volume | 678.9 | 709.3 | 875.9 | 872.8 |
| Volume first | Kernel pipeline | 1479.5 | 1278.2 | 1694.0 | 1561.5 |

The geometry-first kernel-pipeline median is 1.27 times faster and the
volume-first median 1.16 times faster in this run. Recognition improves in
both schedules. Volume does not improve consistently: its volume-first
median worsens from 678.9 to 709.3 ms, while its geometry-first p95 worsens
from 1045.2 to 1182.1 ms. Move and other short stages also show noisy tails.

All 36 compared source/warmup/timed snapshots qualify with exact reachable
Float64 arena documents, Float64 volume bits, complete feature records,
Float32 display positions/normals, indices and face grouping. Maximum
display-position difference is zero. Every edit validates and has a
watertight display mesh. These results measure Node WASM calls and transfer;
they do not measure ZCAD scheduling, browser rendering or an OCCT edit ratio.

The portable [WASM harness](../../scripts/perf/filleted-face-edit/README.md)
records package/module/wrapper/fixture hashes, runtime versions, affinity,
raw snapshots and qualification outcomes. It can reproduce either schedule
with frozen old/new dual-package directories. Investigation raw records are
`implementation[-baseline]-raw.json`, their `-summary.json` files, and
`wasm-qualified-comparison/comparison.json`; prototype timings are excluded.

### Completed gates and package bounds

Completed gates include formatting, all-target/all-feature clippy, 800 math
and 214 check tests, focused regressions, feature-recognition goldens, layer
boundaries, deterministic hashing and documentation paths. The ordinary
dual-WASM workflow, deep runtime verification and clean installed-tarball
smokes passed. These are local qualification results, not deployment evidence.

| Production artifact | Shipped baseline bytes | Candidate bytes | Change |
|---|---:|---:|---:|
| Kernel WASM | 10,424,251 | 10,424,472 | +221 |
| Translator WASM | 3,050,347 | 3,050,601 | +254 |

The kernel remains below the 10,485,760-byte hard cap by 61,288 bytes.
The pre-existing 9 MiB advisory is still exceeded and needs review; passing
the hard cap does not resolve that advisory. Package size and cache byte
limits describe different resources and neither is a process-RSS bound.

### Extended native drag sequence

A separate matched volume-first run uses 40 timed edits after three warmups,
one old/new process pair and the production 32 MiB bounds. It retains the
same between-edit diagnostics described above. This extends the cache-churn
sequence; it is one process pair, not a second two-round experiment.

| Stage or behavior | Old | New |
|---|---:|---:|
| Display mesh median | 48.7 ms | 27.3 ms |
| Display mesh p95 | 246.1 ms | 48.5 ms |
| Display calls exceeding 100 ms | 8 | 0 |
| Feature-recognition median | 246.2 ms | 159.0 ms |

The candidate's first 12 display calls have p95 48.5 ms and its last 12
33.2 ms; this sequence shows no late display slowdown. Maximum sampled
estimated retained mesh bytes are 33,554,367, within the 33,554,432-byte
bound. These samples do not establish continuous heap or RSS maxima. Every
record matches strict nonmesh values and repeat-volume bits, validates and
has a watertight display mesh. Raw records use the distinct
`final-long-drag[-baseline]-raw.json` and `-summary.json` files.

### Native display precision diagnostics

Raw old32/new32 Float64 arrays were compared at iterations 4 and 9 under both
volume-first and geometry-first orders. All four comparisons have byte-exact
normals and indices. Positions differ by at most
`2.842170943040401e-14` mm: volume-first iteration 4 reaches that maximum,
iteration 9 reaches `2.1316282072803006e-14` mm; geometry-first iteration 4 is
exact and iteration 9 reaches the latter bound. These are the same scale of
rounding differences already seen between the baseline's cache bounds.

The diagnostic runs match the final timing probe's fingerprints and exact
nonmesh fields for the first ten records in both orders. The raw check covers
these four representative snapshots, not every display array in the timed
runs. Separately, candidate 32 MiB fingerprints match baseline 128 MiB for
all 30 warmup/timed edit records across two rounds, and the matched 128 MiB pairs
are fingerprint-exact throughout. No timing from diagnostic runs is used in
the tables.

Earlier design and measured context:
[tessellation reuse](perf-d01-tessellation-reuse.md),
[recognition](perf-o06-recognition.md),
[orientation memoization](perf-v02-orientation-probe-memos.md), and
[edit-aware measurement](perf-v02-edit-aware-measurement.md).
