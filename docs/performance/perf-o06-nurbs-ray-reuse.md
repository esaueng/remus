# PERF-O06-NR: NURBS ray evaluation reuse

Tier: core. The existing NURBS ray solver supports point classification inside
live-session Boolean and face-edit recognition operations. This work changes
that existing solver; it adds no new capability or package boundary.

The source-32 recognition profile in
[PERF-O06](perf-o06-recognition-graph.md) attributes about 47% of recognition CPU
time to line–NURBS intersection. That profile identifies a path to investigate,
not a timing baseline for this change. The baseline for this lane is
`866581e9d7a9362022d68f950ebddcf155495da0`.

The solver already reuses surface seed grids, first-partial scratch space and
exact Newton cycles. The fresh diagnostic run records the maintained holder
edit at distances 5.04 and 5.06. Each recognition makes 1,464 line–NURBS calls
and starts 27,577 Newton refinements. The two edits perform 269,591/269,729
position evaluations and 242,014/242,152 first-partial evaluations. Exact
parameter repetitions account for about 48% and 47% respectively. No complete
surface/ray call repeats, so this evidence supports reuse within a call.

Observation-only simulations use the same combined position/partial entries
and eviction rule as the implementation:

| Entries | Position hits | First-partial hits |
| --- | --- | --- |
| 16 | 15.5% | 13.0% |
| 32 | 25.8% | 23.9% |
| 64 | 35.2% | 33.8% |

These are potential evaluation savings on the baseline, not elapsed-time
improvements. The [raw diagnostic summary](evidence/perf-o06-ray-crossings/diagnostic-summary.json)
contains both edits, tool/package identities and shipping-output comparisons;
the [diagnostic overlay](evidence/perf-o06-ray-crossings/diagnostic-overlay.json)
records the observer patch and hash. Diagnostic observers and their durations
are excluded from shipping timing measurements.

The implementation gives one ray query a fixed 64-entry table, approximately
6 KiB of transient storage. Keys compare both native parameter bit patterns;
collisions evict an entry and evaluate normally. The table stores positions
from `surface.evaluate` and partials from the existing `DerivativeScratch`
separately, preserving their arithmetic order. Verified span hints make later
scratch misses independent of skipped partial evaluations. A new query creates
a fresh table, with no persistent surface identity or invalidation policy.

Each candidate also carries its existing seed-grid position into iteration
zero. The grid's same-surface contract and identical native parameter
expressions make this the same evaluated point. The existing ray direction
norm is reused within refinement. The change retains all candidates, Newton
updates, cycle phases, returned root order and deduplication; convergence
thresholds remain `1e-10`/`1e-5`, spatial deduplication remains `1e-6`, and the
iteration limit and correctly rounded fused arithmetic remain unchanged.

Seven focused tests pass in both default and SIMD builds. Complete point and
parameter bits match the retained allocating solver for 1,560 deterministic
rays per build, including rational and repeated-knot surfaces, closed seams,
collapsed poles, degree-nine patches, scales 0.001/1/1000, rigid placement,
non-unit directions and exhausted Newton cycles. Separate Cartesian cylinder
roots, collision eviction, exact-key, bounded-storage and reused-refinement
controls exercise the new path. The native binding contract also passes over
six scaled/translated B-spline boxes with twelve convex edge verdicts and
independent inside/outside/boundary probes per fixture.

The maintained `nurbs_ray_cached` Criterion group checks Cartesian root
oracles before timing bilinear and quadratic hits, a closed rational-cylinder
secant and a miss. It includes each query's local workspace and returned roots;
surface/grid preparation stays outside both timed revisions.

## Quiet measured results

The baseline is the fresh d782 paired build above; the candidate runtime is
`715366e2720a8d94f38f8b6c3873cd9e7efdae33`. Both use local paired version
2026.1.61. The candidate kernel is 9,749,112 bytes, SHA256
`e4a3ed0a178ce0b268e2ae8e3cc7a59c47946a37b9c3efcf5b46e75173c5b55a`:
1,392 bytes above the fresh baseline. The translator is byte-identical.
These are fresh local artifacts, distinct from the committed version-60 pair.

Each schedule uses two fresh processes per revision, two warmup edits and six
retained edits per process: 12 retained edits per revision/schedule. Runs are
untraced and serial on one pinned CPU after validation/build jobs end. Production
memos and display-cache defaults stay enabled, module/JIT state stays warm within
a process, and every edit restores a fresh kernel body. Runtime, harness, native
executables and every package file are hashed before/after; all input guards pass.

| Maintained holder workflow, median ms | baseline | candidate | decrease |
|---|---:|---:|---:|
| Geometry-first recognition | 345.185 | 307.350 | 10.96% |
| Volume-first recognition | 344.533 | 306.573 | 11.02% |
| Geometry-first kernel + bulk mesh transfer | 690.617 | 652.684 | 5.49% |
| Volume-first kernel + bulk mesh transfer | 694.066 | 656.337 | 5.44% |

The pipeline includes move, validation, bounds, display tessellation, volume,
recognition and bulk mesh getters. Constructor/restore, browser rendering, GPU
work and presentation are outside this measurement. These retained process
samples establish a local workload comparison, not production p95.

Native Criterion uses the same benchmark source, profiling settings and default
features for both revisions, one serial process each and ten samples per group.
The printed time estimates and reported change use Criterion's respective
estimators; the latter is not recomputed from rounded estimates.

| Native carrier group | baseline estimate | candidate estimate | reported decrease |
|---|---:|---:|---:|
| Bilinear hit | 4.6996 us | 4.2355 us | 9.40% |
| Rational-cylinder secant | 10.813 us | 9.8377 us | 8.78% |
| Rational-cylinder miss | 56.836 us | 50.285 us | 11.56% |
| Quadratic hit | 4.8169 us | 4.4517 us | 7.64% |

## Final local qualification

Both schedules/rounds preserve all 36 source/warmup/retained snapshot pairs and
180 byte-exact arena/display artifacts, complete feature JSON and volume bits.
Separate untimed arena replays preserve 5,760 per-face area-bit pairs, whole area,
volume, bounds and complete edge-relation records; arenas remain unchanged.

All 8,078 workspace tests pass (38 existing skips), with a required GPU adapter
and serialized render tests. Documentation tests, 13 complexity guards, no-I/O
WASM contracts and all 52 committed census rows pass. Fresh optimized paired
WASM, its full smoke/installed consumer runs, publisher-normalized installed
consumers, paired version and trusted-base size gates pass. The kernel's growth
against the older committed pair includes inherited F6 support; the optimization
itself adds 1,392 bytes against a fresh build of the same baseline source.

[Local qualification](evidence/perf-o06-nurbs-ray-crossings/local-qualification.json)
records immutable source/package hashes and gate results. The build began before
a metadata-only commit amendment; the public runtime commit has the identical
complete Git tree, recorded in that evidence. Raw timing and exact-output evidence
are retained beside it. Hosted CI, package refresh, broader workload performance
and browser presentation are separate. Global NURBS intersection completeness
and foreign/mutated seed-grid validation remain outside this bounded reuse.
