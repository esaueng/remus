# PERF-O06-NR: NURBS ray evaluation reuse

Tier: core. The existing NURBS ray solver supports point classification inside
live-session Boolean and face-edit recognition operations. This work changes
that existing solver; it adds no new capability or package boundary.

The source-32 recognition profile in
[PERF-O06](perf-o06-recognition-graph.md) attributes about 47% of recognition CPU
time to line–NURBS intersection. That profile identifies a path to investigate,
not a timing baseline for this change. The baseline for this lane is
`d78230e122fb075ff0936608029943bce42e3fbf`.

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
surface/grid preparation stays outside both timed revisions. Quiet native
before/after timings, shipping WASM timings, full-workspace qualification and
fresh/installed paired-package checks are pending. No elapsed-time improvement
or package release is claimed by this report yet.
