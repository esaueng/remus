# O06 subset: whole-solid feature recognition and edge relations

OpenZCAD rebuilds an imported STEP body after every direct edit and re-runs
imported-feature recognition on it: `recognizeFeatures` and
`solidEdgeRelations`, each over the whole solid. Neither result can be carried
across an edit (fillet bands change length, new bands appear), so both calls
have to be fast on their own. On `crates/io/tests/data/shapr3d_hammer_holder.step`
(160 faces: 52 planes, 42 cylinders, 42 NURBS, 8 spheres, 14 tori, 2 cones;
386 edges) they were the largest remaining cost of a one-face offset edit.

This is a bounded subset of backlog row O06 ("Cache feature-recognition/selection
adjacency…"): it caches per-call state for recognition and the bulk edge
queries. It changes no verdict, tolerance, refusal or output order. Opposing
planar-face bucketing, the other half of the row, is not touched.

## What dominated

`samply` on the native `profiling` build, both calls on the fixture:

| share of the call | `recognize_features` | `solid_edge_relations` |
|---|---|---|
| point classification of the four concavity quadrant probes per edge | 86 % | 99 % |
| — face bounds + BVH rebuilt per ray (`face_aabb`, NURBS `expand_aabb_for_surface`) | 33 % | 40 % |
| — boundary test: NURBS `project_point_to_surface` for every NURBS face per probe | 25 % | 28 % |
| — ray-NURBS crossings: 20 × 20 seed grid re-evaluated per ray, trim polygon re-projected to UV per hit | 29 % | 30 % |
| face areas (`integrate_face_area` trimming, NURBS tessellation) | 12 % | — |

The probes went through the one-shot `classify_point`, which rebuilds every
face bound, the face BVH and candidate trim polygons for every ray of every
probe: ~1 600 full-solid preparations per call.

## Change

All of it is exact: every shortcut computes a value the old code computed, or
skips work whose result provably could not count.

- **Prepared classifier per call** (`query::EdgeRelationContext`). Recognition,
  `solid_edge_relations` and `edge_relation` classify their probes through one
  `remus_check::classify::PreparedSolid` (PERF-Q01), which runs the shared vote
  loop over the same faces, bounds and trims as the one-shot path. Each edge's
  normals are sampled once and shared by the G1 test, the normal angle and the
  probe point (they were projected three times).
- **Per-face caches in `PreparedSolid`** (`boundary::FaceCache`, lazily filled,
  `OnceLock`, so the context stays `Send + Sync`): the line-NURBS seed grid
  (`LineSurfaceSeedGrid`, new in `remus-math`; `intersect_line_nurbs` now builds
  one and delegates), the point-projection seed grid (`SurfaceSeedGrid`, which
  already existed with the same bit-identity contract), and the UV images of
  each NURBS, cylinder, cone and torus face's trim polygons.
- **Boundary-test skip for NURBS faces.** With positive weights a NURBS surface
  lies in its control points' box, and the boundary test's projection measures
  the distance to an evaluated surface point. A point farther than
  `tolerance + 1e-9 × coordinate scale` from that box would have measured
  `>= tolerance` anyway, so the projection is skipped (`hull_excludes`). The
  margin dominates evaluation rounding by five orders of magnitude.
- **Ray-NURBS Newton refinement** (`intersect_line_nurbs`): the first-derivative
  table is written into a reused workspace (`derivative_table_from`, documented
  bit-identical to `derivatives`), and a run that revisits a state it already
  passed through (fixed point or longer cycle — every state is a pure function
  of the `(u, v)` bits) jumps to the state the iteration budget would have
  ended on, instead of running the budget out. On the fixture's edge
  relations, 2 909 of 25 111 Newton runs reach a fixed point and 6 149 a longer
  cycle; skipping their repeats removes about a third of all iterations.
- **Trimmed-quadrature segment lookup** (`face_integrator/loop_index.rs`).
  Every Gauss abscissa of a trimmed face asked every trim loop for its
  crossings and winding number by scanning all of its segments, and the
  abscissa count grows with the loop's vertex count: `O(n²)` per face. A
  bucketed interval index returns a superset of the segments that can
  contribute, each still tested exactly as before; crossings are sorted or
  counted and winding contributions summed, so the order of visits is not
  observable. This speeds every trimmed analytic-face area, volume and mass
  property, not only recognition.

## Equivalence

- `crates/operations/tests/perf_o06_recognition_golden.rs` pins the complete
  outputs — every feature with its faces and values, every edge with its verdict
  and signed angle — on the hammer holder (consumer deflection 0.08) and on five
  synthetic fixtures the suites already use (two-hole plate, pocketed box,
  blind-hole plate, chamfered box, filleted box; default probe and a 0.01 caller
  probe). The goldens in `tests/golden/data/o06/` were generated from
  `4437751c` before any change and pass unmodified on both. Discrete content
  compares exactly, floats to 1e-9 relative (CI's libm may round a
  transcendental differently from the generating machine); on the generating
  machine the candidate regenerates them byte for byte.
- Same-process, bit-exact differential tests: the prepared classifier against
  the one-shot classifier around the hammer holder (the quadrant probes of
  every manifold edge, every vertex, and points inside and outside the 1e-7
  boundary band at each vertex; every third probe against the one-shot path,
  all of them twice through the prepared context); the seeded line-NURBS intersection and the Newton shortcuts
  against a verbatim copy of the old algorithm on 1 200 rays (hits, grazes,
  misses, fixed points and longer cycles all exercised); the cycle jump against
  a step-by-step simulation of every cycle shape; the indexed winding number
  and segment lookup against the full scans, including NaN and infinite
  queries.
- `crates/remus/examples/recognition_perf.rs` dumps the full `Debug` output of
  both calls (`RECOG_PERF_DUMP`); the candidate's dumps are byte-identical to
  the baseline's.

## Measurement

Native, `profiling` profile (release + debug info, no LTO), Apple Silicon
laptop shared with other jobs, `crates/remus/examples/recognition_perf.rs` on
the fixture at the consumer's deflection (0.08). Baseline: `4437751c`
(origin/main). Candidate: `95755546` (the last code commit of this change).
The two binaries were run interleaved, three rounds of five runs each; both
print the same output digests and the `Debug` dumps are byte-identical.

| call | baseline, 15 runs | candidate, 15 runs | speed-up |
|---|---|---|---|
| `recognize_features` | 940–1 008 ms (median 964) | 146–201 ms (median 150) | 6.4× |
| `solid_edge_relations` | 837–871 ms (median 846) | 69–99 ms (median 72) | 11.7× |

Where the candidate's time goes (`samply`, symbolicated against the binary):

| share | `recognize_features` | `solid_edge_relations` |
|---|---|---|
| ray crossings of the quadrant probes | 50 % | 91 % |
| — line-NURBS Newton refinement (`derivatives`, `evaluate`) | 36 % | 61 % |
| — ray-torus quartic (Durand–Kerner) | 9 % | 20 % |
| boundary test, edge normals | 2 % | 6 % |
| face areas: trimmed Gauss quadrature of analytic faces | 31 % | — |
| face areas: NURBS face tessellation | 15 % | — |

WASM, Node 22: scratch `wasm-pack --target nodejs --release --no-opt` builds
of the kernel (`--no-default-features`, SIMD, the shipped flags minus
`wasm-opt`) from `4437751c` and from this branch, both reading the fixture
through the translator package and calling the bindings directly
(`recognizeFeatures(solid, 0.08)`, `solidEdgeRelations(solid)`), seven runs
per call, two interleaved rounds. The JSON payloads are byte-identical
(same SHA-256).

| binding | baseline | candidate | speed-up |
|---|---|---|---|
| `recognizeFeatures` | 3 245–3 426 ms | 571–578 ms | 5.8× |
| `solidEdgeRelations` | 2 925–2 957 ms | 291–294 ms | 10.1× |

WASM has no `fma` instruction, so every `mul_add` runs as a software call; in
the candidate's WASM edge-relation profile ~60 % of samples are in that
fallback, half of it under the ray-torus quartic and a third under the Newton
refinement (see "Left out" below).

Through the consumer: OpenZCAD's untracked offset-face harness
(`test/perf/offset-face-perf.test.ts`, one cold import sync and one −6 mm
offset sync of the 1 045.93 mm² +X face, every kernel call timed) with the
same two scratch packages (kernel and translator built from the same source).
The machine was heavily and unevenly loaded by other jobs during these runs,
so only the recognition rows are meaningful; whole-sync wall times moved with
the load, not with this change. Volume, face count, delta and warnings are
identical. The harness path does not call `solidEdgeRelations`.

| `recognizeFeatures` per sync | baseline | candidate |
|---|---|---|
| cold import sync | 3 499 ms | 705 ms |
| offset sync | 7 089 ms (load spike) | 670 ms |

The native and WASM candidate builds in the tables were made during a
short-lived experiment that spelled the seed scan's `o.mul_add(1.0, d * t)` as
`o + d * t`; LLVM already folds the multiply by one, so both spellings compile
to the same addition, and the final commit keeps the original. Builds of the
final commit's code measured within the same ranges: WASM 574–576 ms and 293–294 ms;
native 154–180 ms and 75–81 ms over two interleaved rounds at higher load.

## Left out

- **Ray-torus quartic.** Durand–Kerner dominates the remaining WASM time. In
  the fixture's edge relations 1 696 of 6 078 solves run the full 100 sweeps,
  and 1 100 of those revisit an earlier state; jumping to the budget-end state
  as the Newton refinement now does would remove ~23 % of all sweeps, exactly. Not
  done here to keep this change to the recognition path's own helpers; it is
  a self-contained follow-up in `analytic_intersection.rs`. Anything beyond
  that (a bounding-volume reject, a different quartic solver) changes which
  near-miss roots are reported, so it is a semantic change.
- **NURBS face areas in recognition** still tessellate at the caller's
  deflection: `FilletLike` reports that area and the fillet threshold
  averages it, so switching to Gauss quadrature would change the output.
- **Reuse across edits.** Each call prepares its own context; a persistent,
  mutation-invalidated cache (O3.2, `ClassificationCache`) could carry face
  preparation across the consumer's per-edit calls for unchanged faces.
- **Opposing planar-face bucketing**, the other half of O06.

## Reproduce

```sh
cargo build --profile profiling -p remus --example recognition_perf
RECOG_PERF_RUNS=5 RECOG_PERF_DUMP=/tmp/o06 \
  target/profiling/examples/recognition_perf \
  crates/io/tests/data/shapr3d_hammer_holder.step 0.08
# Profile one call:
RECOG_PERF_ONLY=relations samply record --save-only -o relations.json.gz \
  target/profiling/examples/recognition_perf
```

The example prints a digest of each call's full output; it asserts that
repeated runs agree.
