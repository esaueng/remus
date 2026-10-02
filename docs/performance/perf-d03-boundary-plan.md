# PERF-D03 slice: explicit shared-boundary plan (boundary planning, local triangulation, reconciliation, deterministic assembly)

Status: bounded production slice implemented (holed-planar CDT family + neighbours).
Branch: `codex/perf-d03-shared-boundary-plan`.
Scope: `crates/operations/src/tessellate/` + dedicated tessellation tests/benchmarks only.
No change to shell, fillet, extrusion, measurement, GFA, topology, or sketch construction.

## 1. Dependency inventory (milestone 1)

Every producer and consumer of shared edge samples in
`crates/operations/src/tessellate/solid.rs::tessellate_faces_core`:

| # | Stage | Producer / consumer | Policy notes |
|---|-------|---------------------|--------------|
| A1 | Edge sampling | `edge_sampling::sample_edge[_with_params]` per unique edge (parallel ≥32 edges native, serial otherwise and on WASM). `circle_floor` selects display (`false`, exact chord count) vs boolean (`true`, curvature floor for co-refinement). | Display-vs-boolean densities must never mix; the flag travels in `BoundaryPlan.policy`. |
| A2 | Circle-density sync | Gridded cylinder/cone faces upsample circle edges to their analytic grid columns (`face_nu + 1`). Skips walls with inner wires (they walk their own wires). Monotone max-wins, order-independent. | Resampled chains keep uniform authoritative parameters. |
| A3 | Torus-rim densify | Two-rim torus bands densify once-used rim circles to the band mesher's wrap density (B46). Proportional-by-arc for split rims. | Same parameter retention as A2. |
| A4 | Seam-meridian splits | Holed cylinder walls: inner-wire polylines pre-split at seam crossings (60-iteration bisection on NURBS section curves). Pool kept immutable during discovery (quadratic-work guard); applied once per edge, sorted + deduped. | Insertions are geometric (chart interpolation) → `None` parameters. |
| A5 | Pool freeze | `BoundaryPlan::from_samples`: sorted edge order, merge-grid pool, per-edge authorities (domain, closed flag, curve tag), per-sample parameters. | First-writer-wins is a pure function of input. |
| A6 | Planar contact refinement | Line edges gain tangent contact subdivisions shared with every user; circle chains absorb on-circle pool positions via retained authorities. Budgeted by `MAX_PLANAR_CONTACT_CANDIDATE_PAIRS` (fail-closed). | Reads authorities, never re-derives trims from endpoints. |
| B | Local triangulation | Holed-planar faces snapshot chains into `CdtJob`s; `run_planar_cdt` runs parallel (≥2 jobs native) or serial. Job collection is read-only on the plan. | Same inputs → same outputs on any thread count. |
| C1 | Stage requests | Per-job Steiner-on-boundary detection emits `BoundaryRefinementRequest`s (normalized `(lo,hi)`, `t` from `lo`). Lifting merges through the shared grid in face order. | Data, not mutation. |
| C2 | Reconcile | `BoundaryPlan::reconcile`: merge per segment (sort by `(t,gid)`, identity dedup), splice each chain once in sorted key order. Bounded by `MAX_BOUNDARY_REFINEMENT_POINTS` (fail-closed, chains untouched on excess). | Fixes the old per-job sequential splice, which lost a second job's splits on an already-spliced segment. |
| C3/D | Assembly | Staged CDT triangles emitted in face-set order, repaired by proven `split_triangles_spanning_boundary_splits` (bounded fan budget, no fixed-point loop); all other faces triangulate against final chains with unchanged structured/CDT/snap dispatch. | Curved fallback behaviour preserved. |
| Finish | Normals/weld/dedupe/gapfill | Unchanged: analytic-then-fallback normals, boundary weld (closed solids), coincident-triangle dedupe (#696), sub-deflection gap fill. | No broader welding, no deleted-triangle policy change. |

Other shared-sample consumers (unchanged, outside the plan): `sample_solid_edges[_filtered]`
(wireframe, own `sample_edge` calls, floor-free), per-face `tessellate()` (independent
mesher for classification meshes), `mesh_boolean` co-refinement (via
`tessellate_solid_for_boolean`, same plan with `circle_floor = true`).

## 2. Boundary-plan contract (milestone 3)

`crates/operations/src/tessellate/boundary_plan.rs`:

- `BoundaryPlan` owns `edge_points`, `edge_params` (parallel, `None` for
  geometric insertions), `authorities` (domain / closed / curve tag),
  `merged` pool, `point_to_global`, `edge_chains`, sorted `edge_order`,
  `policy`, tolerances, and `PlanMetrics` counts.
- `sample_edge_with_params` (`edge_sampling.rs`) returns points identical to
  `sample_edge` plus per-sample authoritative parameters (pinned to domain
  ends at overwritten endpoints; wrapped full-period walk for closed NURBS).
  Pinned by `edge_sampling_with_params_preserves_points`.
- `ordered_chain` + `collect_wire_vertices` apply coedge direction in one
  place (mirrors `planar::collect_wire_global_vertices` exactly).
- `validate_structure` cross-checks every retained field against the
  topology (debug-only, typed errors, no panics).

## 3. Reconciliation (milestone 5)

- Requests are grouped per segment, sorted, identity-deduped, then spliced
  once per chain in sorted key order: every incident face observes the same
  final subdivision regardless of job completion or input order.
- Already-triangulated staged jobs are repaired with the proven
  `split_triangles_spanning_boundary_splits` route; neighbours tessellated
  afterwards consume the final chains directly.
- Bounded on two levels: reconcile budget (1M points, fail-closed with
  contributing faces named) and repair fan budget (chain-length based,
  pre-existing). No uncontrolled fixed-point loop.
- Unit-pinned: order-independence, identity dedup, flipped-parameter
  normalization, budget refusal leaving chains untouched, coedge-direction
  traversal.

## 4. Integration and production domain (milestone 6)

Bounded domain: the existing holed-planar CDT family and its neighbouring
faces flow through plan → stage → reconcile → assemble. All curved families
(structured bands, latitude/notch/pole/cap meshers, nonplanar CDT, snap
fallback) keep their established dispatch and fallback order; they read the
final chains from the plan instead of ad-hoc locals. Behaviour is otherwise
unchanged: the full pre-existing tessellation suite (158 lib tests incl.
B54 repair tests, dovetail/#696, drilled-hole radii × deflections,
gridfinity tile, seam/split-rim bands) passes unmodified.

## 5. Qualification (milestones 2 + 7)

New tests:

- `tessellate::solid::boundary_plan_pipeline_tests` (unit, in-module):
  face-order permutations (normal/reversed/rotated/index-desc) give
  identical triangle multisets with `(bd,nm) = (0,0)`; repeated runs are
  byte-identical (indices + positions); sampler params preserve points with
  circle endpoints pinned to authoritative domains.
- `crates/operations/tests/perf_d03_boundary_plan.rs` (integration):
  repeated public tessellations byte-identical; watertight at 0.05/0.1/0.5;
  grouped offsets aligned (`faces+1`, sentinel, same triangle count);
  display-vs-boolean policy distinct (boolean never coarser on curves, both
  watertight); drilled-box mesh and B-Rep volumes within 1% of analytic.
- `crates/operations/benches/perf_d03_boundary.rs`: complete-workflow time
  + allocation/quality stats lines (no local-triangulation-only numbers).
  Workspace lints deny prints even in benches, so each bench writes its
  stats line to `$TMPDIR/perf-d03-<label>.txt` (overwritten per run) and
  fails setup loudly on non-watertight output; thread scaling is qualified
  by diffing those files across `RAYON_NUM_THREADS`.

Measured (release, criterion, 20 samples; `build=` is one-shot setup):

| Fixture | verts / tris | bytes pos/nrm/idx | bd/nm | time (default pool) | time (`RAYON_NUM_THREADS=1`) |
|---|---|---|---|---|---|
| Drilled box d=0.1 | 44 / 88 | 1056/1056/1056 | 0/0 | 170 µs | 174 µs |
| Box∩sphere d=0.1 | 122 / 240 | 2928/2928/2880 | 0/0 | 135 µs | 122 µs |
| Hollow box d=0.1 | 16 / 24 | 384/384/288 | 0/0 | 58 µs | 54 µs |
| 64-hole plate d=0.1 | 2312 / 4876 | 55488/55488/58512 | 0/0 | 30.8 ms | 34.5 ms |

Stats lines are byte-identical across thread counts (determinism); small
inputs show no scaling (serial paths dominate by design: <32 edges, <2 CDT
jobs); the 64-hole plate shows a modest parallel gain. WASM executes the
same code serially (`cfg(target_arch = "wasm32")` sampling + CDT branches,
unchanged); packaged WASM tessellation/gridfinity contracts run in the full
validation below.

## 6. What this enables (milestone 8) — not implemented here

- **Mesh caching (PERF-D01):** the plan is the dependency node the cache
  needs — per-edge chains with identities, tolerances, and purpose
  (`circle_floor`) in the key; `PlanMetrics` is the invalidation accounting
  surface. One edge edit invalidates incident faces via `edge_chains`.
- **Broader parallelism (PERF-D04):** stages B/C/D are now separable with
  explicit inputs/outputs (jobs in, requests out, splits in, mesh out);
  grain-size and skewed-face scheduling can build on staged jobs without
  rediscovering the dependencies.

## 7. Remaining dependencies and non-goals

- Depends on M04 (work counters — plan metrics follow that pattern) and
  M07 (profiled workloads — the four bench fixtures are a start, not the
  full matrix).
- Curved-family reconciliation (shared subdivisions beyond holed-planar
  Steiner points, e.g. curved-neighbour densification against NURBS walls)
  stays on established paths; migrating a family requires its own
  qualification slice per the tessellation skill's structured-mesher
  contract (exact shape check, `Ok(false)` deferral).
- No mesh cache, no new parallelism, no welding/dedupe policy change, no
  silent quality change: triangle multisets on the qualification corpus are
  unchanged except where the old sequential splice provably dropped a
  second job's splits (now merged — strictly more sharing, same vertices).
