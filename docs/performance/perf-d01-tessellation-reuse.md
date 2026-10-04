# PERF-D01 slice: content-keyed per-face mesh reuse across direct edits

Status: production slice implemented (stage-D faces of the solid/sheet
pipeline), opt-in per thread, enabled by `BrepKernel::new`.
Branch: `perf/tessellation-reuse`.
Backlog row: D01 (`optimization-backlog.csv`), building on D03's boundary plan
(`perf-d03-boundary-plan.md`).
Scope: `crates/operations/src/tessellate/{face_cache.rs,solid.rs,mod.rs}`, one
line in `crates/wasm/src/kernel.rs`, tests, and the
`crates/remus/examples/tessellation_reuse_perf.rs` diagnostic.

## 1. Workload and profile (before)

OpenZCAD rebuilds the display mesh after every direct edit with
`tessellateSolidGroupedBinary(solid, 2e-4 × extent, 0.06)`. On
`crates/io/tests/data/shapr3d_hammer_holder.step` (160 faces, 125 046
triangles at 0.0148) a planar face move (`push_pull::move_faces`, the +X face
of area 1045.93 by −6) returns a new solid with new handles, so nothing keyed
on handles or topology identity can be reused, yet most faces are unchanged
and the moved region is a rigid translation.

Single-thread native profile of `tessellate_faces_core` (samply, profiling
build): per-face meshing (stage D, `tessellate_face_with_shared_edges`) 58%,
of which NURBS CDT, boundary Newton projection and grid evaluation dominate;
boundary-vertex normals (`face_surface_normal_at`, a NURBS projection per
vertex and face) 13%; `dedupe_coincident_triangles` 9%; weld 4%; gap fill 4%;
stage A6 circle contact refinement about 5%. Edge sampling and the holed
planar CDT jobs run on rayon workers and are small.

## 2. What a face's mesh depends on

`tessellate_face_with_shared_edges` reads:

| Input | Recorded in the key as |
|---|---|
| surface carrier, `is_reversed` | variant tag; analytic origins as positions, axes as directions, radii/angles exact; NURBS degrees, knots and weights exact, control points as positions; plane `d` as `d − n·anchor` |
| wires, edges, vertices, trims | wire/edge counts, coedge directions, face-local edge and vertex ordinals (first appearance), curve data as above, trims exact |
| pcurves (`pcurve_oriented(edge, face, fwd)`) | full 2D curve and range, exact |
| final boundary-plan chains (after A2 circle sync, A3 torus densify, A4 seam splits, A6 contact refinement, C2 Steiner reconcile) | per-edge chain as face-local gid ordinals plus each distinct sample's position |
| deflection, angular tolerance, `circle_floor`, latitude-cap flag | exact |
| merge-grid lookups on the shared pool | not keyed; checked at capture and replay (§3) |

Global vertex ids never enter the key: the equality pattern the meshers rely
on is kept as ordinals, so the new body's numbering is free to differ.

## 3. Design

- **Capture (miss).** The face is meshed as before; its appended pool slice is
  then inspected. It is stored only if every triangle corner is one of its own
  boundary-chain vertices or a vertex it created, no boundary normal was
  rewritten, and the merge map grew by exactly the vertices it interned.
  Anything else (a corner welded to another face's vertex, a pole cap writing
  rim normals) counts as `uncacheable`.
- **Replay (hit).** The cached vertices are interned through the same merge
  grid in the same order and the triangles are emitted with boundary ordinals
  mapped to the new chain ids. A merge-grid collision the capture never saw
  rolls the replay back and meshes the face fresh (`replay_conflicts`).
- **Boundary normals.** The normal pass sums per-face surface-normal
  contributions in its unchanged order; exact hits supply cached
  contributions, everything else evaluates them.
- **Bounds.** Per thread, opt-in (`enable_face_mesh_cache[_with_limits]`),
  default 4096 faces / 32 MiB estimated, deterministic FIFO; an entry larger
  than the budget is never retained; captures are committed only after the
  whole tessellation succeeded.
- **Invalidation.** The key is the content, so `transform_solid`, `*_mut`
  edits, healing, rollback, checkpoint restore and deletion cannot serve a
  stale mesh: changed content misses.

## 4. Translation and exactness

A key match up to translation (positions within 1024 ulps of the face scale,
directions within 64 ulps, everything else exact) is **not** enough for the
replay to equal a fresh mesh of the moved face. Measured on the Hammer Holder
with unrestricted translated reuse: a moved quarter-cylinder blend re-meshed
fresh with flipped grid diagonals, a moved torus emitted the same triangles
in a rotated order, translated twin features (cone, cylinder, torus) meshed
differently, and a NURBS boundary normal moved by 7.9e-3. The causes are
mesher properties, not cache bugs:

- the CDT lays its interior grid out from the boundary's chart bounds; the
  grid's cocircular quads are decided by roundoff, so a one-ulp change in a
  boundary chart coordinate (which a translation introduces whenever
  `(p + δ) − (o + δ) ≠ p − o` in floating point) flips diagonals;
- band meshers sort rims by `atan2(..).rem_euclid(TAU)`;
- NURBS boundary UVs and normal contributions come from Newton projection
  (the normal pass uses a 1e-4 tolerance).

So a translated hit is accepted only when the mesher's chart input is
bit-identical (`TranslationRule`): every boundary sample's `project_point`
coordinates for analytic faces without pcurves, the `project_by_normal`
coordinates for planes. NURBS faces are reused only exactly; refused
translates are counted (`translation_refused`). Translated hits re-evaluate
their boundary-normal contributions.

## 5. Results

### Hammer Holder edit (native and WASM run the same code)

| Stage-D faces (154 of 160; 6 holed planes are stage B) | Count |
|---|---|
| exact hits (untouched faces) | 127 |
| translated hits (4 cylinders, a sphere, a torus, a plane) | 7 |
| misses: re-limited carriers (6 planes, 6 cylinders) | 12 |
| misses: refused translates (cylinders 201/204/238, sphere 191, tori 200/222, NURBS 232/233) | 8 |
| misses because a neighbour's boundary plan changed | 0 |
| uncacheable / replay conflicts | 0 / 0 |

The reused mesh equals the fresh one: identical indices and face offsets,
positions within 1.8e-15, normals bit-identical, watertight.

### Native (Apple M5 Pro, profiling build, shared machine at load ≈ 7–10, medians of 7)

| Call | Cache off | Cache on |
|---|---|---|
| source, cold (capture) | 318.7 ms | 321.3 ms (≈ +1%, noise) |
| source, warm (all exact) | 318.7 ms | 92.7 ms (3.4×) |
| moved body, warm from the source | 298.1 ms | 114.0 ms (2.6×) |
| `move_faces` (its volume checks re-mesh) | ≈ 1.93 s | 1.53 s first edit |

Retained: 5.5 MiB for the source display mesh, 6.4 MiB after the edit.

### WASM (`wasm-pack --release --no-opt`, simd128, Node 22, two runs each)

Direct calls on the packages (scratch script, same deflection 0.0148):

| Call | main | branch |
|---|---|---|
| source display mesh, first call | 928–931 ms | 939–955 ms |
| source display mesh, warm | ≈ 890 ms | ≈ 200 ms |
| moved body display mesh, first call | 888–895 ms | 271–283 ms (3.2×) |
| `moveFaces −6` | 4293–4311 ms | 3164–3222 ms |

OpenZCAD harness (`test/perf/offset-face-perf.test.ts`, offset −6 sync):
`tessellateSolidGroupedBinary` 909.7 ms (main) vs 935.4 / 917.9 ms (branch),
`moveFacesJournaled` 4390.8 ms vs 2627.5 / 2690.3 ms. The display call gains
nothing there because the consumer derives its deflection from the current
bounding box: the edit shrinks the extent, so the offset sync meshes at
0.0136 instead of 0.0148 and every face misses (a different deflection is a
different fresh mesh). Holding the display deflection stable across an edit
session (or quantizing it to a ladder) lets the display mesh reuse as in the
direct-call table.

### What remains (warm profile)

`dedupe_coincident_triangles` 24%, A6 circle contact refinement scanning
every pool vertex per circle edge about 22%, weld 9%, gap fill 8%; cache
lookup and replay about 4%.

## 6. Qualification

- `tessellate::tests::face_cache` (operations, 7 tests): cold and warm equal
  fresh bit for bit; filleted-box move pins exact 1 / translated 3 / miss 4
  and reproduces fresh; unrelated edits, deflection, angular tolerance and
  the boolean circle-floor policy miss; in-place mutation (rotation and
  translation via `transform_solid`, `vertex_mut`/`face_mut`, failed
  transaction rollback, checkpoint restore, heal, delete and rebuild) always
  equals fresh; drilled block, hollow box (cavity shell), sphere, torus, cone
  and an open sheet reproduce fresh and stay watertight; FIFO count/byte
  bounds, oversized entries and zero capacity; per-thread opt-in.
- `crates/io/tests/tessellation_reuse_hammer.rs`: the Hammer Holder edit
  above (indices, offsets, normals identical; positions within 64 ulps;
  watertight; 154 lookups, ≥ 120 exact, ≥ 1 translated, no conflicts).
- Mutation checks: dropping the chart-identity rule fails the Hammer test;
  dropping the translation shift fails the filleted-box test.
- Full `remus-operations` and `remus-wasm` suites (the wasm crate's tests now
  mesh through the cache) and the golden regressions pass unmodified.

## 7. Not done

- Holed planar faces (stage B CDT jobs, cheap here) and the whole-mesh passes
  (§5) are not cached; they bound the warm floor.
- Translated NURBS reuse would need the boundary UVs verified (or the
  meshers' grids derived from translation-invariant data); translated
  analytic faces whose chart coordinates are not bit-identical re-mesh.
- No JS binding for the statistics; the consumer's extent-relative display
  deflection is outside the kernel.
