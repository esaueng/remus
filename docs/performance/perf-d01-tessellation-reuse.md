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
  default 4096 faces / 32 MiB estimated. Current retention gives display
  entries priority over quantity entries, with deterministic FIFO within each
  class (§9); an entry larger than the budget is never retained; captures are
  committed only after the whole tessellation succeeded.
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

## 8. Follow-up (2026-10-04): translated NURBS reuse and the whole-mesh passes

Branch `perf/tessellation-remainder` (on top of #958). Backlog rows D01
(remainder of §7) and D07.

### Why the translates were refused

Instrumenting the eight refusals showed every carrier, vertex and control
point of the moved region translated by exactly (6, 0, 0); the differences
were all in **edge samples**. A circle sample was
`center + u·r·cos t + v·r·sin t`, two roundings at world magnitude, and a
NURBS edge sample a de Boor sum of world control points, so a sample of the
moved edge was its source's plus 6 ± one ulp. Every chart built from those
samples (analytic `project_point`, NURBS Newton projection) then differed in
the last bits, and the CDT resolved cocircular quads accordingly.

### What changed

- **Translation-exact edge sampling** (`edge_sampling.rs`). Circles and
  ellipses form the offset in their own frame and add the centre once
  (`sample_circle_uniform`, also used by the A2 circle sync and the A3 torus
  rim densification); NURBS edges evaluate, and project a closed edge's
  start vertex, relative to their first control point
  (`AnchoredNurbsCurve`). A rigid translation now moves every sample by
  exactly the translation whenever the moved coordinate stays in its binade.
- **NURBS frame chart** (`NurbsFrame` in `nonplanar.rs`). The CDT mesher
  charts a NURBS carrier on `control point − anchor`: boundary Newton
  projection, pcurve lookup, interior grid sizing and interior normals.
  Interior positions are still evaluated on the world surface. The
  projection seed grid (81 surface evaluations) is built once per face
  instead of once per boundary sample; that returns the same projection bit
  for bit and is most of the cold-path gain below.
- **Cache rule** (`TranslationRule::NurbsFrame`). A NURBS face that can only
  reach the frame-charted CDT (non-periodic carrier, so neither the pole-cap
  nor the blend-band mesher; not a recognized cylinder wall with an outer
  circle, so not the wall-band mesher or the wall-chart projection) is
  replayed by translation when every recorded position is an *exact*
  translate (Knuth two-sum error zero, so every difference of positions is
  bit-identical, and unit vectors bit-identical) and its capture was meshed
  by that path (a CDT failure falls back to the world-coordinate snap mesher
  and is never replayed by translation).
- **Whole-mesh passes** (`pool_index.rs`, `mesh_ops.rs`): the circle contact
  refinement scans only pool vertices in grid cells along each arc (the
  acceptance test is unchanged and candidates keep ascending id order);
  dedupe numbers distinct quantized positions once and groups triangles by
  sorted id triples with a counting sort; weld and gap fill find boundary
  half-edges through a vertex-indexed half-edge table; gap fill is skipped
  when the weld found no boundary half-edge and dedupe removed nothing.

### Output

- The pass rewrites are byte-identical: the previous implementations are
  kept verbatim as test oracles (`tessellate::tests::mesh_passes`) and a
  test-only switch meshes drilled, fused, coplanar-fused, filleted, hollow,
  ridge-filleted (circle insertions) and primitive bodies both ways at three
  deflections; damaged meshes (duplicated, reversed, degenerate and removed
  triangles, nudged vertices) are passed through each pass both ways. The
  Hammer Holder digests (source and edit, deflections 0.0148 and 0.1) were
  recorded from #958 and held through the rewrite.
- The sampling and frame-chart change moves untranslated output by roundoff
  only. On the Hammer Holder: identical vertex, triangle and face counts and
  identical triangles (vertex ids and winding) on every face; 34 NURBS faces
  emit their triangles in a different order; about 7 150 positions move by at
  most 2.2e-14 and NURBS normals by at most 1.9e-12. The digests were
  re-pinned for that commit. The golden regressions are unchanged.

### Hammer Holder edit (native and WASM run the same code)

| Stage-D faces of the moved body | #958 | now |
|---|---|---|
| exact hits | 127 | 127 |
| translated hits | 7 | 11 (+ cylinders 204/238, NURBS 232/233) |
| misses: re-limited carriers | 12 | 12 |
| misses: refused translates | 8 | 4 (sphere 191, torus 200, cylinder 201, torus 222) |
| conflicts / uncacheable | 0 / 0 | 0 / 0 |

The four remaining refusals are real roundoff differences in the input, not
mesher artefacts: `move_faces` rebuilds the rim circles of the blends at
191/200/201 with unit vectors one ulp apart (the key accepts them within 64
ulps, the chart does not), and torus 222 has rim samples that cross a binade
(x −17.5 → −11.5), where the moved coordinate gains a bit its source never
had. The reused mesh equals the fresh one: indices and face offsets
identical, normals bit-identical, positions within 1.5e-14, watertight.

### Native (Apple M5 Pro, profiling build, shared machine at load ≈ 7–17, medians of 7, two interleaved runs)

| Call | #958 | now |
|---|---|---|
| source, cache off | 294–301 ms | 193 ms (1.5×) |
| source, cache warm (all exact) | 89.5 ms | 22.3 ms (4.0×) |
| moved body, warm from the source | 112.5–115.0 ms | 28.7 ms (3.9×) |
| `move_faces`, cache off | 1826–1832 ms | 1510–1518 ms |
| `move_faces`, cache warm | 1119–1130 ms | 862–866 ms |

Warm stages before → after: circle contact refinement 38.7 → 2.0 ms, dedupe
22.3 → 1.8 ms, weld 8.1 → 3.1 ms, gap fill 7.9 → 0 ms (skipped).

### WASM (`wasm-pack --release --no-opt`, simd128, Node 22.23.1, two runs each)

Fixed display deflection 0.0148 (the behaviour of OpenZCAD #598), direct
calls on the packages in one kernel (scratch driver):

| Call | #958 | now |
|---|---|---|
| source display mesh, first call | 1001–1057 ms | 668–696 ms |
| source display mesh, warm | 213–238 ms | 41–45 ms |
| moved body display mesh, warm from the source | 287–318 ms | 58–65 ms (≈ 5×) |
| `moveFaces −6` | 3350–3478 ms | 2793–3040 ms |

OpenZCAD harness (`test/perf/offset-face-perf.test.ts`, the main OpenZCAD
checkout as is, which does not carry #598 and so re-meshes the edit at the
new extent's deflection, a cold mesh): `tessellateSolidGroupedBinary` at
import 1012–1099 → 659–668 ms, in the offset sync 916 → 643–697 ms;
`moveFacesJournaled` 2632–2673 → 2147–2381 ms. The offset sync's wall time
(10.9–12.0 s) is dominated by `recognizeFeatures` (3.4 s), `validateSolid`
(2.5 s over 12 calls) and `faceArea` (1.1 s over 510 calls).

### What remains (warm source, native)

Stage D key building and replay about 12 ms (key construction walks every
chain and hashes positions), the holed planar CDT jobs (stage B/C, not
cached) about 9.5 ms, the boundary-normal pass about 4 ms, weld 3 ms, circle
refinement 2 ms, dedupe 1.8 ms.

### Qualification

- `tessellate::tests::face_cache` (9 tests): adds a smooth loft (curved
  NURBS with pcurves) and a converted box (bilinear NURBS) translated by
  (8, 0, 0): every NURBS face is a translated hit and the mesh reproduces
  fresh (indices, offsets, normals identical, positions within 4 ulps);
  disabling the frame chart fails it (face offsets differ). A translate
  across a binade is refused and re-meshes to the fresh mesh; a rounded
  0.1 step within one binade is an exact translate by the rounded step and
  is reused.
- `tessellate::tests::mesh_passes` (3 tests): the reference comparisons
  above, plus the weld and dedupe reports that gate gap fill.
- `crates/io/tests/tessellation_reuse_hammer.rs`: now ≥ 11 translated,
  ≤ 4 refused; `crates/io/tests/tessellation_digest_hammer.rs`: the
  digests, asserted on aarch64 macOS (C `libm` trigonometry differs in the
  last bit between platforms), watertight and deterministic everywhere.
- Full `remus-operations`, `remus-io` and `remus-wasm` suites pass.

### Not done

- Analytic faces still need bit-identical `project_point` charts; the four
  refusals above are input differences the tessellator cannot remove.
  Planar faces translated across their projection plane keep re-meshing
  (their chart is the kept world coordinates).
- Holed planar faces (stage B/C) and the boundary-normal pass are still
  uncached; the key construction is now the largest warm cost.
- No JS binding for the statistics; OpenZCAD #598 is required for the
  display call to hit the cache after an edit.

## 9. Follow-up (2026-10-10): display priority within the existing bounds

Fine measurement meshes and display meshes still share the same 4096-entry,
32 MiB default cache. They retain the same geometry, boundary-plan, tolerance,
translation and replay keys; retention class is bookkeeping, not a new meshing
input. Quantity admission evicts only quantity entries and declines storage
when protected display entries leave insufficient space. Display admission
and explicit bound reductions evict quantity entries first, then display
entries. Each class keeps deterministic FIFO order.

A quantity entry reused for display is promoted without a second copy. Promotion
occurs on successful face replay; a later whole-mesh failure can leave that
retention metadata changed. New captures still commit only after the complete
tessellation succeeds. Total count and byte bounds apply in either case.

This prevents varying fine-volume requests from flushing the display working
set. It does not let one tolerance answer another: an edited bounding-box
diagonal can still change volume's effective tolerance and force fresh
measurement meshing.

The measurements and qualification counts above describe their dated revisions.
Current production changes and their qualification targets are recorded in
[Face-edit responsiveness](perf-face-edit-responsiveness.md).
