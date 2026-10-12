# Local boolean for sequential cuts

**Status (2026-10-12):** stage 1 (through holes) and the first stage 2
widening (blind holes) are implemented in
`crates/operations/src/boolean/local_cut.rs`. Stage 3 makes the GFA's work
on a touched face follow the loops a cut meets. Its two parts are
implemented in `remus-algo` (the last two sections): phase FF, then the
face splitter's untouched holes. The rest is design only.

## Problem

Modeling is mostly one feature at a time: drill a hole, cut a pocket, drill
the next hole. The general GFA processes every face of the blank on every
boolean, so each cut costs more than the last. Drilling an `n × n` grid of
holes into a plate (`examples/sequential_cut_profile.rs`, profiling profile,
Apple M5 Pro) on main at `f30d6853`:

| cut | blank faces | time |
| ---: | ---: | ---: |
| 1 | 7 | 2.4 ms |
| 128 | 134 | 10.4 ms |
| 256 | 262 | 19.7 ms |

That is about 75 µs per existing face per cut, so the whole grid is
quadratic. A profile of the 256th cut, as a share of the boolean:

| phase | share | scales with |
| --- | ---: | --- |
| pave filler (FF 13%, EF 7%, EE overlap 3%) | 30% | blank faces/edges × tool |
| shell assembly (`build_solid_with_origins` 22%, revolved-wire orientation 5%) | 27% | every face |
| face images (`split_face_2d` 11%, of which pcurves 8%) | 21% | every loop on the touched faces |
| shape-store deep copy + export | 10% | every entity |

Two costs are separate:

- **Untouched faces** (≈ 60%): the 255 earlier hole walls and the side walls
  never meet the new tool, yet every assembly pass, the deep copy, junction
  seeding and the revolved-wire orientation walk them.
- **Existing loops on touched faces** (≈ 11%+): the top and bottom faces do
  meet the tool, and each already carries 255 holes that the face splitter
  re-processes (pcurves, outlines, extents) on every cut.

## Stage 1: qualified local through-hole (implemented)

A cut whose tool is a right circular cylinder drilled straight through two
parallel planar faces, clear of everything else, has a closed-form result.
`local_cut::try_local_hole` proves the cell and builds that result
by local surgery; anything outside the cell returns `None` and the general
GFA runs unchanged.

**Cell.**

- The tool is an untrimmed right circular cylinder: one forward cylinder
  wall, two forward planar caps perpendicular to its axis, no inner wires or
  shells.
- The blank has no inner shells. Exactly two of its faces have conservative
  boxes (`conservative_face_set_bounding_box`) that come within the
  clearance of the tool's box; every other face is farther away.
- Both candidates are forward planes perpendicular to the axis
  (`|n·d| ≥ 1 − 1e-12`), strictly inside the tool's axial span by the
  clearance, and bounded only by lines and circles lying in the plane.
- On each, the section disc (tool radius about the axis point) lies inside
  the outer loop and outside every inner loop, at least `radius + clearance`
  from every boundary edge. Distances are exact point–segment and
  point–arc distances. Containment uses a winding number over each loop
  sampled to half that loop's measured distance, so the polygon cannot
  cross the disc centre.
- The lower face's outward normal is `−d` and the upper's `+d`, so the
  material lies between them.
- Clearance is `1000 × tol.linear` plus the largest vertex or edge tolerance
  of either operand. Contacts closer than this belong to the GFA.

**Why it is exact.** No face other than the two pierced faces meets the tool.
Those two meet it only in their section discs. So the open cylinder segment
between the discs does not cross `∂A`, and the orientation of the lower face
puts it inside `A`. The same reasoning puts the tool's ends outside `A`.
Hence `A ∩ B` is exactly that segment, and `A − B` is `A` with the two discs
removed and the segment's lateral surface added. The module docs give the
argument in full.

**Construction.** The cell reproduces the GFA's result representation, not
just its geometry:

- Two closed circle rims are built. Each rim's circle normal is its plane
  face's outward normal, and its reference direction is the radial
  direction of the tool's own seam. That is where the GFA starts the rims
  it cuts, so a later boolean meeting the hole sees the same seam vertices
  either way. A plug fused back into the bore depends on this.
- A seam line joins the rims, and a reversed copy of the tool's cylinder
  surface becomes the wall. The wall's loop is the natural (u, v) rectangle,
  in the GFA's orientation pattern. New edges carry the operation
  tolerance, as the GFA's section edges do.
- `copy::copy_solid_spliced` copies the blank once, with the new entities
  spliced into the copy plan:
  - faces in the GFA shell assembly's order, a breadth-first walk over shared
    edges in wire order;
  - the new hole wire first among each pierced face's inner wires;
  - vertices, edges, wires and faces allocated in the GFA export's
    traversal order.

The result passes `validate_boolean_result_with_tolerance` inside a nested
transaction, or the attempt is rolled back and the GFA runs.

**Evidence.** Criterion, `RAYON_NUM_THREADS=1`, Apple M5 Pro, main at
`f30d6853` against this stage:

| bench | main | stage 1 |
| --- | ---: | ---: |
| `boolean 64 cuts (8x8 grid)` | 121.5 ms | 4.39 ms |
| `sequential_cylinder_cuts/N=64` | 124.1 ms | 9.54 ms |
| `multi-boolean model` (16 holes) | 1.50 ms | 178 µs |
| `boolean/cut_cylinder_through_box` | 250 µs | 10.8 µs |

Instruction counts (`/usr/bin/time -l`, base and candidate binaries
alternated) for `examples/sequential_cut_profile.rs`:

- 144 through-holes: 8.89 G → 0.707 G instructions (−92%), 1.85 G → 0.136 G
  cycles.
- Blind holes, which stage 1 declined: 631–641 M on both binaries.
  Declining costs nothing measurable.

The remaining per-cut slope is the contractual deep copy of the blank and
one cheap outer box per face. Lines and circles bound planar, cylindrical
and conical faces exactly; every other face takes the general conservative
box.

`boolean/local_cut_tests.rs` cuts every qualified case twice, once locally
and once with the cell disabled, and compares a full dump of both results:

- faces in order, with surfaces and orientation;
- coedges, with windings and pcurves;
- wires and edge uses, with each edge's id, curve, trim and tolerance;
- vertices, with ids, points and tolerances.

For axis-aligned cases the dumps are equal, so the result is bit-identical
to the GFA's with entity numbering included. These cases are vertical,
downward-pointing, horizontal through side walls, a 4 × 4 grid compared
after every hole, and a hole beside a rectangular opening. The integrator
and recognition goldens (`perf_face_trim_setup_golden`,
`perf_o06_recognition_golden`) and the plug-fill regressions pass unchanged
on those fixtures.

Off the coordinate axes the GFA reaches the rim frames and seam vertices
through different arithmetic. There the results agree entity for entity,
but those coordinates differ by a few ulps (the rotated-frame case bounds
the drift at 1e-9).

The tests also check that the cell declines:
- coplanar caps;
- tilted tools;
- side-wall breakouts;
- oversized tools;
- stacked plates;
- overlapping holes;
- non-cylinder tools.

**Scope notes.**

- Evolution-tracking entry points (`boolean_with_evolution`,
  `boolean_with_entity_evolution`, `boolean_regions`) call the GFA
  directly, so they keep their existing provenance and are not affected.
- The plain `boolean` path, which the WASM `cut` binding uses, takes the cell.
- Blank faces must be manifold. A boundary use shared by more than two faces
  declines the cell rather than being ordered.

## Stage 2, first widening: blind holes (implemented)

A blind hole has one pierced face instead of two. When exactly one blank
face comes within the clearance of the tool's box, and it qualifies as a
pierced face under the stage 1 rules, the cell is a blind hole:

- The slab of the tool on the side opposite the entry face's outward
  normal lies inside `A`. That slab ends at one of the tool's own caps,
  which the box test puts at least the clearance away from every other
  face, so the cap is strictly inside `A`.
- The slab on the other side, the protruding end, lies wholly outside `A`.

So `A − B` is `A` with the entry disc removed, a reversed wall from the
entry plane to the buried cap, and that cap, reversed, as the floor. The
module docs carry the argument for both cells.

**Construction.** It mirrors the GFA's representation:

- The floor is a reversed copy of the tool's buried cap. It keeps the
  cap's own rim edge (curve, trim and tolerance unchanged) and that rim's
  vertex, used as the cap's wire uses it.
- The tool's seam is split at the entry rim. The new seam keeps the tool
  seam's direction, from the `s0` end to the `s1` end.
- The wall loop is the tool's natural rectangle (`s0` rim forward, seam up,
  `s1` rim back, seam down). The entry rim, reversed, replaces the
  protruding cap's rim. The cell requires the tool's wall to have that
  loop, which `make_cylinder` produces under any rigid placement.
- Faces follow the same breadth-first shell order. The floor joins the walk
  through its rim.

**Rim anchoring.** The GFA anchors a section rim at the wall surface
evaluated at the seam vertex's `u` and the section's `v`, and takes the
circle reference from that point. At small coordinates the rounding of
`cos(3π/2)` surfaces in the reference and the vertex: the magnet pockets
at x = 4 have a vertex at y = 3.9999999999999996. Both cells now use the
same arithmetic, so those cases are bit-identical as well.

**Attributes.** GFA results carry no solid or face attributes. Stage 1's
copy kept the blank's, so an attributed blank came out differently
depending on which path ran. Spliced copies now drop them, and the oracle
dump compares attributes.

**Evidence.** Main at `957201b8` against this stage, Apple M5 Pro, shared
machine, `RAYON_NUM_THREADS=1`.

Instruction counts (`/usr/bin/time -l`, base and candidate binaries
alternated, three rounds) for `examples/sequential_cut_profile.rs 12`:

| run | main | stage 2 |
| --- | ---: | ---: |
| 144 blind holes, instructions | 8.64–8.65 G | 0.525–0.526 G (−94%) |
| 144 blind holes, cycles | 1.91–1.96 G | 0.114–0.125 G |
| 144 blind holes, wall clock | 445–457 ms | 23–25 ms |
| 144 through holes, instructions | 0.707–0.713 G | 0.704–0.716 G (unchanged) |

Criterion, `gridfinity 3x3 baseplate (grid+holes)`, the four magnet
pockets: 955 µs → 92–96 µs (−90%, both rounds). `multi-boolean model` and
`boolean 64 cuts`, which take only through holes, showed no reproducible
change. A reverse-order rerun swung the same pair by ±10%, and the
instruction counts above are equal.

`boolean/local_cut_tests.rs` adds full-dump oracles for blind holes:

- drilled into the top face, the bottom face, downward and into a side
  wall;
- a 16-hole grid mixing through holes and three kinds of blind hole,
  compared after every hole;
- the gridfinity magnet pockets (0.15 floor), compared after every hole.

A rotated-frame blind hole matches structure for structure, with drift
below 1e-9.

It also checks that the cell declines:

- a floor within the clearance of the opposite face;
- a blind hole overlapping an existing hole;
- a hole drilled sideways out of a hole's wall;
- a side-wall entry that breaks out near a corner;
- a steeply tilted plate, whose tilted faces' boxes reach the tool's.

## Later stages (design)

1. **Widen the cell further.** Each extension needs its own proof and
   oracle cases:
   - rectangular and polygonal prism tools;
   - planes oblique to the axis, giving exact ellipse sections;
   - holes through cylindrical blank faces, such as cross-drilled shafts;
   - countersinks, cones whose planar sections are circles.
2. **Loop-local face splitting.** When every section on a face is a closed
   loop strictly inside it and clear of existing loops, the splitter can
   append the loop without recomputing pcurves, outlines and extents for
   the others. This removes the per-loop cost for general (non-cell) cuts
   too.
3. **Frozen untouched faces in the GFA.** A face of `A` whose conservative
   box is farther than the clearance from `B`'s box cannot meet `B`. The
   pave filler can skip its pair setup. The builder can pass it through
   without splitting or classification (outside `B`), and assembly passes
   can skip work that is provably a no-op for it. Each pass needs its own
   exactness argument; the duplicate-edge merge still sees such faces as
   partners of re-instantiated boundary edges.
4. **Store-local subsets.** Copy only touched faces (and a ring of
   neighbours) into the GFA store, and classify tool pieces against the full
   blank through a prepared classifier in the caller's topology. The store
   result is then spliced back onto the untouched faces through their shared
   boundary edges. This makes the GFA itself scale with the touched region.
   It needs the builder to assemble an open patch, which is the largest
   change.

Each stage must keep the current refusal behaviour outside its cell, and
must carry a differential oracle against the GFA and a deterministic
work-count guard.

## Stage 3, part 1: per-loop work on touched faces in phase FF (implemented)

Stage 3 makes the GFA's cost on a *touched* face follow the loops a cut
actually meets, not every loop the face carries. The general cuts the cells
leave behind need it: a rectangular slot, a tilted hole, a pocket. Its
first part is in phase FF.

**Measured cost.** `examples/touched_face_loops_profile.rs` drills an
`n × n` grid of holes (the stage 1 cell, cheap) and then cuts 2 × 6
rectangular slots between them. The slots meet only the plate's top and
bottom, which carry every hole. On main at `957201b8` a slot cost 1.7 ms
on a blank plate and 48 ms at 576 holes, about 80 µs per hole.

The profile did not match the design above, which expected the face
splitter to dominate. The largest per-hole cost was phase FF's junction
snap. Its next cost was rebuilding face extents. Both are now exact
no-ops for loops a section cannot reach:

- **Junction snap box gate.** `snap_to_boundary_junction_band` ran a
  65-sample scan plus 48 ternary steps on every boundary edge of both
  faces, for every section endpoint, then kept an edge only if it was
  within the snap band (about 1e-3). It now first measures the distance
  from the endpoint to a box that holds the whole curve:
  - circles and ellipses: `centre ± (a|u_i| + b|v_i|)` per axis, which
    holds every evaluated point whatever the axes' rounding;
  - NURBS with positive weights: the box of the control points.

  An edge whose box is farther than the band, after a rounding slack,
  cannot be adopted, so its sampling is skipped.
- **Face extents once per face.** Each face pair rebuilt both faces'
  `FaceExtent`, which samples every boundary edge 17 times plus a sagitta
  pass. It did this twice per pair (exact-arc assembly and curve
  restriction). An extent reads only the face's boundary, surface and
  `v_range`, and FF only adds vertices and edges, so it is fixed for the
  whole pass. `FaceExtents` now builds it on the face's first use.

The pairwise registry's cheap distance test also runs before its
per-entry wire walk now, with the same result.

**Exactness.**
- `phase_ff/junction_snap_oracle_tests.rs` keeps the ungated search
  verbatim. It compares the two bit for bit on about 30 000 queries over a
  plate holding circular, elliptical and NURBS holes and crossed by three
  walls, about 900 of which snap. The box bound is also checked against
  sampled curve distances, and an inflated bound fails both tests.
- The extent cache relies on the purity argument above. The full suites
  pass unchanged.

**Work guard.** `scaling_slot_cut_does_not_rescan_touched_face_loops`
(`test(scaling_)`, `perf-counters`) cuts a slot through plates with 36 and
144 holes:

| counter | 36 → 144 holes, fixed | reverted |
| --- | ---: | ---: |
| `junction_curve_scans` | 0 → 0 | 576 → 2304 |
| `face_extent_builds` | 6 → 6 | 32 → 32 |

**Evidence.** Main at `404d41cf` against this part, Apple M5 Pro, shared
machine, `RAYON_NUM_THREADS=1`. `touched_face_loops_profile` reports time
per slot cut, measured in-process:

| holes on the plate | main | part 1 |
| ---: | ---: | ---: |
| 36 | 3.74 ms | 2.72 ms |
| 144 | 12.4 ms | 6.95 ms (−44%) |
| 324 | 27.3 ms | 15.2 ms |
| 576 | 49.7 ms | 28.0 ms (−44%) |

Instruction counts (`/usr/bin/time -l`, binaries alternated, three
rounds):

- 100 slot cuts on a 324-hole plate: 70.7 G → 54.3 G for the whole run,
  including the drilling (26 → 14.2 ms per slot).
- 2000 slot cuts on a blank plate: 13.56 G → 13.52 G. The gate costs
  nothing when there is nothing to skip.

Criterion, two rounds against main:

- consistent wins: `cut(box,cyl) x10` −12%, `perforated_cut_36` −12 to
  −14%, `boolean 64 cuts` −4 to −5%;
- every other boolean bench moved within ±4% with changing sign, except
  single-round outliers that reversed in the other round
  (`intersect_box_box` −0.9% then +17%, `torus_notch_fuse` +12% then
  −0.3%). `cut_cylinder_through_box` never reaches phase FF and drifted
  +3% in both rounds, which sets the noise floor.

**Remaining per-hole cost.** After this part the slot cut costs about
44 µs per hole. The rest is spread over several passes:

| pass | share of the cut |
| --- | ---: |
| shell assembly (`perform_loops`, collinear-vertex arc splitting, duplicate-edge merge) | about 20% |
| face splitting: boundary pcurves for every existing hole (each rim sampled and NURBS-fitted) | about 8% |
| face splitting: hole containment | about 4% |
| GFA store copy and export | about 12% |
| phase EF edge-face scans | about 7% |

Part 2 is the loop-local splitter itself. Holes that no section touches
would pass through without pcurves, assigned to the sub-face that
contains them by a cheap exact test.

## Stage 3, part 2: untouched holes in the face splitter (implemented)

**Why not a literal pass-through.** `split_face_2d` chooses among more
than a dozen calibrated paths. Many of them branch on whether the face has
holes at all, or on how many it has, such as `original_inner_wires.is_empty()`
or `len() >= 2`. Withholding untouched holes from that list would change
which path runs, and so the result. The pass-through therefore keeps every
hole in the list and makes the per-hole work cheap and exact instead.

On a plane, hole pcurves never reach the topology: `fill_images` stores
coedge pcurves only on NURBS walls. Inside the splitter they feed two
per-hole costs:

- **Pcurve fitting.** Each arc rim is sampled sixteen times and fitted
  with an interpolating NURBS on every cut, about 6.6% of the slot cut. An
  arc's pcurve is a pure function of the curve, its traversal endpoints
  and domain, the plane and the plane frame. A content-keyed memo
  (`remus_algo::pcurve_memo`) keys on the exact bits of all of them and
  returns a clone of the first result, so a hit is bit-identical to
  recomputing.
  - It is opt-in and per thread, like the face-integral, volume, NURBS-area
    and face-mesh memos. `BrepKernel::new` enables it module-wide.
  - It is bounded at 4096 pcurves by default, oldest dropped first.
  - An unchanged hole rim is a hit on every later cut of the same face.
- **Hole containment.** A one-edge round hole has no chord polygon, so
  `is_inside_any_hole` resampled its fitted pcurve for every query point
  against every hole, about 3% of the slot cut. A box holding every point
  the sampler takes now skips holes the query lies outside. Outside that
  box, the even-odd crossing test cannot return true, so skipping is exact.
  The sampler takes line starts, NURBS evaluations (each a positive-weight
  average of control points) and chord points.

**Exactness.**
- `pcurve_memo` tests compare memoized and computed pcurves by exact
  `Debug` text, over full circles, open arcs both ways, ellipses and a
  circle one ulp larger than another, on two planes. They also check that
  every case gets its own key.
- `warm_pcurve_memo_leaves_a_slot_cut_bit_identical` cuts a slot through
  a drilled plate cold with the memo off, then again with a warmed memo, and
  compares the full results. The warm cut answers every arc from the memo,
  with no new misses.
- `containment/gate_oracle_tests.rs` keeps the ungated containment test
  verbatim. It agrees with the gated one on over 8 000 queries over round,
  elliptical, half-round and straight holes, many packed against the rims.
  A shrunken box fails it.
- The scaling guard gains `hole_containment_samplings`: 0 → 0 from 36 to
  144 holes, against 288 → 1152 with the gate removed.

**Evidence.** Part 1 (`fe944883`) against this part, Apple M5 Pro, shared
machine, `RAYON_NUM_THREADS=1`. Instruction counts (`/usr/bin/time -l`,
binaries alternated, three rounds):

| run | part 1 | memo off (library) | memo on (kernel) |
| --- | ---: | ---: | ---: |
| 100 slots on a 324-hole plate, whole run | 54.25–54.40 G | 53.18–53.43 G (−2%) | 50.59–50.71 G (−7%) |
| 2000 slots on a blank plate | 13.50–13.55 G | 13.51–13.52 G | 13.50–13.52 G |

Slot cuts are about 85% of the first run, so a slot costs about 8% less
with the memo warm. The gain is modest: the splitter's hole work was about
10% of the cut after part 1, and this part removes most of it.

Criterion over the boolean benches moved within about ±2% with changing
sign. `cross_drilled_cylinder` read +6% and +2%, but repeated 1000 times
it counts 21.21 G instructions on both branches, as do the gridfinity
magnet cuts and a box-cylinder cut (1.065 G and 0.268 G). Those readings
were noise.

**What remains.** With the memo warm, the splitter's per-hole work is about
6% of the slot cut: memo lookups and clones, and the winding sampling. The
remaining per-hole cost is untouched-face work outside the splitter. Shell
assembly and `orient_revolved_face_wires` revisit every hole wall, and the
GFA store copies every face in and out. That is the next stage, frozen
untouched faces.

## Frozen untouched faces, part 1 (implemented)

The design item says a face of `A` that cannot meet `B` needs no pair setup
in the pave filler and no-op work in assembly, each with its own exactness
argument. A slot cut through a drilled plate meets two faces of the blank.
Every hole wall, and every rim on the faces it touches, is untouched work.
Part 1 removes the passes that paid for them:

- **Phase EF outline sampling.** `build_face_containment` sampled every
  face's outline. For cylinder, cone, sphere and torus faces it then
  returned an empty containment, because a sampled box is not conservative
  on a curved patch. Those faces now return it before sampling. `perform`
  validates every edge domain of both solids first, so the sampling could
  not have failed.
- **Phase FF face boxes.** `compute_face_bbox` sampled every blank face.
  A face whose conservative box misses every expanded tool-face box now
  keeps that box. The conservative box is the union of
  `conservative_curve_aabb` over the face's edges, grown by a coordinate
  slack, and it contains the sampled one. So the rejection test refuses
  every pair such a face forms with either box, and no rejected pair reads
  the box again. Spheres and tori, whose boxes take surface extent, always
  sample.
- **Phase FF junction-snap boundaries.** Each snap walked every boundary
  edge of both faces: topology lookups, domain, box. The registry now
  keeps each face's boundary once per pass and walks the same candidates
  in the same order. A face with an edge lacking domain authority falls
  back to the walk, which returns `None` as before.
- **Phase FF pair extents.** The junction snap's band sampled both faces'
  boxes once per face pair. It now caches them once per face.
- **Assembly wire areas.** `orient_revolved_face_wires` measures every
  cylinder and cone wire's chart area: 33 samples per edge, each projected
  onto the surface. A content-keyed memo (`remus_algo::wire_area_memo`)
  keys on the exact bits of the surface and the wire's edges (direction,
  curve, endpoints, domain). A hit is bit-identical, so an unchanged hole
  wall is free on every later cut. It is opt-in and enabled by the WASM
  kernel, like the pcurve memo. The two now share one bounded store
  (`memo.rs`).
- **The operations wrapper's bounds check.** `result_within_operand_bounds`
  sampled the blank's full box on every boolean. It first tries the
  operands' vertex boxes, which lie inside the full ones. When every
  result vertex lies inside the allowed box built from them, with its
  margin, the full test would accept too. Otherwise the full test runs.
- `BK_RAWC` is read once per FF pass rather than once per face pair.

**Exactness.**
- `junction_snap_oracle_tests.rs` compares the cached-boundary snap with
  the verbatim ungated search on every query.
- The same file checks that a face keeps its conservative box only where
  its sampled box is rejected against every partner, and that the
  conservative box contains the sampled one.
- `memoized_wire_areas_are_bit_identical_and_keyed_on_content` compares
  memoized and measured areas bitwise. It covers cylinder and cone loops in
  both windings, and the same content under new entity ids hits.
- `warm_content_memos_leave_a_slot_cut_bit_identical` now warms both
  memos. The warm cut answers every wall's area from the memo.
- The scaling guard adds two counters. `face_bbox_builds` reads 14 → 14 and
  `face_outline_samplings` 12 → 12 from 36 to 144 holes; with the gates
  removed they read 54 → 162 and 48 → 156.

<!--FROZEN-EVIDENCE-->

**What remains.** About 10% of the slot cut is the GFA store copying every
face in and out, which is stage 4 (store-local subsets). Shell assembly's
linear passes (`perform_loops`, collinear-vertex arc splitting,
duplicate-edge merge) are another 20%. They visit every face once with
hashing-heavy bookkeeping. Skipping untouched faces there needs the open
patch assembly of stage 4. Short of that, only cheaper constant factors
help.
