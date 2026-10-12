# Local boolean for sequential cuts

**Status (2026-10-12):** stage 1 implemented — the qualified through-hole
cell in `crates/operations/src/boolean/local_cut.rs`. Later stages are
design only.

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
`local_cut::try_local_through_hole` proves the cell and builds that result
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
- Blind holes, which the cell always declines: 631–641 M on both binaries.
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
- blind holes;
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

## Later stages (design)

1. **Widen the cell.** Each extension needs its own proof and oracle cases:
   - blind holes, where one cap lies strictly inside `A` and becomes the
     hole floor;
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
