# UV-arrangement splitter design

Status: accepted implementation design for Open Kernel O2.3a. The runtime
implementation remains staged as O2.3b-d and waits for P-Class 2.4 to release
the splitter files.

- Measured baseline: `main` at `74dd0e9732e57f1b5f7f43b0177dac8668d207f9`.
- Retirement target:
  `crates/algo/src/builder/face_splitter/special_cases.rs`.
- This note changes no boolean behavior and introduces no public capability.

## Problem and boundary

The face splitter has accumulated narrow topology emitters for configurations
that the greedy wire walk cannot represent reliably. They are valuable,
fixture-backed repairs, but each separately owns some combination of crossing
detection, edge splitting, loop tracing, periodic unwrapping, hole attachment,
and interior-point selection. The target file is 4,368 lines at the measured
baseline: 3,651 production lines and 717 test lines. It exposes ten callable
entry points and contains eighteen private production helpers.

O2.3 does not replace analytic intersection or invent new surface geometry.
It consumes the boundary and section curves already produced for one face and
builds their planar subdivision in that face's parameter space. Exact input
curves stay exact in the emitted topology; chord-tolerant polylines are only an
index for finding candidate crossings and ordering the embedded graph.

## Current entry-point inventory

The table inventories every callable entry point in `special_cases.rs`.
"Direct pin" means a test asserts the geometric contract of that path. A foil
asserts that its gate does not steal a configuration owned elsewhere. Where no
positive isolated pin exists, that is recorded as a coverage gap O2.3b must
close before migration.

| Entry point | Current gate and geometric patch | Pinning fixtures | Arrangement responsibility that replaces it |
|---|---|---|---|
| `split_noseam_face_direct` | A non-planar face whose boundary is all non-degenerate lines and whose open section arcs cannot use seam connections. Emits a cap plus a remainder; its private `split_noseam_by_arrangement` path reconstructs a sphere seam and selects longitude-winding collars when disjoint great-circle arcs cannot chain. | Direct: `crates/operations/src/boolean/tests.rs::{cut_sphere_by_through_cylinder_is_analytic_watertight,intersect_box_centered_sphere_is_analytic_collar}`; render oracle: `crates/operations/src/tessellate/tests.rs::{bored_sphere_band_area_and_watertight,box_centered_sphere_collar_tessellates_watertight}`; in-file seam-arc tests. | Insert boundary and section uses in one lifted chart, refine arc/boundary crossings exactly, extract all regions, and classify the sphere collar by periodic winding rather than constructing it directly. |
| `split_periodic_face_into_bands` | Cylinder/cone lateral with two closed rims, seam lines at one `u`, and seam-anchored closed constant-`v` sections. Emits `N+1` stacked bands instead of treating the sections as discs. | Direct: `crates/algo/src/pave_filler/tests.rs::gfa_cut_box_two_coplanar_cap_cylinders_sequential_valid`, `crates/operations/src/boolean/tests.rs::sequential_cylinder_cuts`, and `crates/wasm/src/bindings/gridfinity_tests.rs::sequential_cut_5_cylinders`. | Quotient a lifted periodic strip, alias its seam vertices, and let constant-`v` section cycles divide the strip into regions. |
| `split_closed_torus_into_bands` | Full torus with only degenerate seam boundary uses and seam-anchored constant-`v` circles. Emits `N` cyclic bands and represents constant-`u` seams as exact meridian arcs split below pi. | Direct: `crates/operations/tests/qualify_torus_boolean.rs::{axis_perpendicular_plane_halves_torus,tilted_plane_through_centre_halves_torus,concentric_sphere_inclusion_exclusion}`. | Build in a doubly periodic lifted rectangle, carry both seam equivalence classes, and quotient cyclic regions without inventing boundary rims. |
| `split_periodic_face_into_sectors` | Cylinder with two rims and full-height constant-`u` rulings. Rescues an under-split strip only when the greedy walk produced at most one loop. | Foil: `crates/operations/src/boolean/tests.rs::fuse_corner_poking_cylinder_stays_analytic`; adjacent identity pin: `crates/algo/src/builder/same_domain/tests.rs::chord_split_disc_halves_are_not_duplicates`. There is no positive isolated sector contract today. | The periodic strip arrangement must emit angular regions from rulings plus the seam. O2.3b adds a positive permutation/determinism fixture before this path can migrate. |
| `split_torus_band_by_arrangement` | Torus-minus-box class: open arcs stitch into exactly two loops that each wrap the tube angle; emits the long kept `u` band between them. | Direct: `crates/operations/src/boolean/tests.rs::cut_torus_by_box_notch_is_analytic_watertight`; render oracle: `crates/operations/src/tessellate/tests.rs::torus_box_notch_band_tessellates_watertight`. | General doubly periodic region extraction must retain loop winding in both axes and select the same long band through ordinary region classification. |
| `split_face_with_internal_loops` | All sections are contractible interior cycles. Emits each removable disc plus the outside face with reversed holes; also unions overlapping all-line holes and finds annular seeds for re-bored openings. | Direct families: `cut_sphere_by_through_cylinder_is_analytic_watertight`, `fuse_perpendicular_cylinders_is_analytic_watertight`, `fuse_capping_slab_preserves_drilled_hole_caps`, `fuse_counterbore_drops_drill_rims_inside_opening`, `crates/io/tests/snapclip_slot_cut_inmem.rs`, and `crates/io/tests/deepened_wall_opening_inmem.rs`. | Extract nested regions from the subdivision, classify by even-odd containment, and attach each hole to the smallest containing material region. Overlap intervals become first-class events instead of a separate union pre-pass. |
| `cylinder_cone_remainder_interior` | Finds a classification point on a partial or full cylinder/cone remainder carrying curved lens holes. It samples the true boundary span because a closed curved hole has coincident endpoint UVs and defeats endpoint polygons. | Direct in-file: `hole_containment_even_odd_inside_vs_outside`, `hole_containment_handles_seam_crossing_loop`, `remainder_interior_point_is_outside_the_lens_hole`, and `remainder_interior_found_for_thin_kept_strip`; integration: `fuse_perpendicular_cylinders_is_analytic_watertight`. | Every extracted region carries a certified interior seed derived from its embedded boundary and periodic lift. Consumers no longer search the surface after topology construction. |
| `chain_boundary_edges` | Greedily reorders and reverses split plane-boundary edges before the crossing-plane emitter consumes them. | Transitive only through the crossing-plane fixtures; there is no isolated permutation or refusal contract. | DCEL half-edge ordering replaces greedy chaining. O2.3b must pin permutation invariance and typed refusal of a non-closing boundary before deletion. |
| `try_split_crossing_plane_face` | Direct construction for two crossing sections (`X`), a two-section `T`, or four rays meeting as two opposing pairs. Splits the outer boundary at ray endpoints and emits three or four regions. | Direct historical contract: `crates/algo/src/pave_filler/tests.rs::gfa_intersect_overlapping_boxes`; foils include `gfa_cut_touching_boxes`. The later three-plus-line wall family is owned by the existing general planar arrangement, not this helper. | Ordinary segment insertion and region extraction; no arity-specific construction. |
| `try_split_disk_by_chords` | A plane disc with a single analytic circle boundary cut by line chords. It preserves major arcs, uses tangent-aware turns, and midpoint-splits a co-endpoint lens arc so the shared merge cannot collapse it. | Direct in-file: `disk_cut_by_corner_chords_yields_two_analytic_regions`, `disk_cut_by_diameter_yields_two_half_discs`, `disk_lens_arc_is_split_to_break_coendpoint_collision`, `non_disc_boundary_defers`, and `circle_section_defers`; wider canaries: `crates/io/tests/socket_junction_disc_inmem.rs` and `crates/io/tests/gridfinity_honeycomb_cut_inmem.rs`. | Insert analytic arc uses with exact tangent ordering and refined chord crossings. Preserve source sub-spans so a major arc and its chord remain distinct. |

`chain_boundary_edges` is callable only inside `special_cases.rs`, but it is
listed because its `pub(super)` visibility makes it part of the measured
retirement surface. `cylinder_cone_remainder_interior` is also consumed by
`fill_images_faces.rs`, so migrating the splitter alone is insufficient to
delete it until the arrangement supplies certified region seeds.

## Adjacent arrangement-shaped code

The target file is not the only place with subdivision logic. O2.3b must reuse
or absorb these current paths rather than placing a second generic engine
beside them:

- `split_plane_face_by_arrangement` and
  `arrangement_regions_from_combined` in `face_splitter/mod.rs` already split
  line/arc soups and repair disconnected-loop twins.
- `split_cylinder_band_by_arrangement` handles a rectilinear cylinder chart
  only after the greedy result is proven broken.
- `split_periodic_face_by_winding_chain` directly emits two bands around a
  seam-anchored winding separator.
- `build_wire_loops_dcel` in `builder/wire_builder.rs` is a loop tracer, not a
  subdivision owner: it receives already-created edges and cannot refine
  crossings or represent periodic identifications.

The new module becomes the one owner of crossing refinement, vertex identity,
half-edge connectivity, region extraction, seam equivalence, pole equivalence,
and deterministic ordering. Existing narrow emitters remain fallbacks until
their own direct fixtures pass through it.

## Arrangement API

The implementation lives in
`crates/algo/src/builder/face_splitter/arrangement.rs` and remains internal to
`remus-algo`.

```rust,ignore
pub(super) struct ArrangementInput<'a> {
    pub surface: &'a FaceSurface,
    pub domain: ParamDomain,
    pub boundary_loops: &'a [Vec<CurveUse>],
    pub sections: &'a [CurveUse],
    pub tolerance: Tolerance,
    pub context: &'a OperationContext,
}

pub(super) struct CurveUse {
    pub curve_3d: EdgeCurve,
    pub pcurve: Curve2D,
    pub parameter_range: (f64, f64),
    pub endpoints_3d: (Point3, Point3),
    pub source: CurveSource,
}

pub(super) struct Arrangement {
    pub vertices: Vec<ArrangementVertex>,
    pub half_edges: Vec<ArrangementHalfEdge>,
    pub regions: Vec<ArrangementRegion>,
    pub identifications: Vec<DomainIdentification>,
}

pub(super) fn build_arrangement(
    input: &ArrangementInput<'_>,
) -> Result<Arrangement, ArrangementError>;
```

`CurveSource` preserves the originating boundary/coedge, section,
`source_edge_idx`, and `pave_block_id` plus the exact parameter sub-span. The
output can therefore build `SplitSubFace` wires without fitting new geometry
or guessing cross-face edge identity.

`ParamDomain` contains the face's finite working window, optional `u` and `v`
periods, seam lifts, and singular boundary descriptors. It is derived from
authoritative boundary uses, not from the untrimmed surface's arbitrary
natural bounds.

Each output half-edge has a twin, `next`, source sub-span, lifted UV endpoints,
and exact 3D endpoints. Each region contains ordered outer and inner cycles,
integer winding in every periodic axis, a certified interior point, and the
source uses that bound it. The unbounded lifted-chart region is explicit and
never inferred by "largest area" after periodic quotienting.

## Construction pipeline

1. **Validate and lift.** Reject non-finite coordinates, invalid parameter
   ranges, missing pcurves, or disconnected authoritative boundary loops.
   Choose a deterministic lift from the first authoritative outer-boundary
   use and duplicate only the periodic images needed by the working window.
2. **Discretize for discovery.** Sample each exact use to a polyline whose
   chord deviation is bounded by the operation tolerance. Store the exact
   source parameter interval behind every chord.
3. **Find candidates.** Use chord AABBs and `orient2d` to find proper
   crossings, endpoint-on-use events, tangent contacts, and collinear overlap
   candidates. Broad-phase proximity never creates vertex identity.
4. **Refine on exact uses.** Solve candidate parameters against the original
   pcurves/3D curves. Point intersections carry both source parameters;
   coincident portions carry paired overlap intervals. An unresolved
   tangent/overlap is a typed refusal, not a snapped vertex.
5. **Create event vertices.** Split every use at its ordered event parameters.
   Establish identity from certified endpoint or intersection events and
   existing topology provenance.
6. **Build the DCEL.** Sort outgoing half-edges by robust orientation and
   limiting tangent, set twins and `next`, then trace all lifted-chart cycles.
7. **Classify and quotient.** Compute containment and winding before applying
   seam/pole identifications. Attach a hole to the smallest containing
   material region and return deterministic region order.

Every iteration and refinement loop consumes `OperationContext` work budgets
and polls cancellation. Partial arrangements are never exposed.

## Vertex model: exact-predicate events, not snap-rounding

O2.3 uses exact-predicate event vertices. It does **not** globally snap-round
coordinates to a tolerance grid.

The reasons are structural:

- Kernel tolerance is a proof band, not permission to erase a sliver. A grid
  can collapse two legitimate thin regions or change winding.
- Co-endpoint line/arc pairs sometimes must merge and sometimes must remain
  distinct. The roadmap's proven-unbuildable universal merge key shows that
  proximity alone cannot decide identity.
- Periodic seam copies are equal by domain identification even when their UV
  coordinates differ by a full period; pole uses can have arbitrary `u` while
  representing one 3D point. A Euclidean UV snap expresses neither relation.

`orient2d` supplies robust combinatorial signs for chord events. Ambiguous
near-zero signs trigger exact-curve refinement. Vertices merge only when one
of these certificates exists:

1. the uses share an authoritative topology endpoint;
2. exact refinement returns the same curve-pair event parameters;
3. provenance identifies both uses with the same pave-block event; or
4. a declared seam or pole identification equates their lifted copies.

Deterministic union-find applies those certificates in sorted source-key and
parameter order. The tolerance is used to verify a certificate's residual,
never to manufacture one from proximity.

## Periodic seams and poles

For a `u`- or `v`-periodic domain, curves are first split at every seam
crossing and represented in an extended chart. The arrangement records the
integer lift on each endpoint. Opposite chart boundaries are paired only
after region tracing; winding is the lift delta divided by the period. This
expresses cylinder bands, angular sectors, torus bands, and winding chains
without special emitters.

For spheres and cones, a pole is one topological vertex with many valid UV
representatives. Incident uses terminate in a `Pole` identification carrying
their limiting 3D tangent. Outgoing order is computed in a deterministic
tangent-plane frame, not by arbitrary `u` at the singularity. A curve merely
near a pole remains distinct; only a residual-verified surface singularity is
identified.

A doubly periodic torus uses both identifications simultaneously. Region Euler
checks are evaluated in the lifted chart and again after quotienting with the
expected domain topology; the unbounded planar formula is not applied to the
quotient torus.

## Typed failure contract

The internal error enum is stable enough for callers to map into the existing
boolean diagnostics:

```rust,ignore
pub(super) enum ArrangementError {
    NonFiniteInput,
    InvalidBoundary,
    MissingParametricCurve,
    ProjectionFailed,
    IntersectionRefinementFailed,
    AmbiguousOverlap,
    NonManifoldEmbedding,
    OpenRegion,
    WorkBudgetExceeded,
    Cancelled,
}
```

Each error receives a positive and negative contract test. O2.3c runs the new
path behind an internal option and falls back only when policy explicitly
permits the existing splitter; it never converts an arrangement failure into
empty sections or a plausible unsplit face. O2.3d maps unsupported cases to a
stable typed boolean refusal before removing their old special path.

## Determinism and property tests for O2.3b

The isolated core is not ready until all of these pass:

- random segment/arc soups preserve DCEL twin/next consistency, close every
  bounded region, and satisfy the planar Euler formula per connected
  component;
- input-use permutations and reversed-but-equivalent input ordering produce
  byte-identical canonical arrangements;
- duplicated disconnected cycles emit one region plus correct containment,
  never orientation twins as overlapping material;
- major/minor arcs, `X`/`T`/star crossings, tangent contacts, and overlap
  intervals have exact source sub-spans;
- cylinder/cone strip seams, torus double seams, sphere/cone poles, and
  seam-crossing holes retain winding and quotient Euler invariants;
- zero work budget and pre-cancelled contexts return typed failures without a
  partial result;
- positive isolated fixtures are added for the periodic-sector path and
  boundary-chain permutation gap identified above.

Use deterministic maps/sets and sort all emitted IDs by source key,
parameter, and lift. Randomized tests must replay a printed seed.

## Migration and deletion plan

O2.3c first differential-tests the arrangement against the current splitter on
the complete in-repo fixture corpus. Agreement means the same kept material,
analytic curve/surface types, manifold edge uses, and oracle volume—not merely
the same face count. Explained divergences remain pinned.

O2.3d migrates one family per PR, with its direct pins routed through the new
engine before deleting the old code:

1. plane `X`/`T`/star plus `chain_boundary_edges`;
2. analytic disc/chord regions;
3. contractible internal loops and overlapping holes;
4. cylinder/cone bands, sectors, and winding chains;
5. sphere noseam collars and pole cases;
6. closed and notched torus bands.

The first two migrations deliberately establish at least three deleted entry
points (`try_split_crossing_plane_face`, `chain_boundary_edges`, and
`try_split_disk_by_chords`) before the periodic cases. A CI ratchet introduced
with the first deletion records production lines and callable entry points in
`special_cases.rs`; neither count may rise without an explicit ledger update.

Every boolean-adjacent migration runs `approx_census`, the full workspace,
`remus-wasm` Gridfinity contracts, the affected criterion benches, and the
position-quantized manifold checks required by the roadmap skill. O2.3a itself
is a design-only issue: it adds no runtime capability, so R8 does not require a
new WASM method or batch operation. Later migrations change the existing
boolean capability and therefore must prove the existing direct and batch WASM
contracts remain exact-or-typed.

## Non-goals

- No new analytic intersection solver or surface/curve variant.
- No rewrite of GFA, pave filling, same-domain merging, or classification.
- No universal proximity merge key and no global tolerance-grid snapping.
- No O2.3b-d source change before P-Class 2.4 releases the splitter files.
- No claim that a validation-only or face-count-only comparison is a geometry
  oracle.
