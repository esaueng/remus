# UV-arrangement splitter design

Status: accepted implementation design for Open Kernel O2.3a. O2.3b has a
bounded isolated runtime slice described below plus a cylinder-strip quotient
fix (single-ruling shared-seam adjacency, roundoff-tolerant seam-`v` matching)
and a bounded production cylinder slice (O2.3b/c/d) described in the appendix;
planar production integration (O2.3c) has a last-resort site and cylinder
production an early-path site, both routed to legacy today (see
[O2.3d routing](#o23d-routing-is-a-constant-not-a-budget-2026-11-13)). No
emitter deleted in the cylinder slice; the
rest of O2.3b-d migration and the three-entry-point deletion floor remain
staged. Shared-file ownership was rechecked before each runtime slice.
O2.3d step 0 (2026-11-12) added the special-case ratchet and a shadow
differential that measures the arrangement adapters against the legacy
emitters on real faces; it migrated and deleted nothing (see
[O2.3d step 0](#o23d-step-0-ratchet-and-shadow-differential-2026-11-12)).
The budget-policy research that followed (2026-11-13) measured what work the
adapters need and recommends a policy; it changed no behaviour (see
[O2.3d budget policy](#o23d-budget-policy-measurements-and-recommendation-2026-11-13)).

- Measured baseline: `main` at `fc4b55a1855514a6cbe206d86572cf9b3ee362e2`.
- Retirement target:
  `crates/algo/src/builder/face_splitter/special_cases.rs`.
- This note changes no boolean behavior with default budgets and introduces no
  public capability (both production adapter sites are routed to the
  established paths whatever the budgets; the adapters run directly in their
  own tests and in the shadow differential).

## Problem and boundary

The face splitter has accumulated narrow topology emitters for configurations
that the greedy wire walk cannot represent reliably. They are valuable,
fixture-backed repairs, but each separately owns some combination of crossing
detection, edge splitting, loop tracing, periodic unwrapping, hole attachment,
and interior-point selection. The target file is 4,368 lines at the measured
baseline: 3,651 production lines and 717 test lines. It exposed ten callable
entry points and contained eighteen private production helpers there. On
2026-11-12 it is 7,960 lines: 6,566 production lines and sixteen callable
entry points, the counts `scripts/check-splitter-ratchet.py` now pins.

O2.3 does not replace analytic intersection or invent new surface geometry.
It consumes the boundary and section curves already produced for one face and
builds their planar subdivision in that face's parameter space. Exact input
curves stay exact in the emitted topology; chord-tolerant polylines are only an
index for finding candidate crossings and ordering the embedded graph.

## Current entry-point inventory

The table inventories every callable entry point in `special_cases.rs`: the
ten of the O2.3a baseline first, then the six added since (inventoried
2026-11-12). "Direct pin" means a test asserts the geometric contract of that
path. A foil asserts that its gate does not steal a configuration owned
elsewhere. Where no positive isolated pin exists, that is recorded as a
coverage gap to close before migration.

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
| `is_sphere_equator_partition` | Gate, not an emitter: recognises a sphere hemisphere boundary made of circle pieces on one stored frame, one direction, whose native spans join into exactly one turn. `split_face_2d_impl` then treats that analytic rim as seam-free and routes the face to `split_noseam_face_direct`. | Direct: `crates/algo/src/builder/face_splitter/special_cases/equator_partition_tests.rs::{complete_equators_admit_both_directions_scales_and_wire_starts,incomplete_retraced_and_malformed_partitions_decline,a_common_circle_that_is_not_the_sphere_equator_declines}`. | A pole-aware sphere chart makes the equator an ordinary boundary cycle; no boundary-shape gate selects an emitter. |
| `sphere_closed_loop_interior` | Interior point of a contractible closed loop on a sphere, in a chart whose pole is placed away from the cap so concave caps stay inside. Used by the noseam emitter and by `interior_point_3d` for all-NURBS sphere loops. | Transitive only, through the sphere collar and cap fixtures of `split_noseam_face_direct`; no isolated contract. | Every extracted region carries a certified seed; no post-hoc sphere search. |
| `split_converted_wall_into_bands` | The band shortcut for a NURBS face the exact-rational wall recognizer accepts (closed NURBS rims plus seam lines) cut by closed transverse rings at distinct chart `v`. Emits `N+1` stacked bands. | Direct in-file: `converted_wall_with_one_transverse_ring_splits_into_two_bands`. | The periodic strip arrangement on the wall chart, once exact-rational carriers are admitted as uses. |
| `converted_wall_remainder_interior` | Classification point on a converted-wall remainder carrying a curved hole loop, by a chart-grid search for the point farthest from every hole. Called only inside `special_cases.rs` despite its `pub` visibility. | Direct in-file: `converted_wall_remainder_interior_lands_below_a_transverse_section_ring`. | Certified region seeds, as for `cylinder_cone_remainder_interior`. |
| `cylinder_cone_loop_interior` | Interior point of a cylinder or cone sub-face bounded by one contractible window loop: the 3D centroid of the loop lies off the wall, so the wall point inside the loop farthest from it is searched in the chart extent of the loop. Called only by the internal-loops emitter. | Transitive only, through the internal-loops fixtures (`fuse_perpendicular_cylinders_is_analytic_watertight`, `crates/io/tests/snapclip_slot_cut_inmem.rs`). | Certified region seeds. |
| `relocate_clear_seam` (`cylinder_seam`) | Pre-pass, not an emitter: moves the artificial seam of a complete cylinder or cone band to the antipodal meridian when whole-carrier bounds prove a mixed ellipse/NURBS section chain clear of it, preserving rim carriers, trims and source lineage. | Direct in-file: `cylinder_seam::tests::{cylinder_seam_relocation_preserves_carriers_trims_orientation_and_rim_lineage,cone_seam_relocation_preserves_native_rim_winding_and_carriers}` and their refusal cases; integration: `crates/operations/tests/regress_opseq_rotated_cylinder_cut.rs`. | Automatic periodic lifting chooses the working window from the boundary and section uses, so no seam has to be moved before the split. |

`chain_boundary_edges` is callable only inside `special_cases.rs`, but it is
listed because its `pub(super)` visibility makes it part of the measured
retirement surface; the same holds for `converted_wall_remainder_interior`
and `cylinder_cone_loop_interior`. `cylinder_cone_remainder_interior` is also
consumed by `fill_images_faces.rs`, so migrating the splitter alone is
insufficient to delete it until the arrangement supplies certified region
seeds. Three of the six later entries are not emitters at all (a boundary
gate, a seam pre-pass and an interior search): the file grew by accreting
support code around the emitters as well as by adding them.

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

## First O2.3b runtime slice (2026-09-26)

The isolated internal implementation is now in
`crates/algo/src/builder/face_splitter/arrangement.rs`. It is compiled in ordinary
builds but has no production splitter dispatch call. O2.3b remains **partial**.

The starting head was `26c5559a7cd1081823f0032516b5f34a64da9e11`. P-Class 2.4a/b
(#206/#207) and the bounded torus-sphere work (#299) were merged, and no open PR
in the inspected queue owned splitter files. This is a file-ownership release,
not completion of general P-Class 2.4.

### Supported input and output contract

- Inputs are certified per-use pcurves in one existing chart, not raw topology
  awaiting projection. Native line intervals and circle intervals within
  `[0, 2π]` are supported, including full circles and major arcs. A signed
  affine map carries pcurve parameters to the original source interval. Source
  lines interpolate the use endpoints; their provenance interval is never
  silently rebased. Circle carriers evaluate their native source parameters.
- Boundary loops carry endpoint-event identities and optional authoritative
  face/Loop/Coedge identity. The caller must retain actual coedge identity when
  adapting topology. Boundary loop keys must describe single closed connected
  cycles. Sections carry source-edge, section and pave-block provenance.
- The result owns the sorted original exact carriers. Each half-edge references
  its source use and carries both exact analytic subspans, directed 3D endpoints,
  twin and next. New vertices come from authoritative endpoints or analytic
  pair events. Cardinal circle cuts only split a source; they do not establish
  intersection identity. Floating roots are residual-checked; unresolved root
  collisions refuse. Multi-pair junctions require a common endpoint certificate
  or exact line-incidence signs. General curved multiway junctions are deferred.
- Areas and region seeds are in the supplied UV chart. Analytic Green integrals
  and horizontal-ray crossings retain circular segments. Negative cycles attach
  to the smallest containing positive cycle, or to the explicit exterior.
  Boundary parity selects material independently of input traversal direction;
  sections partition that material without changing it.
- The singly periodic input is a **pre-cut rectilinear cylinder strip** with
  explicit left/right seam-use IDs one turn apart. Exact rulings and latitude
  circles are supported. Matching seam subdivisions are verified after planar
  tracing, then quotient cells join across those seam uses. Results carry seam
  vertex identifications, quotient boundary windings and Euler characteristics.
  Cylinder area multiplies UV area by the supplied cylinder radius. The caller
  certifies this metric and the pcurve/source correspondence.

This boundary intentionally defers automatic periodic image generation and
clipping, oblique cylinder traces, seam-crossing hole input adaptation, curved
multiway contacts, all overlap emission, tangent limiting-order resolution,
noncanonical circle branches, poles, and double-periodic domains. Coincident
intervals return `AmbiguousOverlap`; tangencies and unresolved near-tangencies
return `AmbiguousContact`. Open/slit regions, inconsistent event certificates,
unsupported domains, non-finite data, exhausted budgets and cancellation are
errors, never unsplit-face or partial-result success. No tolerance is increased.

### Reuse and isolation

The rotation-system successor bijection was extracted from
`build_wire_loops_dcel` into `crates/algo/src/builder/rotation_system.rs`. Both
tracers use that same clockwise-predecessor rule. The existing wire builder
retains its input collection, angular ordering, periodic handling and output
selection. The new core reuses `Curve2D`, robust `orient2d`, exact 3D carriers,
and `OperationContext`.

The inventory also examined `arrangement_regions_from_inputs`,
`split_cylinder_band_by_arrangement`, and the winding-chain emitter. Their
quantized/chord identity and calibrated topology emission are deliberately not
copied into the exact-event core. They remain production fallbacks pending the
O2.3c material-differential gate; no special case was deleted or rerouted.

### Resource policy and qualification fixtures

The isolated core uses `OperationContext.budgets.march_steps` as its total work
counter, `queue_size` as its input/event cap, and `segments` as its undirected
edge cap. These are explicit arrangement interpretations of existing fields;
there are no hidden unlimited refinements. Callers must provide budgets suitable
for the arrangement size; the SSI-oriented defaults do not promise success for
an arbitrary arrangement. Refinement, pair scans, seed searches, graph walks and
quotient traversal poll cancellation and charge work. All staged state is local.

`crates/algo/src/builder/face_splitter/arrangement/tests.rs` is the representative
fixture corpus for later differential integration. It covers X/T/star and true
interior crossings; major/minor circle caps and two-circle lenses; nested holes,
islands and disconnected components; authoritative coedge/source subspans;
sub-tolerance slivers; cylinder bands, seam-crossing sectors and a strip with a
hole; contact/overlap, malformed-input and unrepresentable native-subspan
refusals; and all work-budget prefixes
of a complete square build. A cancellation test covers an already-used work
counter, and the build entry point separately covers pre-cancellation.

Independent oracles include closed-form cap/lens/annulus/rectangle areas,
sampled-curve material probes, known cylinder metric areas and quotient Euler
values. Generated line grids and translated/rotated circle pairs exercise
connectivity, planar Euler, source coverage, orientation reversal and canonical
output. The four-edge boundary fixture checks all 24 input permutations and
16 orientation masks; generated and cylinder cases also permute/reverse inputs.
These are UV-subdivision qualifications, not Boolean capability, installed-WASM
qualification, or a completed production differential comparison.

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
`try_split_disk_by_chords`) before the periodic cases. The CI ratchet
(`scripts/check-splitter-ratchet.py`, introduced ahead of the first deletion
in O2.3d step 0) records production lines and callable entry points in
`special_cases.rs`, the named emitters in `face_splitter/mod.rs` and the
early returns of the dispatch chain; no count may rise without a ledger line
naming a roadmap row.

Every boolean-adjacent migration runs `approx_census`, the full workspace,
`remus-wasm` Gridfinity contracts, the affected criterion benches, and the
position-quantized manifold checks required by the roadmap skill. O2.3a itself
is a design-only issue: it adds no runtime capability, so R8 does not require a
new WASM method or batch operation. Later migrations change the existing
boolean capability and therefore must prove the existing direct and batch WASM
contracts remain exact-or-typed.

## Appendix: bounded production cylinder slice (2026-09-30)

Bounded class: finite `Cylinder` lateral, no holes, boundary two closed
latitude rims + seam lines at one `u`, sections exact latitude circles
(constant-`v`, seam-anchored, strictly between rims) and/or full-height axial
rulings (constant-`u` lines, rim-to-rim, not seam-coincident). All strip uses
are axis-aligned lines with exact 3D carriers (`Circle` for latitudes/rims,
`Line` for rulings/seams); original carrier parameters, source/coedge
identity, section/pave provenance and both seam uses are preserved. Oblique
traces, Ellipse/NURBS contacts, coincident overlaps, tangent contacts, cone
poles, sphere poles and double-periodic torus charts decline (`Ok(None)`) to
the established paths; claimed-domain failures propagate (`Err`), never an
unsplit face.

Topology-to-chart adaptation (`arrangement_cyl::collect_cylinder_uses`) builds
the pre-cut strip rectangle (bottom/right/top/left, seam uses `[3,1]`, radius)
from authoritative loops/coedges, with distinct lifted certificates for
left/right seam copies (seam equivalence declared, never inferred). Exact seam
subdivision and quotient reconstruction reuse the isolated core
(`arrangement::build_arrangement` with `CylinderStrip`); the quotient fix for
single-ruling shared-seam adjacency (distinct sectors sharing seam *and*
ruling stay separate; wrapping cells sharing only seam merge) plus
roundoff-tolerant seam-`v` matching are in `arrangement/periodic.rs`. Region
winding, metric area (`radius * UV area`), certified interior seeds and exact
source subspans are carried into `SplitSubFace` (`emit_cylinder_subfaces`;
non-wrapping via lifted cycles with seam, wrapping via quotient boundaries
seam-free). Production dispatch (`split_face_2d_impl`) tries the cylinder
arrangement early for `Cylinder` faces (before bands) when the family is
routed to it (it is not today, see
[O2.3d routing](#o23d-routing-is-a-constant-not-a-budget-2026-11-13)),
declining on out-of-domain inputs; established
band/sector/rectilinear/DCEL paths remain as fallbacks. No emitter deleted in
this slice (sectors/bands retained; differential agreement evidenced with
opt-in budgets, closed-form `2πrΔz` bands and `rθh` sectors, permutation/
reversed/shuffled/rigid/scale/thin qualifications, atomic budget/cancellation).

### Certified seam adjacency (2026-10-20)

The quotient now identifies a shared non-seam edge through its DCEL twin's
region owner. A budgeted scan indexes each undirected edge once; the seam
pairing then looks up region pairs. Endpoint proximity is not an adjacency
certificate: two distinct rulings can have endpoints closer than linear
tolerance without sharing any edge. The previous nested endpoint scan kept
such seam-adjacent cells apart, yielding three sectors where two physical
sectors were supplied.

`arrangement/oracle_tests.rs::close_distinct_rulings_preserve_topology_area_and_source_spans`
qualifies close rulings, rulings next to opposite seam copies, band
subdivisions, three linear tolerances, reversed uses and shuffled source order.
The oracles check sector counts, genus-zero Euler and winding, closed-form
metric areas, both original ruling sources and native spans. Grid quotient
work is bounded by its edge/cell count, and every smaller budget prefix plus
cancellation refuses. The production adapter regression
`arrangement_cyl::tests::close_distinct_rulings_emit_two_sectors_without_an_extra_seam`
uses radius 0.001 and a 5e-6-radian ruling separation: it remains inside the
adapter's existing angular domain while the endpoint separation falls below
the default linear tolerance. It checks two exact cylindrical sector wires,
closed-form areas, both original line endpoints and pave-block provenance.

The stale single-ruling annulus oracle is replaced with an active two-sector
contract: preserving the real supplied section requires retaining its two
boundary uses. Both sectors retain the artificial seam and have one boundary
and Euler characteristic one. This matches the existing production emitter;
the plain strip without a ruling remains an annulus. Tier: core (the splitter
supplies boundaries for live-session boolean topology). Default-budget
fallbacks and the declared rectilinear-cylinder domain remain unchanged.

Out of scope (remain explicitly outside unless independently qualified):
oblique traces, unsupported contacts/overlaps, cone poles, sphere poles,
double-periodic torus charts. Terminal equal-radius perpendicular-cylinder
render work is not reopened (needs face-split-at-pinch primitive).

## O2.3d step 0: ratchet and shadow differential (2026-11-12)

Step 0 is measurement infrastructure. It changes no dispatch order, adoption
gate, budget or emitted geometry, and migrates and deletes nothing.

**Ratchet.** `scripts/check-splitter-ratchet.py` (Repository Policy job)
pins, in `scripts/splitter-ratchet-baseline.txt`: 6,566 production lines,
16 entry points and 34 `pub` functions in `special_cases.rs`; the line counts
of `split_periodic_face_by_winding_chain` (305),
`split_periodic_face_by_rim_chains` (496),
`arrangement_regions_from_combined` (36) with its worker
`arrangement_regions_from_inputs` (1,228), `split_plane_face_by_arrangement`
(83) and `split_cylinder_band_by_arrangement` (610); and the 19 early-return
dispatch sites of `split_face_2d_impl`. A rise needs a ledger line naming a
roadmap row; a reduction must be banked in the same change.

**Shadow differential.** `face_splitter/shadow.rs` exists only under
`cfg(any(test, feature = "perf-counters"))` and is inert until
`REMUS_SPLITTER_SHADOW` (or a test) opts in. Where `split_face_2d_impl`
adopts the result of `try_split_crossing_plane_face` (with its only-caller
helper `chain_boundary_edges`), `try_split_disk_by_chords`,
`split_periodic_face_into_sectors` or the cylinder arm of
`split_periodic_face_into_bands`, it hands the same face to the planar or
cylinder arrangement adapter with no split registry and compares: region
count, carrier kinds per wire up to rotation, vertices within the linear
tolerance, and each arc's native carrier and signed sub-span. The legacy
result is always returned. Counts per entry point sit in
`gfa::SectionDropScope::splitter_shadow`.

**Engagement census** (`python3 scripts/splitter-shadow-census.py`; the
seven foil families of the roadmap skill, twelve test targets, measured
2026-11-13). Default budgets are `WorkBudgets::new()`; raised budgets are
march steps 2,000,000, queue 20,000, segments 10,000.

| Legacy entry point | Budgets | fired | agreed | disagreed | declined | declined on budget | refused |
|---|---|---:|---:|---:|---:|---:|---:|
| `try_split_crossing_plane_face` | default | 6 | 0 | 0 | 0 | 6 | 0 |
| `try_split_crossing_plane_face` | raised | 6 | 6 | 0 | 0 | 0 | 0 |
| `try_split_disk_by_chords` | either | 0 | 0 | 0 | 0 | 0 | 0 |
| `split_periodic_face_into_sectors` | either | 0 | 0 | 0 | 0 | 0 | 0 |
| `split_periodic_face_into_bands` (cylinder) | either | 0 | 0 | 0 | 0 | 0 | 0 |

What the table says:

- The six crossing faces are two-section `T` splits into three regions: two
  in d4 (`d4_shelled_box_fuse_lip`) and two in each of two honeycomb tests.
  Under raised budgets the arrangement reproduces all six exactly. Under
  default budgets it produces none of them: every one exhausts the work
  budget. The planar adapter's budget, not its domain, is what keeps it from
  these faces today.
- The foil corpus never reaches the disc, sector or cylinder-band emitters
  (nor does the wasm d4 bin reach any of the four). The foils therefore
  carry no evidence for or against migrating those three; their evidence is
  the unit fixtures in `shadow/tests.rs`.
- On those unit fixtures, raised budgets give: disc with two rim chords,
  same three regions but the rim broken at different angles
  (`segmentation-only`); cylinder bands and sectors, same regions, wires,
  vertices and carriers, but `span-direction`: `arrangement_cyl` stores the
  `trim` of a reversed rim use in traversal order, where
  `OrientedPCurveEdge::trim` is documented native-direction and the legacy
  emitters keep it so. That convention has to be reconciled before a
  cylinder face is routed through the adapter. All of them decline on budget
  at default budgets.
- With non-default budgets the adapters report their fixed use caps
  (`MAX_CYL_USES`, 256) as `WorkBudgetExceeded` too, so "declined on budget"
  under raised budgets can also mean the structural cap. No foil face hit it.

## O2.3d budget policy: measurements and recommendation (2026-11-13)

Research for the maintainer decision that blocks O2.3d: what work budget the
arrangement adapters answer to. Measurement and design only: no dispatch,
budget or emitted geometry changed. **R** marks a measurement regenerated
after review (2026-11-14) on the full corpus with input fingerprints, or a
fixture rerun; **P** a measurement taken before
[O2.3d routing](#o23d-routing-is-a-constant-not-a-budget-2026-11-13) fixed
the defects it records, kept as that record; **I** an inference.

### Instrument

`face_splitter/shadow/work.rs` exists in test and `perf-counters` builds
only and is inert until `REMUS_SPLITTER_WORK_LOG` names a file. It probes
every face where the shadow differential runs (a legacy special case fired)
and every face that qualifies for one of the two production adapter sites,
the early cylinder arrangement and the last-resort planar arrangement in
`split_face_2d_impl`, whatever their route. Per face it:

- bisects the least `march_steps`, `queue_size` and `segments` that reach the
  raised-budget answer, each with the other two raised (the adapters are
  deterministic and every cap is monotone);
- runs each field alone at the operation's own value, the other two raised,
  to see which ones trip;
- records cumulative march steps per phase through `mark` calls in the
  planar collector and the core (`#[cfg]` no-ops in every other build);
- sizes the input (loop coedges, sections, collected uses) and the
  arrangement (vertices, edges, regions, vertex degrees);
- times the adapter at raised budgets and the legacy emitter it would
  replace, each the minimum of five runs;
- at the production sites, records whether the raised-budget result would
  pass that site's adoption gate.

No adapter run passes a split registry. Diagnostic reruns save and restore
all existing work counters (including ray and winding counters), also on
unwind. The logging-off path returns before this guard. The regression
`work_logging_preserves_output_and_perf_counters` checks the complete output
and every `PerfSnapshot` field with logging off and on; X stays at 64 probes
(previous logging-on result: 224), T at 79 in both modes.
`scripts/splitter-work-census.py` aggregates the logs. Unit fixtures:
`shadow/tests.rs::{work_of_the_minimal_fixtures,
reversed_rim_trims_misread_through_traversal_domain}`, plus the ignored
`work_of_a_section_grid`. End-to-end probe (ignored; run with `--ignored
--nocapture`): `crates/operations/tests/research_o23d_budget_equality.rs`.

Each record ends with an `input` fingerprint over the exact topology view,
face, ordered sections, rank, tolerances, context, adapter/frame and site.
The census folds rows only on site, test and that content identity: arena
face IDs restart with every `Topology`, so they never identify an input. It
retains every old-schema row (no fingerprint) with a warning.

Corpus (**R**): every lib and integration test binary of `remus-operations`
and `remus-io` that logged any probe record, re-run one binary at a time with
`REMUS_SPLITTER_SHADOW=raised` (221 binaries; the shared disk cannot hold every
test binary at once, and the other binaries log nothing). Base `39b0a7f5`
(after #1715's fingerprint and counter-restore fix, before #1719). 20,313
records, **15,145 distinct inputs** in 917 tests; no folded repeat measured
differently. The four legacy entry points fired 1,460 times. No test failed
in the 221 binaries. A second run of the same binaries before the
counter-restore fix gives the same measurements on 99.8% of records; the rest
come from 8 tests whose inputs differ between the two runs (property tests
with fresh random inputs, and thread tests whose code changed on `main` in
between). #1719 changes neither the adapters' charged work nor what the probe
measures (it only makes exhaustion typed, which the probe already forced); the
shadow agreement below is re-measured after #1719 on the 74 binaries that log
shadow events.

The first census (2026-11-13) folded rows on site, test name, arena face ID,
section count and use count, which merged distinct inputs. Its maxima stand;
its counts were low. Numbers that moved:

| Figure | First census | Regenerated |
|---|---:|---:|
| Smallest real split (march steps) | 394 | **376** |
| Faces over the `queue_size` default of 100 | 130 | **134** |
| Early cylinder site: faces reached | 1,035 | **2,409** |
| Early cylinder site: refusals at raised budgets | 17 | **29** |
| Last-resort plane site: faces reached | 7,742 | **11,771** |
| Last-resort plane site: refusals at raised budgets | 2 | **4** |
| Disc-by-chords faces | 6 | **24** |
| Sector faces | 69 | **111** |
| Band faces (cylinder) | 430 | **772** |
| Crossing-plane faces | 43 | **58** |

Unchanged: the largest split (89,955 steps), the largest work before a
decline (149,199), the p99 (7,452), the queue and segment maxima (309, 148),
every per-family march maximum, and with them the 2²⁰ recommendation's
margins (11.7× and 7×).

Fixture rerun (**R**):

```sh
cargo test -p remus-algo --lib --features perf-counters \
  shadow::tests::work_logging_preserves_output_and_perf_counters
python3 scripts/splitter-work-census.py target/diag/splitter-work-fixtures.tsv --faces
cargo test -p remus-algo --lib shadow::tests::work_of_the_minimal_fixtures -- --nocapture
cargo test -p remus-algo --lib shadow::tests::work_of_a_section_grid -- --ignored --nocapture
```

The log has 11 raw rows and 10 retained site/input rows. It covers four X
inputs (small first, centred, offset, large), T, disc chords, cylinder band
and sector, plus a repeated centred X. The four X inputs have equal local
IDs and counts; under the old key they retained one row and, with small
first, reported maximum 935. They now retain four rows, with steps
935 / 985 / 985 / 985 and maximum **985**; the exact repeat folds. The other
fixture steps are T 868, disc 652, band 654 and sector 459. The grid rerun
covers k = 1, 2, 3, 4, 6, 8, 9, 10, with k = 10 at 64 uses, 121 regions and
86,683 steps.

### 1. Why a two-section T exceeds 200 / 100 / 50

**R:** `march_steps` trips first, and on every probed face it is the only
field that trips, except on large planar faces: on 132 of them `queue_size`
trips as well (six also trip `segments`) and on two only `queue_size` trips.
The d4 foil faces 13 and 14 (4 loop coedges, 2 sections, 10 uses after
collection, 3 regions) need **838** march steps, queue 22 and segments 10.
The honeycomb foil faces 11 and 21 (11 loop coedges, 2 sections, 14 uses)
need 1,317 and 1,299 steps, queue 32 and segments 14. A unit square cut into
a T needs 868, an X 985; a cylinder band from two latitudes 654, a
single-ruling sector 459; a disc cut by two chords 652. The smallest real
split in the corpus needs 376.

`march_steps` is documented as "maximum marching steps per traced direction
of one intersection curve". The arrangement reads it as one face's total
charged work: the collector's concurrency and prejoin scans,
the degenerate-contact scan and the core's explicit step sites charge it.
This does not account for all collection or emission work. `queue_size`
caps the input uses and the event count;
`segments` caps the undirected edges. Cumulative march steps after each phase
of the T fixture (**R**; the corpus medians have the same shape):

| Phase | Work charged | Cumulative | Share |
|---|---|---:|---:|
| Collect: line concurrency | every use pair, plus crossing clusters | 15 | 2% |
| Collect: endpoint prejoin | every section end against every use | 39 | 3% |
| Degenerate-contact scan | every use pair, n(n-1)/2 | 84 | 5% |
| Validate, boundary graph, endpoint events | per use and loop vertex | 172 | 10% |
| Pair intersections | every use pair plus refinement | 421 | 29% |
| Cuts, vertices, junction certificates | per cut and incidence | 551 | 15% |
| Half-edges, rotation sort, successors | the sort is charged deg² per vertex | 633 | 9% |
| Regions | cycle trace, a clearance scan of every edge per cycle, windings, hole owners, material parity, components | 867 | 27% |
| Quotient | one step on a plane; 14-15% on a cylinder | 868 | 0% |

Three passes are quadratic in the use count (concurrency, scan, pairs) and
the region pass is cycles × edges. At n = 10 the three pair loops alone are
135 steps, so 200 cannot cover the smallest real split. Queue and segments
sit far below their caps on such faces.

### 2. Work across the corpus

**R.** Distinct inputs; "fires" is where the site was reached. Work columns
cover the faces the adapter splits under raised budgets.

| Site | Fires (tests) | Raised: split / declined / refused | March: max / p99 / median | Queue max | Segments max |
|---|---|---|---|---:|---:|
| `try_split_crossing_plane_face` | 58 (9) | 56 / 2 / 0 | 1,317 / 1,317 / 1,185 | 32 | 14 |
| `try_split_disk_by_chords` | 24 (6) | 24 / 0 / 0 | 2,004 / 2,004 / 2,004 | 38 | 16 |
| `split_periodic_face_into_sectors` | 111 (63) | 108 / 2 / 1 | 1,055 / 644 / 459 | 24 | 16 |
| `split_periodic_face_into_bands` (cylinder) | 772 (225) | 747 / 0 / 25 | 1,073 / 654 / 654 | 24 | 16 |
| Production early cylinder site | 2,409 (565) | 918 / 1,462 / 29 | 2,033 / 1,055 / 644 | 40 | 28 |
| Production last-resort plane site | 11,771 (606) | 10,579 / 1,188 / 4 | 89,955 / 7,452 / 870 | 309 | 148 |

At the operation's own budgets (`WorkBudgets::new()` on every probed face)
no site ever split: every face that the raised budgets split exhausted the
default budget, and the rest declined out-of-domain.

Shadow agreement (`REMUS_SPLITTER_SHADOW=raised`; every shadow event, not
deduplicated, because the step-0 shadow log carries no input fingerprint).
**R**, before #1719 (the 221-binary run) and after it (the 74 binaries that
log shadow events, on `1632392b`):

| Legacy entry point | Before #1719: agreed / disagreed / declined / refused | After #1719 |
|---|---|---|
| `try_split_crossing_plane_face` | 58 / 0 / 2 / 0 | 58 / 0 / 2 / 0 |
| `try_split_disk_by_chords` | 0 / 24 `segmentation-only(vertex)` / 0 / 0 | unchanged |
| `split_periodic_face_into_sectors` | 0 / 168 (96 `span-direction`, 72 `region-count`) / 2 / 1 | 96 / 68 `region-count` / 2 / 1 |
| `split_periodic_face_into_bands` (cylinder) | 0 / 1,176 `span-direction` / 0 / 29 | 316 / 860 `span-direction` / 0 / 29 |

(Refusals are `UnsupportedDomain` for the sector and 15 bands, `OpenRegion`
for 14 bands. The after-run has four fewer two-ruling sector events because
#1716 changed some of those inputs.)

So the disc and sector emitters, unlike in the foil corpus, do fire: disc
chords on 24 faces of `cylinder_box_{cut,fuse,intersect}_n{5,7}`, sectors in
63 tests and bands in 225. Before #1719 every disagreement was one of three
kinds: the `trim` convention (section 5), rim segmentation on disc chords, or
the sector region count. The region count is the two-ruling case (offset
parallel cylinders): legacy emits 3 regions, the adapter 2; with raised
budgets the same booleans come out with one face fewer at the same volume and
validate (section 6), so **I** the legacy sector emitter keeps a seam cut that
the adapter's quotient removes. After #1719 every sector trim agrees, but 860
band events (695 of them two-section, three-region bands, in bolt circles,
bore and rim resizes, gears and threads) still differ in trim direction while
the two-latitude unit fixture agrees. This run does not show which side
breaks the native-direction contract on those faces; that is open before the
cylinder flip.

### 3. Does the work scale with something cheap?

**R:** over every face the adapter splits, march steps against cheap sizes
(n = uses after collection):

| Predictor | Plane: march ÷ predictor, min / median / max | Cylinder |
|---|---|---|
| n² | 2.7 / 8.7 / 21.1 | 14.1 / 18.2 / 18.6 |
| (loop coedges + sections)² | 2.9 / 24.2 / 99.7 | 14.1 / 18.2 / 18.6 |
| (loop coedges + 3 × sections)² | 0.6 / 8.7 / 18.3 | 2.6 / 6.5 / 9.5 |

The synthetic `k × k` section grid on a square (`work_of_a_section_grid`)
shows a trend the corpus is too small to show: march ÷ n² is 9.9 at k = 1
(n = 10), 8.7 at k = 4, 12.1 at k = 6, 16.1 at k = 8 and 21.2 at k = 10
(n = 64, 121 regions, 86,683 steps). The region pass is cycles × edges, so a
dense arrangement grows towards n⁴, not n². The largest real face goes the
other way: 148 uses, 119 sections, 41 regions and 89,955 steps, only 4.1 n²,
because its sections are sparse.

The work is quadratic-dominated on real faces, but the constant spreads
six-fold between faces of the same size and climbs with density, and the best
cheap pre-collection predictor, (coedges + 3 × sections)², still spreads
thirty-fold. n itself is not known up front: it is the collector's output, and
the collector's pair scans are already charged. **I:** an input-scaled budget
is defensible only as a quadratic with a generous constant *plus* an absolute
ceiling for dense faces. The ceiling bounds charged work; the scaled part
only tightens the bound on small faces, which are cheap regardless
(section 4).

### 4. Cost, and what bounds a pathological face

**R (time per face, minimum of five runs, shared 12-core host):**

| Where | Adapter, median (max) | Legacy emitter, median (max) | Adapter ÷ legacy, median |
|---|---|---|---|
| `try_split_crossing_plane_face` | 118 µs (166 µs) | 6.8 µs (10.8 µs) | 16.9 |
| `try_split_disk_by_chords` | 250 µs (271 µs) | 225 µs (258 µs) | 1.1 |
| `split_periodic_face_into_sectors` | 54 µs (114 µs) | 4.3 µs (9.1 µs) | 12.0 |
| `split_periodic_face_into_bands` (cylinder) | 61 µs (147 µs) | 4.3 µs (7.5 µs) | 14.2 |
| Production last-resort plane site | 97 µs (11.3 ms) | — | — |
| Production early cylinder site | 60 µs (304 µs) | — | — |

The legacy figure is the emitter call alone; the wire building and
classification before it in `split_face_2d_impl` are not in it.

**R:** one march step costs a median 115 ns on planar faces and 116 ns on
cylinder faces (p99 146 and 152 ns; the slowest single reading is 321 ns).
The largest face, 89,955 steps, takes 11.3 ms (126 ns per step). The densest grid costs 273 ns per step (86,683 steps
in 23.7 ms).

The step counter bounds only charged work, and cancellation is polled at
explicit check sites. Not every inner loop charges or polls: in particular,
`Collector::push_loop` resolves and collects an entire loop's coedges without
either. The caller polls before each loop and checks the 256-use cap only
after `push_loop` returns, so an arbitrarily large loop can already have
been walked and allocated before rejection. Other collection and emission
work is uncharged too. The structural caps (`MAX_ARRANGEMENT_USES`,
`MAX_CYL_USES`, 256) limit accepted collected inputs, not all work preceding
those checks. Line-concurrency clustering is crossings × clusters and the
region pass cycles × edges; a dense accepted 256-use face could need
10⁷–10⁸ charged steps (**I**, extrapolated from the grid).

**I:** B × 300 ns is an extrapolation for the charged portion only, using the
regenerated per-step timings. At 2,000,000 steps this gives about 0.6 s;
1,048,576 gives about 0.12 s at the measured median and 0.3 s at 300 ns.
These figures **exclude uncharged collection and emission** and are neither
an end-to-end time bound nor a cancellation-latency guarantee. Follow-up:
charge/poll the coedge collection loop and enforce predictable caps before
collection; then measure total refusal time. This measurement/docs PR does
not change budget, cancellation or dispatch behavior.

**R:** the adapter can spend a large budget only to decline. Five io faces
with 38–242 sections (Gridfinity sockets and the mitsukude panel) spend
13,430–149,199 steps and then return out-of-domain. **I:** for the
242-section face, its 246 input uses can only pass the 256-use cap
(`MAX_ARRANGEMENT_USES`) when the concurrency pass splits them, and the
collector tests the cap only after that pass and the prejoin; the cheap
out-of-domain gates should run before the quadratic passes.

### 5. The cylinder `trim` direction

**P:** the disagreement was real and was an emitter bug, not a comparator
artefact. `arrangement_cyl::emit_cylinder_run` stores
`trim = Some((source_span[0], source_span[1]))` in traversal order with
`forward = source_span[1] >= source_span[0]`. `OrientedPCurveEdge::trim` is
documented native-direction, and `traversal_domain()` swaps it for a reversed
use, so a reversed arc is swapped twice. Read through the documented
accessors (`reversed_rim_trims_misread_through_traversal_domain`), the
adapter's output on the shadow fixtures is:

- band fixture: every reversed closed rim (trim `(4.712, -1.571)`,
  `forward = false`) has the right endpoints, but `traversal_domain()` walks
  it the wrong way round;
- sector fixture: both reversed open rim arcs have a `traversal_domain()` that
  starts at the arc's end point, 3.37 from the true start on radius 2.

The legacy emitters' output passes the same check on the same faces. In the
corpus the shadow labelled all 1,176 band events and 96 sector events
`span-direction` before the fix; after it, the 96 sector events and 316 band
events agree and 860 band events still differ (section 2).

Consumers of `trim` on splitter output, and what they make of the adapter's
convention:

- `OrientedPCurveEdge::{native_domain, traversal_domain, sub_trim,
  sub_trim_fraction, sub_traversal_domain_fraction}` in
  `builder/split_types.rs`, and through them about 55 call sites in
  `face_splitter/mod.rs`, `special_cases.rs`, `edge_splitting.rs`,
  `containment.rs`, `sampling.rs` and `conversion.rs`. **P:** misread, as
  above.
- `fill_images_faces::preflight_wire_edge` and `instantiate_wire_edge`, which
  turn each sub-face wire edge into a topology `Edge`. **I:** an open arc is
  repaired, because the endpoints are matched against the trim in either
  order and `Edge` accepts a descending trim; a closed rim keeps the
  traversal-order trim and takes `forward` from the pcurve edge, so the stored
  edge is walked against the region's sense.
- **P**, end to end: three booleans routed through the cylinder adapter by
  raised budgets (offset-cylinder intersect and fuse, a coaxial magnet and
  screw fuse) validate and keep the exact volume. The misread did not surface
  there; it is latent for any consumer that reads `traversal_domain()` or
  `sub_trim()` of an adapter edge.
- The adapter's own differential tests (`arrangement_cyl::tests`) sample arcs
  from `start_3d`, `end_3d` and `forward`, never from `trim`, which is why
  they did not see it.

Fix (landed with
[O2.3d routing](#o23d-routing-is-a-constant-not-a-budget-2026-11-13), which
lists the pins as they now stand): in `emit_cylinder_run`, store the native
interval,
`Some(if forward { (s0, s1) } else { (s1, s0) })`, keeping `forward`, and make
the endpoint check beside it evaluate the traversal order. That matches the
legacy emitters and `arrangement_prod`. Pins: the two shadow fixtures turn
from `span-direction` into agreements (their doc comment already says so),
`reversed_rim_trims_misread_through_traversal_domain` inverts to "no
misreads", and `arrangement_cyl`'s differential sampling moves to
`traversal_domain()`.

### 6. A production defect: the budget-equality gate

**Status: fixed** by
[O2.3d routing](#o23d-routing-is-a-constant-not-a-budget-2026-11-13); this
section records the defect as measured before the fix.

The adapters treated budget exhaustion as a silent decline only when
`context.budgets == WorkBudgets::new()`. Any other budget value, including
one unrelated to face splitting, turned the same exhaustion into a typed
`FaceSplitFailed` that propagated out of `split_face_2d_impl`; and a budget
that happened to be large let the adapters claim faces ahead of the legacy
emitters.

**P** (`research_o23d_budget_equality.rs`; default fallback policy, then
`ExactOnly`):

| Boolean | Default budgets | `newton_iterations` 20 → 21 | Raised march / queue / segments |
|---|---|---|---|
| Offset cylinders, intersect | exact, 5 faces, V = 979.92 | **mesh fallback, 41 faces, V = 975.81**; `ExactOnly`: error | exact, **4 faces**, V = 979.92 |
| Offset cylinders, fuse | exact, 9 faces, V = 2161.67 | **mesh fallback, 57 faces, V = 2155.58**; `ExactOnly`: error | exact, **8 faces**, V = 2161.67 |
| Coaxial magnet + screw, fuse | exact, 6 faces, V = 47.752 | **mesh fallback, 50 faces, V = 47.255**; `ExactOnly`: error | exact, 6 faces, V = 47.752 |

All results validate. The splitter's debug log names the cause:
`provenance arrangement refused face=Id(2) err=WorkBudgetExceeded uses=6` at
the last-resort planar site. Production callers reached this before the fix: WASM
`booleanWithQuality`, `booleanWithCancellation` and the batch `marchSteps`
option build their context with `quality_context`, which sets exactly these
fields. **I:** any budget option on those calls, even a larger Newton or
subdivision cap, can turn an exact boolean into a mesh fallback, or an
`ExactOnly` boolean into an error, on any model whose faces reach either
adapter site; raising `march_steps` for the marcher's sake instead reroutes
cylinder faces through the adapter (5 → 4 and 9 → 8 faces above, same
volume). Four box/cylinder shapes that take the boolean fast paths were
unaffected.

Across the corpus every probed face ran under the default context, and at
that context neither production site ever produced a split; the adapter
exhausted the budget or declined on every face (**R**). At default budgets
both production call sites were dead code, and their only live effect was the
defect above.

### 7. Recommendation

None of the three listed options as stated. Reusing `march_steps = 200`
cannot cover the smallest real face (the corpus minimum is 376). Deleting the
default-budget decline on its own turns every face that reaches an adapter
site into a typed error at 200 steps. An input-scaled budget is principled
only with an absolute ceiling (section 3), and the ceiling is what bounds the
charged work (subject to the collection exclusions in section 4).

Recommended, in three separable steps:

1. **Decouple routing from budgets now** (done). This preserves behaviour at
   defaults and fixes section 6. The adapters stop reading
   `budgets == WorkBudgets::new()`. Whether a face family goes to the
   arrangement becomes an explicit per-family routing constant in
   `face_splitter` (legacy for every family today), not an option on
   `OperationContext`. While a family is routed to legacy, its production
   site skips the adapter before collecting. Production change: none at
   default budgets (**R**: the sites never split at defaults on the corpus);
   with any non-default budget, the three probed booleans return to their
   exact default results instead of a mesh fallback or an `ExactOnly` error.
   Pin: turn `research_o23d_budget_equality` into an assertion that
   `newton+1` and raised SSI budgets give the same topology as the default.
   It failed on `main` before the fix; it is now
   `regress_arrangement_routing_budgets.rs` (done, see the routing section
   below).
2. **Give the arrangement its own work field.** Add
   `WorkBudgets::arrangement_steps` (the struct is `#[non_exhaustive]`, so
   this is additive) with default **1,048,576** (2²⁰). That is 11.7× the
   largest face the corpus splits (89,955 steps), 7× the most the adapter
   spends on any face before declining (149,199), 12× the densest synthetic
   face (k = 10 grid, 121 regions, 86,683 steps) and 516× the largest
   cylinder face (2,033). The regenerated census leaves these
   margins unchanged (**R**). Its charged portion extrapolates to about
   0.12 s at the measured median cost per step and 0.3 s at 300 ns, excluding uncharged
   collection and emission (section 4). This is not a per-face time bound.
   The 262,144 one might pick from the ops corpus alone (7,452 max) would
   leave under 3× headroom on the io panel face. The collector and the core
   would charge only this field. Drop the arrangement's reads of `queue_size`
   and `segments`: every core event and edge already charges a step, so that
   bounds core allocation, but not the uncharged collection described in
   section 4. 134 corpus faces exceed the SSI `queue_size`
   default of 100 (up to 309 events) and six the `segments` default of 50 (up to 148 edges) for no
   reason of their own. Run the collector's cheap out-of-domain gates, and the
   256-use cap where it can be predicted, before its quadratic passes
   (section 4). Keep the 256-use structural caps but
   report them as their own refusal, not as `WorkBudgetExceeded`. Exhaustion
   is then always a typed refusal; no equality-based silent decline remains.
   Do not expose the field through WASM until a consumer needs it.
   Production change: none while routing stays legacy. Pins: the minimal
   fixtures and the foil faces fit the default with at least 16× headroom (a
   `perf-counters` guard driven by the work probe); a grid past the budget
   refuses typed, with total refusal time measured separately; the existing
   budget-prefix and cancellation atomicity tests move to the new field.
3. **Flip routing per family as each legacy emitter is deleted**, as the
   O2.3d plan already orders. The evidence below says what each flip changes.

Per family, at a sufficient budget (**R**):

- **Plane crossing (`try_split_crossing_plane_face`)**: 58 faces in 9 tests
  (d4, honeycomb pcut3, the lip-fuse halfsockets, the ray-cast vote
  workloads, a boss chamfer); every one the adapter splits agrees (58 shadow
  events) and two decline as out-of-domain. Ready for the first migration
  once step 2 lands.
- **Disc by chords (`try_split_disk_by_chords`)**: fires outside the foils
  (24 faces, `cylinder_box_{cut,fuse,intersect}_n{5,7}`); every one is
  `segmentation-only(vertex)`: the same three regions with the rim broken at
  other angles. Whether that segmentation is acceptable output has to be
  decided before the deletion.
- **Last-resort planar site**: of the 11,771 faces it reaches, the adapter
  would pass the adoption gate on one
  (`push_pull::tests::widening_a_bore_into_other_geometry_fails_closed`,
  6 regions against fewer greedy loops) and refuse four
  (`through_tool_booleans_preserve_material_from_1e_minus_5_to_1e6`,
  `OpenRegion`). Routing it on would change that one output and fail that
  test unless the refusals are triaged first.
- **Cylinder bands and sectors, and the early cylinder site**: 918 of the
  2,409 faces reaching the early site would be adopted, which changes output
  on all of them. After #1719's trim fix, 860 band events still differ in
  `trim` direction (section 2) and 68 two-ruling sector events become 2
  regions instead of 3. 29 faces refuse (`UnsupportedDomain` 15,
  `OpenRegion` 14, in bolt-circle cuts, coaxial booleans, a seam-placement
  test, a rotated plate, a rigidly placed bore resize and
  `f12_cut_large_placed`); at that site a refusal is a typed boolean failure.
  The remaining band trim difference and a triage of those refusals both have
  to be resolved before the cylinder flip.

Risks: per face the adapter is 12–17× slower than the band, sector and
crossing emitters (tens of microseconds; 1.1× for disc chords), so each flip
needs the criterion benches. A routing constant is one more dispatch decision
the ratchet should count. The numbers come from the in-repo corpus, whose
largest split face has 148 uses; a real model with a denser face meets the
typed refusal at 1,048,576 steps, which is the intended outcome once its
legacy path is gone, and the step-2 guard should be re-measured as the corpus
grows. An input-scaled formula remains a later option (for example
32 n² + 4,096 under the same ceiling) if per-face bounds on small faces ever
matter; nothing measured here needs it.

## O2.3d routing is a constant, not a budget (2026-11-13)

Step 1 of the recommendation above, and the fix for the defect in
[section 6](#6-a-production-defect-the-budget-equality-gate) and the `trim`
direction in [section 5](#5-the-cylinder-trim-direction). Before it, both
adapters declined silently on budget exhaustion only while
`context.budgets == WorkBudgets::new()`. Rechecked on `main` 877d8908 by the
regression test below: with `newton_iterations` 21 the three cylinder
booleans of section 6 went to mesh fallbacks (41, 57 and 50 faces), or
errors under `ExactOnly`, and with raised march, queue and segment budgets
the cylinder adapter claimed the offset-cylinder faces (4 and 8 faces).

Now `face_splitter/mod.rs` routes each family with a production adapter site
through a constant, `CYLINDER_LATERAL_ROUTE` (early cylinder site, the faces
the band and sector emitters take) and `PLANAR_LAST_RESORT_ROUTE`
(last-resort planar site), both `ArrangementRoute::Legacy`. A legacy-routed
site skips the adapter before collecting, whatever the budgets. The
crossing-plane and disc-by-chords emitters have no production adapter site,
so they get a route only when a migration adds one. Inside the adapters,
exhaustion is now always a typed `WorkBudgetExceeded` refusal, which is also
what the shadow differential reads. Default-budget behaviour is unchanged;
non-default budgets now give the default-budget result
(`crates/operations/tests/regress_arrangement_routing_budgets.rs`, the
assertion step 1 asked for). The work probe (`shadow::work`) still measures
every face that qualifies for either production site, whatever the route,
and no longer needs to perturb default budgets to see typed exhaustion. The
ignored probe `research_o23d_budget_equality.rs` stays as the record of the
measurement; after the fix its three budget columns agree.

`emit_cylinder_run` now stores the native interval
(`if forward { (s0, s1) } else { (s1, s0) }`) and checks the carrier
endpoints in traversal order. Pins: `arrangement_cyl`'s
`circle_trims_are_native_and_sample_the_traversed_arc` samples every arc
from `traversal_domain()`; the band and sector shadow fixtures agree with
the legacy emitters; `reversed_rim_trims_misread_through_traversal_domain`
asserts no misreads.

Deferred: steps 2 and 3, a dedicated arrangement work budget and flipping
any family to the arrangement. A flipped family would refuse on the default
`march_steps` of 200, so it needs its own budget first.

## Non-goals

- No new analytic intersection solver or surface/curve variant.
- No rewrite of GFA, pave filling, same-domain merging, or classification.
- No universal proximity merge key and no global tolerance-grid snapping.
- No O2.3b-d source change before P-Class 2.4 releases the splitter files.
- No claim that a validation-only or face-count-only comparison is a geometry
  oracle.
