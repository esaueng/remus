# F6 spherical cut qualification

The literal operation sequence creates a unit cube at the origin, then a unit
sphere, and cuts the cube from the sphere with `ExactOnly`. The expected solid
contains seven spherical octants. Its volume is `7π/6`; its exterior area is
`7π/2 + 3π/4 = 17π/4`, including the three planar quarter-disc wounds.

## Baseline and defect

The baseline is source `32d3994860fab2de13d6c9f3caaab836bab40db2`, including
the F3, F5/F12 and nonplanar boundary-clearance repairs. The native literal
regression failed with 14 raw boundary edges at linear deflection 0.1.
The independently rebuilt baseline WASM pair failed the outward/material
triangle oracle at deflection 0.1 and default angular tolerance 0.35. The
retained result's approximate volume, 3.671139, was already incorrect; agreement
between quantity routes sharing a tessellation fallback did not prove correctness.

Four linked problems required repairs:

1. Sphere hemispheres shared straight equator chords, whose interiors leave the
   spherical carrier. The primitive now partitions an exact circle with explicit
   native arc domains while retaining its boundary vertex and edge counts.
2. The no-seam splitter recognized only all-line equators. A narrow adapter now
   accepts a qualified complete circular equator partition and uses the existing
   cap/remainder construction. It rejects incomplete, mixed-frame, retraced,
   malformed and multiple-turn partitions. Circular rim coverage checks the
   entire mapped native interval, including reversed and phase-shifted circle
   frames. Unresolved overlaps refuse the new adapter instead of producing a
   touching hole. A positive-weight NURBS section may be excluded only when
   its whole control hull and authoritative anchors are strictly separated
   from the rim plane. Crossing, touching, coincident and nonpositive-weight
   controls retain an unresolved overlap; this preserves the existing
   sphere–cylinder cells without claiming general NURBS overlap support.
   Native circular sections can likewise be excluded by outward whole-span
   height bounds against the rim plane. Intersections of their complete
   carriers do not imply overlap of the retained arcs.
3. Spherical meshing chose the first great-circle edge's frame. A meridian
   frame can cut across both sides of the retained concave patch. Qualified
   native great-circle loops now try the stored frame and eligible support
   frames, bounding the entire native arcs before choosing a hemisphere.
   A directed vector-area bound also verifies that the loop selects that
   hemisphere's region, including concave boundaries and decreasing domains.
   Declined mesh attempts restore appended vertices, normals, indices and map
   entries. Other carrier families retain their existing dispatch.
4. Analytic trim classification supported two central-circle constraints,
   whereas this concave patch has three support chains. Its sampled fallback
   also misclassified boundary points on the removed spherical carrier.
   A qualified full-hemisphere or simple three-chain great-circle predicate
   now uses outward interval signs and oriented tangent-plane winding for both
   ray and boundary queries. It retains major arcs and respects a finer caller
   tolerance. Unsupported or unresolved trims keep the existing path.

The same trim predicate serves one-shot, prepared and persistent-cache point
classification. Native material-grid and boundary tests compare all three at
both the default and WASM query tolerances; packaged tests exercise the cache
used by the JavaScript front door.

The translated radius-1000 control also has a stored section-circle center
offset of about `1.82e-12`. A whole-carrier and endpoint-connector error bound
can qualify this trim within the existing `1e-10` half-space band. Queries
must be separated from every deformation tube before using ideal winding;
boundary decisions subtract that error from the caller's band. This does not
widen the tolerance for a finer query. Unresolved queries on a qualified
perturbed support refuse instead of returning to the sampled trim predicate.

Scale controls also exposed sampled tangent searches creating phantom pave
vertices near an existing endpoint. Qualified actual Circle/Plane and Line/Sphere
pairs now solve their algebraic events with outward root-image bounds. Native
points must fit within a reserved eighth of the existing linear tolerance;
exact endpoint dependencies preserve tangent ownership. Unknown cases retain
the existing solver. Endpoint windows, modeling tolerances and NURBS dispatch
are unchanged.

Existing complete-sphere containment and distance certificates now also qualify
the exact circular equator, preserving their legacy line-ring checks. A bounded
native latitude-domain adapter integrates complete circular caps and bands
without sampling their rims. This keeps the existing 32-segment cap cut within
the integration resource budget and preserves its closed-form quantities;
unsupported domains retain the existing path and budget. These adapters run
only when the historical sample-count check would reject the face, preserving
under-budget quadrature arithmetic. Other qualified fragmented equator runs
coalesce into their entire native arc before sampling, including major arcs.
Shared vertex identity, identical stored carriers, native continuity and
consistent traversal must hold. The aggregate input-edge and generated-point
limits remain 4096; no topology or resource limit changes.
Complete-sphere certificate controls preserve the existing conservative refusal
for a combined large placement in both Circle and legacy Line representations.
The transform writer's axis cancellation is outside this repair; positive
certificate controls also exercise separate rotation and translation. F6's
material and mesh matrix still uses the original combined placement.

Exact equator partitions also qualify directional projection's existing
hemisphere rim adapter. The partition's native direction determines the rim
orientation; its support frame may be any central great circle.

Affine sphere transforms retain exact boundary arcs when the mapped conic's
axes become skewed: a bounded rational arc replaces that edge and receives its
own native domain. This requires compatible exact NURBS plans for every
incident face, complete coedge authority and no stored pcurve or periodic lift.
In-place and copy paths share pure preparation before mutation; reports disclose
the carrier change and new control points. Generic conic refusals remain.

Sphere and affine-ellipsoid mesh counts change with the exact boundary.
Their updated snapshots are guarded by independent implicit-carrier residuals,
raw opposing edge incidence, outward triangles, no unused vertices, genus-zero
Euler counts and signed mesh volumes. Analytic quantity and bounding-box
expectations remain unchanged. The previous primitive sphere meshes were
already closed over their used vertices; the repair does not claim otherwise.
The drilled-sphere quantity snapshot has last-bit changes in its lower
hemisphere and assembled totals. Replacing only the equator carriers with
legacy Lines reproduces the entire prior snapshot. Exact bit assertions remain,
alongside independent napkin-ring volume, area and per-half flux oracles.

## Regression contracts

`crates/operations/tests/regress_opseq_sphere_box_cut.rs` and
`scripts/opseq-f6-packaged.mjs` use independent Cartesian inequalities for
source and result material membership, closed-form quantities, opposing
coedges, strict validation, raw mesh incidence and unchanged serialized operands.
Mesh checks cover linear deflections 0.1, 0.01 and 0.001 relative to the radius
with angular tolerance 0.35; the packaged contract also checks angular 0.1.
The WASM contract runs through direct calls, `executeBatch` and `executeBatchV2`
in fresh and installed packages.
Cut controls cover radii 0.001, 1 and 1000 at the origin and under a rigid
rotation and translation, plus independent segment counts and a larger cutter.
The packaged contracts also retain the radius-10 sphere cut above `z=5`, with
three analytic faces, `V=1000×9π/8`, `A=100×15π/4` and four raw mesh densities.

The related F8 union now passes the same material, topology and raw mesh
contracts in both operand orders, with its literal sphere-first creation order.
Independent formulas are `V = 1 + 7π/6` and `A = 6 + 11π/4`: the exposed
sphere contributes `7π/2`, the cube's three outer faces contribute 3, and its
three remaining face regions contribute `3 - 3π/4`. The fresh source32 baseline
fails the triangle/material oracle on the literal F8 fixture.

Private tests pin whole-carrier support, native arc domains, wire-start and
frame independence, rollback, conservative classifier arithmetic, decreasing
domains, cyclic support-chain coalescing and unsupported-carrier controls.

The broad X5 and X7 generated families remain excluded until separately qualified.
General spherical arrangements, arbitrary NURBS trims and the other retained
sequence findings are outside this bounded repair.
