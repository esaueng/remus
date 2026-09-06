# Scale-band audit

Status: partial, 2026-09-06. The through-tool family below is qualified;
this is not a claim that every absolute band in `crates/algo` is audited.

## Qualified family

A cubic blank of side `s` and a `0.4s × 0.4s × 2s` tool translated by
`(0.3s, 0.3s, -0.5s)` have normalized volumes 1.16 (Fuse), 0.84 (Cut),
and 0.16 (Intersect). `qualify_boolean_scale.rs` checks all three operators
at every decade from `1e-5` through `1e6`, both unplaced and with a common
Y rotation of 0.37 radians followed by `(17s, -23s, 31s)` translation.

All 72 cells require exact-only success, valid topology, watertight mesh,
and measured and mesh volumes within 1e-6 relative error of those independent
box-volume formulas. The corresponding native batch matrix checks exact
quality, validation, and volume. The direct packaged-WASM matrix runs in
both the smoke and installed-tarball consumer suites.

Before the correction, the 72-cell regression failed ten cells at the two
ends of the scale range. At `1e6`, raw GFA Cut omitted the four hole walls,
had eight free edges, and measured 0.946666667 rather than 0.84. Its junctions
were displaced approximately 1.76e-5 from their supporting planes. Public
acceptance refused this result. At `1e-5`, successive broad weld and closure
bands collapsed separate junctions, rejected an internal footprint, and
mistook individual short edges for closed loops.

## Anisotropic through-tool qualification

A blank of length 1, 1e3, or 1e6 and unit width/height is crossed by a tool
of width 0.1 or 0.001, height 0.4, and depth 2 at (width, 0.3, -0.5).
The 36-cell native matrix covers all three booleans, unplaced and under a
common Y rotation of 0.37 and translation (17, -23, 31). It requires exact
success, valid topology, a watertight mesh, and material classification at
three points along the tool. A local box intersection additionally measures
the tool region against independent box-volume formulas within 1e-5 of the
overlap volume. Packaged direct and batch APIs carry the same material checks.

Before correction, raw clipping dropped real short sections because their
fractions of a long carrier were below 1e-6. Fuse discarded the resulting
open tool fragments and returned the untouched blank as exact. The acceptance
bounds margin, derived only from the long result diagonal, hid the missing
protrusions. Clipping now uses the caller's linear tolerance in model space;
Fuse acceptance caps its margin by the checked operand's diagonal as well.

World-coordinate volume on a rotated long body has a separate precision
limit: the untouched 1e6 blank measured 999999.9995026788, while its Fuse
measured 1000000.0399823142 rather than 1000000.04. Origin-shifted mesh
summation reduces but does not eliminate the error. The active material
matrix keeps a 1e-9 whole-volume relative check (or the small-feature absolute
budget, whichever is larger) for placed bodies, plus the stricter independent
local-volume check. Unplaced world volumes retain the 1e-5 overlap-relative
budget. The ignored, runnable `anisotropic_world_volume_resolves_small_feature_scale`
test preserves the unresolved stricter world-volume target; measurement
precision is not claimed fixed by this qualification.

## Changed bands and refinement

| Location | Correction | Boundary of this change |
| --- | --- | --- |
| `phase_ff.rs`, plane-line clipping | Convert interval acceptance and deduplication to model-space length using the caller tolerance | Angular and other remaining polygon constants are not fully audited |
| `boolean/mod.rs`, Fuse acceptance | Cap bounds margin by both result and checked-operand size | Bounding-box containment remains a necessary, not sufficient, material test |
| `phase_ff.rs`, boundary-junction search | Exact closest-point projection on a straight edge replaces fixed-count ternary refinement | Curved boundary refinement keeps its existing algorithm and budgets |
| `phase_ff.rs`, junction reuse | Cap `100 × linear tolerance` at 1% of the combined face-pair extent, with the caller's linear tolerance as the floor | Same extent authority as the previously capped boundary-search trigger |
| `face_splitter/edge_splitting.rs`, analytic arc anchors | Circle and ellipse endpoint exclusion and duplicate anchors use 3D distance; strict fraction bounds retain arc membership | NURBS projection and endpoint checks remain unaudited; boundary traversal conventions are unchanged |
| `face_splitter/edge_splitting.rs`, line anchors | Limit anchor acceptance and spatial dedup by 1% of the edge length | Endpoint exclusion converts linear tolerance to normalized parameter units; deduplication uses 3D distance |
| `face_splitter/mod.rs`, planar internal loops | Limit endpoint quantization and interior margin by polygon extent | Curved endpoints alone cannot bound a carrier and retain their existing band |
| `face_splitter/mod.rs`, planar arrangement | Limit endpoint adoption and coarse snapping by the arrangement extent | The new extent cap applies to straight inputs |
| `face_splitter/mod.rs`, planar sections | Limit endpoint welding, bridging, and zero-extent filtering by the boundary extent | Non-planar chart handling is unchanged |
| `face_splitter/special_cases.rs`, loop chaining | Use the local planar boundary band for closure and continuation | Curved carriers retain their existing closure band |

The caller's `OperationContext` tolerance is unchanged. The historical
allowance is retained when it is below the local extent cap; the cap never
reduces the band below the caller's linear tolerance. The existing `1e-6`
through-cut remains a typed exact-only refusal. The former large-scale
rollback witness in `boolean_context_authority.rs` now requires the correct
result and unchanged operand geometry; the other refusal/rollback tests remain.

## Remaining audit

- NURBS edge-parameter comparisons and projection acceptance. The circle and
  ellipse boundary/section finders now use model-space endpoint and duplicate
  distances. Their regression covers radii 1e-4, 1, and 1e6; section twins run
  in both directions and must preserve every distinct geometric anchor,
  merge near-coincident anchors, and reject sub-tolerance endpoint fragments.
  Existing closed-rim and other-window tests preserve the traversal contract.
  Straight-edge endpoint exclusion now divides by edge length and its duplicate
  anchors are compared only in 3D. A regression covers edge lengths 1e-4, 1,
  and 1e6, rejecting sub-tolerance endpoint fragments while retaining distinct
  interior anchors near the endpoints and near one another. The existing
  tangent-boss Fuse/Cut matrix now also passes exactly at 1e3 scale; its
  former refusal expectation is replaced by the same watertightness, analytic
  surface, and 1e-9 closed-form volume oracles used at smaller scales.
- Curved and periodic face bands, including the cases where endpoint bounds
  do not represent the carrier's extent.
- Further anisotropic and embedded-feature families beyond the declared box/tool
  matrix, plus world-volume precision on rotated long bodies as described above.
- The remaining raw absolute snap/weld/acceptance constants in the other
  pave-filler, arrangement, classifier, and assembly paths.

P-Class 2.6 remains partial until these bands have dimensional justifications
and corresponding geometry witnesses. Tangent and sliver-contact construction
remains owned by 2.7.
