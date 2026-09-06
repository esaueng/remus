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

## Changed bands and refinement

| Location | Correction | Boundary of this change |
| --- | --- | --- |
| `phase_ff.rs`, boundary-junction search | Exact closest-point projection on a straight edge replaces fixed-count ternary refinement | Curved boundary refinement keeps its existing algorithm and budgets |
| `phase_ff.rs`, junction reuse | Cap `100 × linear tolerance` at 1% of the combined face-pair extent, with the caller's linear tolerance as the floor | Same extent authority as the previously capped boundary-search trigger |
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

- Curved edge-parameter comparisons that currently reuse a length tolerance.
  Straight-edge endpoint exclusion now divides by edge length and its duplicate
  anchors are compared only in 3D. A regression covers edge lengths 1e-4, 1,
  and 1e6, rejecting sub-tolerance endpoint fragments while retaining distinct
  interior anchors near the endpoints and near one another.
- Curved and periodic face bands, including the cases where endpoint bounds
  do not represent the carrier's extent.
- Anisotropic models and small features embedded in much larger faces.
  A packaged-WASM probe of a 1e6 by 1 by 1 blank and a 0.1 by 0.4 by 2
  tool at (0.1, 0.3, -0.5) still returns an incorrect exact Fuse: volume
  1e6 rather than 1e6 + 0.04, and the tool-only point (0.15, 0.5, -0.25)
  is outside the result. The same defect reproduces on the package preceding
  the straight-edge parameter correction. Cut refuses typed; Intersect
  preserves the expected 0.04 volume. This material-loss witness is open.
- The remaining raw absolute snap/weld/acceptance constants in the other
  pave-filler, arrangement, classifier, and assembly paths.

P-Class 2.6 remains partial until these bands have dimensional justifications
and corresponding geometry witnesses. Tangent and sliver-contact construction
remains owned by 2.7.
