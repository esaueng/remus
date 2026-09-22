# Exact blend reconstruction

This delivery belongs to P-Class 6.2, 6.3 and 6.5 in the
[authoritative roadmap](../kernel-maturity/roadmap.md). It broadens imported
blend removal through bounded geometric families. It does not complete general
curved blends, arbitrary NURBS intersection, or every direct-edit operation.

## Construction contract

Recognize the selected band and its incident supports; prove the affected trim
neighborhood; construct sharp intersections on the original carriers; rebuild
oriented wires and derived UV data; validate the solid and complete construction
history; publish atomically. Unsupported or ambiguous arrangements refuse with
the input arena and journal restored. Modeling tolerance is not enlarged to turn
an uncertain classification into an exact result.

The reusable pieces are:

- Split spring-chain proof in `crates/operations/src/resize_blend.rs`. Connected,
  monotone, collinear contact edges may collapse to one sharp edge. Backtracking,
  off-carrier chains and unqualified role-vertex splits refuse.
- Atomic connected-group removal in `crates/operations/src/remove_blends.rs`.
  Explicit seeds select bands; same-carrier patches expand together; spherical
  corners join only when every incident band is selected. Reconstruction acts
  on the union rather than a sequence of partially healed solids.
- Spherical-end reconstruction in `crates/operations/src/local_wound.rs` for
  one cylindrical strip between two planar supports, with a planar end and a
  spherical end. Sharp roots require cap-hemisphere, wound-direction and local
  trim evidence, followed by strict validation of the reconstructed solid.
- Bounded affine NURBS qualification in
  `crates/geometry/src/convert/certified_plane.rs`. Degree-one, clamped 2x2,
  equal-weight control nets must have an exactly zero mixed coefficient.
  `crates/operations/src/affine_blend_caps.rs` uses that certificate for end
  caps while restoring the original NURBS carrier and parameter domain.
- Closed-form analytic section qualification in
  `crates/math/src/analytic_intersection.rs` and `intersect.rs`. Lower-level
  section support is distinct from qualification of an entire blend operation.
- Strict trimming-curve certificates in `crates/topology/src/validation.rs`.
  An exact 3D carrier and a numerically derived UV curve are different claims:
  a UV curve must satisfy a conservative whole-interval residual bound at the
  existing modeling tolerance. Interpolation-node agreement alone is not proof.
- `remove_blends_journaled` and WASM `removeBlendsJournaled`, including
  `executeBatchV2` dispatch, carry construction-derived face, edge and vertex
  evolution. Deleted entities remain explicit; correspondence is never inferred
  from nearest geometry.

## Evidence and boundaries

| Family | Regression evidence | Remaining boundary |
| --- | --- | --- |
| Split contact chains | `regress_blend_split_boundaries`: remove, resize, planar move, transformed scales, STEP, journal census and rollback | Split retained generatrices requiring a new vertex-star proof |
| Connected group | `regress_remove_blends`: convex/concave bands, complete trihedral corner, seed determinism, STEP, journal resolution | Partial corners, mixed radii, disconnected groups and unsupported unions |
| Existing cylindrical terminations | `regress_towel_rack_r1_removal`: planar and compound curved endpoints | Geometry outside its declared compound family |
| Spherical end | `local_wound::tests`: public and journaled removal, scale, rigid placement and chart seam, independent volume, vertex disks, STEP and rollback | One isolated strip with one planar end; ambiguous or tangent roots and other endpoint topology |
| Affine NURBS cap | `affine_blend_caps::tests`: public and journaled removal, retained carrier/domain, STEP, history and atomic refusal | Bounded degree-one affine cap with straight retained boundaries; no general freeform healing |
| Affine NURBS certificate | `certified_plane` unit tests: original UV round trip, scale, translated twist, in-plane mixed coefficient, cancellation and weight refusal | General rational, higher-degree, curved or nearly affine patches |
| Analytic sections | `analytic_special_sections`; existing hammer and tangency regressions preserve the legacy Boolean path | General torus/cone cuts and uncertain special-case classification; full exact torus circles are exposed through the qualified API only |
| Public journal/WASM contract | `remove_blends_has_direct_batch_parity_total_history_and_rollback`; `scripts/test-wasm-smoke.mjs` | No automatic selection of unseeded sibling bands |

The kernel API is independently usable. A consumer must adopt the regenerated
package pair and preserve construction history across replay; updating only
source code does not change an already pinned browser kernel. Private customer
models are local acceptance evidence and are not committed as public fixtures.
