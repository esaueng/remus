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
| Connected group | `regress_remove_blends`: convex/concave bands, complete trihedral corner, seed determinism, STEP, journal resolution; `remove_blends::tests`: any seed on a band split across its axis names the whole carrier band | Partial corners, mixed radii, disconnected groups and unsupported unions. A cylindrical band split into two faces on one carrier is recognized as one band, but its reconstruction refuses ("two equally near sharp corners"): the surgical healer qualifies single-face bands only and the positional fallback cannot close the split wound (found 2026-09-25, B19 blend tranche) |
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

## Interacting shoulder and cap rims (in progress)

Tier: core — the move edits live solid topology, persistent entity history and
session checkpoints. A translating shoulder that reaches a fixed rounded cap
needs a reconstructed rolling contour, shared trim edges and new corner bands;
axial clearance alone is not functionality parity. The acceptance case raises a
19.1 mm shoulder to 24.1 mm below a 28.5 mm cap, preserving an R5 shoulder and
an R1.8 rim. Native regressions now exercise the positive transition, further
moves, reversal through separation, exact contact onset and rotated/scaled
placements. Broader qualification remains in progress. Sampled construction must
require explicit approximation permission and disclose its quality; exact-only
calls continue to refuse it atomically.

The core operation is owned by `operations/src/face_move_quality.rs` and the
bounded translation/reconstruction adapter by
`operations/src/resize_blend/axial_rim_join.rs`. The existing exact-only APIs
retain their contract. The blend trimmer exposes construction-derived split
images for edge history; journal and prepared adapters must preserve the same
explicit quality permission and transactional behavior.

Within the operations crate graph, `face_move_quality` belongs to the feature
tier; it reads common entity scope from the core evolution module. Journal
and prepared adapters remain in the facade tier. Solved contour parameters
pass directly to certified boundary splits, retaining both endpoint and
interior-domain checks instead of repeating a nearest-point search.

Additive core WASM methods expose the opt-in operation, detailed result and
prepared/journal variants; their existing exact-only counterparts do not change.
Both batch envelopes use the same implementation as the direct methods.

The core `operations/src/resize_blend/translation_shadow.rs` guard uses the
translation region's construction boundary to check volume direction when
all boundary carriers have closed-form vector-area integrals (lines, circles
and piecewise linear NURBS). It does not seed measured volume caches. Unsupported
curves or ambiguous winding retain the existing integration/mesh oracle.

Core `blend/src/cap_reblend/restore.rs` recovers the original shoulder limits
from the retained circular-section carriers before a reverse joined-cap edit.
It operates on an owned disposable candidate, records extended edge images,
and refuses ambiguous boundary roles or missing original parameter authority.

Core `blend/src/cap_reblend/touch.rs` handles the zero-width onset of contact:
the retained shoulder spring becomes shared with the analytic rim, whose
remaining lower circle bounds the boss. It creates no zero-area corner bands
and only rewrites an owned disposable candidate.
