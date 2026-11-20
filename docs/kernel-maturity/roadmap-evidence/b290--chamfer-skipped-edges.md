---
row: B290
state: Done (2026-11-20, [#1895](https://github.com/esaueng/remus/pull/1895))
---
**Fixed (2026-11-20, [#1895](https://github.com/esaueng/remus/pull/1895)):** the drop stays (a seam
or tangent join has no dihedral, and "every edge of the body" stays a valid
selection), but it is no longer silent. `BlendResult` gains `skipped:
Vec<(EdgeId, BlendSkip)>`, `BlendSkip` being `Seam` (a closed face meeting
itself) or `Tangent` (a G1 or flat join between two faces);
`query::normalize_{fillet,chamfer}_selection_with_skips` return what the
normalizer dropped, and `chamfer_v2` (with `chamfer_distance_angle`,
`chamfer_with_evolution`, `chamfer_journaled`, `Model::chamfer`) and the
normalized fillet entry points (`Model::fillet`, `fillet_journaled`) report
it. A skip does not make the result partial. `chamferV2Detailed` and
`chamferDistanceAngleDetailed` (direct and both batch twins) add
`details.skippedEdges: [{ edge, reason }]` when one was skipped; the key is
absent otherwise. Witnesses: `operations/tests/regress_chamfer_cylinder_seam.rs`
(both rims and the seam at 0.05, (0.05, 0.15) and (0.4, 0.1), seam named in
any order, placed and rotated, seam-only refusal, and the fuzz target's
11-face box-box reproducer naming its flat join `Tangent`); every rim band is
an exact cone about the axis through both contact circles, the body valid,
closed, manifold and mesh-watertight, and its volume within 1e-9 of
`π r² h − 2π a b (r − a/3)` at deflections 1e-4 and 1e-5. Four of the five
fail with the disclosure line removed (`skipped` empty). Wasm:
`modifier_detailed::tests::chamfer_v2_twin_names_the_cylinder_seam_it_skips` (`chamferV2Detailed` and `chamferDistanceAngleDetailed`, direct and both batch routes); `remus` `model::tests::{fillet,journaled_fillet}_skips_a_cylinder_seam_in_a_whole_body_selection` pin the seam in `skipped`.
The fuzz oracle I8 now counts `succeeded + skipped`. Not changed:
`fillet_v2` still refuses the cylinder's rims-plus-seam selection with an
unsupported vertex blend (the walking engine reads the seam as a stripe), the
legacy `chamfer` (planar bevel first) and `chamferDetailed` do not disclose,
and a closed NURBS face whose seam is a crease is still dropped as a seam.
