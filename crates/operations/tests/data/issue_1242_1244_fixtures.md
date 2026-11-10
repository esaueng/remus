# Issue 1242 and 1244 STEP operands

The production reports for OpenZCAD issues 1242 and 1244 built their operands
in FreeCAD and imported them as STEP. The original files were not attached, so
these were rebuilt from the recipes in the issues with the same kernel calls
FreeCAD makes, through an independent B-rep kernel's Python bindings, used
only to write these test inputs (Remus has no dependency on it). Header
timestamps are normalized and the header's producer labels are generic; the
geometric entities are as written.

The face layout is what matters. The pipe shell gives the swept tube one
torus face bounded by its two end circles and a seam along its inner equator,
so the face bulges a whole tube diameter past every one of its edges. The
torus is one face bounded only by its two seam circles. Remus's own revolve of
the same tube puts its seam on another parallel;
`regress_issue_1242_swept_tube_cut` checks that layout too.

| File | Recipe | Volume (mm³) |
| --- | --- | --- |
| `issue_1242_swept_tube_target.step` | Circle r 2 at (−8, 0, −5), normal along the arc tangent, swept along the arc (−8, 0, −5) → (0, 0, 6) → (8, 0, −5) by a pipe shell with the corrected Frenet frame (FreeCAD `makePipeShell(…, True, False)`), scaled 0.82 about the origin, translated (6, 1, 0.5) | 219.53885 |
| `issue_1242_annular_tool.step` | Rectangle x 4…10, z −6…6 in the XZ plane revolved 360° about Z | 3166.72539 |
| `issue_1244_torus.step` | Torus about Z at the origin, R 7, r 3 | 1243.57015 |
| `issue_1244_holed_block.step` | 24 × 18 rectangle centred in XY with a centred r 3 hole, extruded z −6…6, scaled 0.82 about the origin, translated (6, 1, 0.5) | 2671.21696 |

## References

From the same independent kernel on the same inputs, its volume integration
run at a 1e-9 tolerance (its default tolerance gives 175.43045 and 570.78682):

| Operation | Volume (mm³) | Witnesses |
| --- | --- | --- |
| 1242 tube − annulus | 175.43043 | excludes (6.17578517, 1.03101320, 4.87029036) |
| 1242 tube ∩ annulus | 44.10844 | holds that point |
| 1242 tube ∪ annulus | 3342.15581 | holds that point |
| 1244 torus ∩ holed block | 570.78893 | excludes (6, 1, 0.5) |

The 1244 value is confirmed in closed form. The block spans the whole tube
height, so the intersection is ∫ 2·√(9 − (ρ − 7)²)·ρ·m(ρ) dρ over ρ ∈ [4, 10],
where m(ρ) is the angular measure of the circle of radius ρ inside the block's
rectangle and outside its hole, an exact union of arcs at each ρ.
Gauss–Legendre quadrature in ρ = 7 + 3 sin t converges to 570.78893
(570.788926 at 32 000 panels).

The imported tube's planar end caps carry pcurves whose SameParameter proof
`validate_solid` cannot make ("same_parameter_proof_unavailable … plane
surface / circle edge"), so the operand itself does not validate clean; a
result that keeps those caps reports the same and nothing else.
