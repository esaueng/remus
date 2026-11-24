---
row: B289
---
**Fixed (2026-11-23, fix/b289-thread-cone-inverted-slivers-v2):** the exit witness is green. Root
cause: the non-planar CDT triangulates a cone face in its `(u, v)` chart, which
is not a development of the cone, so a boundary chord's chart image runs beside
the 3D chord between the same two shared samples. Along the far rim the 3D
chord runs inside the cone by the circular sagitta, `sagitta · cos a` in slant,
so it cuts a lens off the chart: a point inside the lens is inside the chart
chord but past the 3D one, and the triangle the CDT builds from the chord and
that point turns over. On a band narrower than the lens every interior point is
inside it.

**Design: check and repair** (`tessellate/nonplanar/cone_fold_repair.rs`). Two
predictive guards (drop candidates before the CDT that would fold with a nearby
chord) were defeated in review: a fixed band missed a narrow band's far chord,
and the lens band dropped points that did not fold, kept one the CDT joined to
a chord outside that chord's span, and skipped chords whose finite differences
rounded to a point `1e8` out. The guard is gone. A cone face now triangulates
as before, then every triangle is checked: it is turned over when its normal
does not agree in sign with the cone's chart normal `S_u × S_v` at its
centroid (the line normal on the upper nappe, its opposite below the apex;
emission is chart-oriented and a reversed face is flipped as a whole after), with
a floor of `64 ε |e1| |e2|`; it stands on edge when it leans more than 53
degrees and more than twice the turn of the cone's normal between its vertices
plus position rounding (a sliver squashed onto a chord, which passes the
deflection but reads up to 1.6 times a narrow band's area). For each such
triangle the interior vertex nearest its opposite chart edge is dropped and the
CDT is rebuilt from a clone of its constrained state (`Cdt` is now `Clone`),
round by round. Boundary, hole and constraint-recovery (Steiner) vertices are
never dropped, so the shared boundary and the watertight seam are unchanged.
Bounded and deterministic: every round drops a vertex, at most 16 rounds, ties
by CDT id, and a round that turns more triangles over than the one before is
undone, so the repair never leaves more turned-over triangles than the CDT
made. Removal is preferred to inserting a refinement point: a vertex in a
chord's lens folds with it wherever it sits, and a narrow band has no room past
the lens; dropping only merges triangles over the same chart region, and the
deflection is asserted directly. Other carriers, and cone faces whose triangles
all face the cone's way, mesh bit for bit as on `main`.

**Witnesses.** `thread::tests::thread_cone_faces_mesh_without_inverted_slivers`
un-ignored: the B276 chamfer cone at 0.01, 0.005 and the scale-derived
deflection, and the B277 tapered thread at 0.3, 0.1, 0.05 and 0.02, closed,
edge-manifold, zero inverted triangles on any face, volume within the chord
slab (on `main` it read 1, 4, 3, 3 and 3 inverted cone triangles).
`tessellate::tests::cone_chord_fold`, on both nappes and both orientations,
oracle independent of the repair (closed-form normal sign, closed-form distance
from the cone of every triangle's edge midpoints and centroid, half-edge
manifold with one boundary cycle, closed-form area within the lantern bound
0.98..1.25): round 1's band (half-angle 0.15, longitude `0..0.5`, slant
`1..1.0001`, deflection 0.005): 14 triangles, 0 inverted, deviation 0.06 of the
deflection; round 2 (a) (0.74, `0..3`, `1..1.016`, 0.096; the lens guard made 4
inverted of 26 at 12.8 times the area): 36 triangles, 0 inverted, 0.07, area
1.21; (b) (apex `1e8` out, 0.05, `0..0.5`, `1..1.00001`, 0.005): 14 triangles, 0
inverted, 0.02; (c) (apex `1e9` out, 1.1, `0..0.78`, `1..1.001`, 0.001; the
lens guard left a midpoint 1.23 deflections off): 24 triangles, 0 inverted,
0.86, area 1.09; the sweep (widths 1e-1 to 1e-5, half-angles 0.05 to 1.4,
deflections 0.005, 0.05 and 0.096, apex at the origin and `1e8` out): no
defect, deviation at most 0.22 of the deflection. A band whose near rim is
split into 16 arcs keeps turned-over triangles made of boundary samples alone
(52 of 68 at half-angle 0.4, width 1e-4): the boundary forces them, the repair
leaves them and ends, and the test reports them. Repair oracles
(`cone_fold_repair::tests`): the chart normal against central differences of
`S_u × S_v` on both nappes, the sign test at scales 1e-6 to 1e6 and `1e8` out,
and the squashed vertex picked.
