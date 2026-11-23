---
row: B289
---
**Fixed (2026-11-23, fix/b289-thread-cone-inverted-slivers-v2):** the exit witness is green. Root
cause: the non-planar CDT triangulates a cone face in its `(u, v)` chart, which
is not a development of the cone, so a boundary chord's chart image runs beside
the 3D chord between the same two shared samples. Along the rim away from the
apex the chart chord's image bulges past the 3D chord by about the circular
sagitta times `cos a`; a base-grid point inside that lens is inside the chart
chord but past the 3D one, and the triangle the CDT builds from the chord and
that point turns over. New `tessellate/nonplanar/chord_fold.rs`
(`ChordFoldGuard`): each boundary chord (outer and hole wires) is read once
with the chart's orientation normal at its middle (central differences of the
surface a thousandth of the chord long, so it never crosses the apex); an
interior candidate beside a chord's span, within a quarter of its chart length,
is left out when the 3D triangle it makes with the chord is not within 60
degrees of that normal (side-agnostic, so holes need no orientation). It runs
in `tessellate_nonplanar_cdt`'s interior filter on cone faces only; every other
carrier keeps its grid bit for bit, and dropping an interior point never
invalidates the triangulation. Witness
`thread::tests::thread_cone_faces_mesh_without_inverted_slivers` un-ignored: on
the B276 chamfer cone at 0.01, 0.005 and the scale-derived deflection, and on
the B277 tapered thread at 0.3, 0.1, 0.05 and 0.02, closed, edge-manifold,
zero inverted triangles on any face including the cones, mesh volume within
the chord slab (on `main` it read 1, 4, 3, 3 and 3 inverted cone triangles).
Guard oracles (`chord_fold::tests`): against the closed-form chart normal
`S_u × S_v` over half angles 0.1 to 1.5, rims at slant 1e-3 to 1e3 and 2 to 7
chords per rim, every turned-over candidate is rejected and every one clear of
the lens by twice the sagitta is kept; the verdict does not depend on wire
direction; degenerate and non-finite chords are left out.

**Review fix (2026-11-23):** a fixed band of a quarter of a chord's chart
length could be scaled out of range. The CDT chart scales `v` by `du / dv`, so
on a narrow band (half-angle 0.15, longitude `0..0.5`, slant `1..1.0001`,
deflection 0.005) a candidate twice a far-rim chord's chart length from it
still lay inside the chord's lens, was never tested, and 4 of 22 triangles
turned over on either nappe and either orientation. Each chord's band is now
the larger of that quarter and four times its chart lens: the chord's image's
largest distance from its 3D chord (read at its quarter points) divided by the
chart's stretch across the chord (`|J n̂|`). The band measures the folding zone
in the CDT's own coordinates, so no chart scaling can put it out of reach. Only
interior points are still dropped. Regressions
(`tessellate::tests::cone_chord_fold`): the reviewed band on both nappes and
both orientations, and widths 1e-1 to 1e-5 over half-angles 0.05 to 1.4 at
deflections 0.005 and 0.05. Every one has zero inverted triangles against the
carrier's closed-form normal, a manifold face mesh with one boundary cycle,
and an area within the lantern bound. Before the fix, widths 1e-3 and narrower
inverted 4 of 22 triangles at deflection 0.005 at half-angles 0.05, 0.15 and
0.4 (the half-angles whose pre-fix counts were read).
