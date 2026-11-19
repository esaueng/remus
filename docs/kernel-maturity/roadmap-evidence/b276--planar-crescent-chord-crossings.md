---
row: B276
---
**Fixed (2026-11-18, fix/thread-tube-mesh-closure):** the exit witness is green. Root cause: the start collar's
shoulder is a planar crescent between the boss rim circle and the thread's
first crest, which leaves the rim at a small angle. At deflection 0.01 a rim
chord passed the crest's denser samples near the crescent's tip, so the sampled
outline crossed itself; the planar CDT cannot recover crossing constraints
without an unshared Steiner vertex, fell back to a fan, and the fan wound the
crescent inward (221 open edges). New boundary-plan stage A4b
(`tessellate/planar_crossings.rs`) bisects, on the curve, the longer curved
chord of every crossing pair of a planar face's sampled outline until none
cross (bounded: 32 rounds, one sample per chord per round, line-line crossings
left alone; every planar face on a split edge is read again, and if the bounds run out with crossings left every chain goes back to what was sampled); the samples go into the shared chains, so every face on the edge
uses them. Witness `thread::tests::fine_pitch_thread_with_a_start_collar_meshes_closed`
un-ignored: valid, and at 0.01, 0.005 and the scale-derived deflection
(diagonal × 1e-5) closed, edge-manifold, no inverted triangle off the cones,
mesh volume within deflection × area of `solid_volume`. Separate finding, not
in this row's exit: the chamfer cone keeps 1 (0.005) and 44 (scale-derived)
inverted slivers where the non-planar CDT's base grid keeps a point a few 1e-8
inside a boundary chord in the chart; ready repro
`thread::tests::thread_cone_faces_mesh_without_inverted_slivers` (ignored).
