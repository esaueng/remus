---
row: B277
---
**Fixed (2026-11-18, fix/thread-tube-mesh-closure):** the exit witness is green. Root cause: the reversed shoulder
triangles were the whole crescent face fanned from a self-crossing sampled
outline: the collar rim's 11.25° chords crossed the crest's samples near the
crescent tips (crest radius falls about 0.41 per radian from the rim, so any
rim chord over 7.8° crosses it), the planar CDT declined the outline and the
fan fallback wound it against the shoulder's normal. Fixed by the planar
outline-crossing bisection (stage A4b, `tessellate/planar_crossings.rs`; see
B276). Witness `thread::tests::a_flush_start_collar_meshes_closed_at_display_deflections`
un-ignored and strengthened: tapered and untapered, at 0.3, 0.1, 0.05, 0.02
and the scale-derived deflection (diagonal × 1e-5), closed, edge-manifold, no
inverted triangle off the cones, mesh volume within deflection × area of
`solid_volume`, and the untapered body's `solid_volume` within 1e-5 of the
closed form. Separate finding: the tapered thread's cones keep 3 inverted
boundary slivers at 0.3–0.05 (ignored repro
`thread::tests::thread_cone_faces_mesh_without_inverted_slivers`).
