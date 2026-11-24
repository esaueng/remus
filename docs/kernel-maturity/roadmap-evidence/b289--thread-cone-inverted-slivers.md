---
row: B289
---
**Partly fixed (2026-11-23, #1973):** the exit witness
`thread::tests::thread_cone_faces_mesh_without_inverted_slivers` runs un-ignored
and is green: the B276 chamfer cone at 0.01, 0.005 and the scale-derived
deflection, and the B277 tapered thread at 0.3, 0.1, 0.05 and 0.02, closed,
edge-manifold, with no inverted triangle on any face. On `main` it read 1, 4, 3
and 3 inverted cone triangles. Narrow cone bands where the repair cannot help
remain open in #1979 (see below).

**Root cause.** The non-planar CDT triangulates a cone face in its `(u, v)`
chart, which is not a development of the cone. A boundary chord's chart image
therefore runs beside the 3D chord between the same two samples. Along the far
rim, the 3D chord cuts a lens off the chart: a point inside the lens is inside
the chart chord but past the 3D one, and the triangle the CDT builds from the
chord and that point turns over.

**Design: detect, repair, gate** (`tessellate/nonplanar/cone_fold_repair.rs`).
- **Detection.** Detection is a sign test only: a triangle's normal against
  the cone's chart normal `S_u × S_v` at its centroid, with a floor of
  `64 ε |e1| |e2|`. It runs on the triangle list emission already builds, so a
  cone face with no turned-over triangle does no snapshot and no extra
  allocation, and meshes exactly as before.
- **Repair.** Only when a triangle is turned over, the face's constrained CDT
  is rebuilt from scratch without the interior vertex each turned-over
  triangle is squashed onto. This repeats round by round, at most 16 rounds,
  and stops at a round that turns more triangles over. Boundary, hole and
  Steiner vertices are never dropped, so shared boundaries stay watertight.
- **Gate.** A rebuilt mesh replaces the CDT's plain mesh only when two things
  hold. It must turn strictly fewer triangles over. And every triangle it does
  not share with the plain mesh must lie within the deflection of the cone,
  by `deviation_bound`. Otherwise the face keeps the plain mesh, which is
  `main`'s. Triangles the rebuild shares with the plain mesh are the CDT's
  own: on the B277 taper at 0.05 one already lies at 1.02 of the deflection,
  and no repair touches it.
- **The bound.** `deviation_bound` is an upper bound, not a sample. It reads
  the closed-form generator-line distance `|r sin a − z cos a|` (convex `r`,
  so it is concave over a flat triangle whose vertices lie on the cone) on a
  6-division barycentric grid plus 8 points per edge. It then adds the
  interpolation error `½ (sin a / r_min) ((T + R sin Δ) / 6)²`: the Hessian
  of `r` acts only circumferentially; `T` and `R` are the triangle's
  tangential and radial extents and `Δ` its angular half-span; `r_min` comes
  from the tangent planes of `r`. A triangle that may reach the axis has no
  bound. Checked against a 400-division brute force, it holds the review's
  π/4 facet (peak 0.1972, 35% along an edge).
- **Dropped.** The earlier "standing" (edge-on) test is gone.

**Witnesses** (`tessellate::tests::cone_chord_fold`). Every band is meshed with
and without the repair. The mesh must equal the plain one bit for bit, or have
strictly fewer inverted triangles against the carrier's closed-form normal with
every changed triangle within the deflection under the same dense sampling. It
must also be manifold with one boundary cycle. Bit identity with `main` is
pinned by fingerprints captured on `main` (`bccdad52`): four clean cone bands
on either nappe and orientation, and a primitive cone solid at two deflections.

| Case | `main` | Now |
|---|---|---|
| Round 1 (0.15, `0..0.5`, `1..1.0001`, 0.005) | 4 inverted | 0 inverted, 0.06 of the deflection |
| Apex `1e8` out (0.05, `0..0.5`, `1..1.00001`, 0.005) | 4 inverted | 0 inverted, 0.02 |
| (0.74, `0..3`, `1..1.016`, 0.096) | clean | unchanged |
| Apex `1e9` out (1.1, `0..0.78`, `1..1.001`, 0.001) | clean | unchanged |
| Round 3 case 2 (1.1, `0..3`, `1..1.001`, 0.001) | 234 triangles, 0 inverted | bit-identical |
| Sweep (widths 1e-1 to 1e-5, half-angles 0.05 to 1.4, three deflections, apex at the origin and `1e8` out; 720 meshes) | 512 inverted | 0 inverted, 128 meshes repaired |

**Remaining (#1979; fix direction: mesh cone faces in a length-true chart,
the development or a metric chart as in `tessellate/nonplanar/sheared_chart.rs`):**
- **Round 3 case 1** (0.4, `0..2.5`, `1..1.0003`, 0.001). The plain mesh turns
  65 of 212 over. Every repair round leaves a facet far past the deflection
  (on the earlier design, area 0.38 at 0.184 from the cone), so the gate keeps
  `main`'s mesh with its 65.
- **Round 3 case 4** (0.03, `0..1.25`, `1..1.0005`, 0.001). Reduced from 30 to
  8 inverted, within the deflection (0.72 of it).
- **A boundary-forced band** (0.4, `0..0.5`, `1..1.0001`, 0.005, near rim split
  into 16 arcs). Reduced from 60 to 52 inverted, all made of boundary samples
  alone, which removal cannot fix.

**Repair oracles** (`cone_fold_repair::tests`): the chart normal against
central differences of `S_u × S_v` on both nappes; the sign test at scales 1e-6
to 1e6 and `1e8` out; the squashed vertex picked and a hole's triangle not
counted; and the deviation bound against brute force (at most 1.25 of the
maximum on mesh-sized triangles, 2.5 on a generator-long sliver, infinite
across the axis).
