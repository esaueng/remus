---
row: B51
---
**Done (2026-11-12, this PR):** the three sweep repros are un-ignored and green, the deterministic 60-draw torus–cone sweep is clean, and `is_torus_cone_pair` is lifted from `arb_bool_pair_slow`. Not measured: the criterion benches. One third of the sweep's draws answer with typed refusals on every leg. Closure note. The two open-mesh draws are one tessellation
defect on an exact, strictly valid 4-face B-Rep, not a boolean defect: a torus
tube that runs through a frustum wall leaves the wall two pierce holes, each
one closed marched torus × cone curve, and a hole that straddles the wall's
seam meridian is cut there by the developed-chart mesher
(`planar.rs::tessellate_revolved_with_holes`), which fabricates a vertex ON
the seam. The pool pre-split that hands that vertex to both faces (stage A4,
`tessellate/solid.rs`) covered cylinder walls only, so the torus band stitched
the unsplit polyline: a three-edge T-junction per crossing, two crossings per
draw (6 boundary edges). `fill_sub_deflection_triangular_gaps` closed it at
0.1 and 0.01, which is why only finer deflections showed it. Stage A4 now
pre-splits cone walls too. Control: the axis-aligned draw turned half a turn
about the cone axis, so no hole reaches the seam, was closed at every
deflection before the change; a cylinder bar through the same cone's seam
opened with the same signature (3 boundary edges at 8e-5) and closes with it.
The disjoint 1e3 draw (torus dropped) was already green on `24040dda` before
this change, three consecutive runs; the commit that closed it was not
bisected. `b51_toruscone_sweep_oblique_fuse_mesh`,
`::b51_toruscone_sweep_axis_aligned_fuse_mesh` and
`::b51_toruscone_sweep_disjoint_fuse_drops_torus` are un-ignored.
`operations/tests/regress_b51_toruscone_seam_pierce.rs` holds fuse, cut and
intersect of both pierce draws and the disjoint draw to oracles independent
of the kernel's volume and classification: exact analytic census, both
validators strict, every edge used twice in opposite senses, closed mesh by
index and by welded position at 0.1 / 0.01 / 1e-3 / diag·1e-5, the mesh's own
signed volume and `solid_volume` against closed forms minus a semi-analytic
torus ∩ frustum integral (I = 6.7866 oblique, 5.3371 axis-aligned, bounded by
its own halving), translation invariance to 1e-5, and ray parity against the
mesh at grid points whose side analytic membership decides.
`tessellate/tests.rs::bar_through_cone_wall_seam_is_watertight` pins the
mesher change on a cone ∪ cylinder bar. With the mesher change reverted that
test and the fixture's four fuse and cut legs fail (3 and 6 boundary edges at
1e-3); the intersect legs, which keep no cone wall hole, pass either way. The
deterministic 60-draw sweep is committed as the manual
`::b51_toruscone_slow_lattice_sweep` (draws 5, 19 and 25 are the three
repros): 60 pass, 0 fail, 0 panic, with fuse / intersect / cut exact on
34 / 37 / 35 draws and the rest typed refusals (20 draws refuse all three
legs). `is_torus_cone_pair` is lifted from `arb_bool_pair_slow`. Verification
on a Windows host (`--profile ci-test`): `remus-operations` 347 of 348 test
binaries green in one run, the wasm library suite 926 passed (the gridfinity
contract tests among them), `approx_census` matching its 52 committed rows.
The failures seen reproduce with the mesher change reverted and none meshes a
holed cone wall: `regress_opseq_sphere_torus_orientation.rs::seed1146_closed_disc_rims_oppose_the_band`
(a cell declared refused answers Exact), `io/tests/analytic_face_area_fixtures.rs`,
and the BRep text-fixture tests in `io` and `wasm-io`. Noted beside the row:
B33's ignored `b26_finding20_spherecone_sibling` passes on `24040dda` with or
without this change.
