---
row: B264
---
Exit met 2026-11-18: the ignored witness is un-ignored and green. Two fallbacks, one cause: each projected a sphere trim onto a coordinate
plane, which does not bound a strongly curved piece. Seed 5201's kept cap is
bounded by the section curve and holds a pole, so it reached the 3D polygon
fallback (`point_in_polygon_3d` along the Newell normal's dominant axis).
Seed 1146's kept hemisphere is pierced twice by the tube near its rim; the
cap path decides its planar rim exactly but tested the holes in the same
axis-aligned projection, which folds once the placement tilts the rim. Both
now read in a stereographic chart (`check/src/classify/boundary/sphere_stereo.rs`):
a cap's holes from the sphere point farthest off the cap, which lies in no
hole, so each hole bounds its own image and no winding convention is used;
a pole-holding or axis-winding trim the `(longitude, latitude)` chart
declines from the best of 26 fixed poles clear of every loop, with the side
of the pole read from the loops' total signed area under the stored-normal
winding the assembly restores, and a hit whose winding is not that of a
simple trim abstaining from the vote. The boundary test (`face_trim_contains`,
one-shot and prepared) reads the same chart, so points on such pieces are
`OnBoundary`. Faces the narrower exact sphere paths decide are untouched.
Tests: `regress_opseq_sphere_torus_orientation.rs::ray_vote_pole_piece_witness`
un-ignored and the matrix's B264 probe allowance removed (every placed cell
agrees at all 696 probes); `regress_sphere_pole_piece_trims.rs` (offset bore
leaving pole caps bounded by curves winding the axis, a box notch leaving a
hole within 0.08 of a hemisphere's rim; fuse, cut, intersect at scales 0.1, 1
and 10, unplaced, translated and rigidly placed; closed-form lattice
membership, sphere support `OnBoundary` and ±5% beside it, one-shot and
prepared agreeing; both fail on the unpatched classifier); unit tests in
`sphere_stereo.rs` (polar caps at three scales and placements, a band between
two winding rims, a holed cap, refusals).
