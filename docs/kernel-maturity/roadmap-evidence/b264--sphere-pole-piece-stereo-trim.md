---
row: B264
---
Exit met 2026-11-18 (PR #1869): the ignored witness is un-ignored and green.
Two fallbacks, one cause: each projected a sphere trim onto a coordinate
plane, which does not bound a strongly curved piece. Seed 5201's kept cap is
bounded by the section curve and holds a pole, so it reached the 3D polygon
fallback (`point_in_polygon_3d` along the Newell normal's dominant axis).
Seed 1146's kept hemisphere is pierced twice by the tube near its rim; the
cap path decides its planar rim exactly but tested the holes in the same
axis-aligned projection, which folds once the placement tilts the rim. Both
now read in stereographic charts (`check/src/classify/boundary/sphere_stereo.rs`),
with every loop sampled along its own edges and every chord carrying a band
the edge stays within: the exact sagitta of the image arc for a circle edge
(stereographic projection maps circles to circles), twice the measured
interior deviation for a spline edge (not certified). A point inside a band
abstains. A cap keeps its exact rim half-space; each hole is read in a chart
from a pole clear of that hole, against the sphere point farthest off the
cap (in no hole), so no winding convention is used. A pole-holding or
axis-winding trim the `(longitude, latitude)` chart declines is read from the
best of 26 fixed poles clear of every loop, the pole's side taken from the
loops' total signed area under the stored-normal winding the assembly
restores; a winding that is not that of a simple trim abstains. Neither path
falls back to the projection: a trim that cannot be read abstains (a face
with no loop of any extent keeps the old path). The boundary test
(`face_trim_contains`, one-shot and prepared) decides these faces the same
way, the cap rim included. Faces the narrower exact sphere paths decide are
untouched. Independent review (gpt-6.2-sol) found three miscounts in the
first version (a circle hole's sagitta, a hole too close to the cap's far
pole falling back to the projection, the boundary test reading a cap rim from
its inscribed polygon); each has a reproducer in
`boundary/sphere_review_tests.rs`. Tests:
`regress_opseq_sphere_torus_orientation.rs::ray_vote_pole_piece_witness`
un-ignored and the matrix's B264 probe allowance removed (every placed cell
agrees at all 696 probes); `regress_sphere_pole_piece_trims.rs` (offset bore
leaving pole caps bounded by curves winding the axis, a box notch leaving a
hole within 0.08 of a hemisphere's rim; fuse, cut, intersect at scales 0.1, 1
and 10, unplaced, translated and rigidly placed; closed-form lattice
membership, sphere support `OnBoundary` and ±5% beside it, one-shot and
prepared agreeing; both fail on the unpatched classifier); unit tests in
`sphere_stereo.rs` (polar caps at three scales and placements, a band between
two winding rims, a holed cap, refusals).
