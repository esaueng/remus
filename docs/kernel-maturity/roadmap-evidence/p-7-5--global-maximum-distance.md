---
row: p-7-5
---

Tier: core. `solidToSolidMaximumDistance` extends the existing live-session
measurement interface. It reads current solid handles and authoritative
boundaries without changing topology, evolution history or checkpoints.

Inputs are already-valid BReps with closed shells and simple, nested trim loops.
The supported global families are straight-Line native planar polyhedra,
complete native spheres, and mixed pairs. Planar faces may be nonconvex and
have holes; cavity shells are traversed too. Vertex convex-hull reduction and
closed-form spherical extrema produce a distance and two boundary witnesses.
Same-body queries return a diameter. Curved polygonal boundaries, incomplete
spheres, non-finite or inconsistent geometry and unresolved finite arithmetic
refuse explicitly. There is no sampled fallback or minimum-distance change.

Native context entry points honor cancellation and the existing candidate
comparison budget. Preparation permits one million shell/face/loop/coedge
items across the two bodies, with at most 4096 coedge uses per face bounding
native authority/coverage helper calls. The default vertex-pair limit remains
one million. Results are ordinary floating-point global extrema on qualified
geometry, without a new interval-arithmetic evaluator-error claim.

Independent oracles in `operations/src/distance/maximum_tests.rs` cover the
farthest-component box formula, sphere overlap/containment/coincidence, mixed
pairs, rigid placement and three scales, actual witness membership/separation,
all cavity boundaries, stale wire authority, preparation limits, the exact
64/63 comparison-budget boundary, cancellation and unchanged mutation/history.
`maximum-distance-packaged.mjs` exercises direct and both batch routes in the
canonical fresh and installed-package suites, including typed refusals and
unchanged serialized geometry/history/session counts. Other curved extrema,
wall-thickness maps and general interrogation remain open.

The posed-sphere regression also covers the analytic transform producer:
directions use only the affine matrix's linear part, matching the curve writer,
so translated-point subtraction cannot rotate the surface frame through
cancellation. The sphere coverage checks and tolerances remain unchanged.
The exact numeric browser pose is checked at three scales against closed-form
volume at two deflections, closed manifold BRep and mesh boundaries, material
classification and maximum-distance witnesses. Topology refusals preserve the
query-specific diagnostic prefix across the direct and both batch contracts,
along with native topology details and checkpoint sharing.
