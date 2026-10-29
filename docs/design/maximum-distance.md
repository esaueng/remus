# Global maximum-distance witnesses

Tier: core

This extends the existing live-session solid distance API. It reads current
topology handles, returns a distance and two boundary witnesses, and changes no
topology, naming history, checkpoints, tolerances, or minimum-distance behavior.
It introduces no dependency or sampled fallback.

`solidToSolidMaximumDistance(a, b)` returns seven numbers:
`[distance, ax, ay, az, bx, by, bz]`, through direct, legacy batch, and V2 batch
calls. Native callers also have an operation-context entry for cancellation and
the existing vertex-pair work budget.

Inputs must already be valid BReps with closed shells and simple, properly
nested trim loops. Callers validate those solid and trim invariants; this
query qualifies the supported carriers and their numeric boundary authority.

The first supported families are straight-edged native planar polyhedra and
qualified complete native spheres, including mixed pairs. Every outer and
cavity boundary is inspected. Curved polyhedron boundaries, partial spheres,
inconsistent authoritative loops, non-finite geometry, and numerically
unrepresentable witnesses return typed refusals.

For planar polygonal faces, every point lies in the convex hull of boundary
vertices. Convexity of distance in each argument therefore reduces the global
maximum to vertex pairs, including nonconvex faces and faces with holes.
Complete sphere pairs use center separation plus both radii; a sphere/polyhedron
pair maximizes center-to-vertex separation plus radius. Coincident centers use a
deterministic axis. A body's maximum distance to itself is its diameter.

Face-owned Loops and Coedges are authoritative. Qualification checks ownership,
wire agreement, closed connected traversals, finite endpoints and strict edge
domains before any carrier shortcut. Sphere qualification also checks both
finite right-handed orthonormal frames, equatorial boundary points, native full
coverage, and actual noncollapsed radial witnesses.

Preparation allows one million shell/face/loop/coedge items across both bodies
and at most 4096 coedge uses per face. It is bounded separately from the Cartesian vertex-pair budget, so
64 box vertex pairs admit at budget 64 and refuse at 63. Cancellation is checked
throughout preparation and comparisons. Unsupported geometry refuses the whole
query; no face or cavity is silently omitted. Results use ordinary floating
point arithmetic on these qualified global extrema, without claiming an
interval-arithmetic evaluator-error certificate.

Independent tests use the farthest-component formula for axis-aligned boxes,
sphere center separation plus radii, and mixed sphere/box extrema at three
scales and rigid placements. They check witness membership, witness separation,
same-body diameter, overlap, containment, authoritative boundaries, malformed
and unsupported inputs, cancellation, exact work-budget boundaries, and
unchanged session state. Canonical fresh and installed-tarball tests exercise
all three browser routes.
