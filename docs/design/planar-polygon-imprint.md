# Bounded planar polygon imprint

Tier: core. This operation edits the live topology, entity handles,
construction journal, persistent naming and checkpointed session. Its points
array enters the native transaction before any private ring is allocated.

`imprint_planar_polygon` and `imprintPlanarPolygon` partition one existing
solid face into an inner planar patch and its annular surround. Both reuse
the original plane and share the new straight ring with opposite coedge uses.
The result returns the solid, both patch handles and the construction
operation ID. It changes the partition, not the solid's material.

The first family requires one planar face with no holes and a simple,
strictly convex all-Line outer polygon. The supplied ring has 3–16 finite
world points, is simple and strictly convex, lies on that plane, and stays
strictly inside every target side by the unchanged linear clearance. Target
loop work is capped at 32 vertices independently. Whole-polygon side tests
reject stars,
crossings, repeated/collinear vertices and nonconvex loops; local turn signs
alone do not qualify a polygon. Unresolved finite arithmetic refuses.

The native context variant checks cancellation and the existing segment work
and vertex-pair budgets. With target size n and ring size m, admission costs
n+m segments and n(n-2)+m(m-2)+nm halfspace tests. Every subtraction, cross
product, dot product and edge-length bound uses outward interval arithmetic;
the lower signed side bound must exceed the upper linear-distance clearance
product. Plane residual intervals must lie inside the original linear
tolerance. This can refuse badly conditioned inputs without changing their
coordinates or tolerance. All validation, private ring allocation, shell
replacement, attributes
and history share one transaction. Unsupported carriers, curved trims, target
holes, boundary crossings/overlaps, nonfinite data and resource refusals
publish no partial handles or history. Public refusal paths also retain an
already shared checkpoint topology owner; the unshared path retains the
existing mutation-local rollback and private-slot high-water behavior.
Result validation reports only counts, and usable private result handles
are returned only on success. No GFA, general arrangement, coordinate
weld, geometry fit or invented planar UV pcurve participates.

Only the shell owning the selected face is replaced; all other outer/cavity
shells and untouched faces, edges and vertices retain their identities. Both
new faces are Modified from the selected face, every new ring edge and vertex
is Generated from that face, and every other result entity is Preserved.
The result scope is total over F/E/V. Face attributes copy through the exact
construction entry. A prior selected-face reference resolves to both new
faces with Construction provenance, rather than crossing a global barrier.

Qualification must cover native, direct, legacy batch and v2 batch paths,
three scales, rigid placement, reversed rings/face uses and cavity ownership.
The closed-form box witness partitions area 80 into 16 and 64 while volume
stays 320. Whole-solid validators must report zero errors, indexed and welded
meshes must have zero boundary/nonmanifold edges, material probes must remain
unchanged, and checkpoint/undo and typed refusals must preserve exact state.
The existing check-crate Euler heuristic warns at chi=3 because it counts the
annular face without subtracting its hole. That warning remains visible;
validators and tolerances are not weakened. Fresh and installed-tarball
consumers must execute the permanent imprint witness.

Boundary-crossing wires, target holes, concave rings/targets, curved carriers
and trims, multiple disconnected rings, emboss/pocket construction and general
surface imprint remain separate capabilities.
