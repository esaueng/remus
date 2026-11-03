# Rotational direct editing

Tier: core. These operations edit live session solid handles and record their
construction history. The owning roadmap row is P-Class 6.1.

The qualified radial cell is one shell of at most 64 coaxial analytic faces:
axial planes, cylinders, cones and rectangular torus bands. The selected wall
has two complete circular rims and at least one tangent torus band. Each
incident band has the selected cylinder and one fixed axial plane or cone as
its two supports. Partial turns, freeform supports, cavity shells, other
band junctions and changes that alter the adjacency graph remain outside this
cell.

Changing a wall radius carries its incident bands. Their minor radii and
minor-angle spans stay fixed. Their major radii change by the same radial
displacement as the wall. A plane holds the band's axial level fixed; a cone
holds its meridian line fixed, so the band translates axially by the radial
displacement divided by the cone slope. The other surfaces stay fixed and
their boundary circles and seam endpoints are re-limited to the moved bands.
The construction copies every entity once and records a total face, edge and
vertex correspondence. It does not infer identity from proximity.

The operation reconstructs the complete analytic meridian from authoritative
circle and seam spans. Line/line, line/circle and circle/circle intersections
must remain at their existing shared boundary points. Collapsed and reversed
segments, a collapsed torus ring and new nonincident contacts refuse the edit.
The exact revolution integral, `pi * integral(r^2 dz)`, checks the result's
volume against the independent B-rep measurement route. Strict solid
validation remains required. A failure restores the topology and journal.

Two related axial face-move cells support imported rotational parts. A
circular planar face translated along its axis cannot reach a radially
separate coaxial support even when that periodic support's bounding box covers
the axis. Full analytic radial bounds, or a torus strip's authoritative
meridian interval, establish that exclusion; unqualified pairs retain the
existing conservative swept-box check. An axial planar cap adjacent to a cone
re-limits along an exact circular plane/cone intersection, preserving its
signed circle span, seam direction and entity counts. General oblique conical
cap moves remain unsupported.

`crates/io/tests/rotational_face_edits.rs` uses the existing
walking-stick-foot STEP fixture and a sequence of imported blend, rim, bore
and floor edits. It checks both offset signs, fine and large bore changes,
both outer walls, exact bore/frustum volume formulas, analytic material probes,
strict validation, watertight tessellation, unchanged cone supports and STEP
round trips. Repeated journaled edits under rotation, translation and three
scales must retain every construction reference. Collision and collapse cases
must refuse without changing the serialized source.

`scripts/rotational-face-edits-packaged.mjs` exercises the same feature chain
through direct, both batch APIs and journaled entry points. It runs in the
in-tree smoke and installed-tarball consumer harnesses. It also checks exact
floor volume, source geometry and boundary authority, complete analytic face
census, STEP round trips and refused-edit rollback. Consumer application
preview, commit, cancellation, undo/redo and durable reload are separate
OpenZCAD acceptance gates.
