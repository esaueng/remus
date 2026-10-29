# Maximum distance on affine NURBS planar bodies

The existing native and direct/batch WASM maximum-distance queries admit a
bounded imported-body representation: straight-Line polyhedra whose planar
faces are clamped degree-one, equal-weight 2x2 NURBS patches. Native planes,
mixed representations, complete native spheres, and cavity-shell traversal
retain their existing behavior.

The geometry crate's existing algebraic certificate must prove an exactly
zero mixed coefficient for the represented control net. Sampling, fitting,
or nearly coplanar control points cannot authorize this path. Both endpoints
of every authoritative Line use must lie in the original finite parameter
rectangle. Convexity then proves the entire segment is contained. Every
NURBS-face use also needs the topology crate's stored whole-interval affine
SameParameter certificate, including native chart bounds and orientation.
Missing, nonfinite, unsupported, or out-of-domain uses refuse; no pcurve is
inferred. The query tolerance must cover each numerical certificate bound.

The complete straight-edged face regions are planar polygons, so their
farthest boundary point is attained at a boundary vertex. Existing candidate
enumeration, witness selection, sphere mixing, cancellation, preparation and
candidate budgets, read-only topology, and diagnostic codes are unchanged.
The input still requires valid closed BReps with simple, nested trim loops.
General planar NURBS parameterizations, curved trims and curved carriers
remain outside this qualification.

This extends an existing core query over live session topology handles; it
adds no binding, eager module, dependency, format or persistent session state.
The original NURBS carriers, pcurves, journal and checkpoints are preserved.
