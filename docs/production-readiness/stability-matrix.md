# Stability matrix

The README labels below are retained; no feature is promoted by this audit.
Rows marked **blocked** have an unresolved P0/P1 issue in their advertised
domain or lack the required production evidence.

| README category | Feature | Current label | Disposition |
| --- | --- | --- | --- |
| Primitives | Box, cylinder, cone, sphere, torus, ellipsoid | Stable | Blocked: native/WASM invalid-input, scale, and full postcondition matrix incomplete. |
| Primitives | Convex hull, Minkowski sum | Stable | Blocked: degenerate/property coverage incomplete. |
| Booleans | Plane/cylinder/cone/sphere/NURBS union, cut, intersect | Stable | Blocked: cavity handling and mesh-fallback quality contract unresolved. |
| Booleans | Batch fuse-all | Stable | Blocked: depends on boolean correctness/fallback contract. |
| Booleans | Torus booleans | Beta | Retained: general torus cases remain limited. |
| Modifiers | Fillet, chamfer | Stable | Blocked: success paths need watertight/cavity postconditions. |
| Modifiers | Shell | Stable | Blocked: cavity and invalid-input matrix incomplete. |
| Modifiers | Offset, thicken, mirror, pattern | Stable | Blocked: offset can return partial/open results. |
| Modifiers | Draft | Beta | Retained: documented planar domain only. |
| Sweeps | Extrude | Stable | Blocked: full degenerate/cavity matrix incomplete. |
| Sweeps | Revolve, sweep, loft, pipe | Stable | Blocked: topology and nonconvergence budgets incomplete. |
| Sweeps | Helical sweep | Stable | Blocked: termination/performance evidence incomplete. |
| Sweeps | Non-planar profiles | Beta | Retained: documented cap and boundary limitations. |
| Construction | Coons fill, sew, untrim | Stable | Blocked: topology postconditions incomplete. |
| Sectioning | Cross-section, split by plane | Stable | Blocked: cavity and degeneracy matrix incomplete. |
| Measurement | Bounding box, area, center of mass | Stable | Blocked: inner-shell correctness review found gaps. |
| Measurement | Distance and classification | Stable | Blocked: cavity classification is incorrect. |
| Drawing | Hidden-line projection | Stable | Evidence pending: public error/performance matrix incomplete. |
| Geometry | NURBS evaluation and fitting | Stable | Blocked: degree-nine panic and fitting endpoint defect unresolved. |
| Geometry | Analytic intersections | Stable | Evidence pending: tolerance/domain matrix incomplete. |
| Geometry | Surface-surface intersection | Stable | Evidence pending: hard iteration budgets incomplete. |
| Geometry | Curve-curve intersection | Stable | Evidence pending: termination/property matrix incomplete. |
| Tessellation | Adaptive/CDT/analytic optimization | Stable | Blocked: failed faces can be skipped while output is returned. |
| Repair | Healing, sewing, validation | Stable | Blocked: permissive healing can mask invalid result semantics. |
| I/O | STEP | Stable | Blocked: hostile-input resource limits and inner-shell export coverage incomplete. |
| I/O | STL, 3MF, OBJ, PLY, glTF | Stable | Blocked: parser budgets and round-trip/integrity matrix incomplete. |
| I/O | IGES | Experimental | Retained: scope is accurately limited in README. |
| Sketching | DogLeg solver | Stable | Evidence pending: nonconvergence budget and degeneracy matrix incomplete. |
| Feature recognition | Holes, pockets, chamfers, fillets | Beta | Retained. |
| Assemblies | Hierarchy, transforms, BOM | Beta | Retained. |
| Evolution | Boolean provenance | Beta | Retained. |
| Defeaturing | Planar face removal | Beta | Retained. |

The evidence required to lift any blocked stable row is the full gate set in
the audit request: documented domain/error/fallback behavior, negative and
boundary regressions, bounded iteration, validated watertight output, native
and WASM consistency, determinism, CI coverage, and a representative
integration result.
