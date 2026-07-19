# Stability matrix

The README labels below are retained; no feature is promoted by this audit.
Rows marked **blocked** lack the full production evidence for their advertised
domain. The P0/P1 defects found by this audit are closed; rows can remain
blocked on broader domain matrices or independent fork-CI evidence.

| README category | Feature | Current label | Disposition |
| --- | --- | --- | --- |
| Primitives | Box, cylinder, cone, sphere, torus, ellipsoid | Stable | Blocked: native/WASM invalid-input, scale, and full postcondition matrix incomplete. |
| Primitives | Convex hull, Minkowski sum | Stable | Blocked: degenerate/property coverage incomplete. |
| Booleans | Plane/cylinder/cone/sphere/NURBS union, cut, intersect | Stable | Local blocker cleared: cavity semantics pass; mesh fallback is bounded, deterministic, and fail-closed; the active 64-cut release test passes twice. Fork-CI evidence remains pending. |
| Booleans | Batch fuse-all | Stable | Blocked: depends on boolean correctness/fallback contract. |
| Booleans | Torus booleans | Beta | Retained: general torus cases remain limited. |
| Modifiers | Fillet, chamfer | Stable / Experimental | Guarded: planar line-edge requests use validated manifold builders. V2 wrappers reject partial or invalid results, preserve cavity shells, and reject closed-edge assembly before mutation; curved analytic geometry remains experimental until watertight solid assembly is implemented. |
| Modifiers | Shell | Stable | Guarded: offset-engine shell results require closed topology and L3 validation; broader curved and excluded-face matrices remain pending. |
| Modifiers | Offset, thicken, mirror, pattern | Stable | Guarded: offset no longer skips failed faces/walls, validates closure and orientation, and explicitly rejects unsupported cavity inputs instead of dropping them. |
| Modifiers | Draft | Beta | Retained: documented planar domain only. |
| Sweeps | Extrude | Stable | Blocked: full degenerate/cavity matrix incomplete. |
| Sweeps | Revolve, sweep, loft, pipe | Stable | Blocked: topology and nonconvergence budgets incomplete. |
| Sweeps | Helical sweep | Stable | Blocked: termination/performance evidence incomplete. |
| Sweeps | Non-planar profiles | Beta | Retained: documented cap and boundary limitations. |
| Construction | Coons fill, sew, untrim | Stable | Blocked: topology postconditions incomplete. |
| Sectioning | Cross-section, split by plane | Stable | Blocked: cavity and degeneracy matrix incomplete. |
| Measurement | Bounding box, area, center of mass | Stable | Evidence pending: inner-shell area, signed volume, and center regressions now pass; curved-cavity and scale matrices remain incomplete. |
| Measurement | Distance and classification | Stable | Evidence pending: all three cavity classifiers now pass inner-shell regressions; general tolerance/domain matrices remain incomplete. |
| Drawing | Hidden-line projection | Stable | Evidence pending: public error/performance matrix incomplete. |
| Geometry | NURBS evaluation and fitting | Stable | Evidence pending: degree-nine direct/cached evaluation, derivatives, and curvature sampling are fixed with a depth-limit regression; imported invariant, fitting, and large-degree budget matrices remain incomplete. |
| Geometry | Analytic intersections | Stable | Evidence pending: tolerance/domain matrix incomplete. |
| Geometry | Surface-surface intersection | Stable | Evidence pending: hard iteration budgets incomplete. |
| Geometry | Curve-curve intersection | Stable | Evidence pending: termination/property matrix incomplete. |
| Tessellation | Adaptive/CDT/analytic optimization | Stable | Local blocker cleared: any face failure aborts solid tessellation; malformed-face regression passes. Broader scale/performance evidence remains pending. |
| Repair | Healing, sewing, validation | Stable | Blocked: permissive healing can mask invalid result semantics. |
| I/O | STEP | Stable | Guarded: shared byte/entity limits are enforced and configurable. Inner-shell export and broader round-trip evidence remain pending. |
| I/O | STL, 3MF, OBJ, PLY, glTF | Stable | Guarded: byte/entity limits are enforced; 3MF separately bounds uncompressed XML. Broader round-trip/integrity evidence remains pending. |
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
