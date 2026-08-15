# Stability matrix

The README labels below are retained; no feature is promoted by this audit.
Rows marked **blocked** lack the full production evidence for their advertised
domain. The P0/P1 defects found by this audit are closed and the fork CI gate
passes; rows can remain blocked on broader domain matrices.

| README category | Feature | Current label | Disposition |
| --- | --- | --- | --- |
| Primitives | Box, cylinder, cone, sphere, torus, ellipsoid | Stable | Blocked: native/WASM invalid-input, scale, and full postcondition matrix incomplete. |
| Primitives | Convex hull, Minkowski sum | Stable | Blocked: degenerate/property coverage incomplete. |
| Booleans | Plane/cylinder/cone/sphere/NURBS union, cut, intersect | Stable | Guarded: cavity semantics pass; mesh fallback is bounded, deterministic, and fail-closed; the active 64-cut release test and final fork CI pass. A cylindrical face CROSSING a planar face of the other operand (a boss overhanging a plate edge, a bore flush with a wall) is now analytic and exact across the overlap sweep, and an acceptance gate rejects a result that has silently lost an operand. Two contact configurations still fail over to the approximate path rather than being answered analytically: exact tangency (the union's pinch vertex is not built) and a fuse whose crossing is a sliver — roughly 1e-5 to 0.05 mm on r = 10. Broader domain matrices remain pending. |
| Booleans | Batch fuse-all | Stable | Blocked: depends on boolean correctness/fallback contract. |
| Booleans | Torus booleans | Beta | Retained: general torus cases remain limited. |
| Modifiers | Fillet, chamfer | Stable / Experimental | Guarded: planar line-edge requests use validated manifold builders, and bare cylinder/cone rims use the exact validated toroidal fillet assembler. On a cylinder cap rim that assembler covers the whole radius range `0 < f < r_c` — including `f >= r_c/2`, where the carrier torus is a horn or spindle but the quarter-tube band cut from it is not, verified against the closed-form removed volume `pi((r_c - f) f^2 (2 - pi/2) + f^3/3)` and band area `pi^2 f (r_c - f) + 2 pi f^2` across the sweep. `f >= r_c` (the rolling ball no longer fits inside the cylinder), and the vertex-tolerance sliver below it (the cap face it leaves is not a face), are refused as typed `RadiusTooLarge` naming the edge and the limit — not as a partial result. The hemispherical `f = r_c` end is a different topology and is not emitted. The wall's own axial extent is a separate limit, reported the same way. A blind hole's FLOOR rim (the concave inward case) is deliberately still capped at `r_c/2` and still declines to the walker past it: its rim assembly moves volume the wrong way — an r = 3 hole rounded at r = 1 loses 7.93 where the closed form adds 3.74, while staying a validate_solid-clean closed 2-manifold — and that bound is all that limits the reach until it is fixed. V2 wrappers reject partial or invalid results and preserve cavity shells. Both engines now share G1 expansion and standard radius-law definitions. The legacy rolling-ball assembler remains until the walking engine reaches its planar/corner parity; closed-rim chamfers and other curved assembly remain experimental and fail closed. |
| Modifiers | Resize/remove analytic blend band | Experimental | Guarded: `resize_blend` re-derives constant-radius torus/cylinder band membership and two tangent supports from exact topology, checks the caller radius through kernel tolerance, snapshots the arena, and accepts only strictly valid solids with sane monotonic volume. Plane/plane bands and closed plane/cylinder rims grow, shrink, and remove exactly; a Shapr3D cylinder/cone band removes to its exact sharp circle, while positive-radius cylinder/cone reconstruction remains a typed `unsupported-support-pair` refusal pending the broader support matrix. Freeform contact, radius mismatch, oversized radii, ambiguous topology, invalid output, and partial fillet results refuse with stable codes and restore existing handle slots. Native and WASM bindings return construction-derived face evolution; variable-radius bands remain out of scope. |
| Modifiers | Shell | Stable | Guarded: offset-engine shell results require closed topology and L3 validation; broader curved and excluded-face matrices remain pending. |
| Modifiers | Offset, thicken, mirror, pattern | Stable | Guarded: offset no longer skips failed faces/walls, and validates closure and orientation on every shell, not just the outer one. A solid with cavity shells is now offset rather than refused — the shell partition is preserved and an outward distance shrinks each cavity — with the necessary shell-separation condition enforced and the input refused when it fails; excluding faces from such a solid still refuses. `JointType::Arc` now builds the rolling-ball offset of a convex polyhedron, checked against the Minkowski/Steiner closed form for volume and area and holding it from 1e3 to 1e-3 scale; a curved source face, a concave or tangent edge, a holed face, a cavity, an excluded face, or an inward distance each refuse with their own reason rather than fall back to the mitred joint. Still refusing, unchanged: global self-intersection removal, and NURBS-NURBS 3D intersection. |
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
| Evolution | Face provenance (booleans, blends, patterns) | Beta | Retained; scope widened, label unchanged. Exact construction-derived provenance covers booleans (GFA face origins), walking and planar fillet/chamfer builders, and patterns. The planar builders carry each face specification through assembly; rolling fillets also carry it through production same-surface unification. The versioned WASM blend payload enumerates complete source/result handle domains, makes deletions and uncertainty explicit, and rejects duplicate, contradictory, phantom, or incomplete claims. Stable fillet/chamfer claims are never inferred from proximity, traversal order, or approximate geometry. The older scale-relative geometric matcher remains available to other Rust evolution routes and marks ambiguous output `unresolved`. Still Beta: offset, shell, draft, split, defeature and direct edits produce no provenance; fallback blend engines without construction records report explicit unavailable provenance; and there is no edge or vertex provenance. |
| Defeaturing | Planar face removal | Beta | Retained. |

The evidence required to lift any blocked stable row is the full gate set in
the audit request: documented domain/error/fallback behavior, negative and
boundary regressions, bounded iteration, validated watertight output, native
and WASM consistency, determinism, CI coverage, and a representative
integration result.
