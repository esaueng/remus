# Bounded sequential tab-fillet rollover

The bracket regression rounds its four inner tab verticals at radius 2, then
selects the two horizontal concave tab/base edges at radius 3. The previous
walking path follows a tangent chain onto the earlier cylindrical bands and
refuses where that chain reaches the coplanar front face. The bounded new
attempt runs only after the complete existing default fillet cascade has
refused, including its reorder and cavity-carry attempts. Existing successful
operations retain their engine and result.

The admitted support family has one or two disjoint tab feet. Each has a
straight back wall, two convex native quarter-cylinder walls of equal radius
`a`, a straight inner side between them, and planar floor/front/outer caps.
The front plane is parallel to the cylinder axes and tangent at the inner
radius of the new torus. Recognition uses support geometry, native edge trims,
orientation and adjacency. Fixture coordinates and face indices are not keys.
Input cavities, inner wires, unqualified carriers or trim definitions, cap
interruptions, intersecting reaches, and contacts without positive clearance
remain outside this family.

Read-only admission evaluates every original authoritative edge endpoint
against its claimed vertex ball. A quarter-circle's native trim endpoints must
match the support stations within the unchanged modelling tolerance. A shifted
trim with the same quarter-turn width cannot be silently repaired by replacing
that old arc. The poisoned-trim regression requires exact input bytes, live
entity counts, journal and allocated-slot count to remain unchanged on refusal.

Each foot adds two exact cylinders and two exact tori. Its front torus shares
a rational quartic boundary with the front plane; all other new boundaries
are native circles or lines. New vertices and edges are shared by construction
role and typed endpoint IDs. Unchanged copied faces retain their native edges,
trims, attributes and qualified PCurves. The general assembly's Circle-only
torus guards and the walking trimmer's unsupported-family guards are retained.

For a torus with major radius `R=a+r`, minor radius `r`, and section-plane
distance `d=R-r=a`, choose section coordinates `x,z` relative to its centre.
The complete section satisfies

```
(x² + z²)² = 4 R (r x² - d z²).
W(s) = 1 + (2 + 4r/d)s² + s⁴
X(s) = 2 sqrt(Rr) (1 - s⁴)
Z(s) = 4r sqrt(R/d) s (1 - s²)
x = X/W, z = Z/W
```

The finite branch uses
`s0=-sqrt(d)/(sqrt(2R-r)+sqrt(R))` through `s=-1`. Affine restriction and
power-to-Bernstein conversion give a degree-four rational Bezier with positive
weights. The coefficient identities
`X²+Z²=4Rr(1-s²)²W` and
`rX²-dZ²=4Rr²(1-s²)⁴` prove the complete section. Native tests reconstruct
coefficients from the stored homogeneous control net, test endpoints/tangents,
placement/scale and both reflected branches, and reject an altered interior
control point.

Every result edge use has a PCurve and passes the strict SameParameter and
SameRange contracts. Planar quartic coordinates are an affine rational twin.
The torus angles of a rational 3D quartic generally are not rational, so their
2D cubic interpolation is explicitly approximate within the existing linear
tolerance. No 3D contact curve or surface is fitted. Admission requires a
whole-interval certificate: outward interval Bernstein products bound the
rational quartic's second derivative, the cubic UV derivative control nets
bound the lifted torus derivative, and the residual remainder on each interval
is at most `M h²/8`. Point evaluations alone never authorize a use. Refinement
is bounded to 257 UV controls and 65,536 proof intervals; a proof that does not
fit the existing tolerance and arithmetic budget refuses. This can refuse
otherwise geometrically similar cases at large scales or placements.

The planar certificate bounds the complete homogeneous residual divided by
its positive denominator. Unsafe 2D common-weight factors refuse because the
actual 2D evaluator falls back below machine epsilon, whereas the 3D evaluator
normalizes weights. Nonfinite interval arithmetic and nonpositive denominator
bounds refuse. Native Circle/Line2D use certificates compare trigonometric
coefficients and parameter frequencies over the entire stored trim, including
decreasing trims and reverse coedges. Their arithmetic bound includes original
parameter interpolation, Line2D evaluation, raw carrier inputs and native edge
angle interpolation, propagated through the carrier's derivative gains. Large
cancelling parameters can agree at both endpoints while their rounded interior
differs, and a small meridian surrogate can hide cancellation in a huge torus
carrier. Explicit adversarial regressions require both cases to remain unproved.

The operation snapshots original face regions before using the existing
traced copy and coplanar unifier. It then composes actual source regions with
the sealed reconstruction. In particular, a front-floor quarter-spandrel is
buried when `a+r` exceeds its full radial extent `a sqrt(2)` with positive
clearance. It is deleted in that case, rather than incorrectly declared a
survivor of the merged floor. A partially retained region must have proved
positive remaining area; uncertainty at the boundary refuses. Each generated
band names its local original support parents. All original and result faces
are covered by disjoint surviving/deleted and modified/generated claims.
The existing journal API retains its explicit edge/vertex resolution policy;
this operation does not invent identity claims for new edges or vertices.

Read-only family and existing-PCurve admission precede the transaction
snapshot. Construction, unification, new PCurves, history and the existing
solid/volume postconditions are transactional. Any failure preserves source
geometry, live boundary authority and journal state, and retired handles remain
stale. The public cascade returns its prior typed refusal outside the admitted
family.

The retained F3 input came from committed pair 2026.1.81, distribution
`831741cc35c95acd58481ae7943003b97b67ed18`, with publisher-attributed source
`1853ad33cee018b01dfe135ccb3415e9503ceb62`. The independent F4 arena witness is
separate from the native constructor. Both are exercised through the existing
shared-boundary plan and face emitters before vertex welding, triangle cleanup
or gap filling. The raw checks include opposite edge uses, outward nondegenerate
triangles, torus area and whole-body volume.

The independent quantity oracle integrates added material. With back/side
straight lengths `L,H`, section area `A=r²(1-pi/4)` and radial first moment
`M=r³(5/6-pi/4)`, each foot adds

```
(L+H) A + (pi/2)(a A + M) + J
J = integral[a,a+r] rho * (r - sqrt(2r(rho-a)-(rho-a)²)) * asin(a/rho) d rho
```

Substitution `rho-a=r(1-cos(theta))` removes the endpoint square root. For the
retained `a=2,r=3,L=16,H=6` bracket, independent refinement gives whole volume
`29746.712312911633` and front-torus area `10.97868850495537`. The tests use
fine linear and angular mesh refinement and local-reference tetrahedra,
separate from the public measurement route. Material probes independently
cover the straight runs, complete and clipped torus regions, front exterior,
retained vertical rounding, base and notch.

Maintained regression entry points are the `inner_tangent` operations tests,
`bracket_sequential_fillets`, the `bounded_tab_rollover` native WASM binding
test, and `scripts/bounded-tab-rollover-packaged.mjs` in both fresh and
installed-tarball consumer harnesses. They cover history, source/unrelated-body
preservation, prior F1/F2/F3 construction, public default/detailed/batch paths,
strict boundary coverage, exact-only disclosure and transactional refusal.
General multi-support walking, arbitrary torus cuts, further rollover through
this new band, and the remaining B4 curved-trimming families remain open.
