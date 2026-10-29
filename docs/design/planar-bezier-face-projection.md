# Exact rational Bézier projection onto planar faces

Owner: P-Class 7.4. Tier: core — extends `projectCurveOntoFace`, which reads
live source/face handles, creates caller-owned edge/vertex handles, records a
scoped journal barrier, and participates in checkpoints and transactions.

This bounded increment projects one positive-weight, clamped, single-span
open rational Bézier carrier onto a native plane along forward parallel rays.
The emitted NURBS retains the degree, knots, weights and full increasing native
source interval, with a finite outward-enclosed width and finite reciprocal. Only control points undergo the affine plane projection;
there is no fit, clipping, degree reduction or replacement by sampled lines.
Optional plane-frame output remains an exact rational 2D representation.
That optional output additionally requires each retained weight to be at least
four times machine epsilon: the current 2D evaluator uses a raw-denominator
cutoff. Unsafe retained weights refuse; they are never silently rescaled. The
normalized-weight 3D evaluator can still accept tiny positive weights.
The optional view also needs an absolute roundoff allowance for its actual
basis/homogeneous evaluator, positive-weight denominator and world
reconstruction. Its actual frame-axis magnitudes and native chart enter that
allowance. Interval control/frame representation error plus this allowance
must fit the same target-scale band. A distant in-plane frame cannot inflate
that band or publish a numerically unresolved view.

The qualified source degree is 1 through 8 (2 through 9 control points). The
existing `maxControlPoints` option also limits this new exact family. The target
must be a closed strictly convex outer polygon of 3 through 32 straight edges with no
inner wires. Its bounded authoritative Loop/Coedge list must agree exactly
with the compatibility wire before that polygon can authorize an image. Whole-control-hull certificates must establish forward rays,
strict image containment, finite representable mapped controls and a regular
image. Boundary contact, partial coverage, holes, nonconvex/curved boundaries,
general multispan/unclamped carriers, nonpositive weights and unresolved arithmetic
refuse before publication. Existing line/conic and sketch-plane routes retain
their contracts. Solid first-hit projection is outside this increment.

Target planarity, optional-frame and image-plane certificates use the target
extent alone. Source distance or projected-away extent cannot loosen them.
Source endpoints separately use their own vertex tolerance capped by the
existing linear tolerance.
The image enclosure uses the original input ray and stored plane coefficients,
including an outward-enclosed normal/direction denominator. Both the true-image
interval hull and emitted controls must lie strictly inside the polygon. Their
coordinate-error sum must fit the target-scale band; positive rational weights
lift those control bounds to the entire curve. The dominant plane coordinate
is recovered algebraically, without fitting or changing the source chart.

The existing projection entry points have no operation-context or cancellation
token argument. This increment makes no cancellation claim. Its resource
controls bound only the newly supported source/face family; they do not change
the existing approximate-fit budget or imply a general projection work budget.

Acceptance uses independent weighted-Bernstein evaluations and affine ray/plane
equations, including nonuniform weights, native knot charts, scale, rigid
placement, reversed face/wire uses and optional 2D output. Negative controls pin
whole-hull containment (including an excursion between sampled stations),
budget boundaries, degeneracy and whole-call topology/journal/checkpoint
ownership restoration. The native API and generated direct/legacy/V2 APIs
must pass the same geometry oracles. Existing projection oracles remain active.

`ProjectCurveError` gains `ResourceLimit` and `ExactBezierDomainUnsupported`.
Rust callers with exhaustive matches must add those arms. JS success schemas
are unchanged; V2 carries `project-curve:<code>` in `details.kernelCode`, with
resource/unsupported categories for these new refusals. Direct error prefixes
retain the kernel code, while legacy batch retains its plain-message contract.

The [stored Bézier-chain extension](planar-bezier-chain-projection.md) admits
a bounded multispan representation without knot insertion; the single-span
qualification above remains unchanged.
