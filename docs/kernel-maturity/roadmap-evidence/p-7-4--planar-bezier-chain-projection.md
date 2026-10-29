---
row: 7.4
---

Tier: core — extends live source/face handle projection and publishes journaled
output edges. The exact planar projection family now admits an open clamped
rational Bézier chain with degree 1–8, at most eight spans, positive weights,
and every interior knot having multiplicity exactly the degree. Every span
uses its stored control window without knot insertion. Each one-sided image
must have a whole-interval regularity certificate; joins preserve positional
continuity and may have different tangents. The original degree, knot chart,
weights and one-edge representation are retained. General B-splines, partial
trims, discontinuous joins and unresolved charts still refuse atomically.

The existing forward-ray, strict whole-hull containment, convex hole-free
native-plane target, source endpoint, optional 2D roundoff, caller control
budget and checkpoint/history contracts remain active. Span/control hard
limits bound only this newly admitted family. No cancellation argument is
added. The native and generated direct/legacy/V2 witnesses compare complete
span interiors and both sides of knots against independent rational Bernstein
and affine ray/plane oracles. See the [bounded contract](../../design/planar-bezier-chain-projection.md).
