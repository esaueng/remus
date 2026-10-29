# Exact rational Bézier-chain projection onto planar faces

Owner: P-Class 7.4. Tier: core — extends projection over live source/face
handles, creates output edge/vertex handles, and records the existing scoped
journal barrier participating in checkpoints and transactions.

This increment admits open, full-native-trim rational Bézier chains represented
as one clamped NURBS curve: degree 1–8, at most eight nonempty spans, positive
finite weights, and every interior knot with multiplicity exactly the degree.
Each span uses its stored degree-plus-one control window. No knot insertion,
fit, approximation, or sampled acceptance is used. Shared endpoint controls
prove positional continuity; one-sided regularity is required on every span.
A tangent discontinuity at a join is allowed and is preserved. The output is
one edge with the original knots, weights, degree and native interval.

The existing forward-ray, strictly interior whole-control-hull, convex planar
face, endpoint, roundoff, optional 2D, budget and atomic-publication contracts
remain in force. Every span's chart receives the existing finite-width and
reciprocal headroom test; optional 2D error uses the worst span allowance.
General B-splines requiring knot insertion, discontinuous joins, closed chains,
partial trims and other targets remain refused. The prior single-span positive
family and existing sketch-plane behavior retain their contracts.

Native and direct/legacy/V2 witnesses compare both sides of joins and complete
span interiors against independent weighted Bernstein and ray/plane equations.
Refusals cover unsupported multiplicity, degeneracy in a later span, a later
hull excursion, unsafe charts, span/control budgets and atomic rollback.
