# Freeform chamfer construction

Master: M5 / 6.1 / 7.4 (curved blends).

Tier: core. Chamfers edit the live solid, shared topology and construction
history, so construction belongs in the eagerly loaded modeling module.

The first qualification family is a straight ridge between linear extrusions
of genuinely curved rational spline profiles, including a planar second
support. Both ends meet planar caps normal to the ridge. Setbacks are measured
from the ridge to the contact in its normal section, rather than as profile
arc lengths. Retained supports and cap boundary segments retain their original
geometry and parameter domains. The band is an exact plane. Existing analytic
constructions retain their distance conventions in this change.

Acceptance requires a changed, closed, oriented manifold; contacts on the
original supports; setbacks checked against an independent geometric
oracle; cap relimitation sharing the band's end edges; complete face history;
and transactional refusal of exhausted supports and intersecting requests.
Other curved ridges and surfaces that are not proven linear extrusions require a
separate qualified contact and relimitation construction. A sampled plane or
an unchanged valid input is not evidence of a successful chamfer.

The next qualification family is a curved, monotone spline rim between a
planar extrusion cap and its spline wall. Its wall contact retains the exact
translated profile. The planar contact is a disclosed, bounded approximation
of the inward parallel curve. Whole-interval, outward-rounded rational
Bernstein bounds must prove both setback and orthogonality to the source
tangent, the material-side branch and monotone contact derivatives. Curvature
collapse, unresolved bounds, intersections and non-normal end supports refuse.
The band is a ruled NURBS sheet sharing both complete contact carriers. A
curved planar contact must never be replaced by its endpoint chord during
trimming. Separate tests and independent solid comparisons qualify this
family before it is enabled.

Both families check every inner boundary against a conservative box of the
changed material. The existing geometry bounds cover complete analytic arcs
and positive rational spline hulls; unknown or overlapping bounds refuse.
Distant bores retain their original curved walls and cap loops. This adds a
dependency on the existing L1 geometry crate, with no new external package.

The bounded rim fit keeps the original parameter domain, uses at most 1364
cubic pieces (4093 controls), and charges at most 16 million coefficient and
construction work units. Its full-vector contact target is 1e-8 model units,
inside the unchanged 1e-7 modeling tolerance. Position-only isocurve proofs
avoid computing unused derivative bounds; active knot spans and weights are
located locally rather than rescanning each refined carrier.

Remaining qualification is explicit: arbitrary doubly curved supports,
non-normal end caps, closed spline rims, endpoint weight gauges that differ,
and interacting bands requiring a new corner patch still need additional
contact and relimitation constructions. No mesh or planar faceting fallback
is introduced, and these cases must continue to refuse transactionally.
