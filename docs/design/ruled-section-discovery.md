# Bounded shared ruled-section discovery

P-Class 2.5 prerequisite. Tier: native for this read-only Rust API increment.
The returned traces and physical boundary events refer to the caller's original
topology handles. Future production FF integration belongs in core when it
constructs the live session's Boolean topology. This increment adds no browser
binding, changes no Boolean dispatch, assembles no solid, and does not complete
the P-Class 2.5 milestone.

`remus_algo::discover_ruled_face_section(&Topology, [FaceId; 2],
&OperationContext)` discovers **one** shared constant-v section of two rational
ruled patches and clips that source section against both physical face regions.
Previously the isolated clipper required a section and both native traces to
be supplied by its caller. The new entry point is public and reachable without
enabling an unqualified curved-chart GFA path.

## Qualified input and construction

Both faces must retain positive-weight, clamped, single-span rational Bezier
carriers with matching degree 1–3 in u and degree one in v. Both must be
nonperiodic, with equal weights along each ruling. A pair of corresponding
control-row lines proposes finite in-chart v fractions. Direction scaling
avoids forming a squared model-space determinant; outward-rounded 2x2 minors
must exclude zero before division. This solve is a proposal, never a proof.

The first carrier's control rows at the actual stored native v coordinate
construct the rational section without a fitted replacement. Its knot vector
and source domain are the first face's unchanged native u domain. The second
face's native u domain maps affinely and in increasing order onto that source;
both trace maps are returned explicitly, including differing knot scales.

The existing clipper then proves whole-span residuals on both original
surfaces, whole-patch regularity and whole-section transversality before
reading material intervals. It certifies physical boundary pcurves against
their original trimmed 3D edges and vertex endpoints. A control-row candidate
that only matches the second surface at endpoints and midpoint is rejected
by the full-span homogeneous Bernstein bound.

The trim domain remains one rectangular outer loop and disjoint strictly
interior rectangular holes, using original axis-aligned Line2D pcurves and
Line or rational Bezier 3D boundary edges. No coordinate weld, sampled polygon,
projection, repair or parameter normalization creates a crossing identity.

## Result and refusal contract

`RuledFaceSection` returns the source curve, both original native trace maps
and `RuledSectionClipping`: every retained source interval, every original
trim event including discarded material, and both conservative full-source
residual bounds. Each crossing retains its original face/loop/coedge/edge,
traversal, native source enclosure, pcurve coordinate and edge coordinate.
The records describe input geometry at call time; callers must not treat
edited output records or stale handles as authorization for GFA assembly.

This is deliberately **not an exhaustive face-face intersector**. It neither
proves the absence of other components nor classifies an unsupported face pair
as disjoint. Empty retained intervals mean that this discovered section
misses the two physical regions. An absent or unproved constant-v candidate
refuses with `RuledSectionError`, rather than returning empty geometry.

The API takes immutable topology and context references, returns only after
complete validation and never allocates topology or writes history. It keeps
the clipper's existing segment budget and cancellation semantics. Candidate
discovery examines at most four paired control rows and three fixed minors
per row. Periodic/pole/tangent/overlap cases, unsupported charts, missing
authority and unresolved event order retain typed refusal. No tolerance,
work allowance or production guard changes.

## Evidence and integration boundary

`curved_section_clip/discovery/tests.rs` exercises 144 cells: rational circular
and polynomial parabolic loci, scales 1e-3/1/1e3, rigid placement, reversed
physical uses, both operand orders and three differing native-u chart pairs.
Independent closed-form loci check the complete source, while rectangular
material expectations pin both retained intervals and per-use events.
Additional tests cover nonmidpoint traces and native-v remapping, a false
cubic candidate matching endpoints/midpoint, missing authority, parallel
rulings, exact state preservation, budget/cancellation and trimmed-away
sections. Independent linear and cubic loci also qualify degrees one and
three across scale and physical-use reversal. Existing clipper qualification
remains in place.

Production FF integration still needs verified traces and event provenance
carried through `RawCurve`, `IntersectionCurveDS`, paves and split boundary
uses; curved-chart arrangement, cross-face shared attachment and rollback
integration remain separate work. This component adds neither general NURBS
Booleans nor a public browser capability claim. That later milestone still
owes native/direct/batch/installed-package geometry and refusal qualification.
