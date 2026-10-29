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

The native maps must have enough floating-point precision for the unchanged
linear tolerance. A mathematically exact trace on the chart
`[1e16, 1e16 + 16]` can round a mapped interior u coordinate to the wrong
surface station. The adapter therefore includes affine-map roundoff in the
published distance bounds, and refuses a poorly conditioned chart instead of
claiming a small geometric residual.

The existing clipper then proves whole-span residuals on both original
surfaces, whole-patch regularity and whole-section transversality before
reading material intervals. It certifies physical boundary pcurves against
their original trimmed 3D edges and vertex endpoints. A control-row candidate
that only matches the second surface at endpoints and midpoint is rejected
by the full-span homogeneous Bernstein bound.

For a caller evaluating the returned native maps at a parameter inside the
source domain as
`f = (t - source_lo) / (source_hi - source_lo)` and
`u = target_lo + (target_hi - target_lo) * f`, outward-rounded one-ULP
allowances cover both rounded span subtractions, source numerator/division,
target multiplication/addition, and the surface's subsequent native-u
normalization. Source curve and fixed-v normalization have their own
allowances. A normalized-u derivative bound uses the complete homogeneous
Bernstein numerator `X' W - X W'` divided by the positive weight hull squared;
the ruled patch's normalized-v derivative is bounded by the difference of
its two end curves. These whole-section bounds convert parameter roundoff
to model units before it is added to each original residual certificate.
Nonfinite bounds or totals above the existing linear tolerance refuse.
The proposed v is rounded before construction, and the original residual
proof uses that actual stored v; no unrepresentable proposed station is
silently substituted into the certificate.

The distance certificate also includes absolute error in the original
`NurbsCurve::evaluate` and `NurbsSurface::evaluate` implementations; a
relative-origin carrier proof alone does not cover their world-coordinate
contractions. The bounded evaluators have at most 128 arithmetic operations
per coordinate: the ordinary surface path uses at most 43 basis operations,
16 scale products and 41 contraction/division operations. The optional
exponent-scaled tiny-term path uses at most 43 basis and 75 position
operations; binary exponent manipulation is exact. An FMA counts as two
ordinary operations, and the curve paths are shorter.

With unit roundoff `e = f64::EPSILON / 2`, define
`g = 128*e / (1-128*e)`, `k = max_weight/min_weight` and `c = g*k*k`.
The component requires `c < 1/4`. For each coordinate with maximum absolute
stored control coordinate `M`, it bounds the evaluator error by
`2*c*M/(1-c) + 128*min_subnormal*k*k`, using outward arithmetic throughout.
Positive denominators and the weight ratio bound basis/weighted-sum
conditioning; the factor two covers numerator and denominator perturbations.
The separate subnormal floor covers coordinate products lost to underflow.
The Euclidean norms for the source and original target are added to every
reported geometric/map residual bound.

This proof qualifies native spans and raw weights only within
`[64*MIN_POSITIVE, MAX/64]`, and requires
`128*k*max_absolute_control_coordinate <= MAX/4`. These admission checks keep
recurrence divisions, raw products and ordinary contraction intermediates
finite and normally scaled; small basis/coordinate terms and the existing
tiny-term branch remain covered by the error allowance.
The tensor basis has a maximum term at least approximately `1/8`; hence
the term scale is at least approximately `min_weight/8`. The lower raw-weight
guard keeps that scale normal. A raw subnormal product loss divided by this
scale is therefore bounded at machine-epsilon scale and is covered by
`g*k*k`; it does not require an unbounded reciprocal-minimum-weight floor.
The separate subnormal floor concerns final coordinate contraction losses.
Common weight factors `1e-300` and `1e300` remain qualified. Subnormal/raw-MAX
weights, unresolved conditioning and nonfinite allowances refuse before the clipper
evaluates physical boundary events. The kernel's general math evaluators can
handle additional regimes that this first operational certificate excludes.

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
Native interval and event endpoints at source fractions zero and one retain
the exact stored knot endpoints. Interior affine reconstruction stays within
that original domain; a rounded span cannot move a retained endpoint outside
the source curve or shorten its authoritative end.
Different normalized crossings must also remain strictly ordered in stored
native source parameters, even if their material intervals are discarded.
Multiple original coedge certificates at the same crossing remain distinct.
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

Roundoff regressions compare the actual returned-map/original-surface
evaluations against the reported bound at non-dyadic source fractions,
preserve admission of conditioned offset charts, and refuse coarse large
anchors in both operand orders. They also pin nonfinite allowance refusal,
native-v proposal quantization and independent polynomial/rational speed
oracles. Non-dyadic source domains also pin exact endpoint/event identity
where recomposing a rounded span would overshoot or undershoot the source.
In the retained pre-fix witness a valid degree-one pair reported
`6.36e-15` while an interior native trace missed by `0.0625`; it now refuses
with the unchanged `1e-7` linear tolerance.
Well-conditioned native charts with physical translation `1e16` also exposed
an evaluator residual of `4` against a `1.42e-13` pre-fix bound. The operational
allowance now refuses those charts and translation `1e8`, while the `1e6`
translation witness remains admitted with its actual evaluations covered.
Weight-scale admissions/refusals and discarded-crossing collapse have
independent native regressions.

Production FF integration still needs verified traces and event provenance
carried through `RawCurve`, `IntersectionCurveDS`, paves and split boundary
uses; curved-chart arrangement, cross-face shared attachment and rollback
integration remain separate work. This component adds neither general NURBS
Booleans nor a public browser capability claim. That later milestone still
owes native/direct/batch/installed-package geometry and refusal qualification.
