# Bounded curved NURBS section clipping (P-Class 2.5)

Status: isolated component, **not production GFA integration or general NURBS
booleans**. Baseline: `e25d0913ddae7ba6ac367eb3dcd39c628fb37e06`.
Implementation: `crates/algo/src/pave_filler/curved_section_clip.rs`.

## Existing credit and first loss of authority

The planar-NURBS recognition delivered by #396 and the section-seeding work
recorded under #416 remain completed work. Neither is replaced here.
`phase_ff.rs::FaceExtent::new` explicitly admits NURBS chart extents only when
`helpers::planar_nurbs_as_plane` succeeds. A genuinely curved NURBS patch
therefore returns no extent. `restrict_curves_to_faces` then returns the
carrier section without consuming the physical loops' pcurves. This is the
first missing trim-authority handoff, before splitter assembly.

`phase_ff/helper_oracle_tests.rs::curved_nurbs_trim_authority_stops_at_the_extent_handoff`
executes that current path on two curved NURBS patches: the full native section
`[-8, 24]` survives, although the outer trim and hole require `[-4, 4]` and
`[12, 20]`. The isolated component returns those two intervals. The existing
sampled polygon extent belongs to the planar path; enabling it indiscriminately
for curved patches would lose per-use event identity and is not this change.

## Supported input contract

- Two positive-weight, clamped, single-span rational NURBS patches, degree
  1–3 in u and degree 1 in v. Each control row has equal weights in its two
  v columns. Both charts are nonperiodic, with finite nonzero knot spans.
- A supplied single-span rational NURBS section of degree 1–3. Its full native
  parameter domain maps affinely, in increasing order, onto each patch's full
  native u domain, at a supplied constant v. Section discovery and recovery of
  these two traces are caller responsibilities.
- Whole-patch regularity and whole-section transversality must pass the
  Bernstein normal-hull sign tests. This is a sufficient test, so some regular
  transverse inputs can refuse. No pole, periodic lift, tangency, or overlap
  handling is implied.
- One rectangular outer trim and zero or more strictly interior, mutually
  disjoint rectangular holes. Every loop has four authoritative coedges with
  exact axis-aligned `Line2D` pcurves, starting at pcurve parameter zero and
  ending at a signed finite parameter. Endpoint sums must be exactly
  representable; rounded/disconnected charts refuse. Stored 3D boundaries are
  Lines or single-span rational NURBS curves with authoritative trims.
  Reversed uses map pcurve traversal to the reversed native edge interval.
- The number of boundary uses is at most `min(context.budgets.segments, 256)`
  across both faces. Cancellation is checked before algebra and between loops.
  Algebra degrees, pairwise hole checks, and event storage are bounded.

The component requires neither plane recognition nor analytic conversion. Tests
assert that both carriers remain `FaceSurface::Nurbs` and that the existing
planar recognizer declines them.

## Proof and result contract

Homogeneous control coordinates are translated by the first section control
point and weights normalized. Every arithmetic operation used for proof bounds
is outward rounded. Bernstein products and de Casteljau restriction retain the
original native parameters. For each coordinate the numerator of
`C(t) - S(u(t), v)` is bounded by its entire Bernstein control hull; positive
weight lower bounds give a distance bound in model units. Both original
surfaces must satisfy the unchanged operation linear tolerance over the entire
source span. Boundary pcurves and their original trimmed 3D edges receive the
same whole-span check, including stored vertex endpoints.

The ruled surface's homogeneous normal numerator is affine in v. A common
strict-sign projection of both v-end control hulls proves regularity over the
whole patch. A strict-sign projection of the cross product of the two section
normal hulls proves transversality over the whole section. Failure to establish
either bound is a typed refusal, not a sample-based acceptance.

Events solve the section's horizontal UV trace against the original vertical
pcurve use. Each retains face, loop, coedge, edge, traversal direction, section
parameter with an enclosing interval, pcurve parameter, native edge parameter,
and evaluated original-surface/boundary residuals. No sampled polygon or
proximity search creates an event or identifies two boundary uses.

All events partition the source span. Interior material tests intersect the
two outer rectangles and subtract every hole, retaining **all** intervals.
Interval arithmetic guards ambiguous ordering and interior classification.
Coincident cuts retain separate per-use records; equality is admitted only
for identical native chart coordinates/domains or common source endpoints.
Unresolved coincidence across differently parameterized charts refuses.
Boundary overlap/corner contact refuses. Ordinary transverse events at a source
endpoint retain their certificates; section boundaries are included in returned
closed intervals. An entirely excluded section returns no intervals.

The API takes `&Topology` and returns a result only after complete validation.
It allocates no topology, exposes no partial result, and cannot disturb rollback.
No weld band, fitting tolerance, existing refusal, or production dispatch changes.

## Witnesses and numerical evidence

`curved_section_clip/tests.rs` contains independent material expectations and
closed-form loci: a parabolic section `(u, 0, u²)` and a rational quarter-circle
section satisfying `x² + z² = 1`. The rational profiles sweep two different
ruled patches, so agreement between two executions of the clipping algorithm
is not the oracle.

The 72-case matrix covers both loci, scales 1e-3/1/1e3, origin and common rigid
placement, forward/reversed uses, and unit/shifted/tiny/large u knot domains.
Separate witnesses vary the source section's knot domain and use stored boundary
subtrims. Every matrix cell retains native intervals `[-4, 4]` and `[12, 20]`.
Other fixtures cover one outer interval, wholly outside material, wholly within
a hole, three intervals from two holes plus the partner trim, common source
endpoints, and a hole narrower than the linear tolerance that must not vanish.

Recorded focused run: maximum whole-section residual bound below 2e-11 and
maximum evaluated event-to-surface residual below 4e-12 across the matrix
(model units; scales span six orders of magnitude). Tests print retained
intervals, both full-span bounds, and event residuals with `--nocapture`.
Refusal witnesses include absent/false pcurves, absent 3D trims, periodic uses,
closed charts, a pole, unsupported v weights, tangent/coincident carriers,
boundary overlap, touching holes, unorderable near events, cancellation and
budget exhaustion. A cubic false section agreeing at both endpoints and the
midpoint still refuses on its interior residual bound.

## Exact integration dependency

`RawCurve` and `IntersectionCurveDS` currently store 3D carrier geometry and a
parameter range, but not both authoritative section traces or per-use trim-event
certificates. A future FF adapter must supply verified traces, consume each
retained interval without refitting, and carry its certificates through paves
and split boundary uses. An SSI fit that exceeds either original-surface
residual bound must refuse; increasing the weld tolerance is not a substitute.

The O2.3b arrangement component currently admits plane and pre-cut cylinder-strip
charts. Its curved NURBS chart adapter, source-interval edge emission, cross-face
event provenance, and GFA dispatch/rollback integration remain unqualified.
That owner must connect these results to `CurveSource`/`CurveUse` and preserve
coedge and section identity through `SplitSubFace` and solid assembly. This slice
does not edit arrangement code, retire splitter paths, or assemble a solid.
