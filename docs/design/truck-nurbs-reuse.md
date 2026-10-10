# Opt-in Truck-derived NURBS reduction and cubic fitting

This change adds geometry-only NURBS surface knot removal, curve/surface
simplification, and adaptive cubic fitting. All are opt-in. Remus's ordinary
modeling, fitting, import and healing defaults retain their existing behavior.
The source license and adaptation boundary are recorded in
[Truck reuse provenance](../production-readiness/truck-reuse-provenance.md).
The existing [P-Class 7.6 owner](../kernel-maturity/roadmap.md#p-7-6) retains
the wider curve construction, fairing, degree reduction and continuity scope.

## Supported domain

The operations accept Remus's finite, clamped, positive-weight NURBS geometry
with exactly nondecreasing knot vectors and degrees 1–12. Interior knot
multiplicity must not exceed the degree. Control points and weights must be
finite, weights strictly positive, and the active parameter domain finite
and nonzero. Zero-degree, periodic/unclamped, discontinuous, nonpositive-weight,
or malformed input receives a typed refusal. Degree 12 is a bounded admission
limit for this implementation, not an extension of general kernel maturity.

Knot removal acts on one interior occurrence in U or V and preserves the
remaining knot values and full native parameter domain. End clamps cannot be
removed. The supplied knot is a value, not an index: it must exactly match an
interior value, and the first matching occurrence is selected. A surface
candidate is reconstructed across every affected control strip and must pass
the full rectangular-domain certificate. The input is borrowed;
success returns new geometry, and failure leaves the input unchanged.

Cubic fitting additionally requires a C1 source: interior multiplicity must
be strictly below degree. A degree-one source therefore cannot contain an
interior knot in this slice. Source corners, ambiguous or unavailable finite
derivatives, and unresolved subdivisions refuse. The output is a piecewise
polynomial cubic NURBS with the original native parameter domain. Its
derivative tolerance measures derivative with respect to that parameter,
so changing the parameter scale changes the derivative units.
Finite input alone does not guarantee representable endpoint derivatives:
extreme weight ratios or minimum-subnormal parameter spans can make their
homogeneous arithmetic unrepresentable. Checked endpoint evaluation returns
a typed numerical-bound refusal for those inputs rather than publishing a
nonfinite cubic or continuing through a failed derivative construction.

## Numerical authority

`crates/math/src/nurbs/reuse_bounds.rs` is independently authored. It builds
outward-rounded interval Bernstein bounds on every nonzero common knot span.
Cox–de Boor evaluation produces enclosing homogeneous polynomial control
coefficients. Rational cross-products enclose the coordinate differences;
strictly positive denominator lower bounds and the Bernstein convex hull
produce a Euclidean distance upper bound. Cubic fitting uses the analogous
quotient derivative construction to bound the first-derivative difference.

These bounds cover each whole interval or surface cell, including its ends.
They include rounding uncertainty from the certificate arithmetic. Samples
can serve as independent regression witnesses, but cannot admit a candidate.
Floating-point input coordinates are the represented source geometry; this
does not certify an unknown physical shape or an upstream evaluation error.
At an output cubic piece join, the derivative certificate covers both
one-sided derivatives against the C1 source derivative. The assembled cubic
uses interior knot multiplicity three: its rounded control points do not
guarantee exact C1 continuity. A small tangent jump is permitted only when
each side independently stays within the reported derivative bound.

Structural equality has an explicit zero-bound path. Other certificates
usually have a positive roundoff floor, including algebraically redundant
knots. An `ExactOnly` context accepts only a zero certificate; reduction can
return the unchanged input if no candidate qualifies. Tolerance-permitted
reduction and all cubic fitting disclose approximation. Cubic fitting refuses
`ExactOnly`, including when a sampled example appears geometrically exact.

The certificate is a conservative sufficient test. A mathematically valid
candidate may refuse when the bound is too loose, the denominator cannot be
proved positive, interval arithmetic overflows or loses a useful positivity
bound, or the work limit is reached. Those refusals are part of the supported
contract; they do not trigger a sampled or implicit approximation fallback.

## Reduction strategy and accumulated error

`crates/math/src/nurbs/reduction.rs` adapts Truck's U/V homogeneous
inverse-insertion recurrence and descending repeated-removal strategy into
Remus types. It replaces local `near` acceptance with the certificate above.
Curve candidates use Remus's existing `curve_knot_remove` implementation.
Candidate construction normalizes placement, parameter range and weights
where needed for conditioning, then restores the source parameterization.
Curve simplification refuses relative knot spacing at or below `1e-15`,
because its existing candidate generator uses a fixed small-interval
threshold. An entire tiny native parameter domain remains supported when its
relative knot spacing resolves; this refusal concerns spacing within it.

Simplification visits knots deterministically from the back, with a fixed
U-then-V order for surfaces, and repeats until no permitted removal remains.
Every accepted candidate is checked against the **original** geometry, not
only the preceding candidate. Thus the returned `deviation_bound` covers the
complete reduction sequence without silently accumulating the per-step
tolerance. The result includes the number of removed knots and accounted
work. An unchanged result reports no removal and zero deviation.

`ReductionOptions` supplies a positive model-space tolerance and a work cap.
The operation context also controls cancellation and the permitted
approximation budget. The effective tolerance cannot exceed that budget.
Options do not widen a global tolerance or override `ExactOnly`.

## Cubic fitting strategy

`crates/math/src/nurbs/cubic_fit.rs` adapts Truck's endpoint/tangent cubic
Hermite construction and adaptive bisection orchestration. It does not reuse
Truck's hash-selected interior sample acceptance or its trailing optimization
pass. Each source knot interval starts with an endpoint/tangent cubic. A
segment is accepted only when the whole-interval position and derivative
bounds satisfy the requested limits; otherwise it is bisected within the
explicit depth, segment and work caps. There is no partial-success output on
cancellation or refusal. The assembled output is certified again against
the original source before publication, including its parameter derivatives.

`CubicFitOptions` supplies position and parameter-derivative tolerances,
maximum work, maximum subdivision depth and maximum output segments.
The context's approximation budget caps the position tolerance; its resource
limits additionally cap subdivision depth and segment count. Success reports
the output geometry, maximum position and derivative bounds across all
segments, segment count, accounted work, and approximation disclosure.

## Public API and topology boundary

The math API exposes `surface_knot_remove_u`, `surface_knot_remove_v`,
`simplify_curve`, `simplify_surface` and `fit_cubic_curve`. Native `Model`
methods are `surface_knot_remove_u`, `surface_knot_remove_v`,
`simplify_nurbs_curve`, `simplify_nurbs_surface` and `fit_cubic_curve`; they
use raw `NurbsCurve`/`NurbsSurface` values and the model's operation context.
WASM direct and batch methods are named
`surfaceKnotRemoveU`, `surfaceKnotRemoveV`, `simplifyNurbsCurve`,
`simplifyNurbsSurface` and `fitCubicCurve`.

The WASM methods accept an existing NURBS edge or face; analytic carriers
receive a typed refusal rather than an implicit conversion. Direct calls
take their options as a JSON object encoded in a string and return a JSON
string envelope. Batch calls take the same options object under `options`
and return the existing batch-v2 envelope.

| WASM operation | Direct arguments | Batch arguments |
| --- | --- | --- |
| `surfaceKnotRemoveU` / `surfaceKnotRemoveV` | `face, knot, optionsJson` | `face`, `knot`, `options` |
| `simplifyNurbsSurface` | `face, optionsJson` | `face`, `options` |
| `simplifyNurbsCurve` / `fitCubicCurve` | `edge, optionsJson` | `edge`, `options` |

Reduction options are `tolerance` (default `1e-7`), `maxWork` (default
`1_000_000`), and `exactOnly` (default `false`). Cubic fitting requires explicit
`positionTolerance` and `derivativeTolerance`, with optional `maxWork`
(default `1_000_000`), `maxDepth` (default `12`), `maxSegments` (default `1024`),
and `exactOnly` (default `false`; `true` refuses fitting). Tolerances must be
finite and positive. WASM resource options must be positive integers, at
most `10_000_000` work, depth `16`, and `4096` segments. Direct options JSON
is limited to 4096 bytes; unknown option fields refuse.

For example, given an existing NURBS face handle:

```js
const options = { tolerance: 1e-6, maxWork: 1_000_000 };
const result = JSON.parse(
  kernel.simplifyNurbsSurface(face, JSON.stringify(options)),
);
if (result.status === "error") {
  throw new Error(`${result.code}: ${JSON.stringify(result.details)}`);
}
const { geometry, removedKnots, deviationBound } = result.value;
```

Successful direct calls return `{status: "ok", value: ...}`. The value contains
`geometry`, `geometryOnly: true`, `workUsed`, `approximate` and `quality`.
Reductions also report `removedKnots` and `deviationBound`; fitting reports
`positionBound`, `derivativeBound` and `segments`. Positive bounds are disclosed
with `approximate: true` and `quality: "certified_within_tolerance"`.
Unchanged/zero-bound reduction uses `quality: "unchanged_or_zero_bound"`.
Direct refusals return `{status: "error", code, category, details, value: null}`;
batch-v2 refusals retain the same native diagnostic classification.

The returned geometry includes degree(s), control points, weights, knots and
complete native domain(s). These operations allocate no topology and do not replace
the source edge or face. The certified domain is the carrier's complete
native domain, not the edge trim or the material left by a face's outer and
hole loops. The source trims, pcurves, vertices, face loops, journal and
persistent references remain unchanged.

A caller wishing to replace a trimmed CAD carrier must separately preserve
and qualify boundary geometry, per-use pcurves and downstream topology.
The geometry-only certificate grants no authority to make that replacement.
No default import cleanup, fillet simplification, automatic fitting swap or
general degree-reduction feature is introduced by these entry points.

## Qualification boundary

The new tests use independently constructed geometry, including
insertion/removal round trips, rational weights, asymmetric surface grids,
repeated knots, scale/placement and parameter-range variants, independent
analytic cubic/derivative checks, and typed refusal/resource limits.
The complete local verification commands and results belong in the pull
request, pinned to its actual source head. Source tests do not establish
the provenance of a previously generated or published WASM package.
The distribution builder in `xtask/src/wasm.rs` copies `LICENSE-APACHE` and
the current root `NOTICE` into new kernel and translator npm packages,
includes both in the npm file list, and verifies their bytes against the
source metadata. This change does not regenerate the frozen package files.
The `remus-math` Cargo source package includes byte-identical crate-local
license and notice files; the provenance checker rejects stale copies.

No general performance improvement, exact imported-shape equivalence,
CAD topology validity, or broader Truck/OCCT parity is claimed. Reduced
control counts can be measured without implying that evaluation or complete
modeling workflows become faster. Any future automatic integration needs
its own boundary-preservation and equivalent-quality performance evidence.

## Initial local benchmark evidence

The nine-case Criterion benchmark in
[`crates/math/benches/truck_reuse.rs`](../../crates/math/benches/truck_reuse.rs)
was run locally with Rust 1.96.0 on an AMD EPYC 7763 VM exposing five CPU
cores. Each case used ten samples, one second of warmup and two seconds of
measurement. The intervals below are Criterion's 95% time estimates for
one complete benchmark iteration; they are not per-point timings.
The evaluation and simplification measurements used the initial implementation
working tree; cubic fitting was measured again after the checked endpoint
evaluator repair. These runs have no immutable release or final-head artifact
pin and establish no production performance claim.

The fixtures are a rational quarter-circle and its quarter-cylinder sweep.
Eight inserted interior knots refine the curve from 3 to 11 control points
and the surface from 6 to 110. Simplification returns 3 and 6 control points,
respectively, under a whole-domain position budget of `1e-8`. The harness
asserts removal counts and certificate bounds before timing and checks
equivalence at every evaluation parameter used by these workloads.

| Workload | 95% iteration time |
| --- | ---: |
| 128 curve evaluations, original 3 control points | 7.071–7.114 µs |
| 128 curve evaluations, refined 11 control points | 7.177–7.206 µs |
| 128 curve evaluations, simplified 3 control points | 6.992–7.301 µs |
| 1024 surface evaluations, original 6 control points | 101.36–105.20 µs |
| 1024 surface evaluations, refined 110 control points | 110.34–131.40 µs |
| 1024 surface evaluations, simplified 6 control points | 112.28–119.24 µs |
| Certified curve simplification, 11 → 3 control points | 464.55–496.28 µs |
| Certified surface simplification, 110 → 6 control points | 21.122–23.015 ms |
| Quarter-circle cubic fitting, position `1e-4`, derivative `1e-3` | 686.93–723.69 µs |

The demonstrated result is a smaller certified representation. Evaluation
timings are noisy and partly overlap; these measurements establish neither
a general evaluation speedup nor a benefit for an entire CAD workflow.
Certification has a visible construction cost, particularly for the surface
fixture. Concurrent build activity and the VM environment also limit how
these measurements transfer to a dedicated production workload.

Reproduce on the reviewed source with:

```sh
cargo bench -p remus-math --bench truck_reuse
```
