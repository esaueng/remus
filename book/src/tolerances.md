# Tolerances and Robustness

brepkit conventionally interprets lengths as millimetres and angles as
radians. Scalars are unitless in the Rust and JavaScript type systems, so the
kernel performs no automatic conversion. A model supplied in metres also
requires metre-scaled deflections and linear tolerances.

## Default model

`brepkit_math::tolerance::Tolerance` separates linear, angular, relative, and
parametric comparisons. Prefer it over a raw epsilon when an algorithm already
accepts a tolerance. Exact topological identity is different from geometric
coincidence: compare typed handles directly, but compare coordinates with the
appropriate tolerance.

Do not use `==` for geometric floating-point values. Typical checks are:

- point coincidence: Euclidean distance below a linear tolerance;
- direction parallelism: a dimensionless dot or cross-product threshold;
- parameter convergence: a parameter-space threshold, not millimetres;
- scale-aware values: absolute plus relative comparison.

Named local constants are acceptable for algorithmic guards when their unit
and concept are documented. Do not reuse one numeric value across unrelated
concepts merely because the current defaults match.

## Choosing values

Start from the default tolerance for ordinary millimetre-scale CAD. Tightening
it can expose nearly coincident topology; widening it can merge intentional
small features. Keep the tolerance materially smaller than the smallest
feature that must survive.

Tessellation deflection is not a topology tolerance. It controls mesh
approximation and measurement paths that use that mesh. Exporting with a
coarser deflection changes mesh formats but does not modify the exact B-Rep.

## Diagnosing robustness failures

1. Confirm model units, coordinate magnitudes, and every supplied tolerance.
2. Validate both operands before the operation.
3. Check for zero-length edges, zero-area faces, or non-positive NURBS weights.
4. Retry only with a justified, recorded tolerance change.
5. Reduce the model to the smallest reproducible pair of operands and preserve
   it with arena serialization or a STEP fixture.

Avoid repeated blind tolerance widening. It may turn an explicit failure into
plausible but topologically incorrect output.
