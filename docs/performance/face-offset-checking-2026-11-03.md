# Face-offset checking and history replay

Owner: [PERF-M08](../kernel-maturity/roadmap.md#perf-m08), with classification reuse from PERF-O06.
Tier: core. Face moves and fillet guards resolve live session topology; the
bounded scratch mesh memo belongs to the kernel's cache scope.

Repeated face moves and fillet attempts revisit properties and classification
of unchanged geometry. These changes reduce that repeated work while retaining
the existing checks and fallbacks. They do not eliminate dependent history
replay or establish a real-time completion claim.

## Changes and invariants

- Determine periodic winding from sampled edge curves, including open rational
  arcs. Endpoint-only polygons can mistake a notched cylinder wall for a full
  turn and unnecessarily decline qualified surface integration.
- When knot-aligned quadrature refuses its cell budget, try the transposed
  nonperiodic parameterization. Swap trims and quadrature axes and reverse the
  normal sign. Existing admitted integrations, periodic refusals and cell caps
  retain their behavior. Dense spline strips can then use the existing
  certified quadrature instead of falling back to mesh volume.
- Use the existing exact-content ray-crossing memo for batch classification
  as well as edge relations. The existing NURBS span prefilter and its
  admission and payoff gates retain their behavior.
- Reuse identical complete robust-classification meshes only within a blend
  cascade, under a 4 MiB storage estimate including exact-content keys.
  Geometry, native handle order, pcurves, tolerances and deflection participate
  in the key. Scope exit releases the memo; failed builds are not cached.
- Accelerate winding classification with bounded cluster solid-angle
  intervals. Only an interval wholly beyond the existing decisive threshold
  can return a verdict. Uncertain intervals run the original complete triangle
  sum and its original analytic fallback. This approximates an intermediate
  integral with an error bound; it does not approximate or change geometry.

No material probe count, validation gate, tolerance, meshing policy or existing
approximation permission is relaxed.

## Regression evidence

Independent regressions cover a rational 270-degree cylinder rim, true periodic
winding, dense anisotropic planar NURBS integration with a hole and orientation
changes, cached/uncached point classification
through edits and eviction, mesh memo lifetime/content invalidation, and solid
angle intervals at multiple scales, placements and orientations. The latter
compare intervals and decisive verdicts against the original triangle sum and
retain an ambiguous open-shell case.

The affected unit suites, paired WASM package smoke tests, installed-tarball
consumer regressions and the consumer parity corpus exercise the existing
geometry contracts. Consumer workflow latency qualification remains open;
this document makes no end-to-end speedup claim.

The paired committed packages use the artifact from CI run `41624660041`,
whose merge tree is identical to source head
`9a1241dfcda58fc00128f7e26ad74a1efebf113a`, including the shared analytic-trim
endpoint repair from #1504. Earlier local Mac and hosted Linux builds passed
the same package runtime checks but produced different WASM bytes while
generated bindings agreed, so the distribution uses the hosted artifact.
The artifact ZIP SHA-256 is
`868059ac5bd5d81574812864188a99e66c56530c657e2a488407629f682cd361`.
The unchanged package integrity gate rebuilds and compares the hosted output.
