# RFC 0001: OperationContext

Status: accepted; surface-surface intersection budgets and the public boolean
pipeline are integrated. This RFC governs how the context grows and how
further paths migrate onto it.

## Problem

Tolerances and work limits are scattered through the kernel as module-local
constants and inline literals (marching step caps, queue sizes, Newton
iteration limits, ad-hoc epsilons). Consequences:

- numeric and resource behavior is not part of any caller-visible contract;
- budgets cannot be tuned per operation, observed, or tested from outside;
- the same conceptual policy is duplicated with drifting values;
- future policy (cancellation, fallback rules, diagnostics) has no carrier.

## Design

`remus_math::context::OperationContext` is the single explicit carrier:

- `tolerance: Tolerance` — the existing linear/angular/relative model.
- `budgets: WorkBudgets` — hard upper bounds for iterative and exploratory
  work. v1 fields: `march_steps`, `queue_size`, `segments`,
  `branches_per_direction`.

It lives in L0 `math` so every layer can consume it without new workspace
dependencies. Both structs are `#[non_exhaustive]` with `new()` +
`with_*` builders, so fields are added additively without breaking callers.

Ground rules (from the kernel-maturity operation contract):

1. **Additive entry points.** A migrated path gains a `*_with_context`
   function; the existing function delegates with `OperationContext::new()`.
   Existing signatures never change.
2. **Defaults reproduce legacy behavior exactly.** `WorkBudgets::new()`
   encodes the constants the path used before migration; a differential test
   pins default-context output against the legacy entry point.
3. **A field exists only when consumed.** No speculative budget fields: a
   field is added in the same change that threads it into an algorithm.
4. **Budgets bound, never spin.** Exhausting a budget terminates the work at
   the bound. Reporting exhaustion as a typed nonconvergence outcome (rather
   than returning the bounded partial result) arrives with the intersection
   result model (backlog Issue 10); until then the with-context behavior at
   the bound matches the legacy behavior at its constant.

## Constant classification

Constants encountered during migration are classified before replacement —
only policy values move into the context; mathematical constants stay put:

| Class | Example | Disposition |
| --- | --- | --- |
| Physical-space tolerance | vertex weld distances | `context.tolerance.linear` |
| Angular tolerance | parallelism thresholds | `context.tolerance.angular` |
| Parameter-space tolerance | SSI refinement `1e-6` in seeding | context, as a derived parameter-space policy (future field; unchanged today) |
| Numerical floor | `1e-12` degeneracy guards | stays local, named, documented |
| Convergence threshold | Newton step acceptance | stays local unless caller-meaningful |
| Resource budget | march steps, queue, segments, branches, Newton iteration caps | `context.budgets` |

## Landed in this slice

- `math::context` with the two structs, builders, defaults, unit tests.
- `intersect_nurbs_nurbs_with_context`: the four marching/exploration
  budgets threaded through seeding and marching; module constants
  `MAX_QUEUE_SIZE` / `MAX_SEGMENTS` / `MAX_BRANCHES_PER_DIRECTION` deleted
  (their values live only in `WorkBudgets::new()`). Differential and
  tiny-budget regression tests.
- `algo::gfa::boolean_with_context`: context entry point for the GFA
  pipeline. The caller's tolerance reaches pave filling, face splitting,
  classification, assembly, and validation; NURBS face-face intersection
  also consumes the context's marching/exploration budgets. `boolean` now
  routes through it.
- `operations::boolean_with_context`: carries the same context through
  analytic shortcuts, GFA, mesh fallback, and recursive/multi-component
  handling. Public boolean entry points are transactional, including failed
  post-processing.
- `operations::BooleanOptions`: every field is consumed. `tolerance` and
  `deflection` map into an `OperationContext`; `unify_faces` and
  `heal_after_boolean` control explicit post-processing. Unification runs in
  a nested transaction and is discarded if it would invalidate an otherwise
  valid boolean result; healing failures propagate and roll back the whole
  operation.

## Migration queue (dependency order)

1. `MAX_NEWTON_ITER` (`refine_ssi_point` and the line/plane/curve-surface
   Newton loops) — one `max_newton_iterations` budget field, threaded
   through the ~7 call sites in one change.
2. ~~Pave-filler propagation~~ — **landed**: NURBS face-face intersection
   receives the context's existing work budgets. Other iterative limits gain
   context fields only alongside a real consumer, per ground rule 3.
3. Parameter-space tolerance policy (replacing seeding's local `1e-6`),
   coordinated with the intersection result model (Issue 10).
4. Cancellation state and diagnostics sink — added when the first consumer
   (long-running boolean or corpus runner) lands, not before.
5. ~~Fallback policy~~ — **landed** (Issue 11): `FallbackPolicy` on the
   context (`ExactOnly` / `AllowApproximate{budget}` /
   `ApproximateOnly{budget}`, default reproducing the legacy mesh
   deflection), consumed by `boolean_with_context`, which discloses the
   result quality (`BooleanOutcome`) and refuses degradation under
   `ExactOnly` with the typed `ExactOnlyUnattainable` error.

Repro bundles reserve a `context` field for serializing this type; that
field unlocks when the context becomes replayable policy (serde support and
a stable JSON shape), which is deliberately not part of v1.
