# PERF-O06-TR: torus ray threshold work

Torus ray intersections now avoid computing a complex `hypot` when conservative
component bounds already decide its threshold comparison. Once one update rules
out convergence for a Durand–Kerner sweep, later convergence norms are skipped;
every polynomial evaluation and root update still runs. Coefficients, FMA,
tolerances, iteration/cycle policy, root filtering, Newton polish and face trim
membership retain their existing arithmetic.

## Attribution before the change

The [shared diagnostic evidence](evidence/perf-o06-ray-crossings/diagnostic-summary.json)
is pinned to main `d78230e122fb075ff0936608029943bce42e3fbf`. It captures the maintained
160-face holder workflow after a +X cap edit: display .02/.35, volume and
recognition .08, two warmups and two retained edits. Fresh local paired version
2026.1.61 was built from that source; the older committed version-60 artifacts
were not substituted for this baseline. The fresh kernel SHA256 is
`9e290c4e81c9c83d7f9b15e8af024c8f136e3aa71b2de266b6d60c24f4eadcce`.

Two named-module profiles, verified byte-identical to the shipping module after
removing only names, attribute 606/2,425 and 600/2,476 samples to torus rays
(24.2–25.0% inclusive). These profiles ran under contention; their durations are
excluded from latency claims. Inclusive categories overlap. The source-32
[earlier graph report](perf-o06-recognition-graph.md) remains attributed to its
own source and packages.

A separate observation-only overlay records the following work. Both retained
calls preserve the shipping control's complete feature JSON, Float64 volume,
arenas and display buffers: five snapshots and 25 artifact pairs per comparison.
The overlay and its hashes are retained in
[diagnostic-overlay.json](evidence/perf-o06-ray-crossings/diagnostic-overlay.json).
Its counters and durations are diagnostic, with no timing claims.

| work per retained recognition | edit 5.04 | edit 5.06 |
|---|---:|---:|
| torus carrier solves | 5,672 | 5,672 |
| quartic sweeps | 163,032 | 162,903 |
| polynomial evaluations | 652,128 | 651,612 |
| denominator `hypot` calls | 652,128 | 651,612 |
| convergence `hypot` calls | 652,128 | 651,612 |
| converged / cycle / budget exits | 4,034 / 1,069 / 569 | 4,034 / 1,068 / 570 |
| returned real roots / forward roots | 5,034 / 3,711 | 5,034 / 3,711 |
| trim-admitted / trim-rejected forward roots | 4 / 3,707 | 4 / 3,707 |
| calls returning no forward roots | 3,660 | 3,660 |
| exact repeated carrier + origin + direction inputs | 0 | 0 |

The existing baseline component test already decides every denominator
comparison. For convergence, 514,182/513,578 updates have a component at least
1e-14, and 114,887/114,952 have both components at most half that threshold.
The final implementation deliberately uses the wider *double-threshold* upper
band, so those baseline counts are not a count of final fast-path hits.

This evidence selects norm-threshold work rather than result memoization.
The large trim-rejection count supports investigating a separately certified
trim-support bound later; this change adds no geometric rejection or trim shortcut.

## Comparison equivalence

For finite components, let `m=max(|real|,|imaginary|)`. The complex norm lies
between `m` and `sqrt(2)*m`. At `m<=threshold/2` it is below the threshold; at
`m>=2*threshold` it is above. These wide bands keep rounding away from the
comparison. The middle band calls the original `hypot`. Unsupported threshold
scales and nonfinite components also call the original function. No squares
are formed in the bound, so subnormal components cannot underflow a comparison.

The two comparison methods preserve their distinct NaN behavior. A NaN norm
is neither below nor at least the threshold, while `hypot(infinity, NaN)` keeps
the original library result. The previous convergence maximum ignored NaN
norms. Testing whether any step norm reaches the unchanged threshold preserves
that rule and permits skipping later *norm checks* after a qualifying step.
All four iterate updates still execute in their original order. Consequently,
convergence, cycle state, root filters and polish see the same iterates.

## Regression coverage and qualification

The existing verbatim pre-cycle full-budget solver remains the independent
implementation reference: 3,600 rays include crossings, misses, grazes,
non-unit directions, placed tori, fixed cycles, longer cycles and exhausted
nonrepeating runs, comparing complete returned root bits.

New numerical controls compare the two threshold predicates to the original
`hypot` on adjacent threshold/band ULPs, deterministic IEEE bit patterns,
subnormals, signed zeros, overflow-scale finite values, NaNs and infinities.
Separate controls pin the original maximum's NaN suppression. Independent
Cartesian sections pin complete zero-, two- and four-root sets over six
scale/placement cells and three non-unit direction speeds, with geometric
support residuals checked independently.

The isolated Criterion target `torus_ray_crossings` measures three 64-ray
families: crossings, misses and near grazes. It uses 20 samples per family.
These carrier timings complement the maintained packaged holder workflow;
they do not establish a whole-recognition or browser speedup.

Native batch and packaged regressions retain periodic notched torus material
and hole probes, planar-boundary classification, complete edge-relation rows
and feature records across scale/placement and direct/batch query routes.
Quiet before/after timings and final immutable-head workspace/package
qualification are pending and will be recorded separately.

A preexisting curved-boundary classifier limitation remains open: on the
untouched baseline, the scale-.1 notched torus reports Outside at exact curved
boundary points corresponding to `(0,13,0)` and `(-10,0,3)`. The ignored
ready-repro retains that witness; this optimization does not claim to repair it.
Mathematical completeness outside the existing torus solver's supported domain
and actual browser presentation latency also remain open.
