---
row: PERF-V01
---
Bounded WASM cylinder orientation integration reuse: consecutive samples at
the same finite `u` share the original cross product and its length within
one volume-only face integration. The ordinary and cylinder loops expand
from the same quadrature definition, keeping the accumulator free of cache
fields and preserving the ordinary loop's direct sample calls.
Native, area-only and full-property requests use the ordinary evaluator.
Point/partial evaluation, quadrature positions, weights, order, trim decisions,
accumulation expressions, validators and tolerances are preserved. Independent
fresh-evaluation comparisons cover all 14 contribution fields and flux bits,
trimmed holes, both orientations, translated/tilted carriers and orders 1–12.
The opt-in Preview Performance workflow builds and qualifies matched baseline
and candidate packages, captures complete kernel preview calls in A–B–B–A /
B–A–A–B processes, runs the existing full-edit output comparison and retains native
new-edit orientation estimates. Those artifacts qualify each measured source
revision; general per-operation prepared quadrature sharing remains open.
