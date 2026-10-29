---
row: PERF-M04
---
Tier: native diagnostics. Feature-only counters separate VE, EE and FF
Cartesian pair visits from EE crossing-solver entries and FF AABB survivors;
existing projection, face-copy, splitter and ray-work counters remain intact.
The bounded 13-label GFA inventory covers store import/export, VV/VE/EE/VF/EF/FF,
coplanar processing, resolution, two-operand build/assembly and combined N-way
build. A supplied monotonic host clock opts into diagnostic durations; ordinary
builds have no counters or clock reads. Clock-disabled instrumented builds still
pay counter overhead and are labeled separately from production timings.
Counters are caller-thread-local, not an aggregate of parallel workers; nested
durations are inclusive. Drains/reset invalidate active old scopes, and tests
cover nested scopes, disabled/reconfigured clocks and invalid clock values.
The operations wrappers separately count general GFA calls, enabled store-local
attempts/successes, successful drilled-hole shortcuts and N-way calls. Requested
locality alone is not route evidence. A `scaling_` operations contract runs in
the existing feature-enabled complexity suite and pins pair visits, rejected
versus surviving work and phase coverage on separated/overlapping boxes. This
adds measurement infrastructure, not a candidate-generation optimization.

The source-bound controls in
`docs/performance/complex-boolean-baseline-2026-10-28.md` quantify clock-disabled
counter overhead separately. The 144-hole diagnostic records 7104 VE visits
with zero projections, 5328 EE visits with zero survivors and 900 FF visits
with eight survivors. Its inclusive timings point toward reconstruction,
assembly and store copies for the next fixture-specific investigation;
Cartesian visit counts alone do not rank elapsed-time opportunities.
