# PERF-O05: lazy scratch allocation for pattern material-overlap probes

Date: 2026-10-28. Owner: [PERF-O05](../kernel-maturity/roadmap.md#perf-o05).
Tier: core, existing pattern operations. This is a bounded native optimization
of the existing material-overlap guard, not instance architecture or delayed
geometry materialization.

Linear, circular and grid patterns share `pattern::refuse_material_overlap`.
It already computes conservative body bounds, sorts its sweep and rejects
pairs with no positive overlap in all three axes. Previously it cloned the
whole topology before visiting any pair, including a completely disjoint or
merely touching pattern. It now creates one scratch topology on the first
surviving pair and reuses it for later material probes. The sweep, bounds,
exact-only boolean, volume threshold, typed refusals and outer transaction
remain unchanged.

## Matched native observations

The baseline is source `70f675bea4aebd35b0a36d59eaec89118c278ef6` plus the timing
example. The candidate is that same source plus only the pattern patch and
its tests, with the identical example. The compared production source files,
example and executable SHA-256 values are retained in the
[portable receipt](evidence/2026-10-28/perf-o05-pattern-summary.json).
The comparison isolates this pattern optimization from other changes that
may be delivered alongside it.

Each case generates a disjoint linear pattern of unit boxes spaced two units
apart, in a topology with 0, 100 or 1000 unrelated live boxes. The timer wraps
only `linear_pattern`: fixture creation, independent output checks, exact
arena serialization and teardown are outside it. Both executables use the
repository `profiling` profile and no optional features. Hardware is
AMD Ryzen 9 5900XT 16-Core Processor; compiler is rustc 1.96.0 (ac68faa20 2026-05-25).

For every case, eight fresh processes execute A–B–B–A / B–A–A–B, with two
warmups and eleven retained samples in each process. Each variant therefore
has 44 retained samples per case. Processes use CPU31 and
`RAYON_NUM_THREADS=1`; agent-owned validation excludes CPU31 and its physical
sibling CPU15. Other validation continues on the remaining cores, so this is
a reserved-core observation, not a whole-host-idle or cross-machine claim.

Times below are milliseconds, with empirical p50 / nearest-rank p95.

| Instances | Unrelated boxes | Baseline p50 / p95 | Candidate p50 / p95 | p50 reduction |
| ---: | ---: | ---: | ---: | ---: |
| 10 | 0 | 0.173 / 0.193 | 0.163 / 0.179 | 6.1% |
| 10 | 100 | 0.496 / 0.604 | 0.174 / 0.182 | 64.9% |
| 10 | 1000 | 3.705 / 4.235 | 0.196 / 0.209 | 94.7% |
| 100 | 0 | 2.627 / 2.802 | 2.354 / 2.642 | 10.4% |
| 100 | 100 | 3.141 / 3.416 | 2.708 / 3.103 | 13.8% |
| 100 | 1000 | 6.434 / 7.341 | 1.909 / 2.208 | 70.3% |
| 1000 | 0 | 22.911 / 26.005 | 21.176 / 24.043 | 7.6% |
| 1000 | 100 | 22.296 / 31.912 | 19.849 / 20.685 | 11.0% |
| 1000 | 1000 | 34.279 / 36.919 | 21.957 / 27.597 | 35.9% |

The observations are timed-loop samples, not Criterion estimates. Small tail
sample pools establish no p99 or tail-confidence claim. They characterize
this synthetic native pattern workload only; they do not measure complex
boolean speed, a full feature recipe, packaged WASM, browser latency or
reference-kernel performance.

## Correctness and reproduction

All 936 warmup/retained samples check output count, source-root ordering, total live
solid/face counts, exact instance bounds and unit volume against independent
unit-box expectations. Complete reachable arena bytes are identical across
all fresh samples within each executable. Baseline and candidate arenas also
compare byte-for-byte across all eight processes for each of the nine size
combinations; all nine
SHA-256 receipts are in the portable JSON. The receipt serializes
the pattern roots; unrelated bodies have their live counts checked but are
not part of the serialized result.

A test-only clone-site observer pins zero scratch copies for disjoint and
touching x/y/z patterns in a document with 128 unrelated boxes. Existing
scaled material-overlap tests additionally require one scratch copy and
retain measured overlap, typed refusal and rollback assertions. A rotated
thin-box witness whose conservative boxes overlap retains its existing
`ExactOnlyUnattainable` refusal and leaves the source topology unchanged.
The focused pattern suite passes 17/17; the example and library/tests pass
Clippy with warnings denied, and formatting/diff checks pass.

Build the same example at the baseline and candidate source revisions with
`cargo build --offline --profile profiling -p remus-operations --example
perf_pattern_copy`, using distinct saved executables. Its positional arguments
are instance count, unrelated box count, retained sample count, warmup count,
and an optional new arena-output path. For example:

```sh
set -C
taskset -c 31 env RAYON_NUM_THREADS=1 ./pattern-baseline 100 1000 11 2 baseline.arena.json > baseline.jsonl
taskset -c 31 env RAYON_NUM_THREADS=1 ./pattern-candidate 100 1000 11 2 candidate.arena.json > candidate.jsonl
cmp baseline.arena.json candidate.arena.json
```

Repeat all 10/100/1000 by 0/100/1000 combinations in the balanced process
order above and retain raw JSON samples. Output arenas must use new paths.
The local profiling build encountered a Rust/LLVM SIGSEGV; a serial build with
an increased compiler-thread stack succeeded without changing profile,
features or kernel source. Build recovery is separate from valid captures.

General shared-instance geometry, delayed materialization, operand-local
scratch staging for surviving pairs and packaged-WASM/browser timing
remain separate work. No modeling tolerance, validation obligation or
material-overlap acceptance threshold was relaxed.
