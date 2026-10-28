# Packaged WASM parameter-rebuild baseline — 2026-10-28

Owner: [PERF-M08](../kernel-maturity/roadmap.md#perf-m08). The unchanged
[maintained driver](../../scripts/perf/parameter-rebuild/README.md) establishes
a source-matched installed-package baseline for the real 31-feature holder
recipe. Both packages produce identical qualified output. The timing
comparison is flat and supports no recipe-speedup claim: this recipe does
not call the pattern APIs optimized in this milestone.

## Source, package and workload identity

| Variant | Immutable source | Generated package version | Kernel WASM SHA-256 |
| --- | --- | --- | --- |
| baseline | `70f675bea4aebd35b0a36d59eaec89118c278ef6` | 2026.1.83 | `a32699eaac02f46a61663f977ed3b5c3c7d5f44fe21c9dd43ea7c48e59918b91` |
| candidate | `a82e1a27749c8ddf29c94797f6ad4ef9a0baceda` | 2026.1.84 | `9aeb7d48af124ed8a32263e54397638c781bbc0a4f7cbe336b6c05029d841b89` |

Both sources passed their canonical `cargo xtask wasm-build` smoke and
installed-tarball consumer suites before these captures. Each measured kernel
was copied from an isolated npm-installed tarball and every file matched the
source package, tarball and installed bytes. The
[receipt](evidence/packaged-parameter-2026-10-28/receipt.json) binds both build
logs, source/package proofs, all package files, harnesses and original capture
hashes. The driver's source-SHA argument remains explicitly externally
supplied; it is not the provenance proof on its own.

The candidate measurement binds source `a82e1a27749c8ddf29c94797f6ad4ef9a0baceda`,
before the delivery PR's later test-only proof adjustment. These timings are
attributed to that immutable source and its installed package.

All 31 geometry features run on every tick: restore one empty checkpoint,
recreate the 17 fixed features and rebuild all 14 dependent features. The
height stays 58 mm; opening widths are 47, 47.02, 47.04, 47.06, 47.08 and
47.10 mm. The fixture's exact arena inputs replace original STEP translation,
opening recognition and text separation. No suffix-cache mode, store-local
setting override, consumer expression/history engine, worker, renderer or
browser presentation runs. Display/measurement policies remain .02/.35/.08;
the original [fixture provenance and license](../../scripts/perf/parameter-rebuild/NOTICE.md)
are unchanged.

## Sampling and timing boundaries

Runtime: v24.22.1, AMD Ryzen 9 5900XT 16-Core Processor. Preview and completion each
use eight fresh processes in A–B–B–A / B–A–A–B order. Each process runs six
new-width ticks with two warmups, yielding four retained ticks per process,
16 retained ticks per variant/mode, and 96 total ticks. All cold and warmup
samples are preserved.

Measured Node processes have CPU31 affinity. This task's own checks/builds
exclude CPUs15/31. Other host work was active, and exclusive ownership of
CPU31 or its physical sibling was not established. These are local baseline
observations; the small pools establish no tail-confidence claim. At n=16,
the empirical nearest-rank p95 is the largest retained sample.

Node `require` reads/compiles/instantiates the WASM module. That module load,
`new BrepKernel()` and empty-checkpoint setup are outside all tick timings.
Their separate p50 observations across eight processes per variant are:

| Variant | Node require / WASM initialization (ms) | Kernel constructor (ms) | Empty-checkpoint setup (ms) |
| --- | ---: | ---: | ---: |
| baseline | 60.140 | 0.546 | 0.148 |
| candidate | 59.777 | 0.556 | 0.147 |

A **cold tick** is the first full 31-feature replay after those setup steps;
it is not module instantiation or a cold application launch. Preview ends
at grouped-mesh transfer; completion additionally reads volume and complete
feature records for all eight solids. `stageSum` adds the driver intervals,
including in-block JavaScript, prefix validation and prefix serialization.
`observedWall` includes between-stage memory probes and ends before the final
memory probe and all post-timing quality/arena reads. Neither interval is an
isolated native function timing or input-to-frame latency.

Times are milliseconds. Cold p50 has n=4 per row; warm columns have n=16.

| Mode | Variant | Cold tick stage-sum p50 | Warm stage sum p50 / p95 | Warm observed wall p50 / p95 |
| --- | --- | ---: | ---: | ---: |
| preview | baseline | 2875.08 | 270.13 / 393.94 | 275.12 / 396.51 |
| preview | candidate | 2859.14 | 274.00 / 398.93 | 278.78 / 405.96 |
| completion | baseline | 3646.43 | 426.42 / 601.22 | 431.91 / 605.83 |
| completion | candidate | 3658.78 | 425.14 / 602.45 | 431.80 / 606.29 |

Completion's retained per-stage p50 intervals are:

| Stage | Baseline (ms) | Candidate (ms) |
| --- | ---: | ---: |
| restore | 0.06 | 0.06 |
| prefix | 107.95 | 108.27 |
| rebuild | 127.98 | 129.72 |
| validate | 1.85 | 1.84 |
| mesh | 30.10 | 29.48 |
| transfer | 0.18 | 0.18 |
| volume | 17.96 | 17.85 |
| recognize | 141.59 | 140.39 |

Stage medians need not sum to a tick median. Prefix/rebuild intervals include
several operations and JavaScript, so they are attribution targets rather
than proof of a particular internal CPU bottleneck.

## Recorded memory snapshots

These values come from the driver's stage snapshots: allocated/retired arena
slots, WASM linear memory, process RSS, JS heap, external bytes and array-buffer
bytes. Sampled maxima are not true temporary-allocation peaks. RSS includes
Node, fixture/transfer buffers, diagnostics and GC effects; it cannot be
attributed solely to the kernel. All stage snapshots and signed deltas remain
in the portable samples.

MiB means 1,048,576 bytes. The last column is the largest recorded RSS
observation across the four processes for that row.

| Mode | Variant | Final allocated / retired slots | WASM linear memory after setup → last tick (MiB) | Maximum sampled process RSS (MiB) |
| --- | --- | ---: | ---: | ---: |
| preview | baseline | 78,000 / 65,000 | 1.44 → 71.06 | 430.98 |
| preview | candidate | 78,000 / 65,000 | 1.44 → 71.06 | 427.64 |
| completion | baseline | 78,000 / 65,000 | 1.44 → 91.31 | 398.45 |
| completion | candidate | 78,000 / 65,000 | 1.44 → 91.31 | 428.58 |

Every process retains one checkpoint. Observed allocated slots progress
13,000, 26,000, …, 78,000; retired slots progress 0, 13,000, …, 65,000.
These six-tick runs preserve retirement tombstones and do not establish a
longer-session memory bound or reclamation behavior. No after-`free` memory
reading or allocator peak was measured.

## Exact comparison and independent qualification

All 96 ticks / 768 output-solid instances pass the unchanged strict validation,
finite/watertight/oriented mesh, opening envelope, material, mounting-bore and
single-checkpoint gates. The existing comparison helpers pass for all 32
preview/completion process pairs and all 16 cross-variant completion process
pairs. Complete arena/mesh hashes, captured volume Float64 bits and complete
feature records agree. This checks the captured histories; it does not assert
bit identity for a different independent query history.

The unchanged independent verifier passes on each package and matches all 48
captured records for that package at all six widths. It also checks reverse
and fresh-owner reconstruction, full/suffix arena identity, all 42 retained
NURBS carriers, the two 5 mm bores and lettering-volume invariance. The largest
closed-form floor-volume-delta residual is
0.0005611901614201997 mm³ under its existing 0.002 mm³ diagnostic quadrature/trim
bound. Independent volume re-query history differs from captured readings by
at most 2.1100277081131935e-10 mm³; these residuals are recorded separately
without weakening any operation tolerance or paired-output assertion.

The [96 compact sample rows](evidence/packaged-parameter-2026-10-28/samples.jsonl)
retain every original record field, including cold/warmup samples, timings,
features and memory telemetry. The [16 process rows](evidence/packaged-parameter-2026-10-28/processes.jsonl)
retain all original metadata and summaries. Together they reconstruct every
original driver output byte-for-byte using the encoding in the receipt; all
16 reconstructed SHA-256 values match the original local raw files.
[Baseline](evidence/packaged-parameter-2026-10-28/baseline-independent-audit.json)
and [candidate](evidence/packaged-parameter-2026-10-28/candidate-independent-audit.json)
audits retain the independent correctness evidence. No runtime implementation,
quality policy, cache setting or CI gate changes are introduced by this report.
