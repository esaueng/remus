# Holder preview topology-work contract — 2026-10-27

Tier: native (qualification tooling; no kernel or package payload change).
Owner: [P-Class 8.2](../kernel-maturity/roadmap.md#p-8-2), with the
[B28](../kernel-maturity/roadmap.md#b28) holder workflow.

The maintained preview driver now enforces a deterministic retained-arena
work limit. An edit may append at most one selected source body's topology
after the first checkpoint copy. Restore allocates no topology slots, and
relaxed validation, display meshing, transfer and untimed diagnostic reads
neither allocate nor retire topology slots. The live inventory cannot exceed
the checkpoint, its one document copy and one edited body. Missing retirement
therefore fails even when each append stays within budget.

These are observable topology-slot limits, not CPU, allocation-byte, peak-RSS
or latency limits. Temporary copies freed inside a call and owners outside the
exposed memory inventory are not measured. Retirement tombstones remain
reserved so old handles cannot alias future entities. The change does not
reuse slots, migrate owners, reduce retained memory or optimize geometry.

## Package and capture identity

The observations use the committed **2026.1.81** kernel/translator pair at
distribution commit `831741cc35c95acd58481ae7943003b97b67ed18`. All 20 package
files were compared byte-for-byte with that commit; the
[file receipt](evidence/holder-preview-work-contract-2026-10-27/package-verification.json)
retains sizes and SHA-256 values. Its publisher attributes the build to
`1853ad33cee018b01dfe135ccb3415e9503ceb62`. This tooling change did not
independently rebuild that distribution or establish source-to-binary identity.

The fixture is the existing Shapr3D holder STEP, SHA-256
`2403b4c9868cbb56292a09ec2f2d37baca16cc4b789d11e97ff8df3f1e46bef8`, with
160 faces and 42 NURBS carriers. Node **24.22.1** ran serial processes on CPU
31 of an AMD Ryzen 9 5900XT. Other task builds were excluded by an exclusive
lock. External host activity, thermal effects and system noise were not fully
characterized; these are descriptive observations, not a quiet-host speedup
qualification. No comparison with the older package or browser frame rate is
claimed.

The three short timing captures use the unchanged driver at `831741cc3`,
SHA-256 `477c8696f977f890fc784384442f161ee301b5b2decbd5b11804405682a24ce3`.
The longer allocation capture uses this PR's driver, SHA-256
`ceca7f4e42fc3717321d993c0385bf6d5be8faca980c624ec90868edc74ddefe`.
The raw reports retain wrapper, WASM, fixture, quality-helper, runtime and
affinity identity. Portable copies omit absolute package/fixture paths.

## Short preview observations

Each process makes 24 prepared preview requests. The first two are excluded
from warmed pools; p95 is an empirical nearest-rank observation over 22 samples.
Display policy stays at deflection 0.02 and angle 0.35, and cache defaults stay
unchanged. Every tick passes relaxed validation, the existing finite/indexed
watertight/manifold display-mesh checks, the 160-face check and the requested
cap-position check. These checks do not certify every geometric property.

| Requests / unrelated bodies | Cold kernel tick, ms | Warm p50 / p95 / max, ms | Warm first / revisited requests |
| --- | ---: | ---: | ---: |
| Unique / 0 | 1297.54 | 38.89 / 56.32 / 163.30 | 22 / 0 |
| Repeated and reversed / 0 | 1287.73 | 25.74 / 51.47 / 55.93 | 9 / 13 |
| Unique / 200 | 1283.18 | 38.06 / 71.98 / 162.62 | 22 / 0 |

`kernel tick` sums restore, move, relaxed validation and display meshing. It
excludes transfer, consumer orchestration, worker queues and presentation.
The raw reports retain transfer and observed-wall readings separately. A
revisited value is a previously requested offset, not evidence of a cache hit.

Raw captures: [unique / 0](evidence/holder-preview-work-contract-2026-10-27/unique-bg0.json),
[repeated / 0](evidence/holder-preview-work-contract-2026-10-27/repeated-bg0.json),
[unique / 200](evidence/holder-preview-work-contract-2026-10-27/unique-bg200.json).
Their original driver predates the work gate; the
[separate rescore](evidence/holder-preview-work-contract-2026-10-27/rescored-work.json)
uses the independently observed 1,906-slot source inventory and the retained
raw stage snapshots. It does not rewrite those captures as new-driver runs.

## Work and longer-session evidence

The new driver's 200-tick repeated/reversed run with 200 background bodies
passes the work gate at every tick:

| Observable | Recorded limit / outcome |
| --- | --- |
| Selected source inventory | 1,906 slots |
| Initial document | 14,706 slots |
| First move append | 16,612 slots, including the one-time document copy |
| Every later move append | 1,906 slots |
| Restore / validation / mesh / transfer append | 0 slots |
| Maximum live inventory | 31,318 slots: two document copies plus one edited holder |
| Final allocated / retired | 410,612 / 379,294 slots |
| WASM linear memory, before / after | 8,060,928 / 104,857,600 bytes |

The [raw long-session capture](evidence/holder-preview-work-contract-2026-10-27/long-session-bg200.json)
excludes all timings. It demonstrates bounded live inventory and linear
tombstone growth for this workflow, not a plateau in total arena slots or
WASM memory. Existing `copyDocument` remains an explicit fresh-owner operation;
consumer session migration is separate work.

The existing completion driver also passes one warmup and two retained edits
on this pair, with its strict-validation, quantity, carrier census and mesh
checks. Its [portable receipt](evidence/holder-preview-work-contract-2026-10-27/completion-smoke.json)
retains stage readings and output hashes; generated arena and mesh payloads
are reproducible and are not copied into this evidence directory. Full
31-feature parameter-rebuild [preview](evidence/holder-preview-work-contract-2026-10-27/parameter-preview.json)
and [completion](evidence/holder-preview-work-contract-2026-10-27/parameter-completion.json)
each pass four requests, including the reverse/revisited width. These are
correctness smoke checks with all timings excluded, not updated completion
or parameter latency baselines.

## Enforcement and remaining scope

Normal fresh-package smoke and installed-tarball qualification run the same
preview driver for three correctness-only ticks at zero and 200 background
bodies. The matched Preview Performance comparator independently scores raw
slot snapshots and the final inventory, rather than trusting `passed: true`.
Injected tests reject repeated whole-document copies, read-stage allocations,
missing retirement, counter tampering, missing snapshots and refusal masking.
Local validation passes all 11 driver tests, eight independent-capture tests,
the complete WASM smoke suite and the complete installed-tarball consumer
suite, including their existing geometry/refusal/history matrices. Installed
qualification used the unchanged tarball script with subprocess permission
after the sandbox blocked its initial npm spawn before testing.

Run the maintained driver and its tests as described in the
[driver guide](../../scripts/perf/face-drag-session/README.md). For the longer
allocation witness, use 200 ticks, repeated requests, 200 background bodies
and excluded timing status. Run package qualification through
`scripts/test-wasm-smoke.mjs` and `scripts/test-wasm-tarball-consumer.mjs`.

P-Class 8.2 remains **Partial**. Browser/worker latency, portable CPU-work
counters, machine-calibrated elapsed-time budgets, other operation families,
temporary allocation peaks and consumer document migration remain open.
