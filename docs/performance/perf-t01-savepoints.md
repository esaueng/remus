# PERF-T01 / B29: unchanged-entry savepoint slice

The coordinator removes duplicate full snapshots of unchanged transaction entry
states. It retains genuine savepoints after outer mutations. This is a bounded
PERF-T01 subset, not completion of B29 or mutation-local transactions.

Baseline: `a3d85e799196315522ac2f08138ae6fe6daa27c2`.
Candidate runtime source: `d94839e9c058b6e356d4c1815f07dccecb8d94da`.
Later evidence/test/package commits do not change the measured runtime source.
See the [pre-implementation contract and public trace](../design/perf-t01-savepoints.md)
and [reproduction instructions](../../scripts/performance/transactions/README.md).

## Measurement

Seven fresh processes per cell, alternating baseline/candidate order. Both runs
use Rust 1.96.0 release (LTO, one codegen unit); WASM uses SIMD and wasm-opt 117
`-Oz --enable-simd`, wasm-bindgen 0.2.126 and Node 24.14.0. The shared host has a
Ryzen 9 5900XT and 32 GiB RAM; background validation was running. Setup and checks
are excluded from the operation timer. These are cold-operation samples, not a
steady-state JIT or browser responsiveness claim. No tail-latency gate is inferred
from seven samples.

The native workload calls the public `operations::boolean` with 0/1/4/8 enclosing
transactions. WASM uses real compiled modules through direct `fuse` and
`executeBatch`, with and without a retained checkpoint. The uninstrumented
candidate is installed from the actual `2.130.54` kernel tarball; baseline is a
private Node-only tarball of rebuilt baseline bindings, not a stale committed
binary. The two production packages passed `cargo xtask wasm-build`, including
installed-tarball consumer regressions. Nothing was published.

Instrumentation counts every full `Topology::clone` and requested heap bytes
inside that clone, including nested geometry/pcurve payloads and map storage.
These are clone allocation bytes, not serialization length or exact CPU memcpy
traffic. The forwarding allocator and injected exports exist only in disposable
measurement archives. The tables combine **uninstrumented median latency** with
**diagnostic clone and live-heap counters**; MB means 1,000,000 bytes.

### GFA fixture

Two unit boxes, one rotated 45 degrees about its center around Z, have analytic
union volume `4 - 2*sqrt(2)`. Unrelated unit boxes remain in the document. Each
sample validates that volume after the timed operation.

| Entry / unrelated boxes | Production ms, before → after | Clones | Clone allocation MB | Peak live heap MB |
| --- | ---: | ---: | ---: | ---: |
| Native / 0 | 4.046 → 3.915 | 2 → 1 | 0.036 → 0.018 | 0.411 → 0.394 |
| Native / 100 | 5.090 → 4.709 | 2 → 1 | 1.866 → 0.933 | 3.629 → 2.697 |
| Native / 1000 | 14.933 → 9.291 | 2 → 1 | 17.842 → 8.921 | 29.602 → 20.681 |
| WASM direct / 0 | 73.176 → 73.898 | 2 → 1 | 0.026 → 0.013 | 0.320 → 0.308 |
| WASM direct / 100 | 74.475 → 72.348 | 2 → 1 | 1.352 → 0.676 | 2.678 → 2.003 |
| WASM direct / 1000 | 80.942 → 74.283 | 2 → 1 | 13.026 → 6.513 | 21.753 → 15.240 |
| WASM batch / 0 | 79.961 → 78.563 | 3 → 1 | 0.039 → 0.013 | 0.334 → 0.309 |
| WASM batch / 100 | 75.350 → 75.193 | 3 → 1 | 2.028 → 0.676 | 3.355 → 2.004 |
| WASM batch / 1000 | 86.172 → 81.303 | 3 → 1 | 19.539 → 6.513 | 28.267 → 15.241 |

### Nesting and checkpoint retention

| GFA at 1,000 unrelated boxes | Production ms, before → after | Clones | Clone allocation MB | Peak live heap MB |
| --- | ---: | ---: | ---: | ---: |
| Native + 8 unchanged outer scopes | 56.939 → 9.907 | 10 → 1 | 89.210 → 8.921 | 100.970 → 20.681 |
| WASM batch + retained checkpoint | 93.187 → 83.408 | 4 → 2 | 26.052 → 13.026 | 40.322 → 27.296 |

At native enclosing depths 0/1/4/8, baseline GFA clones are 2/3/6/10; the candidate
uses one while the entry state is unchanged. Mutation invalidates sharing:
changed-state nested savepoints still deep-copy, as the pointer-identity and
caught-failure tests require. No claim is made that arbitrary nesting is free.

For production GFA batch at 1,000 unrelated boxes, reserved WASM linear memory
fell from 30.47 MB to 17.43 MB (46.01 MB to 32.64 MB with a checkpoint). Process
peak RSS fell from 211.44 MB to 196.37 MB, but includes V8, setup and validation;
it is not attributable solely to snapshots. Allocator peak is read before
validation. Linear memory is a high-water reservation, not live heap.

### Fast path and limits of the latency result

The translated-box fast path (union volume 1.5) needs only one native snapshot
already: at 1,000 unrelated boxes its production median is 5.488 → 5.519 ms.
With eight extra scopes it is 47.501 → 5.518 ms (nine clones → one). Production
WASM batch is 13.358 → 9.312 ms (two clones → one).

Small-document timings are mixed: fast WASM batch at 100 unrelated boxes is
8.369 → 9.161 ms. Cold GFA WASM time is dominated by more than snapshot copying;
its 1,000-box batch ranges overlap (83.070–93.980 ms baseline,
75.967–83.847 ms candidate). These data support the deterministic copy/memory
reduction and a workload-specific latency improvement, not uniformly faster
operations or a stable regression threshold.

Instrumentation itself matters: the 1,000-box native GFA diagnostic medians are
17.899/10.883 ms, versus 14.933/9.291 ms without instrumentation (roughly 20%/17%
higher). Diagnostic WASM medians have overlapping ranges with production. Use
production timing for latency conclusions, and diagnostic counters for copying.

The remaining cost is explicitly O(document size): even the candidate fast edit
rises from about 0.065 ms at zero unrelated boxes to 5.519 ms at 1,000. The outer
snapshot still copies unrelated state, tombstones remain allocated, and genuine
changed-state savepoints/checkpoint first writes still copy. There is no arena
compaction, ID reuse, global cache, immutable-carrier sharing or wholesale COW
rewrite in this slice. Their prerequisites are recorded in the design.

## Fault-injection evidence

The native unit matrix compares coordinator behavior with the original
clone-and-restore implementation. It injects refusal after allocation, in-place
vertex edits, attributes, pcurve changes/boundary work, retirement and journal
recording. It checks all live arenas, loops/coedges, pcurve lookup contents,
attributes, journal entries/ordinal index and persistent-reference resolution;
a later journal begin checks that rollback did not introduce a mutation-tick gap.
Only intentionally monotonic allocation/journal high-water state is excluded from
equality. Separate tests require failed vertex/edge/loop/coedge and journal IDs
not to be reused. Existing handles survive failed retirement.

The matrix covers nested success, propagated failure, caught repeated inner
failure followed by outer success, outer failure after inner success, and
validation veto. Snapshot identity tests verify sharing before mutation and
independent savepoints after attribute-only, journal-only, entity and pcurve
writes, restoration and independent cloning. Topology remains Send + Sync.

Both revisions also pass real-operation injected faults after geometry/export
and after winding normalization: four native cases with three caught failures
each, and eight packaged-WASM combinations of fault point, fast/GFA fixture and
direct/batch entry, again with three repeated failures. The WASM matrix verifies
success–failure–success batch commits, full live topology/history/attribute state,
assembly/sketch/GCS/checkpoint/poison state, and failed solid handles remaining
stale after later allocation and checkpoint restore. The fault switch is one-shot
and is absent from the shipped package.

## Evidence and qualification boundary

- [Manifest](evidence/2026-09-26/perf-t01/manifest.json): source, fixture, hardware,
  toolchain, harness, native binary, module, binding and installed-tarball hashes.
- [Baseline provenance](evidence/2026-09-26/perf-t01/baseline-provenance.json) and
  [candidate provenance](evidence/2026-09-26/perf-t01/candidate-provenance.json):
  all 416 Cargo-reported local runtime source dependencies match independently
  reconstructed source archives plus the instrumentation recipe.
- [Diagnostic samples](evidence/2026-09-26/perf-t01/samples.jsonl) and
  [summary](evidence/2026-09-26/perf-t01/summary.json): 672 process samples.
- [Production samples](evidence/2026-09-26/perf-t01/production-samples.jsonl) and
  [summary](evidence/2026-09-26/perf-t01/production-summary.json): 504 process samples.
- [Baseline native faults](evidence/2026-09-26/perf-t01/baseline-native-faults.json),
  [candidate native faults](evidence/2026-09-26/perf-t01/candidate-native-faults.json),
  [baseline WASM faults](evidence/2026-09-26/perf-t01/baseline-wasm-faults.json),
  [candidate WASM faults](evidence/2026-09-26/perf-t01/candidate-wasm-faults.json).

Focused qualification passed: 375 topology tests, the WASM per-item/session
regression, strict workspace clippy, boundary/deterministic-hash/doc-path checks,
and the paired production package build/installed-consumer suite. The PR's
exact-head CI remains the integration gate for the full repository.

Only batch `fuse`, `cut`, and `intersect` join the coordinator in this PR. Other
WASM dispatch families retain their existing boundaries/policy. Warm steady-state
JIT, real browsers, large NURBS payloads, long-session tombstone growth and broader
session workloads remain PERF-M/W and B29 follow-up qualification; this does not
close those dependencies or the overall performance programme.
