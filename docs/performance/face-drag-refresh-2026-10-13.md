# Face-edit measurement and recognition refresh — 2026-10-13

On the committed paired WASM packages `2026.1.57`, the 160-face hammer-holder
edit spends a median 491.4 ms in volume and 345.2 ms in recognition in the
geometry-first schedule. Their combined share has a median of 90.09% of the
per-edit kernel/transfer pipeline. The first performance priority is the fine
curved-face quantity-mesh stage; the second is finer attribution of the area
and edge-relation work inside recognition graph construction.

The geometry/transfer subtotal is 92.4 ms. With volume scheduled before display,
the subtotal through display transfer is 586.7 ms. These are sums of synchronous
kernel API intervals, excluding construction/restoration and browser work.
They do not measure input-to-visible-frame latency.

[Compact evidence](evidence/face-drag-2026-10-13/refresh.json) retains all six
process results, warmups, phase events, package/fixture/harness hashes and all
qualification rows. Arena and binary mesh payloads are not embedded; their
filenames and hashes are retained. Machine-specific paths are replaced by the
portable `snapshot` location. Initial contended correctness preflights are
excluded. This is a current-package measurement, with no before/after speedup
claim against the [2026-10-11 qualification](face-drag-qualification-2026-10-11.md).

## Frozen inputs and method

The package checkout was main
`d94e03b7d28215c9b44dd6925a32a257729ca5c1`, verified immediately before the
quiet run. This identifies the committed artifacts being measured; it is not
a claim that a new build was made at that revision.

| Input | Version or SHA256 |
| --- | --- |
| Kernel / translator package versions | `2026.1.57` / `2026.1.57` |
| Kernel WASM, 9,412,128 bytes | `f2636194857e0cb48d6753fca0729c91f9e3abbb497a5f5f210b85b285f5864a` |
| Kernel Node wrapper | `415f8da6e547e228d8f5ef99ccd35a400941cdf61c418c15f799f2038ecf4dc8` |
| Translator WASM, 2,705,941 bytes | `e6e689baea1029087f6c4cfbd9522fa4a7cf30b60d1e1b9f2e0d6c5240ecb0d7` |
| Translator Node wrapper | `e90089cf522f2a785ce7c057393b5b053a1e725720f29a0b599e1e00cc329d37` |
| Public hammer-holder STEP fixture | `2403b4c9868cbb56292a09ec2f2d37baca16cc4b789d11e97ff8df3f1e46bef8` |
| Maintained WASM worker | `22103aafc45cbf1d23cf992b2861370e7223da32d0dc5960e1fc4439ab543b79` |
| Maintained comparison/qualification helper | `8e2fee1c751945d49c9970339bedf2dc5636bbf1ae4e0f5cc694ce8027493023` |

The run used Node 24.14.0 on Linux x86_64, AMD Ryzen 9 5900XT, pinned to CPU 0.
Competing repository builds/tests were paused for the measurements, which
started at 21:13:48 and completed before 21:15:00 UTC.
All 24 frozen input files were checked against their recorded hashes before
execution and again before tracing.

The [maintained workload](../../scripts/perf/filleted-face-edit/README.md)
selects the outward +X planar cap at x=48 mm, guarded by its approximately
1045.93 mm² area. Each process imports once, warms source queries and creates
each edit from the source's exact arena document. Module-local production
caches persist across instances; their defaults, including the shared 32 MiB
face-mesh retention budget with display priority, are retained. Distances are
`5 + iteration * 0.02` mm; display linear/angular controls are 0.02/0.35 and
volume/recognition request deflection 0.08. File writes and output diagnostics
are outside the timed intervals.

Four independent untraced processes ran serially: geometry-first then
volume-first in round 0, reversed in round 1. Each process has two warmups and
six retained edits: **12 retained edits per schedule**. Two additional diagnostic
processes each have two warmups and two retained edits. Diagnostic trace times
are excluded from the latency table. The traced worker only enables the
existing public phase-tracing API and drains its events; the exact
[client patch](evidence/face-drag-2026-10-13/phase-tracing.patch) is retained.

## Untraced stage durations

Milliseconds, median [observed minimum–maximum]. Small-sample ranges are
reported without p95/p99 claims. Each subtotal is calculated per edit before
summarizing, so its median need not equal the sum of component medians.

| Stage | Geometry first, n=12 | Volume first, n=12 |
| --- | ---: | ---: |
| Face move | 57.37 [48.83–89.52] | 56.74 [49.30–88.73] |
| Strict validation | 1.74 [1.43–1.88] | 1.83 [1.41–2.96] |
| Bounding box | 0.74 [0.73–1.95] | 0.75 [0.74–1.68] |
| Display tessellation | 29.10 [28.02–59.16] | 29.53 [28.36–60.45] |
| Bulk mesh transfer | 0.14 [0.13–0.20] | 1.18 [0.15–2.27] |
| Volume | 491.37 [487.28–496.41] | 495.19 [487.84–503.00] |
| Complete recognition | 345.24 [343.30–350.40] | 347.27 [342.65–351.97] |
| Kernel pipeline plus transfer | 931.46 [910.93–967.30] | 933.98 [915.60–975.66] |
| Pipeline including construction/restoration | 938.41 [915.26–973.40] | 941.03 [921.80–981.66] |
| Geometry plus transfer subtotal | 92.43 [79.59–124.03] | 93.40 [80.81–124.45] |
| Scheduled kernel subtotal through display transfer | 92.43 [79.59–124.03] | 586.71 [569.23–627.45] |

Geometry plus transfer sums move, validation, bbox, display meshing and bulk
transfer. Scheduled readiness includes the preceding volume call in the
volume-first schedule. Neither subtotal includes import, source warmup,
construction/restoration, application/worker scheduling, mesh installation,
GPU submission or browser presentation. Complete pipelines also exclude
application work. Per-edit volume-plus-recognition share has median
90.09% [87.09–91.26%] geometry-first and 89.99% [87.24–91.30%] volume-first.

## Separate diagnostic phase attribution

Two retained traced edits per schedule; these durations are diagnostic, with
nested phases overlapping. The `mesh.*` rows below are contained inside
`volume.total`, rather than the earlier/later display mesh call.

| Diagnostic phase, median ms | Geometry first | Volume first |
| --- | ---: | ---: |
| Volume total | 496.96 | 493.32 |
| Quantity mesh total | 455.81 | 451.71 |
| Quantity curved-face stage | 432.18 | 428.13 |
| Quantity boundary sampling | 6.42 | 7.39 |
| Quantity planar CDT | 3.91 | 2.12 |
| Quantity weld/dedup/gap fill | 13.20 | 13.57 |
| Volume content key | 0.19 | 0.19 |
| Recognition total | 357.00 | 348.88 |
| Recognition graph construction | 356.81 | 348.70 |
| Prepared edge-relation context, inside graph construction | 4.37 | 3.63 |
| Recognition candidate matching | 0.17 | 0.17 |

The [curved-face stage](https://github.com/esaueng/remus/blob/d94e03b7d28215c9b44dd6925a32a257729ca5c1/crates/operations/src/tessellate/solid.rs#L1522)
includes cache-key construction, replay checks and mesher dispatch; these
coarse traces do not isolate those costs or individual surface families.
[Recognition graph construction](https://github.com/esaueng/remus/blob/d94e03b7d28215c9b44dd6925a32a257729ca5c1/crates/operations/src/feature_recognition.rs#L188)
includes face areas, edge adjacency and relation evaluation. One prepared
classifier per call and shared normal samples
[already ship](https://github.com/esaueng/remus/blob/d94e03b7d28215c9b44dd6925a32a257729ca5c1/crates/operations/src/query.rs#L708).
The trace does not re-establish old sampled-profile percentages for these
remaining internals.

## What to prioritize

1. **Fine curved-face quantity meshes (PERF-D01/D03, V02).** Profile the
   measured stage's fresh meshing and cache/planning work, then accelerate it
   or prove compatible tessellation-plan reuse. The content-keyed face cache
   [already ships and is enabled by WASM](https://github.com/esaueng/remus/blob/d94e03b7d28215c9b44dd6925a32a257729ca5c1/crates/wasm/src/kernel.rs#L63).
   The [volume clamp](https://github.com/esaueng/remus/blob/d94e03b7d28215c9b44dd6925a32a257729ca5c1/crates/operations/src/measure/volume.rs#L1918)
   is `min(requested, max(bbox_diagonal * 5e-5, 1e-9))`. In the retained edits
   its value derived from retained bounding boxes changes approximately from
   0.005572324829 to 0.005575872039 mm.
   [Exact deflection bits belong to the face key](https://github.com/esaueng/remus/blob/d94e03b7d28215c9b44dd6925a32a257729ca5c1/crates/operations/src/tessellate/face_cache.rs#L632),
   so unchanged support faces do not establish compatible mesh requests.
   Preserve every requested/effective tolerance, purpose, boundary dependency
   and fresh quantity bit. Tolerance bucketing, a looser hidden clamp or a
   source-volume-plus-swept-prism seed would change the current contract.
   An optimization must qualify against fresh recomputation at each request,
   including byte-exact meshes where reuse is claimed, exact volume bits,
   holes/seams/cavities/rollback and bounded display-priority retention. Require
   repeatable improvement in this stage and the complete pipeline without
   regressing display readiness. No implementation is selected by this report.
2. **Recognition graph area/relation attribution (PERF-O06).** Split the
   remaining graph work into face-area, edge-normal/projection and ray-query
   costs before choosing a change. Prepared-context setup is only a few ms
   and candidate matching about 0.17 ms in this fixture; repeating the delivered
   preparation fix or prioritizing candidate bucketing is not supported by
   this measurement. Exact scalar NURBS-area memoization also already ships.
   Preserve complete feature records, area values, edge verdicts, ambiguity,
   hole/blend behavior and existing goldens. Other workloads may still justify
   candidate bucketing; this fixture does not rank it first.

The existing face-integral and whole-volume memos remain useful, but this
holder's first measurement after each width edit reaches quantity meshing.
The current trace does not support describing its roughly 0.5 s volume call
as predominantly analytic quadrature or key hashing. Broader sampled CPU and
allocation attribution, native coverage and actual browser presentation
remain open under PERF-M07/M08.

## Output qualification

Every snapshot keeps 160 faces, validates, moves the selected cap to
x=48+distance, increases volume, preserves the source and has a watertight
indexed display mesh. Qualification uses the maintained helper's complete
feature comparison, exact volume bits and mesh/discrete-geometry checks.

| Comparison population | Qualified | Arena bytes exact | Volume bits exact | Full feature records / JSON exact | All mesh arrays exact |
| --- | ---: | ---: | ---: | ---: | ---: |
| Independent repeat processes: source + 8 edits, both schedules | 18/18 | 18/18 | 18/18 | 18/18 | 18/18 |
| Cross-schedule: source + 8 edits, both rounds | 18/18 | 18/18 | 18/18 | 18/18 | 18/18 |
| Traced/untraced: source + 4 edits, both schedules | 10/10 | 10/10 | 10/10 | 10/10 | 10/10 |

All 36 untraced and 10 diagnostic comparisons pass byte identity; none relies
on the helper's bounded numeric allowance. These repeated-output checks
establish reproducibility and absence of tracing effects on this workload,
not a new independent geometric proof or a new optimization qualification.

## Reproduce the frozen workload

Python 3, Node and the existing package artifacts suffice; no rebuild is needed.
The commands below use the same maintained worker, source/package revision,
process order, warmups and sample counts. Choose an allowed CPU and stop
competing intensive work. Keep the output directory new.

```sh
PERF_SNAPSHOT=$(mktemp -d /tmp/remus-face-edit-inputs-XXXXXX)
PERF_OUTPUT=$(mktemp -d /tmp/remus-face-edit-results-XXXXXX)
PERF_CPU=0
git archive d94e03b7d28215c9b44dd6925a32a257729ca5c1 \
  crates/wasm/pkg crates/wasm-io/pkg \
  crates/io/tests/data/shapr3d_hammer_holder.step \
  scripts/perf/filleted-face-edit | tar -x -C "$PERF_SNAPSHOT"
for run in geometry-first-0 volume-first-0 volume-first-1 geometry-first-1; do
  schedule=${run%-?}
  taskset -c "$PERF_CPU" node \
    "$PERF_SNAPSHOT/scripts/perf/filleted-face-edit/wasm_edit_bench.cjs" \
    --kernel-pkg "$PERF_SNAPSHOT/crates/wasm/pkg" \
    --io-pkg "$PERF_SNAPSHOT/crates/wasm-io/pkg" \
    --fixture "$PERF_SNAPSHOT/crates/io/tests/data/shapr3d_hammer_holder.step" \
    --out "$PERF_OUTPUT/$run" --label "$run" \
    --warmup 2 --samples 6 --order "$schedule" > "$PERF_OUTPUT/$run.stdout.json"
done
mkdir "$PERF_OUTPUT/diagnostic"
cp "$PERF_SNAPSHOT/scripts/perf/filleted-face-edit/wasm_edit_bench.cjs" \
  "$PERF_OUTPUT/diagnostic/wasm_edit_bench.cjs"
patch -d "$PERF_OUTPUT/diagnostic" -p1 < \
  docs/performance/evidence/face-drag-2026-10-13/phase-tracing.patch
for schedule in geometry-first volume-first; do
  taskset -c "$PERF_CPU" node "$PERF_OUTPUT/diagnostic/wasm_edit_bench.cjs" \
    --kernel-pkg "$PERF_SNAPSHOT/crates/wasm/pkg" \
    --io-pkg "$PERF_SNAPSHOT/crates/wasm-io/pkg" \
    --fixture "$PERF_SNAPSHOT/crates/io/tests/data/shapr3d_hammer_holder.step" \
    --out "$PERF_OUTPUT/diagnostic/$schedule" --label "profile-only-$schedule" \
    --warmup 2 --samples 2 --order "$schedule" \
    > "$PERF_OUTPUT/diagnostic/$schedule.stdout.json"
done
```

Use `qualify_pair` and `distribution` from the frozen
`run_wasm_edit_comparison.py` to compare repeat and cross-schedule source/edit
records and to summarize non-warmup records. Match edits by iteration and
distance. For diagnostics, compare source and iterations 0–3 against the
untraced round-0 schedule. Assign each phase to a volume/recognition root by
its start/end containment; nested durations overlap. The compact evidence
retains those exact comparisons and summaries for this run. Its input hashes
and the trace patch allow inspection without running the benchmark.
