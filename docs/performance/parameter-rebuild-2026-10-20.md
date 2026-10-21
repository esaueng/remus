# Holder parameter geometry rebuild observations — 2026-10-20

Tier: core. Owner: [PERF-M08](../kernel-maturity/roadmap.md#perf-m08). The maintained [parameter driver](../../scripts/perf/parameter-rebuild/README.md) now covers the real 31-feature opening-width construction through the public kernel API, with both full geometry rebuild and explicit cached-prefix suffix replay. This is measurement tooling and evidence; it changes no kernel algorithm, tolerance, deflection, cache bound or refusal behavior.

The primary full-rebuild observations for new widths are **289.73 ms preview** and **670.91 ms completion** at empirical p50, with observed wall p50 **299.30 ms** and **683.66 ms** respectively. Eight retained samples per process on one reserved physical core support a bounded diagnostic baseline. They do not establish a cross-revision speedup, cache hit, browser input-to-frame latency, or tail-confidence claim.

## Exact workload and provenance

- Source: the existing `crates/io/tests/data/shapr3d_hammer_holder.step`, SHA-256 `2403b4c9868cbb56292a09ec2f2d37baca16cc4b789d11e97ff8df3f1e46bef8`, with 160 faces / 42 NURBS faces.
- Recipe/compiler: OpenZCAD `913bb9ddd015bf2e6284c60a231eaa5afe7f2106`, `packages/command-system/src/growing-holder.ts`. The immutable measured recipe, 31-feature layout, exact plain-base / seven-letter input documents, hashes and original Apache 2.0 copyright/license attribution live in the [fixture notice](../../scripts/perf/parameter-rebuild/NOTICE.md). No consumer checkout was changed.
- The opening starts at 46 mm, cuts the straight floor at x = −4 / 26 mm, and holds the measured height at 58 mm. Four carved rigid pieces and three exact line/arc bridge extrusions are placed and unioned; text remains separate and enabled. The floor is 20 × 8 mm with R3 upper corners. Captured widths are 47, 47.02, …, 47.18, 48 and 49 mm. Correctness also covers return to 46 mm; this is not a continuous-range or minimum-width capability proof.
- Measured package: committed kernel **2026.1.69**, 10,475,327 WASM bytes, SHA-256 `77d5614985666de49858561f8fe159c6afdcb49345ecb7d857a6731a259b5f07`. Current WASM, Node wrapper and package JSON were independently compared byte-for-byte with distribution commit `0d942168550b6dd5101d049f7a10de0230bb5848`; [package verification](evidence/parameter-rebuild-2026-10-20/package-verification.json) contains all hashes. That distribution attributes its build to source `62626997c6584a531b3c8f3c9103fe5b73c7e137`. This verifies artifact identity, not reproducible-build equivalence. The driver's optional supplied source SHA is explicitly external provenance.
- Harness base: `c9a1618087320bb0c1496c578c27f5b14a6ca7c4` (#1221). Runtime: Node v24.14.0, Linux x86_64, AMD Ryzen 9 5900XT. Eight independent processes ran serially on CPU 31; physical sibling 15 was excluded from team validation. Module caches never cross processes.
- Capture window: **2026-10-21 03:10:03–03:11:06 UTC** (2026-10-20 local). The coordinator and both validation lanes confirmed active native/WASM validation process trees and all threads excluded CPUs 15/31, with CPU progress during captures. Agent-local `/proc` snapshots saw zero build tools at both boundaries: their limited visibility does not prove host idleness. This was a reserved core with co-resident validation isolated to other CPUs. [Provenance](evidence/parameter-rebuild-2026-10-20/provenance.json) preserves commands, ordering, visibility limits, frozen-code hashes and raw-file hashes.

## Full and suffix scopes

**Full (default):** every request restores one empty checkpoint, restores the exact base/text imports, constructs four masks and their placements/intersections, then rebuilds all three profiles/extrusions, six body placements, the union and text placement. All **31 geometry features** run per request in the same owner. No temporary prefix checkpoint is retained.

**Suffix:** all 17 fixed features prepare once; each request restores that explicit prefix and rebuilds the 14 dependent features. This is a partial replay. The consumer has a separate history-cache policy; these measurements do not establish which prefix it actually retains.

The unchanged pinned recognizer and lettering separator produced the inputs; the new driver adapts the numeric plan. The complete consumer command compiler/build graph was **not executed** against this driver. An [independent pure-planner check](evidence/parameter-rebuild-2026-10-20/plan-fidelity.json) executes the unchanged `growingHolderPlan` function at all 12 captured widths plus 46: profiles and union order agree exactly, while adapted arithmetic differs by at most 7.11e-15 mm in final binary rounding. This does not establish consumer geometry or timing equivalence.

Both scopes start from immutable exact arena inputs. Original STEP translation, opening recognition and lettering separation are excluded, as are document expressions, undo, application history-cache selection, topology witnesses, worker scheduling, rendering and presentation. Full geometry replay is not full application completion. The constructor and checkpoint setup are reported separately.

Preview includes restore, fixed prefix when applicable, dependent reconstruction with checked face unification, strict validation of all eight solids, display mesh at **.02/.35**, and bulk positions/normals/indices/face-offset transfer. Completion adds volume and complete kernel feature recognition on all eight solids at **.08**. `stageSum` includes in-block JavaScript, validation and prefix serialization; it is a sum of replay blocks, not isolated native function timings. `observedWall` also includes between-stage memory/JavaScript overhead and ends before its final memory probe. Other quality reads and writes occur later and may affect subsequent cache/GC history.

## Cold and warmed observations

Each process has ten ticks, the first is cold, and the first two ticks are excluded from retained pools. Each retained pool has n=8. Unique requests are all new; repeated requests have one new retained width and seven previously requested widths. Previously requested does not prove cache reuse. Empirical nearest-rank p50/p95/max are retained; at n=8 p95 equals the observed maximum.

All values below are milliseconds. Stage sums and observed wall intervals are shown together.

| Scope | Workflow / requests | Cold sum | Warm sum p50 / p95 | Warm wall p50 / p95 |
|---|---|---:|---:|---:|
| full | preview / unique | 3032.42 | 289.73 / 408.73 | 299.30 / 411.40 |
| full | preview / repeated | 3032.84 | 241.67 / 294.59 | 250.03 / 297.50 |
| full | completion / unique | 4140.24 | 670.91 / 957.62 | 683.66 / 971.50 |
| full | completion / repeated | 4125.15 | 273.62 / 690.07 | 283.46 / 694.34 |
| suffix | preview / unique | 768.31 | 174.02 / 307.74 | 177.03 / 309.35 |
| suffix | preview / repeated | 760.50 | 129.84 / 193.99 | 132.95 / 196.47 |
| suffix | completion / unique | 1871.14 | 559.30 / 812.63 | 564.29 / 817.32 |
| suffix | completion / repeated | 1850.40 | 155.56 / 569.88 | 159.98 / 573.22 |

Module loading took 60.10–61.51 ms and kernel construction 0.52–0.68 ms. Full mode then creates its empty checkpoint in 0.16–0.17 ms. Suffix mode instead spends **2,263.83–2,303.28 ms** preparing the 17-feature prefix before its cold tick. Comparing the two cold-tick columns alone would omit this work.

For new widths, the measured per-stage p50 values are:

| Stage | Full preview | Full completion | Suffix preview | Suffix completion |
|---|---:|---:|---:|---:|
| restore | 0.064 | 0.064 | 0.486 | 0.445 |
| prefix | 119.118 | 109.185 | — | — |
| rebuild | 135.118 | 136.436 | 135.883 | 133.634 |
| validate | 1.886 | 1.917 | 1.914 | 1.903 |
| mesh | 30.651 | 30.652 | 33.576 | 31.593 |
| transfer | 0.188 | 0.177 | 0.168 | 0.171 |
| volume | — | 238.764 | — | 237.619 |
| recognize | — | 150.921 | — | 149.743 |

Individual stage medians need not sum to the whole-tick median. The full completion observation places substantial time in prefix construction, dependent reconstruction, volume and recognition. These blocks need further native attribution before selecting an optimization. The small repeated-request set has a different request history and supports no speedup ratio or cache-hit attribution.

## Memory observations

Every full process grows from zero allocated topology slots after setup to **130,000 allocated / 117,000 retired** after ten ticks: 6,944 prefix slots plus 6,056 dependent-rebuild slots per request. Restore retires the preceding 13,000 slots. Every suffix process starts at 6,944 prepared slots and ends at **74,448 allocated / 54,504 retired**: the first mutation also copies the checkpointed prefix; later ticks append 6,056 slots each. All 80 records have exactly one live checkpoint. There is no per-prefix checkpoint leak, slot reuse, or automatic compaction.

| Scope | Workflow / requests | WASM linear memory after setup → final (decimal MB) | Final process RSS (decimal MB) |
|---|---|---:|---:|
| full | preview / unique | 1.507 → 121.242 | 415.2 |
| full | preview / repeated | 1.507 → 113.377 | 404.4 |
| full | completion / unique | 1.507 → 175.112 | 538.3 |
| full | completion / repeated | 1.507 → 148.636 | 462.8 |
| suffix | preview / unique | 7.733 → 58.262 | 290.1 |
| suffix | preview / repeated | 7.733 → 51.773 | 377.0 |
| suffix | completion / unique | 7.733 → 154.272 | 432.0 |
| suffix | completion / repeated | 7.733 → 98.370 | 419.2 |

The full new-width completion rows attribute 6,944 allocated slots to prefix construction and 6,056 to dependent rebuilding, with no arena allocation in validation, mesh, transfer, volume or recognition. Their warmed volume stage has the largest observed one-stage WASM growth, **25,296,896 bytes**. Bulk mesh getters transfer **1,557,204 bytes** per tick. Prefix attestation serialization adds 653,660 observed array-buffer bytes per full tick.

Raw rows retain every stage's arena/WASM/RSS/JS-heap/external/array-buffer snapshots and deltas. Process values include fixture buffers, V8, retained diagnostic records, transfers and GC; decreases remain visible. They do not establish retained mesh ownership or a portable memory bound. WASM linear memory is module-wide and need not shrink. Ten ticks do not establish long-session scaling.

## Geometry and independent quantity checks

All **80 ticks / 640 output-solid instances** pass strict validation and finite, watertight, consistently oriented indexed display-mesh checks. Every result keeps the 108-face holder plus seven text solids, expected opening envelope, material in the floor and both arms, empty opening space and empty centers of the original two mounting bores. The fixed input pieces remain byte-identical after rebuilding.

The correctness-only audit reconstructs all **12 distinct captured widths** and matches complete reachable arena and every mesh-buffer SHA-256 to all 80 actual capture records. This covers full carrier/trim/boundary-authority bytes across full/suffix and preview/completion scopes. Full/suffix captured volume vectors also agree exactly in the 20 paired completion cases.

The analytic rounded-floor area is `20*8 - 2*(9 - 9*pi/4) = 142 + 4.5*pi` mm². Order-8 mass-property volume changes from width 46 agree with that area times the width change, with maximum absolute residual **0.0005611902 mm³**, below the unchanged **0.002 mm³ diagnostic quadrature/trim bound**. This changes no kernel tolerance or measurement deflection.

All **42 canonical-input NURBS carriers** (8 holder, 34 derived lettering) retain exact degrees, knots and weights. Expected rigid control-point translation residual is at most **3.55e-15 mm** for the body and zero for text, against the diagnostic 1e-11 mm check. The recognized-feature inventory remains invariant, including both 5 mm mounting bores; lettering volume invariance uses the unchanged 1e-8 mm³ diagnostic check. Reverse/revisited widths and a fresh owner reproduce the complete arena, and full replay proves only one retained checkpoint while tombstones accumulate.

An independent re-query after a different mass-property/cache history is not universally bit-identical: the largest `.08` volume re-read residual is **6.55e-11 mm³** (text residuals are around 1e-14 mm³). These are reported separately from the actual capture's exact paired comparisons. No broad query-history bit-identity claim is made. [Quality evidence](evidence/parameter-rebuild-2026-10-20/quality.json) includes every width, residual, carrier check and matched-record count; [cross-mode/scope qualification](evidence/parameter-rebuild-2026-10-20/qualification.json) preserves the actual capture comparisons.

## Reproduction and limits

Use the [maintained driver commands](../../scripts/perf/parameter-rebuild/README.md) with a new output path for each full/suffix × preview/completion × unique/repeated case. Pin the same reserved physical core for all processes, retain package and driver hashes, and run the offline verifier against the collected run array. The [raw eight-process results](evidence/parameter-rebuild-2026-10-20/runs.json) and provenance retain exact commands and launch order. Earlier contended smoke and superseded suffix-only captures are excluded.

Local checks cover nine new semantic tests, the sixteen existing performance-harness tests, syntax, layer boundaries, documentation/roadmap inventory, and the independent all-width package contract. This tooling PR changes no Rust implementation or generated package. Hosted CI remains a separate delivery signal.

PERF-M08 remains partial. This supplies a maintained full geometry parameter recipe and its completion memory evidence. Original import/recognition/separation, consumer document/worker/frame boundaries, general feature trees, height interaction, long sessions, repeated timing blocks and other user workflows remain open. The earlier direct-edit report remains a different workload; these observations establish no cross-revision performance change.
