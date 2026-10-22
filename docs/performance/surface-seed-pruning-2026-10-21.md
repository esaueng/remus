# Exact surface-seed distance pruning

Tier: core. Owners: [PERF-N08](../kernel-maturity/roadmap.md#perf-n08) and [PERF-M08](../kernel-maturity/roadmap.md#perf-m08). The cached NURBS surface seed scan now stops evaluating a candidate distance once its rounded nonnegative partial sum cannot beat the current winner. This preserves the selected seed and the existing Newton solver. The cubic native projection microbenchmark observes a reduction; the higher-degree case remains unresolved within variation. Full holder measurements and qualification are recorded below.

## Choice and exactness

A diagnostic profile runs the maintained [real 31-feature Hammer Holder recipe](../../scripts/perf/parameter-rebuild/README.md), full geometry rebuild, completion, three new widths. The baseline named module is qualified against its canonical shipping module: removing **only** the name section produces byte-identical executable/package WASM, with 7,301 function names. [Profile evidence](evidence/surface-seed-pruning-2026-10-21/profile.json) records both hashes and the mapping check. Profile timings are excluded from comparisons.

Of 11,881 samples, face integration has 5,455 inclusive samples (45.9%); emulated FMA has 3,511 self samples (29.6%). Surface projection has 757 inclusive samples (6.4%). The two main projection wrappers contain 716 inclusive samples and refinement contains 430. Their 286-sample difference (2.4% of all samples) includes seed/setup/wrapper work; it is an opportunity envelope in this trace, not direct attribution to the scan or a predicted whole-workflow gain. The profile includes cold work and untimed JavaScript quality diagnostics. Larger integration and basis changes need separate numerical proofs; this change removes unnecessary FMA work without changing the numerical result.

For every node the original squared distance is the same ordered fold: rounded `dx*dx`, then fused `dy*dy + partial`, then fused `dz*dz + partial`. Adding a nonnegative square cannot reduce a nonnegative rounded partial. If that partial is already `>= best`, the full distance cannot win the original strict `<` comparison. Every surviving candidate retains the original arithmetic and scan order. A NaN comparison does not prune; a skipped later NaN could not improve the original winner. Infinite-only candidates retain the original origin fallback. Equal distances keep the first node, including signed-zero and underflow cases.

This changes no grid resolution, convergence rule, Newton step, fallback, operation tolerance, deflection, resource bound, cache key or error policy. It introduces no result cache or translation normalization. It does not establish a general global-minimum projection guarantee.

The regression oracle executes the untouched full scan and compares the selected **index, squared-distance bits and UV bits**. It checks 2,197 coordinate triples covering finite extremes, subnormals, signed zero, infinities and NaN, plus explicit ties and underflow. It also checks 336 actual stored boundary endpoint uses across all 42 NURBS carriers in the canonical holder/text fixtures. Existing projection and multiple-start tests remain unchanged.

## Matched native projection observations

The maintained `nurbs_inner_loops` Criterion target now measures complete cached-grid projection, including refinement, at surface degrees 3 and 9. Each iteration projects 16 deterministic nearby points. Each independent process warms for 300 ms and collects 20 samples over approximately one second per degree. Both variants use the same opt-3, LTO-off profiling build with debug information disabled after a compiler SIGSEGV while profiling debug information was enabled. This is a native microbenchmark, not the full holder or shipping browser workflow.

Four processes from two frozen executables run **A–B–B–A**, serially on CPU 15; its verified SMT sibling 31 is excluded from team validation. The window is **2026-10-21 05:27:30–05:27:43 UTC**, on AMD Ryzen 9 5900XT / Linux x86_64. Validation continues on other cores. Independent readable-process snapshots see 5 build processes and 21 then 24 threads, with no CPU 15/31 affinity violations. The coordinator also verifies all active team trees and future commands. These observations establish a reserved physical core, not an idle host or package power/thermal isolation.

The slope estimates below are microseconds per 16 projections; brackets are each process's Criterion 95% interval. There are only two processes per variant, so these intervals do not establish a population or tail-confidence claim.

| Degree | A first | B first | B second | A last |
|---|---:|---:|---:|---:|
| 3 | 32.748 [32.490, 33.040] | 29.082 [28.784, 29.374] | 29.384 [29.130, 29.708] | 32.289 [32.009, 32.623] |
| 9 | 171.861 [169.914, 174.203] | 170.608 [167.858, 173.876] | 172.341 [169.930, 174.728] | 175.992 [174.076, 178.063] |

The degree-3 process midpoints differ by **10.1%**; both candidate runs are faster than both baseline runs, while baseline slopes differ by 1.4%. This is a narrow observed reduction. Degree 9 overlaps and baseline drift is 2.4%; no improvement is claimed there. [Native evidence](evidence/surface-seed-pruning-2026-10-21/native.json) retains every estimate, raw iteration/sample pair, executable hash, command and reservation summary. An earlier invocation without Criterion's required `--bench` flag performed smoke checks only and is excluded.

## Full holder comparison scope

Baseline A is source `e3585dd7ff8253f955224645807fbd3c5401d42a` with a test/benchmark-only overlay; candidate B is runtime source `852611823e36759e7964c0c2c00f012bf3238139`. [The baseline overlay](evidence/surface-seed-pruning-2026-10-21/baseline-proof-overlay.patch.gz) (gzip archive of the exact original patch) contains no production change. Both use the same lockfile and canonical `cargo xtask wasm-build`: kernel plus lazy translators, bundler plus Node glue, the pinned wasm-bindgen 0.2.126 and wasm-opt 117, and existing `-Oz --enable-simd` policy. The generated package version is prospective **2026.1.70**, derived from committed v69 by the build; workspace Cargo version remains 2026.1.0. These are matched fresh source builds, not the earlier committed-v69 observations or a published release.

The shipping kernel WASM SHA-256 values are A `badcb15af1397e92e84a66a6931a66d41fee6c9e8d40b197324cfa16091683e1` and B `57cfd2a79abdac944d5c7c01478662042b1e84479a7f36a807d409a04ae4e5f8`. Package/wrapper/input hashes and build qualification remain separate from the driver's externally supplied source SHA.

The unchanged full driver restores one empty checkpoint and rebuilds all 31 geometry features on every new-width request in the same owner. Completion includes restore, fixed prefix, dependent reconstruction, strict validation of all eight solids, grouped display mesh at .02/.35, bulk buffer transfer, volume and complete feature recognition at .08. Constructor/checkpoint setup and module loading remain separate. Original STEP translation, input recognition/separation, consumer graph execution, undo/history-cache selection, worker scheduling, renderer and presentation remain outside this kernel replay. The fixture's pinned recipe origin, copyright, license, canonical data and hashes remain in its [public notice](../../scripts/perf/parameter-rebuild/NOTICE.md).

`stageSum` is the sum of replay block intervals, including in-block JavaScript. `observedWall` also includes between-stage memory probes and scheduling. Untimed inventory, arena, material and mesh-quality reads happen later and may warm subsequent ticks. The first request is cold after separately reported setup; the first two requests are excluded from retained pools. Unique widths are new requests, with no revisited pool in this comparison. This provides no repeated-width or cache-hit claim.

Four serial A–B–B–A captures run **2026-10-21 05:43:40–05:44:26 UTC** on the same reserved CPU 15. The coordinator's fresh audit covers 23 team processes / 51 threads, all excluding 15/31. Agent-local snapshots cover 5 / 21 before and 8 / 28 after. Their process-name filter omits Node 24's `MainThread` name, so they are partial visibility; coordinator verification and inherited masks provide the wider reservation evidence. Independent package runtime tests continue on other CPUs. This is co-resident validation, with no hard host, thermal or package-power isolation.

Each process retains eight warmed, first-request samples. Milliseconds below are empirical nearest-rank observations; p95 equals the observed maximum at n=8.

| Process | Cold sum / wall | Warm sum p50 / p95 | Warm wall p50 / p95 | Warm volume p50 |
|---|---:|---:|---:|---:|
| A first | 4973.93 / 4989.70 | 548.53 / 696.72 | 566.34 / 702.07 | 22.396 |
| B first | 5347.89 / 5356.45 | 523.32 / 658.39 | 530.82 / 663.29 | 19.995 |
| B second | 4350.44 / 4357.51 | 485.42 / 823.09 | 491.07 / 829.66 | 19.785 |
| A last | 4817.53 / 4826.33 | 500.16 / 700.33 | 516.82 / 721.58 | 22.340 |

**No complete-workflow speedup is established.** The process medians overlap the roughly 9% baseline drift and wider per-tick variation. The volume block's process-median midpoint is 11.1% lower for B, a narrow observation consistent with the optimization; one B volume tick at width 47.14 takes 31.913 ms versus approximately 22 ms in A, so no universal per-width or tail improvement is claimed. Warm volume is only about 4% of these total ticks; the earlier cold-inclusive profile cannot predict their overall reduction. Prefix, rebuild and recognition dominate the warmed path, with process p50 ranges 131.71–144.85, 143.47–150.52 and 151.16–157.96 ms respectively.

Module load takes 65.56–85.57 ms, construction 0.568–0.691 ms and empty checkpoint setup 0.169–0.202 ms. These are separately reported and not part of the tick sums. [Raw runs](evidence/surface-seed-pruning-2026-10-21/runs.json) retain every stage, cold/warm denominator, complete semantic output, memory probe and package/driver hash; [capture provenance](evidence/surface-seed-pruning-2026-10-21/capture.json) retains launch order, commands and reservation limits. Two-tick contended development checks are excluded.

## Memory and independent quality

All four runs start with zero allocated slots and 1,507,328 bytes of WASM linear memory. After ten full rebuilds, every run has **130,000 allocated / 117,000 retired slots**, one retained checkpoint and **143,982,592 bytes** of linear memory. Every tick appends 6,944 fixed-prefix and 6,056 dependent-rebuild slots; restore retires the preceding result. Warmed volume adds no arena slots or linear-memory growth in these captures. Transfers are exactly **1,557,440 bytes per tick** in both variants. No automatic compaction, slot reuse or checkpoint policy changes are introduced.

Final process RSS is A 426.2 / 491.3 MB, B 428.7 / 514.6 MB (decimal). Every stage's RSS, JS heap, external and array-buffer observations remain in the raw runs, including decreases. They include V8, diagnostic buffers, fixture reads, transfers and GC; they do not attribute memory solely to a kernel or establish a portable bound. **No memory reduction is claimed.**

All **40 captured ticks / 320 output-solid instances** pass strict validation and finite, closed, consistently oriented indexed display meshes. Every result retains the 108-face holder and seven separate text solids, opening envelope, floor/arm material, empty opening and both empty mounting-bore centers. The [guarded cross-variant comparison](evidence/surface-seed-pruning-2026-10-21/guarded-qualification.json) requires different recorded kernel WASM identities and exact complete reachable arena, every mesh buffer, every recorded volume and complete feature record, in addition to inventories, material and policy. The pairs A first/B first `[0,1]`, A first/B second `[0,2]` and A last/B first `[3,1]` cover all four processes and all 40 captured records at all ten widths. This is actual captured-output identity, not only a sampled geometry check. The later qualification rechecks the existing captures; it collects no new timings.

The original [qualification record](evidence/surface-seed-pruning-2026-10-21/qualification.json) remains unchanged. Its third pair `[0,3]` compares A with A and provides same-variant drift agreement, rather than cross-variant proof. The historical comparator SHA-256 `9952a2c448485e12568239ceedb55cac518e46d8f85f7d990cb78b342cbf207b` remains pinned in [build provenance](evidence/surface-seed-pruning-2026-10-21/build.json). The separate guarded record pins the fixed comparator, its validation dependency and the unchanged capture input by hash; its cross-variant pairs replace the original record as the proof covering every process across A and B.

The unchanged independent verifier runs separately for each package, matching its 20 actual capture records at **all ten captured widths**, and retaining its original five-request forward/reverse fixture contract. It rejects mixed-package audit inputs by exact package hash; an initial mixed-array invocation was rejected and preserved, then rerun with correctly split inputs. No verifier guard or measurement is relaxed. [Baseline](evidence/surface-seed-pruning-2026-10-21/baseline-quality.json), [candidate](evidence/surface-seed-pruning-2026-10-21/candidate-quality.json) and [summary](evidence/surface-seed-pruning-2026-10-21/quality-summary.json) record:

- Analytic rounded-floor section area `142 + 4.5*pi` mm²; order-8 mass-property volume deltas have maximum residual **0.0005611902 mm³** in both variants, under the unchanged **0.002 mm³** diagnostic quadrature/trim bound.
- All **42 NURBS carriers** retain exact knots, weights and degrees. Rigid control-point residual is at most **3.55e-15 mm** for the holder and zero for text, against the unchanged 1e-11 mm diagnostic check. Exact arena hashes also cover trims and represented per-use boundary data.
- Both recognized 5 mm mounting bores, the whole feature summary, all lettering material/quantities, strict mesh gates and same-owner/fresh-owner checkpoint contracts remain valid. The existing text-volume diagnostic bound is 1e-8 mm³.

A later volume re-query under the independent audit's different mass/cache history differs from the actual captured quantity by at most **2.11e-10 mm³**, equally for A and B. This residual is recorded separately from exact A/B capture comparisons; no broad query-history bit-identity claim is made.

## Qualification and reproduction

Both variants pass the complete canonical paired kernel/translator build, fresh-package smoke regressions and installed-tarball consumer regressions. Every frozen package file is hash-verified; candidate snapshots also match the final canonical outputs. [Build provenance](evidence/surface-seed-pruning-2026-10-21/build.json) preserves all package hashes, immutable runtime source, raw WASM identity, lockfile and driver hashes, versions, flags and successful log digests. Generated packages are restored before delivery and are not included in this PR.

Early compiler/optimizer attempts faulted and are excluded. A bounded recovery verifies Binaryen 117's official release archive/checksum and byte-identical cached executable. An unchanged bindgen regeneration differs from a failed optimizer input by one bit while raw Rust WASM remains identical; unmodified canonical regeneration then succeeds. No byte patch, optimizer/version weakening or shared-cache overwrite is used. The cause remains unproven. [Recovery evidence](evidence/surface-seed-pruning-2026-10-21/recovery.json) records these hashes and the native debug-information override separately from successful qualification.

Capture-time local validation passed all **1,089 math nextest cases** (one existing skipped), all 1,089 math cargo-test cases, the 22 projection tests, three cross-variant comparator tests and nine parameter-driver tests. Normal capture-time commit hooks passed workspace all-target/all-feature Clippy, fmt, taplo and cargo-machete. The 2026-10-22 integration recheck passed **1,092 math nextest cases** (one existing skipped) and all **35 maintained performance-script tests**, including five comparator cases that reject identical kernels while retaining exact semantic checks. Layer boundaries, deterministic-hash policy, documentation paths/module map, roadmap inventory and the guarded qualification hashes passed. Hosted CI and eventual committed-package refresh remain separate delivery signals.

To reproduce the native comparison, build baseline source with the preserved test/benchmark overlay and candidate source with the maintained benchmark, using separate targets. Preserve both executables and run A–B–B–A with fresh Criterion output directories:

```sh
CARGO_PROFILE_PROFILING_DEBUG=0 CARGO_BUILD_JOBS=2 RUST_MIN_STACK=67108864 \
  cargo bench --locked --offline --profile profiling -p remus-math \
  --bench nurbs_inner_loops --no-run
CRITERION_HOME=/tmp/new-seed-criterion-A \
  taskset -c 15 "$SEED_BASELINE_BIN" surface_projection --bench --noplot
```

For the full holder, build each pinned source with the canonical command and preserve independent package directories. Coordinate a physical-core reservation, use new outputs for each serial process, and retain source/package/driver hashes:

```sh
node scripts/perf/parameter-rebuild/parameter_bench.cjs \
  --kernel-pkg "$SEED_VARIANT_PACKAGE" --out /tmp/new-seed-completion.json \
  --rebuild full --mode completion --pattern unique --ticks 10 --warmup 2
node scripts/perf/parameter-rebuild/compare_variants.cjs before.json after.json
node scripts/perf/parameter-rebuild/verify_recipe.cjs \
  "$SEED_VARIANT_PACKAGE" /tmp/new-seed-quality.json variant-runs.json
```

Run the verifier separately for each exact package's run array. Cross-variant comparisons require all semantic outputs exactly; independent audit re-query residuals remain a separate observation. The baseline and candidate report the same full recipe fidelity boundary as the maintained fixture: this does not execute the complete pinned consumer compiler/build graph.

To recheck all historical records with the maintained identity guard, compare the captured runs across the two kernel WASM identities. The original A/A drift pair is intentionally absent from this cross-variant check:

```sh
node <<'NODE'
const fs = require('node:fs');
const { compareVariants } = require('./scripts/perf/parameter-rebuild/compare_variants.cjs');
const runs = JSON.parse(fs.readFileSync(
  'docs/performance/evidence/surface-seed-pruning-2026-10-21/runs.json'));
for (const pair of [[0, 1], [0, 2], [3, 1]]) {
  console.log(JSON.stringify({
    pair, ...compareVariants(...pair.map(index => runs[index])),
  }, null, 2));
}
NODE
```

Check the comparator, validation dependency and input hashes against `guarded-qualification.json` when reproducing that particular qualification. The capture-time harness and build hashes remain historical evidence; the maintained drivers include subsequent sample-retention guards.

PERF-N08 and PERF-M08 remain partial. The measured native cubic case and observed volume-stage medians support this bounded exact optimization. Higher-degree gains, a complete-workflow improvement, general safeguarded projection, long sessions and browser/worker/input-to-frame behavior remain unqualified. Repeated-width and suffix captures from the earlier measurement report are not cross-revision performance evidence here.
