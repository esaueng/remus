# Complex-boolean native baseline — 2026-10-28

The maintained [complex-boolean runner](../../scripts/performance/complex-boolean/README.md) now covers 14 bounded successful native cases: Cut/Fuse/Intersect on plates with 0/36/144 holes, 16/64 disjoint-body fuses, 8/16 mutually overlapping-body fuses, and a cylinder cut in half by a box. A 256-disjoint-body case remains an explicit unqualified witness. These fixtures extend the local-fuse and exact-hole benchmarks with generic GFA work; they do not complete the kernel workload inventory.

Tier: native. Owner: PERF-M02/M04/M06, with PERF-B02/B03/B04/B09/B10 attribution. No modeling default, tolerance, strict-validation rule, resource budget, WASM package or store-local adoption policy changes.

## Source and timing boundaries

Both controlled artifacts use a checkout based on `70f675bea4aebd35b0a36d59eaec89118c278ef6` plus the benchmark/instrumentation changes. The captured working-tree identity is `eb80f1529398934d852bc948897a743de189e2796f96ef30bdade3d1c950440d`. Actual Cargo compiler-artifact receipts, worker/counter-source hashes and frozen executable hashes bind each capture to its inputs; source and executable identity were checked before/after sampling. The [source-bound receipt](evidence/complex-boolean-2026-10-28/native-smoke.json) records the toolchain, profile, build command and environment. No reproducible-build claim is made.

The recorded build/collection commands use Rust 1.96.0, the profiling profile (optimization 3, debug symbols, overflow checks), two build jobs, incremental compilation off and `RUST_MIN_STACK=536870912`; Cargo’s actual `fresh` flags identify reused artifacts. Content memos mean **pcurve-and-wire-area-only**; all other native caches retain their defaults. The worker does not initialize a WASM kernel session. Store-local is off unless explicitly requested.

Every sample constructs fresh topology. Setup includes fixture construction and drilling. Preflight reads input volumes. The kernel interval contains only the selected public boolean/fuse call, including its transactions and result gates. Qualification independently checks strict validity, a closed-form volume, expected edge-connected components, material probes and unchanged input-operand volume bits. Teardown destroys the topology. `generation_ns` is setup plus kernel; it excludes preflight, qualification and teardown. These are native intervals, with no browser, worker-queue, rendering or presentation claim.

The full 14-case source-bound smoke passed 28 records (one warmup and one retained sample per case). Its contended build-core timings are excluded. The [receipt](evidence/complex-boolean-2026-10-28/native-smoke.json) records all semantic gates and residuals. These checks establish the declared fixture contracts; they do not establish arbitrary face/edge-content equivalence.

## Instrumentation controls

All controlled workers ran serially on CPU 31 of the recorded AMD Ryzen 9 5900XT; this task's builds excluded CPU 31 and sibling 15. External host activity, thermal state and frequency were not fully characterized, and exclusive core ownership was not established. Each case uses balanced A–B–B–A / B–A–A–B process ordering, four independent processes per variant, two labelled warmups and ten retained samples per process. Raw samples remain available; no samples were trimmed and no tail-confidence claim is made.

Both variants leave phase clocks off. The table reports the median of each variant’s four process-level kernel medians. Instrumented timings include the counter cost and must remain separate from counter-free timings.

| Case | Counter-free, ms | Counters, clock off, ms | Observed counter overhead |
| --- | ---: | ---: | ---: |
| slot-cut-0 | 0.972487 | 0.982559 | +1.04% |
| slot-cut-144 | 11.868706 | 12.041799 | +1.46% |

All 192 raw records / 160 retained samples pass the semantic gates. [Controls and producer receipts](evidence/complex-boolean-2026-10-28/counter-overhead.json); [raw samples](evidence/complex-boolean-2026-10-28/counter-overhead-samples.jsonl). These observations quantify this instrumentation configuration on two fixtures, with no general overhead bound.

The separate 144-hole clock-enabled diagnostic records 7,104 VE pair visits with zero projection evaluations, 5,328 EE visits with zero survivors, 900 FF visits with eight survivors, and 310 store-face copies. One retained diagnostic records VE/EE at about 0.145/0.153 ms, build at 2.864 ms, assembly at 3.011 ms, and store import/export at 0.714/1.011 ms. These are diagnostic phase readings; nested/inclusive scopes must not be summed into a disjoint total. On this fixture the next investigation should emphasize local reconstruction, assembly and copies; pair counts alone do not establish the largest time-saving opportunity.

## Existing store-local setting controls

These comparisons use the **same counter-free executable** for both settings, with the same balanced process/sample policy. A separate instrumented run proves each enabled operation actually succeeds through the store-local route (one local success, zero general-GFA calls), while disabled requests use the general GFA (one call, zero local successes). Phase clocks are enabled only in those separate diagnostic proofs.

| 144-hole plate operation | Disabled, ms | Enabled, ms | Observed kernel reduction |
| --- | ---: | ---: | ---: |
| slot_cut | 15.974653 | 9.655234 | 39.56% |
| slot_fuse | 17.971168 | 7.233656 | 59.75% |
| slot_intersect | 12.084420 | 8.048958 | 33.39% |

All 288 raw records / 240 retained timing samples and 12 separate route/phase-proof records pass. [Controls, route proof and phases](evidence/complex-boolean-2026-10-28/store-local-controls.json); [raw timing samples](evidence/complex-boolean-2026-10-28/store-local-controls-samples.jsonl). The two controlled captures are separate sessions; compare variants within each session.

Store-local remains opt-in and off by default. It changes face/entity ordering, and the separate cross-drilled qualifier found a stored-tolerance mismatch that blocks broad adoption. These results apply to the three declared procedural fixtures; they establish no universal adoption or output-byte-equivalence claim.

## Qualification gap and remaining coverage

The 256-body disconnected fuse fails strict validation with `shell is disconnected: 1530 face(s) not reachable from first face`. Its 32,640 component pairs exceed the unchanged 4,096-pair proof budget. The manifest retains this named witness with a default timing exclusion; an explicit `--case sparse-fuse-256` reproduces the failure and rejects the run. The [receipt](evidence/complex-boolean-2026-10-28/qualification-gap.json) preserves the raw-failure hashes and associates the identical executable with its subsequent actual Cargo producer receipt. No validator check or resource budget was weakened.

Imported NURBS/SSI, cavities, tangencies, large translations, broader model recipes, long sessions, temporary-allocation peaks, WASM/direct/batch/installed-package behavior and consumer presentation remain separate work. The native pattern-copy probe is also a separate workload.

## Portable reproduction commands

Use new result directories. The first two commands reproduce the source-bound smoke/build receipts; the worker commands give the exact capture arguments with portable artifact aliases. Reserve the measurement core and its sibling before repeating controlled captures.

```sh
free_run=target/performance/new-complex-free
work_run=target/performance/new-complex-work

taskset -c 0-14,16-30 env CARGO_BUILD_JOBS=2 CARGO_INCREMENTAL=0 RUST_MIN_STACK=536870912 \
  python3 scripts/performance/complex-boolean/run.py --offline \
  --processes 1 --samples 1 --warmup 1 --output "$free_run"
taskset -c 0-14,16-30 env CARGO_BUILD_JOBS=2 CARGO_INCREMENTAL=0 RUST_MIN_STACK=536870912 \
  python3 scripts/performance/complex-boolean/run.py --offline --counters \
  --processes 1 --samples 1 --warmup 1 --case slot-cut-0 --case slot-cut-144 --output "$work_run"

# Counter overhead: A=free worker, B=instrumented worker; repeat ABBA/BAAB.
taskset -c 31 "$free_run/native-worker" slot_cut 0 10 2 off on off
taskset -c 31 "$work_run/native-worker" slot_cut 0 10 2 off on off
# Repeat the same sequence with size 12 for the 144-hole case.

# Store-local controls: one free worker; A=off and B=on; repeat ABBA/BAAB.
taskset -c 31 "$free_run/native-worker" slot_cut 12 10 2 off on off
taskset -c 31 "$free_run/native-worker" slot_cut 12 10 2 on on off
# Repeat for slot_fuse and slot_intersect.

# Separate instrumented route/phase proof, disabled and enabled.
taskset -c 31 "$work_run/native-worker" slot_cut 12 1 1 off on on
taskset -c 31 "$work_run/native-worker" slot_cut 12 1 1 on on on
# Repeat for slot_fuse and slot_intersect; phase durations are inclusive.
```

The public receipts retain the exact case/variant order, raw-record hashes, sample counts, build commands and compiled-input identities. Local full build/process logs and frozen executables are retained with the capture directories; published data omits their absolute paths.
