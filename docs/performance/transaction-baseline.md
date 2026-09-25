# Transaction scaling baseline — 2026-09-25

One fixed edit: 150 translations of one unit box, with 50/200/400 preexisting
boxes. Fifteen retained samples per case (three sequential processes, five
samples each), after one warmup per process. All 24 correctness checks passed.
Other coordinated builds were paused during this run; host-wide affinity,
frequency, thermal state, and unrelated process contention were not controlled.

| Runtime/path | Checkpoints | 50 boxes | 200 boxes | 400 boxes |
| --- | ---: | ---: | ---: | ---: |
| native direct | 0 | 6.170 / 0.184 | 24.229 / 0.466 | 47.595 / 0.410 |
| native direct | 1 | 6.211 / 0.132 | 24.092 / 0.440 | 47.286 / 1.182 |
| native batch | 0 | 6.291 / 0.084 | 24.323 / 0.387 | 47.728 / 0.842 |
| native batch | 1 | 6.664 / 0.237 | 23.930 / 0.363 | 47.466 / 0.853 |
| wasm direct | 0 | 7.113 / 0.232 | 25.505 / 0.490 | 52.341 / 1.381 |
| wasm direct | 1 | 7.186 / 0.205 | 25.537 / 0.693 | 51.061 / 1.817 |
| wasm batch | 0 | 7.336 / 0.232 | 25.621 / 0.285 | 50.235 / 0.680 |
| wasm batch | 1 | 7.281 / 0.267 | 25.015 / 0.209 | 48.987 / 0.715 |

Cells show **median / sample standard deviation in milliseconds per 150 calls**.
Raw min/max and per-process medians remain in the runner output. Native census:
3,200 / 12,800 / 25,600 live entities and equal allocated slot counts, with no
PCurves; counts remain unchanged by the fixed edit. Every native sample measured
150 transaction entries, 150 deep rollback snapshots, maximum depth 1, and final
active depth 0. A retained checkpoint adds exactly one COW copy per sample;
zero checkpoints add none. There is no nested snapshot amplification in this
witness. Fixed-edit time scales approximately with live topology size, but this
is not CPU/allocation attribution or evidence for a particular redesign.

Native timing includes opt-in counter overhead. The installed WASM is a separate,
uninstrumented artifact: snapshot counts are null, and source equivalence with
native is not established. These numbers establish a baseline, not a speedup.

Reproduce from the measured source with a dedicated target directory:

```bash
CARGO_TARGET_DIR=/tmp/remus-priority-platform-target CARGO_BUILD_JOBS=2 \
CARGO_PROFILE_DEV_DEBUG=0 CARGO_PROFILE_TEST_DEBUG=0 CARGO_INCREMENTAL=0 \
python3 scripts/performance/run.py --offline --family transform \
  --processes 3 --samples 5 --jobs 2 --output /tmp/remus-transaction-baseline
```

CPU: AMD Ryzen 9 5900XT 16-Core Processor. Native profile: optimization 3, debug symbols 2,
no debug assertions; features `io,perf-counters`.

Measured base: `2794ae2d200283a2466c21e109eead3707dfcea2` plus the instrumentation changes;
working-tree SHA-256: `55949cab073413f670ac09dea2267b8e6e5b3a33b7f248b8feb91502c4f1eed5`.
This evidence predates the addition of this report and later integration.
Native binary SHA-256: `4ebf98c07d9e0892fb4b269c2328059910a89a7afb3b11e97bd6bfe168a627e1`.
Installed WASM version: `2.130.50`; binary SHA-256:
`bb96d3e2950b57595aabcacc9e15d9b4734f21412d53bbe6785b46463367fb25`.

Complete raw records, provenance and worker logs were retained at
`/tmp/remus-priority5-baseline-20260925` for this local run. That directory is not
a repository fixture; a fresh run produces the same evidence format. The command
above also records the `CARGO_INCREMENTAL=0` override that the original runner's
environment whitelist did not include; profiling is non-incremental by default.
