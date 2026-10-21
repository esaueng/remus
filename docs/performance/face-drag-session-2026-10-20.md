# Holder edit observations — 2026-10-20

The current committed WASM pair still spends tens of milliseconds on a new
holder preview and hundreds on the maintained completion calls. Every preview
tick also allocates 1,906 arena slots after its first checkpoint copy. This
report refreshes PERF-M08's measurement coverage; it contains no kernel
optimization or cross-revision speedup claim.

These are **reserved-core observations with concurrent validation on other
CPUs**, not a fully idle-host timing qualification. All six processes ran
serially on CPU 31 of an AMD Ryzen 9 5900XT, Node 24.14.0. Its physical sibling,
CPU 15, was excluded from the active validation trees too. Host snapshots
before/after record all active Cargo/Rust/optimizer processes on `0-14,16-30`.
Other system noise, shared memory bandwidth, caches and thermal effects were
not isolated. Each table cell comes from one process; p95 is a small-sample
observation, not a confidence bound or a future latency guarantee.

## Provenance and contracts

- Harness checkout base: `de6352906ebeb7eaa5ff24fbede68d0fffdc8949`, plus this
  measurement change. Driver hashes are recorded in
  [provenance.json](evidence/face-drag-session-2026-10-20/provenance.json).
  The exact drivers and shared helper are retained in the
  [capture-source archive](evidence/capture-sources-2026-10-20/README.md).
  The maintained session driver has since added retained-sample validation;
  the historical records and provenance are unchanged.
- Distributed kernel and translator: **2026.1.69**. Both WASM files, both Node
  wrappers and both package manifests were independently compared byte-for-byte
  with distribution commit `0d942168550b6dd5101d049f7a10de0230bb5848`.
  [The six hashes](evidence/face-drag-session-2026-10-20/package-verification.json)
  preserve that check. That commit attributes the build to
  `62626997c6584a531b3c8f3c9103fe5b73c7e137`; this differs from the harness base.
  The driver's build-source flag is externally supplied provenance, not an
  automatic source-to-binary verification.
- Public fixture: `crates/io/tests/data/shapr3d_hammer_holder.step`, SHA-256
  `2403b4c9868cbb56292a09ec2f2d37baca16cc4b789d11e97ff8df3f1e46bef8`.
  It has 160 faces, including 42 NURBS faces. The selected planar cap starts
  at x=48 mm. The selection-only area guard is unchanged at 1e-4 mm².
- Display deflection/angle remain **0.02/0.35**. Completion quantity and
  recognition deflection remain **0.08**. Production cache defaults are kept.
- All **96 preview ticks** pass relaxed validation, finite/grouped display
  buffers, index closure/manifold checks, face count and requested cap placement.
  The existing completion worker separately checks strict validation, volume
  increase, face count, requested cap placement and watertight display meshes
  for **12 edits**, including its four warmups. An offline audit also confirms
  every edit's recorded carrier census and analytic radii match its source.
  These gates do
  not certify every geometric property or compare two different kernels.

Two short smoke processes ran during compilation and were explicitly tagged
`contended-smoke-excluded`. Their timings are excluded from this evidence.

## Preview timing

The maintained session driver keeps one kernel and one source checkpoint.
Each tick restores it, moves the cap, validates with `validateSolidRelaxed`,
tessellates, and transfers positions/normals/indices/grouping. Quality checks
run after the timers. Their allocations and queries can affect the next tick;
this is the recorded instrumented workload, not an application frame loop.

Each process attempts 24 ticks. The first two are retained as warmup evidence
and excluded from warmed distributions; the first edit is also reported alone.
`unique` requests `5 + tick*0.02` mm. `repeated` retains the original 20-tick
triangle from 5 to 7 mm. The schedules differ in offset distribution. A
revisited request includes any earlier requested value, including a refusal;
it is **not evidence of a cache hit**.

Milliseconds. Warmed p50/p95 use empirical nearest rank. `kernel tick` sums
restore/move/validation/mesh durations. `with transfer` adds bulk getters and
grouped-result release. `observed wall` measures restore through transfer and
includes the between-stage memory reads and JavaScript bookkeeping.

| Move / requests | Warm n | First / revisited n | First edit kernel tick | Warm kernel tick p50 / p95 | With transfer p50 / p95 | Observed wall p50 / p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Prepared / unique | 22 | 22 / 0 | 1466.87 | 45.26 / 56.10 | 45.44 / 56.27 | 46.85 / 57.53 |
| Unprepared / unique | 22 | 22 / 0 | 1618.59 | 48.40 / 61.34 | 50.09 / 61.57 | 51.69 / 62.63 |
| Prepared / repeated | 22 | 9 / 13 | 1516.29 | 26.47 / 60.27 | 26.63 / 60.55 | 28.38 / 61.97 |
| Unprepared / repeated | 22 | 9 / 13 | 1516.58 | 34.01 / 63.99 | 34.18 / 64.31 | 35.88 / 65.91 |

Within the prepared repeated process, the nine warmed first-time requests
have kernel-tick p50 52.23 ms; the thirteen revisited requests have p50
25.24 ms. This separates request history without assigning the difference to
a cache or deriving a speedup. In the prepared unique process, move p50 is
25.03 ms and mesh p50 is 16.87 ms. Individual stage medians need not sum to
the median tick.

The prior [October 19 report](face-drag-session-2026-10-19.md) used a cloud
Xeon, Node 22 and a different package/harness. Its numbers are not a controlled
baseline for a percentage improvement here. Neither report measures worker
queues, consumer orchestration, browser rendering or input-to-presented-frame
latency. These observations do not establish a frame rate.

## Completion calls

The unchanged `filleted-face-edit` workflow uses a fresh kernel restored from
the source arena for each edit. It calls `moveFacesJournaled`, strict
validation, bbox, display mesh/transfer, volume and recognition. Two warmups
precede four retained unique offsets per schedule. These are different
contracts from the prepared/relaxed preview; they are not timed application
commit events.

Completion pipeline rows sum the measured API intervals too. JavaScript
between calls, quality/artifact work and observed whole-loop wall time are
excluded by that worker; the preview driver is the one that records both
summed intervals and observed wall time.

Milliseconds. Medians use the middle-pair mean for these four samples; p95 is
the maximum. The full raw per-stage readings remain in
[completion-runs.json](evidence/face-drag-session-2026-10-20/completion-runs.json).

| Stage | Geometry-first median | Volume-first median |
| --- | ---: | ---: |
| Journaled move | 39.75 | 35.79 |
| Strict validation | 2.84 | 2.91 |
| Bounding box | 1.08 | 0.98 |
| Display mesh | 19.95 | 20.81 |
| Bulk transfer | 0.23 | 0.27 |
| Volume | 102.03 | 112.82 |
| Recognition | 44.52 | 46.54 |
| Pipeline including transfer, median / p95 | 214.39 / 264.67 | 226.15 / 273.54 |
| Including construction/restoration, median / p95 | 219.57 / 272.69 | 233.77 / 281.79 |

Volume is the largest measured warmed completion stage in these processes.
The initial source volume reads cost 959–962 ms and subsequent source strict
validation reads cost 987–999 ms, before the edit samples. Those source-query
warmups are recorded separately and do not represent cold application launch.
This report establishes a current distribution baseline; it does not isolate
an optimization or qualify unshipped source changes.

## Per-stage memory

Every preview process starts with 1,906 allocated slots and ends with 49,556,
including 43,838 retired slots. The first tick adds 3,812 slots, which includes
the checkpoint's one-time document copy; each later tick adds 1,906. The
restores retire the preceding result. This remains observable in the current
WASM package regardless of a separate native compaction API.

For prepared unique requests, stage snapshots show:

| Stage | Allocated-slot change p50 | Retired-slot change p50 | Largest observed WASM linear-memory increase |
| --- | ---: | ---: | ---: |
| Restore | 0 | 1906 | 0 bytes |
| Move | 1906 | 0 | 1,507,328 bytes |
| Relaxed validation | 0 | 0 | 0 bytes |
| Display mesh | 0 | 0 | 15,007,744 bytes |
| Transfer | 0 | 0 | 0 bytes |

Kernel WASM linear memory grows by **35.5–40.0 MB** over 24 ticks across the
four processes. For prepared unique requests it grows from 3,866,624 to
40,042,496 bytes. Its allocator can grow during meshing without allocating
new topology slots. Linear-memory growth is not a measurement of retained mesh
ownership, and JavaScript quality checks/diagnostic reads affect process
allocation history. Full stage snapshots also retain RSS, heap, external and
array-buffer values; GC decreases are preserved. Completion-stage memory
attribution is not added by this change.

## Reproduce and remaining coverage

Use the recorded distribution bytes and verify their hashes first. For the
captured source version, follow the
[archive materialization instructions](evidence/capture-sources-2026-10-20/README.md).
The command below runs the maintained driver and produces new observations;
its source hash differs from the historical capture. Run each case in a
separate process on an available reserved CPU; the recorded commands and
launch order are in provenance. For one preview process:

```sh
taskset -c 31 node scripts/perf/face-drag-session/drag_session_bench.cjs \
  --kernel-pkg crates/wasm/pkg --io-pkg crates/wasm-io/pkg \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --ticks 24 --warmup 2 --move prepared --pattern unique \
  --timing-status measured \
  --package-source-sha 62626997c6584a531b3c8f3c9103fe5b73c7e137 \
  --out /path/to/new-preview.json
```

Repeat for both move modes and both request schedules. Run the maintained
completion worker separately with `--warmup 2 --samples 4`, once per order.
[Raw preview records](evidence/face-drag-session-2026-10-20/preview-runs.json)
and full worker result records are retained; only local path prefixes were
normalized. Mesh/arena payload files are regenerated by the recorded commands;
their original hashes remain in the completion records. The committed
provenance file hashes these three evidence inputs independently.

The direct-edit captures above do not include parameter rebuilds. The separate
[parameter-rebuild follow-up](parameter-rebuild-2026-10-20.md) supplies a
maintained driver that runs the real 31-feature opening-width geometry recipe
against a kernel package without a consumer checkout. It reports full geometry
rebuilds and explicit suffix replay from a prepared prefix, including
completion-stage memory. Those captures are separate from these direct-edit
measurements.

Neither replay executes the complete consumer document/worker/presentation
path. Direct-edit completion memory, original import/selection/sketch/export
workflows, more process repetitions and long-session qualification remain open
under PERF-M08.
