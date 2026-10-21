# Face-drag session benchmark

Times the kernel calls of one interactive face drag the way the OpenZCAD
preview makes them, in a single long-lived kernel. It imports the 160-face
Shapr3D hammer holder and checkpoints it. Each tick then runs:

1. `restore` to the source checkpoint;
2. `moveFacesPrepared`, or `moveFaces` with `--move unprepared`, on the +X
   cap at x=48;
3. `validateSolidRelaxed`;
4. `tessellateSolidGroupedBinary` at 0.02/0.35;
5. bulk transfer of positions, normals, indices and face grouping.

The default `repeated` distances start at 5 mm, step outward 0.2 mm per tick
and return every 20 ticks. `--pattern unique` uses `5 + tick * 0.02` mm to
separate first-time work from revisited values. These schedules have different
distance distributions; their timing difference alone is not a cache speedup.
After each stage the script reads `checkpointMemoryStats` and Node process
memory outside the stage timer. This shows observed changes in allocated and
retired arena slots, WASM linear memory, RSS, JavaScript heap, external memory
and array buffers. Process memory can decrease through GC; the snapshots do
not prove retained ownership or isolate allocator costs. Diagnostic reads can
themselves affect memory. Restores retire every slot a tick allocated; slots
are never reused, so raw handles cannot alias later entities.

`--background N` creates N unrelated boxes before the import, which exposes
per-tick work that grows with the document. Volume, recognition and all
consumer work are excluded: the OpenZCAD
`test/perf/offset-face-perf.test.ts` harness measures the complete adapter
path. The script makes no measured claims; see
[the dated baseline](../../../docs/performance/face-drag-session-2026-10-19.md).

```sh
node scripts/perf/face-drag-session/drag_session_bench.cjs \
  --kernel-pkg crates/wasm/pkg --io-pkg crates/wasm-io/pkg \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --ticks 200 --warmup 2 --move prepared --pattern repeated \
  --background 0 --label prepared --timing-status measured \
  --package-source-sha <exact-source-that-built-both-packages> \
  --out /path/to/new-result.json
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--ticks` | 200 | Drag ticks |
| `--move` | `prepared` | `prepared` or `unprepared` |
| `--background` | 0 | Unrelated boxes created before the import |
| `--window` | 20 | Accepted ticks in the early and late drift windows |
| `--warmup` | 2 | Initial ticks excluded from warmed distributions |
| `--pattern` | `repeated` | Original triangular sequence or unique offsets |
| `--timing-status` | `measured` | `contended-smoke-excluded` marks diagnostic timings as excluded |
| `--package-source-sha` | unknown | Externally supplied build-source provenance; the driver does not verify it |
| `--out` | none | New JSON file; an existing file is refused |

The original `accepted` summary still includes all accepted ticks, including
the cold first edit. `cold` retains that edit independently; `warmed` excludes
it and the configured warmups. `firstSeen` and `revisited` split the warmed
pool by whether an offset was requested earlier in this process, including a
refused request. `revisited` is not evidence of a cache hit. Every distribution
records its sample count and nearest-rank p50/p95/max; small samples make p95
a descriptive tail observation, not a statistical confidence bound. Source
import and move preparation have separate durations. This is cold edit work
after source import/face selection, not a cold application launch.

The timing boundaries are explicit:

- `tick` is the sum of restore, move, relaxed validation and display-mesh
  kernel calls, preserving the original reported boundary.
- `transfer` times bulk getters and grouped-result release;
  `tickWithTransfer` adds it to `tick`.
- `observedWall` times restore through transfer and includes between-stage
  memory reads and JavaScript bookkeeping. It is not input-to-frame latency.
- Mesh finiteness, grouping, index closure, face count and requested cap
  placement are checked outside all these timers, using the existing full-edit
  harness's mesh gate. Volume and recognition remain separate commit work.

Early/late windows expose drift; per-stage memory deltas and before/after
snapshots remain in every raw record. Slot growth is
split in two: the first tick's slots include a one-time copy of the whole
document, because the first write after a checkpoint copies the topology and
the stats count both copies. The recurring slots per tick are measured from
the first tick onward, and are unavailable for a one-tick run. Pin the
process to one CPU with `taskset` and compare packages in separate processes,
because WASM thread-local memos persist for the life of a process. Package
WASM/wrapper hashes, fixture and harness hashes, runtime versions and actual
affinity accompany every run. A committed package may predate the checkout;
record its build source explicitly and verify package/wrapper bytes against
the distribution revision independently. Do not compare loaded diagnostic timings
with quiet uninstrumented timing as a performance ratio.

The parameter-rebuild workflow is not implemented by this driver. It needs
the maintained consumer recipe/worker path; moving one face is not a substitute
for rebuilding the imported holder's feature recipe. The full completion
worker in `../filleted-face-edit` separately measures validation, bbox,
meshing/transfer, volume and recognition with stronger quantity/artifact gates.

Checks:

```sh
node --check scripts/perf/face-drag-session/drag_session_bench.cjs
node scripts/perf/face-drag-session/drag_session_bench.test.cjs
node scripts/perf/recognition-graph/profile_recognition.test.cjs
node scripts/perf/recognition-graph/compare_runs.test.cjs
```
