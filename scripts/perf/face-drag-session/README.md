# Face-drag session benchmark

Times the kernel calls of one interactive face drag the way the OpenZCAD
preview makes them, in a single long-lived kernel. It imports the 160-face
Shapr3D hammer holder and checkpoints it. Each tick then runs:

1. `restore` to the source checkpoint;
2. `moveFacesPrepared`, or `moveFaces` with `--move unprepared`, on the +X
   cap at x=48;
3. `validateSolidRelaxed`;
4. `tessellateSolidGroupedBinary` at 0.02/0.35.

Distances start at 5 mm, step outward 0.2 mm per tick and return every 20
ticks, so a long drag revisits values as a real one does. After every tick the
script reads `checkpointMemoryStats` outside the timers. That shows the growth
of allocated and retired arena slots, and of WASM linear memory, across the
drag. Restores retire every slot a tick allocated; slots are never reused, so
raw handles cannot alias later entities.

`--background N` creates N unrelated boxes before the import, which exposes
per-tick work that grows with the document. Mesh transfer, volume, recognition
and all consumer work are excluded: the OpenZCAD
`test/perf/offset-face-perf.test.ts` harness measures the complete adapter
path. The script makes no measured claims; see
[the dated baseline](../../../docs/performance/face-drag-session-2026-10-19.md).

```sh
node scripts/perf/face-drag-session/drag_session_bench.cjs \
  --kernel-pkg crates/wasm/pkg --io-pkg crates/wasm-io/pkg \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --ticks 200 --move prepared --background 0 --label prepared \
  --out /path/to/new-result.json
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--ticks` | 200 | Drag ticks |
| `--move` | `prepared` | `prepared` or `unprepared` |
| `--background` | 0 | Unrelated boxes created before the import |
| `--window` | 20 | Accepted ticks in the early and late drift windows |
| `--out` | none | JSON with metadata, per-tick records and the summary |

The summary reports each stage's p50/p95/max over accepted ticks, including
the cold first tick. It also reports the tick p50 of the first and last
windows, slots per tick, and memory at the first and last ticks. Pin the
process to one CPU with `taskset` and compare packages in separate processes,
because WASM thread-local memos persist for the life of a process.

Static check: `node --check scripts/perf/face-drag-session/drag_session_bench.cjs`.
