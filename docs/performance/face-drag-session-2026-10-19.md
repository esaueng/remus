# Face-drag session baseline — 2026-10-19

This is the kernel side of the Phase 0 edit-latency baseline (PERF-M08). It
runs the hammer-holder face drag the way the OpenZCAD preview calls the kernel:

- one long-lived kernel;
- one source checkpoint;
- per tick: `restore`, then the move, then `validateSolidRelaxed`, then
  `tessellateSolidGroupedBinary` at 0.02/0.35.

[`scripts/perf/face-drag-session`](../../scripts/perf/face-drag-session/README.md)
is the driver. The consumer view is the OpenZCAD
`docs/qa/face-drag-kernel-calls-2026-10-19.md` baseline (esaueng/OpenZCAD#761):
it records about 10,000 kernel calls per adapter tick, most of them per-face and
per-edge reads outside this sequence.

## Inputs

| Input | Value |
| --- | --- |
| Package | Committed `crates/wasm/pkg` 2026.1.68 at `59d02a1`, kernel WASM SHA-256 `4acde153b577e2d23be996bac6a4234cbeb914f19ae0cf14e636d72df6a88172` |
| Fixture | `crates/io/tests/data/shapr3d_hammer_holder.step` (160 faces, 42 NURBS), +X cap at x=48 |
| Host | 4-vCPU cloud container, Intel Xeon @ 2.10 GHz, Node 22.22.0, `taskset -c 3`, one process per row |
| Drag | 200 ticks (100 with background bodies), outward from 5 mm in 0.2 mm steps, back every 20 ticks |

Each row is one process. The cold first tick (a 1.6–2.4 s move and mesh) is
inside each row's max, but not inside its p50.

## Results

Milliseconds over accepted ticks. No tick was refused.

| Row | Restore p50 | Move p50 / p95 | Validate p50 | Mesh p50 / p95 | Tick p50 / p95 | Slots per tick | Linear memory, first → last tick |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Prepared | 0.37 | 11.66 / 29.16 | 0.12 | 28.74 / 41.09 | 41.87 / 66.30 | 1,906 | 18.7 → 94.9 MB |
| Unprepared | 0.35 | 12.84 / 42.83 | 0.12 | 27.34 / 43.86 | 41.04 / 74.18 | 1,906 | 17.7 → 107.0 MB |
| Prepared, 1,000 unrelated boxes | 3.15 | 11.41 / 57.46 | 0.12 | 27.13 / 54.01 | 42.72 / 132.54 | 1,906 | 39.8 → 96.5 MB |

## Findings

1. **The kernel's per-tick floor on this body is about 41 ms, and mesh is about
   70% of it.** The move is 11–13 ms. Validation and restore are each under
   0.4 ms on a small document.
2. **A prepared move saves little here.** Median move time is 11.66 vs
   12.84 ms. Most of the holder's per-frame move work is after preparation:
   the blend translation, its copy and its postconditions. The p95 gap is
   larger (29 vs 43 ms).
3. **Restore scales with the document; move and mesh do not.**
   - Restore goes from 0.37 to 3.15 ms with 1,000 unrelated boxes.
   - Move and mesh medians do not change.
   - So the 3.3× adapter tick growth that OpenZCAD measured with 200 unrelated
     bodies comes from per-body consumer work, not from the kernel calls in this
     sequence.
4. **Each tick retires about 1,900 arena slots.**
   - Restores retire every slot a tick allocated. Slots are never reused, by
     design, so no stale handle can alias a later entity.
   - After 200 ticks the source kernel holds 385,012 slots, 379,294 of them
     retired.
   - Linear memory rises by 76–89 MB. That is about 0.4 MB per tick, and WASM
     never returns it.
   - The late window is not slower than the early one (tick p50 54.8 vs
     66.3 ms prepared, 38.4 vs 74.2 ms unprepared), so the growth costs memory
     rather than time at this scale.
   - This is the arena-compaction cost PERF-T06 and
     [the deferred E6b design](../design/deferred-e6b-arena-compaction-and-slot-reuse.md)
     describe. It is measured here per interactive drag for the first time.

## Reproduce

```sh
for move in prepared unprepared; do
  taskset -c 3 node scripts/perf/face-drag-session/drag_session_bench.cjs \
    --kernel-pkg crates/wasm/pkg --io-pkg crates/wasm-io/pkg \
    --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
    --ticks 200 --move "$move" --label "$move"
done
taskset -c 3 node scripts/perf/face-drag-session/drag_session_bench.cjs \
  --kernel-pkg crates/wasm/pkg --io-pkg crates/wasm-io/pkg \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --ticks 100 --move prepared --background 1000 --label bg1000
```
