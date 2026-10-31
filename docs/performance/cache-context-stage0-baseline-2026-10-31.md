# Cache context stage 0: warm-cache baseline (2026-10-31)

Stage 0 of [RFC 0009](../design/rfc-0009-cache-context.md). It adds the
measurements that later stages are judged against, and changes no production
code:

- the `warm` family of `scripts/performance/run.py`: four edit sequences that
  reuse the per-thread caches, each run natively and through the committed
  kernel package;
- a check on every warm-cache case that each sample records the same per-cache
  hit, miss and eviction counts;
- `scripts/performance/cache_stats.py`, which compares those counts between
  two runs.

A stage that moves a cache family into the cache context has to leave the
counts identical (`cache_stats.py` exits 0). Its medians must also stay within
the run-to-run spread recorded below (owner decision 1 of the
[size architecture](../design/kernel-wasm-size-architecture.md)).

## Workloads

Each workload has a native twin (`crates/wasm/examples/performance_baseline/warm.rs`)
and a WASM twin (`scripts/performance/warm.cjs`) that make the same calls in
the same order.

- **Fixture.** The holder workloads load the Shapr3D hammer holder from one
  exact arena document. The runner writes it with the native worker and
  records its hash, so both twins start from the same handles.
- **Native twin.** It runs the library with the kernel constructor's 11 cache
  switches on.
- **WASM twin.** It calls the package's bindings.
- **Every sample starts with empty caches.** Natively the caches are cleared
  and checked empty. In WASM a fresh module instance is loaded. Only the
  timed phases are counted.

| Case | Phases, in order | Caches it reaches |
|---|---|---|
| `warm_measure` (holder) | strict validation cold, repeated, and of a deserialized copy in a second document (WASM: a probe kernel); face areas ×160 cold and repeated; volume cold and repeated | face integrals, measurement reuse, NURBS area, face mesh, volume |
| `warm_edit` (holder) | display mesh at 0.0148 cold and warm; edge relations; feature recognition; `moveFaces` of the +X cap by −6; a second drag tick of the same source face to −5; mesh, relations, recognition and strict validation of the moved body | crossing, face box, face integrals, face mesh, mesh stage, measurement reuse, NURBS area, volume |
| `warm_cuts` (holder) | cut a slot, cut a hole, cut the same slot again, all on the original body | pcurve, wire area, ray geometry, face integrals, volume |
| `warm_plate_12` | drill 144 holes in a 100 × 100 × 5 plate one cut at a time; then 6 slots between hole columns (first, rest) | pcurve, wire area |

Correctness checks on every sample:

- strict validation passes;
- each repeated reading has the same bits as the first;
- the holder volume is 50 245.22 within 1e-6 relative;
- the moved and retick volumes are ordered;
- the repeat cut has the same volume bits as the first cut;
- the plate volume matches its closed form within 1e-9 relative.

Across the four workloads, every cache in the kernel constructor's list
records hits except the NURBS-area memo; see the observations below.

## Results

- **Machine.** 4-vCPU Intel Xeon cloud container (Linux 6.18), load 1.0–1.3.
- **Toolchain.** rustc 1.96.0, Node 22.22.0, `profiling` build.
- **Inputs.** Committed package v2026.1.93; source `3e6cfdf` plus this
  change; arena `2d777155…`.
- **Sampling.** Three processes × five retained samples after one warmup,
  one Rayon thread. Two complete runs, `a` then `b`. Medians in ms.

| Case | run a | run b | per-process medians (a) |
|---|---:|---:|---|
| `native_warm_measure` | 618.3 | 618.1 | 623.6 / 616.7 / 618.2 |
| `native_warm_edit` | 780.0 | 776.0 | 772.6 / 782.0 / 785.2 |
| `native_warm_cuts` | 55.9 | 55.9 | 56.2 / 55.7 / 56.5 |
| `native_warm_plate_12` | 123.5 | 122.0 | 123.8 / 122.8 / 124.5 |
| `wasm_warm_measure` | 1199.4 | 1205.6 | 1195.2 / 1210.9 / 1199.4 |
| `wasm_warm_edit` | 1483.3 | 1482.6 | 1490.3 / 1474.7 / 1487.3 |
| `wasm_warm_cuts` | 92.5 | 91.9 | 92.5 / 92.4 / 92.7 |
| `wasm_warm_plate_12` | 181.6 | 180.0 | 182.9 / 181.3 / 180.3 |

The two runs' medians differ by at most 1.2 % (`native_warm_plate_12`). That is
the noise floor a later stage is compared against on this machine. A stage
reruns the baseline on its own machine instead of comparing against these
numbers. `cache_stats.py stage0-a stage0-b` reports identical counters.

Phase medians, run a (ms):

| Phase | native | WASM |
|---|---:|---:|
| `validate_cold` / `_warm` / `_copy` | 401.9 / 1.2 / 3.8 | 784.3 / 1.7 / 6.6 |
| `areas_cold` / `_warm` | 19.0 / 0.003 | 34.6 / 0.04 |
| `volume_cold` / `_warm` | 192.4 / 0.003 | 370.9 / 0.01 |
| `mesh_cold` / `_warm` / `_moved` | 81.5 / 17.2 / 17.9 | 115.4 / 23.0 / 26.2 |
| `relations_source` / `_moved` | 88.4 / 19.7 | 201.0 / 39.2 |
| `recognize_source` / `_moved` | 113.9 / 20.2 | 245.2 / 40.4 |
| `move` / `move_retick` | 406.1 / 8.8 | 776.1 / 14.8 |
| `validate_moved` | 1.3 | 1.7 |
| `cut_slot` / `cut_hole` / `cut_slot_repeat` | 27.0 / 13.0 / 15.8 | 44.3 / 21.5 / 26.4 |
| `drill` / `slot_first` / `slots_rest` (5) | 47.7 / 12.8 / 62.7 | 67.0 / 19.3 / 95.8 |

Native counters, every sample (counters that stayed 0 omitted):

| Case | Counters |
|---|---|
| `warm_measure` | face integrals 492 hits / 389 misses / 30 translated; face mesh 42 misses; measurement reuse 160 identity hits / 160 misses, 1 volume identity hit; NURBS area 42 misses; volume 1 hit / 1 miss |
| `warm_edit` | crossing 21 636 hits / 22 972 misses; face box 132 / 132; face integrals 853 hits / 429 misses / 421 re-referenced / 146 translated; face mesh 281 exact / 13 translated / 212 misses / 4 refused; mesh stage 1 boundary, 329 edge, 10 planar, 1 weld, 1 cleanup hit, 60 evictions; measurement reuse 131 content hits / 189 misses; NURBS area 44 misses; volume 2 misses |
| `warm_cuts` | pcurve 8 / 4; ray geometry 8 / 4; wire area 72 / 37; face integrals 328 hits / 322 misses / 24 translated; volume 1 hit, 1 content hit, 2 misses |
| `warm_plate_12` | pcurve 1 440 / 288; wire area 720 / 144; volume 1 miss |

The WASM twins record what the package exposes: mesh-stage reuse,
measurement reuse and the volume memo. RFC 0009 stage 1 adds a binding for
the rest (`cacheMemoryStats`).

## Observations for the later stages

- **The NURBS-area memo gets no hits while face-area reuse is on.**
  `measure::face_area` consults face-area reuse first (`measure/area.rs:53`).
  The NURBS-area memo only answers that path's misses, and in these
  workloads every miss is a face whose content changed, so the memo has
  nothing to hit. It takes 44 misses and no hits in `warm_edit`, while reuse
  answers 131 of the 320 area requests. `perf-o06-recognition-graph.md` records 40/42 hits per
  edit; that record does not say whether face-area reuse was on. The memo is
  about 18.7 KiB of WASM (RFC 0009 §2.5), and stage 4, which moves both,
  should decide whether it still earns its place.
- **Translated readings where none were expected.** `warm_measure` changes no
  geometry, yet it takes 30 *translated* face-integral readings. The holder
  has congruent features, and one face's integral answers its translate's
  request. RFC 0009 §2.3 treats such readings as value-equivalent, not exact.
  So even a body that is never edited reads some integrals from a different
  face.
- **Native and WASM counters are not compared.** The mesh stage of
  `warm_edit` evicts 60 entries natively and 32 in WASM. The likely cause is
  the retained-byte estimates: they use `size_of`, which is smaller on
  `wasm32`, so the same byte budget holds more entries. That has not been
  confirmed. The committed package may also predate the native source (see
  `baseline.md`). The gate compares each runtime only with itself.
- **Holder volume, last bits.** The holder volume comes out as
  `50245.219468635645` natively and `50245.219468635754` in WASM: equal
  within the reference tolerance, but different in the last bits.

## Reproduce

```bash
python3 scripts/performance/run.py --family warm --output target/performance/stage0-a
python3 scripts/performance/run.py --family warm --output target/performance/stage0-b
python3 scripts/performance/cache_stats.py target/performance/stage0-a target/performance/stage0-b
```

A stage runs the same commands on its base, then on its head, and compares the
two with `cache_stats.py`.
