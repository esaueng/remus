# Exact body editing and browser latency

This investigation separates three costs: constructing/editing the exact B-Rep,
tessellating it, and the consumer application's history rebuild and presentation.
The benchmark is deliberately a small, analytic body in a progressively larger
arena. It is a diagnostic for work proportional to unrelated model state, not a
claim about complex imported parts or complete OpenZCAD edits.

## Reproduce

```sh
cargo run --release -p remus-operations --example body_edit_probe
cargo xtask wasm-build
node scripts/bench-body-edit.mjs
python3 -m http.server 8768 --bind 127.0.0.1
```

For the browser measurement, open
`http://127.0.0.1:8768/scripts/bench-body-edit.html` and click **Run benchmark**.
The page runs the same JavaScript benchmark in a dedicated browser worker.
Both JavaScript runners report runtime and package version.

Each case creates a fresh kernel with 0, 1,000, or 10,000 unrelated boxes, then
edits the +Z face of a 10 × 20 × 30 mm box by 5 mm. Extrusion creates a separate
1,000 mm³ prism; face movement and push/pull produce a 7,000 mm³ body. Setup,
handle lookup, validation, and cleanup are excluded from timing. Five warmups
precede 30 measured samples. The probe checks result volume, unchanged source
volume, strict solid validation, and nonempty tessellation on every iteration.
The timing fixture retains no checkpoints. Real history checkpoints can trigger
an additional copy-on-write clone, so the percentage improvement in OpenZCAD may
differ; checkpoint correctness is covered separately by the smoke regression.

Edit and mesh p50/p95 timings are separate. Mesh timing includes the kernel
tessellation call at 0.1 mm linear and 0.5 rad angular deflection, but excludes
JavaScript buffer extraction, worker transfer, GPU upload, and presentation.
Browser clock quantization can report zero for sub-resolution operations; that
does not mean an operation takes no time. Compare runs on an otherwise idle
machine using identical compiler and optimization settings.

## Local measurements, 2026-09-21

Headless Chromium 153 on macOS, 30 measured samples after five warmups. Both
browser binaries were rebuilt from `c21eb02ab3302b1d1098cfc7d267d32b3516f344`
with Rust 1.96.0, SIMD, release LTO, and the repository's `wasm-opt -Oz` settings;
the optimized binary adds only the two transaction-wrapper removals. The
unchanged push/pull operation serves as a control. Native timings use the public
Rust operations API; they do not include the WASM binding layer.

| Unrelated boxes | Operation | Native p50 (ms) | Browser baseline p50 (ms) | Browser optimized p50 (ms) | Browser baseline / optimized p95 (ms) |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0 | Extrude | 0.005 | <0.1 | <0.1 | 0.1 / 0.1 |
| 0 | Move faces | 0.116 | 0.2 | 0.2 | 0.4 / 0.4 |
| 1,000 | Extrude | 0.801 | 1.2 | 0.6 | 1.6 / 0.9 |
| 1,000 | Move faces | 1.778 | 2.1 | 1.6 | 3.2 / 2.1 |
| 10,000 | Extrude | 13.509 | 14.3 | 7.1 | 17.0 / 8.6 |
| 10,000 | Move faces | 18.600 | 22.3 | 15.3 | 26.4 / 24.6 |
| 10,000 | Push/pull (control) | 24.780 | 30.9 | 30.9 | 40.2 / 38.8 |

At 10,000 unrelated boxes, median extrusion time fell about 50%, and median
face-move time about 31%. This is a local synthetic result, not an OpenZCAD
end-to-end speedup claim. Other validation processes were active during these
runs, so absolute timings and especially tail values are indicative. Raw results
and binary hashes are in [body-edit-results.json](body-edit-results.json).

Simple body construction and modification are already sub-millisecond in this
browser fixture. Native execution also slows with unrelated arena size. Together
these observations identify avoidable whole-arena work, rather than a browser-only
barrier to interactive editing. They do not quantify browser overhead for complex
imported geometry or the user's slow model.

## Redundant rollback snapshots

Direct WASM `extrude` previously wrapped the native transactional `extrude` in
another topology snapshot. Direct `moveFaces` did the same around the already
transactional native face-move operation. Each wrapper copied every arena,
including unrelated bodies. This change removes those two redundant boundary
snapshots. Native rollback, validation, tolerances, and checkpoint copy-on-write
remain in force. Batch dispatch is unchanged.

The smoke regression checks byte-identical serialized source geometry after
failed and successful direct edits, exact volumes, checkpoint restoration, and
invalidation of handles created after a checkpoint. Existing native transaction
regressions cover failed operations after allocation.

Full arena copies remain inside native transactions, and face movement can nest
another transaction in the offset layer. This patch reduces that cost; it does
not make edits independent of total arena size. A follow-up should evaluate
transaction ownership or mutation journals with explicit rollback, retired-handle,
metadata, cavity, and checkpoint tests.

## Implications for OpenZCAD

Source inspection of the OpenZCAD checkout at `778d539e` found direct `extrude`
calls in `exact-profile-builders.ts` and direct `moveFaces` calls in
`exact-face-distance.ts`. These are the APIs improved here. That checkout was
behind live main; the live package manifest inspected during this investigation
pinned both Remus packages to `b13ff97b16c405eafd6707ad9098297d867a385f`.
These observations do not establish the version currently serving a user's tab.

In the inspected consumer, editing an early feature replays its changed history
suffix. Changed bodies are then measured, including grouped tessellation, edge
sampling, topology data, and optional feature recognition. Preview requests
already coalesce to one active and one newest pending request. Those app costs
are outside this microbenchmark and still need a trace on a representative
OpenZCAD model before attributing a seconds-long delay.

The next end-to-end trace should separately record queue wait, history restore
and replay, exact edit/Boolean time, validation, tessellation, recognition,
serialization/transfer, viewport installation, and next presentation. Record
cold and warm edits and repeat edits on both a simple body and the slow real
model. For dragging, use a 16.7 ms display-frame budget at 60 Hz separately from
the exact-commit budget; any disposable display preview must preserve the
document's exact B-Rep and export geometry.

WebAssembly runs the same kernel algorithms, so repeated full-model work is
not inherently solved by moving the app to the desktop. A browser worker can
keep computation off the UI thread, and transferable buffers can avoid copies
between worker and UI; neither removes the computation itself. See
[MDN's worker documentation](https://developer.mozilla.org/en-US/docs/Web/API/Web_Workers_API/Using_web_workers)
and [transferable objects](https://developer.mozilla.org/en-US/docs/Web/API/Web_Workers_API/Transferable_objects).
