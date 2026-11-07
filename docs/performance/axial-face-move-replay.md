# Axial face-move replay: certify boundary separation before meshing

The regression is W1's whole-body closed-mesh postcondition on the exact
blend-translation route. It runs when any stretched carrier lacks a full
circular-spring certificate. Ordinary planar walls and partial cylindrical
springs therefore pay to tessellate the entire filleted body for each edit.
The quality-permission route already tries the exact operation first.
Blend removal/recognition was not the dominant cost in this workload.

## Certificate and fallback

For each remaining stretched planar/cylindrical carrier, preparation bounds
the axial levels of every wholly moving and fixed edge. Lines use endpoints,
circles their entire circle, and positive-weight NURBS their control hull.
Both endpoints of every stretched line are included in their respective
moving/fixed sets. Unsupported or overlapping bounds decline certification.
The resulting open travel interval keeps each moving/fixed pair separated
throughout the edit; the interval contains zero, so checking its endpoint
also proves the whole intervening motion.

Moving curves cannot newly cross one another under a rigid translation;
fixed curves cannot newly cross one another either. The remaining edges are
parallel stretched lines, already checked for collapse/reversal. A moving
curve can enter a stretched segment only through its fixed endpoint; a fixed
curve can enter only through the moving endpoint. The interval bounds these
endpoint contacts too. Parallel stretched segments can acquire an overlap
only through such an endpoint contact. This proves that the edit introduces
no new carrier-boundary crossings, including on a periodic cylinder without
unrolling its chart. It assumes valid source geometry; it is not a validator
for arbitrary pre-existing self-intersections.

Only a distance strictly inside every carrier's interval skips the mesh.
Anything else retains the original whole-body mesher, deflection and welded
watertightness gate. Existing full-spring axial bounds, swept-region checks,
line-collapse checks, volume checks, strict validation, quality permission,
journaling and rollback are unchanged. No geometry construction changes.

## Reproduction and scope

The public fixture uses a 32.75 × 18 × 26 block fused to an R6 × 35 boss,
with five junction edges filleted R5. Replay moves the block top by −6.28,
then twenty alternating +0.4/−0.3 edits. Setup and final volume measurement
are outside the timer. This is a native equivalent of the application's
width +2.75 cold replay, not a complete application sync. It produces 19
faces with this fixture construction, versus 15 reported by the application;
the final volume agrees (15537.6959805061). The consumer worktree was read-only.

```sh
cargo test --profile profiling -p remus-operations --test axial_replay_perf \
  profile_axial_history_replay -- --ignored --nocapture
RUSTFLAGS='-C target-feature=+simd128' wasm-pack build crates/wasm \
  --target nodejs --release --out-dir /path/to/scratch-package \
  -- --no-default-features
node scripts/axial-replay-profile.mjs /path/to/scratch-package/remus_wasm.js
```

Native uses the profiling profile (optimized, debug symbols, no LTO) with
library caches at their default disabled settings. Node uses SIMD, wasm-opt,
the same build flags on both revisions, and the binding's default caches.
Three Node runs use fresh kernels in one module: run 0 includes cold JIT and
measurement caches; runs 1–2 have warmed module caches. Node 22.22.1, Rust
1.96.0, WSL2 on a shared Windows host. Times are machine/load dependent.
The baseline is `de6417ce` (the latest main at branch creation).

## Measurements

Native baseline: 21 journaled moves took **12,933.253 ms** and
**12,335.566 ms**. In the first run, the mesh-closure span took
12,100.550 ms, blend remove/rebuild attempts 2.104 ms, translation-copy
3.860 ms, and final strict validation 795.737 ms. Spans overlap and must
not be summed. The second run also captured a native perf recording (1,277
samples). This WSL perf build lacks automatic symbol/unwind support.
Resolving leaf offsets against perf's preserved baseline ELF with
`addr2line` identifies curved-face meshing and CDT work; enclosing-path
attribution comes from the native monotonic-clock phase trace.

Three subsequent interleaved native runs (same preserved baseline executable
and final candidate, alternating order) gave these totals for 21 calls:

| Run | Baseline ms | Candidate ms |
|---|---:|---:|
| 0 | 35489.480 | 2473.568 |
| 1 | 21479.523 | 1577.351 |
| 2 | 22617.678 | 1192.009 |

Median **22617.678 → 1577.351 ms (14.3×)**. The shared host was also
compiling/running validation work; the first round overlapped a Node probe.
An earlier final-candidate isolated run took 892.970 ms. In every candidate
run the closure mesh disappeared; strict validation accounts for most of
the remaining native cost. Geometry, face count and final volume agree in
all runs. Native cache defaults differ from the WASM binding's defaults.

WASM/Node, 21 `moveFacesJournaledWithQuality` calls per run, freshly built
baseline and final packages with identical flags:

| Run | Module state | Baseline ms | Candidate ms |
|---|---|---:|---:|
| 0 | First run | 24716.540 | 606.988 |
| 1 | Warm module, fresh kernel | 27195.938 | 91.234 |
| 2 | Warm module, fresh kernel | 41671.715 | 108.576 |

The final sequential candidate run was taken after this task's heavy
validation/profiling work finished. Its warmed calls average **4.34–5.17 ms**,
near the consumer's reported old-pin average of 5.09 ms. Baseline timings
still reflect variable shared-host load; the exact ratio is not a universal
latency claim. Additional final-candidate diagnostic triplets under heavy
load were 2350.991/345.093/356.541 ms and, overlapping native profiling,
1785.878/358.939/373.574 ms.

All six baseline/final sequential results have 19 faces, volume
15537.695980505712, exact construction quality on every edit, and identical
serialized geometry (journal excluded): SHA-256
`1c73aa20a89d387c4ff11b57979297aac7cac5d93f02af46f496493ef311491d`.
The warmed baseline's closure span consumes essentially the whole move;
the candidate takes the boundary certificate and never enters that span.

Paired raw WASM sizes: **12860718 → 12865642 bytes (+4924)**. No dependency
or public binding was added. This optimizes the existing core session edit.

## Regression coverage

- `axial_replay_preserves_exact_geometry_and_history`: all 21 edits remain
  exact; ordinary and quality-aware journaled edits agree on serialized
  geometry and every evolution table; the source geometry survives; strict
  validation and the independently measured final-volume oracle pass.
  The trace requires 42 certified routes (both entry points) and zero
  whole-body closure meshes, a deterministic performance regression gate.
- `scoped_closure_matches_whole_body_guard`: certificate results and complete
  document state match a forced original whole-body check, both move signs,
  with and without the WASM-style memos.
- Closed-form box travel bounds test both normal directions and touching
  limits; overlapping source hulls decline.
- The existing W1 shoulder-crossing, line reversal, prepared-move and imported
  body regression suites remain applicable. No assertions are weakened.

Local validation: workspace check with all targets; workspace clippy with
all targets/features and warnings denied; 3,896 operations tests passed
(32 skipped); three imported-body tests and five WASM face-move contract
tests passed. The ignored native timing test was run separately. Hosted CI and consumer application replay are
separate from these local checks.
