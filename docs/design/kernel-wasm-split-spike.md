# Spike: profile-guided module splitting of the browser kernel (`wasm-split`)

Status: **research spike, complete; evidence record only**. Recommendation:
**do not adopt the split build as is**: see [Recommendation](#recommendation).
Recording this result does not activate module splitting. Builds on the
[browser kernel size architecture](kernel-wasm-size-architecture.md), which
rejected the W4 lazy modules: they cross as arena documents, lose handle
identity and add a 1.5–2 µs round trip.

## Question

Can Binaryen's `wasm-split` (profile-guided module splitting, as Emscripten's
`SPLIT_MODULE` uses) turn the kernel into an eager primary module and a
deferred secondary module? The two would share memory and the function tables,
so handles, the journal and checkpoints keep working. The aim is to cut what
must be downloaded and compiled before first interaction, without slowing
steady-state performance (owner decision 1 of the size architecture).

## Short answer

- **It works mechanically with wasm-bindgen output.** The split kernel passes
  `scripts/test-wasm-smoke.mjs` in full: lazy loading, eager loading, and
  three different profiles. One `RemusKernel` session spans both modules with
  no document round trip. The glue needs a hand-written loader (about 60
  lines). wasm-bindgen has no support of its own, and the bundler entry has
  to be replaced.
- **It is not free in speed.** Steady-state warm workloads are mostly within
  noise. Cold and short operations are measurably slower, even when every
  function they run is in the primary: most transactions-suite cells are
  slower beyond noise (up to +27%), and the GFA fuses are +7 to +23% on
  every profile. This breaks the
  no-regression rule. Part of the cause is Binaryen re-encoding the binary
  without the final `-Oz` emission; re-optimizing each half afterwards fixes
  some workloads and makes others worse.
- **The size win depends entirely on the profile.**
  - A broad profile (smoke + performance suites) keeps 84% of the bytes
    eager: −592 KB gzip eager, or −14.5%.
  - A narrow profile (performance suites only) keeps 28% eager: −2.94 MB
    gzip. But then most operations (`makeCylinder`, `cut`, even
    `lastPanicMessage`) run through the deferred module. The first such call
    either loads 8 MB synchronously or fails.
  - Either way the two halves together are 1.5% larger raw and 2.6–3.1%
    larger gzip than the unsplit module.

## Setup

| Item | Value |
| --- | --- |
| Source | `c91287d` (main; glue byte-identical to the committed v2026.1.86 package glue apart from the `.wasm` path). Measured before #1358 renamed the kernel class to `RemusKernel`; the evidence scripts now use the new name. |
| Rust / wasm-bindgen / wasm-pack | 1.96.0 / 0.2.126 / 0.15.0 (`cargo install wasm-pack --version 0.15.0 --locked`) |
| Binaryen | `version_123` release (`wasm-opt`, `wasm-split`) |
| Node / V8 | 22.22.0 / 12.4.254.21 |
| Browser | Playwright Chromium 153.0.8010.12, headless |
| Machine | 4-vCPU cloud container (shared; not a qualified benchmark host) |

Build, mirroring `xtask/src/wasm.rs` (same encoded RUSTFLAGS: `-Dwarnings`,
the three `--remap-path-prefix` maps, `+simd128`):

```sh
cd crates/wasm
wasm-pack build --target bundler --release --out-dir $S/pkg -- --no-default-features
wasm-pack build --target nodejs --release --no-opt --out-dir $S/pkg-node -- --no-default-features
wasm-opt $S/pkg/remus_wasm_bg.wasm -o $S/base/opt.wasm -Oz --enable-simd
```

wasm-pack 0.15.0 wants Binaryen `version_117` for its own `wasm-opt` step.
The download failed in this sandbox, so the bundler build kept the unoptimized
wasm-bindgen output, and `wasm-opt -Oz --enable-simd` from version_123 was
applied by hand. The result is within 0.3% of the committed binary (built
from `d819503`, two commits earlier, with version_117):

| Module | Raw | gzip | Brotli q11 |
| --- | ---: | ---: | ---: |
| Committed `crates/wasm/pkg` (v2026.1.86) | 11,005,504 | 4,083,667 | 2,907,310 |
| This baseline (`c91287d`, Binaryen 123) | 10,975,175 | 4,080,184 | 2,905,126 |

All sizes here come from `node scripts/wasm-size-policy.mjs` (Node
`zlib.gzipSync()` defaults, Brotli q11), as the policy and the consumer
measure them.

## Procedure

```sh
wasm-split --instrument base/opt.wasm -o base/instr.wasm
# Node glue + profile-hook.js: on process exit, __write_profile into a malloc'd
# buffer and write it to $WASM_SPLIT_PROFILE_DIR/<pid>-<ts>.prof
WASM_SPLIT_PROFILE_DIR=profiles/smoke node scripts/test-wasm-smoke.mjs   # instrumented pkg swapped into crates/wasm/pkg
python3 profile_perf.py variants/instr profiles/perf                     # all 50 WASM workers, 1 sample + 1 warm-up
wasm-split --merge-profiles profiles/smoke/*.prof -o smoke.prof
wasm-split --merge-profiles profiles/perf/*.prof -o perf.prof
wasm-split --merge-profiles profiles/smoke/*.prof profiles/perf/*.prof -o all.prof
wasm-split --split base/opt.wasm --profile=all.prof -o1 all.primary.wasm -o2 all.secondary.wasm --placeholdermap
```

The performance workers profiled are every WASM case of
`scripts/performance/workloads.json` (12), the `sketch` WASM worker at
`independent_solved`, `coupled_chain` and `drag` (14 cells; the worker accepts
nothing else), and the `transactions` WASM worker (24 cells).

Three profiles:

| Profile | Workloads | Functions kept in primary (of 7,527) |
| --- | --- | ---: |
| `smoke` | `test-wasm-smoke.mjs` | 5,245 |
| `all` | smoke + performance workers | 5,259 |
| `perf` | performance workers only | 1,162 |

The smoke test runs 70% of all functions. It is a broad correctness corpus,
not a "first interaction" trace. `perf` is the closer stand-in for "primitives,
a boolean, a transform, a sketch solve".

## Sizes

| Module | Raw | gzip | Brotli q11 |
| --- | ---: | ---: | ---: |
| Unsplit baseline | 10,975,175 | 4,080,184 | 2,905,126 |
| `smoke` primary | 9,212,602 | 3,474,782 | 2,511,658 |
| `smoke` secondary | 1,929,713 | 714,097 | 522,974 |
| `all` primary | 9,265,971 | 3,487,683 | 2,518,500 |
| `all` secondary | 1,876,695 | 700,297 | 513,873 |
| `perf` primary | 3,145,230 | 1,144,539 | 863,082 |
| `perf` secondary | 7,997,768 | 3,060,261 | 2,154,188 |
| `all` primary, re-run through `wasm-opt -Oz` | 9,214,947 | 3,469,660 | 2,497,791 |
| `all` secondary, re-run through `wasm-opt -Oz` | 1,872,247 | 697,050 | 511,571 |
| `perf` primary, re-run through `wasm-opt -Oz` | 3,119,029 | 1,139,782 | 858,466 |
| `perf` secondary, re-run through `wasm-opt -Oz` | 7,976,933 | 3,043,022 | 2,138,704 |

| Profile | Eager (primary) vs unsplit, gzip | Primary + secondary vs unsplit, raw / gzip / Brotli |
| --- | ---: | ---: |
| `smoke` | −605,402 (−14.8%) | +167,140 / +108,695 / +129,506 |
| `all` | −592,501 (−14.5%) | +167,491 / +107,796 / +127,247 |
| `perf` | −2,935,645 (−71.9%) | +167,823 / +124,616 / +112,144 |

The fixed overhead of about 167 KB raw comes from the cross-module interface:

- `all`: the primary gains 1,780 placeholder imports, and 1,328 extra function
  exports for the secondary to import (minified names).
- `perf`: 2,729 placeholders and 776 extra exports.

Size policy: the `all` primary would sit about 2.27 MB raw under the 11 MiB
review line. Today `scripts/wasm-size-policy.mjs` measures a single module.
Whether the policy should cover the primary alone, or primary plus secondary,
is an owner decision. Splitting must not become a way around the gate: the
secondary still has to be downloaded before any deferred operation.

## Glue: what works, what had to change

### Module structure after the split

- **Placeholders.** The primary imports `placeholder.<slot>`, one function per
  deferred function. Calls from primary code to deferred functions become
  `call_indirect` through a new funcref table (1,780 slots for `all`).
  Binaryen created that table because the kernel's existing function table
  has a fixed size (`1251 1251 funcref`). Each slot starts out holding its
  placeholder.
- **Secondary instantiation.** The secondary imports `memory`, both funcref
  tables, the stack-pointer global and the primary functions it calls, all
  from namespace `primary`. Its active element segment overwrites the
  placeholder slots with the real functions, so after one instantiation every
  later call is a plain `call_indirect` and never touches JS again.
- **Shared state.** Memory, the allocator, the externref table
  (`__wbindgen_externrefs`) and every global are shared. One `RemusKernel`
  object, with its handles, journal and checkpoints, spans both modules.
- **No exports moved.** The 432 wasm-bindgen exports stay exported from the
  primary under their original names. Exported functions that moved to the
  secondary become trampolines. The wasm-bindgen JS glue therefore needed no
  change beyond instantiation.
- **No blockers in the module.** The kernel contains no `ref.func` in code
  (Binaryen version_116's splitter still asserted on that) and has one active
  element segment at a constant offset. wasm-bindgen 0.2.126's abort-handling
  globals (`__abort_handler`, `__instance_terminated`) are shared like any
  other global.

### Node (`remus_wasm_node.cjs`)

- **The change.** The glue's one
  `new WebAssembly.Instance(wasmModule, __wbg_get_imports())` is replaced by
  [`split-loader.js`](evidence/wasm-split-spike/split-loader.js). It adds the
  `placeholder` namespace, and each placeholder does the following:
  1. loads the secondary synchronously (`readFileSync` + `new WebAssembly.Module`);
  2. instantiates it against the primary's exports;
  3. calls `table.get(slot)(...args)`.

  `preload()` does the same asynchronously (`fs.promises.readFile` →
  `WebAssembly.compile`). That is the idle preload, and it works.
  [`make-split-variant.py`](evidence/wasm-split-spike/make-split-variant.py)
  writes the package: the primary, the secondary, `remus_wasm_split.json` (the
  placeholder table's export name) and the patched glue.
- **Smoke results** (whole `scripts/test-wasm-smoke.mjs`, split package
  swapped into `crates/wasm/pkg`):

  | Variant | Load mode | Result | Wall time |
  | --- | --- | --- | ---: |
  | Unsplit baseline | — | pass | 237 s |
  | `perf` | lazy, sync load on first deferred call | pass | 245 s |
  | `all` | eager, secondary loaded at start | pass | 239 s |
  | `smoke` | lazy | pass | 238 s |

- **First deferred call, lazy.** A synchronous load of the 8 MB `perf`
  secondary took 20–22 ms in Node. It happens inside the user's operation.
- **Call before load, `throw` mode.**
  - What happened: the operation failed with a clear error; `preload()` then
    succeeded, and the kernel and its existing handles stayed usable
    ([`throw-then-load.cjs`](evidence/wasm-split-spike/throw-then-load.cjs)).
  - Why it is still unsafe: a JS exception thrown through wasm frames runs no
    Rust destructors and skips the kernel's transaction rollback. "Throw until
    loaded" is safe only at entry points that throw before they mutate
    anything, and a profile-guided split cannot promise that. Even
    `lastPanicMessage` was deferred in the `perf` split.

### Browser

- **Bundler entry.** wasm-pack's bundler entry (`remus_wasm.js`:
  `import * as wasm from "./remus_wasm_bg.wasm"`) does not work. The bundler's
  ESM integration would resolve the primary's `placeholder` imports as a JS
  module. [`browser/split-entry.js`](evidence/wasm-split-spike/browser/split-entry.js)
  replaces it: it uses `compileStreaming` on the primary, builds the
  placeholders from `WebAssembly.Module.imports`, calls `__wbg_set_wasm` and
  `__wbindgen_start`, and preloads with `compileStreaming` on the secondary.
  The unchanged `remus_wasm_bg.js` glue works with it in Chromium:
  `makeBox` → `makeCylinder` → `cut` → `volume` gives the same volume as the
  unsplit kernel.
- **Call before load in a document.** Main-thread JS has no good synchronous
  fetch. Sync XHR is deprecated and cannot return an `ArrayBuffer` from a
  document; the prototype uses the `x-user-defined` text hack. More important
  is the compile limit: Chromium refuses synchronous `new WebAssembly.Module`
  on the main thread above 8 MB.
  - Measured with `browser/synclimit.cjs`: the 10,975,175 B unsplit module
    fails with `RangeError: WebAssembly.Compile is disallowed on the main
    thread, if the buffer size is larger than 8MB`.
  - The `perf` secondary, 7,997,768 B, compiles in 13 ms. It is 390 KB under
    the limit, so growth that lands in the secondary would cross it within
    weeks at the current rate (estimate).
  - On the main thread, then, a narrow profile means "preload and hope, or
    throw". In a Worker, synchronous XHR and synchronous compile are allowed,
    so the lazy-sync fallback works there.
  - Measured with `browser/test.html`: a lazy sync load of the `perf`
    secondary took 125 ms. With preload instead, no placeholder ran.
- **`--jspi`.** wasm-split's `--jspi` mode loads the secondary asynchronously
  through JS Promise Integration. That only works if every export is wrapped
  in `WebAssembly.promising`, which turns the synchronous wasm-bindgen API
  async. It is incompatible with today's API and was not tried.

### Compile and instantiate before first interaction (Chromium)

Measured with `browser/loadtime.cjs`: fresh context per run, download
excluded, 10 runs, median (range), in ms:

| Module | Compile, CPU ×1 | Instantiate, CPU ×1 | Compile, CPU ×4 | Instantiate, CPU ×4 |
| --- | ---: | ---: | ---: | ---: |
| Unsplit | 20.6 (19.8–22.9) | 0.9 (0.8–0.9) | 39.0 (35.9–40.2) | 4.2 (3.6–5.4) |
| `all` primary | 17.8 (17.6–19.9) | 3.4 (3.3–3.7) | 33.0 (30.1–33.5) | 15.5 (14.4–21.2) |
| `perf` primary | 6.5 (6.3–6.9) | 4.0 (3.7–4.1) | 12.2 (10.6–17.6) | 17.4 (16.5–23.9) |
| `all` secondary | 4.6 (4.3–5.8) | — | 9.4 (7.6–10.4) | — |
| `perf` secondary | 16.1 (15.3–18.2) | — | 28.1 (25.0–29.6) | — |

- **Compile.** V8 compiles wasm lazily, so "compile" here is mostly
  validation. Even the unsplit kernel costs only 21 ms (39 ms at ×4).
- **Instantiate.** It grows by 2.5–13 ms because each placeholder import
  needs its own JS-to-wasm import wrapper.
  - `all` at CPU ×4: the ready time gets 5 ms *worse*
    (33.0 + 15.5 = 48.5 ms vs 43.2 ms).
  - `perf` at CPU ×4: it saves about 14 ms.
  - Binaryen's `--no-placeholders` would remove the wrappers but turn every
    call before load into a trap (not tried).
- **Download.** This is where the split could matter. The size
  architecture's W6 measurement (OpenZCAD #657) found mobile-class cold load
  download-dominated, at about +89 ms per 100 KB gzip. On that rate
  (**estimate**, not measured here), the eager download shrinks by about
  0.53 s for `all` and about 2.6 s for `perf`, on the 3.5 s p95 mobile cold
  load. Desktop cold load (47–77 ms p95) has little to gain.

## Steady-state speed

### Method

[`bench.py`](evidence/wasm-split-spike/bench.py) runs the WASM workers of the
`scripts/performance` suites directly, with the suites' own argument shapes
and validation. Each split variant runs in eager mode (secondary loaded at
start, the state after an idle preload).

- **Arms:** two identical baseline arms (`base-a`, `base-b`), plus `all`,
  `smoke` and `perf` splits.
- **Repetition:** 10 fresh processes per arm per case, in rotated order.
- **Samples:** `main` and `sketch` take 20 samples after 5 warm-ups per
  process. `transactions` takes one cold operation per process, so 10
  samples per arm.
- **Noise:** the difference between the `base-a` and `base-b` medians, and
  the interquartile range of the per-process medians of both base arms. A
  delta counts as slower beyond noise when it exceeds both that difference
  and half that IQR.

The full 50-row table is in
[`bench-summary.md`](evidence/wasm-split-spike/bench-summary.md).

### Results (default V8 tiering, pooled median delta vs. base)

- **`main`, warm (12 cases):**
  - `wasm_bool_fuse_50` (1.05 ms) is slower beyond noise:
    `all` +9.9%, `smoke` +3.5%, `perf` +14.7%, with noise 2.4% and IQR 3.9%.
  - Three later diagnostic runs repeated it for `all`: +9.5, +14.0 and +6.9%.
  - Two `perf` transform cells also exceed the noise rule:
    `wasm_transform_direct_50` +1.4% and `wasm_transform_direct_400` +1.1%.
    The other transform and fuse/refusal cells are within noise or faster.
  - The 200- and 400-body fuses are within noise or faster.
- **`sketch`, warm (14 cells):**
  - Within noise apart from small slowdowns: `independent_solved_100` +1.4 to
    +2.0%, and `coupled_chain_1000` +1.5 to +3.9% (`all`, `smoke`), against
    noise of 0.2–1.0%.
  - `drag_100` (97 ms) and the other long solves are within noise.
- **`transactions`, cold (24 cells):** slower beyond noise on every split
  variant in most cells.

  | Cells | `all` | `smoke` | `perf` | Base noise |
  | --- | ---: | ---: | ---: | ---: |
  | `direct` (6) | −2.5 to +5.5% | −1.6 to +7.5% | +0.7 to +7.3% | 0.7–2.5% |
  | `batch` (6) | −12.8 to +24.4% | −12.7 to +8.5% | +5.3 to +26.6% | 0.2–2.7% |
  | `gfa-direct` (6) | +9.0 to +21.8% | +11.0 to +21.8% | +12.2 to +20.3% | 0.2–1.8% |
  | `gfa-batch` (6) | +7.1 to +21.7% | +9.4 to +21.4% | +15.7 to +22.5% | 0.6–2.6% |

The negative `batch` cells are the 1000-body ones on `all` and `smoke`
(−5.9 to −12.8%); `perf` is slower there too (+5.3 to +9.7%). The `all`
profile includes these exact workloads, so the regression is not deferred
code being called.

### Diagnosis (12 processes per arm, same method)

| Case (base median) | `all` eager | `all` lazy | `all` throw | Binaryen round trip, unsplit | `all` + `-Oz` | `perf` + `-Oz` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `tx/gfa-direct/0` (71 ms) | +22.2% | +21.3% | +21.5% | +17.1% | +2.6% | +4.1% |
| `tx/batch/100/cp` (9.3 ms) | +18.8% | +19.1% | +20.0% | — | +10.7% | +24.1% |
| `main/wasm_bool_fuse_50` (1.0 ms) | +9.5% | +0.7% | +7.7% | −1.3% | +15.6% | +31.0% |

- **Not the secondary.** `throw` mode never loads the secondary, and no
  placeholder ran, yet the slowdown is still there. The primary itself is
  slower.
- **Re-encoding is part of it.** `wasm-opt base/opt.wasm -o roundtrip.wasm`
  (no passes, nothing split) is 39,739 B larger and +17% on the cold GFA
  fuse. Binaryen without an optimization level writes the binary without the
  final `-Oz` Stack IR emission. V8's baseline tier (Liftoff), which runs
  cold code, is sensitive to that. Turning Stack IR emission back on by flag
  closes most but not all of the size gap: 10,997,737 B. A full
  `wasm-opt -Oz` of each half afterwards fixes the cold GFA fuse, but makes
  `bool_fuse_50` and `tx/batch/100` worse. It re-optimizes code that was
  already optimized once, and the result moves in both directions.
- **The split itself is the other part.** The round trip leaves
  `bool_fuse_50` within noise; the split does not. Under
  `node --no-liftoff --no-wasm-dynamic-tiering` (TurboFan only), warm
  `bool_fuse_50` is within 2.4% for every variant. Cold operations stay
  slower: +5.9 to +11.6% (`all`) and +8.3 to +22.4% (`perf`) on
  `tx/direct/0`, `tx/batch/100/cp` and `tx/gfa-direct/0`, against a round
  trip within +3.6%. So the split primary costs more on first execution
  (lazy compile plus the baseline tier) even when no deferred code runs.
- **Possible causes, not isolated:**
  - the `call_indirect` sites that replace direct calls inside primary
    functions, which V8 cannot inline;
  - the 1,780–2,729 placeholder imports;
  - the changed function order.

  Finding out which needs per-function V8 profiling and is beyond this spike.

## Recommendation

**Reject as is.** It does not meet owner decision 1, and the safe
configuration buys too little.

1. **Speed.** Every split variant is slower beyond noise on cold and short
   operations: GFA fuses +7 to +23%, the 1 ms 50-body fuse +4 to +15%. That
   includes profiles that contain those very workloads. No fix was found
   that is neutral everywhere.
2. **Size win vs. risk.**
   - The profile that keeps every exercised operation eager (`all`) moves
     only 14.5% of the gzip bytes out of the first download (≈ 0.5 s on the
     mobile cold load, estimate). Total bytes grow by 2.6% gzip.
   - The profile that moves most bytes out (`perf`) defers 72%. But
     ordinary operations then depend on the secondary, so the consumer must
     preload it before any non-trivial edit. If a call comes first, it must
     either throw mid-operation (unsafe), or block on an 8 MB fetch and
     compile. On the main thread that compile is 390 KB from Chromium's
     8 MB limit.
3. **Growth.** It does not change the 0.7 MB/week trend. New capabilities
   land in whichever half the profile puts them. The policy question (gate
   the primary or the total) has to be settled first, or a split simply
   moves the gate.

**What would reopen it:**

- the slowdown of the primary on its own root-caused and removed, with
  `transactions`, `main` and `sketch` all within noise;
- the kernel running in a Worker in the consumer, so a synchronous fallback
  load is safe and not limited to 8 MB;
- a load-time measurement with OpenZCAD `measure-remus-wasm-load.mjs`
  showing the mobile gain is real;
- an xtask/CI step that reproduces the profile deterministically. The
  profile is tied to the exact module bytes by hash, so it has to be
  regenerated on every build.

## Evidence

[`evidence/wasm-split-spike/`](evidence/wasm-split-spike/) holds the prototype
glue and the scripts as run. They use absolute scratch paths
(`/tmp/claude-0/split`) and are not wired into any build:

| File | Purpose |
| --- | --- |
| `profile-hook.js` | Dumps the `--instrument` profile on Node exit |
| `split-loader.js` | Node loader: placeholders, lazy sync load, `preload()`, `throw` mode |
| `make-split-variant.py` | Writes a split package dir from a primary/secondary pair |
| `perf_cases.py`, `profile_perf.py`, `bench.py`, `summarize.py` | The WASM worker matrix, profile collection, interleaved timing, summary |
| `quick.cjs`, `throw-then-load.cjs` | Load-mode and call-before-load checks |
| `browser/split-entry.js`, `browser/test.html`, `browser/run.cjs` | Browser entry replacing the bundler entry, and its driver |
| `browser/synclimit.cjs`, `browser/loadtime.cjs` | Chromium main-thread sync-compile limit; compile/instantiate timing |
| `bench-summary.md` | All 50 cases × 3 variants from the main timing run |

Nothing in the shipped build pipeline, `xtask` or `crates/wasm/pkg` changed.
