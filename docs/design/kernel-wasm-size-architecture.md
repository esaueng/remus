# Browser kernel size architecture

Status: **in progress**; owner decisions and measured outcomes are recorded at
the end and supersede the estimates in the workstream sections. Baseline:
`259683c5` (committed packages v2026.1.55, built from `ed2754cd`).
Roadmap rows: [PERF-W06](../kernel-maturity/roadmap.md#perf-w06) (separate
load and code-size budgets) and [PERF-P03](../kernel-maturity/roadmap.md#perf-p03)
(optimization-level trade-offs). The consumer policy this mirrors is OpenZCAD
`docs/kernel-wasm-size-policy.md` and `scripts/bundle-size-policy.mjs`.

## Problem

The browser kernel (`crates/wasm/pkg/remus_wasm_bg.wasm`) is at its limits:

| Metric (OpenZCAD measurement) | v2026.1.55 | Limit | Headroom |
| --- | ---: | ---: | ---: |
| Raw | 10,479,386 B | 10 MiB hard (both repositories) | 6,374 B |
| Node `zlib.gzipSync()` defaults | 3,673,035 B | 3.5 MiB hard (OpenZCAD only) | **−3,019 B** |
| Brotli q11 | 2,593,182 B | 2.5 MiB review (OpenZCAD only) | 28,258 B |

- The committed package already exceeds the consumer's gzip hard limit, so the
  next OpenZCAD pin update past it fails the bundle check. `cargo xtask
  wasm-build` cannot see this: [`xtask/src/wasm.rs`](../../xtask/src/wasm.rs)
  mirrors only the raw limits.
- Since the translator split (7,602,120 B on 2026-09-04) the kernel grew
  2.88 MB in 38 days, about **0.5 MB per week**. The growth is spread over
  qualified features of 100–300 KB each (for example the B74 affine transforms
  +271 KB, blend reconstruction and entity history +251 KB, curve projection
  +222 KB, the O2.3b UV arrangement +127 KB). H2–H6 hold many more rows of
  that kind.
- The only per-PR control is the advisory `WASM Binary Size` comment
  (`fleet-wasm-size.yml`), raw bytes only, warning above 50 KB.

Raising the cap alone does not fix this: at the historical rate a 12 MiB cap
lasts about two months and 16 MiB about four.

## Evidence

All builds below use the shipped pipeline: `--no-default-features`,
`-C target-feature=+simd128`, wasm-bindgen 0.2.126, `wasm-opt -Oz
--enable-simd` (wasm-pack's cached Binaryen). The reproduced baseline matches
the committed binary within 3 KB.

### Optimization level

Today only the `remus-wasm` crate builds at `opt-level = "z"`; every kernel
crate it links builds at the release default `3`.

| Whole-graph opt-level | Raw | gzip | Brotli q11 |
| --- | ---: | ---: | ---: |
| current (deps `3`) | 10,482,336 | 3,672,922 | 2,592,524 |
| `s` | 8,665,297 (−17.3%) | 3,225,613 | 2,309,808 |
| `z` | 7,500,742 (−28.4%) | 2,775,853 | 2,021,827 |

Node micro-benchmark, median of 7 after 2 warm-ups, two runs, Apple silicon
(an ad-hoc harness, not committed and not a qualified benchmark; volumes identical across
variants):

| Variant | 16 cuts | 8 fuses | 12-edge fillet | tessellate |
| --- | ---: | ---: | ---: | ---: |
| current | 1.7 ms | 16.8–17.4 ms | 15.7–16.1 ms | 10.4–10.5 ms |
| `s` | 1.8–1.9 ms | 18.1–18.6 ms | 9.9–10.5 ms | 6.3–6.7 ms |
| `z` | 2.5–2.7 ms | 27.8–28.4 ms | 21.4–21.7 ms | 13.0–13.6 ms |

**Superseded (W0, PR #1028):** this micro-benchmark was dominated by
first-run compile time, which a smaller module wins. Measured on the committed
`scripts/performance` suites with instruction counts and full V8 tiering,
whole-graph `s` is **18–70% slower in steady state** (sketch `drag_1000`
+30 to +34%, `coupled_chain_1000` +25 to +28%, body edit +38%, booleans
+5 to +15%); only one-shot processes get faster. A per-crate search found no
smaller configuration within noise on every workload: the only no-regression
variant saves 52 KB raw and adds 1 KB gzip. `wasm-opt -Oz --converge` saves
6.6 KB. Under owner decision 1 the optimization level stays as it is.

### Attribution

Function bytes in the `s` build before `wasm-opt` (names kept), grouped by
the first `remus_*` crate in the symbol, so generic std code instantiated for
a Remus type counts toward that crate:

| Group | Share |
| --- | ---: |
| std/core/alloc/hashbrown generics with no Remus type in the name | 32.4% |
| `remus_operations` | 19.0% |
| Binding layer and glue (`RemusKernel`, wasm-bindgen, serde, tsify) | 14.9% |
| `remus_algo` | 11.4% |
| math, topology, check, blend, heal, offset, sketch, io, geometry | 19.3% |
| Other (`robust`, std formatting/panic, `remus_wasm`, misc.) | 3.0% |

The groups total 100.0% of function bytes; the name section and data segments
are excluded.

- Generic monomorphization (twiggy `monos`) totals about 4.0 MB, roughly 30%
  of code. Before `wasm-opt` the largest family looks like
  `hashbrown::RawTable::reserve_rehash` (282 instances, 258 KB), then
  `Vec::from_iter` (230 KB); after `wasm-opt` merges identical copies it is
  stable sort, about 1.07 MB in 267 per-comparator copies (see W3.2 below).
- serde, serde_json and tsify account for about 0.55 MB; `batch.rs` dispatch
  for 0.15 MB.
- Most ops exist twice: a direct `#[wasm_bindgen]` method (392 `js_name`
  exports) and a JSON `executeBatch` arm (154 arms in `dispatch_op`).

### Comparable browser kernels

WASM sizes from the published npm packages (2026-10-13):

| Package | WASM | Scope |
| --- | ---: | --- |
| `opencascade.js` 2.0.0 beta, full | 50.3 MB | All of OCCT |
| `replicad-opencascadejs` 1.1.0 | 22.5–23.0 MB | Modeling + STEP, one module |
| `occt-import-js` 0.0.23 | 7.6 MB | STEP/IGES import |
| `manifold-3d` 3.5.4 | ≈2.8 MB (whole package) | Mesh booleans |

A professional feature set should be planned at **about 25–30 MiB of WASM
across all modules** (estimate). The design question is how much of that the
browser must download and compile before the first interaction.

## Target architecture

### Module tiers

| Tier | Belongs here | Long-term budget |
| --- | --- | --- |
| **Core** (eager) | Anything that edits the live session: handles, journal and evolution, persistent naming, checkpoints, transactions. Primitives, booleans, fillet/chamfer, extrude/revolve/sweep/loft, direct edit, tessellation, measurement, classification. | 16 MiB raw / 6 MiB gzip |
| **Lazy** (loaded on first use) | Work that is naturally document-in/document-out or session-independent: file translators (shipped as `remus-wasm-io`), import repair, the 2D sketch solvers, read-only analysis, drawing projection, PMI/AP242, future native-format translators. | ≤ 5 MiB raw per module |
| **Native-only** | Server batch work, `remus-render`, corpus and qualification tooling. | none |

Bodies cross between tiers as exact arena documents
(`serializeSolids`/`deserializeSolids`), as they already do for the
translators. A lazy module must not need handle identity, the journal or
checkpoints of the core session; an operation that does stays in core.

### Load-time gates

Bytes are a proxy. The gates users feel, measured with OpenZCAD
`scripts/measure-remus-wasm-load.mjs` (Playwright, 20 runs, p50/p95):

| Gate | Proposed target |
| --- | --- |
| Desktop, cold cache, broadband | kernel ready < 2 s p95 |
| Mobile-class (4× CPU throttle, Fast 4G) | kernel ready < 5 s p95 |
| Warm (compiled-module cache) | kernel ready < 0.5 s p95 |

The numbers are proposals: measure the current kernel first and set them from
that baseline. Every raise of a byte ceiling must keep these green.

## Workstreams

Each item lists the repository, the deliverable and its acceptance bar.
Changes to `fleet-*.yml` follow the fleet pin rule: callee change on main
first, then the re-pin.

### W0 — Unblock (Remus)

1. **Whole-graph `opt-level = "s"` for WASM only.** `xtask` sets
   `CARGO_PROFILE_RELEASE_OPT_LEVEL=s` on both wasm-pack invocations, leaving
   native `release` at `3` and keeping wasm-pack's `--release` metadata
   (`wasm-opt -Oz --enable-simd`). Every CI job that builds the shipped
   module (wasm-size, package verification) goes through the same path.
   Acceptance: package ≤ 8.7 MB raw / 3.25 MB gzip; `scripts/performance`
   workloads all within noise; WASM contract
   tests and `scripts/test-wasm-smoke.mjs` green. Under owner decision 1 a
   regression is not recorded and accepted: hot crates keep `opt-level = 3`
   until every workload is within noise.
2. **Gate parity.** `cargo xtask wasm-build` measures gzip and Brotli by
   calling Node's `zlib` with the consumer's exact settings (Node is already
   required by the version scripts), fails above the gzip hard limit and
   warns above the Brotli review limit, next to the existing raw window.
   Acceptance: unit tests on each boundary, as for the raw window today.

### W1 — Per-PR growth control (Remus)

1. `fleet-wasm-size.yml` reports raw, gzip and Brotli deltas for the kernel
   and fails the job when the PR build exceeds any hard limit.
2. A kernel growth above 64 KiB raw requires a `WASM-Size:` line in the PR
   description naming the capability and its tier; the job fails without it.
3. Each package refresh records sizes in its PR body, as the OpenZCAD updater
   already does, and carries the `WASM-Size:` lines of the source PRs merged
   since the previous refresh, each marked with its PR number
   (`scripts/publish-wasm-package-pr.py`). Growth no source PR declared still
   fails the refresh's gate.

### W2 — Ceiling step 1 (Remus + OpenZCAD, paired)

Raise to **12 MiB raw hard / 11 MiB raw review / 4.5 MiB gzip hard / 3 MiB
Brotli review** (gzip ≈ 0.37 × raw for the `s` build). Two PRs that land
together: OpenZCAD `bundle-size-policy.mjs` + policy doc, Remus
`xtask/src/wasm.rs`. Never inside a pin-update PR. Evidence for the
consumer's seven-point checklist:

| Checklist item | Evidence |
| --- | --- |
| 1. Pinned toolchains | `rust-toolchain.toml` (1.96.0), wasm-bindgen 0.2.126; pin wasm-pack and Binaryen versions in xtask. |
| 2. Optimized vs `--skip-opt` | Both measured for the W0 package. |
| 3. Attribution | This document's tables, refreshed on the W0 build. |
| 4. Load measurements | `measure-remus-wasm-load.mjs`, desktop and throttled. |
| 5. Gating/splitting considered | W3 and W4 below. |
| 6. Why in core | The tier rule (W5). |
| 7. Raw/gzip/Brotli reviewed | The W2 PR body. |

### W3 — Shrink core (Remus)

Estimated savings are pre-measurement targets.

1. **One dispatch path.** Direct methods and `executeBatch` arms share one
   typed argument decoder and one implementation; JSON only at the batch
   boundary. Target −0.3 to −0.6 MB raw. Contract tests keep both surfaces
   bit-identical (`execute_batch` suites already cover them).
2. **Generic bloat.** Collapse rarely hot `HashMap`/`HashSet` instantiations
   onto a few shared key types or sorted `Vec`s, route cold collection
   building through non-generic helpers, and check the top `twiggy monos`
   families on every refresh. Target −0.5 to −1.0 MB raw. Hot paths keep
   their specialized code; each change carries a perf check.
3. **serde/tsify.** Replace serde-derived JS conversion on large result types
   with typed arrays or hand-written `JsValue` construction where the
   generated code dominates. Target −0.2 MB.
4. **Error and formatting text.** Keep typed error codes; move long messages
   out of hot generic code. Measure `.rodata` and `fmt` share before and
   after.

### W4 — Lazy modules (Remus + OpenZCAD)

In order of decoupling, verified against the bindings:

1. **`remus-wasm-sketch`**: `remus-sketch` GCS (`gcs_sketch.rs`, 2,053 lines,
   no `self.topo` access) and `polygon2d.rs` (no kernel state). The coupling to
   resolve: GCS sketch state is part of kernel checkpoints (`checkpoint.rs`,
   `state.rs`) and `executeBatch`. Either the sketch module owns its own
   checkpoints and the consumer coordinates both, or sketch state stays in
   core and only the solver moves. Decide in an RFC before code.
2. **`remus-wasm-repair`**: import-time healing and read-only analysis
   (feature recognition, curvature), document in / document out. Heal used
   inside operations stays in core.
3. **Drawing and PMI tiers** when those roadmap rows start (7.5 silhouettes,
   O5.3 PMI read).

Each module follows `remus-wasm-io`: its own crate under `crates/`, its own
`PackageSpec` and size budget in xtask, one shared package version, a
consumer ADR in OpenZCAD for when it loads.

### W5 — Tier rule for new work (Remus)

Every new roadmap capability declares `Tier: core | lazy:<module> |
native` in its row or design doc before implementation. The
`add-operation` and `wasm-bindings` skills and the PR template carry the
rule; W1's `WASM-Size:` line enforces it at review. Core is the exception
that needs the reason ("edits the live session").

### W6 — Load-time gates (OpenZCAD)

Run `measure-remus-wasm-load.mjs` on main for the current kernel, record the
baseline, adopt the W6 targets (adjusted to that baseline) as review gates,
then as hard gates once the measurement is stable in CI.

### W7 — Ceiling step 2 (when H4/H5 lands)

Raise core to **16 MiB raw / 6 MiB gzip** only with W1–W6 in place, the load
gates green and the same seven-point evidence.

## Sequence

| Step | Work | Kernel after (raw / gzip) | Runway at 0.5 MB/week |
| --- | --- | --- | --- |
| 1 | W0 | ≈ 8.7 / 3.2 MB | 3–4 weeks under 10 MiB / 3.5 MiB |
| 2 | W1, W6 baseline | unchanged | growth visible per PR |
| 3 | W2 | ceiling 12 / 4.5 MiB | ≈ 8 weeks |
| 4 | W3, W4.1, W5 | target −1 to −2 MB; core growth ≈ 0.2 MB/week | ≈ 5 months |
| 5 | W4.2–3 | per roadmap | — |
| 6 | W7 | ceiling 16 / 6 MiB | H5/H6 scope |

## Risks

- `s` slows a hot path not covered by the micro-benchmark. Mitigation: the
  full performance suite before W0 lands; per-crate `opt-level = 3` overrides
  for proven hot crates are allowed (cost measured).
- Lazy modules duplicate shared code (math, topology, std) in each module; a
  split pays off only when the moved code is clearly larger than the shared
  base it re-links. Measure the module's own size before committing to it.
- Arena-document round trips add latency and lose handle identity; anything
  inside the edit history stays in core by rule.
- Compression ratios differ between Rust's and Node's deflate; gate parity
  must call Node's zlib, not a Rust reimplementation.

## Owner decisions (2026-10-13)

1. **No speed regression.** No workstream may make any workload in the
   `scripts/performance` suites (main, `sketch`, `transactions`) or the
   criterion benches measurably slower. A size change that costs speed ships
   only with the cost removed (for example a per-crate `opt-level = 3`
   override on the affected hot crate). Every code PR attaches before/after
   numbers; "within noise" means the medians' difference is inside the
   run-to-run spread of repeated baseline runs.
2. **W2 ceiling:** 12 MiB raw hard / 11 MiB raw review / 4.5 MiB gzip hard /
   3 MiB Brotli review, as proposed.
3. **W6 targets:** measure the current kernel first, then set the gates from
   that baseline; the proposed 2 s / 5 s / 0.5 s are the starting point and
   no gate may be looser than today's measurement.
4. **Sketch state stays in core; only the solver moves.** GCS sketch state
   remains in kernel checkpoints, so undo and restore stay atomic with the
   body history. The lazy `remus-wasm-sketch` module receives the system,
   solves and returns parameters. Under decision 1 the round trip must not
   slow the `scripts/performance/sketch` workloads, the consumer preloads the
   module at idle so first use is not slower, and the split ships only if it
   removes at least 250 KiB from the core after the duplicated shared code is
   counted.
5. **W7** stays deferred until H4/H5 capabilities land.

## Outcomes (2026-10-13)

Measured results replace the estimates above. Sizes are shipped-pipeline
bytes against main's kernel (raw 10,484,484 / gzip 3,673,721).

| Workstream | Outcome |
| --- | --- |
| W0.1 opt-level | **Rejected.** Whole-graph `s` is 18–70% slower in steady state; no per-crate split is both smaller and within noise (see Optimization level). |
| W0.2 gate parity | PR #1028: `scripts/wasm-size-policy.mjs` is the single source of the kernel policy (raw, Node gzip, Brotli q11); xtask enforces it. |
| W1 growth control | PR #1030: CI fails a PR over any hard limit, or growing the kernel more than 64 KiB raw without a `WASM-Size:` line naming a tier; wired into `CI Pass`. It edits fleet workflows, so it needs an owner CI bypass and a merge-commit landing (fleet pins must be main ancestors). |
| W2 ceiling step 1 | PR #1028 + OpenZCAD #658: 11/12 MiB raw, 4.5 MiB gzip, 3 MiB Brotli, with the seven-point evidence recorded in OpenZCAD `docs/kernel-wasm-size-policy.md`. |
| W3.1 one dispatch path | **Not worth it.** Deleting every `executeBatch` arm saves only 157 KB raw / 29 KB gzip; a shared implementation projects to about 34 KB, and many direct/batch pairs differ in validation on purpose. The earlier −0.3 to −0.6 MB estimate counted kernel code inlined into bindings. |
| W3.2 generic bloat | PR #1031: the largest family was stable sort (≈ 1.07 MB in 267 per-comparator copies), not hash tables (≈ 145 KB after `wasm-opt` merging). A shared `CompactSort` on ≈ 205 cold sites, 31 measured hot sites unchanged: **−754 KB raw / −106 KB gzip**, outputs identical, instruction counts within spread on all 113 native cases and WASM suites. |
| W3.3 serde | PR #1029: hand-written arena and persistent-reference codec, byte-identical output and error kinds against a verbatim oracle: **−267 KB raw / −78 KB gzip** in the kernel (−208 KB in the translator), 5–33% fewer instructions in serialize/deserialize. |
| W4 lazy modules | **No candidate passes the combined size-and-speed bar.** The sketch solver split clears 250 KiB at today's optimization level (254 KiB; 151 KiB only under a size profile, which W0.1 rejects), but its 1.5–2 µs module round trip makes 10-parameter solves 5–10% slower. Repair + analysis + NURBS reuse + projection together: 175 KiB. A prototype is kept unpublished for a later re-check if the solver grows by about 100 KiB. |
| W5 tier rule | PR #1027. |
| W6 load gates | OpenZCAD #657: desktop cold ≈ 47–77 ms p95, mobile-class cold ≈ 3.5 s p95 (download-dominated, ≈ +89 ms per 100 KB gzip); review budgets 110/110 ms desktop and 3,800/150 ms mobile, cold/warm. |
| W7 | Deferred to H4/H5 (owner decision 5). |

With #1029 and #1031 the kernel is estimated at about 9.46 MB raw / 3.49 MB gzip (the
two PRs were measured separately and change disjoint code; the combined build is not yet measured). From
there the step-1 ceiling leaves about 3.1 MB raw (≈ 6 weeks at 0.5 MB/week)
and 1.2 MB gzip (≈ 7 weeks), and mobile-class cold load drops by about
0.16 s. Keeping new cold code on `CompactSort` and new capabilities on the
tier rule is what keeps that runway.

## Growth attribution (2026-10-29)

The kernel grew from 9,412,128 B (v2026.1.57, built just after #1031) to
11,005,504 B (v2026.1.86) in 16 days, about **0.7 MB raw per week**. That is
faster than the 0.5 MB/week the runway above assumes. At that rate the
step-1 review limits are reached in days and the hard limits in about two
weeks. Both sources were rebuilt with the shipped pipeline and attributed
with twiggy. Evidence, commands and scripts:
[`docs/performance/evidence/kernel-size-attribution-2026-10-29/`](../performance/evidence/kernel-size-attribution-2026-10-29/README.md).
The rebuilds land within 0.3% of the shipped packages, so absolute bytes are
approximate.

| Shipped-equivalent | Base `89a3eac5` | Cur `d8195032` | Growth |
| --- | ---: | ---: | ---: |
| Raw | 9,386,948 | 10,974,155 | +1,587,207 |
| gzip (Node defaults) | 3,481,345 | 4,079,297 | +597,952 |
| Brotli q11 | 2,484,589 | 2,904,757 | +420,168 |

**Most of the growth is new code, not generic bloat.** 1,370 new functions
add 1.26 MB and 724 removed functions take away 0.21 MB. Existing functions
grew by 1.03 MB in total and shrank by 0.43 MB.

| Group (after `wasm-opt`) | Growth | Largest modules |
| --- | ---: | --- |
| `remus_operations` | +569 KB (34%) | tessellate +96K, blend_ops +92K (fillet cascade), resize_blend +84K, fillet +50K, measure +47K, thicken +41K, project_curve +38K, section +37K; push_pull −48K, defeature −44K |
| `remus_algo` | +221 KB | pave_filler +160K (phase-FF seeding, circular-cap and curved-section clipping) |
| `remus_check` | +218 KB | properties +138K (face integrator), classify +70K |
| `remus_topology` | +140 KB | validation +60K (`check_same_parameter_strict_with_tolerance`, 61 KB), transaction +38K (`run_transacted`, 44 KB) |
| std/core/alloc/hashbrown generics | +114 KB | sorts, hash maps |
| `remus_math` | +78 KB | nurbs +70K |
| `remus_wasm` | +74 KB | bindings +68K (batch `dispatch_op` +15K) |
| data segments | +36 KB | |

**Recoverable duplication, about 100–150 KB raw (estimate):**

1. `integrate_parametric` (`crates/check/src/properties/face_integrator.rs`)
   is monomorphized 11 times, 81.7 KB in total, of which twiggy counts about
   68.7 KB as duplication.
2. **The sort lint has a gap.** The kernel crates' `clippy.toml` disallows
   `sort_by` and `sort_by_key` but not `sort_unstable*`. Since `89a3eac5`,
   non-test kernel code gained 22 `sort_unstable`, 12 `sort_unstable_by_key`
   and 8 `sort_by` call sites. The `core::slice::sort` families grew by
   +52.6 KB and 35 copies (579 KB in 396 copies → 632 KB in 431). Closing the
   gap needs an unstable `CompactSort`: substituting the stable one can
   reorder ties and change geometry output.
3. **Opt-in memo modules** grew from 59 KB to 107 KB. Performance caches
   should be declared on the `WASM-Size:` line like any other capability.
4. **hashbrown instantiations** grew by 18 KB across 44 new copies.

Trimming these restores weeks, not months, of runway. The trend comes from
qualified features landing in core, so the remaining levers are the ceiling,
an explicit growth budget, and structural splitting.
