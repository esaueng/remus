---
name: wasm-bindings
description: Use when adding or changing methods on RemusKernel in crates/wasm, wiring ops into executeBatch, writing tests for bindings, building the wasm package (wasm-pack or cargo xtask wasm-build), testing a local kernel build inside brepjs or another JS consumer, or debugging wasm-only failures such as tests that will not compile with JsError, panics that abort the wasm instance, "recursive use of an object" errors, or a JS app silently loading a stale wasm build. Also use when the CI "WASM Binary Size" PR comment shows growth, a change adds a dependency to the wasm graph, or a new capability must declare its WASM module tier (core, lazy module, or native).
---

# wasm-bindings

The WASM layer ships two modules. The kernel package `remus-wasm` exports one class, `RemusKernel` (`crates/wasm/src/kernel.rs`). JS holds opaque `u32` handles (arena indices into the kernel's `Topology`, NOT batch result indices). Bindings live in `crates/wasm/src/bindings/<domain>.rs`, one `#[wasm_bindgen] impl RemusKernel` block per module (wasm-bindgen supports multiple impl blocks across files; non-exported helpers go in a separate plain `impl RemusKernel` block).

The translator package `remus-wasm-io` (`crates/wasm-io/src/lib.rs`) exports `RemusIo`: every STEP/IGES/STL/3MF/OBJ/PLY/glTF import and export, loaded by the consumer only around file I/O. It never sees kernel handles: exports take the bytes of `kernel.serializeSolids(ids)` (or `serializeSheets`), imports return bytes for `kernel.deserializeSolids` / `deserializeSheets`. The arena codec (`bindings/arena.rs`) is always in the kernel; the format bindings in `bindings/io.rs` exist only under the crate's `io` feature, which the shipped package is built without (`cargo xtask wasm-build --kernel-io` restores the single-module layout). Adding a format binding means adding it to `RemusIo` first.

## Choose the module tier first

Every new capability declares `Tier: core | lazy:<module> | native` in its roadmap row or design doc before any binding is written (CLAUDE.md "WASM module tiers").

- **`core`** (`RemusKernel` in `remus-wasm`, loaded eagerly): only for work that edits the live session — handles, journal/evolution, persistent naming, checkpoints, transactions. It is the exception; the PR names the session state that forces it.
- **`lazy:<module>`** (its own package `remus-wasm-<module>`, like `remus-wasm-io`): document-in/document-out or session-independent work. Bodies cross as exact arena documents (`serializeSolids` / `deserializeSolids`); the module never sees kernel handles, the journal or checkpoints. Only `remus-wasm-io` exists today; `remus-wasm-sketch` (the GCS solver; sketch state stays in core) and `remus-wasm-repair` (import repair, read-only analysis) are planned.
- **`native`**: no binding at all.

Consumers preload lazy modules at idle so first use is not slower than a core call. Size work never trades speed: no workload may get measurably slower.

## Quick reference

| Task | Command | Expect |
|---|---|---|
| Type-check for wasm target | `cargo build -p remus-wasm --target wasm32-unknown-unknown` | Clean build; catches target-specific errors |
| Run contract tests | `cargo test -p remus-wasm` | Native target; see "Contract tests" below |
| Build for node/vitest | `wasm-pack build crates/wasm --target nodejs --release` | `crates/wasm/pkg/` with CJS entry `remus_wasm.js` |
| Full release packages | `cargo xtask wasm-build` | Both packages, dual-target merge + wasm-opt, version-locked; entries `remus_wasm_node.cjs` and `remus_wasm_io_node.cjs`. See [reference.md](reference.md) section 1 |
| Smoke-test the packages | `node scripts/test-wasm-smoke.mjs` | `ok - ...` lines, ends `All smoke tests passed`. Only works after xtask build (needs both `*_node.cjs` entries) |

Never `wasm-pack build --target web` for node consumption: the web target's init uses `fetch()` and fails under node. The bundler ESM entry also fails under vitest.

## Procedure: add a binding

CLAUDE.md Recipe 4 covers the skeleton (js_name, `validate_positive`/`validate_finite`, `?` via the blanket `From<E> for JsError`). What it omits or gets stale:

0. Confirm the capability's tier is `core` (above). A lazy capability goes in its module's crate instead, a native one gets no binding.
1. Pick or create `crates/wasm/src/bindings/<domain>.rs`. Do not add bindings to `kernel.rs` (it holds the struct, constructor, and private helpers only, despite Recipe 1's stale wording).
2. Reads go through `self.topo` / `self.topo()`; writes MUST go through `self.topo_mut()` (it is `Rc::make_mut` copy-on-write shared with checkpoints; rg: `fn topo_mut` in `kernel.rs`).
3. Resolve incoming handles with `resolve_solid` / `resolve_face` / etc. from `crates/wasm/src/handles.rs`; convert results back with `*_id_to_u32`.
4. **Batch dispatch**: there are no `batch_*` companion functions (Recipe 4's wording is stale; the only `batch_*` names are test helpers). To add an op to `executeBatch`, add a match arm in `fn dispatch_op` in `crates/wasm/src/bindings/batch.rs`. Parse args with `get_f64` / `get_u32` from `crates/wasm/src/helpers.rs`, map errors with `.map_err(|e| e.to_string())`, return `Ok(serde_json::json!(...))`. Copy the `"makeBox"` arm as the template. Note: `tessellateSolid` is deliberately not in the dispatcher.
5. **Panic safety** for panic-prone ops (booleans, fillets): wrap the body in `std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| ...))` and build the error with `panic_message` (`crates/wasm/src/helpers.rs`, rg: `fn panic_message`). KNOW ITS LIMIT: on wasm32-unknown-unknown panics ABORT, so `catch_unwind` catches nothing there — it protects native (rlib/test) consumers only; on wasm the panic still traps and permanently locks the object's borrow flag (see Symptom-to-cause). The kernel-wide backstop is the panic hook in `crates/wasm/src/panics.rs` (installed by `RemusKernel::new`): it records the panic text for `lastPanicMessage()` and mirrors it to `console.error` before the abort. Live catch_unwind examples: `fillet` in `bindings/operations.rs`, `compound_cut` in `bindings/booleans.rs` (that one also checks and sets `self.poisoned`; subsequent guarded calls refuse with "Kernel poisoned after panic" — native-only behavior). Do NOT use the `#[wasm_binding]` attribute from `crates/wasm-macros`: the macro is fully written but unwired (not a dependency of `remus-wasm`, zero usages), and its poisoned-path message tells callers to use a `reset()` that does not exist. The manual pattern is the live one.
6. Checkpoint: `cargo build -p remus-wasm --target wasm32-unknown-unknown` compiles, then write a contract test (below), then `cargo test -p remus-wasm`.

## Contract tests: why execute_batch, not method calls

`JsError::new` panics on non-wasm targets. Any `#[wasm_bindgen]` method returning `Result<_, JsError>` or `JsValue` cannot be exercised on the error path (or at all, for `JsValue`) under native `cargo test`. Three patterns that work, all in-tree:

1. **Batch contract test** (default): `let mut k = RemusKernel::new(); let r = k.execute_batch(r#"[{"op":"makeBox","args":{"width":10.0,"height":10.0,"depth":10.0}}]"#);` then parse `r` with `serde_json` and assert on `{"ok": ...}` / `{"error": "..."}` entries. Examples: `mod batch_tests` in `kernel.rs`, tests in `bindings/booleans.rs`, `bindings/gridfinity_tests.rs`.
2. **`pub(crate)` `_impl` functions returning `WasmError`** for logic that must be unit-testable natively (rg: `fn from_brep_impl` in `kernel.rs`).
3. **Error paths through `resolve_*`** (return `WasmError`, safe natively). See the comment in `bindings/query/tests.rs`.

Batch semantics to assert against: errors do not stop later ops in the array; invalid JSON returns `[{"error": "invalid JSON: ..."}]`.

## Symptom-to-cause

| Symptom | Cause | Fix |
|---|---|---|
| Test compiles-fails or panics with a JsError construction | Calling a `JsError`/`JsValue` method under native `cargo test` | Rewrite as an `execute_batch` contract test or `_impl` fn |
| JS reports "recursive use of an object detected which would lead to unsafe aliasing" | A panic inside wasm on an EARLIER call. wasm32-unknown-unknown is `panic=abort` (verify: `rustc --print cfg --target wasm32-unknown-unknown`), so the panic traps inside the wasm-bindgen shim's `WasmRefCell` borrow scope; the `RefMut` never drops and the object's borrow flag stays locked forever. No Rust code can reset it — the only recovery is a new `RemusKernel`. Classic root: bare `std::time::Instant::now()` (panics on wasm32) | Read the root panic: call the free function `lastPanicMessage()` (`crates/wasm/src/panics.rs`) — it stays callable post-poison, and the hook also mirrors the text to `console.error` as `[remus] panic: ...`. For `Instant` roots use the cfg-gated `timer_now` pattern (rg: `fn timer_now` in `crates/operations/src/boolean/mod.rs`) |
| Wasm instance dead after one bad op | Uncaught panic crossed the FFI boundary. NOTE: on the real wasm target `catch_unwind` is inert (`panic=abort`) — the manual wraps below only function in native contract tests; the `poisoned` flag can never be set on wasm | Fix the panic at its source; use `lastPanicMessage()` to identify it. `catch_unwind` + `panic_message` (step 5 above) still guards native/test consumers |
| `cargo xtask wasm-build` bails: wasm-bindgen-cli version mismatch demanding 0.2.121 | Stale `WASM_BINDGEN_VERSION` constant in `xtask/src/wasm.rs`; workspace `Cargo.toml` pins `wasm-bindgen = "=0.2.125"` | Update the constant to match the workspace pin, or uninstall the local cli and let wasm-pack fetch its own |
| `node scripts/test-wasm-smoke.mjs` cannot find `remus_wasm_node.cjs` | Plain wasm-pack build names the entry `remus_wasm.js`; only the xtask merge produces the `.cjs` | Run `cargo xtask wasm-build` first, or point your consumer at `remus_wasm.js` directly |
| JS consumer still runs old kernel behavior after overlaying a local build | pnpm `.pnpm` store copy not overlaid, or stale Vite dep-optimizer cache | See [reference.md](reference.md) section 2: copy into BOTH node_modules locations, `rm -rf node_modules/.vite*`, verify with md5 or a new binding key |

## Testing a local kernel in a JS consumer

Cheap path first: brepjs already aliases `'remus-wasm'` in `$BREPJS/vitest.config.ts` (also `vitest.bench.config.ts`, `vitest.stress.config.ts`). Build `wasm-pack build crates/wasm --target nodejs --release`, then temporarily point that alias at `$REMUS/crates/wasm/pkg/remus_wasm.js`. No npm install, no lockfile churn. Full overlay recipe for pnpm+Vite apps: [reference.md](reference.md) section 2.

## Anti-patterns

- Do not conclude "the binding is broken" from a native test that fails constructing `JsError`; that is the test pattern, not the binding.
- Do not conclude "re-entrancy bug" from the "recursive use of an object" error; look for an earlier panic first.
- Do not trust that a JS consumer picked up your local build; verify (md5 the `.wasm`, or probe a binding that only exists in the new build).
- Do not add timing code with bare `std::time::Instant` anywhere reachable from `remus-wasm`.
- Do not follow CLAUDE.md Recipe 4's "add a `batch_*` companion fn" literally; add a `dispatch_op` match arm.
- Do not reach for the `#[wasm_binding]` macro; it is unwired.

## Binary size

Small wasm size is a headline competitive property of this kernel (several times smaller than the reference kernel's wasm). The `wasm-size` CI job posts a PR comment ("WASM Binary Size") with kernel raw, gzip and Brotli deltas; read it as a review item on every wasm-touching PR. The `WASM Size Gate` job in `CI Pass` enforces the kernel's hard limits and the growth rule below. Rules:

- A kernel growth above **64 KiB raw** requires a `WASM-Size:` line in the PR description naming the capability and its tier, e.g. `WASM-Size: +120 KiB raw, curve projection (P-Class 7.4), Tier: core (writes the journal)`. The line must start the line (not indented, not inside an HTML comment), name the capability (a size figure alone does not count) and contain `Tier: core`, `Tier: lazy:<module>` or `Tier: native`, the tier ending at a space, `(`, `,`, `;` or the line end. After adding it, re-run the failed size jobs; they read the current description.
- A new dependency in the wasm graph always needs a justification sentence in the PR body, whatever the delta.
- Growth that belongs in an existing lazy module goes there, not into the kernel.

Measurement details and the twiggy diagnosis recipe: [reference.md](reference.md) section 6.

Generic code is the largest size lever. Every `sort_by` / `sort_by_key` call compiles its own copy of the standard stable sort (2–5 KB each in the shipped module), because each comparator closure is a distinct type. In kernel code where the sort is not a measurable share of the work, use `remus_math::compact_sort::CompactSort` (`compact_sort_by`, `compact_sort_by_key`): same order, ties included, through one shared sort. Keep `sort_by` in sort-heavy loops (BVH, CDT, the GFA builder's per-edge sorts) and check a candidate with instruction counts before converting a hot one. Clippy enforces this in the kernel crates (`disallowed_methods` in their `clippy.toml`); a measured hot site keeps `sort_by` under a statement-level `#[allow(clippy::disallowed_methods)]` with a `// hot: <evidence>` line.

## Related skills

`add-operation` (the L3 operation a binding usually wraps), `testing` (workspace test conventions), `release-flow` (publishing the package), `parity-benchmarking` (running the brepjs bench harness against a local build), `layer-boundaries` (what `wasm/src` may import).
