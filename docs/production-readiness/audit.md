# Production-readiness audit

## Scope and evidence

This ledger records the manual production-readiness audit started from
`d2893af8807b2e7c6c52e90cba2a3ad9cce3bfa7` on 2026-07-18. At the start of
the work, `origin/main` and `upstream/main` resolved to that same commit.
The Codex Security workflow was explicitly not run; security-related entries
below are from source inspection and targeted reproductions only.

| ID | Severity | Component | Evidence and impact | Reproduction | Status |
| --- | --- | --- | --- | --- | --- |
| IO-001 | P0 | STL mesh import | Triangle indices were dereferenced before bounds validation, allowing malformed OBJ, PLY, GLB, or direct mesh input to panic after partial topology allocation. | Construct three positions with indices `[0, 1, 3]`; call `stl::import::import_mesh`. | Fixed: validate finite tolerance, positions, normals, triangle alignment, and every index before topology mutation. Regression tests verify error and empty topology. |
| IO-002 | P0 | IGES fixed-column reader | UTF-8 byte offsets were used as Rust string offsets, so non-ASCII input could panic. | Put `é` in an 80-byte IGES record and call `read_iges`. | Fixed: reject non-ASCII fixed-width records before parsing. Regression test added. |
| WASM-001 | P1 | no-I/O WASM feature set | `--no-default-features` referenced optional `brepkit_io` from BREP STEP methods and did not compile. | `cargo check -p brepkit-wasm --target wasm32-unknown-unknown --no-default-features`. | Fixed: JSON BREP remains available; STEP import/export now return an explicit JavaScript error without the `io` feature. CI covers the variant. |
| WASM-002 | P1 | batch JSON handles | JSON integers above `u32::MAX` silently wrapped to unrelated handles. | Pass `4294967296` for a handle field to `executeBatch`. | Fixed: checked `u32::try_from`; regression test added. |
| PKG-001 | P1 | npm reproducibility | `npm ci` failed because `conventional-commits-parser@6.4.0` was absent from the lockfile. | `npm ci --cache /private/tmp/brepkit-npm-cache` before the repair. | Fixed: regenerated only the missing lock entry; clean install now passes. |
| REL-001 | P2 | WASM toolchain | Cargo pinned `wasm-bindgen 0.2.126` while xtask required CLI `0.2.121`. | Compare `Cargo.toml` and `xtask/src/wasm.rs`. | Fixed: xtask now requires `0.2.126`. A remaining improvement is deriving this value from Cargo metadata. |
| REL-002 | P2 | WASM release validation | Normal `wasm-build` validated files but did not run the Node runtime smoke test. | Inspect `xtask/src/main.rs`. | Fixed: normal builds now run `scripts/test-wasm-smoke.mjs`; CI invokes the command. |
| REL-003 | P1 | npm artifact licensing | The generated npm tarball omitted both repository license texts even though its manifest declared `MIT OR Apache-2.0`. | Run `npm pack --dry-run` in `crates/wasm/pkg`; the initial six-file listing had no license. | Fixed: xtask copies both root licenses into the generated package, declares them in `files`, and validates their presence before smoke or publish. CI and the publish workflow now run an npm dry-run. |
| GOV-001 | P2 | dependency scanning | `Cargo.lock` was ignored while OSV referenced it, producing unproven Cargo coverage. | Inspect `.gitignore` and `.github/workflows/osv-scan.yml`. | Fixed locally: root and xtask lockfiles are now source-controlled; RustSec consumes the checked-in lockfile. Fork CI execution remains to be verified remotely. |
| IO-003 | P0 | hostile-input limits | STEP, IGES, 3MF/ZIP, glTF, PLY, and batch JSON have no complete configurable byte/entity/depth/work budgets. Large inputs can exhaust memory or CPU. | Static audit of parsers and public WASM methods. | Open. Requires shared input-limit API and corpus/fuzz coverage; no production claim is permitted until bounded. |
| WASM-003 | P1 | checkpoint handles | Bare arena indices can alias an entity created after checkpoint restore. | Restore a checkpoint, create entities, then reuse a post-checkpoint handle. | Open. Needs generation-tagged or externally registered handles; compatibility design required. |
| BOOL-001 | P1 | boolean/cavity semantics | Classifiers and some containment fast paths inspected only outer shells, so cavities were classified as material and fuse could discard a body located in a cavity. | Cut a centered box from a larger box, classify the cavity center, then fuse a smaller body into the void. | Fixed: native classifiers traverse all shells, analytic single-region shortcuts defer for cavity solids, and area/volume/center calculations include signed inner-shell contributions. Regressions cover classification, properties, and fuse containment. |
| BOOL-002 | P1 | boolean fallback contract | Mesh fallback may return open or non-manifold topology while public APIs report a solid without quality metadata. | Exercise known difficult boolean fixtures. | Open. Add strict production mode or typed quality result before stable claims. |
| NUM-001 | P1 | NURBS degree handling | Degree nine and above reached fixed eight-element evaluation buffers and panicked; maximum-order derivatives also exposed an incorrect Algorithm A2.3 bound. | Create and evaluate degree-nine Bezier curve and surface, including cached surface evaluation and degree-nine derivatives. | Fixed: evaluators retain stack storage for common degrees and allocate exact scratch storage for larger degrees; derivative bounds and degree-zero power-basis handling are corrected. Curve and surface regressions pass. |
| GEO-001 | P1 | curvature-adaptive sampling | The acceptance estimate used only endpoint curvature and the endpoint chord, so a strongly bowed interval could silently collapse to two endpoints. The corrected NURBS derivative implementation made the existing regression reproduce consistently. | Run `sampling::curvature::tests::high_curvature_produces_more_points_than_low`; the tight and flat curves both initially returned two points. | Fixed: sample midpoint curvature, use the maximum sampled curvature and a two-chord length estimate, and retain the depth-20 hard bound. Regressions cover bowed-curve refinement and explicit limit exhaustion. |
| TEST-001 | P2 | complexity instrumentation | Feature-gated work counters were process-global, so an all-features test run allowed unrelated parallel booleans to pollute the deterministic scaling guard. The isolated guard passed while the full suite reported 114 ray-geometry builds. | Run `cargo test --workspace --all-features`; compare with the isolated `scaling_` command. | Fixed: counters are thread-local to the synchronous boolean caller and use saturating increments. A regression verifies test-thread isolation; the isolated counter ratios remain unchanged. |
| OPS-001 | P1 | modifier/tessellation validation | Offset, fillet, chamfer, and tessellation paths can drop cavities or skip failed faces while returning success. | Real-model and cavity fixtures from operations tests. | Open. Require postcondition validation and explicit partial-result errors. |

## Baseline and changed-state commands

| Command | Baseline result |
| --- | --- |
| `cargo metadata --format-version 1` | Passed; generated the previously ignored resolved graph. |
| `cargo fmt --all -- --check` | Passed. |
| `cargo check --workspace --all-targets --all-features` | Passed. |
| `cargo clippy --workspace --all-targets --all-features -- -D warnings` | Passed. |
| `cargo check -p brepkit-wasm --target wasm32-unknown-unknown` | Passed. |
| `cargo check -p brepkit-wasm --target wasm32-unknown-unknown --no-default-features` | Failed with unresolved `brepkit_io`; now passes after WASM-001. |
| `npm ci` | Failed due to lock drift; now passes with an isolated npm cache (`HUSKY=0` is required in a checkout where `.git/config` is read-only). |
| `cargo test --workspace --all-features` | Passed after fixes; unit, integration, property, and documentation tests completed without failures. |
| `cargo +1.88.0 check --workspace --all-features` | Passed on the declared MSRV. |
| `cargo xtask wasm-build --skip-opt` | Passed with Node 22.22.2 and isolated wasm-pack cache; 236 TypeScript methods, 4,563.3 KiB WASM, and runtime smoke flow validated. |
| `npm pack --dry-run` | Passed after license fix; package 1.7 MB compressed / 5.1 MB unpacked. |
| `cargo test -p brepkit-operations --features perf-counters scaling_ -- --nocapture` | Passed; 4x input produced 4.1x face-split probes and 4.0x local-vertex inserts. |

## Exit criteria

This repository is **not** declared production-ready. `IO-003`, `WASM-003`,
`BOOL-002`, and `OPS-001` remain open P0/P1 release blockers, and fork-hosted
CI has not yet supplied independent evidence. See the stability matrix for
feature-level disposition and the release checklist for the required gate.
