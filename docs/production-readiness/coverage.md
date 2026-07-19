# Audit coverage ledger

This is a grouped disposition of the tracked project. Generated build output,
downloaded dependencies, and binary fixtures are excluded from source review;
their consuming parser or test suite is listed instead.

The audit inventory contained 725 tracked files at the baseline commit: 610
under `crates/`, 11 explicit test-path files, 10 under `docs/` or `book/`, five
CI workflow files, and 89 root/tooling/configuration files. Generated `target/`,
`node_modules/`, and `crates/wasm/pkg*` output is ignored and was validated as
an artifact rather than reviewed as source. No database, migration, server,
container, deployment manifest, background service, or user-interface source
exists in this library workspace, so those review phases are not applicable.

| Area | Tracked paths covered | Disposition |
| --- | --- | --- |
| Governance and manifests | root docs, `Cargo.toml`, `rust-toolchain.toml`, `deny.toml`, `.cargo/`, `package*.json`, release configuration | Reviewed. Lockfile policy and npm lock drift fixed; fork policy documented. |
| Workspace architecture | `CLAUDE.md`, crate manifests, `scripts/check-boundaries.sh` | Reviewed. Layer rules retained; boundary command remains required in final validation. |
| Geometry and math | `crates/math`, `crates/geometry`, `crates/algo` | Manual review complete for high-risk evaluation/fitting/sampling paths. High-degree scratch storage, derivative bounds, and bowed-interval curvature sampling fixed with regressions; broader imported-NURBS invariant budgets remain follow-up work. |
| Topology and operations | `crates/topology`, `crates/operations`, `crates/offset`, `crates/blend`, `crates/heal`, `crates/check`, `crates/sketch` | Manual review complete for public boolean/classification/modifier paths. Cavity classification, containment, and properties fixed; fallback-quality and partial-result postconditions remain open. |
| Import/export | `crates/io`, parser tests and fixtures | Reviewed for malformed input. STL index and IGES UTF-8 panics fixed; size/depth/resource budgeting remains open. |
| WASM and JavaScript | `crates/wasm`, TypeScript bindings, `xtask`, smoke script | Reviewed. no-I/O build, handle narrowing, CLI drift, and normal smoke coverage fixed; stale checkpoint handles and public input budgets remain open. |
| CI and supply chain | `.github/workflows`, Dependabot, action pins | Reviewed. Checked-in lockfiles replace unproven Cargo scan setup. CI and publish now share the validated xtask package path and run npm dry-runs. Workflow permissions are narrow; SBOM/attestation and actual fork runs require follow-up. |
| Tests, examples, fixtures, benchmarks, corpus | all tracked test/example/fixture/benchmark directories | Inventory reviewed. Full workspace tests, targeted regressions, and the deterministic complexity guard pass. No standalone fuzz corpus is tracked; adversarial scanning/fuzzing was outside this run mode. |
| Documentation | README feature matrix and policy docs | Stability and fork ledgers added. Existing feature labels were not promoted. |

## Remaining validation matrix

| Validation | State |
| --- | --- |
| Targeted IO/WASM regressions | Passed after fixes. |
| Default and no-I/O wasm32 checks | Passed after fix. |
| `npm ci` | Passed after lock repair. |
| Workspace all-features tests, docs, xtask tests | Passed locally with Cargo's official test runner. CI uses nextest plus a separate doc-test command. |
| Boundary script | Passed locally. Cargo-deny and RustSec were not invoked because this run explicitly excluded specialized scan workflows; their pinned CI jobs remain the execution gate. |
| Machete and Taplo | CI definitions reviewed; local tools were unavailable. Fork CI must provide final evidence. |
| MSRV 1.88 | Passed locally. |
| WASM package build, Node smoke, npm dry-run | Passed with Rust 1.96.0, wasm-pack 0.15.0, and Node 22.22.2. The tarball includes both license texts. |
| Coverage, benchmark, fuzz/corpus replay | Deterministic complexity guard passed. The 60% llvm-cov gate remains in CI; no local coverage percentage or fuzz claim is made. |
| Fork-hosted CI evidence | Pending remote execution; no GitHub settings or release action was changed. |
