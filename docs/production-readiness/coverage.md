# Audit coverage ledger

This is a grouped disposition of the tracked project. Generated build output,
downloaded dependencies, and binary fixtures are excluded from source review;
their consuming parser or test suite is listed instead.

The audit inventory contained 725 tracked files at the baseline commit. The
current hardened branch contains 727 tracked files, including checked-in
root and xtask lockfiles. The baseline included 610
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
| Topology and operations | `crates/topology`, `crates/operations`, `crates/offset`, `crates/blend`, `crates/heal`, `crates/check`, `crates/sketch` | Manual review complete for public boolean/classification/modifier paths. Cavity semantics, bounded fail-closed fallback, deterministic repeated cuts, modifier partial-result handling, offset postconditions, and tessellation error propagation are fixed with regressions. |
| Import/export | `crates/io`, parser tests and fixtures | Reviewed for malformed and oversized input. STL index and IGES UTF-8 panics are fixed; every importer now applies shared byte/entity limits and 3MF applies a separate uncompressed-entry limit. |
| WASM and JavaScript | `crates/wasm`, TypeScript bindings, `xtask`, smoke script | Reviewed. no-I/O build, checked handle narrowing, checkpoint high-water retirement, batch byte/operation limits, CLI drift, and normal smoke coverage are fixed. |
| CI and supply chain | `.github/workflows`, Dependabot, action pins | Reviewed. Checked-in lockfiles replace unproven Cargo scan setup. CI and publish share the validated xtask package path and run npm dry-runs. Workflow permissions are narrow; the final fork run passed. SBOM/attestation and any actual publish remain follow-up work. |
| Tests, examples, fixtures, benchmarks, corpus | all tracked test/example/fixture/benchmark directories | Inventory reviewed. Full workspace tests, targeted regressions, deterministic complexity guard, and active 64-cut determinism gate pass. Three slow/diagnostic operations tests remain explicitly ignored and are not release blockers. No standalone fuzz corpus is tracked; adversarial scanning/fuzzing was outside this run mode. |
| Documentation | README feature matrix and policy docs | Stability and fork ledgers added. Existing feature labels were not promoted. |

## Remaining validation matrix

| Validation | State |
| --- | --- |
| Targeted IO/WASM regressions | Passed after fixes. |
| Default and no-I/O wasm32 checks | Passed after fix. |
| `npm ci` | Passed after lock repair. |
| Workspace all-features tests, docs, xtask tests | Passed locally with Cargo's official test runner. CI uses nextest plus a separate doc-test command. |
| Boundary script | Passed locally and in fork CI. Cargo-deny, RustSec, and OSV also passed on the final PR head. |
| Machete and Taplo | Passed in fork CI through the unused-dependencies and TOML-format jobs. |
| MSRV 1.88 | Passed locally. |
| WASM package build, Node smoke, npm dry-run | Passed with Rust 1.96.0, wasm-pack 0.15.0, and Node 22.22.2. The tarball includes both license texts. |
| Coverage, benchmark, fuzz/corpus replay | Deterministic complexity, 64-cut release guards, benchmark, and the 60% llvm-cov gate passed. No fuzz/corpus claim is made. |
| Fork-hosted CI evidence | Passed in run `29672853018`: aggregate CI, tests, coverage, MSRV, WASM builds, docs, boundaries, dependency policy, RustSec, OSV, formatting, unused dependencies, and benchmarks were green. No release or repository setting was changed. |
