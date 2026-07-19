# Audit coverage ledger

This is a grouped disposition of the tracked project. Generated build output,
downloaded dependencies, and binary fixtures are excluded from source review;
their consuming parser or test suite is listed instead.

| Area | Tracked paths covered | Disposition |
| --- | --- | --- |
| Governance and manifests | root docs, `Cargo.toml`, `rust-toolchain.toml`, `deny.toml`, `.cargo/`, `package*.json`, release configuration | Reviewed. Lockfile policy and npm lock drift fixed; fork policy documented. |
| Workspace architecture | `CLAUDE.md`, crate manifests, `scripts/check-boundaries.sh` | Reviewed. Layer rules retained; boundary command remains required in final validation. |
| Geometry and math | `crates/math`, `crates/geometry`, `crates/algo` | Manual review complete for high-risk evaluation/fitting paths. Degree-buffer and NURBS invariant findings remain open. |
| Topology and operations | `crates/topology`, `crates/operations`, `crates/offset`, `crates/blend`, `crates/heal`, `crates/check`, `crates/sketch` | Manual review complete for public boolean/classification/modifier paths. Cavity and postcondition findings remain open. |
| Import/export | `crates/io`, parser tests and fixtures | Reviewed for malformed input. STL index and IGES UTF-8 panics fixed; size/depth/resource budgeting remains open. |
| WASM and JavaScript | `crates/wasm`, TypeScript bindings, `xtask`, smoke script | Reviewed. no-I/O build, handle narrowing, CLI drift, and normal smoke coverage fixed; stale checkpoint handles and public input budgets remain open. |
| CI and supply chain | `.github/workflows`, Dependabot, action pins | Reviewed. Checked-in lockfiles replace unproven Cargo scan setup. Workflow permissions, SBOM/attestation, and actual fork runs require follow-up. |
| Tests, examples, fixtures, benchmarks, corpus | all tracked test/example/fixture/benchmark directories | Inventory reviewed and targeted regressions added for confirmed crash/wrap defects. The requested full corpus, fuzz, performance, and downstream matrix is pending. |
| Documentation | README feature matrix and policy docs | Stability and fork ledgers added. Existing feature labels were not promoted. |

## Remaining validation matrix

| Validation | State |
| --- | --- |
| Targeted IO/WASM regressions | Passed after fixes. |
| Default and no-I/O wasm32 checks | Passed after fix. |
| `npm ci` | Passed after lock repair. |
| Full nextest, all-features tests, docs, xtask tests | Pending final run. |
| Boundary, deny, audit, machete, Taplo | Pending final run; tools may require installation. |
| MSRV 1.88 | Pending toolchain installation and run. |
| WASM package build, Node smoke, npm dry-run | Pending pinned wasm-pack/CLI availability. |
| Coverage, benchmark, fuzz/corpus replay | Pending; no coverage percentage is claimed. |
| Fork-hosted CI evidence | Pending remote execution; no GitHub settings or release action was changed. |
