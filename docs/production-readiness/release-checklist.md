# Release checklist

This checklist is a release gate, not authorization to publish. The production
fork must not publish until the ownership requirements in
`fork-maintenance.md` are satisfied and every P0/P1 audit row is closed.

## Source and version

- [ ] Record the exact commit, upstream base, fork-only diff, and clean status.
- [ ] Confirm the tag, `crates/wasm/Cargo.toml`, generated `package.json`,
  changelog, and release notes use the same version.
- [ ] Confirm root and xtask lockfiles are committed and unchanged by builds.
- [ ] Review all changes since the previous tag; identify public behavior or
  format changes and supply migration notes where required.

## Required local validation

Run with the versions pinned by `rust-toolchain.toml` unless a row explicitly
selects the MSRV:

```text
npm ci
cargo fmt --all -- --check
cargo check --workspace --all-targets --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo test -p brepkit-operations --features perf-counters scaling_ -- --nocapture
cargo test --release -p brepkit-operations --test perf_64cut_determinism -- --nocapture
cargo +1.88.0 check --workspace --all-features
cargo check -p brepkit-wasm --target wasm32-unknown-unknown
cargo check -p brepkit-wasm --target wasm32-unknown-unknown --no-default-features
cargo test --manifest-path xtask/Cargo.toml
RUSTDOCFLAGS=-Dwarnings cargo doc --workspace --no-deps --all-features
./scripts/check-boundaries.sh
cargo xtask wasm-build --skip-opt
(cd crates/wasm/pkg && npm pack --dry-run)
```

- [ ] Every command exits zero and its duration/result is attached to the
  release record.
- [ ] The generated tarball contains browser JS, Node CJS, TypeScript types,
  WASM, `LICENSE-MIT`, and `LICENSE-APACHE`, and no temporary inputs.
- [ ] The Node smoke test constructs a kernel, creates and measures a box,
  tessellates it, and exports STL.
- [ ] Representative size and deterministic complexity results are compared
  with the prior release; unexplained regressions block release.
- [ ] Oversized model imports and WASM batches fail with explicit limit errors;
  stale post-checkpoint handles remain invalid after new allocation.
- [ ] Boolean fallback and modifier failures return typed errors rather than
  open, non-manifold, partial, or cavity-dropping success values.

## CI and artifact gate

- [ ] All required fork CI jobs pass on the release commit, including coverage,
  MSRV, WASM/no-I/O, deny, RustSec, docs, boundaries, machete, and Taplo.
- [ ] The coverage job remains at or above its configured 60% line threshold.
- [ ] The CI and publish workflows produce the same package through
  `cargo xtask wasm-build --skip-opt`.
- [ ] Run `cargo xtask wasm-publish --dry-run` with `TAG_NAME` set to the exact
  candidate tag. Never remove `--dry-run` during validation.
- [ ] Record the tarball hash, unpacked file list, WASM size, provenance plan,
  and rollback/yank owner.

## Approval and recovery

- [ ] No P0/P1 audit row affects the declared release scope.
- [ ] Named maintainers approve package identity, vulnerability intake,
  signing/provenance, rollback, and yanking authority.
- [ ] A rollback candidate and previous known-good artifact are available.
- [ ] Publishing credentials and remote settings are changed only by an
  authorized maintainer outside this review workflow.
- [ ] After a future authorized publish, install into disposable browser and
  Node consumers, repeat the smoke flow, and retain the results with the tag.
