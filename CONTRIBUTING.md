# Contributing to Remus

Thanks for your interest in contributing. Remus is a solid modeling kernel,
so the bar is correctness first: a change that makes a failing case pass by
weakening a test, widening a tolerance, or introducing a mesh fallback will
not be accepted. Everything else in this document exists to make meeting that
bar straightforward.

## License and sign-off

All contributions are inbound under the [Apache License, Version 2.0](./LICENSE-APACHE),
the same license the project ships under. Every commit must carry a
[Developer Certificate of Origin](https://developercertificate.org/) sign-off
(`git commit -s`), confirming you have the right to submit the work under
those terms.

One hard rule, because this project is the permanent Apache-2.0 continuation
of an upstream that relicensed to AGPL at v3: **do not port, paste, or adapt
code from upstream v3 or later.** Behavior from those releases enters only
under an explicit Apache-2.0 grant from its copyright holder, or as an
independent implementation with a regression test proving the contract.
`scripts/check-apache-lineage.sh` enforces this in CI. See
[fork maintenance and release policy](docs/production-readiness/fork-maintenance.md).

## Setup

1. Fork and clone the repository.
2. Install Rust via [rustup](https://rustup.rs/) — `rust-toolchain.toml` pins
   the toolchain and the `wasm32-unknown-unknown` target automatically.
3. Install Node.js 20+ and run `npm install` to set up the Husky hooks
   (commitlint plus the pre-commit checks).
4. Optional but recommended: `cargo install taplo-cli cargo-machete` — the
   pre-commit hook uses both when present.
5. Verify with `cargo build --workspace && cargo test --workspace`.

## Development workflow

1. Branch from `main`.
2. Make your changes. [`AGENTS.md`](./AGENTS.md) has the module map, the
   layer rules, and the ripple-effect checklists — read the relevant section
   before touching shared enums like `EdgeCurve` or `FaceSurface`.
3. Run the checks CI will run:

   ```bash
   cargo fmt --all
   cargo clippy --all-targets -- -D warnings
   cargo test --workspace
   ./scripts/check-boundaries.sh    # if you touched crate dependencies
   ./scripts/check-doc-paths.sh     # if you moved or renamed a source file
   ```

4. Commit with a signed-off conventional commit message.
5. Open a pull request describing what changed and how it was verified.

The hooks split the work: pre-commit runs the fast checks (fmt, clippy,
taplo, machete in parallel), commit-msg runs commitlint, and the full test
matrix — nextest, doc tests, cargo-deny, boundaries, lineage, audit — is
gated by CI on every push and PR.

## Commit messages

[Conventional commits](https://www.conventionalcommits.org/), enforced by
commitlint:

```
feat(sketch): add tangent-line-circle constraint
fix(algo): classify full-period quadrics analytically in ray-cast
docs: correct offset limitations in README
test(io): add STEP round-trip golden for cone seam
refactor(topology): simplify arena slot lookup
```

Scope with the crate name (minus the `remus-` prefix) when the change is
crate-local.

## Code style

- `rustfmt` defaults; CI checks formatting.
- No `unsafe`, no `unwrap()`, no `expect()`, no `panic!()` in library code —
  all denied by workspace lints. Return `Result` with the crate's typed error.
- Public items need doc comments (`missing_docs` warns).
- Tests may use `unwrap`/`expect` behind
  `#![allow(clippy::unwrap_used, clippy::expect_used)]` in the test module.

## Architecture

The workspace is a strictly layered DAG — each crate depends only on lower
layers, and `scripts/check-boundaries.sh` fails the build on a violation.
The layer table and per-crate allowed `use` paths are in
[`AGENTS.md`](./AGENTS.md). The one rule that is never bent: **no dependency
from a lower layer to a higher one.**

## Testing

- Unit tests live alongside the code; `proptest` for property-based coverage
  where the input space warrants it.
- Golden files (`tests/golden/`) for STEP/3MF round-trips.
- Integration tests in `tests/integration/`.
- **Bug fixes need a regression.** Where the failure is expressible through
  the batch API, land it as a [reproduction bundle](crates/wasm/src/repro.rs) —
  versioned JSON that replays the failing operation sequence identically on
  native and WASM. Every discovered defect is meant to become a permanent,
  replayable regression.
- Geometry claims need ground truth: verify volumes, watertightness, and
  manifoldness, not just "it didn't error".

## Security

Report vulnerabilities privately — see [SECURITY.md](./SECURITY.md). Please
don't open public issues for security-sensitive bugs.
