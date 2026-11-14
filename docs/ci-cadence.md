# Development CI cadence

How CI runs today. Every job runs on GitHub-hosted `ubuntu-24.04` runners.
When this page and a workflow disagree, the workflow is right; fix the page.

## Where it lives

- `.github/workflows/ci.yml` is the entry point. It runs on pull requests,
  merge-queue groups (`merge_group`), pushes to `main` and manual dispatch.
  It calls `./.github/workflows/ci-checks.yml` from the same commit, so a
  change to either file takes effect in the pull request that makes it.
- `ci-checks.yml` holds every job behind the required status check
  `checks / CI Pass`. `main` requires five checks: `checks / CI Pass`,
  `Changed kernel requires a new version`, `commit-privacy`,
  `history-privacy` and `dependency-review`.
- `ci.yml` also runs, outside `CI Pass`: `public-commit-metadata`, the
  `WASM Size Report` PR comment (`wasm-size-report.yml`, it needs a write
  token) and, on a protected `main` push that built the package, the
  package publisher (`publish.yml`, see the release-flow skill).
- Other workflows (`wasm-version.yml`, `dependency-review.yml`, the
  history-privacy workflows, `benchmark.yml`, `parity.yml`, `osv-scan.yml`,
  `mutation-scope.yml` and the scheduled ones below) are each one
  self-contained file.

## Tier selection

The first job, Classify Changes, runs `scripts/classify-ci-changes.py` on the
diff (its module docstring and `scripts/test-classify-ci-changes.py` are the
reference). Repository Policy and Secrets Scan start at once; every other
job starts as soon as Classify Changes finishes. Nothing waits for Repository
Policy or Secrets Scan.

| Tier | Jobs | Selected when |
| --- | --- | --- |
| always | Classify Changes, Repository Policy, Secrets Scan | every run |
| `heavy` | Test, Native tests (1/8 to 8/8), Clippy, Complexity Guards, Approximation Census, WASM without optional I/O, Fuzz Targets Compile, Software Rendering | a PR or `main` push whose diff is neither only docs and metadata (docs, `.claude/`, `.github/` outside `workflows/`) nor only the committed packages |
| `wasm` | WASM Build & Validate, WASM runtime suites, W9 Preflight, WASM Size Gate (PRs only) | a heavy run whose diff touches a WASM-affecting path: anything under `crates/`, `xtask/` or `tools/vs-bench/`, the root manifests and toolchain, the WASM-related workflows and scripts |
| `full` | MSRV (1.88), Cargo Deny, Security Audit | a heavy run whose diff touches `Cargo.toml`, `Cargo.lock`, `deny.toml`, `clippy.toml`, `rust-toolchain.toml`, `.cargo/`, `.config/` or `.github/workflows/` |
| `docs` | Documentation (rustdoc and the mdBook) | every heavy run, every merge group with a source change, and a diff that touches docs |
| `coverage` | Coverage (instrumented, 60% line floor) | only on request: the `ci:coverage` label or the dispatch `coverage` input; never for fork PRs |

The `ci:full` label and a manual dispatch select every tier whatever the
diff. A diff confined to `crates/wasm/pkg/` and `crates/wasm-io/pkg/` (a
package refresh PR) selects nothing here: `wasm-version.yml` rebuilds both
packages and compares every committed file. A diff of only docs and
metadata runs the light gates plus Documentation when docs changed.

`CI Pass` (`scripts/ci_gate.py`) fails unless every selected job succeeded.
A job may be skipped only when its tier was not selected; a failure,
cancellation or missing result fails the gate.

## What runs where

- **Pull request:** the tiers selected for the diff between the PR's base
  and its merge ref. A new push cancels the PR's running CI.
- **Merge queue:** light gates only: Repository Policy, Secrets Scan and
  Documentation in `CI Pass`, plus the privacy, dependency-review, OSV, WASM
  version and performance-harness-contract workflows. The pull request
  already ran its tiers on its merge ref, and the `main` push of the squashed
  tree runs them again. Two PRs that pass alone but break together therefore
  turn `main` red after merging rather than blocking the queue. Fix that
  forward at once.
- **Push to `main`:** the tiers selected for the pushed diff. Every `main`
  commit keeps its own run; nothing cancels it.
- **Manual dispatch** of `ci.yml`: every tier; tick the `coverage` input to
  add coverage.

A PR that changes the coverage tooling (`scripts/ci_coverage.py`, its tests,
`.config/nextest.toml` or the coverage workflows) also runs
`coverage-tooling.yml`: the archive and scheduler fixtures and the
budget-sensitive instrumented replay.

## Labels

- `ci:full` selects every tier on a PR.
- Adding `ci:full` or `ci:coverage` does not start a run by itself. The label
  is read by the next run: push a commit, or re-run CI.
- `ci:coverage` adds the instrumented coverage job on a same-repository PR.
- `ci:benchmark` runs `benchmark.yml` (`Kernel perf` and its publisher) on a
  PR; later pushes keep benchmarking while the label stays.
- `hold` marks a PR that waits for an owner decision; do not enqueue it.

## Schedules

All times UTC; scheduled runs use `main`.

| Workflow | When | What |
| --- | --- | --- |
| `gauntlet.yml` | daily 03:17, Sunday 03:47 | CAD corpus gauntlet: the daily smoke tier, and the weekly `abc-1k` tier on Sunday |
| `parity.yml` | daily 05:43 | native/WASM parity (also on PRs that touch its paths) |
| `benchmark.yml` | daily 06:29 | baseline kernel benchmarks |
| `daily-validation.yml` | daily 06:43 | Daily dependency validation: Cargo Deny, the advisory audit and MSRV against the committed lockfile |
| `coverage.yml` | daily 07:13 | complete instrumented coverage through `coverage-sharded.yml`, 60% line floor |
| `history-privacy.yml` | daily 08:23 | history privacy audit |
| `mutants.yml` | Sunday 02:00 | sharded mutation testing |
| `fuzz.yml` | Sunday 03:30 | fuzz targets |
| `opseq81-campaign.yml` | Sunday 04:30 | bounded operation-sequence campaign |
| `osv-scan.yml` | Monday 06:00 | OSV scan (also on `main` pushes, and on PRs and merge groups, where it fails on newly introduced vulnerabilities) |

There is no daily full native run: every `main` push with a source change
already runs the complete native suite and Clippy.

## Repository Policy locally

```bash
python3 scripts/ci_preflight.py
```

This runs the same `CHECKS` catalog as the Repository Policy job (the
catalog in `scripts/ci_preflight.py` is the source of truth). It runs every
check, reports each failure with its exit code, and treats a missing tool as
a failure. Install the pinned Rust toolchain, `cargo-machete` and
`taplo-cli` first. `--json <file>` writes the outcomes and durations. It does
not run Clippy or the native tests.

## Native tests in CI

CI always runs the complete native suite. The Test job runs
`scripts/ci_native.py archive`, which writes git-ignored harness files
`tests/ci_harness_<n>.rs` in each package. Each harness includes that
package's integration test files as modules, so a few dozen test binaries
are linked instead of several hundred. Test also runs the doc tests. The
eight Native tests jobs each run one partition of that archive with
`ci_native.py run --shard <i> --total 8`.

Consequence for reading CI logs: a test from `crates/operations/tests/<foo>.rs`
is reported as `remus-operations::ci_harness_<n> foo::test_name`, not
`remus-operations::foo test_name`. Locally nothing changes:
`cargo test -p remus-operations --test foo` and `cargo nextest run` work as
before. A test file that must be its own binary (it uses `CARGO_BIN_EXE_*`,
`current_exe`, `module_path!`, `crate::` paths, a crate-level `#![feature]`
or `#![recursion_limit]`, and the other patterns in `STANDALONE` in the
script) stays standalone automatically.

## Merge queue and merge gate

`main` requires `checks / CI Pass` and every review conversation resolved;
merges go through the merge queue. `gh pr merge <N>` enqueues a PR. The
queue's template is `.github/merge-queue-ruleset.json`: squash merging (the
only merge method the repository allows), `ALLGREEN`, at most four entries
built at once and a 240-minute check timeout. The full merge procedure is in
the pr-workflow skill (`.claude/skills/pr-workflow/SKILL.md`).

## Changing CI

Edit the workflow and open one pull request; that PR's own run uses the
edited `ci.yml` and `ci-checks.yml`. There is no pinning or second
activation PR. Keep these in step with a change:

- the required context `checks / CI Pass`: renaming the `checks` job in
  `ci.yml` or the `CI Pass` job breaks branch protection;
- the job groups in `scripts/ci_gate.py` and the `needs` list of `ci-pass`
  when adding or removing a job (`scripts/test_ci_native.py` checks them);
- `scripts/classify-ci-changes.py` and its tests when a new path should
  select or skip a tier;
- the `CHECKS` catalog in `scripts/ci_preflight.py` for a new repository
  policy check;
- this page.

## Package qualification

`wasm-version.yml` requires the committed packages to match a clean source
build, or reuses an exact protected-`main` qualification of the same source
(`scripts/reuse-wasm-qualification.py`, described in [Reusing exact package
qualification](ci-package-qualification.md)). Each reuse decision writes a
compact `qualification` JSON record and job summary with the outcome and the
stage it stopped at. `package_mismatch` means a committed file or manifest
differs from the rebuilt pair; `provenance_mismatch` means the committed
publisher metadata is wrong; `invalid_package_layout` rejects missing
packages or unsupported entries. Unavailable or malformed evidence reports
`proof_unavailable`, sets `reused=false` and falls back to the clean build.
