---
name: release-flow
description: Getting a merged Remus change into the hands of a consumer. Use when a feature PR has merged and someone needs the new kernel, when refreshing the committed WASM package, when validating a package build before handing it over, or when asked whether Remus can publish to npm or crates.io. Covers the committed-snapshot channel, the validation harness, and the release-ownership gate that blocks publishing.
---

# release-flow: merged change to consumer

## When to use

A Remus change is not in a consumer's hands when its PR merges. This repository
**publishes nothing** — no crates.io, no npm, no GitHub releases — so the only
channel that reaches a consumer is the pair of WASM packages committed at
`crates/wasm/pkg` (kernel) and `crates/wasm-io/pkg` (file-format translators),
installed by git path. This skill covers that channel: how
it is refreshed, how to validate a build, and what publishing is still gated
on.

For getting the feature PR merged, see `pr-workflow`. For benchmark
comparisons, see `parity-benchmarking`. To test an unreleased build inside a
JS consumer without touching the committed snapshot, see `wasm-bindings`.

## The one hard rule

**Remus does not publish.** `docs/production-readiness/fork-maintenance.md`
gates every published artifact behind release ownership that does not yet
exist: named maintainers, package identity, vulnerability intake, signing and
provenance, rollback and yank authority. Until those are established:

- Do not run `npm publish` or `cargo publish` from this repository.
- Do not create GitHub releases or tags intended as releases.
- An npm package under this project's name — or under the name it carried
  before the rename — did not come from here.

`release-please-config.json` and `.release-please-manifest.json` exist in the
tree, but **no workflow runs release-please**. Do not infer a release pipeline
from their presence; verify with
`rg -l 'release-please' .github/workflows/` before believing otherwise.

## The actual chain

| Hop | Action | Gate before next hop |
|-----|--------|---------------------|
| 1 | Feature PR squash-merges to `main` through the merge queue | `checks / CI Pass` green on the merge group |
| 2 | Build and validate both packages locally | `cargo xtask wasm-build` succeeds; both consumer harnesses pass |
| 3 | The refresh workflow rebuilds both snapshots and opens a refresh PR | The refresh PR `chore(wasm): refresh committed packages to vX.Y.Z` passes CI and merges through the queue |
| 4 | Consumer reinstalls from the git path | Consumer's own tests pass against the new snapshot |

Hop 3 opens its PR automatically on every package-affecting push to `main`, but
the PR still has to be enqueued and merged like any other; until it lands, `main`
serves the previous snapshot. Hop 4 is the consumer's move, whenever one exists.

## Hop 2: build and validate

```bash
cargo xtask wasm-build              # both packages: dual-target build, merge, validate
node scripts/test-wasm-smoke.mjs    # loads both packages, runs consumer regressions
node scripts/test-wasm-tarball-consumer.mjs   # npm pack both into a disposable consumer
```

The tarball test is the stronger of the two: it packs both packages and
installs them into a throwaway consumer, so it catches `files`-list omissions
and entry point mistakes that the in-tree smoke test cannot. Both harnesses
exercise the kernel and translator modules together: a STEP round trip is
`io.exportStep(kernel.serializeSolids(ids))` then
`kernel.deserializeSolids(io.importStep(bytes))`.

Both run the regressions in `scripts/openzcad-wasm-consumer-regressions.mjs`,
which encode consumer-shaped geometry rather than synthetic cases — for
example the flange demo's "Union flange blank": two coaxial revolved annuli
sharing an exact cylindrical wall, which must fuse analytically. A mesh
fallback can stay watertight, valid, and close on volume, so those assertions
check face counts and surface kinds, not just volume. **Do not relax them to
get a build out.**

## Hop 3: refreshing the committed snapshot

Workflow: **Refresh Apache Staging Package** (`.github/workflows/publish.yml`),
on every push to `main` that touches a package-affecting path (`crates/**`
except the two `pkg` directories, `xtask/**`, `Cargo.toml`, `Cargo.lock`,
`.cargo/**`, `rust-toolchain.toml`, `LICENSE-APACHE`, the publisher and WASM
version scripts, and the workflow itself), and by `workflow_dispatch`.

Two jobs. `build-committed-package` runs `cargo xtask wasm-build`,
stamps fork provenance into both `package.json` files, and uploads the pair
as one artifact. `sync-committed-package` (environment `remus-bot`) downloads
it, installs it through `scripts/install-wasm-package-archive.py` (which
accepts only regular files and directories under `crates/wasm/pkg` and
`crates/wasm-io/pkg` — never extract a build-produced archive straight into the
checkout), and, when the staged packages differ from `main`, runs
`scripts/publish-wasm-package-pr.py` with a token from the package bot App
(`remus-package-bot`). That script commits the packages on branch
`codex/wasm-refresh-<source sha>` and opens a ready PR titled
`chore(wasm): refresh committed packages to vX.Y.Z`. It never pushes to `main`.

Notes that matter:

- It force-stages (`git add --force`) because wasm-pack writes a `*`
  `.gitignore` into its own output directory.
- The refresh PR runs the full suite (a diff confined to the two `pkg`
  directories always does, see `docs/owner-pr-ci.md`, "Tiers") and goes
  through the merge queue like any other PR. It does not merge on its own.
- Its description carries the `WASM-Size:` lines of the source PRs merged
  since the previous refresh, so the size gate passes when those PRs declared
  their growth.
- The branch is pinned to its source commit. A rerun for the same source reuses
  the branch when the output is identical and refuses to overwrite it when it
  differs.
- A newer push to `main` cancels a refresh run still in progress
  (`concurrency` with `cancel-in-progress`). Older refresh PRs stay open
  today; esaueng/remus#1204 (open as of 2026-10-20) makes the script
  close the open refresh PRs built from an ancestor of the new source commit.

### Why it refreshes on every push

A committed package that lags its source is worse than none: it serves old
kernel behavior while the Rust source reads as current, and nothing about the
repository shows the drift. Merging a refresh PR changes only the two `pkg`
directories, which the workflow's path filter excludes, so the refresh cannot
loop.

This ran manual-only for one cycle, while the snapshot still carried the
pre-rename package name and a consumer might have been importing it. With the
snapshot regenerated under the current name and nothing pinned to the old one,
that reason is spent.

## Hop 4: the consumer

The documented consumer installs the snapshot by git path:

```
"remus-wasm": "github:esaueng/remus#main&path:/crates/wasm/pkg"
"remus-wasm-io": "github:esaueng/remus#main&path:/crates/wasm-io/pkg"
```

The two must be pinned to the same commit: the arena document format they
exchange is versioned by the kernel, and `cargo xtask wasm-build` refuses to
produce them at different package versions.

No consumer is pinned to this today. The form above is what the repository
documents; an older pin naming `apache-main` will not resolve, because that
branch was deleted. A git-path install resolves the ref at install time, so a
consumer picks up each refresh on its next install unless its lockfile pins an
older commit.

There is also a manual **Build OpenZCAD WASM Candidate** workflow
(`.github/workflows/openzcad-wasm-release.yml`). It is validation-only: it
builds, stamps provenance, `npm pack`s, and uploads a short-lived artifact. It
cannot push commits or create releases. Use it to hand someone a candidate
tarball without touching `main`.

## Traps

| Symptom | Cause | Fix |
|---|---|---|
| "Where is the npm package?" | There isn't one; this fork publishes nothing | Build from source, or use the committed snapshot by git path |
| A release-please PR is expected but never appears | The config exists; no workflow runs it | Do not wait for it; there is no release automation |
| Refresh workflow does nothing | The push touched no package-affecting path, a newer push cancelled the run, or the rebuilt packages already match `main` (the run logs a notice); a dispatch additionally requires `sync_package` and `github.ref == refs/heads/main` | Check the path filter and the run's notice; dispatch against `main` with the input enabled |
| Kernel PR merged but `main` still serves the old package | The refresh PR it triggered has not merged yet | Find the open `codex/wasm-refresh-<sha>` PR and get it through the queue |
| Refresh PR merged but a consumer sees nothing | Its pin names a deleted branch, or its lockfile pins an older commit | Check the pin; git-path installs resolve the ref at install time |
| Refresh run fails at "Require publish credentials" | `REMUS_BOT_APP_ID` (repository variable) or `REMUS_BOT_PRIVATE_KEY` (`remus-bot` environment secret) is unset; `scripts/require-publish-credentials.sh` fails closed | Set the missing value in the store the error names |
| Consumer regressions fail only through the tarball | A `files` omission or entry-point error in the generated `package.json` | Fix the package metadata; do not skip the tarball test |

See [reference.md](reference.md) for the package layout, the provenance stamp,
and the build anatomy.
