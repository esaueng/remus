---
name: release-flow
description: Getting a merged Remus change into the hands of a consumer. Use when a feature PR has merged and someone needs the new kernel, when refreshing the committed WASM package, when validating a package build before handing it over, or when asked whether Remus can publish to npm or crates.io. Covers the committed-snapshot channel, the validation harness, and the release-ownership gate that blocks publishing.
---

# release-flow: merged change to consumer

## When to use

A Remus change is not in a consumer's hands when its PR merges. This repository
**publishes nothing** — no crates.io, no npm, no GitHub releases — so the only
channel that reaches a consumer is the WASM package committed at
`crates/wasm/pkg`, installed by git path. This skill covers that channel: how
to refresh it, how to validate it, and what is deliberately not automated.

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
| 1 | Feature PR squash-merges to `main` | `CI Pass` green on the merge |
| 2 | Build and validate the package locally | `cargo xtask wasm-build` succeeds; both consumer harnesses pass |
| 3 | Refresh the committed snapshot (manual dispatch) | New `chore(wasm): refresh committed package` commit on `main` |
| 4 | Consumer reinstalls from the git path | Consumer's own tests pass against the new snapshot |

Hops 3 and 4 are **not automatic**. See "Why hop 3 is manual" below.

## Hop 2: build and validate

```bash
cargo xtask wasm-build              # dual-target build, merge, validate
node scripts/test-wasm-smoke.mjs    # loads the package, runs consumer regressions
node scripts/test-wasm-tarball-consumer.mjs   # npm pack into a disposable consumer
```

The tarball test is the stronger of the two: it packs the package and installs
it into a throwaway consumer, so it catches `files`-list omissions and entry
point mistakes that the in-tree smoke test cannot.

Both run the regressions in `scripts/openzcad-wasm-consumer-regressions.mjs`,
which encode consumer-shaped geometry rather than synthetic cases — for
example the flange demo's "Union flange blank": two coaxial revolved annuli
sharing an exact cylindrical wall, which must fuse analytically. A mesh
fallback can stay watertight, valid, and close on volume, so those assertions
check face counts and surface kinds, not just volume. **Do not relax them to
get a build out.**

## Hop 3: refreshing the committed snapshot

Workflow: **Refresh Apache Staging Package** (`.github/workflows/publish.yml`),
`workflow_dispatch` only, on `main`.

Two jobs. `build-committed-package` runs `cargo xtask wasm-build --skip-opt`,
stamps fork provenance into `package.json`, and uploads the result as an
artifact. `sync-committed-package` downloads it, installs it through
`scripts/install-wasm-package-archive.py` (which accepts only regular files and
directories under `crates/wasm/pkg` — never extract a build-produced archive
straight into the checkout), commits, and pushes to `main` with `[skip ci]`.

Notes that matter:

- It force-stages (`git add --force`) because wasm-pack writes a `*`
  `.gitignore` into its own output directory.
- It never rebases. A concurrent push to `main` makes the push fail on
  purpose; the newer source commit starts its own refresh.
- The commit is `[skip ci]`, so the refreshed snapshot is not itself re-tested
  by CI. Hop 2 is where the validation happens.

### Why hop 3 is manual

The push trigger was removed deliberately, not lost. Automatic refresh on every
push to `main` would regenerate `crates/wasm/pkg` under the current package
name — and after the rename that means the snapshot stops being the package the
existing consumer imports. `fork-maintenance.md` keeps the snapshot frozen
until the consumer migrates. **Restore the push trigger in the same change that
repoints the consumer, not before.**

## Hop 4: the consumer

The documented consumer installs the snapshot by git path:

```
"remus-wasm": "github:esaueng/remus#main&path:/crates/wasm/pkg"
```

⚠️ **Verify this pin before relying on it.** It previously referenced
`apache-main`, a branch that has since been deleted, so any consumer still
holding the old pin has a broken install. The `#main` form above is what the
repository now documents; whether the consumer has repinned is not observable
from inside this repository. Confirm with whoever owns it.

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
| Refresh workflow does nothing | It is `workflow_dispatch` only, and the job also requires `sync_package` and `github.ref == refs/heads/main` | Dispatch it against `main` with the input enabled |
| Snapshot commit succeeds but the consumer sees nothing | Consumer pin points at a deleted branch, or its lockfile pins an older commit | Check the pin; git-path installs resolve at install time |
| App-token step silently skipped | `REMUS_BOT_APP_ID` / `REMUS_BOT_PRIVATE_KEY` unset; `HAS_APP_CREDENTIALS` evaluates false without failing | Set both secrets; the job degrades quietly by design |
| Consumer regressions fail only through the tarball | A `files` omission or entry-point error in the generated `package.json` | Fix the package metadata; do not skip the tarball test |

See [reference.md](reference.md) for the package layout, the provenance stamp,
and what "frozen snapshot" means for the binary itself.
