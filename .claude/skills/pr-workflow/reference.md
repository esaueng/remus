# pr-workflow reference

Deep catalog behind SKILL.md. Everything here was verified against the repo; when it disagrees with prose elsewhere (including CLAUDE.md's "Git Conventions"), the workflow files (`.husky/*`, `.github/workflows/*.yml`) are authoritative.

## CI jobs (`.github/workflows/ci.yml` → `fleet-ci-v4.yml`)

`ci.yml` runs on `pull_request`, `merge_group`, pushes to `main` and `workflow_dispatch`. Its `checks` job calls the reusable `.github/workflows/fleet-ci-v4.yml` at a pinned commit (the active callee is named in `.github/fleet-workflows.json`). There, `ci-pass` (display name `CI Pass`) is an aggregate job whose `needs` list is, verbatim:

```
[changes, repo-policy, test, approx-census, coverage, msrv, wasm, wasm-size-gate,
 wasm-no-io, fuzz-check, render, deny, audit, docs, secrets-scan]
```

It requires `changes`, `repo-policy` and `secrets-scan` to succeed, and accepts a skipped job only when the classifier deselected its tier (`docs/owner-pr-ci.md`, "Tiers"). Branch protection on `main` requires it under the context name `checks / CI Pass` (see "Repo merge settings"). It is the only required status check.

Outside the fan-in: the `wasm-size` comment job and `public-commit-metadata` in `ci.yml`, and other workflows' checks such as the `WASM version guard` jobs (`wasm-version.yml`). Read them yourself. The size policy is enforced inside the fan-in by `wasm-size-gate`.

| Job | Display name | In `CI Pass`? | What it runs |
|-----|--------------|---------------|--------------|
| `changes` | Classify Changes | yes (must succeed) | `scripts/classify-ci-changes.py`: selects the tiers |
| `repo-policy` | Repository Policy | yes (must succeed) | one job of cheap checks: Apache lineage, `cargo fmt --all -- --check`, layer boundaries, det-hash and wildcard-arm ratchets, documentation paths and module map, product naming, publish/gauntlet workflow contracts, fleet and CI policy tests, `cargo machete`, `taplo fmt --check` |
| `secrets-scan` | Secrets Scan | yes (must succeed) | `gitleaks` over the full history |
| `test` | Test | yes | `cargo clippy --all-targets --all-features -- -D warnings`, `cargo nextest run --workspace`, doc tests, complexity guards |
| `approx-census` | Approximation Census | yes | `scripts/check-approx-census.py` |
| `coverage` | Coverage | yes (tier 2) | llvm-cov with a 60% line floor; usually the slowest job (~40 min) |
| `msrv` | MSRV (1.88) | yes (tier 2) | `cargo +1.88.0 check --workspace --all-features` |
| `wasm-no-io` | WASM without optional I/O | yes | clippy and tests for `remus-wasm --no-default-features` |
| `wasm` | WASM Build & Validate | yes (package build) | `cargo xtask wasm-build`, package contents, W9 preflight |
| `wasm-size-gate` | WASM Size Gate | yes (skips without a package build or outside PRs) | base commit's `scripts/wasm-size-gate.mjs`: kernel hard limits, and a `WASM-Size:` line for kernel growth above 64 KiB raw |
| `fuzz-check` | Fuzz Targets Compile | yes | `cargo check --manifest-path fuzz/Cargo.toml --bins` |
| `render` | Software Rendering | yes | `cargo test -p remus-render` on Mesa lavapipe |
| `deny` | Cargo Deny | yes (tier 2) | `cargo deny ... --all-features check` |
| `audit` | Security Audit | yes (tier 2) | `rustsec/audit-check` |
| `docs` | Documentation | yes | `cargo doc --no-deps --all-features`, `mdbook build book` |
| `wasm-size` (`ci.yml`) | WASM Size Report | **no** | PR-only comment with the same report; fails with the gate |

There are no separate Format, Clippy, Layer Boundaries, Unused Dependencies, TOML Format, Apache Lineage or Doc Paths jobs: those checks run inside Test or Repository Policy, so a failure in one shows up as that job failing. macOS CI tests are disabled (`docs/owner-pr-ci.md`).

`benchmark.yml` contributes `Kernel perf` and `Publish benchmark` (PR runs only with the `ci:benchmark` label); neither is in `CI Pass`. `blacksmith-sh` posts a `[code]smith` entry that reports SKIPPED — it is the CI runner provider, not a reviewer or a quality check.

The size jobs check out `ref: ${{ github.event.pull_request.base.sha }}` (your PR's real base) and compare the PR build with the package COMMITTED there, not with a build of the base source. If a kernel change merged into the base and its package refresh has not landed yet, its growth shows up in your delta (and can trip the 64 KiB `WASM-Size:` rule); wait for the refresh or declare it. The refresh PR itself carries the `WASM-Size:` lines of the source PRs merged since the previous refresh (`scripts/publish-wasm-package-pr.py`) when it can collect them; when collection fails the description says so and the declaration has to be added by hand (as on #1208); a refresh that still fails the gate packages growth no source PR declared, so review those PRs and declare it in the refresh PR. Otherwise a nonzero delta on a change that cannot reach the binary (a `#[cfg(test)]`-only or docs-only diff) deserves a second look. `Cargo.lock` is committed, so both builds resolve from their checked-in locks; if your diff does not touch the lockfile, resolution is identical on both sides and a delta on a test-only diff is genuinely unexplained — investigate rather than blaming dependency drift.

Local pre-commit covers only fmt, clippy, taplo, machete, and the last two only when the binaries are installed (the hook skips them silently otherwise). Everything else (tests, boundaries, deny, docs) first runs in CI unless you run it yourself. Before pushing, run at minimum the tests for touched crates and, on any `Cargo.toml` change, `./scripts/check-boundaries.sh`.

## Repo merge settings (verified via `gh api repos/esaueng/remus`)

- Merges go through the live merge queue (`docs/owner-pr-ci.md`, "Convergence and merge queues"). `gh pr merge <N>` enqueues the PR; the queue's merge method is `SQUASH`, and the repository allows squash merging only.
- `allow_auto_merge: **true**` — `gh pr merge <N> --auto` arms the PR to enter the queue once its required checks pass.
- `delete_branch_on_merge: **true**` — GitHub deletes the head branch on merge, which auto-closes any PR stacked on it. Retarget stacked PRs to `main` before merging the parent.
- Branch protection on `main` (verified 2026-10-01): required status check `checks / CI Pass`; `required_conversation_resolution: true`, so every review thread must be resolved; required approvals none; `enforce_admins` false, so admins bypass the check. The queue activation in `docs/owner-pr-ci.md` turns `strict` off, because the queue builds each entry against current `main`; a PR no longer needs updating before it merges. The queue's template is `.github/merge-queue-ruleset.json`, which still reads `"enforcement": "disabled"`: per `docs/owner-pr-ci.md`, the live rule is created from it and activated in the repository settings, and merge-group runs on `gh-readonly-queue/main/pr-<N>-*` show it is active. Not pushing to `main` is still a rule you follow.
- Squash commit titles on `main` look like `type(scope): subject (#N)`.

## Code review

**No automated reviewer is installed on this repo, and no PR in its history has ever been reviewed by anyone.** Verified by surveying the 20 most recent PRs (`pulls/<N>/reviews` and `pulls/<N>/comments` both empty for every one) and by listing the apps posting check runs on a PR head commit — only `github-actions` and `blacksmith-sh`.

Do not wait for, poll for, or cite cubic, Greptile, or Copilot here. Earlier revisions of this skill described a `cubic · AI code reviewer` gate; that belonged to a different remote and never applied to this repo.

If an app is installed later, the two surfaces to read are:

```bash
gh api repos/esaueng/remus/pulls/<N>/comments   # inline (diff-anchored) comments
gh pr view <N> --comments                        # issue-level comments
```

Until then, the substitute for review is your own diff read plus the local test run, and honest reporting about which of those actually happened.

## Push details

- `origin` is `https://github.com/esaueng/remus.git`. Plain `git push -u origin <branch>` works; credentials come from the `osxkeychain` credential helper.
- There is **no** `url.*.insteadOf` rewrite in this checkout (`git config --get-regexp 'url\..*insteadOf'` is empty) and no SSH involved, so the old token-embedded-URL workaround is unnecessary. If you do use an explicit URL, redact the token in any output you show:

```bash
git push "https://x-access-token:$(gh auth token)@github.com/esaueng/remus.git" <branch> \
  2>&1 | sed 's/x-access-token:[^@]*@/x-access-token:***@/g'
```

- Explicit-URL pushes do not update local `origin/<branch>` tracking refs (plain `git push` does). After an explicit-URL push, verify against the remote rather than the tracking ref:

```bash
gh pr view <N> --json headRefOid --jq .headRefOid
git ls-remote --heads origin <branch>
```

- All `gh` operations (create, view, merge, api) go over HTTPS with the CLI token and work normally.

## Committed-package refresh (`.github/workflows/publish.yml`)

There is no release-please pipeline in this fork (no workflow references
release-please — verify with `rg -l 'release-please' .github/workflows/`, and
note the leftover `release-please-config.json` / `.release-please-manifest.json`
are inert). What actually runs, verified against `publish.yml` and
`scripts/publish-wasm-package-pr.py` (2026-10-20):

- Workflow name "Refresh Apache Staging Package"; runs on pushes to `main`
  that touch a package-affecting path. The path filter excludes
  `crates/wasm/pkg/**` and `crates/wasm-io/pkg/**`, so merging a refresh PR
  does not start another refresh.
- Rebuilds `crates/wasm/pkg` and `crates/wasm-io/pkg` with `cargo xtask wasm-build`,
  stamps stable provenance fields (never the commit SHA — that would dirty
  every run), and when the rebuilt package differs from `main`, runs
  `scripts/publish-wasm-package-pr.py` with a token from the package bot App
  (`remus-package-bot`). The script pushes branch
  `codex/wasm-refresh-<source sha>` and opens a ready PR titled
  `chore(wasm): refresh committed packages to vX.Y.Z`. It never pushes to
  `main` and never merges. The version is assigned by `cargo xtask wasm-build`
  (changed bytes advance the patch from the committed packages) and checked by
  `scripts/check-wasm-version.py`, not taken from PR titles.
- The refresh PR runs no heavy CI tier (mode `package`: the required
  "Committed packages match source" job rebuilds both packages, compares every
  file and applies the size gate) and goes through the merge queue like any
  other PR, landing as `chore(wasm): refresh committed packages to vX.Y.Z (#N)`.
  Merging it IS the release: consumers install by git path
  (`github:esaueng/remus#main&path:/crates/wasm/pkg`), so whatever is
  committed there on `main` is what they get. There is no tag, no GitHub
  release, and no npm or crates.io publish.
- After opening (or finding) its PR, the script closes the same-repository
  refresh PRs built from an ancestor of the new source commit and deletes
  their branches (esaueng/remus#1204). Left open, an older refresh would fail
  the version guard once a newer one merged.
- Manual refresh: `workflow_dispatch` with the `sync_package` boolean input,
  from `main` only.
- Credentials fail closed: `scripts/require-publish-credentials.sh` fails the
  run when `REMUS_BOT_APP_ID` (repository variable) or `REMUS_BOT_PRIVATE_KEY`
  (`remus-bot` environment secret) is empty.
- Cross-repo consumption and the package validation harness: see the
  release-flow skill.

## CI failures you did not cause

Supply-chain jobs can fail on a PR that never touched dependencies. `Cargo.lock` is committed (tracked; dependabot updates it), and all jobs — including `audit` — build and audit the pinned resolution reproducibly. But `deny` and `audit` fetch the advisory database live, so a newly published advisory against a pinned version changes the verdict with zero diff on your branch. A new dep release alone does not (it only reaches CI through a lockfile bump).

### cargo-deny / audit / OSV advisories

The `deny` job runs cargo-deny-action against `deny.toml`; the `audit` job runs rustsec/audit-check on a freshly generated lockfile. Both gate PRs through `CI Pass`. OSV lives in its own workflow (`.github/workflows/osv-scan.yml`): on PRs it is report-only and does not gate; pushes to `main` and the Monday 06:00 UTC schedule fail closed.

Policy: do NOT widen the `deny.toml` license allowlist and do NOT add blanket ignore entries to get green. Current state to preserve: `licenses.allow` has eight permissive entries, the four non-obvious ones (CC0-1.0, ISC, BSD-2-Clause, BSD-3-Clause) carrying a provenance comment that names the dependency tree pulling them in, and there is no `[advisories] ignore` list at all (`[advisories]` sets only `unmaintained` and `yanked`).

Triage order when an advisory lands on your PR:

1. Check whether a patched version exists (`cargo audit` output or the advisory page names it). Semver-compatible patches are picked up automatically by the fresh resolution, so a persistent failure usually means the fix is in a new major or minor. Raise the version requirement in the relevant `Cargo.toml` in a separate commit, not mixed into your feature diff.
2. If no patched version exists, add a narrowly scoped `[advisories] ignore` entry to `deny.toml`: the specific advisory id, a comment explaining why it does not apply or cannot be fixed yet, and a concrete re-check trigger (e.g. "remove when <crate> X.Y ships"). State the ignore and its rationale in the PR body.
3. If unsure whether the advisory is exploitable or how to scope it, stop and report per the blocked-state rule. Silencing a security check is never a way to unblock a PR.

### MSRV job

Workspace `Cargo.toml` declares `rust-version = "1.88"`; the CI job `MSRV (1.88)` runs `cargo check --workspace --all-features` on toolchain 1.88.0. Because resolution is fresh, a dependency releasing a version that needs newer Rust breaks this job on unrelated PRs. The errors are confusing: syntax, edition, or feature errors deep inside the dependency, never the word MSRV. Fix: constrain that dependency in `Cargo.toml` to the last version that builds on 1.88. Do not bump `rust-version` casually; it is a public contract, and raising it is its own PR with its own justification.

### wasm-bindgen pin

The workspace `Cargo.toml` pins `wasm-bindgen = "=0.2.125"` and says why in an inline comment: the crate version must match the wasm-bindgen-cli tooling. The coupling is enforced in `xtask/src/wasm.rs` via the `WASM_BINDGEN_VERSION` constant; `cargo xtask wasm-build` bails when a locally installed wasm-bindgen-cli differs. The two locations must move together, and the xtask constant has lagged the Cargo.toml pin before, so verify both:

```bash
rg -n 'wasm-bindgen' Cargo.toml xtask/src/wasm.rs
```

Bumping wasm-bindgen is its own change with its own PR. Never bump it as a drive-by to fix an unrelated failure, and never let a general `cargo update`-style version bump move it silently.

### Scheduled workflows

- `.github/workflows/mutants.yml` (Mutation Testing): Sundays 02:00 UTC plus manual dispatch. Runs cargo-mutants on `remus-math` and `remus-algo` and uploads a report artifact. It never runs on PRs and never gates them; a red scheduled run is a signal to improve tests, not a merge blocker.
- `.github/workflows/osv-scan.yml`: Mondays 06:00 UTC plus main pushes, fail-closed there; report-only on PRs (see above).
- `benchmark.yml` runs on pushes and PRs, not on a schedule, and is not part of `CI Pass`.

## Symptom table

| Symptom | Cause | Action |
|---------|-------|--------|
| `git push` hangs or is rejected | Not the historical SSH problem: `origin` is HTTPS with no rewrite. Suspect expired keychain credentials or a network block | `git ls-remote origin` to isolate auth from the push itself |
| `origin/<branch>` does not match what you pushed | You pushed via an explicit URL; those do not update tracking refs | Verify with `gh pr view <N> --json headRefOid` or `git ls-remote` |
| commitlint prints `✖ found N problems` yet the commit succeeded | The commit-msg hook swallows commitlint failures and exits 0 by design of its fallback | Treat as a rejection: `git commit --amend` to `type(scope): subject` form |
| pre-commit fails on clippy warnings you did not write | Pre-existing breakage on the branch base | Stop and report; never `--no-verify` |
| `⚠️ commitlint not available` warning on commit | `node_modules` missing, but only if no `✖` lines print above it; the same warning also follows a real lint failure (see the `✖` row) because the hook's fallback fires on any nonzero commitlint exit | If `✖` lines precede it, fix the message; otherwise `npm install` at repo root. Either way the commit went through unchecked, re-verify the message manually |
| PR shows mergeable with nothing green yet | `mergeable` only means conflict-free; `mergeStateStatus` carries the check state | Read `mergeStateStatus`: `BLOCKED` waits on `CI Pass` or an open review thread |
| A review check never appears, however long you poll | No reviewer app is installed on this repo; the check does not exist | Stop waiting. Self-review the diff and report that no reviewer ran |
| Enqueued PR left the queue without merging | Its merge-group run (`gh-readonly-queue/main/pr-<N>-*`) failed or timed out, tested against current `main` | Read that `merge_group` run, fix on the branch, enqueue again |
| `BLOCKED` with every check green | An unresolved review thread; protection requires conversation resolution | Address the thread, then ask its author to resolve it (GraphQL `resolveReviewThread`); auto-merge fires on resolution |
| `gh pr update-branch` prints usage text | The installed `gh` predates that subcommand | `gh api -X PUT repos/esaueng/remus/pulls/<N>/update-branch -f expected_head_sha=<head>` |
| A stacked PR went CLOSED on its own | Its base branch was deleted when the parent merged (`delete_branch_on_merge` is on); GitHub auto-closes in that case | Not reversible — base cannot be retargeted while closed, and it cannot reopen with a missing base. Rebase onto `main` and open a new PR. Avoid by retargeting the child to `main` BEFORE deleting the parent branch |
| `CI Pass` missing from the rollup while jobs still run | It only appears once every job in its `needs` list finishes | Not a failure; keep polling. Coverage is the usual straggler |
| No `CI` run at all on the PR head; only `pull_request_target` workflows (privacy checks) report | The head commit message contains a skip marker (`[skip ci]`, `[ci skip]`, `[no ci]`, `[skip actions]`, `[actions skip]`) or ends with a `skip-checks: true` / `skip-checks:true` trailer, so GitHub skips its `push` and `pull_request` workflows; quoting the marker in prose counts | Push a real follow-up commit whose message has no marker (never an empty commit); keep the marker out of the PR description too, because the squash commit on `main` uses it |
| `WASM Size Report` shows a delta on a test-only diff | Either a base package that predates a merged kernel change (the baseline is the package committed on your base, not a build of it) or a real change. Both sides build from committed `Cargo.lock`s, so unless your diff touches the lockfile, resolution is identical | Check whether the base has kernel commits after its last package refresh; if not, confirm your diff cannot reach the binary or the lockfile, then report the delta as unexplained rather than calling it expected drift |
| CI `boundaries` job fails | A crate dependency violates the layer rules | Run `./scripts/check-boundaries.sh` locally; see the layer-boundaries skill |
| CI `taplo` or `machete` fails but pre-commit passed | Tool not installed locally; the hook skips it silently | `cargo install taplo-cli cargo-machete`, fix, re-commit |
| Compliance grep hits in a file you touched | You introduced a banned reference-kernel name, or you touched a grandfathered file | Remove new occurrences; leave grandfathered ones as-is |
| Expecting a release-please PR after merging | There is no release-please in this fork; the leftover config files are inert | The release is the package refresh PR, which still has to merge through the queue (see "Committed-package refresh") |
| No refresh PR opened after a kernel merge | The push missed the path filter, its head commit message carries a skip marker or `skip-checks` trailer (see the row above), a newer push cancelled the run, the rebuilt package matched `main` (the run logs a notice), or `REMUS_BOT_*` is missing (the run fails at "Require publish credentials") | Read the latest "Refresh Apache Staging Package" run |
| A refresh PR fails the version guard in the queue | An older refresh PR is still open after a newer one merged, and reuses its version | The publisher closes superseded refresh PRs itself (esaueng/remus#1204); one still open predates that, or its source is not an ancestor of the newer build. Close it by hand |
| `deny` or `audit` fails on a PR that never touched deps | A newly published advisory against a version pinned in the committed `Cargo.lock` (both jobs fetch the advisory DB live) | Follow the triage order in "CI failures you did not cause"; never blanket-ignore |
| MSRV job fails with syntax or feature errors inside a dependency | The committed `Cargo.lock` now pins a dep version requiring Rust newer than 1.88 — usually after a dependabot lock bump | Constrain that dep in `Cargo.toml` (or downgrade it in the lock); do not bump `rust-version` |
| `cargo xtask wasm-build` bails with a wasm-bindgen-cli version mismatch | Local CLI differs from the pin; the crate pin and `xtask/src/wasm.rs` constant must match | Install the pinned CLI version; bump the pin only as its own PR |
| You pushed straight to `main` and it succeeded | Admin pushes bypass protection (`enforce_admins` is false) | Do not rely on the server. Branch and open a PR; if you already pushed, say so immediately |

## Anti-patterns (what NOT to conclude)

- "The review came back clean": no reviewer runs on this repo. An empty findings list means nobody looked, which is not the same as approval. Report which gate actually passed.
- "The check I polled never reported problems, so there were none": a filter keyed to a nonexistent check name is silent forever and looks identical to a pass. List the rollup unfiltered before trusting any filter.
- "The pre-push hook printed one line and passed, so the change is validated": the hook intentionally runs nothing. Validation is CI plus the local tests you ran yourself.
- "CLAUDE.md says pre-push runs tests and cargo-deny": stale. The hook file delegates to CI; do not re-add local suites to it and do not cite the stale description.
- "High-risk change, better wait for a human": no gate of any kind will stop you, which is an argument for more self-review, not less.
- "gh pr view showed no comments, so there are no findings": inline findings live on `pulls/<N>/comments` (the API), check both surfaces.
- "Branch protection will catch it": protection checks only `CI Pass`, freshness and resolved threads, admins bypass it, and three checks sit outside it. It catches a red `CI Pass` or an open thread, nothing else.
- "The plan doc helps reviewers, commit it": working plans and specs never get committed.
- "The commit went through, so the message passed commitlint": the commit-msg hook never blocks. Check the hook output for `✖` lines and amend if any appeared.
