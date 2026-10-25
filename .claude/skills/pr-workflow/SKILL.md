---
name: pr-workflow
description: Use when committing, pushing, opening, reviewing, or merging a pull request in this repo (esaueng/remus, the Remus B-Rep kernel), or when a git hook fails, a push fails, commitlint flags a message, or parallel work needs a worktree. Covers hooks, conventional commits, the self-imposed merge gate, and the committed-package release channel.
---

# PR Workflow

End-to-end change flow for this repo: branch, commit, push, PR, merge gate, squash-merge, release. Every change lands as a squash-merged PR.

**CI and open threads block a merge here, never a reviewer's approval.** `main` requires the `checks / CI Pass` status and every review conversation resolved, and merges go through the live merge queue, which runs CI on the PR combined with current `main` (for an ordinary source PR only the light gates while the classifier's queue fast path is on; `docs/owner-pr-ci.md`, "Convergence and merge queues", is the authoritative description of the tiers and the queue). Zero required approvals, admins not enforced. No human reviews PRs, but the `chatgpt-codex-connector` bot posts inline P1/P2 review threads on many PRs, and each unresolved one blocks the merge. Everything in the gate below beyond a green `CI Pass` and resolved threads is self-imposed.

## Quick reference

| Task | Command |
|------|---------|
| Branch | `git checkout -b <type>/<kebab-description>` (e.g. `feat/render-lod`, `fix/ci-crates-io-flake`) |
| Local gate before push | `cargo nextest run -p <touched-crate>` and, if any `Cargo.toml` changed, `./scripts/check-boundaries.sh` |
| Compliance grep | See "Banned-name compliance" below |
| Push | `git push -u origin <branch>` (plain push works; `origin` is HTTPS) |
| Verify remote head | `gh pr view <N> --json headRefOid --jq .headRefOid` vs `git rev-parse HEAD` |
| Gate poll | `gh pr view <N> --json statusCheckRollup` until every check completes and `CI Pass` is SUCCESS |
| Check for findings | `gh api repos/esaueng/remus/pulls/<N>/comments` and `gh pr view <N> --comments` (expect none; see gate section) |
| Merge | `gh pr merge <N>` (only when the merge itself is authorized): adds the PR to the merge queue, which squash-merges it once `checks / CI Pass` passes on the merge group |
| Behind `main` | Not a merge blocker: the queue tests each entry against current `main`. To refresh a stale PR's own CI anyway: `gh api -X PUT repos/esaueng/remus/pulls/<N>/update-branch -f expected_head_sha=<head>` (older `gh` lacks `pr update-branch`); restarts CI |
| Post-merge | `git fetch origin main`; GitHub deletes the head branch (`delete_branch_on_merge` is on), so retarget stacked PRs first |
| Worktree | `git worktree add .worktrees/<branch-name> <branch>` |

## Hooks: what actually runs

Hooks live in `.husky/`. Read the hook files themselves when in doubt; the "Git Conventions" section of CLAUDE.md describes an older pre-push behavior and the hook file is authoritative.

- `pre-commit`: fmt, clippy, taplo, and cargo-machete run in parallel. No tests. Expect `✅ Pre-commit checks passed.` If it fails, fix and re-commit. Caveat: the hook silently skips taplo and cargo-machete when the binaries are not installed (`command -v` guards in `.husky/pre-commit`), so a passing hook does not prove TOML formatting or unused-dep cleanliness. Install both with `cargo install taplo-cli cargo-machete`.
- `commit-msg`: runs commitlint (`@commitlint/config-conventional`, `commitlint.config.js`) but always exits 0: its not-installed fallback also swallows real lint failures, so violations print `✖` lines without blocking the commit. Treat any `✖` output as a hard failure and `git commit --amend` the message. Shape: `type(scope): subject`, e.g. `feat(render): screen-space adaptive LOD`. Nothing in CI lints messages either, so the hook's `✖` output is the only signal you get — the conventional-commit history is a repo contract (nothing mechanical consumes the titles today; there is no release-please in this fork, see "Release flow").
- `pre-push`: prints one info line and exits 0. Validation is deliberately delegated to CI (`.github/workflows/ci.yml`). Do not re-add local test runs to this hook, and do not treat its emptiness as a reason to skip local testing: run touched-crate tests yourself before pushing.

Hard rules:
- Never `--no-verify`, never `HUSKY=0`, never edit a hook to get past it. If a hook fails on pre-existing breakage you did not cause, stop and report; do not bypass.
- CI is the real signal: `ci.yml` calls the pinned `.github/workflows/fleet-ci-v4.yml`, whose jobs (Classify Changes, Repository Policy, Secrets Scan, Test, Approximation Census, Coverage, MSRV, WASM without optional I/O, WASM Build & Validate, WASM Size Gate, Fuzz Targets Compile, Software Rendering, Cargo Deny, Security Audit, Documentation) all fan into one aggregate job, `CI Pass`. Branch protection requires it as `checks / CI Pass`, and it is the only required status check. Repository Policy runs the cheap repository checks in one job, including Apache lineage, `cargo fmt --check`, layer boundaries, documentation paths, `cargo machete` and `taplo fmt --check`. The size policy is enforced inside `CI Pass` by `WASM Size Gate` (a kernel growth above 64 KiB raw needs a `WASM-Size:` line in the PR description); the `WASM Size Report` comment runs outside it. Job catalog: see [reference.md](reference.md), "CI jobs".

## Procedure: land a change

1. **Branch** off `main` as `<type>/<kebab-description>`. Never commit to `main` directly.
2. **Develop and test locally.** `cargo nextest run -p <crate>` for touched crates; `./scripts/check-boundaries.sh` if you touched any `Cargo.toml` (this check only runs in CI, catch it early).
3. **Commit.** Conventional message. Never commit plan/spec working documents: if `git status` shows untracked planning docs (ad-hoc `*-plan.md` or `*-spec.md` working documents), leave them untracked.
4. **Compliance grep** (below). Expect zero output.
5. **Push.** `git push -u origin <branch>` works normally: `origin` is `https://github.com/esaueng/remus.git`, credentials come from the osxkeychain helper, and there is no `insteadOf` rewrite in this checkout. Checkpoint: `gh pr view <N> --json headRefOid --jq .headRefOid` matches `git rev-parse HEAD`. (If you ever push via an explicit token URL instead, local `origin/<branch>` refs do not update — verify against the remote, not the tracking ref.)
6. **Create the PR** with `gh pr create`. Never `--draft`; if one slips through, `gh pr ready <N>`.
7. **Merge gate** (next section).
8. **After merge:** `git fetch origin main`. GitHub deletes the remote head branch automatically (`delete_branch_on_merge` is on), so the stacked-PR check below must run BEFORE you merge, not after. **Before merging, check whether another PR is based on this branch:**

   ```bash
   gh pr list --repo esaueng/remus --state open --json number,baseRefName \
     --jq '.[] | select(.baseRefName == "<branch>") | .number'
   ```

   Deleting the base of an open PR makes GitHub auto-close it, and that is not reversible: a closed PR's base cannot be retargeted and it cannot be reopened while its base is missing (`Cannot change the base branch of a closed pull request`). Recovery means rebasing onto `main` and opening a NEW PR, losing the old thread. Retarget the stacked PR to `main` first, then merge the parent. If `main` is checked out in another worktree, fetch rather than trying to switch this one.

## The merge gate

No human reviews PRs here; the only reviewer is the Codex bot's inline threads. Branch protection enforces CI and conversation resolution only. Verified state, not assumption:

- `gh api repos/esaueng/remus/branches/main/protection` (2026-10-01): required status check `checks / CI Pass`, `required_conversation_resolution: true` (every review thread resolved), required approvals none, `enforce_admins` false. At that date `strict` was `true`; the queue activation in `docs/owner-pr-ci.md` sets it to `false`, because the queue validates against current `main` instead.
- The merge queue is live on `main` (2026-10-20: queued PRs run `merge_group` CI on `gh-readonly-queue/main/pr-<N>-<base sha>` branches, and #1202, #1200 and the refresh PR #1208 landed on `main` from merge-group runs). Its template is `.github/merge-queue-ruleset.json`: squash, `ALLGREEN`, at most four entries built at a time, a 240-minute check timeout. Merge groups always run on hosted runners. Which jobs a queue entry runs is set by the classifier tiers in `docs/owner-pr-ci.md`, "Tiers".
- The only check-run apps that post here are `github-actions` and `blacksmith-sh` (the CI runner provider — its `[code]smith` entry reports SKIPPED and is not a reviewer). The `chatgpt-codex-connector` app posts an issue comment on each PR (often a usage-limit notice) and, on many PRs, inline review threads tagged P1 or P2 (2026-10-21: twelve open PRs held only by unanswered bot threads).
- PRs have no human reviews.

So a red `CI Pass` or an unresolved review thread blocks the merge, but an unread diff or a red check outside its fan-in does not. Admins can bypass even the CI requirement. The gate is what you do, in this order:

1. **Wait for every check to complete.** List the rollup unfiltered; do not filter by a name you have not confirmed exists:
   ```bash
   gh pr view <N> --json statusCheckRollup \
     --jq '.statusCheckRollup[] | "\(.name // .context)\t\(.status)\t\(.conclusion // "-")"'
   ```
   `CI Pass` does not appear at all until every job in its fan-in finishes, so its absence early on is not a failure. Coverage is the usual straggler (~40 min); budget for it.
2. **Confirm `CI Pass` is SUCCESS**, and separately read the checks outside its fan-in, for example `WASM Size Report`, `public-commit-metadata` (`ci.yml`) and the `WASM version guard` jobs (`wasm-version.yml`).
3. **Answer every review thread.** Read both comment surfaces: `gh api repos/esaueng/remus/pulls/<N>/comments` (inline) and `gh pr view <N> --comments` (issue-level; the WASM size bot posts here). For each unresolved bot thread, trace the finding from a real caller: fix a real one (with a regression test) and reply naming the commit, or reply with the traced reason it does not apply; then resolve it. The PR's own session owns its bot threads; ask before resolving a human reviewer's thread. Where GraphQL is unavailable (Claude Code cloud sessions), list threads with `gh api repos/esaueng/remus/pulls/<N>/ccr/review_threads` and resolve with `gh api -X POST repos/esaueng/remus/pulls/<N>/ccr/comments/<comment_id>/resolve`.
4. **Confirm the head you tested is the head being merged**: `gh pr view <N> --json headRefOid` vs `git rev-parse HEAD`.
5. **Merge:** `gh pr merge <N>` adds the PR to the merge queue. The queue squash-merges it (the repository allows squash merging only, and the queue's method is `SQUASH`) once its merge-group run passes, so `main` keeps one conventional-titled commit per PR. A PR that leaves the queue without merging failed that run or lost a required check: read the `merge_group` run for `gh-readonly-queue/main/pr-<N>-*`, fix, and enqueue again. Do not pass `--admin`: it bypasses the queue and CI, which needs explicit authorization for that PR. If the PR is `BLOCKED` with every check green, look for an unresolved review thread (`gh api graphql` on `pullRequest.reviewThreads { isResolved }`, or the `ccr/review_threads` route above) and handle it as in step 3.

Because no second pair of eyes exists, the diff you push is the diff that lands. On high-risk changes (GFA boolean engine, public WASM API), self-review the full diff before merging and say plainly in the PR body what you verified and what you did not.

Anti-patterns:
- Do NOT poll for `cubic · AI code reviewer`, Greptile, or Copilot. None are installed. A poll keyed to a check name that does not exist stays silent forever and reads exactly like "no findings" — this is the single most dangerous failure mode in this workflow, because the false pass is indistinguishable from a real one.
- Do NOT report "review came back clean" when the truth is "no reviewer ran". Say which gate actually passed.
- Do NOT enqueue (`gh pr merge`, with or without `--auto`) unless the merge itself was authorized. The queue lands the PR without another look once `CI Pass` is green on the merge group. The owner standing-authorizes a session to arm auto-merge (`gh pr merge <N> --auto`, or `gh api -X PUT repos/esaueng/remus/pulls/<N>/ccr/auto_merge -f merge_method=squash` where GraphQL is unavailable) on a PR it opened once that PR is done: CI green on the exact head, no conflict, every thread answered and resolved, and no open PR based on its branch. Arm it as the session's last action, since a later push lands once green. Instead of arming, add the `hold` label when the PR needs an owner decision.
- Do NOT conclude "no findings" from an empty `gh pr view --comments`; inline comments live on the pulls comments API. Check both.
- Do NOT treat `mergeStateStatus: CLEAN` as a quality signal. With no reviewer, it means only that the branch is conflict-free and has a green `CI Pass`.

## Banned-name compliance

The names of the reference kernel (the incumbent C++ CAD kernel) must not appear in changed files, commit messages, or PR titles/bodies. Run before pushing:

```bash
# pattern split so this file itself stays clean
banned='oc''ct|open''cascade'
git diff main... --name-only | xargs -r rg -n -i "$banned"
git log main.. --format='%s%n%b' | rg -n -i "$banned"
```

Pass condition: no output (rg exits 1). Grandfathered files that legitimately contain the names: `README.md`, `CHANGELOG.md`, `crates/wasm/CHANGELOG.md`, `scripts/bench-compare.sh`, `scripts/bench-report.ts`, `scripts/parity-loop.sh`. Do not add new occurrences anywhere, and do not "clean up" the grandfathered ones. Reading the reference kernel's source locally to study an approach is fine; naming it in committed text is not. For benchmark instructions, point at the brepjs harness scripts by path instead (see the parity-benchmarking skill).

## Worktrees

Parallel work lives inside the repo under `.worktrees/` (gitignored):

```bash
git worktree add .worktrees/<branch-name> <branch>
```

Ignore the older `../feat-branch` sibling-directory form in CLAUDE.md; in-repo `.worktrees/` is the rule. Each worktree pushes and PRs independently with the same procedure above.

## Release flow

**There is no release-please and no release PR in this fork** (verified 2026-08-21: `rg -l 'release-please' .github/workflows/` is empty; the leftover `release-please-config.json` / `.release-please-manifest.json` in the tree are inert upstream residue — the release-flow skill says the same). Do not wait for, look for, or promise a `chore(main): release X.Y.Z` PR.

The actual release channel is the committed package: `.github/workflows/publish.yml` ("Refresh Apache Staging Package") runs on every package-affecting push to `main`, rebuilds `crates/wasm/pkg` and `crates/wasm-io/pkg` with `cargo xtask wasm-build`, and — when the output differs — runs `scripts/publish-wasm-package-pr.py`, which opens a refresh PR `chore(wasm): refresh committed packages to vX.Y.Z` from branch `codex/wasm-refresh-<source sha>` through the package bot App (`remus-package-bot`). That PR runs no heavy CI tier (the required "Committed packages match source" job rebuilds and compares both packages and applies the size gate) and goes through the merge queue like any other; it never merges on its own. Consumers install by git path (`github:esaueng/remus#main&path:/crates/wasm/pkg`), so merging the refresh PR IS the release: a kernel PR reaches consumers only once the next refresh PR lands. After opening its PR, the script closes the older open refresh PRs that this refresh supersedes (esaueng/remus#1204). Nothing publishes to npm or crates.io. Cross-repo consumption and the validation harness: see the release-flow skill. Details and the manual refresh dispatch: [reference.md](reference.md), "Committed-package refresh".

## CI failures you did not cause

`Cargo.lock` is committed (tracked, not in `.gitignore` — dependabot updates it), and every job — including `audit` — builds and audits the committed resolution. The one remaining zero-diff failure source: `deny` and `audit` fetch the advisory database live, so a newly published advisory against a pinned version fails an unrelated PR. A new dep *release* alone cannot. Never widen `deny.toml` to get green. Triage order, the MSRV and wasm-bindgen pins, and scheduled workflows: see [reference.md](reference.md), "CI failures you did not cause".

## Symptoms

Symptom-to-cause table (push hangs, commitlint rejects, review check missing, package refresh not landing): see [reference.md](reference.md), "Symptom table".
