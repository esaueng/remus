# pr-workflow reference

Deep catalog behind SKILL.md. Everything here was verified against the repo; when it disagrees with prose elsewhere (including CLAUDE.md's "Git Conventions"), the workflow files (`.husky/*`, `.github/workflows/*.yml`) are authoritative.

## CI jobs (`.github/workflows/ci.yml`)

`ci-pass` (display name `CI Pass`) is an aggregate job whose `needs` list is, verbatim from `ci.yml`:

```
[fmt, clippy, test, platform-test, coverage, msrv, wasm, fuzz-check,
 render, boundaries, deny, audit, docs, machete, secrets-scan, taplo]
```

It asserts every one of those resolved to `success` or `skipped`. It is NOT required by branch protection — `main` has no protection at all (see "Repo merge settings"), so `CI Pass` is advisory unless you choose to honour it.

Three PR checks sit OUTSIDE the fan-in and are easy to miss: `apache-lineage`, `doc-paths`, and `wasm-size`. Read them yourself.

| Job | Display name | In `CI Pass`? | What it runs |
|-----|--------------|---------------|--------------|
| `apache-lineage` | Apache Lineage | no | provenance/lineage guard |
| `fmt` | Format | yes | `cargo fmt --all -- --check` |
| `clippy` | Clippy | yes | `cargo clippy --all-targets --all-features -- -D warnings` |
| `test` | Test | yes | `cargo nextest run --workspace`, doc tests, complexity guards |
| `platform-test` | Test (macos-latest), Test (windows-latest) | yes | matrix build/test on non-Linux hosts |
| `coverage` | Coverage | yes | llvm-cov coverage report; usually the slowest job (~40 min) |
| `msrv` | MSRV (1.88) | yes | build on the minimum supported Rust |
| `wasm` | WASM Build & Validate | yes | `cargo xtask wasm-build --skip-opt` |
| `fuzz-check` | Fuzz Targets Compile | yes | fuzz targets still compile |
| `render` | Software Rendering | yes | headless render smoke test |
| `boundaries` | Layer Boundaries | yes | `./scripts/check-boundaries.sh` |
| `deny` | Cargo Deny | yes | license/advisory/ban checks |
| `audit` | Security Audit | yes | `cargo audit` |
| `docs` | Documentation | yes | doc build with warnings denied |
| `machete` | Unused Dependencies | yes | `cargo machete` |
| `secrets-scan` | Secrets Scan | yes | committed-secret scan |
| `taplo` | TOML Format | yes | `taplo fmt --check` |
| `doc-paths` | Doc Paths | **no** | `./scripts/check-doc-paths.sh` |
| `wasm-size` | WASM Size Report | **no** | PR-only size delta comment (informational) |

`benchmark.yml` contributes `Boolean perf` and `Publish benchmark` on PRs; neither is in `CI Pass`. `blacksmith-sh` posts a `[code]smith` entry that reports SKIPPED — it is the CI runner provider, not a reviewer or a quality check.

The `wasm-size` comment's first column is *labelled* `main`, but that label is a hardcoded string in `ci.yml`. The job actually checks out `ref: ${{ github.base_ref }}` — your PR's real base. The two coincide today only because PRs target `main`; the header would still read `main` for a PR based on anything else, so never read it as evidence of what was compared. Because the baseline IS your base, a nonzero delta on a change that cannot reach the binary (a `#[cfg(test)]`-only or docs-only diff) is NOT explained away by a stale baseline and deserves a second look. `Cargo.lock` is gitignored, so the PR and base builds resolve dependencies independently — that is the likeliest remaining cause, but confirm it rather than assuming it.

Local pre-commit covers only fmt, clippy, taplo, machete, and the last two only when the binaries are installed (the hook skips them silently otherwise). Everything else (tests, boundaries, deny, docs) first runs in CI unless you run it yourself. Before pushing, run at minimum the tests for touched crates and, on any `Cargo.toml` change, `./scripts/check-boundaries.sh`.

## Repo merge settings (verified via `gh api repos/esaueng/remus`)

- `allow_squash_merge: true`, but `allow_merge_commit: true` and `allow_rebase_merge: true` as well — squash is NOT the only option, so always pass `--squash` explicitly.
- `allow_auto_merge: **false**` — `gh pr merge --auto` is unavailable. Merge once checks are green.
- `delete_branch_on_merge: **false**` — the remote branch survives the merge. Delete it yourself: `git push origin --delete <branch>`.
- Branch protection on `main`: **none**. `gh api repos/esaueng/remus/branches/main/protection` returns 404 "Branch not protected". No required checks, no required approvals, force pushes and direct pushes to `main` are not blocked by GitHub. Not pushing to `main` is a rule you follow, not one the server enforces.
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

## Release-please (`.github/workflows/publish.yml`)

- Runs on every push to `main` using `googleapis/release-please-action` v5 with a bot app token.
- Config: `release-please-config.json`; current version manifest: `.release-please-manifest.json`. Single package rooted at `.`, component `remus-wasm`; the version is also bumped in `crates/wasm/Cargo.toml`.
- Flow: merging a `feat`/`fix`/`perf` PR creates or updates the pending release PR (`chore(main): release X.Y.Z`, head branch `release-please--branches--main--components--remus-wasm`). Merging that release PR creates the tag and GitHub release and publishes to npm.
- Version-neutral changes: `docs` and `chore` commits are changelog-hidden; changes only under excluded paths (`.github`, `book`, `scripts`, `benches`, `bench-results`, `examples`, `bindings`) do not bump.
- Manual escape hatch: `workflow_dispatch` on the Publish workflow with a `publish_version` input skips release-please.
- Cross-repo: brepjs (`andymai/brepjs`) consumes the published wasm package; see the release-flow skill for the two-repo runbook. There is no brepjs checkout on this machine — clone it when you need hop 4.

## CI failures you did not cause

Supply-chain and toolchain jobs can fail on a PR that never touched dependencies. Root cause: `Cargo.lock` is gitignored (see `.gitignore`), so every CI run resolves dependencies fresh. The `audit` job even runs `cargo generate-lockfile` explicitly. A new advisory or a new dep release changes the verdict with zero diff on your branch.

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
| PR shows mergeable with nothing green yet | `main` has no branch protection, so mergeability says nothing about checks | Wait for `CI Pass`; mergeable is not a quality signal here |
| A review check never appears, however long you poll | No reviewer app is installed on this repo; the check does not exist | Stop waiting. Self-review the diff and report that no reviewer ran |
| `gh pr merge --auto` errors or is refused | `allow_auto_merge` is false repo-wide | Wait for checks, then plain `gh pr merge <N> --squash` |
| Merged PR's branch still on the remote | `delete_branch_on_merge` is false | `git push origin --delete <branch>`, but check for stacked PRs first (next row) |
| A stacked PR went CLOSED on its own | Its base branch was deleted when the parent merged; GitHub auto-closes in that case | Not reversible — base cannot be retargeted while closed, and it cannot reopen with a missing base. Rebase onto `main` and open a new PR. Avoid by retargeting the child to `main` BEFORE deleting the parent branch |
| `CI Pass` missing from the rollup while jobs still run | It only appears once every job in its `needs` list finishes | Not a failure; keep polling. Coverage is the usual straggler |
| `WASM Size Report` shows a delta on a test-only diff | NOT a stale baseline — the job builds `github.base_ref`, your real base; the `main` column label is hardcoded. Independent dependency resolution between the two builds (`Cargo.lock` is gitignored) is the likeliest cause | Confirm your diff cannot reach the binary, then report the delta as unexplained rather than calling it expected drift |
| CI `boundaries` job fails | A crate dependency violates the layer rules | Run `./scripts/check-boundaries.sh` locally; see the layer-boundaries skill |
| CI `taplo` or `machete` fails but pre-commit passed | Tool not installed locally; the hook skips it silently | `cargo install taplo-cli cargo-machete`, fix, re-commit |
| Compliance grep hits in a file you touched | You introduced a banned reference-kernel name, or you touched a grandfathered file | Remove new occurrences; leave grandfathered ones as-is |
| Release PR did not update after merge | Commit type was `docs`/`chore`, or all changes fell under excluded paths | Expected; only `feat`/`fix`/`perf` in versioned paths bump |
| `deny` or `audit` fails on a PR that never touched deps | `Cargo.lock` is gitignored; CI resolved a newly-advisoried or newly-released dep | Follow the triage order in "CI failures you did not cause"; never blanket-ignore |
| MSRV job fails with syntax or feature errors inside a dependency | A dep released a version requiring Rust newer than 1.88 | Constrain that dep in `Cargo.toml`; do not bump `rust-version` |
| `cargo xtask wasm-build` bails with a wasm-bindgen-cli version mismatch | Local CLI differs from the pin; the crate pin and `xtask/src/wasm.rs` constant must match | Install the pinned CLI version; bump the pin only as its own PR |
| You pushed straight to `main` and it succeeded | There is no branch protection to stop you | Nothing rejects this — do not rely on the server. Branch and open a PR; if you already pushed, say so immediately |

## Anti-patterns (what NOT to conclude)

- "The review came back clean": no reviewer runs on this repo. An empty findings list means nobody looked, which is not the same as approval. Report which gate actually passed.
- "The check I polled never reported problems, so there were none": a filter keyed to a nonexistent check name is silent forever and looks identical to a pass. List the rollup unfiltered before trusting any filter.
- "The pre-push hook printed one line and passed, so the change is validated": the hook intentionally runs nothing. Validation is CI plus the local tests you ran yourself.
- "CLAUDE.md says pre-push runs tests and cargo-deny": stale. The hook file delegates to CI; do not re-add local suites to it and do not cite the stale description.
- "High-risk change, better wait for a human": no gate of any kind will stop you, which is an argument for more self-review, not less.
- "gh pr view showed no comments, so there are no findings": inline findings live on `pulls/<N>/comments` (the API), check both surfaces.
- "Branch protection will catch it": there is no branch protection on `main`. Nothing will catch it.
- "The plan doc helps reviewers, commit it": working plans and specs never get committed.
- "The commit went through, so the message passed commitlint": the commit-msg hook never blocks. Check the hook output for `✖` lines and amend if any appeared.
