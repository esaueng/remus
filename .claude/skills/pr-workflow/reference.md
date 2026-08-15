# Remus PR workflow reference

## Live settings checks

Repository settings can drift. Reverify them before a governance-sensitive
change:

```bash
gh api repos/esaueng/remus
gh api repos/esaueng/remus/branches/main/protection
gh api repos/esaueng/remus/rules/branches/main
gh api repos/esaueng/remus/rulesets
gh api repos/esaueng/remus/actions/permissions
gh api repos/esaueng/remus/actions/permissions/workflow
```

Observed on 2026-08-15:

- public, standalone repository; default branch `main`
- no branch protection and no rulesets
- merge commits, squash merges, and rebase merges enabled
- auto-merge disabled; branch deletion after merge disabled
- Actions enabled for all actions; default workflow permission is read-only
- workflows cannot approve pull requests
- no environments, repository variables, or repository secrets

Project policy remains stricter than these settings: topic branch, normal PR,
no direct `main` commit, no force-push, and no self-merge.

## CI catalog

`.github/workflows/ci.yml` fans the gated jobs into `CI Pass`:

- Apache Lineage
- Format
- Clippy
- Test
- Test (macOS) and Test (Windows)
- Coverage
- MSRV (1.88)
- WASM Build & Validate
- Fuzz Targets Compile
- Software Rendering
- Layer Boundaries
- Doc Paths
- Cargo Deny
- Security Audit
- Documentation
- Unused Dependencies
- Secrets Scan
- TOML Format

`WASM Size Report` is informational. The separate OSV workflow is report-only
on pull requests and blocking on `main`. Benchmark and mutation workflows are
not part of `CI Pass`.

List checks before filtering so an absent name is not mistaken for success:

```bash
gh pr view <number> --repo esaueng/remus --json statusCheckRollup
gh pr checks <number> --repo esaueng/remus
gh api repos/esaueng/remus/pulls/<number>/comments
gh pr view <number> --repo esaueng/remus --comments
```

No automated reviewer is assumed. Read whichever review surfaces are actually
present and verify they correspond to the current PR head.

## Hooks and pushes

- `.husky/pre-commit`: fmt and Clippy in parallel, plus Taplo and
  cargo-machete when installed.
- `.husky/commit-msg`: commitlint when the npm development dependencies are
  installed; verify conventional syntax yourself if it is skipped.
- `.husky/pre-push`: informational only; CI runs the full matrix.
- `origin` uses HTTPS. Push only the named topic branch and verify its exact
  remote SHA. Never push tags as part of ordinary PR delivery.

## Release boundary

The repository contains inherited release automation, but the existence of a
workflow does not authorize a registry publish, tag, GitHub release, or
settings/credential change. Treat those as separately approved work.
