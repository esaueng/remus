---
name: pr-workflow
description: Use when branching, committing, pushing, opening, reviewing, or merging a pull request in the standalone esaueng/remus repository.
---

# Remus PR workflow

Remus uses topic branches and ready-for-review pull requests against `main`.
Repository settings currently permit direct pushes and self-merges, but project
policy does not: never commit directly to `main`, never force-push, and never
merge your own PR.

## Procedure

1. Refresh `origin/main`, verify the intended base, and create a topic branch.
2. Make the smallest scoped change. Preserve `LICENSE-APACHE`, `NOTICE`, and
   the provenance ledger.
3. Run the relevant local gates from `AGENTS.md`. Any identity or lineage
   change also runs:

   ```bash
   ./scripts/check-apache-lineage.sh
   python3 scripts/check-apache-replay-provenance.py
   ./scripts/check-remus-rename.sh
   ```

4. Commit with a conventional message. Do not bypass hooks.
5. Push over the configured HTTPS remote and verify the remote head SHA.
6. Open a normal, ready-for-review PR to `main`; never create a draft and do
   not enable auto-merge.
7. List the status rollup without filtering once, then wait for `CI Pass` and
   inspect all review comments. Report OSV's PR scan as report-only and any
   absent or skipped checks honestly.
8. Address findings with follow-up commits. Leave merging to another person.

The pre-commit hook runs fmt and Clippy plus optional Taplo and cargo-machete.
The pre-push hook delegates the full suite to CI. A passing hook is therefore
not a substitute for task-specific tests.

## Current repository behavior

As verified on 2026-08-15, `main` has no branch protection or ruleset,
auto-merge is disabled, merge/rebase/squash methods are enabled, and merged
branches are not deleted automatically. These are observations, not policy;
recheck them before relying on repository-side enforcement. See
[reference.md](reference.md) for commands and the CI catalog.

Publishing packages, creating releases, or changing repository settings is a
separate operation requiring explicit authorization.
