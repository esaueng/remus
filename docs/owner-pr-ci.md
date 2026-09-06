# Owner PR CI on the VPS

Native Rust checks use the VPS only for same-repository PRs to `main` opened
by GitHub account `petergstfsn` (user ID `171875562`), with that account also
being the event actor, sender, and rerun actor. Forks, other authors, bots,
pushes, and other event types retain GitHub-hosted checks.

The migrated checks are Clippy, workspace tests, doc tests, complexity
guards, fuzz-target compilation, documentation/book builds, and Rust 1.88
compatibility. They run sequentially in one job to reuse the downloaded
toolchain and compiled dependencies before the host clears job storage.
No tests or thresholds are removed. Compilation and test concurrency are
limited to one; the combined job has a 90-minute timeout.

Repository policy, coverage, approximation census, WASM, software rendering,
security scans, benchmarks, and macOS tests remain hosted. Release workflows
are unchanged. This is a native Rust migration, not proof that the VPS can
replace every hosted workload.

## Access boundary

`.github/workflows/ci.yml` calls `owner-pr.yml` at an immutable commit SHA.
The organization runner group must authorize exactly that reusable workflow
and SHA. Never allow `owner-pr.yml@main`, wildcard workflow refs, or PR merge
refs. Only jobs defined inside the allowlisted callee can reach the VPS;
editing a caller's author check or `runs-on` does not grant runner access.

The hosted authorization job executes the policy embedded in that pinned
workflow before checking out code. It verifies repository and user IDs,
rejects forks, fetches the current PR through GitHub's API, and compares the
PR head/base with the event. It verifies the exact two parents of the
GitHub-generated merge commit. The VPS checks out only that event's SHA
with `persist-credentials: false`; callers cannot supply a source ref,
repository, shell command, or runner.

Authorization API failures fail the workflow. Ineligible or stale events
select hosted fallback. A failed or cancelled VPS job fails `CI Pass`;
hosted jobs skipped after successful VPS execution do not hide its failure.

The callee receives a read-only repository token and no inherited secrets,
deployment credentials, OIDC access, or cross-job Rust caches. Before source
execution it verifies the expected non-root runner, rootless Docker, fresh
storage sentinels, one CPU, 3 GiB CI memory, and no swap. The host owns cleanup
between listener sessions. These controls limit access and resources;
the persistent host still trusts code in the owner's eligible PRs and its
dependencies. This is not an isolation boundary for hostile code.

## Activation and rollback

Verify the live runner-group configuration before activation:

1. Repository access is `selected`, and includes only the intended repositories.
2. Workflow access is restricted. Preserve existing entries and add only
   `esaueng/remus/.github/workflows/owner-pr.yml@<SHA from ci.yml>`.
3. Remus (`1334765869`) is selected, and the single intended runner is online.
4. Set repository Actions variable `REMUS_OWNER_PR_VPS_ENABLED` to exactly
   `true`. Missing, false, or other values select hosted fallback.

The workflow and allowlist must use the same SHA. After changing the callee,
commit it, update the caller pin in a subsequent commit, test both, and
authorize only the new reviewed SHA. Do not rewrite history to manufacture
a self-referencing pin. `scripts/test-owner-pr-routing.py` checks the pinned
file equals the reviewed file in the checkout.

Set `REMUS_OWNER_PR_VPS_ENABLED=false` for hosted fallback on subsequent
runs. Cancel any existing queued/running VPS run separately if needed.
Removing repository access or the immutable allowlist entry prevents new
VPS jobs but can leave an already-selected job queued; disable the variable
before rerunning CI.

## Verification

```sh
python3 scripts/test-owner-pr-policy.py
python3 scripts/test-owner-pr-routing.py
python3 scripts/test-classify-ci-changes.py
actionlint .github/workflows/owner-pr.yml
git diff --check
```

Policy tests exercise the embedded authorization code, including forks,
other authors/actors, reruns, alternate events, stale head/base commits,
merge-parent substitution, and malformed identity/ref fields. Routing tests
check immutable content, hosted fallback, retained workloads, and failure
propagation through the actual `CI Pass` shell command.

Live validation must identify the assigned runner, full check result,
elapsed time, memory/OOM counters, disk use, and cleanup after completion.
A policy test or runner registration alone does not establish workload
capacity.
