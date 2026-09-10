# Trusted owner CI fleet

Required Linux CI jobs and their completion gate live in `fleet-ci.yml`, called
at an immutable commit from `ci.yml`. Benchmarks, corpus gauntlets, fuzzing,
mutation checks, OSV scans, WASM candidates and size reports each have their
own immutable `fleet-*.yml` callee. Their shared pre-checkout guard is a pinned
composite action; build jobs and result publishers retain separate jobs and
permission scopes. GitHub evaluates the runner expression before
scheduling each job. There is no GitHub-hosted authorization or routing job in
front of the Linux checks, and no routing HTTP request from Actions.

Only the approved owner's open, same-repository PR merge refs are eligible.
Main pushes and manual dispatches also require protected main. Scheduled
auxiliary jobs additionally require protected main and the approved actor.
Repository name and numeric identity, PR author, event actor and rerun actor are checked in the
immutable workflow. Forks and other authors remain hosted. The runner group
must separately restrict the exact repository and immutable workflow revision;
a label or repository variable does not grant execution authority.

`CI_FLEET_ENABLED=true` opts in. `CI_FLEET_TARGET` accepts only
`ci-server-jane`, `ci-server-john`, or `github-hosted`. An external controller
can publish this variable after checking fresh collector health and GitHub runner connectivity. A manually selected target stays fixed
until an administrator or controller updates it; GitHub does not automatically
switch hosts when that server goes offline. The preferred order is Jane, then
John, then GitHub-hosted. Busy Jane remains available; jobs queue for either of her two slots.
Missing/invalid configuration selects hosted. Hosted fallback still depends on
GitHub billing and capacity.

## Lightweight job pool

`CI_FLEET_LIGHT_POOL_ENABLED=true` lets Classify Changes, Repository Policy,
Secrets Scan, Documentation, Cargo Deny, Security Audit and CI Pass
use any free runner carrying `ci-remus-light` in `ci-trusted-main`. Provision
that label only on `ci-vm-1441561` (John), `ci-server-jane-1` and
`ci-server-jane-2`; keep their existing labels. Do not label the retired
`ci-server-jane` registration or any additional runner. No new runner process
or extra machine capacity is created.

The pool is opt-in. Missing/false pool configuration preserves the selected-host
routing above. All existing owner, repository, event and protected-ref checks
apply before pool selection. Disabled fleet routing, `github-hosted`, and an
unknown/missing fleet target still select GitHub-hosted runners. Heavy jobs
continue using the existing selected-host expression; their commands, CPU and
memory budgets, test selection and coverage threshold are unchanged. A short
job on John uses its existing one-build-worker/one-test-thread profile.

Pool membership, rather than the preferred-host variable, controls which
machines can accept pooled jobs. Before draining or disabling one machine,
remove `ci-remus-light` from its runners (or disable the pool entirely) as well
as updating host routing. The preferred-host controller does not manage pool
membership. Removing a label prevents new matching assignments; allow active
jobs to finish. GitHub chooses any available matching runner, without a Jane
preference inside the pool.

Activation order: add the label to the three verified runner IDs in the
restricted group; append the exact new callee SHA to the existing workflow
allowlist; enable the pool variable; then validate the PR's immutable caller.
Preserve every existing runner-group permission and workflow entry. Confirm
light jobs execute on both machines and heavy jobs retain host selection before
merging. Roll back subsequent scheduling with
`CI_FLEET_LIGHT_POOL_ENABLED=false`; already assigned jobs keep their runner.

Before checkout, self-hosted jobs verify the non-root identity, protected runner
files, NoNewPrivileges, fresh storage, empty rootless Docker state, mount options
and exact cgroup limits. Jane slots each have six CPU equivalents and 6 GiB RAM;
John has one CPU equivalent and 3 GiB. Guard failure fails the job.
Rust build and registry caches are restored from GitHub after the isolation
check; only main saves them. Runner homes and workspaces are still erased
between jobs. This reuses build artifacts without retaining a local workspace.
Browser suites hold a shared host lock while using fixed localhost test ports.
Browser/native OS dependencies must be installed by the host administrator;
workflow jobs install browser binaries without sudo.

## Activation and rollback

Approve the exact immutable `fleet-ci.yml` runner-group entry without removing
existing restrictions. Configure the external controller's selected repository
identities and dedicated GitHub credential, then validate both hosts. The
required completion check is `checks / CI Pass`, the existing gate inside the
reusable suite. It validates every selected job, including macOS, and runs on
the selected fleet for eligible events. Review requirements stay unchanged.

Run an authorized real application job on each Jane slot, verify cleanup and
unavailable-only selection, then enable normal routing. Code merge alone does
not activate the runner group, repository variables, or controller.

For validation before broad activation, leave `CI_FLEET_ENABLED=false` and set
`CI_FLEET_VALIDATION_REF` to the exact `refs/pull/<number>/merge` being tested.
This selects only that ref and still requires every owner and repository
identity check. Remove the validation variable when enabling normal routing.

Already queued jobs keep their assignment. A host failure after selection needs
a fresh run; this change does not automatically cancel or retry queued jobs.
Controller failure preserves the last target and is shown as stale in the
monitor. A failing build is not an availability outage.

`CI_FLEET_ENABLED=false` returns subsequent jobs to hosted runners. Let active
jobs finish and rerun stranded jobs. Revert workflow pins through a PR. Production
credentials and deployment jobs stay outside the home runners; existing
main-merge deployment integrations still require their normal authorization.

Remus additionally revalidates live PR identity and the event merge commit's two
parents before self-hosted checkout. An API error or stale PR fails the job;
no contributor code runs first. The event sender is also checked here.

Native Rust, repository policy, coverage, WASM, rendering, security and
documentation jobs retain their individual commands and dependency gates.
Cargo Deny uses the same 0.20.2 CLI and all-feature checks directly. OSV uses
the same 2.5.1 scanner/reporter image, pinned by digest and started with
`docker run` after the guard. Docker actions would prepare images before the
empty-Docker check and fail it on an otherwise clean runner.
The macOS platform job remains hosted and required by `checks / CI Pass`.
Jane and John cannot replace macOS capacity. Release workflows are unchanged.

Policy verification: `python3 scripts/test-direct-fleet.py`,
`python3 scripts/test-owner-pr-routing.py`, the existing owner policy tests,
and `actionlint`. The historical `owner-pr.yml` remains unchanged for old
immutable callers; new callers use `fleet-ci.yml`.

## Convergence and merge queues

The caller cancels superseded runs only for the same ref. There is no global
suite concurrency group: separate PR suites can make progress at the same time,
and one suite's slow coverage job cannot prevent another suite from starting.
Jane's two registered slots bound actual server execution to two jobs across
all workflows and repositories. Jobs wait for a free slot without oversubscribing
the host. Two coverage jobs may run together when both slots are available.

The advisory WASM size report runs in its own immutable callee after the suite,
so a reporting failure cannot fail the required completion gate. There is no
additional hosted completion wrapper. Existing PR branches must pick up the
new caller before their runs stop using the old admission queue.

### Required-check migration

Removing the hosted wrapper changes the required check context from `CI Pass`
to `checks / CI Pass`. Before merging this routing PR, obtain maintainer approval
and verify its exact head has a successful `checks / CI Pass` from GitHub Actions
(app ID 15368). Replace only that context in the existing required-status-check
configuration, preserving strict mode, app identity and every review requirement.
Then perform the exact-head guarded merge. This is a check-name migration, not
a bypass of a failed or pending test. Older PR branches must pick up the new
caller before they can satisfy the migrated requirement.

The runner-group allowlist must include each exact callee revision in the
callers. Preserve every existing allowed repository and workflow. Each callee
pins the shared guard action separately; updating its content requires a new
commit and corresponding pin updates. Missing configuration still uses hosted
runners. Untrusted PRs and merge groups remain hosted.

Repository Policy and Secrets Scan run before any expensive checks. CI Pass
requires both to succeed, requires the classifier outputs to be valid, and
rejects unexpectedly skipped selected checks. Documentation-only selection
still skips the heavy suite.

### Tiers

The classifier (`scripts/classify-ci-changes.py`) selects jobs in tiers so a
kernel PR waits only for the checks that can fail on its diff:

- **Tier 1** (`heavy`, every source change): Test (clippy, nextest, doc
  tests, complexity guards in one job), Approximation Census, WASM without
  optional I/O. Roughly 12 minutes of runner time.
- **Tier 2** (`full`): Coverage, macOS, MSRV, Fuzz Targets Compile,
  Software Rendering, Cargo Deny, Security Audit. Selected for every main
  push, merge group and dispatch, and for a PR only while it carries the
  `ci:full` label (label it, then re-run or push).
- **Package build** (`wasm`): WASM Build & Validate and the advisory size
  report. Selected with tier 2, and on any PR whose diff touches
  `crates/wasm*`, `xtask`, `tools/vs-bench`, the WASM scripts, `Cargo.lock`,
  `Cargo.toml`, or `rust-toolchain.toml`. A kernel-only PR does not rebuild
  the distributable packages; the main push and the publisher's refresh PR
  cover that.
- **Package refresh**: a diff confined to `crates/wasm/pkg` and
  `crates/wasm-io/pkg` runs the full suite. Paths alone do not prove that the
  committed bytes came from a trusted build; no package-only validation bypass
  is enabled.

CI Pass accepts a skipped job only when its tier flag is false, and rejects
a tier-2 or package selection without a heavy selection. Every main push
runs the full suite, and the caller cancels superseded runs only for
`pull_request` events, so each merge commit keeps a completed verdict. Optimized coverage uses the existing `ci-test`
profile for running and reporting, with coverage artifacts cleaned first; it retains the entire workspace,
test assertions and the 60% line threshold. WASM optional-I/O clippy and native
tests run as a separate required job. Package validation, optimization, tarball
consumers and W9 preflight remain in the main WASM job.

PR benchmarks run only with the `ci:benchmark` label. Adding the label starts
a run; subsequent pushes keep benchmarking while the label remains. Main
baseline tracking and manual workflow dispatch remain available. Unrelated
label events cannot cancel an active benchmark. Deterministic complexity
regression tests remain required for every full CI suite.

`merge_group: checks_requested` validates the actual merge-group SHA against
`merge_group.base_sha`, including changes from every PR in the group. Merge
groups always use hosted runners: the owner-only single-PR authorization does
not authorize a group containing other contributors' code.

`.github/merge-queue-ruleset.json` is an initially disabled, additive ruleset.
It limits speculative merge builds to two, requires every group's PR merge
commit to pass (`ALLGREEN`), and adds no bypass actor. It neither replaces
existing branch protection nor changes review requirements. Do not enable it
until the caller, pinned callee and passing exact-head CI are on main. Creating
or enabling the rule does not itself enqueue PRs; enqueuing a PR authorizes its
automatic merge and still requires the maintainer's merge approval.

After that approval and workflow merge, create the disabled rule with:

```bash
gh api --method POST repos/esaueng/remus/rulesets \
  --input .github/merge-queue-ruleset.json
```

Inspect the returned ID and existing rules before updating, to avoid duplicate
rules. Activate this exact rule, verify it is active on main, then change only
`strict` to `false` under the existing required-status-check protection. Keep
`checks / CI Pass`, its GitHub Actions app identity, and every review/protection setting.
This ordering leaves protection in place throughout and lets the merge queue
validate against current main without requiring manual branch updates. For
rollback, restore `strict: true` before disabling the queue rule.

Validate with the policy tests and actionlint. `scripts/test-ubuntu-ci-routing.py`
checks every additional callee against trusted, fork, actor-mismatch and
unprotected-main contexts, verifies immutable pins and guard ordering, and
retains OSV comparison and publisher permission contracts. Convergence tests
exercise full-suite completion, cancellation, unexpected skips and independent
PR scheduling.
