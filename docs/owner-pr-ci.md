# Trusted owner CI fleet

Required Linux CI jobs and their completion gate live in `fleet-ci-v6.yml`, called
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
switch hosts when that server goes offline. Production keeps GitHub-hosted execution primary and selects idle Jane slots
under verified hosted queue pressure. John and the legacy local-first policy
remain available only in separately validated configurations.
Missing/invalid configuration selects hosted. Hosted fallback still depends on
GitHub billing and capacity.

## Lightweight job pool

`CI_FLEET_LIGHT_POOL_ENABLED=true` lets Classify Changes, Repository Policy,
Secrets Scan, Documentation, Cargo Deny, Security Audit and CI Pass
use any free runner carrying `ci-remus-light` in `ci-trusted-main`. Provision
that label only on explicitly selected, independently validated runners;
keep their existing labels. The three-slot migration does not enable this
optional pool or add `ci-remus-light` to slot 3. Keep the pool disabled until
its membership and workflow pins are separately validated. Do not label the
retired `ci-server-jane` registration. No new runner process or extra machine
capacity is created by adding a pool label.

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

Activation order: add the label only to the selected verified runner IDs in the
restricted group; append the exact new callee SHA to the existing workflow
allowlist; enable the pool variable; then validate the PR's immutable caller.
Preserve every existing runner-group permission and workflow entry. Confirm
light jobs execute on both machines and heavy jobs retain host selection before
merging. Roll back subsequent scheduling with
`CI_FLEET_LIGHT_POOL_ENABLED=false`; already assigned jobs keep their runner.

Before checkout, self-hosted jobs verify the non-root identity, protected runner
files, NoNewPrivileges, fresh storage, empty rootless Docker state, mount options
and exact cgroup limits. The active `fleet-ci-v6.yml` profile recognizes
`ci-server-jane-1`, `ci-server-jane-2` and `ci-server-jane-3`, each with six CPU
equivalents and a 5,000,000,000-byte RAM setting. systemd aligns the enforced
limit to whole host pages: 4,999,999,488 bytes on a 4 KiB-page host. Each slot
has a separate 150 GiB image. Historical v4 callers retain their legacy 6 GiB contract; keep the old shared
label absent so those callers cannot match migrated slots. The active v6
caller selects only the verified 5 GB profile.

John has one CPU equivalent and 3 GiB. Guard failure fails the job.
Rust build and registry caches are restored from GitHub after the isolation
check; only main saves them. Runner homes and workspaces are still erased
between jobs. This reuses build artifacts without retaining a local workspace.
Browser suites hold a shared host lock while using fixed localhost test ports.
Browser/native OS dependencies must be installed by the host administrator;
workflow jobs install browser binaries without sudo.

## Activation and rollback

Approve the exact immutable v6 and helper entries named in the combined
activation below without removing existing entries or restrictions. Configure the external controller's selected repository
identities and dedicated GitHub credential, then validate every selected slot. The
required completion check is `checks / CI Pass`, the existing gate inside the
reusable suite. It validates every selected job and runs on
the selected fleet for eligible events. Review requirements stay unchanged.

Run an authorized real application job on each Jane slot, verify cleanup and
hosted-first overflow selection, then enable normal routing. Code merge alone does
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
macOS CI tests are temporarily disabled while macOS app development is paused.
Release workflows are unchanged.

Policy verification: `python3 scripts/test-direct-fleet.py`,
`python3 scripts/test-owner-pr-routing.py`, `python3 scripts/test-owner-pr-policy.py`
(the owner identity policy in the fleet guard) and `actionlint`. The unused
`owner-pr.yml` and its availability selector `select-runner.yml` were retired on
2026-10-19; callers use the active `fleet-ci` callee named in
`.github/fleet-workflows.json`.

## Combined three-slot and queue activation

`ci.yml` activates `fleet-ci-v6.yml` at #1260's landed main commit. It combines
the owner-only queue authorization staged in v5
with Jane's three-slot, nominal 5 GB profile. Its queue policy and every
existing command, gate, timeout and permission match the reviewed v5 bytes;
only the runner-name table, memory value, page alignment and profile selector differ. The
canonical policy is embedded in the immutable callee before candidate checkout.
The required direct-fleet suite executes the combined queue policy, scheduling
and optional-probe failure-gate regressions.

The combined workflow selects `ci-server-jane-5gb` while the controller
preference remains `CI_FLEET_TARGET=ci-server-jane`. Deploy the reviewed
controller profile mapping before adding that label to verified slots and
approving the exact compatible main revisions. Keep the old shared
`ci-server-jane` label withheld, so historical 6 GiB callers remain quarantined.
The profile label is a selector, not a runner identity or permission grant.

The combined activation switches v6, the compatible auxiliary manifest and
every caller together using their landed main commits. Both queue authorization
and the new memory contract remain required. Approve every exact workflow
revision before adding the profile selector or enabling Jane execution; keep
the legacy shared scheduling label absent.

### Auxiliary profile stage

The next auxiliary versions replace only the shared guard pin and the Jane
runner selector. `.github/fleet-profile-stage.json` records their immutable
source, landed guard commit and filenames. Required Repository Policy runs
the complete delta, main-ancestry and active-byte regressions through
`scripts/test-fleet-runner-policy.py`; every existing command, permission,
publisher boundary, toolchain and hosted fallback remains unchanged.

| Family | Compatible helper |
| --- | --- |
| Benchmark | `fleet-benchmark-v3.yml` |
| Fuzz | `fleet-fuzz-v4.yml` |
| Gauntlet | `fleet-gauntlet-v4.yml` |
| Sharded mutants | `fleet-mutants-sharded-v4.yml` |
| Mutation scope | `fleet-mutation-scope-v3.yml` |
| OSV | `fleet-osv-v5.yml` |
| WASM candidate | `fleet-wasm-candidate-v4.yml` |
| WASM size report | `fleet-wasm-size-v5.yml` |

The helper stage left the manifest, active files and callers unchanged. This
activation pins all eight helpers to the stage's actual squash commit and v6
to #1260's actual squash commit, switches the manifest and retires the previous
files. Append each exact filename and landed SHA to the
existing `ci-trusted-main` allowlist before Jane execution. Composite-action
pins need no separate runner-group entry. Preserve the existing
`trusted-vps.yml@refs/heads/main` diagnostic entry and every repository and
workflow restriction.

The landed CI revision is #1260's
`05fca2fc1029be782afe41ee6ebbeae812e5e40c`; the landed helper revision is
[#1267](https://github.com/esaueng/remus/pull/1267)'s
`4e5bd846fae03e87e3d4ca1169e35b18952d4fb9`. Append these exact Remus entries to
`ci-trusted-main`, retaining all existing entries and group restrictions:

```text
esaueng/remus/.github/workflows/fleet-ci-v6.yml@05fca2fc1029be782afe41ee6ebbeae812e5e40c
esaueng/remus/.github/workflows/fleet-benchmark-v3.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-fuzz-v4.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-gauntlet-v4.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-mutants-sharded-v4.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-mutation-scope-v3.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-osv-v5.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-wasm-candidate-v4.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
esaueng/remus/.github/workflows/fleet-wasm-size-v5.yml@4e5bd846fae03e87e3d4ca1169e35b18952d4fb9
```

OpenZCAD's compatible activation is on main via
[#819](https://github.com/esaueng/OpenZCAD/pull/819). Its corresponding group
entry is
`esaueng/OpenZCAD/.github/workflows/fleet-ci-v2.yml@6cd1c223fe3b3ac49f6379e0999f1b4bbe310f04`.
The shared runtime rollout owner applies the group and label cutover after both
compatible activation callers are on main; merging this PR alone grants no runner
access.

After the controller mapping, exact allowlist and per-slot guards are verified,
add `ci-server-jane-5gb` to `ci-server-jane-1`, `ci-server-jane-2` and
`ci-server-jane-3`, retaining their individual slot labels and default platform
labels. Keep shared `ci-server-jane` and `ci-remus-light` labels absent from
Jane's slots, and keep `CI_FLEET_LIGHT_POOL_ENABLED=false` during this rollout.
The controller target remains `ci-server-jane`; the profile label does not
replace the target. New queue candidates use v6 only after the activation
caller reaches main. Existing PRs must pick up that caller, and jobs already
assigned retain their runner.

The profile label is shared with OpenZCAD. If it is added for OpenZCAD before
this combined activation reaches Remus main, keep Remus
`CI_FLEET_ENABLED=false` so legacy callers remain hosted. The simplest
coordinated order is to merge both compatible activations with hosted CI,
then add the shared profile label and enable fleet routing. Do not let the
controller select Jane for the old Remus manifest during the transition.

## Updating immutable workflow dependencies

`.github/fleet-workflows.json` names the active file for each workflow family.
Routing and policy tests read those selected files and require every caller to
use a full commit SHA reachable from `origin/main`, with exactly the same bytes
as the checked-in active callee. Mutable pins and inactive filenames fail closed.

A dependency update to a callee therefore takes two protected PRs. First stage
new versioned `fleet-*-vN.yml` files while leaving callers and active files
unchanged. The staging regression compares all nine files with the previous
main revision and permits only the reviewed Action pins and version comments
to change. It also compares both the active worktree and current-main callee
against the staged bytes, so an intervening command, permission or guard change
blocks activation until the staged version incorporates it. After the staging
PR passes the merge queue and lands, switch the
manifest and callers together to the versioned files at the actual landed main
commit. Retire the old files from the current tree in that activation PR;
historical immutable callers can still resolve their original commit.

The runner-group allowlist must separately authorize each exact active filename
and landed SHA before enabling fleet execution. Preserve existing restrictions
and entries. Hosted fallback remains available while fleet routing is disabled;
merging these files does not change repository variables or runner permissions.

## Convergence and merge queues

The caller cancels superseded runs only for the same ref. There is no global
suite concurrency group: separate PR suites can make progress at the same time,
and one suite's slow coverage job cannot prevent another suite from starting.
Jane's three registered slots bound actual server execution to three jobs across
all workflows and repositories. Jobs wait for a free slot without oversubscribing
the host. Coverage jobs may run together when separate slots are available.

The WASM size report comment runs in its own immutable callee after the
suite, because it needs a write token that the suite never receives. The size
policy itself is enforced inside the suite: the read-only `wasm-size-gate` job
runs the base commit's `scripts/wasm-size-gate.mjs` on the PR's built modules
and is part of the completion gate (it may skip only when no package was built
or the run is not a PR). There is no additional hosted completion wrapper. Existing PR branches must pick up the
new caller before their runs stop using the old admission queue.

### Required-check migration

The earlier hosted-wrapper migration changed the required check context from
`CI Pass` to `checks / CI Pass`. The current protection already requires
`checks / CI Pass` from GitHub Actions (app ID 15368). This activation retains
that context, the other required checks, the active merge queue and every
review/protection setting. No check-name migration is needed. Older PR branches
must pick up the activated caller to use its routing policy.

The runner-group allowlist must include each exact callee revision in the
callers. Preserve every existing allowed repository and workflow. Each callee
pins the shared guard action separately; updating its content requires a new
commit and corresponding pin updates. Missing configuration still uses hosted
runners. PRs and merge groups without verified owner trust remain hosted.

Repository Policy and Secrets Scan run before any expensive checks. CI Pass
requires both to succeed, requires the classifier outputs to be valid, and
rejects unexpectedly skipped selected checks. Documentation-only selection
still skips the heavy suite.

Repository Policy runs `scripts/test-owner-pr-routing.py`, whose explicit
`load_tests` hook includes the entire `scripts/test_fleet_workflows.py` suite.
Pin validation therefore gates `checks / CI Pass`: every active workflow must
exist at its immutable main-ancestor pin and match the checked-in bytes. A
discovery regression test keeps this suite in the required entrypoint.

### Tiers

The classifier (`scripts/classify-ci-changes.py`) selects jobs in tiers so a
kernel PR waits only for the checks that can fail on its diff:

- **Tier 1** (`heavy`, every source change): Test (clippy, nextest, doc
  tests, complexity guards in one job), Approximation Census, WASM without
  optional I/O, Fuzz Targets Compile and Software Rendering. The last two
  joined in `fleet-ci-v4.yml`: both build against most kernel crates, so a
  queue entry that skips tier 2 must still run them.
- **Tier 2** (`full`): Coverage, MSRV, Cargo Deny, Security Audit. Selected for every main
  push and dispatch, and for a PR only while it carries the `ci:full` label
  (label it, then re-run or push). A merge group selects it only when its
  diff touches a tier-2 input: a `Cargo.toml`, `Cargo.lock`,
  `rust-toolchain.toml`, `deny.toml`, `clippy.toml`, `.cargo/`, `.config/`,
  `.github/workflows/` or a committed package
  (a queue batch can pair a refresh with source PRs). Any other merge group
  runs tier 1 and the package build (mode `queue`), and the main push of the
  same squashed tree runs tier 2 after the merge. Coverage took about 40 of a
  queue entry's 60-70 minutes (2026-10), so this roughly halves the time a PR
  spends in the queue; a coverage or source-level MSRV regression now turns
  `main` red instead of blocking the entry. The pinned callee still passes
  `--full` for merge groups; the classifier reads `GITHUB_EVENT_NAME` itself.
- **Package build** (`wasm`): WASM Build & Validate and the advisory size
  report. Selected with tier 2, and on any PR whose diff touches any crate
  under `crates/`, `xtask`, `tools/vs-bench`, the WASM scripts,
  `Cargo.lock`, `Cargo.toml`, `rust-toolchain.toml`, or `.cargo/config.toml`.
  Every crate sits below the wasm bindings, so an engine change must pass the
  package smoke before merge. Until 2026-09-24 only `crates/wasm*` selected
  it: PR #618 (algo, math and operations only) passed PR CI with the job
  skipped and then failed `scripts/test-wasm-smoke.mjs` on main. The job runs
  beside Test (median 13.6 vs 20.6 minutes), so it adds runner time, not PR
  latency. Docs-only and CI-metadata-only PRs still skip it.
- **Package refresh**: a diff confined to `crates/wasm/pkg` and
  `crates/wasm-io/pkg` runs the full suite, merge groups included. Paths
  alone do not prove that the committed bytes came from a trusted build; no
  package-only validation bypass is enabled. When `publish.yml` opens a
  refresh PR, `scripts/publish-wasm-package-pr.py` closes the open refresh
  PRs built from an ancestor of its source commit, so only the newest refresh
  runs CI and the version guard never sees two refreshes claim one version.

CI Pass accepts a skipped job only when its tier flag is false, and rejects
a tier-2 or package selection without a heavy selection. Every main push
runs the full suite (the backstop for merge groups that ran tier 1 only), and the caller cancels superseded runs only for
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
groups may use Jane only after the immutable metadata probe verifies the exact
candidate SHA, every constituent owner author/enqueuer, the non-fork repository
and its squash chain to protected main. Missing or unsupported evidence retains
hosted execution. The probe never checks out candidate code, and a skipped or
failed probe cannot skip the application suite or satisfy a failed CI Pass gate.

`.github/merge-queue-ruleset.json` is an initially disabled, additive ruleset.
It uses squash merging, limits speculative merge builds to four, requires every
group's PR merge commit to pass (`ALLGREEN`), and adds no bypass actor. It neither replaces
existing branch protection nor changes review requirements. Do not enable it
until the caller, pinned callee and passing exact-head CI are on main. Creating
or enabling the rule does not itself enqueue PRs; enqueuing a PR authorizes its
automatic merge and still requires the maintainer's merge approval.

The queue waits up to 240 minutes for required checks. This leaves 60 minutes
beyond the longest required job's 180-minute limit for runner waiting, early
policy checks, and the final verdict. The convergence tests require that
headroom as job limits evolve. Keep the live ruleset's timeout aligned with
the template by updating the existing rule; preserve its active enforcement,
all other queue parameters, and every required check and review setting.

The repository allows squash merging only. Squash commit titles use the PR
title, and their bodies use the PR description, so each focused PR lands as
one commit describing the finished change. Start subsequent work from updated
`main` on a new branch. Reusable workflow pins must reference the landed main
commit rather than a branch commit discarded by squash merging.

Queue merge limits do not combine CI builds. The queue removes the need to
update each PR manually after another PR merges; it still validates the
combined changes against current main. Keep all required checks and reviews.

After that approval and workflow merge, create the disabled rule with:

```bash
gh api --method POST repos/esaueng/remus/rulesets \
  --input .github/merge-queue-ruleset.json
```

Inspect the returned ID and existing rules before updating, to avoid duplicate
rules. If the rule already exists, update that rule instead of creating another.
Ensure its merge method is `SQUASH` and squash merging is allowed in repository
settings. Activate this exact rule, verify it is active on main, then change only
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
