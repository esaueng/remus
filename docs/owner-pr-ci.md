# Trusted owner CI fleet

Linux CI jobs and their completion gates live in `fleet-ci.yml`, called at an
immutable commit from `ci.yml`. GitHub evaluates the runner expression before
scheduling each job. There is no GitHub-hosted authorization or routing job in
front of the Linux checks, and no routing HTTP request from Actions.

Only the approved owner's open, same-repository PR merge refs are eligible.
Main pushes and manual dispatches also require protected main. Repository name
and numeric identity, PR author, event actor and rerun actor are checked in the
immutable workflow. Forks and other authors remain hosted. The runner group
must separately restrict the exact repository and immutable workflow revision;
a label or repository variable does not grant execution authority.

`CI_FLEET_ENABLED=true` opts in. `CI_FLEET_TARGET` accepts only
`ci-server-jane`, `ci-server-john`, or `github-hosted`. A controller outside
Actions publishes this variable after checking fresh collector health and GitHub
runner connectivity. Jane is primary, John the first backup, and GitHub-hosted
the second. Busy Jane remains available; jobs queue for either of her two slots.
Missing/invalid configuration selects hosted. Hosted fallback still depends on
GitHub billing and capacity.

Before checkout, self-hosted jobs verify the non-root identity, protected runner
files, NoNewPrivileges, fresh storage, empty rootless Docker state, mount options
and exact cgroup limits. Jane slots each have six CPU equivalents and 6 GiB RAM;
John has one CPU equivalent and 3 GiB. Guard failure fails the job.
Browser suites hold a shared host lock while using fixed localhost test ports.
Browser/native OS dependencies must be installed by the host administrator;
workflow jobs install browser binaries without sudo.

## Activation and rollback

Approve the exact immutable `fleet-ci.yml` runner-group entry without removing
existing restrictions. Configure the external controller's selected repository
identities and dedicated GitHub credential, then validate both hosts. The
caller retains the required `CI Pass` check and accepts only a successful
reusable workflow. Do not change required check names or review requirements.

Run an authorized real application job on each Jane slot, verify cleanup and
unavailable-only selection, then enable normal routing. Code merge alone does
not activate the runner group, repository variables, or controller.

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
The macOS platform job remains hosted and required by CI Pass. Jane and John
cannot replace macOS capacity. Release workflows are unchanged.

Policy verification: `python3 scripts/test-direct-fleet.py`,
`python3 scripts/test-owner-pr-routing.py`, the existing owner policy tests,
and `actionlint`. The historical `owner-pr.yml` remains unchanged for old
immutable callers; new callers use `fleet-ci.yml`.

## Convergence and merge queues

The caller cancels superseded runs only for the same ref. Its reusable `checks`
job also uses the repository-wide `remus-ci-suite` concurrency group with
`queue: max`. One complete suite runs at a time, with its jobs still parallel;
up to 100 pending suites wait without evicting one another. A larger backlog
exceeds GitHub's queue limit and needs a fresh run. The final `CI Pass` wrapper
is hosted and can still wait for hosted capacity, but never holds a fleet slot
while waiting for the suite. Existing PR branches must pick up this caller
before their runs participate in the admission queue.

Repository Policy and Secrets Scan run before any expensive checks. CI Pass
requires both to succeed, requires the classifier outputs to be valid, and
rejects unexpectedly skipped selected checks. Documentation-only selection
still skips the heavy suite. Optimized coverage uses the existing `ci-test`
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
`CI Pass`, its GitHub Actions app identity, and every review/protection setting.
This ordering leaves protection in place throughout and lets the merge queue
validate against current main without requiring manual branch updates. For
rollback, restore `strict: true` before disabling the queue rule.

Validate with the policy tests and actionlint. Actionlint 1.7.12 predates
GitHub's `concurrency.queue` key; until its parser supports it, ignore only the
specific `unexpected key "queue" for "concurrency" section` diagnostic and
confirm server acceptance in a real Actions run. Never suppress other syntax
or expression errors. The convergence tests also enforce the queue contract.
