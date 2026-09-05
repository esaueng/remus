# Trusted main VPS pilot

Status: prepared, **not activated or validated on the VPS**. This is an
additive pilot of layer-boundary lint and the CI-classifier unit tests from
`Repository Policy`. The hosted copies remain authoritative, including on
public PRs and forks. No existing job, check name, threshold, toolchain pin,
artifact flow, or publishing behavior is removed or changed.

## Inspection and blockers

Inspected GitHub on 2026-09-05 and infrastructure main commit
`0400cdc8eeac412c8bfefb9b1034ce73477d4aa8`:
[README](https://github.com/petergstfsn/ci-server/blob/0400cdc8eeac412c8bfefb9b1034ce73477d4aa8/README.md),
[public repository security policy](https://github.com/petergstfsn/ci-server/blob/0400cdc8eeac412c8bfefb9b1034ce73477d4aa8/docs/public-repositories.md),
and [infrastructure validation](https://github.com/petergstfsn/ci-server/blob/0400cdc8eeac412c8bfefb9b1034ce73477d4aa8/docs/public-validation.md).

- `esaueng/remus` is public, repository ID `1334765869`.
- Main reported `protected: false`; classic protection returned HTTP 404
  `Branch not protected`; effective branch rules returned `[]`. Consequently
  **no required checks are currently enforced server-side**, although CI has
  the aggregate `CI Pass` contract.
- Repository runner enumeration returned zero runners. Organization runner
  and group enumeration returned HTTP 403, requiring organization runner
  permissions / `admin:org`. The credential has repository admin permission,
  but this does not establish organization runner access.
- The infrastructure record names runner `ci-vm-1441561`, runner ID 15498,
  and group `ci-trusted-main`, group ID 6. These IDs and its online/idle state
  require fresh organization API verification; they are not live findings.
- `publish.yml` uses `REMUS_BOT_PRIVATE_KEY` and `REMUS_BOT_APP_ID` to push
  generated WASM packages directly to main. Enforcing PR-based changes with
  no bypass conflicts with that current behavior. Resolve that design in a
  separately authorized change before activation; do not silently break the
  publisher or grant it a protection bypass for this pilot.

## Workflow inventory

| Workflow | Triggers | Work and disposition |
| --- | --- | --- |
| `ci.yml` | main push, PR | All remain hosted: change classifier, repository policy, Clippy, workspace nextest/doc tests and complexity guards, approximation census, macOS tests, coverage (60% floor), MSRV, WASM build/validation/size report, fuzz compilation, software rendering, cargo-deny, audit, docs, secrets scan, CI Pass. Only boundaries and classifier tests are repeated in the pilot. |
| `benchmark.yml` | main push, PR | Hosted Rust benchmarks; separate hosted publisher writes gh-pages and comments with a scoped token. No baseline/artifact is imported by the pilot. |
| `fuzz.yml` | weekly, dispatch | Hosted nightly compilation and 12 fuzz targets, 120-second execution and 2048-MB fuzzer RSS limit per target. Compilation plus runner overhead is unmeasured on this VPS. |
| `gauntlet.yml` | daily/weekly, dispatch tier | Hosted CAD corpus, up to 360 minutes, two model workers; separate results-branch publisher. Existing 50-basis-point ratchet retained. |
| `mutants.yml` | weekly, dispatch | Hosted Rust mutation testing, 180-minute budget; too heavy without measurements. |
| `osv-scan.yml` | main push, main-target PR, weekly | Pinned external reusable workflows, security-events write permission; remain hosted and outside the allowlist. |
| `openzcad-wasm-release.yml` | dispatch | Hosted dual-WASM build and candidate artifacts; unchanged. |
| `publish.yml` | path-filtered main push, dispatch | Hosted package build followed by separate credentialed package synchronization; unchanged. |

Rust remains pinned to 1.96.0 in `rust-toolchain.toml` (rustfmt, clippy,
rust-src, wasm32 target); MSRV is 1.88.0 and fuzzing uses its existing nightly
selection. Existing installer versions include cargo-machete 0.9.1, taplo
0.9.3, mdbook 0.4.52, cargo-mutants 27.0.0, Gitleaks 8.30.1 with SHA-256,
and Node 24 for packaging. Existing unversioned nextest/wasm-pack installs
are unchanged. None of these Rust/Node installations is needed by the pilot.

Rendering installs Mesa via sudo and stays hosted. Cargo-deny uses a Docker
action; no container suite is migrated. Coverage has OIDC write permission,
the size reporter has PR write permission, and publishers have narrowly
scoped write tokens; none of those credentials or permissions reaches the VPS.
No Windows job currently exists; the macOS job remains on macOS.

## Trust and capacity

`trusted-vps.yml` accepts only main pushes or input-free manual dispatches.
The job additionally checks the exact repository, main ref, protected-ref
flag, and `REMUS_TRUSTED_VPS_ENABLED == 'true'`. The variable defaults to
disabled when absent. The protected-ref flag alone does not prove sufficient
protection: an administrator must verify the full policy before activation.
Server-side repository **and** workflow-ref restrictions remain mandatory.
There are no PR, target-PR, workflow-run, or reusable-workflow triggers, custom
checkout refs, cache restores, artifact downloads, or repository secrets.
Checkout uses the triggering SHA, the existing reviewed v7.0.1 commit pin,
and `persist-credentials: false`; token permission is only `contents: read`.

Both probe jobs become eligible together. The existing single runner must
serialize them across this and every other selected repository; workflow
concurrency only serializes this repository's pilot runs. Do not register
another runner. Each probe has a five-minute timeout and uses provisioned
Bash, Git, Python 3, core utilities, jq, Docker inspection, and GNU time.
It installs no tools and builds no containers. The checks execute serially.

The preflight verifies runner identity, non-root `ci-runner`, NoNewPrivileges,
root-owned listener protection, rootless Docker, empty Docker state, cleanup
sentinels, and the shared one-CPU / 3-GiB / zero-swap cgroup limits. Existing
infrastructure stops descendants and the CI-only Docker daemon and resets
work/home/tmp/Docker storage between listener sessions. Sentinels are left
for that reset; this workflow performs no pruning or host cleanup. The
32-GiB CI filesystem and five-GiB startup free-space gate remain unchanged.

Cold checkouts, tool downloads in other workflows, and serialization limit
throughput. This pilot cannot establish capacity for Rust, CAD, browser,
coverage, or container suites. Command maximum RSS is not aggregate peak
memory; slice peak/event counters may include earlier jobs.

## Activation after authorized merge

Do not enable access for the PR or a bootstrap branch. The infrastructure's
historical locked-branch bootstrap is deliberately not used here.

1. Merge only after explicit authorization and review of hosted checks.
2. Resolve the direct-main publisher conflict with authorization. Verify
   main requires PRs and passing checks including `CI Pass`, applies to
   admins, has no bypass actors, and forbids force pushes and deletion.
   Re-read classic protection and effective rules, not just `protected`.
   Keep all existing check contracts; do not require the optional VPS pilot
   to pass before a PR can merge, since it does not run on PRs.
3. With an authorized organization credential, read back the group and all
   its selected repositories, allowed workflow refs, and runners. Confirm
   exactly one runner across the machine, with the expected name and label;
   confirm the merged workflow on main matches the reviewed change.
4. Apply only this exact additive access delta, preserving existing entries:

   ```json
   {
     "organization": "esaueng",
     "runner_group": "ci-trusted-main",
     "visibility": "selected",
     "allows_public_repositories": true,
     "restricted_to_workflows": true,
     "add_repository_id": 1334765869,
     "add_selected_workflow": "esaueng/remus/.github/workflows/trusted-vps.yml@refs/heads/main"
   }
   ```

   This describes the delta, not a GitHub PATCH body. Resolve the group ID
   live. Add the repository using `PUT /orgs/esaueng/actions/runner-groups/{id}/repositories/1334765869`.
   For `PATCH /orgs/esaueng/actions/runner-groups/{id}`, retain the freshly
   read `selected_workflows` array and append only the exact entry above;
   verify `visibility`, `allows_public_repositories`, and
   `restricted_to_workflows` afterward. A complete replacement array cannot
   safely be prepared while its current contents are inaccessible. Never
   change visibility to all or disable workflow restrictions.
5. Read back both allowlists, then set repository variable
   `REMUS_TRUSTED_VPS_ENABLED=true`. Dispatch `trusted-vps.yml` with ref
   `main` and no inputs. No deployment workflow should be dispatched.
6. Inspect both job records/logs: exact runner ID/name, source SHA, non-root
   identity, tool versions, successful checks, GNU-time elapsed/RSS, and
   strictly non-overlapping job intervals. Both sentinels must be absent
   at each start, including the second job after the first leaves them.
7. After completion, use the infrastructure's existing read-only diagnosis
   to verify one listener, idle runner, cleanup of all disposable roots,
   disk free space and memory/OOM counters. Compare intervals with other
   repositories' jobs as well. Save run links and results in this document.

Until steps 1–7 pass, report **PR prepared**, not **activated and validated**.

## Local verification

On the development workstation, layer-boundary lint passed in 0.30 seconds
with 3,396 KiB maximum RSS; all 10 classifier tests passed in 0.03 seconds
with 15,460 KiB maximum RSS. Actionlint 1.7.12 (official release archive
verified against its published SHA-256), Bash syntax, documentation paths,
the five gauntlet workflow contract tests, and the publish-credential
contract passed. These timings exclude checkout and runner overhead and
are not VPS capacity results. Hosted check results belong to the PR;
no main-branch VPS run has been submitted.

## Rollback

Set `REMUS_TRUSTED_VPS_ENABLED=false` to stop future submissions, remove
only this workflow's server-side allowlist entry, and drain/cancel any
already queued pilot jobs. Remove repository access only if no other
approved workflow needs it. Keep the shared runner for other repositories.
For hosted execution, submit a PR changing the pilot's runner to
`ubuntu-24.04` and removing VPS-specific isolation/telemetry probes; retain
the two check commands. Normal hosted CI already provides both checks and
needs no rollback change. No protection weakening, Docker pruning, runner
replacement, or deployment is part of rollback.
