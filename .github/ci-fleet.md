# Trusted CI fleet

## Staged merge-queue overflow

`fleet-ci-v5.yml` stages an owner-only merge-queue probe on Jane. It is not
active until a follow-up PR updates the `ci` manifest/caller to the staging
commit already on main and the runner group permits that exact revision.
The existing active workflow and pin remain byte-identical in the staging PR.

The hosted-first controller and its queue-pressure threshold stay unchanged.
Only `CI_FLEET_ENABLED=true` and `CI_FLEET_TARGET=ci-server-jane` permit the
probe. It executes immutable embedded policy and the existing isolation
checks, without checking out any candidate code or using repository secrets.
Contents/PR read access lets it bind the event SHA/ref to the live queue,
verify current protected main and walk the candidate's squash-parent chain.
Every included PR and enqueuer must have the approved owner identity, with
the exact same non-fork repository and main base. The bounded query rejects
truncated evidence, unknown candidates, other authors, forks and API errors.

Application jobs use Jane only for the probe's exact verified SHA. A skipped,
failed or rejecting probe retains hosted checks; it cannot skip them or make
CI Pass green after an application failure. Existing PR/main predicates,
toolchain/action pins, commands, timeouts, required contexts, tier decisions,
publishing separation and runner isolation remain enforced. Queue-bot main
pushes retain the existing policy. Already queued jobs do not move runners.
The existing direct-fleet entrypoint runs the new policy/gate regressions.

## Existing active routing

Fleet jobs choose their runner from repository variables, evaluated in each
job's `runs-on` expression: an eligible owner PR or owner-triggered protected-main push runs on the
runner named by `CI_FLEET_TARGET` (`ci-server-jane` or `ci-server-john`) when
`CI_FLEET_ENABLED` is `true`, and on GitHub-hosted `ubuntu-24.04` otherwise.
The external controller publishes the target using fresh host/runner health
and observed hosted queue pressure. Production uses hosted-first overflow on
Jane's two slots; it has no configured backup. A job already assigned to an
offline server still waits for it.
The availability selector (`select-runner.yml`) and its only caller,
`owner-pr.yml`, were retired unused on 2026-10-19. Routing never selects the
legacy `ci-small` target.

`REMUS_TRUSTED_VPS_ENABLED` remains the pilot's activation switch; it does not
force a VPS target. `REMUS_OWNER_PR_VPS_ENABLED` was read only by `owner-pr.yml`
and was removed from the repository variables on 2026-10-20. `CI_FLEET_ENABLED`
separately enables fleet routing. Owner authorization
still checks repository/actor IDs, current PR metadata and immutable merge
parents before checkout. The runner group independently permits exact workflow
revisions; labels alone grant no access.

`trusted-vps.yml` retains its filename for the existing main-only group entry.
Its default (`auto`) uses the `CI_FLEET_TARGET` runner. Explicit manual
diagnostics may target Jane, one of its two slots, or John. Old VPS labels and
arbitrary inputs are rejected; main pushes never override the configured target. Protected main and the trusted-pilot
activation switch remain required.

Activation still needs the reviewed immutable workflow entry, verified per-slot
collectors and John label, and real job/cleanup validation. Routing cannot
retarget an already queued job or prove GitHub connectivity from a local
listener process; connectivity verification and bounded queue supervision
remain rollout gates.

Disable `CI_FLEET_ENABLED` to return new ordinary jobs to hosted. Keep old access
entries until their consumers are retired. Do not rename collector identities
when changing the backup's GitHub label; preserve its monitoring history.
