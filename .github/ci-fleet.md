# Trusted CI fleet

## Three-slot profile cutover

The staged v6 workflow combines trusted queue authorization with three slots
at nominal 5 GB each. Its scheduling selector is `ci-server-jane-5gb`; the
controller preference stays `ci-server-jane`. The controller mapping and exact
main workflow revisions must be deployed/approved before adding the profile
label to the verified slots. Keep the old shared label withheld so legacy
6 GiB callers cannot match them. Preserve runner names, selected repositories
and exact workflow restrictions. Compatible auxiliary guard versions must be
staged at actual main commits before the coherent caller/manifest activation.
The routing below describes the earlier active workflow contract.

Fleet jobs choose their runner from repository variables, evaluated in each
job's `runs-on` expression: an owner PR or a protected-main push runs on the
runner named by `CI_FLEET_TARGET` (`ci-server-jane` or `ci-server-john`) when
`CI_FLEET_ENABLED` is `true`, and on GitHub-hosted `ubuntu-24.04` otherwise.
Nothing checks availability: a job queued for an offline server waits for it.
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
