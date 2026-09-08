# Trusted CI fleet

Owner PR Rust checks and the main diagnostic workflow use the immutable shared
selector: Jane's two-slot shared label first, John only when Jane is unavailable,
then GitHub-hosted. Busy Jane keeps its queue. Disabled or unavailable routing
selects hosted, never the legacy `ci-small` target.

The legacy `REMUS_OWNER_PR_VPS_ENABLED` and `REMUS_TRUSTED_VPS_ENABLED` variable
names remain compatibility switches; they do not force a VPS target.
`CI_FLEET_ENABLED` separately enables availability routing. Owner authorization
still checks repository/actor IDs, current PR metadata and immutable merge
parents before checkout. The runner group independently permits exact workflow
revisions; labels alone grant no access.

`trusted-vps.yml` retains its filename for the existing main-only group entry.
Its default is automatic selection. Explicit manual diagnostics may target Jane,
one of its two slots, or John. Old VPS labels and arbitrary inputs are rejected;
main pushes never override the selector. Protected main and the trusted-pilot
activation switch remain required.

Activation still needs the reviewed immutable workflow entry, deployed routing
endpoint, verified per-slot collectors and John label, and real job/cleanup
validation. The selector needs hosted Actions capacity. It cannot retarget an
already queued job or prove GitHub connectivity from a local listener process;
connectivity verification and bounded queue supervision remain rollout gates.

Disable `CI_FLEET_ENABLED` to return new ordinary jobs to hosted. Keep old access
entries until their consumers are retired. Do not rename collector identities
when changing the backup's GitHub label; preserve its monitoring history.
