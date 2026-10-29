---
row: 6.6
---

Tier: core: complete-circle inputs extend live GCS construction references,
fixed solver parameters, opaque handles, atomic refresh and checkpoints.
`gcsAddExternalCircle` returns `{circle,center}` for existing constraints.
Native circles require construction-unique identity, a topologically shared
seam vertex, authoritative signed full-turn trim, finite nondegenerate radius,
a sound right-handed carrier and matching canonical parallel normal. Carrier
roundoff must also fit the unchanged linear tolerance; nearby oblique normals
are refused rather than circularizing an ellipse. Both seam endpoints are
checked independently. Curve phase and traversal are not full-circle identity.

Native fixed-radius records exclude radius columns from the solver instead of
adding a removable radius equation. Driven centers reject `gcsSetPoint` and
indirect ellipse-center edits. Constraints, plain/detailed nonconvergence and
constraint deletion cannot unlock or drift a driven radius. Successful refresh
remains current on nonconvergence under the existing refresh-first contract;
thrown refresh, solver and DOF errors restore pre-call sketch state.
Checkpoint restore retains metadata and opaque-handle high-water marks. Native
circle deletion removes lock records; logical lock counts are bounded by live
circles. Existing checkpoint byte telemetry continues to exclude sketches.

Native geometry and actual direct/legacy/V2 package witnesses use a primitive
bored box: radius 2.5→3 and top-plane movement 4→5 retain construction identity,
and an ordinary circle constrained by coincident centers/equal radius follows.
The external input has zero free parameters; its consumer has three. Evidence
covers three scales, translation/rigid placement, signed/phase storage, strict
refusal/rollback, exact arena/reference-history roundtrip, resource failures,
checkpoint undo and retired handles. The permanent linked-sketch package helper
runs in both canonical fresh and npm-installed tarball suites.

Arcs, ellipses, oblique projection, general surface attachments and portable
complete sketch documents remain open; this does not complete all of 6.6 or
refresh committed generated packages.
