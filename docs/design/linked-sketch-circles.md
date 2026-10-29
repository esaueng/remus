# Construction-linked complete sketch circles

Tier: core. This extends the live GCS sketch's construction references, opaque
handles, atomic input refresh and checkpoints. The independent sketch crate
continues to depend on no geometry or topology crate.

The bounded input is one construction-resolved, topologically closed native
Circle edge with a stored signed full-turn domain. Its finite positive-radius
carrier must have a right-handed orthonormal basis parallel to the attached
plane, and both domain endpoints must agree with the seam vertex. Missing
authority, partial/multiple turns, oblique projection, inferred identity and
split/deleted/ambiguous references refuse without changing published state.
The circle has no distinguished angular endpoint; seam phase and stored
traversal changes do not change its center/radius identity.

`gcsAddExternalCircle` returns ordinary `{circle, center}` handles usable by
existing constraints. The center is fixed and read-only. A private native
fixed-radius registry excludes the radius from solver parameters; no removable
radius equation stands in for a lock. Only explicit reference refresh updates
the driven center/radius. Removing user constraints, failed convergence, rank
analysis, resource refusal, checkpoint restore and stale handles must retain
that rule. Native circle deletion also retires its radius-lock metadata.

Refresh resolves every line and circle before publishing their cloned sketch
state. Existing typed linked-sketch refusals and thrown-error
rollback remain shared by direct and both batch APIs. Successful refresh stays
current on nonconvergence; detailed solve restores its free parameters against
those refreshed fixed inputs, following the existing refresh-first contract. Checkpoint
clones include all retained reference and fixed-radius metadata. Existing
checkpoint memory telemetry explicitly excludes session sketches; this change
does not reinterpret its topology-only byte estimates. Native fixed-radius
record counts remain bounded by live circles and are checked through cleanup,
clone/restore and failed-call tests.

Qualification uses a primitive bored box whose rim survives journaled radius
growth from 2.5 to 3 and top-plane movement from z=4 to z=5. A free consumer
circle constrained by coincident centers and equal radius follows the source;
the external circle contributes zero free parameters. Tests include three
scales, placement, domain/carrier poisoning, signed/seam storage, atomic
refusal, read-only setters, lock cleanup, resource rollback and undo. Actual
direct/legacy/V2 exports are exercised by the permanent linked-sketch package
witness in both canonical fresh and npm-installed tarball suites.

Arcs, ellipses, oblique projections, general surface attachments and portable
complete sketch documents remain outside this slice.
