# Browser draft-angle analysis

Tier: core (resolves live session face and solid handles; read-only).
Owner: P-Class 7.5.

Expose the existing native signed draft query without changing its geometry.
`getFaceDraftAngle(face,u,v,pullX,pullY,pullZ)` reports angleRadians and
outwardNormal. Parameters address the surface carrier, not trimmed membership.
`getPlanarSolidDraft(solid,pullX,pullY,pullZ)` returns those fields plus face
for every outer/cavity face in explorer order; curved faces refuse atomically.
Direct calls return JSON strings; both batch contracts return JSON values.
Angles are radians, zero is perpendicular to pull. A negative angle does not
prove global undercut or tool accessibility. No min/max over a curved face,
meshed draft map, topology mutation, new tolerance, or new dependency is implied.

Qualification must include closed-form plane/analytic samples, orientation,
scale/placement, invalid pull/parameters/handles, curved-report refusals,
checkpoint/session preservation, and fresh plus installed-package execution
of direct, legacy batch and V2 routes.
