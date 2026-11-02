# Native BREP import

Owner: O5.6; OpenZCAD consumer: F04a. Tier: lazy:io.

This independently implemented reader replaces the geometry-decoding part of
FreeCAD saved-solid import in stages. It belongs in `remus-io`, exposed through
`remus-wasm-io` as exact arena documents, never in the eager modeling module.
It introduces no reference-kernel library dependency and does not execute
FreeCAD code.

The first qualified envelope is ASCII topology V1–V3, straight 3D edges,
linear pcurves, planar faces with one boundary, one shell per solid, and
compound roots containing solids. Only identity locations are accepted in
this first slice. Units are millimetres, as in FreeCAD saved shapes; standalone
BREP callers must supply geometry in those units. Source vertex/edge tolerances
are retained within the kernel's existing entity-tolerance ceiling. Saved
validity flags are not acceptance evidence. Imported solids must pass strict
Remus validation before the transaction commits.

Malformed references, unsupported geometry/placements, surface-only roots,
non-manifold results and exceeded resource budgets refuse the whole import.
No welding, healing, tessellation fallback or tolerance widening is permitted.
The decoder counts geometry records, shape records, representations and child
references against one aggregate budget before allocating their storage.

The initial API is intentionally additive. OpenZCAD keeps its current reference
converter until native reader support and the existing STEP replay/export
qualification are exercised in a consumer PR. Subsequent slices own rigid
locations, multiple face loops/cavities, curved carriers and seam pcurves,
rational/periodic splines, and FreeCAD binary framing. Native feature-history
transfer is outside this contract.

Qualification uses a saved V1 box from FreeCAD's public test archive and a V3
box from OpenZCAD's synthetic fixture, plus a derived V2 framing control.
Tests must check physical properties,
shared topology, STEP/arena transport, input bounds and atomic refusal. Fixture
provenance is recorded beside the files; the original archives' XML metadata
is not committed.
