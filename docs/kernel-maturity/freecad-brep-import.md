# Native BREP import

Owner: O5.6; OpenZCAD consumer: F04a. Tier: lazy:io.

This independently implemented reader replaces the geometry-decoding part of
FreeCAD saved-solid import in stages. It belongs in `remus-io`, exposed through
`remus-wasm-io` as exact arena documents, never in the eager modeling module.
It introduces no reference-kernel library dependency and does not execute
FreeCAD code.

The first qualified envelope is ASCII topology V1–V3, straight 3D edges,
at most two linear pcurves per edge, planar faces, one shell per solid, and
compound roots containing solids. Only identity locations are accepted in
this first slice. Units are millimetres, as in FreeCAD saved shapes; standalone
BREP callers must supply geometry in those units. Source vertex/edge tolerances
are retained within the kernel's existing entity-tolerance ceiling. Saved
validity flags are not acceptance evidence. Imported solids must pass strict
Remus validation before the transaction commits.

Malformed references, unsupported geometry/placements, surface-only roots,
non-manifold results and exceeded resource budgets refuse the whole import.
Repeated shell/face/wire occurrences also refuse, bounding reconstruction by
source records instead of expanding a shared source DAG. Shared vertices and
edges retain their source identity.
No welding, healing, tessellation fallback or tolerance widening is permitted.
The decoder counts geometry records, shape records, representations and child
references against one aggregate budget before allocating their storage.

Planar face boundaries have one simple perimeter and zero or more simple,
strictly enclosed, mutually disjoint holes. Loop order does not identify the
perimeter. Holes must already wind opposite to it; the reader does not repair
source winding. Crossing, touching, nested or disconnected loops refuse.
Classification uses the exact straight edges and source entity tolerances,
with at most 128 loops per face and 8,000,000 segment/containment comparisons
across the import. Direct dominant-coordinate projection preserves source
incidence independently of the saved UV frame; projected clearance tests
refuse conservatively. A zero-tolerance touching-hole control prevents chart
rounding from creating an accepted gap. This extension remains Tier: lazy:io.

The API is additive. OpenZCAD has withdrawn its previous saved-solid converter;
restoring native import requires broader reader coverage and a consumer PR
qualifying persistence, STEP replay and export. Subsequent slices own rigid
locations, cavity shells, curved carriers and seam pcurves,
rational/periodic splines, and FreeCAD binary framing. Native feature-history
transfer is outside this contract.

`tryImportBrep` adds a consumer fallback contract in the same lazy module.
A result with `supported=true` carries exact, strictly validated solid arena
bytes. `supported=false` carries no arena and a diagnostic `unsupportedReason`;
only the native reader's `UnsupportedEntity` verdict produces it. Parse errors,
invalid topology, resource limits, invalid API arguments and serialization
failures still throw. Consumers branch on `supported`, never diagnostic text.
The existing `importBrep` throwing contract remains available unchanged.
An unsupported verdict stops at the reader's support boundary; it does not
certify the remaining source records. Any alternate converter must validate
the source and its result independently before the consumer commits a body.

Qualification uses a saved V1 box from FreeCAD's public test archive and a V3
box from OpenZCAD's synthetic fixture, plus a derived V2 framing control.
Synthetic two-hole, blind-pocket, concave-perimeter and oblique-plane solids
qualify inner-loop reconstruction and order independence. These additionally
check material/void probes, outward closed tessellation and retained holes
through STEP/arena transport and the packaged JavaScript API.
Tests must check physical properties,
shared topology, STEP/arena transport, input bounds and atomic refusal. Fixture
provenance is recorded beside the files; the original archives' XML metadata
is not committed.
