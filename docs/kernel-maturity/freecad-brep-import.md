# Native BREP import

Owner: O5.6; OpenZCAD consumer: F04a. Tier: lazy:io.

This independently implemented reader replaces the geometry-decoding part of
FreeCAD saved-solid import in stages. It belongs in `remus-io`, exposed through
`remus-wasm-io` as exact arena documents, never in the eager modeling module.
It introduces no reference-kernel library dependency and does not execute
FreeCAD code.

The qualified envelope is ASCII topology V1–V3, line/circle 3D edges,
line/circle pcurves, planar polygons or circular caps, and full cylindrical
bands. Solids have one shell; compound roots contain solids. Proper rigid
locations apply at shape, curve and surface occurrences. Units are millimetres,
as in FreeCAD saved shapes; standalone
BREP callers must supply geometry in those units. Source vertex/edge tolerances
are retained within the kernel's existing entity-tolerance ceiling. Saved
validity flags are not acceptance evidence. Imported solids must pass strict
Remus validation before the transaction commits.

Malformed references, unsupported geometry/placements, surface-only roots,
non-manifold results and exceeded resource budgets refuse the whole import.
Repeated shell/face/wire occurrences at the same canonical placement also refuse.
Distinct placed occurrences of a shared source solid import as distinct solids.
Shared vertices and edges retain their source identity at each placement.
No welding, healing, tessellation fallback or tolerance widening is permitted.
The decoder counts geometry records, shape records, representations and child
references against one aggregate budget before allocating their storage.

Circular caps use closed full-circle rims, optionally with strictly contained,
oppositely wound disjoint circular holes. Cylindrical bands have two full-circle
rims and a line seam used in both directions, with one authoritative pcurve
per branch. Source use order need not follow the UV cycle: a bounded four-use
search must find a unique closed boundary using exact source endpoint identities
and unwrapped UV coordinates. Left-handed cylinder frames preserve radial axes
while reversing native V and boundary orientation. Whole-interval SameParameter
and SameRange proofs must meet the actual source edge tolerance; endpoint or
sample agreement alone never authorizes a curve use. Saved V2 UV cache values
are parsed for framing only. Periodic branch winding survives STEP and arena
transport.

Ordered location compositions include signed powers and inverses. Source shape
identity plus a canonical datum/power word determines sharing; coincident
coordinates never merge source entities, even for distinct numerically equal
datums. Expansion is iterative. Materialized occurrences, references and copied
representations share a 16,384 hard ceiling, tightened by caller limits; location
terms and cached products have separate bounded budgets. Unused source edges
remain validated in scratch storage without adding orphan native entities.
Scale, reflection, shear, mixed polygon/circle caps, partial circular loops,
general curved trims and splines remain outside this extension.

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
qualifying persistence, STEP replay and export. Subsequent slices own cavity
shells, mixed and partial curve trims, additional curved carriers,
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

Qualification uses saved V1 boxes and a cylinder from FreeCAD's public archives
and a V3 box from OpenZCAD's synthetic fixture, plus derived V2 framing controls.
Synthetic two-hole, blind-pocket, concave-perimeter and oblique-plane solids
qualify inner-loop reconstruction and order independence. These additionally
check material/void probes, outward closed tessellation and retained holes
through STEP/arena transport and the packaged JavaScript API. Synthetic analytic
tubes in all three text versions qualify annular caps, inward bore walls and
both seam continuity encodings. Placement controls check oblique rotation,
noncommuting parent/child composition, inverse cancellation, distinct coincident
occurrence identities and expansion-budget rollback. Malformed traces, missing
seam branches, invalid frames/radii and unqualified affine maps refuse atomically.
Tests must check physical properties,
shared topology, STEP/arena transport, input bounds and atomic refusal. Fixture
provenance is recorded beside the files; the original archives' XML metadata
is not committed.
