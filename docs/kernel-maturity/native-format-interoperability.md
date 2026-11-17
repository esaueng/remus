# Native-format interoperability: geometry boundary and qualification

Scope recorded 2026-09-29 against `66ea8d6f6e526756e2f4be6983d1865fb2593fb8`.
[O5.6](roadmap.md#o-o5-6) and [O5.7](roadmap.md#o-o5-7) in the master roadmap
own status and selection. This is a supporting specification, not a new queue
or a runtime qualification report. Consumer scope is
[OpenZCAD PR #486](https://github.com/esaueng/OpenZCAD/pull/486).

## Ownership and execution decision (O5.6)

OpenZCAD requests Parasolid XT/XB, ACIS SAT/SAB and Rhino 3DM import/export,
SolidWorks part/assembly import, and later Inventor, JT, Creo, NX, CATIA V5 and
Solid Edge. Native application writers and original feature-history recovery
are outside this scope. Direction/version support is independently qualified.

Choose between an independently implemented reader, an optional external
translator and a STEP or arena-document geometry bridge. Remus remains the
modeling/validation authority. A mesh intermediate cannot qualify precise
geometry import, and successful STEP conversion alone cannot establish source
geometry, assembly or metadata fidelity.

The initial decision records:

- Format/direction/version matrix and actual-content detection, including NX
  versus Creo `.prt` and Solid Edge versus Creo `.asm` containers.
- Execution location and distribution/licensing terms. Preserve the default
  Apache-2.0/native/browser build; do not vendor proprietary SDKs or assume a
  web visualization SDK is a browser-native precise reader. XT being published
  does not by itself supply a reader or prove feasibility.
- A bounded bridge contract for solid/sheet/wire/mesh representation, source
  units and placement, names/colors, occurrence identity and a versioned report
  of rejected, healed or approximated entities. Use existing O4.6 serialization,
  O5.1/O5.4 assemblies and O5.2 attributes rather than duplicate their APIs.
- Typed error/quality outcomes, byte/entity/archive/time budgets, rollback and
  cancellation. Direct IO bindings live in `remus-wasm-io`; batch equivalence is
  required only where the single-module IO build exposes a corresponding op.
- Source/build/package provenance and native versus actual shipped WASM tests.
  External converter output needs a licensed/synthetic source witness; a
  self-generated Remus round-trip is not independent evidence.

OpenZCAD owns file selection, external-part collection, privacy/upload consent,
offline replay, retention, desktop/service orchestration and vendor purchase.
O5.5 decides external-reference responsibility; no default network fetching or
partial-assembly success is introduced by a kernel bridge.

## Precise geometry acceptance

For each supported family, preserve analytic carriers and rational NURBS
degree/knots/weights, authoritative trim intervals, oriented coedges/pcurves,
periodic seams, shared-edge consistency, shell material sense and cavities.
Conversions requiring fitting or healing must report their changed entities
and measured error; do not call them exact or widen global tolerances.

Exercise source units, small/large scale, rigid placements, multiple bodies,
disconnected components, cavities, repeated assembly instances and nested
placements. Preserve supported sheet/wire types rather than relabeling open
surfaces as solids. JT mesh-only and mixed representations retain their quality
classification; triangle-derived topology is not promoted to analytic B-rep.

Validate topology and carrier/trim agreement, independent dimensions and closed
solid volume/area, and the intended receiving reader's result. Native and
installed-WASM results must agree on successful geometry, quality reports,
refusal categories and transactional rollback; package parity reuses O1.5.
Keep unsupported cells visible. Imported exact geometry does not promise that
all later booleans, blends or direct edits are qualified on that family.

## IGES boundary (O5.7)

The existing roadmap decision declares IGES lossy preview and prefers STEP.
`importIges`/`exportIges` being present does not override that decision. Inspect
the reader and writer independently before deciding whether a precise subset
is supportable; preserve preview disclosure until qualification and a written
scope decision explicitly replace it for a named family/direction.

At the recorded source, `iges/reader.rs` parses geometry and constructs topology,
while `iges/writer.rs` emits trimmed-surface entities and lists manifold-solid
entity 186 as planned. This inventory is not proof of closed-solid, cavity,
unit or trim fidelity. Qualification must identify:

- Supported/ignored entities, transforms and source units; unsupported input
  must refuse or report an explicit preview omission rather than disappear.
- Analytic/NURBS carriers, ordered outer/inner trim loops, curve-on-surface
  relationships, closed-shell/cavity orientation and independent multiple roots.
- Independent native-reader and packaged-WASM read/write fixtures, receiving
  application behavior and round-trip geometric differences.
- A decision to keep preview-only support, or a bounded precise subset with
  implementation prerequisites and unchanged tolerances. A broader IGES program
  is not inferred from one successful fixture.

### Decision (2026-11-17): a precise B-rep subset, additive

Tier: lazy:io. The legacy `read_iges` / `write_iges` pair stays a declared
lossy preview: it reads only type 108 planes as square faces and writes planes
and polyline edges, which an independent reader opens as zero faces. It is
unchanged. The precise subset is a separate API, `read_iges_brep` /
`write_iges_brep` (translator `importIgesBrep` / `exportIgesBrep`), in
`crates/io/src/iges/brep.rs`:

| Cell | Read | Write |
| --- | --- | --- |
| Manifold solid B-rep object 186 over 514 closed shells, 510 faces, 508 loops, 504 edge and 502 vertex lists | exact | exact |
| Cavities (186 void shells) | exact | exact |
| Several 186 solids per file | separate solids | separate solids |
| Planes, cylinders, cones, spheres, tori: 190/192/194/196/198 | forms 0 and 1 | form 1 |
| Rational B-spline surface 128 | exact | exact |
| Edges: line 110, circular arc 100, ellipse 104 (form 1, standard position), B-spline 126 | exact | exact; hyperbola and parabola edges refused by name |
| Rigid 124 frames, chained | applied | written for arcs and conics |
| Units: global unit flag (and name) with model scale | applied | millimetres |
| Sphere poles as 508 vertex entries | read | not needed |
| Surface model: 143/144 trimmed surfaces, no 186 solid | refused, named with its surface count (sewing is a separate decision) | not written |
| Other surface or curve types; non-rigid or mirroring 124; open (form 2) shells; off-standard 104 conics | refused by name | n/a |
| Closed edge with no carrier (the torus primitive's degenerate seams) | n/a | refused by name |
| Names, colours, levels, parameter-space curves | not read | not written (loops carry K = 0) |
| 126 edge whose vertex lies off the curve as read (an independent slanted cut whose ellipses were written as approximating degree-6/8 splines: 0.77 mm) | refused atomically, not repaired (open) | n/a |

Reading restates the B-rep as ISO 10303-42 entities in millimetres. IGES
loops run relative to their surface and are reversed for a face that
disagrees with it. The strict STEP body reader then builds the solid, so
imported IGES solids pass the same topology, carrier and tolerance validation
and the same transactional rollback as STEP solids. Writing walks the
topology and keeps each carrier's exact class; reals use their shortest
round-trip form.

Qualification (`crates/io/tests/iges_brep.rs`) reads eleven B-rep files
written by an independent kernel's IGES writer: box, cylinder, cone, sphere,
torus, holed plate, cavity in millimetres and inches, a B-spline box, and a
tilted cylinder through chained 124 frames in millimetres and inches. Every
volume and centroid matches what that kernel's own IGES reader reports, and
every solid validates. Each file written back reads to the same solid, and
so do the native box, cylinder, cone and sphere, and a slanted cut
cylinder (ellipse edges written as 104 conics in a 124 frame). Locally, that independent
reader also opened every Remus-written file as one valid solid with the
exact volume and area. That check is not part of CI.

## Consumer-only and already-owned work

DXF/SVG parsing/writing, PDF drawings and USDZ visualization packaging stay in
OpenZCAD. Ellipse/spline constraints remain B75 and non-solid body primitives
remain P-Class 4.2/4.7 with their existing qualification limits. PLY writing
already has native and translator APIs; consumer adoption and fixture parity
do not justify rebuilding it. STEP hierarchy/names/colors/AP242 stay O5.1–O5.4.

## Source and feasibility references

- [Recorded Remus IO API](https://github.com/esaueng/remus/blob/66ea8d6f6e526756e2f4be6983d1865fb2593fb8/crates/wasm-io/pkg/remus_wasm_io.d.ts):
  IGES and PLY APIs exist; no XT/SAT/3DM API is advertised.
- [IGES reader](https://github.com/esaueng/remus/blob/66ea8d6f6e526756e2f4be6983d1865fb2593fb8/crates/io/src/iges/reader.rs)
  and [writer](https://github.com/esaueng/remus/blob/66ea8d6f6e526756e2f4be6983d1865fb2593fb8/crates/io/src/iges/writer.rs)
  are the inspection starting points, not a new support claim.
- [Siemens XT/translation overview](https://www.siemens.com/en-us/products/plm-components/parasolid/data-access-translation/),
  [CAD Exchanger direction matrix](https://cadexchanger.com/formats/) and
  [McNeel openNURBS](https://www.rhino3d.com/features/developer/opennurbs/)
  establish candidate routes; deployment/licensing and source fidelity still
  require an implementation decision and independent qualification.
