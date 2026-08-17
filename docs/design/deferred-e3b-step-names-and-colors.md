# E3b: STEP names and colors (deferred design)

Status: partially implemented (Issue 14). The attribute store
(`remus_topology::attributes` — solids + faces; name, sRGB color in
`[0, 1]`), the explicit face-propagation rules
(`remus_operations::evolution::propagate_face_attributes`), lifecycle
integration (delete, restore, solid copy), and the STEP **name** round trip
(entity name fields on `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS` /
`ADVANCED_FACE`, escape-aware) are landed. Still deferred, per the design
below: COLOUR_RGB / STYLED_ITEM presentation chains, edge/vertex/compound
scope, additional application metadata, WASM accessor methods, and the
operation-coverage audit for the remaining modifiers.

## Context

The STEP reader and writer currently exchange geometry and topology. E3a adds
writer-level product/file metadata, but remus has nowhere to retain a name or
color attached to a solid, shell, face, edge, or compound. Importing STEP
presentation data directly into a WASM-only side table would lose it during
native operations, copies, serialization, and round trips.

Names and colors are public model data, not rendering hints. Their ownership,
scope, persistence, and evolution must be defined before STEP I/O can expose
them.

## Goals

- Store optional semantic names and colors on supported topology entities.
- Preserve imported attributes through a STEP read/write round trip.
- Define deterministic propagation through copy, transform, boolean, split,
  heal, defeature, pattern, and compound operations.
- Keep exact B-rep geometry and display material concerns separate.
- Add APIs without changing existing STEP or WASM method signatures.

The first version should not implement layers, textures, transparency styles,
materials, arbitrary STEP property sets, assembly occurrence styling, or a
general document-management system.

## Attribute model

Add an attribute store owned by Topology, keyed by a typed entity reference
rather than raw arena indices. The initial value set should be intentionally
small:

    EntityAttributes
      name: Option<String>
      color: Option<ColorRgb>

ColorRgb needs a documented channel range and color space. STEP COLOUR_RGB
stores three real values; the importer should preserve finite values in the
standard range and reject or explicitly clamp invalid input. Whether those
values are interpreted as linear RGB or sRGB for rendering must be decided
once and shared by the renderer and WASM bindings.

The initial entity scope should be solids and faces:

- Solid names and colors cover common product/body styling.
- Face colors override a solid color for per-face presentation.
- Unset face color inherits from the containing solid at presentation time.

Compounds, edges, and vertices should be added only after their STEP mapping
and operation semantics are specified. Attributes must not be embedded in
Solid or Face; a relational store avoids enlarging topology records and
follows the existing pcurve-registry pattern.

## Evolution and ownership rules

Every operation that creates topology must report enough provenance to move
attributes. The existing evolution maps are the preferred source, but their
coverage must be audited before attributes ship.

Proposed default rules:

| Operation outcome | Name | Color |
|---|---|---|
| Identity/copy/transform | preserve | preserve |
| One source becomes one result | preserve | preserve |
| One source splits into several results | preserve with deterministic suffix policy, or clear pending a naming decision | copy to all descendants |
| Several sources merge into one | clear unless all names are identical | preserve only when all effective colors agree |
| Face survives a boolean | preserve | preserve |
| Generated face | unset | inherit from result solid |
| Deleted entity | remove attribute entry | remove attribute entry |

Names must never be concatenated opportunistically. A separate display label
can be synthesized by a client without changing stored semantic names.
Conflicting colors must have an explicit rule rather than depending on operand
order.

Retirement, checkpoint restore, deep-copy shape stores, and future compaction
must update or clone the attribute store together with topology. Tests should
fail if a live attribute refers to a dead entity.

## STEP import

The reader needs a presentation-resolution pass after topology creation.
Initial support should cover the common AP203/AP214/AP242 chain:

- product and representation item names for solid names;
- STYLED_ITEM and PRESENTATION_STYLE_ASSIGNMENT;
- surface style entities leading to COLOUR_RGB;
- solid-level styles and face-level overrides.

Resolution must use STEP entity references, not names, and must define
precedence when several styles target the same item. Unsupported presentation
entities should be reported through diagnostics and skipped without changing
geometry.

The import result should retain attributes in Topology; returning a detached
metadata map would not survive later modeling operations.

## STEP export

The writer should remain unchanged when no attributes are present. When they
are present:

- use the stored solid name for the relevant representation item and product
  label, with escaping through the existing STEP string encoder;
- emit one reusable COLOUR_RGB entity per distinct color;
- emit the minimal style-assignment chain for solid colors and face overrides;
- keep entity numbering deterministic by sorting on dense export order, never
  hash-map iteration order.

Writer options may control whether presentation data is emitted, but must not
invent names or colors. E3a's file/product metadata remains document metadata
and must not be overloaded as topology attributes.

## Rust and WASM APIs

The Rust API should expose typed getters/setters on Topology or an attributes
registry. The WASM API should be additive, for example:

    getSolidAttributes(handle)
    setSolidAttributes(handle, json)
    getFaceAttributes(handle)
    setFaceAttributes(handle, json)

The JSON structs need generated TypeScript types, camelCase fields, finite
channel validation, and clear unset semantics. Existing STEP import/export
signatures remain valid; optional presentation controls require new methods or
optional trailing arguments only.

## Compatibility, validation, and tests

Before implementation:

1. Approve entity scope, RGB color space/range, inheritance, and merge rules.
2. Audit all topology-producing operations for evolution-map coverage.
3. Add attribute-store invariants and checkpoint/copy/retirement tests.
4. Add STEP fixtures for solid name/color, face override, shared color reuse,
   escaped Unicode names, missing styles, and conflicting assignments.
5. Prove STEP read-write-read preservation without changing geometry,
   tolerances, analytic surfaces, rational weights, or pcurves.
6. Add WASM round trips and regenerate checked-in package artifacts.

The attribute store is a new public model-data subsystem and should receive
its own compatibility review before E3b implementation begins.
