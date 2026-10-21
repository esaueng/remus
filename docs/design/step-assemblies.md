# Native STEP assembly subset

Tier: native. These opt-in APIs operate on native assembly structures; this
slice adds no browser export or arena-document assembly format.

`remus_io::step::write_step_assembly(&Topology, &Assembly)` and
`read_step_assembly(&str, &mut Topology)` are separate, opt-in native APIs.
`read_step_assembly_with_limits` also accepts `ImportLimits`. The read result
contains unique solid definitions, the named `Assembly`, geometry-import
diagnostics, and the declared conversion factor to millimetres. Existing body
APIs still return body definitions; they do not apply occurrence transforms.
The legacy writer's output construction remains separate and unchanged.

The supported product structure has one geometry-free root. Every component,
including a component with children, owns exactly one solid, matching the
existing `Assembly` model. A shared B-Rep entity imports once and all occurrences
refer to its `SolidId`. Repeated product definitions may expand their children
under multiple occurrences; parent-relative transforms compose in the same order
as `Assembly::world_transform`. Names are taken from the root `PRODUCT` and
each occurrence's name. The writer emits deterministic component order and one
B-Rep definition per unique solid handle, reusing existing geometry and boundary
authority encoding. Each outer and cavity shell must be nonempty, closed and
classified as a solid shell; sheet-tagged shells, open shells and empty-result
sentinels are refused by this assembly API.

The links are ordinary AP203 entities: `PRODUCT_DEFINITION`,
`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `PRODUCT_DEFINITION_SHAPE`,
`SHAPE_DEFINITION_REPRESENTATION`, and `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`.
Placements use composite shape/representation relationships with
`ITEM_DEFINED_TRANSFORMATION` and `AXIS2_PLACEMENT_3D`. The opt-in reader requires
child representation first, parent second, and unambiguous, directly owned
placement items in that order. It computes
`parent_frame * inverse(child_frame)`, then composes parent before child in the
assembly tree. ISO 10303-43's WR2/IP1 requires placement membership to agree
with representation order; the two representations have distinct contexts.
[Representation schema, section 4.4.21](https://ap238.org/SMRL_v8_final/data/resource_docs/representation_structures/sys/4_schema.htm).

The existing geometry reader historically expected the two placement items in
reverse order. It recognizes schema order too, retaining historical order only
when that order alone has both required representation memberships. If both
orders are possible, it refuses the ambiguous mapping instead of silently
choosing one for tolerance attribution. Its tolerance-authority traversal and
distinct-context requirements remain intact. The opt-in assembly reader accepts
only unambiguous schema order; historical reversed order remains a body-import
compatibility path, not an assembly-placement convention.

The writer supports finite, proper rigid affine transforms. It checks the
rotation using the kernel's angular tolerance and requires the affine bottom
row to be exactly `(0, 0, 0, 1)`: a small projective coefficient must not be
discarded during placement encoding. Every accepted placement is emitted using
the existing 17-digit numeric format, including tiny rotations and translations.
It refuses scale, shear, reflection and non-finite matrices, including overflow
when world placements compose. Exported lengths are millimetres. Import reuses the existing
declared-unit resolver and placement conversion for both geometry and positions;
one coherent file-wide length factor is required. Missing or conflicting units
are errors. Different units in different representation contexts are unsupported.

The reader refuses geometry-free intermediate groups, multiple solids per
component, mapped items, unsupported shape representations, missing or duplicate
shape links, unassigned solids, disconnected/cyclic graphs and invalid frames.
The importer limits component depth to 256 and expanded occurrence count to
`ImportLimits::max_model_entities`, separately from encoded entity count. Expanded occurrence names share a cumulative
`ImportLimits::max_input_bytes` budget to bound repeated-string amplification. The
writer can describe deeper native trees, but such files exceed this import limit.
Shared product definitions reuse parsed representation item sets and decoded
product labels; aliasing a large source payload does not reparse or copy that
payload for every definition.
Any error, including a graph/limit failure after geometry has been built, restores
the original topology while retiring newly allocated handle slots. Import uses
the current guarded append-only transaction, with mutation-local fallback if a
write ever touches pre-existing state; it does not copy the caller's full
topology as an import checkpoint.

`crates/io/tests/step_assembly.rs` covers shared definitions, repeated products,
nested noncommuting transforms, nonidentity child frames, names/BOM, analytic
faces and volume, declared-metre conversion, mixed-unit refusal, malformed and
legacy placements, graph errors, shared cavities and late rollback at both
resource limits. Additional controls cover malformed placement entity kinds,
false complex-relationship wrappers, exact affine rows, tiny rigid placements,
open/sheet-class component shells, maximum supported depth and permanently
retired solid handles after a refused expansion.
The reader's tolerance tests cover both membership conventions and unowned or
mixed placements. This is an in-process interoperability qualification; it has
not been checked in an external industrial CAD importer. Broader AP242 metadata,
pure grouping nodes, per-context units and assembly transport through the split
WASM packages remain outside this slice. No arena format or JS export changed.
