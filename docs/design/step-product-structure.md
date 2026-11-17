# STEP product structure, names and colours

Tier: lazy:io. `read_step_structure` is document-in/document-out work: the
translator module (`remus-wasm-io`) exposes it as `importStepStructure`, which
returns an exact arena document and a JSON report. It needs no kernel handles,
journal or checkpoints.

## Why a second reader

The flat readers (`read_step`, `read_step_bodies`, `importStepBodies`) build one
solid per `MANIFOLD_SOLID_BREP` in its own representation's coordinates. A file
that places parts through `NEXT_ASSEMBLY_USAGE_OCCURRENCE` therefore reads back
with every part at its definition origin and a repeated part once. Their output
is a published contract (consumers persist solid indices into it), so it stays
exactly as it is. The structure reader is additive.

The native assembly reader (`read_step_assembly`, [design](step-assemblies.md))
maps into the operations `Assembly`, whose components each own one solid. Files
from other systems routinely carry geometry-free intermediate assemblies and
part shapes split across a placement-only `SHAPE_REPRESENTATION` and an
`ADVANCED_BREP_SHAPE_REPRESENTATION`, which that model cannot hold. The
structure reader keeps a product tree instead and places solids itself.

## Contract

`read_step_structure(_with_limits)` returns:

- `nodes`: one `StepProductNode` per product occurrence, the root product(s)
  first, then components depth-first in occurrence entity order. A node's name
  is its product's `name`, or its `id` when the name is empty; `parent` indexes
  the enclosing node.
- `solids`: one root per placed occurrence of a solid: each node's own solids in
  B-Rep entity order, then its components. A definition's first identity
  occurrence is the definition itself; every other occurrence is an exact copy
  moved by the composed rigid placement (`copy_and_transform_solid_detailed`
  under `TransformPolicy::ExactOnly`), so no two roots share topology.
- `solid_nodes`: each root's node; `None` for a solid no product reaches. Such
  solids are still imported, after the tree, so partial product data loses no
  geometry.
- `presentation`: counts of applied solid and face colours and of styles not
  applied (see below).

A definition's shape is the representation its `SHAPE_DEFINITION_REPRESENTATION`
names plus every representation joined to it by an untransformed
`SHAPE_REPRESENTATION_RELATIONSHIP`; they share its coordinate frame. An
occurrence's placement comes from its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`:
the complex `REPRESENTATION_RELATIONSHIP` must join one representation of the
child's shape and one of the parent's, in either order, and the
`ITEM_DEFINED_TRANSFORMATION` items are resolved by representation membership
with the same `occurrence_placement_order` proof the body reader uses (schema
or historical order, exactly one). The local transform is
`parent_frame * inverse(child_frame)`, composed parent before child. Units are
the file's one resolved length factor, applied to geometry and placements once.

Refusals are typed and atomic (the caller's topology is restored): an
occurrence that places geometry without a placement, a relationship that does
not join its child and parent shapes, placement items owned ambiguously or not
at all, a cyclic or unreachable occurrence graph, mapped-item assemblies
(`UnsupportedEntity("MAPPED_ITEM")`), mixed units, depth beyond 256 and
expanded occurrences or names beyond the import budgets.

## Presentation

Colours are topology attributes ([E3b](deferred-e3b-step-names-and-colors.md)):
`STYLED_ITEM` and `OVER_RIDING_STYLED_ITEM` whose item is a solid B-Rep set the
solid's colour, and those whose item is an `ADVANCED_FACE` set that face's
colour (both faces of a vertex-loop sphere). The colour is the
`FILL_AREA_STYLE_COLOUR` reached through `PRESENTATION_STYLE_ASSIGNMENT` (or
`PRESENTATION_STYLE_BY_CONTEXT`) → `SURFACE_STYLE_USAGE` (both or positive side)
→ `SURFACE_SIDE_STYLE` → `SURFACE_STYLE_FILL_AREA` → `FILL_AREA_STYLE`, as
`COLOUR_RGB` (sRGB in `[0, 1]`) or one of the eight
`DRAUGHTING_PRE_DEFINED_COLOUR` names. An overriding style always sets its
item; two plain styles naming different colours leave the item uncoloured.
Copies carry the attributes, so every occurrence of a coloured part is
coloured.

Presentation never refuses an import. Styles on other items (shells, curves,
representations), occurrence-specific `CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM`,
chains without a surface fill colour, conflicts and out-of-range channels are
counted in `StepPresentationSummary` and skipped. The flat readers still set
only the B-Rep and face names they always set.

## Evidence

`crates/io/tests/step_structure.rs` reads fixtures written by an independent
XCAF STEP writer (AP214, AP242 edition 1 and an inch assembly; a millimetre and
an inch part) and compares every placed volume, centroid and colour with what
that kernel's own XCAF reader reports for the same files. Mutations of those
files cover the refusals above, historical item order, reversed relationship
order, conflicting, overriding, predefined, out-of-range and unapplied styles,
unreached solids and atomic rollback; the native assembly writer's output reads
back with its placements. `crates/wasm-io` tests the translator's report and
arena attributes.
