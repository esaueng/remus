# RFC 0005: Body taxonomy

Status: accepted in PR #127; implementation staged as the P-class program
doc's Issues 4.2–4.7 (M4). The Stage 1 class, validation, and arena-tagging
substrate merged in PR #209. The Stage 2 operations/WASM tranche was
implemented in PR #210, standalone arena-v4 sheet roots in PR #211, sheet
bounding box and center-of-area in PR #212, and STEP surface-model exchange in
PR #213. Together they implement Issue 4.2's exit gate. This RFC re-declares
the capability matrix's body-type axis —
"solid, sheet, wire, compound, cavity-bearing solid, and later general body"
(`docs/kernel-maturity/capability-matrix.md`) — against concrete semantics;
every sheet/wire/general cell starts Unqualified, with bounded Issue 4.2 sheet
cells qualified by the capability matrix's cited witnesses.

Characterization anchors: `crates/algo/src/builder/builder_solid.rs` fn
`assemble` (single-solid convention, TODO below); `check_shell_closed`
(`crates/check/src/validate/shell.rs` — an open shell was an unconditional
validation error before Stage 1); `crates/io/src/step/writer.rs`
(the pre-Stage-2 `MANIFOLD_SOLID_BREP`-only baseline).

## Problem

The program doc's Issue 4.1 names the gap:

> Sheet-body semantics (an open shell as a first-class operand: orientation,
> boundary wires, validation contract), wire bodies, and the cellular result
> model — what a boolean returns when the outcome is regions (Compound of
> solids with shared-face bookkeeping vs. true cell complex; recommend the
> former first, it composes with the existing Compound type). Classification
> semantics for sheet operands (side-of, not in/out).

**One body class.** The only body the API surface speaks is `Solid`
(`crates/topology/src/solid.rs:14-19`). `Shell`'s own doc says "An open
shell represents a sheet or partial boundary"
(`crates/topology/src/shell.rs:12-13`) and `Wire` may be "open (a path) or
closed (a loop)" (`crates/topology/src/wire.rs:71-72`), but neither is a
body: no operation accepts one, and the arena format is solid-rooted
(`serialize_solid`, `crates/io/src/arena_io.rs:524`).

**Assuming-solid everywhere.** GFA receives two `SolidId`s
(`crates/algo/src/gfa.rs:28-33`) and deep-copies both into an isolated
store (`crates/algo/src/ds/shape_store.rs:47-51`); the L3 boolean, split,
and `sew_faces` all return `SolidId`s
(`crates/operations/src/boolean/mod.rs:158-163`,
`crates/operations/src/split.rs:75-80`,
`crates/operations/src/sew.rs:37-41`); classification is in/out ray
casting with `PointClassification::{Inside, Outside, OnBoundary}`
(`crates/check/src/classify/mod.rs:20-27`), and the winding-number
classifier also takes a `SolidId` (`crates/check/src/classify/winding.rs:29`).

**The multi-region convention.** When GFA classifies several
independently-closed result shells, `assemble` folds every closed growth
shell into the single outer shell so their volumes add, and drops (or
aborts on) open ones:

```rust
    // TODO: use a `Compound` for true multi-region results.
```

(`crates/algo/src/builder/builder_solid.rs:1509`, surrounding rationale at
lines 1499–1508.) L3 repeats the convention — a disjoint fuse merges both
operands into one solid (`crates/operations/src/boolean/mod.rs:788-794`),
and `cut_multi_region_input` recombines per-component cut results' faces
into one outer shell (`crates/operations/src/boolean/mod.rs:3965-3977`) —
so a cut that severs a body returns a single "solid" with invisible
partition faces, or refuses into the mesh fallback where exact surfaces
are lost (`ExactOnlyUnattainable`,
`crates/operations/src/boolean/mod.rs:1253-1258`).

**I/O and bindings.** The STEP writer emits only `MANIFOLD_SOLID_BREP` over
`CLOSED_SHELL` (`crates/io/src/step/writer.rs:717`, `:767`); nothing
anywhere handles `SHELL_BASED_SURFACE_MODEL`. WASM:
`fuse`/`cut`/`intersectSolids` each return a single `u32` handle
(`crates/wasm/src/bindings/booleans.rs:106`, `:122`, `:194`).

## Design

### Body classes (topology, L1)

A **body** is a first-class handle over an existing entity tagged with a
new additive `BodyClass` enum (`Solid` — today's solid; `Sheet`; `Wire`;
`General`, deferred): a solid body is a `SolidId`; a sheet body is a
`ShellId` tagged `BodyClass::Sheet`; a wire body is a `WireId` tagged
`BodyClass::Wire`. No new arena entity: `Shell` and `Wire` gain an
additive `body_class` field (defaults `Solid` and `Wire`), set via
`Topology::set_shell_body_class` / `set_wire_body_class`, read via
`Topology::body_class_of`. A `Solid` is always `Solid`; tagging it
otherwise is refused.

### Sheet-body semantics

A sheet body is an open-or-closed shell used as a first-class operand.
**Boundary wires**: face boundaries are wires, as RFC 0002 left them — no
new representation; a sheet body additionally carries *free boundary wires*
(edges used by one face), reported, not errors. **Orientation**: shared
edges pair once-forward/once-reversed; the existing sense test
(`crates/check/src/validate/shell.rs:118-159`) applies unchanged. A sheet's
*material side* is defined by its faces' effective normals (`Face::reversed`
composition, `crates/topology/src/face.rs:223-229`). **Validation
contract**: free boundary allowed *and reported*, orientation inconsistency
an error, closure not demanded — the sheet profile replaces
`check_shell_closed`'s error (`crates/check/src/validate/shell.rs:174-208`)
with a `Warning`-severity free-boundary report, so "open by design" is
distinguishable from "should be closed". **Measurement**: area, bounding
box, center of area — yes; volume — a typed refusal
(`body_class_measure_mismatch`).

### Wire bodies

A wire body is a `Wire` tagged `BodyClass::Wire`: measurable (length),
transformable, usable as a sweep profile source. Stage 7 lands tagging,
measurement dispatch, serialization, and a bounded validated closed-planar
sweep profile path; open and non-planar profiles remain typed refusals.

### Cellular result model

- **v1 (recommended): a `Compound` of independently closed solids**, with
  shared-face *bookkeeping* rather than shared topology. `Compound` exists
  and anticipates this ("e.g. the result of a boolean split",
  `crates/topology/src/compound.rs:9-12`); `explode` already turns one back
  into `Vec<SolidId>` (`crates/operations/src/compound_ops.rs:17-23`). Each
  region is watertight, individually valid, independently measurable; and
  the bookkeeping is a journal claim, not topology — the two instantiations
  of a cut face (one per adjacent region) each journal `Generated`/`Merged`
  from the same source face, so the naming stack can answer "these two
  faces were one" without the engine sharing entities.
- **Why not a true cell complex (or `CompSolid` with literal shared
  faces):** one `FaceId` shared by two regions makes its edges used by
  three or more faces, which every manifold consumer treats as an error —
  `check_shell_closed` reports edges shared by >2 faces as non-manifold
  (`crates/check/src/validate/shell.rs:193-194`) — and export,
  tessellation, and measurement all assume two-sided edges. A cell complex
  needs radial edge machinery; that is a later RFC (`CompSolid::
  shared_faces`, `crates/topology/src/compsolid.rs:20-25`, is the future
  target).

### Classification semantics for sheet operands (check, L2)

Sheet operands classify **side-of**, not in/out: a new
`SideOf::{Positive, Negative, On}` with
`classify_point_side_of_sheet(topo, shell, point) -> SideOf`. `Positive`
is the side the sheet's effective normals point toward (its material
side), `Negative` the other, `On` within tolerance. It reuses the ray-
surface machinery (`crates/check/src/classify/ray_surface.rs`): one hit
gives the oriented normal, the sign of the dot product with hit-to-point
answers the side, and `classify_point`'s majority-vote pattern
(`crates/check/src/classify/mod.rs:58-107`) adds robustness. `Inside`/
`Outside` are never returned for a sheet; the solid classifier refuses
sheet inputs typed.

### Boolean pairing type-check (L3 dispatch)

The engine (L2) keeps speaking solids and face sets; body-class typing is
a dispatch matrix in `remus-operations` on
`(operand_a.class, operand_b.class)`:

| A \ B | Solid | Sheet | Wire | Compound |
|---|---|---|---|---|
| **Solid** | booleans (today) | split-by-sheet (Stage 3); sheet-trim tool (Stage 4) | refuse | refuse (flatten first) |
| **Sheet** | trim-by-solid (Stage 4) | mutual sheet×sheet trims (Stage 4) | refuse | refuse |
| **Wire** / **Compound** | refuse | refuse | refuse | refuse |

Qualified in v1: solid×solid (unchanged), solid split-by-sheet, sheet×solid
trim (keep-in and keep-out), sheet×sheet mutual trim. Everything else
refuses with a stable typed code (`body_class_operand_unsupported`, both-
sides tested) — the program's fail-closed rule applied to the new axis.
Refusals live at L3; L2's face-set mode is body-class-agnostic.

## Migration

Every stage names its characterization tests; one authority at a time.

### Stage 1 — body-class enum, tagging, validation profiles

Files: `crates/topology/src/topology.rs`, `shell.rs`, `wire.rs`, `lib.rs`;
`crates/check/src/validate/`; `crates/io/src/arena_io.rs`. Add the enum and
tagging; per-class validation profiles (`validate_solid` rejects a
sheet-tagged boundary; new `validate_sheet_body` — existing shell/wire checks,
free boundary as `Warning` via a new `ShellFreeBoundary` check id — and
`validate_wire_body`); an additive `body_class` field on shell/wire
serialization records whose absence loads as the default class.

Delivered in PR #209: the public body-class vocabulary and
validated tags, class-aware solid/sheet/wire validation, stable diagnostics,
and backward-compatible arena-v3 tags. PR #210 supplies the first sheet L3
and WASM entry points. PR #211 adds versioned standalone sheet roots while
freezing existing v3 writer bytes; wire roots remain a later tranche. PR #212
adds spatial properties and PR #213 adds STEP surface-model exchange. Together
they satisfy the full Issue 4.2 implementation exit witness.

Characterization: the pre-Stage-1 test pinned that an open shell errored on
`ShellClosed`; it flips to the sheet profile emitting the free-boundary warning.
Exit gate: an open shell constructs as a sheet body and validates clean
(warning only); an untagged shell passed where a solid is required refuses
typed; legacy arena round-trip byte-stable.

### Stage 2 — sheet bodies first-class

Files: `crates/operations/src/sew.rs`, `tessellate/`, `measure/`,
`crates/io/src/step/{writer,reader}.rs`, `crates/wasm/src/bindings/`.
Sheet construction from faces with area properties; a body-level
tessellation wrapper (open boundary expected, not an error); measurement
dispatch (volume refuses typed on sheets); STEP `SHELL_BASED_SURFACE_MODEL`
over `OPEN_SHELL` (and `CLOSED_SHELL` when closed) both directions;
construct/measure/mesh bindings with `executeBatch` companions and
contract tests.

Delivered for review in PR #210, PR #211, PR #212, and PR #213: construction is
transactional and validation-gated; body dispatch exposes sheet area, bounding
box, center-of-area, and typed volume refusal; the open-boundary tessellator
omits solid-only proximity repairs so an intentional sub-deflection trim
survives; direct and batch WASM expose the same contracts. Arena v4 preserves
standalone sheet roots, trimmed NURBS authority, pcurves, root order, and
duplicates without changing v3 bytes. STEP maps tagged sheets through
`SHELL_BASED_SURFACE_MODEL` over `OPEN_SHELL` or `CLOSED_SHELL`, preserves the
owning representation's tolerance cap, and leaves legacy solid-only entry
points unchanged. CAx-IF volume validation remains explicitly solid-only.

Characterization: before Stage 1, a NURBS patch could be neither exported nor
validated as a body. PR #210 pins construct → validate → area → tessellate
for a trimmed NURBS patch, deterministic open meshing, and transactional
refusal of a disconnected face set. PR #213 completes the implementation exit
witness: that patch survives deterministic STEP write → read → write through
native, direct WASM, and batch WASM paths; open sheets retain a free-boundary
warning while closed sheets import without one.

### Stage 3 — split solid by sheet

Files: `crates/algo/src/ds/shape_store.rs`, `gfa.rs`,
`crates/operations/src/split.rs`, `section.rs`. A **face-set operand
mode**: tool faces participate in pave/split/classification without a
bounding solid — the builder classifies A's sub-faces against the sheet's
faces but never selects sheet faces for assembly. `split.rs` grows
`split_by_sheet(topo, solid, sheet) -> Result<CompoundId>`, the first
consumer of the cellular result model; `section.rs` keeps its
wire-returning contract.

Implementation: PR #214 adds the first bounded face-set arrangement. A
single connected cylindrical sheet crosses a solid without acquiring a
volumetric classification; only its inside patches close the two cells, with
opposite orientations, and the result is an inside-then-outside Compound.
Each cell validates, the inner volume matches the cylinder closed form, the
sum reconstructs the box, and repeated native plus direct/batch WASM paths
are deterministic. Other surfaces and multi-face sheets refuse with
`unsupported_sheet_split`. `split.rs`'s existing exact-plane, two-solid API
is unchanged, and `section.rs` remains wire-returning.

### Stage 4 — trim sheet by solid; sheet×sheet

Files: `crates/operations/src/boolean/mod.rs` (the dispatch matrix; keep-
in/keep-out trims of a sheet against a solid; mutual sheet×sheet trims),
`crates/algo/src/builder/` (sheet-result assembly — selected faces form a
possibly-open shell, not a solid; classification reuses the M2-hardened
classifiers via side-of semantics), `crates/operations/src/sew.rs`
(`sew_faces` gains a sheet-body return for open results; closed results
keep solids).

Implementation: PRs #215 and #216 deliver the bounded planar Stage 4. The
face-set arrangement splits a sheet without interpreting it as material,
classifies only its patches, and assembles every selection as a new
validation-gated Sheet. Solid trims have exact inside/outside area oracles.
Sheet×sheet trims select positive or negative relative to the tool's effective
normal: a strict mutual operation returns both divided sheets, while a one-way
form composes when a finite target only imprints rather than divides the tool.
Direct and batch WASM agree. Empty selections, same-domain overlap, a target
that is not divided, and unqualified curved or multi-face inputs refuse
transactionally with `unsupported_sheet_trim`.

Characterization and exit gate: six outward-oriented planar carrier sheets
are trimmed by their four adjacent sheets, leaving six exact square faces.
`sew_faces` builds a valid six-face solid whose volume matches `make_box`
exactly and whose repeated native result is deterministic. The direct/batch
pair tests pin side selection and typed refusal at the public WASM boundary.

### Stage 5 — imprint

Files: `crates/operations/src/imprint.rs` (new), `crates/algo/src/builder/`
— GFA's split phase without the classification/discard phase: the
imprinter's faces split the imprintee's faces; every split piece survives.
Journaling is the point: each pre-existing face crossed by the imprint
journals its pieces as several `Modified` subjects sharing one `from` (the
journal's own definition of a split,
`crates/topology/src/journal.rs:215-219`); new section edges journal
`Generated` from the two participating faces. No `Deleted`, no
`Unresolved` — pure Split events, so a `PersistentRef` to a pre-imprint
face resolves `BoundMany` over all pieces in journal order
(`docs/design/rfc-0003-persistent-naming.md`, split resolution rule).

Implementation: PR #217 delivers the bounded transversal planar solid cell.
The tool participates only in the split arrangement and every target patch is
assembled into a new validation-gated solid. Face, edge, and vertex lineage is
translated through the isolated GFA store: split faces become repeated
`Modified` events, section edges are `Generated` from both participating
faces, and the unchanged tool is explicitly `Preserved`. Aliased, non-dividing,
same-domain, curved, and incomplete-lineage inputs refuse transactionally with
`unsupported_imprint`.

Characterization and exit gate: imprinting a rectangular tool loop onto a box
face leaves the 1000-unit target volume unchanged and produces no `Deleted` or
`Unresolved` event. A pre-imprint face reference resolves `Bound`, then
`BoundMany` over every split piece with construction provenance. Repeated
native results are deterministic, and direct and batch WASM agree.

### Stage 6 — multi-region boolean output

Files: `crates/operations/src/boolean/mod.rs`,
`crates/algo/src/builder/builder_solid.rs`,
`crates/wasm/src/bindings/{booleans,batch}.rs`. `assemble` returns regions
instead of folding closed growth shells into one outer shell — the TODO
quoted in the Problem section is retired, and the open-growth-shell abort
(builder_solid.rs:1517-1531) becomes a region failing its own validation,
not a whole-result abort. Multi-region acceptance returns a `Compound`
with per-region provenance; `cut_multi_region_input`'s shell-
recombination (boolean/mod.rs:3965-3977) and the disjoint-fuse shell
merge (boolean/mod.rs:788-794) migrate to genuine compounds;
disjoint-operand fuse stops routing through any approximation.
Compound-returning bindings ship alongside the single-solid signatures
(kept for compatibility during a deprecation window).

Characterization: a severing cut today either folds into one "solid" or
falls back; tests pin both paths, and flip to two valid solids. Exit gate
(Issue 4.6): a cut that severs a body returns two valid solids with
correct volumes and complete evolution; disjoint-operand fuse is exact.

Implementation: PR #218 delivers the first Stage 6 tranche. BuilderSolid's
primary final phase returns one `Solid` per growth shell in deterministic
largest-volume-first order and assigns each closed hole shell to the smallest
containing region; equal-sized ambiguity fails closed. The old `SolidId`
surface is preserved by an explicit compatibility fold. The additive exact
`boolean_regions` L3 API returns a Compound with total, non-unresolved
construction evolution for each member; direct and batch WASM expose the same
`booleanRegions` contract. The severing-box cut returns two valid 400-volume
members and disjoint fuse returns exact 10- and 24-volume members. PR #219
completes the Stage 6 exit with bounded Compound operands. Pairwise-disjoint
fuse preserves existing member roots and records total identity lineage;
intersect distributes exact GFA operations over member pairs; Cut distributes
one tool member over every target member. Direct and batch WASM match the
native contract. Intersecting-member fuse and multi-tool Cut refuse typed
until recursive provenance composition is available. The legacy single-Solid
and shell-folding helpers remain only as compatibility surfaces; new cellular
callers use the Compound-returning APIs.

### Stage 7 — wire bodies (deferrable)

Files: `crates/topology/src/topology.rs`, `measure/edge_length.rs`,
`crates/operations/src/sweep.rs`, `crates/io/src/arena_io.rs`, and
`crates/wasm/src/bindings/`. Wire-body tagging; length as body-level
measurement; a wire body accepted as a sweep profile source; wire-rooted arena
records.

Implemented in [PR #222](https://github.com/esaueng/remus/pull/222): arena v5
round-trips ordered standalone wire roots without changing released v3/v4
writer bytes; `body_length` dispatches the existing exact length calculation;
and `sweep_wire` copies a validated closed planar input into a private profile,
then validation-gates the solid result. Direct and batch WASM match the native
perimeter and prism-volume oracles. Open and non-planar wire profiles refuse
typed and transactionally. The Issue 4.7 exit gate is complete in review.

## Serialization

Stage 1 adds `body_class` as an optional field on arena-v3 shell and wire
records. The field is omitted for the default class, so old documents load to
the default class and re-saving untouched default-class entities stays
byte-identical. A sheet-tagged shell cannot be smuggled into a solid boundary:
loading refuses transactionally. Standalone sheet and wire root records are a
new root shape, not an additive tag: Stage 2 introduces sheet roots in arena
v4, and Stage 7 introduces wire roots in arena v5, each with dedicated parsers
while all earlier readers and released writers remain stable. Journal,
attribute, and persistent-reference
payloads remain per-entity — a sheet body's faces journal exactly like a
solid's.

## STEP mapping

- **Writer** (Stage 2): a sheet body emits `SHELL_BASED_SURFACE_MODEL`
  with `OPEN_SHELL` (or `CLOSED_SHELL` when closed) constituents, one per
  face; oriented edges and p-curves as today.
- **Reader** (Stage 2): `SHELL_BASED_SURFACE_MODEL` builds a tagged sheet
  body; `MANIFOLD_SOLID_BREP`/`BREP_WITH_VOIDS` stay solid-only. An
  `OPEN_SHELL` under a solid mapping remains what it is today (a shell
  wrapper, `crates/io/src/step/reader.rs:778-779`); a standalone one
  becomes a sheet body. Multi-region results (Stage 6): a `Compound`
  exports as multiple `MANIFOLD_SOLID_BREP` solids; import joins them.

## Validation additions

| Check | Code | Category |
| --- | --- | --- |
| Free boundary on a sheet body (reported, not an error) | `shell_free_boundary` | `open_boundary` (Warning) |
| Sheet shared-edge orientation inconsistent | `sheet_orientation_inconsistent` | `invalid_topology` |
| Volume/Area requested across body classes | `body_class_measure_mismatch` | `invalid_input` |
| Operand pair outside the qualified matrix | `body_class_operand_unsupported` | `unsupported` |
| Body class unresolved on load | `body_class_unresolved` | `invalid_topology` |

## Consequences

- **Cost**: a body-typed dispatch surface at L3, the face-set operand mode
  in the GFA store, and STEP writer/reader extensions. No new arena entity,
  no shared-face topology, no change to `EdgeCurve`/`FaceSurface` — the
  ripple is bounded to body-level entry points and the assembler. The
  single-`SolidId` return convention persists through a deprecation window.
- **Unblocks**: surface-modeling workflows (trim/sew loops), principled
  multi-body results, imprint with exact persistent refs — every
  sheet/wire/compound cell moves from Unqualified to Qualified, Partial,
  or Unsupported-typed in the PR that earns it.
- **Deferred**: true cell complexes (radial edges, shared faces), the
  general body class, sheet glue/union beyond trims.

## Resolved questions

- **Does a sheet body have volume?** No — it has area. `volume` on a sheet
  or wire body refuses typed (`body_class_measure_mismatch`), never a
  misleading 0 or an invented signed volume.
- **Can a compound be an operand of a boolean in v1?** Yes, within the bounded
  pairwise-disjoint contract implemented by `boolean_compound_regions`: fuse
  preserves members, intersect distributes over member pairs, and Cut accepts
  one tool member. Intersecting-member fuse and multi-tool Cut refuse typed
  until recursive lineage composition is qualified.
- **Does Face carry a material-side flag, or is orientation enough?**
  Orientation is enough. `Face::reversed` already defines the effective
  normal (`crates/topology/src/face.rs:223-229`), the shell validator
  already checks sense consistency through it, and the boolean's
  same-domain selection already compares orientation across faces. A second
  flag would be an independently-mutable duplicate of the same fact needing
  its own arbitration validator — one more invariant to break, the same
  reasoning that kept RFC 0002 from storing a seam mate pointer. The
  sheet's material side is the side its effective normals point *toward*;
  both keep-in and keep-out trims are exposed.
- **Compound of solids vs. true cell complex?** Compound first (see
  Design); a cell complex needs radial edges and would violate the
  two-sided-edge assumption validation, export, and measurement lean on
  today.
