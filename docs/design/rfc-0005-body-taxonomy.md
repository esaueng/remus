# RFC 0005: Body taxonomy

Status: accepted in PR #127; implementation staged as the P-class program
doc's Issues 4.2–4.7 (M4). The Stage 1 class, validation, and arena-tagging
substrate is in review in PR #209; the Stage 2 operations/WASM tranche is in
review in PR #210, and standalone arena-v4 sheet roots are in review in PR
#211. None completes Issue 4.2. This RFC re-declares the capability matrix's
body-type axis —
"solid, sheet, wire, compound, cavity-bearing solid, and later general body"
(`docs/kernel-maturity/capability-matrix.md`) — against concrete semantics;
every sheet/wire/general cell is Unqualified by default today.

Characterization anchors: `crates/algo/src/builder/builder_solid.rs` fn
`assemble` (single-solid convention, TODO below); `check_shell_closed`
(`crates/check/src/validate/shell.rs` — an open shell is an unconditional
validation error today); `crates/io/src/step/writer.rs`
(`MANIFOLD_SOLID_BREP` only).

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
transformable, usable as a sweep profile source. Stage 7 lands only the
tagging, measurement dispatch, and serialization.

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

Delivered for review in PR #209: the public body-class vocabulary and
validated tags, class-aware solid/sheet/wire validation, stable diagnostics,
and backward-compatible arena-v3 tags. PR #210 supplies the first sheet L3
and WASM entry points. PR #211 adds versioned standalone sheet roots while
freezing existing v3 writer bytes; wire roots remain a later tranche. STEP
remains, so the Issue 4.2 exit gate is still open.

Characterization: a test pins that an open shell errors on `ShellClosed`
today; it flips to the sheet profile emitting the free-boundary warning.
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

Delivered in part for review in PR #210 and PR #211: construction is
transactional and validation-gated; body dispatch exposes sheet area and typed
volume refusal; the open-boundary tessellator omits solid-only proximity
repairs so an intentional sub-deflection trim survives; direct and batch WASM
expose the same contracts. Arena v4 now preserves standalone sheet roots,
trimmed NURBS authority, pcurves, root order, and duplicates without changing
v3 bytes. Bounding box, center-of-area, and STEP
`SHELL_BASED_SURFACE_MODEL` remain before Stage 2 and Issue 4.2 close.

Characterization: before Stage 1, a NURBS patch could be neither exported nor
validated as a body. PR #210 pins construct → validate → area → tessellate
for a trimmed NURBS patch, deterministic open meshing, and transactional
refusal of a disconnected face set. Exit gate (Issue 4.2): that patch also
survives STEP round-trip; validation separates "open by design" from "should
be closed".

### Stage 3 — split solid by sheet

Files: `crates/algo/src/ds/shape_store.rs`, `gfa.rs`,
`crates/operations/src/split.rs`, `section.rs`. A **face-set operand
mode**: tool faces participate in pave/split/classification without a
bounding solid — the builder classifies A's sub-faces against the sheet's
faces but never selects sheet faces for assembly. `split.rs` grows
`split_by_sheet(topo, solid, sheet) -> Result<CompoundId>`, the first
consumer of the cellular result model; `section.rs` keeps its
wire-returning contract.

Characterization: `split.rs` refuses anything but an exact plane today
(split.rs:14-15) and returns exactly two solids; tests pin both, and this
stage generalizes rather than flips them. Exit gate (Issue 4.3): a curved
sheet splits a solid into N regions whose volumes sum exactly to the
original (closed-form oracle); each region individually valid; determinism
pinned across runs and native/WASM.

### Stage 4 — trim sheet by solid; sheet×sheet

Files: `crates/operations/src/boolean/mod.rs` (the dispatch matrix; keep-
in/keep-out trims of a sheet against a solid; mutual sheet×sheet trims),
`crates/algo/src/builder/` (sheet-result assembly — selected faces form a
possibly-open shell, not a solid; classification reuses the M2-hardened
classifiers via side-of semantics), `crates/operations/src/sew.rs`
(`sew_faces` gains a sheet-body return for open results; closed results
keep solids).

Characterization: sheet operands refused typed since Stage 1 flip to
qualified. Exit gate (Issue 4.4): a closed solid built purely from
mutually-trimmed sheets + sew has the same volume as the same solid built
by primitive booleans.

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

Characterization: pre-imprint, a face ref resolves `Bound`. Exit gate
(Issue 4.5): imprinted solid has identical volume; split faces claimed by
Split events, zero unresolved; refs to pre-imprint faces resolve
`BoundMany`.

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

### Stage 7 — wire bodies (deferrable)

Files: `crates/topology/src/topology.rs`, `measure/edge_length.rs`,
`crates/operations/src/sweep.rs`, `crates/io/src/arena_io.rs`. Wire-body
tagging; length as body-level measurement; a wire body accepted as a sweep
profile source; wire-rooted arena records (today only solids serialize —
`crates/io/src/arena_io.rs:524,538`).

Characterization: wires cannot be measured as bodies or sweep-profiles
typed today. Exit gate (Issue 4.7): wire body round-trips arena IO; sweeps
accept it as a profile.

## Serialization

Stage 1 adds `body_class` as an optional field on arena-v3 shell and wire
records. The field is omitted for the default class, so old documents load to
the default class and re-saving untouched default-class entities stays
byte-identical. A sheet-tagged shell cannot be smuggled into a solid boundary:
loading refuses transactionally. Standalone sheet and wire root records are a
new root shape, not an additive tag; Stage 2 / Stage 7 must introduce those
under arena schema v4 (or a separately versioned body document) and provide
their dedicated parsers. Journal, attribute, and persistent-reference
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
- **Can a compound be an operand of a boolean in v1?** No — callers flatten
  first (`explode`, then per-piece operations or `fuse_n`); compound
  operands are revisited with the cell complex.
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
