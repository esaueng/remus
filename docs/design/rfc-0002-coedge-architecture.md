# RFC 0002: Coedge architecture

Status: accepted design; Stages 1–3 and the topology-owned atomic mutation
gate landed. The remaining boundary-authority flip is staged as P-Class
Issue 2.0.
Characterization anchors: `crates/topology/src/pcurve.rs`, module
`seam_characterization` — the flipped tests pin the landed per-use behavior
that the physical storage move must preserve.

## Problem

The original defect combined face boundaries stored as ordered
`Vec<OrientedEdge>` values (`wire.rs`) with p-curves keyed only by
`(EdgeId, FaceId)`. One 3D edge used twice by the same face — the seam of
every closed cylinder, cone, sphere, torus, and periodic NURBS surface —
therefore could not carry per-use data:

- the second seam p-curve **silently overwrote** the first
  (`PCurveRegistry::set` is a plain map insert);
- `pcurves_for_edge` reported one use where the face had two;
- there was no identity to hang per-use trim intervals, periodic-branch
  winding, or per-use tolerance on (needed by Issue 8's explicit trims and
  SameParameter validation).

Stage 2 closed that data-loss defect with the mutation-robust
`(edge, face, orientation)` key. The remaining problem is authority: wires
are still stored face-boundary state, p-curves still live in the registry,
and uncontrolled mutation can stale derived Loop/Coedge data. Seam-crossing
capability remains Partial until Issue 2.0 completes the physical authority
flip (`docs/kernel-maturity/capability-matrix.md`, cross-family limitation 1).

## Design

Two new arena entities in `remus-topology`:

```rust
/// One directed use of an edge by one face boundary.
pub struct Coedge {
    /// The underlying 3D edge.
    edge: EdgeId,
    /// Traversal orientation relative to the edge's natural direction.
    forward: bool,
    /// The loop this use belongs to (owner).
    parent_loop: LoopId,
    /// This use's 2D curve in the owning face's parameter space, with its
    /// own trim interval. `None` only where the surface type does not
    /// require a p-curve (planar faces may derive it).
    pcurve: Option<PCurve>,
}
pub type CoedgeId = Id<Coedge>;

/// An ordered, closed (or open, for future sheet boundaries) cycle of
/// coedge uses bounding one face.
pub struct Loop {
    /// The owning face.
    face: FaceId,
    /// Ordered traversal. Adjacent coedges connect end-vertex to
    /// start-vertex under their orientations.
    coedges: Vec<CoedgeId>,
    closed: bool,
}
pub type LoopId = Id<Loop>;
```

Held invariants (validator-enforced, see below):

- `coedge.parent_loop` and `loop.face` are always live; a coedge belongs to
  exactly one loop, a loop to exactly one face.
- Two coedges may reference the same `EdgeId`; a seam is exactly two uses on
  one face with opposite `forward` and different p-curve branches.
- P-curve identity is the coedge. `PCurveKey {edge, face}` remains only
  inside the compatibility layer.

Deliberately **not** in this RFC: explicit 3D edge trim intervals and
winding counts (Issue 8 adds them — the coedge is where they will live);
radial edge lists for non-manifold bodies (Milestone 8); generational
handles (separately versioned per `deferred-e6b`).

### Handle semantics

`CoedgeId`/`LoopId` are ordinary arena `Id<T>`s with the existing append-only
no-reuse tombstone contract. Retiring a face retires its loops and coedges;
retiring an edge with live coedges is a validation error (the reverse index
`edge → coedges` makes this checkable). Checkpoint restore treats the new
arenas exactly like the existing ones (high-water retirement).

WASM exposure: coedges are not exposed as public numeric handles in the
first release; JS callers keep face/wire/edge handles. Exposure (for
per-use queries) is an additive binding decision after Issue 7.

## Migration

The migration never has two authoritative representations. It has one
authority and one derived view, and flips them once.

### Stage 1 — additive entities (Issue 6)

- Add the `Coedge`/`Loop` arenas, constructors, traversal
  (`loops_of_face`, `coedges_of_edge`), and validators.
- **Wires remain authoritative** for face boundaries. Derivation is
  **explicit**: `Topology::build_face_loops(face)` derives and stores the
  face's loops (one coedge per `OrientedEdge` occurrence — a seam edge gets
  two coedges naturally), retiring any previous derivation. *Refined during
  implementation:* automatic derivation at face creation was dropped for
  Stage 1 because faces were mutated in place throughout L2/L3. Issue 2.0d
  later removed those production mutation paths behind the sanctioned gate
  below; automatic derivation still joins the physical authority flip.
- A consistency validator (`validate_face_loops`) asserts loop ↔ wire
  agreement; faces without a derivation pass vacuously, so it is safe in
  any validation pass today. Divergence is a bug, not a state.
- P-curves stay in the registry; each derived coedge that has a registry
  entry caches nothing yet (no dual storage of geometry).

Exit gate: every face constructed through public APIs has loops; the
seam-characterization face has **two** coedges for its seam edge; no
consumer behavior changes.

### Stage 1 mutation gate — topology-owned and atomic (Issue 2.0d; landed)

Wires remain the stored boundary authority until Issue 2.0e, but production
code no longer mutates that authority through exposed `Wire`/`Face` storage.
Two additive `Topology` methods are the sanctioned boundary:

- `replace_boundary_wire` preflights the replacement and every owning face,
  then replaces a shared/free wire, prunes pcurves for removed oriented uses,
  and re-derives any existing Loop/Coedge views;
- `set_face_boundary_wires` preflights the complete outer-plus-inner set and
  commits the face references, pcurve pruning, and derived-view replacement
  without exposing an intermediate boundary.

Both methods refuse stale wires or edges before mutation. They compose with
`run_transacted`: rollback restores the prior wire/face, pcurve registry, and
exact old Loop/Coedge handles, while handles allocated by the rolled-back
mutation remain retired under the arena high-water contract. The immutable
survey measured 30 production `wire_mut`, `inner_wires_mut`, and
`set_outer_wire` sites; the ratchet now requires zero. Test-only corruption
fixtures and `FaceSpec` construction remain explicitly outside that gate.

### Stage 2 — per-use p-curves (Issue 7; landed) and the authority flip

*Refined during implementation.* The boundary-mutation audit found exactly 30
uncontrolled in-place wire/face mutation sites across four production crates
(`wire_mut` rewrites, `inner_wires_mut`, `set_outer_wire`), so storing
p-curves inside stored `Coedge`s — whose derivations go stale on any such
mutation — would have required migrating every mutation site at once: the
big-bang this RFC forbids. Issue 7 therefore delivered the per-use
capability with a **mutation-robust key** instead: p-curve storage is keyed
by `(edge, face, orientation)`. A manifold face boundary cannot use one
edge twice in the same direction, so orientation fully identifies the use
(the two seam branches have opposite orientations), and — being identity-
rather than position-based — the key survives every in-place wire edit.

What landed:

- storage keyed per use; both seam branches retained independently;
- oriented accessors (`pcurve_oriented`, `set_pcurve_oriented`,
  `remove_pcurve_oriented`) address a use exactly;
- the `(edge, face)` accessors became the fail-closed adapter specified
  below (typed `seam_pcurve_ambiguous` when both branches exist);
- `Topology::coedge_pcurve` resolves a derived coedge's own branch;
- the boolean assembly p-curve pass is per-use (seam-capable);
- the seam characterization tests flipped as promised;
- the arena format records the orientation additively (older documents
  resolve the use from the face's wires on load).

The physical move of storage into `Coedge` and the boundary-sequence authority
flip below remain future work. The sanctioned atomic mutation prerequisite is
now landed; at the authority flip, the stored coedge becomes the key and this
registry becomes its index.

#### The remaining authority flip (original Stage 2 plan)

- Loops become authoritative. `Face` stores `outer_loop` + `inner_loops`;
  the wire the face was built from becomes an input artifact, not state.
- P-curves move into `Coedge.pcurve`. The `(edge, face)` registry API
  becomes a compatibility adapter:
  - `get(edge, face)`: answers only when the face has exactly **one** use
    of that edge; two uses return a typed ambiguity error
    (`invalid_topology` / `seam_pcurve_ambiguous` in the diagnostic
    registry) — the accessor fails closed instead of answering arbitrarily.
  - `set(edge, face)`: same rule; seam p-curves must be set per coedge.
  - This flips the characterization tests, which is the acceptance
    evidence for the stage.
- Compatibility adapters for readers:
  - `face_oriented_edges(topo, face) -> impl Iterator<Item = OrientedEdge>`
    derived from the loop (cheap: each coedge yields `(edge, forward)`);
  - `Face::outer_wire()` remains during the stage, backed by a wire
    materialized from the loop at mutation time, so `&[OrientedEdge]`
    slice-consumers keep compiling.
- Free wires (sweep paths, profiles, wire bodies) are untouched: `Wire`
  remains the representation for wires that are not face boundaries.

Exit gate: the seam face round-trips two independent p-curve branches;
`solid_faces`-based consumers pass unchanged through the adapters; the GFA
boolean suite, blend builders, and tessellation pass on loop-backed faces.

### Stage 3 — trims and SameParameter (Issue 8; carrier migration landed)

What landed:

- `Edge` stores an explicit trim interval `(t0, t1)` on its curve's
  parameterization; `Edge::domain_with_endpoints` prefers it and falls back
  to projection reconstruction. `set_curve` clears the trim (a trim is
  meaningful only on the parameterization it was recorded against);
  non-finite bounds are refused. Per-use p-curve trims already exist
  (`PCurve::t_start`/`t_end`, per-use since Stage 2).
- Trim writers: the GFA pave filler's split edges record the exact pave
  parameters (angular curves unwrap seam-wrapping spans by one period;
  `Line` sub-edges store none — a line re-anchors to its vertices); the
  builder's NURBS/arc split chains record their cut parameters; the
  vertex-weld remap, image canonicalization, and traversal-order rebuilds
  carry trims forward (flipping the span with a flipped vertex order).
- Trim carriers: solid copy, GFA store export, and the arena format
  (additive optional field) preserve stored trims.
- Boolean result assembly now carries exact sub-span trims through transient
  p-curve records, section-edge splitting, face rebuilds, vertex remaps, and
  analytic face construction. Coaxial cylinder/cone shortcuts emit explicit
  full-circle trims, and affine transforms retain or exactly remap them when
  an analytic curve's parameterization changes.
- High-traffic topology readers in boolean assembly, classification,
  validation, healing, blending, tessellation, measurement, and I/O use the
  edge-level accessor. Endpoint projection is now the fallback for raw
  construction and controlled import/healing paths, not the normal way those
  consumers recover an existing edge domain.
- `SameParameter`/`SameRange` validators (`check_*` reporting,
  `validate_*` enforcing) with the registry's first `tolerance_violation`
  codes (`same_parameter_exceeded`, `same_range_exceeded`); planar faces
  and missing p-curves pass vacuously, matching the check-crate convention.
- Controlled repair: `heal::fix::edge::repair_pcurve_within_budget`
  rebuilds a p-curve by projection **within a declared budget**, reports
  deviation before/after, and rolls the original back with a typed
  `RepairBudgetExceeded` when the budget cannot be met — never silent,
  never committing a miss.

Queued:

- Periodic winding counts on `Coedge` (multi-turn support) arrive with the
  physical p-curve storage move.

Landed prerequisite: Issue 2.0d supplies the topology-owned atomic mutation
boundary and exact checkpoint rollback that the remaining migration assumes.

The operations contribution in PR #122 to P-Class issue 2.0b closes seven of
the ten measured missing-writer paths: `merge_result_vertices`, the sphere-cap
and cylindrical-face spec arcs, the box-sphere octant arcs, and the cylinder,
pointed-cone, and frustum primitive rims. Primitive full turns are anchored at
each rim's actual seam parameter. The phase-FF follow-on closes all three
remaining paths: `perform_with_context`, `emit_exact_arc`, and
`emit_split_circle_arcs`. The ratchet now requires all 24 preservation writers
and reports zero remaining missing-writer paths.
Carrier hardening in the same contribution keeps fresh coaxial-cylinder and
coaxial-cone results on the primitive constructors' positive full-turn
contract, makes `copy_and_transform_solid` share `transform_edges`' exact
retention/remap policy, and clears malformed explicit domains from
endpoint-normalized Lines.
The coaxial-torus shortcut is exempt: `make_torus` builds the minimal CW
complex with degenerate seam lines and no circle edge to carry an interval.

### Migration ratchet

#### Measured completion baseline

The Issue 2.0 baseline is immutable `main` commit
`39c7a7b7ccbfc746ed7d9e9b8f156d54d6cfe090`. The script carries checked-in
identity manifests derived from that commit. Run
`scripts/check-edge-domain-authority.py --list` to list and classify the
identities on the **current HEAD** against those immutable manifests; the
table and line numbers below are the immutable baseline snapshot. The
default mode is the CI ratchet. It scans every whitespace-tolerant
`domain_with_endpoints (` token, including UFCS and non-public definitions.
An identity is its file, enclosing function, normalized local-context hash,
and duplicate ordinal. Current approved inventories may only decrease; any
unknown identity fails even if an old site was deleted and the total count
did not change. Tests/examples are baseline-explicit identities, not a path
or fixed-inline-module heuristic.

| Inventory | Baseline | Classification |
| --- | ---: | --- |
| Production `domain_with_endpoints` readers | 131 | Calls in production Rust, excluding the two method definitions, the one `Edge` compatibility fallback, and test/example callers. |
| Definitions | 2 | `EdgeCurve` and `Edge` accessors in `topology/src/edge.rs`. |
| Internal compatibility fallback | 1 | `Edge` falling back to `EdgeCurve` when no explicit trim is stored. |
| Test/example readers | 25 | Test modules, test targets, and examples; retained as characterization coverage. |
| Existing result-construction trim preservation | 12 sites / 10 logical paths | GFA pave splitting, builder rebuild/split/weld paths, and operations boolean assembly/shortcuts. These required identities are non-decreasing: deleting or locally rewriting one fails the ratchet. They are the positive baseline, not the missing-writer inventory. Carrier-only copy, transform, serialization, offset, and healing writes are separate. |
| Missing trim authority | 12 direct constructions + 1 snapshot omission / 10 logical paths | Immutable measured anchors, not automated detection. Three phase-FF emitters, two mixed-assembly branches, the box-sphere arc builder, merge-result-vertices snapshot/rebuild, and primitive cylinder/cone rims. A separate manually reviewed remaining-path manifest is reduced only as fixes and their oracles land. |
| Stored face-boundary mutation | 30 → 0 required | Immutable production survey of `wire_mut`, `inner_wires_mut`, and `set_outer_wire`; all sites migrated to the two sanctioned `Topology` methods. Test mutations and `FaceSpec` mutation remain excluded. |

RFC 0004 Stage 1 later added one production vertex-ball reader under the same
identity ratchet, raising the checked ceiling from the immutable 131-site
snapshot to 132 without changing that baseline inventory.

The 131 production readers, grouped by source file with exact baseline line
numbers, are:

| Source | Count | Lines |
| --- | ---: | --- |
| `crates/algo/src/builder/builder_solid.rs` | 2 | 2469, 2589 |
| `crates/algo/src/builder/face_splitter/edge_splitting.rs` | 1 | 301 |
| `crates/algo/src/builder/fill_images_faces.rs` | 8 | 1875, 2096, 2245, 2261, 2381, 2400, 2981, 3861 |
| `crates/algo/src/builder/mod.rs` | 5 | 272, 1087, 1195, 1236, 1283 |
| `crates/algo/src/builder/pcurve_compute.rs` | 2 | 234, 287 |
| `crates/algo/src/builder/split_types.rs` | 4 | 63, 74, 77, 228 |
| `crates/algo/src/classifier/mod.rs` | 1 | 421 |
| `crates/algo/src/classifier/ray_cast.rs` | 2 | 443, 1275 |
| `crates/algo/src/diagnostic.rs` | 1 | 137 |
| `crates/algo/src/pave_filler/mod.rs` | 2 | 142, 330 |
| `crates/algo/src/pave_filler/phase_ee.rs` | 1 | 115 |
| `crates/algo/src/pave_filler/phase_ef.rs` | 2 | 167, 320 |
| `crates/algo/src/pave_filler/phase_ff.rs` | 10 | 1062, 1289, 2275, 2350, 3267, 3340, 3408, 3444, 4566, 4985 |
| `crates/algo/src/pave_filler/phase_ff_coplanar.rs` | 2 | 128, 402 |
| `crates/algo/src/pave_filler/phase_ve.rs` | 2 | 112, 168 |
| `crates/blend/src/builder_utils.rs` | 2 | 502, 595 |
| `crates/blend/src/spine.rs` | 2 | 182, 202 |
| `crates/check/src/properties/face_integrator.rs` | 5 | 280, 1205, 1234, 1356, 1364 |
| `crates/check/src/util.rs` | 1 | 267 |
| `crates/check/src/validate/edge.rs` | 1 | 163 |
| `crates/check/src/validate/finite.rs` | 1 | 60 |
| `crates/heal/src/analysis/edge.rs` | 2 | 111, 130 |
| `crates/heal/src/analysis/face.rs` | 1 | 67 |
| `crates/heal/src/analysis/wire.rs` | 2 | 100, 203 |
| `crates/heal/src/construct/project_curve.rs` | 1 | 109 |
| `crates/heal/src/custom/convert_to_bspline.rs` | 1 | 225 |
| `crates/heal/src/fix/edge.rs` | 1 | 312 |
| `crates/heal/src/fix/small_face.rs` | 1 | 126 |
| `crates/heal/src/fix/wire.rs` | 4 | 602, 794, 890, 1011 |
| `crates/heal/src/upgrade/merge_split_rim_arcs.rs` | 1 | 78 |
| `crates/heal/src/upgrade/shell_sewing.rs` | 2 | 280, 281 |
| `crates/heal/src/upgrade/unify_same_domain.rs` | 1 | 502 |
| `crates/io/src/step/reader.rs` | 1 | 1665 |
| `crates/operations/src/blend_ops.rs` | 3 | 152, 207, 371 |
| `crates/operations/src/boolean/mod.rs` | 2 | 385, 4701 |
| `crates/operations/src/extrude.rs` | 3 | 294, 517, 533 |
| `crates/operations/src/feature_recognition.rs` | 1 | 281 |
| `crates/operations/src/fillet/rolling_ball.rs` | 1 | 2690 |
| `crates/operations/src/heal.rs` | 1 | 1661 |
| `crates/operations/src/loft.rs` | 3 | 512, 513, 708 |
| `crates/operations/src/measure/bounding_box.rs` | 2 | 321, 784 |
| `crates/operations/src/measure/volume.rs` | 2 | 52, 1946 |
| `crates/operations/src/query.rs` | 2 | 247, 292 |
| `crates/operations/src/revolve.rs` | 3 | 56, 632, 1157 |
| `crates/operations/src/split.rs` | 3 | 462, 469, 1052 |
| `crates/operations/src/sweep.rs` | 1 | 458 |
| `crates/operations/src/tessellate/edge_sampling.rs` | 14 | 145, 162, 186, 207, 321, 395, 399, 435, 447, 463, 632, 663, 683, 705 |
| `crates/operations/src/tessellate/nonplanar.rs` | 1 | 841 |
| `crates/operations/src/tessellate/nurbs.rs` | 6 | 121, 325, 332, 339, 348, 353 |
| `crates/operations/src/tessellate/planar.rs` | 4 | 840, 871, 891, 911 |
| `crates/operations/src/tessellate/rim_chain.rs` | 1 | 104 |
| `crates/operations/src/tessellate/solid.rs` | 1 | 394 |
| `crates/topology/src/validation.rs` | 1 | 329 |
| `crates/wasm/src/bindings/batch.rs` | 1 | 583 |

The 12 **existing preservation writes** are in these 10 logical paths:

1. `pave_filler::make_split_edges::create_split_edge`;
2. `builder::fill_images_faces::fill_images_faces`;
3. `builder::fill_images_faces::rebuild_face_with_fresh_vertices`;
4. `builder::fill_images_faces::rebuild_face_with_cb_edges`;
5. `builder::fill_images_faces::instantiate_wire_edge` (two branch sites);
6. `builder::builder_solid::weld_coincident_vertices`;
7. `builder::builder_solid::split_arc_edges_at_collinear_vertices` (two split loops);
8. `operations::boolean::assembly::edge_with_trim`;
9. `operations::boolean::coaxial_cylinder_shortcut`; and
10. `operations::boolean::unify_coincident_boundary_edges`.

The missing-writer inventory is separate and immutable: it records what was
measured at the baseline; it does not claim to detect whether current code
has fixed an anchor. It has 12 direct edge-construction sites plus the
`merge_result_vertices` snapshot omission that feeds one of them. Snapshot
and rebuild are one logical loss path, so the inventory spans 10 logical
paths. The script's `REMAINING_MISSING_TRIM_PATHS` is the manually reduced
completion manifest reviewed alongside each fix and its oracle. Removing a
path is coupled to writer evidence: its exact new identity hashes must be
registered only in `FIXED_PATH_WRITER_IDENTITIES`. The ratchet derives the
required preservation set exactly as the immutable 12-site baseline union
those claims; fixed claims cannot reuse baseline writers or writers claimed
by another path. Per-path cardinalities are pinned — 1 each for the
three phase-FF paths, sphere-cap, cylinder-face, box-sphere, merge-result,
and pointed-cone paths; 2 each for cylinder rims and frustum rims. The
required preservation count is therefore `12 +` the weights of all fixed
paths, and every required identity must exist on current HEAD:

| Source anchor | Missing authority |
| --- | --- |
| `crates/algo/src/pave_filler/phase_ff.rs:876` | `perform_with_context` raw section edge |
| `crates/algo/src/pave_filler/phase_ff.rs:976` | `emit_exact_arc` |
| `crates/algo/src/pave_filler/phase_ff.rs:5193` | `emit_split_circle_arcs` |
| `crates/operations/src/boolean/assembly.rs:504` | mixed assembly `SphereCapFace` circle |
| `crates/operations/src/boolean/assembly.rs:615` | mixed assembly `CylindricalFace` circle |
| `crates/operations/src/boolean/mod.rs:2359` | box-sphere `build_arc_edge` |
| `crates/operations/src/boolean/mod.rs:4330` and `:4501` | `merge_result_vertices` omits trim in `FaceSnap`, then rebuilds without it |
| `crates/operations/src/primitives.rs:189` and `:190` | cylinder bottom/top rims |
| `crates/operations/src/primitives.rs:347` | pointed-cone rim |
| `crates/operations/src/primitives.rs:405` and `:406` | frustum bottom/top rims |

Issue 2.0 lands in seven independently reviewable stages:

1. **2.0a — measurement and ratchet:** land this immutable survey, CI
   ceiling, and program ledger.
2. **2.0b — missing writers:** close all 12 direct construction gaps and the
   `merge_result_vertices` snapshot omission; pin exact trim invariants,
   geometric oracles, boolean census, and result validators.
3. **2.0c — readers and seam validation:** migrate all 131 production
   readers to the authoritative contract, preserving reconstruction only in
   explicit import/healing adapters; make SameParameter/SameRange seam-safe
   and run them over boolean outputs.
4. **2.0d — atomic boundary mutation (landed):** topology-owned sanctioned
   boundary mutation replaces all 30 direct production sites; the ratchet
   requires zero and exact checkpoint rollback is pinned. This landed before
   any storage authority flip.
5. **2.0e — physical Loop/Coedge authority:** move boundary and p-curve
   authority into Loop/Coedge behind additive compatibility adapters, then
   flip the existing seam characterization tests.
6. **2.0f — STEP per-use round-trip:** map loop-positioned STEP p-curves to
   distinct coedge uses and pin deterministic write/read/write behavior.
7. **2.0g — integration and zero gate:** require zero production readers,
   run the boolean/corpus/WASM/rollback suites, remove obsolete facades, and
   update capability and stability evidence.

The first algorithm-reader continuation after the strict-domain foundation
migrates 18 stored `Edge` and already-carried section-domain reads. The checked
ratchet moves from 98 to 80 production readers without adding a reconstruction
fallback. The following PaveFiller continuation migrates all 19 readers in its
initialization, EE/EF/VE, FF, and coplanar-FF paths, moving the checked ratchet
from 80 to 61. It preflights complete edge sets before arena mutation, copies
stored lifted/reversed ranges verbatim, and keeps Lines on their intrinsic
`[0, 1]` domain. The operations/WASM continuation in PR #165 migrates all 24
non-tessellation operations readers and the final WASM batch reader, moving the
checked ratchet from 61 to 36. Public raw-profile compatibility is confined to
named, validated extrude/revolve adapters; stored topology and read-only
measurements fail closed. The following algorithm-transient continuation
removes all eight direct splitter support/complement readers, moving the
ratchet from 36 to 28. Exact evaluation and child construction use carried
native/traversal ranges; structural partition sampling stays behind one named
endpoint-reconstruction adapter because weld-shifted fitted endpoints define
that support interval until per-use Coedge authority lands. The final reader
continuation migrates the 27 tessellation readers and the later RFC 0004
vertex-ball reader, moving the checked ratchet from 28 to zero. Tessellation
now consumes stored lifted, major, reversed, and NURBS ranges directly; missing
curved authority fails closed. The closing 2.0c continuation preserves existing
oriented pcurves through the deep-copy path used by exact boolean shortcuts and
runs both strict validators over a real identity-intersection result in CI. The
fixture requires six visited boundary uses, exactly two stored cylinder-seam
branches, and exactly two proved uses; it also pins the analytic face census,
closed-form volume, and reverse-branch-only failure. It does not reconstruct
pcurves after the boolean or claim that general GFA assembly populates them.
That storage migration remains staged under physical Coedge authority. These
checks close 2.0c without making a vacuous zero-pcurve result look validated.

Once Issue 2.0e lands, new code must not construct face boundaries from raw
`OrientedEdge` lists. Enforcement:

- `Wire::new` stays public (free wires are legitimate); the ratchet is on
  the face constructors: the wire-taking `Face::new` becomes
  `#[deprecated]` in favor of the loop-taking constructor, and CI treats
  new deprecation warnings as errors (already implied by `-D warnings`).
- The adapter module carries a tracking comment and a deletion gate: the
  facade is removed when `rg` finds no `outer_wire()` consumers outside
  `remus-topology` and the deprecation has been through one release.

## Serialization

The arena/JSON BREP transfer format gains `loops` and `coedges` arrays under
a schema-version bump. Compatibility:

- Old documents (no loops): loader derives loops from wires exactly as the
  Stage 1 builder does — every legacy file remains loadable forever.
- New documents: loops are authoritative; a wire array is still written for
  old readers during a deprecation window, then dropped with a major schema
  bump.

Repro bundles (schema 1) are unaffected: they replay operations, not
serialized topology.

## STEP mapping

STEP's model already matches this design: an `EDGE_LOOP` of
`ORIENTED_EDGE`s where a seam edge legitimately appears twice, and per-use
2D geometry via `SURFACE_CURVE`/`PCURVE` associated geometry.

- **Reader** (after Issue 2.0f): a repeated oriented edge in an edge loop maps
  to two coedges; each `PCURVE` binds to its coedge by loop position, not by
  `(edge, face)`. Today's reader collapses these — the RFC 0002 fixture
  (write/read/write of the seam face) becomes an active I/O regression at
  Issue 2.0f, not before.
- **Writer**: emits one `ORIENTED_EDGE` per coedge and one per-use
  `PCURVE`. Deterministic entity ordering follows loop order.

## Validation additions

New structural checks (stable codes in the diagnostic registry):

| Check | Code | Category |
| --- | --- | --- |
| Coedge references retired edge/loop | `coedge_dangling_reference` | `invalid_topology` |
| Loop not connected under orientations | `loop_not_connected` | `invalid_topology` |
| Loop/wire divergence (Stage 1 only) | `loop_wire_mismatch` | `internal` |
| Seam uses without distinct p-curve branches (Stage 2+) | `seam_branch_missing` | `invalid_topology` |
| `(edge, face)` p-curve access on a seam (Stage 2+) | `seam_pcurve_ambiguous` | `invalid_topology` |

## Consequences

- **Cost**: `FaceSurface`/`EdgeCurve`-scale ripple. Face-boundary iteration
  appears throughout L2/L3; the adapters exist precisely so Stage 2 is a
  flip of authority, not a big-bang rewrite of ~100 consumers. Consumers
  migrate to loop traversal incrementally after the flip.
- **Memory**: one `Coedge` per boundary use (~40 bytes + p-curve). P-curves
  move rather than duplicate.
- **Unblocks**: periodic seams (booleans, blends, offsets on closed
  surfaces), Issue 8 trims/SameParameter, faithful STEP seam round-trips,
  and per-use evolution events in Milestone 5.

## Resolved questions

- One loop entity for both outer and inner boundaries (a face has one outer
  loop and zero-or-more inner loops); no separate hole type.
- Coedge stores `parent_loop`, loop stores `face` — reverse lookups are one
  hop, and a coedge cannot be shared between loops by construction.
- No `mate`/`partner` pointer between the two uses of a seam edge in v1;
  `coedges_of_edge` answers the query, and a stored pointer is one more
  invariant to break. Revisit only if profiling shows the lookup hot.
- Free wires keep `Wire`. Only face boundaries migrate.
