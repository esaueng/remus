# Persistent evolution audit: 6.5 and B18

Current work selection and status: [6.5](roadmap.md#p-6-5) and [B18](roadmap.md#b18). This source-pinned audit provides evidence and candidate slices; the master roadmap owns their priority and disposition.

Source baseline: `9e0f6c1437f66ee9ecd88c38026fcd92885f4a06`, reviewed
2026-09-09 UTC. This inventory reconciles merged implementation with
[P-Class 6.5](p-class-status.md) and [bridge B18](roadmap.md).
It does not promote an entire operation family from bounded fixtures.

## Integration disposition

- [#307](https://github.com/esaueng/remus/pull/307) tangency qualification,
  [#308](https://github.com/esaueng/remus/pull/308) quarter-wall geometry,
  and [#312](https://github.com/esaueng/remus/pull/312) sphere/cylinder
  tessellation are merged. Their general geometry limitations still apply.
- [#338](https://github.com/esaueng/remus/pull/338) integrates #309–#311
  and #313–#324, including direct-edit entity history, healing composition,
  unify, sewing, inner-wire removal, vertex splitting, and wireframe repair.
  These are no longer pending integration slices.
- [#340](https://github.com/esaueng/remus/pull/340) refreshes the two
  committed packages. Package integration is not consumer deployment proof.

## Evidence inventory

F/E/V below means faces, edges, and vertices. A face map alone does not
prove boundary history. Nor does a correct explicit unresolved event prove
that an original reference remains Bound. Test paths are repository-relative.

| Family | Merged construction history | Evidence | Remaining gate |
|---|---|---|---|
| Exact booleans | F/E/V events from GFA, scoped over both inputs and output | `crates/operations/tests/journal.rs`: `journaled_boolean_records_total_construction_history`; `regress_evolution_completeness.rs` in the same directory | Broader geometry qualification; native whole-call rollback now covered by the continuation regressions below |
| Planar/coaxial-bore move and support replacement | F/E/V construction maps; replacement direct/batch API | `crates/operations/tests/qualify_replace_surface.rs`: `journaled_replacement_keeps_all_references_through_tilt_and_bore_resize`; `crates/wasm/src/bindings/evolution.rs` move/replacement contracts | Rotation, lateral moves, surface-type changes, and unrestricted supports remain unqualified |
| Move through analytic blends | Copy maps or uniquely proven boundary incidence; periodic ambiguity remains unresolved | `crates/operations/tests/qualify_move_faces_generalized.rs`: `blended_cap_moves_preserve_every_entity_reference_and_incidence` | Ambiguous periodic/complex boundaries; this is not general blend-resize history |
| Cylindrical radius edits | Qualified construction maps through `resize_cylindrical_face_journaled` | `crates/wasm/src/bindings/evolution.rs` radius direct/batch contracts; `scripts/openzcad-wasm-consumer-regressions.mjs` | General partial walls and topology-changing correspondences |
| Planar draft | Unique boundary correspondence and face evolution; whole-call transaction | `crates/operations/tests/journal.rs`: `successive_draft_preserves_all_original_entity_references`, `failed_draft_does_not_publish_a_preexisting_mutation_gap` | Curved draft and ambiguous correspondence; large bored-box STEP sampling refusal remains |
| Defeature | Copy, merge, deletion, contact and seam maps for qualified reconstructions | `crates/operations/tests/journal.rs`: capped, extended, closed-rim and cylinder/cone defeature tests | General wounds, ambiguous refinement, and seam correspondences |
| Verified fix and healing pipelines | Replacement composition; unsupported/retired mappings remain unresolved rather than stale or falsely deleted | `crates/operations/tests/journal.rs` verified healing/pipeline tests; `crates/operations/src/journal_ops.rs` healing-history unit tests | Total correspondence for every fixer, all defect classes and custom operators is not established |
| Unify / healing sewing / inner-wire removal | Qualified merge, replacement and consumed-entity history through healing pipelines | `crates/operations/tests/journal.rs`: `verified_unification_journals_merged_faces_and_consumed_center`, `verified_sewing_preserves_all_references_through_arena_and_later_draft`, `verified_inner_wire_removal_deletes_consumed_references_and_preserves_survivors` | General standalone sewing and every upgrade variant; pipeline evidence does not certify all construction APIs |
| Fillet / chamfer creation | Faces only, with unresolved output claims retained | `crates/operations/tests/journal.rs`: `blend_face_evolution_journals_with_unresolved_claims_intact`; WASM `chamfer_journaled_severs_edge_refs_like_any_faces_only_entry` | Edge/vertex construction maps |
| Analytic blend-band resize | Journaled single-cylinder/planar-support path retains F/E/V on resize and records complete merges/deletions on removal; legacy resize remains faces-only | `crates/operations/tests/journal_resize_blend.rs`; WASM `blend_resize_history_has_direct_batch_parity_and_rollback`; packaged consumer regression | Multi-face regions, curved supports and ambiguous boundaries remain outside the journaled path |
| Linear pattern | Total F/E/V construction lineage for the journaled linear path: original Modified-into-itself, copies Generated from same-kind copy-time source, zero Preserved/Deleted/Unresolved, Construction origin; legacy face map and material results unchanged | `crates/operations/tests/regress_pattern_evolution_fev.rs` (census, original-only naming, arena round-trip/restore/subsequent edit, scales/curved/cavity/placement, typed refusals); WASM `linear_pattern_journaled_resolves_all_kinds_direct_and_batch`; `crates/operations/src/pattern.rs` `PatternTracker`/`linear_pattern_with_entity_history`, `journal_ops.rs` `linear_pattern_journaled` | Grid pattern; broader blend/shell/split E/V beyond their faces-only scope |
| Circular pattern | Total F/E/V construction lineage for the journaled circular path: original Modified-into-itself, copies Generated from same-kind copy-time source, zero Preserved/Deleted/Unresolved, Construction origin; legacy face map, minimum count 2, origin-based axis and material results unchanged | `crates/operations/tests/regress_circular_pattern_evolution_fev.rs` (census 2/3/6, disjoint/touching, box/cylinder/hollow-box at 1e-3/1/1e3, consistent rotation, rotated positions/carriers, original-only naming, arena round-trip/restore/subsequent edit, typed refusals); WASM `circular_pattern_journaled_resolves_all_kinds_direct_and_batch`; `crates/operations/src/pattern.rs` `circular_pattern_with_entity_history`, `journal_ops.rs` `circular_pattern_journaled` | Grid pattern; broader blend/shell/split E/V beyond their faces-only scope |
| Default V2 offset | F/E/V: one-to-one construction face map, plus edge/vertex claims induced from it by exact incidence and checked against the actual result sets; shared incidence stays typed unresolved | `crates/operations/tests/qualify_offset_entity_evolution.rs`; `crates/operations/tests/journal.rs`: `journaled_offsets_carry_face_references_through_exact_evolution`; WASM `offset_journaled_typed_unresolved_survives_every_envelope` | Torus seams and sphere equator rings need engine-level edge records; arc-joint and self-intersection-removal provenance; curved offsets refuse from 100 units (B55) |
| Shell / plane split | Face maps, including explicitly unresolved generated caps/rims | `crates/operations/tests/qualify_evolution_coverage.rs` | Edge/vertex maps; whole-call rollback repaired by this audit's regression slice |
| Extrude / revolve / sweep / loft / section | No family-wide total journal coverage established by this audit | Construction modules in `crates/operations/src/` | Construction attribution for caps, side faces and boundary entities; one family per slice |

`record_barrier_over_solid` has no production operation-wrapper callers in
this baseline. Its production caller is the explicit WASM naming barrier
API (`crates/wasm/src/bindings/naming.rs`). Therefore a zero-wrapper-call
search does **not** close B18: faces-only wrappers and unjournaled operations
still leave unresolved reference paths.

## Confirmed rollback correction

The native `shell_journaled` and `split_journaled` wrappers opened a journal
scope before invoking the transactional geometry operation. If an earlier
unjournaled edit was outstanding, even an invalid shell thickness or zero
split normal published an `unjournaled_mutations` global barrier on refusal.
The geometry transaction started too late to restore that journal change.

The wrappers now enclose scope creation, geometry and history recording in
one transaction. `failed_journaled_shell_preserves_unpublished_history` and
`failed_journaled_split_preserves_unpublished_history` in
`crates/operations/tests/journal.rs` fail on the baseline with
`refusal published history`. They also check geometric refusal, preserved
live entity counts and journal indices, and publication of the outstanding gap
exactly once on the next successful operation. Op IDs retain their monotonic
high-water semantics. This repair adds no boundary-provenance claim. It adds an outer topology
snapshot to each native wrapper; the existing geometry transaction remains.
Large-arena snapshot cost is not benchmarked by this qualification slice.

## Boolean and pattern rollback continuation

The same unpublished-history defect reproduced in `boolean_journaled` and
`linear_pattern_journaled` on `e49ef5ba`. A retired boolean operand, a disjoint
intersection, invalid pattern spacing, and overlapping pattern instances all
refused after publishing a global history barrier. The overlapping pattern
allocates copies before its geometry-level transaction refuses and retires them.

Both wrappers now enclose scope creation, geometry and history recording in one
transaction. `begin_scoped` also collects and validates every operand scope
before calling `journal_begin`, so a retired operand cannot publish a barrier
or consume an operation ID even when the helper is called directly.

Three regressions in `crates/operations/tests/journal.rs` pin this extension:
`invalid_scope_does_not_publish_a_mutation_gap`,
`refused_journaled_boolean_preserves_history_and_operands`, and
`refused_journaled_pattern_preserves_history_after_copying`. They reproduce
failures on the previous source and check unpublished history, live topology
counts including compounds/loops/coedges, unchanged STEP geometry,
retained and retired handles, operand volume, and the next successful
operation's single gap publication. The pattern
case requires allocated-slot growth to prove refusal happened after copying;
rolled-back slots remain retired. The two wrappers add an outer snapshot;
large-arena overhead remains unbenchmarked. Boolean evolution remains F/E/V,
and pattern evolution remains faces-only.

## Single cylindrical blend resize history

`resize_blend_journaled` and the additive `resizeBlendJournaled` direct/batch
WASM API qualify one cylindrical band between two planar supports, at positive
radii above the kernel linear tolerance, or zero to remove the band. The builder's generating support pair
must identify exactly one successor band. The complete face map must then induce
one edge/vertex incidence isomorphism, including wire cycles; ambiguous maps
refuse rather than bind a guessed entity. Journal setup, reconstruction and
recording form one transaction. Existing `resizeBlend` and
`resizeBlendWithEvolution` retain their broader geometry and face-map contracts.

`crates/operations/tests/journal_resize_blend.rs` covers rotated/translated bodies
at scales 0.1, 1 and 10, native and STEP-imported inputs, repeated enlargement,
shrinkage and equal-radius copies, all original F/E/V references, analytic face
counts/radii, closed-form volume, validation, watertight meshes, STEP round trips,
arena remapping and a subsequent edit. Refusals preserve unpublished history and
operand STEP geometry. A toroidal band on curved support refuses journaled resize
while its legacy geometry-only resize remains available. Direct/batch parity,
reference resolution and rollback are also exercised in native WASM contracts and
`scripts/openzcad-wasm-consumer-regressions.mjs` against packaged WASM.

Zero-radius removal uses the sharp reconstruction's boundary history. Every source
boundary must have exactly one construction disposition (successor or deletion),
and all result boundaries must be covered with matching entity kinds. Surviving
faces must map bijectively. Missing or ambiguous records refuse atomically.
The single-band box fixture records deletion of the band face and two arc edges;
merged vertices and edges retain all incoming references. Native/STEP-imported
fixtures at scales 0.1, 1 and 10 qualify resize-then-remove, analytic box geometry,
unchanged operands, arena remap and a subsequent draft edit. The direct/batch
WASM and installed-package contracts also exercise removal history.

This is a bounded 6.5/B18 extension, not general blend-history closure. Multiple
bands/corners, curved supports and ambiguous periodic boundaries need separate
attribution. The wrapper adds a whole-topology snapshot; large-arena
performance is not qualified here.

## Default V2 offset boundary history (B18, 2026-09-25)

This slice closes the default V2 offset family: every result face, edge and
vertex is attributed or listed as unresolved with a typed reason, and the
claim is checked against the actual result entity sets. Offset geometry is
unchanged.

The public producers and consumers traced for this family are these:

- **Native producers.** `offset_solid_v2_with_evolution` (faces only,
  unchanged) and the new `offset_solid_v2_with_entity_evolution`.
- **Native journal wrappers.** `offset_journaled` and the new
  `offset_journaled_with_entities`. The first delegates to the second and
  now journals edges and vertices as well as faces.
- **WASM.** `offsetJournaled` on the direct binding, `executeBatch` and
  `executeBatchV2`. It adds an `evolution` field with every event, typed
  reasons and the completeness report. OpenZCAD's adapter calls
  `offsetSolidV2`, whose contract is unchanged.

`operations::boundary_evolution` induces the boundary claims. A result
edge's face-use multiset, or a result vertex's face set, is mapped back
through the exact `modified` claims. The result is compared with the source
solid's own incidence, captured before the offset runs:

| Source entities with that incidence | Claim |
|---|---|
| Exactly one | `Modified` from it; several result pieces naming one source form a split |
| None | `Generated` from the source faces that now meet |
| Several | `Unresolved`, `ambiguous_incidence`, naming the candidates |
| An incident face without one `modified` source | `Unresolved`, `unmapped_incident_face` |

No coordinate or tolerance is consulted. The journal keeps the candidates;
the typed reason lives in the native result and the WASM envelope.
`EntityCompletenessReport` extends the 2026-09-21 face-only checker to all
three kinds. The producer refuses, and rolls back, any history that omits a
result entity or claims one outside the result.

Evidence is in `crates/operations/tests/qualify_offset_entity_evolution.rs`:

- **Resolved primitives.** Box at 0.001, 1 and 1000 units; cylinder and
  cone at 0.001, 1 and 10 units. Each runs outward and inward, at the
  origin and under a rigid placement. Every claim is fully resolved and
  bijective, and matches an independent oracle. Ordinary edges and vertices
  must be the strictly nearest source entity. Seams are checked by role,
  because the engine rebuilds a cylinder seam at its own circle start (90°
  from the source seam).
- **Incomplete records.** A dropped face, edge or vertex record and a
  phantom edge claim are each reported per kind and refused.
- **Split and merge.** A split edge binds `BoundMany` over both pieces, and
  each merged source binds the merged edge. Dropping one piece's record
  reports exactly that piece.
- **Typed unresolved.** Torus seams at three scales and both signs, and the
  sphere's equator ring, stay `ambiguous_incidence` and fail closed on
  reference resolution. The torus vertex still binds.
- **Journal.** Edge and vertex references bind the claimed successor
  through a second offset. This test fails with faces-only recording.
- **Geometry and refusal.** The history path matches the plain offset bit
  for bit. A refused offset is atomic and publishes its outstanding gap
  exactly once on the next success.

The WASM contracts check that the torus's typed record reaches JS
identically through the direct binding, `executeBatch` and
`executeBatchV2`. They also check that a refused offset is a typed
`executeBatchV2` error that publishes no history.
`scripts/test-wasm-smoke.mjs` repeats the direct and batch checks against the
built package.

The slice also found a defect outside history. Intersection-joint offsets
of cylinders and cones refuse from 100 units up with "no reconstructed wire
loops", while boxes offset at every scale. The plain offset fails
identically, so this change did not cause it. It is pinned as bridge row
B55 by the ignored ready-repro in
`crates/offset/tests/regress_curved_offset_scale.rs`.

## Hosted proof snapshot

These are historical run results, not proof of the next PR's head. Refresh
before merge or promotion.

| Evidence | Observed result | Disposition |
|---|---|---|
| [#338 CI](https://github.com/esaueng/remus/actions/runs/34295505645) | Required CI Pass succeeded on the integration PR | Evidence for that reviewed integration head |
| [Latest baseline CI](https://github.com/esaueng/remus/actions/runs/34298364885) | Pending when inspected | Not a green exact-baseline claim |
| [#339 CI](https://github.com/esaueng/remus/actions/runs/34297929998) | Classify Changes, Repository Policy, Secrets Scan and CI Pass refused with `PR identity or merge parents no longer match` after the PR merged | Runner authorization refusal, not an observed geometry-test failure; do not weaken the guard |
| [Corpus Gauntlet](https://github.com/esaueng/remus/actions/runs/34202367356) | Success at `30536842` | Older-source proof only |
| [Fuzz Smoke](https://github.com/esaueng/remus/actions/runs/34020633257) | Success at `cfb5c29e` | Older-source proof only |
| [Mutation Testing](https://github.com/esaueng/remus/actions/runs/34017122331) | Failed summary; archived report contains nine missed mutants | Open B19/testing-strategy work; not evidence of nine confirmed product defects |

The archived mutation survivors cover healing totals, sphere-loop projected
area, rolling-ball and variable fillets, loft band selection, wire/surface
alignment, NURBS pole-cap boundary comparison, and two benchmark surface
helpers. Replay each against current source before classifying it as a
missing assertion, equivalent mutant, or implementation defect. Source line
numbers in the archived report belong to `cfb5c29e`.

## Pre-existing tooling failure

`cargo test --manifest-path xtask/Cargo.toml` reports 20 passed and one
failure: `consumer_workflows_cannot_skip_wasm_optimization`, with
`.github/workflows/ci.yml must use the validated package builder`.
The test and workflow are unchanged from the source baseline. The workflow
now delegates to `fleet-ci.yml` at
`6da8b6a0777bd3e4e9248397626dce0636dce4c7`; that pinned implementation does
run `cargo xtask wasm-build`. The test still expects the command inline in
the caller. Track this under Open Kernel O4.2's release-tooling readiness:
qualify the pinned delegation graph and preserve the optimization assertion.
This slice neither bypasses the failing test nor changes CI routing.

## Next bounded slices

1. **Done for the covered wrappers:** native boolean/pattern refusal and scope
   preflight now preserve unpublished history, including pattern copy rollback.
   The regressions above carry the evidence; this does not expand geometry or
   provenance coverage.
2. **Done for one bounded case:** journaled positive-radius resizing of a single
   cylindrical band between planar supports, including zero-radius removal with
   complete merge/deletion history. Extend multi-face regions and curved supports
   only with construction boundary correspondence.
3. Extend one B18 family at a time. **Done: default V2 offset F/E/V
   (2026-09-25, above). Done: journaled linear-pattern F/E/V (this slice,
   above). Done: journaled circular-pattern F/E/V (this slice, above).**
   Remaining, in order: shell boundary maps
   (`shell_op` rebuilds both skins and rims from polygon specs, so it needs
   its own spec-to-edge records), split/section boundaries, then sweep-family
   cap attribution. Face-only entries that still sever edge and vertex
   references are fillet/chamfer creation, grid pattern, shell and
   plane split. Offset residuals are torus seams and sphere equator rings, which
   need edge records from the offset engine's intersection phase, plus
   arc-joint and self-intersection-removal provenance. Preserve explicit
   unresolved records outside each qualified domain.
4. Run 2.4d's merged quadric integration matrix and reconcile its existing
   exit gate before attempting broader arrangements. Continue the 2.6/2.7
   named scale and tangency gaps separately.
5. Prove an imported-body edit chain with reference resolution, arena remap,
   direct/batch WASM parity and STEP geometry round trips. Full 6.5 closure
   still requires every supported 6.x operation's qualified fixture to retain
   its promised references; B18 additionally covers the other families above.
