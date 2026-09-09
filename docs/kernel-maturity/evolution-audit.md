# Persistent evolution audit: 6.5 and B18

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
| Analytic blend-band resize | New journaled single-cylinder/planar-support path retains F/E/V through uniquely proven construction correspondence; legacy resize remains faces-only | `crates/operations/tests/journal_resize_blend.rs`; WASM `blend_resize_history_has_direct_batch_parity_and_rollback`; packaged consumer regression | Removal, multi-face regions, curved supports and ambiguous boundaries remain outside the journaled path |
| Linear pattern | Face map over instances | `crates/operations/src/journal_ops.rs`: `linear_pattern_journaled` | Edge/vertex maps; native whole-call rollback now covered below |
| Default V2 offset | One-to-one face construction map | `crates/operations/tests/journal.rs`: `journaled_offsets_carry_face_references_through_exact_evolution` | Boundary maps, arc-joint and self-intersection-removal provenance |
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
radii above the kernel linear tolerance. The builder's generating support pair
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

This is a bounded 6.5/B18 extension, not general blend-history closure. Zero-radius
removal, multiple bands/corners, curved supports and ambiguous periodic boundaries
need separate attribution. The wrapper adds a whole-topology snapshot; large-arena
performance is not qualified here.

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
   cylindrical band between planar supports. Extend removal, multi-face regions
   and curved supports only with construction boundary correspondence.
3. Extend one B18 family at a time: shell/offset boundary maps, split/section
   boundaries, then sweep-family cap attribution. Preserve explicit unresolved
   records outside each qualified domain.
4. Run 2.4d's merged quadric integration matrix and reconcile its existing
   exit gate before attempting broader arrangements. Continue the 2.6/2.7
   named scale and tangency gaps separately.
5. Prove an imported-body edit chain with reference resolution, arena remap,
   direct/batch WASM parity and STEP geometry round trips. Full 6.5 closure
   still requires every supported 6.x operation's qualified fixture to retain
   its promised references; B18 additionally covers the other families above.
