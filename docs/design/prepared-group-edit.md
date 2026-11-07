# Prepared blend-group edits

Owner: W13 of the Gentle-Gecko video-parity plan (A8: live drag preview).
Tier: core — a prepared group edit is a session handle whose frames and
commits edit the live topology, and whose journaled commit writes the
evolution journal.

## Problem

Dragging a gizmo on a fillet band (A2) or a boss wall (A7) of a blended body
rebuilds the body through its blend group on every frame: recognize the
group, remove it to its sharp parent, edit the sharp body exactly, re-blend
the group in the planned order ([`blend_cluster/edit.rs`]). Unprepared, that
is 0.8–1.2 s per frame on the filleted Gentle-Gecko and 1.6–2.1 s on the
cumulative body (A2 → A3 → A4 → A6) natively with the WASM kernel's caches
on, and several times that in WASM.

Profiled (per-phase spans in `blend_group.*`, `perf` with frame pointers on
`examples/blend_group_edit_profile.rs`), the time is not in the blends:

| Phase (cumulative A7, native, kernel caches) | ms |
|---|---:|
| group recognition ×3 (route, removal re-check, identity carry) | ~210 |
| removal to sharp | ~75 |
| partial-turn attempt in front of the group route | 0–450 |
| five re-blend stages | ~1400 |
| — of which each stage's blend volume oracle and material guard | ~1380 |
| — of which the blend builds themselves | ~15 |
| group postconditions (strict validation, watertight mesh, volume) | ~100 |

The blend guards measure every stage's result volume on a mesh at the
measurement deflection (bbox × 5e-5); with sampled corner patches and walked
transitions that mesh is 250–350 ms per stage (`tessellate_nonplanar_cdt`,
`classify_dense_candidate`).

## API

```text
native  remus_operations::blend_cluster::PreparedGroupEdit
        prepare_band(topo, solid, face, expected_radius) / prepare_cylinder(topo, solid, face)
        preview(topo, value) -> SolidId          // a frame
        commit(topo, value) -> SolidId           // = resize_blend / resize_cylindrical_face
        is_current(topo), learned_refusal(), kind()
        remus_operations::journal_ops::commit_prepared_group_edit_journaled(topo, &prepared, value)
                                                 // = the *_journaled variants

wasm    prepareResizeBlend(solid, face, expectedRadius) -> handle
        prepareResizeCylindricalFace(solid, face) -> handle
        previewPreparedGroupEdit(handle, value) -> solid
        commitPreparedGroupEdit(handle, value) -> solid
        commitPreparedGroupEditJournaled(handle, value) -> "{solid, op}"
        releasePreparedGroupEdit(handle) -> bool
        (batch: previewPreparedGroupEdit, commitPreparedGroupEdit,
         commitPreparedGroupEditJournaled with {prepared, value})
```

`value` is the new radius (the band's, or the wall's: diameter / 2). A
kernel holds at most 16 live preparations; handles are never reused.

## Contract

- **Prepare** copies the solid into a private arena, recognizes the group
  there, removes it to sharp, recognizes it once more on the source for the
  identity carry, and learns whether the attempt in front of the group route
  refuses every value (below). It reads the caller's topology only. A face no
  group route edits is a typed `blend-cluster-not-prepared` refusal; a wrong
  witness is `blend-radius-mismatch`.
  Eligible preparations also warm the source and sharp parent's independent
  order-3/order-5 volume terms and, when the face mesh cache is enabled,
  postcondition-quality meshes. Successful terms are retained under complete
  face content, order and reference-point bits, within 4 MiB per handle.
  Changed faces and changed references miss; warming failures certify nothing.
- **Frame.** Clone the private arena, make the exact sharp edit (A7), plan
  and re-blend the group in the planned order with display-only blends
  ([`set_display_only_blends`]): each stage keeps its structural checks (every
  named edge rounded, closed outer shell, no validation error the input did
  not have) and skips the volume oracle and convex material guard. The
  finished body must have a closed outer shell and pass strict validation
  (the frame gate). The mesh and volume postconditions are left to the
  commit. The result is copied into the caller's arena with the group's face
  history translated and member identities carried, as the unprepared route
  does.
- **Commit** is the unprepared call, with only the group route served from
  the preparation: the per-band or partial-turn attempt runs first as it
  always does, then the group route under every guard (each stage's blend
  postconditions and the group's strict-validation, watertight-mesh and
  volume-agreement postconditions). Same result content and history, same
  refusals — plain and journaled.
  The last preview now retains its private candidate and the exact inputs of
  every omitted volume/material guard. When the value matches bit for bit,
  commit completes those guards and the group's mesh/volume postconditions,
  then copies the candidate into the current arena. No public preview handle
  is trusted; checkpoint restore or public-frame mutation cannot corrupt it.
  Successful checks are retained with the immutable candidate. A failed
  measuring guard falls back to the full route, which can select another engine.
  Capture is conservative: a failed attempt after a deferred guard invalidates
  adoption, and joined caps and groups containing chamfers still rebuild.
  Journaled commits keep the normal initial local attempt, begin, copy and
  record sequence, including operation-ID high-water behavior on rollback.
  Positive-weight, continuous degree-one-v NURBS spans integrate their
  straight generators analytically, retaining independent curved-u quadrature
  and the original convergence/boundary checks. Their generator-wise area
  lower bound makes closure qualification stricter and never answers an area
  property query. Unsupported cases keep the original integrator.
- **Learned refusal.** Preparation runs the attempt in front of the group
  route once on a scratch copy at a probe value. A refusal no value changes —
  the band's description or removal refuses (`BandNotAnalytic`,
  `BandTouchesFreeform`, `UnsupportedSupportPair`, the curved-end
  `Unsupported`), or the partial-turn move meets a neighbour it cannot
  re-intersect (`UnsupportedMoveFace { UnsupportedNeighbor }`) — and that the
  unprepared call would then answer through the group route is learned;
  frames skip the attempt. Otherwise every frame runs it first. A value the
  group route never takes (the current radius, a non-positive or non-finite
  value) runs the unprepared call.
- **Staleness.** As for prepared face moves: the source's content key is
  recorded, and every call refuses with `invalid input: prepared group edit
  is stale …` when it differs. A checkpoint restore that brings the same
  content back keeps the preparation valid. Previews never touch the source;
  cancel is `restore(checkpoint)` plus `releasePreparedGroupEdit`.

## Why display-only frames are safe to show

A frame can only differ from the commit where a measuring guard would refuse
or redirect a stage. Frames still refuse anything that is not a closed,
strictly valid B-rep, and on the reference sequences every frame equals the
unprepared edit (entity counts, census, volume to 1e-9) and meets the full
verification bar (`tests/prepared_group_edit.rs`). The commit completes every
measuring gate; a value whose commit refuses keeps the source. Structural
checks on an unchanged private candidate run once.

## Measured effect

See the PR for the before/after table (native criterion
`benches/prepared_group_edit.rs`, the profile example, and the WASM
`scripts/perf/prepared-group-edit-bench.mjs`).

## Adoption (OpenZCAD)

At drag start, after the checkpoint: `prepareResizeCylindricalFace(source,
wall)` (or `prepareResizeBlend(source, band, currentRadius)`). Per frame:
`restore(checkpoint)`, then `previewPreparedGroupEdit(handle, radius)`. On
release: `restore(checkpoint)`, `commitPreparedGroupEditJournaled(handle,
radius)`; on cancel: `restore(checkpoint)`. Always
`releasePreparedGroupEdit(handle)` at drag end. On a
`blend-cluster-not-prepared` refusal at prepare, drag with the unprepared
`resizeBlendJournaled` / `resizeCylindricalFaceJournaled`; on a stale
refusal, prepare again.

[`blend_cluster/edit.rs`]: ../../crates/operations/src/blend_cluster/edit.rs
[`set_display_only_blends`]: ../../crates/operations/src/blend_ops.rs
