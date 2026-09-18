---
name: offset
description: Diagnose offset, shell (thick-solid), arc-joint, and move-face failures in remus-offset. Use when offset_solid or thick_solid returns an inside-out solid, refuses a valid case, loses analyticity, or when deciding whether a self-intersection, cavity, or move-face configuration is inside the qualified cell.
---

# Offset (solid offset engine)

Engine map: the 8-phase intersection pipeline is documented in `crates/offset/src/lib.rs`; it is reached via `operations/src/offset_v2.rs`, and both wasm `offsetSolid`/`offsetSolidV2` route to v2 (`crates/wasm/src/bindings/operations.rs`).

## When to use

You are working on `offset_solid`, `thick_solid`, arc-joint offsets, `move_faces` / `replace_surface`, or the `offsetSolid` / `shell` bindings, and the result is inside out, refused, non-analytic, or you must decide whether a fold, cavity, or face move is supported at all.

## Quick reference

| Entry | Signature (`crates/offset/src/`) | Use |
|---|---|---|
| `offset_solid` (`lib.rs`) | `pub fn offset_solid(topo: &mut Topology, solid: SolidId, distance: f64, options: OffsetOptions) -> Result<SolidId, OffsetError>` | Outward (+) / inward (−) full offset |
| `thick_solid` (`lib.rs`) | `pub fn thick_solid(topo: &mut Topology, solid: SolidId, distance: f64, exclude: &[FaceId], options: OffsetOptions) -> Result<SolidId, OffsetError>` | Hollow a solid; excluded faces stay, walls close the rim |
| `offset_solid_with_face_map` (`lib.rs`) | `pub fn offset_solid_with_face_map(topo: &mut Topology, solid: SolidId, distance: f64, options: OffsetOptions) -> Result<OffsetResult, OffsetError>` | Same build plus total 1:1 source→result face map; refuses Arc/SI-removal options |
| `move_faces` (`move_faces.rs`) | `pub fn move_faces(topo: &mut Topology, solid: SolidId, faces: &[FaceId], distance: f64) -> Result<SolidId, OffsetError>` | Move a coplanar planar group; supports stay fixed, edges rebuilt |
| `remove_folded_uniform_l_prism_region` (`self_int.rs`) | `pub fn remove_folded_uniform_l_prism_region(topo: &mut Topology, solid: SolidId, tolerance: f64, removable_faces: &[FaceId]) -> Result<SelfIntersectionRemoval, OffsetError>` | Excise one proven-collapsed L-prism fold; general folds refuse |

## Architecture in ten lines

- `analyse.rs`: convex/concave/tangent edge classes from dihedral angle, vertex classes derived; everything downstream keys off this.
- `offset.rs`: per-face surface offset (planes translate, cylinder/cone/sphere/torus radii adjust, NURBS interpolated on a 16×16 grid at degree 3).
- `inter3d.rs`: adjacent offset-face pairs intersected in 3D via analytic dispatch; seam edges skipped, rebuilt by loops.
- `inter2d.rs`: intersection samples become vertices/edges (Circle-certified arcs where fittable, else Line).
- `loops.rs`: trimmed edges cut at corners into one closed wire per face, wound to the face's own stored surface normal (PR #89).
- `assemble.rs`: shells rebuilt preserving the source partition (outer plus one inner shell per cavity); every shell must validate closed.
- `self_int.rs`: fail-closed fold detector; only the proven uniform-L cell excises (roadmap 5.7 Merged; 5.7b general removal Pending).
- `cavity.rs`: same signed distance along each shell's own outward normal (+ grows body, shrinks cavity); extent and survival checks refuse crossings.
- `arc_joint.rs`: separate Minkowski-ball construction for all-convex polyhedra, not a post-pass (refuses curved/concave/holed/cavity/excluded/inward).
- `move_faces.rs` (+`quarter_cylinder.rs`): topology-preserving planar moves and plane/coaxial-cylinder surface replacement; failures roll back transactionally.

## Symptom-to-cause

| Symptom | Cause | Fix commit |
|---|---|---|
| Shelled solid measures both skins added (10 mm cube +1 mm: 2584 not 584) | Loops wound offset wires to the effective normal and assemble flipped the offset skin; cavity is whichever skin ends up inside | `f98292b9` (PR #89) |
| Inward offset past half-thickness returns `Ok` inside-out (−6 on 10 mm box: 8 mm³; −1e6: 8e18) | Planes only translate, so no radius guard exists; caught by the operations-side negative-signed-volume postcondition, not the engine | `3a500279` (PR #86) |
| `offset(torus)` errors "no faces could be assembled" | Doubly-periodic seam wire (degenerate v0→v0 edges) defeats every generic loop strategy; rebuilt as a concentric torus directly | `a6200976` (fork #999); trimmed-patch over-application gated in `a248e007` (fork #1001) |
| `offsetSolidV2` volume off on planar bodies | Plane-plane joints used a 1% margin instead of exact endpoints | `be4dd757` (item #493) |
| Intermittent free edges / wrong volume across runs | HashMap iteration mixed direct and trimmed loop paths; pinned by a 64× repeat-volume regression | `88b25106` (`offset_box_rebuilds_are_manifold_and_exact`) |
| Non-closed or partial result returned as success | Now refused: `validate_offset_result` gates every shell on closed; cavity inputs were refused outright at the time | `f300a5a4` |

## Traps

- The check-crate validator has no shell-orientation check: an inverted offset passes `validate_solid`. Test inversion directly (`solid_is_inverted`, negative outer-shell signed volume), never validation alone (PR #86).
- `OffsetError::CollapsedSolid` exists but is never constructed; the live collapse refusal is `OperationsError::InvalidInput` from `ensure_not_collapsed` in `operations/src/offset_v2.rs` (verify: `grep -rn CollapsedSolid crates/`).
- `offset_solid_with_face_map` is total-or-refusal: Arc joints and SI removal synthesize/replace faces, so they return `InvalidInput`, never a partial map (`face_map_refuses_non_one_to_one_options_without_mutation`, roadmap B5 Done).
- Arc-joint inputs outside all-convex-planar (cylinder face, concave edge, holed face, cavity, excluded face, inward distance) refuse — do not "fall back" to mitre; the rounded body is strictly smaller (`a_curved_source_face_refuses_rather_than_mitres`, `an_inward_rounded_offset_refuses`).
- `thick_solid` on a body that already has a cavity refuses: the wall builder only knows the outer shell (`hollowing_a_solid_that_already_has_a_cavity_is_refused`).
- `remove_self_intersections` (the general entry) authorizes no deletion: any detected fold without L-prism provenance is a typed error, and `OffsetOptions` defaults it off (`self_int.rs`, roadmap 5.7 vs 5.7b).
- `move_faces` keeps the adjacency graph fixed: a move that collapses, splits, or rewires a face/wire/edge is `TopologyChange` with pre-call topology restored (`move_faces.rs`, `collapsed_or_intersecting_bore_moves_restore_every_temporary_entity`).
- Both wasm bindings (`offsetSolid`, `offsetSolidV2`) route to `offset_v2::offset_solid_v2`; the v1 engine was deleted (`f0a363b3`, fork #850). `shell` is exact-only with disclosed quality; the sampled `offset_face` `samples` knob is deprecated (roadmap B25 Done).

## Anti-patterns

- "Census says exact analytic, so the shell is right." The #89 inversion was census-exact its whole life. Assert wall volume and winding, not face counts.
- "Validation passed, so orientation is fine." See Traps 1.
- "The fold is obviously empty; pass the faces and delete." `removable_faces` must equal the complete folded component exactly or the call refuses (PR #237).
- "Retry an inward arc offset with mitre options." Different construction, different volume — pick the joint the drawing needs.
- "A refused offset mutated the solid." Every entry point rolls back (`run_transacted` / snapshot restore); assert arena counts before/after as the face-map test does.
- "Widen the general validator to catch orientation." Rejected in #86: per-op signed-volume postcondition instead, to avoid false refusals on curved offsets.

## Related skills

solid-verification (wall-volume and inversion oracles), boolean-debugging (offsets feed booleans; same fallback discipline), analytic-preservation (torus/cylinder radius paths), testing (a regression fixture per fix), wasm-bindings (`offsetSolid` routing, batch parity), roadmap (5.7/5.7b/B5/B25 states).
