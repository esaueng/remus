---
name: heal
description: Diagnose and apply shape healing in remus-heal (analysis, fix, upgrade, sewing, unify). Use when fix_shape errors or destroys valid geometry, when unify_same_domain merges wrongly or orphans edges, when a shell stays open after sewing, when caps or cavity faces vanish after healing, or when choosing between the heal crate and the operations-layer heal wrappers.
---

# Heal (remus-heal crate)

Two heal implementations exist: `remus_heal::fix::fix_shape` (`crates/heal/src/fix/mod.rs`) and a separate `operations::heal` (`crates/operations/src/heal.rs`). Both carried their own copy of the same `Edge::new` trim defect (PR #103). Know which one your path calls.

## When to use

You are touching `crates/heal/**`, calling `fix_shape` / `unify_same_domain` / `sew_shell`, debugging a healing regression (`fix(...)` history under `crates/heal`), or deciding whether a defect belongs in analysis, fix, upgrade, or the verified wrappers (`fix_shape_verified`, `run_heal_pipeline_verified` in `crates/operations/src/heal.rs`).

## Quick reference

| Task | Entry point | Note |
|---|---|---|
| Heal a solid (record + atomically apply) | `fix_shape` | Full hierarchy, default path |
| Merge co-surface faces after a boolean | `unify_same_domain` | Optimization only; must never orphan edges (PR #1131) |
| Close a patch-built open shell | `sew_shell` | Rewrites wire uses, not endpoints (PR #94) |
| Locate the open loops first | `find_free_bounds` | Groups free edges into chains |
| Scripted repair sequence | `HealProcess::execute` | 13 builtins in `pipeline/builtin.rs` |

```rust
pub fn fix_shape(
    topo: &mut Topology,
    solid_id: SolidId,
    config: &FixConfig,
) -> Result<(SolidId, FixResult), HealError> {
```

```rust
pub fn unify_same_domain(
    topo: &mut Topology,
    solid_id: SolidId,
    options: &UnifyOptions,
) -> Result<(SolidId, UnifyResult), HealError> {
```

```rust
pub fn sew_shell(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<usize, HealError> {
```

```rust
pub fn find_free_bounds(topo: &Topology, shell_id: ShellId) -> Result<Vec<Vec<EdgeId>>, HealError> {
```

```rust
pub fn execute(
    &self,
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<(SolidId, Vec<FixResult>), HealError> {
```

`FixResult` (`crates/heal/src/fix/mod.rs`) carries counted `actions`, typed `refusals`, and `verification: FixVerification::NotPerformed`: L2 `Status::OK` means "no fixer action," never validity. Validity verdicts come only from the operations/check verified wrappers (PR #243).

## Architecture in ten lines

- `analysis/` takes `&Topology` and never mutates; `fix/` mutates via `HealContext` + `ReShape` atomic apply (`crates/heal/src/reshape.rs`).
- Fix hierarchy mirrors the B-Rep tree: `fix_shape` → `fix_solid` → `fix_shell` → `fix_face` → `fix_wire` → `fix_edge` (`crates/heal/src/fix/mod.rs`).
- Every fix type is gated by tri-state `FixMode::{Off, Auto, On}` in `fix/config.rs`; `Off` must short-circuit (commits `5ef57379`, `2a4c9476`).
- `upgrade/` holds the heavy passes: `unify_same_domain`, `shell_sewing`, `merge_split_rim_arcs`, `split_self_intersecting_wires`, `collapse_collinear_vertices`.
- `construct/` projects 3D curves to PCurves and converts analytic ↔ NURBS exactly; `custom/` walks whole solids for representation conversion.
- `pipeline/` wraps all of the above as 13 named `HealOperator`s executed in order by `HealProcess`.
- Solid-scoped passes must walk outer + inner shells via `explorer::solid_faces`, never `outer_shell()` alone (audit PRs #652/#656/#658/#659/#661/#663).
- Closed edges have zero chord: never derive size, degeneracy, or patch ranges from endpoints; sample along the curve.
- Vertex moves use `set_start`/`set_end`; `Edge::new` and `set_curve` reset the stored trim and tolerance (PR #103).
- Seam edges (one face, both senses) are declined with typed refusals, never force-repaired: the projector cannot tell u = 0 from u = 2π (commit `8b52ea7f`).

## Symptom-to-cause

| Symptom | Cause | Fix site |
|---|---|---|
| Bore/hole rim becomes a free edge after unify | Closed circle read as zero-length sliver by an endpoint filter; boundary loop dropped | `upgrade/unify_same_domain.rs`, PR #1129: sample mid-parameter, defer the group |
| Watertight shell gains thousands of free edges after unify | One-sided `merge_collinear_edges` rewrite; neighbor still references pre-merge edges | PR #1131: revert the phase that raises unpaired-edge count |
| `fix_shape` aborts on cylinder/cone/STEP import | Seam `(edge, face)` pcurve refusal escapes instead of declining | `fix/edge.rs`, commit `8b52ea7f`: oriented-pcurve API, FAIL1 decline |
| Cylinder caps vanish after heal (3 faces → 1) | Face size from vertex bbox collapses when start == end | `analysis/face.rs` + `fix/small_face.rs`, commit `8b52ea7f`: sample boundary curves |
| Healed volume wrong on oriented solids | Shell BFS compared raw wire senses, ignored `Face::is_reversed` | `fix/shell.rs`, commit `8b52ea7f`: compose the flag |
| Trims/tolerances lost on vertex merge | `Edge::new(start, end, curve)` to move endpoints | PR #103: `set_start`/`set_end` at all four sites |
| `sew_shell` reports sewn, shell still open | Rewrote edge j's endpoints; wires still reference distinct `EdgeId`s | `upgrade/shell_sewing.rs`, PR #94: redirect wire uses, flip sense, merge vertices, re-key pcurves |
| Cavity faces keep slivers/wires/NURBS after heal | Outer-shell-only walk | Audit PRs #652/#656/#658/#659/#661/#663, commit `f6eb833b`: `solid_faces` + per-shell guards |
| Same op, different topology across processes | `HashMap`-seeded merge-group order | PR #748: sort groups by minimum face index |

## Traps

- `convert_to_bspline` patch smaller than its trim: cap ranges from vertex positions (closed curve collapses) and cone axial-vs-generator `v_range` confusion; both misclassify point-in-solid by parity, not by distance (commit `7015faad`, test `between_sample_nurbs_bulge_stays_inside_converted_plane_patch`).
- Chord and arc share both endpoints: sew candidates must be interior-sampled, disagreements declined; a free edge with two valid partners is a non-manifold junction, left alone (PR #94, tests `sew_shell_declines_when_the_curves_between_shared_endpoints_disagree`, `sew_shell_declines_an_ambiguous_partner`).
- `fix_duplicate_faces` compares effective normals + same-winding boundaries, not centroid/normal/count; opposite-winding and opposite-normal coincident faces are preserved (PR #242, roadmap B11).
- Single-face periodic shells (full torus): `ReShape` declines a rewrite that would empty a retained shell; fail-closed, not corruption (PR #104).
- Per-shell "don't empty this shell" guards must stay per-shell; a globalized guard lets a degenerate cavity mask outer-shell repair (PR #656).
- `HealProcess::execute` never applies `ctx.reshape`: passes reachable through `sew_shells` must rewrite wires directly in `topo` (PR #94; precedent `merge_split_rim_arcs`).
- Un-merged split-rim arcs are a tessellation input class: a valid single-closed-circle cap can still mesh open (roadmap B34, open).

## Anti-patterns

- "Healing returned OK, so the solid is valid." OK means no fixer acted (PR #243). Validate independently; see solid-verification.
- "Endpoints coincide, so the edges share a curve." Chord-vs-arc disproves it; sample interiors (PR #94).
- "This face measures zero, so it is degenerate." Closed-curve faces measure zero from endpoints; sample the curve (#1129, `8b52ea7f`).
- "I walked `outer_shell()` and saw all faces." Cavity shells missed; use `explorer::solid_faces` (audit #652–#663).
- "The merge helper is safe to run on any shell." Unify is an optimization with a revert guard; check unpaired edges before/after (#1131).
- "Same `Edge::new` call, just new endpoints." Trim and tolerance are destroyed; use `set_start`/`set_end` (PR #103).

## Related skills

solid-verification (validity/volume oracles; the no-op and wrong-shape traps), boolean-debugging (booleans call heal pre-gate; unify feeds `compound_cut`), debugging-doctrine (vary-one-variable method), testing (regression fixtures; `qualify_heal.rs` matrix), io-formats (STEP-import healing), roadmap (B1/B11/B17/B34 status).
