# RFC 0007 Splitting `remus-operations` into L3 domain crates

Status: **design only, on hold.** No code moves in this RFC. Its PR carries
the `hold` label.

Baseline: `main` at `c91287d` on 2026-10-29. Measurements come from a 4-vCPU
cloud container with rustc 1.96.0 and a 30 GB disk allowance. Line numbers and
counts describe that commit. Any later edit can shift them, but not by enough
to change the conclusions.

## Summary

- **The module graph has one big cycle today.** 38 of the 60 top-level
  modules in `crates/operations/src` (counting the crate root) form a single
  strongly connected component in production code alone. With test code
  counted, it grows to 46. So no crate split works as-is.
- **Moving 12 items breaks the cycle.** These are small, self-contained
  helpers and two error/report types (see [Cycle breakers](#cycle-breakers)).
  After those moves, the production graph condenses to a DAG of seven crates
  plus the facade. The only cycles left fall inside a single proposed crate:
  `boolean`/`compound_ops`/`mesh_boolean`, and an 8-module blend/edit
  component.
- **The proposed crates form a chain, not a fan.** Bottom to top, the order is
  `remus-ops-core` → (`remus-transform` ∥ `remus-tessellate`) →
  `remus-measure` → `remus-construct` → `remus-boolean` → `remus-features` →
  `remus-operations` (facade). Clean-build parallelism is therefore modest.
  The real wins are:
  - splitting the 71 s unit-test binary;
  - incremental rebuilds after edits high in the stack;
  - per-crate mutation scope.
- **Tests are the main migration cost, not production code.** Most unit tests
  in the lower crates build their fixtures with `primitives`, `boolean` or
  `fillet`. For example, 14.3k lines in 12 of the 32 tessellate files reach
  upward. A lower crate's unit tests cannot depend on a higher crate. If they
  try, they hit the "two versions of the crate" trap from the
  `layer-boundaries` skill, now between L3 crates. Each such test has to
  become an integration test or move up a crate.
- **Stage 1 is `remus-ops-core` plus `remus-tessellate`.** Tessellate needs
  only core in production. A stage-0 prep PR inside the existing crate comes
  first.

## 1. Measured dependency graph

### Method

A script parsed every file reachable through `mod` declarations from
`crates/operations/src/lib.rs`: 152 files. Two `copy/tests/*` oracle files
that sit behind an inline test module were not reached; they are test-only.
For each file, the script:

1. Stripped comments and string literals.
2. Expanded every `use crate::…`, `use super::…` and `use self::…` statement,
   including nested `{…}` groups, and every qualified `crate::`/`super::`
   path outside `use`.
3. Resolved each path to a top-level module, or to `ROOT` for items defined
   in `lib.rs`.
4. Marked each reference as *test* if it sits in a file declared under
   `#[cfg(test)]` or inside a `#[cfg(test)]` item. Everything else counts as
   *prod*.

A second, independent regex pass checked the result. It looked for bare
`module::item` paths that point *up* the proposed order from production code.
It found only the facade's `pub use cap::nonplanar_ring_surface` and a false
positive: `boolean`'s own `assembly` submodule. The parser misses two
bare-path references in `lib.rs`, `heal::HealingReport` and
`resize_blend::ResizeBlendError`. Both were added by hand and both are in the
cycle-breaker list. The test/prod split is a heuristic: a few test modules
gated other than by `#[cfg(test)]` (for example `fillet/arc_torus.rs`'s
`inner_tangent_tests`) count as prod lines. That changes line totals by a few
percent and edges not at all, because their references are test-only.

### Result

| Graph | Nodes | Non-trivial SCCs | Largest SCC |
|---|---|---|---|
| Production references only | 60 (59 modules + `ROOT`) | 1 | 38 modules incl. `ROOT` |
| Production + test references | 61 (adds `test_helpers`) | 1 | 46 |

**Production SCC (38):** `ROOT`, `affine_blend_caps`, `blend_material`,
`blend_ops`, `boolean`, `boundary_evolution`, `cap`, `chamfer`, `classify`,
`compound_ops`, `copy`, `defeature`, `distance`, `draft`, `extrude`,
`fill_face`, `fillet`, `heal`, `journal_ops`, `local_wound`, `measure`,
`mesh_boolean`, `offset_face`, `offset_trim`, `offset_v2`, `pattern`,
`primitives`, `push_pull`, `query`, `remove_blends`, `replace_surface`,
`resize_blend`, `revolve`, `shell_op`, `split`, `tessellate`, `transform`,
`validate`.

The test-inclusive SCC adds `evolution`, `feature_recognition`, `loft`,
`mirror`, `performance`, `sew`, `test_helpers` and `winding`.

### Why it is one component

The cycle comes from a handful of hub edges, not from a general tangle:

- **`ROOT` → features.** `OperationsError` (in `lib.rs`) embeds
  `crate::heal::HealingReport` and has a `#[from]` arm for
  `resize_blend::ResizeBlendError`. Every module depends on `ROOT`, so these
  two fields alone put `heal` and `resize_blend` in a cycle with everything.
- **`boolean` as a utility library.** `boolean::face_polygon` and
  `wire_polygon` are used by `measure`, `validate`, `query`, `loft`, `sweep`,
  `offset_wire`, `section` and `shell_op`. The enclosure probes
  `component_encloses_point` and `any_vertex_of` are used by `validate`.
  `validate` is used by `heal` and `revolve`, while
  `boolean` itself uses `validate`, `heal`, `measure`, `primitives`, `copy`
  and `tessellate`.
- **`tessellate` → `fillet`.** One function,
  `fillet::face_surface_normal_with_grid` (in `fillet/geometry.rs`), is used
  by `tessellate`.
- **`mesh_boolean::triangles_intersect`** is used by `validate`.
- **`journal_ops::solid_entity_keys`** is used by `resize_blend` and
  `remove_blends`, while `journal_ops` drives every journaled operation.

Production edges per module at baseline (count = distinct reference sites),
for reference:

```
tessellate      -> ROOT(182) performance(6) fillet(1)
measure         -> ROOT(103) performance(7) boolean(3) tessellate(2) distance(1)
classify        -> ROOT(1) tessellate(4)
validate        -> ROOT(17) boolean(5) measure(2) tessellate(2) mesh_boolean(1)
heal            -> ROOT(63) validate(15) winding(1)
transform       -> ROOT(75)
copy            -> ROOT(42) transform(17)
primitives      -> ROOT(59) revolve(1)            # degenerate-cone fallback only
revolve         -> ROOT(71) boundary_evolution(6) cap(2) evolution(2) extrude(2) validate(1) winding(3)
boolean         -> ROOT(225) copy(35) measure(22) performance(12) heal(11) classify(9) tessellate(8)
                   primitives(7) transform(7) validate(6) evolution(4) compound_ops(3) mesh_boolean(3) distance(1) winding(1)
fillet          -> ROOT(90) boolean(20) blend_ops(18) query(14) heal(5) copy(2) validate(2) measure(1)
resize_blend    -> ROOT(5) performance(17) measure(13) copy(9) defeature(9) push_pull(5) transform(4) blend_ops(2) boolean(2) ...
journal_ops     -> boolean(9) push_pull(7) heal(4) pattern(3) performance(3) ... (19 modules)
```

### Cycle breakers

Moving these 12 items, and nothing else, turns the production graph into the
DAG below. Each new home re-exports the item from its old path, so
`remus_operations::…` paths do not change.

| # | Item | Today | Moves to | Edge it breaks | Visibility today |
|---|---|---|---|---|---|
| 1 | `HealingReport` (+ `HealingChange`) | `heal.rs` | `remus-ops-core` | `ROOT` → `heal` | `pub` |
| 2 | `ResizeBlendError` (+ its `code()`) | `resize_blend.rs` | `remus-ops-core` | `ROOT` → `resize_blend` | `pub` |
| 3 | `face_polygon` | `boolean/mod.rs:7591` | `remus-ops-core` | 8 modules → `boolean` | `pub` |
| 4 | `wire_polygon` | `boolean/mod.rs:7610` | `remus-ops-core` | `query`, `shell_op` → `boolean` | `pub` |
| 5 | `wire_polygon_closed_subloops` | `boolean/mod.rs:7664` | `remus-ops-core` | `shell_op` → `boolean` | `pub` |
| 6 | `component_encloses_point` | `boolean/mod.rs:6021` | `remus-measure` (`validate`) | `validate` → `boolean` | `pub(crate)` |
| 7 | `component_encloses_any_point` | `boolean/mod.rs` | `remus-measure` (`validate`) | `validate` → `boolean` | `pub(crate)` |
| 8 | `any_vertex_of` | `boolean/mod.rs:6124` | `remus-measure` (`validate`) | `validate` → `boolean` | `pub(crate)` |
| 9 | `COMPONENT_OVERLAP_MARGIN_MM` | `boolean/mod.rs:488` | `remus-measure` (`validate`) | `validate` → `boolean` | `pub(crate)` |
| 10 | `triangles_intersect` | `mesh_boolean.rs:455` | `remus-measure` (`validate`) | `validate` → `mesh_boolean` | `pub(crate)` |
| 11 | `face_surface_normal_with_grid` (+ private `nurbs_normal_at`) | `fillet/geometry.rs:222` | `remus-tessellate` | `tessellate` → `fillet` | `pub` |
| 12 | `solid_entity_keys` | `journal_ops.rs` | `remus-ops-core` (beside `evolution`) | `resize_blend`, `remove_blends` → `journal_ops` | `pub` |

Items 6–9 call only `tessellate` and topology. Item 3 calls only topology and
`OperationsError`. So none of the moves drags further dependencies down.

After the moves, recomputing SCCs over production edges leaves two cycles.
Both are wholly inside one proposed crate, which is allowed:

- `boolean` ↔ `compound_ops` ↔ `mesh_boolean` (`merge_disjoint_solids`,
  `mesh_boolean`, `BooleanOp`).
- `affine_blend_caps`, `blend_ops`, `chamfer`, `defeature`, `fillet`,
  `local_wound`, `push_pull`, `resize_blend`. This is the blend/direct-edit
  core (`fillet` ↔ `blend_ops`, `blend_ops` ↔ `resize_blend`,
  `resize_blend` ↔ `push_pull`, `resize_blend` ↔ `defeature`, …). Splitting
  it further would need real API redesign, such as an injected
  "fillet across earlier bands" callback. This RFC does not attempt that.

## 2. Proposed crates

### Boundaries

Assignment of all 59 modules. Line counts split each file's lines into
production and `#[cfg(test)]` regions, using the heuristic above.

| Order | Crate | Modules | Prod lines | Test lines | Workspace deps (production) |
|---|---|---|---|---|---|
| 0 | `remus-ops-core` | crate-root items (`OperationsError`, `authoritative_edge_domain`, `dot_normal_point`, `refuse_cavity_shells`, edge-domain helpers), `performance`, `winding`, `evolution`, `boundary_evolution` + breakers 1–5, 12 | 2,056 | 678 | math, topology, geometry, algo, blend, check, heal, offset (all through `OperationsError`'s `#[from]` arms) |
| 1a | `remus-transform` | `transform`, `copy`, `mirror` | 5,029 | 3,221 | ops-core, math, topology, geometry, heal |
| 1b | `remus-tessellate` | `tessellate` + breaker 11 | 19,347 | 12,581 | ops-core, math, topology, geometry, algo (`wall_chart`) |
| 2 | `remus-measure` | `measure`, `distance`, `classify`, `validate`, `query`, `feature_recognition`, `projection` + breakers 6–10 | 16,087 | 11,929 | ops-core, tessellate, math, topology, geometry, algo, check |
| 3 | `remus-construct` | `primitives`, `extrude`, `revolve`, `sweep`, `loft`, `pipe`, `helix`, `cap`, `fill_face`, `thicken` | 11,808 | 9,018 | ops-core, transform, measure, math, topology, geometry, check, heal |
| 4 | `remus-boolean` | `boolean`, `mesh_boolean`, `compound_ops`, `heal` (the operations-layer healing wrappers), `section`, `split`, `project_curve` | 28,305 | 19,041 | ops-core, transform, tessellate, measure, construct, math, topology, geometry, algo, check, heal |
| 5 | `remus-features` | `fillet`, `chamfer`, `blend_ops`, `blend_material`, `face_face_blend`, `resize_blend`, `affine_blend_caps`, `local_wound`, `remove_blends`, `defeature`, `push_pull`, `replace_surface`, `offset_face`, `offset_trim`, `offset_v2`, `shell_op`, `draft` | 32,618 | 14,975 | ops-core, transform, measure, construct, boolean, math, topology, geometry, algo, blend, check, offset |
| 6 | `remus-operations` (facade) | `journal_ops`, `prepared_move`, `imprint`, `pattern`, `assembly`, `sketch`, `linked_sketch`, `sew`, `untrim`, `offset_wire`; re-exports | 5,286 | 3,817 | all of the above + sketch |

Placement notes:

- **`boolean` depends on `construct`, not the other way round.** It calls
  `make_box`/`make_cylinder`/`make_cone`/`make_sphere`/`make_torus` in its
  primitive-pair shortcuts (`boolean/mod.rs:2588`–`3367`). `construct` needs
  only `face_polygon` from `boolean`, and that moves to core. `primitives`
  sits with `construct` because `make_cone`'s degenerate-angle fallback calls
  `revolve::revolve` (`primitives.rs:341`).
- **`heal` goes in `remus-boolean`.** Its only dependency is `validate`, so
  it could sit in `remus-measure`. But its callers are `boolean` and the
  layers above. Keeping it beside `boolean` keeps `remus-measure` read-only
  analysis. Either placement is a valid DAG.
- **`journal_ops`, `prepared_move` and `imprint` stay in the facade.**
  `journal_ops` is the hub that journals every operation family (19 outgoing
  module edges). `prepared_move` and `imprint` call into it.
- **The offset/shell/draft modules go in `remus-features`.** They are not part
  of the blend cycle and could become a seventh crate, `remus-offset-ops`,
  above `remus-boolean` and parallel to `remus-features`. Deferred: about 6k
  production lines is too little to pay for another crate now.

The resulting crate DAG (production edges, with distinct items referenced in
brackets):

```
remus-ops-core
  ├─ remus-transform ─────────────────────────────┐ [8]
  └─ remus-tessellate [6]                         │
        └─ remus-measure [8 tess, 28 core]        │
              └─ remus-construct [2 measure, 4 transform]
                    └─ remus-boolean [6 construct, 30 measure, 8 tess, 12 transform]
                          └─ remus-features [27 boolean, 57 measure, 2 construct, 21 transform]
                                └─ remus-operations (facade) [21 features, 14 boolean, ...]
```

### Facade re-exports

`remus-operations` keeps every current path by re-exporting each moved
module under its old name:

```rust
pub use remus_ops_core::{OperationsError, evolution, boundary_evolution, performance, winding};
pub use remus_transform::{copy, mirror, transform};
pub use remus_tessellate as tessellate;            // crate root = old `tessellate` module
pub use remus_measure::{classify, distance, feature_recognition, measure, projection, query, validate};
pub use remus_construct::{cap::nonplanar_ring_surface, extrude, fill_face, helix, loft, pipe, primitives, revolve, sweep, thicken};
pub use remus_boolean::{boolean, compound_ops, heal, mesh_boolean, project_curve, section, split};
pub use remus_features::{blend_ops, chamfer, defeature, draft, face_face_blend, fillet, offset_face,
                         offset_trim, offset_v2, push_pull, remove_blends, replace_surface, resize_blend, shell_op};
```

`winding` and `cap` are `pub(crate)` today. `winding` becomes `pub` in core,
because construct, boolean, features and the facade use it. Its facade
re-export stays `#[doc(hidden)]`, so the documented API does not grow. `cap`
stays crate-private inside `remus-construct`, and only
`nonplanar_ring_surface` is re-exported, exactly as `lib.rs:94` does today.
`affine_blend_caps`, `blend_material` and `local_wound` all land in
`remus-features` and stay `pub(crate)`.

### Effect on boundary rules and downstream crates

**`scripts/check-boundaries.sh` needs real changes. Adding `check_deps` lines
is not enough.** The script tests each crate's `[dependencies]` against a
fixed list of names:
`for dep in remus remus-math … remus-io remus-render`. A crate missing from
that list is never seen. Without editing the list, a `remus-tessellate` →
`remus-boolean` dependency would pass the check, and so would a new crate
depending on `remus-render`. Required edits:

1. Add the seven new names to the `for dep in` list.
2. Add one `check_deps` line per new crate with exactly the allow-list in the
   table above, including the internal L3 order (e.g. `remus-tessellate` may
   not depend on `remus-measure`).
3. Extend the `operations` line with the seven new crates. Its L2 entries can
   shrink to what the facade itself uses (topology, math, sketch, and
   algo/check/heal/blend for `journal_ops`/`sew`/`pattern`).
4. Optionally allow `render` → `remus-tessellate` + `remus-ops-core` (see
   below).

`CLAUDE.md`'s layer table, Module Map and the `layer-boundaries` skill gain
L3 sub-ordering rows. The "Allowed `use` paths" list gains one entry per new
crate.

Downstream crates:

| Crate | Uses today (production) | Change needed |
|---|---|---|
| `crates/wasm` | 49 operations modules | None; it keeps `remus-operations`. Its `blend`-only-via-operations rule is unaffected. |
| `crates/io` | `primitives`, `tessellate`, `measure`, `boolean`, `transform`, `sew`, `loft`, `assembly`, `validate`, `revolve`, `journal_ops`, `heal`, `extrude` (most `primitives` uses are in-file tests) | None |
| `crates/wasm-io` | `tessellate`, `sew`, `primitives`, `measure` | None |
| `crates/render` | `tessellate` + `OperationsError`; `primitives` appears only in a doc link | None required. **Optional win:** depend on `remus-tessellate` + `remus-ops-core` directly, so render compiles in parallel with the measure→features chain. The script's "nothing depends on render" rule is unchanged. |
| `crates/remus` (L5) | `project_curve`, `measure`, `journal_ops`, `tessellate`, `primitives`, `blend_ops`, … | None |
| `crates/offset` (dev-dep, path-only) | operations in tests | None |
| `tools/*`, `fuzz` | operations | None |

The workspace uses `members = ["crates/*", "tools/*"]`, so new crates join
automatically. Each still needs a `[workspace.dependencies]` entry and its own
`[lints] workspace = true`. `cargo machete` will flag facade dependencies that
are no longer used once code moves.

## 3. Compile-time impact

### Measured (baseline `c91287d`, 4 vCPU)

| Scenario | Wall | Notes |
|---|---|---|
| Clean `cargo build -p remus-operations` (dev) | **42.2 s** (user 81 s) | `remus-operations` unit 22.4 s, starting at 19.7 s (pipelined behind `remus-algo`, 11.9 → 25.6 s) |
| Incremental dev build after a comment-only edit in `tessellate/edge_sampling.rs` | 2.4 s | |
| Incremental dev build after a function-body edit in `tessellate/planar.rs` | 2.6–2.9 s | Two runs |
| Incremental dev build after adding a `pub fn` to `tessellate/planar.rs` | 2.6 s | |
| Incremental dev `cargo test -p remus-operations --lib --no-run` after the body edit | **9.2 s** | The unit-test binary links all 152 files' tests |
| Clean `cargo test -p remus-operations --no-run --profile ci-test` (CI's profile; deps included) | **178.7 s** (user 623 s) | Critical path: operations lib **51.5 s** → `remus-io` (dev-dep) 19.8 s → operations unit-test binary **70.9 s** (107.6 → 178.5 s). 273 integration-test binaries: sum 190 s, median 0.5 s each. |
| `ci-test` test rebuild (`cargo test -p remus-operations --no-run`) after the tessellate body edit | **130.7 s** | `ci-test` sets `incremental = false`: operations lib recompiles in full (41.0 s); then `remus-io` 15.2 s, the unit-test binary 76.9 s (53.6 → 130.5 s), and all 273 integration-test binaries relink |
| Dev-profile `cargo test -p remus-operations --no-run` | **did not fit** | 111 of 273 integration-test binaries reached about 23 GB (≈210 MB each, debuginfo), and the build hit the 30 GB session disk limit. The whole `ci-test` build is 4.0 GB. |

Takeaway: a dev incremental *library* build is already fast (≈3 s), so it is
not the bottleneck. The costs sit in three other places:

- the monolithic unit-test binary (71 s in `ci-test`, 9 s incremental in
  dev);
- every non-incremental profile (`ci-test`, release/wasm) recompiling the
  whole 120k production lines on any edit;
- relinking 273 integration-test binaries on every change.

### Projected (estimates, not measurements)

Assumptions: compile time scales roughly with production lines, and Cargo's
pipelining lets a dependent start its front end once the dependency's
metadata exists (the dependency's codegen overlaps).

- **Clean library build: small win, roughly 0–15 %** of the 22 s (dev) or
  52 s (`ci-test`) operations segment. The crate graph is a chain, and only
  `remus-transform` ∥ `remus-tessellate` are truly parallel. Pipelining
  overlaps each crate's codegen with the next one's front end, but rustc
  already spreads one crate's codegen across 4 cores. On machines with more
  cores the gain is somewhat larger. `remus-ops-core` depends on every L2
  crate through `OperationsError`'s `#[from]` arms, so nothing starts before
  `remus-algo`'s metadata, the same as today.
- **Unit-test build: the clearest win.** The 70.9 s unit-test binary becomes
  seven independent test binaries. Each depends only on its own crate and
  can build in parallel once the library chain finishes. Scaling by
  production lines, the largest (`remus-features`, 27 %) comes to about
  19–25 s. Lower crates' unit tests also stop waiting on `remus-io`. Estimate:
  the `ci-test` critical-path tail drops from about 71 s to 25–35 s, i.e.
  roughly 179 s → 130–145 s for a clean `cargo test -p` of the stack.
- **Non-incremental rebuild after one edit: depends on where the edit is.**
  - An edit in `remus-features` (27 % of lines) recompiles only `features` +
    facade: about 30 % of today's 52 s `ci-test` library time, plus the
    dependents' test binaries. Today the same edit costs the full
    130.7 s `ci-test` rebuild measured above.
  - An edit in `remus-tessellate` still recompiles every crate above it:
    about the same as today. **Tessellate gains only test locality:**
    `cargo test -p remus-tessellate` builds and runs tessellate's own tests
    without building the other ~100k lines' unit tests.
- **Release / WASM (`codegen-units = 1`, fat LTO): not measured here.** One
  operations crate at `codegen-units = 1` compiles single-threaded. Seven
  crates give two-wide parallelism at best (transform ∥ tessellate). Fat-LTO
  link time, which dominates `cargo xtask wasm-build`, is unchanged.
  Estimated effect: small.

## 4. Staged migration

Each stage is one PR, behavior-preserving, with no public path change through
`remus-operations`. Every stage runs the full workspace build, test, clippy,
`check-boundaries.sh`, `check-doc-paths.sh`, and the WASM size comparison.

**Stage 0: in-place preparation (no new crates).**
- Move cycle breakers 1–12 inside `remus-operations` into their future
  modules (`face_polygon` → a new `polygon` module, etc.), with re-exports at
  the old paths.
- Split `test_helpers` into:
  - pure-topology assertions (`euler_characteristic`, `assert_manifold`,
    `assert_point_near`) behind a `test-utils` feature, the same pattern as
    `remus-topology`;
  - fixture builders that need higher modules (`blended_box`, `split_band`,
    `make_saddle_profile`), which stay with their users.
- Gate `fillet::arc_torus::constructed_tab_runout_for_test` behind a
  `test-hooks` feature instead of `cfg(test)`. It is the one cross-module
  `#[cfg(test)]` hook, used by tessellate's tests, and `cfg(test)` is not set
  in a dependency.
- Extend `check-boundaries.sh` as described above, so the new names are
  guarded from day one.
- Add a CI or xtask check that recomputes the module graph and fails on a new
  upward edge between future crate groups. That keeps the DAG from regressing
  while the stages land.

*Stage 0a as landed.* The cycle breakers, the `check-boundaries.sh` name list
and the module-graph check (`scripts/check-ops-module-graph.py`, run by
`check-boundaries.sh`) landed first; the `test_helpers` split and the
`test-hooks` feature are left to a stage 0b PR. Moving the 12 items whole
took their private dependencies with them, beyond what the table above names:

- `sample_edge_curve` and `CLOSED_CURVE_SAMPLES` went to `polygon` with
  `face_polygon`.
- `HealingChange` and `HealingChangeKind` went to `error_payloads` with
  `HealingReport`.
- `face_surface_normal_at` went to `tessellate::surface_normal` with
  `face_surface_normal_with_grid`.
- `triangles_intersect` wraps the whole triangle–triangle narrow phase of
  `mesh_boolean` (`intersect_triangles` and 12 helpers, about 500 lines). That
  became `triangle_intersection` in the `remus-measure` tier.
- The new `gear` module (added after this RFC) sits in the facade tier: it
  uses `boolean`, `extrude`, `classify` and `measure`.

*Stage 0b as landed.* `test_helpers` is gone. Its helpers moved to the tier
that provides what they call:

- `test_utils` (core tier): `euler_characteristic`, `assert_euler_genus0`,
  `assert_manifold`, `assert_point_near`, `assert_volume_conservation`.
  It is compiled for the crate's tests and, behind the new `test-utils`
  feature, for downstream tests. The module-graph check now treats
  `#[cfg(any(test, ..))]` code as production, so it stays core-only.
- `measure::test_support`: `assert_volume_near`, `assert_area_near`,
  `assert_cw_profile_produces_valid_solid`.
- `fill_face::test_fixtures`: `make_saddle_profile`.
- `resize_blend::test_fixtures`: `blended_box`, `cylinder_faces`,
  `split_band`.

The `test-hooks` feature was **not** added: it cannot work as specified.
`constructed_tab_runout_for_test` builds its body from
`tests/data/rollover_tabbed_source.arena.json` and checks it through
`remus_io`, which is a dev-only dependency that sits above `remus-operations`.
A feature-gated, non-test build of the hook cannot link it. The hook's one
cross-tier user, `tessellate/solid.rs`'s `inner_tangent_runout_tests`, also
reads tessellate internals (`build_boundary_plan`,
`tessellate_face_with_shared_edges`, `edge_to_face_map_for_faces`), so it
cannot simply move up to the fillet tier either. Stage 1 has to decide that
test's home. The two candidates are:

1. Freeze the four constructed bodies as arena fixtures, with a fillet-side
   test that the constructor still reproduces them. The tessellate test then
   needs only `remus_io` and its own internals.
2. Expose the tessellate internals it reads under a test feature, and make
   it a fillet-tier integration test.

**Stage 1: `remus-ops-core` + `remus-tessellate`.** This is the most
self-contained pair: tessellate's production code references only
`OperationsError` (5 variants), `authoritative_edge_domain`,
`performance::span` and breaker 11.
- Move `lib.rs`'s shared items, `performance`, `winding`, `evolution` and
  `boundary_evolution` into core, and `src/tessellate/**` (32 files) into
  `crates/tessellate/src/`.
- Make public, under `#[doc(hidden)]` or a `__internal` module, what other
  modules use today through crate visibility:
  - `pub(crate) mod boundary_plan`, `edge_sampling`, `rim_chain`;
  - `pub(crate) use` of `COINCIDENT_DEDUPE_GRID`,
    `tessellate_closed_face_set`, `tessellate_face_for_measurement`,
    `tessellate_solid_for_measurement`;
  - `PointMergeMap`.
- Make public from core, under `#[doc(hidden)]`:
  - `authoritative_edge_domain`, `normalize_legacy_edge_domain`,
    `reconstruct_legacy_edge_domain`, `preflight_face_edge_domains`,
    `refuse_cavity_shells`, `dot_normal_point`;
  - `performance::BooleanRoute` and `bump_boolean_route`.
- Tests: tessellate's in-crate tests that need only topology fixtures stay
  unit tests. The 12 files that build fixtures through
  `primitives`/`boolean`/`fillet`/`push_pull`/`shell_op`/`sew`, or read STEP
  through `remus_io` (3 files), either:
  1. become a **single** integration-test binary,
     `crates/tessellate/tests/upstream/main.rs`, with a path-only
     dev-dependency on `remus-operations` (the same trick `remus-operations`
     uses for `remus-io` today) and internals reached through a `test-hooks`
     feature; or
  2. move into the facade's unit tests.

  Option 1 is recommended. It keeps the tests next to the code and keeps one
  binary, not twelve, given that each debug test binary is about 210 MB.

**Stage 2: `remus-transform` and `remus-measure`.** The two crates are
independent and can land in either order. `remus-measure` takes breakers
6–10. Its production users reach across with about 20 `pub(crate)` items:
`RobustClassifier`, `point_to_face_distance`, `point_in_polygon_3d`,
`orientation_error_count`, `volume_memo::*`, `query::edge_concavity_*`,
`ConcavityProbe::new`, `edge_is_tangent`, `measure::edge_length`.

**Stage 3: `remus-construct`.** Small production surface (2 measure items, 4
transform items). Test fixtures in `classify`, `measure`, `validate`, `query`
and `projection` call `primitives`, `extrude` and `revolve`, all of which
sit *above* `remus-measure`. Those tests (about 40 distinct items referenced)
must already become integration tests when `remus-measure` is created in
stage 2. See the risk list.

**Stage 4: `remus-boolean`.** It takes the largest set of private
cross-references: 13 `copy::Splice*`/`copy_solid_*` items from
`remus-transform`, 6 `BooleanRoute` items from core, and 4 `heal::*` items
used by `features`/the facade.

**Stage 5: `remus-features`.** What remains in `remus-operations` is the
facade plus `journal_ops`, `prepared_move`, `imprint`, `pattern`, `assembly`,
`sketch`, `linked_sketch`, `sew`, `untrim` and `offset_wire`.

### Risks

- **Open PRs at baseline.** These are the conflict risks if a stage lands
  while they are open:
  - #1358, the kernel class rename to `RemusKernel`: 100+ files, mostly
    `crates/wasm`, plus 2 operations integration tests.
  - #1360: moves tests inside `performance.rs`, a stage-1 file.
  - #1359: package refresh.

  Stage 1 must rebase after #1360. More generally, every stage is a
  whole-directory `git mv` that conflicts with *any* concurrent edit under the
  moved modules. Land each stage right after a merge lull. Do the move and
  the visibility edits in separate commits, so rebases replay cleanly.
- **`pub(crate)` → `pub`.** 66 `pub(crate)` items, 2 private items and 3
  `pub(crate)` modules are referenced across the proposed production
  boundaries by path. Field and method accesses on `pub(crate)` struct
  fields are not counted by path analysis; the compiler will list them. Each
  newly public item widens the reachable API through the facade's module
  re-exports, a **public API change**. Mitigation: `#[doc(hidden)]` plus a
  `__internal` naming convention, and an API-diff check against the baseline
  rustdoc JSON.
- **Unit tests that reach upward.** This is the largest work item. In the
  test-inclusive graph, lower crates' tests reference 156 distinct items of
  higher crates:
  - tessellate (boolean, primitives, fillet, push_pull, shell_op, sew,
    measure);
  - measure (boolean, primitives, fillet, chamfer, push_pull, shell_op);
  - transform (boolean, primitives, measure);
  - core (`primitives::make_box`, `blend_ops::fillet_v2`);
  - construct and features (`test_helpers`, `journal_ops::*_journaled`).

  None can stay a unit test with a dev-dependency on a higher crate. That
  would build two copies of the lower crate, the "two versions of crate"
  error. Worse, **thread-local memos**, such as the face-mesh cache in
  `tessellate/face_cache.rs` and `measure`'s volume memo, would exist twice,
  so a test enabling the cache in one copy and observing it in the other
  would silently test nothing. Each such test becomes an integration test,
  one binary per crate, or moves up.
- **Feature flags.**
  - `perf-counters` gates code in `performance.rs` (core) and a `scaling_`
    test in `classify.rs` (measure). Core and measure each need the feature,
    and the facade's feature forwards to them and to boolean.
  - CI runs the complexity guard as
    `cargo nextest run -p remus-operations --features perf-counters -E 'test(scaling_)'`.
    Once `scaling_` tests move into `remus-boolean`/`remus-measure`/the core
    crate, that filter **silently matches fewer tests and stays green**. The
    command must change to list the new packages (or `--workspace`). Add a
    guard that the selected test count does not drop.
  - `test-utils`: new crates enable `remus-topology/test-utils` in their
    dev-deps like every L2 crate does.
  - A new `test-hooks` feature (stage 0) must never be enabled by a
    production dependency. `cargo xtask wasm-build` builds only
    `remus-wasm`'s graph, so it stays off there.
- **Mutation testing.** cargo-mutants runs the tests of the package that
  owns the mutated file. Tessellate mutants are killed today partly by
  boolean/fillet tests and the 273 integration tests in `remus-operations`.
  After the split, `remus-tessellate`'s own tests would see them. The
  narrower scope is the goal, but kill rates will drop until the mutation
  config uses `--test-workspace` or a per-crate `test_package` list.
  Re-baseline the mutants report at each stage.
- **Docs and doc-path check.** `CLAUDE.md`'s operations Module Map names 111
  files, and `CLAUDE.md` plus the skills spell out 138
  `operations/src/...` paths. `scripts/check-doc-paths.sh` matches
  path suffixes, so most entries keep resolving only if the documented
  suffix survives the move. `operations/src/…`-prefixed entries break. Each
  stage updates the map in the same PR.
- **WASM size: expected neutral; verify at stage 1.**
  - The release profile is fat LTO with `codegen-units = 1`, so cross-crate
    inlining and dead-code removal see the same whole program as today.
  - The `opt-level = "z"` override applies only to `remus-wasm`. New crates
    inherit `opt-level = 3`, as `remus-operations` does now.
  - The `#[cfg_attr(target_arch = "wasm32", inline(never))]` annotations
    move with their functions.
  - Duplicate generic monomorphizations across crates are merged at fat-LTO
    link.

  If the CI "WASM Binary Size" comment shows growth above noise, the stage PR
  carries a `WASM-Size:` line and the cause is fixed before merge. The
  `core` tier assignment does not change, because these are native crate
  boundaries, not browser modules.
- **Natural split points for future lazy WASM modules.** This RFC does not
  claim them. The `remus-tessellate` / `remus-measure` boundaries are
  *candidates* for read-only `lazy:` modules under the
  [WASM module tiers](../../CLAUDE.md#wasm-module-tiers) rule. A tier change
  still needs its own design under the browser kernel size plan (#1026).
- **Error type.** `OperationsError` stays one enum in core. It therefore
  depends on every L2 crate, as above. Splitting it per crate would change
  every public signature and is out of scope.

## Decision requested

1. Approve the crate boundaries and order in §2, or adjust them (open
   questions: `heal` beside `boolean` vs. in `remus-measure`; a separate
   `remus-offset-ops`).
2. Approve stage 0 as the next PR. It is code-only, makes no crate moves, and
   removes the production cycle inside today's crate. It is worth doing even
   if the split stops there.
3. Choose the test-placement policy for upward-reaching tests: one
   integration-test binary per crate (recommended), or the facade's unit
   tests.
