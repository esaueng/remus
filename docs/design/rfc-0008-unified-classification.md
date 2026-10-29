# RFC 0008 Unified ray–surface intersection and point classification

Status: proposed. Investigation and design only; no production code changes.
Baseline: `origin/main` at `c91287d` on 2026-10-29. Line numbers identify that
baseline; the named function is authoritative when later edits move it.

Tier: `core` (refactor of code already shipped in the kernel; no new export).
The new crate is linked by `remus-algo` and `remus-check`, both already in
`remus-wasm`, so the expected binary-size effect is neutral to negative.

## Summary

Point-in-solid classification is implemented twice in production, with a
third, dead copy:

| Engine | Crate | Status | Used by |
|---|---|---|---|
| GFA classifier: analytic fast path + ray cast | `remus-algo` `classifier/` | live | every boolean: `classify_sub_faces`, `build_fuse_n`, sheet arrangement, coplanar depth probe |
| Ground-truth classifier: ray vote + graze skeleton | `remus-check` `classify/` | live | `operations::classify`, WASM `classifyPoint`, concavity probes, HLR, blend material, boolean material checks |
| Planar-only 3-ray vote | `remus-operations` `boolean/classify.rs` | dead (`#![allow(dead_code)]`; only `polygon_centroid` used) | none |

`operations::classify::classify_point` is a wrapper over the `remus-check`
classifier (its `deflection` argument is ignored). The solid-verification
skill's statement that the JS `classifyPoint` routes to a tessellation-based
caster is stale. The mesh winding variants (`classify_point_winding`,
`RobustClassifier`) are a separate, mesh-based family.

The two live engines differ in surface coverage, degeneracy policy and trim
representation, and **each is wrong where the other is right**. A
differential run over the STEP fixture corpus and synthetic bodies, checked
against a fine-mesh ray-parity oracle, found:

- 1111 wrong `remus-algo` verdicts out of 21327 points;
- 147 wrong `remus-check` verdicts out of the same 21327 points.

It also confirmed six concrete defects (§2.3).

The math kernels below classification are already partly shared through
`remus-math`: the torus quartic, line–NURBS intersection, ray–triangle and
the robust 2D predicates. The duplication sits in:

- ray ∩ plane, cylinder, cone and sphere;
- face-trim containment;
- the degeneracy policy (graze, tangency, recovery rays);
- the vote loop.

**Proposal.**

- Move the analytic ray–surface primitives into `remus-math`, returning
  *qualified* hits (transversal, tangent or uncertain) under one tangency
  policy.
- Move the topology-level classifier into a new crate, `remus-classify`,
  between L1 and L2. It depends on `math`, `topology` and `geometry`;
  both `algo` and `check` depend on it.
- Migrate in seven stages (§4). Each stage is proven by a bit-identity
  oracle or by a differential oracle whose every flip the mesh oracle
  confirms as a fix.
- Fix the confirmed defects once, in the shared code, before `algo` adopts
  it.

## 1. Inventory

### 1.1 Classification entry points

| Entry point | Location | Decides | Notes |
|---|---|---|---|
| `classify_point(_with_tolerance)` | algo `classifier/mod.rs:45,58` | `FaceClass` for a point vs solid | Analytic first, then ray cast. |
| `classify_point_cached(_with_tolerance)` | algo `classifier/mod.rs:83,97` | same | Reuses `RayCastGeoms`; **rebuilds the analytic classifier on every call**. |
| `classify_coincident_coplanar` | algo `classifier/mod.rs:150` | `Some(Outside)` or `None` for a planar sub-face lying on an opposing plane | 2D containment plus a depth probe of up to 7 candidates × 2 vote calls. |
| `classify_ray_cast(_cached)`, `ray_cast_inside_votes(_cached)` | algo `classifier/ray_cast.rs:464–599` | Inside/Outside (never `On`) | 3 cardinal rays plus a 3-ray generic re-cast. |
| `try_build_analytic_classifier` / `classify_analytic` | algo `classifier/analytic.rs:392` | Closed-form point vs solid for 8 shape classes | Declines (`None`) inside the tolerance band, except `ConvexAnalytic`, which returns `On`. Also used by `operations/boolean/mod.rs:202,935,1667`. |
| `classify_point` | check `classify/mod.rs:77` | `Inside`/`Outside`/`OnBoundary` | One-shot; shares the vote loop with prepared. |
| `PreparedSolid::{classify_point(s), count_ray_crossings, is_point_on_boundary}` | check `classify/prepared.rs:250–586` | same | Frozen BVH, trims, `FaceCache`, skeleton, hull/window prunes, NURBS span prefilter, optional `FaceCrossingMemo`. |
| `ClassificationCache::classify_point(s)` | check `classify/cache.rs:243,290` | same | Persistent across calls, keyed `(lineage, generation, solid, max_recovery_attempts)`. Its `CachedSource` skips `FaceCache` and the prunes. |
| `is_point_on_boundary`, `surface_point_in_face` | check `classify/mod.rs:397`, `boundary.rs:2339` | Boundary / trim membership | `surface_point_in_face` is also used by `section.rs`, `projection/silhouette.rs`, `project_curve.rs`, `check/distance`. |
| `classify_point_winding`, `classify_point_robust` | check `classify/mod.rs:429,463` | Fan-triangulated solid angle | Wrong on curved faces (documented). |
| `classify_point(s)`, `classify_point_winding/robust`, `RobustClassifier` | operations `classify.rs:79–333` | Wrappers over check, plus mesh winding | `RobustClassifier`: mesh winding outside `0.5 ± 0.1`, else check `PreparedSolid`. |
| `classify_point` / `multiray_classify` | operations `boolean/classify.rs:123,225` | Planar-only | **Dead code.** |
| `component_encloses_any_point`, `triangles_enclose_points` | operations `boolean/mod.rs:6034,6097` | Mesh parity, one ray `(√2,√3,√5)`, no vote | Same loop twice in one file. |

### 1.2 Ray–surface routines

| Surface | algo (`classifier/ray_cast.rs`) | check (`classify/ray_surface.rs`) | Shared in math? |
|---|---|---|---|
| Plane | `plane_hit` :436 — parallel if `|n·d| < tol.angular` (1e-12); a ray in the plane is flagged *suspicious*; `t > tol.linear` | `ray_plane` :34 — parallel if `< 1e-15`, giving no hit; `t > 1e-12` | no |
| Cylinder | `ray_cylinder_crossings` :1686 — **tangent if `disc < 1e-12·a·R²` (relative): 0 crossings**; v-range ± `tol.linear` | `ray_cylinder` :51 via `solve_quadratic` — **`|disc| < 1e-15` (absolute): one double root, counted as 1 crossing** | no |
| Cone | `ray_cone_crossings` :1841 — same relative tangency rule; nappe and apex rejected by the slant-range filter | `ray_cone` :76 — same absolute rule as the cylinder | no |
| Sphere | **none: the face is flattened to a planar boundary polygon** | `ray_sphere` :101 (quadratic, absolute rule) | no |
| Torus | `ray_torus_crossings` :1777 → `math::intersect_line_torus`; suspicious if two roots fall within `near` | `ray_torus` :120 → `math::intersect_line_torus` | **yes** (`analytic_intersection.rs:1715`, Durand–Kerner quartic, the only quartic solver) |
| NURBS | **none: the face is flattened to a planar polygon**. A rational cylinder wall is recognized as `Cylinder` | `ray_nurbs(_with_grid)` → `math::intersect_line_nurbs(_with_grid)` (grid + Newton + Bézier-subdivision completion); tangency abstention `NURBS_RAY_GRAZE` | **yes** (`nurbs/intersection/line.rs:39,301`) |
| Triangle | — | — | **yes** (`ray_triangle.rs:68`, used only by operations mesh parity) |

Other line/segment-vs-quadric code, outside classification:

- algo: `phase_ff.rs:5545` (segment ∩ cone), `phase_ef.rs:1068` (edge ∩ cylinder), `phase_ef/line_sphere.rs:354` (certified, interval arithmetic);
- operations: `project_curve.rs:1454` `ray_roots`, `local_wound.rs:180`, `boolean/mod.rs:3070`.

There are **five quadratic solvers with five tangency policies**:

| Solver | Tangency policy |
|---|---|
| check `solve_quadratic` | absolute snap to a double root |
| math `real_roots_quadratic` | naive, private |
| `project_curve::quadratic_roots` | relative snap |
| `section::quadratic_roots` | tangent gives no roots |
| `phase_ff::solve_segment_quadratic` | naive, clamped |

### 1.3 Degeneracy handling

| Case | algo | check |
|---|---|---|
| Directions | Cardinal `+Z,+X,+Y`; re-cast with 3 fixed √-prime vectors (`RAY_DIRS` :331) | 3 fixed irrational vectors (`mod.rs:167`), then up to `max_recovery_attempts = 10` golden-ratio recovery directions |
| Ray through an edge or vertex | Hit within `near = 10·tol.linear` (1e-6, absolute) of a boundary segment or band sets *suspicious*. Generic re-cast adopted only if unanimous; with all 3 cardinal rays suspicious, the generic count is used unconditionally | `graze::BoundarySkeleton` holds **every** edge of every face (curved edges as 32 chords plus vertex balls), margin `max(1e-9·diag, 256ε·|coord|)`, scale-aware. A grazing ray **abstains**; recovery rays replace it (`rays_through_vertices_abstain_and_recovery_decides`, `…_far_from_the_origin`) |
| Tangent to a quadric | Counted as 0, not suspicious (relative threshold): **correct parity** | Exact tangency at small scale counted as 1: **wrong parity** (F3) |
| Torus double root | Shared math dedup (`1e-9·(1+|x|)`) can merge a double root into one; flagged only if *two* roots survive within `near` | Same merge, no flag. Neither engine guards a merged double root (code reading; no test pins it) |
| NURBS tangency | n/a (flattened) | Curvature-aware slope test; abstains (`apex_contact_is_preserved_by_math_and_abstains_from_parity`) |
| Seams / poles | Cylinder and cone: `u` by `rem_euclid(TAU)`, largest-gap heuristic (`> 0.2` rad); torus: rectangular domain, or adaptive trim polygon in unwrapped (u, v) | UV loops unwrapped by period; hit shifted to the polygon centre's period; cone apex via `insert_pole_segments`; sphere never uses UV |
| On-boundary | Never, except `ConvexAnalytic`. Same-domain and coplanar passes handle `On` upstream | Distance `< tol` plus a trim test before voting. Planar 3D-polygon projection for curved faces (F4) |

### 1.4 Trim containment

| Face | algo | check |
|---|---|---|
| Plane | 3D boundary polygon; **3 interior samples per open curved edge, 16 per closed curve**; robust `orient2d` winding | 3D polygon, **32 samples per curved edge**; robust winding |
| Cylinder / cone | `v`-range plus `u`-gap, or full-turn hole bands only; other holed faces fall back to a planar polygon | UV polygon (32 samples per edge), parity |
| Sphere | planar polygon of the boundary | Half-space cap, then qualified great-circle trim, then two-plane circle patch, then chorded polygon (F2) |
| Torus | rectangular band, or adaptive UV trim polygon with a certified margin (memoized), else planar polygon | UV polygon plus oriented-band rule for loops that wrap a period |
| NURBS | planar polygon, except recognized rational cylinder walls | Newton-projected UV polygon; certified full-cap fallback |

Other copies: float even-odd 2D point-in-polygon in algo `classify_2d.rs:237`, `phase_ff_coplanar.rs:1111` (verbatim), heal `unify_same_domain.rs:980`, operations `split.rs`, `chamfer.rs`, `shell_op.rs`, `tessellate/nonplanar.rs:6500`. A verbatim copy of check `util::point_in_polygon_3d` sits in operations `distance.rs:228`. The robust originals are `math::predicates::{point_in_polygon, winding_number}`. Out of scope here except as stage-6 cleanup.

### 1.5 Caches and memos

| Cache | Key | Identity guarantee |
|---|---|---|
| algo `RayCastGeoms` | none (lives for one boolean operand) | — |
| algo `ray_geom_memo` (torus trims only) | full trim inputs + tolerance bits; per thread, FIFO 1024 entries / 8 MiB; WASM enables it | bit-identical (`memoized_trims_are_bit_identical_to_computed_ones`, io `hammer_ray_geom_oracle`) |
| check `FaceCache` (`OnceLock`s inside `PreparedSolid`) | the face (borrow-scoped) | pure functions of the face |
| check `PreparedSolid` prunes (hull, trim window, NURBS span prefilter) | — | exact skips (io `boundary_prune_oracle`, `prefilter_never_skips_a_ray_the_solver_hits`) |
| check `FaceCrossingMemo` (impl: operations `query/crossing_memo.rs`) | face content + ray bits | bit-identical (io `hammer_crossing_memo_oracle`) |
| check `ClassificationCache` | `(lineage, generation, solid, max_recovery_attempts)`; tolerance not in the key | mutation, rollback and ABA tests in `cache.rs` |

### 1.6 Hotness

- **algo:**
  - One interior point per split sub-face, against each opposing operand (N-way fuse: each rival source, pruned by candidate pairs).
  - `RayCastGeoms` is built once per operand. The analytic classifier is rebuilt on every point.
  - Work is counted by `perf::RayWorkCounts`.
  - Pinned by `RAYCAST_VOTE_WORKLOADS` (operations `boolean/tests.rs:10957`) and `scaling_raycast_votes_measure_few_segments`.
- **check:**
  - Concavity probes (`query.rs` `ConcavityProbe`, 4 per edge in recognition) and boolean material checks use `PreparedSolid`.
  - Blend material uses `classify_points`.
  - WASM uses `ClassificationCache`.
  - HLR `projection.rs:154` `is_hidden` calls the **one-shot** path per sample, rebuilding the BVH and skeleton every time. That is a standalone performance bug, not in scope.

## 2. Behaviour diff

### 2.1 Which engine is more correct, by case

| Case | More correct | Evidence |
|---|---|---|
| Sphere faces in non-convex solids | check (exact quadric) | Differential (§2.2): `shapr3d_walking_stick_foot` 173 algo-only errors. algo flattens sphere faces |
| NURBS faces | check (exact line–NURBS plus tangency abstention) | Differential: `shapr_untrimmed_nurbs_domain` 206, `openzcad_e_nurbs_fillet_plate` 178 algo errors (check 8). Tests: `nurbs_bump_ray_tests.rs`, `regress_nurbs_bump_ray_crossing` |
| Torus bands and partial tori | check (UV trim plus oriented-band rule) | Differential: `lemon-torus-band` 330 of 645 algo-only errors. check `regress_torus_wrapping_trims` |
| Rays through edges and vertices | check (skeleton abstention plus recovery) | `rays_through_vertices_abstain_and_recovery_decides`, `…_far_from_the_origin` (1e9 offset). algo's `near` is absolute 1e-6, so it is not scale-aware |
| Exact tangency to cylinder/cone/sphere | **algo** (relative threshold, tangent counts 0) | Probe F3: check wrong at r = 0.37, algo right |
| Points near a circular planar rim | **algo for primitives** (the analytic classifier answers before any trim). Both are exposed on general solids, algo more so (16 vs 32 samples) | Probe F1 |
| Hot-path cost on planar-heavy booleans | algo (plane groups, proven-exact polygon gate, torus cull) | `gate_oracle_tests.rs`, `torus_cull_oracle_tests.rs`, `RayWorkCounts` |

### 2.2 Differential run

Method:

- **Bodies:** every solid of every `crates/io/tests/data/*.step` fixture, plus box, cylinder, cone, sphere, torus, box − cylinder, box ∪ sphere and box ∩ torus.
- **Points:** a 7³ grid over the inset bounding box, plus about 300 near-surface probes (triangle centroids ± 1e-3 or 2e-4 × diagonal along the facet normal).
- **Classifiers:** algo `classify_point_cached` and check `PreparedSolid::classify_point`.
- **Oracle:** watertight ray–triangle parity on a `tessellate_solid` mesh at deflection `2e-5 × diagonal`. Five irrational directions; a verdict counts only when all five agree, and no point was ambiguous.
- **Skipped:** five bodies whose fine mesh exceeded 400k triangles (whole torus, `mambo_b12_period_winding_torus`, both `wide_sphere_cap` fixtures).
- **Scale:** 21327 points in total; 1062 algo/check disagreements.

| Body | algo wrong | check wrong | Cause |
|---|---:|---:|---|
| `lemon-torus-band.step` | 330 | 0 | algo torus-band fallback |
| `shapr_untrimmed_nurbs_domain.step` | 206 | 0 | algo NURBS flattening |
| `openzcad_e_nurbs_fillet_plate.step` | 178 | 8 | algo NURBS flattening; check rim-sagitta class (F1) |
| `shapr3d_walking_stick_foot.step` | 173 | 0 | algo sphere/NURBS flattening |
| box ∪ sphere (r 6, protruding through 3 box faces) | 170 | 96 | check sphere trim F2; algo sphere flattening |
| `shapr3d_hammer_holder.step` | 54 | 0 | algo flattening |
| cone, cylinder, `axis2_optional_attrs_cylinder.step`, `oring_nested_holes.step` | 0 | 20, 9, 8, 6 | check rim-sagitta class (F1) |
| 25 other bodies (lipfuse, scoop, multicavity, wallcut, bored and analytic fillet plates, frustum, partial cylinder, sphere, box − cylinder) | 0 | 0 | — |

The harness was a scratch integration test in `crates/io/tests` and is not
part of this PR. Landing it as a pinned oracle is stage 0.

### 2.3 Confirmed defects

Each item below was reproduced in isolation, not only seen in the differential.

- **F1. check: points within one chord sagitta of a circular planar rim.**
  - Repro: in a cylinder of radius 3, `(2.3955, -1.8049, -0.0116)` lies below the bottom cap and is outside by inspection, yet classifies `Inside`. `(-1.8094, -2.3911, 7.9884)` lies inside and classifies `Outside`.
  - Mechanism: the cylinder wall is intersected exactly, but the cap is trimmed by a 32-chord polygon (`util::CLOSED_CURVE_SAMPLES`), whose inradius is `3·cos(π/32) = 2.9856`. A ray that crosses the cap in the sagitta band misses it, and the graze skeleton (margin ≈ 1e-9) does not cover a band 1.4e-2 wide.
  - Coverage: algo shares the defect class with a coarser chord (16 per closed curve, 3 per open arc). It escapes only where its analytic classifier answers first.
  - Exact fix already in the tree: `check/distance/mod.rs:443` `plane_point_in_face` does exact planar trim parity with circle recognition.
- **F2. check: sphere patches bounded by several small circles.**
  - Repro: box `20×20×10` ∪ sphere `(20,10,5)` r 6. The result validates clean and its volume 4470.07 matches the closed form (4470.2 by hand).
  - `surface_point_in_face` rejects `(26,10,5)` and `(23.6, 10±4.8, 5)` on both sphere faces. So `(23,10,5)`, deep inside the sphere, gets ray counts 2, 3, 2, 1, 2 and classifies `Outside` (algo agrees, for its own reason).
  - The sphere dispatch's cap, great-circle and two-plane paths decline, and the final chorded-polygon path gets the containment wrong. Root cause not isolated.
- **F3. check: exact tangency to a quadric at small scale.**
  - Repro: a point outside a cylinder of radius 0.37 where two base rays are both exactly tangent to the wall classifies `Inside`. Radii 1, 2 and 13 classified correctly, because the absolute `NEAR_ZERO = 1e-15` discriminant test fires only at small scale.
  - algo classifies it correctly.
- **F4. check: on-boundary detection on full-turn walls.**
  - Repro: in the cylinder `r 3, h 8`, `(0,3,4)` on the lateral surface returns `Outside` and `is_point_on_boundary = false`. `(±3,0,4)` and points on the top cap return `OnBoundary`.
  - `trim_contains_point` projects the 3D outer polygon of a curved face onto a plane, which is degenerate for a rim-seam-rim-seam loop.
- **F5. algo: sphere and general NURBS faces become planar polygons.**
  - `collect_face_geoms` at ray_cast.rs:1147 falls through to `Planar` for every face kind it does not special-case.
  - The algo inventory found no ray-cast test with a sphere face. Booleans survive today largely because sub-face sample points are rarely in the band between the polygon and the true surface, and because convex analytic solids take the analytic path.
- **F6. algo: torus band fallback.**
  - Two-rim torus bands that are neither rectangular nor accepted by the trim builder fall back to a planar polygon (`lemon-torus-band`).

**Hazard, not reproduced:**

- A torus double root merged by the math dedup counts as a single crossing in both engines.
- `analytic_intersection` should report multiplicity, not leave the decision to the callers.

## 3. Target design

### 3.1 Layering options

| Option | Verdict |
|---|---|
| `algo → check` | Rejected. Couples two L2 siblings and pulls validation, properties and distance into the boolean's dependency graph. It also forecloses `check → algo` (check may one day want certified intersection curves). |
| Everything into `remus-geometry` | Rejected. Classification needs `Topology` (faces, wires, coedges), and `geometry` is deliberately topology-free (L1, parallel to `topology`). |
| Everything into `remus-math` | Rejected for the same reason. Acceptable, and recommended, for the *topology-free* primitives. |
| Into `remus-topology` | Rejected. Bloats the arena crate with numerics and makes every topology build pay for NURBS line intersection. |
| **Primitives into `remus-math` + a new `remus-classify` crate (L1.5: math, topology, geometry)** | **Recommended.** |

`remus-classify` needs `geometry` because the graze skeleton uses
`remus_geometry::bounds::curve`. `scripts/check-boundaries.sh` **already
allows** `algo → geometry` (and `blend → geometry`), although no such
dependency exists yet. `CLAUDE.md`'s layer table says otherwise and should
be corrected separately.

### 3.2 Crate contents

**`remus-math`: new `ray` module** (topology-free):

- `RayHit { t, kind: ContactKind, ... }` reuses `intersect::ContactKind`: `Transversal`, `Tangential`, and `Unclassified` for a contact whose crossing parity is not certified.
- `ray_plane`, `ray_cylinder`, `ray_cone`, `ray_sphere`, all built on one cancellation-free quadratic with a **relative** tangency band. A tangent contact is reported, not snapped. The caller decides: parity ignores `Tangential` and abstains on `Unclassified`.
- `ray_torus` wraps `intersect_line_torus`, extended to report merged double roots as `Tangential` or `Unclassified`.
- `ray_nurbs` wraps the existing line–NURBS routine and lifts check's slope/curvature tangency test (boundary.rs:2135) into math, so the check crate's `NURBS_RAY_GRAZE` string becomes a typed `Unclassified` contact.
- `quadratic_roots` becomes the shared solver. The `project_curve`, `section` and `phase_ff` copies migrate in stage 6.

**`remus-classify`** (moves check's `classify/` wholesale, then absorbs algo's proven-exact accelerations):

- `FaceTrim`: the per-surface trim dispatch from `boundary.rs` (planar, analytic UV, torus oriented band, sphere qualified trims, NURBS UV), plus algo's adaptive torus trim polygon, which carries a certified margin.
- `BoundarySkeleton` (graze), `PreparedSolid`, `FaceCache`, the span prefilter, `FaceCrossingMemo`, `ClassificationCache`.
- One vote loop, `classify_point_with_source`, parameterized by a `VoteProfile`: direction set, recovery count, and whether to run the on-boundary pre-test. algo needs "no `OnBoundary`": a sub-face sample must vote, and `On` would push `build_fuse_n` into its sequential fallback.
- `AnalyticClassifier` (moved from algo `analytic.rs`), cached per prepared solid instead of rebuilt per point.
- Ported from algo, each with its oracle tests: same-plane hit groups (`FaceGeoms::new`), the polygon box gate (`gate_oracle_tests.rs`), and the torus line cull (`torus_cull_oracle_tests.rs`).
- `ClassifyError`. `CheckError` and `AlgoError` gain `#[error(transparent)]` arms. `remus_check::classify::*` re-exports the public API unchanged; `remus_algo::classifier::*` keeps its signatures as thin adapters until callers move.

### 3.3 Boundary table change

`scripts/check-boundaries.sh`:

```diff
 check_deps "math"
 check_deps "topology"   "remus-math"
 check_deps "geometry"   "remus-math"
-check_deps "algo"       "remus-math" "remus-topology" "remus-geometry"
+check_deps "classify"   "remus-math" "remus-topology" "remus-geometry"
+check_deps "algo"       "remus-math" "remus-topology" "remus-geometry" "remus-classify"
 check_deps "blend"      "remus-math" "remus-topology" "remus-geometry"
 check_deps "heal"       "remus-math" "remus-topology" "remus-geometry"
-check_deps "check"      "remus-math" "remus-topology" "remus-geometry"
+check_deps "check"      "remus-math" "remus-topology" "remus-geometry" "remus-classify"
 check_deps "offset"     "remus-math" "remus-topology" "remus-geometry"
 check_deps "sketch"
-check_deps "operations" "remus-math" "remus-topology" "remus-algo" "remus-blend" "remus-heal" "remus-check" "remus-geometry" "remus-offset" "remus-sketch"
+check_deps "operations" "remus-math" "remus-topology" "remus-algo" "remus-blend" "remus-heal" "remus-check" "remus-classify" "remus-geometry" "remus-offset" "remus-sketch"
 ...
-check_deps "wasm"       "remus-math" "remus-topology" "remus-algo" "remus-blend" "remus-heal" "remus-check" "remus-geometry" "remus-offset" "remus-sketch" "remus-operations" "remus-io"
+check_deps "wasm"       "remus-math" "remus-topology" "remus-algo" "remus-blend" "remus-heal" "remus-check" "remus-classify" "remus-geometry" "remus-offset" "remus-sketch" "remus-operations" "remus-io"
```

Other changes that go with it:

- Add `remus-classify` to the script's `for dep in …` list.
- Add a layer comment: `L1.5 (classify) — math, topology, geometry`.
- Mirror both changes in `CLAUDE.md` (layer diagram, deps table, allowed `use` paths: `classify/src/** → remus_math::*, remus_topology::*, remus_geometry::*`).
- Add `classify` to the kernel-crate list in `clippy.toml` (`CompactSort` rule) and to `scripts/check-det-hash.sh`.

## 4. Staged migration

Every stage is its own PR with a regression or oracle gate. The repo's
convention applies:

- keep the replaced code verbatim in a `*_oracle_tests.rs` file;
- compare bit for bit where behaviour must not change;
- where behaviour changes on purpose, require every flip to be a
  wrong→right flip against an independent oracle.

Performance gates are `RayWorkCounts`, the check `perf` counters, the
`cad_operations` criterion benches, and `RAYCAST_VOTE_WORKLOADS`
fingerprints.

| Stage | Change | Proof | Behaviour |
|---|---|---|---|
| 0 | Land the §2.2 differential harness in `crates/io/tests/classify_engine_oracle.rs`, with the current per-body wrong counts **pinned as constants** (they may only go down). Delete the dead `operations/boolean/classify.rs`, moving `polygon_centroid`. | The harness itself. Workspace suite unchanged. | none |
| 1 | `remus_math::ray` primitives with a `TangentPolicy` that reproduces each caller exactly: check's absolute snap and algo's relative drop. Route check `ray_surface.rs` and algo `ray_cylinder_crossings`/`ray_cone_crossings`/`plane_hit` through them. | Bit-identity oracles: the old functions kept verbatim; proptest over random rays including constructed exact-tangent, near-tangent, axis-parallel and in-plane rays at scales 1e-3 to 1e6; identical root vectors and suspicious flags. Torus cull and gate oracles unchanged. | none |
| 2 | Create `remus-classify` by **moving** check's `classify/` and the trim helpers from `util.rs`. check re-exports. | Code move. Existing check tests move with it; io `boundary_prune_oracle`, `hammer_crossing_memo_oracle`, `classify_prepared_reuse` and the stage-0 harness are unchanged bit for bit. Boundary script updated. WASM size comment shows ≤ 0 growth. | none |
| 3 | Fix F1–F4 and the torus double-root hazard in the shared code: exact planar trims for circle and ellipse edges (reuse `plane_point_in_face`); the F2 sphere trim; a relative tangency band (math `TangentPolicy::Report`, parity ignores `Tangential`); a proper on-boundary trim for full-turn walls (UV, not a 3D projection); multiplicity from `intersect_line_torus`. One PR per defect. | Each PR ships its closed-form regression test (F1–F4 repros above). The stage-0 harness: check's wrong count strictly decreases, and it reports zero right→wrong flips. | intended, check only |
| 4 | Port algo's exact accelerations into `PreparedSolid`: plane groups, polygon box gate, torus cull, adaptive torus trims plus `ray_geom_memo`. Cache the analytic classifier per prepared solid. | Bit-identity of check verdicts and crossing counts on the stage-0 corpus. The ported gate and cull oracles pass against the verbatim pre-port loop. check perf counters and criterion show no regression on planar-heavy bodies. | none |
| 5 | algo `classify_sub_faces`, `build_fuse_n`, sheet arrangement and the coplanar depth probe call `remus-classify` with an algo `VoteProfile` (no on-boundary pre-test). The old algo ray cast stays behind a test-only switch as the oracle. | (a) Per-sub-face `FaceClass` compared old vs new over the whole boolean test corpus (operations and io suites, `RAYCAST_VOTE_WORKLOADS`), through a test hook in `classify_sub_faces`. Every disagreement is listed and must be a wrong→right flip per the mesh oracle on that sample. (b) `approx_census` unchanged or improved. (c) Boolean golden and fingerprint tests unchanged, or re-pinned with the verified flips named in the PR. (d) `RayWorkCounts` and criterion `cad_operations` within noise. | intended, algo (F5, F6 fixed) |
| 6 | Delete algo's `ray_cast.rs` engine (keep `compute_solid_bbox`, `u_in_gap` and `largest_u_gap`, which phase VV/VE/EF/FF use). Fold the duplicate float point-in-polygon copies into `math::predicates`, and migrate `project_curve`, `section` and `phase_ff` quadratics to `math::ray::quadratic_roots` where their policy matches. | Per site: a bit-identity oracle where the policy is identical. Where it differs, a behaviour PR with its own regression test. | per site |

**Ordering constraint.** Stage 3 must land before stage 5. Otherwise algo
inherits F1–F4 at the moment it gains exact spheres and NURBS. Stage 4 must
also land before stage 5, so the boolean hot path does not slow down.

## 5. Risks and open questions

**Boolean result churn (stage 5).** Fixing F5 and F6 changes sub-face
verdicts. Some boolean fingerprints will move.

- Mitigation: the per-sub-face oracle hook plus mesh-verified flips. Any
  right→wrong flip blocks the stage.
- Expect the coplanar and grazing paths to be the most sensitive.
  `classify_coincident_coplanar` exists because cardinal rays graze
  coincident caps; check's skeleton abstention should subsume part of it,
  but it stays until the oracle proves that.

**Speed.** check's per-ray work on curved faces (UV trims, exact NURBS) is
higher than algo's polygon tests.

- Mitigation: stage 4 ports algo's accelerations first.
- Stage 5 is gated on `RayWorkCounts` and criterion.
- The analytic fast path, cached per prepared solid instead of rebuilt per
  point, should more than repay convex-operand booleans.

**Tolerance semantics.**

| Where | Linear tolerance |
|---|---|
| algo | absolute `Tolerance`; `near = 10·linear` |
| check classifier | absolute `ClassifyOptions::tolerance = 1e-6` for on-boundary; scale-aware graze margins |
| `ClassificationCache` key | tolerance not included |

`VoteProfile` must carry the caller's `Tolerance` explicitly. Defaults do
not change (CLAUDE.md: preserve units, tolerances, defaults).

**Determinism.**

- check's span-prefilter throttle uses relaxed atomics, so which rays it
  skips depends on history. Every skip is exact, so verdicts stay
  deterministic, but work counters do not. Stage-5 work gates must use
  counters that are deterministic, or disable the throttle in the
  counting tests.

**Open questions.**

- F2's root cause inside the sphere trim dispatch is not yet isolated.
  Stage 3 needs a short investigation first.
- Whether `remus-classify` should also host `surface_point_in_face`'s other
  callers (`section`, `silhouette`, `project_curve`) directly, or keep
  reaching it through check's re-export. Recommendation: re-export; no
  caller churn.
- Mesh-based classifiers (`RobustClassifier`, `component_encloses_any_point`)
  stay as they are; they answer a different question (tolerant mesh
  enclosure).

## Appendix: reproducing §2.2 and §2.3

All points below are in model units on the stated primitive at the origin
(`make_cylinder(r, h)` has its axis along +Z from z = 0).

| ID | Body | Point | Expected | check | algo |
|---|---|---|---|---|---|
| F1 | cylinder r 3, h 1000 | `(2.395539, -1.804909, -0.011615)` | Outside (z < 0) | Inside | Outside |
| F2 | box 20×20×10 ∪ sphere c (20,10,5) r 6, 16 segments | `(23, 10, 5)` | Inside | Outside | Outside |
| F3 | cylinder r 0.37, h 1000 | intersection of the tangent lines of check base directions 0 and 1, at z = 500 (`(-0.035813, -0.650472, 500)`) | Outside | Inside | Outside |
| F4 | cylinder r 3, h 8 | `(0, 3, 4)` | OnBoundary | Outside | — |
