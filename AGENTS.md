# Working agreements

Complete the requested work. Infer scope from context, make reasonable
implementation choices, and continue until finished or genuinely blocked.
Keep investigation-only requests read-only.

An explicit request or prior approval authorizes that action. Do not ask
again. Ask only when consequential ambiguity remains or an additional action
falls outside the authorized scope. Complete independent work while waiting.

Follow repository conventions. Keep changes focused. Preserve units,
tolerances, defaults, formats, and public contracts unless changes are
required. Label approximations and flag breaking changes or new dependencies.

Run required checks and tests proportional to the change. Add regression
coverage for bug fixes. Do not weaken assertions or bypass protections.
Attempt routine recovery from missing tools or Git refs before reporting a
blocker. Distinguish local validation from production verification.

Keep secrets, private instructions, and identifying information out of shared
artifacts. Inspect staged content and metadata before committing. Use the
approved repository Git identity.

## Delivery

- Use a branch and a ready-for-review pull request for repository changes.
- A merge request authorizes the identified pull request’s merge and its
  existing automatic deployment. Mention that consequence and proceed
  without another confirmation.
- The owner standing-authorizes an agent session to arm auto-merge on a pull
  request it opened once that PR is done (green on its exact head, no
  conflict, every review thread answered and resolved), unless the PR carries
  the `hold` label. See the merge gate in `.claude/skills/pr-workflow/SKILL.md`.
- Before merging, verify the current head, mergeability, reviews, and required
  checks. Wait for pending checks. Existing approval remains valid for the
  authorized change.
- Bypassing CI requires explicit authorization for the specific pull request.
  Disclose affected checks and risks; preserve review requirements.
- Standalone production deployments and manual migrations require
  authorization. An explicit request to perform them is sufficient.
- Resolve the target repository, environment, and account from configuration
  and context. Ask only if the target remains ambiguous.

Report the outcome briefly, followed by verification, actual blockers, and
relevant links. Never claim an unverified result passed.

---

# Remus — Project Guidelines

Remus is an exact B-Rep solid modeling kernel: geometry, booleans, blends,
tessellation, and I/O, with a Rust API and a WASM/JavaScript API over the same
engine.

## Architecture

Strict layered Cargo workspace. Each layer depends only on layers below it.

```
L4: remus-wasm        → JS bindings (wasm-bindgen): the kernel module
L4: remus-wasm-io     → JS bindings for the file-format translators (own module)
L4: remus-render      → Offscreen GPU rendering (wgpu) to image + face-id buffer
L3: remus-io          → STEP, 3MF, STL, IGES, OBJ, PLY, glTF import/export
L3: remus-operations  → Booleans, fillets, extrusions, tessellation
L2: remus-algo        → GFA boolean engine, classification, intersection
L2: remus-blend       → Walking-based fillet and chamfer engine
L2: remus-check       → Classification, validation, properties, distance
L2: remus-heal        → Shape healing (analysis, fixing, upgrading)
L2: remus-offset      → Solid offset engine (global face-face intersection)
L2: remus-sketch      → 2D parametric constraint solver (GCS)
L1: remus-topology    → B-Rep data structures (arena-based)
L1: remus-geometry    → Curve sampling, extrema, geometry conversion
L0: remus-math        → Vectors, matrices, NURBS, predicates
```

### Layer dependency rules

Enforced by `scripts/check-boundaries.sh` — run before pushing:

| Crate | Allowed deps |
|-------|-------------|
| `math` | *(none — no workspace deps)* |
| `geometry` | `math` |
| `topology` | `math` |
| `algo` | `math`, `topology` |
| `blend` | `math`, `topology`, `geometry` |
| `heal` | `math`, `topology`, `geometry` |
| `check` | `math`, `topology`, `geometry` |
| `offset` | `math`, `topology`, `geometry` |
| `sketch` | *(none — no workspace deps)* |
| `operations` | `math`, `topology`, `algo`, `blend`, `heal`, `check`, `geometry`, `offset`, `sketch` |
| `io` | `math`, `topology`, `operations` |
| `render` | `math`, `topology`, `operations` (L4 leaf — the script also rejects any crate that depends on `render`) |
| `wasm` | all crates (`blend` only transitively, via `operations`) |
| `wasm-io` | `math`, `topology`, `operations`, `io` |

The script checks `[dependencies]` in each `Cargo.toml`. A violation fails the Repository Policy job (`Layer boundaries` check) in CI.

**Allowed `use` paths per crate:**
- `math/src/**` → only `std`, external crates
- `geometry/src/**` → `remus_math::*`
- `topology/src/**` → `remus_math::*`
- `algo/src/**` → `remus_math::*`, `remus_topology::*`
- `blend/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `heal/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `check/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `offset/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`
- `sketch/src/**` → only `std`, external crates
- `operations/src/**` → `remus_math::*`, `remus_topology::*`, `remus_geometry::*`, `remus_algo::*`, `remus_blend::*`, `remus_heal::*`, `remus_check::*`, `remus_offset::*`, `remus_sketch::*`
- `io/src/**` → `remus_math::*`, `remus_topology::*`, `remus_operations::*`
- `render/src/**` → `remus_math::*`, `remus_topology::*`, `remus_operations::*`
- `wasm/src/**` → all `remus_*`
- `wasm-io/src/**` → `remus_math::*`, `remus_topology::*`, `remus_operations::*`, `remus_io::*`

### WASM module tiers

Every new roadmap capability declares its browser module tier in its roadmap
row or design doc **before implementation**:
`Tier: core | lazy:<module> | native`.

| Tier | Ships in | Use for |
|------|----------|---------|
| `core` | `remus-wasm`, loaded eagerly | Anything that edits the live session: handles, journal/evolution, persistent naming, checkpoints, transactions |
| `lazy:<module>` | its own package `remus-wasm-<module>`, reached through exact arena documents (like `remus-wasm-io`) | Document-in/document-out or session-independent work: translators (`lazy:io`), later the GCS solver, import repair, read-only analysis, drawings, PMI |
| `native` | not shipped to the browser | Server batch work, `remus-render`, corpus and qualification tooling |

- `core` is the exception and needs a reason: name the session state the
  capability edits. A lazy module never needs kernel handles, the journal or
  checkpoints; anything that does stays in core. GCS sketch *state* stays in
  core; only the solver is a lazy candidate.
- Only `remus-wasm-io` exists today. The planned `remus-wasm-sketch` and
  `remus-wasm-repair` modules come from the browser kernel size plan
  ([#1026](https://github.com/esaueng/remus/pull/1026)); read it before
  targeting one.
- No speed regression from size work. Consumers preload lazy modules at idle,
  so first use is no slower than a core call.
- Size review: read the CI "WASM Binary Size" comment on every wasm-touching
  PR. A kernel growth above 64 KiB raw requires a `WASM-Size:` line in the PR
  description naming the capability and its tier, for example
  `WASM-Size: +120 KiB raw, curve projection (P-Class 7.4), Tier: core (writes the journal)`.
  The `WASM Size Gate` job in `CI Pass` enforces it.

## Module Map

Quick reference — find the right file for any task:

### L0: math (`crates/math/src/`)
| Task | File(s) |
|------|---------|
| Points, vectors, matrices | `vec.rs`, `mat.rs` |
| Planes | `plane.rs` |
| NURBS curve evaluation/manipulation | `nurbs/curve.rs` |
| Bit-identity oracle for `NurbsCurve::evaluate`'s basis buffer against the verbatim pre-widening evaluate, degrees 1–12 (tests) | `nurbs/curve/eval_stack_oracle_tests.rs` |
| Compact band storage for NURBS interpolation | `nurbs/banded.rs` |
| NURBS surface evaluation | `nurbs/surface.rs` |
| Bit-identity oracle for `NurbsSurface::evaluate`'s u/v basis buffers against the verbatim pre-widening evaluate, degrees 1–12 (tests) | `nurbs/surface/eval_stack_oracle_tests.rs` |
| Bit-identity oracle for the loop-based `evaluate` contraction, the lane first-order solve, the allocation-free `normal`, the fused `evaluate_and_partials_from`/`point_and_normal(_with)` and the per-axis `first_order_at` tables against the verbatim pre-change code, degrees 1–5, 9 and 11 (tests) | `nurbs/surface/contraction_oracle_tests.rs` |
| The cached periodicity flags against the control-net comparisons kept verbatim, `with_control_points` against `NurbsSurface::new` on copied knots and weights bit for bit, and their work counts: one net comparison per surface, no knot-vector comparison per sample (tests) | `nurbs/surface/shared_storage_tests.rs` |
| NURBS knot insertion/removal | `nurbs/knot_ops.rs` |
| Certified NURBS reduction contracts and interval Bernstein bounds | `nurbs/reuse.rs`, `nurbs/reuse_bounds.rs` |
| Bounded parallel curves for monotone planar spline profiles | `nurbs/planar_offset.rs` |
| Opt-in curve/surface knot simplification and U/V removal | `nurbs/reduction.rs`, `nurbs/reduction/tests.rs` |
| Certified adaptive Hermite cubic approximation | `nurbs/cubic_fit.rs`, `nurbs/cubic_fit/tests.rs` |
| Certified piecewise cubic Hermite fit of a parametric curve with a closed-form fourth-derivative bound (C¹ double-knot B-spline, proven remainder) | `nurbs/hermite_fit.rs`, `nurbs/hermite_fit/tests.rs` |
| Certified NURBS fits of screw-motion sweeps: helices and swept surfaces sharing one span plan, so neighbouring fits match bit for bit (twisted extrusions, helical gears) | `nurbs/screw.rs`, `nurbs/screw/tests.rs` |
| Certified NURBS fits of conical screw sweeps (a screw motion whose radius changes linearly with the advance): each point carried with its radial direction, a closed-form fourth-derivative bound, surfaces as the sweep of a curve and its radial net on one span plan (tapered threads) | `nurbs/conical_screw.rs`, `nurbs/conical_screw/tests.rs` |
| Bezier decomposition | `nurbs/decompose.rs` |
| Bezier clipping intersection | `nurbs/bezier_clip.rs` |
| Closed-form oracles for the Bezier-clip helpers and recursion work counts (tests) | `nurbs/bezier_clip/mutation_oracle_tests.rs` |
| Curve/surface fitting (LSPIA) | `nurbs/fitting.rs`, `nurbs/surface_fitting.rs` |
| Bit-identity oracle for the cached-basis LSPIA loop against the verbatim old loop (tests) | `nurbs/fitting/lspia_oracle_tests.rs` |
| Bit-identity oracle for surface LSPIA's best-iterate result against every iterate of the verbatim old loop (tests) | `nurbs/surface_fitting/lspia_oracle_tests.rs` |
| Point projection onto curves | `nurbs/projection.rs` |
| Self-intersection detection | `nurbs/self_intersection.rs` |
| 3D curves (Line, Circle, Ellipse, Parabola, Hyperbola) | `curves.rs` |
| 2D curves (Line2D, Circle2D, Ellipse2D) | `curves2d.rs` |
| Analytic surfaces (Cylinder, Cone, Sphere, Torus) | `surfaces.rs` |
| Bit-identity oracle for the analytic surfaces' trig-taking `point_and_normal_trig` against their verbatim pre-change `evaluate`/`normal` (tests) | `surfaces/trig_oracle_tests.rs` |
| Exact swept-surface carriers (revolution, linear extrusion) | `surfaces/swept.rs` |
| Surface-surface intersection | `nurbs/intersection/` (mod, surface_marching, surface_seeding, curve_surface, plane, line, chaining, tests) |
| Hand-placed oracles for intersection-point chaining: grid walk, adjacency (tests) | `nurbs/intersection/chaining/mutation_oracle_tests.rs` |
| Line/NURBS root completion: span-block hull tree, Bézier extraction, de Casteljau root isolation for crossings the seed grid misses | `nurbs/intersection/line/subdivision.rs` |
| Narrow-bump line crossings against an independent graph-bisection oracle (tests) | `nurbs/intersection/line/bump_oracle_tests.rs` |
| Closed-form oracles for the SSI marcher's private branch geometry (tests) | `nurbs/intersection/surface_marching/oracle_tests.rs` |
| Analytic-analytic intersection | `analytic_intersection.rs` |
| Bit-exact oracle for the ray-torus quartic solver's Durand–Kerner cycle jump (tests) | `analytic_intersection/quartic_cycle_tests.rs` |
| Certified line–torus contacts: the exact centre-relative chart (power-of-two scaling, error-free `origin − centre`), plain and double-double ball arithmetic, Bernstein/Descartes root isolation of the unexpanded quartic `G² − 4R²N`, and the one map to the caller's parameter (`Finisher::settle`): windows with `f64` ends whose exact chart images carry certified signs, pairwise disjoint, resolved only when they hold one root; roots the parameter cannot separate fused into one contact of known parity | `analytic_intersection/line_torus.rs` |
| Ball arithmetic against exact integer arithmetic and cancelling identities, the three round-3 review lines, exact tangencies as unresolved windows, the caller-parameter map against integer arithmetic (and at subnormal and overflowing parameters), every window end's sign read back in the wide quartic (tests) | `analytic_intersection/line_torus/tests.rs` |
| Line–torus roots from origins up to 10 000 tube sizes away and raw directions 1e-5 to 1e4 long, against a bisection reference; an exact tangent's window holding the contact, set by the torus not the origin's distance (tests) | `math/tests/regress_line_torus_far_origin.rs` |
| Line–torus contacts against the double-double reference: the reviewed near-triple ray, seeded asymptotic, exterior, interior and quadruple tangents and lines either side of them, exact tangencies as the only unresolved contacts, every window of every harness family holding exactly its roots, shallow chords as resolved pairs (tests) | `math/tests/regress_line_torus_contacts.rs` |
| Independent double-double line–torus reference (quartic from its quadratics, critical points by bisection, roots bisected on the tube implicit), the seeded line families (and those at the limit of the caller's parameter), the judge that compares `ray_torus` hits with it, and the window judge for the contract of the caller's parameter (test support) | `math/tests/line_torus_reference/mod.rs` |
| Exact sign of the line–torus quartic at an `f64` parameter, in dyadic big-number arithmetic (test support) | `math/tests/line_torus_reference/exact.rs` |
| The line–torus contract in the caller's parameter (review round 4): four crossings eight spacings of `t` apart on a torus `1e15` out, two crossings in the cells either side of one `f64` as one even contact, a far torus shrinking through the spacing of `t` with closed-form windows, an overflowing parameter (tests) | `math/tests/regress_line_torus_caller_windows.rs` |
| Differential harness: `ray_torus` against the reference over chords, near-tangent families either side of exterior, interior, asymptotic and quadruple contacts, thin tori, far centres, far origins and unnormalised directions; every window of `intersect_line_torus_contacts` against the contract of the caller's parameter with exact end signs (opposite across a resolved window holding one root, the stated parity across an unresolved one, disjoint windows, no separable roots in one window), also over tori `1e9` to `1e15` out and grazing lines `1e3` to `1e12` away whose crossings straddle the spacing of `t`; `LINE_TORUS_REPORT=1` prints the per-family tables, `window_contract_soak` (ignored) repeats from further seeds (tests) | `math/tests/differential_line_torus.rs` |
| Bit-exact oracles for the plane × torus section: the pre-change grid scan, greedy chaining and pipeline, grid line sign certificates (tests) | `analytic_intersection/plane_torus_oracle_tests.rs` |
| AABB / bounding boxes | `aabb.rs` |
| BVH (bounding volume hierarchy) | `bvh.rs` |
| CDT (constrained Delaunay) | `cdt/` (mod, insert, locate, constraints, collinear, adjacency, tests, tests/collinear_index, tests/flood_batch, tests/hole_grid, tests/off_axis_cap, tests/u_bracket_floor) |
| Exact integer in-circle sign for the CDT's undecided (cocircular) tests | `cdt/incircle.rs` |
| Differential oracles for the integer in-circle sign and the scanned duplicate lookup against `robust::incircle` and the eager grid (tests) | `cdt/tests/predicate_oracle.rs` |
| Convex hull | `convex_hull.rs` |
| Filtered exact predicates | `filtered.rs` |
| Float tolerance | `tolerance.rs` |
| Geometric predicates (orient2d/3d) | `predicates.rs` |
| Qualified ray–cylinder/cone/sphere intersection (cancellation-free quadratic, relative tangency band) and ray–torus (merged double roots as tangential): transversal/tangential/unclassified hits (RFC 0008 §3.2); `TangentPolicy::DropRelative` carries the GFA classifier's tangent-dropping cylinder/cone roots bit for bit, and `plane_parameter` the shared ray–plane parameter | `ray.rs`, `ray/tests.rs` |
| Ray-triangle intersection | `ray_triangle.rs` |
| 2D polygon offset | `polygon_offset.rs` |
| 2D polygon ops (clip, fillet, chamfer) | `polygon2d.rs` |
| Parametric geometry traits | `traits.rs` |
| NURBS basis function evaluation | `nurbs/basis.rs` |
| NURBS curvature (fundamental forms) | `nurbs/curvature.rs` |
| Surface evaluator (power-basis cache) | `nurbs/evaluator.rs` |
| Polynomial power-basis (Horner evaluation) | `nurbs/power_basis.rs` |
| Orthonormal reference frame | `frame.rs` |
| Surface curvature primitives (sign convention) | `curvature.rs` |
| Oriented bounding box (PCA + SAT) | `obb.rs` |
| Chord deviation arc discretization | `chord.rs` |
| Gauss-Legendre quadrature | `quadrature.rs` |
| Operation tolerance and resource policy | `context.rs` |
| Owned cache scopes: type-indexed cache parts and the thread's ambient scope (RFC 0009) | `context/cache.rs` |
| Deterministic hashing (seed-stable maps/sets) | `det_hash.rs` |
| Stable and unstable sorts compiled once for all comparators (`CompactSort`, for cold sorts; keeps WASM code size down) | `compact_sort.rs` |
| Correctly rounded `fma` that stays fast on `wasm32` (bit-identical to `mul_add`) | `fma.rs` |
| Structured diagnostics (category, code, `ToDiagnostic`) | `diagnostic.rs` |
| Qualified intersection results (contact kind, quality) | `intersect.rs` |
| 2D polygon booleans (union, intersection, difference) | `polygon_boolean.rs` |

### L1: geometry (`crates/geometry/src/`)
| Task | File(s) |
|------|---------|
| Uniform curve sampling | `sampling/uniform.rs` |
| Deflection-adaptive sampling | `sampling/deflection.rs` |
| Arc-length-uniform sampling | `sampling/arc_length.rs` |
| Curvature-adaptive sampling (NURBS) | `sampling/curvature.rs` |
| Surface grid sampling | `sampling/surface.rs` |
| Point-to-curve projection | `extrema/point_curve.rs` |
| Curve-to-curve distance | `extrema/curve_curve.rs` |
| Point-to-surface distance (analytic) | `extrema/point_surface.rs` |
| Segment-segment distance | `extrema/segment.rs` |
| Lipschitz global optimizer | `extrema/lipschitz.rs` |
| Circle/Ellipse/Line → NURBS | `convert/curve_to_nurbs.rs` |
| Analytic surfaces → NURBS | `convert/surface_to_nurbs.rs` |
| NURBS → analytic curve recognition | `convert/recognize_curve.rs` |
| NURBS → analytic surface recognition | `convert/recognize_surface.rs` |
| Algebraic qualification of affine degree-one NURBS plane patches | `convert/certified_plane.rs` |
| Conservative finite-domain bounds with explicit confidence | `bounds/` (mod, curve, surface) |
| Error types | `error.rs` |

### L1: topology (`crates/topology/src/`)
| Task | File(s) |
|------|---------|
| Arena & typed `Id<T>` handles | `arena.rs` |
| `Topology` struct (the arena owner) | `topology.rs` |
| Accounted topology and shared NURBS allocation estimates | `memory.rs` |
| Vertex, Edge, Wire, Face, Shell, Solid | `vertex.rs`, `edge.rs`, `wire.rs`, `face.rs`, `shell.rs`, `solid.rs` |
| Compound, CompSolid | `compound.rs`, `compsolid.rs` |
| Adjacency index (edge-to-face, face neighbors) | `adjacency.rs` |
| Builder helpers | `builder.rs` |
| Shape explorer (iterate children) | `explorer.rs` |
| PCurve registry | `pcurve.rs` |
| Topology validation | `validation.rs` |
| Narrow whole-interval SameParameter certificates | `validation/same_parameter_proof.rs` |
| Test utilities (`test-utils` feature) | `test_utils.rs` |
| Coedge — one directed use of an edge by a face boundary | `coedge.rs` |
| Loop — ordered cycle of coedges bounding a face | `face_loop.rs` |
| Transactional mutation (`run_transacted`, `run_validated`) | `transaction.rs` |
| Mutation-local undo log, scope marks, append-only guard (`run_append_only`) | `undo_log.rs` |
| Chunk/page COW comparison prototype (test-only, never shipped) | `cow_prototype.rs` |
| Nested savepoint state, failure injection, and stale-handle contract tests | `transaction/savepoint_tests.rs` |
| Differential oracle: the handle-slot-preserving restore with the arena restores it had before they were rewritten in place, kept verbatim, plus restore fixed points (tests) | `topology/restore_oracle_tests.rs` |
| Attribute store (names, colors, app ids) | `attributes.rs` |
| Append-only evolution journal | `journal.rs` |
| Persistent references and their resolver | `naming.rs` |
| Exact whole-topology images for session snapshots: every arena slot verbatim (retired tombstones, liveness), pcurve index, attributes, the whole journal with its recorded ticks; borrowed `image_view`, owned `to_image`, structurally validating `from_image` with a fresh cache lineage | `image.rs` |
| Image round trip, next-handle and journal-continuity parity, structural refusals (tests) | `image/tests.rs` |

### L2: algo (`crates/algo/src/`)
| Task | File(s) |
|------|---------|
| GFA entry point | `gfa.rs` |
| Store-local patch GFA (`boolean_patch`/`cut_patch`: face-subset store, tool pieces classified against the whole blank, open-patch cleanup without shell assembly; full assembly for an intersect) | `gfa.rs`, `ds/shape_store.rs`, `builder/mod.rs`, `builder/builder_solid.rs` |
| Boolean operation selection | `bop.rs` |
| Error types | `error.rs` |
| GFA data structures (Pave, PaveBlock, GfaArena) | `ds/pave.rs`, `ds/arena.rs` |
| Intersection curve data | `ds/curve.rs` |
| Face classification state | `ds/face_info.rs` |
| PaveFiller orchestrator | `pave_filler/mod.rs` |
| Shared PaveFiller phase helpers | `pave_filler/helpers.rs` |
| Isolated curved NURBS section clipping and whole-span bounds | `pave_filler/curved_section_clip.rs`, `pave_filler/curved_section_clip/bernstein.rs`, `pave_filler/curved_section_clip/tests.rs` |
| Read-only shared ruled-section discovery with original native traces and certified physical trim windows | `pave_filler/curved_section_clip/discovery.rs` |
| Phases VV/VE/EE/VF/EF/FF | `pave_filler/phase_*.rs` |
| Bounded algebraic events for an actual Line/Sphere carrier pair | `pave_filler/phase_ef/line_sphere.rs` |
| Pave block splitting + edge creation | `pave_filler/make_blocks.rs`, `make_split_edges.rs` |
| FaceInfo population | `pave_filler/fill_face_info.rs` |
| Builder (face splitting + assembly) | `builder/mod.rs`, `builder/assemble.rs` |
| Face splitting (UV-space); exact line/arc planar loop containment (`point_in_planar_loop`) | `builder/face_splitter/` (mod, containment, conversion, edge_splitting, sampling, special_cases), `builder/classify_2d.rs` |
| Exact-curve chart arrangement: certified noding and leaving sectors, region tracing and holes, interior witnesses outside the curve enclosures; distinguishes unsupported inputs from failed certificates | `builder/face_splitter/chart_arrangement.rs` |
| Curve enclosures in plane, cylinder and torus charts: positive-weight Bezier and conic hulls, interval velocity cones, adaptive noding and node-sector proofs (adversarial tests) | `builder/face_splitter/chart_certificate.rs`, `builder/face_splitter/chart_certificate_tests.rs` |
| Torus and cylinder faces opened along their seams, sections and boundary pieces partitioned by the chart arrangement; primary torus path and cylinder rescue, with separate failed-certificate diagnostics | `builder/face_splitter/periodic_face_chart.rs` |
| Positive-weight NURBS sections certified outside planar or cylindrical face material before the left-out rescue check; crossing, hole-domain and separating-direction controls | `builder/face_splitter/section_clearance_tests.rs` |
| Planar line/circle face re-partition when the established split contains spurs, omits sections or fails its area consistency estimate | `builder/face_splitter/plane_chart.rs` |
| Lifted torus and cylinder parameter rectangles recognized from iso-parameter boundary curves; exact periodic chart membership | `periodic_chart.rs` |
| Native equator partition and whole-interval circular rim coverage oracles | `builder/face_splitter/special_cases/equator_partition_tests.rs` |
| Isolated exact-event UV arrangement and cylinder-strip quotient | `builder/face_splitter/arrangement.rs`, `builder/face_splitter/arrangement/` (geometry, regions, periodic) |
| Shadow differential of the legacy special-case emitters against the arrangement adapters (O2.3d step 0; test and `perf-counters` builds only, measures and never changes a result) | `builder/face_splitter/shadow.rs`, `builder/face_splitter/shadow/tests.rs` |
| Production UV arrangement dispatch and guards | `builder/face_splitter/arrangement_prod.rs` |
| Closed-form oracles for the splitter's periodic-lateral and sphere-collar shortcuts (tests) | `builder/face_splitter/closed_form_split_tests.rs` |
| Closed-form oracles for the plane multi-loop gate: circles compared as discs against each other, inner-wire openings and the outer outline (tests) | `builder/face_splitter/closed_loops_separation_tests.rs` |
| Bounded cylinder arrangement adapter | `builder/face_splitter/arrangement_cyl.rs` |
| Closed-form oracles for the rim-chain splitter, the orphaned-rim detector and the cone-lateral dispatch (tests) | `builder/face_splitter/rim_chain_oracle_tests.rs` |
| Hand-placed oracles for splitting a plane-face hole where sections end inside its edges: split points, untouched holes left whole, one hole of a row, only points of an edge split it, and section ends closer than the arrangement's weld refused rather than merged (tests) | `builder/face_splitter/hole_split_tests.rs` |
| Closed-form oracles for the O2.3b arrangement engine: intersections, tangent order, ray crossing, distance, work budgets, periodic quotient (tests) | `builder/face_splitter/arrangement/oracle_tests.rs` |
| PCurve computation | `builder/pcurve_compute.rs` |
| Opt-in per-thread content-keyed memo of planar arc pcurves (bit-identical hits; the WASM kernel enables it) | `pcurve_memo.rs` |
| Opt-in per-thread content-keyed memo of revolved-face wire areas for shell-assembly orientation (bit-identical hits; the WASM kernel enables it) | `wire_area_memo.rs` |
| Bounded content-keyed FIFO store shared by the opt-in memos | `memo.rs` |
| Opt-in per-thread content-keyed memo of the ray classifier's sampled torus trims (bit-identical hits; the WASM kernel enables it) | `ray_geom_memo.rs` |
| Differential oracle for the hole-containment box gate (the ungated test kept verbatim, tests) | `builder/face_splitter/containment/gate_oracle_tests.rs` |
| Plane frame (3D↔UV projection) | `builder/plane_frame.rs` |
| Wire loop reconstruction | `builder/wire_builder.rs` |
| Shared DCEL clockwise-successor connectivity | `builder/rotation_system.rs` |
| Split types + face class | `builder/split_types.rs`, `builder/face_class.rs` |
| Face image population | `builder/fill_images.rs`, `builder/fill_images_faces.rs` |
| Same-domain face merging | `builder/same_domain.rs` |
| Interference indexing | `ds/interference.rs`, `ds/shape_index.rs` |
| Analytic classifier (7 variants) | `classifier/analytic.rs` |
| Ray-cast classifier | `classifier/ray_cast.rs` |
| Closed-form and differential oracles for the ray-cast polygon gate and shared plane hits (tests) | `classifier/ray_cast/gate_oracle_tests.rs` |
| Differential oracles for the ray-cast torus line cull: no quartic roots on a cleared line, ray triples against the unculled vote loop, near and far past the reach cap, near-miss roots only the margin keeps (tests) | `classifier/ray_cast/torus_cull_oracle_tests.rs` |
| Bit-identity oracle for the ray-cast plane, cylinder and cone hits routed through `remus_math::ray`: the pre-routing functions kept verbatim, on random, tangent, near-tangent, axis- and generator-parallel and in-plane rays at scales 1e-3 to 1e6 (tests) | `classifier/ray_cast/quadric_policy_oracle_tests.rs` |
| Recognition of bounded rectangular torus patches | `classifier/torus_patch.rs` |
| Shell assembly (4-phase `BuilderSolid`) | `builder/builder_solid.rs` |
| Boolean preflight diagnostics | `diagnostic.rs` |
| Isolated GFA shape store (deep-copied operands) | `ds/shape_store.rs` |
| Spatial hash of pave-block endpoints | `ds/pave_vertex_index.rs` |
| Post-split EE overlap → `CommonBlock` | `pave_filler/force_interf_ee.rs` |
| Link section blocks to existing boundary blocks | `pave_filler/link_existing.rs` |
| PCurves for split edges on their faces | `pave_filler/make_pcurves.rs` |
| Deterministic work counters (complexity guards) | `perf.rs` |
| Exact axis extremes of circle/ellipse arcs (broad-phase boxes) | `conic_bounds.rs` |
| The length tolerances a planar face's section vertices are held to, stated once for phase FF, the face splitter's welds and shell assembly (band, weld, separation distance), with every weld a later stage applies and the floor that keeps the separation over the fixed ones at any caller tolerance | `section_tolerance.rs` |
| Independent-oracle tests for phase-FF exact-vs-refuse helpers (disc clip, one-hit graze, torus oval trim, one-rim notch) | `pave_filler/phase_ff/helper_oracle_tests.rs` |
| Independent-oracle tests for the N-way region fuse (`fuse_n_regions_with_entity_evolution`): hand-counted unions, the store's caller maps, the store-agnostic lineage export, the per-source face tally, the pre-export gate and the typed refusals | `gfa/nary_oracle_tests.rs` |
| Closed-section window endpoints on the face boundary crossed by a fitted rim, with margin-band and partner-rim fit-wobble controls against independent chord crossings | `pave_filler/phase_ff/closed_window_end_tests.rs` |
| Closed-section clipping retains a tilted cylinder rim crossed near a planar edge, including crossings hidden within the partner margin | `pave_filler/phase_ff/closed_window_partner_tests.rs` |
| Trimmed-torus broad-phase boxes and minor-angle bands: complete positive-weight Bezier and circle hulls, continuous parameter lifts, winding and area-sign checks, conservative carrier fallback (including adversarial knot/weight and ridge tests) | `pave_filler/phase_ff/torus_extent.rs`, `pave_filler/phase_ff/torus_extent/lift.rs`, `pave_filler/phase_ff/torus_extent/certified_tests.rs`, `pave_filler/phase_ff/torus_extent/tests.rs` |
| Torus minor-band provenance: wrap-straddling and uncertain extents never imply whole-tube coverage; only an explicit proof enables strict shortcuts and exact marcher rim exits (tests) | `pave_filler/phase_ff/torus_extent/certified_tests/band_proof.rs` |
| Differential oracle for the phase-FF junction snap's box gate and per-face boundary cache (the ungated search kept verbatim), the box bound against sampled curve distances, and the untouched-face box gate (tests) | `pave_filler/phase_ff/junction_snap_oracle_tests.rs` |
| Differential oracle for the cylinder × NURBS carrier-clearance gate (a control net clear of the cylinder skips the surface marcher): every cleared pair through the unchanged marcher and against sampled closed-form distances, the clearance as the decision boundary, crossing, wrapping and unseparated nets kept, relative clearance far from the origin (tests) | `pave_filler/phase_ff/carrier_clearance_oracle_tests.rs` |
| Certify-or-refuse gate for a closed section of a planar face against its holes and outline: closed-form zones where the section runs within the tolerance band of a loop EDGE over its own extent (never its carrier), the side of each gap between them, crossings placed at exact roots or corner vertices and carried at the parameter they were decided at, and the one decision for every span: one that ends on a hole kept only when certified on the face, one that ends on none kept only when certified to pass through no hole (read holes by their zones and gaps, unread ones by certified clearance), refused by name otherwise | `pave_filler/phase_ff/plane_section_certificate.rs` |
| Closed-form oracles for a plane × cylinder section against the planar face's holes: line contact (miss, touch, cross), square and round hole crossings, corners, spans kept on a shelled rim, the extent-margin leak, the pinch refusal, the closed-circle collector on an annulus, and the review reproducers (an undecided probe, an odd crossing count, crossings the splitter would weld, length-not-angle at radius 5e6 and eccentricity 1000, a short arc on the section's own circle, arcs within 1e-9 to 1e-3 of it decided at the ends of the overlap) (tests) | `pave_filler/phase_ff/plane_hole_tests.rs` |
| Seeded differential oracle for the section certificate: random polygonal, sliver, near-vertex, chamfered, tiny and circular holes, loops with a short arc on or beside the section's own circle, wedges with a corner 1e-9 to 1e-5 off an eccentric section and unread round holes, against circles and ellipses (eccentricity up to 1000) at scales 1e-3, 1 and 1e6, with crossings from a double-double reference and a dense ladder-sampled check of the kept spans against an even–odd reference; accepted and refused counts per family (tests) | `pave_filler/phase_ff/plane_section_oracle_tests.rs` |
| Phase-FF regressions on a face of many spans (a helical sweep side over three turns): the broad-phase box holds the bulge past its rails, and plane sections seed every span and come back whole (tests) | `pave_filler/phase_ff/multi_span_section_tests.rs` |
| Independent-oracle tests for builder-solid helpers (conic support, closed-pair traversal, revolved wire winding, cylinder faces with an odd number of rims refused, rims counted by their geometry whatever their parameter's speed, bounded walls of every make passed, unread loops left alone) | `builder/builder_solid/helper_oracle_tests.rs` |
| Differential oracle for the NURBS arc refinement's knot-span box prefilter: it rejects only vertices the unchanged search rejects (tests) | `builder/builder_solid/arc_refine_oracle_tests.rs` |
| Independent-oracle tests for fill-images section helpers (cap discs, closed-section containment, curved-face segment classifier) | `builder/fill_images_faces/helper_oracle_tests.rs` |

### L2: blend (`crates/blend/src/`)
| Task | File(s) |
|------|---------|
| Public API, `BlendError`, `BlendResult` | `lib.rs` |
| Radius law (constant, linear, S-curve, custom) | `radius_law.rs` |
| Spine (edge chain, arc-length parameterization) | `spine.rs` |
| Cross-section (contact points, center, radius) | `section.rs` |
| Stripe (fillet band, contact curves, PCurves) | `stripe.rs` |
| Blend constraint functions | `blend_func.rs` |
| Newton-Raphson walking engine | `walker.rs` |
| Closed walked rims resampled at uniform stations into C¹ cubic contacts, pcurves and band rows, measured on their supports within the recorded linear tolerance | `closed_walk.rs` |
| Cooperative cancellation and the deterministic walker step budget a governed blend operation answers to (scoped, sticky trips; no-op when ungoverned) | `governor.rs` |
| Analytic fast paths (plane-plane, plane-cyl, etc.) | `analytic.rs` |
| Fillet builder (orchestration) | `fillet_builder.rs` |
| Chamfer builder (orchestration) | `chamfer_builder.rs` |
| Exact chamfers along tangent and mitred edge chains on one planar floor (plane strips, cones on arcs) | `chamfer_floor_chain.rs` |
| Exact straight-ridge chamfers on proven spline extrusion supports | `chamfer_extrusion.rs` |
| Ruled chamfers on monotone spline rims of linear extrusions | `chamfer_spline_rim.rs` |
| Whole-carrier hole clearance for spline chamfers | `chamfer_clearance.rs` |
| Vertex blend / corner solver | `corner.rs` |
| Face trimming along contact curves | `trimmer.rs` |
| Copy-on-write boundary rewrites, so a blend never edits faces its input still uses | `body_edit.rs` |
| Rebuild a rounded planar cap against joined supports with explicit sampled-corner permission; read back a sampled circular-section carrier's stations and centre field (`sampled_canal`, `sampled_section_centre`) | `cap_reblend.rs` |
| Measured reads of circular-section carriers for recognition: walked stations to the walk's radius tolerance with collapsed end sections left out, lofted arc carriers' sections read off the surface, and a nearest-station seeded projection (`sampled_canal_measured`, `sampled_section_centre_measured`, `sampled_project_measured`); `MeasuredCarrier` keeps one read for repeated queries | `cap_reblend/measured.rs` |
| Restore retained sampled shoulder limits before reversing a joined-cap edit on an owned disposable candidate, including a walked corner (a smaller convex band run up the shoulder) and the seam round it stands on, whose ruling runs back to the spring | `cap_reblend/restore.rs` |
| Restore the full analytic rounded cap after a recovered shoulder separates from its rim (the periodic seam on an existing back-side foot vertex, else where the wall's own `u = 0` ruling first meets its boundary below the rim) | `cap_reblend/unjoin.rs` |
| Share the analytic shoulder spring with its fixed rim at zero-width contact onset | `cap_reblend/touch.rs` |
| Rolling sections for a boss seam crossing an equal-radius convex round | `equal_radius_boss_corner.rs` |
| Shared builder utilities | `builder_utils.rs` |
| G1-continuous fillet edge-chain expansion | `g1_chain.rs` |
| One band along a concave tangent chain whose second face changes (a boss seam over an earlier round): supports switched at bisected stations, the rail trimmed across its seam edge and closed back to the chain end | `rail_chain.rs` |
| Fillet or chamfer along a boss's seam with a block whose face turns sharp corners (or runs round the boss): exact cylinder/torus (fillet) or plane/cone (chamfer) bands per run, mitred where runs meet, ends notched into or grown up to the face the chain runs into | `seam_chain.rs` |
| One walked band along a convex tangent chain whose faces change on both sides (a block's top edge run into an earlier seam band): each side handed over at the bisected station where its contact reaches the seam, run out to a point where the last two faces are tangent; chamfer as the ruled band between the ball's contacts | `ridge_chain.rs` |
| Verbatim per-call G1 chain expansion kept as an oracle for the shared `G1ChainIndex` (tests) | `g1_chain/tests.rs` |
| Read-only analytic blend queries | `query.rs` |
| Exact face-face blend bands for disjoint planar support patches | `face_face.rs` |
| Geometry-aware snap tolerance for assembly dedup | `adaptive_tolerance.rs` |
| Spherical triangle corner patches (3+ stripes) | `spherical_triangle.rs` |

### L2: heal (`crates/heal/src/`)
| Task | File(s) |
|------|---------|
| Public API, `HealError`, `Status` | `lib.rs`, `error.rs`, `status.rs` |
| Shell mutation ownership checks | `shared.rs` |
| Healing context (tolerance, reshape, messages) | `context.rs` |
| Entity replacement/removal tracking | `reshape.rs` |
| Edge analysis (3D/PCurve deviation, degeneracy) | `analysis/edge.rs` |
| Wire analysis (ordering, closure, gaps, small edges) | `analysis/wire.rs` |
| Surface analysis (singularity, closure, equivalence) | `analysis/surface.rs` |
| Face analysis (small face, degenerate face) | `analysis/face.rs` |
| Shell analysis (manifold, free edges, orientation) | `analysis/shell.rs` |
| Curve analysis (length, continuity) | `analysis/curve.rs` |
| Free boundary detection | `analysis/free_bounds.rs` |
| Wire edge ordering | `analysis/wire_order.rs` |
| Tolerance statistics | `analysis/tolerance.rs` |
| Entity counting | `analysis/contents.rs` |
| Fix config (tri-state FixMode per fix type) | `fix/config.rs` |
| Fix orchestrator (shape → solid → shell → face → wire → edge) | `fix/mod.rs` |
| Edge fixing (SameParameter, vertex tolerance) | `fix/edge.rs` |
| Wire fixing (reorder, gaps, small, degenerate) | `fix/wire.rs` |
| Face fixing (orientation, small area, seam insertion) | `fix/face.rs` |
| Duplicate-face recognition and removal | `fix/duplicate_faces.rs` |
| Shell fixing (orientation consistency) | `fix/shell.rs` |
| Solid fixing (top-level orchestrator) | `fix/solid.rs` |
| Small face removal | `fix/small_face.rs` |
| Wireframe repair | `fix/wireframe.rs` |
| Split over-connected vertices | `fix/split_vertex.rs` |
| Conservative spatial index for coincident-vertex candidate discovery | `fix/vertex_merge.rs` |
| Unify same-domain faces (merge coplanar/co-cylindrical) | `upgrade/unify_same_domain.rs` |
| Curve splitting at continuity breaks | `upgrade/split_curve.rs` |
| Surface splitting | `upgrade/split_surface.rs` |
| NURBS → Bezier conversion | `upgrade/convert_to_bezier.rs` |
| Remove internal wires | `upgrade/remove_internal_wires.rs` |
| Shell sewing (stitch free edges) | `upgrade/shell_sewing.rs` |
| 3D → 2D PCurve projection | `construct/project_curve.rs` |
| Curve type conversions | `construct/convert_curve.rs` |
| Surface type conversions | `construct/convert_surface.rs` |
| Convert all to B-Spline | `custom/convert_to_bspline.rs` |
| B-Spline degree/continuity restriction | `custom/bspline_restriction.rs` |
| Recognize NURBS as elementary surfaces | `custom/convert_to_elementary.rs` |
| HealOperator trait | `pipeline/operator.rs` |
| Operator registry | `pipeline/registry.rs` |
| Configurable pipeline executor | `pipeline/process.rs` |
| 13 built-in operators | `pipeline/builtin.rs` |
| Collapse orphan collinear interior wire vertices | `upgrade/collapse_collinear_vertices.rs` |
| Canonicalize a split-arc rim into one closed edge | `upgrade/merge_split_rim_arcs.rs` |
| Split figure-8 / pinched wires into simple cycles | `upgrade/split_self_intersecting_wires.rs` |
| Shared hollow-block and cavity-gap fixtures for the crate's unit tests (tests) | `test_fixtures.rs` |

### L2: check (`crates/check/src/`)
| Task | File(s) |
|------|---------|
| Public API, `CheckError` | `lib.rs`, `error.rs` |
| Shared utilities (face polygon, AABB, edge sampling) | `util.rs` |
| Ray-surface intersection (all types + solvers) | `classify/ray_surface.rs` |
| UV boundary polygon, containment tests | `classify/boundary.rs` |
| Point-in-solid classification (ray casting) | `classify/mod.rs` |
| Winding number classifier | `classify/winding.rs` |
| Operation-local prepared context for repeated point classification (`PreparedSolid`) | `classify/prepared.rs` |
| Conservative NURBS span hulls for boundary-distance and ray pruning, with bounded quadratic-strip subdivision for long narrow bands; the original surface and solver remain authoritative | `classify/prepared/ray_prefilter.rs` |
| Persistent whole-topology classification cache (`ClassificationCache`, PERF-Q02) | `classify/cache.rs` |
| Narrow-bump far-flank crossing through the classifier's NURBS ray path (tests) | `classify/boundary/nurbs_bump_ray_tests.rs` |
| Torus crossings over exterior, interior, triple and quadruple line contacts: a ray counts with the right parity or abstains, never miscounts; a contact whose window reaches the ray origin abstains (tests) | `classify/boundary/torus_contact_tests.rs` |
| Exact trim of sphere faces bounded by circle arcs on the sphere, any number of supports, non-convex or reversed (first crossing along a great-circle walk) | `classify/boundary/sphere_arcs.rs` |
| Boundary skeleton that makes a ray through an edge or vertex abstain from the classification vote | `classify/graze.rs` |
| CheckId enum, ValidationReport, severity | `validate/checks.rs` |
| Wire topological checks | `validate/wire.rs` |
| Shell topological checks | `validate/shell.rs` |
| Solid checks (Euler, duplicate faces) | `validate/solid.rs` |
| Vertex geometric checks | `validate/vertex.rs` |
| Edge geometric checks | `validate/edge.rs` |
| Face geometric checks | `validate/face.rs` |
| Validation orchestrator | `validate/mod.rs` |
| Profile-region adapter to the shared certified inner-loop reader, its parent tree and planar winding parity; preserved trim-bound oracles (tests only) | `validate/planar_regions.rs`, `validate/planar_regions/trim_bounds_tests.rs` |
| GProps accumulator (Huygens' theorem) | `properties/accumulator.rs` |
| Closed-form formulas (box, sphere, cylinder, etc.) | `properties/analytic.rs` |
| AABB computation | `properties/bbox.rs` |
| Face Gauss integration, with area/volume/vector-flux-only orientation accumulation | `properties/face_integrator.rs` |
| Recover a knot-budget refusal by sweeping a nonperiodic trim the other way | `properties/face_integrator/knot_axis.rs` |
| Independent planar oracles for a trim crossing many spline knot spans (tests) | `properties/face_integrator/anisotropic_tests.rs` |
| NURBS faces certified as their carrier's whole parameter rectangle, so the fixed rule integrates them untrimmed: complete degree-(2,1) rectangles from iso-row and circular-section boundary images; ruled degree-(1, q) full-turn bands whose rows are whole circle edges closed on one straight seam; any clamped carrier whose wire runs along its four iso sides (NURBS edges either way round, lines, circles; gear and thread flanks); and bands closed in one direction between two whole rims and one seam (in-place helical sweeps) | `properties/face_integrator/natural_rectangle.rs` |
| Bounded native Circle latitude domains, independent of trim sampling | `properties/face_integrator/sphere_latitude.rs` |
| Bounded sampling of native-contiguous exact equator runs | `properties/face_integrator/sphere_latitude/partition_sampling.rs` |
| Bounded h-refinement on an already resolved rectangular analytic domain | `properties/face_integrator/adaptive.rs` |
| Bucketed trim-loop segment lookup for trimmed quadrature (O06) | `properties/face_integrator/loop_index.rs` |
| Exact boundary reduction (Green's theorem) of cylinder, cone, sphere and torus face areas behind `analytic_face_area`, with the trimmed quadrature as fallback | `properties/face_integrator/boundary_area.rs` |
| Exact moments (volume, centroid, inertia) of a cylinder face trimmed only by lines, circles and ellipses, by Green's theorem on its own boundary; the sampled trim quadrature stays the guard and the path for fitted trims | `properties/face_integrator/cylinder_green.rs` |
| Bit-identity oracle for the NURBS row and grid quadrature (per-axis basis tables) against the verbatim per-abscissa `integrate_parametric` (tests) | `properties/face_integrator/nurbs_rows_oracle_tests.rs` |
| Bit-identity oracle for `integrate_parametric` with its sliced request out of line and its knot-aligned path reached through `as_nurbs`, against the verbatim previous function (tests) | `properties/face_integrator/parametric_split_oracle_tests.rs` |
| Full-turn cylinder bands with rims split into many arcs, integrated over their exact `(u, v)` rectangle past the trim budget (tests) | `properties/face_integrator/cylinder_band_tests.rs` |
| Full-turn cylinder bands with rims split into many arcs, integrated over their exact `(u, v)` rectangle past the trim budget (tests) | `properties/face_integrator/cylinder_band_tests.rs` |
| Independent polynomial Green-moment and authoritative-trim oracles (tests) | `properties/face_integrator/polynomial_boundary_tests.rs` |
| Planar faces with nested inner wires (hole, island, hole) alternate void and material: closed-form rectangle sums and signed single-loop compositions of every moment, exact and fan paths (tests) | `properties/face_integrator/nested_hole_tests.rs` |
| Differential oracle: on planar faces whose inner loops do not nest (lines, circles, split arcs, ellipses, trimmed and rational NURBS, a shallow arc of a far circle, overlapping boxes; flat, tilted and translated by 1e6 and 1e9 times the scale), the exact and fan paths match their pre-nesting subtract-every-hole versions, kept verbatim, bit for bit (tests) | `properties/face_integrator/flat_loop_oracle_tests.rs` |
| Certified nesting of a planar face's inner loops (`inner_loop_nesting`), with immediate parent indices and explicit caller-certified raw profile ranges in the report path (stored trims always win; default measurement stays strict): `Flat`, `Nested(depths)`, `Invalid` (contact within the edges' tolerances certified: subtract every hole, as always) or `Undecided` (refused, `CheckError::UndecidedLoopNesting`; operations measures the face on its mesh). Each wire read over its trims in a frame local to the face (NURBS spans inside the trim only, by blossoming), every piece enclosed with its certified chord deviation and an evaluation-error bound from the carrier's local magnitude, circles as annuli, containment by exact `orient2d` parity at an on-curve probe, disjointness of each loop and its container and of overlapping siblings certified before depths are reported, box-sweep pairing, and a work limit proportional to the pieces read | `properties/loop_nesting.rs` |
| Nesting depths and parent trees on shuffled branches, no partial parents on undecided/invalid readings, explicit raw ranges without document changes or relaxation of stored trims, and nearly touching curved loops in every order and edge split, trims that hide carrier spans, touching and crossing loops certified invalid, the review reproducers (200 glyph pairs, loops a billionth apart, unused NURBS spans, translated squares, a shallow arc of a 1e12 circle), shallow-arc families from radius 1e6 to 1e15, translated families, and linear cost in the number of pairs (tests) | `properties/loop_nesting/tests.rs` |
| The face-integral cache against the uncached integrator on nested faces, primed in both orders: identical content bit for bit, translated copies to rounding, and no translated reuse across a different nesting (tests) | `properties/loop_nesting/cache_tests.rs` |
| Properties orchestrator (volume, area, CoM) | `properties/mod.rs` |
| Opt-in content-keyed memo of fixed-order face integrals (`FaceIntegralCache`, PERF-V02 subset) | `properties/face_cache.rs` |
| Knot-aligned basis-reuse bit oracle, independent ruled-generator quadrature and closed-form volume/flux oracles (tests) | `properties/face_integrator/knot_rows_oracle_tests.rs` |
| Point-to-surface distance (all analytic types) | `distance/analytic.rs` |
| Edge-to-edge distance | `distance/edge.rs` |
| Point-to-solid, solid-to-solid distance | `distance/mod.rs` |
| Exact planar trim membership for line and circle boundaries (nested holes alternate), shared by face distance and the classifier's plane hits | `distance/plane_trim.rs` |
| Bounded global maximum-distance witnesses for planar polyhedra and complete spheres | `distance/maximum.rs` |
| Conservative face bounds for distance pruning | `distance/face_bounds.rs` |
| Operation-local prepared context for repeated point-to-solid distance (`PreparedDistanceSolid`, `DistanceScratch`, `DistanceOptions`) | `distance/prepared.rs` |
| Non-finite geometry detection | `validate/finite.rs` |
| Surface curvature interrogation (pointwise + min radius) | `analyze/curvature.rs` |
| Signed draft-angle samples and constant-angle reports for planar solids (native) | `analyze/draft.rs` |
| Deterministic work counters for the point-classification hot path and for face integration (planar readings, knot-aligned counting passes and Gauss lines; `perf-counters` feature) | `perf.rs` |

### L2: offset (`crates/offset/src/`)
| Task | File(s) |
|------|---------|
| Public API (offset_solid, thick_solid) | `lib.rs` |
| Error types | `error.rs` |
| Central data structures (OffsetData, EdgeClass, etc.) | `data.rs` |
| Edge/vertex convexity classification | `analyse.rs` |
| Per-face surface offset (analytic + NURBS) | `offset.rs` |
| 3D face-face intersection | `inter3d.rs` |
| 2D edge splitting from intersections | `inter2d.rs` |
| Wire loop reconstruction | `loops.rs` |
| Arc joints (pipe + sphere cap) | `arc_joint.rs` |
| Shell assembly + solid creation | `assemble.rs` |
| Self-intersection removal (BOP-based) | `self_int.rs` |
| Cavity (inner shell) extent checks | `cavity.rs` |
| Shell-interference certificates (slab collapse, cavity clearance, cavity inside the outer shell) | `interference.rs` |
| Topology-preserving planar face moves | `move_faces.rs` |
| Quarter-cylinder replacement clearance | `move_faces/quarter_cylinder.rs` |
| Clearance for coaxially resizing a general partial-turn boss wall | `move_faces/partial_cylinder.rs` |

### L2: sketch (`crates/sketch/src/`)
| Task | File(s) |
|------|---------|
| GCS system (CRUD, solve orchestration, `solve_detailed`) | `gcs/system.rs` |
| Constraint types (26 variants, Jacobians) | `gcs/constraint.rs` |
| Entity arena (Points, Lines, Circles) | `gcs/entity.rs` |
| DogLeg trust-region solver | `gcs/solver.rs` |
| Bounded sparse coupled-sketch solver | `gcs/sparse.rs` |
| QR factorization (rank detection) | `gcs/qr.rs` |
| DOF analysis | `gcs/dof.rs` |
| Solve diagnostics + classification | `gcs/diagnostics.rs` |
| Solve-local final-iterate evaluation sharing | `gcs/final_eval.rs` |
| Independent-component decomposition (PERF-S02) | `gcs/components.rs` |
| Error types | `lib.rs` (`SketchError`) |

Adding a `Constraint` variant requires an arm in `residual_count`,
`eval_residuals`, and `eval_jacobian` (all `gcs/constraint.rs`), plus
`validate_constraint` and the four `constraint_references_*` helpers in
`gcs/system.rs`, plus `direct_refs` in `gcs/components.rs` (structural free
parameters for the PERF-S02 decomposition). All are exhaustive matches, so the
compiler flags every site. Expose it to JS through `parse_gcs_constraint` in
`wasm/src/bindings/gcs_sketch.rs`. Every analytic Jacobian needs a
finite-difference test; `gcs/constraint/tests.rs` carries helpers for both a
fixed step (`check_jacobian_fd`) and a scale-relative one
(`check_jacobian_central`) — use the latter when testing at large
coordinates, where a fixed 1e-7 step loses too much to cancellation.

### L3: operations (`crates/operations/src/`)
| Task | File(s) |
|------|---------|
| Primitives (box, cylinder, cone, sphere, torus) | `primitives.rs` |
| Independent carrier and manifold oracles for the sphere equator | `primitives/sphere_equator_tests.rs` |
| Boolean (union/cut/intersect) | `boolean/` (mod, types, classify, assembly, tests, nway_pairs_tests, disjoint_witness_tests) |
| One-sided material contradiction detector for established Boolean results built after dropped sections; grid agreement is not a certificate | `boolean/material_witness.rs` |
| Three-box union grouping and order regression: exact volume, solid validation, closed oriented mesh, material probes, source preservation and STEP replay (OpenZCAD 1246) | `tests/regress_issue_1246_box_union_grouping.rs` |
| Nested inner wires measured with alternating material: a standalone face's area and a pocketed block with a standing, holed island (face area, volume, centre of mass, surface area against signed boxes) (tests) | `tests/nested_inner_wire_measure.rs` |
| Certified inner-loop nesting through the public measures at three scales: an island nearly filling its hole for every edge split, eccentric circles nearly touching, an elliptical island, three levels in every order, reversed and tilted faces, a trimmed NURBS hole whose unused spans surround its neighbour, and a pocketed cylinder whose `solid_volume`, `solid_properties` and `face_area` agree, read the same warm and cold, and fall back to the mesh when the nesting is left undecided (tests) | `tests/certified_loop_nesting_measure.rs` |
| The planar nesting contract end to end: the second review's reproducers through pre-existing entry points, the mesh fallback against the tessellator, and a seeded randomized differential of nested layouts (lines, arcs, circles, ellipses, trimmed and rational NURBS, up to four levels, translated, tilted, reversed) against their closed forms (tests) | `tests/nested_loop_contract.rs` |
| Round cutters on the wall of a shelled box (the rim's cavity opening is a hole of the section's face): closed-form volumes, validation, watertight meshes, edge-use and face censuses for cuts and fuses across scales, two walls in one cut, round bore rims on plates and tubes, tangent and pinched contacts, the L-bracket inner-corner cut, the off-centre post fuse, eccentric sections, and the review reproducers through the public boolean (exact or a typed refusal): notched slots, grazed sides, needles, a short arc of the cutter's own circle, a hole corner off an eccentric section, a crowded spline rim | `tests/regress_shelled_wall_rim_notch.rs` |
| Differential sweep of 1260 round-cutter booleans per scale on the shelled box, a bored plate and a tube, one outcome line per case; run against a baseline dump it fails on a case that was exact and is not, or was refused and is accepted with a defect (ignored; public API only) | `tests/sweep_shelled_wall_rim_notch.rs` |
| Qualified local drilled-hole cut (through and blind holes; stages 1–2 of the local boolean) and its GFA oracle tests | `boolean/local_cut.rs`, `boolean/local_cut_tests.rs` |
| Opt-in store-local cut, fuse and intersect: the GFA on the faces the tool touches, spliced onto the untouched ones (stage 4 of the local boolean), and its equivalence oracle tests | `boolean/store_local.rs`, `boolean/store_local_tests.rs` |
| Mesh boolean (co-refinement) | `mesh_boolean.rs` |
| Robust triangle-triangle intersection: the narrow phase shared by mesh co-refinement and validation's component-overlap check | `triangle_intersection.rs` |
| Extrude, Revolve, Sweep, Loft, Pipe | `extrude.rs`, `revolve.rs`, `sweep.rs`, `loft.rs`, `pipe.rs` |
| Extrusion nesting preflight, separate island caps, and closed-form material/void regressions | `extrude/nesting.rs`, `extrude/nesting_tests.rs` |
| Main-comparison profile families, unnested sweep compatibility, disjoint ellipse/spline holes with overlapping boxes and split-circle seam alignment (tests) | `extrude/profile_compatibility_tests.rs` |
| Exact loft between parallel full circles: coaxial cylinder/cone bands, and laterally offset (oblique) circles as one exact rational ruled band per section pair with circular rims | `loft.rs` (`try_loft_coaxial_circle_stack`, `try_loft_parallel_circle_stack`), `loft/tests.rs` |
| Twisted extrusion: the straight extrusion's topology with the top cap turned, fitted helix edges and certified swept side faces (lands concentric with the axis stay exact cylinders while they sweep less than a full turn, and are swept like the rest past it); herringbone: two opposite-hand halves glued at their shared mid-plane edges | `extrude/twist.rs`, `extrude/twist/tests.rs` |
| Conical twisted extrusion (a tapered thread's ring segment): the straight extrusion's topology swept by a screw motion that also moves every profile point along its radius; concentric arcs become exact cone faces, radial lines ruled surfaces, spline edges certified sweeps carried with the caller's radial direction nets, handed on to the top edges for the next segment | `extrude/conical_twist.rs`, `extrude/conical_twist/tests.rs` |
| Tapered extrusion toward an apex: the straight extrusion's topology with the far cap scaled about the apex; lines sweep to planes through it, concentric circles to exact cones, other curves to ruled NURBS (a concentric hole stays a cylinder) | `extrude/taper.rs`, `extrude/taper/tests.rs` |
| Involute spur, helical, herringbone, internal (ring) and rack gears as one extruded (helical: twisted; herringbone: twisted both ways) outline: dimensions, warnings, typed refusals, certified flank fit | `gear.rs` |
| Gear outlines in the local frame: involute flanks, closed-form root fillets, tooth replication, rack profile | `gear/profile.rs` |
| Gear oracles: closed-form dimensions, exact-involute Green area, pitch-circle thickness probes, fit bound, rack trapezoid volume, construction face roles, refusals (tests) | `gear/tests.rs` |
| Helical gear oracles: ISO 21771 closed forms, Cavalieri volumes, flank points against the exact involute helicoid, hand and twist direction, refusals (tests) | `gear/tests/helical.rs` |
| Herringbone gear oracles: Cavalieri volumes, each half's flank points on its own involute helicoid, far cap in line with the start cap, unique half-qualified roles, refusals (tests) | `gear/tests/herringbone.rs` |
| Straight bevel gears: pitch cone from the tooth ratio and shaft angle, certified fit of the spherical involute centrally projected onto the flat heel, tapered toward the apex | `gear/bevel.rs` |
| Bevel gear oracles: closed-form cone angles and mate pitch angles, cap areas against the exact spherical-involute heel outline, apex-sweep volumes, flank points on the exact spherical-involute cones, pitch-cone classification, roles, refusals (tests) | `gear/bevel/tests.rs` |
| Cylindrical involute (ZI) worms: the involute helical gear with the starts as teeth and helix angle 90° − lead angle (a many-turn twisted extrusion), axial-module conventions, and the crossed-helical wheel that meshes with it | `gear/worm.rs` |
| Throated worm wheels without a boolean: each tooth tip rebuilt as two outside-cylinder strips and an exact torus patch, each flank stopped at the throat along a certified fit of its torus intersection (fourth-derivative bound carried through Faà di Bruno) | `gear/throat.rs` |
| Double-enveloping (Hindley) worms without a boolean: the hourglass blank cut by the wheel's straight-sided teeth swept with it; lands and root floors as exact torus patches between rails that are lines in the torus's `(u, v)`, cut every half turn; flanks as ruled patches between certified rail fits, split along straight rulings at the same cuts; end caps meeting the tori in exact circles and the flanks along certified section fits | `gear/enveloping.rs` |
| Interval bounds on a function and its first four derivatives through compositions and products (the gear fits' fourth-derivative bounds) | `gear/jet.rs` |
| Enveloping worm oracles: volumes against an exact interval integral of the axial sections (hourglass blank less straight-sided teeth), every sampled point's material in closed form, closed-form dimensions, hand, unique roles, refusals (tests) | `gear/tests/enveloping.rs` |
| Double-enveloping (Hindley) wheels: the worm used as a hob (`HobSweep`: material margin over every mesh angle, tabulated once per height and radius, bisected hobbed edges), flank laws fitted per side of the mid-plane (straight in every section, exact in the mid-plane, creased along it as the hobbed tooth is) by least squares raised until no sample is cut, each flank two ruled faces between certified rail fits, tips on the throat torus and outside cylinder | `gear/hindley_wheel.rs` |
| Hindley wheel oracles: volumes against a section integral of the straight-sided laws, the exact mid-plane tooth by classification, clearance from the worm used as a hob, the worm's backlash leaving the wheel unchanged, unique roles, refusals (tests) | `gear/tests/hindley_wheel.rs` |
| Differential oracle for the hob sweep: the depth and hobbed-edge search before the blank-margin skip, four-largest selection and per-edge table, kept verbatim; depths bit for bit, edges to the same outcome within 16 ulps on five worms and the probe wheel's fit grid, built wheels (vertices, NURBS control points, flank laws, relief, volume, counts) within 1e-12 of the verbatim build, the table's sample count against the per-point sweep's, and a margin-evaluation count guard (tests) | `gear/hindley_wheel/hob_oracle_tests.rs` |
| Worm oracles: closed-form dimensions, Cavalieri volumes, flank points on the exact involute helicoid, axial thread thickness and pitch read off the body by classification, hand, the matching wheel's dimensions and clearance; throated wheels' section-integral volumes (exact truncated-profile areas), throat and rim classification, throat curves on the torus, roles, refusals (tests) | `gear/tests/worm.rs` |
| Gear teeth on an existing boss or bore, along the whole face or over a band of it with collars either side: keep-diameter or given-module fit, exact tooth-band material proof, tube cut, then the gear ring glued on entity for entity as the fuse builds it (free ends and shoulders clear of the tips) or fused on one coaxial cylinder, circles started on the phase reference, roles carried through either | `gear/cylinder.rs` |
| Teeth-on-cylinder oracles: standalone-gear and closed-form volumes on a boss, a stepped bored shaft and a bored ring, flush and turned-down teeth over the middle of a hub or boss, turn-down fit and phase probes, internal teeth over part of a bore, any phase reference, cross-hole and span-outside-face refusals, the glued ring against the fused one (faces, loops, edges, vertices, roles, volume), flanks clear of coaxial walls never marched (tests) | `gear/cylinder/tests.rs` |
| Helical sweep: the legacy path sweep along a fitted helix (`helix.rs`), and the exact in-place screw sweep — each profile edge's exact NURBS swept into one round rational surface over every turn (rational quadratic arc spans, only the axial rise approximated within a proven bound), shared rails, rigidly moved end cap, refusals for axis contact and overlapping coils | `helix.rs`, `helix/in_place.rs` |
| Screw threads on an existing boss or bore: a trapezoidal form (independent flank angles, crest and root flats, starts, hand) swept as one twisted extrusion of its transverse outline (crest and root arcs, certified Archimedean-spiral flanks), stacked in half-turn segments, put on by a tube cut and a ring glue (shared with gear teeth through `glue_coaxial_ring`) or fuse after the band is proven solid | `thread.rs`, `thread/profile.rs` |
| Tapered (pipe) threads on a cylinder: crest and root cones and conical screw flanks (`R = f(z − θ/k) + τ·z`), flank widths from the axial-section angles, sized at the gauge plane and checked against the cylinder at both ends; the NPT form and the flat-approximated BSPT form | `thread.rs`, `thread/profile.rs` |
| Cone chamfers at thread ends built into the ring without a boolean: a frustum below the root crossing, and above it the zone's twisted extrusion with crest faces dropped, flanks cut to triangles along certified conical-helix fits of the flank–cone curves, and one seamed cone face | `thread/chamfer.rs` |
| Cone runouts at thread ends that stop short of the face's end (or meet a shoulder), built into the ring without a boolean: the zone's twisted extrusion with root faces dropped, flanks cut to the triangle on the crest side of the same conical-helix trims, the cone over each groove cut along generators into pieces of at most half a turn (never across the root arc it shares with the full thread), and an end cap out to the crest circle | `thread/runout.rs` |
| Thread oracles: closed-form outline-area volumes (external, internal, multi-start, left hand, square flank, sharp crest), collars, chamfered ends against a quadrature of the exact cone-clipped thread, runouts against their closed-form section integral (flush and stepped collars, boss and bore, both ends, a shoulder, meeting runouts), face roles, the runout generator cuts, tapered threads against the NPT and BSPT tables (diameters on the gauge plane and at both ends read off the body, cone lands, closed-form volumes, a female thread from its mouth, taper with runout and chamfer ends), typed refusals (tests) | `thread/tests.rs` |
| Chamfer, Fillet | `chamfer.rs`, `fillet/` (mod, rolling_ball, g1_chain, geometry, radius_law, helpers, notch_torus, tests) |
| Retained support-crown regression: exact rational ellipse, non-unit curve domain, unchanged source and closed-form volume at three scales | `fillet/retained_support_tests.rs` |
| Exact clipping of a complete orthogonal corner after earlier blends | `chamfer/curved_corner.rs` |
| Exact analytic chamfer cutters for ridges the boundary rebuild refuses | `chamfer/analytic_tools.rs` |
| Exact complete chamfers of trivalent orthogonal shells, including inner boundaries | `chamfer/orthogonal.rs` |
| Exact piecewise-analytic sweep tools joined across identical end sections | `chamfer/analytic_tools/joined_sweeps.rs` |
| Rolling-ball arc stripe between a plane and a coaxial cylinder as an exact torus, or a sphere triangle at the band's own radius (a fillet riding an earlier band's end) | `fillet/arc_torus.rs` |
| Two convex stripes mitred over a reflex corner whose concave edge stays sharp | `fillet/mitre.rs` |
| Lone straight rolling-ball stripes ending at a strictly convex planar corner: contacts run out past the vertex onto one side plane, or onto two (a chamfered box's mitred corner) with the end boundary kinked where the band crosses their shared edge | `fillet/rolling_ball/split_runout.rs` |
| Spatial grid behind the rolling-ball Phase 5d vertex dedup and snap (the linear scans' winners and ties), with its oracle tests | `fillet/vertex_snap.rs` |
| Shell (hollow solid) | `shell_op.rs` |
| Draft (taper faces) | `draft.rs` |
| Section / Split | `section.rs`, `split.rs` |
| Exact solid-face imprint with journal history | `imprint.rs` |
| Bounded interior planar polygon imprint with complete construction history | `imprint/polygon.rs`, `imprint/polygon/interval.rs` |
| Directional curve projection onto faces, solids, sketch planes (P-Class 7.4: sweep-plane sections, coaxial circles, fitted-arc cell, clipping/seams, sketch-plane maps) | `project_curve.rs` |
| P-Class 7.4 acceptance oracles (exact cells, approximate disclosure, refusals, solid/2D paths) | `tests/qualify_project_curve.rs` |
| Small cylindrical resizes: thin-sleeve cut and fuse below the chord sagitta, rim-rounded bores carrying their rounds, closed-form volumes (tests) | `tests/resize_cylinder_small_delta.rs` |
| Torus notches thousands of units along long box edges and on 3e-4 edges against slicing-integral volumes, and interior points of large torus-grooved cubes classified (tests) | `tests/regress_torus_far_line_roots.rs` |
| Rays tangent to a whole torus and a fillet's torus band (top/bottom circles, equators, tube, both horizontal rays touching) keep the GFA ray-cast parity: seeded sweeps against the closed-form solid, through the GFA and `remus-check` classifiers (tests) | `tests/regress_torus_tangent_ray_parity.rs` |
| Resolved line–torus contacts through their consumers: points a hair inside and outside large thin tori through the GFA and `remus-check` classifiers, tori far along a ray whose crossings the ray's parameter can or cannot separate, a box edge grazing a torus cut exactly (edge split at the closed-form crossings) or refused, and the open shallow-notch repro (tests) | `tests/regress_torus_contact_windows.rs` |
| Transform, Mirror, Copy | `transform.rs`, `mirror.rs`, `copy.rs` |
| Pcurves carried into a transformed face's UV chart (exact plane chart map, revolved-carrier `u` rotation, re-derived lift counts) | `transform/pcurves.rs` |
| Map-based solid copy paths kept verbatim as differential oracles for `CopyPlan` (tests) | `copy/tests/legacy_oracle.rs` |
| Spliced copy's aligned coedge-authority restore checked against the per-snapshot search on a drilled plate (tests) | `copy/tests/spliced_authority_oracle.rs` |
| Measure (bbox, area, volume, CoM) | `measure/` (mod, volume, area, bounding_box, edge_length, helpers) |
| Opt-in per-thread `solid_volume` memo keyed by topology cache identity and complete solid content | `measure/volume_memo.rs` |
| Prepared-handle source and sharp-parent per-face guard readings, keyed on exact content, order and reference | `measure/guard_readings.rs` |
| Exact reusable face-area and handle-independent volume measurements | `measure/reuse.rs`, `measure/reuse/tests.rs` |
| Closed-form certificate that a straight-edged, axis-aligned planar outline passes the volume path's area-consistency probe without meshing | `measure/volume/planar_outline.rs` |
| Certificate-implies-probe oracle, refusal cases and probe-mesh work guard (tests) | `measure/volume/planar_outline_tests.rs` |
| Hash-map references for the sorted whole-mesh edge censuses (tests) | `measure/volume/edge_census_oracle_tests.rs` |
| Long open spline arcs retain their real winding, rather than their chord's (tests) | `measure/volume/periodic_winding_tests.rs` |
| Bounded opt-in exact-content scalar memo for trimmed NURBS face areas | `measure/nurbs_area_memo.rs`, `measure/nurbs_area_memo/tests.rs` |
| 128-bit exact content digest of a single face (handle-free; lets a consumer keep what it derived from faces an edit left untouched) | `measure/face_digest.rs` |
| Bounded opt-in exact-content memo of single-face bounding boxes (the move's swept-region refusal), and its bit-identity tests | `measure/face_box_memo.rs`, `measure/face_box_memo/tests.rs` |
| Test assertions that measure volumes and areas (tests) | `measure/test_support.rs` |
| Planar faces with several inner loops, flat and nested, flat, tilted and translated far from the origin, for the measurement paths' nesting oracles (tests) | `measure/loop_fixtures.rs` |
| Differential oracles: `planar_face_signed_volume` and the sampled planar area match their pre-nesting subtract-every-hole versions, kept verbatim, bit for bit on unnested loops, and add islands back on a nested stack (tests) | `measure/volume/flat_loop_oracle_tests.rs`, `measure/area/flat_loop_oracle_tests.rs` |
| Bounded opt-in host-clock phase timings | `performance.rs` |
| The thread's default cache scope, shared by kernels and the `*_thread_*` cache switches (RFC 0009) | `cache.rs` |
| Distance queries | `distance.rs` |
| Independent global maximum-distance oracles (tests) | `distance/maximum_tests.rs` |
| Tessellation | `tessellate/` (mod, face, planar, nonplanar, nurbs, solid, edge_sampling, mesh_ops, tests, tests/face_cache, tests/mesh_oracles, tests/mutation_oracles) |
| Differential oracle for the interior grid's boundary scans: the `(i + 1) % n` indexed containment and clearance loops kept verbatim (tests) | `tessellate/nonplanar/polygon_scan_oracle_tests.rs` |
| Whole display meshes with the NURBS chart frame built over shared and over copied knots and weights, bit for bit (tests) | `tessellate/nonplanar/frame_storage_oracle_tests.rs` |
| Freshly generated swept bodies (threads, an in-place coil, a helical gear) and ordinary curved ones for the tessellator's bit-identity oracles (tests) | `tessellate/swept_fixtures.rs` |
| Bit-identity oracle for the NURBS edge sizing measures that carry each segment's end over to the next, with pinned evaluation counts (tests) | `tessellate/edge_measure_oracle_tests.rs` |
| Metric (first-fundamental-form) chart for an open NURBS face whose parameter lines run nearly parallel, e.g. a thread flank: interior grid and CDT in surface lengths; the averaged-metric `v` scale every other NURBS CDT chart triangulates with | `tessellate/nonplanar/sheared_chart.rs` |
| Collapsed NURBS domain sides (a pole, or a blend rail stopping at a corner of its own radius) read from the control net, and monotone chart placement of the CDT boundary samples on them | `tessellate/nonplanar/collapsed_boundary.rs` |
| One chart boundary for a torus face whose wires touch its outer wire: cycles sharing a vertex with it (seam slits and period-winding band boundaries left by face unification) spliced into one boundary walk | `tessellate/nonplanar/touching_cycles.rs` |
| Thread flank display-mesh shading and density regressions (tests) | `tessellate/tests/sheared_chart.rs` |
| In-place helical sweep (coil) display meshes: no side facet folds into the tube, no slivers (tests) | `tessellate/tests/coil_chart.rs` |
| Projection retry for a NURBS face whose coarse seed grid fails: a narrow strip swept many turns, retried from a node in every knot span (tests) | `tessellate/nonplanar/frame_projection_tests.rs` |
| Interior grid samples reused at emission: meshes with the reuse on and off compared bit for bit (tests) | `tessellate/nonplanar/interior_sample_tests.rs` |
| Signed native vector-area qualification for a hemisphere chart | `tessellate/nonplanar/sphere_chart_orientation.rs` |
| Independent spherical-patch geometry and shared-boundary oracles | `tessellate/nonplanar/sphere_chart_tests.rs` |
| Row-wise interior-grid containment against the per-point polygon test (tests) | `tessellate/nonplanar/row_containment_tests.rs` |
| Pre-cache band emitter kept verbatim as a bit-identity oracle for its per-vertex outward reference (tests) | `tessellate/nonplanar/band_emit_oracle_tests.rs` |
| Measurement-only tessellation and cold/warm byte parity tests | `tessellate/measurement_tests.rs` |
| Whole-mesh passes kept verbatim as byte-identity oracles for the rewritten `mesh_ops` and `pool_index` (PERF-D07, tests) | `tessellate/tests/mesh_passes.rs` |
| One-pass boundary and non-manifold edge census (`edge_census`) against the hash-set and hash-map counts it replaced, kept verbatim (tests) | `tessellate/mesh_ops/edge_census_oracle_tests.rs` |
| Per-vertex analytic grid kept verbatim as a bit-identity oracle for the trig-table `tessellate_analytic` (tests) | `tessellate/tests/analytic_grid_oracle.rs` |
| Explicit shared-boundary plan and deterministic refinement reconciliation | `tessellate/boundary_plan.rs` |
| Exact boundary/CDT reuse and incremental mesh connectivity | `tessellate/reuse.rs`, `tessellate/reuse/incremental.rs` |
| Content-keyed per-face mesh reuse across edits, with display-priority retention inside shared bounds (opt-in, per thread; PERF-D01) | `tessellate/face_cache.rs` |
| Spatial index of the shared vertex pool for the boundary plan's circle contact refinement (PERF-D07) | `tessellate/pool_index.rs` |
| Differential oracle for the circle contact candidates: the 27-key neighbour expansion kept verbatim against the merged z-run lookup (tests) | `tessellate/pool_index/oracle_tests.rs` |
| Cost-model full-scan fallback of the circle contact index: decision boundary and fallback-on/off mesh identity (tests) | `tessellate/tests/pool_fallback.rs` |
| Point classification (in/on/out solid) | `classify.rs` |
| Operation-local reuse of identical robust-classification meshes | `classify/mesh_memo.rs` |
| Bounded solid-angle intervals for the robust classifier's existing thresholds | `classify/winding_tree.rs` |
| Offset face / solid | `offset_face.rs`, `offset_v2.rs` (delegates to remus-offset), `offset_trim.rs` |
| Offset wire | `offset_wire.rs` |
| Face filling (Coons patch) | `fill_face.rs` |
| Shape healing | `heal.rs` |
| Defeaturing | `defeature.rs` |
| Feature recognition | `feature_recognition.rs` |
| Pattern (linear/circular) | `pattern.rs` |
| Shape queries (edge filtering) | `query.rs` |
| Oracle for the prepared `ConcavityProbe` against the one-shot edge concavity on primitives, booleans and STEP bodies (tests) | `query/concavity_probe_oracle_tests.rs` |
| Opt-in exact-content memo of the edge-concavity probes' per-face ray crossing counts, a part of the owner's cache scope (RFC 0009 stage 1), so relations and recognition after an edit recount only changed faces (the WASM kernel enables it); a guard-private two-pass store for the convex material guard | `query/crossing_memo.rs` |
| Sewing | `sew.rs` |
| Thickening | `thicken.rs` |
| Untrimming | `untrim.rs` |
| Validation | `validate.rs` |
| Polygon approximations of wires and faces (`face_polygon`, `wire_polygon`; re-exported from `boolean`) | `polygon.rs` |
| Types carried inside `OperationsError` variants (`HealingReport`, `ResizeBlendError`; re-exported at their original paths) | `error_payloads.rs` |
| Assembly management | `assembly.rs` |
| 2D sketch constraint solver | `sketch.rs` |
| Persistent planar sketch attachments and external line inputs | `linked_sketch.rs` |
| Evolution tracking | `evolution.rs` |
| Result-aware edge/vertex evolution induced from an exact face map (typed unresolved reasons, F/E/V completeness) | `boundary_evolution.rs` |
| Compound operations | `compound_ops.rs` |
| Blend v2 wrappers (fillet/chamfer) | `blend_ops.rs` |
| Immutable deferred measuring obligations that certify adoption of a private prepared preview | `blend_ops/preview_checks.rs` |
| Differential oracle for the fillet feature grouping: the pairwise polyline grouping before its connected-pair skip, box prefilter and early-exit scan, kept verbatim (tests) | `blend_ops/grouping_oracle_tests.rs` |
| Sphere-boss seam fillet work counters (no whole-solid robust mesh, pinned walker steps) and the governed fillet cancellation/step-budget contract with rollback (tests) | `blend_ops/governed_fillet_tests.rs` |
| Convex blend material-side acceptance (its input and result classification passes share per-face crossing counts of the faces the blend left unchanged) | `blend_material.rs` |
| Exact standalone face-face blend sheets | `face_face_blend.rs` |
| Offset v2 (delegates to remus-offset) | `offset_v2.rs` |
| Shared winding utilities | `winding.rs` |
| Edge projection with hidden-line removal | `projection.rs` |
| Exact silhouettes of cylinder, cone, sphere and torus faces (torus branches sampled adaptively), trimmed and split by the same occlusion probe; NURBS faces' traced ones, reported approximate | `projection/silhouette.rs` |
| NURBS silhouette tracing: `n·d` sampled on a knot-aligned grid, marching squares, crossings bisected onto `n·d = 0`, refined to the deflection | `projection/silhouette/nurbs.rs` |
| Closed-form oracles for silhouettes: side, axial and oblique views, seam rulings, bores across scales, notch trims, the torus condition and torus views, and traced NURBS cylinders, spheres and tori against their exact outlines (tests) | `projection/silhouette_tests.rs` |
| Shared loft/sweep/pipe/revolve end caps | `cap.rs` |
| Journal ingestion for modeling operations | `journal_ops.rs` |
| Direct push/pull face editing | `push_pull.rs` |
| Conservative radial separation proofs for periodic face sweeps | `push_pull/radial_clearance.rs` |
| Coaxial resize of a full-turn cylindrical wall whose rims meet a sphere off-axis (re-intersected rims, sliding rim vertices) | `push_pull/sphere_rim_resize.rs` |
| Face moves with explicit permission and quality disclosure for sampled cap reconstruction | `face_move_quality.rs` |
| Prepared face moves: repeated moves of one source selection to new distances, distance-independent analysis read once (`PreparedFaceMove`) | `prepared_move.rs` |
| Work-count guards for interactive face moves: writes independent of unrelated bodies, constant slots per drag tick (tests) | `prepared_move/tests/scaling.rs` |
| Exact support-surface replacement with topology-preserving re-limitation | `replace_surface.rs` |
| Exact constant-radius blend-band resizing; a bore resize that carries its rim rounds (`resize_cylindrical_face_with_blends`) | `resize_blend.rs` |
| Rolling-ball seam blends between a full-turn cylindrical wall and a sphere: exact torus / measured freeform-band recognition, exact sharp-seam removal, and the transactional wall resize that re-blends at the source band's quality (`resize_cylindrical_face_reblended`) | `resize_blend/sphere_seam.rs` |
| Topology-preserving radial edits of closed analytic surfaces of revolution | `resize_blend/radial_resize.rs` |
| Exact meridian collision and volume checks for the qualified radial cell | `resize_blend/radial_resize/profile.rs` |
| Exact axial travel limits between a moving shoulder blend and a full fixed rim on their shared cylindrical support | `resize_blend/axial_clearance.rs` |
| Exact annular chamfers that travel with a planar cap and slide on a bore | `resize_blend/cap_chamfer.rs` |
| Approximate cap reconstruction after a translating shoulder reaches a fixed rim, or a rounded cap descends into its fixed shoulder, with construction history | `resize_blend/axial_rim_join.rs` |
| Closed-form vector area of a translating region's construction boundary for the swept-volume sign guard | `resize_blend/translation_shadow.rs` |
| Fillets onto earlier bands: remove them, fillet, rebuild largest first, so fillet order does not matter (the cascade's last fallback) | `resize_blend/fillet_reorder.rs` |
| Differential oracle for the blend move's translation path: the whole-document scratch-copy version kept verbatim, also with its copy-validation refusal forced through a test hook (tests) | `resize_blend/translation_oracle_tests.rs` |
| Blend-translation move carrying an imported edge's stamped endpoint certificate through the strict arena I/O gate (tests) | `resize_blend/endpoint_certificate_tests.rs` |
| Exact local reconstruction of analytic wounds | `local_wound.rs` |
| Cylindrical blend removal with certified affine NURBS caps | `affine_blend_caps.rs` |
| Atomic removal of explicitly selected connected blend groups | `remove_blends.rs` |
| Read-only blend-cluster recognition for a direct edit: the dependent blend group of a seed face (bands, exact and sampled corner patches measured against the rolling-ball closed form, walked transitions continuing an exact band, lofted arc patches, and equal-distance plane/cone chamfer stripes as bands or corners), each member's supports, radius (or chamfer distance) and convexity, the closed-form sharp edges and vertices its removal restores, and a re-blend order (fillets, then chamfer chains) | `blend_cluster.rs`, `blend_cluster/sharp.rs` |
| Blend-cluster recognition on the Gentle-Gecko STEP against its sharp parent, the native boss shoulder at R4/R5, seam bands (with ridge-walked transitions joining their top bands) and bored plates, with typed refusals for freeform faces (tests) | `tests/blend_cluster.rs` |
| Exact removal of a recognized blend group (or a dependency-closed part of it) back to its sharp parent: member faces deleted whole (corner patches never pierced), plane/cylinder supports kept on their carriers, closed-form sharp edges and vertices, retained wires re-cut in their old order, face/edge/vertex and sharp-feature history for re-filleting | `blend_cluster/remove.rs` |
| Blend-group removal: Gentle-Gecko returns its sharp STEP face for face (8/19/13, volume to 1e-9) and re-fillets back; the native shoulder at R4/R5 returns its blank; rim-only removal keeps the other blends; dependency, stale-cluster and sphere-support refusals leave the arena byte-identical (tests) | `tests/blend_cluster_remove.rs` |
| Staged re-blend of a sharp restoration: candidate stage plans from the recognized order and radius overrides (equal radii merged into one call first; chamfer stages last, as tangent-chain or plain chamfers), later stages' edges found again by their closed-form carriers, composed face history (carrier-identity inference where the engine reports no origins), strict-validation and watertight-mesh postconditions, typed `ReblendFailed` naming every attempt | `blend_cluster/reblend.rs` |
| Direct edits through a blend group, run on a private arena: a band radius edit (`resize_blend` falls back to it when the per-band path refuses), a cylindrical support resize (`resize_cylindrical_face`'s partial-turn fallback: remove group, exact sharp wall resize, edges carried through its history, re-fillet at recorded radii), and a new fillet whose chain runs into a group (`fillet_cascade`'s fallback after the reorder: the new edges join the re-fillet plan in the planner's order) | `blend_cluster/edit.rs` |
| Differential oracle: the group a route recognized on its private copy, carried back to the caller's solid, against recognizing the caller's solid again; boss resizes record the same history either way (tests) | `blend_cluster/edit/source_cluster_tests.rs` |
| Prepared blend-group edits for live drags (`PreparedGroupEdit`): recognition, removal to sharp and the learned per-band/partial-turn refusal read once; preview frames clone the sharp arena and re-blend with display-only blends under a closed-shell + strict-validation gate; commits run the unprepared route under every guard (plain and journaled) | `blend_cluster/prepared.rs` |
| Group edits on a body whose rounded cap was lowered into its shoulder with sampled permission: the cap is lifted clear on the private copy, the ordinary group route edits the lifted body, and the cap is lowered back with the permission read off its own sampled rim bands | `blend_cluster/joined_cap.rs` |
| Prepared band-radius and boss-diameter drags on the filleted Gentle-Gecko and the cumulative body: every frame equal to the unprepared edit (F/E/V, census, volume 1e-9) and on the verification bar, commits equal to the unprepared and journaled edits, cancel byte-identical, staleness and typed refusals (tests) | `tests/prepared_group_edit.rs` |
| Per-phase timing of blend-group edits and prepared drags on the Gecko and the cumulative body (example) | `examples/blend_group_edit_profile.rs` |
| Per-phase timing of every geometry generator (primitives, extrude, revolve, loft, sweep, pipe, helix, gears, threads): construction, display tessellation and `solid_volume` medians with face and triangle counts (example) | `examples/geometry_generation_profile.rs` |
| Prepared blend-group edit prepare/frame/commit beside the unprepared edit (bench) | `benches/prepared_group_edit.rs` |
| The video's cumulative workflow on the filleted Gentle-Gecko: the boss diameter after the bottom chamfer chain, after a walked A2 band, after the A3 round and after the free-end chamfer chain, each against its fresh construction; A3 pinned on the 22.5 mm edge; a round larger than the band it crosses through the group; typed refusals byte-identical (tests) | `tests/blend_group_cumulative.rs` |
| The video's full order on one body after A1 shortens the rimmed cap to z26 into the shoulder: A2 R1.9, A3, A4 (the lone edge names its chain), A6 (seed chain and the twelve-edge closed outline, closed-form volume), A7 14.6/17/21.1 preview/14.7, each against its fresh construction with the cap lowered last; typed refusals (tests) | `tests/video_parity_full_order.rs` |
| Blend-group direct edits on the filleted Gentle-Gecko against fresh-cascade oracles: convex top band R4 -> 1.5/1.9 on each side, boss diameter 12 -> 14.6 -> 17 -> 21.1 -> 14.7 chained (and with an R1 cap rim), typed refusals for the R2 concave seam and oversized diameters with the body unchanged (tests) | `tests/blend_group_edit.rs` |
| The journaled boss resize through the Gentle-Gecko blend group pinned to its pre-reuse results (F/E/V, census, volume), its recognition count, `MeasuredCarrier` queries against fresh reads bit for bit, and display-only resizes building the checked bodies (tests) | `tests/blend_group_resize_reuse.rs` |
| Split-band blend fixtures shared by the blend-editing tests (tests) | `resize_blend/test_fixtures.rs` |
| Full-turn rim chain recognition | `tessellate/rim_chain.rs` |
| Outward surface normals of face carriers at a 3D point (shared by the mesh assembler and the fillet engine) | `tessellate/surface_normal.rs` |
| Sphere patches holding a pole (loop winds once around the axis) | `tessellate/sphere_pole_patch.rs` |
| Pure-topology test assertions (Euler characteristic, manifoldness, point distance, volume conservation); `test-utils` feature for downstream tests | `test_utils.rs` |

### L3: io (`crates/io/src/`)
| Task | File(s) |
|------|---------|
| FreeCAD saved-shape ASCII BREP read and bounded planar loop classification | `brep/mod.rs`, `brep/reader.rs`, `brep/planar.rs` |
| Saved-shape analytic line/circle and plane/cylinder carriers, rigid location words, bounded placed occurrences, circular caps and cylindrical seam bands | `brep/reader/carriers.rs`, `brep/reader/locations.rs`, `brep/reader/placed.rs`, `brep/reader/curved.rs` |
| STEP read/write | `step/reader.rs`, `step/writer.rs` |
| Verbatim STEP string-decoder oracle, deterministic fuzz, linear-work guards and hostile-name imports (tests) | `step/reader/directive_oracle_tests.rs` |
| STEP export seam placement: a cylindrical band whose seam runs through one of its own inner wires is written, on a private arena copy, with the seam and its rim vertices moved into the widest free angular gap | `step/writer/seam.rs` |
| Whole-sphere `VERTEX_LOOP` faces: an independently produced one-face sphere imports as the native two-hemisphere sphere, framed and as a void; other vertex-loop layouts refuse atomically (tests) | `tests/step_vertex_loop_sphere.rs` |
| Overlapping-cylinder unions export a seam clear of every window in both orders and after reload, from native and independently produced inputs (tests) | `tests/step_crossed_seam_export.rs` |
| An independently produced box with `same_sense = .F.` planes shells like the native box, open and closed (tests) | `tests/step_reversed_plane_shell.rs` |
| Nested-inner-wire top face of `oring_nested_holes.step`: face area, solid volume, mass properties and centre of mass against rounded-square closed forms, warm and cold through the face cache, and on the mesh when the nesting is left undecided (tests) | `tests/regress_measure_nested_holes.rs` |
| IGES read/write | `iges/reader.rs`, `iges/writer.rs` |
| STL read/write/import | `stl/reader.rs`, `stl/writer.rs`, `stl/import.rs` |
| 3MF read/write | `threemf/reader.rs`, `threemf/writer.rs` |
| OBJ read/write | `obj/reader.rs`, `obj/writer.rs` |
| PLY read/write | `ply/reader.rs`, `ply/writer.rs` |
| glTF read/write | `gltf/reader.rs`, `gltf/writer.rs` |
| Exact arena (sub-topology) serialization | `arena_io.rs` |
| Hand-written arena document JSON codec (`serde_json`-exact reads and writes) | `arena_io/codec.rs` |
| Differential oracle: the derived `serde_json` arena codec kept verbatim (tests) | `arena_io/codec_oracle_tests.rs` |
| Minimal non-generic JSON reader/writer shared by the arena and reference codecs | `json.rs` |
| Differential tests of the JSON reader/writer against `serde_json`, shared mutation fuzzer (tests) | `json/oracle_tests.rs`, `json/mutation.rs` |
| Resource limits for untrusted imports | `limits.rs` |
| Whole-session snapshot codec: magic, format version, build identity, canonical little-endian payload of a topology image (raw `f64` bits), 128-bit digest; all-or-nothing read with limits | `snapshot.rs`, `snapshot/payload.rs` |
| Snapshot round trip over every carrier, determinism, framing and limit refusals, the pcurve-copy budget on repeated pcurve back-references, re-sealed single-byte edits refused or accepted but never a panic (tests) | `snapshot/tests.rs` |
| Persistent-reference JSON codec | `naming_io.rs` |
| Differential oracle for the persistent-reference codec (tests) | `naming_io/oracle_tests.rs` |
| Differential oracles over the STEP fixtures and synthetic bodies for the prepared classifier's exact skips: the boundary test's trim-window prune and the crossing count's NURBS knot-span prefilter, with its skips counted on either side of its span cap (tests) | `tests/boundary_prune_oracle.rs` |
| Differential oracles on the hammer holder for the ray classifier's torus-trim memo (boolean result arenas with it off, cold and warm) and torus line cull (ray triples against the unculled vote loop) (tests) | `tests/hammer_ray_geom_oracle.rs` |
| Differential oracle on the hammer holder for the edge-concavity crossing memo: relations and recognition across two face moves and a cut, memo off against memo carried on (tests) | `tests/hammer_crossing_memo_oracle.rs` |
| The cylinder rim guard over the fixture corpus: no cylinder face of any STEP or arena fixture is counted as having no end along its axis, and nine in ten are read (tests) | `tests/cylinder_rim_guard_corpus.rs` |

### L4: render (`crates/render/src/`)
| Task | File(s) |
|------|---------|
| Public API, `RenderOpts`, `RenderOutput` | `lib.rs` |
| Camera and view/projection matrices | `camera.rs` |
| Solid → GPU vertex/index/edge buffers (per-face indexed) | `mesh.rs` |
| wgpu device setup, render passes, readback | `pipeline.rs` |
| Reusable offscreen rendering session (sync + async submit/poll/wait) | `session.rs` |
| Asynchronous selective readback (tickets, selection, bounded pool) | `async_frame.rs` |
| Explicit prepared-render assets for camera-only frames | `prepared.rs` |
| GPU compute mesher for analytic quadrics | `compute_mesh.rs` |
| Interactive viewer (feature = `window`) | `viewer.rs` |
| Error types | `error.rs` |

### L4: wasm (`crates/wasm/src/`)
| Task | File(s) |
|------|---------|
| `RemusKernel` struct, constructor, private helpers | `kernel.rs` |
| Error types, validation helpers (`validate_positive`, `validate_finite`) | `error.rs` |
| Shape type wrappers (`JsMesh`, `JsEdgeLines`, `JsPoint3`, `JsVec3`) | `shapes.rs` |
| Entity handle resolution (`resolve_*`) & ID converters | `handles.rs` |
| Shared free functions, constants (`TOL`), 2D polygon helpers | `helpers.rs` |
| Checkpoint & sketch state structs | `state.rs` |
| Typed result structs (`GroupedMeshResult`, `UvMeshResult`, tsify) | `types.rs` |
| **Binding modules** (`bindings/`) | |
| Primitive solid creation | `bindings/primitives.rs` |
| Shape construction (vertices, edges, wires, faces) | `bindings/shapes.rs` |
| Modeling operations (extrude, revolve, sweep, loft, fillet, etc.) | `bindings/operations.rs` |
| Offset-circle ruled and two-section smooth lofts through direct and both batch routes: exact circular boundaries, curved side, validity and independent volume oracle (tests) | `bindings/oblique_circle_loft_tests.rs` |
| Boolean operations (fuse, cut, intersect) | `bindings/booleans.rs` |
| Typed modifier results (`moveFacesDetailed`, `resizeCylindricalFaceDetailed`, `filletDetailed`, `chamferDetailed`, `shellDetailed`, `offsetDetailed`, and the `filletV2`/`chamferV2`/`chamferDistanceAngle`/`filletVariable` twins) | `bindings/modifier_detailed.rs` |
| `faceContentDigests` contract: drag ticks keep the digests of untouched faces (tests) | `bindings/face_digest_tests.rs` |
| Source-relative live face moves preserve accepted geometry on refusal (tests) | `bindings/face_move_preview_tests.rs` |
| Batch `getFaceNormal` returns the outward normal of reversed planar faces and refuses curved faces, matching the direct method (tests) | `bindings/face_normal_contract_tests.rs` |
| Differential oracle: the modifier twins' full-copy rollback envelope kept verbatim, against the rollback scope (tests) | `bindings/modifier_detailed/envelope_oracle_tests.rs` |
| Bounded fillet twins (`filletDetailedWithCancellation`, `filletV2DetailedWithCancellation`): cancellation token, `maxWalkSteps` budget, typed `cancelled`/`resource_limit` refusals, `details.walkSteps` | `bindings/modifier_detailed/governed.rs` |
| Typed construction results (`extrudeDetailed`, `revolveDetailed`) | `bindings/construction_detailed.rs` |
| Certified spline-island success and undecided nesting refusals retaining the unsupported category and document/session state through direct and batch routes (tests) | `bindings/profile_nesting_tests.rs` |
| Gear construction from a JSON spec: new bodies (`makeGearDetailed`), straight bevel gears (`makeBevelGearDetailed`), worms and their wheels, plain or throated (`makeWormDetailed`, `makeWormWheelDetailed`), double-enveloping worms and their wheels (`makeEnvelopingWormDetailed`, `makeEnvelopingWormWheelDetailed`) and teeth on an existing cylinder (`cutGearTeethDetailed`), direct and batch | `bindings/gear.rs` |
| Screw threads on an existing cylinder from a JSON spec (`cutThreadDetailed`), direct and batch, with chamfered and runout ends, tapered (pipe) threads, `faceRoles` and `thread:<reason>` refusals | `bindings/thread.rs` |
| Live planar polygon imprint, direct and both batch adapters | `bindings/construction_imprint.rs` |
| Transform, copy, mirror, pattern | `bindings/transforms.rs` |
| Typed affine transform results and refusals | `bindings/transform_detailed.rs` |
| Topology query, edge/surface evaluation | `bindings/query.rs` |
| Every edge of a filleted body lies on each face it bounds, read through `getEdgeCurveType`, `sampleEdge` and `projectPointOnSurface`: quarter-arc rim, mixed-seam and union-junction fillets (tests) | `bindings/blend_edge_coherence_tests.rs` |
| Measurement (volume, area, bbox, distances) | `bindings/measure.rs` |
| Read-only global maximum-distance binding | `bindings/maximum_distance.rs` |
| Read-only signed draft-angle queries over live session handles | `bindings/draft_analysis.rs` |
| Tessellation & wireframe | `bindings/tessellate.rs` |
| Exact arena document bindings (`serializeSolids`, always compiled) | `bindings/arena.rs` |
| File I/O import/export (`#[cfg(feature = "io")]`; off in the shipped kernel package) | `bindings/io.rs` |
| Shape healing, validation, feature recognition | `bindings/heal.rs` |
| Checkpoint / restore | `bindings/checkpoint.rs` |
| Whole-session snapshots (`snapshot`, `fromSnapshot`, `snapshotDigest`, `snapshotBuildId`): the topology verbatim plus the session handle counters, `snapshot:<code>` refusals | `bindings/snapshot.rs` |
| Snapshot contracts: handle and counter parity, stale-handle refusals, journaled edits byte-identical after a restore (primitives and the Gentle-Gecko STEP), refusals of transient, poisoned, truncated, corrupt, foreign-format and foreign-build input (tests) | `bindings/snapshot_tests.rs` |
| Prepared face moves for interactive drags (`prepareMoveFaces`, `moveFacesPrepared`, `moveFacesPreparedJournaled`, `releasePreparedMove`) | `bindings/prepared_move.rs` |
| Prepared blend-group edits for live drags (`prepareResizeBlend`, `prepareResizeCylindricalFace`, `previewPreparedGroupEdit`, `commitPreparedGroupEdit`, `commitPreparedGroupEditJournaled`, `releasePreparedGroupEdit`) | `bindings/prepared_group_edit.rs` |
| Prepared Gecko boss and band drags through checkpoint restores against the unprepared and journaled calls, typed refusals and the live-handle cap (tests) | `bindings/prepared_group_edit/tests.rs` |
| Imported Gecko A2/A6/A7 journaled direct edits, inferred handle continuity and preview cancellation (tests) | `bindings/gecko_edit_tests.rs` |
| Accounted checkpoint storage, allocated WASM pages and phase timing bindings | `bindings/performance.rs` |
| Boundary rollback fault matrix for every mutating family (batch arms, typed modifier twins, direct transactions, `fromBREP`) and retained session-state tests | `bindings/transaction_tests.rs` |
| 2D sketch constraint solver | `bindings/sketch.rs` |
| `fuseWithEntityEvolution` / `cutWithEntityEvolution` keep a round boss's seam lineage on the direct and batch routes: ring pieces `generated`, the surviving seam piece `modified` from the boss seam (tests) | `bindings/seam_edge_lineage_tests.rs` |
| Sketch input validation contracts: legacy `angle` second line, add-time deferred-constraint checks, positive GCS ellipse semiaxes (tests) | `bindings/sketch_validation_tests.rs` |
| Directional curve projection (P-Class 7.4 direct + batch, `project-curve:<code>` refusals) | `bindings/project_curve.rs` |
| Assembly management | `bindings/assembly.rs` |
| 2D polygon operations | `bindings/polygon2d.rs` |
| NURBS curve/surface manipulation | `bindings/nurbs.rs` |
| Read-only certified NURBS carrier reduction and cubic fitting | `bindings/nurbs_reuse.rs` |
| Batch execution & dispatch | `bindings/batch.rs` |
| Optional `executeBatch` arguments: absent reads as the default, present but malformed is refused (contract tests) | `bindings/batch_optional_args_tests.rs` |
| Differential oracle: the full-copy batch rollback dispatcher kept verbatim, in lockstep with the rollback scope (tests) | `bindings/batch_rollback_oracle_tests.rs` |
| Gridfinity integration tests | `bindings/gridfinity_tests.rs` |
| Fillet fail-closed contract tests | `bindings/fillet_fail_closed_tests.rs` |
| Browser-facing spline chamfer contracts through both batch envelopes | `bindings/chamfer_freeform_tests.rs` |
| Sequential fillet onto an earlier blend band (contract test) | `bindings/sequential_fillet_tests.rs` |
| Direct and both batch WASM qualification of `fuseAll` on crossing cylinders (bar and drill, centred, off-centre, equal radius, three scales): split, validator-clean and equal to the pairwise `fuse` | `bindings/cylinder_cross_fuse_all_tests.rs` |
| Direct (`cutDetailed`/`fuseDetailed`) and both batch WASM qualification of box × complete-torus booleans cut normal to the torus axis: Pappus closed-form volumes at three scales, both fuse orders, and the tilted cut refused rather than returned unchanged | `bindings/box_torus_cut_tests.rs` |
| Direct (`intersectDetailed`/`cutDetailed`/`fuseDetailed`) and both batch WASM qualification of the measured volume of equal-radius perpendicular cylinders against the Steinmetz closed form at three scales, both operand orders | `bindings/steinmetz_measurement_tests.rs` |
| Direct (`cutDetailed`/`fuseDetailed`) and both batch WASM qualification of a round cutter through the wall of a shelled box: closed-form volumes, validation, watertight mesh and analytic census at three scales | `bindings/shelled_wall_notch_tests.rs` |
| In-place helical sweep of a circle (esaueng/remus#1623): each cap one closed circle, one side face whose self-owned seam `getSolidSmoothEdges` reports smooth, `filletDetailed`/`chamferDetailed` on either cap rim through the batch ops (contract tests) | `bindings/helical_coil_cap_tests.rs` |
| Sphere-boss seam fillet through the batch ops (esaueng/remus#1561): closed display mesh at the consumer's settings, contacts on their supports within their recorded tolerance (contract test) | `bindings/sphere_boss_seam_fillet_tests.rs` |
| Direct and both batch WASM qualification of the B38 sphere–torus orientation repair (P-Class 8.1 seeds 5201 and 1146): exact, validator-clean, slice-integral volumes, watertight meshes, refusal rollback | `bindings/sphere_torus_orientation_tests.rs` |
| Entity-evolution and journal bindings | `bindings/evolution.rs` |
| Typed persistent GCS sketch bindings | `bindings/gcs_sketch.rs` |
| Linked planar GCS attachment and fixed external lines | `bindings/linked_sketch.rs` |
| Faces with inner (hole) wires — validation & construction | `holed_face.rs` |
| `log` → JS `console` bridge | `logging.rs` |
| Panic capture for post-mortem diagnosis | `panics.rs` |
| Deterministic reproduction bundles | `repro.rs` |
| Entity lifecycle (delete / restore) | `bindings/lifecycle.rs` |
| Persistent-naming bindings | `bindings/naming.rs` |
| Holed-face integration tests | `bindings/holed_face_tests.rs` |
| Qualified-operation integration tests | `bindings/qualify_ops_tests.rs` |
| Direct/batch B6 primitive qualification | `bindings/qualify_primitives_tests.rs` |
| Direct/batch B6 convex-hull/Minkowski qualification | `bindings/qualify_convex_hull_tests.rs` |
| Direct/batch B6 plane-section/split qualification | `bindings/qualify_section_split_tests.rs` |
| Per-edge convexity + face material-sense qualification (B16) | `bindings/qualify_edge_relations_tests.rs` |
| Ordered wire traversal contract (B16) | `bindings/wire_traversal_contract_tests.rs` |
| Growing-holder recipe `volume` contract (B54: closed-form −5440 width step through `executeBatch`) | `bindings/growing_holder_volume_tests.rs` |
| Store-local boolean switch (`setStoreLocalCuts` / `storeLocalCutsEnabled`, direct and batch) contract tests | `bindings/store_local_switch_tests.rs` |
| Sweep bindings refuse atomically (`multiSectionSweep` direct and batch; kernel review WASM-6) | `bindings/sweep_transaction_tests.rs` |
| Batch `sewFaces` without `tolerance` sews at the batch default `TOL`, not 1e-6 (contract test) | `bindings/sew_faces_default_tests.rs` |
| Direct and both batch contracts for the bounded P-Class 7.1 hollow taper | `bindings/tapered_hollow_sweep_tests.rs` |
| WASM contract tests for the mixed-notch L-bracket boundary | `bindings/mixed_notch_tests.rs` |

### L4: wasm-io (`crates/wasm-io/src/`)
| Task | File(s) |
|------|---------|
| `RemusIo` translator class: every format import/export over arena documents | `lib.rs` |
| Error type, `validate_positive` | `error.rs` |

The shipped browser kernel (`crates/wasm/pkg`) is built without the `io`
feature; STEP/IGES/mesh formats ship as the separate `remus-wasm-io` package
(`crates/wasm-io/pkg`), loaded only around import and export. Bodies cross
between the two as exact arena documents: `kernel.serializeSolids(ids)` →
`io.exportStep(bytes)`, and `io.importStep(file)` → `kernel.deserializeSolids(bytes)`.
`cargo xtask wasm-build` builds, validates, and version-locks both packages.
New capabilities pick a module first: see [WASM module tiers](#wasm-module-tiers).

### L5: facade (`crates/remus/src/`)
| Task | File(s) |
|------|---------|
| Owned native modeling session and curated front-door API | `model.rs` |

## Ripple-Effect Checklists

**These enums appear in `match` arms across many files. Adding a variant requires updating
every match site or the code won't compile (unless a `_ =>` wildcard swallows it silently).**

**Delegate methods reduce ripple scope:** `FaceSurface` has delegate methods (`evaluate`,
`normal`, `project_point`, `estimate_radius`, `type_tag`, `is_planar`, `is_analytic`,
`as_analytic`) and `EdgeCurve` has delegates (`evaluate_with_endpoints`,
`tangent_with_endpoints`, `domain_with_endpoints`, `type_tag`) — see `math/src/traits.rs`.
Call sites using these delegates need no update when adding new variants (only the delegate
impl needs the new arm). The files below still use direct match arms.

### Adding an `EdgeCurve` variant

`EdgeCurve` is defined in `topology/src/edge.rs`. Current variants: `Line`,
`NurbsCurve`, `Circle`, `Ellipse`, `Hyperbola`, `Parabola`.

`EdgeCurve::` is matched in ~150 files. The compiler flags the exhaustive
direct-match sites, so you do NOT need a hand-maintained list for those — but
21 production matches carry a `_ =>` arm and will absorb a new variant
silently. See [Wildcard match arms](#wildcard-match-arms); auditing those is
part of the job, not an optional extra. Get the full set with:

```bash
rg -l 'EdgeCurve::' crates/*/src/
```

Many call sites go through the `EdgeCurve` delegate methods
(`evaluate_with_endpoints`, `tangent_with_endpoints`, `domain_with_endpoints`,
`type_tag` — see `math/src/traits.rs`) and need no update; only the delegate
impl needs the new arm. The high-traffic direct-match sites worth checking
first: `operations/src/tessellate/`, `transform.rs`, `copy.rs`,
`measure/edge_length.rs`, `section.rs`, `fill_face.rs`, `boolean/`,
`io/src/{step,iges}/{reader,writer}.rs`, and
`wasm/src/bindings/{query,batch,tessellate,nurbs}.rs`.

### Adding a `FaceSurface` variant

`FaceSurface` is defined in `topology/src/face.rs`. Current variants: `Plane`, `Nurbs`, `Cylinder`, `Cone`, `Sphere`, `Torus`.

`FaceSurface::` is matched in ~150 files. As with `EdgeCurve`, the compiler
flags the exhaustive sites — but 72 production matches carry a `_ =>` arm and
absorb a new variant silently, so
[Wildcard match arms](#wildcard-match-arms) applies with more force here than
anywhere else in the codebase. Get the full set with:

```bash
rg -l 'FaceSurface::' crates/*/src/
```

Many call sites go through the `FaceSurface` delegate methods (`evaluate`,
`normal`, `project_point`, `estimate_radius`, `type_tag`, `is_planar`,
`is_analytic`, `as_analytic` — see `math/src/traits.rs`) and need no update.
The high-traffic direct-match sites worth checking first:
`operations/src/tessellate/`, `transform.rs`, `copy.rs`, `section.rs`,
`distance.rs`, `feature_recognition.rs`, `boolean/`, `offset_face.rs`,
`io/src/{step,iges}/writer.rs`, and
`wasm/src/bindings/{query,batch,tessellate,nurbs}.rs`.

If the new surface is analytic, also update:
- [ ] `math/src/analytic_intersection.rs` — `AnalyticSurface` enum (4 match sites)
- [ ] `math/src/surfaces.rs` — surface definition

## Common Pitfalls

### Borrow checker: "snapshot then allocate"
When copying topology entities, you cannot borrow the arena immutably (to read) and
mutably (to write) at the same time. Read all needed data into local variables first,
then allocate new entities:
```rust
// ✅ Correct: snapshot first
let pos = topo.vertex(vid).position;
let new_vid = topo.add_vertex(pos);

// ❌ Wrong: simultaneous borrow
let new_vid = topo.add_vertex(topo.vertex(vid).position);
```

### Closure return type annotations
When `OperationsError` has multiple `From` impls, closures need explicit return type:
```rust
// ✅ Correct
let f = |x| -> Result<_, OperationsError> { ... };

// ❌ Wrong: compiler can't infer which From impl
let f = |x| { ... };
```

### Lint allows in tests
```rust
#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    // ...
}
```

### Complex functions
Add `#[allow(clippy::too_many_lines)]` above complex CAD operations rather than
artificially splitting them.

### WASM binding blocks
Multiple `#[wasm_bindgen] impl RemusKernel` blocks are needed — one for public
JS-exposed methods, one for private helpers. This is a wasm-bindgen requirement.

### Wildcard match arms
**The compiler does not flag every match site.** Roughly 93 production `match`
blocks over `EdgeCurve` (21) and `FaceSurface` (72) carry a `_ =>` arm, spread
over 39 files. The densest are `operations/src/measure/volume.rs` (13),
`algo/src/pave_filler/phase_ff.rs` (8),
`operations/src/tessellate/nonplanar.rs` (7), and
`operations/src/resize_blend.rs` (6).

Most of those arms are a deliberate "anything else is not the special case I
handle" (`_ => None`, `_ => false`, `_ => return Ok(None)`), which is why they
exist. The cost is that a **new variant lands in them silently**: the shape
does not fail to compile, it quietly degrades — a face skipped, an exact path
declined for a mesh fallback, a surface treated as non-planar. So adding a
variant is a two-part job:

1. Fix what the compiler flags (the exhaustive sites).
2. Audit the wildcard sites by hand and decide, per site, whether the new
   variant belongs with the handled set. Enumerate them with:

```bash
rg -n --multiline '(EdgeCurve|FaceSurface)::[\s\S]{0,600}?^\s+_ =>' crates/*/src/
```

Step 2 is the one that gets skipped; treat a variant addition as incomplete
until it has been done.

### Walking faces in a solid
A solid has both an `outer_shell()` and zero-or-more `inner_shells()`
(cavity shells produced by `shell_op` and boolean cuts). A function
that visits only `outer_shell()` will silently miss cavity faces on
hollow solids. Default to `topology::explorer::solid_faces`, which
flattens outer + inner shells into a single `Vec<FaceId>`:

```rust
// ✅ Correct: covers cavity faces too
use remus_topology::explorer::solid_faces;
let face_ids = solid_faces(topo, solid_id)?;

// ❌ Wrong: silently skips inner-shell faces
let solid_data = topo.solid(solid_id)?;
let shell = topo.shell(solid_data.outer_shell())?;
let face_ids = shell.faces().to_vec();
```

Exceptions: operations that are fundamentally per-shell (orientation
fixes, sewing, "don't empty this shell" guards) should still iterate
shell-by-shell — call `solid_faces` only when the operation is
solid-scoped (counting, recognition, type-conversion). The
`heal::fix::small_face` pattern (top-level loop over shells +
per-shell helper with its own guard) is the template for those.

### Dev-dependency cycles
Never add `remus-operations` as a dev-dependency of `remus-topology` — this
creates a "two versions" error. Use the `test-utils` feature flag instead.

## Cookbook: Common Agent Tasks

### Recipe 1: Add a new primitive

Pattern: see `primitives.rs` (`make_box`, `make_cylinder`, etc.)

1. **Create the function** in `operations/src/primitives.rs`:
   ```rust
   pub fn make_thing(topo: &mut Topology, params...) -> Result<SolidId, OperationsError> {
       // Create vertices → edges → wires → faces → shell → solid
   }
   ```
2. **Add WASM binding** in `wasm/src/kernel.rs`:
   - Add method to `impl RemusKernel` with `#[wasm_bindgen]`
   - Use `validate_positive` / `validate_finite` from `error.rs`
3. **Add test** in the same file or a dedicated test module
4. **Add to batch dispatch** in `kernel.rs` `dispatch_op` if applicable

### Recipe 2: Add a new IO format

Pattern: see `obj/` module (simplest), `step/` (most complex)

1. **Create module** `io/src/format/mod.rs`, `reader.rs`, `writer.rs`
2. **Add module** to `io/src/lib.rs`
3. **Writer signature**: text formats return `String`, binary formats return `Vec<u8>`:
   - B-Rep, text: `pub fn write_step(topo: &Topology, solids: &[SolidId]) -> Result<String, IoError>`
   - Mesh, binary: `pub fn write_ply(..., deflection: f64) -> Result<Vec<u8>, IoError>`
4. **Reader signature**: the input comes first, `topo` second, and B-Rep readers return
   every solid in the file:
   - B-Rep: `pub fn read_step(input: &str, topo: &mut Topology) -> Result<Vec<SolidId>, IoError>`
   - Mesh formats pair a `read_ply(data) -> TriangleMesh` with a
     `read_ply_solid(..) -> SolidId` that also needs `&mut Topology`
5. **Add WASM bindings** `importFormat` / `exportFormat` on `RemusIo` in
   `crates/wasm-io/src/lib.rs` (arena-document in, arena-document out) and,
   for the single-module `io` build, in `bindings/io.rs`
6. **Batch dispatch** (`bindings/batch.rs`) only exists in the single-module
   `io` build; the shipped kernel has no format ops to dispatch

### Recipe 3: Add a new operation

Pattern: see `extrude.rs` (basic), `boolean/` (complex)

1. **Create file** `operations/src/op_name.rs`
2. **Define function**: `pub fn op_name(topo: &mut Topology, ...) -> Result<SolidId, OperationsError>`
3. **Add module** to `operations/src/lib.rs` with `pub mod op_name;`
4. **Add WASM binding** in the appropriate `bindings/` module, or in the
   lazy module its [tier](#wasm-module-tiers) names (native-only ops skip this)
5. **Add to batch dispatch** in `bindings/batch.rs` if applicable
6. **Add test** — create a known shape, apply operation, verify with `measure`

### Recipe 4: Add a new WASM binding

Pick the appropriate `bindings/` module (or create a new one). Each module adds
methods to `RemusKernel` via a separate `#[wasm_bindgen] impl` block.

```rust
// In bindings/my_domain.rs:
use wasm_bindgen::prelude::*;
use crate::kernel::RemusKernel;
use crate::error::{WasmError, validate_positive};
use crate::handles::solid_id_to_u32;

#[wasm_bindgen]
impl RemusKernel {
    #[wasm_bindgen(js_name = "myOperation")]
    pub fn my_operation(&mut self, param: f64) -> Result<u32, JsError> {
        validate_positive(param, "param")?;
        let result = some_operation(&mut self.topo, param)?;
        Ok(solid_id_to_u32(result))
    }
}
```

Key points:
- `RemusKernel` is the `core` tier: confirm the capability's
  [tier](#wasm-module-tiers) is `core` before adding it here
- Use `js_name` for camelCase JS naming
- Validate inputs with helpers from `error.rs`
- Return entity IDs as `u32` (auto-converted to JS number)
- Errors use `?` operator (WasmError → JsError via blanket `From` impl)
- Add a `batch_*` companion fn if the op should be in `executeBatch`
- Add contract tests using `execute_batch()` (not direct method calls,
  since `JsError` can't be constructed on non-wasm targets)

## Commands

```bash
cargo build --workspace                    # Build all
cargo test --workspace                     # Test all
cargo clippy --all-targets -- -D warnings  # Lint
cargo fmt --all                            # Format
cargo build -p remus-wasm --target wasm32-unknown-unknown  # WASM
cargo xtask wasm-build                     # kernel + translator packages, validated
./scripts/check-boundaries.sh              # Verify layer deps
./scripts/check-det-hash.sh                # No new std HashMap/HashSet in geometry crates
./scripts/check-doc-paths.sh               # Verify doc file paths still resolve
```

The two npm packages receive one shared version during `cargo xtask wasm-build`.
Changed distributable bytes or exports advance the patch version from the committed
packages; identical output keeps its version. The matching WASM Cargo versions
provide the minimum release version and can explicitly advance major/minor releases.
The build runs `scripts/test-version-wasm-packages.mjs` before assigning the version.
The WASM version guard checks PR and merge-group package changes against their base.
The package publisher also checks the staged artifacts before any push; changed
packages cannot reuse or lower the shared version.

### Profiling

The `[profile.profiling]` block in `Cargo.toml` inherits from release with debug symbols and no LTO for fast builds with full symbol resolution.

```bash
cargo flamegraph --profile profiling \     # Flamegraph a single benchmark
  --bench cad_operations \
  -p remus-operations \
  -o /tmp/flamegraph.svg \
  -- --bench "bench_name_filter"
```

## Key Patterns

### Error handling
- `thiserror` for typed error enums per crate (`MathError`, `TopologyError`, `OperationsError`, `IoError`)
- Never `unwrap()`, `expect()`, or `panic!()` — return `Result`
- Use `#[error(transparent)]` for error propagation across crate boundaries

### Topology
- Arena-based allocation with typed `Id<T>` handles
- All entities owned by the arena, referenced by ID
- Edge-to-face / face-neighbor adjacency via the `adjacency` module

### Tolerance
- `Tolerance` struct with `linear` (1e-7) and `angular` (1e-12) defaults
- Compare floats via `tolerance.approx_eq(a, b)`, never `==`

### Types
- `Point3` (position) vs `Vec3` (direction) — separate newtypes
- `Mat4` for affine transforms
- NURBS curves/surfaces as the native geometry representation

## Lints

Workspace-level strict lints:
- `unsafe_code = "deny"` — no unsafe
- `unwrap_used = "deny"` — no unwrap
- `panic = "deny"` — no panic
- `clippy::pedantic`, `clippy::nursery` = warn
- `missing_docs` = warn
- `clippy::disallowed_methods` forbids `sort` / `sort_by` / `sort_by_key` and
  `sort_unstable` / `sort_unstable_by` / `sort_unstable_by_key` in the kernel
  crates (math through operations) and the wasm bindings (each has its own
  `clippy.toml`): use
  `remus_math::compact_sort::CompactSort`. The stable `compact_sort_by*` keep
  `sort_by`'s order, ties included. The unstable `compact_sort_unstable*`
  match `sort_unstable*` only where no two distinguishable elements compare
  equal (ties only between identical values, or unique keys), so convert an
  unstable sort only after checking that. A measured hot sort keeps the std
  sort behind a statement-level `#[allow(clippy::disallowed_methods)]` with a
  `// hot: <evidence>` line; an unstable sort whose tie order can reach its
  result keeps it with a `// ties: <reason>` line. Test code is exempt.

## Testing

- Unit tests in each module
- `proptest` for property-based testing
- Golden file tests in `tests/golden/`
- Integration tests in `tests/integration/`
- `criterion` benchmarks in `benches/`

## Git Conventions

- Conventional commits enforced by commitlint
- Pre-commit: fmt + clippy + taplo + machete in parallel (no tests)
- Pre-push: delegates validation to CI (full test, deny, boundaries, lineage)
- Branch: `main` is the primary Apache-2.0 branch
- Never merge upstream v3 or later. Future upstream behavior must arrive under
  an explicit Apache-2.0 grant or be independently implemented.
