# Industrial parity and leadership overlay

A **non-owning** competitive overlay on the existing programs. It defines
the scope inside which Remus claims parity with the incumbent open-source
reference kernel, maps every parity target to the P-Class, Open Kernel, or
Bridge row that owns the work, names the gaps nothing owned, and states the
measurable exit gates for the three post-v1.0 horizons (H5–H7 in
[roadmap.md](roadmap.md)).

- **Drafted:** 2026-09-04, baseline `main` @ `0769b194`.
- **Reference frame:** the current major release line of the incumbent
  open-source reference kernel, verified against its official release feed
  and documentation at drafting time. The exact version, release date, and
  build are pinned only in the head-to-head harness manifest (O1.2a), never
  in this document, per the repository's no-naming policy.
- **What this document is not:** a third implementation backlog. It adds no
  ledger. The authoritative state of every row stays where it already lives:
  [p-class-status.md](p-class-status.md), [open-kernel-status.md](open-kernel-status.md),
  the §B table in [roadmap.md](roadmap.md), the
  [capability matrix](capability-matrix.md), and the
  [stability matrix](../production-readiness/stability-matrix.md). Where a
  parity target had no owner, this overlay added a row **to the program that
  can coherently own it** (§5) and points at it; it never restates that
  row's state here.
- **Maintenance rule:** a crosswalk row changes only when its owner row's
  ledger state changes or a scorecard run moves its competitive state. Any
  PR that flips an owner row updates the pointer here in the same PR (the
  living-document rule in `.claude/skills/roadmap/SKILL.md`).

## §0 Why an overlay, and what parity means here

The P-Class program defines "professional-grade" by four properties
(imports work, general booleans, real edits, honest everywhere); the Open
Kernel program defines "best open kernel" by seven public claims (S1–S7).
Neither says, cell by cell, *where Remus stands against the reference kernel
a skeptic will actually compare it with*. That comparison has three rules
that no existing document states:

1. **A typed refusal is a contract success, not capability parity.** When
   the reference kernel correctly completes an in-scope operation that Remus
   refuses, the cell is `Gap-measured`, however honest the refusal is.
2. **A valid B-Rep is not parity either.** Validation is a postcondition, not
   an oracle. A cell reaches `Parity-proven` only when an independent oracle
   (closed form, inclusion–exclusion, cross-kernel invariant agreement, or
   round-trip identity) confirms the result is *right*, not merely closed.
3. **Parity is not module cloning.** The reference platform ships a viewer,
   a document framework, a Tcl test harness, and proprietary-format
   translators. Parity is scored only inside the scope contract of §1.

The overlay therefore keeps two independent axes (§2) and never collapses
them into a single score.

## §1 Scope contract

Every capability area carries exactly one disposition. The disposition
decides whether a `Gap-measured` cell is a roadmap obligation (in-scope), a
consumer-service obligation (integration-adjacent), a deferred decision
(later/horizon), or a documented non-goal.

| Disposition | Meaning | Who decides a change |
|---|---|---|
| **In-scope kernel parity** | Remus must reach measured parity (or a documented lead) on the reference kernel's public, documented behavior. A gap here is an owned roadmap row. | Program owner rows; promotion authority unchanged. |
| **Integration-adjacent service** | Not a modeling-kernel capability, but a lower-level service the OpenZCAD consumer or a third-party consumer needs *from the kernel* rather than from an application layer. Scored, but against consumer need, not the reference module. | Consumer evidence (OpenZCAD workflows, second consumer). |
| **Later/horizon capability** | A real kernel capability Remus does not attempt before H7 or an explicit owner decision. Listed so it is never silently unowned. | Owner decision recorded in the ledger that would own it. |
| **Intentionally out of scope** | A reference-platform module Remus will not replicate. Its *lower-level kernel services* may still be in scope and are listed separately. | Owner; reopening requires consumer or corpus evidence. |

### In-scope kernel parity

Geometric representation and evaluation; B-Rep topology and the
solid/sheet/wire/compound body taxonomy (RFC 0005); intersections and
classification; tolerant modeling (RFC 0004); booleans, General Fuse,
split, imprint, and section; blends and local modifications; offsets,
thickening, shelling, hollowing, and draft; sweeps, pipes, lofts, and
surfacing; direct modeling, defeaturing, and feature recognition;
validation, sewing, healing, and shape upgrading; tessellation and geometry
interrogation; neutral CAD exchange (STEP, mesh formats); assembly/product
structure and attributes; topology evolution and persistent references
(RFC 0003); safe, versioned native and WASM APIs; large-model performance
and deterministic concurrency; assurance on untrusted input.

### Integration-adjacent service

Selection-ready tessellation with stable per-face and per-edge identity
(the consumer's picking substrate); silhouette and hidden-line data;
presentation metadata (names, colors, layers, materials as kernel
attributes); document deltas and incremental recomputation (journal-driven
cache invalidation and versioned snapshots); assembly occurrence identity;
undo/redo *information* (checkpoint, rollback, transactional history — the
undo *stack* stays the application's). Sketch-to-B-Rep integration sits here
as a Remus leadership item rather than a parity item: the reference kernel
has no constraint solver; Remus ships one.

### Later/horizon capability

Non-manifold and cellular topology with literal shared faces (RFC 0005
names it a later RFC); mesh+B-Rep hybrid modeling (O7, design-only until
M4 is settled); external references and partial loading of large
assemblies; collaboration-grade versioned model deltas beyond the journal;
semantic PMI write (O5.3c); a stable C ABI (decision, O4.5); WASM threads
(evidence-gated, 8.7); lattice representation.

### Intentionally out of scope (with the kernel services that remain in scope)

| Reference-platform module | Out of scope | Kernel service Remus still owes consumers |
|---|---|---|
| Desktop viewer framework | Yes | Tessellation with face/edge ids, normals, UVs; silhouettes; `remus-render` stays an offscreen verification tool (B14). |
| GUI widgets and manipulators | Yes | Stable handles and persistent references so an application can build manipulators. |
| Tcl-style test application | Yes | Reproduction bundles, the gauntlet, and the head-to-head harness cover the test-driver role. |
| CAM toolpath generation | Yes | Exact section curves, offsets of wires and faces, watertight export. |
| CAE solvers | Yes | Mass properties, watertight/manifold meshes, mesh validation. |
| Application document framework | Yes (OpenZCAD owns it) | Journal, checkpoints, transactional rollback, arena serialization with versioned migration, attribute propagation. |
| Proprietary-format translators (SAT, Parasolid, JT, DXF, IFC) | Yes | None; STEP is the exchange path. IGES stays decided (stabilization C3, Option 2) — reassessed in §4.11 and **not** reopened: no consumer or corpus evidence has arrived since the decision. |

## §2 Dual-axis status model

Each crosswalk row carries one value on each axis. The contract axis reuses
the capability-matrix vocabulary plus one annotation; the competitive axis is
new and is *never* derived from the contract axis alone.

**A. Remus contract state** (from the [capability matrix](capability-matrix.md)):
`Qualified` · `Partial with declared bounds` · `Unqualified` ·
`Unsupported-typed` · `Unsupported-untyped` · `Approximate with disclosed
provenance` (a Qualified or Partial cell whose success is delivered under an
explicit approximation policy).

**B. Competitive outcome state:**

| State | Meaning | Evidence required |
|---|---|---|
| `Lead-proven` | Remus is measurably better on an equivalent-quality basis, reproducibly, with losses published. | O1.2 harness row with pinned baseline; a leadership claim in §7. |
| `Parity-proven` | Independent oracles confirm equal correct outcome; runtime and resources within the locked band. | O1.2 harness row, or gauntlet stage at parity band. |
| `Gap-measured` | The reference kernel completes the cell correctly and Remus refuses, degrades, or errs; the gap is measured and owned. | Harness row or pinned repro plus an owner ID. |
| `Behind-unmeasured` | Known missing capability, not yet in a harness. | Code/ledger evidence of absence plus an owner ID. |
| `Unknown` | Nothing measured and no code-level evidence either way. **Forbidden for in-scope rows at H5.** | — |
| `Intentionally out of scope` | §1 non-goal. | §1 table entry. |

Rules: a row may be `Qualified` and `Gap-measured` at the same time (a
qualified typed refusal on a cell the reference kernel completes). A row may
be `Unqualified` and `Parity-proven` only transiently — the harness result
must feed a qualification test within one triage cycle or the row reverts
to `Gap-measured`. `Lead-proven` requires the leadership discipline of §7;
no row is `Lead-proven` on a single benchmark run.

## §3 Competitive scorecard (expands O1.2, adds nothing parallel)

The head-to-head harness (`tools/vs-bench`, O1.2a–c) is the only sanctioned
comparison instrument. This section widens its metric schema and adds the
baseline-pinning milestone; the rows that do that are O1.2d–f in
[open-kernel-implementation.md](open-kernel-implementation.md). Nothing here
duplicates the gauntlet (`tools/gauntlet`), which stays the corpus
instrument; the harness consumes gauntlet manifests rather than defining its
own corpus.

### 3.1 Raw per-domain metrics (never one composite score)

Every scenario reports every applicable column below, per kernel, per run.
Outcome columns are mutually exclusive per scenario; a scenario lands in
exactly one.

| Group | Columns |
|---|---|
| Outcome | `correct_success` · `exact_success` · `disclosed_approximate_success` · `verified_repair_success` (healing or tolerance growth disclosed and verified) · `typed_refusal` · `untyped_error` · `silent_wrong` (success reported, oracle disagrees) · `invalid_success` (success reported, validator rejects) · `crash` · `hang_or_budget_overrun` · `nondeterminism` (repeat disagreement) |
| History | `evolution_completeness` (fraction of result entities with a non-unresolved event) · `persistent_ref_survival` (refs bound before the edit that resolve `Bound`/`BoundMany` after) |
| Interchange | `import_validity` · `post_import_operation_success` · `round_trip_geometry_fidelity` (volume/area/centroid/bounds deviation) · `assembly_metadata_fidelity` (tree, transforms, names, colors, materials) |
| Geometry quality | `tessellation_watertight` · `volume_error` · `area_error` · `centroid_error` · `inertia_error` (each relative to the oracle) |
| Resources | `runtime_median` · `runtime_p95` · `peak_memory` · `entity_growth` (arena entities after / before) · `cancellation_latency` |
| Browser | `wasm_cold_init` · `module_size_raw` · `module_size_gzip` · `module_size_brotli` · `native_wasm_agreement` (invariant equality across builds) |
| Concurrency | `thread_scaling_efficiency` (speedup / threads, deterministic output required) |

### 3.2 Equivalence before speed

A runtime comparison is admissible only when both kernels produced
`correct_success`, `exact_success`, or `disclosed_approximate_success`
**at an equivalent output-quality requirement** declared in the scenario
(same deflection, same tolerance model, exact-vs-exact or approximate within
the same error budget). A coarse approximate result is never timed against
an exact one. A `typed_refusal` on either side removes the scenario from the
speed table and keeps it in the outcome table.

### 3.3 Baseline first, bands second

No competitor pass rate and no performance threshold is written into any
roadmap document before the baseline milestone (O1.2f) has pinned:

- reference kernel version, build flags, and binding layer;
- hardware and software environment (CPU, memory, OS, toolchain, browser
  and Node versions for WASM rows);
- corpus manifests (gauntlet manifests by content hash) and generated
  scenario seeds;
- operation contexts and tolerances per scenario;
- output-quality requirements per scenario (§3.2);
- repetition count and statistical method (median and p95 over N runs,
  with N and the outlier rule stated);
- baseline results, committed to the results branch with the harness SHA;
- refresh policy (re-baseline on reference-kernel minor release, on hardware
  change, or on a harness protocol change; never silently).

Only after that milestone do the H5–H7 gates in §6 lock numeric parity
bands, by editing the band placeholders in §6 in the same PR that lands
O1.2f.

### 3.4 Absolute gates (need no baseline)

These hold regardless of what the reference kernel scores. They are gates
for every horizon from H5 onward and are already the doctrine of
[operation-contract.md](operation-contract.md); the scorecard makes them
measurable per scenario.

1. Zero `silent_wrong` in the gated corpus.
2. Zero `crash` and zero unbounded `hang_or_budget_overrun`.
3. Zero undisclosed approximation (every `Approximate` result carries method
   and error bound).
4. Zero undisclosed healing or tolerance inflation (every repair is a typed,
   counted disclosure — the B1 contract).
5. Every non-success carries a stable typed classification
   ([failure-taxonomy.md](failure-taxonomy.md)); `untyped_error` is a gate
   failure, not a statistic.
6. Deterministic repeated output for a fixed build and operation context.
7. Native/WASM invariant agreement for every user-visible operation.
8. Every topology-producing operation reports complete evolution or an
   explicit typed unresolved record.
9. Every confirmed defect becomes a minimized permanent reproduction
   (bundle, fixture, or fuzz seed).
10. No published claim without a reproducible harness and manifest (R9).

## §4 Workflow scenarios (complete workflows, not isolated functions)

Each scenario is one harness job (O1.2e) run identically through the native
facade and the WASM batch path, with every stage reported separately so a
failure is attributed to a stage, not a workflow. `Owner` is the row whose
completion makes the stage passable; the scenario itself owns nothing.

| ID | Workflow | Stages (each separately scored) | Stage owners today |
|---|---|---|---|
| W1 | Sketch-constrained bracket | constrained sketch solve → profile face → extrude → cut → fillet chain → shell → measure → STEP export → re-import property check | GCS (§5.13 rows) · M4.7/4.2 profile · sweeps ledger · M2 · M5 (chain via `g1_chain`) · 5.7 · K-S2 · O1.4a |
| W2 | Analytic mechanical part | revolve → cross-drill (cylinder cut) → circular pattern → chamfer rims → planar section → tessellate at three deflections | revolve row · K-S1 cross-drill (done) · pattern (done, overlap refuses) · B3 closed-rim chamfers · section row · tessellation row |
| W3 | Dirty supplier STEP | import → diagnose (validation report + tolerance stats) → tolerate or verified-repair with disclosure → boolean → fillet → direct face edit → re-export → re-import agreement | O1.1 · B1 (done) · M3.5 · M3.4 · M5.2 · M6.1/6.2 · O1.4a |
| W4 | Freeform part | imported or generated NURBS bodies → SSI → trim (sheet by solid) → loft or sweep → constrained fill → sew → validate | 2.5 · 4.4 (done, planar) · 7.1/7.2 · 7.3 · sew row · validation row |
| W5 | Mixed-body workflow | solid + sheet + wire → split by sheet → imprint → section → multiple result regions in a Compound with per-region lineage | 4.3 · 4.5 · section row · 4.6 (done) · 4.8 (N-ary) |
| W6 | Assembly workflow | repeated instances → names/colors/materials → interference check → partial load or selection → AP242 round trip | O5.1 · O5.2 · 7.5 clash · O5.5 (decision) · O5.3a |
| W7 | Persistent-edit workflow | long operation history → parameter changes → face splits/merges → persistent-reference rebinding → deterministic rebuild | RFC 0003 (done) · B18 evolution audit · 6.5 · replay determinism gate |
| W8 | Large-model browser workflow | import → progressive/incremental tessellation → face-id selection → repeated edits with bounded memory → responsive cancellation | O1.1 · O3.4 · GroupedMesh (exists) · 8.6 compaction · 2.8 cancellation |
| W9 | Adversarial workflow | malformed or hostile file → bounded parser → typed refusal → session state byte-identical | `io::limits` + fuzz targets (done) · O1.1 taxonomy · arena/checkpoint invariants (done) |

A workflow scenario is `Parity-proven` only when every stage is, and its
`silent_wrong` column is zero for both kernels or the reference kernel's
silent-wrong stage is published as a Remus lead with the oracle that caught
it.

## §5 Competitive crosswalk

The crosswalk is organized by the sixteen outcome-level domains the audit
covered. Each domain has two tables: **A** (identity and status) and **B**
(requirements and gate). Column legend:

- *Disposition* — §1 value. *Contract* / *Competitive* — §2 axes.
- *Evidence* — the exact current evidence (test, fixture, PR, or ledger row)
  or the measured limitation.
- *Owner* — an existing P-Class (`M`), Open Kernel (`O`), Bridge (`B`), or
  stabilization row; `→ new` marks a row added by this overlay (§5.17).
- *Gap / dependency* — the missing primitive or qualification gap, then the
  dependency IDs.
- *Footprint* — likely source/test footprint.
- *Surfaces* — required native (`N`), WASM direct (`W`), WASM batch (`Wb`),
  Python (`Py`, once O4.3 lands), evolution (`Ev`: complete events or typed
  unresolved) requirements. `—` means not applicable to the row.
- *Policy & oracle* — exact/approximate/repair policy, then the independent
  oracle.
- *Matrix / boundary / perf* — the generated matrix or corpus, the negative
  boundary test, and the performance or resource metric.
- *Exit gate* — what establishes completion. *Lead* — the Remus leadership
  opportunity, if any.
- A **B** row may list several IDs (`IP-4.1/4.2/4.4/4.5`) when they share
  one footprint, owner, and gate; a row whose **A** state is already met
  carries `—` and the word `met`. Every **A** row has exactly one **B**
  entry that names it.

### 5.1 Geometry foundation

**A — identity and status**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-1.1 | Points, frames, transforms; line, circle, ellipse, parabola, hyperbola, Bezier, NURBS evaluation and derivatives | Full documented set | In-scope | Qualified (evaluation, degree ≥ 9 fuzzed); Partial (conics through booleans) | Parity-proven (evaluation); Gap-measured (hyperbola/parabola refused at the GFA door) | `math/src/curves.rs`, `nurbs/curve.rs`; NUM-001; `reject_unsupported_curves` in `algo/src/gfa.rs` | O2.2 (conic edges), B10 (conic cells) |
| IP-1.2 | Revolution, linear-extrusion, and offset surfaces as native carriers | Native surface-of-revolution/extrusion/offset surfaces | In-scope | Partial: swept carriers exist in math (O2.1b); topology still lowers STEP revolution/extrusion to NURBS; offset surface absent | Gap-measured (permanent exactness loss at import) | `math/src/surfaces/swept.rs`; RFC 0006; reader lowering in `io/src/step/reader.rs` | O2.1c–e; offset surface rides 7.4 |
| IP-1.3 | Explicit trims, p-curves, reparameterization | Documented | In-scope | Qualified (2.0a–g) | Parity-proven | p-class-status 2.0 rows; authority ratchet at zero | 2.0 (done); B8 |
| IP-1.4 | Periodic geometry, seams, poles, apexes, singularities | Handled in topology and meshing | In-scope | Partial (cylinder/torus/sphere seams qualified; pole and seam-parameter cells Unqualified) | Gap-measured | capability matrix "Intersections" gaps; `split_closed_torus_into_bands` | 2.4c, O2.3 |
| IP-1.5 | Interpolation, approximation, fitting, analytic recognition | Interpolation (C1/C2, periodic), approximation, batten/fairing curves, constraint-built lines and circles | In-scope | Partial: `interpolate`, `approximate_lspia`, `interpolate_surface`, `recognize_*` exist in math; no facade/WASM surface; no fairing; no degree reduction | Behind-unmeasured | `nurbs/fitting.rs`, `surface_fitting.rs`, `knot_ops.rs`, `geometry/convert/`; `bspline_restriction.rs` counts violations only | 7.6 → new |
| IP-1.6 | Continuity, curvature, regularity analysis | Continuity queries and local properties | In-scope | Partial: surface curvature and min-radius landed; curve continuity only via heal knot-multiplicity breaks; `edge_is_g1` only, no G2 | Behind-unmeasured | `check/src/analyze/curvature.rs`; `heal/src/upgrade/split_curve.rs`; `operations/src/query.rs` | 7.5 (partial), 7.6 → new |
| IP-1.7 | Point/curve/surface projection and global extrema | Documented extrema and projection | In-scope | Partial (Lipschitz optimizer; point→curve/surface; curve→curve) | Gap-measured (curve-curve / curve-surface cells Unqualified) | `geometry/src/extrema/*`; B10 open | B10 |
| IP-1.8 | Curve-curve, curve-surface, surface-surface intersection | 2D/3D CC, CS, SS with branch and tangency handling | In-scope | Qualified (seven analytic SS pairs), Partial (NURBS SSI with six budgets), Unqualified (CC/CS: NURBS×NURBS only, no analytic curve arms, no matrix) | Gap-measured | `math/tests/intersection_matrix.rs` (SS only); `bezier_clip.rs`; `intersection/curve_surface.rs` | 2.4c, 2.5, 2.7, B10 |
| IP-1.9 | Filtered predicates, interval bounds, root isolation, escalation policy for numerically ambiguous topology decisions | Tolerance-based; no exact escalation documented | In-scope | Partial: `robust`-backed exact `orient2d/3d`, SoS tie-breakers, and `filtered.rs` exist; **`filtered.rs` and the SoS predicates have zero consumers**; no interval arithmetic; no escalation policy | Behind-unmeasured (a lead candidate) | `math/src/predicates.rs`, `filtered.rs`; consumers in `cdt/`, `classify_2d.rs`, `phase_ff.rs` use unfiltered arms | O2.4 → new |

**B — requirements and gate**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-1.1 | Conic edges through GFA; dep 2.0 | `algo/src/gfa.rs`, `pave_filler/`, `analytic_intersection.rs` | N · W · Wb · Ev | Exact conic sections; cone-frustum section volume | curve type × cut angle × scale; refusal list empty or each entry pinned; census | O2.2 gate | — |
| IP-1.2 | `FaceSurface` variants + 92-site wildcard audit; dep 2.4 settled | per RFC 0006 | N · W (type tags) · Wb · Ev (none) | Exact carriers; NURBS-lowered twin identity | gauntlet "% faces analytic"; zero-lowering fixture | O2.1c–e gates | analytic preservation on turned parts |
| IP-1.3 | Reversed sub-spans; dep B7 | `topology/src/edge.rs` | N | Exact | authority validators in CI | B8 gate | — |
| IP-1.4 | Seam/pole cells; dep 2.4c | `nurbs/intersection/`, `face_splitter/` | N · Wb · Ev | Exact or disclosed marched | periodic seam × pole × scale; census | 2.4c gate + O2.3 pole handling | — |
| IP-1.5 | Surface the fitting API; add fairing, degree reduction, and constraint construction where OpenZCAD or W4 needs them; dep none | `math/src/nurbs/fitting.rs`, `crates/remus/src/model.rs`, `wasm/bindings/nurbs.rs` | N · W · Wb · Py | Exact interpolation (point passage within tol); fitted-twin oracle | point count × degree × periodic × scale; invalid-input refusal | 7.6 gate | — |
| IP-1.6 | Curve continuity (G0/G1/G2) and regularity queries; dep 7.5 | `check/src/analyze/` | N · W · Wb | Exact on analytic; sampled with bound on NURBS | five primitives exact; NURBS within fit tol; degenerate-edge refusal | 7.6 gate (analysis half) | — |
| IP-1.7 | Qualification only | `geometry/src/extrema/`, `math/tests/` | N · W | Exact classification | conic distance/classification cells × scale | B10 gate | — |
| IP-1.8 | Analytic curve×curve/curve×surface arms and matrix; general quartic seams; NURBS×NURBS booleans | as owners | N · W · Wb · Ev | Exact where closed form; marched with disclosed bound; on-surface residual + plane oracle | pair × relationship × scale; budgets observable | 2.4c, 2.5, B10 gates | budgets and cancellation exposed to JS |
| IP-1.9 | Escalation policy record; route pave/CDT/classifier predicates through the filtered arms; dep 2.6 | `math/src/predicates.rs`, `filtered.rs`, `cdt/`, `algo/src/classifier/`, `pave_filler/phase_*.rs` | N (policy in `OperationContext`) · Wb (disclosed) | Filtered exact with typed refusal when undecidable | perturbation sweep 1e-13..1e-6 with pass/fail flip census; determinism gate | O2.4 gate | certified topology decisions (LC2) |

### 5.2 Topology and body model

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-2.1 | Vertex/edge/coedge/loop/wire/face/shell/solid with per-use authority | Full hierarchy with orientation and location | In-scope | Qualified (RFC 0002) | Parity-proven; lead candidate (seam p-curve access fails closed) | 2.0e/g; `topology/src/{coedge,face_loop}.rs` | 2.0 (done) |
| IP-2.2 | Sheet, wire, compound bodies; cellular results as Compound | Shell/wire/compound shapes; CompSolid | In-scope | Partial (bounded cells qualified, PR #209–#222) | Gap-measured (curved sheet trims, multi-face sheets, intersecting-member fuse, multi-tool cut refuse typed) | capability matrix body-type axis | M4 (done, bounded); 4.8 → new |
| IP-2.3 | Multiple edge uses, seams, orientations; locations/instances | Documented; instancing via locations | In-scope (uses); Integration-adjacent (instances) | Qualified (uses); assemblies hold repeated `SolidId` + `Mat4`, no occurrence identity beyond a tree index | Parity-proven (uses); Gap-measured (no shared-definition instancing) | `operations/src/assembly.rs` (`ComponentId` is a tree index) | O5.4 → new |
| IP-2.4 | Cavities and nested shells | Solids with voids | In-scope | Qualified | Parity-proven | BOOL-001; `explorer::solid_faces` | done |
| IP-2.5 | Non-manifold and cellular topology with shared faces; mixed-dimensional models | CompSolid; non-manifold via General Fuse | Later/horizon | Unsupported-typed (`NonManifold` validation error; `CompSolid` is a data holder with no producer; `BodyClass::General` refused everywhere) | Deferred by RFC 0005 | `topology/src/validation.rs`, `compsolid.rs`, `topology.rs` | owner decision (§9.5) |
| IP-2.6 | Validation invariants | BRepCheck-class analyzer | In-scope | Qualified | Parity-proven | `check/src/validate/*`; position-quantized watertightness | done; B11 residue |
| IP-2.7 | Stable serialization and schema migration | BRep persistence; document persistence is application-level | In-scope | Qualified (arena v1–v5 read forever; frozen writer bytes; fuzzed round trip) | Lead candidate (byte-identical re-serialization) | `io/src/arena_io.rs`; `arena_roundtrip` fuzz | O4.6 → new (written policy + version matrix) |
| IP-2.8 | Memory compaction and stale-handle behavior | Reference-counted handles; no stale-handle guarantee | In-scope | Partial: no-reuse invariant qualified; compaction absent (e6b deferred); full deep snapshot per checkpoint | Lead (stale handles never alias); Gap-measured (unbounded growth in long sessions; OpenZCAD caps history at 32 checkpoints) | WASM-003; e6b design; OpenZCAD `exact-history-cache.ts` | 8.6 → new |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-2.1 | read-only wire facade deletion gate | `topology/src/face.rs` | N · W | Exact | authority ratchet | 2.0g met; facade removal is a later API decision | seam fail-closed |
| IP-2.2 | recursive lineage composition; dep 4.6 | `operations/src/boolean/`, `algo/src/builder/builder_solid.rs` | N · W · Wb · Ev | Exact; volume conservation across members | operand class × member count × overlap; `unsupported_*` both sides | 4.8 gate | per-member lineage |
| IP-2.3 | occurrence ids + shared definitions; dep O5.1 | `operations/src/assembly.rs`, arena roots, `wasm/bindings/assembly.rs` | N · W · Wb · Py · Ev (occurrence events) | Exact transforms | depth × sharing × edit-after-instance | O5.4 gate | naming-anchored occurrences |
| IP-2.4 | — | — | — | — | — | met | — |
| IP-2.5 | radial-edge machinery; RFC | later | — | — | — | decision | — |
| IP-2.6 | winding-aware duplicate-face check | `heal/src/fix/solid.rs` | N | — | — | B11 residue | — |
| IP-2.7 | compatibility policy + v1→v5 migration matrix + repro/evolution schema policy | `io/src/arena_io.rs`, `wasm/src/repro.rs`, `docs/` | N · W · Py | byte-identity oracle | every reader × writer version; corrupted-reference refusal | O4.6 gate | LC5 |
| IP-2.8 | copy-compaction (e6b Option A) + versioned checkpoint contract; dep none | `topology/src/arena.rs`, `wasm/bindings/{checkpoint,lifecycle}.rs` | N · W · Wb · Ev (remap) | exact remap; stale-handle fuzz | long-session growth bench; stale-handle refusal both sides; checkpoint count ≥ 256 | 8.6 gate | LC1 |

### 5.3 Booleans and General Fuse

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-3.1 | Fuse, cut, common on analytic solids | Documented, with history | In-scope | Qualified (bounded witnesses), Partial elsewhere | Parity-proven on the census witnesses; Gap-measured on general quadric pairs | capability matrix "Booleans"; `approx_census` | 2.4c/d |
| IP-3.2 | NURBS × NURBS booleans (imported bodies) | Documented | In-scope | Unqualified (boolean-level coverage zero; SSI exists) | Behind-unmeasured | 2.5 pending | 2.5 |
| IP-3.3 | Section, split, imprint | Section, Splitter, imprint via GF | In-scope | Qualified (planar imprint, sheet split, planar section) | Gap-measured (curved/same-domain imprint, curved sheet split, section cavity/degeneracy matrix) | 4.3, 4.5 rows; sectioning ledger | 4.3/4.5 residue (4.8), B6 section family |
| IP-3.4 | N-ary and multi-tool operations; solid/sheet/wire/compound mixes | GF over any shape mix; Cells Builder | In-scope | Partial (pairwise-disjoint Compound members only) | Gap-measured (intersecting-member fuse, multi-tool cut, wire operands, solid×sheet×wire mixes refuse typed) | 4.6 ledger; `boolean_compound_regions` | 4.8 → new |
| IP-3.5 | Transversal, tangent, coincident, near-coincident, seam-crossing, sliver, cavity, singular, degenerate | Fuzzy mode; documented C1 requirement; self-interference check | In-scope | Partial (tangent/sliver fall to disclosed approximation; seam-crossing and nested-shell Unqualified) | Gap-measured | ledger caveats; B2, B7, B15 | 2.6, 2.7, 3.4, B2, B7, B15 |
| IP-3.6 | Same-domain merging and gluing | Unify same domain; glue option | In-scope | Partial (`unify_same_domain` exists; issue #246 Euler failure; no evolution record) | Gap-measured | issue #246; OpenZCAD C1 (`unifyFaces` lineage) | B11 residue, B18 → new |
| IP-3.7 | Multiple disconnected result regions | GF result compound | In-scope | Qualified (4.6) | Parity-proven | PR #218/#219 | done |
| IP-3.8 | Self-intersecting input policy | Self-interference checker | In-scope | Partial (operand validity gate; no self-interference analysis) | Behind-unmeasured | `boolean/mod.rs` preflight | B17 → new (analysis half), 8.1 |
| IP-3.9 | Exact and marched/NURBS section quality | Exact where analytic; approximation otherwise | In-scope | Qualified (disclosed `BooleanQuality`) | Lead (disclosure; reference is silent) | `FallbackPolicy`, census | done (LC6) |
| IP-3.10 | Complete vertex, edge, face evolution | Generated/Modified/Deleted history | In-scope | Qualified for booleans | Lead (construction-derived, zero unresolved on fixtures) | `boolean_with_entity_evolution` | done (LC3) |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-3.1 | general quartic seams + mixed analytic classifier | `face_splitter/`, `classifier/` | N · W · Wb · Ev | Exact/disclosed; inclusion–exclusion, mesh co-refinement | pair × relationship × scale; census | 2.4 gate | — |
| IP-3.2 | periodic-NURBS face splitting; budgets | `phase_ff.rs`, `builder/`, `nurbs/intersection/` | N · W · Wb · Ev | disclosed NURBS seams; B-spline twin oracle | converted primitives × ops; two real imports | 2.5 gate | — |
| IP-3.3 | curved imprint/split; section matrix | `imprint.rs`, `split.rs`, `section.rs` | N · W · Wb · Ev | Exact | surface × cavity × degeneracy | 4.8 + B6 section family | pure Split events for refs |
| IP-3.4 | recursive lineage composition; wire operands | `boolean/`, `builder_solid.rs`, `gfa.rs` | N · W · Wb · Ev | Exact; per-member volume conservation | operand mix × count | 4.8 gate | — |
| IP-3.5 | scale-relative bands; tangent primitive; pave-block attachment | `algo/` | N · Wb | Exact or typed; never silent-wrong | 1e-5..1e6 sweep; tangent/sliver bundles | 2.6, 2.7, B7 gates; B5 exit benchmark | LC2 |
| IP-3.6 | Euler-safe unify + `unify_with_evolution` | `heal/src/upgrade/unify_same_domain.rs`, `journal_ops.rs` | N · W · Wb · Ev | Exact; volume/area identity | face count × loop count | B18 gate (unify item) + #246 fixture | — |
| IP-3.7 | — | — | — | — | — | met | — |
| IP-3.8 | operand self-interference report | `check/src/validate/`, `boolean/mod.rs` | N · W · Wb | typed `invalid_topology` | self-touching corpus | B17 gate | — |
| IP-3.9/10 | — | — | — | — | — | met | LC3, LC6 |

### 5.4 Blends and local modifications

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-4.1 | Constant-radius fillets on planar and analytic supports | Documented | In-scope | Qualified (planar, closed analytic rims); typed elsewhere | Parity-proven on the torture corpus's built rows; Gap-measured on open/non-coaxial curved supports | `fillet_torture.rs`; 5.2 | 5.2 residue (B4 trimmer) |
| IP-4.2 | Variable-radius fillets and laws | Documented | In-scope | Partial (band qualified; trimmed-solid assembly not) | Gap-measured | 5.1 | 5.1 residue (assembly) → 5.8 |
| IP-4.3 | Edge chains and G1 continuation | Documented | In-scope | Qualified (`g1_chain`) | Parity-proven | `blend/src/g1_chain.rs` | done |
| IP-4.4 | Vertex blends and setbacks | Documented | In-scope | Partial (planar same-radius common-ball corners; bounded setbacks) | Gap-measured (mixed-side, nonplanar, mixed-radius junctions) | 5.3, 5.4 | 5.3/5.4 residue → 5.8 |
| IP-4.5 | Face-face blends and hold lines | Documented | In-scope | Partial (one exact planar sheet cell) | Gap-measured | 5.6 | 5.6 residue |
| IP-4.6 | Chamfer distance-distance and distance-angle | Documented | In-scope | Qualified (planar); Experimental closed-rim | Gap-measured (closed-rim chamfers) | B3 open | B3 |
| IP-4.7 | Overflow, cliff, rollover, radius-limit behavior | Documented (may fail) | In-scope | Qualified stop-at-cliff; rollover Unqualified | Gap-measured (rollover) — the reference completes some rollovers | 5.5 merged; 6.1 primitive unused by blends | 5.8 → new |
| IP-4.8 | Closed rims, cavities, thin walls, imported geometry | Documented | In-scope | Partial | Gap-measured (imported blend witness limited to resize) | 5.2; Shapr3D witness | 5.2, M3.5, B1 program exit |
| IP-4.9 | G1/G2 continuity where claimed | G1 | In-scope | Qualified G1 within angular tol; no G2 | Parity (G1); G2 not claimed by either | 5.3 witnesses | — (non-goal until product pull) |
| IP-4.10 | Blend removal and resizing | Not a documented reference op | In-scope | Partial (`resize_blend` Experimental) | Lead candidate | C2 stabilization row | C2 (behind B3) |
| IP-4.11 | Result evolution and persistent-reference survival | History | In-scope | Qualified (versioned face payload) | Lead (LC3/LC4) | `docs/wasm-face-evolution.md` | done; B18 edges/vertices |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-4.1/4.2/4.4/4.5 | trimmer completion; curved open assembly; nonplanar corners | `blend/src/{trimmer,walker,corner}.rs` | N · W · Wb · Ev | Exact where analytic; disclosed NURBS bands; mesh-volume oracle | torture corpus + support pair × radius × scale; `RadiusTooLarge`/`cliff-encountered` both sides | B4, M5 residue gates; O1.3b publication | S2 |
| IP-4.6 | cone-frustum band | `blend/`, `operations/src/chamfer.rs` | N · W · Wb · Ev | Exact; closed-form volume | rim × distance × scale | B3 gate | — |
| IP-4.7 | rollover through 6.1 re-limitation | `blend/src/trimmer.rs`, `operations/src/replace_surface.rs` | N · W · Wb · Ev | Exact re-limitation or typed cliff | wall width × radius sweep across the cliff | 5.8 gate | — |
| IP-4.8 | tolerant contacts; imported corpus | `blend/`, `io` | N · Wb | disclosed tolerance growth | gauntlet blend stage | B1 program exit benchmark | — |
| IP-4.10 | pair matrix | `operations/src/resize_blend.rs` | N · W · Wb · Ev | Exact | pair × radius | C2 gate | resize without history |

### 5.5 Offset, shell, hollow, and draft

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-5.1 | Solid offset, inward and outward, cavity-bearing | Documented (join types, self-intersection removal) | In-scope | Qualified bounded (intersection joints; arc joints on convex polyhedra) | Gap-measured (NURBS-NURBS 3D intersection in the offset path; general face-face offset intersections) | ledger "Offset, thicken" row; B5 done | 2.5 (SSI in offset), 5.7b |
| IP-5.2 | Intersection, arc, and tangent joint modes; variable offset laws | Intersection/arc joins documented; no variable law | In-scope | Partial: `JointType::{Intersection, Arc}`; no tangent joint; scalar distance only | Parity on joins once 5.7b closes; variable laws are not a reference capability (non-goal) | `offset/src/data.rs` | 5.7b; variable laws: non-goal |
| IP-5.3 | Self-intersection detection and removal | Documented | In-scope | Partial (one exact collapsed L-prism cell) | Gap-measured | 5.7 merged bounded | 5.7b → new |
| IP-5.4 | Thick solids with face removal; open and closed shelling | Documented | In-scope | Qualified bounded (`thick_solid` with `exclude`; cavity + exclude refuses) | Gap-measured (curved excluded faces, cavity + exclude) | ledger Shell row | 5.7b, B6 |
| IP-5.5 | Curve, wire, face, sheet offsets | Wire/face offset documented (2D/3D) | In-scope | Partial: planar closed line/circle wires only; face offset exists; no 3D or open-wire offset; no NURBS/ellipse arms | Gap-measured | `operations/src/offset_wire.rs`; `math/polygon_offset.rs` | 7.7 → new |
| IP-5.6 | Planar and curved-face draft; neutral plane and pull direction | Documented | In-scope | Qualified planar; typed non-planar | Gap-measured (curved-face draft) | ledger Draft row | 6.4 |
| IP-5.7 | Local-limit diagnostics and result history | History | In-scope | Qualified (typed refusals; face map on default offsets) | Lead (typed refusal per reason; arc-joint/self-int variants fail closed rather than publish stale provenance) | B5 | B18 (arc-joint provenance) |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-5.1/5.3/5.4 | general BOP-based self-intersection removal; curved excluded faces; dep 2.5 | `offset/src/{self_int,inter3d,assemble}.rs`, `operations/src/shell_op.rs` | N · W · Wb · Ev | Exact where analytic; mesh-volume oracle | profile × wall × cavity × exclude × scale; fold both sides | 5.7b gate; B3 program exit benchmark | — |
| IP-5.2 | tangent joint (only if a consumer asks) | `offset/src/arc_joint.rs` | N · W · Wb | Exact | join × edge convexity | 5.7b gate (arc on concave edges) | — |
| IP-5.5 | 3D/open wires, NURBS/ellipse arms, tangent/arc/chamfer joins | `operations/src/offset_wire.rs`, `math/src/polygon_offset.rs` | N · W · Wb · Ev | Exact for line/circle; disclosed NURBS | curve type × open/closed × join × scale; self-overlap refusal | 7.7 gate | — |
| IP-5.6 | replace-surface with tapered conic; dep 6.1 | `operations/src/draft.rs` | N · W · Wb · Ev | Exact frustum/taper oracles | axes in `qualify_draft.rs` extended | 6.4 gate | — |
| IP-5.7 | richer generated/replaced provenance | `offset/`, `journal_ops.rs` | N · W · Wb · Ev | — | attribution total | B18 gate | LC3 |

### 5.6 Sweeps and surfacing

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-6.1 | Prism and revolution | Documented | In-scope | Stable, blocked on degenerate/cavity matrices | Parity-proven on witnesses; Gap-measured on matrices | ledger Sweeps rows | B6 sweeps family |
| IP-6.2 | Pipe sweeps; multiple guide rails; frame laws | Pipe/PipeShell with Frenet, corrected Frenet, fixed, guide-curve modes | In-scope | Partial: RMF (default), Fixed, ConstantNormal; one aux spine; no Frenet/corrected-Frenet; no twist law; scale law exists | Gap-measured | `operations/src/sweep.rs` (`SweepContactMode`, `sweep_guided`) | 7.1 |
| IP-6.3 | Variable-section and variable-law sweeps | Evolving sections | In-scope | Partial (`multi_section_sweep`, `scale_law`) | Gap-measured (twist law; law verification) | `sweep.rs` | 7.1 |
| IP-6.4 | Multi-section lofts; periodic lofts; continuity | ThruSections with ruled/smooth and closure | In-scope | Partial: `loft`, `loft_smooth`; no periodic loft; no tangency end conditions | Gap-measured | `operations/src/loft.rs` | 7.2 |
| IP-6.5 | Ruled surfaces | Documented | In-scope | Qualified via `loft` (analytic recovery) | Parity-proven | `loft.rs` `ruled_arc_surface` | done |
| IP-6.6 | Coons, Gordon, N-sided fills; G0/G1/G2 constrained filling | 3–4 curve constrained fill; plate surfaces; Gordon (new in the current reference release) | In-scope | Partial: 4-sided bilinear Coons only; no N-sided, no G1 constraints, no Gordon | Gap-measured | `operations/src/fill_face.rs` (4 curves, degree 1) | 7.3 (scope widened to Gordon) |
| IP-6.7 | Surface extension, trimming, untrimming, curve imprint | Documented | In-scope | Partial: untrim and imprint exist; **no surface extension** | Gap-measured | `untrim.rs`, `imprint.rs`; no `extend_surface` | 7.4 |
| IP-6.8 | Fairing and continuity verification | Batten/minimal-variation curves | In-scope | Absent | Behind-unmeasured | — | 7.6 |
| IP-6.9 | Degeneracy, cusp, self-intersection, nonconvergence policies | Documented failure statuses | In-scope | Partial (typed refusals on helix, degenerate profiles; budgets incomplete) | Gap-measured | ledger "topology and nonconvergence budgets incomplete" | B6 sweeps family, 2.8 |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-6.1/6.9 | evidence matrices; budgets | `operations/tests/qualify_sweeps.rs` (new) | N · W · Wb | Exact prism/revolution volumes | profile × path × cavity × degeneracy × scale; nonconvergence budget observable | B6 sweeps family | — |
| IP-6.2/6.3 | Frenet/corrected-Frenet laws, twist law, rail orientation | `sweep.rs`, `pipe.rs`, `math/src/frame.rs` | N · W · Wb · Ev | Exact helical closed form; section sampling | law × spine curvature × twist; cusp refusal | 7.1 gate | RMF default (no Frenet singularities) |
| IP-6.4 | tangency end conditions; periodic loft with a true seam | `loft.rs`, `nurbs/surface_fitting.rs` | N · W · Wb · Ev | G1 within angular tol; watertight seam | sections × closure × continuity | 7.2 gate | — |
| IP-6.6 | N-sided G1 fill; Gordon network | `fill_face.rs`, `nurbs/surface_fitting.rs` | N · W · Wb · Ev | normal deviation sampled on every boundary | sides 3..8 × G0/G1 × curved neighbors | 7.3 gate (Gordon: N×M network reproduces its curves) | — |
| IP-6.7 | analytic and NURBS extension; curve projection to face | `geometry/`, `nurbs/`, `operations/` | N · W · Wb | sampled identity on shared domain | curvature regimes | 7.4 gate | — |

### 5.7 Direct modeling and mechanical features

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-7.1 | Replace surface with neighbor re-limitation | Not a documented reference op (local ops framework only) | In-scope | Partial (plane→plane, coaxial bore); **not exposed in WASM** | Lead candidate (with refs) — but R8 violated until WASM lands | 6.1 merged (PR #238), reused by 6.2 bore moves (PR #257); no `replaceSurface` binding | 6.1 (WASM tranche), 6.5 |
| IP-7.2 | Move, rotate, offset, resize, delete face | Local operations; defeaturing | In-scope | Qualified bounded (`push_pull_face`, `move_faces`, `resize_cylindrical_face`, `defeature`) | Gap-measured (curved neighbors, holes carried) | ledger Push/pull row | 6.2, 6.3 |
| IP-7.3 | Local topology changes (boss across fillet, hole through move) | Documented | In-scope | Partial (holed planar boss cap through an incident constant-radius fillet, exact evolution, PR #257) | Gap-measured (rotation, lateral relocation, outward cylinders, ambiguous blend regions) | 6.2 ledger row | 6.2 residue, 6.3 |
| IP-7.4 | Imported-part edits | Documented | In-scope | Unqualified (imports are exact-tolerance only) | Behind-unmeasured | — | M3.5 + M6 (B4 exit benchmark) |
| IP-7.5 | Defeaturing of holes, pockets, bosses, ribs, slots, grooves, fillets, chamfers | Defeaturing documented | In-scope | Qualified declared set (planar wounds); typed elsewhere | Gap-measured (curved kept faces; fillet-band removal via 6.3) | `qualify_defeature.rs`; `resize_blend` removal | 6.3 |
| IP-7.6 | Recognition of common manufacturing features | Not a reference kernel capability | In-scope | Qualified declared set | Lead (no reference equivalent) | `qualify_feature_recognition.rs` | done (declared set) |
| IP-7.7 | Mechanical features (depressions, protrusions, ribs, grooves) | Documented feature constructors | Integration-adjacent | Composed by consumers from sketch → extrude → boolean (OpenZCAD `hole`, `extrude` cut paths) | Parity by composition; not a kernel op | OpenZCAD `exact-cylinder-ops.ts` | non-goal as monolithic ops; imprint + boolean + naming suffice |
| IP-7.8 | Attributes and persistent references through edits | History only | In-scope | Qualified for journaled ops; direct edits journal barriers | Lead candidate blocked on 6.5 | ledger Evolution row | 6.5 |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-7.1 | WASM direct + batch + evolution for `replace_surface`; dep #238 merge | `wasm/bindings/operations.rs`, `batch.rs`, `journal_ops.rs` | N · W · Wb · Ev | Exact; volume vs oracle | plane/bore cells × scale | 6.1 gate incl. R8 (WASM) | LC4 |
| IP-7.2/7.3 | generalized re-limitation; through-feature preservation | `push_pull.rs`, `offset/move_faces.rs` | N · W · Wb · Ev | Exact; refs resolve Bound | boss × plate × fillet × hole | 6.2 gate | S7 demo |
| IP-7.4 | tolerant import + edits | as owners | N · W · Wb · Ev | disclosed tolerance | corpus | B4 exit benchmark | — |
| IP-7.5 | curved delete-and-heal | `defeature.rs` | N · W · Wb · Ev | Exact restored volume | class × curved neighbors | 6.3 gate | — |
| IP-7.8 | journaled direct edits | `journal_ops.rs` | N · W · Wb · Ev | — | edit → resolve pinned | 6.5 gate | LC4 |

### 5.8 Tolerant modeling, validation, and healing

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-8.1 | Per-entity tolerance semantics, growth limits, provenance | Per-entity tolerances with max-tolerance modes; fuzzy booleans | In-scope | Partial (substrate + VV/VE/EE predicates; FF/EF, result growth, import pending) | Gap-measured (imperfect-body booleans unqualified) | 3.2, 3.3 merged | 3.4, 3.5, 3.6 |
| IP-8.2 | Edge/curve and p-curve consistency; SameParameter/SameRange | ShapeFix SameParameter | In-scope | Qualified for certified combinations; typed elsewhere | Gap-measured (general plane/conic proofs typed unavailable) | 2.0g | 2.0g residue (typed capability boundary) |
| IP-8.3 | Wire ordering, closure, gaps, small edges; face orientation, small faces; seams; shell orientation, sewing, free boundaries; duplicates; continuity splits; representation conversion | ShapeAnalysis/ShapeFix/ShapeUpgrade/ShapeCustom breadth | In-scope | Qualified at the verified boundary (B1); fixers exist per module; **no defect-class × severity × policy matrix**; no degree reduction | Gap-measured (breadth unmeasured; reference completes degree reduction) | `heal/src/{analysis,fix,upgrade,custom}`; `bspline_restriction.rs` counts only | B17 → new; 7.6 (degree reduction) |
| IP-8.4 | Dirty imported bodies participating directly in later operations | Heal-first workflow; fuzzy booleans | In-scope | Unqualified | Behind-unmeasured — the reference kernel's heal-first model is the target to beat | 3.5 pending; gauntlet 29/50 smoke | 3.5, B2 exit benchmark |
| IP-8.5 | Complete repair disclosure and evolution records | Status flags only | In-scope | Qualified (counted repair kinds, typed refusals) | Lead (LC6) | B1 (PR #243) | done |
| IP-8.6 | Sewing with tolerance; imported mesh sew + unify | Sewing documented | In-scope | Partial (`sew_shell` endpoint coincidence + interior sampling; issue #244 heal opens faceted solids) | Gap-measured | issue #244; OpenZCAD C5 | 3.5, B17 |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-8.1 | FF/EF acceptance; result growth; import assignment | `algo/pave_filler/`, `builder/`, `io/step/reader.rs`, `sew.rs` | N · W · Wb · Ev | disclosed growth ≤ context cap | gap 1×–100× tol corpus | 3.4–3.6 gates; B2 exit benchmark | tolerance provenance (LC6) |
| IP-8.3 | generated defect matrix per fixer; degree reduction | `heal/`, `operations/tests/qualify_heal.rs` (new) | N · W · Wb · Ev (repair events) | verified repair or typed refusal | defect class × severity × policy × scale | B17 gate | — |
| IP-8.4/8.6 | tolerant import; mesh-import sew/unify | `io/step/reader.rs`, `stl/import.rs`, `sew.rs` | N · W · Wb | zero heal invocations on the tolerant path | real dirty corpus; #244 fixture | 3.5 gate | — |

### 5.9 Tessellation and visualization-adjacent services

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-9.1 | Watertight, manifold, orientation-correct tessellation with deflection and angular controls | Incremental mesher with linear/angular deflection, relative mode, parallel | In-scope | Qualified bounded (cross-drilled, primitives, sheets); broader scale/perf pending | Parity-proven on witnesses; Gap-measured on breadth | ledger Tessellation rows | B6, 8.3 |
| IP-9.2 | Periodic seams, poles, cavities, open sheets, feature-edge preservation | Documented | In-scope | Qualified (band meshers, pole caps, sheet boundaries) | Parity-proven | `tessellate/nonplanar.rs`, `rim_chain.rs` | done |
| IP-9.3 | Stable per-face and per-edge identity; normals; UVs; analytic provenance | Triangulation per face; no id contract | Integration-adjacent | Qualified (`GroupedMeshResult`, `UvMeshResult`, `meshEdgesAll`) | Lead (face-offset contract is what OpenZCAD picks against) | `wasm/src/types.rs`; OpenZCAD `exact.ts:942-1010` | done; B16 (edge ids) |
| IP-9.4 | Deterministic indexed meshes | Not guaranteed | In-scope | Qualified (det_hash; determinism gates) | Lead (LC5) | 64-cut gate | done |
| IP-9.5 | Incremental and local remeshing; progressive LOD | Incremental mesh reuses unchanged faces | Integration-adjacent | Absent on the CPU path (per-call memos only; LOD only in `render`) — OpenZCAD re-tessellates whole solids per rebuild | Gap-measured | `tessellate/solid.rs`; OpenZCAD `display-tessellation.ts` | O3.4 → new |
| IP-9.6 | Hidden-line, silhouette, section-curve data | HLR documented | Integration-adjacent | Partial (orthographic HLR by exact classification; no silhouettes; section exists) | Gap-measured (silhouettes) | `operations/src/projection.rs` | 7.5, §6 inherited queue |
| IP-9.7 | Mesh validation and B-Rep/mesh property agreement | Not documented | In-scope | Qualified (`meshQuality`, mesh-volume oracles) | Lead | measure witnesses | done |
| IP-9.8 | Mesh + B-Rep hybrid modeling | Not in the reference kernel (commercial category) | Later/horizon | Unsupported-typed (mesh bodies refuse as boolean operands) | Deferred | O7 | O7 |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-9.1 | scale/perf matrices; parallel determinism | `tessellate/`, `benches/` | N · W · Wb | watertight within deflection | surface × deflection regime × body × scale; wall-clock and triangle budgets | B6, 8.3 gates | — |
| IP-9.5 | journal-keyed per-face mesh cache; edit → remesh only touched faces; dep O3.2 | `topology/src/spatial.rs` (O3.2), `tessellate/solid.rs`, `wasm/bindings/tessellate.rs` | N · W · Wb | bit-identical to full remesh | edit sequences × face counts; remesh-time bench | O3.4 gate | LC11 |
| IP-9.6 | silhouettes | `projection.rs`, `check/` | N · W · Wb | torus closed form | view × surface type | 7.5 gate | — |

### 5.10 Interrogation and analysis

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-10.1 | Mass, area, volume, centroid, inertia | Documented (Gauss integration) | In-scope | Partial: exact planar boundary moments (line/circle/parabola); curved-body volume is tessellation-clamped; inertia exists | Gap-measured — OpenZCAD S2 measures 0.2–3.5 % volume error on filleted parts; the reference integrates surfaces directly | K-S2 disposition; `properties/face_integrator.rs`; OpenZCAD `docs/kernel-roadmap-remus.md` S2 | B20 → new |
| IP-10.2 | Bounding boxes and oriented boxes | AABB, OBB documented | In-scope | Qualified AABB (body-class aware); OBB exists in math, unexposed | Gap-measured (no public OBB) | `math/src/obb.rs` (one internal consumer) | 7.5 (scope note) |
| IP-10.3 | Point and body classification | Documented | In-scope | Qualified (three cavity classifiers) | Parity-proven | ledger row | done; B16 (batched `classifyPoint`) |
| IP-10.4 | Minimum distance and extrema | Documented | In-scope | Qualified bounded (point/solid, solid/solid) | Parity-proven on witnesses | `check/src/distance/` | B10 |
| IP-10.5 | Clash and interference | Documented (polyhedron interference accelerated in the current reference release) | In-scope | Absent (no assembly clash query) | Behind-unmeasured | audit item 8 | 7.5 |
| IP-10.6 | Wall thickness | Not documented in the reference kernel | In-scope (consumer pull: printability) | Absent | Unknown → assigned | — | 7.5 (scope note) |
| IP-10.7 | Draft analysis | Documented (draft-angle checker) | In-scope | Absent | Behind-unmeasured | — | 7.5 |
| IP-10.8 | Curvature and minimum-radius analysis | Documented | In-scope | Partial (surface curvature, min radius; no curve continuity/G2) | Parity on surfaces | 7.5 partial | 7.5, 7.6 |
| IP-10.9 | Geometric comparison and deviation; scale/transform invariance | Not documented | In-scope | Partial (distance-based comparison; scale matrices per family) | Lead candidate | 1e-3/1/1e3 doctrine | testing strategy (standing) |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-10.1 | exact curved-face area and volume integrator; ellipse/hyperbola/NURBS planar boundaries; deflection-independent results | `check/src/properties/`, `operations/src/measure/` | N · W · Wb · Py | Exact (Gauss over exact geometry) with stated error bound | primitive/fillet/cavity × scale; error vs closed form ≤ 1e-6 relative | B20 gate | measurement provenance |
| IP-10.2/10.5–10.8 | OBB exposure; clash with witness points; wall thickness (ray/offset based); draft analysis; silhouettes | `check/src/analyze/`, `operations/src/distance.rs`, `wasm/bindings/measure.rs` | N · W · Wb · Py | Exact on analytic; sampled with bound on NURBS | touching/clearance/interfering triples; thin-wall corpus; pull-direction × angle | 7.5 gate (extended) | — |

### 5.11 Interchange and semantic product data

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-11.1 | STEP AP203/AP214/AP242 declared profiles | AP214 primary, AP203, AP242 partial | In-scope | Partial: reader schema-agnostic per entity; writer AP203 only | Gap-measured (AP242 write) | `io/src/step/` | O5.3a |
| IP-11.2 | Geometric and topological round trips; units; uncertainty | Documented | In-scope | Qualified (per-use pcurves, byte-identical write/read/write, validation properties) | Parity-proven; lead on deterministic output | 2.0f, O1.4a, B11 | done; B13 (voids, in PR #251) |
| IP-11.3 | Assemblies, occurrence transforms, shared definitions, instancing | Product structure read/write (XDE) | In-scope | Absent (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` only in a fixture) | Gap-measured | audit item 9 | O5.1, O5.4 |
| IP-11.4 | Names, colors, layers, materials | XDE names/colors/layers | In-scope | Partial: names round-trip; colors absent in both directions; no edge/vertex attributes | Gap-measured | `attributes.rs` (solid/face only); e3b design | O5.2 |
| IP-11.5 | Validation properties | Documented | In-scope | Qualified (CAx-IF 4.6 opt-in) | Parity-proven; lead on transactional malformed refusal | `step-conformance.md` | O1.4b (test rounds) |
| IP-11.6 | External references, partial loading | Not documented for the basic translator | Later/horizon | Absent | Deferred (decision) | — | O5.5 → new |
| IP-11.7 | Semantic and presentation PMI/GD&T | XDE PMI read/write | In-scope (read), Later (write) | Absent | Gap-measured | audit item 9 | O5.3b/c |
| IP-11.8 | Tessellated representations; void shells; surface models | Documented | In-scope | Partial (sheets qualified; voids read + write in PR #251; no tessellated STEP) | Gap-measured (tessellated STEP read) | 4.2, B13 | B13 complete (PR #251); tessellated STEP: non-goal until corpus pull |
| IP-11.9 | Deterministic output; malformed-input and resource-limit behavior; write/read/write stability | Not guaranteed | In-scope | Qualified | Lead (LC1, LC5) | `io::limits`, fuzz targets, B11 | done |
| IP-11.10 | Mesh formats (STL, 3MF, OBJ, PLY, glTF) | Documented | In-scope | Qualified bounded | Parity-proven on limits; Gap-measured on round-trip evidence (issue #244/#247/#245) | ledger row; open issues | O1.1d, PR #251 |
| IP-11.11 | IGES | Documented (5.3) | Intentionally out of scope | Experimental by decision | Out of scope | stabilization C3 | reassessed: no reopen (§1) |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-11.1 | AP242 header/schema; two independent receivers | `io/src/step/writer.rs` | N · W · Wb · Py | schema validation | receiver × solid count × volume | O5.3a gate | — |
| IP-11.3 | product structure read/write; occurrence ids | `io/src/step/{reader,writer}.rs`, `assembly.rs` | N · W · Wb · Py · Ev | flatten(read(write(a))) == a.flatten() | depth × sharing × third-party corpus | O5.1 + O5.4 gates | naming-anchored occurrences |
| IP-11.4 | colour/style chains; edge/vertex scope | per e3b | N · W · Wb · Py | bit-stable attribute payload | attribute × entity × round trip | O5.2 gate | — |
| IP-11.6 | design + decision | `docs/design/` | — | — | — | O5.5 decision record | — |
| IP-11.7 | PMI read bound to persistent refs | `io/src/step/pmi.rs` (new) | N · W · Wb · Py · Ev | refs survive a direct edit | CAx-IF PMI fixtures | O5.3b gate | S7-class demo |
| IP-11.10 | round-trip fixtures for #244/#245/#247; faceted import contract | `io/src/stl/import.rs`, `heal.rs` | N · W · Wb | property agreement | format × size × malformed class | PR #251 + O1.1d classes | — |

### 5.12 Identity, lifecycle, and document semantics

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-12.1 | Complete preserved/modified/generated/split/merged/deleted evolution for every topology-producing operation | Generated/Modified/Deleted for GF and some local ops | In-scope | Qualified for booleans, blends, patterns, draft, defeature, split, shell, default offsets; barriers for direct edits, arc-joint/self-int offsets, unify, sew, sweeps' non-source faces | Lead where covered; Gap-measured where barriers remain | ledger Evolution row; OpenZCAD C1 (`unifyFaces` lineage is the top adapter ask) | 6.5, B18 → new |
| IP-12.2 | Persistent references for every topology-producing operation; deterministic explainable rebinding; ambiguity refusal | Not a kernel capability (application-level naming) | In-scope | Qualified (RFC 0003 stages 1–5, WASM surfaced) | Lead (LC4) — **unused by OpenZCAD today** (hash-based lineage, 12 of 18 classes hash-only) | `topology/src/naming.rs`; OpenZCAD `topology-lineage.ts` | done; adoption → B16/O6.3 |
| IP-12.3 | Transactionality and rollback | Non-destructive mode (inputs preserved) | In-scope | Qualified (`run_transacted`; all blend/boolean/heal paths) | Lead (typed rollback, byte-identical state) | operation contract | remaining ops migrate incrementally (2.8 family adoption) |
| IP-12.4 | Versioned snapshots and deltas; incremental cache invalidation | OCAF undo/redo (application framework) | Integration-adjacent | Partial: full deep snapshots per checkpoint; no deltas; journal supports precise invalidation but no consumer uses it yet | Gap-measured (checkpoint cost bounds OpenZCAD history to 32) | `topology/src/arena.rs`; O3.2 pending | 8.6, O3.2, O3.4 |
| IP-12.5 | Schema migrations; attribute propagation | Persistence versions | In-scope | Qualified (arena v1–v5; journal-driven attribute propagation) | Lead | RFC 0003 stage 4/5 | O4.6 (policy) |
| IP-12.6 | Long-session memory behavior | Reference-counted | In-scope | Gap (no compaction) | Gap-measured | e6b | 8.6 |
| IP-12.7 | Collaboration-friendly deterministic replay; branch/rebase/merge | Not a kernel capability | Later/horizon (merge); In-scope (replay) | Qualified replay (repro bundles, checkpoint prefix replay used by OpenZCAD) | Lead (LC5); merge semantics deferred | `repro.rs`; OpenZCAD `buildWithHistoryCache` | O4.6 (delta format decision) |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-12.1 | audit every family; `unify_with_evolution`, `sew_with_evolution`, sweep/loft caps, arc-joint offsets, direct edits | `journal_ops.rs`, `evolution.rs`, per-op modules | N · W · Wb · Ev | total attribution (every result face claimed once) or typed unresolved | family × operation × fixture | B18 gate + 6.5 gate | LC3 |
| IP-12.4/12.6 | compaction; delta snapshot format decision | `arena.rs`, `checkpoint.rs` | N · W · Wb | exact restore | history depth 256 × memory ceiling | 8.6 gate | LC11 |
| IP-12.5/12.7 | written policy; migration matrix | `arena_io.rs`, `repro.rs` | N · W · Py | byte identity | reader × writer versions | O4.6 gate | LC5 |

### 5.13 Sketch-to-B-Rep integration (leadership item)

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-13.1 | Well-, under-, over-, redundant-, inconsistently-constrained systems with deterministic diagnostics | No constraint solver in the reference kernel | Integration-adjacent (leadership) | Stable, evidence pending (nonconvergence budget and degeneracy matrix incomplete) | Lead-unproven (no reference); needs the GCS matrix | `sketch/src/gcs/`; `gcsSolveDetailed`; OpenZCAD uses it in production | §6 inherited queue "Sketch (GCS) qualification" → B16-adjacent; 6.6 |
| IP-13.2 | Projection of B-Rep geometry into sketches; external-geometry references via persistent topology identity; constraint survival after model edits | None | Integration-adjacent (leadership) | Absent (sketch crate has no topology semantics; OpenZCAD attaches sketches to faces by witness hash and breaks on 12 hash-only lineage classes) | Behind-unmeasured — the LC10 opportunity | audit item 11; OpenZCAD `face-attachment.ts` | 6.6 → new |
| IP-13.3 | Profiles with holes and multiple regions; direct handoff into sweep/revolve/loft/features | Wire/face construction | In-scope | Qualified (`makeFaceFromWires`, `addHolesToFace`, wire-body sweep) | Parity-proven | `holed_face_tests.rs`, 4.7 | done |
| IP-13.4 | Scale behavior of the solver | — | Integration-adjacent | Partial (`check_jacobian_central` at large coordinates) | Unknown → matrix | `gcs/constraint/tests.rs` | GCS matrix (B16-adjacent, inherited queue) |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-13.1/13.4 | generated constraint × state × scale matrix; nonconvergence budget | `sketch/src/gcs/`, `operations/tests/qualify_gcs.rs` (new) | N · W · Wb · Py | exact residuals; DOF/rank oracle | constraint type × system state × scale; budget refusal typed | inherited-queue row gate (declared in §5.17 as part of B16's evidence set) | LC10 |
| IP-13.2 | sketch external references anchored on `PersistentRef`; projection of edges/faces to the sketch plane; re-solve after edits | `operations/src/sketch.rs`, `topology/src/naming.rs`, `wasm/bindings/gcs_sketch.rs` | N · W · Wb · Py · Ev | exact projection; refs resolve Bound/BoundMany/typed | edit class × reference class; ambiguity refusal | 6.6 gate | LC10 |

### 5.14 Performance, concurrency, and scale

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-14.1 | Measured inner-loop performance | Not published | In-scope | Qualified baselines (O3.1) | Unknown until O1.2f pins the baseline | `o31-inner-loop-baseline.md` | O1.2f, 8.2 |
| IP-14.2 | Cached spatial acceleration | Bounding-box caches per shape | In-scope | Absent (seven BVH sites rebuild per query) | Behind-unmeasured | O3.2 pending | O3.2 |
| IP-14.3 | Deterministic parallel tessellation and boolean phases | In-parallel mesher; parallel GF | In-scope | Partial (rayon at two tessellation sites; determinism gates) | Gap-measured (no parallel booleans; parallel tessellation unmeasured for scaling) | audit item 12 | 8.3, 8.4 |
| IP-14.4 | Concurrent readers, independent sessions, thread-safe import/export | Thread-safe STEP read/write per thread (current reference release) | In-scope | Partial (no `unsafe`; no Send/Sync audit; `Model` sessions independent by construction) | Behind-unmeasured | audit item 12 | 8.4 (scope note: session-concurrency test) |
| IP-14.5 | Large assemblies and large B-Reps; out-of-core/streaming | Not documented | In-scope (large B-Reps); Later (streaming) | Unqualified | Behind-unmeasured | gauntlet abc-1k timing | O1.1, 8.6 |
| IP-14.6 | Incremental recomputation | Incremental mesh only | Integration-adjacent | Partial (feature-prefix replay in OpenZCAD; no kernel-side delta) | Gap-measured | OpenZCAD `buildWithHistoryCache` | O3.2, O3.4 |
| IP-14.7 | Peak memory, entity growth, budgets | Not documented | In-scope | Partial (six SSI budgets; no memory or generated-topology budget) | Lead candidate once budgets complete | 2.8 partial | 2.8, 8.2 |
| IP-14.8 | WASM linear-memory behavior; threads; SIMD; cold init; package size | Emscripten builds (single-threaded by default) | In-scope | Partial: size gate (K-W3, 7.7 MB with 664 KB headroom); `--enable-simd` in wasm-opt; no threads; cold-init unmeasured | Lead candidate (LC7) pending measurement | K-W3 disposition; `crates/wasm/Cargo.toml` | 8.7 → new (threads/SIMD evidence), O1.2f (cold init) |
| IP-14.9 | Cancellation latency and hard work budgets | Progress indicator/cancel in some algorithms | In-scope | Partial (boolean/SSI cooperative cancellation; **OpenZCAD does not use it** because a running WASM call cannot be interrupted from the same thread) | Lead candidate (LC9) blocked on family adoption and a worker-side story | 2.8; OpenZCAD `geometryWorker.ts` | 2.8, 8.7 |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-14.1 | baseline pin | `tools/vs-bench/` | — | §3.2 equivalence | §3.3 pins | O1.2f | LC13 |
| IP-14.2 | journal-invalidated cache | `topology/src/spatial.rs` | N | mutate-then-query truth | ≥5× repeated classify | O3.2 gate | LC11 |
| IP-14.3/14.4 | per-face parallelism with deterministic reduction; session-concurrency test | `tessellate/solid.rs`, `algo/` | N · W (single-thread) | bit-identical serial vs parallel | 200-run determinism gate; ≥3× on 8 cores; N concurrent `Model`s | 8.3, 8.4 gates | LC5 |
| IP-14.7 | memory and generated-topology budgets in `OperationContext` | `math/src/context.rs`, family adopters | N · W · Wb | typed `resource_limit` with amount consumed | budget × family; overrun refusal | 2.8 gate (extended) | LC9 |
| IP-14.8 | threads/SIMD evidence; cold-init and size columns | `crates/wasm/Cargo.toml`, xtask | W | ≥1.5× or documented negative | size raw/gzip/brotli; cold init p50/p95 | 8.7 gate; O1.2f columns | LC7 |
| IP-14.9 | cancellation across sweeps/blends/offset/tessellation/import; worker-side token pattern documented | family modules, `wasm/bindings/` | N · W · Wb | typed `cancelled`, rollback complete | latency ≤ budget per family | 2.8 gate (extended) | LC9 |

### 5.15 API, distribution, and ecosystem

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-15.1 | Curated Rust facade | C++ API | In-scope | Qualified (46 `pub fn` on `remus::Model`; examples) | Parity by door; lead on typed errors | O4.1a/b done | O4.1c |
| IP-15.2 | JavaScript/TypeScript and WASM | Emscripten JS wrapper (third-party) | In-scope | Qualified (342 exported methods; 129 batch ops; tsify types) | Lead (native wasm-bindgen; **OpenZCAD calls ~115 methods, uses no batch API**) | audit items 18; OpenZCAD §3 | done; O4.7 (typed direct results) |
| IP-15.3 | Python | Third-party bindings (OCP) | In-scope | Absent | Gap-measured | O4.3 pending | O4.3 |
| IP-15.4 | Stable C ABI | C++ only | Later/horizon (decision) | Absent | Decision | audit item 13 | O4.5 → new |
| IP-15.5 | Semantic versioning; stable diagnostic registry; serialized-format compatibility; release provenance | Versioned releases | In-scope | Partial: `=0.1.0` pins, `2.130.0` WASM train, release-please; `OperationsError` outside the registry; no written serialization policy; provenance gated | Gap-measured (nothing published) | O4.2, O4.4 pending | O4.2, O4.4, O4.6 |
| IP-15.6 | Documentation, examples, browser playground, package-manager installation | Docs site, samples | In-scope | Partial (book, rustdoc, facade examples; no site, no playground, no packages) | Gap-measured | O6.1/O6.2 pending | O6.1, O6.2, O4.2c (owner) |
| IP-15.7 | Second and third real consumers | Large ecosystem | In-scope | One first-party consumer | Gap-measured | S6 | O6.3 |
| IP-15.8 | Extension/plugin boundaries; migration tooling | Plugin system for meshers/DE | Later/horizon | `HealOperator` trait registry is the only plugin seam | Deferred | `heal/src/pipeline/` | non-goal until a consumer asks |
| IP-15.9 | Consumer topology-query API set | Broad query API | Integration-adjacent | Partial: OpenZCAD reimplements trimmed edge domain, face material sense, ordered wire traversal, per-edge convexity, max fillet radius, batched classify | Gap-measured (product-pull) | OpenZCAD roadmap C2 | B16 → new |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-15.1/15.2 | delegation; typed direct results | `wasm/src/kernel.rs`, `bindings/*` | N · W · Wb | contract suite byte-identical | all direct methods with a `*Detailed` twin | O4.1c, O4.7 gates | LC8, LC12 |
| IP-15.3 | PyO3 over the facade | `bindings/python/` | Py | mirrored contract suite | three OS wheels | O4.3 gate | LC12 |
| IP-15.4 | decision record | `docs/design/` | — | — | — | O4.5 record | — |
| IP-15.5 | registry; policy; dry-run automation | `operations/src/`, `docs/`, workflows | N · W · Py | registry-completeness test | every variant maps; every version pair reads | O4.2a/b, O4.4, O4.6 gates | LC12 |
| IP-15.9 | one binding per named heuristic, each retiring an OpenZCAD reimplementation | `wasm/bindings/query.rs`, `batch.rs`, `operations/src/query.rs` | N · W · Wb · Py | exact | per-query matrix; typed refusal on foreign handles | B16 gate | consumer symbiosis |

### 5.16 Assurance and security

**A**

| ID | Outcome | Reference capability | Disposition | Contract | Competitive | Evidence | Owner |
|---|---|---|---|---|---|---|---|
| IP-16.1 | No crash, panic, UB, or partial mutation on untrusted input | Exceptions; consumers wrap in guards | In-scope | Qualified (lint-denied unsafe/panic; `io::limits`; transactional rollback) | Lead (LC1) pending harness publication | audit IO-003; fuzz.yml | done; O1.2 publication |
| IP-16.2 | Hard memory, entity, iteration, and time budgets | Progress/cancel in some algorithms | In-scope | Partial (import limits, batch caps, six SSI budgets; no memory/time budget in context) | Lead candidate | 2.8 partial | 2.8 |
| IP-16.3 | Deterministic reproduction bundles | Draw scripts | In-scope | Qualified (schema 1, 12 bundles, native + WASM replay) | Lead | `repro.rs` | done; O4.6 schema policy |
| IP-16.4 | Fuzzing; property and metamorphic testing; mutation testing; corpus regression | GTest suites (current reference release) | In-scope | Qualified (17 fuzz targets, weekly; `mutants.toml`; proptest; gauntlet); curve-intersection, offset, GCS, tessellation fuzz slices outstanding; `mutants.toml` `cdt.rs` glob is stale (CDT is a directory) | Lead (LC13) with the outstanding slices owned | testing strategy; audit item 13 | B19 → new |
| IP-16.5 | Native/WASM parity; cross-platform determinism | Not claimed | In-scope | Partial (per-op contract tests; no systematic per-operation invariant harness; platform matrix pending) | Lead candidate (LC8) | testing strategy "CI growth path" | O1.5 → new |
| IP-16.6 | Supply-chain and release provenance | Signed releases | In-scope | Partial (deny, OSV, RustSec, lockfiles; SBOM/attestation pending; no publish) | Gap-measured | audit CI row | O4.2b |
| IP-16.7 | Public, reproducible benchmark evidence | Not published | In-scope | Absent (harness pending) | Gap-measured | O1.2 pending | O1.2a–f |

**B**

| ID | Gap / dependency | Footprint | Surfaces | Policy & oracle | Matrix / boundary / perf | Exit gate | Lead |
|---|---|---|---|---|---|---|---|
| IP-16.2 | memory/time budgets | `math/src/context.rs` | N · W · Wb | typed `resource_limit` | overrun refusal per family | 2.8 gate (extended) | LC9 |
| IP-16.4 | remaining fuzz slices; mutants glob fix | `fuzz/fuzz_targets/`, `mutants.toml` | N | invariant oracles per slice | weekly schedule; seeds committed | B19 gate | LC13 |
| IP-16.5 | per-operation differential harness native vs WASM over the batch op list | `tools/parity/` (new) or `crates/wasm/tests/` | N · W · Wb | invariant equality (volume, census, diagnostics codes) | every batch op × fixture; platform matrix | O1.5 gate | LC8 |
| IP-16.6 | SBOM + attestation in the release workflow | `.github/workflows/publish.yml` | — | — | dry-run proven | O4.2b gate | — |

### 5.17 Rows added by this overlay (ownership gaps closed)

Every row below was verified unowned against the code, the ledgers, and
the two open PRs (#251, #252) at drafting time. Each is specified in the
program that owns it; this table is the index, not a ledger.

| New ID | Program | Title | Closes crosswalk rows | Spec |
|---|---|---|---|---|
| 4.8 | P-Class M4 | N-ary and mixed-dimensional General Fuse with recursive lineage | IP-2.2, IP-3.3, IP-3.4 | [p-class-program.md](p-class-program.md) §4.8 |
| 5.7b | P-Class M5 | General offset self-intersection removal beyond the L-prism cell | IP-5.1, IP-5.3, IP-5.4 | §5.7b |
| 5.8 | P-Class M5 | Blend rollover through re-limitation | IP-4.2, IP-4.4, IP-4.7 | §5.8 |
| 6.6 | P-Class M6 | Sketch external references on persistent topology identity | IP-13.2 | §6.6 |
| 7.6 | P-Class M7 | Curve construction, fairing, degree reduction, and continuity analysis | IP-1.5, IP-1.6, IP-6.8, IP-8.3 (degree reduction) | §7.6 |
| 7.7 | P-Class M7 | Wire and curve offset completeness | IP-5.5 | §7.7 |
| 8.6 | P-Class M8 | Arena compaction and the versioned checkpoint contract | IP-2.8, IP-12.4, IP-12.6 | §8.6 |
| 8.7 | P-Class M8 | WASM threads and SIMD evidence gate | IP-14.8, IP-14.9 | §8.7 |
| O1.2d | Open Kernel O1 | Scorecard metric schema and absolute gates | §3 | [open-kernel-implementation.md](open-kernel-implementation.md) O1.2d |
| O1.2e | Open Kernel O1 | Workflow scenarios W1–W9 | §4 | O1.2e |
| O1.2f | Open Kernel O1 | Baseline pin milestone | §3.3, IP-14.1 | O1.2f |
| O1.5 | Open Kernel O1 | Native/WASM per-operation parity harness | IP-16.5 | O1.5 |
| O2.4 | Open Kernel O2 | Predicate escalation policy for topology decisions | IP-1.9 | O2.4 |
| O3.4 | Open Kernel O3 | Journal-driven incremental tessellation | IP-9.5, IP-14.6 | O3.4 |
| O4.5 | Open Kernel O4 | Stable C ABI decision record | IP-15.4 | O4.5 |
| O4.6 | Open Kernel O4 | Serialization compatibility and migration policy | IP-2.7, IP-12.5, IP-12.7 | O4.6 |
| O4.7 | Open Kernel O4 | Typed direct-method results on the WASM surface | IP-15.2 | O4.7 |
| O5.4 | Open Kernel O5 | Assembly occurrence identity and shared-definition instancing | IP-2.3, IP-11.3 | O5.4 |
| O5.5 | Open Kernel O5 | External references and partial loading — design and decision | IP-11.6 | O5.5 |
| B16 | Bridge | Consumer topology-query API set | IP-15.9, IP-9.3, IP-10.3 | [roadmap.md](roadmap.md) §B |
| B17 | Bridge | Healing defect-class qualification matrix | IP-8.3, IP-3.8, IP-8.6 | §B |
| B18 | Bridge | Evolution completeness audit across every operation family | IP-3.6, IP-5.7, IP-12.1 | §B |
| B19 | Bridge | Remaining fuzz slices and the mutants glob | IP-16.4 | §B |
| B20 | Bridge | Exact measurement completion (K-S2 remainder) | IP-10.1 | §B |

Scope notes added to existing rows (no new ID): 7.3 gains Gordon
networks; 7.5 gains wall-thickness analysis and OBB exposure; 2.8 gains
memory/generated-topology budgets and family-wide cancellation adoption as
explicit exit items; 8.4 gains a concurrent-session test.

Rows confirmed **already owned** and therefore not re-added: curve-curve
and curve-surface qualification (B10); conic edges (O2.2); arrangement
splitter (O2.3); revolution/extrusion carriers (O2.1); tangent/sliver
contacts (2.7); scale bands (2.6); NURBS×NURBS booleans (2.5); tolerant
modeling (3.4–3.6); curved-support blends, corners, setbacks (5.2–5.4);
face-face blends (5.6); direct modeling (6.1–6.5); guided sweeps, loft
continuity, N-sided fill, surface extension (7.1–7.4); clash, silhouettes,
draft analysis (7.5); differential harness, perf gates, parallelism, real
corpus (8.1–8.5); STEP assemblies, colors, AP242, PMI (O5.1–O5.3);
publishing, Python, error registry (O4.2–O4.4); docs, playground,
consumers, contributing (O6.1–O6.4); hybrid RFC (O7); closed-rim chamfers
(B3); trimmer completion (B4); evidence matrices (B6); pave-block
attachment (B7); STEP voids (B13, PR #251); cap holes (B12, PR #252).


## §6 Post-v1.0 horizons and their gates

H0–H4 are unchanged (see [roadmap.md](roadmap.md) §H). The three horizons
below are appended after H4; each gate is either absolute (§3.4) or a band
placeholder locked by O1.2f.

### H5 — Core modeling parity (authored-from-scratch domain)

Purpose: prove parity across the in-scope authored-modeling domain.

1. This crosswalk is complete: every in-scope row has a non-`Unknown`
   competitive state and an owner, and §5.17's rows are on their ledgers.
2. Zero `Unsupported-untyped` cells in the capability matrix (carried from
   H4 and re-verified).
3. W1, W2, W5, and W7 complete correctly through both surfaces — not by
   refusal — at 1e-3/1/1e3 scale.
4. General curved booleans qualified: 2.4c/d, 2.5, 2.6, 2.7 closed; 4.8
   N-ary and mixed-dimensional cells qualified or typed with a named
   primitive.
5. Broad matrices qualified for fillet (M5 rows plus 5.8 rollover), chamfer
   (B3), shell/offset (5.7b, B5 done), draft (6.4), sweep (7.1), loft (7.2),
   direct edit (6.2, 6.3), with typed both-sides boundaries everywhere else.
6. Complete topology evolution for all covered operation families (B18 audit
   at zero unowned unresolved classes; 6.5 closed).
7. Stable Rust facade (O4.1c delegation, O4.2a/b dry-run green), stable
   JS/WASM contract (O4.4 registry), and the planned Python surface (O4.3a/b)
   passing the mirrored contract suite.
8. Parity measurements published for the modeled-from-scratch scenarios
   (W1, W2, W5, W7) with O1.2f's baseline pinned and the §3.4 absolute gates
   green; numeric bands: `[locked by O1.2f]`.

### H6 — Industrial interchange and corpus parity

Purpose: comparable outcomes on real supplier data, assemblies, and large
models.

1. W3 passes: dirty STEP models are diagnosed, tolerated (M3.5, B2 exit
   benchmark) or verified-repaired (B1), operated on (M3.4), and re-exported
   with complete disclosure; zero heal invocations on the tolerant path.
2. AP242 product structure (O5.1, O5.3a), attributes (O5.2), and the
   declared PMI read profile (O5.3b) qualified against CAx-IF test-round
   models (O1.4b).
3. Assembly occurrence identity and instancing stable (O5.4) across
   round trip and edits.
4. Real-model gauntlet stage pass rates within locked parity bands
   `[locked by O1.2f]`, with the taxonomy breakdown public (O1.1c/d).
5. Zero `silent_wrong`, `crash`, and unbounded `hang_or_budget_overrun`
   across the gated corpus on both kernels' comparison rows.
6. Imported-model boolean, blend, offset, direct-edit, tessellation, and
   measurement stages within bands `[locked by O1.2f]`.
7. Large-model memory and tail-latency budgets enforced (8.2, 8.6, O3.2,
   O3.4) with W8 passing under a declared memory ceiling.
8. Deterministic concurrent sessions and supported parallel operations
   (8.3, 8.4 under the 200-run determinism gate; 8.7 decision recorded).
9. A second serious external consumer exercising the native or Python API
   with its corpus in Remus CI (O6.3, S6).

### H7 — Demonstrated technical leadership

Purpose: independently reproducible areas where Remus is measurably better.
Requires a minimum portfolio of **five** leadership claims from §7 meeting
the leadership discipline, at least two of them in the correctness family
(LC1–LC4) and at least one in the browser family (LC7–LC9).

## §7 Leadership claims and the discipline they must meet

A leadership claim is admissible only with: a stable benchmark or corpus; an
equivalent output-quality requirement (§3.2); a pinned baseline (O1.2f);
repeatable results (N runs, method stated); known losses published beside
the wins; and an always-on regression gate where practical. Each claim below
names its measuring row.

| ID | Claim | Measured by | Regression gate |
|---|---|---|---|
| LC1 | Safer behavior on hostile input (zero crash/UB, typed refusal, unchanged session) | W9, fuzz campaign, O1.1 taxonomy | fuzz.yml, `io::limits` regressions |
| LC2 | Fewer silent wrong answers | `silent_wrong` column across W1–W8 | approx census, gauntlet ratchet |
| LC3 | More complete and explainable topology evolution | `evolution_completeness` (B18 audit) | `qualify_evolution_coverage.rs` |
| LC4 | Better persistent-selection survival under edits | `persistent_ref_survival` in W7 | naming regressions, S7 demo |
| LC5 | Deterministic replay and byte-stable serialization | `nondeterminism` = 0; arena byte identity; deterministic STEP | 64-cut gate, `arena_roundtrip` fuzz, B11 STEP ordering |
| LC6 | Explicit exact/approximate/repair provenance | `disclosed_*` columns; B1, B5 disclosures | census, verified-heal regressions |
| LC7 | Lower browser payload and startup cost | `module_size_*`, `wasm_cold_init` | xtask size gate (K-W3) |
| LC8 | Native/WASM behavioral consistency | `native_wasm_agreement` (O1.5) | contract-test suite, O4.1c structural delegation |
| LC9 | Stronger cancellation and resource bounding | `cancellation_latency`, `hang_or_budget_overrun` | 2.8 contract tests |
| LC10 | Integrated sketch-to-B-Rep persistent references | W1 + W7 sketch stages (6.6) | GCS matrix + naming regressions |
| LC11 | Faster incremental rebuild or remeshing | O3.2/O3.4 repeated-query and remesh benches | bench-compare baselines |
| LC12 | Easier package installation and API use | S4 (three doors) time-to-first-solid | O4.2/O4.3 CI |
| LC13 | More reproducible public robustness evidence | S1–S3 scoreboards regenerable from manifest + SHA | results branch, R9 |

Known-loss publication is mandatory: the O1.2c results page carries a
"where the reference kernel wins" table generated from the same runs.

## §8 Prioritization and critical path

Ordering uses the sanctioned chase filters and TERMINAL list in
`.claude/skills/roadmap/SKILL.md`, then, in order: correctness and
silent-wrong risk; dependency unlock; OpenZCAD user impact; industrial-corpus
frequency; breadth of downstream operations improved; existence of a stable
reproduction or independent oracle; ability to cut a bounded vertical slice;
file-collision risk. Feature count, novelty, and ease are not criteria.

The critical path, tested against the repository rather than assumed:

```
explicit trims / p-curves / body topology (2.0 done, M4 done)
  → general surface intersection + UV arrangement (2.4c/d, 2.5, O2.3)
    → tolerant modeling integration (3.4, 3.5)
      → general curved booleans, N-ary (2.6, 2.7, 4.8)
        → healing/import integration on real corpora (O1.1d, B17, W3)
          → local operations and direct modeling (5.8, 6.2–6.5, 6.6)
            → advanced sweeps and surfacing (7.1–7.4, 7.6, 7.7)
              → semantic exchange and assemblies (O5.1–O5.5)
                → large-model performance and deterministic parallelism
                  (O3.2, O3.4, 8.2–8.4, 8.6, 8.7)
                    → hybrid B-Rep/faceted modeling (O7, RFC only)
```

The one deviation from the assumed order: healing/import integration sits
*before* local operations here because the gauntlet already runs and W3 is
the OpenZCAD-relevant workflow with the highest measured failure rate
(smoke 29/50 full passes at the last pinned manifest); M6 has no consumer
pull until imported bodies survive it.

## §9 Owner decisions this overlay leaves open

Recorded here so no session resolves them by default:

1. **O4.5 — stable C ABI:** adopt (feeds Python via a C layer, opens
   C++/C# consumers) or decline (PyO3 direct, no ABI surface). Decision
   record only; no implementation either way before the record.
2. **O5.5 — external references and partial loading:** whether the kernel
   owns any part of it or the application document does. Design note plus
   decision.
3. **8.7 — WASM threads and SIMD:** evidence-gated like O3.3; the owner
   decides whether the OpenZCAD deployment can accept the cross-origin
   isolation headers threads require.
4. **IGES:** stays Option 2 (decided 2026-08-21). Reopening needs consumer
   or corpus evidence and an explicit owner decision.
5. **Non-manifold / cellular with shared faces:** later RFC per RFC 0005;
   whether it precedes or follows O7 is the owner's call.
6. **v1 fillet API migration:** unchanged product decision; H5's blend
   gate does not depend on it.
7. **First publish (O4.2c) and PyPI (O4.3c):** owner-gated; H5 gate 7
   requires dry-run readiness only, not the publish itself.
