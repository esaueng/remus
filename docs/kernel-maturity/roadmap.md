# Unified forward roadmap

The one page a session — human or agent — reads to know what to work on
next, and where every open workstream lives. It merges the three sources of
record and the bridge backlog neither program owns:

| Source | Covers | Ledger |
|---|---|---|
| [P-Class program](p-class-program.md) | Correctness & capability (M2–M8) | [p-class-status.md](p-class-status.md) |
| [Open Kernel program](open-kernel-program.md) · [implementation plan](open-kernel-implementation.md) | Proof, adoption, interchange, ecosystem (O1–O7) | [open-kernel-status.md](open-kernel-status.md) |
| [Stabilization plan](stabilization-plan.md) | Historical label promotions; residue absorbed below | its Dispositions section |
| **Bridge backlog (§B below)** | Ready items covered by neither program | §B table, updated in-place |
| [Campaign history](campaign-history.md) | Closed roots, refuted theories, and the digs behind them (not loaded by default) | read-only, append when a dig closes |
| [Industrial parity overlay](industrial-parity.md) | Non-owning competitive crosswalk against the reference kernel, scope contract, scorecard, workflow scenarios, H5–H7 gates | no ledger — points at the rows above; update pointers when an owner row flips |

The work-selection *doctrine* (chase filters, TERMINAL list, acceptance bar,
durable lessons) remains `.claude/skills/roadmap/SKILL.md`; this page is
the *queue*; the narrative behind closed rows is `campaign-history.md`. All
are living documents: update the relevant row in the same PR that changes
its state. Before claiming anything: `gh pr list --state open` (R6).

- **Drafted:** 2026-08-29, baseline `main` @ `3c232e8`.
- **External K-S1 disposition — tangent-boss operand drop: done (PR #143,
  2026-08-30).** The historical pre-fix sequence returned the unchanged plate
  (19,200 instead of 21,713.274 cubic millimetres); current native and WASM
  contracts retain the operand across the ratio/scale matrix, with exact-only
  refusal and explicit approximation provenance where the exact path is unavailable.
- **External K-S1 disposition — cross-drilled render/measure:** done in PR
  #144. The OpenZCAD operation sequence now has a deterministic replay bundle,
  independent volume oracles, ratio/scale display-mesh qualification, and a
  non-vacuous WASM `meshQuality` contract. Follow-ups remain for the separate
  face-orientation inconsistency and the sub-millimeter fine-mesh boundary
  residue; neither is hidden by this disposition.
- **External K-S1 disposition — fillet fail-closed migration: done (PR #181,
  2026-09-01).** Every public fillet/chamfer mutation path — the WASM `fillet`
  cascade, `filletVariable`, `filletV2`/`chamferV2`/`chamferDistanceAngle`,
  the journaled wrappers, `executeBatch`/`executeBatchV2`, and the legacy v1
  Rust engines — is transactional and postcondition-validated: no path returns
  the input handle or a clone of it as success, exposes partially mutated
  topology, or ships a geometrically invalid result; refusals carry the stable
  `blend_failure_code` vocabulary on every surface. The versioned repro bundle
  `fillet-variable-fail-closed` fails on the pre-fix kernel and passes after.
  Two damaged-success cases the old closed-shell gate could not see (the
  blend-adjacent second-pass fillet and the gridfinity lip peak-rim fillet)
  are honest typed refusals now; un-refusing them is the B4 trimmer work.
- **Remus K-S4 disposition — NURBS fuzz slice:** done in PR #163. Bounded
  rational-surface construction/evaluation and NURBS SSI now run in the
  scheduled fuzz campaign against an independent plane oracle; topology
  mutation, native serialization, curve-intersection, and offset-specific
  campaigns remain S4 follow-ups.
- **Remus K-S4 disposition — topology-mutation fuzz slice:** done in PR #170.
  Derivation, validated/transactional rollback, checkpoint restore, and
  solid-deletion sequences now run in the scheduled campaign over a bounded
  box against exact-state, stale-handle, atomic-refusal, closed-manifold
  census, and closed-form volume oracles. The campaign's first run found the
  rollback/restore contract split fixed in the same PR: transactional
  rollback now undoes in-window retirements (`restore_for_rollback`), and the
  checkpoint barrier no longer leaves a dangling face-loop derivation map.
  Native serialization, curve-intersection, and offset-specific campaigns
  remain S4 follow-ups; migrating ad-hoc snapshot/restore call sites off the
  checkpoint-barrier primitive is flagged for the W5 contract work.
- **Remus K-S4 disposition — native-serialization fuzz slice:** done in PR
  #173. Bounded box/cylinder documents with duplicate roots, shared-shell
  aliases, repeated/aliased compound members, hostile tolerances, and
  attributes now round-trip in the scheduled campaign against per-position
  validation/census/closed-form volume oracles, bit-exact state survival,
  byte-identical re-serialization, and typed non-mutating refusal of
  corrupted references. The byte-identity oracle pinned serde_json's
  `float_roundtrip` feature as load-bearing for exact f64 replay (fixed in
  the same PR with a bit-exact tolerance regression). Curve-intersection and
  offset-specific campaigns remain S4 follow-ups.

## §H Horizons

### H0 — in flight (verify before duplicating)

As of 2026-09-04: P-Class 2.0–2.3, 3.1–3.3, 4.1–4.7, 5.1–5.7, and 6.1 are
merged (see `p-class-status.md`); 2.4, 2.8, and 6.2 are partial — 6.2
generalized face moves (PR #257) qualifies a holed planar boss cap moving
through an incident constant-radius fillet with exact evolution and
direct/batch WASM parity, reusing 6.1 for coaxial bores; rotation, lateral
relocation, outward cylinders, and surface-type changes remain open. The
Open Kernel Wave A rows still unclaimed are O4.4, O5.1a–c, O6.1, and O6.4.
Bridge rows closed since the draft: B1, B5, the B6 primitive family
(including ellipsoid), the B11 STEP-ordering item, B13 void export, and B15
pocket-face orientation; B12 is partial (rectangular holes on sweep/pipe
caps, PR #252). The ignore inventory now includes the explicit unresolved
[anisotropic world-volume precision witness](scale-band-audit.md); the
remaining diagnostic, slow, and fork-policy skips retain their stated scope.

Current 2.4 qualification work covers bounded off-axis cone/sphere,
sphere/cylinder, and torus/sphere matrices; integration remains pending. The
2.6 through-tool family now has a 72-cell exact scale/placement matrix;
[remaining band audits](scale-band-audit.md) keep the broader item partial.
The torus/box notch follow-up now passes the 18-cell exact-only matrix in
`pclass_torus_notch_orientation`: three operations, scales 0.1/1/10, and origin
or rotated/translated placement. Gates include strict orientation/manifold
validation, analytic carriers, fitted-seam residual within linear tolerance,
watertight tessellation, volume within 0.1% of independent annular-section
quadrature, and signed mesh volume within 1%. Fixes cover complementary rim
traversal, carried trim domains, scale-relative march/branch distances, bounded
seam refitting, Fuse rim preservation, and oriented band integration. The new
public `integrate_torus_band_face` query lets measurement retain its fallback
for unsupported trims. Workspace Clippy, layer boundaries, warnings-denied docs,
and the unchanged 52-row census pass. Both WASM packages, smoke/installed
consumers (all 18 cells through direct and batch APIs), workspace doctests,
and the deterministic complexity guard pass. Full workspace: 4,842 passed,
13 skipped; a process-leak warning on the passing honeycomb case did not recur
in a clean four-test rerun. An idle performance comparison remains pending
because other browser jobs caused drift in the unchanged parent benchmark;
this follow-up is not integrated.

P-Class 2.7 has bounded qualification in [PR #307](https://github.com/esaueng/remus/pull/307),
which remains unintegrated.
The unintegrated `pclass_tangency_band` regression covers 27 operation/offset
groups, each at three scales and two placements. The current exact-or-typed
contract passes all 162 cells: 120 verified exact results and 42
`ExactOnlyUnattainable` refusals. Refusals are permitted only in the named
nonzero 1e-7/1e-9 offset groups and preserve operand arena bytes and topology
counts. Exact tangency and the larger offset groups must succeed. Every success
checks strict topology, analytic carriers, planar circle-carrier residuals,
independent circular-cap volume, material probes, and welded meshes at two
deflections. The 42 refusals remain unqualified for exact construction; this
matrix alone does not close the full tangency milestone.

The changes address distinct reproduced defects:

- Use the transform's linear matrix for curve directions, preserving Circle
  carriers under translated rotations.
- Tighten straight-edge carrier and exact arrangement T-junction tests, and
  coincident-vertex welding, without relaxing geometric acceptance.
- Split co-endpoint section arcs against straight chords and subdivide cylinder
  rim arrangements; keep selected planar regions inside their source polygon.
- Reject circular section PaveBlocks that leave the receiving plane, including
  extrema of their trimmed carrier rather than endpoints alone.
- Use the guarded analytic face-volume route before whole-solid mesh fallback.
- Exclude interior meshing-grid points on shared outer boundaries and route
  stepped cylindrical faces through the boundary-aware CDT mesher.
- Move full cylindrical-band classification samples off isolated plane contacts.
- Share tangent circle samples with straight edges on the same planar face
  before triangulation. This closes the six-edge exact-contact Fuse mesh gap
  across all six scale/placement cases at both tested deflections.

- Require the inner-wire term in both the pre-heal and final Euler acceptance
  gates. A raw Euler count of two no longer admits a malformed holed face.
- Keep CDT's collinearity distance at its existing vertex resolution rather
  than growing it with constraint length. This preserves a distinct nearby
  circle sample and the thin triangle between it and the straight boundary.
  The new unit fails on the old predicate; all 21 CDT tests pass with the fix.

Current focused checks pass the 162-cell tangency contract, 193 boolean units,
eight corner-placement cases, eight parallel-boss cases, the cavity and
multi-region checks, six import checks, the captured L-shaped lip cut, and
three topsocket regressions. The cylinder boundary integral uses actual arc
sweeps, including major arcs and stepped heights; its new measurement dispatch
is restricted to planar/cylindrical bodies. The imported fillet plate uses an
independent rounded-rectangle volume instead of the old chorded reference.
The historical weld allowance is retained at NURBS surface/curve vertices;
analytic-only vertices use the narrow merge band.

The cone/box census regression is corrected by using the actual merge tolerance
for line-refinement endpoint exclusion; its exact result passes independent
volume and two-deflection mesh checks. All four cone/sphere qualification tests
now pass after periodic-pocket clipping preserves exact chart intersections
when deduplicating nearby fitted samples. This removes the seam slit exposed
by the stricter CDT predicate.

Validation: 4,873 native tests passed, 13 skipped. One passing captured
halfsockets test reported a process-leak warning; its isolated rerun passed.
All-target/all-feature Clippy, formatting, layer boundaries, the deterministic
complexity guard, and the unchanged 52-row census pass. Both rebuilt WASM
packages pass smoke and installed-package consumer tests, each checking 240
exact tangency results and 84 typed refusals with direct/batch parity and
operand rollback. The cylinder integral derives sweep handedness from the
circle basis and normalizes wire traversal independently of face reversal;
existing non-line/circle measurement dispatch remains available. All 33
extrusion tests, the captured lip-band check, and all three revolve-orientation
tests pass with those contracts. Hosted checks and exact-head review remain
merge gates; these results do not claim general exact tangency or deployment.

Scheduled proof status checked during this qualification: Fuzz Smoke and Corpus
Gauntlet passed on `cfb5c29e`; [Mutation Testing run 34017122331](https://github.com/esaueng/remus/actions/runs/34017122331)
passed its unmutated baseline but exhausted the 150-minute budget with nine
missed mutants and incomplete coverage. The missed cases span healing totals,
sphere-loop area, fillets, loft bands, wire alignment, NURBS pole tessellation,
and benchmark surface helpers. They require separate qualification follow-up;
this tangency slice does not claim to clear that scheduled proof gate.


OpenZCAD consumer-roadmap K-S4 (`approx_census` CI enforcement): **done (PR
#140)**. Its authoritative disposition line remains in planning PR
esaueng/OpenZCAD#140 so the two repositories retain separate commit streams.

### H1 — now: three non-colliding lanes

1. **Geometry lane (P-Class M2 track — one session at a time in
   algo/pave-filler):** finish 2.0 (reader migration, boundary-authority
   flip), then 2.2 sphere-in-general-position, 2.3 Steinmetz, toward 2.4.
   Bridge items that ride this lane's files: B2, B7, B8 below.
2. **Infrastructure lane (Open Kernel Wave A — new dirs and io):** O1.1
   gauntlet, O1.3a fillet torture corpus, O1.4a validation properties,
   O4.1 facade, O4.4 error registry, O5.1 STEP assemblies, O3.1 benches,
   O6.1/O6.4 docs + contributing, O2.1a–b RFC 0006 + math substrate.
3. **Qualification lane (bridge backlog — bounded, evidence-heavy,
   disjoint):** B1 healing disclosure, B3 closed-rim chamfers, B4 v2
   trimmer items, B5 offset provenance, B6 evidence matrices, B10/B11
   small hygiene items.

### H2 — after P-Class 2.4 (the parallelization point)

P-Class M3 integration ∥ M4 ∥ 7.4+8.1 (per its §4), plus Open Kernel
Wave B (O2.1c–e variant ripple, O2.3 arrangement splitter, O3.2 spatial
cache, O4.2 publish dry-run, O1.2 head-to-head, O4.3 Python, O5.2 e3b,
O5.3a AP242, O6.2 playground). Bridge: B2 scale residuals close inside
2.6; B9 tangent-torus rides 2.7's tangency machinery.

### H3 — after M4 / M5

M5 blend depth ∥ M6 direct modeling ∥ M7 surfacing; O1.3b torture-suite
publication, O5.3b PMI read, O7 hybrid RFC. Bridge: the B12 residue
(loft-hole correspondence, holed partial revolutions) with M7's cap work.

### H4 — v1.0

**Definition of v1.0** (the first stable publish, O4.2c): P-Class exit
benchmarks **B1–B5** green as permanent tests + Open Kernel scoreboard
claims **S1–S7** live + zero Unsupported-untyped cells in the capability
matrix + the bridge backlog empty or explicitly re-triaged. Anything
short of all four publishes as 0.x.

### H5 — Core modeling parity (post-v1.0)

Prove parity with the reference kernel across the in-scope
authored-modeling domain. Gates (full text in
[industrial-parity.md](industrial-parity.md) §6): complete crosswalk with
no `Unknown` or unowned in-scope rows; zero Unsupported-untyped cells
re-verified; W1, W2, W5, W7 complete correctly (not by refusal) on both
surfaces at three scales; 2.4c/d, 2.5, 2.6, 2.7, 4.8 closed; blend
(M5 + 5.8), chamfer (B3), shell/offset (5.7b), draft (6.4), sweep (7.1),
loft (7.2), direct-edit (6.2, 6.3) matrices qualified with typed
both-sides boundaries; evolution complete for every covered family (B18,
6.5); stable Rust (O4.1c, O4.2a/b), JS/WASM (O4.4, O4.7), and Python
(O4.3a/b) surfaces; parity numbers published for the from-scratch
workflows with the O1.2f baseline pinned. Numeric bands are locked by
O1.2f, not before.

### H6 — Industrial interchange and corpus parity

Comparable outcomes on real supplier data, assemblies, and large models.
Gates: W3 passes with zero heal invocations on the tolerant path (3.4,
3.5, B1, B2 exit benchmark); AP242 structure, attributes, and the declared
PMI read profile qualified on CAx-IF models (O5.1, O5.2, O5.3a/b, O1.4b);
occurrence identity stable (O5.4); gauntlet stage pass rates within the
locked bands with the taxonomy public (O1.1c/d); zero silent-wrong,
crash, and unbounded-hang outcomes on the gated corpus; imported-model
boolean/blend/offset/direct-edit/tessellation/measurement stages within
bands; large-model memory and tail-latency budgets enforced (8.2, 8.6,
O3.2, O3.4; W8); deterministic concurrent sessions and supported parallel
operations (8.3, 8.4; 8.7 decided); a second external consumer with its
corpus in CI (O6.3, S6).

### H7 — Demonstrated technical leadership

At least five of the overlay's leadership claims LC1–LC13 meeting the
leadership discipline (stable corpus, equivalent quality, pinned
baseline, repeatable results, published losses, always-on gate), at
least two from the correctness family and one from the browser family.

Direct-edit follow-up under [P-Class 6.5](p-class-status.md):
[boundary-aware resizing of partial cylindrical faces](../roadmap/partial-cylinder-resize.md)
is implemented on its review branch: native and packaged WASM quarter-wall
resizing/refusal checks pass, and the full workspace passes 4,882 tests.
General partial walls and journaled direct-edit completion remain open.
The next slice retains exact edge/vertex correspondence for planar and
coaxial-bore face moves, with `moveFacesJournaled` direct/batch surfacing.
Native reference and connectivity regressions and both rebuilt-package suites
pass. The unchanged parent fails the new reference witness at the first edge.
Full workspace: 4,888 passed, 13 skipped. Clippy, rustdoc, and the unchanged
52-row approximation census pass. The journaled binding
now applies the same face-count and topology-work preflights as `moveFaces`;
regressions cover over-limit selections and a valid 500-sided prism. Both rebuilt WASM
package suites and the full native suite pass with these preflights. This does not close the
blend-boundary, radius-edit, replacement, draft, or delete/heal history cells.

Surface-replacement history is the next review slice: `replace_surface_journaled`
and direct/batch `replaceSurfaceJournaled` retain construction face, edge, and
vertex identities through planar and qualified coaxial-cylinder replacements.
Native tilt-then-bore composition and refusal tests pass. Rebuilt-package tests
preserve every reference through plane and imported quarter-wall sequences,
including arena save/restore with handle remapping, STEP geometry round trips,
and collision rollback. The binding preserves the established topology-work
limit. Final qualification: 4,894 workspace tests passed, 13 skipped; both
rebuilt WASM package suites, Clippy, rustdoc, and the 52-row census pass.

Blended-move boundary history is the next slice. Rigid translation retains the
actual copy maps; sharp-support reconstruction emits boundary history only when
construction face identities uniquely determine edges, vertices, and outer/hole
wire cycles. Ambiguous periodic boundaries retain explicit unresolved history.
The unchanged parent fails native and packaged reference witnesses. Successive
bored-cap moves, both construction paths, and the periodic ambiguity refusal
pass focused native tests. Final qualification: 4,896 workspace tests passed,
13 skipped; both rebuilt WASM package suites, Clippy, rustdoc, and the unchanged
52-row census pass.

Cylindrical radius history is the current review slice. The native
`resize_cylindrical_face_journaled` and direct/batch
`resizeCylindricalFaceJournaled` entries preserve qualified bore and quarter-wall
replacement maps. Boss edits compose Boolean and unification construction
history, preserving cap splits, generated subdivision boundaries, and explicit
deletions when a later edit consumes them. Unique oriented boundary
correspondence is required; ambiguous or inferred lineage stays unresolved.
Focused native tests pass for successive bore/boss edits, four rigid placements,
small/large scales, every original and result entity reference, and rollback.
A tangent bore is now refused because its new contact changes the adjacency
graph. STEP import now certifies a declared full-turn cylindrical p-curve
against the authoritative 3D circle traversal, preserving periodic endpoints
without accepting extra turns or mismatched curves. All 189 reader tests pass.
Both rebuilt WASM package suites pass the new direct/batch bore, boss, and
quarter-wall reference, arena, STEP, and rollback matrix. Final all-features
qualification: 4,903 passed, 13 skipped; two process-leak warnings in existing
facade tests did not reproduce in isolated checks. Clippy, rustdoc, mdBook,
crate boundaries, and the unchanged 52-row census pass.

Draft boundary history is the current qualification slice. The existing native
`draft_journaled` now records a complete, unique construction boundary map and
wraps geometry plus journal recording in one transaction. Plain and bored-box
sequences retain every original entity reference at three scales and two rigid
placements. Invalid and folding drafts restore state; a failed call also leaves
an existing unjournaled mutation gap unpublished until the next successful
operation. The additive `draftJournaled` JS API uses explicit degree units for
both direct and batch calls (`angleDegrees` in batch). Both rebuilt-package
suites pass reference retention, arena remapping, and rollback. STEP round trips
pass except the 10,000 mm bored-box cells: their 1,000 mm circular rim exceeds
the existing fixed importer sampling budget on both the unchanged source and
drafted result. Those refusals are covered explicitly without changing import
tolerances or resource limits. Final all-features workspace qualification passes
4,907 tests with 13 skipped and no process-leak warnings; the subsequently added
mutation-gap rollback regression also passes separately with all features.
Clippy, rustdoc, doc tests, mdBook, and the unchanged 52-row census pass.
Curved-face draft and ambiguous boundary correspondence remain separate open cells.

Defeature entity history is the next qualification slice. The existing native
`defeature_journaled` wrapper now records retained, merged, and deleted boundary
identities transactionally, with additive direct/batch `defeatureJournaled` JS
bindings. Capping carries actual copy maps; planar reconstruction carries vertex
source groups through collapsed corners and captures assembly allocations before
refinement. Closed plane/cylinder and cylinder/cone bands carry explicit contact
and seam replacement maps. Unqualified refinement or seam correspondence remains
unresolved. Native reference, analytic-volume, and rollback witnesses pass. The focused
packaged matrix passes 30 direct/batch cases, including STEP-imported sources,
arena restoration, and subsequent draft edits. Existing fillet/chamfer journal
anchors expose faces only; defeature output tests cover all entity kinds. The
10x cylinder/cone input is an exact scaled unit fixture: direct creation of that
fillet reproduces a wire self-intersection refusal on the unchanged parent and
remains a separate geometry issue. Both complete rebuilt-package suites pass. Final all-features workspace
qualification passes 4,914 tests with 13 skipped and no process-leak warnings.
Clippy, rustdoc, doc tests, mdBook, boundaries, and the unchanged 52-row census pass. General healing pipeline history remains open.

### Correctness follow-up: curved hole winding (#278)

Cut/intersect assembly now compares multi-opening cylinder wires in
seam-unwrapped UV and preserves reversed coedge p-curves. Periodic
same-wound holes are validation errors. The cross-drilled shaft regression
covers raw GFA and public booleans across scales and bore angles, STEP
round trips, and the WASM render/measure matrix.

This exposed a false success in the cross-drilled rim fillet: its convex
edge received added material, and the malformed input had suppressed the
volume-sign gate. The corrected input now receives a transactional refusal.
Correct-side curved rim assembly remains B4/M5 work; this case is not a
qualified blend success.

### Correctness follow-up: wide spherical caps (#285)

Exact circular rims now enable the shared latitude-cap tessellator without
requiring a second trimmed face on the same sphere. Rim traversal selects
the retained pole, including caps larger than a hemisphere. The primitive
polygon-equator path stays unchanged. `regress_wide_sphere_cap.rs` checks
small and large caps, the radius-9/cut-7.5 ball-stud case, scales, rigid
transforms, two deflections, manifold meshes, closed-form volume, standalone
face area, and STEP. The packaged WASM consumer replays the generated
wide-cap STEP fixtures through the translator and kernel, checking
volume and direct/batch mesh quality, including the explicit doubled pole seam. Equal-axis ellipse representations of
circular rims use the same verified path.
This qualification covers circular rims with or without one doubled pole
seam; arbitrary non-circular trims are not included.


### Healing entity history — qualification in progress

The next B18 slice adds native `fix_shape_journaled` and
`heal_pipeline_journaled`, plus direct/batch `fixShapeWithConfigJournaled`
and `runHealPipelineJournaled`. Both JS calls return the existing verified
repair report with an added journal `op`. Batch arguments are `solid` plus
`configJson` (a JSON string) or `steps` (an array of operator names).

Explicit `ReShape` replacements compose over each pipeline step's actual
input/output entity scope. Identity survives only while the same entity is
still reachable. Splits and converging replacements compose; explicit removal
records deletion. Missing targets, unrecorded replacements, and newly created
entities without construction records stay unresolved. Contradictory live-source
claims and replacement cycles refuse transactionally. Repair reports, both
validation gates, operator tolerances, and rollback remain authoritative.

This does not complete the healing defect matrix or all operator attribution.
`unify_same_domain`, `sew_shells`, `remove_internal_wires`, and
`split_common_vertex` do not currently expose complete replacement records;
their untracked changes remain unresolved. Geometry conversions preserve
surviving identities. `fix_shape` exposes its private repair context separately
so composing history does not change another operator's working tolerance.

Native witnesses cover merged vertices, multi-step repair, later-step rollback,
split/merge/delete composition, unknown history, and cycle refusal. Direct/batch
binding witnesses preserve all entity references across three scales, arena
restore into a populated kernel, and subsequent draft. The broad all-features
run passes 4,926 tests (13 skipped). A subsequent compatibility fix restores the
ordinary pipeline's original custom-operator dispatch; its 21 journal tests and
full-workspace Clippy pass. Both final optimized packages pass smoke and
installed-tarball consumer suites, including 12 damaged-box repair cells with
reported actions, eight repaired vertices, persistent references, arena/STEP,
and later draft. Final all-features affected qualification passes 113 tests. One process-leak
warning in `model_owns_context_topology_and_journal` does not reproduce in its
isolated check; the broad run has no leak warnings. This slice is not yet a
merged claim.

### Same-domain unification history — qualification in progress

The next B18 increment records actual source-face groups and ordered edge runs
inside the existing healing unifier. `unify_same_domain_with_history` is additive;
the ordinary API keeps its existing behavior and tolerances. Each rejected phase
discards its own records. A closed split-box witness proves that the edge phase
can revert while retaining the successful face merge and its history.

The healing pipeline now attributes single-output merge groups and committed
edge runs. Shared internal edges consumed by a face group are recorded as deleted
only when no result-shell use remains. Vertices consumed by those edges or merged
runs are deleted only when absent from the complete result solid. A four-triangle
box-face witness preserves all surviving references and explicitly deletes four
spokes and the unused center across three scales, populated arena restore,
repeated unification, and a later draft.

A follow-on B18 increment traces multiple output regions through source-wire
components and canceled shared edges. A pinched source can contribute to both
regions while its neighbors contribute only to their own region; fully interior
source faces inherit their connected region even without surviving boundary
edges. The collector allocates no topology and requires complete boundary and
component accounting. Pinched sources with holes remain unresolved until hole
ownership is established. Low-level open repair fixtures cover this tracing and
pipeline history; they do not qualify closed-solid acceptance. The unifier's
existing outer-shell scope is
unchanged; cavity unification and broader multi-region qualification remain
open B18 cells. The compiled-package regression reproduces `unresolvedAcrossOperation` on the preceding package;
both rebuilt optimized packages now pass smoke and installed-tarball consumer
suites, including all six direct/batch unification cells. Workspace Clippy,
rustdoc, doctests, mdBook, boundary checks, and the unchanged 52-row census pass.
The full all-features native run passes 4,932 tests (13 skipped), with no
process-leak warnings. The final test-fixture boundary API correction also passes
all 140 healing tests. The rebuilt kernel grows by 12,090 bytes; the translator
binary is unchanged. This increment is qualified locally, not yet merged.

### Sewing entity history — qualification in progress

The next B18 increment records the edge redirects and final vertex representatives
committed by shell sewing. `sew_shell_with_history` returns a `SewHistory` alongside
the existing counted report; the ordinary API keeps its geometry, tolerances,
curve-agreement checks, ambiguity refusals, and rollback. Records are produced
only after boundary and vertex updates succeed. The pipeline attributes consumed
sources only when they are absent from the complete result solid; sources still
used by another shell retain their live identity.

Native witnesses cover all 12 edge joins and 16 vertex joins of a disjoint-face
cube, ordinary-operation parity, repeated no-op sewing, and refusal without
replacement claims for mismatched curves or ambiguous partners. A shared-inner-
patch fixture tests retained-source identity without claiming closed-solid
qualification. A separate verified three-scale box witness preserves every
reference through sewing, populated arena restore, and later draft; a deliberately
failing later operator proves topology and journal rollback after sewing.

The direct/batch WASM regression uses explicit construction anchors in a legacy
arena fixture and reproduces `unresolvedAcrossOperation` on the preceding package.
Both rebuilt optimized packages pass smoke and installed-tarball consumer
suites, including all six direct/batch sewing cells. Workspace Clippy, rustdoc,
doctests, mdBook, boundary checks, and the unchanged 52-row census pass. The
kernel grows by 12,122 bytes; the translator binary is unchanged. The full
all-features native run passes 4,935 tests (13 skipped), with no leak warnings.
This increment is qualified locally, not yet merged. It does not extend sewing
to cavity shells or close the broader B17 repair matrix.

### Inner-wire removal history — qualification in progress

The next B18 increment records boundary entities consumed by
`remove_internal_wires`. The additive `remove_internal_wires_with_history` API
captures candidates from the actual removed wires and excludes edges and vertices
still reachable in any result shell. The journaled pipeline records the remaining
entities as deleted. Shared boundaries and surviving faces retain their identity;
the ordinary removal policy and cavity traversal are unchanged.

A three-scale open-hole repair witness becomes a verified closed cube with eight
explicitly deleted edge/vertex references and all 26 surviving references bound.
Arena restore, repeated removal, later draft, and pipeline rollback retain that
contract. Low-level tests cover cavity wires, shared edges and vertices, and
ordinary-operation parity. Removing loops from a valid through-bore remains a
typed refusal with unchanged topology and journal in six direct/batch scale cells.
The preceding compiled package reproduces unresolved consumed references. Both
rebuilt optimized packages pass smoke and installed-tarball consumer suites,
including all six inner-wire cells. Workspace Clippy, rustdoc, doctests, mdBook,
crate boundaries, and the unchanged 52-row census pass. The kernel grows by
11,512 bytes; the translator is unchanged. The full all-features native run passes
4,937 tests (13 skipped), with no leak warnings. This increment is qualified
locally, not yet merged.

## §B Bridge backlog — owned by neither program

Ready items from the stabilization-plan residue, the capability-matrix
sweep, and the deferred-work inventory (2026-08-29). Each row is claimable
by a bounded session; update state in-place. Items that map onto an
existing program issue are listed there instead — notably: Steinmetz = 
P-Class 2.3 · conic boolean cells = O2.2 · offset self-intersection = 5.7
· e3b = O5.2 · error registry = O4.4 · seam/p-curve round-trips = 2.0.

| ID | Item | Where | Size | Why it matters | State |
|---|---|---|---|---|---|
| B1 | **Healing disclosure typing** — the matrix's only named Unsupported-untyped cell: permissive healing can mask an invalid result as valid. Type every repair (report what changed, refuse to claim validity it didn't verify); both-sides tests. | `heal/src/fix/`, `check/src/validate/` | M | The last untyped silent-failure path in the kernel; highest correctness value per line. Do first in the qualification lane. | **Done (2026-09-03, PR #243):** fixer results enumerate counted repair kinds and typed declined repairs; L2 `OK` explicitly means only “no fixer action,” never validity. Operations, facade verified mode, configurable direct WASM, named pipelines, and additive detailed direct/batch WASM surfaces commit only after independent operations/check validation. Invalid and unverifiable results return stable typed refusals with attempted repairs and roll back. Native and WASM both-sides regressions pin verified success and refusal. |
| B2 | **Boolean scale residuals** — the through-tool family now returns exact material at 1e-5 and 1e6; straight-edge refinement and local planar bands are qualified by 72 operator/scale/placement cells. | `algo` bands | M | Feeds P-Class 2.6; remaining dimensional and curved-band work is tracked in [the audit](scale-band-audit.md). | Partial — named matrix passes; broader audit remains |
| B3 | **Closed-rim chamfers** — cone-frustum band mirroring the validated toroidal fillet assembler; closed-form volume oracle. Stabilization C1.2. | `blend`, `operations/src/chamfer.rs` | M | Exact surfaces, cheap, passes chase filter 1; unblocks resize_blend cylinder/cone (C2). | Open |
| B4 | **v2 walking-trimmer completion** — the four named gaps: keep-side hint, shared contact edges, end-cap notch trim, chamfer external-tangent branch. Stabilization C1.3. | `blend/src/trimmer.rs` | M | Critical path for v2 walker parity → legacy engine retirement (M5 precondition). | Open |
| B5 | **Offset face provenance** — offset derives faces 1:1 and discards the mapping; journal real evolution instead of a barrier. | `offset`, `operations/src/offset_v2.rs` | S | The last declared-barrier operation nobody owns; closes the B3-residual from stabilization. | **Done (2026-09-02, PR #224 (landed via #233)):** default intersection-joint V2 offsets retain and validate the total 1:1 construction map; native and direct/batch WASM journal wrappers record it transactionally. Closed-form plane/volume, persistent-reference, rollback, and WASM parity oracles pin the claim. Arc-joint and self-intersection-removal variants explicitly refuse this map because later face synthesis/replacement needs richer provenance. |
| B6 | **Evidence matrices, batched** — the "Stable-but-blocked" ledger rows that are pure test work: primitives invalid-input/scale/postconditions; plane-section cavity+degeneracy; measurement curved-cavity+scale; sweeps degenerate/cavity + nonconvergence budgets; convex hull/Minkowski degenerates. One qualify_*.rs per family, stabilization-plan pattern. | `operations/tests/` | M (S per family) | Flips ~8 Blocked ledger rows with zero new geometry; ideal bounded-session work. | **Partial — primitive family done (2026-09-03):** box, cylinder, pointed cone, frustum, sphere, torus, and ellipsoid are qualified across 1e-3/1/1e3 scale by closed-form volume/bounds, exact entity/surface censuses, dual validators, oriented closed B-Rep, watertight/manifold mesh, independent mesh-volume, determinism, and direct/batch WASM parity (`qualify_primitives.rs`, WASM `qualify_primitives_tests.rs`). The invalid matrix also closed non-finite box/sphere/torus acceptance. The ellipsoid follow-up repaired hemisphere selection, exact rational preservation, shared-equator pole-cap tessellation, and polar bounds; `ellipsoid-tessellation-scale.json` pins the permanent replay. Plane-section, measurement, sweeps, and convex hull/Minkowski families remain open. |
| B7 | **Pave-block attachment for marched FF curves on curved faces** — the named canonical fix for the cross-face boundary-desync family; three cheaper altitudes already failed. | `algo/pave_filler/make_blocks.rs`, `phase_ff.rs` | L | Deepest structural payoff in algo; root-causes a whole non-manifold family. Geometry lane, coordinate with M2; repro `replay_scplate.rs`. | Open |
| B8 | **Reversed NURBS sub-span convention** — forward spans shipped; reversed validated sub-spans blocked on the same arrangement defect as B7. | `topology/src/edge.rs` | M | Completes the endpoint-trimmed contract 2.0 builds on. | Open (after/with B7) |
| B9 | **Torus ∖ coaxial cylinder tangent cut** — the single cell keeping torus booleans Beta; needs a tangent-contact primitive (explicitly NOT the band splitter). | `math/analytic_intersection.rs`, `algo` splitter | M | B1-ledger promotion Beta→Stable; closed-form oracle exists. Rides 2.7 tangency machinery. | Open |
| B10 | **Curve-curve / curve-surface classification qualification** + conic distance/classification cells | `math`, `geometry/extrema`, matrix harness | M | Unqualified since the matrix was written; sits under many families' claims; pure evidence. | Open |
| B11 | **Small hygiene set** — `log::debug!` false-zero in `fill_images_faces.rs` (diagnostic-infra bug); deterministic STEP entity ordering; heal `fix_duplicate_faces` winding-blind comparison; plane×plane sampled in-both exact upgrade; `n_fine` clamp hazard note→guard. | various | S each | Cheap, each has already cost or will cost a debugging session. | **Partial (2026-09-03, PR #239):** STEP export now canonicalizes unordered face, void-shell, and hole-loop aggregates while preserving semantic coedge traversal order; byte-equality regressions cover reordered faces and void shells. Remaining: false-zero diagnostic, winding-aware duplicate-face healing, plane×plane exact upgrade, and `n_fine` guard. |
| B12 | **Holes on non-planar section caps** — annular Coons or cap-then-subtract vs extruded-annulus ground truth (stabilization B2.2). | `operations/src/cap.rs`, `fill_face.rs` | M | Largest remaining non-planar-cap value with clean ground truth. H3, with M7 cap work. | **Partial (2026-09-04, PR #252):** sweep and pipe caps preserve disjoint rectangular iso-parametric holes on four-sided bilinear caps, matched against an independently extruded annulus by converged volume, manifold B-Rep, watertight mesh, classification, and direct/batch WASM. Off-surface, curved, touching, and n-sided holed trims refuse typed; loft-hole correspondence and holed partial revolutions remain Unsupported-typed and ride M7's cap work. |
| B13 | **STEP inner-shell (voids) export** — emit and read `BREP_WITH_VOIDS`, preserving cavity shell count and volume. | `io/src/step/{writer,reader}.rs` | S–M | Round-trip honesty for hollow parts; gauntlet round-trip stage will hit it. | Complete (2026-09-04): one- and two-void regressions verify single-solid round trips, shell counts, and volume. |
| B15 | **Cut/intersect pocket-face orientation on cylinder walls** | `crates/operations/tests/regress_parallel_boss_band_sections.rs`, `algo` assembly | M | **Done (2026-09-04, PR #255):** non-fuse assembly normalizes selected cylinder outer/inner wire winding before edge merge. Box and cylinder tools on both wall sides pass exact cut/intersect, dual validation, closed-form volume, material classification, and welded-mesh orientation oracles. The formerly ignored regression is permanent coverage. | Done |
| B14 | **Render promotion track** — Experimental→Beta after a contract-stable release cycle (stabilization C4 residue); outside both programs. | `render` | S (time-gated) | Cleans the last stabilization row. | Open |
| B16 | **Consumer topology-query API set** — one binding per OpenZCAD heuristic it currently reimplements (its roadmap C2): trimmed edge parameter domain, face material sense, ordered wire traversal, per-edge convexity, sphere-patch identity, seam-edge parity, `maxFilletRadius(solid, edges)`, batched `classifyPoint`, per-edge ids in `meshEdgesAll`; plus the GCS qualification matrix (constraint type × system state × scale, nonconvergence budget) from the P-Class §6 inherited queue. Each: exact, typed refusal on foreign handles, direct + batch WASM, contract test. Added 2026-09-04 by the [industrial-parity overlay](industrial-parity.md) (rows IP-15.9, IP-9.3, IP-10.3, IP-13.1/13.4). | `wasm/bindings/query.rs`, `batch.rs`, `operations/src/query.rs`, `sketch/`, `operations/tests/qualify_gcs.rs` (new) | S each | Every row retires an adapter-side heuristic; highest OpenZCAD impact per line. Exit: the named heuristic deleted from the consumer's adapter (recorded in the PR), matrix green. | Open |
| B17 | **Healing defect-class qualification matrix** — a generated defect class × severity × repair policy × scale matrix per fixer (wire order/closure/gaps/small edges, face orientation/small faces, seams, shell orientation/sewing/free bounds, duplicates, continuity splits, representation conversion), plus an operand self-interference report for booleans and the faceted-import sew/unify contract (issue #244). Every cell: verified repair with counted disclosure, or typed refusal; both sides. Added 2026-09-04 by the overlay (rows IP-8.3, IP-3.8, IP-8.6). | `heal/`, `operations/src/heal.rs`, `operations/tests/qualify_heal.rs` (new), `stl/import.rs` | M (S per fixer) | The family is Qualified only at the B1 boundary; the reference kernel's healing breadth is its strongest documented area. Exit: every fixer has a matrix; #244 fixture green; self-interference report typed on a self-touching corpus. | Open |
| B18 | **Evolution completeness audit** — every topology-producing family reports total attribution or a typed unresolved record: unify same-domain (`unify_with_evolution`, OpenZCAD C1's top ask), sew, sweep/loft/revolve/extrude caps, arc-joint and self-intersection-removal offsets, section/split edges, direct edits (with 6.5), edge/vertex events beyond booleans. Added 2026-09-04 by the overlay (rows IP-3.6, IP-5.7, IP-12.1; leadership claim LC3). | `journal_ops.rs`, `evolution.rs`, `qualify_evolution_coverage.rs`, per-op modules | M (S per family) | Absolute gate §3.4 item 8; unblocks OpenZCAD's adoption order boolean → pattern → chamfer → shell/offset → direct edits. Exit: the coverage fixture claims every result face of every family exactly once or pins its typed unresolved; no `record_barrier_over_solid` call remains for a family that can construct its map. | Open |
| B19 | **Remaining fuzz slices and mutation scope** — curve-intersection, offset, GCS, and tessellation fuzz targets with independent oracles on the weekly schedule. The mutation-scope slice moves the previously undiscovered root config to `.cargo/mutants.toml` and selects the current CDT directory. | `fuzz/fuzz_targets/`, `.cargo/mutants.toml`, `.github/workflows/fuzz.yml` | S each | Exit: four targets scheduled with committed seeds; mutants report shows CDT mutants examined. Scope regression checks reject ignored config and the stale CDT file glob; the bounded five-mutant sample caught four and retained one survivor for review. | Partial — mutation scope verified; four fuzz slices remain open. Evidence: `scripts/test-mutants-scope.py`, `docs/kernel-maturity/testing-strategy.md` |
| B20 | **Exact measurement completion (K-S2 remainder)** — ellipse, hyperbola, and NURBS planar boundaries; general curved-face area; deflection-independent curved-body volume, centroid, and inertia by Gauss quadrature over exact geometry with a stated bound; direct + batch WASM; scale matrix. Added 2026-09-04 by the overlay (row IP-10.1). | `check/src/properties/`, `operations/src/measure/` | M | OpenZCAD S2 measures 0.2–3.5 % volume error on filleted parts at its display deflection; the reference kernel integrates surfaces directly. Exit: relative error ≤ 1e-6 against closed forms on filleted and cavity primitives at 1e-3/1/1e3, independent of caller deflection; ledger row loses its "incomplete" caveat. | Open |

**Explicitly not queued** (decided or terminal — do not re-open without
the named primitive): IGES growth (C3, decided), box∪sphere and torus∩box
census rows (TERMINAL → O2.3 re-opens them properly), universal
duplicate-edge merge key (proven unbuildable), mesh co-refinement
watertightness (below the chase filter until a live case routes there),
kumiko lattice family (probe only per the roadmap skill's engine-side
question), v1-fillet API migration (product decision, owner's),
monolithic mechanical-feature operators and tessellated-STEP read
(overlay §1: composable from imprint/boolean, and no corpus pull),
non-manifold shared-face topology (RFC 0005 later RFC; owner decision).

## §D External roadmap dispositions

- **K-W3 distributed WASM budget — partial
  ([PR #174](https://github.com/esaueng/remus/pull/174), 2026-08-31):** every
  consumer package workflow now deterministically optimizes the distributed
  bundler binary, validation fails above the 8 MiB OpenZCAD ceiling, and the
  PR size report compares committed distribution artifacts. Current `main`
  falls from 8,773,687 to 7,724,098 bytes, leaving 664,510 bytes of headroom.
  OpenZCAD cold-load timing on target hardware remains the product-side W3
  follow-up.
- **K-S2 exact measurement — partial (PR #151):** production `faceArea` and
  `surfaceArea` now reuse the exact planar boundary-moment integrator for
  line/circle/parabola wires, including circular holes, with scale,
  deflection-independence, direct-WASM, and batch-WASM oracles. Exact ellipse,
  hyperbola, and NURBS planar boundaries, general curved-face area, and
  deflection-independent curved-body volume remain.
- **K-S3 SSI Newton budget — done
  ([PR #147](https://github.com/esaueng/remus/pull/147), 2026-08-30):**
  `WorkBudgets::newton_iterations` is authoritative across NURBS×NURBS seed,
  branch, and march refinement; cancellation is polled inside the coupled
  Newton loop and propagates through the existing typed, transactional WASM
  boolean contract. Per R8 the cap is JS-callable: an additive optional
  `newton_iterations` argument on `booleanWithQuality` /
  `booleanWithCancellation` and a `newtonIterations` field on the
  `executeBatch` `booleanWithQuality` op, validated (non-negative integer
  within the public work budget) with contract tests on the default,
  bounded, and rejection paths. Default behavior remains the historical 20
  iterations. Its then-remaining subdivision slice is closed immediately
  below; parameter-space budgeting remains queued under P-Class 2.8.
- **K-S3 SSI subdivision budget — done
  ([PR #160](https://github.com/esaueng/remus/pull/160), 2026-08-30):**
  `WorkBudgets::subdivision_depth` replaces the seed finder's hard-coded
  recursion depth and is authoritative before every recursive Bezier-patch
  split. The default depth 6 reproduces prior behavior; depth 0 performs no
  recursive split. Direct `booleanWithQuality` / `booleanWithCancellation`
  expose additive `subdivision_depth`, and batch `booleanWithQuality` exposes
  `subdivisionDepth`, with shared validation and default/boundary/rejection
  contract tests. Parameter-space budgeting and wider operation-family
  adoption remain under P-Class 2.8.
- **K-S3 SSI marcher-budget WASM surface — done ([PR #202](https://github.com/esaueng/remus/pull/202),
  2026-09-02):** the existing `march_steps`, `queue_size`, `segments`, and
  `branches_per_direction` caps are additive optional arguments on direct
  `booleanWithQuality` / `booleanWithCancellation` and matching camelCase
  fields on batch `booleanWithQuality`. Shared bounded-integer validation,
  legacy-default equivalence, generated-WASM smoke coverage, context-authority
  tests, and a batch rejection/rollback volume oracle pin the contract.
  Parameter-space tolerance and wider operation-family adoption remain under
  P-Class 2.8, so that parent item stays partial.
- **K-S1 pattern overlap — done (PR #142, 2026-08-30):** linear,
  circular, and grid patterns now refuse measured material overlap with the
  typed `pattern_instances_overlap` contract and full rollback across native,
  direct WASM, and `executeBatchV2` repro coverage. Touching and disjoint
  instances remain supported across a 1e-3/1/1e3 scale sweep. Exact instance
  fusing is intentionally deferred until the separately queued
  pattern-through-fuse provenance work can make its evolution claims truthful.

## §S Session playbook

Match session type to lane; check both ledgers and `gh pr list` first.

- **Geometry-hard session** (budget for multi-pass debugging): H1 lane 1
  in P-Class order, or B7 if M2 files are contended. Never two sessions
  in `algo/pave_filler` at once.
- **Bounded/short session:** one B-row (B5, B11, B13, or one B6 family),
  or an inherited-queue item from P-Class §6.
- **Infrastructure session:** next unclaimed Wave A row in
  [open-kernel-status.md](open-kernel-status.md).
- **Evidence session** (test-writing capacity): B6 families, B10, O1.3a.
- **Docs/ecosystem session:** O6 rows.
- **Consumer-impact session:** B16 (one query binding), O4.7, B20, or
  B18's unify item — each retires a named OpenZCAD adapter heuristic.
- **Parity-evidence session:** O1.2d–e (Remus-only rows first), O1.5,
  B19; see the overlay's §8 ranking.
- **Owner-only:** O4.2c/O4.3c publishes, O6.2 hosting, O6.3 outreach,
  v1-fillet migration decision.

Maintenance rule: any PR that changes an item's state updates its row
here (or its program ledger) in the same PR — same discipline as the
skill's living-document mandate.
