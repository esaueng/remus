# P-Class Program

The roadmap from a capable solids kernel to a Parasolid-class one: three
architectural pillars, seven milestones, forty-odd issues, every one with a
typed exit gate.

- **Drafted:** 2026-08-28 (rev 2), baseline `main` @ `748e408b`.
- **Predecessors:** the P0 backlog (Issues 1–14, complete), RFC 0003
  (complete). RFC 0002 is partially open — see Issue 2.0.
- **Promotion authority** remains
  [capability-matrix.md](capability-matrix.md); this program plans the work,
  it does not promote labels.

## §0 What "Parasolid-class" means here

Parasolid is roughly forty person-years deep in blending alone; feature-count
parity is not a rational target. What *is* reachable — and what actually lets
a CAD product ship on a kernel — is four properties. They are the program's
definition of done:

| | Property | Meaning |
|---|---|---|
| **D1** | Imports work | Imperfect imported bodies participate in booleans, blends, and offsets directly — tolerant modeling, not heal-first. |
| **D2** | General booleans | Two arbitrary curved bodies union, cut, and intersect — exact where a closed form exists, honest NURBS seams where not. |
| **D3** | Real edits | Fillets survive curved neighborhoods; faces move, resize, and delete with neighbor re-limitation — with persistent naming intact. |
| **D4** | Honest everywhere | Everything outside the qualified domain refuses with a stable code. No silent fallback, no scale-dependent wrong answers. |

## §1 Starting position

**Breadth is done.** Every Parasolid operation family exists: booleans,
blends, sweeps, shell/offset/draft, sectioning, healing, mass properties,
assemblies, feature recognition, defeaturing, STEP plus five mesh formats, a
GCS sketch solver, WASM bindings. The
[capability matrix](capability-matrix.md) and the
[stability ledger](../production-readiness/stability-matrix.md) govern what
each family may claim.

**One area is ahead of the pack:** the RFC 0002/0003 stack — evolution
journal, persistent naming with typed resolution, attribute propagation,
serialization — is machinery most kernels never grew. It makes milestone M6
(direct modeling) unusually valuable here.

**Three pillars are missing**, and they are architectural, not features:
per-entity tolerant modeling (M3), sheet/wire/cellular body taxonomy (M4),
and general curved×curved boolean intersection (M2). Issue 2.2 closes the
first general-position cell: two offset spheres now fuse, cut, and intersect
through an exact radical-plane circle with analytic spherical result faces.
General quadric and NURBS pairs remain the honest D2 boundary.

**Ordering principle: architecture before generality, generality before
polish.** Tolerant modeling and body taxonomy change data structures every
later feature touches; retrofitting them under a mature blend engine is how
kernels calcify.

## §2 Program at a glance

| ID | Milestone | Size | Depends on |
|---|---|---|---|
| M2 | General curved booleans — RFC 0002 completion → sphere → quadrics → NURBS×NURBS; scale bands; budgets & cancellation | L | — |
| M3 | Tolerant modeling — RFC 0004; per-entity tolerance through predicates, GFA, sew, import | L | RFC ∥ M2 · integration after 2.4 |
| M4 | Body taxonomy: sheet, wire, cellular — RFC 0005; sheet booleans, imprint, multi-region results | L | M2 |
| M5 | Blend depth — variable radius, curved supports, vertex blends, setbacks, overflow | L | M2, M3 |
| M6 | Direct modeling — replace-surface with re-limitation; move/delete face; curved draft; journaled edits | L | M2, M7.4 |
| M7 | Sweep, surfacing & interrogation — guide rails, loft continuity, constrained fill, surface extension, clash/silhouette/curvature | M | 7.4 early (feeds M6) |
| M8 | Industrialization — differential testing, perf gates, parallelism, real-model corpus | L | 8.1 starts after 2.4 |

Size scale: **S** = one PR · **M** = 2–4 PRs · **L** = 5+ PRs, staged.
Milestone IDs continue the original program's numbering (the P0 backlog is
retro-labeled M0/M1).

## §3 Standing rules

Carried over from the P0 program and the repo's doctrine; they bind every
issue below.

1. **RFC before architecture.** M3 and M4 each open with a design RFC (0004,
   0005) merged before implementation, staged like RFC 0002/0003 — with
   characterization tests that must flip at the stage that changes them.
2. **Fail closed, typed, pinned.** Every capability boundary is a stable
   diagnostic code with a both-sides test. A silent wrong answer is a P0
   defect; a silent fallback is a contract defect.
3. **Oracle-verified or it didn't happen.** Every geometric exit gate names
   its ground truth: closed-form volume, inclusion–exclusion identity,
   mesh-boolean cross-check within deflection bound, or round-trip byte
   identity. "Looks right" and "validates" are not oracles — the scale-gap
   solid passed every validator with volume 43% high.
4. **Census on every boolean-adjacent change.** `approx_census` diffed
   row-by-row against baseline; fallback-row movement (either direction) must
   be explained in the PR.
5. **Capability-matrix bookkeeping in the same PR.** Cells move state
   (Unqualified → Qualified / Partial / Unsupported-typed) in the change that
   earns it; stability-ledger rows update with label changes.
6. **Concurrency hygiene.** `gh pr list --state open` before starting any
   issue; issues are scoped to disjoint file sets where possible so parallel
   sessions don't collide. Merge gate = per-job CI conclusions for the exact
   head SHA.
7. **Minimal diff, evolution always.** New operations journal real evolution
   events or an explicit barrier from day one — never silence. Public-API
   changes flagged prominently.
8. **Not done until JS can call it.** Every capability a milestone delivers
   ships its WASM binding, `executeBatch` companion, and contract tests
   *inside that milestone* — never as trailing cleanup. The browser consumers
   are the product; a native-only feature is half a feature.

## M2 — General curved booleans (L)

Every downstream family — blends, offsets, direct edits, sheet trims —
consumes boolean-grade surface intersections internally. This milestone is
the load-bearing wall. Strategy: exact arms only where closed forms exist
(cheap, permanent); everywhere else, honest NURBS section curves with
splitter and classifier robustness to carry them.

### 2.0 RFC 0002 completion: trims & p-curves under the new load (M)

`crates/topology/src/edge.rs` · `crates/topology/src/coedge.rs` ·
`crates/algo/src/builder/` ·
[rfc-0002-coedge-architecture.md](../design/rfc-0002-coedge-architecture.md)

The consciously staged 0002 deferrals come due before 2.5 and M3 build on
them: the measured missing-writer ratchet is closed and the staged 132-site
`domain_with_endpoints` production-reader migration has reached zero; physical
p-curve storage in Coedge (the boundary-authority flip) waits on sanctioned
mutation. NURBS booleans and tolerant edges on top of
reconstruct-by-projection is building on sand — the mixed-trim bug class from
PR #24 is what that sand does. Stage the reader migration mechanically; flip
the authority behind the existing characterization tests.

> **Exit gate:** every GFA result-assembly path writes explicit trims; the
> SameParameter/SameRange validators run over boolean outputs in CI;
> reader-site count at zero (tracked by grep gate, not by hand).

Issue 2.0c meets its part of that gate with an exact identity-intersection
fixture whose copied cylinder result retains both oriented seam pcurves. CI
requires six boundary uses visited, two pcurves stored, and two uses proved by
both strict validators, alongside the analytic face census and closed-form
volume. Corrupting only the reverse branch must fail while the forward branch
remains valid. The test does not synthesize pcurves after the boolean, and it
does not treat a zero-pcurve result as coverage.

Issue 2.0d closes the mutation prerequisite without advancing the physical
authority flip. The immutable survey's 30 production `wire_mut`,
`inner_wires_mut`, and `set_outer_wire` sites are migrated to
`Topology::replace_boundary_wire` or `Topology::set_face_boundary_wires`, and
the ratchet requires the direct-site count to stay at zero. Both sanctioned
paths preflight the complete replacement, prune stale oriented pcurve uses,
and re-derive an existing Loop/Coedge view in the same commit. A checkpoint
regression pins exact rollback of the old boundary, pcurves, and old handles,
plus retirement of handles allocated by the failed transaction. Wires remain
authoritative until Issue 2.0e.

Issue 2.0e completes that flip. Every valid `Topology::add_face` stores an
outer-then-inner Loop sequence on the Face; each Loop stores ordered Coedge
identities, and pcurves plus `(u, v)` periodic winding counts live on the
Coedge. Wire fields and the historical `(edge, face, orientation)` map remain
compatibility views kept coherent by topology-owned APIs. Direct coedge access
is authoritative; the pair accessor still fails closed on seams. Arena schema
v3 serializes the Loop/Coedge graph and embedded per-use data, verifies it
against the wire facade before commit, and preserves v1/v2 by deriving
authority on import.
The acceptance fixture round-trips a cylinder seam's two independent pcurve
branches and lifted winding count, while a tampered boundary rolls back
without changing live topology.

Issue 2.0f closes the STEP exchange part of the authority flip. Import binds
each `SURFACE_CURVE`/`PCURVE` branch to the exact Loop/Coedge position that
uses its `EDGE_CURVE`; repeated seam uses therefore retain independent 2D
ranges and periodic winding. Every matching branch must be consumed exactly
once, and endpoint mismatch or malformed association rolls the whole import
back. Export walks physical loops, emits parameter-trimmed per-use pcurves in
deterministic order, and refuses pcurve/winding disagreement. The acceptance
matrix pins two lifted cylinder-seam branches and the external 48-pcurve
analytic fillet corpus through byte-identical write/read/write cycles. General
SameParameter proofs for imported plane/conic combinations remain a typed
capability boundary for the 2.0g integration tranche; those combinations are
not treated as proved merely because their STEP pcurves were retained.

Issue 2.0g closes the integration tranche. The semantic ratchet requires zero
of the 132 measured production endpoint-reconstruction readers, all 24 trim-
writer preservation identities, zero of the 30 direct boundary mutations,
and no unknown sites. `validate_boundary_authority` audits every live physical
Loop/Coedge record for connected and synchronized boundaries, exactly-one
ownership, resolvable compatibility indexing, and complete seam branches.
The gate runs over the exact boolean seam fixture, arena-v3 round-trip and
rollback tests, the external 48-pcurve STEP corpus, and a WASM-visible boolean
path. General imported plane/conic SameParameter combinations remain typed
`same_parameter_proof_unavailable`; corpus retention is not promoted into a
false geometric proof. The unsafe direct Face wire mutators are deprecated.
The read-only wire facade is retained because its separately specified
consumer/release deletion gate is not met; removing it here would be a public
API break, not Issue 2.0 completion.

### 2.1 Honest-failure hygiene (S)

`crates/algo/src/pave_filler/phase_ff.rs` ·
`crates/algo/src/builder/pcurve_compute.rs`

Kill the two silent failure modes: the `_ => Ok(Vec::new())`
unsupported-pair arm in `compute_raw_curves` becomes a typed refusal, and the
`project_point(..).unwrap_or((0.0, 0.0))` UV-projection fallback becomes an
error. Cheap, and it turns "mysteriously wrong shape" into "typed error
naming the gap" for every later issue.

> **Exit gate:** new pinned diagnostic codes for both paths; census
> byte-identical to baseline; no arm in phase FF can return empty sections
> without a typed reason.

Issue 2.1 is delivered in two source-ordered tranches. PR #129 replaced the
unsupported phase-FF pair's empty result with
`unsupported_surface_pair`. PR #194 makes pcurve construction and its face-
splitter callers fallible, reports `pcurve_projection_failed`, and calls the
NURBS projector directly so its convergence failure cannot be laundered into
the compatibility midpoint. Plane-frame and analytic projections retain their
established parameterization, and the normalized approximation census must
remain byte-identical.

### 2.2 Sphere in general position (M)

`crates/math/src/analytic_intersection.rs` ·
`crates/algo/src/builder/face_splitter/` · `crates/check/src/classify/`

`exact_sphere_sphere` supplies the closed-form section circle and the sphere
splitter carves each source face along that non-equatorial circle. The
zero-area/winding doctrine from the NURBS-classification stack transfers
directly: a small circle on a sphere *splits* the surface rather than bounding
it, and winding carries which half survives.

> **Exit gate:** the pinned refusal
> `non_concentric_spheres_fuse_fails_closed_without_shortcut` flips to an
> exact-volume test (inclusion–exclusion oracle) for fuse, cut, and
> intersect.

Delivered: phase FF emits the exact radical-plane circle, the sphere splitter
turns its seam-split arcs into two winding-correct patches, and classification
uses the two analytic support-plane half-spaces rather than a chord polygon.
The spherical patch tessellator uses a one-to-one stereographic chart with a
constrained interior grid and verifies every boundary segment before emission.
Offset equal-radius fuse/cut/intersect results retain four analytic sphere
faces, validate as closed solids, classify material probes correctly, and
match independent lens/inclusion–exclusion volumes and manifold meshes at
three deflections. The lower-level radical-plane oracle separately covers
unequal radii, and an oblique-center fixture prevents axis-aligned special
cases from satisfying the gate.

### 2.3 Steinmetz ellipses (S)

`crates/math/src/analytic_intersection.rs` (`algebraic_cylinder_cylinder`)

Equal-radius perpendicular cylinder×cylinder: the seam degenerates to two
planar ellipses with no singularity — both types already exist, and the
marcher currently 128-samples and interpolates what has a closed form. Add
the degenerate equal-radius arm. The best-value exact chase on record.

> **Exit gate:** cyl ∩ cyl = 16/3·r³ exact; the census fallback row (70
> planar faces) becomes an exact analytic result.

Delivered: phase FF consumes the existing closed-form cylinder×cylinder
oracle and emits the two planar ellipses as eight authoritative quarter arcs,
split at both shared pinch points and at each half-span midpoint. The periodic
cylinder splitter promotes the seam-free arc loops into six winding-correct
cylinder patches, while fuse and cut retain their exact surfaces as eight- and
seven-face results. Intersection validates as a closed solid, exposes only
analytic cylinder faces and line/ellipse edges, matches `16/3·r³` at radii 2,
3, and 5 after rigid motion, and tessellates closed and manifold at three
deflections. The batch/WASM exact-only path pins the same six-face result. The
census now reports all three perpendicular-cylinder operators as exact; its
intersection row moves from 70 planar fallback faces to six analytic faces.

### 2.4 Quadric × quadric transversal, NURBS seams (L)

`crates/algo/src/builder/face_splitter/` · `crates/algo/src/classifier/` ·
`crates/algo/src/builder/fill_images_faces.rs`

The general non-coaxial cases — sphere×cylinder, cone×sphere,
torus×anything — have genuinely quartic section curves; NURBS seams are the
*correct* exact-B-Rep answer. The math marcher and the first seam consumers
already exist: winding NURBS chains can split cylinder/cone bands, sphere
hemispheres have a seam arrangement, and a torus notch has a bounded
arrangement arm. The remaining work is to make those consumers
operator-neutral, extend them across general quadric section topology, and
give mixed sphere/torus result solids an analytic classifier instead of the
ray-cast path documented to mis-count on doubly-curved faces.

Measured at `eca4fd4569f8e98e757b212782bd59d50b6d768e`, the 51-row census has
exactly three boolean fallbacks: box ∪ sphere (1192 planar faces),
perpendicular equal-radius cylinder ∩ cylinder (70), and torus ∩ box (312).
Issue 2.3 owns the cylinder row. Issue 2.4 executes in independently
reviewable stages:

1. **2.4a — sphere multi-region arrangement.** Emit every bounded cell and
   closed cap from a hemisphere seam arrangement, close box ∪ sphere as a
   16-face analytic B-Rep, and promote the two stale `sphere_box_partial_*`
   parity expectations.
2. **2.4b — torus complement selection.** Reuse the shipped torus-notch
   arrangement for the complementary Intersect region; close torus ∩ box with
   toroidal faces retained and a mesh-volume oracle.
3. **2.4c — general quartic seams and classification.** Exercise marched
   `NurbsCurve` sections on sphere×cylinder, cone×sphere, and torus pairs;
   extend the arrangement and mixed analytic classifier only where those
   pinned witnesses require it.
4. **2.4d — integration ratchet.** Merge the preceding heads, rerun the full
   operator census and parity matrix, and reconcile the capability/stability
   ledgers without weakening typed refusal or fallback disclosure.

Stage 2.4b is in review in PR #207. The torus-notch arrangement emits both
complementary annular `u`-bands with distinct interior witnesses and
winding-correct outer/inner roles. Operator-neutral classification retains the
long band for Cut and the short band for Intersect. The latter is a five-face
analytic result (one torus and four planes), passes strict shell validation,
and matches an independently co-refined mesh-volume oracle within 1%. Native
and batch/WASM exact-only paths pin the same result, and the census row moves
from 312 planar fallback faces to five analytic faces.

> **Exit gate:** together with Issue 2.3, the three measured census fallback
> rows become true B-Rep results with analytic faces preserved and volumes
> verified against independent or mesh oracles within their stated bounds.
> The `sphere_box_partial_*` parity gaps close, the three quartic witnesses
> above run exact-or-typed-refusal without unbounded work, and every stage is
> represented in the final integration census.

### 2.5 NURBS × NURBS booleans (L)

`crates/algo/src/pave_filler/phase_ff.rs` · `crates/algo/src/builder/` ·
`crates/math/src/nurbs/intersection/`

The imported-body ∪ imported-body case. The math-layer SSI (subdivision
seeding + adaptive marching with branch/tangency detection) exists and is
tested; boolean-level coverage is zero. Needs periodic-NURBS face splitting
(extend the band/seam machinery past analytic surfaces) and hard iteration
budgets threaded from `OperationContext`. The differential oracle is the
technique that closed the classification stack: solids run through
`convert_solid_to_bspline` must produce booleans matching their analytic
twins.

> **Exit gate:** B-spline-converted box/cylinder/sphere pairs match analytic
> boolean volumes across fuse/cut/intersect; two real imported STEP NURBS
> bodies fuse watertight, manifold, journal-complete. First NURBS×NURBS rows
> enter the census.

### 2.6 Scale-relative band audit (M)

`crates/algo/` (all absolute-constant bands) ·
`crates/operations/src/boolean/mod.rs`

Finish what the junction-band fixes started, systematically: sweep every
absolute snap/weld/acceptance band in crates/algo and make each
scale-relative (face-pair AABB or model diagonal), the same treatment the
junction-snap band got. Known residuals: the 100·tol weld bands that keep
1e-5 failing, and the silent 1e6 GFA case currently caught only by the
operations bounds gate.

> **Exit gate:** `boolean_scale_gap.rs` exact from 1e-5 to 1e6 — or a typed
> refusal; never a silent wrong volume at any scale. The rollback fixture in
> `boolean_context_authority.rs` becomes a correctness test.

### 2.7 Tangency & sliver contacts (M, stretch)

`crates/algo/src/pave_filler/` · `crates/math/src/intersect.rs`

The two stability-ledger caveats on the boolean row: exact tangency (the
union's pinch vertex is never built) and sliver crossings (~1e-5–0.05 mm on
r = 10) fall to the approximate path. The qualified-intersection model
(contact kind × quality) already classifies tangency; the construction side
doesn't consume it. Defer if M3/M4 pressure demands — the fallback is
disclosed, not silent.

> **Exit gate:** tangent-cylinder and sliver corpus bundles answer exactly or
> refuse typed; ledger caveats removed or re-typed.

### 2.8 OperationContext completion: budgets & cancellation (M)

— partial: boolean/NURBS-SSI cooperative cancellation is typed,
transactional, and WASM-bound; the coupled SSI Newton loop now consumes the
caller's iteration budget and cancellation token, SSI seed subdivision
consumes the caller's recursion-depth cap, and direct/batch WASM quality
booleans expose the existing march, queue, segment, and branch-exploration
caps. Parameter-space tolerance plus wider operation-family adoption remain
([PR #138](https://github.com/esaueng/remus/pull/138) +
[PR #147](https://github.com/esaueng/remus/pull/147) +
[PR #160](https://github.com/esaueng/remus/pull/160) +
[PR #202](https://github.com/esaueng/remus/pull/202)).

`crates/math/src/context.rs` · `crates/algo/src/pave_filler/` ·
`crates/math/src/nurbs/intersection/` · `crates/wasm/src/bindings/`

The unfinished half of the RFC 0001 migration queue: pave-filler budgets,
param-space tolerance, and *cooperative cancellation* (fallback policy
landed; the rest never did — and the newer GFA entry points hardcoding
`Tolerance::default()` get fixed here too). Cancellation is user-facing for
the browser consumers: a long NURBS boolean with no way out kills
interactivity in a way no geometry feature compensates. Cancel checks at
phase boundaries and marcher iterations; a cancelled op is a typed result,
never a torn arena — the transaction machinery makes this cheap.

> **Exit gate:** a deliberately pathological NURBS×NURBS boolean cancels
> within a bounded latency, leaving the topology untouched; every iteration
> loop in algo/math answers to a context budget; WASM exposes cancellation
> tokens with contract tests.

## M3 — Tolerant modeling (L, RFC 0004)

Parasolid's defining feature and the single biggest lever for "imported
bodies just work." Every edge and vertex carries its own tolerance — a
vertex is a ball, an edge is a tube — so gappy real-world geometry
participates in modeling directly instead of being healed to exactness first
(the pre-Parasolid architecture Remus has today). The RFC is written in
parallel with M2; integration lands after 2.4 so the two don't churn the pave
filler simultaneously.

### 3.1 RFC 0004: per-entity tolerance semantics (M)

[`docs/design/rfc-0004-tolerant-modeling.md`](../design/rfc-0004-tolerant-modeling.md)

The design decisions that must be made once, on paper: containment semantics
(vertex ball ⊇ all incident edge ends; edge tube ⊇ its curve within
tolerance), the authority rule when tolerances disagree
(max-of-contributors), the growth discipline (operations may *raise* a
tolerance, never silently, with a per-context cap from `OperationContext`),
interaction with the SameParameter/SameRange validators from the Issue-8 trim
work, serialization (additive arena fields, absent-when-default), and WASM
disclosure. Staged like RFC 0002 with characterization tests pinned per
stage.

> **Exit gate:** RFC merged with staged plan; characterization tests for
> current single-tolerance behavior written and passing (they flip during
> predicate plumbing).

Delivered in [PR #126](https://github.com/esaueng/remus/pull/126): the RFC
records containment, max-of-contributors authority, capped and journaled
growth, additive serialization, predicate staging, and downstream disclosure.

### 3.2 Topology substrate (M)

`crates/topology/src/vertex.rs` · `crates/topology/src/edge.rs` ·
`crates/topology/src/validation.rs` · `crates/io/src/arena_io.rs`

Tolerance fields on Vertex and Edge with validated setters (a tolerance must
actually cover the gap it claims to cover — checked, not asserted),
validation checks, additive arena-format serialization, journal integration
(tolerance changes are recordable events).

> **Exit gate:** round-trip byte-stability for legacy documents; validators
> reject a tolerance smaller than the measured deviation it papers over.

Delivered in [PR #148](https://github.com/esaueng/remus/pull/148): validated
vertex/edge setters, ball/tube validators, the operation-context growth cap,
journal recordability, and tolerance-bearing arena round-trip coverage landed
without changing the legacy serialized form.

### 3.3 Predicate plumbing (M)

`crates/geometry/src/extrema/` · `crates/algo/src/pave_filler/phase_vv.rs`,
`phase_ve.rs`, `phase_ee.rs`

Coincidence and incidence tests consult entity tolerance (sum of ball radii)
instead of the global linear tolerance — starting with the VV/VE/EE pave
phases and the extrema queries they call. The global tolerance remains the
floor; entity tolerance only widens.

> **Exit gate:** a vertex pair separated by 10× global tolerance but within
> their declared balls interferes in VV; all existing suites unchanged
> (entity tolerances default to the floor).

In review in [PR #208](https://github.com/esaueng/remus/pull/208): the existing
10×-global VV ball witness is joined by declared-tube EE/VE, forced-overlap,
and tolerance-aware pave-vertex lookup regressions. SameParameter and
SameRange use the larger of the caller bound and effective edge tolerance;
invalid or overflowing bands refuse with typed errors. Declared values
contribute only their excess above the global floor, so the no-declaration
foils and all 51 approximation-census rows remain unchanged. Result-tolerance
growth and FF/builder assembly remain Issue 3.4.

### 3.4 GFA integration (L)

`crates/algo/src/pave_filler/` · `crates/algo/src/builder/`

The FF/EF acceptance bands, section-curve endpoint snapping, and builder
assembly respect per-entity tolerance. This is where a boolean on gappy
geometry starts producing watertight results whose seams carry honest
(raised, disclosed) tolerances instead of failing assembly.

> **Exit gate:** a synthetically-gapped operand corpus (controlled gap sizes
> 1×–100× global tolerance) booleans correctly; result tolerances reported
> and bounded by the context cap.

### 3.5 Import & sew integration (M)

`crates/io/src/step/reader.rs` · `crates/operations/src/sew.rs` ·
`crates/heal/src/upgrade/shell_sewing.rs`

Sewing records the residual gap as edge tolerance instead of the current
weld-or-fail; the STEP reader assigns entity tolerances from measured gaps at
read time. Heal becomes optional for imports — its remaining job is genuine
defects, not tolerance mismatches.

> **Exit gate:** a corpus of real imperfect STEP files (starting with the
> committed Shapr3D fixture plus collected client files) completes
> import → boolean → export with *zero* heal invocations.

### 3.6 Downstream disclosure (S)

`crates/check/src/` · `crates/wasm/src/bindings/`

Tolerance statistics in validation reports and measurement results; WASM
accessors for per-entity tolerance; `executeBatch` companions. The heal
analysis tolerance-statistics pass becomes the reporting backbone.

> **Exit gate:** JS callers can read max/mean entity tolerance per solid;
> contract tests pin the payload shape.

## M4 — Body taxonomy: sheet, wire, cellular (L, RFC 0005)

Parasolid bodies are solid, sheet, wire, or general, and booleans work across
them — trim a sheet by a solid, split a solid into cells, imprint edges
without removing material. In Remus the sheet/wire/general axis of the
capability matrix is all-Unqualified, and multi-region results hide behind a
`TODO: use a Compound` in the shell assembler. This is how surface-modeling
workflows and principled multi-body results arrive.

### 4.1 RFC 0005: body classes & cellular results (M)

[`docs/design/rfc-0005-body-taxonomy.md`](../design/rfc-0005-body-taxonomy.md)

Sheet-body semantics (an open shell as a first-class operand: orientation,
boundary wires, validation contract), wire bodies, and the cellular result
model — what a boolean returns when the outcome is regions (Compound of
solids with shared-face bookkeeping vs. true cell complex; recommend the
former first, it composes with the existing Compound type). Classification
semantics for sheet operands (side-of, not in/out).

> **Exit gate:** RFC merged; body-class axis of the capability matrix
> re-declared against it.

Delivered in [PR #127](https://github.com/esaueng/remus/pull/127): the RFC
maps the existing capability-matrix body axis to solid, sheet, wire, and
general-body semantics; side-of sheet classification; Compound-first
cellular results; STEP entities; and construction-derived evolution. All
non-solid cells remain Unqualified until Issues 4.2–4.7 land.

### 4.2 Sheet bodies first-class (M)

`crates/topology/src/shell.rs` · `crates/check/src/validate/` ·
`crates/operations/src/tessellate/` · `crates/io/src/step/`

Partial, in review in [PR #209](https://github.com/esaueng/remus/pull/209),
[PR #210](https://github.com/esaueng/remus/pull/210), and
[PR #211](https://github.com/esaueng/remus/pull/211): RFC 0005 Stage 1 adds
the public solid/sheet/wire/general vocabulary, validated shell/wire tags,
class-aware validation profiles, stable diagnostics, and backward-compatible
arena-v3 tags. The operations tranche adds transactional face-set
construction, body-level area and typed volume refusal, boundary-preserving
tessellation, and direct/batch WASM contracts. Arena v4 adds standalone sheet
roots with exact trimmed-NURBS/coedge-pcurve replay, root order and duplicate
preservation, typed transactional refusal, WASM parity, and frozen v3 writer
bytes. Bounding box, center-of-area, and STEP round-trip remain required; the
Issue 4.2 exit gate and capability-cell promotions stay open.

Open shells as bodies with their own validation profile (free boundary
allowed and reported, orientation consistent), area properties, tessellation,
and STEP round-trip (`SHELL_BASED_SURFACE_MODEL`).

> **Exit gate:** a trimmed NURBS patch survives
> construct → validate → tessellate → STEP round-trip; validation
> distinguishes "open by design" from "should be closed."

### 4.3 Split solid by sheet (M)

`crates/operations/src/split.rs` · `crates/operations/src/section.rs` ·
`crates/algo/src/`

Generalize split-by-plane to split-by-sheet-body: the sheet's faces act as
the tool's face set in GFA without a bounding solid. First consumer of the
cellular result model.

> **Exit gate:** a curved sheet splits a solid into N regions whose volumes
> sum exactly to the original; each region individually valid; determinism
> pinned.

### 4.4 Trim sheet by solid / sheet × sheet (M)

`crates/operations/src/boolean/` · `crates/algo/src/builder/`

Keep-inside/keep-outside trims of a sheet against a solid, and mutual
sheet×sheet trims — the surface-modeling loop that ends in `sew` producing a
solid. Classification of sheet faces against the solid reuses the M2-hardened
classifiers.

> **Exit gate:** build a closed solid purely from mutually-trimmed sheets +
> sew; volume matches the same solid built by primitive booleans.

### 4.5 Imprint (M)

`crates/operations/` (new module) · `crates/algo/src/builder/`

Imprint the intersection edges of one body onto another's faces without
removing material — GFA's split phase without the classification/discard
phase. The naming stack makes this shine: imprints journal as pure Split
events, so persistent refs across an imprint are exact.

> **Exit gate:** imprinted solid has identical volume, split faces claimed by
> Split events, zero unresolved; refs to pre-imprint faces resolve BoundMany.

### 4.6 Multi-region boolean output (M)

`crates/algo/src/builder/builder_solid.rs` ·
`crates/operations/src/boolean/`

Retire the `TODO`: when a boolean genuinely produces multiple disjoint
regions, return them as a Compound with per-region provenance instead of the
current single-solid convention. Also closes the "Fuse/Intersect over
disjoint multi-component inputs are left to mesh" note.

> **Exit gate:** a cut that severs a body returns two valid solids with
> correct volumes and complete evolution; disjoint-operand fuse no longer
> routes to mesh.

### 4.7 Wire bodies (S, deferrable)

Wire bodies as measurable, transformable, sweepable first-class inputs.
Mostly bookkeeping once 4.1 lands; defer freely.

> **Exit gate:** wire body round-trips arena IO; sweeps accept it as a
> profile source.

## M5 — Blend depth (L)

The gap between "fillets a box" and "fillets a casting" is most of what
separates kernel tiers. Deliberately after M2/M3: the walking engine needs
general SSI and tolerant contact before curved-support blends can converge on
real parts. The v2 walking engine is the target; the legacy rolling-ball
assembler retires when parity is proven, per the existing ledger note.

### 5.1 Variable-radius qualification (M)

`crates/blend/src/radius_law.rs` · `crates/blend/src/walker.rs`

The radius-law machinery (constant/linear/S-curve/custom) exists but is
Unqualified. Qualify against oracles: a linear law on a straight edge has a
closed-form band; S-curve verified by sampled-section invariants (radius at
parameter, tangency to both supports).

> **Exit gate:** variable-radius cells move to Qualified with closed-form +
> invariant oracles; refusals typed at law-domain boundaries (radius → 0,
> radius ≥ local limit).

### 5.2 Curved-support blends (L)

`crates/blend/src/walker.rs` · `crates/blend/src/fillet_builder.rs` ·
`crates/blend/src/trimmer.rs` · `crates/blend/src/analytic.rs`

The walking engine over quadric×quadric supports: cylinder/cone,
cylinder/sphere, cone/cone edges. Brings closed-rim chamfers out of
experimental and unblocks the pinned `resize_blend` refusal (cylinder-wall ×
cone-wall rim reconstruction). Contact curves on curved supports are exactly
the geometry 2.4 taught the splitter to consume.

> **Exit gate:** fillet a cylinder-cone shoulder and a cross-drilled hole
> rim: watertight, volume vs. mesh oracle, free-edge count zero; the
> resize_blend `unsupported-support-pair` refusal flips to reconstruction.

### 5.3 General vertex blends, N ≥ 3 (M)

`crates/blend/src/corner.rs` · `crates/blend/src/spherical_triangle.rs`

Generalize the spherical-triangle corner patch to N incident stripes with
mixed convexity — the setback-free corner solver Parasolid calls a vertex
blend.

> **Exit gate:** all-edges-filleted box (3-stripe corners) and 4-stripe
> pyramid apex close watertight; G1 across every stripe-corner boundary
> within angular tolerance.

### 5.4 Setbacks (M)

Per-edge setback distances pulling the corner patch away from the vertex
along each spine — required for the corner topologies mixed-radius chains
produce.

> **Exit gate:** mixed-radius three-edge corner with declared setbacks
> builds; setback distances verified on the result spines.

### 5.5 Overflow & cliff handling (L)

`crates/blend/src/trimmer.rs` · `crates/blend/src/walker.rs`

A blend wider than its support face must roll over the next edge (overflow)
or stop against it (cliff) instead of refusing. This is re-limitation against
the neighbor's neighbor — shared machinery with 6.1, build once.

> **Exit gate:** fillet radius exceeding a thin wall's width produces the
> overflowed topology or a typed cliff refusal per declared policy — never a
> wrong band.

### 5.6 Face-face blends & hold lines (L, stretch)

Blends defined by two face sets rather than an edge (the faces need not share
one), and hold-line variants where one contact curve is prescribed. The
styling tier — genuinely optional for mechanical CAD; keep behind the rest of
M5.

> **Exit gate:** face-face blend between disjoint-edge faces builds with
> prescribed radius; hold-line contact verified on the result.

### 5.7 Offset self-intersection removal (M)

`crates/offset/src/self_int.rs` · `crates/operations/src/shell_op.rs`

Filed under M5 because it unblocks real shelling: global self-intersection
removal is a standing typed refusal, mandatory for thin-wall parts whose
offset folds. The BOP-based approach in `self_int.rs` becomes viable once M2
generality lands. Pull forward opportunistically.

> **Exit gate:** shell an L-bracket at a thickness that folds the inner
> offset: folded region excised, result valid, volume vs. mesh oracle; the
> ledger's "still refusing" note removed.

## M6 — Direct modeling (L)

Move, offset, and delete faces on any body with automatic neighbor
re-limitation — the Synchronous-Technology substrate. Unusually attractive
here: the RFC 0003 naming stack means Remus can offer direct edits *with
persistent references that survive them*, which few kernels can. One core
primitive powers the whole milestone.

### 6.1 Core re-limitation primitive: replace-surface (L)

`crates/operations/` (new module) · `crates/algo/` · `crates/geometry/`

Replace a face's surface, extend the neighbors' surfaces (needs 7.4),
re-intersect every affected edge, rebuild trims and p-curves. Everything else
in this milestone is a caller of this function. Failure policy: if
re-intersection loses an edge or opens the shell, refuse typed with the
offending adjacency named — never emit the broken solid.

> **Exit gate:** replace a planar face with a tilted plane and a cylindrical
> wall with a larger radius on a bored block: watertight, valid, all neighbor
> trims re-derived, volumes vs. oracle.

### 6.2 Move / rotate / offset face, generalized (M)

`crates/operations/src/push_pull.rs` · `crates/offset/src/move_faces.rs`

Generalize the planar-only `move_faces` and `push_pull` through 6.1:
transformed or offset surface in, re-limitation out. Includes through-feature
preservation (a moved wall carries its holes).

> **Exit gate:** move a boss across a filleted plate; holes and fillet bands
> re-limit; persistent refs to every moved face still resolve Bound.

### 6.3 Delete-face-and-heal, curved (M)

`crates/operations/src/defeature.rs`

Generalize the defeature extend strategy past its planar declared domain:
deleting a face heals by extending curved neighbors through 6.1 machinery.
Closes the "wounds crossing curved kept faces" typed refusal.

> **Exit gate:** delete a fillet band and a boss on a curved-walled part;
> neighbors re-extend exactly; restored volume oracles hold.

### 6.4 Curved-face draft (M)

`crates/operations/src/draft.rs`

Draft is Stable for planar faces with typed refusals everywhere else —
non-planar targets, curved re-trim neighbours, hole-rim walls. The 6.1
re-limitation primitive is exactly the machinery that lifts those refusals: a
drafted curved wall is a replace-surface (tapered ruled/conic surface) plus
neighbor re-limitation. Cheap once 6.1 exists; a mold-work staple.

> **Exit gate:** draft a cylindrical boss wall and a wall carrying a hole
> rim: volume oracles against the closed-form frustum/taper, refusal cells
> flip both-sides-tested, qualification axes extended in `qualify_draft.rs`.

### 6.5 Journaled direct edits (S)

`crates/operations/src/journal_ops.rs` · `crates/wasm/src/bindings/`

Every direct-modeling op emits real evolution events (Modified for re-limited
neighbors, Deleted, Generated for heal caps) — no barriers. WASM surfacing
with executeBatch companions. This closes the declared evolution remainder
for direct edits.

> **Exit gate:** edit → resolve-ref round-trip pinned for all 6.x ops; zero
> unresolved events on the qualification fixtures.

## M7 — Sweep, surfacing & interrogation depth (M)

The sweeps family is broad but shallow: no guide rails, laws, continuity
constraints, or periodic lofts, and the ledger holds every row Blocked on
degenerate/nonconvergence matrices. 7.4 is on M6's critical path — build it
early, out of order. 7.5 collects the read-only interrogation staples; it
depends on nothing and parallelizes freely.

### 7.1 Guided sweeps: rails, twist, scale laws (M)

`crates/operations/src/sweep.rs` · `crates/operations/src/pipe.rs`

Sweep a profile along a spine with guide-rail orientation control and
parametric twist/scale laws. Frame transport (rotation-minimizing frames)
verified against closed-form helical cases the helix op already covers.

> **Exit gate:** rail-guided rectangular sweep matches its closed-form ruled
> construction; twist law verified by section sampling; self-intersecting
> spine curvature refused typed.

### 7.2 Loft continuity & periodic lofts (M)

`crates/operations/src/loft.rs` · `crates/math/src/nurbs/surface_fitting.rs`

Tangency end-conditions (to a face or a direction), G1 between loft segments,
and closed/periodic lofts (a loft that meets itself must close with the seam
machinery, not a near-coincident pair of faces).

> **Exit gate:** loft tangent to end faces measures G1 within angular
> tolerance across the junction; periodic loft closes watertight with a true
> seam.

### 7.3 Constrained N-sided fill (M)

`crates/operations/src/fill_face.rs`

Upgrade Coons fill to N-sided patches with G1 boundary conditions against
neighbor faces — the hole-patching workhorse for M4 sheet workflows and 5.6.

> **Exit gate:** fill a 5-sided hole in a curved sheet with G1 to all
> neighbors, measured by normal deviation sampling along every boundary.

### 7.4 Surface extension & curve imprint (M — build first)

`crates/geometry/` · `crates/math/src/nurbs/` · `crates/operations/`

Natural extension of analytic surfaces (trivial) and NURBS surfaces
(knot-extend / extrapolate with curvature control), plus
projecting/imprinting a curve onto a face. Both are 6.1 prerequisites —
schedule immediately after M2, before the rest of M7.

> **Exit gate:** extended NURBS surface contains the original exactly
> (sampled identity on the shared domain); projected curve lies on-face
> within tolerance across curvature regimes.

### 7.5 Interrogation: clash, silhouettes, curvature & draft analysis (M)

`crates/check/` · `crates/operations/src/projection.rs` ·
`crates/operations/src/measure/`

The Parasolid interrogation staples, bundled because they share machinery and
parallelize cleanly across sessions: **assembly clash/clearance** (pairwise
body interference with witness points and clearance distance — the distance
engine plus the M2-hardened classifiers, driven from the assembly tree),
**silhouette curves** (view-dependent outlines on curved faces — also what
makes hidden-line output real rather than tessellation-approximate),
**curvature analysis** (principal/Gaussian/mean maps, minimum-radius
queries — feeds manufacturability checks and M5 radius limits), and
**draft-angle analysis** (per-face pull-direction angle maps, the read-only
twin of 6.4). All read-only, low risk, high product value.

> **Exit gate:** clash: witness-point distance matches brute-force sampled
> minimum on a touching/clearance/interfering triple. Silhouette of a torus
> matches its closed form. Curvature maps exact on all five analytic
> primitives, NURBS within fitted tolerance. All surfaced through WASM per
> R8.

## M8 — Industrialization (L)

What earns the robustness claim rather than asserting it. 8.1 starts as soon
as 2.4 lands — the differential harness then guards every subsequent
milestone's churn, which is worth far more than running it at the end.

### 8.1 Differential testing harness (L — pull early)

Randomized operation sequences over seeded primitive/imported bodies, checked
against invariants rather than golden outputs: boolean identities
(vol(A∪B) + vol(A∩B) = vol(A) + vol(B)), cut/fuse complementarity,
mesh-boolean cross-check within deflection bound, watertightness, determinism
across runs, journal completeness (every face claimed once). Failures shrink
to repro bundles automatically (the schema-1 runner from Issue 2 is the
substrate). Nightly, not per-PR.

> **Exit gate:** harness in CI on a nightly schedule; any failure produces a
> replayable bundle; first 10 shaken-out defects filed and fixed.

### 8.2 Performance budget gates (S)

Criterion baselines for the operation suite become CI gates with declared
regression thresholds, so perf loss is caught at the PR, not the release.

> **Exit gate:** a deliberate 2× slowdown in a bench fails the gate; noise
> threshold tuned so a clean PR passes 20/20.

### 8.3 Parallel tessellation (M)

Per-face parallelism with deterministic assembly ordering (`det_hash` already
provides seed-stable containers). Tessellation is embarrassingly parallel and
the biggest interactive-latency win for consumers.

> **Exit gate:** bit-identical meshes serial vs. parallel across the fixture
> corpus; ≥3× wall-clock on an 8-core reference part.

### 8.4 Parallel boolean internals (L, stretch)

Parallel FF-pair processing and face splitting with deterministic reduction.
Only after 8.1 is guarding — determinism is a shipped guarantee (the 200-run
gate) and this is the riskiest way to lose it.

> **Exit gate:** the 200-run determinism gate holds under parallel execution;
> measured speedup on the N-body fuse benches.

### 8.5 Real-model corpus (M)

Nightly import → operate → export sweep over collected real STEP assemblies,
tracking pass/fail per model over time. The census, scaled up from
constructed pairs to the geometry customers actually have.

> **Exit gate:** corpus ≥ 50 real models running nightly with a tracked
> scoreboard; regressions bisectable to a merge.

## §4 Dependency structure

```
M2 booleans ──────────┬──> M4 bodies ─────┐
  (load-bearing wall) ├──> M5 blends ─────┤
                      ├──> M6 direct ─────┼──> Exit benchmarks B1–B5
                      └──> 8.1 diff-test  │
M3 tolerant (RFC ∥ M2, integrate > 2.4) ──┤
M7.4 extend/imprint (early) ──> M6 ───────┘
```

Practical reading: **M2 first and alone** — it churns the pave filler, so
nothing else should. The M3 and M4 RFCs are written during M2 (design work,
disjoint files). After 2.4, three tracks parallelize cleanly for concurrent
sessions: M3 integration (algo acceptance bands), M4 (builder/operations),
and 7.4 + 8.1 (geometry/math + test harness — near-zero file overlap with
either). The rest of M7 (7.1–7.3, 7.5) and M8 (8.2–8.5) attach where capacity
allows.

## §5 Program exit benchmarks

Milestone gates prove parts; these five scenarios prove the program. Each
becomes a permanent CI-adjacent integration test when it first passes.

| | Scenario | Exit | Milestones |
|---|---|---|---|
| B1 | Import two real freeform STEP bodies, fuse, fillet the intersection seam, export STEP | Re-import matches: watertight, valid, volume stable, names round-trip | M2 + M5 |
| B2 | A gappy real-world import booleans correctly with zero heal invocations | Result tolerances disclosed and bounded by the context cap | M3 |
| B3 | Shell a curved thin-wall part at a thickness that folds the inner offset | Self-intersection excised, valid result, volume vs. mesh oracle | M2 + 5.7 |
| B4 | Direct-edit an imported body: move a boss across a filleted, holed plate | Neighbors re-limit; persistent references resolve Bound afterward | M6 |
| B5 | The full boolean qualification suite at model scales 1e-5 through 1e6 | Exact or typed refusal at every scale — never a silent wrong volume | 2.6 |

## §6 Inherited queue & deliberate non-goals

**Inherited queue** — small open items from prior programs, tracked here so
the big milestones don't silently swallow them. None deserves a milestone;
all deserve to stay findable. Fair game for any session needing a bounded
task:

- **e3b colors & attribute scope** — STEP `COLOUR_RGB`/`STYLED_ITEM` chains,
  edge/vertex attribute scope, remaining WASM accessors (queued in the e3b
  design doc since Issue 14).
- **OperationsError ToDiagnostic registry** — the one error enum still
  outside the pinned diagnostic registries.
- **Conic curve cells** — hyperbola/parabola intersection and boolean cells,
  Unqualified since the capability matrix was written.
- **Sketch (GCS) qualification** — nonconvergence budget and degeneracy
  matrix for the DogLeg solver (out of Parasolid scope — constraint solving
  is a separate product there — but the ledger row is open).
- **Hidden-line qualification** — the projection row's error/performance
  matrix; upgrades further once 7.5 silhouettes land.
- **Multi-body mesh import split** — `SolidId → Vec<SolidId>` reader
  convention across STL/3MF/OBJ/PLY/glTF (breaking API change, pinned as a
  current-behaviour test; owner's call).

**Deliberate non-goals:**

- **Feature-count parity with Parasolid.** The target is the four properties
  in §0, not the 900-function PK interface.
- **Convergent-style facet modeling** (mesh bodies as first-class B-Rep
  operands). `mesh_boolean` stays a bounded fallback; revisit only after M4
  settles the body taxonomy it would extend.
- **IGES growth.** Decided 2026-08-21: STEP is the exchange path; IGES stays
  a declared lossy preview.
- **History/parametrics above the kernel.** Feature trees,
  constraints-driven regeneration, and UI concerns belong to consumers; the
  kernel's contribution is the naming/evolution substrate, which is done.
- **Upstream v3+ behavior.** Standing fork rule: independent implementation
  or explicit Apache-2.0 grant only.

> **Risk worth naming:** M3 and 5.5/6.1 are the two places this plan can
> silently balloon. If tolerant-modeling integration (3.4) starts
> destabilizing the M2 boolean gains, stop and re-stage — the RFC's stage
> boundaries exist so the program can pause there without stranding work.
