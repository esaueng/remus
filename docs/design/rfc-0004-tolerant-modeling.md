# RFC 0004: Tolerant modeling — per-entity tolerance semantics

Status: draft; staged migration for program milestone M3 (issues 3.1–3.6).
Stage 1 merged in [PR #148](https://github.com/esaueng/remus/pull/148);
Stage 2 is in review in [PR #208](https://github.com/esaueng/remus/pull/208).
Characterization anchors: `crates/algo/src/pave_filler/phase_ee.rs` (module
tests pin declared-tube widening and its no-declaration foil), `crates/heal/src/upgrade/
shell_sewing.rs` (module `sew_shell_preserves_trim_and_tolerance_of_retained_
edges` pins weld-or-decline sewing), `crates/topology/src/edge.rs` (trim/tol-
erance accessor tests). Each stage below names the tests that pin current
behavior and the stage at which they flip.

## Problem

The kernel compares geometry through one global linear tolerance
(`Tolerance::linear`, 1e-7 mm — `math/src/tolerance.rs:22`). Real imported
bodies are not that exact: seams arrive gappy, edges arrive off their
surfaces, and the only way they participate today is to be healed to
exactness first (the pre-tolerant architecture the program doc's M3 section
names). Healing is a lossy, order-sensitive proxy for what the target kernel
class does natively: let every vertex and edge declare how far off it is, and
make every predicate honor the declaration.

The data model is closer to this than the program doc assumes. On `main`:

- `Vertex` already stores a tolerance ball (`vertex.rs:15-20`) — created
  with one, read via `Vertex::tolerance()` (`vertex.rs:37-39`), but with no
  setter at all (`vertex.rs:42-44` exposes only `set_point`), so a ball can
  never be raised after construction and nothing validates what it claims.
- `Edge` already stores `tolerance: Option<f64>` (`edge.rs:194-196`) with
  `set_tolerance` that accepts **any** value unchecked (`edge.rs:324-326`) —
  asserted, never checked — and `effective_tolerance(vertex_tol)`
  (`edge.rs:334-336`) that three consumers already call.
- The pave phases already consume vertex balls in VV (`tol_a + tol_b +
  tol.linear`, `phase_vv.rs:47-57`), VE (`vtol + tol.linear`,
  `phase_ve.rs:90,125-126`), and VF (`phase_vf.rs:88,101-107`).

So the honest problem statement is threefold: (1) the fields exist but their
setters are unvalidated — a tolerance is claimed, not proven; (2) the
predicate plumbing is half-landed — EE, the EE-forcing pass, and the pave-
vertex dedup helpers still use the global tolerance alone (`phase_ee.rs:55,
63,68`, `force_interf_ee.rs:124-127`, `helpers.rs:28,37`); (3) nothing above
the pave filler — FF acceptance bands, builder assembly, sewing, import —
reads entity tolerance at all, so gappy geometry still fails assembly or
demands heal-first.

## Design

### Containment semantics

A vertex is a closed ball of radius `Vertex::tolerance()` around
`Vertex::point()`; an edge is a tube of radius `Edge::effective_tolerance(..)`
around its 3D curve over its trim domain. Two invariants, both
validator-enforced (not asserted):

1. **Ball containment**: for every edge and each bounding vertex, the curve
   evaluated at the endpoint parameter lies within that vertex's ball —
   `|curve(t_end) − vertex.point()| ≤ vertex.tolerance()`. Every incident
   edge end of a vertex lives inside its ball.
2. **Tube containment**: for every edge use with a stored p-curve, the
   sampled 3D↔p-curve deviation is within the edge's effective tolerance —
   this is exactly what `check_same_parameter` already measures
   (`validation.rs:315-362`, returning `SameParameterReport.max_deviation`,
   `validation.rs:290-300`).

### The authority rule: max-of-contributors

When an operation must assign or merge tolerances, the assigned value is the
**maximum over every contributor that measured a deviation**:

- welding two vertices assigns the survivor `max(ball_a, ball_b)`;
- sewing two free edges keeps `max(tube_a, tube_b)` and then records the
  residual gap on top of it (Stage 4);
- a boolean's split vertex inherits the largest tolerance of the entities
  that produced it (today `phase_ee.rs:68` hard-codes `Vertex::new(point,
  tol.linear)`; the raised value replaces that constant, never lowers it).

Never the minimum, never an average: an under-estimate silently invalidates
a neighbor that trusted it.

### Growth discipline

Operations may **raise** an entity tolerance and nothing else. A raise is
legitimate only when a measured deviation demands it, is capped per
operation, and is never silent:

- `OperationContext` gains a cap (additively; the struct is
  `#[non_exhaustive]`, `math/src/context.rs:133-145`) — `max_entity_
  tolerance`, defaulting to `1000 × tolerance.linear`, mirroring the widest
  acceptance band the boolean currently uses (`phase_ff.rs:143`). A raise
  beyond the cap is a typed error, not a clamp: silently clamping would
  paper over the exact gap the raise was recording.
- Every raise is returned to the caller (a raise report on the operation
  result) and recordable in the journal (below).

### The floor rule and the validators

The global tolerance stays the **floor**: comparisons clamp entity tolerance
to `max(entity_value, tol.linear)` — an entity may claim extra precision
(`shell_sewing.rs` already preserves a `3.5e-8` edge tolerance through
merges), but no predicate ever acts below the global floor. Entity tolerance
only widens bands.

SameParameter/SameRange interaction: `validate_same_parameter` and
`validate_same_range` take their bound as a bare `tolerance: f64` argument
(`validation.rs:372-392`, `436-454`); callers pass the global default. The
entity-tolerance rule makes the **validation bound** =
`max(passed_tolerance, edge.effective_tolerance(max(ball_start, ball_end)))`.
A validator then rejects a tolerance that fails to cover the measured
deviation it claims — the checked-not-asserted property, reused rather than
duplicated.

Data flow stays downward: the fields live in `remus-topology` (L1); `remus-
algo` (deps: math + topology) and `remus-operations`/`remus-heal` consult
them; `remus-geometry` gains nothing — its extrema (`point_curve.rs`,
`segment.rs`) are pure distance returns with internal convergence constants
only (`point_curve.rs:15`), and stay that way: comparisons happen in L2,
which already depends on topology. No new workspace dependency; no layer
boundary is crossed.

## Migration

Staged like RFC 0002: one authority per stage, characterization tests pin the
current behavior and flip exactly at the stage that changes it. The program
doc's issue 3.1 exit gate — characterization tests written and passing, with
entity tolerances defaulting to the floor — is Stage 1–2's job; its "they
flip at stage 3" warning pins the GFA stage.

### Stage 1 — substrate: validated setters, validators, journal (issue 3.2)

- `crates/topology/src/vertex.rs`: add `set_tolerance` (there is none today)
  behind the validated-setter contract.
- `crates/topology/src/edge.rs`: `set_tolerance` becomes validating —
  finite, non-negative, and (when a p-curve use exists) no smaller than the
  `check_same_parameter` deviation it claims to cover. `set_trim`
  (`edge.rs:296-298`) is the in-file exemplar of a filtering setter.
- `crates/topology/src/validation.rs`: two checks with stable codes in the
  existing `tolerance_violation` family — `vertex_ball_violation` (invariant
  1) and `edge_tube_violation` (invariant 2, reusing
  `check_same_parameter`/`check_same_range` measurements). Both pass
  vacuously at default tolerances, per the check-crate convention.
- `crates/topology/src/transaction.rs`: raises happen inside
  `run_transacted`/`run_validated` (`transaction.rs:39-51, 65-76`) so a
  vetoed raise rolls back with everything else.
- `crates/topology/src/journal.rs`: a tolerance raise is recorded as an
  `EntityEvent::Modified` on the entity (`journal.rs:209-220`; vertices and
  edges are already `EntityKind`s, `journal.rs:102-111`) — no new event kind.
- `crates/io/src/arena_io.rs`: both fields already serialize (`SerVertex.
  tolerance` required, `SerEdge.tolerance: Option`, `arena_io.rs:117-133`);
  no format change in this stage.
- `crates/math/src/context.rs`: `max_entity_tolerance` cap field + builder.

Characterization tests (pin current, flip here): `edge.rs` tests pin that
`set_tolerance` accepts arbitrary values (`with_tolerance_stores_value`,
`set_tolerance_round_trip`) — they flip to reject sub-deviation and
non-finite raises; `validation.rs` tests pin that validators take the bound
purely from the caller (`validate_same_parameter(.., 1e-7, ..)`) — they flip
to the entity-derived bound; a new arena round-trip test pins
byte-identical serialization of tolerance-bearing documents (already true —
keep it true).

> Exit gate: round-trip byte-stability for legacy documents; validators
> reject a tolerance smaller than the measured deviation it papers over; a
> raise beyond the context cap is a typed error. No consumer behavior
> changes at default tolerances.

### Stage 2 — predicate plumbing (issue 3.3)

Coincidence = sum of ball radii, with the global tolerance as the additive
floor pad. File targets in `crates/algo/src/pave_filler/`:

- `phase_ee.rs`: the crossing acceptance band (`:354-363`), the AABB pad
  (`:55`), and pave-vertex proximity (`:63`) gain the edge-tube term:
  `band = tube_a + tube_b + tol.linear`, where `tube` contributes the
  declared excess over the floor and **0 when undeclared** — default
  behavior is bit-identical to today's global-only band.
- `force_interf_ee.rs`: endpoint matching (`:124-127`) becomes
  `ball/edge-aware` under the same shape.
- `helpers.rs` (`find_nearby_pave_vertex`, `:28,37`) and
  `ds/pave_vertex_index.rs` (`find_within`, `:57`): the lookup radius becomes
  the queried vertex's ball plus the floor; the spatial-hash cell sizing
  must derive from the max radius in play, not `tol.linear` alone.
- `phase_vv.rs`/`phase_ve.rs`/`phase_vf.rs`: already ball-aware on main —
  this stage only adds the edge-tube term to VE's fine test
  (`phase_ve.rs:125`).
- `crates/geometry/src/extrema/`: **unchanged by design** — pure distances;
  their callers do the sum-of-radii comparison.

Stage 1 characterization tests pinned that VV merges a pair separated by up
to `ball_a + ball_b + tol.linear` and that EE ignored declared edge
tolerances. The EE/force-EE/helpers tests flip at this stage.

Delivery in [PR #208](https://github.com/esaueng/remus/pull/208) flips those
Stage 2 pins: EE crossing/AABB, forced EE overlap, pave-vertex lookup, and VE
incidence consume declared tolerance excess while no-declaration foils retain
the old bands. SameParameter/SameRange also consume effective edge tolerance,
as specified above. Invalid and overflowing tolerance bands refuse typed;
the approximation census remains byte-for-byte at its 51 committed rows.
The program doc 3.3 exit-gate fixture (vertex pair at 10× global, inside
declared balls, interferes in VV) is written now as a *passing* pin, since
VV already satisfies it.

> Exit gate: the EE counterpart of the 10× fixture passes; all existing
> suites unchanged with default tolerances (every formula reduces to the
> historical band when nothing is declared).

### Stage 3 — GFA integration (issue 3.4)

- `crates/algo/src/pave_filler/phase_ff.rs`: the section-endpoint trigger and
  weld bands (`:143`, `:151` — the scale-relative band machinery documented
  at `:85-101`) accept an entity-widened band: a face pair whose boundary
  edges declare tolerance widens the weld band by their tubes, still capped
  by the pair-extent fraction.
- `phase_ef.rs`: edge-face acceptance bands take the same treatment.
- `crates/algo/src/builder/` (`wire_builder.rs`, `assemble.rs`,
  `builder_solid.rs`): endpoint snapping and loop-closure checks accept gaps
  within declared tolerances instead of only the global floor, and assembled
  vertices/edges **record** the raised tolerance that made the closure
  possible (max-of-contributors, capped by the context).
- `make_split_edges.rs` / `phase_ff_coplanar.rs`: newly created vertices
  inherit the largest declared tolerance among their sources instead of the
  bare `tol.linear` (`phase_ee.rs:68`, `phase_ff_coplanar.rs:670`).

Characterization tests: pin that a boolean on operands with declared
tolerances ignores them today (bands are pure-global, `phase_ff.rs:143`);
census rows byte-identical at default tolerances. Flips: the synthetically-
gapped operand corpus (1×–100× global, program doc 3.4) booleans correctly
with result tolerances reported and bounded by the context cap.

> Exit gate: the gapped-operand corpus fuses/cuts/intersects with volumes
> verified against the mesh oracle within deflection bound; every raised
> tolerance in every result is disclosed and ≤ the context cap;
> `approx_census` diffed row-by-row against baseline with fallback-row
> movement explained (standing rule 4).

### Stage 4 — import & sew (issue 3.5)

- `crates/heal/src/upgrade/shell_sewing.rs`: today a pair either welds
  (`endpoints_coincide`, `:261-267`; interior agreement sampled in
  `curves_agree`, `:271-300`) or is declined (`SewReport.declined`,
  `:33-43`) — weld-or-fail. Sewing gains the third outcome: a pair whose
  endpoints sit within a *wider, capped* band but whose interiors disagree
  within it merges and records the residual gap as the retained edge's
  tolerance (raising the existing preserved-tolerance path, tested at
  `:663-704`).
- `crates/operations/src/sew.rs`: the spatial-hash weld (`:79-92`) assigns
  the survivor `max` of the merged balls (authority rule) instead of the
  flat `Vertex::new(p, tol)`.
- `crates/io/src/step/reader.rs`: the reader assigns entity tolerances from
  **measured gaps** — vertex pairs coincident within the import band get a
  ball of the measured separation, edges get their p-curve deviation —
  replacing the fixed `1e-7` mm default (`reader.rs:275`), still floored and
  capped by the context.
- Heal becomes optional for imports: its remaining job is genuine defects
  (topology, not tolerance mismatch).

Characterization tests: pin that sewing declines a gapped-but-curve-agreeing
pair today and that STEP import stamps `1e-7` on every vertex regardless of
measured gaps; both flip here.

> Exit gate: the imperfect-STEP corpus (program doc 3.5, starting from the
> committed Shapr3D fixture) completes import → boolean → export with zero
> heal invocations; every tolerance assigned on import is ≤ the context cap
> and round-trips through `arena_io`.

### Stage 5 — downstream disclosure (issue 3.6)

- `crates/check/src/`: validation reports and measurement results carry
  tolerance statistics (max/mean per solid); `crates/heal/src/analysis/
  tolerance.rs` (`ToleranceAnalysis`, `analyze_tolerances`) becomes the
  reporting backbone.
- `crates/wasm/src/bindings/query.rs`: per-entity accessors
  `getVertexTolerance` / `getEdgeTolerance` alongside `getVertexPosition`
  (`:181`); `crates/wasm/src/bindings/batch.rs`: `executeBatch` companions;
  contract tests through `execute_batch()` (program doc standing rule 8).

Characterization tests: pin that validation reports and the JS surface carry
no per-entity tolerance today; flip when the accessors and report fields
land, with the payload shape pinned by contract tests.

> Exit gate: JS callers read max/mean entity tolerance per solid; contract
> tests pin the payload shape; the capability-matrix cells move in the same
> PR (standing rule 5).

## Serialization

Both tolerance fields are already additive arena fields
(`io/src/arena_io.rs:117-133`): vertex tolerance is required, edge tolerance
is `Option`. The pattern for anything this RFC later adds (raise provenance,
per-use tolerance) is the `trim` field's: `#[serde(default,
skip_serializing_if = "Option::is_none")]` — absent-when-default, so older
documents load unchanged forever (`arena_io.rs:129-132`). Round-trip byte
stability for legacy documents is Stage 1's exit gate, and repro bundles are
unaffected (they replay operations, not serialized topology).

## WASM disclosure

Per program doc standing rule 8, every stage that changes observable behavior
ships its binding, `executeBatch` companion, and contract tests inside that
stage's PR — not as Stage 5 cleanup. Stage 1–4 disclosures are read-side
(the raise reports on operation results); Stage 5 adds the per-entity
accessors and statistics payload. All bindings validate inputs via
`error.rs` helpers, return tolerances as plain `f64`s, and add batch
companions only for ops the batch dispatcher already carries.

## Risk worth naming

The program doc's own warning, quoted: *"If tolerant-modeling integration
(3.4) starts destabilizing the M2 boolean gains, stop and re-stage — the
RFC's stage boundaries exist so the program can pause there without
stranding work."* Stage 3 is where the pave filler churns again while M2
(2.4–2.5) may still be landing in it — the program doc schedules M3
integration *after* 2.4 for exactly this reason. The pause protocol: Stages
1–2 (substrate + predicate plumbing) are safe to land under any M2 state
because defaults are bit-identical; Stage 3 must not open while a 2.x PR is
mid-flight in `crates/algo/src/pave_filler/`; if the gapped-operand corpus
regresses M2 census rows, Stage 3 reverts to its characterization pins and
Stages 4–5 proceed independently (import/sew and disclosure do not depend on
FF-band widening — a gappy body can gain honest tolerances at import, and
disclosure can ship, while GFA integration waits).

## Consequences

- **Cost**: modest by RFC 0002 standards — the fields exist; the work is
  validators, the remaining global-only predicate sites, and the four
  integration surfaces (FF/EF bands, builder assembly, sew, import). The
  `Edge`/`Vertex` structs gain no variants, so no `EdgeCurve`/`FaceSurface`-
  scale ripple.
- **Memory**: zero new entities; one `f64` already stored per vertex/edge.
- **Unblocks**: heal-optional imports (3.5), gappy-operand booleans (3.4),
  tolerant blend contact (M5), and honest result-quality disclosure (D4).
- **Discipline**: every raise is measured, capped, disclosed, and
  recordable — the failure mode this RFC forbids is the silent one: a
  tolerance that grows because an algorithm needed it to, visible to no one.

## Resolved questions

- Do faces/shells carry tolerance? No — vertices and edges only, v1. Face
  pair acceptance bands derive from the edges that bound them; a face-level
  field would duplicate a derivable quantity and double the authority
  problem.
- Can entity tolerance shrink below the global floor? Never, in effect:
  comparisons clamp to `tol.linear`. A stored value below the floor (e.g.
  the `3.5e-8` sewing already preserves) is a claim of extra precision and
  never narrows a band.
- Are tolerance changes journal events? Yes — a raise is an
  `EntityEvent::Modified` on the entity (`journal.rs:209-220`), recorded by
  the operation that raised it; no new event kind in v1.
- Does the global tolerance remain meaningful? Yes — it is the floor, the
  default entity tolerance, and the cap denominator (`max_entity_tolerance`
  = 1000× by default). Entity tolerance widens; it never replaces.
- VV already sums balls on `main` — why a stage for predicates? Because the
  landed behavior is unguarded by tests and incomplete (EE, the forcing
  pass, and the pave-vertex dedup are still global-only). Stage 2 pins what
  exists and finishes the shape; the characterization tests are the point.
- Per-use (coedge-level) tolerance? Deferred with RFC 0002's coedge storage
  move: an edge-level tube covers v1 use cases, and per-use trims already
  exist where per-use data is genuinely needed.
