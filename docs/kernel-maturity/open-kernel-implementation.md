# Open Kernel Program — implementation plan

Issue-level breakdown of the [Open Kernel Program](open-kernel-program.md):
every pillar decomposed into staged, independently-shippable issues with
files, sizes, dependencies, and typed exit gates, in the style of the
[P-Class program](p-class-program.md). The status ledger is
[open-kernel-status.md](open-kernel-status.md).

- **Drafted:** 2026-08-29, baseline `main` @ `d154e64`.
- **Standing rules:** P-Class R1–R8 plus R9 (public claims are reproducible
  claims) bind every issue here.
- Size scale: **S** = one PR · **M** = 2–4 PRs · **L** = 5+ PRs, staged.
- Issue IDs are `O<pillar>.<issue>`; letters (`a`, `b`, …) are staged PRs
  inside one issue.

## §0 Repository conventions this program introduces

Decisions made once, so forty issues don't re-litigate them:

1. **Program tooling lives in `tools/`, not `crates/`.** The gauntlet and
   head-to-head harness are consumers of the kernel, not part of it. Add
   `"tools/*"` to `[workspace] members` in the root `Cargo.toml`.
   `scripts/check-boundaries.sh` stays scoped to `crates/*`; a one-line
   guard is added rejecting any `crates/*` crate that depends on a `tools/*`
   crate. Tools may depend on any `remus-*` crate.
2. **The facade is a real layer.** `crates/remus` enters the layer table as
   the L5 apex (allowed deps: `operations`, `io`, `check`, `heal`, `math`,
   `topology`, `sketch`; `render` stays a leaf). `check-boundaries.sh` and
   the CLAUDE.md layer table gain the row in the same PR that creates the
   crate.
3. **Python bindings live in `bindings/python/`, outside the workspace**
   (own `Cargo.toml` with path deps on the facade), so `pyo3`/`maturin`
   never enter the kernel dependency graph — the same isolation the
   committed `crates/wasm/pkg` channel already practices.
4. **Corpus models are never committed or redistributed.** Corpora are
   pinned by *manifest* (URL + sha256 + license class per model) under
   `tools/gauntlet/manifests/`; a fetcher populates a local cache.
   Published artifacts are aggregates and manifests only (ABC's per-model
   licenses require this).
5. **Scoreboards are generated files with pinned inputs.** Every published
   number regenerates from `<harness> + <manifest> + <kernel SHA>`; results
   land on a dedicated `results` branch (or CI artifacts until 6.1's site
   exists), never hand-edited (R9).

---

## O1 — Proof of robustness

### O1.1 Corpus gauntlet (L)

`tools/gauntlet/` (new) · `.github/workflows/gauntlet.yml` (new)

**a — Pipeline skeleton (M).** Binary crate `tools/gauntlet` depending on
`remus-io`, `remus-operations`, `remus-check`. Per-model pipeline, each
stage independently pass/fail with a category from the
[failure taxonomy](failure-taxonomy.md):

1. `read_step` (with `io::limits` guards, wall-clock budget per model),
2. `validate` (L3 validation, per-solid),
3. `probe boolean` — cut by a box at the bbox center, half-diagonal sized;
   record `BooleanQuality` (exact vs fallback is *the* tracked statistic),
4. `tessellate` — watertight + `boundary_edge_count == 0` + manifold,
5. `round-trip` — write STEP, re-read, compare volume/area within
   deflection-scaled bound (oracle: `remus_check::properties`).

Output: one JSONL row per model (stage results, timings, diagnostic codes),
plus an aggregate scoreboard (markdown + JSON). Panics are already
impossible by lint; the runner still isolates each model in a subprocess so
a pathological input can only lose its own row (budget kill = its own
failure class, R4-style honest accounting).

**b — Manifests + fetcher (S).** Manifest schema (id, url, sha256, license
class, size); fetcher with content-addressed local cache and `--sample N
--seed S` deterministic subsetting. Ship three manifests: `smoke` (~50
models — the M8.5 corpus, shared), `abc-1k` (seeded ABC sample), `mambo`
(the curated hard-tier set). ABC scales to 10k+ later by manifest only —
no code change.

**c — CI wiring (S).** `gauntlet.yml`: nightly cron on `smoke` + weekly on
`abc-1k`; uploads scoreboard artifact and appends one trend row (date, SHA,
per-stage pass rates) to the results branch. A pass-rate drop beyond a
declared threshold fails the run — the corpus becomes a ratchet, exactly
like `approx_census`.

**d — Triage loop (M, recurring).** Standing issue template: take the
largest failure class from the latest scoreboard, reduce one representative
to an in-repo fixture (`crates/io/tests/` per the testing doctrine), then
fix or convert to a typed refusal. Every triage PR moves a class's count,
never a hand-picked model.

> **Exit gate (pillar):** nightly `smoke` + weekly `abc-1k` publishing
> per-stage pass rates with taxonomy breakdown; every number regenerable
> from manifest + SHA; ≥5 triage classes closed as fixes or typed refusals.

**Depends on:** nothing. Zero overlap with M2 (new directory + io tests).

### O1.2 Head-to-head benchmark harness (M)

`tools/vs-bench/` (new)

**a — Protocol + runners (M).** Job spec in JSON (operation, operand files
or generator params, deflection), result JSON out (wall-clock, volume,
watertight, error class). Runners are isolated subprocesses so competitor
deps never touch the workspace: Remus (native, via the O4.1 facade); OCCT
(python subprocess via `build123d`/OCP, version pinned); truck (separate
pinned cargo project under `tools/vs-bench/runners/truck/`); Manifold (npm
subprocess; mesh-boolean scenarios only). A competitor missing locally =
row marked "not run," never silently skipped.

**b — Scenario set (S).** The census boolean pairs (box∪sphere, cyl∩cyl,
torus∩box, …), the O1.3 fillet suite, STEP-read timing over the `smoke`
manifest, tessellation at three deflections. Correctness columns count
double: `silent_wrong` (volume off with success reported) is the headline
column, per §1's thesis.

**c — Results page (S).** Generator producing the comparison table with
wins *and* losses (R9), pinned competitor versions, and repro instructions
from a clean checkout.

> **Exit gate:** a stranger can clone, run `tools/vs-bench`, and reproduce
> the published table; at least one headline claim of the form "N% of
> scenarios exact-or-refused vs. competitor's silent-wrong count."

**Depends on:** O4.1a (facade, for the Remus runner); scenario breadth
worth publishing arrives with P-Class 2.4. Build the harness now, publish
after 2.4.

### O1.3 Fillet torture suite (M)

`crates/operations/tests/fillet_torture.rs` (new) ·
`crates/operations/src/test_helpers.rs`

**a — Corpus + runner (S, now).** Constructed cases targeting the known
industry failure classes: band-consumes-adjacent-face, band-meets-band at a
shared edge, radius ≥ support width (thin wall), 3/4/5-edge vertex
pileups, mixed-convexity chains, tangent-continuation chains, fillet across
a hole rim, fillet-the-fillet. Each case asserts exactly one of:
`Built` (watertight + volume vs. mesh oracle + free-edge count 0) or
`TypedRefusal` (stable code, both-sides tested). Crash or silent-wrong =
test failure. Today most rows will pin refusals — that is the point: the
suite is the fixed target M5 issues flip case by case.

**b — Publication (S, after M5).** Disposition table (built / refused+code)
rendered next to the O1.2 results, with the same cases run through OCCT for
the side-by-side.

> **Exit gate:** every case built-or-typed; zero crashes; suite wired into
> CI; post-M5, the published table shows the flip history.

**Depends on:** a — nothing; b — P-Class M5.

### O1.4 STEP conformance (M)

`crates/io/src/step/writer.rs` · `crates/io/src/step/reader.rs` ·
`docs/production-readiness/step-conformance.md` (new)

**a — Validation properties (M).** Writer emits CAx-IF geometric
validation properties (volume, surface area, centroid, independent-curve
length where applicable) per solid via the Recommended Practice entity
chain; reader parses them and `read_step` gains an opt-in check comparing
declared vs. recomputed properties (`remus_check::properties`), reporting
deviations as diagnostics. Round-trip test: write → read → recomputed
properties agree with both the original solid and the embedded
declaration.

**b — CAx-IF test rounds (S).** Add the published MBx-IF test-round models
as a `caxif` gauntlet manifest; deviations from Recommended Practices get
documented as typed limitations in `step-conformance.md` — the honest
ledger pattern, applied to interop.

> **Exit gate:** validation properties round-trip within declared bounds on
> the fixture corpus; conformance doc enumerates every known deviation.

**Depends on:** nothing (io-only). `a` before O5.3 (AP242 inherits it).

---

## O2 — Exactness hardening beyond M2

### O2.1 Native revolution & extrusion surfaces (L — staged as RFC 0006)

**a — RFC 0006 (M).** `docs/design/rfc-0006-swept-analytic-surfaces.md`:
variant semantics (`Revolution { profile: EdgeCurve, axis }`,
`LinearExtrusion { profile: EdgeCurve, direction }`), parameter-domain and
seam/periodicity conventions (revolution is periodic in the sweep
parameter — the torus band machinery is the template), delegate-method
coverage (every new capability behind `FaceSurface` delegates in
`math/src/traits.rs`, per the ripple-scope doctrine), recognition rules
(`geometry/src/convert/recognize_surface.rs` both directions), and the
**wildcard-arm audit protocol**: the ~72 `FaceSurface` `_ =>` arms are
enumerated with the CLAUDE.md rg command and dispositioned per-site in a
committed checklist — the audit artifact is a deliverable, not a promise.

**b — Math substrate (M).** Surface types in `crates/math/src/surfaces.rs`
(evaluate, normal, project, principal curvatures from profile curvature +
sweep), delegate arms in `traits.rs`, property tests against the
NURBS-lowered twin (sampled identity — the differential-oracle technique
from the classification stack). Purely additive; no topology change; can
land during M2.

**c — Topology variants + compiler-flagged sites (L).** Add the two
`FaceSurface` variants; fix every exhaustive match the compiler names; work
the RFC's wildcard checklist by hand. Tessellation gets a structured band
mesher arm (revolution ≈ generalized torus band; extrusion ≈ generalized
cylinder band in `operations/src/tessellate/nonplanar.rs`). Transform/copy/
section/measure arms per checklist. This is the collision-prone stage:
schedule after P-Class 2.4 settles the splitter files.

**d — I/O wiring (M).** `io/src/step/reader.rs` stops lowering
`SURFACE_OF_REVOLUTION` / `SURFACE_OF_LINEAR_EXTRUSION` to NURBS;
`writer.rs` emits them; round-trip fixture asserts **zero NURBS faces** on
a file that had none. `arena_io` additive serialization.

**e — Boolean arms (M).** `math/src/analytic_intersection.rs`: closed
forms where they exist (revolution×plane-through-axis, coaxial
revolution×revolution, extrusion×plane along/normal to direction);
everywhere else the general marcher — with the pair *disclosed*, not
refused, since NURBS seams are the correct answer (P-Class 2.4 doctrine).
Classifier arms in `algo/src/classifier/analytic.rs` or an honest bail.

> **Exit gate:** the O1.1 gauntlet's "% faces imported analytic" statistic
> jumps measurably on real corpora (revolution/extrusion are ubiquitous in
> turned/extruded parts); round-trip zero-lowering fixture; census
> byte-explained.

**Depends on:** a, b — now; c, d, e — after P-Class 2.4.

### O2.2 Conic edges through booleans (M)

`crates/algo/src/gfa.rs` (`reject_unsupported_curves`) ·
`crates/algo/src/pave_filler/` · `crates/math/src/analytic_intersection.rs`

Off-axis cone×plane sections are ellipses/parabolas/hyperbolas; the curve
types exist end-to-end (`EdgeCurve::{Ellipse, Hyperbola, Parabola}`,
IO-supported) but GFA refuses them at the door, so those cuts route to
NURBS or fallback. Lift the rejection one curve type at a time:
ellipse first (most common, bounded — likely mostly works already behind
the gate), then parabola/hyperbola (unbounded — trim semantics need the
explicit-domain machinery from P-Class 2.0). Each type needs: EE/EF
intersection arms, pcurve computation on the carrying quadric, splitter
acceptance, and a closed-form volume oracle (cone frustum sections).

> **Exit gate:** off-axis plane cut of a cone carries exact conic edges
> through fuse/cut/intersect with closed-form volume; the
> `reject_unsupported_curves` list is empty or each remaining entry has a
> pinned typed-refusal test; the conic capability-matrix cells move.

**Depends on:** P-Class 2.0 (explicit trims). Same files as M2 — schedule
in the M2 track, not parallel to it.

### O2.3 UV-arrangement splitter (L — the debt retirement)

`crates/algo/src/builder/face_splitter/` (new `arrangement.rs`) ·
`special_cases.rs` (~4,300 lines, the retirement target)

**a — Inventory + design note (S).** Catalog every `special_cases.rs`
entry point with the fixture(s) pinning it and the geometric situation it
patches; define the arrangement API: input = face UV domain + section
curves (chord-tolerant polylines with exact crossing refinement), output =
planar subdivision (vertices/edges/regions) with seam-unwrap for periodic
domains and pole handling for spheres/cones. Decide the snapping model
(snap-rounding vs. exact-predicate vertices) — `orient2d` from
`math/src/predicates.rs` is the substrate either way.

**b — Core arrangement (L).** The subdivision structure + insertion +
region extraction, exhaustively property-tested *in isolation* (random
segment/arc soups: Euler formula holds, regions close, determinism across
permutations — `det_hash` ordering). No consumer change yet.

**c — Winding classification bridge (M).** Region → keep/discard via the
existing winding doctrine (`builder/classify_2d.rs`); wire as an
alternative face-splitting path behind a flag, differential-tested against
the current splitter on the whole fixture corpus (same result or explained
divergence).

**d — Migration per case (M × n).** One special case at a time: route its
configuration through the arrangement, prove its fixtures + census rows,
delete the patch. A tracked line-count ratchet on `special_cases.rs`
(scripted, like the reader-site grep gate in P-Class 2.0) makes regressions
visible. The TERMINAL roadmap entries (plane-by-sphere equator, scoop
cone-split coordination) re-open against the new primitive as candidates.

> **Exit gate:** ≥3 special-case paths deleted with fixtures passing
> through the general path; ratchet trending down in CI; at least one
> TERMINAL entry closed or re-scoped with the primitive in hand.

**Depends on:** P-Class 2.4 (same files, and 2.4's splitter work informs
the API). a can be written during M2 (read-only analysis).

---

## O3 — Native performance

### O3.1 Inner-loop benches (S per crate — do first)

`crates/math/benches/` (new) · `crates/algo/benches/` (new) ·
`crates/blend/benches/` (new) · `scripts/bench-compare.sh`

Criterion suites for the loops a flamegraph of the existing 64-cut and
gridfinity benches names — expected set: NURBS basis/evaluate/derivatives
(degree 3 and 9), SSI seeding + marching on a quadric pair and a
NURBS pair, Bézier clipping, CDT insertion at 1k/10k points, pave-filler
phase timings through a `gfa` fixture, blend walker steps-per-second.
Wire into `bench-compare.sh` and `benchmark.yml`. This is M8.2's
prerequisite made explicit: gates need baselines to gate.

> **Exit gate:** every function above a declared flamegraph threshold
> (≥3% of the 64-cut bench) has a bench; `bench-compare.sh` covers the
> new crates.

**Depends on:** nothing.

### O3.2 Journal-invalidated spatial cache (M)

`crates/topology/src/` (new `spatial.rs`) · consumers:
`crates/check/src/classify/mod.rs`, `crates/check/src/distance/mod.rs`,
`crates/operations/src/distance.rs`, `crates/operations/src/boolean/classify.rs`

All seven BVH consumers currently rebuild per query. Add a face-level
AABB/BVH cache keyed by (solid id, journal revision): the evolution
journal already records exactly which faces an operation touched, so
invalidation is precise, not heuristic — the naming stack paying a perf
dividend. Design note first (S): where the cache lives (topology owns the
journal; BVH type lives in math — layer-legal), mutation-safety rules
(transactions invalidate on commit), memory bounds (LRU per topology).
Then migrate consumers one at a time with correctness suites
(mutate-then-query returns post-mutation truth) and repeated-query benches
(classify N points, N-body clash from 7.5's future workload).

> **Exit gate:** repeated-query benches show rebuild eliminated (≥5× on
> classify-1k-points-again); mutate-then-query suite green; determinism
> gates unchanged; all seven sites migrated or explicitly exempted.

**Depends on:** O3.1 (baseline first). Touches topology — coordinate with
P-Class M3.2's topology PRs (additive fields, different files, low risk).

### O3.3 SIMD in NURBS evaluation (S, evidence-gated)

`crates/math/src/nurbs/basis.rs` · `evaluator.rs`

Only if O3.1's benches name basis/evaluation hot: `wide` (or portable
SIMD) behind a `simd` feature, wasm128 variant, differential-tested
bit-for-bit off vs. on where the lanes allow, else within 1 ulp with the
tolerance doctrine consulted. Closed as not-worth-it with numbers attached
if the speedup is <1.5× (that outcome is a valid exit).

> **Exit gate:** ≥1.5× on the named benches native + wasm, or a documented
> negative result.

---

## O4 — Front door & distribution

### O4.1 The `remus` facade crate (M)

`crates/remus/` (new) · `scripts/check-boundaries.sh` · `CLAUDE.md` ·
`crates/wasm/src/kernel.rs`

**a — Crate + Model type (M).** `remus::Model` owning `Topology` +
`OperationContext` + journal access; prelude re-exporting the curated
surface (primitives, booleans with quality/policy, fillet/chamfer v2,
sweeps, measure, tessellate, STEP, validation, persistent refs); typed
errors re-exported flat. API shape mirrors the wasm `BrepKernel` (it is
the proven consumer surface — 299 methods say what users need), minus
JS-isms. Boundary script + layer table row in the same PR. README
quickstart becomes a doctest.

**b — Examples (S).** Three real examples in `crates/remus/examples/`:
`bracket.rs` (sketch → extrude → fillet → measure → STEP),
`import_repair.rs` (read imperfect STEP → validate → heal-or-tolerate →
boolean), `browser_parity.rs` (the wasm contract fixture, natively). The
debug probes stay where they are.

**c — WASM delegation (M).** `BrepKernel` methods delegate to the facade
— behavior-neutral refactor proven by the untouched wasm contract-test
suite. One implementation, two surfaces; native/WASM parity (target
characteristic 15) becomes structural instead of tested-for.

> **Exit gate:** quickstart doctest compiles and runs in CI; wasm contract
> tests byte-identical before/after delegation; boundaries job green with
> the new row.

**Depends on:** nothing. New crate = zero collision.

### O4.2 Publishing pipeline (M — the publish itself is owner-gated)

`Cargo.toml` (workspace) · `.github/workflows/` ·
`docs/production-readiness/release-checklist.md`

**a — Dry-run readiness (S).** Per-crate metadata (description, keywords,
categories, docs.rs config), `cargo publish --dry-run` in dependency order
scripted as an `xtask publish-check` subcommand (the xtask pattern already
exists for wasm builds), README badges plan. npm: `wasm-pkg` dry-run
`npm pack` validation via the existing release-flow harness.

**b — Tag-driven release automation (S).** Workflow: tag → full CI →
publish crates in order → build + publish npm → attach scoreboard snapshot
(O1.1) to the GitHub release. Dry-run mode on by default.

**c — First publish (owner decision).** Explicit gate: nothing publishes
until the owner flips it. Recommended trigger per the program doc: after
P-Class 2.4 (census fallback rows closed) so v0.1's boolean story leads
strong. Versioning policy: 0.x semver with additive-only within a minor,
matching the existing "no breaking change without versioned migration"
constraint.

> **Exit gate (a+b):** `xtask publish-check` green in CI on every PR;
> release workflow proven end-to-end in dry-run against a test tag.

**Depends on:** O4.1 (publish the facade or publish 13 crates with no
door — facade first).

### O4.3 Python bindings (L)

`bindings/python/` (new, outside workspace)

**a — Core binding (M).** PyO3 over the facade only (never the internal
crates): `Model`, primitives, booleans (+ quality), fillet/chamfer,
measure, STEP bytes/paths, tessellate → numpy-shaped buffers, persistent
refs. Error mapping: typed kernel errors → Python exception hierarchy
carrying the stable diagnostic code. Validation helpers mirrored from
`wasm/src/error.rs` semantics.

**b — Wheels + CI (M).** maturin build matrix (manylinux, macOS
universal2, Windows), abi3 wheels, a `python-ci` workflow running the
contract tests. Contract tests are *ported from the wasm suite* — same
fixtures, same expected volumes — so the three doors provably front one
kernel.

**c — Publish (owner-gated, with O4.2c).** TestPyPI first.

> **Exit gate:** `pip install` from a built wheel → box-fillet-measure in
> <10 lines on all three OSes; contract parity suite green; docs page with
> the CadQuery-community positioning (import/repair/boolean strengths).

**Depends on:** O4.1. Fully parallel to everything else once the facade
exists.

### O4.4 Contract surface completion (S)

`crates/operations/src/` (error registry) ·
`docs/design/deferred-e5b-stable-error-codes.md` (the queued design)

Implement e5b: `OperationsError` joins the pinned `ToDiagnostic`
registries (the one enum still outside them, per the inherited queue);
every public operation's error codes enumerated in rustdoc; a
registry-completeness test (every variant maps, no code reused). This is
what makes O4.3's exception hierarchy and O1.1's taxonomy columns stable
identifiers rather than strings.

> **Exit gate:** e5b's own acceptance list; registry-completeness test in
> CI; docs render the full code table (feeds O6.1).

**Depends on:** nothing.

---

## O5 — Interchange depth

### O5.1 STEP assemblies (M)

`crates/io/src/step/reader.rs` · `writer.rs` ·
`crates/operations/src/assembly.rs` · `crates/wasm/src/bindings/assembly.rs`

**a — Reader (M).** Parse product structure
(`PRODUCT`/`PRODUCT_DEFINITION`/`NEXT_ASSEMBLY_USAGE_OCCURRENCE` +
`ITEM_DEFINED_TRANSFORMATION` chains) into the existing qualified
in-memory assembly model (hierarchy, transforms, instance names). New
entry point `read_step_assembly(input, topo) -> (Assembly, Vec<SolidId>)`
alongside the existing solid-list reader (additive; the old signature
stays). Fixtures: constructed deep-hierarchy files + real multi-part
assemblies into the io corpus.

**b — Writer (M).** Emit product structure from an `Assembly`; instances
share one `SHAPE_REPRESENTATION` per unique solid (the whole point of
assemblies). Round-trip oracle: flatten(read(write(a))) matches
`a.flatten()` — transforms verified against the already-qualified direct
matrix composition, BOM equality, name preservation.

**c — WASM + batch (S).** `importStepAssembly`/`exportStepAssembly`,
`executeBatch` companions, contract tests (R8: not done until JS can call
it).

> **Exit gate:** deep-hierarchy round-trip with transform/BOM/name
> oracles; a real imported third-party assembly re-exports and re-imports
> stable; gauntlet gains an assembly stage for manifest models that carry
> structure.

**Depends on:** nothing (io + assembly module; zero M2 overlap).

### O5.2 Colors, names, attribute scope — e3b promoted (M)

`crates/io/src/step/` · `crates/topology/src/attributes.rs` ·
`crates/wasm/src/bindings/` ·
`docs/design/deferred-e3b-step-names-and-colors.md` (the queued design)

Execute the e3b design as written: reader follows
`COLOUR_RGB`/`STYLED_ITEM`/`PRESENTATION_STYLE_ASSIGNMENT` chains into the
existing attribute store (which already holds color but only serializes
via glTF/arena); writer emits the chains; attribute scope extends to
edges/vertices per the doc; WASM accessors complete. Interacts with O5.1:
styling binds to the assembly-instanced representations, so land after
5.1a's reader restructuring or coordinate the files.

> **Exit gate:** e3b's own acceptance list; a colored, named assembly
> round-trips STEP → arena → STEP with the attribute payload stable;
> capability-matrix I/O row updated.

**Depends on:** O5.1a (file coordination only).

### O5.3 AP242 and PMI (L, staged — each stage gated on CAx-IF models)

`crates/io/src/step/writer.rs` · `reader.rs` · new `step/pmi.rs`

**a — AP242 writer schema (S/M).** Emit AP242 headers/ap-schema (reader is
already schema-agnostic); config on the write call
(`StepSchema::{Ap203, Ap242}`), default unchanged until qualified.
Receiving-system check scripted: import the output in OCCT-python and one
other independent reader in CI, assert solid count + volume.

**b — PMI read (L).** Parse semantic PMI (datums, feature control frames,
dimensions, notes) into an attribute-anchored representation whose
geometry links resolve through **persistent refs** (`topology/naming.rs`)
— the design decision that differentiates: PMI that survives edits. Scope
strictly read-and-report first (interrogation, WASM disclosure); no write.

**c — PMI write (L, later).** Semantic write of the 5.3b model, gated on
CAx-IF PMI test rounds. Explicitly last; the swamp risk from the program
doc's §9 lives here.

> **Exit gate (per stage):** a — output validates + imports in two
> independent systems; b — PMI from CAx-IF fixtures binds to refs that
> survive a direct edit (the flagship demo); c — round-trip through the
> test-round models.

**Depends on:** O1.4a (validation properties precede PMI credibility);
5.3b's binding needs nothing from P-Class but *shows best* after M6
exists.

---

## O6 — Ecosystem & community

### O6.1 Docs site (M)

`docs/book/` (new, mdBook) · `.github/workflows/docs.yml` (new)

Structure: quickstart (the facade doctest), architecture tour (from the
CLAUDE.md module map — single-sourced, not copied), the capability
matrix / stability ledger / failure taxonomy rendered *as-is* (the honesty
is the marketing), error-code registry (from O4.4), corpus scoreboard
(from O1.1c). rustdoc published alongside. Site deploys from `main` to
GitHub Pages.

> **Exit gate:** site builds in CI, deploys on merge; every page that
> mirrors a repo doc is generated from it (no forked truth — the
> two-sources rule from target.md applied to docs).

**Depends on:** content improves as O1/O4 land; skeleton has no deps.

### O6.2 Browser playground (M)

`web/playground/` (new) or separate repo (owner's call)

Vite app on the committed `crates/wasm/pkg` package (the same channel
OpenZCAD consumes — dogfooding the distribution): scene tree, primitive +
boolean + fillet palette, grouped-mesh rendering with face-id picking
(`GroupedMeshResult` exists), STEP import/export via file drop, and a
"persistent naming demo" scene — fillet a face, move it, watch the
reference survive (S7's demo made public). Three.js rendering; the native
`render` crate is not involved (wasm stays the browser path).

> **Exit gate:** hosted playground where fillet-a-bracket works from a
> cold link; the naming demo scripted; page links the scoreboards.

**Depends on:** O4.1c helps but isn't required (wasm surface already
sufficient).

### O6.3 Second-consumer track (ongoing; outreach is owner's)

Agent-side deliverables only: a replicad-style worker integration example
(`web/playground` doubles as it), the O4.3 wheels for the
CadQuery/build123d audience, and a standing rule that an external
consumer's reported defect gets a fixture in-repo within one triage cycle
(the Manifold/OpenSCAD symbiosis, operationalized). Outreach itself —
issues/PRs on third-party repos, forum posts — is explicitly the owner's
to initiate.

### O6.4 Contribution posture (S)

`CONTRIBUTING.md` (new) · issue templates ·
`docs/production-readiness/fork-maintenance.md` (link)

Inbound=outbound Apache-2.0 (no CLA — the OCCT contrast, stated),
build/test quickstart, the conventional-commit + hooks contract, the
clean-room/fork-provenance rules contributors must know, and ~10 starter
issues seeded from the inherited queue + O3.1 bench list (real, bounded,
labeled).

> **Exit gate:** CONTRIBUTING merged; starter issues open; templates live.

---

## O7 — Horizon: mesh+B-Rep hybrid (design-only until M4)

One deliverable in this program: **RFC 0007** (`docs/design/`), drafted
only after P-Class M4 merges its body taxonomy — mesh body as a body class
under RFC 0005's model; booleans across mesh×exact via the existing
co-refinement machinery (`operations/src/mesh_boolean.rs`) recast as a
*declared input class with disclosed quality*, the inverse of today's
fallback; tessellation-tolerance semantics at the mesh/exact seam.
No implementation before the RFC; no RFC before M4. The ledger row exists
so no session "helpfully" starts early.

---

## §W Waves — what runs when, for parallel sessions

Disjoint-file scheduling, same discipline as P-Class §4. `gh pr list`
before claiming anything (R6).

**Wave A — now (no P-Class collision):**

| Issue | Files | Size |
|---|---|---|
| O1.1a–c gauntlet | `tools/gauntlet` (new) | M+S+S |
| O1.3a fillet torture corpus | `operations/tests` | S |
| O1.4a validation properties | `io/step` | M |
| O2.1a–b RFC 0006 + math substrate | `docs/design`, `math` | M+M |
| O2.3a splitter inventory | read-only analysis | S |
| O3.1 benches | `math/algo/blend benches` (new) | S×3 |
| O4.1a–b facade + examples | `crates/remus` (new) | M+S |
| O4.4 error registry (e5b) | `operations` | S |
| O5.1a–b assemblies | `io/step`, `operations/assembly` | M+M |
| O6.1 docs skeleton, O6.4 contributing | `docs/book`, root | S+S |

**Wave B — after P-Class 2.4 (splitter files free; census strong):**
O2.1c–e (the variant ripple), O2.2 (conics, M2 track), O2.3b–d
(arrangement), O3.2 (spatial cache), O4.1c (wasm delegation), O4.2a–b
(publish dry-run), O1.2 (head-to-head, publishable numbers), O4.3
(Python), O5.2 (e3b), O5.3a (AP242 writer), O6.2 (playground).

**Wave C — after M4 / M5:** O1.3b (torture suite public), O5.3b (PMI
read), O7 (RFC 0007), O5.3c last.

**Owner-gated at any time:** O4.2c first publish · O4.3c PyPI · O6.2
hosting target · O6.3 outreach.

## §X Cross-program conflict table

The three places this program can collide with P-Class, and the rule:

| Files | P-Class owner | This program | Rule |
|---|---|---|---|
| `algo/src/builder/face_splitter/` | 2.4 | O2.3 | O2.3b+ waits for 2.4; O2.3a (read-only) any time |
| `algo/src/gfa.rs`, `pave_filler/` | M2 | O2.2 | O2.2 runs *inside* the M2 track, never parallel |
| `topology/src/` | M3.2 | O3.2 | additive files both sides; coordinate PRs, land M3.2 first if same-week |

Everything else in Wave A is new directories or io/operations files M2
does not touch.
