# Open Kernel Program

What it takes for Remus to be the best open-source B-Rep kernel of its kind —
the axes *beyond* the [P-Class program](p-class-program.md). P-Class makes the
kernel correct and capable; this program makes it provably best, adoptable,
and durable as an open-source project.

- **Drafted:** 2026-08-29, baseline `main` @ `d154e64`.
- **Issue-level plan:** [open-kernel-implementation.md](open-kernel-implementation.md)
  · ledger: [open-kernel-status.md](open-kernel-status.md).
- **Relationship to P-Class:** strictly complementary. Nothing here changes
  P-Class scope, ordering, or gates. Every pillar below is scheduled around
  P-Class's file footprint so parallel sessions don't collide (see §8).
- **Promotion authority** remains [capability-matrix.md](capability-matrix.md).
- **Competitive overlay:** [industrial-parity.md](industrial-parity.md)
  scores every capability against the reference kernel on two axes,
  defines the H5–H7 horizons and the leadership-claim discipline, and
  extends O1.2 into the full scorecard (O1.2d–f). It adds no scoreboard
  claim of its own; S1–S7 remain this program's public claims.

## §1 The competitive thesis

A 2026 survey of the field supports one strategic claim: **the
permissively-licensed, memory-safe, WASM-first exact B-Rep kernel niche is
unoccupied.**

- **The incumbent C++ kernel** (7.9/8.0) is improving but carries decades of debt in exactly the
  places users leave over: fillets that fail on legal input (the 10+ year
  "command not done" class), booleans slower and less stable than commercial
  kernels, a leaky per-entity tolerance model that makes heal-after-import
  mandatory, and an LGPL + CLA + paid-services posture. Its second life is
  entirely browser wrappers (its Emscripten JS wrapper, replicad, chili3d),
  all funneling through one aging C++ codebase.
- **truck** is the closest Rust competitor and is pre-industrial: fragile
  curved booleans, no blends worth the name. **Fornjot is dead** (wound down
  by its author), **CADmium stalled**, **SolveSpace's kernel** documents its
  own boolean failure modes. **Zoo/KittyCAD** is serious and funded — and
  closed-source, cloud/enterprise-gated.
- **Parasolid/ACIS** define the quality bar but are unlicensable for open
  ecosystems. What their customers actually pay for: blends that don't fail,
  tolerant modeling of imperfect imports, mesh+B-Rep hybrid modeling
  (Convergent), thread-safe parallelism, attribute/naming persistence, and —
  increasingly (Spatial's whole 2025–26 roadmap) — AP242 PMI interop.
- **Manifold** is the instructive open-source win: it displaced CGAL inside
  OpenSCAD in a year on two artifacts — a *guarantee* (always-watertight) and
  *reproducible 20–100× benchmarks*. Robustness proof beats feature count.
- **FreeCAD's topological-naming decade** is the demand proof for Remus's
  single most differentiated asset: the RFC 0002/0003 journal + persistent
  naming stack, which no open kernel ships natively.

Remus's structural advantages, today: Apache-2.0 with no CLA; zero unsafe and
no-panic as enforced lints (the incumbent's consumers wrap calls in segfault guards);
native WASM without Emscripten-class baggage; a shipped GCS constraint solver
(consumers keep gluing SolveSpace's solver onto the incumbent because kernels don't
have one); disclosed degradation (`FallbackPolicy` + `BooleanQuality` +
`approx_census`) where every other kernel is silent; and the naming/journal
stack.

**The winning position, in one sentence:** booleans and fillets that fail
loudly and rarely, *proven* on public real-model corpora; kernel-native
persistent naming; WASM-first APIs every incumbent-kernel wrapper is structurally
handicapped against; AP242 fidelity with analytic preservation; and a front
door (Rust, JS, Python) people can actually walk through.

## §2 Honest current-state deltas this program owns

From a code-level maturity survey (2026-08-29), the gaps that are *not* (or
only partially) on the P-Class books:

1. **No front door.** ~245 un-re-exported `pub fn` across 13 unpublished
   crates; no `remus` facade crate, no prelude, no session/document type; the
   12 `examples/` are all debug probes. WASM (299 methods) is the only real
   consumer surface. Nothing is published to crates.io or npm.
2. **STEP breadth stops at geometry.** Writer is AP203-only; assemblies exist
   in-memory but cannot round-trip (no NAUO); colors/names attributes exist
   in `topology::attributes` but aren't wired to STEP; PMI/GD&T is absent
   entirely.
3. **Import exactness ceiling:** `SURFACE_OF_REVOLUTION` and
   `SURFACE_OF_LINEAR_EXTRUSION` are read but lowered to NURBS permanently —
   `FaceSurface` has no native variants for them. Hyperbola/parabola edges
   are refused at the GFA entry (`reject_unsupported_curves`).
4. **The general splitter debt:** `face_splitter/special_cases.rs` is ~4,300
   lines of per-geometry topology repair; the roadmap's TERMINAL list names
   the missing primitive (a UV-space arrangement splitter) behind several
   closed-as-workaround cases.
5. **Performance story untapped:** rayon at exactly two call sites
   (tessellation); BVHs rebuilt from scratch at all seven query sites; zero
   benches in math/algo/blend/io (the hot inner loops are unmeasured); no
   SIMD anywhere.
6. **Robustness is proven in-repo only.** ~4,150 tests, 14 fuzz targets,
   fixtures, mutation testing — but no public, reproducible evidence a
   skeptic can run: no corpus scoreboard, no head-to-head benchmark, no
   conformance run.

## §3 Program at a glance

| ID | Pillar | Size | Starts |
|---|---|---|---|
| O1 | Proof of robustness — corpus gauntlet, head-to-head benches, fillet torture suite, STEP conformance | L | now (infrastructure; no P-Class overlap) |
| O2 | Exactness hardening beyond M2 — native revolution/extrusion surfaces, conic booleans, the general arrangement splitter | L | splitter after M2.4; surface variants schedulable now |
| O3 | Native performance — BVH caching, inner-loop benches, then M8's parallelism with a measured baseline | M | benches now; caching after M2 settles |
| O4 | Front door & distribution — `remus` facade crate, publishing, Python bindings, examples | L | facade/docs now; publish is an owner gate |
| O5 | Interchange depth — STEP assemblies, colors, AP242 writer, validation properties, PMI | L | assemblies/colors now (io-crate only) |
| O6 | Ecosystem & community — docs site, browser playground, second consumer, contribution posture | M | rolling |
| O7 | Horizon: mesh+B-Rep hybrid modeling | L | design only until M4 lands |

Standing rules R1–R8 from the P-Class program bind here too, with one
addition:

> **R9 — Public claims are reproducible claims.** Any number this program
> publishes (pass rate, speedup, conformance result) ships with the harness
> and corpus manifest that regenerates it. A benchmark a stranger cannot
> re-run is marketing, and marketing that can't be re-run backfires; the
> Manifold precedent is that reproducibility itself is what converts.

## O1 — Proof of robustness (L) — the credibility engine

The single highest-leverage pillar. Every kernel claims robustness; the open
ones that won *published* it.

### 1.1 Real-model corpus gauntlet at ABC scale (L)

Extends P-Class 8.5 (≥50 curated models, nightly) into the public headline
metric no open kernel currently has: **"% of the ABC dataset that imports,
validates, booleans against a probe body, tessellates watertight, and
round-trips STEP."** The ABC dataset (~1M real STEP models) is free
credibility; MAMBO/HexMe are the curated hard-case tiers. Staged: 1k-model
sample first, then 10k, trend-tracked per release. Every failure class feeds
a typed refusal or a fix — the same verify-or-refuse doctrine, at scale.

> **Exit gate:** scoreboard page regenerated nightly from a pinned corpus
> manifest; per-stage pass rates (read / validate / boolean / tessellate /
> round-trip) with failure taxonomy; every regression bisectable to a merge.

### 1.2 Head-to-head benchmark harness (M)

Reproducible comparisons against the incumbent (via its Python/JS
bindings), truck, and Manifold (mesh ops only), on booleans, fillets,
STEP read, and tessellation — wall-clock *and* correctness (volume error,
watertightness, silent-wrong-answer detection). The incumbent's 8.0 release markets 17–20%
boolean gains; the counter-position is not "faster on everything," it is
**"comparable or better speed, and never silently wrong."** Publish losses
too (R9); a benchmark that only reports wins converts nobody.

> **Exit gate:** `benches/vs/` harness runs from a clean checkout with pinned
> competitor versions; results page auto-generated; at least one
> Manifold-style headline number that survives outside scrutiny.

### 1.3 Fillet torture suite (M)

The incumbent's fillet-fiasco cases — band-consumes-face, band-meets-band,
radius-exceeds-support, vertex pileups — as a named public corpus. Every case
resolves to *built-and-verified* or *typed refusal naming the limit*. This is
M5's qualification work repackaged as the public artifact that targets the
incumbent's best-known weakness. Cheap once M5 lands; start the corpus now so
M5 issues qualify against it directly.

> **Exit gate:** corpus in-repo with per-case disposition; zero
> crash/silent-wrong outcomes; results published alongside 1.1.

### 1.4 STEP conformance alignment (M)

Align reader/writer with CAx-IF/MBx-IF Recommended Practices and run the
published test-round suites. Add validation properties (geometric checksums:
volume, area, centroid per solid) to the writer — the standard mechanism by
which receiving systems verify a translation, and cheap given `measure`
already computes all three.

> **Exit gate:** CAx-IF geometry test-round models round-trip with validation
> properties agreeing within declared tolerance; deviations from Recommended
> Practices documented as typed limitations.

## O2 — Exactness hardening beyond M2 (L)

M2 closes the boolean generality gap. These are the exactness gaps that
remain after it.

### 2.1 Native revolution & extrusion surfaces (L — the last big ripple)

`FaceSurface::{Revolution, LinearExtrusion}` as first-class variants, so
STEP's two most common non-quadric analytic surfaces stop being lowered to
NURBS at import (a permanent exactness loss the analytic-preservation
doctrine exists to prevent). This is a known two-part ripple job (~93
wildcard `_ =>` arms audited by hand per CLAUDE.md) — expensive, therefore
deliberate: after M2.4 settles the splitter, with the delegate-method surface
minimizing call-site churn. Evaluate/normal/project come free (profile curve
+ sweep rule); intersection arms start with the revolution×coaxial-anything
closed forms and refuse typed elsewhere.

> **Exit gate:** a STEP file with revolution/extrusion faces imports with
> zero NURBS lowering, booleans against a box via typed-or-exact paths, and
> re-exports the same surface entities; wildcard-arm audit checklist merged.

### 2.2 Conic edges through the boolean engine (S)

Promote the inherited-queue item: hyperbola/parabola edges currently refused
at `reject_unsupported_curves`. Cone×plane sections off-axis *are*
hyperbolas/parabolas; today those configurations route around the exact path.

> **Exit gate:** off-axis cone×plane cut carries exact conic section edges
> through fuse/cut/intersect; census rows move analytic.

### 2.3 The general UV-arrangement splitter (L — the debt retirement)

The named missing primitive behind several TERMINAL roadmap cases and the
structural answer to `special_cases.rs` (~4,300 lines of per-geometry
patches): a UV-space arrangement of section curves on a face — seam-aware,
pole-aware — from which loop reconstruction is generic. Not a rewrite:
build it as the new default with the special cases as the fallback, retire
patches one census row at a time. Schedule strictly after M2.4 (same files).

> **Exit gate:** ≥3 special-case paths deleted with their fixtures passing
> through the general splitter; TERMINAL entries re-opened against the new
> primitive; `special_cases.rs` line count monotonically falling under a
> tracked gate.

## O3 — Native performance program (M)

M8.2–8.4 gate regressions and parallelize tessellation/booleans. What's
missing is the *baseline* and the cheap structural wins.

### 3.1 Inner-loop benches for math/algo/blend (S — do first)

Criterion suites for SSI marching, pave-filler phases, NURBS
evaluation/fitting, CDT, and the blend walker — currently zero benches in
those crates while all five suites sit in operations. M8.2's gates need
these to exist to guard anything.

> **Exit gate:** hot loops named by a flamegraph of the 64-cut and gridfinity
> benches each have a criterion bench; wired into `bench-compare.sh`.

### 3.2 Topology-cached spatial acceleration (M)

All seven BVH consumers rebuild from scratch per query. Cache face-level BVH
on `Topology` with journal-driven invalidation (the evolution journal already
knows exactly which faces changed — a rare case where the naming stack pays a
performance dividend). Distance, classification, and boolean preflight all
get faster with zero API change.

> **Exit gate:** repeated-query benches (classify N points, N-body clash)
> show the rebuild eliminated; invalidation proven by a mutate-then-query
> correctness suite; determinism gates unchanged.

### 3.3 SIMD in NURBS evaluation (S, measured-first)

Only where 3.1's benches prove it matters, `wide`/portable-SIMD in basis
evaluation and surface grids; wasm128 variant behind a feature flag.
Explicitly not before 3.1 — no speculative vectorization.

> **Exit gate:** ≥1.5× on the named bench, native and wasm, or the item is
> closed as not-worth-it with the numbers attached.

## O4 — Front door & distribution (L)

The kernel is currently 13 internal crates with CI-enforced boundaries and
no way in. Adoption requires exactly three doors: Rust, JS, Python.

### 4.1 The `remus` facade crate (M)

One crate: curated prelude, a `Model`/session type owning a `Topology` plus
an `OperationContext`, builder-style operation entry points, and real
examples (bracket, flange, imported-STEP repair chain) replacing the
debug-probe examples directory. The WASM `BrepKernel` becomes a thin wrapper
over it — one behavior, two surfaces, which is also what native/WASM parity
(target characteristic 15) wants structurally.

> **Exit gate:** the README quickstart compiles as a doctest against
> `remus::prelude`; wasm kernel delegates to the facade with contract tests
> unchanged; boundaries script extended to the new crate.

### 4.2 Publishing pipeline (M — owner-gated)

crates.io for the workspace (the `= 0.1.0` exact-pin structure is already
publish-shaped), npm for the wasm package (currently a committed snapshot
consumed by path). Semver policy, MSRV policy (1.88 declared), release
automation on the existing release-checklist. **This pillar flips the
standing "Remus publishes nothing yet" posture — it is the one item in this
program that is an explicit owner decision, not an agent default.** A
sensible trigger: first publish after M2.4 lands (the census fallback rows
close), so the first public version leads with its strongest boolean story.

> **Exit gate:** `cargo add remus` and `npm i @remus/kernel` work from clean
> environments against a tagged release; release checklist automated to one
> command.

### 4.3 Python bindings (L)

PyO3 `remus-py` over the facade, wheels via maturin for the platform matrix.
Target audience: the CadQuery/build123d community, whose chronic pain is
incumbent-kernel binding distribution (OCP) — a `pip install` that just works is the
shortest path to a second serious consumer and their regression corpus.
Scope v1 to the facade surface (primitives, booleans, fillets, measure,
STEP, tessellate); no attempt at CadQuery API compatibility.

> **Exit gate:** `pip install remus-kernel && python -c "…fillet a box…"` on
> Linux/mac/Windows wheels; contract tests mirrored from the wasm suite.

### 4.4 Contract surface completion (S)

The deferred stable-error-code registry (e5b) and versioned operation
contract docs become public API documentation — the failure taxonomy is a
*feature* to advertise (SolveSpace's known-issues honesty converts users;
the incumbent's silence repels them).

> **Exit gate:** every public operation's error codes enumerated in rustdoc;
> e5b closed; docs site (O6) renders the registry.

## O5 — Interchange depth (L)

The commercial frontier moved to interop (Spatial's entire recent roadmap).
All io-crate work, near-zero collision with P-Class.

### 5.1 STEP assemblies (M — the blocker)

`NEXT_ASSEMBLY_USAGE_OCCURRENCE` read+write, mapping the existing in-memory
assembly module (hierarchy, transforms, BOM — already qualified) to product
structure. Today a multi-part model cannot round-trip at all; this blocks
any assembly-capable consumer including OpenZCAD's growth path.

> **Exit gate:** deep-hierarchy assembly round-trips STEP with transforms
> verified against the qualified in-memory composition; imported third-party
> assemblies (fixture corpus) produce correct trees.

### 5.2 Colors, names, attribute scope (S)

The queued e3b design doc, promoted: `COLOUR_RGB`/`STYLED_ITEM` chains wired
to the existing `topology::attributes` store (which already carries color
but only serializes via glTF/arena), name round-trip beyond the current
face/solid set, WASM accessors.

> **Exit gate:** e3b's own acceptance list; a colored named assembly
> round-trips STEP → arena → STEP bit-stable on the attribute payload.

### 5.3 AP242 writer, then PMI (L, staged)

Stage 1: AP242 schema output (reader is already schema-agnostic) — required
by modern PLM toolchains and a prerequisite for everything semantic. Stage
2: PMI/GD&T read into an attribute-anchored representation (datums, FCFs,
dimensions bound to persistent refs — the naming stack is precisely the
right anchor, and kernel-level PMI-to-topology binding is something the incumbent
does not offer cleanly). Stage 3: semantic PMI write. Each stage gated on
CAx-IF test rounds (1.4).

> **Exit gate (stage-wise):** AP242 files validate against the schema and
> import into two independent receiving systems; PMI entities bind to
> persistent refs that survive a direct edit (the demo no other open kernel
> can run).

## O6 — Ecosystem & community (M, rolling)

What separates a great engine from a great open-source project. Levers, in
observed order of effect:

- **Docs site (S):** mdBook + rustdoc; the capability matrix, stability
  ledger, and failure taxonomy published *as-is* — the honest-disclosure
  posture is a differentiator, not laundry. Architecture tour from the
  existing CLAUDE.md module map.
- **Browser playground (M):** wasm + render already exist; a hosted
  fillet-a-bracket-in-the-browser demo is the discovery moment (chili3d's
  reception proves the appetite; Remus skips its Emscripten tax). Feeds on
  O4.1's facade.
- **Second consumer (ongoing):** OpenZCAD is first-party; credibility needs
  an external one. Courting order matches O4: a replicad-class JS consumer
  (npm package is the ask), then CadQuery-class Python. Their regression
  corpora become Remus fixtures — the Manifold/OpenSCAD symbiosis.
- **Contribution posture (S):** no CLA (Apache-2.0 inbound=outbound, the
  direct contrast with the incumbent), CONTRIBUTING.md, labeled starter issues from
  the inherited queue, fork-provenance policy already documented.
- **Sustainability note:** solo-maintainer kernels die (Fornjot); the
  mitigations are the consumer symbiosis above and the corpus/CI machinery
  that lets autonomous agents keep quality flat between human attention.

No exit gate — rolling; measured by the §7 scoreboard.

## O7 — Horizon: mesh+B-Rep hybrid modeling (L, design-only until M4)

The Convergent-Modeling-shaped hole: Manifold proves open-source mesh demand,
Parasolid proves the product category, nothing open bridges them. P-Class
correctly defers facet bodies until M4's body taxonomy exists; this pillar
just keeps the option deliberately alive: RFC when M4 lands (mesh body as a
body class; booleans across mesh×exact via the existing co-refinement
machinery; disclosed quality class per the fallback policy — the *opposite*
of the current mesh fallback, which degrades exact geometry: here mesh is a
declared input class, not a failure mode). Not before M4. Not silently.

> **Exit gate (for the design stage only):** RFC drafted post-M4 with the
> body-taxonomy extension mapped; no implementation before it.

## §7 What "best of its kind" means, measurably

The program's definition of done — public numbers, each with its harness:

| # | Claim | Evidence |
|---|---|---|
| S1 | Robustness leadership | ABC-scale scoreboard published per release; pass rate and trend public; zero silent-wrong classes open |
| S2 | Fillets that don't fail | Torture suite: 100% built-or-typed-refusal, 0 crashes; side-by-side with the incumbent's dispositions |
| S3 | Honest speed | Head-to-head harness public with wins *and* losses; never-silently-wrong as the headline |
| S4 | Three working doors | `cargo add` / `npm i` / `pip install` each to first solid in <10 lines |
| S5 | Interchange trust | CAx-IF round-trip with validation properties; AP242 assemblies + attributes |
| S6 | Someone else ships on it | ≥1 external consumer in production with their corpus in Remus CI |
| S7 | The naming demo | Direct-edit + persistent-ref survival, in the browser — the capability no other open kernel can show |

The overlay's leadership claims LC1–LC13 are the measured form of S1–S7:
each names its scorecard column, its regression gate, and requires
published losses beside wins before it may be stated publicly.

## §8 Sequencing against P-Class

```
now (disjoint from M2):  1.1 corpus harness · 1.3 corpus collection · 3.1 benches
                         4.1 facade · 4.4 contracts · 5.1 assemblies · 5.2 e3b
                         6 docs site
after M2.4:              2.1 revolution/extrusion · 2.3 arrangement splitter
                         3.2 BVH caching · 4.2 publish (owner gate) · 1.2 head-to-head
after M5:                1.3 torture suite goes public with M5 dispositions
after M4:                7 hybrid RFC
rolling:                 1.4/5.3 conformance stages · 4.3 Python · 6 playground+consumer
```

File-footprint note for parallel sessions: O1 (new harness dirs), O4.1 (new
crate), O5 (io), O6 (docs) touch nothing M2 touches. O2.3 and O3.2 are
algo/topology and must wait their turn exactly as M3/M4 do.

## §9 Risks

- **Spreading thin.** P-Class is large and running. Mitigation: everything
  scheduled "now" above is infrastructure or io-layer — no pave-filler
  contention — and each item is independently shippable.
- **Premature publishing.** A 0.1 on crates.io is a support surface.
  Mitigation: the O4.2 owner gate, and the after-M2.4 trigger so v0.1's
  boolean story is its strongest.
- **Benchmark blowback.** Unfair or irreproducible comparisons cost more
  credibility than they buy. Mitigation: R9 — publish harness, pins, and
  losses.
- **Corpus licensing.** ABC models carry per-model licenses; the scoreboard
  publishes *aggregates and manifests*, never redistributed models.
- **PMI scope creep.** AP242 semantic PMI is a swamp. Mitigation: staged
  gates, read-before-write, bind-to-naming as the differentiator rather
  than breadth.
