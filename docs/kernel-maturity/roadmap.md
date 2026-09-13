# Remus master roadmap

**The single source of truth for priorities, dependencies, implementation status,
and remaining work.** Start here, choose one bounded item, and update its row in
the PR that changes it. Specifications define scope and acceptance; audits retain
source-pinned evidence. Neither maintains a competing work queue.

**Reconciled:** 2026-09-12 against `main` at `dbb227610933985440f18eb38adb24323cb6c391`.
Open PRs below are a dated snapshot, not completion evidence or permanent ownership.
This documentation review preserves existing qualification limits; it does not
re-run geometry suites or certify the current scheduled jobs.

## Start here

- [Current priorities](#current-priorities) and [in-flight work](#in-flight-work).
- [Dependencies and horizons](#dependencies-and-horizons), including the release gates.
- [P-Class implementation register](#p-class-register): M2–M8 capability work.
- [Open Kernel implementation register](#open-kernel-register): O1–O7 proof, APIs and adoption.
- [Bridge register](#bridge-register): correctness, consumer and cross-cutting work.
- [Performance work packages](#performance-work-packages): all 111 packages, dependencies and owners.
- [Decisions and exclusions](#decisions-and-exclusions), [consolidation map](#consolidation-map),
  and [session workflow](#session-workflow).

### How to read status and IDs

`Pending` / `Open` means the registered scope has no recorded completion;
`Partial` identifies a landed subset and names the remainder. `Implemented`,
`Merged`, `Complete` and `Done` retain their original bounded qualification
claims: read the limitations in the same row. None grants capability promotion
or competitive parity beyond the evidence. An open PR is **in flight**, never
Done. `Proposed` performance packages require baseline and dependency review;
it does not mean no relevant implementation already exists.

IDs stay compatible with existing issues and PRs. Use **P-Class 2.5** for a
capability issue, **B4** for a bridge row, **EXIT-B4** for the former P-Class
exit benchmark B4, and **PERF-B04** for performance package B04. The generated
performance CSV/JSON retain their original short IDs and schema. Only this
page owns implementation state; those files are generated work-spec exports.
The [capability matrix](capability-matrix.md) still governs qualification and
the [stability matrix](../production-readiness/stability-matrix.md) governs
public labels. Consolidating planning does not change either authority.

## Current priorities

Order by correctness risk, dependency unlock, consumer impact and measured cost.
The lanes permit independent work after a live file-overlap check; they are not
permission to launch several changes in one session.

| Priority / lane | Next bounded work | Dependency / completion boundary |
| --- | --- | --- |
| 1 — Correctness and proof | Finish/review the in-flight B26 native/fuzz oracles and B24 wildcard ratchet. Route each reproduced defect to its owning geometry row. | #410 and #417 share B26; #422 owns the current B24 slice. Tests-only campaigns must retain minimized failures rather than silently turn into kernel fixes. B19's four other fuzz targets remain separate. |
| 2 — Consumer APIs | B16: first reconcile the consumer use of `unifyFacesChecked`, then one missing topology query; O4.7 for one further typed operation family. | A kernel binding does not close its adapter-deletion acceptance criterion. Broader B18 history work follows the existing B16 three-row consumer milestone unless a specific defect requires it. |
| 3 — Geometry | P-Class 2.4 integration qualification and 2.5's named curved-NURBS slices; remaining 2.6/2.7 cells; B7/B8 as their witnesses require. | #416 is an in-flight 2.5 math slice. B4's four planar gaps landed in #398; its curved-face refusal remainder belongs with M5, not another planar-trimmer rewrite. Serialize GFA/splitter work. |
| 4 — Interactive performance | B28 / PERF-N04: qualify the landed fused NURBS evaluation and the in-flight scratch/budget slices. Then extend profiles (PERF-M04/M05/M07), calibrate gates (PERF-M03), and select one measured cost. | #411 is landed; #413/#418 are in flight. Preserve source/package provenance and numerical behavior. O3.1a already provides a maintained baseline; extend it. |
| 5 — Independent qualification | One B6 family, B10, B17 fixer, B19 target, or O1.5 parity slice. | Independent oracle, explicit unsupported cells, direct/batch WASM where public; no family promotion from one fixture. |
| 6 — Foundation work with clear gates | O4.4 error registry, O4.6 serialization policy, O5.1/O5.4 assembly exchange, B29 transaction design, O3.2 prepared queries. | Respect the dependency table and choose a file-disjoint slice. Cache and transaction proposals require invalidation/rollback evidence before reuse. |

## In-flight work

Snapshot from GitHub on 2026-09-12. Re-fetch before claiming or changing any row.
Several PRs edit this document or the retired status files: port their status
updates to the owner rows here during integration, retaining their geometry/tests.
No PR check result or reviewer approval is inferred from this inventory.

| PR | Scope / owner | Inspected head |
| --- | --- | --- |
| [#410](https://github.com/esaueng/remus/pull/410) | B26 native invariant campaign | `865f0c1fdff6` |
| [#413](https://github.com/esaueng/remus/pull/413) | B28 strict-validation work probes (budget claim under review) | `e401170daad4` |
| [#416](https://github.com/esaueng/remus/pull/416) | P-Class 2.5 plane/NURBS section seeding; recheck prior closure finding | `3a3016360573` |
| [#417](https://github.com/esaueng/remus/pull/417) | B26 fuzz oracles; unresolved findings at the inspected head | `1573b516b00c` |
| [#418](https://github.com/esaueng/remus/pull/418) | B28 / PERF-N04 derivative scratch reuse | `a27d4eb88698` |
| [#419](https://github.com/esaueng/remus/pull/419) | Committed WASM candidate; source/artifact gate, not capability closure | `4fd3a7fc03f9` |
| [#422](https://github.com/esaueng/remus/pull/422) | B24 volume wildcard audit and ratchet; CI wiring remains separate | `f97d506b2ad2` |
| [#423](https://github.com/esaueng/remus/pull/423) | Roadmap-only status reconciliation; incorporated below, superseded in scope by this consolidation | `a9fac6bd0050` |

### Prior review findings to carry through integration

[Reconciliation PR #423](https://github.com/esaueng/remus/pull/423) at
`a9fac6bd0050` records the findings below. Preserve them when integrating its
roadmap-only update into this consolidation. They are attributed review evidence,
not fresh reproductions by this documentation change. A newer head needs a new
verdict and injected regression evidence; green CI alone does not resolve a finding.

| Owner / reviewed head | Recorded finding / remaining gate |
| --- | --- |
| B26 / #417 `1573b516` | Successful non-finite measurements were classified as refusal; a base-refused root skipped nudge comparison; coincident endpoints bypassed closed-curve domain authority. Retain all three injected witnesses before crediting the fuzz slice. |
| P-Class 2.5 / #416 `e1b3a656` | Closure heuristic closed an actually open near-full tube section. Require carrier-domain/continuation evidence and the open-tube regression. The newer head in the inventory is not covered by that old verdict. |
| B28 / #413 `e401170d` | Probes re-integrated shells after validation rather than observing actual work. Collect actual integration counters, cover skipped orientation, and distinguish work identity from an enforced latency budget. |
| B24 / #422 predecessor | Exhaustive volume matches and a 166-arm ratchet were proposed; other dense-file conversions and immutable workflow-source/caller wiring remain part of the parent acceptance. |
| B26 / #410 `865f0c1f` | New refusal-masking and per-family success-count fixes require re-review and checks for that head; previous green checks do not transfer. |
| B28 / #418 `937cae61` | The reviewer reported 33,852 derivative vectors matching bit-for-bit. This comparison belongs to that earlier head and is not the full performance or current-head qualification gate. |

The same dated reconciliation reports [Corpus Gauntlet success on September 12](https://github.com/esaueng/remus/actions/runs/34681892888),
[Fuzz Smoke success on September 6](https://github.com/esaueng/remus/actions/runs/34020633257),
and [Mutation Testing failure on September 6](https://github.com/esaueng/remus/actions/runs/34017122331).
#381 triaged the nine mutation survivors; triage is not a subsequent successful
scheduled run. Refresh these proof jobs before a capability or release claim.
Package #408 (2.130.15, source `4e6499fc`) predates #409/#411; candidate #419
(2.130.16, source `b1a0bf12`) does not include later unmerged optimizations.

### Reconciliation findings

- The former H0 said no PRs were open and also called B22 unfixed. The CI caller
  now cancels superseded PR runs only; B22 remains Done. The separate pin repair #412 is now merged at `dbb22761` and does not reopen that contract.
- B4's planar completion is #398, replacing its dangling “this PR” reference.
  The two curved-face end-to-end refusals remain partial scope.
- #406 delivered O3.1a's maintained baseline; #411 delivered fused position and
  first-partial evaluation in NURBS quadrature. Neither completes the 111-package
  performance program. Older timings remain tied to their original artifacts.
- Native tessellation already contains parallel paths. P-Class 8.3 now records
  that partial implementation rather than implying it must be built from zero;
  broader deterministic qualification and supported WASM execution remain open.
  P-Class 8.5 likewise reuses the landed O1.1 corpus infrastructure while retaining
  its unproven assembly-operation and regression-bisection gates.
- Old H4 counts, scheduled-job verdicts, 8 MiB package figures and market
  assertions are not current readiness proof. The previous roadmap snapshot is
  retained in [campaign history](campaign-history.md#roadmap-snapshot-before-master-consolidation).
  Recompute release evidence on the proposed release head.

## Dependencies and horizons

The horizon labels are capability gates, not dates or versions of the current
committed package. In particular H4's “v1.0” means the first stable public-publish
milestone; the existing 2.x package numbering is a different release channel.

| Horizon | Work and prerequisite | Exit / next gate |
| --- | --- | --- |
| H1 — Current | The priority lanes above, preserving landed M2/M4/M5/M6 subsets. | Qualify one bounded slice at a time; close only its declared row scope. |
| H2 — Generality | Finish 2.4 integration; then M3.4–3.6, O2.1c–e and O2.3b–d. O2.2 stays inside the M2 lane; O2.4 follows 2.6. | Curved intersection/arrangement and tolerant modeling satisfy their specification gates. M4.1–4.7 already have bounded implementations; 4.8 remains. |
| H3 — Modeling depth | M5 remainder, M6 generalization and M7. Keep 7.4 early for general re-limitation; the bounded 6.1 implementation already exists. | Imported-body edits, blends, offsets and surfacing have both positive oracles and typed boundary refusals. O1.3b follows M5; O5.3c follows PMI read; O7 is design-only after the body-model gate. |
| H4 — Stable public release | All release gates below; O4.2c remains an explicit publish decision. | Qualified release head, reproducible evidence, reviewed capability bounds. |
| H5 — Core modeling parity | H4 plus the full authored-modeling gates below. | Equivalent-quality results with the O1.2f baseline pinned. |
| H6 — Industrial interchange and corpus parity | H5 plus real supplier-data and resource gates below. | Published corpus, interchange, lifecycle and concurrency evidence. |
| H7 — Demonstrated leadership | At least five LC1–LC13 claims meeting the evidence discipline, including two correctness claims (LC1–LC4) and one browser claim (LC7–LC9). | Repeatable independent evidence, equivalent quality, published losses and ongoing regression gates. See the [claim definitions](industrial-parity.md#7-leadership-claims-and-the-discipline-they-must-meet). |

### Scheduling constraints

- O2.3b–d and 2.4/2.5 share the splitter. An integration slice waits for its
  prerequisite geometry contract; do not run overlapping changes independently.
- M3 tolerance integration and M2 share GFA acceptance bands. Stop and re-stage
  if integration destabilizes the qualified boolean matrix.
- O3.2 caching follows operation-local preparation and complete mutation/restore
  invalidation. O3.4 / PERF-D01 follows that substrate and shared-edge dependency
  planning (PERF-D03). B29 local transactions preserve all failure-atomicity and
  stale-handle contracts. Compaction (8.6 / PERF-T06) follows its versioned design.
- Native parallelism (8.3/8.4) follows boundary ownership and deterministic commit
  order. WASM threads (8.7 / PERF-W07) additionally need the consumer deployment
  decision. SIMD is a measured experiment, not a prerequisite for ordinary work.
- Original Open Kernel wave labels remain in the register as dependency groups:
  A = potentially independent foundation work, B = integration after its named
  prerequisites, C = later publication/PMI/hybrid scope. A is not proof of readiness;
  check the individual specification and live file ownership.
- O5.1 assembly exchange and O5.4 occurrence identity are coordinated stages.
  O5.2 attributes precedes the full O5.3 AP242/PMI claim. Publishing, hosting and
  outreach retain the explicit decisions listed below.

### H4 release gates

No completion counts are asserted by this consolidation. Each release candidate
must attach a fresh evidence checklist for these four gates:

| Gate | Required evidence |
| --- | --- |
| Modeling | EXIT-B1–EXIT-B5 below pass as permanent integration tests. |
| Adoption and proof | S1–S7 below have reproducible published evidence. |
| Capability | No unresolved cells remain in the capability matrix’s Known Unsupported-untyped / Partial inventory; attach evidence for every disposition. Do not silently weaken this release gate. |
| Remaining work | Every unfinished bridge row is closed or explicitly re-triaged for that release; a later horizon is not an implicit waiver. |

### P-Class exit benchmarks

The `EXIT-` prefix disambiguates these scenarios from bridge work; their scope
is unchanged from the former P-Class §5.

| | Scenario | Exit | Milestones |
|---|---|---|---|
| EXIT-B1 | Import two real freeform STEP bodies, fuse, fillet the intersection seam, export STEP | Re-import matches: watertight, valid, volume stable, names round-trip | M2 + M5 |
| EXIT-B2 | A gappy real-world import booleans correctly with zero heal invocations | Result tolerances disclosed and bounded by the context cap | M3 |
| EXIT-B3 | Shell a curved thin-wall part at a thickness that folds the inner offset | Self-intersection excised, valid result, volume vs. mesh oracle | M2 + 5.7 |
| EXIT-B4 | Direct-edit an imported body: move a boss across a filleted, holed plate | Neighbors re-limit; persistent references resolve Bound afterward | M6 |
| EXIT-B5 | The full boolean qualification suite at model scales 1e-5 through 1e6 | Exact or typed refusal at every scale — never a silent wrong volume | 2.6 |

### Open Kernel public claims

| # | Claim | Evidence |
|---|---|---|
| S1 | Robustness leadership | ABC-scale scoreboard published per release; pass rate and trend public; zero silent-wrong classes open |
| S2 | Fillets that don't fail | Torture suite: 100% built-or-typed-refusal, 0 crashes; side-by-side with the incumbent's dispositions |
| S3 | Honest speed | Head-to-head harness public with wins *and* losses; never-silently-wrong as the headline |
| S4 | Three working doors | `cargo add` / `npm i` / `pip install` each to first solid in <10 lines |
| S5 | Interchange trust | CAx-IF round-trip with validation properties; AP242 assemblies + attributes |
| S6 | Someone else ships on it | ≥1 external consumer in production with their corpus in Remus CI |
| S7 | The naming demo | Direct-edit + persistent-ref survival, in the browser — the capability no other open kernel can show |

### H5 — Core modeling parity (authored-from-scratch domain)

Purpose: prove parity across the in-scope authored-modeling domain.

1. The [competitive crosswalk](industrial-parity.md#5-competitive-crosswalk) is complete: every in-scope row has a non-`Unknown`
   competitive state and an owner, and the crosswalk ownership rows are in this register.
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
   JS/WASM contract (O4.4 registry and O4.7 typed results), and the planned Python surface (O4.3a/b)
   passing the mirrored contract suite.
8. Parity measurements published for the modeled-from-scratch scenarios
   (W1, W2, W5, W7) with O1.2f's baseline pinned and the [absolute gates](industrial-parity.md#34-absolute-gates-need-no-baseline)
   green; numeric bands: `[locked by O1.2f]`.

### H6 — Industrial interchange and corpus parity

Purpose: comparable outcomes on real supplier data, assemblies, and large
models.

1. W3 passes: dirty STEP models are diagnosed, tolerated (M3.5, EXIT-B2) or verified-repaired (B1), operated on (M3.4), and re-exported
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

## P-Class register

Scope and per-issue exit criteria: [capability specifications](p-class-program.md).
All 58 migrated issue rows retain their IDs and evidence. Issue 2.0's original
measurement baseline is `39c7a7b7ccbfc746ed7d9e9b8f156d54d6cfe090`.

<details>
<summary>M2 — General curved booleans · 15 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-2-0a"></a>2.0a Measurement and semantic ratchet | Merged | [#120](https://github.com/esaueng/remus/pull/120) |
| <a id="p-2-0b"></a>2.0b Missing writers, invariants, oracles, and census | Complete — operations and phase-FF contributions | [#122](https://github.com/esaueng/remus/pull/122) + [#125](https://github.com/esaueng/remus/pull/125) |
| <a id="p-2-0c"></a>2.0c Reader migration and seam-safe validation | Complete — the staged reader migration is at zero (132 → 0), and CI strictly validates both oriented cylinder-seam pcurves on an exact boolean output with non-vacuous counts | [#154](https://github.com/esaueng/remus/pull/154) + [#159](https://github.com/esaueng/remus/pull/159) + [#162](https://github.com/esaueng/remus/pull/162) + [#165](https://github.com/esaueng/remus/pull/165) + [#169](https://github.com/esaueng/remus/pull/169) + [#172](https://github.com/esaueng/remus/pull/172) + [#175](https://github.com/esaueng/remus/pull/175) |
| <a id="p-2-0d"></a>2.0d Topology-owned atomic boundary mutation | Complete — all 30 measured production direct mutations are migrated behind two preflighted topology APIs; the ratchet requires zero and checkpoint rollback preserves boundary, pcurve, and derived-handle state | [#176](https://github.com/esaueng/remus/pull/176) |
| <a id="p-2-0e"></a>2.0e Physical Loop/Coedge p-curve authority | Merged — Face boundary order and per-use pcurve/winding storage are authoritative Loop/Coedge state; arena v3 round-trips both seam branches while v1/v2 derive them compatibly | [#179](https://github.com/esaueng/remus/pull/179) |
| <a id="p-2-0f"></a>2.0f STEP per-use deterministic round-trip | Merged — STEP import binds positioned pcurves to exact coedge uses, preserves analytic surface frames and periodic winding, and fails atomically on count/endpoint mismatch; export is deterministic and refuses inconsistent per-use authority | [#182](https://github.com/esaueng/remus/pull/182) |
| <a id="p-2-0g"></a>2.0g Integration, zero gate, corpus, and docs | Merged — whole-topology ownership/seam diagnostics run across exact boolean, arena-v3 rollback, external 48-pcurve STEP, and WASM paths; the 132-reader and 30-mutation gates remain zero, unsafe wire mutators are deprecated, and the read-only compatibility facade is retained behind its measured public-API deletion gate | [#188](https://github.com/esaueng/remus/pull/188) |
| <a id="p-2-1"></a>2.1 Honest-failure hygiene | Merged — unsupported phase-FF pairs and every pcurve UV-projection fallback fail with pinned diagnostics instead of substituting empty sections, zero UV, or a NURBS midpoint | [#129](https://github.com/esaueng/remus/pull/129) + [#194](https://github.com/esaueng/remus/pull/194) |
| <a id="p-2-2"></a>2.2 Sphere in general position | Merged — transversal equal-radius sphere×sphere fuse, cut, and intersect retain analytic sphere patches and pass closed-form volume, classification, manifold-mesh, and WASM exact-path gates | [#199](https://github.com/esaueng/remus/pull/199) |
| <a id="p-2-3"></a>2.3 Steinmetz ellipses | Merged — perpendicular equal-radius cylinder×cylinder intersection retains six analytic cylinder patches on eight authoritative ellipse arcs, matches `16/3·r³`, and passes native, manifold-mesh, census, and WASM exact-only gates | [#205](https://github.com/esaueng/remus/pull/205) |
| <a id="p-2-4"></a>2.4 Quadric × quadric transversal | Partial — 2.4a and 2.4b merged, staged independently: 2.4a emits every bounded sphere seam-arrangement cell for exact box ∪ sphere; 2.4b emits and tessellates the complementary torus-notch band for exact torus ∩ box; the bounded off-axis cone–sphere matrix qualifies all three operators at three scales and two placements; the additional sphere–cylinder matrix qualifies all three operators at three scales and two placements with workspace/census/installed-WASM verification; the bounded torus–sphere matrix now passes all three operators at three scales and two placements with direct/installed-WASM coverage, while an oversized witness retains typed budget refusal; general quartic arrangements/integration remain | [#206](https://github.com/esaueng/remus/pull/206) + [#207](https://github.com/esaueng/remus/pull/207) |
| <a id="p-2-5"></a>2.5 NURBS × NURBS booleans | Partial — first witnesses named by #396 (2026-09-11): a NURBS carrier whose control net is coplanar is now recognised and cut as the exact plane through the face-face, edge-face, and face-info phases (`helpers::planar_nurbs_as_plane`, `FaceExtent::Nurbs` chart-polygon extent, `emit_open_curve_windows`), so `convert_to_bspline` boxes fuse, measure, and tessellate exactly (`crates/operations/tests/converted_bspline_measure.rs`). Genuinely curved NURBS faces deliberately keep the old paths: no trimmed extent in phase FF (sections clip to the carrier and read as floating loops) and no transversal edge-face crossing detection (the unsigned sampler cannot see them); applying the planar machinery to the hammer-holder freeform body paved 12,408 noise crossings. Separately, the SSI marcher's plane×cylinder sections carry a 5e-5 LSPIA fit error beyond the 1e-5 weld (math-layer, sidestepped by the exact arm). Those three are the next slices; general NURBS×NURBS remains unqualified | [#396](https://github.com/esaueng/remus/pull/396) |
| <a id="p-2-6"></a>2.6 Scale-relative band audit | Partial — the through-tool matrix is exact across all three operators, twelve scales from 1e-5 through 1e6, and two placements; straight-edge junction refinement and local planar weld/closure bands are repaired; 36 anisotropic box/tool cells retain local material; curved and remaining parameter-band audits, further anisotropic families, and stricter rotated world-volume precision stay open | [Audit](scale-band-audit.md) |
| <a id="p-2-7"></a>2.7 Tangency and sliver contacts | Partial — merged bounded 162-cell scale/placement contract: 120 exact results and 42 typed refusals. General exact tangency and sliver construction remain unqualified | [#307](https://github.com/esaueng/remus/pull/307) |
| <a id="p-2-8"></a>2.8 OperationContext budgets and cancellation | Partial — boolean/SSI cancellation and all six SSI work budgets are direct/batch WASM-callable; parameter-space tolerance and wider adoption remain | [PR #138](https://github.com/esaueng/remus/pull/138) + [PR #147](https://github.com/esaueng/remus/pull/147) + [PR #160](https://github.com/esaueng/remus/pull/160) + [PR #202](https://github.com/esaueng/remus/pull/202) |

</details>

<details>
<summary>M3 — Tolerant modeling · 6 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-3-1"></a>3.1 RFC 0004 | Merged — staged per-entity tolerance semantics, authority, growth, serialization, and disclosure contract | [#126](https://github.com/esaueng/remus/pull/126) |
| <a id="p-3-2"></a>3.2 Topology substrate | Merged — RFC 0004 Stage 1: validated setters, vertex-ball/edge-tube validators, context cap, journal recordability, and byte-stable legacy arena round-trip | [#148](https://github.com/esaueng/remus/pull/148) |
| <a id="p-3-3"></a>3.3 Predicate plumbing | Merged — EE crossing/AABB, forced EE overlap, pave-vertex lookup, VE incidence, and SameParameter/SameRange validation honor declared entity tolerance while default bands and the 51-row approximation census remain unchanged | [#208](https://github.com/esaueng/remus/pull/208) |
| <a id="p-3-4"></a>3.4 GFA integration | Pending | — |
| <a id="p-3-5"></a>3.5 Import and sew integration | Pending | — |
| <a id="p-3-6"></a>3.6 Downstream disclosure | Pending | — |

</details>

<details>
<summary>M4 — Body taxonomy · 8 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-4-1"></a>4.1 RFC 0005 | Merged — staged solid/sheet/wire/general-body semantics, side-of sheet classification, Compound-first cellular results, STEP mapping, and evolution contract | [#127](https://github.com/esaueng/remus/pull/127) |
| <a id="p-4-2"></a>4.2 Sheet bodies first-class | Implemented — body-class validation, transactional construction, area/bounds/center, typed volume refusal, boundary-preserving tessellation, arena-v4 roots, and direct/batch WASM are joined by deterministic `SHELL_BASED_SURFACE_MODEL` exchange over open or closed shells; the trimmed-NURBS implementation exit witness is green | [#209](https://github.com/esaueng/remus/pull/209) + [#210](https://github.com/esaueng/remus/pull/210) + [#211](https://github.com/esaueng/remus/pull/211) + [#212](https://github.com/esaueng/remus/pull/212) + [#213](https://github.com/esaueng/remus/pull/213) |
| <a id="p-4-3"></a>4.3 Split solid by sheet | Implemented — GFA uses a first-class cylindrical sheet as a non-volumetric face-set tool; the resulting Compound contains two deterministic, individually valid cells whose closed-form volumes reconstruct the input, with native/direct/batch WASM parity and typed refusals outside the bounded subset | [#214](https://github.com/esaueng/remus/pull/214) |
| <a id="p-4-4"></a>4.4 Trim sheet by solid / sheet × sheet | Implemented — validated keep-inside/keep-outside solid trims plus effective-normal one-way and strict mutual planar sheet trims have native/direct/batch WASM parity; six boundary-trimmed sheets sew into a deterministic valid six-face solid whose exact volume matches `make_box`, while curved and multi-face sheet pairs remain unqualified | [#215](https://github.com/esaueng/remus/pull/215) + [#216](https://github.com/esaueng/remus/pull/216) |
| <a id="p-4-5"></a>4.5 Imprint | Implemented — a planar solid tool splits target faces without discarding material; the new validated solid preserves exact volume, journals only construction-derived Modified/Generated/Preserved events, resolves an anchored split face BoundMany, matches direct/batch WASM, and refuses unqualified configurations transactionally | [#217](https://github.com/esaueng/remus/pull/217) |
| <a id="p-4-6"></a>4.6 Multi-region boolean output | Implemented — exact two-solid booleans return a Compound of independently validated regions with deterministic cavity assignment and total per-region construction lineage; bounded pairwise-disjoint Compound operands add member-preserving fuse, distributed intersect, and distributed single-tool cut with native/direct/batch WASM parity. Intersecting-member fuse and multi-tool cut fail closed pending recursive lineage composition | [#218](https://github.com/esaueng/remus/pull/218) + [#219](https://github.com/esaueng/remus/pull/219) |
| <a id="p-4-7"></a>4.7 Wire bodies | Implemented — body-level length and existing copy/transform semantics are joined by additive arena-v5 standalone wire roots plus validation-gated closed-planar wire sweep; native/direct/batch WASM match exact perimeter and prism-volume oracles, while open and non-planar profiles refuse transactionally | [#222](https://github.com/esaueng/remus/pull/222) |
| <a id="p-4-8"></a>4.8 N-ary and mixed-dimensional General Fuse | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-2.2, IP-3.3, IP-3.4) | — |

</details>

<details>
<summary>M5 — Blend depth · 9 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-5-1"></a>5.1 Variable-radius qualification | Implemented — standard-law whole-domain bounds and typed collapse/local-limit refusals guard every walker station; the straight-edge perpendicular-plane linear band matches its analytic surface and closed-form volume, while S-curve samples preserve radius and both support tangencies. Opaque custom callbacks are preserved and station-checked rather than endpoint-linearized, but arbitrary between-sample certification and trimmed-solid assembly remain explicitly unqualified | [#226](https://github.com/esaueng/remus/pull/226) |
| <a id="p-5-2"></a>5.2 Curved-support blends | Implemented — constant-radius closed rims on qualified cylinder/cone, cylinder/sphere, and cone/cone supports assemble exact toroidal shoulders where provable and periodic walking-NURBS bands otherwise; unsupported support combinations and closed legacy spines fail typed. The cross-drilled cylinder/cylinder rim now refuses wrong-side material addition after #278 winding repair; correct-side assembly remains unqualified | [#228](https://github.com/esaueng/remus/pull/228) |
| <a id="p-5-3"></a>5.3 General vertex blends | Implemented — same-radius planar N-way corners with one connected material-side orientation produce analytic sphere caps, cylindrical stripes, and trimmed ellipse runouts with native/direct/batch WASM and G1/watertightness witnesses; mixed-side, non-planar, and variable-radius corners remain unqualified | [#231](https://github.com/esaueng/remus/pull/231) |
| <a id="p-5-4"></a>5.4 Setbacks | Implemented — physical straight-spine setbacks crop variable S-curve bands to a stationary common-radius planar corner ball; the three-edge exit witness pins result stations, G1, topology, mesh/volume, census, and direct/batch WASM parity, while incompatible declarations refuse transactionally | [#232](https://github.com/esaueng/remus/pull/232) |
| <a id="p-5-5"></a>5.5 Overflow and cliff handling | Merged — v2 fillets stop transactionally at planar support boundaries, inner-loop obstacles, closed-rim wall exhaustion, paired bands consuming one wall, and inward cap collapse with typed edge/face/requested/available metadata and stable native/WASM parity; actual rollover remains unqualified pending 6.1 | [#235](https://github.com/esaueng/remus/pull/235) |
| <a id="p-5-6"></a>5.6 Face-face blends and hold lines | Merged — disjoint-edge convex planar face selections with transversal carriers produce a new validated exact cylindrical Sheet; a prescribed complete contact segment is verified analytically, with scale/translation, independent area, direct/batch WASM, and transactional typed-refusal witnesses. Multi-face, curved/holed-support, partial-hold, and support-trimming cells remain unqualified | [#236](https://github.com/esaueng/remus/pull/236) |
| <a id="p-5-7"></a>5.7 Offset self-intersection removal | Merged — a closed hole-free straight-edged uniform-width six-edge orthogonal L-prism may excise one fully collapsed disconnected inner component after retained-profile, cap-inversion, prismatic-topology, manifold, containment, and construction-proven generated-face proofs; the L-bracket witness pins both sides of the collapse boundary, exact/mesh volume, scale/translation, WASM parity, exact evolution, and transactional connected-fold refusal. Other profiles, partial, multiple, holed, curved, and general intersecting cells remain unqualified | [#237](https://github.com/esaueng/remus/pull/237) |
| <a id="p-5-7b"></a>5.7b General offset self-intersection removal | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-5.1, IP-5.3, IP-5.4); program exit benchmark B3 depends on it | — |
| <a id="p-5-8"></a>5.8 Blend rollover through re-limitation | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-4.2, IP-4.4, IP-4.7); unblocked by 6.1 (#238) and B4 | — |

</details>

<details>
<summary>M6 — Direct modeling · 6 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-6-1"></a>6.1 Replace-surface re-limitation | Merged — one planar support or one coaxial inward-facing bore cylinder re-limits exactly against planar/cylindrical neighbors while preserving topology and a total face map; changed conic trims and every result coedge p-curve are rebuilt from authoritative domains. Tilted-cap and doubled-bore witnesses pin validation, watertightness, independent volume, scale/translation, and typed face/edge rollback. A bounded outward quarter-wall extension passes native, packaged WASM, and full-workspace verification in the [partial-cylinder work](../roadmap/partial-cylinder-resize.md). Merged #338 adds construction face/edge/vertex history and direct/batch `replaceSurfaceJournaled`; see the [audit](evolution-audit.md) for qualification evidence. Surface-type changes, general bosses, NURBS supports, and topology-changing edits remain later cells | [#238](https://github.com/esaueng/remus/pull/238) |
| <a id="p-6-2"></a>6.2 Generalized move / rotate / offset face | Partial — holed planar supports move through uniquely attributable constant-radius analytic blend bands with exact re-limitation, total construction face evolution, transactional journal history, scale/translation coverage, and direct/batch WASM parity. Inward coaxial bore moves now reuse 6.1. Rotation, lateral relocation, outward cylinders, surface-type changes, and ambiguous complex blend regions remain unqualified | [#257](https://github.com/esaueng/remus/pull/257) |
| <a id="p-6-3"></a>6.3 Curved delete-face-and-heal | Partial — complete toroidal rim-band deletion reuses exact analytic support reconstruction; complete boss wounds on cylindrical inner wires are also qualified with exact construction, analytic/mesh volume, transformed scale coverage, face history, rollback, and WASM parity. General outer-wire extension, partial bands, and rim-crossing boss wounds remain pending | [#274](https://github.com/esaueng/remus/pull/274), [#276](https://github.com/esaueng/remus/pull/276) |
| <a id="p-6-4"></a>6.4 Curved-face draft | Pending | — |
| <a id="p-6-5"></a>6.5 Journaled direct edits | Partial — #338 integrates qualified F/E/V history for moves, replacement, cylindrical radius edits, planar draft, defeature and verified healing pipelines. Single cylindrical blend resize on planar supports retains total F/E/V history, including explicit merges/deletions for zero-radius removal. Broader blend resize, ambiguous correspondence and later 6.x geometry remain open; B18 covers other families. Native shell/split and boolean/pattern rollback corrected during the audit | [#338](https://github.com/esaueng/remus/pull/338), [coverage and remaining gates](evolution-audit.md) |
| <a id="p-6-6"></a>6.6 Sketch external references on persistent topology identity | Pending — added 2026-09-04 by the industrial-parity overlay (row IP-13.2, leadership claim LC10) | — |

</details>

<details>
<summary>M7 — Sweep, surfacing and interrogation · 7 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-7-1"></a>7.1 Guided sweeps | Pending | — |
| <a id="p-7-2"></a>7.2 Loft continuity and periodic lofts | Pending | — |
| <a id="p-7-3"></a>7.3 Constrained N-sided fill | Pending | — |
| <a id="p-7-4"></a>7.4 Surface extension and curve imprint | Pending | — |
| <a id="p-7-5"></a>7.5 Interrogation | Partial — curvature analysis slice (`analyze::curvature` + `getFaceCurvature`/`getFaceMinRadius`); clash, silhouettes, draft pending | — |
| <a id="p-7-6"></a>7.6 Curve construction, fairing, degree reduction, continuity analysis | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-1.5, IP-1.6, IP-6.8, IP-8.3) | — |
| <a id="p-7-7"></a>7.7 Wire and curve offset completeness | Pending — added 2026-09-04 by the industrial-parity overlay (row IP-5.5) | — |

</details>

<details>
<summary>M8 — Industrialization · 7 registered items</summary>

| Issue | State and remaining scope | Evidence |
| --- | --- | --- |
| <a id="p-8-1"></a>8.1 Differential testing harness | Pending — extend B26 generated/fuzz oracles into randomized operation sequences, automatic shrinking/replay, nightly execution and the first-ten-defect exit. Reuse the existing repro substrate; B26 coverage alone does not satisfy this parent. | B26; [scope](p-class-program.md#81-differential-testing-harness-l--pull-early) |
| <a id="p-8-2"></a>8.2 Performance budget gates | Pending | — |
| <a id="p-8-3"></a>8.3 Parallel tessellation | Partial — native edge sampling and holed-planar CDT already have parallel paths; the remaining surface families, deterministic scaling/memory/cancellation qualification, and supported WASM execution remain open | [Source-pinned performance audit](../performance/kernel-optimization-roadmap.md#existing-optimizations-and-roadmap-reconciliation) |
| <a id="p-8-4"></a>8.4 Parallel boolean internals | Pending | — |
| <a id="p-8-5"></a>8.5 Real-model corpus | Partial — reuse the landed O1.1a–c pipeline, manifests and nightly smoke scoreboard. The real-assembly operate/export coverage and merge-bisectable regression exit still require explicit qualification evidence; do not create a second corpus harness. | [O1 register](#open-kernel-register); [scope](p-class-program.md#85-real-model-corpus-m) |
| <a id="p-8-6"></a>8.6 Arena compaction and versioned checkpoint contract | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-2.8, IP-12.4, IP-12.6); OpenZCAD roadmap W5 | — |
| <a id="p-8-7"></a>8.7 WASM threads and SIMD evidence gate | Pending — owner-gated adoption; added 2026-09-04 by the industrial-parity overlay (rows IP-14.8, IP-14.9) | — |

</details>

## Open Kernel register

Scope, stage dependencies and exit criteria:
[adoption and assurance specifications](open-kernel-implementation.md).
Wave labels are interpreted by the scheduling constraints above.

<details>
<summary>O1 — Robustness evidence · 15 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o1-1a"></a>O1.1a Gauntlet pipeline skeleton | A | Complete — isolated bounded workers run import, validation, disclosed probe boolean, manifold tessellation, and property-checked STEP round-trip; JSONL and aggregate JSON/Markdown outputs use stable taxonomy codes | [#164](https://github.com/esaueng/remus/pull/164) |
| <a id="o-o1-1b"></a>O1.1b Corpus manifests + fetcher | A | Complete — pinned 50-model smoke, 1,000-of-10,000 ABC, and 113-model MAMBO manifests; archive/member SHA-256 verification, content-addressed caching, deterministic sampling, and typed source refusals; no corpus bytes committed | [#166](https://github.com/esaueng/remus/pull/166) |
| <a id="o-o1-1c"></a>O1.1c Gauntlet CI wiring | A | Complete — nightly smoke and weekly abc-1k schedules publish reproducible aggregate scoreboards and append-only per-stage trends; a 0.50pp drop fails while still publishing the red aggregate | [#171](https://github.com/esaueng/remus/pull/171) |
| <a id="o-o1-1d"></a>O1.1d Triage loop (recurring) | A | Partial — 1/5 required classes closed: generic period-winding `FACE_BOUND` bands now reconstruct exact analytic seams or refuse transactionally; pinned smoke manifest `779fcc7f…` at `a36bddac` moved `invalid_input` 14→10 and full passes 26/50→29/50 | [#177](https://github.com/esaueng/remus/pull/177) |
| <a id="o-o1-2a"></a>O1.2a Head-to-head protocol + runners | B | Pending | — |
| <a id="o-o1-2b"></a>O1.2b Head-to-head scenario set | B | Pending | — |
| <a id="o-o1-2c"></a>O1.2c Head-to-head results page | B | Pending | — |
| <a id="o-o1-2d"></a>O1.2d Scorecard metric schema + absolute gates | A | Complete — versioned `tools/vs-bench` observations and deterministic JSON reports preserve all metric groups, one-hot oracle-classified outcomes, independent absolute gates, and equivalent-quality-only timing. Adversarial native/CLI contracts reject schema mismatch, incomplete evidence, undisclosed degradation, and oracle disagreement on either kernel; kernel runners and measured baselines remain O1.2a–c/e/f | [#275](https://github.com/esaueng/remus/pull/275) |
| <a id="o-o1-2e"></a>O1.2e Workflow scenarios W1–W9 | A | Partial — W9 STEP preallocation refusals run twice through the native facade and real WASM batch compatibility build, with per-stage typed-refusal and complete logical-session snapshot checks. W1–W8, post-allocation W9, shipped split translators, scorecard/results-page integration, and reference runners remain pending. | `tools/vs-bench/workflows/README.md`, `scripts/test-w9-preflight.sh` |
| <a id="o-o1-2f"></a>O1.2f Baseline pin milestone | B | Pending — added 2026-09-04 by the industrial-parity overlay (§3.3); locks the H5/H6 numeric bands | — |
| <a id="o-o1-3a"></a>O1.3a Fillet torture corpus + runner | A | Complete — 10 named cases built-and-verified or transactionally refused with stable codes | [#139](https://github.com/esaueng/remus/pull/139) |
| <a id="o-o1-3b"></a>O1.3b Fillet torture publication | C | Pending | — |
| <a id="o-o1-4a"></a>O1.4a STEP validation properties | A | Complete — opt-in CAx-IF validation properties round-trip aggregate and per-solid area, volume, centroid, and bounding boxes with derived units; malformed properties refuse transactionally with stable diagnostics, and direct/batch WASM contracts preserve import diagnostics | [#180](https://github.com/esaueng/remus/pull/180) |
| <a id="o-o1-4b"></a>O1.4b CAx-IF test-round manifest | B | Pending | — |
| <a id="o-o1-5"></a>O1.5 Native/WASM per-operation parity harness | A | Partial — the first 54-cell matrix generates cone/sphere, sphere/cylinder, and torus/sphere fuse/cut/intersect bundles across three scales and two placements, executes each through native and npm-installed release WASM `executeBatchV2`, oracle-checks exact quality and geometry on both, and diffs diagnostics, census, analytic carriers, mesh quality, volume, and serialized-byte SHA-256. Semantic invariants pass all 54 cells; raw arena bytes differ in all 54 and remain a visible non-gating exit gap. Full batch-op coverage, byte identity, failure/evolution fixtures, nightly registration, and the platform matrix remain. | `tools/parity/`, `scripts/test-o15-parity.sh` |

</details>

<details>
<summary>O2 — Exactness hardening · 11 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o2-1a"></a>O2.1a RFC 0006 swept analytic surfaces | A | Complete — the accepted design preserves STEP parameterization with self-contained math-layer profiles, checked projection, exact lowering/recognition, typed unsupported paths, staged R8 contracts, and a measured disposition for all 92 production `FaceSurface` wildcard matches | [#183](https://github.com/esaueng/remus/pull/183) |
| <a id="o-o2-1b"></a>O2.1b Revolution/extrusion math substrate | A | Complete — self-contained swept profiles plus revolution and linear-extrusion carriers provide checked evaluation/projection, exact first and second derivatives, curvature, explicit periods, and exact directed finite-span rational NURBS lowering; scale, seam, pole, reversed-span, success, and typed-refusal properties pin all six profile variants without adding topology variants | [#189](https://github.com/esaueng/remus/pull/189) |
| <a id="o-o2-1c"></a>O2.1c FaceSurface variants + site audit | B | Pending | — |
| <a id="o-o2-1d"></a>O2.1d Revolution/extrusion I/O wiring | B | Pending | — |
| <a id="o-o2-1e"></a>O2.1e Revolution/extrusion boolean arms | B | Pending | — |
| <a id="o-o2-2"></a>O2.2 Conic edges through booleans | B (M2 track) | Pending | — |
| <a id="o-o2-3a"></a>O2.3a Splitter inventory + design note | A | Complete — all ten callable special-case entry points are mapped to their geometric gates and direct or foil fixtures; the accepted design defines an exact-refined deterministic DCEL with certified event identity, periodic seam/pole quotienting, typed failures, property gates, and a staged three-entry-point deletion floor; positive isolation gaps for sector splitting and boundary chaining are explicit | [#193](https://github.com/esaueng/remus/pull/193) |
| <a id="o-o2-3b"></a>O2.3b UV-arrangement core | B | Pending | — |
| <a id="o-o2-3c"></a>O2.3c Winding classification bridge | B | Pending | — |
| <a id="o-o2-3d"></a>O2.3d Special-case migration + ratchet | B | Pending | — |
| <a id="o-o2-4"></a>O2.4 Predicate escalation policy | B (after 2.6) | Pending — added 2026-09-04 by the industrial-parity overlay (row IP-1.9) | — |

</details>

<details>
<summary>O3 — Performance foundations · 5 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o3-1"></a>O3.1 Inner-loop benches (math/algo/blend) | A | Complete — measured 64-cut and Gridfinity flamegraphs declare a 3% inclusive threshold; every qualifying stack family plus the prerequisite NURBS, SSI, Bézier clipping, CDT, GFA, and blend-walker loops now has a Criterion baseline wired into local comparison and hosted trend tracking | [#197](https://github.com/esaueng/remus/pull/197) |
| <a id="o-o3-1a"></a>O3.1a Audit baseline: NURBS, transforms, chaining | A | Complete — maintained three-family native/committed-WASM runner emits fixed workload identity, independent-process raw samples, correctness results, source/artifact provenance and failure records; M01/M06/M10 remain partial beyond this bounded slice | [Runner and acceptance](../performance/baseline.md) |
| <a id="o-o3-2"></a>O3.2 Journal-invalidated spatial cache | B | Pending | — |
| <a id="o-o3-3"></a>O3.3 SIMD in NURBS evaluation | B (evidence-gated) | Pending | — |
| <a id="o-o3-4"></a>O3.4 Journal-driven incremental tessellation | B (after O3.2) | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-9.5, IP-14.6) | — |

</details>

<details>
<summary>O4 — APIs and distribution · 13 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o4-1a"></a>O4.1a Facade crate + Model type | A | Complete — the native `remus::Model` owns topology and operation policy with journal access; its curated prelude exposes quality-disclosed booleans, v2 blends, sweeps, measurement, tessellation, STEP, validation, and persistent references with flat typed errors; the runnable quickstart and transactional refusal tests pin the contract | [#201](https://github.com/esaueng/remus/pull/201) |
| <a id="o-o4-1b"></a>O4.1b Facade examples | A | Complete — packaged native workflows now cover constrained-sketch bracket construction through exact STEP, typed STEP recovery with validate/heal-or-tolerate handling and an exact containment boolean, and native replay of the committed cross-drilled WASM contract; analytic volume plus validation and welded watertight/manifold oracles pin every result | [#225](https://github.com/esaueng/remus/pull/225) (landed via [#233](https://github.com/esaueng/remus/pull/233)) |
| <a id="o-o4-1c"></a>O4.1c WASM delegation to facade | B | Pending | — |
| <a id="o-o4-2a"></a>O4.2a Publish dry-run readiness | B | Pending | — |
| <a id="o-o4-2b"></a>O4.2b Tag-driven release automation | B | Pending | — |
| <a id="o-o4-2c"></a>O4.2c First publish | owner-gated | Pending | — |
| <a id="o-o4-3a"></a>O4.3a Python core binding | B | Pending | — |
| <a id="o-o4-3b"></a>O4.3b Python wheels + CI | B | Pending | — |
| <a id="o-o4-3c"></a>O4.3c PyPI publish | owner-gated | Pending | — |
| <a id="o-o4-4"></a>O4.4 Stable error-code registry (e5b) | A | Pending | — |
| <a id="o-o4-5"></a>O4.5 Stable C ABI decision record | owner-gated | Pending — added 2026-09-04 by the industrial-parity overlay (row IP-15.4) | — |
| <a id="o-o4-6"></a>O4.6 Serialization compatibility and migration policy | A | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-2.7, IP-12.5, IP-12.7) | — |
| <a id="o-o4-7"></a>O4.7 Typed direct-method WASM results | A | Partial — additive typed result envelopes cover the README's two-solid boolean family (`fuseDetailed`, `cutDetailed`, `intersectDetailed`) with direct/`executeBatchV2` code parity and unchanged legacy methods; remaining mutating families, the generated twin-coverage gate, typed replacements for 13 JSON-string returns, and the OpenZCAD regex acceptance list remain pending | `crates/wasm/src/bindings/booleans.rs` |

</details>

<details>
<summary>O5 — Interchange · 9 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o5-1a"></a>O5.1a STEP assembly reader | A | Pending | — |
| <a id="o-o5-1b"></a>O5.1b STEP assembly writer | A | Pending | — |
| <a id="o-o5-1c"></a>O5.1c Assembly WASM + batch | A | Pending | — |
| <a id="o-o5-2"></a>O5.2 Colors/names/attribute scope (e3b) | B | Pending | — |
| <a id="o-o5-3a"></a>O5.3a AP242 writer schema | B | Pending | — |
| <a id="o-o5-3b"></a>O5.3b PMI read, ref-anchored | C | Pending | — |
| <a id="o-o5-3c"></a>O5.3c PMI write | C | Pending | — |
| <a id="o-o5-4"></a>O5.4 Assembly occurrence identity and instancing | A/B (with O5.1) | Pending — added 2026-09-04 by the industrial-parity overlay (rows IP-2.3, IP-11.3) | — |
| <a id="o-o5-5"></a>O5.5 External references and partial loading (design + decision) | A | Pending — added 2026-09-04 by the industrial-parity overlay (row IP-11.6) | — |

</details>

<details>
<summary>O6 — Ecosystem · 4 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o6-1"></a>O6.1 Docs site | A | Pending | — |
| <a id="o-o6-2"></a>O6.2 Browser playground | B | Pending | — |
| <a id="o-o6-3"></a>O6.3 Second-consumer track (ongoing) | rolling | Pending | — |
| <a id="o-o6-4"></a>O6.4 Contribution posture | A | Pending | — |

</details>

<details>
<summary>O7 — Hybrid modeling · 1 registered items</summary>

| Issue | Wave | State and remaining scope | Evidence |
| --- | --- | --- | --- |
| <a id="o-o7"></a>O7 RFC 0007 mesh+B-Rep hybrid | C (after M4) | Pending | — |

</details>

## Bridge register

The B IDs retain existing PR references. B2 and B9 are aliases to their P-Class
owners, avoiding duplicate closure claims. All other rows own the named bridge
scope; related performance packages are child slices, not independent capability
promotions. Original stabilization B1/B2/B3 use the `STAB-` prefix in the
consolidation map below.

| ID | Item | Where | Size | Why it matters | State |
|---|---|---|---|---|---|
| <a id="b1"></a>B1 | **Healing disclosure typing** — the matrix's only named Unsupported-untyped *healing* cell (the matrix still lists tangency and sliver families; see H4): permissive healing can mask an invalid result as valid. Type every repair (report what changed, refuse to claim validity it didn't verify); both-sides tests. | `heal/src/fix/`, `check/src/validate/` | M | The last untyped silent-failure path in the kernel; highest correctness value per line. Do first in the qualification lane. | **Done (2026-09-03, PR #243):** fixer results enumerate counted repair kinds and typed declined repairs; L2 `OK` explicitly means only “no fixer action,” never validity. Operations, facade verified mode, configurable direct WASM, named pipelines, and additive detailed direct/batch WASM surfaces commit only after independent operations/check validation. Invalid and unverifiable results return stable typed refusals with attempted repairs and roll back. Native and WASM both-sides regressions pin verified success and refusal. |
| <a id="b2"></a>B2 | **Boolean scale residuals** — the through-tool family now returns exact material at 1e-5 and 1e6; straight-edge refinement and local planar bands are qualified by 72 operator/scale/placement cells. | `algo` bands | M | Feeds P-Class 2.6; remaining dimensional and curved-band work is tracked in [the audit](scale-band-audit.md). | Alias to P-Class 2.6 — named matrix passes; all remaining dimensional and curved-band work is owned there |
| <a id="b3"></a>B3 | **Closed-rim chamfers** — cone-frustum band mirroring the validated toroidal fillet assembler; closed-form volume oracle. Stabilization C1.2. | `blend`, `operations/src/chamfer.rs` | M | Exact surfaces, cheap, passes chase filter 1; unblocks resize_blend cylinder/cone (C2). | Open |
| <a id="b4"></a>B4 | **v2 walking-trimmer completion** — the four named gaps: keep-side hint, shared contact edges, end-cap notch trim, chamfer external-tangent branch. Stabilization C1.3. | `blend/src/trimmer.rs` | M | Critical path for v2 walker parity → legacy engine retirement (M5 precondition). | **Partial — planar trimmer done (#398, 2026-09-12):** keep-side `AwayFrom` + spine-edge authority, shared contact-edge adoption, two/three-edge end-cap notch, scale-aware parallel gate, vertex sharing across trims, solid-scoped live-wire stitch gate, multi-edge stitch runs, and reversed-loop rewinding are implemented with unit + concave-notch fillet/chamfer pins enabled (`regress_fillet_concave_notch`, `regress_chamfer_obtuse_ridge`, traversal-side analytic tests). The two e2e fixtures remain honest typed refusals: the blend-adjacent second pass (contact endpoints 0.5 off every boundary carrier) and the gridfinity peak rim (endpoints 0.6 = one radius off) both bottom out in curved-face trimming, which is M5 remainder work, not one of the four planar gaps. |
| <a id="b5"></a>B5 | **Offset face provenance** — offset derives faces 1:1 and discards the mapping; journal real evolution instead of a barrier. | `offset`, `operations/src/offset_v2.rs` | S | The last declared-barrier operation nobody owns; closes the B3-residual from stabilization. | **Done (2026-09-02, PR #224 (landed via #233)):** default intersection-joint V2 offsets retain and validate the total 1:1 construction map; native and direct/batch WASM journal wrappers record it transactionally. Closed-form plane/volume, persistent-reference, rollback, and WASM parity oracles pin the claim. Arc-joint and self-intersection-removal variants explicitly refuse this map because later face synthesis/replacement needs richer provenance. |
| <a id="b6"></a>B6 | **Evidence matrices, batched** — the "Stable-but-blocked" ledger rows that are pure test work: primitives invalid-input/scale/postconditions; plane-section cavity+degeneracy; measurement curved-cavity+scale; sweeps degenerate/cavity + nonconvergence budgets; convex hull/Minkowski degenerates. One qualify_*.rs per family, stabilization-plan pattern. | `operations/tests/` | M (S per family) | Flips ~8 Blocked ledger rows with zero new geometry; ideal bounded-session work. | **Partial — primitive family done (2026-09-03):** box, cylinder, pointed cone, frustum, sphere, torus, and ellipsoid are qualified across 1e-3/1/1e3 scale by closed-form volume/bounds, exact entity/surface censuses, dual validators, oriented closed B-Rep, watertight/manifold mesh, independent mesh-volume, determinism, and direct/batch WASM parity (`qualify_primitives.rs`, WASM `qualify_primitives_tests.rs`). The invalid matrix also closed non-finite box/sphere/torus acceptance. The ellipsoid follow-up repaired hemisphere selection, exact rational preservation, shared-equator pole-cap tessellation, and polar bounds; `ellipsoid-tessellation-scale.json` pins the permanent replay. Plane-section, measurement, sweeps, and convex hull/Minkowski families remain open. |
| <a id="b7"></a>B7 | **Pave-block attachment for marched FF curves on curved faces** — the named canonical fix for the cross-face boundary-desync family; three cheaper altitudes already failed. | `algo/pave_filler/make_blocks.rs`, `phase_ff.rs` | L | Deepest structural payoff in algo; root-causes a whole non-manifold family. Geometry lane, coordinate with M2; repro `replay_scplate.rs`. | Open |
| <a id="b8"></a>B8 | **Reversed NURBS sub-span convention** — forward spans shipped; reversed validated sub-spans blocked on the same arrangement defect as B7. | `topology/src/edge.rs` | M | Completes the endpoint-trimmed contract 2.0 builds on. | Open (after/with B7) |
| <a id="b9"></a>B9 | **Torus ∖ coaxial cylinder tangent cut** — the single cell keeping torus booleans Beta; needs a tangent-contact primitive (explicitly NOT the band splitter). | `math/analytic_intersection.rs`, `algo` splitter | M | B1-ledger promotion Beta→Stable; closed-form oracle exists. Rides 2.7 tangency machinery. | Alias to P-Class 2.7 — tangent torus witness and its promotion evidence remain open there |
| <a id="b10"></a>B10 | **Curve-curve / curve-surface classification qualification** + conic distance/classification cells | `math`, `geometry/extrema`, matrix harness | M | Unqualified since the matrix was written; sits under many families' claims; pure evidence. | Open |
| <a id="b11"></a>B11 | **Small hygiene set** — `log::debug!` false-zero in `fill_images_faces.rs` (diagnostic-infra bug); deterministic STEP entity ordering; heal `fix_duplicate_faces` winding-blind comparison; plane×plane sampled in-both exact upgrade; `n_fine` clamp hazard note→guard. | various | S each | Cheap, each has already cost or will cost a debugging session. | **Partial (2026-09-03, PR #239):** STEP export now canonicalizes unordered face, void-shell, and hole-loop aggregates while preserving semantic coedge traversal order; byte-equality regressions cover reordered faces and void shells. Remaining: false-zero diagnostic, winding-aware duplicate-face healing, plane×plane exact upgrade, and `n_fine` guard. |
| <a id="b12"></a>B12 | **Holes on non-planar section caps** — annular Coons or cap-then-subtract vs extruded-annulus ground truth (stabilization B2.2). | `operations/src/cap.rs`, `fill_face.rs` | M | Largest remaining non-planar-cap value with clean ground truth. H3, with M7 cap work. | **Partial (2026-09-04, PR #252):** sweep and pipe caps preserve disjoint rectangular iso-parametric holes on four-sided bilinear caps, matched against an independently extruded annulus by converged volume, manifold B-Rep, watertight mesh, classification, and direct/batch WASM. Off-surface, curved, touching, and n-sided holed trims refuse typed; loft-hole correspondence and holed partial revolutions remain Unsupported-typed and ride M7's cap work. |
| <a id="b13"></a>B13 | **STEP inner-shell (voids) export** — emit and read `BREP_WITH_VOIDS`, preserving cavity shell count and volume. | `io/src/step/{writer,reader}.rs` | S–M | Round-trip honesty for hollow parts; gauntlet round-trip stage will hit it. | Complete (2026-09-04): one- and two-void regressions verify single-solid round trips, shell counts, and volume. |
| <a id="b14"></a>B14 | **Render promotion track** — Experimental→Beta after a contract-stable release cycle (stabilization C4 residue); outside both programs. | `render` | S (time-gated) | Cleans the last stabilization row. | Open |
| <a id="b15"></a>B15 | **Cut/intersect pocket-face orientation on cylinder walls** | `crates/operations/tests/regress_parallel_boss_band_sections.rs`, `algo` assembly | M | **Done (2026-09-04, PR #255):** non-fuse assembly normalizes selected cylinder outer/inner wire winding before edge merge. Box and cylinder tools on both wall sides pass exact cut/intersect, dual validation, closed-form volume, material classification, and welded-mesh orientation oracles. The formerly ignored regression is permanent coverage. | Done |
| <a id="b16"></a>B16 | **Consumer topology-query API set** — one binding per OpenZCAD heuristic it currently reimplements (its roadmap C2): trimmed edge parameter domain, face material sense, ordered wire traversal, per-edge convexity, sphere-patch identity, seam-edge parity, `maxFilletRadius(solid, edges)`, batched `classifyPoint`, per-edge ids in `meshEdgesAll`; `unifyFacesChecked` (the strict input/result verdicts `unify_faces` already computes, so the adapter's union gate stops re-validating the same body up to five times per rebuild; added 2026-09-11); plus the GCS qualification matrix (constraint type × system state × scale, nonconvergence budget) from the P-Class §6 inherited queue. Each: exact, typed refusal on foreign handles, direct + batch WASM, contract test. Added 2026-09-04 by the [industrial-parity overlay](industrial-parity.md) (rows IP-15.9, IP-9.3, IP-10.3, IP-13.1/13.4). | `wasm/bindings/query.rs`, `batch.rs`, `operations/src/query.rs`, `sketch/`, `operations/tests/qualify_gcs.rs` (new) | S each | Every row retires an adapter-side heuristic; highest OpenZCAD impact per line. Exit: the named heuristic deleted from the consumer's adapter (recorded in the PR), matrix green. | **Partial (2026-09-11):** `unifyFacesChecked` shipped in PR #391 (native `heal::unify_faces_checked` + `UnifyFacesReport`, WASM binding, batch parity, packages 2.130.13); this binding slice closes when the OpenZCAD adapter PR that deletes the repeated `validateSolid` calls is linked here. The parent B16 row also requires its other query and GCS acceptance cells; they remain open. |
| <a id="b17"></a>B17 | **Healing defect-class qualification matrix** — a generated defect class × severity × repair policy × scale matrix per fixer (wire order/closure/gaps/small edges, face orientation/small faces, seams, shell orientation/sewing/free bounds, duplicates, continuity splits, representation conversion), plus an operand self-interference report for booleans and the faceted-import sew/unify contract (issue #244). Every cell: verified repair with counted disclosure, or typed refusal; both sides. Added 2026-09-04 by the overlay (rows IP-8.3, IP-3.8, IP-8.6). | `heal/`, `operations/src/heal.rs`, `operations/tests/qualify_heal.rs` (new), `stl/import.rs` | M (S per fixer) | The family is Qualified only at the B1 boundary; the reference kernel's healing breadth is its strongest documented area. Exit: every fixer has a matrix; #244 fixture green; self-interference report typed on a self-touching corpus. | Open |
| <a id="b18"></a>B18 | **Evolution completeness audit** — every topology-producing family reports total attribution or a typed unresolved record: unify same-domain (`unify_with_evolution`, OpenZCAD C1's top ask), sew, sweep/loft/revolve/extrude caps, arc-joint and self-intersection-removal offsets, section/split edges, direct edits (with 6.5), edge/vertex events beyond booleans. Added 2026-09-04 by the overlay (rows IP-3.6, IP-5.7, IP-12.1; leadership claim LC3). | `journal_ops.rs`, `evolution.rs`, `qualify_evolution_coverage.rs`, per-op modules | M (S per family) | Absolute gate §3.4 item 8; unblocks OpenZCAD's adoption order boolean → pattern → chamfer → shell/offset → direct edits. Exit: the coverage fixture claims every result face of every family exactly once or pins its typed unresolved; no `record_barrier_over_solid` call remains for a family that can construct its map. | Partial — direct-edit and healing slices merged in #338; face-only and unjournaled families remain. See [audit](evolution-audit.md). |
| <a id="b19"></a>B19 | **Remaining fuzz slices and mutation scope** — curve-intersection, offset, GCS, and tessellation fuzz targets with independent oracles on the weekly schedule. The mutation-scope slice moves the previously undiscovered root config to `.cargo/mutants.toml` and selects the current CDT directory. | `fuzz/fuzz_targets/`, `.cargo/mutants.toml`, `.github/workflows/fuzz.yml` | S each | Exit: four targets scheduled with committed seeds; mutants report shows CDT mutants examined. Scope regression checks reject ignored config and the stale CDT file glob; the bounded five-mutant sample caught four and retained one survivor for review. | Partial — mutation scope verified; four fuzz slices remain open. Evidence: `scripts/test-mutants-scope.py`, `docs/kernel-maturity/testing-strategy.md` |
| <a id="b20"></a>B20 | **Exact measurement completion (K-S2 remainder)** — ellipse, hyperbola, and NURBS planar boundaries; general curved-face area; deflection-independent curved-body volume, centroid, and inertia by Gauss quadrature over exact geometry with a stated bound; direct + batch WASM; scale matrix. Added 2026-09-04 by the overlay (row IP-10.1). | `check/src/properties/`, `operations/src/measure/` | M | OpenZCAD S2 measures 0.2–3.5 % volume error on filleted parts at its display deflection; the reference kernel integrates surfaces directly. Exit: relative error ≤ 1e-6 against closed forms on filleted and cavity primitives at 1e-3/1/1e3, independent of caller deflection; ledger row loses its "incomplete" caveat. | Open |
| <a id="b21"></a>B21 | **Boolean fallback disclosure by default** — `boolean()`, `fuse`/`cut`/`intersect`, `fuseAll`, and the `*WithOptions`/`*WithEvolution` bindings return a bare handle when the GFA path falls over to the mesh (co-refinement) boolean; the only disclosure is `log::warn!(target: "remus_approx")` in `boolean/mod.rs::run_mesh_fallback`, and `OperationContext::new()` defaults to `AllowApproximate { budget: 0.1 }`. The Rust facade already returns `BooleanOutcome`. Decide one of: default to `ExactOnly` with `booleanWithQuality` as the opt-in, or return quality from every entry point. Found 2026-09-09. | `operations/src/boolean/mod.rs`, `math/src/context.rs`, `wasm/src/bindings/booleans.rs`, `remus/src/model.rs` | S–M | A log line is not a return value; an "exact" kernel whose consumer renders the handle silently ships NURBS-degraded faces. Same contract on Rust and JS. Breaking for JS callers that relied on the silent fallback: changelog + adapter notice. | **Done (2026-09-10):** plain Rust and WASM boolean entry points (handle-returning, options, evolution, `fuseAll`, batch) run `ExactOnly` and return the typed refusal; approximation is reachable only through `boolean_with_context` / `booleanWithQuality` and the new `boolean_outcome_with_options`, all of which return the disclosed quality. Tangent-boss regression pins refusal, rollback, and disclosure natively and in batch; changelogs carry the breaking notice. |
| <a id="b22"></a>B22 | **`main` CI cancellation** — `ci.yml` `concurrency.group: ci-${{ github.ref }}` with `cancel-in-progress: true` cancels the CI run of every merge commit that is followed by another merge within ~30 min (#338, #339, #341, #346, #349 on 2026-09-09 have `conclusion: cancelled`). Scope cancellation to `pull_request` refs (e.g. `cancel-in-progress: ${{ github.event_name == 'pull_request' }}`) so every `main` head gets a verdict. | `.github/workflows/ci.yml` | S | Every CI-green claim on `main` is currently "green on the last push in the burst". Also affects the committed-package refresh evidence. | **Done (2026-09-09):** `cancel-in-progress` is now `github.event_name == 'pull_request'`; the same change tiers the suite (`docs/owner-pr-ci.md` § Tiers) so kernel PRs run ~12 min of jobs and the publisher's refresh PR runs none. |
| <a id="b23"></a>B23 | **One fillet cascade policy** — WASM `fillet` runs v2 → rolling-ball → bevel (`wasm/src/helpers.rs::try_fillet`), `blend_ops.rs::planar_fillet_result` orders a legacy attempt first for concave edges, and the facade `Model::fillet` is v2-only; the deprecated v1 engines are reached in production through `#[allow(deprecated)]`. Move the cascade into `operations` behind one function both surfaces call, with the engine that produced the result disclosed in the outcome. Does NOT decide v1 retirement (owner's product decision, still "not queued"). | `wasm/src/helpers.rs`, `operations/src/blend_ops.rs`, `operations/src/fillet/`, `remus/src/model.rs` | M | Rust and JS callers get different geometry for the same request today; a rejected attempt can leave the input partly filleted unless rollback is explicit (documented trap). Precondition for honest M5 parity numbers. | **Done (2026-09-10):** `blend_ops::fillet_cascade` is the one policy — walking engine (`fillet_v2`, which already tries the rolling-ball rebuild first for planar-line selections), then the guarded rolling-ball rebuild, each transactional; the flat bevel is no longer a fillet fallback. WASM `fillet` / `filletWithEvolution` / batch `fillet`, the facade `Model::fillet`, and `fillet_with_evolution` all call it, and `BlendResult::engine` discloses which engine produced the result. v1 retirement is still not decided. |
| <a id="b24"></a>B24 | **Wildcard-arm audit and gate** — ~180 `_ =>` arms over `EdgeCurve`/`FaceSurface` (densest, regenerated 2026-09-11 at `95de160` with the skill's `rg --multiline` query, 180 total: `measure/volume.rs` 16, `pave_filler/phase_ff.rs` 16, `tessellate/nonplanar.rs` 12, `resize_blend.rs` 11). A new variant compiles clean and silently takes the approximate branch in volume, meshing, and GFA splitting. Convert the four densest files to exhaustive matches; add a `scripts/check-wildcard-arms.sh` ratchet (count per file, no growth) to the `repo-policy` job, same pattern as `check-det-hash.sh`. Companion to RFC 0006 (face-surface wildcard audit). | `operations/src/measure/volume.rs`, `algo/src/pave_filler/phase_ff.rs`, `operations/src/tessellate/nonplanar.rs`, `operations/src/resize_blend.rs`, `scripts/` | M | The only current defence is `approx_census`, which detects drift after the fact. RFC 0006 swept-analytic surfaces will add a variant and hit every one of these. | Open |
| <a id="b25"></a>B25 | **Offset path consolidation** — `offset_v2` (wrapper over `remus-offset`), sampled `offset_face` (public `samples: u32` knob), and `offset_trim` are all exposed independently through WASM (`bindings/operations.rs`, `batch.rs`), and `shell_op.rs` still calls the sampled `offset_face` internally. Route `shell_op` through the exact offset where the face family allows it and mark the sampled binding as approximate in its result/type, or fold it. | `operations/src/offset_face.rs`, `offset_trim.rs`, `shell_op.rs`, `wasm/src/bindings/operations.rs` | M | A discretization knob on the public API contradicts the exact-kernel contract; three ways to offset is a support burden for the adapter. | Open |
| <a id="b27"></a>B27 | **Hammer-holder opening edit (OpenZCAD consumer case)** — imported Shapr3D holder, 46 → 50 mm opening by a mask cut, shifted-source intersection, and fuse. Not a §B chase by filter 2 (a general NURBS-import boolean chain), queued here because ten merged PRs of geometry-lane work had no owner row. Historical evidence: [hammer-holder-opening-status.md](../hammer-holder-opening-status.md); fixtures `crates/io/tests/hammer_opening_partition.rs`, diagnostic `crates/io/examples/hammer_opening.rs`. | `algo/builder/fill_images_faces.rs`, `face_splitter/`, `pave_filler/phase_ff.rs` | L | The first consumer-driven imported-model edit to complete exact-only end to end; every repair on the way (branch retention, tolerance carry-through, curved sections, torus-patch classification, concave subdivision) is generic. Its residual risk is the calibration web: every PR in the stack touched the section/clip/arrangement path, and the deepened-notch foil caught one regression. | **Partial (2026-09-10, #357, #361, #363–#375):** the specific replay completes all eight exact-only booleans natively (36-face shifted intersection, 194-face fuse, dimensions/bores/lettering preserved, strict validation, watertight mesh, STEP round trip) and through the WASM smoke path. Not qualified: general parameter ranges, OpenZCAD parameter/history references, AI editing path, and any second imported model. |
| <a id="b26"></a>B26 | **Boolean and mesh invariant proptests** — the workspace has 15 proptest blocks (13 in `math`) and 8 golden files against ~445k lines, while roughly half of `scripts/` tests the CI policy itself. Add property tests over random primitive pairs and rigid transforms: inclusion–exclusion volume identity (A ∪ B + A ∩ B = A + B), fuse/cut complement, translation invariance of `solid_volume` (the doubled-boundary detector), watertight/manifold mesh, exact-only path stability under 1e-13 nudges. | `operations/tests/prop_boolean_invariants.rs` (new), `operations/src/boolean/tests.rs` | S–M | Cheapest broad oracle the kernel lacks; every lesson in the roadmap skill about doubled boundaries and translation-variant volume is a property nobody generates. Rides B19's schedule for the slow variants. | Open |
| <a id="b28"></a>B28 | **NURBS-heavy imported-model interactive perf (OpenZCAD rebuild chain)** — the growing hammer-holder rebuild (esaueng/OpenZCAD#275: 42 NURBS faces, 38×58 bicubic patches) spent 86 % of every heavy stage in `NurbsSurface::derivatives`, projected 730k edge×NURBS-face pairs per fuse, and re-validated one body five times. Owner row for perf work on this chain; every PR is result-identical by construction (no tolerance, sample, or threshold change) and pins a before/after number. Remaining named stages: strict `validate_solid` ≈ 1 s native for one 42-face body (the inside-out signed-volume integral is still the floor), `mass_properties` ≈ 2.3 s, and the 170 s shifted-holder exact intersection (B27's replay) — none has a budget yet; when one is set it becomes P-Class 8.2's first gate. Queued 2026-09-11 after three merged PRs had no owner row. | `math/src/nurbs/{curve,surface}.rs`, `algo/src/pave_filler/phase_ef.rs`, `operations/src/distance.rs`, `check/src/properties/face_integrator.rs`, bench `crates/io/benches/nurbs_properties.rs`, `perf-counters` scaling guards | M (S per stage) | OpenZCAD's rebuild is interactive or it is not used; before this train the union gate took 14 s per strict validation in WASM. Exit: a per-stage budget on the holder operands enforced by a scaling guard or Criterion floor in CI, and the OpenZCAD rebuild timing recorded against the pinned package. | **Partial (2026-09-11, #389, #394, #395):** weight-scale cache + shared partials (validate 3.16 s → 0.97 s, mass 7.88 s → 2.25 s native; WASM strict validate 14.2 s → 1.89 s), conservative edge/face box gating (3-way fuse at 16.1 mm 3.42 s → 0.17 s native, `fuseAll` 9.61 s → 0.87 s WASM), NURBS-face pruning in the solid distance query (arm×arm 757 ms → 54 ms). Bench and two `scaling_` guards are permanent. #411 additionally fuses position and partial evaluation in the NURBS quadrature path; #413 proposes the strict-validation budget and #418 extends scratch reuse (both in flight at this snapshot). The full stage-budget and consumer-timing exit remains open. |

| <a id="b29"></a>B29 | **Local transactions and document memory** — coordinate nested transactions/savepoints, compare mutation-local undo with chunk COW, qualify append-only fast paths and immutable payload sharing. PERF-T01–T05/T07, PERF-I02 and PERF-W04 supply bounded designs and dependencies. Versioned compaction stays 8.6. | `topology/src/transaction.rs`, `wasm/src/`, import staging | L (S per measured slice) | A local edit should not clone unrelated document state. Exit: injected failures, nested rollback, retire/restore and stale handles preserve the exact pre-operation state; fixed-workload latency/memory scaling improves with source-pinned native/WASM evidence. | Open — consolidated from the performance audit; no implementation claim. |
| <a id="b30"></a>B30 | **Profile-selected subsystem optimization and measurement** — ownership for audit packages without a narrower capability or performance parent (expanded workload/profile coverage, math storage/fitting, sketch, mesh production, IO and build experiments). Select one PERF package and one workload; do not treat this umbrella as one implementation PR. | Per-package source references in the performance register | S per slice | Preserve capability, tolerances, refusal semantics and API compatibility while improving a measured end-to-end or scaling/memory cost. Exit: selected package acceptance plus the performance qualification matrix; close a no-gain experiment with evidence. | Open — performance audit consolidation, not a new feature mandate. |
| <a id="b31"></a>B31 | **Multi-body mesh import result contract** — decide the `SolidId` versus `Vec<SolidId>` reader convention across STL/3MF/OBJ/PLY/glTF; preserve and cite the pinned current-behavior test before any migration. | `io/` mesh readers and WASM import bindings | M | Carries the previously unowned P-Class inherited item. Exit: explicit compatibility decision, migration policy and native/direct/batch contracts for disconnected-body imports. | Decision required — breaking API work is not authorized by adding this row. |

## Performance work packages

All 111 packages from the 2026-09-12 audit are retained, with their original
priorities, phases, dependency IDs, acceptance criteria and pinned source references.
Read evidence and experiment safeguards in the [performance audit](../performance/kernel-optimization-roadmap.md).
`PERF-` is the namespace on this page; dependencies use the original short IDs.

One **owner** is assigned per package. Update both the package disposition and its
owner's remaining scope here when a slice lands; completing a child does not close
its parent. `Proposed` is an audit proposal, not a claim that existing mechanisms
are absent. M01/M06/M10 retain broader scope beyond O3.1a; N04 retains scratch and
consumer-adoption work beyond #411. Baseline-era measurements are not current-head
benchmarks. Priorities are within the performance lane, subordinate to current
correctness and consumer needs.

The CSV/JSON are generated from the tables below with
`python3 scripts/sync-roadmap-inventory.py`; verify with `--check`. Do not edit
those exports by hand. They contain work specifications, not implementation status.

**Evidence:** M = measured at the audit baseline; S = source inspection;
H = hypothesis/design. None implies a shipped speedup. **Priority:** P0 = evidence,
P1 = first optimizations, P2 = selected by profiles, P3 = experiment.
**Effort:** S/M/L/XL ≈ 1–3 days / 1–2 weeks / 2–6 weeks / multi-month work per
qualified slice, not a delivery date. Calibrate estimates after profiling.

### Performance phases


The program has nine phases. The phase number is an implementation lane, not a requirement to finish every item in an earlier lane before starting an independent later one. Use the explicit dependency map in the CSV/JSON to select ready work. Every optimization PR should be bounded to one mechanism and one measured workload family.

| Phase                                 | Deliverable                                                                                                            | Scope and dependency                                                                                                                                                                        | Exit gate                                                                                                                                              |
| ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **0 — Establish evidence**            | Reproducible baseline dashboard, missing workloads, phase/copy/allocation counters and honest gates                    | M01–M08, M10, S01, W01, W06. Extend existing O3.1 rather than restarting it.                                                                                                                | Pin every artifact; report quality, latency and memory; identify top functions and growth terms on procedural and imported workloads.                  |
| **1 — Remove local structural costs** | Small, independently reviewable optimization PRs                                                                       | Line projection, chaining, compact interpolation storage, derivative outputs, scratch reuse, sketch allocations and binding/render copies. Each needs its own baseline/correctness fixture. | Before/after improves the targeted metric outside noise; useful full-workload effect or a justified scaling/memory win; no new correctness regression. |
| **2 — Prepare and index**             | Reusable query context, conservative bounds, invalidation substrate and sparse candidate generation                    | Q01→Q02; Q05 before curved pruning; B02–B06, healing/sewing indexes, common validation/recognition preparation. Can proceed alongside Phase 1.                                              | Repeated queries avoid rebuilding; sparse candidate growth drops; mutate/restore and unknown-bound tests pass.                                         |
| **3 — Local transactions and memory** | Transaction coordinator, chosen undo/COW design, shared immutable payloads, staged imports, later versioned compaction | T01→T02/T03; T04/T05; W04 and I02. Compaction T06 follows explicit handle/checkpoint design and lifetime measurements.                                                                      | A fixed local edit no longer scales with unrelated document size except justified bookkeeping; rollback and handle safety hold under injected failure. |
| **4 — Incremental derived work**      | Mesh, edge sample and property reuse; changed-region recomputation; efficient export/render consumers                  | Q02 and cache correctness; D03 before D04; V03 can reuse transaction boundaries.                                                                                                            | Mutate-then-recompute matches a cold full recomputation, including neighboring boundary dependencies; bounded memory with predictable eviction.        |
| **5 — Subsystem specialization**      | Profile-selected NURBS/SSI, sketch, blend/offset, modeling, IO and render improvements                                 | Select by measured frequency × cost and capability needs. Each row is a separate work package; this phase is not one giant rewrite.                                                         | Per-family correctness oracles, real-model benefit and distribution behavior are demonstrated; unsupported cases stay explicit.                        |
| **6 — Deterministic concurrency**     | Tuned native scheduling, more tessellation/integration/GFA parallelism, optional WASM threads                          | Pure task interfaces and boundary ownership first; aggregate thread-local counters. WASM deployment requirements follow separately.                                                         | 1/2/4/8/16-thread data, identical deterministic contracts, cancellation, small-input crossover and peak-memory budgets.                                |
| **7 — Hardware/compiler experiments** | SIMD, specialized layouts, GPU crossover, PGO, allocator/build variants and optional transport formats                 | Only after a representative baseline identifies the relevant cost. Keep portable and general fallback paths.                                                                                | Held-out workloads improve; native/WASM quality and artifact-size/load budgets pass; close no-gain experiments without shipping them.                  |
| **8 — Qualification and publication** | Combined-kernel regression budgets and reproducible comparison results                                                 | Repeat after each phase, not only once at the end. Real competitor runners remain distinct from the synthetic scorecard.                                                                    | Exact-source native/direct/batch WASM plus real-model, memory/lifetime, failure, determinism and compatibility gates pass.                             |

The shortest path to useful improvements is **0 → selected Phase 1 items**, while cache and transaction designs proceed independently. Incremental work depends on invalidation; broad parallelism depends on task isolation; all phases feed qualification. Do not wait for arena compaction before fixing a sampled line projection, nor attempt persistent caches before their mutation behavior is defined.


<details>
<summary>Measurement and performance governance</summary>

### Measurement and performance governance

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/.github/workflows/fleet-benchmark.yml#L11), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/scripts/bench-compare.sh#L29), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/tools/vs-bench/README.md#L1), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/perf.rs#L31).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-m01"></a>PERF-M01 | P0 / 0 / S | Create one versioned workload manifest spanning all 15 crates, native facade, direct WASM, batch, translator and render entry points; reuse the existing gauntlet manifest and scorecard. | Every public operation family has a representative scenario or an explicit coverage gap; pin source, artifact, fixture and harness hashes. | M/low | — | [B30](#b30) | Partial — bounded baseline #406; full audit scope remains |
| <a id="perf-m02"></a>PERF-M02 | P0 / 0 / S | Expand the maintained runner beyond the current selected suites: include IO NURBS properties, compound/fuse, query, transaction, sketch, import/export and session workloads. | One local command regenerates the inventory and results; no suite silently disappears because it is outside operations or requires a feature. | S/low | M01 | [B30](#b30) | Proposed |
| <a id="perf-m03"></a>PERF-M03 | P0 / 0 / S | Turn performance regression policy into an effective gate; the current reusable workflow needs the ci:benchmark PR label and only comments at 200%. | Calibrate noise on a controlled runner, then block reproducible material regressions on relevant PRs; preserve the untrusted-build/trusted-publisher separation. | M/medium | M02, M06 | [P-Class 8.2](#p-8-2) | Proposed |
| <a id="perf-m04"></a>PERF-M04 | P0 / 0 / H | Add scoped timings and deterministic work counters for snapshot bytes, candidate pairs, projections, BVH builds, quadrature, CDT fallback scans, serialization and copies. | Counters attribute growth and phase time without changing release behavior; measure instrumentation overhead and keep timing assertions out of ordinary tests. | M/low | M01 | [B30](#b30) | Proposed |
| <a id="perf-m05"></a>PERF-M05 | P0 / 0 / H | Record native allocation count, allocated bytes, peak live bytes, RSS, arena live/retired slots, checkpoint retention and WASM memory high-water marks. | Run create/edit/fail/undo/delete cycles at fixed live model size; distinguish live heap from allocator capacity and linear-memory reservation. | M/low | M01 | [B30](#b30) | Proposed |
| <a id="perf-m06"></a>PERF-M06 | P0 / 0 / S | Fix benchmark methodology: separate setup, warmup, operation, validation and teardown; replace minimum-only reporting in batch_profile with raw samples and median/tail analysis. | Use independent processes and paired baseline/candidate runs; enough samples for meaningful p95/p99; do not include untimed fixture construction in a named-operation CPU profile. | S/low | M01 | [B30](#b30) | Partial — bounded baseline #406; full audit scope remains |
| <a id="perf-m07"></a>PERF-M07 | P0 / 0 / H | Profile representative real models and adversarial scaling, including large imported NURBS, hollow solids, tangencies, dense contacts, many disconnected bodies and large coordinate translations. | Archive sampled CPU profiles and allocation profiles; qualify exact success/refusal separately from approximate success and error. | M/low | M02, M04, M05, M06 | [B30](#b30) | Proposed |
| <a id="perf-m08"></a>PERF-M08 | P0 / 0 / H | Measure complete user workflows: import to visible mesh, repeated selection/measurement, sketch drag, direct face edit, save/restore and export. | Report cold and warm latency, p95, memory and quality per stage; include the consumer/worker boundary in addition to native kernel time. | M/low | M01, M06 | [B30](#b30) | Proposed |
| <a id="perf-m09"></a>PERF-M09 | P1 / 8 / S | Finish real competitor runners and scenario pins in vs-bench; the current fixture is synthetic protocol coverage and bench-compare.sh is native-only. | Compare identical units, tolerances, representation, validation obligations and outcomes; publish failures and refusals alongside timing and reproducible commands. | L/medium | M01, M06, M07, M08 | [O1.2a](#o-o1-2a) | Proposed |
| <a id="perf-m10"></a>PERF-M10 | P1 / 0 / S | Reconcile profiling instructions, command aliases and roadmap status with executable code. | Record O3.1 complete, native tessellation partially parallel, SIMD flags enabled, comparison harness retired, and current benchmark PR activation; retain remaining qualification gates. | S/low | M01 | [B30](#b30) | Partial — bounded baseline #406; full audit scope remains |

</details>

<details>
<summary>Transactions, topology storage and lifetime</summary>

### Transactions, topology storage and lifetime

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/topology/src/transaction.rs#L40), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/topology/src/arena.rs#L72), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/topology/src/topology.rs#L108), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm/src/bindings/batch.rs#L458), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/ds/shape_store.rs#L88), [source 6](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/docs/design/deferred-e6b-arena-compaction-and-slot-reuse.md#L1).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-t01"></a>PERF-T01 | P1 / 3 / M | Introduce a transaction coordinator/savepoints so one mutating WASM operation does not independently snapshot the same topology at each nested native wrapper. | Count snapshot count/bytes versus document size and nesting depth; preserve per-item batch commits, nested rollback, journals, attributes, and permanently stale failed handles. | L/high | M04, M05, W01 | [B29](#b29) | Proposed |
| <a id="perf-t02"></a>PERF-T02 | P1 / 3 / M | Prototype mutation-local undo records or page/chunk copy-on-write for topology; compare both against the measured whole-topology clone baseline. | A fixed local edit should track touched state rather than unrelated model size; test all mutation accessors, aliasing, failure injection and snapshot retention. | L/high | T01 | [B29](#b29) | Proposed |
| <a id="perf-t03"></a>PERF-T03 | P1 / 3 / S | Add an append-only transaction fast path for qualified construction, with a write-set guard and full fallback for operations that mutate existing entities. | Measure 150 makeBox operations at increasing preexisting document sizes; rollback must retire every failed allocation without reissuing IDs. | M/high | T01, T02 | [B29](#b29) | Proposed |
| <a id="perf-t04"></a>PERF-T04 | P1 / 3 / S | Share immutable NURBS carrier and pcurve payloads across snapshots, copies and GFA stores, while keeping topology ownership explicit. | Measure clone bytes and import/export cost on large control nets; transformations must detach or create new geometry and leave originals/checkpoints unchanged. | L/high | M05, T02 | [B29](#b29) | Proposed |
| <a id="perf-t05"></a>PERF-T05 | P1 / 3 / S | Remove redundant geometry clones inside GFA deep-copy materialization and benchmark operand-local staging/export separately from document snapshots. | Track each copied carrier and peak simultaneous buffers; preserve source-to-result face/edge/vertex lineage and shared aliases. | M/medium | M04, M05 | [B29](#b29) | Proposed |
| <a id="perf-t06"></a>PERF-T06 | P1 / 3 / S | Complete the versioned arena-compaction/checkpoint design before implementing reclamation of retired or unreachable entities. | Long edit/undo/delete sessions reclaim measured payload memory; old numeric handles cannot alias new objects, persistent references survive or fail explicitly, and formats stay versioned. | XL/high | T02, W04, M05 | [P-Class 8.6](#p-8-6) | Proposed |
| <a id="perf-t07"></a>PERF-T07 | P2 / 3 / S | Tune arena reservation and layout using actual entity-size and allocation distributions; evaluate chunked storage and packed liveness separately. | Preserve the existing bounded geometric growth policy; compare memory spikes, clone cost and iteration locality on native and wasm32. | M/high | M05, T02 | [B29](#b29) | Proposed |
| <a id="perf-t08"></a>PERF-T08 | P2 / 2 / S | Cache solid-local traversal/adjacency results and index persistent-reference signatures/history resolution where repeated queries justify it. | Measure lookup over increasing topology/history sizes; invalidate on edits, retirement, restore and changed discriminators; preserve deterministic ordering and ambiguity. | M/high | Q02 | [O3.2](#o-o3-2) | Proposed |

</details>

<details>
<summary>Spatial bounds, classification and distance</summary>

### Spatial bounds, classification and distance

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/bvh.rs#L212), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/classify/mod.rs#L311), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/distance/mod.rs#L50), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/distance.rs#L57), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/classifier/ray_cast.rs#L159).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-q01"></a>PERF-Q01 | P1 / 2 / S | Build one operation-local prepared query context holding face IDs, conservative bounds, BVH, analytic descriptors and trim data; reuse across rays and points. | Compare 1/100/10000 points on one solid, including recovery rays; prove equivalence to existing paths for cavities, seams and OnBoundary cases. | M/medium | M04, M07 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q02"></a>PERF-Q02 | P1 / 2 / S | Add persistent spatial caching after mutation invalidation is proven: topology incarnation plus mutation generation first, finer per-entity revisions later. | Mutate-then-query, raw mutable access, tolerance changes, failed edits, checkpoint restore and same-ID/different-topology cases never reuse stale entries; bound cache memory. | L/high | Q01, M05 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q03"></a>PERF-Q03 | P1 / 2 / M | Optimize BVH construction after measuring reuse: compare current repeated axis sorts and suffix allocations with binned SAH, median splits and reusable scratch. | Measure build time, memory, traversal work and amortized cost for query counts 1/10/10000 on sparse, clustered and overlapping bounds. | M/medium | Q01, M05 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q04"></a>PERF-Q04 | P1 / 1 / S | Use true nearest-distance branch-and-bound instead of building a BVH then sorting/enumerating all faces; keep explicitly non-prunable faces on a mandatory side list. | Preserve trimmed-surface and cavity distance results; measure faces tested plus candidate setup, with certified conservative bounds and exhaustive fallback for unknown bounds. | M/high | Q05, M04 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q05"></a>PERF-Q05 | P1 / 2 / S | Provide conservative finite-span curve/trimmed-face bounds and explicit confidence, improving selectivity without trusting endpoint-only or sample-only boxes. | Analytic extrema and rational control-hull proofs cover full domains; test periodic seams, nonuniform weights, tilted patches and scale before enabling pruning. | L/high | M01, M04 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q06"></a>PERF-Q06 | P2 / 1 / S | Reuse BVH traversal stacks and caller-owned candidate buffers; the current into variants still allocate an internal stack. | Benchmark allocation count and small-query latency; guarantee clean scratch state after early exit/error and deterministic candidate handling. | S/low | M04 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q07"></a>PERF-Q07 | P2 / 2 / S | Accelerate cached GFA ray geometry with conservative spatial culling; it currently caches geometry but still scans its face descriptors. | Count ray-face evaluations on many-face solids; preserve all analytic crossing multiplicities, grazing recovery and finite trim tests. | M/high | Q01, Q05 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-q08"></a>PERF-Q08 | P2 / 5 / H | Evaluate two-level body/face acceleration and BVH refits for clash, transformed instances, projection and repeated assembly queries. | Compare rebuild versus refit and degeneration after many edits; run exact narrow-phase checks and use instance transforms without copying the body. | L/high | Q02, Q05 | [O3.2](#o-o3-2) | Proposed |

</details>

<details>
<summary>Booleans and GFA</summary>

### Booleans and GFA

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/pave_filler/phase_ve.rs#L72), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/pave_filler/phase_ve.rs#L176), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/pave_filler/phase_ee.rs#L29), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/pave_filler/phase_ff.rs#L384), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/compound_ops.rs#L686), [source 6](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/algo/src/builder/fill_images_faces.rs#L52).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-b01"></a>PERF-B01 | P1 / 1 / S | Replace sampled straight-line vertex-edge projection with a domain-aware analytic projection; add qualified circle/conic paths separately. | Count curve evaluations per surviving pair and run VE/GFA scaling; retain tolerance composition, reversed trims, degenerate lines and endpoint behavior. | S/medium | M04, M06 | [B30](#b30) | Proposed |
| <a id="perf-b02"></a>PERF-B02 | P1 / 2 / S | Generate vertex-edge candidates spatially instead of visiting every cross-solid pair; current VE only bounds straight edges and scans all edges. | Test sparse V-by-E growth and curved-edge cases; preserve mandatory processing for unbounded/unqualified boxes and count broad-phase pairs, not only accepted interferences. | M/high | Q05, M04 | [B30](#b30) | Proposed |
| <a id="perf-b03"></a>PERF-B03 | P1 / 2 / S | Replace EE all-pairs enumeration with deterministic spatial candidate generation over conservative edge bounds. | Count pair visits at 10x edge counts; retain overlaps, tangencies, shared IDs and per-entity tolerance margins, with dense worst cases still bounded. | M/high | Q05, M04 | [B30](#b30) | Proposed |
| <a id="perf-b04"></a>PERF-B04 | P1 / 2 / S | Replace FF nested face loops with BVH or sweep candidate generation and shared face metadata. | Benchmark sparse and dense face-pair workloads; candidate completeness and deterministic junction allocation must match the baseline. | M/high | Q05, M04 | [B30](#b30) | Proposed |
| <a id="perf-b05"></a>PERF-B05 | P2 / 2 / H | Audit VV, VF and EF candidate generation together so phase-local speedups do not leave repeated topology gathering or another all-pairs phase dominant. | Add separate phase counters and reuse validated domains/bounds across compatible phases; do not bypass per-phase tolerance or representation checks. | M/high | Q05, M04 | [B30](#b30) | Proposed |
| <a id="perf-b06"></a>PERF-B06 | P1 / 2 / S | Index N-body overlap partitioning; fuse_all already batches connected clusters but partition_touching still checks every body pair. | Measure 100/1000/10000 disconnected bodies and dense clusters; preserve touching versus strictly disjoint semantics and exact polyhedral/cylinder gap proofs. | M/medium | Q05, M04 | [P-Class 4.8](#p-4-8) | Proposed |
| <a id="perf-b07"></a>PERF-B07 | P2 / 5 / S | Extend true N-ary execution where pairwise/sequential fallbacks remain expensive; keep existing compound_cut and fuse_cluster as the baseline. | Track intermediate face counts, copied bytes and fallback rate; require equivalent material, region partition, order policy and construction lineage. | L/high | B06, M07 | [P-Class 4.8](#p-4-8) | Proposed |
| <a id="perf-b08"></a>PERF-B08 | P1 / 5 / H | Rank exact analytic fast paths by observed fallback cost and workload frequency; qualify the highest-value missing surface/contact family one at a time. | Report exact success, typed refusal and disclosed approximation separately; require independent material/volume and topology oracles, not just fewer faces. | L/high | M07 | [P-Class 2.4](#p-2-4) | Proposed |
| <a id="perf-b09"></a>PERF-B09 | P2 / 1 / S | Reuse phase-local topology lists, curve domains, tolerance checks, surface frames and boundary projections instead of repeated lookup/collection. | Attribute saved allocations and samples on a GFA phase benchmark; cache keys include the domain, tolerance and geometry revision. | M/medium | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-b10"></a>PERF-B10 | P2 / 5 / H | Measure remaining face splitting, same-domain grouping, shell assembly and junction/weld work; extend existing spatial indices only where counters reveal growth. | Preserve existing fixed scaling regressions; count rejected candidates, near-coincident junction probes and assembly passes on both sparse and dense fixtures. | M/high | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-b11"></a>PERF-B11 | P2 / 6 / H | Parallelize pure pair geometry into per-worker result buffers, then sort and commit in a deterministic topology-allocation phase. | Measure 1/2/4/8/16 threads, serial small-input crossover and peak memory; identical results, typed first-error policy and accurate aggregated counters are mandatory. | XL/high | B04, T01, P01 | [P-Class 8.4](#p-8-4) | Proposed |
| <a id="perf-b12"></a>PERF-B12 | P2 / 5 / S | Optimize the already-bounded mesh fallback independently: reuse operand tessellation, BVHs and triangle predicates, and eliminate repeated conversion. | Keep it explicitly approximate; unchanged error bounds, watertightness and resource refusal; exact workloads must never be routed here to appear faster. | L/high | M07, D02, Q01 | [B30](#b30) | Proposed |

</details>

<details>
<summary>Math, NURBS, fitting and intersection</summary>

### Math, NURBS, fitting and intersection

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/fitting.rs#L210), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/fitting.rs#L323), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/intersection/chaining.rs#L184), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/evaluator.rs#L17), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/surface.rs#L389), [source 6](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/nurbs/intersection/surface_seeding.rs#L70).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-n01"></a>PERF-N01 | P1 / 1 / M | Store interpolation matrices in compact band form; the solve is already banded but allocation/zeroing remains n-by-n. | Measure resident allocation and runtime for 128/512/2048/8192 points; retain pivot fill bandwidth and interpolation residuals on ill-conditioned/repeated-knot inputs. | M/high | M05, M06 | [B30](#b30) | Proposed |
| <a id="perf-n02"></a>PERF-N02 | P1 / 1 / S | Exploit local support in least-squares fitting instead of dense N-transpose-N assembly; factor once and solve all coordinates together. | Benchmark sample and control-point scaling; compare residuals and endpoint constraints, and consider banded QR when conditioning makes normal equations unsuitable. | M/high | M04, M05, M06 | [B30](#b30) | Proposed |
| <a id="perf-n03"></a>PERF-N03 | P1 / 1 / M | Replace intersection-point all-pairs adjacency and full-component nearest-neighbor searches with deterministic spatial neighborhoods and ordering. | The measured sparse chain should stop growing approximately quadratically; preserve current threshold graph, all branches, ties, loops and close separate components. | M/high | M04, M06 | [B30](#b30) | Proposed |
| <a id="perf-n04"></a>PERF-N04 | P1 / 1 / S | Expose scratch-buffer derivative APIs and fuse position/first/second derivative evaluation so consumers reuse basis values and homogeneous sums. | Count allocations and basis evaluations on integration, projection, SSI and walker workloads; verify derivative finite differences and projective weight-scale invariance. | M/high | M04, M05, M07 | [B28](#b28) | Partial — fused quadrature evaluation #411; scratch extension #418 in flight |
| <a id="perf-n05"></a>PERF-N05 | P2 / 5 / M | Benchmark direct versus cached evaluators on realistic multi-span grids before adopting the existing power-basis evaluator; both audit fixtures made the cached path slower. | Include construction cost, amortization, rational degrees 3/9, knots and derivatives; adopt only proven winning regimes with a stable dispatch criterion. | M/medium | M07 | [B30](#b30) | Proposed |
| <a id="perf-n06"></a>PERF-N06 | P2 / 5 / S | Precompute per-axis basis/span tables for tensor-product sampling and quadrature; reuse factorizations only when knot/parameter systems really match. | Measure surface grids, loft rows and repeated fits; do not assume independently chord-parameterized rows share the same matrix. | M/high | N04, N05 | [B28](#b28) | Proposed |
| <a id="perf-n07"></a>PERF-N07 | P2 / 5 / S | Reduce SSI patch cloning, redundant subdivision and re-marching of an already traced branch; reuse decomposition/evaluator state per surface pair. | Track patch bytes, seed count, unique branches, Newton calls and retries; preserve tangencies, tiny loops, periodic closure and existing resource budgets. | L/high | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-n08"></a>PERF-N08 | P2 / 5 / S | Improve safeguarded projection/extrema with temporal seeds, reusable workspaces and proven bounds; retain global-search fallback where local Newton is insufficient. | Benchmark repeated nearby queries and adversarial multiple minima; an estimated Lipschitz constant is not a new license for certified pruning. | L/high | Q05, M07 | [B30](#b30) | Proposed |
| <a id="perf-n09"></a>PERF-N09 | P2 / 5 / S | Profile filtered predicate fast-path/fallback rates and duplicate filtering, plus CDT constraint recovery and stale-hint linear scans. | Count actual fallback work by geometry family; preserve exact predicate signs and constrained edges on near-degenerate and large-coordinate inputs. | M/high | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-n10"></a>PERF-N10 | P3 / 7 / H | Evaluate f64 SIMD batches and structure-of-arrays layouts for basis evaluation, projections, bounds and surface grids. | Show end-to-end benefit on native and actual SIMD WASM; respect exact arithmetic/error bounds, alignment, lane tails and code-size budgets. | L/high | N04, N05, W01, M07 | [O3.3](#o-o3-3) | Proposed |
| <a id="perf-n11"></a>PERF-N11 | P3 / 7 / H | Evaluate small fixed-degree kernels, polynomial specializations, inlining and cold-path separation only after profiling identifies a material hotspot. | Measure instructions, cache misses and code size as well as time; keep high-degree/rational/general fallbacks and unchanged floating-point contracts. | M/high | M07 | [B30](#b30) | Proposed |

</details>

<details>
<summary>Tessellation and mesh production</summary>

### Tessellation and mesh production

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/tessellate/solid.rs#L331), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/tessellate/solid.rs#L913), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/cdt/constraints.rs#L8), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/tessellate/nurbs.rs#L31), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/tessellate/mesh_ops.rs#L28).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-d01"></a>PERF-D01 | P1 / 4 / S | Create a revisioned per-face mesh cache with shared edge-sample dependencies and explicit keys for linear/angular tolerance, purpose and face attribution. | Unchanged views reuse meshes; one edge edit invalidates every incident face; tests cover holes, seams, sheets, cavities and rolled-back edits. | L/high | Q02, D03 | [O3.4](#o-o3-4) | Proposed |
| <a id="perf-d02"></a>PERF-D02 | P1 / 4 / S | Share edge sampling between solid tessellation, wireframe, export and queries when contracts match. | Count evaluations and copied points; do not mix display samples with mesh-boolean circle-floor policy or lose directed coedge domains. | M/high | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-d03"></a>PERF-D03 | P1 / 4 / S | Split tessellation into boundary planning, local triangulation, shared-boundary reconciliation and deterministic assembly before broadening parallelism. | Existing edge sampling and holed-planar CDT remain the baseline; Steiner points reaching neighboring faces must be reconciled before dependent triangulation. | L/high | M04, M07 | [O3.4](#o-o3-4) | Proposed |
| <a id="perf-d04"></a>PERF-D04 | P2 / 6 / S | Parallelize eligible nonplanar faces and independent meshes with workload-based grain sizes, after boundary ownership is stable. | Measure thread scaling, skewed face sizes, small inputs and memory; prove watertightness and face-ID stability across thread counts. | L/high | D03, P01 | [P-Class 8.3](#p-8-3) | Proposed |
| <a id="perf-d05"></a>PERF-D05 | P2 / 1 / S | Instrument CDT point-location/constraint walks and reuse legalization scratch; repair frequent fallback scans rather than replacing Hilbert ordering that already exists. | Track triangle visits, flips, Steiner points and fallback scans at 1k/10k/100k points, long holes and skinny constraints. | M/high | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-d06"></a>PERF-D06 | P2 / 5 / H | Reduce over-tessellation through analytic/adaptive subdivision within the requested chord and angular budgets. | Verify maximum geometric deviation and topology on curved trims and singularities; triangle reduction must not weaken an existing quality contract. | L/high | M07 | [B30](#b30) | Proposed |
| <a id="perf-d07"></a>PERF-D07 | P2 / 4 / S | Reduce mesh assembly passes and temporary arrays for grouping, welding, normals and face offsets; reserve from measured counts. | Compare allocated bytes and wall time at fixed triangle count; keep stable grouping, seam normals and shared boundary IDs. | M/medium | M05, M07 | [B30](#b30) | Proposed |
| <a id="perf-d08"></a>PERF-D08 | P2 / 4 / H | Support incremental LOD and cancellation of obsolete display work at an explicit preview API boundary. | Measure time-to-first-visible and final-quality latency; stale jobs cannot replace a newer mesh, and preview output remains clearly approximate. | L/high | D01, W05 | [O3.4](#o-o3-4) | Proposed |
| <a id="perf-d09"></a>PERF-D09 | P2 / 4 / H | Reuse a compatible final mesh for multiple exports and render consumers instead of rebuilding it per format. | Cache keys include body revision and quality; verify format-specific normals, winding, units, attributes and invalidation. | M/medium | D01, D02 | [O3.4](#o-o3-4) | Proposed |
| <a id="perf-d10"></a>PERF-D10 | P3 / 7 / S | Evaluate the existing GPU analytic mesher for supported display workloads and measured CPU-to-GPU transfer costs. | Use graphics output only within declared quality; keep native/WASM CPU paths, supported-device matrix and geometric deviation checks. | L/high | M07, R05 | [B14](#b14) | Proposed |

</details>

<details>
<summary>Validation, mass properties and measurement</summary>

### Validation, mass properties and measurement

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/properties/face_integrator.rs#L97), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/properties/mod.rs#L149), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/check/src/validate/mod.rs#L50), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/measure/volume.rs#L2916), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/io/benches/nurbs_properties.rs#L24).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-v01"></a>PERF-V01 | P1 / 2 / M | Prepare trim loops, projected boundaries and quadrature/evaluator data once per face per operation, then reuse across volume/area/inertia/validation. | Measure the hammer-holder benchmark and analytic/cavity fixtures; preserve periodic UV branches, trim orientation and compensated/local-origin arithmetic. | M/high | M04, M07 | [B28](#b28) | Proposed |
| <a id="perf-v02"></a>PERF-V02 | P1 / 4 / S | Cache face property contributions and recombine only changed faces using construction history. | Repeated measurements become proportional to changes; compare against full recomputation after every supported edit/undo, including inner shells and face reversals. | L/high | Q02, V01 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-v03"></a>PERF-V03 | P1 / 3 / S | Deduplicate repeated validation of the same immutable result within a transaction, using an explicit validation certificate keyed to all relevant policies. | Instrument each validator invocation; preserve full required checks at trust boundaries and invalidate certificates on any change or rollback. | L/high | T01, Q02, M04 | [B28](#b28) | Proposed |
| <a id="perf-v04"></a>PERF-V04 | P2 / 2 / S | Share topological indexes among edge-use, shell, Euler, orientation and finite-geometry checks rather than rebuilding traversals per check. | Measure allocations and entity visits on large valid and malformed bodies; identical diagnostic severities, entity IDs and acceptance are required. | M/medium | Q01, M04 | [O3.2](#o-o3-2) | Proposed |
| <a id="perf-v05"></a>PERF-V05 | P2 / 5 / H | Specialize additional qualified analytic property integrals and reuse shared moment evaluation across requested outputs. | Closed-form independent oracles, thin walls, trimmed holes and translated models pass; do not infer full primitive volume from its carrier alone. | M/high | M07 | [B20](#b20) | Proposed |
| <a id="perf-v06"></a>PERF-V06 | P2 / 6 / H | Parallelize independent face integrations and validation work using deterministic reduction and stable diagnostics. | Compare p95 and reduction error across thread counts, especially cavity cancellation and large translations; retain serial thresholds. | M/high | V01, P01 | [B28](#b28) | Proposed |
| <a id="perf-v07"></a>PERF-V07 | P2 / 5 / S | Accelerate hidden-line/visibility and repeated measurement requests with prepared classification and batched query APIs. | Measure sample-point count, context rebuilds and end-to-end drawing time; preserve silhouette/visibility semantics and declared approximation. | M/high | Q01 | [P-Class 7.5](#p-7-5) | Proposed |

</details>

<details>
<summary>Healing and sewing</summary>

### Healing and sewing

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/heal/src/fix/solid.rs#L32), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/heal/src/upgrade/shell_sewing.rs#L215), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/heal.rs#L530), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/heal/src/upgrade/unify_same_domain.rs#L37).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-h01"></a>PERF-H01 | P1 / 1 / S | Spatially index vertex merging in native healing and the operations wrapper; both contain pairwise vertex loops. | Count distance checks and runtime over 1k/10k/100k vertices; preserve canonical representative, tolerance policy and nontransitive-nearness behavior. | M/high | M04, M06 | [B17](#b17) | Proposed |
| <a id="perf-h02"></a>PERF-H02 | P1 / 2 / S | Index free-edge sewing by endpoint neighborhoods and curve signatures before exact compatibility checks. | Sparse free-edge candidates scale with local contacts; preserve ambiguity refusals, coedge/pcurve lineage and cases with several eligible partners. | M/high | Q05, M04 | [B17](#b17) | Proposed |
| <a id="perf-h03"></a>PERF-H03 | P2 / 2 / S | Bucket duplicate-face candidates by conservative geometry/topology descriptors before the current pairwise comparison. | Verify reversed wires, holes, aliases and genuinely distinct coincident faces; descriptors only select candidates and cannot establish equivalence alone. | M/high | Q05, M04 | [B17](#b17) | Proposed |
| <a id="perf-h04"></a>PERF-H04 | P2 / 5 / H | Make healing pipelines process dirty entities and reuse analysis findings when intervening steps do not invalidate them. | Track scans and repeated repairs; preserve final validation, repair disclosure, tolerance limits and complete construction history. | L/high | Q02, V04 | [B17](#b17) | Proposed |
| <a id="perf-h05"></a>PERF-H05 | P2 / 1 / S | Retain and expand unify_faces scaling guards; the former per-group whole-edge-map rescan has already been fixed. | Reproduce current many-group prism/STL baselines and investigate only remaining hot paths; do not recreate an already-merged optimization ticket. | S/low | M04, M06 | [B17](#b17) | Proposed |
| <a id="perf-h06"></a>PERF-H06 | P2 / 5 / H | Profile SameParameter, curve/surface recognition, small-edge/wire repair and unification at multiple model sizes. | Reuse projections and analytic tests only on matching revisions; no performance gain may come from deleting legitimate small features or loosening repair tolerances. | M/high | M07, Q01 | [B17](#b17) | Proposed |

</details>

<details>
<summary>Sketch solver</summary>

### Sketch solver

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/sketch/src/gcs/system.rs#L343), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/sketch/src/gcs/solver.rs#L32), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/sketch/src/gcs/qr.rs#L35), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/sketch/src/gcs/dof.rs#L23).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-s01"></a>PERF-S01 | P1 / 0 / S | Add sketch benchmarks for drag updates, large sparse systems, independent components, redundant constraints and inconsistent systems. | Record solve/diagnostic time, Jacobian dimensions, iterations, rank, residuals and allocations at 10/100/1000/10000 parameters where supported. | M/low | M01, M06 | [B30](#b30) | Proposed |
| <a id="perf-s02"></a>PERF-S02 | P2 / 5 / S | Split the constraint graph into independent connected components before dense solving. | Time depends on component sizes rather than total unrelated parameters; preserve DOF, rank, conflict classification and all coupled constraints. | L/high | S01, M07 | [B30](#b30) | Proposed |
| <a id="perf-s03"></a>PERF-S03 | P2 / 1 / S | Reuse residual/Jacobian/QR/step workspaces and avoid per-iteration cloning/reallocation where ownership permits. | Allocation count per iteration drops; solver convergence, step acceptance, failed-solve rollback and diagnostics remain equivalent. | M/medium | S01, M05 | [B30](#b30) | Proposed |
| <a id="perf-s04"></a>PERF-S04 | P2 / 5 / S | Evaluate sparse Jacobian assembly and rank-revealing sparse solving for large coupled sketches, with the existing dense solver as the small-system baseline. | Use sparse and dense adversarial matrices; preserve rank/DOF and ill-conditioned behavior; review new numerical dependencies explicitly. | XL/high | S02, S03 | [B30](#b30) | Proposed |
| <a id="perf-s05"></a>PERF-S05 | P2 / 5 / H | Cache structural sparsity/parameter maps and warm-start related drag solves; factorization reuse requires an unchanged or explicitly updated Jacobian. | Measure interactive traces including topology/constraint changes; do not reuse stale numeric factors merely because the graph is unchanged. | L/high | S02, S03 | [B30](#b30) | Proposed |
| <a id="perf-s06"></a>PERF-S06 | P2 / 5 / S | Share final residual/Jacobian work between solve_detailed, DOF and diagnostics when evaluated at the same iterate. | Reduce duplicated computation without changing restored state, per-constraint residuals or error classifications. | M/high | S01, S03 | [B30](#b30) | Proposed |

</details>

<details>
<summary>Blends, offsets and other modeling operations</summary>

### Blends, offsets and other modeling operations

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/blend/src/walker.rs#L268), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/blend/src/analytic.rs#L108), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/offset/src/inter3d.rs#L24), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/offset/src/move_faces.rs#L113), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/sweep.rs#L40), [source 6](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/operations/src/query.rs#L141).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-o01"></a>PERF-O01 | P2 / 5 / S | Reuse support evaluations and derivative/Jacobian work inside blend correctors; tune continuation from measured iteration/retry counts. | Benchmark curved/variable-radius stations and whole fillets, not just the planar walker fixture; preserve G1 contact, radius and stopping conditions. | M/high | M07, N04 | [B30](#b30) | Proposed |
| <a id="perf-o02"></a>PERF-O02 | P2 / 5 / H | Prioritize additional certified analytic blend/offset cases by actual walker and fallback cost. | Independent radius/material/volume checks and transformed-scale fixtures pass; general supports remain typed refusals until qualified. | L/high | M07 | [B30](#b30) | Proposed |
| <a id="perf-o03"></a>PERF-O03 | P2 / 5 / S | Reuse adjacent-support intersections, projected trim curves and vertex planning in offsets/re-limitation; current inter3d is already adjacency-based. | Measure local edit cost on a large body; preserve pcurve authority, winding, tolerance composition and construction mappings. | L/high | Q01, M07 | [B30](#b30) | Proposed |
| <a id="perf-o04"></a>PERF-O04 | P2 / 5 / S | Reuse sweep frames, arc-length maps, profile sampling and compatible loft factorization; avoid reconstructing shared profile geometry. | Benchmark long paths and many sections, including twists/kinks/holes; preserve parameterization, continuity and exact analytic lowering. | M/high | M07, N06 | [B30](#b30) | Proposed |
| <a id="perf-o05"></a>PERF-O05 | P2 / 5 / H | Use assembly instances and delayed topology materialization for repeated transforms/pattern display where the API can express shared geometry. | Measure 10k instances versus copied solids; retain identity/attributes and explicitly materialize when booleans or exact export require independent topology. | XL/high | T04, Q08 | [O5.4](#o-o5-4) | Proposed |
| <a id="perf-o06"></a>PERF-O06 | P2 / 2 / S | Cache feature-recognition/selection adjacency and bucket opposing planar faces by normal/position before polygon overlap tests. | Measure repeated selection on many-face solids; retain holes, blend exclusions, tolerance-aware pairing and ambiguous matches. | M/high | Q02, M07 | [B16](#b16) | Proposed |
| <a id="perf-o07"></a>PERF-O07 | P2 / 5 / S | Plan local direct edits and resize-blend operations against an affected neighborhood, minimizing copied surfaces and repeated trial rebuilds. | Count touched versus total entities; preserve transactional refusal, contact dependencies and total lineage across zero-radius removal and topology changes. | L/high | T01, Q02, M07 | [P-Class 6.2](#p-6-2) | Proposed |

</details>

<details>
<summary>Import, export and serialization</summary>

### Import, export and serialization

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/io/src/step/reader.rs#L346), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/io/src/arena_io.rs#L768), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm-io/src/lib.rs#L171), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/io/src/threemf/writer.rs#L93), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/io/src/gltf/writer.rs#L22).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-i01"></a>PERF-I01 | P1 / 5 / S | Profile STEP lexical scanning, string ownership, attribute reparsing, reference lookup, topology construction and validation separately; prototype borrowed spans/typed parsed attributes. | Measure peak memory and time on large repeated entities/NURBS; preserve units, multiple contexts, escapes, malformed references and preallocation limits. | L/high | M04, M05, M07 | [B30](#b30) | Proposed |
| <a id="perf-i02"></a>PERF-I02 | P1 / 3 / S | Use staged import/document construction to avoid full existing-document snapshots for a qualified atomic append. | Import cost should follow new content rather than unrelated prior models; malformed/post-allocation refusal restores all state and handle safety. | L/high | T01, T03 | [B29](#b29) | Proposed |
| <a id="perf-i03"></a>PERF-I03 | P2 / 5 / S | Serialize through borrowed views or streaming writers to avoid cloning an owned arena dump before JSON encoding. | Measure allocations, peak bytes and output time; preserve current schema versions, ordering, aliases, exact floats, attributes and round-trip compatibility. | M/high | M05, M07 | [B30](#b30) | Proposed |
| <a id="perf-i04"></a>PERF-I04 | P2 / 5 / S | Measure and reduce exact arena transfer overhead between the split kernel and translator WASM modules. | Report kernel serialize, JS transfer, translator deserialize and export separately; retain module isolation, shared versioning, body roots and limits. | L/high | W01, I03 | [B30](#b30) | Proposed |
| <a id="perf-i05"></a>PERF-I05 | P2 / 5 / S | Stream multi-solid mesh exports and archive payloads to avoid holding all meshes, XML, binary staging buffers and final output together. | Peak memory scales with a bounded chunk where API permits; validate complete archive/schema, normals, units and deterministic output. | L/high | M05, M07 | [B30](#b30) | Proposed |
| <a id="perf-i06"></a>PERF-I06 | P2 / 4 / S | Route exporters through a compatible shared mesh producer/cache where possible; inspect per-face glTF tessellation versus solid-level STL/3MF paths. | Preserve each format's actual mesh contract and material/face grouping; compare seam quality and output size as well as speed. | M/high | D09 | [B30](#b30) | Proposed |
| <a id="perf-i07"></a>PERF-I07 | P3 / 7 / H | Evaluate optional versioned binary arena transport and compression settings only after copy/parse measurements show a material benefit. | Keep existing JSON readers/writers compatible; document precision, canonicalization, resource limits, migration and native/WASM round-trip oracles. | XL/high | I03, I04, M07 | [O4.6](#o-o4-6) | Proposed |

</details>

<details>
<summary>WASM bindings and browser delivery</summary>

### WASM bindings and browser delivery

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm/src/bindings/batch.rs#L437), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm/src/shapes.rs#L79), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm/src/bindings/tessellate.rs#L160), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm/src/bindings/checkpoint.rs#L20), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/xtask/src/wasm.rs#L185).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-w01"></a>PERF-W01 | P1 / 0 / M | Add real installed-package direct/batch workflow benchmarks with source and WASM hash provenance; the audit confirms document-size growth on the shipped package. | Measure cold/warm Node and real browsers, copies, memory and error behavior; compare native/WASM only when source, flags and fixtures match. | M/low | M01, M06 | [B30](#b30) | Proposed |
| <a id="perf-w02"></a>PERF-W02 | P1 / 1 / S | Prefer existing typed/binary mesh outputs for bulk geometry and avoid repeated cloning getters; evaluate a consume-once buffer API. | Compare JS JSON/grouped/binary paths at fixed mesh sizes; preserve f64/f32 contracts, ownership, array lengths, face offsets and repeated-call behavior. | M/high | W01, M05 | [O4.7](#o-o4-7) | Proposed |
| <a id="perf-w03"></a>PERF-W03 | P2 / 5 / S | Measure JSON parse/dispatch/serialize cost for tiny batch operations; consider typed numeric batches only when overhead dominates. | Benchmark 1/10/1000 operations with identical native work; preserve validation order, typed errors, references and per-item rollback semantics. | L/high | W01, M04 | [O4.7](#o-o4-7) | Proposed |
| <a id="perf-w04"></a>PERF-W04 | P2 / 3 / S | Reduce checkpoint memory through shared immutable topology and copy-on-write at a finer granularity; Rc checkpoint sharing already exists. | Measure creation, first post-checkpoint edit, restore, discard and retained sketches/assemblies; keep non-reuse of external handles and exact session restoration. | L/high | T02, M05 | [B29](#b29) | Proposed |
| <a id="perf-w05"></a>PERF-W05 | P2 / 4 / H | Keep long kernel work in a worker and coalesce/cancel obsolete interactive requests at the consumer boundary. | Measure main-thread responsiveness, transfer time and cancellation latency; do not hide kernel CPU time or change which operation commits. | L/high | P02, W01 | [P-Class 2.8](#p-2-8) | Proposed |
| <a id="perf-w06"></a>PERF-W06 | P2 / 0 / S | Treat cold loading, code size and module transfer as separate budgets; translator splitting and SIMD build flags already exist. | Measure raw/compressed bytes, compile/instantiate time, first call and steady state on real browsers; preserve the current 9 MiB review/10 MiB hard package policy. | M/medium | W01 | [B30](#b30) | Proposed |
| <a id="perf-w07"></a>PERF-W07 | P3 / 6 / H | Prototype WASM threads only after native task isolation and consumer deployment requirements are settled. | Benchmark startup, pool memory and 1/N-thread exact results; provide non-threaded fallback and obtain separate approval for deployment/header changes. | XL/high | P01, B11, D04, W01 | [P-Class 8.7](#p-8-7) | Proposed |
| <a id="perf-w08"></a>PERF-W08 | P3 / 7 / H | Evaluate explicit borrowed/zero-copy views only behind a documented lifetime/ownership design; copying typed arrays are currently the safe baseline. | Prove validity across memory growth, reentrancy and free; compare saved copies against JS/worker transfer costs and never expose stale views. | L/high | W02, M05 | [O4.7](#o-o4-7) | Proposed |

</details>

<details>
<summary>Rendering</summary>

### Rendering

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/render/src/lib.rs#L169), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/render/src/pipeline.rs#L549), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/render/src/mesh.rs#L64), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/render/src/pipeline.rs#L710), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/render/src/compute_mesh.rs#L71).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-r01"></a>PERF-R01 | P2 / 5 / S | Add a reusable offscreen render session/context; the convenience path currently creates a GPU context per render. | Separate cold device/pipeline setup from repeated-frame cost; preserve device-loss recovery, options and software-adapter behavior. | M/medium | M07 | [B14](#b14) | Proposed |
| <a id="perf-r02"></a>PERF-R02 | P2 / 4 / S | Retain mesh/GPU buffers across camera-only frames and upload only changed geometry. | Measure tessellation, upload bytes and frame latency; update face-ID picking and attributes with the correct geometry revision. | M/high | D01, R01 | [B14](#b14) | Proposed |
| <a id="perf-r03"></a>PERF-R03 | P2 / 1 / S | Replace per-triangle vertex expansion with per-face indexed render vertices where normals/attributes permit. | Measure GPU bytes and vertex processing; preserve sharp normals and the correct face ID at every triangle and seam. | M/medium | M05, M07 | [B14](#b14) | Proposed |
| <a id="perf-r04"></a>PERF-R04 | P2 / 5 / S | Make readback asynchronous or optional for repeated rendering; separate color and ID readback by caller need. | Measure GPU/CPU synchronization and tail latency; do not reuse buffers before completion or return stale picking data. | M/medium | R01, M07 | [B14](#b14) | Proposed |
| <a id="perf-r05"></a>PERF-R05 | P3 / 7 / H | Benchmark culling, instancing, index locality and GPU compute crossover on the intended graphics workload. | Distinguish discrete GPU, integrated GPU and software rendering; optimize a measured render workload without claiming an exact-kernel speedup. | L/medium | M07, R01 | [B14](#b14) | Proposed |

</details>

<details>
<summary>Build optimization, scheduling and qualification</summary>

### Build optimization, scheduling and qualification

Inspected entry points: [source 1](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/Cargo.toml#L118), [source 2](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/wasm/Cargo.toml#L16), [source 3](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/crates/math/src/context.rs#L119), [source 4](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/docs/kernel-maturity/p-class-status.md#L69), [source 5](https://github.com/esaueng/remus/blob/ca91b956cdde99f78f048dd32b656b559ad14e7d/docs/kernel-maturity/open-kernel-status.md#L45).

| ID | Priority / phase / evidence | Work | Measurement and acceptance | Effort / risk | Dependencies | Owner | State |
| --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="perf-p01"></a>PERF-P01 | P2 / 6 / S | Choose task grain and thread-pool policy by estimated work rather than only edge/face counts; avoid nested oversubscription. | Benchmark 1/2/4/8/16 threads on several body sizes; compare p95, peak memory, throughput and serial crossover; aggregate thread-local work counters correctly. | M/high | M04, M07 | [B30](#b30) | Proposed |
| <a id="perf-p02"></a>PERF-P02 | P2 / 5 / S | Extend cooperative cancellation and deterministic budgets into long inner phases; context and phase-boundary checks already exist. | Measure cancellation latency during candidate generation, allocation, fitting and serialization; rollback and output quality stay unchanged. | M/high | M04, M07 | [P-Class 2.8](#p-2-8) | Proposed |
| <a id="perf-p03"></a>PERF-P03 | P3 / 7 / S | Benchmark release LTO/codegen-unit choices, WASM -Oz versus speed-oriented optimization, and CPU-specific builds where distribution permits. | Track execution, cold load, compile time and artifact size together; retain portable builds and exact shipped-artifact correctness tests. | M/medium | M07, W06 | [B30](#b30) | Proposed |
| <a id="perf-p04"></a>PERF-P04 | P3 / 7 / H | Run a profile-guided optimization experiment trained on a representative corpus, with disjoint holdout models. | Accept only broad held-out gains with no meaningful regressions; record training provenance and repeat after important compiler/algorithm changes. | L/medium | M07, P03 | [B30](#b30) | Proposed |
| <a id="perf-p05"></a>PERF-P05 | P3 / 7 / H | Test allocator choice, compact collection layouts, hot/cold data splitting and assembly-level tuning only after allocation/cache profiles justify them. | Measure total workload cost and memory, not isolated allocation speed; document dependencies, unsafe requirements and native/WASM differences. | L/high | M05, M07 | [B30](#b30) | Proposed |
| <a id="perf-p06"></a>PERF-P06 | P1 / 8 / H | Qualify the combined optimized kernel after each wave and freeze reproducible performance/quality baselines for release. | Native plus actual direct/batch WASM, real models, resource exhaustion, deterministic threading, long sessions and exact-source CI all pass; attach no-gain closures too. | L/high | M03, M05, M07, W01 | [P-Class 8.2](#p-8-2) | Proposed |

</details>

## Decisions and exclusions

| Decision / boundary | Owner and disposition |
| --- | --- |
| Stable C ABI | O4.5 decision record before implementation. |
| External references and partial loading | O5.5 decides kernel versus application responsibility. |
| WASM threads / isolation headers | P-Class 8.7; measured feasibility and explicit consumer deployment decision. |
| Arena compaction / slot reuse | P-Class 8.6 and its versioned checkpoint/handle design; current retirement is not reclamation. |
| First crates.io/npm publish and PyPI publish | O4.2c / O4.3c. Build and dry-run readiness are separate from permission to publish. |
| Playground hosting and second-consumer outreach | O6.2 / O6.3 retain explicit hosting/outreach decisions. |
| Multi-body mesh reader API | B31 compatibility decision before changing return types. |
| v1 fillet API retirement | Maintainer product decision, not required by B23's shared cascade or H5's geometry gate. |
| IGES growth | Declined: STEP is the exchange path; IGES remains a declared lossy preview. Reopening requires evidence and a decision. |
| Shared-face non-manifold topology | Later RFC under RFC 0005; its ordering against O7 requires a decision. |
| Hybrid mesh/B-Rep | O7 design only under the body-model gate. Existing mesh fallback does not grant hybrid-modeling capability. |
| Application history and parametrics | Consumer scope; kernel work covers identity/evolution, not the feature tree or UI. |
| Monolithic mechanical-feature operators / tessellated STEP | Not queued without consumer/corpus evidence; existing composable primitives remain the scope. |
| Retired harness/lattice scenarios | Reopen only with an in-repository consumer fixture. |
| General duplicate-edge merge key | Do not retry the disproven universal key; use the splitter-side construction identified by the roadmap skill. |
| Periodic pinch / general arrangement / coordinated scoop split | Terminal without the named missing primitive in the roadmap skill. Closed bounded box/sphere and torus/box cells do not qualify the general arrangement problem. |
| Mesh co-refinement optimization | PERF-B12 is evidence-gated; it remains unselected until a live disclosed-approximation workload justifies it. |
| Fork lineage | Upstream v3+ behavior requires independent implementation or an explicit Apache-2.0 grant. |

## Consolidation map

Previous filenames and issue IDs remain discoverable; retired ledgers link here.
No requirement is considered complete merely because its document was consolidated.

| Former plan / overlapping item | Current owner / reference |
| --- | --- |
| P-Class plan + status ledger | All issue state above; `p-class-program.md` retains detailed capability specifications and typed exit criteria. |
| Open Kernel strategy + implementation + status | O register above; `open-kernel-implementation.md` retains stage specifications; strategy file is a short entry-point redirect. |
| Industrial parity roadmap/ranking | Horizons and priorities here; `industrial-parity.md` retains scope, crosswalk, W1–W9 scenario definitions, scorecard and LC1–LC13 claim rules. |
| Performance roadmap, first-twelve list, CSV/JSON | 111 PERF packages and phase map here. Audit retains measurements/designs/reproduction. CSV/JSON are generated exports, unchanged schema. |
| STAB-A1–A4 | Completed bounded promotions retained in the archived stabilization dispositions; broader evidence is B6/B17, not a repeated promotion campaign. |
| STAB-B1 torus | P-Class 2.4/2.7; tangent-cut witness alias B9. |
| STAB-B2 non-planar profiles | B12 and P-Class 7.1/7.2; preserve qualified caps and typed miter refusal. |
| STAB-B3 evolution | B5 completed bounded offset mapping; broader family attribution B18 and journaled direct edits 6.5. |
| STAB-C1 curved blends | B3 closed-rim chamfers, B4 planar completion and M5 curved remainder. |
| STAB-C2 blend resize | P-Class 6.2/6.5; preserve #346/#348 cylindrical resize/removal limits. |
| STAB-C3 IGES / C4 rendering | Recorded IGES decision / B14 render promotion. |
| Partial-cylinder resizing plan | P-Class 6.2 geometry and 6.5 history; #308/#338 quarter-wall support is bounded. Fixture and acceptance retained in the case study. |
| Hammer-holder opening checkpoint | B27 capability and B28 performance; historical chronological notes are evidence, not the latest support boundary. |
| Evolution and scale-band audit next-slice lists | B18 / 6.5 and 2.6 respectively; audits supply witnesses, this page owns selection and status. |
| P-Class 8.1 and B26 | B26 owns bounded generated/fuzz oracles; 8.1 owns sequence shrinking, nightly integration and the first-ten-defect exit. O1.2 competitor benchmarking is a separate contract. |
| P-Class 8.5 and O1.1 | Reuse O1.1 corpus infrastructure; 8.5 owns real-model operation/export and bisectable-regression acceptance, O1.1d owns recurring triage. |
| Inherited e3b colors / attribute scope | O5.2; design remains `deferred-e3b-step-names-and-colors.md`. |
| Inherited error registry / e5b | O4.4 registry, O4.7 direct-method typed results; shipped batch V2 does not close those parents. |
| Inherited conic cells | O2.2 construction/booleans; B10 distance/classification evidence. |
| Inherited GCS qualification | B16; PERF-S01–S06 cover measurement/optimization, not correctness qualification. |
| Inherited hidden-line qualification | P-Class 7.5; PERF-V07 improves prepared-query performance without closing its capability matrix. |
| Inherited multi-body mesh import | B31, explicit compatibility decision. |
| K-S1 external consumer slices | B27, 6.2/6.5, and existing fixture evidence; application acceptance remains in the consumer repository. |
| K-S2 measurement | B20, preserving completed planar line/circle/parabola boundary moments. |
| K-S3 budgets | P-Class 2.8; completed Newton/subdivision/marcher exposure remains evidence, broader adoption remains. |
| K-S4 fuzz / census | B19 plus B26 invariant campaign; retain existing NURBS/topology/serialization targets. Consumer census disposition is external evidence. |
| K-W3 WASM budgets | PERF-W01/W06 and O3.1a; current policy is defined in executable package guards, historical 8 MiB figures are not a current limit. |

## Session workflow

1. Refresh `origin/main`; inspect live open PR heads and changed files. Select
   **one** owner row and, where useful, one PERF child package. Read its linked
   specification and the roadmap skill's chase filters/terminal cases.
2. Confirm prerequisites and write the bounded acceptance criteria into the PR.
   Use an isolated branch/worktree. A Pending row may already have an open PR.
3. Verify with independent oracles and the repository checks appropriate to the
   change. Geometry changes retain native/direct/batch WASM, transactional refusal,
   scale, census and regression requirements from the existing specifications.
   A documentation edit does not establish new geometry or benchmark evidence.
4. Update state and remaining scope **here**, with the exact PR/artifact evidence.
   If blocked, record the blocker in the selected row and stop; do not switch
   items. Update capability/stability claims only when their promotion gates pass.
5. If a PERF work specification changes, regenerate its CSV/JSON exports and run
   `python3 scripts/sync-roadmap-inventory.py --check`. Keep performance evidence
   fixed to its recorded SHA; new measurements get a new evidence directory.
6. Refresh overlap before delivery. Port concurrent changes from retired ledger
   paths to the owner row here. Deliver a ready PR with exact-head check status;
   publishing/deployment and merge remain separate authorized actions.
