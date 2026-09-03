# P-Class program status

Canonical plan: [p-class-program.md](p-class-program.md). This ledger is
updated in the final PR for every issue. `Pending` means no implementation PR
has landed; it is not evidence that the issue is unowned in another worktree,
so the live open-PR inventory remains authoritative before work starts.

Issue 2.0's measured baseline is
`39c7a7b7ccbfc746ed7d9e9b8f156d54d6cfe090`.

| Issue | State | PR |
| --- | --- | --- |
| 2.0a Measurement and semantic ratchet | Merged | [#120](https://github.com/esaueng/remus/pull/120) |
| 2.0b Missing writers, invariants, oracles, and census | Complete — operations and phase-FF contributions | [#122](https://github.com/esaueng/remus/pull/122) + [#125](https://github.com/esaueng/remus/pull/125) |
| 2.0c Reader migration and seam-safe validation | Complete — the staged reader migration is at zero (132 → 0), and CI strictly validates both oriented cylinder-seam pcurves on an exact boolean output with non-vacuous counts | [#154](https://github.com/esaueng/remus/pull/154) + [#159](https://github.com/esaueng/remus/pull/159) + [#162](https://github.com/esaueng/remus/pull/162) + [#165](https://github.com/esaueng/remus/pull/165) + [#169](https://github.com/esaueng/remus/pull/169) + [#172](https://github.com/esaueng/remus/pull/172) + [#175](https://github.com/esaueng/remus/pull/175) |
| 2.0d Topology-owned atomic boundary mutation | Complete — all 30 measured production direct mutations are migrated behind two preflighted topology APIs; the ratchet requires zero and checkpoint rollback preserves boundary, pcurve, and derived-handle state | [#176](https://github.com/esaueng/remus/pull/176) |
| 2.0e Physical Loop/Coedge p-curve authority | Merged — Face boundary order and per-use pcurve/winding storage are authoritative Loop/Coedge state; arena v3 round-trips both seam branches while v1/v2 derive them compatibly | [#179](https://github.com/esaueng/remus/pull/179) |
| 2.0f STEP per-use deterministic round-trip | Merged — STEP import binds positioned pcurves to exact coedge uses, preserves analytic surface frames and periodic winding, and fails atomically on count/endpoint mismatch; export is deterministic and refuses inconsistent per-use authority | [#182](https://github.com/esaueng/remus/pull/182) |
| 2.0g Integration, zero gate, corpus, and docs | Merged — whole-topology ownership/seam diagnostics run across exact boolean, arena-v3 rollback, external 48-pcurve STEP, and WASM paths; the 132-reader and 30-mutation gates remain zero, unsafe wire mutators are deprecated, and the read-only compatibility facade is retained behind its measured public-API deletion gate | [#188](https://github.com/esaueng/remus/pull/188) |
| 2.1 Honest-failure hygiene | Merged — unsupported phase-FF pairs and every pcurve UV-projection fallback fail with pinned diagnostics instead of substituting empty sections, zero UV, or a NURBS midpoint | [#129](https://github.com/esaueng/remus/pull/129) + [#194](https://github.com/esaueng/remus/pull/194) |
| 2.2 Sphere in general position | Merged — transversal equal-radius sphere×sphere fuse, cut, and intersect retain analytic sphere patches and pass closed-form volume, classification, manifold-mesh, and WASM exact-path gates | [#199](https://github.com/esaueng/remus/pull/199) |
| 2.3 Steinmetz ellipses | Merged — perpendicular equal-radius cylinder×cylinder intersection retains six analytic cylinder patches on eight authoritative ellipse arcs, matches `16/3·r³`, and passes native, manifold-mesh, census, and WASM exact-only gates | [#205](https://github.com/esaueng/remus/pull/205) |
| 2.4 Quadric × quadric transversal | Partial — 2.4a and 2.4b merged, staged independently: 2.4a emits every bounded sphere seam-arrangement cell for exact box ∪ sphere; 2.4b emits and tessellates the complementary torus-notch band for exact torus ∩ box; general quartic seams and integration remain | [#206](https://github.com/esaueng/remus/pull/206) + [#207](https://github.com/esaueng/remus/pull/207) |
| 2.5 NURBS × NURBS booleans | Pending | — |
| 2.6 Scale-relative band audit | Pending | — |
| 2.7 Tangency and sliver contacts | Pending | — |
| 2.8 OperationContext budgets and cancellation | Partial — boolean/SSI cancellation and all six SSI work budgets are direct/batch WASM-callable; parameter-space tolerance and wider adoption remain | [PR #138](https://github.com/esaueng/remus/pull/138) + [PR #147](https://github.com/esaueng/remus/pull/147) + [PR #160](https://github.com/esaueng/remus/pull/160) + [PR #202](https://github.com/esaueng/remus/pull/202) |
| 3.1 RFC 0004 | Merged — staged per-entity tolerance semantics, authority, growth, serialization, and disclosure contract | [#126](https://github.com/esaueng/remus/pull/126) |
| 3.2 Topology substrate | Merged — RFC 0004 Stage 1: validated setters, vertex-ball/edge-tube validators, context cap, journal recordability, and byte-stable legacy arena round-trip | [#148](https://github.com/esaueng/remus/pull/148) |
| 3.3 Predicate plumbing | Merged — EE crossing/AABB, forced EE overlap, pave-vertex lookup, VE incidence, and SameParameter/SameRange validation honor declared entity tolerance while default bands and the 51-row approximation census remain unchanged | [#208](https://github.com/esaueng/remus/pull/208) |
| 3.4 GFA integration | Pending | — |
| 3.5 Import and sew integration | Pending | — |
| 3.6 Downstream disclosure | Pending | — |
| 4.1 RFC 0005 | Merged — staged solid/sheet/wire/general-body semantics, side-of sheet classification, Compound-first cellular results, STEP mapping, and evolution contract | [#127](https://github.com/esaueng/remus/pull/127) |
| 4.2 Sheet bodies first-class | Implemented — body-class validation, transactional construction, area/bounds/center, typed volume refusal, boundary-preserving tessellation, arena-v4 roots, and direct/batch WASM are joined by deterministic `SHELL_BASED_SURFACE_MODEL` exchange over open or closed shells; the trimmed-NURBS implementation exit witness is green | [#209](https://github.com/esaueng/remus/pull/209) + [#210](https://github.com/esaueng/remus/pull/210) + [#211](https://github.com/esaueng/remus/pull/211) + [#212](https://github.com/esaueng/remus/pull/212) + [#213](https://github.com/esaueng/remus/pull/213) |
| 4.3 Split solid by sheet | Implemented — GFA uses a first-class cylindrical sheet as a non-volumetric face-set tool; the resulting Compound contains two deterministic, individually valid cells whose closed-form volumes reconstruct the input, with native/direct/batch WASM parity and typed refusals outside the bounded subset | [#214](https://github.com/esaueng/remus/pull/214) |
| 4.4 Trim sheet by solid / sheet × sheet | Implemented — validated keep-inside/keep-outside solid trims plus effective-normal one-way and strict mutual planar sheet trims have native/direct/batch WASM parity; six boundary-trimmed sheets sew into a deterministic valid six-face solid whose exact volume matches `make_box`, while curved and multi-face sheet pairs remain unqualified | [#215](https://github.com/esaueng/remus/pull/215) + [#216](https://github.com/esaueng/remus/pull/216) |
| 4.5 Imprint | Implemented — a planar solid tool splits target faces without discarding material; the new validated solid preserves exact volume, journals only construction-derived Modified/Generated/Preserved events, resolves an anchored split face BoundMany, matches direct/batch WASM, and refuses unqualified configurations transactionally | [#217](https://github.com/esaueng/remus/pull/217) |
| 4.6 Multi-region boolean output | Implemented — exact two-solid booleans return a Compound of independently validated regions with deterministic cavity assignment and total per-region construction lineage; bounded pairwise-disjoint Compound operands add member-preserving fuse, distributed intersect, and distributed single-tool cut with native/direct/batch WASM parity. Intersecting-member fuse and multi-tool cut fail closed pending recursive lineage composition | [#218](https://github.com/esaueng/remus/pull/218) + [#219](https://github.com/esaueng/remus/pull/219) |
| 4.7 Wire bodies | Implemented — body-level length and existing copy/transform semantics are joined by additive arena-v5 standalone wire roots plus validation-gated closed-planar wire sweep; native/direct/batch WASM match exact perimeter and prism-volume oracles, while open and non-planar profiles refuse transactionally | [#222](https://github.com/esaueng/remus/pull/222) |
| 5.1 Variable-radius qualification | Implemented — standard-law whole-domain bounds and typed collapse/local-limit refusals guard every walker station; the straight-edge perpendicular-plane linear band matches its analytic surface and closed-form volume, while S-curve samples preserve radius and both support tangencies. Opaque custom callbacks are preserved and station-checked rather than endpoint-linearized, but arbitrary between-sample certification and trimmed-solid assembly remain explicitly unqualified | [#226](https://github.com/esaueng/remus/pull/226) |
| 5.2 Curved-support blends | Implemented — constant-radius closed rims on qualified cylinder/cone, cylinder/sphere, cone/cone, and segmented cylinder/cylinder supports assemble exact toroidal shoulders where provable and periodic walking-NURBS bands otherwise; unsupported support combinations and closed legacy spines fail typed | [#228](https://github.com/esaueng/remus/pull/228) |
| 5.3 General vertex blends | Implemented — same-radius planar N-way corners with one connected material-side orientation produce analytic sphere caps, cylindrical stripes, and trimmed ellipse runouts with native/direct/batch WASM and G1/watertightness witnesses; mixed-side, non-planar, and variable-radius corners remain unqualified | [#231](https://github.com/esaueng/remus/pull/231) |
| 5.4 Setbacks | Implemented — physical straight-spine setbacks crop variable S-curve bands to a stationary common-radius planar corner ball; the three-edge exit witness pins result stations, G1, topology, mesh/volume, census, and direct/batch WASM parity, while incompatible declarations refuse transactionally | [#232](https://github.com/esaueng/remus/pull/232) |
| 5.5 Overflow and cliff handling | Implemented, in review — v2 fillets stop transactionally at planar support boundaries, inner-loop obstacles, closed-rim wall exhaustion, paired bands consuming one wall, and inward cap collapse with typed edge/face/requested/available metadata and stable native/WASM parity; actual rollover remains unqualified pending 6.1 | [#235](https://github.com/esaueng/remus/pull/235) |
| 5.6 Face-face blends and hold lines | Pending | — |
| 5.7 Offset self-intersection removal | Pending | — |
| 6.1 Replace-surface re-limitation | Pending | — |
| 6.2 Generalized move / rotate / offset face | Pending | — |
| 6.3 Curved delete-face-and-heal | Pending | — |
| 6.4 Curved-face draft | Pending | — |
| 6.5 Journaled direct edits | Pending | — |
| 7.1 Guided sweeps | Pending | — |
| 7.2 Loft continuity and periodic lofts | Pending | — |
| 7.3 Constrained N-sided fill | Pending | — |
| 7.4 Surface extension and curve imprint | Pending | — |
| 7.5 Interrogation | Partial — curvature analysis slice (`analyze::curvature` + `getFaceCurvature`/`getFaceMinRadius`); clash, silhouettes, draft pending | — |
| 8.1 Differential testing harness | Pending | — |
| 8.2 Performance budget gates | Pending | — |
| 8.3 Parallel tessellation | Pending | — |
| 8.4 Parallel boolean internals | Pending | — |
| 8.5 Real-model corpus | Pending | — |
