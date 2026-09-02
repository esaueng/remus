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
| 2.3 Steinmetz ellipses | In review — perpendicular equal-radius cylinder×cylinder intersection retains six analytic cylinder patches on eight authoritative ellipse arcs, matches `16/3·r³`, and passes native, manifold-mesh, census, and WASM exact-only gates | [#205](https://github.com/esaueng/remus/pull/205) |
| 2.4 Quadric × quadric transversal | In review — staged independently: 2.4a emits every bounded sphere seam-arrangement cell for exact box ∪ sphere; 2.4b emits and tessellates the complementary torus-notch band for exact torus ∩ box; general quartic seams and integration remain | [#206](https://github.com/esaueng/remus/pull/206) + [#207](https://github.com/esaueng/remus/pull/207) |
| 2.5 NURBS × NURBS booleans | Pending | — |
| 2.6 Scale-relative band audit | Pending | — |
| 2.7 Tangency and sliver contacts | Pending | — |
| 2.8 OperationContext budgets and cancellation | Partial — boolean/SSI cancellation plus coupled SSI Newton and recursive seed-subdivision budgets complete; parameter-space budgets and wider adoption remain | [PR #138](https://github.com/esaueng/remus/pull/138) + [PR #147](https://github.com/esaueng/remus/pull/147) + [PR #160](https://github.com/esaueng/remus/pull/160) |
| 3.1 RFC 0004 | Pending | — |
| 3.2 Topology substrate | In review — RFC 0004 Stage 1: validated setters, vertex-ball/edge-tube validators, context cap, journal recordability | — |
| 3.3 Predicate plumbing | Pending | — |
| 3.4 GFA integration | Pending | — |
| 3.5 Import and sew integration | Pending | — |
| 3.6 Downstream disclosure | Pending | — |
| 4.1 RFC 0005 | Pending | — |
| 4.2 Sheet bodies first-class | Pending | — |
| 4.3 Split solid by sheet | Pending | — |
| 4.4 Trim sheet by solid / sheet × sheet | Pending | — |
| 4.5 Imprint | Pending | — |
| 4.6 Multi-region boolean output | Pending | — |
| 4.7 Wire bodies | Pending | — |
| 5.1 Variable-radius qualification | Pending | — |
| 5.2 Curved-support blends | Pending | — |
| 5.3 General vertex blends | Pending | — |
| 5.4 Setbacks | Pending | — |
| 5.5 Overflow and cliff handling | Pending | — |
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
