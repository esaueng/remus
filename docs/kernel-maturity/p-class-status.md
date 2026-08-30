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
| 2.0c Reader migration and seam-safe validation | Pending | — |
| 2.0d Topology-owned atomic boundary mutation | Pending | — |
| 2.0e Physical Loop/Coedge p-curve authority | Pending | — |
| 2.0f STEP per-use deterministic round-trip | Pending | — |
| 2.0g Integration, zero gate, corpus, and docs | Pending | — |
| 2.1 Honest-failure hygiene | In review — part 1 (phase-FF unsupported-pair typed refusal); part 2 (pcurve UV-projection refusals) sequenced behind 2.0c's reader migration (same files) | #129 |
| 2.2 Sphere in general position | Pending | — |
| 2.3 Steinmetz ellipses | Pending | — |
| 2.4 Quadric × quadric transversal | Pending | — |
| 2.5 NURBS × NURBS booleans | Pending | — |
| 2.6 Scale-relative band audit | Pending | — |
| 2.7 Tangency and sliver contacts | Pending | — |
| 2.8 OperationContext budgets and cancellation | Partial — boolean/SSI cancellation; Newton and parameter-space budgets remain | [PR #138](https://github.com/esaueng/remus/pull/138) |
| 3.1 RFC 0004 | Pending | — |
| 3.2 Topology substrate | Pending | — |
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
