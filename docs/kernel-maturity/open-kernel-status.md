# Open Kernel Program status

Canonical plan: [open-kernel-implementation.md](open-kernel-implementation.md)
(strategy in [open-kernel-program.md](open-kernel-program.md)). This ledger is
updated in the final PR for every issue. `Pending` means no implementation PR
has landed; the live open-PR inventory remains authoritative before work
starts (R6). Wave assignments and the cross-program conflict table live in
the implementation plan (§W, §X) — check both before claiming an issue.

Owner-gated rows require an explicit maintainer decision recorded in the PR
that flips them; agents do not flip them autonomously.

| Issue | Wave | State | PR |
| --- | --- | --- | --- |
| O1.1a Gauntlet pipeline skeleton | A | Complete — isolated bounded workers run import, validation, disclosed probe boolean, manifold tessellation, and property-checked STEP round-trip; JSONL and aggregate JSON/Markdown outputs use stable taxonomy codes | [#164](https://github.com/esaueng/remus/pull/164) |
| O1.1b Corpus manifests + fetcher | A | Complete — pinned 50-model smoke, 1,000-of-10,000 ABC, and 113-model MAMBO manifests; archive/member SHA-256 verification, content-addressed caching, deterministic sampling, and typed source refusals; no corpus bytes committed | [#166](https://github.com/esaueng/remus/pull/166) |
| O1.1c Gauntlet CI wiring | A | Complete — nightly smoke and weekly abc-1k schedules publish reproducible aggregate scoreboards and append-only per-stage trends; a 0.50pp drop fails while still publishing the red aggregate | [#171](https://github.com/esaueng/remus/pull/171) |
| O1.1d Triage loop (recurring) | A | Partial — 1/5 required classes closed: generic period-winding `FACE_BOUND` bands now reconstruct exact analytic seams or refuse transactionally; pinned smoke manifest `779fcc7f…` at `a36bddac` moved `invalid_input` 14→10 and full passes 26/50→29/50 | [#177](https://github.com/esaueng/remus/pull/177) |
| O1.2a Head-to-head protocol + runners | B | Pending | — |
| O1.2b Head-to-head scenario set | B | Pending | — |
| O1.2c Head-to-head results page | B | Pending | — |
| O1.3a Fillet torture corpus + runner | A | Complete — 10 named cases built-and-verified or transactionally refused with stable codes | [#139](https://github.com/esaueng/remus/pull/139) |
| O1.3b Fillet torture publication | C | Pending | — |
| O1.4a STEP validation properties | A | Complete — opt-in CAx-IF validation properties round-trip aggregate and per-solid area, volume, centroid, and bounding boxes with derived units; malformed properties refuse transactionally with stable diagnostics, and direct/batch WASM contracts preserve import diagnostics | [#180](https://github.com/esaueng/remus/pull/180) |
| O1.4b CAx-IF test-round manifest | B | Pending | — |
| O2.1a RFC 0006 swept analytic surfaces | A | Complete — the accepted design preserves STEP parameterization with self-contained math-layer profiles, checked projection, exact lowering/recognition, typed unsupported paths, staged R8 contracts, and a measured disposition for all 92 production `FaceSurface` wildcard matches | [#183](https://github.com/esaueng/remus/pull/183) |
| O2.1b Revolution/extrusion math substrate | A | Complete — self-contained swept profiles plus revolution and linear-extrusion carriers provide checked evaluation/projection, exact first and second derivatives, curvature, explicit periods, and exact directed finite-span rational NURBS lowering; scale, seam, pole, reversed-span, success, and typed-refusal properties pin all six profile variants without adding topology variants | [#189](https://github.com/esaueng/remus/pull/189) |
| O2.1c FaceSurface variants + site audit | B | Pending | — |
| O2.1d Revolution/extrusion I/O wiring | B | Pending | — |
| O2.1e Revolution/extrusion boolean arms | B | Pending | — |
| O2.2 Conic edges through booleans | B (M2 track) | Pending | — |
| O2.3a Splitter inventory + design note | A | Complete — all ten callable special-case entry points are mapped to their geometric gates and direct or foil fixtures; the accepted design defines an exact-refined deterministic DCEL with certified event identity, periodic seam/pole quotienting, typed failures, property gates, and a staged three-entry-point deletion floor; positive isolation gaps for sector splitting and boundary chaining are explicit | [#193](https://github.com/esaueng/remus/pull/193) |
| O2.3b UV-arrangement core | B | Pending | — |
| O2.3c Winding classification bridge | B | Pending | — |
| O2.3d Special-case migration + ratchet | B | Pending | — |
| O3.1 Inner-loop benches (math/algo/blend) | A | Complete — measured 64-cut and Gridfinity flamegraphs declare a 3% inclusive threshold; every qualifying stack family plus the prerequisite NURBS, SSI, Bézier clipping, CDT, GFA, and blend-walker loops now has a Criterion baseline wired into local comparison and hosted trend tracking | This PR |
| O3.2 Journal-invalidated spatial cache | B | Pending | — |
| O3.3 SIMD in NURBS evaluation | B (evidence-gated) | Pending | — |
| O4.1a Facade crate + Model type | A | Pending | — |
| O4.1b Facade examples | A | Pending | — |
| O4.1c WASM delegation to facade | B | Pending | — |
| O4.2a Publish dry-run readiness | B | Pending | — |
| O4.2b Tag-driven release automation | B | Pending | — |
| O4.2c First publish | owner-gated | Pending | — |
| O4.3a Python core binding | B | Pending | — |
| O4.3b Python wheels + CI | B | Pending | — |
| O4.3c PyPI publish | owner-gated | Pending | — |
| O4.4 Stable error-code registry (e5b) | A | Pending | — |
| O5.1a STEP assembly reader | A | Pending | — |
| O5.1b STEP assembly writer | A | Pending | — |
| O5.1c Assembly WASM + batch | A | Pending | — |
| O5.2 Colors/names/attribute scope (e3b) | B | Pending | — |
| O5.3a AP242 writer schema | B | Pending | — |
| O5.3b PMI read, ref-anchored | C | Pending | — |
| O5.3c PMI write | C | Pending | — |
| O6.1 Docs site | A | Pending | — |
| O6.2 Browser playground | B | Pending | — |
| O6.3 Second-consumer track (ongoing) | rolling | Pending | — |
| O6.4 Contribution posture | A | Pending | — |
| O7 RFC 0007 mesh+B-Rep hybrid | C (after M4) | Pending | — |
