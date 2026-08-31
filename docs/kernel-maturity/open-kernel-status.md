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
| O1.1b Corpus manifests + fetcher | A | Blocked 2026-08-31 — the [official NYU ABC host](https://archive.nyu.edu/bitstream/2451/44309/3/abc_0000_step_v00.7z) currently restricts STEP-archive downloads, so the required verified 1,000-model id/URL/SHA-256/size manifest cannot be produced without redistributing corpus files; resume when the official bitstreams are downloadable | — |
| O1.1c Gauntlet CI wiring | A | Pending | — |
| O1.1d Triage loop (recurring) | A | Pending | — |
| O1.2a Head-to-head protocol + runners | B | Pending | — |
| O1.2b Head-to-head scenario set | B | Pending | — |
| O1.2c Head-to-head results page | B | Pending | — |
| O1.3a Fillet torture corpus + runner | A | Complete — 10 named cases built-and-verified or transactionally refused with stable codes | [#139](https://github.com/esaueng/remus/pull/139) |
| O1.3b Fillet torture publication | C | Pending | — |
| O1.4a STEP validation properties | A | Pending | — |
| O1.4b CAx-IF test-round manifest | B | Pending | — |
| O2.1a RFC 0006 swept analytic surfaces | A | Pending | — |
| O2.1b Revolution/extrusion math substrate | A | Pending | — |
| O2.1c FaceSurface variants + site audit | B | Pending | — |
| O2.1d Revolution/extrusion I/O wiring | B | Pending | — |
| O2.1e Revolution/extrusion boolean arms | B | Pending | — |
| O2.2 Conic edges through booleans | B (M2 track) | Pending | — |
| O2.3a Splitter inventory + design note | A | Pending | — |
| O2.3b UV-arrangement core | B | Pending | — |
| O2.3c Winding classification bridge | B | Pending | — |
| O2.3d Special-case migration + ratchet | B | Pending | — |
| O3.1 Inner-loop benches (math/algo/blend) | A | Pending | — |
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
