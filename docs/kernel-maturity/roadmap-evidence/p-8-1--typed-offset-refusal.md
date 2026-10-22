---
row: p-8-1
---

PR #1224 retains the F7 inward box-cylinder offset as a typed transactional
assembly refusal. Lower `thick_solid`/`offset_solid` routes roll back temporary
entities; the operations adapter preserves `OffsetError::AssemblyFailed`
rather than calling it invalid caller input. Genuine invalid-distance controls
retain their category. Native regressions cover three widths and inward
distances, unit/1000 scale, rigid placement and seven lower/operations routes,
with independent source material/quantity checks and live-state preservation.
`scripts/opseq-offset-refusal-packaged.mjs` carries direct, detailed, journaled
and both batch contracts into fresh/installed consumer qualification. The wire
loop still fails for this configuration; broad X6/X8 and unqualified offset
cells remain open. Tier: core (live-session rollback and error contracts).
