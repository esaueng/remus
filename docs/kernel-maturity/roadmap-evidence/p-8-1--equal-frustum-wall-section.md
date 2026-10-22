---
row: p-8-1
---

PR #1235 repairs the retained F17 equal-frustum fuse through the qualified
equal-cone wall section and complete lateral split. The native shape regression
checks independent material/quantity probes, strict validity, opposing uses,
raw closed meshes, operand preservation, scale and placement controls.
`regress_equal_cone_section.rs` and the phase-FF helper oracles retain the
section's bounded applicability; the previously merged oblique cylinder-notch
splitter regression remains active alongside it. Native campaign replay and
the WASM batch cross-check pin F17 as successful, while
`scripts/opseq-f17-packaged.mjs` retains direct and both batch API cells for
fresh and installed consumer qualification. Broader cone-cone campaign
coverage and unqualified section/split cases remain open. Tier: core.
