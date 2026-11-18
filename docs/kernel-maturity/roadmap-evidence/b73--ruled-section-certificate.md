---
row: B73
---
Ruled-section certificate (2026-11-18, PR #1852, merge
`c7e2dc168d232918652e1084300bd7a12216f4bc`): the iso-curve shortcut of #1850
now proves, over every span (`nurbs::reuse::certify_ruled_offset_section`),
that each ruling crosses the cylinder or forward cone exactly once and that the
emitted section lies within its band of that crossing, falling back to the
general intersection when the bound is inconclusive; a weighted quadratic wall
that hides a gap between sampled rulings takes the fallback
(`ruled_offset_tests.rs::a_rational_span_can_hide_a_gap_between_sampled_rulings`).
