---
row: p-7-5
---

Tier: native. The first draft-analysis increment provides read-only Rust
surface samples and exact per-face reports for all-planar solids, including
cavity shells. Browser analysis belongs in the planned lazy repair/analysis
module; this increment does not add an eager kernel binding.

Angles are signed radians, zero on a wall parallel to the pull direction,
positive toward pull and negative away from pull. Surface samples use the
face's effective orientation and are explicitly carrier queries, not trimmed
membership or whole-face extrema. Singular NURBS charts refuse instead of
using the normal evaluator's fallback; internal knots permitting discontinuous
first derivatives refuse rather than selecting a one-sided normal. Mixed curved solids refuse the
whole-solid constant-angle report; they can be sampled explicitly through
the surface query. General draft maps, undercut accessibility, pullability,
and curved-face extrema remain open.

Eight focused regressions in `crates/check/src/analyze/draft/tests.rs` cover
independent slope/analytic-angle formulas, face and pull reversal, rigid
placement and three geometric scales, extreme finite direction magnitudes,
NURBS native domains and discontinuous knots in both axes, cavity traversal,
typed refusals and unchanged mutation ticks. All 381 check-crate tests and
all-target/all-feature Clippy pass locally; browser/package qualification is
outside this native-only increment.
