---
row: B263
---
Gauge plane (2026-11-13, PR #1705, from a review of #1701): a tapered thread
fitted to its cylinder (no `majorDiameter`) builds its radii on the start
plane straight from the cylinder, so the gauge plane changes only the
reported gauge-plane diameters; before, a far `gaugeOffset` cancelled two
huge radii and built a quietly wrong valid body (`1e17` on an r6 boss: a
ratio-0.1 taper, 7% less volume). A `|gaugeOffset|` beyond
`MAX_GAUGE_OFFSET_PER_SCALE` (100) times the larger of the thread's length
and the cylinder's radius refuses with `thread:invalid-parameter`. Tests:
`thread/tests.rs::a_fitted_taper_is_the_same_body_whatever_its_gauge_plane`
(external with either small end and internal, gauges up to ±599 against the
default gauge bit for bit and the closed-form volume) and
`a_gauge_plane_far_beyond_the_thread_refuses`. Open findings from the same
review: [B277](#b277), [B278](#b278).
