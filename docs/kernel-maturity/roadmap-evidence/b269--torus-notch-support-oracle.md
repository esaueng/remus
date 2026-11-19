---
row: B269
---
Exit met 2026-11-18 (PR #1869): the ignored witness is un-ignored and green. It already passes on main at 4b81ce74 (not bisected): the prepared
and one-shot classifiers both read (0, 1.3, 0) and (−1, 0, 0.3) `OnBoundary`.
The witness (`qualify_edge_relations_tests.rs::ray_crossing_torus_tests::ray_crossing_torus_notch_curved_boundary_ready_repro`)
is now held by a closed-form oracle,
`torus_notch_support_reads_boundary_at_every_scale`: the same batch cut at
scales 0.1, 1 and 10, unplaced and rigidly placed, with 108 points of the
retained torus support (`u` over the half turn opposite the notch, `v` all
round the tube) `OnBoundary`, the tube 0.03 inside and outside them `Inside`
and `Outside`, and the removed tube `Outside`, through the prepared and the
one-shot classifier alike.
