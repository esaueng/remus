---
row: B266
---
**Done (2026-11-18, [#1867](https://github.com/esaueng/remus/pull/1867)):** the witness is un-ignored and green. Two causes in phase FF: the side face's plane ×
torus oval enters the box face for an arc far shorter than the in-both
prefilter's sample pitch (`1.3e-2` of a `2.2` oval at a depth of `1e-5`), so
the prefilter dropped it; and `trim_torus_oval_to_box_face` placed the
crossings on the oval by a 1024-sample nearest parameter, too coarse for such
an arc (the re-sampled arc ran past its exact ends and the refit folded back,
or ran round the whole oval). Now the prefilter keeps a closed torus × plane
section that a resolved box-edge crossing places inside both face boxes, the
trimmer settles close crossings' parameters by a golden-section closest-point
search, and a kept arc whose ends lie on one straight edge is read against the
section tolerance model: within `band` it is dropped (no material past
tolerance), thinner than `separate` (`1e-5`) it is refused by name
(`IntersectionFailed`, which the exact-only boolean reports as
`ExactOnlyUnattainable`), thicker it is cut. Depths `3e-7` to `1e-5` at scales
0.1 to 100 are exact or refused, never the unchanged box; `3e-5` and `1e-4` are
exact at every scale, with a closed mesh, ray-cast material probes against the
tube's signed distance, and the notch (the intersection of the same operands)
within `1.5e-5` of a slicing integral, now taken piecewise between the
integrand's breaks with a cancellation-free cap area
(`regress_torus_contact_windows.rs::shallow_torus_notch_on_a_box_edge_is_not_dropped`,
`shallow_torus_notches_across_scales_are_exact_or_refused`).
Review fixes: the lens is measured against the face outline's straight edges as segments, so a grazed edge split into collinear pieces between the crossings is refused by the same gate (`shallow_notch_across_a_split_grazed_edge_is_exact_or_refused`, `helper_oracle_tests::a_shallow_lens_on_a_box_edge_is_read_across_collinear_edges`); only crossings next to a short gap on a straight-edged face are settled, so disk caps and far crossings keep their dense parameters (`only_crossings_next_to_a_close_gap_are_settled`).
Review re-check: the lens is read only against the straight edges its ends lie on (their common line when collinear, else the two edges as segments), so a nearer unrelated edge cannot shrink it (`a_lens_is_read_against_its_own_edge_not_the_nearest_one`), and any non-finite end or sample reading refuses (`an_arc_with_a_non_finite_reading_is_unreadable`).
