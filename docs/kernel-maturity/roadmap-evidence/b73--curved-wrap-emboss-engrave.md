---
row: B73
---
Exact emboss and engrave of curved wraps (2026-11-17, PR #1850, merge
`a8e9a3acd79bde9765e82d17bf23ceeb73d3fdd3`): under the exact-only policy the
boolean refused every wrap with a curved edge against its own body
(`ExactOnlyUnattainable`); straight-edged wraps already worked. Phase FF now
takes the section of a ruled offset wall with a cylinder or cone as the wall's
iso-curve `S(·, v*)` (`exact_ruled_offset_section`) when every sampled ruling
crosses the carrier strictly and the crossings agree on one `v*` (8 rulings
and 32 iso-curve points per knot span, band scaled to the control net's own
size); any other wall keeps the general intersection. The wrap splits closed
profile edges into two halves so every wall is a four-sided patch. Tests:
`wrap/tests.rs::a_wrapped_disc_embosses_and_engraves_exactly_across_the_seam_or_beside_it`
(a radius-2 disc cut into and fused onto a radius-10 post, across its seam and
a quarter turn away: valid, `mass_properties` within 1e-8 relative of
`A·(r₂² − r₁²)/2R`, measured 5e-11), `a_closed_profile_edge_wraps_to_two_walls`,
`wrap/cone/tests.rs::a_disc_wrapped_on_a_frustum_embosses_and_engraves_as_one_body`
(valid single bodies; no closed-form volume asserted) and
`algo/src/pave_filler/phase_ff/ruled_offset_tests.rs`. The emboss or engrave is
the consumer's union or cut with the wrap solid; no depth-parameterized kernel
operation was added.
