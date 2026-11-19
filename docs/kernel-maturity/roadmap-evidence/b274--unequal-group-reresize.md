---
row: B274
---

Root cause: along the x-axis band the boss wall's periodic seam passes
through the shoulder's sharp end vertex (6, 0, 24). After the R1.5 rebuild
the shoulder torus's contact on the boss runs on past that seam (the walked
transition replaces the sampled corner patch the seam used to land on), so
the seam splits the contact in two. The group removal emitted the whole
shoulder arc for both pieces, and the boss loop did not close
(`blend_cluster/remove.rs`). Now, when a loop arrives at and leaves from the
same end vertex of an open sharp edge, that contact contributes nothing.
`carry_member_identities` (`blend_cluster/edit.rs`) also matches a band of
several faces (straight run plus walked transition) face by face, on carrier
kind and the supports each face touches, or else the blends it touches. The
re-edit keeps the band's handle, and the transition's handle carries on to
the equal-radius corner patch when the band returns to R4. Witnesses:
`blend_group_edit.rs::convex_band_re_edits_after_an_unequal_group_rebuild`
covers both bands, R4 → 1.5 → 1.9 → 4 → 1.5. Each step is checked against a
fresh cascade on the sharp part (F/E/V, census, volume to 1e-9) and the
verification bar, and must remove back to the sharp part. The un-ignored
`gecko_edit_tests.rs::gecko_convex_reedit_after_unequal_group_rebuild` runs
R4 → 1.5 → 1.9 → 4 through `resizeBlendJournaled` and checks the history
from the import after every edit. Remaining: the two boss–shoulder contact
edges along X share one face pair, so an edit from the R1.5/R1.9 body
records them as unresolved (ambiguous incidence, the B18 contract).
