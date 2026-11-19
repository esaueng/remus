---
row: B278
---
**Closed as a typed refusal (2026-11-19).** Root cause: nothing was missing
from the tapered path. At thread tolerance 1e-8 the two runout-zone flanks
of the body are bicubic carriers of 2184 × 112 spans (692 × 35 at 1e-6),
trimmed by about twenty conical-helix fits. Their knot-aligned quadrature
needs more Gauss lines than the 2¹⁸-cell cap allows, so the integrator
refused with `IntegrationFailed`. `solid_volume` (route "NURBS-trimmed
quadric wall") then took the closed whole-solid mesh at the requested
1e-4 deflection, whose chord error left the volume 1.05e-5 low. The untapered
body refused only because its mesh was open, so its route ended at the exact
fallback, which refused. Fix: `face_integrator::is_knot_aligned_work_refusal`
(remus-check) names that refusal. `qualified_shell_volume`
(`measure/volume.rs`) now returns it instead of declining to the mesh, at all
three orders it reads, as `mass_properties` already did. Other face-integral
failures still decline to the mesh, among them the 4096-point trim budget and
undecided loop nesting. Witness
`thread::tests::a_long_tapered_runout_measures_the_same_at_a_tight_tolerance`
un-ignored. The 1e-6 body measures 5625.755231 on its face integrals. The
1e-8 body is refused by both `solid_volume` and `mass_properties`. Measuring
the tight fit would need the cap raised about fourfold (an estimated 2.8M Gauss
lines on each flank).
