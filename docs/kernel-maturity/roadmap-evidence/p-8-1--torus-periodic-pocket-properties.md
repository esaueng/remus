---
row: p-8-1
---

Retained OpSeq seed `6472044787542844054` reduces to a translated torus
(`R=5`, `r=2`, centre `(3, 1.5, -0.5)`) cut by a radius-5.5, height-7.5
cylinder at the origin. The exact result retains one plane, two cylinders
and one torus; material probes and indexed mesh incidence separate the
measurement defect from missing geometry. Before the repair, all tested
Gauss orders return mass `382.0000493337`. An independent integral of exact
horizontal annulus/disc sections converges from 512 to 2048 Simpson panels
at volume `336.2866423605142`. This convergence is numerical evidence, not
a certified error bound for a general trimmed surface.
The repaired retained mass is `336.2869006031` (relative residual `7.68e-7`);
the sampled trim's remaining chord error is kept visible.

The torus integrator aligns narrower, nonwrapping pocket images with an
already resolved one-period chart. Pocket `u` branches are normalized before
quadrature breaks are built; tube-window clipping preserves polygon/half-plane
intersection breaks, including disjoint concave pieces. Every original hole
must qualify before expansion. Wider or wrapping families retain their prior
path; the existing 4096-point ceiling also bounds clipped images and refuses
without partial assignment. Sampling, quadrature orders, modelling tolerance,
GFA geometry and tessellation policy are unchanged.

`regress_opseq_torus_cylinder_properties.rs` retains the independent section
oracle, mass/centroid checks, similarity transforms, analytic carriers,
material probes, input preservation and cyclic seam starts. Controls distinguish
both periodic seams, multiple pockets, clipping, orientation, missing or
unsupported pockets, reordered-hole admission and budget exhaustion. Direct
measurement bodies and both native batch routes are covered in the WASM
measurement tests. `scripts/opseq-torus-properties-packaged.mjs` exercises the
actual direct and both batch APIs with separate mass accuracy and indexed-mesh
convergence assertions in fresh and installed package consumers.

Scale controls transform the retained exact result: constructing this boolean
directly at scales `1e-3` and `1e3` already refuses on the frozen source and
remains outside this property repair. The sampled trim's chord residual and
general quartic integration remain unqualified. The original sphere/torus
quantity report, torus/sphere mesh report and six offset-contract tripwires
remain separate; this does not close the whole OpSeq campaign. Tier: core.
