---
row: B283
---
Done (2026-11-23, [#1974](https://github.com/esaueng/remus/pull/1974)). Cause: the periodic chart splitter splits the tool's torus along its seam
circle, leaving a two-edge lens bounded by the seam circle and the
torus × torus section spline. Shell assembly's `merge_duplicate_edges`
guarded wire-collapsing line × conic and line × NURBS pairs but not
conic × NURBS, so it merged the section spline into the seam circle, which
collapsed the lens into a spur and left an open four-face body. A
conic × NURBS pair that would collapse a wire now merges only when the spline
is a certified fitted copy of the conic arc: over its trimmed range, cut into
rational Bézier spans halved until decided, the conic's implicit is bounded by
Bernstein coefficients and turned into a distance bound, the out-of-plane
offset by the control hull and the branch by unwrapped angular wedges, all
within the 1e-4 fitted-seam band (`algo/src/builder/builder_solid/conic_fit.rs`).
A lens wider than the band stays two edges wherever it falls between samples, and if a pair
left apart is then a free edge, assembly refuses (`AssemblyFailed`) rather than
return an open body. The imported pair (`tests/data/issue_1833_crossing_sweep_tool.step`
with the B76 target) now cuts to a closed five-face plane/torus B-rep through
`boolean`, `boolean_with_options` (four faces after unification),
entity-evolution and journaled entry points; intersect (formerly
`ExactOnlyUnattainable`), fuse and the reverse cut build exact in both orders,
and the natively revolved pair matches. Every result has watertight meshes at
three deflections, no dropped sections, material probes inside, outside and in
each leg, and a converged mesh volume within 5e-3 of the reference (cut
202.862130 against 202.862135; cut + common and fuse + common close to 5e-3).
`solid_volume` reads these torus faces with fitted trims up to 1.6e-4 relative
low (cut 202.8387, fuse 472.335 against 472.411), a measurement limit of that
face class, not of the B-rep (`tests/regress_issue_1833_crossing_sweeps_cut.rs`).
