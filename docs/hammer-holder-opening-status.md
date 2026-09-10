# Hammer-holder opening: kernel repair checkpoint

The imported Shapr3D holder has a nominal 46 mm opening, 74 mm outside width,
58 mm height and two 5 mm mounting bores. Import is supported. The complete
46 -> 50 mm opening reconstruction is **not yet supported**; no application
parameter or AI editing feature is enabled by this repair.

## Verified cut and intersection

The left mask spans x = [-18, 11], y = [-10, 43], z = [0, 70] mm.
Both `source - mask` and `source intersect mask` now pass strict validation,
produce watertight meshes, preserve the source's arena serialization, and
survive STEP export/reimport. The cut preserves both mounting bores. The
intersection retains the complementary NURBS lettering branches and their
planar faces: 101 faces, matching the independently computed reference.

The repairs address several separate defects:

- Planar split loops were wound against the reversal-adjusted normal instead
  of the stored normal, applying reversal twice.
- CommonBlock replacement and edge merging confused distinct curved branches
  sharing endpoints. Each NURBS branch now retains its own representative;
  geometric membership handles different parameter speeds on the same curve.
- The splitter dropped holes bounded by two open curved edges because their
  vertex count was below three. Such loops can enclose real area.
- Rebuilt and vertex-welded boundaries discarded the source edge's recorded
  tolerance. Copied edges retain that contract; generated edges retain the
  operation tolerance. The source allowance is not multiplied by the normal
  endpoint guard.
- Coplanar NURBS sampling reconstructed parameter intervals instead of using
  stored intervals, folding reversed trimmed loops during comparison.
- A perpendicular cylinder and torus with merely touching axial slabs were
  marched into spurious loops. An exact slab check recognizes that these
  carriers have no one-dimensional intersection. Even a small true overlap
  retains the general intersection path.

The native fixture regressions are in
`crates/io/tests/hammer_opening_partition.rs`. Focused unit tests exercise the
branch, hole, parameter-range, tolerance and tangency cases independently.
`scripts/test-wasm-smoke.mjs` checks the cut and intersection through the
shipped split kernel/translator API, including strict validation, source
preservation, mesh closure and STEP round trips.

## Boundary-audit corrections

The original refusal near (-12.8034951482, 28.5433214964, 16.9921788191) mm was
not evidence of an outside-source point. An independent reference places it on
the original trimming curve. The distance query returned its carrier-surface
projection before considering the closer trim. It now considers both, plus
stored topological vertices when curve endpoints differ within the import's
recorded tolerance.

The audit also used standalone face tessellation, which can sample the full
carrier of a trimmed toroidal patch. It now uses the owning solid's trimmed
mesh and visits each referenced vertex once. The existing sample and distance
work limits, containment tolerance, classification rules and independent
volume audit remain in place. The cached operand classifiers avoid repeated
operand tessellation.

## Remaining blocker and reproduction

Run:

```sh
cargo run --profile ci-test -p remus-io --example hammer_opening
```

The diagnostic now passes the first cut, both left-partition intersections,
and the left-side fuse, including the holder translated by -2 mm in X. The
fuse has 177 faces and passes strict validation, mesh closure, bore preservation
and STEP round-trip checks. The next operation, cutting the right mask from
that result, still refuses a 162-face candidate with seven free boundary edges
and one non-manifold edge. The replay stops at that failure. No approximate
fallback is permitted. The sections below record the successive repair
checkpoints.

An equivalent subtraction construction was also examined:
`current - (mask - translated_source)`. Its first tool cut fails strict sphere
inner-wire orientation checks, so it is not a supported alternative.

## Shifted-intersection investigation

A focused regression exposed a separate coplanar clipping defect: when a line
crossed both arms of a concave U-shaped face, the clipper joined disconnected
inside intervals into one section through the opening. Its early return also
accepted such a crossing whenever both endpoints were inside. The clipper now
emits separate connected sections, preserving traversal direction and the
existing tolerance. The regression covers outside endpoints, inside endpoints,
and reversed traversal; it fails on the previous implementation.

This correction alone does **not** repair the shifted-holder intersection.
Replaying the same raw GFA candidate still produces 100 faces with 30 free
boundary edges. The source-partition trace also shows incomplete face splits:
the shifted bottom plane produces only one sub-face, while the partition's
bottom plane retains boundaries outside the shifted holder. Curved boundary
sections and the coplanar split remain the next investigation target. This is
diagnostic evidence, not permission to accept an open candidate.

## Curved sections and opposing-face extents

The next repair preserves a curved PaveBlock's actual carrier and stored trim
when computing its face-space curve. Previously this path always constructed
a straight 2D curve, even when the 3D edge was circular. Focused coverage checks
midpoints, descending trims and a trim crossing the periodic seam.

When clipping a section to its opposing face, a boundary-coincident interval
must be retained: it defines that face's extent even though it contributes no
new split on that face itself. Previously the helper discarded the interval,
and the caller then retained the overlong original section. The regression
exercises the complete section-building call with perpendicular faces of
different widths.

Clipping to the opposing face's true extent exposed two arrangement gaps
that the overlong sections had been papering over (the snapClip deepened-notch
fixture, `crates/io/tests/snapclip_deepened_notch_inmem.rs`, regressed to eight
unpaired edges). First, a curved face only pre-split its section curves at
points other faces had registered; the countersink's chord runs through the
old floor-bite corner, an existing vertex of the cone face, and that corner
was registered only because the overlong wall section happened to cross the
chord there. Curved faces now also pre-split their sections at their own
wire vertices. Second, the planar arrangement tested an endpoint T-junction
against a section arc's chord; the tightened wall-floor line ends on the
true conic a sagitta away from the chord, so the line dangled and the wall
below the old floor was never split off. Section arcs now measure the
endpoint against the true curve (boundary arcs keep the chord test).

Together these changes reduce the shifted-holder raw candidate from 30 free
boundary edges to 23, still with 100 faces. The bottom boundary is now paired;
a dedicated fixture regression checks every bottom edge and source immutability.
This is partial candidate coverage, not full acceptance. Open boundaries remain
around upper rounds, lettering and the rear blend. Strict exact-only acceptance
is unchanged; the full opening edit remains unsupported.

## Exact classification of rectangular torus patches

The upper round was present but classified outside the opposing holder. Its
old sample hugged the adjacent cylindrical boundary, and moving the sample
inward alone did not fix classification: partial torus faces in the opposing
solid were represented by flat polygons for ray parity.

The classifier now recognizes hole-free, four-circle torus patches whose
on-surface boundary forms a rectangle in continuously unwrapped parameter
space. These use exact torus ray intersections filtered by both angular
intervals. The builder uses the same recognized domain to choose a central
interior sample. Unsupported trims retain the existing path. No classifier
vote rule, strict acceptance rule, or operation tolerance is relaxed.

Focused tests cover rotated patches, multiple scales, reversed traversal,
both angular seams, and exclusion of rays outside the major-angle interval.
Holed and off-surface boundaries are excluded from recognition.

The hammer raw candidate now has 103 faces and 11 free boundary edges,
down from 23. Its upper round and lettering boundaries are paired, alongside
the previously repaired bottom. The fixture regression checks those regions
and source immutability. Remaining openings surround a sloped planar face and
the rear blend. This remains partial candidate coverage, not acceptance of
the complete 46 -> 50 mm operation.

## Scaled plane equations in coincident-face matching

The imported sloped plane stores rounded direction ratios, with normal length
squared 0.99999999947236395. Its translated copy has a unit normal. The old
same-domain comparison took their raw dot product against a unit-normal
threshold, so it missed the coincident plane and discarded both candidates.

Same-domain plane matching now normalizes both the normal and offset of each
`n.p = d` equation. This preserves the plane itself, handles either orientation,
and keeps the existing angular and linear thresholds. Regression coverage uses
the fixture's rounded direction ratios, positive and negative equation scales,
and genuinely separated and tilted planes. It fails on the previous code.

The raw hammer candidate now has 104 faces and seven free boundary edges,
down from 11. The sloped boundary is paired; the fixture checks its presence
and edge incidence alongside the bottom, upper round and lettering. The
remaining openings are at the rear blend. The full opening edit still fails
strict acceptance.

Normalizing the earlier coplanar section-generation phase was tested separately
but exposed a carried-circle endpoint mismatch. That change is excluded from
this repair; the stricter arc check remains intact.

## Next acceptance gate

Repair the remaining holder-to-holder boolean, then complete both symmetric
edits and validate the full result: 50 mm opening, unchanged 74 mm width and
58 mm height, both 5 mm bores, original rounds and raised lettering, closed
manifold geometry, and an exact STEP round trip. Repeat at the original and
nearby edited dimensions. Only then expose a replayable document parameter,
undo/redo and the existing AI command path in OpenZCAD. Remus remains the sole
production geometry kernel.

## Nonrectangular rear torus classification

The remaining original rear torus was incorrectly retained: the ray classifier
substituted a flat polygon for the shifted holder's nonrectangular torus patch.
Several points in the opening consequently classified as inside the holder.

Local, hole-free ring-torus patches spanning less than half a revolution in
each angle now use exact torus ray intersections filtered by an adaptively
sampled UV trim. Only trim membership is approximated. Sampling uses stored
edge parameter ranges, unwraps both periodic angles, and has bounded point and
recursion budgets. The uncertainty band includes observed projection error
from imported space curves; near-trim hits remain suspicious under the existing
ray-voting rules. Edge tolerances and strict geometry acceptance are unchanged.
Unsupported or non-closing trims retain the previous classifier path.

The hammer point-classification regression fails with the previous classifier.
Focused tests cover nonrectangular crossing membership, split arcs, reversed
winding, rotations, scale changes and both periodic seams. The raw shifted
intersection now has 103 faces and four free edges, all outlining the missing
cylindrical strip at x = [-9, -7], y = [39.5, 42.5], z = [9.5, 12.5] mm.
The fixture regression permits open boundaries only within that strip while
retaining the previous bottom, upper-round, lettering, slope and source
immutability checks. This is still a partial repair: the whole candidate
remains invalid and the complete opening edit is not enabled.


## Recovered tangency circles and closed shifted intersection

The final four free edges outlined a 2 mm cylindrical strip. A torus/cylinder
intersection marcher produced an approximate arc that stopped short of its
endpoint, even though the torus boundary already contained the exact circle.
The face-intersection stage now supplements the marched curves with that
stored circular arc when analytic carrier checks establish it as a torus
minor circle and a coaxial section of the partner cylinder. Other marched
curves remain available. Existing face-extent filters and splitting rules
still apply; no endpoint or validation tolerances are widened.

The shifted intersection now returns exact quality through the operations API:
104 faces, zero free edges, valid topology and watertight meshes before and
after STEP export/reimport. The fixture also checks volume preservation across
the round trip, reduction from its operand, all previously repaired boundaries,
and source immutability. The browser WASM smoke contract exercises the same
intersection and round-trip checks. Unit coverage includes rotated and scaled
carriers, both operand orders, reversed parameter ranges, periodic seams, and
near-miss carriers that must not emit a section.

This qualifies the shifted intersection, not the complete 46 -> 50 mm edit.
The subsequent fuse was qualified by the next repair below. OpenZCAD's
production kernel pin and AI parameterization features remain unchanged.


## Left-side fuse: spherical patches and subdivided straight boundaries

The fuse previously created two full circular sections on spherical patches
that lie entirely on the other side of their cutting plane. Convex patches
bounded by minor great-circle arcs now provide inward half-spaces. Recognition
checks analytic arc extrema against every half-space. A circle is discarded
only when its maximum signed distance proves that its entire carrier lies
outside a patch. Unsupported boundaries retain the existing path. The tighter
patch is deliberately not fed into the fixed-sample extent filter, which could
miss a narrow but legitimate section.

Two remaining wire-intersection reports came from a straight boundary split
less than the check tolerance from its corner. The checker now groups monotone,
collinear line subdivisions for adjacency, using shared topology and a
floating-point roundoff bound. It does not change the geometry or check
tolerance. Crossings, genuine bends and backtracking retain their checks.

The left reassembly has 177 faces and passes strict validation, native and
browser WASM mesh closure, both mounting-bore checks, source immutability,
partition-volume additivity and STEP round-trip validation/volume checks.
An existing Euler-characteristic warning remains; it is not suppressed.
The full reconstruction is still blocked at the following right-side cut.
