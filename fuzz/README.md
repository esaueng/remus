# Fuzzing

`fuzz/` has bounded libFuzzer targets for public readers and kernel operations.
Reader inputs use strict `ImportLimits`; engine targets use small structured
generators and independent geometry or topology oracles, so a plausible but
wrong result is a finding as well as a panic.

`nurbs_surface` constructs 2–4 control-point rational patches, tests typed
rejection of corrupted construction data, evaluates points and derivatives,
and intersects every valid patch with a horizontal NURBS plane. Returned SSI
points, parameters, and fitted curves must satisfy the plane equation and
re-evaluate on both input surfaces. Its corpus includes a clustered marching
section that previously made a cubic refit leave the known plane.

`topology_mutation` builds a bounded box (census and `dx * dy * dz` known by
construction) and drives byte-selected topology mutations over it:
authoritative face-loop identity (`build_face_loops` is read-only on a
derived face) and retirement through the sanctioned wire replacement,
validated rollback of a deliberately broken wire, rollback of staged
allocations and of in-transaction wire replacements/deletions, checkpoint
restore, unreferenced-solid deletion, and referenced-deletion refusal. The
oracles are the mutation contracts: comprehensive validation and the
closed-manifold census hold after every step, rollback reproduces the exact
live state while checkpoint restore keeps window retirements tombstoned and
promotes the affected face onto fresh handles rather than dangling, retired
handles fail typed lookups forever and are never reissued, a refused
deletion leaves no partial mutation, and an accepted one retires exactly the
unshared tree — a guard box and an unrelated compound must survive. Its
corpus includes a checkpoint re-derivation seed that previously left the
loop-derivation map referencing retired loops, and the two seeds from the
coedge-authority flip (PR #179): a re-derivation whose handles must now be
preserved, and a guard-box sweep whose derivation census must count the
loops `add_face` installs.

`modifier_ops` builds a bored or bossed primitive and applies one fillet,
chamfer, shell or draft, checking hole preservation, closed-manifold census,
watertight tessellation, scale invariance and integrator agreement. Its corpus
includes a draft seed whose 1° outward taper of a narrow facet slid the facet's
corners past each other; the folded face passed validation and the volume sign
check and only showed as four wrongly-wound half-edges in the fine
tessellation, and is now refused by name. It also carries a fillet seed — a
unit box fused with a large disjoint torus, one box edge filleted at r and r/2
— that once tripped the option-honoured invariant: the two fillets differed by
4e-4 as they should, but the check scaled that against the torus's ~800 of
unrelated volume. The invariant now judges the volume each setting changed.
A second fillet seed — a torus whose tube enters a frustum through its base
cap and leaves through the cone wall, fused, then a 0.05 fillet on the base
rim — reported an open mesh on the fillet that was really the fuse's: the two
closed tube-wrapping pierce sections were filed as inner wires of the full
periodic torus face, which the mesher skinned over and both volume routes
misread by less than the agreement band. The band tracer now accepts closed
sections (B45). Its third seed — a box fused with a disjoint sphere rotated 45°
about Y, then a draft — failed the base body's closed-form volume before the
modifier ran: the rotated sphere's two hemispheres swept the same pole after a
rigid move rebuilt the surface on world axes, so the fused mesh was open and
read 22 % low while the exact route was right (B41).

`arena_roundtrip` builds bounded boxes and cylinders (census and closed-form
volumes known by construction) with duplicate roots, shared-shell aliases,
repeated/aliased compound members, precision-hostile tolerances, and public
attributes, then round-trips the document through the native arena format.
Restored solids must validate, stay closed-manifold, and measure their
closed-form volumes per root/member position; tolerances, trims, and
attributes survive bit-exactly; and serialize → deserialize → serialize is
byte-identical. Deliberately corrupted root/member/wire/version references
must be refused with a typed error, leaving a pre-populated destination
topology untouched and leaking no staged allocations. Its corpus includes a
seed covering attributes on a deliberately uncaptured member (correctly
absent from the document). The byte-identity oracle caught serde_json
losing the last bit of arbitrary f64 tolerances without the
`float_roundtrip` feature, which is now enabled workspace-wide.

`tessellation` builds a bounded unplaced primitive leaf from `shapegen`
(leaves only — a rigid placement can rotate a sphere's seam/pole frame out
of the tessellator's weld alignment, and a misclassified boolean hands the
mesher a wrong-but-closed solid; both failure classes are owned elsewhere)
and tessellates it at two deflections a factor of four apart. The oracle is
independent of the mesher under test: the hand-derived closed-form volume
that `shapegen::build_prim_measured` returns alongside each primitive. The
signed mesh volume at both deflections must agree with the closed form
within `VOL_SLACK`, the finer deflection must not diverge from it, and the
index buffer must be structurally sound (non-empty, valid indices).
**A typed refusal is a pass.** Its corpus covers all five primitive kinds.

`curve_intersection` builds a bounded NURBS curve and a bounded NURBS
surface from the fuzzer's bytes (small degree, few control points,
coordinates on a coarse lattice so near-degeneracy is common) and runs
`intersect_curve_surface`. The oracle is independent of the solver under
test: every reported hit must satisfy BOTH geometries —
`curve.evaluate(hit.t)` and `surface.evaluate(hit.uv)` each within 1e-4 of
`hit.point`. A hit on only one geometry (or neither), or a non-finite hit,
is a finding; constructor or solver `Err` is a pass.

`offset` builds a bounded primitive (never a boolean result — compound
operands entangle offset defects with boolean misclassification, owned by
`boolean_tree`), places it rigidly, and offsets it by a small signed
distance. The oracle is independent of the offset machinery under test:
the result must be a closed 2-manifold whose measured volume moves in the
right direction (outward grows, inward shrinks, within slack at ~zero) and
stays under the one-sided convex ceiling `2·(V + A·|d|)`, which admits exact
second-order edge/corner growth while catching wrong-way offsets and gross
volume inflation. Typed refusal (offset, construction, or measurement) is
a pass.

`gcs` builds a small sketch from the fuzzer's bytes — 2–5 points on a
coarse lattice, lines over them, and three constraints drawn from the six
geometrically re-checkable kinds (coincident, distance, horizontal,
vertical, fix-X, fix-Y; the angular kinds are out of scope and never
constructed) — then runs the solver. The oracle is independent of the
solver under test: when the solver reports `converged`, every constraint
is re-evaluated geometrically from the solved point positions with
hand-written residual functions, within 1e-6. A converged-but-violated
system, or non-finite solved positions, is a finding; builder or solver
`Err` (including non-convergence) is a pass.

Run one target locally with nightly Rust and `cargo-fuzz`:

```bash
cargo +nightly fuzz run nurbs_surface -- -max_total_time=60 -rss_limit_mb=2048
```

PR CI compiles every target. The scheduled `Fuzz Smoke` workflow runs the
public model-reader, boolean-tree, modifier, NURBS-surface,
topology-mutation, arena-roundtrip, tessellation, curve-intersection,
offset, and GCS campaigns for two minutes each and
retains crash artifacts. `arena_reader` and `wasm_batch` currently compile
in PR CI but are not scheduled. All four B19 engine slices (tessellation,
curve-intersection, offset, GCS) landed with independent oracles,
committed geometric seeds, and weekly scheduling in PR #441.
