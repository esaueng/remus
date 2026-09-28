# O2.3b/c/d planar-slice checkpoint — `codex/o23c-arrangement-prod-integration`

Base: `origin/main` @ `bf46fdc0` (2026-09-27 fetch). Worktree:
`/home/peter/code/Remus/.worktrees/o23c-arrangement-prod`. No open PR owns
splitter files (queue 2026-09-27: only PERF-S06, B76-shell, PERF-R02,
PERF-I05-3MF open). Prior slice branch `codex/o23b-uv-arrangement`
(`08b8c79c`) is unmerged local work, not an open PR; its design note is the
spec.

Ownership respected: only `crates/algo/src/builder/face_splitter/**` +
directly necessary builder integration, algo/operations regression fixtures,
arrangement docs. No edits to topology, math intersection, offset,
distance/bounds, STEP, transforms, healing, sketch, render production code.

## M1 — contract (established 2026-09-28)

### Engine inventory (`face_splitter/arrangement{,.rs,/geometry,/regions,/periodic}`)

- Input: caller-supplied `CurveUse`s in ONE lifted chart. `pcurve`
  (`Line`|`Circle` only) + `range` (native 2D interval, circles in
  `[0,2π]`) + `curve_3d` (`Line`|`Circle` only) + `source_range` (affine
  pcurve→source map; lines interpolate use endpoints, circles evaluate
  native source params) + `endpoints_3d` + endpoint-event certs +
  `boundary_loop` keys (sections: none) + `CurveSource`
  (use_id/boundary-coedge/section/source_edge_idx/pave_block_id + exact
  subspan). `ParamDomain::{Plane, CylinderStrip{seam_uses,radius}}`.
- Output: canonical exact carriers (`sources`), half-edges w/ twin+next,
  source+source_range subspans, directed 3D endpoints; cycles w/ analytic
  Green area + left seeds; regions w/ outer+holes, interior, area,
  boundary-parity `material`; cylinder-strip quotient w/ seam
  identifications, windings, Euler.
- Refusals (typed, never partial): NonFiniteInput, InvalidBoundary,
  UnsupportedCurve, IntersectionRefinementFailed, AmbiguousOverlap,
  AmbiguousContact, NonManifoldEmbedding, OpenRegion,
  WorkBudgetExceeded, Cancelled, UnsupportedDomain.
- Deliberate deferrals: auto lift/clip, oblique cylinder traces,
  seam-crossing hole adaptation, curved multiway junctions, overlap
  emission, tangent limiting order, noncanonical circle branches, poles,
  double-periodic.

### Production inventory (`face_splitter/mod.rs` 10k lines, `special_cases.rs` 6k)

Migration floor per design (first two first): `try_split_crossing_plane_face`
(X/T/4-ray; Line-only direct construction, synthesized `EdgeCurve::Line`
emits, NO provenance), `chain_boundary_edges` (greedy reorder, no isolated
pin), `try_split_disk_by_chords` (disc+chord gate, quantized vertex keys,
own tracer). Adjacent keep-as-fallback: `split_plane_face_by_arrangement` /
`arrangement_regions_from_{combined,inputs}` (chord-quantized identity —
deliberately NOT copied), `split_cylinder_band_by_arrangement`,
`split_periodic_face_by_winding_chain`, `build_wire_loops_dcel` (tracer only).

### First supported domain (planar tranche)

Plane faces (`FaceSurface::Plane`) where EVERY boundary edge and EVERY
section use is exactly representable as `Curve2D::{Line,Circle}` +
`EdgeCurve::{Line,Circle}` with the circle lying in the face plane
(frame round-trip ≤ weld band, incl. arc midpoint). Includes: outer
boundary + inner (hole) wires, disconnected material regions/islands,
major arcs, chord/arc lenses, X/T/star + multiway crossings, thin
retained regions, input permutations, reversed uses, rigid placement,
scale. Sections must be Line or in-plane Circle; boundary Circles may be
full rims.

Recorded UNSUPPORTED (typed out-of-domain → established paths):
tangencies (`AmbiguousContact`), coincident overlaps
(`AmbiguousOverlap`), any NURBS/Ellipse/Hyperbola/Parabola curve,
off-plane/straddle arcs, NURBS-fit circle pcurves as input (adapter
rebuilds authoritative Circle2D from the 3D carrier — never consumes the
sampled fit), missing trims, non-finite data, exhausted budget,
cancellation, poles/double-periodic/cylinder-strip auto-lift (strip only
via written contract later).

### Critical M2 facts (verified in source)

- Plane curved edges carry NURBS-FIT pcurves from
  `compute_pcurve_on_surface_in_domain` (`fit_nurbs2d_through_points`);
  adapter must NOT feed them to the engine — rebuild `Curve2D::Circle`
  as `center=frame.project(c.center)`, `r=c.radius` (frame is an
  isometry on the plane).
- `Circle2D` param = raw UV angle; `Circle3D` param = angle in its own
  (u_axis,v_axis) frame. Correspondence is angle offset ± orientation
  sign — derive per use from projected axes + endpoint anchoring, never
  assume identity. `Line2D` normalizes dir (arc-length param).
- `OrientedPCurveEdge`: `trim` = exact native interval; `forward` flips
  traversal; `Line` native domain is `(0,1)`; `traversal_domain()` preserves
  signed branch. `SectionEdge`: rank selects pcurve_a/b + uv_a/uv_b
  (fallback: frame.project(start/end)); `trim` carried authority.
- Emission contract: `SplitSubFace{surface, outer_wire, inner_wires,
  reversed, parent, rank, precomputed_interior}`; edge sharing via
  `source_edge_idx` (same-face) + `pave_block_id` (cross-face); do NOT
  rewrite the universal duplicate-edge merge key.

## M2 — adapter (LANDED 2026-09-28: `77fa7edd`)

`face_splitter/arrangement_prod.rs`: `collect_planar_uses` + `run_planar_arrangement`.
Threading landed first (`53798d44`). Corrections made during implementation:
- engine consumes traversal-order intervals and normalizes direction itself;
  adapter never rebases, only pairs ends consistently;
- closed-circle trims prove span only: source anchored at traversal start's
  own angle (canonical CCW full span), verified against the carrier;
- prejoin splits need cumulative target-offset accounting;
- spur detection is a no-pendant degree rule (reachability is blind to
  spurs: a spur reaches the boundary through itself);
- full-circle engine check requires source ends to evaluate to endpoint
  positions (hence traversal-anchored sources, not trim branches).
14 adapter fixtures green; full algo suite green; clippy/fmt clean.

New `face_splitter/arrangement_prod.rs`: `qualify_planar_face()` gate →
`build_planar_uses()` (boundary loops w/ coedge identity + endpoint certs
+ loop keys; sections w/ rank pcurves + pave ids; authoritative Circle2D
rebuild + angle correspondence) → `build_arrangement` →
`emit_subfaces()` (regions→wires w/ exact subspans, holes attached,
interior seeds, provenance). Out-of-domain → `Ok(None)` (established
paths); claimed-input internal failure → `Err` (never silent unsplit).

## M3 — emission (LANDED 2026-09-28: `ef091e42`)

`emit_planar_subfaces` + `try_split_plane_face_by_provenance_arrangement`
(then unwired): regions→subfaces with holes, interior seeds, source
provenance, exact subspans; atomic pure construction (no topology
allocated on refusal — pinned). Core 1-ulp dust-twin adoption +
`recomputed_crossing_near_certified_endpoint_adopts_it`. 19 fixtures green.

## M4 — dispatch (LANDED 2026-09-28: `dcd2d8df`, first-resort)

Hole-free plane faces with 2+ sections route through the core when
`result.len() > loops.len()` or the greedy trace is demonstrably broken.
Budget policy (`arrangement_context`: input-proportional floor), TAU
canonicalization, `AmbiguousContact`/`AmbiguousOverlap` → `Ok(None)`.

## tangent-boss root causes (fixed 2026-09-28: `640a810b` + M5 follow-ups)

Bisect isolated M4's dispatch (M3 passes, M4 fails
`tangent_boss_batch_contract`: solid 3 vs 5): `try_` errored with
`IntersectionRefinementFailed` on the tangent-graze face (boss arc poking
0.001 past the wall, crossings 0.28 apart — twin events within roundoff,
no endpoint certificate to adopt), aborting GFA early and shifting handle
numbering. The trigger is execution, not adoption: a pure discarded
computation cannot shift numbering; the `Err` → `?` early-abort can.
Fix 1: `IntersectionRefinementFailed` is geometric (grazing/tangent has no
transverse refinement), so it declines like the other geometric refusals;
plus a pairwise interior tangent/overlap pre-check (`uses_have_degenerate_
contact`) that declines exact degeneracies before running the core.
Endpoint touches never decline (certified joints). Other refusals still
propagate; declining never produces wrong geometry (established path is
the shipped baseline) and the differential suite pins engagement on
transverse input.
Fix 2 (found restoring M5): M5's one-sided shared-end adoption lets the
graze refine, but emission then refuses `mixed-use run` (use change at a
merged vertex). Same fail-safe applies — emission's exact-provenance
refusal declines via the `MIXED_USE_RUN` constant (pinned end to end by
the tangent-boss contract). Cancellation still propagates.
Fix 3 (latent, same area): section breaks stage locally and commit to the
split registry only on adoption — recording a discarded arrangement would
hand curved neighbors splits the plane side never took.

## M5 — qualification (restored 2026-09-28, in validation)

M5 WIP (differentials vs crossing/disk helpers, permutation/reversal/
rigid/scale/thin-strip/star invariance, one-sided shared-end adoption,
branch-cert merging, `record_section_breaks`, last-resort dispatch
ordering) restored onto the fixed dispatch; all `DBG-*`/`O23C_*` scratch
probes and the 537-line `dbg_*` growing-holder scratch suite removed
(the real `growing_holder_volume_change_is_the_floor_section` passes).
Last-resort ordering kept: established plane paths run first (their
mesh-coordinated outputs are what the tessellator is calibrated on);
the exact arrangement is reserved for faces no established splitter can
partition. Validation: algo 455, arrangement_prod 35 (incl. 6 classifier
tests), L-cut regress 4, WASM lib 615/615, operations full 4064/4064,
io full 662/662, wasm-io 8/8, census 50 exact-analytic, clippy/boundaries
clean. Refusal pins hold post-change (tangent_boss, plain_batch_booleans,
compound_cut). render compute_mesh_lod SIGSEGVs in sandbox GPU teardown
(no GFA in its path — make_cylinder + wgpu only; environmental).

Nub follow-up (fixed `fdcd960e`): the dovetail back-edge nub
(tangency-meridian, dust-scale junction cluster) aborted exact cut via
unmapped `NonManifoldEmbedding` → same fail-safe mapping (geometric,
declines; witness documented in the mapping comment).

Then roadmap update + one review-ready PR (no merge/deploy).
