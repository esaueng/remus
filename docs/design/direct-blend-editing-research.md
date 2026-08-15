# Direct editing of blends and recognized features on dumb solids

Status: validated design + implementation plan. Every load-bearing tier-1
claim has been tested against the live kernel; the evidence is in §2 and in
`crates/operations/tests/research_blend_edit_probe.rs` (E1–E19), which runs
green. The tracked STEP corpus was also censused and adjacency-probed. Earlier
unvalidated drafts are superseded.

Scope: Shapr3D-style direct editing on STEP-imported solids in OpenZCAD —
select a fillet face, see its radius, drag or type a new value; same for
holes, counterbores, pockets, bosses, chamfers. Fail-closed exactness:
operate only when provable from current topology and exact geometry, refuse
with a typed error otherwise, never silently substitute approximation.

## 1. Ground truth: what the kernel already has

Verified by reading and (§2) executing the code.

| Capability | Location | Notes for this design |
|---|---|---|
| Analytic fillet fast paths | `crates/blend/src/analytic.rs:108` `try_analytic_fillet` | Coverage: plane×plane→cylinder; plane×{cylinder,cone,sphere}→torus; sphere×{cylinder,cone}→torus; sphere×sphere→torus; parallel cyl×cyl→cylinder; coaxial cone×cone→torus. The construction API remains `pub(crate)`; `crates/blend/src/query.rs` exposes only validated carrier re-derivation. It handles inward orientation, rejects malformed/coincident supports, and proves line/circle spine containment before dispatch. The analytic helper still reads real topology: `plane_is_bounded_disc` (`analytic.rs:1066`) inspects the plane face's outer wire, and the reversed flag of the cylinder face decides material side — so recognition/resize must run against real body faces, not detached geometry. |
| Corner solver | `crates/blend/src/corner.rs` | `ThreePlanar` exact corner ball (`tangent_corner_ball`, line 265), `MultiEdge` spherical-triangle patches. E2 measured: trihedral orthogonal fillet corner is `FaceSurface::Sphere` with exact radius and `Circle` edges. |
| G1 chain expansion | `crates/blend/src/g1_chain.rs:71`,`:224` | Reused by recognition for chain ordering. |
| Defeature | `crates/operations/src/defeature.rs:81` | Cap + extend heals. This work adds the exact curved-wound collapse rule: a non-line wound edge may disappear only when both relocated endpoints provably coincide; all surviving curves still refuse. E6 restores a plane-plane fillet exactly. |
| Feature recognition | `crates/operations/src/feature_recognition.rs`, `query.rs` | FAG with `SurfaceClass`, sampled normal angle and corrected convex/concave/G1 edge classification; holes with coaxial grouping incl. counterbores; chamfers; pockets; patterns. `FilletLike` (line 411) is area-only — replaced by §3. |
| Cylinder resize | `crates/operations/src/push_pull.rs:133` | Normalize-to-canonical-frame + expected-volume gate. Template for edit-op structure. |
| Topology surgery primitives | `crates/topology/src/topology.rs:200-215` (`vertex_mut`,`edge_mut`,`face_mut`), `vertex.rs:42` `set_point`, `edge.rs:268` `set_curve`, `face.rs:294` `set_surface`, `face.rs:272,283` wire mutation | E7/E8 proved these suffice for closed-ring resize. |
| Analytic intersections | `crates/math/src/analytic_intersection.rs` | Exact: plane×{cylinder,sphere,cone,torus}, cone×cone, cone×cylinder, sphere×cylinder. Marching `intersect_analytic_analytic` = not exact → refusal boundary for tier 2. |
| NURBS derivatives | `crates/math/src/nurbs/surface.rs:313` `derivatives(u,v,d)` | First/second partials exist; principal curvatures (first/second fundamental forms) are a small new addition needed by the spline-band classifier (§3.2 step 4). |
| Transactional rollback | `crates/operations/src/blend_ops.rs:728` | Every mutating op below uses `transactional`. |
| Validation | `crates/operations/src/validate.rs:440` | Euler, closure, geometry checks. E7 measured: **does not catch an off-curve vertex** (a vertex left at radius 13 on a radius-15 circle passed) — edit ops must add their own strict geometry gate (§3.3). |
| Evolution | `crates/operations/src/evolution.rs`, `crates/wasm/src/types.rs:107` `FaceEvolutionPayloadV1` | Provenance vehicle (§7). |

Two implementation facts found after the first draft materially change the
work breakdown:

- Ordinary constant `fillet_v2` uses the independent planar rolling-ball
  engine for all-planar selections, otherwise delegates through
  `fillet_builder.rs` to **`fillet_builder_legacy.rs`**. The large modern
  `fillet_builder.rs` assembly body is primarily the explicit radius-law path.
  Production closed-rim extraction/rebuild therefore lives at
  `fillet_builder_legacy.rs:951-1458`; phase 0 refactors that path first.
- The original feature FAG angle classifier was internally inconsistent:
  `acos(n1·n2)` returns only `[0, π]`, yet `Convex` required `> π`, and
  `Tangent` was classified near π. The shared query classifier now samples
  exact edge curves, treats aligned outward normals as G1, and decides
  convexity by counting material quadrants around the two inward halfspaces.

Terminology: **band** = blend face; **spring edge** = tangent contact edge
band↔support; **cross edge** = edge closing a band at a chain end, corner, or
seam; **support** = face blended between; **corner patch** = vertex blend face.

## 2. Empirical findings (E1–E19, all reproduced by the probe)

**E1 — plane×plane fillet structure (PASS).** A single-edge r=3 fillet on a
box produces exactly one `FaceSurface::Cylinder` band, radius exact to 1e-12.
Band boundary: two `Line` spring edges + two `Circle-arc` cross edges on the
adjacent top/bottom faces.

**E2 — trihedral corner structure (PASS).** Three-edge r=3 fillet at a box
corner: three cylinder bands + one `FaceSurface::Sphere` corner patch, radius
exact, bounded by three `Circle` edges. Corner patches are analytic, not
NURBS — tier-1 corner rebuild is a closed-form sphere replacement, not a
walking-engine rerun.

**E3 — hole-rim (plane×cylinder) fillet structure (PASS).** The band is an
exact `FaceSurface::Torus` (R=13, r=3): minor = fillet radius, major =
r_bore + r, center on the bore axis at z = plate_top − r. The band's wire has
**three** edges: two spring circles (plate: radius r_bore + r at z_top; wall:
radius r_bore at z_top − r) **plus one seam cross edge** (full tube circle of
radius r centered on the center circle) — any rebuild must classify and
rebuild all three. The bore wall is seam-split (a `Line` seam edge).

**E4 — STEP round-trip (PASS).** The exact torus (R=13, r=3) survives
`write_step`/`read_step` to 1e-9. Recognition works identically on imported
and kernel-built bodies.

**E5 — inverse-analytic recognition check (PASS).** Given only the band and
its two supports, the public carrier query (`try_analytic_fillet_surface` at
the measured radius with a topology-independent `GeometricSpine::Circle`)
**reproduces the band surface exactly** (center/radii to 1e-9 and symmetry/
reference axes to 1e-12). The query validates support and spine frames, then
materializes the private closed-edge representation only in a topology
snapshot. Recognition's decisive test is therefore implementable as
"re-derive and compare", not fitting.

**E6 — plane-plane unfillet (PASS).** The extend heal now admits one exact
curved-wound case: circular cross arcs whose relocated endpoints collapse to
the same recovered sharp corner disappear instead of being chorded. Removing
the cylinder band restores the original six-face box and exact volume;
unrelated or surviving curved edges remain typed refusals.

**E7 — in-place ring resize surgery (PASS).** Replaced band torus r=3→5
(`face_mut.set_surface`), retargeted both spring circles (`edge_mut.set_curve`),
rebuilt the seam cross edge, moved two seam vertices (`vertex_mut.set_point`).
`validate_solid` passes; measured volume 22711.982 = closed-form Pappus
expectation 22711.982 exactly.

**E8 — resize ≡ fresh fillet (PASS).** The E7-resized body is
surface-for-surface identical to a fresh r=5 `fillet_v2` from the same sharp
source; volumes identical. This is the regression oracle for phases 1–2.

**E9 — post-on-plate base fillet (FIXED).** The analytic plane×cylinder
stripe was already correct; the legacy rim assembler applied a convex-only
torus-band orientation heuristic and emitted the concave band inside-out. The
fail-closed volume gate correctly refused that shell. `ClosedRimInfo` now
carries the analytic convexity fact, the torus orientation flips for concave
rims, and the modern builder mirrors the rule. Regression:
`post_base_fillet_adds_exact_material_and_reverses_band` (r=1,2,5) checks the
reversed band, exact torus frame, and closed-form Pappus material addition.

**E10 — open plane×plane resize (PASS).** An r=3 open cylinder band on a box
was changed to r=5 by retargeting the cylinder, two circular cross arcs, and
four shared spring endpoints. No wire or face topology changed. The result's
surface multiset and measured volume are identical to an independently built
fresh r=5 fillet.
The analytic volume identity differs from `solid_volume` by 0.066 on a
16,000-unit body due to numerical integration, so production volume gates use
the repo's scale/deflection tolerance, never exact f64 equality.

**E11 — trihedral network resize (PASS).** Three r=3 cylinder bands plus their
exact r=3 sphere corner were changed to r=5 by retargeting 3 cylinders, 1
sphere, 6 circle arcs and 9 shared vertices. Topology remains fixed; surface
multiset and volume match an independently constructed fresh r=5 network. A
same-topology fresh-oracle attempt also exposed that successful planar fillet
construction may rewrite entities shared by its input solid: direct editing
must always deep-copy before mutation, even on success.

**E12 — elementary recovery (PASS).** An exact rational NURBS cylinder made by
`CylindricalSurface::to_nurbs` refits through `convert_to_elementary` to the
same axis/radius at 1e-7. Exact NURBS analytics should therefore be normalized
on an isolated copy before recognition; curvature sampling is only a
descriptive fallback, not a path to editability.

**E13 — copy/pcurve gap (PASS, documents current behavior).**
`copy_solid_with_face_map` deep-copies faces/wires/edges/vertices but does not
copy registry pcurves. A source `(edge,face)` pcurve exists while its copied
pair has none. `copy_solid_with_entity_maps` is therefore not optional
polish: it must remap pcurves before direct edits can preserve exact STEP/IGES
trim data.

**E14 — pcurve representation baseline (PASS).** Kernel-built open analytic
fillets, kernel-built torus ring fillets, and the imported OpenZCAD fillet STEP
all contain zero registry pcurves. Tier-1 MVP may therefore edit only a
pcurve-free certified region (preserving exact 3D curves/surfaces and refusing
any existing affected pcurve). Full exact pcurve regeneration is a parallel
kernel-foundation track, not a hidden prerequisite for matching today's
representation.

**E15 — imported tangent convention (PASS).** The corrected three-sample G1
test identifies exactly four r=3 cylinder bands in the OpenZCAD fillet plate
and zero candidates in the r=5 bored-plate negative. The proof also exposed
three required special cases: planes use `effective_plane_normal` because
`FaceSurface::Plane` has no UV projection; smooth outward normals align near
zero angle; and an edge used twice by one periodic face is a self-seam, not a
two-support adjacency.

**E16 — counterbore stage radius (PASS).** A large stage r=3→4 edit by direct
carrier/ring/seam-vertex surgery matches an independently built r=4
counterbore and the exact staged-volume formula. The existing generic
`resize_cylindrical_face` was first tried and correctly refused because its
boolean result contained no analytic radius-4 cylinder spanning the stage.
Staged holes therefore need a feature-aware direct editor, not chained single
face resize booleans.

**E17 — counterbore depth (PASS).** Moving the shoulder from z=7 to z=5,
retargeting its two circular rings and two seam vertices, changes depth 3→5.
The edited body matches the fresh feature and closed-form volume exactly.

**E18 — history-free counterbore certificate (PASS).** From the bare topology,
the canonical axial walk recovers ordered stages `(z=0..7,r=1.5)` and
`(z=7..10,r=3)`, each with two complete rings, one annular shoulder at z=7,
and openings at z=0 and z=10. The certificate data required by E16/E17 is
therefore recoverable without feature history.

**E19 — plane-cylinder ring unfillet (PASS).** A hole-rim torus band was
collapsed by recovering the exact plane∩cylinder circle, sharing it between
the support plane and wall, moving the wall seam vertex back to the sharp
plane, and deleting the torus face. The healed body validates and is
volume-identical to the original drilled plate. Both tier-2 L1 primitive
geometries are now proven; network ordering/generalization remains engineering.

**STEP corpus census.** All 20 tracked STEP fixtures import. Representative
recognition fixtures are:

- `openzcad_e_analytic_fillet_plate.step`: 4 exact r=3 cylinder bands, each
  with two tangent planar support contacts and two sharp cap contacts.
- `openzcad_a_export_bored_plate.step`: one r=5 bore cylinder with zero tangent
  support contacts — the required negative control.
- `wallcut_tool_0.step`: 32 cylindrical patches representing 16 logical r=1.11
  rounds; `lipfuse_3x3_body.step`: 16 patches representing 8 logical rounds.
  Recognition must group exact co-surface patches before applying band rules.
- `scoop_cavity_0/1.step`: NURBS walls convert to planes; their r=2.55 cylinder
  bands then have elementary supports.
- The untracked Shapr3D walking-stick fixture contains three exact r=4 torus
  bands (major radii 16,16,20.5) tangent to cylinders/cones, but its existing
  tessellation repro still reports 701 boundary edges (694 on torus groups).
  It is useful for research queries but must not enter the tracked corpus until
  that unrelated defect is resolved or the user explicitly adopts the file.

## 3. (b) Algorithm specification — blend recognition + analytic resize (tier 1)

### 3.1 New module layout

- `crates/blend/src/query.rs` (new, public) — curated carrier-surface
  re-derivation through `try_analytic_fillet_surface`. It accepts support
  `FaceId`s plus a topology-independent `GeometricSpine`, then handles inward
  orientation and the private `Spine` / `Stripe` construction types internally.
  Phase 0 extends this narrow carrier API with contact DTOs.
- `crates/operations/src/blend_recognition.rs` — pure query, no mutation.
- `crates/operations/src/blend_edit.rs` — `resize_blend` (tier 1) and
  `remove_blend` (tier 2); both `transactional`, both emit `EvolutionMap`
  with `EvolutionOrigin::Construction`.
- `feature_recognition.rs` — `Feature::Blend` variant backed by the new
  module; `FilletLike` deprecated.
- `crates/operations/src/transaction.rs` — shared `pub(crate)` topology
  transaction helper; `blend_ops.rs` currently owns it privately.
- `copy_solid_with_entity_maps` — exact face/wire/edge/vertex mappings for
  mutation and provenance. Re-recognition on the copy is an acceptable
  phase-1 shortcut; entity maps are the production endpoint. It also copies
  every pcurve by mapping both key handles and cloning the `PCurve`; E13 pins
  the current omission.

### 3.2 Blend recognition

```
pub struct BlendComponent {
    pub bands: Vec<BlendBand>,
    pub corners: Vec<CornerPatch>,       // sphere or NURBS patch, >= 3 bands incident
    pub chains: Vec<Vec<usize>>,         // band indices, traversal order
    pub network_class: NetworkClass,     // Isolated | Chain | Network(blend-on-blend)
}

pub struct BlendBand {
    pub faces: Vec<FaceId>,              // one logical carrier may be STEP-split
    pub radius: BlendRadius,             // Constant(f64) | Variable { r_min, r_max }
    pub surface_kind: BandKind,          // Torus | Cylinder | Sphere | Cone | Nurbs
    pub supports: (SurfaceGroupId, SurfaceGroupId),
    pub spring_runs: (Vec<EdgeId>, Vec<EdgeId>),
    pub cross_edges: Vec<EdgeId>,        // includes seam edges (E3)
    pub convexity: Convexity,            // Fillet (concave) | Round (convex)
    pub analytic_verdict: AnalyticVerdict,
}

pub enum AnalyticVerdict {
    ConfirmedRollingBall,                // resize_blend-eligible
    RecognizedNotAnalytic { reason: String },  // walking-engine band on analytic supports
    NotAnalytic { reason: String },      // NURBS band or NURBS support
}
```

Pipeline (recognition never refuses — it classifies; the verdict gates
editability):

1. **Normalize an isolated copy.** Run `convert_to_elementary(copy, tol)`
   before recognition. E12 proves exact rational analytics re-enter the
   elementary path. Never mutate the caller's imported body merely to answer a
   query; map results back to original FaceIds. Preserve carrier reference
   directions where recovered because seams depend on them.
2. **Build a corrected FAG.** Reuse `AdjacencyIndex`, not the current
   `compute_dihedral_angle` result. For each shared edge:
   - require two **distinct** incident faces; two uses by the same periodic
     face are a self-seam, not adjacency (E15). A later occurrence-aware index
     should record seam uses explicitly;
   - sample the exact edge curve at 25%, 50%, 75% of its parameter domain
     (closed curves use three distinct parameters, not the coincident vertex);
   - evaluate effective outward normals; planes use
     `Face::effective_plane_normal`, other surfaces use projection (E15);
   - classify G1 iff `1 - n1·n2 <= angular_gate` at every valid sample;
   - otherwise classify convexity with `query::edge_concavity`: probe the
     four normal-halfspace quadrants around the edge; one material quadrant
     means convex (an intersection-like corner), three means concave (a
     union-like/re-entrant corner), and any boundary, oversized probe, or
     other count is `Unknown`, never a guess;
   Store `normal_angle = atan2(|n1×n2|, n1·n2)` in `[0,π]`, not a fictitious
   0..2π dihedral. Nodes retain full `FaceSurface` parameters.
3. **Group exact co-surface patches first.** Union connected faces when
   `brepkit_heal::analysis::surface::surfaces_equivalent` holds. One logical
   band/support may span several STEP faces or seam patches (`wallcut`,
   `lipfuse`). Keep member FaceIds and aggregate boundary edge runs.
4. **Candidate bands.** `Torus` (minor = r); `Cylinder` with two tangent
   **support surface groups** across lateral boundary runs (bore walls fail
   this — cap contacts are sharp); `Sphere` incident to >=3 equal-radius bands
   → corner patch; `Cone` only when inverse analytic derivation supports it;
   remaining NURBS → step 6. A candidate may have many spring segments but
   exactly two support carrier groups. All other runs are cross/seam/corner
   edges (E3's seam included).
5. **Support binding + re-derive-and-compare (the E5 test).** From
   `(support1, support2)` and measured r, run `try_analytic_fillet_surface`
   with a `GeometricSpine` recovered in closed form:
   plane×plane → `plane∩plane` line; plane×cylinder/cone/sphere & sphere×x →
   the perpendicular configuration's circle/line (E5: circle at the support
   intersection); parallel cyl×cyl → centers line; coaxial cone×cone → shared
   axis. `Some(carrier)` whose surface
   parameters match the observed band within `Tolerance` →
   `ConfirmedRollingBall`. `None`, or mismatch → `RecognizedNotAnalytic`
   (radius still reported from torus minor / cylinder radius). NURBS support
   → `NotAnalytic`. **Fail-closed core: editability requires the band to be
   the provable blend of its supports, not to resemble one.**
6. **Spline-band curvature test.** New `principal_curvatures` helper in
   `brepkit-math` built on `NurbsSurface::derivatives(u,v,2)` (first/second
   fundamental forms → shape operator eigenvalues). One principal curvature
   constant 1/r over a ≥5×5 sample AND both boundary edges tangent →
   spline band, `Constant` or `Variable { r_min, r_max }`, verdict always
   `NotAnalytic`. This may improve UI display, but **never upgrades a NURBS
   face to `ConfirmedRollingBall`**; only exact elementary normalization plus
   re-derivation may do that. Non-constant → not a blend. No spine fitting.
7. **Convexity** from the quadrant material-probe result, consistent across
   every spring run. Mixed or unclassifiable results make the band
   non-editable.
8. **Chains/networks.** Connected components over tangent edges; order with
   `order_chain`-style walking (`g1_chain.rs:261`). `Network` when a spring
   edge lies on a face that is itself a band (blend-on-blend) — record the
   dependency ("B rides A") for §4.3.

### 3.3 Tier 1: `resize_blend`

```
pub fn resize_blend(
    topo: &mut Topology,
    solid: SolidId,
    band_face: FaceId,
    new_radius: f64,
) -> Result<BlendEditResult, OperationsError>

pub struct BlendEditResult {
    pub solid: SolidId,
    pub evolution: EvolutionMap,   // Construction origin; unresolved => refuse
}
```

Preconditions, each a typed refusal:

| # | Condition | Refusal |
|---|---|---|
| P1 | band is `Constant`, `ConfirmedRollingBall` | `Unsupported { op: "resize-blend", .. }` |
| P2 | `new_radius` finite, > tol, ≠ old | `InvalidInput` |
| P3 | component not `Network` | `Unsupported { "blend-on-blend network: remove blends outermost-first" }` |
| P4 | every corner patch adjacent to the band is a `Sphere` of radius old_r (rebuildable closed-form) or the band terminates at planar free boundaries | `Unsupported { "band ends at a non-analytic corner" }` |
| P5 | creation path accepts `new_radius` for these supports (dry-run `try_analytic_fillet_surface`; reports an invalid radius/support combination by name) | propagated `BlendError` |

Algorithm (E7/E8 proven for closed rings, E10 for open bands, E11 for
trihedral networks):

1. **Dry-run creation** at `new_radius` exactly as in recognition step 3 —
   the returned `Stripe` supplies the new band surface and closed-form
   spring/cross circles. `None` → refuse.
2. **Surgery on a `copy_solid_with_face_map` copy** (face map → exact
   evolution, same pattern as `push_pull.rs:176`):
   - band face: `set_surface(new_torus_or_cylinder)`;
   - spring edges: `set_curve` to the new circles/lines (E7: plate circle
     radius r_c + r′ at z_top, wall circle radius r_c at z_top − r′);
   - seam/cross edges: recompute per kind — seam tube circle (E7), cross
     arcs on adjacent planar faces (new radius r′, centers at the moved
     spring endpoints), corner-patch boundary circles;
   - corner patches: `Sphere` patches → `set_surface` new sphere (center
     from `tangent_corner_ball` at r′ — the corner-ball center depends only
     on the three support planes and r′, all known); its boundary circles
     retargeted;
   - vertices: `set_point` — closed rings: seam vertices only (E7 moved
     exactly 2); open bands: spring endpoints slide along the adjacent
     faces' boundary edges (line-line intersections, closed form);
   - supports' wires: closed rings need no wire edits (E7); open bands
     shorten/lengthen the adjacent edges sharing the moved vertices — a
     vertex-position change on existing edges, not wire re-sewing (E10);
   - trihedral equal-radius networks retarget the three cylinders, sphere,
     six circles and shared vertices without topology change (E11);
   - invalidate or regenerate every affected pcurve registry entry. Existing
     split code removes stale pcurves; direct curve replacement currently does
     not. An exact edit may not leave stale parameter curves behind.
3. **Evolution (Construction):** band, supports, corners → `modified`
   (identity survives parameter change). If anything would land in
   `unresolved`, refuse instead — per `evolution.rs` module docs, silence is
   its own failure.
4. **Validation gates** (any failure rolls back):
   - `validate_solid` structural;
   - **strict geometry-consistency gate (new):** every moved vertex lies within
     `Tolerance::linear` of every incident curve; every edited edge sample lies
     on both incident face surfaces; every spring satisfies G1 at three
     samples; every cross edge lies on the band and its cap/corner. E7 showed
     `validate_solid` alone misses an off-curve vertex, and check-crate
     vertex-on-curve findings are warnings at a looser default tolerance;
   - closed-form volume delta where available (E7's Pappus check matched
     exactly; straight plane×plane band: ΔV = L·(r_old²−r_new²)·(1−π/4)·wedge
     factor); where no closed form exists, the displacement bound
     (`MAX_HEAL_DISPLACEMENT_FACTOR` pattern, `defeature.rs:33`).

**Pcurve boundary for the MVP.** E14 shows today's imported and kernel-built
analytic blends are pcurve-free. Phase 1 therefore certifies one of two states:

- the complete affected region has no pcurves → preserve that exact 3D-only
  representation and proceed;
- any affected `(edge,face)` has a pcurve → refuse with
  `existing-pcurve-requires-occurrence-regeneration` until the foundation
  track below exists. Never drop, sample, or partially regenerate it.

The full foundation track is occurrence-keyed pcurves (`face,wire,slot` or a
real `CoedgeId`), persistent plane charts, exact bounded line/circle pcurves,
two distinct periodic lifts for torus seam occurrences, and an exact spherical
great-circle lift. The current `(EdgeId,FaceId)` registry cannot represent a
seam edge used twice on the same torus face. `compute_pcurve_on_surface` samples
16 points and fits NURBS, so it is not an exact regeneration path. Arena/STEP
schema migration belongs in kernel infrastructure, not the tier-1 feature MVP.

**In-place vs unfillet-then-refillet:** E8 proves in-place reproduces the
fresh-fillet body bit-for-surface-bit. The two-step route composes two
refusal boundaries at an intermediate sharp body the user never asked for,
and for `RecognizedNotAnalytic` bands it is the only route — that is
tier 1.5, opt-in, with walking-engine NURBS corners disclosed in the result
payload.

### 3.4 Variable-radius bands

Refused with measured `{r_min, r_max}` returned in the error context so the
UI can display "variable radius 2.0–3.5 mm — editing not supported".
Exact impossibility is theorem-level (Farouki–Sverrisson, §8 ref 9).

## 4. (b) Algorithm specification — general unfillet (tier 2)

Design updated with full-text detail from Venkataraman–Sohoni–Rajadhyaksha
(SM'02, fetched from sohoni's IITB page), OCCT master source, and Parasolid
FD chapters 29/30/40 (Wayback mirror). Citations in §8.

### 4.1 What production kernels actually do (verified, not folklore)

- **Parasolid** `PK_FACE_identify_blends` (constant-radius rolling-ball
  only; chains with branch faces; `dependent` mode returns recursively
  dependent chains — the kernel's own blend-on-blend dependency notion) and
  `PK_FACE_delete_facesets` (per-faceset failure via
  `failed_facesets_indices`; `heal_action` incl. rubber-face fallback).
  `PK_FACE_delete_blends` is "as close to an inverse blend as possible"
  **using recorded topological changes from blending** — provenance a dumb
  solid does not have; "cannot recreate any topology that was completely
  destroyed"; only option `check_fa_fa`. Radius editing exists only on
  **unfixed** blends (attributes on edges; overwrite via
  `PK_EDGE_set_blend_constant`), i.e. only with history. `PK_BLENDSF_ask`
  returns `geom_1`, `geom_2`, `radii[2]`, `spine` for a fixed blend face —
  exactly the data our recognizer must reconstruct (supports, radius, spine).
- **OCCT** `BOPAlgo_RemoveFeatures`: connexity blocks of user-supplied
  faces (no blend recognition); extends adjacent faces with
  `BRepLib::ExtendFace` — **analytic surfaces (plane, cylinder, cone,
  sphere, torus): same basis surface, enlarged UV bounds (exact); NURBS:
  `GeomLib::ExtendSurfByLength` (degree-1 extension — a new construction,
  approximate)**; mutual intersections via `BOPAlgo_Builder` general fuse;
  trimming keeps splits not containing extended-boundary edges; solid
  rebuild via `BOPAlgo_MakerVolume`; per-feature failure isolation with
  `BOPAlgo_AlertUnableToRemoveTheFeature`; full `BRepTools_History`.
  Documented limits: adjacent faces must intersect non-emptily and not be
  tangent; extended faces must cover the feature. No spring/cross-edge
  classification, no chain sequencing, no destroyed-support recreation.
- **Venkataraman–Sohoni–Rajadhyaksha** (the most algorithmic treatment):
  recognizer emits chains sequenced in **creation order**; suppress in
  **reverse creation order** so intermediates stay valid; **blend clusters**
  — interacting chains suppressed as one unit with **geometry computation
  postponed until the whole cluster's topology is edited** (avoids unstable
  tangential intersections for edges created between blends); suppression
  procedure per chain: separate at blend-on-blend/cliff edges (vertex split
  operators), recreate destroyed supports (offset-spring-project-sweep
  reconstruction for extrude/revolve traces), then **collapse all cross and
  terminating edges to vertices (KEV), collapse each two-edged blend face to
  a single edge (KEF)**, and only then attach geometry: **edge geometry =
  support₁ ∩ support₂; vertex geometry = created-edge ∩ neighboring
  surface**; corner vertex blends: all leaf chains around the vertex
  suppressed together, the vertex blend collapses to a face-with-one-vertex
  and is removed; degenerate-vertex splitting; misprediction detected by
  overlap with neighboring edges and locally corrected; global-intrusion
  test refuses suppression when extension would penetrate non-local faces;
  failure at any stage → undo, model untouched. Their pipeline maps cleanly
  onto BrepKit: Euler-operator collapse = our wire surgery; geometry
  attachment = our `exact_plane_analytic` + friends; undo = `transactional`.

### 4.2 The exact boundary (unchanged in substance, now evidence-backed)

| Operation | Exact when | Evidence |
|---|---|---|
| Extend a support | plane/cylinder/cone/sphere/torus | OCCT does it exactly by enlarging UV bounds on the same basis surface; analytic surfaces are globally parameterized |
| Extend a support | NURBS | never exact (Shetty–White; OCCT's own `GeomLib` path is a degree-1 construction) → refuse |
| Re-intersect supports | rows of `analytic_intersection.rs` exact paths | `exact_plane_analytic`, `exact_cone_cone`, `exact_cone_cylinder`, `exact_sphere_cylinder` |
| Re-intersect supports | any other pair | marching only → refuse (certified numeric SSI is §9 research) |
| Corner recovery | 3-plane triples (`resolve_corner` exists); plane+plane+quadric closed forms | defeature pattern generalized |
| Corner recovery | vertex blend collapses | SM'02: collapse with the whole leaf-chain cluster |

Levels: **L1** supports ∈ {plane, cylinder}; **L2** add {cone, sphere} under
analytic.rs alignment gates; **L3** anything NURBS/marching — typed refusal.

### 4.3 Removal algorithm (per L1/L2 component)

1. Recognize (§3.2). Bands may be `RecognizedNotAnalytic` — removal needs
   exact *supports* and *intersections*, not an exact band.
2. **Order**: leaf-first on the dependency DAG (reverse creation order,
   SM'02); corner patches dissolve with their last incident band; the
   selection must be upward-closed — removing a host under a rider refuses
   (`"blend carries later blends; remove them first"`). Interacting chains
   → one **cluster**: all topology edits first, geometry last.
3. **Predict topology before editing** (SM'02's central idea, also what
   Parasolid gets free from provenance): per band, recovered edge =
   support₁ ∩ support₂ (must be non-empty, non-tangent — OCCT's documented
   limits are the correct refusal conditions); per wound vertex, recovered
   corner = unique triple intersection among kept supports via the
   generalized `defeature.rs:608` frontier walk — `Many`/`None` stay
   refusals; global-intrusion test: recovered geometry must stay within the
   displacement bound of the wound.
4. **Heal**: topology first (collapse cross/terminating edges; drop band
   faces; insert recovered edges/vertices — the KEV/KEF collapses are wire
   surgery on the copied topology), geometry last (set recovered curves
   from exact intersections; extend supports as parameter-range changes).
   Destroyed-support recreation (SM'02's offset-sweep trace reconstruction)
   is **out of scope** — refuse with `"support face was destroyed by the
   blend; cannot be recreated exactly"`, matching Parasolid's documented
   limit.
5. **Gates**: `validate_solid` + vertex-on-curve + positive volume +
   displacement bound; one transactional op, all-or-nothing (no per-feature
   isolation — matches `fillet_v2`'s contract).

## 5. Certified direct edits for recognized non-blend features

The existing `Feature` enum remains a descriptive/UI query. It is too weak to
authorize edits: hole faces are an unordered group, pocket floors depend on
hash iteration and omit planar walls, and chamfer recognition does not recover
the sharp spine or prove constant setbacks. Editing accepts a separate
certificate whose source hash, complete topology and exactness verdict are
validated immediately before mutation.

### 5.1 Certificate model

```rust
pub struct EditableFeatureCertificate {
    pub source: SolidStamp,              // SolidId + geometry/topology hash
    pub groups: Vec<SurfaceGroup>,       // exact same-domain carriers
    pub boundary_runs: Vec<BoundaryRun>, // ordered, complete ownership
    pub dependencies: Vec<FeatureDependency>,
    pub feature: EditableFeature,
}

pub enum EditableFeature {
    Axial(AxialFeature),                 // holes, bosses, counterbores, cones
    Extruded(ExtrudedFeature),           // pockets, prismatic bosses
    Chamfer(ChamferFeature),
    Taper(TaperFeature),
}

pub struct AxialFeature {
    pub frame: AxialFrame,               // canonical axis, origin, seam ref
    pub material: MaterialSense,         // Add (boss) | Remove (hole)
    pub stages: Vec<AxialStage>,         // ordered by canonical z
    pub interfaces: Vec<AxialInterface>,
    pub ends: [AxialEnd; 2],             // Opening | BlindCap
}

pub struct AxialStage {
    pub carrier_group: SurfaceGroupId,
    pub z: [f64; 2],
    pub radii: [f64; 2],                 // equal=cylinder, unequal=cone
    pub rings: [BoundaryRunId; 2],
}
```

Manufacturing intent is not inferred beyond geometry: a coaxial entry cone is
`ConicalEntry`, not necessarily "countersink"; the UI may choose that label.
Feature dependencies record attached blends/chamfers, cross-holes, islands and
later features. MVP editors require an empty dependency set; later editors may
accept a complete certified closure.

### 5.2 Certified recognition rules

1. Use the corrected material-side/G1 FAG and strict same-domain grouping from
   §3.2. Every feature-region edge is consumed exactly once by an ordered run.
2. Axial stages must share one axis, form an unbranched ordered interval path,
   have two complete rings, and terminate only in certified openings, planar
   annular interfaces, or blind caps. Opposing coaxial holes separated by
   material remain distinct.
3. An extruded pocket/boss requires exact profile correspondence under axial
   projection and one complete wall group per profile segment. Material probes
   establish add/remove sense.
4. A chamfer certificate reconstructs the exact support intersection, then
   proves constant contact setbacks along the spine. An angled plane alone is
   not a chamfer.
5. Taper certification requires a unique pull direction, neutral plane, fixed
   ring, moving ring, and consistent signed wall angle.
6. The certificate hash is checked immediately before editing; stale
   certificates refuse.

### 5.3 Exact feature edit algorithms

For stage height `h` and endpoint radii `r0,r1`:

`V_stage = π h (r0² + r0 r1 + r1²) / 3`.

Every operation independently computes:

`V_expected = V_before + material_sign * (V_feature_new - V_feature_old)`.

- **Simple/staged radius:** retarget cylinder/cone carriers, both rings, seam
  generators and seam vertices. E16 proves the counterbore large-stage case;
  generic `resize_cylindrical_face` is not the staged engine because its
  boolean path could not preserve the analytic stage.
- **Interface/depth:** move the shoulder plane and both rings along the axis;
  adjacent stage intervals change together. E17 proves depth 3→5 exactly.
  Interfaces may not reorder or collapse a stage.
- **Conical entry:** with half-angle `a`, mouth/stem radii `Rm,Rs`,
  `h=(Rm-Rs)/tan(a)`; rebuild cone, rings, and shaft interface. Preserve anchor
  (mouth, stem, or depth) explicitly in the command.
- **Pocket/boss extent:** hold the datum ring, move the terminal face, extend
  all walls, and retarget the terminal ring. A certified single-profile case
  may reuse `push_pull_face` only after exact-only boolean and provenance gates.
- **Profile size:** move selected wall carriers, solve new corners from exact
  intersections, and rebuild datum/terminal loops. Centered and one-sided edits
  are separate commands.
- **Chamfer:** store canonical setback pair `(d1,d2)`, keep supports, recover
  sharp spine, place new contacts, rebuild bevel plane and terminations. Initial
  implementation may transactionally exact-unchamfer then call the existing
  planar chamfer constructor.
- **Taper:** reconstruct the requested absolute wall planes about a fixed
  neutral plane; do not incrementally apply `new-old` to an already drafted
  wall. Existing `draft.rs` plane/corner math is reusable.

Exact edit paths must call `boolean_exact` / `FallbackPolicy::Refuse`, never the
generic boolean that may silently return a faceted mesh fallback. The global
fallback counter is diagnostic, not a sound per-operation proof.

### 5.4 Feature-edit refusals

Stable machine codes, shared where applicable with blend edit:

```text
stale-certificate
ambiguous-material-side
incomplete-boundary
non-analytic-carrier
approximate-intersection
unsupported-dependency
topology-transition
feature-interference
degenerate-result
existing-pcurve-requires-occurrence-regeneration
incomplete-provenance
geometry-validation-failed
volume-mismatch
output-not-recertified
```

The editor refuses if stages cross/reorder, a profile collapses, a pocket
breaks through unexpectedly, unrelated geometry intersects the swept delta,
dependencies are omitted, any required intersection is sampled/marching, any
result face is unattributed, or output recertification does not report the
requested parameter and the same dependency structure.

## 6. Implementation plan (evidence-based)

### 6.1 Phase 0 — foundations (2–3 wks)

1. **Done.** Replaced the FAG angle/concavity implementation before adding a
   recognizer. The shared `query::edge_concavity` classifier covers
   box-convex, L-notch-concave, post-base-concave, hole-rim-convex,
   G1-spring, closed-seam and imported STEP cases; ambiguous results are
   `Unknown`.
2. **Done.** Fixed the E9 post-base fillet creation bug. The regression was
   red first, then the legacy/modern rim assemblers learned to orient concave
   torus bands from the analytic convexity fact. Gate: volume identity to the
   Pappus form used in E7/E9.
3. Extend `crates/blend/src/query.rs`'s curated public geometry API. Its
   topology-independent `GeometricSpine` and carrier-surface query are in place;
   add nameable DTOs with exact `EdgeCurve` spring/cross geometry. Continue to
   hide private `Stripe` / `CircSection` types.
4. Consolidate the robust production closed-rim extractor from
   `fillet_builder_legacy.rs` into pure `ClosedRimGeometry`; keep topology
   materialization private. Split corner solving from topology allocation.
5. Add isolated-copy elementary normalization and co-surface grouping.
6. Add shared transactions, complete provenance coverage validation,
   `boolean_exact`, strict local geometry validation, and pcurve-aware entity
   maps (E13/E14 policy).
7. `blend_recognition.rs` analytic pipeline; remaining NURBS
   candidates classified `NotAnalytic` by surface type alone (step 4's
   curvature test lands with the math helper in phase 1).
   *Tests:* recognition round-trips on `fillet_v2` bodies (radius equality
   at `Tolerance::linear`); every `ConfirmedRollingBall` re-verified by
   re-derive-and-compare; permanent STEP matrix: OpenZCAD four-fillet plate,
   bored-plate negative, wallcut/lipfuse split-patch grouping, scoop
   NURBS→plane normalization. E5/E12 promoted. Add a tracked spherical-corner
   STEP fixture; none exists today.

### 6.2 Phase 1 — tier-1 resize, closed rings (2 wks)

`resize_blend` for closed-ring torus bands (hole rims, post bases after
phase 0) — the E7/E8 path, productionized: dry-run creation, surgery,
vertex-on-curve gate, Pappus volume gate, transactional rollback,
`BlendEditResult` + WASM binding + `FaceEvolutionPayloadV1` emission.
Implement `copy_solid_with_entity_maps` or re-recognize the copied component;
all mutation occurs on the copy. MVP certifies an entirely pcurve-free affected
region (E14); any existing affected pcurve is a typed refusal. In parallel,
extend the copy API to preserve untouched pcurves and return occurrence maps.
*Tests:* E7/E8 promoted to permanent regressions across radii and
directions (grow/shrink); refusal matrix P1–P5 via typed codes
(`blend_failure_code` extended: `blend-not-recognized`,
`blend-not-analytic`, `blend-network`, `blend-corner-unsupported`);
rollback-intact test (`regress_failed_blend_leaves_input_intact` pattern).

### 6.3 Phase 2 — tier-1 resize, open bands, chains, corners (3 wks)

Straight plane×plane bands (E1 structure: line springs + circle-arc cross
edges on adjacent planes), sphere corner rebuild, chain-wide resize with
cross-edge recovery. Vertex slides along adjacent edges (line∩line). E10 and
E11 prove the same-topology rewrite for an open band and a full equal-radius
trihedral network.
*Tests:* E1/E2/E10/E11 promoted; resized ≡ fresh fillet oracle on open bands
and trihedral corners (orthogonal and oblique); chain of mixed bands.

### 6.4 Phase 3 — certified simple/staged features (3 wks)

Ship certified simple hole/boss radius and pocket/boss extent first, then
counterbore stage radius/depth (E16/E17) and conical entries. All use the
certificate/refusal contract in §5 and exact volume identities. Do not use the
current heuristic `Feature` records as edit input.
Phase acceptance includes E18: deterministic stage order, complete ring counts,
interface ownership, and opening/cap classification independent of FaceIds.

### 6.5 Phase 4 — tier-2 unfillet L1 (3–4 wks)

`remove_blend` for plane/cylinder-supported components: dependency DAG,
cluster formation, predict-then-heal, generalization of the extend heal to
curved wound edges. The single open plane-plane case is already implemented in
`defeature` (E6); E19 proves the exact closed plane-cylinder topology collapse.
Phase work generalizes those rewrites over certified chains/clusters and adds
construction provenance.
*Tests:* fillet→unfillet returns the original sharp body surface-for-surface
(strongest oracle); unfillet on imported STEP fillets; dependency-order
refusals on sequentially built blend-on-blend fixtures; all defeature
refusal analogues preserved; volume/displacement gates.

### 6.6 Phase 5 — tier-2 L2 + chamfer/taper + provenance v2 (3 wks)

- Cone/sphere rows of §4.2.
- Chamfer distance/angle (mirror of §3.3 with `try_analytic_chamfer`) and
  certified planar/conical taper edits.
- `feature_edits` payload member (§7).
*Tests:* group edits with per-band volume gates; payload round-trip next to
`types.rs:540` decode tests; STEP corpus round-trips.

### 6.7 Cross-cutting test strategy

Repo patterns to follow: `assert_verified_solid` + coarse/medium/fine volume
convergence (`feature_recognition.rs:963`); typed-refusal assertions
(`defeature.rs:809`); transactional-rollback assertions; STEP corpus
parity; `tessellation_parity_inmem.rs` for geometric identity of results.
Property test to add: recognition never emits `ConfirmedRollingBall` that
fails re-derivation (proptest, `coincident_proptest.rs` pattern). The probe
file itself is cleaned up and kept as the research evidence suite.

### 6.8 Kernel pcurve foundation (parallel, not on the MVP critical path)

1. Introduce occurrence identity (`CoedgeId` preferred; otherwise
   `{face,wire,slot}`) and occurrence-keyed ordered pcurve loops.
2. Persist plane charts and analytic reference frames; arena schema v3 and
   STEP reader/writer support for `PCURVE`/`SEAM_CURVE`.
3. Add exact pcurve variants/factories for bounded UV lines/circles, torus
   seam lifts at `u0` and `u0+2π`, and spherical great-circle lifts.
4. Make `copy_solid_with_entity_maps` clone/remap all occurrences and pcurves.
5. Add UV-loop closure, periodic-lift and exact SameParameter validation.
6. Remove the MVP pcurve refusal only for edit cases whose complete exact
   occurrence pcurve set can be generated.

## 7. Selection persistence (validated against existing machinery)

1. Every edit op emits `EvolutionMap` with `EvolutionOrigin::Construction`
   through `FaceEvolutionPayloadV1` — no schema change for the core claims.
   `resize_blend`'s map is strong: band/supports/corners are `modified`, so
   rebinding is exact — E8 showed identity survives parameter change.
2. Additive optional payload member `feature_edits: [{ op, band_in,
   band_out, old_radius, new_radius, component_hash }]` where
   `component_hash` = content hash of the component's face set + parameters
   (feature-level analogue of ADR-011 face fingerprints); lets OpenZCAD
   rebind a feature selection whose member faces were re-trimmed.
3. App-side fingerprint fallback stays, but faces that would land in
   `unresolved` make the op refuse (tier 1/2 policy) — wrong rebind is worse
   than a dropped selection (`evolution.rs` module rule).
4. Kripac mapping: `modified` = identity carried; split/merge during an
   edit (support consumed, degenerate created) = his ambiguity case →
   refuse; Raghothama–Shapiro br-variance: an edit crossing a topological
   stratum invalidates neighborhood identity → never claim `modified`
   there. Recognition re-derives feature identity per edit from geometry
   (Bidarra history-independent stance); arena ids stay ephemeral.

## 8. (a) Annotated survey — final reference list

Twenty-two verified references (full-text where noted). Items marked ‼
correct a claim in the task brief.

**Recognition.**
1. Venkataraman, Sohoni, Elber, "Blend recognition algorithm and
   applications," ACM SMA'01, 99–108. DOI 10.1145/376957.376970. Canonical
   blend-network recognizer; emits chains sequenced in creation order (the
   ordering our unfillet inverts). Closed access; §3 of ref 11 is a
   self-contained review.
2. Zhu & Menq, "B-Rep model simplification by automatic fillet/round
   suppressing," CAD 34(2):109–123, 2002. DOI 10.1016/S0010-4485(01)00056-2.
   Surface-type + smooth-edge detection, radius from surface parameters,
   chain grouping, fillet/round from support convexity — the pipeline §3.2
   makes fail-closed.
3. Slyadnev & Turlapov, "Simplification of CAD Models by Automatic
   Recognition and Suppression of Blend Chains," Prog. Comp. Software
   46(3):233–243, 2020. DOI 10.1134/S0361768820030081. Modern open-kernel
   AAG+chain+Euler-operator confirmation.
4. Joshi & Chang, "Graph-based heuristics…," CAD 20(2):58–66, 1988.
   DOI 10.1016/0010-4485(88)90050-4. AAG origin; our FAG is one.
5. Li, Tong, Shi, Geng, Zhu, Hagiwara, "Automatic small blend recognition…,"
   Eng. with Computers 25(3):279–285, 2009. DOI 10.1007/s00366-009-0127-4.
6. Babic, Nesic & Miljkovic survey (Comp. in Industry 59(4), 2008,
   DOI 10.1016/j.compind.2007.09.001) and Han, Pratt & Regli status report
   (IEEE TRA 16(6), 2000, DOI 10.1109/70.897789 ‼ venue not TVCG).
   Negative evidence: no published recognizer exceeds constant-radius
   rolling-ball on exact B-reps.

**Blend geometry.**
7. Rossignac & Requicha, "Constant-radius blending in solid modelling,"
   U. Rochester PAP TM-42, 1984 (hdl.handle.net/1802/26371). Rolling-ball
   definition; the recognizer's invariant (one principal curvature = 1/r).
8. Lukács, "Differential geometry of G¹ variable radius rolling ball blend
   surfaces," CAGD 15(6):585–613, 1998, DOI 10.1016/S0167-8396(98)00006-5;
   Peternell & Pottmann, "Rational Parametrizations of Canal Surfaces,"
   JSC 23(2–3), 1997, DOI 10.1006/jsco.1996.0087. Canal-surface model behind
   the §3.2 step-4 test.
9. Farouki & Sverrisson, "Approximation of rolling-ball blends…," CAD
   28(11):871–878, 1996. DOI 10.1016/0010-4485(96)00008-5. Exact blends are
   generically non-rational → tier-1 NURBS refusal is theorem-level.
10. Várady & Rockwood, "Setback vertex blending," CAD 29(6):413–425, 1997,
    DOI 10.1016/S0010-4485(96)00070-X; Braid, "Non-local blending," CAD
    29(2):89–100, 1997, DOI 10.1016/S0010-4485(96)00038-3. Corner-patch
    topology; non-local propagation → network-wide dependency walk.

**Suppression.**
11. Venkataraman, Sohoni, Rajadhyaksha, "Removal of blends from B-rep
    models," ACM SM'02, 83–94. DOI 10.1145/566282.566297. **Full text
    fetched** (cse.iitb.ac.in/~sohoni/delblend.pdf). Reverse-creation-order,
    clusters with postponed geometry, KEV/KEF collapse then
    edge=S₁∩S₂ / vertex=edge∩surface geometry attachment, corner cluster
    collapse, destroyed-support recreation (which we refuse instead),
    misprediction/overlap correction, global-intrusion test, undo-on-failure.
    §4.3 is this algorithm restricted to the provable subset.
12. Venkataraman & Sohoni, "Reconstruction of feature volumes and feature
    suppression," ACM SM'02, 60–71. DOI 10.1145/566282.566295 ‼
    two-authored, not with Kulkarni; Sandiford & Hinduja, "Construction of
    feature volumes using intersection of adjacent surfaces," CAD 33(6),
    2001, DOI 10.1016/S0010-4485(00)00096-8. Delta-volume alternative;
    mental model behind volume gates.
13. Shetty & White, "Curvature-continuous extensions for rational B-spline
    curves and surfaces," CAD 23(7):484–491, 1991.
    DOI 10.1016/0010-4485(91)90046-Y. NURBS extension is a new construction —
    the fail-closed boundary, independently confirmed by OCCT's two-path
    `BRepLib::ExtendFace` (§4.1).
14. Parasolid FD ch. 29/30/40 (Wayback q-solid mirror, fetched):
    `PK_FACE_identify_blends` (constant-radius only; chains; `dependent`
    recursion), `PK_FACE_delete_facesets` (per-faceset failure; heal
    actions), `PK_FACE_delete_blends` (inverse-blend via recorded
    provenance; cannot recreate destroyed topology; `check_fa_fa`),
    unfixed-blend radius editing (`PK_EDGE_set_blend_constant` overwrite),
    `PK_BLENDSF_ask` (= geom₁, geom₂, radii ranges, spine — the record our
    recognizer reconstructs). Ch. 31–32 (overflows, error codes) not
    archived anywhere.
15. OCCT `BOPAlgo_RemoveFeatures`/`BRepAlgoAPI_Defeaturing` (7.3.0+, 2019
    ‼ not 7.8; master source fetched): connexity blocks; analytic
    `ExtendFace` exact / NURBS `GeomLib` approximate; `BOPAlgo_Builder`
    intersections; MakerVolume rebuild; per-feature failure isolation;
    `BRepTools_History` Modified/Generated/IsDeleted. No blend-specific
    recognition at all — §4.3 with §3.2 in front is strictly stronger.
16. Thakur, Banerjee & Gupta, "A survey of CAD model simplification
    techniques…," CAD 41(2):65–80, 2009. DOI 10.1016/j.cad.2008.11.009.

**Direct modeling.**
17. Siemens Synchronous Technology Live Rules docs + TechniCom whitepaper
    (2008, PDF verified): found vs persistent relationships; concentric /
    tangent / parallel / coplanar / perpendicular / symmetry / same-radius
    invariant set. §5 adopts found-relationship queries, refuses the solver
    where ambiguous.
18. SpaceClaim patents US 7,639,267 B1; US 7,830,377 B1; US 8,244,508 B1.
    Product context; heuristic, no exactness contract.

**Naming.**
19. Kripac, SMA'95 (DOI 10.1145/218013.218024) / CAD 29(2), 1997
    (DOI 10.1016/S0010-4485(96)00040-1); Farjana & Han survey (Alexandria
    Eng. J. 57(4), 2018, DOI 10.1016/j.aej.2018.01.007); Marcheix & Pierra
    survey (SM'02, DOI 10.1145/566282.566288).
20. Bidarra, de Kraker & Bronsvoort, "…cellular model," CAD 30(4), 1998,
    DOI 10.1016/S0010-4485(97)00070-5 ‼ title corrected; Raghothama &
    Shapiro, "Boundary Representation Deformation…," ACM TOG 17(4), 1998,
    DOI 10.1145/293145.293148 ‼ title corrected. History-independent feature
    identity; br-variance justifies the stratum rule in §7.
21. (supporting) Nyirenda, Bidarra, Bronsvoort, "A Semantic Blend Feature
    Definition," CAD&A 4(6), 2007, DOI 10.1080/16864360.2007.10738512 —
    blends as first-class features with validity rules.
22. (supporting) Agbodan et al., SMI 2003, DOI 10.1109/SMI.2003.1199623;
    Wang & Nnaji, CAD 37(10), 2005, DOI 10.1016/j.cad.2004.11.009 —
    geometric entity matching / semantic IDs; commercial direct modelers'
    fingerprint schemes are know-how, not literature (unverified).

**Unverifiable seeds:** "Cui, Gao et al. blend recognition" (closest verified:
Sun/Gao/Zhao C&G 34(5), 2010, DOI 10.1016/j.cag.2010.06.007; Gao et al. CAD
42(12), 2010, DOI 10.1016/j.cad.2010.05.010); "Li, Langbein & Martin blend
recognition" (their verified works: symmetry design-intent CAD 42(3), 2010,
DOI 10.1016/j.cad.2009.10.001; constrained fitting CAGD 19(3), 2002,
DOI 10.1016/S0167-8396(01)00085-1); ACIS `api_remove_blend`/`api_defeature`
(ACIS LOP extension capability and a shipping `SPADefeature` library verified
instead); CGM operator names (product-level defeaturing verified only).

## 9. (d) Open research vs. engineering

**Engineering (validated or known):** everything in phases 0–5. E5 proves
re-derive-and-compare recognition; E7/E10/E11 prove ring/open/corner resize;
E16–E18 prove staged-feature certification/editing; E6/E19 prove both tier-2
L1 primitive unfillet geometries. The remaining work composes these pieces and
published network algorithms (refs 2, 11, 15) against the exactness contract.

**Open or genuinely risky:**

1. **Certified numeric SSI.** Unfillet across general analytic pairs needs
   marching intersection with *certified* curve topology (no missed loops,
   explicit bounds). Interval-method literature exists; nothing
   production-ready to adopt. Tier-2 hard boundary until then.
2. **Variable-radius band edit.** Recognition thin (fitting-only RE work);
   exact edit impossible for NURBS bands (ref 9). If ever required: re-run
   the walking engine with a new radius law on a recovered spine — a
   creation, not an edit; spine recovery for non-analytic bands is unproven.
3. **Blend-on-blend corner cases.** Cyclic/mutually dependent networks are
   where provenance-less ordering runs out (refs 1, 11 assume creation
   order is recoverable); expect the refusal set to grow with experience,
   never the tolerances.
4. **Fail-closed live-rules solving.** Found-relationship queries are
   engineering; a solver that acts only on unique provable solutions is a
   worthwhile prototype, not a commitment.
5. **Naming across edit sequences.** Single-edit provenance is engineering
   (§7); fingerprint drift across long direct-edit sessions wants
   measurement before any strengthening.
6. **E9-class creation bugs.** Recognition can only re-derive what creation
   can produce. The post-base case is fixed and pinned, but more creation-gap
   cases should be expected along the convex/concave × bounded/unbounded table
   of `plane_cylinder_fillet`.

The refusal boundary is the deliverable: every tier is defined by what it
can prove. Production kernels draw the same line with more provenance and
fewer scruples — Parasolid leans on recorded blend history, OCCT extends
NURBS approximately; BrepKit's contribution is making the line exact, typed,
and — per §2 — tested.

## Appendix: research infrastructure in the repo

- `crates/operations/tests/research_blend_edit_probe.rs` — E1–E19 evidence
  suite (green). E1–E19 promote to permanent regressions in phases 0–4; E9's
  post-base creation fix is additionally pinned by
  `regress_hole_rim_blend.rs`.
- `crates/blend/src/query.rs` — narrow public carrier-surface query used by
  E5; private `analytic`, `spine`, and `stripe` types remain hidden.
- `crates/operations/Cargo.toml` — `brepkit-io` dev-dependency for the E4
  STEP round-trip probe.
