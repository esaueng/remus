# B17/PERF-H03 duplicate-face equivalence and removal contract

Scope: `remus-heal` duplicate-face repair (`crates/heal/src/fix/duplicate_faces.rs`,
reached via `fix_solid`). This note is the M1 contract for the B17/PERF-H03
campaign: what "duplicate" means, what removal is allowed to do, and what can
never count as proof. The code's module docs restate the supported domain; this
note carries the reasoning.

## 1. Vocabulary

- **Carrier**: the face's supporting surface (a `FaceSurface::Plane` with stored
  normal and offset here; analytic circle/arc boundaries in §5 extend the
  boundary classes, never the carrier class — NURBS and other analytic
  carriers stay out of scope).
- **Trimmed region**: the carrier patch cut out by the face's boundary wires
  (one outer wire, zero or more hole wires), oriented by the face's winding
  and `reversed` flag.
- **Face identity**: the arena `FaceId`. Two faces sharing one `FaceId` in two
  shells are one identity with two uses (see §7), never a duplicate pair.
- **Duplicate pair**: two distinct `FaceId`s whose trimmed regions coincide as
  oriented sets (criteria in §3).
- **Removable duplicate use**: a duplicate pair member that may be dropped from
  its shell without changing the solid's material boundary (§6).

## 2. What must be established jointly

Removal is sound only when ALL of the following hold for the pair:

1. **Equal carriers** (§3.1): same supporting plane within the angular gate,
   with plane-offset agreement implied by boundary coincidence (§3.1 proves the
   bound; offset is never bucketed — see the M2 note).
2. **Equal trimmed regions** (§3.2–§3.3): the outer boundaries trace the same
   closed curve with the same winding (up to cyclic shift and documented
   subdivision normalization), and holes correspond one-to-one with coincident
   boundaries.
3. **Equal orientation** (§3.4): effective normals agree (stored normal composed
   with the `reversed` flag). Opposite-facing coincident faces are preserved:
   dropping one silently picks a side of a zero-thickness or malformed region.
4. **Distinct identity, shell-local scope** (§7): the two `FaceId`s differ and
   live in the same shell. A shared `FaceId` across shells aborts the pass.
5. **Attribute compatibility** (§8): the dropped face carries no attribute the
   survivor lacks.

The geometric predicate (`faces_are_duplicates`) is the final authority for
1–3. Candidate buckets (M2) only select pairs to test; they can never promote
a pair to duplicate.

## 3. Equivalence criteria

### 3.1 Supporting-plane agreement

Effective plane normals `na`, `nb` (stored normal, negated when `reversed`)
must satisfy `na·nb ≥ 1 − 1e-6` (≈0.08°). The gate is absolute, never derived
from the model's linear tolerance, so a coarse tolerance cannot widen the
angular test into matching visibly different orientations.

Boundary coincidence then bounds the plane offset: when every boundary vertex
of B lies within `tol` of A's boundary and both boundaries lie on their
planes, the planes' offsets differ by at most ~`2·tol` plus the tilt term
(`tilt·extent`, bounded by the angular gate). No separate offset gate is
needed, and — critically for M2 — offset is never used to *exclude* pairs.

### 3.2 Outer-region equality (line-bounded class)

Two outer wires bound the same oriented region iff their vertex sequences
coincide pointwise within `tol` under some cyclic shift, in the same winding
order. Cyclic shifts are valid differences (wire start vertices are
arbitrary); reversed order is a genuine difference (opposite winding) and
never matches.

Subdivision normalization (M3): consecutive collinear edges may be merged
before comparison, because splitting one straight side into two collinear
edges does not change the traced region. Normalization only *removes*
intermediate vertices whose deviation from the straight chord is within `tol`
AND whose projection falls strictly between the chord endpoints — it never
moves a vertex, so a thin sliver region cannot be snapped away. A chord is
never equated with an arc: curve-type sequences must agree elementwise
(§5).

### 3.3 Hole correspondence (M3)

A perforated face duplicates another iff, in addition to §3.1–§3.2 on the
outer wire: both faces carry the same number of hole wires, and the holes
match bijectively — every hole of A coincides (same cyclic-shift-tolerant
boundary test, same curve-type discipline) with exactly one hole of B and
vice versa. Hole storage order is arbitrary, so matching is by search, not by
index. Nested holes (a hole inside a hole) are not representable as face
holes and never match. Reordered holes match; added/removed/shifted holes do
not.

### 3.4 Orientation equality

`is_reversed` participates through the effective normal (§3.1): same boundary
with the flag set on exactly one face fails the normal gate and is kept.
Winding participates through §3.2: the same corners in opposite order fail
the boundary gate and are kept. Both keeps are pinned as specified behavior,
not misses.

### 3.5 Explicitly refused classes

Curved carriers (cylinder/cone/sphere/torus/NURBS), NURBS or conic (non-circle)
boundary curves, ellipse/hyperbola/parabola edges, and any face the
descriptor path cannot classify conservatively are skipped: they are left in
place with no disclosure, never removed. Skipping is fail-closed; a missed
duplicate costs a redundant face, a wrong removal corrupts material.

## 4. Non-proofs

None of the following, alone or combined, establishes duplication — each has
a same-value/different-region counterexample pinned in tests:

- **Equal area**: a small triangle and a larger concentric triangle share a
  centroid-relevant footprint class but bound different regions (kept).
- **Equal centroid**: concentric different-size boundaries share a centroid.
- **Equal endpoint/vertex sets**: the same corners in opposite winding bound
  the same unoriented triangle but opposite oriented regions; subdivision
  vertices can also coincide while regions differ.
- **Sampled (Hausdorff-style) distance**: a chord and its arc agree at the
  endpoints and stay within deflection at samples yet bound different
  regions; sampling density, not geometry, decides the verdict.
- **Equal edge count + parallel normals**: necessary but far from sufficient;
  two disjoint coplanar quads match that description.

## 5. Supported domain (grows only by proof)

| Domain | Status |
|---|---|
| Unperforated planar faces, all-`Line` outer boundary | Supported since B11 (winding-aware compare) |
| Perforated planar faces, all-`Line` boundaries (§3.3) | Supported by this campaign (M3) |
| Planar faces with `Line`+`Circle` boundaries (open arcs, closed rims) | Supported under the circle contract (M3): same supporting plane; elementwise curve-type agreement (a `Line` never matches a `Circle`); circle centers within `tol`, radii within `tol`, arc spans covering the same angular interval within an angle tolerance derived from `tol`/radius; closed rims compared phase-invariantly |
| Ellipse/hyperbola/parabola/NURBS boundaries, non-planar carriers | Refused (left in place) |

## 6. Removal semantics

- **Deterministic survivor**: within each duplicate group the lowest `FaceId`
  index survives; every other member is recorded for removal. Index order is
  arena-stable, so the outcome does not depend on shell face order, bucket
  order, or process scheduling.
- **Greedy groups, deterministic**: pairs are considered in survivor-index
  order; a face already marked removed neither survives another pair nor
  anchors a new group (nontransitive nearness: A≈B, B≈C, A≉C keeps C — pinned
  by adversary tests).
- **Shell validity**: removal only drops faces from their shell's face list
  via `ReShape::remove_face`; the survivor's wires, edges, vertices, and
  pcurves are untouched. The `ReShape::apply` don't-empty-a-retained-shell
  guard stays as the backstop, and the survivor rule guarantees it never
  triggers for this pass (the lowest-index member is never marked).
- **Material preservation**: same oriented region + same orientation ⇒ the
  surviving boundary carries the same material side; volume and classification
  probes must agree before/after (M4 qualification).
- **Idempotence**: after a repair, no duplicate pair remains, so a second run
  records zero removals and returns `Status::OK` with no actions.

## 7. Cross-shell sharing

Coincident boundaries in different shells do not establish that either use
can be removed (cavity walls, thin-wall pairs, and legitimate coincident
sheets all look alike geometrically). Concretely: if a face marked for
removal is referenced by more than one shell, the whole pass fails with
`HealError::FixFailed("duplicate face is shared by multiple shells")`
*before recording any removal* — full rollback by construction, since
`fix_duplicate_faces` only records into `ctx.reshape` and the error aborts
`fix_solid` before `apply`.

## 8. Attributes and provenance policy

Face attributes (`EntityAttributes`: name, color) are application
vocabulary; the kernel never synthesizes or merges them. Removal therefore
requires compatibility: the removed face's attributes must be empty or equal
to the survivor's. A removed face carrying a name/color the survivor lacks
(or a conflicting value) vetoes that pair — both faces stay, with an info
message naming the veto. Equal attributes on both sides remove normally (the
survivor keeps its own copy; nothing is transferred).

Provenance: `ReShape::entity_history` maps each removed `FaceId` to an empty
claim (deletion). The survivor is reconstructible without new public shapes:
the pass info message names each `(survivor → removed)` pair, and the
survivor rule (§6) makes it deterministic. No public enum or struct changes
ship just to expose this; stats for scaling analysis travel through
`pub(crate)` channels and test-only hooks.

## 9. Tolerance-boundary discipline

Boundary coincidence uses strict `< tol` per-vertex comparison against the
linear tolerance in force for the pass. Faces engineered to sit exactly at
the boundary (`dist == tol`) do not match; faces at `tol·(1±ε)` behave
identically regardless of scale (the tolerance is absolute, not
scale-relative — pinned across 1e-3/1/1e3 by the B17 matrix convention).
Nontransitive nearness is resolved by index-order greed (§6), never by
transitive closure, so no chain of near-matches can drag a distant face into
a group.
