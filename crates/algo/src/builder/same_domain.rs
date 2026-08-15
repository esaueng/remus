//! Same-domain face detection via edge-set hashing.
//!
//! When two faces from opposing solids share the same underlying surface
//! AND identical boundary edge sets (same vertex pairs), they are "same-domain"
//! faces. This module detects SD groups using edge-set hashing and union-find,
//! returning `SameDomainPair` records for downstream use.
//!
//! The SD pair list is used by [`crate::bop::select_faces`] to apply
//! operation-specific deduplication (fuse keeps one representative,
//! cut keeps B reversed, etc.) without encoding operation semantics
//! into the classification pipeline.
//!
//! **Note:** Representative replacement (substituting all group members'
//! images with a single representative face) is not yet implemented.
//! Currently only pairwise SD records are emitted.

use std::collections::{HashMap, HashSet};
use std::hash::BuildHasher;

use super::SubFace;
use crate::ds::{GfaArena, Rank};
use crate::error::AlgoError;
use brepkit_math::tolerance::Tolerance;
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::face::{FaceId, FaceSurface};

/// A detected same-domain face pair.
#[derive(Debug, Clone)]
pub struct SameDomainPair {
    /// Sub-face index from solid A.
    pub idx_a: usize,
    /// Sub-face index from solid B.
    pub idx_b: usize,
    /// `true` if the effective oriented normals (surface normal combined
    /// with face reversal) point the same direction, `false` if opposite.
    pub same_orientation: bool,
    /// `true` if this is a geometric-overlap pair — one face is contained in
    /// or partially over the other, so the two faces differ in extent. For
    /// edge-set matched (coextensive) faces both boundaries coincide, so this
    /// is `false`. Detection signal observed by tests; the BOP selector orders
    /// the pair via [`Self::representative`] (the larger face) rather than
    /// reading this flag directly.
    #[allow(dead_code)]
    pub geometric_overlap: bool,
    /// Sub-face index of the **larger** face of this pair by projected outer-
    /// wire area (see [`repr_face_area`] — planar faces in their plane,
    /// cylinder/cone faces in `(arc-length, axial)` space), used to keep
    /// representative selection order-independent.
    ///
    /// For coextensive (edge-set) pairs both faces span the same domain, so
    /// area ties and this is `idx_a` — matching historical behaviour. For a
    /// geometric-overlap pair (`geometric_overlap == true`) the two faces have
    /// **different extent**, so the larger is chosen by area rather than by
    /// which operand is A; `idx_a` flips with operand order, so an A-only rule
    /// would make the result order-dependent.
    ///
    /// "Larger" — not "containing": a geometric-overlap pair may be strict
    /// containment OR partial overlap ([`planar_faces_overlap`] and
    /// [`analytic_faces_overlap`] both accept either), so neither face
    /// necessarily contains the other. The consumer
    /// ([`crate::bop::select_faces`]) keeps this face for Fuse (it covers the
    /// most boundary) and the *other* (smaller) face for Intersect (whose
    /// footprint is bounded by both solids).
    pub representative: usize,
}

/// A within-rank duplicate sub-face: same edge set, same surface, same input
/// solid as another face. Issue #696: sequential boolean operations
/// (`booleanPipeline` in the consumer) accumulate stale coincident faces in
/// the input solid; when the next boolean splits its inputs into sub-faces,
/// these duplicates produce 3+-face junctions in the output topology that
/// tessellate as branching mesh edges. The `representative` is the lowest-
/// indexed sub-face in the group; `duplicate` should be excluded from the
/// boolean result.
#[derive(Debug, Clone, Copy)]
pub struct WithinRankDuplicate {
    /// Sub-face index that stays in the result.
    pub representative: usize,
    /// Sub-face index that should be dropped.
    pub duplicate: usize,
}

/// Output of [`detect_same_domain`].
#[derive(Debug, Default, Clone)]
pub struct SameDomainResult {
    /// Cross-rank pairs (one face from A, one from B).
    pub pairs: Vec<SameDomainPair>,
    /// Within-rank duplicates (multiple faces from the same input solid
    /// occupying the same domain — boolean residue that needs removing
    /// before classification).
    pub within_rank_dups: Vec<WithinRankDuplicate>,
}

/// Number of points sampled along each outer-wire edge when building the
/// projected polygon for the coplanar containment / overlap / area tests.
/// Defined once so [`planar_faces_overlap`] and [`planar_face_area`] keep the
/// same density — an arc boundary must sample to the same polygon in both, or
/// the area-based representative pick could disagree with the overlap test.
const SD_EDGE_SAMPLES: usize = 8;

/// Quantized 3D grid position — collision-free vertex identity.
type QVert = (i64, i64, i64);

/// Canonical representation of a face's edge set for SD detection.
///
/// Each edge is stored as a sorted quantized vertex pair `(min, max)`.
/// The set of pairs is sorted for deterministic comparison.
// Endpoint pair + weld-quantized curve midpoint. The midpoint discriminates
// co-endpoint edges with different geometry: the two halves of a chord-split
// cap disc share BOTH vertices (chord + arc), and endpoint-only sets made
// them false within-rank duplicates — the outside half was silently dropped
// as "SD residue". The midpoint bucket is 100x coarser than the endpoint
// bucket so marched/fitted geometry (~1e-6 off exact) cannot split a true
// duplicate pair across buckets.
type EdgeSet = Vec<(QVert, QVert, QVert)>;

/// Detect same-domain face pairs using edge-set hashing.
///
/// Algorithm:
/// 1. For each sub-face, compute its canonical edge set (sorted vertex pairs)
/// 2. Hash the edge set and group faces with identical sets
/// 3. Within each group, verify surface equivalence across opposing solids
/// 4. Build SD pairs via union-find for transitive closure
///
/// Returns a list of SD pairs WITHOUT modifying sub-face classifications.
/// The BOP selector uses these pairs for operation-specific handling.
#[allow(clippy::too_many_lines)]
/// Coincidence grouping shared by the 2-operand and N-way SD emissions.
///
/// `sd_groups` maps each union-find root to the coincident sub-face indices in
/// that group; `pair_data` maps a directly-unioned `(min, max)` pair to whether
/// the two faces share outward orientation; `geometric_overlap_groups` holds the
/// roots whose union came from the geometric-containment pass. The grouping is
/// rank-agnostic — the emission step interprets it per boolean operation.
struct SdGrouping {
    sd_groups: HashMap<usize, Vec<usize>>,
    pair_data: HashMap<(usize, usize), bool>,
    geometric_overlap_groups: HashSet<usize>,
}

/// Detect and union all coincident (same-domain) sub-faces.
fn build_sd_grouping(
    topo: &Topology,
    arena: &GfaArena,
    sub_faces: &[SubFace],
    tol: Tolerance,
) -> SdGrouping {
    let n = sub_faces.len();
    if n < 2 {
        return SdGrouping {
            sd_groups: HashMap::new(),
            pair_data: HashMap::new(),
            geometric_overlap_groups: HashSet::new(),
        };
    }

    // Use quantized vertex positions (not VertexId) so that VV-merged
    // vertices from different solids that share the same position produce
    // matching edge sets.
    let scale = 1.0 / tol.linear;

    let edge_sets: Vec<Option<EdgeSet>> = sub_faces
        .iter()
        .map(|sf| compute_edge_set_quantized(topo, arena, sf.face_id, scale))
        .collect();

    // Same boundary, but walked in traversal order. Distinguishes ADJACENT
    // (glued) patches from COINCIDENT (duplicate) ones — see
    // `opposite_boundary_traversal`.
    let directed: Vec<Option<DirectedBoundary>> = sub_faces
        .iter()
        .map(|sf| compute_directed_boundary_quantized(topo, arena, sf.face_id, scale))
        .collect();

    // Key = edge set, Value = list of sub-face indices with that set.
    let mut groups: HashMap<EdgeSet, Vec<usize>> = HashMap::new();
    for (idx, edge_set) in edge_sets.iter().enumerate() {
        if let Some(es) = edge_set
            && !es.is_empty()
        {
            groups.entry(es.clone()).or_default().push(idx);
        }
    }

    let surfaces: Vec<Option<&FaceSurface>> = sub_faces
        .iter()
        .map(|sf| {
            topo.face(sf.face_id)
                .ok()
                .map(brepkit_topology::face::Face::surface)
        })
        .collect();

    // Surface normals alone don't define orientation: faces kept through a
    // Cut carry their original surface with a reversal flag, so the
    // effective normal is the surface normal flipped when reversed.
    let reversed: Vec<bool> = sub_faces
        .iter()
        .map(|sf| {
            topo.face(sf.face_id)
                .is_ok_and(brepkit_topology::face::Face::is_reversed)
        })
        .collect();

    let mut uf = UnionFind::new(n);
    let mut pair_data: HashMap<(usize, usize), bool> = HashMap::new(); // (min,max) → same_orientation
    // Tracks pairs unioned by the geometric containment pass (Step 3b).
    // Cross-rank groups containing such pairs are "overlapping" same-domain
    // faces (one face contained in / partially over the other) rather than
    // exactly coextensive. `geometric_overlap` records this so the BOP selector
    // can pick the larger face for Fuse and the smaller for Intersect; the two
    // faces always differ in extent for these pairs.
    let mut geometric_overlap_groups: HashSet<usize> = HashSet::new();

    for members in groups.values() {
        if members.len() < 2 {
            continue;
        }

        // Check all pairs within this edge-set group. Pairs can be cross-rank
        // (the classic SD case — same domain across two input solids) or
        // within-rank (issue #696 — boolean residue accumulated in one input
        // across sequential operations). Both unify into the same group; the
        // representative-emission step below splits them by rank composition.
        for (mi, &i) in members.iter().enumerate() {
            let Some(surf_i) = surfaces[i] else {
                continue;
            };

            for &j in &members[mi + 1..] {
                let Some(surf_j) = surfaces[j] else {
                    continue;
                };

                if let Some(same_dir) = surfaces_same_domain(surf_i, surf_j, tol) {
                    // Complementary partition regions of ONE input face's split
                    // (same source, distinct interiors — e.g. the in-tube and
                    // out-tube parts of a box wall cut by a torus) are never
                    // same-domain duplicates of each other. A genuine coincident
                    // same-source duplicate (same interior) is NOT excluded here.
                    if same_source_complementary_split(sub_faces, i, j, tol) {
                        continue;
                    }
                    // Two curved faces of the same underlying surface can share an
                    // outer-wire edge set yet cover DIFFERENT regions — e.g. the
                    // two hemisphere bands of a bored sphere share the equator
                    // polygon but lie on opposite halves. A genuine same-domain
                    // duplicate is coincident (same region → same interior
                    // sample); distinct glued patches have far-apart interiors.
                    // Skip the union when their interior samples disagree.
                    if !planar(surf_i) && distinct_curved_regions(sub_faces, i, j, tol) {
                        continue;
                    }
                    // Two faces of ONE input solid that share their whole
                    // boundary and walk it in OPPOSITE senses are ADJACENT
                    // patches glued along it, not coincident duplicates.
                    // This is the manifold gluing condition itself — the two
                    // faces meeting at an edge use it once in each direction —
                    // so it holds no matter how the patches are trimmed, and
                    // it needs no length constant beyond the vertex-weld
                    // quantization already used for the edge set.
                    //
                    // `make_sphere`'s two hemispheres are exactly this case:
                    // one spherical surface, one shared equatorial loop,
                    // opposite traversal. Without this test they hash to the
                    // same edge set, pass `surfaces_same_domain` (identical
                    // spheres), and one is dropped as within-rank residue.
                    //
                    // Restricted to same-rank pairs: two coincident faces from
                    // OPPOSING solids (a plug's wall and the bore it fills)
                    // legitimately walk their shared boundary in opposite
                    // senses because their outward normals oppose, and those
                    // are genuine same-domain pairs.
                    if sub_faces[i].rank == sub_faces[j].rank
                        && let (Some(di), Some(dj)) = (directed[i].as_ref(), directed[j].as_ref())
                        && opposite_boundary_traversal(di, dj)
                    {
                        continue;
                    }
                    uf.union(i, j);
                    let key = (i.min(j), i.max(j));
                    pair_data.insert(key, same_dir ^ (reversed[i] != reversed[j]));
                }
            }
        }
    }

    // Step 3b (issue #696): geometric containment pass for planar faces.
    // Edge-set hashing alone misses the common boolean-residue pattern where
    // one face is fully contained inside another with a different boundary
    // (e.g., a stale nub-bottom face filling the hole in a slab-top face).
    // For planar faces with the same surface, test whether one's
    // pre-computed interior point lies inside the other's wire — if so, the
    // contained face is a duplicate. Limited to planar faces because the
    // analytic surfaces (cylinder/sphere/etc) produce well-defined trimmed
    // patches that rarely accumulate residue, and a 2D containment test on
    // their parametric domains needs surface-specific handling.
    {
        // Per planar face, from one outer-wire sampling: an AABB (drives the
        // grid broad-phase) and a tol-expanded OBB (tighter oriented reject in
        // the pair loop below). `planar_obbs` is indexed directly by sub-face
        // index — dense and hasher-free in the hot loop; an entry is `Some`
        // exactly for the faces that also entered `planar_aabbs`.
        let mut planar_aabbs: Vec<(usize, brepkit_math::aabb::Aabb3)> = Vec::new();
        let mut planar_obbs: Vec<Option<brepkit_math::obb::Obb3>> = vec![None; n];
        for (idx, surf) in surfaces.iter().enumerate() {
            let Some(FaceSurface::Plane { normal, .. }) = surf else {
                continue;
            };
            let pts = face_outer_wire_points(topo, sub_faces[idx].face_id);
            if pts.len() < 3 {
                continue;
            }
            // Thickness axis pinned to the plane normal, in-plane axes from PCA,
            // expanded by tol so a boundary-coincident pair still passes.
            let mut obb = brepkit_math::obb::Obb3::from_slice_with_normal(&pts, *normal);
            for e in &mut obb.half_extents {
                *e += tol.linear;
            }
            let Some(aabb) = brepkit_math::aabb::Aabb3::try_from_points(pts) else {
                continue;
            };
            planar_aabbs.push((idx, aabb));
            planar_obbs[idx] = Some(obb);
        }
        // Broad-phase: only test pairs whose AABBs overlap. Two planar faces
        // that don't share 3D space (expanded by tol for boundary-coincident
        // outlines) cannot pass `planar_faces_overlap`, so pruning them is
        // result-preserving while collapsing the former O(n²) scan to
        // O(near). Candidate pairs come back in ascending (i, j) order — the
        // same order the nested loop visited — so the union-find sequence (and
        // hence representative selection) is unchanged.
        for_each_overlap_candidate_pair(&planar_aabbs, tol.linear, |i, j| {
            // Cheap surface-match guard first.
            let same_dir = match (surfaces[i], surfaces[j]) {
                (Some(si), Some(sj)) => surfaces_same_domain(si, sj, tol),
                _ => None,
            };
            let Some(same_dir) = same_dir else { return };
            // Oriented-bound reject: an OBB conservatively contains its face
            // (both are tol-expanded), so OBB-disjoint means the faces cannot
            // share any point — no interior point of one can lie in the other,
            // so `planar_faces_overlap` would return false. Skipping the pair
            // is result-preserving and prunes the coplanar-but-disjoint lattice
            // pairs the axis-aligned box admits — a thin diagonal strut's AABB
            // is a large square overlapping every lattice member it crosses,
            // while its OBB hugs the strut. Both `i` and `j` come from
            // `planar_aabbs`, so their OBB entries are always `Some`.
            if let (Some(oi), Some(oj)) = (&planar_obbs[i], &planar_obbs[j])
                && !oi.intersects(oj)
            {
                return;
            }
            // Complementary partition regions of one split (same source, distinct
            // interiors) are not overlapping duplicates; a coincident same-source
            // duplicate (same interior) still reaches `planar_faces_overlap`.
            if same_source_complementary_split(sub_faces, i, j, tol) {
                return;
            }
            if uf.find(i) == uf.find(j) {
                return; // already grouped
            }
            if planar_faces_overlap(topo, sub_faces, i, j, tol) {
                uf.union(i, j);
                let key = (i.min(j), i.max(j));
                pair_data.insert(key, same_dir ^ (reversed[i] != reversed[j]));
                // Mark the post-union root so the emission code knows
                // this group came from geometric containment, not from
                // boundary-identical edge sets.
                geometric_overlap_groups.insert(uf.find(i));
            }
        });
    }

    // Step 3c: geometric-overlap pass for coaxial cylinder/cone faces.
    // Edge-set hashing pairs faces only when their boundaries coincide
    // exactly. Two operands can carry the SAME coincident curved wall with
    // MISMATCHED segmentation — e.g. a body whose rounded corner arrives split
    // into two angular eighth-cylinders against a lip whose corner is one
    // quarter-cylinder (gridfinity 3×3 stacking-lip fuse). The eighths and the
    // quarter share an identical infinite cylinder over an overlapping band but
    // no edge, so Step 1 misses them and the redundant interior pieces survive,
    // leaving the shell open. Test overlap in the surface's (arc-length, axial)
    // parameter space; the BOP selector then keeps the larger patch for Fuse /
    // the smaller for Intersect, exactly as for the planar geometric-overlap
    // pairs above.
    {
        let mut analytic_aabbs: Vec<(usize, brepkit_math::aabb::Aabb3)> = Vec::new();
        for (idx, surf) in surfaces.iter().enumerate() {
            if matches!(surf, Some(FaceSurface::Cylinder(_) | FaceSurface::Cone(_)))
                && let Some(bb) = face_outer_aabb(topo, sub_faces[idx].face_id)
            {
                analytic_aabbs.push((idx, bb));
            }
        }
        // Broad-phase: as with the planar pass, only test AABB-overlapping
        // candidate pairs. Two coaxial patches whose 3D AABBs are disjoint
        // (expanded by tol) cannot share a band, so pruning them preserves the
        // result. Candidate order is ascending (i, j), matching the former
        // nested loop, so the union-find sequence is unchanged.
        for_each_overlap_candidate_pair(&analytic_aabbs, tol.linear, |i, j| {
            let same_dir = match (surfaces[i], surfaces[j]) {
                (Some(si), Some(sj)) => surfaces_same_domain(si, sj, tol),
                _ => None,
            };
            let Some(same_dir) = same_dir else { return };
            // Complementary partition regions of one split (same source, distinct
            // interiors) are not overlapping duplicates; a coincident same-source
            // duplicate (same interior) still reaches `analytic_faces_overlap`.
            if same_source_complementary_split(sub_faces, i, j, tol) {
                return;
            }
            if uf.find(i) == uf.find(j) {
                return; // already grouped (e.g. identical edge sets)
            }
            if analytic_faces_overlap(topo, sub_faces, i, j, tol) {
                uf.union(i, j);
                let key = (i.min(j), i.max(j));
                pair_data.insert(key, same_dir ^ (reversed[i] != reversed[j]));
                geometric_overlap_groups.insert(uf.find(i));
            }
        });
    }

    // Collect all roots that participate in pairs (O(m) not O(n*m)).
    let mut active_roots: HashSet<usize> = HashSet::new();
    for &(a, b) in pair_data.keys() {
        active_roots.insert(uf.find(a));
        active_roots.insert(uf.find(b));
    }

    // Each group picks A's face with smallest index as representative.
    let mut sd_groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for idx in 0..n {
        let root = uf.find(idx);
        if active_roots.contains(&root) {
            sd_groups.entry(root).or_default().push(idx);
        }
    }

    SdGrouping {
        sd_groups,
        pair_data,
        geometric_overlap_groups,
    }
}

/// Two-operand same-domain detection: split each coincident group into a
/// cross-rank [`SameDomainPair`] plus within-rank duplicates for the BOP
/// selector. Behaviour is unchanged from before the grouping was extracted.
#[cfg(test)]
pub fn detect_same_domain<S: BuildHasher>(
    topo: &Topology,
    arena: &GfaArena,
    sub_faces: &[SubFace],
    face_ranks: &HashMap<FaceId, Rank, S>,
    tol: Tolerance,
) -> SameDomainResult {
    detect_same_domain_with_shells(topo, arena, sub_faces, face_ranks, tol, None)
}

/// [`detect_same_domain`] with the operands' face-to-shell membership.
///
/// `face_shells` maps each INPUT face to an ordinal unique per shell across
/// both operand solids. When present, a within-rank coincident pair whose
/// source faces live in DIFFERENT shells of their operand is NOT residue: a
/// valid solid can carry an internal void whose ceiling is coplanar with and
/// contained in the exterior top (a zero-thickness roof — the slotted no-lip
/// bin), and dropping the void face as a "duplicate" orphans the void walls
/// into an open hole shell. Sequential-boolean residue (#696, the honeycomb's
/// stacked caps) accumulates within one shell and still dedups.
pub fn detect_same_domain_with_shells<S: BuildHasher>(
    topo: &Topology,
    arena: &GfaArena,
    sub_faces: &[SubFace],
    _face_ranks: &HashMap<FaceId, Rank, S>,
    tol: Tolerance,
    face_shells: Option<&HashMap<FaceId, usize>>,
) -> SameDomainResult {
    let SdGrouping {
        sd_groups,
        pair_data,
        geometric_overlap_groups,
    } = build_sd_grouping(topo, arena, sub_faces, tol);

    let mut pairs = Vec::new();
    let mut within_rank_dups = Vec::new();

    // Within-rank "residue" (#696) accumulates within ONE shell (stacked caps
    // left by sequential booleans, possibly split differently across cuts —
    // orientation and extent both measure identically to the legitimate case,
    // so neither discriminates). A coincident same-rank pair whose SOURCE
    // faces live in DIFFERENT shells of their operand is structural, not
    // residue: the slotted no-lip bin's internal void has a ceiling coplanar
    // with and contained in the exterior top (a zero-thickness roof), and
    // dropping it as a duplicate orphans the void walls into an open hole
    // shell. Without shell information (`None`, the test-only path) every
    // pair keeps the historic dedup.
    let is_residue = |i: usize, j: usize| -> bool {
        // Two pieces of ONE source face's partition TILE that face — they are
        // complementary regions, never copies of one another, so one is never
        // residue for the other however the grouping reached them. The pairwise
        // passes above already refuse to union such a pair directly, but a
        // third face coincident with both pulls them into one group anyway: a
        // box floor's wedge is coextensive with the cap wedge it meets AND
        // carries the same vertex-pair edge set as the cap's crescent, which is
        // bounded by that same chord walked the other way round the circle.
        // Dropping the crescent as a duplicate leaves the fused shell open
        // exactly where the cylinder wall protrudes past the box.
        if same_source_complementary_split(sub_faces, i, j, tol) {
            return false;
        }
        let cross_shell = face_shells.is_some_and(|fs| {
            match (
                fs.get(&sub_faces[i].source_face),
                fs.get(&sub_faces[j].source_face),
            ) {
                (Some(si), Some(sj)) => si != sj,
                _ => false,
            }
        });
        if std::env::var("BK_SD").is_ok() {
            log::debug!(
                "SD residue-gate i={i} face={:?} src={:?} j={j} face={:?} src={:?} cross_shell={cross_shell}",
                sub_faces[i].face_id,
                sub_faces[i].source_face,
                sub_faces[j].face_id,
                sub_faces[j].source_face
            );
        }
        !cross_shell
    };

    for (root, members) in &sd_groups {
        if members.len() < 2 {
            continue;
        }

        let repr_a = members
            .iter()
            .filter(|&&idx| sub_faces[idx].rank == Rank::A)
            .min()
            .copied();
        let repr_b = members
            .iter()
            .filter(|&&idx| sub_faces[idx].rank == Rank::B)
            .min()
            .copied();

        // True if any pair in this group was unioned by the geometric
        // containment pass. Cross-rank groups flagged here have actual
        // interior overlap (one face contained in / partially over another),
        // not just a shared boundary, so the two faces differ in extent. The
        // BOP selector uses this to keep the larger face for Fuse and the
        // smaller for Intersect (see `representative` below).
        let geometric_overlap = geometric_overlap_groups.contains(root);

        match (repr_a, repr_b) {
            // Cross-rank: classic SD pair — emit for operation-specific selection.
            (Some(idx_a), Some(idx_b)) => {
                let key = (idx_a.min(idx_b), idx_a.max(idx_b));
                let same_orientation = pair_data.get(&key).copied().unwrap_or(true);

                // Record the LARGER face (by projected area) as the
                // representative, so the choice is geometry-based not rank-based.
                // Coextensive (edge-set) pairs share the same domain (area ties),
                // so A is a fine representative and matches historical behaviour.
                // A geometric-overlap pair has two faces of different extent;
                // tagging the larger lets the BOP selector keep it for Fuse and
                // the smaller for Intersect. Which face is A flips with operand
                // order, so deferring to area keeps the result order-independent.
                let representative = if geometric_overlap {
                    let area_a = repr_face_area(topo, sub_faces[idx_a].face_id);
                    let area_b = repr_face_area(topo, sub_faces[idx_b].face_id);
                    match (area_a, area_b) {
                        (Some(aa), Some(ab)) if ab > aa => idx_b,
                        _ => idx_a,
                    }
                } else {
                    idx_a
                };

                pairs.push(SameDomainPair {
                    idx_a,
                    idx_b,
                    same_orientation,
                    geometric_overlap,
                    representative,
                });

                // The group may also contain additional same-rank members
                // (rare — a 3+ member group spanning both ranks). Treat those
                // as within-rank duplicates against the matching-rank repr.
                for &idx in members {
                    if idx == idx_a || idx == idx_b {
                        continue;
                    }
                    let rep = if sub_faces[idx].rank == Rank::A {
                        idx_a
                    } else {
                        idx_b
                    };
                    if !is_residue(idx, rep) {
                        continue;
                    }
                    within_rank_dups.push(WithinRankDuplicate {
                        representative: rep,
                        duplicate: idx,
                    });
                }
            }
            // Within-rank only (A-only or B-only): cumulative boolean residue.
            // Keep the lowest-indexed face as representative; mark the rest
            // as duplicates so the BOP selector can drop them before
            // classification (issue #696).
            (Some(initial_rep), None) | (None, Some(initial_rep)) => {
                let rep = if geometric_overlap {
                    members
                        .iter()
                        .copied()
                        .max_by(|&a, &b| {
                            let aa = repr_face_area(topo, sub_faces[a].face_id).unwrap_or(0.0);
                            let ab = repr_face_area(topo, sub_faces[b].face_id).unwrap_or(0.0);
                            aa.partial_cmp(&ab).unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .unwrap_or(initial_rep)
                } else {
                    initial_rep
                };
                for &idx in members {
                    if idx != rep && is_residue(idx, rep) {
                        within_rank_dups.push(WithinRankDuplicate {
                            representative: rep,
                            duplicate: idx,
                        });
                    }
                }
            }
            (None, None) => {}
        }
    }

    // Sort outputs deterministically — `sd_groups.values()` iterates a
    // HashMap, so without sorting the pair order varies per run and
    // propagates into face ordering in the result shell (drove 100–500×
    // perf variance in `bench_boolean_64_holes`).
    pairs.sort_unstable_by_key(|p| (p.idx_a, p.idx_b));
    within_rank_dups.sort_unstable_by_key(|d| (d.representative, d.duplicate));

    log::debug!(
        "detect_same_domain: {} cross-rank pairs, {} within-rank duplicates (edge-set hash)",
        pairs.len(),
        within_rank_dups.len()
    );
    if std::env::var("BK_SD").is_ok() {
        for p in &pairs {
            log::debug!(
                "SD pair idx_a={} face_a={:?} src_a={:?} idx_b={} face_b={:?} src_b={:?} geo={} rep={}",
                p.idx_a,
                sub_faces[p.idx_a].face_id,
                sub_faces[p.idx_a].source_face,
                p.idx_b,
                sub_faces[p.idx_b].face_id,
                sub_faces[p.idx_b].source_face,
                p.geometric_overlap,
                p.representative
            );
        }
        for d in &within_rank_dups {
            log::debug!(
                "SD within-rank rep={} face={:?} src={:?} dup={} face={:?} src={:?}",
                d.representative,
                sub_faces[d.representative].face_id,
                sub_faces[d.representative].source_face,
                d.duplicate,
                sub_faces[d.duplicate].face_id,
                sub_faces[d.duplicate].source_face
            );
        }
    }

    SameDomainResult {
        pairs,
        within_rank_dups,
    }
}

/// N-way FUSE same-domain decisions over coincident face groups.
pub struct FuseNSameDomain {
    /// Every sub-face that belongs to a coincident group. These are handled
    /// here and excluded from the caller's normal inside/outside classification.
    pub grouped: HashSet<usize>,
    /// For each SAME-oriented (shared-exterior) group, its single kept
    /// representative paired with the set of source indices in that group. The
    /// caller keeps the representative only if it is also outside every source
    /// NOT in the group (a third solid could still cover it). Opposite-oriented
    /// (internal-interface) groups contribute members to `grouped` only — all
    /// dropped.
    pub keep_reprs: Vec<(usize, HashSet<usize>)>,
}

/// Resolve coincident faces for an N-way FUSE.
///
/// Reuses the rank-agnostic [`build_sd_grouping`] and decides each group by the
/// effective outward normals of its members (planar only):
///
/// - **All aligned** — the faces bound the union on the same side; it is an
///   exterior boundary, so keep exactly one (the lowest-indexed member) and drop
///   the coincident duplicates.
/// - **Mixed** — material lies on both sides of the shared plane, so the region
///   is interior to the union; drop every member.
///
/// `source[i]` is the global source index of sub-face `i`.
///
/// # Errors
///
/// Returns [`AlgoError`] on a coincident group with a non-planar member, whose
/// orientation is not a single vector — the caller should fall back to the
/// sequential path.
pub fn detect_same_domain_fuse_n(
    topo: &Topology,
    arena: &GfaArena,
    sub_faces: &[SubFace],
    source: &[usize],
    tol: Tolerance,
) -> Result<FuseNSameDomain, AlgoError> {
    let SdGrouping { sd_groups, .. } = build_sd_grouping(topo, arena, sub_faces, tol);

    let mut grouped = HashSet::new();
    let mut keep_reprs = Vec::new();

    for members in sd_groups.values() {
        if members.len() < 2 {
            continue;
        }

        let mut normals = Vec::with_capacity(members.len());
        for &m in members {
            let normal = topo
                .face(sub_faces[m].face_id)?
                .effective_plane_normal()
                .ok_or_else(|| {
                    AlgoError::AssemblyFailed(
                        "N-way fuse: non-planar coincident face; sequential fallback".into(),
                    )
                })?;
            normals.push(normal);
        }

        for &m in members {
            grouped.insert(m);
        }

        let reference = normals[0];
        let all_aligned = normals.iter().all(|n| n.dot(reference) > 0.0);
        if all_aligned {
            // Shared exterior boundary — keep exactly one representative.
            let repr = members.iter().copied().min().unwrap_or(members[0]);
            let group_sources: HashSet<usize> = members.iter().map(|&m| source[m]).collect();
            keep_reprs.push((repr, group_sources));
        }
        // Mixed orientation → interior interface → drop all (members already in
        // `grouped`, none pushed to `keep_reprs`).
    }

    Ok(FuseNSameDomain {
        grouped,
        keep_reprs,
    })
}

/// Compute the canonical edge set for a face using quantized vertex positions.
///
/// Each edge in the outer wire is represented as a sorted pair of quantized
/// 3D positions. The pairs are sorted for deterministic comparison.
/// Using quantized positions instead of `VertexId` ensures that vertices
/// from different solids that share the same position (merged by VV phase)
/// produce matching edge sets.
///
/// Only the outer wire is considered. Inner wires (holes) are intentionally
/// excluded: SD faces in boolean operations share the same outer boundary
/// but may differ in holes (which are handled by the BOP selector).
fn compute_edge_set_quantized(
    topo: &Topology,
    arena: &GfaArena,
    face_id: FaceId,
    scale: f64,
) -> Option<EdgeSet> {
    use brepkit_topology::vertex::VertexId;

    let face = topo.face(face_id).ok()?;
    let wire = topo.wire(face.outer_wire()).ok()?;

    let mut pairs: Vec<(QVert, QVert, QVert)> = Vec::with_capacity(wire.edges().len());

    // Cache resolved vertex positions to avoid redundant resolve_vertex() calls
    // when the same vertex appears in multiple edges.
    let mut vertex_cache: HashMap<VertexId, QVert> = HashMap::new();
    let mut resolve_and_quantize = |vid: VertexId| -> Option<QVert> {
        if let Some(&cached) = vertex_cache.get(&vid) {
            return Some(cached);
        }
        let resolved = arena.resolve_vertex(vid);
        let pos = topo.vertex(resolved).ok()?.point();
        let q = quantize_point(pos, scale);
        vertex_cache.insert(vid, q);
        Some(q)
    };

    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;

        let qs = resolve_and_quantize(edge.start())?;
        let qe = resolve_and_quantize(edge.end())?;

        let sp = topo
            .vertex(arena.resolve_vertex(edge.start()))
            .ok()?
            .point();
        let ep = topo.vertex(arena.resolve_vertex(edge.end())).ok()?.point();
        // Third slot: a discriminator that separates two different curves
        // sharing the same endpoint pair. OPEN and CLOSED edges need
        // different ones, because the closed case breaks the open case's
        // premise.
        //
        // Open edge — the midpoint in STORED order. Arcs follow the CCW
        // start-to-end convention, so (A,B) and (B,A) are complementary
        // arcs: different geometry that must hash apart (they are exactly
        // the two halves this discriminator separates). Identical geometry
        // always stores identical direction under that convention, so a
        // true duplicate pair cannot hash apart.
        //
        // Closed edge (start == end) — that premise does NOT hold. A full
        // circle has no complementary arc, and two instances of the SAME
        // circle can carry different parameterizations (a revolve seam and
        // a coincident wall rim start at different angles), so their stored
        // midpoints land a quarter-turn apart and a true duplicate hashes
        // apart. That is the coincident-face pair the fuse then fails to
        // recognize. Use the centroid of samples taken uniformly over the
        // whole period instead: for a circle the equally-spaced offsets
        // cancel exactly, so it is the centre regardless of where the
        // parameterization starts or which way it runs. Combined with the
        // shared endpoint (which fixes the radius) it discriminates as well
        // as the midpoint did, and it is canonical.
        //
        // Circle AND Ellipse qualify: both evaluate as
        // `centre + a·cos(t)·u + b·sin(t)·v` over the full `[0, 2π]` domain,
        // so uniform `t` is uniform angle and the sampled offsets cancel for
        // any stored frame. Two coincident ellipses may store opposed major
        // axes (or opposed normals), which moves the t=0.5 midpoint by `2a`
        // while leaving the centroid on the centre.
        //
        // A closed NURBS edge is deliberately excluded. Uniform `t` walks the
        // knot span, so two instances of one curve carrying different knots or
        // a different seam sample different points and the centroid is not
        // invariant. Resampling by arc length would remove the knot dependence
        // but not the seam dependence — only the arc-length-weighted integral
        // `∮p ds / ∮ds` is seam-independent, and a finite-sample estimate of it
        // has no error bound below this key's quantization bucket, so it could
        // still split true duplicates. A closed NURBS therefore keeps the
        // stored-order midpoint: stable per instance, not canonical across
        // instances.
        let closed_uniform_angle =
            qs == qe && matches!(edge.curve(), EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_));
        let disc = if closed_uniform_angle {
            closed_edge_centroid(edge.curve(), sp, ep)
        } else {
            crate::builder::pcurve_compute::evaluate_edge_at_t(edge.curve(), sp, ep, 0.5)
        };
        // quantize_point MULTIPLIES by the scale, so the 100x-coarser
        // discriminator bucket (fit-error tolerance for marched geometry)
        // needs scale / 100, not scale * 100.
        let qmid = quantize_point(disc, scale / 100.0);

        // Canonical ordering: smaller first
        let pair = if qs <= qe {
            (qs, qe, qmid)
        } else {
            (qe, qs, qmid)
        };
        pairs.push(pair);
    }

    pairs.sort_unstable();
    Some(pairs)
}

/// A face's outer wire walked in traversal order: quantized `(from, to)`
/// vertex pairs, one per oriented edge, in wire order.
///
/// Complements [`EdgeSet`], which is deliberately direction-agnostic (it
/// canonicalises each pair so a shared edge matches from either side).
type DirectedBoundary = Vec<(QVert, QVert)>;

/// Walk a face's outer wire in traversal order, quantizing each oriented
/// edge's `(from, to)` vertex pair.
///
/// Uses the same weld quantization and the same `arena.resolve_vertex`
/// indirection as [`compute_edge_set_quantized`], so a boundary shared by two
/// faces yields keys that compare exactly.
fn compute_directed_boundary_quantized(
    topo: &Topology,
    arena: &GfaArena,
    face_id: FaceId,
    scale: f64,
) -> Option<DirectedBoundary> {
    let face = topo.face(face_id).ok()?;
    let wire = topo.wire(face.outer_wire()).ok()?;

    let mut walk = Vec::with_capacity(wire.edges().len());
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let from = topo
            .vertex(arena.resolve_vertex(oe.oriented_start(edge)))
            .ok()?
            .point();
        let to = topo
            .vertex(arena.resolve_vertex(oe.oriented_end(edge)))
            .ok()?
            .point();
        walk.push((quantize_point(from, scale), quantize_point(to, scale)));
    }
    Some(walk)
}

/// Whether two faces walk the same boundary in OPPOSITE senses.
///
/// True when every directed edge of `a` appears reversed in `b` and none
/// appears forward in `b`. That is the manifold gluing relation: two faces
/// adjacent along an edge traverse it once in each direction, whereas two
/// faces covering the SAME region traverse their common boundary the same way
/// (when their normals agree) — so an opposite walk within one solid means the
/// faces are neighbours, not duplicates.
///
/// Deliberately conservative:
/// - Unequal lengths return `false` (the boundaries are not the same walk).
/// - A boundary containing a CLOSED edge (`from == to`, e.g. a full-circle
///   seam) reads as both forward and reversed, so the `no forward key` clause
///   rejects it and the caller keeps its existing behaviour.
///
/// Uses only the caller's quantized vertex keys — no length constant of its
/// own, so the verdict is identical at every model scale.
fn opposite_boundary_traversal(a: &DirectedBoundary, b: &DirectedBoundary) -> bool {
    if a.is_empty() || a.len() != b.len() {
        return false;
    }
    let forward: HashSet<(QVert, QVert)> = b.iter().copied().collect();
    let reversed: HashSet<(QVert, QVert)> = b.iter().map(|&(s, e)| (e, s)).collect();
    a.iter().all(|k| reversed.contains(k)) && !a.iter().any(|k| forward.contains(k))
}

/// Number of samples used for the closed-edge centroid discriminator.
///
/// Any `N >= 2` makes the equally-spaced angular offsets of a circle or an
/// ellipse cancel exactly; 16 keeps the residue well inside the discriminator
/// bucket even for large radii.
const CLOSED_EDGE_SAMPLES: usize = 16;

/// Parameterization-independent point identifying a CLOSED edge.
///
/// Samples uniformly over the whole period and averages. The endpoint is
/// deliberately excluded (it duplicates the start on a closed curve and
/// would bias the average toward it). For a circle or an ellipse — both
/// parameterized by a uniform angle over `[0, 2π]` — the sampled offsets sum
/// to zero whatever the stored frame or direction, so this returns the exact
/// centre, which is what makes two differently-parameterized instances of one
/// curve hash together. Callers must not hand it a closed NURBS: uniform `t`
/// walks the knot span there and the average is not invariant.
fn closed_edge_centroid(
    curve: &brepkit_topology::edge::EdgeCurve,
    sp: brepkit_math::vec::Point3,
    ep: brepkit_math::vec::Point3,
) -> brepkit_math::vec::Point3 {
    let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
    for k in 0..CLOSED_EDGE_SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let t = k as f64 / CLOSED_EDGE_SAMPLES as f64;
        let p = crate::builder::pcurve_compute::evaluate_edge_at_t(curve, sp, ep, t);
        sx += p.x();
        sy += p.y();
        sz += p.z();
    }
    #[allow(clippy::cast_precision_loss)]
    let n = CLOSED_EDGE_SAMPLES as f64;
    brepkit_math::vec::Point3::new(sx / n, sy / n, sz / n)
}

/// Test whether two planar sub-faces are geometrically coincident or one
/// is fully contained inside the other.
///
/// Returns `true` only when ALL outer-wire vertices of one face lie inside
/// or on the boundary of the other face's outer polygon (and the interior
/// sample point confirms it). A weaker "interior-only" containment test was
/// tried and rejected: adjacent coplanar faces with concave geometry could
/// have an interior point that happens to land inside a neighbor's polygon
/// without the faces actually overlapping. Requiring whole-wire containment
/// is the conservative criterion that catches boolean residue (issue #696)
/// — typically a small "filling" face inside a larger face's outer
/// boundary — without firing on legitimate adjacent face pairs.
fn planar_faces_overlap(
    topo: &Topology,
    sub_faces: &[SubFace],
    i: usize,
    j: usize,
    tol: Tolerance,
) -> bool {
    let Ok(face_i) = topo.face(sub_faces[i].face_id) else {
        return false;
    };
    let Ok(face_j) = topo.face(sub_faces[j].face_id) else {
        return false;
    };
    let FaceSurface::Plane {
        normal: normal_i, ..
    } = *face_i.surface()
    else {
        return false;
    };

    // Sample each edge into several points along its curve, not just the
    // start vertex. A closed wire built from a single circular edge (a
    // circular hole left by an earlier cut) has one start vertex, so a
    // vertex-only polygon collapses to a single point and the hole
    // containment test silently treats the hole as absent — letting a
    // coincident coplanar face be wrongly cancelled through the hole.
    let wire_points = |wire_id: brepkit_topology::wire::WireId| -> Vec<brepkit_math::vec::Point3> {
        let samples_per_edge: usize = SD_EDGE_SAMPLES;
        let mut pts = Vec::new();
        let Ok(wire) = topo.wire(wire_id) else {
            return pts;
        };
        for oe in wire.edges() {
            let Ok(edge) = topo.edge(oe.edge()) else {
                continue;
            };
            let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end())) else {
                continue;
            };
            let (sp, ep) = (sv.point(), ev.point());
            // Sample via the shorter-arc evaluator: split faces can store
            // arc edges whose vertex order opposes the circle's CCW
            // parameterization, and domain-based sampling would then trace
            // the complementary (long-way) arc, corrupting the polygon used
            // for the containment tests below.
            super::pcurve_compute::sample_edge_uniform(
                edge.curve(),
                sp,
                ep,
                samples_per_edge,
                oe.is_forward(),
                &mut pts,
            );
        }
        pts
    };

    let pts_i = wire_points(face_i.outer_wire());
    let pts_j = wire_points(face_j.outer_wire());
    if pts_i.len() < 3 || pts_j.len() < 3 {
        return false;
    }
    let frame = super::plane_frame::PlaneFrame::from_plane_face(normal_i, &pts_i);
    let poly_i: Vec<_> = pts_i.iter().map(|&p| frame.project(p)).collect();
    let poly_j: Vec<_> = pts_j.iter().map(|&p| frame.project(p)).collect();

    // Passthrough faces arrive without a pre-computed interior point;
    // derive one from the projected outer polygon so coincident-outline
    // pairs (split disc vs. unsplit opposing cap) are still testable.
    let p_i_2d = sub_faces[i].interior_point.map_or_else(
        || super::classify_2d::sample_interior_point(&poly_i),
        |p| frame.project(p),
    );
    let p_j_2d = sub_faces[j].interior_point.map_or_else(
        || super::classify_2d::sample_interior_point(&poly_j),
        |p| frame.project(p),
    );

    // Strict containment: every vertex of `verts` lies inside `poly` by the
    // ray-cast test, no boundary tolerance.
    let all_inside_strict =
        |verts: &[brepkit_math::vec::Point2], poly: &[brepkit_math::vec::Point2]| -> bool {
            verts
                .iter()
                .all(|&v| super::classify_2d::point_in_polygon_2d(v, poly))
        };

    // Boundary-tolerant containment: a coincident-outline pair (e.g. a
    // section-loop disc vs. the opposing solid's cap with differently split
    // boundary edges) has every vertex exactly ON the container's polygon,
    // where the strict ray-cast is unpredictable.
    let all_inside_tol =
        |verts: &[brepkit_math::vec::Point2], poly: &[brepkit_math::vec::Point2]| -> bool {
            let boundary_eps = super::classify_2d::boundary_eps(poly);
            verts.iter().all(|&v| {
                super::classify_2d::point_in_polygon_2d(v, poly)
                    || super::classify_2d::distance_to_polygon_boundary(v, poly) <= boundary_eps
            })
        };

    // Two coplanar faces that tile disjoint side-by-side regions share a
    // boundary segment, so every vertex of one lands ON the other's polygon
    // and `all_inside_tol` reports a false containment in a single direction.
    // A genuine coincident-outline pair (the case boundary tolerance exists
    // for) instead has BOTH faces' interior points mutually inside, because
    // the outlines coincide. Require that mutual containment before trusting
    // a boundary-tolerant match; strict containment needs no such guard.
    let ip_i_in_j = super::classify_2d::point_in_polygon_2d(p_i_2d, &poly_j);
    let ip_j_in_i = super::classify_2d::point_in_polygon_2d(p_j_2d, &poly_i);
    let outlines_coincide = ip_i_in_j && ip_j_in_i;
    let all_inside =
        |verts: &[brepkit_math::vec::Point2], poly: &[brepkit_math::vec::Point2]| -> bool {
            all_inside_strict(verts, poly) || (outlines_coincide && all_inside_tol(verts, poly))
        };

    // A point landing inside one of the container's inner wires sits in a
    // hole, not on the face — e.g. a frame face whose hole exactly hosts
    // the candidate. Containment through a hole is not overlap.
    let in_hole = |p: brepkit_math::vec::Point2, face: &brepkit_topology::face::Face| -> bool {
        face.inner_wires().iter().any(|&wid| {
            let pts = wire_points(wid);
            if pts.len() < 3 {
                return false;
            }
            let poly: Vec<_> = pts.iter().map(|&q| frame.project(q)).collect();
            super::classify_2d::point_in_polygon_2d(p, &poly)
        })
    };

    // A single interior sample can miss the hole for a non-convex candidate
    // straddling a hole boundary: the sample may land on solid material while
    // the candidate's footprint actually sits entirely over the container's
    // holes. As an additional (not replacement) suppressor, also reject when
    // EVERY sampled point of the candidate that lies inside the container's
    // outer boundary falls inside one of the container's holes. This keeps
    // the common case (interior sample alone) identical and only fires extra
    // for footprints fully over holes.
    let footprint_in_holes = |sample: brepkit_math::vec::Point2,
                              verts: &[brepkit_math::vec::Point2],
                              outer: &[brepkit_math::vec::Point2],
                              face: &brepkit_topology::face::Face|
     -> bool {
        if face.inner_wires().is_empty() {
            return false;
        }
        std::iter::once(sample)
            .chain(verts.iter().copied())
            .filter(|&p| super::classify_2d::point_in_polygon_2d(p, outer))
            .all(|p| in_hole(p, face))
    };

    // A sampled polygon is an inscribed approximation of a circular boundary.
    // A point on the true circle can therefore sit outside the chord polygon.
    // Recover full containment only when the container is convex and every
    // sampled candidate point is either inside that polygon or proven on one
    // of the container's true analytic boundary edges.
    let convex = |poly: &[brepkit_math::vec::Point2]| -> bool {
        let mut sign = 0.0_f64;
        for idx in 0..poly.len() {
            let ab = poly[(idx + 1) % poly.len()] - poly[idx];
            let bc = poly[(idx + 2) % poly.len()] - poly[(idx + 1) % poly.len()];
            let cross = ab.x() * bc.y() - ab.y() * bc.x();
            if cross.abs() <= tol.linear_sq() {
                continue;
            }
            if sign * cross < 0.0 {
                return false;
            }
            sign = cross.signum();
        }
        sign != 0.0
    };
    let on_true_outer_boundary =
        |p: brepkit_math::vec::Point3, face: &brepkit_topology::face::Face| -> bool {
            topo.wire(face.outer_wire()).is_ok_and(|wire| {
                wire.edges().iter().any(|oe| {
                    topo.edge(oe.edge()).is_ok_and(|edge| {
                        let Ok(start) = topo
                            .vertex(edge.start())
                            .map(brepkit_topology::vertex::Vertex::point)
                        else {
                            return false;
                        };
                        let Ok(end) = topo
                            .vertex(edge.end())
                            .map(brepkit_topology::vertex::Vertex::point)
                        else {
                            return false;
                        };
                        super::fill_images_faces::point_on_edge(
                            edge.curve(),
                            start,
                            end,
                            p,
                            tol.linear * 10.0,
                        )
                    })
                })
            })
        };
    let safely_contained = |points: &[brepkit_math::vec::Point3],
                            projected: &[brepkit_math::vec::Point2],
                            container: &[brepkit_math::vec::Point2],
                            face: &brepkit_topology::face::Face|
     -> bool {
        convex(container)
            && points.len() == projected.len()
            && points.iter().zip(projected).all(|(&point, &uv)| {
                super::classify_2d::point_in_polygon_2d(uv, container)
                    || on_true_outer_boundary(point, face)
            })
    };
    let contained_to_tolerance =
        |candidate: &[brepkit_math::vec::Point2], container: &[brepkit_math::vec::Point2]| {
            candidate.iter().all(|&point| {
                super::classify_2d::point_in_polygon_2d(point, container)
                    || super::classify_2d::distance_to_polygon_boundary(point, container)
                        <= tol.linear * 10.0
            })
        };

    // Full containment is representable by the same-domain selection rules;
    // unlike a mere area threshold, every boundary point of the smaller face
    // is proven inside the larger face here.
    // A convex analytic-boundary proof recovers containment that the sampled
    // chord polygon cannot represent exactly.
    if ip_i_in_j
        && safely_contained(&pts_i, &poly_i, &poly_j, face_j)
        && !in_hole(p_i_2d, face_j)
        && !footprint_in_holes(p_i_2d, &poly_i, &poly_j, face_j)
    {
        return true;
    }
    if ip_j_in_i
        && safely_contained(&pts_j, &poly_j, &poly_i, face_i)
        && !in_hole(p_j_2d, face_i)
        && !footprint_in_holes(p_j_2d, &poly_j, &poly_i, face_i)
    {
        return true;
    }

    // i fully contained in j: every vertex of i (plus its interior sample)
    // is inside j's polygon.
    if ip_i_in_j
        && all_inside(&poly_i, &poly_j)
        && !in_hole(p_i_2d, face_j)
        && !footprint_in_holes(p_i_2d, &poly_i, &poly_j, face_j)
    {
        return true;
    }
    // j fully contained in i.
    if ip_j_in_i
        && all_inside(&poly_j, &poly_i)
        && !in_hole(p_j_2d, face_i)
        && !footprint_in_holes(p_j_2d, &poly_j, &poly_i, face_i)
    {
        return true;
    }

    // Plane-line splitters can leave a copied boundary vertex a few units of
    // tolerance outside the polygon used for classification. Every boundary
    // vertex still has to be inside or boundary-close; unlike an area ratio,
    // this cannot accept a face with any materially exposed edge or corner.
    if ip_i_in_j
        && contained_to_tolerance(&poly_i, &poly_j)
        && !in_hole(p_i_2d, face_j)
        && !footprint_in_holes(p_i_2d, &poly_i, &poly_j, face_j)
    {
        return true;
    }
    if ip_j_in_i
        && contained_to_tolerance(&poly_j, &poly_i)
        && !in_hole(p_j_2d, face_i)
        && !footprint_in_holes(p_j_2d, &poly_j, &poly_i, face_i)
    {
        return true;
    }

    face_i.inner_wires().is_empty()
        && face_j.inner_wires().is_empty()
        && polygons_overlap_majority(&poly_i, &poly_j, tol)
}

/// Whether two projected face polygons share a genuine 2D region.
///
/// Two coplanar (or co-surface) faces can overlap without either being fully
/// contained in the other — e.g. a faceted scoop ramp's staircase-shaped wall
/// sub-face lying against a rectangular ramp side facet. The containment tests
/// above miss these, so the coincident pair survives classification and the
/// fused result goes non-manifold (#895).
///
/// Detect it by the intersection AREA of the projected polygons. Faces that
/// merely tile side-by-side share only a boundary segment (zero intersection
/// area), so this does not reintroduce the side-by-side false positive the
/// containment guards defend against. Requiring the overlap to cover more than
/// half of the smaller face keeps a sliver of numerical overlap along a shared
/// edge from pairing disjoint faces.
fn polygons_overlap_majority(
    a: &[brepkit_math::vec::Point2],
    b: &[brepkit_math::vec::Point2],
    tol: Tolerance,
) -> bool {
    let area_a = super::classify_2d::signed_area_2d(a).abs();
    let area_b = super::classify_2d::signed_area_2d(b).abs();
    let smaller = area_a.min(area_b);
    // `smaller` and the overlap are areas, so the degenerate-face guard
    // compares against the squared linear tolerance (area), not `linear`.
    if smaller <= tol.linear_sq() {
        return false;
    }
    // The polygon intersection is contained in the overlap of the two 2D
    // bounding boxes, so `area(poly∩poly) ≤ area(bbox∩bbox)`. The exact (and
    // costly) polygon clip can only clear the 50%-of-smaller threshold below
    // when the box overlap already does — so gate the clip on the cheap box
    // test. This skips the clip for the common touching / side-by-side
    // coplanar pairs (e.g. stacked wall-piece bands) without changing the
    // result.
    if bbox2d_overlap_area(a, b) <= smaller * 0.5 {
        return false;
    }
    crate::perf::bump_sd_poly_clip();
    let intersection = brepkit_math::polygon_boolean::polygon_boolean(
        a,
        b,
        brepkit_math::polygon_boolean::BooleanOp::Intersection,
        tol.linear,
    );
    intersection.area().abs() > smaller * 0.5
}

/// Area of the overlap of two 2D point sets' axis-aligned bounding boxes.
///
/// A conservative (over-)estimate of the polygons' intersection area: the
/// intersection lies inside both boxes, so its area never exceeds this. Used to
/// skip the exact polygon clip when no meaningful overlap is possible.
fn bbox2d_overlap_area(a: &[brepkit_math::vec::Point2], b: &[brepkit_math::vec::Point2]) -> f64 {
    let bounds = |poly: &[brepkit_math::vec::Point2]| {
        let (mut lo_x, mut lo_y) = (f64::MAX, f64::MAX);
        let (mut hi_x, mut hi_y) = (f64::MIN, f64::MIN);
        for p in poly {
            lo_x = lo_x.min(p.x());
            lo_y = lo_y.min(p.y());
            hi_x = hi_x.max(p.x());
            hi_y = hi_y.max(p.y());
        }
        (lo_x, lo_y, hi_x, hi_y)
    };
    let (alx, aly, ahx, ahy) = bounds(a);
    let (blx, bly, bhx, bhy) = bounds(b);
    let ox = (ahx.min(bhx) - alx.max(blx)).max(0.0);
    let oy = (ahy.min(bhy) - aly.max(bly)).max(0.0);
    ox * oy
}

/// Approximate projected outer-wire area of a planar sub-face, in its own
/// plane.
///
/// Returns `None` for non-planar faces or faces whose outer wire samples to
/// fewer than three points. The area is an approximation: each edge is sampled
/// at [`SD_EDGE_SAMPLES`] points (so arc boundaries contribute their swept
/// area, matching [`planar_faces_overlap`]), then the projected polygon's
/// signed area is taken — a finer arc is under-counted by the chord polygon.
///
/// Used only to order the two faces of a geometric-overlap SD pair (see
/// [`SameDomainPair::representative`]). The two faces always share a plane, so
/// the same under-counting applies to both and their relative order is stable;
/// the absolute area is never compared against a tolerance.
fn planar_face_area(topo: &Topology, face_id: FaceId) -> Option<f64> {
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Plane { normal, .. } = *face.surface() else {
        return None;
    };
    let wire = topo.wire(face.outer_wire()).ok()?;
    let mut pts: Vec<brepkit_math::vec::Point3> =
        Vec::with_capacity(wire.edges().len() * SD_EDGE_SAMPLES);
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let sv = topo.vertex(edge.start()).ok()?;
        let ev = topo.vertex(edge.end()).ok()?;
        let (sp, ep) = (sv.point(), ev.point());
        // Sample each edge so arc boundaries contribute their true swept area,
        // mirroring `planar_faces_overlap`'s shorter-arc sampling.
        super::pcurve_compute::sample_edge_uniform(
            edge.curve(),
            sp,
            ep,
            SD_EDGE_SAMPLES,
            oe.is_forward(),
            &mut pts,
        );
    }
    if pts.len() < 3 {
        return None;
    }
    let frame = super::plane_frame::PlaneFrame::from_plane_face(normal, &pts);
    let poly: Vec<_> = pts.iter().map(|&p| frame.project(p)).collect();
    Some(super::classify_2d::signed_area_2d(&poly).abs())
}

/// Sample a cylinder/cone sub-face's outer wire into 3D points, [`SD_EDGE_SAMPLES`]
/// per edge.
///
/// Returns `None` for non-(cylinder/cone) faces or wires that sample to fewer
/// than three points. Mirrors [`planar_faces_overlap`]'s shorter-arc edge
/// sampling so arc boundaries contribute their true swept extent. The raw 3D
/// points (not parameters) are returned so [`analytic_faces_overlap`] can
/// project BOTH faces through a single shared reference surface — projecting
/// each face through its own surface would reference the axial coordinate to a
/// different origin and falsely align disjoint z-bands.
fn wire_points_3d(topo: &Topology, face_id: FaceId) -> Option<Vec<brepkit_math::vec::Point3>> {
    let face = topo.face(face_id).ok()?;
    if !matches!(
        face.surface(),
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
    ) {
        return None;
    }
    let wire = topo.wire(face.outer_wire()).ok()?;
    let mut pts = Vec::with_capacity(wire.edges().len() * SD_EDGE_SAMPLES);
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let sv = topo.vertex(edge.start()).ok()?;
        let ev = topo.vertex(edge.end()).ok()?;
        let (sp, ep) = (sv.point(), ev.point());
        super::pcurve_compute::sample_edge_uniform(
            edge.curve(),
            sp,
            ep,
            SD_EDGE_SAMPLES,
            oe.is_forward(),
            &mut pts,
        );
    }
    if pts.len() < 3 {
        return None;
    }
    Some(pts)
}

/// Project 3D points into the `(θ, axial)` parameter space of `surface`
/// (cylinder or cone), returning the `(θ, axial)` samples and the arc-length
/// scale radius.
///
/// `θ` is the raw angular parameter (radians, not yet seam-unwrapped); `axial`
/// is `v` along the surface's own axis from its origin/apex. Because both faces
/// of a candidate pair are projected through the SAME `surface`, the axial
/// reference is shared and the parameter spaces are directly comparable. The
/// returned `radius` scales `θ` into arc length so the 2D tests operate in mm;
/// for a cone it is the radius at the samples' mid-axial coordinate.
fn project_points_through_surface(
    surface: &FaceSurface,
    pts: &[brepkit_math::vec::Point3],
) -> Option<(Vec<(f64, f64)>, f64)> {
    let samples: Vec<(f64, f64)> = match surface {
        FaceSurface::Cylinder(c) => pts.iter().map(|&p| c.project_point(p)).collect(),
        FaceSurface::Cone(c) => pts.iter().map(|&p| c.project_point(p)).collect(),
        _ => return None,
    };
    if samples.len() < 3 {
        return None;
    }
    let radius = match surface {
        FaceSurface::Cylinder(c) => c.radius(),
        FaceSurface::Cone(c) => {
            let v_min = samples
                .iter()
                .map(|&(_, v)| v)
                .fold(f64::INFINITY, f64::min);
            let v_max = samples
                .iter()
                .map(|&(_, v)| v)
                .fold(f64::NEG_INFINITY, f64::max);
            // `radius_at` is signed: it returns a negative value when the
            // cone's axis points apex→base and the patch sits on the negative
            // side. Only the magnitude scales θ into arc length (the
            // tessellation path takes `.abs()` for the same reason).
            c.radius_at(0.5 * (v_min + v_max)).abs()
        }
        _ => return None,
    };
    // A degenerate (apex-touching) cone band has ~zero radius; the arc-length
    // scaling would collapse θ and make the 2D test meaningless.
    if radius <= 0.0 {
        return None;
    }
    Some((samples, radius))
}

/// Unwrap a sequence of raw angular samples (each in `[0, 2π)`) into a
/// continuous run by adding the multiple of `2π` to each successive sample that
/// minimizes the step from its predecessor.
///
/// A trimmed cylinder/cone patch spans less than a full turn, so its boundary
/// θ values form a continuous arc once seam-wrapping is removed.
fn unwrap_angles(samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
    use std::f64::consts::TAU;
    debug_assert!(
        !samples.is_empty(),
        "unwrap_angles requires at least one sample (callers guard len >= 3)"
    );
    let mut out = Vec::with_capacity(samples.len());
    let mut prev = samples[0].0;
    out.push(samples[0]);
    for &(u, axial) in &samples[1..] {
        // Add the integer multiple of 2π that brings `u` closest to `prev`
        // (i.e. the step into [-π, π]).
        let uu = u - ((u - prev) / TAU).round() * TAU;
        out.push((uu, axial));
        prev = uu;
    }
    out
}

/// Test whether two cylinder/cone sub-faces on the **same** coaxial surface
/// have overlapping trimmed patches in `(arc-length, axial)` parameter space.
///
/// The caller must have already confirmed the two faces share an infinite
/// surface (via [`surfaces_same_domain`]) — that guarantees a `(θ, axial)`
/// pair maps to the *same* 3D point on both, so a genuine parameter-space
/// overlap is a genuine 3D overlap, with one exception this function guards:
/// the angular seam. Each face's boundary is unwrapped into a continuous θ-arc,
/// then face `j`'s arc is shifted by the multiple of `2π` that maximizes its
/// 1D overlap with face `i`'s arc. Because `P(θ, ·) = P(θ + 2π, ·)` on the
/// surface, that shift is an identity in 3D; it only selects which periodic
/// representative to compare, so two patches on *opposite* sides (no genuine
/// overlap) yield no positive overlap under any shift and are not paired.
///
/// θ is scaled by the surface radius (the patch's mid-axial radius for a cone)
/// so both axes are in mm; the 2D containment / overlap-area tests then mirror
/// [`planar_faces_overlap`] exactly, including its area-fraction guard against
/// pairing faces that merely share a boundary segment.
fn analytic_faces_overlap(
    topo: &Topology,
    sub_faces: &[SubFace],
    i: usize,
    j: usize,
    tol: Tolerance,
) -> bool {
    use std::f64::consts::TAU;

    // Project BOTH faces through face i's surface so the axial coordinate and
    // angular origin share one reference frame. The two faces are coaxial with
    // equal radius (the caller's `surfaces_same_domain` guard), so each face's
    // 3D wire points lie on face i's surface too; projecting them through it is
    // exact. Projecting each face through its OWN surface would reference the
    // axial v to a different origin (e.g. a body cylinder at z=0 vs a lip
    // cylinder at z=13.3) and falsely overlap disjoint z-bands.
    let Ok(face_i) = topo.face(sub_faces[i].face_id) else {
        return false;
    };
    let Ok(face_j) = topo.face(sub_faces[j].face_id) else {
        return false;
    };
    let ref_surface = face_i.surface().clone();
    let Some(pts_i) = wire_points_3d(topo, sub_faces[i].face_id) else {
        return false;
    };
    let Some(pts_j) = wire_points_3d(topo, sub_faces[j].face_id) else {
        return false;
    };
    let Some((samples_i, radius_i)) = project_points_through_surface(&ref_surface, &pts_i) else {
        return false;
    };
    let Some((samples_j, _radius_j)) = project_points_through_surface(&ref_surface, &pts_j) else {
        return false;
    };

    let unwrapped_i = unwrap_angles(&samples_i);
    let unwrapped_j = unwrap_angles(&samples_j);

    let theta_span = |pts: &[(f64, f64)]| -> (f64, f64) {
        let lo = pts.iter().map(|&(u, _)| u).fold(f64::INFINITY, f64::min);
        let hi = pts
            .iter()
            .map(|&(u, _)| u)
            .fold(f64::NEG_INFINITY, f64::max);
        (lo, hi)
    };
    let (i_lo, i_hi) = theta_span(&unwrapped_i);
    let (j_lo, j_hi) = theta_span(&unwrapped_j);

    // A patch spanning (near) a full turn is a closed/seam surface, not the
    // partial corner patches this pass targets; comparing it parametrically is
    // ambiguous, so bail and let edge-set matching handle it.
    if (i_hi - i_lo) >= TAU - tol.angular || (j_hi - j_lo) >= TAU - tol.angular {
        return false;
    }

    // Shift j's θ-branch by the k·2π that maximizes 1D interval overlap with i.
    // Real trimmed patches span < 2π, so the physically-correct alignment is the
    // one with the largest overlap; when no genuine overlap exists every shift
    // gives a non-positive overlap and the 2D test below sees disjoint polygons.
    let mut best_shift = 0.0_f64;
    let mut best_overlap = f64::NEG_INFINITY;
    for k in -1..=1 {
        let shift = f64::from(k) * TAU;
        let lo = (j_lo + shift).max(i_lo);
        let hi = (j_hi + shift).min(i_hi);
        let overlap = hi - lo;
        if overlap > best_overlap {
            best_overlap = overlap;
            best_shift = shift;
        }
    }

    // Scale θ to arc length (mm) using the (shared) reference radius so the two
    // polygons are metrically consistent.
    let scale = radius_i;
    let to_2d = |&(u, axial): &(f64, f64), shift: f64| {
        brepkit_math::vec::Point2::new((u + shift) * scale, axial)
    };
    let poly_i: Vec<_> = unwrapped_i.iter().map(|s| to_2d(s, 0.0)).collect();
    let poly_j: Vec<_> = unwrapped_j.iter().map(|s| to_2d(s, best_shift)).collect();
    if poly_i.len() < 3 || poly_j.len() < 3 {
        return false;
    }

    let p_i = super::classify_2d::sample_interior_point(&poly_i);
    let p_j = super::classify_2d::sample_interior_point(&poly_j);

    let all_inside_strict =
        |verts: &[brepkit_math::vec::Point2], poly: &[brepkit_math::vec::Point2]| -> bool {
            verts
                .iter()
                .all(|&v| super::classify_2d::point_in_polygon_2d(v, poly))
        };
    let all_inside_tol =
        |verts: &[brepkit_math::vec::Point2], poly: &[brepkit_math::vec::Point2]| -> bool {
            let boundary_eps = super::classify_2d::boundary_eps(poly);
            verts.iter().all(|&v| {
                super::classify_2d::point_in_polygon_2d(v, poly)
                    || super::classify_2d::distance_to_polygon_boundary(v, poly) <= boundary_eps
            })
        };

    let ip_i_in_j = super::classify_2d::point_in_polygon_2d(p_i, &poly_j);
    let ip_j_in_i = super::classify_2d::point_in_polygon_2d(p_j, &poly_i);
    let outlines_coincide = ip_i_in_j && ip_j_in_i;
    let all_inside =
        |verts: &[brepkit_math::vec::Point2], poly: &[brepkit_math::vec::Point2]| -> bool {
            all_inside_strict(verts, poly) || (outlines_coincide && all_inside_tol(verts, poly))
        };

    // A hole-free line-and-circle cylinder/cone patch maps exactly to a
    // polygon with straight iso-parametric sides. If that polygon is convex,
    // boundary-tolerant containment of every sampled vertex is a whole-patch
    // proof, not an area-percentage guess. Imported NURBS and other curve kinds
    // remain on the conservative path below.
    let exact_iso_patch = |face: &brepkit_topology::face::Face| -> bool {
        face.inner_wires().is_empty()
            && topo.wire(face.outer_wire()).is_ok_and(|wire| {
                wire.edges().iter().all(|oe| {
                    topo.edge(oe.edge()).is_ok_and(|edge| {
                        matches!(edge.curve(), EdgeCurve::Line | EdgeCurve::Circle(_))
                    })
                })
            })
    };
    let convex = |poly: &[brepkit_math::vec::Point2]| -> bool {
        let mut sign = 0.0_f64;
        for idx in 0..poly.len() {
            let ab = poly[(idx + 1) % poly.len()] - poly[idx];
            let bc = poly[(idx + 2) % poly.len()] - poly[(idx + 1) % poly.len()];
            let cross = ab.x() * bc.y() - ab.y() * bc.x();
            if cross.abs() <= tol.linear_sq() {
                continue;
            }
            if sign * cross < 0.0 {
                return false;
            }
            sign = cross.signum();
        }
        sign != 0.0
    };
    if exact_iso_patch(face_i) && exact_iso_patch(face_j) {
        if ip_i_in_j && convex(&poly_j) && all_inside_tol(&poly_i, &poly_j) {
            return true;
        }
        if ip_j_in_i && convex(&poly_i) && all_inside_tol(&poly_j, &poly_i) {
            return true;
        }
    }

    // i contained in j, or j contained in i — the eighth-in-quarter case.
    if ip_i_in_j && all_inside(&poly_i, &poly_j) {
        return true;
    }
    if ip_j_in_i && all_inside(&poly_j, &poly_i) {
        return true;
    }

    // Partial overlap by intersection area (mirrors the planar path).
    polygons_overlap_majority(&poly_i, &poly_j, tol)
}

/// Approximate `(arc-length, axial)` parameter-space area of a cylinder/cone
/// sub-face's outer wire.
///
/// Returns `None` for non-(cylinder/cone) faces or wires that sample to fewer
/// than three points. Used only to order the two faces of a coaxial
/// geometric-overlap SD pair (see [`SameDomainPair::representative`]); both
/// faces share the surface so the same arc-length scaling applies to each and
/// their relative order is stable. The absolute area is never compared against
/// a tolerance.
fn analytic_face_param_area(topo: &Topology, face_id: FaceId) -> Option<f64> {
    let face = topo.face(face_id).ok()?;
    let surface = face.surface().clone();
    let pts = wire_points_3d(topo, face_id)?;
    let (samples, radius) = project_points_through_surface(&surface, &pts)?;
    let unwrapped = unwrap_angles(&samples);
    let poly: Vec<_> = unwrapped
        .iter()
        .map(|&(u, axial)| brepkit_math::vec::Point2::new(u * radius, axial))
        .collect();
    if poly.len() < 3 {
        return None;
    }
    Some(super::classify_2d::signed_area_2d(&poly).abs())
}

/// Outer-wire area used to pick the larger face of a geometric-overlap SD pair,
/// dispatched by surface type: planar area in the face plane, or
/// (arc-length, axial) parameter-space area for cylinder/cone faces.
///
/// Areas are only ever compared between the two faces of one pair, which share
/// a surface, so the (possibly different) projection per surface type is never
/// compared across surfaces.
fn repr_face_area(topo: &Topology, face_id: FaceId) -> Option<f64> {
    let face = topo.face(face_id).ok()?;
    match face.surface() {
        FaceSurface::Plane { .. } => planar_face_area(topo, face_id),
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) => analytic_face_param_area(topo, face_id),
        _ => None,
    }
}

/// Outer-wire AABB of a sub-face, sampled at [`SD_EDGE_SAMPLES`] points per
/// edge to match the polygons the overlap tests build. Returns `None` when the
/// wire has no usable points.
///
/// Used purely as a broad-phase reject for the geometric-overlap passes: both
/// [`planar_faces_overlap`] and [`analytic_faces_overlap`] can only return
/// `true` when the two faces share real 3D area, which requires their AABBs
/// (expanded by tolerance for boundary-coincident cases) to intersect.
fn face_outer_aabb(topo: &Topology, face_id: FaceId) -> Option<brepkit_math::aabb::Aabb3> {
    brepkit_math::aabb::Aabb3::try_from_points(face_outer_wire_points(topo, face_id))
}

/// Sample a face's outer wire at [`SD_EDGE_SAMPLES`] points per edge, matching
/// the polygons the overlap tests build. Samples `0..SD_EDGE_SAMPLES` (not
/// `..=`) so each shared vertex is covered once, by the next edge's `frac=0`.
fn face_outer_wire_points(topo: &Topology, face_id: FaceId) -> Vec<brepkit_math::vec::Point3> {
    let Ok(face) = topo.face(face_id) else {
        return Vec::new();
    };
    let Ok(wire) = topo.wire(face.outer_wire()) else {
        return Vec::new();
    };
    let mut pts: Vec<brepkit_math::vec::Point3> =
        Vec::with_capacity(wire.edges().len() * SD_EDGE_SAMPLES);
    for oe in wire.edges() {
        let Ok(edge) = topo.edge(oe.edge()) else {
            continue;
        };
        let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end())) else {
            continue;
        };
        super::pcurve_compute::sample_edge_uniform(
            edge.curve(),
            sv.point(),
            ev.point(),
            SD_EDGE_SAMPLES,
            oe.is_forward(),
            &mut pts,
        );
    }
    pts
}

/// Generate the spatially-overlapping candidate pairs among `indices` using a
/// uniform grid over the faces' (tolerance-expanded) AABBs.
///
/// Each face is inserted into every grid cell its expanded AABB touches; any
/// two faces that ever land in the same cell become a candidate pair (emitted
/// once, with `i < j` in original-index order, deduplicated). Faces whose AABBs
/// never share a cell cannot overlap, so they are never tested — turning the
/// former all-pairs O(n²) scan into O(n + candidate pairs). Candidates are
/// deduplicated and emitted one left-hand face at a time, so dense inputs do
/// not retain the full quadratic pair set. The candidates are a superset of
/// the truly-overlapping pairs; the caller still runs the exact
/// `*_faces_overlap` test on each.
fn for_each_overlap_candidate_pair(
    aabbs: &[(usize, brepkit_math::aabb::Aabb3)],
    margin: f64,
    mut visit: impl FnMut(usize, usize),
) {
    if aabbs.len() < 2 {
        return;
    }
    // Cell size: the cube root of the per-face AABB volume budget across the
    // populated region, never below the average face extent, so a face spans a
    // bounded number of cells.
    let mut union = aabbs[0].1;
    let mut ext_sum = 0.0_f64;
    for &(_, bb) in aabbs {
        union = union.union(bb);
        let e = bb.max - bb.min;
        ext_sum += e.x().abs().max(e.y().abs()).max(e.z().abs());
    }
    #[allow(clippy::cast_precision_loss)]
    let avg_ext = ext_sum / aabbs.len() as f64;
    let span = {
        let e = union.max - union.min;
        e.x().abs().max(e.y().abs()).max(e.z().abs())
    };
    #[allow(clippy::cast_precision_loss)]
    let n = aabbs.len() as f64;
    let cell = avg_ext
        .max(span / n.cbrt().max(1.0))
        .max(margin)
        .max(f64::MIN_POSITIVE);
    let inv = 1.0 / cell;
    let cell_of = |c: f64| (c * inv).floor() as i64;

    let mut buckets: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    // A face whose expanded AABB spans more cells than this would cost O(cells)
    // to grid (a broad wall patch among many tiny facets); above it, AABB-test
    // the face against all faces directly instead — mirrors PointGrid's guard.
    // Keep the number of retained bucket memberships linear in the face
    // count. A budget proportional to `aabbs.len()` would allow every face to
    // occupy O(n) cells and recreate quadratic memory growth in the grid.
    let cell_budget = 4096_i64;
    let mut ranges = Vec::with_capacity(aabbs.len());
    let mut large = Vec::new();
    for &(idx, bb) in aabbs {
        let e = bb.expanded(margin);
        let (lo, hi) = (e.min, e.max);
        let (cx0, cx1) = (cell_of(lo.x()), cell_of(hi.x()));
        let (cy0, cy1) = (cell_of(lo.y()), cell_of(hi.y()));
        let (cz0, cz1) = (cell_of(lo.z()), cell_of(hi.z()));
        let cells = cx1
            .saturating_sub(cx0)
            .saturating_add(1)
            .saturating_mul(cy1.saturating_sub(cy0).saturating_add(1))
            .saturating_mul(cz1.saturating_sub(cz0).saturating_add(1));
        if cells > cell_budget {
            large.push((idx, e));
            ranges.push((idx, e, None));
            continue;
        }
        ranges.push((idx, e, Some((cx0, cx1, cy0, cy1, cz0, cz1))));
        for cx in cx0..=cx1 {
            for cy in cy0..=cy1 {
                for cz in cz0..=cz1 {
                    buckets.entry((cx, cy, cz)).or_default().push(idx);
                }
            }
        }
    }

    // Retain candidates for only one left-hand face at a time. Dense input can
    // still require quadratic narrow-phase work, but its broad-phase memory is
    // now linear rather than an unbounded cache of every candidate pair.
    ranges.sort_unstable_by_key(|&(idx, _, _)| idx);
    for &(idx, expanded, cells) in &ranges {
        let mut candidates = HashSet::new();
        if let Some((cx0, cx1, cy0, cy1, cz0, cz1)) = cells {
            for cx in cx0..=cx1 {
                for cy in cy0..=cy1 {
                    for cz in cz0..=cz1 {
                        if let Some(bucket) = buckets.get(&(cx, cy, cz)) {
                            candidates.extend(bucket.iter().copied().filter(|&other| other > idx));
                        }
                    }
                }
            }
            for &(other, other_bb) in &large {
                if other > idx && expanded.intersects(other_bb) {
                    candidates.insert(other);
                }
            }
        } else {
            candidates.extend(aabbs.iter().filter_map(|&(other, bb)| {
                (other > idx && expanded.intersects(bb.expanded(margin))).then_some(other)
            }));
        }
        let mut candidates: Vec<_> = candidates.into_iter().collect();
        candidates.sort_unstable();
        for other in candidates {
            visit(idx, other);
        }
    }
}

/// Quantize a 3D point to integer grid coordinates.
///
/// Returns the collision-free `(i64, i64, i64)` triple directly.
fn quantize_point(p: brepkit_math::vec::Point3, scale: f64) -> QVert {
    (
        (p.x() * scale).round() as i64,
        (p.y() * scale).round() as i64,
        (p.z() * scale).round() as i64,
    )
}

/// Simple union-find (disjoint set) with path compression and union by rank.
struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    fn union(&mut self, x: usize, y: usize) {
        let rx = self.find(x);
        let ry = self.find(y);
        if rx == ry {
            return;
        }
        match self.rank[rx].cmp(&self.rank[ry]) {
            std::cmp::Ordering::Less => self.parent[rx] = ry,
            std::cmp::Ordering::Greater => self.parent[ry] = rx,
            std::cmp::Ordering::Equal => {
                self.parent[ry] = rx;
                self.rank[rx] += 1;
            }
        }
    }
}

/// Whether a surface is planar (the planar SD passes have their own geometric
/// containment tests, so the curved-region guard only applies to non-planes).
fn planar(surf: &FaceSurface) -> bool {
    matches!(surf, FaceSurface::Plane { .. })
}

/// Whether two curved sub-faces of the same underlying surface, paired by a
/// shared outer-wire edge set, actually cover DIFFERENT regions of that
/// surface (so they are glued neighbours, not coincident duplicates).
///
/// The discriminator is their precomputed interior sample: a genuine
/// same-domain duplicate is coincident (identical region → coincident
/// interior), whereas the two hemisphere bands of a bored sphere share the
/// equator boundary yet have interiors on opposite halves. Returns `false`
/// (defer to the edge-set union) when either interior is unavailable, keeping
/// the conservative pre-existing behaviour.
fn distinct_curved_regions(sub_faces: &[SubFace], i: usize, j: usize, tol: Tolerance) -> bool {
    match (sub_faces[i].interior_point, sub_faces[j].interior_point) {
        (Some(pi), Some(pj)) => (pi - pj).length() > tol.linear * 100.0,
        _ => false,
    }
}

/// Whether two sub-faces are COMPLEMENTARY partition regions of a single split
/// of one input face — same `source_face` AND distinct interior points (they
/// tile the parent, e.g. the in-tube and out-tube parts of a box wall cut by a
/// torus). Such a pair is never a same-domain DUPLICATE of itself, so it is
/// excluded from SD grouping. Crucially this does NOT exclude two same-source
/// sub-faces that are genuine COINCIDENT duplicates (sequential-boolean /
/// split-cascade residue at the SAME region): those share an interior point
/// (distance ≤ 100·tol) and stay subject to the normal SD overlap checks, so
/// real duplicates are still caught (Greptile #1010-2). Requires both interior
/// points to be set — if either is absent the pair is NOT excluded (the SD
/// checks run as usual).
fn same_source_complementary_split(
    sub_faces: &[SubFace],
    i: usize,
    j: usize,
    tol: Tolerance,
) -> bool {
    if sub_faces[i].source_face != sub_faces[j].source_face {
        return false;
    }
    match (sub_faces[i].interior_point, sub_faces[j].interior_point) {
        (Some(pi), Some(pj)) => (pi - pj).length() > tol.linear * 100.0,
        _ => false,
    }
}

/// Check if two surfaces represent the same geometric domain.
///
/// Returns `Some(true)` for same-direction normals (CoplanarSame),
/// `Some(false)` for opposite normals (CoplanarOpposite), or
/// `None` if not the same domain.
///
/// Visible to `crate::diagnostic` (the boolean preflight API). The
/// `redundant_pub_crate` allow is required because the enclosing
/// `builder` module is private — clippy folds `pub(crate)` to `pub`
/// in that scope, but we keep `pub(crate)` to make the intent
/// explicit in the source.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn surfaces_same_domain(
    a: &FaceSurface,
    b: &FaceSurface,
    tol: Tolerance,
) -> Option<bool> {
    match (a, b) {
        (FaceSurface::Plane { normal: na, d: da }, FaceSurface::Plane { normal: nb, d: db }) => {
            let dot = na.dot(*nb);
            if dot > 1.0 - tol.angular {
                // Same direction — check distance
                if (da - db).abs() < tol.linear {
                    return Some(true);
                }
            } else if dot < -1.0 + tol.angular {
                // Opposite direction — check distance
                if (da + db).abs() < tol.linear {
                    return Some(false);
                }
            }
            None
        }
        (FaceSurface::Cylinder(ca), FaceSurface::Cylinder(cb)) => {
            // Same cylinder: same origin, same axis, same radius
            if (ca.radius() - cb.radius()).abs() > tol.linear {
                return None;
            }
            let axis_dot = ca.axis().dot(cb.axis());
            if axis_dot.abs() < 1.0 - tol.angular {
                return None;
            }
            // Check if origins lie on the same axis line
            let diff = cb.origin() - ca.origin();
            let along_axis = diff.dot(ca.axis());
            let perp_dist = (diff - ca.axis() * along_axis).length();
            if perp_dist > tol.linear {
                return None;
            }
            Some(axis_dot > 0.0)
        }
        (FaceSurface::Sphere(sa), FaceSurface::Sphere(sb)) => {
            if (sa.radius() - sb.radius()).abs() > tol.linear {
                return None;
            }
            let dist = (sa.center() - sb.center()).length();
            if dist > tol.linear {
                return None;
            }
            Some(true)
        }
        (FaceSurface::Cone(ca), FaceSurface::Cone(cb)) => {
            if (ca.half_angle() - cb.half_angle()).abs() > tol.angular {
                return None;
            }
            let axis_dot = ca.axis().dot(cb.axis());
            if axis_dot.abs() < 1.0 - tol.angular {
                return None;
            }
            let dist = (ca.apex() - cb.apex()).length();
            if dist > tol.linear {
                return None;
            }
            Some(axis_dot > 0.0)
        }
        (FaceSurface::Torus(ta), FaceSurface::Torus(tb)) => {
            if (ta.major_radius() - tb.major_radius()).abs() > tol.linear {
                return None;
            }
            if (ta.minor_radius() - tb.minor_radius()).abs() > tol.linear {
                return None;
            }
            let axis_dot = ta.z_axis().dot(tb.z_axis());
            if axis_dot.abs() < 1.0 - tol.angular {
                return None;
            }
            let dist = (ta.center() - tb.center()).length();
            if dist > tol.linear {
                return None;
            }
            Some(axis_dot > 0.0)
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests;
