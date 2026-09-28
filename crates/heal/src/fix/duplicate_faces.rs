//! Duplicate-face repair — recognize and remove coincident duplicate face uses.
//!
//! Equivalence and removal contract: see
//! `docs/kernel-maturity/b17-h03-duplicate-face-contract.md`. In short: two
//! faces are duplicates only when their supporting carriers agree, their
//! oriented trimmed regions coincide (outer boundary plus one-to-one hole
//! correspondence), their effective orientations agree, and the pair is a
//! shell-local use of two distinct identities with compatible attributes.
//! Equal area, centroid, endpoint sets, or sampled distance never prove
//! duplication. The geometric predicate ([`faces_are_duplicates`]) is the
//! final authority — candidate buckets (PERF-H03) only select pairs to test.
//!
//! Currently supported domain: unperforated planar polygon faces (all-`Line`
//! outer boundary). Curved carriers, non-line boundaries, and perforated
//! faces are skipped (left in place, fail-closed).
//!
//! ## Candidate discovery (PERF-H03)
//!
//! [`plan_duplicate_removals`] replaces only candidate *discovery*. The exact
//! predicate, the tolerance policy, the deterministic survivor (lowest
//! `FaceId` index in each duplicate group), and the nontransitive-nearness
//! semantics (a removed face never anchors another removal) are preserved:
//! on any input the plan equals the all-pairs reference over the same
//! index-ordered descriptors, including which faces are removed — only
//! provably non-matching pairs skip the exact predicate.
//!
//! Descriptors and their conservativeness proof (a true match can never be
//! split across buckets):
//!
//! - **Edge count** (exact key component): the predicate requires equal
//!   boundary lengths, so different counts never match.
//! - **Effective-normal cell** (halo 1, cell `8e-3` per component): a true
//!   match satisfies `na·nb ≥ 1 − 1e-6`, i.e. an angle below ~`1.5e-3` rad,
//!   hence per-component `|Δ| < 8e-3`. Two values closer than one cell width
//!   differ by at most one in floored cell coordinates, so every true match
//!   shares the 27-neighborhood.
//! - **Boundary-centroid cell** (halo 1, cell = `tol` per axis): coincident
//!   boundaries differ pointwise by `< tol`, so their means differ per-axis
//!   by `< tol` and again share the 27-neighborhood.
//!
//! Faces whose descriptors cannot be established conservatively (non-finite
//! normal or centroid, centroid quotient beyond the exact-integer grid range,
//! non-finite or non-positive tolerance) fall back to the exact ordered
//! all-pairs loop with identical order and predicate, flagged by
//! [`DuplicatePlan::fell_back_to_all_pairs`]. The plane offset `d` is
//! deliberately never bucketed: near-parallel large faces can carry an
//! offset difference far above tolerance while still matching, so any offset
//! bucket would risk excluding a true match (contract §3.1).
//!
//! Processing streams faces in ascending `FaceId` order: for each unremoved
//! face `j`, candidates `i < j` are gathered from the bucket neighborhood in
//! ascending order and the first exact match marks `j` removed. This visits
//! the same decision dependencies as the legacy `i`-outer loop (a decision
//! for `j` only reads removal flags of `i < j`, final by the time `j` runs),
//! so the outcome equals the reference; only pruned pairs — which provably
//! fail the predicate — skip exact evaluation. Memory stays linear: buckets
//! hold one entry per face and each face materializes only its own candidate
//! prefix (dense coincident inputs still cost O(n²) time, reported honestly
//! via the plan stats).

use std::collections::HashMap;

use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::FixResult;
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Cosine threshold for treating two face normals as parallel/anti-parallel.
/// Fixed (not derived from the model's linear tolerance) so a coarse linear
/// tolerance can't widen the angular test into matching clearly-different
/// orientations: `1 - cos θ ≈ θ²/2`, so 1e-6 ≈ 0.08°.
const NORMAL_PARALLEL_COS_TOL: f64 = 1e-6;

/// Normal-bucket cell width per unit-normal component.
///
/// A true match differs by less than ~`1.5e-3` rad (see
/// [`NORMAL_PARALLEL_COS_TOL`]), an order of magnitude below this cell, so a
/// halo of one cell provably covers every true match (module docs).
const NORMAL_CELL: f64 = 8e-3;

/// Largest exactly-representable grid coordinate (`2^53`).
const MAX_EXACT_CELL: f64 = 9_007_199_254_740_992.0;

/// Bucket coordinates for one candidate face.
type Bucket = (usize, (i64, i64, i64), (i64, i64, i64));

/// Eligible-face descriptor for duplicate comparison, in ascending `FaceId`
/// order.
#[derive(Debug, Clone)]
struct FaceDescriptor {
    /// Face identity (ascending order ⇒ lowest index survives each group).
    face: FaceId,
    /// Effective plane normal (stored normal, negated when reversed).
    normal: Vec3,
    /// Ordered outer-boundary vertices.
    points: Vec<Point3>,
    /// Mean of the boundary vertices (spatial bucket center).
    centroid: Point3,
}

/// Result of [`plan_duplicate_removals`].
#[derive(Debug, Clone)]
struct DuplicatePlan {
    /// `(survivor, removed)` pairs in ascending removed-face order.
    pairs: Vec<(FaceId, FaceId)>,
    /// Candidate examinations, including removed-anchor skips that never
    /// reach the exact predicate.
    candidate_exams: u64,
    /// Exact predicate evaluations performed.
    exact_comparisons: u64,
    /// True when degenerate input forced the exact all-pairs fallback.
    fell_back_to_all_pairs: bool,
}

/// Detect and remove geometrically duplicate faces within each shell of a solid.
///
/// This conservative pass only compares unperforated planar polygon faces.
/// Their ordered outer-boundary vertices must coincide with the same winding
/// (allowing a cyclic shift), and their effective normals must agree. Oppositely
/// oriented coincident faces are preserved: removing one would silently choose
/// a side of a zero-thickness or otherwise malformed region. Curved edges and
/// surfaces, NURBS, and perforated faces are skipped because proving their
/// trimmed regions equal requires a parameter-space comparison.
///
/// Comparisons stay within each shell: coincident boundaries in different
/// shells do not establish that either face use can be removed.
pub(super) fn fix_duplicate_faces(
    topo: &Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let solid = topo.solid(solid_id)?;
    let shells: Vec<_> = std::iter::once(solid.outer_shell())
        .chain(solid.inner_shells().iter().copied())
        .collect();
    let mut face_shell_uses = HashMap::new();
    for &shell in &shells {
        for &face in topo.shell(shell)?.faces() {
            *face_shell_uses.entry(face).or_insert(0usize) += 1;
        }
    }
    let mut result = FixResult::ok();
    for shell in shells {
        result.merge(&fix_shell_duplicate_faces(
            topo,
            shell,
            &face_shell_uses,
            ctx,
        )?);
    }
    Ok(result)
}

fn fix_shell_duplicate_faces(
    topo: &Topology,
    shell_id: remus_topology::shell::ShellId,
    face_shell_uses: &HashMap<FaceId, usize>,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let tol = ctx.tolerance.linear;
    let shell = topo.shell(shell_id)?;
    let face_ids: Vec<_> = shell.faces().to_vec();

    // Eligible-face descriptors in ascending FaceId order, so the survivor of
    // each duplicate group is deterministic (lowest index) regardless of
    // shell face order or bucket layout.
    let mut descriptors: Vec<FaceDescriptor> = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        if !face.inner_wires().is_empty() {
            continue;
        }
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            continue;
        };
        let normal = if face.is_reversed() {
            -*normal
        } else {
            *normal
        };

        let wire = topo.wire(face.outer_wire())?;
        let mut points = Vec::with_capacity(wire.edges().len());
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
                points.clear();
                break;
            }
            points.push(topo.vertex(oe.oriented_start(edge))?.point());
        }
        if points.is_empty() {
            continue;
        }
        let centroid = mean_point(&points);
        descriptors.push(FaceDescriptor {
            face: fid,
            normal,
            points,
            centroid,
        });
    }
    descriptors.sort_by_key(|d| d.face.index());

    let plan = plan_duplicate_removals(&descriptors, tol);

    if plan.pairs.is_empty() {
        return Ok(FixResult::ok());
    }

    // ReShape removals are global to this solid; a shared use cannot be dropped locally.
    if plan
        .pairs
        .iter()
        .any(|(_, removed)| face_shell_uses.get(removed).is_some_and(|&uses| uses > 1))
    {
        return Err(HealError::FixFailed(
            "duplicate face is shared by multiple shells".into(),
        ));
    }

    // The lowest-index member of each group is never recorded as removed, so
    // at least one face always survives — the shell can't be emptied.
    for (_, removed) in &plan.pairs {
        ctx.reshape.remove_face(*removed);
    }
    let removed = plan.pairs.len();
    let mut provenance: Vec<String> = plan
        .pairs
        .iter()
        .map(|(survivor, removed)| format!("F{}<-F{}", survivor.index(), removed.index()))
        .collect();
    provenance.sort();
    // The counts below are the scaling evidence trail (PERF-H03): how many
    // pairs the buckets drew versus how many reached the exact predicate.
    // They also keep the plan stats live outside tests.
    let fallback_note = if plan.fell_back_to_all_pairs {
        " via all-pairs fallback"
    } else {
        ""
    };
    ctx.info(format!(
        "removed {removed} duplicate face(s) [{}] ({} candidates, {} exact comparisons{fallback_note})",
        provenance.join(", "),
        plan.candidate_exams,
        plan.exact_comparisons
    ));

    Ok(FixResult::changed(
        Status::DONE2,
        super::RepairActionKind::DuplicateFaceRemoved,
        removed,
    ))
}

/// Plan duplicate removals over index-ordered descriptors.
///
/// Streaming `j`-ascending scan with an in-neighborhood ascending-`i`
/// candidate walk: equivalent to the all-pairs reference over the same order
/// (module docs), minus provably non-matching pairs.
fn plan_duplicate_removals(descriptors: &[FaceDescriptor], tolerance: f64) -> DuplicatePlan {
    let n = descriptors.len();
    if n < 2 {
        return DuplicatePlan {
            pairs: Vec::new(),
            candidate_exams: 0,
            exact_comparisons: 0,
            fell_back_to_all_pairs: false,
        };
    }
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return all_pairs_plan(descriptors, tolerance, true);
    }

    // Per-face bucket keys; faces without a conservative key fall back to
    // universal candidacy (compared against every other face).
    let mut keys: Vec<Option<Bucket>> = Vec::with_capacity(n);
    let mut any_fallback = false;
    for d in descriptors {
        let key = bucket_key(d, tolerance);
        any_fallback |= key.is_none();
        keys.push(key);
    }
    if any_fallback && keys.iter().all(Option::is_none) {
        return all_pairs_plan(descriptors, tolerance, true);
    }

    // Buckets grow in index order, so every bucket is ascending by construction.
    let mut buckets: DetHashMap<Bucket, Vec<usize>> = DetHashMap::default();
    for (idx, key) in keys.iter().enumerate() {
        if let Some(key) = key {
            buckets.entry(*key).or_default().push(idx);
        }
    }

    let mut removed: Vec<bool> = vec![false; n];
    let mut pairs: Vec<(FaceId, FaceId)> = Vec::new();
    let mut candidate_exams = 0u64;
    let mut exact_comparisons = 0u64;
    let mut scratch: Vec<usize> = Vec::new();

    for j in 0..n {
        if removed[j] {
            continue;
        }
        scratch.clear();
        match keys[j] {
            // Degenerate descriptor: every earlier face is a candidate.
            None => scratch.extend(0..j),
            Some((count, (nx, ny, nz), (cx, cy, cz))) => {
                for dx in -1..=1_i64 {
                    for dy in -1..=1_i64 {
                        for dz in -1..=1_i64 {
                            for dnx in -1..=1_i64 {
                                for dny in -1..=1_i64 {
                                    for dnz in -1..=1_i64 {
                                        if let Some(bucket) = buckets.get(&(
                                            count,
                                            (nx + dnx, ny + dny, nz + dnz),
                                            (cx + dx, cy + dy, cz + dz),
                                        )) {
                                            let len = bucket.partition_point(|&i| i < j);
                                            scratch.extend_from_slice(&bucket[..len]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // Universal candidates: fallback faces below `j`.
                for (i, key) in keys.iter().enumerate().take(j) {
                    if key.is_none() {
                        scratch.push(i);
                    }
                }
                scratch.sort_unstable();
                scratch.dedup();
            }
        }
        for &i in &scratch {
            candidate_exams += 1;
            if removed[i] {
                continue;
            }
            exact_comparisons += 1;
            if faces_are_duplicates(&descriptors[i], &descriptors[j], tolerance) {
                removed[j] = true;
                pairs.push((descriptors[i].face, descriptors[j].face));
                break;
            }
        }
    }

    DuplicatePlan {
        pairs,
        candidate_exams,
        exact_comparisons,
        fell_back_to_all_pairs: false,
    }
}

/// Exact legacy all-pairs loop over the same descriptor order.
///
/// Preserved as the degenerate-input fallback and as the production-code
/// shape of the equivalence oracle (tests re-implement it independently in
/// `reference_all_pairs`).
fn all_pairs_plan(
    descriptors: &[FaceDescriptor],
    tolerance: f64,
    fell_back: bool,
) -> DuplicatePlan {
    let n = descriptors.len();
    let mut removed = vec![false; n];
    let mut pairs: Vec<(FaceId, FaceId)> = Vec::new();
    let mut exact_comparisons = 0u64;
    for i in 0..n {
        if removed[i] {
            continue;
        }
        for j in (i + 1)..n {
            if removed[j] {
                continue;
            }
            exact_comparisons += 1;
            if faces_are_duplicates(&descriptors[i], &descriptors[j], tolerance) {
                removed[j] = true;
                pairs.push((descriptors[i].face, descriptors[j].face));
            }
        }
    }
    DuplicatePlan {
        pairs,
        candidate_exams: exact_comparisons,
        exact_comparisons,
        fell_back_to_all_pairs: fell_back,
    }
}

/// Conservative bucket key for one descriptor, or `None` when no safe key
/// exists (the face becomes a universal candidate).
fn bucket_key(descriptor: &FaceDescriptor, tolerance: f64) -> Option<Bucket> {
    let n = descriptor.normal;
    if !n.x().is_finite() || !n.y().is_finite() || !n.z().is_finite() {
        return None;
    }
    let c = descriptor.centroid;
    if !c.x().is_finite() || !c.y().is_finite() || !c.z().is_finite() {
        return None;
    }
    let cell = |v: f64| v / tolerance;
    if cell(c.x()).abs() > MAX_EXACT_CELL
        || cell(c.y()).abs() > MAX_EXACT_CELL
        || cell(c.z()).abs() > MAX_EXACT_CELL
    {
        return None;
    }
    Some((
        descriptor.points.len(),
        (
            (n.x() / NORMAL_CELL).floor() as i64,
            (n.y() / NORMAL_CELL).floor() as i64,
            (n.z() / NORMAL_CELL).floor() as i64,
        ),
        (
            cell(c.x()).floor() as i64,
            cell(c.y()).floor() as i64,
            cell(c.z()).floor() as i64,
        ),
    ))
}

/// Exact duplicate predicate — the final authority.
///
/// Effective normals must agree within [`NORMAL_PARALLEL_COS_TOL`] and the
/// ordered outer boundaries must coincide pointwise within `tolerance` under
/// some cyclic shift with the same winding.
fn faces_are_duplicates(a: &FaceDescriptor, b: &FaceDescriptor, tolerance: f64) -> bool {
    if a.points.len() != b.points.len() {
        return false;
    }
    if a.normal.dot(b.normal) < 1.0 - NORMAL_PARALLEL_COS_TOL {
        return false;
    }
    boundaries_coincide_with_same_winding(&a.points, &b.points, tolerance)
}

fn boundaries_coincide_with_same_winding(a: &[Point3], b: &[Point3], tolerance: f64) -> bool {
    if a.is_empty() || a.len() != b.len() {
        return false;
    }

    (0..b.len()).any(|offset| {
        (a[0] - b[offset]).length() < tolerance
            && (0..a.len())
                .all(|index| (a[index] - b[(offset + index) % b.len()]).length() < tolerance)
    })
}

fn mean_point(points: &[Point3]) -> Point3 {
    let n = points.len() as f64;
    let (sx, sy, sz) = points.iter().fold((0.0, 0.0, 0.0), |(sx, sy, sz), p| {
        (sx + p.x(), sy + p.y(), sz + p.z())
    });
    Point3::new(sx / n, sy / n, sz / n)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::Face;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    /// Add a planar (+Z) triangle face with the given corner points.
    fn add_triangle(topo: &mut Topology, a: Point3, b: Point3, c: Point3) -> FaceId {
        let va = topo.add_vertex(Vertex::new(a, 1e-7));
        let vb = topo.add_vertex(Vertex::new(b, 1e-7));
        let vc = topo.add_vertex(Vertex::new(c, 1e-7));
        let eab = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
        let ebc = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let eca = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire = Wire::new(
            vec![
                OrientedEdge::new(eab, true),
                OrientedEdge::new(ebc, true),
                OrientedEdge::new(eca, true),
            ],
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    /// Add a planar (+Z) quad face with the given corners in order.
    fn add_quad(topo: &mut Topology, corners: [Point3; 4]) -> FaceId {
        let vs: Vec<_> = corners
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
            .collect();
        let es: Vec<_> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        let wire = Wire::new(
            es.into_iter().map(|e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    /// Descriptors for every eligible face of `shell`, in ascending `FaceId`
    /// order (mirrors `fix_shell_duplicate_faces` without recording).
    fn describe_shell(
        topo: &Topology,
        shell: remus_topology::shell::ShellId,
        tol: f64,
    ) -> Vec<FaceDescriptor> {
        let _ = tol;
        let mut out = Vec::new();
        for &fid in topo.shell(shell).unwrap().faces() {
            let face = topo.face(fid).unwrap();
            if !face.inner_wires().is_empty() {
                continue;
            }
            let FaceSurface::Plane { normal, .. } = face.surface() else {
                continue;
            };
            let normal = if face.is_reversed() {
                -*normal
            } else {
                *normal
            };
            let wire = topo.wire(face.outer_wire()).unwrap();
            let mut points = Vec::new();
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                if !matches!(edge.curve(), EdgeCurve::Line) {
                    points.clear();
                    break;
                }
                points.push(topo.vertex(oe.oriented_start(edge)).unwrap().point());
            }
            if points.is_empty() {
                continue;
            }
            let centroid = mean_point(&points);
            out.push(FaceDescriptor {
                face: fid,
                normal,
                points,
                centroid,
            });
        }
        out.sort_by_key(|d| d.face.index());
        out
    }

    /// Independent all-pairs oracle over the same descriptor order: legacy
    /// `i`-outer loop, own predicate evaluation, no buckets.
    fn reference_all_pairs(
        descriptors: &[FaceDescriptor],
        tolerance: f64,
    ) -> Vec<(FaceId, FaceId)> {
        let n = descriptors.len();
        let mut removed = vec![false; n];
        let mut pairs = Vec::new();
        for i in 0..n {
            if removed[i] {
                continue;
            }
            for j in (i + 1)..n {
                if removed[j] {
                    continue;
                }
                let a = &descriptors[i];
                let b = &descriptors[j];
                let same_len = a.points.len() == b.points.len();
                let same_normal = a.normal.dot(b.normal) >= 1.0 - NORMAL_PARALLEL_COS_TOL;
                if same_len
                    && same_normal
                    && boundaries_coincide_with_same_winding(&a.points, &b.points, tolerance)
                {
                    removed[j] = true;
                    pairs.push((a.face, b.face));
                }
            }
        }
        pairs
    }

    fn assert_plan_equals_reference(
        descriptors: &[FaceDescriptor],
        tolerance: f64,
    ) -> DuplicatePlan {
        let plan = plan_duplicate_removals(descriptors, tolerance);
        let reference = reference_all_pairs(descriptors, tolerance);
        assert_eq!(
            plan.pairs, reference,
            "indexed plan must equal the all-pairs reference (survivor, order, removals)"
        );
        // The index may only skip exact evaluations, never change the outcome.
        let dense = all_pairs_plan(descriptors, tolerance, false);
        assert_eq!(dense.pairs, reference);
        assert!(
            plan.exact_comparisons <= dense.exact_comparisons.max(1),
            "index must not evaluate more pairs than all-pairs"
        );
        plan
    }

    /// Deterministic LCG for reproducible fixtures (no external deps).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (self.0 >> 33) & 0x7fff_ffff
        }
        fn next_f64(&mut self) -> f64 {
            f64::from(self.next() as u32) / f64::from(u32::MAX)
        }
    }

    #[test]
    fn duplicate_repair_refuses_removal_of_a_shared_face_identity() {
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for _ in 0..2 {
            faces.push(add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));
        }
        let outer = topo.add_shell(Shell::new(vec![faces[1]]).unwrap());
        let inner = topo.add_shell(Shell::new(faces).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        let mut ctx = HealContext::new();
        let error = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap_err();
        assert!(error.to_string().contains("shared by multiple shells"));
        assert!(ctx.reshape.is_empty());
    }

    #[test]
    fn duplicate_repair_visits_cavities_without_merging_across_shells() {
        let mut topo = Topology::new();
        let mut triangles = Vec::new();
        for _ in 0..3 {
            triangles.push(add_triangle(
                &mut topo,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));
        }
        // Coincident shell-local fixtures isolate duplicate scope, not cavity containment.
        let outer = topo.add_shell(Shell::new(vec![triangles[0]]).unwrap());
        let inner = topo.add_shell(Shell::new(vec![triangles[1], triangles[2]]).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        let mut ctx = HealContext::new();
        let report = fix_duplicate_faces(&topo, solid, &mut ctx).unwrap();
        assert_eq!(report.actions_taken, 1);
        assert!(!ctx.reshape.is_face_removed(triangles[0]));
        assert!(!ctx.reshape.is_face_removed(triangles[1]));
        assert!(ctx.reshape.is_face_removed(triangles[2]));
        ctx.reshape.apply(&mut topo, solid).unwrap();
        assert_eq!(
            topo.shell(topo.solid(solid).unwrap().outer_shell())
                .unwrap()
                .faces()
                .len(),
            1
        );
        assert_eq!(
            topo.shell(topo.solid(solid).unwrap().inner_shells()[0])
                .unwrap()
                .faces()
                .len(),
            1
        );
    }

    #[test]
    fn flags_and_removes_a_coincident_duplicate_face() {
        let mut topo = Topology::new();
        // Three distinct faces plus an exact geometric duplicate of the first.
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
        );
        let c = add_triangle(
            &mut topo,
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(1.0, 5.0, 0.0),
            Point3::new(0.0, 6.0, 0.0),
        );
        let dup = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b, c, dup]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 1, "exactly one duplicate expected");
        assert!(
            ctx.reshape.is_face_removed(dup),
            "the later face is removed"
        );
        assert!(!ctx.reshape.is_face_removed(a), "the original is kept");
        assert!(!ctx.reshape.is_face_removed(b));

        // Applying the reshape drops the duplicate from the shell.
        ctx.reshape.apply(&mut topo, solid_id).unwrap();
        let faces = topo
            .shell(topo.solid(solid_id).unwrap().outer_shell())
            .unwrap();
        assert_eq!(faces.faces().len(), 3, "shell drops the duplicate");
    }

    #[test]
    fn keeps_all_distinct_faces() {
        let mut topo = Topology::new();
        let a = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let b = add_triangle(
            &mut topo,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();
        assert_eq!(
            result.actions_taken, 0,
            "no duplicates among distinct faces"
        );
    }

    #[test]
    fn keeps_same_centroid_faces_with_different_boundaries() {
        let mut topo = Topology::new();
        let small = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        );
        let large = add_triangle(
            &mut topo,
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(5.0, -1.0, 0.0),
            Point3::new(-1.0, 5.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![small, large]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(small));
        assert!(!ctx.reshape.is_face_removed(large));
    }

    #[test]
    fn keeps_coincident_face_with_opposite_winding() {
        let mut topo = Topology::new();
        let original = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let opposite = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(original));
        assert!(!ctx.reshape.is_face_removed(opposite));
    }

    #[test]
    fn keeps_coincident_face_with_opposite_effective_normal() {
        let mut topo = Topology::new();
        let original = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let opposite = add_triangle(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        topo.face_mut(opposite).unwrap().set_reversed(true);
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));

        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();

        assert_eq!(result.actions_taken, 0);
        assert!(!ctx.reshape.is_face_removed(original));
        assert!(!ctx.reshape.is_face_removed(opposite));
    }

    // ── PERF-H03 indexing proofs ──

    #[test]
    fn survivor_is_lowest_index_regardless_of_shell_order() {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let first = add_quad(&mut topo, corners);
        let second = add_quad(&mut topo, corners);
        // Shell order is reverse index order: the survivor must still be the
        // lowest FaceId, not the first shell entry.
        let shell = topo.add_shell(Shell::new(vec![second, first]).unwrap());
        let solid_id = topo.add_solid(Solid::new(shell, vec![]));
        let mut ctx = HealContext::new();
        let result = fix_duplicate_faces(&topo, solid_id, &mut ctx).unwrap();
        assert_eq!(result.actions_taken, 1);
        assert!(!ctx.reshape.is_face_removed(first));
        assert!(ctx.reshape.is_face_removed(second));
    }

    #[test]
    fn sparse_model_pays_no_exact_comparisons() {
        // 400 disjoint coplanar quads on an integer grid: same edge count and
        // same normal cell, but centroid buckets never meet.
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for i in 0..400 {
            let x = f64::from(i) * 10.0;
            faces.push(add_quad(
                &mut topo,
                [
                    Point3::new(x, 0.0, 0.0),
                    Point3::new(x + 1.0, 0.0, 0.0),
                    Point3::new(x + 1.0, 1.0, 0.0),
                    Point3::new(x, 1.0, 0.0),
                ],
            ));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let plan = assert_plan_equals_reference(&descriptors, 1e-7);
        assert!(plan.pairs.is_empty());
        assert_eq!(plan.exact_comparisons, 0);
        assert_eq!(plan.candidate_exams, 0);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn dense_coincident_exits_early_on_the_survivor() {
        // 60 identical quads: one bucket, but each face after the first
        // matches the survivor immediately — linear exams and exact calls.
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let mut faces = Vec::new();
        for _ in 0..60 {
            faces.push(add_quad(&mut topo, corners));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let plan = assert_plan_equals_reference(&descriptors, 1e-7);
        assert_eq!(plan.pairs.len(), 59);
        assert_eq!(plan.exact_comparisons, 59);
        assert_eq!(plan.candidate_exams, 59);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn dense_bucket_distinct_faces_pay_quadratic_exams() {
        // Honest worst case: 60 faces sharing one bucket (identical centroid
        // by cancelling perturbations, same normal, same edge count) that are
        // pairwise all distinct (corner drift of 2*tol per step, so no cyclic
        // shift can align any two within tol). Every pair is drawn and reaches
        // the exact predicate: exams and exact calls both stay quadratic.
        let tol = 1e-7;
        let mut topo = Topology::new();
        let mut faces = Vec::new();
        for k in 0..60 {
            let d = f64::from(k) * 2.0 * tol;
            faces.push(add_quad(
                &mut topo,
                [
                    Point3::new(d, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(1.0 - d, 1.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
            ));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        // All centroids coincide: single bucket by construction.
        let first = &descriptors[0];
        for d in &descriptors {
            assert!((d.centroid - first.centroid).length() < 1e-9);
        }
        let plan = assert_plan_equals_reference(&descriptors, tol);
        assert!(plan.pairs.is_empty(), "all faces are pairwise distinct");
        assert_eq!(plan.candidate_exams, 60 * 59 / 2);
        assert_eq!(plan.exact_comparisons, 60 * 59 / 2);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn tolerance_boundary_pairs_do_not_match() {
        let tol = 1e-7;
        let mut topo = Topology::new();
        let base = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Exactly tol away in one corner: strict `< tol` keeps both faces.
        let at_tol = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0 + tol, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Just inside tol: duplicates.
        let inside = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0 + tol * (1.0 - 1e-6), 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let a = add_quad(&mut topo, base);
        let b = add_quad(&mut topo, at_tol);
        let c = add_quad(&mut topo, inside);
        let shell = topo.add_shell(Shell::new(vec![a, b, c]).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        let plan = assert_plan_equals_reference(&descriptors, tol);
        // b is kept (boundary distance == tol), c duplicates a.
        assert_eq!(
            plan.pairs,
            vec![(a, c)],
            "at-tolerance pair is kept, inside-tolerance pair merges"
        );
    }

    #[test]
    fn nontransitive_nearness_keeps_the_distant_face() {
        // Chain at 0.6*tol shifts: A≈B and B≈C match, but A≉C (1.2*tol).
        // B is removed into A; C must stay — no transitive chaining.
        let tol = 1e-7;
        let shift = |dx: f64| {
            [
                Point3::new(dx, 0.0, 0.0),
                Point3::new(1.0 + dx, 0.0, 0.0),
                Point3::new(1.0 + dx, 1.0, 0.0),
                Point3::new(dx, 1.0, 0.0),
            ]
        };
        let mut topo = Topology::new();
        let a = add_quad(&mut topo, shift(0.0));
        let b = add_quad(&mut topo, shift(0.6 * tol));
        let c = add_quad(&mut topo, shift(1.2 * tol));
        let shell = topo.add_shell(Shell::new(vec![a, b, c]).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        let plan = assert_plan_equals_reference(&descriptors, tol);
        assert_eq!(plan.pairs, vec![(a, b)]);
    }

    #[test]
    fn cell_boundary_straddlers_neither_miss_nor_invent() {
        let tol = 1e-7;
        // Centroid straddling a cell border at x = 10*tol, 4e-9 apart overall:
        // the shifted copy is a duplicate and must be found across the border.
        let mut topo = Topology::new();
        let base_x = 10.0 * tol - 2e-9;
        let mk = |dx: f64| {
            [
                Point3::new(base_x + dx, 0.0, 0.0),
                Point3::new(base_x + dx + 1.0, 0.0, 0.0),
                Point3::new(base_x + dx + 1.0, 1.0, 0.0),
                Point3::new(base_x + dx, 1.0, 0.0),
            ]
        };
        let a = add_quad(&mut topo, mk(0.0));
        let b = add_quad(&mut topo, mk(4e-9));
        // Far quad sharing the normal cell but nothing else: never a candidate.
        let c = add_quad(
            &mut topo,
            [
                Point3::new(500.0, 0.0, 0.0),
                Point3::new(501.0, 0.0, 0.0),
                Point3::new(501.0, 1.0, 0.0),
                Point3::new(500.0, 1.0, 0.0),
            ],
        );
        let shell = topo.add_shell(Shell::new(vec![a, b, c]).unwrap());
        let descriptors = describe_shell(&topo, shell, tol);
        let plan = assert_plan_equals_reference(&descriptors, tol);
        assert_eq!(plan.pairs, vec![(a, b)]);
    }

    #[test]
    fn degenerate_tolerance_falls_back_without_changing_the_map() {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let a = add_quad(&mut topo, corners);
        let b = add_quad(&mut topo, corners);
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        for tol in [0.0, -1e-7, f64::NAN, f64::INFINITY] {
            let plan = plan_duplicate_removals(&descriptors, tol);
            assert!(plan.fell_back_to_all_pairs, "tol {tol}");
            assert_eq!(plan.pairs, reference_all_pairs(&descriptors, tol));
        }
    }

    #[test]
    fn beyond_grid_range_falls_back() {
        // 1e16 with tol 1e-7 needs centroid cell index 1e23: beyond exact
        // integers, so both faces fall back — still deciding correctly.
        let mut topo = Topology::new();
        let mk = |dx: f64| {
            [
                Point3::new(1e16 + dx, 0.0, 0.0),
                Point3::new(1e16 + dx + 1.0, 0.0, 0.0),
                Point3::new(1e16 + dx + 1.0, 1.0, 0.0),
                Point3::new(1e16 + dx, 1.0, 0.0),
            ]
        };
        let a = add_quad(&mut topo, mk(0.0));
        let b = add_quad(&mut topo, mk(0.0));
        let shell = topo.add_shell(Shell::new(vec![a, b]).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let plan = plan_duplicate_removals(&descriptors, 1e-7);
        assert!(plan.fell_back_to_all_pairs);
        assert_eq!(plan.pairs, vec![(a, b)]);
    }

    #[test]
    fn randomized_fixtures_match_reference() {
        let tol = 1e-7;
        for size in [0, 1, 2, 3, 17, 100, 250] {
            // Sparse grid with a negative offset (cell-boundary coverage).
            let mut topo = Topology::new();
            let mut faces = Vec::new();
            for i in 0..size {
                let x = -50.0 + f64::from(i) * 1.7;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(x, 3.25, -1.5),
                        Point3::new(x + 1.0, 3.25, -1.5),
                        Point3::new(x + 1.0, 4.25, -1.5),
                        Point3::new(x, 4.25, -1.5),
                    ],
                ));
            }
            // One exact duplicate of the first face when non-empty.
            if size > 0 {
                let x = -50.0;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(x, 3.25, -1.5),
                        Point3::new(x + 1.0, 3.25, -1.5),
                        Point3::new(x + 1.0, 4.25, -1.5),
                        Point3::new(x, 4.25, -1.5),
                    ],
                ));
            }
            if faces.is_empty() {
                continue;
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            let plan = assert_plan_equals_reference(&descriptors, tol);
            if size > 0 {
                assert_eq!(plan.pairs.len(), 1, "size {size}: one duplicate");
            }

            // Clustered near-duplicates with deterministic jitter around
            // shared bases (some inside tol, some outside).
            let mut topo = Topology::new();
            let mut rng = Lcg(0xabcd + size as u64);
            let mut faces = Vec::new();
            for i in 0..size {
                let base = f64::from(i / 3) * 2.3 - 100.0;
                let jx = (rng.next_f64() - 0.5) * 1.2e-7;
                let jy = (rng.next_f64() - 0.5) * 1.2e-7;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(base + jx, jy, 0.0),
                        Point3::new(base + jx + 1.0, jy, 0.0),
                        Point3::new(base + jx + 1.0, jy + 1.0, 0.0),
                        Point3::new(base + jx, jy + 1.0, 0.0),
                    ],
                ));
            }
            if faces.is_empty() {
                continue;
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            assert_plan_equals_reference(&descriptors, tol);

            // Adversarial: many coincident copies plus jittered neighbors.
            let mut topo = Topology::new();
            let mut rng = Lcg(0x55aa + size as u64);
            let mut faces = Vec::new();
            for _ in 0..size {
                let jx = rng.next_f64() * 5e-8;
                let jy = rng.next_f64() * 5e-8;
                faces.push(add_quad(
                    &mut topo,
                    [
                        Point3::new(-7.0 + jx, 11.0 + jy, 0.0),
                        Point3::new(-6.0 + jx, 11.0 + jy, 0.0),
                        Point3::new(-6.0 + jx, 12.0 + jy, 0.0),
                        Point3::new(-7.0 + jx, 12.0 + jy, 0.0),
                    ],
                ));
            }
            if faces.is_empty() {
                continue;
            }
            let shell = topo.add_shell(Shell::new(faces).unwrap());
            let descriptors = describe_shell(&topo, shell, tol);
            assert_plan_equals_reference(&descriptors, tol);
        }
    }

    #[test]
    fn plan_is_deterministic_across_runs() {
        let mut topo = Topology::new();
        let mut rng = Lcg(0xbeef);
        let mut faces = Vec::new();
        for i in 0..600 {
            let base = f64::from(i / 3) * 1.1;
            let jx = (rng.next_f64() - 0.5) * 1.2e-7;
            let jy = (rng.next_f64() - 0.5) * 1.2e-7;
            faces.push(add_quad(
                &mut topo,
                [
                    Point3::new(base + jx, jy, 0.0),
                    Point3::new(base + jx + 1.0, jy, 0.0),
                    Point3::new(base + jx + 1.0, jy + 1.0, 0.0),
                    Point3::new(base + jx, jy + 1.0, 0.0),
                ],
            ));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let descriptors = describe_shell(&topo, shell, 1e-7);
        let a = plan_duplicate_removals(&descriptors, 1e-7);
        let b = plan_duplicate_removals(&descriptors, 1e-7);
        assert_eq!(a.pairs, b.pairs);
        assert_eq!(a.candidate_exams, b.candidate_exams);
        assert_eq!(a.exact_comparisons, b.exact_comparisons);
    }
}
