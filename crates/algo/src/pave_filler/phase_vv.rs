//! Phase VV: Vertex-vertex coincidence detection.
//!
//! Finds all vertex pairs (one from each solid) that are spatially
//! coincident within tolerance. Merges them via same-domain mapping.

use remus_math::context::OperationContext;
use remus_topology::Topology;
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use crate::ds::{GfaArena, Interference};
use crate::error::AlgoError;

/// Shared across every source pair in an N-way pave-filling pass.
pub(super) struct VertexPairBudget {
    limit: usize,
    reserved: usize,
}

impl VertexPairBudget {
    pub(super) fn new(limit: usize) -> Self {
        Self { limit, reserved: 0 }
    }

    fn reserve(&mut self, count_a: usize, count_b: usize) -> Result<(), AlgoError> {
        let actual = count_a
            .checked_mul(count_b)
            .and_then(|count| self.reserved.checked_add(count));
        match actual {
            Some(actual) if actual <= self.limit => {
                self.reserved = actual;
                Ok(())
            }
            _ => Err(AlgoError::ResourceLimitExceeded {
                resource: "GFA vertex pairs",
                limit: self.limit,
                actual: actual.unwrap_or(usize::MAX),
            }),
        }
    }
}

/// Small inputs avoid sorting overhead and retain the Cartesian reservation.
const DIRECT_PAIR_THRESHOLD: usize = 4_096;

/// Candidate rows over vertices sorted by X. Discovery ranks restore the
/// original traversal order before any exact test or same-domain mutation.
struct CandidateRows {
    sorted: Vec<(f64, usize)>,
    rows: Vec<std::ops::Range<usize>>,
    count: usize,
}

fn candidate_rows(
    topo: &Topology,
    verts_a: &[VertexId],
    verts_b: &[VertexId],
    context: &OperationContext,
) -> Result<Option<CandidateRows>, AlgoError> {
    if verts_a.len().saturating_mul(verts_b.len()) <= DIRECT_PAIR_THRESHOLD {
        return Ok(None);
    }
    let mut max_a = 0.0_f64;
    let mut max_b = 0.0_f64;
    let mut sorted = Vec::with_capacity(verts_b.len());
    for (rank, &id) in verts_b.iter().enumerate() {
        if rank.is_multiple_of(1_024) {
            context.check_cancelled()?;
        }
        let vertex = topo.vertex(id)?;
        if !vertex.point().x().is_finite() || !vertex.tolerance().is_finite() {
            return Ok(None);
        }
        max_b = max_b.max(vertex.tolerance());
        sorted.push((vertex.point().x(), rank));
    }
    for (rank, &id) in verts_a.iter().enumerate() {
        if rank.is_multiple_of(1_024) {
            context.check_cancelled()?;
        }
        let vertex = topo.vertex(id)?;
        if !vertex.point().x().is_finite() || !vertex.tolerance().is_finite() {
            return Ok(None);
        }
        max_a = max_a.max(vertex.tolerance());
    }
    let radius = max_a + max_b + context.tolerance.linear;
    if !radius.is_finite() || radius < 0.0 {
        return Ok(None);
    }
    // Twice the maximum combined tolerance leaves rounding slack. The floor
    // also covers pairs whose squared distance underflows in the unchanged
    // Euclidean predicate. Degenerate/overflowing radii use bounded all-pairs.
    let radius = (2.0 * radius).max(f64::MIN_POSITIVE.sqrt());
    if !radius.is_finite() {
        return Ok(None);
    }
    sorted.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    context.check_cancelled()?;
    let mut rows = Vec::with_capacity(verts_a.len());
    let mut count = 0usize;
    for (rank, &id) in verts_a.iter().enumerate() {
        if rank.is_multiple_of(1_024) {
            context.check_cancelled()?;
        }
        let x = topo.vertex(id)?.point().x();
        // Subtract in the same direction as the exact distance predicate;
        // no rounded x +/- radius endpoints can exclude a qualifying pair.
        let lo = sorted.partition_point(|&(bx, _)| x - bx > radius);
        let hi = sorted.partition_point(|&(bx, _)| x - bx >= -radius);
        count = count.saturating_add(hi - lo);
        rows.push(lo..hi);
    }
    Ok(Some(CandidateRows {
        sorted,
        rows,
        count,
    }))
}

/// Detect coincident vertices between solid A and solid B.
///
/// For every `(va, vb)` pair where `va` belongs to `solid_a` and `vb` to
/// `solid_b`, check if they are within combined tolerance. Coincident
/// pairs are recorded as VV interferences and merged in the same-domain
/// vertex map.
///
/// # Errors
///
/// Returns [`AlgoError`] if a lookup fails, cancellation is requested, or
/// the total vertex-pair comparison budget would be exceeded. The complete
/// candidate work is reserved before any interference or merge is recorded.
/// Large inputs prune distant X coordinates conservatively; dense or invalid
/// inputs retain a bounded Cartesian scan.
pub(super) fn perform_with_context(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    context: &OperationContext,
    arena: &mut GfaArena,
    budget: &mut VertexPairBudget,
) -> Result<(), AlgoError> {
    context.check_cancelled()?;
    let tol = context.tolerance;
    // AABB pre-filter: skip if solids are disjoint
    let bbox_a = crate::classifier::compute_solid_bbox(topo, solid_a)?;
    let bbox_b = crate::classifier::compute_solid_bbox(topo, solid_b)?;
    if !bbox_a
        .expanded(tol.linear)
        .intersects(bbox_b.expanded(tol.linear))
    {
        log::debug!("VV: solids are disjoint, skipping");
        return Ok(());
    }

    let verts_a = remus_topology::explorer::solid_vertices(topo, solid_a)?;
    let verts_b = remus_topology::explorer::solid_vertices(topo, solid_b)?;

    context.check_cancelled()?;
    let candidates = candidate_rows(topo, &verts_a, &verts_b, context)?;
    if let Some(plan) = &candidates {
        budget.reserve(plan.count, 1)?;
    } else {
        budget.reserve(verts_a.len(), verts_b.len())?;
    }
    let mut row_ranks = Vec::new();
    let mut checks = 0usize;

    for (row, &va) in verts_a.iter().enumerate() {
        context.check_cancelled()?;
        let vertex_a = topo.vertex(va)?;
        let pos_a = vertex_a.point();
        let tol_a = vertex_a.tolerance();

        row_ranks.clear();
        if let Some(plan) = &candidates {
            row_ranks.extend(
                plan.sorted[plan.rows[row].clone()]
                    .iter()
                    .map(|&(_, rank)| rank),
            );
            row_ranks.sort_unstable();
        } else {
            row_ranks.extend(0..verts_b.len());
        }
        for &rank in &row_ranks {
            let vb = verts_b[rank];
            // Check both within a large row and between rows. Cancellation
            // never changes which exact pairs qualify for coincidence.
            if checks.is_multiple_of(1_024) {
                context.check_cancelled()?;
            }
            checks += 1;
            let vertex_b = topo.vertex(vb)?;
            let pos_b = vertex_b.point();
            let tol_b = vertex_b.tolerance();

            let combined_tol = tol_a + tol_b + tol.linear;
            let dist = (pos_a - pos_b).length();

            if dist <= combined_tol {
                arena
                    .interference
                    .vv
                    .push(Interference::VV { v1: va, v2: vb });

                arena.merge_vertices(va, vb);

                log::debug!("VV: vertices {va:?} and {vb:?} coincide (dist={dist:.2e})");
            }
        }
    }

    context.check_cancelled()?;

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use remus_math::tolerance::Tolerance;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::{Solid, SolidId};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    fn perform(
        topo: &Topology,
        a: SolidId,
        b: SolidId,
        tol: Tolerance,
        arena: &mut GfaArena,
    ) -> Result<(), AlgoError> {
        let context = OperationContext::new().with_tolerance(tol);
        let mut budget = VertexPairBudget::new(context.budgets.vertex_pairs);
        perform_with_context(topo, a, b, &context, arena, &mut budget)
    }

    /// Builds a square quad-face solid spanning `[x0, x1] × [y0, y1]` at
    /// height `z`, every vertex carrying `ball`.
    fn quad_solid(topo: &mut Topology, min: [f64; 2], max: [f64; 2], z: f64, ball: f64) -> SolidId {
        let [x0, y0] = min;
        let [x1, y1] = max;

        let corners: [[f64; 2]; 4] = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        let v: Vec<remus_topology::vertex::VertexId> = (0..4)
            .map(|i| {
                topo.add_vertex(Vertex::new(
                    Point3::new(corners[i][0], corners[i][1], z),
                    ball,
                ))
            })
            .collect();

        let e01 = topo.add_edge(Edge::new(v[0], v[1], EdgeCurve::Line));
        let e12 = topo.add_edge(Edge::new(v[1], v[2], EdgeCurve::Line));
        let e23 = topo.add_edge(Edge::new(v[2], v[3], EdgeCurve::Line));
        let e30 = topo.add_edge(Edge::new(v[3], v[0], EdgeCurve::Line));

        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e01, true),
                    OrientedEdge::new(e12, true),
                    OrientedEdge::new(e23, true),
                    OrientedEdge::new(e30, true),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: -z,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        topo.add_solid(Solid::new(shell, vec![]))
    }

    #[test]
    fn vv_merges_a_pair_separated_up_to_the_ball_sum() {
        // Program doc 3.3 exit-gate fixture, written as a passing pin (RFC
        // 0004): two overlapping unit quads offset by 1e-6 in x put four
        // corner pairs 1e-6 apart — 10× the global tolerance, below
        // `ball_a + ball_b + tol.linear` (1e-6 + 1e-6 + 1e-7) — so only the
        // declared balls make the pairs interfere, exactly as
        // `combined_tol` here computes it.
        let mut topo = Topology::new();
        let a = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-6);
        let b = quad_solid(&mut topo, [1e-6, 0.0], [1.0 + 1e-6, 1.0], 0.0, 1e-6);

        let mut arena = GfaArena::new();
        perform(&topo, a, b, Tolerance::default(), &mut arena).unwrap();

        assert_eq!(
            arena.interference.vv.len(),
            4,
            "each 1e-6 corner pair merges"
        );
        assert_eq!(arena.same_domain_vertices.len(), 4);
    }

    #[test]
    fn vv_ignores_a_pair_beyond_the_ball_sum() {
        let mut topo = Topology::new();
        let a = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-7);
        let b = quad_solid(&mut topo, [1e-6, 0.0], [1.0 + 1e-6, 1.0], 0.0, 1e-7);

        let mut arena = GfaArena::new();
        perform(&topo, a, b, Tolerance::default(), &mut arena).unwrap();

        assert!(
            arena.interference.vv.is_empty(),
            "a pair beyond ball_a + ball_b + tol.linear must not merge"
        );
        assert!(arena.same_domain_vertices.is_empty());
    }

    #[test]
    fn vv_refuses_before_recording_any_coincidence() {
        let mut topo = Topology::new();
        let a = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-6);
        let b = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-6);
        let mut arena = GfaArena::new();
        let mut budget = VertexPairBudget::new(15);
        assert!(matches!(
            perform_with_context(
                &topo,
                a,
                b,
                &OperationContext::new(),
                &mut arena,
                &mut budget
            ),
            Err(AlgoError::ResourceLimitExceeded {
                limit: 15,
                actual: 16,
                ..
            })
        ));
        assert_eq!(budget.reserved, 0);
        assert!(arena.interference.vv.is_empty());
        assert!(arena.same_domain_vertices.is_empty());

        let mut budget = VertexPairBudget::new(16);
        perform_with_context(
            &topo,
            a,
            b,
            &OperationContext::new(),
            &mut arena,
            &mut budget,
        )
        .unwrap();
        assert_eq!(budget.reserved, 16);
        assert_eq!(arena.interference.vv.len(), 4);
        assert_eq!(arena.same_domain_vertices.len(), 4);
    }

    #[test]
    fn vv_budget_counts_cavity_and_inner_wire_vertices() {
        let mut topo = Topology::new();
        let a = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-6);
        let inner = quad_solid(&mut topo, [0.1, 0.1], [0.9, 0.9], 0.0, 1e-6);
        let inner_shell = topo.solid(inner).unwrap().outer_shell();
        let outer_shell = topo.solid(a).unwrap().outer_shell();
        let cavity = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));
        let b = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-6);
        let mut budget = VertexPairBudget::new(31);
        let mut arena = GfaArena::new();
        assert!(matches!(
            perform_with_context(
                &topo,
                cavity,
                b,
                &OperationContext::new(),
                &mut arena,
                &mut budget
            ),
            Err(AlgoError::ResourceLimitExceeded { actual: 32, .. })
        ));
        let outer_face = topo.shell(outer_shell).unwrap().faces()[0];
        let inner_face = topo.shell(inner_shell).unwrap().faces()[0];
        let inner_wire = topo.face(inner_face).unwrap().outer_wire();
        let outer_wire = topo.face(outer_face).unwrap().outer_wire();
        topo.set_face_boundary_wires(outer_face, outer_wire, vec![inner_wire])
            .unwrap();
        assert!(matches!(
            perform_with_context(
                &topo,
                a,
                b,
                &OperationContext::new(),
                &mut arena,
                &mut budget
            ),
            Err(AlgoError::ResourceLimitExceeded { actual: 32, .. })
        ));
        assert!(arena.interference.vv.is_empty());
    }

    #[test]
    fn vv_disjoint_broadphase_spends_no_pair_budget() {
        let mut topo = Topology::new();
        let a = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-7);
        let b = quad_solid(&mut topo, [2.0, 2.0], [3.0, 3.0], 0.0, 1e-7);
        let mut budget = VertexPairBudget::new(0);
        let mut arena = GfaArena::new();
        perform_with_context(
            &topo,
            a,
            b,
            &OperationContext::new(),
            &mut arena,
            &mut budget,
        )
        .unwrap();
        assert_eq!(budget.reserved, 0);
        assert!(arena.interference.vv.is_empty());
    }

    #[test]
    fn vv_budget_reservation_is_checked_and_shared() {
        let mut budget = VertexPairBudget::new(47);
        budget.reserve(4, 4).unwrap();
        budget.reserve(4, 4).unwrap();
        assert!(matches!(
            budget.reserve(4, 4),
            Err(AlgoError::ResourceLimitExceeded { actual: 48, .. })
        ));
        assert_eq!(budget.reserved, 32);
        let mut budget = VertexPairBudget::new(usize::MAX);
        assert!(budget.reserve(usize::MAX, 2).is_err());
        budget.reserve(usize::MAX, 1).unwrap();
        assert!(budget.reserve(1, 1).is_err());
    }

    #[test]
    fn vv_cancellation_is_preserved_before_work() {
        let mut topo = Topology::new();
        let a = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-7);
        let b = quad_solid(&mut topo, [0.0, 0.0], [1.0, 1.0], 0.0, 1e-7);
        let token = remus_math::context::CancellationToken::new();
        let context = OperationContext::new().with_cancellation(token.clone());
        token.cancel();
        let mut budget = VertexPairBudget::new(16);
        let mut arena = GfaArena::new();
        assert!(matches!(
            perform_with_context(&topo, a, b, &context, &mut arena, &mut budget),
            Err(AlgoError::Math(remus_math::MathError::Cancelled))
        ));
        assert_eq!(budget.reserved, 0);
        assert!(arena.interference.vv.is_empty());
    }
}

#[cfg(test)]
mod candidate_tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use remus_math::vec::Point3;
    use remus_topology::vertex::Vertex;

    #[test]
    fn indexed_candidates_preserve_cartesian_coincidences_and_order() {
        let mut topo = Topology::new();
        let a: Vec<_> = (0..65)
            .map(|i| topo.add_vertex(Vertex::new(Point3::new(f64::from(i), 0.0, 0.0), 1e-7)))
            .collect();
        let b: Vec<_> = (0..65)
            .rev()
            .map(|i| {
                topo.add_vertex(Vertex::new(
                    Point3::new(f64::from(i) + 1e-7, 0.0, 0.0),
                    if i == 32 { 1.0 } else { 1e-7 },
                ))
            })
            .collect();
        let context = OperationContext::new();
        let plan = candidate_rows(&topo, &a, &b, &context).unwrap().unwrap();
        assert!(plan.count < a.len() * b.len());
        let qualifies = |ia: usize, ib: usize| {
            let va = topo.vertex(a[ia]).unwrap();
            let vb = topo.vertex(b[ib]).unwrap();
            (va.point() - vb.point()).length()
                <= va.tolerance() + vb.tolerance() + context.tolerance.linear
        };
        let oracle: Vec<_> = (0..a.len())
            .flat_map(|ia| (0..b.len()).map(move |ib| (ia, ib)))
            .filter(|&(ia, ib)| qualifies(ia, ib))
            .collect();
        let mut actual = Vec::new();
        for (ia, row) in plan.rows.iter().enumerate() {
            let mut ranks: Vec<_> = plan.sorted[row.clone()]
                .iter()
                .map(|&(_, rank)| rank)
                .collect();
            ranks.sort_unstable();
            actual.extend(
                ranks
                    .into_iter()
                    .filter(|&ib| qualifies(ia, ib))
                    .map(|ib| (ia, ib)),
            );
        }
        assert_eq!(actual, oracle);
        let mut budget = VertexPairBudget::new(plan.count);
        budget.reserve(plan.count, 1).unwrap();
        assert!(budget.reserve(1, 1).is_err());
    }

    #[test]
    fn dense_indexed_rows_keep_the_hard_limit_before_publication() {
        let mut topo = Topology::new();
        let ids: Vec<_> = (0..65)
            .map(|_| topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7)))
            .collect();
        let plan = candidate_rows(&topo, &ids, &ids, &OperationContext::new())
            .unwrap()
            .unwrap();
        assert_eq!(plan.count, 4_225);
        let mut budget = VertexPairBudget::new(4_224);
        assert!(matches!(
            budget.reserve(plan.count, 1),
            Err(AlgoError::ResourceLimitExceeded {
                limit: 4_224,
                actual: 4_225,
                ..
            })
        ));
        assert_eq!(budget.reserved, 0);
    }

    #[test]
    fn indexed_rows_preserve_underflow_and_degenerate_fallback() {
        let mut topo = Topology::new();
        let ids: Vec<_> = (0..65)
            .map(|i| {
                topo.add_vertex(Vertex::new(
                    Point3::new(f64::from(i) * 1e-200, 0.0, 0.0),
                    0.0,
                ))
            })
            .collect();
        let context = OperationContext::new().with_tolerance(remus_math::tolerance::Tolerance {
            linear: 0.0,
            angular: 0.0,
            relative: 0.0,
        });
        let plan = candidate_rows(&topo, &ids, &ids, &context)
            .unwrap()
            .unwrap();
        assert_eq!(plan.count, 4_225);
        assert_eq!(
            (topo.vertex(ids[0]).unwrap().point() - topo.vertex(ids[64]).unwrap().point())
                .length()
                .to_bits(),
            0.0_f64.to_bits()
        );
        let mut invalid = ids.clone();
        invalid.push(topo.add_vertex(Vertex::new(Point3::new(f64::NAN, 0.0, 0.0), 1e-7)));
        assert!(
            candidate_rows(&topo, &ids, &invalid, &context)
                .unwrap()
                .is_none()
        );
    }
}
