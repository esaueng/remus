//! Phase VE: Vertex-on-edge interference detection.
//!
//! For each (vertex, edge) pair across solids, checks if the vertex
//! lies on the edge. If so, adds an extra pave to the edge's pave block.

use remus_math::aabb::Aabb3;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use crate::ds::{GfaArena, Interference, Pave};
use crate::error::AlgoError;

/// Compute a conservative AABB for an edge, expanded by `margin`.
///
/// A recorded vertex lies within the acceptance band of a point the edge
/// evaluates to, and every such point lies in the edge's conservative box.
/// A curved edge's box covers its whole carrier, so it also holds the
/// projection's 1e-10 domain slack. It gets `tol_linear` beyond `margin`
/// plus the coordinate-scaled slack for evaluation rounding, so a vertex
/// outside it can never be recorded. `gate_curved = false` keeps curved
/// edges unboxed (always projected), for the equivalence tests.
fn edge_aabb(
    topo: &Topology,
    eid: EdgeId,
    margin: f64,
    tol_linear: f64,
    gate_curved: bool,
) -> Result<Option<Aabb3>, AlgoError> {
    let edge = topo.edge(eid)?;
    let start_pos = topo.vertex(edge.start())?.point();
    let end_pos = topo.vertex(edge.end())?.point();
    Ok(match edge.curve() {
        EdgeCurve::Line => Aabb3::try_from_points([start_pos, end_pos]).map(|a| a.expanded(margin)),
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_) => gate_curved
            .then(|| super::helpers::conservative_curve_aabb(edge.curve(), start_pos, end_pos))
            .flatten()
            .map(|a| a.expanded(margin + tol_linear + super::helpers::coordinate_slack(a))),
        EdgeCurve::Parabola(_) | EdgeCurve::Hyperbola(_) => None,
    })
}

/// Detect vertices lying on edges between the two solids.
///
/// Checks vertices of A against edges of B, and vertices of B against
/// edges of A. When a vertex lies on an edge (within tolerance), an
/// extra pave is inserted into the edge's pave block for later splitting.
///
/// # Errors
///
/// Returns [`AlgoError`] if any topology lookup fails.
pub fn perform(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    perform_with(topo, solid_a, solid_b, tol, arena, true)
}

/// [`perform`], with the curved-edge broad phase switchable so tests can
/// prove it changes nothing but the work done.
pub(super) fn perform_with(
    topo: &Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
    gate_curved: bool,
) -> Result<(), AlgoError> {
    // AABB pre-filter: skip if solids are disjoint
    let bbox_a = crate::classifier::compute_solid_bbox(topo, solid_a)?;
    let bbox_b = crate::classifier::compute_solid_bbox(topo, solid_b)?;
    if !bbox_a
        .expanded(tol.linear)
        .intersects(bbox_b.expanded(tol.linear))
    {
        log::debug!("VE: solids are disjoint, skipping");
        return Ok(());
    }

    let verts_a = remus_topology::explorer::solid_vertices(topo, solid_a)?;
    let verts_b = remus_topology::explorer::solid_vertices(topo, solid_b)?;
    let edges_a = remus_topology::explorer::solid_edges(topo, solid_a)?;
    let edges_b = remus_topology::explorer::solid_edges(topo, solid_b)?;

    super::helpers::validate_edge_domains(topo, &edges_a, "vertex-edge interference")?;
    super::helpers::validate_edge_domains(topo, &edges_b, "vertex-edge interference")?;

    check_vertex_edge_pairs(topo, &verts_a, &edges_b, tol, arena, gate_curved)?;
    check_vertex_edge_pairs(topo, &verts_b, &edges_a, tol, arena, gate_curved)?;

    Ok(())
}

/// Check each vertex against each edge and record VE interferences.
#[allow(clippy::too_many_lines)]
fn check_vertex_edge_pairs(
    topo: &Topology,
    vertices: &[VertexId],
    edges: &[EdgeId],
    tol: Tolerance,
    arena: &mut GfaArena,
    gate_curved: bool,
) -> Result<(), AlgoError> {
    // Broad-phase: bound each edge once. A vertex lying on an edge is within
    // the edge's AABB, so vertices outside it are rejected before the costly
    // closest-point projection (33 samples + 20 ternary steps per pair).
    // Margin covers both the global linear tolerance and per-vertex tolerance,
    // which is added to `tol.linear` in the fine test below. Parabola and
    // hyperbola edges have no box and are always projected.
    let mut edge_data: Vec<(Option<Aabb3>, f64)> = Vec::with_capacity(edges.len());
    for &eid in edges {
        let tolerance_excess = super::helpers::edge_tolerance_excess(topo, eid, tol.linear)?;
        edge_data.push((
            edge_aabb(
                topo,
                eid,
                super::helpers::tolerance_band(tol.linear, [tolerance_excess])?,
                tol.linear,
                gate_curved,
            )?,
            tolerance_excess,
        ));
    }

    for &vid in vertices {
        let resolved_vid = arena.resolve_vertex(vid);
        let vertex = topo.vertex(resolved_vid)?;
        let pos = vertex.point();
        let vtol = vertex.tolerance();
        let _ = super::helpers::vertex_tolerance_excess(topo, resolved_vid, tol.linear)?;

        for (edge_idx, &eid) in edges.iter().enumerate() {
            // Broad-phase reject: the vertex cannot lie on an edge whose
            // (tolerance-expanded by vtol) AABB does not contain it.
            let (edge_box, edge_tolerance_excess) = &edge_data[edge_idx];
            if let Some(ebox) = edge_box
                && !ebox.expanded(vtol).contains_point(pos)
            {
                continue;
            }

            let edge = topo.edge(eid)?;

            // Skip if vertex is already an endpoint of this edge
            let start_v = arena.resolve_vertex(edge.start());
            let end_v = arena.resolve_vertex(edge.end());
            if resolved_vid == start_v || resolved_vid == end_v {
                continue;
            }

            let start_pos = topo.vertex(edge.start())?.point();
            let end_pos = topo.vertex(edge.end())?.point();
            let (t0, t1) =
                super::helpers::authoritative_edge_domain(edge, eid, "vertex-edge interference")?;

            let param = project_point_on_edge(topo, eid, pos)?;

            let domain_lo = t0.min(t1) - 1e-10;
            let domain_hi = t0.max(t1) + 1e-10;
            if param < domain_lo || param > domain_hi {
                continue;
            }

            let edge_pt = edge
                .curve()
                .evaluate_with_endpoints(param, start_pos, end_pos);
            let dist = (pos - edge_pt).length();

            let combined_tol =
                super::helpers::tolerance_band(tol.linear, [vtol, *edge_tolerance_excess])?;
            if dist <= combined_tol {
                let pave = Pave::new(resolved_vid, param);
                if let Some(pb_ids) = arena.edge_pave_blocks.get(&eid) {
                    let pb_ids_copy: Vec<_> = pb_ids.clone();
                    for pb_id in pb_ids_copy {
                        if let Some(pb) = arena.pave_blocks.get_mut(pb_id)
                            && pb.contains_parameter_interior(param, 1e-10)
                        {
                            pb.add_extra_pave(pave);
                        }
                    }
                }

                arena.interference.ve.push(Interference::VE {
                    vertex: resolved_vid,
                    edge: eid,
                    parameter: param,
                });

                log::debug!(
                    "VE: vertex {resolved_vid:?} on edge {eid:?} at t={param:.6} (dist={dist:.2e})",
                );
            }
        }
    }

    Ok(())
}

/// Project a point onto an edge curve, returning the closest parameter.
///
/// Uses coarse sampling followed by ternary search refinement for
/// robustness across all edge curve types.
fn project_point_on_edge(
    topo: &Topology,
    edge_id: EdgeId,
    point: Point3,
) -> Result<f64, AlgoError> {
    let edge = topo.edge(edge_id)?;
    let start_pos = topo.vertex(edge.start())?.point();
    let end_pos = topo.vertex(edge.end())?.point();
    let (t0, t1) =
        super::helpers::authoritative_edge_domain(edge, edge_id, "vertex-edge projection")?;

    // PERF-M04: every call here is a (vertex, edge) pair that survived the
    // AABB broad-phase and endpoint rejection. Count the generic-path
    // projection and each curve evaluation inside it. There is no analytic
    // `Line` fast path (PERF-B01 closed as a measured no-gain experiment).
    crate::perf::bump_ve_sampled_projection();
    let project = |t: f64| -> Point3 {
        crate::perf::bump_ve_projection_eval();
        edge.curve().evaluate_with_endpoints(t, start_pos, end_pos)
    };

    let n_samples: usize = 32;
    let mut best_t = t0;
    let mut best_dist_sq = f64::MAX;

    for i in 0..=n_samples {
        let t = t0 + (t1 - t0) * (i as f64 / n_samples as f64);
        let pt = project(t);
        let d_sq = (point - pt).length_squared();
        if d_sq < best_dist_sq {
            best_dist_sq = d_sq;
            best_t = t;
        }
    }

    let dt = (t1 - t0).abs() / n_samples as f64;
    let domain_lo = t0.min(t1);
    let domain_hi = t0.max(t1);
    let mut lo = (best_t - dt).max(domain_lo);
    let mut hi = (best_t + dt).min(domain_hi);

    for _ in 0..20 {
        let m1 = lo + (hi - lo) / 3.0;
        let m2 = hi - (hi - lo) / 3.0;
        let d1 = (point - project(m1)).length_squared();
        let d2 = (point - project(m2)).length_squared();
        if d1 < d2 {
            hi = m2;
        } else {
            lo = m1;
        }
    }

    Ok(f64::midpoint(lo, hi))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;

    use super::*;

    #[test]
    fn reversed_circle_accepts_and_records_an_interior_vertex() {
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            2.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let (t0, t1) = (2.5, 0.5);
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(circle.evaluate(t0), 1e-7));
        let end = topo.add_vertex(Vertex::new(circle.evaluate(t1), 1e-7));
        let interior_t = 1.5;
        let interior = topo.add_vertex(Vertex::new(circle.evaluate(interior_t), 1e-4));
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some((t0, t1)));
        let edge_id = topo.add_edge(edge);

        let projected =
            project_point_on_edge(&topo, edge_id, topo.vertex(interior).unwrap().point()).unwrap();
        assert!(
            (projected - interior_t).abs() < 1e-5,
            "projected parameter {projected}"
        );

        let mut arena = GfaArena::new();
        arena.init_edge_pave_block(edge_id, start, t0, end, t1);
        check_vertex_edge_pairs(
            &topo,
            &[interior],
            &[edge_id],
            Tolerance::default(),
            &mut arena,
            true,
        )
        .unwrap();

        assert_eq!(arena.interference.ve.len(), 1);
        let pb_id = arena.edge_pave_blocks[&edge_id][0];
        let pb = arena.pave_blocks.get(pb_id).unwrap();
        assert_eq!(pb.extra_paves.len(), 1);
        assert!((pb.extra_paves[0].parameter - interior_t).abs() < 1e-5);
    }

    #[test]
    fn declared_edge_tube_widens_vertex_edge_incidence() {
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let tested = topo.add_vertex(Vertex::new(Point3::new(0.5, 5e-5, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::with_tolerance(
            start,
            end,
            EdgeCurve::Line,
            Some(1e-4),
        ));
        let mut arena = GfaArena::new();
        arena.init_edge_pave_block(edge, start, 0.0, end, 1.0);

        check_vertex_edge_pairs(
            &topo,
            &[tested],
            &[edge],
            Tolerance::default(),
            &mut arena,
            true,
        )
        .unwrap();

        assert_eq!(arena.interference.ve.len(), 1);
    }

    #[test]
    fn vertex_edge_incidence_stays_narrow_without_declared_tube_excess() {
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let tested = topo.add_vertex(Vertex::new(Point3::new(0.5, 5e-5, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(start, end, EdgeCurve::Line));
        let mut arena = GfaArena::new();
        arena.init_edge_pave_block(edge, start, 0.0, end, 1.0);

        check_vertex_edge_pairs(
            &topo,
            &[tested],
            &[edge],
            Tolerance::default(),
            &mut arena,
            true,
        )
        .unwrap();

        assert!(arena.interference.ve.is_empty());
    }

    #[test]
    fn invalid_declared_edge_tolerance_is_refused() {
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let tested = topo.add_vertex(Vertex::new(Point3::new(0.5, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::with_tolerance(
            start,
            end,
            EdgeCurve::Line,
            Some(f64::NAN),
        ));
        let mut arena = GfaArena::new();
        arena.init_edge_pave_block(edge, start, 0.0, end, 1.0);

        let err = check_vertex_edge_pairs(
            &topo,
            &[tested],
            &[edge],
            Tolerance::default(),
            &mut arena,
            true,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            AlgoError::Topology(remus_topology::TopologyError::InvalidToleranceValue {
                entity: "edge",
                ..
            })
        ));
    }

    /// Everything VE records, with parameters as bits: the interference list
    /// in order and every pave block's extra paves in order.
    type VeRecord = (Vec<(VertexId, EdgeId, u64)>, Vec<Vec<(VertexId, u64)>>);

    fn ve_record(topo: &Topology, vertices: &[VertexId], edges: &[EdgeId], gate: bool) -> VeRecord {
        let mut arena = GfaArena::new();
        for &eid in edges {
            let edge = topo.edge(eid).unwrap();
            let (t0, t1) = edge.strict_domain().unwrap();
            arena.init_edge_pave_block(eid, edge.start(), t0, edge.end(), t1);
        }
        check_vertex_edge_pairs(
            topo,
            vertices,
            edges,
            Tolerance::default(),
            &mut arena,
            gate,
        )
        .unwrap();
        let ve = arena
            .interference
            .ve
            .iter()
            .map(|i| match i {
                Interference::VE {
                    vertex,
                    edge,
                    parameter,
                } => (*vertex, *edge, parameter.to_bits()),
                other => panic!("VE recorded {other:?}"),
            })
            .collect();
        let paves = edges
            .iter()
            .flat_map(|e| arena.edge_pave_blocks[e].clone())
            .map(|pb| {
                let pb = arena.pave_blocks.get(pb).unwrap();
                pb.extra_paves
                    .iter()
                    .map(|p| (p.vertex, p.parameter.to_bits()))
                    .collect()
            })
            .collect();
        (ve, paves)
    }

    /// The curved-edge box only skips work: against circles, arcs, ellipses
    /// and a rational NURBS, with frames tilted 1e-8 off a world axis or
    /// skewed (non-unit, non-orthogonal `with_axes`), and far from the
    /// origin, the gated pass records exactly what projecting every pair
    /// records. Each edge is probed with on-curve vertices (including the
    /// out-of-plane extremes a normal-based box would cut off), vertices
    /// half a tolerance off, and vertices well clear of it, which the box
    /// must prune for the test to mean anything.
    #[test]
    #[allow(clippy::too_many_lines)]
    fn curved_edge_broad_phase_records_exactly_what_projection_records() {
        use remus_math::curves::Ellipse3D;
        use remus_math::nurbs::curve::NurbsCurve;
        use std::f64::consts::{FRAC_1_SQRT_2, TAU};

        let z = Vec3::new(0.0, 0.0, 1.0);
        let tilted = |tilt: f64| Vec3::new(tilt, 0.0, 1.0).normalize().unwrap();
        let slanted = Vec3::new(FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
        let mut cases: Vec<(EdgeCurve, f64, f64, f64)> = Vec::new();
        for offset in [0.0, 1e6, 1e13] {
            let center = Point3::new(1.0 + offset, -offset, 2.0);
            // Vertex tolerance wide enough to accept at the offset's ulp.
            let vtol = if offset > 1e9 { 1e-2 } else { 1e-7 };
            let circle = |n: Vec3, r: f64| Circle3D::new(center, n, r).unwrap();
            cases.push((EdgeCurve::Circle(circle(z, 2.0)), 0.0, TAU, vtol));
            cases.push((EdgeCurve::Circle(circle(z, 2.0)), 2.5, 0.5, vtol));
            cases.push((
                EdgeCurve::Circle(circle(tilted(1e-8), 100.0)),
                0.0,
                TAU,
                vtol,
            ));
            cases.push((
                EdgeCurve::Circle(circle(tilted(1e-8), 1000.0)),
                0.0,
                TAU,
                vtol,
            ));
            cases.push((
                EdgeCurve::Circle(circle(tilted(1e-9), 1000.0)),
                0.0,
                TAU,
                vtol,
            ));
            let skewed = Circle3D::with_axes(center, z, 2.0, slanted, z.cross(slanted)).unwrap();
            cases.push((EdgeCurve::Circle(skewed), 0.0, TAU, vtol));
            let long = Vec3::new(2.0, 0.0, 0.0);
            let stretched = Circle3D::with_axes(center, z, 2.0, long, z.cross(slanted)).unwrap();
            cases.push((EdgeCurve::Circle(stretched), 0.0, TAU, vtol));
            let ellipse = Ellipse3D::new(center, Vec3::new(0.0, 1.0, 0.0), 3.0, 1.5).unwrap();
            cases.push((EdgeCurve::Ellipse(ellipse), 0.2, 4.0, vtol));
            let nurbs = NurbsCurve::new(
                3,
                vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
                [(0.0, 0.0), (1.0, 3.0), (2.0, -3.0), (3.0, 3.0), (4.0, 0.0)]
                    .iter()
                    .map(|&(x, y)| Point3::new(center.x() + x, center.y() + y, x - 2.0))
                    .collect(),
                vec![1.0, 2.0, 0.5, 2.0, 1.0],
            )
            .unwrap();
            cases.push((EdgeCurve::NurbsCurve(nurbs), 0.0, 1.0, vtol));
        }

        let mut recorded = 0;
        let mut pruned = 0;
        for (curve, t0, t1, vtol) in cases {
            let mut topo = Topology::new();
            let zero = Point3::new(0.0, 0.0, 0.0);
            let at = |t: f64| curve.evaluate_with_endpoints(t, zero, zero);
            let start = topo.add_vertex(Vertex::new(at(t0), 1e-7));
            let end = topo.add_vertex(Vertex::new(at(t1), 1e-7));
            let mut edge = Edge::new(start, end, curve.clone());
            edge.set_trim(Some((t0, t1)));
            let edge = topo.add_edge(edge);

            let mut probes = Vec::new();
            for k in 1..8 {
                let t = t0 + (t1 - t0) * f64::from(k) / 8.0;
                probes.push(at(t));
                probes.push(at(t) + Vec3::new(0.0, 0.0, 5e-8));
                probes.push(at(t) + Vec3::new(0.0, 0.0, 1e-3));
                probes.push(at(t) + Vec3::new(30.0, 0.0, 0.0));
            }
            // The carrier's out-of-plane extremes, where a box sized from the
            // normal alone was flat.
            if let EdgeCurve::Circle(c) = &curve {
                let top = c.v_axis().z().atan2(c.u_axis().z());
                probes.push(c.evaluate(top));
                probes.push(c.evaluate(top + std::f64::consts::PI));
            }
            let vertices: Vec<_> = probes
                .into_iter()
                .map(|p| topo.add_vertex(Vertex::new(p, vtol)))
                .collect();

            let gated = ve_record(&topo, &vertices, &[edge], true);
            let ungated = ve_record(&topo, &vertices, &[edge], false);
            assert_eq!(gated, ungated, "{curve:?} over ({t0}, {t1})");
            recorded += gated.0.len();

            let margin = Tolerance::default().linear;
            let ebox = edge_aabb(&topo, edge, margin, margin, true)
                .unwrap()
                .expect("curved edges are boxed");
            pruned += vertices
                .iter()
                .filter(|&&v| {
                    !ebox
                        .expanded(vtol)
                        .contains_point(topo.vertex(v).unwrap().point())
                })
                .count();
        }
        assert!(recorded > 0, "no case recorded a vertex");
        assert!(pruned > 0, "no case exercised the curved-edge box");
    }
}
