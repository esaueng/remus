//! Phase VV: Vertex-vertex coincidence detection.
//!
//! Finds all vertex pairs (one from each solid) that are spatially
//! coincident within tolerance. Merges them via same-domain mapping.

use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::ds::{GfaArena, Interference};
use crate::error::AlgoError;

/// Detect coincident vertices between solid A and solid B.
///
/// For every `(va, vb)` pair where `va` belongs to `solid_a` and `vb` to
/// `solid_b`, check if they are within combined tolerance. Coincident
/// pairs are recorded as VV interferences and merged in the same-domain
/// vertex map.
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

    for &va in &verts_a {
        let vertex_a = topo.vertex(va)?;
        let pos_a = vertex_a.point();
        let tol_a = vertex_a.tolerance();

        for &vb in &verts_b {
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

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::{Solid, SolidId};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

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
}
