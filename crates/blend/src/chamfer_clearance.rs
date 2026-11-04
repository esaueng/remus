//! Whole-carrier hole clearance for newly qualified spline chamfers.

use crate::BlendError;
use remus_geometry::bounds::{FiniteBound, curve};
use remus_math::{aabb::Aabb3, compact_sort::CompactSort, tolerance::Tolerance};
use remus_topology::{
    Topology,
    adjacency::AdjacencyIndex,
    edge::{EdgeCurve, EdgeId},
};

fn edge_bound(topo: &Topology, id: EdgeId) -> Result<FiniteBound, BlendError> {
    let edge = topo.edge(id)?;
    let (a, b) = edge.strict_domain().map_err(|e| BlendError::InvalidInput {
        reason: format!("chamfer obstacle lacks parameter authority: {e}"),
    })?;
    Ok(match edge.curve() {
        EdgeCurve::Line => curve::line_segment_bounds(
            topo.vertex(edge.start())?.point(),
            topo.vertex(edge.end())?.point(),
        ),
        EdgeCurve::Circle(c) => curve::circle_arc_bounds(c, a, b),
        EdgeCurve::Ellipse(c) => curve::ellipse_arc_bounds(c, a, b),
        EdgeCurve::Parabola(c) => curve::parabola_arc_bounds(c, a, b),
        EdgeCurve::Hyperbola(c) => curve::hyperbola_arc_bounds(c, a, b),
        EdgeCurve::NurbsCurve(c) => curve::nurbs_curve_bounds(c, a, b),
    })
}

/// Every inner boundary must be disjoint from a box containing all material
/// the candidate can change. Unknown bounds refuse instead of using samples.
pub fn holes_clear(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    region: Aabb3,
) -> Result<bool, BlendError> {
    let mut faces = adjacency
        .edge_faces_iter()
        .flat_map(|(_, faces)| faces.iter().copied())
        .collect::<Vec<_>>();
    faces.compact_sort_unstable();
    faces.dedup();
    let scale = region
        .min
        .0
        .iter()
        .chain(&region.max.0)
        .fold(1.0_f64, |s, v| s.max(v.abs()));
    let margin = Tolerance::new().linear + 128.0 * f64::EPSILON * scale;
    let region = region.expanded(margin);
    for face in faces {
        for &wire in topo.face(face)?.inner_wires() {
            for usage in topo.wire(wire)?.edges() {
                let bound = edge_bound(topo, usage.edge())?;
                if !bound.is_prunable() || bound.aabb().expanded(margin).intersects(region) {
                    return Ok(false);
                }
            }
        }
    }
    Ok(true)
}
