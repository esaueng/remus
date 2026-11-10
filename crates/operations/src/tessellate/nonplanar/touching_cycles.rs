//! One chart boundary for a torus face whose wires touch its outer wire.
//!
//! The chart CDT reads a face as one outer polygon and strictly interior
//! hole polygons. A torus face can hold the same region as several closed
//! wires that meet at vertices instead: when two faces split along the
//! torus's meridian seam are merged (face unification after a Boolean), the
//! merged face keeps the equator seam, and its cycles close wherever a trace
//! returns to its start. The two band boundaries each wind the tube period,
//! and the seam segments that joined them become zero-area slits (one edge
//! traversed both ways) touching those loops at their shared vertices. Every
//! piece is valid topology, but no single one is a chart polygon: a loop
//! winding a period cannot lie inside the outer one, and a slit has no
//! interior.
//!
//! A cycle that shares a vertex with the outer boundary is not a hole in the
//! chart sense: a hole polygon must lie strictly inside the outer one, and
//! that vertex is on it. Two closed walks through a common vertex join into
//! one closed walk over the same oriented edges, so the material on their
//! left is unchanged. Splicing each touching cycle into the outer wire at
//! the vertex it shares therefore describes the same face as one boundary
//! walk; on the merged band that walk is the cut-open chart's polygon, the
//! walk the two faces had before they were merged. The caller still lifts
//! that walk and refuses unless it closes in both periods, so a cycle that
//! genuinely winds a period and touches nothing keeps the original refusal.

use remus_topology::Topology;
use remus_topology::face::Face;
use remus_topology::vertex::VertexId;
use remus_topology::wire::OrientedEdge;

/// The face's boundary as the chart CDT consumes it.
#[derive(Debug)]
pub(super) struct ChartWires {
    /// The outer walk, with every touching cycle spliced in.
    pub(super) outer: Vec<OrientedEdge>,
    /// The cycles that share no vertex with the outer walk, in face order.
    pub(super) holes: Vec<Vec<OrientedEdge>>,
    /// Whether any cycle was spliced into the outer walk.
    pub(super) spliced: bool,
}

/// Reads `face`'s wires, splicing inner cycles that share a vertex with the
/// outer walk into it when `splice` is set (torus faces); otherwise the
/// wires are returned exactly as stored.
pub(super) fn chart_wires(
    topo: &Topology,
    face: &Face,
    splice: bool,
) -> Result<ChartWires, crate::OperationsError> {
    let mut outer = topo.wire(face.outer_wire())?.edges().to_vec();
    let mut holes = face
        .inner_wires()
        .iter()
        .map(|&wire| Ok(topo.wire(wire)?.edges().to_vec()))
        .collect::<Result<Vec<_>, crate::OperationsError>>()?;
    let mut spliced = false;
    if splice {
        // Each pass removes one hole, so the loop is bounded by the hole
        // count. Only stored wire edges are scanned, never mesh samples.
        while let Some((hole_index, outer_at, hole_at)) = touching(topo, &outer, &holes)? {
            let mut cycle = holes.remove(hole_index);
            cycle.rotate_left(hole_at);
            outer.splice(outer_at..outer_at, cycle);
            spliced = true;
        }
    }
    Ok(ChartWires {
        outer,
        holes,
        spliced,
    })
}

/// The first hole (in face order) with an edge starting at a vertex where an
/// outer edge starts: `(hole, outer edge index, hole edge index)`. The hole
/// rotated to that edge is a closed walk from the vertex the preceding outer
/// edge ends at, so it can be inserted before that outer edge.
fn touching(
    topo: &Topology,
    outer: &[OrientedEdge],
    holes: &[Vec<OrientedEdge>],
) -> Result<Option<(usize, usize, usize)>, crate::OperationsError> {
    let outer_starts = starts(topo, outer)?;
    for (hole_index, hole) in holes.iter().enumerate() {
        for (hole_at, vertex) in starts(topo, hole)?.into_iter().enumerate() {
            if let Some(outer_at) = outer_starts.iter().position(|&start| start == vertex) {
                return Ok(Some((hole_index, outer_at, hole_at)));
            }
        }
    }
    Ok(None)
}

fn starts(topo: &Topology, wire: &[OrientedEdge]) -> Result<Vec<VertexId>, crate::OperationsError> {
    wire.iter()
        .map(|oriented| Ok(oriented.oriented_start(topo.edge(oriented.edge())?)))
        .collect()
}
