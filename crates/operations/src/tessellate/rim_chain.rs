//! Recognition of endpoint-connected full-turn rim chains.

use std::collections::HashMap;
use std::f64::consts::TAU;

use brepkit_math::vec::Point3;
use brepkit_topology::Topology;
use brepkit_topology::vertex::VertexId;

/// One endpoint-connected curved-edge cycle that winds a periodic parameter.
pub struct RimCycle {
    /// Raw topology indices of the edges in traversal order.
    pub edge_indices: Vec<usize>,
    /// Whether the cycle contains a by-construction closed edge.
    pub has_closed_edge: bool,
}

/// Collect curved-edge cycles whose projected parameter winds one full turn.
///
/// `curved` contains unique topology edge indices and their stored endpoints.
/// The recognizer walks each connected run by endpoint identity, rejects open
/// runs or an unexpected cycle count, and then requires every non-closed cycle
/// to accumulate a wrapped winding of approximately `2*pi`.
pub fn collect_full_turn_rim_cycles(
    topo: &Topology,
    curved: &[(usize, VertexId, VertexId)],
    project_u: &dyn Fn(Point3) -> f64,
    expected_cycles: usize,
) -> Result<Option<Vec<RimCycle>>, crate::OperationsError> {
    let cycles = collect_full_turn_rim_cycles_any(topo, curved, project_u)?;
    Ok(cycles.filter(|cycles| cycles.len() == expected_cycles))
}

/// Collect all full-turn rim cycles without requiring a particular count.
pub fn collect_full_turn_rim_cycles_any(
    topo: &Topology,
    curved: &[(usize, VertexId, VertexId)],
    project_u: &dyn Fn(Point3) -> f64,
) -> Result<Option<Vec<RimCycle>>, crate::OperationsError> {
    let mut by_vertex: HashMap<VertexId, Vec<usize>> = HashMap::new();
    for (position, &(_, start, end)) in curved.iter().enumerate() {
        by_vertex.entry(start).or_default().push(position);
        by_vertex.entry(end).or_default().push(position);
    }
    if by_vertex.values().any(|positions| positions.len() != 2) {
        return Ok(None);
    }

    let mut used = vec![false; curved.len()];
    let mut cycles = Vec::new();
    for start_position in 0..curved.len() {
        if used[start_position] {
            continue;
        }

        let (_, origin, mut at) = curved[start_position];
        used[start_position] = true;
        let mut positions = vec![start_position];
        let mut closed = curved[start_position].1 == curved[start_position].2 || at == origin;
        while !closed {
            let Some(&next) = by_vertex
                .get(&at)
                .and_then(|candidates| candidates.iter().find(|&&position| !used[position]))
            else {
                break;
            };
            used[next] = true;
            at = if curved[next].1 == at {
                curved[next].2
            } else {
                curved[next].1
            };
            positions.push(next);
            closed = at == origin;
        }
        if !closed {
            return Ok(None);
        }

        let mut winding = 0.0_f64;
        let mut traversal_vertex: Option<VertexId> = None;
        let mut has_closed_edge = false;
        for &position in &positions {
            let (edge_index, start, end) = curved[position];
            if start == end {
                has_closed_edge = true;
                continue;
            }
            let (from, to) = match traversal_vertex {
                None => (start, end),
                Some(vertex) if vertex == start => (start, end),
                Some(vertex) if vertex == end => (end, start),
                Some(_) => return Ok(None),
            };
            let Some(edge_id) = topo.edge_id_from_index(edge_index) else {
                return Ok(None);
            };
            let edge = topo.edge(edge_id)?;
            if edge.start() != start || edge.end() != end {
                return Ok(None);
            }
            let stored_start = topo.vertex(start)?.point();
            let stored_end = topo.vertex(end)?.point();
            let (t0, t1) = edge.curve().domain_with_endpoints(stored_start, stored_end);
            let midpoint =
                edge.curve()
                    .evaluate_with_endpoints((t0 + t1) * 0.5, stored_start, stored_end);
            let u0 = project_u(topo.vertex(from)?.point());
            let u1 = project_u(topo.vertex(to)?.point());
            let u_mid = project_u(midpoint);
            if !u0.is_finite() || !u1.is_finite() || !u_mid.is_finite() {
                return Ok(None);
            }
            winding += wrap_pi(u_mid - u0) + wrap_pi(u1 - u_mid);
            traversal_vertex = Some(to);
        }
        if !has_closed_edge && (!winding.is_finite() || (winding.abs() - TAU).abs() > 1e-6) {
            return Ok(None);
        }

        cycles.push(RimCycle {
            edge_indices: positions
                .into_iter()
                .map(|position| curved[position].0)
                .collect(),
            has_closed_edge,
        });
    }

    Ok(Some(cycles))
}

fn wrap_pi(delta: f64) -> f64 {
    (delta + TAU / 2.0).rem_euclid(TAU) - TAU / 2.0
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use brepkit_math::curves::Circle3D;
    use brepkit_math::vec::Vec3;
    use brepkit_topology::Topology;
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::vertex::Vertex;

    use super::*;

    #[test]
    fn major_and_minor_arc_pair_is_a_full_turn() {
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let split = topo.add_vertex(Vertex::new(
            circle.evaluate(1.5 * std::f64::consts::PI),
            1e-7,
        ));
        let major = topo.add_edge(Edge::new(start, split, EdgeCurve::Circle(circle.clone())));
        let minor = topo.add_edge(Edge::new(split, start, EdgeCurve::Circle(circle.clone())));
        let curved = [(major.index(), start, split), (minor.index(), split, start)];

        let cycles =
            collect_full_turn_rim_cycles(&topo, &curved, &|point| circle.project(point), 1)
                .unwrap();

        assert!(cycles.is_some());
    }

    #[test]
    fn branched_arc_graph_is_not_a_rim_cycle() {
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(circle.evaluate(std::f64::consts::PI), 1e-7));
        let c = topo.add_vertex(Vertex::new(
            circle.evaluate(std::f64::consts::FRAC_PI_2),
            1e-7,
        ));
        let ab = topo.add_edge(Edge::new(a, b, EdgeCurve::Circle(circle.clone())));
        let ba = topo.add_edge(Edge::new(b, a, EdgeCurve::Circle(circle.clone())));
        let ac = topo.add_edge(Edge::new(a, c, EdgeCurve::Circle(circle.clone())));
        let ca = topo.add_edge(Edge::new(c, a, EdgeCurve::Circle(circle.clone())));
        let curved = [
            (ab.index(), a, b),
            (ba.index(), b, a),
            (ac.index(), a, c),
            (ca.index(), c, a),
        ];

        let cycles =
            collect_full_turn_rim_cycles(&topo, &curved, &|point| circle.project(point), 2)
                .unwrap();

        assert!(cycles.is_none());
    }

    #[test]
    fn non_finite_projection_is_not_a_rim_cycle() {
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let split = topo.add_vertex(Vertex::new(circle.evaluate(std::f64::consts::PI), 1e-7));
        let first = topo.add_edge(Edge::new(start, split, EdgeCurve::Circle(circle.clone())));
        let second = topo.add_edge(Edge::new(split, start, EdgeCurve::Circle(circle)));
        let curved = [
            (first.index(), start, split),
            (second.index(), split, start),
        ];

        let cycles = collect_full_turn_rim_cycles(&topo, &curved, &|_| f64::NAN, 1).unwrap();

        assert!(cycles.is_none());
    }
}
