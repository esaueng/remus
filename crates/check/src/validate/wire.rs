//! Wire validation checks.

use std::collections::HashMap;

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::wire::WireId;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

/// Check that a wire is not empty.
pub fn check_wire_empty(
    topo: &Topology,
    wire_id: WireId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let wire = topo.wire(wire_id)?;
    if wire.edges().is_empty() {
        return Ok(vec![ValidationIssue {
            check: CheckId::WireEmpty,
            severity: Severity::Error,
            entity: EntityRef::Wire(wire_id),
            description: "wire contains no edges".into(),
            deviation: None,
        }]);
    }
    Ok(vec![])
}

/// Check that consecutive edges share vertices.
pub fn check_wire_connected(
    topo: &Topology,
    wire_id: WireId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let edges = wire.edges();
    if edges.len() < 2 {
        return Ok(vec![]);
    }

    let mut issues = Vec::new();
    for i in 0..edges.len() - 1 {
        let edge_a = topo.edge(edges[i].edge())?;
        let edge_b = topo.edge(edges[i + 1].edge())?;
        let end_a = edges[i].oriented_end(edge_a);
        let start_b = edges[i + 1].oriented_start(edge_b);
        if end_a != start_b {
            issues.push(ValidationIssue {
                check: CheckId::WireNotConnected,
                severity: Severity::Error,
                entity: EntityRef::Wire(wire_id),
                description: format!("edges {} and {} not connected", i, i + 1),
                deviation: None,
            });
        }
    }
    Ok(issues)
}

/// Check 3D wire closure (last edge end == first edge start).
pub fn check_wire_closure(
    topo: &Topology,
    wire_id: WireId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let wire = topo.wire(wire_id)?;
    if !wire.is_closed() {
        return Ok(vec![]);
    }
    let edges = wire.edges();
    if edges.is_empty() {
        return Ok(vec![]);
    }

    let first_edge = topo.edge(edges[0].edge())?;
    let last_edge = topo.edge(edges[edges.len() - 1].edge())?;
    let first_start = edges[0].oriented_start(first_edge);
    let last_end = edges[edges.len() - 1].oriented_end(last_edge);

    if first_start != last_end {
        return Ok(vec![ValidationIssue {
            check: CheckId::WireClosure3D,
            severity: Severity::Error,
            entity: EntityRef::Wire(wire_id),
            description: "wire not closed: last edge end != first edge start".into(),
            deviation: None,
        }]);
    }
    Ok(vec![])
}

/// Check for edges appearing 3+ times in same wire.
pub fn check_wire_redundant(
    topo: &Topology,
    wire_id: WireId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut counts: HashMap<_, usize> = HashMap::new();
    for oe in wire.edges() {
        *counts.entry(oe.edge()).or_default() += 1;
    }
    let mut issues = Vec::new();
    for (eid, count) in counts {
        if count >= 3 {
            issues.push(ValidationIssue {
                check: CheckId::WireRedundantEdge,
                severity: Severity::Error,
                entity: EntityRef::Edge(eid),
                description: format!("edge appears {count} times in wire"),
                deviation: None,
            });
        }
    }
    Ok(issues)
}

/// Check for wire self-intersection by sampling edges and testing for crossings.
///
/// Samples each edge at 8 points and checks for segment-segment crossings
/// between non-adjacent edge pairs.
///
/// # Errors
///
/// Returns an error if the wire or one of its referenced topology entities
/// cannot be read.
#[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
pub fn check_wire_self_intersection(
    topo: &Topology,
    wire_id: WireId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    check_wire_self_intersection_impl(topo, wire_id, tolerance, false)
}

/// Check a periodic-surface wire while exempting only its duplicated seam.
///
/// # Errors
///
/// Returns an error if the wire or one of its referenced topology entities
/// cannot be read.
pub fn check_wire_self_intersection_on_periodic_surface(
    topo: &Topology,
    wire_id: WireId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    check_wire_self_intersection_impl(topo, wire_id, tolerance, true)
}

fn check_wire_self_intersection_impl(
    topo: &Topology,
    wire_id: WireId,
    tolerance: f64,
    periodic_surface: bool,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let edges = wire.edges();
    if edges.len() < 3 {
        return Ok(vec![]);
    }

    let samples_per_edge = 8usize;
    let mut edge_segments: Vec<Vec<Point3>> = Vec::new();
    let mut edge_use_count: HashMap<_, usize> = HashMap::new();
    for oriented in edges {
        *edge_use_count.entry(oriented.edge()).or_default() += 1;
    }

    for oe in edges {
        let edge = topo.edge(oe.edge())?;
        let p0 = topo.vertex(edge.start())?.point();
        let p1 = topo.vertex(edge.end())?.point();

        match edge.curve() {
            remus_topology::edge::EdgeCurve::Line => {
                edge_segments.push(vec![p0, p1]);
            }
            remus_topology::edge::EdgeCurve::Circle(c) => {
                let (t0, t1) = if let Some(trim) = edge.trim() {
                    trim
                } else if edge.is_closed() {
                    let start = c.project(p0);
                    (start, start + std::f64::consts::TAU)
                } else {
                    // Legacy imported arcs may not carry an explicit trim.
                    // Match tessellation's authoritative convention and walk
                    // the shorter endpoint-bounded arc; the complementary
                    // major arc can cross unrelated wire edges and report a
                    // false self-intersection.
                    let start = c.project(p0);
                    let forward = (c.project(p1) - start).rem_euclid(std::f64::consts::TAU);
                    if forward <= std::f64::consts::PI {
                        (start, start + forward)
                    } else {
                        (start, start - (std::f64::consts::TAU - forward))
                    }
                };
                let mut pts = Vec::with_capacity(samples_per_edge + 1);
                for k in 0..=samples_per_edge {
                    let t = t0 + (t1 - t0) * (k as f64) / (samples_per_edge as f64);
                    pts.push(c.evaluate(t));
                }
                if !oe.is_forward() {
                    pts.reverse();
                }
                edge_segments.push(pts);
            }
            remus_topology::edge::EdgeCurve::Ellipse(e) => {
                let is_closed = edge.start() == edge.end();
                let (t0, t1) = if is_closed {
                    (0.0, std::f64::consts::TAU)
                } else {
                    let mut ta = e.project(p0);
                    let mut tb = e.project(p1);
                    if !oe.is_forward() {
                        std::mem::swap(&mut ta, &mut tb);
                    }
                    if tb <= ta {
                        tb += std::f64::consts::TAU;
                    }
                    (ta, tb)
                };
                let mut pts = Vec::with_capacity(samples_per_edge + 1);
                for k in 0..=samples_per_edge {
                    let t = t0 + (t1 - t0) * (k as f64) / (samples_per_edge as f64);
                    pts.push(e.evaluate(t));
                }
                if !oe.is_forward() {
                    pts.reverse();
                }
                edge_segments.push(pts);
            }
            remus_topology::edge::EdgeCurve::Hyperbola(h) => {
                // Unbounded branch: the vertices are the only trim, and
                // `project` inverts the parameterization exactly, so the
                // sub-arc is the straight parameter interval — no
                // wrap-around correction as for the periodic conics.
                let (ta, tb) = (h.project(p0), h.project(p1));
                let mut pts = Vec::with_capacity(samples_per_edge + 1);
                for k in 0..=samples_per_edge {
                    let t = ta + (tb - ta) * (k as f64) / (samples_per_edge as f64);
                    pts.push(h.evaluate(t));
                }
                if !oe.is_forward() {
                    pts.reverse();
                }
                edge_segments.push(pts);
            }
            remus_topology::edge::EdgeCurve::Parabola(p) => {
                let (ta, tb) = (p.project(p0), p.project(p1));
                let mut pts = Vec::with_capacity(samples_per_edge + 1);
                for k in 0..=samples_per_edge {
                    let t = ta + (tb - ta) * (k as f64) / (samples_per_edge as f64);
                    pts.push(p.evaluate(t));
                }
                if !oe.is_forward() {
                    pts.reverse();
                }
                edge_segments.push(pts);
            }
            remus_topology::edge::EdgeCurve::NurbsCurve(nc) => {
                let (t0, t1) = nc.domain();
                let mut pts = Vec::with_capacity(samples_per_edge + 1);
                for k in 0..=samples_per_edge {
                    let t = t0 + (t1 - t0) * (k as f64) / (samples_per_edge as f64);
                    pts.push(nc.evaluate(t));
                }
                if !oe.is_forward() {
                    pts.reverse();
                }
                edge_segments.push(pts);
            }
        }
    }

    let (groups, group_count) = collinear_boundary_groups(topo, wire_id)?;
    let n_edges = edge_segments.len();
    for i in 0..n_edges {
        for j in (i + 2)..n_edges {
            // Skip adjacent edges (first and last are also adjacent in a closed wire).
            let separation = groups[i].abs_diff(groups[j]);
            if separation <= 1 || separation + 1 == group_count {
                continue;
            }
            // A periodic band may reuse exactly one seam in opposite
            // directions. No other duplicate-edge contact is exempt.
            if periodic_surface
                && edges[i].edge() == edges[j].edge()
                && edge_use_count[&edges[i].edge()] == 2
                && edges[i].is_forward() != edges[j].is_forward()
            {
                continue;
            }

            let edge_i = topo.edge(edges[i].edge())?;
            let edge_j = topo.edge(edges[j].edge())?;
            let shared_periodic_vertex = if periodic_surface {
                [edge_i.start(), edge_i.end()]
                    .into_iter()
                    .find(|vertex| *vertex == edge_j.start() || *vertex == edge_j.end())
            } else {
                None
            };

            for si in 0..edge_segments[i].len().saturating_sub(1) {
                let a0 = edge_segments[i][si];
                let a1 = edge_segments[i][si + 1];
                for sj in 0..edge_segments[j].len().saturating_sub(1) {
                    let b0 = edge_segments[j][sj];
                    let b1 = edge_segments[j][sj + 1];

                    let (dist, closest_i, closest_j) =
                        crate::distance::edge::segment_segment_distance(a0, a1, b0, b1);
                    if dist < tolerance {
                        // Periodic parameterizations may split a boundary at
                        // their declared seam vertex. Exempt only the exact
                        // topological endpoint contact, never an interior
                        // crossing or a coincident unshared edge.
                        if let Some(vertex) = shared_periodic_vertex {
                            let point = topo.vertex(vertex)?.point();
                            if (closest_i - point).length() < tolerance
                                && (closest_j - point).length() < tolerance
                            {
                                continue;
                            }
                        }
                        return Ok(vec![ValidationIssue {
                            check: CheckId::WireSelfIntersection,
                            severity: Severity::Error,
                            entity: EntityRef::Wire(wire_id),
                            description: format!(
                                "wire self-intersection between edges {i} and {j}"
                            ),
                            deviation: Some(dist),
                        }]);
                    }
                }
            }
        }
    }

    Ok(vec![])
}

/// Treat monotone subdivisions of one straight boundary as one logical edge.
/// This preserves geometric adjacency without changing the wire or its check
/// tolerance. Only shared vertices collinear to floating-point roundoff qualify.
fn collinear_boundary_groups(
    topo: &Topology,
    wire_id: WireId,
) -> Result<(Vec<usize>, usize), CheckError> {
    use remus_topology::edge::EdgeCurve;
    fn root(parent: &[usize], mut i: usize) -> usize {
        while parent[i] != i {
            i = parent[i];
        }
        i
    }
    let edges = topo.wire(wire_id)?.edges();
    let mut parent: Vec<usize> = (0..edges.len()).collect();
    for i in 0..edges.len() {
        let j = (i + 1) % edges.len();
        let a = topo.edge(edges[i].edge())?;
        let b = topo.edge(edges[j].edge())?;
        if !matches!(a.curve(), EdgeCurve::Line)
            || !matches!(b.curve(), EdgeCurve::Line)
            || edges[i].oriented_end(a) != edges[j].oriented_start(b)
        {
            continue;
        }
        let start = topo.vertex(edges[i].oriented_start(a))?.point();
        let mid = topo.vertex(edges[i].oriented_end(a))?.point();
        let end = topo.vertex(edges[j].oriented_end(b))?.point();
        let direction = end - start;
        let length2 = direction.length_squared();
        if !length2.is_finite() || length2 <= 0.0 {
            continue;
        }
        let fraction = (mid - start).dot(direction) / length2;
        if !fraction.is_finite() || fraction <= 0.0 || fraction >= 1.0 {
            continue;
        }
        let scale = [start, mid, end]
            .iter()
            .flat_map(|p| [p.x().abs(), p.y().abs(), p.z().abs()])
            .fold(length2.sqrt(), f64::max);
        let roundoff = 32.0 * f64::EPSILON * scale;
        if (mid - (start + direction * fraction)).length() > roundoff {
            continue;
        }
        let ri = root(&parent, i);
        let rj = root(&parent, j);
        parent[ri.max(rj)] = ri.min(rj);
    }
    let mut labels = vec![usize::MAX; edges.len()];
    let mut count = 0;
    let mut groups = Vec::with_capacity(edges.len());
    for i in 0..edges.len() {
        let r = root(&parent, i);
        if labels[r] == usize::MAX {
            labels[r] = count;
            count += 1;
        }
        groups.push(labels[r]);
    }
    Ok((groups, count))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use remus_math::curves::Circle3D;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::check_wire_self_intersection;

    fn polygon_wire(topo: &mut Topology, points: &[(f64, f64)]) -> remus_topology::wire::WireId {
        let vertices: Vec<_> = points
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)))
            .collect();
        let edges = (0..vertices.len())
            .map(|i| {
                OrientedEdge::new(
                    topo.add_edge(Edge::new(
                        vertices[i],
                        vertices[(i + 1) % vertices.len()],
                        EdgeCurve::Line,
                    )),
                    true,
                )
            })
            .collect();
        topo.add_wire(Wire::new(edges, true).unwrap())
    }

    #[test]
    fn straight_boundary_subdivision_preserves_adjacency() {
        for rotation in 0..5 {
            let mut points = vec![
                (0.0, 0.0),
                (1.0, 0.0),
                (1.0, 1.0),
                (1.0 - 7.75e-7, 1.0),
                (0.0, 1.0),
            ];
            points.rotate_left(rotation);
            let mut topo = Topology::new();
            let wire = polygon_wire(&mut topo, &points);
            assert!(
                check_wire_self_intersection(&topo, wire, 1e-6)
                    .unwrap()
                    .is_empty()
            );
            assert_eq!(super::collinear_boundary_groups(&topo, wire).unwrap().1, 4);
        }
        let mut topo = Topology::new();
        let wire = polygon_wire(&mut topo, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
        assert!(
            check_wire_self_intersection(&topo, wire, 1e-6)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn subdivision_does_not_hide_crossings_bends_or_backtracking() {
        for points in [
            // Crossing remains non-adjacent even after grouping the split diagonal.
            vec![(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)],
            // A tiny real bend does not become a straight-boundary exemption.
            vec![
                (0.0, 0.0),
                (1.0, 0.0),
                (1.0, 1.0),
                (1.0 - 7.75e-7, 1.0 + 1e-8),
                (0.0, 1.0),
            ],
            // Collinear backtracking is not a monotone subdivision.
            vec![
                (0.0, 0.0),
                (1.0, 0.0),
                (1.0, 1.0),
                (1.0 + 7.75e-7, 1.0),
                (0.0, 1.0),
            ],
        ] {
            let mut topo = Topology::new();
            let wire = polygon_wire(&mut topo, &points);
            assert!(
                !check_wire_self_intersection(&topo, wire, 1e-6)
                    .unwrap()
                    .is_empty(),
                "{points:?}"
            );
        }
    }

    #[test]
    fn untrimmed_circle_uses_short_arc_for_crossing_check() {
        let mut topo = Topology::new();
        let points = [
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, -1.0),
            Point3::new(0.0, -1.0, -1.0),
        ];
        let vertices: Vec<_> = points
            .into_iter()
            .map(|point| topo.add_vertex(Vertex::new(point, 1e-7)))
            .collect();
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        let mut edges = vec![topo.add_edge(Edge::new(
            vertices[0],
            vertices[1],
            EdgeCurve::Circle(circle),
        ))];
        for index in 1..vertices.len() {
            edges.push(topo.add_edge(Edge::new(
                vertices[index],
                vertices[(index + 1) % vertices.len()],
                EdgeCurve::Line,
            )));
        }
        let wire = topo.add_wire(
            Wire::new(
                edges
                    .into_iter()
                    .map(|edge| OrientedEdge::new(edge, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );

        let issues = check_wire_self_intersection(&topo, wire, 1e-7).unwrap();
        assert!(
            issues.is_empty(),
            "short quarter arc must not cross: {issues:?}"
        );
    }
}
