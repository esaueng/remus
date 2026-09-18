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
    use remus_math::curves::{Ellipse3D, Hyperbola3D, Parabola3D};
    use remus_math::nurbs::fitting::interpolate;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::vertex::{Vertex, VertexId};
    use remus_topology::wire::{OrientedEdge, Wire, WireId};

    use super::check_wire_self_intersection;
    use super::{
        CheckId, check_wire_closure, check_wire_connected, check_wire_redundant,
        check_wire_self_intersection_on_periodic_surface, collinear_boundary_groups,
    };

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

    fn vtx(topo: &mut Topology, x: f64, y: f64) -> VertexId {
        topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7))
    }

    fn line_edge(topo: &mut Topology, start: VertexId, end: VertexId) -> EdgeId {
        topo.add_edge(Edge::new(start, end, EdgeCurve::Line))
    }

    fn oriented(edge: EdgeId, forward: bool) -> OrientedEdge {
        OrientedEdge::new(edge, forward)
    }

    fn add_wire(edges: Vec<OrientedEdge>, closed: bool, topo: &mut Topology) -> WireId {
        topo.add_wire(Wire::new(edges, closed).unwrap())
    }

    #[test]
    fn connected_gap_reported_and_triangle_clean() {
        let mut topo = Topology::new();
        let v0 = vtx(&mut topo, 0.0, 0.0);
        let v1 = vtx(&mut topo, 1.0, 0.0);
        let v2 = vtx(&mut topo, 1.0, 1.0);
        let v3 = vtx(&mut topo, 5.0, 5.0);
        let v4 = vtx(&mut topo, 6.0, 5.0);
        let wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, v0, v1), true),
                oriented(line_edge(&mut topo, v1, v2), true),
                oriented(line_edge(&mut topo, v3, v4), true),
            ],
            false,
            &mut topo,
        );
        let issues = check_wire_connected(&topo, wire).unwrap();
        assert_eq!(issues.len(), 1, "gap must fire once: {issues:?}");
        assert_eq!(issues[0].check, CheckId::WireNotConnected);

        let mut clean_topo = Topology::new();
        let clean = polygon_wire(&mut clean_topo, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]);
        assert!(check_wire_connected(&clean_topo, clean).unwrap().is_empty());
    }

    #[test]
    fn connected_two_edge_gap_reported() {
        let mut topo = Topology::new();
        let v0 = vtx(&mut topo, 0.0, 0.0);
        let v1 = vtx(&mut topo, 1.0, 0.0);
        let v2 = vtx(&mut topo, 3.0, 0.0);
        let v3 = vtx(&mut topo, 4.0, 0.0);
        let wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, v0, v1), true),
                oriented(line_edge(&mut topo, v2, v3), true),
            ],
            false,
            &mut topo,
        );
        let issues = check_wire_connected(&topo, wire).unwrap();
        assert_eq!(issues.len(), 1, "two-edge gap must fire: {issues:?}");
        assert_eq!(issues[0].check, CheckId::WireNotConnected);
    }

    #[test]
    fn closure_mismatch_reported_and_open_wire_ignored() {
        let mut topo = Topology::new();
        let v0 = vtx(&mut topo, 0.0, 0.0);
        let v1 = vtx(&mut topo, 1.0, 0.0);
        let v2 = vtx(&mut topo, 1.0, 1.0);
        let v3 = vtx(&mut topo, 2.0, 2.0);
        // `Wire::new` performs no closure validation, so a closed-flagged
        // wire with mismatched endpoints is constructible here on purpose.
        let closed_wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, v0, v1), true),
                oriented(line_edge(&mut topo, v1, v2), true),
                oriented(line_edge(&mut topo, v2, v3), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_closure(&topo, closed_wire).unwrap();
        assert_eq!(issues.len(), 1, "closure gap must fire: {issues:?}");
        assert_eq!(issues[0].check, CheckId::WireClosure3D);

        let open_wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, v0, v1), true),
                oriented(line_edge(&mut topo, v1, v2), true),
                oriented(line_edge(&mut topo, v2, v3), true),
            ],
            false,
            &mut topo,
        );
        assert!(
            check_wire_closure(&topo, open_wire).unwrap().is_empty(),
            "open wires are exempt from closure"
        );
    }

    #[test]
    fn redundant_triple_use_reported_and_double_use_clean() {
        let mut topo = Topology::new();
        let va = vtx(&mut topo, 0.0, 0.0);
        let vb = vtx(&mut topo, 1.0, 0.0);
        let vc = vtx(&mut topo, 1.0, 1.0);
        let vd = vtx(&mut topo, 2.0, 2.0);
        let ve = vtx(&mut topo, 3.0, 3.0);
        let shared = line_edge(&mut topo, va, vb);
        let filler_a = line_edge(&mut topo, vb, vc);
        let filler_b = line_edge(&mut topo, vd, ve);
        let triple = add_wire(
            vec![
                oriented(shared, true),
                oriented(filler_a, true),
                oriented(shared, true),
                oriented(filler_b, true),
                oriented(shared, true),
            ],
            false,
            &mut topo,
        );
        let issues = check_wire_redundant(&topo, triple).unwrap();
        assert_eq!(issues.len(), 1, "triple use must fire: {issues:?}");
        assert_eq!(issues[0].check, CheckId::WireRedundantEdge);

        let double = add_wire(
            vec![
                oriented(shared, true),
                oriented(filler_a, true),
                oriented(shared, true),
                oriented(filler_b, true),
            ],
            false,
            &mut topo,
        );
        assert!(
            check_wire_redundant(&topo, double).unwrap().is_empty(),
            "double use stays below the 3-use threshold"
        );
    }

    /// Bowtie quadrilateral whose only crossing is the non-adjacent pair.
    fn bowtie_wire(topo: &mut Topology) -> WireId {
        polygon_wire(topo, &[(0.0, 0.0), (2.0, 1.0), (2.0, 0.0), (0.0, 1.0)])
    }

    #[test]
    fn periodic_bowtie_self_intersection_reported() {
        let mut topo = Topology::new();
        let wire = bowtie_wire(&mut topo);
        let issues = check_wire_self_intersection_on_periodic_surface(&topo, wire, 1e-6).unwrap();
        assert_eq!(issues.len(), 1, "bowtie must fire: {issues:?}");
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn periodic_duplicate_seam_exempt_but_plain_reports() {
        // One arc edge used twice in opposite directions: the periodic
        // entry exempts exactly this duplicated seam; the plain entry does
        // not. An arc (not a line) ensures interior mirror-segment pairs
        // that the seam-vertex fallback exemption cannot swallow.
        let mut topo = Topology::new();
        let circle = unit_circle();
        let arc = circle_edge(&mut topo, &circle, 0.5, -0.5);
        let s0 = vtx(&mut topo, 10.0, 10.0);
        let s1 = vtx(&mut topo, 11.0, 10.0);
        let s2 = vtx(&mut topo, 10.0, -10.0);
        let s3 = vtx(&mut topo, 11.0, -10.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, s0, s1), true),
                oriented(arc, false),
                oriented(line_edge(&mut topo, s2, s3), true),
            ],
            false,
            &mut topo,
        );
        assert!(
            check_wire_self_intersection_on_periodic_surface(&topo, wire, 1e-6)
                .unwrap()
                .is_empty(),
            "duplicated seam must stay exempt"
        );
        assert!(
            !check_wire_self_intersection(&topo, wire, 1e-6)
                .unwrap()
                .is_empty(),
            "plain check must flag the coincident pair"
        );
    }

    /// Hourglass pinched at a shared center vertex: every close contact is
    /// an exact topological endpoint touch, never an interior crossing.
    fn pinched_wire(topo: &mut Topology) -> WireId {
        let a = vtx(topo, 0.0, 0.0);
        let v = vtx(topo, 1.0, 1.0);
        let b = vtx(topo, 2.0, 0.0);
        let c = vtx(topo, 2.0, 2.0);
        let d = vtx(topo, 0.0, 2.0);
        add_wire(
            vec![
                oriented(line_edge(topo, a, v), true),
                oriented(line_edge(topo, v, b), true),
                oriented(line_edge(topo, b, c), true),
                oriented(line_edge(topo, c, v), true),
                oriented(line_edge(topo, v, d), true),
                oriented(line_edge(topo, d, a), true),
            ],
            true,
            topo,
        )
    }

    #[test]
    fn pinched_vertex_contact_reports_plain_but_exempt_periodic() {
        let mut topo = Topology::new();
        let wire = pinched_wire(&mut topo);
        let plain = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(
            !plain.is_empty(),
            "shared-vertex contacts must fire without the seam exemption"
        );
        assert!(
            check_wire_self_intersection_on_periodic_surface(&topo, wire, 1e-6)
                .unwrap()
                .is_empty(),
            "exact seam-vertex touches must stay exempt: {plain:?}"
        );
    }

    #[test]
    fn periodic_mixed_orientation_crossing_still_reports() {
        // Same geometry as the bowtie but one crossing edge is traversed
        // backwards, so an orientation-based exemption must not fire.
        let mut topo = Topology::new();
        let a = vtx(&mut topo, 0.0, 0.0);
        let b = vtx(&mut topo, 2.0, 1.0);
        let c = vtx(&mut topo, 2.0, 0.0);
        let d = vtx(&mut topo, 0.0, 1.0);
        let wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, a, b), true),
                oriented(line_edge(&mut topo, b, c), true),
                oriented(line_edge(&mut topo, c, d), false),
                oriented(line_edge(&mut topo, d, a), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection_on_periodic_surface(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "orientation difference alone must not exempt a crossing: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn periodic_reused_pair_crossing_still_reports() {
        // Two edges each used twice, crossing transversely: reuse alone
        // must not exempt a real crossing (only the exact opposite-direction
        // seam pair is exempt).
        let mut topo = Topology::new();
        let p0 = vtx(&mut topo, 0.0, 0.0);
        let p1 = vtx(&mut topo, 4.0, 0.0);
        let q0 = vtx(&mut topo, 2.0, -2.0);
        let q1 = vtx(&mut topo, 2.0, 2.0);
        let bar = line_edge(&mut topo, p0, p1);
        let post = line_edge(&mut topo, q0, q1);
        let mut stub = |x: f64, y: f64| {
            let s0 = vtx(&mut topo, x, y);
            let s1 = vtx(&mut topo, x + 1.0, y);
            line_edge(&mut topo, s0, s1)
        };
        let wire = add_wire(
            vec![
                oriented(bar, true),
                oriented(stub(10.0, 10.0), true),
                oriented(post, false),
                oriented(stub(10.0, -10.0), true),
                oriented(bar, true),
                oriented(stub(-10.0, 10.0), true),
                oriented(post, false),
            ],
            false,
            &mut topo,
        );
        let issues = check_wire_self_intersection_on_periodic_surface(&topo, wire, 1e-6).unwrap();
        assert!(!issues.is_empty(), "reused crossing edges must still fire");
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn late_index_pair_crossing_reported() {
        // Six-edge wire whose only crossing is pair (3, 5): exercises the
        // pair-enumeration tail rather than the (0, 2) head.
        let mut topo = Topology::new();
        let points = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.0, 5.0),
            (0.0, 5.0),
            (2.0, 1.0),
            (2.0, 4.0),
        ];
        let ids: Vec<VertexId> = points.iter().map(|&(x, y)| vtx(&mut topo, x, y)).collect();
        let wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, ids[0], ids[1]), true),
                oriented(line_edge(&mut topo, ids[1], ids[2]), true),
                oriented(line_edge(&mut topo, ids[2], ids[3]), true),
                oriented(line_edge(&mut topo, ids[3], ids[4]), true),
                oriented(line_edge(&mut topo, ids[4], ids[5]), true),
                oriented(line_edge(&mut topo, ids[5], ids[0]), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "late crossing pair must fire exactly once: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    fn unit_circle() -> Circle3D {
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    fn circle_edge(topo: &mut Topology, circle: &Circle3D, t0: f64, t1: f64) -> EdgeId {
        let start = topo.add_vertex(Vertex::new(circle.evaluate(t0), 1e-7));
        let end = topo.add_vertex(Vertex::new(circle.evaluate(t1), 1e-7));
        topo.add_edge(Edge::new(start, end, EdgeCurve::Circle(circle.clone())))
    }

    /// Short-arc wire plus two radial chords placed strictly outside the
    /// real minor span: short-arc mutants that widen the span hit a chord.
    fn circle_span_discriminator(topo: &mut Topology) -> WireId {
        let circle = unit_circle();
        let arc = circle_edge(topo, &circle, 0.25, -0.25);
        let end_b = topo.edge(arc).unwrap().end();
        let end_a = topo.edge(arc).unwrap().start();
        let c1 = vtx(topo, -1.0, -1.0);
        let low_far = vtx(topo, -1.2, -0.8);
        // Radial chord crossing the circle near angle -0.42.
        let x0 = vtx(topo, 1.15, -0.55);
        let x1 = vtx(topo, 0.75, -0.3);
        // Radial chord crossing the circle at angle +0.5.
        let y0 = vtx(topo, 1.097, 0.599);
        let y1 = vtx(topo, 0.702, 0.384);
        // Single return edge ending at the arc start so the shared vertex
        // sits at an index-adjacent pair.
        add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(topo, end_b, c1), true),
                oriented(line_edge(topo, x0, x1), true),
                oriented(line_edge(topo, y0, y1), true),
                oriented(line_edge(topo, low_far, end_a), true),
            ],
            true,
            topo,
        )
    }

    #[test]
    fn circle_short_arc_span_stays_minor() {
        let mut topo = Topology::new();
        let wire = circle_span_discriminator(&mut topo);
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(
            issues.is_empty(),
            "minor arc must not touch the out-of-span chords: {issues:?}"
        );
    }

    #[test]
    fn circle_short_arc_crossing_reported() {
        let mut topo = Topology::new();
        let circle = unit_circle();
        let arc = circle_edge(&mut topo, &circle, 0.25, -0.25);
        let end_b = topo.edge(arc).unwrap().end();
        let end_a = topo.edge(arc).unwrap().start();
        let c = vtx(&mut topo, 1.5, 0.5);
        let d = vtx(&mut topo, 0.5, -0.5);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, end_b, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, end_a), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "chord through the minor arc must fire: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn circle_ccw_arc_crossing_reported() {
        // Counter-clockwise minor arc (-0.25 to +0.25): exercises the
        // `forward <= PI` span branch rather than the wrapped complement.
        let mut topo = Topology::new();
        let circle = unit_circle();
        let arc = circle_edge(&mut topo, &circle, -0.25, 0.25);
        let end_b = topo.edge(arc).unwrap().end();
        let end_a = topo.edge(arc).unwrap().start();
        let c = vtx(&mut topo, 1.5, -0.5);
        let d = vtx(&mut topo, 0.5, 0.5);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, end_b, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, end_a), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "chord through the CCW minor arc must fire: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn circle_closed_loop_crossing_reported() {
        let mut topo = Topology::new();
        let circle = unit_circle();
        let cv = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let loop_edge = topo.add_edge(Edge::new(cv, cv, EdgeCurve::Circle(circle)));
        let s0 = vtx(&mut topo, 5.0, 5.0);
        let s1 = vtx(&mut topo, 6.0, 5.0);
        let x0 = vtx(&mut topo, -0.65, -2.0);
        let x1 = vtx(&mut topo, -0.65, 2.0);
        let s2 = vtx(&mut topo, 5.0, -5.0);
        let s3 = vtx(&mut topo, 6.0, -5.0);
        let wire = add_wire(
            vec![
                oriented(loop_edge, true),
                oriented(line_edge(&mut topo, s0, s1), true),
                oriented(line_edge(&mut topo, x0, x1), true),
                oriented(line_edge(&mut topo, s2, s3), true),
            ],
            false,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "full-circle span must reach the crossing line: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn circle_wrapped_tip_crossing_reported() {
        // Wrapped-span arc (P0 at 90 deg, P1 at 0 deg) crossed at its tip
        // (angle 0.1): else-branch span mutants that truncate or shift the
        // tip miss it while the true short arc fires.
        let mut topo = Topology::new();
        let circle = unit_circle();
        let p0 = circle.evaluate(std::f64::consts::FRAC_PI_2);
        let p1 = circle.evaluate(0.0);
        let v0 = topo.add_vertex(Vertex::new(p0, 1e-7));
        let v1 = topo.add_vertex(Vertex::new(p1, 1e-7));
        let arc = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Circle(circle)));
        let e0 = vtx(&mut topo, 1.3, 0.1);
        let e1 = vtx(&mut topo, -1.3, 0.1);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, v1, e0), true),
                oriented(line_edge(&mut topo, e0, e1), true),
                oriented(line_edge(&mut topo, e1, v0), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "tip chord through the wrapped arc must fire: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn circle_wrapped_span_short_arc_clean() {
        // Endpoints whose counter-clockwise span is major: the untrimmed
        // convention must still sample the short arc through (0.7, 0.7).
        let mut topo = Topology::new();
        let circle = unit_circle();
        let p0 = circle.evaluate(std::f64::consts::FRAC_PI_2);
        let p1 = circle.evaluate(0.0);
        let v0 = topo.add_vertex(Vertex::new(p0, 1e-7));
        let v1 = topo.add_vertex(Vertex::new(p1, 1e-7));
        let arc = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Circle(circle)));
        let c = vtx(&mut topo, 2.0, -1.5);
        let d = vtx(&mut topo, -1.5, 2.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, v1, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, v0), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(
            issues.is_empty(),
            "wrapped short arc must stay clear: {issues:?}"
        );
    }

    fn side_arc_ellipse() -> Ellipse3D {
        Ellipse3D::with_axes(
            Point3::new(0.5, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.5,
            0.5,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn ellipse_lower_arc_clean() {
        // Forward traversal samples the lower half (through (0.5, -1.5));
        // the box sits above the chord.
        let mut topo = Topology::new();
        let ellipse = side_arc_ellipse();
        let vb = topo.add_vertex(Vertex::new(
            ellipse.evaluate(std::f64::consts::FRAC_PI_2),
            1e-7,
        ));
        let va = topo.add_vertex(Vertex::new(
            ellipse.evaluate(-std::f64::consts::FRAC_PI_2),
            1e-7,
        ));
        let arc = topo.add_edge(Edge::new(vb, va, EdgeCurve::Ellipse(ellipse)));
        let top_right = vtx(&mut topo, 1.0, 1.0);
        let top_left = vtx(&mut topo, 0.0, 1.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, top_right), true),
                oriented(line_edge(&mut topo, top_right, top_left), true),
                oriented(line_edge(&mut topo, top_left, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(issues.is_empty(), "lower arc must stay clear: {issues:?}");
    }

    #[test]
    fn ellipse_upper_arc_crossing_reported() {
        // The same edge traversed backwards samples the upper half, which
        // crosses the lid of the box.
        let mut topo = Topology::new();
        let ellipse = side_arc_ellipse();
        let vb = topo.add_vertex(Vertex::new(
            ellipse.evaluate(std::f64::consts::FRAC_PI_2),
            1e-7,
        ));
        let va = topo.add_vertex(Vertex::new(
            ellipse.evaluate(-std::f64::consts::FRAC_PI_2),
            1e-7,
        ));
        let arc = topo.add_edge(Edge::new(vb, va, EdgeCurve::Ellipse(ellipse)));
        let outer_left = vtx(&mut topo, -0.5, 1.0);
        let outer_right = vtx(&mut topo, 1.5, 1.0);
        let wire = add_wire(
            vec![
                oriented(arc, false),
                oriented(line_edge(&mut topo, va, outer_left), true),
                oriented(line_edge(&mut topo, outer_left, outer_right), true),
                oriented(line_edge(&mut topo, outer_right, vb), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(issues.len(), 1, "upper arc must cross the lid: {issues:?}");
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    fn branch_hyperbola() -> Hyperbola3D {
        Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
            1.0,
        )
        .unwrap()
    }

    #[test]
    fn hyperbola_shallow_arc_clean() {
        let mut topo = Topology::new();
        let hyperbola = branch_hyperbola();
        let va = topo.add_vertex(Vertex::new(hyperbola.evaluate(-0.5), 1e-7));
        let vb = topo.add_vertex(Vertex::new(hyperbola.evaluate(0.5), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Hyperbola(hyperbola)));
        let c = vtx(&mut topo, 4.0, 3.0);
        let d = vtx(&mut topo, 4.0, -3.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(issues.is_empty(), "shallow arc must stay clear: {issues:?}");
    }

    #[test]
    fn hyperbola_arc_crossing_reported() {
        let mut topo = Topology::new();
        let hyperbola = branch_hyperbola();
        let va = topo.add_vertex(Vertex::new(hyperbola.evaluate(-0.5), 1e-7));
        let vb = topo.add_vertex(Vertex::new(hyperbola.evaluate(0.5), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Hyperbola(hyperbola)));
        let c = vtx(&mut topo, 1.05, 1.0);
        let d = vtx(&mut topo, 1.05, -1.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "vertical line must cross the hyperbola arc: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    fn dip_parabola() -> Parabola3D {
        Parabola3D::with_axes(
            Point3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        )
        .unwrap()
    }

    #[test]
    fn parabola_arc_clean() {
        let mut topo = Topology::new();
        let parabola = dip_parabola();
        let va = topo.add_vertex(Vertex::new(parabola.evaluate(-2.0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(parabola.evaluate(2.0), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Parabola(parabola)));
        let c = vtx(&mut topo, 4.0, 3.0);
        let d = vtx(&mut topo, 0.0, 3.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(issues.is_empty(), "dipping arc must stay clear: {issues:?}");
    }

    #[test]
    fn parabola_arc_crossing_reported() {
        let mut topo = Topology::new();
        let parabola = dip_parabola();
        let va = topo.add_vertex(Vertex::new(parabola.evaluate(-2.0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(parabola.evaluate(2.0), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Parabola(parabola)));
        let c = vtx(&mut topo, 2.0, 2.0);
        let d = vtx(&mut topo, 2.0, -1.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "vertical line must cross the parabola dip: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    fn shallow_nurbs() -> remus_math::nurbs::curve::NurbsCurve {
        interpolate(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.1, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(3.0, -0.1, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            3,
        )
        .unwrap()
    }

    fn dip_nurbs() -> remus_math::nurbs::curve::NurbsCurve {
        // Dips below y = 0 and back while both endpoints stay above: the
        // endpoint chord never touches the y = 0 line, only the bulge does.
        interpolate(
            &[
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.2, 0.0),
                Point3::new(2.0, -0.3, 0.0),
                Point3::new(3.0, 0.2, 0.0),
                Point3::new(4.0, 1.0, 0.0),
            ],
            3,
        )
        .unwrap()
    }

    #[test]
    fn nurbs_shallow_arc_clean() {
        let mut topo = Topology::new();
        let curve = shallow_nurbs();
        let (d0, d1) = curve.domain();
        let va = topo.add_vertex(Vertex::new(curve.evaluate(d0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(curve.evaluate(d1), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::NurbsCurve(curve)));
        let c = vtx(&mut topo, 4.0, 2.0);
        let d = vtx(&mut topo, 0.0, 2.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(
            issues.is_empty(),
            "shallow NURBS must stay clear: {issues:?}"
        );
    }

    #[test]
    fn nurbs_dip_crossing_reported() {
        let mut topo = Topology::new();
        let curve = dip_nurbs();
        let (d0, d1) = curve.domain();
        let va = topo.add_vertex(Vertex::new(curve.evaluate(d0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(curve.evaluate(d1), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::NurbsCurve(curve)));
        let f = vtx(&mut topo, -1.0, 0.0);
        let g = vtx(&mut topo, 5.0, 0.0);
        let h = vtx(&mut topo, 5.0, 3.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, f), true),
                oriented(line_edge(&mut topo, f, g), true),
                oriented(line_edge(&mut topo, g, h), true),
                oriented(line_edge(&mut topo, h, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "dip below the axis must cross it: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn collinear_groups_keep_arc_edge_separate() {
        // A line and an arc sharing a vertex stay separate groups even when
        // the arc endpoints are collinear with the line.
        let mut topo = Topology::new();
        let v0 = vtx(&mut topo, 0.0, 0.0);
        let v1 = vtx(&mut topo, 1.0, 0.0);
        let v2 = vtx(&mut topo, 2.0, 0.0);
        let v3 = vtx(&mut topo, 2.0, 1.0);
        let bulge = Ellipse3D::new_with_ref(
            Point3::new(1.5, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.5,
            0.3,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, v0, v1), true),
                oriented(
                    topo.add_edge(Edge::new(v1, v2, EdgeCurve::Ellipse(bulge))),
                    true,
                ),
                oriented(line_edge(&mut topo, v2, v3), true),
                oriented(line_edge(&mut topo, v3, v0), true),
            ],
            true,
            &mut topo,
        );
        let (groups, count) = collinear_boundary_groups(&topo, wire).unwrap();
        assert_eq!(groups.len(), 4);
        assert_eq!(count, 4, "arc edges never merge: {groups:?}");
    }

    #[test]
    fn collinear_groups_keep_disconnected_gap_separate() {
        // Consecutive but disconnected collinear lines are distinct edges.
        let mut topo = Topology::new();
        let v0 = vtx(&mut topo, 0.0, 0.0);
        let v1 = vtx(&mut topo, 1.0, 0.0);
        let v2 = vtx(&mut topo, 2.0, 0.0);
        let v3 = vtx(&mut topo, 3.0, 0.0);
        let v4 = vtx(&mut topo, 3.0, 1.0);
        let wire = add_wire(
            vec![
                oriented(line_edge(&mut topo, v0, v1), true),
                oriented(line_edge(&mut topo, v2, v3), true),
                oriented(line_edge(&mut topo, v3, v4), true),
                oriented(line_edge(&mut topo, v4, v0), true),
            ],
            false,
            &mut topo,
        );
        let (groups, count) = collinear_boundary_groups(&topo, wire).unwrap();
        assert_eq!(groups.len(), 4);
        assert_eq!(count, 4, "gapped lines never merge: {groups:?}");
    }

    #[test]
    fn collinear_midpoint_subdivision_stays_single_group() {
        // A side-2 square with a midpoint split: the even split keeps the
        // fractional arithmetic away from the tiny-offset regime.
        let mut topo = Topology::new();
        let wire = polygon_wire(
            &mut topo,
            &[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.0, 2.0), (0.0, 2.0)],
        );
        let (groups, count) = collinear_boundary_groups(&topo, wire).unwrap();
        assert_eq!(groups.len(), 5);
        assert_eq!(count, 4, "midpoint split must merge: {groups:?}");
        assert!(
            check_wire_self_intersection(&topo, wire, 1e-6)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn collinear_near_roundoff_bump_still_merges() {
        // A 7e-15 perpendicular bump sits between the shrunken and the
        // nominal roundoff bands, so only the true tolerance merges it.
        let mut topo = Topology::new();
        let wire = polygon_wire(
            &mut topo,
            &[
                (0.0, 0.0),
                (2.0, 0.0),
                (2.0, 2.0),
                (1.0, 2.0 + 7e-15),
                (0.0, 2.0),
            ],
        );
        let (groups, count) = collinear_boundary_groups(&topo, wire).unwrap();
        assert_eq!(groups.len(), 5);
        assert_eq!(count, 4, "roundoff bump must merge: {groups:?}");
    }

    #[test]
    fn hyperbola_half_arc_crossing_localized() {
        // Arc over t in [0, 0.5]: the x=1.05 crossing (t ~= 0.31) lives in
        // the sampled span. The `ta + ..` -> `ta * ..` mutant collapses
        // sampling to t = 0 and must lose the crossing.
        let mut topo = Topology::new();
        let hyperbola = branch_hyperbola();
        let va = topo.add_vertex(Vertex::new(hyperbola.evaluate(0.0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(hyperbola.evaluate(0.5), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Hyperbola(hyperbola)));
        let c = vtx(&mut topo, 1.05, 1.0);
        let d = vtx(&mut topo, 1.05, -1.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "half arc must cross the vertical line: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn parabola_half_dip_crossing_localized() {
        // Arc over t in [0.5, 2.0]: the x=3 crossing (t = 1) lives in the
        // sampled span. The `ta + ..` -> `ta * ..` mutant shrinks sampling
        // to t < 0.1 and must lose the crossing.
        let mut topo = Topology::new();
        let parabola = dip_parabola();
        let va = topo.add_vertex(Vertex::new(parabola.evaluate(0.5), 1e-7));
        let vb = topo.add_vertex(Vertex::new(parabola.evaluate(2.0), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Parabola(parabola)));
        let c = vtx(&mut topo, 3.0, 2.0);
        let d = vtx(&mut topo, 3.0, -1.0);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, c), true),
                oriented(line_edge(&mut topo, c, d), true),
                oriented(line_edge(&mut topo, d, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "half dip must cross the vertical line: {issues:?}"
        );
        assert_eq!(issues[0].check, CheckId::WireSelfIntersection);
    }

    #[test]
    fn parabola_chord_crossing_tripwire_stays_clear() {
        // The endpoint chord (y = 1) crosses the x=3 tripwire while the true
        // dip passes 0.25 below its foot. The `/ n` -> `% n` mutant zigzags
        // sampling along the chord and must spuriously fire.
        let mut topo = Topology::new();
        let parabola = dip_parabola();
        let va = topo.add_vertex(Vertex::new(parabola.evaluate(-2.0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(parabola.evaluate(2.0), 1e-7));
        let arc = topo.add_edge(Edge::new(va, vb, EdgeCurve::Parabola(parabola)));
        let top = vtx(&mut topo, 4.0, 3.0);
        let corner = vtx(&mut topo, 3.0, 3.0);
        let foot = vtx(&mut topo, 3.0, 0.5);
        let wire = add_wire(
            vec![
                oriented(arc, true),
                oriented(line_edge(&mut topo, vb, top), true),
                oriented(line_edge(&mut topo, top, corner), true),
                oriented(line_edge(&mut topo, corner, foot), true),
                oriented(line_edge(&mut topo, foot, va), true),
            ],
            true,
            &mut topo,
        );
        let issues = check_wire_self_intersection(&topo, wire, 1e-6).unwrap();
        assert!(issues.is_empty(), "dip must clear the tripwire: {issues:?}");
    }
}
