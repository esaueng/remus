//! Canonicalize a full circular rim represented by several open arcs.
//!
//! Some STEP producers encode a periodic face rim as a cycle of open
//! `Circle` edges instead of the single closed circle edge emitted by
//! BrepKit's writer.  The representation is legal, but it is needlessly
//! hazardous for algorithms that must distinguish the doubled periodic seam
//! from the rim.  This pass recognizes only unambiguous, full-turn cycles and
//! rewrites every wire that references them to one shared closed `EdgeId`.

use std::collections::{HashMap, HashSet, VecDeque};

use brepkit_math::curves::Circle3D;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::Point3;
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
use brepkit_topology::face::FaceId;
use brepkit_topology::solid::SolidId;
use brepkit_topology::vertex::VertexId;
use brepkit_topology::wire::{OrientedEdge, Wire, WireId};

use crate::HealError;

/// Merge unambiguous full-turn cycles of open circular arcs into closed edges.
///
/// Every arc in a cycle must describe the same oriented circle, have the same
/// one- or two-face sharing set, and contribute to exactly one topological
/// cycle whose parameter spans sum to one turn.  All referencing wires are
/// preflighted before any topology is changed.
///
/// Returns the number of arc cycles merged.
///
/// # Errors
///
/// Returns [`HealError`] when a topology lookup or wire replacement fails.
pub fn merge_split_rim_arcs(
    topo: &mut Topology,
    solid_id: SolidId,
    tol: Tolerance,
) -> Result<usize, HealError> {
    let face_ids = brepkit_topology::explorer::solid_faces(topo, solid_id)?;
    let mut wire_ids = Vec::new();
    let mut edge_to_faces: HashMap<EdgeId, HashSet<FaceId>> = HashMap::new();
    let mut edges_in_solid = HashSet::new();

    for &face_id in &face_ids {
        let face = topo.face(face_id)?;
        let mut face_wires = vec![face.outer_wire()];
        face_wires.extend(face.inner_wires().iter().copied());
        for wire_id in face_wires {
            if !wire_ids.contains(&wire_id) {
                wire_ids.push(wire_id);
            }
            for oe in topo.wire(wire_id)?.edges() {
                edges_in_solid.insert(oe.edge());
                edge_to_faces.entry(oe.edge()).or_default().insert(face_id);
            }
        }
    }

    let mut vertex_edges: HashMap<VertexId, HashSet<EdgeId>> = HashMap::new();
    let mut arcs: HashMap<EdgeId, ArcInfo> = HashMap::new();
    for edge_id in edges_in_solid {
        let edge = topo.edge(edge_id)?;
        vertex_edges
            .entry(edge.start())
            .or_default()
            .insert(edge_id);
        vertex_edges.entry(edge.end()).or_default().insert(edge_id);
        if edge.start() == edge.end() {
            continue;
        }
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let (t0, t1) = edge.domain_with_endpoints(start, end);
        arcs.insert(
            edge_id,
            ArcInfo {
                start: edge.start(),
                end: edge.end(),
                circle: circle.clone(),
                span: t1 - t0,
                midpoint: circle.evaluate((t0 + t1) * 0.5),
            },
        );
    }

    let mut vertex_arcs: HashMap<VertexId, Vec<EdgeId>> = HashMap::new();
    for (&edge_id, arc) in &arcs {
        vertex_arcs.entry(arc.start).or_default().push(edge_id);
        vertex_arcs.entry(arc.end).or_default().push(edge_id);
    }

    let mut seeds: Vec<EdgeId> = arcs.keys().copied().collect();
    seeds.sort_unstable();
    let mut visited = HashSet::new();
    let mut candidates = Vec::new();

    for seed in seeds {
        if visited.contains(&seed) {
            continue;
        }
        let seed_circle = &arcs[&seed].circle;
        let mut queue = VecDeque::from([seed]);
        let mut component = Vec::new();
        while let Some(edge_id) = queue.pop_front() {
            if visited.contains(&edge_id)
                || !same_oriented_circle(seed_circle, &arcs[&edge_id].circle, tol)
            {
                continue;
            }
            visited.insert(edge_id);
            component.push(edge_id);
            let arc = &arcs[&edge_id];
            for vertex in [arc.start, arc.end] {
                if let Some(neighbours) = vertex_arcs.get(&vertex) {
                    for &neighbour in neighbours {
                        if !visited.contains(&neighbour)
                            && same_oriented_circle(seed_circle, &arcs[&neighbour].circle, tol)
                        {
                            queue.push_back(neighbour);
                        }
                    }
                }
            }
        }
        component.sort_unstable();
        if let Some(candidate) = candidate_cycle(
            topo,
            component,
            &arcs,
            &edge_to_faces,
            &vertex_edges,
            &wire_ids,
            tol,
        )? {
            candidates.push(candidate);
        }
    }

    let mut merged = 0;
    for candidate in candidates {
        // Re-preflight against the current wires. Components are disjoint, so
        // an earlier rewrite cannot consume this plan's edges, but the check
        // keeps the mutation boundary explicit and fail-closed.
        let rewrites = planned_rewrites(topo, &wire_ids, &candidate.edges, candidate.anchor)?;
        if rewrites.is_empty() {
            continue;
        }
        let new_edge = topo.add_edge(Edge::new(
            candidate.anchor,
            candidate.anchor,
            EdgeCurve::Circle(candidate.circle),
        ));
        for rewrite in rewrites {
            let mut edges = Vec::with_capacity(rewrite.remaining.len() + 1);
            edges.push(OrientedEdge::new(new_edge, rewrite.forward));
            edges.extend(rewrite.remaining);
            let replacement = Wire::new(edges, true)?;
            *topo.wire_mut(rewrite.wire)? = replacement;
        }
        merged += 1;
    }

    Ok(merged)
}

#[derive(Clone)]
struct ArcInfo {
    start: VertexId,
    end: VertexId,
    circle: Circle3D,
    span: f64,
    midpoint: Point3,
}

struct Candidate {
    edges: HashSet<EdgeId>,
    anchor: VertexId,
    circle: Circle3D,
}

struct PlannedRewrite {
    wire: WireId,
    forward: bool,
    remaining: Vec<OrientedEdge>,
}

fn candidate_cycle(
    topo: &Topology,
    component: Vec<EdgeId>,
    arcs: &HashMap<EdgeId, ArcInfo>,
    edge_to_faces: &HashMap<EdgeId, HashSet<FaceId>>,
    vertex_edges: &HashMap<VertexId, HashSet<EdgeId>>,
    wire_ids: &[WireId],
    tol: Tolerance,
) -> Result<Option<Candidate>, HealError> {
    if component.len() < 2 {
        return Ok(None);
    }
    let edges: HashSet<EdgeId> = component.iter().copied().collect();
    let mut cycle_degree: HashMap<VertexId, usize> = HashMap::new();
    for &edge_id in &component {
        let arc = &arcs[&edge_id];
        *cycle_degree.entry(arc.start).or_default() += 1;
        *cycle_degree.entry(arc.end).or_default() += 1;
    }
    if cycle_degree.values().any(|&degree| degree != 2) {
        return Ok(None);
    }

    let faces = edge_to_faces
        .get(&component[0])
        .cloned()
        .unwrap_or_default();
    if !(1..=2).contains(&faces.len())
        || component
            .iter()
            .any(|edge_id| edge_to_faces.get(edge_id) != Some(&faces))
    {
        return Ok(None);
    }

    let reference = &arcs[&component[0]];
    if component
        .iter()
        .any(|edge_id| !same_oriented_circle(&reference.circle, &arcs[edge_id].circle, tol))
    {
        return Ok(None);
    }
    let span: f64 = component.iter().map(|edge_id| arcs[edge_id].span).sum();
    let angular_tol = tol.angular.max(tol.parametric(reference.circle.radius()));
    if (span - std::f64::consts::TAU).abs() > angular_tol {
        return Ok(None);
    }
    if has_near_midpoints(&component, arcs, &reference.circle, tol.linear) {
        return Ok(None);
    }

    let Some(anchor) = common_attachment(topo, wire_ids, &edges)? else {
        return Ok(None);
    };
    // The attachment may also carry a periodic seam. Every junction traversed
    // inside the arc chain, however, must have exactly two incident edges in
    // the solid so no branch can be orphaned by the merge.
    if cycle_degree
        .keys()
        .any(|vertex| *vertex != anchor && vertex_edges.get(vertex).map_or(0, HashSet::len) != 2)
    {
        return Ok(None);
    }
    if planned_rewrites(topo, wire_ids, &edges, anchor)?.is_empty() {
        return Ok(None);
    }

    Ok(Some(Candidate {
        edges,
        anchor,
        circle: reference.circle.clone(),
    }))
}

fn has_near_midpoints(
    component: &[EdgeId],
    arcs: &HashMap<EdgeId, ArcInfo>,
    circle: &Circle3D,
    linear_tol: f64,
) -> bool {
    let center = circle.center();
    let u_axis = (circle.evaluate(0.0) - center) * circle.radius().recip();
    let v_axis = circle.tangent(0.0);
    let mut midpoints: Vec<(f64, Point3)> = component
        .iter()
        .map(|edge_id| {
            let midpoint = arcs[edge_id].midpoint;
            let radial = midpoint - center;
            (radial.dot(v_axis).atan2(radial.dot(u_axis)), midpoint)
        })
        .collect();
    midpoints.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));

    midpoints
        .windows(2)
        .any(|pair| (pair[0].1 - pair[1].1).length() <= linear_tol)
        || midpoints.first().is_some_and(|first| {
            midpoints
                .last()
                .is_some_and(|last| (first.1 - last.1).length() <= linear_tol)
        })
}

fn same_oriented_circle(a: &Circle3D, b: &Circle3D, tol: Tolerance) -> bool {
    let centers_match = (a.center() - b.center()).length() <= tol.linear;
    let radii_match = tol.approx_eq(a.radius(), b.radius());
    let axis_dot = a.normal().dot(b.normal());
    centers_match && radii_match && axis_dot > 0.0 && (1.0 - axis_dot) <= tol.angular.max(1e-12)
}

fn common_attachment(
    topo: &Topology,
    wire_ids: &[WireId],
    plan: &HashSet<EdgeId>,
) -> Result<Option<VertexId>, HealError> {
    let mut constrained = None;
    for &wire_id in wire_ids {
        let wire = topo.wire(wire_id)?;
        let member_count = wire
            .edges()
            .iter()
            .filter(|oe| plan.contains(&oe.edge()))
            .count();
        if member_count == 0 {
            continue;
        }
        if !wire.is_closed() || member_count != plan.len() {
            return Ok(None);
        }
        let unique: HashSet<EdgeId> = wire
            .edges()
            .iter()
            .filter_map(|oe| plan.contains(&oe.edge()).then_some(oe.edge()))
            .collect();
        if unique.len() != plan.len() {
            return Ok(None);
        }
        if member_count == wire.edges().len() {
            continue;
        }
        let Some(start) = run_start(wire.edges(), plan) else {
            return Ok(None);
        };
        let first = wire.edges()[start];
        let start_vertex = first.oriented_start(topo.edge(first.edge())?);
        if constrained.is_some_and(|vertex| vertex != start_vertex) {
            return Ok(None);
        }
        constrained = Some(start_vertex);
    }

    if constrained.is_some() {
        return Ok(constrained);
    }
    let Some(min_edge_id) = plan.iter().min().copied() else {
        return Ok(None);
    };
    Ok(Some(topo.edge(min_edge_id)?.start()))
}

fn planned_rewrites(
    topo: &Topology,
    wire_ids: &[WireId],
    plan: &HashSet<EdgeId>,
    anchor: VertexId,
) -> Result<Vec<PlannedRewrite>, HealError> {
    let mut rewrites = Vec::new();
    for &wire_id in wire_ids {
        let wire = topo.wire(wire_id)?;
        if !wire.edges().iter().any(|oe| plan.contains(&oe.edge())) {
            continue;
        }
        let n = wire.edges().len();
        let member_count = wire
            .edges()
            .iter()
            .filter(|oe| plan.contains(&oe.edge()))
            .count();
        if !wire.is_closed() || member_count != plan.len() || member_count > n {
            return Ok(Vec::new());
        }

        let start = if member_count == n {
            wire.edges().iter().enumerate().find_map(|(index, oe)| {
                let edge = topo.edge(oe.edge()).ok()?;
                (oe.oriented_start(edge) == anchor).then_some(index)
            })
        } else {
            run_start(wire.edges(), plan)
        };
        let Some(start) = start else {
            return Ok(Vec::new());
        };
        let rotated: Vec<OrientedEdge> = (0..n)
            .map(|offset| wire.edges()[(start + offset) % n])
            .collect();
        if rotated[..member_count]
            .iter()
            .any(|oe| !plan.contains(&oe.edge()))
            || rotated[member_count..]
                .iter()
                .any(|oe| plan.contains(&oe.edge()))
        {
            return Ok(Vec::new());
        }

        let forward = rotated[0].is_forward();
        if rotated[..member_count]
            .iter()
            .any(|oe| oe.is_forward() != forward)
        {
            return Ok(Vec::new());
        }
        for index in 0..member_count {
            let current = rotated[index];
            let next = rotated[(index + 1) % member_count];
            let current_edge = topo.edge(current.edge())?;
            let next_edge = topo.edge(next.edge())?;
            if current.oriented_end(current_edge) != next.oriented_start(next_edge) {
                return Ok(Vec::new());
            }
        }
        let first_edge = topo.edge(rotated[0].edge())?;
        if rotated[0].oriented_start(first_edge) != anchor {
            return Ok(Vec::new());
        }
        rewrites.push(PlannedRewrite {
            wire: wire_id,
            forward,
            remaining: rotated[member_count..].to_vec(),
        });
    }
    Ok(rewrites)
}

fn run_start(edges: &[OrientedEdge], plan: &HashSet<EdgeId>) -> Option<usize> {
    let starts: Vec<usize> = (0..edges.len())
        .filter(|&index| {
            plan.contains(&edges[index].edge())
                && !plan.contains(&edges[(index + edges.len() - 1) % edges.len()].edge())
        })
        .collect();
    (starts.len() == 1).then_some(starts[0])
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use brepkit_math::curves::Circle3D;
    use brepkit_math::vec::{Point3, Vec3};
    use brepkit_topology::Topology;
    use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
    use brepkit_topology::face::{Face, FaceSurface};
    use brepkit_topology::shell::Shell;
    use brepkit_topology::solid::Solid;
    use brepkit_topology::vertex::Vertex;
    use brepkit_topology::wire::{OrientedEdge, Wire};

    use super::*;

    fn circle(normal: Vec3) -> Circle3D {
        Circle3D::new(Point3::new(0.0, 0.0, 0.0), normal, 2.0).unwrap()
    }

    const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    fn vertex(topo: &mut Topology, circle: &Circle3D, angle: f64) -> VertexId {
        topo.add_vertex(Vertex::new(circle.evaluate(angle), 1e-7))
    }

    fn arc(topo: &mut Topology, start: VertexId, end: VertexId, circle: &Circle3D) -> EdgeId {
        topo.add_edge(Edge::new(start, end, EdgeCurve::Circle(circle.clone())))
    }

    fn solid_with_two_arc_wires(topo: &mut Topology, arcs: &[EdgeId]) -> (SolidId, WireId, WireId) {
        let forward = topo.add_wire(
            Wire::new(
                arcs.iter()
                    .copied()
                    .map(|edge| OrientedEdge::new(edge, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        let reverse = topo.add_wire(
            Wire::new(
                arcs.iter()
                    .rev()
                    .copied()
                    .map(|edge| OrientedEdge::new(edge, false))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        let surface = FaceSurface::Plane { normal: Z, d: 0.0 };
        let f0 = topo.add_face(Face::new(forward, Vec::new(), surface.clone()));
        let f1 = topo.add_face(Face::new(reverse, Vec::new(), surface));
        let shell = topo.add_shell(Shell::new(vec![f0, f1]).unwrap());
        (
            topo.add_solid(Solid::new(shell, Vec::new())),
            forward,
            reverse,
        )
    }

    fn assert_merged(topo: &Topology, a: WireId, b: WireId) {
        let wa = topo.wire(a).unwrap();
        let wb = topo.wire(b).unwrap();
        assert_eq!(wa.edges().len(), 1);
        assert_eq!(wb.edges().len(), 1);
        assert_eq!(wa.edges()[0].edge(), wb.edges()[0].edge());
        let edge = topo.edge(wa.edges()[0].edge()).unwrap();
        assert_eq!(edge.start(), edge.end());
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
    }

    #[test]
    fn merges_two_arc_cycle_in_both_wires() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, std::f64::consts::PI);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, b, a, &c);
        let (solid, w0, w1) = solid_with_two_arc_wires(&mut topo, &[e0, e1]);

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            1
        );
        assert_merged(&topo, w0, w1);
    }

    #[test]
    fn merges_three_arc_cycle_in_both_wires() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, 2.0 * std::f64::consts::PI / 3.0);
        let d = vertex(&mut topo, &c, 4.0 * std::f64::consts::PI / 3.0);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, b, d, &c);
        let e2 = arc(&mut topo, d, a, &c);
        let (solid, w0, w1) = solid_with_two_arc_wires(&mut topo, &[e0, e1, e2]);

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            1
        );
        assert_merged(&topo, w0, w1);
    }

    #[test]
    fn refuses_edges_on_differently_oriented_circles() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let reversed = c.reversed();
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, std::f64::consts::PI);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, b, a, &reversed);
        let (solid, _, _) = solid_with_two_arc_wires(&mut topo, &[e0, e1]);

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            0
        );
    }

    #[test]
    fn refuses_three_edge_internal_junction() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, std::f64::consts::PI);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, b, a, &c);
        let extra_vertex = topo.add_vertex(Vertex::new(Point3::new(-3.0, 0.0, 0.0), 1e-7));
        let extra = topo.add_edge(Edge::new(b, extra_vertex, EdgeCurve::Line));
        let (solid, _, _) = solid_with_two_arc_wires(&mut topo, &[e0, e1]);
        let extra_wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(extra, true),
                    OrientedEdge::new(extra, false),
                ],
                true,
            )
            .unwrap(),
        );
        let extra_face = topo.add_face(Face::new(
            extra_wire,
            Vec::new(),
            FaceSurface::Plane { normal: Z, d: 0.0 },
        ));
        let outer = topo.solid(solid).unwrap().outer_shell();
        topo.shell_mut(outer).unwrap().faces_mut()[1] = extra_face;

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            0
        );
    }

    #[test]
    fn refuses_cycle_whose_spans_do_not_sum_to_one_turn() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, std::f64::consts::FRAC_PI_2);
        let d = vertex(&mut topo, &c, std::f64::consts::PI);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, d, b, &c);
        let e2 = arc(&mut topo, d, a, &c);
        let (solid, _, _) = solid_with_two_arc_wires(&mut topo, &[e0, e1, e2]);

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            0
        );
    }

    #[test]
    fn refuses_ambiguous_same_order_pair() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, std::f64::consts::PI);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, a, b, &c);
        let forward = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, false)],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            forward,
            Vec::new(),
            FaceSurface::Plane { normal: Z, d: 0.0 },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, Vec::new()));

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            0
        );
    }

    #[test]
    fn refuses_shared_cycle_with_different_periodic_seam_anchors() {
        let mut topo = Topology::new();
        let c = circle(Z);
        let a = vertex(&mut topo, &c, 0.0);
        let b = vertex(&mut topo, &c, std::f64::consts::PI);
        let e0 = arc(&mut topo, a, b, &c);
        let e1 = arc(&mut topo, b, a, &c);

        let c0 = topo.add_vertex(Vertex::new(Point3::new(3.0, 0.0, 1.0), 1e-7));
        let c1 = topo.add_vertex(Vertex::new(Point3::new(-3.0, 0.0, 1.0), 1e-7));
        let closed0 = topo.add_edge(Edge::new(c0, c0, EdgeCurve::Circle(c.clone())));
        let closed1 = topo.add_edge(Edge::new(c1, c1, EdgeCurve::Circle(c)));
        let seam0 = topo.add_edge(Edge::new(a, c0, EdgeCurve::Line));
        let seam1 = topo.add_edge(Edge::new(b, c1, EdgeCurve::Line));
        let wire0 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(closed0, true),
                    OrientedEdge::new(seam0, false),
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(seam0, true),
                ],
                true,
            )
            .unwrap(),
        );
        let wire1 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(closed1, true),
                    OrientedEdge::new(seam1, false),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(seam1, true),
                ],
                true,
            )
            .unwrap(),
        );
        let surface = FaceSurface::Plane { normal: Z, d: 0.0 };
        let f0 = topo.add_face(Face::new(wire0, Vec::new(), surface.clone()));
        let f1 = topo.add_face(Face::new(wire1, Vec::new(), surface));
        let shell = topo.add_shell(Shell::new(vec![f0, f1]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, Vec::new()));

        assert_eq!(
            merge_split_rim_arcs(&mut topo, solid, Tolerance::new()).unwrap(),
            0,
            "one closed EdgeId cannot preserve two distinct wire attachment vertices"
        );
    }
}
