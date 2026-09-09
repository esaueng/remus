//! Remove internal (hole) wires from faces.

use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

use crate::HealError;

/// Boundary entities no longer reachable from the repaired solid.
#[derive(Debug, Clone, Default)]
pub struct RemovedWireHistory {
    /// Edges used by removed inner wires and absent from every result shell.
    pub edges: Vec<remus_topology::EdgeId>,
    /// Their vertices absent from every result shell.
    pub vertices: Vec<remus_topology::VertexId>,
}

/// Remove all inner (hole) wires from faces in a solid.
///
/// Walks the outer shell *and* any inner (cavity) shells so that hollow
/// solids' cavity-face inner wires are also removed — boolean cuts
/// and `shell_op` can produce cavity faces with their own internal
/// loops, which the prior outer-shell-only implementation silently
/// preserved.
///
/// Returns the total number of wires removed.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn remove_internal_wires(topo: &mut Topology, solid_id: SolidId) -> Result<usize, HealError> {
    remove_internal_wires_impl(topo, solid_id, None)
}

/// Remove inner wires and record their consumed boundary entities.
///
/// Shared edges and vertices still reachable in the solid are not consumed.
/// The records describe result membership, not deletion from the arena.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups or boundary updates fail.
pub fn remove_internal_wires_with_history(
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<(usize, RemovedWireHistory), HealError> {
    let mut history = RemovedWireHistory::default();
    let removed = remove_internal_wires_impl(topo, solid_id, Some(&mut history))?;
    Ok((removed, history))
}

fn remove_internal_wires_impl(
    topo: &mut Topology,
    solid_id: SolidId,
    history: Option<&mut RemovedWireHistory>,
) -> Result<usize, HealError> {
    let mut edges = std::collections::BTreeSet::new();
    let mut vertices = std::collections::BTreeSet::new();
    let face_ids = solid_faces(topo, solid_id)?;

    let mut removed = 0;

    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let n_inner = face.inner_wires().len();
        if n_inner > 0 {
            if history.is_some() {
                for &wire in face.inner_wires() {
                    for oe in topo.wire(wire)?.edges() {
                        let edge = topo.edge(oe.edge())?;
                        edges.insert(oe.edge());
                        vertices.extend([edge.start(), edge.end()]);
                    }
                }
            }
            let outer = face.outer_wire();
            topo.set_face_boundary_wires(fid, outer, Vec::new())?;
            removed += n_inner;
        }
    }

    if let Some(history) = history {
        for edge in remus_topology::explorer::solid_edges(topo, solid_id)? {
            edges.remove(&edge);
        }
        for vertex in remus_topology::explorer::solid_vertices(topo, solid_id)? {
            vertices.remove(&vertex);
        }
        history.edges = edges.into_iter().collect();
        history.vertices = vertices.into_iter().collect();
        history.edges.sort_by_key(|edge| edge.index());
        history.vertices.sort_by_key(|vertex| vertex.index());
    }
    Ok(removed)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    fn add_face_with_n_inner_wires(
        topo: &mut Topology,
        anchor: Point3,
        n_inner: usize,
    ) -> remus_topology::face::FaceId {
        let make_triangle = |topo: &mut Topology, base: Point3| -> remus_topology::wire::WireId {
            let va = topo.add_vertex(Vertex::new(base, 1e-7));
            let vb = topo.add_vertex(Vertex::new(
                Point3::new(base.x() + 1.0, base.y(), base.z()),
                1e-7,
            ));
            let vc = topo.add_vertex(Vertex::new(
                Point3::new(base.x(), base.y() + 1.0, base.z()),
                1e-7,
            ));
            let eab = topo.add_edge(Edge::new(va, vb, EdgeCurve::Line));
            let ebc = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
            let eca = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
            topo.add_wire(
                Wire::new(
                    vec![
                        OrientedEdge::new(eab, true),
                        OrientedEdge::new(ebc, true),
                        OrientedEdge::new(eca, true),
                    ],
                    true,
                )
                .unwrap(),
            )
        };

        let outer_wid = make_triangle(topo, anchor);
        let inner_wires: Vec<_> = (0..n_inner)
            .map(|i| {
                make_triangle(
                    topo,
                    Point3::new(
                        anchor.x() + 0.1 * (i as f64 + 1.0),
                        anchor.y() + 0.1,
                        anchor.z(),
                    ),
                )
            })
            .collect();
        topo.add_face(Face::new(
            outer_wid,
            inner_wires,
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn remove_internal_wires_walks_inner_shells() {
        // Outer shell: 1 face with 2 inner wires.
        // Inner (cavity) shell: 1 face with 3 inner wires.
        // Total expected removals: 2 + 3 = 5.
        let mut topo = Topology::new();

        let outer_face = add_face_with_n_inner_wires(&mut topo, Point3::new(0.0, 0.0, 0.0), 2);
        let inner_face = add_face_with_n_inner_wires(&mut topo, Point3::new(0.0, 0.0, 5.0), 3);

        let outer_shell = topo.add_shell(Shell::new(vec![outer_face]).unwrap());
        let inner_shell = topo.add_shell(Shell::new(vec![inner_face]).unwrap());
        let solid_id = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));

        let mut ordinary = topo.clone();
        let ordinary_count = remove_internal_wires(&mut ordinary, solid_id).unwrap();
        let (removed, history) = remove_internal_wires_with_history(&mut topo, solid_id).unwrap();
        assert_eq!(removed, ordinary_count);
        assert_eq!(topo.allocated_slot_count(), ordinary.allocated_slot_count());
        assert_eq!(history.edges.len(), 15);
        assert_eq!(history.vertices.len(), 15);
        let live_edges = remus_topology::explorer::solid_edges(&topo, solid_id).unwrap();
        let live_vertices = remus_topology::explorer::solid_vertices(&topo, solid_id).unwrap();
        assert!(history.edges.iter().all(|edge| !live_edges.contains(edge)));
        assert!(
            history
                .vertices
                .iter()
                .all(|vertex| !live_vertices.contains(vertex))
        );
        assert_eq!(
            removed, 5,
            "expected 5 inner-wire removals (2 outer + 3 inner), got {removed}"
        );

        // Both faces should now have zero inner wires.
        assert_eq!(topo.face(outer_face).unwrap().inner_wires().len(), 0);
        assert_eq!(topo.face(inner_face).unwrap().inner_wires().len(), 0);
    }
    #[test]
    fn removed_wire_history_preserves_shared_edges_and_vertices() {
        let mut topo = Topology::new();
        let retained = add_face_with_n_inner_wires(&mut topo, Point3::new(0.0, 0.0, 0.0), 0);
        let edited = add_face_with_n_inner_wires(&mut topo, Point3::new(0.0, 0.0, 0.0), 0);
        let shared_wire = topo.face(retained).unwrap().outer_wire();
        let first = topo.wire(shared_wire).unwrap().edges()[0].edge();
        let shared_vertex = topo.edge(first).unwrap().start();
        let b = topo.add_vertex(Vertex::new(Point3::new(0.2, 0.2, 0.0), 1e-7));
        let c = topo.add_vertex(Vertex::new(Point3::new(0.3, 0.2, 0.0), 1e-7));
        let edges = [(shared_vertex, b), (b, c), (c, shared_vertex)]
            .map(|(a, b)| topo.add_edge(Edge::new(a, b, EdgeCurve::Line)));
        let inner = topo.add_wire(
            Wire::new(
                edges.map(|edge| OrientedEdge::new(edge, true)).to_vec(),
                true,
            )
            .unwrap(),
        );
        let outer = topo.face(edited).unwrap().outer_wire();
        topo.set_face_boundary_wires(edited, outer, vec![shared_wire, inner])
            .unwrap();
        let outer_shell = topo.add_shell(Shell::new(vec![edited]).unwrap());
        let inner_shell = topo.add_shell(Shell::new(vec![retained]).unwrap());
        let solid = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));
        let (removed, history) = remove_internal_wires_with_history(&mut topo, solid).unwrap();
        assert_eq!(removed, 2);
        assert_eq!(history.edges, edges);
        assert_eq!(history.vertices, vec![b, c]);
        assert!(!history.vertices.contains(&shared_vertex));
        let (removed, repeated) = remove_internal_wires_with_history(&mut topo, solid).unwrap();
        assert_eq!(removed, 0);
        assert!(repeated.edges.is_empty());
        assert!(repeated.vertices.is_empty());
    }
}
