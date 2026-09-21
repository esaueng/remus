//! Solid-scoped adjacency index for edge-to-face and face-to-face queries.
//!
//! [`AdjacencyIndex`] precomputes adjacency relationships from a solid's shell,
//! mapping edges to their adjacent faces and detecting non-manifold or boundary
//! edges.

use std::collections::HashMap;

use smallvec::SmallVec;

use crate::Topology;
use crate::TopologyError;
use crate::edge::EdgeId;
use crate::face::FaceId;
use crate::solid::SolidId;

/// Precomputed adjacency data for a solid's topology.
///
/// Built from a solid's shell, mapping edges to their adjacent faces
/// and detecting non-manifold/boundary edges.
#[derive(Debug, Clone)]
pub struct AdjacencyIndex {
    /// Maps each edge to the faces that reference it.
    edge_faces: HashMap<EdgeId, SmallVec<[FaceId; 2]>>,
    /// Maps each face to its neighbor faces (those sharing an edge).
    face_neighbors: HashMap<FaceId, SmallVec<[FaceId; 6]>>,
    /// Edges referenced by more than 2 faces.
    non_manifold_edges: Vec<EdgeId>,
    /// Edges referenced by exactly 1 face.
    boundary_edges: Vec<EdgeId>,
}

impl AdjacencyIndex {
    /// Builds an adjacency index from a solid's outer shell.
    ///
    /// Walks all faces in the solid's outer shell, collecting edge-to-face
    /// relationships and classifying edges as manifold, non-manifold, or
    /// boundary.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError`] if any referenced entity (solid, shell,
    /// face, or wire) does not exist in the topology.
    pub fn build(topo: &Topology, solid: SolidId) -> Result<Self, TopologyError> {
        // Solid-scoped: a hollow solid's cavity edges are shared by two cavity
        // faces, and indexing only the outer shell reports them as having none —
        // the sibling `explorer::edge_to_face_map` already walks both. Callers
        // that genuinely want one shell have `build_from_faces` below.
        let faces = crate::explorer::solid_faces(topo, solid)?;
        Self::build_from_faces(topo, &faces)
    }

    /// Builds an adjacency index from an explicit list of faces.
    ///
    /// This is useful for open shells or partial topology that is not
    /// wrapped in a solid.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError`] if any referenced face or wire does not
    /// exist in the topology.
    pub fn build_from_faces(topo: &Topology, faces: &[FaceId]) -> Result<Self, TopologyError> {
        let mut edge_faces: HashMap<EdgeId, SmallVec<[FaceId; 2]>> = HashMap::new();

        for &face_id in faces {
            let face = topo.face(face_id)?;
            // Collect all wire IDs (outer + inner) to avoid borrow issues.
            let mut wire_ids = vec![face.outer_wire()];
            wire_ids.extend_from_slice(face.inner_wires());

            for wire_id in wire_ids {
                let wire = topo.wire(wire_id)?;
                for oriented_edge in wire.edges() {
                    edge_faces
                        .entry(oriented_edge.edge())
                        .or_default()
                        .push(face_id);
                }
            }
        }

        let mut non_manifold_edges = Vec::new();
        let mut boundary_edges = Vec::new();
        let mut face_neighbors: HashMap<FaceId, SmallVec<[FaceId; 6]>> = HashMap::new();

        for &face_id in faces {
            face_neighbors.entry(face_id).or_default();
        }

        for (edge_id, adj_faces) in &edge_faces {
            match adj_faces.len() {
                0 => {} // Shouldn't happen if we built from wires, but harmless.
                1 => boundary_edges.push(*edge_id),
                2 => {
                    // Manifold edge: the two faces are neighbors.
                    let f0 = adj_faces[0];
                    let f1 = adj_faces[1];
                    face_neighbors.entry(f0).or_default().push(f1);
                    face_neighbors.entry(f1).or_default().push(f0);
                }
                _ => non_manifold_edges.push(*edge_id),
            }
        }

        for neighbors in face_neighbors.values_mut() {
            neighbors.sort_unstable_by_key(|id| id.index());
            neighbors.dedup();
        }

        Ok(Self {
            edge_faces,
            face_neighbors,
            non_manifold_edges,
            boundary_edges,
        })
    }

    /// Returns the faces adjacent to the given edge.
    ///
    /// Returns an empty slice if the edge is not in this index.
    #[must_use]
    pub fn faces_for_edge(&self, edge: EdgeId) -> &[FaceId] {
        self.edge_faces.get(&edge).map_or(&[], SmallVec::as_slice)
    }

    /// Returns the neighbor faces of the given face.
    ///
    /// Returns an empty slice if the face is not in this index.
    #[must_use]
    pub fn neighbors_of_face(&self, face: FaceId) -> &[FaceId] {
        self.face_neighbors
            .get(&face)
            .map_or(&[], SmallVec::as_slice)
    }

    /// Returns `true` if all edges are shared by exactly 2 faces (manifold)
    /// and there are no boundary edges.
    #[must_use]
    pub fn is_manifold(&self) -> bool {
        self.non_manifold_edges.is_empty() && self.boundary_edges.is_empty()
    }

    /// Returns edges shared by more than 2 faces.
    #[must_use]
    pub fn non_manifold_edges(&self) -> &[EdgeId] {
        &self.non_manifold_edges
    }

    /// Returns edges referenced by exactly 1 face.
    #[must_use]
    pub fn boundary_edges(&self) -> &[EdgeId] {
        &self.boundary_edges
    }

    /// Returns the faces adjacent to the given edge, or `None` if not found.
    #[must_use]
    pub fn edge_faces(&self, edge: EdgeId) -> Option<&[FaceId]> {
        self.edge_faces.get(&edge).map(smallvec::SmallVec::as_slice)
    }

    /// Returns the number of edges in the adjacency index.
    #[must_use]
    pub fn edge_count(&self) -> usize {
        self.edge_faces.len()
    }

    /// Iterates over all (edge, faces) pairs.
    pub fn edge_faces_iter(&self) -> impl Iterator<Item = (EdgeId, &[FaceId])> {
        self.edge_faces.iter().map(|(k, v)| (*k, v.as_slice()))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[cfg(feature = "test-utils")]
    #[test]
    fn manifold_cube_adjacency() {
        let mut topo = Topology::new();
        let solid = crate::test_utils::make_unit_cube_manifold(&mut topo);
        let adj = AdjacencyIndex::build(&topo, solid).unwrap();

        assert!(adj.is_manifold());
        assert!(adj.non_manifold_edges().is_empty());
        assert!(adj.boundary_edges().is_empty());

        // A cube has 12 edges, each shared by exactly 2 faces.
        assert_eq!(adj.edge_count(), 12);
        for (_, faces) in adj.edge_faces_iter() {
            assert_eq!(faces.len(), 2);
        }

        // Each of the 6 faces has 4 neighbors (cube: every face touches 4 others).
        let shell_id = topo.solid(solid).unwrap().outer_shell();
        let face_ids = topo.shell(shell_id).unwrap().faces();
        assert_eq!(face_ids.len(), 6);
        for &fid in face_ids {
            assert_eq!(adj.neighbors_of_face(fid).len(), 4);
        }
    }

    #[cfg(feature = "test-utils")]
    #[test]
    fn non_manifold_detection() {
        use crate::edge::{Edge, EdgeCurve};
        use crate::face::{Face, FaceSurface};
        use crate::shell::Shell;
        use crate::solid::Solid;
        use crate::vertex::Vertex;
        use crate::wire::{OrientedEdge, Wire};
        use remus_math::vec::{Point3, Vec3};

        let mut topo = Topology::new();

        // Create 5 vertices forming 3 triangular faces that share one edge (e01).
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.5, 1.0, 0.0), 1e-7));
        let v3 = topo.add_vertex(Vertex::new(Point3::new(0.5, -1.0, 0.0), 1e-7));
        let v4 = topo.add_vertex(Vertex::new(Point3::new(0.5, 0.0, 1.0), 1e-7));

        let e01 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        let e12 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e20 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        let w1 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e01, true),
                    OrientedEdge::new(e12, true),
                    OrientedEdge::new(e20, true),
                ],
                true,
            )
            .unwrap(),
        );
        let f1 = topo.add_face(Face::new(
            w1,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));

        let e13 = topo.add_edge(Edge::new(v1, v3, EdgeCurve::Line));
        let e30 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));
        let w2 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e01, true),
                    OrientedEdge::new(e13, true),
                    OrientedEdge::new(e30, true),
                ],
                true,
            )
            .unwrap(),
        );
        let f2 = topo.add_face(Face::new(
            w2,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, -1.0),
                d: 0.0,
            },
        ));

        let e14 = topo.add_edge(Edge::new(v1, v4, EdgeCurve::Line));
        let e40 = topo.add_edge(Edge::new(v4, v0, EdgeCurve::Line));
        let w3 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e01, true),
                    OrientedEdge::new(e14, true),
                    OrientedEdge::new(e40, true),
                ],
                true,
            )
            .unwrap(),
        );
        let f3 = topo.add_face(Face::new(
            w3,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
        ));

        let shell = Shell::new(vec![f1, f2, f3]).unwrap();
        let shell_id = topo.add_shell(shell);
        let solid = topo.add_solid(Solid::new(shell_id, vec![]));

        let adj = AdjacencyIndex::build(&topo, solid).unwrap();

        assert!(!adj.is_manifold());
        assert_eq!(adj.non_manifold_edges().len(), 1);
        assert_eq!(adj.non_manifold_edges()[0], e01);

        // e01 is shared by 3 faces.
        assert_eq!(adj.faces_for_edge(e01).len(), 3);

        // Boundary edges: each of the 6 non-shared edges appears in only 1 face.
        assert_eq!(adj.boundary_edges().len(), 6);
    }

    #[test]
    fn build_from_faces_empty() {
        let topo = Topology::new();
        let adj = AdjacencyIndex::build_from_faces(&topo, &[]).unwrap();
        assert!(adj.is_manifold());
        assert_eq!(adj.edge_count(), 0);
    }

    #[test]
    fn faces_for_unknown_edge_returns_empty() {
        use crate::arena::Arena;
        use crate::edge::Edge;

        let topo = Topology::new();
        let adj = AdjacencyIndex::build_from_faces(&topo, &[]).unwrap();

        let mut dummy: Arena<Edge> = Arena::new();
        let fake_eid = dummy.alloc(Edge::new(
            {
                let mut va: Arena<crate::vertex::Vertex> = Arena::new();
                va.alloc(crate::vertex::Vertex::new(
                    remus_math::vec::Point3::new(0.0, 0.0, 0.0),
                    0.0,
                ))
            },
            {
                let mut va: Arena<crate::vertex::Vertex> = Arena::new();
                va.alloc(crate::vertex::Vertex::new(
                    remus_math::vec::Point3::new(1.0, 0.0, 0.0),
                    0.0,
                ))
            },
            crate::edge::EdgeCurve::Line,
        ));

        assert!(adj.faces_for_edge(fake_eid).is_empty());
    }
}

#[cfg(test)]
mod index_contract_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::vec::{Point3, Vec3};

    use crate::edge::{Edge, EdgeCurve};
    use crate::face::{Face, FaceSurface};
    use crate::shell::Shell;
    use crate::solid::Solid;
    use crate::vertex::{Vertex, VertexId};
    use crate::wire::{OrientedEdge, Wire};

    use super::*;

    const TOL: f64 = 1e-7;

    fn vert(topo: &mut Topology, x: f64, y: f64, z: f64) -> VertexId {
        topo.add_vertex(Vertex::new(Point3::new(x, y, z), TOL))
    }

    /// Builds a face from an already-ordered ring of oriented edges.
    fn face_from(topo: &mut Topology, edges: Vec<(EdgeId, bool)>, normal: Vec3, d: f64) -> FaceId {
        let wire = Wire::new(
            edges
                .into_iter()
                .map(|(eid, fwd)| OrientedEdge::new(eid, fwd))
                .collect(),
            true,
        )
        .expect("fixture wire");
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
    }

    /// A closed, manifold unit cube: 6 faces, 12 edges, every edge used once
    /// forward and once reversed. Returns the solid and its 12 edges.
    fn manifold_cube(topo: &mut Topology) -> (SolidId, Vec<EdgeId>) {
        let v: [VertexId; 8] = [
            vert(topo, 0.0, 0.0, 0.0),
            vert(topo, 1.0, 0.0, 0.0),
            vert(topo, 1.0, 1.0, 0.0),
            vert(topo, 0.0, 1.0, 0.0),
            vert(topo, 0.0, 0.0, 1.0),
            vert(topo, 1.0, 0.0, 1.0),
            vert(topo, 1.0, 1.0, 1.0),
            vert(topo, 0.0, 1.0, 1.0),
        ];
        let eb: [EdgeId; 4] = [
            topo.add_edge(Edge::new(v[0], v[1], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[1], v[2], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[2], v[3], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[3], v[0], EdgeCurve::Line)),
        ];
        let et: [EdgeId; 4] = [
            topo.add_edge(Edge::new(v[4], v[5], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[5], v[6], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[6], v[7], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[7], v[4], EdgeCurve::Line)),
        ];
        let ev: [EdgeId; 4] = [
            topo.add_edge(Edge::new(v[0], v[4], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[1], v[5], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[2], v[6], EdgeCurve::Line)),
            topo.add_edge(Edge::new(v[3], v[7], EdgeCurve::Line)),
        ];

        let bottom = face_from(
            topo,
            vec![
                (eb[0], false),
                (eb[3], false),
                (eb[2], false),
                (eb[1], false),
            ],
            Vec3::new(0.0, 0.0, -1.0),
            0.0,
        );
        let top = face_from(
            topo,
            vec![(et[0], true), (et[1], true), (et[2], true), (et[3], true)],
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        );
        let front = face_from(
            topo,
            vec![(eb[0], true), (ev[1], true), (et[0], false), (ev[0], false)],
            Vec3::new(0.0, -1.0, 0.0),
            0.0,
        );
        let back = face_from(
            topo,
            vec![(eb[2], true), (ev[3], true), (et[2], false), (ev[2], false)],
            Vec3::new(0.0, 1.0, 0.0),
            1.0,
        );
        let left = face_from(
            topo,
            vec![(eb[3], true), (ev[0], true), (et[3], false), (ev[3], false)],
            Vec3::new(-1.0, 0.0, 0.0),
            0.0,
        );
        let right = face_from(
            topo,
            vec![(eb[1], true), (ev[2], true), (et[1], false), (ev[1], false)],
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        );

        let shell = topo.add_shell(
            Shell::new(vec![bottom, top, front, back, left, right]).expect("cube shell"),
        );
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        let mut edges = Vec::new();
        edges.extend_from_slice(&eb);
        edges.extend_from_slice(&et);
        edges.extend_from_slice(&ev);
        (solid, edges)
    }

    /// Two quads meeting along one edge: an open sheet with 6 free edges and
    /// no non-manifold edge — the one configuration that violates exactly one
    /// of `is_manifold`'s two conditions.
    fn open_two_quad_sheet(topo: &mut Topology) -> (Vec<FaceId>, EdgeId) {
        let v0 = vert(topo, 0.0, 0.0, 0.0);
        let v1 = vert(topo, 1.0, 0.0, 0.0);
        let v2 = vert(topo, 1.0, 1.0, 0.0);
        let v3 = vert(topo, 0.0, 1.0, 0.0);
        let v5 = vert(topo, 1.0, 0.0, 1.0);
        let v6 = vert(topo, 1.0, 1.0, 1.0);

        let a0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let shared = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let a2 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
        let a3 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));
        let b1 = topo.add_edge(Edge::new(v2, v6, EdgeCurve::Line));
        let b2 = topo.add_edge(Edge::new(v6, v5, EdgeCurve::Line));
        let b3 = topo.add_edge(Edge::new(v5, v1, EdgeCurve::Line));

        let fa = face_from(
            topo,
            vec![(a0, true), (shared, true), (a2, true), (a3, true)],
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let fb = face_from(
            topo,
            vec![(shared, false), (b1, true), (b2, true), (b3, true)],
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        );
        (vec![fa, fb], shared)
    }

    /// Three triangles fanned around one shared edge: a T-junction.
    fn non_manifold_fan(topo: &mut Topology) -> (Vec<FaceId>, EdgeId) {
        let v0 = vert(topo, 0.0, 0.0, 0.0);
        let v1 = vert(topo, 1.0, 0.0, 0.0);
        let apexes = [
            vert(topo, 0.5, 1.0, 0.0),
            vert(topo, 0.5, -1.0, 0.0),
            vert(topo, 0.5, 0.0, 1.0),
        ];
        let shared = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        let mut faces = Vec::new();
        for apex in apexes {
            let e_a = topo.add_edge(Edge::new(v1, apex, EdgeCurve::Line));
            let e_b = topo.add_edge(Edge::new(apex, v0, EdgeCurve::Line));
            faces.push(face_from(
                topo,
                vec![(shared, true), (e_a, true), (e_b, true)],
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            ));
        }
        (faces, shared)
    }

    #[test]
    fn closed_manifold_cube_index() {
        let mut topo = Topology::new();
        let (solid, edges) = manifold_cube(&mut topo);
        let adj = AdjacencyIndex::build(&topo, solid).unwrap();

        assert!(adj.is_manifold(), "a closed cube is manifold");
        assert!(adj.non_manifold_edges().is_empty());
        assert!(adj.boundary_edges().is_empty());
        assert_eq!(adj.edge_count(), 12, "a cube has 12 distinct edges");
        assert_eq!(
            adj.edge_faces_iter().count(),
            12,
            "the iterator must yield every indexed edge"
        );
        for (_, faces) in adj.edge_faces_iter() {
            assert_eq!(faces.len(), 2, "every cube edge is used by 2 faces");
        }

        for &eid in &edges {
            assert_eq!(
                adj.faces_for_edge(eid).len(),
                2,
                "edge {} should map to both of its faces",
                eid.index()
            );
            let looked_up = adj.edge_faces(eid).expect("known edge must be present");
            assert_eq!(looked_up.len(), 2);
            assert_eq!(looked_up, adj.faces_for_edge(eid));
        }

        let face_ids = topo
            .shell(topo.solid(solid).unwrap().outer_shell())
            .unwrap();
        let face_ids = face_ids.faces().to_vec();
        assert_eq!(face_ids.len(), 6);
        for &fid in &face_ids {
            let neighbors = adj.neighbors_of_face(fid);
            assert_eq!(neighbors.len(), 4, "each cube face touches 4 others");
            assert!(
                neighbors.iter().all(|n| n.index() != fid.index()),
                "a face is never its own neighbour"
            );
        }

        // An edge the index never saw is absent, not empty-but-present.
        let dangling = {
            let a = vert(&mut topo, 9.0, 9.0, 9.0);
            let b = vert(&mut topo, 9.0, 9.0, 10.0);
            topo.add_edge(Edge::new(a, b, EdgeCurve::Line))
        };
        assert!(adj.edge_faces(dangling).is_none());
        assert!(adj.faces_for_edge(dangling).is_empty());
    }

    #[test]
    fn open_sheet_has_free_edges_and_is_not_manifold() {
        let mut topo = Topology::new();
        let (faces, shared) = open_two_quad_sheet(&mut topo);
        let adj = AdjacencyIndex::build_from_faces(&topo, &faces).unwrap();

        // Exactly one of `is_manifold`'s two conditions is violated here: the
        // non-manifold list is empty but free edges exist.
        assert!(
            adj.non_manifold_edges().is_empty(),
            "no edge is used by 3+ faces"
        );
        assert_eq!(adj.boundary_edges().len(), 6, "6 free edges on the rim");
        assert!(
            !adj.is_manifold(),
            "an open sheet with free edges is not manifold"
        );

        assert_eq!(adj.edge_count(), 7, "4 + 4 edges with one shared");
        assert_eq!(adj.faces_for_edge(shared).len(), 2);
        assert_eq!(
            adj.edge_faces(shared).expect("shared edge indexed").len(),
            2
        );

        for (eid, used_by) in adj.edge_faces_iter() {
            if eid.index() == shared.index() {
                continue;
            }
            assert_eq!(used_by.len(), 1, "rim edges have a single face use");
            assert!(
                adj.boundary_edges()
                    .iter()
                    .any(|b| b.index() == eid.index()),
                "edge {} with one face use must be reported free",
                eid.index()
            );
        }

        assert_eq!(adj.neighbors_of_face(faces[0]), &[faces[1]][..]);
        assert_eq!(adj.neighbors_of_face(faces[1]), &[faces[0]][..]);
    }

    #[test]
    fn t_junction_edge_is_reported_non_manifold() {
        let mut topo = Topology::new();
        let (faces, shared) = non_manifold_fan(&mut topo);
        let adj = AdjacencyIndex::build_from_faces(&topo, &faces).unwrap();

        assert_eq!(adj.non_manifold_edges().len(), 1);
        assert_eq!(adj.non_manifold_edges()[0].index(), shared.index());
        assert!(!adj.is_manifold(), "a 3-face edge is not manifold");
        assert_eq!(
            adj.faces_for_edge(shared).len(),
            3,
            "the fan edge is used by all three triangles"
        );
        assert_eq!(adj.edge_faces(shared).expect("indexed").len(), 3);
        assert_eq!(adj.boundary_edges().len(), 6, "two free edges per triangle");
        assert_eq!(adj.edge_count(), 7, "1 shared + 6 rim edges");

        // A 3-face edge makes no face-neighbour links: only the 2-face arm does.
        for &fid in &faces {
            assert!(
                adj.neighbors_of_face(fid).is_empty(),
                "a non-manifold edge must not create neighbour links"
            );
        }
    }
}
