//! Loop — an ordered cycle of coedge uses bounding one face.
//!
//! See `docs/design/rfc-0002-coedge-architecture.md`. Loops are authoritative
//! face boundaries. Topology-owned mutation keeps wires synchronized as a
//! compatibility view; any disagreement is a validation error, never
//! tolerated dual authority.

use crate::arena;
use crate::coedge::CoedgeId;
use crate::face::FaceId;

/// Typed handle for a [`Loop`] stored in an [`Arena`](crate::Arena).
pub type LoopId = arena::Id<Loop>;

/// An ordered cycle of coedge uses bounding one face.
#[derive(Debug, Clone)]
pub struct Loop {
    /// The owning face.
    face: FaceId,
    /// Ordered traversal: adjacent coedges connect end vertex to start
    /// vertex under their orientations.
    coedges: Vec<CoedgeId>,
    /// Whether the loop closes back on itself.
    closed: bool,
}

impl Loop {
    /// Creates a new loop.
    #[must_use]
    pub const fn new(face: FaceId, coedges: Vec<CoedgeId>, closed: bool) -> Self {
        Self {
            face,
            coedges,
            closed,
        }
    }

    /// The owning face.
    #[must_use]
    pub const fn face(&self) -> FaceId {
        self.face
    }

    /// The ordered coedge uses of this loop.
    #[must_use]
    pub fn coedges(&self) -> &[CoedgeId] {
        &self.coedges
    }

    /// Whether the loop closes back on itself.
    #[must_use]
    pub const fn is_closed(&self) -> bool {
        self.closed
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use remus_math::curves::Circle3D;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point3, Vec3};

    use crate::edge::{Edge, EdgeCurve, EdgeId};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::topology::Topology;
    use crate::validation::{validate_face_loops, validate_loop_connected};
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};
    use crate::{TopologyError, WireId};

    /// A unit cylinder's side face whose boundary wire uses one seam edge
    /// twice — the RFC 0002 characterization configuration.
    fn seam_face(topo: &mut Topology) -> (EdgeId, FaceId) {
        let v_bottom = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v_top = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 1.0), 1e-7));
        let seam = topo.add_edge(Edge::new(v_bottom, v_top, EdgeCurve::Line));
        let bottom = topo.add_edge(Edge::new(
            v_bottom,
            v_bottom,
            EdgeCurve::Circle(
                Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap(),
            ),
        ));
        let top = topo.add_edge(Edge::new(
            v_top,
            v_top,
            EdgeCurve::Circle(
                Circle3D::new(Point3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap(),
            ),
        ));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(top, true),
                    OrientedEdge::new(seam, false),
                    OrientedEdge::new(bottom, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                    .unwrap(),
            ),
        ));
        (seam, face)
    }

    fn triangle_face(topo: &mut Topology) -> (FaceId, WireId) {
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e2, true),
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
                d: 0.0,
            },
        ));
        (face, wire)
    }

    #[test]
    fn seam_edge_derives_two_independent_coedges() {
        // The RFC 0002 Stage 1 exit gate: one 3D edge, two per-use
        // identities with opposite orientations in the same loop.
        let mut topo = Topology::new();
        let (seam, face) = seam_face(&mut topo);

        let loops = topo.build_face_loops(face).unwrap();
        assert_eq!(loops.len(), 1);

        let uses = topo.coedges_of_edge(seam);
        assert_eq!(uses.len(), 2, "the seam edge must have two coedge uses");
        let a = topo.coedge(uses[0]).unwrap();
        let b = topo.coedge(uses[1]).unwrap();
        assert_ne!(a.is_forward(), b.is_forward());
        assert_eq!(a.parent_loop(), loops[0]);
        assert_eq!(b.parent_loop(), loops[0]);

        validate_face_loops(&topo, face).unwrap();
        validate_loop_connected(&topo, loops[0]).unwrap();
    }

    #[test]
    fn derivation_mirrors_wire_order_and_orientation() {
        let mut topo = Topology::new();
        let (face, wire) = triangle_face(&mut topo);
        let loops = topo.build_face_loops(face).unwrap();

        let boundary_loop = topo.face_loop(loops[0]).unwrap();
        let wire_edges: Vec<_> = topo.wire(wire).unwrap().edges().to_vec();
        assert_eq!(boundary_loop.coedges().len(), wire_edges.len());
        for (&coedge_id, oriented) in boundary_loop.coedges().iter().zip(&wire_edges) {
            let coedge = topo.coedge(coedge_id).unwrap();
            assert_eq!(coedge.edge(), oriented.edge());
            assert_eq!(coedge.is_forward(), oriented.is_forward());
        }
        assert_eq!(
            topo.face_oriented_edges(face)
                .unwrap()
                .iter()
                .map(|oriented| (oriented.edge(), oriented.is_forward()))
                .collect::<Vec<_>>(),
            wire_edges
                .iter()
                .map(|oriented| (oriented.edge(), oriented.is_forward()))
                .collect::<Vec<_>>()
        );
        validate_face_loops(&topo, face).unwrap();
        validate_loop_connected(&topo, loops[0]).unwrap();
    }

    #[test]
    fn valid_face_allocation_installs_authoritative_loops() {
        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        let loops = topo.loops_of_face(face).unwrap();
        assert_eq!(loops.len(), 1);
        assert_eq!(topo.face_loop(loops[0]).unwrap().coedges().len(), 3);
        validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn cloned_face_gets_fresh_owned_loop_handles() {
        let mut topo = Topology::new();
        let (source, _) = triangle_face(&mut topo);
        let source_loop = topo.face(source).unwrap().outer_loop().unwrap();
        let clone = topo.face(source).unwrap().clone();

        let copied = topo.add_face(clone);

        let copied_loop = topo.face(copied).unwrap().outer_loop().unwrap();
        assert_ne!(copied_loop, source_loop);
        assert_eq!(topo.face_loop(source_loop).unwrap().face(), source);
        assert_eq!(topo.face_loop(copied_loop).unwrap().face(), copied);
        validate_face_loops(&topo, source).unwrap();
        validate_face_loops(&topo, copied).unwrap();
    }

    #[test]
    fn compatibility_builder_preserves_authoritative_handles() {
        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        let first = topo.build_face_loops(face).unwrap();
        let first_coedges = topo.face_loop(first[0]).unwrap().coedges().to_vec();

        let second = topo.build_face_loops(face).unwrap();
        assert_eq!(first, second);
        for coedge_id in first_coedges {
            assert!(topo.coedge(coedge_id).is_ok());
        }
        assert_eq!(topo.num_loops(), 1);
        assert_eq!(topo.num_coedges(), 3);
    }

    #[test]
    #[allow(deprecated)]
    fn wire_mutation_after_derivation_is_flagged_as_mismatch() {
        // Stage 1 invariant: wires are authoritative; a stale derivation is
        // a typed validation error, never a tolerated state.
        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        topo.build_face_loops(face).unwrap();

        // Point the face at a different (single-edge) wire.
        let v = topo.add_vertex(Vertex::new(Point3::new(5.0, 5.0, 0.0), 1e-7));
        let e = topo.add_edge(Edge::new(v, v, EdgeCurve::Line));
        let other_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
        topo.face_mut(face).unwrap().set_outer_wire(other_wire);

        assert!(matches!(
            validate_face_loops(&topo, face),
            Err(TopologyError::LoopWireMismatch { .. })
        ));
    }

    #[test]
    fn disconnected_loop_fails_connectivity() {
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(5.0, 5.0, 0.0), 1e-7));
        let v3 = topo.add_vertex(Vertex::new(Point3::new(6.0, 5.0, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let loops = topo.build_face_loops(face).unwrap();
        assert!(matches!(
            validate_loop_connected(&topo, loops[0]),
            Err(TopologyError::LoopNotConnected { .. })
        ));
    }

    #[test]
    fn restore_retires_post_snapshot_derivations() {
        let snapshot = Topology::new();
        let mut topo = snapshot.clone();
        let (face, _) = triangle_face(&mut topo);
        let loops = topo.loops_of_face(face).unwrap().to_vec();
        topo.restore_preserving_handle_slots(&snapshot);

        assert!(topo.loops_of_face(face).is_none());
        assert!(matches!(
            topo.face_loop(loops[0]),
            Err(TopologyError::LoopNotFound(_))
        ));
        assert_eq!(topo.num_loops(), 0);
        assert_eq!(topo.num_coedges(), 0);
    }

    #[test]
    fn restore_after_post_snapshot_rederivation_leaves_no_dangling_map() {
        // Checkpoint barrier semantics: authority handles retired after the
        // snapshot stay retired, so the restored face is promoted onto fresh
        // handles rather than left without a boundary.
        let mut topo = Topology::new();
        let (face, wire) = triangle_face(&mut topo);
        let retired = topo.loops_of_face(face).unwrap().to_vec();
        let retired_coedges = topo.face_loop(retired[0]).unwrap().coedges().to_vec();
        let snapshot = topo.clone();

        let reversed: Vec<_> = topo
            .wire(wire)
            .unwrap()
            .edges()
            .iter()
            .rev()
            .map(|oriented| OrientedEdge::new(oriented.edge(), !oriented.is_forward()))
            .collect();
        topo.replace_boundary_wire(wire, Wire::new(reversed, true).unwrap())
            .unwrap();
        topo.restore_preserving_handle_slots(&snapshot);

        assert!(matches!(
            topo.face_loop(retired[0]),
            Err(TopologyError::LoopNotFound(_))
        ));
        for coedge_id in &retired_coedges {
            assert!(matches!(
                topo.coedge(*coedge_id),
                Err(TopologyError::CoedgeNotFound(_))
            ));
        }
        let rebuilt = topo.loops_of_face(face).unwrap();
        assert_ne!(rebuilt[0], retired[0]);
        assert_eq!(topo.num_loops(), 1);
        assert_eq!(topo.num_coedges(), 3);
        validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn new_diagnostic_codes_are_pinned() {
        use remus_math::diagnostic::{FailureCategory, ToDiagnostic};

        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        let d = TopologyError::LoopWireMismatch { face }.diagnostic();
        assert_eq!(d.category(), FailureCategory::Internal);
        assert_eq!(d.code(), "loop_wire_mismatch");

        let d = TopologyError::LoopNotConnected { face }.diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidTopology);
        assert_eq!(d.code(), "loop_not_connected");
    }
}
