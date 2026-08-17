//! Loop — an ordered cycle of coedge uses bounding one face.
//!
//! See `docs/design/rfc-0002-coedge-architecture.md`. In Stage 1 loops are
//! derived from a face's wires on request and wires remain authoritative;
//! a loop that disagrees with its source wire is a validation error, never
//! a tolerated state.

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
        validate_face_loops(&topo, face).unwrap();
        validate_loop_connected(&topo, loops[0]).unwrap();
    }

    #[test]
    fn underived_face_passes_vacuously_and_reports_no_loops() {
        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        assert!(topo.loops_of_face(face).is_none());
        validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn rebuilding_retires_the_previous_derivation() {
        // No slot reuse: the first derivation's handles become permanently
        // invalid; live counts do not grow.
        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        let first = topo.build_face_loops(face).unwrap();
        let first_coedges = topo.face_loop(first[0]).unwrap().coedges().to_vec();

        let second = topo.build_face_loops(face).unwrap();
        assert_ne!(first[0], second[0]);
        assert!(matches!(
            topo.face_loop(first[0]),
            Err(TopologyError::LoopNotFound(_))
        ));
        for stale in first_coedges {
            assert!(matches!(
                topo.coedge(stale),
                Err(TopologyError::CoedgeNotFound(_))
            ));
        }
        assert_eq!(topo.num_loops(), 1);
        assert_eq!(topo.num_coedges(), 3);
    }

    #[test]
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
        let mut topo = Topology::new();
        let (face, _) = triangle_face(&mut topo);
        let snapshot = topo.clone();

        let loops = topo.build_face_loops(face).unwrap();
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
