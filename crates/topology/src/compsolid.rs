//! `CompSolid` — a set of solids sharing faces.
//!
//! A `CompSolid` represents multiple solids that share boundary faces
//! (e.g., two volumes separated by a common wall). This is the 8th
//! topology type between Solid and Compound in the B-Rep hierarchy.

use crate::arena;
use crate::face::FaceId;
use crate::solid::SolidId;

/// Typed handle for a [`CompSolid`] stored in an [`Arena`](crate::Arena).
pub type CompSolidId = arena::Id<CompSolid>;

/// A topological `CompSolid`: a set of solids sharing faces.
///
/// Unlike a [`Compound`](crate::compound::Compound), which is a loose
/// collection, a `CompSolid` implies topological connectivity through
/// shared boundary faces.
#[derive(Debug, Clone)]
pub struct CompSolid {
    /// The solids in this comp-solid.
    solids: Vec<SolidId>,
    /// Faces shared between adjacent solids.
    shared_faces: Vec<FaceId>,
}

impl CompSolid {
    /// Creates a new comp-solid from the given solids and shared faces.
    #[must_use]
    pub const fn new(solids: Vec<SolidId>, shared_faces: Vec<FaceId>) -> Self {
        Self {
            solids,
            shared_faces,
        }
    }

    /// Returns the solids in this comp-solid.
    #[must_use]
    pub fn solids(&self) -> &[SolidId] {
        &self.solids
    }

    /// Returns the faces shared between solids.
    #[must_use]
    pub fn shared_faces(&self) -> &[FaceId] {
        &self.shared_faces
    }

    /// Returns the number of solids.
    #[must_use]
    pub fn num_solids(&self) -> usize {
        self.solids().len()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_math::vec::Point3;

    use crate::topology::Topology;
    use crate::vertex::Vertex;

    use super::*;

    #[test]
    fn create_empty_compsolid() {
        let cs = CompSolid::new(vec![], vec![]);
        assert_eq!(cs.num_solids(), 0);
        assert!(cs.shared_faces().is_empty());
    }

    #[test]
    fn compsolid_in_arena() {
        let mut topo = Topology::new();

        let _v = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));

        let cs = CompSolid::new(vec![], vec![]);
        let cs_id = topo.add_compsolid(cs);

        let retrieved = topo.compsolid(cs_id).unwrap();
        assert_eq!(retrieved.num_solids(), 0);
    }

    /// One planar triangular face at height `z`.
    fn triangle_face(topo: &mut Topology, z: f64) -> FaceId {
        use remus_math::vec::Vec3;

        use crate::edge::{Edge, EdgeCurve};
        use crate::face::{Face, FaceSurface};
        use crate::wire::{OrientedEdge, Wire};

        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, z), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, z), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, z), 1e-7));
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
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: z,
            },
        ))
    }

    #[test]
    fn populated_compsolid_reports_its_solids_and_shared_faces() {
        use crate::shell::Shell;
        use crate::solid::Solid;

        let mut topo = Topology::new();
        let f0 = triangle_face(&mut topo, 0.0);
        let f1 = triangle_face(&mut topo, 1.0);
        assert_ne!(f0, f1);

        let shell_a = topo.add_shell(Shell::empty());
        let shell_b = topo.add_shell(Shell::empty());
        let s0 = topo.add_solid(Solid::new(shell_a, vec![]));
        let s1 = topo.add_solid(Solid::new(shell_b, vec![]));
        assert_ne!(s0, s1);

        let cs = CompSolid::new(vec![s0, s1], vec![f0, f1]);

        // Two solids: neither the `0` nor a `1` constant matches.
        assert_eq!(cs.num_solids(), 2);
        assert_eq!(cs.solids(), &[s0, s1]);
        assert_eq!(cs.shared_faces(), &[f0, f1]);
    }
}
