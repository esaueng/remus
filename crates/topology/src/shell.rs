//! Shell — a connected set of faces forming a surface boundary.

use crate::TopologyError;
use crate::arena;
use crate::face::FaceId;
use crate::topology::BodyClass;

/// Typed handle for a [`Shell`] stored in an [`Arena`](crate::Arena).
pub type ShellId = arena::Id<Shell>;

/// A topological shell: a connected set of faces.
///
/// A closed shell bounds a volume (solid). An open shell represents
/// a sheet or partial boundary.
#[derive(Debug, Clone)]
pub struct Shell {
    /// The faces that make up this shell.
    faces: Vec<FaceId>,
    /// Whether this shell belongs to a solid or is itself a sheet body.
    body_class: BodyClass,
}

impl Shell {
    /// Creates a new shell from a non-empty list of faces.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::Empty`] if `faces` is empty.
    pub fn new(faces: Vec<FaceId>) -> Result<Self, TopologyError> {
        if faces.is_empty() {
            return Err(TopologyError::Empty { entity: "shell" });
        }
        Ok(Self {
            faces,
            body_class: BodyClass::Solid,
        })
    }

    /// Creates a faceless shell backing an empty-result sentinel.
    ///
    /// A regular shell rejects an empty face list because an ordinary
    /// surface boundary must enclose something. The empty shell exists
    /// only so a boolean whose algebraic outcome is the empty set
    /// (e.g. the intersection of disjoint solids) can be represented as
    /// a valid, queryable solid handle reporting zero faces and zero
    /// volume — distinct from a malformed-input error.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            faces: Vec::new(),
            body_class: BodyClass::Solid,
        }
    }

    /// Returns `true` when this shell has no faces (the empty-result
    /// sentinel — see [`Shell::empty`]).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }

    /// Returns the faces of this shell.
    #[must_use]
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    /// Returns mutable access to the faces of this shell.
    ///
    /// Allows in-place mutation (reorder, replace) but not removal.
    /// The shell must always contain at least one face.
    pub fn faces_mut(&mut self) -> &mut [FaceId] {
        &mut self.faces
    }

    /// Returns the dimensional class stored on this shell.
    #[must_use]
    pub const fn body_class(&self) -> BodyClass {
        self.body_class
    }

    pub(crate) fn set_body_class(&mut self, body_class: BodyClass) {
        self.body_class = body_class;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::vec::{Point3, Vec3};

    use crate::edge::{Edge, EdgeCurve};
    use crate::face::{Face, FaceSurface};
    use crate::topology::Topology;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    use super::*;

    /// One planar triangular face at height `z`.
    fn triangle_face(topo: &mut Topology, z: f64) -> FaceId {
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
    fn is_empty_separates_the_sentinel_from_a_real_shell() {
        assert!(
            Shell::empty().is_empty(),
            "the empty-result sentinel reports zero faces"
        );

        let mut topo = Topology::new();
        let face = triangle_face(&mut topo, 0.0);
        let shell = Shell::new(vec![face]).unwrap();
        assert!(!shell.is_empty(), "a shell holding a face is not empty");
        assert_eq!(shell.faces(), &[face]);
    }

    #[test]
    fn faces_mut_exposes_the_stored_faces_for_in_place_replacement() {
        let mut topo = Topology::new();
        let f0 = triangle_face(&mut topo, 0.0);
        let f1 = triangle_face(&mut topo, 1.0);
        assert_ne!(f0, f1);

        let mut shell = Shell::new(vec![f0, f1]).unwrap();
        {
            let faces = shell.faces_mut();
            assert_eq!(faces.len(), 2, "faces_mut sees the shell's own storage");
            faces[0] = f1;
        }

        // The write through the mutable slice is visible to the shared view.
        assert_eq!(shell.faces(), &[f1, f1]);
    }
}
