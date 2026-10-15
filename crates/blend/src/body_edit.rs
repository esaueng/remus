//! Copy-on-write boundary rewrites for one blend build.
//!
//! A blend result keeps every input face it does not trim, so the input solid
//! and the result share those faces and their wires. Splitting an edge where a
//! contact line crosses it, or stitching an end arc into a cap, must change the
//! result's face and leave the input's alone: rewriting the shared wire in
//! place notched the input's caps while its side faces stayed whole, opening
//! the input solid behind a successful blend.
//!
//! [`EditScope`] records what a build was handed; [`BodyEdit`] applies a wire
//! rewrite to the body under construction only:
//!
//! - a face or wire the build allocated belongs to no one else and is
//!   rewritten in place;
//! - an input face still in the result is copied first — the copy carries the
//!   rewritten wires plus the pcurve and periodic winding of every edge use it
//!   keeps — and the copy replaces it in the result;
//! - anything else (an input face already replaced, another solid's face, a
//!   free wire) is left untouched.
//!
//! The same boundary keeps vertex reuse inside the body: a contact point
//! snaps only to a vertex of the input or one the build minted, never to a
//! coincident vertex of another solid.

use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::coedge::PeriodicWinding;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::face_vertices;
use remus_topology::face::{Face, FaceId};
use remus_topology::pcurve::PCurve;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{Wire, WireId};

use crate::BlendError;

/// What one blend build was handed, fixed before it allocates anything.
pub struct EditScope {
    /// Face, wire and vertex slot counts when the build began; arena slots
    /// are never reused, so an entity at or above them was allocated by the
    /// build.
    face_mark: usize,
    wire_mark: usize,
    vertex_mark: usize,
    /// The faces of the input body.
    input_faces: DetHashSet<FaceId>,
    /// The vertices of the input body.
    input_vertices: DetHashSet<VertexId>,
}

impl EditScope {
    /// Open the scope of a build on the body made of `input_faces`.
    ///
    /// Call before the build allocates any face, wire or vertex.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError`] if an input face's boundary cannot be read.
    pub fn new(
        topo: &Topology,
        input_faces: impl IntoIterator<Item = FaceId>,
    ) -> Result<Self, BlendError> {
        let input_faces: DetHashSet<FaceId> = input_faces.into_iter().collect();
        let mut input_vertices = DetHashSet::default();
        for &face in &input_faces {
            input_vertices.extend(face_vertices(topo, face)?);
        }
        Ok(Self {
            face_mark: topo.faces().slot_len(),
            wire_mark: topo.wires().slot_len(),
            vertex_mark: topo.vertices().slot_len(),
            input_faces,
            input_vertices,
        })
    }

    /// The body under construction: the input body with each input face
    /// replaced as `replacements` records (input face → its current face),
    /// plus every face the build allocated.
    pub fn edit<'a>(&'a self, replacements: &'a mut DetHashMap<FaceId, FaceId>) -> BodyEdit<'a> {
        BodyEdit {
            scope: self,
            replacements,
        }
    }

    fn allocated_face(&self, face: FaceId) -> bool {
        face.index() >= self.face_mark
    }

    fn allocated_wire(&self, wire: WireId) -> bool {
        wire.index() >= self.wire_mark
    }
}

/// Copy-on-write access to the body under construction; see the module docs.
pub struct BodyEdit<'a> {
    scope: &'a EditScope,
    replacements: &'a mut DetHashMap<FaceId, FaceId>,
}

impl BodyEdit<'_> {
    /// Input face → its current face in the result.
    pub fn replacements(&self) -> &DetHashMap<FaceId, FaceId> {
        self.replacements
    }

    fn owns(&self, face: FaceId) -> bool {
        self.scope.allocated_face(face)
            || (self.scope.input_faces.contains(&face) && !self.replacements.contains_key(&face))
    }

    /// The body's vertex at `point` (within `tolerance`), or a new one.
    ///
    /// Contact splits on shared boundary edges must reuse one vertex per
    /// position: two trims splitting the same edge at the same point would
    /// otherwise mint coincident vertices, and the two resulting contact
    /// edges would meet the cap wire at different ids — leaving use-1 stubs
    /// the end-cap notch cannot share. Only the body's own vertices qualify:
    /// a coincident vertex of another solid (an earlier blend of the same
    /// input, say) would cross-link the two bodies.
    pub fn find_or_add_vertex(
        &self,
        topo: &mut Topology,
        point: Point3,
        tolerance: f64,
    ) -> VertexId {
        for (vid, vertex) in topo.vertices().iter() {
            let in_body =
                vid.index() >= self.scope.vertex_mark || self.scope.input_vertices.contains(&vid);
            if in_body && (vertex.point() - point).length() <= tolerance {
                return vid;
            }
        }
        topo.add_vertex(Vertex::new(point, tolerance))
    }

    /// Rewrite boundary wires of the body under construction.
    ///
    /// `rewrite` sees each wire of each face the body owns and returns the
    /// replacement for a wire it changes. `skip` names a face the caller is
    /// about to replace anyway (the face being trimmed), which would otherwise
    /// be copied only to be orphaned. Returns the faces that now carry a
    /// rewritten wire: copies in place of the input faces they replaced.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError`] if `rewrite` fails or a rewritten boundary
    /// references a missing entity.
    pub fn rewrite_wires(
        &mut self,
        topo: &mut Topology,
        skip: Option<FaceId>,
        mut rewrite: impl FnMut(WireId, &Wire) -> Result<Option<Wire>, BlendError>,
    ) -> Result<Vec<FaceId>, BlendError> {
        // Plan against one consistent state, then apply.
        let mut plans: Vec<(FaceId, Vec<(WireId, Wire)>)> = Vec::new();
        for (face_id, face) in topo.faces().iter() {
            if Some(face_id) == skip || !self.owns(face_id) {
                continue;
            }
            let mut changed = Vec::new();
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                if let Some(wire) = rewrite(wire_id, topo.wire(wire_id)?)? {
                    changed.push((wire_id, wire));
                }
            }
            if !changed.is_empty() {
                plans.push((face_id, changed));
            }
        }

        let mut rewritten = Vec::with_capacity(plans.len());
        let mut rewritten_in_place: DetHashSet<WireId> = DetHashSet::default();
        for (face_id, changed) in plans {
            let allocated_face = self.scope.allocated_face(face_id);
            let mut substitutes: Vec<(WireId, WireId)> = Vec::new();
            for (wire_id, wire) in changed {
                // Only faces this build allocated can reference a wire it
                // allocated, so such a wire is rewritten for all its users.
                if allocated_face && self.scope.allocated_wire(wire_id) {
                    if rewritten_in_place.insert(wire_id) {
                        topo.replace_boundary_wire(wire_id, wire)?;
                    }
                } else {
                    substitutes.push((wire_id, topo.add_wire(wire)));
                }
            }
            if substitutes.is_empty() {
                rewritten.push(face_id);
                continue;
            }
            let face = topo.face(face_id)?;
            let swap = |wire: WireId| {
                substitutes
                    .iter()
                    .find(|(old, _)| *old == wire)
                    .map_or(wire, |&(_, new)| new)
            };
            let outer = swap(face.outer_wire());
            let inners: Vec<WireId> = face.inner_wires().iter().map(|&wire| swap(wire)).collect();
            if allocated_face {
                topo.set_face_boundary_wires(face_id, outer, inners)?;
                rewritten.push(face_id);
            } else {
                let copy = copy_face(topo, face_id, outer, inners)?;
                self.replacements.insert(face_id, copy);
                rewritten.push(copy);
            }
        }
        Ok(rewritten)
    }
}

/// A new face on `source`'s surface and orientation bounded by `outer` and
/// `inners`, keeping `source`'s pcurve and periodic winding for every edge
/// use the new boundary still makes — what an in-place wire rewrite keeps.
fn copy_face(
    topo: &mut Topology,
    source: FaceId,
    outer: WireId,
    inners: Vec<WireId>,
) -> Result<FaceId, BlendError> {
    let face = topo.face(source)?;
    let mut copy = Face::new(outer, inners, face.surface().clone());
    copy.set_reversed(face.is_reversed());

    let mut authority: DetHashMap<(EdgeId, bool), (Option<PCurve>, PeriodicWinding)> =
        DetHashMap::default();
    for &loop_id in topo.loops_of_face(source).unwrap_or_default() {
        for &coedge_id in topo.face_loop(loop_id)?.coedges() {
            let coedge = topo.coedge(coedge_id)?;
            authority.insert(
                (coedge.edge(), coedge.is_forward()),
                (coedge.pcurve().cloned(), coedge.periodic_winding()),
            );
        }
    }

    let copy_id = topo.add_face(copy);
    let mut coedges = Vec::new();
    for &loop_id in topo.loops_of_face(copy_id).unwrap_or_default() {
        coedges.extend(topo.face_loop(loop_id)?.coedges().iter().copied());
    }
    for coedge_id in coedges {
        let coedge = topo.coedge(coedge_id)?;
        let Some((pcurve, winding)) = authority
            .get(&(coedge.edge(), coedge.is_forward()))
            .cloned()
        else {
            continue;
        };
        topo.set_coedge_periodic_winding(coedge_id, winding)?;
        if let Some(pcurve) = pcurve {
            topo.set_coedge_pcurve(coedge_id, pcurve)?;
        }
    }
    Ok(copy_id)
}
