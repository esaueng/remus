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
//! coincident vertex of another solid. A spatial hash over those vertices
//! keeps each lookup independent of how many other bodies the arena holds.

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
    input_faces: Vec<FaceId>,
    /// The vertices of the input body.
    input_vertices: DetHashSet<VertexId>,
    /// Spatial hash over the body's vertices, built on the first lookup.
    vertex_grid: Option<VertexGrid>,
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
        // Arena order is part of rewrite ordering, regardless of the caller's
        // face order. All later allocations have indices above this prefix.
        let mut input_faces: Vec<FaceId> = input_faces.into_iter().collect();
        input_faces.sort_unstable_by_key(|face| face.index());
        input_faces.dedup();
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
            vertex_grid: None,
        })
    }

    /// The body under construction: the input body with each input face
    /// replaced as `replacements` records (input face → its current face),
    /// plus every face the build allocated.
    pub fn edit<'a>(
        &'a mut self,
        replacements: &'a mut DetHashMap<FaceId, FaceId>,
    ) -> BodyEdit<'a> {
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

    /// The vertex grid for `tolerance`, holding every vertex of the body.
    fn vertex_grid(&mut self, topo: &Topology, tolerance: f64) -> &mut VertexGrid {
        if self
            .vertex_grid
            .as_ref()
            .is_some_and(|grid| grid.tolerance.to_bits() != tolerance.to_bits())
        {
            self.vertex_grid = None;
        }
        let (mark, input_vertices) = (self.vertex_mark, &self.input_vertices);
        let grid = self.vertex_grid.get_or_insert_with(|| {
            let mut grid = VertexGrid::new(tolerance, mark);
            for &vertex in input_vertices {
                if let Ok(data) = topo.vertex(vertex) {
                    grid.insert(vertex, data.point());
                }
            }
            grid
        });
        grid.catch_up(topo);
        grid
    }
}

/// Uniform spatial hash of vertex positions.
///
/// Cells are twice the snap tolerance wide, so two points within tolerance
/// differ by at most half a cell per axis and land in the same or adjacent
/// cells, even after the rounding of the division: a lookup checks the 27
/// cells around its point and misses nothing.
struct VertexGrid {
    tolerance: f64,
    cell: f64,
    cells: DetHashMap<[i64; 3], Vec<VertexId>>,
    /// Vertex slots below this have been considered; the build's own vertices
    /// at or above it are added on the next lookup.
    indexed_to: usize,
}

impl VertexGrid {
    fn new(tolerance: f64, indexed_to: usize) -> Self {
        Self {
            tolerance,
            cell: 2.0 * tolerance,
            cells: DetHashMap::default(),
            indexed_to,
        }
    }

    // Saturating float-to-int casts are intended: a coordinate too far out for
    // the grid shares an edge cell, and the exact distance test still decides.
    #[allow(clippy::cast_possible_truncation)]
    fn key(&self, point: Point3) -> [i64; 3] {
        [point.x(), point.y(), point.z()].map(|c| (c / self.cell).floor() as i64)
    }

    fn insert(&mut self, vertex: VertexId, point: Point3) {
        self.cells.entry(self.key(point)).or_default().push(vertex);
    }

    /// Add every live vertex allocated since the last refresh.
    fn catch_up(&mut self, topo: &Topology) {
        let end = topo.vertices().slot_len();
        for index in self.indexed_to..end {
            if let Some(vertex) = topo.vertices().id_from_index(index)
                && let Ok(data) = topo.vertex(vertex)
            {
                self.insert(vertex, data.point());
            }
        }
        self.indexed_to = end;
    }

    /// The lowest-index vertex within `self.tolerance` of `point`: the one a
    /// scan in arena order would meet first.
    fn nearest(&self, topo: &Topology, point: Point3) -> Option<VertexId> {
        let [x, y, z] = self.key(point);
        let mut best: Option<VertexId> = None;
        for dx in -1..=1_i64 {
            for dy in -1..=1_i64 {
                for dz in -1..=1_i64 {
                    let key = [
                        x.saturating_add(dx),
                        y.saturating_add(dy),
                        z.saturating_add(dz),
                    ];
                    for &vertex in self.cells.get(&key).into_iter().flatten() {
                        let Ok(data) = topo.vertex(vertex) else {
                            continue;
                        };
                        if (data.point() - point).length() <= self.tolerance
                            && best.is_none_or(|b| vertex.index() < b.index())
                        {
                            best = Some(vertex);
                        }
                    }
                }
            }
        }
        best
    }
}

/// Copy-on-write access to the body under construction; see the module docs.
pub struct BodyEdit<'a> {
    scope: &'a mut EditScope,
    replacements: &'a mut DetHashMap<FaceId, FaceId>,
}

impl BodyEdit<'_> {
    /// Input face → its current face in the result.
    pub fn replacements(&self) -> &DetHashMap<FaceId, FaceId> {
        self.replacements
    }

    /// Enumerate the same owned faces as an arena scan, in arena order,
    /// without visiting other bodies or pre-build history. Recompute on
    /// every rewrite so copies and newly allocated faces are visible.
    fn candidate_faces<'a>(&'a self, topo: &'a Topology) -> impl Iterator<Item = FaceId> + 'a {
        self.scope
            .input_faces
            .iter()
            .copied()
            .filter(|face| !self.replacements.contains_key(face))
            .chain(
                (self.scope.face_mark..topo.faces().slot_len())
                    .filter_map(|index| topo.faces().id_from_index(index)),
            )
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
        &mut self,
        topo: &mut Topology,
        point: Point3,
        tolerance: f64,
    ) -> VertexId {
        let grid = self.scope.vertex_grid(topo, tolerance);
        if let Some(vertex) = grid.nearest(topo, point) {
            return vertex;
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
        for face_id in self.candidate_faces(topo) {
            #[cfg(test)]
            lookup_work::face();
            if Some(face_id) == skip {
                continue;
            }
            // Match arena iteration's treatment of retired slots.
            let Ok(face) = topo.face(face_id) else {
                continue;
            };
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

// Count candidates actually visited by the production lookup loops. These
// thread-local counters are absent from shipped builds and timing-independent.
#[cfg(test)]
pub mod lookup_work {
    use std::cell::Cell;

    std::thread_local! {
        static WORK: Cell<(usize, usize)> = const { Cell::new((0, 0)) };
    }

    pub fn face() {
        let (faces, wires) = WORK.get();
        WORK.set((faces + 1, wires));
    }

    pub fn wire() {
        let (faces, wires) = WORK.get();
        WORK.set((faces, wires + 1));
    }

    pub fn take() -> (usize, usize) {
        WORK.replace((0, 0))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::FaceSurface;
    use remus_topology::wire::OrientedEdge;

    use super::*;

    const TOL: f64 = 1e-7;

    /// The lookup the grid replaced: the first body vertex in arena order
    /// within `TOL` of `point`.
    fn scan(topo: &Topology, scope: &EditScope, point: Point3) -> Option<VertexId> {
        topo.vertices()
            .iter()
            .find(|(id, vertex)| {
                (id.index() >= scope.vertex_mark || scope.input_vertices.contains(id))
                    && (vertex.point() - point).length() <= TOL
            })
            .map(|(id, _)| id)
    }

    fn triangle(topo: &mut Topology, corners: [Point3; 3]) -> FaceId {
        let v = corners.map(|p| topo.add_vertex(Vertex::new(p, TOL)));
        let e = [0, 1, 2].map(|i| topo.add_edge(Edge::new(v[i], v[(i + 1) % 3], EdgeCurve::Line)));
        let wire = topo.add_wire(
            Wire::new(
                e.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn rewrite_candidates_match_arena_order_after_replacements() {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let foreign = triangle(&mut topo, corners);
        let a = triangle(&mut topo, corners);
        triangle(&mut topo, corners);
        let b = triangle(&mut topo, corners);
        // Both outer and inner wires must be offered to the rewrite.
        let inner = topo.face(foreign).unwrap().outer_wire();
        let outer = topo.face(b).unwrap().outer_wire();
        topo.set_face_boundary_wires(b, outer, vec![inner]).unwrap();
        let mut scope = EditScope::new(&topo, [b, a, b]).unwrap();
        let mut replacements = DetHashMap::default();
        let allocated = triangle(&mut topo, corners);

        for skip in [Some(a), None, Some(allocated)] {
            // The original full scan, including its ownership and skip rules.
            let expected: Vec<_> = topo
                .faces()
                .iter()
                .filter_map(|(id, face)| {
                    let owned = id.index() >= scope.face_mark
                        || (scope.input_faces.contains(&id) && !replacements.contains_key(&id));
                    (owned && Some(id) != skip).then(|| {
                        (
                            id,
                            std::iter::once(face.outer_wire())
                                .chain(face.inner_wires().iter().copied())
                                .collect::<Vec<_>>(),
                        )
                    })
                })
                .collect();
            let expected_wires: Vec<_> = expected
                .iter()
                .flat_map(|(_, wires)| wires.iter().copied())
                .collect();
            let mut visited = Vec::new();
            let rewritten = scope
                .edit(&mut replacements)
                .rewrite_wires(&mut topo, skip, |id, wire| {
                    visited.push(id);
                    Ok(Some(wire.clone()))
                })
                .unwrap();
            assert_eq!(visited, expected_wires);
            let expected_faces: Vec<_> = expected
                .iter()
                .map(|(id, _)| replacements.get(id).copied().unwrap_or(*id))
                .collect();
            assert_eq!(rewritten, expected_faces);
        }
        assert!(!replacements.contains_key(&foreign));
    }

    #[test]
    fn grid_lookup_matches_the_arena_scan_and_skips_other_bodies() {
        let mut topo = Topology::new();
        // Another body owns a vertex at the origin, ahead of everything else.
        let foreign = triangle(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(0.0, 5.0, 0.0),
            ],
        );
        let input = triangle(
            &mut topo,
            [
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
            ],
        );
        let mut scope = EditScope::new(&topo, [input]).unwrap();
        let mut replacements = DetHashMap::default();

        // Vertices the build mints around cell corners (cells are 2·TOL), a
        // few of them within TOL of each other so the lowest index must win.
        let cell = 2.0 * TOL;
        let mut state = 0x2545_f491_u64;
        let mut jitter = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            (f64::from(u32::try_from(state >> 40).unwrap()) / f64::from(1_u32 << 24) - 0.5)
                * 1.5
                * TOL
        };
        for i in 0..40 {
            let base = Point3::new(f64::from(i % 4) * cell, f64::from(i / 4 % 3) * cell, 0.0);
            let p = Point3::new(base.x() + jitter(), base.y() + jitter(), jitter());
            topo.add_vertex(Vertex::new(p, TOL));
        }

        let mut queries: Vec<Point3> = topo.vertices().iter().map(|(_, v)| v.point()).collect();
        for i in 0..200 {
            let base = Point3::new(f64::from(i % 5) * cell, f64::from(i / 5 % 4) * cell, 0.0);
            queries.push(Point3::new(
                base.x() + jitter(),
                base.y() + jitter(),
                jitter(),
            ));
        }
        queries.push(Point3::new(1.0, 1.0, 0.0));
        queries.push(Point3::new(1.0 + 0.9 * TOL, 1.0, 0.0));
        queries.push(Point3::new(1.0 + 1.1 * TOL, 1.0, 0.0));

        for point in queries {
            let expected = scan(&topo, &scope, point);
            let mut edit = scope.edit(&mut replacements);
            let before = topo.vertices().slot_len();
            let found = edit.find_or_add_vertex(&mut topo, point, TOL);
            match expected {
                Some(vertex) => assert_eq!(found, vertex, "lookup at {point:?}"),
                None => assert_eq!(found.index(), before, "a miss mints a vertex at {point:?}"),
            }
        }

        // The other body's coincident vertex is never chosen.
        let foreign_origin = face_vertices(&topo, foreign).unwrap();
        let mut edit = scope.edit(&mut replacements);
        let at_origin = edit.find_or_add_vertex(&mut topo, Point3::new(0.0, 0.0, 0.0), TOL);
        assert!(!foreign_origin.contains(&at_origin));
    }
}
