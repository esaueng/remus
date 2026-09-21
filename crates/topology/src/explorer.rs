//! Topology exploration and query utilities.
//!
//! Provides functions
//! for traversing the B-Rep topology graph and querying relationships
//! between entities.

use std::collections::{BTreeMap, HashSet};

use smallvec::SmallVec;

use crate::Topology;
use crate::TopologyError;
use crate::edge::EdgeId;
use crate::face::FaceId;
use crate::solid::SolidId;
use crate::vertex::VertexId;
use crate::wire::WireId;

// ── Solid queries ──────────────────────────────────────────────────

/// Get all unique face IDs from a solid (outer + inner shells).
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn solid_faces(topo: &Topology, solid: SolidId) -> Result<Vec<FaceId>, TopologyError> {
    let solid_data = topo.solid(solid)?;
    let mut faces = Vec::new();

    for shell_id in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        let shell = topo.shell(shell_id)?;
        faces.extend_from_slice(shell.faces());
    }

    Ok(faces)
}

/// Get all unique edge IDs from a solid.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn solid_edges(topo: &Topology, solid: SolidId) -> Result<Vec<EdgeId>, TopologyError> {
    let mut seen = HashSet::new();
    let mut edges = Vec::new();

    for face_id in solid_faces(topo, solid)? {
        for eid in face_edges(topo, face_id)? {
            if seen.insert(eid.index()) {
                edges.push(eid);
            }
        }
    }

    Ok(edges)
}

/// Get all unique vertex IDs from a solid.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn solid_vertices(topo: &Topology, solid: SolidId) -> Result<Vec<VertexId>, TopologyError> {
    let mut seen = HashSet::new();
    let mut vertices = Vec::new();

    for eid in solid_edges(topo, solid)? {
        let edge = topo.edge(eid)?;
        if seen.insert(edge.start().index()) {
            vertices.push(edge.start());
        }
        if seen.insert(edge.end().index()) {
            vertices.push(edge.end());
        }
    }

    Ok(vertices)
}

// ── Face queries ───────────────────────────────────────────────────

/// Get all unique edge IDs from a face (outer wire + inner wires).
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn face_edges(topo: &Topology, face: FaceId) -> Result<Vec<EdgeId>, TopologyError> {
    let face_data = topo.face(face)?;
    let mut seen = HashSet::new();
    let mut edges = Vec::new();

    for wire_id in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        let wire = topo.wire(wire_id)?;
        for oe in wire.edges() {
            if seen.insert(oe.edge().index()) {
                edges.push(oe.edge());
            }
        }
    }

    Ok(edges)
}

/// Get all unique vertex IDs from a face.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn face_vertices(topo: &Topology, face: FaceId) -> Result<Vec<VertexId>, TopologyError> {
    let mut seen = HashSet::new();
    let mut vertices = Vec::new();

    for eid in face_edges(topo, face)? {
        let edge = topo.edge(eid)?;
        if seen.insert(edge.start().index()) {
            vertices.push(edge.start());
        }
        if seen.insert(edge.end().index()) {
            vertices.push(edge.end());
        }
    }

    Ok(vertices)
}

// ── Edge queries ───────────────────────────────────────────────────

/// Build a map from edge index to the faces that reference it.
///
/// This is useful for finding shared edges (manifold edges appear in
/// exactly 2 faces) and boundary edges (appear in only 1 face).
///
/// Keyed by a `BTreeMap`, not a `HashMap`, because callers iterate the map
/// and the resulting order reaches their output: the offset engine reports
/// the first face pair it fails to intersect, `heal` unions faces in map
/// order, and the shell rim builder picks a chain start from it. Under a
/// seed-dependent hash order those results differed between processes on
/// identical input.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn edge_to_face_map(
    topo: &Topology,
    solid: SolidId,
) -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> {
    edge_to_face_map_for_faces(topo, &solid_faces(topo, solid)?)
}

/// Build a map from edge index to the supplied faces that reference it.
///
/// Unlike [`edge_to_face_map`], this accepts an arbitrary face set, including
/// an open sheet body. Repeated seam uses are retained for the same reason as
/// the solid-specific wrapper.
///
/// # Errors
///
/// Returns an error if any face, wire, or edge lookup fails.
pub fn edge_to_face_map_for_faces(
    topo: &Topology,
    faces: &[FaceId],
) -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> {
    let mut map: BTreeMap<usize, SmallVec<[FaceId; 2]>> = BTreeMap::new();

    for &face_id in faces {
        // Iterate wire edges directly (without deduplication) so that seam
        // edges — which appear twice in the same face's wire with opposite
        // orientations — are counted twice.  This matches the BRep convention
        // that each seam edge contributes two face-uses, keeping the
        // edge-sharing count correct for manifold checks.
        let face_data = topo.face(face_id)?;
        for wire_id in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id)?;
            for oe in wire.edges() {
                map.entry(oe.edge().index()).or_default().push(face_id);
            }
        }
    }

    Ok(map)
}

/// Find all edges shared between two faces of a solid.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn shared_edges(
    topo: &Topology,
    face_a: FaceId,
    face_b: FaceId,
) -> Result<Vec<EdgeId>, TopologyError> {
    let edges_a: HashSet<usize> = face_edges(topo, face_a)?
        .iter()
        .map(|e| e.index())
        .collect();
    let edges_b = face_edges(topo, face_b)?;

    Ok(edges_b
        .into_iter()
        .filter(|e| edges_a.contains(&e.index()))
        .collect())
}

/// Find all faces adjacent to a given face (sharing at least one edge).
///
/// Requires a precomputed edge-to-face map for efficiency.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn adjacent_faces<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    face: FaceId,
    edge_face_map: &BTreeMap<usize, V>,
) -> Result<Vec<FaceId>, TopologyError> {
    let mut seen = HashSet::new();
    let mut neighbors = Vec::new();

    for eid in face_edges(topo, face)? {
        if let Some(faces) = edge_face_map.get(&eid.index()) {
            for &fid in &**faces {
                if fid.index() != face.index() && seen.insert(fid.index()) {
                    neighbors.push(fid);
                }
            }
        }
    }

    Ok(neighbors)
}

// ── Wire queries ───────────────────────────────────────────────────

/// Get all wires from a face.
///
/// Returns the outer wire followed by any inner wires.
///
/// # Errors
///
/// Returns an error if the face lookup fails.
pub fn face_wires(topo: &Topology, face: FaceId) -> Result<Vec<WireId>, TopologyError> {
    let face_data = topo.face(face)?;
    let mut wires = vec![face_data.outer_wire()];
    wires.extend_from_slice(face_data.inner_wires());
    Ok(wires)
}

// ── Counting ───────────────────────────────────────────────────────

/// Count entities in a solid.
///
/// Returns `(faces, edges, vertices)` — the Euler characteristic
/// components for topology validation.
///
/// # Errors
///
/// Returns an error if any topology lookup fails.
pub fn solid_entity_counts(
    topo: &Topology,
    solid: SolidId,
) -> Result<(usize, usize, usize), TopologyError> {
    let faces = solid_faces(topo, solid)?.len();
    let edges = solid_edges(topo, solid)?.len();
    let vertices = solid_vertices(topo, solid)?.len();
    Ok((faces, edges, vertices))
}

#[cfg(all(test, feature = "test-utils"))]
mod tests {
    #![allow(clippy::unwrap_used)]

    use crate::Topology;
    use crate::test_utils::make_unit_cube_manifold;

    use super::*;

    #[test]
    fn cube_entity_counts() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let (f, e, v) = solid_entity_counts(&topo, cube).unwrap();
        assert_eq!(f, 6, "cube should have 6 faces");
        assert_eq!(e, 12, "cube should have 12 edges");
        assert_eq!(v, 8, "cube should have 8 vertices");
    }

    #[test]
    fn cube_euler_characteristic() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let (f, e, v) = solid_entity_counts(&topo, cube).unwrap();
        // Euler characteristic for a convex polyhedron: V - E + F = 2
        #[allow(clippy::cast_possible_wrap)]
        let euler = (v as i64) - (e as i64) + (f as i64);
        assert_eq!(euler, 2, "V-E+F should be 2 for a cube");
    }

    #[test]
    fn cube_edge_to_face_map() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let map = edge_to_face_map(&topo, cube).unwrap();

        // Every edge of a manifold cube is shared by exactly 2 faces.
        for (edge_idx, faces) in &map {
            assert_eq!(
                faces.len(),
                2,
                "edge {edge_idx} should be shared by 2 faces, got {}",
                faces.len()
            );
        }
    }

    #[test]
    fn edge_to_face_map_iterates_in_edge_index_order() {
        // Callers iterate this map and let the order reach their output — the
        // offset engine reported whichever face pair it happened to hit first,
        // so the same NURBS solid produced a different error message in each
        // process. A `HashMap` return type fails this assertion in almost
        // every run; the `BTreeMap` makes it structural.
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let keys: Vec<usize> = edge_to_face_map(&topo, cube).unwrap().into_keys().collect();
        assert!(keys.len() >= 12, "cube should have at least 12 edges");

        let mut sorted = keys.clone();
        sorted.sort_unstable();
        assert_eq!(keys, sorted, "edge_to_face_map must yield ascending keys");
    }

    #[test]
    fn cube_face_has_4_edges() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let faces = solid_faces(&topo, cube).unwrap();
        for fid in faces {
            let edges = face_edges(&topo, fid).unwrap();
            assert_eq!(edges.len(), 4, "each cube face should have 4 edges");
        }
    }

    #[test]
    fn cube_face_has_4_vertices() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let faces = solid_faces(&topo, cube).unwrap();
        for fid in faces {
            let verts = face_vertices(&topo, fid).unwrap();
            assert_eq!(verts.len(), 4, "each cube face should have 4 vertices");
        }
    }

    #[test]
    fn shared_edges_adjacent_cube_faces() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let faces = solid_faces(&topo, cube).unwrap();
        let map = edge_to_face_map(&topo, cube).unwrap();

        let neighbors = adjacent_faces(&topo, faces[0], &map).unwrap();

        assert_eq!(neighbors.len(), 4, "cube face should have 4 adjacent faces");

        for &neighbor in &neighbors {
            let shared = shared_edges(&topo, faces[0], neighbor).unwrap();
            assert_eq!(
                shared.len(),
                1,
                "adjacent cube faces should share exactly 1 edge"
            );
        }
    }

    #[test]
    fn face_wires_returns_outer() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let faces = solid_faces(&topo, cube).unwrap();
        let wires = face_wires(&topo, faces[0]).unwrap();

        assert_eq!(wires.len(), 1, "cube face should have 1 wire (outer only)");
    }
}

#[cfg(test)]
mod traversal_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::vec::{Point3, Vec3};

    use crate::edge::{Edge, EdgeCurve};
    use crate::face::{Face, FaceSurface};
    use crate::shell::{Shell, ShellId};
    use crate::solid::Solid;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    use super::*;

    const TOL: f64 = 1e-7;

    /// Builds a closed, manifold axis-aligned cube shell whose corner sits at
    /// `(origin, origin, origin)` with the given edge `size`.
    ///
    /// Every one of the 12 edges is used exactly twice — once forward, once
    /// reversed — so the shell is a genuine closed manifold, not six
    /// independent quads.
    fn cube_shell(topo: &mut Topology, origin: f64, size: f64) -> ShellId {
        let p = |x: f64, y: f64, z: f64| {
            Point3::new(origin + x * size, origin + y * size, origin + z * size)
        };
        let v: [VertexId; 8] = [
            topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL)),
            topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL)),
            topo.add_vertex(Vertex::new(p(1.0, 1.0, 0.0), TOL)),
            topo.add_vertex(Vertex::new(p(0.0, 1.0, 0.0), TOL)),
            topo.add_vertex(Vertex::new(p(0.0, 0.0, 1.0), TOL)),
            topo.add_vertex(Vertex::new(p(1.0, 0.0, 1.0), TOL)),
            topo.add_vertex(Vertex::new(p(1.0, 1.0, 1.0), TOL)),
            topo.add_vertex(Vertex::new(p(0.0, 1.0, 1.0), TOL)),
        ];

        // Bottom ring, top ring, then the four verticals.
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

        let mk =
            |topo: &mut Topology, edges: [(EdgeId, bool); 4], normal: Vec3, d: f64| -> FaceId {
                let wire = Wire::new(
                    edges
                        .iter()
                        .map(|&(eid, fwd)| OrientedEdge::new(eid, fwd))
                        .collect(),
                    true,
                )
                .expect("cube wire");
                let wid = topo.add_wire(wire);
                topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
            };

        let lo = origin;
        let hi = origin + size;
        let bottom = mk(
            topo,
            [
                (eb[0], false),
                (eb[3], false),
                (eb[2], false),
                (eb[1], false),
            ],
            Vec3::new(0.0, 0.0, -1.0),
            -lo,
        );
        let top = mk(
            topo,
            [(et[0], true), (et[1], true), (et[2], true), (et[3], true)],
            Vec3::new(0.0, 0.0, 1.0),
            hi,
        );
        let front = mk(
            topo,
            [(eb[0], true), (ev[1], true), (et[0], false), (ev[0], false)],
            Vec3::new(0.0, -1.0, 0.0),
            -lo,
        );
        let back = mk(
            topo,
            [(eb[2], true), (ev[3], true), (et[2], false), (ev[2], false)],
            Vec3::new(0.0, 1.0, 0.0),
            hi,
        );
        let left = mk(
            topo,
            [(eb[3], true), (ev[0], true), (et[3], false), (ev[3], false)],
            Vec3::new(-1.0, 0.0, 0.0),
            -lo,
        );
        let right = mk(
            topo,
            [(eb[1], true), (ev[2], true), (et[1], false), (ev[1], false)],
            Vec3::new(1.0, 0.0, 0.0),
            hi,
        );

        topo.add_shell(Shell::new(vec![bottom, top, front, back, left, right]).expect("cube shell"))
    }

    /// A solid whose outer shell is a unit cube and which has no cavity.
    fn solid_cube(topo: &mut Topology) -> SolidId {
        let outer = cube_shell(topo, 0.0, 1.0);
        topo.add_solid(Solid::new(outer, vec![]))
    }

    /// A solid with one inner (cavity) shell: a unit cube containing a
    /// half-size cube-shaped void.
    fn solid_cube_with_cavity(topo: &mut Topology) -> (SolidId, ShellId, ShellId) {
        let outer = cube_shell(topo, 0.0, 1.0);
        let inner = cube_shell(topo, 0.25, 0.5);
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        (solid, outer, inner)
    }

    /// A single planar face carrying one square hole: 4 outer + 4 inner edges.
    fn holed_face(topo: &mut Topology) -> FaceId {
        let mut ring = |x0: f64, y0: f64, x1: f64, y1: f64| -> WireId {
            let a = topo.add_vertex(Vertex::new(Point3::new(x0, y0, 0.0), TOL));
            let b = topo.add_vertex(Vertex::new(Point3::new(x1, y0, 0.0), TOL));
            let c = topo.add_vertex(Vertex::new(Point3::new(x1, y1, 0.0), TOL));
            let d = topo.add_vertex(Vertex::new(Point3::new(x0, y1, 0.0), TOL));
            let e0 = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
            let e1 = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
            let e2 = topo.add_edge(Edge::new(c, d, EdgeCurve::Line));
            let e3 = topo.add_edge(Edge::new(d, a, EdgeCurve::Line));
            topo.add_wire(
                Wire::new(
                    vec![
                        OrientedEdge::new(e0, true),
                        OrientedEdge::new(e1, true),
                        OrientedEdge::new(e2, true),
                        OrientedEdge::new(e3, true),
                    ],
                    true,
                )
                .expect("ring wire"),
            )
        };

        let outer = ring(0.0, 0.0, 3.0, 3.0);
        let inner = ring(1.0, 1.0, 2.0, 2.0);
        topo.add_face(Face::new(
            outer,
            vec![inner],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn solid_faces_covers_outer_and_inner_shells() {
        // The project's own guidance: a function that visits only
        // `outer_shell()` silently drops cavity faces. This is the fixture
        // that notices.
        let mut topo = Topology::new();
        let (solid, outer, inner) = solid_cube_with_cavity(&mut topo);

        let faces = solid_faces(&topo, solid).unwrap();
        assert_eq!(faces.len(), 12, "6 outer + 6 cavity faces");

        let mut unique: Vec<usize> = faces.iter().map(|f| f.index()).collect();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), 12, "solid_faces must not repeat a face");

        for shell_id in [outer, inner] {
            for fid in topo.shell(shell_id).unwrap().faces() {
                assert!(
                    faces.iter().any(|f| f.index() == fid.index()),
                    "face {} of shell {} missing from solid_faces",
                    fid.index(),
                    shell_id.index()
                );
            }
        }

        // A solid with no cavity still reports exactly its outer faces.
        let mut plain_topo = Topology::new();
        let plain = solid_cube(&mut plain_topo);
        assert_eq!(solid_faces(&plain_topo, plain).unwrap().len(), 6);
    }

    #[test]
    fn solid_entity_counts_sum_over_every_shell() {
        let mut topo = Topology::new();
        let plain = solid_cube(&mut topo);
        assert_eq!(
            solid_entity_counts(&topo, plain).unwrap(),
            (6, 12, 8),
            "a cube is 6 faces / 12 edges / 8 vertices"
        );
        assert_eq!(solid_edges(&topo, plain).unwrap().len(), 12);
        assert_eq!(solid_vertices(&topo, plain).unwrap().len(), 8);

        let mut cavity_topo = Topology::new();
        let (cavity, _, _) = solid_cube_with_cavity(&mut cavity_topo);
        assert_eq!(
            solid_entity_counts(&cavity_topo, cavity).unwrap(),
            (12, 24, 16),
            "cavity shell doubles every count"
        );
        assert_eq!(solid_edges(&cavity_topo, cavity).unwrap().len(), 24);
        assert_eq!(solid_vertices(&cavity_topo, cavity).unwrap().len(), 16);
    }

    #[test]
    fn face_edges_and_vertices_include_inner_wires() {
        let mut topo = Topology::new();
        let holed = holed_face(&mut topo);

        assert_eq!(
            face_edges(&topo, holed).unwrap().len(),
            8,
            "4 outer + 4 hole edges"
        );
        assert_eq!(
            face_vertices(&topo, holed).unwrap().len(),
            8,
            "4 outer + 4 hole corners"
        );

        let mut cube_topo = Topology::new();
        let cube = solid_cube(&mut cube_topo);
        for fid in solid_faces(&cube_topo, cube).unwrap() {
            assert_eq!(face_edges(&cube_topo, fid).unwrap().len(), 4);
            assert_eq!(face_vertices(&cube_topo, fid).unwrap().len(), 4);
        }
    }

    #[test]
    fn face_wires_returns_outer_then_inner() {
        let mut topo = Topology::new();
        let holed = holed_face(&mut topo);

        let wires = face_wires(&topo, holed).unwrap();
        let face_data = topo.face(holed).unwrap();
        assert_eq!(wires.len(), 2, "outer wire plus one hole wire");
        assert_eq!(wires[0], face_data.outer_wire(), "outer wire comes first");
        assert_eq!(wires[1], face_data.inner_wires()[0]);

        let mut cube_topo = Topology::new();
        let cube = solid_cube(&mut cube_topo);
        let fid = solid_faces(&cube_topo, cube).unwrap()[0];
        assert_eq!(face_wires(&cube_topo, fid).unwrap().len(), 1);
    }

    #[test]
    fn edge_to_face_map_indexes_every_shell_edge_twice() {
        let mut topo = Topology::new();
        let (cavity, _, _) = solid_cube_with_cavity(&mut topo);

        let map = edge_to_face_map(&topo, cavity).unwrap();
        assert_eq!(map.len(), 24, "12 outer + 12 cavity edges are all indexed");
        for (edge_idx, faces) in &map {
            assert_eq!(
                faces.len(),
                2,
                "edge {edge_idx} should be used by exactly 2 faces"
            );
        }

        let keys: Vec<usize> = map.keys().copied().collect();
        let mut sorted = keys.clone();
        sorted.sort_unstable();
        assert_eq!(keys, sorted, "keys must come out in ascending edge order");
    }

    #[test]
    fn edge_to_face_map_for_faces_indexes_only_the_given_faces() {
        let mut topo = Topology::new();
        let (cavity, outer, _) = solid_cube_with_cavity(&mut topo);

        let outer_faces = topo.shell(outer).unwrap().faces().to_vec();
        let map = edge_to_face_map_for_faces(&topo, &outer_faces).unwrap();
        assert_eq!(map.len(), 12, "only the outer shell's 12 edges");
        for (edge_idx, faces) in &map {
            assert_eq!(faces.len(), 2, "edge {edge_idx} is shared by 2 faces");
            for fid in faces {
                assert!(
                    outer_faces.iter().any(|f| f.index() == fid.index()),
                    "map must only mention the supplied faces"
                );
            }
        }

        // A single face on its own gives its 4 edges, each with one use.
        let single = edge_to_face_map_for_faces(&topo, &outer_faces[..1]).unwrap();
        assert_eq!(single.len(), 4);
        for faces in single.values() {
            assert_eq!(faces.len(), 1);
        }

        // And the whole solid's faces reproduce `edge_to_face_map`.
        let all = solid_faces(&topo, cavity).unwrap();
        assert_eq!(
            edge_to_face_map_for_faces(&topo, &all).unwrap().len(),
            edge_to_face_map(&topo, cavity).unwrap().len()
        );
    }

    #[test]
    fn adjacent_faces_and_shared_edges_on_a_cube() {
        let mut topo = Topology::new();
        let cube = solid_cube(&mut topo);
        let faces = solid_faces(&topo, cube).unwrap();
        let map = edge_to_face_map(&topo, cube).unwrap();

        // `cube_shell` emits bottom, top, front, back, left, right — so
        // faces[0] and faces[1] are the opposing pair.
        let bottom = faces[0];
        let top = faces[1];

        let neighbors = adjacent_faces(&topo, bottom, &map).unwrap();
        assert_eq!(neighbors.len(), 4, "a cube face touches exactly 4 others");
        assert!(
            neighbors.iter().all(|f| f.index() != bottom.index()),
            "a face is never its own neighbour"
        );
        let mut unique: Vec<usize> = neighbors.iter().map(|f| f.index()).collect();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), 4, "neighbours must be deduplicated");
        assert!(
            neighbors.iter().all(|f| f.index() != top.index()),
            "the opposing face shares no edge"
        );

        for &neighbor in &neighbors {
            assert_eq!(
                shared_edges(&topo, bottom, neighbor).unwrap().len(),
                1,
                "adjacent cube faces share exactly 1 edge"
            );
        }
        assert!(
            shared_edges(&topo, bottom, top).unwrap().is_empty(),
            "opposing cube faces share no edge"
        );
    }
}
