//! Shared fixtures for the crate's unit tests.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::BTreeMap;

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::explorer::{solid_faces, solid_vertices};
use remus_topology::face::FaceSurface;
use remus_topology::shell::ShellId;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::test_utils::{make_unit_cube_manifold, make_unit_cube_manifold_at};
use remus_topology::vertex::{Vertex, VertexId};

/// A 3³ block with a 1³ cavity at `(1, 1, 1)`: two consistently oriented
/// manifold cube shells, the cavity's faces reversed so they face into it.
pub fn hollow_block(topo: &mut Topology) -> SolidId {
    let outer_shell = block3_shell(topo);
    let cavity = make_unit_cube_manifold_at(topo, 1.0, 1.0, 1.0);
    for face in solid_faces(topo, cavity).unwrap() {
        topo.face_mut(face).unwrap().set_reversed(true);
    }
    let cavity_shell = topo.solid(cavity).unwrap().outer_shell();
    topo.add_solid(Solid::new(outer_shell, vec![cavity_shell]))
}

/// The manifold shell of the cube `[0, 3]³`.
pub fn block3_shell(topo: &mut Topology) -> ShellId {
    let outer = make_unit_cube_manifold(topo);
    for vertex in solid_vertices(topo, outer).unwrap() {
        let p = topo.vertex(vertex).unwrap().point();
        topo.vertex_mut(vertex).unwrap().set_point(Point3::new(
            3.0 * p.x(),
            3.0 * p.y(),
            3.0 * p.z(),
        ));
    }
    for face in solid_faces(topo, outer).unwrap() {
        let FaceSurface::Plane { normal, d } = *topo.face(face).unwrap().surface() else {
            unreachable!("cube faces are planar");
        };
        topo.face_mut(face)
            .unwrap()
            .set_surface(FaceSurface::Plane { normal, d: 3.0 * d });
    }
    topo.solid(outer).unwrap().outer_shell()
}

/// The cavity shell of a [`hollow_block`].
pub fn cavity_shell(topo: &Topology, solid: SolidId) -> ShellId {
    topo.solid(solid).unwrap().inner_shells()[0]
}

/// Opens a `gap` at one corner of `shell`: the end of the first face's
/// first edge moves onto a fresh twin vertex `gap` away along +x, so the
/// two loops using that edge no longer chain there.
pub fn open_corner_gap(topo: &mut Topology, shell: ShellId, gap: f64) -> (VertexId, VertexId) {
    let face = topo.shell(shell).unwrap().faces()[0];
    let wire = topo.face(face).unwrap().outer_wire();
    let edge_id = topo.wire(wire).unwrap().edges()[0].edge();
    let corner = topo.edge(edge_id).unwrap().end();
    let p = topo.vertex(corner).unwrap().point();
    let twin = topo.add_vertex(Vertex::new(Point3::new(p.x() + gap, p.y(), p.z()), 1e-7));
    topo.edge_mut(edge_id).unwrap().set_end(twin);
    (corner, twin)
}

/// Junctions of `shell`'s loops (outer and holes) where one edge's oriented
/// end is not the next edge's oriented start.
pub fn broken_junctions(topo: &Topology, shell: ShellId) -> usize {
    let mut broken = 0;
    for &fid in topo.shell(shell).unwrap().faces() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let edges = topo.wire(wid).unwrap().edges();
            for (i, oe) in edges.iter().enumerate() {
                let next = edges[(i + 1) % edges.len()];
                let end = oe.oriented_end(topo.edge(oe.edge()).unwrap());
                let start = next.oriented_start(topo.edge(next.edge()).unwrap());
                broken += usize::from(end != start);
            }
        }
    }
    broken
}

/// Edges of `shell` whose two uses run the same way once each face's own
/// reversal is composed in.
pub fn misoriented_edges(topo: &Topology, shell: ShellId) -> usize {
    let mut senses: BTreeMap<usize, Vec<bool>> = BTreeMap::new();
    for &fid in topo.shell(shell).unwrap().faces() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                senses
                    .entry(oe.edge().index())
                    .or_default()
                    .push(oe.is_forward() != face.is_reversed());
            }
        }
    }
    senses
        .values()
        .filter(|uses| uses.len() == 2 && uses[0] == uses[1])
        .count()
}
