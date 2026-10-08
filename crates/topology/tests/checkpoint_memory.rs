//! Retired handle storage and shared NURBS nets are accounted separately.
#![allow(clippy::unwrap_used)]
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::{
    Topology,
    edge::Edge,
    face::{Face, FaceSurface},
    vertex::Vertex,
    wire::{OrientedEdge, Wire},
};

#[test]
fn shared_payloads_are_counted_once_across_cloned_arenas() {
    let mut topo = Topology::new();
    let a = topo.add_vertex(Vertex::new(Point3::new(0., 0., 0.), 1e-7));
    let b = topo.add_vertex(Vertex::new(Point3::new(0., 1., 0.), 1e-7));
    let edge = topo.add_edge(Edge::new(a, b, remus_topology::edge::EdgeCurve::Line));
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());
    let surface = NurbsSurface::new(
        1,
        1,
        vec![0., 0., 1., 1.],
        vec![0., 0., 1., 1.],
        vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 0.)],
            vec![Point3::new(1., 0., 0.), Point3::new(1., 1., 1.)],
        ],
        vec![vec![1.; 2]; 2],
    )
    .unwrap();
    topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface)));
    let saved = topo.clone();
    let mut shared = std::collections::HashSet::new();
    let original = topo.memory_estimate(&mut shared);
    let snapshot = saved.memory_estimate(&mut shared);
    assert!(original.nurbs_bytes > 0);
    assert_eq!(snapshot.nurbs_bytes, 0);
    assert!(snapshot.arena_bytes > 0);
    let tail = topo.add_vertex(Vertex::new(Point3::new(2., 2., 2.), 1e-7));
    topo.restore_preserving_handle_slots(&saved);
    let restored = topo.memory_estimate(&mut std::collections::HashSet::new());
    assert!(restored.allocated_slots > original.allocated_slots);
    assert_eq!(restored.retired_slots, 1);
    assert!(topo.vertex(tail).is_err());
    assert!(
        topo.add_vertex(Vertex::new(Point3::new(3., 3., 3.), 1e-7))
            .index()
            > tail.index()
    );
}
