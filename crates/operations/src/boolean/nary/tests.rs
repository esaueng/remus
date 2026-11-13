//! Unit tests for the N-ary fuse helpers that the witness suite reaches only
//! through a whole fuse: grouping, and the vertex-link reading.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::Point3;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::*;

#[test]
fn touching_groups_are_the_connected_components_in_input_order() {
    // No pairs: every operand alone.
    assert_eq!(touching_groups(3, &[]), vec![vec![0], vec![1], vec![2]]);
    // Two separate pairs and a loner between them.
    assert_eq!(
        touching_groups(5, &[(0, 2), (3, 4)]),
        vec![vec![0, 2], vec![1], vec![3, 4]]
    );
    // A chain joined through its highest member, listed out of order: the
    // group still starts at its lowest operand and is ascending.
    assert_eq!(
        touching_groups(5, &[(2, 4), (0, 4), (1, 3)]),
        vec![vec![0, 2, 4], vec![1, 3]]
    );
    // Two groups merged by a late pair.
    assert_eq!(
        touching_groups(4, &[(0, 1), (2, 3), (1, 2)]),
        vec![vec![0, 1, 2, 3]]
    );
}

/// The four faces of a tetrahedron on `apex` and three new base vertices,
/// every edge used once in each direction.
fn tetrahedron(topo: &mut Topology, apex: VertexId, base: [Point3; 3]) -> Vec<FaceId> {
    let b: Vec<VertexId> = base
        .iter()
        .map(|&point| topo.add_vertex(Vertex::new(point, 1e-7)))
        .collect();
    let mut edge =
        |from: VertexId, to: VertexId| topo.add_edge(Edge::new(from, to, EdgeCurve::Line));
    let (b01, b12, b20) = (edge(b[0], b[1]), edge(b[1], b[2]), edge(b[2], b[0]));
    let (a0, a1, a2) = (edge(apex, b[0]), edge(apex, b[1]), edge(apex, b[2]));
    let loops = [
        [(b20, false), (b12, false), (b01, false)],
        [(a0, true), (b01, true), (a1, false)],
        [(a1, true), (b12, true), (a2, false)],
        [(a2, true), (b20, true), (a0, false)],
    ];
    loops
        .iter()
        .map(|uses| {
            let wire = Wire::new(
                uses.iter()
                    .map(|&(edge, forward)| OrientedEdge::new(edge, forward))
                    .collect(),
                true,
            )
            .unwrap();
            let wire = topo.add_wire(wire);
            // The vertex-link reading never looks at the carrier.
            let surface = FaceSurface::Plane {
                normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            };
            topo.add_face(Face::new(wire, vec![], surface))
        })
        .collect()
}

fn solid_of(topo: &mut Topology, faces: Vec<FaceId>) -> SolidId {
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// Two tetrahedra joined at their apex and nowhere else. Every edge is used
/// exactly twice, so the edge count calls the shell a closed manifold; the
/// apex's six edges fall into two fans of three, and that is the pinch.
#[test]
fn a_vertex_shared_by_two_fans_is_a_pinch_the_edge_count_cannot_see() {
    let mut topo = Topology::new();
    let apex = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    let upper = tetrahedron(
        &mut topo,
        apex,
        [
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(-1.0, -1.0, 1.0),
        ],
    );
    let lone = solid_of(&mut topo, upper.clone());
    assert_eq!(pinched_vertex(&topo, lone).unwrap(), None);

    let lower = tetrahedron(
        &mut topo,
        apex,
        [
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(0.0, 1.0, -1.0),
        ],
    );
    let bow_tie = solid_of(&mut topo, upper.into_iter().chain(lower).collect());
    assert!(super::super::is_closed_manifold(&topo, bow_tie).unwrap());
    assert_eq!(pinched_vertex(&topo, bow_tie).unwrap(), Some(apex.index()));

    let error = qualify_region_structure(&topo, bow_tie, 3).unwrap_err();
    assert!(
        matches!(
            &error,
            OperationsError::Algo(AlgoError::UnsupportedNaryFuse {
                cause: NaryFuseRefusal::NonManifoldContact,
                reason,
            }) if reason.contains("region 3") && reason.contains(&apex.index().to_string())
        ),
        "{error:?}"
    );
}

#[test]
fn a_box_has_no_pinched_vertex() {
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    assert_eq!(pinched_vertex(&topo, solid).unwrap(), None);
    qualify_region_structure(&topo, solid, 0).unwrap();
}

/// Two boxes whose corners coincide are two distinct vertices at one
/// position: no shared id, so only the position reading reports it.
#[test]
fn distinct_vertices_at_one_position_are_refused() {
    let mut topo = Topology::new();
    let first = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let second = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

    crate::transform::transform_solid(
        &mut topo,
        second,
        &remus_math::mat::Mat4::translation(1.0, 1.0, 1.0),
    )
    .unwrap();
    let regions = [first, second];
    let error = refuse_coincident_vertices(&topo, &regions, 1e-7).unwrap_err();
    assert!(matches!(
        error,
        OperationsError::Algo(AlgoError::UnsupportedNaryFuse {
            cause: NaryFuseRefusal::NonManifoldContact,
            ..
        })
    ));
    // The same corner is a touch between the two solids.
    let error = refuse_touching_regions(&topo, &regions, 1e-7).unwrap_err();
    assert!(matches!(
        error,
        OperationsError::Algo(AlgoError::UnsupportedNaryFuse {
            cause: NaryFuseRefusal::NonManifoldContact,
            ..
        })
    ));

    // A hair further apart than the tolerance they are two clean regions.
    crate::transform::transform_solid(
        &mut topo,
        second,
        &remus_math::mat::Mat4::translation(1e-3, 0.0, 0.0),
    )
    .unwrap();
    let regions = [first, second];
    refuse_coincident_vertices(&topo, &regions, 1e-7).unwrap();
    refuse_touching_regions(&topo, &regions, 1e-7).unwrap();
}

/// A corner of a plain box is in the closed box although the point
/// classifier calls three of the eight outside; a point clear of it is not.
#[test]
fn a_closed_solid_holds_its_own_corners() {
    let mut topo = Topology::new();
    let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let corners: Vec<Point3> = remus_topology::explorer::solid_vertices(&topo, solid)
        .unwrap()
        .into_iter()
        .map(|vertex| topo.vertex(vertex).unwrap().point())
        .collect();
    assert_eq!(corners.len(), 8);
    assert!(
        closed_solid_holds(&topo, solid, &corners, 1e-7)
            .unwrap()
            .into_iter()
            .all(|held| held)
    );
    let probes = [
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(1.0, 1.0, 2.0),
        Point3::new(1.0, 1.0, 2.001),
        Point3::new(3.0, 3.0, 3.0),
    ];
    assert_eq!(
        closed_solid_holds(&topo, solid, &probes, 1e-7).unwrap(),
        vec![true, true, false, false]
    );
}
