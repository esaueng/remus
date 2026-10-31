//! Split-band blend fixtures shared by the blend-editing tests (RFC 0007
//! stage 0b split of the old `test_helpers`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

// ---- Split-band blend fixtures (B19 survivor tranche) ----

use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

/// A 10 mm box with one r = 1 edge blend.
pub fn blended_box(edges: &[usize]) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let all = solid_edges(&topo, sharp).unwrap();
    let chosen: Vec<_> = edges.iter().map(|index| all[*index]).collect();
    let solid = crate::blend_ops::fillet_v2(&mut topo, sharp, &chosen, 1.0)
        .unwrap()
        .solid;
    (topo, solid)
}

pub fn cylinder_faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .collect()
}

fn split_uses(uses: &[OrientedEdge], edge: EdgeId, halves: [EdgeId; 2]) -> Vec<OrientedEdge> {
    let mut rebuilt = Vec::with_capacity(uses.len() + 1);
    for use_ in uses {
        if use_.edge() != edge {
            rebuilt.push(*use_);
        } else if use_.is_forward() {
            rebuilt.push(OrientedEdge::new(halves[0], true));
            rebuilt.push(OrientedEdge::new(halves[1], true));
        } else {
            rebuilt.push(OrientedEdge::new(halves[1], false));
            rebuilt.push(OrientedEdge::new(halves[0], false));
        }
    }
    rebuilt
}

/// Split the single cylindrical band of `solid` across its axis at mid
/// length into two faces on the same exact carrier: both springs gain a
/// midpoint vertex and the halves share one exact circular arc.
pub fn split_band(topo: &mut Topology, solid: SolidId) -> (SolidId, [FaceId; 2]) {
    let [band] = cylinder_faces(topo, solid)[..] else {
        panic!("one band");
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(band).unwrap().surface().clone() else {
        unreachable!("band");
    };
    let reversed = topo.face(band).unwrap().is_reversed();
    let uses = topo
        .wire(topo.face(band).unwrap().outer_wire())
        .unwrap()
        .edges()
        .to_vec();
    assert_eq!(uses.len(), 4);
    let springs: Vec<_> = uses
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| matches!(topo.edge(*edge).unwrap().curve(), EdgeCurve::Line))
        .collect();
    assert_eq!(springs.len(), 2);
    let mut middles = Vec::new();
    let mut halves = Vec::new();
    for &spring in &springs {
        let data = topo.edge(spring).unwrap();
        let (start, end) = (data.start(), data.end());
        let a = topo.vertex(start).unwrap().point();
        let b = topo.vertex(end).unwrap().point();
        let middle = topo.add_vertex(Vertex::new(a + (b - a) * 0.5, Tolerance::new().linear));
        let first = topo.add_edge(Edge::new(start, middle, EdgeCurve::Line));
        let second = topo.add_edge(Edge::new(middle, end, EdgeCurve::Line));
        middles.push(middle);
        halves.push([first, second]);
    }
    // Exact cross-section arc between the two spring midpoints.
    let axis = cylinder.axis().normalize().unwrap();
    let point = |vertex: VertexId| topo.vertex(vertex).unwrap().point();
    let m0 = point(middles[0]);
    let m1 = point(middles[1]);
    let center = cylinder.origin() + axis * (m0 - cylinder.origin()).dot(axis);
    let mut normal = axis;
    let mut circle =
        Circle3D::new_with_ref(center, normal, cylinder.radius(), m0 - center).unwrap();
    let mut sweep = circle.project(m1).rem_euclid(std::f64::consts::TAU);
    if sweep > std::f64::consts::PI {
        normal = normal * -1.0;
        circle = Circle3D::new_with_ref(center, normal, cylinder.radius(), m0 - center).unwrap();
        sweep = circle.project(m1).rem_euclid(std::f64::consts::TAU);
    }
    assert!((circle.evaluate(sweep) - m1).length() < 1e-12);
    let mut arc = Edge::new(middles[0], middles[1], EdgeCurve::Circle(circle));
    arc.set_trim(Some((0.0, sweep)));
    let arc = topo.add_edge(arc);

    // Supports: replace each spring by its halves.
    for face in solid_faces(topo, solid).unwrap() {
        if face == band {
            continue;
        }
        let outer = topo.face(face).unwrap().outer_wire();
        let mut sequence = topo.wire(outer).unwrap().edges().to_vec();
        let before = sequence.len();
        for (spring, pair) in springs.iter().zip(&halves) {
            sequence = split_uses(&sequence, *spring, *pair);
        }
        if sequence.len() != before {
            let wire = topo.add_wire(Wire::new(sequence, true).unwrap());
            let inner = topo.face(face).unwrap().inner_wires().to_vec();
            topo.set_face_boundary_wires(face, wire, inner).unwrap();
        }
    }
    // Band: six uses, rotated to start at the half that ends at middle 0.
    let mut sequence = uses;
    for (spring, pair) in springs.iter().zip(&halves) {
        sequence = split_uses(&sequence, *spring, *pair);
    }
    let ends_at = |use_: &OrientedEdge, vertex: VertexId| {
        use_.oriented_end(topo.edge(use_.edge()).unwrap()) == vertex
    };
    let start = sequence
        .iter()
        .position(|use_| ends_at(use_, middles[0]))
        .unwrap();
    sequence.rotate_left(start);
    // [.. -> m0, m0 -> .., cross, .. -> m1, m1 -> .., cross]
    assert!(ends_at(&sequence[3], middles[1]));
    let first_half = vec![
        sequence[1],
        sequence[2],
        sequence[3],
        OrientedEdge::new(arc, false),
    ];
    let second_half = vec![
        sequence[4],
        sequence[5],
        sequence[0],
        OrientedEdge::new(arc, true),
    ];
    let mut faces = Vec::new();
    for half in [first_half, second_half] {
        let wire = topo.add_wire(Wire::new(half, true).unwrap());
        let surface = FaceSurface::Cylinder(cylinder.clone());
        faces.push(topo.add_face(if reversed {
            Face::new_reversed(wire, Vec::new(), surface)
        } else {
            Face::new(wire, Vec::new(), surface)
        }));
    }
    let mut shell_faces: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| *face != band)
        .collect();
    shell_faces.extend(&faces);
    let shell = topo.add_shell(Shell::new(shell_faces).unwrap());
    let split = topo.add_solid(Solid::new(shell, Vec::new()));
    let report = crate::validate::validate_solid(topo, split).unwrap();
    assert!(report.is_valid(), "split band fixture: {:?}", report.issues);
    (split, [faces[0], faces[1]])
}
