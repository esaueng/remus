//! Full-turn torus rims may be stored as endpoint-connected circle-arc chains.
//!
//! A partial revolution of a circle is a closed torus sector: one torus band,
//! two planar disc caps, two full-turn circle rims, and one doubled seam arc.
//! Splitting either shared rim must not change the exact solid or crack its
//! tessellation. A chain that returns without winding the torus parameter is a
//! different topology and must not be skinned as though it were a full rim.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_operations::revolve::revolve;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid_with_tolerance,
};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

const MAJOR_RADIUS: f64 = 6.0;
const MINOR_RADIUS: f64 = 2.0;
const SWEEP_ANGLE: f64 = 2.0 * PI / 3.0;

fn make_partial_torus(topo: &mut Topology) -> SolidId {
    let circle = Circle3D::new(
        Point3::new(MAJOR_RADIUS, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        MINOR_RADIUS,
    )
    .unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let edge = topo.add_edge(Edge::new(vertex, vertex, EdgeCurve::Circle(circle)));
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        },
    ));
    revolve(
        topo,
        profile,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        SWEEP_ANGLE,
    )
    .unwrap()
}

fn torus_rims(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let torus_face = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face_id| matches!(topo.face(face_id).unwrap().surface(), FaceSurface::Torus(_)))
        .expect("one torus band");
    let face = topo.face(torus_face).unwrap();
    let mut rims: Vec<EdgeId> = topo
        .wire(face.outer_wire())
        .unwrap()
        .edges()
        .iter()
        .filter_map(|oriented| {
            let edge = topo.edge(oriented.edge()).unwrap();
            (edge.start() == edge.end() && matches!(edge.curve(), EdgeCurve::Circle(_)))
                .then_some(oriented.edge())
        })
        .collect();
    rims.sort_by_key(|edge| edge.index());
    rims.dedup();
    assert_eq!(rims.len(), 2, "partial torus has two rims");
    rims
}

fn rewrite_wire(
    topo: &mut Topology,
    wire_id: WireId,
    target: EdgeId,
    forward_chain: &[OrientedEdge],
) -> WireId {
    let old = topo.wire(wire_id).unwrap();
    if !old.edges().iter().any(|edge| edge.edge() == target) {
        return wire_id;
    }

    let mut edges = Vec::with_capacity(old.edges().len() + forward_chain.len() - 1);
    for oriented in old.edges() {
        if oriented.edge() != target {
            edges.push(*oriented);
        } else if oriented.is_forward() {
            edges.extend_from_slice(forward_chain);
        } else {
            edges.extend(
                forward_chain
                    .iter()
                    .rev()
                    .map(|edge| OrientedEdge::new(edge.edge(), !edge.is_forward())),
            );
        }
    }
    topo.add_wire(Wire::new(edges, old.is_closed()).unwrap())
}

fn rewrite_solid_edge(
    topo: &mut Topology,
    solid: SolidId,
    target: EdgeId,
    forward_chain: &[OrientedEdge],
) {
    let faces = solid_faces(topo, solid).unwrap();
    let mut rewrites: HashMap<WireId, WireId> = HashMap::new();
    for &face_id in &faces {
        let face = topo.face(face_id).unwrap();
        let wire_ids: Vec<WireId> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for wire_id in wire_ids {
            let replacement = rewrite_wire(topo, wire_id, target, forward_chain);
            if replacement != wire_id {
                rewrites.insert(wire_id, replacement);
            }
        }
    }
    for face_id in faces {
        let face = topo.face_mut(face_id).unwrap();
        if let Some(&wire) = rewrites.get(&face.outer_wire()) {
            face.set_outer_wire(wire);
        }
        for wire in face.inner_wires_mut() {
            if let Some(&replacement) = rewrites.get(wire) {
                *wire = replacement;
            }
        }
    }
}

fn split_rim_edge(
    topo: &mut Topology,
    solid: SolidId,
    target: EdgeId,
    parts: usize,
) -> Vec<EdgeId> {
    assert!(parts >= 2);
    let target_edge = topo.edge(target).unwrap();
    let EdgeCurve::Circle(source_circle) = target_edge.curve() else {
        panic!("rim is not circular");
    };
    let start = target_edge.start();
    let start_point = topo.vertex(start).unwrap().point();
    let circle = source_circle.clone();
    let t0 = circle.project(start_point);

    let mut vertices: Vec<VertexId> = Vec::with_capacity(parts + 1);
    vertices.push(start);
    for part in 1..parts {
        let parameter = t0 + TAU * part as f64 / parts as f64;
        vertices.push(topo.add_vertex(Vertex::new(circle.evaluate(parameter), 1e-7)));
    }
    vertices.push(start);

    let parameter_chain: Vec<OrientedEdge> = (0..parts)
        .map(|part| {
            let t_start = t0 + TAU * part as f64 / parts as f64;
            let t_end = t0 + TAU * (part + 1) as f64 / parts as f64;
            let mut edge = Edge::new(
                vertices[part],
                vertices[part + 1],
                EdgeCurve::Circle(circle.clone()),
            );
            edge.set_trim(Some((t_start, t_end)));
            let edge = topo.add_edge(edge);
            OrientedEdge::new(edge, true)
        })
        .collect();
    rewrite_solid_edge(topo, solid, target, &parameter_chain);
    parameter_chain.iter().map(OrientedEdge::edge).collect()
}

fn assert_closed_exact_sector(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: {:?}", report.issues);

    let volume = solid_volume(topo, solid, 0.02).unwrap();
    let expected = PI * MAJOR_RADIUS * MINOR_RADIUS * MINOR_RADIUS * SWEEP_ANGLE;
    assert!(
        (volume - expected).abs() <= expected * 1e-4,
        "{what}: volume {volume} != closed form {expected}"
    );

    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.03, 8.0_f64.to_radians()).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold mesh"
    );
}

#[test]
fn one_rim_split_in_two_closes_with_both_vertex_orders() {
    let mut topo = Topology::new();
    let solid = make_partial_torus(&mut topo);
    let target = torus_rims(&topo, solid)[0];
    let arcs = split_rim_edge(&mut topo, solid, target, 2);
    let (first, second) = (topo.edge(arcs[0]).unwrap(), topo.edge(arcs[1]).unwrap());
    assert_eq!(first.start(), second.end());
    assert_eq!(first.end(), second.start());
    assert_closed_exact_sector(&topo, solid, "two half-arcs");
}

#[test]
fn one_rim_split_in_three_closes() {
    let mut topo = Topology::new();
    let solid = make_partial_torus(&mut topo);
    let target = torus_rims(&topo, solid)[1];
    split_rim_edge(&mut topo, solid, target, 3);
    assert_closed_exact_sector(&topo, solid, "three-arc rim");
}

#[test]
fn both_rims_may_be_split_independently() {
    let mut topo = Topology::new();
    let solid = make_partial_torus(&mut topo);
    let rims = torus_rims(&topo, solid);
    split_rim_edge(&mut topo, solid, rims[1], 3);
    split_rim_edge(&mut topo, solid, rims[0], 2);
    assert_closed_exact_sector(&topo, solid, "both rims split");
}

#[test]
fn non_winding_circle_pair_is_not_skinned_as_a_full_rim() {
    let mut topo = Topology::new();
    let solid = make_partial_torus(&mut topo);
    let target = torus_rims(&topo, solid)[0];
    let edge = topo.edge(target).unwrap();
    let EdgeCurve::Circle(circle) = edge.curve() else {
        panic!("rim is not circular");
    };
    let circle = circle.clone();
    let v0 = edge.start();
    let t0 = circle.project(topo.vertex(v0).unwrap().point());
    let v1 = topo.add_vertex(Vertex::new(circle.evaluate(t0 + PI / 2.0), 1e-7));
    // Both edges have the same stored vertex order and therefore describe the
    // same quarter arc. The wire returns by traversing the second edge in
    // reverse, so its projected net winding is zero rather than one turn.
    let first = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Circle(circle.clone())));
    let second = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Circle(circle)));
    rewrite_solid_edge(
        &mut topo,
        solid,
        target,
        &[
            OrientedEdge::new(first, true),
            OrientedEdge::new(second, false),
        ],
    );

    if let Ok(mesh) = tessellate_solid_with_tolerance(&topo, solid, 0.03, 8.0_f64.to_radians()) {
        assert!(
            boundary_edge_count(&mesh) > 0,
            "a non-winding arc pair must not be skinned into a closed torus band"
        );
    }
}
