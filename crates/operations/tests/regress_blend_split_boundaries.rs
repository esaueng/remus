//! Exact cylindrical blend removal with topologically split spring contacts.
//!
//! A split vertex on a straight tangent contact is importer/history topology,
//! not a second geometric boundary. These regressions prove that the public
//! removal path recognizes only connected collinear carrier-certified chains,
//! collapses them to one sharp edge, records total construction history, and
//! rolls back malformed chains.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::HashSet;

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::journal_ops::{resize_blend_journaled, solid_entity_keys};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntryPayload;
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

fn fixture() -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, sharp).unwrap()[0];
    let solid = fillet_v2(&mut topo, sharp, &[edge], 1.0).unwrap().solid;
    let band = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .expect("one cylindrical edge band");
    (topo, solid, band)
}

fn spring_edges(topo: &Topology, solid: SolidId, band: FaceId) -> Vec<EdgeId> {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let mut springs: Vec<EdgeId> = topo
        .wire(topo.face(band).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| {
            matches!(topo.edge(*edge).unwrap().curve(), EdgeCurve::Line)
                && adjacency
                    .faces_for_edge(*edge)
                    .iter()
                    .copied()
                    .any(|face| face != band && topo.face(face).unwrap().surface().is_planar())
        })
        .collect();
    springs.sort_unstable_by_key(|edge| edge.index());
    springs.dedup();
    assert_eq!(
        springs.len(),
        2,
        "one spring per planar support before splitting"
    );
    springs
}

fn tangent_support(topo: &Topology, solid: SolidId, band: FaceId) -> FaceId {
    let spring = spring_edges(topo, solid, band)[0];
    topo.build_adjacency(solid)
        .unwrap()
        .faces_for_edge(spring)
        .iter()
        .copied()
        .find(|face| *face != band)
        .expect("spring has a planar support")
}

fn split_line_edge(
    topo: &mut Topology,
    solid: SolidId,
    target: EdgeId,
    fraction: f64,
    offset: Vec3,
) -> ([EdgeId; 2], VertexId) {
    let source = topo.edge(target).unwrap();
    assert!(matches!(source.curve(), EdgeCurve::Line));
    let start = source.start();
    let end = source.end();
    let start_point = topo.vertex(start).unwrap().point();
    let end_point = topo.vertex(end).unwrap().point();
    let middle = topo.add_vertex(Vertex::new(
        start_point + (end_point - start_point) * fraction + offset,
        Tolerance::new().linear,
    ));
    let first = topo.add_edge(Edge::new(start, middle, EdgeCurve::Line));
    let second = topo.add_edge(Edge::new(middle, end, EdgeCurve::Line));
    let forward = [
        OrientedEdge::new(first, true),
        OrientedEdge::new(second, true),
    ];

    replace_solid_edge(topo, solid, target, &forward);
    ([first, second], middle)
}

fn replace_solid_edge(
    topo: &mut Topology,
    solid: SolidId,
    target: EdgeId,
    forward: &[OrientedEdge],
) {
    let mut wires: Vec<WireId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .flat_map(|face| {
            let face = topo.face(face).unwrap();
            std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .collect::<Vec<_>>()
        })
        .collect();
    wires.sort_unstable_by_key(|wire| wire.index());
    wires.dedup();
    for wire_id in wires {
        let old = topo.wire(wire_id).unwrap();
        if !old.edges().iter().any(|edge| edge.edge() == target) {
            continue;
        }
        let mut replacement = Vec::with_capacity(old.edges().len() + 1);
        for edge in old.edges() {
            if edge.edge() != target {
                replacement.push(*edge);
            } else if edge.is_forward() {
                replacement.extend_from_slice(forward);
            } else {
                replacement.extend(
                    forward
                        .iter()
                        .rev()
                        .map(|edge| OrientedEdge::new(edge.edge(), false)),
                );
            }
        }
        topo.replace_boundary_wire(wire_id, Wire::new(replacement, old.is_closed()).unwrap())
            .unwrap();
    }
}

fn install_backtracking_chain(topo: &mut Topology, solid: SolidId, target: EdgeId) {
    let edge = topo.edge(target).unwrap();
    let start = edge.start();
    let end = edge.end();
    let start_point = topo.vertex(start).unwrap().point();
    let end_point = topo.vertex(end).unwrap().point();
    let direction = end_point - start_point;
    let far = topo.add_vertex(Vertex::new(
        start_point + direction * 0.7,
        Tolerance::new().linear,
    ));
    let near = topo.add_vertex(Vertex::new(
        start_point + direction * 0.3,
        Tolerance::new().linear,
    ));
    let first = topo.add_edge(Edge::new(start, far, EdgeCurve::Line));
    let second = topo.add_edge(Edge::new(far, near, EdgeCurve::Line));
    let third = topo.add_edge(Edge::new(near, end, EdgeCurve::Line));
    replace_solid_edge(
        topo,
        solid,
        target,
        &[
            OrientedEdge::new(first, true),
            OrientedEdge::new(second, true),
            OrientedEdge::new(third, true),
        ],
    );
}

fn assert_exact_box(topo: &Topology, solid: SolidId, scale: f64) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
    let faces = solid_faces(topo, solid).unwrap();
    assert_eq!(faces.len(), 6);
    assert!(
        faces
            .iter()
            .all(|face| topo.face(*face).unwrap().surface().is_planar())
    );
    let expected = 1000.0 * scale.powi(3);
    let actual = solid_volume(topo, solid, 0.01 * scale).unwrap();
    assert!(
        (actual - expected).abs() <= expected * 1e-6,
        "exact box volume {actual} != {expected}"
    );
}

#[test]
fn collinear_split_springs_remove_exactly_across_scale_transform_and_step() {
    for (index, scale) in [0.1_f64, 1.0, 10.0].into_iter().enumerate() {
        let (mut topo, solid, band) = fixture();
        let transform = Mat4::translation(13.0 * scale, -7.0 * scale, 5.0 * scale)
            * Mat4::rotation_z(0.31)
            * Mat4::rotation_y(-0.19)
            * Mat4::scale(scale, scale, scale);
        remus_operations::transform::transform_solid(&mut topo, solid, &transform).unwrap();
        let springs = spring_edges(&topo, solid, band);
        split_line_edge(&mut topo, solid, springs[0], 0.31, Vec3::new(0.0, 0.0, 0.0));
        split_line_edge(&mut topo, solid, springs[1], 0.73, Vec3::new(0.0, 0.0, 0.0));
        let report = validate_solid(&topo, solid, &ValidateOptions::default()).unwrap();
        assert!(
            report.is_valid(),
            "split input invalid: {:?}",
            report.issues
        );

        // Exercise imported split topology at one representative scale.
        let (mut topo, solid, band) = if index == 1 {
            let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
            let mut restored = Topology::new();
            let solid = remus_io::step::reader::read_step(&step, &mut restored).unwrap()[0];
            let band = solid_faces(&restored, solid)
                .unwrap()
                .into_iter()
                .find(|face| {
                    matches!(
                        restored.face(*face).unwrap().surface(),
                        FaceSurface::Cylinder(cylinder)
                            if (cylinder.radius() - scale).abs() <= scale * 1e-8
                    )
                })
                .expect("round-tripped split band");
            (restored, solid, band)
        } else {
            (topo, solid, band)
        };
        let result = resize_blend(&mut topo, solid, band, scale, 0.0)
            .unwrap()
            .solid;
        assert_exact_box(&topo, result, scale);

        let step = remus_io::step::writer::write_step(&topo, &[result]).unwrap();
        let mut restored = Topology::new();
        let restored_solid = remus_io::step::reader::read_step(&step, &mut restored).unwrap()[0];
        assert_exact_box(&restored, restored_solid, scale);
    }
}

#[test]
fn split_springs_support_positive_resize_and_blend_aware_planar_move() {
    let (mut topo, solid, band) = fixture();
    let springs = spring_edges(&topo, solid, band);
    split_line_edge(&mut topo, solid, springs[0], 0.27, Vec3::new(0.0, 0.0, 0.0));
    split_line_edge(&mut topo, solid, springs[1], 0.68, Vec3::new(0.0, 0.0, 0.0));
    let resized = resize_blend(&mut topo, solid, band, 1.0, 2.0)
        .unwrap()
        .solid;
    let rebuilt = solid_faces(&topo, resized)
        .unwrap()
        .into_iter()
        .find_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) => Some(cylinder.radius()),
            _ => None,
        })
        .expect("resized cylindrical band");
    assert!(Tolerance::new().approx_eq(rebuilt, 2.0));
    let expected = 1000.0 - 10.0 * 4.0 * (1.0 - std::f64::consts::FRAC_PI_4);
    let actual = solid_volume(&topo, resized, 0.01).unwrap();
    assert!((actual - expected).abs() < expected * 1e-6);

    let (mut topo, solid, band) = fixture();
    let support = tangent_support(&topo, solid, band);
    let springs = spring_edges(&topo, solid, band);
    split_line_edge(&mut topo, solid, springs[0], 0.37, Vec3::new(0.0, 0.0, 0.0));
    split_line_edge(&mut topo, solid, springs[1], 0.61, Vec3::new(0.0, 0.0, 0.0));
    let moved = remus_operations::push_pull::move_faces(&mut topo, solid, &[support], 1.0).unwrap();
    let report = validate_solid(&topo, moved, &ValidateOptions::default()).unwrap();
    assert!(
        report.is_valid(),
        "moved result invalid: {:?}",
        report.issues
    );
    let fillet_removed = (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
    let expected = 1100.0 - fillet_removed;
    let actual = solid_volume(&topo, moved, 0.01).unwrap();
    assert!((actual - expected).abs() < expected * 1e-6);
}

#[test]
fn split_spring_removal_records_total_boundary_history() {
    let (mut topo, solid, band) = fixture();
    let springs = spring_edges(&topo, solid, band);
    let (_, first_middle) =
        split_line_edge(&mut topo, solid, springs[0], 0.4, Vec3::new(0.0, 0.0, 0.0));
    let (_, second_middle) =
        split_line_edge(&mut topo, solid, springs[1], 0.6, Vec3::new(0.0, 0.0, 0.0));
    let source_keys = solid_entity_keys(&topo, solid).unwrap();
    let result = resize_blend_journaled(&mut topo, solid, band, 1.0, 0.0).unwrap();
    assert_exact_box(&topo, result.solid, 1.0);
    assert!(result.map.deleted.contains(&band.index()));

    let result_vertices: HashSet<_> = remus_topology::explorer::solid_vertices(&topo, result.solid)
        .unwrap()
        .into_iter()
        .collect();
    assert!(!result_vertices.contains(&first_middle));
    assert!(!result_vertices.contains(&second_middle));
    let result_keys = solid_entity_keys(&topo, result.solid).unwrap();
    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == result.op)
        .unwrap();
    let EntryPayload::Evolution { scope, events, .. } = entry.payload() else {
        panic!("resize must publish construction evolution")
    };
    assert_eq!(scope.len(), source_keys.len() + result_keys.len());
    assert!(events.len() >= result_keys.len());
}

#[test]
fn non_collinear_split_spring_refuses_and_rolls_back_geometry_and_journal() {
    let (mut topo, solid, band) = fixture();
    let spring = spring_edges(&topo, solid, band)[0];
    split_line_edge(&mut topo, solid, spring, 0.5, Vec3::new(0.0, 0.0, 0.01));
    let journal_before = topo.journal().snapshot();
    let counts_before = solid_entity_counts(&topo, solid).unwrap();
    let keys_before: HashSet<_> = solid_entity_keys(&topo, solid)
        .unwrap()
        .into_iter()
        .collect();
    let volume_before = solid_volume(&topo, solid, 0.01).unwrap();
    let error = resize_blend_journaled(&mut topo, solid, band, 1.0, 0.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "resize-blend-failed");
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts_before);
    assert_eq!(
        solid_entity_keys(&topo, solid)
            .unwrap()
            .into_iter()
            .collect::<HashSet<_>>(),
        keys_before
    );
    assert!((solid_volume(&topo, solid, 0.01).unwrap() - volume_before).abs() < 1e-12);
    assert_eq!(topo.journal().snapshot(), journal_before);
}

#[test]
fn collinear_but_backtracking_spring_refuses_without_publishing_a_result() {
    let (mut topo, solid, band) = fixture();
    let spring = spring_edges(&topo, solid, band)[0];
    install_backtracking_chain(&mut topo, solid, spring);
    let counts_before = solid_entity_counts(&topo, solid).unwrap();
    let keys_before: HashSet<_> = solid_entity_keys(&topo, solid)
        .unwrap()
        .into_iter()
        .collect();
    let journal_before = topo.journal().snapshot();
    let error = resize_blend_journaled(&mut topo, solid, band, 1.0, 0.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "resize-blend-failed");
    assert!(error.to_string().contains("backtracks or overlaps"));
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts_before);
    assert_eq!(
        solid_entity_keys(&topo, solid)
            .unwrap()
            .into_iter()
            .collect::<HashSet<_>>(),
        keys_before
    );
    assert_eq!(topo.journal().snapshot(), journal_before);
}
