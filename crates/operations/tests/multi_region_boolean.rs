//! Qualification tests for exact Compound-returning boolean regions.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeSet;

use remus_algo::gfa::EdgeEvent;
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanRegionsResult, boolean_regions};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};

fn severing_cut_fixture() -> (Topology, remus_topology::SolidId, remus_topology::SolidId) {
    let mut topo = Topology::new();
    let target = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 2.0, 20.0, 20.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(4.0, -5.0, -5.0)).unwrap();
    (topo, target, tool)
}

fn assert_total_evolution(topo: &Topology, result: &BooleanRegionsResult) {
    let members = topo.compound(result.compound).unwrap().solids();
    assert_eq!(members.len(), result.regions.len());
    for (&member, region) in members.iter().zip(&result.regions) {
        assert_eq!(member, region.solid);
        let faces: BTreeSet<_> = solid_faces(topo, member)
            .unwrap()
            .into_iter()
            .map(remus_topology::FaceId::index)
            .collect();
        let edges: BTreeSet<_> = solid_edges(topo, member)
            .unwrap()
            .into_iter()
            .map(remus_topology::EdgeId::index)
            .collect();
        let vertices: BTreeSet<_> = solid_vertices(topo, member)
            .unwrap()
            .into_iter()
            .map(remus_topology::VertexId::index)
            .collect();
        assert_eq!(
            region
                .evolution
                .faces
                .iter()
                .map(|(index, _)| *index)
                .collect::<BTreeSet<_>>(),
            faces
        );
        assert_eq!(
            region
                .evolution
                .edges
                .iter()
                .map(|(index, _)| *index)
                .collect::<BTreeSet<_>>(),
            edges
        );
        assert_eq!(
            region
                .evolution
                .vertices
                .iter()
                .map(|(index, _)| *index)
                .collect::<BTreeSet<_>>(),
            vertices
        );
        assert!(
            region
                .evolution
                .edges
                .iter()
                .all(|(_, event)| !matches!(event, EdgeEvent::Unresolved))
        );
    }
}

#[test]
fn severing_cut_returns_two_valid_regions_with_exact_volume_and_evolution() {
    let (mut topo, target, tool) = severing_cut_fixture();
    let result = boolean_regions(&mut topo, BooleanOp::Cut, target, tool).unwrap();
    let members = topo.compound(result.compound).unwrap().solids().to_vec();
    assert_eq!(members.len(), 2);

    let mut volumes = members
        .iter()
        .map(|&solid| {
            assert!(
                remus_operations::validate::validate_solid(&topo, solid)
                    .unwrap()
                    .is_valid()
            );
            solid_volume(&topo, solid, 0.01).unwrap()
        })
        .collect::<Vec<_>>();
    volumes.sort_by(f64::total_cmp);
    assert!((volumes[0] - 400.0).abs() < 1.0e-9, "{volumes:?}");
    assert!((volumes[1] - 400.0).abs() < 1.0e-9, "{volumes:?}");
    assert_total_evolution(&topo, &result);
}

#[test]
fn disjoint_fuse_returns_two_exact_regions_deterministically() {
    fn run() -> (Vec<f64>, Vec<remus_algo::gfa::EntityEvolution>) {
        let mut topo = Topology::new();
        let a = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let b = make_box(&mut topo, 5.0, 2.0, 1.0).unwrap();
        transform_solid(&mut topo, b, &Mat4::translation(10.0, 0.0, 0.0)).unwrap();

        let result = boolean_regions(&mut topo, BooleanOp::Fuse, a, b).unwrap();
        assert_total_evolution(&topo, &result);
        let mut volumes = result
            .regions
            .iter()
            .map(|region| solid_volume(&topo, region.solid, 0.01).unwrap())
            .collect::<Vec<_>>();
        volumes.sort_by(f64::total_cmp);
        let evolution = result
            .regions
            .into_iter()
            .map(|region| region.evolution)
            .collect();
        (volumes, evolution)
    }

    let first = run();
    let second = run();
    assert_eq!(first, second);
    assert_eq!(first.0, vec![10.0, 24.0]);
}
