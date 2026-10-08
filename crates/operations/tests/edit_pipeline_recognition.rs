//! A local request must preserve whole-solid material classification.
#![allow(clippy::unwrap_used)]
use remus_operations::{
    primitives::make_box,
    query::{solid_edge_relations, solid_edge_relations_subset},
};
use remus_topology::Topology;

#[test]
fn selected_relations_match_whole_solid_bits_in_caller_order() {
    let mut topo = Topology::new();
    let imported = remus_io::step::reader::read_step(
        include_str!("../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .unwrap();
    for solid in imported {
        for probe in [None, Some(0.01)] {
            let all = solid_edge_relations(&topo, solid, probe).unwrap();
            let selected: Vec<_> = all.iter().step_by(13).rev().map(|row| row.edge).collect();
            let subset = solid_edge_relations_subset(&topo, solid, &selected, probe).unwrap();
            assert_eq!(subset.len(), selected.len());
            for (actual, edge) in subset.iter().zip(selected) {
                let expected = all.iter().find(|row| row.edge == edge).unwrap();
                assert_eq!(actual.edge, expected.edge);
                assert_eq!(actual.concavity, expected.concavity);
                assert_eq!(
                    actual.dihedral_angle.map(f64::to_bits),
                    expected.dihedral_angle.map(f64::to_bits)
                );
            }
        }
    }
}

#[test]
fn duplicate_foreign_and_invalid_probe_requests_refuse() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 2., 3., 4.).unwrap();
    let b = make_box(&mut topo, 4., 3., 2.).unwrap();
    let edge = solid_edge_relations(&topo, a, None).unwrap()[0].edge;
    assert!(solid_edge_relations_subset(&topo, a, &[edge, edge], None).is_err());
    assert!(solid_edge_relations_subset(&topo, b, &[edge], None).is_err());
    assert!(solid_edge_relations_subset(&topo, a, &[], Some(f64::NAN)).is_err());
    assert!(
        solid_edge_relations_subset(&topo, a, &[], None)
            .unwrap()
            .is_empty()
    );
}
