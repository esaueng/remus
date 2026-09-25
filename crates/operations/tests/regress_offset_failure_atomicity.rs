//! Public offset refusals must retire temporary geometry, including late failures.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_operations::measure::solid_volume;
use remus_operations::offset_v2::{offset_solid_v2, shell_v2};
use remus_operations::primitives::make_box;
use remus_topology::Topology;

fn live_counts(topo: &Topology) -> [usize; 13] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_compounds(),
        topo.num_compsolids(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
        topo.attributes().len(),
        topo.journal().len(),
    ]
}

#[test]
fn offset_postcondition_refusal_restores_topology() {
    for shell in [false, true] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let before = live_counts(&topo);
        let before_slots = topo.allocated_slot_count();
        let vertices: Vec<_> = topo
            .vertices()
            .iter()
            .map(|(id, v)| (id, v.point()))
            .collect();
        let error = if shell {
            shell_v2(&mut topo, solid, -6.0, &[])
        } else {
            offset_solid_v2(&mut topo, solid, -6.0)
        }
        .unwrap_err();
        assert!(error.to_string().contains("collapsed"), "{error}");
        assert_eq!(live_counts(&topo), before);
        assert!(topo.allocated_slot_count() > before_slots);
        for (id, point) in vertices {
            assert!((topo.vertex(id).unwrap().point() - point).length() < 1e-12);
        }
        assert!((solid_volume(&topo, solid, 0.01).unwrap() - 1000.0).abs() < 1e-9);
        let slot_floor = topo.allocated_slot_count();
        let result = offset_solid_v2(&mut topo, solid, 1.0).unwrap();
        assert!(topo.allocated_slot_count() > slot_floor);
        assert!((solid_volume(&topo, result, 0.01).unwrap() - 1728.0).abs() < 1e-9);
    }
}

#[test]
fn offset_engine_refusal_restores_topology() {
    for shell in [false, true] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let before = live_counts(&topo);
        let before_slots = topo.allocated_slot_count();
        let error = if shell {
            remus_offset::thick_solid(
                &mut topo,
                solid,
                -5.0,
                &[],
                remus_offset::OffsetOptions::default(),
            )
        } else {
            remus_offset::offset_solid(
                &mut topo,
                solid,
                -5.0,
                remus_offset::OffsetOptions::default(),
            )
        }
        .unwrap_err();
        assert_eq!(live_counts(&topo), before, "{error}");
        assert!(topo.allocated_slot_count() > before_slots);
        assert!((solid_volume(&topo, solid, 0.01).unwrap() - 1000.0).abs() < 1e-9);
    }
}
