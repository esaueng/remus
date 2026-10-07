//! Measurement may omit normals but must preserve every geometry bit.
#![allow(clippy::unwrap_used)]

use super::{solid::tessellate_solid_for_measurement, tessellate_solid};
use remus_topology::Topology;

#[test]
fn shading_omission_preserves_closed_and_repaired_meshes_with_warm_cache() {
    let mut topo = Topology::new();
    let mut solids = remus_io::step::reader::read_step(
        include_str!("../../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .unwrap();
    solids.push(crate::primitives::make_box(&mut topo, 2., 3., 4.).unwrap());
    solids.push(crate::primitives::make_cylinder(&mut topo, 2., 3.).unwrap());
    super::enable_face_mesh_cache();
    for solid in solids {
        for deflection in [0.08, 0.003] {
            super::clear_face_mesh_cache();
            let baseline = tessellate_solid(&topo, solid, deflection).unwrap();
            // Start measurement cold as well as replaying its cached faces.
            super::clear_face_mesh_cache();
            for _ in 0..2 {
                let measured = tessellate_solid_for_measurement(&topo, solid, deflection).unwrap();
                assert_eq!(measured.indices, baseline.indices);
                assert_eq!(measured.positions.len(), baseline.positions.len());
                for (a, b) in measured.positions.iter().zip(&baseline.positions) {
                    assert_eq!(a.0.map(f64::to_bits), b.0.map(f64::to_bits));
                }
                // A quantity cache must not poison a subsequent display replay.
                let display = tessellate_solid(&topo, solid, deflection).unwrap();
                assert_eq!(display.indices, baseline.indices);
                assert_eq!(display.normals, baseline.normals);
            }
        }
    }
    super::disable_face_mesh_cache();
}
