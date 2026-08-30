//! K-S1 regression: a blind coaxial bore in a thin-walled cylinder must not
//! leave an unmatched seam split that cracks the indexed tessellation.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const OUTER_RADIUS: f64 = 32.9;
const HEIGHT: f64 = 25.0;
const BORE_DEPTH: f64 = 21.5;

fn blind_bore(scale: f64, wall_ratio: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let outer = make_cylinder(&mut topo, OUTER_RADIUS * scale, HEIGHT * scale).unwrap();
    let inner_radius = OUTER_RADIUS * (1.0 - wall_ratio) * scale;
    let tool = make_cylinder(&mut topo, inner_radius, BORE_DEPTH * scale).unwrap();
    transform_solid(
        &mut topo,
        tool,
        &Mat4::translation(0.0, 0.0, (HEIGHT - BORE_DEPTH) * scale),
    )
    .unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, outer, tool).unwrap();
    (topo, result)
}

fn edge_use_counts(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut uses = HashMap::<usize, usize>::new();
    for face_id in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id).unwrap().edges() {
                *uses.entry(oriented.edge().index()).or_default() += 1;
            }
        }
    }
    (
        uses.values().filter(|&&count| count == 1).count(),
        uses.values().filter(|&&count| count > 2).count(),
    )
}

#[test]
fn thin_wall_coaxial_blind_bores_are_closed_and_mesh_watertight() {
    for scale in [0.001, 1.0, 1000.0] {
        for wall_ratio in [
            0.01,
            0.015,
            0.018,
            0.02,
            0.05,
            2.5 / 32.9,
            0.088,
            0.09,
            0.10,
        ] {
            let (topo, solid) = blind_bore(scale, wall_ratio);
            let label = format!("scale={scale} wall/r={wall_ratio:.6}");

            assert_eq!(
                edge_use_counts(&topo, solid),
                (0, 0),
                "{label}: B-Rep must be a closed two-manifold"
            );
            let (_, edges, _) =
                remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap();
            assert_eq!(edges, 6, "{label}: blind bore must have six unsplit edges");

            let inner_radius = OUTER_RADIUS * (1.0 - wall_ratio) * scale;
            let expected = PI * (OUTER_RADIUS * scale).powi(2) * HEIGHT * scale
                - PI * inner_radius.powi(2) * BORE_DEPTH * scale;
            let measured = solid_volume(&topo, solid, 0.05 * scale).unwrap();
            assert!(
                (measured - expected).abs() <= 1e-9 * expected.abs().max(1.0),
                "{label}: volume {measured:.12} vs closed form {expected:.12}"
            );

            for deflection in [0.1, 0.05, 0.01] {
                let mesh = tessellate_solid(&topo, solid, deflection * scale).unwrap();
                assert_eq!(
                    (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh)),
                    (0, 0),
                    "{label} deflection={deflection}: indexed mesh must be watertight"
                );
            }
        }
    }
}
