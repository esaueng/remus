//! Phase 4.3 regression coverage for transactional cylindrical bore moves.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_offset::OffsetError;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.002;

fn drilled_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 40.0, 40.0, 10.0).expect("plate");
    let drill = make_cylinder(topo, 3.0, 10.0).expect("drill");
    transform_solid(topo, drill, &Mat4::translation(20.0, 20.0, 0.0)).expect("place drill");
    boolean(topo, BooleanOp::Cut, plate, drill).expect("drill through-bore")
}

fn bore_face(topo: &Topology, solid: SolidId) -> FaceId {
    let cylinders: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).expect("face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .collect();
    assert_eq!(cylinders.len(), 1, "expected one cylindrical bore wall");
    assert!(
        topo.face(cylinders[0]).expect("bore face").is_reversed(),
        "the selected cylinder must be inward-facing"
    );
    cylinders[0]
}

fn assert_bore_radius(topo: &Topology, solid: SolidId, expected: f64) {
    let face = bore_face(topo, solid);
    let FaceSurface::Cylinder(cylinder) = topo.face(face).expect("bore face").surface() else {
        unreachable!();
    };
    assert!(
        Tolerance::new().approx_eq(cylinder.radius(), expected),
        "bore radius {} != {expected}",
        cylinder.radius()
    );
}

fn assert_volume(topo: &Topology, solid: SolidId, radius: f64) {
    let expected = 40.0f64.mul_add(40.0 * 10.0, -(PI * radius * radius * 10.0));
    let actual = solid_volume(topo, solid, DEFLECTION).expect("solid volume");
    let tolerance = expected.abs().mul_add(2e-4, 1e-6);
    assert!(
        (actual - expected).abs() <= tolerance,
        "volume {actual} != {expected} within {tolerance}"
    );
}

fn positional_edge_health(positions: &[Point3], indices: &[u32]) -> (usize, usize) {
    let quantization = 1e6;
    let mut canonical = HashMap::new();
    let mut remap = vec![0_u32; positions.len()];
    for (index, point) in positions.iter().enumerate() {
        let key = (
            (point.x() * quantization).round() as i64,
            (point.y() * quantization).round() as i64,
            (point.z() * quantization).round() as i64,
        );
        let next = canonical.len() as u32;
        remap[index] = *canonical.entry(key).or_insert(next);
    }
    let mut uses = HashMap::new();
    for triangle in indices.chunks_exact(3) {
        let vertices = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for &(first, second) in &[
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ] {
            let edge = if first < second {
                (first, second)
            } else {
                (second, first)
            };
            *uses.entry(edge).or_insert(0_usize) += 1;
        }
    }
    (
        uses.values().filter(|&&count| count == 1).count(),
        uses.values().filter(|&&count| count > 2).count(),
    )
}

fn assert_verified(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).expect("strict solid validation");
    assert!(report.is_valid(), "strict validation: {:?}", report.issues);

    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).expect("solid tessellation");
    assert!(is_watertight(&mesh), "index-based mesh is not watertight");
    let (boundary, non_manifold) = positional_edge_health(&mesh.positions, &mesh.indices);
    assert_eq!(boundary, 0, "position-welded mesh has boundary edges");
    assert_eq!(
        non_manifold, 0,
        "position-welded mesh has non-manifold edges"
    );
}

fn live_counts(topo: &Topology) -> (usize, usize, usize, usize, usize, usize) {
    (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    )
}

#[test]
fn signed_radial_moves_resize_a_bore_without_changing_topology() {
    for (distance, radius) in [(-2.0, 5.0), (1.0, 2.0)] {
        let mut topo = Topology::new();
        let source = drilled_plate(&mut topo);
        let source_counts = solid_entity_counts(&topo, source).expect("source counts");
        let bore = bore_face(&topo, source);

        let result = move_faces(&mut topo, source, &[bore], distance).expect("radial bore move");

        assert_eq!(
            solid_entity_counts(&topo, result).expect("result counts"),
            source_counts,
            "move_faces must preserve the source adjacency graph"
        );
        assert_bore_radius(&topo, result, radius);
        assert_volume(&topo, result, radius);
        assert_verified(&topo, result);
    }
}

#[test]
fn repeated_radial_moves_keep_exact_watertight_bore_tessellation() {
    let mut topo = Topology::new();
    let source = drilled_plate(&mut topo);
    let source_bore = bore_face(&topo, source);
    let widened = move_faces(&mut topo, source, &[source_bore], -2.0).expect("widen bore");
    let widened_bore = bore_face(&topo, widened);
    let narrowed = move_faces(&mut topo, widened, &[widened_bore], 1.0).expect("narrow bore");

    assert_bore_radius(&topo, narrowed, 4.0);
    assert_volume(&topo, narrowed, 4.0);
    assert_verified(&topo, narrowed);
}

#[test]
fn rigidly_transformed_bore_move_stays_exact_and_watertight() {
    let mut topo = Topology::new();
    let source = drilled_plate(&mut topo);
    let transform =
        Mat4::translation(7.0, -11.0, 5.0) * Mat4::rotation_x(0.7) * Mat4::rotation_y(-0.4);
    transform_solid(&mut topo, source, &transform).expect("transform drilled plate");
    let bore = bore_face(&topo, source);

    let result = move_faces(&mut topo, source, &[bore], -2.0).expect("widen transformed bore");

    assert_bore_radius(&topo, result, 5.0);
    assert_volume(&topo, result, 5.0);
    assert_verified(&topo, result);
}

#[test]
fn collapsed_or_intersecting_bore_moves_restore_every_temporary_entity() {
    for distance in [3.0, -25.0] {
        let mut topo = Topology::new();
        let source = drilled_plate(&mut topo);
        let bore = bore_face(&topo, source);
        let before = live_counts(&topo);

        let error = move_faces(&mut topo, source, &[bore], distance)
            .expect_err("invalid radial move must fail closed");

        assert_eq!(
            live_counts(&topo),
            before,
            "failure leaked topology: {error}"
        );
        assert_bore_radius(&topo, source, 3.0);
        assert_volume(&topo, source, 3.0);
        assert_verified(&topo, source);
    }
}

#[test]
fn radial_moves_reject_mixed_groups_and_outward_facing_cylinders() {
    let mut topo = Topology::new();
    let source = drilled_plate(&mut topo);
    let bore = bore_face(&topo, source);
    let planar = solid_faces(&topo, source)
        .expect("solid faces")
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).expect("face").surface(),
                FaceSurface::Plane { .. }
            )
        })
        .expect("planar face");
    let mixed =
        move_faces(&mut topo, source, &[bore, planar], -1.0).expect_err("mixed radial selection");
    assert!(matches!(
        mixed,
        OperationsError::Offset(OffsetError::MoveGroupMismatch { .. })
    ));

    let boss = make_cylinder(&mut topo, 3.0, 10.0).expect("cylindrical boss");
    let wall = solid_faces(&topo, boss)
        .expect("boss faces")
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).expect("face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .expect("boss wall");
    let unsupported = move_faces(&mut topo, boss, &[wall], 1.0).expect_err("boss radial move");
    assert!(matches!(
        unsupported,
        OperationsError::Offset(OffsetError::UnsupportedMoveFace { .. })
    ));
}
