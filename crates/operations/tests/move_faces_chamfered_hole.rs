//! Face moves must carry a bore's chamfer together with its rounded cap.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::disallowed_methods
)]

use remus_math::mat::Mat4;
use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use std::f64::consts::PI;

fn rounded_plate(topo: &mut Topology, mouths: usize, chamfer_depth: f64) -> SolidId {
    rounded_plate_with_angle(topo, mouths, chamfer_depth, PI / 4.0)
}

fn rounded_plate_with_angle(
    topo: &mut Topology,
    mouths: usize,
    chamfer_depth: f64,
    half_angle: f64,
) -> SolidId {
    let mut body = make_box(topo, 30.0, 30.0, 12.0).unwrap();
    for (level, radius) in [(12.0, 2.0), (0.0, 1.0)] {
        let edges = solid_edges(topo, body)
            .unwrap()
            .into_iter()
            .filter(|&id| {
                let edge = topo.edge(id).unwrap();
                [edge.start(), edge.end()].into_iter().all(|vertex| {
                    let point = topo.vertex(vertex).unwrap().point();
                    (point.z() - level).abs() < 1e-8 && point.x().abs() < 1e-8
                })
            })
            .collect::<Vec<_>>();
        body = fillet_v2(topo, body, &edges, radius).unwrap().solid;
    }
    let bore = make_cylinder(topo, 3.0, 12.0).unwrap();
    transform_solid(topo, bore, &Mat4::translation(15.0, 15.0, 0.0)).unwrap();
    body = boolean(topo, BooleanOp::Cut, body, bore).unwrap();
    // Remus measures a cone's half-angle from its radial plane.
    let mouth_radius = 3.0 + chamfer_depth / half_angle.tan();
    for (r0, r1, z) in [
        (3.0, mouth_radius, 12.0 - chamfer_depth),
        (mouth_radius, 3.0, 0.0),
    ]
    .into_iter()
    .take(mouths)
    {
        let cutter = make_cone(topo, r0, r1, chamfer_depth).unwrap();
        transform_solid(topo, cutter, &Mat4::translation(15.0, 15.0, z)).unwrap();
        body = boolean(topo, BooleanOp::Cut, body, cutter).unwrap();
    }
    remus_operations::heal::unify_faces(topo, body).unwrap();
    body
}

#[test]
fn non_45_degree_mouths_keep_their_cone_angle_and_the_opposite_end() {
    for angle in [PI / 6.0, PI / 3.0] {
        let mut topo = Topology::new();
        let body = rounded_plate_with_angle(&mut topo, 2, 1.0, angle);
        let before = solid_volume(&topo, body, 0.001).unwrap();
        for z in [1.0, -1.0] {
            let selected = cap(&topo, body, Vec3::new(0.0, 0.0, z));
            for distance in [-1.0, 1.25] {
                let moved = move_faces(&mut topo, body, &[selected], distance).unwrap();
                verified(&topo, moved);
                for face in solid_faces(&topo, moved).unwrap() {
                    if let FaceSurface::Cone(cone) = topo.face(face).unwrap().surface() {
                        assert!((cone.half_angle() - angle).abs() < 1e-10);
                    }
                }
                let expected = before + (900.0 - 9.0 * PI) * distance;
                assert!((solid_volume(&topo, moved, 0.001).unwrap() - expected).abs() < 0.03);
                let fixed = cap(&topo, moved, Vec3::new(0.0, 0.0, -z));
                if let FaceSurface::Plane { d, .. } = topo.face(fixed).unwrap().surface() {
                    assert!((d.abs() - if z > 0.0 { 0.0 } else { 12.0 }).abs() < 1e-8);
                }
            }
        }
    }
}

fn cap(topo: &Topology, body: SolidId, direction: Vec3) -> FaceId {
    solid_faces(topo, body)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(direction) > 1.0 - 1e-8)
        })
        .max_by(|&a, &b| {
            remus_operations::measure::face_area(topo, a, 0.001)
                .unwrap()
                .total_cmp(&remus_operations::measure::face_area(topo, b, 0.001).unwrap())
        })
        .unwrap()
}

fn verified(topo: &Topology, body: SolidId) {
    let report = validate_solid(topo, body).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, body, 0.01, 0.1).unwrap();
    assert!(is_watertight(&mesh));
}

#[test]
fn rounded_caps_carry_one_or_two_chamfered_bore_mouths() {
    for mouths in [1, 2] {
        let mut topo = Topology::new();
        let body = rounded_plate(&mut topo, mouths, 1.0);
        verified(&topo, body);
        let source = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
        let before = solid_volume(&topo, body, 0.001).unwrap();
        let counts = solid_entity_counts(&topo, body).unwrap();
        for z in [1.0, -1.0] {
            let selected = cap(&topo, body, Vec3::new(0.0, 0.0, z));
            for distance in [-1.0, 0.01, 1.25, 6.0] {
                let moved = move_faces(&mut topo, body, &[selected], distance)
                    .unwrap_or_else(|e| panic!("mouths={mouths}, z={z}, distance={distance}: {e}"));
                verified(&topo, moved);
                assert_eq!(solid_entity_counts(&topo, moved).unwrap(), counts);
                let after = solid_volume(&topo, moved, 0.001).unwrap();
                let expected = before + (900.0 - 9.0 * PI) * distance;
                assert!(
                    (after - expected).abs() < 0.03,
                    "volume {after} != {expected}"
                );
                assert_eq!(
                    remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
                    source
                );
                assert_eq!(
                    solid_faces(&topo, moved)
                        .unwrap()
                        .into_iter()
                        .filter(|&face| matches!(
                            topo.face(face).unwrap().surface(),
                            FaceSurface::Cone(_)
                        ))
                        .count(),
                    mouths
                );
            }
        }
    }
}

#[test]
fn inward_move_refuses_before_the_two_mouths_meet() {
    let mut topo = Topology::new();
    let body = rounded_plate(&mut topo, 2, 3.0);
    let selected = cap(&topo, body, Vec3::new(0.0, 0.0, 1.0));
    let source = remus_io::arena_io::serialize_solid(&topo, body).unwrap();
    // The two cone springs are 6 mm apart; the outside fillets still have
    // 9 mm of clearance, so this specifically exercises the bore limit.
    for distance in [-6.0, -6.01] {
        let error = move_faces(&mut topo, body, &[selected], distance).unwrap_err();
        assert!(
            error.to_string().contains("moving blend reaches fixed rim"),
            "{error}"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, body).unwrap(),
            source
        );
    }
}

#[test]
fn rotated_step_mouths_support_repeated_edits_and_prepared_preview() {
    for imported in [false, true] {
        let mut topo = Topology::new();
        let mut body = rounded_plate(&mut topo, 2, 1.0);
        let angle = 0.63;
        transform_solid(&mut topo, body, &Mat4::rotation_y(angle)).unwrap();
        transform_solid(&mut topo, body, &Mat4::translation(41.0, -23.0, 8.0)).unwrap();
        if imported {
            let step = remus_io::step::write_step(&topo, &[body]).unwrap();
            topo = Topology::new();
            body = remus_io::step::read_step(&step, &mut topo).unwrap()[0];
        }
        let direction = Vec3::new(angle.sin(), 0.0, angle.cos());
        let before = solid_volume(&topo, body, 0.001).unwrap();
        for distance in [1.25, -0.75, 0.01] {
            let selected = cap(&topo, body, direction);
            let mut preview_topo = topo.clone();
            let mut prepared = PreparedFaceMove::prepare(&preview_topo, body, &[selected]).unwrap();
            let preview = prepared.move_faces(&mut preview_topo, distance).unwrap();
            let moved = move_faces(&mut topo, body, &[selected], distance).unwrap();
            assert_eq!(
                remus_io::arena_io::serialize_solid(&preview_topo, preview.solid).unwrap(),
                remus_io::arena_io::serialize_solid(&topo, moved).unwrap()
            );
            verified(&topo, moved);
            body = moved;
        }
        let expected = before + (900.0 - 9.0 * PI) * 0.51;
        assert!((solid_volume(&topo, body, 0.001).unwrap() - expected).abs() < 0.03);
        let step = remus_io::step::write_step(&topo, &[body]).unwrap();
        let mut reopened = Topology::new();
        let restored = remus_io::step::read_step(&step, &mut reopened).unwrap()[0];
        verified(&reopened, restored);
        assert!((solid_volume(&reopened, restored, 0.001).unwrap() - expected).abs() < 0.03);
    }
}
