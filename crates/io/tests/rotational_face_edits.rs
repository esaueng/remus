//! Imported rotational bodies retain their analytic profile through direct edits.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::{
    blend_ops::fillet_v2,
    measure::solid_volume,
    prepared_move::PreparedFaceMove,
    push_pull::{move_faces, resize_cylindrical_face},
    resize_blend::resize_blend,
};
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    explorer::{face_vertices, solid_edges, solid_entity_counts, solid_faces},
    face::{FaceId, FaceSurface},
    solid::SolidId,
};
use std::f64::consts::PI;

fn imported(topo: &mut Topology) -> SolidId {
    remus_io::step::read_step(include_str!("data/shapr3d_walking_stick_foot.step"), topo).unwrap()
        [0]
}

fn cylinder(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let faces:Vec<_>=solid_faces(topo,solid).unwrap().into_iter().filter(|&face|matches!(topo.face(face).unwrap().surface(),FaceSurface::Cylinder(c) if (c.radius()-radius).abs()<1e-7)).collect();
    assert_eq!(faces.len(), 1, "one cylinder at radius {radius}");
    faces[0]
}

fn plane(topo: &Topology, solid: SolidId, z: f64, normal: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() * normal > 0.99)
                && face_vertices(topo, face)
                    .unwrap()
                    .iter()
                    .all(|&vertex| (topo.vertex(vertex).unwrap().point().z() - z).abs() < 1e-7)
        })
        .expect("requested planar face")
}

fn valid(topo: &Topology, solid: SolidId) {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh =
        remus_operations::tessellate::tessellate_solid_with_tolerance(topo, solid, 0.02, 0.1)
            .unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(
        quality.is_watertight(),
        "mesh boundary {}, nonmanifold {}",
        quality.boundary_edges,
        quality.non_manifold_edges
    );
}

fn roundtrip(topo: &Topology, solid: SolidId) {
    let step = remus_io::step::writer::write_step(topo, &[solid]).unwrap();
    let mut restored = Topology::new();
    let round = remus_io::step::read_step(&step, &mut restored).unwrap()[0];
    valid(&restored, round);
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap(),
        solid_entity_counts(&restored, round).unwrap()
    );
    assert!(
        (solid_volume(topo, solid, 0.01).unwrap() - solid_volume(&restored, round, 0.01).unwrap())
            .abs()
            < 1e-5
    );
}

fn edited(topo: &mut Topology) -> SolidId {
    let mut solid = imported(topo);
    for (height, from, to) in [
        (30.156_854_249, 4.0, 3.0),
        (5.656_854_249, 4.0, 8.6),
        (7.562_236_639, 8.6, 5.0),
        (23.343_145_751, 4.0, 5.0),
        (29.742_640_685, 3.0, 5.0),
    ] {
        let face=solid_faces(topo,solid).unwrap().into_iter().find(|&face|matches!(topo.face(face).unwrap().surface(),FaceSurface::Torus(t) if (t.minor_radius()-from).abs()<1e-6 && (t.center().z()-height).abs()<1e-6)).unwrap();
        solid = resize_blend(topo, solid, face, from, to).unwrap().solid;
    }
    for radius in [16.5, 12.5] {
        let edge=solid_edges(topo,solid).unwrap().into_iter().find(|&edge|matches!(topo.edge(edge).unwrap().curve(),EdgeCurve::Circle(c) if (c.radius()-radius).abs()<1e-7 && (c.center().z()-50.0).abs()<1e-7)).unwrap();
        solid = fillet_v2(topo, solid, &[edge], 1.0).unwrap().solid;
    }
    let bore = cylinder(topo, solid, 12.5);
    solid = resize_cylindrical_face(topo, solid, bore, 12.0).unwrap();
    let mut level = 9.0;
    for offset in [13.0, -7.5, -7.0, 9.0] {
        let floor = plane(topo, solid, level, 1.0);
        solid = move_faces(topo, solid, &[floor], offset).unwrap();
        level += offset;
    }
    let bore = cylinder(topo, solid, 12.0);
    solid = resize_cylindrical_face(topo, solid, bore, 11.5).unwrap();
    assert_eq!(solid_entity_counts(topo, solid).unwrap(), (13, 22, 12));
    valid(topo, solid);
    solid
}

#[test]
fn bore_floor_crosses_exterior_shoulder_with_exact_volume_and_prepared_parity() {
    for offset in [-10.0, 14.0, 25.0] {
        let mut topo = Topology::new();
        let solid = edited(&mut topo);
        let floor = plane(&topo, solid, 16.5, 1.0);
        let before = solid_volume(&topo, solid, 0.01).unwrap();
        let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[floor]).unwrap();
        for result in [
            move_faces(&mut topo, solid, &[floor], offset).unwrap(),
            prepared.move_faces(&mut topo, offset).unwrap().solid,
        ] {
            valid(&topo, result);
            assert_eq!(solid_entity_counts(&topo, result).unwrap(), (13, 22, 12));
            let expected = before + PI * 11.5_f64.powi(2) * offset;
            assert!((solid_volume(&topo, result, 0.01).unwrap() - expected).abs() < 1e-5);
            assert_eq!(
                classify_point(
                    &topo,
                    result,
                    Point3::new(0.0, 0.0, 16.5 + offset - 0.1),
                    &ClassifyOptions::default()
                )
                .unwrap(),
                PointClassification::Inside
            );
            assert_eq!(
                classify_point(
                    &topo,
                    result,
                    Point3::new(0.0, 0.0, 16.5 + offset + 0.1),
                    &ClassifyOptions::default()
                )
                .unwrap(),
                PointClassification::Outside
            );
            roundtrip(&topo, result);
        }
        assert!((solid_volume(&topo, solid, 0.01).unwrap() - before).abs() < 1e-6);
    }
}

#[test]
fn conical_bottom_moves_with_exact_frustum_volume() {
    for offset in [-1.0, 1.0, 3.0] {
        let mut topo = Topology::new();
        let solid = edited(&mut topo);
        let bottom = plane(&topo, solid, 0.0, -1.0);
        let before = solid_volume(&topo, solid, 0.01).unwrap();
        let result = move_faces(&mut topo, solid, &[bottom], offset).unwrap();
        valid(&topo, result);
        assert_eq!(solid_entity_counts(&topo, result).unwrap(), (13, 22, 12));
        let radius = 16.0 - offset;
        let expected = before + offset * PI * (16.0 * 16.0 + 16.0 * radius + radius * radius) / 3.0;
        assert!((solid_volume(&topo, result, 0.01).unwrap() - expected).abs() < 1e-5);
        roundtrip(&topo, result);
    }
}

#[test]
fn small_and_large_bore_resizes_preserve_rim_floor_and_volume() {
    for radius in [3.5, 9.5, 11.0, 11.45, 11.55, 12.0, 13.5, 14.49] {
        let mut topo = Topology::new();
        let solid = edited(&mut topo);
        let bore = cylinder(&topo, solid, 11.5);
        let before = solid_volume(&topo, solid, 0.01).unwrap();
        let result = resize_cylindrical_face(&mut topo, solid, bore, radius).unwrap();
        valid(&topo, result);
        assert_eq!(solid_entity_counts(&topo, result).unwrap(), (13, 22, 12));
        let expected = before
            - PI * (radius * radius - 11.5 * 11.5) * 33.5
            - 2.0 * PI * (1.0 - PI / 4.0) * (radius - 11.5);
        assert!((solid_volume(&topo, result, 0.01).unwrap() - expected).abs() < 1e-5);
        assert_eq!(
            classify_point(
                &topo,
                result,
                Point3::new(radius - 0.05, 0.0, 35.0),
                &ClassifyOptions::default()
            )
            .unwrap(),
            PointClassification::Outside
        );
        assert_eq!(
            classify_point(
                &topo,
                result,
                Point3::new(radius + 0.05, 0.0, 35.0),
                &ClassifyOptions::default()
            )
            .unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            topo.face(plane(&topo, result, 16.5, 1.0))
                .unwrap()
                .effective_plane_normal()
                .unwrap(),
            Vec3::new(0.0, 0.0, 1.0)
        );
        roundtrip(&topo, result);
    }
}

#[test]
fn both_outer_walls_carry_their_incident_blends_without_new_ledges() {
    for (from, radius, z) in [
        (20.0, 19.5, 14.0),
        (20.0, 20.5, 14.0),
        (16.5, 16.0, 40.0),
        (16.5, 17.0, 40.0),
    ] {
        let mut topo = Topology::new();
        let solid = edited(&mut topo);
        let wall = cylinder(&topo, solid, from);
        let result = resize_cylindrical_face(&mut topo, solid, wall, radius).unwrap();
        valid(&topo, result);
        assert_eq!(solid_entity_counts(&topo, result).unwrap(), (13, 22, 12));
        assert_eq!(
            classify_point(
                &topo,
                result,
                Point3::new(radius - 0.05, 0.0, z),
                &ClassifyOptions::default()
            )
            .unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            classify_point(
                &topo,
                result,
                Point3::new(radius + 0.05, 0.0, z),
                &ClassifyOptions::default()
            )
            .unwrap(),
            PointClassification::Outside
        );
        for original in solid_faces(&topo, solid).unwrap() {
            if let FaceSurface::Cone(cone) = topo.face(original).unwrap().surface() {
                assert!(solid_faces(&topo,result).unwrap().into_iter().any(|face|matches!(topo.face(face).unwrap().surface(),FaceSurface::Cone(c) if (c.apex()-cone.apex()).length()<1e-9 && c.axis()==cone.axis() && c.half_angle().to_bits()==cone.half_angle().to_bits())));
            }
        }
        roundtrip(&topo, result);
    }
}

#[test]
fn collisions_and_collapsed_sections_refuse_and_preserve_input() {
    for (from, radius) in [
        (11.5, 20.0),
        (11.5, 14.5),
        (16.5, 12.0),
        (20.0, 11.0),
        (20.0, 50.0),
    ] {
        let mut topo = Topology::new();
        let solid = edited(&mut topo);
        let wall = cylinder(&topo, solid, from);
        let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
        assert!(
            resize_cylindrical_face(&mut topo, solid, wall, radius).is_err(),
            "r{from} to r{radius} must refuse"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
            before
        );
        valid(&topo, solid);
    }
    for (height, normal, offset) in [
        (16.5, 1.0, -20.0),
        (16.5, 1.0, 40.0),
        (0.0, -1.0, 16.0),
        (0.0, -1.0, 16.1),
        (0.0, -1.0, -30.0),
    ] {
        let mut topo = Topology::new();
        let solid = edited(&mut topo);
        let cap = plane(&topo, solid, height, normal);
        let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
        let error = move_faces(&mut topo, solid, &[cap], offset)
            .expect_err("cap must refuse collision or collapse");
        if normal < 0.0 && offset >= 16.0 {
            assert!(
                error
                    .to_string()
                    .contains("conical cap reaches or crosses its support apex"),
                "{error}"
            );
        }
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
            before
        );
        valid(&topo, solid);
    }
}

#[test]
fn repeated_journaled_radial_edits_preserve_every_entity_reference_at_scale_and_placement() {
    use remus_math::mat::Mat4;
    use remus_operations::journal_ops::{resize_cylindrical_face_journaled, solid_entity_keys};
    use remus_topology::{
        journal::{EntityKind, EventDraft, EvolutionDraft},
        naming::{PersistentRef, Provenance, Resolution, resolve},
    };
    for scale in [0.1, 1.0, 10.0] {
        let mut topo = Topology::new();
        let mut solid = edited(&mut topo);
        let transform = Mat4::translation(23.0 * scale, -7.0 * scale, 4.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::scale(scale, scale, scale);
        remus_operations::transform::transform_solid(&mut topo, solid, &transform).unwrap();
        let keys = solid_entity_keys(&topo, solid).unwrap();
        let pending = topo.journal_begin("fixture");
        let mut draft = EvolutionDraft::construction();
        draft.add_scope(keys.iter().copied());
        for &key in &keys {
            draft.push(key, EventDraft::Generated { sources: vec![] });
        }
        let anchor = topo.journal_record_evolution(pending, draft).unwrap();
        let refs: Vec<_> = [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex]
            .into_iter()
            .flat_map(|kind| {
                (0..keys.iter().filter(|key| key.kind == kind).count())
                    .map(move |index| PersistentRef::operation_output(anchor, kind, index))
            })
            .collect();
        for (from, next) in [(11.5, 11.0), (11.0, 12.0), (20.0, 19.5), (16.5, 16.0)] {
            let face = cylinder(&topo, solid, from * scale);
            let mut comparison = topo.clone();
            let direct =
                resize_cylindrical_face(&mut comparison, solid, face, next * scale).unwrap();
            solid = resize_cylindrical_face_journaled(&mut topo, solid, face, next * scale)
                .unwrap()
                .solid;
            valid(&topo, solid);
            assert_eq!(solid_entity_counts(&topo, solid).unwrap(), (13, 22, 12));
            assert!(
                (solid_volume(&topo, solid, 0.001 * scale).unwrap()
                    - solid_volume(&comparison, direct, 0.001 * scale).unwrap())
                .abs()
                    < 1e-5 * scale.powi(3)
            );
            let mut found = std::collections::BTreeSet::new();
            for reference in &refs {
                let outcome = resolve(&topo, reference);
                let Resolution::Bound {
                    entity,
                    provenance: Provenance::Construction,
                } = outcome
                else {
                    panic!(
                        "lost construction reference at scale {scale}, r{from} to r{next}: {outcome:?}"
                    )
                };
                assert!(found.insert(entity));
            }
            assert_eq!(
                found,
                solid_entity_keys(&topo, solid)
                    .unwrap()
                    .into_iter()
                    .collect()
            );
        }
        roundtrip(&topo, solid);
    }
}

#[test]
fn imported_bore_floor_rises_past_split_rim_shoulder_fillet() {
    // The import's outer shoulder fillet (R 16, r 4) carries its rims as half
    // circles. Its seam still bounds it to radius 16.8 and up, clear of the
    // radius-12.5 floor, so the floor's sweep must not stop at it.
    for offset in [15.0, 25.0, 40.0] {
        let mut topo = Topology::new();
        let solid = imported(&mut topo);
        let floor = plane(&topo, solid, 9.0, 1.0);
        let before = solid_volume(&topo, solid, 0.01).unwrap();
        let counts = solid_entity_counts(&topo, solid).unwrap();
        let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[floor]).unwrap();
        for result in [
            move_faces(&mut topo, solid, &[floor], offset).unwrap(),
            prepared.move_faces(&mut topo, offset).unwrap().solid,
        ] {
            valid(&topo, result);
            assert_eq!(solid_entity_counts(&topo, result).unwrap(), counts);
            let expected = before + PI * 12.5_f64.powi(2) * offset;
            assert!((solid_volume(&topo, result, 0.01).unwrap() - expected).abs() < 1e-5);
        }
    }
    // The open rim at z = 50 still stops the floor.
    let mut topo = Topology::new();
    let solid = imported(&mut topo);
    let floor = plane(&topo, solid, 9.0, 1.0);
    let error = move_faces(&mut topo, solid, &[floor], 41.0).expect_err("floor reaches the rim");
    assert!(error.to_string().contains("nonadjacent"), "{error}");
}
