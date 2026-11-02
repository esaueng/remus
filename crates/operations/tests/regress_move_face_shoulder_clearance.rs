//! A moving shoulder blend must not pass through the fixed boss rim.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::face_move_quality::{
    FaceMoveApproximation, FaceMoveQuality, move_faces_with_quality,
};
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{
    face_edges, face_vertices, solid_edges, solid_entity_counts, solid_faces,
};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn top_at(topo: &Topology, solid: SolidId, height: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .expect("planar top at requested height")
}

#[test]
fn cap_permission_preserves_unrelated_exact_refusals_and_rollback() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let face = top_at(&topo, solid, 10.0);
    let expected = move_faces(&mut topo, solid, &[face], -11.0)
        .unwrap_err()
        .to_string();
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let faces = solid_faces(&topo, solid).unwrap();
    let entries = topo.journal().entries().len();
    let permission = Some(FaceMoveApproximation { stations: 256 });
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[face]).unwrap();
    let ordinary =
        move_faces_with_quality(&mut topo, solid, &[face], -11.0, permission).unwrap_err();
    let journaled = remus_operations::journal_ops::move_faces_journaled_with_quality(
        &mut topo,
        solid,
        &[face],
        -11.0,
        permission,
    )
    .unwrap_err();
    let preview = prepared
        .move_faces_with_quality(&mut topo, -11.0, permission)
        .unwrap_err();
    let journaled_preview = prepared
        .move_faces_journaled_with_quality(&mut topo, -11.0, permission)
        .unwrap_err();
    for error in [ordinary, journaled, preview, journaled_preview] {
        assert_eq!(error.to_string(), expected);
    }
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
    assert_eq!(solid_faces(&topo, solid).unwrap(), faces);
    assert_eq!(topo.journal().entries().len(), entries);
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
}

fn shoulder() -> (Topology, SolidId, FaceId) {
    shoulder_at(26.0, 35.0)
}

fn shoulder_at(height: f64, cap_height: f64) -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, height).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, cap_height).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let cap = top_at(&topo, blank, cap_height);
    let rim = face_edges(&topo, cap).unwrap();
    let rounded = fillet_cascade(&mut topo, blank, &rim, 1.8).unwrap().solid;
    let perimeter: Vec<_> = solid_edges(&topo, rounded)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let at = |a: f64, b: f64| (a - b).abs() < 1e-7;
            let top = at(a.z(), height) && at(b.z(), height);
            match edge.curve() {
                EdgeCurve::Circle(_) => top,
                EdgeCurve::Line => {
                    (top && ((at(a.x(), 0.0) && at(b.x(), 0.0))
                        || (at(a.y(), 0.0) && at(b.y(), 0.0))))
                        || ((a.z() - b.z()).abs() > height - 0.01
                            && ((at(a.x(), 6.0)
                                && at(b.x(), 6.0)
                                && at(a.y(), 0.0)
                                && at(b.y(), 0.0))
                                || (at(a.y(), 6.0)
                                    && at(b.y(), 6.0)
                                    && at(a.x(), 0.0)
                                    && at(b.x(), 0.0))))
                }
                _ => false,
            }
        })
        .collect();
    assert_eq!(perimeter.len(), 5);
    let solid = fillet_cascade(&mut topo, rounded, &perimeter, 5.0)
        .unwrap()
        .solid;
    let top = top_at(&topo, solid, height);
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
    // Rectangle [r,W] x [r,D], less the circular bite of the R+r
    // shoulder between its two exact tangent points. This distinguishes
    // the support-first corner from a convex ridge walked over a seam.
    let r = 5.0_f64;
    let rail = 6.0_f64 + r;
    let tangent = (rail * rail - r * r).sqrt();
    let bite = 0.5 * rail * rail * (std::f64::consts::FRAC_PI_2 - 2.0 * (r / rail).asin())
        - r * (tangent - r);
    let expected = (30.0 - r) * (18.0 - r) - bite;
    assert!((face_area(&topo, top, 0.01).unwrap() - expected).abs() < 1e-5);
    (topo, solid, top)
}

#[test]
fn equal_radius_corner_keeps_the_boss_as_its_rolling_rail() {
    use remus_topology::face::FaceSurface;

    let (topo, solid, _) = shoulder_at(19.1, 28.5);
    let faces = solid_faces(&topo, solid).unwrap();
    assert_eq!(faces.len(), 16);
    assert_eq!(
        faces
            .iter()
            .filter(|&&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .count(),
        5
    );
    let tangent = 96.0_f64.sqrt();
    for &face in &faces {
        let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface() else {
            continue;
        };
        if (cylinder.radius() - 5.0).abs() < 1e-7 && cylinder.axis().z().abs() > 0.99 {
            assert!(
                face_vertices(&topo, face)
                    .unwrap()
                    .iter()
                    .all(|&vertex| topo.vertex(vertex).unwrap().point().z() <= 14.1 + 1e-7)
            );
        }
        if (cylinder.radius() - 5.0).abs() < 1e-7 && cylinder.axis().z().abs() < 1e-7 {
            assert!(face_vertices(&topo, face).unwrap().iter().any(|&vertex| {
                let p = topo.vertex(vertex).unwrap().point();
                (p.x() - tangent).abs() < 1e-7 || (p.y() - tangent).abs() < 1e-7
            }));
        }
    }
}

#[test]
fn shoulder_reconstructs_a_tangent_transition_into_the_lowered_rim() {
    use remus_topology::face::FaceSurface;

    let (mut topo, solid, top) = shoulder_at(19.1, 28.5);
    let source_faces = solid_faces(&topo, solid).unwrap();
    let source_volume = solid_volume(&topo, solid, 0.01).unwrap();
    let counts = solid_entity_counts(&topo, solid).unwrap();
    move_faces_with_quality(&mut topo, solid, &[top], 5.0, None)
        .expect_err("exact-only calls must refuse sampled corner reconstruction");
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
    let moved = move_faces_with_quality(
        &mut topo,
        solid,
        &[top],
        5.0,
        Some(FaceMoveApproximation { stations: 256 }),
    )
    .expect("the shoulder must reconstruct its valid junction with the rim");
    assert_eq!(
        moved.quality,
        FaceMoveQuality::SampledCapJoin { stations: 256 }
    );
    let moved = moved.result.solid;
    assert!(validate_solid(&topo, moved).unwrap().is_valid());
    top_at(&topo, moved, 24.1);
    top_at(&topo, moved, 28.5);
    assert!(
        remus_operations::measure::solid_bounding_box(&topo, moved)
            .unwrap()
            .min
            .z()
            .abs()
            < 1e-7
    );
    // The new R1.8 torus is tangent to the R5 shoulder and to the fixed
    // cap plane. In their axial section their circle centres are R5+R1.8
    // apart, with an axial separation of (24.1+5)-(28.5-1.8) = 2.4 mm.
    let ring = 11.0_f64 - (6.8_f64.powi(2) - 2.4_f64.powi(2)).sqrt();
    assert!(solid_faces(&topo, moved).unwrap().iter().any(|&face| {
        let FaceSurface::Torus(torus) = topo.face(face).unwrap().surface() else {
            return false;
        };
        (torus.major_radius() - ring).abs() < 1e-6 && (torus.minor_radius() - 1.8).abs() < 1e-7
    }));
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[top]).unwrap();
    let permission = Some(FaceMoveApproximation { stations: 256 });
    let (journaled, quality) = remus_operations::journal_ops::move_faces_journaled_with_quality(
        &mut topo,
        solid,
        &[top],
        5.0,
        permission,
    )
    .expect("journaled reconstruction must share the positive geometry path");
    assert_eq!(quality, FaceMoveQuality::SampledCapJoin { stations: 256 });
    let (preview, preview_quality) = prepared
        .move_faces_journaled_with_quality(&mut topo, 5.0, permission)
        .expect("prepared reconstruction must share the positive geometry path");
    assert_eq!(preview_quality, quality);
    for result in [&journaled, &preview] {
        assert_eq!(
            solid_entity_counts(&topo, result.solid).unwrap(),
            solid_entity_counts(&topo, moved).unwrap()
        );
        assert!(result.map.unresolved.is_empty());
        top_at(&topo, result.solid, 24.1);
        top_at(&topo, result.solid, 28.5);
        let output_keys =
            remus_operations::journal_ops::solid_entity_keys(&topo, result.solid).unwrap();
        let entry = topo
            .journal()
            .entries()
            .iter()
            .find(|entry| entry.op() == result.op)
            .unwrap();
        let remus_topology::journal::EntryPayload::Evolution { scope, events, .. } =
            entry.payload()
        else {
            panic!("face move must publish evolution");
        };
        assert!(output_keys.iter().all(|key| {
            topo.journal()
                .ordinal_of(*key)
                .is_some_and(|ordinal| scope.contains(&ordinal))
        }));
        assert!(events.len() >= output_keys.len());
    }
    assert_eq!(solid_faces(&topo, solid).unwrap(), source_faces);
    assert_eq!(
        solid_volume(&topo, solid, 0.01).unwrap().to_bits(),
        source_volume.to_bits()
    );
}

#[test]
fn shoulder_cannot_cross_the_fixed_cap_blend() {
    // The shoulder's R5 spring reaches z=31. The fixed R1.8 boss rim
    // starts at z=33.2, leaving 2.2 mm of travel before they meet.
    let (mut topo, solid, top) = shoulder();
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let faces = solid_faces(&topo, solid).unwrap();
    let volume = solid_volume(&topo, solid, 0.01).unwrap();
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[top]).unwrap();
    for distance in [2.3, 3.3, 5.0] {
        let refusal = move_faces(&mut topo, solid, &[top], distance)
            .expect_err("the moving blend must not cross the fixed rim");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{refusal}"
        );
        let refusal = prepared
            .move_faces(&mut topo, distance)
            .expect_err("prepared moves must enforce the same clearance");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{refusal}"
        );
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
        assert_eq!(solid_faces(&topo, solid).unwrap(), faces);
        assert_eq!(
            solid_volume(&topo, solid, 0.01).unwrap().to_bits(),
            volume.to_bits()
        );
    }
    for distance in [-7.0, -1.51, 1.0, 2.0] {
        let moved = move_faces(&mut topo, solid, &[top], distance)
            .unwrap_or_else(|error| panic!("distance {distance}: {error}"));
        assert!(validate_solid(&topo, moved).unwrap().is_valid());
        let moved_top = top_at(&topo, moved, 26.0 + distance);
        assert!(
            (face_area(&topo, moved_top, 0.01).unwrap() - face_area(&topo, top, 0.01).unwrap())
                .abs()
                < 1e-5
        );
        assert_eq!(solid_entity_counts(&topo, moved).unwrap(), counts);
        top_at(&topo, moved, 35.0);
    }
}

#[test]
fn joined_shoulder_accepts_further_moves_and_reverses_the_topology_transition() {
    let (mut topo, source, top) = shoulder_at(19.1, 28.5);
    let permission = Some(FaceMoveApproximation { stations: 256 });
    let first = move_faces_with_quality(&mut topo, source, &[top], 5.0, permission).unwrap();
    let mut solid = first.result.solid;
    let mut height = 24.1;
    for distance in [0.5, -1.5, -3.1, 4.1] {
        let face = top_at(&topo, solid, height);
        let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
        let moved = move_faces_with_quality(&mut topo, solid, &[face], distance, permission)
            .unwrap_or_else(|error| panic!("joined shoulder {height} + {distance}: {error}"));
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
            before
        );
        height += distance;
        solid = moved.result.solid;
        assert!(validate_solid(&topo, solid).unwrap().is_valid());
        top_at(&topo, solid, height);
        top_at(&topo, solid, 28.5);
        assert_eq!(
            solid_faces(&topo, solid).unwrap().len(),
            if height > 21.7 { 19 } else { 16 }
        );
    }
}

#[test]
fn shoulder_keeps_the_periodic_boss_boundary_when_contact_begins() {
    let (mut topo, source, top) = shoulder_at(19.1, 28.5);
    remus_operations::measure::enable_thread_volume_memo();
    let permission = Some(FaceMoveApproximation { stations: 256 });
    let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
    for (distance, face_count) in [
        (2.6, 16),
        (2.61, 19),
        (2.75, 19),
        (3.5, 19),
        (4.0, 19),
        (5.0, 19),
    ] {
        let result = move_faces_with_quality(&mut topo, source, &[top], distance, permission)
            .unwrap_or_else(|error| panic!("onset offset {distance}: {error}"));
        assert_eq!(
            solid_faces(&topo, result.result.solid).unwrap().len(),
            face_count
        );
        assert!(
            validate_solid(&topo, result.result.solid)
                .unwrap()
                .is_valid()
        );
        top_at(&topo, result.result.solid, 19.1 + distance);
        top_at(&topo, result.result.solid, 28.5);
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
            before
        );
    }
}

#[test]
fn touching_shoulder_can_move_in_either_direction() {
    let (mut topo, source, top) = shoulder_at(19.1, 28.5);
    remus_operations::measure::enable_thread_volume_memo();
    let permission = Some(FaceMoveApproximation { stations: 256 });
    let touching = move_faces_with_quality(&mut topo, source, &[top], 2.6, permission)
        .unwrap()
        .result
        .solid;
    let before = remus_io::arena_io::serialize_solid(&topo, touching).unwrap();
    let top = top_at(&topo, touching, 21.7);
    for (distance, face_count) in [(0.1, 19), (-0.1, 16), (2.4, 19)] {
        let result = move_faces_with_quality(&mut topo, touching, &[top], distance, permission)
            .unwrap_or_else(|error| panic!("move from touching by {distance}: {error}"));
        assert_eq!(
            solid_faces(&topo, result.result.solid).unwrap().len(),
            face_count
        );
        assert!(
            validate_solid(&topo, result.result.solid)
                .unwrap()
                .is_valid()
        );
        top_at(&topo, result.result.solid, 21.7 + distance);
        top_at(&topo, result.result.solid, 28.5);
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, touching).unwrap(),
            before
        );
    }
}

#[test]
fn cap_join_respects_rigid_placement_and_uniform_scale() {
    use remus_math::mat::Mat4;
    use remus_math::vec::Point3;
    let (source_topo, source, _) = shoulder_at(19.1, 28.5);
    let bytes = remus_io::arena_io::serialize_solid(&source_topo, source).unwrap();
    for (scale, angle) in [(0.25, 0.71), (1.0, std::f64::consts::PI), (4.0, -0.41)] {
        let mut topo = Topology::new();
        let source = remus_io::arena_io::deserialize_solid(&bytes, &mut topo).unwrap();
        let selected = top_at(&topo, source, 19.1);
        let matrix = Mat4::translation(137.0, -41.0, 73.0)
            * Mat4::rotation_y(-0.36)
            * Mat4::rotation_x(angle)
            * Mat4::scale(scale, scale, scale);
        remus_operations::transform::transform_solid(&mut topo, source, &matrix).unwrap();
        let normal = topo
            .face(selected)
            .unwrap()
            .effective_plane_normal()
            .unwrap();
        let before = remus_io::arena_io::serialize_solid(&topo, source).unwrap();
        for (distance, count) in [(2.6, 16), (5.0, 19)] {
            let moved = move_faces_with_quality(
                &mut topo,
                source,
                &[selected],
                distance * scale,
                Some(FaceMoveApproximation { stations: 256 }),
            )
            .unwrap_or_else(|error| {
                panic!("scale {scale}, angle {angle}, offset {distance}: {error}")
            });
            assert!(
                validate_solid(&topo, moved.result.solid)
                    .unwrap()
                    .is_valid()
            );
            let faces = solid_faces(&topo, moved.result.solid).unwrap();
            assert_eq!(faces.len(), count);
            for height in [19.1 + distance, 28.5] {
                let expected = matrix.mul_point(Point3::new(18.0, 12.0, height));
                assert!(faces.iter().any(|&face| {
                    topo.face(face)
                        .unwrap()
                        .effective_plane_normal()
                        .is_some_and(|n| n.dot(normal) > 1.0 - 1e-10)
                        && face_vertices(&topo, face).unwrap().into_iter().all(|v| {
                            (topo.vertex(v).unwrap().point() - expected)
                                .dot(normal)
                                .abs()
                                < 1e-7
                        })
                }));
            }
            assert_eq!(
                remus_io::arena_io::serialize_solid(&topo, source).unwrap(),
                before
            );
        }
    }
}
