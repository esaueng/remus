//! A moving shoulder blend must not pass through the fixed boss rim.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::compact_sort::CompactSort;
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
    shoulder_with(height, cap_height, 1.8, 5.0)
}

/// An R6 boss on a 30x18 block of `height`, its cap rounded by `rim`, then
/// the boss/block perimeter rounded by `radius`. The shoulder wraps only the
/// quarter of the boss inside the block.
fn shoulder_with(
    height: f64,
    cap_height: f64,
    rim: f64,
    radius: f64,
) -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, height).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, cap_height).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let cap = top_at(&topo, blank, cap_height);
    let cap_edges = face_edges(&topo, cap).unwrap();
    let rounded = fillet_cascade(&mut topo, blank, &cap_edges, rim)
        .unwrap()
        .solid;
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
    let solid = fillet_cascade(&mut topo, rounded, &perimeter, radius)
        .unwrap()
        .solid;
    let top = top_at(&topo, solid, height);
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
    // Rectangle [r,W] x [r,D], less the circular bite of the R+r
    // shoulder between its two exact tangent points. This distinguishes
    // the support-first corner from a convex ridge walked over a seam.
    let r = radius;
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

/// Volume of `solid`'s mesh at `deflection`, refusing an open or
/// non-manifold mesh.
fn closed_mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "mesh at {deflection}: {quality:?}");
    let origin = mesh.positions[0];
    mesh.indices
        .chunks_exact(3)
        .map(|ix| {
            let a = mesh.positions[ix[0] as usize] - origin;
            let b = mesh.positions[ix[1] as usize] - origin;
            let c = mesh.positions[ix[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// The verification bar for an accepted move: closed at two deflections no
/// coarser than the volume clamp, API and mesh volumes agreeing, and the
/// moved cap bounding material exactly where it now sits.
fn assert_closed_solid(topo: &Topology, solid: SolidId, cap_height: f64) {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    use remus_math::vec::Point3;
    assert!(validate_solid(topo, solid).unwrap().is_valid());
    let bbox = remus_operations::measure::solid_bounding_box(topo, solid).unwrap();
    let diag = (bbox.max - bbox.min).length();
    let api = solid_volume(topo, solid, diag * 5e-5).unwrap();
    for deflection in [diag * 5e-5, diag * 2.5e-5] {
        let mesh = closed_mesh_volume(topo, solid, deflection);
        assert!(
            (mesh - api).abs() <= api * 1e-4,
            "API volume {api} against mesh volume {mesh} at {deflection}"
        );
    }
    let options = ClassifyOptions::default();
    for (point, expected) in [
        (
            Point3::new(0.0, 0.0, cap_height - 0.5),
            PointClassification::Inside,
        ),
        (
            Point3::new(0.0, 0.0, cap_height + 0.5),
            PointClassification::Outside,
        ),
        (
            Point3::new(-5.5, 0.0, cap_height - 1.5),
            PointClassification::Inside,
        ),
        (Point3::new(20.0, 9.0, 12.0), PointClassification::Inside),
        (
            Point3::new(-5.5, 0.0, cap_height + 0.2),
            PointClassification::Outside,
        ),
    ] {
        assert_eq!(
            classify_point(topo, solid, point, &options).unwrap(),
            expected,
            "{point:?} with the cap at {cap_height}"
        );
    }
}

#[test]
fn cap_cannot_cross_the_fixed_shoulder() {
    // The Gentle-Gecko boss: an R4 shoulder wraps the quarter of the R6 boss
    // inside the 30x18x24 block, meeting the boss on an arc at z=28. The R1
    // cap rim meets the boss on a full circle at z=35, so the cap can come
    // down 7 mm before the rim reaches the shoulder. Past that the rigid
    // translation once returned a body both validators accepted, with an
    // open mesh and a 2.4% API/mesh volume disagreement.
    let (mut topo, solid, _) = shoulder_with(24.0, 36.0, 1.0, 4.0);
    let cap = top_at(&topo, solid, 36.0);
    let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let faces = solid_faces(&topo, solid).unwrap();
    let entries = topo.journal().entries().len();
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[cap]).unwrap();
    let unchanged = |topo: &Topology| {
        assert_eq!(
            remus_io::arena_io::serialize_solid(topo, solid).unwrap(),
            before
        );
        assert_eq!(solid_entity_counts(topo, solid).unwrap(), counts);
        assert_eq!(solid_faces(topo, solid).unwrap(), faces);
        assert_eq!(topo.journal().entries().len(), entries);
    };
    for distance in [-7.0, -7.5, -8.0, -9.0, -10.0, -11.5] {
        let refusal = move_faces(&mut topo, solid, &[cap], distance)
            .expect_err("the moving rim must not cross the fixed shoulder");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{distance}: {refusal}"
        );
        unchanged(&topo);
        let refusal = prepared
            .move_faces(&mut topo, distance)
            .expect_err("prepared moves must enforce the same clearance");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{distance}: {refusal}"
        );
        unchanged(&topo);
    }
    // Shortening short of the shoulder, and lengthening (A1 taller), keep
    // moving the rim rigidly and stay closed.
    let source_volume = solid_volume(&topo, solid, 0.01).unwrap();
    for distance in [-6.0, -3.0, 2.0, 25.2] {
        let moved = move_faces(&mut topo, solid, &[cap], distance)
            .unwrap_or_else(|error| panic!("distance {distance}: {error}"));
        assert_eq!(solid_entity_counts(&topo, moved).unwrap(), counts);
        top_at(&topo, moved, 36.0 + distance);
        assert_closed_solid(&topo, moved, 36.0 + distance);
        let volume = solid_volume(&topo, moved, 0.01).unwrap();
        assert!((volume - source_volume).signum() == distance.signum());
        unchanged(&topo);
        let previewed = prepared
            .move_faces(&mut topo, distance)
            .unwrap_or_else(|error| panic!("prepared distance {distance}: {error}"));
        assert_eq!(
            solid_volume(&topo, previewed.solid, 0.01)
                .unwrap()
                .to_bits(),
            volume.to_bits()
        );
        unchanged(&topo);
    }
}

const SAMPLED: Option<FaceMoveApproximation> = Some(FaceMoveApproximation { stations: 256 });

/// Volume of the body one sampled move of the fresh Gecko cap to
/// `cap_height` builds.
fn fresh_descended_volume(cap_height: f64) -> f64 {
    let (mut topo, solid, _) = shoulder_with(24.0, 36.0, 1.0, 4.0);
    let cap = top_at(&topo, solid, 36.0);
    let moved = move_faces_with_quality(&mut topo, solid, &[cap], cap_height - 36.0, SAMPLED)
        .unwrap_or_else(|error| panic!("fresh cap at {cap_height}: {error}"));
    solid_volume(&topo, moved.result.solid, 0.002).unwrap()
}

/// Faces of `solid` by carrier: planes, cylinders, tori, sampled NURBS.
fn carrier_census(topo: &Topology, solid: SolidId) -> [usize; 4] {
    use remus_topology::face::FaceSurface;
    let mut census = [0; 4];
    for face in solid_faces(topo, solid).unwrap() {
        census[match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => 0,
            FaceSurface::Cylinder(_) => 1,
            FaceSurface::Torus(_) => 2,
            FaceSurface::Nurbs(_) => 3,
            surface => panic!("unexpected {} carrier", surface.type_tag()),
        }] += 1;
    }
    census
}

/// The verification bar for the Gecko cap lowered to `cap_height` into the
/// retained R4 shoulder: the R1 rim re-rolls on the boss and on the
/// shoulder torus as exact tori with exact circular contacts, the cap plane
/// trims the sampled corner patches with sampled bands, and the shoulder is
/// cut back by the cap plane while staying where it was below it.
fn assert_descended_into_shoulder(topo: &Topology, solid: SolidId, cap_height: f64) {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    use remus_math::vec::Point3;
    use remus_topology::face::FaceSurface;
    let level = cap_height - 1.0;
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap().0,
        19,
        "cap at {cap_height}"
    );
    // Planes, cylinders and tori stay analytic; only the two retained corner
    // patches and their two new rim bands are sampled.
    assert_eq!(
        carrier_census(topo, solid),
        [7, 5, 3, 4],
        "cap at {cap_height}"
    );
    let cap = top_at(topo, solid, cap_height);
    // The ball rolling under the cap meets the R4 shoulder tube (ring R10 at
    // z=28) at the R4+R1 offset: its centres circle the axis at this radius.
    let shoulder_ring = 10.0 - (25.0 - (level - 28.0).powi(2)).sqrt();
    let mut rims = Vec::new();
    for face in solid_faces(topo, solid).unwrap() {
        let FaceSurface::Torus(torus) = topo.face(face).unwrap().surface() else {
            continue;
        };
        if (torus.minor_radius() - 1.0).abs() > 1e-9 {
            continue;
        }
        assert!((torus.center().z() - level).abs() < 1e-9);
        assert!(torus.z_axis().z().abs() > 1.0 - 1e-12);
        rims.push(torus.major_radius());
        for edge in face_edges(topo, face).unwrap() {
            assert!(
                matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(_)),
                "exact rim band {} has a sampled boundary",
                face.index()
            );
        }
    }
    rims.compact_sort_by(f64::total_cmp);
    assert_eq!(rims.len(), 2, "cap at {cap_height}: {rims:?}");
    assert!((rims[0] - 5.0).abs() < 1e-9, "{rims:?}");
    assert!((rims[1] - shoulder_ring).abs() < 1e-9, "{rims:?}");
    // The cap outline grows toward the block where it rides the shoulder.
    let reach = face_vertices(topo, cap)
        .unwrap()
        .into_iter()
        .map(|vertex| {
            let p = topo.vertex(vertex).unwrap().point();
            p.x().hypot(p.y())
        })
        .fold(0.0, f64::max);
    assert!(
        (reach - shoulder_ring).abs() < 1e-6,
        "{reach} {shoulder_ring}"
    );
    assert!(face_area(topo, cap, 0.001).unwrap() > 25.0 * std::f64::consts::PI);
    assert_closed_solid(topo, solid, cap_height);
    let options = ClassifyOptions::default();
    let diagonal = Vec3::new(1.0, 1.0, 0.0).normalize().unwrap();
    let up = Vec3::new(0.0, 0.0, 1.0);
    let at = |radial: Vec3, radius: f64, z: f64| Point3::new(0.0, 0.0, z) + radial * radius;
    // The new shoulder band: inside and just outside its rolling ball.
    let centre = at(diagonal, shoulder_ring, level);
    let toward_shoulder = (at(diagonal, 10.0, 28.0) - centre).normalize().unwrap();
    let band = (toward_shoulder + up).normalize().unwrap();
    // The boss band on the far side of the boss.
    let outward = -diagonal;
    let boss_centre = at(outward, 5.0, level);
    let boss_band = (outward + up).normalize().unwrap();
    // R8 on the block diagonal: the shoulder's height there, cut back by the
    // cap plane wherever the cap now lies below it.
    let shoulder_top = (28.0 - 12.0_f64.sqrt()).min(cap_height);
    for (point, expected) in [
        (centre + band * 0.8, PointClassification::Inside),
        (centre + band * 1.2, PointClassification::Outside),
        (boss_centre + boss_band * 0.8, PointClassification::Inside),
        (boss_centre + boss_band * 1.2, PointClassification::Outside),
        (
            at(diagonal, 8.0, cap_height + 0.1),
            PointClassification::Outside,
        ),
        (
            at(diagonal, 8.0, shoulder_top - 0.1),
            PointClassification::Inside,
        ),
        (
            at(diagonal, 6.05, cap_height + 0.1),
            PointClassification::Outside,
        ),
    ] {
        assert_eq!(
            classify_point(topo, solid, point, &options).unwrap(),
            expected,
            "{point:?} with the cap at {cap_height}"
        );
    }
}

#[test]
fn cap_descends_into_the_retained_shoulder_with_sampled_permission() {
    // A1 "shorten" on the Gentle-Gecko boss: the cap with its R1 rim comes
    // down past z=29, where the rim's contact at z=28 reaches the fixed R4
    // shoulder arc and the exact move refuses. With sampled permission the
    // shoulder is kept and trimmed by the cap plane. z=26 is the reference
    // recording's shortened cap; z=24.1 leaves 0.1 mm above the block.
    let (mut topo, solid, _) = shoulder_with(24.0, 36.0, 1.0, 4.0);
    let cap = top_at(&topo, solid, 36.0);
    let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[cap]).unwrap();
    let source_volume = solid_volume(&topo, solid, 0.002).unwrap();
    let mut previous = source_volume;
    for cap_height in [28.99, 28.5, 27.0, 26.0, 25.0, 24.1] {
        let distance = cap_height - 36.0;
        let refusal = move_faces_with_quality(&mut topo, solid, &[cap], distance, None)
            .expect_err("exact-only moves keep refusing to reach the shoulder");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{cap_height}: {refusal}"
        );
        let moved = move_faces_with_quality(&mut topo, solid, &[cap], distance, SAMPLED)
            .unwrap_or_else(|error| panic!("cap at {cap_height}: {error}"));
        assert_eq!(
            moved.quality,
            FaceMoveQuality::SampledCapJoin { stations: 256 }
        );
        assert_descended_into_shoulder(&topo, moved.result.solid, cap_height);
        let volume = solid_volume(&topo, moved.result.solid, 0.002).unwrap();
        assert!(volume < previous, "{cap_height}: {volume} after {previous}");
        previous = volume;
        let preview = prepared
            .move_faces_with_quality(&mut topo, distance, SAMPLED)
            .unwrap_or_else(|error| panic!("preview at {cap_height}: {error}"));
        assert_eq!(preview.quality, moved.quality);
        assert_eq!(
            solid_volume(&topo, preview.result.solid, 0.002)
                .unwrap()
                .to_bits(),
            volume.to_bits()
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
            before
        );
    }
    // Journaled, the same move records its history and leaves the source.
    let faces = solid_faces(&topo, solid).unwrap();
    let (journaled, quality) = remus_operations::journal_ops::move_faces_journaled_with_quality(
        &mut topo,
        solid,
        &[cap],
        -10.0,
        SAMPLED,
    )
    .unwrap();
    assert_eq!(quality, FaceMoveQuality::SampledCapJoin { stations: 256 });
    assert!(journaled.map.unresolved.is_empty());
    assert_descended_into_shoulder(&topo, journaled.solid, 26.0);
    assert_eq!(solid_faces(&topo, solid).unwrap(), faces);
    assert_eq!(
        solid_volume(&topo, solid, 0.002).unwrap().to_bits(),
        source_volume.to_bits()
    );
}

#[test]
fn descending_cap_refuses_unbuildable_depths_atomically() {
    // At z=29 the new contour meets the shoulder's boss arc exactly; within
    // 10 um of it a corner contour cannot span its patch; from z=24 down the
    // cap would meet the block top.
    let (mut topo, solid, _) = shoulder_with(24.0, 36.0, 1.0, 4.0);
    let cap = top_at(&topo, solid, 36.0);
    let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let entries = topo.journal().entries().len();
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[cap]).unwrap();
    for distance in [-7.0, -7.000_01, -12.0, -12.5, -20.0] {
        let direct = move_faces_with_quality(&mut topo, solid, &[cap], distance, SAMPLED)
            .expect_err("an unbuildable depth must refuse");
        let preview = prepared
            .move_faces_with_quality(&mut topo, distance, SAMPLED)
            .expect_err("prepared moves must refuse the same depth");
        let journaled = remus_operations::journal_ops::move_faces_journaled_with_quality(
            &mut topo,
            solid,
            &[cap],
            distance,
            SAMPLED,
        )
        .expect_err("journaled moves must refuse the same depth");
        assert_eq!(preview.to_string(), direct.to_string(), "{distance}");
        assert_eq!(journaled.to_string(), direct.to_string(), "{distance}");
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
            before,
            "{distance}: {direct}"
        );
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
        assert_eq!(topo.journal().entries().len(), entries);
    }
}

#[test]
fn joined_cap_moves_again_and_climbs_back_out_of_the_shoulder() {
    // From the recording's shortened z=26, further moves of the joined cap
    // build the same body as one move from the original: deeper, back up
    // while still joined, and out of the shoulder. At z=29.5 the rim is one
    // full circle above the restored shoulder again, and the recording's
    // lengthened z=61.2 follows.
    let (mut topo, solid, _) = shoulder_with(24.0, 36.0, 1.0, 4.0);
    let cap = top_at(&topo, solid, 36.0);
    let joined = move_faces_with_quality(&mut topo, solid, &[cap], -10.0, SAMPLED)
        .unwrap()
        .result
        .solid;
    let joined_cap = top_at(&topo, joined, 26.0);
    let before = remus_io::arena_io::serialize_solid(&topo, joined).unwrap();
    for (cap_height, faces) in [(25.5, 19), (27.0, 19), (28.5, 19), (29.5, 16), (61.2, 16)] {
        let moved =
            move_faces_with_quality(&mut topo, joined, &[joined_cap], cap_height - 26.0, SAMPLED)
                .unwrap_or_else(|error| panic!("joined cap to {cap_height}: {error}"));
        assert_eq!(
            moved.quality,
            FaceMoveQuality::SampledCapJoin { stations: 256 }
        );
        let result = moved.result.solid;
        assert_eq!(solid_entity_counts(&topo, result).unwrap().0, faces);
        if faces == 19 {
            assert_descended_into_shoulder(&topo, result, cap_height);
        } else {
            assert_eq!(carrier_census(&topo, result), [7, 5, 2, 2]);
            assert_closed_solid(&topo, result, cap_height);
        }
        let volume = solid_volume(&topo, result, 0.002).unwrap();
        let fresh = fresh_descended_volume(cap_height);
        assert!(
            (volume - fresh).abs() <= fresh * 1e-6,
            "{cap_height}: {volume} against a single move's {fresh}"
        );
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, joined).unwrap(),
            before
        );
    }
    // The joined cap cannot reach the block top either.
    for cap_height in [24.0, 23.5] {
        move_faces_with_quality(&mut topo, joined, &[joined_cap], cap_height - 26.0, SAMPLED)
            .expect_err("the joined cap must not reach the block top");
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, joined).unwrap(),
            before
        );
    }
}
