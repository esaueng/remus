//! Phase 4.2 regression for blend-aware planar moves on the Hammer Holder.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::collections::HashMap;

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_io::step::reader::read_step;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_topology::Topology;
use remus_topology::explorer::{edge_to_face_map, face_vertices, solid_entity_counts, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("data/shapr3d_hammer_holder.step");
const ARM_OUTER_X: f64 = 48.0;
const ARM_BLEND_RADIUS: f64 = 3.0;

fn import_hammer_holder() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(HAMMER_HOLDER, &mut topo).expect("import Hammer Holder");
    assert_eq!(solids.len(), 1, "fixture contains one solid");
    (topo, solids[0])
}

fn face_bounds(topo: &Topology, face: FaceId) -> (Point3, Point3) {
    let points: Vec<Point3> = face_vertices(topo, face)
        .expect("face vertices")
        .into_iter()
        .map(|vertex| topo.vertex(vertex).expect("vertex").point())
        .collect();
    let min = Point3::new(
        points
            .iter()
            .map(|point| point.x())
            .fold(f64::INFINITY, f64::min),
        points
            .iter()
            .map(|point| point.y())
            .fold(f64::INFINITY, f64::min),
        points
            .iter()
            .map(|point| point.z())
            .fold(f64::INFINITY, f64::min),
    );
    let max = Point3::new(
        points
            .iter()
            .map(|point| point.x())
            .fold(f64::NEG_INFINITY, f64::max),
        points
            .iter()
            .map(|point| point.y())
            .fold(f64::NEG_INFINITY, f64::max),
        points
            .iter()
            .map(|point| point.z())
            .fold(f64::NEG_INFINITY, f64::max),
    );
    (min, max)
}

fn right_arm_outer_face(topo: &Topology, solid: SolidId, expected_x: f64) -> FaceId {
    let matches: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|face| {
            let face_data = topo.face(*face).expect("face");
            let (min, max) = face_bounds(topo, *face);
            face_data
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(1.0, 0.0, 0.0)) > 1.0 - 1e-9)
                && (min.x() - expected_x).abs() <= Tolerance::new().linear
                && (max.x() - expected_x).abs() <= Tolerance::new().linear
                && min.y() < 10.0
                && max.y() > 45.0
                && max.z() > 59.0
        })
        .collect();
    assert_eq!(
        matches.len(),
        1,
        "one right-arm outer face at x={expected_x}"
    );
    matches[0]
}

fn adjacent_arm_blends(topo: &Topology, solid: SolidId, support: FaceId) -> Vec<FaceId> {
    let adjacency = edge_to_face_map(topo, solid).expect("face adjacency");
    let mut matches = Vec::new();
    for adjacent in adjacency.values().filter(|uses| uses.contains(&support)) {
        for &candidate in adjacent.iter().filter(|&&face| face != support) {
            let is_arm_blend = match topo.face(candidate).expect("candidate").surface() {
                FaceSurface::Cylinder(surface) => {
                    Tolerance::new().approx_eq(surface.radius(), ARM_BLEND_RADIUS)
                }
                FaceSurface::Torus(surface) => {
                    Tolerance::new().approx_eq(surface.minor_radius(), ARM_BLEND_RADIUS)
                }
                FaceSurface::Sphere(surface) => {
                    Tolerance::new().approx_eq(surface.radius(), ARM_BLEND_RADIUS)
                }
                _ => false,
            };
            if is_arm_blend {
                matches.push(candidate);
            }
        }
    }
    matches.sort_unstable_by_key(|face| face.index());
    matches.dedup();
    assert_eq!(
        matches.len(),
        8,
        "arm face keeps its eight r=3 blend patches"
    );
    assert_eq!(
        matches
            .iter()
            .filter(|face| matches!(
                topo.face(**face).expect("blend").surface(),
                FaceSurface::Torus(_)
            ))
            .count(),
        2,
        "two toroidal corner patches bound the outer arm face"
    );
    matches
}

fn blend_radius_census(topo: &Topology, solid: SolidId) -> (usize, usize) {
    solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .fold((0, 0), |(cylinders, tori), face| {
            match topo.face(face).expect("face").surface() {
                FaceSurface::Cylinder(surface)
                    if Tolerance::new().approx_eq(surface.radius(), ARM_BLEND_RADIUS) =>
                {
                    (cylinders + 1, tori)
                }
                FaceSurface::Torus(surface)
                    if Tolerance::new().approx_eq(surface.minor_radius(), ARM_BLEND_RADIUS) =>
                {
                    (cylinders, tori + 1)
                }
                _ => (cylinders, tori),
            }
        })
}

fn surface_census(topo: &Topology, solid: SolidId) -> HashMap<&'static str, usize> {
    let mut census = HashMap::new();
    for face in solid_faces(topo, solid).expect("solid faces") {
        *census
            .entry(topo.face(face).expect("face").surface().type_tag())
            .or_insert(0) += 1;
    }
    census
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
            let key = if first < second {
                (first, second)
            } else {
                (second, first)
            };
            *uses.entry(key).or_insert(0_usize) += 1;
        }
    }
    (
        uses.values().filter(|&&count| count == 1).count(),
        uses.values().filter(|&&count| count > 2).count(),
    )
}

fn assert_strict_and_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).expect("strict validate");
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
    let mesh =
        tessellate_solid_with_tolerance(topo, solid, 0.05, 0.1).expect("tessellate Hammer Holder");
    assert!(is_watertight(&mesh), "index mesh must be watertight");
    let (boundary, non_manifold) = positional_edge_health(&mesh.positions, &mesh.indices);
    assert_eq!(boundary, 0, "position-welded mesh has boundary edges");
    assert_eq!(
        non_manifold, 0,
        "position-welded mesh has non-manifold edges"
    );
}

#[test]
fn hammer_holder_arm_moves_plus_and_minus_five_with_blend_intact() {
    let mut results = Vec::new();
    for distance in [5.0, -5.0] {
        let (mut topo, source) = import_hammer_holder();
        let source_counts = solid_entity_counts(&topo, source).expect("source counts");
        let source_census = surface_census(&topo, source);
        assert_eq!(source_census.get("cylinder"), Some(&42));
        assert_eq!(source_census.get("torus"), Some(&14));
        assert_eq!(blend_radius_census(&topo, source), (32, 12));
        let source_volume = solid_volume(&topo, source, 0.01).expect("source volume");
        let arm = right_arm_outer_face(&topo, source, ARM_OUTER_X);
        let source_blends = adjacent_arm_blends(&topo, source, arm);
        let result = move_faces(&mut topo, source, &[arm], distance).expect("move blended arm");

        assert_eq!(
            solid_entity_counts(&topo, result).expect("result counts"),
            source_counts,
            "blend-aware move must preserve topology cardinality"
        );
        assert_eq!(surface_census(&topo, result), source_census);
        assert_eq!(blend_radius_census(&topo, result), (32, 12));
        let moved_arm = right_arm_outer_face(&topo, result, ARM_OUTER_X + distance);
        let moved_blends = adjacent_arm_blends(&topo, result, moved_arm);
        assert_eq!(moved_blends.len(), source_blends.len());
        assert_strict_and_watertight(&topo, result);

        let result_volume = solid_volume(&topo, result, 0.01).expect("result volume");
        assert_eq!(
            (result_volume - source_volume).is_sign_positive(),
            distance.is_sign_positive(),
            "the signed arm move must change volume in the same direction"
        );
        results.push((source_volume, result_volume));
    }

    let source_volume = results[0].0;
    let symmetric = results[0].1 + results[1].1;
    assert!(
        (symmetric - 2.0 * source_volume).abs() <= source_volume * 2e-3,
        "equal outward/inward arm moves must have opposite volume deltas"
    );
}
