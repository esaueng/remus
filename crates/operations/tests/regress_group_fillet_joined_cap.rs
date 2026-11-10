//! A fillet through a blend group on a body whose cap was lowered into its
//! shoulder.
//!
//! A cap moved down past its shoulder with sampled permission leaves rim
//! bands rolling on sampled corner patches, and no blend group is
//! recognizable on such a body as it stands. The band and cylinder group
//! routes edit it with the cap lifted clear on their private copy and lower
//! the cap back afterwards (`blend_cluster/joined_cap.rs`); the fillet route
//! (`fillet_through_blend_group`) did not, so a fillet that only that route
//! builds refused there.
//!
//! The video's A4 after A1: the free-end top edge's tangent chain runs over
//! A3's R1 band, too small for an R1.5 or R3.7 ball to roll on. Both radii
//! refused with `walking blend cannot follow the ridgeline across the tangent
//! seam`; they now build with the cap lifted, the R1 band re-filleted after
//! the new round and the cap lowered back with the permission it came down
//! with.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::face_move_quality::{
    FaceMoveApproximation, FaceMoveQuality, move_faces_with_quality,
};
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, face_vertices, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const SAMPLED: Option<FaceMoveApproximation> = Some(FaceMoveApproximation { stations: 256 });

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

/// The single straight edge with both ends satisfying `on`.
fn line_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            on(topo.vertex(data.start()).unwrap().point())
                && on(topo.vertex(data.end()).unwrap().point())
        })
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

fn sampled(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count()
}

/// The filleted Gentle-Gecko with its R1 cap rim, the cap lengthened to z61.2
/// and shortened to z26 into its shoulder (A1), then A3's R1 round on the
/// x = 0 / y = 18 vertical edge.
fn after_a1_a3() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(
        include_str!("data/gentle_gecko_filleted.step"),
        &mut topo,
    )
    .unwrap()[0];
    let cap = top_at(&topo, solid, 36.0);
    let rim = face_edges(&topo, cap).unwrap();
    let solid = fillet_cascade(&mut topo, solid, &rim, 1.0).unwrap().solid;
    let cap = top_at(&topo, solid, 36.0);
    let long = move_faces_with_quality(&mut topo, solid, &[cap], 25.2, SAMPLED).unwrap();
    let cap = top_at(&topo, long.result.solid, 61.2);
    let short =
        move_faces_with_quality(&mut topo, long.result.solid, &[cap], -35.2, SAMPLED).unwrap();
    assert_eq!(
        short.quality,
        FaceMoveQuality::SampledCapJoin { stations: 256 }
    );
    let solid = short.result.solid;
    let pick = line_where(&topo, solid, |p| {
        p.x().abs() < 1e-7 && (p.y() - 18.0).abs() < 1e-7
    });
    let solid = fillet_cascade(&mut topo, solid, &[pick], 1.0)
        .unwrap()
        .solid;
    (topo, solid)
}

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

#[test]
fn free_end_fillet_through_the_group_of_a_lowered_cap() {
    let (mut topo, solid) = after_a1_a3();
    let source_sampled = sampled(&topo, solid);
    let before = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
    for radius in [1.5, 3.7] {
        let label = format!("R{radius}");
        let pick = line_where(&topo, solid, |p| {
            (p.x() - 30.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7
        });
        let result = fillet_cascade(&mut topo, solid, &[pick], radius)
            .unwrap_or_else(|error| panic!("{label}: {error}"))
            .solid;
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[solid]).unwrap(),
            before,
            "{label}: source"
        );
        let report = remus_operations::validate::validate_solid(&topo, result).unwrap();
        assert!(report.is_valid(), "{label}: {:?}", report.issues);
        let strict = remus_check::validate::validate_solid(
            &topo,
            result,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
        let volume = solid_volume(&topo, result, 5e-4).unwrap();
        for deflection in [2.5e-3, 1e-3] {
            let mesh = tessellate_solid(&topo, result, deflection).unwrap();
            assert_eq!(boundary_edge_count(&mesh), 0, "{label} at {deflection}");
            assert_eq!(non_manifold_edge_count(&mesh), 0, "{label} at {deflection}");
            let independent = mesh_volume(&mesh);
            assert!(
                (independent - volume).abs() <= 1e-4 * volume,
                "{label}: mesh volume {independent} vs {volume} at {deflection}"
            );
        }
        let back = Vec3::new(-1.0, -1.0, 0.0).normalize().unwrap();
        let inside = 30.0 - 0.4 * radius;
        for (point, expected) in [
            // The round takes the edge's corner and leaves the ball's side.
            (Point3::new(29.8, 9.0, 23.8), PointClassification::Outside),
            (
                Point3::new(inside, 9.0, 24.0 - 0.4 * radius),
                PointClassification::Inside,
            ),
            // The cap is back at z26 with its rim.
            (
                Point3::new(0.0, 0.0, 25.9) + back * 4.0,
                PointClassification::Inside,
            ),
            (
                Point3::new(0.0, 0.0, 26.1) + back * 4.0,
                PointClassification::Outside,
            ),
            (
                Point3::new(0.0, 0.0, 25.9) + back * 5.9,
                PointClassification::Outside,
            ),
        ] {
            assert_eq!(
                classify_point(&topo, result, point, &ClassifyOptions::default()).unwrap(),
                expected,
                "{label}: {point:?}"
            );
        }
        // Planes, cylinders and tori stay analytic, and so do the two corners
        // that run over an end arc: one a ring torus, the other a spindle
        // torus (the re-filleted R1 band over the R1.5 round's end arc, ring
        // 0.5; the R3.7 round over the R4 band's, ring 0.3). The spindle one
        // was the sampled face these fillets added until the rolling-ball arc
        // stripe stopped lofting non-ring tori.
        let tori = |solid: SolidId| {
            solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Torus(_)))
                .count()
        };
        assert_eq!(sampled(&topo, result), source_sampled, "{label}");
        assert_eq!(tori(result), tori(solid) + 2, "{label}");
    }
}
