//! A fillet through a blend group finds its picked edge on the sharp body.
//!
//! The group route (`blend_cluster::fillet_through_blend_group`) removes the
//! group a fillet's tangent chain runs into and looks the picked edges up in
//! the removal's `boundary_history` by binary search. That history is
//! documented as sorted by source key, but it was built as every edge, then
//! every vertex, while keys order vertices before edges: the search missed,
//! and the route gave up on every pick.
//!
//! The video's A4 shows the case: on the filleted Gentle-Gecko with A3's R1
//! round, the free-end top edge's tangent chain runs over the R1 band's end
//! arc. A ball of R1.5 or R3.7 cannot roll on an R1 convex band, so the
//! engines refuse, and only the group route (the R1 band re-filleted after
//! the new round) builds it. Before the fix both radii refused with
//! `walking blend cannot follow the ridgeline across the tangent seam`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_cluster::{recognize_blend_cluster, remove_blend_cluster};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, face_vertices, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
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

/// The filleted Gentle-Gecko with its R1 cap rim and A3's R1 round on the
/// x = 0 / y = 18 vertical edge (its chain runs along the y = 18 top edge to
/// the free end).
fn after_a3() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(
        include_str!("data/gentle_gecko_filleted.step"),
        &mut topo,
    )
    .unwrap()[0];
    let cap = top_at(&topo, solid, 36.0);
    let rim = face_edges(&topo, cap).unwrap();
    let solid = fillet_cascade(&mut topo, solid, &rim, 1.0).unwrap().solid;
    let pick = line_where(&topo, solid, |p| {
        p.x().abs() < 1e-7 && (p.y() - 18.0).abs() < 1e-7
    });
    let solid = fillet_cascade(&mut topo, solid, &[pick], 1.0)
        .unwrap()
        .solid;
    (topo, solid)
}

/// The free end's top edge.
fn free_end_top(topo: &Topology, solid: SolidId) -> EdgeId {
    line_where(topo, solid, |p| {
        (p.x() - 30.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7
    })
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
fn group_removal_history_is_sorted_by_source() {
    let (topo, solid) = after_a3();
    let pick = free_end_top(&topo, solid);
    let adjacency = topo.build_adjacency(solid).unwrap();
    let band = adjacency
        .faces_for_edge(pick)
        .iter()
        .flat_map(|&face| face_edges(&topo, face).unwrap())
        .flat_map(|edge| adjacency.faces_for_edge(edge).to_vec())
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .expect("a band beside the free end");
    let cluster = recognize_blend_cluster(&topo, solid, band).unwrap();
    let mut work = topo.clone();
    let restoration = remove_blend_cluster(&mut work, solid, &cluster).unwrap();
    let history = &restoration.boundary_history;
    assert!(
        history.windows(2).all(|pair| pair[0].0 < pair[1].0),
        "boundary history is not sorted by source"
    );
    // Every source edge is found, the picked one with a surviving image.
    for edge in solid_edges(&topo, solid).unwrap() {
        let key = remus_topology::journal::EntityKey::edge(edge.index());
        let at = history
            .binary_search_by(|(source, _)| source.cmp(&key))
            .unwrap_or_else(|_| panic!("edge {} missing from the history", edge.index()));
        if edge == pick {
            assert!(history[at].1.is_some(), "the free-end top edge survives");
        }
    }
}

#[test]
fn free_end_fillet_larger_than_the_band_it_runs_over_builds_through_the_group() {
    for radius in [1.5, 3.7] {
        let label = format!("R{radius}");
        let (mut topo, solid) = after_a3();
        let before = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
        let pick = free_end_top(&topo, solid);
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
        // The round takes the edge's corner and leaves the ball's side.
        let inside = 30.0 - 0.4 * radius;
        for (point, expected) in [
            (Point3::new(29.8, 9.0, 23.8), PointClassification::Outside),
            (
                Point3::new(inside, 9.0, 24.0 - 0.4 * radius),
                PointClassification::Inside,
            ),
            (Point3::new(15.0, 9.0, 12.0), PointClassification::Inside),
        ] {
            assert_eq!(
                classify_point(&topo, result, point, &ClassifyOptions::default()).unwrap(),
                expected,
                "{label}: {point:?}"
            );
        }
        // Planes, cylinders and tori stay analytic: the one new sampled face
        // is the corner where the re-filleted R1 band meets the new round.
        let sampled = |solid: SolidId| {
            solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
                .count()
        };
        assert_eq!(sampled(result), sampled(solid) + 1, "{label}");
    }
}
