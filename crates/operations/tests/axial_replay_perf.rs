//! Public filleted-union history replay workload; setup is outside the timer.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stdout)]

use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::face_move_quality::FaceMoveApproximation;
use remus_operations::journal_ops::move_faces_journaled_with_quality;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{face_vertices, solid_edges, solid_faces};
use remus_topology::{FaceId, SolidId};

// A linear edit from the history fixture, not an angular constant.
#[allow(clippy::approx_constant)]
const INITIAL_OFFSET_MM: f64 = -6.28;

fn top(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(Vec3::new(0.0, 0.0, 1.0)) > 0.99)
                && face_vertices(topo, face)
                    .unwrap()
                    .iter()
                    .all(|&v| topo.vertex(v).unwrap().point().x() > 4.9)
        })
        .expect("box top")
}

fn fixture() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 32.75, 18.0, 26.0).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, 35.0).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let at = |a: f64, b: f64| (a - b).abs() < 1e-7;
    let perimeter: Vec<_> = solid_edges(&topo, blank)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let on_top = at(a.z(), 26.0) && at(b.z(), 26.0);
            match edge.curve() {
                EdgeCurve::Circle(_) => on_top,
                EdgeCurve::Line => {
                    (on_top
                        && ((at(a.x(), 0.0) && at(b.x(), 0.0))
                            || (at(a.y(), 0.0) && at(b.y(), 0.0))))
                        || ((a.z() - b.z()).abs() > 25.99
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
    let solid = fillet_cascade(&mut topo, blank, &perimeter, 5.0)
        .unwrap()
        .solid;
    (topo, solid)
}

#[test]
fn axial_replay_preserves_exact_geometry_and_history() {
    use remus_operations::face_move_quality::FaceMoveQuality;
    use remus_operations::push_pull::move_faces_with_evolution;
    let (mut topo, mut solid) = fixture();
    remus_operations::performance::configure(Some(|| 0.0));
    for index in 0..21 {
        let distance = if index == 0 {
            INITIAL_OFFSET_MM
        } else if index % 2 == 1 {
            0.4
        } else {
            -0.3
        };
        let face = top(&topo, solid);
        let before = remus_io::arena_io::serialize_solid(&topo, solid).unwrap();
        let mut plain = topo.clone();
        let exact = move_faces_with_evolution(&mut plain, solid, &[face], distance).unwrap();
        let (journaled, quality) = move_faces_journaled_with_quality(
            &mut topo,
            solid,
            &[face],
            distance,
            Some(FaceMoveApproximation { stations: 256 }),
        )
        .unwrap();
        assert_eq!(quality, FaceMoveQuality::Exact);
        assert_eq!(journaled.map.modified, exact.evolution.modified);
        assert_eq!(journaled.map.generated, exact.evolution.generated);
        assert_eq!(journaled.map.deleted, exact.evolution.deleted);
        assert_eq!(journaled.map.unresolved, exact.evolution.unresolved);
        assert_eq!(journaled.map.origin, exact.evolution.origin);
        let geometry = |topo: &Topology, solid| {
            let bytes = remus_io::arena_io::serialize_solid(topo, solid).unwrap();
            let mut doc: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
            doc.as_object_mut().unwrap().remove("journal");
            doc
        };
        assert_eq!(
            geometry(&topo, journaled.solid),
            geometry(&plain, exact.solid)
        );
        // The original geometry survives; the global journal grows.
        let mut source_before: serde_json::Value = serde_json::from_slice(&before).unwrap();
        source_before.as_object_mut().unwrap().remove("journal");
        assert_eq!(geometry(&topo, solid), source_before);
        assert!(
            remus_operations::validate::validate_solid(&topo, journaled.solid)
                .unwrap()
                .is_valid()
        );
        solid = journaled.solid;
    }
    let volume = remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
    assert!((volume - 15_537.695_980_506_103).abs() < 1e-7);
    let phases = remus_operations::performance::drain();
    remus_operations::performance::configure(None);
    assert!(
        !phases
            .iter()
            .any(|phase| phase.name == "move_faces.blend.translation_mesh_closure"),
        "the replay must stay on the certified path"
    );
    assert_eq!(
        phases
            .iter()
            .filter(|phase| phase.name == "move_faces.blend.translation_boundary_certificate")
            .count(),
        42
    );
}

#[test]
#[ignore = "timing diagnostic; run with --profile profiling --ignored --nocapture"]
fn profile_axial_history_replay() {
    use std::time::Instant;
    static EPOCH: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    fn clock() -> f64 {
        EPOCH.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.0
    }
    let (mut topo, mut solid) = fixture();
    remus_operations::performance::configure(Some(clock));
    let start = Instant::now();
    for index in 0..21 {
        let distance = if index == 0 {
            INITIAL_OFFSET_MM
        } else if index % 2 == 1 {
            0.4
        } else {
            -0.3
        };
        let face = top(&topo, solid);
        let result = move_faces_journaled_with_quality(
            &mut topo,
            solid,
            &[face],
            distance,
            Some(FaceMoveApproximation { stations: 256 }),
        )
        .unwrap();
        solid = result.0.solid;
    }
    println!(
        "21 journaled moves: {:.3} ms",
        start.elapsed().as_secs_f64() * 1000.0
    );
    let phases = remus_operations::performance::drain();
    remus_operations::performance::configure(None);
    let mut totals = std::collections::BTreeMap::<_, f64>::new();
    for phase in phases {
        *totals.entry(phase.name).or_default() += phase.duration_ms;
    }
    for (name, ms) in totals {
        println!("{name}: {ms:.3} ms");
    }
    println!(
        "faces={} volume={:.12}",
        solid_faces(&topo, solid).unwrap().len(),
        remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap()
    );
}
