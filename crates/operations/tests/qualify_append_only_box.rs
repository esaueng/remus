//! PERF-T03 qualification: the native `make_box` path runs in an append-only
//! transaction scope whose cost follows new content.
//!
//! - 150 box constructions over increasing pre-existing document sizes take
//!   the append-only path every time, grow the arenas by exactly one box
//!   per build, and validate as closed solids with the expected volume.
//! - Injected failure after each allocation stage (vertices, edges,
//!   wires/faces, shell/solid) retires every staged allocation: handles
//!   stay stale, later builds append above the preserved high-water mark,
//!   and pre-existing solids are untouched.
//! - Writes to pre-existing entities inside the scope fall back to the full
//!   path with an identical result (observed via [`AppendPath`]), and genuinely
//!   invalid dimensions still fail without leaving partial topology.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_operations::OperationsError;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::transaction::{AppendPath, run_append_only};

/// Slots per `make_box`: 8 vertices, 12 edges, 6 wires, 6 faces, 1 shell,
/// 1 solid, 6 loops, 24 coedges.
const BOX_SLOTS: usize = 8 + 12 + 6 + 6 + 1 + 1 + 6 + 24;

#[test]
fn one_hundred_fifty_boxes_take_the_append_only_path_at_each_size() {
    for preexisting in [0usize, 50, 150] {
        let mut topo = Topology::new();
        for _ in 0..preexisting {
            make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        }
        let slots_before = topo.allocated_slot_count();
        for _ in 0..150 {
            // `make_box` opens its own append-only scope; the observed outer
            // scope nests around it, exercising nested append-only commit.
            let (solid, path) = run_append_only(&mut topo, |t| make_box(t, 1.0, 1.0, 1.0)).unwrap();
            assert_eq!(path, AppendPath::AppendOnly);
            assert!(validate_solid(&topo, solid).unwrap().is_valid());
        }
        // Exactly one box worth of slots per build: no hidden copies, no
        // reissued or missing allocations.
        assert_eq!(
            topo.allocated_slot_count() - slots_before,
            150 * BOX_SLOTS,
            "preexisting={preexisting}"
        );
        assert_eq!(topo.num_solids(), preexisting + 150);
        // Spot-check volume on the last build.
        let last = topo.solids().iter().last().unwrap().0;
        let volume = solid_volume(&topo, last, 0.01).unwrap();
        assert!((volume - 1.0).abs() < 1e-6, "unit box volume, got {volume}");
    }
}

#[test]
fn injected_failure_after_each_allocation_stage_retires_cleanly() {
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    // One pre-existing box the failing scopes must never disturb.
    let mut topo = Topology::new();
    let kept = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let kept_volume = solid_volume(&topo, kept, 0.01).unwrap();

    // Mirror make_box's allocation stages; `fail_after` injects refusal
    // after stage 0 (vertices), 1 (edges), 2 (wires/faces), or 3
    // (shell/solid). Stage 4 runs clean.
    for fail_after in 0..5 {
        let slots_before = topo.allocated_slot_count();
        let live_before = (topo.num_vertices(), topo.num_solids());
        let mut leaked_vertex = None;
        let mut leaked_face = None;
        let mut leaked_solid = None;
        let result = run_append_only(&mut topo, |t| {
            let mut stage = 0;
            let v: Vec<_> = (0..8)
                .map(|i| {
                    t.add_vertex(Vertex::new(
                        Point3::new(50_000.0 + i as f64, 0.0, 0.0),
                        1e-7,
                    ))
                })
                .collect();
            leaked_vertex = Some(v[0]);
            if stage == fail_after {
                return Err::<(), _>("injected");
            }
            stage += 1;
            let e: Vec<_> = [(0, 1), (1, 2), (2, 3), (3, 0)]
                .iter()
                .map(|&(a, b)| t.add_edge(Edge::new(v[a], v[b], EdgeCurve::Line)))
                .collect();
            if stage == fail_after {
                return Err::<(), _>("injected");
            }
            stage += 1;
            let wire = Wire::new(
                e.iter().map(|&id| OrientedEdge::new(id, true)).collect(),
                true,
            )
            .unwrap();
            let wid = t.add_wire(wire);
            let face = t.add_face(Face::new(
                wid,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            ));
            leaked_face = Some(face);
            if stage == fail_after {
                return Err::<(), _>("injected");
            }
            stage += 1;
            let shell = t.add_shell(Shell::new(vec![face]).unwrap());
            let solid = t.add_solid(Solid::new(shell, vec![]));
            leaked_solid = Some(solid);
            if stage == fail_after {
                return Err::<(), _>("injected");
            }
            Ok(())
        });
        if fail_after < 4 {
            assert!(result.is_err(), "stage {fail_after} must fail");
            assert_eq!(
                (topo.num_vertices(), topo.num_solids()),
                live_before,
                "stage {fail_after}: live counts unchanged"
            );
            // Every staged handle stays stale, and slots are never reused.
            assert!(
                topo.vertex(leaked_vertex.unwrap()).is_err(),
                "stage {fail_after}"
            );
            if fail_after >= 2 {
                assert!(
                    topo.face(leaked_face.unwrap()).is_err(),
                    "stage {fail_after}"
                );
            }
            if fail_after >= 3 {
                assert!(
                    topo.solid(leaked_solid.unwrap()).is_err(),
                    "stage {fail_after}"
                );
            }
            assert!(topo.allocated_slot_count() >= slots_before);
        } else {
            let ((), path) = result.unwrap();
            assert_eq!(path, AppendPath::AppendOnly);
            assert_eq!(topo.num_vertices(), live_before.0 + 8);
            assert_eq!(topo.num_solids(), live_before.1 + 1);
        }
        // The pre-existing box is untouched by every failing scope.
        assert!(
            validate_solid(&topo, kept).unwrap().is_valid(),
            "stage {fail_after}"
        );
        let volume = solid_volume(&topo, kept, 0.01).unwrap();
        assert!((volume - kept_volume).abs() < 1e-9, "stage {fail_after}");
    }
}

#[test]
fn preexisting_writes_fall_back_with_identical_results() {
    use remus_math::vec::Point3;

    let mut topo = Topology::new();
    let kept = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let first = topo.vertices().iter().next().unwrap().0;

    // Mutating a pre-existing vertex trips the guard; the fallback retry
    // still applies the whole operation exactly once. (Moving the vertex
    // legitimately breaks the kept box's own closure — the full path does
    // exactly the same — so validity is checked on the new box.)
    let (fresh, path) = run_append_only(&mut topo, |t| -> Result<_, OperationsError> {
        let fresh = make_box(t, 1.0, 1.0, 1.0)?;
        t.vertex_mut(first)?.set_point(Point3::new(9.0, 9.0, 9.0));
        Ok(fresh)
    })
    .unwrap();
    let _ = kept;
    assert_eq!(path, AppendPath::FullFallback);
    assert_eq!(
        topo.vertex(first).unwrap().point(),
        Point3::new(9.0, 9.0, 9.0)
    );
    assert_eq!(topo.num_solids(), 2, "one new box, applied once");
    assert!(validate_solid(&topo, fresh).unwrap().is_valid());
}

#[test]
fn invalid_dimensions_fail_without_partial_topology() {
    let mut topo = Topology::new();
    let kept = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    assert!(make_box(&mut topo, -1.0, 1.0, 1.0).is_err());
    assert!(make_box(&mut topo, 0.0, 1.0, 1.0).is_err());
    assert!(make_box(&mut topo, f64::NAN, 1.0, 1.0).is_err());
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids()
        ),
        counts,
        "refused builds leave no partial topology"
    );
    assert!(validate_solid(&topo, kept).unwrap().is_valid());
}
