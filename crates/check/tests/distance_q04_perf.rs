//! PERF-Q04/Q05 performance probe: point-to-solid distance scaling.
//!
//! Uses the public `point_to_solid*` API. Prints wall-time, pruning, and
//! traversal counters with `--nocapture`; asserts correctness against
//! closed forms plus pruning-effectiveness invariants (never wall-time
//! thresholds, which are runner-noise sensitive).
//!
//! Models: small (6 faces), sparse row, clustered row, heavily overlapping
//! row (240 faces each), a 12-face wavy-NURBS row (expensive narrow phase),
//! and an unknown-bound-heavy model whose rims carry invalid trim authority
//! (all faces mandatory).

// Bitwise float equality is intentional below: both traversal modes share the
// narrow phase and candidate order, so agreement must be exact, not approximate.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout,
    clippy::float_cmp
)]

use std::time::Instant;

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::test_utils::make_unit_cube_manifold_at;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use remus_check::distance::{DistanceStats, point_to_solid_with_stats};

const TOL: f64 = 1e-7;

/// Merge the faces of `count` disjoint unit cubes into one multi-face solid.
///
/// `spacing` controls overlap: values > 1.0 give sparse (separated) bounds,
/// values < 1.0 give heavily overlapping bounds.
fn make_cube_row(topo: &mut Topology, count: usize, spacing: f64) -> SolidId {
    let mut faces: Vec<FaceId> = Vec::with_capacity(6 * count);
    for k in 0..count {
        #[allow(clippy::cast_precision_loss)]
        let origin = k as f64 * spacing;
        let solid = make_unit_cube_manifold_at(topo, origin, 0.0, 0.0);
        let shell_id = topo.solid(solid).unwrap().outer_shell();
        faces.extend(topo.shell(shell_id).unwrap().faces().iter().copied());
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// A row of planar disc faces, each bounded by a closed circle rim whose
/// stored trim overruns the full turn (invalid authority).
///
/// Every face is unknown-bound (mandatory path) while the narrow phase still
/// functions (closed rims sample without trim authority): the acceleration
/// structures must decline to prune while still returning the exact answer.
fn make_unknown_row(topo: &mut Topology, count: usize) -> SolidId {
    let mut faces: Vec<FaceId> = Vec::new();
    for k in 0..count {
        #[allow(clippy::cast_precision_loss)]
        let ox = k as f64 * 3.0;
        let rim = Circle3D::new(Point3::new(ox, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let seam = topo.add_vertex(Vertex::new(Point3::new(ox + 1.0, 0.0, 0.0), TOL));
        let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(rim));
        // Overrun trim: authoritative validation rejects it, so the bound is
        // unknown even though the closed rim still samples for the polygon.
        edge.set_trim(Some((0.0, std::f64::consts::TAU + 0.5)));
        let rim_id = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim_id, true)], true).unwrap());
        faces.push(topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        )));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// Run one query, print the counters, and return them.
fn measure(
    label: &str,
    topo: &Topology,
    solid: SolidId,
    query: Point3,
    expected: f64,
) -> DistanceStats {
    // Warm up once so the timed run excludes cold-cache effects.
    let _ = point_to_solid_with_stats(topo, query, solid).unwrap();
    let start = Instant::now();
    let (result, stats) = point_to_solid_with_stats(topo, query, solid).unwrap();
    let elapsed = start.elapsed().as_secs_f64();
    println!(
        "PERF-Q04 {label}: faces={} prunable={} mandatory={} evaluated={} skipped={} \
         failures={} latency={elapsed:.6}s distance={:.6}",
        stats.faces_total,
        stats.faces_prunable,
        stats.faces_mandatory,
        stats.faces_evaluated,
        stats.faces_skipped_by_bound,
        stats.narrow_phase_failures,
        result.distance,
    );
    assert!(
        (result.distance - expected).abs() < 1e-9,
        "{label}: expected {expected}, got {}",
        result.distance
    );
    assert_eq!(
        stats.faces_evaluated + stats.faces_skipped_by_bound,
        stats.faces_total,
        "{label}: every face must be evaluated or skipped"
    );
    stats
}

#[test]
fn perf_q04_small_input_single_box() {
    let mut topo = Topology::new();
    let solid = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
    // 6 prunable faces: linear best-first scan, no tree.
    let stats = measure("small", &topo, solid, Point3::new(0.5, 0.5, 3.0), 2.0);
    assert_eq!(stats.faces_total, 6);
}

#[test]
fn perf_q04_sparse_row() {
    let mut topo = Topology::new();
    // 40 disjoint cubes spaced 3 apart: 240 faces, query near the first cube.
    let solid = make_cube_row(&mut topo, 40, 3.0);
    let stats = measure(
        "sparse-row-40",
        &topo,
        solid,
        Point3::new(0.5, 0.5, 3.0),
        2.0,
    );
    assert!(
        stats.faces_skipped_by_bound > stats.faces_evaluated,
        "sparse query must skip most faces: {:?}",
        stats
    );
}

#[test]
fn perf_q04_clustered_row() {
    let mut topo = Topology::new();
    // 40 adjacent cubes (touching): bounds cluster with shared walls.
    let solid = make_cube_row(&mut topo, 40, 1.0);
    let stats = measure(
        "clustered-row-40",
        &topo,
        solid,
        Point3::new(20.5, 0.5, 3.0),
        2.0,
    );
    assert!(
        stats.faces_skipped_by_bound > 0,
        "clustered query must prune some faces: {:?}",
        stats
    );
}

#[test]
fn perf_q04_heavily_overlapping_row() {
    let mut topo = Topology::new();
    // 40 coincident cubes: every bound overlaps; pruning helps the least.
    let solid = make_cube_row(&mut topo, 40, 0.0);
    let stats = measure(
        "overlapping-row-40",
        &topo,
        solid,
        Point3::new(0.5, 0.5, 3.0),
        2.0,
    );
    // Correctness is what matters here; overlap defeats selectivity, and the
    // counters must show the work went to evaluation, not to pruning claims.
    assert_eq!(
        stats.faces_evaluated + stats.faces_skipped_by_bound,
        stats.faces_total
    );
}

#[test]
fn perf_q04_unknown_bound_heavy() {
    let mut topo = Topology::new();
    // 20 disc faces with invalid rim authority: all mandatory.
    let solid = make_unknown_row(&mut topo, 20);
    let stats = measure(
        "unknown-row-20",
        &topo,
        solid,
        Point3::new(0.0, 0.0, 3.0),
        3.0,
    );
    assert_eq!(stats.faces_mandatory, 20, "overrun trims must be mandatory");
    assert_eq!(stats.faces_prunable, 0);
    assert_eq!(stats.faces_evaluated, 20);
    assert_eq!(stats.faces_skipped_by_bound, 0);
}

/// A row of wavy NURBS patches spaced apart: the narrow phase (Newton
/// projection) is expensive, so pruning shows up in wall time, not just in
/// face counts. Compares branch-and-bound against forced-exhaustive on the
/// identical narrow phase.
#[test]
fn perf_q04_nurbs_row_bnb_vs_exhaustive() {
    use remus_check::distance::point_to_solid_exhaustive;
    use remus_math::nurbs::surface::NurbsSurface;

    let mut topo = Topology::new();
    let mut faces: Vec<FaceId> = Vec::new();
    for k in 0..12 {
        #[allow(clippy::cast_precision_loss)]
        let ox = k as f64 * 4.0;
        let mut cps = Vec::new();
        let mut ws = Vec::new();
        for i in 0..4 {
            let mut row = Vec::new();
            let mut wrow = Vec::new();
            for j in 0..4 {
                let x = ox + f64::from(i) / 3.0;
                let y = f64::from(j) / 3.0;
                let z = 0.08 * (2.0 * x).sin() * (2.0 * y).cos();
                row.push(Point3::new(x, y, z));
                wrow.push(1.0);
            }
            cps.push(row);
            ws.push(wrow);
        }
        let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let surface = NurbsSurface::new(3, 3, knots.clone(), knots, cps, ws).unwrap();
        let corners = [
            (ox, 0.0, surface.evaluate(0.0, 0.0).z()),
            (ox + 1.0, 0.0, surface.evaluate(1.0, 0.0).z()),
            (ox + 1.0, 1.0, surface.evaluate(1.0, 1.0).z()),
            (ox, 1.0, surface.evaluate(0.0, 1.0).z()),
        ]
        .map(|(x, y, z)| topo.add_vertex(Vertex::new(Point3::new(x, y, z), TOL)));
        let line = |a: remus_topology::vertex::VertexId, b: remus_topology::vertex::VertexId| {
            Edge::new(a, b, EdgeCurve::Line)
        };
        let f0 = topo.add_edge(line(corners[0], corners[1]));
        let f1 = topo.add_edge(line(corners[1], corners[2]));
        let f2 = topo.add_edge(line(corners[2], corners[3]));
        let f3 = topo.add_edge(line(corners[3], corners[0]));
        let wire = topo.add_wire(
            Wire::new(
                [f0, f1, f2, f3]
                    .iter()
                    .map(|&id| OrientedEdge::new(id, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        faces.push(topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface))));
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let row = topo.add_solid(Solid::new(shell, vec![]));

    let query = Point3::new(0.5, 0.5, 3.0);
    let _ = point_to_solid_with_stats(&topo, query, row).unwrap();
    let _ = point_to_solid_exhaustive(&topo, query, row).unwrap();

    let start = Instant::now();
    let (fast, fast_stats) = point_to_solid_with_stats(&topo, query, row).unwrap();
    let fast_time = start.elapsed().as_secs_f64();
    let start = Instant::now();
    let (slow, slow_stats) = point_to_solid_exhaustive(&topo, query, row).unwrap();
    let slow_time = start.elapsed().as_secs_f64();
    println!(
        "PERF-Q04 nurbs-row-12: bnb evaluated={} skipped={} latency={fast_time:.6}s | \
         exhaustive evaluated={} latency={slow_time:.6}s | distance={:.6}",
        fast_stats.faces_evaluated,
        fast_stats.faces_skipped_by_bound,
        slow_stats.faces_evaluated,
        fast.distance,
    );
    assert_eq!(fast.distance, slow.distance);
    assert!(fast_stats.faces_skipped_by_bound > 0);
    assert_eq!(slow_stats.faces_evaluated, 12);
    // Non-vacuous: the narrow phase must converge (otherwise both modes
    // trivially agree on failure).
    assert_eq!(fast_stats.narrow_phase_failures, 0);
    assert!(fast.distance.is_finite() && fast.distance < 3.5);
}
