//! Duplicate-face repair scaling benchmarks (B17/PERF-H03).
//!
//! Each benchmark builds one shell of planar quad faces and runs the public
//! [`remus_heal::fix::fix_shape_with_history`] path with every fixer off
//! except duplicate-face removal, so measured time tracks the duplicate pass
//! (candidate discovery + exact predicate + reshape apply).
//!
//! Shapes (all sparse-model friendly unless noted):
//!
//! - `sparse/N`: `N` disjoint coplanar quads on an integer grid. No two share
//!   a bucket; the index pays zero exact comparisons.
//! - `clustered/N`: `N` distinct quads on a sub-tolerance-spaced grid (3e-7
//!   pitch, pairwise differences above tolerance). Neighbors share halo
//!   buckets; each face examines only its neighborhood.
//! - `coincident/N`: `N` geometrically identical quads (one bucket). Every
//!   face matches the survivor on its first candidate (early exit).
//!
//! The dense worst case (one bucket, all faces pairwise distinct, quadratic
//! exact calls) is covered by the `dense_bucket_distinct_*` unit tests with
//! exact counts, not wall time.

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use remus_heal::fix::{FixConfig, FixMode, fix_shape_with_history};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// Every fixer off except duplicate-face removal.
fn duplicate_only_config() -> FixConfig {
    FixConfig {
        fix_reorder: FixMode::Off,
        fix_connectivity: FixMode::Off,
        fix_closure: FixMode::Off,
        fix_small_edges: FixMode::Off,
        fix_self_intersection: FixMode::Off,
        fix_degenerate_edges: FixMode::Off,
        fix_gaps_2d: FixMode::Off,
        fix_gaps_3d: FixMode::Off,
        fix_lacking: FixMode::Off,
        fix_notched: FixMode::Off,
        fix_tail: FixMode::Off,
        fix_intersecting_edges: FixMode::Off,
        fix_wire_orientation: FixMode::Off,
        fix_add_natural_bound: FixMode::Off,
        fix_missing_seam: FixMode::Off,
        fix_small_area: FixMode::Off,
        fix_duplicate_faces: FixMode::Auto,
        fix_intersecting_wires: FixMode::Off,
        fix_orientation: FixMode::Off,
        fix_same_parameter: FixMode::Off,
        fix_vertex_tolerance: FixMode::Off,
        fix_pcurve: FixMode::Off,
        fix_coincident_vertices: FixMode::Off,
        fix_wireframe: FixMode::Off,
        fix_split_common_vertex: FixMode::Off,
        fix_small_faces: FixMode::Off,
    }
}

fn add_quad_at(topo: &mut Topology, x: f64, y: f64) -> remus_topology::face::FaceId {
    let corners = [
        Point3::new(x, y, 0.0),
        Point3::new(x + 1.0, y, 0.0),
        Point3::new(x + 1.0, y + 1.0, 0.0),
        Point3::new(x, y + 1.0, 0.0),
    ];
    let vs: Vec<_> = corners
        .iter()
        .map(|point| topo.add_vertex(Vertex::new(*point, 1e-7)))
        .collect();
    let es: Vec<_> = (0..4)
        .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
        .collect();
    let wire = topo.add_wire(
        Wire::new(
            es.into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ))
}

fn build_shell(topo: &mut Topology, positions: &[(f64, f64)]) -> SolidId {
    let faces: Vec<_> = positions
        .iter()
        .map(|(x, y)| add_quad_at(topo, *x, *y))
        .collect();
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

fn sparse_positions(n: usize) -> Vec<(f64, f64)> {
    (0..n).map(|i| (i as f64 * 10.0, 0.0)).collect()
}

fn clustered_positions(n: usize) -> Vec<(f64, f64)> {
    // Grid pitch 1.5e-7 (1.5x tolerance): neighbors share halo buckets but
    // every pairwise difference stays above tolerance — distinct faces that
    // still exercise the exact predicate on their neighborhoods.
    let side = n.isqrt().max(1);
    (0..n)
        .map(|i| ((i % side) as f64 * 1.5e-7, (i / side) as f64 * 1.5e-7))
        .collect()
}

fn coincident_positions(n: usize) -> Vec<(f64, f64)> {
    vec![(0.0, 0.0); n]
}

fn bench_shape(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    sizes: &[usize],
    positions: fn(usize) -> Vec<(f64, f64)>,
) {
    let config = duplicate_only_config();
    for size in sizes {
        group.bench_with_input(BenchmarkId::new(name, size), size, |b, &size| {
            b.iter_batched(
                || {
                    let mut topo = Topology::new();
                    let solid = build_shell(&mut topo, &positions(size));
                    (topo, solid)
                },
                |(mut topo, solid)| {
                    let (_, result, _) =
                        fix_shape_with_history(&mut topo, solid, &config, Some(1e-7)).unwrap();
                    black_box(result.actions_taken);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
}

fn duplicate_faces(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("duplicate_faces");
    bench_shape(&mut group, "sparse", &[200, 800, 2000], sparse_positions);
    bench_shape(&mut group, "clustered", &[200, 800], clustered_positions);
    bench_shape(&mut group, "coincident", &[50, 200], coincident_positions);
    group.finish();
}

criterion_group!(
    name = duplicate_face_benches;
    config = Criterion::default()
        .sample_size(20)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));
    targets = duplicate_faces
);
criterion_main!(duplicate_face_benches);
