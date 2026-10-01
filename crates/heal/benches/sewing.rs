//! Shell-sewing scaling benchmarks (PERF-H02 curve-pruning slice).
//!
//! Each benchmark builds one shell of planar quad faces with disjoint
//! vertices/edges (patch-by-patch input) and runs the public
//! [`remus_heal::upgrade::shell_sewing::sew_shell`] path, so measured time
//! tracks planning (endpoint index + midpoint-descriptor prune + exact
//! compatibility) plus reshape apply.
//!
//! Shapes:
//!
//! - `sparse/N`: `N` disjoint line quads on an integer grid. No two share an
//!   endpoint neighborhood; the index proposes nothing, no curve checks.
//! - `clustered/N`: `N` line quads as `N/2` far-apart compatible twin pairs
//!   (same segment, opposite seam directions for manifold closure). Each pair
//!   sews; neighborhoods stay local.
//! - `dense_incompatible/N`: `N` quads sharing one seam segment (0,0)-(1,0)
//!   with distinct circular-arc bulges (pairwise incompatible via distinct
//!   midpoints) plus unique side edges per quad to isolate the seam.
//!   Endpoint index alone proposes every pair; midpoint pruning rejects all
//!   without sampling (targeted workload).
//! - `dense_compatible/N`: `N` quads (even) sharing one seam as `N/2` twin
//!   arc pairs (same bulge twice, cross-bulge distinct) with unique sides.
//!   Each twin sews; cross pairs prune. Expensive checks drop from quadratic
//!   to linear in the twin count; cheap descriptor examinations remain
//!   quadratic by design (dense legitimate interactions may remain quadratic).

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use remus_heal::upgrade::shell_sewing::sew_shell;
use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const TOL: f64 = 1e-7;
const SEW_TOL: f64 = 1e-6;

fn line_quad(topo: &mut Topology, pts: [Point3; 4]) -> remus_topology::face::FaceId {
    let vs: Vec<_> = pts
        .iter()
        .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
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

fn circle_edge_for_bench(
    topo: &mut Topology,
    start: remus_topology::vertex::VertexId,
    end: remus_topology::vertex::VertexId,
    circle: Circle3D,
) -> remus_topology::edge::EdgeId {
    let start_parameter = circle.project(topo.vertex(start).unwrap().point());
    let canonical_end = circle.project(topo.vertex(end).unwrap().point());
    let end_parameter = if start == end {
        start_parameter + std::f64::consts::TAU
    } else if canonical_end <= start_parameter {
        canonical_end + std::f64::consts::TAU
    } else {
        canonical_end
    };
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
    edge.set_trim(Some((start_parameter, end_parameter)));
    topo.add_edge(edge)
}

fn build_sparse(topo: &mut Topology, n: usize) -> remus_topology::shell::ShellId {
    let faces: Vec<_> = (0..n)
        .map(|i| {
            let x = i as f64 * 10.0;
            line_quad(
                topo,
                [
                    Point3::new(x, 0.0, 0.0),
                    Point3::new(x + 1.0, 0.0, 0.0),
                    Point3::new(x + 1.0, 1.0, 0.0),
                    Point3::new(x, 1.0, 0.0),
                ],
            )
        })
        .collect();
    topo.add_shell(Shell::new(faces).unwrap())
}

fn build_clustered(topo: &mut Topology, n: usize) -> remus_topology::shell::ShellId {
    // N quads as N/2 far-apart twin pairs sharing one segment per pair
    // (opposite seam directions for manifold closure). N must be even.
    assert!(n.is_multiple_of(2));
    let mut faces = Vec::new();
    for k in 0..n / 2 {
        let x = k as f64 * 10.0;
        faces.push(line_quad(
            topo,
            [
                Point3::new(x, 0.0, 0.0),
                Point3::new(x + 1.0, 0.0, 0.0),
                Point3::new(x + 1.0, 1.0, 0.0),
                Point3::new(x, 1.0, 0.0),
            ],
        ));
        faces.push(line_quad(
            topo,
            [
                Point3::new(x + 1.0, 0.0, 0.0),
                Point3::new(x, 0.0, 0.0),
                Point3::new(x, -1.0, 0.0),
                Point3::new(x + 1.0, -1.0, 0.0),
            ],
        ));
    }
    topo.add_shell(Shell::new(faces).unwrap())
}

fn distinct_arc_quad(
    topo: &mut Topology,
    s: f64,
    side_base_y: f64,
) -> remus_topology::face::FaceId {
    let r = (0.25 + s * s) / (2.0 * s);
    let yc = s - r;
    let circle = Circle3D::new(Point3::new(0.5, yc, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
    let va = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
    let vb = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), TOL));
    let vc = topo.add_vertex(Vertex::new(Point3::new(1.0, side_base_y, 0.0), TOL));
    let vd = topo.add_vertex(Vertex::new(Point3::new(0.0, side_base_y, 0.0), TOL));
    let arc = circle_edge_for_bench(topo, va, vb, circle);
    let e1 = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(vc, vd, EdgeCurve::Line));
    let e3 = topo.add_edge(Edge::new(vd, va, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            [arc, e1, e2, e3]
                .into_iter()
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

fn build_dense_incompatible(topo: &mut Topology, n: usize) -> remus_topology::shell::ShellId {
    let faces: Vec<_> = (0..n)
        .map(|k| {
            let s = 0.05 + k as f64 * 0.005;
            distinct_arc_quad(topo, s, -2.0 - k as f64 * 10.0)
        })
        .collect();
    topo.add_shell(Shell::new(faces).unwrap())
}

fn build_dense_compatible(topo: &mut Topology, n: usize) -> remus_topology::shell::ShellId {
    // N quads (even) as N/2 twin arc pairs sharing one seam, cross-bulge
    // distinct, sides unique per quad to isolate the seam.
    assert!(n.is_multiple_of(2));
    let mut faces = Vec::new();
    for k in 0..n / 2 {
        let s = 0.05 + k as f64 * 0.008;
        faces.push(distinct_arc_quad(topo, s, -2.0 - (2 * k) as f64 * 10.0));
        faces.push(distinct_arc_quad(topo, s, -2.0 - (2 * k + 1) as f64 * 10.0));
    }
    topo.add_shell(Shell::new(faces).unwrap())
}

fn bench_sewing_shape(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    sizes: &[usize],
    build: fn(&mut Topology, usize) -> remus_topology::shell::ShellId,
) {
    for size in sizes {
        group.bench_with_input(BenchmarkId::new(name, size), size, |b, &size| {
            b.iter_batched(
                || {
                    let mut topo = Topology::new();
                    let shell = build(&mut topo, size);
                    (topo, shell)
                },
                |(mut topo, shell)| {
                    let sewn = sew_shell(&mut topo, shell, SEW_TOL).unwrap();
                    black_box(sewn);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
}

fn sewing(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("sewing");
    bench_sewing_shape(&mut group, "sparse", &[10, 50, 100], build_sparse);
    bench_sewing_shape(&mut group, "clustered", &[10, 50, 100], build_clustered);
    bench_sewing_shape(
        &mut group,
        "dense_incompatible",
        &[10, 25, 50],
        build_dense_incompatible,
    );
    bench_sewing_shape(
        &mut group,
        "dense_compatible",
        &[10, 50, 100],
        build_dense_compatible,
    );
    group.finish();
}

criterion_group!(
    name = sewing_benches;
    config = Criterion::default()
        .sample_size(20)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));
    targets = sewing
);
criterion_main!(sewing_benches);
