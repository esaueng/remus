//! PERF-D03 paired benchmark: complete-workflow tessellation time and
//! allocations over the bounded production domain.
//!
//! Measures the WHOLE `tessellate_solid` workflow (boundary planning, local
//! triangulation, reconciliation, deterministic assembly, weld, dedupe) —
//! never local triangulation alone. Each bench prints one stats line with
//! mesh allocations (vertex/triangle bytes), watertight counts, and face
//! attribution width so time and memory stay paired:
//!
//!   `cargo bench -p remus-operations --bench perf_d03_boundary`
//!
//! Thread scaling is qualified by running the same binary under
//! `RAYON_NUM_THREADS=1` and the default pool and comparing both the
//! criterion times and the printed stats lines (they must match exactly).

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::{Duration, Instant};

use criterion::{Criterion, criterion_group, criterion_main};

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::tessellate;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn fast_config() -> Criterion {
    Criterion::default()
        .sample_size(20)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2))
}

/// One stats line pairing allocations with quality for a tessellated solid.
///
/// Criterion timing covers wall time; this records the allocation side
/// (vertex/triangle bytes) plus the watertight counts into a report file
/// (`$TMPDIR/perf-d03-<label>.txt`, overwritten per run) because workspace
/// lints deny prints even in benches. Compare files across
/// `RAYON_NUM_THREADS` settings: the stats lines must match exactly.
/// Non-watertight output fails the bench setup loudly instead of
/// benchmarking a broken mesh.
fn report_stats(label: &str, mesh: &tessellate::TriangleMesh, build: Duration) {
    let verts = mesh.positions.len();
    let tris = mesh.indices.len() / 3;
    let pos_bytes = verts * size_of::<remus_math::vec::Point3>();
    let nrm_bytes = mesh.normals.len() * size_of::<remus_math::vec::Vec3>();
    let idx_bytes = mesh.indices.len() * size_of::<u32>();
    let boundary = tessellate::boundary_edge_count(mesh);
    let nonmanifold = tessellate::non_manifold_edge_count(mesh);
    assert_eq!(
        (boundary, nonmanifold),
        (0, 0),
        "{label} fixture must tessellate watertight"
    );
    let line = format!(
        "[perf-d03] {label}: verts={verts} tris={tris} \
         bytes(pos/nrm/idx)={pos_bytes}/{nrm_bytes}/{idx_bytes} \
         boundary={boundary} nonmanifold={nonmanifold} build={build:?}\n"
    );
    let path = std::env::temp_dir().join(format!("perf-d03-{label}.txt"));
    std::fs::write(&path, line).unwrap_or(());
}

fn drilled_box() -> (Topology, remus_topology::solid::SolidId) {
    let mut topo = Topology::new();
    let box_s = primitives::make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let cyl = primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
    transform_solid(&mut topo, cyl, &Mat4::translation(10.0, 10.0, -5.0)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, box_s, cyl).unwrap();
    (topo, result)
}

/// Holed-planar production domain: drilled box at display deflection.
fn bench_drilled_box(c: &mut Criterion) {
    let (topo, solid) = drilled_box();
    let built = Instant::now();
    let mesh = tessellate::tessellate_solid(&topo, solid, 0.1).unwrap();
    report_stats("drilled-box", &mesh, built.elapsed());
    c.bench_function("perf-d03 drilled box (tol=0.1)", |b| {
        b.iter(|| black_box(tessellate::tessellate_solid(&topo, solid, 0.1).unwrap()));
    });
}

/// Curved-trim fallback preserved: box∩sphere exercises spherical patches
/// and planar clips through the curved dispatch outside the holed-planar
/// family.
fn bench_curved_intersect(c: &mut Criterion) {
    let mut topo = Topology::new();
    let bx = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let sp = primitives::make_sphere(&mut topo, 7.0, 16).unwrap();
    let result = boolean(&mut topo, BooleanOp::Intersect, bx, sp).unwrap();
    let built = Instant::now();
    let mesh = tessellate::tessellate_solid(&topo, result, 0.1).unwrap();
    report_stats("box-sphere-intersect", &mesh, built.elapsed());
    c.bench_function("perf-d03 box-sphere intersect (tol=0.1)", |b| {
        b.iter(|| black_box(tessellate::tessellate_solid(&topo, result, 0.1).unwrap()));
    });
}

/// Cavity solid: hollow box keeps inner-shell faces in the same workflow.
fn bench_hollow_box(c: &mut Criterion) {
    let mut topo = Topology::new();
    let outer = primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let inner = primitives::make_box(&mut topo, 8.0, 8.0, 8.0).unwrap();
    transform_solid(&mut topo, inner, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, outer, inner).unwrap();
    let built = Instant::now();
    let mesh = tessellate::tessellate_solid(&topo, result, 0.1).unwrap();
    report_stats("hollow-box", &mesh, built.elapsed());
    c.bench_function("perf-d03 hollow box (tol=0.1)", |b| {
        b.iter(|| black_box(tessellate::tessellate_solid(&topo, result, 0.1).unwrap()));
    });
}

/// Scaling input: 64-hole plate drives the parallel sampling/CDT paths and
/// the contact-refinement budget at production size.
fn bench_64_hole_plate(c: &mut Criterion) {
    let mut topo = Topology::new();
    let mut result = primitives::make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    for row in 0..8 {
        for col in 0..8 {
            let cyl = primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
            let x = 6.0 + col as f64 * 12.0;
            let y = 6.0 + row as f64 * 12.0;
            transform_solid(&mut topo, cyl, &Mat4::translation(x, y, -5.0)).unwrap();
            result = boolean(&mut topo, BooleanOp::Cut, result, cyl).unwrap();
        }
    }
    let built = Instant::now();
    let mesh = tessellate::tessellate_solid(&topo, result, 0.1).unwrap();
    report_stats("64-hole-plate", &mesh, built.elapsed());
    c.bench_function("perf-d03 64-hole plate (tol=0.1)", |b| {
        b.iter(|| black_box(tessellate::tessellate_solid(&topo, result, 0.1).unwrap()));
    });
}

criterion_group!(
    name = perf_d03;
    config = fast_config();
    targets = bench_drilled_box, bench_curved_intersect, bench_hollow_box,
        bench_64_hole_plate
);
criterion_main!(perf_d03);
