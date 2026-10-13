//! Fresh holder quantity cost after a width edit, including volume policy and
//! the complete quantity mesh workflow. Import/edit are setup; each timed
//! call clears scalar/mesh memos and preserves the requested 0.08 deflection.
//! Native timings qualify this workload only; packaged WASM uses its own
//! maintained edit-to-analysis harness.
#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};
use remus_operations::{measure, push_pull, tessellate};
use remus_topology::Topology;

/// Benchmark fresh quantity reads after the maintained holder width edits.
fn fresh_holder_quantity(c: &mut Criterion) {
    tessellate::disable_face_mesh_cache();
    let mut source = Topology::new();
    let source_solid = remus_io::step::reader::read_step(
        include_str!("../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut source,
    )
    .unwrap()[0];
    let faces = remus_topology::explorer::solid_faces(&source, source_solid).unwrap();
    assert_eq!(faces.len(), 160);
    let face = faces
        .into_iter()
        .find(|&face| {
            source
                .face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.99)
                && (measure::face_area(&source, face, 0.05).unwrap() - 1_045.930_731_242_697).abs()
                    < 1e-4
        })
        .expect("holder +X width face");
    for distance in [5., 5.06] {
        let mut topo = source.clone();
        let solid = push_pull::move_faces(&mut topo, source_solid, &[face], distance).unwrap();
        let baseline = measure::solid_volume(&topo, solid, 0.08).unwrap();
        assert!(baseline.is_finite() && baseline > 0.);
        c.bench_function(
            &format!("perf-d01 fresh holder quantity width={distance}"),
            |b| {
                b.iter(|| {
                    measure::clear_thread_volume_memo();
                    tessellate::clear_face_mesh_cache();
                    black_box(measure::solid_volume(black_box(&topo), solid, 0.08).unwrap())
                });
            },
        );
    }
}

criterion_group! {
    name = quantity;
    config = Criterion::default()
        .sample_size(20)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(3));
    targets = fresh_holder_quantity
}
criterion_main!(quantity);
