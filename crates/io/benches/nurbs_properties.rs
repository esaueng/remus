//! NURBS-heavy validation and mass-property benchmarks on the shipped
//! Shapr3D hammer-holder fixture (42 NURBS faces incl. 38x58 bicubic patches).
//!
//! Strict `validate_solid` integrates a signed volume over every face for its
//! inside-out check, and `mass_properties` integrates second moments at Gauss
//! order 8; both are bound by `NurbsSurface::derivatives`.

#![allow(missing_docs, clippy::expect_used, clippy::unwrap_used)]

use criterion::{Criterion, criterion_group, criterion_main};
use remus_operations::{measure::mass_properties, validate::validate_solid};
use remus_topology::Topology;

fn hammer_holder() -> (Topology, remus_topology::solid::SolidId) {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .expect("fixture parses");
    (topo, solids[0])
}

fn bench_nurbs_properties(c: &mut Criterion) {
    let (topo, solid) = hammer_holder();
    let mut group = c.benchmark_group("nurbs_properties");
    group.sample_size(10);
    group.bench_function("validate_solid strict (hammer holder)", |b| {
        b.iter(|| {
            let report = validate_solid(&topo, solid).expect("validation runs");
            assert!(report.is_valid());
        });
    });
    group.bench_function("mass_properties (hammer holder)", |b| {
        b.iter(|| mass_properties(&topo, solid).expect("mass properties"));
    });
    group.finish();
}

criterion_group!(benches, bench_nurbs_properties);
criterion_main!(benches);
