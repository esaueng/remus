//! NURBS-heavy validation and mass-property benchmarks on the shipped
//! Shapr3D hammer-holder fixture (42 NURBS faces incl. 38x58 bicubic patches).
//!
//! Strict `validate_solid` integrates a signed volume over every face for its
//! inside-out check, and `mass_properties` integrates second moments at Gauss
//! order 8; both are bound by `NurbsSurface::derivatives`. The warm variant
//! measures strict validation answered from the opt-in face-integral cache.

#![allow(missing_docs, clippy::expect_used, clippy::unwrap_used)]

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
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
    // The opt-in face-integral cache (off above): every iteration after the
    // first is answered from it, which is what a consumer repeating strict
    // validation of an unchanged body pays.
    remus_check::properties::face_cache::enable_thread_face_cache();
    group.bench_function(
        "validate_solid strict, warm face cache (hammer holder)",
        |b| {
            b.iter(|| {
                let report = validate_solid(&topo, solid).expect("validation runs");
                assert!(report.is_valid());
            });
        },
    );
    remus_check::properties::face_cache::set_thread_face_cache_limits(0, 0);
    group.bench_function("mass_properties (hammer holder)", |b| {
        b.iter(|| mass_properties(&topo, solid).expect("mass properties"));
    });
    group.finish();
}

/// Measure the orientation integrals a new offset misses after priming only
/// its source. Source-cache cloning and result-cache teardown stay outside
/// the timer; lookups and all new face integrations remain inside it.
fn bench_new_edit_orientation(c: &mut Criterion) {
    use remus_check::properties::face_cache::FaceIntegralCache;
    use remus_math::vec::Point3;
    use remus_topology::{explorer::solid_faces, face::FaceSurface};

    let (mut topo, solid) = hammer_holder();
    let faces = solid_faces(&topo, solid).unwrap();
    let face = *faces.iter().find(|&&id| {
        matches!(topo.face(id).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.x() > 0.99)
            && (remus_operations::measure::face_area(&topo, id, 0.08).unwrap() - 1045.931).abs() < 0.01
    }).expect("holder cap");
    let reference = Point3::new(0.0, 0.0, 0.0);
    let mut source_cache = FaceIntegralCache::new();
    for &id in &faces {
        source_cache
            .integrate_face_volume_about(&topo, id, 5, reference)
            .unwrap();
    }
    let moved = remus_operations::push_pull::move_faces(&mut topo, solid, &[face], 5.02).unwrap();
    assert!(validate_solid(&topo, moved).unwrap().is_valid());
    let moved_faces = solid_faces(&topo, moved).unwrap();
    let mut group = c.benchmark_group("new_edit_orientation");
    group.sample_size(20);
    group.warm_up_time(std::time::Duration::from_millis(500));
    group.measurement_time(std::time::Duration::from_secs(2));
    group.bench_function("hammer_holder", |b| {
        b.iter_batched(
            || source_cache.clone(),
            |mut cache| {
                let mut signed = 0.0;
                for &id in &moved_faces {
                    signed += cache
                        .integrate_face_volume_about(&topo, id, 5, reference)
                        .unwrap()
                        .volume;
                }
                assert!(signed.is_finite() && signed > 0.0);
                std::hint::black_box(signed);
                cache
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();
}

criterion_group!(benches, bench_nurbs_properties, bench_new_edit_orientation);
criterion_main!(benches);
