//! Live blend-group edits on the filleted Gentle-Gecko part: one prepare,
//! one preview frame and one commit of the band-radius drag (A2, R4 → 1.5)
//! and the boss-diameter drag (A7, Ø12 → 14.6), beside the unprepared edit
//! each replaces.
//!
//! * `prepared_group_edit/{a2,a7}/unprepared`: `resize_blend` /
//!   `resize_cylindrical_face`, what a drag paid per frame before.
//! * `prepared_group_edit/{a2,a7}/prepare`: recognize, remove to sharp,
//!   learn the refusal in front of the group route.
//! * `prepared_group_edit/{a2,a7}/frame`: one `preview` (display-only
//!   re-blend under the frame gate).
//! * `prepared_group_edit/{a2,a7}/commit`: one `commit` (every guard).
//!
//! The measurement caches the WASM kernel enables on construction are on,
//! as in an OpenZCAD session. The cumulative body (A2, A3, A4, A6, then A7)
//! is timed by `examples/blend_group_edit_profile.rs`; building it takes
//! seconds per sample.
//!
//! Run locally: `cargo bench -p remus-operations --bench prepared_group_edit`

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

use remus_math::vec::Vec3;
use remus_operations::blend_cluster::{Convexity, PreparedGroupEdit, recognize_blend_cluster};
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::resize_blend::resize_blend;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn gecko() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let text = include_str!("../tests/data/gentle_gecko_filleted.step");
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .unwrap()
}

fn x0_band(topo: &Topology, solid: SolidId) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    cluster
        .members
        .iter()
        .filter(|m| m.convexity == Convexity::Convex)
        .flat_map(|m| m.faces.iter().copied())
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9)
        })
        .unwrap()
}

fn bench_prepared_group_edit(c: &mut Criterion) {
    remus_operations::measure::enable_thread_volume_memo();
    remus_operations::measure::enable_default_measurement_reuse();
    remus_operations::tessellate::enable_face_mesh_cache();

    let mut group = c.benchmark_group("prepared_group_edit");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(6));

    let (topo, solid) = gecko();
    let band = x0_band(&topo, solid);
    let boss = boss_of(&topo, solid);

    group.bench_function("a2/unprepared", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut topo| black_box(resize_blend(&mut topo, solid, band, 4.0, 1.5).unwrap()),
            BatchSize::LargeInput,
        );
    });
    group.bench_function("a2/prepare", |bencher| {
        bencher
            .iter(|| black_box(PreparedGroupEdit::prepare_band(&topo, solid, band, 4.0).unwrap()));
    });
    let a2 = PreparedGroupEdit::prepare_band(&topo, solid, band, 4.0).unwrap();
    group.bench_function("a2/frame", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut topo| black_box(a2.preview(&mut topo, 1.5).unwrap()),
            BatchSize::LargeInput,
        );
    });
    group.bench_function("a2/commit", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut topo| black_box(a2.commit(&mut topo, 1.5).unwrap()),
            BatchSize::LargeInput,
        );
    });

    group.bench_function("a7/unprepared", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut topo| black_box(resize_cylindrical_face(&mut topo, solid, boss, 7.3).unwrap()),
            BatchSize::LargeInput,
        );
    });
    group.bench_function("a7/prepare", |bencher| {
        bencher
            .iter(|| black_box(PreparedGroupEdit::prepare_cylinder(&topo, solid, boss).unwrap()));
    });
    let a7 = PreparedGroupEdit::prepare_cylinder(&topo, solid, boss).unwrap();
    group.bench_function("a7/frame", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut topo| black_box(a7.preview(&mut topo, 7.3).unwrap()),
            BatchSize::LargeInput,
        );
    });
    group.bench_function("a7/commit", |bencher| {
        bencher.iter_batched(
            || topo.clone(),
            |mut topo| black_box(a7.commit(&mut topo, 7.3).unwrap()),
            BatchSize::LargeInput,
        );
    });
    group.finish();
}

criterion_group!(benches, bench_prepared_group_edit);
criterion_main!(benches);
