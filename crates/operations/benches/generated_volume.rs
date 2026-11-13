//! `solid_volume` of freshly generated bodies: a helical gear and an
//! external thread (NURBS flanks that are their carriers' whole parameter
//! rectangles), an in-place helical sweep of a circle (two caps and one
//! full-turn band) and a throated worm wheel (flanks cut at the throat). Building them takes milliseconds; measuring them is the
//! cost the geometry-generation probe
//! (`examples/geometry_generation_profile.rs`) found.
//!
//! Run with:
//!   `cargo bench -p remus-operations --bench generated_volume`

#![allow(clippy::unwrap_used, missing_docs)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};

use remus_math::vec::{Point3, Vec3};
use remus_operations::gear::{
    GearKind, GearParams, GearPlacement, ThroatedWheelOptions, WormParams, make_gear,
    make_throated_worm_wheel,
};
use remus_operations::helix::helical_sweep_in_place;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::thread::{CylinderThreadOptions, ThreadForm, ThreadKind, cut_thread};
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

fn config() -> Criterion {
    Criterion::default()
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(5))
}

fn helical_gear(topo: &mut Topology) -> SolidId {
    let params = GearParams {
        helix_angle: 20.0_f64.to_radians(),
        ..GearParams::standard(GearKind::External, 2.0, 20, 10.0)
    };
    make_gear(topo, &params, &GearPlacement::default())
        .unwrap()
        .solid
}

/// ISO M6×1 on an r 3 boss, 10 tall.
fn external_thread(topo: &mut Topology) -> SolidId {
    let blank = make_cylinder(topo, 3.0, 10.0).unwrap();
    let face = solid_faces(topo, blank)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let form = ThreadForm::iso_metric(1.0, ThreadKind::External);
    cut_thread(topo, blank, face, &form, &CylinderThreadOptions::default())
        .unwrap()
        .solid
}

/// The wheel of a module-2 single-start worm, 30 teeth, 14 wide, throated.
fn throated_wheel(topo: &mut Topology) -> SolidId {
    let worm = WormParams::standard(2.0, 1, 10.0, 20.0);
    make_throated_worm_wheel(
        topo,
        &worm,
        30,
        14.0,
        &ThroatedWheelOptions::default(),
        &GearPlacement::default(),
    )
    .unwrap()
    .gear
    .solid
}

/// A radius-1.5 wire centred 12 from the Z axis, pitch 5, five turns.
fn coil(topo: &mut Topology) -> SolidId {
    let center = Point3::new(12.0, 0.0, 0.0);
    let normal = Vec3::new(0.0, 1.0, 0.0);
    let edge = make_circle_edge(topo, center, normal, 1.5, 1e-7).unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: normal.dot(Vec3::new(center.x(), center.y(), center.z())),
        },
    ));
    let origin = Point3::new(0.0, 0.0, 0.0);
    helical_sweep_in_place(topo, profile, origin, Vec3::new(0.0, 0.0, 1.0), 5.0, 5.0).unwrap()
}

/// A body generator.
type Build = fn(&mut Topology) -> SolidId;

fn bench_generated_volume(c: &mut Criterion) {
    let bodies: [(&str, Build, f64); 4] = [
        ("volume helical gear z20", helical_gear, 0.05),
        ("volume M6 external thread", external_thread, 0.01),
        ("volume in-place coil 5 turns", coil, 0.05),
        ("volume throated worm wheel z30", throated_wheel, 0.05),
    ];
    for (name, build, deflection) in bodies {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        c.bench_function(name, |b| {
            b.iter(|| black_box(solid_volume(black_box(&topo), solid, deflection).unwrap()));
        });
    }
}

criterion_group! {
    name = generated_volume;
    config = config();
    targets = bench_generated_volume
}
criterion_main!(generated_volume);
