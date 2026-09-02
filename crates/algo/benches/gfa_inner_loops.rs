#![allow(
    clippy::expect_used,
    clippy::too_many_lines,
    clippy::unwrap_used,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BatchSize, Criterion, Throughput, criterion_group, criterion_main};
use remus_algo::bop::BooleanOp;
use remus_algo::gfa;
use remus_math::curves::Circle3D;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::test_utils::make_unit_cube_manifold_at;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn make_cylinder(
    topology: &mut Topology,
    center_x: f64,
    center_y: f64,
    base_z: f64,
    radius: f64,
    height: f64,
) -> SolidId {
    let bottom_vertex = topology.add_vertex(Vertex::new(
        Point3::new(center_x + radius, center_y, base_z),
        1e-7,
    ));
    let top_vertex = topology.add_vertex(Vertex::new(
        Point3::new(center_x + radius, center_y, base_z + height),
        1e-7,
    ));
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let bottom_circle = Circle3D::new(Point3::new(center_x, center_y, base_z), axis, radius)
        .expect("benchmark bottom circle must be valid");
    let top_circle = Circle3D::new(
        Point3::new(center_x, center_y, base_z + height),
        axis,
        radius,
    )
    .expect("benchmark top circle must be valid");
    let surface = CylindricalSurface::new(Point3::new(center_x, center_y, base_z), axis, radius)
        .expect("benchmark cylinder surface must be valid");
    let bottom_seam = bottom_circle.project(Point3::new(center_x + radius, center_y, base_z));
    let top_seam = top_circle.project(Point3::new(center_x + radius, center_y, base_z + height));

    let mut bottom_rim = Edge::with_tolerance(
        bottom_vertex,
        bottom_vertex,
        EdgeCurve::Circle(bottom_circle),
        Some(1e-7),
    );
    bottom_rim.set_trim(Some((bottom_seam, bottom_seam + std::f64::consts::TAU)));
    let bottom_rim = topology.add_edge(bottom_rim);
    let mut top_rim = Edge::with_tolerance(
        top_vertex,
        top_vertex,
        EdgeCurve::Circle(top_circle),
        Some(1e-7),
    );
    top_rim.set_trim(Some((top_seam, top_seam + std::f64::consts::TAU)));
    let top_rim = topology.add_edge(top_rim);
    let seam = topology.add_edge(Edge::new(bottom_vertex, top_vertex, EdgeCurve::Line));

    let lateral_wire = topology.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(bottom_rim, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(top_rim, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .expect("benchmark lateral wire must be valid"),
    );
    let lateral = topology.add_face(Face::new(
        lateral_wire,
        vec![],
        FaceSurface::Cylinder(surface),
    ));

    let bottom_wire = topology.add_wire(
        Wire::new(vec![OrientedEdge::new(bottom_rim, false)], true)
            .expect("benchmark bottom wire must be valid"),
    );
    let bottom_cap = topology.add_face(Face::new(
        bottom_wire,
        vec![],
        FaceSurface::Plane {
            normal: -axis,
            d: -base_z,
        },
    ));
    let top_wire = topology.add_wire(
        Wire::new(vec![OrientedEdge::new(top_rim, true)], true)
            .expect("benchmark top wire must be valid"),
    );
    let top_cap = topology.add_face(Face::new(
        top_wire,
        vec![],
        FaceSurface::Plane {
            normal: axis,
            d: base_z + height,
        },
    ));

    let shell = topology.add_shell(
        Shell::new(vec![lateral, bottom_cap, top_cap])
            .expect("benchmark cylinder shell must be valid"),
    );
    topology.add_solid(Solid::new(shell, vec![]))
}

fn box_cylinder_fixture() -> (Topology, SolidId, SolidId) {
    let mut topology = Topology::new();
    let target = make_unit_cube_manifold_at(&mut topology, 0.0, 0.0, 0.0);
    let tool = make_cylinder(&mut topology, 0.5, 0.5, -0.5, 0.15, 2.0);
    (topology, target, tool)
}

fn overlapping_boxes_fixture() -> (Topology, SolidId, SolidId) {
    let mut topology = Topology::new();
    let first = make_unit_cube_manifold_at(&mut topology, 0.0, 0.0, 0.0);
    let second = make_unit_cube_manifold_at(&mut topology, 0.5, 0.5, 0.5);
    (topology, first, second)
}

fn bench_gfa(c: &mut Criterion) {
    let mut group = c.benchmark_group("gfa_phases");
    group.throughput(Throughput::Elements(1));
    group.bench_function("box_cylinder_cut", |bencher| {
        bencher.iter_batched(
            box_cylinder_fixture,
            |(mut topology, target, tool)| {
                black_box(
                    gfa::boolean(black_box(&mut topology), BooleanOp::Cut, target, tool)
                        .expect("box-cylinder GFA fixture must succeed"),
                )
            },
            BatchSize::SmallInput,
        );
    });
    group.bench_function("overlapping_boxes_fuse", |bencher| {
        bencher.iter_batched(
            overlapping_boxes_fixture,
            |(mut topology, first, second)| {
                black_box(
                    gfa::boolean(black_box(&mut topology), BooleanOp::Fuse, first, second)
                        .expect("box-box GFA fixture must succeed"),
                )
            },
            BatchSize::SmallInput,
        );
    });
    group.finish();
}

fn criterion_config() -> Criterion {
    Criterion::default()
        .sample_size(10)
        .warm_up_time(Duration::from_millis(300))
        .measurement_time(Duration::from_secs(1))
}

criterion_group! {
    name = benches;
    config = criterion_config();
    targets = bench_gfa
}
criterion_main!(benches);
