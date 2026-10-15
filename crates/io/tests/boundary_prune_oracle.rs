//! Differential oracle for the prepared boundary test's trim-window prune, over the
//! imported STEP fixtures and synthetic bodies: `PreparedSolid::is_point_on_boundary`
//! with the trim-window prune against the unpruned loop, on points on and
//! within a few tolerances of every face (seams, sphere caps, NURBS faces
//! included).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr,
    clippy::cast_precision_loss,
    deprecated
)]

use std::path::Path;

use remus_check::classify::PreparedSolid;
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::fillet::fillet_rolling_ball;
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::tessellate::tessellate_solid;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_vertices};
use remus_topology::solid::SolidId;

type Body = (String, Topology, SolidId);

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// Every solid of every STEP fixture.
fn step_bodies() -> Vec<Body> {
    let mut files: Vec<_> = std::fs::read_dir(data_dir())
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .filter(|name| {
            Path::new(name)
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("step"))
        })
        .collect();
    files.sort();
    let mut bodies = Vec::new();
    for file in files {
        let text = std::fs::read_to_string(data_dir().join(&file)).unwrap();
        let mut topo = Topology::new();
        let Ok(solids) = remus_io::step::reader::read_step(&text, &mut topo) else {
            continue;
        };
        for (k, solid) in solids.into_iter().enumerate() {
            bodies.push((format!("{file}#{k}"), topo.clone(), solid));
        }
    }
    assert!(
        bodies.len() >= 25,
        "fixture corpus shrank: {}",
        bodies.len()
    );
    bodies
}

fn translated(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

/// The synthetic bodies of `perf_o06_recognition_golden` and
/// `qualify_feature_recognition`.
fn synthetic_bodies() -> Vec<Body> {
    let mut bodies = Vec::new();
    let mut push = |name: &str, build: &dyn Fn(&mut Topology) -> SolidId| {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        bodies.push((name.to_string(), topo, solid));
    };
    push("box", &|t| make_box(t, 2.0, 1.5, 1.0).unwrap());
    push("cylinder", &|t| make_cylinder(t, 1.0, 2.0).unwrap());
    push("cone", &|t| make_cone(t, 2.0, 0.5, 3.0).unwrap());
    push("sphere", &|t| make_sphere(t, 2.0, 16).unwrap());
    push("torus", &|t| make_torus(t, 3.0, 1.0, 16).unwrap());
    push("two_hole_plate", &|t| {
        let cube = make_box(t, 3.0, 2.0, 1.0).unwrap();
        let d1 = make_cylinder(t, 0.2, 2.0).unwrap();
        translated(t, d1, 0.7, 1.0, -0.5);
        let d2 = make_cylinder(t, 0.2, 2.0).unwrap();
        translated(t, d2, 2.3, 1.0, -0.5);
        let b1 = boolean(t, BooleanOp::Cut, cube, d1).unwrap();
        boolean(t, BooleanOp::Cut, b1, d2).unwrap()
    });
    push("through_hole", &|t| {
        let cube = make_box(t, 2.0, 2.0, 1.0).unwrap();
        let drill = make_cylinder(t, 0.3, 2.0).unwrap();
        translated(t, drill, 1.0, 1.0, -0.5);
        boolean(t, BooleanOp::Cut, cube, drill).unwrap()
    });
    push("blind_hole", &|t| {
        let cube = make_box(t, 2.0, 2.0, 1.0).unwrap();
        let drill = make_cylinder(t, 0.3, 0.6).unwrap();
        translated(t, drill, 1.0, 1.0, 0.4);
        boolean(t, BooleanOp::Cut, cube, drill).unwrap()
    });
    push("pocketed_box", &|t| {
        let cube = make_box(t, 2.0, 2.0, 1.0).unwrap();
        let tool = make_box(t, 0.6, 0.8, 0.5).unwrap();
        translated(t, tool, 0.7, 0.6, 0.5);
        boolean(t, BooleanOp::Cut, cube, tool).unwrap()
    });
    push("blind_hole_plate", &|t| {
        let plate = make_box(t, 20.0, 20.0, 6.0).unwrap();
        let drill = make_cylinder(t, 3.0, 4.0).unwrap();
        translated(t, drill, 10.0, 10.0, 2.0);
        boolean(t, BooleanOp::Cut, plate, drill).unwrap()
    });
    push("boss", &|t| {
        let plate = make_box(t, 10.0, 10.0, 2.0).unwrap();
        let post = make_cylinder(t, 1.5, 3.0).unwrap();
        translated(t, post, 5.0, 5.0, 1.0);
        boolean(t, BooleanOp::Fuse, plate, post).unwrap()
    });
    push("chamfered_box", &|t| {
        let cube = make_box(t, 1.0, 1.0, 1.0).unwrap();
        let target = solid_edges(t, cube)
            .unwrap()
            .into_iter()
            .find(|&edge| {
                let e = t.edge(edge).unwrap();
                let on = |p: Point3| (p.x() - 1.0).abs() < 1e-9 && (p.z() - 1.0).abs() < 1e-9;
                on(t.vertex(e.start()).unwrap().point()) && on(t.vertex(e.end()).unwrap().point())
            })
            .unwrap();
        chamfer(t, cube, &[target], 0.15).unwrap()
    });
    push("filleted_box", &|t| {
        let cube = make_box(t, 10.0, 10.0, 10.0).unwrap();
        let edge = solid_edges(t, cube).unwrap()[0];
        fillet_rolling_ball(t, cube, &[edge], 1.0).unwrap()
    });
    bodies
}

/// Points on and around every face: tessellation vertices (on faces, seams,
/// sphere caps, rims) offset along their normals by fractions and multiples
/// of `tolerance`, every vertex likewise, and the edge quadrant probes.
fn boundary_corpus(topo: &Topology, solid: SolidId, tolerance: f64) -> Vec<Point3> {
    let mut points = Vec::new();
    if let Ok(mesh) = tessellate_solid(topo, solid, 0.05) {
        for (position, normal) in mesh.positions.iter().zip(&mesh.normals).step_by(5) {
            points.push(*position);
            for k in [-3.0, -1.5, -0.5, 0.5, 0.99, 1.01, 1.5, 3.0] {
                points.push(*position + *normal * (k * tolerance));
            }
        }
    }
    for vertex in solid_vertices(topo, solid).unwrap() {
        let p = topo.vertex(vertex).unwrap().point();
        points.push(p);
        for offset in [0.5, 2.0, 50.0] {
            let d = offset * tolerance;
            points.push(Point3::new(p.x() + d, p.y() - d, p.z() + d));
        }
    }
    points
}

/// The trim-window prune never changes a boundary verdict.
#[test]
fn pruned_boundary_test_matches_unpruned_loop() {
    let mut bodies = step_bodies();
    bodies.extend(synthetic_bodies());
    let mut on_boundary = 0usize;
    let mut checked = 0usize;
    for (name, topo, solid) in &bodies {
        let prepared = PreparedSolid::prepare(topo, *solid).unwrap();
        for tolerance in [1.0e-7, 1.0e-6, 1.0e-3] {
            for point in boundary_corpus(topo, *solid, tolerance) {
                let pruned = prepared.is_point_on_boundary(point, tolerance);
                let reference = prepared.is_point_on_boundary_unpruned(point, tolerance);
                assert_eq!(
                    format!("{pruned:?}"),
                    format!("{reference:?}"),
                    "{name} tol {tolerance} at {point:?}"
                );
                checked += 1;
                on_boundary += usize::from(matches!(reference, Ok(true)));
            }
        }
    }
    eprintln!("trim-window prune: {checked} points, {on_boundary} on the boundary");
    assert!(
        on_boundary * 10 > checked,
        "the corpus must reach the boundary"
    );
}
