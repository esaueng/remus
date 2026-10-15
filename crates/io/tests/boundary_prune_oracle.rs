//! Differential oracles for the prepared classifier's exact skips, over the
//! imported STEP fixtures and synthetic bodies:
//!
//! * `PreparedSolid::is_point_on_boundary` with the trim-window prune against
//!   the unpruned loop, on points on and within a few tolerances of every face
//!   (seams, sphere caps, NURBS faces included);
//! * `PreparedSolid::count_ray_crossings` with the NURBS knot-span prefilter
//!   against the unfiltered count, from the same kind of points along the
//!   classifier's vote directions (the dragged hammer holder and boxes with a
//!   dense NURBS dome on either side of the prefilter's span cap included),
//!   with the faces the prefilter skips counted so the oracle knows it fires.

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
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::fillet::fillet_rolling_ball;
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::tessellate_solid;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{face_vertices, solid_edges, solid_faces, solid_vertices};
use remus_topology::face::FaceSurface;
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

/// The hammer holder after the face drag `kernel_perf_probe` analyses: its
/// +X arm face moved out by 5.
fn hammer_edit() -> Body {
    let text = std::fs::read_to_string(data_dir().join("shapr3d_hammer_holder.step")).unwrap();
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(&text, &mut topo).unwrap()[0];
    let face = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.x() > 0.999)
                && face_vertices(&topo, face)
                    .unwrap()
                    .iter()
                    .all(|&v| (topo.vertex(v).unwrap().point().x() - 48.0).abs() < 1e-7)
        })
        .unwrap();
    let edited = move_faces(&mut topo, solid, &[face], 5.0).unwrap();
    ("hammer edit".to_string(), topo, edited)
}

/// A 20 x 20 x 10 box whose top is a bicubic NURBS dome of height 6 with
/// `k x k` uniform knot spans; its boundary rows stay on the box's top edges.
fn dome_box(k: usize) -> Body {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let top = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.999)
        })
        .unwrap();
    let n = k + 3;
    let mut knots = vec![0.0; 4];
    knots.extend((1..k).map(|i| i as f64 / k as f64));
    knots.extend([1.0; 4]);
    let control = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let (s, t) = (i as f64 / (n - 1) as f64, j as f64 / (n - 1) as f64);
                    let bump = (std::f64::consts::PI * s).sin() * (std::f64::consts::PI * t).sin();
                    Point3::new(20.0 * s, 20.0 * t, 6.0_f64.mul_add(bump, 10.0))
                })
                .collect()
        })
        .collect();
    let dome =
        NurbsSurface::new(3, 3, knots.clone(), knots, control, vec![vec![1.0; n]; n]).unwrap();
    topo.face_mut(top)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(dome));
    (format!("dome {k}x{k}"), topo, solid)
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

/// The NURBS knot-span prefilter never changes a crossing count.
#[test]
fn nurbs_ray_prefilter_matches_unfiltered_counts() {
    let directions = [
        Vec3::new(
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
            0.350_889_803_483_932_2,
        ),
        Vec3::new(
            -0.350_889_803_483_932_2,
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
        ),
        Vec3::new(
            0.740_535_693_464_567_5,
            -0.350_889_803_483_932_2,
            0.573_576_436_351_046,
        ),
        Vec3::new(-0.6, 0.0, -0.8),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let mut bodies = step_bodies();
    bodies.extend(synthetic_bodies());
    bodies.push(hammer_edit());
    // 400 spans is the prefilter's cap.
    bodies.push(dome_box(20));
    bodies.push(dome_box(21));
    let (mut rays, mut skips) = (0usize, 0usize);
    for (name, topo, solid) in &bodies {
        let prepared = PreparedSolid::prepare(topo, *solid).unwrap();
        let mut probes: Vec<(Point3, Vec3)> = Vec::new();
        if name.starts_with("dome") {
            // Every corpus ray that reaches the dome starts on it or hits it;
            // these level rays cross its box over the low rim instead. They
            // come first: after a run of hits the prefilter tests only one
            // ray in 32 until its skips catch up.
            for a in 0..32 {
                let y = if a < 16 {
                    0.1 * f64::from(a)
                } else {
                    20.0 - 0.1 * f64::from(a - 16)
                };
                for c in 0..16 {
                    let z = 10.5 + 0.35 * f64::from(c);
                    probes.push((Point3::new(-5.0, y, z), Vec3::new(1.0, 0.0, 0.0)));
                    probes.push((Point3::new(y, 25.0, z), Vec3::new(0.0, -1.0, 0.0)));
                }
            }
        }
        probes.extend(
            boundary_corpus(topo, *solid, 1.0e-3)
                .into_iter()
                .step_by(3)
                .flat_map(|point| directions.map(|direction| (point, direction))),
        );
        let (mut body_rays, mut body_skips) = (0usize, 0usize);
        for (point, direction) in probes {
            let filtered = prepared.count_ray_crossings_and_skips(point, direction);
            let reference = prepared.count_ray_crossings_unfiltered(point, direction);
            assert_eq!(
                format!("{:?}", filtered.as_ref().map(|&(crossings, _)| crossings)),
                format!("{reference:?}"),
                "{name} from {point:?} along {direction:?}"
            );
            body_skips += filtered.map_or(0, |(_, skips)| skips);
            body_rays += 1;
        }
        eprintln!("{name}: {body_rays} rays, {body_skips} NURBS faces skipped");
        match name.as_str() {
            "dome 20x20" => assert!(body_skips * 10 > body_rays, "{name}: the skip must fire"),
            "dome 21x21" => assert_eq!(body_skips, 0, "{name}: past the span cap"),
            _ => {}
        }
        rays += body_rays;
        skips += body_skips;
    }
    eprintln!("NURBS ray prefilter: {rays} rays compared, {skips} NURBS faces skipped");
    assert!(rays > 100_000, "only {rays} rays compared");
    assert!(
        skips > 5_000,
        "only {skips} NURBS faces skipped on {rays} rays"
    );
}
