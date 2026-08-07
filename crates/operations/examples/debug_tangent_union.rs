#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::items_after_statements,
    missing_docs
)]

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure;
use brepkit_operations::primitives;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;

fn census(topo: &Topology, solid: brepkit_topology::solid::SolidId) -> String {
    use std::collections::BTreeMap;
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let faces = brepkit_topology::explorer::solid_faces(topo, solid).unwrap();
    for fid in &faces {
        let tag = match topo.face(*fid).unwrap().surface() {
            brepkit_topology::face::FaceSurface::Plane { .. } => "plane",
            brepkit_topology::face::FaceSurface::Nurbs(_) => "nurbs",
            brepkit_topology::face::FaceSurface::Cylinder(_) => "cylinder",
            brepkit_topology::face::FaceSurface::Cone(_) => "cone",
            brepkit_topology::face::FaceSurface::Sphere(_) => "sphere",
            brepkit_topology::face::FaceSurface::Torus(_) => "torus",
        };
        *counts.entry(tag).or_default() += 1;
    }
    format!("{} faces {:?}", faces.len(), counts)
}

fn edge_usage(topo: &Topology, solid: brepkit_topology::solid::SolidId) -> (usize, usize) {
    use std::collections::HashMap;
    let mut usage: HashMap<brepkit_topology::edge::EdgeId, usize> = HashMap::new();
    for fid in brepkit_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        let mut wires = vec![face.outer_wire()];
        wires.extend(face.inner_wires().iter().copied());
        for wid in wires {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge()).or_default() += 1;
            }
        }
    }
    let free = usage.values().filter(|&&c| c == 1).count();
    let nonman = usage.values().filter(|&&c| c > 2).count();
    (free, nonman)
}

fn run_case(name: &str, cx: f64, cy: f64, cz: f64, r: f64, h: f64) {
    let mut topo = Topology::new();
    let bx = primitives::make_box(&mut topo, 60.0, 40.0, 40.0).unwrap();
    let cyl = primitives::make_cylinder(&mut topo, r, h).unwrap();
    let mat = Mat4::translation(cx, cy, cz);
    transform_solid(&mut topo, cyl, &mat).unwrap();

    match boolean(&mut topo, BooleanOp::Fuse, bx, cyl) {
        Ok(result) => {
            let (free, nonman) = edge_usage(&topo, result);
            let vol = measure::solid_volume(&topo, result, 0.1).unwrap_or(-1.0);
            // Ray-cast classification: box interior, cylinder interior, and a
            // point outside both must classify correctly after the fuse.
            use brepkit_check::classify::{ClassifyOptions, classify_point};
            let probes = [
                ("box-in", brepkit_math::vec::Point3::new(30.0, 20.0, 20.0)),
                (
                    "cyl-in",
                    brepkit_math::vec::Point3::new(cx, cy, cz + h * 0.5),
                ),
                ("out", brepkit_math::vec::Point3::new(-30.0, -30.0, 20.0)),
            ];
            let cls: Vec<String> = probes
                .iter()
                .map(|(tag, p)| {
                    let c = classify_point(&topo, result, *p, &ClassifyOptions::default());
                    format!("{tag}={c:?}")
                })
                .collect();
            println!(
                "{name}: OK  {}  free_edges={free} nonmanifold_edges={nonman} vol={vol:.2}  {}",
                census(&topo, result),
                cls.join(" ")
            );
            if let Ok(report) = brepkit_operations::validate::validate_solid(&topo, result) {
                for issue in &report.issues {
                    println!("    validate: [{:?}] {}", issue.severity, issue.description);
                }
            }
        }
        Err(e) => println!("{name}: ERR {e}"),
    }
}

fn main() {
    env_logger::init();
    let r = 8.0;
    // Box: x in [0,60], y in [0,40], z in [0,40]. Left face plane x=0.
    // Cylinder axis vertical at (cx, cy), z from cz to cz+h.

    if std::env::var("ONLY_TALLER").is_ok() {
        run_case("tangent-taller     ", -r, 20.0, -5.0, r, 55.0);
        return;
    }

    if std::env::var("ONLY_QUARTER").is_ok() {
        // Mirror of boolean::tests::box_cylinder_fuse_returns_manifold_result
        let mut topo = Topology::new();
        let bx = primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let cyl = primitives::make_cylinder(&mut topo, 0.5, 2.0).unwrap();
        match boolean(&mut topo, BooleanOp::Fuse, bx, cyl) {
            Ok(result) => {
                let (free, nonman) = edge_usage(&topo, result);
                let vol = measure::solid_volume(&topo, result, 0.01).unwrap_or(-1.0);
                let expected = 8.0 + 0.75 * std::f64::consts::PI * 0.25 * 2.0;
                println!(
                    "quarter: OK {} free={free} nonman={nonman} vol={vol:.6} expected={expected:.6}",
                    census(&topo, result)
                );
            }
            Err(e) => println!("quarter: ERR {e}"),
        }
        return;
    }

    // 1) Exact external tangency to left face: axis at x = -r
    run_case("tangent-exact      ", -r, 20.0, 0.0, r, 40.0);
    // 2) Tangent, cylinder taller than box (like the screenshot)
    run_case("tangent-taller     ", -r, 20.0, -5.0, r, 55.0);
    // 3) Slight overlap (0.5)
    run_case("overlap-0.5        ", -r + 0.5, 20.0, 0.0, r, 40.0);
    // 4) Tiny overlap (1e-3)
    run_case("overlap-1e-3       ", -r + 1e-3, 20.0, 0.0, r, 40.0);
    // 5) Tiny gap (1e-3)
    run_case("gap-1e-3           ", -r - 1e-3, 20.0, 0.0, r, 40.0);
    // 6) Tangent near-ish (1e-9 overlap)
    run_case("overlap-1e-9       ", -r + 1e-9, 20.0, 0.0, r, 40.0);
    // 7) Tangent to a vertical edge of the box (axis on the corner diagonal)
    let d = r / std::f64::consts::SQRT_2;
    run_case("corner-tangent     ", -d, -d, 0.0, r, 40.0);
    // 8) Halfway overlap for sanity
    run_case("overlap-half       ", 0.0, 20.0, 0.0, r, 40.0);
}
