//! Throwaway probe: non-concentric sphere-sphere booleans.
#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs
)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn main() {
    env_logger::builder().format_timestamp(None).init();
    let _ = measure::solid_volume;
    for (op, dx) in [
        (BooleanOp::Fuse, 1.0),
        (BooleanOp::Cut, 1.0),
        (BooleanOp::Intersect, 1.0),
        (BooleanOp::Fuse, 0.4),
        (BooleanOp::Cut, 0.4),
        (BooleanOp::Intersect, 0.4),
        (BooleanOp::Fuse, 1.7),
        (BooleanOp::Cut, 1.7),
        (BooleanOp::Intersect, 1.7),
        (BooleanOp::Fuse, 0.3),
        (BooleanOp::Intersect, 0.3),
    ] {
        let mut topo = Topology::new();
        let a = primitives::make_sphere(&mut topo, 1.0, 32).unwrap();
        let b = primitives::make_sphere(&mut topo, 1.0, 32).unwrap();
        transform_solid(&mut topo, b, &Mat4::translation(dx, 0.0, 0.0)).unwrap();
        let r = boolean(&mut topo, op, a, b);
        match r {
            Ok(sid) => {
                let faces = remus_topology::explorer::solid_faces(&topo, sid).unwrap();
                let mut types = std::collections::BTreeMap::new();
                for f in &faces {
                    let t = match topo.face(*f).unwrap().surface() {
                        remus_topology::face::FaceSurface::Plane { .. } => "Plane",
                        remus_topology::face::FaceSurface::Nurbs(_) => "Nurbs",
                        remus_topology::face::FaceSurface::Cylinder(_) => "Cyl",
                        remus_topology::face::FaceSurface::Cone(_) => "Cone",
                        remus_topology::face::FaceSurface::Sphere(_) => "Sphere",
                        remus_topology::face::FaceSurface::Torus(_) => "Torus",
                    };
                    *types.entry(t).or_insert(0usize) += 1;
                }
                print!("{op:?} dx={dx}: faces={} types={types:?}", faces.len());
                for d in [0.0001_f64] {
                    let vol = measure::solid_volume(&topo, sid, d).unwrap();
                    print!(" vol@{d}={vol:.6}");
                }
                let area = measure::solid_surface_area(&topo, sid, 0.001).unwrap();
                print!(" area={area:.6}");
                println!();
            }
            Err(e) => println!("{op:?} dx={dx}: ERR {e:?}"),
        }
    }
}
