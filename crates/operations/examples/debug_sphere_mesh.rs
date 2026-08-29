//! Throwaway probe: inspect sphere-sphere boolean result meshes.
#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs
)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;

fn main() {
    env_logger::builder().format_timestamp(None).init();
    dump_intersect();
    for (op, dx) in [(BooleanOp::Fuse, 1.0), (BooleanOp::Cut, 1.0)] {
        let mut topo = Topology::new();
        let a = primitives::make_sphere(&mut topo, 1.0, 32).unwrap();
        let b = primitives::make_sphere(&mut topo, 1.0, 32).unwrap();
        transform_solid(&mut topo, b, &Mat4::translation(dx, 0.0, 0.0)).unwrap();
        let Ok(sid) = boolean(&mut topo, op, a, b) else {
            println!("{op:?}: failed");
            continue;
        };
        let mesh = remus_operations::tessellate::tessellate_solid(&topo, sid, 0.001).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, sid).unwrap();
        println!(
            "{op:?}: faces={} tris={}",
            faces.len(),
            mesh.indices.len() / 3
        );
        // Area + AABB per face via per-face tessellation.
        for f in &faces {
            let rev = topo.face(*f).unwrap().is_reversed();
            let fm = remus_operations::tessellate::tessellate(&topo, *f, 0.001).unwrap();
            let mut area = 0.0;
            let mut aabb_min = [f64::MAX; 3];
            let mut aabb_max = [f64::MIN; 3];
            for t in 0..fm.indices.len() / 3 {
                let (p0, p1, p2) = (
                    fm.positions[fm.indices[t * 3] as usize],
                    fm.positions[fm.indices[t * 3 + 1] as usize],
                    fm.positions[fm.indices[t * 3 + 2] as usize],
                );
                let e1 = p1 - p0;
                let e2 = p2 - p0;
                area += e1.cross(e2).length() * 0.5;
                for p in [p0, p1, p2] {
                    for k in 0..3 {
                        aabb_min[k] = aabb_min[k].min([p.x(), p.y(), p.z()][k]);
                        aabb_max[k] = aabb_max[k].max([p.x(), p.y(), p.z()][k]);
                    }
                }
            }
            let mut signed = 0.0;
            for t in 0..fm.indices.len() / 3 {
                let (p0, p1, p2) = (
                    fm.positions[fm.indices[t * 3] as usize],
                    fm.positions[fm.indices[t * 3 + 1] as usize],
                    fm.positions[fm.indices[t * 3 + 2] as usize],
                );
                let v0 = remus_math::vec::Vec3::new(p0.x(), p0.y(), p0.z());
                let v1 = remus_math::vec::Vec3::new(p1.x(), p1.y(), p1.z());
                let v2 = remus_math::vec::Vec3::new(p2.x(), p2.y(), p2.z());
                signed += v0.dot(v1.cross(v2));
            }
            println!(
                "  face {f:?}: reversed={rev} tris={} area={area:.4} signed_div={signed:.4} aabb=[{aabb_min:.3?} .. {aabb_max:.3?}]",
                fm.indices.len() / 3,
            );
        }
        let solid_mesh = remus_operations::tessellate::tessellate_solid(&topo, sid, 0.001).unwrap();
        let mut sv = 0.0;
        for t in 0..solid_mesh.indices.len() / 3 {
            let (p0, p1, p2) = (
                solid_mesh.positions[solid_mesh.indices[t * 3] as usize],
                solid_mesh.positions[solid_mesh.indices[t * 3 + 1] as usize],
                solid_mesh.positions[solid_mesh.indices[t * 3 + 2] as usize],
            );
            let v0 = remus_math::vec::Vec3::new(p0.x(), p0.y(), p0.z());
            let v1 = remus_math::vec::Vec3::new(p1.x(), p1.y(), p1.z());
            let v2 = remus_math::vec::Vec3::new(p2.x(), p2.y(), p2.z());
            sv += v0.dot(v1.cross(v2));
        }
        println!(
            "  whole-solid mesh: tris={} signed_vol/6={:.6} boundary_edges={}",
            solid_mesh.indices.len() / 3,
            sv / 6.0,
            {
                use std::collections::HashMap;
                let mut counts: HashMap<(u32, u32), usize> = HashMap::new();
                for tri in solid_mesh.indices.chunks_exact(3) {
                    for &(i, j) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                        let key = if i < j { (i, j) } else { (j, i) };
                        *counts.entry(key).or_insert(0) += 1;
                    }
                }
                counts.values().filter(|&&c| c != 2).count()
            }
        );
        // classify probes
        for (label, pt) in [
            (
                "A collar back",
                remus_math::vec::Point3::new(-0.5, 0.0, 0.0),
            ),
            ("lens center", remus_math::vec::Point3::new(0.5, 0.0, 0.0)),
            ("A pole", remus_math::vec::Point3::new(0.0, 0.0, 0.9)),
            ("B far side", remus_math::vec::Point3::new(1.8, 0.0, 0.0)),
        ] {
            let c = remus_operations::classify::classify_point(&topo, sid, pt, 0.001, 1e-7);
            println!("  probe {label} {pt:?}: {c:?}");
        }
    }
}

#[allow(clippy::format_in_format_args, clippy::print_stdout)]
fn dump_intersect() {
    use remus_topology::edge::EdgeCurve;
    let mut topo = Topology::new();
    let a = primitives::make_sphere(&mut topo, 1.0, 32).unwrap();
    let b = primitives::make_sphere(&mut topo, 1.0, 32).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(1.0, 0.0, 0.0)).unwrap();
    let gfa_op = remus_algo::bop::BooleanOp::Intersect;
    let Ok(sid) = remus_algo::gfa::boolean_with_context(
        &mut topo,
        gfa_op,
        a,
        b,
        &remus_math::context::OperationContext::new(),
    ) else {
        println!("dump_intersect: GFA failed");
        return;
    };
    let solid = topo.solid(sid).unwrap();
    let mut shells = vec![("outer", solid.outer_shell())];
    for (i, s) in solid.inner_shells().iter().enumerate() {
        shells.push((format!("inner{i}").leak(), *s));
    }
    for (label, sh) in shells {
        let shell = topo.shell(sh).unwrap();
        println!("shell {label}: {} faces", shell.faces().len());
        for &fid in shell.faces() {
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            println!("  face {fid:?} reversed={} edges:", face.is_reversed());
            for oe in wire.edges() {
                let e = topo.edge(oe.edge()).unwrap();
                let sp = topo.vertex(e.start()).unwrap().point();
                let ep = topo.vertex(e.end()).unwrap().point();
                let tag = match e.curve() {
                    EdgeCurve::Circle(c) => {
                        format!(
                            "Circle c=({:.3},{:.3},{:.3})",
                            c.center().x(),
                            c.center().y(),
                            c.center().z()
                        )
                    }
                    _ => "?".to_string(),
                };
                println!(
                    "    edge {:?} fwd={} ({:.4},{:.4},{:.4})->({:.4},{:.4},{:.4}) {}",
                    oe.edge(),
                    oe.is_forward(),
                    sp.x(),
                    sp.y(),
                    sp.z(),
                    ep.x(),
                    ep.y(),
                    ep.z(),
                    tag
                );
            }
        }
    }
}
