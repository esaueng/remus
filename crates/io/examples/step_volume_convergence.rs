#![allow(clippy::print_stdout, clippy::expect_used, missing_docs)]
//! K0.1 spline-accuracy probe: read a STEP file whose fillet bands arrived as
//! non-rational B-splines, and report where the measured volume comes from —
//! surface mix, recognition verdicts on each NURBS face, and the tessellated
//! volume's convergence across deflections. Compares against a closed-form
//! target when one is passed as the second argument.

use remus_io::step::reader::read_step;
use remus_operations::measure;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

fn mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    use remus_math::vec::Vec3;
    let mut v = 0.0;
    for tri in mesh.indices.chunks(3) {
        let p0 = mesh.positions[tri[0] as usize];
        let p1 = mesh.positions[tri[1] as usize];
        let p2 = mesh.positions[tri[2] as usize];
        let a = Vec3::new(p0.x(), p0.y(), p0.z());
        let b = Vec3::new(p1.x(), p1.y(), p1.z());
        let c = Vec3::new(p2.x(), p2.y(), p2.z());
        v += a.dot(b.cross(c)) / 6.0;
    }
    v
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: k01_volume_probe <file.step> [closed_form_volume]");
    let target: Option<f64> = std::env::args().nth(2).and_then(|s| s.parse().ok());
    let text = std::fs::read_to_string(&path).expect("read");

    let mut topo = Topology::new();
    let solids = read_step(&text, &mut topo).expect("read_step");
    println!("{path}: {} solid(s)", solids.len());

    for &sid in &solids {
        let faces = solid_faces(&topo, sid).expect("faces");
        println!("solid: {} faces", faces.len());
        for &fid in &faces {
            let face = topo.face(fid).expect("face");
            if let FaceSurface::Nurbs(surf) = face.surface() {
                let net = surf.control_points();
                println!(
                    "  NURBS face {:?}: deg ({},{}), net {}x{}, rational={}",
                    fid,
                    surf.degree_u(),
                    surf.degree_v(),
                    net.len(),
                    net.first().map_or(0, Vec::len),
                    surf.is_rational(),
                );
            }
        }

        let v_measure = measure::solid_volume(&topo, sid, 0.01).expect("solid_volume");
        println!("  measure::solid_volume(defl=0.01) = {v_measure:.6}");
        for defl in [0.1, 0.01, 0.001, 0.0005, 0.0001] {
            let mesh = tessellate_solid_with_tolerance(&topo, sid, defl, 0.5).expect("mesh");
            let v = mesh_volume(&mesh);
            let delta = target.map(|t| format!("  ({:+.4}% vs target)", (v - t) / t * 100.0));
            println!(
                "  mesh volume @ defl {defl}: {v:.6} ({} tris){}",
                mesh.indices.len() / 3,
                delta.unwrap_or_default()
            );
        }
        if let Some(t) = target {
            println!(
                "  target {t:.6}; measure delta {:+.4}%",
                (v_measure - t) / t * 100.0
            );
        }
    }
}
