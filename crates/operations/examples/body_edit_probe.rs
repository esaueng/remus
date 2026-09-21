//! Exact body editing versus unrelated arena size. Run in release mode:
//! `cargo run --release -p remus-operations --example body_edit_probe`.
//! Setup, validation, and cleanup are excluded from operation/mesh timings.
#![allow(clippy::print_stdout, clippy::unwrap_used, clippy::expect_used)]

use std::time::Instant;

use remus_math::vec::Vec3;
use remus_operations::{extrude, measure, primitives, push_pull, tessellate, validate};
use remus_topology::{Topology, explorer::solid_faces};

fn percentile(values: &mut [f64], percent: usize) -> f64 {
    values.sort_by(f64::total_cmp);
    values[(values.len() * percent).div_ceil(100) - 1]
}

fn main() {
    println!("background_bodies,operation,edit_p50_ms,edit_p95_ms,mesh_p50_ms,mesh_p95_ms");
    for background in [0, 1000, 10_000] {
        for operation in ["extrude", "moveFaces", "pushPullFace"] {
            let mut edits = Vec::new();
            let mut meshes = Vec::new();
            for iteration in 0..35 {
                let mut topo = Topology::new();
                for _ in 0..background {
                    primitives::make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();
                }
                let body = primitives::make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();
                let face = solid_faces(&topo, body)
                    .unwrap()
                    .into_iter()
                    .find(|&face| {
                        topo.face(face)
                            .unwrap()
                            .effective_plane_normal()
                            .is_some_and(|normal| normal.z() > 0.99)
                    })
                    .expect("top face");
                let start = Instant::now();
                let result = match operation {
                    "extrude" => extrude::extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 5.0),
                    "moveFaces" => push_pull::move_faces(&mut topo, body, &[face], 5.0),
                    _ => push_pull::push_pull_face(&mut topo, body, face, 5.0),
                }
                .unwrap();
                let edited = Instant::now();
                let mesh =
                    tessellate::tessellate_solid_with_tolerance(&topo, result, 0.1, 0.5).unwrap();
                let meshed = Instant::now();
                let expected = if operation == "extrude" {
                    1000.0
                } else {
                    7000.0
                };
                assert!(
                    (measure::solid_volume(&topo, result, 0.1).unwrap() - expected).abs() < 1e-6
                );
                assert!((measure::solid_volume(&topo, body, 0.1).unwrap() - 6000.0).abs() < 1e-6);
                assert!(validate::validate_solid(&topo, result).unwrap().is_valid());
                assert!(!mesh.indices.is_empty());
                if iteration >= 5 {
                    edits.push(edited.duration_since(start).as_secs_f64() * 1000.0);
                    meshes.push(meshed.duration_since(edited).as_secs_f64() * 1000.0);
                }
            }
            println!(
                "{background},{operation},{:.6},{:.6},{:.6},{:.6}",
                percentile(&mut edits, 50),
                percentile(&mut edits, 95),
                percentile(&mut meshes, 50),
                percentile(&mut meshes, 95)
            );
        }
    }
}
