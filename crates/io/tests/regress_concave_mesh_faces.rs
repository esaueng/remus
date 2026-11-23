//! An extrusion of a U with a 2-by-3 notch: perimeter 22, cap area 10.
#![allow(clippy::unwrap_used)]
use remus_operations::measure::{solid_surface_area, solid_volume};
use remus_topology::Topology;
use std::fmt::Write;

#[test]
fn concave_obj_and_ply_prisms_match_the_explicit_triangle_control() {
    let base = [
        (0, 0),
        (4, 0),
        (4, 4),
        (3, 4),
        (3, 1),
        (1, 1),
        (1, 4),
        (0, 4),
    ];
    for triangles in [false, true] {
        let caps: Vec<Vec<usize>> = if triangles {
            [
                [0, 1, 4],
                [0, 4, 5],
                [1, 2, 3],
                [1, 3, 4],
                [0, 5, 6],
                [0, 6, 7],
            ]
            .iter()
            .map(|t| t.to_vec())
            .collect()
        } else {
            vec![(0..8).collect()]
        };
        let mut faces = Vec::new();
        for cap in caps {
            faces.push(cap.iter().rev().copied().collect::<Vec<_>>());
            faces.push(cap.iter().map(|i| i + 8).collect());
        }
        for i in 0..8 {
            let j = (i + 1) % 8;
            faces.push(vec![i, j, j + 8, i + 8]);
        }
        let mut obj = String::new();
        let mut ply = format!(
            "ply\nformat ascii 1.0\nelement face {}\nproperty list uchar int vertex_indices\nelement vertex 16\nproperty float x\nproperty float y\nproperty float z\nend_header\n",
            faces.len()
        );
        // Both formats admit this ordering: corner positions arrive after faces.
        for face in faces {
            obj.push('f');
            write!(ply, "{}", face.len()).unwrap();
            for i in face {
                write!(obj, " {}", i + 1).unwrap();
                write!(ply, " {i}").unwrap();
            }
            obj.push('\n');
            ply.push('\n');
        }
        for z in [0, 1] {
            for (x, y) in base {
                writeln!(obj, "v {x} {y} {z}").unwrap();
                writeln!(ply, "{x} {y} {z}").unwrap();
            }
        }
        let mut topo = Topology::new();
        for s in [
            remus_io::obj::read_obj_solid(&mut topo, &obj, 1e-7).unwrap(),
            remus_io::ply::read_ply_solid(&mut topo, ply.as_bytes(), 1e-7).unwrap(),
        ] {
            assert!((solid_volume(&topo, s, 0.01).unwrap() - 10.).abs() < 1e-8);
            assert!((solid_surface_area(&topo, s, 0.01).unwrap() - 42.).abs() < 1e-8);
            assert!(
                remus_operations::validate::validate_solid(&topo, s)
                    .unwrap()
                    .is_valid()
            );
        }
    }
}
