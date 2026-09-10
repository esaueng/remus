//! Issue #284: many small planar merge groups in a triangulated solid.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stderr)]

use std::collections::HashMap;
use std::time::Instant;

use remus_math::vec::Point3;
use remus_operations::{heal::unify_faces, measure::solid_volume, validate::validate_solid};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn corrugated_prism(segments: usize) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let rings: Vec<_> = (0..=segments)
        .map(|i| {
            let height = 1.0 + (i % 2) as f64;
            [(0.0, 0.0), (1.0, 0.0), (1.0, height), (0.0, height)]
                .map(|(y, z)| topo.add_vertex(Vertex::new(Point3::new(i as f64, y, z), 1e-7)))
        })
        .collect();
    let mut quads = vec![[rings[0][3], rings[0][2], rings[0][1], rings[0][0]]];
    for pair in rings.windows(2) {
        for k in 0..4 {
            let next = (k + 1) % 4;
            quads.push([pair[0][k], pair[0][next], pair[1][next], pair[1][k]]);
        }
    }
    quads.push(rings[segments]);
    let mut edges = HashMap::new();
    let mut faces = Vec::new();
    for quad in quads {
        for vertices in [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]] {
            let points = vertices.map(|v| topo.vertex(v).unwrap().point());
            let normal = (points[1] - points[0])
                .cross(points[2] - points[0])
                .normalize()
                .unwrap();
            let mut boundary = Vec::new();
            for i in 0..3 {
                let (a, b) = (vertices[i], vertices[(i + 1) % 3]);
                let forward = a.index() < b.index();
                let key = if forward { (a, b) } else { (b, a) };
                let edge = *edges
                    .entry(key)
                    .or_insert_with(|| topo.add_edge(Edge::new(key.0, key.1, EdgeCurve::Line)));
                boundary.push(OrientedEdge::new(edge, forward));
            }
            let wire = topo.add_wire(Wire::new(boundary, true).unwrap());
            faces.push(topo.add_face(Face::new(
                wire,
                Vec::new(),
                FaceSurface::Plane {
                    normal,
                    d: normal.dot(points[0] - Point3::new(0.0, 0.0, 0.0)),
                },
            )));
        }
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    let solid = topo.add_solid(Solid::new(shell, Vec::new()));
    (topo, solid)
}

#[test]
fn many_planar_groups_preserve_closed_geometry_and_boundary_limit() {
    for segments in [8, 64, 128] {
        let (mut topo, solid) = corrugated_prism(segments);
        assert!(validate_solid(&topo, solid).unwrap().is_valid());
        let removed = unify_faces(&mut topo, solid).unwrap();
        let expected = if segments < 100 {
            7 * segments - 1
        } else {
            segments + 2
        };
        assert_eq!(removed, expected);
        assert!(validate_solid(&topo, solid).unwrap().is_valid());
        assert!((solid_volume(&topo, solid, 0.01).unwrap() - 1.5 * segments as f64).abs() < 1e-6);
        assert_eq!(unify_faces(&mut topo, solid).unwrap(), 0);
    }
}

#[test]
#[ignore = "manual release-mode scaling measurement for issue #284"]
fn measure_unify_scaling() {
    for segments in [512, 2048, 6000] {
        let (topo, solid) = corrugated_prism(segments);
        assert!(validate_solid(&topo, solid).unwrap().is_valid());
        let mut timings = Vec::new();
        for _ in 0..3 {
            let mut input = topo.clone();
            let start = Instant::now();
            assert_eq!(unify_faces(&mut input, solid).unwrap(), segments + 2);
            timings.push(start.elapsed());
        }
        timings.sort();
        eprintln!("{} faces: median {:?}", 8 * segments + 4, timings[1]);
    }
}

#[test]
#[ignore = "set REMUS_UNIFY_STL to an STL path for a release-mode measurement"]
fn measure_unify_stl() {
    let path = std::env::var("REMUS_UNIFY_STL").expect("set REMUS_UNIFY_STL");
    let bytes = std::fs::read(path).unwrap();
    let mut topo = Topology::new();
    let solid = remus_io::stl::reader::read_stl_solid(&mut topo, &bytes, 1e-7).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let before = validate_solid(&topo, solid).unwrap();
    let volume = solid_volume(&topo, solid, 0.01).unwrap();
    let mut timings = Vec::new();
    for _ in 0..3 {
        let mut input = topo.clone();
        let start = Instant::now();
        let removed = unify_faces(&mut input, solid).unwrap();
        timings.push(start.elapsed());
        let after = validate_solid(&input, solid).unwrap();
        assert_eq!(after.is_valid(), before.is_valid());
        assert!((solid_volume(&input, solid, 0.01).unwrap() - volume).abs() < volume.abs() * 1e-6);
        eprintln!(
            "STL: {faces} faces, {removed} removed, {} validation errors",
            after.error_count()
        );
    }
    timings.sort();
    eprintln!("STL median {:?}", timings[1]);
}
