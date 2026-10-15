//! Kernel review 2026-10-14, IO-6 (B91): an inside-out mesh must import
//! outward.
//!
//! `import_mesh` flipped a closed mesh by its signed volume only when the mesh
//! carried no normals. Every file reader supplies normals: OBJ, PLY, 3MF and
//! glTF synthesise them from the winding, so they always agree with an inward
//! winding, and an STL facet normal of zero never disagrees with anything. An
//! inward-wound OBJ cube, or an STL cube with zero normals, imported with
//! signed volume −1.
#![allow(clippy::unwrap_used)]

use std::fmt::Write as _;

use remus_math::vec::Point3;
use remus_operations::measure;
use remus_topology::Topology;
use remus_topology::explorer::{face_vertices, solid_faces, solid_vertices};
use remus_topology::solid::SolidId;

/// Outward-wound triangles over the unit-cube corners listed by [`corners`].
const OUTWARD: [[usize; 3]; 12] = [
    [0, 2, 1],
    [0, 3, 2],
    [4, 5, 6],
    [4, 6, 7],
    [0, 1, 5],
    [0, 5, 4],
    [2, 3, 7],
    [2, 7, 6],
    [0, 4, 7],
    [0, 7, 3],
    [1, 2, 6],
    [1, 6, 5],
];

fn corners(offset: [f64; 3]) -> Vec<Point3> {
    [
        [0., 0., 0.],
        [1., 0., 0.],
        [1., 1., 0.],
        [0., 1., 0.],
        [0., 0., 1.],
        [1., 0., 1.],
        [1., 1., 1.],
        [0., 1., 1.],
    ]
    .into_iter()
    .map(|[x, y, z]| Point3::new(x + offset[0], y + offset[1], z + offset[2]))
    .collect()
}

fn triangles(inward: bool) -> Vec<[usize; 3]> {
    OUTWARD
        .into_iter()
        .map(|[a, b, c]| if inward { [a, c, b] } else { [a, b, c] })
        .collect()
}

fn cube_obj(offset: [f64; 3], inward: bool) -> String {
    let mut text = String::new();
    for p in corners(offset) {
        writeln!(text, "v {} {} {}", p.x(), p.y(), p.z()).unwrap();
    }
    for [a, b, c] in triangles(inward) {
        writeln!(text, "f {} {} {}", a + 1, b + 1, c + 1).unwrap();
    }
    text
}

/// Binary STL whose facet normals are all zero, as many exporters write them.
fn cube_stl_zero_normals(offset: [f64; 3], inward: bool) -> Vec<u8> {
    let points = corners(offset);
    let tris = triangles(inward);
    let mut bytes = vec![0u8; 80];
    bytes.extend((tris.len() as u32).to_le_bytes());
    for tri in tris {
        bytes.extend([0f32; 3].into_iter().flat_map(f32::to_le_bytes));
        for p in tri.map(|i| points[i]) {
            for value in [p.x(), p.y(), p.z()] {
                bytes.extend((value as f32).to_le_bytes());
            }
        }
        bytes.extend([0u8; 2]);
    }
    bytes
}

/// Twelve triangles on exactly the eight cube corners, each facing away from
/// the cube's centre, enclosing a signed volume of +1.
fn assert_outward_unit_cube(topo: &Topology, solid: SolidId, offset: [f64; 3], label: &str) {
    let faces = solid_faces(topo, solid).unwrap();
    assert_eq!(faces.len(), 12, "{label}: triangle count");

    let mut points: Vec<Point3> = solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(|v| topo.vertex(v).unwrap().point())
        .collect();
    assert_eq!(points.len(), 8, "{label}: welded vertex count");
    for expected in corners(offset) {
        let found = points.iter().position(|p| (*p - expected).length() < 1e-12);
        assert!(
            found.is_some(),
            "{label}: corner {expected:?} missing from {points:?}"
        );
        points.swap_remove(found.unwrap());
    }

    let centre = Point3::new(offset[0] + 0.5, offset[1] + 0.5, offset[2] + 0.5);
    for face in faces {
        let normal = topo.face(face).unwrap().effective_plane_normal().unwrap();
        for vertex in face_vertices(topo, face).unwrap() {
            let p = topo.vertex(vertex).unwrap().point();
            assert!(
                normal.dot(p - centre) > 0.0,
                "{label}: face {face:?} normal {normal:?} points into the cube"
            );
        }
    }

    let volume = measure::oriented_solid_volume(topo, solid, 0.1).unwrap();
    assert!(
        (volume - 1.0).abs() < 1e-9,
        "{label}: signed volume {volume}, expected +1"
    );
    assert!(
        !measure::solid_is_inverted(topo, solid).unwrap(),
        "{label}: solid is inverted"
    );
}

const OFFSETS: [[f64; 3]; 2] = [[0.0, 0.0, 0.0], [1000.0, -2000.0, 500.0]];

#[test]
fn inward_wound_obj_cube_imports_outward() {
    for offset in OFFSETS {
        let mut topo = Topology::new();
        let solid =
            remus_io::obj::read_obj_solid(&mut topo, &cube_obj(offset, true), 1e-7).unwrap();
        assert_outward_unit_cube(&topo, solid, offset, &format!("inward OBJ at {offset:?}"));
    }
}

#[test]
fn inward_wound_stl_cube_with_zero_normals_imports_outward() {
    for offset in OFFSETS {
        let mut topo = Topology::new();
        let stl = cube_stl_zero_normals(offset, true);
        let solid = remus_io::stl::read_stl_solid(&mut topo, &stl, 1e-7).unwrap();
        assert_outward_unit_cube(&topo, solid, offset, &format!("inward STL at {offset:?}"));
    }
}

/// Controls: an outward cube must not be flipped by the same rule.
#[test]
fn outward_wound_cubes_stay_outward() {
    for offset in OFFSETS {
        let mut topo = Topology::new();
        let solid =
            remus_io::obj::read_obj_solid(&mut topo, &cube_obj(offset, false), 1e-7).unwrap();
        assert_outward_unit_cube(&topo, solid, offset, &format!("outward OBJ at {offset:?}"));

        let mut topo = Topology::new();
        let stl = cube_stl_zero_normals(offset, false);
        let solid = remus_io::stl::read_stl_solid(&mut topo, &stl, 1e-7).unwrap();
        assert_outward_unit_cube(&topo, solid, offset, &format!("outward STL at {offset:?}"));
    }
}

/// STL facet normals that agree with an inward winding are both wrong; the
/// enclosed volume, not the stored normal, decides.
#[test]
fn inward_stl_cube_with_matching_inward_normals_imports_outward() {
    let offset = [0.0, 0.0, 0.0];
    let points = corners(offset);
    let mut stl = String::from("solid inward\n");
    for [a, b, c] in triangles(true) {
        let n = (points[b] - points[a])
            .cross(points[c] - points[a])
            .normalize()
            .unwrap();
        writeln!(
            stl,
            "facet normal {} {} {}\nouter loop",
            n.x(),
            n.y(),
            n.z()
        )
        .unwrap();
        for p in [points[a], points[b], points[c]] {
            writeln!(stl, "vertex {} {} {}", p.x(), p.y(), p.z()).unwrap();
        }
        stl += "endloop\nendfacet\n";
    }
    stl += "endsolid inward\n";
    let mut topo = Topology::new();
    let solid = remus_io::stl::read_stl_solid(&mut topo, stl.as_bytes(), 1e-7).unwrap();
    assert_outward_unit_cube(&topo, solid, offset, "inward ASCII STL, inward normals");
}

/// An OBJ of axis-aligned cubes `(origin, edge, inward)`, plus unused
/// vertices that no face references.
fn cubes_obj(cubes: &[([f64; 3], f64, bool)], unused: &[[f64; 3]]) -> String {
    let mut text = String::new();
    for (index, &(origin, edge, inward)) in cubes.iter().enumerate() {
        for p in corners([0.0; 3]) {
            let [x, y, z] = [p.x(), p.y(), p.z()];
            writeln!(
                text,
                "v {} {} {}",
                origin[0] + edge * x,
                origin[1] + edge * y,
                origin[2] + edge * z
            )
            .unwrap();
        }
        for [a, b, c] in triangles(inward) {
            let base = 8 * index + 1;
            writeln!(text, "f {} {} {}", a + base, b + base, c + base).unwrap();
        }
    }
    for [x, y, z] in unused {
        writeln!(text, "v {x} {y} {z}").unwrap();
    }
    text
}

fn import_obj_signed_volume(text: &str) -> f64 {
    let mut topo = Topology::new();
    let solid = remus_io::obj::read_obj_solid(&mut topo, text, 1e-7).unwrap();
    measure::oriented_solid_volume(&topo, solid, 0.1).unwrap()
}

/// Every component of a multi-body mesh inward: all are flipped.
#[test]
fn multi_body_mesh_with_every_body_inward_imports_outward() {
    let text = cubes_obj(&[([0.0; 3], 1.0, true), ([5.0, 0.0, 0.0], 2.0, true)], &[]);
    let volume = import_obj_signed_volume(&text);
    assert!((volume - 9.0).abs() < 1e-9, "signed volume {volume}");
}

/// Mixed component signs: one global flip cannot fix both, and the larger
/// inward body must not drag the outward one inside out (summing the whole
/// mesh would have flipped both, giving +8 − 1). A correctly wound cavity
/// has mixed signs too and must keep them.
#[test]
fn mixed_component_orientations_are_left_as_wound() {
    let mixed = cubes_obj(&[([0.0; 3], 1.0, false), ([5.0, 0.0, 0.0], 2.0, true)], &[]);
    let volume = import_obj_signed_volume(&mixed);
    assert!((volume + 7.0).abs() < 1e-9, "signed volume {volume}");

    let cavity = cubes_obj(&[([0.0; 3], 3.0, false), ([1.0; 3], 1.0, true)], &[]);
    let volume = import_obj_signed_volume(&cavity);
    assert!((volume - 26.0).abs() < 1e-9, "signed volume {volume}");
}

/// A far, unreferenced vertex must not move the summation point off the body.
#[test]
fn unused_far_vertex_does_not_hide_an_inward_cube() {
    let text = cubes_obj(&[([0.0; 3], 1.0, true)], &[[1e20, 0.0, 0.0]]);
    let volume = import_obj_signed_volume(&text);
    assert!((volume - 1.0).abs() < 1e-9, "signed volume {volume}");
}

/// Shells that only touch at one corner are separate bodies: an inward cube
/// must not decide the orientation of the outward cube at its corner (summed
/// as one body they enclose −7, and both would flip).
#[test]
fn shells_touching_at_a_vertex_are_oriented_separately() {
    let mixed = cubes_obj(&[([0.0; 3], 1.0, false), ([1.0; 3], 2.0, true)], &[]);
    let volume = import_obj_signed_volume(&mixed);
    assert!((volume + 7.0).abs() < 1e-9, "signed volume {volume}");

    let inward = cubes_obj(&[([0.0; 3], 1.0, true), ([1.0; 3], 2.0, true)], &[]);
    let volume = import_obj_signed_volume(&inward);
    assert!((volume - 9.0).abs() < 1e-9, "signed volume {volume}");
}
