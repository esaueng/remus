//! Kernel review 2026-10-14, IO-5 (B89): the PLY reader must read the body
//! the header declares.
//!
//! The reader assumed `float x y z` vertices (plus `nx ny nz` when any
//! property name contained "nx") and `uchar int` face lists. It ignored
//! declared types, their order and any extra property, so a binary PLY with
//! colour bytes read garbage vertices and dropped every face, and an ASCII PLY
//! with colours before its normals read the normals from the colour columns,
//! both returning `Ok`.

#![allow(clippy::unwrap_used, clippy::panic)]

use std::fmt::Write as _;

use remus_io::IoError;
use remus_io::limits::ImportLimits;
use remus_io::ply::{read_ply, read_ply_with_limits};
use remus_math::vec::{Point3, Vec3};

const TETRA: [[f64; 3]; 4] = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
const TETRA_FACES: [[u32; 3]; 4] = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];

fn assert_points(actual: &[Point3], expected: &[[f64; 3]]) {
    assert_eq!(actual.len(), expected.len(), "vertex count");
    for (index, (a, &[x, y, z])) in actual.iter().zip(expected).enumerate() {
        assert!(
            (*a - Point3::new(x, y, z)).length() < 1e-12,
            "vertex {index}: got {a:?}, expected ({x}, {y}, {z})"
        );
    }
}

/// `x y z` floats followed by `red green blue` bytes: 15-byte vertices, not 12.
#[test]
fn binary_vertices_with_colour_bytes_read_by_declaration() {
    let mut data = b"ply\nformat binary_little_endian 1.0\nelement vertex 4\n\
property float x\nproperty float y\nproperty float z\n\
property uchar red\nproperty uchar green\nproperty uchar blue\n\
element face 4\nproperty list uchar int vertex_indices\nend_header\n"
        .to_vec();
    for p in TETRA {
        for value in p {
            data.extend((value as f32).to_le_bytes());
        }
        data.extend([200u8, 100, 50]);
    }
    for face in TETRA_FACES {
        data.push(3);
        for index in face {
            data.extend(index.cast_signed().to_le_bytes());
        }
    }

    let mesh = read_ply(&data).unwrap();
    assert_points(&mesh.positions, &TETRA);
    assert_eq!(mesh.indices.len(), 4 * 3, "triangle count");
    assert_eq!(mesh.indices, TETRA_FACES.concat());
}

/// Colours sit between the position and the normal columns.
#[test]
fn ascii_normals_come_from_their_declared_columns() {
    let normals = [[-1., -1., -1.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let mut text = String::from(
        "ply\nformat ascii 1.0\nelement vertex 4\n\
property float x\nproperty float y\nproperty float z\n\
property uchar red\nproperty uchar green\nproperty uchar blue\n\
property float nx\nproperty float ny\nproperty float nz\n\
element face 4\nproperty list uchar int vertex_indices\nend_header\n",
    );
    for (p, n) in TETRA.iter().zip(normals) {
        writeln!(
            text,
            "{} {} {} 255 128 7 {} {} {}",
            p[0], p[1], p[2], n[0], n[1], n[2]
        )
        .unwrap();
    }
    for [a, b, c] in TETRA_FACES {
        writeln!(text, "3 {a} {b} {c}").unwrap();
    }

    let mesh = read_ply(text.as_bytes()).unwrap();
    assert_points(&mesh.positions, &TETRA);
    assert_eq!(mesh.indices.len(), 4 * 3, "triangle count");
    assert_eq!(mesh.normals.len(), 4);
    for (index, (actual, [x, y, z])) in mesh.normals.iter().zip(normals).enumerate() {
        assert!(
            (*actual - Vec3::new(x, y, z)).length() < 1e-12,
            "normal {index}: got {actual:?}, expected ({x}, {y}, {z})"
        );
    }
}

/// Doubles in a shuffled order, extra scalar and list properties, an extra
/// element between vertices and faces, a signed `int` count with `uint`
/// items under the `vertex_index` name, and a quad.
#[test]
fn binary_types_order_extra_elements_and_lists_follow_the_header() {
    let mut data = b"ply\nformat binary_little_endian 1.0\ncomment shuffled\n\
element vertex 4\nproperty double z\nproperty ushort flags\nproperty double x\n\
property list uchar float texcoord\nproperty float y\n\
element material 2\nproperty short id\nproperty list ushort char name\n\
element face 1\nproperty uchar red\nproperty list int uint vertex_index\nproperty int32 tag\n\
end_header\n"
        .to_vec();
    let quad: [[f64; 3]; 4] = [[0., 0., 5.], [2., 0., 5.], [2., 3., 5.], [0., 3., 5.]];
    for (i, [x, y, z]) in quad.into_iter().enumerate() {
        data.extend(z.to_le_bytes());
        data.extend(7u16.to_le_bytes());
        data.extend(x.to_le_bytes());
        data.push(i as u8); // `i` texcoords
        for _ in 0..i {
            data.extend(0.5f32.to_le_bytes());
        }
        data.extend((y as f32).to_le_bytes());
    }
    for id in [1i16, 2] {
        data.extend(id.to_le_bytes());
        data.extend(3u16.to_le_bytes());
        data.extend(*b"abc");
    }
    data.push(9);
    data.extend(4i32.to_le_bytes());
    for index in [0u32, 1, 2, 3] {
        data.extend(index.to_le_bytes());
    }
    data.extend((-1i32).to_le_bytes());

    let mesh = read_ply(&data).unwrap();
    assert_points(&mesh.positions, &quad);
    assert_eq!(mesh.indices, [0, 1, 2, 0, 2, 3], "fan-triangulated quad");
}

fn tetra_ascii(vertex_properties: &str, rows: &[&str], face_property: &str) -> String {
    let mut text = format!(
        "ply\nformat ascii 1.0\nelement vertex {}\n{vertex_properties}\
element face 1\n{face_property}end_header\n",
        rows.len()
    );
    for row in rows {
        text += row;
        text += "\n";
    }
    text += "3 0 1 2\n";
    text
}

#[test]
fn every_declared_count_is_bounded_by_import_limits() {
    let limits = ImportLimits {
        max_model_entities: 3,
        ..ImportLimits::default()
    };
    // A skipped list longer than the limit.
    let text = tetra_ascii(
        "property float x\nproperty float y\nproperty float z\nproperty list uchar float uv\n",
        &["0 0 0 0", "1 0 0 0", "0 1 0 4 1 2 3 4"],
        "property list uchar int vertex_indices\n",
    );
    assert!(read_ply_with_limits(text.as_bytes(), ImportLimits::default()).is_ok());
    assert!(matches!(
        read_ply_with_limits(text.as_bytes(), limits),
        Err(IoError::LimitExceeded {
            resource: "PLY list items",
            limit: 3,
            actual: 4
        })
    ));
    // An element the reader skips still has its count bounded.
    let extra = "ply\nformat ascii 1.0\nelement edge 4\nproperty int vertex1\nend_header\n";
    assert!(matches!(
        read_ply_with_limits(extra.as_bytes(), limits),
        Err(IoError::LimitExceeded {
            resource: "PLY element instances",
            limit: 3,
            actual: 4
        })
    ));
    // Skipped elements share one budget: two elements of two instances each
    // pass one by one but not together.
    let split = "ply\nformat ascii 1.0\nelement edge 2\nproperty int vertex1\n\
element material 2\nproperty int id\nend_header\n1\n2\n3\n4\n";
    assert!(matches!(
        read_ply_with_limits(split.as_bytes(), limits),
        Err(IoError::LimitExceeded {
            resource: "PLY element instances",
            limit: 3,
            actual: 4
        })
    ));
}

#[test]
fn bodies_that_disagree_with_the_header_are_refused() {
    let xyz = "property float x\nproperty float y\nproperty float z\n";
    let list = "property list uchar int vertex_indices\n";
    let cases = [
        // An undeclared extra value on a vertex line.
        tetra_ascii(xyz, &["0 0 0", "1 0 0 9", "0 1 0"], list),
        // A missing value.
        tetra_ascii(xyz, &["0 0 0", "1 0", "0 1 0"], list),
        // A byte property out of range.
        tetra_ascii(
            &format!("{xyz}property uchar red\n"),
            &["0 0 0 1", "1 0 0 256", "0 1 0 1"],
            list,
        ),
        // A float list count.
        tetra_ascii(
            xyz,
            &["0 0 0", "1 0 0", "0 1 0"],
            "property list float int vertex_indices\n",
        ),
        // Missing coordinates.
        tetra_ascii(
            "property float x\nproperty float y\n",
            &["0 0", "1 0", "0 1"],
            list,
        ),
        // Faces without a vertex index list.
        tetra_ascii(xyz, &["0 0 0", "1 0 0", "0 1 0"], "property int material\n"),
        // A face referencing a vertex past the end.
        tetra_ascii(xyz, &["0 0 0", "1 0 0"], list),
    ];
    for text in cases {
        assert!(
            matches!(read_ply(text.as_bytes()), Err(IoError::ParseError { .. })),
            "accepted:\n{text}"
        );
    }

    let negative = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
property float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n\
0 0 0\n1 0 0\n0 1 0\n3 0 -1 2\n";
    assert!(matches!(
        read_ply(negative.as_bytes()),
        Err(IoError::ParseError { .. })
    ));

    let big_endian = "ply\nformat binary_big_endian 1.0\nelement vertex 0\nend_header\n";
    match read_ply(big_endian.as_bytes()) {
        Err(IoError::ParseError { reason }) => assert!(reason.contains("binary_big_endian")),
        other => panic!("big-endian PLY must be refused by name, got {other:?}"),
    }
}
