//! Kernel review 2026-10-14, IO-4 (B90): 3MF objects built from components.
//!
//! The reader produced one mesh per `<object>` from its `<mesh>` alone, so an
//! object made only of `<components>` came back as an empty mesh, and every
//! component transform was ignored. The binding import loops then failed the
//! whole file with "mesh has no triangles". Components now resolve
//! recursively with their transforms; the objects they reference are parts,
//! not separate top-level meshes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Write as _;
use std::io::{Cursor, Write as _};

use remus_io::IoError;
use remus_io::limits::ImportLimits;
use remus_io::threemf::{read_threemf, read_threemf_solid, read_threemf_with_limits};
use remus_math::vec::Point3;
use remus_operations::measure;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;

const CORNERS: [[f64; 3]; 8] = [
    [0., 0., 0.],
    [1., 0., 0.],
    [1., 1., 0.],
    [0., 1., 0.],
    [0., 0., 1.],
    [1., 0., 1.],
    [1., 1., 1.],
    [0., 1., 1.],
];

/// Outward-wound unit-cube triangles over [`CORNERS`].
const TRIANGLES: [[u32; 3]; 12] = [
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

fn cube_object(id: u32) -> String {
    let mut xml = format!(r#"<object id="{id}" type="model"><mesh><vertices>"#);
    for [x, y, z] in CORNERS {
        write!(xml, r#"<vertex x="{x}" y="{y}" z="{z}"/>"#).unwrap();
    }
    xml += "</vertices><triangles>";
    for [a, b, c] in TRIANGLES {
        write!(xml, r#"<triangle v1="{a}" v2="{b}" v3="{c}"/>"#).unwrap();
    }
    xml + "</triangles></mesh></object>"
}

/// An object made only of components, each `(objectid, transform)`.
fn component_object(id: u32, components: &[(u32, Option<&str>)]) -> String {
    let mut xml = format!(r#"<object id="{id}" type="model"><components>"#);
    for (object_id, transform) in components {
        match transform {
            Some(t) => write!(
                xml,
                r#"<component objectid="{object_id}" transform="{t}"/>"#
            ),
            None => write!(xml, r#"<component objectid="{object_id}"/>"#),
        }
        .unwrap();
    }
    xml + "</components></object>"
}

fn model(unit: &str, objects: &[String], build: &[u32]) -> Vec<u8> {
    let mut xml = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?><model unit="{unit}" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06"><resources>"#
    );
    for object in objects {
        xml += object;
    }
    xml += "</resources><build>";
    for id in build {
        write!(xml, r#"<item objectid="{id}"/>"#).unwrap();
    }
    xml += "</build></model>";

    let mut zip = zip::ZipWriter::new(Cursor::new(Vec::new()));
    zip.start_file("3D/3dmodel.model", zip::write::SimpleFileOptions::default())
        .unwrap();
    zip.write_all(xml.as_bytes()).unwrap();
    zip.finish().unwrap().into_inner()
}

fn cube_corners(place: impl Fn(Point3) -> Point3) -> Vec<Point3> {
    CORNERS
        .iter()
        .map(|&[x, y, z]| place(Point3::new(x, y, z)))
        .collect()
}

fn assert_positions(actual: &[Point3], expected: &[Point3]) {
    assert_eq!(actual.len(), expected.len(), "vertex count");
    for (index, (a, e)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (*a - *e).length() < 1e-12,
            "vertex {index}: got {a:?}, expected {e:?}"
        );
    }
}

/// A component-only object flattens into one mesh: one transformed copy of
/// the referenced cube per component, and the referenced cube is a part, not
/// a second top-level mesh.
#[test]
fn component_only_object_flattens_with_transforms() {
    let data = model(
        "millimeter",
        &[
            cube_object(1),
            component_object(
                2,
                &[
                    (1, Some("1 0 0 0 1 0 0 0 1 10 0 0")),
                    // Mirror x, then shift: x' = -x - 5.
                    (1, Some("-1 0 0 0 1 0 0 0 1 -5 0 0")),
                ],
            ),
        ],
        &[2],
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 1, "only the component object is top-level");
    let mesh = &meshes[0];
    assert_eq!(mesh.indices.len(), 2 * 12 * 3, "triangle count");

    let mut expected = cube_corners(|p| Point3::new(p.x() + 10.0, p.y(), p.z()));
    expected.extend(cube_corners(|p| Point3::new(-p.x() - 5.0, p.y(), p.z())));
    assert_positions(&mesh.positions, &expected);

    // The mirrored copy keeps its winding outward, so the two disjoint cubes
    // enclose +2 rather than cancelling to 0.
    let mut topo = Topology::new();
    let solid = read_threemf_solid(&mut topo, &data, 1e-7).unwrap();
    assert_eq!(solid_faces(&topo, solid).unwrap().len(), 24);
    let volume = measure::oriented_solid_volume(&topo, solid, 0.1).unwrap();
    assert!((volume - 2.0).abs() < 1e-9, "signed volume {volume}");
}

/// Nested components compose child-first: object 2 shifts the cube by
/// (10, 0, 0), then object 3 turns object 2 a quarter turn about +z and lifts
/// it by 100. The rotation uses 3MF's row-vector layout (`m01 = 1`,
/// `m10 = -1` is x' = −y, y' = x).
#[test]
fn nested_components_compose_child_transform_first() {
    let data = model(
        "millimeter",
        &[
            cube_object(1),
            component_object(2, &[(1, Some("1 0 0 0 1 0 0 0 1 10 0 0"))]),
            component_object(3, &[(2, Some("0 1 0 -1 0 0 0 0 1 0 0 100"))]),
        ],
        &[3],
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 1);
    assert_eq!(meshes[0].indices.len(), 12 * 3);
    let expected = cube_corners(|p| {
        let shifted = Point3::new(p.x() + 10.0, p.y(), p.z());
        Point3::new(-shifted.y(), shifted.x(), shifted.z() + 100.0)
    });
    assert_positions(&meshes[0].positions, &expected);
}

/// Translations are model-unit lengths, scaled to millimeters with the
/// vertices; a component without a transform is placed as is.
#[test]
fn component_translation_follows_model_unit() {
    let data = model(
        "centimeter",
        &[
            cube_object(1),
            component_object(2, &[(1, Some("1 0 0 0 1 0 0 0 1 1 0 0")), (1, None)]),
        ],
        &[2],
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 1);
    let mut expected =
        cube_corners(|p| Point3::new(10.0 * p.x() + 10.0, 10.0 * p.y(), 10.0 * p.z()));
    expected.extend(cube_corners(|p| {
        Point3::new(10.0 * p.x(), 10.0 * p.y(), 10.0 * p.z())
    }));
    assert_positions(&meshes[0].positions, &expected);
}

/// Files without components keep one mesh per object, in document order.
#[test]
fn objects_without_components_stay_separate() {
    let data = model("millimeter", &[cube_object(1), cube_object(2)], &[1, 2]);
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 2);
    for mesh in &meshes {
        assert_eq!(mesh.indices.len(), 12 * 3);
        assert_positions(&mesh.positions, &cube_corners(|p| p));
    }
}

fn parse_error(data: &[u8]) -> String {
    match read_threemf(data) {
        Err(IoError::ParseError { reason }) => reason,
        other => panic!("expected a parse error, got {other:?}"),
    }
}

#[test]
fn invalid_component_references_are_refused() {
    let cycle = model(
        "millimeter",
        &[
            component_object(1, &[(2, None)]),
            component_object(2, &[(1, None)]),
        ],
        &[1],
    );
    assert!(parse_error(&cycle).contains("cycle"));

    let self_reference = model("millimeter", &[component_object(1, &[(1, None)])], &[1]);
    assert!(parse_error(&self_reference).contains("cycle"));

    let unknown = model("millimeter", &[component_object(1, &[(7, None)])], &[1]);
    assert!(parse_error(&unknown).contains("unknown object id 7"));

    let ambiguous = model(
        "millimeter",
        &[
            cube_object(1),
            cube_object(1),
            component_object(2, &[(1, None)]),
        ],
        &[2],
    );
    assert!(parse_error(&ambiguous).contains("more than once"));

    let malformed = model(
        "millimeter",
        &[
            cube_object(1),
            component_object(2, &[(1, Some("1 0 0 0 1 0 0 0 1 0 0"))]),
        ],
        &[2],
    );
    assert!(parse_error(&malformed).contains("transform"));

    // The production extension's `p:path` names an object in another model
    // part; resolving `objectid` in this part would place the wrong object.
    let external = model(
        "millimeter",
        &[
            cube_object(1),
            r#"<object id="2" type="model"><components><component p:path="/3D/Objects/part.model" objectid="1"/></components></object>"#.to_string(),
        ],
        &[2],
    );
    assert!(parse_error(&external).contains("production extension"));
}

#[test]
fn component_nesting_depth_is_bounded() {
    let chain = |levels: u32| {
        let mut objects = vec![cube_object(1)];
        for id in 2..=levels + 1 {
            objects.push(component_object(id, &[(id - 1, None)]));
        }
        model("millimeter", &objects, &[levels + 1])
    };
    let meshes = read_threemf(&chain(32)).unwrap();
    assert_eq!(meshes.len(), 1);
    assert_positions(&meshes[0].positions, &cube_corners(|p| p));
    assert!(parse_error(&chain(33)).contains("deeper than 32"));
}

/// Ten components per level, nine levels deep, describe 12·10⁹ triangles in
/// a few kilobytes. The flattened size is bounded before any geometry is
/// copied, under the default limits and under a tightened one.
#[test]
fn flattened_output_is_bounded_by_import_limits() {
    let mut objects = vec![cube_object(1)];
    for id in 2..=10 {
        objects.push(component_object(id, &[(id - 1, None); 10]));
    }
    let data = model("millimeter", &objects, &[10]);
    assert!(matches!(
        read_threemf(&data),
        Err(IoError::LimitExceeded {
            resource: "3MF flattened vertices",
            ..
        })
    ));

    let two_level = model(
        "millimeter",
        &[cube_object(1), component_object(2, &[(1, None); 10])],
        &[2],
    );
    let limits = ImportLimits {
        max_model_entities: 100,
        ..ImportLimits::default()
    };
    // 80 vertices fit; 120 triangles do not.
    assert!(matches!(
        read_threemf_with_limits(&two_level, limits),
        Err(IoError::LimitExceeded {
            resource: "3MF flattened triangles",
            limit: 100,
            actual: 120,
        })
    ));
}

/// Attributes of unsupported extensions are ignored by namespace, not by
/// local name: `v:transform` does not move the part and `v:path` does not
/// read as the production extension's `p:path`.
#[test]
fn extension_attributes_are_matched_by_namespace() {
    let data = model(
        "millimeter",
        &[
            cube_object(1),
            r#"<object id="2" type="model"><components><component xmlns:v="urn:example:vendor" objectid="1" v:transform="1 0 0 0 1 0 0 0 1 9 9 9" v:path="/3D/other.model" transform="1 0 0 0 1 0 0 0 1 10 0 0"/></components></object>"#.to_string(),
        ],
        &[2],
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 1);
    assert_positions(
        &meshes[0].positions,
        &cube_corners(|p| Point3::new(p.x() + 10.0, p.y(), p.z())),
    );
}
