//! Kernel review 2026-10-15, IO-4 follow-up (B102): 3MF build items.
//!
//! The reader ignored `<build>`: `<item transform>` was never applied, and
//! the returned meshes did not depend on the build list, so an object a
//! slicer places on the bed through its item transform imported at its
//! resource origin, an object listed twice came back once, and unused
//! resources came back as extra meshes. The build items are now the roots,
//! in order and with multiplicity, each placed by its item transform after
//! component flattening.

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

/// A single-triangle object: three vertices, one triangle.
fn triangle_object(id: u32) -> String {
    format!(
        r#"<object id="{id}" type="model"><mesh><vertices><vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/></vertices><triangles><triangle v1="0" v2="1" v3="2"/></triangles></mesh></object>"#
    )
}

fn component_object(id: u32, child: u32, transform: &str) -> String {
    format!(
        r#"<object id="{id}" type="model"><components><component objectid="{child}" transform="{transform}"/></components></object>"#
    )
}

fn item(object_id: u32, transform: Option<&str>) -> String {
    match transform {
        Some(t) => format!(r#"<item objectid="{object_id}" transform="{t}"/>"#),
        None => format!(r#"<item objectid="{object_id}"/>"#),
    }
}

/// `build`: `None` omits the `<build>` element, `Some(items)` writes it.
fn model(unit: &str, objects: &[String], build: Option<&[String]>) -> Vec<u8> {
    let mut xml = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?><model unit="{unit}" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06"><resources>"#
    );
    for object in objects {
        xml += object;
    }
    xml += "</resources>";
    if let Some(items) = build {
        xml += "<build>";
        for item in items {
            xml += item;
        }
        xml += "</build>";
    }
    xml += "</model>";

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

/// One mesh per build item, in build order: object 3 (a component placing
/// cube 1 at x + 10) turned a quarter about +z and lifted by 100, then cube 1
/// twice at two offsets. Object 2 is an unused resource and is not returned.
/// The quarter turn uses 3MF's row-vector layout (`m01 = 1`, `m10 = -1` is
/// x' = −y, y' = x), applied after the component transform.
#[test]
fn build_items_are_the_roots_in_order_with_multiplicity() {
    let data = model(
        "millimeter",
        &[
            cube_object(1),
            cube_object(2),
            component_object(3, 1, "1 0 0 0 1 0 0 0 1 10 0 0"),
        ],
        Some(&[
            item(3, Some("0 1 0 -1 0 0 0 0 1 0 0 100")),
            item(1, Some("1 0 0 0 1 0 0 0 1 5 0 0")),
            item(1, Some("1 0 0 0 1 0 0 0 1 0 -7 0")),
        ]),
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 3, "one mesh per build item");
    for mesh in &meshes {
        assert_eq!(mesh.indices.len(), 12 * 3, "triangle count");
    }
    assert_positions(
        &meshes[0].positions,
        &cube_corners(|p| Point3::new(-p.y(), p.x() + 10.0, p.z() + 100.0)),
    );
    assert_positions(
        &meshes[1].positions,
        &cube_corners(|p| Point3::new(p.x() + 5.0, p.y(), p.z())),
    );
    assert_positions(
        &meshes[2].positions,
        &cube_corners(|p| Point3::new(p.x(), p.y() - 7.0, p.z())),
    );
}

/// A mirroring item transform keeps the winding outward, and
/// `read_threemf_solid` imports the first build item, not the first object.
#[test]
fn mirrored_build_item_keeps_winding_and_solid_reads_first_item() {
    let data = model(
        "millimeter",
        &[cube_object(1), cube_object(2)],
        Some(&[item(2, Some("-1 0 0 0 1 0 0 0 1 -5 0 0")), item(1, None)]),
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 2);
    assert_positions(
        &meshes[0].positions,
        &cube_corners(|p| Point3::new(-p.x() - 5.0, p.y(), p.z())),
    );
    assert_positions(&meshes[1].positions, &cube_corners(|p| p));

    let mut topo = Topology::new();
    let solid = read_threemf_solid(&mut topo, &data, 1e-7).unwrap();
    assert_eq!(solid_faces(&topo, solid).unwrap().len(), 12);
    let volume = measure::oriented_solid_volume(&topo, solid, 0.1).unwrap();
    assert!((volume - 1.0).abs() < 1e-9, "signed volume {volume}");
    let bbox = measure::solid_bounding_box(&topo, solid).unwrap();
    assert!(
        (bbox.min.x() + 6.0).abs() < 1e-9 && (bbox.max.x() + 5.0).abs() < 1e-9,
        "the first build item (mirrored cube 2) was imported: {bbox:?}"
    );
}

/// Item translations are model-unit lengths, scaled with the vertices.
#[test]
fn build_item_translation_follows_model_unit() {
    let data = model(
        "centimeter",
        &[cube_object(1)],
        Some(&[item(1, Some("1 0 0 0 1 0 0 0 1 1 2 3"))]),
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 1);
    assert_positions(
        &meshes[0].positions,
        &cube_corners(|p| {
            Point3::new(
                10.0 * p.x() + 10.0,
                10.0 * p.y() + 20.0,
                10.0 * p.z() + 30.0,
            )
        }),
    );
}

/// Without build items, every object no component references is a root, in
/// document order, unplaced.
#[test]
fn missing_or_empty_build_falls_back_to_unreferenced_objects() {
    let objects = [
        cube_object(1),
        cube_object(2),
        component_object(3, 1, "1 0 0 0 1 0 0 0 1 10 0 0"),
    ];
    for build in [None, Some(&[][..])] {
        let meshes = read_threemf(&model("millimeter", &objects, build)).unwrap();
        assert_eq!(meshes.len(), 2, "objects 2 and 3 ({build:?})");
        assert_positions(&meshes[0].positions, &cube_corners(|p| p));
        assert_positions(
            &meshes[1].positions,
            &cube_corners(|p| Point3::new(p.x() + 10.0, p.y(), p.z())),
        );
    }
}

/// Core `transform` and `objectid` are unqualified attributes; a vendor
/// extension's `v:transform` neither replaces nor invalidates them.
#[test]
fn extension_attributes_do_not_override_core_placement() {
    let data = model(
        "millimeter",
        &[cube_object(1)],
        Some(&[
            r#"<item xmlns:v="urn:vendor" v:transform="not a matrix" objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 3" v:objectid="9"/>"#.to_string(),
            r#"<item xmlns:v="urn:vendor" objectid="1" v:transform="1 0 0 0 1 0 0 0 1 50 0 0"/>"#.to_string(),
        ]),
    );
    let meshes = read_threemf(&data).unwrap();
    assert_eq!(meshes.len(), 2);
    assert_positions(
        &meshes[0].positions,
        &cube_corners(|p| Point3::new(p.x(), p.y(), p.z() + 3.0)),
    );
    assert_positions(&meshes[1].positions, &cube_corners(|p| p));
}

/// Extension elements that share a core local name are not core build or
/// component instructions, and another extension's `path` is not the
/// production extension's `p:path`.
#[test]
fn extension_elements_are_not_core_build_instructions() {
    let vendor = r#"xmlns:v="urn:vendor""#;
    let data = model(
        "millimeter",
        &[
            cube_object(1),
            format!(
                r#"<object id="2" type="model" {vendor}><components><component objectid="1"/><v:component objectid="1" transform="1 0 0 0 1 0 0 0 1 9 9 9"/></components></object>"#
            ),
        ],
        Some(&[
            format!(r#"<item {vendor} objectid="2" v:path="/elsewhere"/>"#),
            format!(r#"<v:item {vendor} objectid="1" transform="1 0 0 0 1 0 0 0 1 50 0 0"/>"#),
        ]),
    );
    // A vendor build after the core build names nothing the core reads.
    let data_with_vendor_build = {
        let mut zip = zip::ZipArchive::new(Cursor::new(data.as_slice())).unwrap();
        let mut xml = String::new();
        std::io::Read::read_to_string(&mut zip.by_name("3D/3dmodel.model").unwrap(), &mut xml)
            .unwrap();
        let xml = xml.replace(
            "</build>",
            &format!(r#"</build><v:build {vendor}><v:item objectid="1"/></v:build>"#),
        );
        let mut out = zip::ZipWriter::new(Cursor::new(Vec::new()));
        out.start_file("3D/3dmodel.model", zip::write::SimpleFileOptions::default())
            .unwrap();
        out.write_all(xml.as_bytes()).unwrap();
        out.finish().unwrap().into_inner()
    };
    let meshes = read_threemf(&data_with_vendor_build).unwrap();
    assert_eq!(meshes.len(), 1, "only the core item");
    assert_positions(&meshes[0].positions, &cube_corners(|p| p));

    // A `path` whose prefix no declaration binds is refused, not guessed.
    let undeclared = model(
        "millimeter",
        &[cube_object(1)],
        Some(&[r#"<item q:path="/3D/Objects/part.model" objectid="1"/>"#.to_string()]),
    );
    assert!(parse_error(&undeclared).contains("production extension"));
}

fn parse_error(data: &[u8]) -> String {
    match read_threemf(data) {
        Err(IoError::ParseError { reason }) => reason,
        other => panic!("expected a parse error, got {other:?}"),
    }
}

#[test]
fn invalid_build_items_are_refused() {
    let unknown = model("millimeter", &[cube_object(1)], Some(&[item(7, None)]));
    assert!(parse_error(&unknown).contains("unknown object id 7"));

    let ambiguous = model(
        "millimeter",
        &[cube_object(1), cube_object(1)],
        Some(&[item(1, None)]),
    );
    assert!(parse_error(&ambiguous).contains("more than once"));

    let malformed = model(
        "millimeter",
        &[cube_object(1)],
        Some(&[item(1, Some("1 0 0 0 1 0 0 0 1 0 0"))]),
    );
    assert!(parse_error(&malformed).contains("transform"));

    // The production extension's `p:path` names an object in another model
    // part; resolving `objectid` in this part would place the wrong object.
    let external = model(
        "millimeter",
        &[cube_object(1)],
        Some(&[r#"<item p:path="/3D/Objects/part.model" objectid="1"/>"#.to_string()]),
    );
    assert!(parse_error(&external).contains("production extension"));

    // Finite inputs whose placement overflows to infinity.
    let overflow = model(
        "millimeter",
        &[cube_object(1)],
        Some(&[item(1, Some("1e308 0 0 0 1 0 0 0 1 1e308 0 0"))]),
    );
    assert!(parse_error(&overflow).contains("finite millimeter range"));
}

/// An object listed many times multiplies the output; the total is bounded
/// by `max_model_entities` before any geometry is copied, and so is the
/// number of items.
#[test]
fn repeated_build_items_are_bounded_by_import_limits() {
    let limits = ImportLimits {
        max_model_entities: 100,
        ..ImportLimits::default()
    };
    let nine = vec![item(1, None); 9];
    // 72 vertices fit; 108 triangles do not.
    assert!(matches!(
        read_threemf_with_limits(&model("millimeter", &[cube_object(1)], Some(&nine)), limits),
        Err(IoError::LimitExceeded {
            resource: "3MF flattened triangles",
            limit: 100,
            actual: 108,
        })
    ));

    let limits = ImportLimits {
        max_model_entities: 5,
        ..ImportLimits::default()
    };
    let six = vec![item(1, None); 6];
    assert!(matches!(
        read_threemf_with_limits(
            &model("millimeter", &[triangle_object(1)], Some(&six)),
            limits
        ),
        Err(IoError::LimitExceeded {
            resource: "3MF build items",
            limit: 5,
            actual: 6,
        })
    ));
}
