//! PERF-I05 3MF streaming: compatibility and failure-path coverage.
//!
//! The streaming writer (`write_threemf`, `write_mesh_threemf`) must preserve
//! the pre-streaming format contract while bounding intermediate memory to
//! one live mesh plus the final archive (no staged `Vec<TriangleMesh>` of
//! every solid, no cloned copy of every caller mesh, no complete model-XML
//! buffer). These tests pin the contract from outside the writer:
//!
//! - Object/build ordering, 1-based IDs, units, `:.6` coordinate formatting,
//!   winding, archive entries/order, relationships and `Deflated` policy.
//! - Typed errors and no-partial-success on late failures.
//! - Degenerate filtering (including all-filtered) and input immutability.
//! - Independent ZIP/XML validation plus reader round-trip as a second oracle.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::io::{Cursor, Read as _};

use remus_math::vec::Point3;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;

// ── Helpers ──────────────────────────────────────────────────────────────

fn model_xml(bytes: &[u8]) -> String {
    let mut archive = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
    let mut f = archive.by_name("3D/3dmodel.model").unwrap();
    let mut s = String::new();
    f.read_to_string(&mut s).unwrap();
    s
}

fn archive_names(bytes: &[u8]) -> Vec<String> {
    let archive = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
    archive.file_names().map(str::to_string).collect()
}

fn tetra_mesh() -> TriangleMesh {
    TriangleMesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
        ],
        normals: Vec::new(),
        indices: vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2],
    }
}

// ── Empty and malformed input ────────────────────────────────────────────

#[test]
fn empty_solids_is_typed_refusal() {
    let topo = Topology::new();
    let err = remus_io::threemf::write_threemf(&topo, &[], 0.1).unwrap_err();
    assert!(matches!(err, remus_io::IoError::InvalidTopology { .. }));
}

#[test]
fn empty_meshes_is_typed_refusal() {
    let err = remus_io::threemf::write_mesh_threemf(&[]).unwrap_err();
    assert!(matches!(err, remus_io::IoError::InvalidTopology { .. }));
}

#[test]
fn invalid_deflections_are_typed_refusals() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    for defl in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let err = remus_io::threemf::write_threemf(&topo, &[solid], defl).unwrap_err();
        assert!(
            matches!(err, remus_io::IoError::InvalidTopology { .. }),
            "deflection {defl}: {err:?}"
        );
    }
}

#[test]
fn mesh_with_zero_indices_is_refused() {
    let mesh = TriangleMesh {
        positions: vec![Point3::new(0.0, 0.0, 0.0)],
        normals: Vec::new(),
        indices: Vec::new(),
    };
    let err = remus_io::threemf::write_mesh_threemf(&[mesh]).unwrap_err();
    assert!(matches!(err, remus_io::IoError::InvalidTopology { .. }));
}

#[test]
fn mesh_with_non_multiple_of_three_is_refused() {
    let mut mesh = tetra_mesh();
    mesh.indices.truncate(4);
    assert_eq!(mesh.indices.len(), 4);
    let err = remus_io::threemf::write_mesh_threemf(&[mesh]).unwrap_err();
    assert!(matches!(err, remus_io::IoError::InvalidTopology { .. }));
}

#[test]
fn mesh_with_out_of_bounds_index_is_typed_refusal_not_panic() {
    let mut mesh = tetra_mesh();
    mesh.indices[0] = 99;
    let err = remus_io::threemf::write_mesh_threemf(&[mesh]).unwrap_err();
    assert!(
        matches!(err, remus_io::IoError::InvalidTopology { .. }),
        "{err:?}"
    );
}

// ── Filtering ────────────────────────────────────────────────────────────

#[test]
fn filtering_all_triangles_is_zero_triangle_refusal() {
    // Every triangle collapses: two coincident-vertex facets plus one
    // collinear facet. Nothing survives the provider filter.
    let mesh = TriangleMesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        normals: Vec::new(),
        indices: vec![0, 0, 1, 1, 1, 2, 0, 1, 2],
    };
    let err = remus_io::threemf::write_mesh_threemf(&[mesh]).unwrap_err();
    assert!(
        matches!(err, remus_io::IoError::InvalidTopology { .. }),
        "{err:?}"
    );
}

#[test]
fn mesh_filtering_preserves_input_immutability() {
    let mut mesh = tetra_mesh();
    // Append one collapsed facet; it must be filtered on write.
    mesh.indices.extend_from_slice(&[0, 0, 0]);
    let before_positions = mesh.positions.clone();
    let before_indices = mesh.indices.clone();
    let bytes = remus_io::threemf::write_mesh_threemf(std::slice::from_ref(&mesh)).unwrap();
    // Caller-owned mesh is untouched by the borrow.
    assert_eq!(mesh.positions, before_positions);
    assert_eq!(mesh.indices, before_indices);
    // Serialized output carries only the 4 kept triangles.
    let meshes = remus_io::threemf::read_threemf(&bytes).unwrap();
    assert_eq!(meshes[0].indices.len(), 12);
}

#[test]
fn tiny_sliver_survives_filtering() {
    // 1e-15-height sliver: nonzero finite area, must be kept (provider rule).
    let mesh = TriangleMesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 1.0e-15, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        normals: Vec::new(),
        indices: vec![0, 1, 2, 0, 0, 2, 0, 1, 4, 0, 1, 3],
    };
    let bytes = remus_io::threemf::write_mesh_threemf(&[mesh]).unwrap();
    let meshes = remus_io::threemf::read_threemf(&bytes).unwrap();
    // (0,0,2) and (0,1,4-collinear) drop; (0,1,2) and the sliver (0,1,3) stay.
    assert_eq!(meshes[0].indices.len(), 6);
}

// ── Late failure: no partial success ─────────────────────────────────────

#[test]
fn invalid_later_solid_fails_without_partial_document() {
    let mut topo = Topology::new();
    let valid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let before = remus_io::threemf::write_threemf(&topo, &[valid], 0.1).unwrap();

    // A handle whose arena index is out of bounds locally.
    let mut other = Topology::new();
    for _ in 0..5 {
        make_box(&mut other, 1.0, 1.0, 1.0).unwrap();
    }
    let foreign = other.solid_id_from_index(4).unwrap();
    assert!(topo.solid(foreign).is_err());

    let err = remus_io::threemf::write_threemf(&topo, &[valid, foreign], 0.1).unwrap_err();
    assert!(
        matches!(
            err,
            remus_io::IoError::Topology(_) | remus_io::IoError::Operations(_)
        ),
        "{err:?}"
    );

    // No partial document: Err, not truncated bytes. Borrowed topology is
    // unchanged, so the valid export is byte-stable.
    let after = remus_io::threemf::write_threemf(&topo, &[valid], 0.1).unwrap();
    assert_eq!(before, after);
    assert!(topo.solid(valid).is_ok());
}

#[test]
fn invalid_later_mesh_fails_without_partial_document() {
    let good = tetra_mesh();
    let before = remus_io::threemf::write_mesh_threemf(std::slice::from_ref(&good)).unwrap();
    let mut bad = tetra_mesh();
    bad.indices[1] = 77;

    let err = remus_io::threemf::write_mesh_threemf(&[good.clone(), bad]).unwrap_err();
    assert!(matches!(err, remus_io::IoError::InvalidTopology { .. }));

    let after = remus_io::threemf::write_mesh_threemf(std::slice::from_ref(&good)).unwrap();
    assert_eq!(before, after);
}

// ── Format preservation: independent ZIP/XML oracle ──────────────────────

#[test]
fn archive_entries_order_and_compression_are_preserved() {
    let mut topo = Topology::new();
    let s1 = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let s2 = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let bytes = remus_io::threemf::write_threemf(&topo, &[s1, s2], 0.1).unwrap();

    // Entry order is part of the contract.
    assert_eq!(
        archive_names(&bytes),
        vec![
            "[Content_Types].xml".to_string(),
            "_rels/.rels".to_string(),
            "3D/3dmodel.model".to_string(),
        ]
    );

    let mut archive = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
    for i in 0..archive.len() {
        let f = archive.by_index(i).unwrap();
        assert_eq!(
            f.compression(),
            zip::CompressionMethod::Deflated,
            "entry {} must stay Deflated",
            f.name()
        );
    }

    // Relationships point at the model part.
    let mut rels = String::new();
    archive
        .by_name("_rels/.rels")
        .unwrap()
        .read_to_string(&mut rels)
        .unwrap();
    assert!(rels.contains("/3D/3dmodel.model"));
}

#[test]
fn object_build_ordering_ids_units_and_formatting_are_preserved() {
    let mut topo = Topology::new();
    let s1 = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let s2 = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let bytes = remus_io::threemf::write_threemf(&topo, &[s1, s2], 0.1).unwrap();
    let xml = model_xml(&bytes);

    assert!(xml.contains("unit=\"millimeter\""));
    assert!(xml.contains("http://schemas.microsoft.com/3dmanufacturing/core/2015/02"));
    // 1-based object ids in input order, build items reference them in order.
    assert!(xml.contains("object id=\"1\""));
    assert!(xml.contains("object id=\"2\""));
    let build = xml
        .split("<build>")
        .nth(1)
        .unwrap()
        .split("</build>")
        .next()
        .unwrap();
    let item1 = build.find("objectid=\"1\"").unwrap();
    let item2 = build.find("objectid=\"2\"").unwrap();
    assert!(item1 < item2, "build items must follow solid order");
    // `:.6` coordinate formatting is unchanged (box corner at 1.0).
    assert!(xml.contains("x=\"1.000000\""));
    assert!(!xml.contains("NaN"));
}

#[test]
fn multi_solid_order_pins_output() {
    // Two different-sized boxes: the first object's vertices identify which
    // solid went first.
    let mut topo = Topology::new();
    let small = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let large = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let small_first = remus_io::threemf::write_threemf(&topo, &[small, large], 0.1).unwrap();
    let large_first = remus_io::threemf::write_threemf(&topo, &[large, small], 0.1).unwrap();
    let xml_small = model_xml(&small_first);
    let xml_large = model_xml(&large_first);
    assert_ne!(xml_small, xml_large, "solid order pins output");
    // 2.0 extent appears in the first object only when large goes first.
    let first_object_small = xml_small
        .split("<object")
        .nth(1)
        .unwrap()
        .split("</object>")
        .next()
        .unwrap();
    let first_object_large = xml_large
        .split("<object")
        .nth(1)
        .unwrap()
        .split("</object>")
        .next()
        .unwrap();
    assert!(!first_object_small.contains("2.000000"));
    assert!(first_object_large.contains("2.000000"));
}

#[test]
fn winding_and_topology_survive_round_trip() {
    let mesh = tetra_mesh();
    let bytes = remus_io::threemf::write_mesh_threemf(std::slice::from_ref(&mesh)).unwrap();
    // Independent oracle: parse triangle indices straight from the XML.
    let xml = model_xml(&bytes);
    let tri_count = xml.matches("<triangle ").count();
    assert_eq!(tri_count, 4);
    // Reader as second oracle: same counts, same winding order.
    let meshes = remus_io::threemf::read_threemf(&bytes).unwrap();
    assert_eq!(meshes.len(), 1);
    assert_eq!(meshes[0].positions.len(), 4);
    assert_eq!(meshes[0].indices, mesh.indices);
    // Coordinates survive the `:.6` quantization exactly for integers.
    for pos in &meshes[0].positions {
        assert!((pos.x().round() - pos.x()).abs() < 1e-9);
    }
}

#[test]
fn solid_export_round_trip_preserves_counts_and_bounds() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let bytes = remus_io::threemf::write_threemf(&topo, &[solid], 0.1).unwrap();
    let xml = model_xml(&bytes);
    assert_eq!(xml.matches("<vertex ").count(), 8);
    assert_eq!(xml.matches("<triangle ").count(), 12);
    let meshes = remus_io::threemf::read_threemf(&bytes).unwrap();
    assert_eq!(meshes.len(), 1);
    assert_eq!(meshes[0].indices.len(), 36);
}
