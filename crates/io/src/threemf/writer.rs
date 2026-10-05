//! 3MF file writer.
//!
//! Exports one or more [`Solid`](remus_topology::solid::Solid)s to the
//! [3D Manufacturing Format](https://3mf.io/specification/) (`.3mf`).
//!
//! A `.3mf` file is a ZIP archive containing:
//! - `[Content_Types].xml` — MIME declarations
//! - `_rels/.rels` — root relationships
//! - `3D/3dmodel.model` — mesh XML (vertices + triangles per object)

use std::io::{Cursor, Write as _};

use quick_xml::Writer;
use quick_xml::events::{BytesDecl, BytesEnd, BytesStart, Event};
use remus_operations::tessellate::{self, TriangleMesh};
use remus_topology::Topology;
use remus_topology::solid::SolidId;
use zip::ZipWriter;
use zip::write::SimpleFileOptions;

use crate::IoError;

/// The 3MF model namespace.
const NS_3MF: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";

/// Write-coalescing buffer for the streamed model XML entry.
///
/// Small XML events (one per vertex/triangle) are coalesced into 32 KiB
/// chunks before reaching the ZIP deflate encoder. This is a fixed
/// auxiliary buffer: it does not grow with object count or mesh size.
const MODEL_XML_BUF_CAP: usize = 32 * 1024;

/// Write one or more solids to a 3MF byte buffer.
///
/// Each solid is tessellated (all face meshes merged) and written as a
/// separate `<object>` in the model XML. The `deflection` parameter
/// controls tessellation density — smaller values produce finer meshes.
///
/// The model XML is streamed directly into its ZIP entry one solid mesh at
/// a time: peak intermediate mesh storage is one solid's mesh, not a merged
/// mesh of every solid, and no complete model-XML buffer is ever staged.
/// The returned archive buffer still scales with file size: this is a
/// bounded intermediate-memory improvement, not constant-memory export.
///
/// # Errors
///
/// Returns an error if:
/// - `solids` is empty
/// - `deflection` is not positive and finite
/// - Tessellation of any face fails
/// - ZIP or XML writing fails
///
/// A failure returns `Err` without a partial successful document; the input
/// topology is only borrowed and is left unchanged.
pub fn write_threemf(
    topo: &Topology,
    solids: &[SolidId],
    deflection: f64,
) -> Result<Vec<u8>, IoError> {
    if solids.is_empty() {
        return Err(IoError::InvalidTopology {
            reason: "no solids to export".to_string(),
        });
    }
    if !deflection.is_finite() || deflection <= 0.0 {
        return Err(IoError::InvalidTopology {
            reason: format!("deflection must be positive and finite, got {deflection}"),
        });
    }

    let buf = Cursor::new(Vec::new());
    let mut zip = ZipWriter::new(buf);
    let options = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    zip.start_file("[Content_Types].xml", options)?;
    zip.write_all(CONTENT_TYPES_XML)?;

    zip.start_file("_rels/.rels", options)?;
    zip.write_all(RELS_XML)?;

    zip.start_file("3D/3dmodel.model", options)?;
    {
        let mut buffered = std::io::BufWriter::with_capacity(MODEL_XML_BUF_CAP, &mut zip);
        {
            let mut writer = Writer::new_with_indent(&mut buffered, b' ', 1);
            writer.write_event(Event::Decl(BytesDecl::new("1.0", Some("UTF-8"), None)))?;

            let mut model = BytesStart::new("model");
            model.push_attribute(("xmlns", NS_3MF));
            model.push_attribute(("unit", "millimeter"));
            writer.write_event(Event::Start(model))?;

            writer.write_event(Event::Start(BytesStart::new("resources")))?;
            for (i, &solid_id) in solids.iter().enumerate() {
                let mesh = tessellate_solid(topo, solid_id, deflection)?;
                validate_solid_mesh(&mesh, i)?;
                write_object(&mut writer, i, &mesh)?;
            }
            writer.write_event(Event::End(BytesEnd::new("resources")))?;

            writer.write_event(Event::Start(BytesStart::new("build")))?;
            for i in 0..solids.len() {
                let mut item = BytesStart::new("item");
                let id_str = (i + 1).to_string();
                item.push_attribute(("objectid", id_str.as_str()));
                writer.write_event(Event::Empty(item))?;
            }
            writer.write_event(Event::End(BytesEnd::new("build")))?;

            writer.write_event(Event::End(BytesEnd::new("model")))?;
        }
        std::io::Write::flush(&mut buffered)?;
    }

    let cursor = zip.finish()?;
    Ok(cursor.into_inner())
}

/// Validate a tessellated solid mesh carries 3MF geometry.
fn validate_solid_mesh(mesh: &TriangleMesh, index: usize) -> Result<(), IoError> {
    if mesh.indices.len() < 3 {
        return Err(IoError::InvalidTopology {
            reason: format!("solid {index} tessellated to zero triangles"),
        });
    }
    if !mesh.indices.len().is_multiple_of(3) {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "solid {index} has {} indices (not a multiple of 3)",
                mesh.indices.len()
            ),
        });
    }
    Ok(())
}

/// Write already-tessellated meshes to 3MF format as bytes.
///
/// Each mesh is filtered through the same degenerate-facet rule the solid
/// writer applies ([`crate::retain_nondegenerate_triangles`]: exact
/// zero-area or non-finite triangles are dropped, every finite nonzero-area
/// triangle is kept) — then validated the same way [`write_threemf`]
/// validates its tessellation (at least one triangle, index count a
/// multiple of three). Unused non-finite vertices are omitted and indices
/// remapped; finite vertices and triangle winding retain their stored order.
/// This is the mesh-level seam
/// of [`write_threemf`]: it exists so the export contract (no exact
/// zero-area facet reaches a serialized file) can be exercised on
/// controlled meshes without a B-rep in the loop.
///
/// Filtering borrows each caller-supplied mesh and streams its kept
/// triangles: no cloned copy of every mesh is ever retained. Peak
/// intermediate mesh storage is one mesh's filtered index view, not a
/// cloned `Vec<TriangleMesh>` of the whole input, and no complete
/// model-XML buffer is staged. The input slice is only borrowed and is
/// left unchanged. The returned archive buffer still scales with file
/// size: this is a bounded intermediate-memory improvement, not
/// constant-memory export.
///
/// # Errors
///
/// Returns an error if:
/// - `meshes` is empty
/// - Any mesh has fewer than three indices, or a non-multiple-of-three
///   index count, after filtering
/// - Any triangle index is out of bounds for its mesh's positions
/// - ZIP or XML writing fails
///
/// A failure returns `Err` without a partial successful document.
pub fn write_mesh_threemf(meshes: &[TriangleMesh]) -> Result<Vec<u8>, IoError> {
    if meshes.is_empty() {
        return Err(IoError::InvalidTopology {
            reason: "no meshes to export".to_string(),
        });
    }

    let buf = Cursor::new(Vec::new());
    let mut zip = ZipWriter::new(buf);
    let options = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    zip.start_file("[Content_Types].xml", options)?;
    zip.write_all(CONTENT_TYPES_XML)?;

    zip.start_file("_rels/.rels", options)?;
    zip.write_all(RELS_XML)?;

    zip.start_file("3D/3dmodel.model", options)?;
    {
        let mut buffered = std::io::BufWriter::with_capacity(MODEL_XML_BUF_CAP, &mut zip);
        {
            let mut writer = Writer::new_with_indent(&mut buffered, b' ', 1);
            writer.write_event(Event::Decl(BytesDecl::new("1.0", Some("UTF-8"), None)))?;

            let mut model = BytesStart::new("model");
            model.push_attribute(("xmlns", NS_3MF));
            model.push_attribute(("unit", "millimeter"));
            writer.write_event(Event::Start(model))?;

            writer.write_event(Event::Start(BytesStart::new("resources")))?;
            for (i, mesh) in meshes.iter().enumerate() {
                validate_borrowed_mesh(mesh, i)?;
                write_object_filtered(&mut writer, i, mesh)?;
            }
            writer.write_event(Event::End(BytesEnd::new("resources")))?;

            writer.write_event(Event::Start(BytesStart::new("build")))?;
            for i in 0..meshes.len() {
                let mut item = BytesStart::new("item");
                let id_str = (i + 1).to_string();
                item.push_attribute(("objectid", id_str.as_str()));
                writer.write_event(Event::Empty(item))?;
            }
            writer.write_event(Event::End(BytesEnd::new("build")))?;

            writer.write_event(Event::End(BytesEnd::new("model")))?;
        }
        std::io::Write::flush(&mut buffered)?;
    }

    let cursor = zip.finish()?;
    Ok(cursor.into_inner())
}

/// Check a caller-supplied mesh can be filtered to 3MF geometry.
///
/// Mirrors the post-filter validation the solid writer applies: at least
/// one kept triangle and a multiple-of-three index count. Out-of-bounds
/// indices are a typed refusal here; the in-place provider filter would
/// index them directly.
fn validate_borrowed_mesh(mesh: &TriangleMesh, index: usize) -> Result<(), IoError> {
    if mesh.indices.len() < 3 {
        return Err(IoError::InvalidTopology {
            reason: format!("mesh {index} tessellated to zero triangles"),
        });
    }
    if !mesh.indices.len().is_multiple_of(3) {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "mesh {index} has {} indices (not a multiple of 3)",
                mesh.indices.len()
            ),
        });
    }
    let kept = count_kept_triangles(mesh, index)?;
    if kept == 0 {
        return Err(IoError::InvalidTopology {
            reason: format!("mesh {index} tessellated to zero triangles"),
        });
    }
    Ok(())
}

/// Count triangles the degenerate filter would keep, refusing
/// out-of-bounds indices instead of indexing them directly.
fn count_kept_triangles(mesh: &TriangleMesh, index: usize) -> Result<usize, IoError> {
    let mut kept = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = triangle_positions(mesh, tri, index)?;
        if is_nondegenerate(a, b, c) {
            kept += 1;
        }
    }
    Ok(kept)
}

/// Resolve one triangle's positions, refusing out-of-bounds indices.
fn triangle_positions(
    mesh: &TriangleMesh,
    tri: &[u32],
    mesh_index: usize,
) -> Result<
    (
        remus_math::vec::Point3,
        remus_math::vec::Point3,
        remus_math::vec::Point3,
    ),
    IoError,
> {
    let i0 = tri[0] as usize;
    let i1 = tri[1] as usize;
    let i2 = tri[2] as usize;
    if i0 >= mesh.positions.len() || i1 >= mesh.positions.len() || i2 >= mesh.positions.len() {
        return Err(IoError::InvalidTopology {
            reason: format!(
                "mesh {mesh_index} has triangle index out of bounds: [{}, {}, {}] but only {} vertices",
                tri[0],
                tri[1],
                tri[2],
                mesh.positions.len(),
            ),
        });
    }
    Ok((mesh.positions[i0], mesh.positions[i1], mesh.positions[i2]))
}

/// The provider filter predicate: keep every finite nonzero-area triangle.
fn is_nondegenerate(
    a: remus_math::vec::Point3,
    b: remus_math::vec::Point3,
    c: remus_math::vec::Point3,
) -> bool {
    let area_squared = (b - a).cross(c - a).length_squared();
    area_squared.is_finite() && area_squared > 0.0
}

/// Tessellate all faces of a solid into a single merged [`TriangleMesh`].
fn tessellate_solid(
    topo: &Topology,
    solid_id: SolidId,
    deflection: f64,
) -> Result<TriangleMesh, IoError> {
    // Use watertight tessellation that shares edge vertices between
    // adjacent faces, producing gap-free meshes for 3MF export.
    let mut mesh =
        tessellate::tessellate_solid(topo, solid_id, deflection).map_err(IoError::Operations)?;
    crate::retain_nondegenerate_triangles(&mut mesh);
    Ok(mesh)
}

/// Write a single `<object>` element containing `<mesh>` data.
///
/// The mesh is already filtered (solid path) or is written via
/// [`write_object_filtered`] (borrowed-mesh path); indices are serialized
/// in stored order, preserving winding; non-finite unused vertices are omitted.
fn write_object<W: std::io::Write>(
    writer: &mut Writer<W>,
    index: usize,
    mesh: &TriangleMesh,
) -> Result<(), IoError> {
    let id_str = (index + 1).to_string();

    let mut object = BytesStart::new("object");
    object.push_attribute(("id", id_str.as_str()));
    object.push_attribute(("type", "model"));
    writer.write_event(Event::Start(object))?;

    writer.write_event(Event::Start(BytesStart::new("mesh")))?;

    let omitted = write_finite_vertices(writer, mesh)?;

    writer.write_event(Event::Start(BytesStart::new("triangles")))?;
    for tri in mesh.indices.chunks_exact(3) {
        let mut triangle = BytesStart::new("triangle");
        triangle.push_attribute(("v1", remap_vertex(tri[0], &omitted).to_string().as_str()));
        triangle.push_attribute(("v2", remap_vertex(tri[1], &omitted).to_string().as_str()));
        triangle.push_attribute(("v3", remap_vertex(tri[2], &omitted).to_string().as_str()));
        writer.write_event(Event::Empty(triangle))?;
    }
    writer.write_event(Event::End(BytesEnd::new("triangles")))?;

    writer.write_event(Event::End(BytesEnd::new("mesh")))?;

    writer.write_event(Event::End(BytesEnd::new("object")))?;

    Ok(())
}

/// Write a single `<object>` from a borrowed caller mesh, keeping only the
/// triangles the provider filter keeps.
///
/// Finite positions retain their stored order, including unused vertices.
/// Non-finite vertices are omitted and kept triangle indices are remapped,
/// preserving winding. The caller mesh is only borrowed.
fn write_object_filtered<W: std::io::Write>(
    writer: &mut Writer<W>,
    index: usize,
    mesh: &TriangleMesh,
) -> Result<(), IoError> {
    let id_str = (index + 1).to_string();

    let mut object = BytesStart::new("object");
    object.push_attribute(("id", id_str.as_str()));
    object.push_attribute(("type", "model"));
    writer.write_event(Event::Start(object))?;

    writer.write_event(Event::Start(BytesStart::new("mesh")))?;

    let omitted = write_finite_vertices(writer, mesh)?;

    writer.write_event(Event::Start(BytesStart::new("triangles")))?;
    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = triangle_positions(mesh, tri, index)?;
        if !is_nondegenerate(a, b, c) {
            continue;
        }
        let mut triangle = BytesStart::new("triangle");
        triangle.push_attribute(("v1", remap_vertex(tri[0], &omitted).to_string().as_str()));
        triangle.push_attribute(("v2", remap_vertex(tri[1], &omitted).to_string().as_str()));
        triangle.push_attribute(("v3", remap_vertex(tri[2], &omitted).to_string().as_str()));
        writer.write_event(Event::Empty(triangle))?;
    }
    writer.write_event(Event::End(BytesEnd::new("triangles")))?;

    writer.write_event(Event::End(BytesEnd::new("mesh")))?;

    writer.write_event(Event::End(BytesEnd::new("object")))?;

    Ok(())
}

/// Retain finite vertices and record only removed slots for index remapping.
/// Valid meshes need no additional index storage; malformed vertices cannot
/// remain in the XML after their incident triangles have been filtered out.
fn write_finite_vertices<W: std::io::Write>(
    writer: &mut Writer<W>,
    mesh: &TriangleMesh,
) -> Result<Vec<usize>, IoError> {
    let mut omitted = Vec::new();
    writer.write_event(Event::Start(BytesStart::new("vertices")))?;
    for (index, pos) in mesh.positions.iter().enumerate() {
        if !pos.0.iter().all(|v| v.is_finite()) {
            omitted.push(index);
            continue;
        }
        let mut vertex = BytesStart::new("vertex");
        vertex.push_attribute(("x", format_f64(pos.x()).as_str()));
        vertex.push_attribute(("y", format_f64(pos.y()).as_str()));
        vertex.push_attribute(("z", format_f64(pos.z()).as_str()));
        writer.write_event(Event::Empty(vertex))?;
    }
    writer.write_event(Event::End(BytesEnd::new("vertices")))?;
    Ok(omitted)
}

fn remap_vertex(index: u32, omitted: &[usize]) -> usize {
    let index = index as usize;
    index - omitted.partition_point(|&slot| slot < index)
}

/// Format a float for XML output (enough precision, no trailing noise).
fn format_f64(v: f64) -> String {
    // Use enough digits for sub-micron precision in millimeters.
    format!("{v:.6}")
}

/// Static content for `[Content_Types].xml`.
const CONTENT_TYPES_XML: &[u8] = br#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#;

/// Static content for `_rels/.rels`.
const RELS_XML: &[u8] = br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#;

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::vec::{Point3, Vec3};
    use remus_operations::extrude::extrude;
    use remus_operations::revolve::revolve;
    use remus_topology::Topology;
    use remus_topology::test_utils::{make_unit_cube_non_manifold, make_unit_square_face};

    use super::*;

    /// Helper: extrude a unit square along +Z to create a box solid.
    fn make_extruded_box(topo: &mut Topology) -> SolidId {
        let face = make_unit_square_face(topo);
        extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
    }

    #[test]
    fn export_extruded_box() {
        let mut topo = Topology::new();
        let solid = make_extruded_box(&mut topo);

        let bytes = write_threemf(&topo, &[solid], 0.1).unwrap();

        let reader = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
        assert_eq!(reader.len(), 3);
        assert!(reader.file_names().any(|n| n == "[Content_Types].xml"));
        assert!(reader.file_names().any(|n| n == "_rels/.rels"));
        assert!(reader.file_names().any(|n| n == "3D/3dmodel.model"));
    }

    #[test]
    fn export_unit_cube() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_threemf(&topo, &[solid], 0.1).unwrap();

        let mut archive = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
        let mut model_file = archive.by_name("3D/3dmodel.model").unwrap();
        let mut xml_str = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut xml_str).unwrap();

        assert!(xml_str.contains("<vertex"));
        assert!(xml_str.contains("<triangle"));
        assert!(xml_str.contains("<object"));
        assert!(xml_str.contains("<item"));
    }

    #[test]
    fn export_revolved_solid() {
        let mut topo = Topology::new();
        let face = make_unit_square_face(&mut topo);
        // Revolve 90 degrees around Y axis at x=2 (offset so profile doesn't
        // intersect the axis).
        let solid = revolve(
            &mut topo,
            face,
            Point3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            std::f64::consts::FRAC_PI_2,
        )
        .unwrap();

        let bytes = write_threemf(&topo, &[solid], 0.25).unwrap();

        let reader = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
        assert_eq!(reader.len(), 3);
    }

    #[test]
    fn export_multiple_solids() {
        let mut topo = Topology::new();
        let s1 = make_extruded_box(&mut topo);
        let s2 = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_threemf(&topo, &[s1, s2], 0.1).unwrap();

        let mut archive = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
        let mut model_file = archive.by_name("3D/3dmodel.model").unwrap();
        let mut xml_str = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut xml_str).unwrap();

        assert_eq!(xml_str.matches("<object").count(), 2);
        assert_eq!(xml_str.matches("<item").count(), 2);
    }

    #[test]
    fn export_empty_solids_error() {
        let topo = Topology::new();
        let result = write_threemf(&topo, &[], 0.1);
        assert!(result.is_err());
    }

    #[test]
    fn export_invalid_deflection_error() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        assert!(write_threemf(&topo, &[solid], 0.0).is_err());
        assert!(write_threemf(&topo, &[solid], -1.0).is_err());
        assert!(write_threemf(&topo, &[solid], f64::NAN).is_err());
        assert!(write_threemf(&topo, &[solid], f64::INFINITY).is_err());
    }

    /// Parse the model XML and count vertices/triangles for verification.
    fn count_elements(xml: &str, tag: &str) -> usize {
        // Count self-closing tags like `<vertex .../>` and open tags like `<vertex ...>`.
        xml.matches(&format!("<{tag} ")).count()
    }

    /// Extract the model XML from a 3MF byte buffer.
    fn extract_model_xml(bytes: &[u8]) -> String {
        let mut archive = zip::ZipArchive::new(Cursor::new(bytes)).unwrap();
        let mut model_file = archive.by_name("3D/3dmodel.model").unwrap();
        let mut xml_str = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut xml_str).unwrap();
        xml_str
    }

    #[test]
    fn unit_cube_vertex_and_triangle_counts() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_threemf(&topo, &[solid], 0.1).unwrap();
        let xml = extract_model_xml(&bytes);

        // Unit cube: 8 corner vertices (shared across faces via watertight tessellation).
        assert_eq!(count_elements(&xml, "vertex"), 8);
        // Unit cube: 6 faces × 2 triangles = 12 triangles.
        assert_eq!(count_elements(&xml, "triangle"), 12);
    }

    #[test]
    fn xml_has_correct_namespace() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_threemf(&topo, &[solid], 0.5).unwrap();
        let xml = extract_model_xml(&bytes);

        assert!(xml.contains(NS_3MF));
        assert!(xml.contains("unit=\"millimeter\""));
    }

    #[test]
    fn vertex_attributes_are_finite() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_threemf(&topo, &[solid], 0.5).unwrap();
        let xml = extract_model_xml(&bytes);

        assert!(!xml.contains("NaN"));
        assert!(!xml.contains("Infinity"));
        assert!(!xml.contains("inf"));
    }

    #[test]
    fn roundtrip_zip_contains_expected_entries() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_threemf(&topo, &[solid], 0.5).unwrap();

        let reader = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
        let names: Vec<&str> = reader.file_names().collect();

        assert!(names.contains(&"[Content_Types].xml"));
        assert!(names.contains(&"_rels/.rels"));
        assert!(names.contains(&"3D/3dmodel.model"));
    }
}
