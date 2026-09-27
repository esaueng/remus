//! STL file writer: binary and ASCII formats.

use std::io::Write;

use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::{self, TriangleMesh};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// STL output format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StlFormat {
    /// Binary STL (compact, standard for 3D printing).
    Binary,
    /// ASCII STL (human-readable, larger files).
    Ascii,
}

/// Write one or more solids to STL format as bytes.
///
/// Tessellates each solid with shared-edge tessellation for watertight output
/// and serializes one solid's mesh at a time into the final output buffer,
/// preserving solid order and triangle order within each solid. The `deflection`
/// parameter controls tessellation quality.
///
/// Peak intermediate mesh storage is one solid's mesh, not a merged mesh of
/// every solid. The final output buffer still scales with file size: this is a
/// bounded intermediate-memory improvement, not constant-memory export.
///
/// # Errors
///
/// Returns an error if tessellation or serialization fails, or if the total
/// binary triangle count or output size overflows its representation. A
/// failure returns `Err` without a partial successful document; the input
/// topology is only borrowed and is left unchanged.
pub fn write_stl(
    topo: &Topology,
    solids: &[SolidId],
    deflection: f64,
    format: StlFormat,
) -> Result<Vec<u8>, crate::IoError> {
    match format {
        StlFormat::Binary => write_binary_stl_streaming(topo, solids, deflection),
        StlFormat::Ascii => write_ascii_stl_streaming(topo, solids, deflection),
    }
}

/// Write one already-tessellated mesh to STL format as bytes.
///
/// The mesh is filtered through [`crate::retain_nondegenerate_triangles`] —
/// the same degenerate-facet removal the solid writers apply — and then
/// serialized verbatim. This is the mesh-level seam of [`write_stl`]: it
/// exists so the export contract (no exact zero-area facet reaches a
/// serialized file) can be exercised on a controlled mesh without a
/// B-rep in the loop.
///
/// # Errors
///
/// Returns an error if serialization fails.
pub fn write_mesh_stl(mesh: &TriangleMesh, format: StlFormat) -> Result<Vec<u8>, crate::IoError> {
    let mut filtered = mesh.clone();
    crate::retain_nondegenerate_triangles(&mut filtered);
    match format {
        StlFormat::Binary => write_binary_stl(&filtered),
        StlFormat::Ascii => write_ascii_stl(&filtered),
    }
}

/// Resolve the vertices and face normal for the `t`-th triangle in a mesh.
fn triangle_data(mesh: &TriangleMesh, t: usize) -> (Vec3, Point3, Point3, Point3) {
    let i0 = mesh.indices[t * 3] as usize;
    let i1 = mesh.indices[t * 3 + 1] as usize;
    let i2 = mesh.indices[t * 3 + 2] as usize;

    let v0 = mesh.positions[i0];
    let v1 = mesh.positions[i1];
    let v2 = mesh.positions[i2];

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let normal = edge1
        .cross(edge2)
        .normalize()
        .unwrap_or(Vec3::new(0.0, 0.0, 1.0));

    (normal, v0, v1, v2)
}

/// Binary STL body size: 50 bytes per triangle.
const BINARY_BYTES_PER_TRIANGLE: usize = 50;

/// Binary STL prefix size: 80-byte header plus 4-byte triangle count.
const BINARY_PREFIX_LEN: usize = 84;

/// Accumulate one mesh's triangle count into a running binary STL total.
///
/// The on-disk count is a `u32`; totals beyond `u32::MAX` are refused rather
/// than truncated so a reader never sees a count that disagrees with the
/// payload.
fn checked_add_triangle_count(total: u32, add: usize) -> Result<u32, crate::IoError> {
    let add_u32 = u32::try_from(add).map_err(|_| crate::IoError::InvalidTopology {
        reason: format!("STL triangle count {add} exceeds u32 range"),
    })?;
    total
        .checked_add(add_u32)
        .ok_or_else(|| crate::IoError::InvalidTopology {
            reason: format!("STL triangle count overflow: {total} + {add_u32} exceeds u32 range"),
        })
}

/// Byte length of a binary STL payload holding `tri_count` triangles.
///
/// Returns an error instead of wrapping when `84 + 50 * tri_count` overflows
/// `usize` (only reachable on 32-bit targets at the `u32::MAX` boundary, but
/// checked everywhere the count is used).
fn checked_binary_len(tri_count: u32) -> Result<usize, crate::IoError> {
    let tri_count = usize::try_from(tri_count).map_err(|_| crate::IoError::InvalidTopology {
        reason: format!("STL triangle count {tri_count} exceeds address space"),
    })?;
    tri_count
        .checked_mul(BINARY_BYTES_PER_TRIANGLE)
        .and_then(|body| body.checked_add(BINARY_PREFIX_LEN))
        .ok_or_else(|| crate::IoError::InvalidTopology {
            reason: "STL output size overflows address space".to_string(),
        })
}

/// Serialize every triangle of one filtered mesh into a binary STL buffer.
///
/// `f64` coordinates and recomputed geometric normals are narrowed to `f32`
/// little-endian, matching the historical `write_stl` behavior exactly.
fn append_mesh_binary(buf: &mut Vec<u8>, mesh: &TriangleMesh) {
    let tri_count = mesh.indices.len() / 3;
    for t in 0..tri_count {
        let (normal, v0, v1, v2) = triangle_data(mesh, t);

        write_f32_le(buf, normal.x());
        write_f32_le(buf, normal.y());
        write_f32_le(buf, normal.z());

        for v in [v0, v1, v2] {
            write_f32_le(buf, v.x());
            write_f32_le(buf, v.y());
            write_f32_le(buf, v.z());
        }

        // Attribute byte count (always 0).
        buf.extend_from_slice(&[0u8, 0u8]);
    }
}

/// Serialize every triangle of one filtered mesh as ASCII STL facets.
fn append_mesh_ascii(buf: &mut Vec<u8>, mesh: &TriangleMesh) -> Result<(), crate::IoError> {
    let tri_count = mesh.indices.len() / 3;
    for t in 0..tri_count {
        let (normal, v0, v1, v2) = triangle_data(mesh, t);

        writeln!(
            buf,
            "  facet normal {} {} {}",
            normal.x(),
            normal.y(),
            normal.z()
        )
        .map_err(crate::IoError::Io)?;
        writeln!(buf, "    outer loop").map_err(crate::IoError::Io)?;
        write_ascii_vertex(buf, v0)?;
        write_ascii_vertex(buf, v1)?;
        write_ascii_vertex(buf, v2)?;
        writeln!(buf, "    endloop").map_err(crate::IoError::Io)?;
        writeln!(buf, "  endfacet").map_err(crate::IoError::Io)?;
    }
    Ok(())
}

/// Tessellate each solid and stream its triangles into one binary STL buffer.
///
/// One solid's mesh is live at a time; the running `u32` count is patched
/// into bytes 80..84 after the last solid. Any tessellation or overflow
/// failure returns `Err` and discards the partial buffer.
fn write_binary_stl_streaming(
    topo: &Topology,
    solids: &[SolidId],
    deflection: f64,
) -> Result<Vec<u8>, crate::IoError> {
    let mut buf = Vec::new();
    let header = b"remus STL export";
    buf.extend_from_slice(header);
    // The STL binary header is a fixed 80 bytes; zero-pad whatever the
    // header string didn't fill.
    buf.resize(80, 0);
    // Placeholder count; patched after the last solid.
    buf.extend_from_slice(&[0u8, 0u8, 0u8, 0u8]);

    let mut total: u32 = 0;
    for &solid_id in solids {
        let mut mesh = tessellate::tessellate_solid(topo, solid_id, deflection)?;
        crate::retain_nondegenerate_triangles(&mut mesh);
        let tri_count = mesh.indices.len() / 3;
        total = checked_add_triangle_count(total, tri_count)?;
        let bytes = tri_count
            .checked_mul(BINARY_BYTES_PER_TRIANGLE)
            .ok_or_else(|| crate::IoError::InvalidTopology {
                reason: "STL output size overflows address space".to_string(),
            })?;
        // Detect address-space overflow before growing the output buffer.
        buf.len()
            .checked_add(bytes)
            .ok_or_else(|| crate::IoError::InvalidTopology {
                reason: "STL output size overflows address space".to_string(),
            })?;
        buf.reserve(bytes);
        append_mesh_binary(&mut buf, &mesh);
    }

    // The final length must agree with the patched count.
    debug_assert_eq!(buf.len(), checked_binary_len(total)?);
    buf[80..84].copy_from_slice(&total.to_le_bytes());
    Ok(buf)
}

/// Tessellate each solid and stream its facets into one ASCII STL document.
///
/// The `solid remus` / `endsolid remus` envelope and solid-order facet order
/// match the historical merged-mesh writer. Any failure returns `Err` and
/// discards the partial buffer.
fn write_ascii_stl_streaming(
    topo: &Topology,
    solids: &[SolidId],
    deflection: f64,
) -> Result<Vec<u8>, crate::IoError> {
    let mut buf = Vec::new();
    writeln!(buf, "solid remus").map_err(crate::IoError::Io)?;

    for &solid_id in solids {
        let mut mesh = tessellate::tessellate_solid(topo, solid_id, deflection)?;
        crate::retain_nondegenerate_triangles(&mut mesh);
        append_mesh_ascii(&mut buf, &mesh)?;
    }

    writeln!(buf, "endsolid remus").map_err(crate::IoError::Io)?;
    Ok(buf)
}

/// Write a binary STL file.
///
/// Format:
/// - 80-byte header
/// - 4-byte little-endian triangle count
/// - Per triangle (50 bytes):
///   - 12 bytes: normal (3 × f32)
///   - 36 bytes: 3 vertices (3 × 3 × f32)
///   - 2 bytes: attribute byte count (0)
fn write_binary_stl(mesh: &TriangleMesh) -> Result<Vec<u8>, crate::IoError> {
    let tri_count = mesh.indices.len() / 3;
    let total = checked_add_triangle_count(0, tri_count)?;
    let len = checked_binary_len(total)?;

    // 80-byte header + 4-byte count + 50 bytes per triangle.
    let mut buf = Vec::with_capacity(len);

    let header = b"remus STL export";
    buf.extend_from_slice(header);
    // The STL binary header is a fixed 80 bytes; zero-pad whatever the
    // header string didn't fill.
    buf.resize(80, 0);

    buf.extend_from_slice(&total.to_le_bytes());
    append_mesh_binary(&mut buf, mesh);

    Ok(buf)
}

/// Write an ASCII STL file.
fn write_ascii_stl(mesh: &TriangleMesh) -> Result<Vec<u8>, crate::IoError> {
    let mut buf = Vec::new();

    writeln!(buf, "solid remus").map_err(crate::IoError::Io)?;
    append_mesh_ascii(&mut buf, mesh)?;

    writeln!(buf, "endsolid remus").map_err(crate::IoError::Io)?;

    Ok(buf)
}

/// Write a vertex line in ASCII STL format.
fn write_ascii_vertex(buf: &mut Vec<u8>, p: Point3) -> Result<(), crate::IoError> {
    writeln!(buf, "      vertex {} {} {}", p.x(), p.y(), p.z()).map_err(crate::IoError::Io)
}

/// Write an f64 as f32 little-endian bytes.
#[allow(clippy::cast_possible_truncation)]
fn write_f32_le(buf: &mut Vec<u8>, v: f64) {
    buf.extend_from_slice(&(v as f32).to_le_bytes());
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_non_manifold;

    use super::*;

    #[test]
    fn write_binary_stl_unit_cube() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_stl(&topo, &[solid], 0.1, StlFormat::Binary).unwrap();

        // Header: 80 bytes + count: 4 bytes = 84 bytes header.
        assert!(bytes.len() >= 84);

        let tri_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;

        // Unit cube with 6 faces × 2 triangles each = 12 triangles.
        assert_eq!(tri_count, 12, "expected 12 triangles for unit cube");

        // Total size: 84 + 12 × 50 = 684 bytes.
        assert_eq!(bytes.len(), 84 + tri_count * 50);
    }

    #[test]
    fn write_ascii_stl_unit_cube() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = write_stl(&topo, &[solid], 0.1, StlFormat::Ascii).unwrap();
        let text = String::from_utf8(bytes).unwrap();

        assert!(text.starts_with("solid remus"));
        assert!(text.contains("facet normal"));
        assert!(text.contains("vertex"));
        assert!(text.trim().ends_with("endsolid remus"));

        let facet_count = text.matches("facet normal").count();
        assert_eq!(facet_count, 12, "expected 12 facets for unit cube");
    }

    #[test]
    fn write_stl_box_primitive() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        let bytes = write_stl(&topo, &[solid], 0.1, StlFormat::Binary).unwrap();
        let tri_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;

        assert_eq!(tri_count, 12, "box should have 12 triangles");
    }

    #[test]
    fn write_stl_multiple_solids() {
        let mut topo = Topology::new();
        let s1 = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let bytes = write_stl(&topo, &[s1, s2], 0.1, StlFormat::Binary).unwrap();
        let tri_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;

        assert_eq!(tri_count, 24, "two boxes should have 24 triangles");
    }

    #[test]
    fn write_stl_watertight_box() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        // Tessellate the same way write_stl does internally (via tessellate_solid).
        let mesh = remus_operations::tessellate::tessellate_solid(&topo, solid, 0.1).unwrap();

        // A watertight mesh has 0 boundary edges: every half-edge (a,b) has a twin (b,a).
        let boundary = remus_operations::tessellate::boundary_edge_count(&mesh);
        assert_eq!(
            boundary, 0,
            "STL mesh should have 0 boundary edges (watertight)"
        );
    }

    #[test]
    fn write_stl_shared_vertices_box() {
        // With tessellate_solid, a box should have exactly 8 unique vertices
        // (one per corner), not 24 (4 per face × 6 faces) as with per-face tessellation.
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let mesh = remus_operations::tessellate::tessellate_solid(&topo, solid, 0.1).unwrap();
        assert_eq!(
            mesh.positions.len(),
            8,
            "box should share vertices at corners"
        );
    }

    #[test]
    fn write_stl_empty_solids_preserves_envelope() {
        let topo = Topology::new();

        let binary = write_stl(&topo, &[], 0.1, StlFormat::Binary).unwrap();
        assert_eq!(
            binary.len(),
            84,
            "empty binary STL is header plus zero count"
        );
        let count = u32::from_le_bytes([binary[80], binary[81], binary[82], binary[83]]);
        assert_eq!(count, 0);
        assert_eq!(&binary[0..16], b"remus STL export");

        let ascii = write_stl(&topo, &[], 0.1, StlFormat::Ascii).unwrap();
        let text = String::from_utf8(ascii).unwrap();
        assert_eq!(text, "solid remus\nendsolid remus\n");
    }

    #[test]
    fn write_stl_multi_solid_preserves_order() {
        // Two different-sized boxes at the origin: the first facet identifies
        // which solid went first, pinning solid order.
        let mut topo = Topology::new();
        let small = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let large = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        let small_bytes = write_stl(&topo, &[small], 0.1, StlFormat::Binary).unwrap();
        let large_bytes = write_stl(&topo, &[large], 0.1, StlFormat::Binary).unwrap();
        let small_first = write_stl(&topo, &[small, large], 0.1, StlFormat::Binary).unwrap();
        let large_first = write_stl(&topo, &[large, small], 0.1, StlFormat::Binary).unwrap();

        for bytes in [&small_bytes, &large_bytes] {
            let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
            assert_eq!(count, 12, "each box contributes 12 triangles");
        }
        let both_count = u32::from_le_bytes([
            small_first[80],
            small_first[81],
            small_first[82],
            small_first[83],
        ]);
        assert_eq!(both_count, 24);

        // Facet payloads concatenate in solid order: [small, large] starts with
        // small's first facet and ends with large's last facet.
        assert_eq!(&small_first[84..84 + 50], &small_bytes[84..84 + 50]);
        assert_eq!(
            &small_first[small_first.len() - 50..],
            &large_bytes[large_bytes.len() - 50..]
        );
        assert_eq!(&large_first[84..84 + 50], &large_bytes[84..84 + 50]);
        assert!(
            small_first[84..] != large_first[84..],
            "solid order pins output"
        );
    }

    #[test]
    fn write_stl_curved_solids_cover_bounds() {
        let mut topo = Topology::new();
        let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 1.0, 5.0).unwrap();
        let sphere = remus_operations::primitives::make_sphere(&mut topo, 1.0, 12).unwrap();
        let torus = remus_operations::primitives::make_torus(&mut topo, 3.0, 1.0, 12).unwrap();

        for (name, solid) in [("cylinder", cylinder), ("sphere", sphere), ("torus", torus)] {
            let bytes = write_stl(&topo, &[solid], 0.05, StlFormat::Binary).unwrap();
            let mesh = crate::stl::read_stl(&bytes).unwrap();
            let tris = mesh.indices.len() / 3;
            assert!(
                tris > 12,
                "{name} should tessellate finer than a box, got {tris}"
            );
            // Normals are recomputed geometrically and must be unit length.
            for normal in &mesh.normals {
                let len =
                    (normal.x() * normal.x() + normal.y() * normal.y() + normal.z() * normal.z())
                        .sqrt();
                assert!(
                    (len - 1.0).abs() < 1e-5,
                    "{name} normal should be unit, got {len}"
                );
            }
        }

        // Multi-solid curved export concatenates counts.
        let bytes = write_stl(&topo, &[cylinder, sphere, torus], 0.05, StlFormat::Binary).unwrap();
        let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        let single_total: usize = [cylinder, sphere, torus]
            .iter()
            .map(|&s| {
                let b = write_stl(&topo, &[s], 0.05, StlFormat::Binary).unwrap();
                u32::from_le_bytes([b[80], b[81], b[82], b[83]]) as usize
            })
            .sum();
        assert_eq!(count, single_total);
        assert_eq!(bytes.len(), 84 + count * 50);
    }

    #[test]
    fn write_stl_hollow_solid_exports_cavity() {
        use remus_operations::boolean::{BooleanOp, boolean};

        let mut topo = Topology::new();
        let outer = remus_operations::primitives::make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
        let void = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        remus_operations::transform::transform_solid(
            &mut topo,
            void,
            &remus_math::mat::Mat4::translation(1.0, 1.0, 1.0),
        )
        .unwrap();
        let hollow = boolean(&mut topo, BooleanOp::Cut, outer, void).unwrap();
        assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);

        let bytes = write_stl(&topo, &[hollow], 0.1, StlFormat::Binary).unwrap();
        let mesh = crate::stl::read_stl(&bytes).unwrap();
        let tris = mesh.indices.len() / 3;
        // Outer plus inner skins: strictly more than a solid box.
        assert!(
            tris >= 24,
            "hollow box should carry outer and cavity facets, got {tris}"
        );
        // Outer bounds still pin the 3x3x3 envelope.
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for pos in &mesh.positions {
            min[0] = min[0].min(pos.x());
            min[1] = min[1].min(pos.y());
            min[2] = min[2].min(pos.z());
            max[0] = max[0].max(pos.x());
            max[1] = max[1].max(pos.y());
            max[2] = max[2].max(pos.z());
        }
        for (got, want) in min.iter().zip([0.0, 0.0, 0.0]) {
            assert!((got - want).abs() < 1e-6, "hollow min {min:?}");
        }
        for (got, want) in max.iter().zip([3.0, 3.0, 3.0]) {
            assert!((got - want).abs() < 1e-6, "hollow max {max:?}");
        }
    }

    #[test]
    fn write_mesh_stl_filters_exact_degenerate_facets() {
        use remus_math::vec::{Point3, Vec3};
        use remus_operations::tessellate::TriangleMesh;

        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(5.0, 5.0, 5.0),
            ],
            normals: vec![
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            // Triangle 0 is valid; triangle 1 collapses to one point.
            indices: vec![0, 1, 2, 3, 3, 3],
        };
        let binary = write_mesh_stl(&mesh, StlFormat::Binary).unwrap();
        let count = u32::from_le_bytes([binary[80], binary[81], binary[82], binary[83]]);
        assert_eq!(count, 1, "exact zero-area facet must be filtered");
        assert_eq!(binary.len(), 84 + 50);

        let ascii = write_mesh_stl(&mesh, StlFormat::Ascii).unwrap();
        let text = String::from_utf8(ascii).unwrap();
        assert_eq!(text.matches("facet normal").count(), 1);
    }

    #[test]
    fn write_stl_invalid_later_solid_fails_without_partial_document() {
        let mut topo = Topology::new();
        let valid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let before = write_stl(&topo, &[valid], 0.1, StlFormat::Binary).unwrap();

        // A handle whose arena index is out of bounds locally: the second
        // topology holds more solids so its last handle cannot alias locally.
        let mut other = Topology::new();
        for _ in 0..5 {
            remus_operations::primitives::make_box(&mut other, 1.0, 1.0, 1.0).unwrap();
        }
        let foreign = other.solid_id_from_index(4).unwrap();
        assert!(
            topo.solid(foreign).is_err(),
            "test setup must use a truly invalid handle"
        );

        let binary_err = write_stl(&topo, &[valid, foreign], 0.1, StlFormat::Binary);
        assert!(binary_err.is_err(), "later invalid solid must fail");
        let ascii_err = write_stl(&topo, &[valid, foreign], 0.1, StlFormat::Ascii);
        assert!(ascii_err.is_err(), "ascii path must fail the same way");

        // No partial document: the call returns Err, not truncated bytes. The
        // borrowed topology is unchanged, so the valid export is byte-stable.
        let after = write_stl(&topo, &[valid], 0.1, StlFormat::Binary).unwrap();
        assert_eq!(before, after);
        assert!(topo.solid(valid).is_ok());
    }

    #[test]
    fn write_stl_triangle_count_overflow_is_refused() {
        assert_eq!(checked_add_triangle_count(0, 12).unwrap(), 12);
        assert_eq!(
            checked_add_triangle_count(u32::MAX - 5, 5).unwrap(),
            u32::MAX
        );
        assert!(checked_add_triangle_count(u32::MAX, 1).is_err());
        assert!(checked_add_triangle_count(u32::MAX - 5, 6).is_err());
        #[allow(clippy::cast_possible_truncation)]
        let over_u32 = u32::MAX as usize + 1;
        assert!(checked_add_triangle_count(0, over_u32).is_err());

        // 12 triangles serialize to the historical 684-byte payload.
        assert_eq!(checked_binary_len(12).unwrap(), 84 + 12 * 50);
        assert_eq!(checked_binary_len(0).unwrap(), 84);
        // The maximal count still describes a finite (if unallocatable) length
        // on 64-bit; the checked path must agree with u64 arithmetic.
        let max_len = checked_binary_len(u32::MAX).unwrap();
        assert_eq!(
            max_len as u64,
            84u64 + u64::from(u32::MAX) * 50u64,
            "max count length must match checked u64 math"
        );
    }

    #[test]
    fn write_stl_streaming_matches_merged_reference() {
        use remus_operations::tessellate::TriangleMesh;

        // Reference: the pre-PERF-I05 merge (per-solid filter, offset-joined
        // indices, single serialized mesh). Streaming must be byte-identical
        // where the count fits in u32.
        let mut topo = Topology::new();
        let a = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let b = remus_operations::primitives::make_cylinder(&mut topo, 0.5, 2.0).unwrap();
        let solids = [a, b];

        let mut merged = TriangleMesh::default();
        for &solid_id in &solids {
            let mut mesh =
                remus_operations::tessellate::tessellate_solid(&topo, solid_id, 0.1).unwrap();
            crate::retain_nondegenerate_triangles(&mut mesh);
            #[allow(clippy::cast_possible_truncation)]
            let offset = merged.positions.len() as u32;
            merged.positions.extend_from_slice(&mesh.positions);
            merged.normals.extend_from_slice(&mesh.normals);
            merged
                .indices
                .extend(mesh.indices.iter().map(|i| i + offset));
        }

        let expected_binary = write_mesh_stl(&merged, StlFormat::Binary).unwrap();
        let actual_binary = write_stl(&topo, &solids, 0.1, StlFormat::Binary).unwrap();
        assert_eq!(actual_binary, expected_binary);

        let expected_ascii = write_mesh_stl(&merged, StlFormat::Ascii).unwrap();
        let actual_ascii = write_stl(&topo, &solids, 0.1, StlFormat::Ascii).unwrap();
        assert_eq!(actual_ascii, expected_ascii);
    }

    #[test]
    fn write_stl_ascii_multi_matches_binary_counts() {
        let mut topo = Topology::new();
        let a = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let b = remus_operations::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let binary = write_stl(&topo, &[a, b], 0.1, StlFormat::Binary).unwrap();
        let binary_count =
            u32::from_le_bytes([binary[80], binary[81], binary[82], binary[83]]) as usize;

        let ascii = write_stl(&topo, &[a, b], 0.1, StlFormat::Ascii).unwrap();
        let text = String::from_utf8(ascii).unwrap();
        assert!(text.starts_with("solid remus\n"));
        assert!(text.trim().ends_with("endsolid remus"));
        assert_eq!(text.matches("facet normal").count(), binary_count);
        assert_eq!(binary_count, 24);

        // f64-to-f32 narrowing is preserved: 2.0 is exact, and the parsed
        // mesh carries f32-rounded coordinates in original units.
        let mesh = crate::stl::read_stl(&binary).unwrap();
        let mut seen_two = false;
        for pos in &mesh.positions {
            assert!(pos.x() >= -1e-6 && pos.x() <= 2.0 + 1e-6);
            if (pos.x() - 2.0).abs() < 1e-6 {
                seen_two = true;
            }
        }
        assert!(seen_two, "large box extent must survive f32 rounding");
    }
}
