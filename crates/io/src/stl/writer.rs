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
/// every solid. Binary multi-solid export tessellates twice to size its final
/// output buffer exactly before writing. The buffer still scales with file
/// size: this is a bounded intermediate-memory improvement, not constant-memory
/// export.
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
    write_stl_with_tolerance(
        topo,
        solids,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
        format,
    )
}

/// [`write_stl`] with an explicit angular tolerance.
///
/// `angular_tol` caps, in radians, how far the tangent may turn along one
/// segment of a curved edge or surface, on top of the chord-height cap that
/// `deflection` sets. It must lie in `(0, π/2]`. [`write_stl`] passes
/// [`remus_math::chord::DEFAULT_ANGULAR_TOL`].
///
/// # Errors
///
/// As [`write_stl`], and [`OperationsError::InvalidInput`] for an angle that
/// is not finite or lies outside `(0, π/2]`.
///
/// [`OperationsError::InvalidInput`]: remus_operations::OperationsError::InvalidInput
pub fn write_stl_with_tolerance(
    topo: &Topology,
    solids: &[SolidId],
    deflection: f64,
    angular_tol: f64,
    format: StlFormat,
) -> Result<Vec<u8>, crate::IoError> {
    validate_angular_tol(angular_tol)?;
    match format {
        StlFormat::Binary => write_binary_stl_streaming(topo, solids, deflection, angular_tol),
        StlFormat::Ascii => write_ascii_stl_streaming(topo, solids, deflection, angular_tol),
    }
}

/// The largest angular tolerance a writer takes, in radians. A coarser cap
/// would let a quarter circle become one segment.
pub const MAX_ANGULAR_TOL: f64 = std::f64::consts::FRAC_PI_2;

/// Refuse an angular tolerance that is not finite or lies outside
/// `(0, MAX_ANGULAR_TOL]`. Zero would switch the cap off, and a negative or
/// NaN angle would reach the tessellator's internal checks.
///
/// # Errors
///
/// [`remus_operations::OperationsError::InvalidInput`] naming the angle.
pub fn validate_angular_tol(angular_tol: f64) -> Result<(), crate::IoError> {
    if angular_tol.is_finite() && angular_tol > 0.0 && angular_tol <= MAX_ANGULAR_TOL {
        return Ok(());
    }
    Err(remus_operations::OperationsError::InvalidInput {
        reason: format!(
            "angular tolerance must be a finite angle in (0, {MAX_ANGULAR_TOL}] radians, got {angular_tol}"
        ),
    }
    .into())
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
        StlFormat::Binary => {
            check_binary_precision(&filtered)?;
            write_binary_stl(&filtered)
        }
        StlFormat::Ascii => write_ascii_stl(&filtered),
    }
}

/// Refuse a mesh that the `f32` coordinates of a binary STL would damage.
///
/// Far from the origin `f32` resolves only a few hundredths of a millimetre,
/// so points a fine tessellation keeps apart in `f64` can meet, or fall on one
/// line, once written. A facet would then be written with zero area, or two
/// vertices would merge, which opens the mesh or makes an edge carry more
/// than two facets for a reader that joins equal points. Both are refused
/// rather than written; the mesh itself is never changed, so a mesh that
/// passes is written exactly as before.
///
/// # Errors
///
/// [`crate::IoError::InvalidTopology`] naming the first facet or vertex pair
/// that `f32` damages.
fn check_binary_precision(mesh: &TriangleMesh) -> Result<(), crate::IoError> {
    let rounded = |p: Point3| Point3::new(to_f32(p.x()), to_f32(p.y()), to_f32(p.z()));
    let refuse = |what: String| crate::IoError::InvalidTopology {
        reason: format!(
            "{what}: a binary STL stores coordinates as f32, which cannot keep this \
             mesh apart at its distance from the origin; translate the part nearer the \
             origin or write ASCII STL"
        ),
    };

    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| rounded(mesh.positions[i as usize]));
        if (b - a).cross(c - a).length_squared() == 0.0 {
            return Err(refuse(format!("facet {t} has zero area in f32")));
        }
    }

    // Two vertices a facet uses, at different f64 positions, that land on one
    // f32 position. Each used vertex is sorted once, by its f32 bits.
    let mut used = vec![false; mesh.positions.len()];
    for &i in &mesh.indices {
        used[i as usize] = true;
    }
    #[allow(clippy::cast_possible_truncation)]
    let narrow = |v: f64| (v as f32).to_bits();
    let mut landing: Vec<([u32; 3], [u64; 3])> = mesh
        .positions
        .iter()
        .zip(&used)
        .filter(|&(_, &is_used)| is_used)
        .map(|(&p, _)| {
            (
                [narrow(p.x()), narrow(p.y()), narrow(p.z())],
                [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()],
            )
        })
        .collect();
    landing.sort_unstable();
    if let Some(pair) = landing
        .windows(2)
        .find(|pair| pair[0].0 == pair[1].0 && pair[0].1 != pair[1].1)
    {
        let p = pair[0].0.map(f32::from_bits);
        return Err(refuse(format!(
            "two distinct vertices meet at ({}, {}, {}) in f32",
            p[0], p[1], p[2]
        )));
    }
    Ok(())
}

/// `v` as the nearest `f32`, back in `f64`.
#[allow(clippy::cast_possible_truncation)]
fn to_f32(v: f64) -> f64 {
    f64::from(v as f32)
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
/// One solid's mesh is live at a time. Multi-solid export counts filtered
/// facets first, then allocates the exact output length and tessellates again
/// to write them. A single solid needs only one tessellation. Any
/// tessellation, count, or size failure returns `Err` without a partial output.
fn write_binary_stl_streaming(
    topo: &Topology,
    solids: &[SolidId],
    deflection: f64,
    angular_tol: f64,
) -> Result<Vec<u8>, crate::IoError> {
    if let [solid_id] = solids {
        let mut mesh =
            tessellate::tessellate_solid_for_analysis(topo, *solid_id, deflection, angular_tol)?;
        crate::retain_nondegenerate_triangles(&mut mesh);
        check_binary_precision(&mesh)?;
        return write_binary_stl(&mesh);
    }

    let mut total = 0;
    for &solid_id in solids {
        let mut mesh =
            tessellate::tessellate_solid_for_analysis(topo, solid_id, deflection, angular_tol)?;
        crate::retain_nondegenerate_triangles(&mut mesh);
        check_binary_precision(&mesh)?;
        total = checked_add_triangle_count(total, mesh.indices.len() / 3)?;
    }

    let mut buf = Vec::with_capacity(checked_binary_len(total)?);
    let header = b"remus STL export";
    buf.extend_from_slice(header);
    // The STL binary header is a fixed 80 bytes; zero-pad whatever the
    // header string didn't fill.
    buf.resize(80, 0);
    buf.extend_from_slice(&total.to_le_bytes());

    let mut written = 0;
    for &solid_id in solids {
        let mut mesh =
            tessellate::tessellate_solid_for_analysis(topo, solid_id, deflection, angular_tol)?;
        crate::retain_nondegenerate_triangles(&mut mesh);
        written = checked_add_triangle_count(written, mesh.indices.len() / 3)?;
        if written > total {
            return Err(crate::IoError::InvalidTopology {
                reason: "STL triangle count changed between sizing and writing".to_string(),
            });
        }
        append_mesh_binary(&mut buf, &mesh);
    }

    if written != total {
        return Err(crate::IoError::InvalidTopology {
            reason: "STL triangle count changed between sizing and writing".to_string(),
        });
    }
    debug_assert_eq!(buf.len(), checked_binary_len(total)?);
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
    angular_tol: f64,
) -> Result<Vec<u8>, crate::IoError> {
    let mut buf = Vec::new();
    writeln!(buf, "solid remus").map_err(crate::IoError::Io)?;

    for &solid_id in solids {
        let mut mesh =
            tessellate::tessellate_solid_for_analysis(topo, solid_id, deflection, angular_tol)?;
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
    fn write_stl_angular_tolerance_caps_curved_segments() {
        // At a 1 mm deflection the chord-height cap allows coarse segments on a
        // 2 mm radius, so the angular cap is what sets the segment count.
        let mut topo = Topology::new();
        let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 2.0, 1.0).unwrap();
        let count = |angular: f64, format: StlFormat| {
            let bytes = write_stl_with_tolerance(&topo, &[cylinder], 1.0, angular, format).unwrap();
            crate::stl::read_stl(&bytes).unwrap().indices.len() / 3
        };

        let default = write_stl(&topo, &[cylinder], 1.0, StlFormat::Binary).unwrap();
        assert_eq!(
            default,
            write_stl_with_tolerance(
                &topo,
                &[cylinder],
                1.0,
                remus_math::chord::DEFAULT_ANGULAR_TOL,
                StlFormat::Binary,
            )
            .unwrap(),
            "write_stl is the default angular tolerance"
        );

        let coarse = count(remus_math::chord::DEFAULT_ANGULAR_TOL, StlFormat::Binary);
        let fine = count(5f64.to_radians(), StlFormat::Binary);
        assert!(
            fine >= coarse * 3,
            "a 5 degree cap should need at least three times the triangles of a 20 degree cap, got {fine} against {coarse}"
        );
        assert_eq!(fine, count(5f64.to_radians(), StlFormat::Ascii));
    }

    #[test]
    fn write_stl_with_tolerance_refuses_invalid_angles() {
        let mut topo = Topology::new();
        let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 2.0, 1.0).unwrap();
        for angle in [
            0.0,
            -0.1,
            f64::NAN,
            f64::INFINITY,
            MAX_ANGULAR_TOL * 1.01,
            1e9,
        ] {
            for format in [StlFormat::Binary, StlFormat::Ascii] {
                let result = write_stl_with_tolerance(&topo, &[cylinder], 0.1, angle, format);
                assert!(
                    matches!(
                        result,
                        Err(crate::IoError::Operations(
                            remus_operations::OperationsError::InvalidInput { .. }
                        ))
                    ),
                    "{angle} ({format:?}) should be refused"
                );
            }
        }
        assert!(
            write_stl_with_tolerance(&topo, &[cylinder], 0.1, MAX_ANGULAR_TOL, StlFormat::Binary)
                .is_ok()
        );
    }

    /// Weld an STL read back (one vertex per facet corner) by exact position.
    fn welded(mesh: &TriangleMesh) -> TriangleMesh {
        let mut ids = std::collections::BTreeMap::new();
        TriangleMesh {
            indices: mesh
                .positions
                .iter()
                .map(|p| {
                    let key = [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()];
                    let next = u32::try_from(ids.len()).unwrap();
                    *ids.entry(key).or_insert(next)
                })
                .collect(),
            positions: Vec::new(),
            normals: Vec::new(),
        }
    }

    /// A written STL reads back closed, 2-manifold and free of zero-area facets.
    fn assert_sound(bytes: &[u8], what: &str) {
        let mesh = crate::stl::read_stl(bytes).unwrap();
        let zero_area = mesh
            .indices
            .chunks_exact(3)
            .filter(|t| {
                let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize]);
                (b - a).cross(c - a).length_squared() == 0.0
            })
            .count();
        assert_eq!(zero_area, 0, "{what}: zero-area facets");
        let joined = welded(&mesh);
        assert_eq!(
            remus_operations::tessellate::boundary_edge_count(&joined),
            0,
            "{what}: open edges"
        );
        assert_eq!(
            remus_operations::tessellate::non_manifold_edge_count(&joined),
            0,
            "{what}: edges used by more than two facets"
        );
    }

    fn assert_precision_refusal(result: Result<Vec<u8>, crate::IoError>, what: &str) {
        let reason = match result {
            Err(crate::IoError::InvalidTopology { reason }) => reason,
            Err(other) => format!("wrong error: {other}"),
            Ok(bytes) => format!("wrote {} bytes", bytes.len()),
        };
        assert!(
            reason.contains("f32") && reason.contains("ASCII"),
            "{what}: {reason}"
        );
    }

    fn moved(topo: &mut Topology, solid: SolidId, x: f64, y: f64) -> SolidId {
        remus_operations::transform::transform_solid(
            topo,
            solid,
            &remus_math::mat::Mat4::translation(x, y, 0.0),
        )
        .unwrap();
        solid
    }

    #[test]
    fn binary_stl_refuses_meshes_f32_would_damage() {
        let five_degrees = 5f64.to_radians();
        let mut topo = Topology::new();

        // Four facets fall on one line in f32.
        let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 0.5, 1.0).unwrap();
        let cylinder = moved(&mut topo, cylinder, 300_000.0, 300_000.0);
        // Vertices meet in f32 and edges would carry more than two facets.
        let sphere = remus_operations::primitives::make_sphere(&mut topo, 0.5, 16).unwrap();
        let sphere = moved(&mut topo, sphere, 1_000_000.0, 1_000_000.0);
        // At 1e7 f32 steps by 1 mm, and the whole cylinder collapses.
        let vanishing = remus_operations::primitives::make_cylinder(&mut topo, 0.5, 1.0).unwrap();
        let vanishing = moved(&mut topo, vanishing, 10_000_000.0, 10_000_000.0);
        let unit_box = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        for (what, solids) in [
            ("cylinder at 3e5", vec![cylinder]),
            ("sphere at 1e6", vec![sphere]),
            ("cylinder at 1e7", vec![vanishing]),
            ("box with the cylinder at 1e7", vec![unit_box, vanishing]),
            ("cylinder at 1e7 with the box", vec![vanishing, unit_box]),
        ] {
            assert_precision_refusal(
                write_stl_with_tolerance(&topo, &solids, 0.01, five_degrees, StlFormat::Binary),
                what,
            );
            // ASCII writes f64, so the same solids export sound.
            if what != "cylinder at 1e7" && !what.contains("box") {
                let ascii =
                    write_stl_with_tolerance(&topo, &solids, 0.01, five_degrees, StlFormat::Ascii)
                        .unwrap();
                assert_sound(&ascii, what);
            }
        }
        // The box alone still exports.
        assert_sound(
            &write_stl(&topo, &[unit_box], 0.01, StlFormat::Binary).unwrap(),
            "box",
        );
    }

    #[test]
    fn binary_stl_refuses_a_mesh_whose_curved_edge_flattens_in_f32() {
        // A closed pyramid whose base edge bows outward by 1e-4 mm, near
        // (3e5, 3e5, 0) where f32 steps by about 0.03 mm. In f32 the arc is a
        // straight line and its thin base facets have no area.
        let (x, y) = (300_000.0, 300_000.0);
        let mut positions = vec![Point3::new(x + 5.0, y + 5.0, 10.0)];
        let arc = 8;
        for i in 0..=arc {
            let t = f64::from(i) / f64::from(arc);
            let bow = 1e-4 * (std::f64::consts::PI * t).sin();
            positions.push(Point3::new(x + 10.0 * t, y - bow, 0.0));
        }
        positions.push(Point3::new(x + 10.0, y + 10.0, 0.0));
        positions.push(Point3::new(x, y + 10.0, 0.0));
        let base: Vec<u32> = (1..=u32::try_from(arc + 3).unwrap()).collect();
        let mut indices = Vec::new();
        for k in 0..base.len() {
            let (a, b) = (base[k], base[(k + 1) % base.len()]);
            indices.extend_from_slice(&[0, a, b]);
        }
        // The base is a fan from the first point of the arc, so it holds thin
        // facets whose three corners all lie on the arc.
        for k in 1..base.len() - 1 {
            indices.extend_from_slice(&[base[0], base[k + 1], base[k]]);
        }
        let mesh = TriangleMesh {
            normals: vec![Vec3::new(0.0, 0.0, 1.0); positions.len()],
            positions,
            indices,
        };
        // Closed and sound in f64.
        assert_eq!(remus_operations::tessellate::boundary_edge_count(&mesh), 0);
        assert_eq!(
            remus_operations::tessellate::non_manifold_edge_count(&mesh),
            0
        );
        assert_precision_refusal(write_mesh_stl(&mesh, StlFormat::Binary), "pyramid");
        assert_sound(
            &write_mesh_stl(&mesh, StlFormat::Ascii).unwrap(),
            "pyramid in ASCII",
        );
    }

    #[test]
    fn binary_stl_refuses_vertices_that_meet_in_f32() {
        // Two closed tetrahedra whose nearest corners lie 1e-4 mm apart near
        // (3e5, 3e5, 0). No facet flattens, but the two corners become one
        // point, and the edges there would join four facets.
        let (x, y) = (300_000.0, 300_000.0);
        let tetra = |dx: f64, sign: f64| {
            [
                Point3::new(x + dx, y, 0.0),
                Point3::new(x + dx + sign * 10.0, y, 0.0),
                Point3::new(x + dx + sign * 5.0, y + 10.0, 0.0),
                Point3::new(x + dx + sign * 5.0, y + 3.0, 10.0),
            ]
        };
        let mut positions = tetra(0.0, -1.0).to_vec();
        positions.extend(tetra(1e-4, 1.0));
        let faces = |o: u32, flip: bool| {
            let f = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
            f.map(|[a, b, c]| {
                if flip {
                    [o + a, o + c, o + b]
                } else {
                    [o + a, o + b, o + c]
                }
            })
        };
        let indices: Vec<u32> = faces(0, false)
            .into_iter()
            .chain(faces(4, true))
            .flatten()
            .collect();
        let mesh = TriangleMesh {
            normals: vec![Vec3::new(0.0, 0.0, 1.0); positions.len()],
            positions,
            indices,
        };
        assert_eq!(remus_operations::tessellate::boundary_edge_count(&mesh), 0);
        match write_mesh_stl(&mesh, StlFormat::Binary) {
            Err(crate::IoError::InvalidTopology { reason }) => {
                assert!(reason.contains("meet"), "{reason}");
            }
            other => assert!(other.is_err(), "wrote the merged corners"),
        }
        assert_sound(
            &write_mesh_stl(&mesh, StlFormat::Ascii).unwrap(),
            "tetrahedra in ASCII",
        );
    }

    #[test]
    fn binary_stl_away_from_the_origin_stays_sound() {
        let five_degrees = 5f64.to_radians();
        let mut topo = Topology::new();
        let mut solids = Vec::new();
        for offset in [0.0, 1_000.0, 50_000.0] {
            let cylinder =
                remus_operations::primitives::make_cylinder(&mut topo, 2.0, 5.0).unwrap();
            let sphere = remus_operations::primitives::make_sphere(&mut topo, 3.0, 16).unwrap();
            let torus = remus_operations::primitives::make_torus(&mut topo, 6.0, 1.5, 16).unwrap();
            for solid in [cylinder, sphere, torus] {
                solids.push(moved(&mut topo, solid, offset, offset));
            }
        }
        for solid in &solids {
            let bytes = write_stl_with_tolerance(
                &topo,
                std::slice::from_ref(solid),
                0.01,
                five_degrees,
                StlFormat::Binary,
            )
            .unwrap();
            assert_sound(&bytes, "binary");
        }
    }

    // From d69ed9c6's writer: `write_stl` at 0.05 mm, binary.
    const GOLDEN_CYLINDER: (usize, u64) = (3484, 0x9ed8_9caa_4689_294f);
    const GOLDEN_THREE: (usize, u64) = (217_584, 0x8a5b_0d4d_5f74_3bea);

    /// FNV-1a, enough to pin the bytes of an export in a test.
    fn fnv1a(bytes: &[u8]) -> u64 {
        bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, &byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
        })
    }

    #[test]
    fn default_binary_stl_bytes_are_unchanged() {
        // The digests of the writer on main before the angular tolerance and
        // the f32 check, for the same solids.
        let mut topo = Topology::new();
        let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 1.0, 5.0).unwrap();
        let sphere = remus_operations::primitives::make_sphere(&mut topo, 1.0, 12).unwrap();
        let torus = remus_operations::primitives::make_torus(&mut topo, 3.0, 1.0, 12).unwrap();
        let cases = [
            (vec![cylinder], GOLDEN_CYLINDER),
            (vec![cylinder, sphere, torus], GOLDEN_THREE),
        ];
        for (solids, golden) in cases {
            let bytes = write_stl(&topo, &solids, 0.05, StlFormat::Binary).unwrap();
            assert_eq!((bytes.len(), fnv1a(&bytes)), golden);
        }
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
