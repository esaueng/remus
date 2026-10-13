//! glTF 2.0 binary (.glb) reader.
//!
//! Reads mesh geometry (positions, normals, triangle indices) from GLB files.
//! Supports multiple meshes/primitives, dynamic accessor indices, and both
//! uint16 (5123) and uint32 (5125) index component types.

use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Read a GLB (glTF binary) file and return a triangle mesh.
///
/// Extracts vertex positions, normals, and triangle indices from all
/// mesh primitives in the file, combining them into a single mesh. Missing
/// normals are represented by zero vectors, preserving primitive alignment
/// without supplying an invented direction to solid import.
///
/// # Errors
///
/// Returns an error if the file is malformed or uses unsupported features.
#[allow(clippy::too_many_lines)]
pub fn read_glb(data: &[u8]) -> Result<TriangleMesh, crate::IoError> {
    read_glb_with_limits(data, ImportLimits::default())
}

/// Read a GLB file with explicit hostile-input resource limits.
///
/// # Errors
///
/// Returns [`crate::IoError`] when a limit is exceeded or the GLB is malformed.
#[allow(clippy::too_many_lines)]
pub fn read_glb_with_limits(
    data: &[u8],
    limits: ImportLimits,
) -> Result<TriangleMesh, crate::IoError> {
    ensure_input_size(data.len(), limits)?;
    if data.len() < 12 {
        return Err(crate::IoError::ParseError {
            reason: "GLB too short for header".into(),
        });
    }

    let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    if magic != 0x4654_6C67 {
        return Err(crate::IoError::ParseError {
            reason: "not a GLB file (invalid magic)".into(),
        });
    }

    let version = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    if version != 2 {
        return Err(crate::IoError::ParseError {
            reason: format!("unsupported glTF version {version}"),
        });
    }

    let mut offset = 12usize;
    let mut json_data: Option<&[u8]> = None;
    let mut bin_data: Option<&[u8]> = None;

    while let Some(hdr_end) = offset.checked_add(8) {
        if hdr_end > data.len() {
            break;
        }
        let chunk_len = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]) as usize;
        let chunk_type = u32::from_le_bytes([
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        offset = hdr_end;

        // checked_add keeps the bounds test sound on narrow (32-bit) targets,
        // where a near-u32::MAX chunk length would otherwise wrap below len.
        let Some(chunk_end) = offset.checked_add(chunk_len) else {
            break;
        };
        if chunk_end > data.len() {
            break;
        }

        match chunk_type {
            0x4E4F_534A => json_data = Some(&data[offset..chunk_end]), // JSON
            0x004E_4942 => bin_data = Some(&data[offset..chunk_end]),  // BIN
            _ => {}                                                    // skip unknown
        }

        offset = chunk_end;
    }

    let json_bytes = json_data.ok_or_else(|| crate::IoError::ParseError {
        reason: "GLB missing JSON chunk".into(),
    })?;
    let bin = bin_data.ok_or_else(|| crate::IoError::ParseError {
        reason: "GLB missing BIN chunk".into(),
    })?;

    let json_str = std::str::from_utf8(json_bytes).map_err(|_| crate::IoError::ParseError {
        reason: "GLB JSON is not valid UTF-8".into(),
    })?;

    let document: GltfDocument =
        serde_json::from_str(json_str).map_err(|error| crate::IoError::ParseError {
            reason: format!("invalid GLB JSON: {error}"),
        })?;
    let accessors = document.accessors;
    let declared_accessor_entities = accessors
        .iter()
        .try_fold(0usize, |total, accessor| total.checked_add(accessor.count))
        .ok_or(crate::IoError::LimitExceeded {
            resource: "GLB accessor entities",
            limit: limits.max_model_entities,
            actual: usize::MAX,
        })?;
    ensure_limit(
        "GLB accessor entities",
        declared_accessor_entities,
        limits.max_model_entities,
    )?;
    let buffer_views = document.buffer_views;
    let primitives: Vec<_> = document
        .meshes
        .into_iter()
        .flat_map(|mesh| mesh.primitives)
        .collect();

    if primitives.is_empty() {
        return Err(crate::IoError::ParseError {
            reason: "GLB has no mesh primitives".into(),
        });
    }

    // Shared accessors are materialized once per primitive reference. Charge
    // that expanded work before any decoder allocates its output, including
    // normal accessors whose counts may later fail validation.
    ensure_limit(
        "GLB primitives",
        primitives.len(),
        limits.max_model_entities,
    )?;
    let mut decoded_entities = 0usize;
    for prim in &primitives {
        if prim.mode != 4 {
            return Err(crate::IoError::ParseError {
                reason: "unsupported GLB primitive mode (expected TRIANGLES=4)".into(),
            });
        }
        for index in [
            prim.attributes.position_accessor,
            prim.attributes.normal_accessor,
            prim.indices_accessor,
        ]
        .into_iter()
        .flatten()
        {
            if let Some(accessor) = accessors.get(index) {
                decoded_entities = decoded_entities.checked_add(accessor.count).ok_or(
                    crate::IoError::LimitExceeded {
                        resource: "GLB decoded accessor entities",
                        limit: limits.max_model_entities,
                        actual: usize::MAX,
                    },
                )?;
                ensure_limit(
                    "GLB decoded accessor entities",
                    decoded_entities,
                    limits.max_model_entities,
                )?;
            }
        }
        // Unindexed TRIANGLES materialize an implicit sequential index for
        // each vertex. Charge that output before allocating either vector.
        if prim.indices_accessor.is_none()
            && let Some(accessor) = prim
                .attributes
                .position_accessor
                .and_then(|i| accessors.get(i))
        {
            decoded_entities = decoded_entities.checked_add(accessor.count).ok_or(
                crate::IoError::LimitExceeded {
                    resource: "GLB decoded accessor entities",
                    limit: limits.max_model_entities,
                    actual: usize::MAX,
                },
            )?;
            ensure_limit(
                "GLB decoded accessor entities",
                decoded_entities,
                limits.max_model_entities,
            )?;
        }
    }

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();

    for prim in &primitives {
        let vertex_offset = positions.len();

        if let Some(pos_idx) = prim.attributes.position_accessor {
            let accessor = accessors
                .get(pos_idx)
                .ok_or_else(|| crate::IoError::ParseError {
                    reason: format!("POSITION accessor index {pos_idx} out of range"),
                })?;
            let view = buffer_views.get(accessor.buffer_view).ok_or_else(|| {
                crate::IoError::ParseError {
                    reason: format!("buffer view index {} out of range", accessor.buffer_view),
                }
            })?;
            for [x, y, z] in decode_accessor_vertices(accessor, view, bin, "POSITION")? {
                positions.push(Point3::new(f64::from(x), f64::from(y), f64::from(z)));
            }
        }

        let vertex_count = positions.len() - vertex_offset;
        if let Some(norm_idx) = prim.attributes.normal_accessor {
            let accessor = accessors
                .get(norm_idx)
                .ok_or_else(|| crate::IoError::ParseError {
                    reason: format!("NORMAL accessor index {norm_idx} out of range"),
                })?;
            if accessor.count != vertex_count {
                return Err(crate::IoError::ParseError {
                    reason: format!(
                        "NORMAL count {} does not match POSITION count {vertex_count}",
                        accessor.count
                    ),
                });
            }
            let view = buffer_views.get(accessor.buffer_view).ok_or_else(|| {
                crate::IoError::ParseError {
                    reason: format!(
                        "NORMAL buffer view index {} out of range",
                        accessor.buffer_view
                    ),
                }
            })?;
            for [x, y, z] in decode_accessor_vertices(accessor, view, bin, "NORMAL")? {
                normals.push(Vec3::new(f64::from(x), f64::from(y), f64::from(z)));
            }
        } else {
            // Zero means no supplied normal. Keep primitive alignment without
            // inventing a direction that import_mesh would use to flip winding.
            normals.resize(positions.len(), Vec3::new(0.0, 0.0, 0.0));
        }

        if let Some(idx_accessor_idx) = prim.indices_accessor {
            let accessor =
                accessors
                    .get(idx_accessor_idx)
                    .ok_or_else(|| crate::IoError::ParseError {
                        reason: format!("indices accessor index {idx_accessor_idx} out of range"),
                    })?;
            let view = buffer_views.get(accessor.buffer_view).ok_or_else(|| {
                crate::IoError::ParseError {
                    reason: format!("buffer view index {} out of range", accessor.buffer_view),
                }
            })?;
            if accessor.element_type != "SCALAR"
                || accessor.normalized
                || accessor.sparse.is_some()
                || view.byte_stride.is_some()
            {
                return Err(crate::IoError::ParseError {
                    reason: "unsupported GLB index accessor layout".into(),
                });
            }

            let v_offset =
                u32::try_from(vertex_offset).map_err(|_| crate::IoError::ParseError {
                    reason: "glTF vertex offset exceeds u32".into(),
                })?;

            // Decode only the declared count, even when the view is shared.
            let elem_bytes = match accessor.component_type {
                5123 => 2,
                5125 => 4,
                ct => {
                    return Err(crate::IoError::ParseError {
                        reason: format!(
                            "unsupported index component type {ct} (expected 5123=uint16 or 5125=uint32)"
                        ),
                    });
                }
            };
            let (idx_data, _) = accessor_data(accessor, view, bin, elem_bytes, "indices")?;
            match accessor.component_type {
                5123 => {
                    // uint16
                    for chunk in idx_data.chunks_exact(2) {
                        let idx = u16::from_le_bytes([chunk[0], chunk[1]]);
                        let global = u32::from(idx).checked_add(v_offset).ok_or_else(|| {
                            crate::IoError::ParseError {
                                reason: "glTF index + vertex offset overflows u32".into(),
                            }
                        })?;
                        if usize::from(idx) >= vertex_count {
                            return Err(crate::IoError::ParseError {
                                reason: "GLB index exceeds primitive POSITION count".into(),
                            });
                        }
                        indices.push(global);
                    }
                }
                5125 => {
                    // uint32
                    for chunk in idx_data.chunks_exact(4) {
                        let idx = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                        let global = idx.checked_add(v_offset).ok_or_else(|| {
                            crate::IoError::ParseError {
                                reason: "glTF index + vertex offset overflows u32".into(),
                            }
                        })?;
                        if idx as usize >= vertex_count {
                            return Err(crate::IoError::ParseError {
                                reason: "GLB index exceeds primitive POSITION count".into(),
                            });
                        }
                        indices.push(global);
                    }
                }
                ct => {
                    return Err(crate::IoError::ParseError {
                        reason: format!(
                            "unsupported index component type {ct} (expected 5123=uint16 or 5125=uint32)"
                        ),
                    });
                }
            }
        } else {
            if !vertex_count.is_multiple_of(3) {
                return Err(crate::IoError::ParseError {
                    reason: "unindexed TRIANGLES POSITION count is not a multiple of three".into(),
                });
            }
            for index in vertex_offset..positions.len() {
                indices.push(
                    u32::try_from(index).map_err(|_| crate::IoError::ParseError {
                        reason: "glTF implicit index exceeds u32".into(),
                    })?,
                );
            }
        }
    }

    if positions.is_empty() {
        return Err(crate::IoError::ParseError {
            reason: "GLB contains no vertex data".into(),
        });
    }

    Ok(TriangleMesh {
        positions,
        normals,
        indices,
    })
}

/// Safe sub-slice with bounds checking.
fn safe_slice(data: &[u8], offset: usize, length: usize) -> Result<&[u8], crate::IoError> {
    let end = offset
        .checked_add(length)
        .ok_or_else(|| crate::IoError::ParseError {
            reason: "buffer view offset + length overflow".into(),
        })?;
    if end > data.len() {
        return Err(crate::IoError::ParseError {
            reason: format!(
                "buffer view [{offset}..{end}] exceeds binary buffer length {}",
                data.len()
            ),
        });
    }
    Ok(&data[offset..end])
}

// Deserialize metadata as JSON, retaining every array slot and validating
// integer widths rather than dropping malformed entries and shifting indices.
#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct GltfDocument {
    #[serde(default)]
    accessors: Vec<AccessorInfo>,
    #[serde(default)]
    buffer_views: Vec<BufferViewInfo>,
    #[serde(default)]
    meshes: Vec<GltfMesh>,
}

#[derive(serde::Deserialize)]
struct GltfMesh {
    primitives: Vec<MeshPrimitive>,
}

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct AccessorInfo {
    buffer_view: usize,
    #[serde(deserialize_with = "component_type")]
    component_type: u32,
    count: usize,
    #[serde(rename = "type")]
    element_type: String,
    #[serde(default)]
    byte_offset: usize,
    #[serde(default)]
    normalized: bool,
    sparse: Option<serde_json::Value>,
}

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct BufferViewInfo {
    #[serde(default)]
    buffer: usize,
    #[serde(default)]
    byte_offset: usize,
    byte_length: usize,
    byte_stride: Option<usize>,
}

#[derive(serde::Deserialize)]
struct MeshAttributes {
    #[serde(rename = "POSITION", default, deserialize_with = "optional_index")]
    position_accessor: Option<usize>,
    #[serde(rename = "NORMAL", default, deserialize_with = "optional_index")]
    normal_accessor: Option<usize>,
}

#[derive(serde::Deserialize)]
struct MeshPrimitive {
    attributes: MeshAttributes,
    #[serde(rename = "indices", default, deserialize_with = "indices_accessor")]
    indices_accessor: Option<usize>,
    #[serde(default = "triangle_mode", deserialize_with = "primitive_mode")]
    mode: u64,
}

// A missing optional index is legal; an explicitly null/negative/fractional
// index is malformed and must not silently select implicit indices instead.
fn optional_index<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<usize>, D::Error> {
    <usize as serde::Deserialize>::deserialize(deserializer).map(Some)
}

fn indices_accessor<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<usize>, D::Error> {
    optional_index(deserializer).map_err(|error| {
        <D::Error as serde::de::Error>::custom(format!("invalid GLB indices accessor: {error}"))
    })
}

fn component_type<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<u32, D::Error> {
    <u32 as serde::Deserialize>::deserialize(deserializer).map_err(|error| {
        <D::Error as serde::de::Error>::custom(format!("invalid GLB component type: {error}"))
    })
}

fn primitive_mode<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<u64, D::Error> {
    <u64 as serde::Deserialize>::deserialize(deserializer).map_err(|error| {
        <D::Error as serde::de::Error>::custom(format!("invalid GLB primitive mode: {error}"))
    })
}

const fn triangle_mode() -> u64 {
    4
}

/// Bound the exact referenced subrange before allocation. Trailing view data
/// is legal; it is never decoded or charged as additional accessor output.
fn accessor_data<'a>(
    accessor: &AccessorInfo,
    view: &BufferViewInfo,
    bin: &'a [u8],
    element_bytes: usize,
    kind: &str,
) -> Result<(&'a [u8], usize), crate::IoError> {
    let invalid = || crate::IoError::ParseError {
        reason: format!("GLB {kind} accessor range or stride exceeds its buffer view"),
    };
    if view.buffer != 0 {
        return Err(invalid());
    }
    let data = safe_slice(bin, view.byte_offset, view.byte_length)?;
    let stride = view.byte_stride.unwrap_or(element_bytes);
    if stride < element_bytes
        || (view.byte_stride.is_some() && (stride > 252 || !stride.is_multiple_of(4)))
    {
        return Err(invalid());
    }
    let extent = if accessor.count == 0 {
        0
    } else {
        (accessor.count - 1)
            .checked_mul(stride)
            .and_then(|n| n.checked_add(element_bytes))
            .ok_or_else(invalid)?
    };
    let end = accessor
        .byte_offset
        .checked_add(extent)
        .ok_or_else(invalid)?;
    let referenced = data.get(accessor.byte_offset..end).ok_or_else(invalid)?;
    Ok((referenced, stride))
}

/// Decode exactly count VEC3/FLOAT values using the accessor's offset and
/// buffer view's stride. Sparse and normalized layouts remain unsupported.
fn decode_accessor_vertices(
    accessor: &AccessorInfo,
    view: &BufferViewInfo,
    bin: &[u8],
    kind: &str,
) -> Result<Vec<[f32; 3]>, crate::IoError> {
    if accessor.component_type != 5126
        || accessor.element_type != "VEC3"
        || accessor.normalized
        || accessor.sparse.is_some()
    {
        return Err(crate::IoError::ParseError {
            reason: format!("unsupported GLB {kind} accessor (expected nonsparse VEC3/FLOAT)"),
        });
    }
    let (data, stride) = accessor_data(accessor, view, bin, 12, kind)?;
    let mut out = Vec::with_capacity(accessor.count);
    for chunk in data.chunks(stride).take(accessor.count) {
        let x = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        let y = f32::from_le_bytes([chunk[4], chunk[5], chunk[6], chunk[7]]);
        let z = f32::from_le_bytes([chunk[8], chunk[9], chunk[10], chunk[11]]);
        out.push([x, y, z]);
    }
    Ok(out)
}

/// Read a GLB (glTF binary) file and import it as a solid with one planar
/// face per triangle.
///
/// This is a convenience wrapper that calls [`read_glb`] followed by
/// [`import_mesh`](crate::stl::import::import_mesh). Vertices within
/// `tolerance` of each other are merged.
///
/// # Errors
///
/// Returns [`IoError`](crate::IoError) if the file is malformed or the mesh
/// cannot be converted to a valid solid.
pub fn read_glb_solid(
    topo: &mut Topology,
    data: &[u8],
    tolerance: f64,
) -> Result<SolidId, crate::IoError> {
    let mesh = read_glb(data)?;
    crate::stl::import::import_mesh(topo, &mesh, tolerance)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_glb() {
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let glb = crate::gltf::write_glb(&topo, &[solid], 0.1).unwrap();
        let mesh = read_glb(&glb).unwrap();

        assert!(!mesh.positions.is_empty(), "should have vertices");
        assert!(!mesh.indices.is_empty(), "should have indices");
        assert_eq!(mesh.indices.len() % 3, 0, "should be triangles");
    }

    #[test]
    fn invalid_magic() {
        let data = vec![0u8; 20];
        assert!(read_glb(&data).is_err());
    }

    #[test]
    fn too_short() {
        let data = vec![0u8; 4];
        assert!(read_glb(&data).is_err());
    }

    #[test]
    fn parse_primitives_from_json() {
        let json = r#"{"meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1},"indices":2}]}],"accessors":[{"bufferView":0,"componentType":5126,"count":4,"type":"VEC3"},{"bufferView":1,"componentType":5126,"count":4,"type":"VEC3"},{"bufferView":2,"componentType":5125,"count":6,"type":"SCALAR"}]}"#;

        let prims = serde_json::from_str::<GltfDocument>(json)
            .unwrap()
            .meshes
            .into_iter()
            .flat_map(|mesh| mesh.primitives)
            .collect::<Vec<_>>();
        assert_eq!(prims.len(), 1);
        assert_eq!(prims[0].attributes.position_accessor, Some(0));
        assert_eq!(prims[0].attributes.normal_accessor, Some(1));
        assert_eq!(prims[0].indices_accessor, Some(2));
    }

    #[test]
    fn parse_primitives_non_default_indices() {
        // Accessors in a different order: positions=2, normals=3, indices=4
        let json =
            r#"{"meshes":[{"primitives":[{"attributes":{"POSITION":2,"NORMAL":3},"indices":4}]}]}"#;

        let prims = serde_json::from_str::<GltfDocument>(json)
            .unwrap()
            .meshes
            .into_iter()
            .flat_map(|mesh| mesh.primitives)
            .collect::<Vec<_>>();
        assert_eq!(prims.len(), 1);
        assert_eq!(prims[0].attributes.position_accessor, Some(2));
        assert_eq!(prims[0].attributes.normal_accessor, Some(3));
        assert_eq!(prims[0].indices_accessor, Some(4));
    }

    #[test]
    fn parse_multi_mesh_primitives() {
        let json = r#"{"meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1},"indices":2}]},{"primitives":[{"attributes":{"POSITION":3,"NORMAL":4},"indices":5}]}]}"#;

        let prims = serde_json::from_str::<GltfDocument>(json)
            .unwrap()
            .meshes
            .into_iter()
            .flat_map(|mesh| mesh.primitives)
            .collect::<Vec<_>>();
        assert_eq!(prims.len(), 2);
        assert_eq!(prims[0].attributes.position_accessor, Some(0));
        assert_eq!(prims[1].attributes.position_accessor, Some(3));
        assert_eq!(prims[1].indices_accessor, Some(5));
    }

    #[test]
    fn parse_accessor_component_type() {
        let json = r#"{"accessors":[{"bufferView":0,"componentType":5126,"count":4,"type":"VEC3"},{"bufferView":1,"componentType":5123,"count":6,"type":"SCALAR"}]}"#;
        let accessors = serde_json::from_str::<GltfDocument>(json)
            .unwrap()
            .accessors;
        assert_eq!(accessors.len(), 2);
        assert_eq!(accessors[0].component_type, 5126);
        assert_eq!(accessors[1].component_type, 5123);
    }

    #[test]
    fn read_uint16_indices() {
        // Build a minimal GLB with uint16 indices manually.
        // 1 triangle: 3 positions + 3 normals + 3 uint16 indices

        // Positions: 3 vertices (3 * 12 bytes = 36 bytes)
        let positions: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let normals: [f32; 9] = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let indices: [u16; 3] = [0, 1, 2];

        let pos_bytes = 36;
        let norm_bytes = 36;
        let idx_bytes = 6;
        // Pad index buffer to 4-byte alignment
        let idx_bytes_padded = 8;
        let buf_len = pos_bytes + norm_bytes + idx_bytes_padded;

        let mut bin_buffer = Vec::with_capacity(buf_len);
        for &v in &positions {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &normals {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &indices {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        // Pad to 4-byte alignment
        while bin_buffer.len() % 4 != 0 {
            bin_buffer.push(0);
        }

        let json = format!(
            r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0}}],"meshes":[{{"primitives":[{{"attributes":{{"POSITION":0,"NORMAL":1}},"indices":2}}]}}],"accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":2,"componentType":5123,"count":3,"type":"SCALAR"}}],"bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":{pos_bytes}}},{{"buffer":0,"byteOffset":{norm_off},"byteLength":{norm_bytes}}},{{"buffer":0,"byteOffset":{idx_off},"byteLength":{idx_bytes}}}],"buffers":[{{"byteLength":{buf_len}}}]}}"#,
            pos_bytes = pos_bytes,
            norm_off = pos_bytes,
            norm_bytes = norm_bytes,
            idx_off = pos_bytes + norm_bytes,
            idx_bytes = idx_bytes,
            buf_len = bin_buffer.len(),
        );

        let mut json_bytes = json.into_bytes();
        while json_bytes.len() % 4 != 0 {
            json_bytes.push(b' ');
        }

        let total_len = 12 + 8 + json_bytes.len() + 8 + bin_buffer.len();
        let mut glb = Vec::with_capacity(total_len);

        // Header
        glb.extend_from_slice(&0x4654_6C67_u32.to_le_bytes());
        glb.extend_from_slice(&2_u32.to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(total_len as u32).to_le_bytes());

        // JSON chunk
        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x4E4F_534A_u32.to_le_bytes());
        glb.extend_from_slice(&json_bytes);

        // BIN chunk
        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(bin_buffer.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x004E_4942_u32.to_le_bytes());
        glb.extend_from_slice(&bin_buffer);

        let mesh = read_glb(&glb).unwrap();
        assert_eq!(mesh.positions.len(), 3);
        assert_eq!(mesh.normals.len(), 3);
        assert_eq!(mesh.indices.len(), 3);
        assert_eq!(mesh.indices, vec![0, 1, 2]);
    }

    #[test]
    fn read_multi_primitive_glb() {
        // Two primitives with different accessor indices.
        // Primitive 0: positions=accessor 0, normals=accessor 1, indices=accessor 2
        // Primitive 1: positions=accessor 3, normals=accessor 4, indices=accessor 5

        // 3 vertices each: 2 triangles total
        let positions_0: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let normals_0: [f32; 9] = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let indices_0: [u32; 3] = [0, 1, 2];

        let positions_1: [f32; 9] = [2.0, 0.0, 0.0, 3.0, 0.0, 0.0, 2.0, 1.0, 0.0];
        let normals_1: [f32; 9] = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let indices_1: [u32; 3] = [0, 1, 2];

        let bv0_len = 36_usize; // positions_0
        let bv1_len = 36_usize; // normals_0
        let bv2_len = 12_usize; // indices_0
        let bv3_len = 36_usize; // positions_1
        let bv4_len = 36_usize; // normals_1
        let bv5_len = 12_usize; // indices_1

        let bv0_off = 0_usize;
        let bv1_off = bv0_off + bv0_len;
        let bv2_off = bv1_off + bv1_len;
        let bv3_off = bv2_off + bv2_len;
        let bv4_off = bv3_off + bv3_len;
        let bv5_off = bv4_off + bv4_len;
        let buf_len = bv5_off + bv5_len;

        let mut bin_buffer = Vec::with_capacity(buf_len);
        for &v in &positions_0 {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &normals_0 {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &indices_0 {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &positions_1 {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &normals_1 {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &indices_1 {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        while bin_buffer.len() % 4 != 0 {
            bin_buffer.push(0);
        }

        let json = format!(
            r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0,1]}}],"nodes":[{{"mesh":0}},{{"mesh":1}}],"meshes":[{{"primitives":[{{"attributes":{{"POSITION":0,"NORMAL":1}},"indices":2}}]}},{{"primitives":[{{"attributes":{{"POSITION":3,"NORMAL":4}},"indices":5}}]}}],"accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":2,"componentType":5125,"count":3,"type":"SCALAR"}},{{"bufferView":3,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":4,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":5,"componentType":5125,"count":3,"type":"SCALAR"}}],"bufferViews":[{{"buffer":0,"byteOffset":{bv0_off},"byteLength":{bv0_len}}},{{"buffer":0,"byteOffset":{bv1_off},"byteLength":{bv1_len}}},{{"buffer":0,"byteOffset":{bv2_off},"byteLength":{bv2_len}}},{{"buffer":0,"byteOffset":{bv3_off},"byteLength":{bv3_len}}},{{"buffer":0,"byteOffset":{bv4_off},"byteLength":{bv4_len}}},{{"buffer":0,"byteOffset":{bv5_off},"byteLength":{bv5_len}}}],"buffers":[{{"byteLength":{buf_len}}}]}}"#,
        );

        let mut json_bytes = json.into_bytes();
        while json_bytes.len() % 4 != 0 {
            json_bytes.push(b' ');
        }

        let total_len = 12 + 8 + json_bytes.len() + 8 + bin_buffer.len();
        let mut glb = Vec::with_capacity(total_len);

        glb.extend_from_slice(&0x4654_6C67_u32.to_le_bytes());
        glb.extend_from_slice(&2_u32.to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(total_len as u32).to_le_bytes());

        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x4E4F_534A_u32.to_le_bytes());
        glb.extend_from_slice(&json_bytes);

        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(bin_buffer.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x004E_4942_u32.to_le_bytes());
        glb.extend_from_slice(&bin_buffer);

        let mesh = read_glb(&glb).unwrap();

        // Combined: 6 positions, 6 normals, 6 indices
        assert_eq!(mesh.positions.len(), 6, "should have 6 vertices total");
        assert_eq!(mesh.normals.len(), 6, "should have 6 normals total");
        assert_eq!(mesh.indices.len(), 6, "should have 6 indices total");

        // Second primitive indices should be offset by 3
        assert_eq!(mesh.indices[0], 0);
        assert_eq!(mesh.indices[1], 1);
        assert_eq!(mesh.indices[2], 2);
        assert_eq!(mesh.indices[3], 3); // 0 + vertex_offset(3)
        assert_eq!(mesh.indices[4], 4); // 1 + vertex_offset(3)
        assert_eq!(mesh.indices[5], 5); // 2 + vertex_offset(3)

        assert!((mesh.positions[3].x() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn roundtrip_multi_solid_glb() {
        let mut topo = remus_topology::Topology::new();
        let box1 = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let box2 = remus_operations::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let glb = crate::gltf::write_glb(&topo, &[box1, box2], 0.1).unwrap();
        let mesh = read_glb(&glb).unwrap();

        assert!(!mesh.positions.is_empty(), "should have vertices");
        assert!(!mesh.indices.is_empty(), "should have indices");
        assert_eq!(mesh.indices.len() % 3, 0, "should be triangles");
    }

    #[test]
    fn read_glb_solid_returns_solid_id() {
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let glb = crate::gltf::write_glb(&topo, &[solid], 0.1).unwrap();

        let mut import_topo = remus_topology::Topology::new();
        let result = read_glb_solid(&mut import_topo, &glb, 1e-6);
        assert!(
            result.is_ok(),
            "read_glb_solid should return Ok: {result:?}"
        );
    }

    fn build_glb_bytes(json: &str, bin: &[u8]) -> Vec<u8> {
        let mut json_bytes = json.as_bytes().to_vec();
        while !json_bytes.len().is_multiple_of(4) {
            json_bytes.push(b' ');
        }
        let total_len = 12 + 8 + json_bytes.len() + 8 + bin.len();
        let mut glb = Vec::with_capacity(total_len);

        glb.extend_from_slice(&0x4654_6C67_u32.to_le_bytes());
        glb.extend_from_slice(&2_u32.to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(total_len as u32).to_le_bytes());

        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x4E4F_534A_u32.to_le_bytes());
        glb.extend_from_slice(&json_bytes);

        #[allow(clippy::cast_possible_truncation)]
        glb.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x004E_4942_u32.to_le_bytes());
        glb.extend_from_slice(bin);
        glb
    }

    #[test]
    fn unterminated_json_array_is_error_not_panic() {
        // Truncated JSON: the accessors array never closes. Previously this
        // produced an inverted slice range (&json[n+1..n]) and panicked.
        let json = r#"{"asset":{"version":"2.0"},"accessors":["#;
        let bin = [0u8; 4];
        let glb = build_glb_bytes(json, &bin);
        assert!(read_glb(&glb).is_err(), "must reject truncated JSON");
    }

    #[test]
    fn metadata_rejects_truncated_json_and_accepts_unicode_strings() {
        assert!(serde_json::from_str::<GltfDocument>(r#"{"accessors":["#).is_err());
        let json = r#"{"name":"é 日本語 ü 😀 ] } \"", "meshes":[]}"#;
        assert!(serde_json::from_str::<GltfDocument>(json).is_ok());
    }

    #[test]
    fn multibyte_json_text_before_arrays_is_error_not_panic() {
        // Fuzz finding (glb_reader, crash on 2026-08-30): non-ASCII text ahead
        // of an array used to panic in the array extractor; the reader must
        // instead reject the (incomplete) mesh input with a typed error.
        let json = r#"{"asset":{"version":"2.0","generator":"Übung ✓"},"accessors":[{"name":"é"}],"meshes":[{"name":"日本","primitives":[]}]}"#;
        let bin = [0u8; 4];
        let glb = build_glb_bytes(json, &bin);
        let result = read_glb(&glb);
        assert!(result.is_err(), "must reject without panicking: {result:?}");
    }

    #[test]
    fn oversized_chunk_length_is_rejected_without_overflow() {
        // A chunk length near u32::MAX used to wrap `offset + chunk_len` on
        // 32-bit targets; it must simply be skipped / rejected.
        let mut glb = Vec::new();
        glb.extend_from_slice(&0x4654_6C67_u32.to_le_bytes());
        glb.extend_from_slice(&2_u32.to_le_bytes());
        glb.extend_from_slice(&28_u32.to_le_bytes());
        glb.extend_from_slice(&0xFFFF_FFF0_u32.to_le_bytes()); // bogus len
        glb.extend_from_slice(&0x4E4F_534A_u32.to_le_bytes());
        assert!(read_glb(&glb).is_err(), "missing JSON chunk must error");
    }

    #[test]
    fn uint32_index_plus_vertex_offset_overflow_errors() {
        // Primitive 1's index of u32::MAX plus its vertex base (3) would
        // previously wrap to a silently corrupted small index.
        let positions_0: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let normals_0: [f32; 9] = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let indices_0: [u32; 3] = [0, 1, 2];
        let positions_1: [f32; 9] = [5.0, 0.0, 0.0, 6.0, 0.0, 0.0, 5.0, 1.0, 0.0];
        let normals_1: [f32; 9] = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let indices_1: [u32; 3] = [u32::MAX, 1, 2];

        let bv0 = (0_usize, 36_usize);
        let bv1 = (bv0.0 + bv0.1, 36_usize);
        let bv2 = (bv1.0 + bv1.1, 12_usize);
        let bv3 = (bv2.0 + bv2.1, 36_usize);
        let bv4 = (bv3.0 + bv3.1, 36_usize);
        let bv5 = (bv4.0 + bv4.1, 12_usize);
        let buf_len = bv5.0 + bv5.1;

        let mut bin_buffer = Vec::with_capacity(buf_len);
        for v in positions_0.iter().chain(normals_0.iter()) {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for i in &indices_0 {
            bin_buffer.extend_from_slice(&i.to_le_bytes());
        }
        for v in positions_1.iter().chain(normals_1.iter()) {
            bin_buffer.extend_from_slice(&v.to_le_bytes());
        }
        for i in &indices_1 {
            bin_buffer.extend_from_slice(&i.to_le_bytes());
        }

        let json = serde_json::json!({
            "meshes": [{
                "primitives": [
                    {"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2},
                    {"attributes": {"POSITION": 3, "NORMAL": 4}, "indices": 5}
                ]
            }],
            "accessors": [
                {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
                {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC3"},
                {"bufferView": 2, "componentType": 5125, "count": 3, "type": "SCALAR"},
                {"bufferView": 3, "componentType": 5126, "count": 3, "type": "VEC3"},
                {"bufferView": 4, "componentType": 5126, "count": 3, "type": "VEC3"},
                {"bufferView": 5, "componentType": 5125, "count": 3, "type": "SCALAR"}
            ],
            "bufferViews": [
                {"buffer": 0, "byteOffset": bv0.0, "byteLength": bv0.1},
                {"buffer": 0, "byteOffset": bv1.0, "byteLength": bv1.1},
                {"buffer": 0, "byteOffset": bv2.0, "byteLength": bv2.1},
                {"buffer": 0, "byteOffset": bv3.0, "byteLength": bv3.1},
                {"buffer": 0, "byteOffset": bv4.0, "byteLength": bv4.1},
                {"buffer": 0, "byteOffset": bv5.0, "byteLength": bv5.1}
            ],
            "buffers": [{"byteLength": buf_len}]
        })
        .to_string();

        let glb = build_glb_bytes(&json, &bin_buffer);
        let err = read_glb(&glb)
            .map(|_| ())
            .expect_err("u32 index overflow must be an error");
        assert!(
            format!("{err}").contains("overflow"),
            "unexpected error: {err}"
        );
    }

    /// Decode only the declared count from a shared view, maintaining the
    /// entity budget and refusing any range extending beyond the view.
    #[test]
    fn accessor_count_bounds_decoding_and_refuses_overruns() {
        // 3 declared vertices but a 4-vertex (48-byte) position view.
        let positions: [f32; 12] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let indices: [u16; 3] = [0, 1, 2];
        let mut bin = Vec::new();
        for &v in &positions {
            bin.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &indices {
            bin.extend_from_slice(&v.to_le_bytes());
        }
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        // count:3 over a 48-byte view: the fourth vertex is unused view data.
        let json = r#"{"asset":{"version":"2.0"},"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"},{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":48},{"buffer":0,"byteOffset":48,"byteLength":6}],"buffers":[{"byteLength":54}]}"#;
        let glb = build_glb_bytes(json, &bin);
        let mesh = read_glb(&glb).expect("a view may contain trailing unused data");
        assert_eq!(mesh.positions.len(), 3);
        assert_eq!(mesh.indices, [0, 1, 2]);
        let limits = ImportLimits {
            max_model_entities: 5,
            ..ImportLimits::default()
        };
        assert!(matches!(
            read_glb_with_limits(&glb, limits),
            Err(crate::IoError::LimitExceeded { .. })
        ));

        // count:4 over a 36-byte view (3 vertices): over-declared.
        let json = r#"{"asset":{"version":"2.0"},"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],"accessors":[{"bufferView":0,"componentType":5126,"count":4,"type":"VEC3"},{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":6}],"buffers":[{"byteLength":42}]}"#;
        let mut bin = Vec::new();
        for &v in &positions[..9] {
            bin.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &indices {
            bin.extend_from_slice(&v.to_le_bytes());
        }
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let glb = build_glb_bytes(json, &bin);
        assert!(
            read_glb(&glb).is_err(),
            "over-declared count must error, not decode past the view"
        );
    }

    /// A `componentType` outside the u32 range used to truncate via `as u32`
    /// (5123+2³² decoded as uint16) and silently reinterpret the index
    /// width (E-11). The accessor is dropped now, so the primitive's
    /// indices accessor is missing and the read fails closed.
    #[test]
    fn out_of_range_component_type_is_rejected_not_truncated() {
        let positions: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let indices: [u16; 3] = [0, 1, 2];
        let mut bin = Vec::new();
        for &v in &positions {
            bin.extend_from_slice(&v.to_le_bytes());
        }
        for &v in &indices {
            bin.extend_from_slice(&v.to_le_bytes());
        }
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        // 5123 + 2^32 = 4294972419: truncates to 5123 under `as u32`.
        let json = r#"{"asset":{"version":"2.0"},"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"},{"bufferView":1,"componentType":4294972419,"count":3,"type":"SCALAR"}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":6}],"buffers":[{"byteLength":42}]}"#;
        let glb = build_glb_bytes(json, &bin);
        // Reject the invalid component type without truncation or removing
        // the accessor slot and accidentally retargeting later references.
        let err = read_glb(&glb).map(|_| ()).expect_err(
            "out-of-range componentType must fail closed, not decode as truncated width",
        );
        assert!(
            format!("{err}").contains("component type"),
            "unexpected error: {err}"
        );
    }

    fn shared_accessor_glb(
        mesh_counts: &[usize],
        normal_count: usize,
        component_type: u32,
    ) -> Vec<u8> {
        let mut bin = Vec::new();
        for value in [0.0_f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
            bin.extend_from_slice(&value.to_le_bytes());
        }
        let index_length = if component_type == 5123 { 6 } else { 12 };
        for value in [0_u32, 1, 2] {
            if component_type == 5123 {
                bin.extend_from_slice(&(value as u16).to_le_bytes());
            } else {
                bin.extend_from_slice(&value.to_le_bytes());
            }
        }
        let normal_offset = bin.len();
        if normal_count > 0 {
            for value in [0.0_f32, 0.0, 1.0].repeat(normal_count) {
                bin.extend_from_slice(&value.to_le_bytes());
            }
        }
        let mut accessors = vec![
            serde_json::json!({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"}),
            serde_json::json!({"bufferView":1,"componentType":component_type,"count":3,"type":"SCALAR"}),
        ];
        let mut views = vec![
            serde_json::json!({"buffer":0,"byteOffset":0,"byteLength":36}),
            serde_json::json!({"buffer":0,"byteOffset":36,"byteLength":index_length}),
        ];
        let mut primitive = serde_json::json!({"attributes":{"POSITION":0},"indices":1});
        if normal_count > 0 {
            accessors.push(
                serde_json::json!({"bufferView":2,"componentType":5126,"count":normal_count,"type":"VEC3"}),
            );
            views.push(serde_json::json!({"buffer":0,"byteOffset":normal_offset,"byteLength":normal_count * 12}));
            primitive["attributes"]["NORMAL"] = serde_json::json!(2);
        }
        let meshes: Vec<_> = mesh_counts
            .iter()
            .map(|&count| serde_json::json!({"primitives":vec![primitive.clone();count]}))
            .collect();
        let json = serde_json::json!({"asset":{"version":"2.0"},"meshes":meshes,
            "accessors":accessors,"bufferViews":views,"buffers":[{"byteLength":bin.len()}]});
        build_glb_bytes(&json.to_string(), &bin)
    }

    #[test]
    fn shared_accessors_are_budgeted_per_reference_before_decoding() {
        for component_type in [5123, 5125] {
            for mesh_counts in [&[3][..], &[1, 2][..]] {
                let glb = shared_accessor_glb(mesh_counts, 0, component_type);
                for budget in [6, 17] {
                    let limits = ImportLimits {
                        max_model_entities: budget,
                        ..Default::default()
                    };
                    assert!(matches!(
                        read_glb_with_limits(&glb, limits),
                        Err(crate::IoError::LimitExceeded {
                            resource: "GLB decoded accessor entities",
                            ..
                        })
                    ));
                }
                let limits = ImportLimits {
                    max_model_entities: 18,
                    ..Default::default()
                };
                let mesh = read_glb_with_limits(&glb, limits).unwrap();
                assert_eq!(mesh.positions.len(), 9);
                assert_eq!(mesh.normals.len(), 9);
                assert_eq!(mesh.indices, (0..9).collect::<Vec<u32>>());
            }
        }
    }

    #[test]
    fn repeated_normals_are_charged_before_count_validation() {
        for (normal_count, rejected_budget, exact_budget) in [(3, 24, 27), (6, 35, 36)] {
            let glb = shared_accessor_glb(&[3], normal_count, 5123);
            let limits = ImportLimits {
                max_model_entities: rejected_budget,
                ..Default::default()
            };
            assert!(matches!(
                read_glb_with_limits(&glb, limits),
                Err(crate::IoError::LimitExceeded {
                    resource: "GLB decoded accessor entities",
                    ..
                })
            ));
            let limits = ImportLimits {
                max_model_entities: exact_budget,
                ..Default::default()
            };
            let result = read_glb_with_limits(&glb, limits);
            if normal_count != 3 {
                let error = result.unwrap_err();
                assert!(error.to_string().contains("NORMAL count"), "{error}");
                continue;
            }
            let mesh = result.unwrap();
            assert_eq!(mesh.normals.len(), 9);
            assert!(
                mesh.normals
                    .iter()
                    .all(|normal| *normal == Vec3::new(0.0, 0.0, 1.0))
            );
        }
    }

    #[test]
    fn only_triangle_primitive_modes_are_accepted() {
        let original = shared_accessor_glb(&[1], 0, 5123);
        // Reuse this fixture's chunks, changing only the mode declaration.
        let json_len = u32::from_le_bytes(original[12..16].try_into().unwrap()) as usize;
        let json_end = 20 + json_len;
        let mut json: serde_json::Value = serde_json::from_slice(&original[20..json_end]).unwrap();
        let bin = &original[json_end + 8..];
        for mode in [
            serde_json::json!(0),
            serde_json::json!(1),
            serde_json::json!(2),
            serde_json::json!(3),
            serde_json::json!(5),
            serde_json::json!(6),
            serde_json::json!(7),
            serde_json::json!(-1),
            serde_json::json!(4.5),
            serde_json::json!("4"),
            serde_json::Value::Null,
        ] {
            json["meshes"][0]["primitives"][0]["mode"] = mode;
            let error = read_glb(&build_glb_bytes(&json.to_string(), bin)).unwrap_err();
            assert!(error.to_string().contains("primitive mode"), "{error}");
        }
        json["meshes"][0]["primitives"][0]["mode"] = serde_json::json!(4);
        assert_eq!(
            read_glb(&build_glb_bytes(&json.to_string(), bin))
                .unwrap()
                .indices,
            vec![0, 1, 2]
        );
        assert_eq!(read_glb(&original).unwrap().indices, vec![0, 1, 2]);
    }

    fn fixture_parts(glb: &[u8]) -> (serde_json::Value, Vec<u8>) {
        let json_len = u32::from_le_bytes(glb[12..16].try_into().unwrap()) as usize;
        let json_end = 20 + json_len;
        (
            serde_json::from_slice(&glb[20..json_end]).unwrap(),
            glb[json_end + 8..].to_vec(),
        )
    }

    #[test]
    fn unindexed_triangles_generate_offset_indices_and_charge_the_budget() {
        let original = shared_accessor_glb(&[2], 0, 5125);
        let (mut json, bin) = fixture_parts(&original);
        for primitive in json["meshes"][0]["primitives"].as_array_mut().unwrap() {
            primitive.as_object_mut().unwrap().remove("indices");
        }
        let glb = build_glb_bytes(&json.to_string(), &bin);
        let mesh = read_glb(&glb).unwrap();
        assert_eq!(mesh.indices, vec![0, 1, 2, 3, 4, 5]);
        assert_eq!(mesh.positions.len(), 6);
        for budget in [6, 11] {
            let limits = ImportLimits {
                max_model_entities: budget,
                ..Default::default()
            };
            assert!(matches!(
                read_glb_with_limits(&glb, limits),
                Err(crate::IoError::LimitExceeded {
                    resource: "GLB decoded accessor entities",
                    ..
                })
            ));
        }
        let limits = ImportLimits {
            max_model_entities: 12,
            ..Default::default()
        };
        assert_eq!(
            read_glb_with_limits(&glb, limits).unwrap().indices,
            vec![0, 1, 2, 3, 4, 5]
        );
    }

    #[test]
    fn missing_normals_are_padded_before_the_following_primitive() {
        let original = shared_accessor_glb(&[2], 3, 5125);
        let (mut json, mut bin) = fixture_parts(&original);
        json["meshes"][0]["primitives"][0]["attributes"]
            .as_object_mut()
            .unwrap()
            .remove("NORMAL");
        let normal_offset = json["bufferViews"][2]["byteOffset"].as_u64().unwrap() as usize;
        for chunk in bin[normal_offset..normal_offset + 36].chunks_exact_mut(12) {
            chunk[..4].copy_from_slice(&1.0f32.to_le_bytes());
            chunk[4..].fill(0);
        }
        let position_offset = bin.len();
        for value in [0.0f32, 0., 0., 0., 1., 0., 0., 0., 1.] {
            bin.extend_from_slice(&value.to_le_bytes());
        }
        json["bufferViews"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::json!(
            {"buffer":0,"byteOffset":position_offset,"byteLength":36}));
        json["accessors"].as_array_mut().unwrap().push(serde_json::json!(
            {"bufferView":3,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[0,1,1]}));
        json["meshes"][0]["primitives"][1]["attributes"]["POSITION"] = serde_json::json!(3);
        json["buffers"][0]["byteLength"] = serde_json::json!(bin.len());
        let mesh = read_glb(&build_glb_bytes(&json.to_string(), &bin)).unwrap();
        assert_eq!(&mesh.normals[..3], &[Vec3::new(0., 0., 0.); 3]);
        assert_eq!(&mesh.normals[3..], &[Vec3::new(1., 0., 0.); 3]);
        assert_eq!(mesh.indices, vec![0, 1, 2, 3, 4, 5]);
    }

    #[test]
    fn invalid_normal_references_and_counts_refuse() {
        let original = shared_accessor_glb(&[1], 3, 5125);
        let (json, bin) = fixture_parts(&original);
        let mut bad = json.clone();
        bad["meshes"][0]["primitives"][0]["attributes"]["NORMAL"] = serde_json::json!(99);
        assert!(
            read_glb(&build_glb_bytes(&bad.to_string(), &bin))
                .unwrap_err()
                .to_string()
                .contains("NORMAL accessor")
        );
        let mut bad = json.clone();
        bad["accessors"][2]["bufferView"] = serde_json::json!(99);
        assert!(
            read_glb(&build_glb_bytes(&bad.to_string(), &bin))
                .unwrap_err()
                .to_string()
                .contains("NORMAL buffer view")
        );
        for count in [2, 4] {
            let mut bad = json.clone();
            bad["accessors"][2]["count"] = serde_json::json!(count);
            assert!(
                read_glb(&build_glb_bytes(&bad.to_string(), &bin))
                    .unwrap_err()
                    .to_string()
                    .contains("NORMAL count")
            );
        }
    }

    #[test]
    fn incomplete_unindexed_triangle_refuses() {
        let json = serde_json::json!({"asset":{"version":"2.0"},
            "meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],
            "buffers":[{"byteLength":24}],"bufferViews":[{"buffer":0,"byteLength":24}],
            "accessors":[{"bufferView":0,"componentType":5126,"count":2,"type":"VEC3"}]});
        let error = read_glb(&build_glb_bytes(&json.to_string(), &[0; 24])).unwrap_err();
        assert!(error.to_string().contains("multiple of three"), "{error}");
    }

    #[test]
    fn malformed_explicit_indices_refuse_before_implicit_index_generation() {
        let original = shared_accessor_glb(&[1], 0, 5125);
        let (json, bin) = fixture_parts(&original);
        for value in [
            serde_json::json!(-1),
            serde_json::Value::Null,
            serde_json::json!("1"),
            serde_json::json!(1.5),
            serde_json::json!(true),
        ] {
            let mut bad = json.clone();
            bad["meshes"][0]["primitives"][0]["indices"] = value;
            let error = read_glb(&build_glb_bytes(&bad.to_string(), &bin)).unwrap_err();
            assert!(
                matches!(error, crate::IoError::ParseError { .. }),
                "{error}"
            );
            assert!(error.to_string().contains("indices accessor"), "{error}");
        }
        let mut unindexed = json;
        unindexed["meshes"][0]["primitives"][0]
            .as_object_mut()
            .unwrap()
            .remove("indices");
        assert_eq!(
            read_glb(&build_glb_bytes(&unindexed.to_string(), &bin))
                .unwrap()
                .indices,
            [0, 1, 2]
        );
    }
}
