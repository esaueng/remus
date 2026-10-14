//! PLY file reader (ASCII and binary little-endian).
//!
//! The header is parsed into typed elements and properties, and the body is
//! read by that declaration: every PLY scalar type, properties in any order,
//! extra properties and elements skipped by their declared sizes, and list
//! properties read with their declared count and item types.

use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Read a PLY file from bytes and return a triangle mesh.
///
/// Supports ASCII and binary little-endian formats. Reads vertex positions
/// (`x`, `y`, `z`), optional normals (`nx`, `ny`, `nz`), and polygon faces
/// (`vertex_indices` or `vertex_index`, fan-triangulated). Other properties
/// and elements are skipped by their declared types.
///
/// # Errors
///
/// Returns an error if the file is malformed.
pub fn read_ply(data: &[u8]) -> Result<TriangleMesh, crate::IoError> {
    read_ply_with_limits(data, ImportLimits::default())
}

/// Read a PLY file with explicit hostile-input resource limits.
///
/// Every declared element count, and every list length as it is read, is
/// bounded by `max_model_entities`.
///
/// # Errors
///
/// Returns [`crate::IoError`] when a limit is exceeded or the PLY is malformed.
pub fn read_ply_with_limits(
    data: &[u8],
    limits: ImportLimits,
) -> Result<TriangleMesh, crate::IoError> {
    ensure_input_size(data.len(), limits)?;
    let header_end = find_header_end(data)?;
    let header_text =
        std::str::from_utf8(&data[..header_end]).map_err(|_| crate::IoError::ParseError {
            reason: "PLY header is not valid UTF-8".into(),
        })?;

    let header = parse_header(header_text, limits)?;
    let layout = MeshLayout::new(&header)?;
    let body = &data[header_end..];

    match header.format {
        PlyFormat::Ascii => {
            let text = std::str::from_utf8(body).map_err(|_| crate::IoError::ParseError {
                reason: "PLY body is not valid UTF-8".into(),
            })?;
            read_body(&header, &layout, &mut AsciiValues::new(text), limits)
        }
        PlyFormat::BinaryLittleEndian => read_body(
            &header,
            &layout,
            &mut BinaryValues { body, offset: 0 },
            limits,
        ),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlyFormat {
    Ascii,
    BinaryLittleEndian,
}

/// A PLY scalar type. Every one converts to `f64` exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ScalarType {
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Float32,
    Float64,
}

impl ScalarType {
    fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "char" | "int8" => Self::Int8,
            "uchar" | "uint8" => Self::Uint8,
            "short" | "int16" => Self::Int16,
            "ushort" | "uint16" => Self::Uint16,
            "int" | "int32" => Self::Int32,
            "uint" | "uint32" => Self::Uint32,
            "float" | "float32" => Self::Float32,
            "double" | "float64" => Self::Float64,
            _ => return None,
        })
    }

    const fn size(self) -> usize {
        match self {
            Self::Int8 | Self::Uint8 => 1,
            Self::Int16 | Self::Uint16 => 2,
            Self::Int32 | Self::Uint32 | Self::Float32 => 4,
            Self::Float64 => 8,
        }
    }

    /// Inclusive value range of an integer type; `None` for floats.
    const fn integer_range(self) -> Option<(i64, i64)> {
        match self {
            Self::Int8 => Some((i8::MIN as i64, i8::MAX as i64)),
            Self::Uint8 => Some((0, u8::MAX as i64)),
            Self::Int16 => Some((i16::MIN as i64, i16::MAX as i64)),
            Self::Uint16 => Some((0, u16::MAX as i64)),
            Self::Int32 => Some((i32::MIN as i64, i32::MAX as i64)),
            Self::Uint32 => Some((0, u32::MAX as i64)),
            Self::Float32 | Self::Float64 => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PropertyKind {
    Scalar(ScalarType),
    List { count: ScalarType, item: ScalarType },
}

#[derive(Debug)]
struct Property {
    name: String,
    kind: PropertyKind,
}

#[derive(Debug)]
struct Element {
    name: String,
    count: usize,
    properties: Vec<Property>,
}

impl Element {
    /// Smallest encoded size of one binary instance (every list empty).
    fn min_binary_size(&self) -> usize {
        self.properties
            .iter()
            .map(|property| match property.kind {
                PropertyKind::Scalar(ty) | PropertyKind::List { count: ty, .. } => ty.size(),
            })
            .sum()
    }
}

#[derive(Debug)]
struct PlyHeader {
    format: PlyFormat,
    elements: Vec<Element>,
}

fn find_header_end(data: &[u8]) -> Result<usize, crate::IoError> {
    let marker = b"end_header\n";
    data.windows(marker.len())
        .position(|w| w == marker)
        .map(|pos| pos + marker.len())
        .ok_or_else(|| crate::IoError::ParseError {
            reason: "PLY header missing end_header".into(),
        })
}

fn header_error(reason: impl Into<String>) -> crate::IoError {
    crate::IoError::ParseError {
        reason: reason.into(),
    }
}

fn parse_header(text: &str, limits: ImportLimits) -> Result<PlyHeader, crate::IoError> {
    let mut lines = text.lines().map(str::trim);
    if lines.next() != Some("ply") {
        return Err(header_error("PLY header must start with 'ply'"));
    }

    let mut format = None;
    let mut elements: Vec<Element> = Vec::new();
    let mut skipped_instances = 0_usize;
    for line in lines {
        let mut tokens = line.split_whitespace();
        match tokens.next() {
            None | Some("comment" | "obj_info") => {}
            Some("end_header") => break,
            Some("format") => {
                format = Some(match tokens.next() {
                    Some("ascii") => PlyFormat::Ascii,
                    Some("binary_little_endian") => PlyFormat::BinaryLittleEndian,
                    Some(other) => {
                        return Err(header_error(format!("unsupported PLY format {other}")));
                    }
                    None => return Err(header_error("PLY format line names no format")),
                });
            }
            Some("element") => {
                let (Some(name), Some(count), None) = (tokens.next(), tokens.next(), tokens.next())
                else {
                    return Err(header_error(format!("invalid PLY element line: {line}")));
                };
                let count: usize = count
                    .parse()
                    .map_err(|_| header_error(format!("invalid count in: {line}")))?;
                match name {
                    "vertex" => ensure_limit("PLY vertices", count, limits.max_model_entities)?,
                    "face" => ensure_limit("PLY faces", count, limits.max_model_entities)?,
                    // Every skipped element is read instance by instance, so
                    // they share one budget: many small declarations must not
                    // add up past the limit.
                    _ => {
                        skipped_instances = skipped_instances.saturating_add(count);
                        ensure_limit(
                            "PLY element instances",
                            skipped_instances,
                            limits.max_model_entities,
                        )?;
                    }
                }
                if elements.iter().any(|element| element.name == name) {
                    return Err(header_error(format!("PLY element {name} declared twice")));
                }
                elements.push(Element {
                    name: name.to_string(),
                    count,
                    properties: Vec::new(),
                });
            }
            Some("property") => {
                let element = elements.last_mut().ok_or_else(|| {
                    header_error(format!("PLY property before any element: {line}"))
                })?;
                let property = parse_property(&mut tokens, line)?;
                if element.properties.iter().any(|p| p.name == property.name) {
                    return Err(header_error(format!(
                        "PLY element {} declares property {} twice",
                        element.name, property.name
                    )));
                }
                element.properties.push(property);
            }
            Some(other) => {
                return Err(header_error(format!(
                    "unsupported PLY header keyword {other}"
                )));
            }
        }
    }

    let format = format.ok_or_else(|| header_error("PLY format not specified"))?;
    if let Some(element) = elements
        .iter()
        .find(|element| element.count > 0 && element.properties.is_empty())
    {
        return Err(header_error(format!(
            "PLY element {} has instances but declares no properties",
            element.name
        )));
    }
    Ok(PlyHeader { format, elements })
}

fn parse_property(
    tokens: &mut std::str::SplitWhitespace<'_>,
    line: &str,
) -> Result<Property, crate::IoError> {
    let scalar = |token: Option<&str>| {
        token
            .and_then(ScalarType::parse)
            .ok_or_else(|| header_error(format!("invalid PLY property type in: {line}")))
    };
    let kind = match tokens.next() {
        Some("list") => {
            let count = scalar(tokens.next())?;
            let item = scalar(tokens.next())?;
            if count.integer_range().is_none() {
                return Err(header_error(format!(
                    "PLY list count type must be an integer: {line}"
                )));
            }
            PropertyKind::List { count, item }
        }
        token => PropertyKind::Scalar(scalar(token)?),
    };
    let (Some(name), None) = (tokens.next(), tokens.next()) else {
        return Err(header_error(format!("invalid PLY property line: {line}")));
    };
    Ok(Property {
        name: name.to_string(),
        kind,
    })
}

/// Where the mesh data sits in the declared elements.
#[derive(Debug)]
struct MeshLayout {
    /// Positions of `x`, `y`, `z` in the vertex element's properties.
    position: Option<[usize; 3]>,
    /// Positions of `nx`, `ny`, `nz`, when all three are declared.
    normal: Option<[usize; 3]>,
    /// Position of the face element's vertex index list.
    face_indices: Option<usize>,
}

impl MeshLayout {
    fn new(header: &PlyHeader) -> Result<Self, crate::IoError> {
        let mut layout = Self {
            position: None,
            normal: None,
            face_indices: None,
        };
        let scalar_named = |element: &Element, name: &str| {
            element
                .properties
                .iter()
                .position(|p| p.name == name)
                .map(|index| match element.properties[index].kind {
                    PropertyKind::Scalar(_) => Ok(index),
                    PropertyKind::List { .. } => Err(header_error(format!(
                        "PLY vertex property {name} must be a scalar"
                    ))),
                })
                .transpose()
        };
        for element in &header.elements {
            match element.name.as_str() {
                "vertex" => {
                    let [x, y, z] = ["x", "y", "z"].map(|name| scalar_named(element, name));
                    match (x?, y?, z?) {
                        (Some(x), Some(y), Some(z)) => layout.position = Some([x, y, z]),
                        _ if element.count == 0 => {}
                        _ => {
                            return Err(header_error(
                                "PLY vertex element must declare x, y and z properties",
                            ));
                        }
                    }
                    let [nx, ny, nz] = ["nx", "ny", "nz"].map(|name| scalar_named(element, name));
                    if let (Some(nx), Some(ny), Some(nz)) = (nx?, ny?, nz?) {
                        layout.normal = Some([nx, ny, nz]);
                    }
                }
                "face" => {
                    let index = element
                        .properties
                        .iter()
                        .position(|p| p.name == "vertex_indices" || p.name == "vertex_index");
                    match index.map(|index| (index, element.properties[index].kind)) {
                        Some((index, PropertyKind::List { item, .. }))
                            if item.integer_range().is_some() =>
                        {
                            layout.face_indices = Some(index);
                        }
                        Some(_) => {
                            return Err(header_error(
                                "PLY face vertex indices must be a list of integers",
                            ));
                        }
                        None if element.count == 0 => {}
                        None => {
                            return Err(header_error(
                                "PLY face element must declare a vertex_indices list",
                            ));
                        }
                    }
                }
                _ => {}
            }
        }
        Ok(layout)
    }
}

/// A source of PLY body values, read in declaration order.
trait Values {
    /// Start an element instance.
    fn begin_instance(&mut self) -> Result<(), String>;
    /// Finish an element instance.
    fn end_instance(&mut self) -> Result<(), String>;
    /// The next value, of declared type `ty`.
    fn scalar(&mut self, ty: ScalarType) -> Result<f64, String>;
    /// Skip `n` values of declared type `ty`.
    fn skip(&mut self, ty: ScalarType, n: usize) -> Result<(), String>;
    /// Instances of `element` worth reserving room for up front.
    fn capacity_hint(&self, element: &Element) -> usize;
}

/// ASCII bodies hold one element instance per line.
struct AsciiValues<'a> {
    lines: std::str::Lines<'a>,
    tokens: std::str::SplitWhitespace<'a>,
}

impl<'a> AsciiValues<'a> {
    fn new(text: &'a str) -> Self {
        Self {
            lines: text.lines(),
            tokens: "".split_whitespace(),
        }
    }
}

impl Values for AsciiValues<'_> {
    fn begin_instance(&mut self) -> Result<(), String> {
        loop {
            let line = self.lines.next().ok_or("unexpected end of data")?;
            if !line.trim().is_empty() {
                self.tokens = line.split_whitespace();
                return Ok(());
            }
        }
    }

    fn end_instance(&mut self) -> Result<(), String> {
        match self.tokens.next() {
            Some(extra) => Err(format!("more values than declared (at '{extra}')")),
            None => Ok(()),
        }
    }

    fn scalar(&mut self, ty: ScalarType) -> Result<f64, String> {
        let token = self.tokens.next().ok_or("fewer values than declared")?;
        match ty.integer_range() {
            Some((lo, hi)) => match token.parse::<i64>() {
                Ok(value) if (lo..=hi).contains(&value) => Ok(value as f64),
                _ => Err(format!("invalid {ty:?} value '{token}'")),
            },
            None => token
                .parse::<f64>()
                .map_err(|_| format!("invalid {ty:?} value '{token}'")),
        }
    }

    fn skip(&mut self, ty: ScalarType, n: usize) -> Result<(), String> {
        for _ in 0..n {
            self.scalar(ty)?;
        }
        Ok(())
    }

    fn capacity_hint(&self, _element: &Element) -> usize {
        // ASCII counts do not prove that any valid records exist. Allocate
        // only as records are parsed.
        0
    }
}

/// Binary little-endian bodies pack instances back to back.
struct BinaryValues<'a> {
    body: &'a [u8],
    offset: usize,
}

impl BinaryValues<'_> {
    fn take<const N: usize>(&mut self) -> Result<[u8; N], String> {
        let bytes = self
            .body
            .get(self.offset..)
            .and_then(|rest| rest.first_chunk::<N>())
            .ok_or("unexpected end of data")?;
        self.offset += N;
        Ok(*bytes)
    }
}

impl Values for BinaryValues<'_> {
    fn begin_instance(&mut self) -> Result<(), String> {
        Ok(())
    }

    fn end_instance(&mut self) -> Result<(), String> {
        Ok(())
    }

    fn scalar(&mut self, ty: ScalarType) -> Result<f64, String> {
        Ok(match ty {
            ScalarType::Int8 => f64::from(i8::from_le_bytes(self.take()?)),
            ScalarType::Uint8 => f64::from(u8::from_le_bytes(self.take()?)),
            ScalarType::Int16 => f64::from(i16::from_le_bytes(self.take()?)),
            ScalarType::Uint16 => f64::from(u16::from_le_bytes(self.take()?)),
            ScalarType::Int32 => f64::from(i32::from_le_bytes(self.take()?)),
            ScalarType::Uint32 => f64::from(u32::from_le_bytes(self.take()?)),
            ScalarType::Float32 => f64::from(f32::from_le_bytes(self.take()?)),
            ScalarType::Float64 => f64::from_le_bytes(self.take()?),
        })
    }

    fn skip(&mut self, ty: ScalarType, n: usize) -> Result<(), String> {
        let end = n
            .checked_mul(ty.size())
            .and_then(|bytes| self.offset.checked_add(bytes))
            .filter(|&end| end <= self.body.len())
            .ok_or("unexpected end of data")?;
        self.offset = end;
        Ok(())
    }

    fn capacity_hint(&self, element: &Element) -> usize {
        // Reserve no more instances than the remaining bytes can hold.
        let remaining = self.body.len().saturating_sub(self.offset);
        element
            .count
            .min(remaining / element.min_binary_size().max(1))
    }
}

/// Read a list length of declared type `ty`, bounded by `limit`.
fn list_len(
    values: &mut impl Values,
    ty: ScalarType,
    resource: &'static str,
    limit: usize,
) -> Result<Result<usize, crate::IoError>, String> {
    let count = values.scalar(ty)?;
    if count < 0.0 {
        return Err(format!("negative list length {count}"));
    }
    // integral, >= 0, <= u32::MAX
    let count = count as usize;
    Ok(ensure_limit(resource, count, limit).map(|()| count))
}

/// Read the body by the declared elements and assemble the mesh.
fn read_body(
    header: &PlyHeader,
    layout: &MeshLayout,
    values: &mut impl Values,
    limits: ImportLimits,
) -> Result<TriangleMesh, crate::IoError> {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mut face = Vec::new();
    let mut triangle_count = 0_usize;

    for element in &header.elements {
        let context = |instance: usize, reason: String| crate::IoError::ParseError {
            reason: format!(
                "PLY {} {instance} of {}: {reason}",
                element.name, element.count
            ),
        };
        let is_vertex = element.name == "vertex";
        let face_list = if element.name == "face" {
            layout.face_indices
        } else {
            None
        };
        if is_vertex {
            positions.reserve(values.capacity_hint(element));
            if layout.normal.is_some() {
                normals.reserve(positions.capacity());
            }
        }

        let mut scalars = vec![0.0; element.properties.len()];
        for instance in 0..element.count {
            values
                .begin_instance()
                .map_err(|reason| context(instance, reason))?;
            for (index, property) in element.properties.iter().enumerate() {
                match property.kind {
                    PropertyKind::Scalar(ty) => {
                        scalars[index] = values
                            .scalar(ty)
                            .map_err(|reason| context(instance, reason))?;
                    }
                    PropertyKind::List { count, item } if face_list == Some(index) => {
                        let n = list_len(
                            values,
                            count,
                            "PLY face vertices",
                            limits.max_model_entities,
                        )
                        .map_err(|reason| context(instance, reason))??;
                        if n == 0 {
                            return Err(context(
                                instance,
                                "face must contain at least one vertex".into(),
                            ));
                        }
                        triangle_count = triangle_count.saturating_add(n.saturating_sub(2));
                        ensure_limit("PLY triangles", triangle_count, limits.max_model_entities)?;
                        face.clear();
                        for _ in 0..n {
                            let value = values
                                .scalar(item)
                                .map_err(|reason| context(instance, reason))?;
                            let vertex = u32::try_from(value as i64).ok();
                            face.push(vertex.ok_or_else(|| {
                                context(instance, format!("invalid vertex index {value}"))
                            })?);
                        }
                        // Fan triangulation for polygons: v0-v1-v2, v0-v2-v3, ...
                        for pair in face.windows(2).skip(1) {
                            indices.extend([face[0], pair[0], pair[1]]);
                        }
                    }
                    PropertyKind::List { count, item } => {
                        let n =
                            list_len(values, count, "PLY list items", limits.max_model_entities)
                                .map_err(|reason| context(instance, reason))??;
                        values
                            .skip(item, n)
                            .map_err(|reason| context(instance, reason))?;
                    }
                }
            }
            values
                .end_instance()
                .map_err(|reason| context(instance, reason))?;

            if is_vertex {
                if let Some([x, y, z]) = layout.position {
                    positions.push(Point3::new(scalars[x], scalars[y], scalars[z]));
                }
                if let Some([nx, ny, nz]) = layout.normal {
                    normals.push(Vec3::new(scalars[nx], scalars[ny], scalars[nz]));
                }
            }
        }
    }

    if let Some(&index) = indices.iter().find(|&&i| i as usize >= positions.len()) {
        return Err(crate::IoError::ParseError {
            reason: format!(
                "PLY face references vertex {index}, but the file has {} vertices",
                positions.len()
            ),
        });
    }
    if normals.is_empty() {
        normals = compute_normals(&positions, &indices);
    }

    Ok(TriangleMesh {
        positions,
        normals,
        indices,
    })
}

fn compute_normals(positions: &[Point3], indices: &[u32]) -> Vec<Vec3> {
    let mut normals = vec![Vec3::new(0.0, 0.0, 0.0); positions.len()];

    for tri in indices.chunks_exact(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;
        if i0 >= positions.len() || i1 >= positions.len() || i2 >= positions.len() {
            continue;
        }
        let e1 = positions[i1] - positions[i0];
        let e2 = positions[i2] - positions[i0];
        let n = e1.cross(e2);
        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    }

    for n in &mut normals {
        let len = n.length();
        if len > 1e-15 {
            *n = Vec3::new(n.x() / len, n.y() / len, n.z() / len);
        } else {
            *n = Vec3::new(0.0, 0.0, 1.0);
        }
    }

    normals
}

/// Read a PLY file and import it as a solid with one planar face per triangle.
///
/// This is a convenience wrapper that calls [`read_ply`] followed by
/// [`import_mesh`](crate::stl::import::import_mesh). Vertices within
/// `tolerance` of each other are merged.
///
/// # Errors
///
/// Returns [`IoError`](crate::IoError) if the file is malformed or the mesh
/// cannot be converted to a valid solid.
pub fn read_ply_solid(
    topo: &mut Topology,
    data: &[u8],
    tolerance: f64,
) -> Result<SolidId, crate::IoError> {
    let mesh = read_ply(data)?;
    crate::stl::import::import_mesh(topo, &mesh, tolerance)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn declared_ascii_counts_cannot_allocate_without_valid_records() {
        // Using the native configurable limit makes the old eager reservation
        // fail with capacity overflow, without depending on RSS/overcommit.
        let limits = ImportLimits {
            max_model_entities: usize::MAX,
            ..ImportLimits::default()
        };
        for body in ["", "\n\n", "0 0 0\n", "\r\n"] {
            let ply = format!(
                "ply\nformat ascii 1.0\nelement vertex {}\nelement face 0\nend_header\n{body}",
                usize::MAX
            );
            assert!(matches!(
                read_ply_with_limits(ply.as_bytes(), limits),
                Err(crate::IoError::ParseError { .. })
            ));
        }
    }

    #[test]
    fn binary_vertex_bytes_overflow_is_an_error() {
        let ply = format!(
            "ply\nformat binary_little_endian 1.0\nelement vertex {}\nelement face 0\nend_header\n",
            usize::MAX
        );
        let limits = ImportLimits {
            max_model_entities: usize::MAX,
            ..ImportLimits::default()
        };
        assert!(matches!(
            read_ply_with_limits(ply.as_bytes(), limits),
            Err(crate::IoError::ParseError { .. })
        ));
    }

    #[test]
    fn read_ascii_triangle() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n";

        let mesh = read_ply(ply).unwrap();
        assert_eq!(mesh.positions.len(), 3);
        assert_eq!(mesh.indices.len(), 3);
    }

    #[test]
    fn rejects_ascii_face_with_zero_vertices() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n0\n";

        let err = read_ply(ply).unwrap_err();
        assert!(matches!(err, crate::IoError::ParseError { .. }));
    }

    #[test]
    fn rejects_declared_vertex_count_before_allocation() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nelement face 0\nend_header\n";
        let limits = ImportLimits {
            max_model_entities: 3,
            ..ImportLimits::default()
        };
        let err = read_ply_with_limits(ply, limits).unwrap_err();
        assert!(matches!(
            err,
            crate::IoError::LimitExceeded {
                resource: "PLY vertices",
                limit: 3,
                actual: 4
            }
        ));
    }

    #[test]
    fn rejects_oversized_ascii_face_before_collecting_indices() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uint int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n1000000 0 1 2\n";
        let limits = ImportLimits {
            max_model_entities: 3,
            ..ImportLimits::default()
        };
        let err = read_ply_with_limits(ply, limits).unwrap_err();
        assert!(matches!(
            err,
            crate::IoError::LimitExceeded {
                resource: "PLY face vertices",
                limit: 3,
                actual: 1_000_000
            }
        ));
    }

    #[test]
    fn rejects_total_triangulated_output_over_limit() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 3\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n4 0 1 2 0\n4 0 1 2 0\n4 0 1 2 0\n";
        let limits = ImportLimits {
            max_model_entities: 4,
            ..ImportLimits::default()
        };
        let err = read_ply_with_limits(ply, limits).unwrap_err();
        assert!(matches!(
            err,
            crate::IoError::LimitExceeded {
                resource: "PLY triangles",
                limit: 4,
                actual: 6
            }
        ));
    }

    #[test]
    fn roundtrip_ascii() {
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let ply_data =
            crate::ply::write_ply(&topo, &[solid], 0.1, crate::ply::writer::PlyFormat::Ascii)
                .unwrap();
        let mesh = read_ply(&ply_data).unwrap();

        assert!(!mesh.positions.is_empty());
        assert!(!mesh.indices.is_empty());
        assert_eq!(mesh.indices.len() % 3, 0);
    }

    #[test]
    fn roundtrip_binary() {
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let ply_data = crate::ply::write_ply(
            &topo,
            &[solid],
            0.1,
            crate::ply::writer::PlyFormat::BinaryLittleEndian,
        )
        .unwrap();
        let mesh = read_ply(&ply_data).unwrap();

        assert!(!mesh.positions.is_empty());
        assert!(!mesh.indices.is_empty());
    }

    #[test]
    fn missing_header_error() {
        let data = b"not a ply file";
        assert!(read_ply(data).is_err());
    }

    /// A truncated binary face section used to end the mesh silently with
    /// `Ok` and fewer faces than declared (E-6). The ASCII path errors on
    /// short faces; the binary path does too now.
    #[test]
    fn rejects_truncated_binary_face_section() {
        // 1 vertex, 100 declared faces, zero face bytes.
        let mut data = b"ply\nformat binary_little_endian 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nelement face 100\nproperty list uchar int vertex_indices\nend_header\n"
            .to_vec();
        data.extend_from_slice(&0.0f32.to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
        let err = read_ply(&data).unwrap_err();
        assert!(
            matches!(err, crate::IoError::ParseError { .. }),
            "truncated binary faces must error, got {err}"
        );
    }

    #[test]
    fn read_ply_solid_returns_solid_id() {
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

        let ply_data =
            crate::ply::write_ply(&topo, &[solid], 0.1, crate::ply::writer::PlyFormat::Ascii)
                .unwrap();

        let mut import_topo = remus_topology::Topology::new();
        let result = read_ply_solid(&mut import_topo, &ply_data, 1e-6);
        assert!(
            result.is_ok(),
            "read_ply_solid should return Ok: {result:?}"
        );
    }
}
