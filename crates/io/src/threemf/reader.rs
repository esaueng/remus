//! 3MF file reader.
//!
//! Parses [3D Manufacturing Format](https://3mf.io/specification/) (`.3mf`)
//! files into [`TriangleMesh`] objects. A `.3mf` file is a ZIP archive
//! containing an XML model description at `3D/3dmodel.model`.

use std::collections::HashMap;
use std::io::{Cursor, Read as _};

use quick_xml::name::{Namespace, NamespaceResolver, QName, ResolveResult};

use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::IoError;
use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};

/// Read a 3MF file from raw bytes and return one [`TriangleMesh`] per build
/// item.
///
/// The roots are the `<build>` `<item>`s, in build order and with
/// multiplicity: an object listed twice gives two meshes, and an object no
/// item names is not returned. Each root object is flattened into one mesh:
/// an object built from `<components>` places every referenced object by its
/// component transform, recursively, and the item's `transform` is then
/// applied to the result. Transforms use the 3MF row-vector layout
/// (`[x y z 1] · M`), their translation is scaled with the model unit like
/// the vertices, and a mirroring transform (det < 0) reverses the winding so
/// it stays outward. When `<build>` is missing or empty, every object that no
/// component references is a root instead, in document order, unplaced.
/// Coordinates are normalized from the model's declared unit to millimeters.
///
/// # Errors
///
/// Returns [`IoError`] if:
/// - The data is not a valid ZIP archive
/// - The archive lacks a `3D/3dmodel.model` entry
/// - The XML is malformed or missing required elements
/// - A component or build item references a missing or ambiguous object id
///   or an object in another model part (production extension, `p:path`),
///   components form a cycle or nest deeper than 32 levels, or a placement
///   leaves the finite millimeter range
/// - The build items or the flattened output (counted with multiplicity)
///   exceed the entity limit
pub fn read_threemf(data: &[u8]) -> Result<Vec<TriangleMesh>, IoError> {
    read_threemf_with_limits(data, ImportLimits::default())
}

/// Read a 3MF file with explicit compressed-input, uncompressed-entry, and
/// model-entity limits.
///
/// # Errors
///
/// Returns [`IoError`] when a limit is exceeded or the 3MF data is invalid.
pub fn read_threemf_with_limits(
    data: &[u8],
    limits: ImportLimits,
) -> Result<Vec<TriangleMesh>, IoError> {
    ensure_input_size(data.len(), limits)?;
    let model_xml = extract_model_xml(data, limits)?;
    parse_model_xml(&model_xml, limits)
}

/// Extract the `3D/3dmodel.model` XML from the ZIP archive.
fn extract_model_xml(data: &[u8], limits: ImportLimits) -> Result<String, IoError> {
    let cursor = Cursor::new(data);
    let mut archive = zip::ZipArchive::new(cursor).map_err(IoError::Zip)?;

    let model_file = archive
        .by_name("3D/3dmodel.model")
        .map_err(|_| IoError::ParseError {
            reason: "3MF archive missing '3D/3dmodel.model' entry".to_string(),
        })?;

    let declared_size = usize::try_from(model_file.size()).unwrap_or(usize::MAX);
    ensure_limit(
        "3MF model XML bytes",
        declared_size,
        limits.max_archive_entry_bytes,
    )?;

    // ZIP metadata is untrusted. Grow from bytes actually decoded instead of
    // reserving the full declared size before reading the stream.
    let mut xml_str = String::with_capacity(declared_size.min(64 * 1024));
    model_file
        .take(limits.max_archive_entry_bytes.saturating_add(1) as u64)
        .read_to_string(&mut xml_str)
        .map_err(IoError::Io)?;
    ensure_limit(
        "3MF model XML bytes",
        xml_str.len(),
        limits.max_archive_entry_bytes,
    )?;

    Ok(xml_str)
}

/// The 3MF core namespace. Elements are also accepted unbound, from models
/// that declare no namespace.
const CORE_NS: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
/// The production extension, whose `p:path` points into another model part.
const PRODUCTION_NS: &str = "http://schemas.microsoft.com/3dmanufacturing/production/2015/06";

/// Whether `name` is a core 3MF element rather than an extension element
/// that happens to share its local name (such as a vendor `<v:build>`).
fn is_core_element(resolver: &NamespaceResolver, name: QName<'_>) -> bool {
    match resolver.resolve_element(name).0 {
        ResolveResult::Unbound => true,
        ResolveResult::Bound(Namespace(ns)) => ns == CORE_NS,
        ResolveResult::Unknown(_) => false,
    }
}

/// Deepest component nesting a model may use. Producers nest a few levels;
/// the bound keeps component resolution shallow on hostile input.
const MAX_COMPONENT_DEPTH: usize = 32;

/// One `<object>`: its own mesh and the components it places.
#[derive(Debug, Default)]
struct ParsedObject {
    /// `None` when the object has no `id`, so no component can reference it.
    id: Option<u32>,
    vertices: Vec<Point3>,
    indices: Vec<u32>,
    components: Vec<Component>,
}

/// A `<component>`: another object placed by an affine transform.
#[derive(Debug, Clone, Copy)]
struct Component {
    object_id: u32,
    transform: Affine,
}

/// A `<build>` `<item>`: an output object and its optional placement.
#[derive(Debug, Clone, Copy)]
struct BuildItem {
    object_id: u32,
    transform: Option<Affine>,
}

/// A 3MF transform: points are row vectors, `[x y z 1] · M`, with `M` the
/// 4×3 matrix `m00 m01 m02 m10 … m32` in attribute order. `0..9` hold the
/// linear part row by row and `9..12` the translation, in millimeters.
#[derive(Debug, Clone, Copy)]
struct Affine([f64; 12]);

impl Affine {
    const IDENTITY: Self = Self([1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0.]);

    fn apply(&self, p: Point3) -> Point3 {
        let m = &self.0;
        let row =
            |col: usize| p.x() * m[col] + p.y() * m[3 + col] + p.z() * m[6 + col] + m[9 + col];
        Point3::new(row(0), row(1), row(2))
    }

    /// `self` applied first, then `outer`.
    fn then(&self, outer: &Self) -> Self {
        let (a, b) = (&self.0, &outer.0);
        let mut m = [0.0; 12];
        for row in 0..4 {
            for col in 0..3 {
                let translation = if row == 3 { b[9 + col] } else { 0.0 };
                m[row * 3 + col] = a[row * 3] * b[col]
                    + a[row * 3 + 1] * b[3 + col]
                    + a[row * 3 + 2] * b[6 + col]
                    + translation;
            }
        }
        Self(m)
    }

    /// A negative determinant mirrors, which turns the winding inside out.
    fn is_mirroring(&self) -> bool {
        let m = &self.0;
        let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6]);
        det < 0.0
    }
}

/// Parse the model XML and return one mesh per root (see [`read_threemf`]).
#[allow(clippy::too_many_lines)]
fn parse_model_xml(xml: &str, limits: ImportLimits) -> Result<Vec<TriangleMesh>, IoError> {
    let mut reader = quick_xml::NsReader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut objects = Vec::new();
    let mut items = Vec::new();
    let mut current: Option<ParsedObject> = None;
    // Only direct children of their core parent may change geometry state.
    // Unsupported extension subtrees are opaque, including any unprefixed
    // children that inherit the core namespace from the document root.
    let mut depth = 0usize;
    let mut ignored_depth: Option<usize> = None;
    let mut model_depth: Option<usize> = None;
    let mut resources_depth: Option<usize> = None;
    let mut object_depth: Option<usize> = None;
    let mut mesh_depth: Option<usize> = None;
    let mut vertices_depth: Option<usize> = None;
    let mut triangles_depth: Option<usize> = None;
    let mut components_depth: Option<usize> = None;
    let mut build_depth: Option<usize> = None;
    let mut total_vertices = 0usize;
    let mut total_triangles = 0usize;
    let mut total_objects = 0usize;
    let mut total_components = 0usize;
    let mut unit_scale = 1.0;

    loop {
        match reader.read_event() {
            Ok(quick_xml::events::Event::Start(ref e)) => {
                depth += 1;
                if ignored_depth.is_some() {
                    continue;
                }
                if !is_core_element(reader.resolver(), e.name()) {
                    ignored_depth = Some(depth);
                    continue;
                }
                let local = e.local_name();
                let name = local.as_ref();

                match name {
                    "model" if depth == 1 => {
                        unit_scale = model_unit_scale(e)?;
                        model_depth = Some(depth);
                    }
                    "resources" if model_depth == Some(depth - 1) => {
                        resources_depth = Some(depth);
                    }
                    "object" if resources_depth == Some(depth - 1) => {
                        total_objects = total_objects.saturating_add(1);
                        ensure_limit("3MF objects", total_objects, limits.max_model_entities)?;
                        current = Some(ParsedObject {
                            id: parse_object_id(e)?,
                            ..ParsedObject::default()
                        });
                        object_depth = Some(depth);
                    }
                    "mesh" if object_depth == Some(depth - 1) => mesh_depth = Some(depth),
                    "vertices" if mesh_depth == Some(depth - 1) => vertices_depth = Some(depth),
                    "triangles" if mesh_depth == Some(depth - 1) => triangles_depth = Some(depth),
                    "components" if object_depth == Some(depth - 1) => {
                        components_depth = Some(depth);
                    }
                    "component" if components_depth == Some(depth - 1) => {
                        total_components = total_components.saturating_add(1);
                        ensure_limit(
                            "3MF components",
                            total_components,
                            limits.max_model_entities,
                        )?;
                        push_component(e, reader.resolver(), unit_scale, current.as_mut())?;
                    }
                    // A direct child of the root `<model>`.
                    "build" if model_depth == Some(depth - 1) => {
                        build_depth = Some(depth);
                    }
                    "item" if build_depth == Some(depth - 1) => {
                        push_item(e, reader.resolver(), unit_scale, &mut items, limits)?;
                    }
                    _ => {}
                }
            }
            Ok(quick_xml::events::Event::Empty(ref e)) => {
                if ignored_depth.is_some() || !is_core_element(reader.resolver(), e.name()) {
                    continue;
                }
                let local = e.local_name();
                let name = local.as_ref();

                match name {
                    "vertex" if vertices_depth == Some(depth) => {
                        let point = parse_vertex_attributes(e)?;
                        let pt = Point3::new(
                            point.x() * unit_scale,
                            point.y() * unit_scale,
                            point.z() * unit_scale,
                        );
                        if pt.0.iter().any(|coordinate| !coordinate.is_finite()) {
                            return Err(IoError::ParseError {
                                reason: "3MF vertex exceeds the finite millimeter range".into(),
                            });
                        }
                        total_vertices = total_vertices.saturating_add(1);
                        ensure_limit("3MF vertices", total_vertices, limits.max_model_entities)?;
                        if let Some(object) = current.as_mut() {
                            object.vertices.push(pt);
                        }
                    }
                    "triangle" if triangles_depth == Some(depth) => {
                        let (v1, v2, v3) = parse_triangle_attributes(e)?;
                        total_triangles = total_triangles.saturating_add(1);
                        ensure_limit("3MF triangles", total_triangles, limits.max_model_entities)?;
                        if let Some(object) = current.as_mut() {
                            object.indices.extend([v1, v2, v3]);
                        }
                    }
                    "component" if components_depth == Some(depth) => {
                        total_components = total_components.saturating_add(1);
                        ensure_limit(
                            "3MF components",
                            total_components,
                            limits.max_model_entities,
                        )?;
                        push_component(e, reader.resolver(), unit_scale, current.as_mut())?;
                    }
                    "item" if build_depth == Some(depth) => {
                        push_item(e, reader.resolver(), unit_scale, &mut items, limits)?;
                    }
                    _ => {}
                }
            }
            Ok(quick_xml::events::Event::End(ref e)) => {
                let local = e.local_name();
                let name = local.as_ref();
                let closing = depth;
                depth = depth.saturating_sub(1);
                if ignored_depth.is_some() {
                    if ignored_depth == Some(closing) {
                        ignored_depth = None;
                    }
                    continue;
                }
                if !is_core_element(reader.resolver(), e.name()) {
                    continue;
                }

                match name {
                    "object" if object_depth == Some(closing) => {
                        object_depth = None;
                        if let Some(object) = current.take() {
                            check_indices(&object.vertices, &object.indices)?;
                            objects.push(object);
                        }
                    }
                    "model" if model_depth == Some(closing) => model_depth = None,
                    "resources" if resources_depth == Some(closing) => resources_depth = None,
                    "mesh" if mesh_depth == Some(closing) => mesh_depth = None,
                    "vertices" if vertices_depth == Some(closing) => vertices_depth = None,
                    "triangles" if triangles_depth == Some(closing) => triangles_depth = None,
                    "components" if components_depth == Some(closing) => components_depth = None,
                    "build" if build_depth == Some(closing) => build_depth = None,
                    _ => {}
                }
            }
            Ok(quick_xml::events::Event::Eof) => break,
            Err(e) => {
                return Err(IoError::ParseError {
                    reason: format!("XML parse error: {e}"),
                });
            }
            _ => {}
        }
    }

    if objects.is_empty() {
        return Err(IoError::ParseError {
            reason: "no objects found in 3MF model".to_string(),
        });
    }

    resolve_objects(objects, &items, limits)
}

/// Flatten every root into a mesh, placing each component's object by the
/// composed transforms and the root by its item transform. The roots are
/// the build items, or every object no component references when there are
/// none.
///
/// Fails on a reference to a missing or ambiguous object id, on a component
/// cycle, on nesting deeper than [`MAX_COMPONENT_DEPTH`], and when the
/// flattened output, counted with item multiplicity, would exceed the entity
/// limit. All of these are decided before any flattened geometry is
/// allocated.
fn resolve_objects(
    mut objects: Vec<ParsedObject>,
    items: &[BuildItem],
    limits: ImportLimits,
) -> Result<Vec<TriangleMesh>, IoError> {
    // `None` marks an id that more than one object claims.
    let mut by_id: HashMap<u32, Option<usize>> = HashMap::new();
    for (index, object) in objects.iter().enumerate() {
        if let Some(id) = object.id {
            by_id
                .entry(id)
                .and_modify(|slot| *slot = None)
                .or_insert(Some(index));
        }
    }

    let mut referenced = vec![false; objects.len()];
    let mut children = Vec::with_capacity(objects.len());
    for object in &objects {
        let mut placed = Vec::with_capacity(object.components.len());
        for component in &object.components {
            let child = lookup_object(&by_id, component.object_id, "component")?;
            referenced[child] = true;
            placed.push((child, component.transform));
        }
        children.push(placed);
    }

    let mut extents = vec![Extent::Unvisited; objects.len()];
    for index in 0..objects.len() {
        flattened_extent(index, &objects, &children, &mut extents, 0)?;
    }

    // Validate the complete graph above before pruning evaluation work. A
    // geometry-empty DAG can have exponentially many paths even though its
    // flattened vertex and triangle counts are zero. Such paths contribute
    // no output, so discard their edges without changing `referenced`: root
    // selection must still use the original component relationships.
    for child_list in &mut children {
        child_list.retain(|&(child, _)| {
            !matches!(
                extents[child],
                Extent::Done(FlatSize {
                    vertices: 0,
                    triangles: 0,
                    ..
                })
            )
        });
    }

    // `None` leaves the root unplaced.
    let roots: Vec<(usize, Option<Affine>)> = if items.is_empty() {
        (0..objects.len())
            .filter(|&i| !referenced[i])
            .map(|i| (i, None))
            .collect()
    } else {
        items
            .iter()
            .map(|item| {
                lookup_object(&by_id, item.object_id, "build item")
                    .map(|index| (index, item.transform))
            })
            .collect::<Result<_, _>>()?
    };
    let mut root_uses = vec![0usize; objects.len()];
    let (mut vertices, mut triangles) = (0usize, 0usize);
    for &(root, _) in &roots {
        root_uses[root] += 1;
        if let Extent::Done(size) = extents[root] {
            vertices = vertices.saturating_add(size.vertices);
            triangles = triangles.saturating_add(size.triangles);
        }
    }
    ensure_limit(
        "3MF flattened vertices",
        vertices,
        limits.max_model_entities,
    )?;
    ensure_limit(
        "3MF flattened triangles",
        triangles,
        limits.max_model_entities,
    )?;

    let mut meshes = Vec::with_capacity(roots.len());
    for (root, transform) in roots {
        if children[root].is_empty() && !referenced[root] && root_uses[root] == 1 {
            // Nothing else uses this object's geometry, so its buffers can
            // move into the mesh and be placed in place.
            let object = &mut objects[root];
            let mut vertices = std::mem::take(&mut object.vertices);
            let mut indices = std::mem::take(&mut object.indices);
            if let Some(transform) = transform {
                place_in_place(&transform, &mut vertices, &mut indices)?;
            }
            meshes.push(build_mesh(vertices, indices)?);
            continue;
        }
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        append_flattened(
            root,
            &transform.unwrap_or(Affine::IDENTITY),
            &objects,
            &children,
            &mut vertices,
            &mut indices,
        )?;
        meshes.push(build_mesh(vertices, indices)?);
    }
    Ok(meshes)
}

/// The object `object_id` names; `what` is the referencing element.
fn lookup_object(
    by_id: &HashMap<u32, Option<usize>>,
    object_id: u32,
    what: &str,
) -> Result<usize, IoError> {
    match by_id.get(&object_id) {
        Some(Some(index)) => Ok(*index),
        Some(None) => Err(IoError::ParseError {
            reason: format!("3MF {what} references object id {object_id} defined more than once"),
        }),
        None => Err(IoError::ParseError {
            reason: format!("3MF {what} references unknown object id {object_id}"),
        }),
    }
}

/// Apply a root's item transform to geometry it owns outright.
fn place_in_place(
    transform: &Affine,
    vertices: &mut [Point3],
    indices: &mut [u32],
) -> Result<(), IoError> {
    for p in vertices.iter_mut() {
        *p = transform.apply(*p);
        if p.0.iter().any(|coordinate| !coordinate.is_finite()) {
            return Err(placement_range_error());
        }
    }
    if transform.is_mirroring() {
        // Keep the winding outward under a mirroring placement.
        for tri in indices.chunks_exact_mut(3) {
            tri.swap(1, 2);
        }
    }
    Ok(())
}

fn placement_range_error() -> IoError {
    IoError::ParseError {
        reason: "3MF component or build-item transform leaves the finite millimeter range".into(),
    }
}

/// Flattened size of an object and the nesting depth below it.
#[derive(Debug, Clone, Copy)]
struct FlatSize {
    vertices: usize,
    triangles: usize,
    depth: usize,
}

#[derive(Debug, Clone, Copy)]
enum Extent {
    Unvisited,
    InProgress,
    Done(FlatSize),
}

/// Memoized flattened size of `index`; `path_len` is the number of objects
/// above it on the current resolution path.
fn flattened_extent(
    index: usize,
    objects: &[ParsedObject],
    children: &[Vec<(usize, Affine)>],
    extents: &mut [Extent],
    path_len: usize,
) -> Result<FlatSize, IoError> {
    match extents[index] {
        Extent::Done(size) => return Ok(size),
        Extent::InProgress => {
            return Err(IoError::ParseError {
                reason: format!(
                    "3MF component cycle through object id {}",
                    objects[index].id.unwrap_or_default()
                ),
            });
        }
        Extent::Unvisited => {}
    }
    if path_len > MAX_COMPONENT_DEPTH {
        return Err(component_depth_error());
    }
    extents[index] = Extent::InProgress;
    let object = &objects[index];
    let mut size = FlatSize {
        vertices: object.vertices.len(),
        triangles: object.indices.len() / 3,
        depth: 0,
    };
    for &(child, _) in &children[index] {
        let child = flattened_extent(child, objects, children, extents, path_len + 1)?;
        size.vertices = size.vertices.saturating_add(child.vertices);
        size.triangles = size.triangles.saturating_add(child.triangles);
        size.depth = size.depth.max(child.depth + 1);
    }
    // A child resolved earlier through a shorter path is memoized; its depth
    // still counts against every path that reaches it.
    if size.depth > MAX_COMPONENT_DEPTH {
        return Err(component_depth_error());
    }
    extents[index] = Extent::Done(size);
    Ok(size)
}

fn component_depth_error() -> IoError {
    IoError::ParseError {
        reason: format!("3MF components nest deeper than {MAX_COMPONENT_DEPTH} levels"),
    }
}

/// Append `index`'s own triangles and, recursively, its components' in the
/// root object's frame. `to_root` maps this object's coordinates there.
/// [`flattened_extent`] has already bounded the depth and output size.
fn append_flattened(
    index: usize,
    to_root: &Affine,
    objects: &[ParsedObject],
    children: &[Vec<(usize, Affine)>],
    vertices: &mut Vec<Point3>,
    indices: &mut Vec<u32>,
) -> Result<(), IoError> {
    let object = &objects[index];
    let base = u32::try_from(vertices.len()).map_err(|_| flattened_index_error())?;
    for &p in &object.vertices {
        let placed = to_root.apply(p);
        if placed.0.iter().any(|coordinate| !coordinate.is_finite()) {
            return Err(placement_range_error());
        }
        vertices.push(placed);
    }
    let mirrored = to_root.is_mirroring();
    for tri in object.indices.chunks_exact(3) {
        let [a, b, c] =
            [tri[0], tri[1], tri[2]].map(|i| i.checked_add(base).ok_or_else(flattened_index_error));
        let (a, b, c) = (a?, b?, c?);
        // Keep the winding outward under a mirroring placement.
        indices.extend(if mirrored { [a, c, b] } else { [a, b, c] });
    }
    for (child, placement) in &children[index] {
        append_flattened(
            *child,
            &placement.then(to_root),
            objects,
            children,
            vertices,
            indices,
        )?;
    }
    Ok(())
}

fn flattened_index_error() -> IoError {
    IoError::ParseError {
        reason: "3MF flattened vertex index exceeds u32".into(),
    }
}

/// Parse an `<object>`'s optional `id`.
fn parse_object_id(e: &quick_xml::events::BytesStart<'_>) -> Result<Option<u32>, IoError> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == "id" {
            return parse_u32_attr(attr.value.as_ref(), "object id").map(Some);
        }
    }
    Ok(None)
}

/// Record a `<component>` (written either empty or with an end tag) on the
/// object being parsed.
fn push_component(
    e: &quick_xml::events::BytesStart<'_>,
    resolver: &NamespaceResolver,
    unit_scale: f64,
    object: Option<&mut ParsedObject>,
) -> Result<(), IoError> {
    let component = parse_component(e, resolver, unit_scale)?;
    if let Some(object) = object {
        object.components.push(component);
    }
    Ok(())
}

/// Record a build `<item>` (written either empty or with an end tag).
fn push_item(
    e: &quick_xml::events::BytesStart<'_>,
    resolver: &NamespaceResolver,
    unit_scale: f64,
    items: &mut Vec<BuildItem>,
    limits: ImportLimits,
) -> Result<(), IoError> {
    ensure_limit(
        "3MF build items",
        items.len().saturating_add(1),
        limits.max_model_entities,
    )?;
    let (object_id, transform) = parse_placement(e, resolver, unit_scale, "build item")?;
    items.push(BuildItem {
        object_id,
        transform,
    });
    Ok(())
}

/// Parse a `<component>`'s `objectid` and optional `transform`.
fn parse_component(
    e: &quick_xml::events::BytesStart<'_>,
    resolver: &NamespaceResolver,
    unit_scale: f64,
) -> Result<Component, IoError> {
    let (object_id, transform) = parse_placement(e, resolver, unit_scale, "component")?;
    Ok(Component {
        object_id,
        transform: transform.unwrap_or(Affine::IDENTITY),
    })
}

/// Parse the `objectid` and optional `transform` shared by `<component>`
/// and `<item>`; `what` names the element in errors.
fn parse_placement(
    e: &quick_xml::events::BytesStart<'_>,
    resolver: &NamespaceResolver,
    unit_scale: f64,
    what: &str,
) -> Result<(u32, Option<Affine>), IoError> {
    let mut object_id = None;
    let mut transform = None;
    for attr in e.attributes().flatten() {
        // Core attributes are unqualified; a prefixed `transform` or
        // `objectid` belongs to an extension and must not override them.
        let qualified = attr.key.prefix().is_some();
        match attr.key.local_name().as_ref() {
            "objectid" if !qualified => {
                // `xs:positiveInteger` collapses surrounding XML whitespace.
                let value = normalized_attribute(&attr, what)?;
                let value = value.trim_matches(|c| matches!(c, ' ' | '\t' | '\r' | '\n'));
                object_id = Some(parse_u32_attr(value, &format!("{what} objectid"))?);
            }
            "transform" if !qualified => {
                let value = normalized_attribute(&attr, what)?;
                transform = Some(parse_transform(&value, unit_scale)?);
            }
            // Production extension (`p:path`): the object lives in another
            // model part, where `objectid` names a different object than it
            // does here. An undeclared prefix is refused too, rather than
            // risk a wrong placement; another extension's `path` is ignored.
            "path" if references_production_part(resolver, attr.key) => {
                return Err(IoError::ParseError {
                    reason: format!(
                        "3MF {what} references another model part (production extension), \
                         which is not supported"
                    ),
                });
            }
            _ => {}
        }
    }
    let object_id = object_id.ok_or_else(|| IoError::ParseError {
        reason: format!("{what} element missing objectid attribute"),
    })?;
    Ok((object_id, transform))
}

/// An attribute value with character references resolved, so `&#49;` reads
/// as `1`.
fn normalized_attribute<'a>(
    attr: &'a quick_xml::events::attributes::Attribute<'_>,
    what: &str,
) -> Result<std::borrow::Cow<'a, str>, IoError> {
    attr.normalized_value(quick_xml::XmlVersion::Implicit1_0)
        .map_err(|error| IoError::ParseError {
            reason: format!("invalid 3MF {what} attribute: {error}"),
        })
}

/// Whether a `path` attribute is the production extension's (or carries a
/// prefix no namespace declaration binds).
fn references_production_part(resolver: &NamespaceResolver, key: QName<'_>) -> bool {
    match resolver.resolve_attribute(key).0 {
        ResolveResult::Unbound => false,
        ResolveResult::Bound(Namespace(ns)) => ns == PRODUCTION_NS,
        ResolveResult::Unknown(_) => true,
    }
}

/// Parse a 12-value 3MF transform; the translation is scaled to millimeters.
fn parse_transform(value: &str, unit_scale: f64) -> Result<Affine, IoError> {
    let invalid = || IoError::ParseError {
        reason: format!("invalid 3MF transform '{value}': expected 12 finite numbers"),
    };
    let mut m = [0.0; 12];
    let mut values = value.split_whitespace();
    for slot in &mut m {
        *slot = values
            .next()
            .ok_or_else(invalid)?
            .parse::<f64>()
            .map_err(|_| invalid())?;
    }
    for translation in &mut m[9..] {
        *translation *= unit_scale;
    }
    if values.next().is_some() || m.iter().any(|v| !v.is_finite()) {
        return Err(invalid());
    }
    Ok(Affine(m))
}

/// The core format defaults to millimeters when `unit` is omitted.
fn model_unit_scale(model: &quick_xml::events::BytesStart<'_>) -> Result<f64, IoError> {
    for attribute in model.attributes() {
        let attribute = attribute.map_err(|error| IoError::ParseError {
            reason: format!("invalid 3MF model attribute: {error}"),
        })?;
        if attribute.key.as_ref() == "unit" {
            let unit = attribute
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .map_err(|error| IoError::ParseError {
                    reason: format!("invalid 3MF unit: {error}"),
                })?;
            return match unit.as_ref() {
                "micron" => Ok(0.001),
                "millimeter" => Ok(1.0),
                "centimeter" => Ok(10.0),
                "inch" => Ok(25.4),
                "foot" => Ok(304.8),
                "meter" => Ok(1000.0),
                _ => Err(IoError::ParseError {
                    reason: format!("unsupported 3MF model unit: {unit}"),
                }),
            };
        }
    }
    Ok(1.0)
}

/// Parse `x`, `y`, `z` attributes from a `<vertex>` element.
fn parse_vertex_attributes(e: &quick_xml::events::BytesStart<'_>) -> Result<Point3, IoError> {
    let mut x = None;
    let mut y = None;
    let mut z = None;

    for attr in e.attributes().flatten() {
        let key = attr.key.as_ref();
        let val = attr.value.as_ref();
        match key {
            "x" => x = Some(parse_f64_attr(val, "vertex x")?),
            "y" => y = Some(parse_f64_attr(val, "vertex y")?),
            "z" => z = Some(parse_f64_attr(val, "vertex z")?),
            _ => {}
        }
    }

    match (x, y, z) {
        (Some(x), Some(y), Some(z)) => Ok(Point3::new(x, y, z)),
        _ => Err(IoError::ParseError {
            reason: "vertex element missing x, y, or z attribute".to_string(),
        }),
    }
}

/// Parse `v1`, `v2`, `v3` attributes from a `<triangle>` element.
fn parse_triangle_attributes(
    e: &quick_xml::events::BytesStart<'_>,
) -> Result<(u32, u32, u32), IoError> {
    let mut v1 = None;
    let mut v2 = None;
    let mut v3 = None;

    for attr in e.attributes().flatten() {
        let key = attr.key.as_ref();
        let val = attr.value.as_ref();
        match key {
            "v1" => v1 = Some(parse_u32_attr(val, "triangle v1")?),
            "v2" => v2 = Some(parse_u32_attr(val, "triangle v2")?),
            "v3" => v3 = Some(parse_u32_attr(val, "triangle v3")?),
            _ => {}
        }
    }

    match (v1, v2, v3) {
        (Some(a), Some(b), Some(c)) => Ok((a, b, c)),
        _ => Err(IoError::ParseError {
            reason: "triangle element missing v1, v2, or v3 attribute".to_string(),
        }),
    }
}

/// Build a [`TriangleMesh`] from indexed vertices, computing per-vertex normals.
fn build_mesh(vertices: Vec<Point3>, indices: Vec<u32>) -> Result<TriangleMesh, IoError> {
    check_indices(&vertices, &indices)?;

    let mut normals = vec![Vec3::new(0.0, 0.0, 0.0); vertices.len()];

    for tri in indices.chunks_exact(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;

        let v0 = vertices[i0];
        let v1 = vertices[i1];
        let v2 = vertices[i2];

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let face_normal = edge1.cross(edge2);

        normals[i0] += face_normal;
        normals[i1] += face_normal;
        normals[i2] += face_normal;
    }

    let up = Vec3::new(0.0, 0.0, 1.0);
    for n in &mut normals {
        *n = n.normalize().unwrap_or(up);
    }

    Ok(TriangleMesh {
        positions: vertices,
        normals,
        indices,
    })
}

/// Reject triangles that are incomplete or index past the object's vertices.
fn check_indices(vertices: &[Point3], indices: &[u32]) -> Result<(), IoError> {
    if !indices.len().is_multiple_of(3) {
        return Err(IoError::ParseError {
            reason: format!(
                "triangle indices count {} is not a multiple of 3",
                indices.len()
            ),
        });
    }

    for tri in indices.chunks_exact(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;

        if i0 >= vertices.len() || i1 >= vertices.len() || i2 >= vertices.len() {
            return Err(IoError::ParseError {
                reason: format!(
                    "triangle index out of bounds: [{i0}, {i1}, {i2}] but only {} vertices",
                    vertices.len(),
                ),
            });
        }
    }
    Ok(())
}

/// Parse a float attribute value.
fn parse_f64_attr(val: &str, context: &str) -> Result<f64, IoError> {
    val.parse::<f64>().map_err(|e| IoError::ParseError {
        reason: format!("invalid {context} value '{val}': {e}"),
    })
}

/// Parse a u32 attribute value.
fn parse_u32_attr(val: &str, context: &str) -> Result<u32, IoError> {
    val.parse::<u32>().map_err(|e| IoError::ParseError {
        reason: format!("invalid {context} value '{val}': {e}"),
    })
}

/// Read a 3MF file and import its first root (the first build item) as a
/// solid.
///
/// This is a convenience wrapper that calls [`read_threemf`] followed by
/// [`import_mesh`](crate::stl::import::import_mesh) on the first mesh.
/// Vertices within `tolerance` of each other are merged.
///
/// **Note:** Only the first root is imported; additional build items are
/// silently ignored. Use [`read_threemf`] directly to access all of them.
///
/// # Errors
///
/// Returns [`IoError`] if:
/// - The file is malformed
/// - The archive contains no objects
/// - The mesh cannot be converted to a valid solid
pub fn read_threemf_solid(
    topo: &mut Topology,
    data: &[u8],
    tolerance: f64,
) -> Result<SolidId, IoError> {
    let meshes = read_threemf(data)?;
    let mesh = meshes
        .into_iter()
        .next()
        .ok_or_else(|| IoError::ParseError {
            reason: "3MF file contains no objects".to_string(),
        })?;
    crate::stl::import::import_mesh(topo, &mesh, tolerance)
}

/// Read the first 3MF root (build item) as a solid with explicit import
/// limits.
///
/// # Errors
///
/// Returns [`IoError`] when a limit is exceeded, the 3MF data is invalid, or
/// the first mesh cannot be converted into a solid.
pub fn read_threemf_solid_with_limits(
    topo: &mut Topology,
    data: &[u8],
    tolerance: f64,
    limits: ImportLimits,
) -> Result<SolidId, IoError> {
    let meshes = read_threemf_with_limits(data, limits)?;
    let mesh = meshes
        .into_iter()
        .next()
        .ok_or_else(|| IoError::ParseError {
            reason: "3MF file contains no objects".to_string(),
        })?;
    crate::stl::import::import_mesh(topo, &mesh, tolerance)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_cube_non_manifold;

    use super::*;
    use crate::threemf::writer;

    fn tiny_archive() -> Vec<u8> {
        let mut zip = zip::ZipWriter::new(Cursor::new(Vec::new()));
        zip.start_file(
            "3D/3dmodel.model",
            zip::write::SimpleFileOptions::default()
                .compression_method(zip::CompressionMethod::Deflated),
        )
        .unwrap();
        std::io::Write::write_all(&mut zip, b"<model><resources></resources></model>").unwrap();
        zip.finish().unwrap().into_inner()
    }

    fn forge_declared_size(bytes: &mut [u8], size: u32) {
        let central = bytes.windows(4).position(|w| w == b"PK\x01\x02").unwrap();
        bytes[central + 24..central + 28].copy_from_slice(&size.to_le_bytes());
        bytes[22..26].copy_from_slice(&size.to_le_bytes());
    }

    #[test]
    fn forged_zip_size_does_not_reserve_the_declared_entry() {
        let mut bytes = tiny_archive();
        assert!(bytes.len() < 1024);
        let limits = ImportLimits::default();
        forge_declared_size(&mut bytes, limits.max_archive_entry_bytes as u32);
        let xml = extract_model_xml(&bytes, limits).unwrap();
        assert_eq!(xml, "<model><resources></resources></model>");
        assert!(
            xml.capacity() <= 64 * 1024,
            "untrusted metadata reserved {} bytes",
            xml.capacity()
        );
        // Parsing the extracted XML still follows the ordinary format rules.
        assert!(matches!(
            read_threemf(&bytes),
            Err(IoError::ParseError { .. })
        ));
    }

    #[test]
    fn understated_xml_size_is_rejected_before_the_import_limit() {
        let mut bytes = tiny_archive();
        forge_declared_size(&mut bytes, 1);
        let limits = ImportLimits {
            max_archive_entry_bytes: 8,
            ..ImportLimits::default()
        };
        // zip rejects output beyond the declared byte count before our
        // bounded read reaches the higher application limit.
        let error = extract_model_xml(&bytes, limits).unwrap_err();
        assert!(
            matches!(
                &error,
                IoError::Io(error) if error.kind() == std::io::ErrorKind::InvalidData
                    && error.to_string().contains("declared uncompressed size")
            ),
            "unexpected refusal: {error}"
        );
    }

    #[test]
    fn roundtrip_unit_cube() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = writer::write_threemf(&topo, &[solid], 0.1).unwrap();
        let meshes = read_threemf(&bytes).unwrap();

        assert_eq!(meshes.len(), 1);
        let mesh = &meshes[0];

        // Unit cube: 8 corner vertices (shared across faces via watertight tessellation).
        assert_eq!(mesh.positions.len(), 8);
        // 12 triangles × 3 indices = 36.
        assert_eq!(mesh.indices.len(), 36);
    }

    #[test]
    fn rejects_archive_entry_larger_than_explicit_limit() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);
        let bytes = writer::write_threemf(&topo, &[solid], 0.1).unwrap();
        let limits = ImportLimits {
            max_archive_entry_bytes: 1,
            ..ImportLimits::default()
        };

        let err = read_threemf_with_limits(&bytes, limits).unwrap_err();
        assert!(matches!(
            err,
            IoError::LimitExceeded {
                resource: "3MF model XML bytes",
                ..
            }
        ));
    }

    #[test]
    fn roundtrip_box_primitive() {
        let mut topo = Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();

        let bytes = writer::write_threemf(&topo, &[solid], 0.1).unwrap();
        let meshes = read_threemf(&bytes).unwrap();

        assert_eq!(meshes.len(), 1);
        assert_eq!(meshes[0].indices.len(), 36); // 12 triangles.
    }

    #[test]
    fn roundtrip_multiple_solids() {
        let mut topo = Topology::new();
        let s1 = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let s2 = make_unit_cube_non_manifold(&mut topo);

        let bytes = writer::write_threemf(&topo, &[s1, s2], 0.1).unwrap();
        let meshes = read_threemf(&bytes).unwrap();

        assert_eq!(meshes.len(), 2);
    }

    #[test]
    fn roundtrip_preserves_vertex_bounds() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = writer::write_threemf(&topo, &[solid], 0.1).unwrap();
        let meshes = read_threemf(&bytes).unwrap();

        for pos in &meshes[0].positions {
            assert!(pos.x() >= -0.01 && pos.x() <= 1.01);
            assert!(pos.y() >= -0.01 && pos.y() <= 1.01);
            assert!(pos.z() >= -0.01 && pos.z() <= 1.01);
        }
    }

    #[test]
    fn normals_are_unit_length() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = writer::write_threemf(&topo, &[solid], 0.1).unwrap();
        let meshes = read_threemf(&bytes).unwrap();

        for n in &meshes[0].normals {
            let len = n.length();
            assert!(
                (len - 1.0).abs() < 1e-6,
                "normal length should be ~1.0, got {len}"
            );
        }
    }

    #[test]
    fn invalid_zip_data() {
        let result = read_threemf(b"this is not a zip file");
        assert!(result.is_err());
    }

    #[test]
    fn missing_model_entry() {
        let buf = std::io::Cursor::new(Vec::new());
        let mut zip = zip::ZipWriter::new(buf);
        let options = zip::write::SimpleFileOptions::default();
        zip.start_file("dummy.txt", options).unwrap();
        std::io::Write::write_all(&mut zip, b"hello").unwrap();
        let cursor = zip.finish().unwrap();

        let result = read_threemf(&cursor.into_inner());
        assert!(result.is_err());
    }

    #[test]
    fn read_threemf_solid_returns_solid_id() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_non_manifold(&mut topo);

        let bytes = writer::write_threemf(&topo, &[solid], 0.1).unwrap();

        let mut import_topo = Topology::new();
        let result = read_threemf_solid(&mut import_topo, &bytes, 1e-6);
        assert!(
            result.is_ok(),
            "read_threemf_solid should return Ok: {result:?}"
        );
    }

    fn unit_model(unit: Option<&str>, coordinate: &str) -> String {
        let unit = unit.map_or_else(String::new, |unit| format!(" unit=\"{unit}\""));
        format!(
            r#"<model{unit}><resources><object id="1" type="model"><mesh><vertices><vertex x="0" y="0" z="0"/><vertex x="{coordinate}" y="0" z="0"/><vertex x="0" y="{coordinate}" z="0"/><vertex x="0" y="0" z="{coordinate}"/></vertices><triangles><triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="1" v3="3"/><triangle v1="0" v2="3" v3="2"/><triangle v1="1" v2="2" v3="3"/></triangles></mesh></object></resources><build><item objectid="1"/></build></model>"#
        )
    }

    #[test]
    fn model_units_are_normalized_to_millimeters() {
        for (unit, scale) in [
            (None, 1.0),
            (Some("micron"), 0.001),
            (Some("millimeter"), 1.0),
            (Some("centimeter"), 10.0),
            (Some("inch"), 25.4),
            (Some("in&#99;h"), 25.4),
            (Some("foot"), 304.8),
            (Some("meter"), 1000.0),
        ] {
            let xml = unit_model(unit, "1");
            let meshes = parse_model_xml(&xml, ImportLimits::default()).unwrap();
            assert_eq!(meshes[0].positions[1], Point3::new(scale, 0.0, 0.0));
            assert_eq!(meshes[0].positions[2], Point3::new(0.0, scale, 0.0));
            assert_eq!(meshes[0].positions[3], Point3::new(0.0, 0.0, scale));
            assert!(
                meshes[0]
                    .normals
                    .iter()
                    .all(|normal| normal.0.iter().all(|value| value.is_finite()))
            );
        }
    }

    #[test]
    fn invalid_units_and_scaled_coordinate_overflow_are_refused() {
        for (unit, coordinate) in [("unknown", "1"), ("meter", "1e308")] {
            assert!(matches!(
                parse_model_xml(&unit_model(Some(unit), coordinate), ImportLimits::default()),
                Err(IoError::ParseError { .. })
            ));
        }
    }

    #[test]
    fn unsupported_namespace_subtrees_cannot_change_core_geometry() {
        let xml = unit_model(Some("millimeter"), "1").replacen(
            "<model",
            "<model xmlns:v=\"urn:remus-test:optional-extension\"",
            1,
        );
        let expected = parse_model_xml(&xml, ImportLimits::default()).unwrap();
        let variants = [
            xml.replace("<resources>", "<resources><v:model unit=\"inch\"></v:model>"),
            xml.replace(
                "<mesh>",
                "<v:object id=\"99\"><vertices><vertex x=\"9\" y=\"9\" z=\"9\"/></vertices></v:object><mesh>",
            ),
            xml.replace(
                "<vertices>",
                "<vertices><v:vertices><vertex x=\"9\" y=\"9\" z=\"9\"/></v:vertices><v:vertex x=\"8\" y=\"8\" z=\"8\"/>",
            ),
            xml.replace(
                "<triangles>",
                "<triangles><v:triangles><triangle v1=\"999\" v2=\"999\" v3=\"999\"/></v:triangles><v:triangle v1=\"999\" v2=\"999\" v3=\"999\"/>",
            ),
            xml.replace(
                "<resources>",
                "<resources><v:extension><model unit=\"inch\"><object id=\"1\"/></model></v:extension>",
            ),
        ];
        for variant in variants {
            let actual = parse_model_xml(&variant, ImportLimits::default()).unwrap();
            assert_eq!(actual.len(), expected.len(), "{variant}");
            assert_eq!(actual[0].positions, expected[0].positions, "{variant}");
            assert_eq!(actual[0].indices, expected[0].indices, "{variant}");
            assert_eq!(actual[0].normals, expected[0].normals, "{variant}");
        }
    }

    #[test]
    fn core_geometry_state_requires_the_expected_parent_depth() {
        let xml = unit_model(Some("millimeter"), "1");
        let expected = parse_model_xml(&xml, ImportLimits::default()).unwrap();
        for variant in [
            xml.replace(
                "<resources>",
                "<metadata><model unit=\"inch\"></model></metadata><resources>",
            ),
            xml.replace(
                "<vertices>",
                "<vertices><metadata><vertex x=\"9\" y=\"9\" z=\"9\"/></metadata>",
            ),
            xml.replace(
                "<triangles>",
                "<triangles><metadata><triangle v1=\"999\" v2=\"999\" v3=\"999\"/></metadata>",
            ),
        ] {
            let actual = parse_model_xml(&variant, ImportLimits::default()).unwrap();
            assert_eq!(actual.len(), expected.len(), "{variant}");
            assert_eq!(actual[0].positions, expected[0].positions, "{variant}");
            assert_eq!(actual[0].indices, expected[0].indices, "{variant}");
        }
    }

    #[test]
    fn bound_core_namespaces_preserve_the_unbound_geometry() {
        let xml = unit_model(Some("millimeter"), "1");
        let expected = parse_model_xml(&xml, ImportLimits::default()).unwrap();
        let default_namespace = xml.replacen("<model", &format!("<model xmlns=\"{CORE_NS}\""), 1);
        let mut prefixed = xml;
        for tag in [
            "model",
            "resources",
            "object",
            "mesh",
            "vertices",
            "vertex",
            "triangles",
            "triangle",
            "build",
            "item",
        ] {
            prefixed = prefixed.replace(&format!("<{tag}"), &format!("<core:{tag}"));
            prefixed = prefixed.replace(&format!("</{tag}"), &format!("</core:{tag}"));
        }
        prefixed = prefixed.replacen(
            "<core:model",
            &format!("<core:model xmlns:core=\"{CORE_NS}\""),
            1,
        );
        for variant in [default_namespace, prefixed] {
            let actual = parse_model_xml(&variant, ImportLimits::default()).unwrap();
            assert_eq!(actual.len(), expected.len());
            assert_eq!(actual[0].positions, expected[0].positions);
            assert_eq!(actual[0].indices, expected[0].indices);
        }
    }
}
