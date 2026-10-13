//! Hand-written JSON encoding and decoding of arena documents.
//!
//! The format is the one `serde_json` produced from the `Ser*` mirror types
//! with `#[derive(Serialize, Deserialize)]`; this module reproduces it
//! without instantiating the generic serde machinery per type, which kept a
//! few hundred kilobytes of code in the browser kernel. Output is byte for
//! byte what `serde_json::to_vec` wrote, and the reader accepts and refuses
//! exactly what the derived impls accepted and refused, with the same
//! [`IoError`] kinds and the same order of limit checks:
//!
//! 1. the encoded byte budget;
//! 2. a syntax check of the whole document plus its `version` header (the
//!    old `VersionHeader` pass);
//! 3. for every version but 1, the journal item budget, counted over the
//!    encoded `journal` values before anything is materialized;
//! 4. the typed decode of the version's schema;
//! 5. the table and reference budgets of [`check_document_limits`].
//!
//! Documents whose first key is the `version` (every writer emits it first)
//! take a single typed pass instead: a typed decode that succeeds has
//! checked every byte, and the journal budget is counted before the journal
//! is materialized, so it reaches the same result as steps 2-4. Any failure
//! on that path reruns the staged steps, which decide the error kind.
//!
//! Struct rules follow `serde`'s derive: an object keyed by field name
//! (unknown keys skipped, repeated keys refused, `Option` fields optional)
//! or an array of fields in declaration order (only `#[serde(default)]`
//! fields optional). Enums are externally tagged, except the journal's
//! internally tagged events and flattened payload, which are decoded from a
//! buffered [`Content`] tree exactly as `serde` buffers them.
//!
//! The math carriers keep private fields and no unchecked constructors, so
//! their values are decoded here and handed to their derived `Deserialize`
//! impls through a tiny in-memory deserializer ([`Shape`]), which populates
//! the fields directly with no validation, as before.

use serde::de::{DeserializeOwned, DeserializeSeed, SeqAccess, Visitor};

use super::{
    FORMAT_VERSION, ImportLimits, IoError, LEGACY_MULTI_ROOT_VERSION, LEGACY_SINGLE_SOLID_VERSION,
    ParsedDocument, SHEET_ROOT_FORMAT_VERSION, SerAttributes, SerBodyClass, SerBoundaryAuthority,
    SerBoundaryAuthorityView, SerCoedgeAuthority, SerCoedgeAuthorityView, SerCompound, SerEdge,
    SerEdgeCurve, SerEdgeCurveView, SerEdgeView, SerEntityAttributes, SerFace,
    SerFaceLoopAuthority, SerFaceSurface, SerFaceSurfaceView, SerFaceView, SerJournal,
    SerJournalEntry, SerJournalEvent, SerJournalIndexEntry, SerJournalPayload, SerLoopAuthority,
    SerOrientedEdge, SerPCurve, SerShell, SerSolid, SerUsePCurve, SerVertex, SerWire,
    WIRE_ROOT_FORMAT_VERSION, check_document_limits,
};
use crate::json::{Content, Error, MAX_DEPTH, Reader, Res, Writer};
use crate::limits::ensure_input_size;
use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::curves2d::Curve2D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point2, Point3, Position, Vector};

// ===========================================================================
// Decoding
// ===========================================================================

fn parse_error(context: &str, error: Error) -> IoError {
    IoError::ParseError {
        reason: format!("{context}: {error}"),
    }
}

/// Parses and limit-checks an arena document of any supported version.
pub(super) fn parse_document(
    bytes: &[u8],
    limits: ImportLimits,
) -> Result<ParsedDocument, IoError> {
    ensure_input_size(bytes.len(), limits)?;
    // One reader, so the input's UTF-8 is validated once for all passes.
    let mut reader = Reader::new(bytes);
    if let Some(version) = leading_version(bytes) {
        let budget = Some(limits.max_model_entities);
        if let Ok(document) = read_document(&mut reader, version, budget)
            && reader.at_end()
        {
            check_document_limits(&document, limits)?;
            return Ok(document);
        }
        reader.seek(0);
    }
    let header = scan_header(&mut reader)
        .map_err(|error| parse_error("arena deserialization failed", error))?;
    if header.version != LEGACY_SINGLE_SOLID_VERSION {
        preflight_journal(&mut reader, &header.journals, limits.max_model_entities)?;
    }
    let version = header.version;
    let context = match version {
        LEGACY_SINGLE_SOLID_VERSION => "arena v1 deserialization failed",
        LEGACY_MULTI_ROOT_VERSION => "arena v2 deserialization failed",
        FORMAT_VERSION => "arena v3 deserialization failed",
        SHEET_ROOT_FORMAT_VERSION => "arena v4 deserialization failed",
        WIRE_ROOT_FORMAT_VERSION => "arena v5 deserialization failed",
        _ => {
            return Err(IoError::ParseError {
                reason: format!(
                    "unsupported arena dump version {version} (supported: {LEGACY_SINGLE_SOLID_VERSION}, {LEGACY_MULTI_ROOT_VERSION}, {FORMAT_VERSION}, {SHEET_ROOT_FORMAT_VERSION}, {WIRE_ROOT_FORMAT_VERSION})"
                ),
            });
        }
    };
    reader.seek(0);
    let document =
        read_document(&mut reader, version, None).map_err(|error| parse_error(context, error))?;
    check_document_limits(&document, limits)?;
    Ok(document)
}

/// The version of a document that opens with `{"version":<u32>` followed by
/// `,` or `}`, which is how every writer starts one; `None` when the
/// document starts any other way or names an unsupported version.
fn leading_version(bytes: &[u8]) -> Option<u32> {
    let mut reader = Reader::new_unvalidated(bytes);
    reader.begin_object().ok()?;
    let rest = bytes.get(reader.pos()..)?;
    let rest = rest.strip_prefix(b"\"version\"")?;
    let mut reader = Reader::new_unvalidated(rest);
    if reader.peek() != Some(b':') {
        return None;
    }
    reader.seek(reader.pos() + 1);
    if !matches!(reader.peek(), Some(b'1'..=b'9')) {
        return None;
    }
    let version = reader.read_u32().ok()?;
    if !matches!(reader.peek(), Some(b',' | b'}')) {
        return None;
    }
    (LEGACY_SINGLE_SOLID_VERSION..=WIRE_ROOT_FORMAT_VERSION)
        .contains(&version)
        .then_some(version)
}

/// The document's `version` and the offsets of its top-level `journal`
/// values.
struct Header {
    version: u32,
    journals: Vec<usize>,
}

/// Checks the syntax of the whole document and reads its header, like the
/// old `VersionHeader` pass: the root must be an object (an array root never
/// survives the later passes), every top-level key must be valid UTF-8, and
/// `version` must appear exactly once as a `u32`.
fn scan_header(reader: &mut Reader<'_>) -> Res<Header> {
    let mut version = None;
    let mut journals = Vec::new();
    if reader.peek() == Some(b'{') {
        reader.begin_object()?;
    } else {
        reader.skip_value()?;
        return Err(reader.error("invalid type: expected an arena document object"));
    }
    let mut first = true;
    while let Some(key) = reader.next_key(&mut first)? {
        match key.as_ref() {
            "version" => {
                if version.is_some() {
                    return Err(reader.error("duplicate field `version`"));
                }
                version = Some(reader.read_u32()?);
            }
            name => {
                if name == "journal" {
                    reader.peek();
                    journals.push(reader.pos());
                }
                reader.skip_value()?;
            }
        }
    }
    if !reader.at_end() {
        return Err(reader.error("trailing characters"));
    }
    let version = version.ok_or_else(|| reader.error("missing field `version`"))?;
    Ok(Header { version, journals })
}

enum Preflight {
    Parse(Error),
    Limit(usize),
}

impl From<Error> for Preflight {
    fn from(error: Error) -> Self {
        Self::Parse(error)
    }
}

/// Counts the array elements nested in the encoded `journal` values before
/// anything is materialized, as the old journal preflight did: values are
/// visited as typed data (valid strings, finite numbers, bounded nesting)
/// in document order, and the first element past `limit` refuses the
/// document.
fn preflight_journal(
    reader: &mut Reader<'_>,
    journals: &[usize],
    limit: usize,
) -> Result<(), IoError> {
    let mut used = 0_usize;
    for &position in journals {
        reader.seek(position);
        match count_journal_items(reader, 2, &mut used, limit) {
            Ok(()) => {}
            Err(Preflight::Limit(actual)) => {
                return Err(IoError::LimitExceeded {
                    resource: "arena journal items",
                    limit,
                    actual,
                });
            }
            Err(Preflight::Parse(error)) => {
                return Err(parse_error("arena journal preflight failed", error));
            }
        }
    }
    Ok(())
}

fn count_journal_items(
    reader: &mut Reader<'_>,
    depth: usize,
    used: &mut usize,
    limit: usize,
) -> Result<(), Preflight> {
    match reader.peek() {
        Some(b'[') => {
            if depth >= MAX_DEPTH {
                return Err(reader.error("recursion limit exceeded").into());
            }
            reader.begin_array()?;
            let mut first = true;
            while reader.next_element(&mut first)? {
                count_journal_items(reader, depth + 1, used, limit)?;
                match used.checked_add(1) {
                    Some(next) if next <= limit => *used = next,
                    Some(next) => return Err(Preflight::Limit(next)),
                    None => return Err(Preflight::Limit(usize::MAX)),
                }
            }
        }
        Some(b'{') => {
            if depth >= MAX_DEPTH {
                return Err(reader.error("recursion limit exceeded").into());
            }
            reader.begin_object()?;
            let mut first = true;
            while reader.next_key(&mut first)?.is_some() {
                count_journal_items(reader, depth + 1, used, limit)?;
            }
        }
        Some(b'"') => {
            reader.read_string()?;
        }
        Some(b'-' | b'0'..=b'9') => {
            reader.read_number()?;
        }
        Some(b't' | b'f') => {
            reader.read_bool()?;
        }
        Some(b'n') => {
            reader.null()?;
        }
        _ => return Err(reader.error("expected value").into()),
    }
    Ok(())
}

/// Top-level document fields across all versions.
#[derive(Clone, Copy)]
enum DocField {
    Version,
    Vertices,
    Edges,
    Wires,
    Faces,
    Shells,
    OuterShell,
    InnerShells,
    Solids,
    SolidRoots,
    SheetRoots,
    WireRoots,
    Compounds,
    PCurves,
    Boundary,
    Journal,
    Attributes,
}

const V1_FIELDS: &[(&str, DocField)] = &[
    ("version", DocField::Version),
    ("vertices", DocField::Vertices),
    ("edges", DocField::Edges),
    ("wires", DocField::Wires),
    ("faces", DocField::Faces),
    ("shells", DocField::Shells),
    ("outer_shell", DocField::OuterShell),
    ("inner_shells", DocField::InnerShells),
    ("pcurves", DocField::PCurves),
];

const V2_FIELDS: &[(&str, DocField)] = &[
    ("version", DocField::Version),
    ("vertices", DocField::Vertices),
    ("edges", DocField::Edges),
    ("wires", DocField::Wires),
    ("faces", DocField::Faces),
    ("shells", DocField::Shells),
    ("solids", DocField::Solids),
    ("solid_roots", DocField::SolidRoots),
    ("compounds", DocField::Compounds),
    ("pcurves", DocField::PCurves),
    ("journal", DocField::Journal),
    ("attributes", DocField::Attributes),
];

const V3_FIELDS: &[(&str, DocField)] = &[
    ("version", DocField::Version),
    ("vertices", DocField::Vertices),
    ("edges", DocField::Edges),
    ("wires", DocField::Wires),
    ("faces", DocField::Faces),
    ("shells", DocField::Shells),
    ("solids", DocField::Solids),
    ("solid_roots", DocField::SolidRoots),
    ("compounds", DocField::Compounds),
    ("boundary_authority", DocField::Boundary),
    ("journal", DocField::Journal),
    ("attributes", DocField::Attributes),
];

const V4_FIELDS: &[(&str, DocField)] = &[
    ("version", DocField::Version),
    ("vertices", DocField::Vertices),
    ("edges", DocField::Edges),
    ("wires", DocField::Wires),
    ("faces", DocField::Faces),
    ("shells", DocField::Shells),
    ("solids", DocField::Solids),
    ("solid_roots", DocField::SolidRoots),
    ("sheet_roots", DocField::SheetRoots),
    ("compounds", DocField::Compounds),
    ("boundary_authority", DocField::Boundary),
    ("journal", DocField::Journal),
    ("attributes", DocField::Attributes),
];

const V5_FIELDS: &[(&str, DocField)] = &[
    ("version", DocField::Version),
    ("vertices", DocField::Vertices),
    ("edges", DocField::Edges),
    ("wires", DocField::Wires),
    ("faces", DocField::Faces),
    ("shells", DocField::Shells),
    ("solids", DocField::Solids),
    ("solid_roots", DocField::SolidRoots),
    ("sheet_roots", DocField::SheetRoots),
    ("wire_roots", DocField::WireRoots),
    ("compounds", DocField::Compounds),
    ("boundary_authority", DocField::Boundary),
    ("journal", DocField::Journal),
    ("attributes", DocField::Attributes),
];

/// Values of the top-level fields seen so far.
#[derive(Default)]
struct DocumentFields {
    version: Option<u32>,
    vertices: Option<Vec<SerVertex>>,
    edges: Option<Vec<SerEdge>>,
    wires: Option<Vec<SerWire>>,
    faces: Option<Vec<SerFace>>,
    shells: Option<Vec<SerShell>>,
    outer_shell: Option<usize>,
    inner_shells: Option<Vec<usize>>,
    solids: Option<Vec<SerSolid>>,
    solid_roots: Option<Vec<usize>>,
    sheet_roots: Option<Vec<usize>>,
    wire_roots: Option<Vec<usize>>,
    compounds: Option<Vec<SerCompound>>,
    pcurves: Option<Vec<SerPCurve>>,
    boundary: Option<SerBoundaryAuthority>,
    journal: Option<SerJournal>,
    attributes: Option<SerAttributes>,
}

/// Up to 16 names of a field table, borrowed for [`Reader::read_struct`].
fn names_of<T>(fields: &[(&'static str, T)]) -> ([&'static str; 16], usize) {
    let mut names = [""; 16];
    for (slot, (name, _)) in names.iter_mut().zip(fields) {
        *slot = name;
    }
    (names, fields.len().min(16))
}

#[inline(never)]
/// Reads the typed document. With a `journal_budget`, a `journal` value is
/// first counted like the staged preflight and refused if it exceeds the
/// budget, before anything of it is materialized.
fn read_document(
    reader: &mut Reader<'_>,
    version: u32,
    journal_budget: Option<usize>,
) -> Res<ParsedDocument> {
    let fields = match version {
        LEGACY_SINGLE_SOLID_VERSION => V1_FIELDS,
        LEGACY_MULTI_ROOT_VERSION => V2_FIELDS,
        FORMAT_VERSION => V3_FIELDS,
        SHEET_ROOT_FORMAT_VERSION => V4_FIELDS,
        _ => V5_FIELDS,
    };
    // Every version-1 field is required; later versions end with the
    // defaulted `journal` and `attributes`.
    let required_count = if version == LEGACY_SINGLE_SOLID_VERSION {
        fields.len()
    } else {
        fields.len() - 2
    };
    let (names, count) = names_of(fields);
    let mut values = DocumentFields::default();
    reader.read_struct(
        names.get(..count).unwrap_or_default(),
        (1 << required_count) - 1,
        required_count,
        &mut |reader, index| {
            let Some(&(_, field)) = fields.get(index) else {
                return Err(reader.error("unknown field"));
            };
            match field {
                DocField::Version => values.version = Some(reader.read_u32()?),
                DocField::Vertices => values.vertices = Some(reader.read_vec(read_vertex)?),
                DocField::Edges => values.edges = Some(reader.read_vec(read_edge)?),
                DocField::Wires => values.wires = Some(reader.read_vec(read_wire)?),
                DocField::Faces => values.faces = Some(reader.read_vec(read_face)?),
                DocField::Shells => values.shells = Some(reader.read_vec(read_shell)?),
                DocField::OuterShell => values.outer_shell = Some(reader.read_usize()?),
                DocField::InnerShells => {
                    values.inner_shells = Some(reader.read_vec(Reader::read_usize)?);
                }
                DocField::Solids => values.solids = Some(reader.read_vec(read_solid)?),
                DocField::SolidRoots => {
                    values.solid_roots = Some(reader.read_vec(Reader::read_usize)?);
                }
                DocField::SheetRoots => {
                    values.sheet_roots = Some(reader.read_vec(Reader::read_usize)?);
                }
                DocField::WireRoots => {
                    values.wire_roots = Some(reader.read_vec(Reader::read_usize)?);
                }
                DocField::Compounds => values.compounds = Some(reader.read_vec(read_compound)?),
                DocField::PCurves => values.pcurves = Some(reader.read_vec(read_pcurve)?),
                DocField::Boundary => values.boundary = Some(read_boundary(reader)?),
                // Repeats are refused by `read_struct`, so `null` and an
                // absent field both leave `None`.
                DocField::Journal => {
                    if let Some(limit) = journal_budget {
                        let start = reader.pos();
                        let mut used = 0;
                        if count_journal_items(reader, 2, &mut used, limit).is_err() {
                            return Err(reader.error("journal over budget"));
                        }
                        reader.seek(start);
                    }
                    if !reader.null()? {
                        values.journal = Some(read_journal(reader)?);
                    }
                }
                DocField::Attributes => {
                    if !reader.null()? {
                        values.attributes = Some(read_attributes(reader)?);
                    }
                }
            }
            Ok(())
        },
    )?;
    let missing = || reader.error("missing field");
    let vertices = values.vertices.ok_or_else(missing)?;
    let edges = values.edges.ok_or_else(missing)?;
    let wires = values.wires.ok_or_else(missing)?;
    let faces = values.faces.ok_or_else(missing)?;
    let shells = values.shells.ok_or_else(missing)?;
    if version == LEGACY_SINGLE_SOLID_VERSION {
        let reported = values.version.ok_or_else(missing)?;
        if reported != LEGACY_SINGLE_SOLID_VERSION {
            return Err(reader.error("arena v1 document reported another version"));
        }
        return Ok(ParsedDocument {
            vertices,
            edges,
            wires,
            faces,
            shells,
            solids: vec![SerSolid {
                outer_shell: values.outer_shell.ok_or_else(missing)?,
                inner_shells: values.inner_shells.ok_or_else(missing)?,
            }],
            solid_roots: vec![0],
            sheet_roots: Vec::new(),
            wire_roots: Vec::new(),
            compounds: Vec::new(),
            pcurves: values.pcurves.ok_or_else(missing)?,
            boundary_authority: None,
            journal: None,
            attributes: None,
        });
    }
    let boundary_authority = if version == LEGACY_MULTI_ROOT_VERSION {
        None
    } else {
        Some(values.boundary.ok_or_else(missing)?)
    };
    Ok(ParsedDocument {
        vertices,
        edges,
        wires,
        faces,
        shells,
        solids: values.solids.ok_or_else(missing)?,
        solid_roots: values.solid_roots.ok_or_else(missing)?,
        sheet_roots: values.sheet_roots.unwrap_or_default(),
        wire_roots: values.wire_roots.unwrap_or_default(),
        compounds: values.compounds.ok_or_else(missing)?,
        pcurves: values.pcurves.unwrap_or_default(),
        boundary_authority,
        journal: values.journal,
        attributes: values.attributes,
    })
}

#[inline(never)]
fn read_point3(reader: &mut Reader<'_>) -> Res<Point3> {
    reader.read_f64s::<3>().map(Position)
}

#[inline(never)]
fn read_vertex(reader: &mut Reader<'_>) -> Res<SerVertex> {
    let (mut point, mut tolerance) = (None, None);
    reader.read_struct(&["point", "tolerance"], 0b11, 2, &mut |reader, index| {
        if index == 0 {
            point = Some(read_point3(reader)?);
        } else {
            tolerance = Some(reader.read_f64()?);
        }
        Ok(())
    })?;
    match (point, tolerance) {
        (Some(point), Some(tolerance)) => Ok(SerVertex { point, tolerance }),
        _ => Err(reader.error("missing field")),
    }
}

fn read_optional_f64(reader: &mut Reader<'_>) -> Res<Option<f64>> {
    if reader.null()? {
        Ok(None)
    } else {
        reader.read_f64().map(Some)
    }
}

#[inline(never)]
fn read_edge(reader: &mut Reader<'_>) -> Res<SerEdge> {
    let (mut start, mut end, mut curve, mut tolerance, mut trim) = (None, None, None, None, None);
    reader.read_struct(
        &["start", "end", "curve", "tolerance", "trim"],
        0b111,
        4,
        &mut |reader, index| {
            match index {
                0 => start = Some(reader.read_usize()?),
                1 => end = Some(reader.read_usize()?),
                2 => curve = Some(read_edge_curve(reader)?),
                3 => tolerance = Some(read_optional_f64(reader)?),
                _ => {
                    trim = Some(if reader.null()? {
                        None
                    } else {
                        let [a, b] = reader.read_f64s::<2>()?;
                        Some((a, b))
                    });
                }
            }
            Ok(())
        },
    )?;
    match (start, end, curve) {
        (Some(start), Some(end), Some(curve)) => Ok(SerEdge {
            start,
            end,
            curve,
            tolerance: tolerance.flatten(),
            trim: trim.flatten(),
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_oriented_edge(reader: &mut Reader<'_>) -> Res<SerOrientedEdge> {
    let (mut edge, mut forward) = (None, None);
    reader.read_struct(&["edge", "forward"], 0b11, 2, &mut |reader, index| {
        if index == 0 {
            edge = Some(reader.read_usize()?);
        } else {
            forward = Some(reader.read_bool()?);
        }
        Ok(())
    })?;
    match (edge, forward) {
        (Some(edge), Some(forward)) => Ok(SerOrientedEdge { edge, forward }),
        _ => Err(reader.error("missing field")),
    }
}

const BODY_CLASSES: &[&str] = &["solid", "sheet", "wire", "general"];

#[inline(never)]
fn read_body_class(reader: &mut Reader<'_>) -> Res<Option<SerBodyClass>> {
    if reader.null()? {
        return Ok(None);
    }
    let (index, object) = reader.read_variant(BODY_CLASSES)?;
    reader.unit_variant(object)?;
    Ok(Some(match index {
        0 => SerBodyClass::Solid,
        1 => SerBodyClass::Sheet,
        2 => SerBodyClass::Wire,
        _ => SerBodyClass::General,
    }))
}

#[inline(never)]
fn read_wire(reader: &mut Reader<'_>) -> Res<SerWire> {
    let (mut edges, mut closed, mut body_class) = (None, None, None);
    reader.read_struct(
        &["edges", "closed", "body_class"],
        0b11,
        2,
        &mut |reader, index| {
            match index {
                0 => edges = Some(reader.read_vec(read_oriented_edge)?),
                1 => closed = Some(reader.read_bool()?),
                _ => body_class = Some(read_body_class(reader)?),
            }
            Ok(())
        },
    )?;
    match (edges, closed) {
        (Some(edges), Some(closed)) => Ok(SerWire {
            edges,
            closed,
            body_class: body_class.flatten(),
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_face(reader: &mut Reader<'_>) -> Res<SerFace> {
    let (mut outer_wire, mut inner_wires, mut surface, mut reversed) = (None, None, None, None);
    reader.read_struct(
        &["outer_wire", "inner_wires", "surface", "reversed"],
        0b1111,
        4,
        &mut |reader, index| {
            match index {
                0 => outer_wire = Some(reader.read_usize()?),
                1 => inner_wires = Some(reader.read_vec(Reader::read_usize)?),
                2 => surface = Some(read_face_surface(reader)?),
                _ => reversed = Some(reader.read_bool()?),
            }
            Ok(())
        },
    )?;
    match (outer_wire, inner_wires, surface, reversed) {
        (Some(outer_wire), Some(inner_wires), Some(surface), Some(reversed)) => Ok(SerFace {
            outer_wire,
            inner_wires,
            surface,
            reversed,
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_shell(reader: &mut Reader<'_>) -> Res<SerShell> {
    let (mut faces, mut body_class) = (None, None);
    reader.read_struct(&["faces", "body_class"], 0b1, 1, &mut |reader, index| {
        if index == 0 {
            faces = Some(reader.read_vec(Reader::read_usize)?);
        } else {
            body_class = Some(read_body_class(reader)?);
        }
        Ok(())
    })?;
    match faces {
        Some(faces) => Ok(SerShell {
            faces,
            body_class: body_class.flatten(),
        }),
        None => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_solid(reader: &mut Reader<'_>) -> Res<SerSolid> {
    let (mut outer_shell, mut inner_shells) = (None, None);
    reader.read_struct(
        &["outer_shell", "inner_shells"],
        0b11,
        2,
        &mut |reader, index| {
            if index == 0 {
                outer_shell = Some(reader.read_usize()?);
            } else {
                inner_shells = Some(reader.read_vec(Reader::read_usize)?);
            }
            Ok(())
        },
    )?;
    match (outer_shell, inner_shells) {
        (Some(outer_shell), Some(inner_shells)) => Ok(SerSolid {
            outer_shell,
            inner_shells,
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_compound(reader: &mut Reader<'_>) -> Res<SerCompound> {
    let mut solids = None;
    reader.read_struct(&["solids"], 0b1, 1, &mut |reader, _| {
        solids = Some(reader.read_vec(Reader::read_usize)?);
        Ok(())
    })?;
    solids
        .map(|solids| SerCompound { solids })
        .ok_or_else(|| reader.error("missing field"))
}

#[inline(never)]
fn read_pcurve(reader: &mut Reader<'_>) -> Res<SerPCurve> {
    let (mut edge, mut face, mut curve, mut t_start, mut t_end, mut forward) =
        (None, None, None, None, None, None);
    reader.read_struct(
        &["edge", "face", "curve", "t_start", "t_end", "forward"],
        0b11111,
        5,
        &mut |reader, index| {
            match index {
                0 => edge = Some(reader.read_usize()?),
                1 => face = Some(reader.read_usize()?),
                2 => curve = Some(read_curve2d(reader)?),
                3 => t_start = Some(reader.read_f64()?),
                4 => t_end = Some(reader.read_f64()?),
                _ => {
                    forward = Some(if reader.null()? {
                        None
                    } else {
                        Some(reader.read_bool()?)
                    });
                }
            }
            Ok(())
        },
    )?;
    match (edge, face, curve, t_start, t_end) {
        (Some(edge), Some(face), Some(curve), Some(t_start), Some(t_end)) => Ok(SerPCurve {
            edge,
            face,
            curve,
            t_start,
            t_end,
            forward: forward.flatten(),
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_use_pcurve(reader: &mut Reader<'_>) -> Res<SerUsePCurve> {
    let (mut curve, mut t_start, mut t_end) = (None, None, None);
    reader.read_struct(
        &["curve", "t_start", "t_end"],
        0b111,
        3,
        &mut |reader, index| {
            match index {
                0 => curve = Some(read_curve2d(reader)?),
                1 => t_start = Some(reader.read_f64()?),
                _ => t_end = Some(reader.read_f64()?),
            }
            Ok(())
        },
    )?;
    match (curve, t_start, t_end) {
        (Some(curve), Some(t_start), Some(t_end)) => Ok(SerUsePCurve {
            curve,
            t_start,
            t_end,
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_coedge(reader: &mut Reader<'_>) -> Res<SerCoedgeAuthority> {
    let (mut edge, mut forward, mut winding, mut pcurve) = (None, None, None, None);
    reader.read_struct(
        &["edge", "forward", "periodic_winding", "pcurve"],
        0b111,
        3,
        &mut |reader, index| {
            match index {
                0 => edge = Some(reader.read_usize()?),
                1 => forward = Some(reader.read_bool()?),
                2 => {
                    reader.begin_array()?;
                    let mut values = [0_i32; 2];
                    let mut first = true;
                    for value in &mut values {
                        if !reader.next_element(&mut first)? {
                            return Err(reader.error("invalid length"));
                        }
                        *value = reader.read_i32()?;
                    }
                    if reader.next_element(&mut first)? {
                        return Err(reader.error("trailing characters"));
                    }
                    winding = Some(values);
                }
                _ => {
                    pcurve = Some(if reader.null()? {
                        None
                    } else {
                        Some(read_use_pcurve(reader)?)
                    });
                }
            }
            Ok(())
        },
    )?;
    match (edge, forward, winding) {
        (Some(edge), Some(forward), Some(periodic_winding)) => Ok(SerCoedgeAuthority {
            edge,
            forward,
            periodic_winding,
            pcurve: pcurve.flatten(),
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_loop(reader: &mut Reader<'_>) -> Res<SerLoopAuthority> {
    let (mut coedges, mut closed) = (None, None);
    reader.read_struct(&["coedges", "closed"], 0b11, 2, &mut |reader, index| {
        if index == 0 {
            coedges = Some(reader.read_vec(Reader::read_usize)?);
        } else {
            closed = Some(reader.read_bool()?);
        }
        Ok(())
    })?;
    match (coedges, closed) {
        (Some(coedges), Some(closed)) => Ok(SerLoopAuthority { coedges, closed }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_face_loop(reader: &mut Reader<'_>) -> Res<SerFaceLoopAuthority> {
    let (mut outer_loop, mut inner_loops) = (None, None);
    reader.read_struct(
        &["outer_loop", "inner_loops"],
        0b11,
        2,
        &mut |reader, index| {
            if index == 0 {
                outer_loop = Some(reader.read_usize()?);
            } else {
                inner_loops = Some(reader.read_vec(Reader::read_usize)?);
            }
            Ok(())
        },
    )?;
    match (outer_loop, inner_loops) {
        (Some(outer_loop), Some(inner_loops)) => Ok(SerFaceLoopAuthority {
            outer_loop,
            inner_loops,
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_boundary(reader: &mut Reader<'_>) -> Res<SerBoundaryAuthority> {
    let (mut loops, mut coedges, mut faces) = (None, None, None);
    reader.read_struct(
        &["loops", "coedges", "faces"],
        0b111,
        3,
        &mut |reader, index| {
            match index {
                0 => loops = Some(reader.read_vec(read_loop)?),
                1 => coedges = Some(reader.read_vec(read_coedge)?),
                _ => faces = Some(reader.read_vec(read_face_loop)?),
            }
            Ok(())
        },
    )?;
    match (loops, coedges, faces) {
        (Some(loops), Some(coedges), Some(faces)) => Ok(SerBoundaryAuthority {
            loops,
            coedges,
            faces,
        }),
        _ => Err(reader.error("missing field")),
    }
}

// ---------------------------------------------------------------------------
// Geometry carriers
// ---------------------------------------------------------------------------

/// A decoded field of a math carrier.
enum Part {
    F64(f64),
    Usize(usize),
    Coords3([f64; 3]),
    Coords2([f64; 2]),
    F64s(Vec<f64>),
    Points3(Vec<Point3>),
    Points2(Vec<Point2>),
    Grid3(Vec<Vec<Point3>>),
    Grid(Vec<Vec<f64>>),
}

impl Part {
    fn shape(&self) -> Shape<'_> {
        match self {
            Self::F64(value) => Shape::F64(*value),
            Self::Usize(value) => Shape::Usize(*value),
            Self::Coords3(coords) => Shape::F64s(coords),
            Self::Coords2(coords) => Shape::F64s(coords),
            Self::F64s(values) => Shape::F64s(values),
            Self::Points3(points) => Shape::Points3(points),
            Self::Points2(points) => Shape::Points2(points),
            Self::Grid3(rows) => Shape::Grid3(rows),
            Self::Grid(rows) => Shape::Grid(rows),
        }
    }
}

/// The JSON shape of a math carrier field.
#[derive(Clone, Copy)]
enum Kind {
    F64,
    Usize,
    Coords3,
    Coords2,
    F64s,
    Points3,
    Points2,
    Grid3,
    Grid,
}

/// A borrowed view of decoded carrier data, replayed into a carrier's
/// derived `Deserialize` impl: a number, or a sequence whose elements are
/// again shapes. One deserializer type keeps the replay's code small.
#[derive(Clone, Copy)]
enum Shape<'s> {
    F64(f64),
    Usize(usize),
    /// Floats: a point, a vector, or a `Vec<f64>`.
    F64s(&'s [f64]),
    Points3(&'s [Point3]),
    Points2(&'s [Point2]),
    Grid3(&'s [Vec<Point3>]),
    Grid(&'s [Vec<f64>]),
    /// The carrier's fields in declaration order.
    Fields(&'s [Self]),
}

/// Error of the [`Shape`] replay; the decoded data always has the shape the
/// derived impls expect, so it is never produced in practice.
#[derive(Debug)]
struct ShapeError;

impl std::fmt::Display for ShapeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("geometry carrier does not match its schema")
    }
}

impl std::error::Error for ShapeError {}

impl serde::de::Error for ShapeError {
    fn custom<T: std::fmt::Display>(_message: T) -> Self {
        Self
    }
}

impl<'de> serde::Deserializer<'de> for Shape<'_> {
    type Error = ShapeError;

    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, ShapeError> {
        match self {
            Self::F64(value) => visitor.visit_f64(value),
            Self::Usize(value) => visitor.visit_u64(value as u64),
            sequence => visitor.visit_seq(ShapeSeq {
                shape: sequence,
                index: 0,
            }),
        }
    }

    serde::forward_to_deserialize_any! {
        bool i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 f32 f64 char str string
        bytes byte_buf option unit unit_struct newtype_struct seq tuple
        tuple_struct map struct enum identifier ignored_any
    }
}

struct ShapeSeq<'s> {
    shape: Shape<'s>,
    index: usize,
}

impl<'de> SeqAccess<'de> for ShapeSeq<'_> {
    type Error = ShapeError;

    /// The exact remaining length, so the derived `Vec` impls allocate once.
    fn size_hint(&self) -> Option<usize> {
        let len = match self.shape {
            Shape::F64s(values) => values.len(),
            Shape::Points3(points) => points.len(),
            Shape::Points2(points) => points.len(),
            Shape::Grid3(rows) => rows.len(),
            Shape::Grid(rows) => rows.len(),
            Shape::Fields(fields) => fields.len(),
            Shape::F64(_) | Shape::Usize(_) => 0,
        };
        Some(len.saturating_sub(self.index))
    }

    fn next_element_seed<T: DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, ShapeError> {
        let index = self.index;
        let element = match self.shape {
            Shape::F64s(values) => values.get(index).map(|value| Shape::F64(*value)),
            Shape::Points3(points) => points.get(index).map(|point| Shape::F64s(&point.0)),
            Shape::Points2(points) => points.get(index).map(|point| Shape::F64s(&point.0)),
            Shape::Grid3(rows) => rows.get(index).map(|row| Shape::Points3(row)),
            Shape::Grid(rows) => rows.get(index).map(|row| Shape::F64s(row)),
            Shape::Fields(fields) => fields.get(index).copied(),
            Shape::F64(_) | Shape::Usize(_) => None,
        };
        match element {
            Some(element) => {
                self.index += 1;
                seed.deserialize(element).map(Some)
            }
            None => Ok(None),
        }
    }
}

#[inline(never)]
fn read_part(reader: &mut Reader<'_>, kind: Kind) -> Res<Part> {
    Ok(match kind {
        Kind::F64 => Part::F64(reader.read_f64()?),
        Kind::Usize => Part::Usize(reader.read_usize()?),
        Kind::Coords3 => Part::Coords3(reader.read_f64s::<3>()?),
        Kind::Coords2 => Part::Coords2(reader.read_f64s::<2>()?),
        Kind::F64s => Part::F64s(reader.read_f64_vec()?),
        Kind::Points3 => Part::Points3(reader.read_vec(read_point3)?),
        Kind::Points2 => {
            Part::Points2(reader.read_vec(|reader| reader.read_f64s::<2>().map(Position))?)
        }
        Kind::Grid3 => Part::Grid3(reader.read_vec(|reader| reader.read_vec(read_point3))?),
        Kind::Grid => Part::Grid(reader.read_vec(Reader::read_f64_vec)?),
    })
}

/// Reads the fields of a math carrier (every one required) in declaration
/// order.
#[inline(never)]
fn read_parts(reader: &mut Reader<'_>, schema: &[(&'static str, Kind)]) -> Res<Vec<Part>> {
    let (names, count) = names_of(schema);
    let mut parts: [Option<Part>; 8] = Default::default();
    reader.read_struct(
        names.get(..count).unwrap_or_default(),
        (1 << count) - 1,
        count,
        &mut |reader, index| {
            let (Some(&(_, kind)), Some(slot)) = (schema.get(index), parts.get_mut(index)) else {
                return Err(reader.error("unknown field"));
            };
            *slot = Some(read_part(reader, kind)?);
            Ok(())
        },
    )?;
    parts
        .into_iter()
        .take(count)
        .map(|slot| slot.ok_or_else(|| reader.error("missing field")))
        .collect()
}

/// Reads the fields of a math carrier and hands them, as one
/// [`Shape::Fields`], to `build`. Shared by every carrier type.
#[inline(never)]
fn with_carrier(
    reader: &mut Reader<'_>,
    schema: &[(&'static str, Kind)],
    build: &mut dyn FnMut(Shape<'_>) -> bool,
) -> Res<()> {
    let parts = read_parts(reader, schema)?;
    let mut shapes = [Shape::F64(0.0); 8];
    for (shape, part) in shapes.iter_mut().zip(&parts) {
        *shape = part.shape();
    }
    let fields = shapes.get(..parts.len()).unwrap_or_default();
    if build(Shape::Fields(fields)) {
        Ok(())
    } else {
        Err(reader.error("invalid geometry carrier"))
    }
}

/// Reads a math carrier and builds it through its derived `Deserialize`
/// impl, which populates the private fields directly with no validation.
fn read_carrier<T: DeserializeOwned>(
    reader: &mut Reader<'_>,
    schema: &[(&'static str, Kind)],
) -> Res<T> {
    let mut value = None;
    with_carrier(reader, schema, &mut |fields| {
        value = T::deserialize(fields).ok();
        value.is_some()
    })?;
    value.ok_or_else(|| reader.error("invalid geometry carrier"))
}

const CIRCLE3: &[(&str, Kind)] = &[
    ("center", Kind::Coords3),
    ("normal", Kind::Coords3),
    ("radius", Kind::F64),
    ("u_axis", Kind::Coords3),
    ("v_axis", Kind::Coords3),
];
const CONIC3: &[(&str, Kind)] = &[
    ("center", Kind::Coords3),
    ("normal", Kind::Coords3),
    ("semi_major", Kind::F64),
    ("semi_minor", Kind::F64),
    ("u_axis", Kind::Coords3),
    ("v_axis", Kind::Coords3),
];
const PARABOLA3: &[(&str, Kind)] = &[
    ("vertex", Kind::Coords3),
    ("axis_dir", Kind::Coords3),
    ("focal_length", Kind::F64),
    ("u_axis", Kind::Coords3),
];
const NURBS_CURVE: &[(&str, Kind)] = &[
    ("degree", Kind::Usize),
    ("knots", Kind::F64s),
    ("control_points", Kind::Points3),
    ("weights", Kind::F64s),
];
const NURBS_SURFACE: &[(&str, Kind)] = &[
    ("degree_u", Kind::Usize),
    ("degree_v", Kind::Usize),
    ("knots_u", Kind::F64s),
    ("knots_v", Kind::F64s),
    ("control_points", Kind::Grid3),
    ("weights", Kind::Grid),
];
const CYLINDER: &[(&str, Kind)] = &[
    ("origin", Kind::Coords3),
    ("axis", Kind::Coords3),
    ("radius", Kind::F64),
    ("x_axis", Kind::Coords3),
    ("y_axis", Kind::Coords3),
];
const CONE: &[(&str, Kind)] = &[
    ("apex", Kind::Coords3),
    ("axis", Kind::Coords3),
    ("half_angle", Kind::F64),
    ("x_axis", Kind::Coords3),
    ("y_axis", Kind::Coords3),
];
const SPHERE: &[(&str, Kind)] = &[
    ("center", Kind::Coords3),
    ("radius", Kind::F64),
    ("x_axis", Kind::Coords3),
    ("y_axis", Kind::Coords3),
    ("z_axis", Kind::Coords3),
];
const TORUS: &[(&str, Kind)] = &[
    ("center", Kind::Coords3),
    ("major_radius", Kind::F64),
    ("minor_radius", Kind::F64),
    ("x_axis", Kind::Coords3),
    ("y_axis", Kind::Coords3),
    ("z_axis", Kind::Coords3),
];
const LINE2: &[(&str, Kind)] = &[("origin", Kind::Coords2), ("direction", Kind::Coords2)];
const CIRCLE2: &[(&str, Kind)] = &[("center", Kind::Coords2), ("radius", Kind::F64)];
const ELLIPSE2: &[(&str, Kind)] = &[
    ("center", Kind::Coords2),
    ("semi_major", Kind::F64),
    ("semi_minor", Kind::F64),
    ("rotation", Kind::F64),
];
const NURBS2: &[(&str, Kind)] = &[
    ("degree", Kind::Usize),
    ("knots", Kind::F64s),
    ("control_points", Kind::Points2),
    ("weights", Kind::F64s),
];

const EDGE_CURVES: &[&str] = &[
    "Line",
    "NurbsCurve",
    "Circle",
    "Ellipse",
    "Hyperbola",
    "Parabola",
];

#[inline(never)]
fn read_edge_curve(reader: &mut Reader<'_>) -> Res<SerEdgeCurve> {
    let (index, object) = reader.read_variant(EDGE_CURVES)?;
    if index == 0 {
        reader.unit_variant(object)?;
        return Ok(SerEdgeCurve::Line);
    }
    reader.data_variant(object)?;
    let curve = match index {
        1 => SerEdgeCurve::NurbsCurve(read_carrier::<NurbsCurve>(reader, NURBS_CURVE)?),
        2 => SerEdgeCurve::Circle(read_carrier::<Circle3D>(reader, CIRCLE3)?),
        3 => SerEdgeCurve::Ellipse(read_carrier::<Ellipse3D>(reader, CONIC3)?),
        4 => SerEdgeCurve::Hyperbola(read_carrier::<Hyperbola3D>(reader, CONIC3)?),
        _ => SerEdgeCurve::Parabola(read_carrier::<Parabola3D>(reader, PARABOLA3)?),
    };
    reader.end_variant()?;
    Ok(curve)
}

const FACE_SURFACES: &[&str] = &["Plane", "Nurbs", "Cylinder", "Cone", "Sphere", "Torus"];

#[inline(never)]
fn read_face_surface(reader: &mut Reader<'_>) -> Res<SerFaceSurface> {
    let (index, object) = reader.read_variant(FACE_SURFACES)?;
    reader.data_variant(object)?;
    let surface = match index {
        0 => {
            let (mut normal, mut d) = (None, None);
            reader.read_struct(&["normal", "d"], 0b11, 2, &mut |reader, index| {
                if index == 0 {
                    normal = Some(Vector(reader.read_f64s::<3>()?));
                } else {
                    d = Some(reader.read_f64()?);
                }
                Ok(())
            })?;
            match (normal, d) {
                (Some(normal), Some(d)) => SerFaceSurface::Plane { normal, d },
                _ => return Err(reader.error("missing field")),
            }
        }
        1 => SerFaceSurface::Nurbs(read_carrier::<NurbsSurface>(reader, NURBS_SURFACE)?),
        2 => SerFaceSurface::Cylinder(read_carrier::<CylindricalSurface>(reader, CYLINDER)?),
        3 => SerFaceSurface::Cone(read_carrier::<ConicalSurface>(reader, CONE)?),
        4 => SerFaceSurface::Sphere(read_carrier::<SphericalSurface>(reader, SPHERE)?),
        _ => SerFaceSurface::Torus(read_carrier::<ToroidalSurface>(reader, TORUS)?),
    };
    reader.end_variant()?;
    Ok(surface)
}

const CURVES_2D: &[&str] = &["Line", "Circle", "Ellipse", "Nurbs"];

#[inline(never)]
fn read_curve2d(reader: &mut Reader<'_>) -> Res<Curve2D> {
    let (index, object) = reader.read_variant(CURVES_2D)?;
    reader.data_variant(object)?;
    let curve = match index {
        0 => Curve2D::Line(read_carrier(reader, LINE2)?),
        1 => Curve2D::Circle(read_carrier(reader, CIRCLE2)?),
        2 => Curve2D::Ellipse(read_carrier(reader, ELLIPSE2)?),
        _ => Curve2D::Nurbs(read_carrier(reader, NURBS2)?),
    };
    reader.end_variant()?;
    Ok(curve)
}

// ---------------------------------------------------------------------------
// Journal and attributes
// ---------------------------------------------------------------------------

#[inline(never)]
fn read_journal(reader: &mut Reader<'_>) -> Res<SerJournal> {
    let (mut next_op, mut next_ordinal, mut index, mut entries) = (None, None, None, None);
    reader.read_struct(
        &["next_op", "next_ordinal", "index", "entries"],
        0b1111,
        4,
        &mut |reader, field| {
            match field {
                0 => next_op = Some(reader.read_u64()?),
                1 => next_ordinal = Some(reader.read_u64()?),
                2 => index = Some(reader.read_vec(read_index_entry)?),
                _ => entries = Some(reader.read_vec(read_journal_entry)?),
            }
            Ok(())
        },
    )?;
    match (next_op, next_ordinal, index, entries) {
        (Some(next_op), Some(next_ordinal), Some(index), Some(entries)) => Ok(SerJournal {
            next_op,
            next_ordinal,
            index,
            entries,
        }),
        _ => Err(reader.error("missing field")),
    }
}

#[inline(never)]
fn read_index_entry(reader: &mut Reader<'_>) -> Res<SerJournalIndexEntry> {
    let (mut ordinal, mut kind, mut local) = (None, None, None);
    reader.read_struct(
        &["ordinal", "kind", "local"],
        0b11,
        3,
        &mut |reader, index| {
            match index {
                0 => ordinal = Some(reader.read_u64()?),
                1 => kind = Some(reader.read_string()?.into_owned()),
                _ => {
                    local = Some(if reader.null()? {
                        None
                    } else {
                        Some(reader.read_usize()?)
                    });
                }
            }
            Ok(())
        },
    )?;
    match (ordinal, kind) {
        (Some(ordinal), Some(kind)) => Ok(SerJournalIndexEntry {
            ordinal,
            kind,
            local: local.flatten(),
        }),
        _ => Err(reader.error("missing field")),
    }
}

/// Nesting level of a journal entry's buffered field values: document,
/// journal, entry list, entry, value.
const ENTRY_VALUE_DEPTH: usize = 5;

/// Reads a journal entry. Its payload is `#[serde(flatten)]`ed, so the
/// entry is an object only: `op` and `kind` are read directly and every
/// other field is buffered as [`Content`] for the internally tagged payload.
#[inline(never)]
fn read_journal_entry(reader: &mut Reader<'_>) -> Res<SerJournalEntry> {
    reader.begin_object()?;
    let (mut op, mut kind) = (None, None);
    let mut rest = Vec::new();
    let mut first = true;
    while let Some(key) = reader.next_key(&mut first)? {
        match key.as_ref() {
            "op" => {
                if op.is_some() {
                    return Err(reader.error("duplicate field `op`"));
                }
                op = Some(reader.read_u64()?);
            }
            "kind" => {
                if kind.is_some() {
                    return Err(reader.error("duplicate field `kind`"));
                }
                kind = Some(reader.read_string()?.into_owned());
            }
            _ => {
                let value = reader.read_content(ENTRY_VALUE_DEPTH)?;
                rest.push((key, value));
            }
        }
    }
    let error = reader.error("invalid journal entry");
    let (Some(op), Some(kind)) = (op, kind) else {
        return Err(error);
    };
    let payload = payload_from_content(&rest).ok_or(error)?;
    Ok(SerJournalEntry { op, kind, payload })
}

/// A buffered object member.
type Entry<'a> = (std::borrow::Cow<'a, str>, Content<'a>);

/// Splits buffered entries into an internally tagged enum's tag (a name or,
/// as `serde` allows for buffered identifiers, a variant index) and the
/// remaining fields. A repeated or missing tag is refused.
fn split_tag<'c, 'a>(
    entries: impl IntoIterator<Item = &'c Entry<'a>>,
    tag: &str,
    variants: &[&str],
) -> Option<(usize, Vec<&'c Entry<'a>>)>
where
    'a: 'c,
{
    let mut variant = None;
    let mut rest = Vec::new();
    for entry in entries {
        if entry.0 == tag {
            if variant.is_some() {
                return None;
            }
            variant = Some(entry.1.variant(variants)?);
        } else {
            rest.push(entry);
        }
    }
    Some((variant?, rest))
}

/// The value of the one field of a buffered struct variant: unknown keys are
/// ignored, a repeated or missing field is refused.
fn single_field<'c, 'a: 'c>(entries: &[&'c Entry<'a>], name: &str) -> Option<&'c Content<'a>> {
    let mut found = None;
    for (key, value) in entries.iter().map(|entry| (&entry.0, &entry.1)) {
        if key == name {
            if found.is_some() {
                return None;
            }
            found = Some(value);
        }
    }
    found
}

fn content_u64s(content: &Content<'_>) -> Option<Vec<u64>> {
    match content {
        Content::Seq(items) => items.iter().map(Content::as_u64).collect(),
        _ => None,
    }
}

const PAYLOADS: &[&str] = &["Evolution", "Barrier", "GlobalBarrier"];

#[inline(never)]
fn payload_from_content(entries: &[Entry<'_>]) -> Option<SerJournalPayload> {
    let (variant, rest) = split_tag(entries, "payload", PAYLOADS)?;
    match variant {
        0 => {
            let (mut construction, mut scope, mut events) = (None, None, None);
            for (key, value) in rest.iter().map(|entry| (&entry.0, &entry.1)) {
                match key.as_ref() {
                    "construction" => {
                        if construction.is_some() {
                            return None;
                        }
                        construction = Some(match value {
                            Content::Bool(value) => *value,
                            _ => return None,
                        });
                    }
                    "scope" => {
                        if scope.is_some() {
                            return None;
                        }
                        scope = Some(content_u64s(value)?);
                    }
                    "events" => {
                        if events.is_some() {
                            return None;
                        }
                        let Content::Seq(items) = value else {
                            return None;
                        };
                        events = Some(
                            items
                                .iter()
                                .map(event_pair_from_content)
                                .collect::<Option<Vec<_>>>()?,
                        );
                    }
                    _ => {}
                }
            }
            Some(SerJournalPayload::Evolution {
                construction: construction?,
                scope: scope?,
                events: events?,
            })
        }
        1 => Some(SerJournalPayload::Barrier {
            affected: content_u64s(single_field(&rest, "affected")?)?,
        }),
        _ => Some(SerJournalPayload::GlobalBarrier),
    }
}

#[inline(never)]
fn event_pair_from_content(content: &Content<'_>) -> Option<(u64, SerJournalEvent)> {
    match content {
        Content::Seq(items) => match items.as_slice() {
            [subject, event] => Some((subject.as_u64()?, event_from_content(event)?)),
            _ => None,
        },
        _ => None,
    }
}

const EVENTS: &[&str] = &[
    "Preserved",
    "Modified",
    "Generated",
    "Merged",
    "Deleted",
    "Unresolved",
];

/// The one field of each event variant (`Deleted` has none).
const EVENT_FIELDS: &[&str] = &["from", "from", "sources", "from", "", "candidates"];

#[inline(never)]
fn event_from_content(content: &Content<'_>) -> Option<SerJournalEvent> {
    // The tagged variant's single field, or `None` for `Deleted`.
    let (variant, field) = match content {
        Content::Map(entries) => {
            let (variant, rest) = split_tag(entries, "event", EVENTS)?;
            let name = EVENT_FIELDS.get(variant)?;
            let field = if name.is_empty() {
                None
            } else {
                Some(single_field(&rest, name)?)
            };
            (variant, field)
        }
        // The tag leads, the variant's fields follow by position.
        Content::Seq(items) => {
            let (tag, rest) = items.split_first()?;
            let variant = tag.variant(EVENTS)?;
            let field = match (EVENT_FIELDS.get(variant)?.is_empty(), rest) {
                (true, []) => None,
                (false, [field]) => Some(field),
                _ => return None,
            };
            (variant, field)
        }
        _ => return None,
    };
    Some(match (variant, field) {
        (0, Some(from)) => SerJournalEvent::Preserved {
            from: from.as_u64()?,
        },
        (1, Some(from)) => SerJournalEvent::Modified {
            from: from.as_u64()?,
        },
        (2, Some(sources)) => SerJournalEvent::Generated {
            sources: content_u64s(sources)?,
        },
        (3, Some(from)) => SerJournalEvent::Merged {
            from: content_u64s(from)?,
        },
        (5, Some(candidates)) => SerJournalEvent::Unresolved {
            candidates: content_u64s(candidates)?,
        },
        (4, None) => SerJournalEvent::Deleted,
        _ => return None,
    })
}

#[inline(never)]
fn read_attributes(reader: &mut Reader<'_>) -> Res<SerAttributes> {
    let (mut solids, mut faces) = (None, None);
    reader.read_struct(&["solids", "faces"], 0, 0, &mut |reader, index| {
        let list = reader.read_vec(read_attribute_pair)?;
        if index == 0 {
            solids = Some(list);
        } else {
            faces = Some(list);
        }
        Ok(())
    })?;
    Ok(SerAttributes {
        solids: solids.unwrap_or_default(),
        faces: faces.unwrap_or_default(),
    })
}

#[inline(never)]
fn read_attribute_pair(reader: &mut Reader<'_>) -> Res<(usize, SerEntityAttributes)> {
    reader.begin_array()?;
    let mut first = true;
    if !reader.next_element(&mut first)? {
        return Err(reader.error("invalid length"));
    }
    let local = reader.read_usize()?;
    if !reader.next_element(&mut first)? {
        return Err(reader.error("invalid length"));
    }
    let (mut name, mut color) = (None, None);
    reader.read_struct(&["name", "color"], 0, 0, &mut |reader, index| {
        if reader.null()? {
            if index == 0 {
                name = Some(None);
            } else {
                color = Some(None);
            }
        } else if index == 0 {
            name = Some(Some(reader.read_string()?.into_owned()));
        } else {
            let [r, g, b] = reader.read_f64s::<3>()?;
            color = Some(Some((r, g, b)));
        }
        Ok(())
    })?;
    if reader.next_element(&mut first)? {
        return Err(reader.error("trailing characters"));
    }
    Ok((
        local,
        SerEntityAttributes {
            name: name.flatten(),
            color: color.flatten(),
        },
    ))
}

// ===========================================================================
// Encoding
// ===========================================================================

/// Borrowed parts of a version 3, 4 or 5 document, in field order.
pub(super) struct DocumentParts<'v, 'a> {
    pub(super) version: u32,
    pub(super) vertices: &'v [SerVertex],
    pub(super) edges: &'v [SerEdgeView<'a>],
    pub(super) wires: &'v [SerWire],
    pub(super) faces: &'v [SerFaceView<'a>],
    pub(super) shells: &'v [SerShell],
    pub(super) solids: &'v [SerSolid],
    pub(super) solid_roots: &'v [usize],
    pub(super) sheet_roots: Option<&'v [usize]>,
    pub(super) wire_roots: Option<&'v [usize]>,
    pub(super) compounds: &'v [SerCompound],
    pub(super) boundary_authority: &'v SerBoundaryAuthorityView<'a>,
    pub(super) journal: Option<&'v SerJournal>,
    pub(super) attributes: Option<&'v SerAttributes>,
}

/// Encodes a document exactly as `serde_json` encoded the derived views,
/// refusing once the output would exceed `limit` bytes.
pub(super) fn encode_document(
    document: &DocumentParts<'_, '_>,
    limit: Option<usize>,
) -> Result<Vec<u8>, IoError> {
    let mut w = Writer::new(limit);
    w.key_u64(b"{\"version\":", u64::from(document.version));
    w.raw(b",\"vertices\":");
    list(&mut w, document.vertices, write_vertex);
    w.raw(b",\"edges\":");
    list(&mut w, document.edges, write_edge);
    w.raw(b",\"wires\":");
    list(&mut w, document.wires, write_wire);
    w.raw(b",\"faces\":");
    list(&mut w, document.faces, write_face);
    w.raw(b",\"shells\":");
    list(&mut w, document.shells, write_shell);
    w.raw(b",\"solids\":");
    list(&mut w, document.solids, write_solid);
    w.key_usizes(b",\"solid_roots\":", document.solid_roots);
    if let Some(sheet_roots) = document.sheet_roots {
        w.key_usizes(b",\"sheet_roots\":", sheet_roots);
    }
    if let Some(wire_roots) = document.wire_roots {
        w.key_usizes(b",\"wire_roots\":", wire_roots);
    }
    w.raw(b",\"compounds\":");
    list(&mut w, document.compounds, |w, compound| {
        w.key_usizes(b"{\"solids\":", &compound.solids);
        w.byte(b'}');
    });
    w.raw(b",\"boundary_authority\":{\"loops\":");
    list(
        &mut w,
        &document.boundary_authority.loops,
        |w, boundary_loop| {
            w.key_usizes(b"{\"coedges\":", &boundary_loop.coedges);
            w.key_bool(b",\"closed\":", boundary_loop.closed);
            w.byte(b'}');
        },
    );
    w.raw(b",\"coedges\":");
    list(&mut w, &document.boundary_authority.coedges, write_coedge);
    w.raw(b",\"faces\":");
    list(&mut w, &document.boundary_authority.faces, |w, face| {
        w.key_usize(b"{\"outer_loop\":", face.outer_loop);
        w.key_usizes(b",\"inner_loops\":", &face.inner_loops);
        w.byte(b'}');
    });
    w.byte(b'}');
    if let Some(journal) = document.journal {
        write_journal(&mut w, journal);
    }
    if let Some(attributes) = document.attributes {
        write_attributes(&mut w, attributes);
    }
    w.byte(b'}');
    if w.exceeded() {
        let limit = limit.unwrap_or(usize::MAX);
        return Err(IoError::LimitExceeded {
            resource: "arena document bytes",
            limit,
            actual: limit.saturating_add(1),
        });
    }
    Ok(w.finish())
}

/// Writes `[item, ...]`, stopping early once the byte budget is spent.
fn list<T>(w: &mut Writer, items: &[T], mut item: impl FnMut(&mut Writer, &T)) {
    w.byte(b'[');
    for (index, value) in items.iter().enumerate() {
        if index > 0 {
            w.byte(b',');
        }
        item(w, value);
        if w.exceeded() {
            break;
        }
    }
    w.byte(b']');
}

/// Writes `[[x,y,z], ...]`.
#[inline(never)]
fn write_points<const N: usize>(w: &mut Writer, key: &[u8], points: &[Position<N>]) {
    w.raw(key);
    list(w, points, |w, point| w.f64s(&point.0));
}

#[inline(never)]
fn write_vertex(w: &mut Writer, vertex: &SerVertex) {
    w.key_f64s(b"{\"point\":", &vertex.point.0);
    w.key_f64(b",\"tolerance\":", vertex.tolerance);
    w.byte(b'}');
}

#[inline(never)]
fn write_edge(w: &mut Writer, edge: &SerEdgeView<'_>) {
    w.key_usize(b"{\"start\":", edge.start);
    w.key_usize(b",\"end\":", edge.end);
    w.raw(b",\"curve\":");
    write_edge_curve(w, edge.curve);
    match edge.tolerance {
        Some(tolerance) => w.key_f64(b",\"tolerance\":", tolerance),
        None => w.raw(b",\"tolerance\":null"),
    }
    if let Some((start, end)) = edge.trim {
        w.key_f64s(b",\"trim\":", &[start, end]);
    }
    w.byte(b'}');
}

#[inline(never)]
fn write_edge_curve(w: &mut Writer, curve: SerEdgeCurveView<'_>) {
    match curve {
        SerEdgeCurveView::Line => w.raw(b"\"Line\""),
        SerEdgeCurveView::NurbsCurve(curve) => {
            w.key_usize(b"{\"NurbsCurve\":{\"degree\":", curve.degree());
            w.key_f64s(b",\"knots\":", curve.knots());
            write_points(w, b",\"control_points\":", curve.control_points());
            w.key_f64s(b",\"weights\":", curve.weights());
            w.raw(b"}}");
        }
        SerEdgeCurveView::Circle(circle) => {
            w.key_f64s(b"{\"Circle\":{\"center\":", &circle.center().0);
            w.key_f64s(b",\"normal\":", &circle.normal().0);
            w.key_f64(b",\"radius\":", circle.radius());
            w.key_f64s(b",\"u_axis\":", &circle.u_axis().0);
            w.key_f64s(b",\"v_axis\":", &circle.v_axis().0);
            w.raw(b"}}");
        }
        SerEdgeCurveView::Ellipse(ellipse) => write_conic(
            w,
            b"{\"Ellipse\":{\"center\":",
            [ellipse.center().0, ellipse.normal().0],
            [ellipse.semi_major(), ellipse.semi_minor()],
            [ellipse.u_axis().0, ellipse.v_axis().0],
        ),
        SerEdgeCurveView::Hyperbola(hyperbola) => write_conic(
            w,
            b"{\"Hyperbola\":{\"center\":",
            [hyperbola.center().0, hyperbola.normal().0],
            [hyperbola.semi_major(), hyperbola.semi_minor()],
            [hyperbola.u_axis().0, hyperbola.v_axis().0],
        ),
        SerEdgeCurveView::Parabola(parabola) => {
            w.key_f64s(b"{\"Parabola\":{\"vertex\":", &parabola.vertex().0);
            w.key_f64s(b",\"axis_dir\":", &parabola.axis_dir().0);
            w.key_f64(b",\"focal_length\":", parabola.focal_length());
            w.key_f64s(b",\"u_axis\":", &parabola.u_axis().0);
            w.raw(b"}}");
        }
    }
}

/// Writes an ellipse or hyperbola: center, normal, semi axes, frame.
fn write_conic(
    w: &mut Writer,
    open: &[u8],
    center_normal: [[f64; 3]; 2],
    semi: [f64; 2],
    frame: [[f64; 3]; 2],
) {
    w.key_f64s(open, &center_normal[0]);
    w.key_f64s(b",\"normal\":", &center_normal[1]);
    w.key_f64(b",\"semi_major\":", semi[0]);
    w.key_f64(b",\"semi_minor\":", semi[1]);
    w.key_f64s(b",\"u_axis\":", &frame[0]);
    w.key_f64s(b",\"v_axis\":", &frame[1]);
    w.raw(b"}}");
}

fn write_body_class(w: &mut Writer, body_class: Option<SerBodyClass>) {
    if let Some(body_class) = body_class {
        w.raw(match body_class {
            SerBodyClass::Solid => b",\"body_class\":\"solid\"",
            SerBodyClass::Sheet => b",\"body_class\":\"sheet\"",
            SerBodyClass::Wire => b",\"body_class\":\"wire\"",
            SerBodyClass::General => b",\"body_class\":\"general\"",
        });
    }
}

#[inline(never)]
fn write_wire(w: &mut Writer, wire: &SerWire) {
    w.raw(b"{\"edges\":");
    list(w, &wire.edges, |w, oriented| {
        w.key_usize(b"{\"edge\":", oriented.edge);
        w.key_bool(b",\"forward\":", oriented.forward);
        w.byte(b'}');
    });
    w.key_bool(b",\"closed\":", wire.closed);
    write_body_class(w, wire.body_class);
    w.byte(b'}');
}

#[inline(never)]
fn write_face(w: &mut Writer, face: &SerFaceView<'_>) {
    w.key_usize(b"{\"outer_wire\":", face.outer_wire);
    w.key_usizes(b",\"inner_wires\":", &face.inner_wires);
    w.raw(b",\"surface\":");
    write_surface(w, face.surface);
    w.key_bool(b",\"reversed\":", face.reversed);
    w.byte(b'}');
}

#[inline(never)]
fn write_surface(w: &mut Writer, surface: SerFaceSurfaceView<'_>) {
    match surface {
        SerFaceSurfaceView::Plane { normal, d } => {
            w.key_f64s(b"{\"Plane\":{\"normal\":", &normal.0);
            w.key_f64(b",\"d\":", d);
        }
        SerFaceSurfaceView::Nurbs(surface) => {
            w.key_usize(b"{\"Nurbs\":{\"degree_u\":", surface.degree_u());
            w.key_usize(b",\"degree_v\":", surface.degree_v());
            w.key_f64s(b",\"knots_u\":", surface.knots_u());
            w.key_f64s(b",\"knots_v\":", surface.knots_v());
            w.raw(b",\"control_points\":");
            list(w, surface.control_points(), |w, row| {
                write_points(w, b"", row);
            });
            w.raw(b",\"weights\":");
            list(w, surface.weights(), |w, row| w.f64s(row));
        }
        SerFaceSurfaceView::Cylinder(cylinder) => {
            w.key_f64s(b"{\"Cylinder\":{\"origin\":", &cylinder.origin().0);
            w.key_f64s(b",\"axis\":", &cylinder.axis().0);
            w.key_f64(b",\"radius\":", cylinder.radius());
            w.key_f64s(b",\"x_axis\":", &cylinder.x_axis().0);
            w.key_f64s(b",\"y_axis\":", &cylinder.y_axis().0);
        }
        SerFaceSurfaceView::Cone(cone) => {
            w.key_f64s(b"{\"Cone\":{\"apex\":", &cone.apex().0);
            w.key_f64s(b",\"axis\":", &cone.axis().0);
            w.key_f64(b",\"half_angle\":", cone.half_angle());
            w.key_f64s(b",\"x_axis\":", &cone.x_axis().0);
            w.key_f64s(b",\"y_axis\":", &cone.y_axis().0);
        }
        SerFaceSurfaceView::Sphere(sphere) => {
            w.key_f64s(b"{\"Sphere\":{\"center\":", &sphere.center().0);
            w.key_f64(b",\"radius\":", sphere.radius());
            w.key_f64s(b",\"x_axis\":", &sphere.x_axis().0);
            w.key_f64s(b",\"y_axis\":", &sphere.y_axis().0);
            w.key_f64s(b",\"z_axis\":", &sphere.z_axis().0);
        }
        SerFaceSurfaceView::Torus(torus) => {
            w.key_f64s(b"{\"Torus\":{\"center\":", &torus.center().0);
            w.key_f64(b",\"major_radius\":", torus.major_radius());
            w.key_f64(b",\"minor_radius\":", torus.minor_radius());
            w.key_f64s(b",\"x_axis\":", &torus.x_axis().0);
            w.key_f64s(b",\"y_axis\":", &torus.y_axis().0);
            w.key_f64s(b",\"z_axis\":", &torus.z_axis().0);
        }
    }
    w.raw(b"}}");
}

#[inline(never)]
fn write_shell(w: &mut Writer, shell: &SerShell) {
    w.key_usizes(b"{\"faces\":", &shell.faces);
    write_body_class(w, shell.body_class);
    w.byte(b'}');
}

#[inline(never)]
fn write_solid(w: &mut Writer, solid: &SerSolid) {
    w.key_usize(b"{\"outer_shell\":", solid.outer_shell);
    w.key_usizes(b",\"inner_shells\":", &solid.inner_shells);
    w.byte(b'}');
}

#[inline(never)]
fn write_coedge(w: &mut Writer, coedge: &SerCoedgeAuthorityView<'_>) {
    w.key_usize(b"{\"edge\":", coedge.edge);
    w.key_bool(b",\"forward\":", coedge.forward);
    w.key_i64(
        b",\"periodic_winding\":[",
        i64::from(coedge.periodic_winding[0]),
    );
    w.key_i64(b",", i64::from(coedge.periodic_winding[1]));
    w.byte(b']');
    if let Some(pcurve) = &coedge.pcurve {
        w.raw(b",\"pcurve\":{\"curve\":");
        write_curve2d(w, pcurve.curve);
        w.key_f64(b",\"t_start\":", pcurve.t_start);
        w.key_f64(b",\"t_end\":", pcurve.t_end);
        w.byte(b'}');
    }
    w.byte(b'}');
}

#[inline(never)]
fn write_curve2d(w: &mut Writer, curve: &Curve2D) {
    match curve {
        Curve2D::Line(line) => {
            w.key_f64s(b"{\"Line\":{\"origin\":", &line.origin().0);
            w.key_f64s(b",\"direction\":", &line.direction().0);
        }
        Curve2D::Circle(circle) => {
            w.key_f64s(b"{\"Circle\":{\"center\":", &circle.center().0);
            w.key_f64(b",\"radius\":", circle.radius());
        }
        Curve2D::Ellipse(ellipse) => {
            w.key_f64s(b"{\"Ellipse\":{\"center\":", &ellipse.center().0);
            w.key_f64(b",\"semi_major\":", ellipse.semi_major());
            w.key_f64(b",\"semi_minor\":", ellipse.semi_minor());
            w.key_f64(b",\"rotation\":", ellipse.rotation());
        }
        Curve2D::Nurbs(nurbs) => {
            w.key_usize(b"{\"Nurbs\":{\"degree\":", nurbs.degree());
            w.key_f64s(b",\"knots\":", nurbs.knots());
            write_points(w, b",\"control_points\":", nurbs.control_points());
            w.key_f64s(b",\"weights\":", nurbs.weights());
        }
    }
    w.raw(b"}}");
}

#[inline(never)]
fn write_journal(w: &mut Writer, journal: &SerJournal) {
    w.key_u64(b",\"journal\":{\"next_op\":", journal.next_op);
    w.key_u64(b",\"next_ordinal\":", journal.next_ordinal);
    w.raw(b",\"index\":");
    list(w, &journal.index, |w, entry| {
        w.key_u64(b"{\"ordinal\":", entry.ordinal);
        w.raw(b",\"kind\":");
        w.string(&entry.kind);
        match entry.local {
            Some(local) => w.key_usize(b",\"local\":", local),
            None => w.raw(b",\"local\":null"),
        }
        w.byte(b'}');
    });
    w.raw(b",\"entries\":");
    list(w, &journal.entries, write_journal_entry);
    w.byte(b'}');
}

#[inline(never)]
fn write_journal_entry(w: &mut Writer, entry: &SerJournalEntry) {
    w.key_u64(b"{\"op\":", entry.op);
    w.raw(b",\"kind\":");
    w.string(&entry.kind);
    match &entry.payload {
        SerJournalPayload::Evolution {
            construction,
            scope,
            events,
        } => {
            w.key_bool(
                b",\"payload\":\"Evolution\",\"construction\":",
                *construction,
            );
            w.key_u64s(b",\"scope\":", scope);
            w.raw(b",\"events\":");
            list(w, events, |w, (subject, event)| {
                w.key_u64(b"[", *subject);
                match event {
                    SerJournalEvent::Preserved { from } => {
                        w.key_u64(b",{\"event\":\"Preserved\",\"from\":", *from);
                    }
                    SerJournalEvent::Modified { from } => {
                        w.key_u64(b",{\"event\":\"Modified\",\"from\":", *from);
                    }
                    SerJournalEvent::Generated { sources } => {
                        w.key_u64s(b",{\"event\":\"Generated\",\"sources\":", sources);
                    }
                    SerJournalEvent::Merged { from } => {
                        w.key_u64s(b",{\"event\":\"Merged\",\"from\":", from);
                    }
                    SerJournalEvent::Deleted => w.raw(b",{\"event\":\"Deleted\""),
                    SerJournalEvent::Unresolved { candidates } => {
                        w.key_u64s(b",{\"event\":\"Unresolved\",\"candidates\":", candidates);
                    }
                }
                w.raw(b"}]");
            });
        }
        SerJournalPayload::Barrier { affected } => {
            w.key_u64s(b",\"payload\":\"Barrier\",\"affected\":", affected);
        }
        SerJournalPayload::GlobalBarrier => w.raw(b",\"payload\":\"GlobalBarrier\""),
    }
    w.byte(b'}');
}

#[inline(never)]
fn write_attributes(w: &mut Writer, attributes: &SerAttributes) {
    w.raw(b",\"attributes\":{");
    if !attributes.solids.is_empty() {
        w.raw(b"\"solids\":");
        list(w, &attributes.solids, write_entity_attributes);
    }
    if !attributes.faces.is_empty() {
        if !attributes.solids.is_empty() {
            w.byte(b',');
        }
        w.raw(b"\"faces\":");
        list(w, &attributes.faces, write_entity_attributes);
    }
    w.byte(b'}');
}

fn write_entity_attributes(w: &mut Writer, (local, entity): &(usize, SerEntityAttributes)) {
    w.key_usize(b"[", *local);
    w.raw(b",{");
    if let Some(name) = &entity.name {
        w.raw(b"\"name\":");
        w.string(name);
    }
    if let Some((r, g, b)) = entity.color {
        let key: &[u8] = if entity.name.is_some() {
            b",\"color\":"
        } else {
            b"\"color\":"
        };
        w.key_f64s(key, &[r, g, b]);
    }
    w.raw(b"}]");
}
