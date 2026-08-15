//! Exact serialization of solid and compound topology sub-arenas.
//!
//! Captures every entity reachable from selected solid and compound roots —
//! vertices (with exact `Point3` and tolerance), edges (curve + analytic
//! params), wires, faces (surface + analytic params + reversed flag), shells,
//! solids, compounds, and the pcurves on captured (edge, face) pairs — and
//! replays them into a [`Topology`] with byte-identical f64 values.
//!
//! Unlike the geometry-exchange formats (STEP, IGES), this preserves the
//! kernel's in-memory representation verbatim: no curve/surface re-derivation,
//! no tolerance normalization, no vertex welding. It exists so an in-memory
//! operand captured from a live session (e.g. a WASM kernel) can be replayed
//! in a native Rust harness with the *exact* floating-point state that drives
//! sub-ULP-sensitive boolean behavior.
//!
//! Entity ids are remapped to dense local indices in deterministic discovery
//! order, so the dump is compact and self-contained (independent of the
//! source arena's global id layout). Deserialization always allocates fresh
//! ids; session state, retired slots, assemblies, GCS sketches, and checkpoints
//! are deliberately outside this format.

use std::collections::HashMap;

use brepkit_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use brepkit_math::curves2d::Curve2D;
use brepkit_math::nurbs::curve::NurbsCurve;
use brepkit_math::nurbs::surface::NurbsSurface;
use brepkit_math::surfaces::{
    ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
};
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::compound::{Compound, CompoundId};
use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
use brepkit_topology::face::{Face, FaceSurface};
use brepkit_topology::pcurve::PCurve;
use brepkit_topology::shell::{Shell, ShellId};
use brepkit_topology::solid::{Solid, SolidId};
use brepkit_topology::topology::Topology;
use brepkit_topology::vertex::Vertex;
use brepkit_topology::wire::{OrientedEdge, Wire};
use serde::{Deserialize, Serialize};

use crate::IoError;
use crate::limits::{ImportLimits, ensure_input_size, ensure_limit};

/// Serialized form of an [`EdgeCurve`].
#[derive(Debug, Clone, Serialize, Deserialize)]
enum SerEdgeCurve {
    Line,
    NurbsCurve(NurbsCurve),
    Circle(Circle3D),
    Ellipse(Ellipse3D),
    Hyperbola(Hyperbola3D),
    Parabola(Parabola3D),
}

impl SerEdgeCurve {
    fn from_curve(curve: &EdgeCurve) -> Self {
        match curve {
            EdgeCurve::Line => Self::Line,
            EdgeCurve::NurbsCurve(c) => Self::NurbsCurve(c.clone()),
            EdgeCurve::Circle(c) => Self::Circle(c.clone()),
            EdgeCurve::Ellipse(e) => Self::Ellipse(e.clone()),
            EdgeCurve::Hyperbola(h) => Self::Hyperbola(h.clone()),
            EdgeCurve::Parabola(pb) => Self::Parabola(pb.clone()),
        }
    }

    fn into_curve(self) -> EdgeCurve {
        match self {
            Self::Line => EdgeCurve::Line,
            Self::NurbsCurve(c) => EdgeCurve::NurbsCurve(c),
            Self::Circle(c) => EdgeCurve::Circle(c),
            Self::Ellipse(e) => EdgeCurve::Ellipse(e),
            Self::Hyperbola(h) => EdgeCurve::Hyperbola(h),
            Self::Parabola(pb) => EdgeCurve::Parabola(pb),
        }
    }
}

/// Serialized form of a [`FaceSurface`].
#[derive(Debug, Clone, Serialize, Deserialize)]
enum SerFaceSurface {
    Plane { normal: Vec3, d: f64 },
    Nurbs(NurbsSurface),
    Cylinder(CylindricalSurface),
    Cone(ConicalSurface),
    Sphere(SphericalSurface),
    Torus(ToroidalSurface),
}

impl SerFaceSurface {
    fn from_surface(surface: &FaceSurface) -> Self {
        match surface {
            FaceSurface::Plane { normal, d } => Self::Plane {
                normal: *normal,
                d: *d,
            },
            FaceSurface::Nurbs(s) => Self::Nurbs(s.clone()),
            FaceSurface::Cylinder(s) => Self::Cylinder(s.clone()),
            FaceSurface::Cone(s) => Self::Cone(s.clone()),
            FaceSurface::Sphere(s) => Self::Sphere(s.clone()),
            FaceSurface::Torus(s) => Self::Torus(s.clone()),
        }
    }

    fn into_surface(self) -> FaceSurface {
        match self {
            Self::Plane { normal, d } => FaceSurface::Plane { normal, d },
            Self::Nurbs(s) => FaceSurface::Nurbs(s),
            Self::Cylinder(s) => FaceSurface::Cylinder(s),
            Self::Cone(s) => FaceSurface::Cone(s),
            Self::Sphere(s) => FaceSurface::Sphere(s),
            Self::Torus(s) => FaceSurface::Torus(s),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerVertex {
    point: Point3,
    tolerance: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerEdge {
    start: usize,
    end: usize,
    curve: SerEdgeCurve,
    tolerance: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerOrientedEdge {
    edge: usize,
    forward: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerWire {
    edges: Vec<SerOrientedEdge>,
    closed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerFace {
    outer_wire: usize,
    inner_wires: Vec<usize>,
    surface: SerFaceSurface,
    reversed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerShell {
    faces: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerSolid {
    outer_shell: usize,
    inner_shells: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerCompound {
    solids: Vec<usize>,
}

/// A pcurve attached to a captured (edge, face) pair, keyed by local indices.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerPCurve {
    edge: usize,
    face: usize,
    curve: Curve2D,
    t_start: f64,
    t_end: f64,
}

/// Frozen version 1 single-solid schema, retained for read compatibility.
#[derive(Debug, Deserialize)]
struct SerializedSolidV1 {
    /// Format version, so a future change can be detected on load.
    version: u32,
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    /// Local index of the solid's outer shell.
    outer_shell: usize,
    /// Local indices of the solid's inner (cavity) shells.
    inner_shells: Vec<usize>,
    pcurves: Vec<SerPCurve>,
}

/// Version 2 is additive: it retains v1 entity encodings and adds solid and
/// compound root tables. Released versions are read forever. Existing fields
/// and enum encodings must not change in place; incompatible additions require
/// a new version and a dedicated read path, while the v1 schema stays frozen.
const FORMAT_VERSION: u32 = 2;
const LEGACY_SINGLE_SOLID_VERSION: u32 = 1;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerializedDocumentV2 {
    version: u32,
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    /// Dense table of every solid directly selected or referenced by a compound.
    solids: Vec<SerSolid>,
    /// Entries in `solids` that were explicitly selected as solid roots.
    solid_roots: Vec<usize>,
    /// Explicitly selected compound roots, referencing the dense solid table.
    compounds: Vec<SerCompound>,
    pcurves: Vec<SerPCurve>,
}

#[derive(Debug, Deserialize)]
struct VersionHeader {
    version: u32,
}

struct ParsedDocument {
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    solids: Vec<SerSolid>,
    solid_roots: Vec<usize>,
    compounds: Vec<SerCompound>,
    pcurves: Vec<SerPCurve>,
}

/// Fresh topology roots reconstructed from an arena document.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeserializedDocument {
    /// Explicit solid roots, preserving document order and duplicates.
    pub solids: Vec<SolidId>,
    /// Explicit compound roots, preserving document order.
    pub compounds: Vec<CompoundId>,
}

/// Discovers and remaps a solid's reachable entities into dense local indices.
struct Builder<'a> {
    topo: &'a Topology,
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    vertex_map: HashMap<usize, usize>,
    edge_map: HashMap<usize, usize>,
    wire_map: HashMap<usize, usize>,
    face_map: HashMap<usize, usize>,
    shell_map: HashMap<usize, usize>,
    solids: Vec<SerSolid>,
    solid_map: HashMap<usize, usize>,
}

impl<'a> Builder<'a> {
    fn new(topo: &'a Topology) -> Self {
        Self {
            topo,
            vertices: Vec::new(),
            edges: Vec::new(),
            wires: Vec::new(),
            faces: Vec::new(),
            shells: Vec::new(),
            vertex_map: HashMap::new(),
            edge_map: HashMap::new(),
            wire_map: HashMap::new(),
            face_map: HashMap::new(),
            shell_map: HashMap::new(),
            solids: Vec::new(),
            solid_map: HashMap::new(),
        }
    }

    fn intern_vertex(&mut self, id: brepkit_topology::vertex::VertexId) -> Result<usize, IoError> {
        if let Some(&local) = self.vertex_map.get(&id.index()) {
            return Ok(local);
        }
        let v = self.topo.vertex(id)?;
        let local = self.vertices.len();
        self.vertices.push(SerVertex {
            point: v.point(),
            tolerance: v.tolerance(),
        });
        self.vertex_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_edge(&mut self, id: EdgeId) -> Result<usize, IoError> {
        if let Some(&local) = self.edge_map.get(&id.index()) {
            return Ok(local);
        }
        let e = self.topo.edge(id)?;
        let start = self.intern_vertex(e.start())?;
        let end = self.intern_vertex(e.end())?;
        let curve = SerEdgeCurve::from_curve(e.curve());
        let tolerance = e.tolerance();
        let local = self.edges.len();
        self.edges.push(SerEdge {
            start,
            end,
            curve,
            tolerance,
        });
        self.edge_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_wire(&mut self, id: brepkit_topology::wire::WireId) -> Result<usize, IoError> {
        if let Some(&local) = self.wire_map.get(&id.index()) {
            return Ok(local);
        }
        let w = self.topo.wire(id)?;
        let mut edges = Vec::with_capacity(w.edges().len());
        for oe in w.edges() {
            let edge = self.intern_edge(oe.edge())?;
            edges.push(SerOrientedEdge {
                edge,
                forward: oe.is_forward(),
            });
        }
        let closed = w.is_closed();
        let local = self.wires.len();
        self.wires.push(SerWire { edges, closed });
        self.wire_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_face(&mut self, id: brepkit_topology::face::FaceId) -> Result<usize, IoError> {
        if let Some(&local) = self.face_map.get(&id.index()) {
            return Ok(local);
        }
        let f = self.topo.face(id)?;
        let outer_wire = self.intern_wire(f.outer_wire())?;
        let mut inner_wires = Vec::with_capacity(f.inner_wires().len());
        for &iw in f.inner_wires() {
            inner_wires.push(self.intern_wire(iw)?);
        }
        let surface = SerFaceSurface::from_surface(f.surface());
        let reversed = f.is_reversed();
        let local = self.faces.len();
        self.faces.push(SerFace {
            outer_wire,
            inner_wires,
            surface,
            reversed,
        });
        self.face_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_shell(&mut self, id: ShellId) -> Result<usize, IoError> {
        if let Some(&local) = self.shell_map.get(&id.index()) {
            return Ok(local);
        }
        let s = self.topo.shell(id)?;
        let mut faces = Vec::with_capacity(s.faces().len());
        for &fid in s.faces() {
            faces.push(self.intern_face(fid)?);
        }
        let local = self.shells.len();
        self.shells.push(SerShell { faces });
        self.shell_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_solid(&mut self, id: SolidId) -> Result<usize, IoError> {
        if let Some(&local) = self.solid_map.get(&id.index()) {
            return Ok(local);
        }
        let solid = self.topo.solid(id)?;
        let outer_shell_id = solid.outer_shell();
        let inner_shell_ids = solid.inner_shells().to_vec();
        let outer_shell = self.intern_shell(outer_shell_id)?;
        let mut inner_shells = Vec::with_capacity(inner_shell_ids.len());
        for shell in inner_shell_ids {
            inner_shells.push(self.intern_shell(shell)?);
        }
        let local = self.solids.len();
        self.solids.push(SerSolid {
            outer_shell,
            inner_shells,
        });
        self.solid_map.insert(id.index(), local);
        Ok(local)
    }

    /// Collects all pcurves whose (edge, face) are both in the captured set.
    fn collect_pcurves(&self) -> Vec<SerPCurve> {
        let mut out = Vec::new();
        for (&global_face, &local_face) in &self.face_map {
            let Some(fid) = self.topo.face_id_from_index(global_face) else {
                continue;
            };
            for (eid, pc) in self.topo.pcurves().pcurves_for_face(fid) {
                if let Some(&local_edge) = self.edge_map.get(&eid.index()) {
                    out.push(SerPCurve {
                        edge: local_edge,
                        face: local_face,
                        curve: pc.curve().clone(),
                        t_start: pc.t_start(),
                        t_end: pc.t_end(),
                    });
                }
            }
        }
        // Deterministic order so the dump is reproducible across runs
        // (HashMap iteration order is randomized per process).
        out.sort_unstable_by_key(|p| (p.face, p.edge));
        out
    }
}

/// Serializes a solid's complete topology sub-arena to a byte buffer.
///
/// The result captures every vertex, edge, wire, face, shell reachable from
/// `solid_id`, plus the pcurves on the captured (edge, face) pairs, with
/// byte-identical f64 values. Load it with [`deserialize_solid`].
///
/// # Errors
///
/// Returns [`IoError`] if any referenced entity is missing or serialization
/// fails.
pub fn serialize_solid(topo: &Topology, solid_id: SolidId) -> Result<Vec<u8>, IoError> {
    serialize_solids(topo, &[solid_id])
}

/// Serializes explicit solid roots into a version 2 arena document.
///
/// Shared topology is emitted once using dense local indices. Root order and
/// duplicate roots are preserved. Use [`serialize_document`] when compound
/// roots must be included as well.
///
/// # Errors
///
/// Returns [`IoError`] if any root or referenced entity is missing, or JSON
/// serialization fails.
pub fn serialize_solids(topo: &Topology, solid_ids: &[SolidId]) -> Result<Vec<u8>, IoError> {
    serialize_document(topo, solid_ids, &[])
}

/// Serializes solid and compound roots into one version 2 arena document.
///
/// Every solid referenced by a selected compound is included in the dense
/// solid table. Only `solid_ids` are returned as explicit solid roots after
/// deserialization; compound members remain accessible through their restored
/// [`Compound`] roots. Global arena indices and unrelated session state are
/// not serialized.
///
/// # Errors
///
/// Returns [`IoError`] if any root or referenced entity is missing, or JSON
/// serialization fails.
pub fn serialize_document(
    topo: &Topology,
    solid_ids: &[SolidId],
    compound_ids: &[CompoundId],
) -> Result<Vec<u8>, IoError> {
    let mut builder = Builder::new(topo);
    let mut solid_roots = Vec::with_capacity(solid_ids.len());
    for &solid in solid_ids {
        solid_roots.push(builder.intern_solid(solid)?);
    }

    let mut compounds = Vec::with_capacity(compound_ids.len());
    for &compound_id in compound_ids {
        let member_ids = builder.topo.compound(compound_id)?.solids().to_vec();
        let mut members = Vec::with_capacity(member_ids.len());
        for solid in member_ids {
            members.push(builder.intern_solid(solid)?);
        }
        compounds.push(SerCompound { solids: members });
    }

    let pcurves = builder.collect_pcurves();
    let dump = SerializedDocumentV2 {
        version: FORMAT_VERSION,
        vertices: builder.vertices,
        edges: builder.edges,
        wires: builder.wires,
        faces: builder.faces,
        shells: builder.shells,
        solids: builder.solids,
        solid_roots,
        compounds,
        pcurves,
    };

    serde_json::to_vec(&dump).map_err(|e| IoError::ParseError {
        reason: format!("arena serialization failed: {e}"),
    })
}

/// Reconstructs one solid from a version 1 or single-root version 2 document.
///
/// All entities are appended to `topo` as fresh ids. Floating-point values are
/// restored byte-for-byte; analytic curves and surfaces are rebuilt by direct
/// field population (no constructor re-derivation), so the parametric frame is
/// preserved exactly. Version 1 documents remain supported permanently.
///
/// # Errors
///
/// Returns [`IoError`] if the buffer is malformed, references an out-of-range
/// local index, or any entity construction fails.
pub fn deserialize_solid(bytes: &[u8], topo: &mut Topology) -> Result<SolidId, IoError> {
    deserialize_solid_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstruct a solid with explicit hostile-input resource limits.
///
/// Limits are checked before any topology mutation. The encoded byte limit
/// bounds allocations performed by JSON parsing; model-entity limits bound
/// the topology allocations that follow.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the encoded dump or any entity
/// collection exceeds the configured budget.
pub fn deserialize_solid_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<SolidId, IoError> {
    let document = parse_document(bytes, limits)?;
    if document.solids.len() != 1
        || document.solid_roots.as_slice() != [0]
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason: "single-solid deserialization requires exactly one solid root and no compounds"
                .to_owned(),
        });
    }
    let restored = replay_document(document, topo)?;
    restored
        .solids
        .into_iter()
        .next()
        .ok_or_else(|| index_err("solid", 0))
}

/// Reconstructs explicit solid roots from a version 1 or version 2 document.
///
/// Version 1 input produces a one-element vector. Version 2 input must not
/// contain compound roots; use [`deserialize_document`] for mixed roots.
/// Every entity receives a fresh topology id.
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, contains compounds, or
/// reconstruction fails.
pub fn deserialize_solids(bytes: &[u8], topo: &mut Topology) -> Result<Vec<SolidId>, IoError> {
    deserialize_solids_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs solid roots with explicit hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or [`IoError::ParseError`] when it contains compound roots.
pub fn deserialize_solids_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<SolidId>, IoError> {
    let document = parse_document(bytes, limits)?;
    if !document.compounds.is_empty() {
        return Err(IoError::ParseError {
            reason: "solid-only deserialization does not accept compound roots".to_owned(),
        });
    }
    Ok(replay_document(document, topo)?.solids)
}

/// Reconstructs solid and compound roots from a version 1 or version 2 arena
/// document.
///
/// Version 1 input is represented as one solid root and no compounds. All
/// restored topology entities receive fresh ids.
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, exceeds default resource
/// limits, references an out-of-range local index, or reconstruction fails.
pub fn deserialize_document(
    bytes: &[u8],
    topo: &mut Topology,
) -> Result<DeserializedDocument, IoError> {
    deserialize_document_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs solid and compound roots with explicit hostile-input limits.
///
/// Limits are checked before any topology mutation. The encoded byte limit
/// bounds allocations performed by JSON parsing; model-entity limits bound
/// the topology allocations that follow.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or another [`IoError`] when parsing or reconstruction fails.
pub fn deserialize_document_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<DeserializedDocument, IoError> {
    let document = parse_document(bytes, limits)?;
    replay_document(document, topo)
}

fn parse_document(bytes: &[u8], limits: ImportLimits) -> Result<ParsedDocument, IoError> {
    ensure_input_size(bytes.len(), limits)?;
    let header: VersionHeader = serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
        reason: format!("arena deserialization failed: {e}"),
    })?;
    let document = match header.version {
        LEGACY_SINGLE_SOLID_VERSION => {
            let dump: SerializedSolidV1 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v1 deserialization failed: {e}"),
                })?;
            if dump.version != LEGACY_SINGLE_SOLID_VERSION {
                return Err(IoError::ParseError {
                    reason: format!("arena v1 document reported version {}", dump.version),
                });
            }
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: vec![SerSolid {
                    outer_shell: dump.outer_shell,
                    inner_shells: dump.inner_shells,
                }],
                solid_roots: vec![0],
                compounds: Vec::new(),
                pcurves: dump.pcurves,
            }
        }
        FORMAT_VERSION => {
            let dump: SerializedDocumentV2 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v2 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                compounds: dump.compounds,
                pcurves: dump.pcurves,
            }
        }
        version => {
            return Err(IoError::ParseError {
                reason: format!(
                    "unsupported arena dump version {version} (supported: {LEGACY_SINGLE_SOLID_VERSION}, {FORMAT_VERSION})"
                ),
            });
        }
    };
    check_document_limits(&document, limits)?;
    Ok(document)
}

fn check_document_limits(document: &ParsedDocument, limits: ImportLimits) -> Result<(), IoError> {
    let inner_shell_refs = checked_reference_count(
        "arena inner shell references",
        document.solids.iter().map(|solid| solid.inner_shells.len()),
        limits.max_model_entities,
    )?;
    let compound_refs = checked_reference_count(
        "arena compound solid references",
        document
            .compounds
            .iter()
            .map(|compound| compound.solids.len()),
        limits.max_model_entities,
    )?;
    for (resource, actual) in [
        ("arena vertices", document.vertices.len()),
        ("arena edges", document.edges.len()),
        ("arena wires", document.wires.len()),
        ("arena faces", document.faces.len()),
        ("arena shells", document.shells.len()),
        ("arena solids", document.solids.len()),
        ("arena solid roots", document.solid_roots.len()),
        ("arena compounds", document.compounds.len()),
        ("arena inner shell references", inner_shell_refs),
        ("arena compound solid references", compound_refs),
        ("arena pcurves", document.pcurves.len()),
    ] {
        ensure_limit(resource, actual, limits.max_model_entities)?;
    }
    let total_entities = checked_reference_count(
        "arena total entities",
        [
            document.vertices.len(),
            document.edges.len(),
            document.wires.len(),
            document.faces.len(),
            document.shells.len(),
            document.solids.len(),
            document.compounds.len(),
            document.pcurves.len(),
        ],
        limits.max_model_entities,
    )?;
    ensure_limit(
        "arena total entities",
        total_entities,
        limits.max_model_entities,
    )?;

    for &index in &document.solid_roots {
        if index >= document.solids.len() {
            return Err(index_err("solid", index));
        }
    }
    for compound in &document.compounds {
        for &index in &compound.solids {
            if index >= document.solids.len() {
                return Err(index_err("solid", index));
            }
        }
    }
    Ok(())
}

fn checked_reference_count(
    resource: &'static str,
    counts: impl IntoIterator<Item = usize>,
    limit: usize,
) -> Result<usize, IoError> {
    counts.into_iter().try_fold(0_usize, |total, count| {
        total.checked_add(count).ok_or(IoError::LimitExceeded {
            resource,
            limit,
            actual: usize::MAX,
        })
    })
}

fn replay_document(
    document: ParsedDocument,
    topo: &mut Topology,
) -> Result<DeserializedDocument, IoError> {
    // Replay against a snapshot so every parse/validation/construction error
    // leaves the caller's live topology untouched.  Committing with one move
    // also preserves the fresh ids allocated relative to the existing arenas.
    let mut staged = topo.clone();
    let restored = replay_document_into(document, &mut staged)?;
    *topo = staged;
    Ok(restored)
}

fn replay_document_into(
    document: ParsedDocument,
    topo: &mut Topology,
) -> Result<DeserializedDocument, IoError> {
    let ParsedDocument {
        vertices,
        edges,
        wires,
        faces,
        shells,
        solids,
        solid_roots,
        compounds,
        pcurves,
    } = document;

    let mut vertex_ids = Vec::with_capacity(vertices.len());
    for v in vertices {
        vertex_ids.push(topo.add_vertex(Vertex::new(v.point, v.tolerance)));
    }

    let mut edge_ids = Vec::with_capacity(edges.len());
    for e in edges {
        let start = *vertex_ids
            .get(e.start)
            .ok_or_else(|| index_err("vertex", e.start))?;
        let end = *vertex_ids
            .get(e.end)
            .ok_or_else(|| index_err("vertex", e.end))?;
        if let SerEdgeCurve::NurbsCurve(curve) = &e.curve {
            curve.validate().map_err(|err| IoError::ParseError {
                reason: format!("invalid arena NURBS curve: {err}"),
            })?;
        }
        edge_ids.push(topo.add_edge(Edge::with_tolerance(
            start,
            end,
            e.curve.into_curve(),
            e.tolerance,
        )));
    }

    let mut wire_ids = Vec::with_capacity(wires.len());
    for w in wires {
        let mut oriented = Vec::with_capacity(w.edges.len());
        for oe in w.edges {
            let edge = *edge_ids
                .get(oe.edge)
                .ok_or_else(|| index_err("edge", oe.edge))?;
            oriented.push(OrientedEdge::new(edge, oe.forward));
        }
        wire_ids.push(topo.add_wire(Wire::new(oriented, w.closed)?));
    }

    let mut face_ids = Vec::with_capacity(faces.len());
    for f in faces {
        let outer = *wire_ids
            .get(f.outer_wire)
            .ok_or_else(|| index_err("wire", f.outer_wire))?;
        let mut inner = Vec::with_capacity(f.inner_wires.len());
        for iw in f.inner_wires {
            inner.push(*wire_ids.get(iw).ok_or_else(|| index_err("wire", iw))?);
        }
        if let SerFaceSurface::Nurbs(surface) = &f.surface {
            surface.validate().map_err(|err| IoError::ParseError {
                reason: format!("invalid arena NURBS surface: {err}"),
            })?;
        }
        let mut face = Face::new(outer, inner, f.surface.into_surface());
        face.set_reversed(f.reversed);
        face_ids.push(topo.add_face(face));
    }

    let mut shell_ids = Vec::with_capacity(shells.len());
    for s in shells {
        let mut faces = Vec::with_capacity(s.faces.len());
        for fid in s.faces {
            faces.push(*face_ids.get(fid).ok_or_else(|| index_err("face", fid))?);
        }
        let shell = if faces.is_empty() {
            Shell::empty()
        } else {
            Shell::new(faces)?
        };
        shell_ids.push(topo.add_shell(shell));
    }

    for pc in pcurves {
        let edge = *edge_ids
            .get(pc.edge)
            .ok_or_else(|| index_err("edge", pc.edge))?;
        let face = *face_ids
            .get(pc.face)
            .ok_or_else(|| index_err("face", pc.face))?;
        topo.pcurves_mut()
            .set(edge, face, PCurve::new(pc.curve, pc.t_start, pc.t_end));
    }

    let mut solid_ids = Vec::with_capacity(solids.len());
    for solid in solids {
        let outer = *shell_ids
            .get(solid.outer_shell)
            .ok_or_else(|| index_err("shell", solid.outer_shell))?;
        let inner = solid
            .inner_shells
            .iter()
            .map(|&index| {
                shell_ids
                    .get(index)
                    .copied()
                    .ok_or_else(|| index_err("shell", index))
            })
            .collect::<Result<Vec<_>, _>>()?;
        solid_ids.push(topo.add_solid(Solid::new(outer, inner)));
    }

    let restored_solids = solid_roots
        .into_iter()
        .map(|index| {
            solid_ids
                .get(index)
                .copied()
                .ok_or_else(|| index_err("solid", index))
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut restored_compounds = Vec::with_capacity(compounds.len());
    for compound in compounds {
        let members = compound
            .solids
            .into_iter()
            .map(|index| {
                solid_ids
                    .get(index)
                    .copied()
                    .ok_or_else(|| index_err("solid", index))
            })
            .collect::<Result<Vec<_>, _>>()?;
        restored_compounds.push(topo.add_compound(Compound::new(members)));
    }

    Ok(DeserializedDocument {
        solids: restored_solids,
        compounds: restored_compounds,
    })
}

fn index_err(kind: &str, index: usize) -> IoError {
    IoError::ParseError {
        reason: format!("arena dump references out-of-range {kind} index {index}"),
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use brepkit_operations::primitives::{make_box, make_cylinder};
    use brepkit_topology::explorer::solid_faces;

    fn face_type_histogram(
        topo: &Topology,
        solid: SolidId,
    ) -> std::collections::BTreeMap<&'static str, usize> {
        let mut hist = std::collections::BTreeMap::new();
        for fid in solid_faces(topo, solid).unwrap() {
            let tag = topo.face(fid).unwrap().surface().type_tag();
            *hist.entry(tag).or_insert(0) += 1;
        }
        hist
    }

    #[test]
    fn roundtrip_box_preserves_counts_and_exact_bits() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();

        let bytes = serialize_solid(&topo, solid).unwrap();
        assert_eq!(
            serde_json::from_slice::<VersionHeader>(&bytes)
                .unwrap()
                .version,
            FORMAT_VERSION
        );
        let mut topo2 = Topology::new();
        let solid2 = deserialize_solid(&bytes, &mut topo2).unwrap();

        // Entity counts identical for the sub-arena.
        assert_eq!(topo2.num_vertices(), topo.num_vertices());
        assert_eq!(topo2.num_edges(), topo.num_edges());
        assert_eq!(topo2.num_wires(), topo.num_wires());
        assert_eq!(topo2.num_faces(), topo.num_faces());
        assert_eq!(topo2.num_shells(), topo.num_shells());
        assert_eq!(topo2.num_solids(), 1);

        // Face-type breakdown identical.
        assert_eq!(
            face_type_histogram(&topo2, solid2),
            face_type_histogram(&topo, solid)
        );

        // Sampled vertex position must match bit-for-bit.
        let orig: Vec<Point3> = solid_faces(&topo, solid)
            .unwrap()
            .iter()
            .flat_map(|&fid| {
                let w = topo.face(fid).unwrap().outer_wire();
                topo.wire(w)
                    .unwrap()
                    .edges()
                    .iter()
                    .map(|oe| topo.edge(oe.edge()).unwrap().start())
                    .map(|vid| topo.vertex(vid).unwrap().point())
                    .collect::<Vec<_>>()
            })
            .collect();
        let restored: Vec<Point3> = solid_faces(&topo2, solid2)
            .unwrap()
            .iter()
            .flat_map(|&fid| {
                let w = topo2.face(fid).unwrap().outer_wire();
                topo2
                    .wire(w)
                    .unwrap()
                    .edges()
                    .iter()
                    .map(|oe| topo2.edge(oe.edge()).unwrap().start())
                    .map(|vid| topo2.vertex(vid).unwrap().point())
                    .collect::<Vec<_>>()
            })
            .collect();
        assert_eq!(orig.len(), restored.len());
        for (a, b) in orig.iter().zip(&restored) {
            assert_eq!(a.x().to_bits(), b.x().to_bits(), "x bits differ");
            assert_eq!(a.y().to_bits(), b.y().to_bits(), "y bits differ");
            assert_eq!(a.z().to_bits(), b.z().to_bits(), "z bits differ");
        }
    }

    #[test]
    fn deserialize_limits_fail_before_topology_mutation() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 10.0, 20.0, 30.0).unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let limits = ImportLimits {
            max_input_bytes: bytes.len(),
            max_model_entities: 1,
            ..ImportLimits::default()
        };
        let mut destination = Topology::new();
        let error = deserialize_solid_with_limits(&bytes, &mut destination, limits).unwrap_err();
        assert!(matches!(error, IoError::LimitExceeded { .. }));
        assert_eq!(destination.num_vertices(), 0);
        assert_eq!(destination.num_edges(), 0);
        assert_eq!(destination.num_faces(), 0);
        assert_eq!(destination.num_solids(), 0);
    }

    #[test]
    fn deserialize_rejects_oversized_input_before_json_parse() {
        let limits = ImportLimits {
            max_input_bytes: 2,
            ..ImportLimits::default()
        };
        let mut topo = Topology::new();
        let error = deserialize_solid_with_limits(b"{}\n", &mut topo, limits).unwrap_err();
        assert!(matches!(
            error,
            IoError::LimitExceeded {
                resource: "input bytes",
                ..
            }
        ));
        assert_eq!(topo.num_solids(), 0);
    }

    #[test]
    fn malformed_document_is_atomic() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 10.0, 20.0, 30.0).unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["wires"][0]["edges"][0]["edge"] = serde_json::json!(usize::MAX);
        let malformed = serde_json::to_vec(&document).unwrap();

        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let before = destination.clone();
        let error = deserialize_solid(&malformed, &mut destination).unwrap_err();

        assert!(matches!(error, IoError::ParseError { .. }));
        assert_eq!(destination.num_vertices(), before.num_vertices());
        assert_eq!(destination.num_edges(), before.num_edges());
        assert_eq!(destination.num_wires(), before.num_wires());
        assert_eq!(destination.num_faces(), before.num_faces());
        assert_eq!(destination.num_shells(), before.num_shells());
        assert_eq!(destination.num_solids(), before.num_solids());
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn structurally_invalid_nurbs_is_rejected_atomically() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 10.0, 20.0, 30.0).unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["edges"][0]["curve"] = serde_json::json!({
            "NurbsCurve": {
                "degree": 2,
                "knots": [],
                "control_points": [],
                "weights": []
            }
        });
        let malformed = serde_json::to_vec(&document).unwrap();

        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let error = deserialize_solid(&malformed, &mut destination).unwrap_err();

        assert!(matches!(error, IoError::ParseError { .. }));
        assert_eq!(destination.num_vertices(), 0);
        assert_eq!(destination.num_edges(), 0);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn roundtrip_cylinder_preserves_analytic_surface_exact() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 7.5, 12.5).unwrap();

        let bytes = serialize_solid(&topo, solid).unwrap();
        let mut topo2 = Topology::new();
        let solid2 = deserialize_solid(&bytes, &mut topo2).unwrap();

        // The cylindrical surface must round-trip with the exact same frame.
        let mut cyl_orig = None;
        for fid in solid_faces(&topo, solid).unwrap() {
            if let FaceSurface::Cylinder(c) = topo.face(fid).unwrap().surface() {
                cyl_orig = Some(c.clone());
            }
        }
        let mut cyl_restored = None;
        for fid in solid_faces(&topo2, solid2).unwrap() {
            if let FaceSurface::Cylinder(c) = topo2.face(fid).unwrap().surface() {
                cyl_restored = Some(c.clone());
            }
        }
        let a = cyl_orig.expect("orig has cylinder");
        let b = cyl_restored.expect("restored has cylinder");
        assert_eq!(a.radius().to_bits(), b.radius().to_bits());
        for i in 0..3 {
            assert_eq!(a.origin().0[i].to_bits(), b.origin().0[i].to_bits());
            assert_eq!(a.axis().0[i].to_bits(), b.axis().0[i].to_bits());
            assert_eq!(a.x_axis().0[i].to_bits(), b.x_axis().0[i].to_bits());
            assert_eq!(a.y_axis().0[i].to_bits(), b.y_axis().0[i].to_bits());
        }
    }

    #[test]
    fn v1_minimal_document_remains_readable() {
        let bytes = br#"{"version":1,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[]}],"outer_shell":0,"inner_shells":[],"pcurves":[]}"#;
        let mut topo = Topology::new();

        let solid = deserialize_solid(bytes, &mut topo).unwrap();

        assert!(topo.is_empty_solid(solid));
        assert_eq!(topo.num_solids(), 1);
        assert_eq!(topo.num_shells(), 1);
    }

    #[test]
    fn deserialize_solid_accepts_single_root_v2_document() {
        let mut source = Topology::new();
        let solid = source.add_empty_solid();
        let bytes = serialize_solids(&source, &[solid]).unwrap();
        let mut destination = Topology::new();

        let restored = deserialize_solid(&bytes, &mut destination).unwrap();

        assert!(destination.is_empty_solid(restored));
    }

    #[test]
    fn v2_multi_solid_roundtrip_preserves_root_order_and_duplicates() {
        let mut source = Topology::new();
        let first = make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
        let second = make_cylinder(&mut source, 2.0, 4.0).unwrap();
        let bytes = serialize_solids(&source, &[second, first, second]).unwrap();
        assert_eq!(
            serde_json::from_slice::<VersionHeader>(&bytes)
                .unwrap()
                .version,
            FORMAT_VERSION
        );

        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let roots = deserialize_solids(&bytes, &mut destination).unwrap();

        assert_eq!(roots.len(), 3);
        assert_eq!(roots[0], roots[2]);
        assert_ne!(roots[0], roots[1]);
        assert!(roots[0].index() > sentinel.index());
        assert_eq!(
            face_type_histogram(&destination, roots[0]),
            face_type_histogram(&source, second)
        );
        assert_eq!(
            face_type_histogram(&destination, roots[1]),
            face_type_histogram(&source, first)
        );
    }

    #[test]
    fn v2_roundtrip_preserves_topology_shared_by_distinct_solids() {
        let mut source = Topology::new();
        let first = make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
        let shared_shell = source.solid(first).unwrap().outer_shell();
        let second = source.add_solid(Solid::new(shared_shell, Vec::new()));
        let bytes = serialize_solids(&source, &[first, second]).unwrap();
        let mut destination = Topology::new();

        let restored = deserialize_solids(&bytes, &mut destination).unwrap();

        assert_eq!(restored.len(), 2);
        assert_ne!(restored[0], restored[1]);
        assert_eq!(destination.num_shells(), 1);
        assert_eq!(
            destination.solid(restored[0]).unwrap().outer_shell(),
            destination.solid(restored[1]).unwrap().outer_shell()
        );
    }

    #[test]
    fn v2_mixed_roots_match_golden_and_roundtrip() {
        let mut source = Topology::new();
        let direct = source.add_empty_solid();
        let member = source.add_empty_solid();
        let compound = source.add_compound(Compound::new(vec![member]));

        let bytes = serialize_document(&source, &[direct], &[compound]).unwrap();
        let golden = include_str!("../tests/data/arena_v2_multi_compound.json").trim_end();
        assert_eq!(std::str::from_utf8(&bytes).unwrap(), golden);

        let mut destination = Topology::new();
        let restored = deserialize_document(&bytes, &mut destination).unwrap();
        assert_eq!(restored.solids.len(), 1);
        assert_eq!(restored.compounds.len(), 1);
        assert!(destination.is_empty_solid(restored.solids[0]));
        let restored_compound = destination.compound(restored.compounds[0]).unwrap();
        assert_eq!(restored_compound.solids().len(), 1);
        assert!(destination.is_empty_solid(restored_compound.solids()[0]));
        assert_ne!(restored.solids[0], restored_compound.solids()[0]);

        let roundtrip =
            serialize_document(&destination, &restored.solids, &restored.compounds).unwrap();
        assert_eq!(roundtrip, bytes);
    }

    #[test]
    fn invalid_v2_solid_root_does_not_mutate_topology() {
        let bytes = br#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[]}],"solids":[{"outer_shell":0,"inner_shells":[]}],"solid_roots":[1],"compounds":[],"pcurves":[]}"#;
        let mut destination = Topology::new();
        destination.add_empty_solid();
        let counts_before = (destination.num_shells(), destination.num_solids());

        let error = deserialize_solids(bytes, &mut destination).unwrap_err();

        assert!(error.to_string().contains("out-of-range solid index 1"));
        assert_eq!(
            (destination.num_shells(), destination.num_solids()),
            counts_before
        );
    }

    #[test]
    fn invalid_v2_compound_member_does_not_mutate_topology() {
        let bytes = br#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[]}],"solids":[{"outer_shell":0,"inner_shells":[]}],"solid_roots":[],"compounds":[{"solids":[1]}],"pcurves":[]}"#;
        let mut destination = Topology::new();
        destination.add_empty_solid();
        let counts_before = (
            destination.num_shells(),
            destination.num_solids(),
            destination.num_compounds(),
        );

        let error = deserialize_document(bytes, &mut destination).unwrap_err();

        assert!(error.to_string().contains("out-of-range solid index 1"));
        assert_eq!(
            (
                destination.num_shells(),
                destination.num_solids(),
                destination.num_compounds(),
            ),
            counts_before
        );
    }
}
