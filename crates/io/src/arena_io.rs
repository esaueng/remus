//! Exact serialization of solid, sheet, wire, and compound topology sub-arenas.
//!
//! Captures every entity reachable from selected solid, sheet, wire, and compound roots —
//! vertices (with exact `Point3` and tolerance), edges (curve + analytic
//! params), authoritative loops/coedges (including per-use pcurves), wires as
//! their compatibility view, faces (surface + analytic params + reversed
//! flag), shells, solids, and compounds — and replays them into a [`Topology`]
//! with byte-identical f64 values.
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

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::curves2d::Curve2D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::BodyClass;
use remus_topology::compound::{Compound, CompoundId};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::{Solid, SolidId};
use remus_topology::topology::Topology;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};
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
    /// Explicit trim interval on the curve (RFC 0002, Stage 3). Absent in
    /// documents written before trims were recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    trim: Option<(f64, f64)>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerOrientedEdge {
    edge: usize,
    forward: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SerBodyClass {
    Solid,
    Sheet,
    Wire,
    General,
}

impl From<BodyClass> for SerBodyClass {
    fn from(value: BodyClass) -> Self {
        match value {
            BodyClass::Solid => Self::Solid,
            BodyClass::Sheet => Self::Sheet,
            BodyClass::Wire => Self::Wire,
            BodyClass::General => Self::General,
        }
    }
}

impl From<SerBodyClass> for BodyClass {
    fn from(value: SerBodyClass) -> Self {
        match value {
            SerBodyClass::Solid => Self::Solid,
            SerBodyClass::Sheet => Self::Sheet,
            SerBodyClass::Wire => Self::Wire,
            SerBodyClass::General => Self::General,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerWire {
    edges: Vec<SerOrientedEdge>,
    closed: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    body_class: Option<SerBodyClass>,
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    body_class: Option<SerBodyClass>,
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
    /// Traversal orientation of this use (per-use pcurves, RFC 0002).
    /// Absent in documents written before orientation was recorded; the
    /// reader then resolves the use from the face's wires.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    forward: Option<bool>,
}

/// Pcurve geometry embedded in its authoritative coedge use (arena v3).
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerUsePCurve {
    curve: Curve2D,
    t_start: f64,
    t_end: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerCoedgeAuthority {
    edge: usize,
    forward: bool,
    periodic_winding: [i32; 2],
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pcurve: Option<SerUsePCurve>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerLoopAuthority {
    coedges: Vec<usize>,
    closed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerFaceLoopAuthority {
    outer_loop: usize,
    inner_loops: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerBoundaryAuthority {
    loops: Vec<SerLoopAuthority>,
    coedges: Vec<SerCoedgeAuthority>,
    /// Parallel to the document's dense face table.
    faces: Vec<SerFaceLoopAuthority>,
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

/// One live-index entry of a serialized journal: the ordinal, the entity
/// kind, and the entity's dense local index in this document (`None` when
/// the entity is not part of the document — retired, or outside the
/// selected roots; the kind is kept so anchor output ordering stays
/// replay-stable).
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerJournalIndexEntry {
    ordinal: u64,
    kind: String,
    local: Option<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "event")]
enum SerJournalEvent {
    Preserved { from: u64 },
    Modified { from: u64 },
    Generated { sources: Vec<u64> },
    Merged { from: Vec<u64> },
    Deleted,
    Unresolved { candidates: Vec<u64> },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "payload")]
enum SerJournalPayload {
    Evolution {
        construction: bool,
        scope: Vec<u64>,
        events: Vec<(u64, SerJournalEvent)>,
    },
    Barrier {
        affected: Vec<u64>,
    },
    GlobalBarrier,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerJournalEntry {
    op: u64,
    kind: String,
    #[serde(flatten)]
    payload: SerJournalPayload,
}

/// The evolution journal (RFC 0003, Stage 5). Additive: absent in
/// journal-less documents and in documents written before this field
/// existed.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerJournal {
    next_op: u64,
    next_ordinal: u64,
    index: Vec<SerJournalIndexEntry>,
    entries: Vec<SerJournalEntry>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerEntityAttributes {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    name: Option<String>,
    /// sRGB channels, each in `[0, 1]`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    color: Option<(f64, f64, f64)>,
}

/// Attributes of exported entities, by dense local index. Additive:
/// absent when nothing in the document is attributed.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerAttributes {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    solids: Vec<(usize, SerEntityAttributes)>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    faces: Vec<(usize, SerEntityAttributes)>,
}

/// Version 3 moves boundary authority into Loop/Coedge records. Version 4
/// adds standalone sheet roots; version 5 adds standalone wire roots.
/// Released versions are read forever;
/// incompatible additions require a new version and dedicated parser.
const FORMAT_VERSION: u32 = 3;
const SHEET_ROOT_FORMAT_VERSION: u32 = 4;
const WIRE_ROOT_FORMAT_VERSION: u32 = 5;
const LEGACY_MULTI_ROOT_VERSION: u32 = 2;
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    journal: Option<SerJournal>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    attributes: Option<SerAttributes>,
}

/// Version 3 moves face-boundary and pcurve authority into serialized
/// Loop/Coedge records while retaining wire tables as the compatibility view.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerializedDocumentV3 {
    version: u32,
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    solids: Vec<SerSolid>,
    solid_roots: Vec<usize>,
    compounds: Vec<SerCompound>,
    boundary_authority: SerBoundaryAuthority,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    journal: Option<SerJournal>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    attributes: Option<SerAttributes>,
}

/// Version 4 adds standalone sheet roots that reference the dense shell table.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerializedDocumentV4 {
    version: u32,
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    solids: Vec<SerSolid>,
    solid_roots: Vec<usize>,
    sheet_roots: Vec<usize>,
    compounds: Vec<SerCompound>,
    boundary_authority: SerBoundaryAuthority,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    journal: Option<SerJournal>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    attributes: Option<SerAttributes>,
}

/// Version 5 adds standalone wire roots that reference the dense wire table.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerializedDocumentV5 {
    version: u32,
    vertices: Vec<SerVertex>,
    edges: Vec<SerEdge>,
    wires: Vec<SerWire>,
    faces: Vec<SerFace>,
    shells: Vec<SerShell>,
    solids: Vec<SerSolid>,
    solid_roots: Vec<usize>,
    sheet_roots: Vec<usize>,
    wire_roots: Vec<usize>,
    compounds: Vec<SerCompound>,
    boundary_authority: SerBoundaryAuthority,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    journal: Option<SerJournal>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    attributes: Option<SerAttributes>,
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
    sheet_roots: Vec<usize>,
    wire_roots: Vec<usize>,
    compounds: Vec<SerCompound>,
    pcurves: Vec<SerPCurve>,
    boundary_authority: Option<SerBoundaryAuthority>,
    journal: Option<SerJournal>,
    attributes: Option<SerAttributes>,
}

/// Fresh topology roots reconstructed from an arena document.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeserializedDocument {
    /// Explicit solid roots, preserving document order and duplicates.
    pub solids: Vec<SolidId>,
    /// Explicit sheet roots, preserving document order and duplicates.
    pub sheets: Vec<ShellId>,
    /// Explicit wire roots, preserving document order and duplicates.
    pub wires: Vec<WireId>,
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
    loops: Vec<SerLoopAuthority>,
    coedges: Vec<SerCoedgeAuthority>,
    face_loop_authority: Vec<SerFaceLoopAuthority>,
    shells: Vec<SerShell>,
    vertex_map: HashMap<usize, usize>,
    edge_map: HashMap<usize, usize>,
    wire_map: HashMap<usize, usize>,
    face_map: HashMap<usize, usize>,
    loop_map: HashMap<usize, usize>,
    coedge_map: HashMap<usize, usize>,
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
            loops: Vec::new(),
            coedges: Vec::new(),
            face_loop_authority: Vec::new(),
            shells: Vec::new(),
            vertex_map: HashMap::new(),
            edge_map: HashMap::new(),
            wire_map: HashMap::new(),
            face_map: HashMap::new(),
            loop_map: HashMap::new(),
            coedge_map: HashMap::new(),
            shell_map: HashMap::new(),
            solids: Vec::new(),
            solid_map: HashMap::new(),
        }
    }

    fn intern_vertex(&mut self, id: remus_topology::vertex::VertexId) -> Result<usize, IoError> {
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
            trim: e.trim(),
        });
        self.edge_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_wire(&mut self, id: remus_topology::wire::WireId) -> Result<usize, IoError> {
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
        let body_class =
            (w.body_class() != BodyClass::Wire).then(|| SerBodyClass::from(w.body_class()));
        self.wires.push(SerWire {
            edges,
            closed,
            body_class,
        });
        self.wire_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_face(&mut self, id: remus_topology::face::FaceId) -> Result<usize, IoError> {
        if let Some(&local) = self.face_map.get(&id.index()) {
            return Ok(local);
        }
        remus_topology::validation::validate_face_loops(self.topo, id).map_err(|error| {
            IoError::InvalidTopology {
                reason: format!("face {id:?} has inconsistent boundary authority: {error}"),
            }
        })?;
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

        let loop_ids = self
            .topo
            .loops_of_face(id)
            .ok_or_else(|| IoError::InvalidTopology {
                reason: format!("face {id:?} has no authoritative boundary loops"),
            })?;
        let expected = 1 + f.inner_wires().len();
        if loop_ids.len() != expected {
            return Err(IoError::InvalidTopology {
                reason: format!(
                    "face {id:?} has {} authoritative loops but {expected} wire boundaries",
                    loop_ids.len()
                ),
            });
        }
        let mut serialized_loops = Vec::with_capacity(loop_ids.len());
        for &loop_id in loop_ids {
            serialized_loops.push(self.intern_loop(loop_id)?);
        }
        self.face_loop_authority.push(SerFaceLoopAuthority {
            outer_loop: serialized_loops[0],
            inner_loops: serialized_loops[1..].to_vec(),
        });
        Ok(local)
    }

    fn intern_loop(&mut self, id: remus_topology::face_loop::LoopId) -> Result<usize, IoError> {
        if let Some(&local) = self.loop_map.get(&id.index()) {
            return Ok(local);
        }
        let boundary_loop = self.topo.face_loop(id)?;
        let coedge_ids = boundary_loop.coedges().to_vec();
        let closed = boundary_loop.is_closed();
        let mut coedges = Vec::with_capacity(coedge_ids.len());
        for coedge in coedge_ids {
            coedges.push(self.intern_coedge(coedge)?);
        }
        let local = self.loops.len();
        self.loops.push(SerLoopAuthority { coedges, closed });
        self.loop_map.insert(id.index(), local);
        Ok(local)
    }

    fn intern_coedge(&mut self, id: remus_topology::coedge::CoedgeId) -> Result<usize, IoError> {
        if let Some(&local) = self.coedge_map.get(&id.index()) {
            return Ok(local);
        }
        let coedge = self.topo.coedge(id)?;
        let edge = self.intern_edge(coedge.edge())?;
        let pcurve = coedge
            .pcurve()
            .map(|pcurve| {
                validate_arena_pcurve(
                    pcurve.curve(),
                    pcurve.t_start(),
                    pcurve.t_end(),
                    &format!("coedge {id:?} pcurve"),
                )?;
                Ok::<SerUsePCurve, IoError>(SerUsePCurve {
                    curve: pcurve.curve().clone(),
                    t_start: pcurve.t_start(),
                    t_end: pcurve.t_end(),
                })
            })
            .transpose()?;
        let local = self.coedges.len();
        self.coedges.push(SerCoedgeAuthority {
            edge,
            forward: coedge.is_forward(),
            periodic_winding: [coedge.periodic_winding().u(), coedge.periodic_winding().v()],
            pcurve,
        });
        self.coedge_map.insert(id.index(), local);
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
        let body_class =
            (s.body_class() != BodyClass::Solid).then(|| SerBodyClass::from(s.body_class()));
        self.shells.push(SerShell { faces, body_class });
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

/// Serializes explicit solid roots into a version 3 arena document.
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

/// Serializes solid and compound roots into one version 3 arena document.
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

    let journal = serialize_journal(topo, &builder);
    let attributes = serialize_attributes(topo, &builder);
    let dump = SerializedDocumentV3 {
        version: FORMAT_VERSION,
        vertices: builder.vertices,
        edges: builder.edges,
        wires: builder.wires,
        faces: builder.faces,
        shells: builder.shells,
        solids: builder.solids,
        solid_roots,
        compounds,
        boundary_authority: SerBoundaryAuthority {
            loops: builder.loops,
            coedges: builder.coedges,
            faces: builder.face_loop_authority,
        },
        journal,
        attributes,
    };

    serde_json::to_vec(&dump).map_err(|e| IoError::ParseError {
        reason: format!("arena serialization failed: {e}"),
    })
}

/// Serializes one standalone sheet root into a version 4 arena document.
///
/// # Errors
///
/// Returns [`IoError`] if the shell is missing, is not tagged as a sheet, any
/// referenced entity is missing, or JSON serialization fails.
pub fn serialize_sheet(topo: &Topology, sheet_id: ShellId) -> Result<Vec<u8>, IoError> {
    serialize_sheets(topo, &[sheet_id])
}

/// Serializes standalone sheet roots into a version 4 arena document.
///
/// Shared topology is emitted once using dense local indices. Root order and
/// duplicate roots are preserved.
///
/// # Errors
///
/// Returns [`IoError`] if a shell is missing, is not tagged as a sheet, any
/// referenced entity is missing, or JSON serialization fails.
pub fn serialize_sheets(topo: &Topology, sheet_ids: &[ShellId]) -> Result<Vec<u8>, IoError> {
    serialize_body_document(topo, &[], sheet_ids, &[])
}

/// Serializes one standalone wire root into a version 5 arena document.
///
/// # Errors
///
/// Returns [`IoError`] if the wire is missing, has an invalid body class, any
/// referenced entity is missing, or JSON serialization fails.
pub fn serialize_wire(topo: &Topology, wire_id: WireId) -> Result<Vec<u8>, IoError> {
    serialize_wires(topo, &[wire_id])
}

/// Serializes standalone wire roots into a version 5 arena document.
///
/// Shared topology is emitted once with dense local indices. Input order and
/// duplicate roots are preserved.
///
/// # Errors
///
/// Returns [`IoError`] if a wire is missing, has an invalid body class, any
/// referenced entity is missing, or JSON serialization fails.
pub fn serialize_wires(topo: &Topology, wire_ids: &[WireId]) -> Result<Vec<u8>, IoError> {
    serialize_body_document_with_wires(topo, &[], &[], wire_ids, &[])
}

/// Serializes solid, sheet, and compound roots into a version 4 arena document.
///
/// Sheet roots reference the document's dense shell table directly. Existing
/// solid-only writers intentionally remain on the frozen version 3 schema so
/// their output stays byte-for-byte stable.
///
/// # Errors
///
/// Returns [`IoError`] if any root or referenced entity is missing, a sheet
/// root is not tagged as a sheet, or JSON serialization fails.
pub fn serialize_body_document(
    topo: &Topology,
    solid_ids: &[SolidId],
    sheet_ids: &[ShellId],
    compound_ids: &[CompoundId],
) -> Result<Vec<u8>, IoError> {
    let mut builder = Builder::new(topo);
    let mut solid_roots = Vec::with_capacity(solid_ids.len());
    for &solid in solid_ids {
        solid_roots.push(builder.intern_solid(solid)?);
    }

    let mut sheet_roots = Vec::with_capacity(sheet_ids.len());
    for &sheet in sheet_ids {
        let actual = builder.topo.shell(sheet)?.body_class();
        if actual != BodyClass::Sheet {
            return Err(remus_topology::TopologyError::BodyClassMismatch {
                entity: "sheet root",
                expected: BodyClass::Sheet.as_str(),
                actual: actual.as_str(),
            }
            .into());
        }
        sheet_roots.push(builder.intern_shell(sheet)?);
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

    let journal = serialize_journal(topo, &builder);
    let attributes = serialize_attributes(topo, &builder);
    let dump = SerializedDocumentV4 {
        version: SHEET_ROOT_FORMAT_VERSION,
        vertices: builder.vertices,
        edges: builder.edges,
        wires: builder.wires,
        faces: builder.faces,
        shells: builder.shells,
        solids: builder.solids,
        solid_roots,
        sheet_roots,
        compounds,
        boundary_authority: SerBoundaryAuthority {
            loops: builder.loops,
            coedges: builder.coedges,
            faces: builder.face_loop_authority,
        },
        journal,
        attributes,
    };

    serde_json::to_vec(&dump).map_err(|e| IoError::ParseError {
        reason: format!("arena serialization failed: {e}"),
    })
}

/// Serializes solid, sheet, wire, and compound roots into a version 5 arena
/// document.
///
/// Wire roots reference the dense wire table directly. The version 3 and 4
/// writers remain separate so their released byte representation stays frozen.
///
/// # Errors
///
/// Returns [`IoError`] if a root is missing, a sheet or wire root has the
/// wrong body class, a referenced entity is missing, or JSON serialization
/// fails.
pub fn serialize_body_document_with_wires(
    topo: &Topology,
    solid_ids: &[SolidId],
    sheet_ids: &[ShellId],
    wire_ids: &[WireId],
    compound_ids: &[CompoundId],
) -> Result<Vec<u8>, IoError> {
    let mut builder = Builder::new(topo);
    let mut solid_roots = Vec::with_capacity(solid_ids.len());
    for &solid in solid_ids {
        solid_roots.push(builder.intern_solid(solid)?);
    }

    let mut sheet_roots = Vec::with_capacity(sheet_ids.len());
    for &sheet in sheet_ids {
        let actual = builder.topo.shell(sheet)?.body_class();
        if actual != BodyClass::Sheet {
            return Err(remus_topology::TopologyError::BodyClassMismatch {
                entity: "sheet root",
                expected: BodyClass::Sheet.as_str(),
                actual: actual.as_str(),
            }
            .into());
        }
        sheet_roots.push(builder.intern_shell(sheet)?);
    }

    let mut wire_roots = Vec::with_capacity(wire_ids.len());
    for &wire in wire_ids {
        let actual = builder.topo.wire(wire)?.body_class();
        if actual != BodyClass::Wire {
            return Err(remus_topology::TopologyError::BodyClassMismatch {
                entity: "wire root",
                expected: BodyClass::Wire.as_str(),
                actual: actual.as_str(),
            }
            .into());
        }
        wire_roots.push(builder.intern_wire(wire)?);
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

    let journal = serialize_journal(topo, &builder);
    let attributes = serialize_attributes(topo, &builder);
    let dump = SerializedDocumentV5 {
        version: WIRE_ROOT_FORMAT_VERSION,
        vertices: builder.vertices,
        edges: builder.edges,
        wires: builder.wires,
        faces: builder.faces,
        shells: builder.shells,
        solids: builder.solids,
        solid_roots,
        sheet_roots,
        wire_roots,
        compounds,
        boundary_authority: SerBoundaryAuthority {
            loops: builder.loops,
            coedges: builder.coedges,
            faces: builder.face_loop_authority,
        },
        journal,
        attributes,
    };

    serde_json::to_vec(&dump).map_err(|e| IoError::ParseError {
        reason: format!("arena serialization failed: {e}"),
    })
}

/// Reconstructs one solid from a version 1, 2, 3, 4, or 5 single-root document.
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
        || !document.sheet_roots.is_empty()
        || !document.wire_roots.is_empty()
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason:
                "single-solid deserialization requires exactly one solid root and no sheet, wire, or compound roots"
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

/// Reconstructs explicit solid roots from a version 1, 2, 3, 4, or 5 document.
///
/// Version 1 input produces a one-element vector. Input must not contain
/// sheet, wire, or compound roots; use [`deserialize_document`] for mixed
/// roots.
/// Every entity receives a fresh topology id.
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, contains sheet, wire, or
/// compound roots, or reconstruction fails.
pub fn deserialize_solids(bytes: &[u8], topo: &mut Topology) -> Result<Vec<SolidId>, IoError> {
    deserialize_solids_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs solid roots with explicit hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or [`IoError::ParseError`] when it contains a non-solid root.
pub fn deserialize_solids_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<SolidId>, IoError> {
    let document = parse_document(bytes, limits)?;
    if !document.sheet_roots.is_empty()
        || !document.wire_roots.is_empty()
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason: "solid-only deserialization does not accept sheet, wire, or compound roots"
                .to_owned(),
        });
    }
    Ok(replay_document(document, topo)?.solids)
}

/// Reconstructs one standalone sheet root from a version 4 or 5 arena document.
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, does not contain exactly
/// one sheet root, contains another root class, or reconstruction fails.
pub fn deserialize_sheet(bytes: &[u8], topo: &mut Topology) -> Result<ShellId, IoError> {
    deserialize_sheet_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs one standalone sheet root with hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or another [`IoError`] when the root contract or reconstruction
/// fails.
pub fn deserialize_sheet_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<ShellId, IoError> {
    let document = parse_document(bytes, limits)?;
    if document.sheet_roots.len() != 1
        || !document.solid_roots.is_empty()
        || !document.wire_roots.is_empty()
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason:
                "single-sheet deserialization requires exactly one sheet root and no solid, wire, or compound roots"
                    .to_owned(),
        });
    }
    let restored = replay_document(document, topo)?;
    restored
        .sheets
        .into_iter()
        .next()
        .ok_or_else(|| index_err("shell", 0))
}

/// Reconstructs standalone sheet roots from a version 4 or 5 arena document.
///
/// Root order and duplicates are preserved. Documents with another root class
/// must be loaded through [`deserialize_document`].
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, contains another root
/// class, or reconstruction fails.
pub fn deserialize_sheets(bytes: &[u8], topo: &mut Topology) -> Result<Vec<ShellId>, IoError> {
    deserialize_sheets_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs standalone sheet roots with hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or [`IoError::ParseError`] when it contains another root class.
pub fn deserialize_sheets_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<ShellId>, IoError> {
    let document = parse_document(bytes, limits)?;
    if !document.solid_roots.is_empty()
        || !document.wire_roots.is_empty()
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason: "sheet-only deserialization does not accept solid, wire, or compound roots"
                .to_owned(),
        });
    }
    Ok(replay_document(document, topo)?.sheets)
}

/// Reconstructs one standalone wire root from a version 5 arena document.
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, does not contain exactly
/// one wire root, contains another root class, or reconstruction fails.
pub fn deserialize_wire(bytes: &[u8], topo: &mut Topology) -> Result<WireId, IoError> {
    deserialize_wire_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs one standalone wire root with hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or another [`IoError`] when the root contract or reconstruction
/// fails.
pub fn deserialize_wire_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<WireId, IoError> {
    let document = parse_document(bytes, limits)?;
    if document.wire_roots.len() != 1
        || !document.solid_roots.is_empty()
        || !document.sheet_roots.is_empty()
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason:
                "single-wire deserialization requires exactly one wire root and no solid, sheet, or compound roots"
                    .to_owned(),
        });
    }
    let restored = replay_document(document, topo)?;
    restored
        .wires
        .into_iter()
        .next()
        .ok_or_else(|| index_err("wire", 0))
}

/// Reconstructs standalone wire roots from a version 5 arena document.
///
/// Root order and duplicates are preserved. Documents with another root class
/// must be loaded through [`deserialize_document`].
///
/// # Errors
///
/// Returns [`IoError`] if the document is malformed, contains another root
/// class, or reconstruction fails.
pub fn deserialize_wires(bytes: &[u8], topo: &mut Topology) -> Result<Vec<WireId>, IoError> {
    deserialize_wires_with_limits(bytes, topo, ImportLimits::default())
}

/// Reconstructs standalone wire roots with hostile-input resource limits.
///
/// # Errors
///
/// Returns [`IoError::LimitExceeded`] when the document exceeds a configured
/// budget, or [`IoError::ParseError`] when it contains another root class.
pub fn deserialize_wires_with_limits(
    bytes: &[u8],
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<Vec<WireId>, IoError> {
    let document = parse_document(bytes, limits)?;
    if !document.solid_roots.is_empty()
        || !document.sheet_roots.is_empty()
        || !document.compounds.is_empty()
    {
        return Err(IoError::ParseError {
            reason: "wire-only deserialization does not accept solid, sheet, or compound roots"
                .to_owned(),
        });
    }
    Ok(replay_document(document, topo)?.wires)
}

/// Reconstructs solid, sheet, wire, and compound roots from a version 1, 2, 3,
/// 4, or 5 arena document.
///
/// Version 1 input is represented as one solid root and no other root classes.
/// All restored topology entities receive fresh ids.
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

/// Reconstructs solid, sheet, wire, and compound roots with explicit
/// hostile-input limits.
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
                sheet_roots: Vec::new(),
                wire_roots: Vec::new(),
                compounds: Vec::new(),
                pcurves: dump.pcurves,
                boundary_authority: None,
                journal: None,
                attributes: None,
            }
        }
        LEGACY_MULTI_ROOT_VERSION => {
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
                sheet_roots: Vec::new(),
                wire_roots: Vec::new(),
                compounds: dump.compounds,
                pcurves: dump.pcurves,
                boundary_authority: None,
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        FORMAT_VERSION => {
            let dump: SerializedDocumentV3 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v3 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: Vec::new(),
                wire_roots: Vec::new(),
                compounds: dump.compounds,
                pcurves: Vec::new(),
                boundary_authority: Some(dump.boundary_authority),
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        SHEET_ROOT_FORMAT_VERSION => {
            let dump: SerializedDocumentV4 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v4 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: dump.sheet_roots,
                wire_roots: Vec::new(),
                compounds: dump.compounds,
                pcurves: Vec::new(),
                boundary_authority: Some(dump.boundary_authority),
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        WIRE_ROOT_FORMAT_VERSION => {
            let dump: SerializedDocumentV5 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v5 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: dump.sheet_roots,
                wire_roots: dump.wire_roots,
                compounds: dump.compounds,
                pcurves: Vec::new(),
                boundary_authority: Some(dump.boundary_authority),
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        version => {
            return Err(IoError::ParseError {
                reason: format!(
                    "unsupported arena dump version {version} (supported: {LEGACY_SINGLE_SOLID_VERSION}, {LEGACY_MULTI_ROOT_VERSION}, {FORMAT_VERSION}, {SHEET_ROOT_FORMAT_VERSION}, {WIRE_ROOT_FORMAT_VERSION})"
                ),
            });
        }
    };
    check_document_limits(&document, limits)?;
    Ok(document)
}

fn check_document_limits(document: &ParsedDocument, limits: ImportLimits) -> Result<(), IoError> {
    let (loop_count, coedge_count, face_authority_count) = document
        .boundary_authority
        .as_ref()
        .map_or((0, 0, 0), |authority| {
            (
                authority.loops.len(),
                authority.coedges.len(),
                authority.faces.len(),
            )
        });
    let loop_coedge_refs = checked_reference_count(
        "arena loop coedge references",
        document.boundary_authority.iter().flat_map(|authority| {
            authority
                .loops
                .iter()
                .map(|boundary_loop| boundary_loop.coedges.len())
        }),
        limits.max_model_entities,
    )?;
    let face_loop_refs = checked_reference_count(
        "arena face loop references",
        document.boundary_authority.iter().flat_map(|authority| {
            authority
                .faces
                .iter()
                .map(|face| face.inner_loops.len().saturating_add(1))
        }),
        limits.max_model_entities,
    )?;
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
        ("arena sheet roots", document.sheet_roots.len()),
        ("arena wire roots", document.wire_roots.len()),
        ("arena compounds", document.compounds.len()),
        ("arena inner shell references", inner_shell_refs),
        ("arena compound solid references", compound_refs),
        ("arena pcurves", document.pcurves.len()),
        ("arena loops", loop_count),
        ("arena coedges", coedge_count),
        ("arena face loop authority", face_authority_count),
        ("arena loop coedge references", loop_coedge_refs),
        ("arena face loop references", face_loop_refs),
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
            loop_count,
            coedge_count,
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
    for &index in &document.sheet_roots {
        if index >= document.shells.len() {
            return Err(index_err("shell", index));
        }
    }
    for &index in &document.wire_roots {
        if index >= document.wires.len() {
            return Err(index_err("wire", index));
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

/// Serializes the topology's evolution journal (RFC 0003, Stage 5),
/// remapping live-index arena keys to this document's dense local
/// indices. Entities outside the document keep their kind with no local
/// index, so anchor output ordering stays replay-stable.
fn serialize_journal(topo: &Topology, builder: &Builder<'_>) -> Option<SerJournal> {
    use remus_topology::journal::{EntityKind, EventSnapshot, PayloadSnapshot};

    if topo.journal().is_empty() {
        return None;
    }
    let snapshot = topo.journal().snapshot();
    let index = snapshot
        .index
        .into_iter()
        .map(|(ordinal, key)| {
            let local = match key.kind {
                EntityKind::Vertex => builder.vertex_map.get(&key.index).copied(),
                EntityKind::Edge => builder.edge_map.get(&key.index).copied(),
                EntityKind::Face => builder.face_map.get(&key.index).copied(),
            };
            SerJournalIndexEntry {
                ordinal,
                kind: key.kind.as_str().to_owned(),
                local,
            }
        })
        .collect();
    let entries = snapshot
        .entries
        .into_iter()
        .map(|entry| SerJournalEntry {
            op: entry.op,
            kind: entry.kind,
            payload: match entry.payload {
                PayloadSnapshot::Evolution {
                    construction,
                    scope,
                    events,
                } => SerJournalPayload::Evolution {
                    construction,
                    scope,
                    events: events
                        .into_iter()
                        .map(|(subject, event)| {
                            let event = match event {
                                EventSnapshot::Preserved { from } => {
                                    SerJournalEvent::Preserved { from }
                                }
                                EventSnapshot::Modified { from } => {
                                    SerJournalEvent::Modified { from }
                                }
                                EventSnapshot::Generated { sources } => {
                                    SerJournalEvent::Generated { sources }
                                }
                                EventSnapshot::Merged { from } => SerJournalEvent::Merged { from },
                                EventSnapshot::Deleted => SerJournalEvent::Deleted,
                                EventSnapshot::Unresolved { candidates } => {
                                    SerJournalEvent::Unresolved { candidates }
                                }
                            };
                            (subject, event)
                        })
                        .collect(),
                },
                PayloadSnapshot::Barrier { affected } => SerJournalPayload::Barrier { affected },
                PayloadSnapshot::GlobalBarrier => SerJournalPayload::GlobalBarrier,
            },
        })
        .collect();
    Some(SerJournal {
        next_op: snapshot.next_op,
        next_ordinal: snapshot.next_ordinal,
        index,
        entries,
    })
}

/// Serializes attributes of the document's solids and faces, by dense
/// local index in deterministic order.
fn serialize_attributes(topo: &Topology, builder: &Builder<'_>) -> Option<SerAttributes> {
    let encode = |attributes: &remus_topology::attributes::EntityAttributes| SerEntityAttributes {
        name: attributes.name.clone(),
        color: attributes.color.map(|c| (c.r(), c.g(), c.b())),
    };
    let mut faces: Vec<(usize, SerEntityAttributes)> = builder
        .face_map
        .iter()
        .filter_map(|(&arena, &local)| {
            let id = topo.face_id_from_index(arena)?;
            topo.attributes().face(id).map(|a| (local, encode(a)))
        })
        .collect();
    faces.sort_by_key(|(local, _)| *local);
    let mut solids: Vec<(usize, SerEntityAttributes)> = builder
        .solid_map
        .iter()
        .filter_map(|(&arena, &local)| {
            let id = topo.solid_id_from_index(arena)?;
            topo.attributes().solid(id).map(|a| (local, encode(a)))
        })
        .collect();
    solids.sort_by_key(|(local, _)| *local);
    if faces.is_empty() && solids.is_empty() {
        return None;
    }
    Some(SerAttributes { solids, faces })
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
        sheet_roots,
        wire_roots,
        compounds,
        pcurves,
        boundary_authority,
        journal,
        attributes,
    } = document;

    for (index, pcurve) in pcurves.iter().enumerate() {
        validate_arena_pcurve(
            &pcurve.curve,
            pcurve.t_start,
            pcurve.t_end,
            &format!("legacy pcurve {index}"),
        )?;
    }

    for (index, vertex) in vertices.iter().enumerate() {
        validate_arena_point(vertex.point, &format!("vertex {index}"))?;
        validate_arena_tolerance(vertex.tolerance, &format!("vertex {index}"))?;
    }

    let mut vertex_ids = Vec::with_capacity(vertices.len());
    for v in &vertices {
        vertex_ids.push(topo.add_vertex(Vertex::new(v.point, v.tolerance)));
    }

    let mut edge_ids = Vec::with_capacity(edges.len());
    for (index, e) in edges.into_iter().enumerate() {
        let start = *vertex_ids
            .get(e.start)
            .ok_or_else(|| index_err("vertex", e.start))?;
        let end = *vertex_ids
            .get(e.end)
            .ok_or_else(|| index_err("vertex", e.end))?;
        let start_vertex = vertices
            .get(e.start)
            .ok_or_else(|| index_err("vertex", e.start))?;
        let end_vertex = vertices
            .get(e.end)
            .ok_or_else(|| index_err("vertex", e.end))?;
        if let Some(tolerance) = e.tolerance {
            validate_arena_tolerance(tolerance, &format!("edge {index}"))?;
        }
        if let SerEdgeCurve::NurbsCurve(curve) = &e.curve {
            curve.validate().map_err(|err| IoError::ParseError {
                reason: format!("invalid arena NURBS curve: {err}"),
            })?;
        }
        let curve = e.curve.into_curve();
        let trim = legacy_arena_edge_trim_adapter(
            index,
            &curve,
            e.trim,
            start == end,
            start_vertex.point,
            end_vertex.point,
            start_vertex.tolerance,
            end_vertex.tolerance,
            e.tolerance,
        )?;
        edge_ids.push(topo.add_edge({
            let mut restored = Edge::with_tolerance(start, end, curve, e.tolerance);
            restored.set_trim(trim);
            restored
        }));
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
        let body_class = w.body_class.map_or(BodyClass::Wire, BodyClass::from);
        let wire = topo.add_wire(Wire::new(oriented, w.closed)?);
        topo.set_wire_body_class(wire, body_class)?;
        wire_ids.push(wire);
    }

    let restored_wires = wire_roots
        .into_iter()
        .map(|index| {
            let wire = wire_ids
                .get(index)
                .copied()
                .ok_or_else(|| index_err("wire", index))?;
            let actual = topo.wire(wire)?.body_class();
            if actual != BodyClass::Wire {
                return Err(IoError::from(
                    remus_topology::TopologyError::BodyClassMismatch {
                        entity: "wire root",
                        expected: BodyClass::Wire.as_str(),
                        actual: actual.as_str(),
                    },
                ));
            }
            Ok(wire)
        })
        .collect::<Result<Vec<_>, IoError>>()?;

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

    if let Some(authority) = boundary_authority {
        replay_boundary_authority(authority, topo, &edge_ids, &face_ids)?;
    }

    let mut shell_ids = Vec::with_capacity(shells.len());
    for s in shells {
        let mut faces = Vec::with_capacity(s.faces.len());
        for fid in s.faces {
            faces.push(*face_ids.get(fid).ok_or_else(|| index_err("face", fid))?);
        }
        let body_class = s.body_class.map_or(BodyClass::Solid, BodyClass::from);
        let shell = if faces.is_empty() {
            Shell::empty()
        } else {
            Shell::new(faces)?
        };
        let shell = topo.add_shell(shell);
        topo.set_shell_body_class(shell, body_class)?;
        shell_ids.push(shell);
    }

    let restored_sheets = sheet_roots
        .into_iter()
        .map(|index| {
            let sheet = shell_ids
                .get(index)
                .copied()
                .ok_or_else(|| index_err("shell", index))?;
            let actual = topo.shell(sheet)?.body_class();
            if actual != BodyClass::Sheet {
                return Err(IoError::from(
                    remus_topology::TopologyError::BodyClassMismatch {
                        entity: "sheet root",
                        expected: BodyClass::Sheet.as_str(),
                        actual: actual.as_str(),
                    },
                ));
            }
            Ok(sheet)
        })
        .collect::<Result<Vec<_>, IoError>>()?;

    for pc in pcurves {
        let edge = *edge_ids
            .get(pc.edge)
            .ok_or_else(|| index_err("edge", pc.edge))?;
        let face = *face_ids
            .get(pc.face)
            .ok_or_else(|| index_err("face", pc.face))?;
        let pcurve = PCurve::new(pc.curve, pc.t_start, pc.t_end);
        match pc.forward {
            Some(forward) => topo.set_pcurve_oriented(edge, face, forward, pcurve)?,
            // Pre-orientation document: resolve the use from the wires.
            // Such files cannot hold two branches for one (edge, face), so
            // the resolved set cannot be ambiguous unless the file also has
            // a seam wire; attach to the forward branch in that case.
            None => {
                if topo.set_pcurve(edge, face, pcurve.clone()).is_err() {
                    topo.set_pcurve_oriented(edge, face, true, pcurve)?;
                }
            }
        }
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
        for shell in std::iter::once(outer).chain(inner.iter().copied()) {
            let actual = topo.shell(shell)?.body_class();
            if actual != BodyClass::Solid {
                return Err(remus_topology::TopologyError::BodyClassMismatch {
                    entity: "solid shell",
                    expected: BodyClass::Solid.as_str(),
                    actual: actual.as_str(),
                }
                .into());
            }
        }
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

    if let Some(attributes) = attributes {
        for (local, encoded) in attributes.faces {
            let face = *face_ids
                .get(local)
                .ok_or_else(|| index_err("face", local))?;
            topo.set_face_attributes(face, decode_attributes(encoded)?)
                .map_err(IoError::from)?;
        }
        for (local, encoded) in attributes.solids {
            let solid = *solid_ids
                .get(local)
                .ok_or_else(|| index_err("solid", local))?;
            topo.set_solid_attributes(solid, decode_attributes(encoded)?)
                .map_err(IoError::from)?;
        }
    }

    if let Some(journal) = journal {
        let restored = restore_journal(journal, &vertex_ids, &edge_ids, &face_ids)?;
        topo.load_journal(restored);
    }

    Ok(DeserializedDocument {
        solids: restored_solids,
        sheets: restored_sheets,
        wires: restored_wires,
        compounds: restored_compounds,
    })
}

fn replay_boundary_authority(
    authority: SerBoundaryAuthority,
    topo: &mut Topology,
    edge_ids: &[EdgeId],
    face_ids: &[remus_topology::face::FaceId],
) -> Result<(), IoError> {
    if authority.faces.len() != face_ids.len() {
        return Err(IoError::ParseError {
            reason: format!(
                "arena v3 has {} face boundary records for {} faces",
                authority.faces.len(),
                face_ids.len()
            ),
        });
    }

    let mut restored_loops = vec![None; authority.loops.len()];
    let mut restored_coedges = vec![None; authority.coedges.len()];
    for (face_index, face_authority) in authority.faces.iter().enumerate() {
        let face_id = face_ids[face_index];
        let actual_loops = topo
            .loops_of_face(face_id)
            .ok_or_else(|| IoError::ParseError {
                reason: format!("arena v3 face {face_index} has no restored boundary loops"),
            })?
            .to_vec();
        let mut serialized_loops = Vec::with_capacity(1 + face_authority.inner_loops.len());
        serialized_loops.push(face_authority.outer_loop);
        serialized_loops.extend(face_authority.inner_loops.iter().copied());
        if actual_loops.len() != serialized_loops.len() {
            return Err(IoError::ParseError {
                reason: format!(
                    "arena v3 face {face_index} declares {} loops but its wire facade restores {}",
                    serialized_loops.len(),
                    actual_loops.len()
                ),
            });
        }

        for (serialized_loop_index, actual_loop_id) in
            serialized_loops.into_iter().zip(actual_loops)
        {
            let serialized_loop = authority
                .loops
                .get(serialized_loop_index)
                .ok_or_else(|| index_err("loop", serialized_loop_index))?;
            if restored_loops[serialized_loop_index]
                .replace(actual_loop_id)
                .is_some()
            {
                return Err(IoError::ParseError {
                    reason: format!(
                        "arena v3 loop index {serialized_loop_index} is used by more than one face boundary"
                    ),
                });
            }
            let actual_loop = topo.face_loop(actual_loop_id)?;
            if actual_loop.face() != face_id || actual_loop.is_closed() != serialized_loop.closed {
                return Err(IoError::ParseError {
                    reason: format!(
                        "arena v3 loop index {serialized_loop_index} disagrees with its restored face or closure"
                    ),
                });
            }
            let actual_coedges = actual_loop.coedges().to_vec();
            if actual_coedges.len() != serialized_loop.coedges.len() {
                return Err(IoError::ParseError {
                    reason: format!(
                        "arena v3 loop index {serialized_loop_index} declares {} coedges but its wire facade restores {}",
                        serialized_loop.coedges.len(),
                        actual_coedges.len()
                    ),
                });
            }

            for (&serialized_coedge_index, actual_coedge_id) in
                serialized_loop.coedges.iter().zip(actual_coedges)
            {
                let serialized_coedge = authority
                    .coedges
                    .get(serialized_coedge_index)
                    .ok_or_else(|| index_err("coedge", serialized_coedge_index))?;
                if restored_coedges[serialized_coedge_index]
                    .replace(actual_coedge_id)
                    .is_some()
                {
                    return Err(IoError::ParseError {
                        reason: format!(
                            "arena v3 coedge index {serialized_coedge_index} is used by more than one loop"
                        ),
                    });
                }
                let expected_edge = *edge_ids
                    .get(serialized_coedge.edge)
                    .ok_or_else(|| index_err("edge", serialized_coedge.edge))?;
                let actual_coedge = topo.coedge(actual_coedge_id)?;
                if actual_coedge.edge() != expected_edge
                    || actual_coedge.is_forward() != serialized_coedge.forward
                    || actual_coedge.parent_loop() != actual_loop_id
                {
                    return Err(IoError::ParseError {
                        reason: format!(
                            "arena v3 coedge index {serialized_coedge_index} disagrees with its restored wire use"
                        ),
                    });
                }
                topo.set_coedge_periodic_winding(
                    actual_coedge_id,
                    remus_topology::PeriodicWinding::new(
                        serialized_coedge.periodic_winding[0],
                        serialized_coedge.periodic_winding[1],
                    ),
                )?;
                if let Some(pcurve) = &serialized_coedge.pcurve {
                    validate_arena_pcurve(
                        &pcurve.curve,
                        pcurve.t_start,
                        pcurve.t_end,
                        &format!("arena v3 coedge index {serialized_coedge_index} pcurve"),
                    )?;
                    topo.set_coedge_pcurve(
                        actual_coedge_id,
                        PCurve::new(pcurve.curve.clone(), pcurve.t_start, pcurve.t_end),
                    )?;
                }
            }
        }
    }

    if let Some(index) = restored_loops.iter().position(Option::is_none) {
        return Err(IoError::ParseError {
            reason: format!("arena v3 loop index {index} is not owned by any face"),
        });
    }
    if let Some(index) = restored_coedges.iter().position(Option::is_none) {
        return Err(IoError::ParseError {
            reason: format!("arena v3 coedge index {index} is not owned by any loop"),
        });
    }
    Ok(())
}

fn validate_arena_pcurve(
    curve: &Curve2D,
    t_start: f64,
    t_end: f64,
    context: &str,
) -> Result<(), IoError> {
    let finite_point =
        |point: remus_math::vec::Point2| point.0.iter().all(|value| value.is_finite());
    let finite_vec = |vector: remus_math::vec::Vec2| vector.0.iter().all(|value| value.is_finite());
    let valid_definition = match curve {
        Curve2D::Line(line) => {
            finite_point(line.origin())
                && finite_vec(line.direction())
                && line.direction().length_squared().is_finite()
                && line.direction().length_squared() > 0.0
        }
        Curve2D::Circle(circle) => {
            finite_point(circle.center()) && circle.radius().is_finite() && circle.radius() > 0.0
        }
        Curve2D::Ellipse(ellipse) => {
            finite_point(ellipse.center())
                && ellipse.semi_major().is_finite()
                && ellipse.semi_major() > 0.0
                && ellipse.semi_minor().is_finite()
                && ellipse.semi_minor() > 0.0
                && ellipse.semi_minor() <= ellipse.semi_major()
                && ellipse.rotation().is_finite()
        }
        Curve2D::Nurbs(nurbs) => {
            let control_points = nurbs.control_points();
            let knots = nurbs.knots();
            let weights = nurbs.weights();
            !control_points.is_empty()
                && nurbs.degree() < control_points.len()
                && knots.len()
                    == control_points
                        .len()
                        .saturating_add(nurbs.degree())
                        .saturating_add(1)
                && weights.len() == control_points.len()
                && control_points.iter().copied().all(finite_point)
                && knots.iter().all(|value| value.is_finite())
                && knots.windows(2).all(|pair| pair[0] <= pair[1])
                && weights
                    .iter()
                    .all(|value| value.is_finite() && *value > 0.0)
                && {
                    let domain = nurbs.domain();
                    let lower = t_start.min(t_end);
                    let upper = t_start.max(t_end);
                    domain.0.is_finite()
                        && domain.1.is_finite()
                        && domain.0 < domain.1
                        && lower >= domain.0
                        && upper <= domain.1
                }
        }
    };
    if !t_start.is_finite() || !t_end.is_finite() || !valid_definition {
        return Err(IoError::ParseError {
            reason: format!("{context} has an invalid definition or parameter range"),
        });
    }
    let midpoint = 0.5f64.mul_add(t_end - t_start, t_start);
    if [t_start, midpoint, t_end].into_iter().any(|parameter| {
        !finite_point(curve.evaluate(parameter)) || !finite_vec(curve.tangent(parameter))
    }) {
        return Err(IoError::ParseError {
            reason: format!("{context} evaluates to non-finite geometry"),
        });
    }
    Ok(())
}

/// Import-only compatibility boundary for arena documents written before
/// non-linear edges carried explicit parameter authority.
#[allow(clippy::too_many_arguments)]
fn legacy_arena_edge_trim_adapter(
    edge_index: usize,
    curve: &EdgeCurve,
    stored_trim: Option<(f64, f64)>,
    closed: bool,
    start: Point3,
    end: Point3,
    start_tolerance: f64,
    end_tolerance: f64,
    edge_tolerance: Option<f64>,
) -> Result<Option<(f64, f64)>, IoError> {
    if matches!(curve, EdgeCurve::Line) {
        if let Some((trim_start, trim_end)) = stored_trim
            && (!trim_start.is_finite() || !trim_end.is_finite())
        {
            return Err(arena_edge_error(
                edge_index,
                "a line carried a non-finite legacy trim",
            ));
        }
        return Ok(None);
    }

    let tolerance = edge_tolerance.unwrap_or_else(|| start_tolerance.max(end_tolerance));
    let trim = match stored_trim {
        Some(trim) => trim,
        None => reconstruct_legacy_arena_trim(edge_index, curve, closed, start, end, tolerance)?,
    };

    let mut scratch = Topology::new();
    let scratch_start = scratch.add_vertex(Vertex::new(start, start_tolerance));
    let scratch_end = if closed {
        scratch_start
    } else {
        scratch.add_vertex(Vertex::new(end, end_tolerance))
    };
    let mut candidate =
        Edge::with_tolerance(scratch_start, scratch_end, curve.clone(), edge_tolerance);
    candidate.set_trim(Some(trim));
    let authority = candidate.strict_domain().map_err(|error| {
        arena_edge_error(
            edge_index,
            &format!("invalid parameter authority {trim:?}: {error}"),
        )
    })?;
    certify_arena_edge_authority(edge_index, curve, authority, start, end, tolerance)?;
    Ok(Some(authority))
}

fn reconstruct_legacy_arena_trim(
    edge_index: usize,
    curve: &EdgeCurve,
    closed: bool,
    start: Point3,
    end: Point3,
    tolerance: f64,
) -> Result<(f64, f64), IoError> {
    let trim = match curve {
        EdgeCurve::Line => return Ok((0.0, 1.0)),
        EdgeCurve::Circle(circle) => {
            legacy_periodic_trim(circle.project(start), circle.project(end), closed)
        }
        EdgeCurve::Ellipse(ellipse) => {
            legacy_periodic_trim(ellipse.project(start), ellipse.project(end), closed)
        }
        EdgeCurve::Hyperbola(hyperbola) => (hyperbola.project(start), hyperbola.project(end)),
        EdgeCurve::Parabola(parabola) => (parabola.project(start), parabola.project(end)),
        EdgeCurve::NurbsCurve(nurbs) => {
            legacy_nurbs_trim(edge_index, nurbs, closed, start, end, tolerance)?
        }
    };
    if !trim.0.is_finite() || !trim.1.is_finite() {
        return Err(arena_edge_error(
            edge_index,
            "legacy endpoint reconstruction produced a non-finite range",
        ));
    }
    Ok(trim)
}

fn legacy_periodic_trim(start: f64, end: f64, closed: bool) -> (f64, f64) {
    let span = if closed {
        std::f64::consts::TAU
    } else {
        let projected = (end - start).rem_euclid(std::f64::consts::TAU);
        if projected <= 4.0 * f64::EPSILON * std::f64::consts::TAU {
            std::f64::consts::TAU
        } else {
            projected
        }
    };
    (start, start + span)
}

fn legacy_nurbs_trim(
    edge_index: usize,
    curve: &NurbsCurve,
    closed: bool,
    start: Point3,
    end: Point3,
    tolerance: f64,
) -> Result<(f64, f64), IoError> {
    let domain = curve.domain();
    let natural_start = curve.evaluate(domain.0);
    let natural_end = curve.evaluate(domain.1);
    let forward = [
        (natural_start - start).length(),
        (natural_end - end).length(),
    ];
    let reverse = [
        (natural_end - start).length(),
        (natural_start - end).length(),
    ];
    let forward_matches = residuals_within(forward, tolerance);
    let reverse_matches = residuals_within(reverse, tolerance);
    if forward_matches && !reverse_matches {
        return Ok(domain);
    }
    if reverse_matches && !forward_matches {
        return Ok((domain.1, domain.0));
    }
    if closed && forward_matches && reverse_matches {
        return Ok(domain);
    }
    if let (Some(trim_start), Some(trim_end)) = (
        monotone_nurbs_parameter(curve, start, tolerance),
        monotone_nurbs_parameter(curve, end, tolerance),
    ) && trim_start.partial_cmp(&trim_end) != Some(std::cmp::Ordering::Equal)
    {
        return Ok((trim_start, trim_end));
    }
    Err(arena_edge_error(
        edge_index,
        "legacy NURBS endpoints do not uniquely establish a parameter range",
    ))
}

fn monotone_nurbs_parameter(curve: &NurbsCurve, point: Point3, tolerance: f64) -> Option<f64> {
    let first_weight = *curve.weights().first()?;
    if !first_weight.is_finite()
        || first_weight <= 0.0
        || curve
            .weights()
            .iter()
            .any(|weight| weight.to_bits() != first_weight.to_bits())
    {
        return None;
    }
    let coordinates = [
        (|point: Point3| point.x()) as fn(Point3) -> f64,
        (|point: Point3| point.y()) as fn(Point3) -> f64,
        (|point: Point3| point.z()) as fn(Point3) -> f64,
    ];
    let coordinate = coordinates.into_iter().find(|coordinate| {
        let control_points = curve.control_points();
        control_points
            .windows(2)
            .all(|pair| coordinate(pair[1]) > coordinate(pair[0]))
            || control_points
                .windows(2)
                .all(|pair| coordinate(pair[1]) < coordinate(pair[0]))
    })?;
    let (mut low, mut high) = curve.domain();
    let mut low_value = coordinate(curve.evaluate(low));
    let high_value = coordinate(curve.evaluate(high));
    let target = coordinate(point);
    if !low_value.is_finite() || !high_value.is_finite() || !target.is_finite() {
        return None;
    }
    let increasing = high_value > low_value;
    if increasing {
        if target < low_value {
            return (low_value - target <= tolerance).then_some(low);
        }
        if target > high_value {
            return (target - high_value <= tolerance).then_some(high);
        }
    } else {
        if target > low_value {
            return (target - low_value <= tolerance).then_some(low);
        }
        if target < high_value {
            return (high_value - target <= tolerance).then_some(high);
        }
    }
    for _ in 0..96 {
        let middle = f64::midpoint(low, high);
        if middle.to_bits() == low.to_bits() || middle.to_bits() == high.to_bits() {
            break;
        }
        let middle_value = coordinate(curve.evaluate(middle));
        if !middle_value.is_finite() {
            return None;
        }
        if (middle_value < target) == increasing {
            low = middle;
            low_value = middle_value;
        } else {
            high = middle;
        }
    }
    let low_error = (low_value - target).abs();
    let high_error = (coordinate(curve.evaluate(high)) - target).abs();
    Some(if low_error <= high_error { low } else { high })
}

fn residuals_within(residuals: [f64; 2], tolerance: f64) -> bool {
    residuals
        .into_iter()
        .all(|residual| residual.is_finite() && residual <= tolerance)
}

fn certify_arena_edge_authority(
    edge_index: usize,
    curve: &EdgeCurve,
    trim: (f64, f64),
    start: Point3,
    end: Point3,
    tolerance: f64,
) -> Result<(), IoError> {
    for (label, parameter, expected) in [("start", trim.0, start), ("end", trim.1, end)] {
        let actual = curve.evaluate_with_endpoints(parameter, start, end);
        validate_arena_point(actual, &format!("edge {edge_index} {label} evaluation"))?;
        let residual = (actual - expected).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(arena_edge_error(
                edge_index,
                &format!("{label} residual {residual} exceeds effective tolerance {tolerance}"),
            ));
        }
    }
    let midpoint = f64::midpoint(trim.0, trim.1);
    validate_arena_point(
        curve.evaluate_with_endpoints(midpoint, start, end),
        &format!("edge {edge_index} midpoint evaluation"),
    )?;
    let tangent = curve.tangent_with_endpoints(midpoint, start, end);
    if tangent.0.iter().any(|coordinate| !coordinate.is_finite()) {
        return Err(arena_edge_error(
            edge_index,
            "the authoritative range has a non-finite interior tangent",
        ));
    }
    Ok(())
}

fn validate_arena_tolerance(tolerance: f64, subject: &str) -> Result<(), IoError> {
    if !tolerance.is_finite() || tolerance.is_sign_negative() {
        return Err(IoError::ParseError {
            reason: format!("arena {subject} has invalid tolerance {tolerance}"),
        });
    }
    Ok(())
}

fn validate_arena_point(point: Point3, subject: &str) -> Result<(), IoError> {
    if point.0.iter().any(|coordinate| !coordinate.is_finite()) {
        return Err(IoError::ParseError {
            reason: format!("arena {subject} is non-finite"),
        });
    }
    Ok(())
}

fn arena_edge_error(edge_index: usize, reason: &str) -> IoError {
    IoError::ParseError {
        reason: format!("arena edge {edge_index}: {reason}"),
    }
}

fn decode_attributes(
    encoded: SerEntityAttributes,
) -> Result<remus_topology::attributes::EntityAttributes, IoError> {
    let color = encoded
        .color
        .map(|(r, g, b)| remus_topology::attributes::ColorRgb::new(r, g, b))
        .transpose()
        .map_err(IoError::from)?;
    Ok(remus_topology::attributes::EntityAttributes {
        name: encoded.name,
        color,
    })
}

/// Rebuilds the journal from its serialized form, remapping document-local
/// entity indices to the freshly allocated arena ids. Entities the
/// document does not contain keep their kind with the
/// [`EntityKey::UNMAPPED`](remus_topology::journal::EntityKey::UNMAPPED)
/// placeholder, so references to them report not-present instead of
/// binding a stale index.
fn restore_journal(
    encoded: SerJournal,
    vertex_ids: &[remus_topology::VertexId],
    edge_ids: &[EdgeId],
    face_ids: &[remus_topology::FaceId],
) -> Result<remus_topology::journal::Journal, IoError> {
    use remus_topology::journal::{
        EntityKey, EntrySnapshot, EventSnapshot, Journal, JournalSnapshot, PayloadSnapshot,
    };

    let mut index = Vec::with_capacity(encoded.index.len());
    for entry in encoded.index {
        let key = match (entry.kind.as_str(), entry.local) {
            ("vertex", Some(local)) => EntityKey::vertex(
                vertex_ids
                    .get(local)
                    .ok_or_else(|| index_err("vertex", local))?
                    .index(),
            ),
            ("edge", Some(local)) => EntityKey::edge(
                edge_ids
                    .get(local)
                    .ok_or_else(|| index_err("edge", local))?
                    .index(),
            ),
            ("face", Some(local)) => EntityKey::face(
                face_ids
                    .get(local)
                    .ok_or_else(|| index_err("face", local))?
                    .index(),
            ),
            ("vertex", None) => EntityKey::vertex(EntityKey::UNMAPPED),
            ("edge", None) => EntityKey::edge(EntityKey::UNMAPPED),
            ("face", None) => EntityKey::face(EntityKey::UNMAPPED),
            (kind, _) => {
                return Err(IoError::ParseError {
                    reason: format!("journal index entry has unknown entity kind {kind:?}"),
                });
            }
        };
        index.push((entry.ordinal, key));
    }
    let entries = encoded
        .entries
        .into_iter()
        .map(|entry| EntrySnapshot {
            op: entry.op,
            kind: entry.kind,
            payload: match entry.payload {
                SerJournalPayload::Evolution {
                    construction,
                    scope,
                    events,
                } => PayloadSnapshot::Evolution {
                    construction,
                    scope,
                    events: events
                        .into_iter()
                        .map(|(subject, event)| {
                            let event = match event {
                                SerJournalEvent::Preserved { from } => {
                                    EventSnapshot::Preserved { from }
                                }
                                SerJournalEvent::Modified { from } => {
                                    EventSnapshot::Modified { from }
                                }
                                SerJournalEvent::Generated { sources } => {
                                    EventSnapshot::Generated { sources }
                                }
                                SerJournalEvent::Merged { from } => EventSnapshot::Merged { from },
                                SerJournalEvent::Deleted => EventSnapshot::Deleted,
                                SerJournalEvent::Unresolved { candidates } => {
                                    EventSnapshot::Unresolved { candidates }
                                }
                            };
                            (subject, event)
                        })
                        .collect(),
                },
                SerJournalPayload::Barrier { affected } => PayloadSnapshot::Barrier { affected },
                SerJournalPayload::GlobalBarrier => PayloadSnapshot::GlobalBarrier,
            },
        })
        .collect();
    Journal::from_snapshot(JournalSnapshot {
        next_op: encoded.next_op,
        next_ordinal: encoded.next_ordinal,
        index,
        entries,
    })
    .map_err(|error| IoError::ParseError {
        reason: format!("journal restore failed: {error}"),
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
    use remus_math::curves2d::{Circle2D, Line2D};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::vec::{Point2, Vec2};
    use remus_operations::primitives::{make_box, make_cylinder};
    use remus_operations::sew::make_sheet_body;
    use remus_topology::explorer::solid_faces;

    fn single_edge_document(
        curve: SerEdgeCurve,
        start: Point3,
        end: Point3,
        closed: bool,
        tolerance: Option<f64>,
        trim: Option<(f64, f64)>,
    ) -> Vec<u8> {
        let mut vertices = vec![SerVertex {
            point: start,
            tolerance: 1e-9,
        }];
        let end_index = if closed {
            0
        } else {
            vertices.push(SerVertex {
                point: end,
                tolerance: 1e-9,
            });
            1
        };
        serde_json::to_vec(&SerializedDocumentV2 {
            version: LEGACY_MULTI_ROOT_VERSION,
            vertices,
            edges: vec![SerEdge {
                start: 0,
                end: end_index,
                curve,
                tolerance,
                trim,
            }],
            wires: Vec::new(),
            faces: Vec::new(),
            shells: vec![SerShell {
                faces: Vec::new(),
                body_class: None,
            }],
            solids: vec![SerSolid {
                outer_shell: 0,
                inner_shells: Vec::new(),
            }],
            solid_roots: vec![0],
            compounds: Vec::new(),
            pcurves: Vec::new(),
            journal: None,
            attributes: None,
        })
        .unwrap()
    }

    fn restored_edge(bytes: &[u8]) -> (Topology, EdgeId) {
        let mut topology = Topology::new();
        deserialize_solid(bytes, &mut topology).unwrap();
        let edge = topology.edge_id_from_index(0).unwrap();
        (topology, edge)
    }

    fn assert_trim_near(actual: (f64, f64), expected: (f64, f64)) {
        assert!((actual.0 - expected.0).abs() < 1e-12, "{actual:?}");
        assert!((actual.1 - expected.1).abs() < 1e-12, "{actual:?}");
    }

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

    fn cylinder_seam(topo: &Topology, solid: SolidId) -> (remus_topology::face::FaceId, EdgeId) {
        let face = solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let wire = topo.face(face).unwrap().outer_wire();
        let mut counts = std::collections::HashMap::new();
        for oriented in topo.wire(wire).unwrap().edges() {
            *counts.entry(oriented.edge()).or_insert(0_usize) += 1;
        }
        let seam = counts
            .into_iter()
            .find_map(|(edge, count)| (count == 2).then_some(edge))
            .unwrap();
        (face, seam)
    }

    fn seam_branch(u: f64, reverse: bool) -> PCurve {
        let direction = if reverse { -1.0 } else { 1.0 };
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(u, 0.0), Vec2::new(0.0, direction)).unwrap()),
            0.25,
            1.75,
        )
    }

    fn trimmed_nurbs_sheet(topo: &mut Topology) -> ShellId {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![
                    Point3::new(-0.75, -0.25, 0.0),
                    Point3::new(0.75, -0.25, 0.0),
                ],
                vec![Point3::new(-0.75, 0.25, 0.0), Point3::new(0.75, 0.25, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .unwrap();
        let face = remus_topology::builder::make_nurbs_face(topo, surface, 3.25e-8).unwrap();
        let sheet = make_sheet_body(topo, &[face]).unwrap();
        let boundary_loop = topo.face(face).unwrap().outer_loop().unwrap();
        let coedges = topo.face_loop(boundary_loop).unwrap().coedges().to_vec();

        let first_edge = topo.coedge(coedges[0]).unwrap().edge();
        let first_start = topo.edge(first_edge).unwrap().start();
        let first_end = topo.edge(first_edge).unwrap().end();
        let first_start_point = topo.vertex(first_start).unwrap().point();
        let first_end_point = topo.vertex(first_end).unwrap().point();
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 2.0, 2.0],
            vec![first_start_point, first_end_point],
            vec![1.0, 1.0],
        )
        .unwrap();
        let edge = topo.edge_mut(first_edge).unwrap();
        edge.set_curve(EdgeCurve::NurbsCurve(curve));
        edge.set_trim(Some((0.0, 2.0)));

        let pcurves = [
            (Point2::new(0.0, 0.0), Vec2::new(0.5, 0.0), 0.0, 2.0),
            (Point2::new(1.0, 0.0), Vec2::new(0.0, 1.0), 0.0, 1.0),
            (Point2::new(1.0, 1.0), Vec2::new(-1.0, 0.0), 0.0, 1.0),
            (Point2::new(0.0, 1.0), Vec2::new(0.0, -1.0), 0.0, 1.0),
        ];
        for (coedge, (origin, direction, t_start, t_end)) in coedges.into_iter().zip(pcurves) {
            topo.set_coedge_pcurve(
                coedge,
                PCurve::new(
                    Curve2D::Line(Line2D::new(origin, direction).unwrap()),
                    t_start,
                    t_end,
                ),
            )
            .unwrap();
        }
        sheet
    }

    #[test]
    fn roundtrip_box_preserves_counts_and_exact_bits() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();

        let bytes = serialize_solid(&topo, solid).unwrap();
        assert!(
            !std::str::from_utf8(&bytes).unwrap().contains("body_class"),
            "default class tags must remain absent for byte-stable legacy output"
        );
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
    fn v3_empty_solid_bytes_remain_frozen() {
        let mut topo = Topology::new();
        let solid = topo.add_empty_solid();

        let bytes = serialize_solid(&topo, solid).unwrap();

        assert_eq!(
            bytes,
            br#"{"version":3,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[]}],"solids":[{"outer_shell":0,"inner_shells":[]}],"solid_roots":[0],"compounds":[],"boundary_authority":{"loops":[],"coedges":[],"faces":[]}}"#
        );
    }

    #[test]
    fn v4_trimmed_nurbs_sheet_roundtrip_is_exact_and_preserves_duplicate_roots() {
        let mut source = Topology::new();
        let sheet = trimmed_nurbs_sheet(&mut source);

        let bytes = serialize_sheets(&source, &[sheet, sheet]).unwrap();
        let encoded: SerializedDocumentV4 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(encoded.version, SHEET_ROOT_FORMAT_VERSION);
        assert_eq!(encoded.sheet_roots, vec![0, 0]);
        assert!(matches!(
            encoded.shells[0].body_class,
            Some(SerBodyClass::Sheet)
        ));
        assert_eq!(
            encoded
                .boundary_authority
                .coedges
                .iter()
                .filter(|coedge| coedge.pcurve.is_some())
                .count(),
            4
        );
        assert_eq!(encoded.edges[0].trim, Some((0.0, 2.0)));

        let mut destination = Topology::new();
        let restored = deserialize_sheets(&bytes, &mut destination).unwrap();
        assert_eq!(restored.len(), 2);
        assert_eq!(restored[0], restored[1]);
        assert_eq!(
            destination.shell(restored[0]).unwrap().body_class(),
            BodyClass::Sheet
        );
        let face = destination.shell(restored[0]).unwrap().faces()[0];
        assert!(matches!(
            destination.face(face).unwrap().surface(),
            FaceSurface::Nurbs(_)
        ));
        let boundary_loop = destination.face(face).unwrap().outer_loop().unwrap();
        let coedges = destination.face_loop(boundary_loop).unwrap().coedges();
        assert_eq!(coedges.len(), 4);
        assert!(
            coedges
                .iter()
                .all(|&coedge| destination.coedge_pcurve(coedge).unwrap().is_some())
        );

        let rewritten = serialize_sheets(&destination, &restored).unwrap();
        assert_eq!(rewritten, bytes);
    }

    #[test]
    fn additive_sheet_tag_loads_when_the_shell_is_not_a_solid_boundary() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["shells"][0]["body_class"] = serde_json::Value::String("sheet".into());
        document["solids"] = serde_json::Value::Array(Vec::new());
        document["solid_roots"] = serde_json::Value::Array(Vec::new());
        let tagged = serde_json::to_vec(&document).unwrap();

        let mut destination = Topology::new();
        let roots = deserialize_document(&tagged, &mut destination).unwrap();

        assert!(roots.solids.is_empty());
        assert!(roots.sheets.is_empty());
        assert_eq!(
            destination
                .shell(destination.shell_id_from_index(0).unwrap())
                .unwrap()
                .body_class(),
            BodyClass::Sheet
        );
    }

    #[test]
    fn v4_mixed_body_document_restores_each_explicit_root_class() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
        let sheet = trimmed_nurbs_sheet(&mut source);
        let compound = source.add_compound(Compound::new(vec![solid]));
        let bytes = serialize_body_document(&source, &[solid], &[sheet], &[compound]).unwrap();

        let mut destination = Topology::new();
        let roots = deserialize_document(&bytes, &mut destination).unwrap();

        assert_eq!(roots.solids.len(), 1);
        assert_eq!(roots.sheets.len(), 1);
        assert!(roots.wires.is_empty());
        assert_eq!(roots.compounds.len(), 1);
        assert_eq!(
            destination.shell(roots.sheets[0]).unwrap().body_class(),
            BodyClass::Sheet
        );
        assert_eq!(
            destination.compound(roots.compounds[0]).unwrap().solids(),
            roots.solids.as_slice()
        );
    }

    #[test]
    fn v5_wire_roundtrip_is_exact_and_preserves_duplicate_roots() {
        let mut source = Topology::new();
        let wire = remus_topology::builder::make_polygon_wire(
            &mut source,
            &[
                Point3::new(-1.25, -0.75, 0.0),
                Point3::new(1.25, -0.75, 0.0),
                Point3::new(1.25, 0.75, 0.0),
                Point3::new(-1.25, 0.75, 0.0),
            ],
            3.25e-8,
        )
        .unwrap();

        let bytes = serialize_wires(&source, &[wire, wire]).unwrap();
        let encoded: SerializedDocumentV5 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(encoded.version, WIRE_ROOT_FORMAT_VERSION);
        assert_eq!(encoded.wire_roots, vec![0, 0]);
        assert_eq!(encoded.wires.len(), 1);
        assert!(encoded.wires[0].body_class.is_none());

        let mut destination = Topology::new();
        let sentinel =
            remus_topology::builder::make_regular_polygon_wire(&mut destination, 0.25, 3, 1e-7)
                .unwrap();
        let restored = deserialize_wires(&bytes, &mut destination).unwrap();
        assert_eq!(restored.len(), 2);
        assert_eq!(restored[0], restored[1]);
        assert!(restored[0].index() > sentinel.index());
        assert_eq!(
            destination.wire(restored[0]).unwrap().body_class(),
            BodyClass::Wire
        );
        let length = remus_operations::measure::wire_length(&destination, restored[0]).unwrap();
        assert!((length - 8.0).abs() < 1e-12);

        let rewritten = serialize_wires(&destination, &restored).unwrap();
        assert_eq!(rewritten, bytes);
    }

    #[test]
    fn v5_mixed_body_document_restores_wire_root_alongside_existing_classes() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
        let sheet = trimmed_nurbs_sheet(&mut source);
        let wire =
            remus_topology::builder::make_regular_polygon_wire(&mut source, 2.0, 5, 1e-7).unwrap();
        let compound = source.add_compound(Compound::new(vec![solid]));
        let bytes =
            serialize_body_document_with_wires(&source, &[solid], &[sheet], &[wire], &[compound])
                .unwrap();

        let mut destination = Topology::new();
        let roots = deserialize_document(&bytes, &mut destination).unwrap();

        assert_eq!(roots.solids.len(), 1);
        assert_eq!(roots.sheets.len(), 1);
        assert_eq!(roots.wires.len(), 1);
        assert_eq!(roots.compounds.len(), 1);
        assert_eq!(
            destination.wire(roots.wires[0]).unwrap().body_class(),
            BodyClass::Wire
        );
    }

    #[test]
    fn v5_invalid_wire_root_refuses_before_topology_mutation() {
        let mut source = Topology::new();
        let wire =
            remus_topology::builder::make_regular_polygon_wire(&mut source, 1.0, 4, 1e-7).unwrap();
        let bytes = serialize_wire(&source, wire).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["wire_roots"][0] = serde_json::json!(usize::MAX);
        let malformed = serde_json::to_vec(&document).unwrap();
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();

        let error = deserialize_wire(&malformed, &mut destination).unwrap_err();

        assert!(error.to_string().contains("out-of-range wire index"));
        assert_eq!(destination.num_wires(), 0);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn solid_only_loader_refuses_v5_wire_roots_without_mutation() {
        let mut source = Topology::new();
        let wire =
            remus_topology::builder::make_regular_polygon_wire(&mut source, 1.0, 4, 1e-7).unwrap();
        let bytes = serialize_wire(&source, wire).unwrap();
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();

        let error = deserialize_solids(&bytes, &mut destination).unwrap_err();

        assert!(
            error
                .to_string()
                .contains("does not accept sheet, wire, or compound roots")
        );
        assert_eq!(destination.num_wires(), 0);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn v5_wire_root_count_obeys_import_limits_before_mutation() {
        let mut source = Topology::new();
        let edge = remus_topology::builder::make_circle_edge(
            &mut source,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            1e-7,
        )
        .unwrap();
        let wire = source.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let bytes = serialize_wires(&source, &[wire, wire]).unwrap();
        let limits = ImportLimits {
            max_input_bytes: bytes.len(),
            max_model_entities: 1,
            ..ImportLimits::default()
        };
        let mut destination = Topology::new();

        let error = deserialize_wires_with_limits(&bytes, &mut destination, limits).unwrap_err();

        assert!(matches!(
            error,
            IoError::LimitExceeded {
                resource: "arena wire roots",
                actual: 2,
                limit: 1,
            }
        ));
        assert_eq!(destination.num_wires(), 0);
    }

    #[test]
    fn serialize_sheet_refuses_a_solid_class_shell() {
        let mut topo = Topology::new();
        let shell = topo.add_shell(Shell::empty());

        let error = serialize_sheet(&topo, shell).unwrap_err();

        assert!(matches!(
            error,
            IoError::Topology(remus_topology::TopologyError::BodyClassMismatch {
                entity: "sheet root",
                expected: "sheet",
                actual: "solid"
            })
        ));
    }

    #[test]
    fn v4_wrong_sheet_class_refuses_transactionally() {
        let mut source = Topology::new();
        let sheet = trimmed_nurbs_sheet(&mut source);
        let bytes = serialize_sheet(&source, sheet).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["shells"][0]["body_class"] = serde_json::json!("solid");
        let malformed = serde_json::to_vec(&document).unwrap();
        let mut destination = Topology::new();
        destination.add_empty_solid();
        let counts_before = (
            destination.num_vertices(),
            destination.num_edges(),
            destination.num_wires(),
            destination.num_faces(),
            destination.num_shells(),
            destination.num_solids(),
        );

        let error = deserialize_sheet(&malformed, &mut destination).unwrap_err();

        assert!(matches!(
            error,
            IoError::Topology(remus_topology::TopologyError::BodyClassMismatch {
                entity: "sheet root",
                expected: "sheet",
                actual: "solid"
            })
        ));
        assert_eq!(
            (
                destination.num_vertices(),
                destination.num_edges(),
                destination.num_wires(),
                destination.num_faces(),
                destination.num_shells(),
                destination.num_solids(),
            ),
            counts_before
        );
    }

    #[test]
    fn v4_invalid_sheet_root_refuses_before_topology_mutation() {
        let mut source = Topology::new();
        let sheet = trimmed_nurbs_sheet(&mut source);
        let bytes = serialize_sheet(&source, sheet).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["sheet_roots"][0] = serde_json::json!(usize::MAX);
        let malformed = serde_json::to_vec(&document).unwrap();
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();

        let error = deserialize_sheet(&malformed, &mut destination).unwrap_err();

        assert!(error.to_string().contains("out-of-range shell index"));
        assert_eq!(destination.num_shells(), 1);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn solid_only_loader_refuses_v4_sheet_roots_without_mutation() {
        let mut source = Topology::new();
        let sheet = trimmed_nurbs_sheet(&mut source);
        let bytes = serialize_sheet(&source, sheet).unwrap();
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();

        let error = deserialize_solids(&bytes, &mut destination).unwrap_err();

        assert!(
            error
                .to_string()
                .contains("does not accept sheet, wire, or compound roots")
        );
        assert_eq!(destination.num_shells(), 1);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn v4_sheet_root_count_obeys_import_limits_before_mutation() {
        let mut source = Topology::new();
        let sheet = source.add_shell(Shell::empty());
        source
            .set_shell_body_class(sheet, BodyClass::Sheet)
            .unwrap();
        let bytes = serialize_sheets(&source, &[sheet, sheet]).unwrap();
        let limits = ImportLimits {
            max_input_bytes: bytes.len(),
            max_model_entities: 1,
            ..ImportLimits::default()
        };
        let mut destination = Topology::new();

        let error = deserialize_sheets_with_limits(&bytes, &mut destination, limits).unwrap_err();

        assert!(matches!(
            error,
            IoError::LimitExceeded {
                resource: "arena sheet roots",
                actual: 2,
                limit: 1,
            }
        ));
        assert_eq!(destination.num_shells(), 0);
    }

    #[test]
    fn sheet_tagged_solid_boundary_refuses_transactionally() {
        let mut source = Topology::new();
        let solid = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let mut document: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        document["shells"][0]["body_class"] = serde_json::Value::String("sheet".into());
        let tagged = serde_json::to_vec(&document).unwrap();
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();

        let error = deserialize_solid(&tagged, &mut destination).unwrap_err();

        assert!(matches!(
            error,
            IoError::Topology(remus_topology::TopologyError::BodyClassMismatch {
                entity: "solid shell",
                expected: "solid",
                actual: "sheet"
            })
        ));
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn v3_roundtrip_preserves_loop_identity_and_two_seam_pcurve_branches() {
        let mut source = Topology::new();
        let solid = make_cylinder(&mut source, 1.0, 2.0).unwrap();
        let (face, seam) = cylinder_seam(&source, solid);
        source
            .set_pcurve_oriented(seam, face, true, seam_branch(0.0, false))
            .unwrap();
        source
            .set_pcurve_oriented(seam, face, false, seam_branch(std::f64::consts::TAU, true))
            .unwrap();
        let lifted_use = source
            .coedges_of_edge(seam)
            .into_iter()
            .find(|&coedge| !source.coedge(coedge).unwrap().is_forward())
            .unwrap();
        source
            .set_coedge_periodic_winding(lifted_use, remus_topology::PeriodicWinding::new(1, 0))
            .unwrap();

        let bytes = serialize_solid(&source, solid).unwrap();
        let encoded: SerializedDocumentV3 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(encoded.version, FORMAT_VERSION);
        assert_eq!(
            encoded
                .boundary_authority
                .coedges
                .iter()
                .filter(|coedge| coedge.pcurve.is_some())
                .count(),
            2
        );

        let mut restored_topology = Topology::new();
        let restored = deserialize_solid(&bytes, &mut restored_topology).unwrap();
        let (restored_face, restored_seam) = cylinder_seam(&restored_topology, restored);
        let loops = restored_topology.loops_of_face(restored_face).unwrap();
        assert_eq!(loops.len(), 1);
        assert_eq!(
            restored_topology.face(restored_face).unwrap().outer_loop(),
            Some(loops[0])
        );
        let uses: Vec<_> = restored_topology
            .face_loop(loops[0])
            .unwrap()
            .coedges()
            .iter()
            .filter_map(|&coedge_id| {
                let coedge = restored_topology.coedge(coedge_id).unwrap();
                (coedge.edge() == restored_seam).then_some((coedge_id, coedge.is_forward()))
            })
            .collect();
        assert_eq!(uses.len(), 2);
        for (coedge_id, forward) in uses {
            let branch = restored_topology.coedge_pcurve(coedge_id).unwrap().unwrap();
            assert_eq!(branch.t_start().to_bits(), 0.25_f64.to_bits());
            assert_eq!(branch.t_end().to_bits(), 1.75_f64.to_bits());
            let uv = branch.evaluate(1.0);
            let expected_u = if forward { 0.0 } else { std::f64::consts::TAU };
            assert_eq!(uv.x().to_bits(), expected_u.to_bits());
            assert_eq!(
                uv.y().to_bits(),
                if forward { 1.0 } else { -1.0_f64 }.to_bits()
            );
            assert_eq!(
                restored_topology
                    .coedge(coedge_id)
                    .unwrap()
                    .periodic_winding(),
                if forward {
                    remus_topology::PeriodicWinding::ZERO
                } else {
                    remus_topology::PeriodicWinding::new(1, 0)
                }
            );
        }
        assert!(matches!(
            restored_topology.pcurve(restored_seam, restored_face),
            Err(remus_topology::TopologyError::SeamPcurveAmbiguous { .. })
        ));
        let boundary =
            remus_topology::validation::validate_boundary_authority(&restored_topology).unwrap();
        assert_eq!(boundary.faces, restored_topology.num_faces());
        assert_eq!(boundary.loops, restored_topology.num_loops());
        assert_eq!(boundary.coedges, restored_topology.num_coedges());
        assert_eq!(boundary.seam_edges, 1);
        assert_eq!(boundary.stored_seam_branches, 2);
    }

    #[test]
    fn v3_boundary_tamper_is_rejected_before_live_topology_mutation() {
        let mut source = Topology::new();
        let solid = make_cylinder(&mut source, 1.0, 2.0).unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let mut encoded: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        let forward = encoded["boundary_authority"]["coedges"][0]["forward"]
            .as_bool()
            .unwrap();
        encoded["boundary_authority"]["coedges"][0]["forward"] = (!forward).into();
        let tampered = serde_json::to_vec(&encoded).unwrap();

        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let counts = (
            destination.num_vertices(),
            destination.num_edges(),
            destination.num_faces(),
            destination.num_loops(),
            destination.num_coedges(),
            destination.num_solids(),
        );
        let error = deserialize_solid(&tampered, &mut destination).unwrap_err();

        assert!(
            error
                .to_string()
                .contains("disagrees with its restored wire use")
        );
        assert!(destination.solid(sentinel).is_ok());
        assert_eq!(
            (
                destination.num_vertices(),
                destination.num_edges(),
                destination.num_faces(),
                destination.num_loops(),
                destination.num_coedges(),
                destination.num_solids(),
            ),
            counts
        );
    }

    #[test]
    fn v3_invalid_embedded_pcurve_is_rejected_before_live_topology_mutation() {
        let mut source = Topology::new();
        let solid = make_cylinder(&mut source, 1.0, 2.0).unwrap();
        let (face, seam) = cylinder_seam(&source, solid);
        source
            .set_pcurve_oriented(seam, face, true, seam_branch(0.0, false))
            .unwrap();
        let bytes = serialize_solid(&source, solid).unwrap();
        let mut encoded: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        let pcurve = encoded["boundary_authority"]["coedges"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find_map(|coedge| coedge.get_mut("pcurve"))
            .unwrap();
        pcurve["curve"] = serde_json::json!({
            "Circle": { "center": [0.0, 0.0], "radius": -1.0 }
        });
        let tampered = serde_json::to_vec(&encoded).unwrap();

        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let counts = (
            destination.num_vertices(),
            destination.num_edges(),
            destination.num_faces(),
            destination.num_loops(),
            destination.num_coedges(),
            destination.num_solids(),
        );
        let error = deserialize_solid(&tampered, &mut destination).unwrap_err();

        assert!(error.to_string().contains("invalid definition"));
        assert!(destination.solid(sentinel).is_ok());
        assert_eq!(
            (
                destination.num_vertices(),
                destination.num_edges(),
                destination.num_faces(),
                destination.num_loops(),
                destination.num_coedges(),
                destination.num_solids(),
            ),
            counts
        );
    }

    #[test]
    fn v3_export_refuses_an_invalid_embedded_pcurve() {
        let mut source = Topology::new();
        let solid = make_cylinder(&mut source, 1.0, 2.0).unwrap();
        let (face, seam) = cylinder_seam(&source, solid);
        let poisoned = Circle2D::new(Point2::new(0.0, 0.0), f64::NAN).unwrap();
        source
            .set_pcurve_oriented(
                seam,
                face,
                true,
                PCurve::new(Curve2D::Circle(poisoned), 0.0, 1.0),
            )
            .unwrap();

        let error = serialize_solid(&source, solid).unwrap_err();

        assert!(error.to_string().contains("invalid definition"));
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
    fn legacy_arena_reconstructs_every_non_line_curve_class() {
        let circle =
            Circle3D::new(Point3::new(2.0, -1.0, 3.0), Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
        let circle_range = (1.1, 0.35 + std::f64::consts::TAU);
        let bytes = single_edge_document(
            SerEdgeCurve::Circle(circle.clone()),
            circle.evaluate(circle_range.0),
            circle.evaluate(circle_range.1),
            false,
            None,
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_trim_near(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            circle_range,
        );

        let ellipse = Ellipse3D::new(
            Point3::new(-3.0, 2.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
            5.0,
            2.0,
        )
        .unwrap();
        let seam = 1.35;
        let seam_point = ellipse.evaluate(seam);
        let bytes = single_edge_document(
            SerEdgeCurve::Ellipse(ellipse),
            seam_point,
            seam_point,
            true,
            Some(1e-9),
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_trim_near(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            (seam, seam + std::f64::consts::TAU),
        );

        let hyperbola = Hyperbola3D::new(
            Point3::new(1.0, 2.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            1.5,
        )
        .unwrap();
        let hyperbola_range = (-0.8, 1.2);
        let bytes = single_edge_document(
            SerEdgeCurve::Hyperbola(hyperbola.clone()),
            hyperbola.evaluate(hyperbola_range.0),
            hyperbola.evaluate(hyperbola_range.1),
            false,
            None,
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_trim_near(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            hyperbola_range,
        );

        let parabola =
            Parabola3D::new(Point3::new(-2.0, 1.0, 4.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let parabola_range = (3.0, -2.0);
        let bytes = single_edge_document(
            SerEdgeCurve::Parabola(parabola.clone()),
            parabola.evaluate(parabola_range.0),
            parabola.evaluate(parabola_range.1),
            false,
            None,
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_trim_near(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            parabola_range,
        );

        let nurbs = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            vec![1.0; 3],
        )
        .unwrap();
        let nurbs_range = (0.2, 0.8);
        let bytes = single_edge_document(
            SerEdgeCurve::NurbsCurve(nurbs.clone()),
            nurbs.evaluate(nurbs_range.0),
            nurbs.evaluate(nurbs_range.1),
            false,
            Some(1e-9),
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_trim_near(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            nurbs_range,
        );

        // Legacy exporters rounded a natural endpoint independently from the
        // NURBS control point. A monotone carrier still gives one unique
        // branch; clamp only within the declared model tolerance and let the
        // full 3D residual certificate below enforce that bound.
        let rounded_start_range = (nurbs.domain().0, 0.8);
        let rounded_start =
            nurbs.evaluate(rounded_start_range.0) + remus_math::vec::Vec3::new(-5.0e-10, 0.0, 0.0);
        let bytes = single_edge_document(
            SerEdgeCurve::NurbsCurve(nurbs.clone()),
            rounded_start,
            nurbs.evaluate(rounded_start_range.1),
            false,
            Some(1e-7),
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_trim_near(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            rounded_start_range,
        );

        let natural_reverse = (nurbs.domain().1, nurbs.domain().0);
        let bytes = single_edge_document(
            SerEdgeCurve::NurbsCurve(nurbs.clone()),
            nurbs.evaluate(natural_reverse.0),
            nurbs.evaluate(natural_reverse.1),
            false,
            Some(1e-9),
            None,
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_eq!(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            natural_reverse
        );

        let line = single_edge_document(
            SerEdgeCurve::Line,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 3.0),
            false,
            None,
            Some((42.0, -17.0)),
        );
        let (topology, edge) = restored_edge(&line);
        assert_eq!(topology.edge(edge).unwrap().trim(), None);
        assert_eq!(
            topology.edge(edge).unwrap().strict_domain().unwrap(),
            (0.0, 1.0)
        );
    }

    #[test]
    fn explicit_arena_trim_is_certified_before_replay() {
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 3.0).unwrap();
        let trim = (-0.4, 0.7);
        let bytes = single_edge_document(
            SerEdgeCurve::Circle(circle.clone()),
            circle.evaluate(trim.0),
            circle.evaluate(trim.1),
            false,
            Some(1e-9),
            Some(trim),
        );
        let (topology, edge) = restored_edge(&bytes);
        assert_eq!(topology.edge(edge).unwrap().strict_domain().unwrap(), trim);

        let invalid = single_edge_document(
            SerEdgeCurve::Circle(circle.clone()),
            circle.evaluate(trim.0),
            circle.evaluate(trim.1),
            false,
            Some(1e-9),
            Some((0.1, 0.1)),
        );
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        assert!(deserialize_solid(&invalid, &mut destination).is_err());
        assert_eq!(destination.num_edges(), 0);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn ambiguous_legacy_nurbs_and_corrupt_tolerance_rollback() {
        let folded = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
            ],
            vec![1.0; 3],
        )
        .unwrap();
        let ambiguous = single_edge_document(
            SerEdgeCurve::NurbsCurve(folded.clone()),
            folded.evaluate(0.2),
            folded.evaluate(0.4),
            false,
            Some(1e-9),
            None,
        );
        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let error = deserialize_solid(&ambiguous, &mut destination).unwrap_err();
        assert!(error.to_string().contains("do not uniquely establish"));
        assert_eq!(destination.num_edges(), 0);
        assert!(destination.is_empty_solid(sentinel));

        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let mut corrupt: serde_json::Value = serde_json::from_slice(&single_edge_document(
            SerEdgeCurve::Circle(circle.clone()),
            circle.evaluate(0.0),
            circle.evaluate(1.0),
            false,
            None,
            None,
        ))
        .unwrap();
        corrupt["vertices"][1]["tolerance"] = serde_json::json!(-1.0);
        let corrupt = serde_json::to_vec(&corrupt).unwrap();
        let error = deserialize_solid(&corrupt, &mut destination).unwrap_err();
        assert!(error.to_string().contains("invalid tolerance"));
        assert_eq!(destination.num_edges(), 0);
        assert_eq!(destination.num_solids(), 1);
        assert!(destination.is_empty_solid(sentinel));
    }

    #[test]
    fn nonfinite_legacy_line_trim_is_rejected_atomically() {
        let bytes = single_edge_document(
            SerEdgeCurve::Line,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            false,
            None,
            None,
        );
        let mut document = parse_document(&bytes, ImportLimits::default()).unwrap();
        document.edges[0].trim = Some((f64::NAN, 1.0));

        let mut destination = Topology::new();
        let sentinel = destination.add_empty_solid();
        let before = destination.clone();
        let error = replay_document(document, &mut destination).unwrap_err();

        assert!(error.to_string().contains("non-finite legacy trim"));
        assert_eq!(destination.num_vertices(), before.num_vertices());
        assert_eq!(destination.num_edges(), before.num_edges());
        assert_eq!(destination.num_solids(), before.num_solids());
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
    fn deserialize_solid_accepts_single_root_v3_document() {
        let mut source = Topology::new();
        let solid = source.add_empty_solid();
        let bytes = serialize_solids(&source, &[solid]).unwrap();
        let mut destination = Topology::new();

        let restored = deserialize_solid(&bytes, &mut destination).unwrap();

        assert!(destination.is_empty_solid(restored));
    }

    #[test]
    fn v3_multi_solid_roundtrip_preserves_root_order_and_duplicates() {
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
    fn v3_roundtrip_preserves_topology_shared_by_distinct_solids() {
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
    fn v2_mixed_roots_golden_remains_readable_and_rewrites_stably_as_v3() {
        let golden = include_str!("../tests/data/arena_v2_multi_compound.json").trim_end();
        let mut destination = Topology::new();
        let restored = deserialize_document(golden.as_bytes(), &mut destination).unwrap();
        assert_eq!(restored.solids.len(), 1);
        assert_eq!(restored.compounds.len(), 1);
        assert!(destination.is_empty_solid(restored.solids[0]));
        let restored_compound = destination.compound(restored.compounds[0]).unwrap();
        assert_eq!(restored_compound.solids().len(), 1);
        assert!(destination.is_empty_solid(restored_compound.solids()[0]));
        assert_ne!(restored.solids[0], restored_compound.solids()[0]);

        let v3 = serialize_document(&destination, &restored.solids, &restored.compounds).unwrap();
        assert_eq!(
            serde_json::from_slice::<VersionHeader>(&v3)
                .unwrap()
                .version,
            FORMAT_VERSION
        );
        let mut second_destination = Topology::new();
        let second = deserialize_document(&v3, &mut second_destination).unwrap();
        let roundtrip =
            serialize_document(&second_destination, &second.solids, &second.compounds).unwrap();
        assert_eq!(roundtrip, v3);
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

    #[test]
    fn arbitrary_vertex_tolerances_round_trip_bit_exactly() {
        // serde_json's default float path can round the last bit on parse;
        // the `float_roundtrip` feature is load-bearing for the arena
        // format's exact-replay contract. The first six values are measured
        // to lose a bit without it (serde_json 1.0.151); the sweep keeps the
        // test honest if the printer/parser changes.
        let known_bad = [
            2.321_710_310_996_591_2e-7,
            1.360_502_408_555_352_7e-8,
            1.621_364_684_473_496_4e-9,
            1.021_981_500_474_656_4e-7,
            9.454_133_714_726_404e-8,
            1.037_389_917_956_039_5e-6,
        ];
        let mut state = 0x9E37_79B9_7F4A_7C15_u64;
        let mut splitmix = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let sweep = (0..64).map(|_| {
            let mantissa = (splitmix() >> 11) as f64 / (1u64 << 53) as f64;
            10f64.powf(-9.0 + mantissa * 6.0)
        });
        for tolerance in known_bad.into_iter().chain(sweep) {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
            let vertex = topo.vertex_id_from_index(0).expect("box has a vertex 0");
            topo.vertex_mut(vertex)
                .unwrap()
                .set_tolerance(tolerance)
                .unwrap();

            let bytes = serialize_solid(&topo, solid).unwrap();
            let mut restored = Topology::new();
            let restored_solid = deserialize_solid(&bytes, &mut restored).unwrap();
            let restored_vertex = restored
                .vertex_id_from_index(0)
                .expect("restored box has a vertex 0");
            let restored_tolerance = restored.vertex(restored_vertex).unwrap().tolerance();
            assert_eq!(
                restored_tolerance.to_bits(),
                tolerance.to_bits(),
                "vertex tolerance {tolerance:e} lost precision through the arena round-trip",
            );
            // And the re-serialization of the restored document is
            // byte-identical, so chained replays cannot drift.
            let restored_bytes = serialize_solid(&restored, restored_solid).unwrap();
            assert_eq!(
                restored_bytes, bytes,
                "round-tripped document with tolerance {tolerance:e} is not byte-identical",
            );
        }
    }
}
