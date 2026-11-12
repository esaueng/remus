//! Exact whole-topology images for session snapshots.
//!
//! A [`TopologyImage`] is a [`Topology`] as plain data: every arena slot in
//! order, retired tombstones and live flags included, the pcurve use index,
//! the attribute store, the whole journal (entries, live index, high-water
//! counters and the tick each entry recorded) and the journal-continuity
//! mutation counter. Handles are raw slot indices, so a topology rebuilt
//! with [`Topology::from_image`] answers every handle exactly as its source
//! did: live handles resolve to identical entities, retired ones stay
//! stale, and the next allocation in each arena takes the same index.
//!
//! What an image leaves out is runtime state: the transaction undo log
//! (empty between operations) and the PERF-Q02 cache identity. A rebuilt
//! topology starts a fresh cache lineage and undo lineage, exactly like
//! [`Topology::new`].
//!
//! [`Topology::image_view`] borrows the arenas for a writer, so encoding a
//! large session never copies its geometry; [`Topology::to_image`] is the
//! owned copy. [`Topology::from_image`] validates before it builds: every
//! reference of every slot names a slot of its arena, live entities hang
//! off live entities along the boundary chain (solid, shell, face, wire or
//! loop, coedge, edge, vertex), loops and coedges agree with their owners,
//! NURBS carriers are structurally sound, and the journal satisfies
//! [`Journal::from_snapshot`]. A refused image builds nothing.

use remus_math::curves2d::Curve2D;

use remus_math::compact_sort::CompactSort;

use super::Topology;
use crate::arena::{Arena, Id};
use crate::attributes::{AttributeStore, EntityAttributes};
use crate::coedge::{Coedge, PeriodicWinding};
use crate::compound::Compound;
use crate::compsolid::CompSolid;
use crate::edge::{Edge, EdgeCurve};
use crate::face::{Face, FaceSurface};
use crate::face_loop::Loop;
use crate::journal::{EntityKey, EntityKind, Journal, JournalSnapshot};
use crate::pcurve::{PCurve, PCurveRegistry};
use crate::shell::Shell;
use crate::solid::Solid;
use crate::vertex::Vertex;
use crate::wire::{OrientedEdge, Wire};
use crate::{BodyClass, TopologyError};

/// Why [`Topology::from_image`] refused an image.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid topology image: {reason}")]
pub struct TopologyImageError {
    /// What is wrong, naming the arena and slot.
    pub reason: String,
}

impl TopologyImageError {
    fn new(reason: impl Into<String>) -> Self {
        Self {
            reason: reason.into(),
        }
    }
}

/// One arena as plain data: a value and a liveness flag per slot.
#[derive(Debug, Clone)]
pub struct ArenaImage<T> {
    /// Slot values in slot order, retired tombstones included.
    pub items: Vec<T>,
    /// Whether each slot holds a live entity.
    pub live: Vec<bool>,
}

impl<T> Default for ArenaImage<T> {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            live: Vec::new(),
        }
    }
}

/// A borrowed arena: its slots and their liveness, in slot order.
#[derive(Debug)]
pub struct ArenaSlots<'a, T> {
    /// Slot values, retired tombstones included.
    pub items: &'a [T],
    /// Whether each slot holds a live entity.
    pub live: &'a [bool],
}

// Manual impls: a borrow is `Copy` whatever `T` is.
impl<T> Clone for ArenaSlots<'_, T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for ArenaSlots<'_, T> {}

impl<'a, T> ArenaSlots<'a, T> {
    fn of(arena: &'a Arena<T>) -> Self {
        let (items, live) = arena.raw_slots();
        Self { items, live }
    }

    /// `(slot index, value, live)` for every slot.
    pub fn iter(&self) -> impl Iterator<Item = (usize, &'a T, bool)> + 'a {
        let (items, live) = (self.items, self.live);
        items
            .iter()
            .zip(live)
            .enumerate()
            .map(|(index, (value, &is_live))| (index, value, is_live))
    }
}

/// An edge with raw slot references.
#[derive(Debug, Clone)]
pub struct EdgeImage {
    /// Start vertex slot.
    pub start: usize,
    /// End vertex slot.
    pub end: usize,
    /// The edge's curve.
    pub curve: EdgeCurve,
    /// Edge-specific tolerance, if any.
    pub tolerance: Option<f64>,
    /// Explicit trim interval, if any (finite bounds).
    pub trim: Option<(f64, f64)>,
}

/// A wire with raw slot references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireImage {
    /// `(edge slot, forward)` in traversal order; never empty.
    pub edges: Vec<(usize, bool)>,
    /// Whether the wire is closed.
    pub closed: bool,
    /// The wire's stored body class.
    pub body_class: BodyClass,
}

/// A face with raw slot references.
#[derive(Debug, Clone)]
pub struct FaceImage {
    /// Outer wire slot.
    pub outer_wire: usize,
    /// Inner wire slots.
    pub inner_wires: Vec<usize>,
    /// Authoritative boundary loop slots, outer first.
    pub boundary_loops: Vec<usize>,
    /// The face's support surface.
    pub surface: FaceSurface,
    /// Whether the face is reversed relative to its surface.
    pub reversed: bool,
}

/// A shell with raw slot references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShellImage {
    /// Face slots (empty only for the empty-result sentinel shell).
    pub faces: Vec<usize>,
    /// The shell's stored body class.
    pub body_class: BodyClass,
}

/// A solid with raw slot references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SolidImage {
    /// Outer shell slot.
    pub outer_shell: usize,
    /// Inner (cavity) shell slots.
    pub inner_shells: Vec<usize>,
}

/// A compound with raw slot references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompoundImage {
    /// Solid slots.
    pub solids: Vec<usize>,
}

/// A comp-solid with raw slot references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompSolidImage {
    /// Solid slots.
    pub solids: Vec<usize>,
    /// Shared face slots.
    pub shared_faces: Vec<usize>,
}

/// A boundary loop with raw slot references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoopImage {
    /// Owning face slot.
    pub face: usize,
    /// Coedge slots in traversal order.
    pub coedges: Vec<usize>,
    /// Whether the loop closes.
    pub closed: bool,
}

/// A coedge with raw slot references.
#[derive(Debug, Clone)]
pub struct CoedgeImage {
    /// Edge slot.
    pub edge: usize,
    /// Traversal orientation relative to the edge.
    pub forward: bool,
    /// Parent loop slot.
    pub parent_loop: usize,
    /// The use's pcurve, if stored.
    pub pcurve: Option<PCurve>,
    /// Periodic parameter-space lift counts.
    pub periodic_winding: PeriodicWinding,
}

/// One entry of the `(edge, face, orientation) → coedge` use index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct PCurveUseImage {
    /// Edge slot.
    pub edge: usize,
    /// Face slot.
    pub face: usize,
    /// Orientation of the use.
    pub forward: bool,
    /// The indexed coedge slot.
    pub coedge: usize,
}

/// A whole [`Topology`] as plain data; see the module docs.
#[derive(Debug, Clone, Default)]
pub struct TopologyImage {
    /// Vertex arena.
    pub vertices: ArenaImage<Vertex>,
    /// Edge arena.
    pub edges: ArenaImage<EdgeImage>,
    /// Wire arena.
    pub wires: ArenaImage<WireImage>,
    /// Face arena.
    pub faces: ArenaImage<FaceImage>,
    /// Shell arena.
    pub shells: ArenaImage<ShellImage>,
    /// Solid arena.
    pub solids: ArenaImage<SolidImage>,
    /// Compound arena.
    pub compounds: ArenaImage<CompoundImage>,
    /// Comp-solid arena.
    pub compsolids: ArenaImage<CompSolidImage>,
    /// Loop arena.
    pub loops: ArenaImage<LoopImage>,
    /// Coedge arena.
    pub coedges: ArenaImage<CoedgeImage>,
    /// The pcurve use index, strictly increasing by `(edge, face, forward)`.
    pub pcurve_uses: Vec<PCurveUseImage>,
    /// Solid attributes, strictly increasing by slot; none empty.
    pub solid_attributes: Vec<(usize, EntityAttributes)>,
    /// Face attributes, strictly increasing by slot; none empty.
    pub face_attributes: Vec<(usize, EntityAttributes)>,
    /// The whole journal.
    pub journal: JournalSnapshot,
    /// The tick count each journal entry recorded, oldest first.
    pub journal_ticks: Vec<u64>,
    /// The journal-continuity mutation counter.
    pub mutation_ticks: u64,
}

/// A borrowed [`TopologyImage`]: the arenas in place, for writers that must
/// not copy a session's geometry.
#[derive(Debug)]
pub struct TopologyView<'a> {
    /// Vertex arena.
    pub vertices: ArenaSlots<'a, Vertex>,
    /// Edge arena.
    pub edges: ArenaSlots<'a, Edge>,
    /// Wire arena.
    pub wires: ArenaSlots<'a, Wire>,
    /// Face arena.
    pub faces: ArenaSlots<'a, Face>,
    /// Shell arena.
    pub shells: ArenaSlots<'a, Shell>,
    /// Solid arena.
    pub solids: ArenaSlots<'a, Solid>,
    /// Compound arena.
    pub compounds: ArenaSlots<'a, Compound>,
    /// Comp-solid arena.
    pub compsolids: ArenaSlots<'a, CompSolid>,
    /// Loop arena.
    pub loops: ArenaSlots<'a, Loop>,
    /// Coedge arena.
    pub coedges: ArenaSlots<'a, Coedge>,
    /// The pcurve use index, sorted by `(edge, face, forward)`.
    pub pcurve_uses: Vec<PCurveUseImage>,
    /// The attribute store.
    pub attributes: &'a AttributeStore,
    /// The journal.
    pub journal: &'a Journal,
    /// The journal-continuity mutation counter.
    pub mutation_ticks: u64,
}

impl Topology {
    /// Borrows the whole topology for an exact writer; see the module docs.
    #[must_use]
    pub fn image_view(&self) -> TopologyView<'_> {
        let mut pcurve_uses: Vec<PCurveUseImage> = self
            .pcurves
            .raw_uses()
            .map(|(key, coedge)| PCurveUseImage {
                edge: key.edge.index(),
                face: key.face.index(),
                forward: key.forward,
                coedge: coedge.index(),
            })
            .collect();
        // Keys are unique, so the unstable order is the stable one.
        pcurve_uses.compact_sort_unstable();
        TopologyView {
            vertices: ArenaSlots::of(&self.vertices),
            edges: ArenaSlots::of(&self.edges),
            wires: ArenaSlots::of(&self.wires),
            faces: ArenaSlots::of(&self.faces),
            shells: ArenaSlots::of(&self.shells),
            solids: ArenaSlots::of(&self.solids),
            compounds: ArenaSlots::of(&self.compounds),
            compsolids: ArenaSlots::of(&self.compsolids),
            loops: ArenaSlots::of(&self.loops),
            coedges: ArenaSlots::of(&self.coedges),
            pcurve_uses,
            attributes: &self.attributes,
            journal: &self.journal,
            mutation_ticks: self.mutation_ticks,
        }
    }

    /// An owned copy of the whole topology as plain data.
    #[must_use]
    pub fn to_image(&self) -> TopologyImage {
        fn copy<T, U>(slots: ArenaSlots<'_, T>, map: impl Fn(&T) -> U) -> ArenaImage<U> {
            ArenaImage {
                items: slots.items.iter().map(map).collect(),
                live: slots.live.to_vec(),
            }
        }
        let view = self.image_view();
        TopologyImage {
            vertices: copy(view.vertices, Clone::clone),
            edges: copy(view.edges, |edge| EdgeImage {
                start: edge.start().index(),
                end: edge.end().index(),
                curve: edge.curve().clone(),
                tolerance: edge.tolerance(),
                trim: edge.trim(),
            }),
            wires: copy(view.wires, |wire| WireImage {
                edges: wire
                    .edges()
                    .iter()
                    .map(|oriented| (oriented.edge().index(), oriented.is_forward()))
                    .collect(),
                closed: wire.is_closed(),
                body_class: wire.body_class(),
            }),
            faces: copy(view.faces, |face| FaceImage {
                outer_wire: face.outer_wire().index(),
                inner_wires: face.inner_wires().iter().map(|id| id.index()).collect(),
                boundary_loops: face.boundary_loops().iter().map(|id| id.index()).collect(),
                surface: face.surface().clone(),
                reversed: face.is_reversed(),
            }),
            shells: copy(view.shells, |shell| ShellImage {
                faces: shell.faces().iter().map(|id| id.index()).collect(),
                body_class: shell.body_class(),
            }),
            solids: copy(view.solids, |solid| SolidImage {
                outer_shell: solid.outer_shell().index(),
                inner_shells: solid.inner_shells().iter().map(|id| id.index()).collect(),
            }),
            compounds: copy(view.compounds, |compound| CompoundImage {
                solids: compound.solids().iter().map(|id| id.index()).collect(),
            }),
            compsolids: copy(view.compsolids, |compsolid| CompSolidImage {
                solids: compsolid.solids().iter().map(|id| id.index()).collect(),
                shared_faces: compsolid
                    .shared_faces()
                    .iter()
                    .map(|id| id.index())
                    .collect(),
            }),
            loops: copy(view.loops, |boundary| LoopImage {
                face: boundary.face().index(),
                coedges: boundary.coedges().iter().map(|id| id.index()).collect(),
                closed: boundary.is_closed(),
            }),
            coedges: copy(view.coedges, |coedge| CoedgeImage {
                edge: coedge.edge().index(),
                forward: coedge.is_forward(),
                parent_loop: coedge.parent_loop().index(),
                pcurve: coedge.pcurve().cloned(),
                periodic_winding: coedge.periodic_winding(),
            }),
            pcurve_uses: view.pcurve_uses,
            solid_attributes: view
                .attributes
                .solids_with_attributes()
                .into_iter()
                .map(|(id, attributes)| (id.index(), attributes.clone()))
                .collect(),
            face_attributes: view
                .attributes
                .faces_with_attributes()
                .into_iter()
                .map(|(id, attributes)| (id.index(), attributes.clone()))
                .collect(),
            journal: view.journal.snapshot(),
            journal_ticks: view
                .journal
                .entries()
                .iter()
                .map(crate::journal::JournalEntry::ticks_after)
                .collect(),
            mutation_ticks: view.mutation_ticks,
        }
    }

    /// Rebuilds a topology from an image, validating it first; see the
    /// module docs. The result has a fresh cache and undo lineage.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyImageError`] naming the first violated invariant;
    /// nothing is built.
    pub fn from_image(image: TopologyImage) -> Result<Self, TopologyImageError> {
        validate_image(&image)?;
        build(image)
    }
}

/// Slot counts of every arena, for range checks.
struct Counts {
    vertices: usize,
    edges: usize,
    faces: usize,
    loops: usize,
    coedges: usize,
}

/// A `(slot, live)` lookup over one arena's liveness flags.
#[derive(Clone, Copy)]
struct Live<'a> {
    name: &'static str,
    flags: &'a [bool],
}

impl Live<'_> {
    /// Refuses `target` unless it names a slot of this arena, and, when the
    /// referring slot is live, a live one.
    #[inline(never)]
    fn check(
        self,
        target: usize,
        from_live: bool,
        subject: &Subject,
    ) -> Result<(), TopologyImageError> {
        match self.flags.get(target) {
            None => Err(TopologyImageError::new(format!(
                "{subject} references {} slot {target}, beyond the arena's {} slots",
                self.name,
                self.flags.len()
            ))),
            Some(false) if from_live => Err(TopologyImageError::new(format!(
                "live {subject} references retired {} slot {target}",
                self.name
            ))),
            Some(_) => Ok(()),
        }
    }

    fn all(
        self,
        targets: impl IntoIterator<Item = usize>,
        from_live: bool,
        subject: &Subject,
    ) -> Result<(), TopologyImageError> {
        targets
            .into_iter()
            .try_for_each(|target| self.check(target, from_live, subject))
    }

    /// Refuses `target` unless it names a slot of this arena (live or not).
    fn in_range(self, target: usize, subject: &Subject) -> Result<(), TopologyImageError> {
        self.check(target, false, subject)
    }
}

/// The entity a check is about, rendered only on refusal.
struct Subject {
    kind: &'static str,
    index: usize,
}

impl std::fmt::Display for Subject {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} {}", self.kind, self.index)
    }
}

fn subject(kind: &'static str, index: usize) -> Subject {
    Subject { kind, index }
}

fn check_lengths<T>(name: &str, arena: &ArenaImage<T>) -> Result<(), TopologyImageError> {
    if arena.items.len() == arena.live.len() {
        Ok(())
    } else {
        Err(TopologyImageError::new(format!(
            "{name} arena has {} values but {} liveness flags",
            arena.items.len(),
            arena.live.len()
        )))
    }
}

fn slots<T>(arena: &ArenaImage<T>) -> impl Iterator<Item = (usize, &T, bool)> {
    arena
        .items
        .iter()
        .zip(&arena.live)
        .enumerate()
        .map(|(index, (value, &live))| (index, value, live))
}

#[allow(clippy::too_many_lines)]
fn validate_image(image: &TopologyImage) -> Result<(), TopologyImageError> {
    check_lengths("vertex", &image.vertices)?;
    check_lengths("edge", &image.edges)?;
    check_lengths("wire", &image.wires)?;
    check_lengths("face", &image.faces)?;
    check_lengths("shell", &image.shells)?;
    check_lengths("solid", &image.solids)?;
    check_lengths("compound", &image.compounds)?;
    check_lengths("comp-solid", &image.compsolids)?;
    check_lengths("loop", &image.loops)?;
    check_lengths("coedge", &image.coedges)?;
    let counts = Counts {
        vertices: image.vertices.live.len(),
        edges: image.edges.live.len(),
        faces: image.faces.live.len(),
        loops: image.loops.live.len(),
        coedges: image.coedges.live.len(),
    };
    let vertices = Live {
        name: "vertex",
        flags: &image.vertices.live,
    };
    let edges = Live {
        name: "edge",
        flags: &image.edges.live,
    };
    let wires = Live {
        name: "wire",
        flags: &image.wires.live,
    };
    let faces = Live {
        name: "face",
        flags: &image.faces.live,
    };
    let shells = Live {
        name: "shell",
        flags: &image.shells.live,
    };
    let solids = Live {
        name: "solid",
        flags: &image.solids.live,
    };
    let loops = Live {
        name: "loop",
        flags: &image.loops.live,
    };
    let coedges = Live {
        name: "coedge",
        flags: &image.coedges.live,
    };

    for (index, edge, live) in slots(&image.edges) {
        let at = subject("edge", index);
        vertices.check(edge.start, live, &at)?;
        vertices.check(edge.end, live, &at)?;
        if let Some((t0, t1)) = edge.trim
            && !(t0.is_finite() && t1.is_finite())
        {
            return Err(TopologyImageError::new(format!(
                "{at} has a non-finite trim"
            )));
        }
        if live && let EdgeCurve::NurbsCurve(curve) = &edge.curve {
            curve.validate().map_err(|error| {
                TopologyImageError::new(format!("{at} has an invalid NURBS curve: {error}"))
            })?;
        }
    }
    for (index, wire, live) in slots(&image.wires) {
        let at = subject("wire", index);
        if wire.edges.is_empty() {
            return Err(TopologyImageError::new(format!("{at} has no edges")));
        }
        edges.all(wire.edges.iter().map(|&(edge, _)| edge), live, &at)?;
    }
    // Every live loop is listed by exactly the live face it names.
    let mut loop_owner: Vec<Option<usize>> = vec![None; counts.loops];
    for (index, face, live) in slots(&image.faces) {
        let at = subject("face", index);
        wires.check(face.outer_wire, live, &at)?;
        wires.all(face.inner_wires.iter().copied(), live, &at)?;
        loops.all(face.boundary_loops.iter().copied(), live, &at)?;
        if !live {
            continue;
        }
        if let FaceSurface::Nurbs(surface) = &face.surface {
            surface.validate().map_err(|error| {
                TopologyImageError::new(format!("{at} has an invalid NURBS surface: {error}"))
            })?;
        }
        for &boundary in &face.boundary_loops {
            let owner = loop_owner.get_mut(boundary).ok_or_else(|| {
                TopologyImageError::new(format!("{at} references loop slot {boundary}"))
            })?;
            if owner.replace(index).is_some() {
                return Err(TopologyImageError::new(format!(
                    "loop {boundary} is listed by more than one live face"
                )));
            }
        }
    }
    for (index, shell, live) in slots(&image.shells) {
        faces.all(shell.faces.iter().copied(), live, &subject("shell", index))?;
    }
    for (index, solid, live) in slots(&image.solids) {
        let at = subject("solid", index);
        shells.check(solid.outer_shell, live, &at)?;
        shells.all(solid.inner_shells.iter().copied(), live, &at)?;
    }
    for (index, compound, _) in slots(&image.compounds) {
        let at = subject("compound", index);
        solids.all(compound.solids.iter().copied(), false, &at)?;
    }
    for (index, compsolid, _) in slots(&image.compsolids) {
        let at = subject("comp-solid", index);
        solids.all(compsolid.solids.iter().copied(), false, &at)?;
        faces.all(compsolid.shared_faces.iter().copied(), false, &at)?;
    }
    // Every live coedge is listed by exactly the live loop it names.
    let mut coedge_owner: Vec<Option<usize>> = vec![None; counts.coedges];
    for (index, boundary, live) in slots(&image.loops) {
        let at = subject("loop", index);
        faces.check(boundary.face, live, &at)?;
        coedges.all(boundary.coedges.iter().copied(), live, &at)?;
        if !live {
            continue;
        }
        if loop_owner.get(index).copied().flatten() != Some(boundary.face) {
            return Err(TopologyImageError::new(format!(
                "live {at} is not listed by its face {}",
                boundary.face
            )));
        }
        for &coedge in &boundary.coedges {
            let owner = coedge_owner.get_mut(coedge).ok_or_else(|| {
                TopologyImageError::new(format!("{at} references coedge slot {coedge}"))
            })?;
            if owner.replace(index).is_some() {
                return Err(TopologyImageError::new(format!(
                    "coedge {coedge} is listed by more than one live loop"
                )));
            }
        }
    }
    for (index, coedge, live) in slots(&image.coedges) {
        let at = subject("coedge", index);
        edges.check(coedge.edge, live, &at)?;
        loops.check(coedge.parent_loop, live, &at)?;
        if !live {
            continue;
        }
        if coedge_owner.get(index).copied().flatten() != Some(coedge.parent_loop) {
            return Err(TopologyImageError::new(format!(
                "live {at} is not listed by its loop {}",
                coedge.parent_loop
            )));
        }
        if let Some(pcurve) = &coedge.pcurve
            && let Some(defect) = pcurve_defect(pcurve)
        {
            return Err(TopologyImageError::new(format!(
                "{at} has a pcurve whose {defect}"
            )));
        }
    }
    validate_pcurve_uses(image, &counts, edges, faces, coedges)?;
    validate_attributes("solid", &image.solid_attributes, solids)?;
    validate_attributes("face", &image.face_attributes, faces)?;
    validate_journal_keys(&image.journal, &counts)?;
    if image.journal_ticks.len() != image.journal.entries.len() {
        return Err(TopologyImageError::new(format!(
            "journal has {} entries but {} tick counts",
            image.journal.entries.len(),
            image.journal_ticks.len()
        )));
    }
    Ok(())
}

/// The use index is canonical (strictly increasing keys), in range, and
/// each live use names a live coedge of that edge, orientation and face.
fn validate_pcurve_uses(
    image: &TopologyImage,
    counts: &Counts,
    edges: Live<'_>,
    faces: Live<'_>,
    coedges: Live<'_>,
) -> Result<(), TopologyImageError> {
    for pair in image.pcurve_uses.windows(2) {
        if (pair[0].edge, pair[0].face, pair[0].forward)
            >= (pair[1].edge, pair[1].face, pair[1].forward)
        {
            return Err(TopologyImageError::new(
                "pcurve use index is not strictly increasing",
            ));
        }
    }
    for (index, entry) in image.pcurve_uses.iter().enumerate() {
        let at = subject("pcurve use", index);
        edges.in_range(entry.edge, &at)?;
        faces.in_range(entry.face, &at)?;
        coedges.in_range(entry.coedge, &at)?;
        debug_assert!(entry.coedge < counts.coedges);
        let Some((coedge, true)) = image
            .coedges
            .items
            .get(entry.coedge)
            .zip(image.coedges.live.get(entry.coedge).copied())
        else {
            continue;
        };
        let face = image
            .loops
            .items
            .get(coedge.parent_loop)
            .map(|boundary| boundary.face);
        if coedge.edge != entry.edge || coedge.forward != entry.forward || face != Some(entry.face)
        {
            return Err(TopologyImageError::new(format!(
                "{at} indexes coedge {} of another edge use",
                entry.coedge
            )));
        }
    }
    Ok(())
}

fn validate_attributes(
    kind: &'static str,
    entries: &[(usize, EntityAttributes)],
    arena: Live<'_>,
) -> Result<(), TopologyImageError> {
    for pair in entries.windows(2) {
        if pair[0].0 >= pair[1].0 {
            return Err(TopologyImageError::new(format!(
                "{kind} attributes are not strictly increasing by slot"
            )));
        }
    }
    for (index, attributes) in entries {
        let at = Subject {
            kind: if kind == "solid" {
                "solid attribute"
            } else {
                "face attribute"
            },
            index: *index,
        };
        arena.in_range(*index, &at)?;
        if attributes.is_empty() {
            return Err(TopologyImageError::new(format!("{at} is empty")));
        }
    }
    Ok(())
}

/// Live-index keys name a slot of their arena, or are explicit
/// [`EntityKey::UNMAPPED`] placeholders.
fn validate_journal_keys(
    journal: &JournalSnapshot,
    counts: &Counts,
) -> Result<(), TopologyImageError> {
    for (ordinal, key) in &journal.index {
        let slots = match key.kind {
            EntityKind::Vertex => counts.vertices,
            EntityKind::Edge => counts.edges,
            EntityKind::Face => counts.faces,
        };
        if key.index != EntityKey::UNMAPPED && key.index >= slots {
            return Err(TopologyImageError::new(format!(
                "journal ordinal {ordinal} names {} slot {}, beyond the arena's {slots} slots",
                key.kind.as_str(),
                key.index
            )));
        }
    }
    Ok(())
}

/// What makes a stored pcurve structurally unusable, if anything: the
/// checks that keep evaluation in bounds, not geometric quality.
fn pcurve_defect(pcurve: &PCurve) -> Option<&'static str> {
    let Curve2D::Nurbs(nurbs) = pcurve.curve() else {
        return None;
    };
    // `NurbsCurve2D::new`'s structural checks, which keep evaluation's
    // indexing in bounds. Like it, tolerate ulp-level knot wobble and leave
    // non-finite poles to pcurve validation.
    let points = nurbs.control_points().len();
    let knots = nurbs.knots();
    let weights = nurbs.weights();
    let sound = nurbs.degree() >= 1
        && nurbs.degree() < points
        && knots.len() == points.saturating_add(nurbs.degree()).saturating_add(1)
        && weights.len() == points
        && knots.iter().all(|value| value.is_finite())
        && !weights.iter().any(|value| *value <= 0.0);
    (!sound).then_some("NURBS curve is malformed")
}

fn arena_of<T, U>(
    image: ArenaImage<T>,
    mut convert: impl FnMut(T) -> Result<U, TopologyImageError>,
) -> Result<Arena<U>, TopologyImageError> {
    let items = image
        .items
        .into_iter()
        .map(&mut convert)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Arena::from_raw_slots(items, image.live))
}

fn ids<T>(indices: Vec<usize>) -> Vec<Id<T>> {
    indices.into_iter().map(Id::from_raw_index).collect()
}

fn topology_error(error: &TopologyError) -> TopologyImageError {
    TopologyImageError::new(error.to_string())
}

/// Builds the validated image. Entity constructors are the public ones,
/// plus the crate-private setters for fields only the topology writes.
fn build(image: TopologyImage) -> Result<Topology, TopologyImageError> {
    let TopologyImage {
        vertices,
        edges,
        wires,
        faces,
        shells,
        solids,
        compounds,
        compsolids,
        loops,
        coedges,
        pcurve_uses,
        solid_attributes,
        face_attributes,
        journal,
        journal_ticks,
        mutation_ticks,
    } = image;
    let mut journal = Journal::from_snapshot(journal).map_err(|error| topology_error(&error))?;
    if !journal.set_entry_ticks(&journal_ticks) {
        return Err(TopologyImageError::new(
            "journal tick counts do not match its entries",
        ));
    }

    let vertices = Arena::from_raw_slots(vertices.items, vertices.live);
    let edges = arena_of(edges, |edge| {
        let mut built = Edge::with_tolerance(
            Id::from_raw_index(edge.start),
            Id::from_raw_index(edge.end),
            edge.curve,
            edge.tolerance,
        );
        built.set_trim(edge.trim);
        Ok(built)
    })?;
    let wires = arena_of(wires, |wire| {
        let oriented = wire
            .edges
            .into_iter()
            .map(|(edge, forward)| OrientedEdge::new(Id::from_raw_index(edge), forward))
            .collect();
        let mut built = Wire::new(oriented, wire.closed).map_err(|error| topology_error(&error))?;
        built.set_body_class(wire.body_class);
        Ok(built)
    })?;
    let faces = arena_of(faces, |face| {
        let outer = Id::from_raw_index(face.outer_wire);
        let inner = ids(face.inner_wires);
        let mut built = if face.reversed {
            Face::new_reversed(outer, inner, face.surface)
        } else {
            Face::new(outer, inner, face.surface)
        };
        built.replace_boundary_loops(ids(face.boundary_loops));
        Ok(built)
    })?;
    let shells = arena_of(shells, |shell| {
        let mut built = if shell.faces.is_empty() {
            Shell::empty()
        } else {
            Shell::new(ids(shell.faces)).map_err(|error| topology_error(&error))?
        };
        built.set_body_class(shell.body_class);
        Ok(built)
    })?;
    let solids = arena_of(solids, |solid| {
        Ok(Solid::new(
            Id::from_raw_index(solid.outer_shell),
            ids(solid.inner_shells),
        ))
    })?;
    let compounds = arena_of(compounds, |compound| {
        Ok(Compound::new(ids(compound.solids)))
    })?;
    let compsolids = arena_of(compsolids, |compsolid| {
        Ok(CompSolid::new(
            ids(compsolid.solids),
            ids(compsolid.shared_faces),
        ))
    })?;
    let loops = arena_of(loops, |boundary| {
        Ok(Loop::new(
            Id::from_raw_index(boundary.face),
            ids(boundary.coedges),
            boundary.closed,
        ))
    })?;
    let coedges = arena_of(coedges, |coedge| {
        let mut built = Coedge::with_pcurve(
            Id::from_raw_index(coedge.edge),
            coedge.forward,
            Id::from_raw_index(coedge.parent_loop),
            coedge.pcurve,
        );
        built.replace_periodic_winding(coedge.periodic_winding);
        Ok(built)
    })?;
    let mut registry = PCurveRegistry::new();
    for entry in pcurve_uses {
        registry.index_use(
            Id::from_raw_index(entry.edge),
            Id::from_raw_index(entry.face),
            entry.forward,
            Id::from_raw_index(entry.coedge),
        );
    }
    let mut attributes = AttributeStore::new();
    for (index, attributes_of) in solid_attributes {
        attributes.set_solid(Id::from_raw_index(index), attributes_of);
    }
    for (index, attributes_of) in face_attributes {
        attributes.set_face(Id::from_raw_index(index), attributes_of);
    }
    Ok(Topology {
        vertices,
        edges,
        wires,
        faces,
        shells,
        solids,
        compounds,
        compsolids,
        pcurves: registry,
        loops,
        coedges,
        attributes,
        journal,
        mutation_ticks,
        ..Topology::default()
    })
}

#[cfg(test)]
#[path = "image/tests.rs"]
mod tests;
