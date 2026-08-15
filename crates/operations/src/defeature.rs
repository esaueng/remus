//! Defeaturing: remove small features from a solid for simulation simplification.
//!
//! Removing a face leaves a gap in the shell. [`defeature`] closes that gap
//! exactly, or refuses; it never returns a shell it could not close.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::shell::Shell;
use brepkit_topology::solid::{Solid, SolidId};

use crate::OperationsError;
use crate::boolean::{FaceSpec, assemble_solid_mixed_with_history};
use crate::dot_normal_point;

/// Operation name carried by [`OperationsError::Unsupported`] refusals.
const OP: &str = "defeature";

/// Smallest `|n1 · (n2 × n3)|` (unit normals) accepted for a three-plane
/// corner. Below this the three planes are too near-parallel for the corner
/// position to be meaningful, and the heal is refused rather than emitting a
/// far-away intersection point.
const MIN_PLANE_TRIPLE_DET: f64 = 1e-6;

/// A healed corner may move at most this multiple of the wound loop's own
/// bounding-box diagonal. A face extension that restores a feature moves
/// corners by roughly the feature's size; anything far beyond that is a
/// runaway extension, not a heal.
const MAX_HEAL_DISPLACEMENT_FACTOR: f64 = 4.0;

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: OP,
        reason: reason.into(),
    }
}

/// Remove selected faces from a solid and heal the resulting gap.
///
/// The selected faces are deleted from the shell and the hole they leave is
/// closed exactly. Two heal strategies are implemented, chosen from the
/// topology of the wound (the set of edges that separated a removed face from
/// a kept one):
///
/// 1. **Cap** — every wound edge lies on an inner wire of a kept face and that
///    inner wire is wound in its entirety. The removed faces are a blind
///    cavity, a protrusion or a through-hole wall whose opening(s) are holes
///    in kept faces; deleting those inner wires closes the shell. No geometry
///    moves, so analytic surfaces, curved edges and unrelated holes survive
///    untouched.
/// 2. **Extend** — the wound runs across the outer wires of kept faces. Each
///    wound corner is recomputed as the intersection of three kept planes
///    found by walking the wound loop, which is what extending the adjacent
///    faces until they meet produces. This restores the sharp edge behind a
///    chamfer or a corner cut.
///
/// Anything else is refused with [`OperationsError::Unsupported`] naming the
/// reason. The result is validated with [`crate::validate::validate_solid`]
/// before it is returned; a result carrying validation errors is reported as a
/// refusal instead, so this function never yields a solid it could not close.
///
/// This is useful for removing small features (holes, pockets, bosses,
/// chamfers) to simplify geometry for FEA/CFD simulation.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] if no face is selected, a
/// selected face is not part of the solid's outer shell, or removing the
/// selection would leave fewer than 4 faces.
///
/// Returns [`OperationsError::Unsupported`] if the gap cannot be closed
/// exactly — for example the solid has cavity shells, the input is not
/// edge-manifold, the wound crosses a curved kept face, the wound loop is not
/// a simple cycle, no well-conditioned three-plane corner exists (the
/// adjacent faces are parallel and would have to be merged rather than
/// extended), or the healed shell fails validation.
pub fn defeature(
    topo: &mut Topology,
    solid: SolidId,
    faces_to_remove: &[FaceId],
) -> Result<SolidId, OperationsError> {
    Ok(defeature_impl(topo, solid, faces_to_remove)?.solid)
}

/// Exact defeature result used by operations that must compose face history.
pub(crate) struct DefeatureOutcome {
    /// Healed solid.
    pub(crate) solid: SolidId,
    /// Original face index to healed face.
    pub(crate) face_map: HashMap<usize, FaceId>,
}

/// Remove an analytic blend band while allowing curved edges that belong to
/// the wound itself. Such edges disappear during the planar rebuild; unrelated
/// curved edges remain a refusal so no retained geometry is faceted.
pub(crate) fn defeature_blend_band(
    topo: &mut Topology,
    solid: SolidId,
    faces_to_remove: &[FaceId],
) -> Result<DefeatureOutcome, OperationsError> {
    defeature_impl(topo, solid, faces_to_remove)
}

fn defeature_impl(
    topo: &mut Topology,
    solid: SolidId,
    faces_to_remove: &[FaceId],
) -> Result<DefeatureOutcome, OperationsError> {
    if faces_to_remove.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "must select at least one face to remove".into(),
        });
    }

    let solid_data = topo.solid(solid)?;
    if !solid_data.inner_shells().is_empty() {
        return Err(unsupported(format!(
            "solid has {} cavity shell(s); defeaturing only operates on the outer shell",
            solid_data.inner_shells().len()
        )));
    }
    let all_faces: Vec<FaceId> = topo.shell(solid_data.outer_shell())?.faces().to_vec();

    // Map face index -> position in the shell, so the rest of the algorithm
    // can work with dense positions.
    let position_of: BTreeMap<usize, usize> = all_faces
        .iter()
        .enumerate()
        .map(|(pos, f)| (f.index(), pos))
        .collect();

    let mut removed = vec![false; all_faces.len()];
    for f in faces_to_remove {
        let Some(&pos) = position_of.get(&f.index()) else {
            return Err(OperationsError::InvalidInput {
                reason: format!("face {} is not part of the solid's outer shell", f.index()),
            });
        };
        removed[pos] = true;
    }

    let kept_positions: Vec<usize> = (0..all_faces.len()).filter(|p| !removed[*p]).collect();
    if kept_positions.len() < 4 {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "removing {} faces would leave only {} faces (minimum 4 for a solid)",
                faces_to_remove.len(),
                kept_positions.len()
            ),
        });
    }

    let wound = wound_edges(topo, &all_faces, &removed)?;
    let plan = classify_wound(topo, &all_faces, &kept_positions, &wound)?;

    let result = if plan.needs_extend {
        heal_by_extending(topo, &all_faces, &kept_positions, &removed, &wound, &plan)?
    } else {
        heal_by_capping(topo, solid, &kept_positions, &plan)?
    };

    let report = crate::validate::validate_solid(topo, result.solid)?;
    if !report.is_valid() {
        let detail: Vec<&str> = report
            .issues
            .iter()
            .filter(|i| i.severity == crate::validate::Severity::Error)
            .map(|i| i.description.as_str())
            .collect();
        return Err(unsupported(format!(
            "healed shell failed validation ({})",
            detail.join("; ")
        )));
    }
    // A shell can pass the structural checks and still be turned inside out.
    if crate::measure::solid_volume(topo, result.solid, 0.05)? <= 0.0 {
        return Err(unsupported(
            "healed shell encloses no volume; the faces do not close over the gap".to_string(),
        ));
    }

    Ok(result)
}

/// Edges that separated a removed face from a kept one, each paired with the
/// shell position of the kept face on its other side.
struct Wound {
    /// Wound edge index -> position of the kept face across it.
    kept_side: BTreeMap<usize, usize>,
}

impl Wound {
    fn contains(&self, edge: EdgeId) -> bool {
        self.kept_side.contains_key(&edge.index())
    }
}

/// Collect the wound edges, checking the input is edge-manifold first.
fn wound_edges(
    topo: &Topology,
    all_faces: &[FaceId],
    removed: &[bool],
) -> Result<Wound, OperationsError> {
    // Edge index -> shell positions of the faces using it.
    let mut uses: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (pos, &fid) in all_faces.iter().enumerate() {
        let face = topo.face(fid)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oe in topo.wire(wire_id)?.edges() {
                uses.entry(oe.edge().index()).or_default().push(pos);
            }
        }
    }

    let mut kept_side = BTreeMap::new();
    for (&edge, faces) in &uses {
        if faces.len() != 2 {
            return Err(unsupported(format!(
                "input solid is not edge-manifold: edge {edge} is used by {} face(s)",
                faces.len()
            )));
        }
        match (removed[faces[0]], removed[faces[1]]) {
            (true, false) => {
                kept_side.insert(edge, faces[1]);
            }
            (false, true) => {
                kept_side.insert(edge, faces[0]);
            }
            _ => {}
        }
    }

    Ok(Wound { kept_side })
}

/// How the wound meets each kept face, and therefore which heal applies.
struct HealPlan {
    /// `(shell position, inner-wire slots)` for inner wires that are wound in
    /// their entirety and so must be deleted to close the shell. Sorted by
    /// position for determinism.
    drop_inner: Vec<(usize, Vec<usize>)>,
    /// Whether any wound edge lies somewhere other than a fully wound inner
    /// wire, which means faces have to be extended rather than merely
    /// uncapped.
    needs_extend: bool,
}

fn classify_wound(
    topo: &Topology,
    all_faces: &[FaceId],
    kept_positions: &[usize],
    wound: &Wound,
) -> Result<HealPlan, OperationsError> {
    let mut drop_inner: Vec<(usize, Vec<usize>)> = Vec::new();
    let mut needs_extend = false;

    for &pos in kept_positions {
        let face = topo.face(all_faces[pos])?;

        let outer = topo.wire(face.outer_wire())?;
        if outer.edges().iter().any(|oe| wound.contains(oe.edge())) {
            needs_extend = true;
        }

        let mut slots = Vec::new();
        for (slot, &wire_id) in face.inner_wires().iter().enumerate() {
            let wire = topo.wire(wire_id)?;
            let total = wire.edges().len();
            let n_wound = wire
                .edges()
                .iter()
                .filter(|oe| wound.contains(oe.edge()))
                .count();
            if n_wound == 0 {
                continue;
            }
            if n_wound == total {
                slots.push(slot);
            } else {
                needs_extend = true;
            }
        }
        if !slots.is_empty() {
            drop_inner.push((pos, slots));
        }
    }

    Ok(HealPlan {
        drop_inner,
        needs_extend,
    })
}

// ---------------------------------------------------------------------------
// Cap heal
// ---------------------------------------------------------------------------

/// Close the wound by deleting the kept-face inner wires it bounds.
///
/// Works on a deep copy so the input solid is untouched, and moves no vertex:
/// the healed body differs from the input by exactly the volume the removed
/// patch and those inner-wire openings enclosed.
fn heal_by_capping(
    topo: &mut Topology,
    solid: SolidId,
    kept_positions: &[usize],
    plan: &HealPlan,
) -> Result<DefeatureOutcome, OperationsError> {
    let (copy, copied_faces) = crate::copy::copy_solid_with_face_map(topo, solid)?;
    // `copy_solid` preserves shell face order, so positions carry over.
    let copy_faces: Vec<FaceId> = topo
        .shell(topo.solid(copy)?.outer_shell())?
        .faces()
        .to_vec();

    for (pos, slots) in &plan.drop_inner {
        let face = topo.face_mut(copy_faces[*pos])?;
        let mut slot = 0usize;
        face.inner_wires_mut().retain(|_| {
            let keep = !slots.contains(&slot);
            slot += 1;
            keep
        });
    }

    let kept: Vec<FaceId> = kept_positions.iter().map(|&p| copy_faces[p]).collect();
    let shell = Shell::new(kept).map_err(OperationsError::Topology)?;
    let shell_id = topo.add_shell(shell);
    let result = topo.add_solid(Solid::new(shell_id, Vec::new()));
    let kept_indices: BTreeSet<usize> = kept_positions
        .iter()
        .map(|&position| copy_faces[position].index())
        .collect();
    let face_map = copied_faces
        .into_iter()
        .filter_map(|(source, copied)| {
            if !kept_indices.contains(&copied) {
                return None;
            }
            topo.face_id_from_index(copied).map(|face| (source, face))
        })
        .collect();
    Ok(DefeatureOutcome {
        solid: result,
        face_map,
    })
}
// ---------------------------------------------------------------------------
// Extend heal
// ---------------------------------------------------------------------------

/// A kept face's plane, in `normal · p = d` form with a unit normal whose
/// sign has been canonicalised so coplanar faces of opposite orientation
/// compare equal.
#[derive(Clone, Copy)]
struct Plane {
    normal: Vec3,
    d: f64,
}

impl Plane {
    fn new(normal: Vec3, d: f64) -> Self {
        // Canonical sign: first non-negligible component positive. Keeps the
        // plane set deduplicated regardless of which side each face faces.
        let flip = if normal.x().abs() > 1e-12 {
            normal.x() < 0.0
        } else if normal.y().abs() > 1e-12 {
            normal.y() < 0.0
        } else {
            normal.z() < 0.0
        };
        if flip {
            Self {
                normal: normal * -1.0,
                d: -d,
            }
        } else {
            Self { normal, d }
        }
    }

    fn same_as(&self, other: &Self, eps: f64) -> bool {
        (self.normal - other.normal).length() < 1e-9 && (self.d - other.d).abs() < eps
    }

    fn contains(&self, p: Point3, eps: f64) -> bool {
        (dot_normal_point(self.normal, p) - self.d).abs() <= eps
    }
}

/// Above this many distinct adjacent planes the corner search is refused
/// rather than run: enumerating triples is cubic, and a patch that large is
/// not the small feature defeaturing is for.
const MAX_ADJACENT_PLANES: usize = 64;

/// Everything the extend heal needs to know about the shell's connectivity.
struct Adjacency {
    /// Shell position -> positions of faces sharing at least one edge.
    neighbours: Vec<Vec<usize>>,
    /// Vertex index -> shell positions of the faces meeting there.
    at_vertex: BTreeMap<usize, Vec<usize>>,
}

fn build_adjacency(topo: &Topology, all_faces: &[FaceId]) -> Result<Adjacency, OperationsError> {
    let mut by_edge: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut at_vertex: BTreeMap<usize, Vec<usize>> = BTreeMap::new();

    for (pos, &fid) in all_faces.iter().enumerate() {
        let face = topo.face(fid)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oe in topo.wire(wire_id)?.edges() {
                by_edge.entry(oe.edge().index()).or_default().push(pos);
                let edge = topo.edge(oe.edge())?;
                for vid in [edge.start(), edge.end()] {
                    let slot = at_vertex.entry(vid.index()).or_default();
                    if !slot.contains(&pos) {
                        slot.push(pos);
                    }
                }
            }
        }
    }

    let mut neighbours = vec![Vec::new(); all_faces.len()];
    for faces in by_edge.values() {
        for &a in faces {
            for &b in faces {
                if a != b && !neighbours[a].contains(&b) {
                    neighbours[a].push(b);
                }
            }
        }
    }

    Ok(Adjacency {
        neighbours,
        at_vertex,
    })
}

/// Close the wound by extending the adjacent planar faces until they meet.
///
/// Every vertex shared between a removed face and a kept one is relocated to
/// the corner of the local plane arrangement it must move to once the removed
/// patch is gone. That corner is the unique point where three of the patch's
/// adjacent kept planes meet and which still lies on every kept plane already
/// passing through the vertex — i.e. exactly where the adjacent faces, grown
/// within their own planes, close over the gap. The search starts at the
/// patch faces touching the vertex and widens through the patch only while
/// the corner is still undetermined, so the nearest enclosing corner wins.
///
/// Ambiguity is a refusal, not a guess: a vertex with no such corner (the
/// neighbours are parallel and would have to be merged rather than extended)
/// or with more than one is reported through [`OperationsError::Unsupported`].
fn heal_by_extending(
    topo: &mut Topology,
    all_faces: &[FaceId],
    kept_positions: &[usize],
    removed: &[bool],
    wound: &Wound,
    plan: &HealPlan,
) -> Result<DefeatureOutcome, OperationsError> {
    let tol = Tolerance::new();

    // Extending a face means growing its boundary within its own surface.
    // Only planes support that here. Curved kept edges are checked after the
    // healed corners are known: a wound arc may disappear exactly when both
    // of its endpoints collapse to the same recovered corner, but every curve
    // that would survive the rebuild is still refused rather than chorded.
    let mut planes: Vec<Option<Plane>> = vec![None; all_faces.len()];
    for &pos in kept_positions {
        let fid = all_faces[pos];
        let face = topo.face(fid)?;
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            return Err(unsupported(format!(
                "kept face {} is a {} surface; extending the shell to close the \
                 gap is only implemented for planar faces",
                fid.index(),
                face.surface().type_tag()
            )));
        };
        let normal = normal.normalize()?;
        let anchor = first_wire_vertex(topo, fid)?;
        planes[pos] = Some(Plane::new(normal, dot_normal_point(normal, anchor)));
    }

    let adjacency = build_adjacency(topo, all_faces)?;

    // Scale-relative epsilon for "the corner lies on this plane" tests. It only
    // ever makes the heal refuse more; the result itself is still checked by
    // the strict `validate_solid` in `defeature`.
    let (patch_span, model_span) = spans(topo, all_faces, removed, &adjacency)?;
    let eps = tol.linear.max(model_span * 1e-10);
    let max_displacement = patch_span * MAX_HEAL_DISPLACEMENT_FACTOR;

    // Vertices shared between a removed face and a kept one have to move;
    // vertices interior to the patch simply disappear with it.
    let mut moved: BTreeMap<usize, Point3> = BTreeMap::new();
    for (&vertex, faces) in &adjacency.at_vertex {
        if !faces.iter().any(|p| removed[*p]) || !faces.iter().any(|p| !removed[*p]) {
            continue;
        }
        let vid = topo
            .vertex_id_from_index(vertex)
            .ok_or_else(|| unsupported(format!("vertex {vertex} disappeared")))?;
        let original = topo.vertex(vid)?.point();
        let corner = resolve_corner(faces, &planes, removed, &adjacency, original, eps)?;
        if (corner - original).length() > max_displacement {
            return Err(unsupported(
                "extending the adjacent faces moves a corner far outside the \
                 removed feature; the faces do not close over the gap"
                    .to_string(),
            ));
        }
        moved.insert(vertex, corner);
    }

    // `assemble_solid_mixed` rebuilds planar wires from vertices and therefore
    // emits straight edges. That is exact for a curved wound edge only when
    // the edge vanishes: both of its endpoints move to the same healed corner.
    // This is how the circular end arcs of a removed plane-plane fillet
    // disappear. Any unrelated or surviving curve remains a typed refusal.
    for &pos in kept_positions {
        let fid = all_faces[pos];
        let face = topo.face(fid)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oe in topo.wire(wire_id)?.edges() {
                let edge = topo.edge(oe.edge())?;
                if matches!(edge.curve(), EdgeCurve::Line) {
                    continue;
                }
                let collapsed_wound =
                    wound_edge_collapses(topo, edge, oe.edge(), &moved, wound, tol.linear)?;
                if !collapsed_wound {
                    return Err(unsupported(format!(
                        "kept face {} has a curved edge that survives the heal; \
                         extending the shell is exact only when a curved wound \
                         edge collapses to one recovered corner",
                        fid.index()
                    )));
                }
            }
        }
    }

    // Rebuild every kept face with its relocated corners substituted.
    let mut specs: Vec<FaceSpec> = Vec::with_capacity(kept_positions.len());
    let mut sources: Vec<FaceId> = Vec::with_capacity(kept_positions.len());
    for &pos in kept_positions {
        let fid = all_faces[pos];
        let face = topo.face(fid)?;
        let plane = planes[pos].ok_or_else(|| unsupported("kept face lost its plane"))?;

        let outer = substitute_wire(topo, face.outer_wire(), &moved)?;
        if outer.len() < 3 {
            // The face was consumed by the heal (e.g. a chamfer running the
            // full length of a narrow face). Dropping it is correct; if it was
            // not, the shell will not close and validation will refuse.
            continue;
        }

        let drop_slots: &[usize] = plan
            .drop_inner
            .iter()
            .find(|(p, _)| *p == pos)
            .map_or(&[], |(_, slots)| slots.as_slice());
        let mut inner_wires = Vec::new();
        for (slot, &wire_id) in face.inner_wires().iter().enumerate() {
            if drop_slots.contains(&slot) {
                continue;
            }
            let pts = substitute_wire(topo, wire_id, &moved)?;
            if pts.len() >= 3 {
                inner_wires.push(pts);
            }
        }

        // A relocated corner must stay on its own face's plane, otherwise the
        // "extension" bent the face instead of growing it.
        for p in outer.iter().chain(inner_wires.iter().flatten()) {
            if !plane.contains(*p, eps) {
                return Err(unsupported(format!(
                    "healing would pull face {} off its own plane; the removed \
                     patch does not meet its neighbours in a single corner",
                    fid.index()
                )));
            }
        }

        let normal = match face.surface() {
            FaceSurface::Plane { normal, .. } => normal.normalize()?,
            _ => return Err(unsupported("kept face is not planar")),
        };
        let d = dot_normal_point(normal, outer[0]);
        specs.push(if face.is_reversed() {
            FaceSpec::Surface {
                vertices: outer,
                surface: FaceSurface::Plane { normal, d },
                reversed: true,
                inner_wires,
            }
        } else {
            FaceSpec::Planar {
                vertices: outer,
                normal,
                d,
                inner_wires,
            }
        });
        sources.push(fid);
    }

    if specs.len() < 4 {
        return Err(unsupported(format!(
            "healing collapsed the shell to {} face(s); the adjacent faces do \
             not close over the gap",
            specs.len()
        )));
    }

    let assembly = assemble_solid_mixed_with_history(topo, &specs, tol)?;
    let face_map = sources
        .into_iter()
        .zip(assembly.faces_by_spec)
        .filter_map(|(source, result)| result.map(|face| (source.index(), face)))
        .collect();
    Ok(DefeatureOutcome {
        solid: assembly.solid,
        face_map,
    })
}

fn wound_edge_collapses(
    topo: &Topology,
    edge: &brepkit_topology::edge::Edge,
    edge_id: EdgeId,
    moved: &BTreeMap<usize, Point3>,
    wound: &Wound,
    collapse_tolerance: f64,
) -> Result<bool, OperationsError> {
    // A closed curve has real extent even though both topological endpoints
    // coincide. Reject both a shared vertex and distinct but coincident
    // vertices; either can make a circle or ellipse represent its full domain.
    if !wound.contains(edge_id)
        || (topo.vertex(edge.start())?.point() - topo.vertex(edge.end())?.point()).length()
            <= collapse_tolerance
    {
        return Ok(false);
    }
    let Some(start) = moved.get(&edge.start().index()) else {
        return Ok(false);
    };
    let Some(end) = moved.get(&edge.end().index()) else {
        return Ok(false);
    };
    Ok((*start - *end).length() <= collapse_tolerance)
}

/// Bounding-box diagonals of the removed patch and of the whole shell.
fn spans(
    topo: &Topology,
    all_faces: &[FaceId],
    removed: &[bool],
    adjacency: &Adjacency,
) -> Result<(f64, f64), OperationsError> {
    let mut patch: Option<(Point3, Point3)> = None;
    let mut model: Option<(Point3, Point3)> = None;
    for (&vertex, faces) in &adjacency.at_vertex {
        let vid = topo
            .vertex_id_from_index(vertex)
            .ok_or_else(|| unsupported(format!("vertex {vertex} disappeared")))?;
        let p = topo.vertex(vid)?.point();
        grow(&mut model, p);
        if faces.iter().any(|f| removed[*f]) {
            grow(&mut patch, p);
        }
    }
    let _ = all_faces;
    let diagonal = |b: Option<(Point3, Point3)>| b.map_or(0.0, |(lo, hi)| (hi - lo).length());
    Ok((diagonal(patch), diagonal(model)))
}

fn grow(bounds: &mut Option<(Point3, Point3)>, p: Point3) {
    *bounds = Some(match *bounds {
        None => (p, p),
        Some((lo, hi)) => (
            Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z())),
            Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z())),
        ),
    });
}

/// Find where a vertex of the removed patch ends up once the patch is gone.
///
/// `required` — the planes of the kept faces already meeting at the vertex —
/// must still pass through the answer, because those faces only grow within
/// their own planes. Candidate corners come from the planes bounding the
/// patch, taken from the patch faces at this vertex first and widened one hop
/// at a time while no corner has been found.
fn resolve_corner(
    faces_at_vertex: &[usize],
    planes: &[Option<Plane>],
    removed: &[bool],
    adjacency: &Adjacency,
    original: Point3,
    eps: f64,
) -> Result<Point3, OperationsError> {
    let mut required: Vec<Plane> = Vec::new();
    for &pos in faces_at_vertex {
        if let (false, Some(plane)) = (removed[pos], planes[pos]) {
            push_plane(&mut required, plane, eps);
        }
    }

    let mut frontier: Vec<usize> = faces_at_vertex
        .iter()
        .copied()
        .filter(|p| removed[*p])
        .collect();
    let mut visited: BTreeSet<usize> = frontier.iter().copied().collect();
    let mut candidates: Vec<Plane> = required.clone();

    loop {
        for &pos in &frontier {
            for &neighbour in &adjacency.neighbours[pos] {
                if let (false, Some(plane)) = (removed[neighbour], planes[neighbour]) {
                    push_plane(&mut candidates, plane, eps);
                }
            }
        }
        if candidates.len() > MAX_ADJACENT_PLANES {
            return Err(unsupported(format!(
                "the removed patch touches more than {MAX_ADJACENT_PLANES} distinct \
                 planes; defeaturing only heals small features"
            )));
        }

        match corners_on(&candidates, &required, eps) {
            Corners::One(p) => return Ok(p),
            Corners::Many => {
                return Err(unsupported(
                    "the faces around the removed patch close over the gap in more \
                     than one way; the selection is ambiguous"
                        .to_string(),
                ));
            }
            Corners::None => {}
        }

        // Widen through the patch by one hop and try again.
        let mut next = Vec::new();
        for &pos in &frontier {
            for &neighbour in &adjacency.neighbours[pos] {
                if removed[neighbour] && visited.insert(neighbour) {
                    next.push(neighbour);
                }
            }
        }
        if next.is_empty() {
            let _ = original;
            return Err(unsupported(
                "no three faces around the removed patch meet in a corner; the \
                 adjacent faces are parallel and would have to be merged rather \
                 than extended"
                    .to_string(),
            ));
        }
        frontier = next;
    }
}

fn push_plane(set: &mut Vec<Plane>, plane: Plane, eps: f64) {
    if !set.iter().any(|p| p.same_as(&plane, eps)) {
        set.push(plane);
    }
}

enum Corners {
    None,
    One(Point3),
    Many,
}

/// Corners of the `candidates` arrangement that lie on every `required` plane.
fn corners_on(candidates: &[Plane], required: &[Plane], eps: f64) -> Corners {
    let mut found: Option<Point3> = None;
    for i in 0..candidates.len() {
        for j in (i + 1)..candidates.len() {
            for k in (j + 1)..candidates.len() {
                let Some(p) = intersect_three_planes(candidates[i], candidates[j], candidates[k])
                else {
                    continue;
                };
                if !required.iter().all(|plane| plane.contains(p, eps)) {
                    continue;
                }
                match found {
                    None => found = Some(p),
                    Some(q) if (q - p).length() <= eps => {}
                    Some(_) => return Corners::Many,
                }
            }
        }
    }
    found.map_or(Corners::None, Corners::One)
}

/// Position of the first vertex on a face's outer wire, used to anchor the
/// face's plane equation to its actual geometry.
fn first_wire_vertex(topo: &Topology, face: FaceId) -> Result<Point3, OperationsError> {
    let wire = topo.wire(topo.face(face)?.outer_wire())?;
    let oe = wire.edges().first().ok_or_else(|| {
        unsupported(format!(
            "kept face {} has an empty outer wire",
            face.index()
        ))
    })?;
    let edge = topo.edge(oe.edge())?;
    Ok(topo.vertex(oe.oriented_start(edge))?.point())
}

/// Solve `n·p = d` for three planes, or `None` when they are too near-parallel
/// to define a corner.
fn intersect_three_planes(a: Plane, b: Plane, c: Plane) -> Option<Point3> {
    let bc = b.normal.cross(c.normal);
    let det = a.normal.dot(bc);
    if det.abs() < MIN_PLANE_TRIPLE_DET {
        return None;
    }
    let ca = c.normal.cross(a.normal);
    let ab = a.normal.cross(b.normal);
    let v = (bc * a.d + ca * b.d + ab * c.d) * (1.0 / det);
    Some(Point3::new(v.x(), v.y(), v.z()))
}

/// Walk a wire's vertices, substituting relocated corners and dropping
/// corners that collapsed onto their neighbour.
fn substitute_wire(
    topo: &Topology,
    wire: brepkit_topology::wire::WireId,
    moved: &BTreeMap<usize, Point3>,
) -> Result<Vec<Point3>, OperationsError> {
    let tol = Tolerance::new();
    let mut points: Vec<Point3> = Vec::new();
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        let vid = oe.oriented_start(edge);
        let p = moved
            .get(&vid.index())
            .copied()
            .unwrap_or(topo.vertex(vid)?.point());
        if points
            .last()
            .is_none_or(|last| (*last - p).length() > tol.linear)
        {
            points.push(p);
        }
    }
    while points.len() >= 2 && (points[0] - points[points.len() - 1]).length() <= tol.linear {
        points.pop();
    }
    Ok(points)
}

/// Auto-detect small features in a solid.
///
/// Returns face IDs of faces that are likely small features (holes, fillets)
/// based on their area being below the threshold.
///
/// # Errors
///
/// Returns an error if topology lookups or area computation fails.
pub fn detect_small_features(
    topo: &Topology,
    solid: SolidId,
    area_threshold: f64,
    deflection: f64,
) -> Result<Vec<FaceId>, OperationsError> {
    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;

    let mut small_faces = Vec::new();

    for &fid in shell.faces() {
        let area = crate::measure::face_area(topo, fid, deflection)?;
        if area < area_threshold {
            small_faces.push(fid);
        }
    }

    Ok(small_faces)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::primitives::make_box;
    use brepkit_math::curves::Circle3D;
    use brepkit_topology::edge::Edge;
    use brepkit_topology::vertex::Vertex;

    #[test]
    fn defeature_refuses_to_leave_an_open_shell() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let faces: Vec<FaceId> = shell.faces().to_vec();
        assert_eq!(faces.len(), 6);

        // Two faces of a box cannot be healed away: the four remaining side
        // planes are pairwise parallel and never close.
        let err = defeature(&mut topo, solid, &[faces[0], faces[1]]).unwrap_err();
        assert!(
            matches!(err, OperationsError::Unsupported { operation, .. } if operation == OP),
            "expected a typed refusal, got {err:?}"
        );
    }

    #[test]
    fn curved_wound_collapse_uses_rebuild_tolerance() {
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let circle =
            Circle3D::new(Point3::new(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let edge = topo.add_edge(Edge::new(start, end, EdgeCurve::Circle(circle)));
        let wound = Wound {
            kept_side: BTreeMap::from([(edge.index(), 0)]),
        };
        let moved = BTreeMap::from([
            (start.index(), Point3::new(1.0e9, 0.0, 0.0)),
            (end.index(), Point3::new(1.0e9 + 0.05, 0.0, 0.0)),
        ]);

        assert!(
            !wound_edge_collapses(
                &topo,
                topo.edge(edge).unwrap(),
                edge,
                &moved,
                &wound,
                Tolerance::new().linear,
            )
            .unwrap()
        );
    }

    #[test]
    fn closed_curved_wound_never_collapses_from_its_single_vertex() {
        let mut topo = Topology::new();
        let vertex = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let edge = topo.add_edge(Edge::new(vertex, vertex, EdgeCurve::Circle(circle)));
        let wound = Wound {
            kept_side: BTreeMap::from([(edge.index(), 0)]),
        };
        let moved = BTreeMap::from([(vertex.index(), Point3::new(0.0, 0.0, 0.0))]);

        assert!(
            !wound_edge_collapses(
                &topo,
                topo.edge(edge).unwrap(),
                edge,
                &moved,
                &wound,
                Tolerance::new().linear,
            )
            .unwrap()
        );
    }

    #[test]
    fn defeature_too_many_faces_error() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let solid_data = topo.solid(solid).unwrap();
        let shell = topo.shell(solid_data.outer_shell()).unwrap();
        let faces: Vec<FaceId> = shell.faces().to_vec();

        // Remove 3 faces — only 3 left, which is below minimum
        let result = defeature(&mut topo, solid, &[faces[0], faces[1], faces[2]]);
        assert!(result.is_err());
    }

    #[test]
    fn defeature_empty_selection_error() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result = defeature(&mut topo, solid, &[]);
        assert!(result.is_err());
    }

    #[test]
    fn defeature_rejects_face_outside_solid() {
        let mut topo = Topology::new();
        let a = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let b = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
        let foreign = topo
            .shell(topo.solid(b).unwrap().outer_shell())
            .unwrap()
            .faces()[0];

        let err = defeature(&mut topo, a, &[foreign]).unwrap_err();
        assert!(
            matches!(err, OperationsError::InvalidInput { .. }),
            "expected InvalidInput, got {err:?}"
        );
    }

    #[test]
    fn detect_no_small_features_in_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        // Box faces have area 4.0, threshold 0.1 should find nothing
        let small = detect_small_features(&topo, solid, 0.1, 0.1).unwrap();
        assert!(small.is_empty(), "box should have no small features");
    }

    #[test]
    fn detect_all_faces_with_large_threshold() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 0.01, 0.01, 0.01).unwrap();

        // Very small box — all faces should be below threshold 1.0
        let small = detect_small_features(&topo, solid, 1.0, 0.01).unwrap();
        assert_eq!(small.len(), 6, "all 6 faces of tiny box should be small");
    }
}
