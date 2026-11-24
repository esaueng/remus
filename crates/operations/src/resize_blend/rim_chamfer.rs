//! Chamfer bands on the rim of a moved planar cap (#1957).
//!
//! Moving a planar cap whose rim is chamfered must carry the chamfer: the
//! band translates with the cap and the walls it stands on lengthen or
//! shorten. Left to the planar offset engine, which keeps every unselected
//! support fixed, the chamfer planes are extended instead (a different, valid
//! solid), and cones turned round rounded corners refuse outright.
//!
//! A drafted face has the same local shape as a chamfer band, and a draft is
//! re-limited by the move, never carried. This module decides which bands
//! are chamfers.
//!
//! # Criterion
//!
//! A connected component of faces next to the selected cap `C` (outward
//! normal `n`, the move direction) is a **rim chamfer** when all of these
//! hold:
//!
//! 1. **It stands on walls.** Every face of the component is a plane or a
//!    cone about `n`, and every face it meets is the selection, another
//!    face of the component, or a *wall*: a plane parallel to `n` or a
//!    cylinder about `n`, which the move slides along. A tilted face that
//!    climbs to anything else (a frustum side reaching the far face) is a
//!    draft and is left to the planar engine.
//! 2. **Each face is an exact chamfer of one cap edge.** Each face meets `C`
//!    and exactly one wall `W` such that, with `S = plane(C) ∩ W` the sharp
//!    edge the chamfer replaced (a line, or a circle about `n`):
//!    - every edge it shares with `C` is a line parallel to `S` (a circle
//!      about the cylinder's axis) inside the material at one setback `d_c`
//!      from `S`;
//!    - every edge it shares with `W` is a line parallel to `S` (a circle on
//!      the cylinder) at one depth `d_w` below the cap plane;
//!    - a plane face runs along `S`, a cone face is coaxial with the
//!      cylinder.
//!
//!    Exact edge curves only: sampled or spline edges never qualify.
//! 3. **It is a chamfer, not a draft.** Geometry alone cannot tell a
//!    two-distance chamfer from a drafted section standing on straight walls
//!    (equal setbacks describe a 45° draft as well as a symmetric chamfer),
//!    so the face's identity comes from the journal:
//!    - when the journal traces every face of the component, through the
//!      entries that modified or preserved it and with no barrier after the
//!      entry that made it, to that entry, the band is a chamfer exactly
//!      when every such entry is a `chamfer` (any distances; distance-angle
//!      chamfers are two-distance chamfers), and a draft otherwise. A face
//!      the entry *generated* counts; so does one it left *unresolved*
//!      (a chamfer cannot attribute the cones it turns round rounded
//!      corners), but only in a construction record that names the faces it
//!      kept, so a record that names nothing proves nothing;
//!    - without such a record the band is **ambiguous**. It is read as the
//!      caller's [`AmbiguousRimResolution`] says: carried as a chamfer,
//!      re-limited as a draft, or (the default) refused with
//!      [`crate::OperationsError::AmbiguousRimChamfer`] naming the band.
//!      Any unjournaled edit followed by a journaled operation leaves a
//!      global barrier, so history is conservative: a band whose record it
//!      severs is ambiguous again.
//!
//! A complete cone between the cap and one coaxial cylinder (a hole mouth
//! or a boss rim, [`super::cap_chamfer::is_axial_cap_chamfer`]) keeps the
//! rule of #1808: it travels with the cap whatever its setbacks.
//!
//! A component that fails 1 is not a rim band at all and is ignored here. A
//! component that passes 1 but fails 2, or is read as a draft by 3, is
//! *declined*: it is not carried,
//! and a move that carries another chamfer of the same cap refuses rather
//! than translating it (the translation cannot carry one band and re-limit
//! another).

use crate::OperationsError;
pub(super) use crate::push_pull::AmbiguousRimBand;
use crate::push_pull::AmbiguousRimResolution;
use remus_math::compact_sort::CompactSort;
use remus_math::det_hash::DetHashSet;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::{
    EntityEvent, EntityKey, EntryPayload, JournalOrdinal, RecordedOrigin,
};
use std::collections::{BTreeMap, VecDeque};

/// Limit on the number of faces rebuilt by a single carried chamfer band.
const MAX_BAND_FACES: usize = 64;

/// Absolute floor of the tolerance on setbacks and depths.
const SETBACK_TOLERANCE: f64 = 1e-9;

/// Rounding allowance per unit of coordinate magnitude: setbacks are
/// differences of coordinates, so far from the origin they carry rounding
/// of the coordinates' size, never more.
const ROUNDING_PER_UNIT: f64 = 1024.0 * f64::EPSILON;

/// How many journal links a face's origin may be followed through.
const MAX_LINEAGE_STEPS: usize = 4096;

/// The rim chamfer bands of a cap selection.
#[derive(Debug, Default)]
pub(super) struct RimChamfers {
    /// Faces of every component recognized as a rim chamfer, in face order.
    pub(super) carried: Vec<FaceId>,
    largest_band: usize,
    /// One face and the reason for every component that stands on walls like
    /// a chamfer but is not recognized as one, in face order.
    pub(super) declined: Vec<(FaceId, String)>,
    /// One band of every component no history reads as a chamfer or a
    /// draft, when the caller left the decision to the move
    /// ([`AmbiguousRimResolution::Refuse`]), in face order.
    pub(super) ambiguous: Vec<AmbiguousRimBand>,
}

/// What criteria 2 and 3 make of a component.
enum Identity {
    Chamfer,
    NotChamfer(String),
    Ambiguous(AmbiguousRimBand),
}

/// The setbacks of one face read as an exact chamfer of one cap edge.
#[derive(Clone, Copy)]
struct Setbacks {
    cap_face: FaceId,
    cap_edge: EdgeId,
    wall: FaceId,
    cap: f64,
    wall_depth: f64,
}

impl RimChamfers {
    /// Enforce the rebuild limit when executing with the current lineage.
    pub(super) fn validate_carry_limit(&self) -> Result<(), OperationsError> {
        if self.largest_band > MAX_BAND_FACES {
            return Err(super::reconstruction(format!(
                "rim chamfer has {} faces, above the carry limit of {MAX_BAND_FACES}",
                self.largest_band
            )));
        }
        Ok(())
    }
}

/// The selected cap's plane: outward normal and offset `n · p`.
#[derive(Clone, Copy)]
struct Cap {
    normal: Vec3,
    level: f64,
}

fn scale_of(point: Point3) -> f64 {
    point
        .x()
        .abs()
        .max(point.y().abs())
        .max(point.z().abs())
        .max(1.0)
}

/// The tolerance on setbacks and depths read near `point`: a fixed floor,
/// widened only by the rounding of coordinates far from the origin, so a
/// translated body reads the same setbacks.
fn tolerance_at(point: Point3) -> f64 {
    SETBACK_TOLERANCE.max(ROUNDING_PER_UNIT * scale_of(point))
}

fn close(a: f64, b: f64, point: Point3) -> bool {
    (a - b).abs() <= tolerance_at(point)
}

fn dot(point: Point3, normal: Vec3) -> f64 {
    normal.dot(point - Point3::new(0.0, 0.0, 0.0))
}

fn is_wall(surface: &FaceSurface, direction: Vec3) -> bool {
    !matches!(surface, FaceSurface::Nurbs(_))
        && super::surface_translation_invariant(surface, direction)
}

/// The rim chamfer bands next to `selected` (coplanar planar faces facing
/// `direction`), ambiguous bands read by the resolution in force
/// ([`crate::push_pull::ambiguous_rim_resolution`]). An empty result for any
/// other selection.
pub(super) fn rim_chamfers(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    selected: &[FaceId],
    direction: Vec3,
) -> Result<RimChamfers, OperationsError> {
    let mut ordered = selected.to_vec();
    ordered.compact_sort_unstable_by_key(|face| face.index());
    ordered.dedup();
    let Some(cap) = cap_plane(topo, &ordered, direction)? else {
        return Ok(RimChamfers::default());
    };
    let is_selected = |face: FaceId| ordered.binary_search(&face).is_ok();

    let mut seeds = Vec::new();
    for &face in &ordered {
        for edge in super::face_edges(topo, face)? {
            for adjacent in super::distinct_faces(adjacency.faces_for_edge(edge)) {
                if !is_selected(adjacent) && !is_wall(topo.face(adjacent)?.surface(), direction) {
                    seeds.push(adjacent);
                }
            }
        }
    }
    seeds.compact_sort_unstable_by_key(|face| face.index());
    seeds.dedup();

    let mut result = RimChamfers::default();
    let mut visited = DetHashSet::default();
    for seed in seeds {
        if !visited.insert(seed) {
            continue;
        }
        let mut component = vec![seed];
        let mut queue = VecDeque::from([seed]);
        let mut stands_on_walls = true;
        while let Some(current) = queue.pop_front() {
            if !is_band_surface(topo.face(current)?.surface(), direction) {
                stands_on_walls = false;
            }
            for edge in super::face_edges(topo, current)? {
                for adjacent in super::distinct_faces(adjacency.faces_for_edge(edge)) {
                    if adjacent == current
                        || is_selected(adjacent)
                        || is_wall(topo.face(adjacent)?.surface(), direction)
                    {
                        continue;
                    }
                    if visited.insert(adjacent) {
                        component.push(adjacent);
                        queue.push_back(adjacent);
                    }
                }
            }
        }
        if !stands_on_walls {
            continue;
        }
        component.compact_sort_unstable_by_key(|face| face.index());
        // A complete coaxial cone travels with the cap whatever its history
        // (#1808); the blend walk carries it.
        if let [face] = component.as_slice()
            && super::cap_chamfer::is_axial_cap_chamfer(
                topo,
                adjacency,
                is_selected,
                *face,
                direction,
            )?
        {
            result.carried.push(*face);
            continue;
        }
        match component_identity(topo, adjacency, &is_selected, &component, cap)? {
            Identity::Chamfer => {
                result.largest_band = result.largest_band.max(component.len());
                result.carried.extend(component);
            }
            Identity::NotChamfer(reason) => result.declined.push((component[0], reason)),
            Identity::Ambiguous(band) => match crate::push_pull::ambiguous_rim_resolution() {
                AmbiguousRimResolution::TreatAsChamfer => {
                    result.largest_band = result.largest_band.max(component.len());
                    result.carried.extend(component);
                }
                AmbiguousRimResolution::TreatAsDraft => result.declined.push((
                    component[0],
                    format!(
                        "face {} has no history and the caller reads it as a draft",
                        component[0].index()
                    ),
                )),
                AmbiguousRimResolution::Refuse => result.ambiguous.push(band),
            },
        }
    }
    result
        .carried
        .compact_sort_unstable_by_key(|face| face.index());
    Ok(result)
}

/// A plane tilted to the move or a cone about it: the surfaces a rim
/// chamfer cuts.
fn is_band_surface(surface: &FaceSurface, direction: Vec3) -> bool {
    let angular = Tolerance::new().angular;
    match surface {
        FaceSurface::Plane { normal, .. } => {
            let along = normal.dot(direction).abs() / normal.length();
            along > angular && along < 1.0 - angular
        }
        FaceSurface::Cone(cone) => cone.axis().cross(direction).length() <= angular,
        FaceSurface::Cylinder(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => false,
    }
}

/// The plane shared by every selected face, when each is planar with outward
/// normal `direction`.
fn cap_plane(
    topo: &Topology,
    selected: &[FaceId],
    direction: Vec3,
) -> Result<Option<Cap>, OperationsError> {
    let mut level: Option<(f64, Point3)> = None;
    for &face in selected {
        let data = topo.face(face)?;
        let Some(normal) = data.effective_plane_normal() else {
            return Ok(None);
        };
        if normal.dot(direction) < 1.0 - Tolerance::new().angular {
            return Ok(None);
        }
        let Some(&vertex) = remus_topology::explorer::face_vertices(topo, face)?.first() else {
            return Ok(None);
        };
        let point = topo.vertex(vertex)?.point();
        let here = dot(point, direction);
        if let Some((before, _)) = level
            && !close(before, here, point)
        {
            return Ok(None);
        }
        level.get_or_insert((here, point));
    }
    Ok(level.map(|(level, _)| Cap {
        normal: direction,
        level,
    }))
}

/// What criteria 2 and 3 make of `component`.
fn component_identity(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    is_selected: &impl Fn(FaceId) -> bool,
    component: &[FaceId],
    cap: Cap,
) -> Result<Identity, OperationsError> {
    let mut first = None;
    for &face in component {
        let Some(found) = chamfer_setbacks(topo, adjacency, is_selected, face, cap)? else {
            return Ok(Identity::NotChamfer(format!(
                "face {} is not a constant-setback chamfer of one cap edge on a wall",
                face.index()
            )));
        };
        first.get_or_insert((face, found));
    }
    let Some((face, setbacks)) = first else {
        return Ok(Identity::NotChamfer("empty band".into()));
    };

    // History, when the journal holds the origin of every face: each came
    // out of a chamfer entry.
    let origins: Option<Vec<Origin>> = component
        .iter()
        .map(|&face| journal_origin(topo, face))
        .collect();
    if let Some(origins) = origins {
        if let Some((face, origin)) = component
            .iter()
            .zip(&origins)
            .find(|(_, origin)| origin.kind != "chamfer")
        {
            return Ok(Identity::NotChamfer(format!(
                "face {} came out of a `{}` operation, not a chamfer: a draft is re-limited",
                face.index(),
                origin.kind
            )));
        }
        return Ok(Identity::Chamfer);
    }

    // No history: a chamfer and a draft on walls have this one shape.
    Ok(Identity::Ambiguous(AmbiguousRimBand {
        cap: setbacks.cap_face,
        face,
        wall: setbacks.wall,
        edge: setbacks.cap_edge,
        cap_setback: setbacks.cap,
        wall_setback: setbacks.wall_depth,
    }))
}

fn edge_ends(topo: &Topology, edge: EdgeId) -> Result<[Point3; 2], OperationsError> {
    let data = topo.edge(edge)?;
    Ok([
        topo.vertex(data.start())?.point(),
        topo.vertex(data.end())?.point(),
    ])
}

/// The cap and wall setbacks of `face` when it is an exact chamfer of one
/// edge between the cap and a wall (criterion 2), else `None`.
#[allow(clippy::too_many_lines)]
fn chamfer_setbacks(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    is_selected: &impl Fn(FaceId) -> bool,
    face: FaceId,
    cap: Cap,
) -> Result<Option<Setbacks>, OperationsError> {
    let surface = topo.face(face)?.surface().clone();
    let mut cap_edges = Vec::new();
    let mut cap_face = None;
    let mut wall_edges: BTreeMap<FaceId, Vec<EdgeId>> = BTreeMap::new();
    for edge in super::face_edges(topo, face)? {
        for other in super::distinct_faces(adjacency.faces_for_edge(edge)) {
            if other == face {
                continue;
            }
            if is_selected(other) {
                cap_edges.push(edge);
                cap_face.get_or_insert(other);
            } else if is_wall(topo.face(other)?.surface(), cap.normal) {
                wall_edges.entry(other).or_default().push(edge);
            }
        }
    }
    let (Some(&cap_edge), Some(cap_face)) = (cap_edges.first(), cap_face) else {
        return Ok(None);
    };
    for &edge in &cap_edges {
        for point in edge_ends(topo, edge)? {
            if !close(dot(point, cap.normal), cap.level, point) {
                return Ok(None);
            }
        }
    }

    let mut found = None;
    for (&wall, edges) in &wall_edges {
        let wall_surface = topo.face(wall)?.surface();
        // A plane chamfer stands on a plane wall, a cone on a cylinder; any
        // other pairing explains nothing.
        let setbacks = if let (FaceSurface::Plane { .. }, FaceSurface::Plane { normal, .. }) =
            (wall_surface, &surface)
        {
            let Some(outward) = topo.face(wall)?.effective_plane_normal() else {
                continue;
            };
            let anchor = edge_ends(topo, edges[0])?[0];
            plane_wall_setbacks(topo, &cap_edges, edges, cap, outward, anchor, *normal)?
        } else if let (FaceSurface::Cylinder(cylinder), FaceSurface::Cone(cone)) =
            (wall_surface, &surface)
        {
            let outward = if topo.face(wall)?.is_reversed() {
                -1.0
            } else {
                1.0
            };
            let apex_off = (cone.apex() - cylinder.origin()).cross(cylinder.axis());
            if apex_off.length() > tolerance_at(cone.apex()) {
                continue;
            }
            cylinder_wall_setbacks(
                topo,
                &cap_edges,
                edges,
                cap,
                cylinder.origin(),
                cylinder.axis(),
                cylinder.radius(),
                outward,
            )?
        } else {
            None
        };
        if let Some((cap, wall_depth)) = setbacks {
            if found.is_some() {
                // Two walls explain the face: not a chamfer of one edge.
                return Ok(None);
            }
            found = Some(Setbacks {
                cap_face,
                cap_edge,
                wall,
                cap,
                wall_depth,
            });
        }
    }
    Ok(found)
}

/// Setbacks of a plane chamfer against a plane wall with outward normal
/// `outward` through `anchor`.
fn plane_wall_setbacks(
    topo: &Topology,
    cap_edges: &[EdgeId],
    wall_edges: &[EdgeId],
    cap: Cap,
    outward: Vec3,
    anchor: Point3,
    face_normal: Vec3,
) -> Result<Option<(f64, f64)>, OperationsError> {
    // The sharp edge runs along the cap plane and the wall.
    let along = cap.normal.cross(outward);
    if along.length() <= Tolerance::new().angular
        || face_normal.dot(along).abs() > Tolerance::new().angular * face_normal.length()
    {
        return Ok(None);
    }
    let mut d_cap: Option<f64> = None;
    for &edge in cap_edges {
        if !matches!(topo.edge(edge)?.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        for point in edge_ends(topo, edge)? {
            // Inside the wall's material, at one setback.
            let setback = -(point - anchor).dot(outward);
            if setback <= tolerance_at(point)
                || d_cap.is_some_and(|before| !close(before, setback, point))
            {
                return Ok(None);
            }
            d_cap.get_or_insert(setback);
        }
    }
    let mut d_wall: Option<f64> = None;
    for &edge in wall_edges {
        if !matches!(topo.edge(edge)?.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        for point in edge_ends(topo, edge)? {
            let depth = cap.level - dot(point, cap.normal);
            if depth <= tolerance_at(point)
                || d_wall.is_some_and(|before| !close(before, depth, point))
            {
                return Ok(None);
            }
            d_wall.get_or_insert(depth);
        }
    }
    Ok(d_cap.zip(d_wall))
}

/// Setbacks of a cone chamfer against a cylinder wall about `axis`;
/// `outward` is `1` when the wall's material lies inside the cylinder (a
/// boss or a rounded corner) and `-1` when it lies outside (a bore).
#[allow(clippy::too_many_arguments)]
fn cylinder_wall_setbacks(
    topo: &Topology,
    cap_edges: &[EdgeId],
    wall_edges: &[EdgeId],
    cap: Cap,
    origin: Point3,
    axis: Vec3,
    radius: f64,
    outward: f64,
) -> Result<Option<(f64, f64)>, OperationsError> {
    let coaxial = |center: Point3, normal: Vec3| {
        normal.cross(axis).length() <= Tolerance::new().angular
            && (center - origin).cross(axis).length() <= tolerance_at(center)
    };
    let mut d_cap: Option<f64> = None;
    for &edge in cap_edges {
        let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() else {
            return Ok(None);
        };
        let center = circle.center();
        let setback = outward * (radius - circle.radius());
        if !coaxial(center, circle.normal())
            || setback <= tolerance_at(center)
            || d_cap.is_some_and(|before| !close(before, setback, center))
        {
            return Ok(None);
        }
        d_cap.get_or_insert(setback);
    }
    let mut d_wall: Option<f64> = None;
    for &edge in wall_edges {
        let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() else {
            return Ok(None);
        };
        let center = circle.center();
        let depth = cap.level - dot(center, cap.normal);
        if !coaxial(center, circle.normal())
            || !close(circle.radius(), radius, center)
            || depth <= tolerance_at(center)
            || d_wall.is_some_and(|before| !close(before, depth, center))
        {
            return Ok(None);
        }
        d_wall.get_or_insert(depth);
    }
    Ok(d_cap.zip(d_wall))
}

/// Where the journal says a face came from.
struct Origin {
    /// Kind of the entry that made the face.
    kind: String,
}

/// The journal entry that made `face`, followed back through the entries
/// that modified or preserved it, when the journal holds that record with no
/// barrier after the entry. `None` when the face's origin is not on record.
fn journal_origin(topo: &Topology, face: FaceId) -> Option<Origin> {
    let journal = topo.journal();
    let mut ordinal = journal.ordinal_of(EntityKey::face(face.index()))?;
    let mut chain: Vec<JournalOrdinal> = Vec::new();
    for _ in 0..MAX_LINEAGE_STEPS {
        chain.push(ordinal);
        // The subject's oldest event that names an origin other than itself
        // is the one that made it.
        let (op, event) = journal.events_for(ordinal).into_iter().find(|(_, event)| {
            !matches!(
                event,
                EntityEvent::Preserved { from } | EntityEvent::Modified { from } if *from == ordinal
            )
        })?;
        let generated = match event {
            EntityEvent::Generated { .. } => true,
            EntityEvent::Unresolved { candidates } if candidates.is_empty() => false,
            EntityEvent::Preserved { from } | EntityEvent::Modified { from } => {
                ordinal = *from;
                continue;
            }
            _ => return None,
        };
        let entry = journal.entries().iter().find(|entry| entry.op() == op)?;
        let severed = chain.iter().any(|&link| {
            journal
                .barriers_crossing(link)
                .into_iter()
                .any(|barrier| barrier > op)
        });
        if severed {
            return None;
        }
        // A blend leaves the new faces it cannot attribute to an input (a
        // chamfer's corner cones) unresolved, but so does a record that
        // names nothing at all. An unresolved face counts as made by the
        // entry only in a construction record that accounts for the faces
        // it kept.
        if !generated {
            let EntryPayload::Evolution { origin, events, .. } = entry.payload() else {
                return None;
            };
            let names_survivors = events
                .iter()
                .any(|(_, event)| matches!(event, EntityEvent::Modified { .. }));
            if *origin != RecordedOrigin::Construction || !names_survivors {
                return None;
            }
        }
        return Some(Origin {
            kind: entry.kind().to_owned(),
        });
    }
    None
}
