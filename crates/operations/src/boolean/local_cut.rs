//! Local drilled-hole cut: the first qualified cells of a boolean whose cost
//! follows the region the tool touches instead of the whole blank.
//!
//! The general GFA re-processes every face of the blank on every cut, so a
//! plate drilled one hole at a time pays for all earlier holes again
//! (`docs/performance/perf-local-boolean.md`). This module recognizes the
//! commonest sequential cuts — a right circular cylinder drilled straight
//! through two parallel planar faces, or into one planar face down to a
//! floor inside the blank, clear of every other piece of the blank — and
//! builds the exact GFA result by local surgery on a copy. Each pierced face
//! gains one circular inner wire, one cylindrical wall face lines the hole,
//! and a blind hole keeps the tool's buried cap as its floor. Every other
//! face is copied unchanged.
//!
//! The cells are fail-closed. Anything they cannot prove returns `None` and
//! the caller runs the general GFA, so a refusal costs one pass over the
//! blank's conservative face boxes and changes no result.
//!
//! # Why the result is exact
//!
//! Let the tool `B` be the solid cylinder of radius `r` about axis
//! `o + s·d`, `s ∈ [s0, s1]`, and let `P` be the blank faces whose boxes
//! come within the clearance of `B`'s box. The cells prove:
//!
//! 1. every face of `A` outside `P` has a conservative box at least the
//!    clearance away from `B`'s box, so it does not meet `B`;
//! 2. each face of `P` is a plane perpendicular to `d` at some `s` strictly
//!    inside `(s0, s1)`, and its section disc (radius `r` about the axis
//!    point) lies inside the face region with at least the clearance to
//!    every boundary edge, so `plane ∩ B` meets `∂A` exactly in that disc;
//! 3. the outward normals of the faces of `P` put the material on the side
//!    each cell needs.
//!
//! By (1) and (2), `∂A ∩ B` is the union of the discs. They cut `B` into
//! slabs, each wholly inside or wholly outside `A`, and the outward normal
//! of a disc's face says which of its two sides is inside.
//!
//! - **Through hole**: `P = {L, U}` at `s_L < s_U`, with `L`'s outward
//!   normal `−d` and `U`'s `+d`. Only the slab between the discs is inside,
//!   so `A − B` is `A` with both discs removed and that slab's lateral
//!   surface added as a reversed wall.
//! - **Blind hole**: `P = {E}`. The slab on the side opposite `E`'s outward
//!   normal is inside and ends at one of the tool's caps, which (1) puts
//!   strictly inside `A`. So `A − B` is `A` with `E`'s disc removed, a
//!   reversed wall from `E` to that cap, and the cap, reversed, as the
//!   hole's floor.

use remus_math::aabb::Aabb3;
use remus_math::curves::Circle3D;
use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use crate::copy::{ShellSlot, Splice, SpliceEdge, SpliceFace, SpliceWire};

/// Clearance, in linear tolerances, the cell demands between the tool and
/// every piece of geometry it must not touch. Contacts closer than this are
/// left to the general GFA, which owns tolerance-scale coincidence.
const CLEARANCE_TOLERANCES: f64 = 1.0e3;

/// Most chords one arc may be sampled into for the containment test.
const MAX_ARC_SEGMENTS: usize = 4096;

/// Cosine bound for "perpendicular to the tool axis".
const PARALLEL_COSINE: f64 = 1.0 - 1.0e-12;

/// `a > b`, and `false` when either is NaN: every qualification test is
/// written as "decline unless strictly greater", so a NaN fails closed.
fn greater(a: f64, b: f64) -> bool {
    a > b
}

#[cfg(test)]
thread_local! {
    /// Lets tests compare the local cell against the general GFA path.
    pub(super) static DISABLED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Cuts this thread built locally, so tests can tell which path ran.
    pub(super) static BUILT: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// A right circular cylinder recognized as a tool.
struct ToolCylinder {
    surface: CylindricalSurface,
    origin: Point3,
    axis: Vec3,
    radius: f64,
    /// Axial extent `(s0, s1)` of the caps along `axis` from `origin`.
    span: (f64, f64),
    /// Radial direction of the tool's own seam. The GFA starts the rims it
    /// cuts there, so the local rims do too: a later boolean that meets the
    /// hole (a plug filling it, a coaxial counterbore) then sees the same
    /// seam vertices either way.
    reference: Vec3,
    /// The wall surface, for the GFA's seam anchoring arithmetic.
    wall: FaceSurface,
    /// The start of the wall's seam.
    seam_point: Point3,
    /// The cap rims at `s0` and `s1`, when the wall's loop is the natural
    /// `(u, v)` rectangle — `s0` rim forward, seam up, `s1` rim back, seam
    /// down — that a blind hole's wall and floor are built from.
    rims: Option<[ToolRim; 2]>,
}

/// One cap of the tool and the circle it shares with the wall.
#[derive(Clone, Copy)]
struct ToolRim {
    /// The cap face.
    cap: FaceId,
    /// The cap's one edge, a closed circle the wall also uses.
    edge: EdgeId,
    /// Whether the cap's wire uses `edge` forward.
    cap_forward: bool,
    /// The edge's vertex, where the wall's seam meets it.
    vertex: VertexId,
}

/// A blank face the tool pierces.
struct PiercedFace {
    face: FaceId,
    /// Axial coordinate of the face plane along the tool axis.
    s: f64,
    /// The face's (outward, unreversed) plane normal.
    normal: Vec3,
}

/// A qualified cut.
enum Cell {
    /// Through two pierced faces, ordered by axial position.
    Through {
        lower: PiercedFace,
        upper: PiercedFace,
    },
    /// Into one pierced face, down to the tool's cap rim `floor`
    /// (0 at `s0`, 1 at `s1`).
    Blind { entry: PiercedFace, floor: usize },
}

impl std::fmt::Debug for Cell {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Through { lower, upper } => write!(f, "through {lower:?}/{upper:?}"),
            Self::Blind { entry, floor } => write!(f, "blind {entry:?} to cap {floor}"),
        }
    }
}

/// Cut `b` out of `a` locally when the pair lies in a drilled-hole cell.
///
/// Returns `Ok(None)` — having changed nothing — whenever no cell applies
/// or its result fails the strict boolean-result gate; the caller then runs
/// the general GFA.
///
/// # Errors
///
/// Returns an error only for a failed topology lookup while reading the
/// operands; construction failures roll back and decline instead.
pub(super) fn try_local_hole(
    topo: &mut Topology,
    a: SolidId,
    b: SolidId,
    tol: Tolerance,
) -> Result<Option<SolidId>, crate::OperationsError> {
    #[cfg(test)]
    if DISABLED.with(std::cell::Cell::get) {
        return Ok(None);
    }
    if a == b {
        return Ok(None);
    }
    let Some(tool) = recognize_tool(topo, b, tol)? else {
        return Ok(None);
    };
    let Some(cell) = qualify_blank(topo, a, b, &tool, tol)? else {
        return Ok(None);
    };
    let attempt = remus_topology::transaction::run_transacted(topo, |topo| {
        let splice = match &cell {
            Cell::Through { lower, upper } => {
                through_hole_splice(topo, a, &tool, lower, upper, tol)?
            }
            Cell::Blind { entry, floor } => blind_hole_splice(topo, a, &tool, entry, *floor, tol)?,
        };
        let result = crate::copy::copy_solid_spliced(topo, a, &splice)?;
        super::assembly::validate_boolean_result_with_tolerance(topo, result, tol)?;
        Ok::<_, crate::OperationsError>(result)
    });
    match attempt {
        Ok(result) => {
            log::debug!("Cut built locally: {cell:?}");
            #[cfg(test)]
            BUILT.with(|built| built.set(built.get() + 1));
            Ok(Some(result))
        }
        Err(error) => {
            log::debug!("local hole cut declined after construction: {error}");
            Ok(None)
        }
    }
}

impl std::fmt::Debug for PiercedFace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}@{}", self.face.index(), self.s)
    }
}

/// Recognize `b` as an untrimmed right circular cylinder: one forward
/// cylindrical wall and two forward planar caps perpendicular to its axis,
/// no inner wires or shells.
fn recognize_tool(
    topo: &Topology,
    b: SolidId,
    tol: Tolerance,
) -> Result<Option<ToolCylinder>, crate::OperationsError> {
    let solid = topo.solid(b)?;
    if !solid.inner_shells().is_empty() {
        return Ok(None);
    }
    let faces = topo.shell(solid.outer_shell())?.faces();
    if faces.len() != 3 {
        return Ok(None);
    }
    let mut wall = None;
    let mut caps = Vec::with_capacity(2);
    for &face_id in faces {
        let face = topo.face(face_id)?;
        if face.is_reversed() || !face.inner_wires().is_empty() {
            return Ok(None);
        }
        match face.surface() {
            FaceSurface::Cylinder(cylinder) if wall.is_none() => {
                wall = Some((cylinder.clone(), face.outer_wire()));
            }
            FaceSurface::Plane { normal, d } => caps.push((face_id, *normal, *d)),
            FaceSurface::Cylinder(_)
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => return Ok(None),
        }
    }
    let (Some((surface, wall_wire)), [cap_a, cap_b]) = (wall, caps.as_slice()) else {
        return Ok(None);
    };
    // The wall's seam is its one straight edge.
    let mut seam_point = None;
    for oriented in topo.wire(wall_wire)?.edges() {
        let edge = topo.edge(oriented.edge())?;
        if matches!(edge.curve(), EdgeCurve::Line) {
            seam_point = Some(topo.vertex(edge.start())?.point());
            break;
        }
    }
    let Some(seam_point) = seam_point else {
        return Ok(None);
    };
    let axis = surface.axis();
    let origin = surface.origin();
    let radius = surface.radius();
    if !(radius.is_finite() && greater(radius, tol.linear)) || (axis.length() - 1.0).abs() > 1e-12 {
        return Ok(None);
    }
    let mut placed = [(cap_a.0, 0.0), (cap_b.0, 0.0)];
    for (slot, &(_, normal, d)) in placed.iter_mut().zip([cap_a, cap_b]) {
        let Some(s) = axial_plane_position(origin, axis, normal, d) else {
            return Ok(None);
        };
        slot.1 = s;
    }
    if placed[1].1 < placed[0].1 {
        placed.swap(0, 1);
    }
    let span = (placed[0].1, placed[1].1);
    if !greater(span.1 - span.0, tol.linear) {
        return Ok(None);
    }
    let radial = seam_point - origin;
    let reference = radial - axis * radial.dot(axis);
    if !greater(reference.length(), 0.5 * radius) {
        return Ok(None);
    }
    let rims = natural_rims(topo, wall_wire, [placed[0].0, placed[1].0])?;
    Ok(Some(ToolCylinder {
        wall: FaceSurface::Cylinder(surface.clone()),
        surface,
        origin,
        axis,
        radius,
        span,
        reference,
        seam_point,
        rims,
    }))
}

/// The cap rims of a tool whose wall loop is the natural rectangle
/// `[s0 rim +, seam +, s1 rim −, seam −]`, each rim a closed circle that is
/// its cap's only edge and starts where the seam meets it; `None` for any
/// other wall loop.
fn natural_rims(
    topo: &Topology,
    wall_wire: remus_topology::wire::WireId,
    caps: [FaceId; 2],
) -> Result<Option<[ToolRim; 2]>, crate::OperationsError> {
    let uses = topo.wire(wall_wire)?.edges();
    let [lower, seam_up, upper, seam_down] = uses else {
        return Ok(None);
    };
    if !(lower.is_forward()
        && seam_up.is_forward()
        && !upper.is_forward()
        && !seam_down.is_forward()
        && seam_up.edge() == seam_down.edge())
    {
        return Ok(None);
    }
    let seam = topo.edge(seam_up.edge())?;
    if !matches!(seam.curve(), EdgeCurve::Line) {
        return Ok(None);
    }
    let mut rims = Vec::with_capacity(2);
    for ((cap, rim_use), seam_end) in caps
        .into_iter()
        .zip([lower, upper])
        .zip([seam.start(), seam.end()])
    {
        let edge = topo.edge(rim_use.edge())?;
        if !matches!(edge.curve(), EdgeCurve::Circle(_))
            || edge.start() != seam_end
            || edge.end() != seam_end
        {
            return Ok(None);
        }
        let cap_uses = topo.wire(topo.face(cap)?.outer_wire())?.edges();
        let [cap_use] = cap_uses else {
            return Ok(None);
        };
        if cap_use.edge() != rim_use.edge() {
            return Ok(None);
        }
        rims.push(ToolRim {
            cap,
            edge: rim_use.edge(),
            cap_forward: cap_use.is_forward(),
            vertex: seam_end,
        });
    }
    Ok(<[ToolRim; 2]>::try_from(rims).ok())
}

/// Axial coordinate along `origin + s·axis` where the plane `normal·x = d`
/// crosses it, when the plane is perpendicular to the axis.
fn axial_plane_position(origin: Point3, axis: Vec3, normal: Vec3, d: f64) -> Option<f64> {
    if (normal.length() - 1.0).abs() > 1e-12 {
        return None;
    }
    let cosine = normal.dot(axis);
    if cosine.abs() < PARALLEL_COSINE {
        return None;
    }
    let s = (d - normal.dot(origin - Point3::new(0.0, 0.0, 0.0))) / cosine;
    s.is_finite().then_some(s)
}

/// Prove the blank side of a cell: two pierced faces with the material
/// between them, or one pierced face with the material on the side of a
/// tool cap the wall can end on.
fn qualify_blank(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
    tool: &ToolCylinder,
    tol: Tolerance,
) -> Result<Option<Cell>, crate::OperationsError> {
    let solid = topo.solid(a)?;
    if !solid.inner_shells().is_empty() {
        return Ok(None);
    }
    let faces = topo.shell(solid.outer_shell())?.faces();
    let Some(entity_tolerance) = max_entity_tolerance(topo, a)
        .zip(max_entity_tolerance(topo, b))
        .map(|(ta, tb)| ta.max(tb))
    else {
        return Ok(None);
    };
    let clearance = CLEARANCE_TOLERANCES * tol.linear + entity_tolerance;
    let reach = tool_box(tool).expanded(clearance);

    let mut candidates = Vec::with_capacity(2);
    for &face_id in faces {
        let bounds = match ruled_face_bound(topo, face_id)? {
            Some(bounds) => bounds,
            None => crate::measure::conservative_face_set_bounding_box(topo, &[face_id])?,
        };
        if !reach.intersects(bounds) {
            continue;
        }
        if candidates.len() == 2 || candidates.contains(&face_id) {
            return Ok(None);
        }
        candidates.push(face_id);
    }
    match *candidates.as_slice() {
        [entry] => {
            let (Some(entry), Some(_)) = (
                pierced_face(topo, entry, tool, clearance, tol)?,
                tool.rims.as_ref(),
            ) else {
                return Ok(None);
            };
            // The material lies against the outward normal, so the wall
            // runs from the entry plane to the cap on that side.
            let toward_axis = entry.normal.dot(tool.axis);
            let floor = if greater(toward_axis, 0.0) {
                0
            } else if greater(-toward_axis, 0.0) {
                1
            } else {
                return Ok(None);
            };
            Ok(Some(Cell::Blind { entry, floor }))
        }
        [first, second] => {
            let (Some(first), Some(second)) = (
                pierced_face(topo, first, tool, clearance, tol)?,
                pierced_face(topo, second, tool, clearance, tol)?,
            ) else {
                return Ok(None);
            };
            let (lower, upper) = if first.s < second.s {
                (first, second)
            } else {
                (second, first)
            };
            // Material between the discs: outward normals point away from it.
            if !greater(upper.s - lower.s, 2.0 * clearance)
                || lower.normal.dot(tool.axis) >= 0.0
                || upper.normal.dot(tool.axis) <= 0.0
            {
                return Ok(None);
            }
            Ok(Some(Cell::Through { lower, upper }))
        }
        _ => Ok(None),
    }
}

/// An outer box for a planar, cylindrical or conical face bounded only by
/// lines and circles, or `None` for any other face.
///
/// Such a patch lies in its boundary's box: a planar region lies in the
/// hull of its boundary, and on a cylinder or cone every point sits on a
/// ruling whose linear extremes are reached where the ruling leaves the
/// compact patch — on the boundary, or at the cone apex, which is added.
/// Each circle contributes its whole circle's box, a superset of any arc.
/// This is the cheap bound for the common faces; others take the general
/// conservative box.
pub(super) fn ruled_face_bound(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Option<Aabb3>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let apex = match face.surface() {
        FaceSurface::Plane { .. } | FaceSurface::Cylinder(_) => None,
        FaceSurface::Cone(cone) => Some(cone.apex()),
        FaceSurface::Nurbs(_) | FaceSurface::Sphere(_) | FaceSurface::Torus(_) => return Ok(None),
    };
    let mut bounds: Option<Aabb3> = apex.map(|apex| Aabb3 {
        min: apex,
        max: apex,
    });
    let mut include = |lo: Point3, hi: Point3| {
        let next = Aabb3 { min: lo, max: hi };
        bounds = Some(bounds.map_or(next, |current| current.union(next)));
    };
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            match edge.curve() {
                EdgeCurve::Line => {
                    for vertex in [edge.start(), edge.end()] {
                        let point = topo.vertex(vertex)?.point();
                        include(point, point);
                    }
                }
                EdgeCurve::Circle(circle) => {
                    let n = circle.normal();
                    let r = circle.radius();
                    let reach = |component: f64| r * (1.0 - component * component).max(0.0).sqrt();
                    let half = Vec3::new(reach(n.x()), reach(n.y()), reach(n.z()));
                    let c = circle.center();
                    include(
                        Point3::new(c.x() - half.x(), c.y() - half.y(), c.z() - half.z()),
                        Point3::new(c.x() + half.x(), c.y() + half.y(), c.z() + half.z()),
                    );
                }
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => return Ok(None),
            }
        }
    }
    Ok(bounds.filter(|bounds| {
        [bounds.min, bounds.max]
            .iter()
            .all(|p| p.x().is_finite() && p.y().is_finite() && p.z().is_finite())
    }))
}

/// Axis-aligned box of the tool cylinder.
fn tool_box(tool: &ToolCylinder) -> Aabb3 {
    let d = tool.axis;
    let reach = |component: f64| tool.radius * (1.0 - component * component).max(0.0).sqrt();
    let half = Vec3::new(reach(d.x()), reach(d.y()), reach(d.z()));
    let lo = tool.origin + d * tool.span.0;
    let hi = tool.origin + d * tool.span.1;
    Aabb3 {
        min: Point3::new(
            lo.x().min(hi.x()) - half.x(),
            lo.y().min(hi.y()) - half.y(),
            lo.z().min(hi.z()) - half.z(),
        ),
        max: Point3::new(
            lo.x().max(hi.x()) + half.x(),
            lo.y().max(hi.y()) + half.y(),
            lo.z().max(hi.z()) + half.z(),
        ),
    }
}

/// Largest vertex or stored edge tolerance of `solid`, or `None` when the
/// solid cannot be walked or a tolerance is not finite.
fn max_entity_tolerance(topo: &Topology, solid: SolidId) -> Option<f64> {
    let mut largest = 0.0_f64;
    for vertex in remus_topology::explorer::solid_vertices(topo, solid).ok()? {
        largest = largest.max(topo.vertex(vertex).ok()?.tolerance());
    }
    for edge in remus_topology::explorer::solid_edges(topo, solid).ok()? {
        if let Some(edge_tolerance) = topo.edge(edge).ok()?.tolerance() {
            largest = largest.max(edge_tolerance);
        }
    }
    largest.is_finite().then_some(largest)
}

/// Check one candidate face: a forward plane perpendicular to the tool axis,
/// strictly inside the tool's span, bounded by lines and circles, whose
/// section disc lies inside its region with `clearance` to every edge.
fn pierced_face(
    topo: &Topology,
    face_id: FaceId,
    tool: &ToolCylinder,
    clearance: f64,
    tol: Tolerance,
) -> Result<Option<PiercedFace>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Plane { normal, d } = *face.surface() else {
        return Ok(None);
    };
    if face.is_reversed() {
        return Ok(None);
    }
    let Some(s) = axial_plane_position(tool.origin, tool.axis, normal, d) else {
        return Ok(None);
    };
    if !(greater(s - tool.span.0, clearance) && greater(tool.span.1 - s, clearance)) {
        return Ok(None);
    }
    let center = tool.origin + tool.axis * s;
    let frame = PlaneFrame::new(normal, tool.surface.x_axis(), center);
    let mut inside_outer = false;
    for (index, wire_id) in std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .enumerate()
    {
        let wire = topo.wire(wire_id)?;
        // The loop's distance to the disc centre bounds how coarsely it may
        // be sampled: chords within half that distance of the curve cannot
        // pass over the centre, so the polygon's winding about it is the
        // loop's. Far loops therefore cost a few chords each.
        let mut nearest = f64::INFINITY;
        for oriented in wire.edges() {
            let Some(clear) = edge_clearance(topo, oriented.edge(), center, normal, d, tol)? else {
                return Ok(None);
            };
            if !greater(clear, tool.radius + clearance) {
                return Ok(None);
            }
            nearest = nearest.min(clear);
        }
        let mut polygon = Vec::new();
        for oriented in wire.edges() {
            if !sample_edge(
                topo,
                oriented.edge(),
                oriented.is_forward(),
                nearest * 0.5,
                &frame,
                &mut polygon,
            )? {
                return Ok(None);
            }
        }
        let encloses = winding_number(&polygon) != 0;
        if index == 0 {
            inside_outer = encloses;
        } else if encloses {
            return Ok(None);
        }
    }
    Ok(inside_outer.then_some(PiercedFace {
        face: face_id,
        s,
        normal,
    }))
}

/// Distance from `center` (on the face plane) to an edge of the face, or
/// `None` for a curve kind or placement the cell does not qualify.
fn edge_clearance(
    topo: &Topology,
    edge_id: EdgeId,
    center: Point3,
    normal: Vec3,
    d: f64,
    tol: Tolerance,
) -> Result<Option<f64>, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    match edge.curve() {
        EdgeCurve::Line => Ok(Some(point_segment_distance(center, start, end))),
        EdgeCurve::Circle(circle) => {
            let on_plane = (normal.dot(circle.center() - Point3::new(0.0, 0.0, 0.0)) - d).abs()
                <= tol.linear
                && circle.normal().dot(normal).abs() >= PARALLEL_COSINE;
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(None);
            };
            if !on_plane || !greater(t1, t0) {
                return Ok(None);
            }
            let offset = center - circle.center();
            let radial = offset.length();
            let ring = (radial - circle.radius()).abs();
            if t1 - t0 >= std::f64::consts::TAU - 1e-12 {
                return Ok(Some(ring));
            }
            // Closest point of the full circle, then whether the arc holds it.
            let nearest_on_arc = radial > 0.0 && {
                let towards = circle.center() + offset * (circle.radius() / radial);
                let t = circle.project(towards);
                let turns = ((t - t0) / std::f64::consts::TAU).floor();
                let t = t - turns * std::f64::consts::TAU;
                t <= t1
            };
            if radial == 0.0 || nearest_on_arc {
                return Ok(Some(ring));
            }
            Ok(Some((center - start).length().min((center - end).length())))
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => Ok(None),
    }
}

fn point_segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let length_squared = ab.dot(ab);
    let t = if length_squared > 0.0 {
        ((p - a).dot(ab) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length()
}

/// Orthonormal 2D frame on the face plane, centred on the disc centre.
struct PlaneFrame {
    origin: Point3,
    x: Vec3,
    y: Vec3,
}

impl PlaneFrame {
    fn new(normal: Vec3, reference: Vec3, origin: Point3) -> Self {
        let projected = reference - normal * reference.dot(normal);
        let x = projected.normalize().unwrap_or_else(|_| {
            let fallback = if normal.x().abs() < 0.9 {
                Vec3::new(1.0, 0.0, 0.0)
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            };
            (fallback - normal * fallback.dot(normal))
                .normalize()
                .unwrap_or(fallback)
        });
        let y = normal.cross(x);
        Self { origin, x, y }
    }

    fn project(&self, p: Point3) -> (f64, f64) {
        let offset = p - self.origin;
        (offset.dot(self.x), offset.dot(self.y))
    }
}

/// Append an edge's traversal (start included, end excluded) to `polygon`,
/// with chord error at most `sagitta`. Returns `false` when the edge cannot
/// be sampled within the bound.
fn sample_edge(
    topo: &Topology,
    edge_id: EdgeId,
    forward: bool,
    sagitta: f64,
    frame: &PlaneFrame,
    polygon: &mut Vec<(f64, f64)>,
) -> Result<bool, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    match edge.curve() {
        EdgeCurve::Line => {
            polygon.push(frame.project(if forward { start } else { end }));
            Ok(true)
        }
        EdgeCurve::Circle(circle) => {
            let Ok((t0, t1)) = edge.strict_domain() else {
                return Ok(false);
            };
            let Some(segments) = arc_segments(circle, t1 - t0, sagitta) else {
                return Ok(false);
            };
            #[allow(clippy::cast_precision_loss)]
            let step = (t1 - t0) / segments as f64;
            for k in 0..segments {
                #[allow(clippy::cast_precision_loss)]
                let t = if forward {
                    step.mul_add(k as f64, t0)
                } else {
                    (-step).mul_add(k as f64, t1)
                };
                polygon.push(frame.project(circle.evaluate(t)));
            }
            Ok(true)
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => Ok(false),
    }
}

/// Chords needed so an arc of `sweep` radians stays within `sagitta`.
fn arc_segments(circle: &Circle3D, sweep: f64, sagitta: f64) -> Option<usize> {
    let radius = circle.radius();
    if !(greater(sweep, 0.0) && greater(sagitta, 0.0)) {
        return None;
    }
    let ratio = (1.0 - sagitta / radius).max(-1.0);
    let max_step = 2.0 * ratio.acos();
    if !greater(max_step, 0.0) {
        return None;
    }
    let segments = (sweep / max_step).ceil().max(3.0);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let segments = segments as usize;
    (segments <= MAX_ARC_SEGMENTS).then_some(segments)
}

/// Winding number of a closed polygon around the frame origin.
fn winding_number(polygon: &[(f64, f64)]) -> i32 {
    let mut winding = 0;
    for (index, &(x0, y0)) in polygon.iter().enumerate() {
        let (x1, y1) = polygon[(index + 1) % polygon.len()];
        let cross = x0.mul_add(y1, -(x1 * y0));
        if y0 <= 0.0 {
            if y1 > 0.0 && cross > 0.0 {
                winding += 1;
            }
        } else if y1 <= 0.0 && cross < 0.0 {
            winding -= 1;
        }
    }
    winding
}

/// The rim the cut leaves on a pierced face: a full circle about the axis
/// point whose normal is the face's outward normal, starting at a new
/// vertex on the tool's seam. New edges carry the operation tolerance, as
/// the GFA's section edges do.
///
/// The start is found as the GFA's seam anchoring finds it — the wall
/// surface evaluated at the seam vertex's `u` and the section's `v` — and
/// the circle's reference is that point's offset from the centre. The same
/// arithmetic keeps the rim's frame and vertex bit-identical to the GFA's,
/// including the rounding of `cos(3π/2)` that surfaces near small
/// coordinates.
fn section_rim(
    tool: &ToolCylinder,
    face: &PiercedFace,
    vertex: usize,
    tol: Tolerance,
) -> Result<((Point3, f64), SpliceEdge), crate::OperationsError> {
    let center = tool.origin + tool.axis * face.s;
    let refused = || crate::OperationsError::InvalidInput {
        reason: "local cut: section rim cannot be anchored on the tool seam".into(),
    };
    let reference = Circle3D::new_with_ref(center, face.normal, tool.radius, tool.reference)
        .map_err(crate::OperationsError::Math)?;
    let (seam_u, _) = tool
        .wall
        .project_point(tool.seam_point)
        .ok_or_else(refused)?;
    let (_, section_v) = tool
        .wall
        .project_point(reference.evaluate(0.0))
        .ok_or_else(refused)?;
    let anchor = tool.wall.evaluate(seam_u, section_v).ok_or_else(refused)?;
    let radial = anchor - center;
    if !((radial.length() - tool.radius).abs() <= tol.linear
        && radial.dot(face.normal).abs() <= tol.linear)
    {
        return Err(refused());
    }
    let circle = Circle3D::new_with_ref(center, face.normal, tool.radius, radial)
        .map_err(crate::OperationsError::Math)?;
    Ok((
        (anchor, tol.linear),
        SpliceEdge {
            start: vertex,
            end: vertex,
            curve: EdgeCurve::Circle(circle),
            trim: Some((0.0, std::f64::consts::TAU)),
            tolerance: Some(tol.linear),
        },
    ))
}

/// The new wall's seam from splice vertex `start` (at the `s0` end) to
/// `end`.
fn seam(start: usize, end: usize, tol: Tolerance) -> SpliceEdge {
    SpliceEdge {
        start,
        end,
        curve: EdgeCurve::Line,
        trim: None,
        tolerance: Some(tol.linear),
    }
}

/// A pierced face's new hole: its rim, traversed clockwise seen from
/// outside (the rim's circle normal is the face's outward normal).
fn hole(rim: usize) -> SpliceWire {
    SpliceWire {
        edges: vec![(rim, false)],
        closed: true,
    }
}

/// The wall: a reversed copy of the tool's cylinder.
fn wall(tool: &ToolCylinder, outer: usize) -> SpliceFace {
    SpliceFace {
        outer,
        surface: FaceSurface::Cylinder(tool.surface.clone()),
        reversed: true,
    }
}

/// Splice for a through hole: two pierced faces gain circular inner wires,
/// joined by a reversed cylindrical wall.
fn through_hole_splice(
    topo: &Topology,
    a: SolidId,
    tool: &ToolCylinder,
    lower: &PiercedFace,
    upper: &PiercedFace,
    tol: Tolerance,
) -> Result<Splice, crate::OperationsError> {
    // Spliced entity indices: vertices 0 (lower) and 1 (upper); edges
    // 0 (lower rim), 1 (upper rim), 2 (seam); wires 0 and 1 (the holes) and
    // 2 (the wall); face 0 (the wall).
    let (lower_vertex, lower_rim) = section_rim(tool, lower, 0, tol)?;
    let (upper_vertex, upper_rim) = section_rim(tool, upper, 1, tol)?;
    // The wall's loop is the cylinder's natural (u, v) rectangle: lower rim
    // with increasing u, seam up, upper rim back, seam down. The plane faces
    // and the reversed wall both traverse each rim reversed.
    let wall_loop = vec![(0, false), (2, true), (1, false), (2, false)];
    let outer_shell = gfa_shell_order(
        topo,
        a,
        &[(lower.face, Use::Spliced(0)), (upper.face, Use::Spliced(1))],
        &[vec![
            Use::Spliced(0),
            Use::Spliced(2),
            Use::Spliced(1),
            Use::Spliced(2),
        ]],
    )?;
    Ok(Splice {
        vertices: vec![lower_vertex, upper_vertex],
        edges: vec![lower_rim, upper_rim, seam(0, 1, tol)],
        wires: vec![
            hole(0),
            hole(1),
            SpliceWire {
                edges: wall_loop,
                closed: true,
            },
        ],
        faces: vec![wall(tool, 2)],
        leading_inner: vec![(lower.face, 0), (upper.face, 1)],
        outer_shell,
    })
}

/// Splice for a blind hole: the entry face gains a circular inner wire, a
/// reversed wall runs down to the tool's buried cap, and a reversed copy of
/// that cap is the floor.
///
/// The GFA keeps the cap's own rim (curve, trim and tolerance) and vertex
/// for the floor and splits the tool's seam at the entry rim; the wall loop
/// keeps the tool's natural rectangle with the entry rim, reversed, in
/// place of the protruding cap's rim.
fn blind_hole_splice(
    topo: &Topology,
    a: SolidId,
    tool: &ToolCylinder,
    entry: &PiercedFace,
    floor: usize,
    tol: Tolerance,
) -> Result<Splice, crate::OperationsError> {
    let Some(rims) = tool.rims.as_ref() else {
        return Err(crate::OperationsError::InvalidInput {
            reason: "local cut: blind hole without natural tool rims".into(),
        });
    };
    let rim = rims[floor];
    // Spliced entity indices: vertices 0 (entry) and 1 (floor); edges
    // 0 (entry rim), 1 (floor rim), 2 (seam); wires 0 (the hole), 1 (the
    // wall) and 2 (the floor); faces 0 (the wall) and 1 (the floor).
    let (entry_vertex, entry_rim) = section_rim(tool, entry, 0, tol)?;
    let floor_vertex = topo.vertex(rim.vertex)?;
    let floor_vertex = (floor_vertex.point(), floor_vertex.tolerance());
    let floor_edge = topo.edge(rim.edge)?;
    let floor_rim = SpliceEdge {
        start: 1,
        end: 1,
        curve: floor_edge.curve().clone(),
        trim: floor_edge.trim(),
        tolerance: floor_edge.tolerance(),
    };
    let cap = topo.face(rim.cap)?;
    let floor_face = SpliceFace {
        outer: 2,
        surface: cap.surface().clone(),
        reversed: !cap.is_reversed(),
    };
    let (wall_loop, seam_edge, wall_uses) = if floor == 0 {
        (
            vec![(1, true), (2, true), (0, false), (2, false)],
            seam(1, 0, tol),
            [Use::Spliced(1), Use::Spliced(0)],
        )
    } else {
        (
            vec![(0, false), (2, true), (1, false), (2, false)],
            seam(0, 1, tol),
            [Use::Spliced(0), Use::Spliced(1)],
        )
    };
    let outer_shell = gfa_shell_order(
        topo,
        a,
        &[(entry.face, Use::Spliced(0))],
        &[
            vec![wall_uses[0], Use::Spliced(2), wall_uses[1], Use::Spliced(2)],
            vec![Use::Spliced(1)],
        ],
    )?;
    Ok(Splice {
        vertices: vec![entry_vertex, floor_vertex],
        edges: vec![entry_rim, floor_rim, seam_edge],
        wires: vec![
            hole(0),
            SpliceWire {
                edges: wall_loop,
                closed: true,
            },
            SpliceWire {
                edges: vec![(1, rim.cap_forward)],
                closed: true,
            },
        ],
        faces: vec![wall(tool, 1), floor_face],
        leading_inner: vec![(entry.face, 0)],
        outer_shell,
    })
}

/// A boundary use in the result, before its entities exist.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Use {
    /// An edge of the blank, kept by the copy.
    Blank(EdgeId),
    /// The splice edge with this index.
    Spliced(usize),
}

/// The result's outer-shell face order, as the GFA's shell assembly would
/// produce it: a breadth-first walk from the blank's first face, visiting
/// neighbours across each face's boundary uses in wire order (outer wire,
/// then inner wires, a new hole first).
///
/// `pierced` gives each pierced face's new hole rim; `spliced` gives each
/// spliced face's boundary uses, in its loop's order.
///
/// Matching that order keeps every order-sensitive reading of the result —
/// per-face listings, summed integrals — identical to the GFA's. The blank
/// is manifold by the time it reaches here (its result validates), but a
/// use shared by more than two faces is refused rather than ordered.
fn gfa_shell_order(
    topo: &Topology,
    a: SolidId,
    pierced: &[(FaceId, Use)],
    spliced: &[Vec<Use>],
) -> Result<Vec<ShellSlot>, crate::OperationsError> {
    let blank_faces = topo.shell(topo.solid(a)?.outer_shell())?.faces();
    let mut nodes: Vec<(ShellSlot, Vec<Use>)> =
        Vec::with_capacity(blank_faces.len() + spliced.len());
    for &face_id in blank_faces {
        let face = topo.face(face_id)?;
        let mut uses = Vec::new();
        for oriented in topo.wire(face.outer_wire())?.edges() {
            uses.push(Use::Blank(oriented.edge()));
        }
        uses.extend(
            pierced
                .iter()
                .filter(|&&(target, _)| target == face_id)
                .map(|&(_, rim)| rim),
        );
        for &wire in face.inner_wires() {
            for oriented in topo.wire(wire)?.edges() {
                uses.push(Use::Blank(oriented.edge()));
            }
        }
        nodes.push((ShellSlot::Source(face_id), uses));
    }
    for (index, uses) in spliced.iter().enumerate() {
        nodes.push((ShellSlot::Spliced(index), uses.clone()));
    }

    let mut faces_of: remus_math::det_hash::DetHashMap<Use, Vec<usize>> =
        remus_math::det_hash::DetHashMap::default();
    for (node, (_, uses)) in nodes.iter().enumerate() {
        for &usage in uses {
            let users = faces_of.entry(usage).or_default();
            if users.last() != Some(&node) {
                users.push(node);
            }
            if users.len() > 2 {
                return Err(crate::OperationsError::InvalidInput {
                    reason: "local cut: blank boundary use shared by more than two faces".into(),
                });
            }
        }
    }

    let mut visited = vec![false; nodes.len()];
    let mut use_count: remus_math::det_hash::DetHashMap<Use, u32> =
        remus_math::det_hash::DetHashMap::default();
    let mut order = Vec::with_capacity(nodes.len());
    let mut queue = std::collections::VecDeque::new();
    let visit = |node: usize,
                 visited: &mut Vec<bool>,
                 order: &mut Vec<usize>,
                 queue: &mut std::collections::VecDeque<usize>,
                 use_count: &mut remus_math::det_hash::DetHashMap<Use, u32>| {
        visited[node] = true;
        order.push(node);
        queue.push_back(node);
        for &usage in &nodes[node].1 {
            *use_count.entry(usage).or_default() += 1;
        }
    };
    visit(0, &mut visited, &mut order, &mut queue, &mut use_count);
    while let Some(current) = queue.pop_front() {
        for &usage in &nodes[current].1 {
            if use_count.get(&usage).copied().unwrap_or(0) >= 2 {
                continue;
            }
            let next = faces_of
                .get(&usage)
                .and_then(|users| users.iter().copied().find(|&n| n != current && !visited[n]));
            if let Some(next) = next {
                visit(next, &mut visited, &mut order, &mut queue, &mut use_count);
            }
        }
    }
    if order.len() != nodes.len() {
        return Err(crate::OperationsError::InvalidInput {
            reason: "local cut: blank boundary is not one connected shell".into(),
        });
    }
    Ok(order.into_iter().map(|node| nodes[node].0).collect())
}
