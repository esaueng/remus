//! Shell-interference certificates for an offset (P-Class 5.7b, first slice).
//!
//! An offset moves every shell of a solid along its own outward normal.
//! Three things can then go wrong without any single face noticing:
//!
//! - a shell passes through itself along a direction in which it is thin
//!   (a capped cylinder shrunk past half its height, a slab-shaped cavity
//!   closed by an outward offset);
//! - two cavities grow into each other;
//! - a cavity grows through the outer boundary.
//!
//! Nothing here removes or repairs such a result. Each function is a
//! certificate that either proves the configuration sound, proves it folded,
//! or reports that it could not decide, so the caller can refuse instead of
//! returning a closed, plausible body with the wrong material.
//!
//! # The slab argument
//!
//! Along any direction `n` a shell lies between the planes `n.x = lo` and
//! `n.x = hi`. A point at depth `|d|` inside the region the shell bounds is at
//! least `|d|` from both planes, so nothing is left once `2|d| >= hi - lo`.
//! The mitred offset the engine builds never keeps more than that region, so
//! the argument certifies an empty result for any direction and any shape.
//! It only needs `[lo, hi]` to *contain* the shell's true extent, which the
//! support intervals below guarantee for planes, cylinders and cones bounded
//! by straight and circular edges. Everything else is left unjudged.
//!
//! # The carrier argument
//!
//! Let `K` be the intersection of the inward regions of an outer shell's
//! carriers, each of them convex (a half-space, the inside of a cylinder). A
//! straight segment that leaves the body crosses some face from its inward
//! to its outward side and, the carrier region being convex, never returns,
//! so `K` lies inside the body whatever the body's shape. When the outer
//! shell is itself inside `K` the two are equal, and a cavity is inside the
//! body exactly when a set of points whose hull contains it is inside `K`.

use remus_math::aabb::Aabb3;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;

use crate::cavity;
use crate::error::OffsetError;

/// Directions examined by [`shell_slab_thickness`] before it stops looking.
///
/// Any subset of directions is sound (it can only miss a collapse, never
/// invent one), so the cap bounds the work on a many-faceted shell.
const MAX_SLAB_DIRECTIONS: usize = 64;

/// Carrier/face pairs examined by the carrier argument before it reports the
/// separation as unproven.
const MAX_CARRIER_WORK: usize = 4_000_000;

/// Which boundary regions an offset would carry through each other.
///
/// [`code`](Self::code) is the stable machine-readable name of each kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShellInterferenceKind {
    /// An outward offset reaches a cavity's own half-thickness: opposite
    /// cavity walls meet or pass through each other.
    CavityCollapse,
    /// An inward offset closes the wall between two cavities.
    CavityCavity,
    /// An inward offset closes the wall between a cavity and the outer
    /// boundary.
    CavityOuter,
    /// The offset result could not be proven free of shell interference: the
    /// outer shell is outside the class the carrier argument covers.
    SeparationUnproven,
}

impl ShellInterferenceKind {
    /// Stable lowercase snake-case code of this kind.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::CavityCollapse => "cavity_collapse",
            Self::CavityCavity => "cavity_cavity",
            Self::CavityOuter => "cavity_outer",
            Self::SeparationUnproven => "separation_unproven",
        }
    }
}

/// A shell interference an offset would create, naming the shells involved.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShellInterference {
    /// What would interfere.
    pub kind: ShellInterferenceKind,
    /// Arena index of the cavity shell the finding names.
    pub shell: usize,
    /// Arena index of the other shell: the second cavity, or the outer shell.
    pub other_shell: Option<usize>,
    /// Model-unit room the body has: the cavity's certified thickness, or
    /// the clearance between the two shells. `None` when it was not measured.
    pub available: Option<f64>,
    /// Model-unit room the offset needs for the shells to stay apart.
    pub required: f64,
}

impl std::fmt::Display for ShellInterference {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let shell = self.shell;
        match self.kind {
            ShellInterferenceKind::CavityCollapse => {
                write!(f, "cavity shell {shell} would pass through itself")?;
            }
            ShellInterferenceKind::CavityCavity => write!(
                f,
                "cavity shells {shell} and {} would meet",
                self.other_shell.unwrap_or(shell)
            )?,
            ShellInterferenceKind::CavityOuter => {
                write!(f, "cavity shell {shell} would reach the outer boundary")?;
            }
            ShellInterferenceKind::SeparationUnproven => write!(
                f,
                "cavity shell {shell} cannot be proven to stay inside the outer boundary"
            )?,
        }
        match self.available {
            Some(available) => write!(
                f,
                " (needs more than {:.6e}, has {available:.6e})",
                self.required
            ),
            None => write!(f, " (needs more than {:.6e})", self.required),
        }
    }
}

fn vec(p: Point3) -> Vec3 {
    Vec3::new(p.x(), p.y(), p.z())
}

fn unit(v: Vec3) -> Option<Vec3> {
    let length = v.length();
    (length.is_finite() && length > 0.0).then(|| v * (1.0 / length))
}

/// `sin` of the angle between two unit vectors, accurate when they are
/// nearly parallel (where `sqrt(1 - dot^2)` loses every digit).
fn sine_between(a: Vec3, b: Vec3) -> f64 {
    a.cross(b).length().min(1.0)
}

fn face_edges(topo: &Topology, face_id: FaceId) -> Result<Vec<EdgeId>, OffsetError> {
    let face = topo.face(face_id)?;
    let mut edges = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        edges.extend(
            topo.wire(wire)?
                .edges()
                .iter()
                .map(remus_topology::OrientedEdge::edge),
        );
    }
    Ok(edges)
}

fn widen(interval: &mut Option<(f64, f64)>, lo: f64, hi: f64) {
    *interval = Some(match *interval {
        None => (lo, hi),
        Some((a, b)) => (a.min(lo), b.max(hi)),
    });
}

/// An interval containing `n . x` for every point `x` of an edge, or `None`
/// for a curve type this module does not bound.
fn edge_support(
    topo: &Topology,
    edge_id: EdgeId,
    n: Vec3,
) -> Result<Option<(f64, f64)>, OffsetError> {
    let edge = topo.edge(edge_id)?;
    Ok(match edge.curve() {
        EdgeCurve::Line => {
            let a = n.dot(vec(topo.vertex(edge.start())?.point()));
            let b = n.dot(vec(topo.vertex(edge.end())?.point()));
            Some((a.min(b), a.max(b)))
        }
        // An arc is part of its circle, whose extent along `n` is the
        // centre's plus or minus the radius foreshortened by the tilt.
        EdgeCurve::Circle(circle) => unit(circle.normal()).map(|axis| {
            let centre = n.dot(vec(circle.center()));
            let reach = circle.radius() * sine_between(n, axis);
            (centre - reach, centre + reach)
        }),
        EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::NurbsCurve(_) => None,
    })
}

/// Axial range and largest axis distance reached by a face's boundary,
/// measured against the axis `origin + t * axis`.
fn boundary_about_axis(
    topo: &Topology,
    face_id: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<Option<(f64, f64, f64)>, OffsetError> {
    let mut range: Option<(f64, f64)> = None;
    let mut reach: f64 = 0.0;
    for edge_id in face_edges(topo, face_id)? {
        let edge = topo.edge(edge_id)?;
        match edge.curve() {
            // Both the axial coordinate (linear) and the axis distance
            // (convex) attain their extremes over a segment at its ends.
            EdgeCurve::Line => {
                for vertex in [edge.start(), edge.end()] {
                    let offset = topo.vertex(vertex)?.point() - origin;
                    let t = offset.dot(axis);
                    widen(&mut range, t, t);
                    reach = reach.max((offset - axis * t).length());
                }
            }
            EdgeCurve::Circle(circle) => {
                let Some(circle_axis) = unit(circle.normal()) else {
                    return Ok(None);
                };
                let offset = circle.center() - origin;
                let t = offset.dot(axis);
                let spread = circle.radius() * sine_between(axis, circle_axis);
                widen(&mut range, t - spread, t + spread);
                reach = reach.max((offset - axis * t).length() + circle.radius());
            }
            EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_) => return Ok(None),
        }
    }
    Ok(range.map(|(lo, hi)| (lo, hi, reach)))
}

/// An interval containing `n . x` for every point `x` of a trimmed face, or
/// `None` when the face is outside the bounded class (a plane, cylinder or
/// cone trimmed by straight and circular edges).
///
/// A planar patch stays inside the hull of its boundary. A cylinder or cone
/// patch is ruled: every point lies on a ruling segment that ends on the
/// boundary, so its axial coordinate stays inside the boundary's axial range
/// and its distance from the axis never exceeds the boundary's largest.
pub(crate) fn face_support(
    topo: &Topology,
    face_id: FaceId,
    n: Vec3,
) -> Result<Option<(f64, f64)>, OffsetError> {
    let ruled = |origin: Point3, axis: Vec3, fixed_radius: Option<f64>| {
        let Some(axis) = unit(axis) else {
            return Ok(None);
        };
        let Some((t_lo, t_hi, reach)) = boundary_about_axis(topo, face_id, origin, axis)? else {
            return Ok(None);
        };
        let along = n.dot(axis);
        let radial = fixed_radius.unwrap_or(reach) * sine_between(n, axis);
        let base = n.dot(vec(origin));
        let (a, b) = (along * t_lo, along * t_hi);
        Ok(Some((base + a.min(b) - radial, base + a.max(b) + radial)))
    };
    match topo.face(face_id)?.surface() {
        FaceSurface::Plane { .. } => {
            let mut interval = None;
            for edge_id in face_edges(topo, face_id)? {
                let Some((lo, hi)) = edge_support(topo, edge_id, n)? else {
                    return Ok(None);
                };
                widen(&mut interval, lo, hi);
            }
            Ok(interval)
        }
        FaceSurface::Cylinder(cylinder) => {
            ruled(cylinder.origin(), cylinder.axis(), Some(cylinder.radius()))
        }
        FaceSurface::Cone(cone) => ruled(cone.apex(), cone.axis(), None),
        FaceSurface::Sphere(_) | FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => Ok(None),
    }
}

/// An axis-aligned box containing a face, from its support along the three
/// world axes; `None` outside the bounded class.
pub(crate) fn face_support_extent(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Option<Aabb3>, OffsetError> {
    let mut lo = [0.0; 3];
    let mut hi = [0.0; 3];
    let axes = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    for (index, axis) in axes.into_iter().enumerate() {
        let Some((a, b)) = face_support(topo, face_id, axis)? else {
            return Ok(None);
        };
        lo[index] = a;
        hi[index] = b;
    }
    Ok(Some(Aabb3 {
        min: Point3::new(lo[0], lo[1], lo[2]),
        max: Point3::new(hi[0], hi[1], hi[2]),
    }))
}

/// The effective outward unit normal of a planar face, if it is one.
fn outward_plane(topo: &Topology, face_id: FaceId) -> Result<Option<(Vec3, f64)>, OffsetError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Plane { normal, d } = face.surface() else {
        return Ok(None);
    };
    let length = normal.length();
    if !(length.is_finite() && length > 0.0) {
        return Ok(None);
    }
    let sign = if face.is_reversed() { -1.0 } else { 1.0 };
    Ok(Some((*normal * (sign / length), sign * d / length)))
}

/// A certified upper bound on a shell's smallest width, or `None` when the
/// shell is outside the bounded class.
///
/// The width is measured along each planar face's normal, with every face
/// of the shell bounded by its support interval (see the module notes). The
/// value is never below the shell's true extent in that direction, so
/// `2|d| >= thickness` proves an offset by `|d|` toward the region the shell
/// bounds leaves nothing of it.
///
/// # Errors
///
/// Returns [`OffsetError::Topology`] if the shell references missing
/// topology.
pub fn shell_slab_thickness(topo: &Topology, shell: ShellId) -> Result<Option<f64>, OffsetError> {
    let faces = topo.shell(shell)?.faces().to_vec();
    let mut directions: Vec<Vec3> = Vec::new();
    for &face_id in &faces {
        let Some((normal, _)) = outward_plane(topo, face_id)? else {
            continue;
        };
        if directions.len() < MAX_SLAB_DIRECTIONS
            && !directions
                .iter()
                .any(|seen| sine_between(*seen, normal) <= f64::EPSILON)
        {
            directions.push(normal);
        }
    }
    let mut thickness: Option<f64> = None;
    for n in directions {
        let mut interval = None;
        for &face_id in &faces {
            let Some((lo, hi)) = face_support(topo, face_id, n)? else {
                return Ok(None);
            };
            widen(&mut interval, lo, hi);
        }
        let Some((lo, hi)) = interval else {
            return Ok(None);
        };
        let width = hi - lo;
        if !width.is_finite() {
            return Ok(None);
        }
        thickness = Some(thickness.map_or(width, |t: f64| t.min(width)));
    }
    Ok(thickness)
}

/// Predict, before construction, a shell interference the offset would
/// create on a solid whose cavities are sound as given.
///
/// - Outward (`distance > 0`): a cavity whose certified thickness
///   ([`shell_slab_thickness`], or its extent along a world axis) does not
///   exceed `2 * distance` passes through itself.
/// - Inward (`distance < 0`): each wall closes by `2 * |distance|`, so a
///   cavity nearer than that to the outer shell's extent, or to another
///   cavity's, is refused. Extents are boxes, so this side can refuse a pair
///   that would in fact have stayed apart; it never accepts one that meets.
///
/// A solid whose cavities already touch each other or the outer extent is
/// not judged here: that is a malformed input, not a consequence of the
/// distance, and the engine reports it as such.
///
/// # Errors
///
/// Returns [`OffsetError`] if the solid references missing topology or a
/// shell bounds no geometry.
pub fn predict_shell_interference(
    topo: &Topology,
    solid: SolidId,
    distance: f64,
    linear_tolerance: f64,
) -> Result<Option<ShellInterference>, OffsetError> {
    let solid_data = topo.solid(solid)?;
    let cavities = solid_data.inner_shells().to_vec();
    if cavities.is_empty() || !distance.is_finite() || distance == 0.0 {
        return Ok(None);
    }
    let outer_shell = solid_data.outer_shell();
    let scale = cavity::solid_extent_scale(topo, solid)?;
    let floor = linear_tolerance.max(scale * 1e-9);
    if cavity::check_cavity_extents(topo, solid, floor, cavity::Stage::Input).is_err() {
        return Ok(None);
    }
    let outer = cavity::shell_extent(topo, outer_shell, cavity::Stage::Input)?;
    let mut extents = Vec::with_capacity(cavities.len());
    for &shell in &cavities {
        extents.push(cavity::shell_extent(topo, shell, cavity::Stage::Input)?);
    }

    if distance > 0.0 {
        let required = 2.0 * distance;
        for (&shell, extent) in cavities.iter().zip(&extents) {
            let spans = (extent.max.x() - extent.min.x())
                .min(extent.max.y() - extent.min.y())
                .min(extent.max.z() - extent.min.z());
            let available = shell_slab_thickness(topo, shell)?.map_or(spans, |t| t.min(spans));
            if available <= required + floor {
                return Ok(Some(ShellInterference {
                    kind: ShellInterferenceKind::CavityCollapse,
                    shell: shell.index(),
                    other_shell: None,
                    available: Some(available),
                    required,
                }));
            }
        }
        return Ok(None);
    }

    let required = -2.0 * distance;
    for (&shell, extent) in cavities.iter().zip(&extents) {
        let available = (extent.min.x() - outer.min.x())
            .min(extent.min.y() - outer.min.y())
            .min(extent.min.z() - outer.min.z())
            .min(outer.max.x() - extent.max.x())
            .min(outer.max.y() - extent.max.y())
            .min(outer.max.z() - extent.max.z());
        if available <= required + floor {
            return Ok(Some(ShellInterference {
                kind: ShellInterferenceKind::CavityOuter,
                shell: shell.index(),
                other_shell: Some(outer_shell.index()),
                available: Some(available),
                required,
            }));
        }
    }
    for i in 0..cavities.len() {
        for j in (i + 1)..cavities.len() {
            let (a, b) = (extents[i], extents[j]);
            // The widest axis gap: two boxes are apart by at least this.
            let available = (b.min.x() - a.max.x())
                .max(a.min.x() - b.max.x())
                .max(b.min.y() - a.max.y())
                .max(a.min.y() - b.max.y())
                .max(b.min.z() - a.max.z())
                .max(a.min.z() - b.max.z());
            if available <= required + floor {
                return Ok(Some(ShellInterference {
                    kind: ShellInterferenceKind::CavityCavity,
                    shell: cavities[i].index(),
                    other_shell: Some(cavities[j].index()),
                    available: Some(available),
                    required,
                }));
            }
        }
    }
    Ok(None)
}

/// A cavity's slab collapse under an outward offset, for the engine's own
/// input check (the direction-independent part of
/// [`predict_shell_interference`]).
pub(crate) fn certified_cavity_collapse(
    topo: &Topology,
    solid: SolidId,
    distance: f64,
    floor: f64,
) -> Result<Option<ShellInterference>, OffsetError> {
    if distance <= 0.0 {
        return Ok(None);
    }
    let required = 2.0 * distance;
    for &shell in topo.solid(solid)?.inner_shells() {
        let Some(available) = shell_slab_thickness(topo, shell)? else {
            continue;
        };
        if available <= required + floor {
            return Ok(Some(ShellInterference {
                kind: ShellInterferenceKind::CavityCollapse,
                shell: shell.index(),
                other_shell: None,
                available: Some(available),
                required,
            }));
        }
    }
    Ok(None)
}

/// A convex inward region one outer face lies on the boundary of.
#[derive(Clone, Copy)]
enum Carrier {
    /// `normal . x <= offset`.
    HalfSpace { normal: Vec3, offset: f64 },
    /// Within `radius` of the axis `origin + t * axis`.
    Cylinder {
        origin: Point3,
        axis: Vec3,
        radius: f64,
    },
}

impl Carrier {
    /// How far inside the carrier a point is (negative: outside).
    fn margin(&self, point: Point3) -> f64 {
        match *self {
            Self::HalfSpace { normal, offset } => offset - normal.dot(vec(point)),
            Self::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let offset = point - origin;
                radius - (offset - axis * offset.dot(axis)).length()
            }
        }
    }
}

/// The outer shell's carriers when every one of them is convex inward and
/// the shell provably lies inside all of them; `None` otherwise.
fn convex_outer_carriers(
    topo: &Topology,
    outer_shell: ShellId,
    tolerance: f64,
) -> Result<Option<Vec<Carrier>>, OffsetError> {
    let faces = topo.shell(outer_shell)?.faces().to_vec();
    let mut carriers = Vec::with_capacity(faces.len());
    for &face_id in &faces {
        let face = topo.face(face_id)?;
        match face.surface() {
            FaceSurface::Plane { .. } => {
                let Some((normal, offset)) = outward_plane(topo, face_id)? else {
                    return Ok(None);
                };
                carriers.push(Carrier::HalfSpace { normal, offset });
            }
            // An un-reversed cylinder face has its material on the axis
            // side; a reversed one is a bore, whose inward region (outside
            // the cylinder) is not convex.
            FaceSurface::Cylinder(cylinder) if !face.is_reversed() => {
                let Some(axis) = unit(cylinder.axis()) else {
                    return Ok(None);
                };
                carriers.push(Carrier::Cylinder {
                    origin: cylinder.origin(),
                    axis,
                    radius: cylinder.radius(),
                });
            }
            FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => return Ok(None),
        }
    }
    if carriers.len().saturating_mul(faces.len()) > MAX_CARRIER_WORK {
        return Ok(None);
    }
    for carrier in &carriers {
        for &face_id in &faces {
            let inside = match *carrier {
                Carrier::HalfSpace { normal, offset } => face_support(topo, face_id, normal)?
                    .is_some_and(|(_, hi)| hi <= offset + tolerance),
                Carrier::Cylinder {
                    origin,
                    axis,
                    radius,
                } => match topo.face(face_id)?.surface() {
                    // A planar patch is inside a cylinder when its boundary is.
                    FaceSurface::Plane { .. } => boundary_about_axis(topo, face_id, origin, axis)?
                        .is_some_and(|(_, _, reach)| reach <= radius + tolerance),
                    // Another cylinder patch only when it is the same carrier.
                    FaceSurface::Cylinder(other) => unit(other.axis()).is_some_and(|other_axis| {
                        let offset = other.origin() - origin;
                        sine_between(axis, other_axis) * radius <= tolerance
                            && (offset - axis * offset.dot(axis)).length() <= tolerance
                            && (other.radius() - radius).abs() <= tolerance
                    }),
                    FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_)
                    | FaceSurface::Nurbs(_) => false,
                },
            };
            if !inside {
                return Ok(None);
            }
        }
    }
    Ok(Some(carriers))
}

/// Points whose convex hull contains a cavity shell: its vertices when the
/// cavity is a straight-edged polyhedron, otherwise the corners of its
/// extent.
fn cavity_hull_points(topo: &Topology, shell: ShellId) -> Result<Vec<Point3>, OffsetError> {
    let faces = topo.shell(shell)?.faces().to_vec();
    let mut points = Vec::new();
    let mut polyhedral = true;
    'faces: for &face_id in &faces {
        if !matches!(topo.face(face_id)?.surface(), FaceSurface::Plane { .. }) {
            polyhedral = false;
            break;
        }
        for edge_id in face_edges(topo, face_id)? {
            let edge = topo.edge(edge_id)?;
            if !matches!(edge.curve(), EdgeCurve::Line) {
                polyhedral = false;
                break 'faces;
            }
            points.push(topo.vertex(edge.start())?.point());
            points.push(topo.vertex(edge.end())?.point());
        }
    }
    if polyhedral && !points.is_empty() {
        return Ok(points);
    }
    let extent = cavity::shell_extent(topo, shell, cavity::Stage::Result)?;
    let (lo, hi) = (extent.min, extent.max);
    Ok(vec![
        Point3::new(lo.x(), lo.y(), lo.z()),
        Point3::new(hi.x(), lo.y(), lo.z()),
        Point3::new(lo.x(), hi.y(), lo.z()),
        Point3::new(hi.x(), hi.y(), lo.z()),
        Point3::new(lo.x(), lo.y(), hi.z()),
        Point3::new(hi.x(), lo.y(), hi.z()),
        Point3::new(lo.x(), hi.y(), hi.z()),
        Point3::new(hi.x(), hi.y(), hi.z()),
    ])
}

/// Verify, on an offset result, that every cavity lies inside the outer
/// shell with more than `clearance` to spare (the carrier argument in the
/// module notes).
///
/// Returns `None` when separation is proven. A cavity that reaches or
/// leaves a carrier is [`ShellInterferenceKind::CavityOuter`]; an outer
/// shell outside the convex-carrier class is
/// [`ShellInterferenceKind::SeparationUnproven`], because a box-extent
/// comparison alone cannot tell a cavity inside the body from one that has
/// crossed a slanted or curved wall.
///
/// # Errors
///
/// Returns [`OffsetError`] if the solid references missing topology.
pub fn verify_cavities_inside_outer(
    topo: &Topology,
    solid: SolidId,
    clearance: f64,
) -> Result<Option<ShellInterference>, OffsetError> {
    let solid_data = topo.solid(solid)?;
    let cavities = solid_data.inner_shells().to_vec();
    let Some(&first) = cavities.first() else {
        return Ok(None);
    };
    let outer_shell = solid_data.outer_shell();
    let Some(carriers) = convex_outer_carriers(topo, outer_shell, clearance)? else {
        return Ok(Some(ShellInterference {
            kind: ShellInterferenceKind::SeparationUnproven,
            shell: first.index(),
            other_shell: Some(outer_shell.index()),
            available: None,
            required: clearance,
        }));
    };
    for shell in cavities {
        let mut margin = f64::INFINITY;
        for point in cavity_hull_points(topo, shell)? {
            for carrier in &carriers {
                margin = margin.min(carrier.margin(point));
            }
        }
        // A NaN margin must not pass as separation.
        if margin.is_nan() || margin <= clearance {
            return Ok(Some(ShellInterference {
                kind: ShellInterferenceKind::CavityOuter,
                shell: shell.index(),
                other_shell: Some(outer_shell.index()),
                available: Some(margin),
                required: clearance,
            }));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;

    #[test]
    fn interference_kind_codes_are_pinned() {
        // Stable-code pins: these strings reach the WASM `details.kind`.
        assert_eq!(
            ShellInterferenceKind::CavityCollapse.code(),
            "cavity_collapse"
        );
        assert_eq!(ShellInterferenceKind::CavityCavity.code(), "cavity_cavity");
        assert_eq!(ShellInterferenceKind::CavityOuter.code(), "cavity_outer");
        assert_eq!(
            ShellInterferenceKind::SeparationUnproven.code(),
            "separation_unproven"
        );
    }

    #[test]
    fn sine_between_keeps_digits_for_nearly_parallel_directions() {
        let a = Vec3::new(0.0, 0.0, 1.0);
        let b = unit(Vec3::new(1e-9, 0.0, 1.0)).unwrap();
        assert!((sine_between(a, b) - 1e-9).abs() < 1e-18);
        assert!(sine_between(a, a) == 0.0);
        assert!((sine_between(a, Vec3::new(1.0, 0.0, 0.0)) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn a_carrier_margin_is_signed_distance_to_its_boundary() {
        let plane = Carrier::HalfSpace {
            normal: Vec3::new(0.0, 0.0, 1.0),
            offset: 2.0,
        };
        assert!((plane.margin(Point3::new(5.0, 5.0, 0.5)) - 1.5).abs() < 1e-15);
        assert!((plane.margin(Point3::new(0.0, 0.0, 3.0)) + 1.0).abs() < 1e-15);
        let cylinder = Carrier::Cylinder {
            origin: Point3::new(0.0, 0.0, 0.0),
            axis: Vec3::new(0.0, 0.0, 1.0),
            radius: 5.0,
        };
        assert!((cylinder.margin(Point3::new(3.0, 4.0, 9.0))).abs() < 1e-15);
        assert!((cylinder.margin(Point3::new(1.0, 0.0, -2.0)) - 4.0).abs() < 1e-15);
    }
}
