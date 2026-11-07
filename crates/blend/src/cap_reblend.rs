//! Rebuild a rounded planar cap where its old rim meets joined supports.
//!
//! The cap's rolling-ball centres lie in its inward parallel plane. Their
//! contour on the inward offsets of the retained supports determines the new
//! contact curves. This builder is explicitly approximate on walked corner
//! carriers; callers must preserve that quality and validate the assembled body.

mod restore;
pub use restore::{SupportExtension, restore_joined_support_limits_in_place};
mod unjoin;
pub use unjoin::restore_unjoined_cap_in_place;
mod touch;
pub use touch::{rebuild_touching_cap_in_place, restore_touching_support_limits_in_place};

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::body_edit::EditScope;
use crate::builder_utils::add_certified_curve_edge;
use crate::section::CircSection;
use crate::trimmer::{self, CutPiece};
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};

/// Explicit permission and work bound for the sampled cap-corner construction.
#[derive(Clone, Copy, Debug)]
pub struct CapReblendApproximation {
    /// Maximum number of stations on each contact contour.
    pub stations: usize,
}

/// A reconstructed cap with its approximation disclosed to the caller.
pub struct CapReblendOutcome {
    /// Constructed body and construction-derived face origins.
    pub result: BlendResult,
    /// Number of stations used per sampled corner.
    pub corner_stations: usize,
    /// Surviving children of split source edges, recorded by construction.
    pub edge_images: Vec<(EdgeId, EdgeId)>,
}

#[derive(Clone)]
struct Node {
    edge: EdgeId,
    parameter: f64,
    faces: [FaceId; 2],
    point: Point3,
    center: Point3,
    vertex: Option<VertexId>,
    top_vertex: Option<VertexId>,
    cross_edge: Option<EdgeId>,
}

struct Support {
    face: FaceId,
    a: usize,
    b: usize,
    sections: Vec<CircSection>,
    surface: FaceSurface,
    lower_edge: Option<EdgeId>,
    upper_edge: Option<EdgeId>,
}

fn refusal(reason: impl Into<String>) -> BlendError {
    BlendError::InvalidInput {
        reason: reason.into(),
    }
}
fn height(point: Point3, axis: Vec3) -> f64 {
    (point - Point3::new(0.0, 0.0, 0.0)).dot(axis)
}
fn live(map: &remus_math::det_hash::DetHashMap<FaceId, FaceId>, face: FaceId) -> FaceId {
    map.get(&face).copied().unwrap_or(face)
}
fn outward(surface: &FaceSurface, reversed: bool, point: Point3) -> Result<Vec3, BlendError> {
    let (u, v) = surface
        .project_point(point)
        .ok_or_else(|| refusal("cap support projection failed"))?;
    let normal = surface.normal(u, v);
    Ok(if reversed { -normal } else { normal })
}
fn edge_point(topo: &Topology, edge: EdgeId, t: f64) -> Result<Point3, BlendError> {
    let data = topo.edge(edge)?;
    Ok(data.curve().evaluate_with_endpoints(
        t,
        topo.vertex(data.start())?.point(),
        topo.vertex(data.end())?.point(),
    ))
}
fn edge_domain(topo: &Topology, edge: EdgeId) -> Result<(f64, f64), BlendError> {
    topo.edge(edge)?
        .strict_domain()
        .map_err(|error| refusal(format!("cap edge has no parameter authority: {error}")))
}

fn roots(
    topo: &Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    axis: Vec3,
    level: f64,
    radius: f64,
) -> Result<Vec<Node>, BlendError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut nodes = Vec::new();
    for edge in solid_edges(topo, solid)? {
        let adjacent = adjacency.faces_for_edge(edge);
        let [a, b] = adjacent else { continue };
        if a == b
            || [*a, *b]
                .iter()
                .any(|face| *face == cap || rims.contains(face))
        {
            continue;
        }
        let fa = topo.face(*a)?;
        let fb = topo.face(*b)?;
        let witness = if fa.surface().is_analytic() && !fa.surface().is_planar() {
            fa
        } else if fb.surface().is_analytic() && !fb.surface().is_planar() {
            fb
        } else {
            continue;
        };
        let value = |t| -> Result<f64, BlendError> {
            let point = edge_point(topo, edge, t)?;
            let normal = outward(witness.surface(), witness.is_reversed(), point)?;
            Ok(height(point - normal * radius, axis) - level)
        };
        let (t0, t1) = edge_domain(topo, edge)?;
        let mut hits = Vec::new();
        for i in 0..128 {
            let mut lo = t0 + (t1 - t0) * (f64::from(i) / 128.0);
            let mut hi = t0 + (t1 - t0) * (f64::from(i + 1) / 128.0);
            let flo = value(lo)?;
            let fhi = value(hi)?;
            if flo.abs() < 1e-10 || fhi.abs() < 1e-10 {
                return Err(refusal(
                    "cap contour meets an existing vertex or unresolved tangent",
                ));
            }
            if flo * fhi > 0.0 {
                continue;
            }
            for _ in 0..60 {
                let mid = f64::midpoint(lo, hi);
                if value(mid)? * flo > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let parameter = f64::midpoint(lo, hi);
            let point = edge_point(topo, edge, parameter)?;
            let normal = outward(witness.surface(), witness.is_reversed(), point)?;
            let center = point - normal * radius;
            if (height(center, axis) - level).abs() > 1e-7 {
                return Err(refusal("cap contour root is discontinuous"));
            }
            hits.push((point, center, parameter));
        }
        if hits.len() > 1 {
            return Err(refusal(
                "cap contour has multiple crossings on one boundary",
            ));
        }
        if let Some((point, center, parameter)) = hits.pop() {
            nodes.push(Node {
                edge,
                parameter,
                faces: [*a, *b],
                point,
                center,
                vertex: None,
                top_vertex: None,
                cross_edge: None,
            });
        }
    }
    Ok(nodes)
}

/// Remove the old cap-ring excursion from an axial support's periodic wire.
/// Its two seam uses meet at the same lower vertex, so removing the complete
/// excursion preserves the lower boundary. The new contour replaces the old
/// ring. Its excluded angular wedge supplies a chart seam outside the kept
/// support, including when the new contour lies above the old seam endpoint.
fn detach_cap_detours(
    topo: &mut Topology,
    solid: SolidId,
    rim: FaceId,
    nodes: &[Node],
    axis: Vec3,
    replacements: &mut remus_math::det_hash::DetHashMap<FaceId, FaceId>,
) -> Result<(), BlendError> {
    let adjacency = topo.build_adjacency(solid)?;
    let mut supports: BTreeMap<FaceId, Vec<Point3>> = BTreeMap::new();
    for node in nodes {
        for &face in &node.faces {
            supports.entry(face).or_default().push(node.point);
        }
    }
    for (id, contacts) in supports {
        let face = topo.face(id)?.clone();
        let FaceSurface::Cylinder(cylinder) = face.surface() else {
            continue;
        };
        if cylinder.axis().cross(axis).length() > 1e-10 || face.is_reversed() {
            continue;
        }
        if contacts.len() != 2 || !face.inner_wires().is_empty() {
            return Err(refusal(
                "periodic cap support is not one bounded outer loop",
            ));
        }
        let wire = topo.wire(face.outer_wire())?.edges().to_vec();
        let mut detour = None;
        for start in 0..wire.len() {
            let seam = wire[start];
            let data = topo.edge(seam.edge())?;
            if !matches!(data.curve(), EdgeCurve::Line) {
                continue;
            }
            let span = topo.vertex(data.end())?.point() - topo.vertex(data.start())?.point();
            if span.cross(axis).length() > 1e-7 {
                continue;
            }
            for length in 3..wire.len() {
                let end = (start + length - 1) % wire.len();
                if wire[end].edge() != seam.edge() || wire[end].is_forward() == seam.is_forward() {
                    continue;
                }
                let ring_excursion = (1..length - 1).all(|offset| {
                    adjacency
                        .faces_for_edge(wire[(start + offset) % wire.len()].edge())
                        .contains(&rim)
                });
                if ring_excursion && detour.replace((start, length)).is_some() {
                    return Err(refusal(
                        "periodic cap support has ambiguous ring excursions",
                    ));
                }
            }
        }
        let Some((start, length)) = detour else {
            continue;
        };
        let kept: Vec<_> = (length..wire.len())
            .map(|offset| wire[(start + offset) % wire.len()])
            .collect();
        if kept.len() < 3 {
            return Err(refusal(
                "removing the cap excursion collapses its support boundary",
            ));
        }
        let radial = |point: Point3| {
            let delta = point - cylinder.origin();
            (delta - axis * delta.dot(axis)).normalize()
        };
        let a = radial(contacts[0])?;
        let b = radial(contacts[1])?;
        if a.dot(b) <= -1.0 + 1e-8 {
            return Err(refusal(
                "cap support does not have a unique excluded chart wedge",
            ));
        }
        let reference = (a + b).normalize()?;
        let surface = CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            cylinder.radius(),
            reference,
        )?;
        let wire_id = topo.add_wire(Wire::new(kept, true)?);
        let replacement = Face::new(wire_id, vec![], FaceSurface::Cylinder(surface));
        replacements.insert(id, topo.add_face(replacement));
    }
    Ok(())
}

/// The centre field of a sampled, constant-radius circular-section band.
/// Its two degree-one contact rows and weighted quadratic apex retain each
/// section's centre. Reading this continuous field avoids differentiating
/// across the sampled band's C0 station knots.
struct CanalNormal<'a> {
    surface: &'a remus_math::nurbs::surface::NurbsSurface,
    sign: f64,
    radius: f64,
}
impl<'a> CanalNormal<'a> {
    fn centre(&self, v: f64) -> Result<Point3, BlendError> {
        let surface = self.surface;
        let n = surface.control_points()[0].len();
        let knots = surface.knots_v();
        let index = knots
            .partition_point(|&knot| knot <= v)
            .saturating_sub(2)
            .min(n - 2);
        let lo = knots[index + 1];
        let hi = knots[index + 2];
        if hi <= lo {
            return Err(refusal("cap canal station has no parameter extent"));
        }
        let fraction = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
        let p = |row: usize| {
            surface.control_points()[row][index]
                + (surface.control_points()[row][index + 1] - surface.control_points()[row][index])
                    * fraction
        };
        let w0 = surface.weights()[1][index];
        let w1 = surface.weights()[1][index + 1];
        let weight = w0 + (w1 - w0) * fraction;
        let a = surface.control_points()[1][index];
        let b = surface.control_points()[1][index + 1];
        let apex = a + (b - a) * (fraction * w1 / weight);
        let first = p(0);
        let last = p(2);
        let middle = first + (last - first) * 0.5;
        let square = weight * weight;
        Ok(middle + (middle - apex) * (square / (1.0 - square)))
    }
    fn new(face: &'a remus_topology::face::Face) -> Result<Self, BlendError> {
        let FaceSurface::Nurbs(surface) = face.surface() else {
            return Err(refusal("expected circular-section corner"));
        };
        if surface.degree_u() != 2
            || surface.degree_v() != 1
            || surface.control_points().len() != 3
            || surface.domain_u() != (0.0, 1.0)
            || surface.domain_v() != (0.0, 1.0)
        {
            return Err(refusal("cap corner is outside the circular-section family"));
        }
        if surface.knots_u().len() != 6
            || surface.knots_u()[..3]
                .iter()
                .any(|k| k.to_bits() != 0.0_f64.to_bits())
            || surface.knots_u()[3..]
                .iter()
                .any(|k| k.to_bits() != 1.0_f64.to_bits())
        {
            return Err(refusal("cap corner has no clamped circular-section domain"));
        }
        let n = surface.control_points()[0].len();
        if n < 2
            || surface.control_points().iter().any(|row| row.len() != n)
            || surface.weights().len() != 3
            || surface.weights().iter().any(|row| row.len() != n)
        {
            return Err(refusal("cap corner has inconsistent station rows"));
        }
        let mut expected = None;
        for index in 0..n {
            let weight = surface.weights()[1][index];
            if !(0.1..0.999).contains(&weight)
                || surface.weights()[0][index].to_bits() != 1.0_f64.to_bits()
                || surface.weights()[2][index].to_bits() != 1.0_f64.to_bits()
            {
                return Err(refusal(
                    "cap corner has an unresolved or degenerate circular section",
                ));
            }
            let first = surface.control_points()[0][index];
            let last = surface.control_points()[2][index];
            let apex = surface.control_points()[1][index];
            let middle = first + (last - first) * 0.5;
            let square = weight * weight;
            let centre = middle + (middle - apex) * (square / (1.0 - square));
            let radius = (first - centre).length();
            let source = expected.get_or_insert(radius);
            if !radius.is_finite()
                || radius <= 0.0
                || (radius - (last - centre).length()).abs() > radius * 1e-8 + 1e-9
                || (radius - *source).abs() > radius * 1e-8 + 1e-9
            {
                return Err(refusal(
                    "cap corner does not retain one circular-section radius",
                ));
            }
        }
        let mut field = Self {
            surface,
            sign: 1.0,
            radius: expected.ok_or_else(|| refusal("cap corner has no circular sections"))?,
        };
        let point = surface.evaluate(0.5, 0.5);
        let radial = (point - field.centre(0.5)?).normalize()?;
        let mut outward = surface.normal(0.5, 0.5)?;
        if face.is_reversed() {
            outward = -outward;
        }
        let alignment = outward.dot(radial);
        if alignment.abs() < 0.9 {
            return Err(refusal(
                "cap corner normal does not follow its rolling sections",
            ));
        }
        field.sign = alignment.signum();
        Ok(field)
    }
    fn normal(&self, u: f64, v: f64) -> Result<Vec3, BlendError> {
        Ok((self.surface.evaluate(u, v) - self.centre(v)?).normalize()? * self.sign)
    }
}

/// Radius of a qualified sampled circular-section carrier.
///
/// This identifies the construction family and its station radius. It does
/// not certify an exact constant-radius surface between sampled stations.
///
/// # Errors
/// Refuses carriers outside the bounded circular-section representation.
pub fn sampled_section_radius(face: &Face) -> Result<f64, BlendError> {
    Ok(CanalNormal::new(face)?.radius)
}

/// Outward rolling-section normal of a qualified sampled carrier at a point.
///
/// This is the construction's radial normal field, not a claim that the
/// piecewise interpolated surface has an exact continuous derivative there.
///
/// # Errors
/// Refuses an unqualified carrier or failed point projection.
pub fn sampled_section_normal(face: &Face, point: Point3) -> Result<Vec3, BlendError> {
    let field = CanalNormal::new(face)?;
    let (u, v) = face
        .surface()
        .project_point(point)
        .ok_or_else(|| refusal("sampled carrier contact projection failed"))?;
    field.normal(u, v)
}

/// One stored rolling station of a qualified sampled circular-section
/// carrier: the circular section the construction interpolated through.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampledSection {
    /// The station's parameter along the carrier's V direction.
    pub v: f64,
    /// Rolling-ball centre of the station's circular section.
    pub centre: Point3,
    /// Section end on the carrier's `U = 0` boundary.
    pub start: Point3,
    /// Section end on the carrier's `U = 1` boundary.
    pub end: Point3,
}

/// A qualified sampled circular-section carrier read back from its control
/// net: the construction family the walked and boss-corner blends store.
///
/// This names the stored stations and their common radius. It does not
/// certify the surface between stations; callers measure that themselves.
#[derive(Clone, Debug)]
pub struct SampledCanal {
    /// Common radius of every stored section.
    pub radius: f64,
    /// Whether the face's outward normal points away from the section
    /// centres (a convex round) rather than toward them (a concave one).
    pub convex: bool,
    /// Stored stations in increasing `v`.
    pub sections: Vec<SampledSection>,
}

/// Read a qualified sampled circular-section carrier's stations.
///
/// # Errors
/// Refuses carriers outside the bounded circular-section representation.
pub fn sampled_canal(face: &Face) -> Result<SampledCanal, BlendError> {
    let field = CanalNormal::new(face)?;
    let surface = field.surface;
    let knots = surface.knots_v();
    let rows = surface.control_points();
    let count = rows[0].len();
    if knots.len() != count + 2 {
        return Err(refusal(
            "sampled carrier is not linear between its stations",
        ));
    }
    let mut sections = Vec::with_capacity(count);
    for index in 0..count {
        let v = knots[index + 1];
        sections.push(SampledSection {
            v,
            centre: field.centre(v)?,
            start: rows[0][index],
            end: rows[2][index],
        });
    }
    Ok(SampledCanal {
        radius: field.radius,
        convex: field.sign > 0.0,
        sections,
    })
}

/// Rolling-ball centre of a qualified sampled carrier's section through a
/// point, from the stored centre field at the point's projected `v`.
///
/// # Errors
/// Refuses an unqualified carrier or failed point projection.
pub fn sampled_section_centre(face: &Face, point: Point3) -> Result<Point3, BlendError> {
    let field = CanalNormal::new(face)?;
    let (_, v) = face
        .surface()
        .project_point(point)
        .ok_or_else(|| refusal("sampled carrier point projection failed"))?;
    field.centre(v)
}

#[derive(Clone, Copy)]
struct CapContour {
    axis: Vec3,
    level: f64,
    radius: f64,
    count: usize,
}

fn nurbs_sections(
    topo: &Topology,
    face: FaceId,
    a: &Node,
    b: &Node,
    contour: CapContour,
) -> Result<Vec<CircSection>, BlendError> {
    let CapContour {
        axis,
        level,
        radius,
        count,
    } = contour;
    let f = topo.face(face)?;
    let field = CanalNormal::new(f)?;
    let surface = field.surface;
    let (ua, va) = f
        .surface()
        .project_point(a.point)
        .ok_or_else(|| refusal("cap corner start projection failed"))?;
    let (ub, vb) = f
        .surface()
        .project_point(b.point)
        .ok_or_else(|| refusal("cap corner end projection failed"))?;
    let boundary = |u: f64, v: f64| {
        u.abs() < 1e-6 || (u - 1.0).abs() < 1e-6 || v.abs() < 1e-6 || (v - 1.0).abs() < 1e-6
    };
    if !boundary(ua, va) || !boundary(ub, vb) || (ua - ub).abs() < 1e-5 {
        return Err(refusal(
            "cap corner contour does not span qualified parameter boundaries",
        ));
    }
    let mut sections = Vec::with_capacity(count);
    for i in 0..count {
        let t = i as f64 / (count - 1) as f64;
        let u = (ua + (ub - ua) * t).clamp(0.0, 1.0);
        let (point, center, uv) = if i == 0 {
            (a.point, a.center, (ua, va))
        } else if i + 1 == count {
            (b.point, b.center, (ub, vb))
        } else {
            let value = |v| -> Result<(f64, Point3, Vec3), BlendError> {
                let point = surface.evaluate(u, v);
                let normal = field.normal(u, v)?;
                Ok((height(point - normal * radius, axis) - level, point, normal))
            };
            let mut lo = 0.0;
            let mut hi = 1.0;
            let flo = value(lo)?.0;
            let fhi = value(hi)?.0;
            if flo * fhi >= 0.0 {
                return Err(refusal("cap corner contour leaves its parameter domain"));
            }
            for _ in 0..60 {
                let mid = f64::midpoint(lo, hi);
                if value(mid)?.0 * flo > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let v = f64::midpoint(lo, hi);
            let (residual, point, normal) = value(v)?;
            if residual.abs() > 1e-7 {
                return Err(refusal("cap corner contour root was unresolved"));
            }
            (point, point - normal * radius, (u, v))
        };
        sections.push(CircSection {
            p1: point,
            p2: center + axis * radius,
            center,
            radius,
            uv1: uv,
            uv2: (0.0, 0.0),
            t,
        });
    }
    Ok(sections)
}

fn polyline(points: Vec<Point3>) -> Result<NurbsCurve, BlendError> {
    let n = points.len();
    if n < 2 {
        return Err(refusal("cap contact needs two stations"));
    }
    let mut knots = vec![0.0, 0.0];
    for i in 1..n - 1 {
        knots.push(i as f64 / (n - 1) as f64);
    }
    knots.extend([1.0, 1.0]);
    Ok(NurbsCurve::new(1, knots, points, vec![1.0; n])?)
}
fn arc_between(wire: &[OrientedEdge], from: usize, to: usize) -> Vec<OrientedEdge> {
    let mut arc = Vec::new();
    let mut pos = from;
    while pos != to {
        arc.push(wire[pos]);
        pos = (pos + 1) % wire.len();
    }
    arc
}
fn arc_field(
    topo: &Topology,
    face: FaceId,
    arc: &[OrientedEdge],
    axis: Vec3,
    level: f64,
    radius: f64,
) -> Result<(f64, f64), BlendError> {
    let f = topo.face(face)?;
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for edge in arc {
        let (a, b) = edge_domain(topo, edge.edge())?;
        let point = edge_point(topo, edge.edge(), f64::midpoint(a, b))?;
        let normal = outward(f.surface(), f.is_reversed(), point)?;
        let value = height(point - normal * radius, axis) - level;
        low = low.min(value);
        high = high.max(value);
    }
    Ok((low, high))
}
fn angular_advance(topo: &Topology, face: FaceId, arc: &[OrientedEdge]) -> Result<f64, BlendError> {
    let surface = topo.face(face)?.surface();
    let mut previous: Option<f64> = None;
    let mut advance = 0.0;
    for oe in arc {
        let (a, b) = edge_domain(topo, oe.edge())?;
        for i in 0..=8 {
            let fraction = f64::from(i) / 8.0;
            let t = if oe.is_forward() {
                a + (b - a) * fraction
            } else {
                b + (a - b) * fraction
            };
            let point = edge_point(topo, oe.edge(), t)?;
            let (u, _) = surface
                .project_point(point)
                .ok_or_else(|| refusal("cap angular boundary projection failed"))?;
            if let Some(last) = previous {
                advance += (u - last + PI).rem_euclid(TAU) - PI;
            }
            previous = Some(u);
        }
    }
    Ok(advance)
}

fn analytic_sections(
    topo: &Topology,
    face: FaceId,
    a: &Node,
    b: &Node,
    advance: f64,
    contour: CapContour,
) -> Result<(Vec<CircSection>, FaceSurface), BlendError> {
    let CapContour {
        axis,
        level,
        radius,
        count,
    } = contour;
    let f = topo.face(face)?;
    let (u0, _) = f
        .surface()
        .project_point(a.point)
        .ok_or_else(|| refusal("cap analytic start projection failed"))?;
    let (base_center, base_radius, contact_v, major, reference) = match f.surface() {
        FaceSurface::Cylinder(cylinder) if cylinder.axis().dot(axis) > 1.0 - 1e-10 => {
            let sign = if f.is_reversed() { -1.0 } else { 1.0 };
            let center = cylinder.origin() + axis * (level - height(cylinder.origin(), axis));
            (
                center,
                cylinder.radius(),
                0.0,
                cylinder.radius() - radius * sign,
                cylinder.x_axis(),
            )
        }
        FaceSurface::Torus(torus) if torus.z_axis().dot(axis) > 1.0 - 1e-10 && f.is_reversed() => {
            let offset_radius = torus.minor_radius() + radius;
            let dz = level - height(torus.center(), axis);
            if dz >= 0.0 || dz <= -offset_radius {
                return Err(refusal(
                    "cap does not cross the lower inner shoulder quadrant",
                ));
            }
            let radial = -(offset_radius * offset_radius - dz * dz).sqrt();
            let v = dz.atan2(radial).rem_euclid(TAU);
            let center = torus.center() + axis * (level - height(torus.center(), axis));
            (
                center,
                torus.major_radius() + torus.minor_radius() * v.cos(),
                v,
                torus.major_radius() + radial,
                torus.x_axis(),
            )
        }
        FaceSurface::Plane { .. }
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
        | FaceSurface::Nurbs(_) => {
            return Err(refusal("cap analytic support is outside the axial family"));
        }
    };
    if major <= 0.0 {
        return Err(refusal("cap rolling circle collapses"));
    }
    let cap_torus =
        ToroidalSurface::with_axis_and_ref_dir(base_center, major, radius, axis, reference)?;
    let contact_circle = Circle3D::new_with_ref(base_center, axis, base_radius, reference)?;
    let mut sections = Vec::new();
    for i in 0..count {
        let t = i as f64 / (count - 1) as f64;
        let u = u0 + advance * t;
        let radial = cap_torus.x_axis() * u.cos() + cap_torus.y_axis() * u.sin();
        let center = base_center + radial * major;
        let point = match f.surface() {
            FaceSurface::Cylinder(_) => contact_circle.evaluate(u),
            FaceSurface::Torus(torus) => torus.evaluate(u, contact_v),
            FaceSurface::Plane { .. }
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Nurbs(_) => return Err(refusal("cap analytic support changed")),
        };
        sections.push(CircSection {
            p1: point,
            p2: center + axis * radius,
            center,
            radius,
            uv1: (u, contact_v),
            uv2: (0.0, 0.0),
            t,
        });
    }
    for (index, node) in [(0, a), (count - 1, b)] {
        sections[index].p1 = node.point;
        sections[index].center = node.center;
        sections[index].p2 = node.center + axis * radius;
    }
    Ok((sections, FaceSurface::Torus(cap_torus)))
}

/// Rebuild the selected planar cap against its joined lower supports.
///
/// Requires explicit approximation permission. The caller supplies a transient
/// candidate whose moving support carriers are already at their new positions;
/// its old rim may overlap those carriers. Unrelated input faces are preserved
/// through copy-on-write rewrites. The caller must validate the resulting solid
/// and disclose the sampled corner construction before publishing it.
///
/// # Errors
/// Refuses unsupported carriers, ambiguous roots, an open support chain, or a
/// trim that cannot be assembled using shared construction edges.
#[allow(clippy::too_many_lines)]
pub fn reblend_planar_cap(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rim: FaceId,
    radius: f64,
    approximation: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    reblend_planar_cap_group(topo, solid, cap, &[rim], radius, approximation)
}

/// [`reblend_planar_cap`] replacing a complete group of existing cap bands.
///
/// Every old band must have the declared section radius and belong to the
/// source. This permits rebuilding an already joined cap on later edits.
///
/// # Errors
/// Returns the same atomic refusals as the single-rim builder.
pub fn reblend_planar_cap_group(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    approximation: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = reblend_impl(topo, solid, cap, rims, radius, approximation);
    if result.is_ok() {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

fn reblend_impl(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    approximation: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    if !radius.is_finite() || radius <= 0.0 || !(16..=2048).contains(&approximation.stations) {
        return Err(refusal("invalid cap radius or station budget"));
    }
    let cap_data = topo.face(cap)?;
    let axis = cap_data
        .effective_plane_normal()
        .ok_or_else(|| refusal("cap must be planar"))?;
    let FaceSurface::Plane { normal, d } = cap_data.surface() else {
        return Err(refusal("cap must be planar"));
    };
    let cap_level = *d * normal.dot(axis);
    let level = cap_level - radius;
    if !cap_data.inner_wires().is_empty() {
        return Err(refusal("cap holes are outside this reconstruction family"));
    }
    let original_faces = solid_faces(topo, solid)?;
    if !original_faces.contains(&cap)
        || rims.is_empty()
        || rims.len() > 12
        || rims
            .iter()
            .any(|rim| !original_faces.contains(rim) || *rim == cap)
    {
        return Err(refusal("cap and rim must belong to the source"));
    }
    let adjacency = topo.build_adjacency(solid)?;
    let declared: BTreeSet<_> = rims.iter().copied().collect();
    let adjacent: BTreeSet<_> = remus_topology::explorer::face_edges(topo, cap)?
        .into_iter()
        .flat_map(|edge| adjacency.faces_for_edge(edge).iter().copied())
        .filter(|&face| face != cap)
        .collect();
    if declared.len() != rims.len() || declared != adjacent {
        return Err(refusal(
            "cap reconstruction requires its complete distinct band group",
        ));
    }
    for &rim in rims {
        let face = topo.face(rim)?;
        let section_radius = match face.surface() {
            FaceSurface::Torus(torus) if torus.z_axis().cross(axis).length() < 1e-10 => {
                torus.minor_radius()
            }
            FaceSurface::Nurbs(_) => sampled_section_radius(face)?,
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => {
                return Err(refusal(
                    "old cap band is outside the axial circular-section family",
                ));
            }
        };
        if (section_radius - radius).abs() > 1e-7 {
            return Err(refusal("old cap band does not match its declared radius"));
        }
    }
    let inner_shells = topo.solid(solid)?.inner_shells().to_vec();
    if !inner_shells.is_empty() {
        return Err(refusal("cap reconstruction does not handle cavity shells"));
    }
    let mut nodes = roots(topo, solid, cap, rims, axis, level, radius)?;
    if nodes.len() < 4 || nodes.len() > 12 {
        return Err(refusal(format!(
            "cap contour needs a bounded support cycle, found {} nodes",
            nodes.len()
        )));
    }
    let mut incident: BTreeMap<FaceId, Vec<usize>> = BTreeMap::new();
    for (index, node) in nodes.iter().enumerate() {
        for &face in &node.faces {
            incident.entry(face).or_default().push(index);
        }
    }
    if incident.values().any(|nodes| nodes.len() != 2) {
        return Err(refusal("cap supports do not form a simple cycle"));
    }
    let mut scope = EditScope::new(topo, original_faces.iter().copied())?;
    let mut replacements = remus_math::det_hash::DetHashMap::default();
    if let [rim] = rims {
        detach_cap_detours(topo, solid, *rim, &nodes, axis, &mut replacements)?;
    }
    let mut split_images = Vec::new();
    for node in &mut nodes {
        let face = live(&replacements, node.faces[0]);
        let mut edit = scope.edit(&mut replacements);
        let split = trimmer::split_body_boundary_at_parameter(
            topo,
            &mut edit,
            face,
            node.edge,
            node.parameter,
            node.point,
            1e-7,
        )?;
        if let Some((source, children)) = split.edge_images {
            if source != node.edge {
                return Err(refusal("cap split did not use its qualified source edge"));
            }
            split_images.extend(children.map(|child| (source, child)));
        }
        let vertex = split.vertex;
        node.vertex = Some(vertex);
        node.top_vertex = Some(topo.add_vertex(Vertex::new(node.center + axis * radius, 1e-7)));
        let pa = node.point;
        let pb = node.center + axis * radius;
        let ca = (pa - node.center).normalize()?;
        let cross_axis = ca.cross(axis).normalize()?;
        let circle = Circle3D::new_with_ref(node.center, cross_axis, radius, ca)?;
        let angle = circle.project(pb).rem_euclid(TAU);
        if angle <= 0.0 || angle >= PI {
            return Err(refusal("cap end section is not a minor circular arc"));
        }
        node.cross_edge = Some(add_certified_curve_edge(
            topo,
            vertex,
            node.top_vertex
                .ok_or_else(|| refusal("cap top vertex missing"))?,
            EdgeCurve::Circle(circle),
            (0.0, angle),
        )?);
    }
    log::debug!("cap rebuild: {} shared contour nodes split", nodes.len());
    let mut supports = Vec::new();
    for (face, pair) in incident {
        let wire = topo
            .wire(topo.face(live(&replacements, face))?.outer_wire())?
            .edges()
            .to_vec();
        let position = |node: usize| -> Result<usize, BlendError> {
            let vertex = nodes[node]
                .vertex
                .ok_or_else(|| refusal("cap root vertex missing"))?;
            wire.iter()
                .position(|oe| {
                    topo.edge(oe.edge())
                        .is_ok_and(|edge| oe.oriented_start(edge) == vertex)
                })
                .ok_or_else(|| refusal("cap root is absent from its face wire"))
        };
        let (mut a, mut b) = (pair[0], pair[1]);
        let (pa, pb) = (position(a)?, position(b)?);
        let arc = arc_between(&wire, pa, pb);
        let reverse = arc_between(&wire, pb, pa);
        let af = arc_field(topo, face, &arc, axis, level, radius)?;
        let bf = arc_field(topo, face, &reverse, axis, level, radius)?;
        let kept = if af.1 < -1e-7 && bf.0 >= -1e-7 && bf.1 > 1e-7 {
            arc
        } else if bf.1 < -1e-7 && af.0 >= -1e-7 && af.1 > 1e-7 {
            std::mem::swap(&mut a, &mut b);
            reverse
        } else {
            return Err(refusal(format!(
                "cap contour cannot separate support {}: {af:?} {bf:?}",
                face.index()
            )));
        };
        let contour = CapContour {
            axis,
            level,
            radius,
            count: approximation.stations,
        };
        let (sections, surface) = match topo.face(face)?.surface() {
            FaceSurface::Nurbs(_) => {
                let sections = nurbs_sections(topo, face, &nodes[a], &nodes[b], contour)?;
                let surface = FaceSurface::Nurbs(
                    crate::walker::approximate_blend_surface_linear_v(&sections)?,
                );
                (sections, surface)
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => analytic_sections(
                topo,
                live(&replacements, face),
                &nodes[a],
                &nodes[b],
                angular_advance(topo, face, &kept)?,
                contour,
            )?,
        };
        log::debug!(
            "cap rebuild: support {} contour constructed from node {} to {}",
            face.index(),
            a,
            b
        );
        supports.push(Support {
            face,
            a,
            b,
            sections,
            surface,
            lower_edge: None,
            upper_edge: None,
        });
    }
    for support in &mut supports {
        let a = &nodes[support.a];
        let b = &nodes[support.b];
        let lower = polyline(support.sections.iter().map(|section| section.p1).collect())?;
        let upper = polyline(support.sections.iter().map(|section| section.p2).collect())?;
        let av = a
            .vertex
            .ok_or_else(|| refusal("cap lower vertex missing"))?;
        let bv = b
            .vertex
            .ok_or_else(|| refusal("cap lower vertex missing"))?;
        let at = a
            .top_vertex
            .ok_or_else(|| refusal("cap upper vertex missing"))?;
        let bt = b
            .top_vertex
            .ok_or_else(|| refusal("cap upper vertex missing"))?;
        support.lower_edge = Some(add_certified_curve_edge(
            topo,
            av,
            bv,
            EdgeCurve::NurbsCurve(lower),
            (0.0, 1.0),
        )?);
        support.upper_edge = Some(add_certified_curve_edge(
            topo,
            at,
            bt,
            EdgeCurve::NurbsCurve(upper),
            (0.0, 1.0),
        )?);
        let current = live(&replacements, support.face);
        let wire = topo
            .wire(topo.face(current)?.outer_wire())?
            .edges()
            .to_vec();
        let position = |vertex| {
            wire.iter()
                .position(|oe| {
                    topo.edge(oe.edge())
                        .is_ok_and(|edge| oe.oriented_start(edge) == vertex)
                })
                .ok_or_else(|| refusal("cap cut vertex disappeared"))
        };
        let (pa, pb) = (position(av)?, position(bv)?);
        let consumed = arc_between(&wire, pb, pa);
        let avoid: Vec<_> = consumed.iter().map(OrientedEdge::edge).collect();
        let trimmed = trimmer::cut_face(
            topo,
            current,
            &wire,
            &[CutPiece {
                from: pa,
                to: pb,
                edges: vec![OrientedEdge::new(
                    support
                        .lower_edge
                        .ok_or_else(|| refusal("cap contact edge missing"))?,
                    true,
                )],
            }],
            &avoid,
        )?;
        replacements.insert(support.face, trimmed);
    }
    log::debug!("cap rebuild: all support trims assembled");
    let mut created = Vec::new();
    let mut top_edges = Vec::new();
    let mut next = 0;
    let mut visited = vec![false; supports.len()];
    for _ in 0..supports.len() {
        let index = supports
            .iter()
            .enumerate()
            .position(|(index, support)| {
                !visited[index] && (support.a == next || support.b == next)
            })
            .ok_or_else(|| refusal("cap support cycle does not close"))?;
        visited[index] = true;
        let support = &supports[index];
        let forward = support.a == next;
        top_edges.push(OrientedEdge::new(
            support
                .upper_edge
                .ok_or_else(|| refusal("cap upper edge missing"))?,
            forward,
        ));
        next = if forward { support.b } else { support.a };
    }
    if next != 0 {
        return Err(refusal("cap contour is open"));
    }
    let mut polygon = Vec::new();
    for oe in &top_edges {
        let (a, b) = edge_domain(topo, oe.edge())?;
        for i in 0..16 {
            let f = f64::from(i) / 16.0;
            let t = if oe.is_forward() {
                a + (b - a) * f
            } else {
                b + (a - b) * f
            };
            polygon.push(edge_point(topo, oe.edge(), t)?);
        }
    }
    let mut area_normal = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..polygon.len() {
        area_normal +=
            (polygon[i] - polygon[0]).cross(polygon[(i + 1) % polygon.len()] - polygon[0]);
    }
    if area_normal.dot(axis) < 0.0 {
        top_edges = top_edges
            .into_iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
    }
    let top_wire = topo.add_wire(Wire::new(top_edges, true)?);
    let top = topo.add_face(Face::new(
        top_wire,
        vec![],
        FaceSurface::Plane {
            normal: axis,
            d: cap_level,
        },
    ));
    replacements.insert(cap, top);
    for support in &supports {
        let a = &nodes[support.a];
        let b = &nodes[support.b];
        let mut wire_edges = vec![
            OrientedEdge::new(
                support
                    .lower_edge
                    .ok_or_else(|| refusal("cap lower edge missing"))?,
                true,
            ),
            OrientedEdge::new(
                b.cross_edge.ok_or_else(|| refusal("cap end arc missing"))?,
                true,
            ),
            OrientedEdge::new(
                support
                    .upper_edge
                    .ok_or_else(|| refusal("cap upper edge missing"))?,
                false,
            ),
            OrientedEdge::new(
                a.cross_edge
                    .ok_or_else(|| refusal("cap start arc missing"))?,
                false,
            ),
        ];
        let mid = &support.sections[support.sections.len() / 2];
        let desired = ((mid.p1 - mid.center).normalize()? + axis).normalize()?;
        let probe = mid.center + desired * radius;
        let (u, v) = support
            .surface
            .project_point(probe)
            .ok_or_else(|| refusal("new cap band projection failed"))?;
        let reversed = support.surface.normal(u, v).dot(desired) < 0.0;
        if reversed != topo.face(support.face)?.is_reversed() {
            wire_edges = wire_edges
                .iter()
                .rev()
                .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
                .collect();
        }
        let wire = topo.add_wire(Wire::new(wire_edges, true)?);
        let mut face = Face::new(wire, vec![], support.surface.clone());
        face.set_reversed(reversed);
        let mut parents = vec![cap, support.face];
        parents.extend_from_slice(rims);
        created.push((topo.add_face(face), parents));
    }
    let faces = original_faces
        .iter()
        .filter(|face| !rims.contains(face))
        .map(|&face| live(&replacements, face))
        .chain(created.iter().map(|(face, _)| *face))
        .collect();
    let shell = topo.add_shell(Shell::new(faces)?);
    let result = topo.add_solid(Solid::new(shell, inner_shells));
    let result_edges = solid_edges(topo, result)?;
    Ok(CapReblendOutcome {
        corner_stations: approximation.stations,
        edge_images: split_images
            .into_iter()
            .filter(|(_, child)| result_edges.contains(child))
            .collect(),
        result: BlendResult {
            solid: result,
            succeeded: vec![],
            failed: vec![],
            is_partial: false,
            engine: BlendEngine::Walking,
            face_origins: Some(BlendFaceOrigins {
                survived: original_faces
                    .iter()
                    .filter(|face| !rims.contains(face))
                    .map(|&face| (face, live(&replacements, face)))
                    .collect(),
                deleted: rims.to_vec(),
                created,
                created_unattributed: vec![],
            }),
        },
    })
}
