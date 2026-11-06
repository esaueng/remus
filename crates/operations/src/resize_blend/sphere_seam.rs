//! Rolling-ball blends on the seam where a full-turn cylindrical wall meets a
//! sphere: exact removal, and a transactional wall resize that carries the
//! blend to the new seam.
//!
//! Two band forms are recognized on such a seam:
//!
//! - **Exact torus** (a coaxial seam). Removal and rebuild stay exact; the
//!   rebuilt band is a torus again.
//! - **Freeform rolling-ball band** (an off-axis seam). A rolling ball
//!   between an off-axis cylinder and a sphere sweeps a canal surface with no
//!   analytic form, so the blend engine builds it as a NURBS face. Such a band
//!   is recognized only by measurement: an exact circular cross-section edge
//!   inside the band names the ball radius and centre, the centre must lie on
//!   both offset supports, and every sample of the band lies on the closed-form
//!   rolling-ball envelope to a measured deviation. A band that fails any of
//!   those checks is not claimed.
//!
//! Removal restores the sharp seam from both analytic carriers: a coaxial
//! seam is an exact circle, an off-axis one the closed-form intersection
//! `t(θ) = c_t ± sqrt(R² − |ρ·e(θ) − c⊥|²)` fitted by the cubic interpolant of
//! the sphere-rim resize cell (certified at every half step to the model
//! linear tolerance). Faces stay on their exact carriers.
//!
//! [`resize_cylindrical_face_reblended`] removes the band, resizes the sharp
//! wall and rolls the same radius back onto the new seam, all in one topology
//! transaction. An exact band is rebuilt exactly or the edit refuses; a
//! freeform band is rebuilt at its own (approximate) quality only when the
//! caller permits it, and the result names the approximated faces and both
//! measured envelope deviations.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntityKey;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::{
    BandDescription, SharpResult, add_certified_closed_circle_edge, distinct_faces, face_edges,
    reconstruction, replace_face_wire, shared_edges, solid_adjacency, tangent_across,
    validate_exact_result, validate_volume_progress, wire_containing_edge,
};
use crate::OperationsError;
use crate::push_pull::sphere_rim_resize::{Frame, Support, fit_rim};

/// Largest accepted envelope deviation of a freeform band, as a fraction of
/// its ball radius. A band farther from the rolling-ball envelope than this is
/// not treated as a rolling-ball blend.
const FREEFORM_RECOGNITION_FRACTION: f64 = 0.02;

/// Who may approximate the rebuilt blend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReblendPolicy {
    /// Only an exact rebuild is accepted. A freeform source band refuses with
    /// [`OperationsError::ExactOnlyUnattainable`].
    ExactOnly,
    /// A freeform source band may be rebuilt as a freeform band of the same
    /// kind. An exact source band is still rebuilt exactly or refused.
    AllowApproximate,
}

/// Quality of a rebuilt seam blend.
#[derive(Debug, Clone, PartialEq)]
pub enum ReblendQuality {
    /// Every rebuilt face is on an exact analytic carrier.
    Exact,
    /// The rebuilt band is a sampled NURBS rolling-ball surface, as the
    /// source band was.
    Approximate {
        /// Result faces that carry the approximation.
        faces: Vec<FaceId>,
        /// Measured distance of the source band from its rolling-ball
        /// envelope, in model units.
        source_deviation: f64,
        /// Measured distance of the rebuilt band from its rolling-ball
        /// envelope, in model units.
        result_deviation: f64,
        /// Largest normal angle (radians) across the source band's support
        /// contacts: how far the sampled band is from tangent.
        source_contact_angle: f64,
        /// Largest normal angle (radians) across the rebuilt band's support
        /// contacts.
        result_contact_angle: f64,
    },
}

/// Result of a cylindrical resize that carried its seam blend.
#[derive(Debug, Clone, PartialEq)]
pub struct ReblendedResize {
    /// Edited solid.
    pub solid: SolidId,
    /// Ball radius measured on the source band and rebuilt on the new seam.
    pub blend_radius: f64,
    /// Source band faces that were removed and rebuilt.
    pub source_band: Vec<FaceId>,
    /// Quality of the rebuilt band.
    pub quality: ReblendQuality,
}

/// Form of a recognized seam band.
#[derive(Debug, Clone, Copy, PartialEq)]
enum BandForm {
    Exact,
    /// `deviation`: largest distance from the rolling-ball envelope;
    /// `contact_angle`: largest normal angle across a support contact.
    Freeform {
        deviation: f64,
        contact_angle: f64,
    },
}

/// A recognized rolling-ball band between a cylindrical wall and a sphere.
#[derive(Debug)]
struct SeamBand {
    description: BandDescription,
    wall: FaceId,
    sphere: FaceId,
    form: BandForm,
}

/// Closed-form seam geometry shared by recognition and reconstruction.
#[derive(Clone, Copy)]
struct SeamFrame {
    frame: Frame,
    wall_radius: f64,
    sphere_center: Point3,
    sphere_radius: f64,
    /// `(c_x, c_y, c_t)` of the sphere centre in the wall frame.
    center: (f64, f64, f64),
    linear: f64,
}

impl SeamFrame {
    fn new(cylinder: &CylindricalSurface, sphere: &SphericalSurface) -> Option<Self> {
        let axis = cylinder.axis().normalize().ok()?;
        let frame = Frame {
            origin: cylinder.origin(),
            axis,
            x: cylinder.x_axis(),
            y: cylinder.y_axis(),
        };
        let (rho, theta, t) = frame.local(sphere.center());
        let (sin, cos) = theta.sin_cos();
        let tol = Tolerance::new();
        let mut scale = cylinder.radius().max(sphere.radius()).max(1.0);
        for value in frame.origin.0.into_iter().chain(sphere.center().0) {
            scale = scale.max(value.abs());
        }
        Some(Self {
            frame,
            wall_radius: cylinder.radius(),
            sphere_center: sphere.center(),
            sphere_radius: sphere.radius(),
            center: (rho * cos, rho * sin, t),
            linear: tol.linear.max(scale * tol.relative),
        })
    }

    fn coaxial(&self) -> bool {
        self.center.0.hypot(self.center.1) <= self.linear
    }

    /// The sphere (or an offset of it) as an axial-station support.
    fn support(&self, radius: f64, branch: f64) -> Support {
        Support::Sphere {
            c_t: self.center.2,
            c_x: self.center.0,
            c_y: self.center.1,
            radius,
            branch,
        }
    }

    /// Root branch (`±1`) of the sphere on which `points` lie, or `None` when
    /// they straddle or approach the silhouette.
    fn branch(&self, points: &[Point3]) -> Option<f64> {
        let mut branch = None;
        for &point in points {
            let offset = self.frame.local(point).2 - self.center.2;
            if offset.abs() <= 1e3 * self.linear {
                return None;
            }
            let upper = offset > 0.0;
            if branch.is_some_and(|known| known != upper) {
                return None;
            }
            branch = Some(upper);
        }
        branch.map(|upper| if upper { 1.0 } else { -1.0 })
    }
}

/// Rolling-ball envelope of one seam: the ball centre runs on the
/// intersection of the wall offset by `σ_w·r` and the sphere offset by
/// `σ_s·r`.
#[derive(Clone, Copy)]
struct Envelope {
    seam: SeamFrame,
    spine_radius: f64,
    spine: Support,
    ball: f64,
}

impl Envelope {
    fn spine_point(&self, theta: f64) -> Option<Point3> {
        let t = self.spine.station(self.spine_radius, theta)?;
        Some(self.seam.frame.point(self.spine_radius, theta, t))
    }

    /// `| dist(point, spine) − r |`, minimizing over the spine azimuth near
    /// the point's own azimuth.
    fn deviation(&self, point: Point3) -> Option<f64> {
        let theta = self.seam.frame.local(point).1;
        let distance = |angle: f64| {
            self.spine_point(angle)
                .map_or(f64::INFINITY, |center| (point - center).length())
        };
        // The ball centre's azimuth differs from its contact point's by at
        // most asin(r / ρ_spine); scan a window wider than that, then refine.
        let window = (self.ball / self.spine_radius).clamp(0.0, 1.0).asin() + 0.1;
        let steps = 48;
        let mut best = (f64::INFINITY, theta);
        for index in 0..=steps {
            let angle = theta - window + 2.0 * window * f64::from(index) / f64::from(steps);
            let value = distance(angle);
            if value < best.0 {
                best = (value, angle);
            }
        }
        let step = 2.0 * window / f64::from(steps);
        let (mut lo, mut hi) = (best.1 - step, best.1 + step);
        let ratio = 0.5 * (5.0_f64.sqrt() - 1.0);
        for _ in 0..60 {
            let a = hi - ratio * (hi - lo);
            let b = lo + ratio * (hi - lo);
            if distance(a) <= distance(b) {
                hi = b;
            } else {
                lo = a;
            }
        }
        let value = distance(0.5 * (lo + hi));
        value.is_finite().then(|| (value - self.ball).abs())
    }
}

/// Centre and radius of a band cross-section edge: an exact circle, or the
/// rational quadratic arc (three control points, end weights equal) the
/// walking engine closes a full-turn band with.
fn cross_section(curve: &EdgeCurve) -> Option<(Point3, f64)> {
    match curve {
        EdgeCurve::Circle(circle) => Some((circle.center(), circle.radius())),
        EdgeCurve::NurbsCurve(arc) => {
            let points = arc.control_points();
            let weights = arc.weights();
            if arc.degree() != 2 || points.len() != 3 || weights.len() != 3 {
                return None;
            }
            let ratio = weights[1] / weights[0];
            let arc_weights = weights[0] > 0.0
                && (weights[0] - weights[2]).abs() <= 1e-12 * weights[0]
                && (0.0..1.0).contains(&ratio);
            if !arc_weights {
                return None;
            }
            // Apex P1 sits on the bisector at r / cos θ from the centre and
            // the chord midpoint at r cos θ, with cos θ = w1 / w0.
            let midpoint = points[0] + (points[2] - points[0]) * 0.5;
            let center =
                midpoint + (midpoint - points[1]) * (ratio * ratio / (1.0 - ratio * ratio));
            let radius = (points[0] - center).length();
            // Both ends sit on the ball to the walker's own Newton gate.
            let gate = (100.0 * Tolerance::new().linear).max(1e-6 * radius);
            (((points[2] - center).length() - radius).abs() <= gate).then_some((center, radius))
        }
        EdgeCurve::Line
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => None,
    }
}

fn edge_points(topo: &Topology, edge: EdgeId, count: u32) -> Result<Vec<Point3>, OperationsError> {
    let data = topo.edge(edge)?;
    let start = topo.vertex(data.start())?.point();
    let end = topo.vertex(data.end())?.point();
    let (lo, hi) = crate::authoritative_edge_domain(data, "seam band samples")?;
    Ok((0..=count)
        .map(|index| {
            let parameter = lo + (hi - lo) * f64::from(index) / f64::from(count);
            data.curve().evaluate_with_endpoints(parameter, start, end)
        })
        .collect())
}

fn cylinder_of(
    topo: &Topology,
    face: FaceId,
) -> Result<Option<CylindricalSurface>, OperationsError> {
    Ok(match topo.face(face)?.surface() {
        FaceSurface::Cylinder(cylinder) => Some(cylinder.clone()),
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => None,
    })
}

fn sphere_of(topo: &Topology, face: FaceId) -> Result<Option<SphericalSurface>, OperationsError> {
    Ok(match topo.face(face)?.surface() {
        FaceSurface::Sphere(sphere) => Some(sphere.clone()),
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Torus(_) => None,
    })
}

/// Band-carrier class of a face: `Some(true)` for a torus, `Some(false)` for
/// a NURBS face, `None` for anything that cannot be a seam band.
fn band_carrier(topo: &Topology, face: FaceId) -> Result<Option<bool>, OperationsError> {
    Ok(match topo.face(face)?.surface() {
        FaceSurface::Torus(_) => Some(true),
        FaceSurface::Nurbs(_) => Some(false),
        FaceSurface::Plane { .. }
        | FaceSurface::Cone(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Sphere(_) => None,
    })
}

/// Recognize the rolling-ball band on `wall`'s seam with a sphere.
///
/// `Ok(None)` when the wall has no tangent torus/NURBS neighbour, or when that
/// neighbour is not a complete band between this wall and one sphere face.
#[allow(clippy::too_many_lines)]
fn recognize(
    topo: &Topology,
    solid: SolidId,
    wall: FaceId,
) -> Result<Option<SeamBand>, OperationsError> {
    let Some(cylinder) = cylinder_of(topo, wall)? else {
        return Ok(None);
    };
    let adjacency = solid_adjacency(topo, solid)?;
    let mut band = BTreeSet::new();
    let mut queue = Vec::new();
    // A torus neighbour must be exactly tangent. A NURBS neighbour is only
    // as tangent as its fit: its contact angle is measured and disclosed,
    // and the rolling-ball envelope test below decides whether it is a band.
    for edge in face_edges(topo, wall)? {
        for other in distinct_faces(adjacency.faces_for_edge(edge)) {
            if other == wall {
                continue;
            }
            let candidate = match band_carrier(topo, other)? {
                Some(true) => tangent_across(topo, edge, wall, other)?,
                Some(false) => true,
                None => false,
            };
            if candidate && band.insert(other) {
                queue.push(other);
            }
        }
    }
    if band.is_empty() {
        return Ok(None);
    }
    let mut supports = BTreeSet::new();
    while let Some(current) = queue.pop() {
        for edge in face_edges(topo, current)? {
            for other in distinct_faces(adjacency.faces_for_edge(edge)) {
                if other == current || band.contains(&other) {
                    continue;
                }
                if band_carrier(topo, current)? == Some(true)
                    && !tangent_across(topo, edge, current, other)?
                {
                    // An exact band edge that is not tangent to its
                    // neighbour is not a blend bounded by its supports.
                    return Ok(None);
                }
                if band_carrier(topo, other)?.is_some() {
                    band.insert(other);
                    queue.push(other);
                } else {
                    supports.insert(other);
                }
            }
        }
    }
    let supports: Vec<_> = supports.into_iter().collect();
    let [first, second] = supports.as_slice() else {
        return Ok(None);
    };
    let sphere_face = if *first == wall { *second } else { *first };
    if !supports.contains(&wall) {
        return Ok(None);
    }
    let Some(sphere) = sphere_of(topo, sphere_face)? else {
        return Ok(None);
    };
    let Some(seam) = SeamFrame::new(&cylinder, &sphere) else {
        return Ok(None);
    };
    let faces: Vec<FaceId> = band.into_iter().collect();
    let mut torus = 0;
    for &face in &faces {
        torus += usize::from(band_carrier(topo, face)? == Some(true));
    }
    if torus == faces.len() {
        // An exact band must also pass the analytic region recognizer: one
        // common minor radius, tangent to both supports.
        let region = super::blend_region(topo, solid, faces[0])?;
        if region.faces != faces {
            return Ok(None);
        }
        return Ok(Some(SeamBand {
            description: BandDescription {
                faces,
                supports: vec![wall.min(sphere_face), wall.max(sphere_face)],
                radius: region.radius,
            },
            wall,
            sphere: sphere_face,
            form: BandForm::Exact,
        }));
    }
    if torus != 0 {
        return Ok(None);
    }

    // Freeform band: the ball radius and centre come from an exact
    // cross-section circle on an edge interior to the band.
    let band_set: BTreeSet<_> = faces.iter().copied().collect();
    let mut witnesses = Vec::new();
    for &face in &faces {
        for edge in face_edges(topo, face)? {
            let users = distinct_faces(adjacency.faces_for_edge(edge));
            if users.iter().all(|user| band_set.contains(user))
                && let Some(witness) = cross_section(topo.edge(edge)?.curve())
            {
                witnesses.push(witness);
            }
        }
    }
    let Some(&(center, ball)) = witnesses.first() else {
        return Ok(None);
    };
    if ball <= seam.linear
        || ball.is_nan()
        || witnesses
            .iter()
            .any(|(_, r)| (r - ball).abs() > seam.linear)
    {
        return Ok(None);
    }
    let rho = seam.frame.local(center).0;
    let wall_sign: f64 =
        if (rho - (seam.wall_radius + ball)).abs() <= (rho - (seam.wall_radius - ball)).abs() {
            1.0
        } else {
            -1.0
        };
    let to_sphere = (center - seam.sphere_center).length();
    let sphere_sign: f64 = if (to_sphere - (seam.sphere_radius + ball)).abs()
        <= (to_sphere - (seam.sphere_radius - ball)).abs()
    {
        1.0
    } else {
        -1.0
    };
    let spine_radius = wall_sign.mul_add(ball, seam.wall_radius);
    let offset_sphere = sphere_sign.mul_add(ball, seam.sphere_radius);
    let centre_residual = (rho - spine_radius)
        .abs()
        .max((to_sphere - offset_sphere).abs());
    let limit = FREEFORM_RECOGNITION_FRACTION * ball;
    if centre_residual > limit || spine_radius <= seam.linear || offset_sphere <= seam.linear {
        return Ok(None);
    }

    // Contact samples: the wall contact must be on the wall, the sphere
    // contact on the sphere, both at one ball radius from the spine.
    let mut contacts = Vec::new();
    let mut sphere_contacts = Vec::new();
    for &face in &faces {
        for edge in face_edges(topo, face)? {
            let users = distinct_faces(adjacency.faces_for_edge(edge));
            if users.contains(&wall) {
                contacts.extend(edge_points(topo, edge, 64)?);
            } else if users.contains(&sphere_face) {
                let points = edge_points(topo, edge, 64)?;
                sphere_contacts.extend(points.iter().copied());
                contacts.extend(points);
            }
        }
    }
    let Some(branch) = seam.branch(&sphere_contacts) else {
        return Ok(None);
    };
    let envelope = Envelope {
        seam,
        spine_radius,
        spine: seam.support(offset_sphere, branch),
        ball,
    };
    let mut deviation = centre_residual;
    for point in contacts {
        let Some(value) = envelope.deviation(point) else {
            return Ok(None);
        };
        deviation = deviation.max(value);
    }
    for &face in &faces {
        let FaceSurface::Nurbs(surface) = topo.face(face)?.surface() else {
            return Ok(None);
        };
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        for i in 0..=16_u32 {
            for j in 0..=16_u32 {
                let u = u0 + (u1 - u0) * f64::from(i) / 16.0;
                let v = v0 + (v1 - v0) * f64::from(j) / 16.0;
                let Some(value) = envelope.deviation(surface.evaluate(u, v)) else {
                    return Ok(None);
                };
                deviation = deviation.max(value);
            }
        }
    }
    if deviation > limit {
        return Ok(None);
    }
    let mut contact_angle = 0.0_f64;
    for &face in &faces {
        for edge in face_edges(topo, face)? {
            for support in [wall, sphere_face] {
                if distinct_faces(adjacency.faces_for_edge(edge)).contains(&support) {
                    let angle = crate::query::edge_normal_angle(topo, edge, face, support)?
                        .ok_or_else(|| reconstruction("seam band contact has no normal angle"))?;
                    contact_angle = contact_angle.max(angle);
                }
            }
        }
    }
    Ok(Some(SeamBand {
        description: BandDescription {
            faces,
            supports: vec![wall.min(sphere_face), wall.max(sphere_face)],
            radius: ball,
        },
        wall,
        sphere: sphere_face,
        form: BandForm::Freeform {
            deviation,
            contact_angle,
        },
    }))
}

/// Azimuthal travel sense (`±1`) of an edge about the wall axis.
fn travel_sense(topo: &Topology, frame: Frame, edge: EdgeId) -> Result<f64, OperationsError> {
    let points = edge_points(topo, edge, 16)?;
    let mut total = 0.0;
    for pair in points.windows(2) {
        let a = frame.local(pair[0]).1;
        let b = frame.local(pair[1]).1;
        let step = (b - a + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI;
        total += step;
    }
    if total.abs() < 0.5 * TAU {
        return Err(reconstruction(
            "seam band contact does not run once around the wall axis",
        ));
    }
    Ok(total.signum())
}

/// Restore the sharp seam of a closed band between a full-turn cylindrical
/// wall and a sphere.
///
/// Each support carries one closed contact edge. The wall contact is
/// replaced by the exact seam (a circle when coaxial, the certified
/// closed-form fit otherwise) starting at the wall's line-seam vertex, the
/// line seam is re-limited to that vertex, and the sphere's contact loop is
/// closed by the same seam edge. The band faces and every edge interior to
/// the band are consumed.
#[allow(clippy::too_many_lines)]
pub(super) fn heal_cylinder_sphere_band(
    topo: &mut Topology,
    solid: SolidId,
    band: &BandDescription,
) -> Result<SharpResult, OperationsError> {
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Err(reconstruction(
            "cylinder/sphere reconstruction does not edit cavity shells",
        ));
    }
    let [first, second] = band.supports.as_slice() else {
        return Err(reconstruction(
            "cylinder/sphere reconstruction requires exactly two supports",
        ));
    };
    let (wall_source, sphere_source) = if cylinder_of(topo, *first)?.is_some() {
        (*first, *second)
    } else {
        (*second, *first)
    };
    let (Some(cylinder), Some(sphere)) = (
        cylinder_of(topo, wall_source)?,
        sphere_of(topo, sphere_source)?,
    ) else {
        return Err(reconstruction("support classification changed during heal"));
    };
    let seam = SeamFrame::new(&cylinder, &sphere)
        .ok_or_else(|| reconstruction("cylinder support has a degenerate axis"))?;

    let copied = crate::copy::copy_solid_with_entity_map(topo, solid)?;
    let copy = copied.solid;
    let copied_face = |source: FaceId| {
        copied
            .face_map
            .get(&source.index())
            .copied()
            .ok_or_else(|| reconstruction(format!("face {} was not copied", source.index())))
    };
    let wall = copied_face(wall_source)?;
    let sphere_face = copied_face(sphere_source)?;
    let band_faces: BTreeSet<FaceId> = band
        .faces
        .iter()
        .map(|&face| copied_face(face))
        .collect::<Result<_, _>>()?;

    let mut wall_contacts = BTreeSet::new();
    let mut sphere_contacts = BTreeSet::new();
    let mut consumed_edges = BTreeSet::new();
    for &face in &band_faces {
        for edge in shared_edges(topo, copy, wall, face)? {
            wall_contacts.insert(edge);
        }
        for edge in shared_edges(topo, copy, sphere_face, face)? {
            sphere_contacts.insert(edge);
        }
        consumed_edges.extend(face_edges(topo, face)?);
    }
    let (Some(&wall_contact), Some(&sphere_contact), 1, 1) = (
        wall_contacts.first(),
        sphere_contacts.first(),
        wall_contacts.len(),
        sphere_contacts.len(),
    ) else {
        return Err(reconstruction(format!(
            "closed seam band needs one contact on each support; found {} and {}",
            wall_contacts.len(),
            sphere_contacts.len()
        )));
    };
    if !topo.edge(wall_contact)?.is_closed() || !topo.edge(sphere_contact)?.is_closed() {
        return Err(reconstruction(
            "cylinder/sphere support contacts are not full closed loops",
        ));
    }
    let branch = seam
        .branch(&edge_points(topo, sphere_contact, 32)?)
        .ok_or_else(|| reconstruction("sphere contact straddles the sphere's silhouette"))?;
    let support = seam.support(seam.sphere_radius, branch);

    let (wall_wire_id, wall_wire) = wire_containing_edge(topo, wall, wall_contact)?;
    let (sphere_wire_id, sphere_wire) = wire_containing_edge(topo, sphere_face, sphere_contact)?;
    if sphere_wire.len() != 1 {
        return Err(reconstruction(
            "sphere contact loop contains more than the band contact edge",
        ));
    }
    let wall_vertex = topo.edge(wall_contact)?.start();
    let sphere_vertex = topo.edge(sphere_contact)?.start();

    // The wall's line seam ends on the contact vertex; it is the only edge
    // outside the band that may touch either contact vertex.
    let mut seams = Vec::new();
    for edge in remus_topology::explorer::solid_edges(topo, copy)? {
        if consumed_edges.contains(&edge) {
            continue;
        }
        let data = topo.edge(edge)?;
        let touches_wall = data.start() == wall_vertex || data.end() == wall_vertex;
        let touches_sphere = data.start() == sphere_vertex || data.end() == sphere_vertex;
        if touches_sphere || (touches_wall && !matches!(data.curve(), EdgeCurve::Line)) {
            return Err(reconstruction(
                "a seam band contact vertex is shared outside the band",
            ));
        }
        if touches_wall {
            seams.push(edge);
        }
    }
    let [old_seam] = seams.as_slice() else {
        return Err(reconstruction(format!(
            "closed cylinder support has {} seam edges on the band contact; exactly one is required",
            seams.len()
        )));
    };
    if !wall_wire
        .iter()
        .any(|oriented| oriented.edge() == *old_seam)
    {
        return Err(reconstruction("cylinder seam is not on the wall boundary"));
    }

    let theta = seam.frame.local(topo.vertex(wall_vertex)?.point()).1;
    let station = support
        .station(seam.wall_radius, theta)
        .ok_or_else(|| reconstruction("wall seam azimuth leaves the sphere silhouette"))?;
    let sharp_point = seam.frame.point(seam.wall_radius, theta, station);
    let sharp_vertex = topo.add_vertex(Vertex::new(sharp_point, Tolerance::new().linear));
    let sense = travel_sense(topo, seam.frame, wall_contact)?;
    let sphere_sense = travel_sense(topo, seam.frame, sphere_contact)?;

    // The seam in travel order (the wall contact's sense). A coaxial seam is
    // one exact circle; an off-axis seam is four quarter-turn closed-form
    // fits, the split the boolean itself makes, which the exact face
    // integrators measure (a single closed rim winding the period is not).
    let sharp_edges: Vec<EdgeId> = if seam.coaxial() {
        let radial = sharp_point - (seam.frame.origin + seam.frame.axis * station);
        let radial = radial
            .normalize()
            .map_err(|error| reconstruction(format!("invalid sharp seam direction: {error}")))?;
        let circle = Circle3D::new_with_ref(
            seam.frame.origin + seam.frame.axis * station,
            seam.frame.axis * sense,
            seam.wall_radius,
            radial,
        )
        .map_err(|error| reconstruction(format!("sharp circle construction failed: {error}")))?;
        vec![add_certified_closed_circle_edge(
            topo,
            sharp_vertex,
            circle,
        )?]
    } else {
        const QUARTERS: u32 = 4;
        let sweep = sense * TAU / f64::from(QUARTERS);
        let mut corners = vec![sharp_vertex];
        for k in 1..QUARTERS {
            let angle = sweep.mul_add(f64::from(k), theta);
            let t = support
                .station(seam.wall_radius, angle)
                .ok_or_else(|| reconstruction("sharp seam leaves the sphere silhouette"))?;
            let point = seam.frame.point(seam.wall_radius, angle, t);
            corners.push(topo.add_vertex(Vertex::new(point, Tolerance::new().linear)));
        }
        corners.push(sharp_vertex);
        let mut edges = Vec::with_capacity(corners.len() - 1);
        for (k, pair) in (0_u32..).zip(corners.windows(2)) {
            let curve = fit_rim(
                wall,
                wall_contact,
                support,
                seam.frame,
                seam.linear,
                seam.wall_radius,
                sweep.mul_add(f64::from(k), theta),
                sweep,
            )?;
            let mut edge = Edge::with_tolerance(
                pair[0],
                pair[1],
                EdgeCurve::NurbsCurve(curve),
                Some(seam.linear),
            );
            edge.set_trim(Some((0.0, 1.0)));
            edges.push(topo.add_edge(edge));
        }
        edges
    };
    // Uses of the seam in a wire that traversed the old contact `forward`.
    let seam_uses = |forward: bool| -> Vec<OrientedEdge> {
        if forward {
            sharp_edges
                .iter()
                .map(|&edge| OrientedEdge::new(edge, true))
                .collect()
        } else {
            sharp_edges
                .iter()
                .rev()
                .map(|&edge| OrientedEdge::new(edge, false))
                .collect()
        }
    };

    let seam_data = topo.edge(*old_seam)?;
    let (seam_start, seam_end) = if seam_data.start() == wall_vertex {
        (sharp_vertex, seam_data.end())
    } else {
        (seam_data.start(), sharp_vertex)
    };
    let sharp_seam = topo.add_edge(Edge::new(seam_start, seam_end, EdgeCurve::Line));
    let mut wall_edges: Vec<OrientedEdge> = Vec::with_capacity(wall_wire.len() + 3);
    for oriented in &wall_wire {
        if oriented.edge() == wall_contact {
            wall_edges.extend(seam_uses(oriented.is_forward()));
        } else if oriented.edge() == *old_seam {
            wall_edges.push(OrientedEdge::new(sharp_seam, oriented.is_forward()));
        } else {
            wall_edges.push(*oriented);
        }
    }
    let wall_new_wire = topo.add_wire(Wire::new(wall_edges, true)?);
    replace_face_wire(topo, wall, wall_wire_id, wall_new_wire)?;
    let sphere_forward = if (sphere_sense > 0.0) == (sense > 0.0) {
        sphere_wire[0].is_forward()
    } else {
        !sphere_wire[0].is_forward()
    };
    let sphere_new_wire = topo.add_wire(Wire::new(seam_uses(sphere_forward), true)?);
    replace_face_wire(topo, sphere_face, sphere_wire_id, sphere_new_wire)?;

    // Coedges that carried a stored p-curve get a fresh one on the new seam.
    let mut stored = BTreeSet::new();
    for (old, new) in sharp_edges
        .iter()
        .map(|&edge| (wall_contact, edge))
        .chain(std::iter::once((*old_seam, sharp_seam)))
    {
        for face in [wall, sphere_face] {
            for forward in [true, false] {
                if topo.pcurve_oriented(old, face, forward).is_some() {
                    stored.insert((new.index(), face.index()));
                }
            }
        }
    }
    if !stored.is_empty() {
        crate::replace_surface::register_fresh_pcurves_where(
            topo,
            &[wall, sphere_face],
            |edge, face, _| stored.contains(&(edge.index(), face.index())),
        )?;
    }

    let old_shell = topo.solid(copy)?.outer_shell();
    let kept: Vec<FaceId> = topo
        .shell(old_shell)?
        .faces()
        .iter()
        .copied()
        .filter(|face| !band_faces.contains(face))
        .collect();
    let shell = topo.add_shell(Shell::new(kept)?);
    let sharp_solid = topo.add_solid(Solid::new(shell, Vec::new()));
    let band_sources: BTreeSet<usize> = band.faces.iter().map(|face| face.index()).collect();
    let face_map = copied
        .face_map
        .iter()
        .filter(|(source, _)| !band_sources.contains(source))
        .map(|(&source, &face)| (source, face))
        .collect();

    let live_edges: BTreeSet<_> = remus_topology::explorer::solid_edges(topo, sharp_solid)?
        .into_iter()
        .collect();
    let live_vertices: BTreeSet<_> = remus_topology::explorer::solid_vertices(topo, sharp_solid)?
        .into_iter()
        .collect();
    let mut boundary_history = Vec::new();
    let edge_sources: BTreeMap<_, _> = copied.edge_map.iter().map(|(&s, &c)| (s, c)).collect();
    for (source, copied_edge) in edge_sources {
        if copied_edge == wall_contact || copied_edge == sphere_contact {
            // The contact is replaced by every edge of the sharp seam.
            for &edge in &sharp_edges {
                boundary_history
                    .push((EntityKey::edge(source), Some(EntityKey::edge(edge.index()))));
            }
            continue;
        }
        let target = if copied_edge == *old_seam {
            sharp_seam
        } else {
            copied_edge
        };
        boundary_history.push((
            EntityKey::edge(source),
            live_edges
                .contains(&target)
                .then_some(EntityKey::edge(target.index())),
        ));
    }
    let vertex_sources: BTreeMap<usize, VertexId> =
        copied.vertex_map.iter().map(|(&s, &c)| (s, c)).collect();
    for (source, copied_vertex) in vertex_sources {
        let target = if copied_vertex == wall_vertex || copied_vertex == sphere_vertex {
            sharp_vertex
        } else {
            copied_vertex
        };
        boundary_history.push((
            EntityKey::vertex(source),
            live_vertices
                .contains(&target)
                .then_some(EntityKey::vertex(target.index())),
        ));
    }
    Ok(SharpResult {
        solid: sharp_solid,
        edges: sharp_edges,
        face_map,
        boundary_history: Some(boundary_history),
    })
}

/// The resized wall: the one cylindrical face of `solid` on the source axis
/// at `radius`.
fn resized_wall(
    topo: &Topology,
    solid: SolidId,
    source: &CylindricalSurface,
    radius: f64,
) -> Result<FaceId, OperationsError> {
    let tol = Tolerance::new();
    let mut found = Vec::new();
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        if let Some(cylinder) = cylinder_of(topo, face)?
            && tol.approx_eq(cylinder.radius(), radius)
            && cylinder.axis().cross(source.axis()).length() <= tol.angular
        {
            let offset = cylinder.origin() - source.origin();
            if offset.cross(source.axis()).length() <= tol.linear.max(radius * tol.relative) {
                found.push(face);
            }
        }
    }
    let [face] = found.as_slice() else {
        return Err(reconstruction(format!(
            "resized solid has {} walls on the source axis at radius {radius}",
            found.len()
        )));
    };
    Ok(*face)
}

/// The sphere face of `solid` on `source`'s carrier that meets `wall`.
fn seam_sphere(
    topo: &Topology,
    solid: SolidId,
    wall: FaceId,
    source: &SphericalSurface,
) -> Result<(FaceId, Vec<EdgeId>), OperationsError> {
    let tol = Tolerance::new();
    let mut found = Vec::new();
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        if let Some(sphere) = sphere_of(topo, face)?
            && tol.approx_eq(sphere.radius(), source.radius())
            && (sphere.center() - source.center()).length()
                <= tol.linear.max(source.radius() * tol.relative)
        {
            let edges = shared_edges(topo, solid, wall, face)?;
            if !edges.is_empty() {
                found.push((face, edges));
            }
        }
    }
    if found.len() != 1 {
        return Err(reconstruction(format!(
            "resized wall meets {} faces of its sphere support; exactly one is required",
            found.len()
        )));
    }
    Ok(found.swap_remove(0))
}

/// Resize a full-turn cylindrical wall whose seam with a sphere carries a
/// rolling-ball blend, and rebuild that blend at its measured radius.
///
/// The band is removed to the exact sharp seam, the sharp wall is resized
/// (the coaxial cell or the off-axis sphere-rim cell), and the measured ball
/// radius is rolled back onto the new seam, all inside one topology
/// transaction. An exact (torus) band is rebuilt exactly or the edit refuses;
/// it is never silently approximated. A freeform (NURBS) band is rebuilt only
/// under [`ReblendPolicy::AllowApproximate`], and the result reports the
/// approximated faces with both measured envelope deviations. The result must
/// have the source's face/edge/vertex counts, a rebuilt band on the same
/// radius and envelope bound, and a blend volume effect consistent with the
/// source band's.
///
/// `Ok(None)` when the wall has no recognized seam band with a sphere; the
/// caller's other constructions apply unchanged.
///
/// # Errors
///
/// [`OperationsError::ExactOnlyUnattainable`] for a freeform band under
/// [`ReblendPolicy::ExactOnly`]; a typed resize-blend or move-face refusal
/// when a step has no qualified construction. Every refusal restores the
/// arena exactly.
pub fn resize_cylindrical_face_reblended(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
    policy: ReblendPolicy,
) -> Result<Option<ReblendedResize>, OperationsError> {
    let tol = Tolerance::new();
    if !new_radius.is_finite() || new_radius <= tol.linear {
        return Err(OperationsError::InvalidInput {
            reason: format!("cylinder radius must be positive, got {new_radius}"),
        });
    }
    if !remus_topology::explorer::solid_faces(topo, solid)?.contains(&face) {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "face {} is not part of solid {}",
                face.index(),
                solid.index()
            ),
        });
    }
    let _memo = super::DiscoveryMemoScope::enter();
    let Some(band) = recognize(topo, solid, face)? else {
        return Ok(None);
    };
    if matches!(band.form, BandForm::Freeform { .. }) && policy == ReblendPolicy::ExactOnly {
        return Err(OperationsError::ExactOnlyUnattainable);
    }
    remus_topology::transaction::run_transacted(topo, |topo| {
        reblend(topo, solid, &band, new_radius)
    })
    .map(Some)
}

#[allow(clippy::too_many_lines)]
fn reblend(
    topo: &mut Topology,
    solid: SolidId,
    band: &SeamBand,
    new_radius: f64,
) -> Result<ReblendedResize, OperationsError> {
    let (Some(cylinder), Some(sphere)) =
        (cylinder_of(topo, band.wall)?, sphere_of(topo, band.sphere)?)
    else {
        return Err(reconstruction("seam band supports changed during the edit"));
    };
    let radius = band.description.radius;
    let source_counts = remus_topology::explorer::solid_entity_counts(topo, solid)?;
    let input_volume = crate::measure::solid_volume(topo, solid, 0.05)?;

    let sharp = heal_cylinder_sphere_band(topo, solid, &band.description)?;
    validate_exact_result(
        topo,
        sharp.solid,
        "sharp cylinder/sphere seam reconstruction",
    )?;
    let sharp_volume = crate::measure::solid_volume(topo, sharp.solid, 0.05)?;
    validate_volume_progress(input_volume, sharp_volume, sharp_volume, radius, 0.0)?;
    let source_effect = input_volume - sharp_volume;
    if source_effect.abs() <= tol_volume(input_volume) {
        return Err(reconstruction(
            "removing the seam band did not change the volume",
        ));
    }
    let wall = sharp
        .face_map
        .get(&band.wall.index())
        .copied()
        .ok_or_else(|| reconstruction("sharp seam reconstruction lost the wall"))?;

    let resized = crate::push_pull::resize_cylindrical_face(topo, sharp.solid, wall, new_radius)?;
    let resized_volume = crate::measure::solid_volume(topo, resized, 0.05)?;
    let new_wall = resized_wall(topo, resized, &cylinder, new_radius)?;
    let (_, seam_edges) = seam_sphere(topo, resized, new_wall, &sphere)?;
    let resized_faces: BTreeSet<_> = remus_topology::explorer::solid_faces(topo, resized)?
        .into_iter()
        .collect();

    let rebuilt = crate::blend_ops::fillet_cascade(topo, resized, &seam_edges, radius)?;
    if rebuilt.is_partial {
        return Err(OperationsError::PartialResult {
            operation: "seam re-blend",
            succeeded: rebuilt.succeeded.len(),
            failed: rebuilt.failed.len(),
        });
    }
    let result = rebuilt.solid;
    validate_exact_result(topo, result, "re-blended cylinder/sphere seam")?;
    let mut approximated = Vec::new();
    for result_face in remus_topology::explorer::solid_faces(topo, result)? {
        if !resized_faces.contains(&result_face)
            && matches!(topo.face(result_face)?.surface(), FaceSurface::Nurbs(_))
        {
            approximated.push(result_face);
        }
    }
    if band.form == BandForm::Exact && !approximated.is_empty() {
        return Err(OperationsError::ExactOnlyUnattainable);
    }

    // The rebuilt band must be recognized again: same form, same radius,
    // and (for a freeform band) an envelope bound of the same order.
    let final_wall = resized_wall(topo, result, &cylinder, new_radius)?;
    let rebuilt_band = recognize(topo, result, final_wall)?.ok_or_else(|| {
        reconstruction("the rebuilt seam blend is not a recognized rolling-ball band")
    })?;
    if !Tolerance::new().approx_eq(rebuilt_band.description.radius, radius) {
        return Err(reconstruction(format!(
            "rebuilt seam blend measures radius {} mm; the source band is {radius} mm",
            rebuilt_band.description.radius
        )));
    }
    let quality = match (band.form, rebuilt_band.form) {
        (BandForm::Exact, BandForm::Exact) => ReblendQuality::Exact,
        (
            BandForm::Freeform {
                deviation: source,
                contact_angle: source_angle,
            },
            BandForm::Freeform {
                deviation: result,
                contact_angle: result_angle,
            },
        ) => {
            // The quality floor is the source band's own measured envelope
            // deviation and contact angle, with headroom for the new seam's
            // different shape.
            let floor = 4.0 * source.max(10.0 * Tolerance::new().linear);
            let angle_floor = 2.0 * source_angle.max(Tolerance::new().angular);
            if result > floor || result_angle > angle_floor {
                return Err(reconstruction(format!(
                    "rebuilt seam blend deviates {result:.3e} mm from its rolling-ball envelope \
                     with a {result_angle:.3e} rad contact angle; the source band measures \
                     {source:.3e} mm and {source_angle:.3e} rad"
                )));
            }
            ReblendQuality::Approximate {
                faces: approximated,
                source_deviation: source,
                result_deviation: result,
                source_contact_angle: source_angle,
                result_contact_angle: result_angle,
            }
        }
        // A better-than-source rebuild is accepted and reported exact.
        (BandForm::Freeform { .. }, BandForm::Exact) => ReblendQuality::Exact,
        (BandForm::Exact, BandForm::Freeform { .. }) => {
            return Err(OperationsError::ExactOnlyUnattainable);
        }
    };

    // Volume: the rebuilt blend adds or removes material on the same side as
    // the source, by an amount of the same order (the band section is the
    // same ball; only the seam length and dihedral change with the wall).
    let result_volume = crate::measure::solid_volume(topo, result, 0.05)?;
    let result_effect = result_volume - resized_volume;
    let ratio = result_effect / source_effect;
    let scale = new_radius / cylinder.radius();
    if !(ratio > 0.5 * scale.min(1.0) && ratio < 2.0 * scale.max(1.0)) {
        return Err(reconstruction(format!(
            "rebuilt seam blend changes the volume by {result_effect}; the source band changed it by {source_effect}"
        )));
    }
    let final_counts = remus_topology::explorer::solid_entity_counts(topo, result)?;
    if final_counts != source_counts {
        return Err(reconstruction(format!(
            "seam re-blend changed (faces, edges, vertices) from {source_counts:?} to {final_counts:?}"
        )));
    }
    Ok(ReblendedResize {
        solid: result,
        blend_radius: radius,
        source_band: band.description.faces.clone(),
        quality,
    })
}

fn tol_volume(volume: f64) -> f64 {
    Tolerance::new().linear.powi(3).max(volume.abs() * 1e-12)
}
