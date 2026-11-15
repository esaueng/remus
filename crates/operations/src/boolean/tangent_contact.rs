//! Line contacts an exact boolean result must not keep.
//!
//! A face of one operand can be tangent to a face of the other along a
//! straight line: a plane that holds one ruling of a cylinder, or two
//! cylinders with parallel axes that touch. Phase FF emits no section there,
//! because a grazing line splits no face, so both faces can reach the result
//! whole. When they bound the same piece of material, the boundary touches
//! itself along that line. A hole drilled tangent to a wall from the inside
//! leaves a wall of zero thickness (#1739): every edge still has two faces
//! and the Euler count balances, so no topological gate sees it, yet the
//! result is not a solid.
//!
//! [`result_touches_itself_along_a_line`] finds the tangent lines of the
//! operand pair and refuses a result in which one of them runs through the
//! interior of two faces of one piece. Two separate pieces that touch along
//! a line (a cylinder fused against a box wall) are a disjoint union and are
//! left alone, as is a tangent line that is an edge of the faces it joins
//! (a fillet meeting its support).
//!
//! # What is decided, and how
//!
//! The tangent lines are closed form, read within phase FF's own tangency
//! band ([`TANGENT_BAND`]): the pairs FF leaves unsectioned because they
//! graze. Whether a point of a line is on a face
//! is read with [`remus_check::distance::point_to_face`] and its distance to
//! the face's edges with [`remus_check::distance::point_to_edge`], which is
//! exact for lines and circles; other edge curves are measured on
//! [`EDGE_SAMPLES`] chords. The line is tested at the midpoint of every
//! interval between the places where an edge of a candidate face meets or
//! ends beside it, so a contact shorter than one such interval of a sampled
//! curve can be missed. A point counts only when it is on both faces and
//! farther than [`MARGIN_TOLERANCES`] linear tolerances from every edge of
//! each, which proves a contact at least twice that long. A measurement that
//! fails leaves the point undecided, and undecided never refuses.

use remus_math::compact_sort::CompactSort;
use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// Sine below which two unit directions are read as parallel, and cosine
/// below which as perpendicular: phase FF's own bound for a plane parallel
/// to a cylinder axis.
const PARALLEL: f64 = 1.0e-9;

/// Relative band within which a plane or cylinder is read as tangent to a
/// cylinder: phase FF's own. A plane whose distance from the axis is within
/// `1e-9` of the radius gets no section there, and a nearer one is cut along
/// two lines. A surface clear of the cylinder by more than this, however
/// little, leaves a wall, and that wall is geometry the engine resolves (a
/// bore `1e-7` inside a plate's side is cut exactly).
const TANGENT_BAND: f64 = 1.0e-9;

/// Sine bound for a radial direction measured from a point, which carries
/// the point's rounding.
const RADIAL_ALIGNMENT: f64 = 1.0e-6;

/// Distance, in linear tolerances, a contact point keeps from every edge of
/// both faces. The drilled-hole cell demands the same clearance before it
/// calls a tool clear of a face (`local_cut::CLEARANCE_TOLERANCES`).
const MARGIN_TOLERANCES: f64 = 1.0e3;

/// Chords a curved edge other than a circle is measured on.
const EDGE_SAMPLES: usize = 32;

/// A straight line along which two surfaces share a tangent plane.
#[derive(Clone, Copy)]
pub(super) struct ContactLine {
    pub(super) point: Point3,
    /// Unit direction.
    pub(super) dir: Vec3,
    /// Unit normal of the shared tangent plane.
    pub(super) normal: Vec3,
}

impl ContactLine {
    fn parameter(&self, point: Point3) -> f64 {
        (point - self.point).dot(self.dir)
    }

    fn at(&self, t: f64) -> Point3 {
        self.point + self.dir * t
    }

    fn same_line(&self, other: &Self, tol: f64) -> bool {
        let offset = other.point - self.point;
        self.dir.cross(other.dir).length() <= PARALLEL
            && (offset - self.dir * offset.dot(self.dir)).length() <= tol
    }
}

fn is_unit(v: Vec3) -> bool {
    (v.length() - 1.0).abs() <= PARALLEL
}

/// The plane of `surface` as `(normal, d)`, when it is one.
fn as_plane(surface: &FaceSurface) -> Option<(Vec3, f64)> {
    match surface {
        FaceSurface::Plane { normal, d } => Some((*normal, *d)),
        FaceSurface::Cylinder(_)
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => None,
    }
}

/// The cylinder of `surface`, when it is one.
fn as_cylinder(surface: &FaceSurface) -> Option<&CylindricalSurface> {
    match surface {
        FaceSurface::Cylinder(cylinder) => Some(cylinder),
        FaceSurface::Plane { .. }
        | FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => None,
    }
}

/// The line along which a plane holds one ruling of `cylinder`.
fn plane_cylinder_line(normal: Vec3, d: f64, cylinder: &CylindricalSurface) -> Option<ContactLine> {
    let axis = cylinder.axis();
    if !is_unit(normal) || !is_unit(axis) || normal.dot(axis).abs() > PARALLEL {
        return None;
    }
    let origin = cylinder.origin();
    let height = normal.dot(origin - Point3::new(0.0, 0.0, 0.0)) - d;
    let radius = cylinder.radius();
    ((height.abs() - radius).abs() <= TANGENT_BAND * radius).then(|| ContactLine {
        point: origin - normal * height,
        dir: axis,
        normal,
    })
}

/// The line along which two cylinders with parallel axes touch, from outside
/// or one inside the other.
fn cylinder_cylinder_line(
    one: &CylindricalSurface,
    two: &CylindricalSurface,
    tol: f64,
) -> Option<ContactLine> {
    let axis = one.axis();
    if !is_unit(axis) || !is_unit(two.axis()) || axis.cross(two.axis()).length() > PARALLEL {
        return None;
    }
    let offset = two.origin() - one.origin();
    let across = offset - axis * offset.dot(axis);
    let distance = across.length();
    if distance <= tol {
        // Coaxial: the same carrier or nested ones, never one line.
        return None;
    }
    let toward = across * (1.0 / distance);
    let (r1, r2) = (one.radius(), two.radius());
    let reach = if (distance - (r1 + r2)).abs() <= TANGENT_BAND * (r1 + r2) {
        r1
    } else if (distance - (r1 - r2).abs()).abs() <= TANGENT_BAND * r1.max(r2) {
        if r1 > r2 { r1 } else { -r1 }
    } else {
        return None;
    };
    Some(ContactLine {
        point: one.origin() + toward * reach,
        dir: axis,
        normal: toward,
    })
}

/// The line along which `first` and `second` are tangent, when they are a
/// plane holding a ruling of a cylinder or two parallel cylinders touching.
pub(super) fn tangent_line(
    first: &FaceSurface,
    second: &FaceSurface,
    tol: f64,
) -> Option<ContactLine> {
    if let (Some(one), Some(two)) = (as_cylinder(first), as_cylinder(second)) {
        return cylinder_cylinder_line(one, two, tol);
    }
    let (plane, cylinder) = as_plane(first)
        .zip(as_cylinder(second))
        .or_else(|| as_plane(second).zip(as_cylinder(first)))?;
    plane_cylinder_line(plane.0, plane.1, cylinder)
}

/// Whether `surface` contains `line` and has `line.normal` as its normal
/// along it.
fn holds_line(surface: &FaceSurface, line: &ContactLine, tol: f64) -> bool {
    match surface {
        FaceSurface::Plane { normal, d } => {
            is_unit(*normal)
                && normal.cross(line.normal).length() <= RADIAL_ALIGNMENT
                && (normal.dot(line.point - Point3::new(0.0, 0.0, 0.0)) - d).abs() <= tol
        }
        FaceSurface::Cylinder(cylinder) => {
            let axis = cylinder.axis();
            if !is_unit(axis) || axis.cross(line.dir).length() > PARALLEL {
                return false;
            }
            let offset = line.point - cylinder.origin();
            let radial = offset - axis * offset.dot(axis);
            let length = radial.length();
            (length - cylinder.radius()).abs() <= tol
                && radial.cross(line.normal).length() <= RADIAL_ALIGNMENT * length
        }
        FaceSurface::Nurbs(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => false,
    }
}

/// The distinct lines along which a face of `a` is tangent to a face of `b`.
fn operand_contact_lines(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
    tol: f64,
) -> Result<Vec<ContactLine>, crate::OperationsError> {
    let carriers = |solid: SolidId| -> Result<Vec<&FaceSurface>, crate::OperationsError> {
        let mut surfaces = Vec::new();
        for face in remus_topology::explorer::solid_faces(topo, solid)? {
            let surface = topo.face(face)?.surface();
            if matches!(
                surface,
                FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
            ) {
                surfaces.push(surface);
            }
        }
        Ok(surfaces)
    };
    let of_a = carriers(a)?;
    let of_b = carriers(b)?;
    let mut lines: Vec<ContactLine> = Vec::new();
    for first in &of_a {
        for second in &of_b {
            if let Some(line) = tangent_line(first, second, tol)
                && !lines.iter().any(|known| known.same_line(&line, tol))
            {
                lines.push(line);
            }
        }
    }
    Ok(lines)
}

/// The edges that bound `face`: every edge its wires use once. An edge used
/// twice is a seam the surface runs straight through.
fn bounding_edges(topo: &Topology, face: FaceId) -> Result<Vec<EdgeId>, crate::OperationsError> {
    let face = topo.face(face)?;
    let mut uses: Vec<(EdgeId, usize)> = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire)?.edges() {
            match uses.iter_mut().find(|(edge, _)| *edge == oriented.edge()) {
                Some((_, count)) => *count += 1,
                None => uses.push((oriented.edge(), 1)),
            }
        }
    }
    Ok(uses
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect())
}

/// `edge` as a polyline: its two ends when straight, else its ends and
/// [`EDGE_SAMPLES`] chords between them.
fn edge_polyline(topo: &Topology, edge: EdgeId) -> Result<Vec<Point3>, crate::OperationsError> {
    let edge = topo.edge(edge)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    if matches!(edge.curve(), EdgeCurve::Line) {
        return Ok(vec![start, end]);
    }
    let Ok((t0, t1)) = edge.strict_domain() else {
        return Ok(vec![start, end]);
    };
    Ok((0..=EDGE_SAMPLES)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let t = (t1 - t0).mul_add(k as f64 / EDGE_SAMPLES as f64, t0);
            edge.curve().evaluate_with_endpoints(t, start, end)
        })
        .collect())
}

fn point_segment_distance(point: Point3, a: Point3, b: Point3) -> f64 {
    let along = b - a;
    let length_sq = along.dot(along);
    let s = if length_sq > 0.0 {
        ((point - a).dot(along) / length_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (point - (a + along * s)).length()
}

/// Parameter on `line` of its closest approach to the segment `a`–`b`, when
/// the two come within `band` and are not parallel.
fn approach_parameter(line: &ContactLine, a: Point3, b: Point3, band: f64) -> Option<f64> {
    let along = b - a;
    let from = line.point - a;
    let cosine = line.dir.dot(along);
    let length_sq = along.dot(along);
    let denominator = length_sq - cosine * cosine;
    if denominator <= length_sq * PARALLEL {
        return None;
    }
    let s = ((along.dot(from) - line.dir.dot(from) * cosine) / denominator).clamp(0.0, 1.0);
    let foot = a + along * s;
    let t = line.parameter(foot);
    ((foot - line.at(t)).length() <= band).then_some(t)
}

/// Whether `point`, a point of the face's surface, is on `face` and farther
/// than `margin` from every edge that bounds it. `false` when undecided.
fn strictly_inside(
    topo: &Topology,
    face: FaceId,
    edges: &[EdgeId],
    point: Point3,
    margin: f64,
    tol: f64,
) -> bool {
    let on_face = match remus_check::distance::point_to_face(topo, point, face) {
        Ok(Some((distance, _))) => distance <= tol,
        Ok(None) | Err(_) => {
            remus_check::classify::surface_point_in_face(topo, face, point).unwrap_or(false)
        }
    };
    on_face
        && edges.iter().all(|&edge| {
            let distance = match remus_check::distance::point_to_edge(topo, point, edge) {
                Ok((distance, _)) => distance,
                Err(_) => match edge_polyline(topo, edge) {
                    Ok(polyline) => polyline
                        .windows(2)
                        .map(|pair| point_segment_distance(point, pair[0], pair[1]))
                        .fold(f64::INFINITY, f64::min),
                    Err(_) => return false,
                },
            };
            distance > margin
        })
}

/// A face of the result that holds a contact line.
struct Holder {
    face: FaceId,
    /// Index of the face's outer-shell component; `None` on a cavity shell.
    piece: Option<usize>,
    edges: Vec<EdgeId>,
}

/// Whether two faces bound the same piece of material: not two outer
/// components, which are separate bodies. A cavity bounds the body around it.
fn same_piece(a: &Holder, b: &Holder) -> bool {
    a.piece.zip(b.piece).is_none_or(|(a, b)| a == b)
}

/// Whether `line` runs through the interior of two of `holders` that bound
/// the same piece of material.
fn line_inside_two_faces(
    topo: &Topology,
    holders: &[Holder],
    line: &ContactLine,
    margin: f64,
    tol: f64,
) -> Result<bool, crate::OperationsError> {
    // Where an edge ends beside the line or meets it: the membership of the
    // line in each face can only change there.
    let mut events: Vec<f64> = Vec::new();
    for holder in holders {
        for &edge in &holder.edges {
            let polyline = edge_polyline(topo, edge)?;
            events.extend(polyline.iter().map(|&point| line.parameter(point)));
            events.extend(
                polyline
                    .windows(2)
                    .filter_map(|pair| approach_parameter(line, pair[0], pair[1], margin)),
            );
        }
    }
    events.retain(|t| t.is_finite());
    events.compact_sort_by(f64::total_cmp);
    for pair in events.windows(2) {
        if pair[1] - pair[0] <= 2.0 * margin {
            continue;
        }
        let point = line.at(0.5 * (pair[0] + pair[1]));
        let mut inside: Vec<&Holder> = Vec::new();
        for holder in holders {
            if inside.iter().any(|known| same_piece(known, holder))
                && strictly_inside(topo, holder.face, &holder.edges, point, margin, tol)
            {
                return Ok(true);
            }
            if !inside.iter().any(|known| same_piece(known, holder))
                && strictly_inside(topo, holder.face, &holder.edges, point, margin, tol)
            {
                inside.push(holder);
            }
        }
    }
    Ok(false)
}

/// Whether the boundary of `result` touches itself along a line where a face
/// of `a` is tangent to a face of `b`.
///
/// # Errors
///
/// Returns an error for a failed topology lookup.
pub(super) fn result_touches_itself_along_a_line(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
    result: SolidId,
    tolerance: Tolerance,
) -> Result<bool, crate::OperationsError> {
    let tol = tolerance.linear;
    let lines = operand_contact_lines(topo, a, b, tol)?;
    if lines.is_empty() {
        return Ok(false);
    }
    let margin = MARGIN_TOLERANCES * tol;
    let components = super::assembly::face_components(topo, result);
    let faces = remus_topology::explorer::solid_faces(topo, result)?;
    for line in &lines {
        let mut holders = Vec::new();
        for &face in &faces {
            if holds_line(topo.face(face)?.surface(), line, tol) {
                holders.push(Holder {
                    face,
                    piece: components.iter().position(|piece| piece.contains(&face)),
                    edges: bounding_edges(topo, face)?,
                });
            }
        }
        if holders.len() >= 2 && line_inside_two_faces(topo, &holders, line, margin, tol)? {
            log::debug!(
                "boolean result touches itself along the tangent line through {:?} along {:?}",
                line.point,
                line.dir
            );
            return Ok(true);
        }
    }
    Ok(false)
}
