//! Exact face-face blend bands for disjoint planar support patches.

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::plane::plane_plane_intersection;
use remus_math::predicates::orient2d;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};

use crate::BlendError;
use crate::query::{GeometricSpine, inward_surface, materialize_spine};

/// A prescribed contact segment for one selected support face.
#[derive(Debug, Clone, Copy)]
pub struct FaceFaceHoldLine {
    /// Support face whose exact contact is prescribed.
    pub support: FaceId,
    /// First endpoint of the contact segment.
    pub start: Point3,
    /// Second endpoint of the contact segment.
    pub end: Point3,
}

/// Exact standalone blend band built from two face selections.
#[derive(Debug, Clone, Copy)]
pub struct FaceFaceBlendBand {
    /// Cylindrical blend face.
    pub face: FaceId,
    /// Longitudinal contact edges, in support-set order.
    pub contact_edges: [EdgeId; 2],
    /// Start of the synthetic carrier-intersection spine.
    pub spine_start: Point3,
    /// End of the synthetic carrier-intersection spine.
    pub spine_end: Point3,
    /// Prescribed constant radius.
    pub radius: f64,
}

struct PlanarSupport {
    face: FaceId,
    normal: Vec3,
    d: f64,
    points: Vec<Point3>,
    inward: FaceSurface,
}

/// Build one exact constant-radius band between two disjoint planar faces.
///
/// The qualified subset is deliberately narrow: each selection contains one
/// convex, hole-free face bounded only by straight edges; the carrier planes
/// are transversal; the faces share no edge; and both generated contact
/// segments remain inside their selected support patches. The result is a
/// standalone face. Higher layers may wrap it in a first-class sheet body.
///
/// A hold line prescribes the complete contact segment on either selected
/// support. It is verified against the exact analytic contact before topology
/// is allocated.
///
/// # Errors
///
/// Returns [`BlendError::InvalidInput`] for malformed numeric input and
/// [`BlendError::UnsupportedFaceFaceBlend`] when the request lies outside the
/// qualified exact subset.
pub fn build_face_face_blend_band(
    topo: &mut Topology,
    first_faces: &[FaceId],
    second_faces: &[FaceId],
    radius: f64,
    hold_line: Option<FaceFaceHoldLine>,
) -> Result<FaceFaceBlendBand, BlendError> {
    if !radius.is_finite() || radius <= 0.0 || radius > f64::MAX.sqrt() {
        return Err(BlendError::InvalidInput {
            reason: "face-face radius must be finite, positive, and safe for squared geometry"
                .into(),
        });
    }
    let first = one_face(first_faces)?;
    let second = one_face(second_faces)?;
    if first == second {
        return Err(unsupported("support faces must be distinct"));
    }

    let tolerance = Tolerance::new();
    let first = planar_support(topo, first, tolerance)?;
    let second = planar_support(topo, second, tolerance)?;
    if shares_edge(topo, first.face, second.face)? {
        return Err(unsupported("support faces must not share an edge"));
    }

    let Some((line_point, line_direction)) = plane_plane_intersection(
        first.normal,
        first.d,
        second.normal,
        second.d,
        tolerance.angular,
    ) else {
        return Err(unsupported("support planes must be transversal"));
    };
    let first_range = projected_range(&first.points, line_point, line_direction);
    let second_range = projected_range(&second.points, line_point, line_direction);
    let start_parameter = first_range.0.max(second_range.0);
    let end_parameter = first_range.1.min(second_range.1);
    if end_parameter - start_parameter <= tolerance.linear {
        return Err(unsupported(
            "support patches need a positive common longitudinal span",
        ));
    }
    let spine_start = line_point + line_direction * start_parameter;
    let spine_end = line_point + line_direction * end_parameter;

    let mut scratch = topo.clone();
    let geometric_spine = GeometricSpine::Line {
        start: spine_start,
        end: spine_end,
    };
    let spine_edge = materialize_spine(&mut scratch, &geometric_spine, tolerance);
    let spine = crate::spine::Spine::from_single_edge(&scratch, spine_edge)?;
    let Some(stripe_result) = crate::analytic::try_analytic_fillet(
        &first.inward,
        &second.inward,
        &spine,
        &scratch,
        radius,
        first.face,
        second.face,
    )?
    else {
        return Err(unsupported(
            "support pair has no qualified exact analytic blend",
        ));
    };
    let stripe = stripe_result.stripe;

    if !curve_inside_support(&stripe.contact1, &first, tolerance)
        || !curve_inside_support(&stripe.contact2, &second, tolerance)
    {
        return Err(unsupported(
            "the prescribed radius places a contact outside its support patch",
        ));
    }
    if let Some(hold) = hold_line {
        validate_hold_line(
            &stripe.contact1,
            &stripe.contact2,
            first.face,
            second.face,
            hold,
            tolerance,
        )?;
    }

    let blend = crate::builder_utils::create_blend_face_with_contacts(topo, &stripe, None, None)?;
    let wire = topo.wire(topo.face(blend.face)?.outer_wire())?;
    if wire.edges().len() != 4 {
        return Err(unsupported(
            "analytic blend band did not produce four boundaries",
        ));
    }
    let contact_edges = [wire.edges()[0].edge(), wire.edges()[2].edge()];

    Ok(FaceFaceBlendBand {
        face: blend.face,
        contact_edges,
        spine_start,
        spine_end,
        radius,
    })
}

fn one_face(faces: &[FaceId]) -> Result<FaceId, BlendError> {
    match faces {
        [face] => Ok(*face),
        _ => Err(unsupported(
            "each qualified face set must contain exactly one face",
        )),
    }
}

fn unsupported(reason: impl Into<String>) -> BlendError {
    BlendError::UnsupportedFaceFaceBlend {
        reason: reason.into(),
    }
}

fn planar_support(
    topo: &Topology,
    face_id: FaceId,
    tolerance: Tolerance,
) -> Result<PlanarSupport, BlendError> {
    let face = topo.face(face_id)?;
    if !face.inner_wires().is_empty() {
        return Err(unsupported("support faces must be hole-free"));
    }
    let (normal, d) = match face.surface() {
        FaceSurface::Plane { normal, d }
            if finite_vec(*normal)
                && d.is_finite()
                && (normal.length() - 1.0).abs() <= tolerance.angular =>
        {
            (*normal, *d)
        }
        FaceSurface::Plane { .. } => {
            return Err(BlendError::InvalidInput {
                reason: "face-face support plane must have a finite unit normal".into(),
            });
        }
        _ => return Err(unsupported("qualified face-face supports must be planar")),
    };
    let wire = topo.wire(face.outer_wire())?;
    if !wire.is_closed() || wire.edges().len() < 3 {
        return Err(unsupported("support outer wires must be closed polygons"));
    }

    let mut points = Vec::with_capacity(wire.edges().len());
    let mut first_vertex = None;
    let mut previous_end = None;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Err(unsupported(
                "qualified support boundaries must contain only straight edges",
            ));
        }
        let start = oriented.oriented_start(edge);
        let end = oriented.oriented_end(edge);
        if previous_end.is_some_and(|previous| previous != start) {
            return Err(unsupported("support outer wire is not vertex-continuous"));
        }
        first_vertex.get_or_insert(start);
        previous_end = Some(end);
        let point = topo.vertex(start)?.point();
        let end_point = topo.vertex(end)?.point();
        if !finite_point(point) || !finite_point(end_point) {
            return Err(BlendError::InvalidInput {
                reason: "face-face support vertices must be finite".into(),
            });
        }
        if (normal.dot(Vec3::new(point.x(), point.y(), point.z())) - d).abs() > tolerance.linear {
            return Err(unsupported(
                "support polygon vertices must lie on their carrier plane",
            ));
        }
        if (end_point - point).length() <= tolerance.linear {
            return Err(unsupported("support polygon contains a collapsed edge"));
        }
        points.push(point);
    }
    if previous_end != first_vertex {
        return Err(unsupported(
            "support outer wire is not topologically closed",
        ));
    }
    if !simple_polygon(&points, normal, tolerance.linear)
        || !convex_polygon(&points, normal, tolerance.linear)
    {
        return Err(unsupported("qualified support polygons must be convex"));
    }

    Ok(PlanarSupport {
        face: face_id,
        normal,
        d,
        points,
        inward: inward_surface(face.surface(), face.is_reversed()),
    })
}

fn shares_edge(topo: &Topology, first: FaceId, second: FaceId) -> Result<bool, BlendError> {
    let first_wire = topo.wire(topo.face(first)?.outer_wire())?;
    let second_wire = topo.wire(topo.face(second)?.outer_wire())?;
    Ok(first_wire.edges().iter().any(|first_edge| {
        second_wire
            .edges()
            .iter()
            .any(|second_edge| first_edge.edge() == second_edge.edge())
    }))
}

fn projected_range(points: &[Point3], origin: Point3, direction: Vec3) -> (f64, f64) {
    points
        .iter()
        .map(|point| (*point - origin).dot(direction))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        })
}

fn convex_polygon(points: &[Point3], normal: Vec3, tolerance: f64) -> bool {
    let mut sign = None;
    for index in 0..points.len() {
        let first = points[(index + 1) % points.len()] - points[index];
        let second = points[(index + 2) % points.len()] - points[(index + 1) % points.len()];
        let turn = first.cross(second).dot(normal);
        let scale = first.length().max(second.length()).max(1.0);
        if turn.abs() <= tolerance * scale {
            continue;
        }
        if sign.is_some_and(|sign| turn * sign < 0.0) {
            return false;
        }
        sign.get_or_insert_with(|| turn.signum());
    }
    sign.is_some()
}

fn simple_polygon(points: &[Point3], normal: Vec3, tolerance: f64) -> bool {
    let projected = points
        .iter()
        .map(|point| project_dominant_plane(*point, normal))
        .collect::<Vec<_>>();
    for first in 0..projected.len() {
        let first_next = (first + 1) % projected.len();
        for second in (first + 1)..projected.len() {
            let second_next = (second + 1) % projected.len();
            if first == second_next || first_next == second {
                continue;
            }
            if segments_intersect(
                projected[first],
                projected[first_next],
                projected[second],
                projected[second_next],
                tolerance,
            ) {
                return false;
            }
        }
    }
    true
}

fn project_dominant_plane(point: Point3, normal: Vec3) -> Point2 {
    let abs = [normal.x().abs(), normal.y().abs(), normal.z().abs()];
    if abs[0] >= abs[1] && abs[0] >= abs[2] {
        Point2::new(point.y(), point.z())
    } else if abs[1] >= abs[2] {
        Point2::new(point.x(), point.z())
    } else {
        Point2::new(point.x(), point.y())
    }
}

fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2, tolerance: f64) -> bool {
    let scale = ((b - a).length().max((d - c).length())).max(1.0);
    let epsilon = tolerance * scale;
    let orientations = [
        orient2d(a, b, c),
        orient2d(a, b, d),
        orient2d(c, d, a),
        orient2d(c, d, b),
    ];
    if orientations[0] * orientations[1] < 0.0 && orientations[2] * orientations[3] < 0.0 {
        return true;
    }
    for (orientation, point, start, end) in [
        (orientations[0], c, a, b),
        (orientations[1], d, a, b),
        (orientations[2], a, c, d),
        (orientations[3], b, c, d),
    ] {
        if orientation.abs() <= epsilon && point_on_segment(point, start, end, tolerance) {
            return true;
        }
    }
    false
}

fn point_on_segment(point: Point2, start: Point2, end: Point2, tolerance: f64) -> bool {
    point.x() >= start.x().min(end.x()) - tolerance
        && point.x() <= start.x().max(end.x()) + tolerance
        && point.y() >= start.y().min(end.y()) - tolerance
        && point.y() <= start.y().max(end.y()) + tolerance
}

fn curve_inside_support(curve: &NurbsCurve, support: &PlanarSupport, tolerance: Tolerance) -> bool {
    let (start, end) = curve.domain();
    [start, f64::midpoint(start, end), end]
        .into_iter()
        .map(|parameter| curve.evaluate(parameter))
        .all(|point| point_inside_convex_support(point, support, tolerance.linear))
}

fn point_inside_convex_support(point: Point3, support: &PlanarSupport, tolerance: f64) -> bool {
    let plane_distance = support
        .normal
        .dot(Vec3::new(point.x(), point.y(), point.z()))
        - support.d;
    if plane_distance.abs() > tolerance {
        return false;
    }
    let mut sign = None;
    for index in 0..support.points.len() {
        let start = support.points[index];
        let end = support.points[(index + 1) % support.points.len()];
        let side = (end - start).cross(point - start).dot(support.normal);
        let scale = (end - start).length().max(1.0);
        if side.abs() <= tolerance * scale {
            continue;
        }
        if sign.is_some_and(|sign| side * sign < 0.0) {
            return false;
        }
        sign.get_or_insert_with(|| side.signum());
    }
    true
}

fn validate_hold_line(
    first_contact: &NurbsCurve,
    second_contact: &NurbsCurve,
    first_face: FaceId,
    second_face: FaceId,
    hold: FaceFaceHoldLine,
    tolerance: Tolerance,
) -> Result<(), BlendError> {
    if !finite_point(hold.start)
        || !finite_point(hold.end)
        || (hold.end - hold.start).length() <= tolerance.linear
    {
        return Err(BlendError::InvalidInput {
            reason: "hold line must be a finite non-degenerate segment".into(),
        });
    }
    let contact = if hold.support == first_face {
        first_contact
    } else if hold.support == second_face {
        second_contact
    } else {
        return Err(BlendError::InvalidInput {
            reason: "hold-line support must belong to a selected face set".into(),
        });
    };
    if segment_curve_deviation(hold.start, hold.end, contact) > tolerance.linear {
        return Err(unsupported(
            "hold line does not match the exact constant-radius contact",
        ));
    }
    Ok(())
}

fn segment_curve_deviation(start: Point3, end: Point3, curve: &NurbsCurve) -> f64 {
    let (curve_start, curve_end) = curve.domain();
    let first = curve.evaluate(curve_start);
    let last = curve.evaluate(curve_end);
    let direct = (first - start).length().max((last - end).length());
    let reversed = (first - end).length().max((last - start).length());
    direct.min(reversed).max(
        (curve.evaluate(f64::midpoint(curve_start, curve_end)) - (start + (end - start) * 0.5))
            .length(),
    )
}

fn finite_point(point: Point3) -> bool {
    point.x().is_finite() && point.y().is_finite() && point.z().is_finite()
}

fn finite_vec(vector: Vec3) -> bool {
    vector.x().is_finite() && vector.y().is_finite() && vector.z().is_finite()
}
