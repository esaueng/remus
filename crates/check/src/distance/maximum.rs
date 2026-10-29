//! Global maximum-distance witnesses for qualified polygonal and spherical bodies.

use remus_geometry::convert::certify_affine_nurbs_plane;
use remus_math::context::OperationContext;
use remus_math::curves2d::Curve2D;
use remus_math::det_hash::DetHashSet;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Preparation visits across both bodies, separate from vertex comparisons.
const MAX_PREPARATION_ITEMS: usize = 1_000_000;
/// Bound non-cancellable native authority/coverage helper calls per face.
const MAX_FACE_USES: usize = 4096;
const AXIS_ROUNDOFF: f64 = 128.0 * f64::EPSILON;

/// A global maximum and corresponding boundary witnesses.
#[derive(Debug, Clone)]
pub struct MaximumDistanceResult {
    /// Maximum separation of the two qualified boundaries.
    pub distance: f64,
    /// Boundary witness on the first body.
    pub point_a: Point3,
    /// Boundary witness on the second body.
    pub point_b: Point3,
}

/// Typed refusal of an unsupported, invalid, unresolved, or bounded-out query.
#[derive(Debug, thiserror::Error)]
pub enum MaximumDistanceError {
    /// A referenced entity or authoritative boundary is invalid.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),
    /// The supplied topology has invalid geometric data.
    #[error("invalid maximum-distance geometry: {0}")]
    InvalidGeometry(&'static str),
    /// This first supported family does not cover a boundary.
    #[error("unsupported maximum-distance geometry: {0}")]
    Unsupported(&'static str),
    /// Finite arithmetic cannot resolve a required certificate or witness.
    #[error("maximum-distance arithmetic is unresolved: {0}")]
    Unresolved(&'static str),
    /// The query exceeded a hard work bound.
    #[error("maximum-distance {resource} requires {required} items; limit is {limit}")]
    ResourceLimit {
        /// Work being bounded.
        resource: &'static str,
        /// Required items (saturated on overflow).
        required: usize,
        /// Permitted items.
        limit: usize,
    },
    /// Cooperative cancellation was requested.
    #[error("maximum-distance query cancelled")]
    Cancelled,
}

impl MaximumDistanceError {
    /// Stable diagnostic code shared by native and browser callers.
    #[must_use]
    pub const fn kernel_code(&self) -> &'static str {
        match self {
            Self::Topology(_) => "maximum-distance:invalid-topology",
            Self::InvalidGeometry(_) => "maximum-distance:invalid-geometry",
            Self::Unsupported(_) => "maximum-distance:unsupported",
            Self::Unresolved(_) => "maximum-distance:unresolved",
            Self::ResourceLimit { .. } => "maximum-distance:resource-limit",
            Self::Cancelled => "maximum-distance:cancelled",
        }
    }
}

enum Body {
    Polygonal(Vec<Point3>),
    Sphere(SphericalSurface),
}

struct Preparation<'a> {
    context: &'a OperationContext,
    visited: usize,
}

impl Preparation<'_> {
    fn visit(&mut self, count: usize) -> Result<(), MaximumDistanceError> {
        cancelled(self.context)?;
        self.visited = self.visited.saturating_add(count);
        budget("preparation", self.visited, MAX_PREPARATION_ITEMS)
    }
}

/// Compute a global maximum on straight-Line planar polyhedra and complete
/// native spheres, including mixed pairs. All cavity boundaries are inspected.
///
/// Planar carriers may also be algebraically certified degree-one affine
/// NURBS patches, with complete finite-domain and per-use pcurve proofs.
///
/// Inputs must be valid BReps, with closed shells and simple, properly nested
/// trim loops. Qualification here checks supported carriers and their numeric
/// and authoritative-boundary representation.
///
/// No sampled fallback is used. The query is read-only and returns ordinary
/// floating-point witnesses, not an interval-arithmetic error certificate.
///
/// # Errors
/// Returns typed invalid/unsupported/unresolved, resource, or cancellation
/// errors. Preparation permits one million shell/face/loop/coedge items across both
/// bodies and at most 4096 coedge uses per face. Candidate comparisons use the
/// default one-million vertex-pair budget.
pub fn solid_to_solid(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
) -> Result<MaximumDistanceResult, MaximumDistanceError> {
    solid_to_solid_with_context(topo, a, b, &OperationContext::new())
}

/// Context-aware form of [`solid_to_solid`].
///
/// `vertex_pairs` bounds the actual
/// Cartesian vertex comparisons, mixed vertex comparisons, or one sphere pair.
/// Existing fallback policy never authorizes an approximate maximum.
///
/// # Errors
/// Returns the same errors as [`solid_to_solid`].
pub fn solid_to_solid_with_context(
    topo: &Topology,
    a: SolidId,
    b: SolidId,
    context: &OperationContext,
) -> Result<MaximumDistanceResult, MaximumDistanceError> {
    cancelled(context)?;
    if !context.tolerance.linear.is_finite() || context.tolerance.linear <= 0.0 {
        return Err(MaximumDistanceError::InvalidGeometry(
            "invalid linear tolerance",
        ));
    }
    let mut preparation = Preparation {
        context,
        visited: 0,
    };
    let first = prepare(topo, a, &mut preparation)?;
    let second = prepare(topo, b, &mut preparation)?;
    let candidates = match (&first, &second) {
        (Body::Polygonal(a), Body::Polygonal(b)) => {
            a.len()
                .checked_mul(b.len())
                .ok_or(MaximumDistanceError::ResourceLimit {
                    resource: "candidate comparisons (count overflow)",
                    required: usize::MAX,
                    limit: context.budgets.vertex_pairs,
                })?
        }
        (Body::Polygonal(vertices), Body::Sphere(_))
        | (Body::Sphere(_), Body::Polygonal(vertices)) => vertices.len(),
        (Body::Sphere(_), Body::Sphere(_)) => 1,
    };
    budget(
        "candidate comparisons",
        candidates,
        context.budgets.vertex_pairs,
    )?;
    match (&first, &second) {
        (Body::Polygonal(a), Body::Polygonal(b)) => {
            let mut best = None;
            for &point_a in a {
                for &point_b in b {
                    cancelled(context)?;
                    improve(&mut best, result(point_a, point_b)?);
                }
            }
            best.ok_or(MaximumDistanceError::InvalidGeometry("empty boundary"))
        }
        (Body::Sphere(a), Body::Sphere(b)) => {
            cancelled(context)?;
            let direction = direction(b.center() - a.center(), a.x_axis())?;
            let point_a = radial(a, -direction, context)?;
            let point_b = radial(b, direction, context)?;
            result(point_a, point_b)
        }
        (Body::Sphere(sphere), Body::Polygonal(vertices)) => {
            sphere_polygon(sphere, vertices, true, context)
        }
        (Body::Polygonal(vertices), Body::Sphere(sphere)) => {
            sphere_polygon(sphere, vertices, false, context)
        }
    }
}

fn prepare(
    topo: &Topology,
    solid_id: SolidId,
    work: &mut Preparation<'_>,
) -> Result<Body, MaximumDistanceError> {
    let solid = topo.solid(solid_id)?;
    for shell_id in std::iter::once(solid.outer_shell()).chain(solid.inner_shells().iter().copied())
    {
        work.visit(1)?;
        work.visit(topo.shell(shell_id)?.faces().len())?;
    }
    // Count first so the complete shared traversal cannot allocate an
    // unbounded face vector before this query applies its preparation cap.
    let faces = remus_topology::explorer::solid_faces(topo, solid_id)?;
    if faces.is_empty() {
        return Err(MaximumDistanceError::InvalidGeometry("empty boundary"));
    }
    let spherical = solid.inner_shells().is_empty()
        && faces.len() == 2
        && faces.iter().all(|&id| {
            topo.face(id)
                .is_ok_and(|f| matches!(f.surface(), FaceSurface::Sphere(_)))
        });
    let mut points = Vec::new();
    let mut seen_vertices = DetHashSet::default();
    let mut seen_faces = DetHashSet::default();
    for &face_id in &faces {
        cancelled(work.context)?;
        if !seen_faces.insert(face_id) {
            return Err(MaximumDistanceError::InvalidGeometry("duplicate face use"));
        }
        let face = topo.face(face_id)?;
        let affine_plane = if let FaceSurface::Nurbs(surface) = face.surface() {
            Some(
                certify_affine_nurbs_plane(surface, work.context.tolerance.linear).ok_or(
                    MaximumDistanceError::Unsupported("uncertified affine NURBS plane"),
                )?,
            )
        } else {
            None
        };
        match face.surface() {
            FaceSurface::Plane { normal, d } if !spherical => {
                if !finite(*normal) || !d.is_finite() || length(*normal) == 0.0 {
                    return Err(MaximumDistanceError::InvalidGeometry("invalid plane"));
                }
            }
            FaceSurface::Sphere(sphere) if spherical => check_sphere_frame(sphere)?,
            FaceSurface::Nurbs(_) if affine_plane.is_some() => {}
            FaceSurface::Plane { .. }
            | FaceSurface::Sphere(_)
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_) => {
                return Err(MaximumDistanceError::Unsupported(
                    "boundary is not a supported planar polyhedron or complete native sphere",
                ));
            }
        }
        let loops = topo
            .loops_of_face(face_id)
            .ok_or(MaximumDistanceError::InvalidGeometry(
                "missing authoritative loops",
            ))?;
        let mut face_uses = 0usize;
        for &loop_id in loops {
            work.visit(1)?;
            face_uses = face_uses.saturating_add(topo.face_loop(loop_id)?.coedges().len());
            budget("face boundary uses", face_uses, MAX_FACE_USES)?;
        }
        // The preceding cap bounds each helper call. Cancellation is checked
        // around helpers and on each individual boundary item below.
        cancelled(work.context)?;
        remus_topology::validation::validate_face_loops(topo, face_id)?;
        for &loop_id in loops {
            cancelled(work.context)?;
            let boundary = topo.face_loop(loop_id)?;
            if !boundary.is_closed() {
                return Err(MaximumDistanceError::InvalidGeometry("open face boundary"));
            }
            remus_topology::validation::validate_loop_connected(topo, loop_id)?;
            for &coedge_id in boundary.coedges() {
                work.visit(1)?;
                let coedge = topo.coedge(coedge_id)?;
                let edge = topo.edge(coedge.edge())?;
                edge.strict_domain()
                    .map_err(|_| MaximumDistanceError::InvalidGeometry("invalid edge domain"))?;
                if !spherical && !matches!(edge.curve(), EdgeCurve::Line) {
                    return Err(MaximumDistanceError::Unsupported(
                        "curved polygonal boundary",
                    ));
                }
                if let FaceSurface::Nurbs(surface) = face.surface() {
                    let stored = topo
                        .pcurve_oriented(coedge.edge(), face_id, coedge.is_forward())
                        .ok_or(MaximumDistanceError::Unsupported(
                            "missing affine-plane pcurve use",
                        ))?;
                    // Reject other representations before the generic helper
                    // traverses their controls. Only this constant-size family
                    // belongs to the preparation budget of this query.
                    let Curve2D::Line(line) = stored.curve() else {
                        return Err(MaximumDistanceError::Unsupported(
                            "uncertified affine-plane pcurve use",
                        ));
                    };
                    let (domain_u, domain_v) = (surface.domain_u(), surface.domain_v());
                    for parameter in [stored.t_start(), stored.t_end()] {
                        let uv = line.evaluate(parameter);
                        if !parameter.is_finite()
                            || !uv.x().is_finite()
                            || !uv.y().is_finite()
                            || !(domain_u.0..=domain_u.1).contains(&uv.x())
                            || !(domain_v.0..=domain_v.1).contains(&uv.y())
                        {
                            return Err(MaximumDistanceError::Unsupported(
                                "affine-plane pcurve leaves its finite carrier domain",
                            ));
                        }
                    }
                    // This helper proves the entire Line2D/affine-surface
                    // image, including the stored use's finite UV domain.
                    // Absence and unsupported proofs never pass vacuously.
                    let proof = remus_topology::validation::check_same_parameter_strict(
                        topo,
                        coedge.edge(),
                        face_id,
                        coedge.is_forward(),
                        0,
                    )
                    .map_err(|error| match error {
                        remus_topology::validation::CurveUseValidationError::SameParameterProofUnavailable { .. } => {
                            MaximumDistanceError::Unresolved("affine-plane pcurve proof arithmetic is unresolved")
                        }
                        _ => MaximumDistanceError::Unsupported("uncertified affine-plane pcurve use"),
                    })?
                    .ok_or(MaximumDistanceError::Unsupported(
                        "missing affine-plane pcurve use",
                    ))?;
                    if proof.max_deviation > work.context.tolerance.linear {
                        return Err(MaximumDistanceError::Unresolved(
                            "affine-plane pcurve error exceeds query tolerance",
                        ));
                    }
                }
                for vertex_id in [edge.start(), edge.end()] {
                    let point = topo.vertex(vertex_id)?.point();
                    if !finite_point(point) {
                        return Err(MaximumDistanceError::InvalidGeometry("non-finite endpoint"));
                    }
                    match face.surface() {
                        FaceSurface::Plane { normal, d } => {
                            check_plane_point(*normal, *d, point, work.context)?;
                        }
                        FaceSurface::Sphere(sphere) => check_radial(sphere, point, work.context)?,
                        FaceSurface::Nurbs(_) => {
                            let Some(plane) = affine_plane.as_ref() else {
                                return Err(MaximumDistanceError::Unsupported(
                                    "uncertified affine NURBS plane",
                                ));
                            };
                            // An affine rectangular domain is convex, and a
                            // Line's image is affine. Both endpoints therefore
                            // certify membership of the complete 3D interval.
                            if plane
                                .parameters(point, work.context.tolerance.linear)
                                .is_none()
                            {
                                return Err(MaximumDistanceError::Unresolved(
                                    "affine-plane edge leaves its finite carrier domain",
                                ));
                            }
                        }
                        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) | FaceSurface::Torus(_) => {
                            unreachable!("carrier qualified above")
                        }
                    }
                    if seen_vertices.insert(vertex_id) {
                        points.push(point);
                    }
                }
                if length(topo.vertex(edge.end())?.point() - topo.vertex(edge.start())?.point())
                    == 0.0
                {
                    return Err(MaximumDistanceError::InvalidGeometry(
                        "collapsed boundary edge",
                    ));
                }
            }
        }
    }
    cancelled(work.context)?;
    if spherical {
        let sphere = super::analytic::complete_sphere(topo, solid_id)
            .map_err(|_| MaximumDistanceError::InvalidGeometry("invalid native sphere boundary"))?
            .ok_or(MaximumDistanceError::Unsupported(
                "sphere boundary does not certify full coverage",
            ))?;
        cancelled(work.context)?;
        Ok(Body::Sphere(sphere))
    } else if points.is_empty() {
        Err(MaximumDistanceError::InvalidGeometry(
            "empty polygonal boundary",
        ))
    } else {
        Ok(Body::Polygonal(points))
    }
}

fn check_plane_point(
    normal: Vec3,
    d: f64,
    point: Point3,
    context: &OperationContext,
) -> Result<(), MaximumDistanceError> {
    let normal_length = length(normal);
    if !normal_length.is_finite() || normal_length == 0.0 {
        return Err(MaximumDistanceError::Unresolved(
            "plane normal magnitude exceeds finite arithmetic",
        ));
    }
    let terms = [
        normal.x() * point.x(),
        normal.y() * point.y(),
        normal.z() * point.z(),
    ];
    let residual = (terms[0] + terms[1] + terms[2]) - d;
    let magnitude = terms.iter().map(|x| x.abs()).sum::<f64>() + d.abs();
    let roundoff = 32.0 * f64::EPSILON * magnitude + 8.0 * f64::from_bits(1);
    let bound = (residual.abs() + roundoff) / normal_length;
    if !bound.is_finite() || bound > context.tolerance.linear {
        return Err(MaximumDistanceError::Unresolved(
            "endpoint planarity cannot be resolved at the requested tolerance",
        ));
    }
    Ok(())
}

fn check_sphere_frame(sphere: &SphericalSurface) -> Result<(), MaximumDistanceError> {
    let (x, y, z) = (sphere.x_axis(), sphere.y_axis(), sphere.z_axis());
    if !finite_point(sphere.center())
        || !sphere.radius().is_finite()
        || sphere.radius() <= 0.0
        || ![x, y, z].into_iter().all(finite)
        || [x, y, z]
            .into_iter()
            .any(|axis| (axis.length_squared() - 1.0).abs() > AXIS_ROUNDOFF)
        || x.dot(y).abs() > AXIS_ROUNDOFF
        || x.dot(z).abs() > AXIS_ROUNDOFF
        || y.dot(z).abs() > AXIS_ROUNDOFF
        || length(x.cross(y) - z) > AXIS_ROUNDOFF
    {
        return Err(MaximumDistanceError::InvalidGeometry(
            "invalid sphere frame or radius",
        ));
    }
    Ok(())
}

fn check_radial(
    sphere: &SphericalSurface,
    point: Point3,
    context: &OperationContext,
) -> Result<(), MaximumDistanceError> {
    let radius = length(point - sphere.center());
    if !radius.is_finite()
        || radius == 0.0
        || (radius - sphere.radius()).abs() > context.tolerance.linear
    {
        return Err(MaximumDistanceError::Unresolved(
            "sphere boundary or witness is not a representable radial point",
        ));
    }
    Ok(())
}

fn radial(
    sphere: &SphericalSurface,
    axis: Vec3,
    context: &OperationContext,
) -> Result<Point3, MaximumDistanceError> {
    let point = sphere.center() + axis * sphere.radius();
    if !finite_point(point) {
        return Err(MaximumDistanceError::Unresolved("sphere witness overflow"));
    }
    check_radial(sphere, point, context)?;
    Ok(point)
}

fn sphere_polygon(
    sphere: &SphericalSurface,
    vertices: &[Point3],
    sphere_first: bool,
    context: &OperationContext,
) -> Result<MaximumDistanceResult, MaximumDistanceError> {
    let mut best = None;
    for &point in vertices {
        cancelled(context)?;
        let axis = direction(sphere.center() - point, sphere.x_axis())?;
        let sphere_point = radial(sphere, axis, context)?;
        improve(
            &mut best,
            if sphere_first {
                result(sphere_point, point)?
            } else {
                result(point, sphere_point)?
            },
        );
    }
    best.ok_or(MaximumDistanceError::InvalidGeometry(
        "empty polygonal boundary",
    ))
}

fn result(point_a: Point3, point_b: Point3) -> Result<MaximumDistanceResult, MaximumDistanceError> {
    let distance = length(point_b - point_a);
    if !distance.is_finite() {
        return Err(MaximumDistanceError::Unresolved(
            "distance exceeds finite arithmetic",
        ));
    }
    Ok(MaximumDistanceResult {
        distance,
        point_a,
        point_b,
    })
}

fn improve(best: &mut Option<MaximumDistanceResult>, candidate: MaximumDistanceResult) {
    if best
        .as_ref()
        .is_none_or(|old| candidate.distance > old.distance)
    {
        *best = Some(candidate);
    }
}

fn direction(delta: Vec3, fallback: Vec3) -> Result<Vec3, MaximumDistanceError> {
    if !finite(delta) {
        return Err(MaximumDistanceError::Unresolved(
            "center separation overflow",
        ));
    }
    let scale = delta.x().abs().max(delta.y().abs()).max(delta.z().abs());
    if scale == 0.0 {
        return Ok(fallback);
    }
    let scaled = Vec3::new(delta.x() / scale, delta.y() / scale, delta.z() / scale);
    Ok(scaled * (1.0 / length(scaled)))
}

fn finite(point: Vec3) -> bool {
    point.0.into_iter().all(f64::is_finite)
}
fn finite_point(point: Point3) -> bool {
    point.0.into_iter().all(f64::is_finite)
}
fn length(vector: Vec3) -> f64 {
    vector.x().hypot(vector.y()).hypot(vector.z())
}
fn cancelled(context: &OperationContext) -> Result<(), MaximumDistanceError> {
    context
        .check_cancelled()
        .map_err(|_| MaximumDistanceError::Cancelled)
}
fn budget(
    resource: &'static str,
    required: usize,
    limit: usize,
) -> Result<(), MaximumDistanceError> {
    if required > limit {
        Err(MaximumDistanceError::ResourceLimit {
            resource,
            required,
            limit,
        })
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overflowed_normal_norm_never_turns_off_plane_residual_into_zero() {
        // The normalized distance is1/sqrt(2), not0; finite normal
        // components cannot authorize division by an infinite norm.
        assert!(matches!(
            check_plane_point(
                Vec3::new(1.7e308, 1.7e308, 0.0),
                0.0,
                Point3::new(1.0, 0.0, 0.0),
                &OperationContext::new()
            ),
            Err(MaximumDistanceError::Unresolved(_))
        ));
    }
}
