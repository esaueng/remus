//! Convert analytic geometry to B-spline representation.
//!
//! Replaces every analytic surface (Plane, Cylinder, Cone, Sphere, Torus) with a
//! NURBS surface and every analytic curve (Line, Circle, Ellipse, Hyperbola,
//! Parabola) with a NURBS curve.
//!
//! Surfaces use the rational NURBS representations exposed by
//! [`remus_geometry::convert`]. Curves use the rational quadratic arc form for
//! Circle/Ellipse and a degree-1 form for Line.
//!
//! # Limitation: pcurves are dropped
//!
//! Stored pcurves on the (edge, face) registry are removed for every face whose
//! surface is converted. The (u, v) coordinates of pcurves on an analytic
//! surface do not map linearly to the equivalent NURBS surface (e.g. cylindrical
//! `u` is angular, but the NURBS u is rational), so the stored pcurves would
//! silently misalign without re-projection. Callers that need pcurves should
//! recompute them after this op.

use crate::construct::convert_curve::{hyperbola_to_nurbs, parabola_to_nurbs};
use remus_geometry::convert::curve_to_nurbs::{circle_to_nurbs, ellipse_to_nurbs, line_to_nurbs};
use remus_geometry::convert::surface_to_nurbs::{
    cone_to_nurbs, cylinder_to_nurbs, sphere_to_nurbs, torus_to_nurbs,
};

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::HealError;

/// Convert all analytic geometry in a solid to B-Spline representation.
///
/// Returns the total number of faces and edges that were converted (NURBS
/// faces/edges are skipped and not counted).
///
/// # Errors
///
/// Returns [`HealError`] if any topology lookup, NURBS construction, or face
/// surface replacement fails.
pub fn convert_solid_to_bspline(
    topo: &mut Topology,
    solid_id: SolidId,
) -> Result<usize, HealError> {
    let face_ids = solid_faces(topo, solid_id)?;
    let edge_ids = solid_edges(topo, solid_id)?;

    // Plan every conversion before changing topology. In particular, a later
    // curved edge with missing parameter authority must not leave earlier
    // faces converted or their pcurves removed.
    let edge_conversions = edge_ids
        .iter()
        .map(|&eid| plan_edge_curve_conversion(topo, eid))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    let face_conversions = face_ids
        .iter()
        .map(|&fid| plan_face_surface_conversion(topo, fid))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();

    let converted = face_conversions.len() + edge_conversions.len();
    for conversion in face_conversions {
        for (edge, forward) in conversion.pcurve_uses {
            let _ = topo.remove_pcurve_oriented(edge, conversion.face, forward);
        }
        topo.face_mut(conversion.face)?
            .set_surface(FaceSurface::Nurbs(conversion.surface));
    }
    for conversion in edge_conversions {
        let edge = topo.edge_mut(conversion.edge)?;
        edge.set_curve(EdgeCurve::NurbsCurve(conversion.curve));
        edge.set_trim(Some(conversion.domain));
    }

    Ok(converted)
}

struct FaceConversion {
    face: FaceId,
    surface: NurbsSurface,
    pcurve_uses: Vec<(EdgeId, bool)>,
}

fn plan_face_surface_conversion(
    topo: &Topology,
    fid: FaceId,
) -> Result<Option<FaceConversion>, HealError> {
    let surface = topo.face(fid)?.surface().clone();
    let nurbs = match surface {
        FaceSurface::Plane { normal, d } => plane_face_to_nurbs(topo, fid, normal, d)?,
        FaceSurface::Cylinder(c) => {
            let v_range = axial_v_range(topo, fid, c.origin(), c.axis())?;
            cylinder_to_nurbs(&c, v_range)?
        }
        FaceSurface::Cone(c) => {
            let mut v_range = generator_v_range(topo, fid, c.apex())?;
            // Cone has a parametric singularity at v=0 (the apex). Pull v_min
            // strictly positive to keep the rational NURBS construction stable.
            if v_range.0 < 1e-9 {
                v_range.0 = 1e-9;
            }
            if v_range.1 <= v_range.0 {
                v_range.1 = v_range.0 + 1.0;
            }
            cone_to_nurbs(&c, v_range)?
        }
        FaceSurface::Sphere(s) => sphere_to_nurbs(&s)?,
        FaceSurface::Torus(t) => torus_to_nurbs(&t)?,
        FaceSurface::Nurbs(_) => return Ok(None),
    };

    // Capture per-use keys so the apply phase can remove both seam branches
    // without the fallible `(edge, face)` ambiguity boundary.
    let pcurve_uses = topo
        .pcurves_for_face(fid)
        .into_iter()
        .map(|(edge, forward, _)| (edge, forward))
        .collect();
    Ok(Some(FaceConversion {
        face: fid,
        surface: nurbs,
        pcurve_uses,
    }))
}

struct EdgeConversion {
    edge: EdgeId,
    curve: NurbsCurve,
    domain: (f64, f64),
}

fn plan_edge_curve_conversion(
    topo: &Topology,
    eid: EdgeId,
) -> Result<Option<EdgeConversion>, HealError> {
    let edge = topo.edge(eid)?;
    let curve = edge.curve().clone();
    let start_v = edge.start();
    let end_v = edge.end();
    let start_vertex = topo.vertex(start_v)?;
    let end_vertex = topo.vertex(end_v)?;
    let start_tolerance = start_vertex.tolerance();
    let end_tolerance = end_vertex.tolerance();
    for (label, tolerance) in [
        ("start vertex", start_tolerance),
        ("end vertex", end_tolerance),
    ] {
        if !tolerance.is_finite() || tolerance.is_sign_negative() {
            return Err(HealError::AnalysisFailed(format!(
                "edge {eid:?} has invalid {label} conversion tolerance {tolerance}"
            )));
        }
    }
    if let Some(tolerance) = edge.tolerance()
        && (!tolerance.is_finite() || tolerance.is_sign_negative())
    {
        return Err(HealError::AnalysisFailed(format!(
            "edge {eid:?} has invalid explicit conversion tolerance {tolerance}"
        )));
    }
    if matches!(curve, EdgeCurve::NurbsCurve(_)) {
        return Ok(None);
    }
    let start_pt = start_vertex.point();
    let end_pt = end_vertex.point();
    let source_domain = edge.strict_domain().map_err(|error| {
        HealError::AnalysisFailed(format!(
            "edge {eid:?} cannot be converted without parameter authority: {error}"
        ))
    })?;

    let nurbs = match &curve {
        EdgeCurve::Line => {
            // Skip near-degenerate edges. Use the topology linear tolerance so
            // we don't propagate a `line_to_nurbs` rejection (which would abort
            // the whole solid conversion) for edges that are noise-only-long.
            if (end_pt - start_pt).length() < remus_math::tolerance::Tolerance::new().linear {
                return Ok(None);
            }
            line_to_nurbs(start_pt, end_pt)?
        }
        EdgeCurve::Circle(c) => circle_to_nurbs(c, source_domain.0, source_domain.1)?,
        EdgeCurve::Ellipse(e) => ellipse_to_nurbs(e, source_domain.0, source_domain.1)?,
        EdgeCurve::Hyperbola(h) => {
            if source_domain.0 < source_domain.1 {
                hyperbola_to_nurbs(h, source_domain.0, source_domain.1)?
            } else {
                hyperbola_to_nurbs(h, source_domain.1, source_domain.0)?.reversed()
            }
        }
        EdgeCurve::Parabola(p) => {
            if source_domain.0 < source_domain.1 {
                parabola_to_nurbs(p, source_domain.0, source_domain.1)?
            } else {
                parabola_to_nurbs(p, source_domain.1, source_domain.0)?.reversed()
            }
        }
        EdgeCurve::NurbsCurve(_) => return Ok(None),
    };

    let vertex_tolerance = start_tolerance.max(end_tolerance);
    let endpoint_tolerance = edge.effective_tolerance(vertex_tolerance);
    let domain = nurbs.domain();
    certify_curve_conversion(
        eid,
        &curve,
        source_domain,
        start_pt,
        end_pt,
        &nurbs,
        domain,
        endpoint_tolerance,
    )?;

    Ok(Some(EdgeConversion {
        edge: eid,
        curve: nurbs,
        domain,
    }))
}

#[allow(clippy::too_many_arguments)]
fn certify_curve_conversion(
    eid: EdgeId,
    source: &EdgeCurve,
    source_domain: (f64, f64),
    start: Point3,
    end: Point3,
    converted: &NurbsCurve,
    converted_domain: (f64, f64),
    tolerance: f64,
) -> Result<(), HealError> {
    const FRACTIONS: [f64; 9] = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0];
    for (label, parameter, vertex) in [
        ("start", source_domain.0, start),
        ("end", source_domain.1, end),
    ] {
        let residual = (source.evaluate_with_endpoints(parameter, start, end) - vertex).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(HealError::AnalysisFailed(format!(
                "edge {eid:?} source {label} residual {residual} exceeds tolerance {tolerance}"
            )));
        }
    }

    // Exact analytic conversions must remain on their source curve away from
    // the endpoints too. Rational conics reparameterize the source, so compare
    // general samples through the analytic inverse; the symmetric midpoint is
    // additionally required to represent the source range's half parameter.
    for fraction in FRACTIONS {
        let converted_parameter =
            (converted_domain.1 - converted_domain.0).mul_add(fraction, converted_domain.0);
        let actual = converted.evaluate(converted_parameter);
        let source_parameter = match source {
            EdgeCurve::Line => fraction,
            EdgeCurve::Circle(curve) => curve.project(actual),
            EdgeCurve::Ellipse(curve) => curve.project(actual),
            EdgeCurve::Hyperbola(curve) => curve.project(actual),
            EdgeCurve::Parabola(curve) => curve.project(actual),
            EdgeCurve::NurbsCurve(_) => return Ok(()),
        };
        let on_source = source.evaluate_with_endpoints(source_parameter, start, end);
        let residual = (actual - on_source).length();
        if !residual.is_finite() || residual > tolerance {
            return Err(HealError::AnalysisFailed(format!(
                "edge {eid:?} conversion oracle residual {residual} at fraction {fraction} exceeds tolerance {tolerance}"
            )));
        }
    }

    let source_midpoint = f64::midpoint(source_domain.0, source_domain.1);
    let converted_midpoint = f64::midpoint(converted_domain.0, converted_domain.1);
    let expected_midpoint = source.evaluate_with_endpoints(source_midpoint, start, end);
    let midpoint_residual = (converted.evaluate(converted_midpoint) - expected_midpoint).length();
    if !midpoint_residual.is_finite() || midpoint_residual > tolerance {
        return Err(HealError::AnalysisFailed(format!(
            "edge {eid:?} conversion midpoint oracle residual {midpoint_residual} exceeds tolerance {tolerance}"
        )));
    }
    Ok(())
}

/// Conservative bounds of every boundary point projected onto `axis` from
/// `origin`. Analytic curves use closed-form projection enclosures; a
/// positive-weight NURBS stays in its control-point convex hull, so projecting
/// that hull is a certificate rather than a sampling heuristic.
fn boundary_projection_bounds(
    topo: &Topology,
    face_id: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<(f64, f64), HealError> {
    let face = topo.face(face_id)?;
    let mut bounds: Option<(f64, f64)> = None;
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oe in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oe.edge())?;
            if let EdgeCurve::NurbsCurve(curve) = edge.curve()
                && curve.validate().is_err()
            {
                return Err(HealError::AnalysisFailed(format!(
                    "face {face_id:?} edge {:?} has no certifiable projection bound",
                    oe.edge()
                )));
            }
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let domain = edge.strict_domain().map_err(|error| {
                HealError::AnalysisFailed(format!(
                    "face {face_id:?} edge {:?} has no parameter authority: {error}",
                    oe.edge()
                ))
            })?;
            let edge_bounds =
                edge_projection_bounds(edge.curve(), domain, start, end, origin, axis).ok_or_else(
                    || {
                        HealError::AnalysisFailed(format!(
                            "face {face_id:?} edge {:?} has no certifiable projection bound",
                            oe.edge()
                        ))
                    },
                )?;
            bounds = Some(bounds.map_or(edge_bounds, |current| {
                (current.0.min(edge_bounds.0), current.1.max(edge_bounds.1))
            }));
        }
    }
    bounds.ok_or_else(|| {
        HealError::AnalysisFailed(format!("face {face_id:?} has no boundary edges to bound"))
    })
}

fn edge_projection_bounds(
    curve: &EdgeCurve,
    domain: (f64, f64),
    start: Point3,
    end: Point3,
    origin: Point3,
    axis: Vec3,
) -> Option<(f64, f64)> {
    let offset = |point: Point3| axis.dot(point - origin);
    match curve {
        EdgeCurve::Line => expanded_bounds(
            [offset(start), offset(end)],
            point_projection_scale(start, origin, axis)
                .max(point_projection_scale(end, origin, axis)),
        ),
        EdgeCurve::Circle(circle) => harmonic_projection_bounds(
            offset(circle.center()),
            circle.radius() * axis.dot(circle.u_axis()),
            circle.radius() * axis.dot(circle.v_axis()),
            point_projection_scale(circle.center(), origin, axis)
                .max(circle.radius() * dot_product_scale(axis, circle.u_axis()))
                .max(circle.radius() * dot_product_scale(axis, circle.v_axis())),
        ),
        EdgeCurve::Ellipse(ellipse) => harmonic_projection_bounds(
            offset(ellipse.center()),
            ellipse.semi_major() * axis.dot(ellipse.u_axis()),
            ellipse.semi_minor() * axis.dot(ellipse.v_axis()),
            point_projection_scale(ellipse.center(), origin, axis)
                .max(ellipse.semi_major() * dot_product_scale(axis, ellipse.u_axis()))
                .max(ellipse.semi_minor() * dot_product_scale(axis, ellipse.v_axis())),
        ),
        EdgeCurve::Hyperbola(hyperbola) => {
            let constant = offset(hyperbola.center());
            let cosh_coefficient = hyperbola.semi_major() * axis.dot(hyperbola.u_axis());
            let sinh_coefficient = hyperbola.semi_minor() * axis.dot(hyperbola.v_axis());
            let parameter_extent = domain.0.abs().max(domain.1.abs());
            let cosh_extent = parameter_extent.cosh();
            let sinh_extent = parameter_extent.sinh().abs();
            let radius = cosh_coefficient
                .abs()
                .mul_add(cosh_extent, sinh_coefficient.abs() * sinh_extent);
            let geometry_scale = point_projection_scale(hyperbola.center(), origin, axis)
                .max(
                    hyperbola.semi_major()
                        * dot_product_scale(axis, hyperbola.u_axis())
                        * cosh_extent,
                )
                .max(
                    hyperbola.semi_minor()
                        * dot_product_scale(axis, hyperbola.v_axis())
                        * sinh_extent,
                );
            expanded_bounds([constant - radius, constant + radius], geometry_scale)
        }
        EdgeCurve::Parabola(parabola) => {
            let constant = offset(parabola.vertex());
            let linear = axis.dot(parabola.u_axis());
            let quadratic = axis.dot(parabola.axis_dir()) / (4.0 * parabola.focal_length());
            let parameter_extent = domain.0.abs().max(domain.1.abs());
            let squared_extent = parameter_extent * parameter_extent;
            let radius = linear
                .abs()
                .mul_add(parameter_extent, quadratic.abs() * squared_extent);
            let geometry_scale = point_projection_scale(parabola.vertex(), origin, axis)
                .max(dot_product_scale(axis, parabola.u_axis()) * parameter_extent)
                .max(
                    dot_product_scale(axis, parabola.axis_dir()) * squared_extent
                        / (4.0 * parabola.focal_length()),
                );
            expanded_bounds([constant - radius, constant + radius], geometry_scale)
        }
        EdgeCurve::NurbsCurve(nurbs) => {
            if nurbs
                .weights()
                .iter()
                .any(|weight| !weight.is_finite() || *weight <= 0.0)
            {
                return None;
            }
            let projections: Vec<_> = nurbs.control_points().iter().copied().map(offset).collect();
            let scale = nurbs
                .control_points()
                .iter()
                .copied()
                .map(|point| point_projection_scale(point, origin, axis))
                .fold(1.0_f64, f64::max);
            expanded_bounds(projections, scale)
        }
    }
}

fn harmonic_projection_bounds(
    constant: f64,
    cosine: f64,
    sine: f64,
    geometry_scale: f64,
) -> Option<(f64, f64)> {
    let amplitude = cosine.hypot(sine);
    expanded_bounds(
        [constant - amplitude, constant + amplitude],
        geometry_scale.max(cosine.abs()).max(sine.abs()),
    )
}

fn point_projection_scale(point: Point3, origin: Point3, axis: Vec3) -> f64 {
    axis.x()
        .abs()
        .mul_add(
            point.x().abs() + origin.x().abs(),
            axis.y().abs().mul_add(
                point.y().abs() + origin.y().abs(),
                axis.z().abs() * (point.z().abs() + origin.z().abs()),
            ),
        )
        .max(1.0)
}

fn dot_product_scale(left: Vec3, right: Vec3) -> f64 {
    left.x().abs().mul_add(
        right.x().abs(),
        left.y()
            .abs()
            .mul_add(right.y().abs(), left.z().abs() * right.z().abs()),
    )
}

fn expanded_bounds(
    values: impl IntoIterator<Item = f64>,
    numeric_scale: f64,
) -> Option<(f64, f64)> {
    if !numeric_scale.is_finite() {
        return None;
    }
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    let mut evaluated_scale = numeric_scale.max(1.0);
    for value in values {
        if !value.is_finite() {
            return None;
        }
        minimum = minimum.min(value);
        maximum = maximum.max(value);
        evaluated_scale = evaluated_scale.max(value.abs());
    }
    if !minimum.is_finite() || !maximum.is_finite() {
        return None;
    }
    let arithmetic_band = 128.0 * f64::EPSILON * evaluated_scale;
    let lower = minimum - arithmetic_band;
    let upper = maximum + arithmetic_band;
    (lower.is_finite() && upper.is_finite()).then_some((lower, upper))
}

fn axial_v_range(
    topo: &Topology,
    face_id: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<(f64, f64), HealError> {
    nondegenerate_range(boundary_projection_bounds(topo, face_id, origin, axis)?)
}

/// Extent along a cone's GENERATOR direction, measured from the apex.
///
/// `cone_to_nurbs` documents `v_range` as "the extent along the cone's
/// generator direction from the apex" -- the ruling line, not the axis. Feeding
/// it an axial extent under-reports by `cos(half_angle)`: for
/// `make_cone(6, 2, 12)` that is 5.1%, and the converted patch stopped 0.92
/// short of its own base circle, so rays crossing the cone there found no
/// surface at all and point-in-solid classification counted the wrong parity.
///
/// A point that lies ON the cone sits on a ruling through the apex, so its
/// distance to the apex IS its generator coordinate. Certified coordinate
/// projection intervals give a conservative distance interval without
/// reconstructing the generator from the half-angle.
fn generator_v_range(
    topo: &Topology,
    face_id: FaceId,
    apex: Point3,
) -> Result<(f64, f64), HealError> {
    let x = boundary_projection_bounds(topo, face_id, apex, Vec3::new(1.0, 0.0, 0.0))?;
    let y = boundary_projection_bounds(topo, face_id, apex, Vec3::new(0.0, 1.0, 0.0))?;
    let z = boundary_projection_bounds(topo, face_id, apex, Vec3::new(0.0, 0.0, 1.0))?;
    let minimum = interval_min_abs(x)
        .hypot(interval_min_abs(y))
        .hypot(interval_min_abs(z));
    let maximum =
        x.0.abs()
            .max(x.1.abs())
            .hypot(y.0.abs().max(y.1.abs()))
            .hypot(z.0.abs().max(z.1.abs()));
    if !minimum.is_finite() || !maximum.is_finite() || maximum <= 0.0 {
        return Err(HealError::AnalysisFailed(format!(
            "face {face_id:?} has no finite generator-distance bound"
        )));
    }
    Ok((minimum, maximum))
}

fn interval_min_abs(interval: (f64, f64)) -> f64 {
    if interval.0 <= 0.0 && interval.1 >= 0.0 {
        0.0
    } else {
        interval.0.abs().min(interval.1.abs())
    }
}

fn nondegenerate_range(range: (f64, f64)) -> Result<(f64, f64), HealError> {
    if !range.0.is_finite() || !range.1.is_finite() {
        return Err(HealError::AnalysisFailed(
            "boundary projection range is not finite".to_string(),
        ));
    }
    if range.0 < range.1 {
        return Ok(range);
    }
    let padding = range.0.abs().max(1.0) * 128.0 * f64::EPSILON;
    let padding = padding.max(1.0);
    Ok((range.0 - padding, range.1 + padding))
}

/// Build a NURBS plane surface that comfortably contains every boundary curve
/// of `face_id`.
fn plane_face_to_nurbs(
    topo: &Topology,
    face_id: FaceId,
    normal: Vec3,
    d: f64,
) -> Result<NurbsSurface, HealError> {
    let (u_axis, v_axis) = plane_frame_axes(normal);
    let plane_origin = Point3::new(0.0, 0.0, 0.0) + normal * d;

    let (u_min, u_max) = nondegenerate_range(boundary_projection_bounds(
        topo,
        face_id,
        plane_origin,
        u_axis,
    )?)?;
    let (v_min, v_max) = nondegenerate_range(boundary_projection_bounds(
        topo,
        face_id,
        plane_origin,
        v_axis,
    )?)?;
    let margin_u = 0.1 * (u_max - u_min);
    let margin_v = 0.1 * (v_max - v_min);

    let u_range = (u_min - margin_u, u_max + margin_u);
    let v_range = (v_min - margin_v, v_max + margin_v);

    let cp = vec![
        vec![
            plane_origin + u_axis * u_range.0 + v_axis * v_range.0,
            plane_origin + u_axis * u_range.0 + v_axis * v_range.1,
        ],
        vec![
            plane_origin + u_axis * u_range.1 + v_axis * v_range.0,
            plane_origin + u_axis * u_range.1 + v_axis * v_range.1,
        ],
    ];
    let weights = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
    let knots_u = vec![u_range.0, u_range.0, u_range.1, u_range.1];
    let knots_v = vec![v_range.0, v_range.0, v_range.1, v_range.1];

    Ok(NurbsSurface::new(1, 1, knots_u, knots_v, cp, weights)?)
}

fn plane_frame_axes(normal: Vec3) -> (Vec3, Vec3) {
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u_raw = normal.cross(seed);
    let u_axis = u_raw.normalize().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let v_axis = normal.cross(u_axis);
    (u_axis, v_axis)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use std::f64::consts::{PI, TAU};

    use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::surfaces::{
        ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
    };
    use remus_math::traits::ParametricCurve;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::pcurve::PCurve;
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    fn x_axis() -> Vec3 {
        Vec3::new(1.0, 0.0, 0.0)
    }
    fn z_axis() -> Vec3 {
        Vec3::new(0.0, 0.0, 1.0)
    }

    /// Build a single-face solid with a degenerate-edge wire so we can convert
    /// arbitrary surfaces in isolation. This keeps the per-surface tests
    /// independent of `make_cylinder`/`make_sphere` topology details.
    fn single_face_solid(topo: &mut Topology, surface: FaceSurface, ring: &[Point3]) -> SolidId {
        assert!(ring.len() >= 3, "need at least 3 points for a ring");
        let n = ring.len();
        let vids: Vec<_> = ring
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let mut edges = Vec::new();
        for i in 0..n {
            let eid = topo.add_edge(Edge::new(vids[i], vids[(i + 1) % n], EdgeCurve::Line));
            edges.push(OrientedEdge::new(eid, true));
        }
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        let fid = topo.add_face(Face::new(wire, vec![], surface));
        let shell = topo.add_shell(Shell::new(vec![fid]).unwrap());
        topo.add_solid(Solid::new(shell, vec![]))
    }

    fn single_edge_solid(topo: &mut Topology, edge: EdgeId, closed: bool) -> (SolidId, FaceId) {
        let uses = if closed {
            vec![OrientedEdge::new(edge, true)]
        } else {
            vec![
                OrientedEdge::new(edge, true),
                OrientedEdge::new(edge, false),
            ]
        };
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: z_axis(),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        (topo.add_solid(Solid::new(shell, vec![])), face)
    }

    fn converted_nurbs(topo: &Topology, edge: EdgeId) -> NurbsCurve {
        match topo.edge(edge).unwrap().curve() {
            EdgeCurve::NurbsCurve(curve) => curve.clone(),
            other => panic!("expected NurbsCurve, got {}", other.type_tag()),
        }
    }

    fn assert_converted_endpoints(
        topo: &Topology,
        edge: EdgeId,
        expected_start: Point3,
        expected_end: Point3,
        tolerance: f64,
    ) -> NurbsCurve {
        let curve = converted_nurbs(topo, edge);
        let domain = curve.domain();
        assert_eq!(topo.edge(edge).unwrap().strict_domain().unwrap(), domain);
        assert!((curve.evaluate(domain.0) - expected_start).length() <= tolerance);
        assert!((curve.evaluate(domain.1) - expected_end).length() <= tolerance);
        curve
    }

    fn assert_ellipse_implicit(ellipse: &Ellipse3D, point: Point3, tolerance: f64) {
        let offset = point - ellipse.center();
        let u = offset.dot(ellipse.u_axis()) / ellipse.semi_major();
        let v = offset.dot(ellipse.v_axis()) / ellipse.semi_minor();
        assert!((u.mul_add(u, v * v) - 1.0).abs() <= tolerance);
        assert!(offset.dot(ellipse.normal()).abs() <= tolerance);
    }

    fn assert_hyperbola_implicit(hyperbola: &Hyperbola3D, point: Point3, tolerance: f64) {
        let offset = point - hyperbola.center();
        let u = offset.dot(hyperbola.u_axis()) / hyperbola.semi_major();
        let v = offset.dot(hyperbola.v_axis()) / hyperbola.semi_minor();
        assert!((u.mul_add(u, -(v * v)) - 1.0).abs() <= tolerance);
    }

    fn assert_parabola_implicit(parabola: &Parabola3D, point: Point3, tolerance: f64) {
        let offset = point - parabola.vertex();
        let tangent = offset.dot(parabola.u_axis());
        let axial = offset.dot(parabola.axis_dir());
        let expected_axial = tangent * tangent / (4.0 * parabola.focal_length());
        assert!((axial - expected_axial).abs() <= tolerance);
    }

    #[test]
    fn box_solid_all_faces_become_nurbs() {
        let mut topo = Topology::default();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);

        let n = convert_solid_to_bspline(&mut topo, solid).unwrap();
        assert!(n > 0);

        for fid in solid_faces(&topo, solid).unwrap() {
            assert!(
                matches!(topo.face(fid).unwrap().surface(), FaceSurface::Nurbs(_)),
                "face {fid:?} should be NURBS after convert_to_bspline"
            );
        }
        for eid in solid_edges(&topo, solid).unwrap() {
            assert!(
                matches!(topo.edge(eid).unwrap().curve(), EdgeCurve::NurbsCurve(_)),
                "edge {eid:?} should be NURBS after convert_to_bspline"
            );
        }
    }

    #[test]
    fn idempotent_on_already_nurbs() {
        let mut topo = Topology::default();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);

        let first = convert_solid_to_bspline(&mut topo, solid).unwrap();
        assert!(first > 0);
        let second = convert_solid_to_bspline(&mut topo, solid).unwrap();
        assert_eq!(second, 0);
    }

    #[test]
    fn cylinder_face_converts_with_axial_range() {
        let cyl = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), z_axis(), 2.0).unwrap();
        let mut topo = Topology::default();
        let ring = [
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 5.0),
            Point3::new(0.0, 2.0, 5.0),
        ];
        let solid = single_face_solid(&mut topo, FaceSurface::Cylinder(cyl), &ring);

        convert_solid_to_bspline(&mut topo, solid).unwrap();

        let fid = solid_faces(&topo, solid).unwrap()[0];
        let surf = topo.face(fid).unwrap().surface().clone();
        let nurbs = match surf {
            FaceSurface::Nurbs(n) => n,
            other => panic!("expected NURBS, got {:?}", other.type_tag()),
        };

        // Sample the NURBS and verify points lie at distance 2 from the z-axis
        // and within the v-range derived from the wire (0..5).
        let (u_min, u_max) = nurbs.domain_u();
        let (v_min, v_max) = nurbs.domain_v();
        for i in 0..=8 {
            for j in 0..=4 {
                let u = u_min + (u_max - u_min) * f64::from(i) / 8.0;
                let v = v_min + (v_max - v_min) * f64::from(j) / 4.0;
                let p = nurbs.evaluate(u, v);
                let r = (p.x() * p.x() + p.y() * p.y()).sqrt();
                assert!((r - 2.0).abs() < 1e-6, "u={u}, v={v}: r={r}");
                assert!(
                    p.z() >= -1e-9 && p.z() <= 5.0 + 1e-9,
                    "z out of range: {}",
                    p.z()
                );
            }
        }
    }

    #[test]
    fn sphere_face_converts() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0).unwrap();
        let mut topo = Topology::default();
        let ring = [
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(-3.0, 0.0, 0.0),
        ];
        let solid = single_face_solid(&mut topo, FaceSurface::Sphere(sphere), &ring);

        convert_solid_to_bspline(&mut topo, solid).unwrap();
        let fid = solid_faces(&topo, solid).unwrap()[0];
        assert!(matches!(
            topo.face(fid).unwrap().surface(),
            FaceSurface::Nurbs(_)
        ));
    }

    #[test]
    fn cone_face_converts_with_clamped_apex() {
        let cone = ConicalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            z_axis(),
            std::f64::consts::FRAC_PI_4,
        )
        .unwrap();
        let mut topo = Topology::default();
        let ring = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 2.0),
            Point3::new(0.0, 2.0, 2.0),
        ];
        let solid = single_face_solid(&mut topo, FaceSurface::Cone(cone), &ring);

        convert_solid_to_bspline(&mut topo, solid).unwrap();
        let fid = solid_faces(&topo, solid).unwrap()[0];
        assert!(matches!(
            topo.face(fid).unwrap().surface(),
            FaceSurface::Nurbs(_)
        ));
    }

    #[test]
    fn torus_face_converts() {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0, 1.0).unwrap();
        let mut topo = Topology::default();
        let ring = [
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(-5.0, 0.0, 0.0),
        ];
        let solid = single_face_solid(&mut topo, FaceSurface::Torus(torus), &ring);

        convert_solid_to_bspline(&mut topo, solid).unwrap();
        let fid = solid_faces(&topo, solid).unwrap()[0];
        assert!(matches!(
            topo.face(fid).unwrap().surface(),
            FaceSurface::Nurbs(_)
        ));
    }

    #[test]
    fn closed_circle_edge_becomes_full_nurbs() {
        let mut topo = Topology::default();
        let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), z_axis(), 1.0).unwrap();
        let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let eid = topo.add_edge(edge);

        // Plug the closed edge into a one-edge wire on a planar face so the
        // solid traversal sees it.
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: z_axis(),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        convert_solid_to_bspline(&mut topo, solid).unwrap();

        let nurbs = match topo.edge(eid).unwrap().curve().clone() {
            EdgeCurve::NurbsCurve(n) => n,
            other => panic!("expected NurbsCurve, got {}", other.type_tag()),
        };
        // Sample the closed NURBS and ensure points lie on the circle.
        for i in 0..16 {
            let t = ParametricCurve::domain(&nurbs).0
                + (ParametricCurve::domain(&nurbs).1 - ParametricCurve::domain(&nurbs).0)
                    * f64::from(i)
                    / 16.0;
            let p = nurbs.evaluate(t);
            let r = (p.x() * p.x() + p.y() * p.y()).sqrt();
            assert!(
                (r - 1.0).abs() < 1e-6,
                "circle radius drift at t={t}: r={r}"
            );
            assert!(
                p.z().abs() < 1e-9,
                "circle out-of-plane at t={t}: z={}",
                p.z()
            );
        }
        assert_eq!(
            topo.edge(eid).unwrap().strict_domain().unwrap(),
            nurbs.domain()
        );
    }

    #[test]
    fn anchored_closed_ellipse_preserves_seam_and_full_turn() {
        let mut topo = Topology::default();
        let ellipse = Ellipse3D::new(Point3::new(3.0, -2.0, 0.0), z_axis(), 5.0, 2.0).unwrap();
        let source_start = 2.8;
        let source_end = source_start + TAU;
        let seam = ellipse.evaluate(source_start);
        let vertex = topo.add_vertex(Vertex::new(seam, 1e-7));
        let mut edge = Edge::with_tolerance(
            vertex,
            vertex,
            EdgeCurve::Ellipse(ellipse.clone()),
            Some(1e-7),
        );
        edge.set_trim(Some((source_start, source_end)));
        let edge = topo.add_edge(edge);
        let (solid, _) = single_edge_solid(&mut topo, edge, true);

        convert_solid_to_bspline(&mut topo, solid).unwrap();

        let curve = assert_converted_endpoints(&topo, edge, seam, seam, 1e-7);
        let domain = curve.domain();
        let midpoint = curve.evaluate(domain.0.midpoint(domain.1));
        assert_ellipse_implicit(&ellipse, midpoint, 1e-12);
        assert!((midpoint - ellipse.evaluate(source_start + PI)).length() < 1e-10);
    }

    #[test]
    fn wrapped_open_ellipse_preserves_the_declared_arc() {
        let mut topo = Topology::default();
        let ellipse = Ellipse3D::new(Point3::new(-1.0, 4.0, 0.0), z_axis(), 4.0, 1.5).unwrap();
        let source_start = 5.5;
        let source_end = TAU + 0.7;
        let start = ellipse.evaluate(source_start);
        let end = ellipse.evaluate(source_end);
        let start_vertex = topo.add_vertex(Vertex::new(start, 1e-7));
        let end_vertex = topo.add_vertex(Vertex::new(end, 1e-7));
        let mut edge = Edge::new(
            start_vertex,
            end_vertex,
            EdgeCurve::Ellipse(ellipse.clone()),
        );
        edge.set_trim(Some((source_start, source_end)));
        let edge = topo.add_edge(edge);
        let (solid, _) = single_edge_solid(&mut topo, edge, false);

        convert_solid_to_bspline(&mut topo, solid).unwrap();

        let curve = assert_converted_endpoints(&topo, edge, start, end, 1e-10);
        let domain = curve.domain();
        let midpoint = curve.evaluate(domain.0.midpoint(domain.1));
        assert_ellipse_implicit(&ellipse, midpoint, 1e-12);
        assert!((midpoint - ellipse.evaluate(source_start.midpoint(source_end))).length() < 1e-10);
    }

    #[test]
    fn reversed_unbounded_conics_preserve_vertices_direction_and_implicit_curve() {
        let mut topo = Topology::default();
        let hyperbola = Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), z_axis(), 3.0, 2.0).unwrap();
        let hyperbola_range = (1.2, -0.8);
        let hyperbola_start = hyperbola.evaluate(hyperbola_range.0);
        let hyperbola_end = hyperbola.evaluate(hyperbola_range.1);
        let h_start = topo.add_vertex(Vertex::new(hyperbola_start, 1e-7));
        let h_end = topo.add_vertex(Vertex::new(hyperbola_end, 1e-7));
        let mut hyperbola_edge = Edge::new(h_start, h_end, EdgeCurve::Hyperbola(hyperbola.clone()));
        hyperbola_edge.set_trim(Some(hyperbola_range));
        let hyperbola_edge = topo.add_edge(hyperbola_edge);
        let (hyperbola_solid, _) = single_edge_solid(&mut topo, hyperbola_edge, false);

        convert_solid_to_bspline(&mut topo, hyperbola_solid).unwrap();

        let converted = assert_converted_endpoints(
            &topo,
            hyperbola_edge,
            hyperbola_start,
            hyperbola_end,
            1e-10,
        );
        let domain = converted.domain();
        assert_hyperbola_implicit(
            &hyperbola,
            converted.evaluate(domain.0.midpoint(domain.1)),
            1e-12,
        );

        let parabola = Parabola3D::new(Point3::new(0.0, 0.0, 0.0), z_axis(), 2.0).unwrap();
        let parabola_range = (2.0, -1.0);
        let parabola_start = parabola.evaluate(parabola_range.0);
        let parabola_end = parabola.evaluate(parabola_range.1);
        let p_start = topo.add_vertex(Vertex::new(parabola_start, 1e-7));
        let p_end = topo.add_vertex(Vertex::new(parabola_end, 1e-7));
        let mut parabola_edge = Edge::new(p_start, p_end, EdgeCurve::Parabola(parabola.clone()));
        parabola_edge.set_trim(Some(parabola_range));
        let parabola_edge = topo.add_edge(parabola_edge);
        let (parabola_solid, _) = single_edge_solid(&mut topo, parabola_edge, false);

        convert_solid_to_bspline(&mut topo, parabola_solid).unwrap();

        let converted =
            assert_converted_endpoints(&topo, parabola_edge, parabola_start, parabola_end, 1e-10);
        let domain = converted.domain();
        assert_parabola_implicit(
            &parabola,
            converted.evaluate(domain.0.midpoint(domain.1)),
            1e-12,
        );
    }

    #[test]
    fn ill_conditioned_large_hyperbola_refuses_before_mutation() {
        let mut topo = Topology::default();
        let hyperbola = Hyperbola3D::new(Point3::new(0.0, 0.0, 0.0), z_axis(), 1e10, 3e9).unwrap();
        let source_domain = (-20.0, 60.0);
        let start = hyperbola.evaluate(source_domain.0);
        let end = hyperbola.evaluate(source_domain.1);
        let start_vertex = topo.add_vertex(Vertex::new(start, 1e-7));
        let end_vertex = topo.add_vertex(Vertex::new(end, 1e-7));
        let mut source_edge = Edge::with_tolerance(
            start_vertex,
            end_vertex,
            EdgeCurve::Hyperbola(hyperbola),
            Some(1e-7),
        );
        source_edge.set_trim(Some(source_domain));
        let edge = topo.add_edge(source_edge);
        let (solid, face) = single_edge_solid(&mut topo, edge, false);
        let counts_before = (
            topo.vertices().len(),
            topo.edges().len(),
            topo.wires().len(),
            topo.faces().len(),
            topo.shells().len(),
            topo.solids().len(),
        );

        let error = convert_solid_to_bspline(&mut topo, solid).unwrap_err();

        assert!(matches!(error, HealError::AnalysisFailed(_)));
        assert!(error.to_string().contains("conversion oracle residual"));
        assert_eq!(
            counts_before,
            (
                topo.vertices().len(),
                topo.edges().len(),
                topo.wires().len(),
                topo.faces().len(),
                topo.shells().len(),
                topo.solids().len(),
            )
        );
        let unchanged = topo.edge(edge).unwrap();
        assert!(matches!(unchanged.curve(), EdgeCurve::Hyperbola(_)));
        assert_eq!(unchanged.trim(), Some(source_domain));
        assert!(matches!(
            topo.face(face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ));
    }

    #[test]
    fn mismatched_source_domain_refuses_before_conversion_mutation() {
        let mut topo = Topology::default();
        let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), z_axis(), 3.0).unwrap();
        let domain = (0.25, 1.25);
        let start = topo.add_vertex(Vertex::new(circle.evaluate(domain.0 + 0.5), 1e-7));
        let end = topo.add_vertex(Vertex::new(circle.evaluate(domain.1 + 0.5), 1e-7));
        let mut source = Edge::new(start, end, EdgeCurve::Circle(circle));
        source.set_trim(Some(domain));
        let edge = topo.add_edge(source);
        let (solid, face) = single_edge_solid(&mut topo, edge, false);
        let counts_before = (
            topo.vertices().len(),
            topo.edges().len(),
            topo.wires().len(),
            topo.faces().len(),
            topo.shells().len(),
            topo.solids().len(),
        );

        let error = convert_solid_to_bspline(&mut topo, solid).unwrap_err();

        assert!(matches!(error, HealError::AnalysisFailed(_)));
        assert!(error.to_string().contains("source start residual"));
        assert_eq!(
            counts_before,
            (
                topo.vertices().len(),
                topo.edges().len(),
                topo.wires().len(),
                topo.faces().len(),
                topo.shells().len(),
                topo.solids().len(),
            )
        );
        let unchanged = topo.edge(edge).unwrap();
        assert!(matches!(unchanged.curve(), EdgeCurve::Circle(_)));
        assert_eq!(unchanged.trim(), Some(domain));
        assert!(matches!(
            topo.face(face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ));
    }

    #[test]
    fn invalid_individual_tolerances_refuse_before_conversion_mutation() {
        for (start_tolerance, edge_tolerance, expected) in [
            (f64::NAN, Some(1e-7), "start vertex"),
            (1e-7, Some(-1.0), "explicit"),
        ] {
            let mut topo = Topology::default();
            let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), start_tolerance));
            let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
            // `with_tolerance` stores the claim unchecked; `set_tolerance`
            // refuses invalid values (RFC 0004), and this test needs the
            // invalid value stored to exercise the downstream refusal.
            let source = Edge::with_tolerance(start, end, EdgeCurve::Line, edge_tolerance);
            let edge = topo.add_edge(source);
            let (solid, face) = single_edge_solid(&mut topo, edge, false);
            let counts_before = (
                topo.vertices().len(),
                topo.edges().len(),
                topo.wires().len(),
                topo.faces().len(),
                topo.shells().len(),
                topo.solids().len(),
            );

            let error = convert_solid_to_bspline(&mut topo, solid).unwrap_err();

            assert!(
                matches!(error, HealError::AnalysisFailed(ref message) if message.contains(expected)),
                "unexpected error: {error}"
            );
            assert_eq!(
                counts_before,
                (
                    topo.vertices().len(),
                    topo.edges().len(),
                    topo.wires().len(),
                    topo.faces().len(),
                    topo.shells().len(),
                    topo.solids().len(),
                )
            );
            assert!(matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line));
            assert!(matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            ));
        }
    }

    #[test]
    fn between_sample_nurbs_bulge_stays_inside_converted_plane_patch() {
        const DEGREE: usize = 64;
        const DEGREE_F64: f64 = 64.0;
        let mut control_points = (0..=DEGREE)
            .map(|index| {
                #[allow(clippy::cast_precision_loss)]
                let x = index as f64 / DEGREE_F64;
                Point3::new(x, 0.0, 0.0)
            })
            .collect::<Vec<_>>();
        control_points[1] = Point3::new(1.0 / DEGREE_F64, 1_000.0, 0.0);
        let knots = [vec![0.0; DEGREE + 1], vec![1.0; DEGREE + 1]].concat();
        let curve = NurbsCurve::new(DEGREE, knots, control_points, vec![1.0; DEGREE + 1]).unwrap();
        let witness_parameter = 1.0 / DEGREE_F64;
        let witness = curve.evaluate(witness_parameter);
        let old_sample_max = (0..=16)
            .map(|index| curve.evaluate(f64::from(index) / 16.0).y())
            .fold(0.0_f64, f64::max);
        assert!(
            witness.y() > 4.0 * old_sample_max,
            "fixture must bulge between the former 16 uniform samples"
        );

        let mut topo = Topology::default();
        let start = topo.add_vertex(Vertex::new(curve.evaluate(0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(curve.evaluate(1.0), 1e-7));
        let mut source_edge = Edge::new(start, end, EdgeCurve::NurbsCurve(curve));
        source_edge.set_trim(Some((0.0, 1.0)));
        let edge = topo.add_edge(source_edge);
        let (solid, face) = single_edge_solid(&mut topo, edge, false);

        assert_eq!(convert_solid_to_bspline(&mut topo, solid).unwrap(), 1);

        let surface = match topo.face(face).unwrap().surface() {
            FaceSurface::Nurbs(surface) => surface,
            other => panic!("expected NURBS plane, got {}", other.type_tag()),
        };
        // For a +Z plane, plane_frame_axes maps (u, v) to (+Y, -X).
        let uv = (witness.y(), -witness.x());
        let (u_min, u_max) = surface.domain_u();
        let (v_min, v_max) = surface.domain_v();
        assert!(uv.0 >= u_min && uv.0 <= u_max);
        assert!(uv.1 >= v_min && uv.1 <= v_max);
        let on_surface = surface.evaluate(uv.0, uv.1);
        assert!(
            (on_surface - witness).length() <= 1e-9,
            "certified patch domain must contain the between-sample boundary witness"
        );
    }

    #[test]
    fn missing_curved_authority_refuses_before_any_face_or_edge_mutation() {
        let mut topo = Topology::default();
        let first_solid = single_face_solid(
            &mut topo,
            FaceSurface::Plane {
                normal: z_axis(),
                d: 0.0,
            },
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
        let first_face = solid_faces(&topo, first_solid).unwrap()[0];

        let circle = Circle3D::new(Point3::new(3.0, 0.0, 0.0), z_axis(), 1.0).unwrap();
        let seam = circle.evaluate(0.0);
        let vertex = topo.add_vertex(Vertex::new(seam, 1e-7));
        let missing_edge = topo.add_edge(Edge::new(vertex, vertex, EdgeCurve::Circle(circle)));
        let (_, second_face) = single_edge_solid(&mut topo, missing_edge, true);
        let shell = topo.add_shell(Shell::new(vec![first_face, second_face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let counts_before = (
            topo.vertices().len(),
            topo.edges().len(),
            topo.wires().len(),
            topo.faces().len(),
            topo.shells().len(),
            topo.solids().len(),
        );

        let error = convert_solid_to_bspline(&mut topo, solid).unwrap_err();

        assert!(matches!(error, HealError::AnalysisFailed(_)));
        assert_eq!(
            counts_before,
            (
                topo.vertices().len(),
                topo.edges().len(),
                topo.wires().len(),
                topo.faces().len(),
                topo.shells().len(),
                topo.solids().len(),
            )
        );
        assert!(matches!(
            topo.face(first_face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ));
        assert!(matches!(
            topo.face(second_face).unwrap().surface(),
            FaceSurface::Plane { .. }
        ));
        let edge = topo.edge(missing_edge).unwrap();
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
        assert!(edge.trim().is_none());
    }

    #[test]
    fn two_branch_seam_pcurves_are_completely_removed_after_preflight() {
        let mut topo = Topology::default();
        let start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let seam = topo.add_edge(Edge::new(start, end, EdgeCurve::Line));
        let (solid, face) = single_edge_solid(&mut topo, seam, false);
        let branch = |u: f64, direction: f64| {
            PCurve::new(
                Curve2D::Line(Line2D::new(Point2::new(u, 0.0), Vec2::new(0.0, direction)).unwrap()),
                0.0,
                1.0,
            )
        };
        topo.set_pcurve_oriented(seam, face, true, branch(0.0, 1.0));
        topo.set_pcurve_oriented(seam, face, false, branch(TAU, -1.0));
        assert_eq!(topo.pcurves_for_face(face).len(), 2);

        assert_eq!(convert_solid_to_bspline(&mut topo, solid).unwrap(), 2);

        assert!(matches!(
            topo.face(face).unwrap().surface(),
            FaceSurface::Nurbs(_)
        ));
        assert!(matches!(
            topo.edge(seam).unwrap().curve(),
            EdgeCurve::NurbsCurve(_)
        ));
        assert!(topo.pcurve_oriented(seam, face, true).is_none());
        assert!(topo.pcurve_oriented(seam, face, false).is_none());
        assert!(topo.pcurves_for_face(face).is_empty());
    }

    #[test]
    fn near_degenerate_line_edge_is_skipped_not_errored() {
        // Edge with length below topology tolerance must skip cleanly, not
        // bubble a GeomError that aborts the whole solid conversion.
        let mut topo = Topology::default();
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(1e-10, 0.0, 0.0);
        let v0 = topo.add_vertex(Vertex::new(p0, 1e-7));
        let v1 = topo.add_vertex(Vertex::new(p1, 1e-7));
        let degenerate_eid = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        // Embed in a face so solid_edges traversal sees it.
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(degenerate_eid, true),
                    OrientedEdge::new(degenerate_eid, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: z_axis(),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        // Should succeed without converting the degenerate edge.
        convert_solid_to_bspline(&mut topo, solid).unwrap();
        assert!(matches!(
            topo.edge(degenerate_eid).unwrap().curve(),
            EdgeCurve::Line
        ));
    }

    #[test]
    fn line_to_nurbs_preserves_endpoints() {
        let mut topo = Topology::default();
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(3.0, 4.0, 0.0);
        let v0 = topo.add_vertex(Vertex::new(p0, 1e-7));
        let v1 = topo.add_vertex(Vertex::new(p1, 1e-7));
        let eid = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        // Embed in a (degenerate, unbounded) face so solid_edges finds it.
        let wire = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(eid, true), OrientedEdge::new(eid, false)],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: z_axis(),
                d: 0.0,
            },
        ));
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        convert_solid_to_bspline(&mut topo, solid).unwrap();
        let curve = topo.edge(eid).unwrap().curve().clone();
        let nurbs: NurbsCurve = match curve {
            EdgeCurve::NurbsCurve(n) => n,
            other => panic!("expected NurbsCurve, got {}", other.type_tag()),
        };
        let (t0, t1) = ParametricCurve::domain(&nurbs);
        let q0 = nurbs.evaluate(t0);
        let q1 = nurbs.evaluate(t1);
        assert!((q0 - p0).length() < 1e-12);
        assert!((q1 - p1).length() < 1e-12);
    }

    #[test]
    fn x_axis_plane_picks_safe_uv_frame() {
        // Normal along +x triggers the alternate seed in plane_frame_axes.
        let mut topo = Topology::default();
        let normal = x_axis();
        let ring = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let solid = single_face_solid(&mut topo, FaceSurface::Plane { normal, d: 0.0 }, &ring);
        convert_solid_to_bspline(&mut topo, solid).unwrap();
        let fid = solid_faces(&topo, solid).unwrap()[0];
        assert!(matches!(
            topo.face(fid).unwrap().surface(),
            FaceSurface::Nurbs(_)
        ));
    }
}
