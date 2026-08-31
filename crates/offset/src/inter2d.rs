//! Phase 4: create new edges from face-face intersection curves.
//!
//! After Phase 3 computes intersection curves between adjacent offset faces,
//! this phase creates the corresponding topology: vertices at the intersection
//! line endpoints and edges connecting them.

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::solid::SolidId;

use crate::data::{OffsetData, VertexCache, find_or_create_vertex};
use crate::error::OffsetError;

/// Create new edges from the intersection curves computed in Phase 3.
///
/// For each `FaceIntersection` with non-empty `curve_points`, this function:
/// 1. Finds or creates vertices at the first and last curve points
///    (deduplicated by tolerance).
/// 2. Creates a certified `Circle` arc/full-turn edge when the samples permit
///    it, otherwise a `Line` edge between the endpoints.
/// 3. Stores the new edge ID in `FaceIntersection::new_edges`.
///
/// # Errors
///
/// Returns [`OffsetError`] if topology operations fail.
pub fn intersect_pcurves_2d(
    topo: &mut Topology,
    _solid: SolidId,
    data: &mut OffsetData,
) -> Result<(), OffsetError> {
    let tol = data.options.tolerance.linear;
    let mut vertex_cache = VertexCache::new(tol);

    for intersection in &mut data.intersections {
        if intersection.curve_points.len() < 2 {
            continue;
        }

        if let Some(edge_id) =
            create_edge_from_curve_points(topo, &mut vertex_cache, &intersection.curve_points, tol)?
        {
            intersection.new_edges.push(edge_id);
        }
    }

    Ok(())
}

/// Create a topological edge from sampled intersection curve points.
///
/// If the points form a circle, creates a `Circle` edge carrying the signed,
/// unwrapped range described by the sample sequence. Endpoint-coincident
/// chains and uniform one-turn chains with an omitted duplicate seam sample
/// become closed full turns. Otherwise creates a `Line` edge between the first
/// and last points.
///
/// Returns `None` if the edge would be degenerate.
///
/// # Errors
///
/// Returns [`OffsetError::InvalidInput`] for an invalid tolerance, or
/// [`OffsetError::AssemblyFailed`] when fitted circle authority cannot be
/// certified before allocation.
fn create_edge_from_curve_points(
    topo: &mut Topology,
    vertex_cache: &mut VertexCache,
    points: &[Point3],
    tol: f64,
) -> Result<Option<EdgeId>, OffsetError> {
    if !tol.is_finite() || tol < 0.0 {
        return Err(OffsetError::InvalidInput {
            reason: format!(
                "intersection edge tolerance must be finite and non-negative, got {tol}"
            ),
        });
    }
    if points.len() < 2 {
        return Ok(None);
    }

    if points.len() >= 8
        && let Some(circle) = fit_circle_3d(points, tol)
    {
        let authority = certify_circle_authority(&circle, points, tol)?;
        let start = find_or_create_vertex(topo, vertex_cache, authority.start_point, tol);
        let end = if authority.closed {
            start
        } else {
            let end = find_or_create_vertex(topo, vertex_cache, authority.end_point, tol);
            if end == start {
                return Err(OffsetError::AssemblyFailed {
                    reason: "open fitted circle endpoints collapsed to one cached vertex".into(),
                });
            }
            end
        };
        let mut edge = Edge::with_tolerance(start, end, EdgeCurve::Circle(circle), Some(tol));
        edge.set_trim(Some(authority.range));
        edge.strict_domain()
            .map_err(|error| OffsetError::AssemblyFailed {
                reason: format!("fitted circle has invalid parameter authority: {error}"),
            })?;
        return Ok(Some(topo.add_edge(edge)));
    }

    let p_start = points[0];
    let p_end = points[points.len() - 1];
    let v_start = find_or_create_vertex(topo, vertex_cache, p_start, tol);
    let v_end = find_or_create_vertex(topo, vertex_cache, p_end, tol);
    if v_start == v_end {
        return Ok(None);
    }
    Ok(Some(topo.add_edge(Edge::new(
        v_start,
        v_end,
        EdgeCurve::Line,
    ))))
}

struct CircleAuthority {
    range: (f64, f64),
    start_point: Point3,
    end_point: Point3,
    closed: bool,
}

#[allow(clippy::too_many_lines)]
fn certify_circle_authority(
    circle: &remus_math::curves::Circle3D,
    points: &[Point3],
    tol: f64,
) -> Result<CircleAuthority, OffsetError> {
    let projected: Vec<_> = points
        .iter()
        .map(|point| {
            let parameter = circle.project(*point);
            (parameter, circle.evaluate(parameter))
        })
        .collect();
    if projected.iter().any(|(parameter, point)| {
        !parameter.is_finite() || point.0.iter().any(|coordinate| !coordinate.is_finite())
    }) {
        return Err(OffsetError::AssemblyFailed {
            reason: "fitted circle samples produced non-finite projected parameters".into(),
        });
    }

    let raw_endpoint_distance = (points[points.len() - 1] - points[0]).length();
    let projected_endpoint_distance = (projected[projected.len() - 1].1 - projected[0].1).length();
    if !raw_endpoint_distance.is_finite() || !projected_endpoint_distance.is_finite() {
        return Err(OffsetError::AssemblyFailed {
            reason: "fitted circle endpoint closure is not finite".into(),
        });
    }
    let raw_closed = raw_endpoint_distance <= tol;
    let projected_closed = projected_endpoint_distance <= tol;
    if raw_closed != projected_closed {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "fitted circle endpoint closure is ambiguous (raw {raw_endpoint_distance}, \
                 projected {projected_endpoint_distance}, tolerance {tol})"
            ),
        });
    }
    if !raw_closed && projected_endpoint_distance <= 2.0 * tol {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "open fitted circle endpoints cannot be certified as distinct for vertex \
                 caching (distance {projected_endpoint_distance}, tolerance {tol})"
            ),
        });
    }

    let angular_tolerance =
        2.0 * (tol / (2.0 * circle.radius())).clamp(0.0, 1.0).asin() + 64.0 * f64::EPSILON;
    let sequence_roundoff = 64.0
        * f64::EPSILON
        * std::f64::consts::TAU
        * u32::try_from(points.len()).map_or_else(|_| f64::from(u32::MAX), f64::from);
    let mut positive_direction = None;
    let mut span = 0.0;
    let mut minimum_step = f64::INFINITY;
    let mut maximum_step = 0.0_f64;
    for pair in projected.windows(2) {
        let raw_delta = (pair[1].0 - pair[0].0).rem_euclid(std::f64::consts::TAU);
        let delta = if raw_delta > std::f64::consts::PI {
            raw_delta - std::f64::consts::TAU
        } else {
            raw_delta
        };
        if !delta.is_finite()
            || delta.abs() <= angular_tolerance
            || (delta.abs() - std::f64::consts::PI).abs() <= angular_tolerance
        {
            return Err(OffsetError::AssemblyFailed {
                reason: format!(
                    "fitted circle sample sequence has an ambiguous angular step {delta}"
                ),
            });
        }
        let step_is_positive = delta.is_sign_positive();
        if positive_direction.is_some_and(|expected| expected != step_is_positive) {
            return Err(OffsetError::AssemblyFailed {
                reason: "fitted circle sample sequence reverses direction".into(),
            });
        }
        positive_direction = Some(step_is_positive);
        minimum_step = minimum_step.min(delta.abs());
        maximum_step = maximum_step.max(delta.abs());
        span += delta;
    }
    let positive_direction = positive_direction.ok_or_else(|| OffsetError::AssemblyFailed {
        reason: "fitted circle has no certifiable sample direction".into(),
    })?;
    if !span.is_finite() {
        return Err(OffsetError::AssemblyFailed {
            reason: "fitted circle sample span is not finite".into(),
        });
    }

    let closure_raw =
        (projected[0].0 - projected[projected.len() - 1].0).rem_euclid(std::f64::consts::TAU);
    let closure_delta = if closure_raw > std::f64::consts::PI {
        closure_raw - std::f64::consts::TAU
    } else {
        closure_raw
    };
    let step_tolerance = angular_tolerance + sequence_roundoff;
    let expected_step = span.abs() / (projected.len() - 1) as f64;
    let implicit_full_turn = !raw_closed
        && closure_delta.is_finite()
        && closure_delta.abs() > angular_tolerance
        && closure_delta.is_sign_positive() == positive_direction
        && maximum_step - minimum_step <= step_tolerance
        && (closure_delta.abs() - expected_step).abs() <= step_tolerance
        && ((span + closure_delta).abs() - std::f64::consts::TAU).abs() <= step_tolerance;
    let closed = raw_closed || implicit_full_turn;
    let start_parameter = projected[0].0;
    let end_parameter = if closed {
        let certified_turn_span = if raw_closed {
            span
        } else {
            span + closure_delta
        };
        if (certified_turn_span.abs() - std::f64::consts::TAU).abs() > step_tolerance {
            return Err(OffsetError::AssemblyFailed {
                reason: format!(
                    "closed fitted circle samples do not describe exactly one turn (span \
                     {certified_turn_span})"
                ),
            });
        }
        if positive_direction {
            start_parameter + std::f64::consts::TAU
        } else {
            start_parameter - std::f64::consts::TAU
        }
    } else {
        if span.abs() > std::f64::consts::TAU {
            return Err(OffsetError::AssemblyFailed {
                reason: format!("open fitted circle samples exceed one turn (span {span})"),
            });
        }
        start_parameter + span
    };
    if !start_parameter.is_finite()
        || !end_parameter.is_finite()
        || end_parameter.partial_cmp(&start_parameter) == Some(std::cmp::Ordering::Equal)
    {
        return Err(OffsetError::AssemblyFailed {
            reason: "fitted circle produced invalid parameter authority".into(),
        });
    }

    let start_point = projected[0].1;
    let end_point = if closed {
        start_point
    } else {
        projected[projected.len() - 1].1
    };
    for (label, parameter, expected) in [
        ("start", start_parameter, start_point),
        ("end", end_parameter, end_point),
    ] {
        let residual = (circle.evaluate(parameter) - expected).length();
        if !residual.is_finite() || residual > tol {
            return Err(OffsetError::AssemblyFailed {
                reason: format!(
                    "fitted circle {label} does not certify its sampled branch: residual \
                     {residual}, tolerance {tol}"
                ),
            });
        }
    }
    let midpoint_parameter = (end_parameter - start_parameter).mul_add(0.5, start_parameter);
    let midpoint = circle.evaluate(midpoint_parameter);
    if !midpoint_parameter.is_finite()
        || midpoint.0.iter().any(|coordinate| !coordinate.is_finite())
    {
        return Err(OffsetError::AssemblyFailed {
            reason: "fitted circle branch midpoint is not finite".into(),
        });
    }

    Ok(CircleAuthority {
        range: (start_parameter, end_parameter),
        start_point,
        end_point,
        closed,
    })
}

/// Fit a `Circle3D` to sampled points if they lie on a circle within tolerance.
///
/// Uses a 3-point circumcircle from well-spaced samples (marching samples
/// are non-uniform, so centroid ≠ center). Validates against all points.
///
/// Returns `None` if points don't form a circle.
#[allow(clippy::too_many_lines)]
fn fit_circle_3d(points: &[Point3], tol: f64) -> Option<remus_math::curves::Circle3D> {
    let n = points.len();
    if n < 8
        || tol < 0.0
        || points
            .iter()
            .any(|point| point.0.iter().any(|coordinate| !coordinate.is_finite()))
    {
        return None;
    }

    let p0 = points[0];
    let p1 = points[n / 3];
    let p2 = points[2 * n / 3];

    let d1 = Vec3::new(p1.x() - p0.x(), p1.y() - p0.y(), p1.z() - p0.z());
    let d2 = Vec3::new(p2.x() - p0.x(), p2.y() - p0.y(), p2.z() - p0.z());
    let normal = d1.cross(d2);
    let normal_len = normal.length();
    if normal_len < 1e-15 {
        return None; // Collinear
    }
    let normal = Vec3::new(
        normal.x() / normal_len,
        normal.y() / normal_len,
        normal.z() / normal_len,
    );
    let components = [normal.x().abs(), normal.y().abs(), normal.z().abs()];
    let dominant = components
        .iter()
        .enumerate()
        .max_by(|(_, first), (_, second)| first.total_cmp(second))?
        .0;
    let normal = if normal.0[dominant].is_sign_negative() {
        normal * -1.0
    } else {
        normal
    };

    let u_axis = {
        let len = d1.length();
        if len < 1e-15 {
            return None;
        }
        Vec3::new(d1.x() / len, d1.y() / len, d1.z() / len)
    };
    let v_axis = normal.cross(u_axis);

    let proj = |p: Point3| -> (f64, f64) {
        let dx = p.x() - p0.x();
        let dy = p.y() - p0.y();
        let dz = p.z() - p0.z();
        let v = Vec3::new(dx, dy, dz);
        (v.dot(u_axis), v.dot(v_axis))
    };
    let (ax, ay) = (0.0, 0.0); // p0 in local coords
    let (bx, by) = proj(p1);
    let (cx_l, cy_l) = proj(p2);

    // Circumcenter in 2D: solve perpendicular bisector intersection.
    let d_val = 2.0 * (ax * (by - cy_l) + bx * (cy_l - ay) + cx_l * (ay - by));
    if d_val.abs() < 1e-15 {
        return None;
    }
    let ax2 = ax.mul_add(ax, ay * ay);
    let bx2 = bx.mul_add(bx, by * by);
    let cx2 = cx_l.mul_add(cx_l, cy_l * cy_l);
    let ux = (ax2 * (by - cy_l) + bx2 * (cy_l - ay) + cx2 * (ay - by)) / d_val;
    let uy = (ax2 * (cx_l - bx) + bx2 * (ax - cx_l) + cx2 * (bx - ax)) / d_val;

    let radius = ((ax - ux).powi(2) + (ay - uy).powi(2)).sqrt();
    if radius < tol {
        return None;
    }

    let center = Point3::new(
        p0.x() + ux * u_axis.x() + uy * v_axis.x(),
        p0.y() + ux * u_axis.y() + uy * v_axis.y(),
        p0.z() + ux * u_axis.z() + uy * v_axis.z(),
    );

    let circle = remus_math::curves::Circle3D::new(center, normal, radius).ok()?;
    if points.iter().any(|point| {
        let parameter = circle.project(*point);
        let residual = (circle.evaluate(parameter) - *point).length();
        !parameter.is_finite() || !residual.is_finite() || residual > tol
    }) {
        return None;
    }

    Some(circle)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use crate::data::{OffsetData, OffsetOptions};
    use remus_topology::Topology;
    use remus_topology::solid::SolidId;

    fn sampled_circle(start: f64, span: f64, segments: usize) -> Vec<Point3> {
        let radius = 2.5;
        (0..=segments)
            .map(|index| {
                let fraction = index as f64 / segments as f64;
                let parameter = span.mul_add(fraction, start);
                Point3::new(radius * parameter.cos(), radius * parameter.sin(), 5.0)
            })
            .collect()
    }

    fn assert_sampled_circle_authority(
        topo: &Topology,
        edge_id: EdgeId,
        points: &[Point3],
        expected_span: f64,
        closed: bool,
        tol: f64,
    ) {
        let edge = topo.edge(edge_id).unwrap();
        assert_eq!(edge.start() == edge.end(), closed);
        let range = edge.strict_domain().expect("certified circle authority");
        let span = range.1 - range.0;
        assert!(
            (span - expected_span).abs() < 1e-12,
            "stored span {span} != expected {expected_span}"
        );
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        assert!((start - points[0]).length() <= tol);
        let sequence_repeats_seam = (points[points.len() - 1] - points[0]).length() <= tol;
        let expected_end = if closed && !sequence_repeats_seam {
            points[0]
        } else {
            points[points.len() - 1]
        };
        assert!((end - expected_end).length() <= tol);

        let denominator = if closed && !sequence_repeats_seam {
            points.len() as f64
        } else {
            (points.len() - 1) as f64
        };
        for (index, expected) in points.iter().enumerate() {
            let parameter = span.mul_add(index as f64 / denominator, range.0);
            let actual = edge.curve().evaluate_with_endpoints(parameter, start, end);
            assert!(
                (actual - *expected).length() <= tol,
                "sample {index} left its certified sequence"
            );
        }
        let midpoint_parameter = span.mul_add(0.5, range.0);
        let midpoint = edge
            .curve()
            .evaluate_with_endpoints(midpoint_parameter, start, end);
        assert!((midpoint - points[points.len() / 2]).length() <= tol);
    }

    fn run_phases_1_to_4(topo: &mut Topology, solid: SolidId, distance: f64) -> OffsetData {
        let mut data = OffsetData::new(distance, OffsetOptions::default(), vec![]);
        crate::analyse::analyse_edges(topo, solid, &mut data).unwrap();
        crate::offset::build_offset_faces(topo, solid, &mut data).unwrap();
        crate::inter3d::intersect_faces_3d(topo, solid, &mut data).unwrap();
        intersect_pcurves_2d(topo, solid, &mut data).unwrap();
        data
    }

    #[test]
    fn box_intersections_have_new_edges() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_4(&mut topo, solid, 0.5);
        for fi in &data.intersections {
            assert!(
                !fi.new_edges.is_empty(),
                "intersection for edge {:?} should have new edges",
                fi.original_edge
            );
        }
    }

    #[test]
    fn box_new_edges_are_valid() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_4(&mut topo, solid, 0.5);
        for fi in &data.intersections {
            for &eid in &fi.new_edges {
                let edge = topo.edge(eid).unwrap();
                let start = topo.vertex(edge.start()).unwrap().point();
                let end = topo.vertex(edge.end()).unwrap().point();
                let length = ((end.x() - start.x()).powi(2)
                    + (end.y() - start.y()).powi(2)
                    + (end.z() - start.z()).powi(2))
                .sqrt();
                assert!(
                    length > 1e-10,
                    "new edge should have non-zero length, got {length}"
                );
            }
        }
    }

    #[test]
    fn vertices_are_deduplicated_within_tolerance() {
        let mut topo = Topology::new();
        let tol = 1e-7;
        let mut cache = VertexCache::new(tol);
        let p = remus_math::vec::Point3::new(1.0, 2.0, 3.0);
        let v1 = find_or_create_vertex(&mut topo, &mut cache, p, tol);
        let p_near = remus_math::vec::Point3::new(1.0, 2.0, 3.0 + 1e-9);
        let v2 = find_or_create_vertex(&mut topo, &mut cache, p_near, tol);
        assert_eq!(v1, v2, "nearby points should reuse the same vertex");

        let p_far = remus_math::vec::Point3::new(1.0, 2.0, 4.0);
        let v3 = find_or_create_vertex(&mut topo, &mut cache, p_far, tol);
        assert_ne!(v1, v3, "distant points should get different vertices");
    }

    #[test]
    fn box_creates_12_edges() {
        let mut topo = Topology::new();
        let solid = remus_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let data = run_phases_1_to_4(&mut topo, solid, 0.5);
        let total_new_edges: usize = data.intersections.iter().map(|fi| fi.new_edges.len()).sum();
        assert_eq!(
            total_new_edges, 12,
            "box offset should create 12 new edges (one per original edge)"
        );
    }

    #[test]
    fn open_sampled_circle_chains_preserve_minor_major_and_reversed_ranges() {
        let tol = 1e-7;
        for (_label, span) in [
            ("minor", 1.2),
            ("major", 4.8),
            ("reversed minor", -1.7),
            ("reversed major", -4.8),
        ] {
            let mut topo = Topology::new();
            let mut cache = VertexCache::new(tol);
            let points = sampled_circle(0.35, span, 16);
            let edge_id = create_edge_from_curve_points(&mut topo, &mut cache, &points, tol)
                .expect("open circle construction")
                .expect("open circle edge");
            assert!(matches!(
                topo.edge(edge_id).unwrap().curve(),
                EdgeCurve::Circle(_)
            ));
            assert_sampled_circle_authority(&topo, edge_id, &points, span, false, tol);
        }
    }

    #[test]
    fn closed_sampled_circle_preserves_both_full_turn_directions() {
        let tol = 1e-7;
        for span in [std::f64::consts::TAU, -std::f64::consts::TAU] {
            let mut topo = Topology::new();
            let mut cache = VertexCache::new(tol);
            let points = sampled_circle(0.35, span, 32);
            let edge_id = create_edge_from_curve_points(&mut topo, &mut cache, &points, tol)
                .unwrap()
                .expect("closed circle edge");
            assert_sampled_circle_authority(&topo, edge_id, &points, span, true, tol);
        }
    }

    #[test]
    fn closed_sampled_circle_accepts_uniform_turn_without_duplicate_seam() {
        let tol = 1e-7;
        for span in [std::f64::consts::TAU, -std::f64::consts::TAU] {
            let mut topo = Topology::new();
            let mut cache = VertexCache::new(tol);
            let mut points = sampled_circle(0.35, span, 32);
            points.pop();
            let edge_id = create_edge_from_curve_points(&mut topo, &mut cache, &points, tol)
                .unwrap()
                .expect("closed circle edge");
            assert_sampled_circle_authority(&topo, edge_id, &points, span, true, tol);
        }
    }

    #[test]
    fn reversing_circle_sequence_refuses_before_allocation() {
        let tol = 1e-7;
        let radius = 2.5;
        let parameters = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.2, 1.8, 2.1];
        let points: Vec<_> = parameters
            .into_iter()
            .map(|parameter: f64| {
                Point3::new(radius * parameter.cos(), radius * parameter.sin(), 5.0)
            })
            .collect();
        let mut topo = Topology::new();
        let mut cache = VertexCache::new(tol);

        let error = create_edge_from_curve_points(&mut topo, &mut cache, &points, tol)
            .expect_err("a reversing sequence has no single signed arc authority");
        assert!(matches!(error, OffsetError::AssemblyFailed { .. }));
        assert_eq!(topo.vertices().len(), 0);
        assert_eq!(topo.edges().len(), 0);
    }

    #[test]
    fn spherical_but_nonplanar_samples_are_not_fitted_as_a_circle() {
        use std::f64::consts::TAU;

        let mut topo = Topology::new();
        let tol = 1e-7;
        let mut cache = VertexCache::new(tol);
        let radius = 2.5;
        let n = 32;
        let mut points: Vec<_> = (0..n)
            .map(|i| {
                let t = TAU * i as f64 / n as f64;
                Point3::new(radius * t.cos(), radius * t.sin(), 5.0)
            })
            .collect();
        let z = 0.1;
        let xy_radius = (radius * radius - z * z).sqrt();
        let witness_parameter = TAU * 5.0 / n as f64;
        points[5] = Point3::new(
            xy_radius * witness_parameter.cos(),
            xy_radius * witness_parameter.sin(),
            5.0 + z,
        );

        let edge_id = create_edge_from_curve_points(&mut topo, &mut cache, &points, tol)
            .unwrap()
            .expect("non-degenerate fallback edge");
        assert!(matches!(
            topo.edge(edge_id).unwrap().curve(),
            EdgeCurve::Line
        ));
    }

    #[test]
    fn invalid_tolerance_refuses_before_allocation() {
        let mut topo = Topology::new();
        let mut cache = VertexCache::new(1e-7);
        let points = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];

        let error = create_edge_from_curve_points(&mut topo, &mut cache, &points, f64::NAN)
            .expect_err("non-finite tolerance must refuse");
        assert!(matches!(error, OffsetError::InvalidInput { .. }));
        assert_eq!(topo.vertices().len(), 0);
        assert_eq!(topo.edges().len(), 0);
    }
}
