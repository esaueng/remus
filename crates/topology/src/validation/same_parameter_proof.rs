//! Narrow, fail-closed `SameParameter` certificates.
//!
//! These routines authorize only curve/surface families whose complete
//! definitions give a global residual bound.  Point evaluations below are
//! witnesses inside an analytic or derivative-control-net bound; they are
//! never treated as a sampling proof by themselves.

use remus_math::curves::Circle3D;
use remus_math::curves2d::{Circle2D, Line2D, NurbsCurve2D};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::SphericalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};

const STRICT_TOLERANCE: f64 = Tolerance::new().linear;
const MAX_SPHERE_INTERVALS: usize = 65_536;

#[derive(Clone, Copy, Debug)]
pub(super) struct ProofBound {
    pub(super) max_deviation: f64,
    pub(super) at_parameter: f64,
    pub(super) evaluations: usize,
}

#[derive(Clone, Copy, Debug)]
pub(super) enum ProofResult {
    Certified(ProofBound),
    NonFinite,
    Unavailable,
}

#[derive(Clone, Copy)]
pub(super) struct CurveUse {
    pub(super) p0: f64,
    pub(super) p1: f64,
    pub(super) edge_domain: (f64, f64),
    pub(super) forward: bool,
}

#[derive(Clone, Copy)]
pub(super) struct PlaneEndpoints {
    pub(super) uv0: Point2,
    pub(super) uv1: Point2,
    pub(super) oriented_start: Point3,
    pub(super) oriented_end: Point3,
    pub(super) p0: f64,
    pub(super) p1: f64,
}

#[derive(Clone, Copy)]
struct PlaneChart {
    origin: Point3,
    u_axis: Vec3,
    v_axis: Vec3,
}

impl PlaneChart {
    fn canonical(normal: Vec3, d: f64) -> Option<Self> {
        if !normal.0.iter().all(|value| value.is_finite()) || !d.is_finite() {
            return None;
        }
        let denominator = normal.dot(normal);
        if !denominator.is_finite() || denominator <= 0.0 {
            return None;
        }
        let origin = Point3::new(
            normal.x() * d / denominator,
            normal.y() * d / denominator,
            normal.z() * d / denominator,
        );
        let normal = normal.normalize().ok()?;
        let seed = if normal.x().abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let u_axis = normal.cross(seed).normalize().ok()?;
        let v_axis = normal.cross(u_axis);
        let chart = Self {
            origin,
            u_axis,
            v_axis,
        };
        chart.finite().then_some(chart)
    }

    fn evaluate(self, u: f64, v: f64) -> Point3 {
        self.origin + self.u_axis * u + self.v_axis * v
    }

    fn finite(self) -> bool {
        self.origin.0.iter().all(|value| value.is_finite())
            && self.u_axis.0.iter().all(|value| value.is_finite())
            && self.v_axis.0.iter().all(|value| value.is_finite())
    }
}

pub(super) fn plane_line(
    line: &Line2D,
    normal: Vec3,
    d: f64,
    p0: f64,
    p1: f64,
    oriented_start: Point3,
    oriented_end: Point3,
) -> ProofResult {
    let Some(chart) = PlaneChart::canonical(normal, d) else {
        return ProofResult::Unavailable;
    };
    let uv0 = line.evaluate(p0);
    let uv1 = line.evaluate(p1);
    let surface0 = chart.evaluate(uv0.x(), uv0.y());
    let surface1 = chart.evaluate(uv1.x(), uv1.y());
    let d0 = (surface0 - oriented_start).length();
    let d1 = (surface1 - oriented_end).length();
    if !points_finite(&[surface0, surface1, oriented_start, oriented_end])
        || !d0.is_finite()
        || !d1.is_finite()
    {
        return ProofResult::NonFinite;
    }
    // Both lifts are affine in the normalized use parameter.  Their residual
    // is affine too, and the norm of an affine vector is convex, so the
    // complete segment is bounded by its two endpoint norms.
    let arithmetic = arithmetic_bound(
        256.0,
        scalar_scale(&[
            p0,
            p1,
            d,
            uv0.x(),
            uv0.y(),
            uv1.x(),
            uv1.y(),
            surface0.x(),
            surface0.y(),
            surface0.z(),
            surface1.x(),
            surface1.y(),
            surface1.z(),
            oriented_start.x(),
            oriented_start.y(),
            oriented_start.z(),
            oriented_end.x(),
            oriented_end.y(),
            oriented_end.z(),
        ]),
    );
    finish_endpoint_bound(d0, d1, p0, p1, arithmetic)
}

pub(super) fn plane_range(normal: Vec3, d: f64, endpoints: PlaneEndpoints) -> ProofResult {
    let PlaneEndpoints {
        uv0,
        uv1,
        oriented_start,
        oriented_end,
        p0,
        p1,
    } = endpoints;
    let Some(chart) = PlaneChart::canonical(normal, d) else {
        return ProofResult::Unavailable;
    };
    let surface0 = chart.evaluate(uv0.x(), uv0.y());
    let surface1 = chart.evaluate(uv1.x(), uv1.y());
    let d0 = (surface0 - oriented_start).length();
    let d1 = (surface1 - oriented_end).length();
    if !points_finite(&[surface0, surface1, oriented_start, oriented_end])
        || !d0.is_finite()
        || !d1.is_finite()
    {
        return ProofResult::NonFinite;
    }
    let scale = scalar_scale(&[
        d,
        p0,
        p1,
        uv0.x(),
        uv0.y(),
        uv1.x(),
        uv1.y(),
        surface0.x(),
        surface0.y(),
        surface0.z(),
        surface1.x(),
        surface1.y(),
        surface1.z(),
        oriented_start.x(),
        oriented_start.y(),
        oriented_start.z(),
        oriented_end.x(),
        oriented_end.y(),
        oriented_end.z(),
    ]);
    finish_endpoint_bound(d0, d1, p0, p1, arithmetic_bound(256.0, scale))
}

pub(super) fn plane_circle(
    pcurve: &Circle2D,
    normal: Vec3,
    d: f64,
    edge: &Circle3D,
    curve_use: CurveUse,
) -> ProofResult {
    if !pcurve.radius().is_finite() || pcurve.radius() <= 0.0 || !circle_is_qualified(edge) {
        return ProofResult::Unavailable;
    }
    let CurveUse {
        p0,
        p1,
        edge_domain,
        forward,
    } = curve_use;
    let Some(chart) = PlaneChart::canonical(normal, d) else {
        return ProofResult::Unavailable;
    };
    let lifted_center = chart.evaluate(pcurve.center().x(), pcurve.center().y());
    let lifted_cosine = chart.u_axis * pcurve.radius();
    let lifted_sine = chart.v_axis * pcurve.radius();

    let edge_start = if forward {
        edge_domain.0
    } else {
        edge_domain.1
    };
    let edge_span = if forward {
        edge_domain.1 - edge_domain.0
    } else {
        edge_domain.0 - edge_domain.1
    };
    let pcurve_span = p1 - p0;
    let sigma = if (edge_span - pcurve_span).abs() <= (edge_span + pcurve_span).abs() {
        1.0
    } else {
        -1.0
    };
    let phase = edge_start - sigma * p0;
    let (sin_phase, cos_phase) = phase.sin_cos();
    let edge_cosine = (edge.u_axis() * cos_phase + edge.v_axis() * sin_phase) * edge.radius();
    let edge_sine =
        (edge.u_axis() * (-sin_phase) + edge.v_axis() * cos_phase) * (sigma * edge.radius());
    let edge_axis_gain = axis_pair_gain(edge.u_axis(), edge.v_axis());
    let span_residual = mul_up(
        mul_up(edge.radius(), edge_axis_gain),
        (edge_span - sigma * pcurve_span).abs(),
    );
    let bound = add_up(
        add_up(
            outward((lifted_center - edge.center()).length()),
            outward((lifted_cosine - edge_cosine).length()),
        ),
        add_up(outward((lifted_sine - edge_sine).length()), span_residual),
    );
    let scale = scalar_scale(&[
        d,
        p0,
        p1,
        edge_domain.0,
        edge_domain.1,
        pcurve.radius(),
        edge.radius(),
        lifted_center.x(),
        lifted_center.y(),
        lifted_center.z(),
        edge.center().x(),
        edge.center().y(),
        edge.center().z(),
        bound,
    ]);
    let arithmetic = arithmetic_bound(512.0, scale);
    if !bound.is_finite() || !arithmetic.is_finite() {
        return ProofResult::NonFinite;
    }
    if arithmetic > STRICT_TOLERANCE {
        return ProofResult::Unavailable;
    }
    ProofResult::Certified(ProofBound {
        max_deviation: add_up(bound, arithmetic),
        at_parameter: p0,
        evaluations: 0,
    })
}

pub(super) fn affine_nurbs_line(
    surface: &NurbsSurface,
    line: &Line2D,
    p0: f64,
    p1: f64,
    oriented_start: Point3,
    oriented_end: Point3,
) -> ProofResult {
    let Some((domain_u, domain_v)) = qualify_affine_surface(surface) else {
        return ProofResult::Unavailable;
    };
    let uv0 = line.evaluate(p0);
    let uv1 = line.evaluate(p1);
    if !inside(uv0.x(), domain_u)
        || !inside(uv1.x(), domain_u)
        || !inside(uv0.y(), domain_v)
        || !inside(uv1.y(), domain_v)
    {
        return ProofResult::Unavailable;
    }
    let surface0 = surface.evaluate(uv0.x(), uv0.y());
    let surface1 = surface.evaluate(uv1.x(), uv1.y());
    let d0 = (surface0 - oriented_start).length();
    let d1 = (surface1 - oriented_end).length();
    if !points_finite(&[surface0, surface1, oriented_start, oriented_end])
        || !d0.is_finite()
        || !d1.is_finite()
    {
        return ProofResult::NonFinite;
    }
    let mut scale = scalar_scale(&[
        p0,
        p1,
        uv0.x(),
        uv0.y(),
        uv1.x(),
        uv1.y(),
        domain_u.0,
        domain_u.1,
        domain_v.0,
        domain_v.1,
    ]);
    for point in surface.control_points().iter().flatten() {
        scale = scale
            .max(point.x().abs())
            .max(point.y().abs())
            .max(point.z().abs());
    }
    finish_endpoint_bound(d0, d1, p0, p1, arithmetic_bound(512.0, scale))
}

pub(super) fn sphere_circle_nurbs(
    surface: &SphericalSurface,
    pcurve: &NurbsCurve2D,
    edge: &Circle3D,
    curve_use: CurveUse,
) -> ProofResult {
    let Some(sphere_axis_gain) = sphere_axis_gain(surface) else {
        return ProofResult::Unavailable;
    };
    if !circle_is_qualified(edge) {
        return ProofResult::Unavailable;
    }
    let CurveUse {
        p0,
        p1,
        edge_domain,
        ..
    } = curve_use;
    let Some(derivatives) = qualify_cubic_derivatives(pcurve, p0, p1) else {
        return ProofResult::Unavailable;
    };
    let parameter_span = (p1 - p0).abs();
    let u1 = mul_up(derivatives.u1, parameter_span);
    let v1 = mul_up(derivatives.v1, parameter_span);
    let parameter_span_squared = mul_up(parameter_span, parameter_span);
    let u2 = mul_up(derivatives.u2, parameter_span_squared);
    let v2 = mul_up(derivatives.v2, parameter_span_squared);
    let edge_span = (edge_domain.1 - edge_domain.0).abs();
    let first_sum = add_up(u1, v1);
    let sphere_inner = add_up(add_up(u2, v2), mul_up(first_sum, first_sum));
    let sphere_second = mul_up(mul_up(surface.radius(), sphere_axis_gain), sphere_inner);
    let circle_axis_gain = axis_pair_gain(edge.u_axis(), edge.v_axis());
    let circle_second = mul_up(
        mul_up(edge.radius(), circle_axis_gain),
        mul_up(edge_span, edge_span),
    );
    let residual_second = add_up(sphere_second, circle_second);
    if !residual_second.is_finite() {
        return ProofResult::NonFinite;
    }

    let mut scale = scalar_scale(&[
        p0,
        p1,
        edge_domain.0,
        edge_domain.1,
        surface.radius(),
        edge.radius(),
        surface.center().x(),
        surface.center().y(),
        surface.center().z(),
        edge.center().x(),
        edge.center().y(),
        edge.center().z(),
    ]);
    for point in pcurve.control_points() {
        scale = scale.max(point.x().abs()).max(point.y().abs());
    }
    for knot in pcurve.knots() {
        scale = scale.max(knot.abs());
    }
    let arithmetic = arithmetic_bound(8192.0, scale);
    if !arithmetic.is_finite() || arithmetic >= STRICT_TOLERANCE {
        return ProofResult::Unavailable;
    }

    let available = STRICT_TOLERANCE - arithmetic;
    let estimated = if residual_second == 0.0 {
        1
    } else {
        (residual_second / (4.0 * available)).sqrt().ceil() as usize
    };
    let mut intervals = estimated.clamp(1, MAX_SPHERE_INTERVALS);
    loop {
        let Some((max_endpoint, at_parameter)) =
            sphere_residual_endpoints(surface, pcurve, edge, curve_use, intervals)
        else {
            return ProofResult::NonFinite;
        };
        #[allow(clippy::cast_precision_loss)]
        let h = 1.0 / intervals as f64;
        let curvature = outward(mul_up(residual_second, mul_up(h, h)) / 8.0);
        let bound = add_up(add_up(max_endpoint, curvature), arithmetic);
        if !bound.is_finite() {
            return ProofResult::NonFinite;
        }
        // A node beyond the strict tolerance is already a sound rejection
        // witness.  Otherwise only issue a certificate when the complete
        // interval remainder fits inside the default strict band.
        if max_endpoint > STRICT_TOLERANCE || bound <= STRICT_TOLERANCE {
            return ProofResult::Certified(ProofBound {
                max_deviation: bound,
                at_parameter,
                evaluations: intervals + 1,
            });
        }
        if intervals == MAX_SPHERE_INTERVALS {
            return ProofResult::Unavailable;
        }
        intervals = (intervals.saturating_mul(2)).min(MAX_SPHERE_INTERVALS);
    }
}

fn sphere_residual_endpoints(
    surface: &SphericalSurface,
    pcurve: &NurbsCurve2D,
    edge: &Circle3D,
    curve_use: CurveUse,
    intervals: usize,
) -> Option<(f64, f64)> {
    let CurveUse {
        p0,
        p1,
        edge_domain,
        forward,
    } = curve_use;
    let mut max_deviation = 0.0_f64;
    let mut at_parameter = p0;
    for index in 0..=intervals {
        #[allow(clippy::cast_precision_loss)]
        let fraction = index as f64 / intervals as f64;
        let parameter = (p1 - p0).mul_add(fraction, p0);
        let uv = pcurve.evaluate(parameter);
        let on_surface = surface.evaluate(uv.x(), uv.y());
        let edge_parameter = if forward {
            (edge_domain.1 - edge_domain.0).mul_add(fraction, edge_domain.0)
        } else {
            (edge_domain.0 - edge_domain.1).mul_add(fraction, edge_domain.1)
        };
        let on_edge = edge.evaluate(edge_parameter);
        let deviation = (on_surface - on_edge).length();
        if !parameter.is_finite()
            || !uv.0.iter().all(|value| value.is_finite())
            || !points_finite(&[on_surface, on_edge])
            || !deviation.is_finite()
        {
            return None;
        }
        if deviation > max_deviation {
            max_deviation = deviation;
            at_parameter = parameter;
        }
    }
    Some((max_deviation, at_parameter))
}

#[derive(Clone, Copy)]
struct DerivativeBounds {
    u1: f64,
    v1: f64,
    u2: f64,
    v2: f64,
}

fn qualify_cubic_derivatives(curve: &NurbsCurve2D, p0: f64, p1: f64) -> Option<DerivativeBounds> {
    let degree = curve.degree();
    let points = curve.control_points();
    let knots = curve.knots();
    let weights = curve.weights();
    if degree != 3
        || points.len() < degree + 1
        || knots.len() != points.len() + degree + 1
        || weights.len() != points.len()
    {
        return None;
    }
    let weight = *weights.first()?;
    if !weight.is_finite()
        || weight <= 0.0
        || weight.to_bits() != 1.0_f64.to_bits()
        || weights
            .iter()
            .any(|value| value.to_bits() != weight.to_bits())
    {
        return None;
    }
    let domain_start = knots[degree];
    let domain_end = knots[knots.len() - degree - 1];
    if !domain_start.is_finite()
        || !domain_end.is_finite()
        || domain_end <= domain_start
        || !bounds_are_domain(p0, p1, domain_start, domain_end)
        || knots[..=degree]
            .iter()
            .any(|value| value.to_bits() != domain_start.to_bits())
        || knots[knots.len() - degree - 1..]
            .iter()
            .any(|value| value.to_bits() != domain_end.to_bits())
        || !knots[degree..(knots.len() - degree)]
            .windows(2)
            .all(|pair| pair[1] > pair[0])
    {
        return None;
    }

    let knot_scale = knots.iter().copied().map(f64::abs).fold(1.0_f64, f64::max);
    let point_scale = points
        .iter()
        .flat_map(|point| point.0)
        .map(f64::abs)
        .fold(1.0_f64, f64::max);
    let min_safe_denominator = mul_up(1024.0 * f64::EPSILON, knot_scale);
    let mut first = Vec::with_capacity(points.len() - 1);
    for index in 0..points.len() - 1 {
        let denominator = knots[index + degree + 1] - knots[index + 1];
        if !denominator.is_finite() || denominator <= min_safe_denominator {
            return None;
        }
        first.push((points[index + 1] - points[index]) * (degree as f64 / denominator));
    }
    if first
        .iter()
        .any(|vector| !vector.0.iter().all(|value| value.is_finite()))
    {
        return None;
    }
    let mut second = Vec::with_capacity(first.len() - 1);
    for index in 0..first.len() - 1 {
        let denominator = knots[index + degree + 1] - knots[index + 2];
        if !denominator.is_finite() || denominator <= min_safe_denominator {
            return None;
        }
        second.push((first[index + 1] - first[index]) * ((degree - 1) as f64 / denominator));
    }
    if second
        .iter()
        .any(|vector| !vector.0.iter().all(|value| value.is_finite()))
    {
        return None;
    }
    let first_scale = first
        .iter()
        .flat_map(|vector| vector.0)
        .map(f64::abs)
        .fold(1.0_f64, f64::max);
    // Inflate each computed control-net extremum outwards.  The additive
    // terms cover subtraction at the input scale and division by the
    // smallest admitted knot interval; the relative term covers the
    // remaining multiply/divide roundoff.  Tiny, ill-conditioned knot spans
    // are refused above instead of claiming a misleading derivative bound.
    let component_max = |vectors: &[Vec2], component: fn(Vec2) -> f64, additive: f64| {
        let value = vectors
            .iter()
            .copied()
            .map(component)
            .map(f64::abs)
            .fold(0.0_f64, f64::max);
        add_up(add_up(value, mul_up(64.0 * f64::EPSILON, value)), additive)
    };
    let first_inflation = outward(mul_up(64.0 * f64::EPSILON, point_scale) / min_safe_denominator);
    let second_inflation =
        outward(mul_up(128.0 * f64::EPSILON, first_scale) / min_safe_denominator);
    let result = DerivativeBounds {
        u1: component_max(&first, Vec2::x, first_inflation),
        v1: component_max(&first, Vec2::y, first_inflation),
        u2: component_max(&second, Vec2::x, second_inflation),
        v2: component_max(&second, Vec2::y, second_inflation),
    };
    [result.u1, result.v1, result.u2, result.v2]
        .into_iter()
        .all(f64::is_finite)
        .then_some(result)
}

fn bounds_are_domain(p0: f64, p1: f64, start: f64, end: f64) -> bool {
    (p0.to_bits() == start.to_bits() && p1.to_bits() == end.to_bits())
        || (p0.to_bits() == end.to_bits() && p1.to_bits() == start.to_bits())
}

fn qualify_affine_surface(surface: &NurbsSurface) -> Option<((f64, f64), (f64, f64))> {
    if surface.degree_u() != 1 || surface.degree_v() != 1 {
        return None;
    }
    let points = surface.control_points();
    let weights = surface.weights();
    if points.len() != 2
        || points.iter().any(|row| row.len() != 2)
        || weights.len() != 2
        || weights.iter().any(|row| row.len() != 2)
    {
        return None;
    }
    let weight = weights[0][0];
    // The evaluator normalizes by max(Nu*Nv*w).  Degree-one basis maxima
    // are at least 1/2 in each direction, so this range keeps that scale
    // nonzero and finite and bounds the normalized ratios.  Common factors
    // outside it are mathematically cancellable but unsafe in the actual
    // floating evaluator.
    if !weight.is_finite()
        || weight <= 0.0
        || !(f64::MIN_POSITIVE..=f64::MAX / 4.0).contains(&weight)
        || weights
            .iter()
            .flatten()
            .any(|value| value.to_bits() != weight.to_bits())
        || points
            .iter()
            .flatten()
            .any(|point| !point.0.iter().all(|value| value.is_finite()))
    {
        return None;
    }
    let domain_u = clamped_linear_domain(surface.knots_u())?;
    let domain_v = clamped_linear_domain(surface.knots_v())?;
    for component in [Point3::x, Point3::y, Point3::z] {
        if !exact_sum_equal(
            component(points[0][0]),
            component(points[1][1]),
            component(points[1][0]),
            component(points[0][1]),
        ) {
            return None;
        }
    }
    Some((domain_u, domain_v))
}

fn clamped_linear_domain(knots: &[f64]) -> Option<(f64, f64)> {
    let [a, b, c, d] = knots else {
        return None;
    };
    (a.is_finite()
        && c.is_finite()
        && c > a
        && (c - a).is_finite()
        && a.to_bits() == b.to_bits()
        && c.to_bits() == d.to_bits())
    .then_some((*a, *c))
}

fn exact_sum_equal(a: f64, b: f64, c: f64, d: f64) -> bool {
    fn sum(a: f64, b: f64) -> Option<(u64, u64)> {
        if !a.is_finite() || !b.is_finite() {
            return None;
        }
        let high = a + b;
        if !high.is_finite() {
            return None;
        }
        let bb = high - a;
        let low = (a - (high - bb)) + (b - bb);
        if !low.is_finite() {
            return None;
        }
        let bits = |value: f64| {
            if value.abs().to_bits() == 0 {
                0
            } else {
                value.to_bits()
            }
        };
        Some((bits(high), bits(low)))
    }
    matches!((sum(a, b), sum(c, d)), (Some(left), Some(right)) if left == right)
}

fn finish_endpoint_bound(d0: f64, d1: f64, p0: f64, p1: f64, arithmetic: f64) -> ProofResult {
    if !arithmetic.is_finite() {
        return ProofResult::NonFinite;
    }
    if arithmetic > STRICT_TOLERANCE {
        return ProofResult::Unavailable;
    }
    let (endpoint, parameter) = if d0 >= d1 { (d0, p0) } else { (d1, p1) };
    ProofResult::Certified(ProofBound {
        max_deviation: add_up(endpoint, arithmetic),
        at_parameter: parameter,
        evaluations: 2,
    })
}

fn inside(value: f64, domain: (f64, f64)) -> bool {
    value.is_finite() && value >= domain.0 && value <= domain.1
}

fn points_finite(points: &[Point3]) -> bool {
    points
        .iter()
        .all(|point| point.0.iter().all(|value| value.is_finite()))
}

fn circle_is_qualified(circle: &Circle3D) -> bool {
    circle.radius().is_finite()
        && circle.radius() > 0.0
        && points_finite(&[circle.center()])
        && circle.normal().0.iter().all(|value| value.is_finite())
        && circle.u_axis().0.iter().all(|value| value.is_finite())
        && circle.v_axis().0.iter().all(|value| value.is_finite())
        && circle.normal().length() > 0.0
        && circle.u_axis().length() > 0.0
        && circle.v_axis().length() > 0.0
}

fn sphere_axis_gain(sphere: &SphericalSurface) -> Option<f64> {
    if !sphere.radius().is_finite()
        || sphere.radius() <= 0.0
        || !points_finite(&[sphere.center()])
        || !sphere.x_axis().0.iter().all(|value| value.is_finite())
        || !sphere.y_axis().0.iter().all(|value| value.is_finite())
        || !sphere.z_axis().0.iter().all(|value| value.is_finite())
    {
        return None;
    }
    // A square root of the maximum absolute Gram-matrix row sum bounds the
    // frame's operator norm.  It stays close to one for constructor-produced
    // orthonormal frames, while remaining sound for malformed deserialized
    // axes.  `dot_abs_upper` accounts for cancellation and rounds outwards.
    let xx = dot_abs_upper(sphere.x_axis(), sphere.x_axis());
    let xy = dot_abs_upper(sphere.x_axis(), sphere.y_axis());
    let xz = dot_abs_upper(sphere.x_axis(), sphere.z_axis());
    let yy = dot_abs_upper(sphere.y_axis(), sphere.y_axis());
    let yz = dot_abs_upper(sphere.y_axis(), sphere.z_axis());
    let zz = dot_abs_upper(sphere.z_axis(), sphere.z_axis());
    let max_row = add_up(add_up(xx, xy), xz)
        .max(add_up(add_up(xy, yy), yz))
        .max(add_up(add_up(xz, yz), zz));
    let gain = outward(max_row.sqrt());
    (gain.is_finite() && gain > 0.0).then_some(gain)
}

fn scalar_scale(values: &[f64]) -> f64 {
    values.iter().copied().map(f64::abs).fold(1.0_f64, f64::max)
}

fn arithmetic_bound(multiplier: f64, scale: f64) -> f64 {
    mul_up(mul_up(multiplier, f64::EPSILON), scale)
}

fn axis_pair_gain(first: Vec3, second: Vec3) -> f64 {
    let aa = dot_abs_upper(first, first);
    let ab = dot_abs_upper(first, second);
    let bb = dot_abs_upper(second, second);
    outward(add_up(aa, ab).max(add_up(ab, bb)).sqrt())
}

fn dot_abs_upper(first: Vec3, second: Vec3) -> f64 {
    let product_sum = add_up(
        add_up(
            mul_up(first.x().abs(), second.x().abs()),
            mul_up(first.y().abs(), second.y().abs()),
        ),
        mul_up(first.z().abs(), second.z().abs()),
    );
    add_up(
        outward(first.dot(second).abs()),
        mul_up(8.0 * f64::EPSILON, product_sum),
    )
}

fn add_up(left: f64, right: f64) -> f64 {
    outward(left + right)
}

fn mul_up(left: f64, right: f64) -> f64 {
    outward(left * right)
}

fn outward(value: f64) -> f64 {
    if !value.is_finite() || value < 0.0 {
        return value;
    }
    if value == 0.0 {
        return f64::from_bits(1);
    }
    f64::from_bits(value.to_bits() + 1)
}

/// Certificates for the bounded inner-tangent torus runout.  The 3D edge is
/// an exact rational quartic; only its torus coordinates are approximated.
/// Every accepting path bounds the complete interval, including roundoff.
pub(super) mod runout {
    use super::{
        CurveUse, PlaneChart, ProofBound, ProofResult, STRICT_TOLERANCE, add_up, arithmetic_bound,
        axis_pair_gain, bounds_are_domain, circle_is_qualified, dot_abs_upper, mul_up, outward,
        points_finite, qualify_cubic_derivatives, scalar_scale,
    };
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Line2D, NurbsCurve2D};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
    use remus_math::vec::{Point3, Vec3};

    const MAX_INTERVALS: usize = 65_536;
    // The bounded constructor refines at most 256 interpolation intervals.
    // Larger arbitrary UV definitions do not enter this certificate family.
    const MAX_UV_CONTROLS: usize = 257;

    #[derive(Clone, Copy, Debug)]
    struct Iv {
        lo: f64,
        hi: f64,
    }

    impl Iv {
        fn exact(x: f64) -> Self {
            Self { lo: x, hi: x }
        }
        fn add(self, rhs: Self) -> Self {
            Self {
                lo: (self.lo + rhs.lo).next_down(),
                hi: (self.hi + rhs.hi).next_up(),
            }
        }
        fn neg(self) -> Self {
            Self {
                lo: -self.hi,
                hi: -self.lo,
            }
        }
        fn sub(self, rhs: Self) -> Self {
            self.add(rhs.neg())
        }
        fn mul(self, rhs: Self) -> Self {
            let a = [
                self.lo * rhs.lo,
                self.lo * rhs.hi,
                self.hi * rhs.lo,
                self.hi * rhs.hi,
            ];
            Self {
                lo: a.into_iter().fold(f64::INFINITY, f64::min).next_down(),
                hi: a.into_iter().fold(f64::NEG_INFINITY, f64::max).next_up(),
            }
        }
        fn div_positive(self, rhs: Self) -> Option<Self> {
            if rhs.lo <= 0.0 || !rhs.hi.is_finite() {
                return None;
            }
            Some(self.mul(Self {
                lo: (1.0 / rhs.hi).next_down(),
                hi: (1.0 / rhs.lo).next_up(),
            }))
        }
        fn abs_upper(self) -> f64 {
            self.lo.abs().max(self.hi.abs())
        }
        fn finite(self) -> bool {
            self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
        }
    }

    fn choose(n: usize, k: usize) -> f64 {
        // Degrees are at most twelve; all integer intermediates are exactly
        // representable.  The only noninteger operation is enclosed below.
        let mut value = 1_u32;
        for i in 0..k.min(n - k) {
            value = value * (n - i) as u32 / (i + 1) as u32;
        }
        f64::from(value)
    }

    fn product(a: &[Iv], b: &[Iv]) -> Vec<Iv> {
        let (p, q) = (a.len() - 1, b.len() - 1);
        (0..=p + q)
            .map(|k| {
                let mut result = Iv::exact(0.0);
                for i in k.saturating_sub(q)..=k.min(p) {
                    let coefficient = choose(p, i) * choose(q, k - i) / choose(p + q, k);
                    let factor = Iv {
                        lo: coefficient.next_down(),
                        hi: coefficient.next_up(),
                    };
                    result = result.add(a[i].mul(b[k - i]).mul(factor));
                }
                result
            })
            .collect()
    }

    fn derivative(a: &[Iv]) -> Vec<Iv> {
        let degree = Iv::exact((a.len() - 1) as f64);
        a.windows(2).map(|v| v[1].sub(v[0]).mul(degree)).collect()
    }

    fn quotient_norm(numerators: &[Vec<Iv>; 3], denominator: &[Iv]) -> Option<f64> {
        let mut bound = 0.0_f64;
        for (i, &d) in denominator.iter().enumerate() {
            if !d.finite() {
                return None;
            }
            let mut squared = 0.0_f64;
            for numerator in numerators {
                if !numerator[i].finite() {
                    return None;
                }
                let component = numerator[i].div_positive(d)?.abs_upper();
                if !component.is_finite() {
                    return None;
                }
                squared = add_up(squared, mul_up(component, component));
            }
            if !squared.is_finite() {
                return None;
            }
            bound = bound.max(outward(squared.sqrt()));
        }
        bound.is_finite().then_some(bound)
    }

    fn quartic_knots(knots: &[f64]) -> bool {
        knots.len() == 10
            && knots[..5].iter().all(|&x| x.to_bits() == 0.0_f64.to_bits())
            && knots[5..].iter().all(|&x| x.to_bits() == 1.0_f64.to_bits())
    }

    fn homogeneous(edge: &NurbsCurve, anchor: Point3) -> Option<([Vec<Iv>; 3], Vec<Iv>)> {
        if edge.degree() != 4
            || edge.control_points().len() != 5
            || edge.weights().len() != 5
            || !quartic_knots(edge.knots())
            || !points_finite(edge.control_points())
        {
            return None;
        }
        let max = edge.weights().iter().copied().fold(0.0_f64, f64::max);
        if !max.is_finite() || max <= 0.0 {
            return None;
        }
        let weights: Vec<_> = edge
            .weights()
            .iter()
            .map(|&w| {
                if !w.is_finite() || w <= 0.0 {
                    return None;
                }
                Iv::exact(w).div_positive(Iv::exact(max))
            })
            .collect::<Option<_>>()?;
        if weights.iter().any(|w| !w.finite() || w.lo <= 0.0) {
            return None;
        }
        let axes = std::array::from_fn(|j| {
            edge.control_points()
                .iter()
                .zip(&weights)
                .map(|(p, &w)| Iv::exact(p.0[j]).sub(Iv::exact(anchor.0[j])).mul(w))
                .collect()
        });
        Some((axes, weights))
    }

    fn second_bound(edge: &NurbsCurve) -> Option<f64> {
        let (a, w) = homogeneous(edge, *edge.control_points().first()?)?;
        let w1 = derivative(&w);
        let w2 = derivative(&w1);
        let ww = product(&w, &w);
        let denominator = product(&ww, &w);
        let numerators = std::array::from_fn(|j| {
            let a1 = derivative(&a[j]);
            let a2 = derivative(&a1);
            let t0 = product(&a2, &ww);
            let t1 = product(&product(&a[j], &w2), &w);
            let t2 = product(&product(&a1, &w1), &w);
            let t3 = product(&a[j], &product(&w1, &w1));
            let q: Vec<_> = (0..t0.len())
                .map(|i| {
                    t0[i]
                        .sub(t1[i])
                        .sub(t2[i].mul(Iv::exact(2.0)))
                        .add(t3[i].mul(Iv::exact(2.0)))
                })
                .collect();
            // Elevate degree ten to the denominator's degree twelve.
            product(&q, &[Iv::exact(1.0); 3])
        });
        quotient_norm(&numerators, &denominator)
    }

    fn whole_or_witness(
        bound: f64,
        witness: f64,
        parameter: f64,
        evaluations: usize,
    ) -> ProofResult {
        if !bound.is_finite() {
            return ProofResult::NonFinite;
        }
        if bound <= STRICT_TOLERANCE || witness > STRICT_TOLERANCE {
            ProofResult::Certified(ProofBound {
                max_deviation: bound,
                at_parameter: parameter,
                evaluations,
            })
        } else {
            ProofResult::Unavailable
        }
    }

    pub(in crate::validation) fn plane_quartic(
        normal: Vec3,
        d: f64,
        edge: &NurbsCurve,
        uv: &NurbsCurve2D,
        usage: CurveUse,
    ) -> ProofResult {
        let Some(chart) = PlaneChart::canonical(normal, d) else {
            return ProofResult::Unavailable;
        };
        if uv.degree() != 4
            || uv.control_points().len() != 5
            || !quartic_knots(uv.knots())
            || !bounds_are_domain(usage.p0, usage.p1, 0.0, 1.0)
            || !bounds_are_domain(usage.edge_domain.0, usage.edge_domain.1, 0.0, 1.0)
        {
            return ProofResult::Unavailable;
        }
        let anchor = match edge.control_points().first() {
            Some(&p) => p,
            None => return ProofResult::Unavailable,
        };
        let Some((mut a, w)) = homogeneous(edge, anchor) else {
            return ProofResult::Unavailable;
        };
        // `p0..p1` traverses the pcurve; the edge traversal additionally uses
        // the coedge orientation.  Reverse exactly one control net if needed.
        let edge_increasing = (usage.edge_domain.1 > usage.edge_domain.0) == usage.forward;
        if edge_increasing != (usage.p1 > usage.p0) {
            for axis in &mut a {
                axis.reverse();
            }
        }
        let mut ew = w;
        if edge_increasing != (usage.p1 > usage.p0) {
            ew.reverse();
        }
        let maximum = uv.weights().iter().copied().fold(0.0_f64, f64::max);
        if !maximum.is_finite() || maximum <= 0.0 || uv.weights().len() != 5 {
            return ProofResult::Unavailable;
        }
        let uv_scale = uv
            .control_points()
            .iter()
            .flat_map(|p| p.0)
            .map(f64::abs)
            .fold(1.0_f64, f64::max);
        // The 2D evaluator accumulates unnormalized weights and deliberately
        // falls back below EPSILON.  Do not certify that different evaluator
        // by cancelling a mathematically harmless but unsafe common factor.
        if uv.weights().iter().any(|&w| w < 2.0 * f64::EPSILON)
            || maximum > f64::MAX / (128.0 * uv_scale)
        {
            return ProofResult::Unavailable;
        }
        let Some(pw) = uv
            .weights()
            .iter()
            .map(|&v| {
                (v.is_finite() && v > 0.0)
                    .then(|| Iv::exact(v).div_positive(Iv::exact(maximum)))
                    .flatten()
            })
            .collect::<Option<Vec<_>>>()
        else {
            return ProofResult::Unavailable;
        };
        let b: [Vec<Iv>; 3] = std::array::from_fn(|j| {
            uv.control_points()
                .iter()
                .zip(&pw)
                .map(|(p, &weight)| {
                    Iv::exact(chart.origin.0[j])
                        .sub(Iv::exact(anchor.0[j]))
                        .add(Iv::exact(chart.u_axis.0[j]).mul(Iv::exact(p.x())))
                        .add(Iv::exact(chart.v_axis.0[j]).mul(Iv::exact(p.y())))
                        .mul(weight)
                })
                .collect()
        });
        let denominator = product(&ew, &pw);
        let numerator = std::array::from_fn(|j| {
            let left = product(&b[j], &ew);
            let right = product(&a[j], &pw);
            left.iter().zip(right).map(|(&l, r)| l.sub(r)).collect()
        });
        let Some(bound) = quotient_norm(&numerator, &denominator) else {
            return ProofResult::Unavailable;
        };
        let scale = edge
            .control_points()
            .iter()
            .flat_map(|p| p.0)
            .map(f64::abs)
            .fold(d.abs().max(uv_scale).max(1.0), f64::max);
        let arithmetic = arithmetic_bound(8192.0, scale);
        let witness = (chart.evaluate(uv.evaluate(usage.p0).x(), uv.evaluate(usage.p0).y())
            - edge.evaluate(if usage.forward {
                usage.edge_domain.0
            } else {
                usage.edge_domain.1
            }))
        .length();
        whole_or_witness(add_up(bound, arithmetic), witness, usage.p0, 2)
    }

    fn frame_gain(x: Vec3, y: Vec3, z: Vec3) -> Option<f64> {
        if [x, y, z].iter().any(|v| v.0.iter().any(|x| !x.is_finite())) {
            return None;
        }
        let row = |a: Vec3, b: Vec3, c: Vec3| {
            add_up(
                add_up(dot_abs_upper(a, a), dot_abs_upper(a, b)),
                dot_abs_upper(a, c),
            )
        };
        let gain = outward(row(x, y, z).max(row(y, x, z)).max(row(z, x, y)).sqrt());
        (gain.is_finite() && gain > 0.0).then_some(gain)
    }

    pub(in crate::validation) fn torus_quartic(
        surface: &ToroidalSurface,
        uv: &NurbsCurve2D,
        edge: &NurbsCurve,
        usage: CurveUse,
    ) -> ProofResult {
        let Some(gain) = frame_gain(surface.x_axis(), surface.y_axis(), surface.z_axis()) else {
            return ProofResult::Unavailable;
        };
        let (major, minor) = (surface.major_radius(), surface.minor_radius());
        if !major.is_finite()
            || !minor.is_finite()
            || minor <= 0.0
            || major <= minor
            || uv.control_points().len() > MAX_UV_CONTROLS
            || !bounds_are_domain(usage.edge_domain.0, usage.edge_domain.1, 0.0, 1.0)
        {
            return ProofResult::Unavailable;
        }
        let Some(derivatives) = qualify_cubic_derivatives(uv, usage.p0, usage.p1) else {
            return ProofResult::Unavailable;
        };
        let Some(edge_second) = second_bound(edge) else {
            return ProofResult::Unavailable;
        };
        let span = (usage.p1 - usage.p0).abs();
        let u1 = mul_up(derivatives.u1, span);
        let v1 = mul_up(derivatives.v1, span);
        let span2 = mul_up(span, span);
        let u2 = mul_up(derivatives.u2, span2);
        let v2 = mul_up(derivatives.v2, span2);
        let torus_second = mul_up(
            gain,
            add_up(
                mul_up(add_up(major, minor), add_up(u2, mul_up(u1, u1))),
                mul_up(
                    minor,
                    add_up(v2, add_up(mul_up(2.0, mul_up(u1, v1)), mul_up(v1, v1))),
                ),
            ),
        );
        let second = add_up(torus_second, edge_second);
        let scale = edge
            .control_points()
            .iter()
            .flat_map(|p| p.0)
            .chain(uv.control_points().iter().flat_map(|p| p.0))
            .map(f64::abs)
            .fold(major.max(minor).max(1.0), f64::max);
        let min_weight = edge.weights().iter().copied().fold(f64::INFINITY, f64::min);
        let condition = outward(edge.max_weight() / min_weight);
        let carrier_scale = add_up(
            scalar_scale(&[
                surface.center().x(),
                surface.center().y(),
                surface.center().z(),
            ]),
            mul_up(gain, add_up(major, mul_up(2.0, minor))),
        );
        let parameter_scale = uv.knots().iter().copied().map(f64::abs).fold(
            scalar_scale(&[usage.p0, usage.p1, usage.p1 - usage.p0]),
            f64::max,
        );
        let parameter_error = arithmetic_bound(256.0, parameter_scale);
        let surface_first = mul_up(
            gain,
            add_up(
                mul_up(add_up(major, minor), derivatives.u1),
                mul_up(minor, derivatives.v1),
            ),
        );
        let uv_scale = uv
            .control_points()
            .iter()
            .flat_map(|p| p.0)
            .map(f64::abs)
            .fold(1.0_f64, f64::max);
        let uv_error = mul_up(
            mul_up(gain, add_up(major, mul_up(2.0, minor))),
            arithmetic_bound(8192.0, uv_scale),
        );
        // Endpoints are evaluated in floating point too. Cover conversion
        // from their witnesses to the real residual and back at an interior
        // point, including parameter-grid quantization and the raw carrier.
        let arithmetic = mul_up(
            2.0,
            add_up(
                add_up(
                    arithmetic_bound(8192.0, mul_up(scale.max(carrier_scale), condition)),
                    mul_up(surface_first, parameter_error),
                ),
                uv_error,
            ),
        );
        if !second.is_finite() || !arithmetic.is_finite() || arithmetic >= STRICT_TOLERANCE {
            return ProofResult::Unavailable;
        }
        let available = STRICT_TOLERANCE - arithmetic;
        let mut intervals =
            ((second / (4.0 * available)).sqrt().ceil() as usize).clamp(1, MAX_INTERVALS);
        loop {
            let mut witness = 0.0_f64;
            let mut parameter = usage.p0;
            for index in 0..=intervals {
                let f = index as f64 / intervals as f64;
                let p = (usage.p1 - usage.p0).mul_add(f, usage.p0);
                let t = if usage.forward {
                    (usage.edge_domain.1 - usage.edge_domain.0).mul_add(f, usage.edge_domain.0)
                } else {
                    (usage.edge_domain.0 - usage.edge_domain.1).mul_add(f, usage.edge_domain.1)
                };
                let coords = uv.evaluate(p);
                let on_surface = surface.evaluate(coords.x(), coords.y());
                let on_edge = edge.evaluate(t);
                let residual = (on_surface - on_edge).length();
                if !points_finite(&[on_surface, on_edge]) || !residual.is_finite() {
                    return ProofResult::NonFinite;
                }
                if residual > witness {
                    witness = residual;
                    parameter = p;
                }
            }
            let h = outward(1.0 / intervals as f64);
            let bound = add_up(
                add_up(witness, outward(mul_up(second, mul_up(h, h)) / 8.0)),
                arithmetic,
            );
            if witness > STRICT_TOLERANCE || bound <= STRICT_TOLERANCE {
                return whole_or_witness(bound, witness, parameter, intervals + 1);
            }
            if intervals == MAX_INTERVALS {
                return ProofResult::Unavailable;
            }
            intervals = (intervals * 2).min(MAX_INTERVALS);
        }
    }

    /// A trigonometric coefficient comparison plus a frequency mismatch
    /// bound proves the entire interval; endpoint agreement is insufficient.
    #[allow(clippy::too_many_arguments)] // Two trigonometric coefficient frames and their native use.
    fn circle_bound(
        center: Point3,
        a: Vec3,
        b: Vec3,
        phase: f64,
        span: f64,
        edge: &Circle3D,
        usage: CurveUse,
        drift: f64,
        evaluation_error: f64,
    ) -> ProofResult {
        if !circle_is_qualified(edge) {
            return ProofResult::Unavailable;
        }
        let (e0, e1) = if usage.forward {
            usage.edge_domain
        } else {
            (usage.edge_domain.1, usage.edge_domain.0)
        };
        let (se, ce) = e0.sin_cos();
        let (ss, cs) = phase.sin_cos();
        let ea = (edge.u_axis() * ce + edge.v_axis() * se) * edge.radius();
        let eb = (-edge.u_axis() * se + edge.v_axis() * ce) * edge.radius();
        let sa = a * cs + b * ss;
        let sb = -a * ss + b * cs;
        let coefficient = |sign: f64| {
            add_up(
                add_up((center - edge.center()).length(), (sa - ea).length()),
                add_up(
                    (sb * sign - eb).length(),
                    mul_up(
                        add_up(a.length(), b.length()),
                        (span - sign * (e1 - e0)).abs(),
                    ),
                ),
            )
        };
        let scale = scalar_scale(&[
            center.x(),
            center.y(),
            center.z(),
            edge.center().x(),
            edge.center().y(),
            edge.center().z(),
            a.length(),
            b.length(),
            phase,
            span,
            e0,
            e1,
        ]);
        let native_gain = mul_up(edge.radius(), axis_pair_gain(edge.u_axis(), edge.v_axis()));
        let native_parameter_error = mul_up(
            native_gain,
            arithmetic_bound(256.0, scalar_scale(&[e0, e1, e1 - e0])),
        );
        let arithmetic = add_up(
            add_up(arithmetic_bound(8192.0, scale), evaluation_error),
            mul_up(2.0, native_parameter_error),
        );
        let bound = add_up(
            add_up(coefficient(1.0).min(coefficient(-1.0)), drift),
            arithmetic,
        );
        // The constant-coordinate surrogate may differ from the actual
        // surface by `drift`, so its endpoint is not a rejection witness.
        whole_or_witness(bound, 0.0, usage.p0, 2)
    }

    fn line_roundoff(
        line: &Line2D,
        usage: CurveUse,
        carrier_scale: f64,
        u_gain: f64,
        v_gain: f64,
    ) -> Option<f64> {
        if line
            .origin()
            .0
            .iter()
            .chain(line.direction().0.iter())
            .any(|x| !x.is_finite())
            || [usage.p0, usage.p1, carrier_scale, u_gain, v_gain]
                .iter()
                .any(|x| !x.is_finite())
        {
            return None;
        }
        if carrier_scale < 0.0 || u_gain < 0.0 || v_gain < 0.0 {
            return None;
        }
        // The coefficient proof uses a line through rounded endpoint values.
        // A Line2D with cancelling origin/parameters can agree there and jump
        // in the interior. Bound original parameter interpolation and line
        // evaluation, then propagate that UV error through the carrier's
        // global derivative gains. Never infer this error from endpoints.
        let parameters = scalar_scale(&[usage.p0, usage.p1, usage.p1 - usage.p0]);
        let direction = line.direction();
        // L1 gains avoid squaring underflow/overflow in malformed but finite
        // serialized directions. A computed zero Euclidean norm is not a
        // bound on a nonzero subnormal-scale derivative.
        let direction_gain = add_up(direction.x().abs(), direction.y().abs());
        let uv_scale = add_up(
            scalar_scale(&[line.origin().x(), line.origin().y()]),
            mul_up(parameters, direction_gain),
        );
        let uv_error = arithmetic_bound(256.0, uv_scale);
        let error = mul_up(
            2.0,
            add_up(
                arithmetic_bound(8192.0, carrier_scale),
                mul_up(add_up(u_gain, v_gain), uv_error),
            ),
        );
        (error.is_finite() && error < STRICT_TOLERANCE).then_some(error)
    }

    fn vector_gain(vector: Vec3) -> f64 {
        add_up(add_up(vector.x().abs(), vector.y().abs()), vector.z().abs())
    }

    pub(in crate::validation) fn cylinder_circle(
        surface: &CylindricalSurface,
        line: &Line2D,
        edge: &Circle3D,
        usage: CurveUse,
    ) -> ProofResult {
        if !surface.radius().is_finite() || surface.radius() <= 0.0 {
            return ProofResult::Unavailable;
        }
        let p0 = line.evaluate(usage.p0);
        let p1 = line.evaluate(usage.p1);
        let v = (p0.y() + p1.y()) * 0.5;
        let drift = mul_up(
            vector_gain(surface.axis()),
            outward((p1.y() - p0.y()).abs() * 0.5),
        );
        let u_gain = mul_up(
            surface.radius(),
            axis_pair_gain(surface.x_axis(), surface.y_axis()),
        );
        let v_gain = vector_gain(surface.axis());
        let carrier_scale = add_up(
            scalar_scale(&[
                surface.origin().x(),
                surface.origin().y(),
                surface.origin().z(),
                surface.radius(),
            ]),
            add_up(
                mul_up(
                    surface.radius(),
                    add_up(vector_gain(surface.x_axis()), vector_gain(surface.y_axis())),
                ),
                mul_up(v_gain, p0.y().abs().max(p1.y().abs())),
            ),
        );
        let Some(evaluation_error) = line_roundoff(line, usage, carrier_scale, u_gain, v_gain)
        else {
            return ProofResult::Unavailable;
        };
        circle_bound(
            surface.origin() + surface.axis() * v,
            surface.x_axis() * surface.radius(),
            surface.y_axis() * surface.radius(),
            p0.x(),
            p1.x() - p0.x(),
            edge,
            usage,
            drift,
            evaluation_error,
        )
    }

    pub(in crate::validation) fn torus_circle(
        surface: &ToroidalSurface,
        line: &Line2D,
        edge: &Circle3D,
        usage: CurveUse,
    ) -> ProofResult {
        let Some(gain) = frame_gain(surface.x_axis(), surface.y_axis(), surface.z_axis()) else {
            return ProofResult::Unavailable;
        };
        let (major, minor) = (surface.major_radius(), surface.minor_radius());
        if !major.is_finite() || !minor.is_finite() || minor <= 0.0 || major <= minor {
            return ProofResult::Unavailable;
        }
        let p0 = line.evaluate(usage.p0);
        let p1 = line.evaluate(usage.p1);
        let carrier_scale = add_up(
            scalar_scale(&[
                surface.center().x(),
                surface.center().y(),
                surface.center().z(),
                major,
                minor,
            ]),
            mul_up(gain, add_up(major, mul_up(2.0, minor))),
        );
        let Some(evaluation_error) = line_roundoff(
            line,
            usage,
            carrier_scale,
            mul_up(add_up(major, minor), gain),
            mul_up(minor, gain),
        ) else {
            return ProofResult::Unavailable;
        };
        let u = (p0.x() + p1.x()) * 0.5;
        let v = (p0.y() + p1.y()) * 0.5;
        let (sv, cv) = v.sin_cos();
        let radial = major + minor * cv;
        let ring = circle_bound(
            surface.center() + surface.z_axis() * (minor * sv),
            surface.x_axis() * radial,
            surface.y_axis() * radial,
            p0.x(),
            p1.x() - p0.x(),
            edge,
            usage,
            mul_up(mul_up(minor, gain), outward((p1.y() - p0.y()).abs() * 0.5)),
            evaluation_error,
        );
        let (su, cu) = u.sin_cos();
        let direction = surface.x_axis() * cu + surface.y_axis() * su;
        let meridian = circle_bound(
            surface.center() + direction * major,
            direction * minor,
            surface.z_axis() * minor,
            p0.y(),
            p1.y() - p0.y(),
            edge,
            usage,
            mul_up(
                mul_up(add_up(major, minor), gain),
                outward((p1.x() - p0.x()).abs() * 0.5),
            ),
            evaluation_error,
        );
        match (ring, meridian) {
            (ProofResult::Certified(a), ProofResult::Certified(b)) => {
                ProofResult::Certified(if a.max_deviation <= b.max_deviation {
                    a
                } else {
                    b
                })
            }
            (ProofResult::Certified(a), _) | (_, ProofResult::Certified(a)) => {
                ProofResult::Certified(a)
            }
            (ProofResult::NonFinite, _) | (_, ProofResult::NonFinite) => ProofResult::NonFinite,
            _ => ProofResult::Unavailable,
        }
    }

    #[cfg(test)]
    mod tests {
        #![allow(clippy::unwrap_used, clippy::panic)]
        use super::*;
        use remus_math::vec::{Point2, Vec2};

        fn usage(reverse: bool, decreasing: bool) -> CurveUse {
            let domain = if decreasing { (1.0, 0.0) } else { (0.0, 1.0) };
            let p = if reverse {
                (domain.1, domain.0)
            } else {
                domain
            };
            CurveUse {
                p0: p.0,
                p1: p.1,
                edge_domain: domain,
                forward: !reverse,
            }
        }
        fn fixture() -> NurbsCurve {
            NurbsCurve::new(
                4,
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                vec![
                    Point3::new(53.417_424_305_044_165, 0.0, 4.0),
                    Point3::new(54.999_443_258_096_74, 0.0, 4.0),
                    Point3::new(56.258_309_964_312_73, 0.0, 4.866_874_061_238_179),
                    Point3::new(57.146_765_619_130_93, 0.0, 5.955_005_567_935_635),
                    Point3::new(58.0, 0.0, 7.0),
                ],
                vec![
                    1.678_404_338_007_679_2,
                    2.518_751_156_974_872,
                    4.074_159_777_049_061,
                    6.448_449_316_513_905,
                    10.0,
                ],
            )
            .unwrap()
        }
        fn certified(proof: ProofResult) -> ProofBound {
            let ProofResult::Certified(bound) = proof else {
                panic!("missing whole-interval certificate: {proof:?}");
            };
            assert!(bound.max_deviation <= STRICT_TOLERANCE, "{bound:?}");
            bound
        }

        #[test]
        fn runout_quartic_plane_certifies_both_uses_and_native_trims() {
            let edge = fixture();
            let chart = PlaneChart::canonical(Vec3::new(0.0, 1.0, 0.0), 0.0).unwrap();
            for factor in [1.0, 2.0_f64.powi(100)] {
                let poles = edge
                    .control_points()
                    .iter()
                    .map(|&p| {
                        let q = p - chart.origin;
                        Point2::new(q.dot(chart.u_axis), q.dot(chart.v_axis))
                    })
                    .collect();
                let uv = NurbsCurve2D::new(
                    4,
                    edge.knots().to_vec(),
                    poles,
                    edge.weights().iter().map(|w| w * factor).collect(),
                )
                .unwrap();
                for reverse in [false, true] {
                    for decreasing in [false, true] {
                        certified(plane_quartic(
                            Vec3::new(0.0, 1.0, 0.0),
                            0.0,
                            &edge,
                            &uv,
                            usage(reverse, decreasing),
                        ));
                    }
                }
            }
        }

        #[test]
        fn runout_quartic_plane_refuses_hidden_control_and_unsafe_weight_factor() {
            let edge = fixture();
            let chart = PlaneChart::canonical(Vec3::new(0.0, 1.0, 0.0), 0.0).unwrap();
            let poles: Vec<_> = edge
                .control_points()
                .iter()
                .map(|&p| {
                    let q = p - chart.origin;
                    Point2::new(q.dot(chart.u_axis), q.dot(chart.v_axis))
                })
                .collect();
            let mut wrong = poles.clone();
            wrong[2].0[0] += 0.01;
            let uv = NurbsCurve2D::new(4, edge.knots().to_vec(), wrong, edge.weights().to_vec())
                .unwrap();
            assert!(matches!(
                plane_quartic(
                    Vec3::new(0.0, 1.0, 0.0),
                    0.0,
                    &edge,
                    &uv,
                    usage(false, false)
                ),
                ProofResult::Unavailable
            ));
            let uv = NurbsCurve2D::new(
                4,
                edge.knots().to_vec(),
                poles,
                edge.weights().iter().map(|w| w * 1e-100).collect(),
            )
            .unwrap();
            assert!(matches!(
                plane_quartic(
                    Vec3::new(0.0, 1.0, 0.0),
                    0.0,
                    &edge,
                    &uv,
                    usage(false, false)
                ),
                ProofResult::Unavailable
            ));
        }

        #[test]
        fn runout_interval_quotient_refuses_nonfinite_and_nonpositive_denominators() {
            let zero = vec![Iv::exact(0.0)];
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                assert!(
                    quotient_norm(
                        &[vec![Iv::exact(invalid)], zero.clone(), zero.clone()],
                        &[Iv::exact(1.0)]
                    )
                    .is_none()
                );
            }
            assert!(
                quotient_norm(
                    &[zero.clone(), zero.clone(), zero],
                    &[Iv { lo: 0.0, hi: 1.0 }]
                )
                .is_none()
            );
        }

        fn torus_uv(edge: &NurbsCurve, surface: &ToroidalSurface, samples: usize) -> NurbsCurve2D {
            let mut points: Vec<Point3> = Vec::new();
            let mut params = Vec::new();
            for i in 0..=samples {
                let t = i as f64 / samples as f64;
                let q = edge.evaluate(t) - surface.center();
                let x = q.dot(surface.x_axis());
                let y = q.dot(surface.y_axis());
                let mut u = y.atan2(x);
                let mut v = q
                    .dot(surface.z_axis())
                    .atan2(x.hypot(y) - surface.major_radius());
                if let Some(p) = points.last() {
                    u += std::f64::consts::TAU * ((p.x() - u) / std::f64::consts::TAU).round();
                    v += std::f64::consts::TAU * ((p.y() - v) / std::f64::consts::TAU).round();
                }
                points.push(Point3::new(u, v, 0.0));
                params.push(t);
            }
            let fitted =
                remus_math::nurbs::fitting::interpolate_with_params(&points, 3, &params).unwrap();
            NurbsCurve2D::new(
                3,
                fitted.knots().to_vec(),
                fitted
                    .control_points()
                    .iter()
                    .map(|p| Point2::new(p.x(), p.y()))
                    .collect(),
                fitted.weights().to_vec(),
            )
            .unwrap()
        }

        #[test]
        fn runout_torus_quartic_certifies_complete_interval_not_only_fit_nodes() {
            let edge = fixture();
            let surface = ToroidalSurface::new(Point3::new(58.0, 2.0, 7.0), 5.0, 3.0).unwrap();
            let bound = second_bound(&edge).unwrap();
            assert!((28.0..29.0).contains(&bound), "{bound}");
            let uv = torus_uv(&edge, &surface, 128);
            for reverse in [false, true] {
                for decreasing in [false, true] {
                    let proof = certified(torus_quartic(
                        &surface,
                        &uv,
                        &edge,
                        usage(reverse, decreasing),
                    ));
                    assert!(proof.evaluations > 128 && proof.evaluations <= MAX_INTERVALS + 1);
                }
            }
            let coarse = torus_uv(&edge, &surface, 4);
            assert!(
                !matches!(torus_quartic(&surface,&coarse,&edge,usage(false,false)),ProofResult::Certified(b) if b.max_deviation<=STRICT_TOLERANCE)
            );
            let mut poles = uv.control_points().to_vec();
            let middle = poles.len() / 2;
            poles[middle].0[0] += 0.01;
            let wrong =
                NurbsCurve2D::new(3, uv.knots().to_vec(), poles, uv.weights().to_vec()).unwrap();
            assert!(
                !matches!(torus_quartic(&surface,&wrong,&edge,usage(false,false)),ProofResult::Certified(b) if b.max_deviation<=STRICT_TOLERANCE)
            );
        }

        #[test]
        fn runout_native_circles_cover_ring_meridian_and_decreasing_use() {
            let surface = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0, 3.0).unwrap();
            for ring in [false, true] {
                let (circle, line) = if ring {
                    (
                        Circle3D::new_with_ref(
                            surface.center(),
                            surface.z_axis(),
                            8.0,
                            surface.x_axis(),
                        )
                        .unwrap(),
                        Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
                    )
                } else {
                    // An explicit native circle frame avoids assuming the
                    // constructor's arbitrary meridian reference direction.
                    (
                        Circle3D::new_with_ref(
                            surface.center() + surface.x_axis() * 5.0,
                            surface.x_axis().cross(surface.z_axis()),
                            3.0,
                            surface.x_axis(),
                        )
                        .unwrap(),
                        Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                    )
                };
                for reverse in [false, true] {
                    for decreasing in [false, true] {
                        certified(torus_circle(
                            &surface,
                            &line,
                            &circle,
                            usage(reverse, decreasing),
                        ));
                    }
                }
            }
        }

        #[test]
        fn runout_circle_line_refuses_cancelling_parameters_in_both_directions() {
            let big = 1e16_f64;
            let line = Line2D::new(Point2::new(-big, 0.0), Vec2::new(1.0, 0.0)).unwrap();
            let cylinder = CylindricalSurface::with_ref_dir(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let circle = Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 0.0),
                cylinder.axis(),
                1.0,
                cylinder.x_axis(),
            )
            .unwrap();
            let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 2.0, 1.0).unwrap();
            let ring = Circle3D::new_with_ref(torus.center(), torus.z_axis(), 3.0, torus.x_axis())
                .unwrap();
            for reverse in [false, true] {
                for decreasing in [false, true] {
                    let domain = if decreasing { (2.0, 0.0) } else { (0.0, 2.0) };
                    let (e0, e1) = if reverse {
                        (domain.1, domain.0)
                    } else {
                        domain
                    };
                    let (p0, p1) = if e1 > e0 {
                        (big, big.next_up())
                    } else {
                        (big.next_up(), big)
                    };
                    let usage = CurveUse {
                        p0,
                        p1,
                        edge_domain: domain,
                        forward: !reverse,
                    };
                    let at = (p1 - p0).mul_add(0.25, p0);
                    let uv = line.evaluate(at);
                    let edge_at = (e1 - e0).mul_add(0.25, e0);
                    assert!(
                        (cylinder.evaluate(uv.x(), uv.y()) - circle.evaluate(edge_at)).length()
                            > 0.49
                    );
                    for (p, t) in [(p0, e0), (p1, e1)] {
                        let uv = line.evaluate(p);
                        assert!(
                            (cylinder.evaluate(uv.x(), uv.y()) - circle.evaluate(t)).length()
                                < 1e-14
                        );
                    }
                    assert!(matches!(
                        cylinder_circle(&cylinder, &line, &circle, usage),
                        ProofResult::Unavailable
                    ));
                    assert!(matches!(
                        torus_circle(&torus, &line, &ring, usage),
                        ProofResult::Unavailable
                    ));
                }
            }
        }

        #[test]
        fn runout_circle_refuses_hidden_large_torus_carrier_cancellation() {
            let surface = ToroidalSurface::new(Point3::new(-1e16, 0.0, 0.0), 1e16, 1.0).unwrap();
            let edge = Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                1.0,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let line = Line2D::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap();
            assert!((surface.evaluate(0.0, 0.25) - edge.evaluate(0.25)).length() > 0.9);
            assert!(matches!(
                torus_circle(&surface, &line, &edge, usage(false, false)),
                ProofResult::Unavailable
            ));
        }

        #[test]
        fn runout_carrier_derivative_gain_does_not_underflow() {
            let axis = Vec3::new(1e-200, 0.0, 0.0);
            assert_eq!(axis.length().to_bits(), 0.0_f64.to_bits());
            assert!(vector_gain(axis) >= 1e-200);
            assert!(mul_up(vector_gain(axis), 1e200) >= 1.0);
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn affine_qualification_refuses_overflowing_finite_knot_span() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![-f64::MAX, -f64::MAX, f64::MAX, f64::MAX],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();

        assert!(qualify_affine_surface(&surface).is_none());
    }
}
