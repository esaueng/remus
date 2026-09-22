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
