//! Analytic candidate refinement; residuals verify events, never create them.
use super::{ArrangementError, ArrangementHalfEdge, CurveUse, Result, Work};
use remus_math::curves2d::Curve2D;
use remus_math::predicates::orient2d;
use remus_math::vec::{Point2, Vec2};
use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::f64::consts::TAU;

/// Numeric identity of a computed native parameter, with signed zero erased.
/// This is intentionally NOT a geometric tolerance comparison.
pub(super) fn same(a: f64, b: f64) -> bool {
    let a = if a.abs() <= 0.0 { 0.0 } else { a };
    let b = if b.abs() <= 0.0 { 0.0 } else { b };
    a.total_cmp(&b).is_eq()
}
pub(super) fn roundoff(p: Point2) -> f64 {
    64.0 * f64::EPSILON * (1.0 + p.x().abs() + p.y().abs())
}
fn finite(p: Point2) -> bool {
    p.x().is_finite() && p.y().is_finite()
}
fn cross(a: Vec2, b: Vec2) -> f64 {
    orient2d(
        Point2::new(0.0, 0.0),
        Point2::new(a.x(), a.y()),
        Point2::new(b.x(), b.y()),
    )
}

pub(super) fn validate_use(u: &CurveUse, tolerance: f64) -> Result<()> {
    if !u.range.iter().chain(&u.source_range).all(|t| t.is_finite())
        || u.endpoints_3d
            .iter()
            .any(|p| !p.x().is_finite() || !p.y().is_finite() || !p.z().is_finite())
        || !finite(u.point(u.range[0]))
        || !finite(u.point(u.range[1]))
    {
        return Err(ArrangementError::NonFiniteInput);
    }
    if !matches!(
        u.curve_3d,
        remus_topology::edge::EdgeCurve::Line | remus_topology::edge::EdgeCurve::Circle(_)
    ) {
        return Err(ArrangementError::UnsupportedCurve);
    }
    if same(u.range[0], u.range[1]) || same(u.source_range[0], u.source_range[1]) {
        return Err(ArrangementError::InvalidBoundary);
    }
    if let remus_topology::edge::EdgeCurve::Circle(circle) = &u.curve_3d {
        for end in 0..2 {
            let p = circle.evaluate(u.source_range[end]);
            if !p.x().is_finite() || !p.y().is_finite() || !p.z().is_finite() {
                return Err(ArrangementError::NonFiniteInput);
            }
            if (p - u.endpoints_3d[end]).length() > tolerance {
                return Err(ArrangementError::InvalidBoundary);
            }
        }
    }
    match &u.pcurve {
        Curve2D::Line(line) => {
            if !finite(line.origin()) || !line.direction().length().is_finite() {
                return Err(ArrangementError::NonFiniteInput);
            }
        }
        Curve2D::Circle(circle) => {
            if !finite(circle.center()) || !circle.radius().is_finite() {
                return Err(ArrangementError::NonFiniteInput);
            }
            if u.range[0].min(u.range[1]) < 0.0 || u.range[0].max(u.range[1]) > TAU {
                return Err(ArrangementError::UnsupportedCurve);
            }
        }
        _ => return Err(ArrangementError::UnsupportedCurve),
    }
    Ok(())
}

fn in_range(u: &CurveUse, t: f64) -> bool {
    t >= u.range[0] && t <= u.range[1]
}
fn angle(u: &CurveUse, p: Point2) -> f64 {
    let Curve2D::Circle(c) = &u.pcurve else {
        return f64::NAN;
    };
    (p.y() - c.center().y())
        .atan2(p.x() - c.center().x())
        .rem_euclid(TAU)
}

#[allow(clippy::too_many_lines)]
pub(super) fn intersections(
    a: &CurveUse,
    b: &CurveUse,
    work: &mut Work<'_>,
) -> Result<Vec<(f64, f64)>> {
    let mut hits = Vec::new();
    match (&a.pcurve, &b.pcurve) {
        (Curve2D::Line(la), Curve2D::Line(lb)) => {
            let p = a.point(a.range[0]);
            let q = a.point(a.range[1]);
            let r = b.point(b.range[0]);
            let s = b.point(b.range[1]);
            let ar = orient2d(p, q, r);
            let as_ = orient2d(p, q, s);
            let bp = orient2d(r, s, p);
            let bq = orient2d(r, s, q);
            if same(ar, 0.0) && same(as_, 0.0) {
                let t0 = la.project(r);
                let t1 = la.project(s);
                let lo = a.range[0].max(t0.min(t1));
                let hi = a.range[1].min(t0.max(t1));
                if lo < hi {
                    return Err(ArrangementError::AmbiguousOverlap);
                }
                if same(lo, hi) {
                    hits.push((lo, lb.project(a.point(lo))));
                }
            } else if !((ar > 0.0 && as_ > 0.0)
                || (ar < 0.0 && as_ < 0.0)
                || (bp > 0.0 && bq > 0.0)
                || (bp < 0.0 && bq < 0.0))
            {
                // Endpoint-on-support is certified by the exact orientation sign.
                for (t, side) in [(a.range[0], bp), (a.range[1], bq)] {
                    work.step()?;
                    if same(side, 0.0) {
                        hits.push((t, lb.project(a.point(t))));
                    }
                }
                for (t, side) in [(b.range[0], ar), (b.range[1], as_)] {
                    work.step()?;
                    if same(side, 0.0) {
                        hits.push((la.project(b.point(t)), t));
                    }
                }
                if hits.is_empty() {
                    let d = la.direction();
                    let e = lb.direction();
                    let den = cross(d, e);
                    if same(den, 0.0) {
                        return Err(ArrangementError::IntersectionRefinementFailed);
                    }
                    let delta = lb.origin() - la.origin();
                    hits.push((cross(delta, e) / den, cross(delta, d) / den));
                }
            }
        }
        (Curve2D::Line(line), Curve2D::Circle(circle)) => {
            let d = line.direction();
            let offset = line.origin() - circle.center();
            let projection = offset.dot(d);
            let aa = d.length_squared();
            let cc = offset.length_squared() - circle.radius() * circle.radius();
            let disc = projection * projection - aa * cc;
            if !disc.is_finite() {
                return Err(ArrangementError::NonFiniteInput);
            }
            let error = 64.0
                * f64::EPSILON
                * (projection * projection + (aa * cc).abs() + circle.radius() * circle.radius());
            if disc.abs() <= error {
                let t = -projection / aa;
                if in_range(a, t) && in_range(b, angle(b, a.point(t))) {
                    return Err(ArrangementError::AmbiguousContact);
                }
            } else if disc > 0.0 {
                let root = disc.sqrt();
                // Stable quadratic pair avoids subtracting near-equal terms.
                let q = -projection - root.copysign(projection);
                for t in [q / aa, cc / q] {
                    work.step()?;
                    let p = a.point(t);
                    hits.push((t, angle(b, p)));
                }
            }
        }
        (Curve2D::Circle(_), Curve2D::Line(_)) => {
            return Ok(intersections(b, a, work)?
                .into_iter()
                .map(|(x, y)| (y, x))
                .collect());
        }
        (Curve2D::Circle(ca), Curve2D::Circle(cb)) => {
            let delta = cb.center() - ca.center();
            let d = delta.length();
            if same(d, 0.0) {
                if same(ca.radius(), cb.radius()) {
                    if a.range[0].max(b.range[0]) < a.range[1].min(b.range[1]) {
                        return Err(ArrangementError::AmbiguousOverlap);
                    }
                    // Adjacent arcs on one carrier have native-parameter events.
                    for x in a.range {
                        for y in b.range {
                            work.step()?;
                            if same((x - y).rem_euclid(TAU), 0.0) {
                                hits.push((x, y));
                            }
                        }
                    }
                }
            } else {
                let x = (ca.radius() * ca.radius() - cb.radius() * cb.radius() + d * d) / (2.0 * d);
                let height2 = ca.radius() * ca.radius() - x * x;
                if !height2.is_finite() || !d.is_finite() {
                    return Err(ArrangementError::NonFiniteInput);
                }
                let error = 64.0 * f64::EPSILON * (ca.radius() * ca.radius() + x * x);
                if height2.abs() <= error {
                    let p = ca.center() + delta * (x / d);
                    if in_range(a, angle(a, p)) && in_range(b, angle(b, p)) {
                        return Err(ArrangementError::AmbiguousContact);
                    }
                } else if height2 > 0.0 {
                    let base = ca.center() + delta * (x / d);
                    let perpendicular = Vec2::new(-delta.y(), delta.x()) * (height2.sqrt() / d);
                    for p in [base + perpendicular, base - perpendicular] {
                        work.step()?;
                        hits.push((angle(a, p), angle(b, p)));
                    }
                }
            }
        }
        _ => return Err(ArrangementError::UnsupportedCurve),
    }
    let mut result = Vec::new();
    for (mut x, mut y) in hits {
        work.step()?;
        if !x.is_finite() || !y.is_finite() {
            return Err(ArrangementError::NonFiniteInput);
        }
        // Shared topology/pave endpoints are independent identity certificates.
        for i in 0..2 {
            for j in 0..2 {
                work.step()?;
                if a.endpoints[i] == b.endpoints[j]
                    && (a.point(x) - a.point(a.range[i])).length() <= roundoff(a.point(x))
                    && (b.point(y) - b.point(b.range[j])).length() <= roundoff(b.point(y))
                {
                    x = a.range[i];
                    y = b.range[j];
                }
            }
        }
        if in_range(a, x) && in_range(b, y) {
            if work.uv_distance(a.point(x), b.point(y)) > work.context.tolerance.linear {
                return Err(ArrangementError::IntersectionRefinementFailed);
            }
            result.push((x, y));
        }
    }
    result.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    result.dedup_by(|a, b| same(a.0, b.0) && same(a.1, b.1));
    Ok(result)
}

pub(super) fn tangent(h: &ArrangementHalfEdge, uses: &BTreeMap<u64, &CurveUse>) -> Vec2 {
    uses[&h.source.use_id].pcurve.tangent(h.range[0]) * (h.range[1] - h.range[0]).signum()
}
pub(super) fn tangent_order(
    a: &ArrangementHalfEdge,
    b: &ArrangementHalfEdge,
    uses: &BTreeMap<u64, &CurveUse>,
) -> Ordering {
    let a = tangent(a, uses);
    let b = tangent(b, uses);
    let half = |v: Vec2| v.y() < 0.0 || (same(v.y(), 0.0) && v.x() < 0.0);
    half(a).cmp(&half(b)).then_with(|| {
        let sign = cross(a, b);
        if sign > 0.0 {
            Ordering::Less
        } else if sign < 0.0 {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    })
}

pub(super) fn integral(u: &CurveUse, range: [f64; 2], origin: Point2) -> f64 {
    let a = u.point(range[0]) - origin;
    let b = u.point(range[1]) - origin;
    match &u.pcurve {
        Curve2D::Line(_) => 0.5 * cross(a, b),
        Curve2D::Circle(c) => {
            let center = c.center() - origin;
            0.5 * (center.x() * (b.y() - a.y()) - center.y() * (b.x() - a.x())
                + c.radius() * c.radius() * (range[1] - range[0]))
        }
        _ => f64::NAN,
    }
}

/// Exact analytic horizontal ray crossing on a cardinal-split, y-monotone arc.
pub(super) fn ray_crossing(u: &CurveUse, range: [f64; 2], p: Point2) -> i32 {
    let a = u.point(range[0]);
    let b = u.point(range[1]);
    let upward = a.y() <= p.y() && b.y() > p.y();
    let downward = b.y() <= p.y() && a.y() > p.y();
    if !upward && !downward {
        return 0;
    }
    let x = match &u.pcurve {
        Curve2D::Line(_) => a.x() + (p.y() - a.y()) / (b.y() - a.y()) * (b.x() - a.x()),
        Curve2D::Circle(c) => {
            let mid = (range[0] + range[1]) * 0.5;
            let height = p.y() - c.center().y();
            c.center().x()
                + (c.radius() * c.radius() - height * height).max(0.0).sqrt() * mid.cos().signum()
        }
        _ => return 0,
    };
    if x > p.x() {
        if upward { 1 } else { -1 }
    } else {
        0
    }
}

pub(super) fn distance(u: &CurveUse, range: [f64; 2], p: Point2) -> f64 {
    let parameter = match &u.pcurve {
        Curve2D::Line(l) => l
            .project(p)
            .clamp(range[0].min(range[1]), range[0].max(range[1])),
        Curve2D::Circle(_) => {
            let t = angle(u, p);
            if t >= range[0].min(range[1]) && t <= range[0].max(range[1]) {
                t
            } else if (u.point(range[0]) - p).length() < (u.point(range[1]) - p).length() {
                range[0]
            } else {
                range[1]
            }
        }
        _ => return f64::NAN,
    };
    (u.point(parameter) - p).length()
}
