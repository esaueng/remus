//! Natural degree-(1, p) and degree-(2, p) domains witnessed by complete boundary images.

use remus_math::compact_sort::CompactSort;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;

use crate::CheckError;

fn clamped(knots: &[f64], degree: usize, range: (f64, f64)) -> bool {
    knots
        .iter()
        .take(degree + 1)
        .all(|&x| x.to_bits() == range.0.to_bits())
        && knots
            .iter()
            .rev()
            .take(degree + 1)
            .all(|&x| x.to_bits() == range.1.to_bits())
}

fn close_point(a: Point3, b: Point3, tolerance: f64) -> bool {
    (a - b).length() <= tolerance
}

fn same_curve(a: &NurbsCurve, b: &NurbsCurve, tolerance: f64) -> bool {
    a.degree() == b.degree()
        && a.knots() == b.knots()
        && a.weights() == b.weights()
        && a.control_points().len() == b.control_points().len()
        && a.control_points()
            .iter()
            .zip(b.control_points())
            .all(|(&a, &b)| close_point(a, b, tolerance))
}

/// A quadratic iso section is a minor circular arc, not an arbitrary conic.
fn circular_section_matches(
    iso: &NurbsCurve,
    circle: &remus_math::curves::Circle3D,
    range: (f64, f64),
    tolerance: f64,
) -> bool {
    let points = iso.control_points();
    let weights = iso.weights();
    if iso.degree() != 2
        || points.len() != 3
        || weights.len() != 3
        || !clamped(iso.knots(), 2, iso.domain())
        || weights[0].to_bits() != weights[2].to_bits()
        || weights[0] <= 0.0
    {
        return false;
    }
    let w = weights[1] / weights[0];
    if !(0.05..0.999_999).contains(&w) {
        return false;
    }
    let midpoint = points[0] + (points[2] - points[0]) * 0.5;
    let center = midpoint + (midpoint - points[1]) * (w * w / (1.0 - w * w));
    let r = (points[0] - center).length();
    if !r.is_finite()
        || r <= tolerance
        || (r - (points[2] - center).length()).abs() > tolerance
        || ((midpoint - center).length() - r * w).abs() > tolerance
        || !close_point(center, circle.center(), tolerance)
        || (circle.radius() - r).abs() > tolerance
    {
        return false;
    }
    let a = points[0] - center;
    let b = points[2] - center;
    let angle = a.cross(b).length().atan2(a.dot(b));
    // The ends are held to `tolerance` below, which pins the angle to about
    // `2 · tolerance / r`; an absolute 128 ulps refused the rims a glued gear
    // ring rebuilds, whose trims round 3–5e-14 off the arc's angle while
    // their ends sit within 1e-14 of it.
    if (angle - (range.1 - range.0).abs()).abs() > (4.0 * tolerance / r).max(128.0 * f64::EPSILON) {
        return false;
    }
    let normal = match a.cross(b).normalize() {
        Ok(n) => n,
        Err(_) => return false,
    };
    if normal.cross(circle.normal()).length() > 128.0 * f64::EPSILON {
        return false;
    }
    let first = circle.evaluate(range.0);
    let last = circle.evaluate(range.1);
    let ends = (close_point(first, points[0], tolerance)
        && close_point(last, points[2], tolerance))
        || (close_point(first, points[2], tolerance) && close_point(last, points[0], tolerance));
    ends && close_point(
        circle.evaluate(f64::midpoint(range.0, range.1)),
        iso.evaluate(f64::midpoint(iso.domain().0, iso.domain().1)),
        tolerance,
    )
}

/// A ruled full-turn band: two rows joined by straight rails, each row the
/// whole of one closed circle edge, and the band closed in v on one straight
/// seam whose two uses are both v-boundaries (the oblique circle loft band).
///
/// The rows are witnessed exactly: a circle edge must convert, over its own
/// full-turn domain, to the row's curve bit for bit in knots and weights and
/// within `tolerance` in control points; a NURBS edge must be that row over
/// its whole domain. Nothing here samples or projects.
pub(super) fn ruled_full_turn_is_complete(
    topo: &Topology,
    face: FaceId,
    s: &NurbsSurface,
) -> Result<bool, CheckError> {
    let f = topo.face(face)?;
    if !f.inner_wires().is_empty()
        || s.degree_u() != 1
        || s.control_points().len() != 2
        || s.degree_v() == 0
        || s.control_points()[0].len() < 3
        || s.control_points()[1].len() != s.control_points()[0].len()
        || s.is_periodic_u()
        || !s.is_periodic_v()
        || !clamped(s.knots_u(), 1, s.domain_u())
        || !clamped(s.knots_v(), s.degree_v(), s.domain_v())
    {
        return Ok(false);
    }
    let points = s.control_points();
    let weights = s.weights();
    let n = points[0].len();
    let magnitude = points
        .iter()
        .flatten()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1.0_f64, f64::max);
    let tolerance = (128.0 * f64::EPSILON * magnitude).min(1e-7);
    if points
        .iter()
        .any(|row| !close_point(row[0], row[n - 1], tolerance))
    {
        return Ok(false);
    }
    let rows = [
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[0].clone(),
            weights[0].clone(),
        )?,
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[1].clone(),
            weights[1].clone(),
        )?,
    ];
    let seam = (points[0][0], points[1][0]);
    let wire = topo.wire(f.outer_wire())?;
    if wire.edges().len() != 4
        || remus_topology::validation::validate_wire_closed(wire, topo).is_err()
    {
        return Ok(false);
    }
    let turn = std::f64::consts::TAU;
    let mut row_uses = [0_usize; 2];
    let mut seam_uses = Vec::with_capacity(2);
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let Ok(range) = e.strict_domain() else {
            return Ok(false);
        };
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        if !close_point(
            e.curve().evaluate_with_endpoints(range.0, start, end),
            start,
            tolerance,
        ) || !close_point(
            e.curve().evaluate_with_endpoints(range.1, start, end),
            end,
            tolerance,
        ) {
            return Ok(false);
        }
        let row = match e.curve() {
            EdgeCurve::Line => {
                let along =
                    close_point(start, seam.0, tolerance) && close_point(end, seam.1, tolerance);
                let against =
                    close_point(start, seam.1, tolerance) && close_point(end, seam.0, tolerance);
                if !along && !against {
                    return Ok(false);
                }
                seam_uses.push((oe.edge(), oe.is_forward()));
                continue;
            }
            EdgeCurve::Circle(circle) => {
                if e.start() != e.end()
                    || ((range.1 - range.0).abs() - turn).abs() > 128.0 * f64::EPSILON * turn
                {
                    return Ok(false);
                }
                let Ok(rim) = remus_geometry::convert::circle_to_nurbs(circle, range.0, range.1)
                else {
                    return Ok(false);
                };
                rows.iter().position(|row| same_curve(row, &rim, tolerance))
            }
            EdgeCurve::NurbsCurve(curve) => {
                let whole = curve.domain();
                if e.start() != e.end()
                    || range.0.min(range.1).to_bits() != whole.0.to_bits()
                    || range.0.max(range.1).to_bits() != whole.1.to_bits()
                {
                    return Ok(false);
                }
                rows.iter()
                    .position(|row| same_curve(row, curve, tolerance))
            }
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
        };
        let Some(row) = row else {
            return Ok(false);
        };
        row_uses[row] += 1;
    }
    Ok(row_uses == [1, 1]
        && matches!(
            seam_uses.as_slice(),
            [(first, first_forward), (second, second_forward)]
                if first == second && first_forward != second_forward
        ))
}

/// `edge` is `iso` traced the other way: degree and weights (reversed) bit
/// for bit, the mirrored knots to rounding (mirroring `k ↦ k₀ + k₁ − k` is
/// not exact in floating point), control points (reversed) within
/// `tolerance`.
fn same_curve_reversed(iso: &NurbsCurve, edge: &NurbsCurve, tolerance: f64) -> bool {
    let knots = iso.knots();
    let (Some(&first), Some(&last)) = (knots.first(), knots.last()) else {
        return false;
    };
    let knot_tolerance = 128.0 * f64::EPSILON * first.abs().max(last.abs()).max(1.0);
    iso.degree() == edge.degree()
        && knots.len() == edge.knots().len()
        && knots
            .iter()
            .rev()
            .zip(edge.knots())
            .all(|(&k, &e)| ((first + last - k) - e).abs() <= knot_tolerance)
        && iso.weights().len() == edge.weights().len()
        && iso
            .weights()
            .iter()
            .rev()
            .zip(edge.weights())
            .all(|(a, b)| a.to_bits() == b.to_bits())
        && iso.control_points().len() == edge.control_points().len()
        && iso
            .control_points()
            .iter()
            .rev()
            .zip(edge.control_points())
            .all(|(&a, &b)| close_point(a, b, tolerance))
}

/// The straight side of a degree-one iso curve with two control points: a
/// `Line` edge between exactly its two ends (either way round) traces it
/// whole.
fn line_traces_iso(iso: &NurbsCurve, start: Point3, end: Point3, tolerance: f64) -> bool {
    let points = iso.control_points();
    iso.degree() == 1
        && points.len() == 2
        && ((close_point(start, points[0], tolerance) && close_point(end, points[1], tolerance))
            || (close_point(start, points[1], tolerance) && close_point(end, points[0], tolerance)))
}

/// Any clamped, non-periodic carrier whose outer wire runs along its four
/// iso boundaries, each covered once from end to end: every edge is a NURBS
/// edge equal to a side's iso curve (knots and weights bit for bit, control
/// points within the witness tolerance) over a piece of its domain, a `Line`
/// tracing a straight side, or a circle equal to a quadratic side. Nothing
/// here samples or projects.
pub(super) fn iso_bounded_is_complete(
    topo: &Topology,
    face: FaceId,
    s: &NurbsSurface,
) -> Result<bool, CheckError> {
    let f = topo.face(face)?;
    let points = s.control_points();
    if !f.inner_wires().is_empty()
        || s.degree_u() == 0
        || s.degree_v() == 0
        || points.len() < 2
        || points[0].len() < 2
        || s.is_periodic_u()
        || s.is_periodic_v()
        || !clamped(s.knots_u(), s.degree_u(), s.domain_u())
        || !clamped(s.knots_v(), s.degree_v(), s.domain_v())
    {
        return Ok(false);
    }
    let weights = s.weights();
    let (m, n) = (points.len(), points[0].len());
    let magnitude = points
        .iter()
        .flatten()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1.0_f64, f64::max);
    let tolerance = (128.0 * f64::EPSILON * magnitude).min(1e-7);
    let iso = [
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[0].clone(),
            weights[0].clone(),
        )?,
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[m - 1].clone(),
            weights[m - 1].clone(),
        )?,
        NurbsCurve::new(
            s.degree_u(),
            s.knots_u().to_vec(),
            points.iter().map(|r| r[0]).collect(),
            weights.iter().map(|r| r[0]).collect(),
        )?,
        NurbsCurve::new(
            s.degree_u(),
            s.knots_u().to_vec(),
            points.iter().map(|r| r[n - 1]).collect(),
            weights.iter().map(|r| r[n - 1]).collect(),
        )?,
    ];
    let wire = topo.wire(f.outer_wire())?;
    if remus_topology::validation::validate_wire_closed(wire, topo).is_err() {
        return Ok(false);
    }
    let mut coverage: [Vec<(f64, f64)>; 4] = std::array::from_fn(|_| Vec::new());
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        let range = if matches!(e.curve(), EdgeCurve::Line) {
            (0.0, 1.0)
        } else {
            let Ok(range) = e.strict_domain() else {
                return Ok(false);
            };
            if !close_point(
                e.curve().evaluate_with_endpoints(range.0, start, end),
                start,
                tolerance,
            ) || !close_point(
                e.curve().evaluate_with_endpoints(range.1, start, end),
                end,
                tolerance,
            ) {
                return Ok(false);
            }
            range
        };
        let mut claims = Vec::new();
        for (side, curve) in iso.iter().enumerate() {
            match e.curve() {
                EdgeCurve::NurbsCurve(edge_curve) if same_curve(curve, edge_curve, tolerance) => {
                    let (a, b) = curve.domain();
                    let low = range.0.min(range.1);
                    let high = range.0.max(range.1);
                    if low >= a && high <= b && high > low {
                        claims.push((side, (low, high)));
                    }
                }
                EdgeCurve::NurbsCurve(edge_curve)
                    if same_curve_reversed(curve, edge_curve, tolerance) =>
                {
                    // The edge's `t` is the side's `a + b − t`.
                    let (a, b) = curve.domain();
                    let low = a + b - range.0.max(range.1);
                    let high = a + b - range.0.min(range.1);
                    let eps = 128.0 * f64::EPSILON * a.abs().max(b.abs()).max(1.0);
                    if low >= a - eps && high <= b + eps && high > low {
                        claims.push((side, (low.max(a), high.min(b))));
                    }
                }
                EdgeCurve::Line if line_traces_iso(curve, start, end, tolerance) => {
                    claims.push((side, curve.domain()));
                }
                EdgeCurve::Circle(circle)
                    if circular_section_matches(curve, circle, range, tolerance) =>
                {
                    claims.push((side, curve.domain()));
                }
                EdgeCurve::Line
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Circle(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {}
            }
        }
        // An edge two sides could claim (a collapsed or doubled side) is not
        // a witness of either.
        let [claim] = claims.as_slice() else {
            return Ok(false);
        };
        coverage[claim.0].push(claim.1);
    }
    for (side, spans) in coverage.iter_mut().enumerate() {
        spans.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let domain = iso[side].domain();
        let mut end = domain.0;
        for &(a, b) in spans.iter() {
            if (a - end).abs() > 128.0 * f64::EPSILON * domain.1.abs().max(1.0) {
                return Ok(false);
            }
            end = b;
        }
        if spans.is_empty()
            || (end - domain.1).abs() > 128.0 * f64::EPSILON * domain.1.abs().max(1.0)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

/// `a` and `b` are one point set traced whole: the same degree, weights bit
/// for bit and control points within `tolerance`, in the same order or
/// reversed, and knot vectors equal to rounding once each is mapped onto
/// `[0, 1]`. An affine change of parameter leaves a NURBS curve's points
/// where they are, so a circle converted over `[0, 1]` traces the same rim
/// as a carrier's iso curve over `[0, 2π]`.
fn same_image(a: &NurbsCurve, b: &NurbsCurve, tolerance: f64) -> bool {
    let normalized = |c: &NurbsCurve| -> Option<Vec<f64>> {
        let knots = c.knots();
        let (&first, &last) = (knots.first()?, knots.last()?);
        let span = last - first;
        (span.is_finite() && span > 0.0).then(|| knots.iter().map(|k| (k - first) / span).collect())
    };
    let (Some(ka), Some(kb)) = (normalized(a), normalized(b)) else {
        return false;
    };
    let knot_tolerance = 128.0 * f64::EPSILON;
    let (pa, pb) = (a.control_points(), b.control_points());
    let (wa, wb) = (a.weights(), b.weights());
    if a.degree() != b.degree()
        || ka.len() != kb.len()
        || pa.len() != pb.len()
        || wa.len() != wb.len()
    {
        return false;
    }
    let forward = ka
        .iter()
        .zip(&kb)
        .all(|(x, y)| (x - y).abs() <= knot_tolerance)
        && wa.iter().zip(wb).all(|(x, y)| x.to_bits() == y.to_bits())
        && pa
            .iter()
            .zip(pb)
            .all(|(&x, &y)| close_point(x, y, tolerance));
    forward
        || (ka
            .iter()
            .rev()
            .zip(&kb)
            .all(|(x, y)| ((1.0 - x) - y).abs() <= knot_tolerance)
            && wa
                .iter()
                .rev()
                .zip(wb)
                .all(|(x, y)| x.to_bits() == y.to_bits())
            && pa
                .iter()
                .rev()
                .zip(pb)
                .all(|(&x, &y)| close_point(x, y, tolerance)))
}

/// `edge`, over `range`, traces `iso` whole, either way round.
fn traces_whole(iso: &NurbsCurve, edge: &NurbsCurve, range: (f64, f64), tolerance: f64) -> bool {
    let whole = edge.domain();
    let eps = 128.0 * f64::EPSILON * whole.0.abs().max(whole.1.abs()).max(1.0);
    let covers = (range.0.min(range.1) - whole.0).abs() <= eps
        && (range.0.max(range.1) - whole.1).abs() <= eps;
    covers && same_image(iso, edge, tolerance)
}

/// A band closed in exactly one parameter direction (an in-place helical
/// sweep of a circle, a full-turn swept or lofted wall): each end of the open
/// direction is one closed rim edge tracing that end's iso curve whole, and
/// the band closes on one seam edge, used once each way, tracing the closing
/// iso curve whole. Rims are circle edges converted over their own full-turn
/// domain or NURBS edges, compared as [`ruled_full_turn_is_complete`] does;
/// a seam is a NURBS edge or a `Line` along a straight seam. Nothing here
/// samples or projects.
pub(super) fn periodic_band_is_complete(
    topo: &Topology,
    face: FaceId,
    s: &NurbsSurface,
) -> Result<bool, CheckError> {
    let f = topo.face(face)?;
    let points = s.control_points();
    let closed_u = s.is_periodic_u();
    if !f.inner_wires().is_empty()
        || closed_u == s.is_periodic_v()
        || s.degree_u() == 0
        || s.degree_v() == 0
        || points.len() < 2
        || points[0].len() < 2
        || !clamped(s.knots_u(), s.degree_u(), s.domain_u())
        || !clamped(s.knots_v(), s.degree_v(), s.domain_v())
    {
        return Ok(false);
    }
    let weights = s.weights();
    let (m, n) = (points.len(), points[0].len());
    let magnitude = points
        .iter()
        .flatten()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1.0_f64, f64::max);
    let tolerance = (128.0 * f64::EPSILON * magnitude).min(1e-7);
    let row = |i: usize| {
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[i].clone(),
            weights[i].clone(),
        )
    };
    let column = |j: usize| {
        NurbsCurve::new(
            s.degree_u(),
            s.knots_u().to_vec(),
            points.iter().map(|r| r[j]).collect(),
            weights.iter().map(|r| r[j]).collect(),
        )
    };
    // Closed in u: the rims are the columns at the ends of v, the seam the
    // row at u's start (which, closed, is also the row at its end).
    let (rims, seam) = if closed_u {
        ([column(0)?, column(n - 1)?], row(0)?)
    } else {
        ([row(0)?, row(m - 1)?], column(0)?)
    };
    let wire = topo.wire(f.outer_wire())?;
    if remus_topology::validation::validate_wire_closed(wire, topo).is_err() {
        return Ok(false);
    }
    let turn = std::f64::consts::TAU;
    let mut rim_uses = [0_usize; 2];
    let mut seam_uses = Vec::with_capacity(2);
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        if matches!(e.curve(), EdgeCurve::Line) {
            if !line_traces_iso(&seam, start, end, tolerance) {
                return Ok(false);
            }
            seam_uses.push((oe.edge(), oe.is_forward()));
            continue;
        }
        let Ok(range) = e.strict_domain() else {
            return Ok(false);
        };
        let rim = match e.curve() {
            EdgeCurve::Circle(circle) => {
                if e.start() != e.end()
                    || ((range.1 - range.0).abs() - turn).abs() > 128.0 * f64::EPSILON * turn
                {
                    return Ok(false);
                }
                let Ok(converted) =
                    remus_geometry::convert::circle_to_nurbs(circle, range.0, range.1)
                else {
                    return Ok(false);
                };
                rims.iter()
                    .position(|rim| same_image(rim, &converted, tolerance))
            }
            EdgeCurve::NurbsCurve(curve) => {
                if e.start() == e.end() {
                    rims.iter()
                        .position(|rim| traces_whole(rim, curve, range, tolerance))
                } else {
                    if !traces_whole(&seam, curve, range, tolerance) {
                        return Ok(false);
                    }
                    seam_uses.push((oe.edge(), oe.is_forward()));
                    continue;
                }
            }
            EdgeCurve::Line
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => None,
        };
        let Some(rim) = rim else {
            return Ok(false);
        };
        rim_uses[rim] += 1;
    }
    Ok(rim_uses == [1, 1]
        && matches!(
            seam_uses.as_slice(),
            [(first, first_forward), (second, second_forward)]
                if first == second && first_forward != second_forward
        ))
}

/// Complete side coverage is a domain witness; no sampled projection decides it.
pub(super) fn is_complete(
    topo: &Topology,
    face: FaceId,
    s: &NurbsSurface,
) -> Result<bool, CheckError> {
    if s.degree_u() == 1 {
        return ruled_full_turn_is_complete(topo, face, s);
    }
    let f = topo.face(face)?;
    if !f.inner_wires().is_empty()
        || s.degree_u() != 2
        || s.degree_v() == 0
        || s.control_points().len() != 3
        || s.control_points()[0].len() < 2
        || s.is_periodic_u()
        || !clamped(s.knots_u(), 2, s.domain_u())
        || !clamped(s.knots_v(), s.degree_v(), s.domain_v())
    {
        return Ok(false);
    }
    let points = s.control_points();
    let weights = s.weights();
    let n = points[0].len();
    let magnitude = points
        .iter()
        .flatten()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1.0_f64, f64::max);
    let tolerance = (128.0 * f64::EPSILON * magnitude).min(1e-7);
    let iso = [
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[0].clone(),
            weights[0].clone(),
        )?,
        NurbsCurve::new(
            s.degree_v(),
            s.knots_v().to_vec(),
            points[2].clone(),
            weights[2].clone(),
        )?,
        NurbsCurve::new(
            2,
            s.knots_u().to_vec(),
            points.iter().map(|r| r[0]).collect(),
            weights.iter().map(|r| r[0]).collect(),
        )?,
        NurbsCurve::new(
            2,
            s.knots_u().to_vec(),
            points.iter().map(|r| r[n - 1]).collect(),
            weights.iter().map(|r| r[n - 1]).collect(),
        )?,
    ];
    let mut coverage: [Vec<(f64, f64)>; 4] = std::array::from_fn(|_| Vec::new());
    let wire = topo.wire(f.outer_wire())?;
    if remus_topology::validation::validate_wire_closed(wire, topo).is_err() {
        return Ok(false);
    }
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let Ok(range) = e.strict_domain() else {
            return Ok(false);
        };
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        if !close_point(
            e.curve().evaluate_with_endpoints(range.0, start, end),
            start,
            tolerance,
        ) || !close_point(
            e.curve().evaluate_with_endpoints(range.1, start, end),
            end,
            tolerance,
        ) {
            return Ok(false);
        }
        let mut claims = Vec::new();
        for (side, curve) in iso.iter().enumerate() {
            match e.curve() {
                EdgeCurve::NurbsCurve(edge_curve) if same_curve(curve, edge_curve, tolerance) => {
                    let (a, b) = curve.domain();
                    let low = range.0.min(range.1);
                    let high = range.0.max(range.1);
                    if low >= a && high <= b && high > low {
                        claims.push((side, (low, high)));
                    }
                }
                EdgeCurve::Circle(circle)
                    if side >= 2 && circular_section_matches(curve, circle, range, tolerance) =>
                {
                    claims.push((side, curve.domain()));
                }
                EdgeCurve::Line
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Circle(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {}
            }
        }
        // A full-turn band (periodic in v) closes on one seam section: its
        // two v-boundaries are the same arc, and the seam edge's two uses
        // cover them one each.
        let claim = match claims.as_slice() {
            [only] => *only,
            [(2, first), (3, second)] if s.is_periodic_v() => {
                if coverage[2].is_empty() {
                    (2, *first)
                } else {
                    (3, *second)
                }
            }
            _ => return Ok(false),
        };
        coverage[claim.0].push(claim.1);
    }
    for (side, spans) in coverage.iter_mut().enumerate() {
        spans.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let domain = iso[side].domain();
        let mut end = domain.0;
        for &(a, b) in spans.iter() {
            if (a - end).abs() > 128.0 * f64::EPSILON * domain.1.abs().max(1.0) {
                return Ok(false);
            }
            end = b;
        }
        if spans.is_empty()
            || (end - domain.1).abs() > 128.0 * f64::EPSILON * domain.1.abs().max(1.0)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::cast_precision_loss
    )]
    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::Edge;
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    fn quarter_cylinder(split: bool) -> (Topology, FaceId, NurbsSurface) {
        let mut t = Topology::new();
        let n = 514;
        let z = |j: usize| 4.0 * j as f64 / (n - 1) as f64;
        let cp = vec![
            (0..n).map(|j| Point3::new(2.0, 0.0, z(j))).collect(),
            (0..n).map(|j| Point3::new(2.0, 2.0, z(j))).collect(),
            (0..n).map(|j| Point3::new(0.0, 2.0, z(j))).collect(),
        ];
        let weights = vec![
            vec![1.0; n],
            vec![std::f64::consts::FRAC_1_SQRT_2; n],
            vec![1.0; n],
        ];
        let mut kv = vec![0.0, 0.0];
        kv.extend((1..n - 1).map(|j| j as f64 / (n - 1) as f64));
        kv.extend([1.0, 1.0]);
        let surface = NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            kv.clone(),
            cp.clone(),
            weights.clone(),
        )
        .unwrap();
        let points = [
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 4.0),
            Point3::new(2.0, 0.0, 4.0),
        ];
        let v: Vec<_> = points
            .into_iter()
            .map(|p| t.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let mut add = |a, b, curve, range| {
            let mut e = Edge::new(a, b, curve);
            e.set_trim(Some(range));
            t.add_edge(e)
        };
        let bottom = add(
            v[0],
            v[1],
            EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
            (0.0, std::f64::consts::FRAC_PI_2),
        );
        let right = add(
            v[1],
            v[2],
            EdgeCurve::NurbsCurve(
                NurbsCurve::new(1, kv.clone(), cp[2].clone(), weights[2].clone()).unwrap(),
            ),
            (0.0, 1.0),
        );
        let top = add(
            v[3],
            v[2],
            EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, 4.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
            (0.0, std::f64::consts::FRAC_PI_2),
        );
        let curve = NurbsCurve::new(1, kv, cp[0].clone(), weights[0].clone()).unwrap();
        let mut wire = vec![
            OrientedEdge::new(bottom, true),
            OrientedEdge::new(right, true),
            OrientedEdge::new(top, false),
        ];
        if split {
            let middle = t.add_vertex(Vertex::new(curve.evaluate(0.37), 1e-7));
            let mut a = Edge::new(v[0], middle, EdgeCurve::NurbsCurve(curve.clone()));
            a.set_trim(Some((0.0, 0.37)));
            let a = t.add_edge(a);
            let mut b = Edge::new(middle, v[3], EdgeCurve::NurbsCurve(curve));
            b.set_trim(Some((0.37, 1.0)));
            let b = t.add_edge(b);
            wire.extend([OrientedEdge::new(b, false), OrientedEdge::new(a, false)]);
        } else {
            let mut e = Edge::new(v[0], v[3], EdgeCurve::NurbsCurve(curve));
            e.set_trim(Some((0.0, 1.0)));
            wire.push(OrientedEdge::new(t.add_edge(e), false));
        }
        let wire = t.add_wire(Wire::new(wire, true).unwrap());
        let face = t.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
        (t, face, surface)
    }

    #[test]
    fn dense_natural_rectangle_and_split_row_match_cylinder_area() {
        for split in [false, true] {
            let (t, face, s) = quarter_cylinder(split);
            assert!(is_complete(&t, face, &s).unwrap());
            let area = super::super::integrate_face_area(&t, face, 5).unwrap();
            assert!((area - 4.0 * std::f64::consts::PI).abs() < 1e-8, "{area}");
        }
    }

    #[test]
    fn holes_partial_sections_and_changed_row_carriers_decline() {
        let (mut t, face, s) = quarter_cylinder(false);
        let outer = t.face(face).unwrap().outer_wire();
        t.set_face_boundary_wires(face, outer, vec![outer]).unwrap();
        assert!(!is_complete(&t, face, &s).unwrap());
        t.set_face_boundary_wires(face, outer, vec![]).unwrap();
        let edge = t.wire(outer).unwrap().edges()[0].edge();
        t.edge_mut(edge)
            .unwrap()
            .set_trim(Some((0.1, std::f64::consts::FRAC_PI_2)));
        assert!(!is_complete(&t, face, &s).unwrap());
        let (t, face, s) = quarter_cylinder(false);
        let mut points = s.control_points().to_vec();
        points[0][2] = points[0][2] + Vec3::new(0.02, 0.0, 0.0);
        let changed = NurbsSurface::new(
            2,
            1,
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            points,
            s.weights().to_vec(),
        )
        .unwrap();
        assert!(!is_complete(&t, face, &changed).unwrap());
    }

    /// The oblique circle loft band: rows are full turns of two parallel
    /// circles sharing a reference direction, closed on one straight seam.
    fn oblique_band(rim_range: (f64, f64)) -> (Topology, FaceId, NurbsSurface) {
        let mut t = Topology::new();
        let z = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        let lower = Circle3D::new_with_ref(Point3::new(0.0, 0.0, 0.0), z, 25.0, x).unwrap();
        let upper = Circle3D::new_with_ref(Point3::new(12.0, 0.0, 60.0), z, 10.0, x).unwrap();
        let turn = std::f64::consts::TAU;
        let rims = [&upper, &lower]
            .map(|c| remus_geometry::convert::circle_to_nurbs(c, 0.0, turn).unwrap());
        let surface = NurbsSurface::new(
            1,
            2,
            vec![0.0, 0.0, 1.0, 1.0],
            rims[0].knots().to_vec(),
            rims.iter().map(|r| r.control_points().to_vec()).collect(),
            rims.iter().map(|r| r.weights().to_vec()).collect(),
        )
        .unwrap();
        let low = t.add_vertex(Vertex::new(lower.evaluate(0.0), 1e-7));
        let high = t.add_vertex(Vertex::new(upper.evaluate(0.0), 1e-7));
        let mut ring = |v, c: &Circle3D, range| {
            let mut e = Edge::new(v, v, EdgeCurve::Circle(c.clone()));
            e.set_trim(Some(range));
            t.add_edge(e)
        };
        let bottom = ring(low, &lower, rim_range);
        let top = ring(high, &upper, (0.0, turn));
        let seam = t.add_edge(Edge::new(low, high, EdgeCurve::Line));
        let wire = t.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(bottom, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(top, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = t.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
        (t, face, surface)
    }

    #[test]
    fn ruled_full_turn_band_is_its_natural_rectangle() {
        let turn = std::f64::consts::TAU;
        let (t, face, s) = oblique_band((0.0, turn));
        assert!(is_complete(&t, face, &s).unwrap());

        // A rim that is not the row's whole turn is not a domain witness.
        let (t, face, s) = oblique_band((0.0, turn - 0.25));
        assert!(!is_complete(&t, face, &s).unwrap());

        // Rows that moved off the stored rims decline.
        let (t, face, s) = oblique_band((0.0, turn));
        let mut points = s.control_points().to_vec();
        points[1][3] = points[1][3] + Vec3::new(0.0, 0.0, 0.01);
        let changed = NurbsSurface::new(
            1,
            2,
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            points,
            s.weights().to_vec(),
        )
        .unwrap();
        assert!(!is_complete(&t, face, &changed).unwrap());

        // A hole leaves the rectangle incomplete.
        let (mut t, face, s) = oblique_band((0.0, turn));
        let outer = t.face(face).unwrap().outer_wire();
        t.set_face_boundary_wires(face, outer, vec![outer]).unwrap();
        assert!(!is_complete(&t, face, &s).unwrap());
    }

    /// A straight cubic B-spline profile of `spans` spans along x (uneven
    /// speed, collinear control points), swept up z by `height`: a flat
    /// rectangle `length × height` stored as a degree-(3, 1) carrier bounded
    /// by its own four iso sides (two NURBS rows, two lines). `top_reversed`
    /// stores the top edge's curve traced the other way.
    fn straight_sheet(
        spans: usize,
        top_reversed: bool,
        top_range: (f64, f64),
    ) -> (Topology, FaceId, NurbsSurface, f64) {
        const HEIGHT: f64 = 2.5;
        let mut t = Topology::new();
        let n = spans + 3;
        let mut knots = vec![0.0; 4];
        knots.extend((1..spans).map(|i| (i as f64 / spans as f64).powf(1.3)));
        knots.extend([1.0; 4]);
        let xs: Vec<f64> = (0..n)
            .map(|i| 3.0 * (i as f64 / (n - 1) as f64).powi(2))
            .collect();
        let base: Vec<Point3> = xs.iter().map(|&x| Point3::new(x, 1.0, 0.0)).collect();
        let lift = |p: Point3| Point3::new(p.x(), p.y(), HEIGHT);
        let surface = NurbsSurface::new(
            3,
            1,
            knots.clone(),
            vec![0.0, 0.0, 1.0, 1.0],
            base.iter().map(|&p| vec![p, lift(p)]).collect(),
            vec![vec![1.0, 1.0]; n],
        )
        .unwrap();
        let bottom = NurbsCurve::new(3, knots.clone(), base.clone(), vec![1.0; n]).unwrap();
        let top = NurbsCurve::new(
            3,
            knots,
            base.iter().map(|&p| lift(p)).collect(),
            vec![1.0; n],
        )
        .unwrap();
        let corners = [
            bottom.evaluate(0.0),
            bottom.evaluate(1.0),
            top.evaluate(1.0),
            top.evaluate(top_range.0),
        ];
        let v: Vec<_> = corners
            .iter()
            .map(|&p| t.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let mut bottom_edge = Edge::new(v[0], v[1], EdgeCurve::NurbsCurve(bottom));
        bottom_edge.set_trim(Some((0.0, 1.0)));
        let bottom_edge = t.add_edge(bottom_edge);
        // Stored either way round, the top edge runs from v[2] to v[3].
        let (top_curve, top_trim) = if top_reversed {
            (top.reversed(), (1.0 - top_range.1, 1.0 - top_range.0))
        } else {
            (top, top_range)
        };
        let (a, b) = if top_reversed {
            (v[2], v[3])
        } else {
            (v[3], v[2])
        };
        let mut top_edge = Edge::new(a, b, EdgeCurve::NurbsCurve(top_curve));
        top_edge.set_trim(Some(top_trim));
        let top_edge = t.add_edge(top_edge);
        let right = t.add_edge(Edge::new(v[1], v[2], EdgeCurve::Line));
        let left = t.add_edge(Edge::new(v[3], v[0], EdgeCurve::Line));
        let wire = t.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(bottom_edge, true),
                    OrientedEdge::new(right, true),
                    OrientedEdge::new(top_edge, top_reversed),
                    OrientedEdge::new(left, true),
                ],
                true,
            )
            .unwrap(),
        );
        let face = t.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
        (t, face, surface, xs[n - 1] * HEIGHT)
    }

    #[test]
    fn iso_bounded_sheet_is_its_natural_rectangle_either_way_round() {
        for spans in [1, 7, 40] {
            for top_reversed in [false, true] {
                let (t, face, s, area) = straight_sheet(spans, top_reversed, (0.0, 1.0));
                assert!(!is_complete(&t, face, &s).unwrap());
                assert!(
                    iso_bounded_is_complete(&t, face, &s).unwrap(),
                    "{spans} spans, reversed {top_reversed}"
                );
                let measured = super::super::integrate_face_area(&t, face, 5).unwrap();
                assert!(
                    (measured - area).abs() <= 1e-12 * area,
                    "{spans} spans, reversed {top_reversed}: {measured} vs {area}"
                );
            }
        }
    }

    #[test]
    fn iso_bounded_declines_partial_sides_holes_and_moved_carriers() {
        // A top edge stopping short of the side's end leaves the rectangle
        // incomplete, either way round.
        for top_reversed in [false, true] {
            let (t, face, s, _) = straight_sheet(7, top_reversed, (0.1, 1.0));
            assert!(!iso_bounded_is_complete(&t, face, &s).unwrap());
        }
        // A hole.
        let (mut t, face, s, _) = straight_sheet(7, false, (0.0, 1.0));
        let outer = t.face(face).unwrap().outer_wire();
        t.set_face_boundary_wires(face, outer, vec![outer]).unwrap();
        assert!(!iso_bounded_is_complete(&t, face, &s).unwrap());
        // A carrier whose bottom row moved off the stored edge.
        let (t, face, s, _) = straight_sheet(7, false, (0.0, 1.0));
        let mut points = s.control_points().to_vec();
        points[3][0] = points[3][0] + Vec3::new(0.0, 0.01, 0.0);
        let moved = NurbsSurface::new(
            3,
            1,
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            points,
            s.weights().to_vec(),
        )
        .unwrap();
        assert!(!iso_bounded_is_complete(&t, face, &moved).unwrap());
    }

    /// A radius-2 cylinder wall of height 3 stored as a degree-(2, 1) carrier
    /// closed in `u` (the circle's rows) and bounded by two whole circle rims
    /// and one straight seam used each way. The rims are converted over
    /// `[0, 2π]` but the carrier's `u` knots run over `[0, 1]`.
    fn closed_band(rim_range: (f64, f64)) -> (Topology, FaceId, NurbsSurface) {
        let mut t = Topology::new();
        let z = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        let lower = Circle3D::new_with_ref(Point3::new(0.0, 0.0, 0.0), z, 2.0, x).unwrap();
        let upper = Circle3D::new_with_ref(Point3::new(0.0, 0.0, 3.0), z, 2.0, x).unwrap();
        let turn = std::f64::consts::TAU;
        let rims = [&lower, &upper]
            .map(|c| remus_geometry::convert::circle_to_nurbs(c, 0.0, turn).unwrap());
        let k = rims[0].knots();
        let span = k[k.len() - 1] - k[0];
        let surface = NurbsSurface::new(
            2,
            1,
            k.iter().map(|x| (x - k[0]) / span).collect(),
            vec![0.0, 0.0, 1.0, 1.0],
            (0..rims[0].control_points().len())
                .map(|i| vec![rims[0].control_points()[i], rims[1].control_points()[i]])
                .collect(),
            (0..rims[0].weights().len())
                .map(|i| vec![rims[0].weights()[i], rims[1].weights()[i]])
                .collect(),
        )
        .unwrap();
        let low = t.add_vertex(Vertex::new(lower.evaluate(0.0), 1e-7));
        let high = t.add_vertex(Vertex::new(upper.evaluate(0.0), 1e-7));
        let mut ring = |v, c: &Circle3D, range| {
            let mut e = Edge::new(v, v, EdgeCurve::Circle(c.clone()));
            e.set_trim(Some(range));
            t.add_edge(e)
        };
        let bottom = ring(low, &lower, rim_range);
        let top = ring(high, &upper, (0.0, turn));
        let seam = t.add_edge(Edge::new(low, high, EdgeCurve::Line));
        let wire = t.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(bottom, true),
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(top, false),
                    OrientedEdge::new(seam, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = t.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
        (t, face, surface)
    }

    #[test]
    fn closed_band_is_its_natural_rectangle() {
        let turn = std::f64::consts::TAU;
        let (t, face, s) = closed_band((0.0, turn));
        assert!(s.is_periodic_u() && !s.is_periodic_v());
        assert!(!is_complete(&t, face, &s).unwrap());
        assert!(periodic_band_is_complete(&t, face, &s).unwrap());
        let area = super::super::integrate_face_area(&t, face, 5).unwrap();
        let exact = 2.0 * std::f64::consts::PI * 2.0 * 3.0;
        assert!((area - exact).abs() <= 1e-9 * exact, "{area} vs {exact}");
    }

    #[test]
    fn closed_band_declines_partial_rims_holes_and_moved_carriers() {
        let turn = std::f64::consts::TAU;
        let (t, face, s) = closed_band((0.0, turn - 0.25));
        assert!(!periodic_band_is_complete(&t, face, &s).unwrap());

        let (mut t, face, s) = closed_band((0.0, turn));
        let outer = t.face(face).unwrap().outer_wire();
        t.set_face_boundary_wires(face, outer, vec![outer]).unwrap();
        assert!(!periodic_band_is_complete(&t, face, &s).unwrap());

        let (t, face, s) = closed_band((0.0, turn));
        let mut points = s.control_points().to_vec();
        points[4][1] = points[4][1] + Vec3::new(0.0, 0.0, 0.01);
        let moved = NurbsSurface::new(
            2,
            1,
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            points,
            s.weights().to_vec(),
        )
        .unwrap();
        assert!(!periodic_band_is_complete(&t, face, &moved).unwrap());
    }
}
