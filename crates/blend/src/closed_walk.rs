//! Cubic contact geometry for a closed walked stripe, measured on its
//! supports.
//!
//! The walker's adaptive march leaves a closed rim with a few dozen stations.
//! Joining them with straight chords (a degree-1 contact) puts the contact
//! edge up to its chord sagitta off its support (0.09 mm on an r 6 boss with
//! 20 chords) while the edge records the 1e-7 linear tolerance. The display
//! tessellator then rightly refuses the hole the chords bound, and its
//! fallback cracks the support's other seams (esaueng/remus#1561).
//!
//! [`fit_closed_walk`] instead re-solves the rolling ball at stations spread
//! evenly along the spine and joins them with cubic Hermite spans, on all
//! three rows of the blend surface at once. The spine is cut into segments
//! at its start and at every chain vertex where its tangent turns (a boolean
//! rim's edges can meet at a fraction of a degree): within a segment the
//! spans share a station spacing and the curve is C¹ (double knots), with
//! fourth-order central differences for the station tangents and one-sided
//! ones at the segment ends; at a turning vertex the knot is tripled, so the
//! curve keeps the corner the rolling ball really has there. The rows are
//! fitted in homogeneous coordinates: at every station the surface's
//! cross-section is that station's exact ball arc, and the two boundary
//! rows, whose weights stay exactly one, are bit-for-bit the stored contact
//! curves. Each contact's pcurve is the same construction in its support's
//! (unwrapped) parameter plane.
//!
//! The fit then measures, on [`SAMPLES_PER_SPAN`] points of every span, how
//! far each contact lies from its support (closest point) and from its
//! pcurve's image. It doubles the station count until that deviation is
//! within half the linear tolerance the contact edges record. When
//! [`MAX_STATIONS`] cannot reach it (a rim whose walk is not smooth
//! everywhere, such as a cross-drilled hole's), it returns the fit only
//! within [`recordable_deviation`], for a caller that records the deviation
//! as the edges' tolerance, and otherwise refuses the stripe. The measurement
//! samples; it is not an interval certificate. The supports must project
//! exactly, so callers use it only for analytic supports.

use remus_math::compact_sort::CompactSort;
use remus_math::curves2d::{Curve2D, NurbsCurve2D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point2, Point3};

use crate::BlendError;
use crate::blend_func::{BlendFunction, BlendParams};
use crate::section::CircSection;
use crate::walker::{Walker, section_arc};

/// Station count of the first fit.
const FIRST_STATIONS: usize = 64;
/// Largest station count tried before the stripe is refused.
pub const MAX_STATIONS: usize = 1024;
/// Largest deviation a fit that [`MAX_STATIONS`] cannot bring within the
/// budget may return, for a rim of arc length `length`: `1e-5`, or a
/// millionth of the rim on a larger one (the deviation a corner leaves
/// scales with the body). Its caller must record it as the contact edges'
/// tolerance (the closed-rim assembly measures and records it).
pub fn recordable_deviation(length: f64) -> f64 {
    (1e-6 * length).max(1e-5)
}
/// Interior points per span at which the deviation is measured.
pub const SAMPLES_PER_SPAN: usize = 7;
/// Fewest spans in one segment: the one-sided differences read five
/// stations.
const MIN_SEGMENT_SPANS: usize = 4;
/// Tangent turn (radians) above which a chain vertex is a corner.
const CORNER_TURN: f64 = 1e-6;

/// A closed walked stripe's resampled geometry (see the
/// [module docs](self)).
#[derive(Debug)]
pub struct ClosedWalkFit {
    /// The fitted stations; the last repeats the first at `t = 1`.
    pub sections: Vec<CircSection>,
    /// Rational quadratic in U, cubic through the stations in V.
    pub surface: NurbsSurface,
    /// Contact on the first support: the surface's `u = 0` boundary.
    pub contact1: NurbsCurve,
    /// Contact on the second support: the surface's `u = 1` boundary.
    pub contact2: NurbsCurve,
    /// `contact1` in the first support's parameter plane.
    pub pcurve1: Curve2D,
    /// `contact2` in the second support's parameter plane.
    pub pcurve2: Curve2D,
    /// Largest measured distance of a contact from its support or from its
    /// pcurve's image.
    pub deviation: f64,
}

/// One support of the stripe: the surface the walker solved on and its
/// parameter periods `(u, v)`.
pub type Support<'a> = (&'a dyn ParametricSurface, (Option<f64>, Option<f64>));

/// Resample the closed walk `walked` (see the [module docs](self)).
///
/// # Errors
///
/// [`BlendError::WalkingFailure`] when a station does not converge onto the
/// walked branch, or, with the measured deviation as its `residual`, when
/// [`MAX_STATIONS`] stations still leave a contact further from its support
/// than [`recordable_deviation`]; [`BlendError::Math`] when a curve or
/// surface cannot be built, and governor refusals.
pub fn fit_closed_walk<F: BlendFunction>(
    walker: &Walker<'_, F>,
    walked: &[CircSection],
    supports: [Support<'_>; 2],
) -> Result<ClosedWalkFit, BlendError> {
    let budget = 0.5 * Tolerance::new().linear;
    let breaks = segment_breaks(walker)?;
    // Powers of two keep evenly spread stations exact in binary.
    let mut stations = FIRST_STATIONS.max(walked.len().saturating_sub(1).next_power_of_two());
    loop {
        let fit = fit_with_stations(walker, walked, supports, &breaks, stations)?;
        if fit.deviation <= budget {
            return Ok(fit);
        }
        if stations >= MAX_STATIONS {
            log::debug!(
                target: "remus_approx",
                "closed walk: {stations} stations leave the contacts {:.3e} off their supports",
                fit.deviation
            );
            if fit.deviation <= recordable_deviation(walker.spine().length()) {
                return Ok(fit);
            }
            return Err(walk_failure(walker, 0.0, fit.deviation));
        }
        stations *= 2;
    }
}

fn walk_failure<F: BlendFunction>(walker: &Walker<'_, F>, t: f64, residual: f64) -> BlendError {
    BlendError::WalkingFailure {
        edge: walker.spine().edges()[0],
        t,
        residual,
    }
}

/// Normalized stations where a segment starts: `0` and every chain vertex
/// where the spine's tangent turns by more than [`CORNER_TURN`], ascending.
fn segment_breaks<F: BlendFunction>(walker: &Walker<'_, F>) -> Result<Vec<f64>, BlendError> {
    let spine = walker.spine();
    let length = spine.length();
    let mut breaks = vec![0.0];
    if length <= 0.0 {
        return Ok(breaks);
    }
    let probe = 1e-7 * length;
    for index in 0..spine.edge_count() {
        let Some(entry) = spine.edge_entry_station(index) else {
            continue;
        };
        let station = (entry - spine.origin()).rem_euclid(length);
        if station <= probe || station >= length - probe {
            continue;
        }
        let before = spine.tangent(walker.topo(), station - probe)?;
        let after = spine.tangent(walker.topo(), station + probe)?;
        let turn = before.cross(after).length().atan2(before.dot(after));
        if turn > CORNER_TURN {
            breaks.push(station / length);
        }
    }
    breaks.compact_sort_by(f64::total_cmp);
    breaks.dedup();
    Ok(breaks)
}

const fn params_of(section: &CircSection) -> BlendParams {
    BlendParams {
        u1: section.uv1.0,
        v1: section.uv1.1,
        u2: section.uv2.0,
        v2: section.uv2.1,
    }
}

/// Ball centre of the walked polyline at normalized station `t`, and the
/// length of the walked chord it lies on.
fn walked_center(walked: &[CircSection], t: f64) -> Option<(Point3, f64)> {
    let next = walked.iter().position(|section| section.t >= t)?;
    let Some(previous) = next.checked_sub(1) else {
        return Some((walked[0].center, 0.0));
    };
    let (a, b) = (&walked[previous], &walked[next]);
    let span = b.t - a.t;
    let f = if span > 0.0 { (t - a.t) / span } else { 0.0 };
    Some((
        a.center + (b.center - a.center) * f,
        (b.center - a.center).length(),
    ))
}

/// The station layout: every station's normalized parameter, and for each
/// segment the index range of its stations (ends included, so neighbouring
/// segments share their break station).
struct Layout {
    stations: Vec<f64>,
    segments: Vec<(usize, usize)>,
}

/// About `n` stations spread over the segments in proportion to their
/// length, each segment evenly and with at least [`MIN_SEGMENT_SPANS`]
/// spans. The last station is `1`.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn layout(breaks: &[f64], n: usize) -> Layout {
    let mut stations = Vec::with_capacity(n + breaks.len() * MIN_SEGMENT_SPANS + 1);
    let mut segments = Vec::with_capacity(breaks.len());
    for (index, &start) in breaks.iter().enumerate() {
        let end = breaks.get(index + 1).copied().unwrap_or(1.0);
        let spans = ((end - start) * n as f64)
            .ceil()
            .max(MIN_SEGMENT_SPANS as f64) as usize;
        let first = stations.len();
        for k in 0..spans {
            stations.push((end - start).mul_add(k as f64 / spans as f64, start));
        }
        segments.push((first, first + spans));
    }
    stations.push(1.0);
    Layout { stations, segments }
}

#[allow(clippy::too_many_lines)]
fn fit_with_stations<F: BlendFunction>(
    walker: &Walker<'_, F>,
    walked: &[CircSection],
    supports: [Support<'_>; 2],
    breaks: &[f64],
    n: usize,
) -> Result<ClosedWalkFit, BlendError> {
    let first = walked
        .first()
        .ok_or(BlendError::Math(remus_math::MathError::EmptyInput))?;
    let length = walker.spine().length();
    let layout = layout(breaks, n);
    let count = layout.stations.len();
    let mut params: Vec<BlendParams> = Vec::with_capacity(count);
    let mut sections: Vec<CircSection> = Vec::with_capacity(count);
    for (k, &t) in layout.stations[..count - 1].iter().enumerate() {
        crate::governor::checkpoint()?;
        let guess = match k {
            0 => params_of(first),
            1 => params[0],
            _ => {
                // Linear extrapolation over the last two stations, whatever
                // their spacing.
                let (a, b) = (params[k - 2], params[k - 1]);
                let (ta, tb) = (layout.stations[k - 2], layout.stations[k - 1]);
                let ratio = (t - tb) / (tb - ta);
                BlendParams {
                    u1: (b.u1 - a.u1).mul_add(ratio, b.u1),
                    v1: (b.v1 - a.v1).mul_add(ratio, b.v1),
                    u2: (b.u2 - a.u2).mul_add(ratio, b.u2),
                    v2: (b.v2 - a.v2).mul_add(ratio, b.v2),
                }
            }
        };
        let expected = walked_center(walked, t);
        // On the walked branch: within half a ball radius of the walked
        // centre polyline, or within its chord where the walk's stations are
        // too sparse for its chords to follow the centre's path.
        // A chamfer's straight section has no ball: its reach is half its
        // chord instead.
        let on_branch = |section: &CircSection| {
            let reach = if section.radius == 0.0 {
                0.5 * (section.p2 - section.p1).length()
            } else {
                0.5 * section.radius.abs()
            };
            expected.is_some_and(|(center, chord)| {
                (section.center - center).length() <= reach.max(chord)
            })
        };
        let solved = walker
            .solve_section_polished(length * t, Some(guess))
            .filter(|(_, section)| on_branch(section))
            .or_else(|| {
                walker
                    .solve_section_polished(length * t, None)
                    .filter(|(_, section)| on_branch(section))
            })
            .or_else(|| {
                // The walk's own nearest station: across a spine corner the
                // ball's section turns with the spine, and neither the
                // extrapolation nor the guide-point projection lands on it.
                let nearest = walked
                    .iter()
                    .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()))?;
                walker
                    .solve_section_polished(length * t, Some(params_of(nearest)))
                    .filter(|(_, section)| on_branch(section))
            })
            .ok_or_else(|| walk_failure(walker, t, f64::NAN))?;
        params.push(solved.0);
        sections.push(solved.1);
    }
    // The walk closes on its first station.
    params.push(params[0]);
    let mut closing = sections[0].clone();
    closing.t = 1.0;
    sections.push(closing);

    let knots = piecewise_knots(&layout);

    // Surface rows in homogeneous coordinates.
    let mut rows: [Vec<[f64; 4]>; 3] = [
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
    ];
    for section in &sections {
        let (start, apex, end, weight) = section_arc(section);
        rows[0].push([start.x(), start.y(), start.z(), 1.0]);
        rows[1].push([
            apex.x() * weight,
            apex.y() * weight,
            apex.z() * weight,
            weight,
        ]);
        rows[2].push([end.x(), end.y(), end.z(), 1.0]);
    }
    let mut control_points = Vec::with_capacity(3);
    let mut weights = Vec::with_capacity(3);
    for row in &rows {
        let poles = piecewise_poles(row, &layout);
        let mut points = Vec::with_capacity(poles.len());
        let mut row_weights = Vec::with_capacity(poles.len());
        for pole in poles {
            if pole[3].partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
                return Err(walk_failure(walker, 0.0, f64::NAN));
            }
            points.push(Point3::new(
                pole[0] / pole[3],
                pole[1] / pole[3],
                pole[2] / pole[3],
            ));
            row_weights.push(pole[3]);
        }
        control_points.push(points);
        weights.push(row_weights);
    }
    let contact =
        |row: &[Point3]| NurbsCurve::new(3, knots.clone(), row.to_vec(), vec![1.0; row.len()]);
    let contact1 = contact(&control_points[0])?;
    let contact2 = contact(&control_points[2])?;
    let surface = NurbsSurface::new(
        2,
        3,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        knots.clone(),
        control_points,
        weights,
    )?;

    let pcurve = |side: usize| -> Result<NurbsCurve2D, BlendError> {
        let (u_period, v_period) = supports[side].1;
        let mut uv: Vec<[f64; 2]> = Vec::with_capacity(count);
        for station in &params {
            let (mut u, mut v) = if side == 0 {
                (station.u1, station.v1)
            } else {
                (station.u2, station.v2)
            };
            if let Some(previous) = uv.last() {
                if let Some(period) = u_period {
                    u = unwrap_periodic(u, previous[0], period);
                }
                if let Some(period) = v_period {
                    v = unwrap_periodic(v, previous[1], period);
                }
            }
            uv.push([u, v]);
        }
        // A NURBS support clamps outside its knot domain, so a contact
        // unwrapped from the far image of its seam (`u = hi` rather than
        // `lo`) would read the clamped boundary for its whole turn. Move the
        // run whole periods to sit in the domain. Analytic supports are
        // periodic in their own trigonometry and keep their values.
        if let Some(nurbs) = supports[side].0.as_nurbs() {
            let domains = [nurbs.domain_u(), nurbs.domain_v()];
            for (axis, period) in [u_period, v_period].into_iter().enumerate() {
                let Some(period) = period else { continue };
                let (lo, hi) = uv
                    .iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |acc, p| {
                        (acc.0.min(p[axis]), acc.1.max(p[axis]))
                    });
                let middle = f64::midpoint(domains[axis].0, domains[axis].1);
                let turns = ((f64::midpoint(lo, hi) - middle) / period).round();
                if turns != 0.0 && turns.is_finite() {
                    for point in &mut uv {
                        point[axis] -= turns * period;
                    }
                }
            }
        }
        let poles = piecewise_poles(&uv, &layout);
        let points: Vec<Point2> = poles.iter().map(|p| Point2::new(p[0], p[1])).collect();
        let pole_count = points.len();
        Ok(NurbsCurve2D::new(
            3,
            knots.clone(),
            points,
            vec![1.0; pole_count],
        )?)
    };
    let pcurve1 = pcurve(0)?;
    let pcurve2 = pcurve(1)?;

    let deviation = contact_deviation(&contact1, &pcurve1, supports[0].0, &layout.stations).max(
        contact_deviation(&contact2, &pcurve2, supports[1].0, &layout.stations),
    );
    Ok(ClosedWalkFit {
        sections,
        surface,
        contact1,
        contact2,
        pcurve1: Curve2D::Nurbs(pcurve1),
        pcurve2: Curve2D::Nurbs(pcurve2),
        deviation,
    })
}

/// Clamped cubic knots over `[0, 1]`: every interior station doubled, and
/// tripled where a segment starts (a corner).
fn piecewise_knots(layout: &Layout) -> Vec<f64> {
    let last = layout.stations.len() - 1;
    let mut knots = Vec::with_capacity(2 * last + 6 + layout.segments.len());
    knots.extend([0.0; 4]);
    for (index, &station) in layout.stations.iter().enumerate().take(last).skip(1) {
        let corner = layout.segments.iter().any(|&(start, _)| start == index);
        knots.extend(std::iter::repeat_n(station, if corner { 3 } else { 2 }));
    }
    knots.extend([1.0; 4]);
    knots
}

/// Fourth-order difference weights for `12w·f′` at local station `j` of a
/// segment of `spans` spans, and the first of the five stations they read:
/// central inside, one-sided at the two ends and their neighbours, so a
/// corner is never differentiated across.
const fn difference_stencil(j: usize, spans: usize) -> ([f64; 5], usize) {
    if j == 0 {
        ([-25.0, 48.0, -36.0, 16.0, -3.0], 0)
    } else if j == 1 {
        ([-3.0, -10.0, 18.0, -6.0, 1.0], 0)
    } else if j + 1 == spans {
        ([-1.0, 6.0, -18.0, 10.0, 3.0], spans - 4)
    } else if j == spans {
        ([3.0, -16.0, 36.0, -48.0, 25.0], spans - 4)
    } else {
        ([1.0, -8.0, 0.0, 8.0, -1.0], j - 2)
    }
}

/// The poles of the cubic Hermite B-spline through `values` (one per
/// station of `layout`) with [`piecewise_knots`].
///
/// Each span `[a, b]` of width `w` is the Bézier cubic `f_a, f_a + w/3·f′_a,
/// f_b − w/3·f′_b, f_b`. Inside a segment the node `f_b` between two spans is
/// implied by its doubled knot; at a segment end it is a pole of its own.
/// `w/3·f′` is the [`difference_stencil`] sum over 36: the stations of a
/// segment are `w` apart, so the quotient's `12w` cancels the span's `w`.
fn piecewise_poles<const D: usize>(values: &[[f64; D]], layout: &Layout) -> Vec<[f64; D]> {
    let mut poles = Vec::with_capacity(2 * values.len() + layout.segments.len());
    poles.push(values[0]);
    for &(start, end) in &layout.segments {
        let spans = end - start;
        let leg = |j: usize| -> [f64; D] {
            let (coefficients, base) = difference_stencil(j, spans);
            let mut out = [0.0; D];
            for (offset, coefficient) in coefficients.iter().enumerate() {
                let value = values[start + base + offset];
                for axis in 0..D {
                    out[axis] = coefficient.mul_add(value[axis], out[axis]);
                }
            }
            for coordinate in &mut out {
                *coordinate /= 36.0;
            }
            out
        };
        for j in 0..spans {
            let (mut out_a, mut in_b) = (values[start + j], values[start + j + 1]);
            let (da, db) = (leg(j), leg(j + 1));
            for axis in 0..D {
                out_a[axis] += da[axis];
                in_b[axis] -= db[axis];
            }
            poles.push(out_a);
            poles.push(in_b);
        }
        poles.push(values[end]);
    }
    poles
}

fn unwrap_periodic(value: f64, previous: f64, period: f64) -> f64 {
    value - period * ((value - previous) / period).round()
}

/// Largest distance of `contact` from `support` (closest point) and from
/// `pcurve`'s image, sampled at every station and [`SAMPLES_PER_SPAN`]
/// interior points of each span.
#[allow(clippy::cast_precision_loss)]
fn contact_deviation(
    contact: &NurbsCurve,
    pcurve: &NurbsCurve2D,
    support: &dyn ParametricSurface,
    stations: &[f64],
) -> f64 {
    let mut worst = 0.0_f64;
    for span in stations.windows(2) {
        for j in 0..=SAMPLES_PER_SPAN {
            let f = j as f64 / (SAMPLES_PER_SPAN + 1) as f64;
            let t = (span[1] - span[0]).mul_add(f, span[0]);
            let point = contact.evaluate(t);
            let (u, v) = support.project_point(point);
            let closest = (support.evaluate(u, v) - point).length();
            let uv = pcurve.evaluate(t);
            let image = (support.evaluate(uv.x(), uv.y()) - point).length();
            // NaN propagates as a refusal: `max` would drop it.
            if !closest.is_finite() || !image.is_finite() {
                return f64::INFINITY;
            }
            worst = worst.max(closest).max(image);
        }
    }
    worst
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    /// Points of a closed curve at `stations`: a unit circle, or with
    /// `corner` a polar radius that kinks at that station.
    fn curve_points(stations: &[f64], corner: Option<f64>) -> Vec<[f64; 3]> {
        stations
            .iter()
            .map(|&t| {
                let angle = std::f64::consts::TAU * t;
                let radius = corner.map_or(1.0, |c| 0.1f64.mul_add((t - c).abs(), 1.0));
                [radius * angle.cos(), radius * angle.sin(), 0.0]
            })
            .collect()
    }

    fn spline(layout: &Layout, values: &[[f64; 3]]) -> NurbsCurve {
        let poles: Vec<Point3> = piecewise_poles(values, layout)
            .iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect();
        let count = poles.len();
        NurbsCurve::new(3, piecewise_knots(layout), poles, vec![1.0; count]).unwrap()
    }

    /// A unit circle through 64 even stations: one segment, double knots,
    /// and within the cubic Hermite remainder of the circle.
    #[test]
    fn piecewise_poles_follow_a_circle() {
        let layout = layout(&[0.0], 64);
        assert_eq!(layout.stations.len(), 65);
        let curve = spline(&layout, &curve_points(&layout.stations, None));
        assert_eq!(curve.control_points().len(), 2 * 64 + 2);
        let mut worst = 0.0_f64;
        for i in 0..=4096 {
            let p = curve.evaluate(f64::from(i) / 4096.0);
            worst = worst.max(((p - Point3::new(0.0, 0.0, 0.0)).length() - 1.0).abs());
        }
        // Cubic Hermite remainder of a unit circle at h = 2π/64: ~2.6e-7.
        assert!(worst < 4e-7, "radial error {worst:e}");
    }

    /// A corner in the middle of the closed curve becomes a tripled knot
    /// with the corner point as a pole: the curve passes through it and
    /// keeps a different tangent on each side.
    #[test]
    fn a_corner_keeps_its_point_and_both_tangents() {
        let layout = layout(&[0.0, 0.5], 64);
        let curve = spline(&layout, &curve_points(&layout.stations, Some(0.5)));
        assert_eq!(
            curve
                .knots()
                .iter()
                .filter(|&&k| (k - 0.5).abs() < 1e-15)
                .count(),
            3
        );
        let corner = curve.evaluate(0.5);
        assert!((corner - Point3::new(-1.0, 0.0, 0.0)).length() < 1e-12);
        let before = (corner - curve.evaluate(0.5 - 1e-6)).normalize().unwrap();
        let after = (curve.evaluate(0.5 + 1e-6) - corner).normalize().unwrap();
        assert!(before.cross(after).length() > 1e-2, "the corner is kept");
    }
}
