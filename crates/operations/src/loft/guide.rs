//! Loft along a guide curve: the loft passes through every section and its
//! edge from each section's guide vertex is the guide curve itself.
//!
//! Construction. Every section is a polygon with the same corner count, and
//! the guide `G(u)`, reparameterised onto `[0, 1]`, passes through corner 0
//! of section `k` at `u = t_k`. Each other corner `i` gets the offset curve
//! `D_i(u)` interpolating `V_i^k − V_0^k` at `t_k` (degree 1 for a ruled
//! loft, up to cubic for a smooth one), and its rail is `G(u) + D_i(u)`.
//! That is the Gordon-style correction of the corner-interpolating skin with a
//! blend weight of one at every corner: the section offsets ride the guide.
//! At `u = t_k` every rail is exactly at its corner, so each section is
//! interpolated, and rail 0 is the guide. The wall between two rails is their
//! degree-1 blend, straight across, so its section iso-curves are the
//! sections' edges.
//!
//! The sum is exact in either case:
//! - a polynomial guide and the offsets are raised to a common degree and
//!   knot vector, and their control points add;
//! - a rational guide (an arc or another conic) is added in homogeneous form,
//!   `W·(G + D_i)` with `W` the guide's weight function. Each Bézier span of
//!   `W·D_i` is the exact Bernstein product, and every rail then shares the
//!   guide's weights raised by the offsets' degree, so each wall stays a
//!   straight blend between its rails.
//!
//! Curved sections and unequal corner counts have no exact construction here
//! and are refused as unsupported rather than approximated; an unclamped
//! guide, or one that misses a section's corner, is refused as invalid.

use remus_math::compact_sort::CompactSort;
use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::basis::{basis_funs, find_span};
use remus_math::nurbs::knot_ops::curve_knot_refine;
use remus_math::nurbs::projection::project_point_to_curve;
use remus_math::nurbs::{curve_degree_elevate, curve_to_bezier_segments};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::{build_rail_loft, reject_holed_profiles, validate_loft_result};
use crate::OperationsError;
use crate::winding::ensure_ccw_profiles;

const OPERATION: &str = "loft_with_guide";

/// A section parameter or offset knot this close to a guide knot is taken as
/// that knot, so the merged knot vector never holds a sliver span.
const KNOT_SNAP: f64 = 1e-9;

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: OPERATION,
        reason: reason.into(),
    }
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    OperationsError::InvalidInput {
        reason: reason.into(),
    }
}

/// Loft `profiles` so the loft also follows `guide` exactly.
///
/// The guide must pass through one corner of every section, in section
/// order, and start and end on the first and last sections; that corner is
/// the section's connection start, as in [`super::loft_with_connections`].
/// Section offsets from the guide vary linearly between sections when
/// `smooth` is false and are interpolated smoothly (up to cubic) when it is
/// true. Every face is exact: planar caps and NURBS walls whose edge from
/// the guide corners is the guide itself.
///
/// # Errors
///
/// - [`OperationsError::InvalidInput`] for fewer than two sections, a section
///   with a hole or fewer than three corners, an unclamped guide, or a guide
///   that does not pass through exactly one corner of every section, in
///   order, from the first section to the last.
/// - [`OperationsError::Unsupported`] (`loft_with_guide`) for a section with
///   a curved edge or sections with different corner counts, which have no
///   exact construction here yet.
/// - [`OperationsError::ExactOnlyUnattainable`] when the built solid fails
///   validation (for example a guide that folds a wall through another).
///
/// Every error leaves the topology unchanged.
pub fn loft_with_guide(
    topo: &mut Topology,
    profiles: &[FaceId],
    guide: &NurbsCurve,
    smooth: bool,
) -> Result<SolidId, OperationsError> {
    if profiles.len() < 2 {
        return Err(invalid("loft requires at least 2 profiles"));
    }
    reject_holed_profiles(topo, profiles)?;
    let mut rings = section_polygons(topo, profiles)?;
    let guide = normalized_guide(guide)?;
    let tol = Tolerance::new().linear;

    let mut params = Vec::with_capacity(rings.len());
    for (k, ring) in rings.iter_mut().enumerate() {
        let (corner, t) = guide_corner(&guide, ring, k, tol)?;
        ring.rotate_left(corner);
        params.push(t);
    }
    let last = params.len() - 1;
    let guide = if params[0] > params[last] {
        for t in &mut params {
            *t = 1.0 - *t;
        }
        guide.reversed()
    } else {
        guide
    };
    if (guide.evaluate(0.0) - rings[0][0]).length() > tol {
        return Err(invalid(
            "the loft guide must start on the first section's guide corner",
        ));
    }
    if (guide.evaluate(1.0) - rings[last][0]).length() > tol {
        return Err(invalid(
            "the loft guide must end on the last section's guide corner",
        ));
    }
    params[0] = 0.0;
    params[last] = 1.0;
    if let Some(k) = params.windows(2).position(|w| w[1] - w[0] <= KNOT_SNAP) {
        return Err(invalid(format!(
            "the loft guide must meet the sections in order: it reaches section {} no later \
             than section {k}",
            k + 1
        )));
    }

    if ensure_ccw_profiles(&mut rings) {
        // Reversal moved the guide corner from the front to the back.
        for ring in &mut rings {
            ring.rotate_right(1);
        }
    }
    let rails = guided_rails(&guide, &rings, &params, smooth)?;
    remus_topology::transaction::run_validated(
        topo,
        |topo| build_rail_loft(topo, &rings, &rails),
        |topo, solid| validate_loft_result(topo, *solid),
    )
}

/// Each section's corners in loop order, or a refusal for a curved edge or
/// unequal corner counts.
fn section_polygons(
    topo: &Topology,
    profiles: &[FaceId],
) -> Result<Vec<Vec<Point3>>, OperationsError> {
    let mut rings = Vec::with_capacity(profiles.len());
    for (k, &face) in profiles.iter().enumerate() {
        let wire = topo.wire(topo.face(face)?.outer_wire())?;
        let mut ring = Vec::with_capacity(wire.edges().len());
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return Err(unsupported(format!(
                    "section {k} has a curved edge: a guided loft builds straight-edged \
                     sections only"
                )));
            }
            let start = if oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            ring.push(topo.vertex(start)?.point());
        }
        if ring.len() < 3 {
            return Err(invalid(format!(
                "loft section {k} has fewer than three corners"
            )));
        }
        if let Some(first) = rings.first().map(Vec::len)
            && ring.len() != first
        {
            return Err(unsupported(format!(
                "section {k} has {} corners and section 0 has {first}: a guided loft pairs \
                 corners one to one",
                ring.len()
            )));
        }
        rings.push(ring);
    }
    Ok(rings)
}

/// The guide clamped on `[0, 1]`, with equal weights made one.
fn normalized_guide(guide: &NurbsCurve) -> Result<NurbsCurve, OperationsError> {
    let p = guide.degree();
    let knots = guide.knots();
    let (a, b) = (knots[0], knots[knots.len() - 1]);
    #[allow(clippy::float_cmp)]
    let clamped =
        knots[..=p].iter().all(|&k| k == a) && knots[knots.len() - p - 1..].iter().all(|&k| k == b);
    if !clamped {
        return Err(invalid(
            "the loft guide must be clamped: its ends are its first and last control points",
        ));
    }
    let span = b - a;
    if !(span.is_finite() && span > 0.0) {
        return Err(invalid("the loft guide has an empty parameter domain"));
    }
    let last = knots.len() - 1;
    let unit: Vec<f64> = knots
        .iter()
        .enumerate()
        .map(|(i, &k)| {
            if i <= p {
                0.0
            } else if i >= last - p {
                1.0
            } else {
                ((k - a) / span).clamp(0.0, 1.0)
            }
        })
        .collect();
    let w0 = guide.weights()[0];
    let weights = if guide
        .weights()
        .iter()
        .all(|&w| (w - w0).abs() <= 1e-12 * w0)
    {
        vec![1.0; guide.weights().len()]
    } else {
        guide.weights().to_vec()
    };
    Ok(NurbsCurve::new(
        p,
        unit,
        guide.control_points().to_vec(),
        weights,
    )?)
}

/// The one corner of `ring` on the guide, and the guide parameter there.
fn guide_corner(
    guide: &NurbsCurve,
    ring: &[Point3],
    section: usize,
    tol: f64,
) -> Result<(usize, f64), OperationsError> {
    let mut found: Option<(usize, f64)> = None;
    for (i, &corner) in ring.iter().enumerate() {
        let hit = project_point_to_curve(guide, corner, 1e-12)?;
        if hit.distance > tol {
            continue;
        }
        if found.is_some() {
            return Err(invalid(format!(
                "the loft guide passes through more than one corner of section {section}"
            )));
        }
        found = Some((i, hit.parameter));
    }
    found.ok_or_else(|| {
        invalid(format!(
            "the loft guide does not pass through a corner of section {section}"
        ))
    })
}

/// The junction rails `G + D_i`, sharing one degree, knot vector and weight
/// vector on `[0, 1]`.
fn guided_rails(
    guide: &NurbsCurve,
    rings: &[Vec<Point3>],
    params: &[f64],
    smooth: bool,
) -> Result<Vec<NurbsCurve>, OperationsError> {
    let m = rings.len();
    let degree = if smooth { (m - 1).min(3) } else { 1 };
    let knots = offset_knots(params, degree, guide.knots());
    let offsets = (1..rings[0].len())
        .map(|i| {
            let points: Vec<Point3> = rings
                .iter()
                .map(|ring| {
                    let d = ring[i] - ring[0];
                    Point3::new(d.x(), d.y(), d.z())
                })
                .collect();
            interpolate_on_knots(&points, params, &knots, degree)
        })
        .collect::<Result<Vec<_>, _>>()?;
    if guide.is_rational() {
        rational_rails(guide, &offsets)
    } else {
        polynomial_rails(guide, &offsets)
    }
}

/// Averaged interpolation knots (NURBS Book eq. 9.8) for `params`, each
/// interior knot snapped onto a guide knot within [`KNOT_SNAP`].
fn offset_knots(params: &[f64], degree: usize, guide_knots: &[f64]) -> Vec<f64> {
    let n = params.len();
    let mut knots = vec![0.0; degree + 1];
    for j in 1..n - degree {
        #[allow(clippy::cast_precision_loss)]
        let average = params[j..j + degree].iter().sum::<f64>() / degree as f64;
        let snapped = guide_knots
            .iter()
            .copied()
            .filter(|&k| k > 0.0 && k < 1.0 && (k - average).abs() <= KNOT_SNAP)
            .min_by(|x, y| (x - average).abs().total_cmp(&(y - average).abs()))
            .unwrap_or(average);
        knots.push(snapped);
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

/// The B-spline of `degree` on `knots` through `points` at `params`, solved
/// exactly (dense elimination with partial pivoting).
fn interpolate_on_knots(
    points: &[Point3],
    params: &[f64],
    knots: &[f64],
    degree: usize,
) -> Result<NurbsCurve, OperationsError> {
    let n = points.len();
    let mut rows: Vec<Vec<f64>> = vec![vec![0.0; n + 3]; n];
    for (row, (&t, point)) in rows.iter_mut().zip(params.iter().zip(points)) {
        let span = find_span(n, degree, t, knots);
        for (j, value) in basis_funs(span, t, degree, knots).into_iter().enumerate() {
            row[span - degree + j] = value;
        }
        row[n] = point.x();
        row[n + 1] = point.y();
        row[n + 2] = point.z();
    }
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&a, &b| rows[a][col].abs().total_cmp(&rows[b][col].abs()))
            .unwrap_or(col);
        if rows[pivot][col].abs() < 1e-12 {
            return Err(unsupported(
                "the sections' guide parameters give no unique offset interpolation",
            ));
        }
        rows.swap(col, pivot);
        let lead = rows[col].clone();
        for (r, row) in rows.iter_mut().enumerate() {
            if r == col || row[col] == 0.0 {
                continue;
            }
            let factor = row[col] / lead[col];
            for (cell, &l) in row.iter_mut().zip(&lead).skip(col) {
                *cell -= factor * l;
            }
        }
    }
    let control_points = rows
        .iter()
        .enumerate()
        .map(|(i, row)| Point3::new(row[n] / row[i], row[n + 1] / row[i], row[n + 2] / row[i]))
        .collect();
    Ok(NurbsCurve::new(
        degree,
        knots.to_vec(),
        control_points,
        vec![1.0; n],
    )?)
}

/// Distinct interior knot values with their multiplicities.
fn interior_multiplicities(curve: &NurbsCurve) -> Vec<(f64, usize)> {
    let p = curve.degree();
    let knots = curve.knots();
    let mut out: Vec<(f64, usize)> = Vec::new();
    for &k in &knots[p + 1..knots.len() - p - 1] {
        match out.last_mut() {
            #[allow(clippy::float_cmp)]
            Some((value, count)) if *value == k => *count += 1,
            _ => out.push((k, 1)),
        }
    }
    out
}

/// Insert knots into `curve` until each value of `targets` has at least its
/// target multiplicity.
fn refine_to(curve: &NurbsCurve, targets: &[(f64, usize)]) -> Result<NurbsCurve, OperationsError> {
    let have = interior_multiplicities(curve);
    let mut insert = Vec::new();
    for &(value, target) in targets {
        #[allow(clippy::float_cmp)]
        let current = have
            .iter()
            .find(|(k, _)| *k == value)
            .map_or(0, |&(_, c)| c);
        insert.extend(std::iter::repeat_n(value, target.saturating_sub(current)));
    }
    Ok(curve_knot_refine(curve, &insert)?)
}

/// The union of two curves' interior knots, each at its larger multiplicity.
fn merged_knots(a: &NurbsCurve, b: &NurbsCurve) -> Vec<(f64, usize)> {
    let mut merged: Vec<(f64, usize)> = interior_multiplicities(a);
    for (value, count) in interior_multiplicities(b) {
        #[allow(clippy::float_cmp)]
        match merged.iter_mut().find(|(k, _)| *k == value) {
            Some((_, have)) => *have = (*have).max(count),
            None => merged.push((value, count)),
        }
    }
    merged.compact_sort_by(|x, y| x.0.total_cmp(&y.0));
    merged
}

fn same_knots(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() <= 1e-12)
}

/// Polynomial guide: raise the guide and offsets to one degree and knot
/// vector, then add control points.
fn polynomial_rails(
    guide: &NurbsCurve,
    offsets: &[NurbsCurve],
) -> Result<Vec<NurbsCurve>, OperationsError> {
    let offset_degree = offsets.first().map_or(1, NurbsCurve::degree);
    let p = guide.degree().max(offset_degree);
    let guide = curve_degree_elevate(guide, p - guide.degree())?;
    let elevated = offsets
        .iter()
        .map(|d| curve_degree_elevate(d, p - offset_degree))
        .collect::<Result<Vec<_>, _>>()?;
    let Some(first) = elevated.first() else {
        return Err(invalid("loft sections need at least three corners"));
    };
    let targets = merged_knots(&guide, first);
    let guide = refine_to(&guide, &targets)?;
    let mut rails = vec![guide.clone()];
    for offset in &elevated {
        let offset = refine_to(offset, &targets)?;
        if !same_knots(offset.knots(), guide.knots()) {
            return Err(unsupported(
                "the guide and the section offsets did not reach one knot vector",
            ));
        }
        let control_points = guide
            .control_points()
            .iter()
            .zip(offset.control_points())
            .map(|(&g, &d)| g + Vec3::new(d.x(), d.y(), d.z()))
            .collect();
        rails.push(NurbsCurve::new(
            p,
            guide.knots().to_vec(),
            control_points,
            guide.weights().to_vec(),
        )?);
    }
    Ok(rails)
}

fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n - k);
    let mut value = 1.0;
    for i in 0..k {
        #[allow(clippy::cast_precision_loss)]
        {
            value = value * (n - i) as f64 / (i + 1) as f64;
        }
    }
    value
}

/// The Bernstein coefficients of the product of two Bernstein polynomials of
/// degrees `f.len() − 1` and `g.len() − 1`, componentwise over `D` values.
fn bernstein_product<const D: usize>(f: &[f64], g: &[[f64; D]]) -> Vec<[f64; D]> {
    let (a, b) = (f.len() - 1, g.len() - 1);
    (0..=a + b)
        .map(|k| {
            let mut sum = [0.0; D];
            for i in k.saturating_sub(b)..=k.min(a) {
                let c = binomial(a, i) * binomial(b, k - i) / binomial(a + b, k);
                for (s, &gv) in sum.iter_mut().zip(&g[k - i]) {
                    *s += c * f[i] * gv;
                }
            }
            sum
        })
        .collect()
}

/// Rational guide: on each Bézier span of the merged breakpoints, the rail's
/// homogeneous form is `(W·G + W·D_i, W)` raised by the offsets' degree,
/// with `W·D_i` the exact Bernstein product.
fn rational_rails(
    guide: &NurbsCurve,
    offsets: &[NurbsCurve],
) -> Result<Vec<NurbsCurve>, OperationsError> {
    let Some(first) = offsets.first() else {
        return Err(invalid("loft sections need at least three corners"));
    };
    let pg = guide.degree();
    let pd = first.degree();
    let q = pg + pd;
    let breaks: Vec<(f64, usize)> = merged_knots(guide, first)
        .into_iter()
        .map(|(value, _)| (value, 1))
        .collect();
    let guide_spans = curve_to_bezier_segments(&refine_to(guide, &breaks)?)?;
    let span_count = guide_spans.len();
    if span_count != breaks.len() + 1 {
        return Err(unsupported(
            "the guide's Bézier spans do not match the section breakpoints",
        ));
    }
    let ones = vec![[1.0]; pd + 1];
    let weight_spans: Vec<Vec<f64>> = guide_spans
        .iter()
        .map(|span| span.weights().to_vec())
        .collect();
    let mut knots = vec![0.0; q + 1];
    for &(value, _) in &breaks {
        knots.extend(std::iter::repeat_n(value, q));
    }
    knots.extend(std::iter::repeat_n(1.0, q + 1));

    // The raised guide in homogeneous form, span by span.
    let raised: Vec<Vec<[f64; 4]>> = guide_spans
        .iter()
        .zip(&weight_spans)
        .map(|(span, weights)| {
            let ws: Vec<f64> = bernstein_product(weights, &ones)
                .into_iter()
                .map(|[w]| w)
                .collect();
            let wp: Vec<[f64; 3]> = span
                .control_points()
                .iter()
                .zip(span.weights())
                .map(|(pt, &w)| [w * pt.x(), w * pt.y(), w * pt.z()])
                .collect();
            let ones3: Vec<f64> = vec![1.0; pd + 1];
            bernstein_product(&ones3, &wp)
                .into_iter()
                .zip(ws)
                .map(|([x, y, z], w)| [x, y, z, w])
                .collect()
        })
        .collect();

    let assemble = |spans: &[Vec<[f64; 4]>]| -> Result<NurbsCurve, OperationsError> {
        let mut control_points = Vec::new();
        let mut weights = Vec::new();
        for (s, span) in spans.iter().enumerate() {
            for &[x, y, z, w] in span.iter().skip(usize::from(s > 0)) {
                control_points.push(Point3::new(x / w, y / w, z / w));
                weights.push(w);
            }
        }
        Ok(NurbsCurve::new(q, knots.clone(), control_points, weights)?)
    };

    let mut rails = vec![assemble(&raised)?];
    for offset in offsets {
        let offset_spans = curve_to_bezier_segments(&refine_to(offset, &breaks)?)?;
        if offset_spans.len() != span_count {
            return Err(unsupported(
                "the section offsets' Bézier spans do not match the guide's",
            ));
        }
        let spans: Vec<Vec<[f64; 4]>> = raised
            .iter()
            .zip(&weight_spans)
            .zip(&offset_spans)
            .map(|((guide_span, weights), offset_span)| {
                let d: Vec<[f64; 3]> = offset_span
                    .control_points()
                    .iter()
                    .map(|pt| [pt.x(), pt.y(), pt.z()])
                    .collect();
                bernstein_product(weights, &d)
                    .into_iter()
                    .zip(guide_span)
                    .map(|([x, y, z], &[gx, gy, gz, w])| [gx + x, gy + y, gz + z, w])
                    .collect()
            })
            .collect();
        rails.push(assemble(&spans)?);
    }
    Ok(rails)
}

#[cfg(test)]
mod tests;
