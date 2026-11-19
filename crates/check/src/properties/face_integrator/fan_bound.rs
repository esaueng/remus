//! A bound on the area the chord-polygon fan loses on a planar face.
//!
//! [`super::integrate_planar_face_fan`] reads a planar face the exact
//! Green's-theorem reader declines as the polygon of its sampled boundary
//! ([`crate::util::wire_polygon`]): [`CLOSED_CURVE_SAMPLES`] chords for a
//! closed curved edge, [`OPEN_CURVE_SAMPLES`] for an open one, a line as its
//! own chord. Its reading does not depend on any quadrature order, so no
//! order comparison can tell how far it is off. This module bounds it from
//! the boundary's own geometry instead, chord by chord, over exactly the
//! parameters that sampler takes.
//!
//! # The bound
//!
//! The true region and the polygon differ by the slivers between each chord
//! and the piece of curve it cuts off, so `|A_true − A_fan| ≤ Σ |sliver|`
//! (holes and islands included, whatever their sign). Take the chord as the
//! `x` axis of its plane: the sliver's area is `|∫ y dx|` along the piece,
//! at most `g · ℓ`, where `g` bounds the piece's distance from the chord's
//! line and `ℓ` its arc length (which bounds `∫ |dx|`). Distances and
//! lengths are taken in 3D, never smaller than in the plane.
//!
//! * A conic piece of parameter length `Δ` with `|P'| ≤ M₁` and
//!   `|P''| ≤ M₂` on it: the curve's offset from the linear interpolant of
//!   its ends is `½ (t − a)(b − t)` times a weighted mean of `P''`, so
//!   `g ≤ Δ² M₂ / 8`, and `ℓ ≤ Δ M₁`. A circle of radius `r` has
//!   `M₁ = M₂ = r`; an ellipse `√(a² + b²)`; a hyperbola that times
//!   `cosh max|t|`; a parabola `1 + max|t| / 2f` and `1 / 2f`.
//! * A NURBS piece with positive weights, split at its knots and each span
//!   restricted to the piece (and halved) by blossoming, lies in the hull of
//!   its rational Bézier control points, so `g` is their largest distance from the chord's
//!   line. Rational de Casteljau subdivision cuts corners (each new point
//!   lies on a leg of the old polygon), so the control polygon's length
//!   bounds `ℓ`. A closed edge's chord that crosses the end of its domain
//!   adds the straight jump from the domain's end back to its start, whose
//!   distance from the line is bounded by its ends'.
//!
//! The bound is `+∞` (no certificate) for a NURBS edge with a weight that is
//! not finite and positive, a parameter outside its domain, or a closed edge
//! the sampler reads without a finite domain.
//!
//! It is evaluated in floating point; its own rounding, and the gaps between
//! consecutive edges' end samples (within the edges' tolerances), are below
//! anything it is compared with.

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;
use remus_topology::wire::WireId;

use crate::CheckError;
use crate::util::{CLOSED_CURVE_SAMPLES, OPEN_CURVE_SAMPLES};

/// Bound on `|A_true − A_fan|` for the planar face `face_id`, over its outer
/// and inner wires, as [`super::integrate_planar_face_fan`] samples them.
///
/// # Errors
///
/// Topology lookups and edge domains, exactly as the fan's own sampler.
pub(super) fn fan_area_bound(topo: &Topology, face_id: FaceId) -> Result<f64, CheckError> {
    let face = topo.face(face_id)?;
    let mut bound = wire_bound(topo, face.outer_wire())?;
    for &wire in face.inner_wires() {
        bound += wire_bound(topo, wire)?;
    }
    Ok(bound)
}

/// Mirrors the edge cases of `wire_polygon_curve_sampled_except` with
/// [`CLOSED_CURVE_SAMPLES`] and [`OPEN_CURVE_SAMPLES`] and no chord list.
fn wire_bound(topo: &Topology, wire: WireId) -> Result<f64, CheckError> {
    let mut bound = 0.0;
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        let curve = edge.curve();
        let closed = edge.start() == edge.end()
            && matches!(
                curve,
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) | EdgeCurve::NurbsCurve(_)
            );
        let edge_bound = if closed {
            match curve {
                EdgeCurve::NurbsCurve(nc) => {
                    let (u0, u1) = nc.domain();
                    let span = u1 - u0;
                    if span.is_finite() && span > 0.0 {
                        let seam = crate::util::nurbs_seam_parameter(
                            nc,
                            topo.vertex(edge.start())?.point(),
                            u0,
                            u1,
                        );
                        closed_nurbs_bound(nc, seam, CLOSED_CURVE_SAMPLES)
                    } else {
                        // The sampler's fallback: no parameters to bound.
                        f64::INFINITY
                    }
                }
                EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => {
                    let (t0, t1) = edge
                        .strict_domain()
                        .map_err(crate::error::edge_domain_validation)?;
                    conic_bound(curve, t0, t1, CLOSED_CURVE_SAMPLES)
                }
                // `closed` admits only the three curves above.
                EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => 0.0,
            }
        } else if matches!(curve, EdgeCurve::Line) {
            0.0
        } else {
            let (t0, t1) = edge
                .strict_domain()
                .map_err(crate::error::edge_domain_validation)?;
            match curve {
                EdgeCurve::NurbsCurve(nc) => open_nurbs_bound(nc, t0, t1, OPEN_CURVE_SAMPLES),
                EdgeCurve::Line
                | EdgeCurve::Circle(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => conic_bound(curve, t0, t1, OPEN_CURVE_SAMPLES),
            }
        };
        bound += edge_bound;
    }
    Ok(bound)
}

/// The sampler's `i`-th of `n` parameters over `(t0, t1)`.
#[allow(clippy::cast_precision_loss)]
fn sample(t0: f64, t1: f64, i: usize, n: usize) -> f64 {
    (t1 - t0).mul_add(i as f64 / n as f64, t0)
}

/// `Σ g · ℓ` over the `n` chords of a conic over `(t0, t1)`, from closed-form
/// bounds on its first and second derivatives.
fn conic_bound(curve: &EdgeCurve, t0: f64, t1: f64, n: usize) -> f64 {
    let mut bound = 0.0;
    for i in 0..n {
        let (a, b) = (sample(t0, t1, i, n), sample(t0, t1, i + 1, n));
        let delta = (b - a).abs();
        let reach = a.abs().max(b.abs());
        let (m1, m2) = match curve {
            EdgeCurve::Circle(c) => (c.radius(), c.radius()),
            EdgeCurve::Ellipse(e) => {
                let m = e.semi_major().hypot(e.semi_minor());
                (m, m)
            }
            EdgeCurve::Hyperbola(h) => {
                let m = h.semi_major().hypot(h.semi_minor()) * reach.cosh();
                (m, m)
            }
            EdgeCurve::Parabola(p) => {
                let inverse = 1.0 / (2.0 * p.focal_length().abs());
                (reach.mul_add(inverse, 1.0), inverse)
            }
            // A line is its own chord; NURBS edges take the hull bound.
            EdgeCurve::Line => (0.0, 0.0),
            EdgeCurve::NurbsCurve(_) => (f64::INFINITY, f64::INFINITY),
        };
        bound += delta.powi(3) * m1 * m2 / 8.0;
    }
    if bound.is_nan() { f64::INFINITY } else { bound }
}

/// `Σ g · ℓ` over the `n` chords an open NURBS edge is sampled in.
fn open_nurbs_bound(nc: &NurbsCurve, t0: f64, t1: f64, n: usize) -> f64 {
    let mut bound = 0.0;
    for i in 0..n {
        let (a, b) = (sample(t0, t1, i, n), sample(t0, t1, i + 1, n));
        let chord = (nc.evaluate(a), nc.evaluate(b));
        bound += hull_bound(nc, &[(a.min(b), a.max(b))], chord, 0.0);
    }
    bound
}

/// `Σ g · ℓ` over the `n` chords of a closed NURBS edge sampled from `seam`
/// around its whole domain.
fn closed_nurbs_bound(nc: &NurbsCurve, seam: f64, n: usize) -> f64 {
    let (u0, u1) = nc.domain();
    let span = u1 - u0;
    #[allow(clippy::cast_precision_loss)]
    let param = |i: usize| u0 + (seam - u0 + span * (i as f64) / (n as f64)).rem_euclid(span);
    let jump = (nc.evaluate(u1) - nc.evaluate(u0)).length();
    let mut bound = 0.0;
    for i in 0..n {
        let (a, b) = (param(i), param(i + 1));
        let chord = (nc.evaluate(a), nc.evaluate(b));
        bound += if a <= b {
            hull_bound(nc, &[(a, b)], chord, 0.0)
        } else {
            // The chord crosses the end of the domain.
            hull_bound(nc, &[(a, u1), (u0, b)], chord, jump)
        };
    }
    bound
}

/// `g · ℓ` for the curve over `ranges` against the line through `chord`:
/// `g` the largest distance of a span's rational Bézier control point from
/// that line, `ℓ` the control polygons' total length plus `extra`.
fn hull_bound(nc: &NurbsCurve, ranges: &[(f64, f64)], chord: (Point3, Point3), extra: f64) -> f64 {
    let (origin, end) = chord;
    let direction = end - origin;
    let along = direction.length();
    let distance = |q: Vec3| {
        if along > 0.0 {
            q.cross(direction).length() / along
        } else {
            q.length()
        }
    };
    let (mut g, mut length) = (0.0_f64, extra);
    for &(lo, hi) in ranges {
        let Some(pieces) = bezier_pieces(nc, lo, hi, origin) else {
            return f64::INFINITY;
        };
        for control in pieces {
            for pair in control.windows(2) {
                length += (pair[1] - pair[0]).length();
            }
            for &q in &control {
                g = g.max(distance(q));
            }
        }
    }
    let bound = g * length;
    if bound.is_finite() {
        bound
    } else {
        f64::INFINITY
    }
}

/// The rational Bézier control points, relative to `origin`, of every knot
/// span of `nc` restricted to `(lo, hi)`; `None` when a weight is not
/// finite and positive, the curve's data is malformed, or `(lo, hi)` leaves
/// its domain.
fn bezier_pieces(nc: &NurbsCurve, lo: f64, hi: f64, origin: Point3) -> Option<Vec<Vec<Vec3>>> {
    let p = nc.degree();
    let knots = nc.knots();
    let points = nc.control_points();
    let weights = nc.weights();
    let n = points.len();
    if p == 0 || n <= p || knots.len() != n + p + 1 || weights.len() != n {
        return None;
    }
    if !(lo >= knots[p] && hi <= knots[n] && lo <= hi) {
        return None;
    }
    if weights.iter().any(|&w| !(w.is_finite() && w > 0.0)) {
        return None;
    }
    if lo >= hi {
        return Some(Vec::new());
    }
    let homogeneous: Vec<[f64; 4]> = points
        .iter()
        .zip(weights)
        .map(|(&point, &w)| {
            let d = point - origin;
            [d.x() * w, d.y() * w, d.z() * w, w]
        })
        .collect();
    let mut k = (knots[p..=n].partition_point(|&u| u <= lo) + p).saturating_sub(1);
    k = k.clamp(p, n - 1);
    let mut pieces = Vec::new();
    while k < n && knots[k] < hi {
        let (a, b) = (knots[k].max(lo), knots[k + 1].min(hi));
        if b > a {
            let window = &homogeneous[k - p..=k];
            // Halved once: a quadratic piece's middle control point stands
            // off its chord twice the sagitta, its halves' at most the
            // sagitta, which brings `g · ℓ` from three times the sliver to
            // about one and a half.
            let m = f64::midpoint(a, b);
            for (a, b) in [(a, m), (m, b)] {
                let mut control = Vec::with_capacity(p + 1);
                for i in 0..=p {
                    let q = blossom(window, knots, k, p, a, b, i);
                    if !(q[3].is_finite() && q[3] > 0.0) {
                        return None;
                    }
                    control.push(Vec3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]));
                }
                pieces.push(control);
            }
        }
        k += 1;
    }
    Some(pieces)
}

/// The Bézier control point `i` of span `k` restricted to `(a, b)`: the
/// blossom at `a` repeated `p − i` times and `b` repeated `i` times, by de
/// Boor's triangle over the span's own control points `window`.
fn blossom(
    window: &[[f64; 4]],
    knots: &[f64],
    k: usize,
    p: usize,
    a: f64,
    b: f64,
    i: usize,
) -> [f64; 4] {
    let mut d = window.to_vec();
    for r in 1..=p {
        let t = if r <= p - i { a } else { b };
        for j in (r..=p).rev() {
            let g = k - p + j;
            let (lo, hi) = (knots[g], knots[g + p + 1 - r]);
            let alpha = (t - lo) / (hi - lo);
            for c in 0..4 {
                d[j][c] = (d[j][c] - d[j - 1][c]).mul_add(alpha, d[j - 1][c]);
            }
        }
    }
    d[p]
}
