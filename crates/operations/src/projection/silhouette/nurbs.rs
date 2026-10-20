//! Silhouette contours of NURBS surfaces: the zero set of
//! `g(u, v) = n(u, v)·d` over the surface's parameter domain.
//!
//! NURBS surfaces have no closed form, so the contour is traced: `g` is
//! sampled on a grid aligned with the domain, marching squares pairs the
//! sign changes on cell edges into segments (a saddle cell resolved by its
//! centre value), each crossing is bisected along its cell edge onto
//! `g = 0`, and every segment whose middle strays more than the deflection
//! from its chord is split at a point bisected across it onto `g = 0`.
//! Every vertex therefore lies on the silhouette to bisection precision;
//! what the grid sets is which contours are found. A loop smaller than a
//! cell, or a contour that crosses one cell edge twice, can be missed, so
//! the result is an approximation and is reported as one.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};

/// Fewest and most grid intervals along one parameter direction.
const MIN_GRID_INTERVALS: usize = 16;
const MAX_GRID_INTERVALS: usize = 400;

/// Grid intervals per knot span, so a contour that turns within a span is
/// still sampled several times across it.
const INTERVALS_PER_SPAN: usize = 8;

/// A face whose normal stays this close to perpendicular to the view over
/// the whole grid is seen edge-on: its outline is its boundary edges, and
/// the sign of `g` is rounding noise.
const EDGE_ON_TOLERANCE: f64 = 1e-9;

/// Fraction of a grid interval by which the grid and the refinement keep
/// off the domain's edges. A domain edge can be collapsed to a point (a
/// sphere's pole, a cone's apex), where the normal is undefined and the
/// surface's fallback for it varies along the edge, so `g` sampled exactly
/// there changes sign without crossing zero.
const EDGE_INSET: f64 = 1e-6;

/// Bisection steps that place a crossing on `g = 0`: 2⁻⁴⁸ of a cell edge.
const ROOT_BISECTIONS: usize = 48;

/// Halvings of one contour segment while refining it to the deflection.
const MAX_REFINEMENT_DEPTH: u32 = 16;

/// One traced contour: parameter points on `g = 0`, in order. A closed
/// contour does not repeat its first point.
pub(in crate::projection) struct Contour {
    pub(in crate::projection) uv: Vec<(f64, f64)>,
    pub(in crate::projection) closed: bool,
}

/// `n·d` at `(u, v)`; a point whose normal cannot be resolved even by the
/// surface's degenerate fallback counts as on the silhouette's boundary
/// side, `0`.
fn g(surface: &NurbsSurface, view: Vec3, (u, v): (f64, f64)) -> f64 {
    surface.normal(u, v).map_or(0.0, |n| n.dot(view))
}

/// Grid parameters along one direction: `intervals + 1` values spanning
/// `[lo, hi]` inset by [`EDGE_INSET`] of an interval at each end, with
/// interval count from the knot spans.
fn grid_axis(knots: &[f64], (lo, hi): (f64, f64)) -> Vec<f64> {
    let mut spans = 0usize;
    for pair in knots.windows(2) {
        if pair[1] > pair[0] && pair[0] >= lo && pair[1] <= hi {
            spans += 1;
        }
    }
    let intervals = spans
        .saturating_mul(INTERVALS_PER_SPAN)
        .clamp(MIN_GRID_INTERVALS, MAX_GRID_INTERVALS);
    let inset = (hi - lo) / intervals as f64 * EDGE_INSET;
    let (lo, hi) = (lo + inset, hi - inset);
    (0..=intervals)
        .map(|i| lo + (hi - lo) * i as f64 / intervals as f64)
        .collect()
}

/// The point between `a` (where `g` has sign `a_positive`) and `b` where
/// `g` changes sign, by bisection on the segment `a`–`b`.
fn bisect(
    surface: &NurbsSurface,
    view: Vec3,
    mut a: (f64, f64),
    mut b: (f64, f64),
    a_positive: bool,
) -> (f64, f64) {
    for _ in 0..ROOT_BISECTIONS {
        let mid = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
        if (g(surface, view, mid) > 0.0) == a_positive {
            a = mid;
        } else {
            b = mid;
        }
    }
    (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1))
}

/// Trace the silhouette contours of `surface` in view `view`.
///
/// Returns `None` when the grid would take more than `grid_budget` normal
/// evaluations (which it is charged) or the contours more than
/// `max_vertices` vertices.
pub(in crate::projection) fn contours(
    surface: &NurbsSurface,
    view: Vec3,
    deflection: f64,
    grid_budget: &mut usize,
    max_vertices: usize,
) -> Option<Vec<Contour>> {
    let us = grid_axis(surface.knots_u(), surface.domain_u());
    let vs = grid_axis(surface.knots_v(), surface.domain_v());
    let (nu, nv) = (us.len(), vs.len());
    *grid_budget = grid_budget.checked_sub(nu * nv)?;

    let values: Vec<f64> = vs
        .iter()
        .flat_map(|&v| us.iter().map(move |&u| (u, v)))
        .map(|p| g(surface, view, p))
        .collect();
    if values.iter().all(|value| value.abs() <= EDGE_ON_TOLERANCE) {
        return Some(Vec::new());
    }
    let node = |i: usize, j: usize| (us[i], vs[j]);
    let positive = |i: usize, j: usize| values[j * nu + i] > 0.0;

    // Crossings on horizontal edges (i, j)–(i + 1, j) and vertical edges
    // (i, j)–(i, j + 1), each bisected onto g = 0 once.
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut crossing = |i0: usize, j0: usize, i1: usize, j1: usize| -> Option<usize> {
        let (p0, p1) = (positive(i0, j0), positive(i1, j1));
        if p0 == p1 {
            return None;
        }
        points.push(bisect(surface, view, node(i0, j0), node(i1, j1), p0));
        Some(points.len() - 1)
    };
    let horizontal: Vec<Option<usize>> = (0..nv)
        .flat_map(|j| (0..nu - 1).map(move |i| (i, j)))
        .map(|(i, j)| crossing(i, j, i + 1, j))
        .collect();
    let vertical: Vec<Option<usize>> = (0..nv - 1)
        .flat_map(|j| (0..nu).map(move |i| (i, j)))
        .map(|(i, j)| crossing(i, j, i, j + 1))
        .collect();
    let h = |i: usize, j: usize| horizontal[j * (nu - 1) + i];
    let v = |i: usize, j: usize| vertical[j * nu + i];

    // Marching squares: each cell links the crossings on its edges in pairs.
    let mut links: Vec<Vec<usize>> = vec![Vec::new(); points.len()];
    let mut link = |a: usize, b: usize| {
        links[a].push(b);
        links[b].push(a);
    };
    for j in 0..nv - 1 {
        for i in 0..nu - 1 {
            let (bottom, top, left, right) = (h(i, j), h(i, j + 1), v(i, j), v(i + 1, j));
            if let (Some(b), Some(t), Some(l), Some(r)) = (bottom, top, left, right) {
                // A saddle: the corners alternate in sign. If the centre
                // shares the (i, j) corner's sign, that corner's region runs
                // through the middle and the contours cut off the other two
                // corners, (i + 1, j) and (i, j + 1); otherwise they cut off
                // (i, j) and (i + 1, j + 1).
                let centre = (0.5 * (us[i] + us[i + 1]), 0.5 * (vs[j] + vs[j + 1]));
                if (g(surface, view, centre) > 0.0) == positive(i, j) {
                    link(b, r);
                    link(t, l);
                } else {
                    link(b, l);
                    link(t, r);
                }
            } else {
                let present: Vec<usize> =
                    [bottom, top, left, right].into_iter().flatten().collect();
                if let [a, b] = present[..] {
                    link(a, b);
                }
            }
        }
    }

    // Chain the links: open chains from their free ends first, then cycles.
    let mut visited = vec![false; points.len()];
    let mut chains: Vec<Contour> = Vec::new();
    let walk = |start: usize, visited: &mut Vec<bool>| -> Vec<usize> {
        let mut chain = vec![start];
        visited[start] = true;
        let mut current = start;
        while let Some(&next) = links[current].iter().find(|&&n| !visited[n]) {
            visited[next] = true;
            chain.push(next);
            current = next;
        }
        chain
    };
    for start in 0..points.len() {
        if !visited[start] && links[start].len() < 2 {
            let chain = walk(start, &mut visited);
            if chain.len() >= 2 {
                chains.push(Contour {
                    uv: chain.into_iter().map(|k| points[k]).collect(),
                    closed: false,
                });
            }
        }
    }
    for start in 0..points.len() {
        if !visited[start] {
            let chain = walk(start, &mut visited);
            if chain.len() >= 3 {
                chains.push(Contour {
                    uv: chain.into_iter().map(|k| points[k]).collect(),
                    closed: true,
                });
            }
        }
    }

    // Refine each chain to the deflection, inside the grid's inset domain.
    let (u_lo, u_hi, v_lo, v_hi) = (us[0], us[nu - 1], vs[0], vs[nv - 1]);
    let clamp = |(u, v): (f64, f64)| (u.clamp(u_lo, u_hi), v.clamp(v_lo, v_hi));
    let mut vertices = 0usize;
    let mut refined: Vec<Contour> = Vec::with_capacity(chains.len());
    for chain in chains {
        let n = chain.uv.len();
        let segments = if chain.closed { n } else { n - 1 };
        let mut uv = Vec::with_capacity(n);
        for k in 0..segments {
            let (a, b) = (chain.uv[k], chain.uv[(k + 1) % n]);
            uv.push(a);
            refine(surface, view, deflection, clamp, a, b, 0, &mut uv);
        }
        if !chain.closed {
            uv.push(chain.uv[n - 1]);
        }
        vertices = vertices.saturating_add(uv.len());
        if vertices > max_vertices {
            return None;
        }
        refined.push(Contour {
            uv,
            closed: chain.closed,
        });
    }
    Some(refined)
}

/// Push the points strictly between `a` and `b` that bring the polyline
/// within `deflection` of the silhouette: the middle of `a`–`b` is moved
/// across the segment onto `g = 0` by bisection, and kept, recursively,
/// while it lies more than `deflection` from the chord in space. A middle
/// whose cross-section shows no sign change is left on the chord.
#[allow(clippy::too_many_arguments)]
fn refine(
    surface: &NurbsSurface,
    view: Vec3,
    deflection: f64,
    clamp: impl Fn((f64, f64)) -> (f64, f64) + Copy,
    a: (f64, f64),
    b: (f64, f64),
    depth: u32,
    out: &mut Vec<(f64, f64)>,
) {
    if depth >= MAX_REFINEMENT_DEPTH {
        return;
    }
    let mid = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
    // Perpendicular to the segment in parameter space, as long as it.
    let across = (a.1 - b.1, b.0 - a.0);
    let lo = clamp((mid.0 - across.0, mid.1 - across.1));
    let hi = clamp((mid.0 + across.0, mid.1 + across.1));
    let lo_positive = g(surface, view, lo) > 0.0;
    if lo_positive == (g(surface, view, hi) > 0.0) {
        return;
    }
    let on = bisect(surface, view, lo, hi, lo_positive);
    let (pa, pb) = (surface.evaluate(a.0, a.1), surface.evaluate(b.0, b.1));
    let chord_mid = Point3::new(
        0.5 * (pa.x() + pb.x()),
        0.5 * (pa.y() + pb.y()),
        0.5 * (pa.z() + pb.z()),
    );
    if (surface.evaluate(on.0, on.1) - chord_mid).length() <= deflection {
        return;
    }
    refine(surface, view, deflection, clamp, a, on, depth + 1, out);
    out.push(on);
    refine(surface, view, deflection, clamp, on, b, depth + 1, out);
}
