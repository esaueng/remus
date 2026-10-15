//! Certified completion of the seed-grid line/surface intersection.
//!
//! The seed grid of [`super::intersect_line_nurbs_with_grid`] can miss a
//! crossing whose Newton basin falls between its nodes: a steep, narrow bump
//! narrower than about two grid cells leaves no seed on its far flank, and
//! Newton from the nearest seeds slides off the bump to another root. This
//! pass finds such roots by root isolation rather than sampling.
//!
//! The line is the intersection of two planes through it, with unit normals
//! `n1`, `n2`. Over any parameter box the surface lies in the convex hull of
//! the control points governing that box (positive weights), so the signed
//! distances `f = n1 · (S - O)` and `g = n2 · (S - O)` lie between the minimum
//! and maximum of their control values. A box can hold a root only when both
//! value ranges contain zero (Nishita, Sederberg, Kakimoto, "Ray tracing
//! trimmed rational surface patches", SIGGRAPH 1990). Boxes are rejected by:
//!
//! 1. a tree of control-point AABBs over blocks of `BLOCK × BLOCK` knot
//!    spans (built once per surface), tested against the line by the slab
//!    method, then the block's control values;
//! 2. the B-spline control values of one knot span;
//! 3. the span's Bézier control values (homogeneous `w · f`, `w · g`, which
//!    share the sign and the zeros of `f`, `g`), split by de Casteljau
//!    until both nets are nearly affine (so Newton converges from the box
//!    centre) and their interval Jacobian is nonsingular (so the box holds
//!    at most one root);
//! 4. a Bézier box whose affine model, widened by its proven deviation from
//!    the nets, puts every root outside the box.
//!
//! Each surviving box runs the shared Newton refinement clamped to the box
//! (with a margin), so it cannot slide off to a different root. It accepts
//! a root only when both plane distances from the line are within the
//! rounding the arithmetic producing them can commit ([`ROUNDOFF`] per axis,
//! weighted by each plane normal, the line origin rebased next to the
//! surface): a root this pass reports is a genuine crossing, never the grid
//! pass's looser `1e-5` near-miss, whatever the surface's size, shape or
//! position. A box proven to
//! hold at most one root that already holds a reported one is not refined
//! again. A box that reaches the split-depth limit unproven (a near-tangent
//! line) is always refined.
//!
//! Work is bounded per line by [`MAX_BEZIER_NODES`] and
//! [`MAX_LEAF_REFINEMENTS`]. A line lying in or grazing along the surface can
//! exhaust them; the pass then stops and the result is the grid result plus
//! whatever was found so far, never worse than the grid alone.
//!
//! Arithmetic in the loops is plain products and sums, not `mul_add`: off
//! `wasm32` and without an FMA target feature that is a library call, and
//! these loops are all multiply-adds. The once-per-line origin rebase is the
//! exception, where a single rounding keeps the rebased origin on the line.

use std::ops::Range;

use crate::aabb::Aabb3;
use crate::fma::FusedMulAdd;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::{Point3, Vec3};

use super::{IntersectionPoint, LineQuery, NewtonWorkspace, refine_line_surface_point};

/// Bézier boxes examined per line (all spans together).
const MAX_BEZIER_NODES: usize = 4096;
/// Newton refinements started per line.
const MAX_LEAF_REFINEMENTS: usize = 256;
/// de Casteljau halvings within one knot span: 2^-15 of the span per
/// direction when split evenly.
const MAX_SPLIT_DEPTH: usize = 30;
/// A net is nearly affine when its deviation from the affine part is at most
/// this fraction of its value range.
const AFFINE_FRACTION: f64 = 0.05;
/// Parameter margin around a leaf box for its clamped Newton, as a fraction
/// of the box extent per side.
const LEAF_MARGIN: f64 = 0.25;
/// Round-off allowance per unit of coordinate magnitude, taken per axis (the
/// larger of the surface's control points and the rebased line origin along
/// that axis), with margin for high-degree and rational evaluation. It sets
/// the slab-test padding per axis, and the sign-test slack and acceptance
/// bound of each plane distance `n · (P - O)` as `Σ |n_i| · allowance_i`: an
/// axis the plane normal does not use cannot round its distance. No geometric
/// term: a tolerance scaled by the surface's size accepts real misses on a
/// large or elongated surface, and a large coordinate along the line does
/// not loosen the perpendicular test.
const ROUNDOFF: f64 = 256.0 * f64::EPSILON;

/// Knot spans per direction in one leaf block of the span tree. A leaf
/// box covers a block of up to `BLOCK × BLOCK` spans, which keeps the tree
/// (built once per surface, and once per call by the one-shot entry point)
/// small on large surfaces; spans inside a block are screened by their
/// control values instead.
const BLOCK: usize = 4;

/// Per-surface data of the completion pass: the non-empty knot spans and an
/// AABB tree over blocks of them.
#[derive(Debug, Clone)]
pub(super) struct SpanHull {
    degree_u: usize,
    degree_v: usize,
    /// Knot index `k` of every non-empty span `[t_k, t_{k+1})`.
    u_spans: Vec<usize>,
    v_spans: Vec<usize>,
    /// Control-point boxes of the implicit block-range tree, in preorder.
    boxes: Vec<Aabb3>,
    /// Largest absolute control-point coordinate, per axis.
    axis_magnitude: [f64; 3],
    /// Control-point grid size, with the degrees the layout the spans index.
    rows: usize,
    cols: usize,
}

impl SpanHull {
    pub(super) fn for_surface(surface: &NurbsSurface) -> Self {
        let degree_u = surface.degree_u();
        let degree_v = surface.degree_v();
        let cps = surface.control_points();
        let u_spans = nonempty_spans(surface.knots_u(), degree_u, cps.len());
        let v_spans = nonempty_spans(surface.knots_v(), degree_v, cps.first().map_or(0, Vec::len));

        let mut hull = Self {
            degree_u,
            degree_v,
            u_spans,
            v_spans,
            boxes: Vec::new(),
            axis_magnitude: [0.0; 3],
            rows: cps.len(),
            cols: cps.first().map_or(0, Vec::len),
        };
        let root = hull.root();
        if root.leaves() > 0 {
            hull.boxes.reserve(2 * root.leaves() - 1);
            let bounds = hull.build_box(surface, root);
            for axis in 0..3 {
                hull.axis_magnitude[axis] = bounds.min.0[axis].abs().max(bounds.max.0[axis].abs());
            }
        }
        hull
    }

    /// Whether `surface` has the layout this hull indexes. A grid reused with
    /// a different surface is meaningless but must not index out of range.
    fn fits(&self, surface: &NurbsSurface) -> bool {
        let cps = surface.control_points();
        surface.degree_u() == self.degree_u
            && surface.degree_v() == self.degree_v
            && cps.len() == self.rows
            && cps.first().map_or(0, Vec::len) == self.cols
    }

    /// The whole block grid.
    fn root(&self) -> BlockRange {
        BlockRange {
            u0: 0,
            u1: self.u_spans.len().div_ceil(BLOCK),
            v0: 0,
            v1: self.v_spans.len().div_ceil(BLOCK),
        }
    }

    /// Span indices (into `u_spans`, `v_spans`) of block `(bu, bv)`.
    fn block_spans(&self, bu: usize, bv: usize) -> (Range<usize>, Range<usize>) {
        (
            bu * BLOCK..(bu * BLOCK + BLOCK).min(self.u_spans.len()),
            bv * BLOCK..(bv * BLOCK + BLOCK).min(self.v_spans.len()),
        )
    }

    /// Control-point rows and columns governing the spans `us × vs`.
    fn control_window(&self, us: &Range<usize>, vs: &Range<usize>) -> (Range<usize>, Range<usize>) {
        (
            self.u_spans[us.start] - self.degree_u..self.u_spans[us.end - 1] + 1,
            self.v_spans[vs.start] - self.degree_v..self.v_spans[vs.end - 1] + 1,
        )
    }

    /// Push the preorder boxes of `range`'s subtree; returns its root box.
    fn build_box(&mut self, surface: &NurbsSurface, range: BlockRange) -> Aabb3 {
        let slot = self.boxes.len();
        let placeholder = Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(0.0, 0.0, 0.0),
        };
        self.boxes.push(placeholder);
        let bounds = if let Some((left, right)) = range.split() {
            let a = self.build_box(surface, left);
            let b = self.build_box(surface, right);
            a.union(b)
        } else {
            let (us, vs) = self.block_spans(range.u0, range.v0);
            let (rows, cols) = self.control_window(&us, &vs);
            let cps = surface.control_points();
            // A window spans at least one span's `(p + 1) × (q + 1)` points.
            let first = cps[rows.start][cols.start];
            let (mut lo, mut hi) = (first.0, first.0);
            for row in &cps[rows] {
                for point in &row[cols.clone()] {
                    // Plain comparisons: control points are validated
                    // finite, and NaN-aware `min`/`max` nearly tripled the build.
                    for axis in 0..3 {
                        let c = point.0[axis];
                        if c < lo[axis] {
                            lo[axis] = c;
                        }
                        if c > hi[axis] {
                            hi[axis] = c;
                        }
                    }
                }
            }
            Aabb3 {
                min: Point3::new(lo[0], lo[1], lo[2]),
                max: Point3::new(hi[0], hi[1], hi[2]),
            }
        };
        self.boxes[slot] = bounds;
        bounds
    }
}

/// A rectangle of block indices `[u0, u1) × [v0, v1)`.
#[derive(Clone, Copy)]
struct BlockRange {
    u0: usize,
    u1: usize,
    v0: usize,
    v1: usize,
}

impl BlockRange {
    const fn leaves(self) -> usize {
        (self.u1 - self.u0) * (self.v1 - self.v0)
    }

    /// Halve the longer side; `None` for a single block. The tree is full
    /// binary, so a subtree over `n` blocks holds `2n - 1` boxes and the
    /// right child sits that many slots after the left one.
    const fn split(self) -> Option<(Self, Self)> {
        let (nu, nv) = (self.u1 - self.u0, self.v1 - self.v0);
        if nu * nv <= 1 {
            None
        } else if nu >= nv {
            let mid = self.u0 + nu / 2;
            Some((Self { u1: mid, ..self }, Self { u0: mid, ..self }))
        } else {
            let mid = self.v0 + nv / 2;
            Some((Self { v1: mid, ..self }, Self { v0: mid, ..self }))
        }
    }
}

/// Knot indices `k ∈ [p, n)` with `t_k < t_{k+1}`.
fn nonempty_spans(knots: &[f64], degree: usize, n_ctrl: usize) -> Vec<usize> {
    (degree..n_ctrl)
        .filter(|&k| knots[k] < knots[k + 1])
        .collect()
}

/// Bézier extraction operator of span `k` into `out` (row-major
/// `(p + 1)²`: Bézier value `r` is `Σ_a C[r][a] · P[k - p + a]`), from the
/// blossom: Bézier control value `r` of span `[a, b]` is the blossom at
/// `(a^(p-r), b^r)`, evaluated by de Boor's recurrence with those arguments
/// on all local basis vectors at once (`d[i * m + basis]`), so each `alpha`
/// is computed once for all of them. Computed per visited span rather than
/// stored: building every span's operator costs more than a whole line query
/// on a large surface.
fn extraction_operator(knots: &[f64], p: usize, k: usize, d: &mut Vec<f64>, out: &mut Vec<f64>) {
    let m = p + 1;
    let (a, b) = (knots[k], knots[k + 1]);
    out.clear();
    d.resize(m * m, 0.0);
    for r in 0..m {
        d.fill(0.0);
        for i in 0..m {
            d[i * m + i] = 1.0;
        }
        for level in 1..=p {
            let x = if level <= p - r { a } else { b };
            for i in (level..=p).rev() {
                let gi = k - p + i;
                // `gi <= k < k + 1 <= gi + p + 1 - level`, and the span is
                // non-empty, so the denominator is positive.
                let lo = knots[gi];
                let alpha = (x - lo) / (knots[gi + p + 1 - level] - lo);
                for basis in 0..m {
                    d[i * m + basis] =
                        (1.0 - alpha) * d[(i - 1) * m + basis] + alpha * d[i * m + basis];
                }
            }
        }
        out.extend_from_slice(&d[p * m..]);
    }
}

/// Whether the infinite line meets `bounds` padded by `pad` per axis (slab
/// method).
fn line_meets_box(origin: Point3, dir: Vec3, bounds: Aabb3, pad: [f64; 3]) -> bool {
    let (mut t_lo, mut t_hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for axis in 0..3 {
        let o = origin.0[axis];
        let d = dir.0[axis];
        let lo = bounds.min.0[axis] - pad[axis];
        let hi = bounds.max.0[axis] + pad[axis];
        // Only an exactly zero component is parallel: a tiny one still
        // crosses the slab far along a long box, and its quotients below
        // are finite or correctly signed infinities.
        if d == 0.0 {
            if o < lo || o > hi {
                return false;
            }
            continue;
        }
        let (t1, t2) = ((lo - o) / d, (hi - o) / d);
        t_lo = t_lo.max(t1.min(t2));
        t_hi = t_hi.min(t1.max(t2));
        if t_lo > t_hi {
            return false;
        }
    }
    true
}

/// Whether every value is on one side of zero by more than `slack`, the
/// rounding allowance of the computed control values.
fn one_signed(values: &[f64], slack: f64) -> bool {
    values.iter().all(|&x| x > slack) || values.iter().all(|&x| x < -slack)
}

/// The affine part of a tensor Bézier net over its box, in box coordinates
/// `(s, r) ∈ [0, 1]²`: `centre + ds · (s - ½) + dr · (r - ½)`, with `error`
/// bounding the function's distance from it over the whole box.
///
/// The bilinear interpolant of the corner values has, by linear precision,
/// the Bézier values of the bilinear function at the Greville points, so
/// the function stays within the largest control-value deviation `dev` of
/// it; the bilinear interpolant differs from its affine part by
/// `twist · (s - ½)(r - ½)`, at most `|twist| / 4`.
struct AffineFit {
    centre: f64,
    ds: f64,
    dr: f64,
    error: f64,
    range: f64,
}

fn affine_fit(net: &[f64], p: usize, q: usize) -> AffineFit {
    let at = |i: usize, j: usize| net[i * (q + 1) + j];
    let (c00, c10, c01, c11) = (at(0, 0), at(p, 0), at(0, q), at(p, q));
    let (mut lo, mut hi, mut dev) = (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64);
    for i in 0..=p {
        #[allow(clippy::cast_precision_loss)]
        let s = i as f64 / p as f64;
        for j in 0..=q {
            #[allow(clippy::cast_precision_loss)]
            let r = j as f64 / q as f64;
            let value = at(i, j);
            lo = lo.min(value);
            hi = hi.max(value);
            let bilinear =
                (1.0 - s) * ((1.0 - r) * c00 + r * c01) + s * ((1.0 - r) * c10 + r * c11);
            dev = dev.max((value - bilinear).abs());
        }
    }
    let twist = (c00 - c10 - c01 + c11).abs();
    AffineFit {
        centre: 0.25 * (c00 + c10 + c01 + c11),
        ds: 0.5 * ((c10 + c11) - (c00 + c01)),
        dr: 0.5 * ((c01 + c11) - (c00 + c10)),
        error: dev + 0.25 * twist,
        range: hi - lo,
    }
}

impl AffineFit {
    /// Within `AFFINE_FRACTION` of affine, relative to the value range.
    fn nearly_affine(&self) -> bool {
        self.error <= AFFINE_FRACTION * self.range
    }
}

/// Whether the pair `(f, g)` provably has no root in the box: a root there
/// solves `A · (s - ½, r - ½) = -(centre + e)` with `|e| <= error`, so it
/// lies within `|A⁻¹| · error` of the affine model's root; when the model's
/// root is farther outside the box than that, there is none. A singular
/// model proves nothing.
fn affine_excludes_root(f: &AffineFit, g: &AffineFit) -> bool {
    let det = f.ds * g.dr - f.dr * g.ds;
    if det == 0.0 || !det.is_finite() {
        return false;
    }
    // Offsets of the model root from the centre, and how far a true root
    // can sit from it, per coordinate.
    let s = (f.dr * g.centre - g.dr * f.centre) / det;
    let r = (g.ds * f.centre - f.ds * g.centre) / det;
    let s_slack = (g.dr.abs() * f.error + f.dr.abs() * g.error) / det.abs();
    let r_slack = (g.ds.abs() * f.error + f.ds.abs() * g.error) / det.abs();
    s.abs() > 0.5 + s_slack || r.abs() > 0.5 + r_slack
}

/// Range `[lo, hi]` of each partial derivative of a tensor Bézier net over
/// its box: the derivative is a Bézier net of the scaled forward
/// differences, so its convex hull bounds it.
fn derivative_ranges(net: &[f64], p: usize, q: usize) -> ((f64, f64), (f64, f64)) {
    let at = |i: usize, j: usize| net[i * (q + 1) + j];
    let (mut ds, mut dr) = (
        (f64::INFINITY, f64::NEG_INFINITY),
        (f64::INFINITY, f64::NEG_INFINITY),
    );
    #[allow(clippy::cast_precision_loss)]
    let (pf, qf) = (p as f64, q as f64);
    for i in 0..=p {
        for j in 0..=q {
            if i < p {
                let d = pf * (at(i + 1, j) - at(i, j));
                ds = (ds.0.min(d), ds.1.max(d));
            }
            if j < q {
                let d = qf * (at(i, j + 1) - at(i, j));
                dr = (dr.0.min(d), dr.1.max(d));
            }
        }
    }
    (ds, dr)
}

/// Product of two intervals.
fn interval_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let products = [a.0 * b.0, a.0 * b.1, a.1 * b.0, a.1 * b.1];
    products
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &x| {
            (lo.min(x), hi.max(x))
        })
}

/// Whether `(f, g)` has at most one zero in the box.
///
/// By the mean value theorem, row by row, `F(x) - F(y) = M (x - y)` with
/// each row of `M` a gradient of one component at some point of the box, so
/// inside the interval matrix of the derivative ranges. When no matrix there
/// is singular (the interval determinant excludes zero), `F(x) = F(y)`
/// forces `x = y`. Nearly affine nets alone do not suffice: two nearly
/// parallel components can each be nearly affine and still share two zeros.
fn at_most_one_root(fnet: &[f64], gnet: &[f64], p: usize, q: usize) -> bool {
    let (fs, fr) = derivative_ranges(fnet, p, q);
    let (gs, gr) = derivative_ranges(gnet, p, q);
    let main = interval_mul(fs, gr);
    let cross = interval_mul(fr, gs);
    let det = (main.0 - cross.1, main.1 - cross.0);
    det.0 > 0.0 || det.1 < 0.0
}

/// Halve a tensor Bézier net at the parameter midpoint along u (`along_u`) or
/// v, writing the two halves into `low` and `high`.
fn split_net(net: &[f64], p: usize, q: usize, along_u: bool, low: &mut [f64], high: &mut [f64]) {
    let (outer, degree) = if along_u { (q + 1, p) } else { (p + 1, q) };
    let index = |line: usize, k: usize| {
        if along_u {
            k * (q + 1) + line
        } else {
            line * (q + 1) + k
        }
    };
    let mut work = [0.0_f64; 32];
    let mut heap;
    let row: &mut [f64] = if degree < work.len() {
        &mut work[..=degree]
    } else {
        heap = vec![0.0; degree + 1];
        &mut heap
    };
    for line in 0..outer {
        for k in 0..=degree {
            row[k] = net[index(line, k)];
        }
        low[index(line, 0)] = row[0];
        high[index(line, degree)] = row[degree];
        for level in 1..=degree {
            for k in 0..=degree - level {
                row[k] = 0.5 * (row[k] + row[k + 1]);
            }
            low[index(line, level)] = row[0];
            high[index(line, degree - level)] = row[degree - level];
        }
    }
}

/// Total variation of a net along u and along v.
fn variation(net: &[f64], p: usize, q: usize) -> (f64, f64) {
    let at = |i: usize, j: usize| net[i * (q + 1) + j];
    let (mut along_u, mut along_v) = (0.0, 0.0);
    for i in 0..=p {
        for j in 0..=q {
            if i < p {
                along_u += (at(i + 1, j) - at(i, j)).abs();
            }
            if j < q {
                along_v += (at(i, j + 1) - at(i, j)).abs();
            }
        }
    }
    (along_u, along_v)
}

/// A pending Bézier box: its parameter rectangle, split depth and the offset
/// of its `f` then `g` nets in the shared net stack.
#[derive(Clone, Copy)]
struct BezierBox {
    u: (f64, f64),
    v: (f64, f64),
    depth: usize,
    offset: usize,
}

struct Completion<'a> {
    surface: &'a NurbsSurface,
    hull: &'a SpanHull,
    line: LineQuery,
    n1: Vec3,
    n2: Vec3,
    pad: [f64; 3],
    /// Rounding allowance of the homogeneous `f` and `g` control values.
    slack_f: f64,
    slack_g: f64,
    /// Newton's own (3D) acceptance; [`Perpendicular`] then decides.
    tolerance: f64,
    perpendicular: Perpendicular,
    nodes: usize,
    refinements: usize,
    scratch: Scratch,
}

/// Buffers reused across the spans of one line query.
#[derive(Default)]
struct Scratch {
    /// Homogeneous plane distances over a block's control window.
    window_f: Vec<f64>,
    window_g: Vec<f64>,
    /// One span's B-spline values, then its Bézier nets.
    f: Vec<f64>,
    g: Vec<f64>,
    partial: Vec<f64>,
    /// Extraction operators: the u one of span `cu_key`, and the v ones in
    /// `BLOCK` slots keyed by v span (`vi % BLOCK`), so a block computes each
    /// at most once; plus the de Boor workspace.
    cu: Vec<f64>,
    cu_key: Option<usize>,
    cv: Vec<f64>,
    cv_slots: Vec<f64>,
    cv_keys: Vec<Option<usize>>,
    deboor: Vec<f64>,
    /// Stack of pending boxes and their `f`, `g` nets.
    pending: Vec<BezierBox>,
    nets: Vec<f64>,
    fg: Vec<f64>,
    halves: Vec<f64>,
}

/// Append the roots of `line` on `surface` that `results` (the grid pass)
/// missed. Never removes or reorders an existing result.
pub(super) fn complete_line_roots(
    surface: &NurbsSurface,
    hull: &SpanHull,
    line: LineQuery,
    workspace: &mut NewtonWorkspace,
    results: &mut Vec<IntersectionPoint>,
) {
    if hull.boxes.is_empty() || !hull.fits(surface) {
        return;
    }
    let caller_origin = line.origin;
    // The same line through the point nearest the surface's box centre: the
    // control values and Newton residuals are differences from the origin,
    // so a far origin would otherwise inflate the round-off allowance below
    // although sliding the origin along the line changes nothing.
    let centre = hull.boxes[0].center();
    let along = (centre - line.origin).dot(line.direction) / line.dir_len_sq;
    // One rounding per component (`fma`): `O + d·s` with a far origin
    // cancels, and rounding `d_i·s` separately would shift the line sideways
    // by ~ulp(|O|). Rounding in `s` itself only slides the origin along it.
    let (o, d) = (line.origin, line.direction);
    let line = LineQuery {
        origin: Point3::new(
            d.x().fma(along, o.x()),
            d.y().fma(along, o.y()),
            d.z().fma(along, o.z()),
        ),
        ..line
    };
    let dir_len = line.dir_len_sq.sqrt();
    let mut pad = [0.0; 3];
    for axis in 0..3 {
        pad[axis] = ROUNDOFF * hull.axis_magnitude[axis].max(line.origin.0[axis].abs());
    }
    if !line_meets_box(line.origin, line.direction, hull.boxes[0], pad) {
        return;
    }
    let unit = line.direction * (1.0 / dir_len);
    // Any axis not parallel to the line gives a frame; take the least
    // aligned one.
    let axis = if unit.x().abs() <= unit.y().abs() && unit.x().abs() <= unit.z().abs() {
        Vec3::new(1.0, 0.0, 0.0)
    } else if unit.y().abs() <= unit.z().abs() {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let n1 = unit.cross(axis);
    let n1 = n1 * (1.0 / n1.length());
    let n2 = unit.cross(n1);
    let plane_slack = |n: Vec3| (0..3).map(|axis| n.0[axis].abs() * pad[axis]).sum::<f64>();
    let (slack_f, slack_g) = (plane_slack(n1), plane_slack(n2));

    let mut pass = Completion {
        surface,
        hull,
        line,
        n1,
        n2,
        pad,
        // Weights enter divided by the block's largest (see `block`), so the
        // homogeneous values are bounded by the plane distances.
        slack_f,
        slack_g,
        // Along the line a large coordinate legitimately rounds Newton's 3D
        // residual; the perpendicular test below is the strict one.
        tolerance: pad.iter().map(|a| a * a).sum::<f64>().sqrt(),
        perpendicular: Perpendicular {
            origin: line.origin,
            n1,
            n2,
            slack_f,
            slack_g,
        },
        nodes: 0,
        refinements: 0,
        scratch: Scratch::default(),
    };

    let reported = results.len();
    pass.traverse(workspace, results);
    // Line parameters of the appended roots relative to the caller's origin,
    // as the grid pass reports them.
    for hit in &mut results[reported..] {
        hit.param2.0 = (hit.point - caller_origin).dot(line.direction) / line.dir_len_sq;
    }
}

impl Completion<'_> {
    /// Walk the span-block tree along the line, solving every block it meets.
    fn traverse(&mut self, workspace: &mut NewtonWorkspace, results: &mut Vec<IntersectionPoint>) {
        let (hull, line) = (self.hull, self.line);
        let mut stack = vec![(0_usize, hull.root())];
        while let Some((slot, range)) = stack.pop() {
            if slot > 0 && !line_meets_box(line.origin, line.direction, hull.boxes[slot], self.pad)
            {
                continue;
            }
            match range.split() {
                Some((left, right)) => {
                    let right_slot = slot + 1 + (2 * left.leaves() - 1);
                    // Right first so the left subtree is visited first.
                    stack.push((right_slot, right));
                    stack.push((slot + 1, left));
                }
                None => {
                    if !self.block(range.u0, range.v0, workspace, results) {
                        return;
                    }
                }
            }
        }
    }

    /// Screen and solve every span of block `(bu, bv)`. Returns `false` once
    /// a work budget is exhausted.
    fn block(
        &mut self,
        bu: usize,
        bv: usize,
        workspace: &mut NewtonWorkspace,
        results: &mut Vec<IntersectionPoint>,
    ) -> bool {
        let hull = self.hull;
        let (slack_f, slack_g) = (self.slack_f, self.slack_g);
        let (us, vs) = hull.block_spans(bu, bv);
        let (rows, cols) = hull.control_window(&us, &vs);
        let cps = self.surface.control_points();
        let weights = self.surface.weights();
        let width = cols.len();
        let scratch = &mut self.scratch;
        scratch.window_f.clear();
        scratch.window_g.clear();
        // Weights enter divided by the block's largest, a positive factor that
        // changes no sign or zero: a common or regional weight factor leaves
        // the surface unchanged, and must neither overflow the products nor
        // shrink them below the rounding slack (a region of weights ~1e-200
        // beside weights ~1 would leave every value inside the slack).
        // Divided, not multiplied by a reciprocal, which overflows for a
        // subnormal factor.
        let block_weight = weights[rows.clone()]
            .iter()
            .flat_map(|row| row[cols.clone()].iter().copied())
            .fold(0.0_f64, f64::max);
        for i in rows.clone() {
            for j in cols.clone() {
                let rel = cps[i][j] - self.line.origin;
                let w = weights[i][j] / block_weight;
                scratch.window_f.push(w * self.n1.dot(rel));
                scratch.window_g.push(w * self.n2.dot(rel));
            }
        }
        if one_signed(&scratch.window_f, slack_f) || one_signed(&scratch.window_g, slack_g) {
            return true;
        }
        for ui in us {
            for vi in vs.clone() {
                let (ku, kv) = (hull.u_spans[ui], hull.v_spans[vi]);
                let (row0, col0) = (
                    ku - hull.degree_u - rows.start,
                    kv - hull.degree_v - cols.start,
                );
                let scratch = &mut self.scratch;
                scratch.f.clear();
                scratch.g.clear();
                for a in 0..=hull.degree_u {
                    let at = (row0 + a) * width + col0;
                    scratch
                        .f
                        .extend_from_slice(&scratch.window_f[at..=at + hull.degree_v]);
                    scratch
                        .g
                        .extend_from_slice(&scratch.window_g[at..=at + hull.degree_v]);
                }
                // Rescale to the span's own largest weight, for a span whose
                // weights sit far below the rest of its block.
                let span_weight = weights[ku - hull.degree_u..=ku]
                    .iter()
                    .flat_map(|row| row[kv - hull.degree_v..=kv].iter().copied())
                    .fold(0.0_f64, f64::max)
                    / block_weight;
                if span_weight < 1.0 {
                    for value in scratch.f.iter_mut().chain(scratch.g.iter_mut()) {
                        *value /= span_weight;
                    }
                }
                if one_signed(&scratch.f, slack_f) || one_signed(&scratch.g, slack_g) {
                    continue;
                }
                if !self.span(ui, vi, workspace, results) {
                    return false;
                }
            }
        }
        true
    }

    /// Isolate and refine the roots in one knot span, whose B-spline values
    /// are in `scratch.f`, `scratch.g`. Returns `false` once a work budget is
    /// exhausted.
    fn span(
        &mut self,
        ui: usize,
        vi: usize,
        workspace: &mut NewtonWorkspace,
        results: &mut Vec<IntersectionPoint>,
    ) -> bool {
        let hull = self.hull;
        let (p, q) = (hull.degree_u, hull.degree_v);
        let (ku, kv) = (hull.u_spans[ui], hull.v_spans[vi]);
        let len = (p + 1) * (q + 1);

        // Bézier nets of the span: C_u · H · C_vᵀ, pushed as the first entry
        // of the stack of pending boxes, each holding its `f` and `g` nets.
        let Scratch {
            f,
            g,
            partial,
            cu,
            cu_key,
            cv,
            cv_slots,
            cv_keys,
            deboor,
            pending,
            nets,
            fg,
            halves,
            ..
        } = &mut self.scratch;
        if *cu_key != Some(ui) {
            extraction_operator(self.surface.knots_u(), p, ku, deboor, cu);
            *cu_key = Some(ui);
        }
        let cv_len = (q + 1) * (q + 1);
        let slot = vi % BLOCK;
        if cv_keys.len() < BLOCK {
            cv_keys.resize(BLOCK, None);
            cv_slots.resize(BLOCK * cv_len, 0.0);
        }
        if cv_keys[slot] != Some(vi) {
            extraction_operator(self.surface.knots_v(), q, kv, deboor, cv);
            cv_slots[slot * cv_len..(slot + 1) * cv_len].copy_from_slice(cv);
            cv_keys[slot] = Some(vi);
        }
        let cv = &cv_slots[slot * cv_len..(slot + 1) * cv_len];
        nets.clear();
        partial.resize(len, 0.0);
        for source in [&*f, &*g] {
            for r in 0..=p {
                for b in 0..=q {
                    partial[r * (q + 1) + b] = (0..=p)
                        .map(|a| cu[r * (p + 1) + a] * source[a * (q + 1) + b])
                        .sum();
                }
            }
            for r in 0..=p {
                for s in 0..=q {
                    nets.push(
                        (0..=q)
                            .map(|b| partial[r * (q + 1) + b] * cv[s * (q + 1) + b])
                            .sum(),
                    );
                }
            }
        }
        let knots_u = self.surface.knots_u();
        let knots_v = self.surface.knots_v();
        pending.clear();
        pending.push(BezierBox {
            u: (knots_u[ku], knots_u[ku + 1]),
            v: (knots_v[kv], knots_v[kv + 1]),
            depth: 0,
            offset: 0,
        });
        fg.resize(2 * len, 0.0);
        halves.resize(4 * len, 0.0);

        while let Some(node) = pending.pop() {
            if self.nodes >= MAX_BEZIER_NODES {
                return false;
            }
            self.nodes += 1;
            // LIFO: the popped node's nets are the last ones on the stack.
            fg.copy_from_slice(&nets[node.offset..node.offset + 2 * len]);
            nets.truncate(node.offset);
            let (fnet, gnet) = fg.split_at(len);
            if one_signed(fnet, self.slack_f) || one_signed(gnet, self.slack_g) {
                continue;
            }
            let (f_fit, g_fit) = (affine_fit(fnet, p, q), affine_fit(gnet, p, q));
            if affine_excludes_root(&f_fit, &g_fit) {
                continue;
            }
            let at_depth_limit = node.depth >= MAX_SPLIT_DEPTH;
            let unique = f_fit.nearly_affine()
                && g_fit.nearly_affine()
                && at_most_one_root(fnet, gnet, p, q);
            if unique || at_depth_limit {
                let (surface, line, tolerance) = (self.surface, self.line, self.tolerance);
                let perpendicular = &self.perpendicular;
                if self.refinements >= MAX_LEAF_REFINEMENTS {
                    return false;
                }
                if refine_box(
                    surface,
                    line,
                    tolerance,
                    perpendicular,
                    node,
                    unique,
                    workspace,
                    results,
                ) {
                    self.refinements += 1;
                }
                continue;
            }
            let (fu, fv) = variation(fnet, p, q);
            let (gu, gv) = variation(gnet, p, q);
            let along_u = fu + gu >= fv + gv;
            let (low, high) = halves.split_at_mut(2 * len);
            split_net(fnet, p, q, along_u, &mut low[..len], &mut high[..len]);
            split_net(gnet, p, q, along_u, &mut low[len..], &mut high[len..]);
            let (lo_box, hi_box) = if along_u {
                let mid = 0.5 * (node.u.0 + node.u.1);
                (
                    BezierBox {
                        u: (node.u.0, mid),
                        ..node
                    },
                    BezierBox {
                        u: (mid, node.u.1),
                        ..node
                    },
                )
            } else {
                let mid = 0.5 * (node.v.0 + node.v.1);
                (
                    BezierBox {
                        v: (node.v.0, mid),
                        ..node
                    },
                    BezierBox {
                        v: (mid, node.v.1),
                        ..node
                    },
                )
            };
            for (half, net) in [(hi_box, &high[..]), (lo_box, &low[..])] {
                pending.push(BezierBox {
                    depth: node.depth + 1,
                    offset: nets.len(),
                    ..half
                });
                nets.extend_from_slice(net);
            }
        }
        true
    }
}

/// The strict acceptance of a completion root: both plane distances of the
/// point from the line within their own rounding allowance.
struct Perpendicular {
    origin: Point3,
    n1: Vec3,
    n2: Vec3,
    slack_f: f64,
    slack_g: f64,
}

impl Perpendicular {
    fn accepts(&self, point: Point3) -> bool {
        let rel = point - self.origin;
        self.n1.dot(rel).abs() <= self.slack_f && self.n2.dot(rel).abs() <= self.slack_g
    }
}

/// Newton from the box centre, clamped to the box plus a margin; appends a
/// new root. Returns whether a refinement ran: a box proven to hold at most
/// one root (`unique`) that already holds a reported one is skipped.
#[allow(clippy::too_many_arguments)]
fn refine_box(
    surface: &NurbsSurface,
    line: LineQuery,
    tolerance: f64,
    perpendicular: &Perpendicular,
    node: BezierBox,
    unique: bool,
    workspace: &mut NewtonWorkspace,
    results: &mut Vec<IntersectionPoint>,
) -> bool {
    let known = unique
        && results.iter().any(|hit| {
            let (u, v) = hit.param1;
            (node.u.0..=node.u.1).contains(&u) && (node.v.0..=node.v.1).contains(&v)
        });
    if known {
        return false;
    }
    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();
    let du = LEAF_MARGIN * (node.u.1 - node.u.0);
    let dv = LEAF_MARGIN * (node.v.1 - node.v.0);
    let bounds = [
        (node.u.0 - du).max(u_min),
        (node.u.1 + du).min(u_max),
        (node.v.0 - dv).max(v_min),
        (node.v.1 + dv).min(v_max),
    ];
    let u = 0.5 * (node.u.0 + node.u.1);
    let v = 0.5 * (node.v.0 + node.v.1);
    let seed = workspace.point(surface, u, v);
    if let Some(hit) = refine_line_surface_point(
        surface,
        line,
        (u, v),
        seed,
        bounds,
        perpendicular.slack_f.min(perpendicular.slack_g),
        tolerance,
        workspace,
    ) && perpendicular.accepts(hit.point)
    {
        let duplicate = results
            .iter()
            .any(|existing| (existing.point - hit.point).length() < 1e-6);
        if !duplicate {
            results.push(hit);
        }
    }
    true
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    /// Extraction reproduces the span's polynomial: evaluating the Bézier
    /// form of every span of a non-uniform cubic matches the B-spline.
    #[test]
    fn extraction_matches_bspline_basis() {
        let knots = [0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.5, 0.9, 1.0, 1.0, 1.0, 1.0];
        let p = 3;
        let n = knots.len() - p - 1;
        let spans = nonempty_spans(&knots, p, n);
        assert_eq!(spans, vec![3, 5, 6, 7]);
        let (mut d, mut c) = (Vec::new(), Vec::new());
        let values: Vec<f64> = (0..n)
            .map(|i| (i as f64).mul_add(0.7, (i * i) as f64 * -0.1))
            .collect();
        for &k in &spans {
            extraction_operator(&knots, p, k, &mut d, &mut c);
            let bez: Vec<f64> = (0..=p)
                .map(|r| (0..=p).map(|a| c[r * 4 + a] * values[k - p + a]).sum())
                .collect();
            for step in 0..=8 {
                let x = f64::from(step) / 8.0;
                let u = knots[k] + x * (knots[k + 1] - knots[k]);
                // Bernstein form of the Bézier coefficients.
                let bern = [
                    (1.0 - x).powi(3),
                    3.0 * x * (1.0 - x).powi(2),
                    3.0 * x * x * (1.0 - x),
                    x.powi(3),
                ];
                let from_bez: f64 = (0..=p).map(|r| bern[r] * bez[r]).sum();
                // The span's own polynomial piece, also at its closed ends.
                let basis = crate::nurbs::basis::basis_funs(k, u, p, &knots);
                let from_basis: f64 = (0..=p).map(|a| basis[a] * values[k - p + a]).sum();
                assert!(
                    (from_bez - from_basis).abs() < 1e-12,
                    "span {k} x {x}: {from_bez} vs {from_basis}"
                );
            }
        }
    }

    /// Degree-2×2 Bézier net of `h` on `[0, 1]²`, from its values at the
    /// 3×3 grid of `0, ½, 1` (`b1 = 2·h(½) - (h(0) + h(1)) / 2` per direction).
    fn quadratic_net(h: impl Fn(f64, f64) -> f64) -> Vec<f64> {
        let to_bernstein = |v: [f64; 3]| [v[0], 2.0 * v[1] - 0.5 * (v[0] + v[2]), v[2]];
        let t = [0.0, 0.5, 1.0];
        let rows: Vec<[f64; 3]> = t
            .iter()
            .map(|&s| to_bernstein([h(s, 0.0), h(s, 0.5), h(s, 1.0)]))
            .collect();
        let mut net = vec![0.0; 9];
        for j in 0..3 {
            let column = to_bernstein([rows[0][j], rows[1][j], rows[2][j]]);
            for i in 0..3 {
                net[i * 3 + j] = column[i];
            }
        }
        net
    }

    /// Two nearly parallel, nearly affine components can still share two
    /// zeros; such a box is no leaf. `f = s + r - 1` and
    /// `g = f + ε((s - r)² - a²)` meet at `s - r = ±a` on `s + r = 1`.
    #[test]
    fn nearly_affine_pair_with_two_roots_is_not_proven_unique() {
        let (eps, a) = (0.01, 0.3);
        let f = quadratic_net(|s, r| s + r - 1.0);
        let g = quadratic_net(|s, r| s + r - 1.0 + eps * ((s - r).powi(2) - a * a));
        assert!(affine_fit(&f, 2, 2).nearly_affine());
        assert!(affine_fit(&g, 2, 2).nearly_affine());
        assert!(!at_most_one_root(&f, &g, 2, 2));
        // Transversal components prove uniqueness.
        let across = quadratic_net(|s, r| s - r + 0.1 * s * s);
        assert!(at_most_one_root(&f, &across, 2, 2));
    }

    #[test]
    fn split_net_halves_reproduce_the_net() {
        // A quadratic in u times linear in v: f(s, r) = (s² - s/2)(1 + r).
        let (p, q) = (2, 1);
        // Bernstein coefficients of s² - s/2: 0, -1/4, 1/2.
        let along = [0.0, -0.25, 0.5];
        let net: Vec<f64> = (0..=p)
            .flat_map(|i| (0..=q).map(move |j| along[i] * (1.0 + j as f64)))
            .collect();
        let mut low = vec![0.0; net.len()];
        let mut high = vec![0.0; net.len()];
        split_net(&net, p, q, true, &mut low, &mut high);
        let value_at = |n: &[f64], s: f64, r: f64| {
            let bu = [(1.0 - s).powi(2), 2.0 * s * (1.0 - s), s * s];
            let bv = [1.0 - r, r];
            (0..=p)
                .map(|i| {
                    (0..=q)
                        .map(|j| bu[i] * bv[j] * n[i * (q + 1) + j])
                        .sum::<f64>()
                })
                .sum::<f64>()
        };
        for k in 0..=4 {
            let s = f64::from(k) / 4.0;
            let r = 0.3;
            let exact = (s * s - 0.5 * s) * (1.0 + r);
            assert!((value_at(&net, s, r) - exact).abs() < 1e-14);
            assert!((value_at(&low, s, r) - (0.25 * s * s - 0.25 * s) * (1.0 + r)).abs() < 1e-14);
            let t = 0.5 + 0.5 * s;
            assert!((value_at(&high, s, r) - (t * t - 0.5 * t) * (1.0 + r)).abs() < 1e-14);
        }
    }

    #[test]
    fn line_box_slab_test() {
        let bounds = Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(1.0, 1.0, 1.0),
        };
        let o = Point3::new(-1.0, 0.5, 0.5);
        let hit = Vec3::new(1.0, 0.0, 0.0);
        assert!(line_meets_box(o, hit, bounds, [0.0; 3]));
        // Behind the origin still counts: this is a line, not a ray.
        assert!(line_meets_box(o, hit * -1.0, bounds, [0.0; 3]));
        let miss = Vec3::new(1.0, 2.0, 0.0);
        assert!(!line_meets_box(o, miss, bounds, [0.0; 3]));
        let above = Point3::new(-1.0, 0.5, 1.0 + 1e-12);
        assert!(!line_meets_box(above, hit, bounds, [0.0; 3]));
        assert!(line_meets_box(above, hit, bounds, [1e-9; 3]));
    }

    #[test]
    fn tiny_direction_component_still_crosses_a_long_box() {
        // y changes by 5 over x ∈ [0, 5e17]: the line enters the y slab near
        // the box's far end although `1e-17 < eps · |dir|`.
        let bounds = Aabb3 {
            min: Point3::new(-5e17, 4.99, -1.0),
            max: Point3::new(5e17, 5.01, 1.0),
        };
        let (o, pad) = (Point3::new(0.0, 0.0, 0.0), [0.0; 3]);
        let (tilted, flatter) = (Vec3::new(1.0, 1e-17, 0.0), Vec3::new(1.0, 1e-18, 0.0));
        assert!(line_meets_box(o, tilted, bounds, pad));
        assert!(!line_meets_box(o, flatter, bounds, pad));
        // An exactly zero component is still parallel, on a slab face too.
        let (along_x, on_face) = (Vec3::new(1.0, 0.0, 0.0), Point3::new(0.0, 4.99, 0.0));
        assert!(line_meets_box(on_face, along_x, bounds, pad));
        assert!(!line_meets_box(o, along_x, bounds, pad));
    }
}
