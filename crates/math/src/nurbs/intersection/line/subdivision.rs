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
//!    until both nets are nearly affine, where at most one transversal root
//!    remains and Newton converges from the box centre;
//! 4. a Bézier box whose affine model, widened by its proven deviation from
//!    the nets, puts every root outside the box.
//!
//! Each surviving box runs the shared Newton refinement clamped to the box
//! (with a margin), so it cannot slide off to a different root, and accepts
//! only a residual below `1e-9` of the surface's extent plus a round-off
//! allowance for its distance from the origin: a root this pass reports is a
//! genuine crossing, never the grid pass's looser `1e-5` near-miss, and the
//! thresholds do not grow when the geometry is translated. A box
//! already holding a reported root is not refined again: a nearly affine
//! box holds at most one transversal crossing, and two crossings of a
//! near-tangent line inside one tiny box would merge under the `1e-6`
//! deduplication anyway.
//!
//! Work is bounded per line by [`MAX_BEZIER_NODES`] and
//! [`MAX_LEAF_REFINEMENTS`]. A line lying in or grazing along the surface can
//! exhaust them; the pass then stops and the result is the grid result plus
//! whatever was found so far, never worse than the grid alone.
//!
//! Arithmetic is plain products and sums, not `mul_add`: off `wasm32` and
//! without an FMA target feature that is a library call, and these loops are
//! all multiply-adds.

use std::ops::Range;

use crate::aabb::Aabb3;
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
/// Acceptance residual of a completion root, relative to the surface's extent
/// (its control box diagonal). Translation-invariant: position enters only
/// through [`ROUNDOFF`].
const ROOT_TOLERANCE: f64 = 1e-9;
/// Slab-test padding, relative to the surface's extent, so rounding in the
/// box test never rejects a box the line touches.
const BOX_PADDING: f64 = 1e-9;
/// Round-off allowance per unit of coordinate magnitude, added to both: a
/// surface far from the origin computes differences no finer than this.
const ROUNDOFF: f64 = 64.0 * f64::EPSILON;

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
    /// Diagonal of the control-point box.
    extent: f64,
    /// Largest absolute control-point coordinate.
    magnitude: f64,
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
            extent: 0.0,
            magnitude: 0.0,
        };
        let root = hull.root();
        if root.leaves() > 0 {
            hull.boxes.reserve(2 * root.leaves() - 1);
            let bounds = hull.build_box(surface, root);
            hull.extent = (bounds.max - bounds.min).length();
            hull.magnitude = [bounds.min, bounds.max]
                .iter()
                .flat_map(|corner| corner.0)
                .fold(0.0_f64, |acc, c| acc.max(c.abs()));
        }
        hull
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

/// Whether the infinite line meets `bounds` padded by `pad` (slab method).
fn line_meets_box(origin: Point3, dir: Vec3, dir_len: f64, bounds: Aabb3, pad: f64) -> bool {
    let (mut t_lo, mut t_hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for axis in 0..3 {
        let o = origin.0[axis];
        let d = dir.0[axis];
        let lo = bounds.min.0[axis] - pad;
        let hi = bounds.max.0[axis] + pad;
        // Parallel within rounding: the line's coordinate moves by at most
        // `eps · extent` across the box, far below `pad`.
        if d.abs() <= f64::EPSILON * dir_len {
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
    pad: f64,
    /// Rounding allowance of the homogeneous control values.
    slack: f64,
    /// Weights are scaled by this (`1 / max weight`): a common weight factor
    /// leaves the surface unchanged and must not overflow the products.
    inv_max_weight: f64,
    tolerance: f64,
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
    if hull.boxes.is_empty() {
        return;
    }
    let caller_origin = line.origin;
    // The same line through the point nearest the surface's box centre: the
    // control values and Newton residuals are differences from the origin,
    // so a far origin would otherwise inflate the round-off allowance below
    // although sliding the origin along the line changes nothing.
    let centre = hull.boxes[0].center();
    let along = (centre - line.origin).dot(line.direction) / line.dir_len_sq;
    let line = LineQuery {
        origin: line.origin + line.direction * along,
        ..line
    };
    let dir_len = line.dir_len_sq.sqrt();
    let magnitude = line
        .origin
        .0
        .iter()
        .fold(hull.magnitude, |acc, c| acc.max(c.abs()));
    let roundoff = ROUNDOFF * magnitude;
    let pad = BOX_PADDING * hull.extent + roundoff;
    if !line_meets_box(line.origin, line.direction, dir_len, hull.boxes[0], pad) {
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

    let mut pass = Completion {
        surface,
        hull,
        line,
        n1,
        n2,
        pad,
        // Weights enter divided by the largest, so the homogeneous values are
        // bounded by the plane distances.
        slack: pad,
        inv_max_weight: 1.0 / surface.max_weight(),
        tolerance: ROOT_TOLERANCE * hull.extent + roundoff,
        nodes: 0,
        refinements: 0,
        scratch: Scratch::default(),
    };

    let reported = results.len();
    pass.traverse(dir_len, workspace, results);
    // Line parameters of the appended roots relative to the caller's origin,
    // as the grid pass reports them.
    for hit in &mut results[reported..] {
        hit.param2.0 = (hit.point - caller_origin).dot(line.direction) / line.dir_len_sq;
    }
}

impl Completion<'_> {
    /// Walk the span-block tree along the line, solving every block it meets.
    fn traverse(
        &mut self,
        dir_len: f64,
        workspace: &mut NewtonWorkspace,
        results: &mut Vec<IntersectionPoint>,
    ) {
        let (hull, line) = (self.hull, self.line);
        let mut stack = vec![(0_usize, hull.root())];
        while let Some((slot, range)) = stack.pop() {
            if slot > 0
                && !line_meets_box(
                    line.origin,
                    line.direction,
                    dir_len,
                    hull.boxes[slot],
                    self.pad,
                )
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
        let slack = self.slack;
        let (us, vs) = hull.block_spans(bu, bv);
        let (rows, cols) = hull.control_window(&us, &vs);
        let cps = self.surface.control_points();
        let weights = self.surface.weights();
        let width = cols.len();
        let scratch = &mut self.scratch;
        scratch.window_f.clear();
        scratch.window_g.clear();
        for i in rows.clone() {
            for j in cols.clone() {
                let rel = cps[i][j] - self.line.origin;
                let w = weights[i][j] * self.inv_max_weight;
                scratch.window_f.push(w * self.n1.dot(rel));
                scratch.window_g.push(w * self.n2.dot(rel));
            }
        }
        if one_signed(&scratch.window_f, slack) || one_signed(&scratch.window_g, slack) {
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
                if one_signed(&scratch.f, slack) || one_signed(&scratch.g, slack) {
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
            if one_signed(fnet, self.slack) || one_signed(gnet, self.slack) {
                continue;
            }
            let (f_fit, g_fit) = (affine_fit(fnet, p, q), affine_fit(gnet, p, q));
            if affine_excludes_root(&f_fit, &g_fit) {
                continue;
            }
            let leaf =
                node.depth >= MAX_SPLIT_DEPTH || (f_fit.nearly_affine() && g_fit.nearly_affine());
            if leaf {
                let (surface, line, tolerance) = (self.surface, self.line, self.tolerance);
                if self.refinements >= MAX_LEAF_REFINEMENTS {
                    return false;
                }
                if refine_box(surface, line, tolerance, node, workspace, results) {
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

/// Newton from the box centre, clamped to the box plus a margin; appends a
/// new root. Returns whether a refinement ran (`false` when a reported root
/// already lies in the box: a nearly affine box holds at most one
/// transversal crossing).
fn refine_box(
    surface: &NurbsSurface,
    line: LineQuery,
    tolerance: f64,
    node: BezierBox,
    workspace: &mut NewtonWorkspace,
    results: &mut Vec<IntersectionPoint>,
) -> bool {
    let known = results.iter().any(|hit| {
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
    if let Some(hit) =
        refine_line_surface_point(surface, line, (u, v), seed, bounds, tolerance, workspace)
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
        assert!(line_meets_box(o, hit, 1.0, bounds, 0.0));
        // Behind the origin still counts: this is a line, not a ray.
        assert!(line_meets_box(o, hit * -1.0, 1.0, bounds, 0.0));
        let miss = Vec3::new(1.0, 2.0, 0.0);
        assert!(!line_meets_box(o, miss, miss.length(), bounds, 0.0));
        let above = Point3::new(-1.0, 0.5, 1.0 + 1e-12);
        assert!(!line_meets_box(above, hit, 1.0, bounds, 0.0));
        assert!(line_meets_box(above, hit, 1.0, bounds, 1e-9));
    }
}
