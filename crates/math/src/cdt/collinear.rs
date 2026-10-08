//! The collinear-vertex query of [`Cdt::insert_constraint`].
//!
//! A constraint is split through every existing vertex that lies on it. The
//! plain answer tests every vertex, which costs O(V) per constraint and
//! O(V·C) per CDT: a 64-hole plate cap (1156 points, 1156 constraints) spent
//! 15% of its tessellation time there. Once that scanning has cost about as
//! much as sorting, the vertices are indexed by coordinate and each
//! constraint tests only the vertices in a window around its bounding box.
//!
//! # Why the window returns the same vertices
//!
//! Both paths run the same test, [`collinear_hit`], on each vertex they
//! consider, and sort the hits by `(t, vi)`. So the answers match if every
//! vertex the plain scan accepts lies inside the window. The window is used
//! only when every indexed vertex and both endpoints are finite and at most
//! `INDEX_COORD_LIMIT` in magnitude, and the squared segment length `L²` is
//! at least `INDEX_MIN_LEN_SQ`. Then nothing overflows, underflow error is
//! negligible next to the tolerances below, and Rust never contracts the
//! test's products into fused multiply-adds.
//!
//! Write `P = v − p0` and `D = p1 − p0`, both rounded, and `ε = 2⁻⁵³`. Each
//! computed dot or cross product of them is off by at most about `2ε|P||D|`.
//! An accepted vertex has computed `t` in `(1e-6, 1 − 1e-6)` and computed
//! `cross²/L² < DUP_TOL²`. So its exact distance from the line through `D`
//! is below `DUP_TOL(1 + 2ε) + 2ε|P|`, its exact parameter is within
//! `3ε + 2ε|P|/|D|` of `[0, 1]`, and so `|P| ≤ (|D| + DUP_TOL)(1 + 8ε)`.
//! With the rounding of `P` and `D` themselves, the vertex lies within
//! `DUP_TOL + 12ε(|D| + DUP_TOL)` of the segment's bounding box on each
//! axis. The margin `2·DUP_TOL + 1e-12·(|dx| + |dy|)` covers the `DUP_TOL`
//! term twice and the `|D|` term about 750 times over. The factor of 2 on
//! `DUP_TOL` is a deliberate safety margin: shrinking it changes no answer.
//!
//! The window ends are rounded too. Rounding to nearest is monotone, so a
//! float below `fl(lo)` is below `lo`, and a float above `fl(hi)` is above
//! `hi`: the rounded window never cuts a vertex inside the exact one.
//!
//! Non-finite vertices stay out of the index. The plain test always rejects
//! them (`cross²/L²` comes out NaN or infinite), and a NaN key would break
//! the binary search. A finite coordinate above the limit makes the CDT use
//! the plain scan from then on. Vertices are only ever appended, never
//! moved, so vertices added after the index was built are tested directly
//! until the next rebuild.

use crate::vec::Point2;

use super::{Cdt, DUP_TOL};

/// Coordinate magnitude above which the window's rounding analysis no longer
/// applies (products could overflow).
const INDEX_COORD_LIMIT: f64 = 1e50;

/// Squared constraint length below which the window's rounding analysis no
/// longer applies (products could underflow). Distinct CDT vertices are
/// `DUP_TOL` apart, so real constraints are far longer.
const INDEX_MIN_LEN_SQ: f64 = 1e-60;

/// CDTs with fewer non-super vertices never build the index.
const INDEX_MIN_VERTICES: usize = 64;

/// Unindexed vertices that force a rebuild, at least.
const INDEX_MIN_TAIL: usize = 64;

/// The collinear-vertex index of one CDT.
pub(super) enum CollinearIndex {
    /// Not built yet. Counts the vertex tests plain scans have made, so the
    /// index is built only once scanning has cost about as much as sorting.
    Pending {
        /// Vertex tests made by plain scans so far.
        scanned: usize,
    },
    /// Built over the vertices before `AxisIndex::end`.
    Built(AxisIndex),
    /// A vertex exceeds `INDEX_COORD_LIMIT`. Vertices never move, so every
    /// later query scans.
    Unindexable,
}

/// Every finite non-super vertex below `end`, sorted along each axis.
pub(super) struct AxisIndex {
    /// `(x, y, vi)`, ordered by `x` then `vi`.
    by_x: Vec<(f64, f64, usize)>,
    /// `(y, x, vi)`, ordered by `y` then `vi`.
    by_y: Vec<(f64, f64, usize)>,
    /// First vertex not in the index.
    pub(super) end: usize,
}

impl Default for CollinearIndex {
    fn default() -> Self {
        Self::Pending { scanned: 0 }
    }
}

/// Parameter `t` of `v` along the constraint `p0 + t·(dx, dy)` if `v` lies on
/// its open interior within `DUP_TOL`.
///
/// Both query paths use this one test; its expressions must stay exactly as
/// they are for the two paths to agree.
#[inline]
pub(super) fn collinear_hit(
    v: Point2,
    p0: Point2,
    dx: f64,
    dy: f64,
    seg_len_sq: f64,
) -> Option<f64> {
    #[cfg(test)]
    super::work::bump(&super::work::COLLINEAR_TESTS);
    let px = v.x() - p0.x();
    let py = v.y() - p0.y();
    let t = (px * dx + py * dy) / seg_len_sq;
    if t <= 1e-6 || t >= 1.0 - 1e-6 {
        return None;
    }
    let cross = px * dy - py * dx;
    let dist_sq = cross * cross / seg_len_sq;
    // Constraint insertion must not bend a long boundary through a distinct
    // nearby vertex. Use the same linear resolution as vertex insertion,
    // independent of the constraint's length.
    if dist_sq < DUP_TOL * DUP_TOL {
        Some(t)
    } else {
        None
    }
}

fn within_index_limit(p: Point2) -> bool {
    p.x().abs() <= INDEX_COORD_LIMIT && p.y().abs() <= INDEX_COORD_LIMIT
}

/// `[min(a, b) − margin, max(a, b) + margin]`.
fn padded_span(a: f64, b: f64, margin: f64) -> (f64, f64) {
    (a.min(b) - margin, a.max(b) + margin)
}

/// Plain-scan work after which building the index pays: about `2·n·log2 n`
/// vertex tests.
fn build_threshold(n: usize) -> usize {
    let log2 = (usize::BITS - n.leading_zeros()) as usize;
    n.saturating_mul(log2).saturating_mul(2)
}

impl AxisIndex {
    /// Index the vertices `[start, vertices.len())`, or `None` if one of
    /// them exceeds `INDEX_COORD_LIMIT`.
    fn build(vertices: &[Point2], start: usize) -> Option<Self> {
        let mut by_x = Vec::with_capacity(vertices.len().saturating_sub(start));
        for (vi, p) in vertices.iter().enumerate().skip(start) {
            let (x, y) = (p.x(), p.y());
            if !(x.is_finite() && y.is_finite()) {
                continue;
            }
            if !within_index_limit(*p) {
                return None;
            }
            by_x.push((x, y, vi));
        }
        let mut by_y: Vec<(f64, f64, usize)> = by_x.iter().map(|&(x, y, vi)| (y, x, vi)).collect();
        let order =
            |a: &(f64, f64, usize), b: &(f64, f64, usize)| a.0.total_cmp(&b.0).then(a.2.cmp(&b.2));
        by_x.sort_unstable_by(order);
        by_y.sort_unstable_by(order);
        Some(Self {
            by_x,
            by_y,
            end: vertices.len(),
        })
    }

    /// Push the hits among the indexed vertices inside the window around the
    /// segment `(p0, p1)`.
    fn query(
        &self,
        v0: usize,
        v1: usize,
        p0: Point2,
        p1: Point2,
        seg_len_sq: f64,
        out: &mut Vec<(f64, usize)>,
    ) {
        let dx = p1.x() - p0.x();
        let dy = p1.y() - p0.y();
        let margin = 2.0 * DUP_TOL + 1e-12 * (dx.abs() + dy.abs());
        let x_span = padded_span(p0.x(), p1.x(), margin);
        let y_span = padded_span(p0.y(), p1.y(), margin);
        // Walk the axis along which the segment is narrower.
        let along_x = dx.abs() <= dy.abs();
        let (keys, (lo, hi), (other_lo, other_hi)) = if along_x {
            (&self.by_x, x_span, y_span)
        } else {
            (&self.by_y, y_span, x_span)
        };
        let first = keys.partition_point(|e| e.0 < lo);
        for &(key, other, vi) in &keys[first..] {
            if key > hi {
                break;
            }
            #[cfg(test)]
            super::work::bump(&super::work::COLLINEAR_VISITS);
            if other < other_lo || other > other_hi || vi == v0 || vi == v1 {
                continue;
            }
            let v = if along_x {
                Point2::new(key, other)
            } else {
                Point2::new(other, key)
            };
            if let Some(t) = collinear_hit(v, p0, dx, dy, seg_len_sq) {
                out.push((t, vi));
            }
        }
    }
}

impl Cdt {
    /// Every vertex other than `v0` and `v1` lying on the open segment
    /// `(v0, v1)` within `DUP_TOL`, as `(t, vi)` sorted by `t` then `vi`.
    ///
    /// `seg_len_sq` is the caller's `dx·dx + dy·dy` and must be positive.
    pub(super) fn collinear_vertices(
        &mut self,
        v0: usize,
        v1: usize,
        seg_len_sq: f64,
    ) -> Vec<(f64, usize)> {
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        let indexable = within_index_limit(p0)
            && within_index_limit(p1)
            && seg_len_sq.is_finite()
            && seg_len_sq >= INDEX_MIN_LEN_SQ;
        if indexable {
            self.refresh_collinear_index();
        }

        let mut hits = Vec::new();
        let mut scan_from = self.super_count;
        if indexable && let CollinearIndex::Built(index) = &self.collinear_index {
            index.query(v0, v1, p0, p1, seg_len_sq, &mut hits);
            scan_from = index.end;
        } else if let CollinearIndex::Pending { scanned } = &mut self.collinear_index {
            *scanned = scanned.saturating_add(self.vertices.len() - self.super_count);
        }
        let dx = p1.x() - p0.x();
        let dy = p1.y() - p0.y();
        for vi in scan_from..self.vertices.len() {
            if vi == v0 || vi == v1 {
                continue;
            }
            #[cfg(test)]
            super::work::bump(&super::work::COLLINEAR_VISITS);
            if let Some(t) = collinear_hit(self.vertices[vi], p0, dx, dy, seg_len_sq) {
                hits.push((t, vi));
            }
        }
        // The plain scan pushes in `vi` order and sorts stably by `t`.
        hits.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        hits
    }

    /// Build the index once plain scanning has cost about as much as
    /// sorting, and rebuild it once enough vertices were added after it.
    fn refresh_collinear_index(&mut self) {
        let sc = self.super_count;
        let n = self.vertices.len() - sc;
        let stale = match &self.collinear_index {
            CollinearIndex::Pending { scanned } => {
                n >= INDEX_MIN_VERTICES && *scanned >= build_threshold(n)
            }
            CollinearIndex::Built(index) => {
                let tail = self.vertices.len() - index.end;
                tail > INDEX_MIN_TAIL.max((index.end - sc) / 8)
            }
            CollinearIndex::Unindexable => false,
        };
        #[cfg(test)]
        let stale = match super::work::INDEX_POLICY.with(std::cell::Cell::get) {
            Some(true) => match &self.collinear_index {
                CollinearIndex::Pending { .. } => true,
                _ => stale,
            },
            Some(false) => false,
            None => stale,
        };
        if stale {
            self.collinear_index = match AxisIndex::build(&self.vertices, sc) {
                Some(index) => CollinearIndex::Built(index),
                None => CollinearIndex::Unindexable,
            };
        }
    }
}
