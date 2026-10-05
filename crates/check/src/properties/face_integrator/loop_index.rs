//! Bucketed segment lookup for trimmed-domain quadrature (O06).
//!
//! A trimmed face integrates one Gauss abscissa at a time, and every
//! abscissa asks each trim loop two questions: where does the vertical line
//! at `u` cross it ([`super::UvLoop::for_each_v_crossing`]), and does it
//! enclose a span midpoint (winding number). Both scanned every segment of
//! every loop, and the abscissa count itself grows with the loop's vertex
//! count (each vertex is a quadrature break), so a densely sampled boundary
//! cost `O(n²)` per face.
//!
//! Only segments whose coordinate interval holds the query can contribute to
//! either answer, so an index that returns a superset of those segments —
//! with the exact per-segment test still applied to each — gives the same
//! crossings and the same winding number as the full scan. Crossings feed a
//! sorted cut list or a count, and winding contributions an integer sum, so
//! visiting fewer segments cannot reorder anything observable.

use remus_math::predicates::orient2d;
use remus_math::vec::Point2;

/// Upper bound on the bucket count, so a pathological loop cannot allocate
/// an index larger than the loop itself by more than this factor.
const MAX_BUCKETS: usize = 4096;

/// Segments filed under every bucket their closed coordinate interval
/// overlaps.
///
/// The bucket map is monotone in the queried coordinate, so a segment whose
/// interval `[lo, hi]` holds `x` is always among `x`'s candidates: `lo <= x <=
/// hi` implies `bucket(lo) <= bucket(x) <= bucket(hi)`, and the segment was
/// filed under that whole bucket range. Candidates come back in ascending
/// segment order.
#[derive(Debug, Clone)]
pub(super) struct IntervalIndex {
    origin: f64,
    inv_width: f64,
    buckets: Vec<Vec<usize>>,
}

impl IntervalIndex {
    /// Index `intervals` (segment `i` is the `i`-th item). Returns `None`
    /// when an interval is not finite or the intervals have no extent; the
    /// caller then scans every segment, exactly as before.
    pub(super) fn build(intervals: &[(f64, f64)]) -> Option<Self> {
        if intervals.is_empty() {
            return None;
        }
        let mut origin = f64::INFINITY;
        let mut end = f64::NEG_INFINITY;
        for &(lo, hi) in intervals {
            if !lo.is_finite() || !hi.is_finite() {
                return None;
            }
            origin = origin.min(lo);
            end = end.max(hi);
        }
        let count = intervals.len().clamp(1, MAX_BUCKETS);
        #[allow(clippy::cast_precision_loss)]
        let width = (end - origin) / count as f64;
        let inv_width = width.recip();
        if !width.is_finite() || width <= 0.0 || !inv_width.is_finite() {
            return None;
        }
        let mut index = Self {
            origin,
            inv_width,
            buckets: vec![Vec::new(); count],
        };
        for (segment, &(lo, hi)) in intervals.iter().enumerate() {
            let (first, last) = (index.bucket(lo), index.bucket(hi));
            for bucket in &mut index.buckets[first..=last] {
                bucket.push(segment);
            }
        }
        Some(index)
    }

    /// The bucket holding `x`: monotone non-decreasing in `x`, clamped to
    /// the index. A NaN lands in bucket 0, where the caller's exact test
    /// rejects every candidate as the full scan would.
    fn bucket(&self, x: f64) -> usize {
        let last = self.buckets.len() - 1;
        let position = ((x - self.origin) * self.inv_width).floor();
        if position >= 0.0 {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let position = position as usize;
            position.min(last)
        } else {
            0
        }
    }

    /// Segments whose interval may hold `x`, in ascending order.
    pub(super) fn candidates(&self, x: f64) -> &[usize] {
        &self.buckets[self.bucket(x)]
    }
}

/// [`remus_math::predicates::winding_number`] over the closed polygon,
/// visiting only `edges` (indices `i` of edges `i -> (i + 1) % n`).
///
/// An edge contributes only when `min(y_i, y_j) <= point.y < max(y_i, y_j)`,
/// so passing every edge whose `y` interval holds `point.y` (an
/// [`IntervalIndex`] over edge `y` extents) sums exactly the contributions
/// the full scan sums.
pub(super) fn winding_number_over(point: Point2, polygon: &[Point2], edges: &[usize]) -> i32 {
    let n = polygon.len();
    if n < 3 {
        return 0;
    }

    let mut wn = 0i32;
    for &i in edges {
        let j = (i + 1) % n;
        let vi = polygon[i];
        let vj = polygon[j];

        if vi.y() <= point.y() {
            if vj.y() > point.y() {
                // Upward crossing
                if orient2d(vi, vj, point) > 0.0 {
                    wn += 1;
                }
            }
        } else if vj.y() <= point.y() {
            // Downward crossing
            if orient2d(vi, vj, point) < 0.0 {
                wn -= 1;
            }
        }
    }
    wn
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

    use super::*;
    use remus_math::predicates::winding_number;

    /// Deterministic low-discrepancy sample in [0, 1).
    fn halton(mut i: u32, base: u32) -> f64 {
        let (mut f, mut r) = (1.0_f64, 0.0_f64);
        while i > 0 {
            f /= f64::from(base);
            r += f * f64::from(i % base);
            i /= base;
        }
        r
    }

    /// A star-shaped, densely sampled, non-convex loop with vertical and
    /// horizontal runs and repeated coordinates.
    fn star(n: usize) -> Vec<Point2> {
        let mut points = Vec::with_capacity(n + 4);
        for k in 0..n {
            let t = std::f64::consts::TAU * k as f64 / n as f64;
            let r = if k % 2 == 0 { 1.0 } else { 0.45 };
            points.push(Point2::new(r * t.cos(), r * t.sin()));
        }
        // A vertical and a horizontal run.
        points.push(Point2::new(1.2, 0.0));
        points.push(Point2::new(1.2, 0.3));
        points.push(Point2::new(0.9, 0.3));
        points
    }

    fn edge_y_intervals(polygon: &[Point2]) -> Vec<(f64, f64)> {
        let n = polygon.len();
        (0..n)
            .map(|i| {
                let (a, b) = (polygon[i].y(), polygon[(i + 1) % n].y());
                (a.min(b), a.max(b))
            })
            .collect()
    }

    #[test]
    fn indexed_winding_matches_full_scan() {
        for n in [3, 4, 17, 64, 301] {
            let polygon = star(n);
            let index = IntervalIndex::build(&edge_y_intervals(&polygon)).unwrap();
            let mut probes: Vec<Point2> = (1..=2_000)
                .map(|i| Point2::new(2.6 * halton(i, 2) - 1.3, 2.6 * halton(i, 3) - 1.3))
                .collect();
            // Probes exactly on vertex heights and vertices, plus non-finite.
            probes.extend(polygon.iter().copied());
            probes.extend(polygon.iter().map(|p| Point2::new(0.0, p.y())));
            probes.push(Point2::new(0.0, f64::NAN));
            probes.push(Point2::new(0.0, f64::INFINITY));
            probes.push(Point2::new(0.0, f64::NEG_INFINITY));
            for probe in probes {
                assert_eq!(
                    winding_number_over(probe, &polygon, index.candidates(probe.y())),
                    winding_number(probe, &polygon),
                    "n {n} probe {probe:?}"
                );
            }
        }
    }

    #[test]
    fn candidates_cover_every_interval_holding_the_query() {
        let intervals: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let a = 10.0 * halton(i + 1, 2) - 5.0;
                let b = a + 3.0 * halton(i + 1, 3) * halton(i + 1, 5);
                (a, b)
            })
            .chain([(0.0, 0.0), (-5.0, 5.0)])
            .collect();
        let index = IntervalIndex::build(&intervals).unwrap();
        for i in 1..=5_000 {
            let x = 12.0 * halton(i, 7) - 6.0;
            let candidates = index.candidates(x);
            for (segment, &(lo, hi)) in intervals.iter().enumerate() {
                if lo <= x && x <= hi {
                    assert!(candidates.contains(&segment), "x {x} segment {segment}");
                }
            }
            assert!(candidates.windows(2).all(|w| w[0] < w[1]));
        }
        // Interval endpoints themselves.
        for (segment, &(lo, hi)) in intervals.iter().enumerate() {
            assert!(index.candidates(lo).contains(&segment));
            assert!(index.candidates(hi).contains(&segment));
        }
    }

    #[test]
    fn degenerate_or_non_finite_extents_decline_to_index() {
        assert!(IntervalIndex::build(&[]).is_none());
        assert!(IntervalIndex::build(&[(1.0, 1.0), (1.0, 1.0)]).is_none());
        assert!(IntervalIndex::build(&[(0.0, f64::INFINITY)]).is_none());
        assert!(IntervalIndex::build(&[(f64::NAN, 1.0)]).is_none());
    }
}
