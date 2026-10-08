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
///
/// The buckets are laid out back to back in one array (bucket `b` is
/// `segments[offsets[b]..offsets[b + 1]]`) rather than one allocation each:
/// a densely sampled loop has hundreds of buckets, and the integrator
/// indexes every trim loop of every curved face it measures.
#[derive(Debug, Clone)]
pub(super) struct IntervalIndex {
    origin: f64,
    inv_width: f64,
    /// The last bucket's index (the bucket count less one).
    last: usize,
    /// Where each bucket's run of `segments` starts, then where the last
    /// run ends.
    offsets: Vec<usize>,
    segments: Vec<usize>,
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
        let last = count - 1;
        // Each segment's bucket range, kept for the fill pass so `floor`
        // runs once per endpoint, while every bucket's members are counted
        // one slot ahead: the running sum then turns the counts into each
        // bucket's start.
        let mut offsets = vec![0; count + 1];
        let mut ranges = Vec::with_capacity(intervals.len());
        for &(lo, hi) in intervals {
            let (first, end) = (
                bucket_of(origin, inv_width, last, lo),
                bucket_of(origin, inv_width, last, hi),
            );
            for bucket in first..=end {
                offsets[bucket + 1] += 1;
            }
            ranges.push((first, end));
        }
        for bucket in 0..count {
            offsets[bucket + 1] += offsets[bucket];
        }
        // Segments are filed in ascending order, so every bucket's run is
        // ascending. A bucket's start advances as it fills and ends on the
        // next bucket's start, so shifting them back one slot restores them.
        let mut segments = vec![0; offsets[count]];
        for (segment, &(first, end)) in ranges.iter().enumerate() {
            for bucket in first..=end {
                segments[offsets[bucket]] = segment;
                offsets[bucket] += 1;
            }
        }
        offsets.copy_within(0..count, 1);
        offsets[0] = 0;
        Some(Self {
            origin,
            inv_width,
            last,
            offsets,
            segments,
        })
    }

    /// Segments whose interval may hold `x`, in ascending order.
    pub(super) fn candidates(&self, x: f64) -> &[usize] {
        let bucket = bucket_of(self.origin, self.inv_width, self.last, x);
        &self.segments[self.offsets[bucket]..self.offsets[bucket + 1]]
    }
}

/// The bucket holding `x` in an index whose buckets start at `origin` and
/// are `1 / inv_width` wide: monotone non-decreasing in `x`, clamped to
/// `0..=last`. A NaN lands in bucket 0, where the caller's exact test
/// rejects every candidate as the full scan would.
fn bucket_of(origin: f64, inv_width: f64, last: usize, x: f64) -> usize {
    let position = ((x - origin) * inv_width).floor();
    if position >= 0.0 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let position = position as usize;
        position.min(last)
    } else {
        0
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

    /// The index as it stood before its buckets were stored flat: one
    /// `Vec` per bucket. Kept verbatim as the oracle for the flat layout.
    struct NestedIndex {
        origin: f64,
        inv_width: f64,
        buckets: Vec<Vec<usize>>,
    }

    impl NestedIndex {
        fn build(intervals: &[(f64, f64)]) -> Option<Self> {
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

        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        fn bucket(&self, x: f64) -> usize {
            let last = self.buckets.len() - 1;
            let position = ((x - self.origin) * self.inv_width).floor();
            if position >= 0.0 {
                let position = position as usize;
                position.min(last)
            } else {
                0
            }
        }

        fn candidates(&self, x: f64) -> &[usize] {
            &self.buckets[self.bucket(x)]
        }

        /// Every query the bucket map can tell apart: each bucket boundary
        /// `origin + k * width` and its neighbouring floats, either side of
        /// the extent, signed zeros and the non-finite values.
        fn boundary_probes(&self) -> Vec<f64> {
            let width = self.inv_width.recip();
            let mut probes = vec![
                0.0,
                -0.0,
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::MAX,
                f64::MIN,
            ];
            for k in 0..=self.buckets.len() + 1 {
                let x = width.mul_add(k as f64, self.origin);
                probes.extend([x, x.next_up(), x.next_down()]);
                let y = self.origin + k as f64 * width;
                probes.extend([y, y.next_up(), y.next_down()]);
            }
            probes
        }
    }

    /// Assert the flat index answers exactly as the nested one — the same
    /// segments in the same order — at the interval endpoints, every bucket
    /// boundary and its neighbours, and a spread of interior queries.
    fn assert_matches_nested(intervals: &[(f64, f64)], what: &str) {
        let flat = IntervalIndex::build(intervals);
        let nested = NestedIndex::build(intervals);
        assert_eq!(
            flat.is_some(),
            nested.is_some(),
            "{what}: one layout indexed what the other declined"
        );
        let (Some(flat), Some(nested)) = (flat, nested) else {
            return;
        };
        let (lo, hi) = intervals
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &(a, b)| {
                (lo.min(a), hi.max(b))
            });
        let mut probes = nested.boundary_probes();
        for &(a, b) in intervals {
            probes.extend([a, b, a.next_down(), b.next_up()]);
        }
        let span = hi - lo;
        probes.extend((1..=10_000).map(|i| (1.2 * span).mul_add(halton(i, 2), lo - 0.1 * span)));
        for x in probes {
            assert_eq!(
                flat.candidates(x),
                nested.candidates(x),
                "{what}: query {x:e} ({:#018x})",
                x.to_bits()
            );
        }
    }

    #[test]
    fn flat_index_matches_bucket_vectors() {
        // Densely sampled loops, as the integrator indexes them.
        for n in [3, 4, 17, 64, 301] {
            let polygon = star(n);
            assert_matches_nested(&edge_y_intervals(&polygon), &format!("star({n}) edges"));
        }
        // A single interval is a single bucket; two give two.
        assert_matches_nested(&[(-1.5, 2.5)], "one interval");
        assert_matches_nested(&[(0.0, 1.0), (0.5, 3.0)], "two intervals");
        // More intervals than MAX_BUCKETS, with zero-width intervals, one
        // spanning the whole extent and repeated endpoints.
        for n in [1, 2, 3, 17, 301, 5_000] {
            let intervals: Vec<(f64, f64)> = (0..n)
                .map(|i| {
                    let a = 10.0 * halton(i + 1, 2) - 5.0;
                    let b = if i % 7 == 0 {
                        a
                    } else {
                        a + 3.0 * halton(i + 1, 3) * halton(i + 1, 5)
                    };
                    (a, b)
                })
                .chain([(0.0, 0.0), (-5.0, 5.0), (-5.0, -5.0), (5.0, 5.0)])
                .collect();
            assert_matches_nested(&intervals, &format!("{n} random intervals"));
        }
        // A period-wrapping band's segments, closed by the step one turn on
        // (`UvLoop::segment_u_index(true)`): the last one ends past `2pi`.
        let tau = std::f64::consts::TAU;
        let band: Vec<(f64, f64)> = (0..129)
            .map(|k| {
                let a = tau * k as f64 / 129.0 + 0.3;
                let b = if k + 1 < 129 {
                    tau * (k + 1) as f64 / 129.0 + 0.3
                } else {
                    0.3 + tau
                };
                (a.min(b), a.max(b))
            })
            .collect();
        assert_matches_nested(&band, "wrapping band");
        // Declined inputs stay declined.
        for intervals in [
            vec![],
            vec![(1.0, 1.0), (1.0, 1.0)],
            vec![(0.0, f64::INFINITY)],
            vec![(f64::NAN, 1.0)],
            vec![(-f64::MAX, f64::MAX)],
        ] {
            assert_matches_nested(&intervals, "declined");
        }
        // The intervals the coverage test above indexes.
        let intervals: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let a = 10.0 * halton(i + 1, 2) - 5.0;
                (a, a + 3.0 * halton(i + 1, 3) * halton(i + 1, 5))
            })
            .chain([(0.0, 0.0), (-5.0, 5.0)])
            .collect();
        assert_matches_nested(&intervals, "coverage intervals");
    }
}
