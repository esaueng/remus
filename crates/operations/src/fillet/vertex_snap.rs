//! Spatial index behind the rolling-ball engine's Phase 5d vertex dedup and
//! snap.
//!
//! Phase 5d collects the input body's vertex positions without near
//! duplicates, then snaps every emitted face-spec vertex to the closest of
//! them within a snap tolerance. Both used to be linear scans over every
//! collected position, quadratic in the body's vertex count. [`SnapGrid`]
//! answers the same two queries from a uniform grid and returns exactly what
//! the scans returned: the same "any position within `r`" verdict and the
//! same nearest position, ties going to the earliest collected one.

use remus_math::det_hash::DetLookupHashMap;
use remus_math::vec::Point3;

/// Cell width in query reaches. Vertex spacing on a body is far above the
/// reach, so cells this wide stay sparse, and a query's block is one cell
/// unless it straddles a face of the grid.
const CELL_REACHES: f64 = 64.0;

/// Largest cell index magnitude a key is formed for, so the cast to `i64`
/// is exact. A position beyond it, or with a non-finite coordinate, is
/// checked on every query instead.
const MAX_KEY: f64 = 4_503_599_627_370_496.0;

/// Smallest radius the grid answers. A coordinate offset of at least
/// `2^-500` squares to a normal float, so `length` rounds every term by a
/// few ulps at most, and a computed distance below the radius bounds every
/// coordinate offset by the radius plus [`MARGIN`] of it. Smaller offsets
/// lie inside any margin of at least this radius.
const MIN_GRID_RADIUS: f64 = 1.0e-140;

/// Relative slack of the per-axis search margin over the query radius:
/// many orders above the few ulps by which a computed distance can fall
/// short of a coordinate offset.
const MARGIN: f64 = 1.0 / 1_048_576.0;

/// Positions in insertion order, bucketed by a uniform grid.
///
/// A position's key is `floor(coordinate / cell)` per axis. A query for
/// radius `r` visits every cell from the key of `c - margin` to the key of
/// `c + margin` on each axis, where `margin` covers every offset a stored
/// position with a computed distance below `r` can have. Rounding is
/// monotone, so the key of any such position lies in that range. Positions
/// without a key live in `unbinned` and are checked on every query; a query
/// whose block cannot be formed falls back to the full scan.
pub(super) struct SnapGrid {
    points: Vec<Point3>,
    reach: f64,
    cell: f64,
    cells: DetLookupHashMap<[i64; 3], Vec<usize>>,
    unbinned: Vec<usize>,
}

impl SnapGrid {
    /// An empty grid sized for query radii up to `reach`; a larger radius
    /// is answered by the full scan.
    pub(super) fn new(reach: f64) -> Self {
        Self {
            points: Vec::new(),
            reach,
            cell: CELL_REACHES * reach,
            cells: DetLookupHashMap::default(),
            unbinned: Vec::new(),
        }
    }

    fn cell_index(&self, coordinate: f64) -> Option<i64> {
        let scaled = (coordinate / self.cell).floor();
        // A NaN fails the comparison as well.
        if scaled.abs() < MAX_KEY {
            #[allow(clippy::cast_possible_truncation)]
            Some(scaled as i64)
        } else {
            None
        }
    }

    fn key(&self, p: Point3) -> Option<[i64; 3]> {
        Some([
            self.cell_index(p.x())?,
            self.cell_index(p.y())?,
            self.cell_index(p.z())?,
        ])
    }

    /// Per-axis cell ranges holding every stored position closer than
    /// `radius` to `p`, or `None` for the full scan.
    fn block(&self, p: Point3, radius: f64) -> Option<[(i64, i64); 3]> {
        // Also refuses a NaN radius and, through the reach, a cell that is
        // not positive.
        if !(MIN_GRID_RADIUS..=self.reach).contains(&radius) {
            return None;
        }
        let margin = radius + radius * MARGIN;
        let mut block = [(0, 0); 3];
        for (range, coordinate) in block.iter_mut().zip([p.x(), p.y(), p.z()]) {
            *range = (
                self.cell_index(coordinate - margin)?,
                self.cell_index(coordinate + margin)?,
            );
        }
        Some(block)
    }

    /// Every stored index in `block`'s cells, then every unbinned one; each
    /// stored index at most once.
    fn candidates(&self, block: [(i64, i64); 3]) -> impl Iterator<Item = usize> + '_ {
        let [(x0, x1), (y0, y1), (z0, z1)] = block;
        (x0..=x1)
            .flat_map(move |x| (y0..=y1).flat_map(move |y| (z0..=z1).map(move |z| [x, y, z])))
            .filter_map(|key| self.cells.get(&key))
            .flatten()
            .chain(&self.unbinned)
            .copied()
    }

    /// Whether any stored position is closer than `radius` to `p`:
    /// `points.iter().any(|q| (*q - p).length() < radius)`.
    pub(super) fn any_within(&self, p: Point3, radius: f64) -> bool {
        let within = |i: usize| (self.points[i] - p).length() < radius;
        match self.block(p, radius) {
            Some(block) => self.candidates(block).any(within),
            None => (0..self.points.len()).any(within),
        }
    }

    /// Store `p` unless a stored position is closer than `radius`; the first
    /// of a cluster of near duplicates wins.
    pub(super) fn insert_unique(&mut self, p: Point3, radius: f64) {
        if self.any_within(p, radius) {
            return;
        }
        let index = self.points.len();
        self.points.push(p);
        match self.key(p) {
            Some(key) => self.cells.entry(key).or_default().push(index),
            None => self.unbinned.push(index),
        }
    }

    /// The stored position closest to `p` among those closer than `radius`,
    /// the earliest stored on a tie: what the `filter` / `min_by` scan over
    /// every position in insertion order returns.
    pub(super) fn nearest_within(&self, p: Point3, radius: f64) -> Option<Point3> {
        let within = |i: usize| {
            let distance = (self.points[i] - p).length();
            (distance < radius).then_some((distance, i))
        };
        // Distances that pass the filter are never NaN or -0.0, so
        // `total_cmp` orders them as the scan's `partial_cmp` does, and the
        // index decides a tie the way the scan's first minimum does.
        let closer = |best: Option<(f64, usize)>, candidate: (f64, usize)| {
            let keep = best.is_some_and(|(distance, index)| {
                distance
                    .total_cmp(&candidate.0)
                    .then(index.cmp(&candidate.1))
                    .is_lt()
            });
            if keep { best } else { Some(candidate) }
        };
        let best = match self.block(p, radius) {
            Some(block) => self.candidates(block).filter_map(within).fold(None, closer),
            None => (0..self.points.len()).filter_map(within).fold(None, closer),
        };
        best.map(|(_, i)| self.points[i])
    }

    /// Stored positions in insertion order.
    #[cfg(test)]
    pub(super) fn points(&self) -> &[Point3] {
        &self.points
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    const TOL: f64 = 1.0e-7;
    const SNAP: f64 = 1.0e-5;

    /// The Phase 5d dedup scan, verbatim.
    fn reference_dedup(input: &[Point3]) -> Vec<Point3> {
        let mut original_verts: Vec<Point3> = Vec::new();
        for &p in input {
            let already = original_verts
                .iter()
                .any(|existing| (*existing - p).length() < TOL);
            if !already {
                original_verts.push(p);
            }
        }
        original_verts
    }

    /// The Phase 5d snap scan, verbatim.
    fn reference_snap(original_verts: &[Point3], v: Point3) -> Option<Point3> {
        original_verts
            .iter()
            .filter(|ov| (**ov - v).length() < SNAP)
            .min_by(|a, b| {
                (**a - v)
                    .length()
                    .partial_cmp(&(**b - v).length())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
    }

    /// [`reference_snap`] at any radius.
    fn scan_nearest(original_verts: &[Point3], v: Point3, radius: f64) -> Option<Point3> {
        original_verts
            .iter()
            .filter(|ov| (**ov - v).length() < radius)
            .min_by(|a, b| {
                (**a - v)
                    .length()
                    .partial_cmp(&(**b - v).length())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
    }

    fn bits(p: Point3) -> [u64; 3] {
        [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]
    }

    /// Deterministic xorshift stream in `[0, 1)`.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            #[allow(clippy::cast_precision_loss)]
            {
                (self.0 >> 11) as f64 / (1_u64 << 53) as f64
            }
        }

        fn below(&mut self, n: usize) -> usize {
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            {
                (self.next() * n as f64) as usize
            }
        }
    }

    /// A cloud mixing well-separated vertices, near duplicates at and around
    /// the dedup and snap radii, exact duplicates, exact equidistant pairs,
    /// cell-boundary coordinates and points without a trustworthy cell key.
    fn cloud(rng: &mut Rng, scale: f64, count: usize) -> Vec<Point3> {
        let mut points: Vec<Point3> = Vec::new();
        let cell = CELL_REACHES * SNAP;
        for _ in 0..count {
            let pick = rng.below(10);
            let base = points
                .get(rng.below(points.len().max(1)))
                .copied()
                .unwrap_or_else(|| Point3::new(0.0, 0.0, 0.0));
            let p = match pick {
                0 | 1 => Point3::new(
                    (rng.next() - 0.5) * scale,
                    (rng.next() - 0.5) * scale,
                    (rng.next() - 0.5) * scale,
                ),
                2 => base,
                3 => {
                    // Within the dedup radius, on either side of it.
                    let r = TOL * (0.5 + rng.next());
                    Point3::new(base.x() + r, base.y(), base.z() - r * 0.25)
                }
                4 => {
                    // Within the snap radius, on either side of it.
                    let r = SNAP * (0.2 + rng.next());
                    Point3::new(base.x() - r * 0.5, base.y() + r * 0.7, base.z() + r * 0.5)
                }
                5 => {
                    // Exactly on the radii.
                    let r = if rng.next() < 0.5 { TOL } else { SNAP };
                    Point3::new(base.x() + r, base.y(), base.z())
                }
                6 => {
                    // On a cell boundary, one ulp either side of it, or one
                    // query radius (and margin) away from it, so query
                    // blocks straddle cells.
                    #[allow(clippy::cast_precision_loss)]
                    let k = (rng.below(2001) as f64 - 1000.0) * cell;
                    let x = match rng.below(7) {
                        0 => k,
                        1 => f64::from_bits(k.to_bits() + 1),
                        2 => k - k.abs().max(f64::MIN_POSITIVE) * f64::EPSILON,
                        3 => k + TOL,
                        4 => k - SNAP,
                        5 => k + SNAP * (1.0 + MARGIN),
                        _ => k - TOL * (1.0 + MARGIN),
                    };
                    Point3::new(x, base.y(), base.z())
                }
                7 => {
                    // Far out: keys at the edge of, or past, the trusted range.
                    let far = MAX_KEY * cell * (0.999 + rng.next() * 0.002);
                    Point3::new(far, base.y(), -far)
                }
                8 => match rng.below(4) {
                    0 => Point3::new(f64::NAN, base.y(), base.z()),
                    1 => Point3::new(f64::INFINITY, base.y(), base.z()),
                    2 => Point3::new(base.x(), f64::NEG_INFINITY, base.z()),
                    _ => Point3::new(1.0e300, -1.0e300, 0.0),
                },
                _ => {
                    // Exactly equidistant from `base` along one dyadic step,
                    // so the snap tie rule decides.
                    let d = f64::powi(2.0, -18 - i32::try_from(rng.below(4)).unwrap());
                    if rng.next() < 0.5 {
                        Point3::new(base.x() + d, base.y(), base.z())
                    } else {
                        Point3::new(base.x() - d, base.y(), base.z())
                    }
                }
            };
            points.push(p);
        }
        points
    }

    /// Queries the grid answered from cells, and those whose block spans
    /// more than one cell, at the snap radius.
    fn grid_coverage(grid: &SnapGrid, queries: &[Point3]) -> (usize, usize) {
        let blocks: Vec<_> = queries
            .iter()
            .filter_map(|&q| grid.block(q, SNAP))
            .collect();
        let straddling = blocks
            .iter()
            .filter(|block| block.iter().any(|(lo, hi)| hi > lo))
            .count();
        (blocks.len(), straddling)
    }

    fn check(input: &[Point3], queries: &[Point3]) -> (usize, usize) {
        let reference = reference_dedup(input);
        let mut grid = SnapGrid::new(SNAP);
        for &p in input {
            grid.insert_unique(p, TOL);
        }
        assert_eq!(grid.points().len(), reference.len());
        for (a, b) in grid.points().iter().zip(&reference) {
            assert_eq!(bits(*a), bits(*b));
        }
        let mut snapped = 0;
        let mut ties = 0;
        for &q in queries {
            let expected = reference_snap(&reference, q);
            let got = grid.nearest_within(q, SNAP);
            assert_eq!(expected.map(bits), got.map(bits), "query {q:?}");
            if let Some(winner) = expected {
                snapped += 1;
                let d = (winner - q).length();
                if reference.iter().filter(|p| (**p - q).length() == d).count() > 1 {
                    ties += 1;
                }
            }
            // The dedup radius, the full reach, and radii the grid hands to
            // the full scan (above the reach, below the grid minimum).
            for radius in [TOL, SNAP, 3.0 * SNAP, 1.0e-150] {
                assert_eq!(
                    grid.any_within(q, radius),
                    reference.iter().any(|p| (*p - q).length() < radius),
                    "query {q:?} radius {radius}"
                );
            }
            assert_eq!(
                grid.nearest_within(q, 3.0 * SNAP).map(bits),
                scan_nearest(&reference, q, 3.0 * SNAP).map(bits),
                "query {q:?}"
            );
        }
        (snapped, ties)
    }

    #[test]
    fn grid_matches_linear_scans_on_random_clouds() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let (mut snapped, mut ties) = (0, 0);
        let (mut answered, mut straddling, mut total) = (0, 0, 0);
        for round in 0..60 {
            let scale = [1.0e-4, 1.0, 100.0, 1.0e4][round % 4];
            let input = cloud(&mut rng, scale, 400);
            let mut queries = cloud(&mut rng, scale, 200);
            // Probe the stored positions themselves and their neighbours.
            for &p in input.iter().step_by(3) {
                queries.push(p);
                queries.push(Point3::new(p.x() + SNAP * 0.5, p.y(), p.z()));
            }
            let (s, t) = check(&input, &queries);
            snapped += s;
            ties += t;
            let mut grid = SnapGrid::new(SNAP);
            for &p in &input {
                grid.insert_unique(p, TOL);
            }
            let (keyed, straddle) = grid_coverage(&grid, &queries);
            answered += keyed;
            straddling += straddle;
            total += queries.len();
        }
        // The corpus must exercise snapping and the tie rule, or the equality
        // above is vacuous.
        assert!(snapped > 1000, "snapped {snapped}");
        assert!(ties > 50, "ties {ties}");
        // Most queries must come from the cells, some from blocks across a
        // cell face; the rest exercise the full-scan fallback.
        assert!(answered * 2 > total, "{answered} of {total} from cells");
        assert!(straddling > 100, "straddling {straddling}");
        assert!(answered < total, "no fallback query");
    }

    #[test]
    fn equidistant_tie_goes_to_the_first_stored_position() {
        let q = Point3::new(1.0, 1.0, 1.0);
        let d = f64::powi(2.0, -20);
        let a = Point3::new(1.0 + d, 1.0, 1.0);
        let b = Point3::new(1.0 - d, 1.0, 1.0);
        for (first, second) in [(a, b), (b, a)] {
            let mut grid = SnapGrid::new(SNAP);
            grid.insert_unique(first, TOL);
            grid.insert_unique(second, TOL);
            assert_eq!(grid.points().len(), 2);
            assert_eq!(grid.nearest_within(q, SNAP).map(bits), Some(bits(first)));
            assert_eq!(
                reference_snap(&[first, second], q).map(bits),
                Some(bits(first))
            );
        }
    }

    #[test]
    fn unkeyed_positions_still_match() {
        let input = [
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(f64::INFINITY, 0.0, 0.0),
            Point3::new(1.0e300, 0.0, 0.0),
            Point3::new(1.0e300, 0.0, 0.0),
            Point3::new(1.0e300 + 1.0e284, 0.0, 0.0),
        ];
        let queries = [
            Point3::new(1.0e300, 0.0, 0.0),
            Point3::new(0.0, 0.0, 5.0e-6),
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(f64::INFINITY, 0.0, 0.0),
        ];
        check(&input, &queries);
        let mut degenerate = SnapGrid::new(0.0);
        degenerate.insert_unique(Point3::new(0.0, 0.0, 0.0), 0.0);
        assert_eq!(degenerate.points().len(), 1);
    }
}
