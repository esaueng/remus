//! Chaining intersection points into curves.

use crate::MathError;
use crate::det_hash::DetHashMap;
use crate::nurbs::fitting::{approximate_lspia, chord_length_params, interpolate};
use crate::nurbs::projection::project_point_to_curve;
use crate::vec::Point3;

use super::{IntersectionCurve, IntersectionPoint};

/// Below this input size the grid's build cost exceeds the all-pairs scan
/// it replaces (measured crossover near 200 points), so short chains keep
/// the verbatim scan. See [`chain_intersection_points`].
const GRID_THRESHOLD: usize = 256;

/// Ring-expansion cap per walk step; the stop rule fires far earlier on
/// every real input, so this only bounds degenerate expansion before the
/// full-scan fallback takes over with the identical outcome.
const RING_CAP: i64 = 32;

/// Build intersection curves from a set of points by chaining and fitting.
///
/// First chains points into connected components (separate intersection
/// branches), then fits a NURBS curve through each chain independently.
pub(super) fn build_curves_from_points(
    points: &[IntersectionPoint],
) -> Result<Vec<IntersectionCurve>, MathError> {
    if points.is_empty() {
        return Ok(Vec::new());
    }

    // Estimate a chaining threshold from the average spacing.
    let threshold = estimate_chain_threshold(points);

    // Chain points into connected components.
    let chains = chain_intersection_points(points, threshold);

    let mut curves = Vec::with_capacity(chains.len());

    for chain in &chains {
        // Deduplicate closely spaced points within the chain.
        let mut deduped: Vec<IntersectionPoint> = Vec::new();
        for pt in chain {
            let is_dup = deduped
                .last()
                .is_some_and(|last: &IntersectionPoint| (last.point - pt.point).length() < 1e-6);
            if !is_dup {
                deduped.push(*pt);
            }
        }

        if deduped.len() < 2 {
            continue;
        }

        // Fit a NURBS curve through this chain's points.
        let positions: Vec<Point3> = deduped.iter().map(|p| p.point).collect();
        let degree = if positions.len() <= 3 {
            1
        } else {
            3.min(positions.len() - 1)
        };
        let curve = if positions.len() > 50 {
            let num_cps = (positions.len() / 3).max(degree + 1).min(positions.len());
            let fitted = approximate_lspia(&positions, degree, num_cps, 1e-6, 100)?;

            // Validate fit quality: re-evaluate residual at each sample point
            // using the same chord-length parameterisation used during fitting.
            // Use a relative threshold (residual / point-cloud diagonal) so the
            // check is scale-independent.  A relative residual > 1% warrants a
            // warning; the intersection curve may be geometrically inaccurate.
            let fit_params = chord_length_params(&positions);
            let mut max_residual = 0.0f64;
            let mut bbox_min = positions[0];
            let mut bbox_max = positions[0];
            for (i, &t) in fit_params.iter().enumerate() {
                let src = positions[i];
                // Nearest-point projection gives the true geometric residual.
                // Fall back to parametric evaluation only for degenerate curves.
                let d = if let Ok(proj) = project_point_to_curve(&fitted, src, 1e-6) {
                    proj.distance
                } else {
                    let pt = fitted.evaluate(t);
                    (pt.x() - src.x()).hypot((pt.y() - src.y()).hypot(pt.z() - src.z()))
                };
                max_residual = max_residual.max(d);
                bbox_min = Point3::new(
                    bbox_min.x().min(src.x()),
                    bbox_min.y().min(src.y()),
                    bbox_min.z().min(src.z()),
                );
                bbox_max = Point3::new(
                    bbox_max.x().max(src.x()),
                    bbox_max.y().max(src.y()),
                    bbox_max.z().max(src.z()),
                );
            }
            let diagonal = (bbox_max.x() - bbox_min.x())
                .hypot((bbox_max.y() - bbox_min.y()).hypot(bbox_max.z() - bbox_min.z()));
            let rel_residual = if diagonal > 1e-12 {
                max_residual / diagonal
            } else {
                max_residual
            };
            if rel_residual > 1e-2 {
                log::warn!(
                    "SSI: LSPIA fit relative residual {rel_residual:.2e} (abs={max_residual:.2e}) \
                     exceeds 1% of curve extent — intersection curve may be inaccurate \
                     (degree={degree}, num_cps={num_cps}, samples={})",
                    positions.len()
                );
            }
            fitted
        } else {
            interpolate(&positions, degree)?
        };

        curves.push(IntersectionCurve {
            curve,
            points: deduped,
        });
    }

    Ok(curves)
}

/// Estimate a reasonable chaining threshold from point spacing.
#[allow(clippy::cast_precision_loss)]
#[must_use]
pub(super) fn estimate_chain_threshold(points: &[IntersectionPoint]) -> f64 {
    if points.len() < 2 {
        return 1.0;
    }

    // Compute average nearest-neighbor distance (sample up to 100 points for speed).
    let sample_size = points.len().min(100);
    let mut total_min_dist = 0.0_f64;
    let mut count = 0_usize;
    for i in 0..sample_size {
        let mut min_d = f64::MAX;
        for (j, q) in points.iter().enumerate() {
            if i == j {
                continue;
            }
            let d = (points[i].point - q.point).length();
            if d < min_d {
                min_d = d;
            }
        }
        if min_d < f64::MAX {
            total_min_dist += min_d;
            count += 1;
        }
    }

    if count == 0 {
        return 1.0;
    }

    // Use 3x average nearest-neighbor distance as threshold.
    // The threshold must be large enough to chain adjacent sampling
    // points along the same intersection branch. We also compute
    // the bounding box diagonal as an upper-bound reference.
    let avg = total_min_dist / count as f64;

    // Also compute the bounding box diagonal of all points.
    let mut bb_min = [f64::MAX; 3];
    let mut bb_max = [f64::MIN; 3];
    for p in points {
        bb_min[0] = bb_min[0].min(p.point.x());
        bb_min[1] = bb_min[1].min(p.point.y());
        bb_min[2] = bb_min[2].min(p.point.z());
        bb_max[0] = bb_max[0].max(p.point.x());
        bb_max[1] = bb_max[1].max(p.point.y());
        bb_max[2] = bb_max[2].max(p.point.z());
    }
    let diag = ((bb_max[0] - bb_min[0]).powi(2)
        + (bb_max[1] - bb_min[1]).powi(2)
        + (bb_max[2] - bb_min[2]).powi(2))
    .sqrt();

    // Floor: 5% of the bounding diagonal, which handles cases where
    // many points converge to the same location after Newton refinement.
    let floor = diag * 0.05;
    (avg * 3.0).max(floor).max(1e-4)
}

/// Chain intersection points into connected components using proximity.
///
/// Points within `threshold` distance are considered connected. Returns
/// ordered chains (each chain is a connected component, ordered by
/// nearest-neighbor walk). Closed loops are detected when the last
/// point is within `threshold` of the first.
///
/// Scaling (PERF-N03): the threshold graph is built through a uniform spatial
/// grid instead of all-pairs enumeration, and each component's
/// nearest-neighbor walk expands grid cells ring by ring instead of scanning
/// every unused point per step. Both preserve the all-pairs outcome
/// bit-for-bit:
///
/// - Grid cells are a power-of-two width, so point-to-cell assignment uses an
///   exact division: the computed cell always equals the true cell and every
///   within-threshold pair shares a 27-cell neighborhood. The same pairwise
///   distance expression tests the same candidate pairs; adjacency lists are
///   sorted back into index order, so breadth-first discovery matches.
/// - The walk keeps the exact argmin with the same first-in-component
///   tie-break. Ring expansion stops only once every unsearched ring is
///   provably beyond the incumbent (plus one extra ring and a rounding
///   tolerance that covers distance rounding with wide margin); ties can
///   never hide past the stop ring. A ring cap falls back to the full scan
///   for that step, so no input can silently take a different path.
/// - Degenerate thresholds keep their comparison outcome without enumerating
///   pairs: NaN/zero connect nothing, an infinite threshold connects every
///   finite-distance pair.
#[must_use]
pub fn chain_intersection_points(
    points: &[IntersectionPoint],
    threshold: f64,
) -> Vec<Vec<IntersectionPoint>> {
    if points.is_empty() {
        return Vec::new();
    }

    let n = points.len();
    let cell = threshold.abs();
    // NaN and zero thresholds connect nothing: every `< threshold²`
    // comparison is false (NaN poisons the test; squared distances are
    // never negative). Each point is its own component, as in the scan.
    if cell.is_nan() || cell == 0.0 {
        return points.iter().map(|p| vec![*p]).collect();
    }
    if !cell.is_finite() {
        // Infinite threshold: every finite-distance pair connects. Points
        // with a non-finite coordinate never connect (their pairwise
        // distances are NaN or infinite, never `< inf`).
        return chain_with_clique(points);
    }
    // Smallest power of two >= cell, built from exponent bits (exact, no
    // libm rounding): cell assignment below divides by a power of two, which
    // is exact, so the computed cell always equals the true cell. A
    // downward-rounded `log2` can land one exponent low; one exact doubling
    // then restores `width >= cell` while staying a power of two.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let exp = cell.log2().ceil() as i32;
    if exp > 1023 {
        // Threshold above ~2^1023: its square overflows to infinity in the
        // comparison, so this is the infinite case in disguise.
        return chain_with_clique(points);
    }
    let mut width = if exp >= -1022 {
        f64::from_bits((exp + 1023) as u64 * 0x10_0000_0000_0000)
    } else {
        // Subnormal powers of two: biased exponent field is zero.
        f64::from_bits(1u64 << (exp + 1074))
    };
    if width < cell {
        width *= 2.0;
    }
    if !width.is_finite() {
        // Only when `cell` itself sits above 2^1022: same overflow case.
        return chain_with_clique(points);
    }

    // Below this size the grid's build cost exceeds the all-pairs scan it
    // replaces (measured crossover near 200 points: 128 runs ~76µs scanned
    // vs ~101µs gridded, 512 runs ~1156µs vs ~415µs). Small inputs keep the
    // verbatim scan, so short SSI chains never regress.
    if n < GRID_THRESHOLD {
        let threshold_sq = threshold * threshold;
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for i in 0..n {
            for j in (i + 1)..n {
                let d = points[i].point - points[j].point;
                if d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z())) < threshold_sq {
                    adj[i].push(j);
                    adj[j].push(i);
                }
            }
        }
        return chain_from_adjacency(points, &adj, None);
    }

    let grid = ChainGrid::build(points, width);

    // Build adjacency through the grid: for each point, test the points in
    // its 27-cell neighborhood with the same distance expression the
    // all-pairs scan used. A within-threshold pair is at most one true cell
    // apart per axis (crossing two exact-width boundaries needs twice the
    // width, which exceeds the threshold), so the neighborhood always
    // contains it. Per-list sorting restores index order for discovery.
    let threshold_sq = threshold * threshold;
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (i, p) in points.iter().enumerate() {
        let Some(home) = grid.cell_of_point(p.point) else {
            continue; // Non-finite coordinate: isolated, as in the scan.
        };
        let (keys, count) = ChainGrid::neighborhood(home);
        for key in keys.iter().take(count) {
            let Some(members) = grid.members_of(*key) else {
                continue;
            };
            for &j in members {
                if j <= i {
                    continue;
                }
                let d = p.point - points[j].point;
                if d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z())) < threshold_sq {
                    adj[i].push(j);
                    adj[j].push(i);
                }
            }
        }
    }
    for list in &mut adj {
        list.sort_unstable();
    }

    chain_from_adjacency(points, &adj, Some(&grid))
}

/// Chain when the threshold connects every finite-distance pair.
///
/// Used for infinite (or square-overflowing) thresholds. Finite-coordinate
/// points form cliques in breadth-first discovery order; non-finite points
/// stay isolated. The walk runs the full scan: rings are meaningless without
/// a finite cell width.
fn chain_with_clique(points: &[IntersectionPoint]) -> Vec<Vec<IntersectionPoint>> {
    let n = points.len();
    let finite: Vec<bool> = points
        .iter()
        .map(|p| {
            let q = p.point;
            q.x().is_finite() && q.y().is_finite() && q.z().is_finite()
        })
        .collect();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        if !finite[i] {
            continue;
        }
        for j in (i + 1)..n {
            if finite[j] {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }
    chain_from_adjacency(points, &adj, None)
}

/// Breadth-first components plus the nearest-neighbor walk over an adjacency
/// built either way. The component walk is grid-accelerated when a grid is
/// supplied and runs the full scan otherwise; both spell the same outcome.
fn chain_from_adjacency(
    points: &[IntersectionPoint],
    adj: &[Vec<usize>],
    grid: Option<&ChainGrid>,
) -> Vec<Vec<IntersectionPoint>> {
    let n = points.len();

    // BFS to find connected components.
    let mut visited = vec![false; n];
    let mut components: Vec<Vec<usize>> = Vec::new();

    for start in 0..n {
        if visited[start] {
            continue;
        }
        let mut component = Vec::new();
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start);
        visited[start] = true;
        while let Some(idx) = queue.pop_front() {
            component.push(idx);
            for &neighbor in &adj[idx] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }
        components.push(component);
    }

    // Scratch stamps shared across components: component membership (+rank)
    // and per-walk used marks. Epoch counters keep every component's work
    // linear instead of clearing whole arrays per component.
    let mut membership = StampSet::new(n);
    let mut used = StampSet::new(n);

    // Order each component via nearest-neighbor walk.
    let mut chains = Vec::with_capacity(components.len());
    for comp in &components {
        if comp.is_empty() {
            continue;
        }

        // Find endpoint: a point with degree <= 1 in the adjacency (within component).
        let epoch = membership.next_epoch();
        for (rank, &i) in comp.iter().enumerate() {
            membership.mark(i, epoch, rank);
        }
        let is_member = |i: usize| membership.is_marked(i, epoch);
        let rank_of = |i: usize| membership.value(i, epoch);
        let start_idx = comp
            .iter()
            .copied()
            .min_by_key(|&i| adj[i].iter().filter(|&&j| is_member(j)).count())
            .unwrap_or(comp[0]);

        let mut chain = Vec::with_capacity(comp.len());
        if comp.len() == 1 {
            chain.push(points[comp[0]]);
            chains.push(chain);
            continue;
        }
        // Rounding tolerance for the ring stop rule needs the component's
        // coordinate magnitude (NaN-safe: `max` skips NaN operands, and a
        // leftover non-finite magnitude only widens the net to the fallback).
        let mut cmax = 0.0f64;
        for &i in comp {
            let p = points[i].point;
            cmax = cmax.max(p.x().abs()).max(p.y().abs()).max(p.z().abs());
        }
        let cmax_sq = cmax * cmax;
        let used_epoch = used.next_epoch();
        let mut current = start_idx;
        used.mark(current, used_epoch, 0);
        chain.push(points[current]);

        for _ in 1..comp.len() {
            // Find nearest unused point in the component.
            let next = match grid {
                Some(g) => nearest_unused_ring(
                    points, comp, adj, g, current, cmax_sq, used_epoch, &used, &is_member, &rank_of,
                ),
                None => nearest_unused_scan(points, comp, current, used_epoch, &used),
            };

            if let Some(next) = next {
                used.mark(next, used_epoch, 0);
                chain.push(points[next]);
                current = next;
            } else {
                break;
            }
        }

        chains.push(chain);
    }

    chains
}

/// Baseline full scan for the nearest unused component member.
///
/// Kept for the grid-less (infinite-threshold) path and as the per-step
/// fallback when ring expansion hits its cap. Compares with strict
/// less-than in component order, so the first minimum wins ties.
fn nearest_unused_scan(
    points: &[IntersectionPoint],
    comp: &[usize],
    current: usize,
    used_epoch: u32,
    used: &StampSet,
) -> Option<usize> {
    let mut best_dist = f64::MAX;
    let mut best_idx = None;
    for &idx in comp {
        if used.is_marked(idx, used_epoch) {
            continue;
        }
        let d = points[current].point - points[idx].point;
        let dist_sq = d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z()));
        if dist_sq < best_dist {
            best_dist = dist_sq;
            best_idx = Some(idx);
        }
    }
    best_idx
}

/// Ring-bounded nearest-unused search through the spatial grid.
///
/// Two tiers, both exact. Tier one scans the current point's own adjacency:
/// every neighbor sits within threshold, so whenever any of them is unused,
/// the global minimum is among them and the rings never run. Otherwise tier
/// two examines expanding Chebyshev rings around the current point's cell
/// and stops once every unsearched ring is provably beyond the incumbent
/// (plus one extra ring and a rounding tolerance that covers distance
/// rounding with wide margin); ties can never hide past the stop ring. The
/// incumbent keeps the exact first-in-component tie-break by comparing
/// component ranks on equal distances. If expansion reaches the ring cap
/// without stopping, the step falls back to the full scan, which spells
/// the same outcome.
#[allow(clippy::too_many_arguments, clippy::cast_precision_loss)]
// Exact `==` tie-breaks below are intentional: they reproduce the scan's
// first-in-component winner on equal distances bit-for-bit. A margin would
// pick a different point than the baseline on near-ties.
#[allow(clippy::float_cmp)]
fn nearest_unused_ring(
    points: &[IntersectionPoint],
    comp: &[usize],
    adj: &[Vec<usize>],
    grid: &ChainGrid,
    current: usize,
    cmax_sq: f64,
    used_epoch: u32,
    used: &StampSet,
    is_member: &impl Fn(usize) -> bool,
    rank_of: &impl Fn(usize) -> u32,
) -> Option<usize> {
    // Tier one: adjacency members are exactly the component members within
    // threshold (edges connect), so a hit here is the global minimum.
    let mut best_sq = f64::MAX;
    let mut best_rank = u32::MAX;
    let mut best_idx = None;
    for &m in &adj[current] {
        if used.is_marked(m, used_epoch) {
            continue;
        }
        let d = points[current].point - points[m].point;
        let dist_sq = d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z()));
        let rank = rank_of(m);
        if dist_sq < best_sq || (best_idx.is_some() && dist_sq == best_sq && rank < best_rank) {
            best_sq = dist_sq;
            best_rank = rank;
            best_idx = Some(m);
        }
    }
    if best_idx.is_some() {
        return best_idx;
    }
    let Some(home) = grid.cell_of_index(points, current) else {
        // Only reachable for non-finite points, which never share an edge
        // and therefore never walk: keep the baseline outcome regardless.
        return nearest_unused_scan(points, comp, current, used_epoch, used);
    };
    // Ring cap: the stop rule fires far earlier on every real input (rings
    // past the incumbent plus two); past the cap the full scan takes over
    // with the identical outcome. The cap only bounds degenerate expansion.
    // Tier one already ruled out a within-threshold minimum, so the rings
    // start from a clean slate.
    let w = grid.width;
    let mut ring: i64 = 0;
    loop {
        for key in ChainGrid::ring_cells(home, ring) {
            let Some(members) = grid.members_of(key) else {
                continue;
            };
            for &m in members {
                if !is_member(m) || used.is_marked(m, used_epoch) {
                    continue;
                }
                let d = points[current].point - points[m].point;
                let dist_sq = d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z()));
                let rank = rank_of(m);
                if dist_sq < best_sq
                    || (best_idx.is_some() && dist_sq == best_sq && rank < best_rank)
                {
                    best_sq = dist_sq;
                    best_rank = rank;
                    best_idx = Some(m);
                }
            }
        }
        if best_idx.is_some() && ring >= 2 {
            // Rounding tolerance: covers distance rounding (a few ulps of
            // the coordinate magnitude) with wide margin. Only widens the
            // examined net on huge-coordinate inputs; never skips wrongly.
            let tol_sq = 2.0f64.powi(-40) * (cmax_sq + best_sq + w * w + 1e-280);
            let reach = (ring - 1) as f64 * w;
            if reach * reach > best_sq + tol_sq {
                break;
            }
        }
        if ring >= RING_CAP {
            return nearest_unused_scan(points, comp, current, used_epoch, used);
        }
        ring += 1;
    }
    best_idx
}

/// Uniform spatial grid over finite chaining input points.
///
/// The cell width is a power of two, so `point / width` is exact and the
/// floored cell always equals the true cell: every within-threshold pair is
/// at most one cell apart per axis and shares a 27-cell neighborhood.
/// Member lists stay in ascending index order.
struct ChainGrid {
    width: f64,
    cells: DetHashMap<(i64, i64, i64), Vec<usize>>,
}

impl ChainGrid {
    /// Build the grid, skipping points with a non-finite coordinate (they
    /// never connect, exactly as in the all-pairs comparison).
    fn build(points: &[IntersectionPoint], width: f64) -> Self {
        let mut cells: DetHashMap<(i64, i64, i64), Vec<usize>> = DetHashMap::default();
        for (i, p) in points.iter().enumerate() {
            if let Some(key) = cell_key(p.point, width) {
                cells.entry(key).or_default().push(i);
            }
        }
        Self { width, cells }
    }

    /// The cell holding point `i`, if its coordinates are finite.
    fn cell_of_index(&self, points: &[IntersectionPoint], i: usize) -> Option<(i64, i64, i64)> {
        cell_key(points[i].point, self.width)
    }

    /// The cell holding a bare position, if finite.
    fn cell_of_point(&self, p: Point3) -> Option<(i64, i64, i64)> {
        cell_key(p, self.width)
    }

    /// Members of a cell in ascending index order, if the cell is occupied.
    fn members_of(&self, key: (i64, i64, i64)) -> Option<&[usize]> {
        self.cells.get(&key).map(Vec::as_slice)
    }

    /// The 27 cells around `home`, skipping key arithmetic overflow
    /// (saturating indices already share their cell, so nothing is lost).
    /// Returns the filled prefix; overflow-deduplicated slots are omitted so
    /// no pair is ever tested twice.
    fn neighborhood(home: (i64, i64, i64)) -> ([(i64, i64, i64); 27], usize) {
        let mut keys = [(0i64, 0i64, 0i64); 27];
        let mut count = 0;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (
                        home.0.checked_add(dx).unwrap_or(home.0),
                        home.1.checked_add(dy).unwrap_or(home.1),
                        home.2.checked_add(dz).unwrap_or(home.2),
                    );
                    // Deduplicate the saturated fallback so pairs are tested once.
                    if !keys[..count].contains(&key) {
                        keys[count] = key;
                        count += 1;
                    }
                }
            }
        }
        (keys, count)
    }

    /// Cells at exactly Chebyshev distance `ring` from `home`, skipping key
    /// arithmetic overflow the same way as [`neighborhood`](Self::neighborhood).
    fn ring_cells(home: (i64, i64, i64), ring: i64) -> Vec<(i64, i64, i64)> {
        let mut keys = Vec::new();
        for dx in -ring..=ring {
            for dy in -ring..=ring {
                for dz in -ring..=ring {
                    if dx.abs().max(dy.abs()).max(dz.abs()) != ring {
                        continue;
                    }
                    let Some(x) = home.0.checked_add(dx) else {
                        continue;
                    };
                    let Some(y) = home.1.checked_add(dy) else {
                        continue;
                    };
                    let Some(z) = home.2.checked_add(dz) else {
                        continue;
                    };
                    keys.push((x, y, z));
                }
            }
        }
        keys
    }
}

/// Cell assignment with an exact power-of-two divisor: the quotient is
/// exact, so flooring always lands in the true cell. Returns `None` for
/// non-finite coordinates.
fn cell_key(p: Point3, width: f64) -> Option<(i64, i64, i64)> {
    if !(p.x().is_finite() && p.y().is_finite() && p.z().is_finite()) {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)]
    Some((
        (p.x() / width).floor() as i64,
        (p.y() / width).floor() as i64,
        (p.z() / width).floor() as i64,
    ))
}

/// Epoch-stamped boolean marks with payloads over a fixed index range.
///
/// Marking is O(1) and a fresh epoch retires every previous mark without
/// clearing the arrays, so per-component setup stays linear in the
/// component instead of the whole input. Epoch exhaustion (unreachable in
/// practice) zeroes the arrays and restarts.
struct StampSet {
    epochs: Vec<u32>,
    values: Vec<u32>,
    next: u32,
}

impl StampSet {
    fn new(n: usize) -> Self {
        Self {
            epochs: vec![0; n],
            values: vec![0; n],
            next: 1,
        }
    }

    /// Reserve a fresh epoch, retiring all previous marks.
    fn next_epoch(&mut self) -> u32 {
        if self.next == u32::MAX {
            self.epochs.fill(0);
            self.values.fill(0);
            self.next = 1;
        }
        let epoch = self.next;
        self.next += 1;
        epoch
    }

    /// Mark index `i` under `epoch` with payload `value`.
    #[allow(clippy::cast_possible_truncation)]
    fn mark(&mut self, i: usize, epoch: u32, value: usize) {
        self.epochs[i] = epoch;
        self.values[i] = value as u32;
    }

    /// Whether index `i` carries `epoch`'s mark.
    fn is_marked(&self, i: usize, epoch: u32) -> bool {
        self.epochs[i] == epoch
    }

    /// The payload stored by [`mark`](Self::mark); only valid for marked
    /// indices under the same epoch.
    fn value(&self, i: usize, epoch: u32) -> u32 {
        debug_assert_eq!(self.epochs[i], epoch);
        self.values[i]
    }
}
