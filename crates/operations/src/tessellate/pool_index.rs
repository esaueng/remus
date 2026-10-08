//! Spatial index of the shared vertex pool for the circle contact
//! refinement of the boundary plan (PERF-D07).
//!
//! Stage A6 merges every pool vertex lying on a circle edge into that edge's
//! sample chain. Testing every pool vertex against every circle edge made it
//! one of the largest whole-body costs of a display tessellation (about a
//! fifth of a warm Hammer Holder mesh). The index narrows each circle's scan
//! to the vertices in grid cells along its arc; the caller then applies the
//! unchanged acceptance test to those candidates in ascending id order, so
//! the refined chains are identical to the full scan.

use remus_math::curves::Circle3D;
use remus_math::vec::Point3;

/// Upper bound on arc samples walked for one circle before falling back to
/// scanning the whole pool (a huge circle relative to the cell size).
const MAX_ARC_STEPS: usize = 1 << 16;

/// Callgrind Ir per pool vertex of the caller's full scan (hash probe,
/// `project`, `evaluate`, distances): 290-535 over the calibration sweep
/// (partial and full bench plates, box ∩ sphere, drilled box, cylinders, a
/// cone, fillets, full-turn tubes, mixed hole sizes; four tolerances).
const SCAN_IR_PER_VERTEX: usize = 540;

/// Callgrind Ir of a whole walk per walk point (`steps + 1`): 3020-7760
/// over the sweep, lowest for small circles whose points share cells.
const WALK_IR_PER_POINT: usize = 3000;

/// Callgrind Ir of the walk left once its cells are known (the 27-cell
/// expansion, its sort and lookups, the candidates' acceptance tests), per
/// distinct cell: 7510-12660 over the sweep. The cells themselves cost
/// 350-570 per walk point.
const WALK_IR_PER_CELL: usize = 7500;

/// Scan only when the scan estimate is at most this fraction (3/5) of the
/// walk estimate. The estimates are the sweep's extremes (dearest scan,
/// cheapest walk), so near the break-even the walk is kept: for every
/// circle of the sweep the rule scans, the walk cost at least 1.9x the scan
/// (plus the cells already built, after the cell check).
const SCAN_MARGIN: (usize, usize) = (3, 5);

/// Cells per side of the pool's bounding box (the cell size floor is set by
/// the acceptance tolerance, see [`PoolIndex::new`]).
const CELLS_PER_EXTENT: f64 = 128.0;

/// Uniform-grid index over pool positions, sorted by cell.
pub(super) struct PoolIndex {
    cell: f64,
    /// `(cell, gid)` sorted by cell then gid.
    entries: Vec<((i64, i64, i64), u32)>,
    /// Vertices with a non-finite coordinate: the acceptance test cannot
    /// reject them by distance, so they are always candidates.
    non_finite: Vec<u32>,
}

impl PoolIndex {
    /// Index `positions`. `accept_tol` is the largest distance from a circle
    /// at which the caller can accept a vertex; cells are kept well above it.
    pub(super) fn new(positions: &[Point3], accept_tol: f64) -> Option<Self> {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut non_finite = Vec::new();
        for (gid, p) in positions.iter().enumerate() {
            let c = [p.x(), p.y(), p.z()];
            if c.iter().all(|v| v.is_finite()) {
                for k in 0..3 {
                    lo[k] = lo[k].min(c[k]);
                    hi[k] = hi[k].max(c[k]);
                }
            } else {
                non_finite.push(u32::try_from(gid).ok()?);
            }
        }
        let extent = (0..3)
            .map(|k| hi[k] - lo[k])
            .filter(|e| e.is_finite())
            .fold(0.0_f64, f64::max);
        let cell = (extent / CELLS_PER_EXTENT).max(64.0 * accept_tol).max(1e-9);
        if !cell.is_finite() {
            return None;
        }
        let mut entries = Vec::with_capacity(positions.len());
        for (gid, &p) in positions.iter().enumerate() {
            if p.x().is_finite() && p.y().is_finite() && p.z().is_finite() {
                entries.push((cell_of(p, cell), u32::try_from(gid).ok()?));
            }
        }
        entries.sort_unstable();
        Some(Self {
            cell,
            entries,
            non_finite,
        })
    }

    /// Every pool vertex within half a cell of the circle's points with
    /// angle in `[t_lo, t_hi]` (the whole circle when `None`), in ascending
    /// id order. A superset of the vertices within `accept_tol` of that arc;
    /// `None` when the arc is too long to walk or scanning the whole pool is
    /// estimated to be clearly cheaper than the walk (scan the pool instead).
    pub(super) fn circle_candidates(
        &self,
        circle: &Circle3D,
        range: Option<(f64, f64)>,
    ) -> Option<Vec<u32>> {
        let radius = circle.radius();
        let (t_lo, t_hi) = match range {
            Some((lo, hi)) if hi - lo < std::f64::consts::TAU => (lo, hi),
            _ => (0.0, std::f64::consts::TAU),
        };
        if !(radius.is_finite() && t_lo.is_finite() && t_hi.is_finite()) {
            return None;
        }
        // Consecutive walk points at most half a cell apart along the arc,
        // so any point of the arc is within a quarter cell of one of them
        // and a vertex within `accept_tol` of the arc lies in that walk
        // point's cell or a neighbouring one.
        let step_angle = if radius > 0.0 {
            0.5 * self.cell / radius
        } else {
            std::f64::consts::TAU
        };
        let span = t_hi - t_lo + 2.0 * step_angle;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let steps = (span / step_angle).ceil().max(1.0) as usize;
        if steps > MAX_ARC_STEPS {
            return None;
        }
        // The caller's full scan gives the same chain (the same acceptance
        // test over every vertex in the same ascending order); take it when
        // clearly cheaper. The point count prices the walk before any cell
        // is built, which settles long arcs over small pools; the distinct
        // cells then price the rest of it.
        let pool_len = self.entries.len() + self.non_finite.len();
        let scan = scan_is_cheaper(pool_len, (steps + 1).saturating_mul(WALK_IR_PER_POINT));
        #[cfg(test)]
        let scan = test_hooks::decide(test_hooks::Check::Points, scan);
        if scan {
            return None;
        }
        let mut cells: Vec<(i64, i64, i64)> = Vec::with_capacity(steps + 1);
        for k in 0..=steps {
            #[allow(clippy::cast_precision_loss)]
            let t = (t_lo - step_angle) + span * (k as f64 / steps as f64);
            let p = circle.evaluate(t);
            if !(p.x().is_finite() && p.y().is_finite() && p.z().is_finite()) {
                return None;
            }
            cells.push(cell_of(p, self.cell));
        }
        cells.sort_unstable();
        cells.dedup();
        let scan = scan_is_cheaper(pool_len, cells.len().saturating_mul(WALK_IR_PER_CELL));
        #[cfg(test)]
        let scan = test_hooks::decide(test_hooks::Check::Cells(cells.len()), scan);
        if scan {
            return None;
        }
        let mut around: Vec<(i64, i64, i64)> = Vec::with_capacity(cells.len() * 27);
        for &(x, y, z) in &cells {
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        around.push((x + dx, y + dy, z + dz));
                    }
                }
            }
        }
        around.sort_unstable();
        around.dedup();
        let mut out: Vec<u32> = self.non_finite.clone();
        for key in around {
            let first = self.entries.partition_point(|(c, _)| *c < key);
            out.extend(
                self.entries[first..]
                    .iter()
                    .take_while(|(c, _)| *c == key)
                    .map(|&(_, gid)| gid),
            );
        }
        out.sort_unstable();
        out.dedup();
        Some(out)
    }
}

/// Whether scanning a pool of `pool_len` vertices is estimated to cost at
/// most [`SCAN_MARGIN`] of a walk estimated at `walk_ir`.
fn scan_is_cheaper(pool_len: usize, walk_ir: usize) -> bool {
    pool_len.saturating_mul(SCAN_IR_PER_VERTEX * SCAN_MARGIN.1)
        <= walk_ir.saturating_mul(SCAN_MARGIN.0)
}

#[allow(clippy::cast_possible_truncation)]
fn cell_of(p: Point3, cell: f64) -> (i64, i64, i64) {
    (
        (p.x() / cell).floor() as i64,
        (p.y() / cell).floor() as i64,
        (p.z() / cell).floor() as i64,
    )
}

/// Test control of the full-scan fallback in
/// [`PoolIndex::circle_candidates`].
#[cfg(test)]
pub(super) mod test_hooks {
    use std::cell::Cell;

    /// Which estimate of the walk a verdict compared the scan with.
    pub(super) enum Check {
        /// From the walk's point count, before any cell is built.
        Points,
        /// From the walk's distinct cells (their count).
        Cells(usize),
    }

    /// The cost rule's verdicts on this thread. With nothing forced every
    /// circle that reaches the rule adds exactly one walk or scan; forcing
    /// the walk also runs the cell check after a point-check scan, and
    /// forcing the scan stops after the point check.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub(in crate::tessellate) struct Verdicts {
        /// Point checks that scanned.
        pub(in crate::tessellate) point_scans: usize,
        /// Cell checks that scanned.
        pub(in crate::tessellate) cell_scans: usize,
        /// Cell checks that walked.
        pub(in crate::tessellate) walks: usize,
        /// Distinct cells summed over the cell checks.
        pub(in crate::tessellate) cells: usize,
    }

    impl Verdicts {
        /// Circles the rule scanned, at either check.
        pub(in crate::tessellate) const fn scans(&self) -> usize {
            self.point_scans + self.cell_scans
        }
    }

    thread_local! {
        /// `Some(true)` forces every circle to scan the pool, `Some(false)`
        /// forces the walk (the behaviour before the fallback existed).
        static FORCE: Cell<Option<bool>> = const { Cell::new(None) };
        /// The cost rule's verdicts on this thread.
        static VERDICTS: Cell<Verdicts> = const {
            Cell::new(Verdicts {
                point_scans: 0,
                cell_scans: 0,
                walks: 0,
                cells: 0,
            })
        };
    }

    /// Record the cost rule's verdict at `check` and apply any forced
    /// override.
    pub(super) fn decide(check: Check, scan: bool) -> bool {
        VERDICTS.with(|verdicts| {
            let mut v = verdicts.get();
            match check {
                Check::Points => v.point_scans += usize::from(scan),
                Check::Cells(cells) => {
                    v.cells += cells;
                    if scan {
                        v.cell_scans += 1;
                    } else {
                        v.walks += 1;
                    }
                }
            }
            verdicts.set(v);
        });
        FORCE.with(Cell::get).unwrap_or(scan)
    }

    /// Run `f` with the fallback forced (`Some`) or left to the cost rule
    /// (`None`); returns its result and the rule's verdicts during `f`.
    pub(in crate::tessellate) fn with_fallback<T>(
        force: Option<bool>,
        f: impl FnOnce() -> T,
    ) -> (T, Verdicts) {
        struct Restore(Option<bool>, Verdicts);
        impl Drop for Restore {
            fn drop(&mut self) {
                FORCE.with(|c| c.set(self.0));
                VERDICTS.with(|c| c.set(self.1));
            }
        }
        let _restore = Restore(
            FORCE.with(|c| c.replace(force)),
            VERDICTS.with(|c| c.replace(Verdicts::default())),
        );
        let out = f();
        (out, VERDICTS.with(Cell::get))
    }
}
