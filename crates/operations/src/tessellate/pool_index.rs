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
    /// `None` when the arc is too long to walk (scan the pool instead).
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

#[allow(clippy::cast_possible_truncation)]
fn cell_of(p: Point3, cell: f64) -> (i64, i64, i64) {
    (
        (p.x() / cell).floor() as i64,
        (p.y() / cell).floor() as i64,
        (p.z() / cell).floor() as i64,
    )
}
