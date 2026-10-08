//! The planar cap of the PERF-D03 64-hole plate: a 100 × 100 square with an
//! 8 × 8 grid of r = 2 holes centred at 6 + 12k, each sampled as a regular
//! 18-gon, as `run_planar_cdt` in `remus-operations` receives it.
//!
//! The holes are translated copies of one exact regular polygon, so many
//! vertex quadruples are cocircular up to rounding. That is the input that
//! makes per-constraint and per-hole work dominate the tessellation.

use std::f64::consts::TAU;

use crate::vec::Point2;

/// Holes per row and per column.
pub(super) const GRID: usize = 8;

/// Samples per hole.
pub(super) const SIDES: usize = 18;

/// The centre of hole `k`, row by row.
pub(super) fn centre(k: usize) -> Point2 {
    let (row, col) = (k / GRID, k % GRID);
    Point2::new(6.0 + 12.0 * col as f64, 6.0 + 12.0 * row as f64)
}

/// The points (the square counter-clockwise, then each hole clockwise) and
/// the `(start, end)` range of each wire, outer first.
pub(super) fn layout() -> (Vec<Point2>, Vec<(usize, usize)>) {
    let mut pts = vec![
        Point2::new(0.0, 0.0),
        Point2::new(100.0, 0.0),
        Point2::new(100.0, 100.0),
        Point2::new(0.0, 100.0),
    ];
    let mut wires = vec![(0, pts.len())];
    for k in 0..GRID * GRID {
        let c = centre(k);
        let start = pts.len();
        for i in 0..SIDES {
            let a = -(i as f64) * TAU / SIDES as f64;
            pts.push(Point2::new(c.x() + 2.0 * a.cos(), c.y() + 2.0 * a.sin()));
        }
        wires.push((start, pts.len()));
    }
    (pts, wires)
}
