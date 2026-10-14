//! `polygon_row_contains` answers exactly what `point_in_polygon_2d` answers
//! for every point of a row, including points on vertices and segments.

#![allow(clippy::unwrap_used)]

use super::{point_in_polygon_2d, polygon_row_contains};
use remus_math::vec::Point2;

/// Xorshift, so the cases are reproducible without a dependency.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// A coordinate on a coarse lattice (so rows and points often land
    /// exactly on vertices and segments) or a fine-grained real.
    #[allow(clippy::cast_precision_loss)]
    fn coordinate(&mut self) -> f64 {
        if self.next().is_multiple_of(2) {
            (self.next() % 9) as f64 * 0.5
        } else {
            (self.next() % 1_000_000) as f64 * 4.5e-6
        }
    }
}

#[test]
fn row_containment_matches_the_per_point_test() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut inside_points = 0;
    for _ in 0..3000 {
        let vertices = 3 + usize::try_from(rng.next() % 9).unwrap();
        let polygon: Vec<(f64, f64)> = (0..vertices)
            .map(|_| (rng.coordinate(), rng.coordinate()))
            .collect();
        let mut xs: Vec<f64> = (0..=rng.next() % 40).map(|_| rng.coordinate()).collect();
        xs.sort_by(f64::total_cmp);
        let y = if rng.next().is_multiple_of(3) {
            polygon[usize::try_from(rng.next()).unwrap() % vertices].1
        } else {
            rng.coordinate()
        };
        let rows = polygon_row_contains(&polygon, &xs, y);
        for (&x, &inside) in xs.iter().zip(&rows) {
            assert_eq!(
                inside,
                point_in_polygon_2d(&polygon, Point2::new(x, y)),
                "polygon {polygon:?} point ({x}, {y})"
            );
            inside_points += usize::from(inside);
        }
    }
    assert!(inside_points > 1000, "inside points {inside_points}");
}
