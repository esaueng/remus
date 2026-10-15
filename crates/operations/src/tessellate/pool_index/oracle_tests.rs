//! Differential oracle for [`PoolIndex::circle_candidates`]: the version that
//! expanded every walked cell to its 27 neighbour keys, sorted them and looked
//! each one up, kept verbatim, against the merged z-run lookup. Candidate
//! lists must be identical, element for element, including the full-scan
//! (`None`) verdicts, over pools with clustered, scattered, coincident and
//! non-finite vertices and circles of every size, tilt, arc and offset.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};

use super::*;

impl PoolIndex {
    fn legacy_circle_candidates(
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

/// Deterministic generator (64-bit LCG, top bits).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        (hi - lo).mul_add(self.next(), lo)
    }
}

/// Points on and near circles (as a boundary plan's pool holds them), plus
/// a uniform scatter, duplicates and, optionally, non-finite entries.
fn pool(seed: u64, len: usize, extent: f64, non_finite: bool) -> Vec<Point3> {
    let mut rng = Lcg(seed);
    let mut points = Vec::with_capacity(len);
    while points.len() < len {
        match points.len() % 5 {
            0 | 1 => {
                let (r, t) = (
                    rng.range(0.1, extent * 0.4),
                    rng.range(0.0, std::f64::consts::TAU),
                );
                let z = rng.range(-extent * 0.1, extent * 0.1);
                points.push(Point3::new(r * t.cos(), r * t.sin(), z));
            }
            2 => points.push(Point3::new(
                rng.range(-extent, extent),
                rng.range(-extent, extent),
                rng.range(-extent, extent),
            )),
            3 => {
                let copy = points[points.len() / 2];
                points.push(copy);
            }
            _ => {
                if non_finite && points.len() % 7 == 4 {
                    points.push(Point3::new(f64::NAN, 0.0, rng.next()));
                } else if non_finite && points.len() % 11 == 4 {
                    points.push(Point3::new(0.0, f64::INFINITY, 1.0));
                } else {
                    points.push(Point3::new(
                        rng.range(-1.0, 1.0),
                        rng.range(-1.0, 1.0),
                        rng.range(-1.0, 1.0),
                    ));
                }
            }
        }
    }
    points
}

fn circles(seed: u64, extent: f64) -> Vec<(Circle3D, Option<(f64, f64)>)> {
    let mut rng = Lcg(seed);
    let mut out = Vec::new();
    for k in 0..60 {
        let normal = match k % 4 {
            0 => Vec3::new(0.0, 0.0, 1.0),
            1 => Vec3::new(1.0, 0.0, 0.0),
            _ => Vec3::new(
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(0.1, 1.0),
            ),
        };
        let center = if k % 3 == 0 {
            Point3::new(0.0, 0.0, rng.range(-extent * 0.1, extent * 0.1))
        } else {
            Point3::new(
                rng.range(-extent, extent),
                rng.range(-extent, extent),
                rng.range(-extent, extent),
            )
        };
        let radius = match k % 6 {
            0 => rng.range(1e-4, 1e-2) * extent,
            1 => rng.range(0.05, 0.5) * extent,
            2 => rng.range(0.5, 3.0) * extent,
            _ => rng.range(0.01, 0.2) * extent,
        };
        let Ok(circle) = Circle3D::new(center, normal, radius) else {
            continue;
        };
        let start = rng.range(-7.0, 7.0);
        let range = match k % 5 {
            0 => None,
            1 => Some((start, start + std::f64::consts::TAU + 0.5)),
            _ => Some((start, start + rng.range(1e-3, 6.0))),
        };
        out.push((circle, range));
    }
    out
}

fn assert_same_candidates(index: &PoolIndex, circles: &[(Circle3D, Option<(f64, f64)>)]) -> usize {
    let mut walked = 0;
    for (circle, range) in circles {
        let legacy = index.legacy_circle_candidates(circle, *range);
        let merged = index.circle_candidates(circle, *range);
        assert_eq!(legacy, merged, "circle {circle:?} range {range:?}");
        walked += usize::from(merged.is_some());
    }
    walked
}

#[test]
fn merged_runs_return_the_expanded_neighbour_candidates() {
    let mut walked = 0;
    for (seed, len, extent, non_finite) in [
        (1, 50, 1.0, false),
        (2, 2_000, 10.0, false),
        (3, 20_000, 100.0, true),
        (4, 5_000, 1e-3, false),
        (5, 8_000, 1e4, true),
    ] {
        let points = pool(seed, len, extent, non_finite);
        for accept_tol in [1e-9, 1e-6 * extent, 1e-3 * extent] {
            let Some(index) = PoolIndex::new(&points, accept_tol) else {
                continue;
            };
            let probes = circles(seed * 31 + 7, extent);
            // The cost rule as shipped, then every circle forced to walk so
            // the lookup itself is compared on every arc.
            walked += assert_same_candidates(&index, &probes);
            let (forced, _) =
                test_hooks::with_fallback(Some(false), || assert_same_candidates(&index, &probes));
            walked += forced;
        }
    }
    assert!(walked > 1_000, "{walked} walks compared");
}
