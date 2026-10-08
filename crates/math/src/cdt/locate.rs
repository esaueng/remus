use crate::MathError;
use crate::predicates::orient2d;
use crate::vec::Point2;

use super::Cdt;

/// Location of a point within a triangle.
pub(super) enum PointLocation {
    /// Point is strictly inside the triangle.
    Inside,
    /// Point is on the edge with the given local index (0, 1, or 2).
    /// Local edge `i` is opposite vertex `v[i]`, connecting `v[(i+1)%3]`
    /// and `v[(i+2)%3]`.
    OnEdge(usize),
}

impl Cdt {
    /// Locate the triangle containing point `p` by walking from a hint
    /// triangle (exploits spatial coherence for sequential insertions).
    pub(super) fn locate_point(&self, p: Point2) -> Result<(usize, PointLocation), MathError> {
        // Start from the last successfully located triangle for spatial coherence.
        // Fall back to the last non-removed triangle if the hint is stale.
        let mut current = if self.last_located < self.triangles.len()
            && !self.triangles[self.last_located].removed
        {
            self.last_located
        } else {
            self.triangles
                .iter()
                .rposition(|t| !t.removed)
                .ok_or(MathError::ConvergenceFailure { iterations: 0 })?
        };

        let max_steps = self.triangles.len() * 2 + 10;
        for _ in 0..max_steps {
            let tri = &self.triangles[current];
            if tri.removed {
                // Shouldn't happen, but recover by scanning.
                current = self
                    .triangles
                    .iter()
                    .rposition(|t| !t.removed)
                    .ok_or(MathError::ConvergenceFailure { iterations: 0 })?;
                continue;
            }

            let v0 = self.vertices[tri.v[0]];
            let v1 = self.vertices[tri.v[1]];
            let v2 = self.vertices[tri.v[2]];

            let d0 = orient2d(v1, v2, p);
            let d1 = orient2d(v2, v0, p);
            let d2 = orient2d(v0, v1, p);

            // Check if point is on an edge (orientation ~ 0 while others >= 0).
            let on_edge_tol = 0.0;

            if d0 > on_edge_tol && d1 > on_edge_tol && d2 > on_edge_tol {
                return Ok((current, PointLocation::Inside));
            }

            // Check for on-edge conditions.
            if d0.abs() <= on_edge_tol && d1 >= 0.0 && d2 >= 0.0 {
                // On edge opposite v[0].
                return Ok((current, PointLocation::OnEdge(0)));
            }
            if d1.abs() <= on_edge_tol && d0 >= 0.0 && d2 >= 0.0 {
                return Ok((current, PointLocation::OnEdge(1)));
            }
            if d2.abs() <= on_edge_tol && d0 >= 0.0 && d1 >= 0.0 {
                return Ok((current, PointLocation::OnEdge(2)));
            }

            // All positive or zero means inside.
            if d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0 {
                return Ok((current, PointLocation::Inside));
            }

            // Walk toward the point: cross the most negative edge.
            let min_d = d0.min(d1).min(d2);
            let edge_to_cross = if (d0 - min_d).abs() < 1e-30 {
                0
            } else if (d1 - min_d).abs() < 1e-30 {
                1
            } else {
                2
            };

            if let Some(adj) = tri.adj[edge_to_cross] {
                current = adj;
            } else {
                // No adjacent triangle — point is outside. Fall back to scan.
                return self.locate_point_scan(p);
            }
        }

        // Fallback: brute-force scan.
        self.locate_point_scan(p)
    }

    /// Locate each seed of [`Cdt::flood_remove_from_points`] in a live
    /// triangle that strictly contains it, or `None`.
    ///
    /// The walks chain: each starts where the previous seed was found, so
    /// seeds listed hole by hole walk between neighbouring holes. The first
    /// starts from the same hint as [`Cdt::locate_point`]. A walk gives up on
    /// a seed that is not finite, that lies on an edge or vertex, or that it
    /// reaches only through a removed or missing neighbour, and every walk
    /// stops once their shared budget of `2·T + 16·S` steps is spent. Seeds
    /// given up on take the one-seed lookup when they are flooded.
    pub(super) fn locate_seeds_strictly_inside(&self, seeds: &[Point2]) -> Vec<Option<usize>> {
        let hint = if self.last_located < self.triangles.len()
            && !self.triangles[self.last_located].removed
        {
            Some(self.last_located)
        } else {
            self.triangles.iter().rposition(|t| !t.removed)
        };
        let Some(mut start) = hint else {
            return vec![None; seeds.len()];
        };
        let mut budget = self
            .triangles
            .len()
            .saturating_mul(2)
            .saturating_add(seeds.len().saturating_mul(16));
        seeds
            .iter()
            .map(|&seed| {
                let hit = self.walk_strictly_inside(start, seed, &mut budget);
                if let Some(ti) = hit {
                    start = ti;
                }
                hit
            })
            .collect()
    }

    /// Walk from `start` toward `p` as [`Cdt::locate_point`] does, but
    /// accept only a triangle with `p` strictly inside, never reset, and
    /// spend at most `budget` steps.
    fn walk_strictly_inside(&self, start: usize, p: Point2, budget: &mut usize) -> Option<usize> {
        if !(p.x().is_finite() && p.y().is_finite()) {
            return None;
        }
        let mut current = start;
        while *budget > 0 {
            *budget -= 1;
            #[cfg(test)]
            super::work::bump(&super::work::SEED_WALK_STEPS);
            let tri = &self.triangles[current];
            if tri.removed {
                return None;
            }

            let v0 = self.vertices[tri.v[0]];
            let v1 = self.vertices[tri.v[1]];
            let v2 = self.vertices[tri.v[2]];

            let d0 = orient2d(v1, v2, p);
            let d1 = orient2d(v2, v0, p);
            let d2 = orient2d(v0, v1, p);

            if d0 > 0.0 && d1 > 0.0 && d2 > 0.0 {
                return Some(current);
            }
            // On an edge or a vertex: crossing a zero edge could oscillate.
            if d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0 {
                return None;
            }

            // Cross the most negative edge.
            let min_d = d0.min(d1).min(d2);
            let edge_to_cross = if (d0 - min_d).abs() < 1e-30 {
                0
            } else if (d1 - min_d).abs() < 1e-30 {
                1
            } else {
                2
            };
            current = tri.adj[edge_to_cross]?;
        }
        None
    }

    /// Brute-force scan to find the triangle containing `p`.
    pub(super) fn locate_point_scan(&self, p: Point2) -> Result<(usize, PointLocation), MathError> {
        for (i, tri) in self.triangles.iter().enumerate() {
            if tri.removed {
                continue;
            }
            let v0 = self.vertices[tri.v[0]];
            let v1 = self.vertices[tri.v[1]];
            let v2 = self.vertices[tri.v[2]];

            let d0 = orient2d(v1, v2, p);
            let d1 = orient2d(v2, v0, p);
            let d2 = orient2d(v0, v1, p);

            if d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0 {
                if d0 == 0.0 {
                    return Ok((i, PointLocation::OnEdge(0)));
                }
                if d1 == 0.0 {
                    return Ok((i, PointLocation::OnEdge(1)));
                }
                if d2 == 0.0 {
                    return Ok((i, PointLocation::OnEdge(2)));
                }
                return Ok((i, PointLocation::Inside));
            }
        }

        Err(MathError::ConvergenceFailure { iterations: 0 })
    }
}
