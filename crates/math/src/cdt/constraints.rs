use std::collections::VecDeque;

use crate::MathError;
use crate::predicates::orient2d;

use super::{Cdt, segment_intersection_point, segments_properly_intersect, sorted_pair};

const MAX_SPLIT_DEPTH: usize = 16;

/// Outcome of [`Cdt::split_at_constrained_crossing`].
enum CrossingSplit {
    /// Both halves of the segment were recovered.
    Recovered,
    /// The crossing welded onto one of the segment's own endpoints. The
    /// crossed constraint was split there and no longer crosses.
    OntoEndpoint,
    /// No intersection point could be computed.
    NoIntersection,
}

/// Outcome of [`Cdt::edge_triangle_fan`].
enum FanLookup {
    /// This triangle contains the edge.
    Found(usize),
    /// The fan around the vertex has no such edge.
    Absent,
    /// The vertex→triangle hint is stale; the fan could not be walked.
    StaleHint,
}

/// Outcome of [`Cdt::next_crossing_edge`].
enum NextCrossing {
    /// This edge, `(triangle, local index of the opposite vertex)`,
    /// properly crosses the segment.
    Edge((usize, usize)),
    /// The segment is a triangulation edge.
    Recovered,
    /// The segment is no edge, yet no crossing edge was found.
    Unseen,
}

/// Outcome of [`Cdt::walk_toward`].
enum FanWalk {
    /// This edge, `(triangle, local index of the opposite vertex)`,
    /// properly crosses the segment.
    Crossing((usize, usize)),
    /// The walk reached a triangle holding both ends: the segment is an
    /// edge.
    Reached,
    /// The walk ended without finding either.
    Lost,
}

/// Outcome of [`Cdt::recover_edge_by_queue`].
enum QueueRecovery {
    /// The segment is a triangulation edge.
    Recovered,
    /// This constraint crosses the segment, so flips cannot recover it.
    ConstraintCrossing(usize, usize),
    /// Flipping cannot finish.
    Stuck,
}

impl Cdt {
    /// Recover a constraint edge (v0, v1) by flipping intersecting edges.
    ///
    /// Uses an iterative approach: find edges that cross the constraint
    /// segment and flip them until the constraint edge exists.
    #[allow(clippy::too_many_lines)]
    pub(super) fn recover_edge(&mut self, v0: usize, v1: usize) -> Result<(), MathError> {
        self.recover_edge_depth(v0, v1, 0)
    }

    /// [`Cdt::recover_edge`] with a Steiner-split depth budget.
    ///
    /// Flip recovery can stall without converging: a long constraint whose
    /// endpoints carry last-ULP coordinate noise (a 33.5 mm rail tilted by
    /// 1.8e-14 from boolean vertex welding) threads a corridor of
    /// exactly-degenerate quads that refuse every flip, and the loop spins to
    /// `max_iter` with the edge still missing. Returning Ok there poisons the
    /// caller: the constraint is recorded but no triangulation edge matches
    /// it, so `remove_exterior`'s flood pours through the gap and can erase
    /// an entire face (the mixed-socket z=5 floor tessellated to ZERO
    /// triangles this way). On non-convergence, split the constraint at its
    /// midpoint and recover both halves — each strictly shorter, so the
    /// degenerate corridor is bisected until every piece recovers. The
    /// sub-pairs are registered as constraints (the original pair never
    /// becomes an edge).
    ///
    /// Recovery runs in three stages. A cheap loop flips the first crossing
    /// edge it finds from either endpoint. If that loop stalls,
    /// [`Cdt::recover_edge_by_queue`] works through every crossing edge.
    /// Only if that also fails does the midpoint split above run.
    #[allow(clippy::too_many_lines)]
    fn recover_edge_depth(&mut self, v0: usize, v1: usize, depth: usize) -> Result<(), MathError> {
        if v0 == v1 {
            return Ok(());
        }
        let max_iter = self.triangles.len() * 4 + 100;

        for _ in 0..max_iter {
            let (ti, local) = match self.next_crossing_edge(v0, v1) {
                NextCrossing::Edge(found) => found,
                NextCrossing::Recovered => return Ok(()),
                // No crossing edge could be found although the segment is no
                // edge: near-degenerate geometry, where the walk and the
                // scan fail to see the crossing. Fall through to the Steiner
                // split rather than claiming success.
                NextCrossing::Unseen => break,
            };
            let Some(adj) = self.triangles[ti].adj[local] else {
                // Nothing changes before the next iteration, which
                // would find this same edge again.
                break;
            };

            let e0 = self.triangles[ti].v[(local + 1) % 3];
            let e1 = self.triangles[ti].v[(local + 2) % 3];

            // If the intersecting edge is constrained, split both edges
            // at their intersection point rather than giving up.
            if self.constraints.contains(&sorted_pair(e0, e1)) {
                match self.split_at_constrained_crossing(v0, v1, e0, e1)? {
                    CrossingSplit::Recovered => return Ok(()),
                    // The crossed constraint no longer properly
                    // crosses this segment. Retry the flip loop.
                    CrossingSplit::OntoEndpoint => continue,
                    CrossingSplit::NoIntersection => {}
                }
                // Intersection computation failed — give up gracefully.
                if std::env::var("BK_CDT").is_ok() {
                    log::debug!(
                        "CDT recover_edge: constrained-crossing give-up, edge {v0}->{v1} exists={}",
                        self.edge_exists(v0, v1)
                    );
                }
                return Ok(());
            }

            let opp_local = self.find_shared_edge_local(adj, e0, e1).unwrap_or(0);

            // Check that flipping is valid (the quad is convex).
            if self.is_convex_quad(ti, local, adj, opp_local) {
                self.flip_edge(ti, local, adj, opp_local);
            } else if !self.flip_other_intersecting_edge(v0, v1, e0, e1) {
                // Neither edge this loop inspects can flip: the first
                // crossing seen from v0 and the one seen from v1. The
                // loop is a pure function of the triangulation, so every
                // later iteration would repeat this one verbatim until
                // `max_iter`. Stop here and let the queue look at every
                // crossing edge instead.
                break;
            }
        }

        // The first-crossing loop stalled or ran out of budget. Flip the
        // whole corridor of crossing edges before resorting to a Steiner
        // point.
        match self.recover_edge_by_queue(v0, v1, max_iter) {
            QueueRecovery::Recovered => return Ok(()),
            QueueRecovery::Stuck => {}
            QueueRecovery::ConstraintCrossing(e0, e1) => {
                // The constraints genuinely cross, so they must meet at a
                // vertex: split both at the crossing, as the loop above
                // does when it meets one first, instead of bisecting
                // blindly toward it.
                match self.split_at_constrained_crossing(v0, v1, e0, e1)? {
                    CrossingSplit::Recovered => return Ok(()),
                    // One crossing constraint fewer: start over, within the
                    // same depth budget as the Steiner splits.
                    CrossingSplit::OntoEndpoint if depth < MAX_SPLIT_DEPTH => {
                        return self.recover_edge_depth(v0, v1, depth + 1);
                    }
                    CrossingSplit::OntoEndpoint | CrossingSplit::NoIntersection => {}
                }
            }
        }

        // Flip recovery did not converge. Bisect: insert the constraint's
        // midpoint and recover both (strictly shorter) halves.
        if depth >= MAX_SPLIT_DEPTH {
            return Err(MathError::ConvergenceFailure {
                iterations: max_iter,
            });
        }
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        let mid_pt =
            crate::vec::Point2::new(f64::midpoint(p0.x(), p1.x()), f64::midpoint(p0.y(), p1.y()));
        let mid = self.insert_point(mid_pt)?;
        if mid == v0 || mid == v1 {
            return Err(MathError::ConvergenceFailure {
                iterations: max_iter,
            });
        }
        self.recover_edge_depth(v0, mid, depth + 1)?;
        self.constraints.insert(sorted_pair(v0, mid));
        self.recover_edge_depth(mid, v1, depth + 1)?;
        self.constraints.insert(sorted_pair(mid, v1));
        Ok(())
    }

    /// What the first-crossing loop does next for the segment `(v0, v1)`.
    ///
    /// The loop used to ask [`Cdt::edge_exists`] first and only then look for
    /// a crossing edge. `edge_exists` walks the whole fan around `v0` when
    /// the edge is missing, which is every iteration but the last; around a
    /// vertex of high degree (a wall seam sampled only at its ends, beside a
    /// dense grid column) that made recovering one long constraint quadratic.
    /// The directed walk from `v0` toward `v1` is cheap, and when it finds an
    /// edge that properly crosses the open segment, `(v0, v1)` cannot be an
    /// edge: two edges of a triangulation never cross, and the crossing test
    /// is exact. Nor can one cross it when the walk reaches a triangle holding
    /// `v1`, which makes the segment an edge. So the walk's answer is taken
    /// first, and the fan walk and the linear scan run only when it finds
    /// neither, in the order the loop always ran them. The loop therefore
    /// flips the same edges as before.
    fn next_crossing_edge(&self, v0: usize, v1: usize) -> NextCrossing {
        #[cfg(test)]
        if super::work::legacy_recovery() {
            if self.edge_exists(v0, v1) {
                return NextCrossing::Recovered;
            }
            return self
                .find_intersecting_edge(v0, v1)
                .map_or(NextCrossing::Unseen, NextCrossing::Edge);
        }
        match self.walk_toward(v0, v1, None) {
            FanWalk::Crossing(found) => return NextCrossing::Edge(found),
            FanWalk::Reached => return NextCrossing::Recovered,
            FanWalk::Lost => {}
        }
        if self.edge_exists(v0, v1) {
            return NextCrossing::Recovered;
        }
        // The walk is a pure function of the triangulation and found
        // nothing; this is `find_intersecting_edge`'s scan fallback.
        self.scan_for_intersecting_edge(v0, v1)
            .map_or(NextCrossing::Unseen, NextCrossing::Edge)
    }

    /// Split the segment `(v0, v1)` and the constraint `(e0, e1)` that
    /// crosses it at their intersection, then recover both halves of the
    /// segment.
    fn split_at_constrained_crossing(
        &mut self,
        v0: usize,
        v1: usize,
        e0: usize,
        e1: usize,
    ) -> Result<CrossingSplit, MathError> {
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        let q0 = self.vertices[e0];
        let q1 = self.vertices[e1];
        let Some(mid_pt) = segment_intersection_point(p0, p1, q0, q1) else {
            return Ok(CrossingSplit::NoIntersection);
        };
        // `insert_point` welds onto an existing vertex when the
        // intersection lands within snap distance of one, so
        // `mid` can come back as any of the four endpoints.
        // Recursing with a degenerate pair (v0 == mid) spins
        // the flip loop and dead-ends in the bisect backstop
        // (its midpoint snaps straight back to the vertex), so
        // every recursion and constraint below is guarded.
        let mid = self.insert_point(mid_pt)?;
        if mid != e0 && mid != e1 {
            // Replace old constraint (e0,e1) with two sub-constraints.
            self.constraints.remove(&sorted_pair(e0, e1));
            self.constraints.insert(sorted_pair(e0, mid));
            self.constraints.insert(sorted_pair(mid, e1));
        }
        if mid == v0 || mid == v1 {
            // The crossing degenerated onto one of our own endpoints: the
            // crossed constraint (if any) was split there, so it no longer
            // properly crosses this segment.
            return Ok(CrossingSplit::OntoEndpoint);
        }
        // Recover the two halves of the original edge.
        self.recover_edge(v0, mid)?;
        self.constraints.insert(sorted_pair(v0, mid));
        self.recover_edge(mid, v1)?;
        self.constraints.insert(sorted_pair(mid, v1));
        Ok(CrossingSplit::Recovered)
    }

    /// Try one flip on a crossing edge other than `(e0, e1)`.
    ///
    /// This is the fast loop's second chance after the first crossing edge
    /// seen from `v0` turned out to have a non-convex quad. Returns whether
    /// an edge was flipped.
    fn flip_other_intersecting_edge(&mut self, v0: usize, v1: usize, e0: usize, e1: usize) -> bool {
        let Some((ti, local)) = self.find_other_intersecting_edge(v0, v1, e0, e1) else {
            return false;
        };
        let Some(adj) = self.triangles[ti].adj[local] else {
            return false;
        };
        let a = self.triangles[ti].v[(local + 1) % 3];
        let b = self.triangles[ti].v[(local + 2) % 3];
        if self.constraints.contains(&sorted_pair(a, b)) {
            return false;
        }
        let opp = self.find_shared_edge_local(adj, a, b).unwrap_or(0);
        if !self.is_convex_quad(ti, local, adj, opp) {
            return false;
        }
        self.flip_edge(ti, local, adj, opp);
        true
    }

    /// Sloan-style recovery of the edge `(v0, v1)` by flipping every edge
    /// that crosses it, in a queue.
    ///
    /// The first-crossing loop in [`Cdt::recover_edge_depth`] only ever
    /// inspects two crossing edges: the first seen from each endpoint. It
    /// stalls when both have non-convex quads while flippable edges sit
    /// further along the corridor. The U-bracket floor cap does that: of
    /// the crossing edges it keeps retrying, one has a reflex quad and the
    /// other's flip diagonal runs exactly through a collinear boundary
    /// vertex, while three other crossing edges are flippable.
    ///
    /// This follows Sloan (1993): take a crossing edge from the front of the
    /// queue. If its quad is strictly convex, flip it and requeue the new
    /// diagonal if that still crosses. Otherwise requeue the edge unchanged.
    /// Only strictly convex quads flip, so the triangulation stays valid.
    ///
    /// A constraint crossing the segment cannot be flipped away. The queue
    /// then changes nothing and reports the first such constraint along
    /// the segment. It reports [`QueueRecovery::Stuck`], leaving a valid,
    /// partly flipped triangulation, when a vertex lies exactly on the open
    /// segment, when a full pass over the queue flips nothing, or when
    /// `budget` runs out. The caller then falls back to its Steiner split.
    fn recover_edge_by_queue(&mut self, v0: usize, v1: usize, budget: usize) -> QueueRecovery {
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        let mut queue = match self.crossing_edges(v0, v1) {
            Ok(queue) => queue,
            Err((e0, e1)) => return QueueRecovery::ConstraintCrossing(e0, e1),
        };
        // Consecutive edges requeued without a flip. The triangulation is
        // unchanged across them, so once every queued edge has been
        // rejected in a row, no further pass can do anything else.
        let mut rejected = 0usize;
        for _ in 0..budget {
            let Some((a, b)) = queue.pop_front() else {
                return if self.edge_exists(v0, v1) {
                    QueueRecovery::Recovered
                } else {
                    QueueRecovery::Stuck
                };
            };
            let Some((ti, local)) = self.edge_triangle(a, b) else {
                return QueueRecovery::Stuck;
            };
            let Some(adj) = self.triangles[ti].adj[local] else {
                return QueueRecovery::Stuck;
            };
            let Some(opp) = self.find_shared_edge_local(adj, a, b) else {
                return QueueRecovery::Stuck;
            };
            if self.is_convex_quad(ti, local, adj, opp) {
                let c = self.triangles[ti].v[local];
                let d = self.triangles[adj].v[opp];
                self.flip_edge(ti, local, adj, opp);
                rejected = 0;
                if c != v0
                    && c != v1
                    && d != v0
                    && d != v1
                    && segments_properly_intersect(p0, p1, self.vertices[c], self.vertices[d])
                {
                    queue.push_back((c, d));
                }
            } else {
                queue.push_back((a, b));
                rejected += 1;
                if rejected >= queue.len() {
                    return QueueRecovery::Stuck;
                }
            }
        }
        QueueRecovery::Stuck
    }

    /// Every triangulation edge that properly crosses the open segment
    /// `(v0, v1)`, ordered along the segment from `v0`.
    ///
    /// If any of them is a constraint, flips cannot remove it: returns the
    /// first constrained one along the segment as the error.
    fn crossing_edges(
        &self,
        v0: usize,
        v1: usize,
    ) -> Result<VecDeque<(usize, usize)>, (usize, usize)> {
        #[cfg(test)]
        let corridor = if super::work::legacy_recovery() {
            None
        } else {
            self.corridor_crossings(v0, v1)
        };
        #[cfg(not(test))]
        let corridor = self.corridor_crossings(v0, v1);
        let mut found = match corridor {
            Some(found) => found,
            None => self.scan_crossings(v0, v1),
        };
        // hot: CDT constraint recovery inner loop (sort-heavy by construction on large triangulations).
        #[allow(clippy::disallowed_methods)]
        found.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        found.dedup_by(|x, y| x.1 == y.1);
        if let Some(&(_, edge)) = found.iter().find(|(_, e)| self.constraints.contains(e)) {
            return Err(edge);
        }
        Ok(found.into_iter().map(|(_, edge)| edge).collect())
    }

    /// `(t, edge)` for every live edge that properly crosses the open segment
    /// `(v0, v1)`, `t` its crossing parameter along the segment, found by
    /// testing every triangle (each interior edge is listed twice).
    fn scan_crossings(&self, v0: usize, v1: usize) -> Vec<(f64, (usize, usize))> {
        #[cfg(test)]
        super::work::bump(&super::work::CROSSING_SCANS);
        let mut found: Vec<(f64, (usize, usize))> = Vec::new();
        for tri in &self.triangles {
            if tri.removed {
                continue;
            }
            for local in 0..3 {
                let edge = sorted_pair(tri.v[(local + 1) % 3], tri.v[(local + 2) % 3]);
                if let Some(t) = self.crossing_parameter(v0, v1, edge) {
                    found.push((t, edge));
                }
            }
        }
        found
    }

    /// The parameter along `(v0, v1)` at which the edge `(a, b)` (sorted)
    /// properly crosses it, or `None` when it shares an endpoint with the
    /// segment or does not properly cross it.
    fn crossing_parameter(&self, v0: usize, v1: usize, (a, b): (usize, usize)) -> Option<f64> {
        if a == v0 || a == v1 || b == v0 || b == v1 {
            return None;
        }
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        let (pa, pb) = (self.vertices[a], self.vertices[b]);
        if !segments_properly_intersect(p0, p1, pa, pb) {
            return None;
        }
        // Crossing parameter along (v0, v1). Both orientations are nonzero
        // with opposite signs, so the denominator is too.
        let s0 = orient2d(pa, pb, p0);
        let s1 = orient2d(pa, pb, p1);
        Some(s0 / (s0 - s1))
    }

    /// [`Cdt::scan_crossings`]' edges, each once, found by walking the
    /// corridor of triangles the segment `(v0, v1)` passes through, or
    /// `None` when the walk cannot vouch for having found them all.
    ///
    /// With no vertex on the open segment, the segment leaves `v0` through
    /// one triangle of its fan, crosses the interior of one edge into the
    /// next triangle, and so on until it enters a triangle holding `v1`. The
    /// edges it crosses on the way are exactly the edges that properly cross
    /// it: such an edge meets the segment at an interior point of both,
    /// which lies on the two triangles beside the edge, so on the corridor.
    /// The walk tests each step with the same exact predicate as the scan,
    /// and gives up, leaving the scan to answer, on anything else: no
    /// crossing found from `v0`, a vertex exactly on the segment's line, an
    /// exit edge that is not unique, a missing or removed neighbour, or a
    /// walk longer than the triangulation. The caller sorts both answers the
    /// same way, so the result does not depend on which one produced it.
    fn corridor_crossings(&self, v0: usize, v1: usize) -> Option<Vec<(f64, (usize, usize))>> {
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        let (start, local) = self.walk_for_intersecting_edge(v0, v1, None)?;
        let tri = &self.triangles[start];
        let mut edge = (tri.v[(local + 1) % 3], tri.v[(local + 2) % 3]);
        let mut next = tri.adj[local];
        let mut found = Vec::new();
        for _ in 0..self.triangles.len() {
            let crossing = sorted_pair(edge.0, edge.1);
            found.push((self.crossing_parameter(v0, v1, crossing)?, crossing));
            let ti = next?;
            let tri = self.triangles.get(ti)?;
            if tri.removed {
                return None;
            }
            let apex_local = (0..3).find(|&i| tri.v[i] != edge.0 && tri.v[i] != edge.1)?;
            let apex = tri.v[apex_local];
            if !tri.v.contains(&edge.0) || !tri.v.contains(&edge.1) {
                return None;
            }
            if apex == v1 {
                return Some(found);
            }
            if apex == v0 || orient2d(p0, p1, self.vertices[apex]) == 0.0 {
                return None;
            }
            // The two edges through the apex: each is opposite one end of the
            // edge just crossed.
            let mut exits = [0, 1, 2]
                .into_iter()
                .filter(|&i| i != apex_local)
                .map(|i| (i, (tri.v[(i + 1) % 3], tri.v[(i + 2) % 3])))
                .filter(|&(_, (a, b))| {
                    self.crossing_parameter(v0, v1, sorted_pair(a, b)).is_some()
                });
            let (exit_local, exit) = exits.next()?;
            if exits.next().is_some() {
                return None;
            }
            edge = exit;
            next = tri.adj[exit_local];
        }
        None
    }

    /// A live triangle containing the edge `(a, b)`, with the local index
    /// of the vertex opposite that edge.
    fn edge_triangle(&self, a: usize, b: usize) -> Option<(usize, usize)> {
        let ti = match self.edge_triangle_fan(a, b) {
            FanLookup::Found(ti) => ti,
            FanLookup::Absent => return None,
            FanLookup::StaleHint => self
                .triangles
                .iter()
                .position(|t| !t.removed && t.v.contains(&a) && t.v.contains(&b))?,
        };
        let local = self.triangles[ti]
            .v
            .iter()
            .position(|&v| v != a && v != b)?;
        Some((ti, local))
    }

    /// Check if an edge between v0 and v1 exists in the triangulation.
    /// Uses the vertex→triangle hint to walk the fan around v0 in O(degree).
    fn edge_exists(&self, v0: usize, v1: usize) -> bool {
        // Try fast fan walk first using vertex_tri hint.
        if let Some(result) = self.edge_exists_fan(v0, v1) {
            return result;
        }
        // Fallback: linear scan (only if hint is stale).
        for tri in &self.triangles {
            if tri.removed {
                continue;
            }
            for i in 0..3 {
                let a = tri.v[i];
                let b = tri.v[(i + 1) % 3];
                if (a == v0 && b == v1) || (a == v1 && b == v0) {
                    return true;
                }
            }
        }
        false
    }

    /// Walk the triangle fan around vertex v0 checking for edge (v0, v1).
    /// Returns Some(bool) if successful, None if the hint is stale.
    fn edge_exists_fan(&self, v0: usize, v1: usize) -> Option<bool> {
        match self.edge_triangle_fan(v0, v1) {
            FanLookup::Found(_) => Some(true),
            FanLookup::Absent => Some(false),
            FanLookup::StaleHint => None,
        }
    }

    /// Walk the triangle fan around vertex v0 looking for edge (v0, v1).
    fn edge_triangle_fan(&self, v0: usize, v1: usize) -> FanLookup {
        if v0 >= self.vertex_tri.len() {
            return FanLookup::StaleHint;
        }
        let start = self.vertex_tri[v0];
        if start >= self.triangles.len() || self.triangles[start].removed {
            return FanLookup::StaleHint;
        }
        // Verify the hint triangle actually contains v0.
        let tri = &self.triangles[start];
        let Some(v0_local) = tri.v.iter().position(|&v| v == v0) else {
            return FanLookup::StaleHint;
        };

        // Walk around v0 in one direction, then the other.
        // Check each triangle for the edge (v0, v1).
        let check_tri = |tri: &super::CdtTriangle, v0_local: usize| -> bool {
            let a = tri.v[(v0_local + 1) % 3];
            let b = tri.v[(v0_local + 2) % 3];
            a == v1 || b == v1
        };

        if check_tri(tri, v0_local) {
            return FanLookup::Found(start);
        }

        // Walk clockwise (follow adj to the "left" of v0).
        let mut current = start;
        let mut cur_v0_local = v0_local;
        let max_steps = self.triangles.len();
        for _ in 0..max_steps {
            // In triangle (v0, a, b) with v0 at position v0_local:
            //   adj[v0_local] is across edge (a, b) — doesn't touch v0
            //   adj[(v0_local+1)%3] is across edge (b, v0) — touches v0
            //   adj[(v0_local+2)%3] is across edge (v0, a) — touches v0
            let next = self.triangles[current].adj[(cur_v0_local + 1) % 3];
            match next {
                Some(ni) if ni != start && !self.triangles[ni].removed => {
                    #[cfg(test)]
                    super::work::bump(&super::work::FAN_STEPS);
                    current = ni;
                    let t = &self.triangles[ni];
                    let Some(local) = t.v.iter().position(|&v| v == v0) else {
                        return FanLookup::StaleHint;
                    };
                    cur_v0_local = local;
                    if check_tri(t, cur_v0_local) {
                        return FanLookup::Found(current);
                    }
                }
                _ => break,
            }
        }

        // Walk counter-clockwise.
        current = start;
        cur_v0_local = v0_local;
        for _ in 0..max_steps {
            let next = self.triangles[current].adj[(cur_v0_local + 2) % 3];
            match next {
                Some(ni) if ni != start && !self.triangles[ni].removed => {
                    #[cfg(test)]
                    super::work::bump(&super::work::FAN_STEPS);
                    current = ni;
                    let t = &self.triangles[ni];
                    let Some(local) = t.v.iter().position(|&v| v == v0) else {
                        return FanLookup::StaleHint;
                    };
                    cur_v0_local = local;
                    if check_tri(t, cur_v0_local) {
                        return FanLookup::Found(current);
                    }
                }
                _ => break,
            }
        }

        FanLookup::Absent
    }

    /// Find a non-constrained edge that intersects segment (v0, v1).
    /// Walks from v0 toward v1 using triangle adjacency (O(k) where k =
    /// number of crossed edges) instead of scanning all triangles.
    #[cfg(test)]
    fn find_intersecting_edge(&self, v0: usize, v1: usize) -> Option<(usize, usize)> {
        // Try walking from v0 first (O(degree) amortized).
        if let Some(result) = self.walk_for_intersecting_edge(v0, v1, None) {
            return Some(result);
        }
        // Fallback: linear scan (only when walk fails).
        self.scan_for_intersecting_edge(v0, v1)
    }

    /// The first edge, in triangle order, that properly crosses the segment
    /// `(v0, v1)`: [`Cdt::find_intersecting_edge`]'s fallback when the walk
    /// from `v0` finds none.
    fn scan_for_intersecting_edge(&self, v0: usize, v1: usize) -> Option<(usize, usize)> {
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];

        for (ti, tri) in self.triangles.iter().enumerate() {
            if tri.removed {
                continue;
            }
            for local in 0..3 {
                let ea = tri.v[(local + 1) % 3];
                let eb = tri.v[(local + 2) % 3];

                if ea == v0 || ea == v1 || eb == v0 || eb == v1 {
                    continue;
                }

                let pa = self.vertices[ea];
                let pb = self.vertices[eb];

                if segments_properly_intersect(p0, p1, pa, pb) {
                    return Some((ti, local));
                }
            }
        }
        None
    }

    /// Walk the triangle fan around `v0` toward `v1`, returning the first
    /// intersecting edge. If `skip` is provided, edges matching that pair
    /// are ignored.
    fn walk_for_intersecting_edge(
        &self,
        v0: usize,
        v1: usize,
        skip: Option<(usize, usize)>,
    ) -> Option<(usize, usize)> {
        match self.walk_toward(v0, v1, skip) {
            FanWalk::Crossing(found) => Some(found),
            FanWalk::Reached | FanWalk::Lost => None,
        }
    }

    /// [`Cdt::walk_for_intersecting_edge`], also telling apart a walk that
    /// reached a triangle holding `v1`.
    ///
    /// Such a triangle holds both `v0` and `v1`, so `(v0, v1)` is an edge and
    /// no edge properly crosses the segment: the walk could never find one
    /// further on. It used to carry on regardless, bouncing between the two
    /// triangles beside the edge until it had taken as many steps as there
    /// are triangles; it now stops there.
    fn walk_toward(&self, v0: usize, v1: usize, skip: Option<(usize, usize)>) -> FanWalk {
        #[cfg(test)]
        let stop_at_v1 = !super::work::legacy_recovery();
        #[cfg(not(test))]
        let stop_at_v1 = true;
        if v0 >= self.vertex_tri.len() {
            return FanWalk::Lost;
        }
        let start = self.vertex_tri[v0];
        if start >= self.triangles.len() || self.triangles[start].removed {
            return FanWalk::Lost;
        }
        if !self.triangles[start].v.contains(&v0) {
            return FanWalk::Lost;
        }

        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];

        let mut current = start;
        let max_steps = self.triangles.len();
        for _ in 0..max_steps {
            let t = &self.triangles[current];
            if t.removed {
                break;
            }
            let v0_local = match t.v.iter().position(|&v| v == v0) {
                Some(l) => l,
                None => break,
            };
            let ea = t.v[(v0_local + 1) % 3];
            let eb = t.v[(v0_local + 2) % 3];
            if stop_at_v1 && (ea == v1 || eb == v1) {
                return FanWalk::Reached;
            }
            let should_skip = skip.is_some_and(|s| sorted_pair(ea, eb) == s);
            if ea != v1 && eb != v1 && !should_skip {
                let pa = self.vertices[ea];
                let pb = self.vertices[eb];
                if segments_properly_intersect(p0, p1, pa, pb) {
                    return FanWalk::Crossing((current, v0_local));
                }
            }
            // Walk in the direction that the target point lies.
            let va = self.vertices[ea];
            let side = orient2d(p0, p1, va);
            let next_adj = if side >= 0.0 {
                t.adj[(v0_local + 2) % 3]
            } else {
                t.adj[(v0_local + 1) % 3]
            };
            match next_adj {
                Some(ni) if ni != start && !self.triangles[ni].removed => {
                    #[cfg(test)]
                    super::work::bump(&super::work::WALK_STEPS);
                    current = ni;
                }
                _ => break,
            }
        }

        FanWalk::Lost
    }

    /// Find an intersecting edge different from (skip_e0, skip_e1).
    ///
    /// Tries multiple strategies: walk from v1 (reverse), walk from v0 with
    /// skip, then falls back to linear scan.
    fn find_other_intersecting_edge(
        &self,
        v0: usize,
        v1: usize,
        skip_e0: usize,
        skip_e1: usize,
    ) -> Option<(usize, usize)> {
        let skip = sorted_pair(skip_e0, skip_e1);

        // Strategy 1: Walk from v1 toward v0 (reverse direction).
        if let Some(result) = self.walk_for_intersecting_edge(v1, v0, None) {
            let tri = &self.triangles[result.0];
            let ea = tri.v[(result.1 + 1) % 3];
            let eb = tri.v[(result.1 + 2) % 3];
            if sorted_pair(ea, eb) != skip {
                return Some(result);
            }
        }

        // Strategy 2: Walk from v0 with skip.
        if let Some(result) = self.walk_for_intersecting_edge(v0, v1, Some(skip)) {
            return Some(result);
        }

        // Strategy 3: Linear scan fallback (rare).
        let p0 = self.vertices[v0];
        let p1 = self.vertices[v1];
        for (ti, tri) in self.triangles.iter().enumerate() {
            if tri.removed {
                continue;
            }
            for local in 0..3 {
                let ea = tri.v[(local + 1) % 3];
                let eb = tri.v[(local + 2) % 3];
                if ea == v0 || ea == v1 || eb == v0 || eb == v1 {
                    continue;
                }
                if sorted_pair(ea, eb) == skip {
                    continue;
                }
                let pa = self.vertices[ea];
                let pb = self.vertices[eb];
                if segments_properly_intersect(p0, p1, pa, pb) {
                    return Some((ti, local));
                }
            }
        }
        None
    }

    /// Check if the quadrilateral formed by two adjacent triangles is convex.
    fn is_convex_quad(&self, tri_a: usize, local_a: usize, tri_b: usize, local_b: usize) -> bool {
        let a_opp = self.vertices[self.triangles[tri_a].v[local_a]];
        let a_e0 = self.vertices[self.triangles[tri_a].v[(local_a + 1) % 3]];
        let a_e1 = self.vertices[self.triangles[tri_a].v[(local_a + 2) % 3]];
        let b_opp = self.vertices[self.triangles[tri_b].v[local_b]];

        // The quad is (a_opp, a_e0, b_opp, a_e1) — check that the new
        // diagonal (a_opp, b_opp) lies inside the quad.
        // This is equivalent to checking that a_e0 and a_e1 are on
        // opposite sides of (a_opp, b_opp).
        let d1 = orient2d(a_opp, b_opp, a_e0);
        let d2 = orient2d(a_opp, b_opp, a_e1);

        // They must be on strictly opposite sides.
        d1 * d2 < 0.0
    }
}
