//! 2D polygon offset via parallel edge translation and miter joins.
//!
//! Given a closed polygon in 2D, offsets each edge by a signed distance
//! (positive = outward, negative = inward) and computes new vertex
//! positions at the intersection of adjacent offset lines.
//!
//! An offset past the size of a feature collapses it: an edge whose two
//! miter points pass each other comes out reversed, and the miter loop then
//! crosses itself or turns inside out (a 4×2 rectangle inset by 1.5 gives a
//! clockwise 1×1 loop; inset by 2.5, a counter-clockwise loop of four
//! reversed edges). The offset region itself is the area that Clipper's raw
//! offset path winds around positively: each edge translated, joined by its
//! miter where adjacent offset edges diverge and by a detour through the
//! source vertex where they overlap. The miter loop is returned only when it
//! bounds exactly that region.

use crate::MathError;
use crate::compact_sort::CompactSort;
use crate::det_hash::DetHashMap;
use crate::predicates::orient2d;
use crate::vec::{Point2, Vec2};

/// Default miter limit: if a miter join extends further than this factor
/// times the offset distance, it is clamped to a bevel join.
const DEFAULT_MITER_LIMIT: f64 = 2.0;

/// Offset a closed 2D polygon by a signed distance.
///
/// Positive `distance` offsets outward, negative offsets inward, for either
/// winding. Applies a miter limit of 2× the offset distance to prevent
/// spikes at sharp corners.
///
/// The result keeps the input's winding. While no feature collapses, vertex
/// `i` of the result is the offset of input vertex `i`. When the offset
/// collapses part of the polygon, the collapsed edges are dropped and the
/// boundary of what remains is returned; when it collapses the whole
/// polygon, the result is empty. When what remains splits into several
/// pieces or encloses a hole, only the outer boundary of the largest piece
/// is returned.
///
/// # Algorithm
///
/// For each edge, compute a parallel offset line by translating along
/// the outward normal. Intersect adjacent offset lines to find new
/// vertex positions (miter join). Degenerate intersections (parallel
/// edges) fall back to simple translation. The miter loop is then checked
/// against the positively wound region of the raw offset path (see the
/// module docs), whose boundary replaces it when the two differ. Cost is
/// about O(n log n) for well-spread edges, rising toward O(n²) when many
/// edges crowd the same neighbourhood.
///
/// # Errors
///
/// Returns an error if fewer than 3 vertices are provided or the polygon's
/// area is below `tolerance`.
pub fn offset_polygon_2d(
    vertices: &[Point2],
    distance: f64,
    tolerance: f64,
) -> Result<Vec<Point2>, MathError> {
    let n = vertices.len();
    if n < 3 {
        return Err(MathError::EmptyInput);
    }

    // The outward-normal convention below assumes CCW winding. For a CW
    // loop the same perpendicular points inward, inverting the sign of
    // `distance`. Detect the actual winding via the signed area and flip
    // the effective distance so that negative always shrinks the loop.
    let area2 = signed_area_2x(vertices);
    if area2.abs() < tolerance {
        return Err(MathError::EmptyInput);
    }
    let distance = if area2 < 0.0 { -distance } else { distance };

    let (result, mut raw) = offset_paths(vertices, distance, tolerance);

    // Positive winding is measured on the counter-clockwise path.
    if area2 < 0.0 {
        raw.reverse();
    }
    let region = positive_winding_loops(&raw, tolerance);
    let Some(largest) = region
        .iter()
        .filter(|ring| signed_area_2x(ring) > 0.0)
        .max_by(|a, b| signed_area_2x(a).total_cmp(&signed_area_2x(b)))
    else {
        // The offset collapsed the whole polygon.
        return Ok(Vec::new());
    };

    // The raw path is the miter loop plus a closed kite (source vertex, its
    // two offset feet, the miter point) at every overlapping join. The kites
    // only remove area inward and only add it outward, so a simple miter loop
    // with the region's area bounds exactly that region.
    let region_area_2x: f64 = region.iter().map(|ring| signed_area_2x(ring)).sum();
    let miter_area_2x = signed_area_2x(&result) * area2.signum();
    let band = 2.0 * tolerance * perimeter(&result) + 1e-12 * miter_area_2x.abs();
    if (miter_area_2x - region_area_2x).abs() <= band && is_simple_loop(&result, tolerance) {
        return Ok(result);
    }

    let mut boundary = largest.clone();
    if area2 < 0.0 {
        boundary.reverse();
    }
    Ok(boundary)
}

/// The miter loop and Clipper's raw offset path, both in the input's
/// winding, for each edge moved `distance` along its right-hand normal.
///
/// A join where the offset edges diverge takes the miter vertex (or its
/// bevel) in both. Where they overlap, the raw path detours from the end of
/// one offset edge through the source vertex to the start of the next.
fn offset_paths(vertices: &[Point2], distance: f64, tolerance: f64) -> (Vec<Point2>, Vec<Point2>) {
    let n = vertices.len();
    let miter_limit_dist = distance.abs() * DEFAULT_MITER_LIMIT;

    // Compute offset edge lines: each edge shifts by distance along its outward normal.
    // For a CCW polygon, the outward normal of edge (A→B) is (dy, -dx) normalized.
    let mut result = Vec::with_capacity(n);
    let mut raw = Vec::with_capacity(3 * n);

    for i in 0..n {
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;

        // Previous edge: prev → i
        let (nx0, ny0) = edge_outward_normal(vertices[prev], vertices[i]);
        // Current edge: i → next
        let (nx1, ny1) = edge_outward_normal(vertices[i], vertices[next]);

        // Offset lines:
        // Line 0 passes through (vertices[prev] + d*n0) with direction (vertices[i] - vertices[prev])
        // Line 1 passes through (vertices[i] + d*n1) with direction (vertices[next] - vertices[i])
        let p0 = Point2::new(
            vertices[i].x() + distance * nx0,
            vertices[i].y() + distance * ny0,
        );
        let p1 = Point2::new(
            vertices[i].x() + distance * nx1,
            vertices[i].y() + distance * ny1,
        );

        let d0x = vertices[i].x() - vertices[prev].x();
        let d0y = vertices[i].y() - vertices[prev].y();
        let d1x = vertices[next].x() - vertices[i].x();
        let d1y = vertices[next].y() - vertices[i].y();

        // Intersect two lines: p0 + t*d0 = p1 + s*d1
        // Cross product for 2D line intersection
        let cross = d0x * d1y - d0y * d1x;

        if cross.abs() < tolerance {
            // Parallel edges — just translate the vertex
            let avg_nx = (nx0 + nx1) * 0.5;
            let avg_ny = (ny0 + ny1) * 0.5;
            result.push(Point2::new(
                vertices[i].x() + distance * avg_nx,
                vertices[i].y() + distance * avg_ny,
            ));
        } else {
            // Solve for t: (p1 - p0) × d1 / (d0 × d1)
            let dx = p1.x() - p0.x();
            let dy = p1.y() - p0.y();
            let t = (dx * d1y - dy * d1x) / cross;
            let miter_pt = Point2::new(p0.x() + t * d0x, p0.y() + t * d0y);

            // Miter limit: if the miter point is too far from the original
            // vertex, clamp to prevent spikes at near-parallel edges. Spikes
            // only form where the offset edges diverge (`cross` and `distance`
            // share a sign); where they overlap, the far miter point is their
            // true crossing and a bevel would pull the vertex back toward the
            // source corner.
            let miter_dist = ((miter_pt.x() - vertices[i].x()).powi(2)
                + (miter_pt.y() - vertices[i].y()).powi(2))
            .sqrt();

            if cross * distance > 0.0
                && miter_dist > miter_limit_dist
                && miter_limit_dist > tolerance
            {
                // Bevel: use the average of the two offset points.
                result.push(Point2::new(
                    (p0.x() + p1.x()) * 0.5,
                    (p0.y() + p1.y()) * 0.5,
                ));
            } else {
                result.push(miter_pt);
            }
        }

        let straight = cross.abs() < tolerance && d0x * d1x + d0y * d1y > 0.0;
        if straight || cross * distance > 0.0 {
            raw.push(result[i]);
        } else {
            // The offset edges overlap here (or fold back at a spike).
            raw.extend([p0, vertices[i], p1]);
        }
    }

    (result, raw)
}

/// Boundary of the region a closed `path` winds around positively (the
/// positive fill rule): outer loops counter-clockwise, holes clockwise.
///
/// The path is cut wherever it meets itself, points closer than `tol` are
/// merged, and each piece between cuts is kept where the winding number
/// steps between non-positive and positive across it, oriented with the
/// positive side on its left. The kept pieces are traced into loops.
fn positive_winding_loops(path: &[Point2], tol: f64) -> Vec<Vec<Point2>> {
    let len = path.len();
    let tol_sq = tol * tol;
    let mut nodes = Nodes {
        pos: path.to_vec(),
        parent: (0..len).collect(),
    };

    // Segment `k` runs from path node `k` to node `k + 1`; one shorter than
    // `tol` only merges its two ends.
    let mut live = Vec::with_capacity(len);
    for k in 0..len {
        if dist_sq(path[k], path[(k + 1) % len]) > tol_sq {
            live.push(k);
        } else {
            nodes.union(k, (k + 1) % len);
        }
    }
    if live.len() < 3 {
        return Vec::new();
    }
    let ends = |s: usize| (live[s], (live[s] + 1) % len);
    let segs: Vec<(Point2, Point2)> = live
        .iter()
        .map(|&k| (path[k], path[(k + 1) % len]))
        .collect();

    let mut cuts: Vec<Vec<(f64, usize)>> = (0..segs.len())
        .map(|s| {
            let (start, end) = ends(s);
            vec![(0.0, start), (1.0, end)]
        })
        .collect();
    for_each_box_pair(&segs, tol, |s, t| {
        let mut touched = false;
        for (on, other) in [(s, t), (t, s)] {
            let (a, b) = segs[on];
            let (start, end) = ends(on);
            let (other_start, other_end) = ends(other);
            for node in [other_start, other_end] {
                let p = path[node];
                if segment_dist_sq(p, a, b) > tol_sq {
                    continue;
                }
                touched = true;
                if dist_sq(p, a) <= tol_sq {
                    nodes.union(node, start);
                } else if dist_sq(p, b) <= tol_sq {
                    nodes.union(node, end);
                } else {
                    cuts[on].push((segment_param(p, a, b), node));
                }
            }
        }
        let ((a, b), (c, d)) = (segs[s], segs[t]);
        let (oa, ob) = (orient2d(c, d, a), orient2d(c, d, b));
        let (oc, od) = (orient2d(a, b, c), orient2d(a, b, d));
        if !touched && opposite(oa, ob) && opposite(oc, od) {
            let u = oa / (oa - ob);
            let crossing = nodes.push(lerp(a, b, u));
            cuts[s].push((u, crossing));
            cuts[t].push((oc / (oc - od), crossing));
        }
    });

    // Nodes that land within `tol` of each other along a segment are one node.
    for list in &mut cuts {
        list.compact_sort_by(|x, y| x.0.total_cmp(&y.0));
        for w in 1..list.len() {
            let (p, q) = (list[w - 1].1, list[w].1);
            let (pp, qp) = (nodes.at(p), nodes.at(q));
            if dist_sq(pp, qp) <= tol_sq {
                nodes.union(p, q);
            }
        }
    }

    // Pieces between consecutive distinct nodes, each with every segment that
    // runs along it and whether it runs the piece's way. Piece ends are root
    // nodes, so `nodes.pos` reads their positions directly.
    let mut pieces: Vec<Piece> = Vec::new();
    let mut by_ends: DetHashMap<(usize, usize), usize> = DetHashMap::default();
    for (s, list) in cuts.iter().enumerate() {
        let mut from = nodes.find(list[0].1);
        for &(_, node) in &list[1..] {
            let to = nodes.find(node);
            if to == from {
                continue;
            }
            let idx = *by_ends
                .entry((from.min(to), from.max(to)))
                .or_insert_with(|| {
                    pieces.push(Piece {
                        from,
                        to,
                        along: Vec::new(),
                    });
                    pieces.len() - 1
                });
            let forward = pieces[idx].from == from;
            pieces[idx].along.push((s, forward));
            from = to;
        }
    }

    let slabs = SlabIndex::new(&segs);
    let mut edges = Vec::new();
    for piece in &pieces {
        let (a, b) = (nodes.pos[piece.from], nodes.pos[piece.to]);
        let mid = Point2::new(f64::midpoint(a.x(), b.x()), f64::midpoint(a.y(), b.y()));
        // With the piece's own segments left out, the crossing number at
        // `mid` is the winding on the piece's +x side, or just above it when
        // the piece is horizontal (vertex heights resolve upward).
        let mut wind = 0;
        for &s in slabs.at(mid.y()) {
            if !piece.along.iter().any(|&(t, _)| t == s) {
                wind += crossing(segs[s], mid);
            }
        }
        let step: i32 = piece
            .along
            .iter()
            .map(|&(_, forward)| if forward { 1 } else { -1 })
            .sum();
        let (dx, dy) = (b.x() - a.x(), b.y() - a.y());
        let (left, right) = if dy > 0.0 || (dy == 0.0 && dx < 0.0) {
            (wind + step, wind)
        } else {
            (wind, wind - step)
        };
        match (left > 0, right > 0) {
            (true, false) => edges.push((piece.from, piece.to)),
            (false, true) => edges.push((piece.to, piece.from)),
            _ => {}
        }
    }

    let mut outgoing = vec![Vec::new(); nodes.pos.len()];
    for (e, &(from, _)) in edges.iter().enumerate() {
        outgoing[from].push(e);
    }
    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();
    for start in 0..edges.len() {
        if used[start] {
            continue;
        }
        let mut ring = Vec::new();
        let mut current = start;
        let closed = loop {
            used[current] = true;
            let (from, to) = edges[current];
            ring.push(nodes.pos[from]);
            let incoming = nodes.pos[to] - nodes.pos[from];
            // The leftmost turn keeps each loop around its own piece of the
            // region where two pieces touch at a node.
            let turn = |e: usize| turn_angle(incoming, nodes.pos[edges[e].1] - nodes.pos[to]);
            let next = outgoing[to]
                .iter()
                .copied()
                .filter(|&e| !used[e] || e == start)
                .max_by(|&e, &f| turn(e).total_cmp(&turn(f)));
            match next {
                Some(e) if e == start => break true,
                Some(e) => current = e,
                None => break false,
            }
        };
        if closed {
            let ring = drop_collinear(ring, tol);
            if ring.len() >= 3 && signed_area_2x(&ring).abs() > 2.0 * tol * perimeter(&ring) {
                loops.push(ring);
            }
        }
    }
    loops
}

/// A piece of the cut raw path: root node ids, and each segment running
/// along it with whether it runs from `from` to `to`.
struct Piece {
    from: usize,
    to: usize,
    along: Vec<(usize, bool)>,
}

/// Arrangement nodes; union-find merges nodes closer than the tolerance.
struct Nodes {
    pos: Vec<Point2>,
    parent: Vec<usize>,
}

impl Nodes {
    fn push(&mut self, p: Point2) -> usize {
        self.pos.push(p);
        self.parent.push(self.parent.len());
        self.parent.len() - 1
    }

    fn find(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }

    fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        self.parent[a.max(b)] = a.min(b);
    }

    fn at(&mut self, i: usize) -> Point2 {
        let root = self.find(i);
        self.pos[root]
    }
}

/// Segments bucketed by the horizontal slabs their y-ranges span, so a
/// crossing-number query visits only segments near its height.
struct SlabIndex {
    y0: f64,
    inv_height: f64,
    slabs: Vec<Vec<usize>>,
}

impl SlabIndex {
    #[allow(clippy::cast_precision_loss)]
    fn new(segs: &[(Point2, Point2)]) -> Self {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for &(a, b) in segs {
            lo = lo.min(a.y()).min(b.y());
            hi = hi.max(a.y()).max(b.y());
        }
        let count = segs.len().isqrt().max(1);
        let span = hi - lo;
        let mut index = Self {
            y0: lo,
            inv_height: if span > 0.0 { count as f64 / span } else { 0.0 },
            slabs: vec![Vec::new(); count],
        };
        for (s, &(a, b)) in segs.iter().enumerate() {
            for slab in index.slab(a.y().min(b.y()))..=index.slab(a.y().max(b.y())) {
                index.slabs[slab].push(s);
            }
        }
        index
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn slab(&self, y: f64) -> usize {
        // The float-to-int cast saturates, so heights below the first slab
        // (and NaN) land in slab 0.
        (((y - self.y0) * self.inv_height) as usize).min(self.slabs.len() - 1)
    }

    fn at(&self, y: f64) -> &[usize] {
        &self.slabs[self.slab(y)]
    }
}

/// Calls `visit(i, j)`, `i < j`, for each pair of segments whose bounding
/// boxes, grown by `pad`, overlap (sweep and prune along x).
fn for_each_box_pair(segs: &[(Point2, Point2)], pad: f64, mut visit: impl FnMut(usize, usize)) {
    let mut order: Vec<usize> = (0..segs.len()).collect();
    order.compact_sort_by(|&i, &j| {
        let (a, b) = (segs[i], segs[j]);
        a.0.x().min(a.1.x()).total_cmp(&b.0.x().min(b.1.x()))
    });
    for (rank, &i) in order.iter().enumerate() {
        let (a1, a2) = segs[i];
        let max_x = a1.x().max(a2.x()) + pad;
        let (lo_y, hi_y) = (a1.y().min(a2.y()) - pad, a1.y().max(a2.y()) + pad);
        for &j in &order[rank + 1..] {
            let (b1, b2) = segs[j];
            if b1.x().min(b2.x()) > max_x {
                break;
            }
            if b1.y().max(b2.y()) >= lo_y && b1.y().min(b2.y()) <= hi_y {
                visit(i.min(j), i.max(j));
            }
        }
    }
}

/// Whether a closed loop is simple: no edge shorter than `tol`, no two
/// non-adjacent edges within `tol` of each other, and no edge folding back
/// over its neighbour.
fn is_simple_loop(ring: &[Point2], tol: f64) -> bool {
    let n = ring.len();
    let tol_sq = tol * tol;
    let edges: Vec<(Point2, Point2)> = (0..n).map(|i| (ring[i], ring[(i + 1) % n])).collect();
    if edges.iter().any(|&(a, b)| dist_sq(a, b) <= tol_sq) {
        return false;
    }
    let mut simple = true;
    for_each_box_pair(&edges, tol, |i, j| {
        let ((a1, a2), (b1, b2)) = (edges[i], edges[j]);
        simple &= if j == i + 1 {
            segment_dist_sq(b2, a1, a2) > tol_sq && segment_dist_sq(a1, b1, b2) > tol_sq
        } else if i == 0 && j == n - 1 {
            segment_dist_sq(b1, a1, a2) > tol_sq && segment_dist_sq(a2, b1, b2) > tol_sq
        } else {
            !segments_meet(a1, a2, b1, b2, tol_sq)
        };
    });
    simple
}

/// Whether two segments cross or come within `sqrt(tol_sq)` of each other.
fn segments_meet(a1: Point2, a2: Point2, b1: Point2, b2: Point2, tol_sq: f64) -> bool {
    segment_dist_sq(b1, a1, a2) <= tol_sq
        || segment_dist_sq(b2, a1, a2) <= tol_sq
        || segment_dist_sq(a1, b1, b2) <= tol_sq
        || segment_dist_sq(a2, b1, b2) <= tol_sq
        || (opposite(orient2d(a1, a2, b1), orient2d(a1, a2, b2))
            && opposite(orient2d(b1, b2, a1), orient2d(b1, b2, a2)))
}

fn opposite(a: f64, b: f64) -> bool {
    (a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0)
}

/// Signed crossing of the ray from `p` toward +x by the segment `(a, b)`
/// (Sunday's winding-number rule, half-open in y).
fn crossing((a, b): (Point2, Point2), p: Point2) -> i32 {
    if a.y() <= p.y() {
        i32::from(b.y() > p.y() && orient2d(a, b, p) > 0.0)
    } else if b.y() <= p.y() && orient2d(a, b, p) < 0.0 {
        -1
    } else {
        0
    }
}

/// Signed angle from `incoming` to `outgoing`, counter-clockwise positive.
fn turn_angle(incoming: Vec2, outgoing: Vec2) -> f64 {
    let cross = incoming.x() * outgoing.y() - incoming.y() * outgoing.x();
    let dot = incoming.x() * outgoing.x() + incoming.y() * outgoing.y();
    cross.atan2(dot)
}

/// Drops vertices lying within `tol` of the segment joining their
/// neighbours, such as cut nodes left on a straight run.
fn drop_collinear(mut ring: Vec<Point2>, tol: f64) -> Vec<Point2> {
    let tol_sq = tol * tol;
    let mut i = 0;
    while ring.len() >= 3 && i < ring.len() {
        let n = ring.len();
        if segment_dist_sq(ring[i], ring[(i + n - 1) % n], ring[(i + 1) % n]) <= tol_sq {
            ring.remove(i);
            i = i.saturating_sub(1);
        } else {
            i += 1;
        }
    }
    ring
}

fn perimeter(ring: &[Point2]) -> f64 {
    let n = ring.len();
    (0..n)
        .map(|i| dist_sq(ring[i], ring[(i + 1) % n]).sqrt())
        .sum()
}

fn dist_sq(a: Point2, b: Point2) -> f64 {
    let (dx, dy) = (b.x() - a.x(), b.y() - a.y());
    dx * dx + dy * dy
}

/// Squared distance from `p` to the segment `[a, b]`.
fn segment_dist_sq(p: Point2, a: Point2, b: Point2) -> f64 {
    dist_sq(p, lerp(a, b, segment_param(p, a, b).clamp(0.0, 1.0)))
}

/// Parameter of the projection of `p` onto the line through `a` and `b`.
fn segment_param(p: Point2, a: Point2, b: Point2) -> f64 {
    let (dx, dy) = (b.x() - a.x(), b.y() - a.y());
    let len_sq = dx * dx + dy * dy;
    if len_sq > 0.0 {
        ((p.x() - a.x()) * dx + (p.y() - a.y()) * dy) / len_sq
    } else {
        0.0
    }
}

fn lerp(a: Point2, b: Point2, t: f64) -> Point2 {
    Point2::new(a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y()))
}

/// Twice the signed area of a closed 2D polygon (shoelace formula).
///
/// Positive for CCW winding, negative for CW.
fn signed_area_2x(vertices: &[Point2]) -> f64 {
    let n = vertices.len();
    let mut acc = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        acc += vertices[i].x() * vertices[j].y() - vertices[j].x() * vertices[i].y();
    }
    acc
}

/// Compute the outward normal of a 2D edge (assuming CCW winding).
///
/// For edge A→B, the outward normal is the left-perpendicular of (B-A),
/// normalized to unit length. Returns `(0, 0)` for degenerate edges.
fn edge_outward_normal(a: Point2, b: Point2) -> (f64, f64) {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-15 {
        return (0.0, 0.0);
    }
    // Left perpendicular = (dy, -dx) / len
    (dy / len, -dx / len)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use proptest::prelude::*;
    use std::f64::consts::{PI, TAU};

    fn pts(coords: &[(f64, f64)]) -> Vec<Point2> {
        coords.iter().map(|&(x, y)| Point2::new(x, y)).collect()
    }

    fn reversed(poly: &[Point2]) -> Vec<Point2> {
        poly.iter().rev().copied().collect()
    }

    fn rect(w: f64, h: f64) -> Vec<Point2> {
        pts(&[(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)])
    }

    /// Both windings of `poly`, for tests whose answer must not depend on it.
    fn both_windings(poly: &[Point2]) -> [Vec<Point2>; 2] {
        [poly.to_vec(), reversed(poly)]
    }

    fn assert_area(poly: &[Point2], source: &[Point2], expected: f64) {
        let area = signed_area(poly);
        assert!(
            (area.abs() - expected).abs() < 1e-9 * expected.max(1.0),
            "area {area}, expected {expected}: {poly:?}"
        );
        assert!(
            (area > 0.0) == (signed_area(source) > 0.0),
            "the result must keep the input's winding"
        );
    }

    #[test]
    fn offset_square_outward() {
        let square = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];
        let result = offset_polygon_2d(&square, 0.5, 1e-10).unwrap();
        assert_eq!(result.len(), 4);
        // Outward offset of unit square by 0.5 → vertices at (-0.5, -0.5), (1.5, -0.5), etc.
        assert!((result[0].x() - (-0.5)).abs() < 1e-10);
        assert!((result[0].y() - (-0.5)).abs() < 1e-10);
        assert!((result[1].x() - 1.5).abs() < 1e-10);
        assert!((result[1].y() - (-0.5)).abs() < 1e-10);
        assert!((result[2].x() - 1.5).abs() < 1e-10);
        assert!((result[2].y() - 1.5).abs() < 1e-10);
        assert!((result[3].x() - (-0.5)).abs() < 1e-10);
        assert!((result[3].y() - 1.5).abs() < 1e-10);
    }

    #[test]
    fn offset_square_inward() {
        let square = vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 2.0),
            Point2::new(0.0, 2.0),
        ];
        let result = offset_polygon_2d(&square, -0.5, 1e-10).unwrap();
        assert_eq!(result.len(), 4);
        assert!((result[0].x() - 0.5).abs() < 1e-10);
        assert!((result[0].y() - 0.5).abs() < 1e-10);
        assert!((result[1].x() - 1.5).abs() < 1e-10);
        assert!((result[1].y() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn offset_triangle() {
        let tri = vec![
            Point2::new(0.0, 0.0),
            Point2::new(4.0, 0.0),
            Point2::new(2.0, 3.0),
        ];
        let result = offset_polygon_2d(&tri, 0.1, 1e-10).unwrap();
        assert_eq!(result.len(), 3);
        // All vertices should be further from the centroid
        let cx = (tri[0].x() + tri[1].x() + tri[2].x()) / 3.0;
        let cy = (tri[0].y() + tri[1].y() + tri[2].y()) / 3.0;
        for (orig, off) in tri.iter().zip(result.iter()) {
            let d_orig = ((orig.x() - cx).powi(2) + (orig.y() - cy).powi(2)).sqrt();
            let d_off = ((off.x() - cx).powi(2) + (off.y() - cy).powi(2)).sqrt();
            assert!(
                d_off > d_orig,
                "offset vertex should be farther from centroid"
            );
        }
    }

    #[test]
    fn too_few_vertices() {
        let pts = vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)];
        assert!(offset_polygon_2d(&pts, 0.1, 1e-10).is_err());
    }

    fn signed_area(poly: &[Point2]) -> f64 {
        let n = poly.len();
        let mut acc = 0.0;
        for i in 0..n {
            let j = (i + 1) % n;
            acc += poly[i].x() * poly[j].y() - poly[j].x() * poly[i].y();
        }
        acc * 0.5
    }

    /// The inset of a triangle is the triangle scaled about its incenter by
    /// `(r - d) / r`, `r` the inradius. The 28° apex miter lies 2.06 below the
    /// apex, beyond the 2·d miter limit; beveling it there left the apex
    /// 0.12 below the source apex and the area at 84.48 instead of 76.02.
    #[test]
    fn inset_sharp_triangle_keeps_far_miter() {
        let tri = vec![
            Point2::new(0.0, 0.0),
            Point2::new(10.0, 0.0),
            Point2::new(5.0, 20.0),
        ];
        let d = 0.5;
        let inradius = 100.0 / (5.0 + 425.0_f64.sqrt());
        let scale = (inradius - d) / inradius;
        for poly in [tri.clone(), tri.iter().rev().copied().collect()] {
            let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
            assert_eq!(inset.len(), 3);
            let area = signed_area(&inset).abs();
            assert!(
                (area - 100.0 * scale * scale).abs() < 1e-9,
                "inset area {area}, expected {}",
                100.0 * scale * scale
            );
            let apex_y = inset.iter().map(|p| p.y()).fold(f64::MIN, f64::max);
            let expected_apex_y = inradius + scale * (20.0 - inradius);
            assert!((apex_y - expected_apex_y).abs() < 1e-9, "apex at {apex_y}");
        }
    }

    /// MATH-6: a 4×2 rectangle inset by more than half its height came back
    /// inside out — by 1.5, a clockwise loop of area −1; by 2.5, a
    /// counter-clockwise loop of area 3 built from four reversed edges.
    #[test]
    fn rectangle_inset_past_half_height_vanishes() {
        for poly in both_windings(&rect(4.0, 2.0)) {
            for d in [1.0, 1.1, 1.5, 2.0, 2.5, 3.0, 10.0] {
                let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
                assert!(inset.is_empty(), "inset by {d} left {inset:?}");
            }
        }
    }

    /// Offsets that collapse nothing keep their closed-form area,
    /// `(4 + 2d)(2 + 2d)`, and vertex `i` stays the offset of vertex `i`.
    #[test]
    fn rectangle_offsets_keep_area_and_vertex_order() {
        for poly in both_windings(&rect(4.0, 2.0)) {
            for d in [-0.9, -0.5, -0.25, 0.0, 0.5, 1.5, 10.0] {
                let out = offset_polygon_2d(&poly, d, 1e-10).unwrap();
                assert_eq!(out.len(), 4, "offset by {d}: {out:?}");
                assert_area(&out, &poly, (4.0 + 2.0 * d) * (2.0 + 2.0 * d));
                for (src, off) in poly.iter().zip(&out) {
                    assert!(((off.x() - src.x()).abs() - d.abs()).abs() < 1e-12);
                    assert!(((off.y() - src.y()).abs() - d.abs()).abs() < 1e-12);
                }
            }
        }
    }

    /// An inset past a short edge's collapse distance drops that edge. The
    /// 10×10 square with a 1×1 chamfer inset by 2 is [2, 8]²: the chamfer's
    /// offset line x + y = 19 − 2√2 passes outside the corner (8, 8).
    #[test]
    fn inset_drops_a_collapsed_chamfer() {
        let poly = pts(&[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 9.0),
            (9.0, 10.0),
            (0.0, 10.0),
        ]);
        for poly in both_windings(&poly) {
            let inset = offset_polygon_2d(&poly, -2.0, 1e-10).unwrap();
            assert_eq!(inset.len(), 4, "{inset:?}");
            assert_area(&inset, &poly, 36.0);
            for p in &inset {
                assert!((p.x() - 2.0).abs() < 1e-9 || (p.x() - 8.0).abs() < 1e-9);
                assert!((p.y() - 2.0).abs() < 1e-9 || (p.y() - 8.0).abs() < 1e-9);
            }
        }
    }

    /// A regular n-gon inset by d is the regular n-gon of apothem a − d,
    /// area n (a − d)² tan(π/n); past its apothem it vanishes.
    #[test]
    fn regular_polygon_inset_shrinks_then_vanishes() {
        let n = 32;
        let poly: Vec<Point2> = (0..n)
            .map(|k| {
                let t = TAU * f64::from(k) / f64::from(n);
                Point2::new(5.0 * t.cos(), 5.0 * t.sin())
            })
            .collect();
        let apothem = 5.0 * (PI / f64::from(n)).cos();
        for poly in both_windings(&poly) {
            for d in [1.0, 2.0, apothem - 0.01] {
                let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
                assert_eq!(inset.len(), 32);
                let a = apothem - d;
                assert_area(
                    &inset,
                    &poly,
                    f64::from(n) * a * a * (PI / f64::from(n)).tan(),
                );
            }
            for d in [apothem + 0.01, 5.0, 12.0] {
                let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
                assert!(inset.is_empty(), "inset by {d} left {inset:?}");
            }
        }
    }

    /// Mitered offsets of a rectilinear polygon are its erosion by an
    /// axis-aligned square. The L with arms 1 wide inset by 0.25 keeps arms
    /// 0.5 wide (area 3.5·0.5·2 − 0.5² = 3.25) and loses them from 0.5 on.
    #[test]
    fn l_shape_inset_thins_then_vanishes() {
        let poly = pts(&[
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 1.0),
            (1.0, 1.0),
            (1.0, 4.0),
            (0.0, 4.0),
        ]);
        for poly in both_windings(&poly) {
            let inset = offset_polygon_2d(&poly, -0.25, 1e-10).unwrap();
            assert_eq!(inset.len(), 6);
            assert_area(&inset, &poly, 3.25);
            for d in [0.5, 0.6, 1.0, 3.0] {
                let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
                assert!(inset.is_empty(), "inset by {d} left {inset:?}");
            }
        }
    }

    /// An inset past half a bridge's width splits the polygon, and the larger
    /// piece comes back. A 6×6 and a 4×4 square joined by a 1-wide bridge,
    /// inset by 1, leave [1, 5]² and [9, 11]×[2, 4].
    #[test]
    fn inset_that_splits_returns_the_larger_piece() {
        let poly = pts(&[
            (0.0, 0.0),
            (6.0, 0.0),
            (6.0, 2.5),
            (8.0, 2.5),
            (8.0, 1.0),
            (12.0, 1.0),
            (12.0, 5.0),
            (8.0, 5.0),
            (8.0, 3.5),
            (6.0, 3.5),
            (6.0, 6.0),
            (0.0, 6.0),
        ]);
        for poly in both_windings(&poly) {
            let inset = offset_polygon_2d(&poly, -1.0, 1e-10).unwrap();
            assert_eq!(inset.len(), 4, "{inset:?}");
            assert_area(&inset, &poly, 16.0);
            for p in &inset {
                assert!((1.0 - 1e-9..=5.0 + 1e-9).contains(&p.x()), "{p:?}");
                assert!((1.0 - 1e-9..=5.0 + 1e-9).contains(&p.y()), "{p:?}");
            }
        }
    }

    /// The single-loop result's documented limit: an outset that closes a
    /// C-shape's 2-wide mouth encloses the hole [3.5, 6.5]², and only the
    /// outer boundary [−1.5, 11.5]² comes back.
    #[test]
    fn outset_closing_a_mouth_returns_the_outer_boundary() {
        let poly = pts(&[
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (6.0, 10.0),
            (6.0, 8.0),
            (8.0, 8.0),
            (8.0, 2.0),
            (2.0, 2.0),
            (2.0, 8.0),
            (4.0, 8.0),
            (4.0, 10.0),
            (0.0, 10.0),
        ]);
        for poly in both_windings(&poly) {
            let outset = offset_polygon_2d(&poly, 1.5, 1e-10).unwrap();
            assert_eq!(outset.len(), 4, "{outset:?}");
            assert_area(&outset, &poly, 169.0);
        }
    }

    /// The convex `poly` clipped to the half-plane `n · p <= c`.
    fn clip_half_plane(poly: &[Point2], (nx, ny): (f64, f64), c: f64) -> Vec<Point2> {
        let side = |p: Point2| nx * p.x() + ny * p.y() - c;
        let mut out = Vec::new();
        for i in 0..poly.len() {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            let (sp, sq) = (side(p), side(q));
            if sp <= 0.0 {
                out.push(p);
            }
            if (sp < 0.0 && sq > 0.0) || (sp > 0.0 && sq < 0.0) {
                let t = sp / (sp - sq);
                out.push(Point2::new(
                    p.x() + t * (q.x() - p.x()),
                    p.y() + t * (q.y() - p.y()),
                ));
            }
        }
        out
    }

    /// Star-shaped polygon from sorted angles and per-vertex radii.
    fn star(mut spokes: Vec<(f64, f64)>) -> Vec<Point2> {
        spokes.sort_by(|a, b| a.0.total_cmp(&b.0));
        spokes.dedup_by(|a, b| a.0 - b.0 < 0.05);
        spokes
            .iter()
            .map(|&(t, r)| Point2::new(r * t.cos(), r * t.sin()))
            .collect()
    }

    fn boundary_dist(p: Point2, poly: &[Point2]) -> f64 {
        (0..poly.len())
            .map(|i| segment_dist_sq(p, poly[i], poly[(i + 1) % poly.len()]))
            .fold(f64::INFINITY, f64::min)
            .sqrt()
    }

    /// Largest turning angle at a reflex (clockwise-turning) vertex of a
    /// counter-clockwise polygon. Inward miters there exceed the 2·d limit and
    /// bevel past 2π/3.
    fn max_reflex_turn(poly: &[Point2]) -> f64 {
        let n = poly.len();
        (0..n)
            .map(|i| {
                let d0 = poly[i] - poly[(i + n - 1) % n];
                let d1 = poly[(i + 1) % n] - poly[i];
                let cross = d0.x() * d1.y() - d0.y() * d1.x();
                if cross < 0.0 {
                    turn_angle(d0, d1).abs()
                } else {
                    0.0
                }
            })
            .fold(0.0, f64::max)
    }

    /// Sum of the winding numbers of `loops` at `p`: 1 inside the region they
    /// bound (outer loops counter-clockwise, holes clockwise), 0 outside.
    fn region_winding(loops: &[Vec<Point2>], p: Point2) -> i32 {
        loops
            .iter()
            .map(|ring| crate::predicates::winding_number(p, ring))
            .sum()
    }

    fn near_any_edge(p: Point2, ring: &[Point2], tol: f64) -> bool {
        (0..ring.len())
            .any(|i| segment_dist_sq(p, ring[i], ring[(i + 1) % ring.len()]) <= tol * tol)
    }

    proptest! {
        /// `positive_winding_loops` bounds exactly the points the raw offset
        /// path winds around positively, checked on a grid against the winding
        /// number of the path itself. Covers outward and inward offsets,
        /// beveled joins, splits and holes.
        #[test]
        fn traced_region_matches_raw_path_winding(
            spokes in proptest::collection::vec((0.0..TAU, 1.0..10.0_f64), 3..20),
            d in -6.0..6.0_f64,
        ) {
            let poly = star(spokes);
            prop_assume!(poly.len() >= 3);
            let (_, raw) = offset_paths(&poly, d, 1e-10);
            let loops = positive_winding_loops(&raw, 1e-10);
            let reach = 10.0 + 2.0 * DEFAULT_MITER_LIMIT * d.abs();
            for i in 0..=40 {
                for j in 0..=40 {
                    let p = Point2::new(
                        reach * (f64::from(i) / 20.0 - 1.0) + 1e-3,
                        reach * (f64::from(j) / 20.0 - 1.0) + 2e-3,
                    );
                    if near_any_edge(p, &raw, 1e-6) {
                        continue;
                    }
                    let expected = i32::from(crate::predicates::winding_number(p, &raw) >= 1);
                    prop_assert_eq!(region_winding(&loops, p), expected, "at {:?}", p);
                }
            }
        }

        /// The inset of a convex polygon is the intersection of its edges'
        /// half-planes moved in by `d`, computed here by direct clipping.
        #[test]
        fn convex_inset_matches_half_plane_intersection(
            angles in proptest::collection::vec(0.0..TAU, 3..14),
            radius in 0.5..20.0_f64,
            depth in 0.0..1.2_f64,
        ) {
            let poly = star(angles.iter().map(|&t| (t, radius)).collect());
            prop_assume!(poly.len() >= 3 && signed_area(&poly) > 1e-3 * radius * radius);
            let d = depth * radius;
            let mut oracle = poly.clone();
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                let (nx, ny) = edge_outward_normal(a, b);
                oracle = clip_half_plane(&oracle, (nx, ny), nx * a.x() + ny * a.y() - d);
            }
            let expected = if oracle.len() >= 3 { signed_area(&oracle) } else { 0.0 };
            for poly in both_windings(&poly) {
                let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
                let got = if inset.is_empty() { 0.0 } else { signed_area(&inset).abs() };
                prop_assert!(
                    (got - expected).abs() <= 1e-9 * radius * radius,
                    "inset by {} has area {}, half-planes give {}", d, got, expected
                );
            }
        }

        /// Whatever collapses, an inset is a simple loop in the input's winding
        /// whose vertices lie inside the polygon at least `d` from its boundary:
        /// a mitered inset lies within the erosion by a disk of radius `d`.
        /// (A beveled reflex join cuts closer, so polygons that bevel are
        /// left to the next property.)
        #[test]
        fn star_inset_stays_simple_and_inside(
            spokes in proptest::collection::vec((0.0..0.5_f64, 6.0..10.0_f64), 3..12),
            d in 0.0..8.0_f64,
        ) {
            // Evenly spaced, jittered spokes over a 6–10 radius band rarely
            // turn a reflex corner past the bevel threshold.
            let count = spokes.len() as f64;
            let poly = star(
                spokes
                    .iter()
                    .enumerate()
                    .map(|(k, &(jitter, r))| (TAU * (k as f64 + jitter) / count, r))
                    .collect(),
            );
            prop_assume!(poly.len() >= 3 && max_reflex_turn(&poly) <= 2.0 * PI / 3.0);
            for poly in both_windings(&poly) {
                let inset = offset_polygon_2d(&poly, -d, 1e-10).unwrap();
                if inset.is_empty() {
                    continue;
                }
                prop_assert!(is_simple_loop(&inset, 1e-10), "{:?}", inset);
                prop_assert_eq!(signed_area(&inset) > 0.0, signed_area(&poly) > 0.0);
                for &p in &inset {
                    let dist = boundary_dist(p, &poly);
                    prop_assert!(
                        dist <= 1e-9 || crate::predicates::point_in_polygon(p, &poly),
                        "{:?} outside", p
                    );
                    prop_assert!(dist >= d - 1e-9, "{:?} within {} of the boundary", p, d);
                }
            }
        }
    }

    #[test]
    fn offset_cw_square_inward() {
        // Clockwise-wound 20x20 square. A negative distance must shrink it
        // (area 256), regardless of winding.
        let cw_square = vec![
            Point2::new(20.0, 0.0),
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 20.0),
            Point2::new(20.0, 20.0),
        ];
        let inward = offset_polygon_2d(&cw_square, -2.0, 1e-10).unwrap();
        assert_eq!(inward.len(), 4);
        assert!(
            (signed_area(&inward).abs() - 256.0).abs() < 1e-6,
            "CW square offset -2 should enclose area 256, got {}",
            signed_area(&inward).abs()
        );

        let outward = offset_polygon_2d(&cw_square, 2.0, 1e-10).unwrap();
        assert_eq!(outward.len(), 4);
        assert!(
            (signed_area(&outward).abs() - 576.0).abs() < 1e-6,
            "CW square offset +2 should enclose area 576, got {}",
            signed_area(&outward).abs()
        );
    }
}
