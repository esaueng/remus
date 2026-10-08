//! Batch hole flooding removes exactly the triangles, in the same state, that
//! the one-seed flood removes called seed by seed.
//!
//! The reference below is `flood_remove_from_point` before the batch API,
//! verbatim: it rebuilds the barrier `constraints ∪ self.constraints` per
//! seed and locates each seed after the earlier floods.

use super::*;
use crate::cdt::work;

/// `flood_remove_from_point` before the batch API, verbatim.
fn reference_flood(cdt: &mut Cdt, seed: Point2, constraints: &DetHashSet<(usize, usize)>) -> bool {
    let barrier: DetHashSet<(usize, usize)> =
        constraints.union(&cdt.constraints).copied().collect();
    let constraints = &barrier;
    let seed_tri = cdt.locate_point(seed).ok().map(|(i, _)| i).or_else(|| {
        cdt.triangles
            .iter()
            .enumerate()
            .filter(|(_, t)| !t.removed)
            .find(|(_, t)| {
                let p0 = cdt.vertices[t.v[0]];
                let p1 = cdt.vertices[t.v[1]];
                let p2 = cdt.vertices[t.v[2]];
                let d0 = orient2d(p0, p1, seed);
                let d1 = orient2d(p1, p2, seed);
                let d2 = orient2d(p2, p0, seed);
                (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0 && d2 <= 0.0)
            })
            .map(|(i, _)| i)
    });

    let Some(start) = seed_tri else {
        return false;
    };

    let mut stack = vec![start];
    while let Some(ti) = stack.pop() {
        if cdt.triangles[ti].removed {
            continue;
        }
        cdt.triangles[ti].removed = true;

        for local in 0..3 {
            let va = cdt.triangles[ti].v[(local + 1) % 3];
            let vb = cdt.triangles[ti].v[(local + 2) % 3];
            let edge_key = sorted_pair(va, vb);
            if constraints.contains(&edge_key) {
                continue;
            }
            if let Some(adj) = cdt.triangles[ti].adj[local]
                && !cdt.triangles[adj].removed
            {
                stack.push(adj);
            }
        }
    }

    true
}

/// Triangulate the wires the way `run_planar_cdt` does up to its hole
/// floods: Hilbert-ordered points, every wire edge as a constraint, the
/// exterior flood from the first wire, and the caller's barrier holding both
/// orientations of every constraint.
fn holed(pts: &[Point2], wires: &[(usize, usize)]) -> (Cdt, DetHashSet<(usize, usize)>) {
    let mut cdt = Cdt::with_capacity(planar_bounds(pts), pts.len());
    let ids = cdt.insert_points_hilbert(pts).unwrap();
    let mut all = Vec::new();
    for &(start, end) in wires {
        let n = end - start;
        for i in 0..n {
            let (a, b) = (ids[start + i], ids[start + (i + 1) % n]);
            if a != b {
                cdt.insert_constraint(a, b).unwrap();
                all.push((a, b));
            }
        }
    }
    let outer_n = wires[0].1 - wires[0].0;
    let outer: Vec<_> = (0..outer_n)
        .map(|i| (ids[wires[0].0 + i], ids[wires[0].0 + (i + 1) % outer_n]))
        .filter(|(a, b)| a != b)
        .collect();
    cdt.remove_exterior(&outer);
    let barrier = all
        .iter()
        .flat_map(|&(a, b)| {
            let (lo, hi) = sorted_pair(a, b);
            [(lo, hi), (hi, lo)]
        })
        .collect();
    (cdt, barrier)
}

fn removed_flags(cdt: &Cdt) -> Vec<bool> {
    cdt.triangles.iter().map(|t| t.removed).collect()
}

/// Every live triangle is counter-clockwise with positive area by the exact
/// predicate, and together they cover `area`. This is the invariant the
/// batch lookup's strict-inside uniqueness rests on.
fn assert_live_triangles_tile(cdt: &Cdt, area: f64, what: &str) {
    let mut sum = 0.0;
    for t in cdt.triangles.iter().filter(|t| !t.removed) {
        let [a, b, c] = t.v.map(|v| cdt.vertices[v]);
        assert!(
            orient2d(a, b, c) > 0.0,
            "{what}: live triangle {:?} is not CCW",
            t.v
        );
        sum += signed_tri_area(a, b, c);
    }
    assert!(
        (sum - area).abs() <= 1e-9 * area,
        "{what}: live triangles cover {sum}, expected {area}"
    );
}

/// Flood `seeds` through the reference, the one-seed API and the batch API
/// on three identically built CDTs, and require identical results.
/// `areas` is the domain area before and after the floods, when known.
fn assert_batch_matches(
    pts: &[Point2],
    wires: &[(usize, usize)],
    seeds: &[Point2],
    areas: Option<(f64, f64)>,
    what: &str,
) {
    assert_batch_matches_with_barrier(pts, wires, seeds, areas, &[], what);
}

/// [`assert_batch_matches`] with `extra` edges added to the caller's
/// barrier.
fn assert_batch_matches_with_barrier(
    pts: &[Point2],
    wires: &[(usize, usize)],
    seeds: &[Point2],
    areas: Option<(f64, f64)>,
    extra: &[(usize, usize)],
    what: &str,
) {
    let (mut legacy, mut barrier) = holed(pts, wires);
    barrier.extend(extra.iter().copied());
    let (mut single, _) = holed(pts, wires);
    let (mut batch, _) = holed(pts, wires);
    if let Some((before, _)) = areas {
        assert_live_triangles_tile(&batch, before, what);
    }
    let expected: Vec<bool> = seeds
        .iter()
        .map(|&s| reference_flood(&mut legacy, s, &barrier))
        .collect();
    let singles: Vec<bool> = seeds
        .iter()
        .map(|&s| single.flood_remove_from_point(s, &barrier))
        .collect();
    let got = batch.flood_remove_from_points(seeds, &barrier);
    assert_eq!(got, expected, "{what}: per-seed results");
    assert_eq!(singles, expected, "{what}: one-seed results");
    assert_eq!(
        removed_flags(&batch),
        removed_flags(&legacy),
        "{what}: batch removed"
    );
    assert_eq!(
        removed_flags(&single),
        removed_flags(&legacy),
        "{what}: one-seed removed"
    );
    assert_eq!(batch.triangles(), legacy.triangles(), "{what}: triangles");
    assert_eq!(
        batch.last_located, legacy.last_located,
        "{what}: hint moved"
    );
    if let Some((_, after)) = areas {
        assert_live_triangles_tile(&batch, after, what);
    }
}

fn polygon(pts: &[Point2], (start, end): (usize, usize)) -> f64 {
    shoelace_area(&pts[start..end])
}

/// A point just inside the first edge of wire `(start, end)` (wound either
/// way), the way `hole_removal_seeds` seeds a hole next to its own wire.
fn near_wire_seed(pts: &[Point2], (start, end): (usize, usize)) -> Point2 {
    let n = end - start;
    let mut c = Point2::new(0.0, 0.0);
    for p in &pts[start..end] {
        c = c + (*p - Point2::new(0.0, 0.0)) * (1.0 / n as f64);
    }
    let (a, b) = (pts[start], pts[start + 1]);
    let mid = a + (b - a) * 0.5;
    mid + (c - mid) * 0.0371
}

/// The hole grid with seeds next to each wire, at each centre, and in a
/// scrambled order: one result per seed, the same removals, and the batch
/// never needs the one-seed lookup for seeds near their wires.
#[test]
fn flood_batch_matches_one_seed_floods_on_the_hole_grid() {
    let (pts, wires) = hole_grid::layout();
    let outer = polygon(&pts, wires[0]);
    let holes: f64 = wires[1..].iter().map(|&w| polygon(&pts, w)).sum();
    let areas = Some((outer, outer - holes));

    let near: Vec<Point2> = wires[1..]
        .iter()
        .map(|&w| near_wire_seed(&pts, w))
        .collect();
    assert_batch_matches(&pts, &wires, &near, areas, "near-wire seeds");

    let centres: Vec<Point2> = (0..wires.len() - 1).map(hole_grid::centre).collect();
    assert_batch_matches(&pts, &wires, &centres, areas, "centre seeds");

    let mut scrambled = near.clone();
    let mut rng = 11;
    for i in (1..scrambled.len()).rev() {
        scrambled.swap(i, (xorshift64(&mut rng) as usize) % (i + 1));
    }
    assert_batch_matches(&pts, &wires, &scrambled, areas, "scrambled seeds");

    // Complexity guard: no seed falls back to the one-seed lookup, and the
    // chained walks between neighbouring holes stay short. The one-seed
    // loop walked into flooded holes, reset, and scanned.
    let (mut cdt, barrier) = holed(&pts, &wires);
    work::take(&work::SEED_WALK_STEPS);
    work::take(&work::SEED_LOOKUPS);
    assert!(
        cdt.flood_remove_from_points(&near, &barrier)
            .iter()
            .all(|&r| r)
    );
    assert_eq!(work::take(&work::SEED_LOOKUPS), 0);
    let steps = work::take(&work::SEED_WALK_STEPS);
    assert!(
        steps <= 32 * near.len(),
        "{steps} walk steps for {} seeds",
        near.len()
    );
}

/// The captured U-bracket floor: a concave outline whose notch was removed
/// as exterior, with its two hole seeds in both orders.
#[test]
fn flood_batch_matches_one_seed_floods_on_the_u_bracket_floor() {
    let pts: Vec<Point2> = u_bracket_floor::POINTS
        .iter()
        .map(|&(x, y)| Point2::new(x, y))
        .collect();
    let wires = u_bracket_floor::WIRES.to_vec();
    let [(x0, y0), (x1, y1)] = u_bracket_floor::HOLE_SEEDS;
    let (s0, s1) = (Point2::new(x0, y0), Point2::new(x1, y1));
    let outer = polygon(&pts, wires[0]);
    let after = outer - polygon(&pts, wires[1]) - polygon(&pts, wires[2]);
    assert_batch_matches(&pts, &wires, &[s0, s1], Some((outer, after)), "u-bracket");
    assert_batch_matches(
        &pts,
        &wires,
        &[s1, s0],
        Some((outer, after)),
        "u-bracket reversed",
    );
    assert_batch_matches(
        &pts,
        &wires,
        &[s1, s0, s1],
        Some((outer, after)),
        "u-bracket repeat",
    );
}

/// An L-shaped outline with holes in both arms: walks between the arms cross
/// the removed notch, so seeds there fall back to the one-seed lookup.
#[test]
fn flood_batch_matches_one_seed_floods_across_a_concave_notch() {
    let mut pts = vec![
        Point2::new(0.0, 0.0),
        Point2::new(60.0, 0.0),
        Point2::new(60.0, 20.0),
        Point2::new(20.0, 20.0),
        Point2::new(20.0, 60.0),
        Point2::new(0.0, 60.0),
    ];
    let mut wires = vec![(0, pts.len())];
    let mut seeds = Vec::new();
    let centres = [
        (50.0, 10.0),
        (10.0, 50.0),
        (35.0, 10.0),
        (10.0, 35.0),
        (10.0, 10.0),
    ];
    for (k, &(cx, cy)) in centres.iter().enumerate() {
        let start = pts.len();
        for i in 0..7 {
            let a = -f64::from(i) * std::f64::consts::TAU / 7.0 + 0.3 * k as f64;
            pts.push(Point2::new(cx + 3.0 * a.cos(), cy + 3.0 * a.sin()));
        }
        wires.push((start, pts.len()));
        seeds.push(near_wire_seed(&pts, (start, pts.len())));
    }
    let outer = polygon(&pts, wires[0]);
    let holes: f64 = wires[1..].iter().map(|&w| polygon(&pts, w)).sum();
    assert_batch_matches(
        &pts,
        &wires,
        &seeds,
        Some((outer, outer - holes)),
        "L outline",
    );
}

/// Nested square wires: a hole ring, an island, and a hole inside the
/// island. Seeds for the two hole regions, then the same plus seeds that
/// fall on a constraint edge, on a vertex, in a flooded region, outside the
/// domain, and NaN.
#[test]
fn flood_batch_matches_one_seed_floods_on_nested_wires_and_degenerate_seeds() {
    let square = |lo: f64, hi: f64| {
        [
            Point2::new(lo, lo),
            Point2::new(hi, lo),
            Point2::new(hi, hi),
            Point2::new(lo, hi),
        ]
    };
    let mut pts = Vec::new();
    let mut wires = Vec::new();
    for (lo, hi) in [(0.0, 100.0), (10.0, 90.0), (20.0, 80.0), (30.0, 70.0)] {
        let start = pts.len();
        pts.extend(square(lo, hi));
        wires.push((start, pts.len()));
    }
    let ring = Point2::new(15.0, 47.3);
    let inner = Point2::new(51.7, 48.9);
    let area = |lo: f64, hi: f64| (hi - lo) * (hi - lo);
    let after = area(0.0, 100.0) - area(10.0, 90.0) + area(20.0, 80.0) - area(30.0, 70.0);
    assert_batch_matches(
        &pts,
        &wires,
        &[ring, inner],
        Some((area(0.0, 100.0), after)),
        "nested holes",
    );

    let degenerate = [
        Point2::new(50.0, 10.0), // on the ring's outer edge
        Point2::new(20.0, 20.0), // on a vertex
        inner,
        Point2::new(52.0, 47.0),   // in the hole flooded just before
        Point2::new(-50.0, -50.0), // outside the domain
        Point2::new(150.0, 50.0),
        Point2::new(f64::NAN, 5.0),
        Point2::new(f64::INFINITY, 5.0),
        ring,
        Point2::new(25.0, 50.0), // the island
    ];
    assert_batch_matches(&pts, &wires, &degenerate, None, "degenerate seeds");

    // The one-seed lookup serves the six seeds no live triangle strictly
    // contains, and two whose triangles earlier floods removed: the second
    // seed in the inner hole, and the ring seed after the on-edge seed's
    // flood took the ring. A walk that kept crossing the on-edge seed's zero
    // edge, or wandered on the NaN seed, would spend the shared budget and
    // push the island seed onto that lookup too.
    let (mut cdt, barrier) = holed(&pts, &wires);
    let located = cdt.locate_seeds_strictly_inside(&degenerate);
    assert_eq!(located.iter().filter(|t| t.is_none()).count(), 6);
    work::take(&work::SEED_WALK_STEPS);
    work::take(&work::SEED_LOOKUPS);
    cdt.flood_remove_from_points(&degenerate, &barrier);
    assert_eq!(work::take(&work::SEED_LOOKUPS), 8);
    let steps = work::take(&work::SEED_WALK_STEPS);
    assert!(steps <= 32 * degenerate.len(), "{steps} walk steps");
}

/// Edges that only the caller's barrier holds stop the flood too: a live
/// triangle whose three edges are given as barrier edges floods alone.
#[test]
fn flood_batch_respects_caller_only_barrier_edges() {
    let (pts, wires) = hole_grid::layout();
    let (cdt, _) = holed(&pts, &wires);
    let (lone, tri) = cdt
        .triangles
        .iter()
        .enumerate()
        .find(|(_, t)| {
            !t.removed
                && (0..3).all(|i| {
                    let edge = sorted_pair(t.v[i], t.v[(i + 1) % 3]);
                    !cdt.constraints.contains(&edge)
                })
        })
        .unwrap();
    let extra: Vec<(usize, usize)> = (0..3)
        .map(|i| sorted_pair(tri.v[i], tri.v[(i + 1) % 3]))
        .collect();
    let [a, b, c] = tri.v.map(|v| cdt.vertices[v]);
    let centroid = Point2::new((a.x() + b.x() + c.x()) / 3.0, (a.y() + b.y() + c.y()) / 3.0);
    let mut seeds = vec![centroid];
    seeds.extend(wires[1..].iter().map(|&w| near_wire_seed(&pts, w)));
    assert_batch_matches_with_barrier(&pts, &wires, &seeds, None, &extra, "caller-only edges");

    // The centroid's flood takes exactly that triangle.
    let (mut cdt, mut barrier) = holed(&pts, &wires);
    barrier.extend(extra.iter().copied());
    let live = |cdt: &Cdt| cdt.triangles.iter().filter(|t| !t.removed).count();
    let before = live(&cdt);
    assert!(cdt.flood_remove_from_point(centroid, &barrier));
    assert!(cdt.triangles[lone].removed);
    assert_eq!(live(&cdt), before - 1);
}

/// A hole whose bottom edge runs through a vertex of the hole below it:
/// constraint insertion splits that edge there, so only the CDT's own
/// constraints hold the two halves, and the floods must stop at them.
#[test]
fn flood_batch_respects_split_constraints() {
    let pts = vec![
        Point2::new(0.0, 0.0),
        Point2::new(60.0, 0.0),
        Point2::new(60.0, 60.0),
        Point2::new(0.0, 60.0),
        // Hole A: its bottom edge passes through B's apex at (30, 20).
        Point2::new(20.0, 20.0),
        Point2::new(20.0, 40.0),
        Point2::new(40.0, 40.0),
        Point2::new(40.0, 20.0),
        // Hole B, below A, touching it at its apex.
        Point2::new(30.0, 20.0),
        Point2::new(35.0, 10.0),
        Point2::new(25.0, 10.0),
    ];
    let wires = [(0, 4), (4, 8), (8, 11)];
    let (cdt, barrier) = holed(&pts, &wires);
    assert!(
        cdt.constraints
            .iter()
            .any(|&(a, b)| !barrier.contains(&(a, b))),
        "no constraint was split"
    );
    let a = Point2::new(31.3, 29.1);
    let b = Point2::new(30.2, 13.7);
    let outer = polygon(&pts, wires[0]);
    let after = outer - polygon(&pts, wires[1]) - polygon(&pts, wires[2]);
    assert_batch_matches(&pts, &wires, &[a, b], Some((outer, after)), "split edge");
    assert_batch_matches(
        &pts,
        &wires,
        &[b, a],
        Some((outer, after)),
        "split edge reversed",
    );
}

#[test]
fn flood_batch_handles_zero_and_one_seed() {
    let (pts, wires) = hole_grid::layout();
    assert_batch_matches(&pts, &wires, &[], None, "no seeds");
    let one = [near_wire_seed(&pts, wires[5])];
    assert_batch_matches(&pts, &wires, &one, None, "one seed");
}

proptest! {
    /// Random grids of rotated regular holes, seeded near their wires or at
    /// their centres, in shuffled order, sometimes twice.
    #[test]
    fn flood_batch_matches_one_seed_floods_on_random_hole_grids(
        seed in 0u64..1_000_000,
        cols in 1usize..6,
        rows in 1usize..6,
        sides in 3usize..13,
    ) {
        let mut rng = seed | 1;
        let pitch = 10.0;
        let (w, h) = (pitch * cols as f64, pitch * rows as f64);
        let mut pts = vec![
            Point2::new(0.0, 0.0),
            Point2::new(w, 0.0),
            Point2::new(w, h),
            Point2::new(0.0, h),
        ];
        let mut wires = vec![(0, 4)];
        let mut seeds = Vec::new();
        for k in 0..cols * rows {
            let c = Point2::new(
                pitch * ((k % cols) as f64 + 0.5),
                pitch * ((k / cols) as f64 + 0.5),
            );
            let r = rand_f64(&mut rng, 1.0, 4.5);
            let phase = rand_f64(&mut rng, 0.0, 1.0);
            let start = pts.len();
            for i in 0..sides {
                let a = -(i as f64 + phase) * std::f64::consts::TAU / sides as f64;
                pts.push(Point2::new(c.x() + r * a.cos(), c.y() + r * a.sin()));
            }
            wires.push((start, pts.len()));
            let s = if xorshift64(&mut rng).is_multiple_of(3) {
                c
            } else {
                near_wire_seed(&pts, (start, pts.len()))
            };
            seeds.push(s);
            if xorshift64(&mut rng).is_multiple_of(7) {
                seeds.push(s);
            }
        }
        for i in (1..seeds.len()).rev() {
            seeds.swap(i, (xorshift64(&mut rng) as usize) % (i + 1));
        }
        assert_batch_matches(&pts, &wires, &seeds, None, &format!("seed {seed}"));
    }
}
