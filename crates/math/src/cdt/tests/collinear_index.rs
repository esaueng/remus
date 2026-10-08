//! The coordinate index behind `insert_constraint`'s collinear-vertex query
//! returns exactly what the plain scan over every vertex returns.
//!
//! The reference below is that scan, verbatim from before the index. The
//! query tests push vertices straight into `Cdt::vertices`, without
//! triangulating, since the query reads nothing else. That reaches inputs
//! `insert_point` would weld or cannot locate: near-duplicates, NaN, and
//! coordinates past the index's limit.

use super::*;
use crate::cdt::collinear::CollinearIndex;
use crate::cdt::work;

/// Forces the collinear index on or off on this thread until dropped.
struct Policy;

impl Policy {
    fn set(policy: Option<bool>) -> Self {
        work::INDEX_POLICY.with(|c| c.set(policy));
        Self
    }
}

impl Drop for Policy {
    fn drop(&mut self) {
        work::INDEX_POLICY.with(|c| c.set(None));
    }
}

/// `insert_constraint`'s collinear scan before the index, verbatim.
fn reference_collinear(cdt: &Cdt, v0: usize, v1: usize) -> Vec<(f64, usize)> {
    let p0 = cdt.vertices[v0];
    let p1 = cdt.vertices[v1];
    let dx = p1.x() - p0.x();
    let dy = p1.y() - p0.y();
    let seg_len_sq = dx * dx + dy * dy;

    let mut collinear: Vec<(f64, usize)> = Vec::new();
    if seg_len_sq > 0.0 {
        for vi in cdt.super_count..cdt.vertices.len() {
            if vi == v0 || vi == v1 {
                continue;
            }
            let px = cdt.vertices[vi].x() - p0.x();
            let py = cdt.vertices[vi].y() - p0.y();
            let t = (px * dx + py * dy) / seg_len_sq;
            if t <= 1e-6 || t >= 1.0 - 1e-6 {
                continue;
            }
            let cross = px * dy - py * dx;
            let dist_sq = cross * cross / seg_len_sq;
            if dist_sq < DUP_TOL * DUP_TOL {
                collinear.push((t, vi));
            }
        }
        collinear.sort_by(|a, b| a.0.total_cmp(&b.0));
    }
    collinear
}

/// The query as `insert_constraint` makes it.
fn query(cdt: &mut Cdt, v0: usize, v1: usize) -> Vec<(f64, usize)> {
    let p0 = cdt.vertices[v0];
    let p1 = cdt.vertices[v1];
    let dx = p1.x() - p0.x();
    let dy = p1.y() - p0.y();
    let seg_len_sq = dx * dx + dy * dy;
    if seg_len_sq > 0.0 {
        cdt.collinear_vertices(v0, v1, seg_len_sq)
    } else {
        Vec::new()
    }
}

fn bits(hits: &[(f64, usize)]) -> Vec<(u64, usize)> {
    hits.iter().map(|&(t, vi)| (t.to_bits(), vi)).collect()
}

/// Require the query to match the reference; returns the number of hits.
fn assert_query_matches(cdt: &mut Cdt, v0: usize, v1: usize, what: &str) -> usize {
    let expected = reference_collinear(cdt, v0, v1);
    let got = query(cdt, v0, v1);
    assert_eq!(bits(&got), bits(&expected), "{what}: ({v0}, {v1})");
    expected.len()
}

/// A CDT shell whose vertex list is `pts` after the super-triangle.
fn raw_cdt(pts: &[Point2]) -> Cdt {
    let mut cdt = Cdt::new((Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)));
    cdt.vertices.extend_from_slice(pts);
    cdt
}

fn push(cdt: &mut Cdt, p: Point2) -> usize {
    cdt.vertices.push(p);
    cdt.vertices.len() - 1
}

fn is_built(cdt: &Cdt) -> bool {
    matches!(cdt.collinear_index, CollinearIndex::Built(_))
}

fn indexed_end(cdt: &Cdt) -> Option<usize> {
    match &cdt.collinear_index {
        CollinearIndex::Built(index) => Some(index.end),
        _ => None,
    }
}

/// Perpendicular offsets around the 1e-8 acceptance distance.
const OFFSETS: [f64; 9] = [
    0.0, 0.5e-8, -0.5e-8, 0.99e-8, -0.99e-8, 1.01e-8, -1.01e-8, 1e-7, -1e-7,
];

/// Parameters around the open-interval ends and inside.
const PARAMS: [f64; 7] = [
    1e-6 - 1e-12,
    1e-6 + 1e-12,
    0.25,
    0.5,
    0.75,
    1.0 - 1e-6 - 1e-12,
    1.0 - 1e-6 + 1e-12,
];

/// Plant a vertex at each parameter and perpendicular offset along the
/// segment `(a, b)`.
fn plant(cdt: &mut Cdt, a: Point2, b: Point2) {
    let d = b - a;
    let len = d.length();
    let n = Point2::new(-d.y() / len, d.x() / len);
    for t in PARAMS {
        for off in OFFSETS {
            push(
                cdt,
                Point2::new(
                    a.x() + t * d.x() + off * n.x(),
                    a.y() + t * d.y() + off * n.y(),
                ),
            );
        }
    }
}

/// Segment directions: horizontal, vertical, diagonal, nearly vertical and
/// nearly horizontal.
const DIRECTIONS: [(f64, f64); 5] = [
    (1.0, 0.0),
    (0.0, 1.0),
    (0.6, -0.8),
    (1e-9, 1.0),
    (1.0, 3e-7),
];

/// One random cloud with planted segments, queried with the index forced on
/// or left to its default gate, with a tail of vertices added after the
/// index was built.
fn check_planted_cloud(seed: u64, scale: f64, policy: Option<bool>) {
    let _policy = Policy::set(policy);
    let mut rng = seed | 1;
    let mut cdt = raw_cdt(&[]);
    for _ in 0..120 {
        push(
            &mut cdt,
            Point2::new(
                rand_f64(&mut rng, 0.0, scale),
                rand_f64(&mut rng, 0.0, scale),
            ),
        );
    }
    let mut segments = Vec::new();
    for (ux, uy) in DIRECTIONS {
        let len = scale * rand_f64(&mut rng, 0.05, 0.5);
        let a = Point2::new(
            rand_f64(&mut rng, 0.0, 0.5 * scale),
            rand_f64(&mut rng, 0.0, 0.5 * scale),
        );
        let b = Point2::new(a.x() + len * ux, a.y() + len * uy);
        let (ia, ib) = (push(&mut cdt, a), push(&mut cdt, b));
        plant(&mut cdt, a, b);
        segments.push((ia, ib));
    }
    let n = cdt.vertices.len();
    for _ in 0..40 {
        let a = cdt.super_count + (xorshift64(&mut rng) as usize) % (n - cdt.super_count);
        let b = cdt.super_count + (xorshift64(&mut rng) as usize) % (n - cdt.super_count);
        segments.push((a, b));
    }

    let what = format!("seed {seed}, scale {scale}, policy {policy:?}");
    let mut hits = 0;
    for round in 0..3 {
        for &(a, b) in &segments {
            hits += assert_query_matches(&mut cdt, a, b, &what);
            hits += assert_query_matches(&mut cdt, b, a, &what);
        }
        // Vertices added after the index was built, on the segments too.
        for &(a, b) in &segments[..DIRECTIONS.len()] {
            let (pa, pb) = (cdt.vertices[a], cdt.vertices[b]);
            let t = 0.1 * f64::from(round + 1);
            push(&mut cdt, pa + (pb - pa) * t);
        }
    }
    assert!(
        policy == Some(false) || is_built(&cdt),
        "{what}: index never built"
    );
    // Each planted segment carries accepted vertices.
    assert!(hits >= 2 * DIRECTIONS.len(), "{what}: only {hits} hits");
}

proptest! {
    #[test]
    fn collinear_index_matches_the_plain_scan_on_planted_clouds(
        seed in 0u64..1_000_000,
        scale_index in 0usize..4,
        forced in any::<bool>(),
    ) {
        let scale = [1e-3, 1.0, 1e3, 1e6][scale_index];
        check_planted_cloud(seed, scale, forced.then_some(true));
    }
}

#[test]
fn collinear_index_matches_the_plain_scan_at_every_scale() {
    for scale in [1e-3, 1.0, 1e3, 1e6] {
        for policy in [Some(true), None] {
            for seed in [1, 7, 42] {
                check_planted_cloud(seed, scale, policy);
            }
        }
    }
}

/// Vertices added after the build are found by the tail scan, the first of
/// them included, and the index is rebuilt exactly when the tail outgrows
/// `max(64, indexed / 8)`.
#[test]
fn collinear_index_scans_its_tail_and_rebuilds_on_schedule() {
    let _policy = Policy::set(Some(true));
    let mut cdt = raw_cdt(&[]);
    let mut rng = 3;
    for _ in 0..1000 {
        push(
            &mut cdt,
            Point2::new(
                rand_f64(&mut rng, 0.0, 100.0),
                rand_f64(&mut rng, 0.0, 100.0),
            ),
        );
    }
    let a = push(&mut cdt, Point2::new(-10.0, -5.0));
    let b = push(&mut cdt, Point2::new(-10.0, 15.0));
    assert!(query(&mut cdt, a, b).is_empty());
    let end = indexed_end(&cdt).unwrap();
    assert_eq!(end, cdt.vertices.len());
    let indexed = end - cdt.super_count;
    let limit = 64.max(indexed / 8);

    // The first vertex past the index lies on the segment.
    let first = push(&mut cdt, Point2::new(-10.0, 0.0));
    assert_eq!(
        bits(&query(&mut cdt, a, b)),
        bits(&reference_collinear(&cdt, a, b))
    );
    assert_eq!(query(&mut cdt, a, b).len(), 1);
    assert_eq!(query(&mut cdt, a, b)[0].1, first);
    assert_eq!(indexed_end(&cdt), Some(end), "rebuilt early");

    while cdt.vertices.len() - end < limit {
        let y = rand_f64(&mut rng, -4.0, 14.0);
        push(&mut cdt, Point2::new(-10.0, y));
    }
    assert_query_matches(&mut cdt, a, b, "tail at the limit");
    assert_eq!(indexed_end(&cdt), Some(end), "rebuilt at the limit");
    push(&mut cdt, Point2::new(-10.0, 14.5));
    assert_query_matches(&mut cdt, a, b, "tail past the limit");
    assert_eq!(indexed_end(&cdt), Some(cdt.vertices.len()), "not rebuilt");
}

/// The default gate builds the index once plain scans have tested
/// `2·n·⌈log2(n + 1)⌉` vertices, and never below 64 vertices.
#[test]
fn collinear_index_builds_after_its_scan_budget() {
    for n in [63_usize, 64, 100] {
        let mut cdt = raw_cdt(&[]);
        for i in 0..n {
            let x = i as f64;
            push(&mut cdt, Point2::new(x, (x * 0.37).sin()));
        }
        let (a, b) = (cdt.super_count, cdt.super_count + 1);
        let log2 = (usize::BITS - n.leading_zeros()) as usize;
        let budget = 2 * n * log2;
        let scans_before_build = budget.div_ceil(n);
        for q in 0..scans_before_build + 3 {
            assert_eq!(
                is_built(&cdt),
                n >= 64 && q > scans_before_build,
                "n {n}, query {q}"
            );
            work::take(&work::COLLINEAR_VISITS);
            assert_query_matches(&mut cdt, a, b, "gate");
            let visits = work::take(&work::COLLINEAR_VISITS);
            if !is_built(&cdt) {
                assert_eq!(visits, n - 2, "a plain scan visits every other vertex");
            }
        }
    }
}

/// The index walks only the vertices in the window along the segment's
/// narrower axis, on a 40 × 40 unit grid one column or row, and tests only
/// those inside the window on the other axis too.
#[test]
fn collinear_index_visits_only_the_narrow_window() {
    let _policy = Policy::set(Some(true));
    let mut cdt = raw_cdt(&[]);
    for i in 0..40 {
        for j in 0..40 {
            push(&mut cdt, Point2::new(f64::from(i), f64::from(j)));
        }
    }
    let id = |cdt: &Cdt, i: usize, j: usize| cdt.super_count + 40 * i + j;
    for ((a, b), mid) in [
        // Vertical: the x = 10 column.
        ((id(&cdt, 10, 5), id(&cdt, 10, 7)), id(&cdt, 10, 6)),
        // Horizontal: the y = 20 row.
        ((id(&cdt, 3, 20), id(&cdt, 5, 20)), id(&cdt, 4, 20)),
    ] {
        query(&mut cdt, a, b);
        work::take(&work::COLLINEAR_VISITS);
        work::take(&work::COLLINEAR_TESTS);
        let hits = query(&mut cdt, a, b);
        assert_eq!(work::take(&work::COLLINEAR_VISITS), 40);
        // Only the vertex between the endpoints survives the other axis.
        assert_eq!(work::take(&work::COLLINEAR_TESTS), 1);
        assert_eq!(hits.iter().map(|h| h.1).collect::<Vec<_>>(), [mid]);
        assert_eq!(bits(&hits), bits(&reference_collinear(&cdt, a, b)));
    }
}

/// NaN and infinite vertices stay out of the index without disturbing it,
/// including a sign-bit NaN that would sort before every key.
#[test]
fn collinear_index_ignores_non_finite_vertices() {
    let _policy = Policy::set(Some(true));
    let zero = 0.0_f64;
    let nan = zero / zero;
    let mut cdt = raw_cdt(&[]);
    push(&mut cdt, Point2::new(-nan.abs(), 5.0));
    push(&mut cdt, Point2::new(nan, nan));
    push(&mut cdt, Point2::new(f64::INFINITY, 0.0));
    push(&mut cdt, Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    let a = push(&mut cdt, Point2::new(0.0, 0.0));
    let b = push(&mut cdt, Point2::new(100.0, 0.0));
    let c = push(&mut cdt, Point2::new(0.0, 100.0));
    let on_ab = push(&mut cdt, Point2::new(50.0, 0.0));
    let on_ac = push(&mut cdt, Point2::new(0.0, 25.0));
    for (s, t, hit) in [(a, b, on_ab), (b, a, on_ab), (a, c, on_ac), (c, a, on_ac)] {
        let got = query(&mut cdt, s, t);
        assert!(is_built(&cdt));
        assert_eq!(got.iter().map(|h| h.1).collect::<Vec<_>>(), [hit]);
        assert_eq!(bits(&got), bits(&reference_collinear(&cdt, s, t)));
    }
}

/// Past the coordinate limit the window's rounding analysis no longer holds,
/// so the query falls back to the plain scan, whose answers there include
/// vertices far outside the segment.
#[test]
fn collinear_index_falls_back_past_its_coordinate_limit() {
    let _policy = Policy::set(Some(true));

    // A segment so long that `dx²` overflows: the plain test computes
    // t = inf/inf = NaN and cross²/L² = 0, and accepts every vertex on the
    // line. No finite window holds them.
    let mut cdt = raw_cdt(&[]);
    let a = push(&mut cdt, Point2::new(0.0, 0.0));
    let b = push(&mut cdt, Point2::new(1e200, 0.0));
    let far = push(&mut cdt, Point2::new(2e300, 0.0));
    let expected = reference_collinear(&cdt, a, b);
    assert_eq!(expected.iter().map(|h| h.1).collect::<Vec<_>>(), [far]);
    assert_eq!(bits(&query(&mut cdt, a, b)), bits(&expected));
    assert!(matches!(
        cdt.collinear_index,
        CollinearIndex::Pending { .. }
    ));

    // One indexed vertex past the limit disables the index for good.
    let mut cdt = raw_cdt(&[]);
    let a = push(&mut cdt, Point2::new(0.0, 0.0));
    let b = push(&mut cdt, Point2::new(10.0, 10.0));
    push(&mut cdt, Point2::new(5.0, 5.0));
    push(&mut cdt, Point2::new(1e60, 1.0));
    assert_query_matches(&mut cdt, a, b, "huge vertex");
    assert!(matches!(cdt.collinear_index, CollinearIndex::Unindexable));
    assert_query_matches(&mut cdt, b, a, "huge vertex, reversed");

    // Tiny segments scan too.
    let mut cdt = raw_cdt(&[]);
    let a = push(&mut cdt, Point2::new(1e-40, 0.0));
    let b = push(&mut cdt, Point2::new(3e-40, 0.0));
    push(&mut cdt, Point2::new(2e-40, 0.0));
    assert_query_matches(&mut cdt, a, b, "tiny segment");
    assert!(matches!(
        cdt.collinear_index,
        CollinearIndex::Pending { .. }
    ));
}

/// How a fixture is triangulated before its constraints go in.
#[derive(Clone, Copy)]
enum Insertion {
    /// `insert_points_hilbert` over `run_planar_cdt`'s bounds.
    Hilbert,
    /// `insert_point` one by one over the given bounds.
    OneByOne(Point2, Point2),
}

/// Constraint outcomes, vertex bits, triangles and constraint edges.
type Constrained = (
    Vec<Option<String>>,
    Vec<u64>,
    Vec<(usize, usize, usize)>,
    Vec<(usize, usize)>,
);

/// Triangulate `pts` and insert `edges` as constraints, with the collinear
/// index forced on or off, or left to its default gate.
fn constrained(
    pts: &[Point2],
    edges: &[(usize, usize)],
    insertion: Insertion,
    policy: Option<bool>,
) -> Constrained {
    let _policy = Policy::set(policy);
    let (mut cdt, ids) = match insertion {
        Insertion::Hilbert => {
            let mut cdt = Cdt::with_capacity(planar_bounds(pts), pts.len());
            let ids = cdt.insert_points_hilbert(pts).unwrap();
            (cdt, ids)
        }
        Insertion::OneByOne(lo, hi) => {
            let mut cdt = Cdt::with_capacity((lo, hi), pts.len());
            let ids = pts.iter().map(|&p| cdt.insert_point(p).unwrap()).collect();
            (cdt, ids)
        }
    };
    let outcomes = edges
        .iter()
        .map(|&(a, b)| {
            cdt.insert_constraint(ids[a], ids[b])
                .err()
                .map(|e| format!("{e:?}"))
        })
        .collect();
    let verts = cdt
        .vertices()
        .iter()
        .flat_map(|p| [p.x().to_bits(), p.y().to_bits()])
        .collect();
    let mut constraints: Vec<_> = cdt.constraint_edges().iter().copied().collect();
    constraints.sort_unstable();
    (outcomes, verts, cdt.triangles(), constraints)
}

fn wire_edges(wires: &[(usize, usize)]) -> Vec<(usize, usize)> {
    let mut edges = Vec::new();
    for &(start, end) in wires {
        let n = end - start;
        for i in 0..n {
            edges.push((start + i, start + (i + 1) % n));
        }
    }
    edges
}

/// Constraint insertion on the captured caps and the hole grid gives the
/// same triangulation whether the collinear query scans or uses the index.
/// The off-axis cap also adds a Steiner point after the index is built.
#[test]
fn collinear_index_keeps_constrained_triangulations_identical() {
    let (grid, grid_wires) = hole_grid::layout();
    let bracket: Vec<Point2> = u_bracket_floor::POINTS
        .iter()
        .map(|&(x, y)| Point2::new(x, y))
        .collect();
    let cap: Vec<Point2> = off_axis_cap::POINTS
        .iter()
        .map(|&(x, y)| Point2::new(x, y))
        .collect();
    let h = off_axis_cap::HALF_EXTENT;
    let cap_bounds = Insertion::OneByOne(Point2::new(-h, -h), Point2::new(h, h));
    let fixtures = [
        (
            "hole grid",
            grid,
            wire_edges(&grid_wires),
            Insertion::Hilbert,
        ),
        (
            "u-bracket floor",
            bracket,
            wire_edges(&u_bracket_floor::WIRES),
            Insertion::Hilbert,
        ),
        (
            "off-axis cap",
            cap,
            off_axis_cap::CONSTRAINTS.to_vec(),
            cap_bounds,
        ),
    ];
    for (name, pts, edges, insertion) in fixtures {
        let plain = constrained(&pts, &edges, insertion, Some(false));
        assert!(plain.0.iter().all(Option::is_none), "{name}: {:?}", plain.0);
        let forced = constrained(&pts, &edges, insertion, Some(true));
        assert!(forced == plain, "{name}: forced index differs");
        let default = constrained(&pts, &edges, insertion, None);
        assert!(default == plain, "{name}: default gate differs");
    }
}

proptest! {
    /// Random clouds with constraints along planted collinear runs: the
    /// same triangulation with the index forced on as with the plain scan.
    #[test]
    fn collinear_index_keeps_random_constrained_triangulations_identical(
        seed in 0u64..1_000_000,
    ) {
        let mut rng = seed | 1;
        let mut pts = Vec::new();
        for _ in 0..60 {
            pts.push(Point2::new(rand_f64(&mut rng, 0.0, 100.0), rand_f64(&mut rng, 0.0, 100.0)));
        }
        let mut edges = Vec::new();
        for _ in 0..6 {
            let a = Point2::new(rand_f64(&mut rng, 0.0, 100.0), rand_f64(&mut rng, 0.0, 100.0));
            let b = Point2::new(rand_f64(&mut rng, 0.0, 100.0), rand_f64(&mut rng, 0.0, 100.0));
            let start = pts.len();
            pts.push(a);
            for t in [0.2, 0.45, 0.7] {
                pts.push(a + (b - a) * t);
            }
            pts.push(b);
            edges.push((start, pts.len() - 1));
        }
        let plain = constrained(&pts, &edges, Insertion::Hilbert, Some(false));
        let forced = constrained(&pts, &edges, Insertion::Hilbert, Some(true));
        prop_assert!(forced == plain, "seed {}", seed);
    }
}

/// Complexity guard on the hole grid: once the index is built, each
/// constraint visits at most 64 vertices (the plain scan visits all 1153
/// others), and the whole insertion stays near its build budget.
#[test]
fn collinear_index_bounds_hole_grid_constraint_work() {
    let (pts, wires) = hole_grid::layout();
    let mut cdt = Cdt::with_capacity(planar_bounds(&pts), pts.len());
    let ids = cdt.insert_points_hilbert(&pts).unwrap();
    let edges = wire_edges(&wires);
    let n = pts.len();
    work::take(&work::COLLINEAR_VISITS);
    let mut total = 0;
    for &(a, b) in &edges {
        let built = is_built(&cdt);
        cdt.insert_constraint(ids[a], ids[b]).unwrap();
        let visits = work::take(&work::COLLINEAR_VISITS);
        total += visits;
        if built {
            assert!(
                visits <= 64,
                "constraint ({a}, {b}) visited {visits} vertices"
            );
        }
    }
    assert!(is_built(&cdt));
    let log2 = (usize::BITS - n.leading_zeros()) as usize;
    let bound = 2 * n * log2 + n + 64 * edges.len();
    assert!(total <= bound, "{total} vertex visits, bound {bound}");
}
