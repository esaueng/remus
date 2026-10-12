//! The integer in-circle sign and the scanned duplicate lookup change no
//! triangulation.
//!
//! `incircle::exact_in_circle_sign` must agree in sign with
//! `robust::incircle` wherever it answers, and a CDT built with both
//! shortcuts must be identical, vertex for vertex and triangle for triangle,
//! to one built with `work::LEGACY_PREDICATES` set: the eager duplicate grid
//! and `robust::incircle` for every in-circle test the float filter cannot
//! decide.

use super::off_axis_cap;
use super::*;
use crate::cdt::incircle::exact_in_circle_sign;
use crate::cdt::work;

/// Runs the CDT on the legacy predicates on this thread until dropped.
struct Legacy;

impl Legacy {
    fn set() -> Self {
        work::LEGACY_PREDICATES.with(|c| c.set(true));
        Self
    }
}

impl Drop for Legacy {
    fn drop(&mut self) {
        work::LEGACY_PREDICATES.with(|c| c.set(false));
    }
}

/// Deterministic SplitMix64 stream.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    #[allow(clippy::cast_precision_loss)]
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1_u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    /// A "CAD" decimal: a multiple of `10^-decimals` in `[lo, hi)`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    fn decimal(&mut self, lo: f64, hi: f64, decimals: i32) -> f64 {
        let scale = 10_f64.powi(decimals);
        (self.range(lo, hi) * scale).round() / scale
    }

    fn index(&mut self, n: usize) -> usize {
        #[allow(clippy::cast_possible_truncation)]
        let i = (self.next_u64() % n as u64) as usize;
        i
    }
}

fn sign(x: f64) -> i8 {
    if x > 0.0 {
        1
    } else if x < 0.0 {
        -1
    } else {
        0
    }
}

/// Quadruples whose in-circle sign the float filter cannot settle, plus
/// ordinary and out-of-window ones.
fn quadruples() -> Vec<[Point2; 4]> {
    let mut rng = Rng(0x1234_5678);
    let mut out = Vec::new();
    let p = Point2::new;
    for _ in 0..4000 {
        // Axis-aligned rectangles with decimal corners: exactly cocircular,
        // in every vertex order the legalizer can present them.
        let (x0, x1) = (rng.decimal(-500.0, 500.0, 1), rng.decimal(-500.0, 500.0, 3));
        let (y0, y1) = (rng.decimal(-500.0, 500.0, 2), rng.decimal(-500.0, 500.0, 1));
        let corners = [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
        let r = rng.index(4);
        out.push([
            corners[r],
            corners[(r + 1) % 4],
            corners[(r + 2) % 4],
            corners[(r + 3) % 4],
        ]);
        out.push([
            corners[r],
            corners[(r + 2) % 4],
            corners[(r + 1) % 4],
            corners[(r + 3) % 4],
        ]);

        // The same rectangle nudged by one ulp at a random corner.
        let mut nudged = corners;
        let k = rng.index(4);
        let bumped = f64::from_bits(nudged[k].x().to_bits() ^ 1);
        nudged[k] = p(bumped, nudged[k].y());
        out.push(nudged);

        // Integer grid points: frequent exact cocircularity.
        let g = |rng: &mut Rng| p(rng.range(-4.0, 4.0).round(), rng.range(-4.0, 4.0).round());
        out.push([g(&mut rng), g(&mut rng), g(&mut rng), g(&mut rng)]);

        // Points on one circle (nearly cocircular after rounding).
        let (cx, cy, rad) = (
            rng.range(-50.0, 50.0),
            rng.range(-50.0, 50.0),
            rng.range(0.1, 40.0),
        );
        let on = |rng: &mut Rng| {
            let t = rng.range(0.0, std::f64::consts::TAU);
            p(cx + rad * t.cos(), cy + rad * t.sin())
        };
        out.push([on(&mut rng), on(&mut rng), on(&mut rng), on(&mut rng)]);

        // Generic points.
        let q = |rng: &mut Rng| p(rng.range(-1e3, 1e3), rng.range(-1e3, 1e3));
        out.push([q(&mut rng), q(&mut rng), q(&mut rng), q(&mut rng)]);

        // Mixed magnitudes: a far vertex beside near-zero noise (outside the
        // integer window), and a rectangle scaled far from the unit range.
        out.push([
            p(-6.8e-15, 1.0),
            p(37.1, 0.0),
            p(0.0, 37.1),
            p(-37.1, 4.5e-15),
        ]);
        let s = 2_f64.powi(i32::try_from(rng.index(400)).unwrap() - 200);
        out.push([
            p(x0 * s, y0 * s),
            p(x1 * s, y0 * s),
            p(x1 * s, y1 * s),
            p(x0 * s, y1 * s),
        ]);
    }
    // Repeated and coincident points, zeros, and non-finite coordinates.
    out.push([p(0.0, 0.0); 4]);
    out.push([p(1.0, 2.0), p(1.0, 2.0), p(3.0, 4.0), p(5.0, -1.0)]);
    out.push([p(-0.0, 0.0), p(1.0, 0.0), p(0.0, 1.0), p(1.0, 1.0)]);
    out.push([p(f64::NAN, 0.0), p(1.0, 0.0), p(0.0, 1.0), p(1.0, 1.0)]);
    out.push([p(f64::INFINITY, 0.0), p(1.0, 0.0), p(0.0, 1.0), p(1.0, 1.0)]);
    out.push([
        p(f64::MIN_POSITIVE, 0.0),
        p(1e-310, 0.0),
        p(0.0, 1e-310),
        p(1e-310, 1e-310),
    ]);
    out.push([p(1e300, 0.0), p(-1e300, 0.0), p(0.0, 1e300), p(0.0, -1e300)]);
    out
}

#[test]
fn exact_in_circle_sign_matches_robust() {
    let mut answered = 0_usize;
    let mut cocircular = 0_usize;
    for [a, b, c, d] in quadruples() {
        let reference = in_circle(a, b, c, d);
        if let Some(exact) = exact_in_circle_sign(a, b, c, d) {
            answered += 1;
            if exact == 0.0 {
                cocircular += 1;
            }
            assert_eq!(
                sign(exact),
                sign(reference),
                "sign mismatch for {a:?} {b:?} {c:?} {d:?}: {exact} vs {reference}"
            );
        }
    }
    // The window must actually cover the decimal and grid cases.
    assert!(answered > 20_000, "answered {answered}");
    assert!(cocircular > 8_000, "cocircular {cocircular}");
}

#[test]
fn exact_in_circle_sign_declines_outside_its_window() {
    let p = Point2::new;
    // Near-zero noise beside model-sized coordinates spans > 61 bits.
    assert!(
        exact_in_circle_sign(p(-6.8e-15, 1.0), p(37.1, 0.0), p(0.0, 37.1), p(1.0, 1.0)).is_none()
    );
    assert!(
        exact_in_circle_sign(p(f64::NAN, 0.0), p(1.0, 0.0), p(0.0, 1.0), p(1.0, 1.0)).is_none()
    );
    assert!(
        exact_in_circle_sign(p(1e-70, 0.0), p(2e-70, 0.0), p(0.0, 1e-70), p(1e-70, 1e-70))
            .is_none()
    );
    assert!(
        exact_in_circle_sign(p(1e70, 0.0), p(2e70, 0.0), p(0.0, 1e70), p(1e70, 1e70)).is_none()
    );
    // A decimal rectangle is inside it, and exactly cocircular.
    assert_eq!(
        exact_in_circle_sign(p(1.6, 1.6), p(40.4, 1.6), p(40.4, 40.4), p(1.6, 40.4)),
        Some(0.0)
    );
}

/// What a planar face triangulation reads back from a CDT.
#[derive(Debug, PartialEq)]
struct Outcome {
    inserted: Vec<usize>,
    vertices: Vec<(u64, u64)>,
    triangles: Vec<(usize, usize, usize)>,
}

/// `cdt_triangulate_simple`'s CDT calls over a closed polygon, optionally
/// with extra free points.
fn triangulate(boundary: &[Point2], extra: &[Point2]) -> Result<Outcome, MathError> {
    let mut points = boundary.to_vec();
    points.extend_from_slice(extra);
    let (mut lo_x, mut lo_y, mut hi_x, mut hi_y) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for q in &points {
        lo_x = lo_x.min(q.x());
        lo_y = lo_y.min(q.y());
        hi_x = hi_x.max(q.x());
        hi_y = hi_y.max(q.y());
    }
    let margin = (hi_x - lo_x).max(hi_y - lo_y) * 0.1 + 1e-6;
    let mut cdt = Cdt::with_capacity(
        (
            Point2::new(lo_x - margin, lo_y - margin),
            Point2::new(hi_x + margin, hi_y + margin),
        ),
        points.len(),
    );
    let inserted = cdt.insert_points_hilbert(&points)?;
    let n = boundary.len();
    let mut constraints = Vec::new();
    for i in 0..n {
        let (ci, cj) = (inserted[i], inserted[(i + 1) % n]);
        if ci != cj {
            cdt.insert_constraint(ci, cj)?;
            constraints.push((ci, cj));
        }
    }
    cdt.remove_exterior(&constraints);
    Ok(Outcome {
        inserted,
        vertices: cdt
            .vertices()
            .iter()
            .map(|v| (v.x().to_bits(), v.y().to_bits()))
            .collect(),
        triangles: cdt.triangles(),
    })
}

/// Fails unless the shortcut and legacy CDTs agree on `boundary` + `extra`;
/// returns the shared outcome.
fn assert_same(boundary: &[Point2], extra: &[Point2]) -> Option<Outcome> {
    let current = triangulate(boundary, extra);
    let legacy = {
        let _legacy = Legacy::set();
        triangulate(boundary, extra)
    };
    assert_eq!(
        current.is_ok(),
        legacy.is_ok(),
        "outcome differs: {current:?} vs {legacy:?}"
    );
    match (current, legacy) {
        (Ok(current), Ok(legacy)) => {
            assert_eq!(current, legacy, "boundary {boundary:?}");
            Some(current)
        }
        _ => None,
    }
}

/// A star-shaped polygon around `(cx, cy)` with `n` vertices.
fn star(rng: &mut Rng, n: usize, cx: f64, cy: f64, decimals: i32) -> Vec<Point2> {
    #[allow(clippy::cast_precision_loss)]
    (0..n)
        .map(|i| {
            let t = std::f64::consts::TAU * i as f64 / n as f64;
            let r = rng.range(2.0, 20.0);
            let round = |v: f64| {
                let s = 10_f64.powi(decimals);
                (v * s).round() / s
            };
            Point2::new(round(cx + r * t.cos()), round(cy + r * t.sin()))
        })
        .collect()
}

#[test]
fn small_cdts_match_the_legacy_predicates() {
    let mut rng = Rng(0xC0FF_EE00);
    let p = Point2::new;
    for _ in 0..400 {
        // Rectangles: the cocircular case the integer sign exists for.
        let (x0, y0) = (rng.decimal(-200.0, 200.0, 1), rng.decimal(-200.0, 200.0, 2));
        let (w, h) = (rng.decimal(0.5, 60.0, 1), rng.decimal(0.5, 60.0, 3));
        assert_same(
            &[p(x0, y0), p(x0 + w, y0), p(x0 + w, y0 + h), p(x0, y0 + h)],
            &[],
        );
        // An L-shaped outline with collinear runs.
        assert_same(
            &[
                p(x0, y0),
                p(x0 + w, y0),
                p(x0 + w, y0 + h / 2.0),
                p(x0 + w / 2.0, y0 + h / 2.0),
                p(x0 + w / 2.0, y0 + h),
                p(x0, y0 + h),
                p(x0, y0 + h / 2.0),
            ],
            &[],
        );
        // Star polygons below and above the duplicate-scan and boundary-scan
        // thresholds, on decimal and on raw coordinates.
        for n in [5, 12, 16, 17, 31, 32, 33, 48, 90] {
            let (cx, cy) = (rng.range(-100.0, 100.0), rng.range(-100.0, 100.0));
            assert_same(&star(&mut rng, n, cx, cy, 2), &[]);
            assert_same(&star(&mut rng, n, cx, cy, 40), &[]);
        }
    }
}

#[test]
fn duplicate_lookup_matches_the_grid_across_cell_boundaries() {
    let p = Point2::new;
    let mut rng = Rng(0x0D0B_1E00);
    // Cells are 1e-5 wide. Put the query at a cell corner with near
    // duplicates in several neighbouring cells, so the grid's visiting order
    // (not just its candidate set) decides the answer.
    let mut merges = 0_usize;
    for _ in 0..300 {
        let cx = (rng.range(-1000.0, 1000.0) * 1e5).round() / 1e5;
        let cy = (rng.range(-1000.0, 1000.0) * 1e5).round() / 1e5;
        let mut cluster = Vec::new();
        for _ in 0..6 {
            cluster.push(p(cx + rng.range(-6e-9, 6e-9), cy + rng.range(-6e-9, 6e-9)));
        }
        let outline = [
            p(cx - 1.0, cy - 1.0),
            p(cx + 1.0, cy - 1.0),
            p(cx + 1.0, cy + 1.0),
            p(cx - 1.0, cy + 1.0),
        ];
        let welded = assert_same(&outline, &cluster).expect("triangulates");
        merges += welded.inserted.len() + 3 - welded.vertices.len();
        // The same cluster once the CDT has switched to the grid.
        let mut many = cluster.clone();
        for i in 0..40 {
            let t = f64::from(i) * 0.37;
            many.insert(0, p(cx + 0.5 * t.cos(), cy + 0.5 * t.sin()));
        }
        assert_same(&outline, &many);
    }
    // The clusters really do weld (so the visiting order is exercised).
    assert!(merges > 600, "merges {merges}");
}

/// Indices `insert_point` returns for `points`, inserted in order.
fn weld(points: &[Point2]) -> Vec<usize> {
    let lo = Point2::new(-2000.0, -2000.0);
    let hi = Point2::new(2000.0, 2000.0);
    let mut cdt = Cdt::new((lo, hi));
    points
        .iter()
        .map(|&q| cdt.insert_point(q).expect("locates"))
        .collect()
}

#[test]
fn duplicate_lookup_returns_the_grids_first_candidate() {
    let p = Point2::new;
    let mut rng = Rng(0x5EED_D0B1);
    let mut contested = 0_usize;
    for trial in 0..600 {
        // A centre on a cell corner, a cell edge, or inside a cell.
        let base_x = (rng.range(-1000.0, 1000.0) * 1e5).round() / 1e5;
        let base_y = (rng.range(-1000.0, 1000.0) * 1e5).round() / 1e5;
        let (cx, cy) = match trial % 3 {
            0 => (base_x, base_y),
            1 => (base_x + 5e-6, base_y),
            _ => (base_x + 5e-6, base_y + 5e-6),
        };
        // Candidates 0.55–0.95 DUP_TOL from the centre: each can weld a
        // query at the centre while staying distinct from the others.
        let mut points = Vec::new();
        for _ in 0..=rng.index(4) {
            let t = rng.range(0.0, std::f64::consts::TAU);
            let r = rng.range(0.55e-8, 0.95e-8);
            points.push(p(cx + r * t.cos(), cy + r * t.sin()));
        }
        // Queries at and around the centre.
        let queries = [
            p(cx, cy),
            p(cx + rng.range(-3e-9, 3e-9), cy + rng.range(-3e-9, 3e-9)),
        ];
        let distinct: Vec<Point2> = points.clone();
        for &q in &queries {
            let near = distinct
                .iter()
                .filter(|v| (q - **v).length_squared() < DUP_TOL * DUP_TOL)
                .count();
            if near >= 2 {
                contested += 1;
            }
        }
        points.extend(queries);
        let current = weld(&points);
        let legacy = {
            let _legacy = Legacy::set();
            weld(&points)
        };
        assert_eq!(current, legacy, "points {points:?}");
    }
    // Enough queries see two or more candidates for the order to matter.
    assert!(contested > 300, "contested {contested}");
}

#[test]
fn captured_cap_matches_the_legacy_predicates() {
    let points: Vec<Point2> = off_axis_cap::POINTS
        .iter()
        .map(|&(x, y)| Point2::new(x, y))
        .collect();
    assert_same(&points, &[]);
    assert_same(&points[..30], &points[30..]);
}

#[test]
fn decimal_rectangle_needs_no_robust_in_circle() {
    let p = Point2::new;
    let rectangle = [p(1.6, 1.6), p(40.4, 1.6), p(40.4, 40.4), p(1.6, 40.4)];
    work::take(&work::ROBUST_IN_CIRCLE);
    let current = triangulate(&rectangle, &[]).expect("triangulates");
    assert_eq!(current.triangles.len(), 2);
    assert_eq!(work::take(&work::ROBUST_IN_CIRCLE), 0);
    {
        let _legacy = Legacy::set();
        triangulate(&rectangle, &[]).expect("triangulates");
    }
    // The legacy predicate did need the adaptive stages here.
    assert!(work::take(&work::ROBUST_IN_CIRCLE) > 0);
}
