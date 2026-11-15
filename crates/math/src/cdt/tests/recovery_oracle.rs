//! Constraint recovery that walks before it scans builds the same
//! triangulations.
//!
//! Recovery now takes the directed walk from a constraint's first end before
//! asking `edge_exists`, stops that walk once it reaches the far end, and
//! collects the queue's crossing edges by walking the corridor along the
//! segment rather than testing every triangle. Every CDT here is built twice,
//! once as it is and once with `work::LEGACY_RECOVERY` set (the lookups in
//! their old order and the whole-triangulation scan), and the two must agree
//! vertex for vertex, triangle for triangle and constraint for constraint,
//! or fail alike.

use super::*;
use crate::cdt::work;

/// Runs constraint recovery the legacy way on this thread until dropped.
struct Legacy;

impl Legacy {
    fn set() -> Self {
        work::LEGACY_RECOVERY.with(|c| c.set(true));
        Self
    }
}

impl Drop for Legacy {
    fn drop(&mut self) {
        work::LEGACY_RECOVERY.with(|c| c.set(false));
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

    fn index(&mut self, n: usize) -> usize {
        #[allow(clippy::cast_possible_truncation)]
        let i = (self.next_u64() % n as u64) as usize;
        i
    }
}

/// What a caller reads back from a CDT, plus the recovered constraints.
#[derive(Debug, PartialEq)]
struct Outcome {
    inserted: Vec<usize>,
    vertices: Vec<(u64, u64)>,
    triangles: Vec<(usize, usize, usize)>,
    constraints: Vec<(usize, usize)>,
}

/// The work one build did.
#[derive(Debug, Default, Clone, Copy)]
struct Work {
    fan_steps: usize,
    walk_steps: usize,
    crossing_scans: usize,
}

fn take_work() -> Work {
    Work {
        fan_steps: work::take(&work::FAN_STEPS),
        walk_steps: work::take(&work::WALK_STEPS),
        crossing_scans: work::take(&work::CROSSING_SCANS),
    }
}

/// Insert `points` (Hilbert order, as the tessellators do), then each
/// constraint `(i, j)` between the inserted ids of points `i` and `j`, in
/// order; with `outer` given, remove what lies outside that closed loop.
fn build(
    points: &[Point2],
    constraints: &[(usize, usize)],
    outer: Option<std::ops::Range<usize>>,
) -> (Result<Outcome, MathError>, Work) {
    take_work();
    let result = (|| {
        let (mut lo_x, mut lo_y, mut hi_x, mut hi_y) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for q in points {
            lo_x = lo_x.min(q.x());
            lo_y = lo_y.min(q.y());
            hi_x = hi_x.max(q.x());
            hi_y = hi_y.max(q.y());
        }
        let margin = (hi_x - lo_x).max(hi_y - lo_y) * 0.01 + 1e-6;
        let mut cdt = Cdt::with_capacity(
            (
                Point2::new(lo_x - margin, lo_y - margin),
                Point2::new(hi_x + margin, hi_y + margin),
            ),
            points.len(),
        );
        let inserted = cdt.insert_points_hilbert(points)?;
        for &(i, j) in constraints {
            cdt.insert_constraint(inserted[i], inserted[j])?;
        }
        if let Some(outer) = outer {
            let n = outer.len();
            let boundary: Vec<(usize, usize)> = outer
                .clone()
                .map(|i| {
                    (
                        inserted[i],
                        inserted[outer.start + (i - outer.start + 1) % n],
                    )
                })
                .filter(|(a, b)| a != b)
                .collect();
            cdt.remove_exterior(&boundary);
        }
        let mut constraints: Vec<(usize, usize)> = cdt.constraint_edges().iter().copied().collect();
        constraints.sort_unstable();
        Ok(Outcome {
            inserted,
            vertices: cdt
                .vertices()
                .iter()
                .map(|v| (v.x().to_bits(), v.y().to_bits()))
                .collect(),
            triangles: cdt.triangles(),
            constraints,
        })
    })();
    (result, take_work())
}

/// Fails unless the current and legacy recoveries agree; returns the work
/// each did.
fn assert_same(
    what: &str,
    points: &[Point2],
    constraints: &[(usize, usize)],
    outer: Option<std::ops::Range<usize>>,
) -> (Work, Work) {
    let (current, current_work) = build(points, constraints, outer.clone());
    let (legacy, legacy_work) = {
        let _legacy = Legacy::set();
        build(points, constraints, outer)
    };
    assert_eq!(
        current.is_ok(),
        legacy.is_ok(),
        "{what}: only one recovery failed"
    );
    match (current, legacy) {
        (Ok(current), Ok(legacy)) => assert!(current == legacy, "{what}: triangulations differ"),
        (Err(current), Err(legacy)) => {
            assert_eq!(current.to_string(), legacy.to_string(), "{what}");
        }
        _ => {}
    }
    (current_work, legacy_work)
}

/// The closed loop over `range`, as constraint pairs.
fn loop_pairs(range: std::ops::Range<usize>) -> Vec<(usize, usize)> {
    let n = range.len();
    range
        .clone()
        .map(|i| (i, range.start + (i - range.start + 1) % n))
        .collect()
}

/// A cylinder wall's unwrapped chart, as `tessellate_revolved_with_holes`
/// builds it: the outline (two rims and two seams sampled only at their
/// ends and where an outline meets them), a grid of interior columns that
/// stops one column short of each seam, and `pockets` elliptical holes. The
/// seam vertices fan out to the whole column beside them. Returns the
/// points, the constraints and the outline's range.
fn wall_chart(
    width: f64,
    height: f64,
    (columns, rows): (usize, usize),
    pockets: &[(f64, f64, f64, f64, usize)],
) -> (Vec<Point2>, Vec<(usize, usize)>, std::ops::Range<usize>) {
    let p = Point2::new;
    let mut points = Vec::new();
    // Bottom rim, right seam (with one sample half-way), top rim, left seam.
    #[allow(clippy::cast_precision_loss)]
    for i in 0..columns {
        points.push(p(width * i as f64 / columns as f64, 0.0));
    }
    points.push(p(width, 0.0));
    points.push(p(width, 0.5 * height));
    #[allow(clippy::cast_precision_loss)]
    for i in (1..=columns).rev() {
        points.push(p(width * i as f64 / columns as f64, height));
    }
    points.push(p(0.0, height));
    points.push(p(0.0, 0.5 * height));
    let outer = 0..points.len();
    let mut constraints = loop_pairs(outer.clone());
    for &(cx, cy, rx, ry, n) in pockets {
        let start = points.len();
        #[allow(clippy::cast_precision_loss)]
        for i in 0..n {
            let angle = std::f64::consts::TAU * i as f64 / n as f64;
            points.push(p(rx.mul_add(angle.cos(), cx), ry.mul_add(angle.sin(), cy)));
        }
        constraints.extend(loop_pairs(start..points.len()));
    }
    #[allow(clippy::cast_precision_loss)]
    for iu in 1..columns {
        for iv in 1..rows {
            points.push(p(
                width * iu as f64 / columns as f64,
                height * iv as f64 / rows as f64,
            ));
        }
    }
    (points, constraints, outer)
}

/// The cylinder wall whose seams made recovery quadratic: thousands of
/// triangles round each seam vertex. Both recoveries build it alike, and the
/// current one walks a bounded number of triangles per point where the
/// legacy one walks each seam fan once per flip.
#[test]
fn a_wall_chart_with_sparse_seams_matches_and_walks_little() {
    let (points, constraints, outer) = wall_chart(
        6.0,
        2000.0,
        (16, 3000),
        &[
            (1.5, 700.0, 1.0, 400.0, 400),
            (4.5, 1300.0, 1.2, 500.0, 600),
        ],
    );
    let (current, legacy) = assert_same("wall chart", &points, &constraints, Some(outer));
    let walked = current.fan_steps + current.walk_steps;
    assert!(
        walked <= points.len(),
        "walked {walked} triangles for {} points ({current:?})",
        points.len()
    );
    assert!(
        legacy.fan_steps > 50 * walked,
        "the fixture no longer exercises the seam fans: {current:?} vs {legacy:?}"
    );
}

/// Random CDTs: free points, a closed outline, holes, and constraints that
/// cross one another, run through collinear runs of vertices or end on
/// another constraint, so the recovery's splits, Steiner points and queue all
/// take part. The queue's corridor walk must answer at least once, and the
/// whole-triangulation scans it saves must be the only difference in work.
#[test]
fn random_cdts_match_the_legacy_recovery() {
    let p = Point2::new;
    let mut rng = Rng(0x5EED_0CD7);
    let mut corridor_answers = 0_usize;
    let mut legacy_scans = 0_usize;
    for case in 0..300 {
        let mut points = Vec::new();
        let mut constraints = Vec::new();
        // An outline: a star around the origin.
        let n_outer = 8 + rng.index(40);
        #[allow(clippy::cast_precision_loss)]
        for i in 0..n_outer {
            let angle = std::f64::consts::TAU * i as f64 / n_outer as f64;
            let radius = rng.range(60.0, 100.0);
            points.push(p(radius * angle.cos(), radius * angle.sin()));
        }
        let outer = 0..n_outer;
        constraints.extend(loop_pairs(outer.clone()));
        // A hole.
        let start = points.len();
        let n_hole = 3 + rng.index(20);
        let (cx, cy) = (rng.range(-20.0, 20.0), rng.range(-20.0, 20.0));
        #[allow(clippy::cast_precision_loss)]
        for i in 0..n_hole {
            let angle = std::f64::consts::TAU * i as f64 / n_hole as f64;
            points.push(p(cx + 8.0 * angle.cos(), cy + 8.0 * angle.sin()));
        }
        constraints.extend(loop_pairs(start..points.len()));
        // Free points: uniform, a dense grid band (collinear rows and
        // columns) and a tight cluster.
        for _ in 0..rng.index(300) {
            points.push(p(rng.range(-50.0, 50.0), rng.range(-50.0, 50.0)));
        }
        let pitch = rng.range(0.5, 3.0);
        #[allow(clippy::cast_precision_loss)]
        for i in 0..rng.index(20) {
            for j in 0..rng.index(20) {
                points.push(p(-40.0 + pitch * i as f64, -30.0 + pitch * j as f64));
            }
        }
        for _ in 0..rng.index(40) {
            points.push(p(rng.range(30.0, 30.01), rng.range(-5.0, -4.99)));
        }
        // Free constraints between random points: they cross the hole and
        // each other, and run along the grid's rows.
        for _ in 0..rng.index(12) {
            let (i, j) = (rng.index(points.len()), rng.index(points.len()));
            if i != j {
                constraints.push((i, j));
            }
        }
        let keep_outside = case % 3 == 0;
        let what = format!("case {case}");
        let (current, legacy) = assert_same(
            &what,
            &points,
            &constraints,
            (!keep_outside).then_some(outer),
        );
        corridor_answers += legacy.crossing_scans - current.crossing_scans;
        legacy_scans += legacy.crossing_scans;
    }
    assert!(
        corridor_answers > 0,
        "the queue never ran: {legacy_scans} legacy scans"
    );
}

/// The two captured planar caps whose recovery needs the queue: the
/// U-bracket floor, in every placement its test uses, and the off-axis cap.
#[test]
fn captured_caps_match_the_legacy_recovery() {
    type Placement = fn(f64, f64) -> Point2;
    let placements: [Placement; 3] = [
        |x, y| Point2::new(x, y),
        |x, y| Point2::new(-y, x),
        |x, y| Point2::new(y, x),
    ];
    let mut constraints = Vec::new();
    for (start, end) in super::u_bracket_floor::WIRES {
        constraints.extend(loop_pairs(start..end));
    }
    let mut queued = 0;
    for place in placements {
        let points: Vec<Point2> = super::u_bracket_floor::POINTS
            .iter()
            .map(|&(x, y)| place(x, y))
            .collect();
        let (_, legacy) = assert_same("U-bracket floor", &points, &constraints, None);
        queued += legacy.crossing_scans;
    }
    assert!(queued > 0, "the U-bracket floor no longer needs the queue");

    let points: Vec<Point2> = super::off_axis_cap::POINTS
        .iter()
        .map(|&(x, y)| Point2::new(x, y))
        .collect();
    assert_same(
        "off-axis cap",
        &points,
        &super::off_axis_cap::CONSTRAINTS,
        None,
    );
}
