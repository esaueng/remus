//! Crossings of lines with steep, narrow NURBS bumps, against an independent
//! graph oracle.
//!
//! The surfaces are degree-2×2 graphs: control points at the Greville
//! abscissae of uniform clamped knots on `[0, 1]`, so `x = u` and `y = v`
//! exactly, flat at `z = 1` except for raised control points. A line's
//! crossings are then the sign changes of `h(t) = z(t) - S(x(t), y(t)).z`,
//! found by marching and bisection without any surface inversion.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use crate::nurbs::surface::NurbsSurface;
use crate::vec::{Point3, Vec3};

use super::{LineSurfaceSeedGrid, intersect_line_nurbs, intersect_line_nurbs_with_grid};

/// Degree-2 graph over `[0, 1]²` with `n × n` control points, `z = 1` except
/// `(i, j, z)` raised entries.
fn bump_graph(n: usize, raised: &[(usize, usize, f64)]) -> NurbsSurface {
    let spans = n - 2;
    let mut knots = vec![0.0; 3];
    knots.extend((1..spans).map(|k| k as f64 / spans as f64));
    knots.extend([1.0; 3]);
    let g: Vec<f64> = (0..n)
        .map(|i| 0.5 * (knots[i + 1] + knots[i + 2]))
        .collect();
    let control_points = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let z = raised
                        .iter()
                        .find(|&&(ri, rj, _)| ri == i && rj == j)
                        .map_or(1.0, |&(_, _, z)| z);
                    Point3::new(g[i], g[j], z)
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(
        2,
        2,
        knots.clone(),
        knots,
        control_points,
        vec![vec![1.0; n]; n],
    )
    .unwrap()
}

/// Line parameters of the crossings of `origin + t·dir` with the graph.
fn oracle_crossings(surface: &NurbsSurface, origin: Point3, dir: Vec3) -> Vec<f64> {
    let h = |t: f64| -> Option<f64> {
        let q = origin + dir * t;
        let inside = (0.0..=1.0).contains(&q.x()) && (0.0..=1.0).contains(&q.y());
        inside.then(|| q.z() - surface.evaluate(q.x(), q.y()).z())
    };
    let steps = 40_000;
    let (t0, t1) = (-3.0, 3.0);
    let mut roots = Vec::new();
    let mut prev: Option<(f64, f64)> = None;
    for k in 0..=steps {
        let t = t0 + (t1 - t0) * k as f64 / f64::from(steps);
        let Some(value) = h(t) else {
            prev = None;
            continue;
        };
        // Zero counts as positive, so a sample landing exactly on a crossing
        // (the flat top meets `z = 1` exactly) still brackets it once.
        if let Some((pt, pv)) = prev
            && (pv < 0.0) != (value < 0.0)
        {
            let (mut lo, mut hi) = (pt, t);
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if (h(mid).unwrap() < 0.0) == (pv < 0.0) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            roots.push(0.5 * (lo + hi));
        }
        prev = Some((t, value));
    }
    roots
}

fn line_parameters(origin: Point3, dir: Vec3, hits: &[super::IntersectionPoint]) -> Vec<f64> {
    let mut ts: Vec<f64> = hits
        .iter()
        .map(|hit| (hit.point - origin).dot(dir) / dir.dot(dir))
        .collect();
    ts.sort_by(f64::total_cmp);
    ts
}

fn assert_matches_oracle(found: &[f64], expected: &[f64], context: &str) {
    assert_eq!(
        found.len(),
        expected.len(),
        "{context}: found {found:?}, oracle {expected:?}"
    );
    for (a, b) in found.iter().zip(expected) {
        assert!(
            (a - b).abs() < 1e-7,
            "{context}: found {found:?}, oracle {expected:?}"
        );
    }
}

/// The HC-8 repro: the classifier's first base ray from inside the bump,
/// below its peak, exits through the bump's far flank. The 20×20 seed grid
/// had no seed in that crossing's Newton basin and returned only the two
/// crossings behind the origin.
#[test]
fn narrow_bump_far_flank_crossing_is_found() {
    let surface = bump_graph(30, &[(13, 13, 1.5)]);
    let peak = 12.5 / 28.0;
    let apex = surface.evaluate(peak, peak);
    assert!((apex.z() - 1.281_25).abs() < 1e-12);

    let origin = Point3::new(peak, peak, 1.1);
    let dir = Vec3::new(
        0.573_576_436_351_046,
        0.740_535_693_464_567_5,
        0.350_889_803_483_932_2,
    );
    let expected = oracle_crossings(&surface, origin, dir);
    assert_eq!(expected.len(), 3, "oracle: {expected:?}");
    assert!(
        expected[2] > 0.0,
        "one crossing ahead of the origin: {expected:?}"
    );

    let one_shot = intersect_line_nurbs(&surface, origin, dir, 20).unwrap();
    assert_matches_oracle(
        &line_parameters(origin, dir, &one_shot),
        &expected,
        "one-shot",
    );
    let grid = LineSurfaceSeedGrid::for_surface(&surface, 20);
    let seeded = intersect_line_nurbs_with_grid(&surface, &grid, origin, dir).unwrap();
    assert_matches_oracle(&line_parameters(origin, dir, &seeded), &expected, "seeded");

    // The vertical ray from the same origin already worked.
    let up = Vec3::new(0.0, 0.0, 1.0);
    let hits = intersect_line_nurbs(&surface, origin, up, 20).unwrap();
    assert_matches_oracle(
        &line_parameters(origin, up, &hits),
        &oracle_crossings(&surface, origin, up),
        "vertical",
    );
}

/// Narrow bumps of several heights, positions and widths, crossed by lines
/// from inside and around them in scattered directions: every oracle
/// crossing is reported, and nothing else.
#[test]
fn narrow_bump_crossings_match_graph_oracle() {
    let mut checked = 0usize;
    let mut crossings = 0usize;
    for (n, raised) in [
        (30, vec![(13, 13, 1.5)]),
        (30, vec![(4, 21, 2.0)]),
        (24, vec![(10, 11, 1.8), (11, 11, 1.8)]),
        (40, vec![(20, 7, 0.4)]),
        (40, vec![(9, 30, 2.5), (31, 12, 1.6)]),
    ] {
        let surface = bump_graph(n, &raised);
        let spans = (n - 2) as f64;
        for &(i, j, z) in &raised {
            let (cx, cy) = ((i as f64 - 0.5) / spans, (j as f64 - 0.5) / spans);
            let apex = surface.evaluate(cx, cy).z();
            for k in 1..=24_u32 {
                // Origins inside the bump (between the flat top and the apex)
                // and just around it; directions over the whole sphere.
                let a = f64::from(k) * 2.399_963_229_728_653;
                let r = 0.6 / spans * f64::from(k % 4);
                let level = 1.0 + (apex - 1.0) * (0.15 + 0.7 * f64::from(k % 5) / 4.0);
                let origin = Point3::new(cx + r * a.cos(), cy + r * a.sin(), level);
                let cz = 1.0 - 2.0 * (f64::from(k) - 0.5) / 24.0;
                let s = (1.0 - cz * cz).sqrt();
                let b = f64::from(k) * 3.883_222_077_450_933;
                let dir = Vec3::new(s * b.cos(), s * b.sin(), cz);
                let expected = oracle_crossings(&surface, origin, dir);
                let hits = intersect_line_nurbs(&surface, origin, dir, 20).unwrap();
                assert_matches_oracle(
                    &line_parameters(origin, dir, &hits),
                    &expected,
                    &format!("n={n} bump=({i},{j},{z}) ray {k}"),
                );
                checked += 1;
                crossings += expected.len();
            }
        }
    }
    assert!(
        checked >= 140 && crossings > checked,
        "{checked} rays, {crossings} crossings"
    );
}

/// A unit patch far from the origin: a line one unit off it is no crossing.
/// The completion thresholds once scaled with the largest coordinate, so at
/// `x = 1e12` they reached 1000 and accepted the one-unit miss as a root.
#[test]
fn far_translated_patch_reports_no_near_miss() {
    let x0 = 1e12;
    let corner = |x: f64, y: f64| Point3::new(x0 + x, y, 0.0);
    let surface = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![corner(0.0, 0.0), corner(0.0, 1.0)],
            vec![corner(1.0, 0.0), corner(1.0, 1.0)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap();
    // Parallel to the patch, one unit above it.
    let above = intersect_line_nurbs(
        &surface,
        Point3::new(x0 - 1.0, 0.5, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        20,
    )
    .unwrap();
    assert!(above.is_empty(), "{above:?}");
    // A transversal line still crosses it once, at its aim point.
    let target = Point3::new(x0 + 0.5, 0.5, 0.0);
    let dir = Vec3::new(0.3, 0.2, 1.0);
    let hits = intersect_line_nurbs(&surface, target - dir * 2.0, dir, 20).unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!((hits[0].point - target).length() < 1e-3, "{hits:?}");
}

/// A common weight factor leaves the surface unchanged, so it leaves its
/// crossings unchanged, even near the top of the `f64` range.
#[test]
fn common_weight_factor_keeps_narrow_bump_crossings() {
    let plain = bump_graph(30, &[(13, 13, 1.5)]);
    let heavy = NurbsSurface::new(
        2,
        2,
        plain.knots_u().to_vec(),
        plain.knots_v().to_vec(),
        plain.control_points().to_vec(),
        vec![vec![1.5e308; 30]; 30],
    )
    .unwrap();
    let peak = 12.5 / 28.0;
    let origin = Point3::new(peak, peak, 1.1);
    let dir = Vec3::new(
        0.573_576_436_351_046,
        0.740_535_693_464_567_5,
        0.350_889_803_483_932_2,
    );
    let expected = oracle_crossings(&plain, origin, dir);
    let hits = intersect_line_nurbs(&heavy, origin, dir, 20).unwrap();
    assert_matches_oracle(
        &line_parameters(origin, dir, &hits),
        &expected,
        "heavy weights",
    );
}
