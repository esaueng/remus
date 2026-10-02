//! Closed-form oracles for the Bezier-clipping helpers (B19 F3a).
//!
//! Every assertion compares against a value derived by hand from the
//! geometry (3-4-5 triangles, dyadic windows, circle arcs whose closest
//! points and curvatures are known), never against another run of the
//! clipper. Threshold tests bracket each documented bound from both sides
//! (a case just inside and one just outside), so a mutated comparison,
//! constant or factor flips one of the pair.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use super::*;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// Degree-1 segment from `a` to `b` over `[0, 1]`.
fn line(a: Point3, b: Point3) -> NurbsCurve {
    NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![a, b], vec![1.0, 1.0]).expect("line")
}

/// Degree-1 polyline through `pts`, uniform knots over `[0, 1]`.
fn polyline(pts: &[Point3]) -> NurbsCurve {
    let spans = pts.len() - 1;
    let mut knots = vec![0.0];
    for i in 0..=spans {
        #[allow(clippy::cast_precision_loss)]
        knots.push(i as f64 / spans as f64);
    }
    knots.push(1.0);
    NurbsCurve::new(1, knots, pts.to_vec(), vec![1.0; pts.len()]).expect("polyline")
}

/// Exact rational quadratic arc of the circle `(center, r)` in the z = 0
/// plane from angle `a0` to `a1` (|a1 - a0| < pi), over `[0, 1]`. The
/// parameter midpoint maps to the mid angle (the arc is symmetric).
fn arc(center: Point3, r: f64, a0: f64, a1: f64) -> NurbsCurve {
    let h = 0.5 * (a1 - a0);
    let m = 0.5 * (a0 + a1);
    let at = |ang: f64, rad: f64| {
        p(
            center.x() + rad * ang.cos(),
            center.y() + rad * ang.sin(),
            0.0,
        )
    };
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![at(a0, r), at(m, r / h.cos()), at(a1, r)],
        vec![1.0, h.cos(), 1.0],
    )
    .expect("arc")
}

fn sub(pts: Vec<Point3>) -> SubSegment {
    let weights = vec![1.0; pts.len()];
    SubSegment { pts, weights }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn vclose(a: Vec3, b: Vec3, tol: f64) -> bool {
    (a - b).length() <= tol
}

// ---------------------------------------------------------------------------
// param_tolerance, ClipSide
// ---------------------------------------------------------------------------

/// A segment of length 4 over `[0, 1]` has speed 4, so a model tolerance
/// of 0.2 is a parameter slack of exactly 0.05; a zero-speed (collapsed)
/// segment gets no slack at all.
#[test]
fn param_tolerance_is_tolerance_over_speed() {
    let c = line(p(1.0, 2.0, 3.0), p(5.0, 2.0, 3.0));
    assert_eq!(param_tolerance(&c, 0.3, 0.2), 0.2 / 4.0);
    assert_eq!(param_tolerance(&c, 0.9, 0.5), 0.125);
    let dot = line(p(1.0, 1.0, 1.0), p(1.0, 1.0, 1.0));
    assert_eq!(param_tolerance(&dot, 0.5, 0.2), 0.0);
}

#[test]
fn clip_side_span_mid_and_local_map() {
    let c = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    let side = ClipSide::new(&c, 0.25, 1.0);
    assert_eq!(side.span(), 0.75);
    assert_eq!(side.mid(), 0.625);
    // Interior local parameters map affinely; the ends are pinned.
    assert_eq!(side.at(0.5), 0.625);
    assert_eq!(side.at(0.0), 0.25);
    assert_eq!(side.at(-0.5), 0.25);
    assert_eq!(side.at(1.0), 1.0);
    assert_eq!(side.at(1.5), 1.0);
    let n = side.narrowed(0.5, 1.0);
    assert_eq!((n.lo, n.hi), (0.625, 1.0));
    // An inverted local interval collapses to its start, never inverts.
    let inv = side.narrowed(0.5, 0.0);
    assert_eq!((inv.lo, inv.hi), (0.625, 0.625));
}

/// The floor is `span <= 64 eps max(|lo|, |hi|)`: at magnitude 1024 the
/// bound is 64 * 1024 * eps ~ 1.46e-11, so a 1e-12 window is at the floor
/// and a 1e-10 window is not.
#[test]
fn clip_side_param_floor_scales_with_magnitude() {
    let c = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    let at = |lo: f64, hi: f64| ClipSide::new(&c, lo, hi).at_param_floor();
    assert!(at(1024.0, 1024.0 + 1e-12));
    assert!(!at(1024.0, 1024.0 + 1e-10));
    assert!(at(-1024.0 - 1e-12, -1024.0));
    // Magnitude 1: the bound is 64 eps ~ 1.42e-14.
    assert!(at(1.0, 1.0 + 4.0 * f64::EPSILON));
    assert!(!at(1.0, 1.0 + 1e-12));
    // A zero window at the origin is at the floor (MIN_POSITIVE guard).
    assert!(at(0.0, 0.0));
    assert!(!at(0.0, 1e-300));
}

// ---------------------------------------------------------------------------
// SubSegment: extent, magnitude, flatness
// ---------------------------------------------------------------------------

#[test]
fn sub_segment_extent_magnitude_flatness() {
    // Box (1,1,1)-(4,5,1): diagonal (3,4,0), length 5.
    let s = sub(vec![p(1.0, 1.0, 1.0), p(3.0, 5.0, 1.0), p(4.0, 1.0, 1.0)]);
    assert_eq!(s.extent(), 5.0);
    // Chord (3,0,0); the middle point's perpendicular offset is (0,4,0).
    assert_eq!(s.flatness(), 4.0);
    let m = sub(vec![p(2.0, -7.0, 1.0), p(3.0, 5.0, -6.5)]);
    assert_eq!(m.magnitude(), 7.0);
    // Closed window (chord of length zero): flatness falls back to the
    // box diagonal, (2,2,0) here.
    let closed = sub(vec![p(1.0, 1.0, 0.0), p(3.0, 3.0, 0.0), p(1.0, 1.0, 0.0)]);
    assert_eq!(closed.flatness(), 8.0_f64.sqrt());
    // Off-axis chord: (0,0,0)-(3,4,0)-(6,8,0) is straight.
    let straight = sub(vec![p(2.0, 2.0, 2.0), p(5.0, 6.0, 2.0), p(8.0, 10.0, 2.0)]);
    assert!(straight.flatness() < 1e-14);
    // Chord along (3,4,0)/5; a point offset (4,-3,0)/5 * 10 from the chord
    // start sits at flatness 10.
    let bent = sub(vec![p(0.0, 0.0, 0.0), p(8.0, -6.0, 0.0), p(6.0, 8.0, 0.0)]);
    assert!(close(bent.flatness(), 10.0, 1e-14));
}

// ---------------------------------------------------------------------------
// fat_line_normal
// ---------------------------------------------------------------------------

/// A curved window's normal is perpendicular to its chord and points
/// toward its farthest control point, whatever the other window does.
#[test]
fn fat_line_normal_of_a_curved_window() {
    // Chord (0,-3,0)->(2,-3,0); the middle point is 2 above the chord.
    let a = sub(vec![
        p(0.0, -3.0, 0.0),
        p(1.0, -1.0, 0.0),
        p(2.0, -3.0, 0.0),
    ]);
    // `b` bulges the other way and farther: it must not be consulted.
    let b = sub(vec![
        p(0.0, -3.0, 0.0),
        p(1.0, -9.0, 5.0),
        p(2.0, -3.0, 0.0),
    ]);
    let m = fat_line_normal(&a, &b).expect("normal");
    assert!(vclose(m, Vec3::new(0.0, 1.0, 0.0), 1e-15), "{m:?}");
    // Tilted chord: (1,1,0) + t (3,4,0); the apex sits 8.4 along (-4,3,0)/5.
    let a = sub(vec![p(1.0, 1.0, 0.0), p(-3.5, 9.0, 0.0), p(4.0, 5.0, 0.0)]);
    let m = fat_line_normal(&a, &b).expect("normal");
    let want = Vec3::new(-0.8, 0.6, 0.0);
    assert!(vclose(m, want, 1e-15), "{m:?}");
    // Not unit-length by accident: the apex is far from the chord.
    assert!(close(m.length(), 1.0, 1e-15));
}

/// A straight `a` borrows the direction toward `b`'s farthest control
/// point, so the slab still separates the pair.
#[test]
fn fat_line_normal_of_a_straight_window_borrows_from_b() {
    let a = sub(vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(2.0, 0.0, 0.0)]);
    // b's control points sit 1 below and 0.5 above the line: farthest below.
    let b = sub(vec![p(0.0, -1.0, 0.0), p(1.0, 0.5, 0.0), p(2.0, -1.0, 0.0)]);
    let m = fat_line_normal(&a, &b).expect("normal");
    assert!(vclose(m, Vec3::new(0.0, -1.0, 0.0), 1e-15), "{m:?}");
}

/// Both windows on one line: the normal is `dir x axis`, with `axis` the
/// coordinate axis of the smallest |dir| component (x on ties, then y).
#[test]
fn fat_line_normal_of_two_collinear_windows_uses_the_least_axis() {
    let cases = [
        // dir (1,0,0): |x| is largest; |y| <= |z| -> axis y -> (0,0,1).
        (Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        // dir (0,0.6,0.8): x smallest -> axis x -> (0,0.8,-0.6).
        (Vec3::new(0.0, 3.0, 4.0), Vec3::new(0.0, 0.8, -0.6)),
        // dir (0.6,0.8,0): z smallest, |y| > |z| -> axis z -> (0.8,-0.6,0).
        (Vec3::new(3.0, 4.0, 0.0), Vec3::new(0.8, -0.6, 0.0)),
        // dir (0.8,0,0.6): y smallest -> axis y -> (-0.6,0,0.8).
        (Vec3::new(4.0, 0.0, 3.0), Vec3::new(-0.6, 0.0, 0.8)),
    ];
    for (d, want) in cases {
        let o = p(1.0, -2.0, 0.5);
        let a = sub(vec![o, o + d * 0.5, o + d]);
        let b = sub(vec![o + d * 0.25, o + d * 0.75]);
        let m = fat_line_normal(&a, &b).expect("normal");
        assert!(vclose(m, want, 1e-15), "dir {d:?}: {m:?}");
    }
}

/// A closed window (chord of length zero) takes the direction to its
/// farthest control point as the chord.
#[test]
fn fat_line_normal_of_a_closed_window() {
    // Closed loop at (1,1,0) reaching (4,5,0): chord direction (0.6,0.8,0);
    // the loop is straight along it, so `b` decides the side: b's
    // farthest perpendicular offset is along (0.8,-0.6,0).
    let a = sub(vec![p(1.0, 1.0, 0.0), p(4.0, 5.0, 0.0), p(1.0, 1.0, 0.0)]);
    let b = sub(vec![p(1.0, 1.0, 0.0), p(9.0, -5.0, 0.0)]);
    let m = fat_line_normal(&a, &b).expect("normal");
    assert!(vclose(m, Vec3::new(0.8, -0.6, 0.0), 1e-15), "{m:?}");
}

/// A perpendicular offset that overflows to infinity has no unit normal:
/// straight `a` at y = -1e308 borrows from `b` at y = +1e308, whose offset
/// (2e308) overflows. (A curved window with such an offset has an infinite
/// extent and is refused earlier, as a closed window with no finite chord.)
#[test]
fn fat_line_normal_refuses_an_overflowing_offset() {
    let a = sub(vec![p(0.0, -1e308, 0.0), p(1.0, -1e308, 0.0)]);
    let b = sub(vec![p(0.5, 1e308, 0.0), p(0.6, 1e308, 0.0)]);
    assert!(fat_line_normal(&a, &b).is_none());
    let a = sub(vec![
        p(0.0, -1e308, 0.0),
        p(0.5, 1e308, 0.0),
        p(1.0, -1e308, 0.0),
    ]);
    let b = sub(vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)]);
    assert!(fat_line_normal(&a, &b).is_none());
    // A window collapsed to one point has no chord either.
    let dot = sub(vec![p(2.0, 2.0, 2.0), p(2.0, 2.0, 2.0)]);
    assert!(fat_line_normal(&dot, &dot).is_none());
}

// ---------------------------------------------------------------------------
// clip_to_fat_line, convex_hull_clip, hulls
// ---------------------------------------------------------------------------

fn interval(c: Clip) -> Option<(f64, f64)> {
    match c {
        Clip::Empty => None,
        Clip::Interval(t0, t1) => Some((t0, t1)),
    }
}

/// `a` spans the slab 0 <= (P - a0).y <= 1 above y = 2; the line `b`
/// climbs from y = 1 to y = 5, so its signed distance is -1 + 4t and the
/// clip is [1/4, 1/2]; a pad of 0.4 widens it to [0.15, 0.6].
#[test]
fn clip_to_fat_line_matches_the_hand_interval() {
    let a = sub(vec![p(1.0, 2.0, 0.0), p(2.0, 3.0, 0.0), p(3.0, 2.0, 0.0)]);
    let b = sub(vec![p(1.5, 1.0, 0.0), p(1.5, 5.0, 0.0)]);
    assert_eq!(interval(clip_to_fat_line(&a, &b, 0.0)), Some((0.25, 0.5)));
    let (t0, t1) = interval(clip_to_fat_line(&a, &b, 0.4)).expect("interval");
    assert!(close(t0, 0.15, 1e-15) && close(t1, 0.6, 1e-15), "{t0} {t1}");
    // Quadratic b with distances (-1, 3, -1) at t = 0, 1/2, 1 (hull
    // vertices at i / degree): the hull rises above the lower bound (0)
    // over [1/8, 7/8] and dips below the upper bound (1) over [0, 1].
    let b = sub(vec![p(1.0, 1.0, 0.0), p(2.0, 5.0, 0.0), p(3.0, 1.0, 0.0)]);
    assert_eq!(
        interval(clip_to_fat_line(&a, &b, 0.0)),
        Some((0.125, 0.875))
    );
    // A rational b: weights scale the distance numerators, not their signs,
    // so the line's crossing moves to where w0 (1 - t)(-1) + w1 t (3) = 0:
    // with w = (3, 1) that is t = 1/2 for the lower bound.
    let b = SubSegment {
        pts: vec![p(1.5, 1.0, 0.0), p(1.5, 5.0, 0.0)],
        weights: vec![3.0, 1.0],
    };
    // Upper bound: 3 (1 - t)(-2) + t (2) = 0 at t = 3/4.
    assert_eq!(interval(clip_to_fat_line(&a, &b, 0.0)), Some((0.5, 0.75)));
}

/// A window that only touches the slab boundary clips to one point, and
/// one entirely outside is empty.
#[test]
fn clip_to_fat_line_touching_and_disjoint() {
    let a = sub(vec![p(0.0, 0.0, 0.0), p(1.0, 1.0, 0.0), p(2.0, 0.0, 0.0)]);
    // Distances 1 + 2t: inside only at t = 0.
    let touch = sub(vec![p(0.5, 1.0, 0.0), p(0.5, 3.0, 0.0)]);
    assert_eq!(
        interval(clip_to_fat_line(&a, &touch, 0.0)),
        Some((0.0, 0.0))
    );
    let above = sub(vec![p(0.5, 1.5, 0.0), p(0.5, 3.0, 0.0)]);
    assert!(interval(clip_to_fat_line(&a, &above, 0.0)).is_none());
    let below = sub(vec![p(0.5, -3.0, 0.0), p(0.7, -0.5, 0.0)]);
    assert!(interval(clip_to_fat_line(&a, &below, 0.0)).is_none());
}

#[test]
fn convex_hull_clip_hand_cases() {
    // Two-point hull from (0.5,-1) to (1,3): d = 0 at t = 0.625, d = 1 at
    // t = 0.75.
    assert_eq!(
        convex_hull_clip(&[(0.5, -1.0), (1.0, 3.0)], 0.0, 1.0),
        Some((0.625, 0.75))
    );
    // Start vertex inside the band: [0, crossing of d = 1 at t = 0.2].
    assert_eq!(
        convex_hull_clip(&[(0.0, 0.5), (1.0, 3.0)], 0.0, 1.0),
        Some((0.0, 0.2))
    );
    // Touching the upper bound only at t = 0: a single-point interval.
    assert_eq!(
        convex_hull_clip(&[(0.0, 1.0), (1.0, 3.0)], 0.0, 1.0),
        Some((0.0, 0.0))
    );
    // Entirely above the band: no overlap.
    assert_eq!(convex_hull_clip(&[(0.0, 2.0), (1.0, 3.0)], 0.0, 1.0), None);
    // Entirely below.
    assert_eq!(
        convex_hull_clip(&[(0.0, -2.0), (1.0, -3.0)], 0.0, 1.0),
        None
    );
    // A descending edge crossing both bounds: d = 1 at 1/8, d = 0 at 1/4.
    assert_eq!(
        convex_hull_clip(&[(0.0, 1.5), (1.0, -2.5)], 0.0, 1.0),
        Some((0.125, 0.375))
    );
    // Hull vertex on the band between outside vertices: (0.5, 0); the
    // lower chain crosses d = 1 at 3/8 and 5/8.
    assert_eq!(
        convex_hull_clip(&[(0.0, 4.0), (0.5, 0.0), (1.0, 4.0)], 0.0, 1.0),
        Some((0.375, 0.625))
    );
    // A sub-1e-30 edge touching a bound counts its whole t-range: the edge
    // from (0, -1e-31) to (1, 5e-31) is feasible over [0, 1], not from the
    // interpolated crossing at 1/6.
    assert_eq!(
        convex_hull_clip(&[(0.0, -1e-31), (1.0, 5e-31)], 0.0, 1.0),
        Some((0.0, 1.0))
    );
    // At |dd| == 1e-30 exactly the edge is no longer treated as lying on
    // the bound: the crossing of d = 0 is interpolated at 1e-36 / 1e-30.
    let (d0, d1) = (-1e-36, 1e-30 - 1e-36);
    assert_eq!(d1 - d0, 1e-30);
    let (lo, hi) = convex_hull_clip(&[(0.0, d0), (1.0, d1)], 0.0, 1.0).expect("feasible");
    assert!(close(lo, 1e-6, 1e-18) && hi == 1.0, "{lo} {hi}");
}

#[test]
fn hulls_and_cross_product_hand_cases() {
    assert_eq!(cross_2d((1.0, 2.0), (4.0, 3.0), (2.0, 7.0)), 14.0);
    assert_eq!(cross_2d((1.0, 2.0), (2.0, 7.0), (4.0, 3.0)), -14.0);
    let tent = [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)];
    assert_eq!(upper_hull(&tent), tent.to_vec());
    assert_eq!(lower_hull(&tent), vec![(0.0, 0.0), (1.0, 0.0)]);
    let vee = [(0.0, 1.0), (0.5, -1.0), (1.0, 1.0)];
    assert_eq!(upper_hull(&vee), vec![(0.0, 1.0), (1.0, 1.0)]);
    assert_eq!(lower_hull(&vee), vee.to_vec());
    // Collinear middle points are dropped from both chains.
    let ramp = [(0.0, 0.0), (0.25, 0.5), (0.5, 1.0), (1.0, 2.0)];
    assert_eq!(upper_hull(&ramp), vec![(0.0, 0.0), (1.0, 2.0)]);
    assert_eq!(lower_hull(&ramp), vec![(0.0, 0.0), (1.0, 2.0)]);
    // Four points, one reflex on each side.
    let zig = [(0.0, 0.0), (1.0 / 3.0, 3.0), (2.0 / 3.0, -3.0), (1.0, 0.0)];
    assert_eq!(
        upper_hull(&zig),
        vec![(0.0, 0.0), (1.0 / 3.0, 3.0), (1.0, 0.0)]
    );
    assert_eq!(
        lower_hull(&zig),
        vec![(0.0, 0.0), (2.0 / 3.0, -3.0), (1.0, 0.0)]
    );
}

// ---------------------------------------------------------------------------
// project_onto_window, tangent_and_curvature
// ---------------------------------------------------------------------------

/// The closest point of a circle to an outside point lies on the ray from
/// the centre: distance |p - c| - r, polished past the 1/16 sampling.
#[test]
fn project_onto_window_matches_the_radial_foot() {
    let c = p(1.0, -2.0, 0.0);
    let quarter = arc(c, 2.0, 0.0, std::f64::consts::FRAC_PI_2);
    let side = ClipSide::new(&quarter, 0.0, 1.0);
    let ang = 0.5_f64; // radians, off every sample angle
    let q = p(c.x() + 5.0 * ang.cos(), c.y() + 5.0 * ang.sin(), 0.0);
    let (u, d) = project_onto_window(side, q);
    assert!(close(d, 3.0, 1e-13), "distance {d}");
    // The foot itself is only resolved to ~sqrt(eps): the iteration stops
    // once the distance stops improving, and the distance is flat (second
    // order) at the foot.
    let foot = quarter.evaluate(u);
    let want = p(c.x() + 2.0 * ang.cos(), c.y() + 2.0 * ang.sin(), 0.0);
    assert!((foot - want).length() < 1e-7, "{foot:?}");
    // Restricted to a window that excludes the foot: the nearer window end.
    let left = ClipSide::new(&quarter, 0.5, 1.0);
    let (u, d) = project_onto_window(left, q);
    assert_eq!(u, 0.5);
    assert!(close(d, (quarter.evaluate(0.5) - q).length(), 1e-15));
    let right = ClipSide::new(&quarter, 0.0, 0.0625);
    let (u, _) = project_onto_window(right, q);
    assert_eq!(u, 0.0625);
    // A point on the curve projects onto itself at zero distance.
    let on = quarter.evaluate(0.3);
    let (u, d) = project_onto_window(side, on);
    assert!(close(u, 0.3, 1e-12) && d < 1e-14, "{u} {d}");
}

/// A circle of radius r has unit tangent perpendicular to the radius and
/// curvature vector of length 1/r pointing at the centre, whatever the
/// rational parameter speed; a collapsed curve has neither.
#[test]
fn tangent_and_curvature_of_a_circle() {
    let c = p(3.0, 1.0, 0.0);
    let r = 2.0;
    let quarter = arc(c, r, 0.0, std::f64::consts::FRAC_PI_2);
    for u in [0.0, 0.2, 0.5, 0.9] {
        let q = quarter.evaluate(u);
        let radial = (q - c) * (1.0 / r);
        let (t, k) = tangent_and_curvature(&quarter, u).expect("regular");
        assert!(close(t.length(), 1.0, 1e-14));
        // Counter-clockwise travel: tangent = z x radial.
        let want_t = Vec3::new(-radial.y(), radial.x(), 0.0);
        assert!(vclose(t, want_t, 1e-14), "u {u}: {t:?}");
        assert!(vclose(k, radial * (-1.0 / r), 1e-13), "u {u}: {k:?}");
    }
    let l = line(p(0.0, 0.0, 0.0), p(3.0, 4.0, 0.0));
    let (t, k) = tangent_and_curvature(&l, 0.3).expect("regular");
    assert!(vclose(t, Vec3::new(0.6, 0.8, 0.0), 1e-15));
    assert!(k.length() == 0.0);
    let dot = line(p(1.0, 1.0, 1.0), p(1.0, 1.0, 1.0));
    assert!(tangent_and_curvature(&dot, 0.5).is_none());
}

// ---------------------------------------------------------------------------
// newton_refine
// ---------------------------------------------------------------------------

/// Two crossing lines: one Gauss-Newton step lands on the crossing.
#[test]
fn newton_refine_lands_on_a_line_crossing() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = line(p(1.0, -1.0, 0.0), p(1.0, 3.0, 0.0));
    let hit = newton_refine(&a, &b, 0.9, 0.9, 1e-9).expect("hit");
    assert!(
        close(hit.u1, 0.25, 1e-15) && close(hit.u2, 0.25, 1e-15),
        "{hit:?}"
    );
    assert!((hit.point - p(1.0, 0.0, 0.0)).length() < 1e-15);
    // Already at the root: nothing to polish, and the start is returned.
    let hit = newton_refine(&a, &b, 0.25, 0.25, 1e-9).expect("hit");
    assert_eq!((hit.u1, hit.u2), (0.25, 0.25));
}

/// Skew lines 0.5 apart: the polished gap is exactly 0.5, accepted only
/// when the tolerance admits it.
#[test]
fn newton_refine_accepts_by_the_polished_gap() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = line(p(1.0, -1.0, 0.5), p(1.0, 3.0, 0.5));
    assert!(newton_refine(&a, &b, 0.9, 0.9, 0.49).is_none());
    let hit = newton_refine(&a, &b, 0.9, 0.9, 0.51).expect("within");
    assert!(
        close(hit.u1, 0.25, 1e-15) && close(hit.u2, 0.25, 1e-15),
        "{hit:?}"
    );
    assert!((hit.point - p(1.0, 0.0, 0.0)).length() < 1e-15);
}

/// The crossing of the carriers lies beyond `a`'s end: the step is
/// clamped to the domain, and the gap there (1) decides acceptance.
#[test]
fn newton_refine_clamps_to_the_domain() {
    let a = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    let b = line(p(2.0, -1.0, 0.0), p(2.0, 1.0, 0.0));
    let hit = newton_refine(&a, &b, 0.5, 0.25, 1.01).expect("within");
    assert_eq!(hit.u1, 1.0);
    assert!(close(hit.u2, 0.5, 1e-15), "{hit:?}");
    assert!(newton_refine(&a, &b, 0.5, 0.25, 0.99).is_none());
}

/// Parallel tangents (a tangent contact) stop the iteration at once: the
/// start pair is the answer, accepted by its own gap.
#[test]
fn newton_refine_keeps_the_start_on_parallel_tangents() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = line(p(0.0, 0.1, 0.0), p(4.0, 0.1, 0.0));
    let hit = newton_refine(&a, &b, 0.3, 0.3, 0.2).expect("within");
    assert_eq!((hit.u1, hit.u2), (0.3, 0.3));
    assert!(newton_refine(&a, &b, 0.3, 0.3, 0.05).is_none());
    // A collapsed curve has no tangent at all (j11 = 0): no step either.
    let dot = line(p(1.0, 0.1, 0.0), p(1.0, 0.1, 0.0));
    let hit = newton_refine(&dot, &a, 0.5, 0.2, 0.25).expect("within");
    assert_eq!((hit.u1, hit.u2), (0.5, 0.2));
}

/// Curved crossing: the polish reaches the model-scale floating-point
/// floor, far below the acceptance tolerance (B10).
#[test]
fn newton_refine_polishes_an_arc_crossing_to_the_floor() {
    // Unit circle and the line x = 0.6: crossing at (0.6, 0.8).
    let circle = arc(p(0.0, 0.0, 0.0), 1.0, 0.0, std::f64::consts::FRAC_PI_2);
    let l = line(p(0.6, -1.0, 0.0), p(0.6, 2.0, 0.0));
    let hit = newton_refine(&circle, &l, 0.6, 0.5, 1e-3).expect("hit");
    assert!((hit.point - p(0.6, 0.8, 0.0)).length() < 2e-14, "{hit:?}");
    assert!(close(hit.u2, 0.6, 2e-14), "{hit:?}");
}

// ---------------------------------------------------------------------------
// same_contact, merge_duplicate_hits, merge_overlaps, shared_window
// ---------------------------------------------------------------------------

fn hit_on(c1: &NurbsCurve, u1: f64, u2: f64) -> CurveCurveHit {
    CurveCurveHit {
        u1,
        u2,
        point: c1.evaluate(u1),
    }
}

/// Two hits 2 apart on parallel lines 0.01 apart (different parameter
/// speeds) are one contact exactly when the curves stay within tolerance
/// at the interpolated parameters between them.
#[test]
fn same_contact_follows_the_stretch_between_hits() {
    let c1 = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let c2 = line(p(-4.0, 0.01, 0.0), p(4.0, 0.01, 0.0));
    // x = 1 at (0.25, 5/8), x = 3 at (0.75, 7/8).
    let h = hit_on(&c1, 0.25, 0.625);
    let k = hit_on(&c1, 0.75, 0.875);
    assert!(same_contact(&c1, &c2, &h, &k, 0.011));
    assert!(!same_contact(&c1, &c2, &h, &k, 0.009));
    // Crossing lines: the curves part between the hits, but hits whose
    // points lie within tolerance are still one contact.
    let x = line(p(0.0, -4.0, 0.0), p(4.0, 4.0, 0.0));
    let h = CurveCurveHit {
        u1: 0.5,
        u2: 0.25,
        point: p(2.0, 0.0, 0.0),
    };
    let k = CurveCurveHit {
        u1: 0.6,
        u2: 0.9,
        point: p(2.3, 0.0, 0.0),
    };
    assert!(same_contact(&c1, &x, &h, &k, 0.31));
    assert!(!same_contact(&c1, &x, &h, &k, 0.29));
}

/// Hits are sorted by `u1`, same contacts merge to the representative with
/// the smallest gap between the curves, distinct contacts stay apart.
#[test]
fn merge_duplicate_hits_keeps_the_tightest_representative() {
    let c1 = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let c2 = line(p(1.0, -1.0, 0.0), p(1.0, 3.0, 0.0));
    // The true crossing is (0.25, 0.25). Two approximations of it (gaps
    // 0.02 and 0.004) and a far, unrelated point.
    let mut hits = vec![
        hit_on(&c1, 0.75, 0.75),
        hit_on(&c1, 0.255, 0.25),
        hit_on(&c1, 0.251, 0.25),
    ];
    merge_duplicate_hits(&mut hits, &c1, &c2, 0.05);
    assert_eq!(hits.len(), 2, "{hits:?}");
    assert_eq!((hits[0].u1, hits[0].u2), (0.251, 0.25));
    assert_eq!((hits[1].u1, hits[1].u2), (0.75, 0.75));
    // The tighter hit replaces an earlier, looser one (gap 0.002 < 0.02).
    let mut hits = vec![hit_on(&c1, 0.2505, 0.25), hit_on(&c1, 0.245, 0.25)];
    merge_duplicate_hits(&mut hits, &c1, &c2, 0.05);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].u1, 0.2505);
    // A lone hit is untouched.
    let mut one = vec![hit_on(&c1, 0.6, 0.1)];
    merge_duplicate_hits(&mut one, &c1, &c2, 0.05);
    assert_eq!((one[0].u1, one[0].u2), (0.6, 0.1));
}

/// Overlaps on `curve1` (speed 4) merge across a gap below the parameter
/// slack `tolerance / 4` and stay apart across a larger one.
#[test]
fn merge_overlaps_joins_within_the_parameter_slack() {
    let c1 = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let ov = |a: f64, b: f64, c: f64, d: f64| CurveCurveOverlap {
        u1_start: a,
        u1_end: b,
        u2_start: c,
        u2_end: d,
    };
    let mut overlaps = vec![
        ov(0.6, 0.7, 0.9, 0.95),
        ov(0.39, 0.45, 0.05, 0.3),
        ov(0.1, 0.3, 0.2, 0.4),
    ];
    // tolerance 0.4 -> slack 0.1: gap 0.09 merges, gap 0.15 does not.
    merge_overlaps(&mut overlaps, &c1, 0.4);
    assert_eq!(overlaps.len(), 2);
    let got: Vec<_> = overlaps
        .iter()
        .map(|o| (o.u1_start, o.u1_end, o.u2_start, o.u2_end))
        .collect();
    assert_eq!(got, vec![(0.1, 0.45, 0.05, 0.4), (0.6, 0.7, 0.9, 0.95)]);
    // A contained interval does not shrink its host.
    let mut nested = vec![ov(0.1, 0.8, 0.1, 0.8), ov(0.2, 0.3, 0.2, 0.3)];
    merge_overlaps(&mut nested, &c1, 0.0);
    assert_eq!(nested.len(), 1);
    assert_eq!((nested[0].u1_end, nested[0].u2_end), (0.8, 0.8));
}

#[test]
fn shared_window_spans_the_given_parameters() {
    let c = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    let side = ClipSide::new(&c, 0.2, 0.8);
    let w = shared_window(side, &[0.5, 0.3, 0.6]).expect("window");
    assert_eq!((w.lo, w.hi), (0.3, 0.6));
    // Parameters past the window are cut back to it.
    let w = shared_window(side, &[0.1, 0.9]).expect("window");
    assert_eq!((w.lo, w.hi), (0.2, 0.8));
    // A single parameter (or none) spans nothing.
    assert!(shared_window(side, &[0.5, 0.5]).is_none());
    assert!(shared_window(side, &[]).is_none());
}

// ---------------------------------------------------------------------------
// curve_curve_intersect_full: overlap-hit filtering
// ---------------------------------------------------------------------------

/// `c2` runs along `c1` over x in [1, 3] (u1 in [0.25, 0.75]) and then
/// crosses it four more times: at x = 2.5 (inside the overlap, removed),
/// at x = 3.25 (outside, kept), and at x = 0.9995 and x = 3.0005 (outside
/// by 0.0005 in model space, inside the parameter slack tolerance / speed
/// at either end, removed).
#[test]
fn hits_inside_an_overlap_are_removed_and_others_kept() {
    let c1 = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let c2 = polyline(&[
        p(1.0, 0.0, 0.0),
        p(3.0, 0.0, 0.0),
        p(3.0, 1.0, 0.0),
        p(2.0, -1.0, 0.0),
        p(4.5, 1.0, 0.0),
        p(4.5, 2.0, 0.0),
        p(0.9995, 2.0, 0.0),
        p(0.9995, -1.0, 0.0),
        p(0.9995, -2.0, 0.0),
        p(3.0005, -2.0, 0.0),
        p(3.0005, 2.0, 0.0),
    ]);
    let tol = 1e-3;
    let r = curve_curve_intersect_full(&c1, &c2, tol).expect("intersect");
    assert_eq!(r.overlaps.len(), 1, "{:?}", r.overlaps);
    let o = r.overlaps[0];
    assert!(
        close(o.u1_start, 0.25, 1e-9) && close(o.u1_end, 0.75, 1e-9),
        "{o:?}"
    );
    // The only survivor is the crossing at x = 3.25 (u1 = 0.8125).
    assert_eq!(r.hits.len(), 1, "{:?}", r.hits);
    assert!(close(r.hits[0].u1, 0.8125, 1e-12), "{:?}", r.hits);
    assert!((r.hits[0].point - p(3.25, 0.0, 0.0)).length() < 1e-12);
}

// ---------------------------------------------------------------------------
// coincident_to_second_order, check_overlap, check_overlap_aligned,
// tolerance_contact
// ---------------------------------------------------------------------------

fn out(tolerance: f64) -> ClipOutput {
    ClipOutput {
        tolerance,
        hits: Vec::new(),
        overlaps: Vec::new(),
    }
}

/// Internally tangent arcs of radii 1 and 1.25 touching at (1, 0), both
/// symmetric about the contact: the tangents agree, the curvature vectors
/// differ by 0.2, and the larger arc's control box (diagonal
/// 1.25 sqrt(2.5)) is the size, so the second-order gap is
/// 0.2 * 1.25^2 * 2.5 / 8. The test passes just above it and fails just
/// below it.
#[test]
fn coincident_to_second_order_brackets_the_curvature_gap() {
    use std::f64::consts::FRAC_PI_4;
    let a = arc(p(0.0, 0.0, 0.0), 1.0, -FRAC_PI_4, FRAC_PI_4);
    let b = arc(p(-0.25, 0.0, 0.0), 1.25, -FRAC_PI_4, FRAC_PI_4);
    let gap = 0.2 * 1.25 * 1.25 * 2.5 / 8.0;
    let sa = ClipSide::new(&a, 0.0, 1.0);
    let sb = ClipSide::new(&b, 0.0, 1.0);
    assert!(coincident_to_second_order(sa, sb, 1.1 * gap));
    assert!(!coincident_to_second_order(sa, sb, 0.9 * gap));
    // The same arc against itself is coincident at any tolerance.
    assert!(coincident_to_second_order(sa, sa, 1e-12));
}

/// Lines crossing at angle phi through (1, 0): the tangent gap is
/// sin(phi) times the larger segment's extent (4 here).
#[test]
fn coincident_to_second_order_brackets_the_direction_gap() {
    let (sin, cos) = (0.003_f64, 0.003_f64.mul_add(-0.003, 1.0).sqrt());
    let a = line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0));
    let b = line(
        p(2.0f64.mul_add(-cos, 1.0), -2.0 * sin, 0.0),
        p(2.0f64.mul_add(cos, 1.0), 2.0 * sin, 0.0),
    );
    let gap = sin * 4.0;
    let sa = ClipSide::new(&a, 0.0, 1.0);
    let sb = ClipSide::new(&b, 0.0, 1.0);
    assert!(coincident_to_second_order(sa, sb, 1.1 * gap));
    assert!(!coincident_to_second_order(sa, sb, 0.9 * gap));
    // No usable differential geometry: the Hausdorff verdict stands.
    let dot = line(p(1.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    assert!(coincident_to_second_order(
        ClipSide::new(&dot, 0.0, 1.0),
        sb,
        1e-12
    ));
}

/// Parallel lines 0.01 apart with aligned windows: the sampled Hausdorff
/// distance is exactly 0.01, so the overlap is accepted for a tolerance
/// just above 0.001 and refused just below; the overlap is recorded with
/// the windows in input order.
#[test]
fn check_overlap_brackets_the_hausdorff_bound() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = line(p(0.0, 0.01, 0.0), p(4.0, 0.01, 0.0));
    let sa = ClipSide::new(&a, 0.2, 0.6);
    let sb = ClipSide::new(&b, 0.2, 0.6);
    let mut o = out(0.00101);
    assert!(check_overlap(sa, sb, false, &mut o));
    let got: Vec<_> = o
        .overlaps
        .iter()
        .map(|v| (v.u1_start, v.u1_end, v.u2_start, v.u2_end))
        .collect();
    assert_eq!(got, vec![(0.2, 0.6, 0.2, 0.6)]);
    let mut o = out(0.00099);
    assert!(!check_overlap(sa, sb, false, &mut o));
    assert!(o.overlaps.is_empty());
    // Roles swapped: the record is still (curve1, curve2). `c` is `b`'s
    // carrier at twice the speed, so x in [0.8, 2.4] is u in [0.6, 0.8].
    let c = line(p(-4.0, 0.01, 0.0), p(4.0, 0.01, 0.0));
    let sc = ClipSide::new(&c, 0.6, 0.8);
    let mut o = out(0.00101);
    assert!(check_overlap(sa, sc, true, &mut o));
    let v = o.overlaps[0];
    assert_eq!(
        (v.u1_start, v.u1_end, v.u2_start, v.u2_end),
        (0.6, 0.8, 0.2, 0.6)
    );
}

/// A window pair sharing less than 50 tolerances of arc and less than 100
/// tolerances of parameter on both sides is a point contact, not an
/// overlap. Each of the three guards is bracketed on coincident lines with
/// different parameter speeds (Hausdorff distance zero).
#[test]
fn check_overlap_short_stretch_guard() {
    let run = |a: &NurbsCurve, wa: f64, b: &NurbsCurve, wb: f64, tol: f64| {
        let mut o = out(tol);
        check_overlap(
            ClipSide::new(a, 0.0, wa),
            ClipSide::new(b, 0.0, wb),
            false,
            &mut o,
        )
    };
    let len = |l: f64| line(p(0.0, 0.0, 0.0), p(l, 0.0, 0.0));
    // Arc 1, spans 0.25 and 0.125: the arc guard decides at tol = 0.02.
    let (a, b) = (len(4.0), len(8.0));
    assert!(run(&a, 0.25, &b, 0.125, 0.0199));
    assert!(!run(&a, 0.25, &b, 0.125, 0.0201));
    // Arc 0.25, span_a 1: the span_a guard decides at tol = 0.01.
    let (a, b) = (len(0.25), len(4.0));
    assert!(run(&a, 1.0, &b, 0.0625, 0.0099));
    assert!(!run(&a, 1.0, &b, 0.0625, 0.0101));
    // Arc 0.25, span_b 1: the span_b guard decides at tol = 0.01.
    let (a, b) = (len(4.0), len(0.25));
    assert!(run(&a, 0.0625, &b, 1.0, 0.0099));
    assert!(!run(&a, 0.0625, &b, 1.0, 0.0101));
}

/// One window's samples close to the other's but not vice versa: the
/// symmetric half of the Hausdorff test refuses the pair.
#[test]
fn check_overlap_is_symmetric() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    // b is a's carrier from x = 1 to x = 1.6 (inside a's window [1, 3]).
    let b = line(p(1.0, 0.0, 0.0), p(1.6, 0.0, 0.0));
    let sa = ClipSide::new(&a, 0.25, 0.75);
    let sb = ClipSide::new(&b, 0.0, 1.0);
    let mut o = out(0.001);
    assert!(!check_overlap(sa, sb, false, &mut o));
    assert!(!check_overlap(sb, sa, false, &mut o));
    assert!(o.overlaps.is_empty());
}

/// `b` covers x in [1, 3] of `a`'s carrier, 0.05 off it; `a`'s window runs
/// past both ends. The shared stretch is found by projecting `b`'s ends
/// onto `a` and accepted when the ends lie within 10 tolerances.
#[test]
fn check_overlap_aligned_trims_to_the_shared_stretch() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = line(p(1.0, 0.05, 0.0), p(3.0, 0.05, 0.0));
    let sa = ClipSide::new(&a, 0.0, 1.0);
    let sb = ClipSide::new(&b, 0.0, 1.0);
    let mut o = out(0.0051);
    assert!(check_overlap_aligned(sa, sb, false, &mut o));
    let v = o.overlaps[0];
    assert!(
        close(v.u1_start, 0.25, 1e-12) && close(v.u1_end, 0.75, 1e-12),
        "{v:?}"
    );
    assert!(
        close(v.u2_start, 0.0, 1e-12) && close(v.u2_end, 1.0, 1e-12),
        "{v:?}"
    );
    let mut o = out(0.0049);
    assert!(!check_overlap_aligned(sa, sb, false, &mut o));
    assert!(o.overlaps.is_empty());
    // Reversed roles find the same stretch through `b`'s own ends.
    let mut o = out(0.0051);
    assert!(check_overlap_aligned(sb, sa, false, &mut o));
    let v = o.overlaps[0];
    assert!(
        close(v.u1_start, 0.0, 1e-12) && close(v.u1_end, 1.0, 1e-12),
        "{v:?}"
    );
    assert!(
        close(v.u2_start, 0.25, 1e-12) && close(v.u2_end, 0.75, 1e-12),
        "{v:?}"
    );
}

/// An arc dipping to 0.001 above a line: every sample of the arc window is
/// within its end offset 0.001 + 10 (1 - cos 0.1) of the line, so it is a
/// tangent contact for a tolerance above that and not below; the refinement
/// start is the closest sample, the arc's middle over x = 2.
#[test]
fn tolerance_contact_starts_at_the_closest_sample() {
    use std::f64::consts::FRAC_PI_2;
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = arc(
        p(2.0, 10.001, 0.0),
        10.0,
        -FRAC_PI_2 - 0.1,
        -FRAC_PI_2 + 0.1,
    );
    let reach = 10.0f64.mul_add(1.0 - 0.1_f64.cos(), 0.001);
    let sa = ClipSide::new(&a, 0.0, 1.0);
    let sb = ClipSide::new(&b, 0.0, 1.0);
    let (u, v) = tolerance_contact(sa, sb, reach * 1.01).expect("contact");
    assert!(close(u, 0.5, 1e-12) && close(v, 0.5, 1e-12), "{u} {v}");
    assert!(tolerance_contact(sa, sb, reach * 0.99).is_none());
    // Equal distances everywhere (parallel lines): the first sample wins,
    // reported in (a, b) order whichever window was within the other.
    let c = line(p(1.0, 0.01, 0.0), p(3.0, 0.01, 0.0));
    let sc = ClipSide::new(&c, 0.0, 1.0);
    let (u, v) = tolerance_contact(sa, sc, 0.02).expect("contact");
    assert!(close(u, 0.25, 1e-12) && v == 0.0, "{u} {v}");
    let (u, v) = tolerance_contact(sc, sa, 0.02).expect("contact");
    assert!(u == 0.0 && close(v, 0.25, 1e-12), "{u} {v}");
    assert!(tolerance_contact(sa, sc, 0.009).is_none());
}

// ---------------------------------------------------------------------------
// bezier_clip_recurse: end-to-end closed forms that pin the recursion's
// depth bookkeeping and overlap routing
// ---------------------------------------------------------------------------

fn on_circle(c: Point3, r: f64, deg: f64) -> Point3 {
    let a = deg.to_radians();
    p(c.x() + r * a.cos(), c.y() + r * a.sin(), 0.0)
}

/// Regression (found triaging B19 bezier-clip survivors): two arcs of one
/// circle sharing 40..80 degrees, at three scales. Curved windows never
/// clip effectively against each other and never look straight, so only
/// the deep (depth >= 8) aligned-overlap route reports the shared stretch:
/// one overlap whose ends evaluate to the 40- and 80-degree points on both
/// curves, and no point hits. The overlap test used to measure each
/// sample's distance to the other window's SAMPLES; the two arcs run at
/// different parameter speeds over the shared stretch, so it passed only
/// on windows below ~100 tolerances and returned 49 overlap fragments and
/// 55 point hits at scale 1.
#[test]
fn coincident_arcs_report_one_overlap_with_closed_form_ends() {
    for scale in [1e-3, 1.0, 1e3] {
        let c = p(0.5 * scale, -0.25 * scale, 0.0);
        let r = 2.0 * scale;
        let a = arc(c, r, 0.0_f64.to_radians(), 80.0_f64.to_radians());
        let b = arc(c, r, 40.0_f64.to_radians(), 120.0_f64.to_radians());
        let tol = 1e-7 * scale;
        let res = curve_curve_intersect_full(&a, &b, tol).expect("intersect");
        assert!(
            res.hits.is_empty(),
            "scale {scale}: {} hits, {} overlaps",
            res.hits.len(),
            res.overlaps.len()
        );
        assert_eq!(res.overlaps.len(), 1, "scale {scale}: {:?}", res.overlaps);
        let o = res.overlaps[0];
        let (p40, p80) = (on_circle(c, r, 40.0), on_circle(c, r, 80.0));
        let near = |q: Point3, w: Point3| (q - w).length() <= 10.0 * tol;
        assert!(near(a.evaluate(o.u1_start), p40), "scale {scale}: {o:?}");
        assert!(near(a.evaluate(o.u1_end), p80), "scale {scale}: {o:?}");
        assert!(near(b.evaluate(o.u2_start), p40), "scale {scale}: {o:?}");
        assert!(near(b.evaluate(o.u2_end), p80), "scale {scale}: {o:?}");
    }
}

// ---------------------------------------------------------------------------
// bezier_clip_recurse: hand-derived work counts
// ---------------------------------------------------------------------------

/// Run `f` and count the `bezier_clip_recurse` calls it made on this thread.
fn recurse_calls<T>(f: impl FnOnce() -> T) -> (T, usize) {
    RECURSE_CALLS.with(|c| c.set(0));
    let r = f();
    (r, RECURSE_CALLS.with(std::cell::Cell::get))
}

/// Perpendicular lines: B clipped against A's zero-thickness fat line
/// shrinks to the crossing (call 1), A clipped against that sliver does
/// the same (call 2), and both windows are then below tolerance (call 3,
/// which emits the hit). Three calls, one exact hit.
#[test]
fn perpendicular_lines_converge_in_three_calls() {
    let a = line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0));
    let b = line(p(0.8, -1.0, 0.0), p(0.8, 1.0, 0.0));
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&a, &b, 1e-8).expect("ok"));
    assert_eq!(calls, 3);
    assert!(r.overlaps.is_empty());
    assert_eq!(r.hits.len(), 1);
    assert!((r.hits[0].point - p(0.8, 0.0, 0.0)).length() < 1e-15);
    assert!(close(r.hits[0].u1, 0.4, 1e-15) && close(r.hits[0].u2, 0.5, 1e-15));
}

/// The parabola (2t, 4t(1 - t)) and the segment x = 1.4, y in [0.2, 1.2]:
/// the segment lies inside the parabola's fat line (y in [0, 2]), so the
/// first clip is ineffective and the second (the parabola against the
/// segment's zero-thickness line) shrinks it to t = 0.7 (call 1); the
/// segment then clips to the crossing (call 2) and both are done (call 3).
#[test]
fn curve_against_segment_takes_the_second_clip_in_three_calls() {
    let a = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![p(0.0, 0.0, 0.0), p(1.0, 2.0, 0.0), p(2.0, 0.0, 0.0)],
        vec![1.0; 3],
    )
    .expect("parabola");
    let b = line(p(1.4, 0.2, 0.0), p(1.4, 1.2, 0.0));
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&a, &b, 1e-8).expect("ok"));
    assert_eq!(calls, 3);
    assert!(r.overlaps.is_empty());
    assert_eq!(r.hits.len(), 1);
    assert!((r.hits[0].point - p(1.4, 0.84, 0.0)).length() < 1e-14);
    assert!(close(r.hits[0].u1, 0.7, 1e-14) && close(r.hits[0].u2, 0.64, 1e-14));
}

/// Identical straight windows are recognised as an overlap at depth 0,
/// before any clipping: one call, one full-span overlap, no hits.
#[test]
fn identical_lines_overlap_in_one_call() {
    let a = line(p(0.0, 0.0, 0.0), p(1.0, 1.0, 0.0));
    let b = line(p(0.0, 0.0, 0.0), p(1.0, 1.0, 0.0));
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&a, &b, 1e-8).expect("ok"));
    assert_eq!(calls, 1);
    assert!(r.hits.is_empty());
    let o: Vec<_> = r
        .overlaps
        .iter()
        .map(|v| (v.u1_start, v.u1_end, v.u2_start, v.u2_end))
        .collect();
    assert_eq!(o, vec![(0.0, 1.0, 0.0, 1.0)]);
}

/// Identical curved windows never clip each other and never look straight,
/// so the overlap is only checked once the recursion is 8 levels deep: at
/// least 9 calls along one chain (depths 0..=8), however the subdivision
/// goes. The result is one overlap spanning both arcs.
#[test]
fn identical_arcs_overlap_only_past_the_depth_gate() {
    use std::f64::consts::FRAC_PI_2;
    let a = arc(p(0.0, 0.0, 0.0), 1.0, 0.0, FRAC_PI_2);
    let b = arc(p(0.0, 0.0, 0.0), 1.0, 0.0, FRAC_PI_2);
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&a, &b, 1e-8).expect("ok"));
    assert!(calls >= 9, "calls {calls}");
    assert!(r.hits.is_empty(), "{:?}", r.hits);
    assert_eq!(r.overlaps.len(), 1, "{:?}", r.overlaps);
    let o = r.overlaps[0];
    assert!(
        close(o.u1_start, 0.0, 1e-9) && close(o.u1_end, 1.0, 1e-9),
        "{o:?}"
    );
    assert!(
        close(o.u2_start, 0.0, 1e-9) && close(o.u2_end, 1.0, 1e-9),
        "{o:?}"
    );
}

/// The depth-0 straight-overlap shortcut needs BOTH windows strictly
/// thinner than `DEGENERATE_FAT_LINE` (1e-12). A parabola bump of exactly
/// 1e-12 against its own chord is not straight, so the shortcut must not
/// fire on the first call (it would return after one call); the pair still
/// ends as one overlap.
#[test]
fn straight_overlap_shortcut_needs_flatness_strictly_below_the_bound() {
    let bump = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![p(0.0, 0.0, 0.0), p(1.0, 1e-12, 0.0), p(2.0, 0.0, 0.0)],
        vec![1.0; 3],
    )
    .expect("bump");
    let flat = line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0));
    let s = SubSegment::new(ClipSide::new(&bump, 0.0, 1.0)).expect("window");
    assert_eq!(s.flatness(), DEGENERATE_FAT_LINE);
    for (c1, c2) in [(&bump, &flat), (&flat, &bump)] {
        let (r, calls) = recurse_calls(|| curve_curve_intersect_full(c1, c2, 1e-8).expect("ok"));
        assert!(calls >= 2, "calls {calls}");
        assert!(r.hits.is_empty(), "{:?}", r.hits);
        assert_eq!(r.overlaps.len(), 1, "{:?}", r.overlaps);
    }
    // Strictly below the bound, the shortcut answers on the first call.
    let thin = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![p(0.0, 0.0, 0.0), p(1.0, 5e-13, 0.0), p(2.0, 0.0, 0.0)],
        vec![1.0; 3],
    )
    .expect("thin");
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&thin, &flat, 1e-8).expect("ok"));
    assert_eq!(calls, 1);
    assert_eq!(r.overlaps.len(), 1, "{:?}", r.overlaps);
}

/// The clip pad is 1e-12 of the coordinate magnitude. At magnitude 1e6 it
/// is 1e-6, so the first clip of the perpendicular pair leaves a window of
/// extent 2e-6, above the 1e-8 tolerance, and the three-call convergence of
/// `perpendicular_lines_converge_in_three_calls` cannot happen; the hit is
/// still exact.
#[test]
fn clip_pad_scales_with_the_coordinate_magnitude() {
    let o = 1e6;
    let a = line(p(o, o, 0.0), p(o + 2.0, o, 0.0));
    let b = line(p(o + 0.8, o - 1.0, 0.0), p(o + 0.8, o + 1.0, 0.0));
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&a, &b, 1e-8).expect("ok"));
    assert!(calls > 3, "calls {calls}");
    assert_eq!(r.hits.len(), 1, "{:?}", r.hits);
    assert!((r.hits[0].point - p(o + 0.8, o, 0.0)).length() < 1e-9);
}

/// Boundary cases of the overlap test, each exactly representable: the
/// short-stretch guards are strict (`arc == 50 tol` and `span == 100 tol`
/// are long enough), and so is the Hausdorff bound (a gap of exactly
/// `10 tol` is too far).
#[test]
fn check_overlap_bounds_are_strict() {
    assert_eq!(0.02 * 50.0, 1.0);
    assert_eq!(0.01 * 100.0, 1.0);
    assert_eq!(0.001 * 10.0, 0.01);
    let len = |l: f64| line(p(0.0, 0.0, 0.0), p(l, 0.0, 0.0));
    let run = |a: &NurbsCurve, wa: f64, b: &NurbsCurve, wb: f64, tol: f64| {
        let mut o = out(tol);
        check_overlap(
            ClipSide::new(a, 0.0, wa),
            ClipSide::new(b, 0.0, wb),
            false,
            &mut o,
        )
    };
    // arc_a == 50 tol, spans below 100 tol.
    assert!(run(&len(4.0), 0.25, &len(8.0), 0.125, 0.02));
    // span_a == 100 tol, arc and span_b below their bounds.
    assert!(run(&len(0.25), 1.0, &len(4.0), 0.0625, 0.01));
    // span_b == 100 tol.
    assert!(run(&len(4.0), 0.0625, &len(0.25), 1.0, 0.01));
    // Parallel lines exactly 0.01 = 10 tol apart are not coincident.
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let b = line(p(0.0, 0.01, 0.0), p(4.0, 0.01, 0.0));
    let mut o = out(0.001);
    assert!(!check_overlap(
        ClipSide::new(&a, 0.2, 0.6),
        ClipSide::new(&b, 0.2, 0.6),
        false,
        &mut o
    ));
}

/// The Hausdorff samples cover the window interior, not just its ends.
/// `b` is the segment y = 0, x in [0, 2], plus a quintic wiggle
/// y = u (u - 1/2)^3 (u - 1) (Bernstein ordinates 0, 1/40, -3/80, 3/80,
/// -1/40, 0): it meets the segment at both ends and agrees with it to
/// second order at the middle, but its interior samples sit 0.00432 off
/// it (at u = 1/5 and 4/5). So it is coincident within 10 tol = 0.01 and
/// not within 10 tol = 0.001.
#[test]
fn check_overlap_samples_the_window_interior() {
    let a = line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0));
    let ys = [0.0, 0.025, -0.0375, 0.0375, -0.025, 0.0];
    let b = NurbsCurve::new(
        5,
        vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        (0..6u8)
            .map(|i| p(0.4 * f64::from(i), ys[usize::from(i)], 0.0))
            .collect(),
        vec![1.0; 6],
    )
    .expect("quintic");
    assert!((b.evaluate(0.2).y() - 0.00432).abs() < 1e-15);
    let sa = ClipSide::new(&a, 0.0, 1.0);
    let sb = ClipSide::new(&b, 0.0, 1.0);
    assert!(coincident_to_second_order(sa, sb, 1e-12));
    assert!(check_overlap(sa, sb, false, &mut out(1e-3)));
    assert!(!check_overlap(sa, sb, false, &mut out(1e-4)));
}

/// A window lying exactly `tolerance` from the other still counts as
/// within it (the test is `dist > tolerance` to refuse).
#[test]
fn tolerance_contact_accepts_a_gap_of_exactly_the_tolerance() {
    let a = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let c = line(p(1.0, 0.01, 0.0), p(3.0, 0.01, 0.0));
    let (u, v) = tolerance_contact(
        ClipSide::new(&a, 0.0, 1.0),
        ClipSide::new(&c, 0.0, 1.0),
        0.01,
    )
    .expect("contact at exactly the tolerance");
    assert!(close(u, 0.25, 1e-12) && v == 0.0, "{u} {v}");
}

/// Equidistant samples keep the first: the parabola (2t, 4t(1 - t)) seen
/// from (1, -10) has its two ends equally near, Newton cannot improve
/// either (the parabola bends away), and the lower parameter is returned.
#[test]
fn project_onto_window_keeps_the_first_of_equidistant_samples() {
    let parabola = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![p(0.0, 0.0, 0.0), p(1.0, 2.0, 0.0), p(2.0, 0.0, 0.0)],
        vec![1.0; 3],
    )
    .expect("parabola");
    let (u, d) = project_onto_window(ClipSide::new(&parabola, 0.0, 1.0), p(1.0, -10.0, 0.0));
    assert_eq!(u, 0.0);
    assert!(close(d, 101.0_f64.sqrt(), 1e-14), "{d}");
}

/// A clip that keeps exactly 60% of the window is not a good clip (the
/// test is `width < CLIP_THRESHOLD`). The parabola (2t, 2t(1 - t)) has the
/// fat line 0 <= y <= 1 (plus the rounding pad); the segment x = 0.7 from
/// y = -0.3 upward is tuned (by stepping its top end one ulp at a time) so
/// that the computed clip width is exactly 0.6 (the bottom end steps
/// down by 1e-4 until such a top exists). Then call 0 goes on to
/// clip the parabola against the segment (to t = 0.35), call 1 clips the
/// segment against that sliver, and call 2 emits: three calls. Treating
/// the 0.6 clip as good recurses on the segment first and takes four.
#[test]
fn clip_keeping_exactly_sixty_percent_is_not_a_good_clip() {
    let a = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![p(0.0, 0.0, 0.0), p(1.0, 1.0, 0.0), p(2.0, 0.0, 0.0)],
        vec![1.0; 3],
    )
    .expect("parabola");
    let sub_a = SubSegment::new(ClipSide::new(&a, 0.0, 1.0)).expect("window");
    let width = |bottom: f64, top: f64| {
        let b = line(p(0.7, bottom, 0.0), p(0.7, top, 0.0));
        let sub_b = SubSegment::new(ClipSide::new(&b, 0.0, 1.0)).expect("window");
        let pad = CLIP_NOISE_PAD * sub_a.magnitude().max(sub_b.magnitude());
        match clip_to_fat_line(&sub_a, &sub_b, pad) {
            Clip::Interval(t0, t1) => t1 - t0,
            Clip::Empty => f64::NAN,
        }
    };
    let mut found = None;
    'search: for k in 0..512 {
        let bottom = f64::from(k).mul_add(-1e-4, -0.3);
        // Pad: 1e-12 of the magnitude 2 on each side of the unit slab.
        let mut top = bottom + 4.0f64.mul_add(CLIP_NOISE_PAD, 1.0) / CLIP_THRESHOLD;
        for _ in 0..64 {
            let w = width(bottom, top);
            if w == CLIP_THRESHOLD {
                found = Some(line(p(0.7, bottom, 0.0), p(0.7, top, 0.0)));
                break 'search;
            }
            top = if w > CLIP_THRESHOLD {
                top.next_up()
            } else {
                top.next_down()
            };
        }
    }
    let b = found.expect("a segment whose clip keeps exactly 60%");
    let (r, calls) = recurse_calls(|| curve_curve_intersect_full(&a, &b, 1e-8).expect("ok"));
    assert_eq!(calls, 3);
    assert_eq!(r.hits.len(), 1, "{:?}", r.hits);
    assert!(
        (r.hits[0].point - p(0.7, 0.455, 0.0)).length() < 1e-14,
        "{:?}",
        r.hits
    );
}

/// Equal-gap duplicates keep the first in parameter order: hits at
/// u1 = 1/4 -+ 2^-9 on the line pair of the merge test are 4 * 2^-9 off
/// the crossing either way.
#[test]
fn merge_duplicate_hits_keeps_the_first_of_equal_gaps() {
    let c1 = line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    let c2 = line(p(1.0, -1.0, 0.0), p(1.0, 3.0, 0.0));
    let d = 2.0f64.powi(-9);
    let mut hits = vec![hit_on(&c1, 0.25 + d, 0.25), hit_on(&c1, 0.25 - d, 0.25)];
    merge_duplicate_hits(&mut hits, &c1, &c2, 0.05);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].u1, 0.25 - d);
}
