//! Oracles for the ray-cast vote loop's exact work reductions: the polygon
//! gate and the plane groups.
//!
//! Both skip work they can prove changes no result (segment distances and
//! windings; repeated plane hits), so a broken reduction shows up either as a
//! different answer or as different work. The closed-form tests therefore pin
//! the answer and the work counts ([`crate::perf::take_ray_work`]) on
//! hand-placed hits, each gate threshold bracketed by a hit exactly at it and
//! one an ulp beyond. The differential tests compare every answer with a
//! verbatim copy of the classifier from before both, run on the same
//! collected faces before grouping.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

use super::*;
use proptest::prelude::*;
use remus_topology::explorer::{solid_faces, solid_vertices};

// ---------------------------------------------------------------------------
// The classifier before the gate, verbatim.
// ---------------------------------------------------------------------------

fn dist_to_polygon_boundary_reference(p: Point3, verts: &[Point3]) -> f64 {
    let mut best = f64::INFINITY;
    let n = verts.len();
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let ab = b - a;
        let len2 = ab.dot(ab);
        let t = if len2 > 0.0 {
            ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let foot = Point3::new(
            ab.x().mul_add(t, a.x()),
            ab.y().mul_add(t, a.y()),
            ab.z().mul_add(t, a.z()),
        );
        best = best.min((p - foot).length());
    }
    best
}

fn ray_face_crossing_reference(
    origin: Point3,
    ray_dir: Vec3,
    verts: &[Point3],
    holes: &[Vec<Point3>],
    normal: Vec3,
    d: f64,
    tol: Tolerance,
) -> (i32, bool) {
    let near = 10.0 * tol.linear;
    let denom = normal.dot(ray_dir);
    if denom.abs() < tol.angular {
        let numer = d - dot_normal_point(normal, origin);
        return (0, numer.abs() <= near);
    }
    let numer = d - dot_normal_point(normal, origin);
    let t = numer / denom;
    if t <= tol.linear {
        return (0, false);
    }
    let hit = Point3::new(
        origin.x() + ray_dir.x() * t,
        origin.y() + ray_dir.y() * t,
        origin.z() + ray_dir.z() * t,
    );
    let boundary_graze = dist_to_polygon_boundary_reference(hit, verts) <= near
        || holes
            .iter()
            .any(|h| dist_to_polygon_boundary_reference(hit, h) <= near);
    if !point_in_face_3d(hit, verts, &normal) {
        return (0, boundary_graze);
    }
    if holes.iter().any(|h| point_in_face_3d(hit, h, &normal)) {
        return (0, boundary_graze);
    }
    (1, boundary_graze)
}

/// The pre-gate crossing test of one collected face: planar faces through
/// the verbatim copy above (reading the polygons the face was built from),
/// every other face through the unchanged analytic tests.
fn crossing_reference(
    origin: Point3,
    ray_dir: Vec3,
    geom: &FaceGeom,
    tol: Tolerance,
) -> (i32, bool) {
    match geom {
        FaceGeom::Planar {
            normal, d, face, ..
        } => {
            let holes: Vec<Vec<Point3>> = face.loops.holes.iter().map(|h| h.pts.clone()).collect();
            ray_face_crossing_reference(
                origin,
                ray_dir,
                &face.loops.outer.pts,
                &holes,
                *normal,
                *d,
                tol,
            )
        }
        other => ray_geom_crossings(origin, ray_dir, other, tol),
    }
}

/// The cardinal ray directions before the shared `RAY_DIRS`.
const CARDINAL_DIRS: [Vec3; 3] = [
    Vec3::new(0.0, 0.0, 1.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
];

/// The generic ray directions before the shared `RAY_DIRS`.
const GENERIC_DIRS: [Vec3; 3] = [
    Vec3::new(
        0.447_213_595_499_957_9,
        0.547_722_557_505_166_1,
        std::f64::consts::FRAC_1_SQRT_2,
    ),
    Vec3::new(-0.5, 0.763_762_615_825_973_4, 0.408_248_290_463_863),
    Vec3::new(
        0.597_614_304_667_196_8,
        -0.377_964_473_009_227_2,
        std::f64::consts::FRAC_1_SQRT_2,
    ),
];

/// The `vote` closure of `votes_from_geoms` before the gate, verbatim but
/// for the trace output: every face, ray by ray.
fn rays_reference(
    face_data: &[FaceGeom],
    point: Point3,
    dirs: &[Vec3; 3],
    tol: Tolerance,
) -> [(bool, bool); 3] {
    let mut rays = [(false, false); 3];
    for (i, ray_dir) in dirs.iter().enumerate() {
        let mut crossings = 0i32;
        let mut suspicious = false;
        for geom in face_data {
            let (c, s) = crossing_reference(point, *ray_dir, geom, tol);
            crossings += c;
            suspicious |= s;
        }
        rays[i] = (crossings % 2 != 0, suspicious);
    }
    rays
}

/// Whether the reference vote below re-casts with the generic rays after
/// these cardinal `rays`: on a clean/suspicious conflict, or when every
/// cardinal ray is suspicious.
fn recasts(rays: [(bool, bool); 3]) -> bool {
    let suspicious = rays.iter().filter(|r| r.1).count();
    let clean_verdicts: Vec<bool> = rays.iter().filter(|r| !r.1).map(|r| r.0).collect();
    let conflict = suspicious > 0
        && !clean_verdicts.is_empty()
        && clean_verdicts.iter().all(|&v| v == clean_verdicts[0])
        && rays
            .iter()
            .filter(|r| r.1)
            .all(|r| r.0 != clean_verdicts[0]);
    conflict || suspicious == 3
}

/// `votes_from_geoms` before the gate, verbatim but for the trace output.
fn votes_reference(face_data: &[FaceGeom], point: Point3, tol: Tolerance) -> Result<u8, AlgoError> {
    if face_data.is_empty() {
        return Err(AlgoError::ClassificationFailed(
            "no face polygons collected for ray-cast".into(),
        ));
    }
    let cardinal_dirs = CARDINAL_DIRS;
    let generic_dirs = GENERIC_DIRS;
    let vote = |dirs: &[Vec3; 3]| rays_reference(face_data, point, dirs, tol);
    let count_inside = |rays: &[(bool, bool); 3]| rays.iter().filter(|r| r.0).count() as u8;
    let rays = vote(&cardinal_dirs);
    let cardinal = count_inside(&rays);
    let suspicious = rays.iter().filter(|r| r.1).count() as u8;
    let clean_verdicts: Vec<bool> = rays.iter().filter(|r| !r.1).map(|r| r.0).collect();
    let clean_vs_suspicious_conflict = suspicious > 0
        && !clean_verdicts.is_empty()
        && clean_verdicts.iter().all(|&v| v == clean_verdicts[0])
        && rays
            .iter()
            .filter(|r| r.1)
            .all(|r| r.0 != clean_verdicts[0]);
    if clean_vs_suspicious_conflict {
        let generic = vote(&generic_dirs);
        let inside = count_inside(&generic);
        if inside == 0 || inside == 3 {
            return Ok(inside);
        }
        return Ok(cardinal);
    }
    if suspicious < 3 {
        return Ok(cardinal);
    }
    let generic = vote(&generic_dirs);
    Ok(count_inside(&generic))
}

// ---------------------------------------------------------------------------
// Hand-placed faces and hits.
// ---------------------------------------------------------------------------

/// Work counts: polygon tests, face skips, segment distances, windings.
type Work = [u64; 4];

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn loop_at_z0(pts: &[(f64, f64)]) -> Vec<Point3> {
    pts.iter().map(|&(x, y)| p3(x, y, 0.0)).collect()
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<(f64, f64)> {
    vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
}

/// A face in the plane z = 0 (normal +z).
fn flat_face(outer: &[(f64, f64)], holes: &[Vec<(f64, f64)>]) -> FaceGeom {
    FaceGeom::planar(
        loop_at_z0(outer),
        holes.iter().map(|h| loop_at_z0(h)).collect(),
        Vec3::new(0.0, 0.0, 1.0),
        0.0,
    )
}

/// Cast +z from below a z = 0 face (far enough to clear `tol.linear`): the
/// hit is exactly `(x, y, 0)`. Returns the crossing result, checked against
/// the reference, and the work.
fn probe_with(face: &FaceGeom, x: f64, y: f64, tol: Tolerance) -> ((i32, bool), Work) {
    let depth = if tol.linear < 0.5 {
        1.0
    } else {
        1e3 * tol.linear
    };
    let origin = p3(x, y, -depth);
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let _ = crate::perf::take_ray_work();
    let got = ray_geom_crossings(origin, dir, face, tol);
    let w = crate::perf::take_ray_work();
    assert_eq!(
        got,
        crossing_reference(origin, dir, face, tol),
        "gate result differs from the reference at ({x:e}, {y:e})"
    );
    assert_eq!(w.votes, 0);
    (
        got,
        [w.polygon_tests, w.face_skips, w.segment_evals, w.windings],
    )
}

fn probe(face: &FaceGeom, x: f64, y: f64) -> ((i32, bool), Work) {
    probe_with(face, x, y, Tolerance::default())
}

fn near() -> f64 {
    10.0 * Tolerance::default().linear
}

/// The graze margin of a face whose largest |coordinate| is `scale`, derived
/// independently of the classifier: `2·near + 16ε·scale`.
fn margin(scale: f64) -> f64 {
    2.0 * near() + 16.0 * f64::EPSILON * scale
}

/// A tolerance whose `near` (= 10·linear) is exactly `target`.
fn tol_with_near(target: f64) -> Tolerance {
    let mut linear = target / 10.0;
    for _ in 0..8 {
        if 10.0 * linear == target {
            return Tolerance {
                linear,
                ..Tolerance::default()
            };
        }
        linear = if 10.0 * linear < target {
            linear.next_up()
        } else {
            linear.next_down()
        };
    }
    panic!("no linear tolerance gives near = {target:e}");
}

#[test]
fn axis_is_point_in_face_3d_projection_choice() {
    let cases = [
        ((0.0, 0.0, 1.0), Axis::Xy),
        ((0.0, 0.0, -1.0), Axis::Xy),
        ((1.0, 0.0, 0.0), Axis::Yz),
        ((-1.0, 0.0, 0.0), Axis::Yz),
        ((0.0, 1.0, 0.0), Axis::Xz),
        ((0.0, -1.0, 0.0), Axis::Xz),
        // Ties go to the earlier plane: z before y before x.
        ((1.0, 0.0, 1.0), Axis::Xy),
        ((0.0, 1.0, -1.0), Axis::Xy),
        ((1.0, 1.0, 1.0), Axis::Xy),
        ((-1.0, 1.0, 0.0), Axis::Xz),
        ((1.0, 0.0, 0.5), Axis::Yz),
        ((0.5, 0.0, 1.0), Axis::Xy),
        ((0.3, -0.9, 0.3), Axis::Xz),
        ((0.9, 0.3, -0.3), Axis::Yz),
        // NaN fails every comparison, as in `point_in_face_3d`.
        ((f64::NAN, 0.0, 0.0), Axis::Yz),
    ];
    for ((x, y, z), want) in cases {
        assert_eq!(Axis::of(Vec3::new(x, y, z)), want, "normal ({x}, {y}, {z})");
    }
    let p = p3(1.0, 2.0, 3.0);
    assert_eq!(Axis::Xy.project(p), Point2::new(1.0, 2.0));
    assert_eq!(Axis::Xz.project(p), Point2::new(1.0, 3.0));
    assert_eq!(Axis::Yz.project(p), Point2::new(2.0, 3.0));
}

#[test]
fn gate_coordinates_are_zero_or_inside_the_exact_range() {
    for c in [
        0.0, -0.0, GATE_MIN, -GATE_MIN, 1.0, -3.5, GATE_MAX, -GATE_MAX,
    ] {
        assert!(gate_coord(c), "{c:e} is a gate coordinate");
    }
    for c in [
        GATE_MIN.next_down(),
        -GATE_MIN.next_down(),
        GATE_MAX.next_up(),
        f64::MIN_POSITIVE,
        5e-324,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ] {
        assert!(!gate_coord(c), "{c:e} is not a gate coordinate");
    }
}

#[test]
fn outside_needs_a_definite_comparison_on_some_side() {
    let b = [1.0, 2.0, 3.0, 4.0];
    assert!(!outside(Point2::new(2.0, 3.0), b, 0.0));
    for (x, y) in [(1.0, 3.0), (3.0, 3.0), (2.0, 2.0), (2.0, 4.0)] {
        assert!(
            !outside(Point2::new(x, y), b, 0.0),
            "({x}, {y}) is on the box"
        );
    }
    for (x, y) in [(0.5, 3.0), (3.5, 3.0), (2.0, 1.5), (2.0, 4.5)] {
        assert!(
            outside(Point2::new(x, y), b, 0.0),
            "({x}, {y}) is off the box"
        );
        assert!(
            !outside(Point2::new(x, y), b, 0.5),
            "({x}, {y}) is on the grown box"
        );
    }
    assert!(!outside(
        Point2::new(f64::NAN, 9.0),
        [1.0, 2.0, 3.0, 4.0],
        f64::NAN
    ));
    assert!(!outside(Point2::new(f64::NAN, f64::NAN), b, 0.0));
}

/// The whole-face skip sits exactly at `margin` past the union box on every
/// side: a hit there still measures the one segment whose box it touches,
/// one ulp further it is settled without any work.
#[test]
fn whole_face_skip_brackets_the_graze_margin() {
    for (lo, hi) in [(1.0, 3.0), (1000.0, 1003.0)] {
        let face = flat_face(&rect(lo, lo, hi, hi), &[]);
        let m = margin(hi);
        let mid = 0.5 * (lo + hi);
        let sides = [
            ((hi + m, mid), (hi + m).next_up(), true),
            ((lo - m, mid), (lo - m).next_down(), true),
            ((mid, hi + m), (hi + m).next_up(), false),
            ((mid, lo - m), (lo - m).next_down(), false),
        ];
        for ((x, y), beyond, along_x) in sides {
            assert_eq!(
                probe(&face, x, y),
                ((0, false), [1, 0, 1, 0]),
                "at ({x}, {y})"
            );
            let (bx, by) = if along_x { (beyond, y) } else { (x, beyond) };
            assert_eq!(
                probe(&face, bx, by),
                ((0, false), [1, 1, 0, 0]),
                "beyond at ({bx}, {by})"
            );
        }
    }
}

/// Inside the union box, each segment's own box gates its distance by the
/// same margin; an interior hit clear of every segment measures none.
#[test]
fn segment_gate_brackets_the_margin_inside_the_face() {
    let face = flat_face(&rect(1.0, 1.0, 3.0, 3.0), &[]);
    let m = margin(3.0);
    assert_eq!(probe(&face, 2.0, 2.0), ((1, false), [1, 0, 0, 1]));
    for (x, y) in [
        (2.0, 1.0 + m),
        (2.0, 3.0 - m),
        (1.0 + m, 2.0),
        (3.0 - m, 2.0),
    ] {
        assert_eq!(
            probe(&face, x, y),
            ((1, false), [1, 0, 1, 1]),
            "at ({x}, {y})"
        );
    }
    for (x, y) in [
        (2.0, (1.0 + m).next_up()),
        (2.0, (3.0 - m).next_down()),
        ((1.0 + m).next_up(), 2.0),
        ((3.0 - m).next_down(), 2.0),
    ] {
        assert_eq!(
            probe(&face, x, y),
            ((1, false), [1, 0, 0, 1]),
            "at ({x}, {y})"
        );
    }
}

/// A hit exactly `near` from a segment grazes; one ulp further does not.
/// The edges are 2 long and off the origin, so the projection parameter and
/// the foot point are both exercised. The first grazing segment ends the scan.
#[test]
fn graze_is_inclusive_at_near_on_every_edge() {
    let near = 2f64.powi(-20);
    let tol = tol_with_near(near);
    let face = flat_face(&rect(1.0, 1.0, 3.0, 3.0), &[]);
    let edges = [
        (2.5, 1.0 + near),
        (3.0 - near, 1.5),
        (1.5, 3.0 - near),
        (1.0 + near, 2.5),
    ];
    for (x, y) in edges {
        assert_eq!(
            probe_with(&face, x, y, tol),
            ((1, true), [1, 0, 1, 1]),
            "at ({x}, {y})"
        );
    }
    let clear = [
        (2.5, (1.0 + near).next_up()),
        ((3.0 - near).next_down(), 1.5),
        (1.5, (3.0 - near).next_down()),
        ((1.0 + near).next_up(), 2.5),
    ];
    for (x, y) in clear {
        assert_eq!(
            probe_with(&face, x, y, tol),
            ((1, false), [1, 0, 1, 1]),
            "at ({x}, {y})"
        );
    }
    // Both edges at a corner reach the hit; the scan stops at the first.
    assert_eq!(
        probe_with(&face, 1.0 + 0.5 * near, 1.0 + 0.5 * near, tol),
        ((1, true), [1, 0, 1, 1])
    );
    // Outside the face, past an edge by exactly `near`.
    assert_eq!(
        probe_with(&face, 3.0 + near, 2.0, tol),
        ((0, true), [1, 0, 1, 0])
    );
}

/// A degenerate segment measures to its point (`t = 0`); a face collapsed to
/// one point still grazes.
#[test]
fn zero_length_segments_measure_to_their_point() {
    let point = flat_face(&[(1.0, 1.0), (1.0, 1.0), (1.0, 1.0)], &[]);
    let n = near();
    assert_eq!(probe(&point, 1.0 + 0.5 * n, 1.0), ((0, true), [1, 0, 1, 0]));
    assert_eq!(
        probe(&point, 1.0 + 1.5 * n, 1.0),
        ((0, false), [1, 0, 3, 0])
    );
    // A repeated vertex, its zero-length segment scanned first.
    let face = flat_face(
        &[(3.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0), (1.0, 1.0)],
        &[],
    );
    assert_eq!(probe(&face, 3.0 + 0.5 * n, 1.0), ((0, true), [1, 0, 1, 0]));
    assert_eq!(probe(&face, 2.0, 2.0), ((1, false), [1, 0, 0, 1]));
}

/// Holes are wound only when the hit lies on their bounding box; a hole that
/// reaches past the outer loop widens the union box so its rim still grazes.
#[test]
fn holes_are_wound_only_on_their_own_box() {
    let face = flat_face(
        &rect(0.0, 0.0, 8.0, 8.0),
        &[rect(2.0, 2.0, 4.0, 4.0), rect(5.0, 5.0, 7.0, 7.0)],
    );
    assert_eq!(probe(&face, 1.0, 1.0), ((1, false), [1, 0, 0, 1]));
    assert_eq!(probe(&face, 3.0, 6.0), ((1, false), [1, 0, 0, 1]));
    assert_eq!(probe(&face, 3.0, 3.0), ((0, false), [1, 0, 0, 2]));
    assert_eq!(probe(&face, 6.0, 6.0), ((0, false), [1, 0, 0, 2]));
    assert_eq!(probe(&face, 6.0, 3.0), ((1, false), [1, 0, 0, 1]));
    // On a hole's rim: the rim grazes and its winding is evaluated.
    assert_eq!(probe(&face, 2.0, 3.0), ((0, true), [1, 0, 1, 2]));

    let n = near();
    let spill = flat_face(&rect(0.0, 0.0, 4.0, 4.0), &[rect(3.0, 1.0, 6.0, 2.0)]);
    assert_eq!(probe(&spill, 6.0 + 0.5 * n, 1.5), ((0, true), [1, 0, 1, 0]));
    assert_eq!(probe(&spill, 5.0, 1.5), ((0, false), [1, 0, 0, 0]));
    assert_eq!(probe(&spill, 2.0, 1.5), ((1, false), [1, 0, 0, 1]));
    assert_eq!(probe(&spill, 3.5, 1.5), ((0, false), [1, 0, 0, 2]));
}

/// Faces with a coordinate outside the exact range, hits projecting to one,
/// and a `near` outside `[GATE_MIN, GATE_MAX]` all take the exact test: every
/// segment distance and the outer winding, however far the hit.
#[test]
fn faces_and_hits_off_the_exact_range_take_the_exact_test() {
    let exact_far = ((0, false), [1, 0, 4, 1]);
    for bad in [1e-150, f64::INFINITY, f64::NAN, 1e101] {
        let face = flat_face(&[(bad, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)], &[]);
        let ((c, _), w) = probe(&face, 50.0, 50.0);
        assert_eq!((c, w), (0, [1, 0, 4, 1]), "vertex coordinate {bad:e}");
        let holed = flat_face(
            &rect(0.0, 0.0, 8.0, 8.0),
            &[vec![(2.0, 2.0), (3.0, 2.0), (3.0, bad)]],
        );
        assert_eq!(
            probe(&holed, 50.0, 50.0).1,
            [1, 0, 7, 1],
            "hole coordinate {bad:e}"
        );
    }
    let face = flat_face(&rect(1.0, 1.0, 3.0, 3.0), &[]);
    assert_eq!(probe(&face, 1e-150, 50.0), exact_far);
    assert_eq!(probe(&face, 50.0, 1e-150), exact_far);
    assert_eq!(probe(&face, 50.0, 1e101), exact_far);
    assert_eq!(probe(&face, 1e101, 50.0), exact_far);
    // A coordinate inside the range on a face whose z is out of it: the third
    // coordinate of every vertex counts too.
    let tilted = FaceGeom::planar(
        vec![
            p3(1.0, 1.0, 1e-150),
            p3(3.0, 1.0, 0.0),
            p3(3.0, 3.0, 0.0),
            p3(1.0, 3.0, 0.0),
        ],
        Vec::new(),
        Vec3::new(0.0, 0.0, 1.0),
        0.0,
    );
    assert_eq!(probe(&tilted, 50.0, 50.0), exact_far);
    for linear in [0.0, 1e-142, 1e100] {
        let tol = Tolerance {
            linear,
            ..Tolerance::default()
        };
        let (_, w) = probe_with(&face, 50.0, 50.0, tol);
        assert_eq!(w, [1, 0, 4, 1], "linear tolerance {linear:e}");
    }
    // The ends of the range still gate: at the bottom the far hit is skipped
    // outright, at the top every segment is within the margin and the first
    // one already grazes.
    let (_, w) = probe_with(&face, 50.0, 50.0, tol_with_near(GATE_MIN));
    assert_eq!(w, [1, 1, 0, 0]);
    let (_, w) = probe_with(&face, 50.0, 50.0, tol_with_near(GATE_MAX));
    assert_eq!(w, [1, 0, 1, 0]);

    // The exact test grazes inclusively (here at `near = 0`), on the outer
    // loop's every edge and on a hole's rim, and measures every segment.
    let exact = Tolerance {
        linear: 0.0,
        ..Tolerance::default()
    };
    let holed = flat_face(&rect(1.0, 1.0, 3.0, 3.0), &[rect(1.5, 1.5, 2.5, 2.5)]);
    for (x, y) in [(2.0, 1.0), (1.0, 2.0)] {
        assert_eq!(
            probe_with(&holed, x, y, exact),
            ((1, true), [1, 0, 4, 2]),
            "at ({x}, {y})"
        );
    }
    for (x, y) in [(3.0, 2.0), (2.0, 3.0)] {
        assert_eq!(
            probe_with(&holed, x, y, exact),
            ((0, true), [1, 0, 4, 1]),
            "at ({x}, {y})"
        );
    }
    assert_eq!(
        probe_with(&holed, 2.0, 1.5, exact),
        ((0, true), [1, 0, 8, 2])
    );
    assert_eq!(
        probe_with(&holed, 2.0, 1.25, exact),
        ((1, false), [1, 0, 8, 2])
    );
}

// ---------------------------------------------------------------------------
// Differential: random polygons against the reference.
// ---------------------------------------------------------------------------

/// xorshift64*: a small deterministic generator for the polygon shapes.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// A 2D outline: convex, star-shaped or comb-like, roughly within [-1, 1]².
fn outline(rng: &mut Rng) -> Vec<(f64, f64)> {
    match rng.below(3) {
        0 => {
            let n = 3 + rng.below(10);
            (0..n)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / n as f64;
                    (a.cos(), a.sin())
                })
                .collect()
        }
        1 => {
            let n = 5 + rng.below(12);
            (0..n)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / n as f64;
                    let r = rng.range(0.3, 1.0);
                    (r * a.cos(), r * a.sin())
                })
                .collect()
        }
        _ => {
            // A comb: a spine along y = -1 with `teeth` upward fingers.
            let teeth = 1 + rng.below(8);
            let w = 2.0 / (2 * teeth + 1) as f64;
            let mut pts = vec![(1.0, -1.0), (1.0, -0.5)];
            for k in (0..teeth).rev() {
                let x1 = -1.0 + w * (2 * k + 2) as f64;
                let x0 = x1 - w;
                pts.extend([(x1, -0.5), (x1, 1.0), (x0, 1.0), (x0, -0.5)]);
            }
            pts.extend([(-1.0, -0.5), (-1.0, -1.0)]);
            pts
        }
    }
}

/// A random planar face in 3D with its feature points (vertices, edge points,
/// points near edges, box corners, hole interiors) to aim rays at.
fn random_face(rng: &mut Rng) -> (Vec<Point3>, Vec<Vec<Point3>>, Vec3, f64, Vec<Point3>) {
    let scale = [1e-3, 1.0, 1e3, 1e6][rng.below(4)];
    let center = [
        rng.range(-2.0, 2.0),
        rng.range(-2.0, 2.0),
        rng.range(-2.0, 2.0),
    ];
    // An axis-aligned frame most of the time, else a tilted one.
    let (u, v) = match rng.below(4) {
        0 => ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        1 => ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        2 => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        _ => {
            let a = rng.range(0.0, std::f64::consts::TAU);
            let b = rng.range(-1.2, 1.2);
            let u = [a.cos(), a.sin(), 0.0];
            let v = [-a.sin() * b.sin(), a.cos() * b.sin(), b.cos()];
            (u, v)
        }
    };
    let warp = if rng.below(4) == 0 { 1e-6 } else { 0.0 };
    let w = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let to3 = |(s, t): (f64, f64), rng: &mut Rng| {
        let h = warp * rng.range(-1.0, 1.0);
        let c = |i: usize| scale * (center[i] + s * u[i] + t * v[i] + h * w[i]);
        p3(c(0), c(1), c(2))
    };
    let outer2 = outline(rng);
    let mut holes2 = Vec::new();
    for _ in 0..rng.below(4) {
        let (cx, cy) = (rng.range(-0.8, 0.8), rng.range(-0.8, 0.8));
        let r = rng.range(0.05, 0.4);
        holes2.push(vec![
            (cx - r, cy - r),
            (cx + r, cy - r),
            (cx + r, cy + r),
            (cx - r, cy + r),
        ]);
    }
    let verts: Vec<Point3> = outer2.iter().map(|&q| to3(q, rng)).collect();
    let holes: Vec<Vec<Point3>> = holes2
        .iter()
        .map(|h| h.iter().map(|&q| to3(q, rng)).collect())
        .collect();
    let normal = if warp == 0.0 && rng.below(2) == 0 {
        Vec3::new(w[0], w[1], w[2])
    } else {
        newell_normal(&verts)
    };
    let normal = if rng.below(2) == 0 { -normal } else { normal };
    let d = dot_normal_point(normal, verts[0]);

    let near = 10.0 * Tolerance::default().linear;
    let mut targets = Vec::new();
    for ring in std::iter::once(&verts).chain(&holes) {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            targets.push(a);
            let f = rng.unit();
            let on = a + (b - a) * f;
            targets.push(on);
            // Off the edge, in the plane, by `near` give or take a part in 1e9.
            if let Ok(side) = (b - a).cross(normal).normalize() {
                for k in [1.0 - 1e-9, 1.0, 1.0 + 1e-9, 2.0, 3.0] {
                    targets.push(on + side * (near * k));
                    targets.push(on - side * (near * k));
                }
            }
        }
    }
    for h in &holes2 {
        let (cx, cy) = (0.5 * (h[0].0 + h[1].0), 0.5 * (h[0].1 + h[2].1));
        targets.push(to3((cx, cy), rng));
    }
    for _ in 0..8 {
        targets.push(to3((rng.range(-1.5, 1.5), rng.range(-1.5, 1.5)), rng));
    }
    (verts, holes, normal, d, targets)
}

fn ray_dirs() -> [Vec3; 6] {
    [
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(
            0.447_213_595_499_957_9,
            0.547_722_557_505_166_1,
            std::f64::consts::FRAC_1_SQRT_2,
        ),
        Vec3::new(-0.5, 0.763_762_615_825_973_4, 0.408_248_290_463_863),
        Vec3::new(
            0.597_614_304_667_196_8,
            -0.377_964_473_009_227_2,
            std::f64::consts::FRAC_1_SQRT_2,
        ),
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]
    #[test]
    fn gated_crossings_match_the_reference_on_random_faces(seed in any::<u64>()) {
        let mut rng = Rng(seed | 1);
        let (verts, holes, normal, d, targets) = random_face(&mut rng);
        let face = FaceGeom::planar(verts.clone(), holes.clone(), normal, d);
        let mut dirs = ray_dirs().to_vec();
        for _ in 0..2 {
            let r = Vec3::new(rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0));
            if let Ok(r) = r.normalize() {
                dirs.push(r);
            }
        }
        let tols = [
            Tolerance::default(),
            Tolerance { linear: 1e-4, ..Tolerance::default() },
            Tolerance { linear: 0.0, ..Tolerance::default() },
        ];
        let mut origins = Vec::new();
        for &target in &targets {
            for &dir in &dirs {
                origins.push((target - dir * rng.range(0.5, 2.0), dir));
            }
            // In the plane: parallel rays from the face's own plane.
            origins.push((target, Vec3::new(normal.y(), -normal.x(), 0.0)));
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            origins.push((p3(bad, 0.0, 0.0), dirs[0]));
            origins.push((p3(0.0, 0.0, bad), dirs[0]));
        }
        for tol in tols {
            for &(origin, dir) in &origins {
                prop_assert_eq!(
                    ray_geom_crossings(origin, dir, &face, tol),
                    ray_face_crossing_reference(origin, dir, &verts, &holes, normal, d, tol),
                    "origin {:?} dir {:?} linear {:e}", origin, dir, tol.linear
                );
            }
        }
    }
}

/// The reference agrees on the shapes the profiled workloads stress: a
/// comb-outlined cap with 36 square holes, and a 91-rectangle honeycomb row
/// of faces, hit near every rim and corner.
#[test]
fn gated_crossings_match_the_reference_on_workload_shapes() {
    let mut comb = vec![(100.0, 0.0), (100.0, 100.0)];
    for k in (0..28).rev() {
        let x = 100.0 * f64::from(k) / 28.0;
        comb.extend([(x + 2.0, 100.0), (x + 2.0, 96.0), (x, 96.0), (x, 100.0)]);
    }
    comb.push((0.0, 0.0));
    let holes: Vec<Vec<(f64, f64)>> = (0..36)
        .map(|k| {
            let (x, y) = (
                10.0 + 13.0 * f64::from(k % 6),
                10.0 + 13.0 * f64::from(k / 6),
            );
            rect(x, y, x + 4.0, y + 4.0)
        })
        .collect();
    let mut faces = vec![flat_face(&comb, &holes)];
    for k in 0..91 {
        let (x, y) = (8.0 * f64::from(k % 13), 8.0 * f64::from(k / 13));
        faces.push(flat_face(&rect(x, y, x + 5.196, y + 5.196), &[]));
    }
    let n = near();
    let mut hits = Vec::new();
    for x in 0..=110 {
        for y in 0..=110 {
            hits.push((f64::from(x), f64::from(y)));
        }
    }
    for k in 0..36 {
        let (x, y) = (
            10.0 + 13.0 * f64::from(k % 6),
            10.0 + 13.0 * f64::from(k / 6),
        );
        for (dx, dy) in [(0.0, 2.0), (4.0, 2.0), (2.0, 0.0), (2.0, 4.0), (0.0, 0.0)] {
            for e in [-2.0 * n, -n, -0.5 * n, 0.0, 0.5 * n, n, 2.0 * n, 1e-3] {
                hits.push((x + dx + e, y + dy));
                hits.push((x + dx, y + dy + e));
            }
        }
    }
    for face in &faces {
        for &(x, y) in &hits {
            probe(face, x, y);
        }
    }
}

// ---------------------------------------------------------------------------
// Differential: whole votes on solids.
// ---------------------------------------------------------------------------

fn halton(mut i: u64, base: u64) -> f64 {
    let (mut f, mut r) = (1.0, 0.0);
    while i > 0 {
        f /= base as f64;
        r += f * (i % base) as f64;
        i /= base;
    }
    r
}

/// Probe points for a solid: a Halton cloud over its box, every vertex, and
/// points on and just off each face's plane through its vertex centroid.
fn probe_points(topo: &Topology, solid: SolidId, count: u64) -> Vec<Point3> {
    let bbox = compute_solid_bbox(topo, solid).unwrap();
    let (lo, hi) = (bbox.min, bbox.max);
    let mut pts: Vec<Point3> = (1..=count)
        .map(|i| {
            let s = |t: f64, a: f64, b: f64| (b - a).mul_add(1.2 * t - 0.1, a);
            p3(
                s(halton(i, 2), lo.x(), hi.x()),
                s(halton(i, 3), lo.y(), hi.y()),
                s(halton(i, 5), lo.z(), hi.z()),
            )
        })
        .collect();
    for v in solid_vertices(topo, solid).unwrap() {
        pts.push(topo.vertex(v).unwrap().point());
    }
    for f in solid_faces(topo, solid).unwrap() {
        if let Some((outer, _, normal)) = planar_face_polygons(topo, f).unwrap() {
            let inv = 1.0 / outer.len() as f64;
            let c = outer.iter().fold(p3(0.0, 0.0, 0.0), |acc, q| {
                p3(
                    acc.x() + q.x() * inv,
                    acc.y() + q.y() * inv,
                    acc.z() + q.z() * inv,
                )
            });
            for off in [0.0, 1e-3, -1e-3] {
                pts.push(c + normal * off);
            }
            let q = outer[0] + (outer[1] - outer[0]) * 0.5;
            pts.push(q);
            pts.push(p3(q.x(), q.y(), c.z()));
        }
    }
    pts
}

/// Votes, both ray triples' parity and suspicion, and every face's crossing
/// for all six ray directions equal the reference's at every probe point.
/// The reference runs on the collected faces, the vote on them grouped.
/// Returns how many probes re-cast with the generic rays.
fn assert_votes_match(topo: &Topology, solid: SolidId, count: u64) -> usize {
    let flat = collect_face_geoms(topo, solid).unwrap();
    let grouped = FaceGeoms::new(collect_face_geoms(topo, solid).unwrap());
    assert_eq!(grouped.len(), flat.len());
    let tol = Tolerance::default();
    let mut recast = 0;
    for point in probe_points(topo, solid, count) {
        assert_eq!(
            votes_from_geoms(&grouped, point, tol).unwrap(),
            votes_reference(&flat, point, tol).unwrap(),
            "votes at {point:?}"
        );
        let cardinal = rays_reference(&flat, point, &CARDINAL_DIRS, tol);
        assert_eq!(cast_rays(&grouped, point, 0, tol, false), cardinal);
        assert_eq!(
            cast_rays(&grouped, point, 3, tol, false),
            rays_reference(&flat, point, &GENERIC_DIRS, tol),
            "generic rays at {point:?}"
        );
        recast += usize::from(recasts(cardinal));
        for dir in ray_dirs() {
            for geom in &flat {
                assert_eq!(
                    ray_geom_crossings(point, dir, geom, tol),
                    crossing_reference(point, dir, geom, tol),
                    "face crossing at {point:?} along {dir:?}"
                );
            }
        }
    }
    recast
}

fn boxes(topo: &mut Topology, specs: &[([f64; 3], [f64; 3])]) -> Vec<SolidId> {
    specs
        .iter()
        .map(|&(lo, hi)| super::tests::make_box(topo, lo, hi))
        .collect()
}

/// Disjoint solids as one multi-region solid (one shell of all their faces).
fn merged(topo: &mut Topology, solids: &[SolidId]) -> SolidId {
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    let mut faces = Vec::new();
    for &s in solids {
        faces.extend(solid_faces(topo, s).unwrap());
    }
    let shell = topo.add_shell(Shell::new(faces).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

#[test]
fn votes_match_the_reference_on_a_cut_strut_lattice() {
    let mut topo = Topology::default();
    let slab = super::tests::make_box(&mut topo, [0.0, 0.0, 0.0], [100.0, 100.0, 10.0]);
    let mut specs = Vec::new();
    for k in 1..=3 {
        let at = 25.0 * f64::from(k) - 2.0;
        specs.push(([0.0, at, 0.0], [100.0, at + 4.0, 10.0]));
        specs.push(([at, 0.0, 0.0], [at + 4.0, 100.0, 10.0]));
    }
    let struts = boxes(&mut topo, &specs);
    let lattice = crate::gfa::fuse_n(&mut topo, &struts).unwrap();
    let a = assert_votes_match(&topo, lattice, 300);
    let cut = crate::gfa::boolean(&mut topo, crate::bop::BooleanOp::Cut, slab, lattice).unwrap();
    let b = assert_votes_match(&topo, cut, 300);
    // Votes re-cast with the generic rays here, so the generic directions'
    // shared plane denominators decide votes (both triples are also compared
    // ray by ray at every probe).
    assert!(a + b > 0, "no probe re-cast with the generic rays");
}

#[test]
fn votes_match_the_reference_on_a_perforated_slab_and_a_box_field() {
    let mut topo = Topology::default();
    let slab = super::tests::make_box(&mut topo, [0.0, 0.0, 1.0], [9.6, 9.6, 3.0]);
    let mut specs = Vec::new();
    for j in 1..=3 {
        for i in 1..=3 {
            let (x, y) = (2.4 * f64::from(i), 2.4 * f64::from(j));
            specs.push(([x, y, 0.0], [x + 1.0, y + 1.0, 4.0]));
        }
    }
    let holes = boxes(&mut topo, &specs);
    let tool = merged(&mut topo, &holes);
    let cut = crate::gfa::boolean(&mut topo, crate::bop::BooleanOp::Cut, slab, tool).unwrap();
    assert_votes_match(&topo, cut, 300);

    let mut specs = Vec::new();
    for j in 0..4 {
        for i in 0..4 {
            let (x, y) = (8.0 * f64::from(i), 8.0 * f64::from(j));
            specs.push(([x, y, -5.0], [x + 5.196, y + 5.196, 15.0]));
        }
    }
    let field = boxes(&mut topo, &specs);
    let field = merged(&mut topo, &field);
    assert_votes_match(&topo, field, 300);
}

#[test]
fn votes_match_the_reference_on_mixed_planar_and_curved_solids() {
    let mut topo = Topology::default();
    for seam in [0.25, 2.5] {
        let cyl = super::tests::make_seamed_cylinder(&mut topo, seam);
        assert_votes_match(&topo, cyl, 200);
    }
    let a = super::tests::make_box(&mut topo, [0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b = super::tests::make_box(&mut topo, [1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
    let fused = crate::gfa::boolean(&mut topo, crate::bop::BooleanOp::Fuse, a, b).unwrap();
    assert_votes_match(&topo, fused, 200);
}

/// The scaling-guard shape in miniature: on a field of 16 disjoint boxes a
/// vote measures almost no segments and settles most polygon tests from the
/// face box alone; without the gate every polygon test measured all four
/// edges and wound the outer loop.
#[test]
fn a_vote_on_a_box_field_measures_few_segments() {
    let mut topo = Topology::default();
    let mut specs = Vec::new();
    for j in 0..4 {
        for i in 0..4 {
            let (x, y) = (8.0 * f64::from(i), 8.0 * f64::from(j));
            specs.push(([x, y, 0.0], [x + 5.0, y + 5.0, 10.0]));
        }
    }
    let field = boxes(&mut topo, &specs);
    let field = merged(&mut topo, &field);
    let _ = crate::perf::take_ray_work();
    let geoms = FaceGeoms::new(collect_face_geoms(&topo, field).unwrap());
    let build = crate::perf::take_ray_work();
    // 96 faces on 18 planes: the bottoms (normal −z, d = +0) and the tops
    // (+z, d = 10) are one plane each, and each of the 4 wall positions per
    // side of x and y is its own.
    assert_eq!((build.planar_faces, build.plane_groups), (96, 18));
    let mut votes = 0;
    for i in 1..=64 {
        let p = p3(
            31.0 * halton(i, 2) + 1.0,
            31.0 * halton(i, 3) + 1.0,
            1.0 + 8.0 * halton(i, 5),
        );
        votes_from_geoms(&geoms, p, Tolerance::default()).unwrap();
        votes += 1;
    }
    let crate::perf::RayWorkCounts {
        votes: vote,
        plane_evals,
        polygon_tests: polygons,
        face_skips: skips,
        segment_evals: segments,
        windings,
        ..
    } = crate::perf::take_ray_work();
    assert_eq!(
        plane_evals,
        3 * 18 * vote,
        "one plane hit per plane and ray"
    );
    assert!(
        vote >= votes,
        "{vote} votes counted for {votes} classifications"
    );
    assert!(
        polygons >= 20 * vote,
        "{polygons} polygon tests over {vote} votes"
    );
    assert!(
        2 * skips >= polygons,
        "{skips} of {polygons} polygon tests skipped"
    );
    assert!(
        segments <= vote,
        "{segments} segment distances over {vote} votes"
    );
    assert!(windings <= polygons - skips, "{windings} windings");
}

// ---------------------------------------------------------------------------
// Plane groups.
// ---------------------------------------------------------------------------

#[test]
fn ray_dirs_are_the_historic_directions() {
    let bits = |v: Vec3| [v.x().to_bits(), v.y().to_bits(), v.z().to_bits()];
    for k in 0..3 {
        assert_eq!(bits(RAY_DIRS[k]), bits(CARDINAL_DIRS[k]), "cardinal {k}");
        assert_eq!(bits(RAY_DIRS[3 + k]), bits(GENERIC_DIRS[k]), "generic {k}");
    }
}

/// Faces share a group only when their `(normal, d)` bits are identical: a
/// last-ulp difference in `d` or a differently signed zero keeps planes
/// apart. Groups keep first-seen order, faces their collection order, and
/// rebuilding gives the same grouping.
#[test]
fn plane_groups_share_exact_plane_bits_only() {
    let up = Vec3::new(0.0, 0.0, 1.0);
    let down = Vec3::new(0.0, 0.0, -1.0);
    let down_signed = Vec3::new(-0.0, -0.0, -1.0);
    let tilted = Vec3::new(0.6, 0.0, 0.8);
    // Each face is tagged by its first vertex's x.
    let face = |tag: f64, normal: Vec3, d: f64| {
        FaceGeom::planar(
            loop_at_z0(&rect(tag, 0.0, tag + 0.5, 1.0)),
            Vec::new(),
            normal,
            d,
        )
    };
    let cylinder = || {
        let mut topo = Topology::default();
        let solid = super::tests::make_seamed_cylinder(&mut topo, 0.25);
        collect_face_geoms(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|g| matches!(g, FaceGeom::Cylinder { .. }))
            .unwrap()
    };
    let build = || {
        FaceGeoms::new(vec![
            face(0.0, up, 0.0),
            face(1.0, down, 0.0),
            face(2.0, down_signed, 0.0),
            face(3.0, up, 0.0),
            face(4.0, up, -0.0),
            cylinder(),
            face(5.0, tilted, 1.0),
            face(6.0, tilted, 1.0_f64.next_up()),
            face(7.0, down_signed, 0.0),
            face(8.0, tilted, 1.0),
        ])
    };
    let _ = crate::perf::take_ray_work();
    let geoms = build();
    let work = crate::perf::take_ray_work();
    assert_eq!((work.planar_faces, work.plane_groups), (9, 6));
    let tags = |g: &FaceGeoms| -> Vec<Vec<f64>> {
        g.planes
            .iter()
            .map(|p| p.faces.iter().map(|f| f.loops.outer.pts[0].x()).collect())
            .collect()
    };
    let want: Vec<Vec<f64>> = vec![
        vec![0.0, 3.0],
        vec![1.0],
        vec![2.0, 7.0],
        vec![4.0],
        vec![5.0, 8.0],
        vec![6.0],
    ];
    assert_eq!(tags(&geoms), want);
    assert_eq!(tags(&build()), want);
    assert_eq!(geoms.others.len(), 1);
    assert_eq!(geoms.len(), 10);
    for group in &geoms.planes {
        assert_eq!(group.axis, Axis::of(group.normal));
        for (k, dir) in RAY_DIRS.iter().enumerate() {
            assert_eq!(group.denoms[k].to_bits(), group.normal.dot(*dir).to_bits());
        }
    }
    assert_eq!(geoms.planes[3].d.to_bits(), (-0.0_f64).to_bits());
    assert_eq!(geoms.planes[5].d, 1.0_f64.next_up());
}

/// A group's shared plane hit gives each face exactly what the face's own
/// test gives, for hits ahead, behind, parallel off the plane and parallel in
/// it, along every ray direction; and a vote costs one plane hit per group
/// and ray however many faces share the plane.
#[test]
fn shared_plane_hits_match_each_face_alone() {
    let tilted = Vec3::new(0.6, 0.0, 0.8);
    let flat_faces = || {
        let mut out = Vec::new();
        for k in 0..4 {
            let x = 3.0 * f64::from(k);
            out.push(flat_face(&rect(x, 0.0, x + 2.0, 2.0), &[]));
            out.push(flat_face(
                &rect(x, 4.0, x + 2.0, 6.0),
                &[rect(x + 0.5, 4.5, x + 1.5, 5.5)],
            ));
            let verts = vec![
                p3(x, 0.0, 0.0),
                p3(x + 0.8, 0.0, -0.6),
                p3(x + 0.8, 1.0, -0.6),
                p3(x, 1.0, 0.0),
            ];
            out.push(FaceGeom::planar(verts, Vec::new(), tilted, 0.0));
        }
        out
    };
    let grouped = FaceGeoms::new(flat_faces());
    assert_eq!(grouped.planes.len(), 2);
    let flat = flat_faces();
    let tol = Tolerance::default();
    let mut points = Vec::new();
    for i in 1..=200 {
        let x = 14.0 * halton(i, 2) - 1.0;
        let y = 8.0 * halton(i, 3) - 1.0;
        for z in [-1.0, 0.0, 1e-7, 1.0] {
            points.push(p3(x, y, z));
        }
    }
    // On a rim, in the plane of the faces.
    points.extend([p3(1.0, 0.0, 0.0), p3(3.0, 4.5, 0.0), p3(0.0, 0.0, 0.0)]);
    for point in points {
        let _ = crate::perf::take_ray_work();
        for (base, dirs) in [(0, CARDINAL_DIRS), (3, GENERIC_DIRS)] {
            assert_eq!(
                cast_rays(&grouped, point, base, tol, false),
                rays_reference(&flat, point, &dirs, tol),
                "rays {base} at {point:?}"
            );
        }
        let work = crate::perf::take_ray_work();
        assert_eq!((work.votes, work.plane_evals), (2, 2 * 3 * 2));
        assert_eq!(
            votes_from_geoms(&grouped, point, tol).unwrap(),
            votes_reference(&flat, point, tol).unwrap(),
            "votes at {point:?}"
        );
    }
}

#[test]
fn an_empty_solid_still_fails_the_vote() {
    let empty = FaceGeoms::new(Vec::new());
    assert_eq!(empty.len(), 0);
    assert!(votes_from_geoms(&empty, p3(0.0, 0.0, 0.0), Tolerance::default()).is_err());
}

/// Each plane-hit threshold, bracketed: a ray parallel to the plane (|denom|
/// below `tol.angular`) flags its origin in the plane up to exactly `near`;
/// otherwise the plane counts only beyond `t = tol.linear`.
#[test]
fn plane_hit_brackets_its_thresholds() {
    let near = 2f64.powi(-20);
    let tol = tol_with_near(near);
    let (o, up) = (p3(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(plane_hit(o, up, 0.0, near, tol), PlaneHit::Parallel(true));
    assert_eq!(plane_hit(o, up, 0.0, -near, tol), PlaneHit::Parallel(true));
    assert_eq!(
        plane_hit(o, up, 0.0, near.next_up(), tol),
        PlaneHit::Parallel(false)
    );
    let angular = tol.angular;
    assert_eq!(
        plane_hit(o, up, angular.next_down(), 0.0, tol),
        PlaneHit::Parallel(true)
    );
    assert_eq!(
        plane_hit(o, up, -angular.next_down(), 1.0, tol),
        PlaneHit::Parallel(false)
    );
    assert_eq!(
        plane_hit(o, up, angular, angular * 4.0, tol),
        PlaneHit::Ahead(p3(1.0, 2.0, 7.0))
    );
    assert_eq!(plane_hit(o, up, 1.0, tol.linear, tol), PlaneHit::Behind);
    assert_eq!(plane_hit(o, up, -2.0, 4.0, tol), PlaneHit::Behind);
    let t = tol.linear.next_up();
    assert_eq!(
        plane_hit(o, up, 1.0, t, tol),
        PlaneHit::Ahead(p3(1.0, 2.0, 3.0 + t))
    );
    assert_eq!(
        plane_hit(o, Vec3::new(1.0, -2.0, 0.5), -2.0, -4.0, tol),
        PlaneHit::Ahead(p3(3.0, -2.0, 4.0))
    );
}
