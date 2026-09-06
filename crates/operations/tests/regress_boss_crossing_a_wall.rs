//! A cylinder that overhangs a planar wall of the other operand.
//!
//! A boss hanging over a plate's edge, or a bore drilled flush with a wall, is
//! ordinary design intent. Both put a cylindrical face across a planar face of
//! the other operand, and the boolean went silently wrong for every one of
//! them: the result was always a well-formed solid `validate_solid` accepts,
//! just not the right one.
//!
//! Measured on a 60x40x8 plate and an r=10, h=16 boss whose axis stands at
//! `x = R + d`, so the boss is tangent to the plate's `x = 0` wall at `d = 0`
//! and crosses it below. Three regimes, wrong in three different ways:
//!
//! | `d`            | fuse before                     | cut before                  |
//! | -------------- | ------------------------------- | --------------------------- |
//! | `> 0` clear    | 9 faces, exact                  | 7 faces, exact              |
//! | `= 0` tangent  | 6 planes, 19200 — the plate alone, **-11.57 %** | 6 planes, 19200 — the cut ignored, **+15.06 %** |
//! | `< 0` crossing | 70-71 planes, no cylinder at all | 62-69 planes, no cylinder  |
//!
//! The crossing regime is the wide one: it fired at ANY overlap depth, from
//! 1e-7 to past the full radius, and it is fixed — the result is analytic and
//! its volume is the closed form. Tangency is a knife edge whose union has a
//! pinch vertex the analytic splitter does not build; it now fails over to the
//! approximate path instead of returning an operand-shaped lie.
//!
//! Every expected volume is composed from the construction's own dimensions —
//! the plate box, the boss cylinder, and the circular segment
//! `r^2 acos(d/r) - d sqrt(r^2 - d^2)` the wall cuts off the boss's footprint.
//! Nothing is a recorded measurement, because the two worst symptoms produce no
//! approximation to record: a dropped operand and an ignored cut are simply
//! less geometry, and an approximation census sees neither.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const PLATE_X: f64 = 60.0;
const PLATE_Y: f64 = 40.0;
const PLATE_Z: f64 = 8.0;
const R: f64 = 10.0;
const H: f64 = 16.0;
const DEFLECTION: f64 = 0.01;
const SCALE_SWEEP: [f64; 3] = [1e-3, 1.0, 1e3];

/// Overlaps at which the analytic path must carry the whole operation: the boss
/// clear of the wall, then crossing it at depths spanning seven orders of
/// magnitude out to half the radius. A fix that works at one placement and not
/// the next is half a fix, so this is a sweep and not a case.
///
/// One band is deliberately absent: a fuse whose boss crosses by between about
/// 1e-5 and 0.05 mm (on r = 10) still over-splits the boss wall and falls back
/// to the approximate path. That sliver band is swept by
/// [`every_crossing_depth_is_at_least_close`], which holds the whole range to
/// the approximate bound without demanding the analytic result.
const CROSSING_SWEEP: [f64; 8] = [1.0, 0.5, 1e-7, -1e-7, -0.1, -0.5, -1.0, -5.0];

/// The full crossing range including the sliver band the analytic path still
/// declines. Nothing here may be an operand-shaped answer, analytic or not.
const FULL_SWEEP: [f64; 13] = [
    1.0, 0.5, 1e-7, 0.0, -1e-7, -1e-5, -0.001, -0.01, -0.05, -0.1, -0.5, -1.0, -5.0,
];

/// Plate `[0,60] x [0,40] x [0,8]`; boss r=10, h=16 on a vertical axis at
/// `(R + d, 20)`. `d > 0` clears the `x = 0` wall, `d = 0` is tangent to it,
/// `d < 0` crosses it.
fn build(topo: &mut Topology, d: f64) -> (SolidId, SolidId) {
    build_scaled(topo, d, 1.0)
}

fn build_scaled(topo: &mut Topology, d: f64, scale: f64) -> (SolidId, SolidId) {
    let plate = make_box(topo, PLATE_X * scale, PLATE_Y * scale, PLATE_Z * scale).unwrap();
    let boss = make_cylinder(topo, R * scale, H * scale).unwrap();
    transform_solid(
        topo,
        boss,
        &Mat4::translation((R + d) * scale, PLATE_Y * scale / 2.0, 0.0),
    )
    .unwrap();
    (plate, boss)
}

/// Area of the boss footprint OUTSIDE the plate: the circular segment beyond
/// `x = 0`, whose signed distance from the boss axis is `R + d`. Zero while the
/// boss clears the wall.
fn segment_outside(d: f64) -> f64 {
    let h = R + d;
    if h >= R {
        return 0.0;
    }
    if h <= -R {
        return PI * R * R;
    }
    R * R * (h / R).acos() - h * (R * R - h * h).sqrt()
}

/// Footprint the boss and the plate share in plan.
fn disc_inside(d: f64) -> f64 {
    PI * R * R - segment_outside(d)
}

fn fuse_volume(d: f64) -> f64 {
    PLATE_X * PLATE_Y * PLATE_Z + PI * R * R * H - disc_inside(d) * PLATE_Z
}

fn cut_volume(d: f64) -> f64 {
    PLATE_X * PLATE_Y * PLATE_Z - disc_inside(d) * PLATE_Z
}

fn expected(op: BooleanOp, d: f64) -> f64 {
    if op == BooleanOp::Fuse {
        fuse_volume(d)
    } else {
        cut_volume(d)
    }
}

fn expected_scaled(op: BooleanOp, d: f64, scale: f64) -> f64 {
    expected(op, d) * scale.powi(3)
}

/// `(planes, cylinders, other)` over the result's faces.
fn surface_census(topo: &Topology, s: SolidId) -> (usize, usize, usize) {
    let mut census = (0, 0, 0);
    for f in remus_topology::explorer::solid_faces(topo, s).unwrap() {
        match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { .. } => census.0 += 1,
            FaceSurface::Cylinder(_) => census.1 += 1,
            _ => census.2 += 1,
        }
    }
    census
}

/// Panics with the validator's own words if `solid` is not a valid solid, which
/// for these results means closed and 2-manifold: zero free boundary edges and
/// zero non-manifold edges.
fn assert_watertight(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate::validate_solid(topo, solid).expect("validate");
    assert!(
        report.is_valid(),
        "{what}: not a valid solid: {}",
        report
            .issues
            .iter()
            .filter(|i| i.severity == validate::Severity::Error)
            .map(|i| i.description.clone())
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// The body's own volume, read off the exact per-face integral rather than a
/// mesh. This is the assertion that catches a dropped operand and an ignored
/// cut: both leave a perfectly measurable body whose measurement is simply the
/// wrong number.
fn integrated_volume(topo: &Topology, solid: SolidId) -> f64 {
    mass_properties(topo, solid).expect("mass_properties").mass
}

fn run_sweep(op: BooleanOp, name: &str) {
    for d in CROSSING_SWEEP {
        let mut topo = Topology::new();
        let (plate, boss) = build(&mut topo, d);
        let result =
            boolean(&mut topo, op, plate, boss).unwrap_or_else(|e| panic!("{name} at d={d}: {e}"));
        let what = format!("{name} at d={d}");

        assert_watertight(&topo, result, &what);

        let want = expected(op, d);
        let got = integrated_volume(&topo, result);
        let rel = (got - want).abs() / want;
        assert!(
            rel < 1e-9,
            "{what}: volume {got:.10} against closed form {want:.10} ({:.6} %)",
            rel * 100.0
        );

        // "Watertight with the right volume" passes for a faceted body too, and
        // faceting was the symptom across the whole crossing regime — so count
        // the cylinders. The boss wall (fuse) and the bore wall (cut) must both
        // still be cylinders, and nothing may have turned into a NURBS patch.
        let (_, cylinders, other) = surface_census(&topo, result);
        assert!(
            cylinders >= 1 && other == 0,
            "{what}: expected the curved wall to stay analytic, got {cylinders} cylinder(s) \
             and {other} other non-planar face(s)"
        );

        // `solid_volume` reads a quadric wall as a rectangle in (u, v) unless
        // the wall is recognisably notched, so on the shapes where the crossing
        // splits the boss wall into a tab plus a ring it still over-credits the
        // buried arc. Bounded here rather than asserted exact — the exact
        // statement is the integral above.
        let meshed = solid_volume(&topo, result, DEFLECTION).unwrap();
        let mesh_rel = (meshed - want).abs() / want;
        assert!(
            mesh_rel < 4e-3,
            "{what}: solid_volume {meshed:.6} against closed form {want:.6} ({:.4} %)",
            mesh_rel * 100.0
        );
    }
}

#[test]
fn fuse_across_the_wall_is_exact_and_stays_a_cylinder() {
    run_sweep(BooleanOp::Fuse, "fuse");
}

#[test]
fn cut_across_the_wall_is_exact_and_stays_a_cylinder() {
    run_sweep(BooleanOp::Cut, "cut");
}

/// Exact tangency has no analytic answer here — the union's top face is a
/// rectangle with a disc removed that touches its own boundary at a single
/// point, and the splitter does not build that pinch. What it must NOT do is
/// hand back one operand and call it the answer: the fuse came back as the bare
/// plate (-11.57 %) and the cut as the untouched blank (+15.06 %), both passing
/// every structural check there is. The approximate path is allowed here; a
/// wrong body is not.
#[test]
fn a_tangent_boss_is_never_silently_dropped() {
    for &(op, name) in &[(BooleanOp::Fuse, "fuse"), (BooleanOp::Cut, "cut")] {
        let mut topo = Topology::new();
        let (plate, boss) = build(&mut topo, 0.0);
        let result = boolean(&mut topo, op, plate, boss)
            .unwrap_or_else(|e| panic!("{name} of a tangent boss: {e}"));
        let what = format!("{name} of a tangent boss");

        assert_watertight(&topo, result, &what);

        let want = expected(op, 0.0);
        let got = solid_volume(&topo, result, DEFLECTION).unwrap();
        let rel = (got - want).abs() / want;
        // The chord error of a polygonised r=10 boss, not an operand-shaped
        // hole in the answer.
        assert!(
            rel < 1e-3,
            "{what}: volume {got:.4} against closed form {want:.4} ({:.4} %)",
            rel * 100.0
        );
    }
}

/// Both sides of tangency and the exact contact retain analytic material
/// across the scale sweep, including the former large-scale refusal.
#[test]
fn tangent_boundary_is_stable_across_scale() {
    for scale in SCALE_SWEEP {
        for d in [0.1, 0.0, -0.1] {
            for &(op, name) in &[(BooleanOp::Fuse, "fuse"), (BooleanOp::Cut, "cut")] {
                let mut topo = Topology::new();
                let (plate, boss) = build_scaled(&mut topo, d, scale);
                let what = format!("{name} at d/r={} and scale={scale}", d / R);
                let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
                let outcome = boolean_with_context(&mut topo, op, plate, boss, &exact)
                    .unwrap_or_else(|e| panic!("{what}: qualified exact cell failed: {e}"));
                assert_eq!(outcome.quality, BooleanQuality::Exact, "{what}");
                let result = outcome.solid;
                assert_watertight(&topo, result, &what);
                let want = expected_scaled(op, d, scale);
                let got = integrated_volume(&topo, result);
                let rel = (got - want).abs() / want;
                assert!(
                    rel < 1e-9,
                    "{what}: volume {got:.10} against closed form {want:.10} ({rel:e})"
                );
                let (_, cylinders, other) = surface_census(&topo, result);
                assert!(
                    cylinders >= 1 && other == 0,
                    "{what}: expected analytic curved wall, got {cylinders} cylinders and {other} other non-planar faces"
                );
            }
        }
    }
}

/// The material the boss and the plate share is one quantity, so the two
/// operations that account for it have to agree: `(A ∪ B) - (A - B)` is exactly
/// `vol(B)`, whatever the overlap, because both differ from `vol(A)` by the same
/// shared part. Independent of either result being right on its own, and it is
/// the invariant a dropped operand breaks hardest — the tangent fuse lost the
/// whole boss and the tangent cut kept the whole overlap.
#[test]
fn cut_and_fuse_account_for_the_same_material() {
    for d in CROSSING_SWEEP {
        let mut cut_topo = Topology::new();
        let (plate, boss) = build(&mut cut_topo, d);
        let cut = boolean(&mut cut_topo, BooleanOp::Cut, plate, boss).unwrap();
        let mut fuse_topo = Topology::new();
        let (plate2, boss2) = build(&mut fuse_topo, d);
        let fused = boolean(&mut fuse_topo, BooleanOp::Fuse, plate2, boss2).unwrap();

        let lhs = integrated_volume(&fuse_topo, fused) - integrated_volume(&cut_topo, cut);
        let rhs = PI * R * R * H;
        let rel = (lhs - rhs).abs() / rhs;
        assert!(
            rel < 1e-9,
            "d={d}: (A∪B) - (A-B) = {lhs:.10}, vol(B) = {rhs:.10} ({:.6} %)",
            rel * 100.0
        );
    }
}

/// The whole crossing range, analytic or not: every placement must produce a
/// closed body whose volume is the closed form to within the chord error of a
/// polygonised r=10 boss. This is the sweep that pins the two silent failures,
/// which were -11.57 % and +15.06 % — neither of them an approximation, so
/// neither visible to an approximation census.
#[test]
fn every_crossing_depth_is_at_least_close() {
    for &(op, name) in &[(BooleanOp::Fuse, "fuse"), (BooleanOp::Cut, "cut")] {
        for d in FULL_SWEEP {
            let mut topo = Topology::new();
            let (plate, boss) = build(&mut topo, d);
            let result = boolean(&mut topo, op, plate, boss)
                .unwrap_or_else(|e| panic!("{name} at d={d}: {e}"));
            let what = format!("{name} at d={d}");
            assert_watertight(&topo, result, &what);
            let want = expected(op, d);
            let got = solid_volume(&topo, result, DEFLECTION).unwrap();
            let rel = (got - want).abs() / want;
            assert!(
                rel < 4e-3,
                "{what}: volume {got:.4} against closed form {want:.4} ({:.4} %)",
                rel * 100.0
            );
        }
    }
}

/// The same contact against a bored plate: the boss now crosses a wall of a
/// body that already carries a cylindrical cavity, so two curved-planar
/// contacts have to coexist. Before, this fused to 115 planes from 7 analytic
/// faces going in.
#[test]
fn a_bored_plate_keeps_its_bore_when_a_crossing_boss_is_fused() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, PLATE_X, PLATE_Y, PLATE_Z).unwrap();
    let bore = make_cylinder(&mut topo, 5.0, PLATE_Z * 3.0).unwrap();
    transform_solid(
        &mut topo,
        bore,
        &Mat4::translation(45.0, PLATE_Y / 2.0, -PLATE_Z),
    )
    .unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, plate, bore).expect("bore the plate");

    let boss = make_cylinder(&mut topo, R, H).unwrap();
    transform_solid(
        &mut topo,
        boss,
        &Mat4::translation(R - 0.5, PLATE_Y / 2.0, 0.0),
    )
    .unwrap();
    let result = boolean(&mut topo, BooleanOp::Fuse, bored, boss).expect("fuse the boss");

    assert_watertight(&topo, result, "bored plate fused with a crossing boss");

    let want = PLATE_X * PLATE_Y * PLATE_Z - PI * 25.0 * PLATE_Z + PI * R * R * H
        - disc_inside(-0.5) * PLATE_Z;
    let got = integrated_volume(&topo, result);
    let rel = (got - want).abs() / want;
    assert!(
        rel < 1e-9,
        "bored plate fused with a crossing boss: volume {got:.10} against closed form \
         {want:.10} ({:.6} %)",
        rel * 100.0
    );

    let (_, cylinders, other) = surface_census(&topo, result);
    assert!(
        cylinders >= 2 && other == 0,
        "bored plate fused with a crossing boss: expected the bore AND the boss wall to stay \
         analytic, got {cylinders} cylinder(s) and {other} other non-planar face(s)"
    );
}
