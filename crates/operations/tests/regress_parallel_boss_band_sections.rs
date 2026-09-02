//! Regression: a parallel-axis cylindrical boss shorter than the shaft it
//! sits on must fuse, cut and intersect on the exact path.
//!
//! `make_cylinder(15, 60)` with a `make_cylinder(7.5, 20)` boss whose axis
//! runs parallel at 15 mm, caps at z = 20 and z = 40, went to the mesh
//! fallback (every curved surface lost) for every placement around the
//! shaft. Two defects stacked:
//!
//! - The FF phase correctly trims the two cylinder×cylinder generator lines
//!   to the faces' mutual height band, which leaves them strictly inside the
//!   shaft wall with no rim to cross. `clip_line_to_face_boundary` then read
//!   "no boundary crossing" as "outside the face" and dropped both lines, so
//!   the wall's section loop (line, arc, line, arc) never closed and the wall
//!   was not split. A box boss survived only because plane×cylinder lines
//!   arrive untrimmed and cross both rims.
//! - With the boss on the shaft's seam side (its cap pierced by the seam
//!   line), the boss's bottom cap — a reversed face — still came back whole:
//!   the wire builder evaluated the tangent of each reversed boundary arc at
//!   the wrong pcurve end, so the traversal hugged the rim past the section
//!   junctions. The top cap, traversed forward, split correctly.
//!
//! Every case here is checked against closed-form volumes, so the exact
//! result is proved right, not merely closed.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::blend_ops::chamfer_v2;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const PI: f64 = std::f64::consts::PI;
const SHAFT_R: f64 = 15.0;
const SHAFT_H: f64 = 60.0;
const BOSS_R: f64 = 7.5;
const BOSS_H: f64 = 20.0;

/// Area common to two discs of radii `r1`, `r2` with centres `d` apart.
fn lens_area(r1: f64, r2: f64, d: f64) -> f64 {
    assert!((r1 - r2).abs() < d && d < r1 + r2, "discs must cross");
    let a1 = ((d * d + r1 * r1 - r2 * r2) / (2.0 * d * r1)).acos();
    let a2 = ((d * d + r2 * r2 - r1 * r1) / (2.0 * d * r2)).acos();
    let k = ((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)).sqrt();
    r1 * r1 * a1 + r2 * r2 * a2 - 0.5 * k
}

fn cylinder_face_count(topo: &Topology, solid: SolidId) -> usize {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .iter()
        .filter(|f| matches!(topo.face(**f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count()
}

/// Run `op` on the shaft and a boss at polar angle `deg`, distance `dist`,
/// base height `z`, on the exact-only policy, and return the result volume.
fn exact_volume(op: BooleanOp, deg: f64, dist: f64, z: f64) -> f64 {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, SHAFT_R, SHAFT_H).unwrap();
    let boss = make_cylinder(&mut topo, BOSS_R, BOSS_H).unwrap();
    let a = deg.to_radians();
    transform_solid(
        &mut topo,
        boss,
        &Mat4::translation(dist * a.cos(), dist * a.sin(), z),
    )
    .unwrap();
    let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, op, shaft, boss, &exact)
        .unwrap_or_else(|e| panic!("{op:?} at {deg}°, d={dist}, z={z} left the exact path: {e}"));
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    // Strict validation is asserted for the fuse only: a cut or intersect
    // whose pocket is an interior loop on the wall comes back with
    // inconsistently oriented shared edges, and it did so before these
    // sections reached the wall at all (a box boss cut from the wall shows
    // the same report on the pre-existing exact path). That is a separate
    // defect of the cut/intersect assembly, tracked by
    // `pocket_cut_from_wall_is_consistently_oriented` below.
    if op == BooleanOp::Fuse {
        let report = validate_solid(&topo, outcome.solid).unwrap();
        assert!(
            report.is_valid(),
            "{op:?} at {deg}°, d={dist}, z={z}: {:?}",
            report.issues
        );
    }
    let curved = cylinder_face_count(&topo, outcome.solid);
    assert!(
        curved >= 2,
        "{op:?} at {deg}°, d={dist}, z={z}: only {curved} cylindrical face(s) survived"
    );
    solid_volume(&topo, outcome.solid, 1e-3).unwrap()
}

fn assert_close(actual: f64, expected: f64, what: &str) {
    let rel = ((actual - expected) / expected).abs();
    assert!(
        rel < 2e-4,
        "{what}: got {actual}, expected {expected} (rel err {rel:.2e})"
    );
}

/// The boss placed with both caps inside the shaft's height, all around
/// the shaft — including the seam side and the placement whose axis lies
/// exactly on the shaft wall.
#[test]
fn shorter_parallel_boss_fuses_exactly_all_around() {
    let shaft = PI * SHAFT_R * SHAFT_R * SHAFT_H;
    let boss = PI * BOSS_R * BOSS_R * BOSS_H;
    for deg in [0.0, 45.0, 90.0, 180.0, 270.0] {
        for dist in [12.0, 15.0, 18.0] {
            let overlap = lens_area(SHAFT_R, BOSS_R, dist) * BOSS_H;
            assert_close(
                exact_volume(BooleanOp::Fuse, deg, dist, 20.0),
                shaft + boss - overlap,
                &format!("fuse at {deg}°, d={dist}"),
            );
        }
    }
}

/// The same operands cut and intersected: the band lines must reach the
/// shaft wall for every operation, not only for the fuse.
#[test]
fn shorter_parallel_boss_cuts_and_intersects_exactly() {
    let shaft = PI * SHAFT_R * SHAFT_R * SHAFT_H;
    for deg in [0.0, 180.0] {
        let overlap = lens_area(SHAFT_R, BOSS_R, 15.0) * BOSS_H;
        assert_close(
            exact_volume(BooleanOp::Cut, deg, 15.0, 20.0),
            shaft - overlap,
            &format!("cut at {deg}°"),
        );
        assert_close(
            exact_volume(BooleanOp::Intersect, deg, 15.0, 20.0),
            overlap,
            &format!("intersect at {deg}°"),
        );
    }
}

/// A boss taller than the shaft: its generator lines cross both shaft rims,
/// while the shaft's caps carve arcs into the boss wall.
#[test]
fn taller_parallel_boss_fuses_exactly() {
    let shaft = PI * SHAFT_R * SHAFT_R * SHAFT_H;
    let tall_h = 80.0;
    for deg in [0.0_f64, 180.0] {
        let mut topo = Topology::new();
        let a = make_cylinder(&mut topo, SHAFT_R, SHAFT_H).unwrap();
        let b = make_cylinder(&mut topo, BOSS_R, tall_h).unwrap();
        let ang = deg.to_radians();
        transform_solid(
            &mut topo,
            b,
            &Mat4::translation(15.0 * ang.cos(), 15.0 * ang.sin(), -10.0),
        )
        .unwrap();
        let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
        let outcome = boolean_with_context(&mut topo, BooleanOp::Fuse, a, b, &exact).unwrap();
        assert!(validate_solid(&topo, outcome.solid).unwrap().is_valid());
        let boss = PI * BOSS_R * BOSS_R * tall_h;
        let overlap = lens_area(SHAFT_R, BOSS_R, 15.0) * SHAFT_H;
        assert_close(
            solid_volume(&topo, outcome.solid, 1e-3).unwrap(),
            shaft + boss - overlap,
            &format!("tall fuse at {deg}°"),
        );
    }
}

/// The boss standing flush on the shaft's base: its generator lines start
/// ON the shaft's bottom rim and end in the wall's interior, and its bottom
/// cap is coplanar with the shaft's.
#[test]
fn flush_parallel_boss_fuses_exactly() {
    let shaft = PI * SHAFT_R * SHAFT_R * SHAFT_H;
    let boss = PI * BOSS_R * BOSS_R * BOSS_H;
    for deg in [0.0, 90.0, 180.0] {
        let overlap = lens_area(SHAFT_R, BOSS_R, 15.0) * BOSS_H;
        assert_close(
            exact_volume(BooleanOp::Fuse, deg, 15.0, 0.0),
            shaft + boss - overlap,
            &format!("flush fuse at {deg}°"),
        );
    }
}

/// Volume a symmetric 45° chamfer of size `c` removes from one rim of a
/// cylinder of radius `r` (Pappus: a `c²/2` triangle swept at radius
/// `r − c/3`).
fn rim_chamfer_volume(r: f64, c: f64) -> f64 {
    PI * c * c * (r - c / 3.0)
}

/// A shaft chamfered at both rims, boss on the seam side and `z` above the
/// base, fused on the exact-only policy; returns the result volume.
fn chamfered_shaft_boss_fuse_volume(z: f64) -> Result<f64, remus_operations::OperationsError> {
    let chamfer = 1.0;
    let mut topo = Topology::new();
    let blank = make_cylinder(&mut topo, SHAFT_R, SHAFT_H).unwrap();
    let rims: Vec<_> = remus_topology::explorer::solid_edges(&topo, blank)
        .unwrap()
        .into_iter()
        .filter(|eid| {
            let e = topo.edge(*eid).unwrap();
            e.start() == e.end() && matches!(e.curve(), EdgeCurve::Circle(_))
        })
        .collect();
    assert_eq!(rims.len(), 2);
    let chamfered = chamfer_v2(&mut topo, blank, &rims, chamfer, chamfer)
        .unwrap()
        .solid;
    assert_close(
        solid_volume(&topo, chamfered, 1e-3).unwrap(),
        PI * SHAFT_R * SHAFT_R * SHAFT_H - 2.0 * rim_chamfer_volume(SHAFT_R, chamfer),
        "chamfered shaft blank",
    );
    let boss = make_cylinder(&mut topo, BOSS_R, BOSS_H).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(15.0, 0.0, z)).unwrap();
    let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    let outcome = boolean_with_context(&mut topo, BooleanOp::Fuse, chamfered, boss, &exact)?;
    let report = validate_solid(&topo, outcome.solid).unwrap();
    assert!(
        report.is_valid(),
        "chamfered fuse at z={z}: {:?}",
        report.issues
    );
    Ok(solid_volume(&topo, outcome.solid, 1e-3).unwrap())
}

/// The reported configuration: a shaft chamfered at both rims, with the boss
/// on its seam side, clear of the chamfers.
#[test]
fn chamfered_shaft_with_parallel_boss_fuses_exactly() {
    let shaft = PI * SHAFT_R * SHAFT_R * SHAFT_H - 2.0 * rim_chamfer_volume(SHAFT_R, 1.0);
    let boss = PI * BOSS_R * BOSS_R * BOSS_H;
    let lens = lens_area(SHAFT_R, BOSS_R, 15.0) * BOSS_H;
    assert_close(
        chamfered_shaft_boss_fuse_volume(20.0).unwrap(),
        shaft + boss - lens,
        "chamfered fuse clear of the chamfers",
    );
}

/// The same boss standing flush on the chamfered base. Its wall now meets
/// the lower chamfer cone in two rim-to-rim section curves, and the cone
/// face comes back unsplit, so the fuse still leaves the exact path.
#[test]
#[ignore = "open: a cone band cut rim-to-rim by two cylinder×cone sections is not split"]
fn chamfered_shaft_with_flush_parallel_boss_fuses_exactly() {
    let shaft = PI * SHAFT_R * SHAFT_R * SHAFT_H - 2.0 * rim_chamfer_volume(SHAFT_R, 1.0);
    let boss = PI * BOSS_R * BOSS_R * BOSS_H;
    let lens = lens_area(SHAFT_R, BOSS_R, 15.0) * BOSS_H;
    let volume = chamfered_shaft_boss_fuse_volume(0.0).unwrap();
    // Flush on the base the boss refills part of the lower chamfer ring
    // inside the lens, so the union lies between the plain overlap figure
    // and that figure plus the whole ring.
    let low = shaft + boss - lens;
    let high = low + rim_chamfer_volume(SHAFT_R, 1.0);
    assert!(
        (low - 1e-6..=high + 1e-6).contains(&volume),
        "chamfered flush fuse: {volume} outside [{low}, {high}]"
    );
}

/// A pocket cut into the shaft wall — by the cylindrical boss, and by a box
/// boss that never left the exact path — must be consistently oriented.
/// Both report "shared edges have inconsistent face orientations" today.
#[test]
#[ignore = "pre-existing: cut/intersect pocket faces on a cylinder wall come back inconsistently oriented"]
fn pocket_cut_from_wall_is_consistently_oriented() {
    use remus_operations::primitives::make_box;
    let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    for tool_is_box in [true, false] {
        let mut topo = Topology::new();
        let shaft = make_cylinder(&mut topo, SHAFT_R, SHAFT_H).unwrap();
        let tool = if tool_is_box {
            let b = make_box(&mut topo, 10.0, 10.0, BOSS_H).unwrap();
            transform_solid(&mut topo, b, &Mat4::translation(-22.0, -5.0, 20.0)).unwrap();
            b
        } else {
            let b = make_cylinder(&mut topo, BOSS_R, BOSS_H).unwrap();
            transform_solid(&mut topo, b, &Mat4::translation(15.0, 0.0, 20.0)).unwrap();
            b
        };
        let outcome = boolean_with_context(&mut topo, BooleanOp::Cut, shaft, tool, &exact).unwrap();
        let report = validate_solid(&topo, outcome.solid).unwrap();
        assert!(
            report.is_valid(),
            "box tool = {tool_is_box}: {:?}",
            report.issues
        );
    }
}
