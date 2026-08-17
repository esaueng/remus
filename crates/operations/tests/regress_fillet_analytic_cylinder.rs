//! A constant-radius fillet along a STRAIGHT edge between two planes is a
//! portion of a right circular cylinder, exactly. This pins that the kernel
//! emits it as one — `FaceSurface::Cylinder`, one face per filleted edge — and
//! never as a B-spline stripe.
//!
//! The loss this guards against was measurable: OpenZCAD's kernel parity
//! harness replays its "Mounting Bracket" demo (an L-blank with a boss, a bore
//! and two mount holes, whose Rev C fillets the four vertical base-plate
//! corners at r = 3) and censuses the result. The four corner blends arrived as
//! eight NURBS patches instead of four cylinders, which costs the analytic
//! volume/boolean fast paths, a CIRCLE-based STEP export, and — because
//! OpenZCAD's topology identity scheme (ADR-011/ADR-013) fingerprints analytic
//! faces but fails closed on B-splines — the user's later edits stop resolving
//! to that face.
//!
//! Both engines behind `fillet_v2` are covered: the planar rolling-ball rebuild
//! (which the bracket and the plate take) and the walking builder.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const W: f64 = 80.0;
const D: f64 = 40.0;
const PT: f64 = 8.0;
const WH: f64 = 32.0;
const BOSS_R: f64 = 10.0;
const HOLE_R: f64 = 4.0;
const MOUNT_R: f64 = 3.0;
const MOUNT_INSET: f64 = 16.0;
const FILLET_R: f64 = 3.0;

/// Faces of `solid` split by surface kind: (planes, cylinders, b-splines,
/// other).
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0);
    for fid in remus_topology::explorer::solid_faces(topo, solid).expect("faces") {
        match topo.face(fid).expect("face").surface() {
            FaceSurface::Plane { .. } => counts.0 += 1,
            FaceSurface::Cylinder(_) => counts.1 += 1,
            FaceSurface::Nurbs(_) => counts.2 += 1,
            _ => counts.3 += 1,
        }
    }
    counts
}

/// Faces of `solid` that are cylinders of exactly `radius`.
fn cylinders_of_radius(topo: &Topology, solid: SolidId, radius: f64) -> Vec<FaceId> {
    remus_topology::explorer::solid_faces(topo, solid)
        .expect("faces")
        .into_iter()
        .filter(|&fid| {
            matches!(
                topo.face(fid).expect("face").surface(),
                FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9
            )
        })
        .collect()
}

/// Assert no face of `solid` is a B-spline, naming the operation that produced
/// it.
fn assert_no_nurbs(topo: &Topology, solid: SolidId, what: &str) {
    let (planes, cylinders, nurbs, other) = census(topo, solid);
    assert_eq!(
        nurbs, 0,
        "{what}: a straight-edge fillet must stay analytic, but the result has \
         {nurbs} b-spline face(s) (planes {planes}, cylinders {cylinders}, \
         other {other})"
    );
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 0.1
}

/// A plain plate's four vertical corner edges.
fn corner_edges(topo: &Topology, solid: SolidId, w: f64, d: f64, z_hi: f64) -> Vec<EdgeId> {
    let mut picked = Vec::new();
    for eid in remus_topology::explorer::solid_edges(topo, solid).expect("edges") {
        let edge = topo.edge(eid).expect("edge");
        let a = topo.vertex(edge.start()).expect("vertex").point();
        let b = topo.vertex(edge.end()).expect("vertex").point();
        let corner = |x: f64, y: f64| (near(x, 0.0) || near(x, w)) && (near(y, 0.0) || near(y, d));
        let in_z = |z: f64| (-0.1..=z_hi + 0.1).contains(&z);
        if corner(a.x(), a.y())
            && corner(b.x(), b.y())
            && in_z(a.z())
            && in_z(b.z())
            && (a.x() - b.x()).abs() <= 1.5
            && (a.y() - b.y()).abs() <= 1.5
            && (a.z() - b.z()).abs() >= 4.0
        {
            picked.push(eid);
        }
    }
    picked
}

/// ZYX-Euler rotate-about-x-90° + translate, matching OpenZCAD's
/// `transformMatrix`: (x, y, z) → (x, -z, y) + t.
fn rot_x90_translate(tx: f64, ty: f64, tz: f64) -> Mat4 {
    Mat4::translation(tx, ty, tz) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2)
}

/// The OpenZCAD demo bracket, built the way its adapter builds it: every
/// boolean result is face-unified, which is what leaves the side faces
/// L-shaped and the base-plate caps holed — the topology that pushed this
/// blend onto the rolling-ball rebuild in the first place.
fn build_bracket(topo: &mut Topology) -> SolidId {
    let unify = |topo: &mut Topology, solid: SolidId| {
        remus_operations::heal::unify_faces(topo, solid).expect("unify");
        solid
    };

    let base = make_box(topo, W, D, PT).expect("base plate");
    let wall = make_box(topo, W, PT, WH).expect("wall plate");
    transform_solid(topo, wall, &Mat4::translation(0.0, D - PT, PT - 0.5)).expect("seat wall");
    let l_blank = boolean(topo, BooleanOp::Fuse, base, wall).expect("union L bracket");
    let l_blank = unify(topo, l_blank);

    let boss = make_cylinder(topo, BOSS_R, PT + 4.0).expect("boss");
    transform_solid(
        topo,
        boss,
        &rot_x90_translate(W / 2.0, D - PT + 2.0, PT + WH / 2.0),
    )
    .expect("place boss");
    let with_boss = boolean(topo, BooleanOp::Fuse, l_blank, boss).expect("union boss");
    let with_boss = unify(topo, with_boss);

    let bore = make_cylinder(topo, HOLE_R, WH + 16.0).expect("bore");
    transform_solid(
        topo,
        bore,
        &rot_x90_translate(W / 2.0, D + 8.0, PT + WH / 2.0),
    )
    .expect("aim bore");
    let bored = boolean(topo, BooleanOp::Cut, with_boss, bore).expect("boss bore");
    let bored = unify(topo, bored);

    let mount_a = make_cylinder(topo, MOUNT_R, PT + 4.0).expect("mount hole L");
    transform_solid(
        topo,
        mount_a,
        &Mat4::translation(MOUNT_INSET, D / 2.0, -2.0),
    )
    .expect("place mount L");
    let cut_a = boolean(topo, BooleanOp::Cut, bored, mount_a).expect("cut mount L");

    let mount_b = make_cylinder(topo, MOUNT_R, PT + 4.0).expect("mount hole R");
    transform_solid(
        topo,
        mount_b,
        &Mat4::translation(W - MOUNT_INSET, D / 2.0, -2.0),
    )
    .expect("place mount R");
    let cut_b = boolean(topo, BooleanOp::Cut, cut_a, mount_b).expect("cut mount R");
    unify(topo, cut_b)
}

/// One vertical edge of a box: the irreducible case. One cylinder, one face.
#[test]
fn single_straight_edge_fillet_is_one_cylinder() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 20.0, 12.0, 10.0).expect("box");
    let edges = corner_edges(&topo, solid, 20.0, 12.0, 10.0);
    let edge = *edges.first().expect("a vertical corner edge");

    let result = fillet_v2(&mut topo, solid, &[edge], 2.0).expect("fillet a box edge");
    assert_no_nurbs(&topo, result.solid, "box, one edge");
    assert_eq!(
        cylinders_of_radius(&topo, result.solid, 2.0).len(),
        1,
        "one filleted edge must yield exactly one blend face"
    );
}

/// All four vertical edges at once — the corner-patch case #38 taught the
/// rolling-ball rebuild to close.
#[test]
fn box_corner_chain_fillet_is_four_cylinders() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 80.0, 40.0, 8.0).expect("plate");
    let edges = corner_edges(&topo, solid, 80.0, 40.0, 8.0);
    assert_eq!(edges.len(), 4, "expected 4 corner edges");

    let result = fillet_v2(&mut topo, solid, &edges, 3.0).expect("fillet all four corners");
    assert_no_nurbs(&topo, result.solid, "plate, four corners");
    assert_eq!(
        census(&topo, result.solid),
        (6, 4, 0, 0),
        "a filleted plate is 6 planes and 4 quarter-cylinders — no more, no less"
    );
}

/// The OpenZCAD demo bracket's Rev C. Its four corner blends were the eight
/// B-spline patches the parity harness caught; they must be four cylinders.
#[test]
fn bracket_corner_fillets_are_analytic_cylinders() {
    let mut topo = Topology::new();
    let bracket = build_bracket(&mut topo);
    let edges = corner_edges(&topo, bracket, W, D, PT);
    assert_eq!(edges.len(), 4, "expected 4 base-plate corner edges");

    let (planes_before, cylinders_before, nurbs_before, _) = census(&topo, bracket);
    assert_eq!(
        (planes_before, cylinders_before, nurbs_before),
        (9, 4, 0),
        "the un-filleted bracket should be 9 planes and 4 cylinders \
         (bore, two mount holes, boss)"
    );

    let result = fillet_v2(&mut topo, bracket, &edges, FILLET_R).expect("fillet bracket corners");
    assert_no_nurbs(&topo, result.solid, "bracket, four corners");

    // Four new cylinders, one per corner, on top of the four the body already
    // had. Anything else means a corner came out as several patches — the two
    // rear columns are each cut into three collinear edges by the wall seat
    // (z = 7.5) and the plate top (z = 8), so their three stripes have to end
    // up merged back into the one cylinder they lie on.
    assert_eq!(
        census(&topo, result.solid),
        (9, 8, 0, 0),
        "each corner blend must be ONE cylindrical face"
    );
    assert_eq!(
        cylinders_of_radius(&topo, result.solid, FILLET_R).len(),
        4 + 2, // 4 corner blends + the two r = 3 mount holes
        "expected one r=3 blend face per corner alongside the two mount bores"
    );

    let shell = topo.solid(result.solid).expect("solid").outer_shell();
    remus_topology::validation::validate_shell_closed(topo.shell(shell).expect("shell"), &topo)
        .expect("the filleted bracket must stay a closed shell");
}

/// The blend faces are not merely typed as cylinders — they carry the exact
/// rolling-ball geometry. Every point of every blend face is one radius from
/// its corner's axis.
#[test]
fn bracket_blend_cylinders_carry_exact_geometry() {
    let mut topo = Topology::new();
    let bracket = build_bracket(&mut topo);
    let edges = corner_edges(&topo, bracket, W, D, PT);
    let result = fillet_v2(&mut topo, bracket, &edges, FILLET_R).expect("fillet bracket corners");

    // A blend of radius 3 on a right-angled corner has its axis 3 mm inside the
    // corner in both x and y.
    let axes = [
        (FILLET_R, FILLET_R),
        (W - FILLET_R, FILLET_R),
        (FILLET_R, D - FILLET_R),
        (W - FILLET_R, D - FILLET_R),
    ];
    let mut blend_faces = 0;
    for fid in cylinders_of_radius(&topo, result.solid, FILLET_R) {
        let face = topo.face(fid).expect("face");
        let FaceSurface::Cylinder(cyl) = face.surface() else {
            unreachable!("filtered to cylinders")
        };
        let origin = cyl.origin();
        let Some(&(ax, ay)) = axes
            .iter()
            .find(|&&(ax, ay)| near(origin.x(), ax) && near(origin.y(), ay))
        else {
            continue; // a mount bore, not a corner blend
        };
        blend_faces += 1;
        assert!(
            cyl.axis().z().abs() > 1.0 - 1e-9,
            "a vertical corner blend sweeps a vertical axis, got {:?}",
            cyl.axis()
        );
        for oe in topo.wire(face.outer_wire()).expect("wire").edges() {
            let edge = topo.edge(oe.edge()).expect("edge");
            for vid in [edge.start(), edge.end()] {
                let p = topo.vertex(vid).expect("vertex").point();
                let r = ((p.x() - ax).powi(2) + (p.y() - ay).powi(2)).sqrt();
                assert!(
                    (r - FILLET_R).abs() < 1e-9,
                    "blend wire vertex {p:?} sits {r} from the axis, not {FILLET_R}"
                );
            }
        }
    }
    assert_eq!(blend_faces, 4, "expected four corner blend faces");
}
