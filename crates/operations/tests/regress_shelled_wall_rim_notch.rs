//! Round cutters standing on the wall of a shelled box, and the bodies that
//! share its defects.
//!
//! The rim of an open shelled box is one planar face: a square outline with
//! the cavity opening as its hole. A cylinder standing on the wall crosses
//! both, and four things went wrong with it:
//!
//! 1. the rim × cylinder section was split only where it crosses the
//!    outline, so it ran on across the opening;
//! 2. a section ending on a hole edge found no vertex there, and the hole
//!    was attached whole;
//! 3. a span of a section that left a planar face by less than 1 % of the
//!    face's size was kept (over material it corrupts the partner cylinder:
//!    an L-bracket cut through its inner corner came back 0.6 % off,
//!    accepted);
//! 4. the single hole of a cylinder wall was wound by a 3D oracle that
//!    fails once the hole wraps well over half the wall (accepted with
//!    orientation errors).
//!
//! Every expected volume is a closed form: the area of a disc inside
//! axis-aligned rectangles, or the lens of two discs, times a height.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::disallowed_methods,
    missing_docs
)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanOptions, boolean, compound_cut};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::shell_op::shell;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const SCALES: [f64; 3] = [0.1, 1.0, 10.0];
/// 100 × 100 × 50 less the 96 × 96 × 48 cavity.
const SHELL_VOLUME: f64 = 57_632.0;
const VOLUME_TOL: f64 = 1e-10;

// ── Closed forms ────────────────────────────────────────────────────────

/// Area of the disc of radius `r` about the origin inside
/// `[x0, x1] × [y0, y1]`.
///
/// Between the abscissae where the circle meets `y = y0` and `y = y1` each
/// bound of the strip is either a line or the circle, so the integral is a
/// sum of rectangles and of `∫ √(r² − x²) dx = ½ (x √(r² − x²) + r² asin(x/r))`.
fn disc_in_rect(r: f64, x0: f64, x1: f64, y0: f64, y1: f64) -> f64 {
    let (a, b) = (x0.max(-r), x1.min(r));
    if a >= b {
        return 0.0;
    }
    let mut cuts = vec![a, b];
    for y in [y0, y1] {
        if y.abs() < r {
            let x = (r * r - y * y).sqrt();
            cuts.extend([-x, x].into_iter().filter(|c| *c > a && *c < b));
        }
    }
    cuts.sort_by(f64::total_cmp);
    let half_disc = |x: f64| {
        0.5 * (x * (r * r - x * x).max(0.0).sqrt() + r * r * (x / r).clamp(-1.0, 1.0).asin())
    };
    let mut area = 0.0;
    for w in cuts.windows(2) {
        let (p, q) = (w[0], w[1]);
        let mid = 0.5 * (p + q);
        let s = (r * r - mid * mid).max(0.0).sqrt();
        let (top_on_circle, bottom_on_circle) = (s < y1, -s > y0);
        let top = if top_on_circle { s } else { y1 };
        let bottom = if bottom_on_circle { -s } else { y0 };
        if top <= bottom {
            continue;
        }
        let arc = half_disc(q) - half_disc(p);
        area += if top_on_circle { arc } else { y1 * (q - p) };
        area -= if bottom_on_circle { -arc } else { y0 * (q - p) };
    }
    area
}

/// Area common to discs of radii `r` and `big` whose centres are `d` apart.
fn lens(r: f64, big: f64, d: f64) -> f64 {
    if d >= r + big {
        return 0.0;
    }
    if d <= (big - r).abs() {
        return PI * r.min(big).powi(2);
    }
    r * r * ((d * d + r * r - big * big) / (2.0 * d * r)).acos()
        + big * big * ((d * d + big * big - r * r) / (2.0 * d * big)).acos()
        - 0.5 * ((-d + r + big) * (d + r - big) * (d - r + big) * (d + r + big)).sqrt()
}

/// Area of a disc at `(cx, cy)` inside the shelled box's footprint.
fn in_footprint(r: f64, cx: f64, cy: f64) -> f64 {
    disc_in_rect(r, -cx, 100.0 - cx, -cy, 100.0 - cy)
}

/// Area of a disc at `(cx, cy)` inside the shelled box's wall section (the
/// footprint less the cavity).
fn in_wall(r: f64, cx: f64, cy: f64) -> f64 {
    in_footprint(r, cx, cy) - disc_in_rect(r, 2.0 - cx, 98.0 - cx, 2.0 - cy, 98.0 - cy)
}

/// Volume a vertical cylinder shares with the shelled box (unit scale):
/// its footprint over the floor (z in 0..2) and its wall section above.
fn in_shell(r: f64, cx: f64, cy: f64, z0: f64, h: f64) -> f64 {
    let (lo, hi) = (z0.max(0.0), (z0 + h).min(50.0));
    let floor = (hi.min(2.0) - lo).max(0.0);
    let wall = (hi - lo.max(2.0)).max(0.0);
    floor * in_footprint(r, cx, cy) + wall * in_wall(r, cx, cy)
}

fn expected(op: BooleanOp, body: f64, tool: f64, common: f64) -> f64 {
    match op {
        BooleanOp::Cut => body - common,
        BooleanOp::Fuse => body + tool - common,
        BooleanOp::Intersect => common,
    }
}

// ── Bodies ──────────────────────────────────────────────────────────────

/// A 100 × 100 × 50 box at the origin, shelled 2 thick with its top open.
fn shelled_box(topo: &mut Topology, s: f64) -> SolidId {
    let solid = make_box(topo, 100.0 * s, 100.0 * s, 50.0 * s).unwrap();
    let top = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            let face = topo.face(f).unwrap();
            matches!(face.surface(), FaceSurface::Plane { normal, .. }
                if normal.z() > 0.9 && !face.is_reversed())
        })
        .unwrap();
    let shelled = shell(topo, solid, 2.0 * s, &[top]).unwrap();
    let volume = solid_volume(topo, shelled, 0.01 * s).unwrap();
    let target = SHELL_VOLUME * s.powi(3);
    assert!(
        (volume - target).abs() <= VOLUME_TOL * target,
        "shell {volume}"
    );
    shelled
}

/// A plate 100 × 100 × 2 with a wall x in 98..100 standing on it to z = 50:
/// the shelled box's floor and one wall, with no holed face anywhere.
fn l_bracket(topo: &mut Topology, s: f64) -> SolidId {
    let plate = make_box(topo, 100.0 * s, 100.0 * s, 2.0 * s).unwrap();
    let wall = make_box(topo, 2.0 * s, 100.0 * s, 48.0 * s).unwrap();
    transform_solid(topo, wall, &Mat4::translation(98.0 * s, 0.0, 2.0 * s)).unwrap();
    boolean(topo, BooleanOp::Fuse, plate, wall).unwrap()
}
const L_BRACKET_VOLUME: f64 = 20_000.0 + 9_600.0;

/// Volume a vertical cylinder at `(cx, 50)` shares with the L-bracket.
fn in_l_bracket(r: f64, cx: f64, z0: f64, h: f64) -> f64 {
    let (lo, hi) = (z0.max(0.0), (z0 + h).min(50.0));
    let floor = (hi.min(2.0) - lo).max(0.0);
    let wall = (hi - lo.max(2.0)).max(0.0);
    floor * in_footprint(r, cx, 50.0) + wall * disc_in_rect(r, 98.0 - cx, 100.0 - cx, -50.0, 50.0)
}

/// A vertical cylinder of radius `r` from `z0` to `z0 + h` at `(cx, cy)`.
fn cylinder_at(
    topo: &mut Topology,
    s: f64,
    r: f64,
    (cx, cy): (f64, f64),
    z0: f64,
    h: f64,
) -> SolidId {
    let solid = make_cylinder(topo, r * s, h * s).unwrap();
    transform_solid(topo, solid, &Mat4::translation(cx * s, cy * s, z0 * s)).unwrap();
    solid
}

// ── The acceptance bar ──────────────────────────────────────────────────

/// The result is a small analytic body with at least `cylinders` cylinder
/// faces and no spline face, validates, tessellates watertight, and has the
/// closed-form `volume`.
fn assert_exact(
    topo: &Topology,
    solid: SolidId,
    volume: f64,
    s: f64,
    cylinders: usize,
    what: &str,
) {
    let faces = solid_faces(topo, solid).unwrap();
    let count = |tag: &str| {
        faces
            .iter()
            .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == tag)
            .count()
    };
    assert!(faces.len() <= 64, "{what}: {} faces", faces.len());
    assert!(
        count("cylinder") >= cylinders,
        "{what}: {} cylinder faces, expected at least {cylinders}",
        count("cylinder")
    );
    assert_eq!(
        count("plane") + count("cylinder"),
        faces.len(),
        "{what}: a face is neither a plane nor a cylinder"
    );

    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: {:?}", report.issues);

    let mesh = tessellate_solid(topo, solid, 0.01 * s).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh edges");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold mesh edges"
    );

    let measured = solid_volume(topo, solid, 0.01 * s).unwrap();
    assert!(
        (measured - volume).abs() <= VOLUME_TOL * volume,
        "{what}: volume {measured} vs closed form {volume}"
    );
}

fn classify(
    topo: &Topology,
    solid: SolidId,
    s: f64,
    (x, y, z): (f64, f64, f64),
) -> PointClassification {
    classify_point(
        topo,
        solid,
        Point3::new(x * s, y * s, z * s),
        &ClassifyOptions::default(),
    )
    .unwrap()
}

/// One boolean of the shelled box with a vertical cylinder, held to
/// [`assert_exact`].
fn shell_case(
    s: f64,
    r: f64,
    at: (f64, f64),
    (z0, h): (f64, f64),
    op: BooleanOp,
) -> (Topology, SolidId) {
    let what = format!("shell {op:?} r={r} at {at:?} z0={z0} h={h} scale {s}");
    let mut topo = Topology::new();
    let body = shelled_box(&mut topo, s);
    let tool = cylinder_at(&mut topo, s, r, at, z0, h);
    let result = boolean(&mut topo, op, body, tool).unwrap_or_else(|e| panic!("{what}: {e}"));
    let volume = expected(
        op,
        SHELL_VOLUME,
        PI * r * r * h,
        in_shell(r, at.0, at.1, z0, h),
    );
    assert_exact(&topo, result, volume * s.powi(3), s, 1, &what);
    (topo, result)
}

const THROUGH: (f64, f64) = (-20.0, 100.0);
/// From z = 40 up: crosses the rim and the wall only.
const BLIND_FROM_TOP: (f64, f64) = (40.0, 40.0);
const MID_WALL: (f64, f64) = (99.0, 50.0);

// ── The reported bug ────────────────────────────────────────────────────

/// r = 0.5 fits inside the 2 mm wall, r = 2 is the wall thickness, r = 6
/// swallows it: through and blind from the top, cut and fused, at three
/// scales. Before the fix every case but r = 0.5 was refused.
#[test]
fn round_cutter_on_a_shelled_wall_is_exact() {
    for s in SCALES {
        for r in [0.5, 2.0, 6.0] {
            for span in [THROUGH, BLIND_FROM_TOP] {
                for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                    let (topo, result) = shell_case(s, r, MID_WALL, span, op);
                    // The cutter's axis runs through the wall: air after a
                    // cut, material after a fuse. The wall away from it is
                    // material either way, and so is the wall under a blind
                    // cutter's floor.
                    let in_cutter = classify(&topo, result, s, (99.0, 50.0, 45.0));
                    let wanted = if op == BooleanOp::Cut {
                        PointClassification::Outside
                    } else {
                        PointClassification::Inside
                    };
                    assert_eq!(in_cutter, wanted, "{op:?} r={r} {span:?} scale {s}");
                    assert_eq!(
                        classify(&topo, result, s, (99.0, 75.0, 25.0)),
                        PointClassification::Inside
                    );
                    if span == BLIND_FROM_TOP {
                        assert_eq!(
                            classify(&topo, result, s, (99.0, 50.0, 30.0)),
                            PointClassification::Inside
                        );
                    }
                }
            }
        }
    }
}

/// The same cutters stopping at other depths: above the floor (rim and wall
/// only) and inside it.
#[test]
fn blind_cutters_at_other_depths_are_exact() {
    for s in SCALES {
        for r in [2.0, 6.0] {
            for span in [(25.0, 60.0), (1.0, 60.0)] {
                for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                    shell_case(s, r, MID_WALL, span, op);
                }
            }
        }
    }
}

/// Cutters through two opposite walls in one cut: the one rim carries four
/// crossings of its hole and four of its outline.
#[test]
fn through_cuts_on_two_opposite_walls_in_one_go_are_exact() {
    for s in SCALES {
        for r in [2.0, 6.0] {
            let what = format!("two walls r={r} scale {s}");
            let mut topo = Topology::new();
            let body = shelled_box(&mut topo, s);
            let tools = [
                cylinder_at(&mut topo, s, r, (99.0, 50.0), -20.0, 100.0),
                cylinder_at(&mut topo, s, r, (1.0, 50.0), -20.0, 100.0),
            ];
            let result = compound_cut(&mut topo, body, &tools, BooleanOptions::default())
                .unwrap_or_else(|e| panic!("{what}: {e}"));
            let removed =
                in_shell(r, 99.0, 50.0, -20.0, 100.0) + in_shell(r, 1.0, 50.0, -20.0, 100.0);
            assert_exact(
                &topo,
                result,
                (SHELL_VOLUME - removed) * s.powi(3),
                s,
                2,
                &what,
            );
        }
    }
}

/// Cutters that meet the rim's hole on more than one edge, or only there:
/// over the cavity corner (two hole edges), overlapping the inner side of
/// the wall alone (the hole and nothing of the outline), and a bore nearly
/// as wide as the box (all four hole edges, eight crossings, none of the
/// outline).
#[test]
fn cutters_crossing_the_opening_elsewhere_are_exact() {
    for s in SCALES {
        for op in [BooleanOp::Cut, BooleanOp::Fuse] {
            for span in [THROUGH, BLIND_FROM_TOP] {
                shell_case(s, 3.0, (98.0, 98.0), span, op);
                shell_case(s, 1.5, (97.5, 50.0), span, op);
            }
            shell_case(s, 49.5, (50.0, 50.0), THROUGH, op);
        }
    }
}

// ── Tangent contacts ────────────────────────────────────────────────────

/// r = 1 touches both sides of the wall and crosses neither. The touches
/// are not crossings of the rim's hole, so this change leaves the case as
/// it was: either refused, or accepted with the closed-form volume. (As
/// measured when this was written, the unit-scale and ×10 cuts are accepted
/// with the exact volume and one open mesh edge along the tangent line —
/// the acceptance gate does not look at the mesh — and at ×0.1 the through
/// cut is accepted but `solid_volume` declines to measure it.) What must
/// never happen is an accepted body with a wrong volume.
#[test]
fn a_cutter_tangent_to_both_wall_faces_is_exact_or_refused() {
    for s in SCALES {
        for span in [THROUGH, BLIND_FROM_TOP] {
            for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                let what = format!("tangent {op:?} {span:?} scale {s}");
                let mut topo = Topology::new();
                let body = shelled_box(&mut topo, s);
                let tool = cylinder_at(&mut topo, s, 1.0, MID_WALL, span.0, span.1);
                match boolean(&mut topo, op, body, tool) {
                    Err(OperationsError::ExactOnlyUnattainable) => {}
                    Err(other) => panic!("{what}: untyped refusal {other}"),
                    Ok(result) => {
                        let volume = expected(
                            op,
                            SHELL_VOLUME,
                            PI * span.1,
                            in_shell(1.0, 99.0, 50.0, span.0, span.1),
                        ) * s.powi(3);
                        if let Ok(measured) = solid_volume(&topo, result, 0.01 * s) {
                            assert!(
                                (measured - volume).abs() <= VOLUME_TOL * volume,
                                "{what}: accepted with volume {measured} vs {volume}"
                            );
                        }
                    }
                }
            }
        }
    }
}

/// A cutter tangent to the outer side of the wall that crosses the cavity
/// side leaves a rim pinched to a point. Nothing downstream is qualified on
/// such a face (accepted with a volume 1e-6 off and an open mesh edge when
/// the crossings were first added), so the pair is refused by name.
#[test]
fn a_cutter_tangent_to_the_outline_that_crosses_the_opening_is_refused() {
    for s in SCALES {
        for (r, cx) in [(2.0, 98.0), (3.0, 97.0), (1.5, 98.5)] {
            for span in [THROUGH, BLIND_FROM_TOP] {
                for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                    let what = format!("pinch {op:?} r={r} cx={cx} {span:?} scale {s}");
                    let mut topo = Topology::new();
                    let body = shelled_box(&mut topo, s);
                    let tool = cylinder_at(&mut topo, s, r, (cx, 50.0), span.0, span.1);
                    let refusal = boolean(&mut topo, op, body, tool);
                    assert!(
                        matches!(refusal, Err(OperationsError::ExactOnlyUnattainable)),
                        "{what}: {refusal:?}"
                    );
                }
            }
        }
    }
    // The engine names the reason.
    let mut topo = Topology::new();
    let body = shelled_box(&mut topo, 1.0);
    let tool = cylinder_at(&mut topo, 1.0, 2.0, (98.0, 50.0), 40.0, 40.0);
    let error = remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Cut, body, tool)
        .expect_err("pinched rim");
    assert!(error.to_string().contains("narrows to a point"), "{error}");
}

// ── Round holes ─────────────────────────────────────────────────────────

/// A 40 × 40 × 10 plate with a bore of radius 8 through its centre.
fn bored_plate(topo: &mut Topology, s: f64) -> SolidId {
    let plate = make_box(topo, 40.0 * s, 40.0 * s, 10.0 * s).unwrap();
    let bore = cylinder_at(topo, s, 8.0, (20.0, 20.0), -10.0, 30.0);
    boolean(topo, BooleanOp::Cut, plate, bore).unwrap()
}

/// A cylinder notching the rim of a round bore: the plate's caps are
/// squares with a circular hole, crossed at two points of the circle.
#[test]
fn a_cutter_crossing_a_round_bore_rim_is_exact() {
    let plate_volume = 16_000.0 - PI * 64.0 * 10.0;
    for s in SCALES {
        for (r, off) in [(3.0, 8.0), (3.0, 10.0), (8.0, 8.0)] {
            for (z0, h) in [(-5.0, 30.0), (5.0, 30.0), (-5.0, 10.0)] {
                for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                    let what = format!("bore {op:?} r={r} off={off} z0={z0} scale {s}");
                    let mut topo = Topology::new();
                    let body = bored_plate(&mut topo, s);
                    let tool = cylinder_at(&mut topo, s, r, (20.0 + off, 20.0), z0, h);
                    let result = boolean(&mut topo, op, body, tool)
                        .unwrap_or_else(|e| panic!("{what}: {e}"));
                    let depth = (z0 + h).min(10.0) - z0.max(0.0);
                    let common = depth * (PI * r * r - lens(r, 8.0, off));
                    let volume = expected(op, plate_volume, PI * r * r * h, common);
                    assert_exact(&topo, result, volume * s.powi(3), s, 2, &what);
                }
            }
        }
    }
}

/// Three bores in one plate, one of them notched: the two the cutter does
/// not reach stay whole closed circles on the caps.
#[test]
fn untouched_bores_stay_whole_beside_a_notched_one() {
    for s in SCALES {
        let mut topo = Topology::new();
        let mut body = make_box(&mut topo, 60.0 * s, 40.0 * s, 10.0 * s).unwrap();
        for cx in [15.0, 30.0, 45.0] {
            let bore = cylinder_at(&mut topo, s, 5.0, (cx, 20.0), -10.0, 30.0);
            body = boolean(&mut topo, BooleanOp::Cut, body, bore).unwrap();
        }
        let tool = cylinder_at(&mut topo, s, 2.0, (35.0, 20.0), -5.0, 30.0);
        let result = boolean(&mut topo, BooleanOp::Cut, body, tool).unwrap();
        let volume = 24_000.0 - 3.0 * PI * 25.0 * 10.0 - 10.0 * (PI * 4.0 - lens(2.0, 5.0, 5.0));
        assert_exact(
            &topo,
            result,
            volume * s.powi(3),
            s,
            4,
            &format!("three bores scale {s}"),
        );

        // Each cap keeps the two untouched bores as one closed circle each.
        for face in solid_faces(&topo, result).unwrap() {
            let face = topo.face(face).unwrap();
            let FaceSurface::Plane { normal, .. } = face.surface() else {
                continue;
            };
            if normal.z().abs() < 0.9 {
                continue;
            }
            let closed_circles = face
                .inner_wires()
                .iter()
                .filter(|&&w| {
                    let wire = topo.wire(w).unwrap();
                    wire.edges().len() == 1
                        && topo.edge(wire.edges()[0].edge()).unwrap().is_closed()
                })
                .count();
            assert_eq!(closed_circles, 2, "untouched bores on a cap, scale {s}");
        }
    }
}

/// A tube's end faces are annuli: a circular outline with a circular hole,
/// which the closed-section path handles (`closed_circle_boundary_crossings`
/// read the outline only). A cutter on the bore rim crosses the hole alone.
#[test]
fn a_cutter_on_a_tube_bore_rim_is_exact() {
    let tube_volume = PI * (100.0 - 25.0) * 10.0;
    for s in SCALES {
        for (r, off) in [(2.0, 5.0), (1.5, 5.0), (2.0, 4.0)] {
            for (z0, h) in [(-5.0, 30.0), (5.0, 30.0)] {
                for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                    let what = format!("tube {op:?} r={r} off={off} z0={z0} scale {s}");
                    let mut topo = Topology::new();
                    let outer = make_cylinder(&mut topo, 10.0 * s, 10.0 * s).unwrap();
                    let bore = cylinder_at(&mut topo, s, 5.0, (0.0, 0.0), -10.0, 30.0);
                    let body = boolean(&mut topo, BooleanOp::Cut, outer, bore).unwrap();
                    let tool = cylinder_at(&mut topo, s, r, (off, 0.0), z0, h);
                    let result = boolean(&mut topo, op, body, tool)
                        .unwrap_or_else(|e| panic!("{what}: {e}"));
                    let depth = (z0 + h).min(10.0) - z0.max(0.0);
                    let common = depth * (PI * r * r - lens(r, 5.0, off));
                    let volume = expected(op, tube_volume, PI * r * r * h, common);
                    assert_exact(&topo, result, volume * s.powi(3), s, 3, &what);
                }
            }
        }
    }
}

// ── The defects the rim was hiding ──────────────────────────────────────

/// A cutter through the inner corner of an L-bracket leaves the plate's top
/// face into the wall by less than 1 % of that face's size. The span of the
/// section inside the wall was kept, and the cylinder's band between the
/// plate and the wall top came back as two faces bounding nothing: a body
/// 0.6 % light, validator-clean, accepted. No face here has a hole.
#[test]
fn a_cutter_through_the_inner_corner_of_an_l_bracket_is_exact() {
    for s in SCALES {
        for (r, cx) in [(1.5, 97.5), (3.0, 96.5), (1.0, 97.5)] {
            for span in [THROUGH, BLIND_FROM_TOP] {
                for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                    let what = format!("L-bracket {op:?} r={r} cx={cx} {span:?} scale {s}");
                    let mut topo = Topology::new();
                    let body = l_bracket(&mut topo, s);
                    let tool = cylinder_at(&mut topo, s, r, (cx, 50.0), span.0, span.1);
                    let result = boolean(&mut topo, op, body, tool)
                        .unwrap_or_else(|e| panic!("{what}: {e}"));
                    let volume = expected(
                        op,
                        L_BRACKET_VOLUME,
                        PI * r * r * span.1,
                        in_l_bracket(r, cx, span.0, span.1),
                    );
                    assert_exact(&topo, result, volume * s.powi(3), s, 1, &what);
                }
            }
        }
    }
}

/// A post fused through a slab off-centre keeps one cylinder face whose
/// single hole is the slab's footprint on it, wrapping well over half the
/// wall. The hole came back wound like the outer wire (nine shared edges
/// oriented the same way on both sides), accepted.
#[test]
fn a_post_fused_through_off_centre_keeps_its_wall_oriented() {
    for s in SCALES {
        for (r, cx) in [(6.0, 96.5), (3.0, 98.0), (3.0, 98.5)] {
            let what = format!("post r={r} cx={cx} scale {s}");
            let mut topo = Topology::new();
            let body = l_bracket(&mut topo, s);
            let tool = cylinder_at(&mut topo, s, r, (cx, 50.0), -20.0, 100.0);
            let result = boolean(&mut topo, BooleanOp::Fuse, body, tool)
                .unwrap_or_else(|e| panic!("{what}: {e}"));
            let volume = L_BRACKET_VOLUME + PI * r * r * 100.0 - in_l_bracket(r, cx, -20.0, 100.0);
            assert_exact(
                &topo,
                result,
                volume * s.powi(3),
                s,
                1,
                &format!("L-bracket {what}"),
            );

            shell_case(s, r, (cx, 50.0), THROUGH, BooleanOp::Fuse);
        }
    }
}
