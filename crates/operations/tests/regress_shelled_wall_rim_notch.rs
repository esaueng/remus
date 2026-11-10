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
/// it was: either refused, or accepted with the closed-form volume. What
/// must never happen is an accepted body with a wrong volume.
///
/// Every accepted result is held to the bar without going through
/// `solid_volume`: it validates, the volume its mesh encloses at two
/// deflections is the closed form to within the deflection times the area
/// of its curved faces, and it is air or material where it should be. Two
/// known defects are named rather than skipped:
///
/// - the cuts mesh with one non-manifold edge along the line where the
///   cutter touches the wall (the acceptance gate does not read the mesh);
/// - at scale 0.1, `solid_volume` at the usual deflection (0.01 · s) fails
///   to converge on the through cut and fuse; at 0.002 · s it measures the
///   closed form. Follow-up: make the volume path converge there.
#[test]
fn a_cutter_tangent_to_both_wall_faces_is_exact_or_refused() {
    for s in SCALES {
        for span in [THROUGH, BLIND_FROM_TOP] {
            for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                let what = format!("tangent {op:?} {span:?} scale {s}");
                let mut topo = Topology::new();
                let body = shelled_box(&mut topo, s);
                let tool = cylinder_at(&mut topo, s, 1.0, MID_WALL, span.0, span.1);
                let result = match boolean(&mut topo, op, body, tool) {
                    Err(OperationsError::ExactOnlyUnattainable) => continue,
                    Err(other) => panic!("{what}: untyped refusal {other}"),
                    Ok(result) => result,
                };
                let volume = expected(
                    op,
                    SHELL_VOLUME,
                    PI * span.1,
                    in_shell(1.0, 99.0, 50.0, span.0, span.1),
                ) * s.powi(3);

                // Known defect: one non-manifold mesh edge on the tangent
                // line of a cut.
                let known_mesh_defects = usize::from(op == BooleanOp::Cut);
                assert_mesh_geometry(&topo, result, volume, s, known_mesh_defects, &what);

                let in_cutter = classify(&topo, result, s, (99.0, 50.0, 45.0));
                let wanted = if op == BooleanOp::Cut {
                    PointClassification::Outside
                } else {
                    PointClassification::Inside
                };
                assert_eq!(in_cutter, wanted, "{what}: in the cutter");
                assert_eq!(
                    classify(&topo, result, s, (99.0, 75.0, 25.0)),
                    PointClassification::Inside,
                    "{what}: in the wall"
                );
                if span == BLIND_FROM_TOP {
                    assert_eq!(
                        classify(&topo, result, s, (99.0, 50.0, 30.0)),
                        PointClassification::Inside,
                        "{what}: under the cutter's floor"
                    );
                }

                let measured = match solid_volume(&topo, result, 0.01 * s) {
                    Ok(measured) => measured,
                    // Known: see the doc comment. Only this case, only this
                    // error, and the finer deflection must still measure it.
                    Err(error) if s < 1.0 && span == THROUGH => {
                        assert!(
                            format!("{error:?}").contains("ConvergenceFailure"),
                            "{what}: {error:?}"
                        );
                        solid_volume(&topo, result, 0.002 * s)
                            .unwrap_or_else(|e| panic!("{what}: {e:?} at the finer deflection"))
                    }
                    Err(error) => panic!("{what}: solid_volume {error:?}"),
                };
                assert!(
                    (measured - volume).abs() <= VOLUME_TOL * volume,
                    "{what}: accepted with volume {measured} vs {volume}"
                );
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

// ── Independent geometry checks ─────────────────────────────────────────

/// Signed volume enclosed by a triangle mesh (divergence theorem).
fn mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| {
                let p = mesh.positions[i as usize];
                remus_math::vec::Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

fn mesh_area(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize]);
            0.5 * (b - a).cross(c - a).length()
        })
        .sum()
}

/// Area of the result's non-planar faces, meshed at `deflection`: the only
/// faces whose mesh departs from the surface.
fn curved_area(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| !topo.face(f).unwrap().surface().is_planar())
        .map(|f| mesh_area(&remus_operations::tessellate::tessellate(topo, f, deflection).unwrap()))
        .sum()
}

/// What an accepted result looks like when it is measured without
/// `solid_volume`: validation, and at two deflections the mesh's open and
/// non-manifold edges and the volume it encloses, held to the closed form
/// within the volume between the curved faces and their mesh (at most the
/// deflection times their area).
///
/// `known_defects` is the number of open or non-manifold mesh edges a named
/// defect leaves (zero for an exact result).
fn assert_mesh_geometry(
    topo: &Topology,
    solid: SolidId,
    volume: f64,
    s: f64,
    known_defects: usize,
    what: &str,
) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: {:?}", report.issues);
    for deflection in [0.01 * s, 0.002 * s] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        let (open, non_manifold) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            open + non_manifold <= known_defects,
            "{what}: {open} open and {non_manifold} non-manifold mesh edges at deflection \
             {deflection}"
        );
        let measured = mesh_volume(&mesh).abs();
        let bound = 1.5 * deflection * curved_area(topo, solid, deflection) + VOLUME_TOL * volume;
        assert!(
            (measured - volume).abs() <= bound,
            "{what}: mesh volume {measured} vs {volume} (bound {bound}) at deflection {deflection}"
        );
    }
}

// ── Eccentric sections against a straight hole edge ─────────────────────

/// Plate thickness of the eccentric cases.
const SLAB: f64 = 0.05;
/// How far the section's major vertex reaches past the slot's near end on
/// the plate's top face: 100 × the default linear tolerance at unit scale.
const POKE: f64 = 1e-5;
/// The slot: x from `a − POKE` to `a + SLOT_LENGTH`, y within ±`SLOT_HALF`.
const SLOT_LENGTH: f64 = 3.0;
const SLOT_HALF: f64 = 0.25;

/// A slotted plate and a radius-1 cylinder tilted so that its section with
/// the plate's top face (z = `SLAB`) is an ellipse of semi-axes `(a, 1)`
/// centred on the origin, whose major vertex reaches `poke` past the slot's
/// near end (a negative `poke` stops short of it). Going down through the plate the section slides toward the
/// slot (by `tan θ` per unit height), so the cylinder also cuts the slot's
/// walls. Returns the plate, the cylinder and the cylinder's length.
fn eccentric_case(topo: &mut Topology, s: f64, a: f64, poke: f64) -> (SolidId, SolidId, f64) {
    let tan = (a * a - 1.0).sqrt();
    let x_min = -a - 1.0;
    let x_max = a + SLOT_LENGTH + SLAB * tan + 2.0;
    let plate = make_box(topo, (x_max - x_min) * s, 6.0 * s, SLAB * s).unwrap();
    transform_solid(topo, plate, &Mat4::translation(x_min * s, -3.0 * s, 0.0)).unwrap();
    let slot = make_box(
        topo,
        (SLOT_LENGTH + poke) * s,
        2.0 * SLOT_HALF * s,
        (SLAB + 2.0) * s,
    )
    .unwrap();
    transform_solid(
        topo,
        slot,
        &Mat4::translation((a - poke) * s, -SLOT_HALF * s, -s),
    )
    .unwrap();
    let plate = boolean(topo, BooleanOp::Cut, plate, slot).unwrap();

    // Axis direction (−sin θ, 0, cos θ) with cos θ = 1/a, through
    // (0, 0, SLAB); the caps sit at z = −2 and z = SLAB + 2, clear of the
    // plate. The seam is turned to the side (+y), away from the slot.
    let theta = (1.0 / a).acos();
    let below = (SLAB + 2.0) * a;
    let length = (SLAB + 4.0) * a;
    let cylinder = make_cylinder(topo, s, length * s).unwrap();
    let placement = Mat4::translation(0.0, 0.0, SLAB * s)
        * Mat4::rotation_y(-theta)
        * Mat4::translation(0.0, 0.0, -below * s)
        * Mat4::rotation_z(PI / 2.0);
    transform_solid(topo, cylinder, &placement).unwrap();
    (plate, cylinder, length)
}

/// Adaptive Simpson quadrature of a function that is smooth between the
/// given breakpoints.
fn integrate(f: &dyn Fn(f64) -> f64, breaks: &[f64], tol: f64) -> f64 {
    fn simpson(a: f64, b: f64, fa: f64, fm: f64, fb: f64) -> f64 {
        (b - a) / 6.0 * (fa + 4.0 * fm + fb)
    }
    fn recurse(
        f: &dyn Fn(f64) -> f64,
        (a, b): (f64, f64),
        (fa, fm, fb): (f64, f64, f64),
        whole: f64,
        tol: f64,
        depth: u32,
    ) -> f64 {
        let m = 0.5 * (a + b);
        let (lm, rm) = (0.5 * (a + m), 0.5 * (m + b));
        let (flm, frm) = (f(lm), f(rm));
        let left = simpson(a, m, fa, flm, fm);
        let right = simpson(m, b, fm, frm, fb);
        if depth == 0 || (left + right - whole).abs() <= 15.0 * tol {
            return left + right + (left + right - whole) / 15.0;
        }
        recurse(f, (a, m), (fa, flm, fm), left, 0.5 * tol, depth - 1)
            + recurse(f, (m, b), (fm, frm, fb), right, 0.5 * tol, depth - 1)
    }
    breaks
        .windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let (fa, fm, fb) = (f(a), f(0.5 * (a + b)), f(b));
            let whole = simpson(a, b, fa, fm, fb);
            recurse(f, (a, b), (fa, fm, fb), whole, tol, 40)
        })
        .sum()
}

/// Volume (unit scale) the tilted cylinder of [`eccentric_case`] shares
/// with the slotted plate: by Cavalieri each horizontal section of the
/// cylinder is an ellipse of area `π a`, less what of it lies over the
/// slot — the unit disc inside the slot's rectangle mapped to the
/// ellipse's chart, times `a`.
fn eccentric_common(a: f64, poke: f64) -> f64 {
    let tan = (a * a - 1.0).sqrt();
    let (x0, x1) = (a - poke, a + SLOT_LENGTH);
    let centre = |z: f64| (SLAB - z) * tan;
    let over_slot = |z: f64| {
        let c = centre(z);
        a * disc_in_rect(1.0, (x0 - c) / a, (x1 - c) / a, -SLOT_HALF, SLOT_HALF)
    };
    // Where the slot's corners or its far end meet the section.
    let reach = (1.0 - SLOT_HALF * SLOT_HALF).sqrt() * a;
    let mut breaks = vec![0.0, SLAB];
    for c in [
        x0 - a,
        x1 - a,
        x0 - reach,
        x0 + reach,
        x1 - reach,
        x1 + reach,
    ] {
        let z = SLAB - c / tan;
        if z > 0.0 && z < SLAB {
            breaks.push(z);
        }
    }
    breaks.sort_by(f64::total_cmp);
    let slot = integrate(&over_slot, &breaks, 1e-15 * a);
    PI * a * SLAB - slot
}

fn eccentric_plate_volume(a: f64, poke: f64) -> f64 {
    let tan = (a * a - 1.0).sqrt();
    let length = 2.0 * a + SLOT_LENGTH + SLAB * tan + 3.0;
    (length * 6.0 - (SLOT_LENGTH + poke) * 2.0 * SLOT_HALF) * SLAB
}

/// One eccentric boolean: the operation's result, and the closed-form
/// volume (scaled).
fn eccentric_boolean(
    s: f64,
    a: f64,
    poke: f64,
    op: BooleanOp,
) -> (Topology, Result<SolidId, OperationsError>, f64) {
    let mut topo = Topology::new();
    let (plate, cylinder, length) = eccentric_case(&mut topo, s, a, poke);
    let volume = expected(
        op,
        eccentric_plate_volume(a, poke),
        PI * length,
        eccentric_common(a, poke),
    ) * s.powi(3);
    let result = boolean(&mut topo, op, plate, cylinder);
    (topo, result, volume)
}

/// An accepted eccentric result: exact by [`assert_exact`]'s bar (bar the
/// face census: the tunnel is one cylinder face), the same by the mesh, and
/// air or material where the tunnel runs through the plate.
fn assert_eccentric_exact(
    topo: &Topology,
    result: SolidId,
    volume: f64,
    s: f64,
    (a, op): (f64, BooleanOp),
    what: &str,
) {
    let measured = solid_volume(topo, result, 0.002 * s).unwrap();
    assert!(
        (measured - volume).abs() <= 1e-9 * volume,
        "{what}: volume {measured} vs slicing integral {volume}"
    );
    assert_mesh_geometry(topo, result, volume, s, 0, what);
    // On the cylinder's axis half-way through the plate, and in the plate
    // beside the tunnel.
    let tan = (a * a - 1.0).sqrt();
    let on_axis = classify(topo, result, s, (0.5 * SLAB * tan, 0.0, 0.5 * SLAB));
    let wanted = if op == BooleanOp::Cut {
        PointClassification::Outside
    } else {
        PointClassification::Inside
    };
    assert_eq!(on_axis, wanted, "{what}: on the axis");
    assert_eq!(
        classify(topo, result, s, (0.0, 2.0, 0.5 * SLAB)),
        PointClassification::Inside,
        "{what}: beside the tunnel"
    );
}

/// A tilted cylinder whose section with a slotted plate's top face is an
/// ellipse of axes (a, 1), its major vertex 1e-5 past the slot's near end:
/// 100 tolerances in the plane. Measured on the plane alone that is a
/// crossing; but the plane meets the cylinder at a grazing angle there, and
/// the edge enters the cylinder by only `1e-5 · s / a` (1e-8 for the axes
/// (1000, 1)). Where that is within the tolerance the two crossings and a
/// touch are the same contact, and the pair is refused by name; where it is
/// not, the result is exact. Nothing is accepted wrong.
///
/// The contact used to be measured in the chart that maps the ellipse to
/// the unit circle, scaled by the smaller semi-axis, and read as a touch
/// for (10, 1) at scale 0.1, (100, 1) up to scale 1 and (1000, 1) at every
/// scale: the hole's crossings were dropped and the trim kept spans across
/// the slot. The engine built wrong bodies (volume 12 % to 480 % off),
/// which the acceptance gate refused.
#[test]
fn an_eccentric_section_past_a_hole_edge_is_exact_or_refused() {
    let tol = remus_math::tolerance::Tolerance::new().linear;
    for a in [10.0, 100.0, 1000.0] {
        for s in SCALES {
            for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                let what = format!("eccentric {op:?} axes ({a}, 1) scale {s}");
                let (topo, result, volume) = eccentric_boolean(s, a, POKE, op);
                // How far the slot's near end enters the cylinder.
                let depth = POKE * s / a;
                match result {
                    Ok(result) => {
                        assert_eccentric_exact(&topo, result, volume, s, (a, op), &what);
                    }
                    Err(OperationsError::ExactOnlyUnattainable) => {
                        assert!(depth < 10.0 * tol, "{what}: refused at depth {depth:e}");
                    }
                    Err(other) => panic!("{what}: untyped refusal {other}"),
                }

                // The engine names the grazing contact rather than build a
                // body from half a reading of it.
                if depth <= 0.5 * tol {
                    let mut topo = Topology::new();
                    let (plate, cylinder, _) = eccentric_case(&mut topo, s, a, POKE);
                    let algo_op = if op == BooleanOp::Cut {
                        remus_algo::bop::BooleanOp::Cut
                    } else {
                        remus_algo::bop::BooleanOp::Fuse
                    };
                    let error = remus_algo::gfa::boolean(&mut topo, algo_op, plate, cylinder)
                        .expect_err("a hole edge grazing the cut surface");
                    assert!(
                        error.to_string().contains("cannot be told from a tangency"),
                        "{what}: {error}"
                    );
                }
            }
        }
    }
}

/// The milder eccentricities with the slot's near end well inside the
/// section (1e-3 past the vertex): exact at every scale.
#[test]
fn an_eccentric_section_well_past_a_hole_edge_is_exact() {
    for a in [10.0, 100.0] {
        for s in SCALES {
            for op in [BooleanOp::Cut, BooleanOp::Fuse] {
                let what = format!("eccentric {op:?} axes ({a}, 1) poke 1e-3 scale {s}");
                let (topo, result, volume) = eccentric_boolean(s, a, 1e-3, op);
                let result = result.unwrap_or_else(|e| panic!("{what}: {e}"));
                assert_eccentric_exact(&topo, result, volume, s, (a, op), &what);
            }
        }
    }
}

/// Open: a tilted cylinder that enters the slot only below the plate's top
/// face (its top section stops 0.5 short of the slot) comes out of the
/// engine with validation errors and the wrong volume (the fuse loses most
/// of the cylinder), and is refused by the acceptance gate, on `main` as
/// here. The (1000, 1) cases above also build wrong bodies with the slot's
/// end well inside the section (0.1 past the vertex), which is what still
/// refuses them at scale 10; whether that is this defect is not established.
#[test]
#[ignore = "open: a tilted cylinder entering a through slot below the top face builds a wrong body"]
fn a_tilted_cylinder_entering_a_slot_below_the_top_face_is_exact() {
    for s in SCALES {
        for op in [BooleanOp::Cut, BooleanOp::Fuse] {
            let what = format!("slot below the top {op:?} scale {s}");
            let (topo, result, volume) = eccentric_boolean(s, 10.0, -0.5, op);
            let result = result.unwrap_or_else(|e| panic!("{what}: {e}"));
            assert_eccentric_exact(&topo, result, volume, s, (10.0, op), &what);
        }
    }
}
