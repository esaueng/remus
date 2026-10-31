//! Closed-form oracles for [`plane_closed_loops_separate`], the gate that
//! sends a plane face carrying several closed section circles (a bolt circle
//! stamped onto a flange cap) to the internal-loops path.
//!
//! Every verdict is decided by hand from centre distances and radii, never by
//! reading the gate's arithmetic back. The cases pin the diagonal geometry
//! that axis-aligned boxes misjudge: a disc's box reaches past the disc at
//! its four 45° corners, so two discs on a diagonal of each other can have
//! overlapping boxes while the discs stay well apart.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::Vec2;
use std::f64::consts::{FRAC_1_SQRT_2, TAU};

const TOL: f64 = 1e-7;

fn frame() -> PlaneFrame {
    PlaneFrame::from_normal_and_point(Vec3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 0.0))
}

fn dummy_pcurve() -> Curve2D {
    Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap())
}

fn circle(cx: f64, cy: f64, r: f64) -> Circle3D {
    Circle3D::new(Point3::new(cx, cy, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap()
}

fn section_circle(cx: f64, cy: f64, r: f64) -> SectionEdge {
    let curve = circle(cx, cy, r);
    let seam = Point3::new(cx + r, cy, 0.0);
    let t0 = curve.project(seam);
    SectionEdge {
        curve_3d: EdgeCurve::Circle(curve),
        trim: Some((t0, t0 + TAU)),
        pcurve_a: dummy_pcurve(),
        pcurve_b: dummy_pcurve(),
        start: seam,
        end: seam,
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: None,
    }
}

fn line_edge(frame: &PlaneFrame, a: Point3, b: Point3) -> OrientedPCurveEdge {
    let (ua, ub) = (frame.project(a), frame.project(b));
    OrientedPCurveEdge {
        curve_3d: EdgeCurve::Line,
        trim: None,
        pcurve: Curve2D::Line(
            Line2D::new(ua, Vec2::new(ub.x() - ua.x(), ub.y() - ua.y())).unwrap(),
        ),
        start_uv: ua,
        end_uv: ub,
        start_3d: a,
        end_3d: b,
        forward: true,
        source_edge_idx: None,
        pave_block_id: None,
        source_topo_edge: None,
    }
}

/// A closed polygon of line edges through `corners` (z = 0).
fn polygon(frame: &PlaneFrame, corners: &[(f64, f64)]) -> Vec<OrientedPCurveEdge> {
    let n = corners.len();
    (0..n)
        .map(|i| {
            let (a, b) = (corners[i], corners[(i + 1) % n]);
            line_edge(
                frame,
                Point3::new(a.0, a.1, 0.0),
                Point3::new(b.0, b.1, 0.0),
            )
        })
        .collect()
}

/// The square `[-h, h]²` as a counter-clockwise outer wire.
fn square(frame: &PlaneFrame, h: f64) -> Vec<OrientedPCurveEdge> {
    polygon(frame, &[(-h, -h), (h, -h), (h, h), (-h, h)])
}

/// A hole wire made of `arcs` arcs of one circle about the origin.
fn hole(frame: &PlaneFrame, r: f64, arcs: usize) -> Vec<OrientedPCurveEdge> {
    #[allow(clippy::cast_precision_loss)]
    let step = TAU / arcs as f64;
    (0..arcs)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            let (a0, a1) = (step * i as f64, step * (i + 1) as f64);
            let p0 = Point3::new(r * a0.cos(), r * a0.sin(), 0.0);
            let p1 = Point3::new(r * a1.cos(), r * a1.sin(), 0.0);
            OrientedPCurveEdge {
                curve_3d: EdgeCurve::Circle(circle(0.0, 0.0, r)),
                trim: Some((a0, a1)),
                pcurve: dummy_pcurve(),
                start_uv: frame.project(p0),
                end_uv: frame.project(p1),
                start_3d: p0,
                end_3d: p1,
                forward: true,
                source_edge_idx: None,
                pave_block_id: None,
                source_topo_edge: None,
            }
        })
        .collect()
}

fn separate(sections: &[SectionEdge], holes: &[Vec<OrientedPCurveEdge>]) -> bool {
    let frame = frame();
    plane_closed_loops_separate(sections, &frame, &square(&frame, 50.0), holes, TOL)
}

/// The flange rim cap: an r24 opening and bolt holes r3 on the r34 circle.
/// The 45° hole's box corner sits at 34/√2 − 3 ≈ 21.04 < 24 on both axes,
/// inside the opening's box, yet the discs are 34 − 3 − 24 = 7 apart.
#[test]
fn diagonal_bolt_hole_beside_the_opening_is_separate() {
    let frame = frame();
    let opening = vec![hole(&frame, 24.0, 1)];
    let d = 34.0 * FRAC_1_SQRT_2;
    for (cx, cy) in [(d, d), (-d, d), (-d, -d), (d, -d)] {
        assert!(
            separate(
                &[section_circle(34.0, 0.0, 3.0), section_circle(cx, cy, 3.0)],
                &opening
            ),
            "bolt hole at ({cx}, {cy}) is 7 clear of the opening"
        );
    }
    // The same answer when the opening's circle arrives as two arcs.
    assert!(separate(
        &[section_circle(d, d, 3.0), section_circle(-d, -d, 3.0)],
        &[hole(&frame, 24.0, 2)],
    ));
}

/// A disc that overlaps the opening, or one that merely touches it, must
/// still route to the wire builder.
#[test]
fn disc_reaching_the_opening_is_not_separate() {
    let frame = frame();
    let opening = vec![hole(&frame, 24.0, 1)];
    let far = section_circle(-34.0, 0.0, 3.0);
    // Centre 26 out: the disc reaches r = 23 < 24.
    let d = 26.0 * FRAC_1_SQRT_2;
    assert!(!separate(
        &[far.clone(), section_circle(d, d, 3.0)],
        &opening
    ));
    // Centre exactly 27 out: tangent to the opening, no clearance.
    let d = 27.0 * FRAC_1_SQRT_2;
    assert!(!separate(
        &[far.clone(), section_circle(d, d, 3.0)],
        &opening
    ));
    // Centre 27.5 out: half a unit clear.
    let d = 27.5 * FRAC_1_SQRT_2;
    assert!(separate(&[far, section_circle(d, d, 3.0)], &opening));
}

/// Two section discs on a diagonal of each other: boxes overlap on both axes
/// while the discs are 4.5·√2 − 6 ≈ 0.36 apart. Pulled to 4·√2 ≈ 5.66 < 6
/// apart they overlap.
#[test]
fn diagonal_section_discs_compare_by_distance() {
    let a = section_circle(30.0, 30.0, 3.0);
    assert!(separate(&[a.clone(), section_circle(34.5, 25.5, 3.0)], &[]));
    assert!(!separate(&[a, section_circle(34.0, 26.0, 3.0)], &[]));
}

/// A disc must sit inside the outer outline with clearance: 3 clear of the
/// square's side passes, touching it does not.
#[test]
fn disc_against_the_outer_outline() {
    assert!(separate(
        &[
            section_circle(45.0, 0.0, 2.0),
            section_circle(-45.0, 0.0, 2.0)
        ],
        &[],
    ));
    assert!(!separate(
        &[
            section_circle(48.0, 0.0, 2.0),
            section_circle(-45.0, 0.0, 2.0)
        ],
        &[],
    ));
    // A diamond |x| + |y| ≤ 50 and a disc r3 at (22, 22): its box corner
    // (25, 25) lies ON the diamond's side, but the disc is (50 − 44)/√2 ≈ 4.24
    // from that side, 1.24 clear. At (23, 23) it is 2.83 < 3 from the side.
    let frame = frame();
    let diamond = polygon(
        &frame,
        &[(50.0, 0.0), (0.0, 50.0), (-50.0, 0.0), (0.0, -50.0)],
    );
    let in_diamond = |cx: f64, cy: f64| {
        plane_closed_loops_separate(
            &[section_circle(cx, cy, 3.0), section_circle(-30.0, 0.0, 3.0)],
            &frame,
            &diamond,
            &[],
            TOL,
        )
    };
    assert!(in_diamond(22.0, 22.0));
    assert!(!in_diamond(23.0, 23.0));
}

/// A star-shaped bore (an internal gear's tooth outline, here an 80-gon
/// alternating r 19.5 and r 21) inside two coaxial section circles r 25 and
/// r 27: a tube cut into the ring's web. The circles nest (27 > 25 + 0) and
/// both contain every polygon vertex, so the loops do not cross.
#[test]
fn nested_section_circles_around_a_star_bore_pass() {
    let frame = frame();
    let star: Vec<(f64, f64)> = (0..80)
        .map(|k| {
            let r = if k % 2 == 0 { 19.5 } else { 21.0 };
            let a = TAU * f64::from(k) / 80.0;
            (r * a.cos(), r * a.sin())
        })
        .collect();
    let bore = vec![polygon(&frame, &star)];
    assert!(separate(
        &[
            section_circle(0.0, 0.0, 25.0),
            section_circle(0.0, 0.0, 27.0)
        ],
        &bore
    ));
    // The star's box corners reach 21·√2 ≈ 29.7 > 25: a box test alone
    // would refuse, the vertex test must not.
    assert!(separate(&[section_circle(0.0, 0.0, 25.0)], &bore));
}

/// Nesting needs clearance: a circle through the star's tips (r 21), or one
/// crossing it (r 20), still straddles the bore, and two circles that cross
/// each other are not nested.
#[test]
fn circles_meeting_the_bore_or_each_other_do_not_pass() {
    let frame = frame();
    let star: Vec<(f64, f64)> = (0..80)
        .map(|k| {
            let r = if k % 2 == 0 { 19.5 } else { 21.0 };
            let a = TAU * f64::from(k) / 80.0;
            (r * a.cos(), r * a.sin())
        })
        .collect();
    let bore = vec![polygon(&frame, &star)];
    for r in [20.0, 21.0] {
        assert!(!separate(
            &[section_circle(0.0, 0.0, r), section_circle(0.0, 0.0, 27.0)],
            &bore
        ));
    }
    // r 25 about the origin and r 5 about (22, 0) cross: 22 + 5 > 25.
    assert!(!separate(
        &[
            section_circle(0.0, 0.0, 25.0),
            section_circle(22.0, 0.0, 5.0)
        ],
        &[]
    ));
    // Fully inside it (16 + 5 < 25 − tol) they nest.
    assert!(separate(
        &[
            section_circle(0.0, 0.0, 25.0),
            section_circle(16.0, 0.0, 5.0)
        ],
        &[]
    ));
}
