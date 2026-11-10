//! Hand-placed oracles for [`split_hole_at_section_ends`]: a plane-face hole
//! is split exactly where a section ends strictly inside one of its edges,
//! and nowhere else.
//!
//! The expected split points are the section ends themselves, placed by
//! coordinates on a square hole's sides; "untouched" holes are ones no
//! section end lies on, by construction.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::Vec2;

const TOL: f64 = 1e-7;
const SCALES: [f64; 3] = [0.1, 1.0, 10.0];

fn frame() -> PlaneFrame {
    PlaneFrame::from_normal_and_point(Vec3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 0.0))
}

fn plane() -> FaceSurface {
    FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 0.0,
    }
}

fn point(x: f64, y: f64) -> Point3 {
    Point3::new(x, y, 0.0)
}

/// A straight hole edge from `a` to `b`, its pcurve and UVs in [`frame`].
fn hole_edge(a: Point3, b: Point3) -> OrientedPCurveEdge {
    let frame = frame();
    let (ua, ub) = (frame.project(a), frame.project(b));
    let direction = ub - ua;
    let length = direction.length();
    let pcurve = Curve2D::Line(
        Line2D::new(
            ua,
            Vec2::new(direction.x() / length, direction.y() / length),
        )
        .unwrap(),
    );
    OrientedPCurveEdge {
        curve_3d: EdgeCurve::Line,
        trim: None,
        pcurve,
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

/// The square hole `[-h, h]²` about `(cx, cy)`, wound clockwise.
fn square_hole(h: f64, (cx, cy): (f64, f64)) -> Vec<OrientedPCurveEdge> {
    let corners = [
        point(cx - h, cy - h),
        point(cx - h, cy + h),
        point(cx + h, cy + h),
        point(cx + h, cy - h),
    ];
    (0..4)
        .map(|i| hole_edge(corners[i], corners[(i + 1) % 4]))
        .collect()
}

fn section(
    curve_3d: EdgeCurve,
    trim: Option<(f64, f64)>,
    start: Point3,
    end: Point3,
) -> SectionEdge {
    let dummy = Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
    SectionEdge {
        curve_3d,
        trim,
        pcurve_a: dummy.clone(),
        pcurve_b: dummy,
        start,
        end,
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: None,
    }
}

/// The arc of the circle of radius `r` about `(cx, 0)` on the `x > x0` side
/// of the line `x = x0`, from its lower crossing to its upper one.
fn arc_right_of(cx: f64, r: f64, x0: f64) -> SectionEdge {
    let circle = Circle3D::new(point(cx, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
    let half = (r * r - (x0 - cx) * (x0 - cx)).sqrt();
    let (start, end) = (point(x0, -half), point(x0, half));
    let (t0, t1) = (circle.project(start), circle.project(end));
    let t1 = if t1 < t0 {
        t1 + std::f64::consts::TAU
    } else {
        t1
    };
    section(EdgeCurve::Circle(circle), Some((t0, t1)), start, end)
}

fn split(hole: &[OrientedPCurveEdge], sections: &[SectionEdge]) -> Option<Vec<OrientedPCurveEdge>> {
    split_hole_at_section_ends(hole, sections, &frame(), &plane(), TOL).unwrap()
}

/// The pieces chain end to start round the loop, and cover the same four
/// sides.
fn assert_closed_loop(pieces: &[OrientedPCurveEdge]) {
    for (i, piece) in pieces.iter().enumerate() {
        let next = &pieces[(i + 1) % pieces.len()];
        assert!(
            (piece.end_3d - next.start_3d).length() == 0.0,
            "piece {i} ends at {:?}, the next starts at {:?}",
            piece.end_3d,
            next.start_3d
        );
        assert!((piece.end_uv - next.start_uv).length() < 1e-12);
        assert!(matches!(piece.curve_3d, EdgeCurve::Line));
    }
}

#[test]
fn a_hole_edge_is_split_at_every_section_end_inside_it() {
    for s in SCALES {
        let hole = square_hole(2.0 * s, (0.0, 0.0));
        // An arc bulging away from the hole's x = 2 side, ending on it at
        // y = ±√(1.5² − 1²), and a line ending on its y = 2 side at x = 0.5.
        let arc = arc_right_of(3.0 * s, 1.5 * s, 2.0 * s);
        let line = section(
            EdgeCurve::Line,
            None,
            point(0.5 * s, 2.0 * s),
            point(0.5 * s, 9.0 * s),
        );
        let pieces = split(&hole, &[arc.clone(), line.clone()]).expect("split hole");
        assert_eq!(pieces.len(), 4 + 3, "scale {s}");
        assert_closed_loop(&pieces);
        for end in [arc.start, arc.end, line.start] {
            assert!(
                pieces
                    .iter()
                    .any(|p| (p.end_3d - end).length() < 1e-12 * s.max(1.0)),
                "scale {s}: no vertex at section end {end:?}"
            );
        }
        // The far end of the line is nowhere near the hole.
        assert!(pieces.iter().all(|p| (p.end_3d - line.end).length() > s));
    }
}

#[test]
fn a_hole_no_section_ends_on_stays_whole() {
    for s in SCALES {
        let hole = square_hole(2.0 * s, (0.0, 0.0));
        // A section clear of the hole, one that ends at a hole CORNER (a
        // vertex already), one that crosses the hole's box without ending
        // on it, and a closed circle round it.
        let clear = section(
            EdgeCurve::Line,
            None,
            point(5.0 * s, 0.0),
            point(9.0 * s, 0.0),
        );
        let corner = section(
            EdgeCurve::Line,
            None,
            point(2.0 * s, 2.0 * s),
            point(9.0 * s, 9.0 * s),
        );
        let across = section(EdgeCurve::Line, None, point(-9.0 * s, s), point(9.0 * s, s));
        let ring = Circle3D::new(point(0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0 * s).unwrap();
        let seam = point(5.0 * s, 0.0);
        let closed = section(
            EdgeCurve::Circle(ring),
            Some((0.0, std::f64::consts::TAU)),
            seam,
            seam,
        );
        for sections in [
            vec![],
            vec![clear.clone()],
            vec![corner.clone()],
            vec![across.clone()],
            vec![closed.clone()],
            vec![clear, corner, across, closed],
        ] {
            assert!(split(&hole, &sections).is_none(), "scale {s}");
        }
    }
}

#[test]
fn only_the_hole_a_section_reaches_is_split() {
    for s in SCALES {
        // A row of openings, as on a baseplate; the arc ends on the middle
        // one alone.
        let holes: Vec<_> = [-10.0, 0.0, 10.0]
            .into_iter()
            .map(|cx| square_hole(2.0 * s, (cx * s, 0.0)))
            .collect();
        let arc = arc_right_of(3.0 * s, 1.5 * s, 2.0 * s);
        let split: Vec<_> = holes
            .iter()
            .map(|hole| {
                split_hole_at_section_ends(
                    hole,
                    std::slice::from_ref(&arc),
                    &frame(),
                    &plane(),
                    TOL,
                )
            })
            .collect::<Result<_, _>>()
            .unwrap();
        assert!(split[0].is_none() && split[2].is_none(), "scale {s}");
        assert_eq!(split[1].as_ref().map(Vec::len), Some(6), "scale {s}");
    }
}
