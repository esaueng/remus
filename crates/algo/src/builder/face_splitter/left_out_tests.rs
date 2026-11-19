//! Hand-placed oracles for [`sections_left_out`], the check that sends an
//! established partition missing a section to the certified re-partition
//! (B265 and its review).
//!
//! Each case builds sub-face wires and sections whose relation is known by
//! construction: a section the partition carries as an edge, a section that
//! only touches an edge at its midpoint, a section running beside a long
//! straight edge. Nothing reads the check's arithmetic back.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::Vec2;
use std::f64::consts::{PI, TAU};

const TOL: f64 = 1e-7;

fn dummy_pcurve() -> Curve2D {
    Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap())
}

fn edge(curve_3d: EdgeCurve, trim: Option<(f64, f64)>, a: Point3, b: Point3) -> OrientedPCurveEdge {
    OrientedPCurveEdge {
        curve_3d,
        trim,
        pcurve: dummy_pcurve(),
        start_uv: Point2::new(0.0, 0.0),
        end_uv: Point2::new(0.0, 0.0),
        start_3d: a,
        end_3d: b,
        forward: true,
        source_edge_idx: None,
        pave_block_id: None,
        source_topo_edge: None,
    }
}

fn line(a: (f64, f64, f64), b: (f64, f64, f64)) -> OrientedPCurveEdge {
    edge(
        EdgeCurve::Line,
        None,
        Point3::new(a.0, a.1, a.2),
        Point3::new(b.0, b.1, b.2),
    )
}

/// The horizontal circle of radius `r` about `(cx, 0, z)`, whose parameter
/// 0 is at `(cx + r, 0, z)`.
fn circle(cx: f64, z: f64, r: f64) -> Circle3D {
    Circle3D::new_with_ref(
        Point3::new(cx, 0.0, z),
        Vec3::new(0.0, 0.0, 1.0),
        r,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

/// The whole circle as one closed sub-face edge.
fn circle_edge(c: &Circle3D) -> OrientedPCurveEdge {
    let p = c.evaluate(0.0);
    edge(EdgeCurve::Circle(c.clone()), Some((0.0, TAU)), p, p)
}

fn section(curve_3d: EdgeCurve, trim: Option<(f64, f64)>, a: Point3, b: Point3) -> SectionEdge {
    SectionEdge {
        curve_3d,
        trim,
        pcurve_a: dummy_pcurve(),
        pcurve_b: dummy_pcurve(),
        start: a,
        end: b,
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: None,
    }
}

fn circle_section(c: &Circle3D) -> SectionEdge {
    let p = c.evaluate(0.0);
    section(EdgeCurve::Circle(c.clone()), Some((0.0, TAU)), p, p)
}

fn line_section(a: (f64, f64, f64), b: (f64, f64, f64)) -> SectionEdge {
    section(
        EdgeCurve::Line,
        None,
        Point3::new(a.0, a.1, a.2),
        Point3::new(b.0, b.1, b.2),
    )
}

fn part(outer_wire: Vec<OrientedPCurveEdge>) -> SplitSubFace {
    SplitSubFace {
        surface: FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
        outer_wire,
        inner_wires: Vec::new(),
        reversed: false,
        parent: remus_topology::test_utils::make_unit_square_face(&mut Topology::new()),
        rank: Rank::A,
        precomputed_interior: None,
    }
}

/// A lateral's wire as the established splitter leaves it when it drops a
/// section: the bottom rim (radius 1 about the z axis at z = 0), the seam up
/// the generator at angle 0, the top rim at z = 10 backwards, the seam down.
fn whole_lateral() -> SplitSubFace {
    let (bottom, top) = (circle(0.0, 0.0, 1.0), circle(0.0, 10.0, 1.0));
    let mut top_back = circle_edge(&top);
    top_back.forward = false;
    part(vec![
        circle_edge(&bottom),
        line((1.0, 0.0, 0.0), (1.0, 0.0, 10.0)),
        top_back,
        line((1.0, 0.0, 10.0), (1.0, 0.0, 0.0)),
    ])
}

/// A closed section whose midpoint (parameter π) lies `offset` beside the
/// seam of [`whole_lateral`] at z = 5, the rest of it well clear of every
/// edge of the wire.
fn section_touching_the_seam(offset: f64) -> SectionEdge {
    circle_section(&circle(2.0 + offset, 5.0, 1.0))
}

/// The review's first finding: a dropped closed section whose midpoint lies
/// on (or within the band of) the lateral's seam read as present, because
/// one point of it was near an edge. It is left out at every offset inside
/// the band, and on the seam itself.
#[test]
fn a_section_touching_the_seam_at_its_midpoint_is_left_out() {
    for offset in [0.0, 1e-6, 5e-5, -5e-5] {
        assert!(
            sections_left_out(
                [&section_touching_the_seam(offset)],
                &[whole_lateral()],
                TOL
            ),
            "offset {offset}"
        );
    }
}

/// Controls: a section the partition carries as its own edge is present,
/// whether it is a whole circle or the seam itself; a closed section on the
/// lateral the partition does carry stays present beside a seam it crosses.
#[test]
fn a_section_the_partition_carries_is_present() {
    let ring = circle(0.0, 5.0, 1.0);
    let mut lateral = whole_lateral();
    lateral.inner_wires.push(vec![circle_edge(&ring)]);
    // The ring crosses the seam at its parameter 0.
    assert!(!sections_left_out(
        [&circle_section(&ring)],
        &[lateral.clone()],
        TOL
    ));
    let seam = line_section((1.0, 0.0, 2.0), (1.0, 0.0, 7.0));
    assert!(!sections_left_out([&seam], &[lateral.clone()], TOL));
    // The same ring, with only the seam near it, is left out.
    assert!(sections_left_out(
        [&circle_section(&ring)],
        &[whole_lateral()],
        TOL
    ));
    // A section carried in two pieces (split at its own midpoint) is
    // followed piece by piece.
    let (a, b) = (ring.evaluate(0.0), ring.evaluate(PI));
    let halves = part(vec![
        edge(EdgeCurve::Circle(ring.clone()), Some((0.0, PI)), a, b),
        edge(EdgeCurve::Circle(ring.clone()), Some((PI, TAU)), b, a),
    ]);
    assert!(!sections_left_out([&circle_section(&ring)], &[halves], TOL));
}

/// The rectangle [0, 100] × [0, 10] left whole.
fn rectangle() -> SplitSubFace {
    part(vec![
        line((0.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
        line((100.0, 0.0, 0.0), (100.0, 10.0, 0.0)),
        line((100.0, 10.0, 0.0), (0.0, 10.0, 0.0)),
        line((0.0, 10.0, 0.0), (0.0, 0.0, 0.0)),
    ])
}

/// The review's second finding, a ready repro of a pre-existing heuristic:
/// a long straight edge keeps a slack of a tenth of its length, so the
/// rectangle left whole reads as carrying the dividing segment from (0, 1)
/// to (100, 1), 1 from its bottom edge (slack 10). Holding straight edges to
/// the band refuses the hammer opening replay
/// (`io/tests/hammer_opening_partition.rs`), whose trimmed fillet's
/// established partition omits a ruling section the re-partition cannot
/// certify; no wrong body has been shown to come of the slack.
#[test]
#[ignore = "pre-existing: a long straight edge's slack masks a dividing section (B265 review)"]
fn a_section_beside_a_long_straight_edge_is_left_out() {
    let dividing = line_section((0.0, 1.0, 0.0), (100.0, 1.0, 0.0));
    assert!(sections_left_out([&dividing], &[rectangle()], TOL));
}

/// The rectangle split along the segment carries it.
#[test]
fn a_section_the_split_rectangle_carries_is_present() {
    let dividing = line_section((0.0, 1.0, 0.0), (100.0, 1.0, 0.0));
    let below = part(vec![
        line((0.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
        line((100.0, 0.0, 0.0), (100.0, 1.0, 0.0)),
        line((100.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        line((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),
    ]);
    assert!(!sections_left_out([&dividing], &[below], TOL));
}

/// The scoped re-check's finding: different edges carried the samples
/// between them. The rectangle [0, 9] × [−1, 1] left whole, with eight
/// holes of radius 0.09995 about (k, −0.1): the dropped dividing segment
/// from (0, 0) to (9, 0) passes 5e-5 above each hole's top, so each of its
/// eight interior samples (at x = 1 … 8) lies within the band of a
/// different hole. No chain of edges follows it, and it is left out; the
/// same face split along the segment carries it.
#[test]
fn a_section_whose_samples_lie_near_different_edges_is_left_out() {
    let dividing = line_section((0.0, 0.0, 0.0), (9.0, 0.0, 0.0));
    let holes: Vec<Vec<OrientedPCurveEdge>> = (1..=8)
        .map(|k| vec![circle_edge(&hole(f64::from(k)))])
        .collect();
    let mut whole = part(vec![
        line((0.0, -1.0, 0.0), (9.0, -1.0, 0.0)),
        line((9.0, -1.0, 0.0), (9.0, 1.0, 0.0)),
        line((9.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        line((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
    ]);
    whole.inner_wires = holes.clone();
    assert!(sections_left_out([&dividing], &[whole], TOL));

    let mut below = part(vec![
        line((0.0, -1.0, 0.0), (9.0, -1.0, 0.0)),
        line((9.0, -1.0, 0.0), (9.0, 0.0, 0.0)),
        line((9.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        line((0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ]);
    below.inner_wires = holes;
    let above = part(vec![
        line((0.0, 0.0, 0.0), (9.0, 0.0, 0.0)),
        line((9.0, 0.0, 0.0), (9.0, 1.0, 0.0)),
        line((9.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        line((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),
    ]);
    assert!(!sections_left_out([&dividing], &[below, above], TOL));
}

/// The hole of radius 0.09995 about `(x, −0.1, 0)`.
fn hole(x: f64) -> Circle3D {
    Circle3D::new_with_ref(
        Point3::new(x, -0.1, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        0.099_95,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}
