//! The shadow differential's comparison rules, and its counters driven
//! through the production splitter on faces each legacy emitter owns.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::context::OperationContext;
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::det_hash::DetHashMap;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::super::super::split_types::{
    OrientedPCurveEdge, SectionEdge, SplitSubFace, SurfaceInfo,
};
use super::super::split_face_2d;
use super::{Mismatch, ShadowBudgets, Verdict, compare, take_reasons, with_mode};
use crate::ds::Rank;
use crate::gfa::{SectionDropScope, SplitterShadowCounts, SplitterShadowEntry};

const TOL: f64 = 1e-7;

fn dummy_pcurve() -> Curve2D {
    Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap())
}

fn z_plane() -> FaceSurface {
    FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 0.0,
    }
}

fn edge(
    curve: EdgeCurve,
    trim: Option<(f64, f64)>,
    start: Point3,
    end: Point3,
) -> OrientedPCurveEdge {
    OrientedPCurveEdge {
        curve_3d: curve,
        trim,
        pcurve: dummy_pcurve(),
        start_uv: Point2::new(start.x(), start.y()),
        end_uv: Point2::new(end.x(), end.y()),
        start_3d: start,
        end_3d: end,
        forward: true,
        source_edge_idx: None,
        pave_block_id: None,
        source_topo_edge: None,
    }
}

fn line(start: Point3, end: Point3) -> OrientedPCurveEdge {
    edge(EdgeCurve::Line, None, start, end)
}

fn unit_circle() -> Circle3D {
    Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
}

/// A counter-clockwise arc of `circle` from angle `t0` to `t1`.
fn arc(circle: &Circle3D, t0: f64, t1: f64) -> OrientedPCurveEdge {
    edge(
        EdgeCurve::Circle(circle.clone()),
        Some((t0, t1)),
        circle.evaluate(t0),
        circle.evaluate(t1),
    )
}

fn region(outer: Vec<OrientedPCurveEdge>) -> SplitSubFace {
    SplitSubFace {
        surface: z_plane(),
        outer_wire: outer,
        inner_wires: Vec::new(),
        reversed: false,
        // Any face handle: the comparison never reads the parent.
        parent: square_face(1.0).1,
        rank: Rank::A,
        precomputed_interior: None,
    }
}

fn polygon(points: &[(f64, f64)]) -> Vec<OrientedPCurveEdge> {
    let at = |i: usize| {
        let (x, y) = points[i % points.len()];
        Point3::new(x, y, 0.0)
    };
    (0..points.len()).map(|i| line(at(i), at(i + 1))).collect()
}

/// The upper half disc: the diameter, then the arc back.
fn half_disc(arc_pieces: &[(f64, f64)]) -> Vec<OrientedPCurveEdge> {
    let circle = unit_circle();
    let mut wire = vec![line(circle.evaluate(PI), circle.evaluate(0.0))];
    wire.extend(arc_pieces.iter().map(|&(t0, t1)| arc(&circle, t0, t1)));
    wire
}

#[test]
fn a_rotated_wire_and_reordered_regions_agree() {
    let left = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (0.0, 2.0)];
    let right = [(1.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.0, 2.0)];
    let legacy = [region(polygon(&left)), region(polygon(&right))];
    let mut rotated = polygon(&left);
    rotated.rotate_left(3);
    let shadow = [region(polygon(&right)), region(rotated)];
    assert_eq!(compare(&legacy, &shadow, TOL), Verdict::Agreed);
}

#[test]
fn each_property_is_named_by_the_first_mismatch() {
    let square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let legacy = [region(polygon(&square))];

    assert_eq!(
        compare(&legacy, &[], TOL),
        Verdict::Different(Mismatch::RegionCount)
    );

    // A vertex beyond the linear tolerance, and one inside it.
    let moved = |d: f64| [(0.0, 0.0), (1.0, 0.0), (1.0 + d, 1.0), (0.0, 1.0)];
    assert_eq!(
        compare(&legacy, &[region(polygon(&moved(1e-5)))], TOL),
        Verdict::Different(Mismatch::Vertex)
    );
    assert_eq!(
        compare(&legacy, &[region(polygon(&moved(5e-8)))], TOL),
        Verdict::Agreed
    );

    // The same traversal backwards is another wire, not a rotation.
    let mut backwards: Vec<_> = polygon(&square)
        .into_iter()
        .rev()
        .map(|e| line(e.end_3d, e.start_3d))
        .collect();
    backwards.rotate_left(1);
    assert_eq!(
        compare(&legacy, &[region(backwards)], TOL),
        Verdict::Different(Mismatch::Vertex)
    );

    let triangle = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)];
    assert_eq!(
        compare(&legacy, &[region(polygon(&triangle))], TOL),
        Verdict::Different(Mismatch::EdgeCount)
    );

    let mut holed = region(polygon(&square));
    holed.inner_wires.push(polygon(&triangle));
    assert_eq!(
        compare(&legacy, &[holed], TOL),
        Verdict::Different(Mismatch::HoleCount)
    );

    let mut flipped = region(polygon(&square));
    flipped.reversed = true;
    assert_eq!(
        compare(&legacy, &[flipped], TOL),
        Verdict::Different(Mismatch::Orientation)
    );
}

#[test]
fn a_chord_is_not_its_arc() {
    let legacy = [region(half_disc(&[(0.0, PI)]))];
    let circle = unit_circle();
    let chorded = vec![
        line(circle.evaluate(PI), circle.evaluate(0.0)),
        line(circle.evaluate(0.0), circle.evaluate(PI)),
    ];
    assert_eq!(
        compare(&legacy, &[region(chorded)], TOL),
        Verdict::Different(Mismatch::KindSequence)
    );
}

#[test]
fn an_arc_keeps_its_carrier_and_its_way_round() {
    let legacy = [region(half_disc(&[(0.0, PI)]))];
    let unit = unit_circle();
    let (east, north, west) = (
        unit.evaluate(0.0),
        unit.evaluate(0.5 * PI),
        unit.evaluate(PI),
    );

    // The same endpoints on a larger circle through them.
    let origin = Point3::new(0.0, 0.0, 0.0);
    let far = Circle3D::new(
        origin - (north - origin),
        Vec3::new(0.0, 0.0, 1.0),
        2f64.sqrt(),
    )
    .unwrap();
    let mut bulged = half_disc(&[]);
    bulged.push(edge(
        EdgeCurve::Circle(far.clone()),
        Some((far.project(east), far.project(west))),
        east,
        west,
    ));
    assert_eq!(
        compare(&legacy, &[region(bulged)], TOL),
        Verdict::Different(Mismatch::Carrier)
    );

    // The same circle and endpoints the other way round: the complement.
    let mut lower = half_disc(&[]);
    lower.push(edge(EdgeCurve::Circle(unit), Some((0.0, -PI)), east, west));
    assert_eq!(
        compare(&legacy, &[region(lower)], TOL),
        Verdict::Different(Mismatch::SubSpan)
    );

    // The same interval stored in traversal order on a reversed use: the
    // ends match, the direction `traversal_domain` reads from it does not.
    let mut top = edge(
        EdgeCurve::Circle(unit_circle()),
        Some((0.0, PI)),
        west,
        east,
    );
    top.forward = false;
    let mut swapped = top.clone();
    swapped.trim = Some((PI, 0.0));
    let cap = |arc: OrientedPCurveEdge| vec![line(east, west), arc];
    assert_eq!(
        compare(
            &[region(cap(top.clone()))],
            &[region(cap(top.clone()))],
            TOL
        ),
        Verdict::Agreed
    );
    assert_eq!(
        compare(&[region(cap(top))], &[region(cap(swapped))], TOL),
        Verdict::Different(Mismatch::SpanDirection)
    );

    // The same arc a whole turn later is the same sub-span.
    assert_eq!(
        compare(&legacy, &[region(half_disc(&[(TAU, TAU + PI)]))], TOL),
        Verdict::Agreed
    );
}

#[test]
fn a_carrier_broken_elsewhere_is_segmentation_only() {
    let legacy = [region(half_disc(&[(0.0, PI)]))];
    let split = [region(half_disc(&[(0.0, 0.5 * PI), (0.5 * PI, PI)]))];
    assert_eq!(
        compare(&legacy, &split, TOL),
        Verdict::SegmentationOnly(Mismatch::EdgeCount)
    );

    // A full circle cut at different angles, and into different pieces.
    let circle = unit_circle();
    let whole = [region(vec![arc(&circle, 0.0, TAU)])];
    let turned = [region(vec![arc(&circle, 1.0, 1.0 + TAU)])];
    assert_eq!(
        compare(&whole, &turned, TOL),
        Verdict::SegmentationOnly(Mismatch::Vertex)
    );
    let quartered = [region(
        (0..4)
            .map(|k| {
                arc(
                    &circle,
                    f64::from(k) * 0.5 * PI,
                    f64::from(k + 1) * 0.5 * PI,
                )
            })
            .collect(),
    )];
    assert_eq!(
        compare(&whole, &quartered, TOL),
        Verdict::SegmentationOnly(Mismatch::EdgeCount)
    );

    // Merging never hides a different partition.
    let other = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.5).unwrap();
    assert_eq!(
        compare(&whole, &[region(vec![arc(&other, 0.0, TAU)])], TOL),
        Verdict::Different(Mismatch::Carrier)
    );
}

// ---------------------------------------------------------------------------
// Counters, through the production splitter
// ---------------------------------------------------------------------------

fn line_section(start: Point3, end: Point3, pave_block: usize) -> SectionEdge {
    let dir = Vec2::new(end.x() - start.x(), end.y() - start.y());
    let len = dir.length().max(1e-12);
    let pcurve = Curve2D::Line(
        Line2D::new(
            Point2::new(start.x(), start.y()),
            Vec2::new(dir.x() / len, dir.y() / len),
        )
        .unwrap(),
    );
    SectionEdge {
        curve_3d: EdgeCurve::Line,
        trim: None,
        pcurve_a: pcurve.clone(),
        pcurve_b: pcurve,
        start,
        end,
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: Some(pave_block),
    }
}

fn square_face(size: f64) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let corners = [(0.0, 0.0), (size, 0.0), (size, size), (0.0, size)];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), TOL)))
        .collect();
    let edges: Vec<_> = (0..4)
        .map(|i| {
            topo.add_edge(Edge::new(
                vertices[i],
                vertices[(i + 1) % 4],
                EdgeCurve::Line,
            ))
        })
        .collect();
    let wire = topo.add_wire(
        Wire::new(
            edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, Vec::new(), z_plane()));
    (topo, face)
}

fn disc_face(radius: f64) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let circle =
        Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius).unwrap();
    let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
    let mut rim = Edge::new(v, v, EdgeCurve::Circle(circle));
    rim.set_trim(Some((0.0, TAU)));
    let rim = topo.add_edge(rim);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap());
    let face = topo.add_face(Face::new(wire, Vec::new(), z_plane()));
    (topo, face)
}

/// A full cylinder lateral of radius `r` between `z = 0` and `z = h`.
fn lateral_face(r: f64, h: f64) -> (Topology, FaceId) {
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let surface =
        remus_math::surfaces::CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), z, r).unwrap();
    let v_bot = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, 0.0), TOL));
    let v_top = topo.add_vertex(Vertex::new(Point3::new(r, 0.0, h), TOL));
    let bot = Circle3D::new(Point3::new(0.0, 0.0, 0.0), z, r).unwrap();
    let top = Circle3D::new(Point3::new(0.0, 0.0, h), z, r).unwrap();
    let b0 = bot.project(Point3::new(r, 0.0, 0.0));
    let t0 = top.project(Point3::new(r, 0.0, h));
    let mut be = Edge::new(v_bot, v_bot, EdgeCurve::Circle(bot));
    be.set_trim(Some((b0, b0 + TAU)));
    let mut te = Edge::new(v_top, v_top, EdgeCurve::Circle(top));
    te.set_trim(Some((t0, t0 + TAU)));
    let be = topo.add_edge(be);
    let te = topo.add_edge(te);
    let seam = topo.add_edge(Edge::new(v_bot, v_top, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(be, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(te, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cylinder(surface)));
    (topo, face)
}

fn latitude_section(r: f64, z: f64, pave_block: usize) -> SectionEdge {
    let circle = Circle3D::new(Point3::new(0.0, 0.0, z), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
    let seam = Point3::new(r, 0.0, z);
    let t0 = circle.project(seam);
    SectionEdge {
        curve_3d: EdgeCurve::Circle(circle),
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
        pave_block_id: Some(pave_block),
    }
}

fn ruling_section(r: f64, theta: f64, h: f64, pave_block: usize) -> SectionEdge {
    let at = |z: f64| Point3::new(r * theta.cos(), r * theta.sin(), z);
    SectionEdge {
        curve_3d: EdgeCurve::Line,
        trim: None,
        pcurve_a: dummy_pcurve(),
        pcurve_b: dummy_pcurve(),
        start: at(0.0),
        end: at(h),
        start_uv_a: None,
        end_uv_a: None,
        start_uv_b: None,
        end_uv_b: None,
        target_face: None,
        pave_block_id: Some(pave_block),
    }
}

type Counts = [SplitterShadowCounts; SplitterShadowEntry::ALL.len()];

/// Split through the production entry with the shadow differential set to
/// `mode`, returning the partition and the shadow counts it recorded.
fn split(
    mode: Option<ShadowBudgets>,
    topo: &Topology,
    face: FaceId,
    sections: &[SectionEdge],
) -> (Vec<SplitSubFace>, Counts, Vec<String>) {
    let scope = SectionDropScope::enter();
    take_reasons();
    let info = topo.face(face).ok().and_then(|f| {
        matches!(f.surface(), FaceSurface::Cylinder(_)).then_some(SurfaceInfo::Parametric {
            u_periodic: true,
            v_periodic: false,
        })
    });
    let parts = with_mode(mode, || {
        split_face_2d(
            topo,
            face,
            sections,
            Rank::A,
            &Tolerance::default(),
            None,
            info.as_ref(),
            &DetHashMap::default(),
            &OperationContext::new(),
            None,
        )
        .unwrap()
    });
    (
        parts,
        SplitterShadowEntry::ALL.map(|entry| scope.splitter_shadow(entry)),
        take_reasons(),
    )
}

/// What one budget mode recorded for the entry point under test.
#[derive(Debug, PartialEq, Eq)]
struct Seen {
    counts: SplitterShadowCounts,
    reasons: Vec<String>,
}

/// What `entry` recorded under default and raised budgets, after checking
/// that the differential is inert until enabled, that neither shadow run
/// changed the partition, and that no other entry point fired.
fn shadowed(
    entry: SplitterShadowEntry,
    topo: &Topology,
    face: FaceId,
    sections: &[SectionEdge],
) -> [Seen; 2] {
    let (plain, off, reasons) = split(None, topo, face, sections);
    assert_eq!(off, Counts::default(), "inert until enabled");
    assert!(reasons.is_empty(), "inert until enabled");
    [ShadowBudgets::Inherit, ShadowBudgets::Raised].map(|mode| {
        let (parts, counts, reasons) = split(Some(mode), topo, face, sections);
        assert_eq!(
            format!("{parts:?}"),
            format!("{plain:?}"),
            "a shadow run must not change the partition"
        );
        for other in SplitterShadowEntry::ALL {
            if other != entry {
                assert_eq!(counts[other as usize], SplitterShadowCounts::default());
            }
        }
        Seen {
            counts: counts[entry as usize],
            reasons,
        }
    })
}

fn seen(counts: SplitterShadowCounts, reason: &str) -> Seen {
    Seen {
        counts,
        reasons: vec![reason.to_owned()],
    }
}

const fn counts(
    agreed: u64,
    disagreed: u64,
    declined: u64,
    declined_on_budget: u64,
) -> SplitterShadowCounts {
    SplitterShadowCounts {
        fired: agreed + disagreed + declined + declined_on_budget,
        agreed,
        disagreed,
        declined,
        declined_on_budget,
        refused: 0,
    }
}

#[test]
fn crossing_sections_shadow_the_crossing_emitter() {
    let (topo, face) = square_face(2.0);
    let sections = [
        line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), 7),
        line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0), 8),
    ];
    let [default, raised] = shadowed(SplitterShadowEntry::CrossingPlane, &topo, face, &sections);
    assert_eq!(default, seen(counts(0, 0, 0, 1), "work-budget"));
    assert_eq!(raised, seen(counts(1, 0, 0, 0), "-"));
}

#[test]
fn rim_chords_shadow_the_disc_emitter() {
    // Two chords fanning from one rim point: three regions.
    let (topo, face) = disc_face(8.0);
    let y = 8.0 * (PI / 4.0).sin();
    let apex = Point3::new(8.0, 0.0, 0.0);
    let sections = [
        line_section(apex, Point3::new(-y, y, 0.0), 7),
        line_section(apex, Point3::new(-y, -y, 0.0), 8),
    ];
    let [default, raised] = shadowed(SplitterShadowEntry::DiskByChords, &topo, face, &sections);
    assert_eq!(default, seen(counts(0, 0, 0, 1), "work-budget"));
    // Same three regions on the same carrier; the two sides break the rim
    // at different angles.
    assert_eq!(
        raised,
        seen(counts(0, 1, 0, 0), "segmentation-only(vertex)")
    );
}

/// The cylinder adapter stores a reversed rim use's `trim` in traversal
/// order, where [`OrientedPCurveEdge::trim`] is native-direction and the
/// legacy emitters keep it so. Regions, wires, vertices and carriers all
/// match; only that interval's direction differs. Pinned as measured: an
/// adapter that adopts the native convention turns both into agreements.
#[test]
fn latitude_circles_shadow_the_band_emitter() {
    let (topo, face) = lateral_face(2.0, 3.0);
    let sections = [
        latitude_section(2.0, 0.9, 100),
        latitude_section(2.0, 2.1, 101),
    ];
    let [default, raised] = shadowed(SplitterShadowEntry::CylinderBands, &topo, face, &sections);
    assert_eq!(default, seen(counts(0, 0, 0, 1), "work-budget"));
    assert_eq!(raised, seen(counts(0, 1, 0, 0), "span-direction"));
}

#[test]
fn a_ruling_shadows_the_sector_emitter() {
    let (topo, face) = lateral_face(2.0, 3.0);
    let sections = [ruling_section(2.0, 2.0, 3.0, 200)];
    let [default, raised] = shadowed(SplitterShadowEntry::PeriodicSectors, &topo, face, &sections);
    assert_eq!(default, seen(counts(0, 0, 0, 1), "work-budget"));
    assert_eq!(raised, seen(counts(0, 1, 0, 0), "span-direction"));
}

// ---------------------------------------------------------------------------
// Work probe (budget-policy research): what each fixture needs
// ---------------------------------------------------------------------------

/// The work record of one fixture face, printed so a run can read it.
#[allow(clippy::print_stderr)]
fn work_of(topo: &Topology, face: FaceId, sections: &[SectionEdge]) -> super::work::Record {
    use super::work::{Adapter, Probe, measure};
    let frame = super::super::super::plane_frame::PlaneFrame::from_normal_and_point(
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
    );
    let tol = Tolerance::default();
    let context = OperationContext::new();
    let cylinder = topo
        .face(face)
        .is_ok_and(|f| matches!(f.surface(), FaceSurface::Cylinder(_)));
    let record = measure(&Probe {
        topo,
        face_id: face,
        sections,
        rank: Rank::A,
        tol: &tol,
        context: &context,
        adapter: if cylinder {
            Adapter::Cylinder
        } else {
            Adapter::Plane(&frame)
        },
    });
    eprintln!("{record:#?}");
    record
}

#[test]
fn work_of_the_minimal_fixtures() {
    let (topo, face) = square_face(2.0);
    // X: two full crossing sections, four regions.
    let x = work_of(
        &topo,
        face,
        &[
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), 7),
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0), 8),
        ],
    );
    // T: a full divider and a second section ending on it, three regions.
    let t = work_of(
        &topo,
        face,
        &[
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), 7),
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0), 8),
        ],
    );
    let (disc, disc_face_id) = disc_face(8.0);
    let y = 8.0 * (PI / 4.0).sin();
    let apex = Point3::new(8.0, 0.0, 0.0);
    let chords = work_of(
        &disc,
        disc_face_id,
        &[
            line_section(apex, Point3::new(-y, y, 0.0), 7),
            line_section(apex, Point3::new(-y, -y, 0.0), 8),
        ],
    );
    let (lateral, lateral_face_id) = lateral_face(2.0, 3.0);
    let bands = work_of(
        &lateral,
        lateral_face_id,
        &[
            latitude_section(2.0, 0.9, 100),
            latitude_section(2.0, 2.1, 101),
        ],
    );
    let sectors = work_of(
        &lateral,
        lateral_face_id,
        &[ruling_section(2.0, 2.0, 3.0, 200)],
    );
    for record in [&x, &t, &chords, &bands, &sectors] {
        assert!(matches!(record.raised, super::work::Outcome::Split(_)));
        assert_eq!(record.default, super::work::Outcome::Budget);
    }
}

/// Where each wire edge's `traversal_domain` starts and ends on its carrier,
/// against the edge's own traversal endpoints, and whether the carrier
/// tangent at the traversal start points the way the wire goes.
///
/// `OrientedPCurveEdge::trim` is native-direction: `traversal_domain` swaps
/// it for a reversed use. A trim stored in traversal order on a reversed use
/// is swapped once too often, so its traversal interval runs backwards: an
/// open arc starts at its end point, and a closed rim (whose endpoints
/// coincide) is walked the wrong way round.
fn trim_misreads(parts: &[SplitSubFace]) -> Vec<String> {
    let mut out = Vec::new();
    for (r, part) in parts.iter().enumerate() {
        for (i, e) in part.outer_wire.iter().enumerate() {
            let EdgeCurve::Circle(circle) = &e.curve_3d else {
                continue;
            };
            let (t0, t1) = e.traversal_domain();
            let start_off = (circle.evaluate(t0) - e.start_3d).length();
            let end_off = (circle.evaluate(t1) - e.end_3d).length();
            // The wire's next edge starts where this one ends; its traversal
            // direction near the start is (end of a short step) - start.
            let step = (t1 - t0) * 1e-3;
            let ahead = circle.evaluate(t0 + step) - circle.evaluate(t0);
            // Reference direction: the arc swept with `forward`'s sense from
            // start to end (the convention every emitter's polyline uses).
            let sense = if e.forward { 1.0 } else { -1.0 };
            let tangent = circle.tangent(circle.project(e.start_3d)) * sense;
            let backwards = ahead.dot(tangent) < 0.0;
            if start_off > 1e-7 || end_off > 1e-7 || backwards {
                out.push(format!(
                    "region {r} edge {i}: forward={} trim={:?} traversal=({t0:.4},{t1:.4}) \
                     start_off={start_off:.3e} end_off={end_off:.3e} backwards={backwards}",
                    e.forward, e.trim
                ));
            }
        }
    }
    out
}

#[test]
#[allow(clippy::print_stderr)]
fn reversed_rim_trims_misread_through_traversal_domain() {
    use super::super::arrangement_cyl::try_split_cylinder_face_by_provenance_arrangement;
    let (topo, face) = lateral_face(2.0, 3.0);
    let raised = OperationContext::new().with_budgets(super::RAISED_BUDGETS);
    let tol = Tolerance::default();
    for (label, sections) in [
        (
            "bands",
            vec![
                latitude_section(2.0, 0.9, 100),
                latitude_section(2.0, 2.1, 101),
            ],
        ),
        ("sectors", vec![ruling_section(2.0, 2.0, 3.0, 200)]),
    ] {
        let (legacy, _, _) = split(None, &topo, face, &sections);
        let adapter = try_split_cylinder_face_by_provenance_arrangement(
            &topo,
            face,
            &sections,
            Rank::A,
            &tol,
            &raised,
            None,
        )
        .unwrap()
        .unwrap();
        let legacy_misreads = trim_misreads(&legacy);
        let adapter_misreads = trim_misreads(&adapter);
        eprintln!("{label}: legacy {legacy_misreads:#?}\n{label}: adapter {adapter_misreads:#?}");
        assert!(legacy_misreads.is_empty(), "{label}: legacy misread");
        assert!(
            !adapter_misreads.is_empty(),
            "{label}: adapter read correctly"
        );
    }
}

/// Work and time of a `k` x `k` grid of full-width sections on a square:
/// how the planar adapter's march steps grow with the input, and what one
/// step costs. `k = 10` passes `MAX_ARRANGEMENT_USES` and declines.
#[test]
#[ignore = "O2.3d budget-policy research probe (about 10 s); run with --ignored --nocapture"]
#[allow(clippy::print_stderr, clippy::cast_precision_loss)]
fn work_of_a_section_grid() {
    for k in [1usize, 2, 3, 4, 6, 8, 9, 10] {
        let size = (k + 1) as f64;
        let (topo, face) = square_face(size);
        let mut sections = Vec::new();
        for i in 1..=k {
            let at = i as f64;
            sections.push(line_section(
                Point3::new(at, 0.0, 0.0),
                Point3::new(at, size, 0.0),
                2 * i,
            ));
            sections.push(line_section(
                Point3::new(0.0, at, 0.0),
                Point3::new(size, at, 0.0),
                2 * i + 1,
            ));
        }
        let record = work_of(&topo, face, &sections);
        eprintln!(
            "GRID k={k} sections={} uses={} march={:?} queue={:?} segments={:?} regions={} ns={:?} raised={:?}",
            sections.len(),
            record.uses,
            record.march,
            record.queue,
            record.segments,
            record.regions,
            record.adapter_ns,
            record.raised
        );
    }
}

/// Exercise the actual logging sink through production dispatch, with shadow
/// work disabled in both runs. Pin the existing counters as well as comparing
/// every PerfSnapshot field, ray work, winding work and the complete output.
#[cfg(feature = "perf-counters")]
#[test]
#[allow(clippy::too_many_lines)]
fn work_logging_preserves_output_and_perf_counters() {
    use std::io::{Read as _, Seek as _};

    let log_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/diag");
    std::fs::create_dir_all(&log_dir).unwrap();
    // Retain the log so the census in the design note can be regenerated.
    let log_path = log_dir.join("splitter-work-fixtures.tsv");
    let mut log = std::fs::OpenOptions::new()
        .create(true)
        .truncate(true)
        .read(true)
        .write(true)
        .open(&log_path)
        .unwrap();
    let snapshot = || {
        (
            crate::perf::snapshot(),
            crate::perf::take_ray_work(),
            crate::perf::take_winding_cut_projections(),
        )
    };
    let mut inputs = Vec::new();
    // Put the 935-step small case first, reproducing the old census failure.
    for (label, size, x, y) in [
        ("X-small", 2e-3, 0.7e-3, 0.8e-3),
        ("X", 2.0, 1.0, 1.0),
        ("X-offset", 2.0, 0.7, 0.8),
        ("X-large", 2e3, 700.0, 800.0),
        ("T", 2.0, 1.0, 1.0),
    ] {
        let (topo, face) = square_face(size);
        let sections = vec![
            line_section(Point3::new(x, 0.0, 0.0), Point3::new(x, size, 0.0), 7),
            line_section(
                Point3::new(0.0, y, 0.0),
                Point3::new(if label == "T" { x } else { size }, y, 0.0),
                8,
            ),
        ];
        inputs.push((label, topo, face, sections));
    }
    let (topo, face) = lateral_face(2.0, 3.0);
    inputs.push((
        "bands",
        topo,
        face,
        vec![
            latitude_section(2.0, 0.9, 100),
            latitude_section(2.0, 2.1, 101),
        ],
    ));
    let (topo, face) = lateral_face(2.0, 3.0);
    inputs.push((
        "sectors",
        topo,
        face,
        vec![ruling_section(2.0, 2.0, 3.0, 200)],
    ));
    let (topo, face) = disc_face(8.0);
    let y = 8.0 * (PI / 4.0).sin();
    let apex = Point3::new(8.0, 0.0, 0.0);
    inputs.push((
        "disc",
        topo,
        face,
        vec![
            line_section(apex, Point3::new(-y, y, 0.0), 7),
            line_section(apex, Point3::new(-y, -y, 0.0), 8),
        ],
    ));
    for (label, topo, face, sections) in &inputs {
        crate::perf::reset();
        let _ = crate::perf::take_ray_work();
        let (off, off_shadow, off_reasons) =
            super::work::with_log(None, || split(None, topo, *face, sections));
        let off_work = snapshot();
        match *label {
            "T" => assert_eq!(off_work.0.face_split_probes, 79),
            "X-small" | "X" | "X-offset" | "X-large" => {
                assert_eq!(off_work.0.face_split_probes, 64);
            }
            _ => {}
        }
        crate::perf::reset();
        let (on, on_shadow, on_reasons) =
            super::work::with_log(Some(log.try_clone().unwrap()), || {
                split(None, topo, *face, sections)
            });
        assert_eq!(snapshot(), off_work, "{label}: diagnostic work leaked");
        assert_eq!(
            format!("{on:?}"),
            format!("{off:?}"),
            "{label}: output changed"
        );
        assert_eq!(on_shadow, off_shadow);
        assert_eq!(on_reasons, off_reasons);
    }
    // Repeating exactly the same input must repeat its content identity.
    let (_, topo, face, sections) = &inputs[1];
    super::work::with_log(Some(log.try_clone().unwrap()), || {
        split(None, topo, *face, sections)
    });
    log.rewind().unwrap();
    let mut text = String::new();
    log.read_to_string(&mut text).unwrap();
    let rows: Vec<Vec<_>> = text
        .lines()
        .map(|line| line.split('\t').collect())
        .collect();
    let crossing: Vec<_> = rows
        .iter()
        .filter(|row| row[0] == "try_split_crossing_plane_face")
        .collect();
    assert_eq!(crossing.len(), 5);
    assert_eq!(crossing[0][12], "935");
    assert!(crossing[1..].iter().all(|row| row[12] == "985"));
    let fingerprints: std::collections::BTreeSet<_> = crossing.iter().map(|row| row[26]).collect();
    assert_eq!(
        fingerprints.len(),
        4,
        "equal local IDs must retain distinct inputs"
    );
    assert_eq!(
        crossing[1][26], crossing[4][26],
        "repeated inputs must fold"
    );
}
