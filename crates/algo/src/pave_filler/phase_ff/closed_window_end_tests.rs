//! Closed-section window ends land on the boundary the section crosses.
//!
//! A plane slicing a cone or cylinder lateral exactly at its rim meets it in
//! a marched ring that rides the lateral's own band end, so the lateral's
//! strict extent test flips with the fit's wobble all along it. The windows
//! are cut where the ring leaves the PLANE face; the expected ends come from
//! intersecting the ring's own chords with the face edge, not from the
//! extent predicates under test.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_topology::face::Face;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{
    Aabb3, Edge, EdgeCurve, FaceBand, FaceExtents, FaceId, FaceSurface, JunctionRegistry, Point3,
    RawCurve, Tolerance, Topology, Vec3, Vertex, restrict_curves_to_faces,
};

/// Box top: the rectangle `[0, 2] × [0, 3]` in the plane `z = 0`.
fn box_top(topo: &mut Topology) -> (FaceId, FaceSurface) {
    let corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (0.0, 3.0)]
        .map(|(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7)));
    let edges = (0..4)
        .map(|i| {
            let e = topo.add_edge(Edge::new(corners[i], corners[(i + 1) % 4], EdgeCurve::Line));
            OrientedEdge::new(e, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let surface = FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 0.0,
    };
    (
        topo.add_face(Face::new(wire, vec![], surface.clone())),
        surface,
    )
}

/// A radius-0.5 lateral about `(0.25, 1.5)` whose band `v ∈ [-4, 0]` ends
/// on the box top: its rim crosses the box edge `x = 0` twice.
fn lateral(topo: &mut Topology) -> (FaceId, FaceSurface) {
    let centre = Point3::new(0.25, 1.5, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let surface = FaceSurface::Cylinder(CylindricalSurface::new(centre, z, 0.5).unwrap());
    let circle = Circle3D::new(centre, z, 0.5).unwrap();
    let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::with_tolerance(v, v, EdgeCurve::Circle(circle), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let e = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
    (
        topo.add_face(Face::new(wire, vec![], surface.clone())),
        surface,
    )
}

/// The rim as a closed degree-1 ring of `SEGMENTS` chords from angle `seam`,
/// its height `-wobble` (inside the band) at every third vertex and
/// `+wobble` (past its end) elsewhere: the clip's samples fall on both
/// sides, as a marched ring's do.
const SEGMENTS: usize = 96;
fn wobbly_rim(seam: f64, wobble: f64) -> (RawCurve, Vec<Point3>) {
    let points: Vec<Point3> = (0..=SEGMENTS)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let a = seam + TAU * k as f64 / SEGMENTS as f64;
            let h = if k % 3 == 0 { -wobble } else { wobble };
            Point3::new(0.25 + 0.5 * a.cos(), 1.5 + 0.5 * a.sin(), h)
        })
        .collect();
    let mut knots = vec![0.0; 2];
    #[allow(clippy::cast_precision_loss)]
    knots.extend((1..SEGMENTS).map(|i| i as f64 / SEGMENTS as f64));
    knots.extend([1.0; 2]);
    let curve = NurbsCurve::new(1, knots, points.clone(), vec![1.0; SEGMENTS + 1]).unwrap();
    let raw = RawCurve {
        bbox: Aabb3::from_points(points.iter().copied()),
        t_range: (0.0, 1.0),
        p_start: points[0],
        p_end: points[SEGMENTS],
        curve: EdgeCurve::NurbsCurve(curve),
    };
    (raw, points)
}

/// The ring's two chord crossings of the box edge `x = 0`.
fn edge_crossings(points: &[Point3]) -> Vec<Point3> {
    points
        .windows(2)
        .filter(|w| w[0].x().signum() != w[1].x().signum())
        .map(|w| {
            let s = w[0].x() / (w[0].x() - w[1].x());
            w[0] + (w[1] - w[0]) * s
        })
        .collect()
}

#[test]
fn closed_ring_riding_the_partner_band_end_is_cut_at_the_plane_edge() {
    // Seam 0.81 rad puts the 24-sample clip's sample 5 at x ≈ -0.01: inside
    // the box top's 0.02 margin but off the face, the run's last sample.
    // Seam 0.3 keeps every run end strictly inside the box top.
    for seam in [0.808_6, 0.3] {
        for wobble in [0.0, 1e-5] {
            let mut topo = Topology::new();
            let (top, top_surface) = box_top(&mut topo);
            let (wall, wall_surface) = lateral(&mut topo);
            let (raw, points) = wobbly_rim(seam, wobble);
            let crossings = edge_crossings(&points);
            assert_eq!(crossings.len(), 2);
            let pieces = restrict_curves_to_faces(
                &topo,
                top,
                wall,
                &top_surface,
                &wall_surface,
                &FaceBand::NONE,
                &FaceBand::bounded(Some((-4.0, 0.0))),
                vec![raw.clone()],
                Tolerance::default(),
                &mut JunctionRegistry::default(),
                &mut FaceExtents::default(),
            )
            .unwrap();
            // The in-face arc wraps the seam: one piece each side of it.
            assert_eq!(pieces.len(), 2, "seam {seam} wobble {wobble}");
            let ends = [
                (pieces[0].t_range.0, pieces[0].p_start),
                (pieces[1].t_range.1, pieces[1].p_end),
            ];
            for (t, p) in ends {
                let on_ring = raw.curve.evaluate_with_endpoints(t, raw.p_start, raw.p_end);
                assert!(
                    on_ring.x().abs() < 1e-9,
                    "seam {seam} wobble {wobble}: end t {t} at {on_ring:?}, not on the box edge"
                );
                // The endpoint is the junction weld: on the box edge, at most
                // the ring's chord sagitta (2.7e-4 radially, 3.1e-4 along an
                // edge it crosses at 60°) from its parameter end — the exact
                // edge × lateral crossing, or that end itself.
                let exact = Point3::new(
                    0.0,
                    1.5 + (on_ring.y() - 1.5).signum() * 0.1875_f64.sqrt(),
                    0.0,
                );
                assert!(
                    p.x().abs() < 1e-9
                        && (p - on_ring).length() < 3.2e-4
                        && ((p - on_ring).length() < 1e-9 || (p - exact).length() < 1e-9),
                    "seam {seam} wobble {wobble}: end {p:?} is not welded on the box edge \
                     at {on_ring:?} or {exact:?}"
                );
                assert!(
                    crossings.iter().any(|c| (c.y() - on_ring.y()).abs() < 1e-9),
                    "seam {seam} wobble {wobble}: end {on_ring:?} is no crossing of {crossings:?}"
                );
            }
            // The pieces meet at the seam and keep the in-face arc only.
            assert!((pieces[0].p_end - pieces[1].p_start).length() < 1e-12);
            for piece in &pieces {
                for k in 1..16 {
                    let t =
                        piece.t_range.0 + (piece.t_range.1 - piece.t_range.0) * f64::from(k) / 16.0;
                    let p = raw.curve.evaluate_with_endpoints(t, raw.p_start, raw.p_end);
                    assert!(
                        p.x() > 0.0,
                        "seam {seam} wobble {wobble}: {p:?} off the face"
                    );
                }
            }
        }
    }
}
