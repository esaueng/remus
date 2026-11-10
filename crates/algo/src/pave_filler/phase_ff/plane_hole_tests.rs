//! Closed-form oracles for a plane × cylinder section against the planar
//! face's HOLES.
//!
//! The expected answers share no code with the helpers under test: a circle
//! meets the line `x = c` at `y = cy ± √(r² − (c − cx)²)`, two circles meet
//! where the standard chord construction puts them, a span lies on the face
//! when a point of it does (tested here by coordinates), and a touch is a
//! distance equal to the radius.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;
use remus_topology::face::Face;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::{
    Aabb3, Edge, EdgeCurve, FaceExtent, FaceId, FaceSurface, LineContact, Point3, RawCurve,
    SectionConic, Tolerance, Topology, Vec3, Vertex, closed_circle_boundary_crossings,
    plane_hole_crossings, trim_ellipse_to_boundary_crossings,
};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
const SCALES: [f64; 3] = [0.1, 1.0, 10.0];

// ── Fixtures ────────────────────────────────────────────────────────────

fn vertex(topo: &mut Topology, p: Point3) -> remus_topology::vertex::VertexId {
    topo.add_vertex(Vertex::new(p, 1e-7))
}

/// A closed loop of straight edges through `corners`, in that order.
fn polygon_wire(topo: &mut Topology, corners: &[(f64, f64)], z: f64) -> WireId {
    let verts: Vec<_> = corners
        .iter()
        .map(|&(x, y)| vertex(topo, Point3::new(x, y, z)))
        .collect();
    let edges = (0..verts.len())
        .map(|i| {
            let edge = Edge::new(verts[i], verts[(i + 1) % verts.len()], EdgeCurve::Line);
            OrientedEdge::new(topo.add_edge(edge), true)
        })
        .collect();
    topo.add_wire(Wire::new(edges, true).unwrap())
}

fn square(half: f64, (cx, cy): (f64, f64)) -> [(f64, f64); 4] {
    [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]
}

/// A full circle as one closed edge, traversed with or against its
/// parameter.
fn circle_wire(topo: &mut Topology, circle: &Circle3D, forward: bool) -> WireId {
    let seam = vertex(topo, circle.evaluate(0.0));
    let mut edge = Edge::with_tolerance(seam, seam, EdgeCurve::Circle(circle.clone()), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
}

fn plane_at(z: f64) -> FaceSurface {
    FaceSurface::Plane { normal: Z, d: z }
}

fn circle_at((cx, cy): (f64, f64), z: f64, r: f64) -> Circle3D {
    Circle3D::new(Point3::new(cx, cy, z), Z, r).unwrap()
}

/// A full cylinder wall of radius `r` about the vertical through `at`,
/// from `z0` to `z1`: two rim circles joined by one seam.
fn cylinder_wall(
    topo: &mut Topology,
    at: (f64, f64),
    r: f64,
    (z0, z1): (f64, f64),
) -> (FaceId, FaceSurface) {
    let (bottom, top) = (circle_at(at, z0, r), circle_at(at, z1, r));
    let (vb, vt) = (
        vertex(topo, bottom.evaluate(0.0)),
        vertex(topo, top.evaluate(0.0)),
    );
    let mut rim = |circle: &Circle3D, v| {
        let mut edge = Edge::with_tolerance(v, v, EdgeCurve::Circle(circle.clone()), Some(1e-7));
        edge.set_trim(Some((0.0, TAU)));
        topo.add_edge(edge)
    };
    let (eb, et) = (rim(&bottom, vb), rim(&top, vt));
    let seam = topo.add_edge(Edge::new(vb, vt, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(eb, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(et, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .unwrap(),
    );
    let surface =
        FaceSurface::Cylinder(CylindricalSurface::new(Point3::new(at.0, at.1, z0), Z, r).unwrap());
    (
        topo.add_face(Face::new(wire, vec![], surface.clone())),
        surface,
    )
}

/// The closed plane × cylinder section at height `z`.
fn section(at: (f64, f64), z: f64, r: f64) -> RawCurve {
    let circle = circle_at(at, z, r);
    let start = circle.evaluate(0.0);
    RawCurve {
        bbox: Aabb3::from_points([
            Point3::new(at.0 - r, at.1 - r, z),
            Point3::new(at.0 + r, at.1 + r, z),
        ]),
        curve: EdgeCurve::Circle(circle),
        t_range: (0.0, TAU),
        p_start: start,
        p_end: start,
    }
}

/// A planar face at height `z` and a cylinder wall through it, with the
/// section the pair meets in and the arcs the trim keeps of it.
struct Pair {
    topo: Topology,
    plane: FaceId,
    wall: FaceId,
    surfaces: (FaceSurface, FaceSurface),
    raw: RawCurve,
}

impl Pair {
    fn new(
        outline: &[(f64, f64)],
        holes: impl FnOnce(&mut Topology) -> Vec<WireId>,
        at: (f64, f64),
        r: f64,
        s: f64,
    ) -> Self {
        let mut topo = Topology::new();
        let z = 50.0 * s;
        let outline: Vec<_> = outline.iter().map(|&(x, y)| (x * s, y * s)).collect();
        let outer = polygon_wire(&mut topo, &outline, z);
        let inner = holes(&mut topo);
        let plane = topo.add_face(Face::new(outer, inner, plane_at(z)));
        let at = (at.0 * s, at.1 * s);
        let (wall, wall_surface) =
            cylinder_wall(&mut topo, at, r * s, (z - 30.0 * s, z + 30.0 * s));
        Self {
            topo,
            plane,
            wall,
            surfaces: (plane_at(z), wall_surface),
            raw: section(at, z, r * s),
        }
    }

    fn trim(&self) -> Result<Option<Vec<RawCurve>>, super::AlgoError> {
        let tol = Tolerance::new();
        let (plane_surface, wall_surface) = &self.surfaces;
        let plane_extent = FaceExtent::new(&self.topo, self.plane, plane_surface, None, tol)
            .unwrap()
            .unwrap();
        let s = (self.raw.p_start.z()) / 50.0;
        let wall_extent = FaceExtent::new(
            &self.topo,
            self.wall,
            wall_surface,
            Some((0.0, 60.0 * s)),
            tol,
        )
        .unwrap()
        .unwrap();
        trim_ellipse_to_boundary_crossings(
            &self.topo,
            self.plane,
            self.wall,
            plane_surface,
            wall_surface,
            &self.raw,
            &plane_extent,
            &wall_extent,
            tol,
        )
    }
}

/// The rim of a shelled 100 × 100 box with a 2 mm wall, scaled by `s`, and a
/// cylinder of radius `r` at `at`.
fn rim_pair(at: (f64, f64), r: f64, s: f64) -> Pair {
    Pair::new(
        &square(50.0, (50.0, 50.0)),
        |topo| {
            let mut hole = square(48.0 * s, (50.0 * s, 50.0 * s)).to_vec();
            hole.reverse();
            vec![polygon_wire(topo, &hole, 50.0 * s)]
        },
        at,
        r,
        s,
    )
}

/// Whether `p` (unit scale) lies strictly on the rim: inside the outline and
/// outside the cavity opening.
fn on_rim(p: Point3, s: f64) -> bool {
    let (x, y) = (p.x() / s, p.y() / s);
    let inside = |lo: f64, hi: f64| x > lo && x < hi && y > lo && y < hi;
    inside(0.0, 100.0) && !inside(2.0, 98.0)
}

fn arc_midpoint(arc: &RawCurve) -> Point3 {
    let EdgeCurve::Circle(circle) = &arc.curve else {
        panic!("arc of a circle")
    };
    circle.evaluate(0.5 * (arc.t_range.0 + arc.t_range.1))
}

// ── SectionConic::line_contact ──────────────────────────────────────────

#[test]
fn a_circle_reaching_past_a_line_crosses_and_one_at_its_radius_touches() {
    let tol = Tolerance::new();
    for s in [1e-2, 1.0, 1e3] {
        let conic = SectionConic::of_circle(&circle_at((3.0 * s, -2.0 * s), 0.0, s));
        // The line `y = offset` over x in [1, 5]: its distance from the
        // centre is `offset + 2`, so it is tangent at offset −1.
        let contact = |offset: f64| {
            conic.line_contact(
                Point3::new(s, offset, 0.0),
                Point3::new(5.0 * s, offset, 0.0),
                tol,
            )
        };
        assert_eq!(contact(-1.5 * s), LineContact::Cross, "chord, scale {s}");
        assert!(
            matches!(contact(-s), LineContact::Touch(_)),
            "tangent, scale {s}"
        );
        // The band is the linear tolerance either side of tangency.
        assert!(matches!(
            contact(-s - 0.5 * tol.linear),
            LineContact::Touch(_)
        ));
        assert!(matches!(
            contact(-s + 0.5 * tol.linear),
            LineContact::Touch(_)
        ));
        assert_eq!(contact(-s - 3.0 * tol.linear), LineContact::Cross);
        assert_eq!(contact(-s + 3.0 * tol.linear), LineContact::Miss);
        assert_eq!(contact(0.5 * s), LineContact::Miss, "clear, scale {s}");
        // The same tangent line, but an edge of it that stops short of the
        // touch point at x = 3.
        let short = conic.line_contact(
            Point3::new(4.0 * s, -s, 0.0),
            Point3::new(5.0 * s, -s, 0.0),
            tol,
        );
        assert_eq!(short, LineContact::Miss, "touch off the edge, scale {s}");
        // An edge that ENDS at the touch point: a straight side running
        // tangentially into a corner arc the section rides. Not a touch.
        let ending = conic.line_contact(
            Point3::new(3.0 * s, -s, 0.0),
            Point3::new(5.0 * s, -s, 0.0),
            tol,
        );
        assert_eq!(
            ending,
            LineContact::Miss,
            "touch at the edge end, scale {s}"
        );
    }
}

#[test]
fn an_ellipse_touches_the_tangent_at_any_of_its_points() {
    let tol = Tolerance::new();
    let (a, b) = (5.0, 2.0);
    let ellipse = Ellipse3D::new(Point3::new(1.0, 1.0, 0.0), Z, a, b).unwrap();
    let conic = SectionConic::of_curve(&EdgeCurve::Ellipse(ellipse.clone())).unwrap();
    for k in 0..16 {
        let t = TAU * f64::from(k) / 16.0 + 0.1;
        let (p, tangent) = (ellipse.evaluate(t), ellipse.tangent(t).normalize().unwrap());
        // Towards the centre, across the tangent.
        let inward = (ellipse.center() - p).normalize().unwrap();
        let line = |shift: f64| {
            let q = p + inward * shift;
            conic.line_contact(q - tangent * 3.0, q + tangent * 3.0, tol)
        };
        assert!(
            matches!(line(0.0), LineContact::Touch(_)),
            "tangent at t = {t}"
        );
        assert_eq!(line(0.2), LineContact::Cross, "chord at t = {t}");
        assert_eq!(line(-0.2), LineContact::Miss, "clear at t = {t}");
    }
    assert!(SectionConic::of_curve(&EdgeCurve::Line).is_none());
}

// ── plane_hole_crossings ────────────────────────────────────────────────

#[test]
fn a_square_hole_is_crossed_where_the_circle_meets_its_edges() {
    let tol = Tolerance::new();
    for s in SCALES {
        // Centre 1 outside the cavity's x = 98 side, radius 3: it meets
        // that side at y = 50 ± √(9 − 1) and no other.
        let pair = rim_pair((99.0, 50.0), 3.0, s);
        let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
        let frame =
            crate::builder::plane_frame::PlaneFrame::from_normal_and_point(Z, pair.raw.p_start);
        let found = plane_hole_crossings(
            &pair.topo,
            hole,
            &pair.raw.curve,
            &pair.surfaces.1,
            &frame,
            tol,
        )
        .unwrap();
        assert!(found.touches.is_empty());
        assert_eq!(found.outline.len(), 4);
        let mut ys: Vec<f64> = found.crossings.iter().map(|p| p.y() / s).collect();
        ys.sort_by(f64::total_cmp);
        assert_eq!(ys.len(), 2, "scale {s}");
        let half = 8.0_f64.sqrt();
        for (y, wanted) in ys.iter().zip([50.0 - half, 50.0 + half]) {
            assert!((y - wanted).abs() < 1e-12, "y {y} vs {wanted}, scale {s}");
        }
        for p in &found.crossings {
            assert!((p.x() / s - 98.0).abs() < 1e-12);
        }

        // Radius 1 only touches that side: no crossing, and the touch is
        // reported.
        let pair = rim_pair((99.0, 50.0), 1.0, s);
        let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
        let found = plane_hole_crossings(
            &pair.topo,
            hole,
            &pair.raw.curve,
            &pair.surfaces.1,
            &frame,
            tol,
        )
        .unwrap();
        assert!(found.crossings.is_empty(), "scale {s}");
        assert_eq!(found.touches.len(), 1, "scale {s}");
    }
}

#[test]
fn a_round_hole_is_crossed_at_the_two_circle_circle_points() {
    let tol = Tolerance::new();
    for s in SCALES {
        for forward in [false, true] {
            // Bore of radius 8 at (20, 20); cutter of radius 3 at (28, 20),
            // on the bore's rim. The chord sits at x = 20 + (64 + 64 − 9)/16.
            let bore = circle_at((20.0 * s, 20.0 * s), 50.0 * s, 8.0 * s);
            let build = |at: (f64, f64), r: f64| {
                Pair::new(
                    &square(20.0, (20.0, 20.0)),
                    |topo| vec![circle_wire(topo, &bore, forward)],
                    at,
                    r,
                    s,
                )
            };
            let pair = build((28.0, 20.0), 3.0);
            let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
            let frame =
                crate::builder::plane_frame::PlaneFrame::from_normal_and_point(Z, pair.raw.p_start);
            let found = plane_hole_crossings(
                &pair.topo,
                hole,
                &pair.raw.curve,
                &pair.surfaces.1,
                &frame,
                tol,
            )
            .unwrap();
            assert!(found.touches.is_empty());
            assert_eq!(found.crossings.len(), 2, "scale {s}");
            let along = (64.0 + 64.0 - 9.0) / 16.0;
            let across = (64.0_f64 - along * along).sqrt();
            for p in &found.crossings {
                assert!(
                    (p.x() / s - (20.0 + along)).abs() < 1e-12,
                    "x {}",
                    p.x() / s
                );
                assert!(((p.y() / s - 20.0).abs() - across).abs() < 1e-12);
            }

            // Radius 3 at distance 11: externally tangent. Radius 3 at
            // distance 5: internally tangent. Neither crosses.
            for at in [(31.0, 20.0), (25.0, 20.0)] {
                let pair = build(at, 3.0);
                let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
                let found = plane_hole_crossings(
                    &pair.topo,
                    hole,
                    &pair.raw.curve,
                    &pair.surfaces.1,
                    &frame,
                    tol,
                )
                .unwrap();
                assert!(found.crossings.is_empty(), "tangent at {at:?}, scale {s}");
                assert_eq!(found.touches.len(), 1, "tangent at {at:?}, scale {s}");
            }

            // An elliptical section against a circular edge is not solved
            // here: the hole is left out whole.
            let pair = build((28.0, 20.0), 3.0);
            let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
            let ellipse = Ellipse3D::new(
                Point3::new(28.0 * s, 20.0 * s, 50.0 * s),
                Z,
                4.0 * s,
                3.0 * s,
            )
            .unwrap();
            assert!(
                plane_hole_crossings(
                    &pair.topo,
                    hole,
                    &EdgeCurve::Ellipse(ellipse),
                    &pair.surfaces.1,
                    &frame,
                    tol,
                )
                .is_none()
            );
        }
    }
}

// ── trim_ellipse_to_boundary_crossings ──────────────────────────────────

/// A cutter standing on the wall leaves two arcs on the rim, from the
/// outline to the cavity opening, and nothing over the opening or outside.
#[test]
fn a_section_across_the_rim_keeps_only_its_spans_on_the_rim() {
    for s in SCALES {
        // r = 1.5 leaves the rim by 0.5 on either side, well inside the
        // extent's 1 % margin (1.0 here); r = 6 leaves it by 5.
        for r in [1.5, 2.0, 6.0] {
            let pair = rim_pair((99.0, 50.0), r, s);
            let arcs = pair.trim().unwrap().expect("exact arcs");
            assert!(!arcs.is_empty());
            for arc in &arcs {
                assert!(
                    on_rim(arc_midpoint(arc), s),
                    "r={r} scale {s}: span through {:?} is off the rim",
                    arc_midpoint(arc)
                );
            }
            // The kept spans end on the two crossed lines, at the closed-form
            // ordinates. (The wall's own seam also splits the section, at
            // the circle's angular origin; it is not a rim crossing.)
            let seam = pair.raw.p_start;
            let mut ends: Vec<(f64, f64)> = arcs
                .iter()
                .flat_map(|arc| [arc.p_start, arc.p_end])
                .filter(|p| (*p - seam).length() > 1e-9 * s)
                .map(|p| (p.x() / s, (p.y() / s - 50.0).abs()))
                .collect();
            ends.sort_by(|a, b| a.0.total_cmp(&b.0));
            ends.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9);
            let half = (r * r - 1.0).sqrt();
            assert_eq!(ends.len(), 2, "r={r} scale {s}: {ends:?}");
            for ((x, dy), wanted_x) in ends.into_iter().zip([98.0, 100.0]) {
                assert!((x - wanted_x).abs() < 1e-12 && (dy - half).abs() < 1e-12);
            }
        }
    }
}

/// A section that leaves a hole-free face by less than the extent's margin
/// keeps nothing outside it: the cavity floor of the shelled box (2..98
/// square) under a cutter that overlaps the wall by 1.
#[test]
fn a_span_that_leaves_the_face_inside_the_extent_margin_is_dropped() {
    for s in SCALES {
        let pair = Pair::new(
            &square(48.0, (50.0, 50.0)),
            |_| Vec::new(),
            (97.5, 50.0),
            1.5,
            s,
        );
        let arcs = pair.trim().unwrap().expect("exact arcs");
        assert!(!arcs.is_empty());
        for arc in &arcs {
            let mid = arc_midpoint(arc);
            assert!(
                mid.x() / s < 98.0,
                "scale {s}: span through {mid:?} is in the wall"
            );
        }
    }
}

/// A section tangent to the outline that crosses the hole pinches the rim to
/// a point: refused by name. Tangent to both, or tangent to the hole while
/// crossing the outline, is not a pinch through the hole and is left alone.
#[test]
fn a_touch_with_a_hole_crossing_is_refused_and_a_touch_alone_is_not() {
    for s in SCALES {
        for (at, r) in [
            ((98.0, 50.0), 2.0),
            ((97.0, 50.0), 3.0),
            ((98.0, 98.0), 2.0),
        ] {
            let Err(error) = rim_pair(at, r, s).trim() else {
                panic!("pinched rim at {at:?} r={r} scale {s} was not refused")
            };
            assert!(error.to_string().contains("narrows to a point"), "{error}");
        }
        // Tangent to both sides of the wall, no crossing at all: whatever
        // the wall's seam splits off stays on the rim.
        for arc in rim_pair((99.0, 50.0), 1.0, s)
            .trim()
            .unwrap()
            .unwrap_or_default()
        {
            let mid = arc_midpoint(&arc);
            assert!(
                (98.0..=100.0).contains(&(mid.x() / s)),
                "scale {s}: {mid:?}"
            );
        }
        // Tangent to the cavity side, crossing the outline: the hole adds
        // nothing, and the span on the rim is kept whole.
        let arcs = rim_pair((99.5, 50.0), 1.5, s)
            .trim()
            .unwrap()
            .expect("arcs");
        for arc in &arcs {
            let mid = arc_midpoint(arc);
            assert!(mid.x() / s <= 100.0 + 1e-9, "scale {s}: {mid:?}");
        }
    }
}

// ── closed_circle_boundary_crossings ────────────────────────────────────

/// A tube's end face (an annulus) under a cutter on its bore rim: the
/// closed-section collector reads the hole, at the two circle–circle points.
#[test]
fn the_closed_circle_collector_reads_a_plane_faces_holes() {
    let tol = Tolerance::new();
    for s in SCALES {
        let z = 50.0 * s;
        let annulus = |topo: &mut Topology| {
            let outer = circle_wire(topo, &circle_at((0.0, 0.0), z, 10.0 * s), true);
            let inner = circle_wire(topo, &circle_at((0.0, 0.0), z, 5.0 * s), false);
            topo.add_face(Face::new(outer, vec![inner], plane_at(z)))
        };
        let hits = |at: f64, r: f64| {
            let mut topo = Topology::new();
            let plane = annulus(&mut topo);
            let (wall, _) = cylinder_wall(
                &mut topo,
                (at * s, 0.0),
                r * s,
                (z - 30.0 * s, z + 30.0 * s),
            );
            let circle = circle_at((at * s, 0.0), z, r * s);
            closed_circle_boundary_crossings(&topo, plane, wall, &circle, tol)
        };

        // Radius 2 on the bore rim (distance 5): crosses the bore at
        // x = (25 + 25 − 4)/10, y = ±√(25 − x²). The wall's own seam adds
        // one hit at its angular origin.
        let (found, contacts) = hits(5.0, 2.0).unwrap();
        let x = (25.0 + 25.0 - 4.0) / 10.0;
        let y = (25.0_f64 - x * x).sqrt();
        let on_bore: Vec<_> = found
            .iter()
            .map(|(_, p)| (p.x() / s, p.y() / s))
            .filter(|(px, py)| (px.hypot(*py) - 5.0).abs() < 1e-9)
            .collect();
        assert_eq!(on_bore.len(), 2, "scale {s}: {found:?}");
        for (px, py) in on_bore {
            assert!((px - x).abs() < 1e-12 && (py.abs() - y).abs() < 1e-12);
        }
        assert_eq!(contacts.hole_crossings.len(), 2);
        assert!(contacts.touches.is_empty());

        // Radius 2 at distance 7 touches the bore from outside and stays
        // clear of the outline: no hit on the bore, one touch at (5, 0).
        let (found, contacts) = hits(7.0, 2.0).unwrap();
        assert!(
            found
                .iter()
                .all(|(_, p)| ((p.x() / s).hypot(p.y() / s) - 5.0).abs() > 1e-6),
            "scale {s}: {found:?}"
        );
        assert!(contacts.hole_crossings.is_empty());
        assert_eq!(contacts.touches.len(), 1, "scale {s}");
        assert!((contacts.touches[0] - Point3::new(5.0 * s, 0.0, z)).length() < 1e-9 * s);

        // Radius 3 at distance 7 crosses the bore and touches the outline
        // at (10, 0): the annulus is pinched there.
        let (_, contacts) = hits(7.0, 3.0).unwrap();
        assert_eq!(contacts.hole_crossings.len(), 2, "scale {s}");
        assert_eq!(contacts.touches.len(), 1, "scale {s}");
        assert!((contacts.touches[0] - Point3::new(10.0 * s, 0.0, z)).length() < 1e-9 * s);
        let circle = circle_at((7.0 * s, 0.0), z, 3.0 * s);
        let project = |p: Point3| circle.project(p);
        let (touch, bore) = (
            project(contacts.touches[0]),
            project(contacts.hole_crossings[0]),
        );
        // Spans kept from one bore crossing round through the touch pinch
        // the face; the same contacts with only the span away from the touch
        // kept (the partner face a patch on the bore's side) do not.
        let refusal = contacts
            .pinch(&[(touch - 3.0, touch + 3.0)], project, 3.0 * s, tol)
            .expect("pinched annulus");
        assert!(
            refusal.to_string().contains("narrows to a point"),
            "{refusal}"
        );
        assert!(
            contacts
                .pinch(&[(bore - 0.2, bore + 0.2)], project, 3.0 * s, tol)
                .is_none()
        );
        assert!(
            contacts
                .pinch(&[(touch - 0.2, touch + 0.2)], project, 3.0 * s, tol)
                .is_none()
        );
    }
}
