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

use super::plane_section_certificate::certify_hole;
use super::{
    Aabb3, AlgoError, Edge, EdgeCurve, FaceExtent, FaceId, FaceSurface, GfaArena, LineContact,
    Point3, RawCurve, SectionConic, Tolerance, Topology, Vec3, Vertex,
    closed_circle_boundary_crossings, emit_split_circle_arcs, plane_hole_crossings,
    trim_ellipse_to_boundary_crossings,
};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
const SCALES: [f64; 3] = [0.1, 1.0, 10.0];

// ── Fixtures ────────────────────────────────────────────────────────────

pub(super) fn vertex(topo: &mut Topology, p: Point3) -> remus_topology::vertex::VertexId {
    topo.add_vertex(Vertex::new(p, 1e-7))
}

/// A closed loop of straight edges through `corners`, in that order.
pub(super) fn polygon_wire(topo: &mut Topology, corners: &[(f64, f64)], z: f64) -> WireId {
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
pub(super) fn circle_wire(topo: &mut Topology, circle: &Circle3D, forward: bool) -> WireId {
    let seam = vertex(topo, circle.evaluate(0.0));
    let mut edge = Edge::with_tolerance(seam, seam, EdgeCurve::Circle(circle.clone()), Some(1e-7));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
}

pub(super) fn plane_at(z: f64) -> FaceSurface {
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
        assert!(
            matches!(contact(-1.5 * s), LineContact::Cross(_)),
            "chord, scale {s}"
        );
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
        assert!(matches!(
            contact(-s - 3.0 * tol.linear),
            LineContact::Cross(_)
        ));
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
        assert!(
            matches!(line(0.2), LineContact::Cross(_)),
            "chord at t = {t}"
        );
        assert_eq!(line(-0.2), LineContact::Miss, "clear at t = {t}");
    }
    assert!(SectionConic::of_curve(&EdgeCurve::Line).is_none());
}

// ── Eccentric sections ──────────────────────────────────────────────────

/// The section of a radius-`b` cylinder tilted in the xz-plane so that the
/// plane `z = z0` cuts it in an ellipse of semi-axes `(a, b)`, major axis
/// along x, centred on the z axis (`cos θ = b / a`).
fn tilted_section(a: f64, b: f64, z0: f64) -> (Ellipse3D, FaceSurface) {
    let cos = b / a;
    let axis = Vec3::new(-(1.0 - cos * cos).sqrt(), 0.0, cos);
    let centre = Point3::new(0.0, 0.0, z0);
    let ellipse = Ellipse3D::new_with_ref(centre, Z, a, b, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let surface = FaceSurface::Cylinder(CylindricalSurface::new(centre, axis, b).unwrap());
    (ellipse, surface)
}

/// Half the chord the line `x = a − depth` cuts from the ellipse of
/// semi-axes `(a, b)`: `b √(1 − (x/a)²)`, written without cancellation.
fn chord_half(a: f64, b: f64, depth: f64) -> f64 {
    b * (depth * (2.0 * a - depth)).sqrt() / a
}

/// A straight hole edge 1e-5 inside an eccentric ellipse's major vertex is
/// crossed: the curve passes 100 × the tolerance beyond it. Its reach used
/// to be measured in the chart that maps the ellipse to the unit circle and
/// scaled by the smaller semi-axis, which read 1e-8 for axes (1000, 1) and
/// called it a touch.
#[test]
fn an_eccentric_ellipse_past_a_line_by_more_than_the_tolerance_crosses_it() {
    let tol = Tolerance::new();
    for (a, b) in [(10.0, 1.0), (100.0, 1.0), (1000.0, 1.0), (1000.0, 30.0)] {
        for s in SCALES {
            let (ellipse, _) = tilted_section(a * s, b * s, 0.0);
            let conic = SectionConic::of_curve(&EdgeCurve::Ellipse(ellipse)).unwrap();
            let edge = |x: f64| {
                conic.line_contact(
                    Point3::new(x, -0.25 * s, 0.0),
                    Point3::new(x, 0.25 * s, 0.0),
                    tol,
                )
            };
            let depth = 1e-5 * s;
            let what = format!("axes ({a}, {b}) scale {s}");
            let LineContact::Cross(chord) = edge(a * s - depth) else {
                panic!("{what}: {:?}", edge(a * s - depth));
            };
            // The reach `h − s` carries the rounding of coordinates `a`
            // across (about 1e-14 · a); the half chord, its square root,
            // half that relative to the reach.
            let half = chord_half(a * s, b * s, depth);
            assert!(
                (chord.half - half).abs() <= 1e-6 * half,
                "{what}: half chord {} vs {half}",
                chord.half
            );
            assert!((chord.mid - 0.25 * s).abs() <= 1e-12 * s, "{what}");
            assert_eq!(chord.ends_inside(), Some(2), "{what}");
            // The band is the linear tolerance in length units, at every
            // eccentricity.
            assert!(
                matches!(edge(a * s - 0.5 * tol.linear), LineContact::Touch(_)),
                "{what}"
            );
            assert!(
                matches!(edge(a * s - 3.0 * tol.linear), LineContact::Cross(_)),
                "{what}"
            );
            assert_eq!(edge(a * s + 3.0 * tol.linear), LineContact::Miss, "{what}");
        }
    }
}

/// Along the major axis, near the minor vertex, the chord of a shallow
/// crossing is long: `a √(1 − (y/b)²)`.
#[test]
fn an_eccentric_ellipse_crossed_near_its_minor_vertex_has_the_closed_form_chord() {
    let tol = Tolerance::new();
    for s in SCALES {
        let (a, b) = (1000.0 * s, s);
        let (ellipse, _) = tilted_section(a, b, 0.0);
        let conic = SectionConic::of_curve(&EdgeCurve::Ellipse(ellipse)).unwrap();
        let depth = 1e-5 * s;
        let y = b - depth;
        let contact = conic.line_contact(
            Point3::new(-2.0 * a, y, 0.0),
            Point3::new(2.0 * a, y, 0.0),
            tol,
        );
        let LineContact::Cross(chord) = contact else {
            panic!("scale {s}: {contact:?}");
        };
        let half = chord_half(b, a, depth);
        assert!((chord.half - half).abs() <= 1e-6 * half, "scale {s}");
        assert!((chord.mid - 2.0 * a).abs() <= 1e-9 * a, "scale {s}");
    }
}

/// A slot `[near, near + 3] × [−w, w]` in the plane z = 0, as a hole wire.
fn slot_hole(topo: &mut Topology, near: f64, w: f64) -> WireId {
    polygon_wire(
        topo,
        &[
            (near, -w),
            (near, w),
            (near + 3.0 * w, w),
            (near + 3.0 * w, -w),
        ],
        0.0,
    )
}

/// The crossings of the eccentric section of semi-axes `(a, b)` with a slot
/// whose near end stands `depth` inside the section's major vertex.
fn eccentric_slot(
    a: f64,
    b: f64,
    depth: f64,
    w: f64,
) -> Result<Option<super::PlaneHoleCrossings>, super::AlgoError> {
    let (ellipse, cylinder) = tilted_section(a, b, 0.0);
    let mut topo = Topology::new();
    let hole = slot_hole(&mut topo, a - depth, w);
    plane_hole_crossings(
        &topo,
        hole,
        &EdgeCurve::Ellipse(ellipse),
        &cylinder,
        Tolerance::new(),
    )
}

/// A hole edge inside an eccentric section's major vertex by enough that
/// the edge enters the cylinder by ten tolerances (`depth · b / a` from the
/// surface: the plane meets the tilted cylinder at a grazing angle there):
/// both crossings, at the closed-form height.
#[test]
fn an_eccentric_ellipse_crosses_a_hole_edge_that_enters_its_cylinder() {
    let tol = Tolerance::new();
    for a in [10.0, 100.0, 1000.0] {
        for s in SCALES {
            let what = format!("a = {a}, scale {s}");
            let depth = 10.0 * tol.linear * a;
            let found = eccentric_slot(a * s, s, depth, 0.25 * s).unwrap().unwrap();
            assert!(found.touches.is_empty(), "{what}");
            let mut ys: Vec<f64> = found.crossings.iter().map(|p| p.y()).collect();
            ys.sort_by(f64::total_cmp);
            let half = chord_half(a * s, s, depth);
            assert_eq!(ys.len(), 2, "{what}");
            for (y, wanted) in ys.iter().zip([-half, half]) {
                assert!(
                    (y - wanted).abs() <= 1e-6 * half,
                    "{what}: y {y} vs {wanted}"
                );
            }
            for p in &found.crossings {
                assert!((p.x() - (a * s - depth)).abs() <= 1e-12 * a * s, "{what}");
            }
        }
    }
}

/// The reviewer's case, and its family: axes (1000, 1), a hole edge 1e-5
/// inside the major vertex. In the plane the curve passes the edge by 100
/// tolerances, yet the edge stays within 1e-8 of the tilted cylinder along
/// the whole 2.8e-4 chord, so a crossing pair and a touch are the same
/// contact to within tolerance. It is refused by name; it used to be read
/// as a touch and its crossings dropped.
#[test]
fn an_eccentric_ellipse_whose_hole_edge_grazes_its_cylinder_is_refused() {
    let tol = Tolerance::new();
    let refused = |a: f64, b: f64, depth: f64| {
        let error = eccentric_slot(a, b, depth, 0.25 * b)
            .map(|_| ())
            .expect_err("a grazing hole edge");
        assert!(
            error
                .to_string()
                .contains("within tolerance of the cut surface"),
            "{error}"
        );
    };
    refused(1000.0, 1.0, 1e-5);
    for s in SCALES {
        // The edge a tenth of the tolerance from the cylinder.
        refused(1000.0 * s, s, 0.1 * tol.linear * 1000.0);
    }
}

/// A section that passes a hole edge by more than the tolerance along a
/// chord shorter than the distance at which two crossings merge cannot be
/// split there: refused, rather than kept or dropped across the hole.
#[test]
fn an_unresolvable_needle_past_a_hole_edge_is_refused() {
    // Axes (1, 5e-5), the edge 4e-3 inside the major vertex: it enters the
    // cylinder by 2e-7, along a chord 8.9e-6 long, under the 1e-5 at which
    // the trim takes two crossings for one.
    let error = eccentric_slot(1.0, 5e-5, 4e-3, 1e-4)
        .map(|_| ())
        .expect_err("an unresolvable chord");
    assert!(
        error
            .to_string()
            .contains("crossing(s) of the edge bound it"),
        "{error}"
    );
}

#[test]
fn a_chord_counts_its_ends_inside_the_edge_and_defers_at_an_end() {
    let chord = |mid: f64, half: f64| super::Chord {
        mid,
        half,
        length: 10.0,
        slack: 0.1,
        deepest: Point3::new(0.0, 0.0, 0.0),
        band: 0.01,
    };
    assert_eq!(chord(5.0, 1.0).ends_inside(), Some(2));
    assert_eq!(chord(0.0, 1.0).ends_inside(), Some(1));
    assert_eq!(chord(-5.0, 1.0).ends_inside(), Some(0));
    assert_eq!(
        chord(1.0, 1.0).ends_inside(),
        None,
        "an end at the edge's start"
    );
    assert_eq!(
        chord(9.05, 0.9).ends_inside(),
        None,
        "an end at the edge's end"
    );
    let p = |y: f64| Point3::new(0.0, y, 0.0);
    assert!(chord(5.0, 1.0).resolved_by(&[p(4.0), p(6.0)], 1e-5));
    assert!(!chord(5.0, 1.0).resolved_by(&[p(4.0)], 1e-5));
    assert!(!chord(5.0, 1.0).resolved_by(&[p(4.0), p(4.0 + 1e-6)], 1e-5));
    assert!(chord(1.0, 1.0).resolved_by(&[], 1e-5), "undecided count");
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
        let found = plane_hole_crossings(&pair.topo, hole, &pair.raw.curve, &pair.surfaces.1, tol)
            .unwrap()
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
        let found = plane_hole_crossings(&pair.topo, hole, &pair.raw.curve, &pair.surfaces.1, tol)
            .unwrap()
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
            let found =
                plane_hole_crossings(&pair.topo, hole, &pair.raw.curve, &pair.surfaces.1, tol)
                    .unwrap()
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
                let found =
                    plane_hole_crossings(&pair.topo, hole, &pair.raw.curve, &pair.surfaces.1, tol)
                        .unwrap()
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
                    tol,
                )
                .unwrap()
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

/// The closed-circle collector against a straight hole edge 1e-5 inside a
/// circular section (100 tolerances in the plane), the section cut from a
/// cone. A cone standing steeply on the plane is entered by the edge and
/// crossed twice; a nearly flat one (generator 1e-3 off the plane) is
/// entered by only 1e-8 along the whole chord, and the pair is refused:
/// the collector hands the crossings over with the refusal attached, and
/// the emitter raises it for the spans it would keep beside them.
#[test]
fn the_closed_circle_collector_refuses_a_hole_edge_grazing_a_flat_cone() {
    use remus_math::surfaces::ConicalSurface;

    let tol = Tolerance::new();
    for s in SCALES {
        let (radius, depth) = (5.0 * s, 1e-5 * s);
        let build = |slope: f64| {
            let mut topo = Topology::new();
            let outer = polygon_wire(&mut topo, &square(10.0 * s, (0.0, 0.0)), 0.0);
            let near = radius - depth;
            let hole = polygon_wire(
                &mut topo,
                &[
                    (near, -s),
                    (near, s),
                    (near + 2.0 * s, s),
                    (near + 2.0 * s, -s),
                ],
                0.0,
            );
            let plane = topo.add_face(Face::new(outer, vec![hole], plane_at(0.0)));
            // `ρ = t · cot(slope)`: the cone meets z = 0 in the circle of
            // `radius` when its apex is `radius · tan(slope)` below.
            let apex = Point3::new(0.0, 0.0, -radius * slope.tan());
            let cone = FaceSurface::Cone(ConicalSurface::new(apex, Z, slope).unwrap());
            let (wall, _) = cylinder_wall(&mut topo, (0.0, 0.0), radius, (-s, s));
            topo.face_mut(wall).unwrap().set_surface(cone);
            let circle = circle_at((0.0, 0.0), 0.0, radius);
            (topo, plane, wall, circle)
        };
        let hits = |slope: f64| {
            let (topo, plane, wall, circle) = build(slope);
            closed_circle_boundary_crossings(&topo, plane, wall, &circle, tol)
        };

        let (_, contacts) = hits(0.5).unwrap();
        assert_eq!(contacts.hole_crossings.len(), 2, "scale {s}");
        let y = (depth * (2.0 * radius - depth)).sqrt();
        for p in &contacts.hole_crossings {
            assert!((p.y().abs() - y).abs() <= 1e-6 * y, "scale {s}: {p:?}");
        }
        // The steep cone's spans pass the gate: the arc over the hole is
        // dropped, the rest kept.
        let (mut topo, plane, wall, circle) = build(0.5);
        let kept = emit(&mut topo, plane, wall, &circle).unwrap();
        assert!(!kept.is_empty(), "scale {s}");
        assert!(
            !kept.iter().any(|&(t0, t1)| t0 < TAU && t1 > TAU),
            "scale {s}: an arc through the hole at t = 0: {kept:?}"
        );

        let (found, contacts) = hits(1e-3).unwrap();
        let error = contacts
            .certificate
            .as_ref()
            .unwrap()
            .plan(&found)
            .first_refusal()
            .expect("a grazing hole edge");
        assert!(
            error
                .to_string()
                .contains("within tolerance of the cut surface"),
            "scale {s}: {error}"
        );
        let (mut topo, plane, wall, circle) = build(1e-3);
        let error = emit(&mut topo, plane, wall, &circle).expect_err("a grazing hole edge");
        assert!(
            error
                .to_string()
                .contains("within tolerance of the cut surface"),
            "scale {s}: {error}"
        );
    }
}

// ── Contacts of the finite edge, corners and the merge distance ──────────

/// The crossings of a section with one hole, through the ellipse trim's
/// collector.
fn hole_hits(
    topo: &Topology,
    hole: WireId,
    section: &EdgeCurve,
    surface: &FaceSurface,
) -> Result<Option<super::PlaneHoleCrossings>, super::AlgoError> {
    plane_hole_crossings(topo, hole, section, surface, Tolerance::new())
}

/// The vertical cylinder of radius `r` about `(cx, cy)` and its section with
/// z = 0.
fn upright(cx: f64, cy: f64, r: f64) -> (EdgeCurve, FaceSurface) {
    (
        EdgeCurve::Circle(circle_at((cx, cy), 0.0, r)),
        FaceSurface::Cylinder(CylindricalSurface::new(Point3::new(cx, cy, -1.0), Z, r).unwrap()),
    )
}

/// The reviewer's case: axes (1000, 1) and a rectangular hole spanning
/// x = 999.99999..1002.99999 at y = 10..11. The line of its near side
/// crosses the section 1e-5 inside the major vertex, at y ≈ 0, where it
/// grazes the tilted cylinder; the side itself, ten units away, never meets
/// the section, which only spans y = −1..1. The chord's midpoint on the
/// LINE was tested before the chord was found to miss the edge, so this
/// disjoint hole refused the whole boolean. It contributes nothing.
#[test]
fn a_hole_beside_an_eccentric_section_whose_edge_line_grazes_it_contributes_nothing() {
    let (ellipse, cylinder) = tilted_section(1000.0, 1.0, 0.0);
    let mut topo = Topology::new();
    let hole = polygon_wire(
        &mut topo,
        &[
            (999.999_99, 10.0),
            (1_002.999_99, 10.0),
            (1_002.999_99, 11.0),
            (999.999_99, 11.0),
        ],
        0.0,
    );
    let found = hole_hits(&topo, hole, &EdgeCurve::Ellipse(ellipse), &cylinder)
        .expect("a disjoint hole is not refused")
        .expect("a straight-edged hole is consulted");
    assert!(found.crossings.is_empty(), "{:?}", found.crossings);
    assert!(found.touches.is_empty(), "{:?}", found.touches);
}

/// Holes clear of the section whose four side LINES each cross it or
/// graze it, at every side of the section: near the major vertices (where
/// the lines graze the tilted cylinder), near the minor vertices, and a
/// square off a circle's diagonal whose four lines all cross the circle.
/// None contributes a crossing or a touch, at every scale.
#[test]
fn holes_clear_of_the_section_whose_edge_lines_cross_or_graze_it_contribute_nothing() {
    let rect =
        |(x0, x1): (f64, f64), (y0, y1): (f64, f64)| [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    for s in SCALES {
        let (a, b) = (1000.0 * s, s);
        let (ellipse, cylinder) = tilted_section(a, b, 0.0);
        let poke = 1e-5 * s;
        // The ellipse at |x| = 10 s reaches |y| = b √(1 − 1e-4) < b − 1e-5 s.
        let holes = [
            rect((a - poke, a + 3.0 * s), (10.0 * s, 11.0 * s)),
            rect((a - poke, a + 3.0 * s), (-11.0 * s, -10.0 * s)),
            rect((-a - 3.0 * s, -a + poke), (10.0 * s, 11.0 * s)),
            rect((10.0 * s, 11.0 * s), (b - poke, 2.0 * s)),
            rect((-11.0 * s, -10.0 * s), (-2.0 * s, -b + poke)),
        ];
        for corners in holes {
            let mut topo = Topology::new();
            let hole = polygon_wire(&mut topo, &corners, 0.0);
            let found = hole_hits(&topo, hole, &EdgeCurve::Ellipse(ellipse.clone()), &cylinder)
                .unwrap_or_else(|e| panic!("scale {s} {corners:?}: {e}"))
                .unwrap();
            assert!(
                found.crossings.is_empty() && found.touches.is_empty(),
                "scale {s} {corners:?}: {:?} {:?}",
                found.crossings,
                found.touches
            );
        }
        // A circle of radius 5 s and a square off its diagonal: every side
        // line crosses the circle, the nearest corner is 5.3 s from it.
        let (section, surface) = upright(0.0, 0.0, 5.0 * s);
        for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
            let (x0, x1) = if sx > 0.0 {
                (3.75 * s, 4.75 * s)
            } else {
                (-4.75 * s, -3.75 * s)
            };
            let (y0, y1) = if sy > 0.0 {
                (3.75 * s, 4.75 * s)
            } else {
                (-4.75 * s, -3.75 * s)
            };
            let mut topo = Topology::new();
            let hole = polygon_wire(&mut topo, &rect((x0, x1), (y0, y1)), 0.0);
            let found = hole_hits(&topo, hole, &section, &surface).unwrap().unwrap();
            assert!(
                found.crossings.is_empty() && found.touches.is_empty(),
                "scale {s} quadrant ({sx}, {sy})"
            );
        }
    }
}

/// The closed-circle collector meets the same disjoint hole the same way: a
/// rectangle beside a circle cut from a nearly flat cone, whose near side's
/// LINE passes 1e-5 inside the circle (where the plane grazes the cone),
/// the side itself well clear of it. The circle also leaves the face across
/// its outline (x = −4), so the face's boundary is read at all. It used to
/// refuse the pair.
#[test]
fn the_closed_circle_collector_ignores_a_hole_whose_edge_line_only_grazes() {
    use remus_math::surfaces::ConicalSurface;

    let tol = Tolerance::new();
    for s in SCALES {
        let radius = 5.0 * s;
        let near = radius - 1e-5 * s;
        let mut topo = Topology::new();
        let outer = polygon_wire(
            &mut topo,
            &[
                (-4.0 * s, -12.0 * s),
                (12.0 * s, -12.0 * s),
                (12.0 * s, 12.0 * s),
                (-4.0 * s, 12.0 * s),
            ],
            0.0,
        );
        let hole = polygon_wire(
            &mut topo,
            &[
                (near, 8.0 * s),
                (near + 2.0 * s, 8.0 * s),
                (near + 2.0 * s, 9.0 * s),
                (near, 9.0 * s),
            ],
            0.0,
        );
        let plane = topo.add_face(Face::new(outer, vec![hole], plane_at(0.0)));
        let slope: f64 = 1e-3;
        let apex = Point3::new(0.0, 0.0, -radius * slope.tan());
        let cone = FaceSurface::Cone(ConicalSurface::new(apex, Z, slope).unwrap());
        let (wall, _) = cylinder_wall(&mut topo, (0.0, 0.0), radius, (-s, s));
        topo.face_mut(wall).unwrap().set_surface(cone);
        let circle = circle_at((0.0, 0.0), 0.0, radius);
        let (found, contacts) = closed_circle_boundary_crossings(&topo, plane, wall, &circle, tol)
            .unwrap_or_else(|e| panic!("scale {s}: {e}"));
        assert!(contacts.hole_crossings.is_empty(), "scale {s}");
        assert!(contacts.touches.is_empty(), "scale {s}");
        // The outline's two crossings at x = −4, y = ±3.
        let on_outline = found
            .iter()
            .filter(|(_, p)| {
                (p.x() + 4.0 * s).abs() <= 1e-9 * s && (p.y().abs() - 3.0 * s).abs() <= 1e-9 * s
            })
            .count();
        assert_eq!(on_outline, 2, "scale {s}: {found:?}");
    }
}

/// The reviewer's corner case: axes (1, 5e-5), a hole whose side x = 0.996
/// runs from its corner at y = −4.46766158e-6 — exactly where the section
/// crosses that line — to y = 1e-4. The two crossings of the side are
/// 8.9e-6 apart, under the 1e-5 at which the trim takes them for one, and
/// the section between them runs 4.5e-6 inside the hole: they are neither
/// one crossing nor a touch. The end at the corner used to exempt the side
/// from the separation check, and the trim then dropped one of the two.
#[test]
fn crossings_at_a_hole_corner_closer_than_the_merge_distance_are_refused() {
    let (ellipse, cylinder) = tilted_section(1.0, 5e-5, 0.0);
    let corner_y = -4.467_661_58e-6;
    let mut topo = Topology::new();
    let hole = polygon_wire(
        &mut topo,
        &[
            (0.996, corner_y),
            (1.1, corner_y),
            (1.1, 1e-4),
            (0.996, 1e-4),
        ],
        0.0,
    );
    let error = hole_hits(&topo, hole, &EdgeCurve::Ellipse(ellipse), &cylinder)
        .map(|_| ())
        .expect_err("two crossings closer than the merge distance");
    assert!(
        error
            .to_string()
            .contains("the distance at which two crossings are one"),
        "{error}"
    );
    assert!(error.to_string().contains("across the hole"), "{error}");
}

/// A circle through the corner shared by two hole edges, entering the hole
/// there: one crossing at the corner, whichever edges find it, and the
/// other where it leaves; at scales 1e-3, 1 and 1e6.
///
/// The circle of radius 5 about (−3, 4) passes through the hole's corner at
/// the origin heading (0.8, 0.6), into the square [0, 10]², and leaves it
/// across the side x = 0 at y = 8.
#[test]
fn a_section_through_a_hole_corner_crosses_it_once() {
    for s in [1e-3, 1.0, 1e6] {
        let (section, surface) = upright(-3.0 * s, 4.0 * s, 5.0 * s);
        for reversed in [false, true] {
            let mut corners = square(5.0 * s, (5.0 * s, 5.0 * s)).to_vec();
            if reversed {
                corners.reverse();
            }
            let mut topo = Topology::new();
            let hole = polygon_wire(&mut topo, &corners, 0.0);
            let found = hole_hits(&topo, hole, &section, &surface)
                .unwrap_or_else(|e| panic!("scale {s}: {e}"))
                .unwrap();
            let what = format!("scale {s} reversed {reversed}: {:?}", found.crossings);
            assert!(found.touches.is_empty(), "{what}");
            assert_eq!(found.crossings.len(), 2, "{what}");
            let at_corner = found
                .crossings
                .iter()
                .filter(|p| (p.x().hypot(p.y())) <= 1e-12 * s.max(1.0))
                .count();
            assert_eq!(at_corner, 1, "{what}");
            assert!(
                found
                    .crossings
                    .iter()
                    .any(|p| p.x().abs() <= 1e-12 * s.max(1.0)
                        && (p.y() - 8.0 * s).abs() <= 1e-12 * s.max(1.0)),
                "{what}"
            );
        }
    }
}

/// A circle through a hole's corner that stays outside the hole on both
/// sides of it (its tangent there points out of the corner) touches the
/// hole: no crossing at the corner, one touch, and its two real crossings.
#[test]
fn a_section_touching_a_hole_corner_from_outside_is_a_touch() {
    for s in [1e-3, 1.0, 1e6] {
        // Centre (3, 4), radius 5: through the origin with tangent
        // (0.8, −0.6); it meets y = 0 again at x = 6 and x = 0 at y = 8.
        let (section, surface) = upright(3.0 * s, 4.0 * s, 5.0 * s);
        let mut topo = Topology::new();
        let hole = polygon_wire(&mut topo, &square(5.0 * s, (5.0 * s, 5.0 * s)), 0.0);
        let found = hole_hits(&topo, hole, &section, &surface)
            .unwrap_or_else(|e| panic!("scale {s}: {e}"))
            .unwrap();
        let what = format!("scale {s}: {:?} {:?}", found.crossings, found.touches);
        assert_eq!(found.crossings.len(), 2, "{what}");
        assert!(
            found.crossings.iter().all(|p| p.x().hypot(p.y()) > 5.0 * s),
            "{what}"
        );
        assert_eq!(found.touches.len(), 1, "{what}");
        assert!(found.touches[0].x().hypot(found.touches[0].y()) <= 1e-12 * s.max(1.0));
    }
}

/// Hole corners exactly on, just short of, and just past a circle, with
/// both edges at the corner heading away from it. On: a touch at the
/// corner, no crossing. Short by ten tolerances: nothing. Past by a hundred
/// tolerances: two crossings, one on each edge. Past by half the
/// tolerance: a touch.
#[test]
fn hole_corners_on_short_of_and_past_a_section() {
    let tol = Tolerance::new().linear;
    for s in [1e-3, 1.0, 1e6] {
        let r = 5.0 * s;
        let (section, surface) = upright(0.0, 0.0, r);
        // The corner on the ray through (3, 4), `shift` outside the circle,
        // and the hole running away from the centre.
        let hole_at = |shift: f64| {
            let k = 1.0 + shift / r;
            let (x, y) = (3.0 * s * k, 4.0 * s * k);
            let mut topo = Topology::new();
            let hole = polygon_wire(
                &mut topo,
                &[
                    (x, y),
                    (x + 3.0 * s, y),
                    (x + 3.0 * s, y + 2.0 * s),
                    (x, y + 2.0 * s),
                ],
                0.0,
            );
            let found = hole_hits(&topo, hole, &section, &surface)
                .unwrap_or_else(|e| panic!("scale {s} shift {shift:e}: {e}"))
                .unwrap();
            (found, Point3::new(x, y, 0.0))
        };
        let (on, corner) = hole_at(0.0);
        assert!(on.crossings.is_empty(), "on, scale {s}: {:?}", on.crossings);
        assert_eq!(on.touches.len(), 1, "on, scale {s}");
        assert!(
            (on.touches[0] - corner).length() <= 2.0 * tol,
            "on, scale {s}"
        );

        let (short, _) = hole_at(10.0 * tol);
        assert!(
            short.crossings.is_empty() && short.touches.is_empty(),
            "short, scale {s}: {:?} {:?}",
            short.crossings,
            short.touches
        );

        let (past, corner) = hole_at(-100.0 * tol);
        assert!(past.touches.is_empty(), "past, scale {s}");
        assert_eq!(
            past.crossings.len(),
            2,
            "past, scale {s}: {:?}",
            past.crossings
        );
        for p in &past.crossings {
            assert!(
                ((p.x().hypot(p.y())) - r).abs() <= 1e-9 * r.max(1.0),
                "past, scale {s}: {p:?} off the circle"
            );
            let on_bottom = (p.y() - corner.y()).abs() <= 1e-12 * s.max(1.0);
            let on_left = (p.x() - corner.x()).abs() <= 1e-12 * s.max(1.0);
            assert!(on_bottom != on_left, "past, scale {s}: {p:?}");
        }

        let (graze, _) = hole_at(-0.5 * tol);
        assert!(
            graze.crossings.is_empty(),
            "graze, scale {s}: {:?}",
            graze.crossings
        );
        assert_eq!(graze.touches.len(), 1, "graze, scale {s}");
    }
}

/// Two crossings of a hole's corner a few tolerances apart: the circle cuts
/// the corner at origin of the square [0, 20]² with the corner `depth`
/// inside it, so it crosses the two sides √2 · depth from the corner and
/// 2 · depth apart, and runs 1e-6 inside the hole between them. They are
/// two crossings, 3e-6 apart: closer than the separation distance at every
/// scale, so the trim refuses by name rather than keep a span that ends on
/// them.
///
/// At scale 1e-3 this trim used to be accepted, with both crossings as span
/// ends. The face splitter then welds the two (its boundary weld is 1e-5
/// on these 0.02-long edges), so the hole's vertex set and the section's
/// disagreed: the separation distance is that weld, at every scale.
#[test]
fn crossings_a_few_tolerances_apart_are_refused_at_every_scale() {
    let depth = 1.5e-6;
    for s in [1e-3, 1.0, 1e6] {
        let r = 5.0;
        // The centre on the diagonal, `r · s − depth` from the corner.
        let k = (r * s - depth) / std::f64::consts::SQRT_2 / s;
        let pair = Pair::new(
            &square(50.0, (0.0, 0.0)),
            |topo| {
                let mut hole = square(10.0 * s, (10.0 * s, 10.0 * s)).to_vec();
                hole.reverse();
                vec![polygon_wire(topo, &hole, 50.0 * s)]
            },
            (-k, -k),
            r,
            s,
        );
        // The certificate places both crossings where they are.
        let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
        let certified = certify_hole(
            &pair.topo,
            hole,
            &pair.raw.curve,
            Some(&pair.surfaces.1),
            Tolerance::new(),
        )
        .unwrap();
        let crossings: Vec<Point3> = certified.crossings().map(|c| c.at).collect();
        assert_eq!(crossings.len(), 2, "scale {s}: {crossings:?}");
        for wanted in [
            Point3::new(std::f64::consts::SQRT_2 * depth, 0.0, 50.0 * s),
            Point3::new(0.0, std::f64::consts::SQRT_2 * depth, 50.0 * s),
        ] {
            assert!(
                crossings
                    .iter()
                    .any(|p| (*p - wanted).length() <= 1e-9 * s.max(1.0)),
                "scale {s}: no crossing at {wanted:?}: {crossings:?}"
            );
        }
        let Err(error) = pair.trim() else {
            panic!("scale {s}: crossings within the separation distance were not refused");
        };
        assert!(
            error.to_string().contains("across the hole"),
            "scale {s}: {error}"
        );
    }
}

// ── Certify or refuse (the third review round's reproducers) ────────────

/// The closed-circle path end to end: the collector's hits through the
/// emitter's gate. The parameter ranges of the arcs it keeps.
fn emit(
    topo: &mut Topology,
    plane: FaceId,
    wall: FaceId,
    circle: &Circle3D,
) -> Result<Vec<(f64, f64)>, AlgoError> {
    let tol = Tolerance::new();
    let (crossings, contacts) = closed_circle_boundary_crossings(topo, plane, wall, circle, tol)?;
    let (c, r) = (circle.center(), circle.radius());
    let start = circle.evaluate(0.0);
    let raw = RawCurve {
        bbox: Aabb3::from_points([c + Vec3::new(-r, -r, -r), c + Vec3::new(r, r, r)]),
        curve: EdgeCurve::Circle(circle.clone()),
        t_range: (0.0, TAU),
        p_start: start,
        p_end: start,
    };
    let mut arena = GfaArena::new();
    emit_split_circle_arcs(
        topo, &mut arena, plane, wall, &raw, circle, &crossings, &contacts, tol,
    )?;
    Ok(arena.curves.iter().map(|curve| curve.t_range).collect())
}

/// Whether one of `arcs` runs through the point `p` of their circle.
fn covers(arcs: &[RawCurve], p: Point3) -> bool {
    arcs.iter().any(|arc| {
        let EdgeCurve::Circle(circle) = &arc.curve else {
            panic!("arc of a circle")
        };
        let (t0, t1) = arc.t_range;
        (circle.project(p) - t0).rem_euclid(TAU) <= t1 - t0
    })
}

/// The slot of the first finding, `stretch` times as tall and `stretch²`
/// times as far to the left of the circle: a hole across the unit circle at
/// (1, 0) spanning y = 0 .. 8e-6 · stretch, whose left side (3e-6 ·
/// stretch² from the circle) is notched inward to a tip 5e-8 short of the
/// circle half-way up.
fn notched_slot(stretch: f64) -> Vec<(f64, f64)> {
    let y = |k: f64| k * 1e-6 * stretch;
    let left = 1.0 - 3e-6 * stretch * stretch;
    vec![
        (left, 0.0),
        (1.001, 0.0),
        (1.001, y(8.0)),
        (left, y(8.0)),
        (left, y(6.0)),
        (0.999_999_95, y(4.0)),
        (left, y(2.0)),
    ]
}

/// Finding 1: an undecided probe is not a proof of a touch.
///
/// The unit circle runs up through the slot, 3e-6 inside the hole, and
/// passes the notch's tip by 5e-8: within the band, so the point of the
/// section half-way between its two crossings cannot be put on either
/// side. That used to read as "the section does not dip into the hole",
/// the two crossings as one touch, and the whole circle was kept. The
/// crossings are found where they are, at (1, 0) and (1, 8e-6); the tip is
/// a touch between them, inside the hole.
#[test]
fn an_undecided_probe_is_not_a_proof_of_a_touch() {
    let (section, surface) = upright(0.0, 0.0, 1.0);
    for stretch in [1.0, 10.0] {
        let top = 8e-6 * stretch;
        let mut topo = Topology::new();
        let hole = polygon_wire(&mut topo, &notched_slot(stretch), 0.0);
        let certified =
            certify_hole(&topo, hole, &section, Some(&surface), Tolerance::new()).unwrap();
        let crossings: Vec<Point3> = certified.crossings().map(|c| c.at).collect();
        let what = format!("stretch {stretch}: {crossings:?}");
        assert_eq!(crossings.len(), 2, "{what}");
        for wanted in [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new((1.0 - top * top).sqrt(), top, 0.0),
        ] {
            assert!(
                crossings.iter().any(|p| (*p - wanted).length() <= 1e-12),
                "{what}: no crossing at {wanted:?}"
            );
        }
        let touches: Vec<Point3> = certified.touches().collect();
        assert_eq!(touches.len(), 1, "{what}: {touches:?}");
        assert!((touches[0].y() - 0.5 * top).abs() <= 1e-9, "{touches:?}");

        let found = hole_hits(&topo, hole, &section, &surface);
        if stretch > 1.0 {
            // 8e-5 apart: two crossings, kept.
            let found = found.unwrap().unwrap();
            assert_eq!(found.crossings.len(), 2, "{what}");
        } else {
            // 8e-6 apart: closer than the separation distance, refused by
            // name. Never no crossing and a touch.
            let error = found.map(|_| ()).expect_err("crossings 8e-6 apart");
            assert!(
                error
                    .to_string()
                    .contains("the distance at which two crossings are one"),
                "{error}"
            );
        }
    }
}

/// Finding 1, through the trim: no kept span runs through the hole.
#[test]
fn a_span_through_a_hole_is_never_kept() {
    for stretch in [1.0, 10.0] {
        let pair = Pair::new(
            &[(-0.5, -2.0), (2.0, -2.0), (2.0, 2.0), (-0.5, 2.0)],
            |topo| vec![polygon_wire(topo, &notched_slot(stretch), 50.0)],
            (0.0, 0.0),
            1.0,
            1.0,
        );
        let on_circle = |y: f64| Point3::new((1.0 - y * y).sqrt(), y, 50.0);
        match pair.trim() {
            Ok(arcs) => {
                assert!(stretch > 1.0, "crossings 8e-6 apart were accepted");
                let arcs = arcs.expect("exact arcs");
                // Inside the hole either side of the notch's tip, and at
                // the tip itself: not kept.
                for k in [1.0, 2.0, 4.0, 6.0, 7.0] {
                    let p = on_circle(k * 1e-6 * stretch);
                    assert!(!covers(&arcs, p), "stretch {stretch}: kept through {p:?}");
                }
                // On the face below and above the slot: kept.
                for y in [-1e-3, 1e-3] {
                    assert!(
                        covers(&arcs, on_circle(y)),
                        "stretch {stretch}: dropped at {y}"
                    );
                }
            }
            Err(error) => {
                assert!(stretch <= 1.0, "stretch {stretch}: {error}");
                assert!(
                    error
                        .to_string()
                        .contains("the distance at which two crossings are one"),
                    "{error}"
                );
            }
        }
    }
}

/// Finding 2: a closed hole is never crossed an odd number of times.
///
/// The reviewer's quadrilateral: its side AB runs at y = 1 − 5e-8, within
/// the band under the top of the unit circle, and ends at B, 5e-9 inside
/// the circle, where the boundary turns away along BC. The circle enters
/// the hole across AB at x = −√(1 − (1 − 5e-8)²) ≈ −3.16e-4, a shallow
/// crossing whose partner on AB's line lies past B, and leaves it across
/// the diagonal CD. The first crossing used to be dropped as a tangency of
/// AB's line, leaving one crossing of a closed loop.
#[test]
fn a_closed_hole_is_never_crossed_an_odd_number_of_times() {
    let (section, surface) = upright(0.0, 0.0, 1.0);
    let corners = [
        (-0.1, 0.999_999_95),
        (-0.0003, 0.999_999_95),
        (0.0997, 0.899_999_95),
        (-0.1, 1.1),
    ];
    let mut topo = Topology::new();
    let hole = polygon_wire(&mut topo, &corners, 0.0);
    let found = hole_hits(&topo, hole, &section, &surface).unwrap().unwrap();
    assert_eq!(found.crossings.len(), 2, "{:?}", found.crossings);
    let y = 0.999_999_95_f64;
    let shallow = Point3::new(-(1.0 - y * y).sqrt(), y, 0.0);
    assert!(
        found
            .crossings
            .iter()
            .any(|p| (*p - shallow).length() <= 1e-10),
        "no crossing on AB at {shallow:?}: {:?}",
        found.crossings
    );
    // The other on CD, the line through C and D: x(t), y(t) on the circle.
    let on_cd = |p: &Point3| {
        let (c, d) = (corners[2], corners[3]);
        ((p.x() - c.0) * (d.1 - c.1) - (p.y() - c.1) * (d.0 - c.0)).abs() <= 1e-12
            && (p.x().hypot(p.y()) - 1.0).abs() <= 1e-12
    };
    assert!(found.crossings.iter().any(on_cd), "{:?}", found.crossings);

    // The family it came from: the side's height, the corner's position and
    // the next side's slope swept through the band. Every accepted answer
    // is even.
    let (mut accepted, mut refused) = (0, 0);
    for tilt in [-1.0, -0.1, -0.01, -0.001, 0.001, 0.01, 0.1, 1.0] {
        for d in [0.05e-7, 0.5e-7, 0.99e-7, 1.01e-7, 2e-7, 10e-7] {
            for vx in [-1e-3, -3e-4, -1e-4, 0.0, 1e-4, 3e-4, 1e-3] {
                let corners = [
                    (-0.1, 1.0 - d),
                    (vx, 1.0 - d),
                    (vx + 0.1, 1.0 - d + 0.1 * tilt),
                    (-0.1, 1.1),
                ];
                let mut topo = Topology::new();
                let hole = polygon_wire(&mut topo, &corners, 0.0);
                match hole_hits(&topo, hole, &section, &surface) {
                    Ok(found) => {
                        accepted += 1;
                        let found = found.unwrap();
                        assert_eq!(
                            found.crossings.len() % 2,
                            0,
                            "{corners:?}: {:?}",
                            found.crossings
                        );
                    }
                    Err(error) => {
                        refused += 1;
                        assert!(
                            error.to_string().contains("cannot be told from a tangency"),
                            "{corners:?}: {error}"
                        );
                    }
                }
            }
        }
    }
    assert_eq!(accepted + refused, 336);
    assert!(accepted > refused, "{accepted} accepted, {refused} refused");
}

/// Finding 3, on phase FF's side: two crossings of one hole edge 3e-6
/// apart are not handed to the face splitter, which welds what lies within
/// 1e-5 on that edge. A needle of semi-axes (0.005, 5e-6) pokes through the
/// side x = 0.002 of the square hole [−0.002, 0.002]² at y = ±1.5e-6.
/// Fifty tolerances wider (crossings 3.6e-5 apart), both are kept.
#[test]
fn crossings_the_face_splitter_would_weld_are_refused() {
    for (b, y, kept) in [(5e-6_f64, 1.5e-6_f64, false), (5e-5, 1.8e-5, true)] {
        let a = 0.005_f64;
        let cx = 0.002 - a * (1.0 - (y / b).powi(2)).sqrt();
        let cos = b / a;
        let axis = Vec3::new(-(1.0 - cos * cos).sqrt(), 0.0, cos);
        let centre = Point3::new(cx, 0.0, 0.0);
        let section = EdgeCurve::Ellipse(
            Ellipse3D::new_with_ref(centre, Z, a, b, Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        );
        let surface = FaceSurface::Cylinder(CylindricalSurface::new(centre, axis, b).unwrap());
        let mut topo = Topology::new();
        let hole = polygon_wire(&mut topo, &square(0.002, (0.0, 0.0)), 0.0);
        // The certificate places both, 2 y apart on the side x = 0.002.
        let certified =
            certify_hole(&topo, hole, &section, Some(&surface), Tolerance::new()).unwrap();
        let right: Vec<Point3> = certified
            .crossings()
            .map(|c| c.at)
            .filter(|p| (p.x() - 0.002).abs() < 1e-12)
            .collect();
        assert_eq!(right.len(), 2, "b = {b}");
        for p in &right {
            assert!((p.y().abs() - y).abs() < 1e-12, "b = {b}: {p:?}");
        }
        let found = hole_hits(&topo, hole, &section, &surface);
        if kept {
            assert!(found.unwrap().unwrap().crossings.len() >= 2, "b = {b}");
        } else {
            let error = found.map(|_| ()).expect_err("crossings 3e-6 apart");
            assert!(
                error.to_string().contains("cannot be told from a tangency"),
                "{error}"
            );
        }
    }
}

/// Finding 4: a length, not an angle, decides whether two crossings are
/// one. A circle of radius 5e6 crosses a slot 1e-3 wide at (3e6, 4e6): the
/// two crossings are 1.25e-3 apart and 2.5e-10 rad apart. The trim
/// deduplicated parameters at 1e-9 rad, dropped one, and kept a span across
/// the slot.
#[test]
fn crossings_a_millimetre_apart_on_a_large_circle_stay_apart() {
    let (x0, x1) = (3e6 - 0.0005, 3e6 + 0.0005);
    let pair = Pair::new(
        &square(10.0, (0.0, 0.0)),
        |topo| {
            vec![polygon_wire(
                topo,
                &[
                    (x0, 4e6 - 10.0),
                    (x1, 4e6 - 10.0),
                    (x1, 4e6 + 10.0),
                    (x0, 4e6 + 10.0),
                ],
                50e6,
            )]
        },
        (0.0, 0.0),
        5.0,
        1e6,
    );
    let hole = pair.topo.face(pair.plane).unwrap().inner_wires()[0];
    let found = hole_hits(&pair.topo, hole, &pair.raw.curve, &pair.surfaces.1)
        .unwrap()
        .unwrap();
    // x = x0, x1 on the circle x² + y² = (5e6)².
    let on_circle = |x: f64| Point3::new(x, (25e12 - x * x).sqrt(), 50e6);
    assert_eq!(found.crossings.len(), 2, "{:?}", found.crossings);
    for wanted in [on_circle(x0), on_circle(x1)] {
        assert!(
            found
                .crossings
                .iter()
                .any(|p| (*p - wanted).length() <= 1e-8),
            "no crossing at {wanted:?}: {:?}",
            found.crossings
        );
    }
    let arcs = pair.trim().unwrap().expect("exact arcs");
    for p in &found.crossings {
        assert!(
            arcs.iter()
                .any(|a| (a.p_start - *p).length() < 1e-8 || (a.p_end - *p).length() < 1e-8),
            "no span ends at {p:?}"
        );
    }
    // Nothing kept across the slot, and the circle kept either side of it.
    assert!(!covers(&arcs, on_circle(3e6)));
    for x in [3e6 - 1.0, 3e6 + 1.0] {
        assert!(covers(&arcs, on_circle(x)), "dropped at x = {x}");
    }
}

/// The same four lengths on an eccentric section: an ellipse of semi-axes
/// (1000, 1) crossed twice 1e-3 apart on its flat side, where one radian of
/// its parameter is a thousand units long, and twice 1e-3 apart round its
/// end, where it is one. Both pairs stay apart; a parameter tolerance
/// would treat them a thousandfold differently.
#[test]
fn crossings_on_an_eccentric_section_are_told_apart_by_length() {
    let (a, b) = (1000.0, 1.0);
    let (ellipse, cylinder) = tilted_section(a, b, 0.0);
    let section = EdgeCurve::Ellipse(ellipse);
    // A slot 1e-3 wide across the flat side at x = 0, and one across the
    // end at y = 0.
    for corners in [
        [(-5e-4, 0.5), (5e-4, 0.5), (5e-4, 1.5), (-5e-4, 1.5)],
        [
            (999.0, -5e-4),
            (1001.0, -5e-4),
            (1001.0, 5e-4),
            (999.0, 5e-4),
        ],
    ] {
        let mut topo = Topology::new();
        let hole = polygon_wire(&mut topo, &corners, 0.0);
        let found = hole_hits(&topo, hole, &section, &cylinder)
            .unwrap_or_else(|e| panic!("{corners:?}: {e}"))
            .unwrap();
        assert_eq!(
            found.crossings.len(),
            2,
            "{corners:?}: {:?}",
            found.crossings
        );
        let apart = (found.crossings[0] - found.crossings[1]).length();
        assert!((apart - 1e-3).abs() < 1e-6, "{corners:?}: {apart}");
    }
}

/// A corner of a hole cut by a short chamfer the section crosses, and holes
/// the section does not cross at all.
///
/// The unit circle enters the hole `[0.8, 2] × [−1, 0.6]` at (0.8, −0.6)
/// and leaves it across the chamfer that cuts its corner at (0.8, 0.6),
/// where the chamfer's line `x − y = 0.2 + c` meets the circle. Both
/// crossings are found where they are, however short the chamfer. With the
/// chamfer 4.2e-6 long the second lies 2.1e-6 from the chamfer's ends:
/// closer to two vertices of the face than the face's arrangement keeps a
/// section end apart from a vertex, so a span kept beside it is refused by
/// name. Ten times longer, both are kept.
///
/// A hole wholly inside the section and one that holds it whole are crossed
/// nowhere: no crossing, no touch, and nothing to refuse.
#[test]
fn a_short_chamfer_is_crossed_where_it_is_and_whole_holes_are_left_alone() {
    let (section, surface) = upright(0.0, 0.0, 1.0);
    for (c, kept) in [(3e-6_f64, false), (3e-5, true)] {
        let mut topo = Topology::new();
        let hole = polygon_wire(
            &mut topo,
            &[
                (0.8, 0.6 - c),
                (0.8 + c, 0.6),
                (2.0, 0.6),
                (2.0, -1.0),
                (0.8, -1.0),
            ],
            0.0,
        );
        let certified =
            certify_hole(&topo, hole, &section, Some(&surface), Tolerance::new()).unwrap();
        let crossings: Vec<Point3> = certified.crossings().map(|x| x.at).collect();
        assert_eq!(crossings.len(), 2, "c = {c}: {crossings:?}");
        // On the chamfer: x − y = 0.2 + c and x² + y² = 1.
        let k = 0.2 + c;
        let y = 0.5 * (-k + (2.0 - k * k).sqrt());
        for wanted in [Point3::new(0.8, -0.6, 0.0), Point3::new(y + k, y, 0.0)] {
            assert!(
                crossings.iter().any(|p| (*p - wanted).length() <= 1e-12),
                "c = {c}: no crossing at {wanted:?}: {crossings:?}"
            );
        }
        let found = hole_hits(&topo, hole, &section, &surface);
        if kept {
            assert_eq!(found.unwrap().unwrap().crossings.len(), 2, "c = {c}");
        } else {
            let error = found
                .map(|_| ())
                .expect_err("a crossing 2e-6 from two vertices");
            assert!(
                error.to_string().contains("is a vertex of the face"),
                "{error}"
            );
        }
    }
    for corners in [square(0.1, (0.0, 0.0)), square(2.0, (0.0, 0.0))] {
        let mut topo = Topology::new();
        let hole = polygon_wire(&mut topo, &corners, 0.0);
        let found = hole_hits(&topo, hole, &section, &surface).unwrap().unwrap();
        assert!(
            found.crossings.is_empty() && found.touches.is_empty(),
            "{corners:?}: {:?} {:?}",
            found.crossings,
            found.touches
        );
    }
}
