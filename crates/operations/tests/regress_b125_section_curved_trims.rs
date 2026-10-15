//! B125: `section` must close periodic loops and keep only the part of a
//! curved face's carrier section that lies on the face.
//!
//! The sphere, cone and torus arms sectioned the untrimmed carriers, so a
//! patch contributed its whole carrier's section: a spherical cap sitting
//! above `z = 0.5` yielded the full unit circle at `z = 0` (a phantom disc of
//! area π where the plane misses the body), and a notched torus kept the
//! tube loop of the side that was cut away. The cone arm never closed its loop,
//! so a frustum's section did not assemble at all.
//!
//! Oracles are closed-form areas: a disc, an ellipse (the oblique cone cut, a
//! focal conic `r = ℓ / (1 − e cos θ)` in projection), and a circular segment.
//! The section polylines keep the 64-chord sampling of the carrier circle, so
//! the areas carry its chord deficit, at most `(2π²/3)/64² ≈ 1.6e-3` relative;
//! the bound below is twice that. `classify_point` probes check that a
//! section region lies in the material and that a missed plane really misses.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::{make_box, make_cone, make_torus};
use remus_operations::revolve::revolve;
use remus_operations::section::section;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// Twice the 64-chord area deficit of the sampled section loops.
const CHORD_BOUND: f64 = 3.3e-3;

fn section_area(topo: &mut Topology, solid: SolidId, point: Point3, normal: Vec3) -> f64 {
    let faces = section(topo, solid, point, normal).unwrap().faces;
    faces
        .iter()
        .map(|&f| face_area(topo, f, 1e-3).unwrap())
        .sum()
}

fn assert_area(area: f64, expected: f64, what: &str) {
    let rel = (area - expected).abs() / expected;
    assert!(
        rel <= CHORD_BOUND,
        "{what}: section area {area}, closed form {expected} ({rel:e})"
    );
}

/// The unit sphere's cap above `z = 0.5`: a sphere face and a planar disc
/// sharing one closed rim circle.
fn dome(topo: &mut Topology) -> SolidId {
    let cut = 0.5_f64;
    let up = Vec3::new(0.0, 0.0, 1.0);
    let rim = Circle3D::new(Point3::new(0.0, 0.0, cut), up, (1.0 - cut * cut).sqrt()).unwrap();
    let vertex = topo.add_vertex(Vertex::new(rim.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(rim));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    // The cap runs the rim counter-clockwise about +z, so it keeps the pole;
    // the base disc faces down and runs it the other way.
    let cap_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let base_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, false)], true).unwrap());
    let cap = topo.add_face(Face::new(
        cap_wire,
        vec![],
        FaceSurface::Sphere(SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap()),
    ));
    let base = topo.add_face(Face::new(
        base_wire,
        vec![],
        FaceSurface::Plane {
            normal: -up,
            d: -cut,
        },
    ));
    let shell = topo.add_shell(Shell::new(vec![cap, base]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    assert!(validate_solid(topo, solid).unwrap().is_valid());
    solid
}

/// The unit sphere between `z = 0.5` and `z = top`, closed by two planar
/// discs: a sphere band between two rims.
fn truncated_dome(topo: &mut Topology, top: f64) -> SolidId {
    let up = Vec3::new(0.0, 0.0, 1.0);
    let rim = |topo: &mut Topology, z: f64| {
        let circle = Circle3D::new(Point3::new(0.0, 0.0, z), up, (1.0 - z * z).sqrt()).unwrap();
        let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        topo.add_edge(edge)
    };
    let (low, high) = (rim(topo, 0.5), rim(topo, top));
    let wire = |topo: &mut Topology, edge, forward| {
        topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
    };
    // The band keeps the region between its rims: the low rim counter-clockwise
    // about +z (pole side kept), the high rim clockwise (pole side removed).
    let band_outer = wire(topo, low, true);
    let band_hole = wire(topo, high, false);
    let band = topo.add_face(Face::new(
        band_outer,
        vec![band_hole],
        FaceSurface::Sphere(SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap()),
    ));
    let base_wire = wire(topo, low, false);
    let base = topo.add_face(Face::new(
        base_wire,
        vec![],
        FaceSurface::Plane {
            normal: -up,
            d: -0.5,
        },
    ));
    let lid_wire = wire(topo, high, true);
    let lid = topo.add_face(Face::new(
        lid_wire,
        vec![],
        FaceSurface::Plane { normal: up, d: top },
    ));
    let shell = topo.add_shell(Shell::new(vec![band, base, lid]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    assert!(validate_solid(topo, solid).unwrap().is_valid());
    solid
}

/// A face narrower than one of the 64 carrier chords: the unit sphere's band
/// between `z = 0.5` and `z = 0.52`, lidded, spans 0.023 rad of the section
/// circle against a 0.098 rad chord. Classifying only the chord ends drops
/// that span whenever both ends land off the band, leaving the two lid
/// chords unjoined.
#[test]
fn b125_section_keeps_a_span_narrower_than_a_chord() {
    let top = 0.52_f64;
    let mut topo = Topology::new();
    let solid = truncated_dome(&mut topo, top);
    let probe = |p: Point3| classify_point(&topo, solid, p, 0.01, 1e-7).unwrap();
    assert_eq!(
        probe(Point3::new(0.0, 0.0, 0.51)),
        PointClassification::Inside
    );
    assert_eq!(
        probe(Point3::new(0.0, 0.0, 0.6)),
        PointClassification::Outside
    );
    // Through the axis: the unit circle's strip 0.5 ≤ z ≤ top, the difference
    // of the segments above each, `(θ − sin θ)/2` with θ = 2·acos(z).
    let segment = |z: f64| {
        let angle = 2.0 * z.acos();
        0.5 * (angle - angle.sin())
    };
    for normal in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.6, 0.8, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-0.28, 0.96, 0.0),
    ] {
        // The strip ends on the rims' sampled 32-gons (the planar arm's
        // `wire_polygon`), so it can fall short of the circle by their
        // relative sagitta, 1 − cos(π/32) ≈ 4.8e-3, on top of the arcs' own
        // 64-chord deficit.
        let area = section_area(&mut topo, solid, Point3::new(0.0, 0.0, 0.0), normal);
        let expected = segment(0.5) - segment(top);
        let rel = (area - expected).abs() / expected;
        assert!(
            rel <= 1.0 - (PI / 32.0).cos() + 0.5 * CHORD_BOUND,
            "{normal:?}: section area {area}, closed form {expected} ({rel:e})"
        );
    }
}

/// The review's case: `z = 0` misses the dome, so the section is empty.
#[test]
fn b125_plane_below_a_dome_misses_it() {
    let mut topo = Topology::new();
    let solid = dome(&mut topo);
    let probe = |p: Point3| classify_point(&topo, solid, p, 0.01, 1e-7).unwrap();
    assert_eq!(
        probe(Point3::new(0.0, 0.0, 0.0)),
        PointClassification::Outside
    );
    assert_eq!(
        probe(Point3::new(0.0, 0.0, 0.75)),
        PointClassification::Inside
    );

    let faces = section(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap()
    .faces;
    assert!(faces.is_empty(), "phantom section of {} faces", faces.len());
}

/// Planes through the dome: a horizontal disc inside the cap, and a vertical
/// cut whose arc is clipped where it meets the base rim.
#[test]
fn b125_dome_sections_follow_its_trim() {
    let mut topo = Topology::new();
    let solid = dome(&mut topo);
    // z = 0.75: a disc of radius √(1 − 0.75²).
    let area = section_area(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 0.75),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert_area(area, PI * (1.0 - 0.75 * 0.75), "z = 0.75");
    // x = 0: the unit circle's segment above its chord at z = 0.5, with
    // central angle 2·acos(0.5).
    let angle = 2.0 * 0.5_f64.acos();
    let area = section_area(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    assert_area(area, 0.5 * (angle - angle.sin()), "x = 0");
}

/// A frustum's section closes: a circle across it, an ellipse obliquely.
#[test]
fn b125_frustum_sections_close() {
    let mut topo = Topology::new();
    // Radius 2 at z = 0 down to 1 at z = 3: ρ(z) = 2 − z/3.
    let solid = make_cone(&mut topo, 2.0, 1.0, 3.0).unwrap();
    let area = section_area(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 1.5),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert_area(area, PI * 1.5 * 1.5, "z = 1.5");

    // z = 1.5 − 0.2x: in projection ρ ≤ 1.5 + x/15, the conic with
    // semi-latus rectum 1.5 and eccentricity 1/15 about the axis, of area
    // π ℓ² / (1 − e²)^{3/2}; the plane tilts it by √1.04. It stays inside
    // the frustum (z from 1.18 to 1.78).
    let (latus, e) = (1.5_f64, 1.0_f64 / 15.0);
    let expected = PI * latus * latus / (1.0 - e * e).powf(1.5) * 1.04_f64.sqrt();
    let area = section_area(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 1.5),
        Vec3::new(0.2, 0.0, 1.0),
    );
    assert_area(area, expected, "oblique");
}

/// A torus with a box notch through one side of its tube keeps only the
/// other side's cross-section (the B70 body, an exact one-torus-face cut).
#[test]
fn b125_notched_torus_drops_the_removed_tube() {
    let mut topo = Topology::new();
    let ring = make_torus(&mut topo, 10.0, 3.0, 16).unwrap();
    let notch = make_box(&mut topo, 8.0, 8.0, 8.0).unwrap();
    transform_solid(&mut topo, notch, &Mat4::translation(6.0, -4.0, -4.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, ring, notch).unwrap();
    let probe = |p: Point3| classify_point(&topo, solid, p, 0.01, 1e-7).unwrap();
    assert_eq!(
        probe(Point3::new(10.0, 0.0, 0.0)),
        PointClassification::Outside
    );
    assert_eq!(
        probe(Point3::new(-10.0, 0.0, 0.0)),
        PointClassification::Inside
    );

    // y = 0 crosses the tube at x = ±10; the x = 10 crossing is notched away.
    let area = section_area(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    assert_area(area, 9.0 * PI, "y = 0");
}

/// A torus band between the parallels `v = ±π/3` of major radius `major` and
/// `r = 1`, closed by an inner cylinder of radius `major − 2` and two annular
/// caps: the revolution of a profile whose outer side is the tube arc
/// through `(major + 1, 0)`.
fn torus_band(topo: &mut Topology, major: f64) -> SolidId {
    let h = (PI / 3.0).sin();
    let corner = |x: f64, z: f64| Point3::new(x, 0.0, z);
    let [a, b, c, d] = [
        corner(major - 2.0, -h),
        corner(major + 0.5, -h),
        corner(major + 0.5, h),
        corner(major - 2.0, h),
    ]
    .map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
    // Normal −y puts the circle's v-axis on +z, so B → C runs through
    // (major + 1, 0).
    let tube = Circle3D::new_with_ref(
        Point3::new(major, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        1.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let mut line = |s, e| topo.add_edge(Edge::new(s, e, EdgeCurve::Line));
    let (bottom, top, axis_side) = (line(a, b), line(c, d), line(d, a));
    let arc = topo.add_edge(Edge::new(b, c, EdgeCurve::Circle(tube)));
    let wire = topo.add_wire(
        Wire::new(
            [bottom, arc, top, axis_side]
                .into_iter()
                .map(|e| OrientedEdge::new(e, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let profile = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, -1.0, 0.0),
            d: 0.0,
        },
    ));
    revolve(
        topo,
        profile,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        TAU,
    )
    .unwrap()
}

/// A plane flush with one rim of the torus band: the rim is the section's
/// boundary there, and the carrier's other loop at that height (radius 2.5,
/// on the far side of the tube) is not on the band.
#[test]
fn b125_section_flush_with_one_rim_keeps_clipping() {
    let mut topo = Topology::new();
    let solid = torus_band(&mut topo, 3.0);
    let h = (PI / 3.0).sin();
    // Pappus: the rectangle 1 ≤ x ≤ 3.5, |z| ≤ h, plus the tube's segment
    // beyond its chord at x = 3.5 (angle 2π/3, centroid at
    // 3 + 4 sin³(π/3) / (3 (2π/3 − sin(2π/3)))).
    let angle = 2.0 * PI / 3.0;
    let segment = 0.5 * (angle - angle.sin());
    let centroid = 3.0 + 4.0 * (angle / 2.0).sin().powi(3) / (3.0 * (angle - angle.sin()));
    let expected = TAU * (2.5 * 2.0 * h * 2.25 + segment * centroid);
    let volume = solid_volume(&topo, solid, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "{volume} vs {expected}"
    );
    let probe = |p: Point3| classify_point(&topo, solid, p, 0.01, 1e-7).unwrap();
    assert_eq!(
        probe(Point3::new(2.5, 0.0, h - 0.1)),
        PointClassification::Inside
    );
    assert_eq!(
        probe(Point3::new(3.8, 0.0, 0.0)),
        PointClassification::Inside
    );

    // z = h: the top cap, the annulus 1 ≤ ρ ≤ 3.5.
    let area = section_area(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, h),
        Vec3::new(0.0, 0.0, 1.0),
    );
    assert_area(area, PI * (3.5 * 3.5 - 1.0), "z = h");
}

/// A vertical plane nearly tangent to the dome's closed rim (radius √0.75 at
/// z = 0.5), at distance c = 0.865 from the axis and turned π/32, midway
/// between two of the rim's 32 polygon samples. It crosses the rim twice
/// within that one sample interval, so no polygon vertex changed side: the
/// cap's narrow section was dropped. The section is the sphere's circle of
/// radius √(1 − c²) above its chord at z = 0.5, and that chord must end
/// exactly where the rim crosses the plane.
#[test]
fn b125_section_crosses_a_closed_rim_between_its_samples() {
    let mut topo = Topology::new();
    let solid = dome(&mut topo);
    let (c, turn) = (0.865_f64, PI / 32.0);
    let normal = Vec3::new(turn.cos(), turn.sin(), 0.0);
    let probe = |p: Point3| classify_point(&topo, solid, p, 1e-4, 1e-7).unwrap();
    assert_eq!(
        probe(Point3::new(0.0, 0.0, 0.5) + normal * c + Vec3::new(0.0, 0.0, 1e-3)),
        PointClassification::Inside
    );

    let faces = section(
        &mut topo,
        solid,
        Point3::new(0.0, 0.0, 0.0) + normal * c,
        normal,
    )
    .unwrap()
    .faces;
    assert_eq!(faces.len(), 1, "one narrow section");
    // Its base runs along z = 0.5 between the rim's two exact crossings.
    let half_chord = (0.75 - c * c).sqrt();
    let face = topo.face(faces[0]).unwrap();
    let mut base: Vec<Point3> = Vec::new();
    for oriented in topo.wire(face.outer_wire()).unwrap().edges() {
        let edge = topo.edge(oriented.edge()).unwrap();
        for vertex in [edge.start(), edge.end()] {
            let p = topo.vertex(vertex).unwrap().point();
            if (p.z() - 0.5).abs() < 1e-9 {
                base.push(p);
            }
        }
    }
    let along = |p: Point3| p.y() * turn.cos() - p.x() * turn.sin();
    let extent = base.iter().map(|&p| along(p).abs()).fold(0.0_f64, f64::max);
    assert!(
        (extent - half_chord).abs() < 1e-9,
        "base reaches {extent}, the rim crosses at ±{half_chord}"
    );
    // The circular segment, against its 64-chord sampling: its arc spans
    // under two chords, so only a lower bound is meaningful.
    let r = (1.0 - c * c).sqrt();
    let segment = r * r * (0.5 / r).acos() - 0.5 * (r * r - 0.25).sqrt();
    let area = face_area(&topo, faces[0], 1e-3).unwrap();
    assert!(
        area > 0.0 && area <= segment * (1.0 + 1e-9),
        "{area} vs {segment}"
    );
}
