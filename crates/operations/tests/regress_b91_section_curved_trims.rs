//! B91: `section` must close periodic loops and keep only the part of a
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
use remus_operations::measure::face_area;
use remus_operations::primitives::{make_box, make_cone, make_torus};
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
fn b91_section_keeps_a_span_narrower_than_a_chord() {
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
fn b91_plane_below_a_dome_misses_it() {
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
fn b91_dome_sections_follow_its_trim() {
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
fn b91_frustum_sections_close() {
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
fn b91_notched_torus_drops_the_removed_tube() {
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
