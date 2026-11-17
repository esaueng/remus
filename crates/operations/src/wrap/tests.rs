#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use super::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Axis Z through the origin, angle zero along +X, wrap radius 10: a
/// profile on the tangent plane x = 10 has arc length y and height z.
fn cylinder() -> WrapCylinder {
    WrapCylinder {
        origin: Point3::new(0.0, 0.0, 0.0),
        axis: Z,
        reference: X,
        radius: 10.0,
    }
}

fn plane_face(topo: &mut Topology, wire: WireId, holes: Vec<WireId>) -> FaceId {
    topo.add_face(Face::new(
        wire,
        holes,
        FaceSurface::Plane { normal: X, d: 10.0 },
    ))
}

/// A polygon on x = 10 from `(y, z)` corners.
fn polygon(topo: &mut Topology, yz: &[(f64, f64)]) -> FaceId {
    let points: Vec<Point3> = yz.iter().map(|&(y, z)| Point3::new(10.0, y, z)).collect();
    let wire = make_polygon_wire(topo, &points, 1e-7).unwrap();
    plane_face(topo, wire, vec![])
}

fn circle_wire(topo: &mut Topology, y: f64, z: f64, r: f64, forward: bool) -> WireId {
    let edge = make_circle_edge(topo, Point3::new(10.0, y, z), X, r, 1e-7).unwrap();
    topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, forward)], true).unwrap())
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {report:?}");
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::mass_properties(topo, solid).unwrap().mass
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

/// Every wrap of a region of area `A` between radii `ri` and `ro` about
/// wrap radius `R` encloses `A · (ro² − ri²) / (2R)`.
fn exact(area: f64, inner: f64, outer: f64) -> f64 {
    area * (outer * outer - inner * inner) / 20.0
}

fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    let mut c = (0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => c.0 += 1,
            FaceSurface::Cylinder(_) => c.1 += 1,
            FaceSurface::Nurbs(_) => c.2 += 1,
            other => panic!("unexpected {}", other.type_tag()),
        }
    }
    c
}

#[test]
fn a_square_wraps_to_planes_and_cylinders_exactly() {
    let mut topo = Topology::new();
    let profile = polygon(
        &mut topo,
        &[(-5.0, 0.0), (5.0, 0.0), (5.0, 10.0), (-5.0, 10.0)],
    );
    let solid = wrap_profile_solid(&mut topo, profile, &cylinder(), 9.0, 11.0).unwrap();
    assert_valid(&topo, solid, "square");
    assert_eq!(census(&topo, solid), (4, 2, 0));
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact(100.0, 9.0, 11.0)) < 1e-12, "{measured}");
}

#[test]
fn an_oblique_parallelogram_wraps_to_certified_helical_walls() {
    let mut topo = Topology::new();
    let profile = polygon(
        &mut topo,
        &[(-5.0, 0.0), (5.0, 0.0), (9.0, 10.0), (-1.0, 10.0)],
    );
    let solid = wrap_profile_solid(&mut topo, profile, &cylinder(), 9.0, 11.0).unwrap();
    assert_valid(&topo, solid, "parallelogram");
    assert_eq!(census(&topo, solid), (2, 2, 2));
    let measured = volume(&topo, solid);
    assert!(rel(measured, exact(100.0, 9.0, 11.0)) < 1e-7, "{measured}");
}

#[test]
fn a_disc_and_an_annulus_wrap_with_fitted_curves() {
    let mut topo = Topology::new();
    let outer = circle_wire(&mut topo, 0.0, 5.0, 4.0, true);
    let disc = plane_face(&mut topo, outer, vec![]);
    let solid = wrap_profile_solid(&mut topo, disc, &cylinder(), 9.5, 11.0).unwrap();
    assert_valid(&topo, solid, "disc");
    let measured = volume(&topo, solid);
    assert!(
        rel(measured, exact(16.0 * PI, 9.5, 11.0)) < 1e-7,
        "{measured}"
    );

    let mut topo = Topology::new();
    let outer = circle_wire(&mut topo, 0.0, 5.0, 4.0, true);
    let hole = circle_wire(&mut topo, 0.0, 5.0, 2.0, false);
    let ring = plane_face(&mut topo, outer, vec![hole]);
    let solid = wrap_profile_solid(&mut topo, ring, &cylinder(), 9.5, 11.0).unwrap();
    assert_valid(&topo, solid, "annulus");
    let measured = volume(&topo, solid);
    assert!(
        rel(measured, exact(12.0 * PI, 9.5, 11.0)) < 1e-7,
        "{measured}"
    );
}

#[test]
fn wrapped_solids_tessellate_watertight() {
    let mut topo = Topology::new();
    let outer = circle_wire(&mut topo, 0.0, 5.0, 4.0, true);
    let disc = plane_face(&mut topo, outer, vec![]);
    let solid = wrap_profile_solid(&mut topo, disc, &cylinder(), 9.5, 11.0).unwrap();
    let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
    let mut uses = std::collections::HashMap::<(u32, u32), i32>::new();
    for t in mesh.indices.chunks(3) {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    assert_eq!(uses.values().filter(|&&c| c == 1).count(), 0);
}

#[test]
fn refusals_are_typed_and_leave_the_topology_unchanged() {
    // Not parallel to the axis.
    let mut topo = Topology::new();
    let points = [
        Point3::new(10.0, -1.0, 0.0),
        Point3::new(11.0, 1.0, 0.0),
        Point3::new(11.0, 1.0, 2.0),
        Point3::new(10.0, -1.0, 2.0),
    ];
    let wire = make_polygon_wire(&mut topo, &points, 1e-7).unwrap();
    let n = crate::winding::newell_normal(&points).normalize().unwrap();
    let tilted = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: n,
            d: crate::dot_normal_point(n, points[0]),
        },
    ));
    // A flat square in a plane holding the axis direction is fine; this one
    // is tilted about Z, so it is still parallel to the axis: build it.
    assert!(wrap_profile_solid(&mut topo, tilted, &cylinder(), 9.0, 11.0).is_ok());

    let mut topo = Topology::new();
    let points = [
        Point3::new(10.0, -1.0, 0.0),
        Point3::new(10.0, 1.0, 0.0),
        Point3::new(11.0, 1.0, 2.0),
        Point3::new(11.0, -1.0, 2.0),
    ];
    let wire = make_polygon_wire(&mut topo, &points, 1e-7).unwrap();
    let n = crate::winding::newell_normal(&points).normalize().unwrap();
    let leaning = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: n,
            d: crate::dot_normal_point(n, points[0]),
        },
    ));
    let faces = topo.num_faces();
    let refused = wrap_profile_solid(&mut topo, leaning, &cylinder(), 9.0, 11.0);
    assert!(
        matches!(&refused, Err(OperationsError::InvalidInput { reason }) if reason.contains("parallel")),
        "{refused:?}"
    );
    assert_eq!(topo.num_faces(), faces);

    // More than a turn round.
    let mut topo = Topology::new();
    let wide = polygon(
        &mut topo,
        &[(-40.0, 0.0), (40.0, 0.0), (40.0, 1.0), (-40.0, 1.0)],
    );
    let refused = wrap_profile_solid(&mut topo, wide, &cylinder(), 9.0, 11.0);
    assert!(
        matches!(&refused, Err(OperationsError::InvalidInput { reason }) if reason.contains("one turn")),
        "{refused:?}"
    );

    // An empty radius range.
    let mut topo = Topology::new();
    let square = polygon(&mut topo, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
    let refused = wrap_profile_solid(&mut topo, square, &cylinder(), 11.0, 11.0);
    assert!(matches!(refused, Err(OperationsError::InvalidInput { .. })));
    assert_eq!(topo.num_faces(), 1);
}
