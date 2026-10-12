//! Differential oracle for the hole-containment box gate.
//!
//! `is_inside_any_hole` skips sampling a hole whose sample box the query
//! point lies outside. The function below is the test as it was before the
//! gate, kept verbatim, and the test compares the two on round, elliptical,
//! half-round and straight holes with real fitted pcurves, at query points
//! spread over the plane and packed against every rim.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::curves2d::Curve2D;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

use super::super::super::pcurve_compute::compute_pcurve_on_surface_in_domain;
use super::super::super::plane_frame::PlaneFrame;
use super::super::super::split_types::OrientedPCurveEdge;
use super::super::sampling::sample_wire_loop_uv;
use super::{hole_chord_polygon, is_inside_any_hole};

/// `is_inside_any_hole` before the box gate, verbatim.
fn is_inside_any_hole_before_gate(pt: &Point2, inner_wires: &[Vec<OrientedPCurveEdge>]) -> bool {
    for hole in inner_wires {
        // Prefer the chord polygon (endpoint-derived). Fall back to the sampled
        // polygon only when the hole degenerates to < 3 endpoints.
        let chord = hole_chord_polygon(hole);
        if chord.len() >= 3 {
            if super::super::super::classify_2d::point_in_polygon_2d(*pt, &chord) {
                return true;
            }
            continue;
        }
        let hole_pts = sample_wire_loop_uv(hole);
        if hole_pts.len() >= 3
            && super::super::super::classify_2d::point_in_polygon_2d(*pt, &hole_pts)
        {
            return true;
        }
    }
    false
}

struct Plane {
    surface: FaceSurface,
    frame: PlaneFrame,
    normal: Vec3,
}

fn plane() -> Plane {
    let normal = Vec3::new(0.2, -0.3, 0.93).normalize().unwrap();
    let origin = Point3::new(1.0, 2.0, 3.0);
    Plane {
        surface: FaceSurface::Plane {
            normal,
            d: normal.dot(origin - Point3::new(0.0, 0.0, 0.0)),
        },
        frame: PlaneFrame::from_normal_and_point(normal, origin),
        normal,
    }
}

/// One wire use of `curve` from `start` to `end` over `domain`, with the
/// pcurve the splitter would fit.
fn edge(
    plane: &Plane,
    curve: EdgeCurve,
    (start, end): (Point3, Point3),
    domain: (f64, f64),
    forward: bool,
) -> OrientedPCurveEdge {
    let pcurve = compute_pcurve_on_surface_in_domain(
        &curve,
        start,
        end,
        domain,
        &plane.surface,
        &[],
        Some(&plane.frame),
    )
    .unwrap();
    OrientedPCurveEdge {
        curve_3d: curve,
        trim: Some(domain),
        pcurve,
        start_uv: plane.frame.project(start),
        end_uv: plane.frame.project(end),
        start_3d: start,
        end_3d: end,
        forward,
        source_edge_idx: None,
        pave_block_id: None,
        source_topo_edge: None,
    }
}

fn holes(plane: &Plane) -> Vec<Vec<OrientedPCurveEdge>> {
    let at = |u: f64, v: f64| plane.frame.evaluate(u, v);
    let mut holes = Vec::new();
    // Full circles both ways round, at several radii.
    for (k, radius) in [2.0, 0.75, 3.25].into_iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let circle = Circle3D::new(at(6.0 * k as f64, 0.0), plane.normal, radius).unwrap();
        let p = circle.evaluate(0.0);
        holes.push(vec![edge(
            plane,
            EdgeCurve::Circle(circle.clone()),
            (p, p),
            (0.0, std::f64::consts::TAU),
            k % 2 == 0,
        )]);
    }
    // A full ellipse.
    let ellipse = Ellipse3D::new(at(0.0, 9.0), plane.normal, 2.5, 1.0).unwrap();
    let p = ellipse.evaluate(0.0);
    holes.push(vec![edge(
        plane,
        EdgeCurve::Ellipse(ellipse),
        (p, p),
        (0.0, std::f64::consts::TAU),
        false,
    )]);
    // A half-round: one arc and its chord (two endpoints, so sampled).
    let circle = Circle3D::new(at(10.0, 9.0), plane.normal, 1.5).unwrap();
    let (a, b) = (circle.evaluate(0.0), circle.evaluate(std::f64::consts::PI));
    holes.push(vec![
        edge(
            plane,
            EdgeCurve::Circle(circle),
            (a, b),
            (0.0, std::f64::consts::PI),
            true,
        ),
        edge(plane, EdgeCurve::Line, (b, a), (0.0, 1.0), true),
    ]);
    // A straight triangle (chord polygon, never sampled).
    let corners = [at(15.0, 0.0), at(19.0, 1.0), at(16.0, 4.0)];
    holes.push(
        (0..3)
            .map(|i| {
                edge(
                    plane,
                    EdgeCurve::Line,
                    (corners[i], corners[(i + 1) % 3]),
                    (0.0, 1.0),
                    true,
                )
            })
            .collect(),
    );
    holes
}

fn queries(holes: &[Vec<OrientedPCurveEdge>]) -> Vec<Point2> {
    let mut points = Vec::new();
    for i in 0..=120 {
        for j in 0..=60 {
            points.push(Point2::new(
                -5.0 + f64::from(i) * 0.2,
                -5.0 + f64::from(j) * 0.3,
            ));
        }
    }
    // Packed against every sample and endpoint, inside and out.
    for hole in holes {
        let mut rim = sample_wire_loop_uv(hole);
        for edge in hole {
            rim.push(edge.start_uv);
            if let Curve2D::Nurbs(nurbs) = &edge.pcurve {
                rim.extend(nurbs.control_points().iter().copied());
            }
        }
        for p in rim {
            for (dx, dy) in [
                (0.0, 0.0),
                (1e-12, 0.0),
                (-1e-9, 2e-9),
                (3e-6, -1e-6),
                (-2e-3, 0.0),
                (0.0, 5e-2),
            ] {
                points.push(Point2::new(p.x() + dx, p.y() + dy));
            }
        }
    }
    points
}

#[test]
fn gated_containment_matches_the_ungated_oracle() {
    let plane = plane();
    let holes = holes(&plane);
    let mut inside = 0;
    let points = queries(&holes);
    for point in &points {
        let gated = is_inside_any_hole(point, &holes);
        assert_eq!(
            gated,
            is_inside_any_hole_before_gate(point, &holes),
            "containment of {point:?} differs from the ungated test"
        );
        inside += usize::from(gated);
        for hole in &holes {
            let single = std::slice::from_ref(hole);
            assert_eq!(
                is_inside_any_hole(point, single),
                is_inside_any_hole_before_gate(point, single),
                "containment of {point:?} in one hole differs"
            );
        }
    }
    assert!(points.len() > 8_000, "only {} queries", points.len());
    assert!(inside > 300, "only {inside} queries inside a hole");
}
