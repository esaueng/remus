//! A cylinder wall bounded by an ellipse (a plane section that is not normal
//! to the axis) integrates exactly, so its volume no longer depends on which
//! vertex angles a fallback happens to see (#1740).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::mat::Mat4;

/// Radius and height of the uncut cylinder; the cut plane passes through the
/// axis at `PIVOT` and leans `TILT` off horizontal.
const RADIUS: f64 = 2.0;
const HEIGHT: f64 = 10.0;
const PIVOT: f64 = 6.0;
const TILT: f64 = 0.35;

/// The cylinder with everything above the oblique plane cut away, and that
/// plane as `(normal, d)` with `normal · p = d`, the normal pointing out of the
/// kept material.
fn obliquely_cut_cylinder(topo: &mut Topology) -> (SolidId, Vec3, f64) {
    let cylinder = crate::primitives::make_cylinder(topo, RADIUS, HEIGHT).unwrap();
    let cutter = crate::primitives::make_box(topo, 40.0, 40.0, 40.0).unwrap();
    crate::transform::transform_solid(topo, cutter, &Mat4::translation(-20.0, -20.0, 0.0)).unwrap();
    crate::transform::transform_solid(topo, cutter, &Mat4::rotation_x(TILT)).unwrap();
    crate::transform::transform_solid(topo, cutter, &Mat4::translation(0.0, 0.0, PIVOT)).unwrap();
    let solid =
        crate::boolean::boolean(topo, crate::boolean::BooleanOp::Cut, cylinder, cutter).unwrap();
    // The cutter's base, z = 0 before the rotation, is the cut plane; its
    // rotated +z points out of the kept material.
    let normal = Vec3::new(0.0, -TILT.sin(), TILT.cos());
    (solid, normal, normal.dot(Vec3::new(0.0, 0.0, PIVOT)))
}

fn wall(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<_> = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .collect();
    assert_eq!(walls.len(), 1, "one cylinder wall");
    walls[0]
}

/// `(1/3) ∫ (p − reference) · n dA` over the wall, by quadrature from first
/// principles: the wall is `(R cos u, R sin u, v)` for `0 ≤ v ≤ v_top(u)`, its
/// outward normal `(cos u, sin u, 0)` and `dA = R du dv`.
fn wall_integral_by_quadrature(normal: Vec3, d: f64, reference: Point3) -> f64 {
    const STEPS: u32 = 20_000;
    let step = std::f64::consts::TAU / f64::from(STEPS);
    (0..STEPS)
        .map(|i| {
            let u = f64::from(i) * step;
            let (sin, cos) = u.sin_cos();
            let top = (d - RADIUS * (normal.x() * cos + normal.y() * sin)) / normal.z();
            let reach = RADIUS - reference.x() * cos - reference.y() * sin;
            top * reach * RADIUS * step
        })
        .sum::<f64>()
        / 3.0
}

#[test]
fn ellipse_bounded_wall_integrates_exactly_about_any_reference() {
    let mut topo = Topology::new();
    let (solid, normal, d) = obliquely_cut_cylinder(&mut topo);
    let face = wall(&topo, solid);
    let ellipses = remus_topology::explorer::face_edges(&topo, face)
        .unwrap()
        .into_iter()
        .filter(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Ellipse(_)))
        .count();
    // The seam splits the section into two open arcs.
    assert!(
        ellipses >= 1,
        "the oblique cut bounds the wall with ellipse arcs"
    );
    for reference in [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, -1.5, 2.0),
        Point3::new(-7.0, 4.0, -3.0),
    ] {
        let exact = line_circle_cylinder_signed_volume(&topo, face, reference)
            .expect("an ellipse that is a section of the wall integrates exactly");
        let expected = wall_integral_by_quadrature(normal, d, reference);
        assert!(
            (exact - expected).abs() <= 1e-9 * expected.abs().max(1.0),
            "reference {reference:?}: {exact} vs quadrature {expected}"
        );
    }
}

#[test]
fn obliquely_cut_cylinder_volume_matches_closed_form() {
    let mut topo = Topology::new();
    let (solid, normal, d) = obliquely_cut_cylinder(&mut topo);
    // Cavalieri: every slice normal to the axis is the full disc, so the
    // volume is the disc area times the axial height at the axis.
    let expected = std::f64::consts::PI * RADIUS * RADIUS * (d / normal.z());
    let measured = solid_volume(&topo, solid, 1e-3).unwrap();
    assert!(
        (measured - expected).abs() <= 1e-4 * expected,
        "volume {measured} vs {expected}"
    );
}
