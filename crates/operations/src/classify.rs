//! Point-in-solid classification via ray casting and generalized winding
//! numbers.
//!
//! Determines whether a 3D point is inside, outside, or on the boundary of a
//! solid.
//!
//! Three classifiers are provided:
//! - [`classify_point`]: analytic ray casting. Delegates to
//!   [`remus_check::classify::classify_point`], which is the ground-truth
//!   classifier — exact for every supported surface type, and hole-aware.
//! - [`classify_point_winding`]: a true generalized winding number, summing
//!   signed solid angles over a watertight tessellation of the solid.
//! - [`classify_point_robust`]: winding numbers with a ray-casting fallback in
//!   the ambiguous band.
//!
//! # Why winding lives here and not in `remus-check`
//!
//! A correct winding number needs a triangulation of the solid, and the mesher
//! is L3 (`crate::tessellate`) while `remus-check` is L2. The version in
//! `remus-check` fan-triangulates each face's *boundary loop*, which equals
//! the face only when the face is planar — on curved geometry it is wrong.
//! This module sums solid angles over `tessellate_solid` output instead, which
//! handles cylinders, cones, spheres, tori, and NURBS correctly.

use std::f64::consts::PI;

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::OperationsError;

/// Result of classifying a point relative to a solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointClassification {
    /// The point is inside the solid.
    Inside,
    /// The point is outside the solid.
    Outside,
    /// The point is on the boundary (within tolerance).
    OnBoundary,
}

impl From<remus_check::classify::PointClassification> for PointClassification {
    fn from(c: remus_check::classify::PointClassification) -> Self {
        use remus_check::classify::PointClassification as C;
        match c {
            C::Inside => Self::Inside,
            C::Outside => Self::Outside,
            C::OnBoundary => Self::OnBoundary,
        }
    }
}

/// Winding number above which a point counts as inside.
const INSIDE_THRESHOLD: f64 = 0.5;

/// Winding numbers within `INSIDE_THRESHOLD +/- AMBIGUOUS_BAND` are treated as
/// unreliable by [`classify_point_robust`], which then defers to ray casting.
const AMBIGUOUS_BAND: f64 = 0.1;

/// Classifies a point relative to a solid using analytic ray casting.
///
/// Shoots rays from `point` and counts crossings with the solid's boundary
/// faces, using direct ray-surface intersection for analytic faces (plane,
/// cylinder, cone, sphere, torus) and line-surface intersection for NURBS.
/// Crossings landing inside a face's holes (inner wires) are correctly not
/// counted.
///
/// `deflection` is accepted for API compatibility and ignored: the analytic
/// ray caster needs no tessellation.
/// `tolerance` is the distance threshold for "on boundary" classification.
///
/// # Errors
/// Returns an error if the solid or its faces are invalid.
pub fn classify_point(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    deflection: f64,
    tolerance: f64,
) -> Result<PointClassification, OperationsError> {
    let _ = deflection;
    let options = remus_check::classify::ClassifyOptions {
        tolerance,
        ..Default::default()
    };
    Ok(remus_check::classify::classify_point(topo, solid, point, &options)?.into())
}

/// Generalized winding number of `point` with respect to `solid`.
///
/// Tessellates the solid at `deflection` and sums the signed solid angle each
/// triangle subtends at `point` (Van Oosterom & Strackee). The total, divided
/// by 4*pi, is ~1.0 for interior points and ~0.0 for exterior points.
///
/// Unlike a ray-parity test this degrades gracefully on imperfect meshes: small
/// gaps and T-junctions perturb the sum slightly rather than flipping it.
///
/// # Performance
///
/// Tessellates the solid on **every call**, which costs roughly 80x an analytic
/// [`classify_point`] query (measured ~0.8ms vs ~10us on a 7-face boolean
/// result). Prefer [`classify_point`] for bulk point queries; reach for winding
/// when the mesh may be imperfect and ray parity cannot be trusted.
///
/// # Errors
/// Returns an error if the solid is invalid or cannot be tessellated.
pub fn winding_number(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    deflection: f64,
) -> Result<f64, OperationsError> {
    let mesh = crate::tessellate::tessellate_solid(topo, solid, deflection)?;

    let mut total = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let (a, b, c) = (
            mesh.positions[tri[0] as usize],
            mesh.positions[tri[1] as usize],
            mesh.positions[tri[2] as usize],
        );
        total += solid_angle(point, a, b, c);
    }

    Ok(total / (4.0 * PI))
}

/// Classifies a point using generalized winding numbers.
///
/// `deflection` controls tessellation quality — finer meshes give sharper
/// winding numbers near thin features.
/// `tolerance` is the distance threshold for "on boundary" classification.
///
/// # Errors
/// Returns an error if the solid is invalid or cannot be tessellated.
pub fn classify_point_winding(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    deflection: f64,
    tolerance: f64,
) -> Result<PointClassification, OperationsError> {
    if remus_check::classify::is_point_on_boundary(topo, solid, point, tolerance)? {
        return Ok(PointClassification::OnBoundary);
    }
    let w = winding_number(topo, solid, point, deflection)?;
    Ok(if w > INSIDE_THRESHOLD {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    })
}

/// Robust point classification combining winding numbers and ray casting.
///
/// Uses the winding number when it is decisive, and falls back to analytic ray
/// casting when it lands in the ambiguous band around the 0.5 threshold — the
/// regime where a mesh defect or a probe very close to a thin wall makes the
/// sum untrustworthy.
///
/// # Errors
/// Returns an error if the solid is invalid or cannot be tessellated.
pub fn classify_point_robust(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    deflection: f64,
    tolerance: f64,
) -> Result<PointClassification, OperationsError> {
    if remus_check::classify::is_point_on_boundary(topo, solid, point, tolerance)? {
        return Ok(PointClassification::OnBoundary);
    }

    let w = winding_number(topo, solid, point, deflection)?;
    if w > INSIDE_THRESHOLD + AMBIGUOUS_BAND {
        return Ok(PointClassification::Inside);
    }
    if w < INSIDE_THRESHOLD - AMBIGUOUS_BAND {
        return Ok(PointClassification::Outside);
    }
    classify_point(topo, solid, point, deflection, tolerance)
}

/// Signed solid angle subtended by triangle `(a, b, c)` at `p`, in steradians.
///
/// Van Oosterom & Strackee (1983):
/// `tan(omega/2) = det(a',b',c') / (|a'||b'||c'| + (a'.b')|c'| + (a'.c')|b'| + (b'.c')|a'|)`
/// where `a' = a - p`.
fn solid_angle(p: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    let pa = a - p;
    let pb = b - p;
    let pc = c - p;

    let la = pa.length();
    let lb = pb.length();
    let lc = pc.length();

    // The point coincides with a triangle vertex — contributes nothing.
    if la < 1e-15 || lb < 1e-15 || lc < 1e-15 {
        return 0.0;
    }

    let num = pa.x() * (pb.y() * pc.z() - pb.z() * pc.y())
        + pa.y() * (pb.z() * pc.x() - pb.x() * pc.z())
        + pa.z() * (pb.x() * pc.y() - pb.y() * pc.x());

    let den = la
        .mul_add(lb * lc, pa.dot(pb) * lc)
        .mul_add(1.0, pa.dot(pc) * lb + pb.dot(pc) * la);

    2.0 * num.atan2(den)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::primitives::{self, make_box, make_cone, make_cylinder, make_sphere, make_torus};
    use remus_math::vec::Vec3;
    use remus_topology::face::FaceSurface;

    #[test]
    fn point_inside_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(1.0, 1.0, 1.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn point_outside_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(5.0, 5.0, 5.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_on_boundary_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(1.0, 1.0, 2.0), 0.1, 1e-3).unwrap();
        assert_eq!(result, PointClassification::OnBoundary);
    }

    #[test]
    fn hollow_box_classifiers_treat_cavity_as_outside() {
        let mut topo = Topology::new();
        let outer = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
        let inner = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            inner,
            &remus_math::mat::Mat4::translation(1.0, 1.0, 1.0),
        )
        .unwrap();
        let hollow =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, outer, inner)
                .unwrap();
        assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);

        let cavity = Point3::new(1.5, 1.5, 1.5);
        let material = Point3::new(0.5, 0.5, 0.5);
        for classify in [
            classify_point,
            classify_point_winding,
            classify_point_robust,
        ] {
            assert_eq!(
                classify(&topo, hollow, cavity, 0.01, 1e-7).unwrap(),
                PointClassification::Outside
            );
            assert_eq!(
                classify(&topo, hollow, material, 0.01, 1e-7).unwrap(),
                PointClassification::Inside
            );
        }
    }

    #[test]
    fn point_outside_negative_direction() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result =
            classify_point(&topo, solid, Point3::new(-5.0, -5.0, -5.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_near_corner() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(0.9, 0.9, 0.9), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn point_inside_cylinder() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(0.0, 0.0, 2.5), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn point_outside_cylinder() {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(10.0, 0.0, 2.5), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_inside_sphere() {
        let mut topo = Topology::new();
        let solid = make_sphere(&mut topo, 3.0, 32).unwrap();

        let result = classify_point(&topo, solid, Point3::new(0.0, 0.0, 0.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn point_outside_sphere() {
        let mut topo = Topology::new();
        let solid = make_sphere(&mut topo, 3.0, 32).unwrap();

        let result = classify_point(&topo, solid, Point3::new(5.0, 0.0, 0.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_inside_cone() {
        let mut topo = Topology::new();
        let solid = make_cone(&mut topo, 2.0, 1.0, 5.0).unwrap();

        // Point on the axis, inside the cone
        let result = classify_point(&topo, solid, Point3::new(0.0, 0.0, 2.5), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn point_outside_cone() {
        let mut topo = Topology::new();
        let solid = make_cone(&mut topo, 2.0, 1.0, 5.0).unwrap();

        let result = classify_point(&topo, solid, Point3::new(10.0, 0.0, 2.5), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_inside_torus() {
        let mut topo = Topology::new();
        // major=3, minor=1 → tube center at distance 3 from origin
        let solid = make_torus(&mut topo, 3.0, 1.0, 32).unwrap();

        // Point inside the tube (on the x-axis at distance 3 from origin)
        let result = classify_point(&topo, solid, Point3::new(3.0, 0.0, 0.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn point_outside_torus() {
        let mut topo = Topology::new();
        let solid = make_torus(&mut topo, 3.0, 1.0, 32).unwrap();

        // Point at origin — in the hole of the torus
        let result = classify_point(&topo, solid, Point3::new(0.0, 0.0, 0.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn point_outside_torus_far() {
        let mut topo = Topology::new();
        let solid = make_torus(&mut topo, 3.0, 1.0, 32).unwrap();

        // Point far from torus
        let result = classify_point(&topo, solid, Point3::new(10.0, 0.0, 0.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    /// Build the partial-turn revolve of a circle profile: one trimmed torus
    /// band (wire = 2 closed rims + doubled seam) plus 2 planar disc caps.
    fn make_partial_torus(
        topo: &mut Topology,
        big_r: f64,
        rho: f64,
        angle: f64,
    ) -> remus_topology::solid::SolidId {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::Face;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let circ =
            Circle3D::new(Point3::new(big_r, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), rho).unwrap();
        let p0 = circ.evaluate(0.0);
        let v0 = topo.add_vertex(Vertex::new(p0, 1e-7));
        let eid = topo.add_edge(Edge::new(v0, v0, EdgeCurve::Circle(circ)));
        let wire = Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap();
        let wid = topo.add_wire(wire);
        let face = topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
        ));
        crate::revolve::revolve(
            topo,
            face,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            angle,
        )
        .unwrap()
    }

    /// Regression: the trimmed-torus band of a partial-turn revolve. Two
    /// stacked defects made every interior point read Outside: the local
    /// Ferrari ray-torus quartic missed real roots and emitted off-surface
    /// spurious ones, and the UV boundary sampled closed rim circles from the
    /// curve's parameter origin, so the two rims entered the periodic unwrap
    /// at incoherent phases and the UV polygon rejected real band hits.
    #[test]
    fn partial_turn_torus_band_classification() {
        let (big_r, rho, angle) = (6.0_f64, 2.0_f64, 2.0 * PI / 3.0);
        let mut topo = Topology::new();
        let solid = make_partial_torus(&mut topo, big_r, rho, angle);

        let mid = angle / 2.0;
        let inside = [
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), 0.0),
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), 1.0),
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), -1.0),
            Point3::new(big_r * 0.05f64.cos(), big_r * 0.05f64.sin(), 0.0),
            Point3::new(
                big_r * (angle - 0.05).cos(),
                big_r * (angle - 0.05).sin(),
                0.0,
            ),
            Point3::new((big_r - 1.5) * mid.cos(), (big_r - 1.5) * mid.sin(), 0.0),
            Point3::new((big_r + 1.5) * mid.cos(), (big_r + 1.5) * mid.sin(), 0.0),
        ];
        for p in inside {
            let result = classify_point(&topo, solid, p, 0.05, 1e-6).unwrap();
            assert_eq!(result, PointClassification::Inside, "probe {p:?}");
        }

        let outside = [
            Point3::new(big_r * mid.cos(), big_r * mid.sin(), 2.5),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-big_r, 0.0, 0.0),
            Point3::new(
                big_r * (angle + 0.1).cos(),
                big_r * (angle + 0.1).sin(),
                0.0,
            ),
            Point3::new(big_r * (-0.1f64).cos(), big_r * (-0.1f64).sin(), 0.0),
        ];
        for p in outside {
            let result = classify_point(&topo, solid, p, 0.05, 1e-6).unwrap();
            assert_eq!(result, PointClassification::Outside, "probe {p:?}");
        }
    }

    /// A full-turn revolve (single closed torus face, seam edges only) must
    /// keep classifying correctly alongside the partial-band fix.
    #[test]
    fn full_turn_torus_classification() {
        let (big_r, rho) = (6.0_f64, 2.0_f64);
        let mut topo = Topology::new();
        let solid = make_partial_torus(&mut topo, big_r, rho, 2.0 * PI);

        for theta in [0.0_f64, 1.0, 2.5, 4.0, 5.5] {
            let p = Point3::new(big_r * theta.cos(), big_r * theta.sin(), 0.0);
            let result = classify_point(&topo, solid, p, 0.05, 1e-6).unwrap();
            assert_eq!(result, PointClassification::Inside, "tube center {theta}");
        }
        for p in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(big_r, 0.0, 2.5),
            Point3::new(2.0 * big_r, 0.0, 0.0),
        ] {
            let result = classify_point(&topo, solid, p, 0.05, 1e-6).unwrap();
            assert_eq!(result, PointClassification::Outside, "probe {p:?}");
        }
    }

    #[test]
    fn winding_point_inside_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result =
            classify_point_winding(&topo, solid, Point3::new(1.0, 1.0, 1.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn winding_point_outside_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result =
            classify_point_winding(&topo, solid, Point3::new(5.0, 5.0, 5.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn robust_point_inside_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result =
            classify_point_robust(&topo, solid, Point3::new(1.0, 1.0, 1.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Inside);
    }

    #[test]
    fn robust_point_outside_box() {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

        let result =
            classify_point_robust(&topo, solid, Point3::new(5.0, 5.0, 5.0), 0.1, 1e-6).unwrap();
        assert_eq!(result, PointClassification::Outside);
    }

    #[test]
    fn winding_handles_curved_faces() {
        let mut topo = Topology::new();
        let cyl = primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();

        for p in [
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(9.0, 0.0, 1.0),
            Point3::new(0.0, -9.0, 19.0),
            Point3::new(5.0, 5.0, 10.0),
        ] {
            let w = winding_number(&topo, cyl, p, 0.05).unwrap();
            assert!(w > 0.9, "interior point {p:?} has winding {w}");
            assert_eq!(
                classify_point_winding(&topo, cyl, p, 0.05, 1e-6).unwrap(),
                PointClassification::Inside,
                "probe {p:?}"
            );
        }

        for p in [
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(11.0, 0.0, 10.0),
            Point3::new(0.0, 0.0, 21.0),
            Point3::new(20.0, 20.0, 10.0),
        ] {
            let w = winding_number(&topo, cyl, p, 0.05).unwrap();
            assert!(w < 0.1, "exterior point {p:?} has winding {w}");
            assert_eq!(
                classify_point_winding(&topo, cyl, p, 0.05, 1e-6).unwrap(),
                PointClassification::Outside,
                "probe {p:?}"
            );
        }
    }

    #[test]
    fn winding_handles_sphere() {
        let mut topo = Topology::new();
        let sph = primitives::make_sphere(&mut topo, 8.0, 32).unwrap();

        let w_in = winding_number(&topo, sph, Point3::new(0.0, 0.0, 0.0), 0.05).unwrap();
        assert!(w_in > 0.9, "sphere centre winding {w_in}");
        let w_out = winding_number(&topo, sph, Point3::new(20.0, 0.0, 0.0), 0.05).unwrap();
        assert!(w_out < 0.1, "point outside sphere winding {w_out}");
    }

    #[test]
    fn on_boundary_detected() {
        let mut topo = Topology::new();
        let cyl = primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();
        // Dead centre of the z=0 cap.
        let p = Point3::new(0.0, 0.0, 0.0);
        assert_eq!(
            classify_point_winding(&topo, cyl, p, 0.05, 1e-6).unwrap(),
            PointClassification::OnBoundary
        );
        assert_eq!(
            classify_point_robust(&topo, cyl, p, 0.05, 1e-6).unwrap(),
            PointClassification::OnBoundary
        );
    }
}
