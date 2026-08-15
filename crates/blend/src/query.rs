//! Read-only analytic blend queries for higher-level modeling operations.

use brepkit_math::curves::Circle3D;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::vertex::Vertex;

/// Exact topology-independent geometry for a single blend spine.
///
/// The enum is non-exhaustive so future exact conic representations can be
/// added without exposing the blend engine's private topology types.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum GeometricSpine {
    /// A non-degenerate straight segment.
    Line {
        /// Segment start point.
        start: Point3,
        /// Segment end point.
        end: Point3,
    },
    /// A complete circle.
    Circle(Circle3D),
}

/// Re-derive only the analytic carrier surface for a fillet candidate.
///
/// This deliberately hides the construction-only `Stripe` topology recipe.
/// Direct-edit recognition can compare the returned carrier
/// with an existing face without depending on the builder's private assembly
/// types. Face surfaces are oriented toward the material exactly as they are
/// by the fillet builder.
///
/// `None` means this exact query cannot certify the requested support/spine
/// combination. It never routes through the approximate walking engine.
///
/// # Errors
///
/// Returns [`crate::BlendError::InvalidInput`] for non-finite geometry, a
/// non-positive radius, identical/coincident supports, or another malformed
/// request. Other errors report topology or analytic construction failures.
pub fn try_analytic_fillet_surface(
    topo: &Topology,
    face1: FaceId,
    face2: FaceId,
    spine: &GeometricSpine,
    radius: f64,
) -> Result<Option<FaceSurface>, crate::BlendError> {
    if !radius.is_finite() || radius <= 0.0 || radius > f64::MAX.sqrt() {
        return Err(invalid_input(
            "radius must be finite, positive, and safe for squared geometry",
        ));
    }
    if face1 == face2 {
        return Err(invalid_input("support faces must be distinct"));
    }

    let face1_data = topo.face(face1)?;
    let face2_data = topo.face(face2)?;
    validate_surface(face1_data.surface())?;
    validate_surface(face2_data.surface())?;
    if !supports_analytic_pair(face1_data.surface(), face2_data.surface()) {
        return Ok(None);
    }

    let tolerance = Tolerance::new();
    if same_carrier(face1_data.surface(), face2_data.surface(), tolerance) {
        return Err(invalid_input(
            "support faces must have distinct carrier surfaces",
        ));
    }
    if !spine_lies_on_surface(spine, face1_data.surface(), tolerance)?
        || !spine_lies_on_surface(spine, face2_data.surface(), tolerance)?
    {
        return Ok(None);
    }

    // The private analytic engine still consumes a topological Spine and, for
    // plane-cylinder cases, inspects the real support faces. Materialize only
    // the synthetic spine in a snapshot so this public query remains read-only.
    let mut scratch = topo.clone();
    let spine_edge = materialize_spine(&mut scratch, spine, tolerance);
    let private_spine = crate::spine::Spine::from_single_edge(&scratch, spine_edge)?;
    let scratch_face1 = scratch.face(face1)?;
    let surface1 = inward_surface(scratch_face1.surface(), scratch_face1.is_reversed());
    let scratch_face2 = scratch.face(face2)?;
    let surface2 = inward_surface(scratch_face2.surface(), scratch_face2.is_reversed());

    let result = crate::analytic::try_analytic_fillet(
        &surface1,
        &surface2,
        &private_spine,
        &scratch,
        radius,
        face1,
        face2,
    );
    match result {
        Ok(Some(stripe)) => {
            validate_surface(&stripe.stripe.surface)?;
            Ok(Some(stripe.stripe.surface))
        }
        Ok(None) => Ok(None),
        Err(crate::BlendError::RadiusTooLarge { max_radius, .. }) => Err(invalid_input(format!(
            "radius exceeds the maximum {max_radius} for these supports"
        ))),
        Err(error) => Err(error),
    }
}

fn invalid_input(reason: impl Into<String>) -> crate::BlendError {
    crate::BlendError::InvalidInput {
        reason: reason.into(),
    }
}

fn validate_surface(surface: &FaceSurface) -> Result<(), crate::BlendError> {
    let tolerance = Tolerance::new();
    let valid = match surface {
        FaceSurface::Plane { normal, d } => {
            vector_finite(*normal)
                && d.is_finite()
                && (normal.length() - 1.0).abs() <= tolerance.angular
        }
        FaceSurface::Cylinder(cylinder) => {
            point_finite(cylinder.origin())
                && orthonormal_frame(
                    cylinder.x_axis(),
                    cylinder.y_axis(),
                    cylinder.axis(),
                    tolerance,
                )
                && cylinder.radius().is_finite()
                && cylinder.radius() > 0.0
        }
        FaceSurface::Cone(cone) => {
            point_finite(cone.apex())
                && orthonormal_frame(cone.x_axis(), cone.y_axis(), cone.axis(), tolerance)
                && cone.half_angle().is_finite()
                && cone.half_angle() > 0.0
                && cone.half_angle() < std::f64::consts::FRAC_PI_2
        }
        FaceSurface::Sphere(sphere) => {
            point_finite(sphere.center())
                && sphere.radius().is_finite()
                && sphere.radius() > 0.0
                && orthonormal_frame(sphere.x_axis(), sphere.y_axis(), sphere.z_axis(), tolerance)
        }
        FaceSurface::Torus(torus) => {
            point_finite(torus.center())
                && torus.major_radius().is_finite()
                && torus.major_radius() > 0.0
                && torus.minor_radius().is_finite()
                && torus.minor_radius() > 0.0
                && orthonormal_frame(torus.x_axis(), torus.y_axis(), torus.z_axis(), tolerance)
        }
        FaceSurface::Nurbs(_) => true,
    };
    if valid {
        Ok(())
    } else {
        Err(invalid_input(format!(
            "{} support/carrier has a non-finite or non-canonical frame",
            surface.type_tag()
        )))
    }
}

fn orthonormal_frame(x: Vec3, y: Vec3, z: Vec3, tolerance: Tolerance) -> bool {
    vector_finite(x)
        && vector_finite(y)
        && vector_finite(z)
        && (x.length() - 1.0).abs() <= tolerance.angular
        && (y.length() - 1.0).abs() <= tolerance.angular
        && (z.length() - 1.0).abs() <= tolerance.angular
        && x.dot(y).abs() <= tolerance.angular
        && x.dot(z).abs() <= tolerance.angular
        && y.dot(z).abs() <= tolerance.angular
        && x.cross(y).dot(z) >= 1.0 - tolerance.angular
}

fn supports_analytic_pair(a: &FaceSurface, b: &FaceSurface) -> bool {
    matches!(
        (a, b),
        (
            FaceSurface::Plane { .. },
            FaceSurface::Plane { .. }
                | FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_),
        ) | (
            FaceSurface::Cylinder(_),
            FaceSurface::Plane { .. } | FaceSurface::Cylinder(_) | FaceSurface::Sphere(_),
        ) | (
            FaceSurface::Cone(_),
            FaceSurface::Plane { .. } | FaceSurface::Cone(_) | FaceSurface::Sphere(_),
        ) | (
            FaceSurface::Sphere(_),
            FaceSurface::Plane { .. }
                | FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_),
        )
    )
}

fn same_carrier(a: &FaceSurface, b: &FaceSurface, tolerance: Tolerance) -> bool {
    match (a, b) {
        (FaceSurface::Plane { normal: na, d: da }, FaceSurface::Plane { normal: nb, d: db }) => {
            let alignment = na.dot(*nb);
            alignment.abs() >= 1.0 - tolerance.angular
                && tolerance.approx_eq(*da, db * alignment.signum())
        }
        (FaceSurface::Cylinder(a), FaceSurface::Cylinder(b)) => {
            tolerance.approx_eq(a.radius(), b.radius())
                && a.axis().dot(b.axis()).abs() >= 1.0 - tolerance.angular
                && axis_distance(a.origin(), b.origin(), a.axis()) <= tolerance.linear
        }
        (FaceSurface::Cone(a), FaceSurface::Cone(b)) => {
            tolerance.approx_eq_abs(a.half_angle(), b.half_angle())
                && a.axis().dot(b.axis()) >= 1.0 - tolerance.angular
                && (a.apex() - b.apex()).length() <= tolerance.linear
        }
        (FaceSurface::Sphere(a), FaceSurface::Sphere(b)) => {
            tolerance.approx_eq(a.radius(), b.radius())
                && (a.center() - b.center()).length() <= tolerance.linear
        }
        _ => false,
    }
}

fn spine_lies_on_surface(
    spine: &GeometricSpine,
    surface: &FaceSurface,
    tolerance: Tolerance,
) -> Result<bool, crate::BlendError> {
    match spine {
        GeometricSpine::Line { start, end } => {
            if !point_finite(*start) || !point_finite(*end) {
                return Err(invalid_input("line spine points must be finite"));
            }
            let direction = *end - *start;
            let length = direction.length();
            if !length.is_finite() || length <= tolerance.linear {
                return Err(invalid_input("line spine must be non-degenerate"));
            }
            Ok(line_lies_on_surface(
                *start,
                *end,
                direction * (1.0 / length),
                surface,
                tolerance,
            ))
        }
        GeometricSpine::Circle(circle) => {
            validate_circle(circle, tolerance)?;
            Ok(circle_lies_on_surface(circle, surface, tolerance))
        }
    }
}

fn validate_circle(circle: &Circle3D, tolerance: Tolerance) -> Result<(), crate::BlendError> {
    let unit = |vector: Vec3| (vector.length() - 1.0).abs() <= tolerance.angular;
    let frame_valid = point_finite(circle.center())
        && circle.radius().is_finite()
        && circle.radius() > tolerance.linear
        && vector_finite(circle.normal())
        && vector_finite(circle.u_axis())
        && vector_finite(circle.v_axis())
        && unit(circle.normal())
        && unit(circle.u_axis())
        && unit(circle.v_axis())
        && circle.normal().dot(circle.u_axis()).abs() <= tolerance.angular
        && circle.normal().dot(circle.v_axis()).abs() <= tolerance.angular
        && circle.u_axis().dot(circle.v_axis()).abs() <= tolerance.angular
        && circle.u_axis().cross(circle.v_axis()).dot(circle.normal()) >= 1.0 - tolerance.angular;
    if frame_valid {
        Ok(())
    } else {
        Err(invalid_input(
            "circle spine must have finite geometry and an orthonormal right-handed frame",
        ))
    }
}

fn line_lies_on_surface(
    start: Point3,
    end: Point3,
    direction: Vec3,
    surface: &FaceSurface,
    tolerance: Tolerance,
) -> bool {
    let midpoint = start + (end - start) * 0.5;
    match surface {
        FaceSurface::Plane { normal, .. } => {
            point_lies_on_surface(start, surface, tolerance)
                && point_lies_on_surface(end, surface, tolerance)
                && direction.dot(*normal).abs() <= tolerance.angular
        }
        FaceSurface::Cylinder(cylinder) => {
            point_lies_on_surface(start, surface, tolerance)
                && point_lies_on_surface(midpoint, surface, tolerance)
                && point_lies_on_surface(end, surface, tolerance)
                && direction.dot(cylinder.axis()).abs() >= 1.0 - tolerance.angular
        }
        FaceSurface::Cone(cone) => {
            let to_apex = cone.apex() - start;
            let distance_to_line = (to_apex - direction * to_apex.dot(direction)).length();
            distance_to_line <= tolerance.linear
                && point_lies_on_surface(start, surface, tolerance)
                && point_lies_on_surface(midpoint, surface, tolerance)
                && point_lies_on_surface(end, surface, tolerance)
        }
        FaceSurface::Sphere(_) | FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => false,
    }
}

fn point_lies_on_surface(point: Point3, surface: &FaceSurface, tolerance: Tolerance) -> bool {
    match surface {
        FaceSurface::Plane { normal, d } => (dot(*normal, point) - d).abs() <= tolerance.linear,
        FaceSurface::Cylinder(cylinder) => {
            (axis_distance(cylinder.origin(), point, cylinder.axis()) - cylinder.radius()).abs()
                <= tolerance.linear
        }
        FaceSurface::Cone(cone) => {
            let offset = point - cone.apex();
            let height = offset.dot(cone.axis());
            let radial = (offset - cone.axis() * height).length();
            height >= -tolerance.linear
                && (radial - height * cone.half_angle().cos() / cone.half_angle().sin()).abs()
                    <= tolerance.linear
        }
        FaceSurface::Sphere(sphere) => {
            ((point - sphere.center()).length() - sphere.radius()).abs() <= tolerance.linear
        }
        FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => false,
    }
}

fn circle_lies_on_surface(circle: &Circle3D, surface: &FaceSurface, tolerance: Tolerance) -> bool {
    match surface {
        FaceSurface::Plane { normal, d } => {
            normal.dot(circle.normal()).abs() >= 1.0 - tolerance.angular
                && (dot(*normal, circle.center()) - d).abs() <= tolerance.linear
        }
        FaceSurface::Cylinder(cylinder) => {
            circle.normal().dot(cylinder.axis()).abs() >= 1.0 - tolerance.angular
                && axis_distance(cylinder.origin(), circle.center(), cylinder.axis())
                    <= tolerance.linear
                && tolerance.approx_eq(circle.radius(), cylinder.radius())
        }
        FaceSurface::Cone(cone) => {
            let offset = circle.center() - cone.apex();
            let height = offset.dot(cone.axis());
            circle.normal().dot(cone.axis()).abs() >= 1.0 - tolerance.angular
                && (offset - cone.axis() * height).length() <= tolerance.linear
                && height >= -tolerance.linear
                && tolerance.approx_eq(
                    circle.radius(),
                    height * cone.half_angle().cos() / cone.half_angle().sin(),
                )
        }
        FaceSurface::Sphere(sphere) => {
            let offset = circle.center() - sphere.center();
            let axial = offset.dot(circle.normal());
            (offset - circle.normal() * axial).length() <= tolerance.linear
                && tolerance.approx_eq(circle.radius().hypot(axial), sphere.radius())
        }
        FaceSurface::Torus(_) | FaceSurface::Nurbs(_) => false,
    }
}

fn materialize_spine(topo: &mut Topology, spine: &GeometricSpine, tolerance: Tolerance) -> EdgeId {
    let (curve, start, end) = match spine {
        GeometricSpine::Line { start, end } => (EdgeCurve::Line, *start, *end),
        GeometricSpine::Circle(circle) => {
            let seam = circle.evaluate(0.0);
            (EdgeCurve::Circle(circle.clone()), seam, seam)
        }
    };
    let start_vertex = topo.add_vertex(Vertex::new(start, tolerance.linear));
    let end_vertex = if (start - end).length() <= tolerance.linear {
        start_vertex
    } else {
        topo.add_vertex(Vertex::new(end, tolerance.linear))
    };
    topo.add_edge(Edge::new(start_vertex, end_vertex, curve))
}

fn axis_distance(origin: Point3, point: Point3, axis: Vec3) -> f64 {
    let offset = point - origin;
    (offset - axis * offset.dot(axis)).length()
}

fn point_finite(point: Point3) -> bool {
    point.x().is_finite() && point.y().is_finite() && point.z().is_finite()
}

fn vector_finite(vector: Vec3) -> bool {
    vector.x().is_finite() && vector.y().is_finite() && vector.z().is_finite()
}

fn dot(vector: Vec3, point: Point3) -> f64 {
    vector.x() * point.x() + vector.y() * point.y() + vector.z() * point.z()
}

fn inward_surface(surface: &FaceSurface, is_reversed: bool) -> FaceSurface {
    if is_reversed {
        return surface.clone();
    }
    match surface {
        FaceSurface::Plane { normal, d } => FaceSurface::Plane {
            normal: -*normal,
            d: -*d,
        },
        other => other.clone(),
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use brepkit_math::curves::Circle3D;
    use brepkit_math::vec::Vec3;
    use brepkit_topology::explorer::{solid_edges, solid_faces};
    use brepkit_topology::test_utils::make_unit_cube_manifold;

    fn adjacent_faces(
        topo: &Topology,
        solid: brepkit_topology::solid::SolidId,
    ) -> (FaceId, FaceId, GeometricSpine) {
        let edge = solid_edges(topo, solid).unwrap()[0];
        let edge_data = topo.edge(edge).unwrap();
        let faces = topo
            .build_adjacency(solid)
            .unwrap()
            .faces_for_edge(edge)
            .to_vec();
        let spine = GeometricSpine::Line {
            start: topo.vertex(edge_data.start()).unwrap().point(),
            end: topo.vertex(edge_data.end()).unwrap().point(),
        };
        (faces[0], faces[1], spine)
    }

    #[test]
    fn rejects_invalid_radius_and_identical_supports() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let (face1, face2, spine) = adjacent_faces(&topo, solid);

        for radius in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::MAX] {
            assert!(matches!(
                try_analytic_fillet_surface(&topo, face1, face2, &spine, radius),
                Err(crate::BlendError::InvalidInput { .. })
            ));
        }
        assert!(matches!(
            try_analytic_fillet_surface(&topo, face1, face1, &spine, 0.1),
            Err(crate::BlendError::InvalidInput { .. })
        ));
    }

    #[test]
    fn rejects_distinct_faces_on_the_same_carrier() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let faces = solid_faces(&topo, solid).unwrap();
        let original = topo.face(faces[0]).unwrap().clone();
        let duplicate = topo.add_face(original);
        let edge = solid_edges(&topo, solid).unwrap()[0];
        let edge_data = topo.edge(edge).unwrap();
        let spine = GeometricSpine::Line {
            start: topo.vertex(edge_data.start()).unwrap().point(),
            end: topo.vertex(edge_data.end()).unwrap().point(),
        };

        assert!(matches!(
            try_analytic_fillet_surface(&topo, faces[0], duplicate, &spine, 0.1),
            Err(crate::BlendError::InvalidInput { .. })
        ));
    }

    #[test]
    fn off_carrier_spines_fail_closed() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let (face1, face2, _) = adjacent_faces(&topo, solid);
        let spine = GeometricSpine::Line {
            start: Point3::new(0.0, 0.0, 0.0),
            end: Point3::new(999.0, 999.0, 1.0e15),
        };

        assert!(
            try_analytic_fillet_surface(&topo, face1, face2, &spine, 0.1)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn malformed_plane_and_circle_frames_are_rejected() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let (face1, face2, spine) = adjacent_faces(&topo, solid);
        let original = topo.face(face1).unwrap().surface().clone();
        let malformed = match &original {
            FaceSurface::Plane { normal, d } => FaceSurface::Plane {
                normal: *normal * 2.0,
                d: d * 2.0,
            },
            _ => unreachable!(),
        };
        topo.face_mut(face1).unwrap().set_surface(malformed);
        assert!(matches!(
            try_analytic_fillet_surface(&topo, face1, face2, &spine, 0.1),
            Err(crate::BlendError::InvalidInput { .. })
        ));
        topo.face_mut(face1).unwrap().set_surface(original);

        let malformed_circle = Circle3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        )
        .unwrap();
        let spine = GeometricSpine::Circle(malformed_circle);
        assert!(matches!(
            try_analytic_fillet_surface(&topo, face1, face2, &spine, 0.1),
            Err(crate::BlendError::InvalidInput { .. })
        ));
    }

    #[test]
    fn valid_line_spine_rederives_a_cylinder() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let (face1, face2, spine) = adjacent_faces(&topo, solid);

        assert!(matches!(
            try_analytic_fillet_surface(&topo, face1, face2, &spine, 0.1).unwrap(),
            Some(FaceSurface::Cylinder(cylinder)) if (cylinder.radius() - 0.1).abs() < 1e-12
        ));
    }

    #[test]
    fn closed_circle_must_lie_on_both_supports() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let (face1, face2, _) = adjacent_faces(&topo, solid);
        let circle =
            Circle3D::new(Point3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let spine = GeometricSpine::Circle(circle);

        assert!(
            try_analytic_fillet_surface(&topo, face1, face2, &spine, 0.1)
                .unwrap()
                .is_none()
        );
    }
}
